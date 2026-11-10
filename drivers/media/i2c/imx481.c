// SPDX-License-Identifier: GPL-2.0
/*
 * Sony IMX481 image sensor driver
 *
 * The IMX481 is a 16 Mpixel (4656x3496) Bayer sensor with a four-lane MIPI
 * CSI-2 D-PHY output and an SMIA++ style register map. The standard
 * registers (mode select, frame and line length, integration time, gains,
 * orientation, test pattern) are driven from here. The global settings,
 * the PLL and D-PHY timing settings and the two readout modes come from the
 * GPL-2.0 MediaTek imgsensor driver for this sensor; the OTP identification
 * sequence is based on the same driver.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define IMX481_REG_MODE_SELECT		CCI_REG8(0x0100)
#define IMX481_MODE_STANDBY		0x00
#define IMX481_MODE_STREAMING		0x01

#define IMX481_REG_ORIENTATION		CCI_REG8(0x0101)
#define IMX481_ORIENTATION_HFLIP	BIT(0)
#define IMX481_ORIENTATION_VFLIP	BIT(1)

#define IMX481_REG_CHIP_ID		CCI_REG16(0x0016)
#define IMX481_CHIP_ID			0x0481

/* Coarse integration time, in lines */
#define IMX481_REG_EXPOSURE		CCI_REG16(0x0202)
#define IMX481_EXPOSURE_MIN		4
#define IMX481_EXPOSURE_STEP		1
/* Lines between the end of the integration time and the frame length */
#define IMX481_EXPOSURE_MARGIN		18

/* Analogue gain = 1024 / (1024 - code), 1x to 16x */
#define IMX481_REG_ANALOG_GAIN		CCI_REG16(0x0204)
#define IMX481_ANA_GAIN_MIN		0
#define IMX481_ANA_GAIN_MAX		960
#define IMX481_ANA_GAIN_STEP		1
#define IMX481_ANA_GAIN_DEFAULT		IMX481_ANA_GAIN_MIN

/* Digital gain per colour channel, U4.8 */
#define IMX481_REG_DIG_GAIN_GR		CCI_REG16(0x020e)
#define IMX481_REG_DIG_GAIN_R		CCI_REG16(0x0210)
#define IMX481_REG_DIG_GAIN_B		CCI_REG16(0x0212)
#define IMX481_REG_DIG_GAIN_GB		CCI_REG16(0x0214)
#define IMX481_DGTL_GAIN_MIN		0x0100
#define IMX481_DGTL_GAIN_MAX		0x0fff
#define IMX481_DGTL_GAIN_STEP		1
#define IMX481_DGTL_GAIN_DEFAULT	0x0100

#define IMX481_REG_FRAME_LENGTH		CCI_REG16(0x0340)
#define IMX481_FRAME_LENGTH_MAX		0xffff
#define IMX481_REG_LINE_LENGTH		CCI_REG16(0x0342)

#define IMX481_REG_TEST_PATTERN		CCI_REG16(0x0600)
#define IMX481_REG_TEST_DATA_RED	CCI_REG16(0x0602)
#define IMX481_REG_TEST_DATA_GREENR	CCI_REG16(0x0604)
#define IMX481_REG_TEST_DATA_BLUE	CCI_REG16(0x0606)
#define IMX481_REG_TEST_DATA_GREENB	CCI_REG16(0x0608)
#define IMX481_TEST_DATA_MAX		0x03ff

/* OTP read window */
#define IMX481_REG_OTP_CTRL		CCI_REG8(0x0a00)
#define IMX481_OTP_CTRL_READ		0x01
#define IMX481_REG_OTP_STATUS		CCI_REG8(0x0a01)
#define IMX481_OTP_STATUS_READY		BIT(0)
#define IMX481_REG_OTP_PAGE		CCI_REG8(0x0a02)
#define IMX481_OTP_ID_PAGE		0x1b
/* The chip ID is the top 12 bits of the 16-bit word at 0x0a22 */
#define IMX481_REG_OTP_ID		CCI_REG16(0x0a22)
#define IMX481_OTP_ID_SHIFT		4

#define IMX481_MCLK_RATE		24000000

/* Delay from XCLR release to the first I2C access */
#define IMX481_XCLR_DELAY_US		10000

#define IMX481_DATA_LANES		4

/*
 * PLL, from a 24 MHz INCK:
 *  - video timing: 24 MHz / 4 * 290 = 1740 MHz, / 6 = 290 MHz, with two
 *    pixels per clock a 580 Mpixel/s pixel rate. At the modes' 5120-pixel
 *    line and 3776-line frame this gives 30.0 fps.
 *  - output: 24 MHz / 2 * 153 = 1836 Mbps per lane, a 918 MHz link
 *    frequency.
 * Both modes use the same PLL settings.
 */
#define IMX481_PIXEL_RATE		580000000
#define IMX481_LINK_FREQ_918MHZ		918000000LL

/* Pixel array */
#define IMX481_NATIVE_WIDTH		4656
#define IMX481_NATIVE_HEIGHT		3496
#define IMX481_PIXEL_ARRAY_LEFT		0
#define IMX481_PIXEL_ARRAY_TOP		0
#define IMX481_PIXEL_ARRAY_WIDTH	4656
#define IMX481_PIXEL_ARRAY_HEIGHT	3496

static const s64 imx481_link_freqs[] = {
	IMX481_LINK_FREQ_918MHZ,
};

/* Global settings, written once after every power up */
static const struct cci_reg_sequence imx481_init_regs[] = {
	/* External clock: 24 MHz */
	{ CCI_REG16(0x0136), 0x1800 },
	/* Register version */
	{ CCI_REG8(0x3c7e), 0x02 },
	{ CCI_REG8(0x3c7f), 0x07 },
	/* Global settings */
	{ CCI_REG8(0x3f7f), 0x01 },
	{ CCI_REG8(0x531c), 0x01 },
	{ CCI_REG8(0x531d), 0x02 },
	{ CCI_REG8(0x531e), 0x04 },
	{ CCI_REG8(0x5928), 0x00 },
	{ CCI_REG8(0x5929), 0x2f },
	{ CCI_REG8(0x592a), 0x00 },
	{ CCI_REG8(0x592b), 0x85 },
	{ CCI_REG8(0x592c), 0x00 },
	{ CCI_REG8(0x592d), 0x32 },
	{ CCI_REG8(0x592e), 0x00 },
	{ CCI_REG8(0x592f), 0x88 },
	{ CCI_REG8(0x5930), 0x00 },
	{ CCI_REG8(0x5931), 0x3d },
	{ CCI_REG8(0x5932), 0x00 },
	{ CCI_REG8(0x5933), 0x93 },
	{ CCI_REG8(0x5938), 0x00 },
	{ CCI_REG8(0x5939), 0x24 },
	{ CCI_REG8(0x593a), 0x00 },
	{ CCI_REG8(0x593b), 0x7a },
	{ CCI_REG8(0x593c), 0x00 },
	{ CCI_REG8(0x593d), 0x24 },
	{ CCI_REG8(0x593e), 0x00 },
	{ CCI_REG8(0x593f), 0x7a },
	{ CCI_REG8(0x5940), 0x00 },
	{ CCI_REG8(0x5941), 0x2f },
	{ CCI_REG8(0x5942), 0x00 },
	{ CCI_REG8(0x5943), 0x85 },
	{ CCI_REG8(0x5f0e), 0x6e },
	{ CCI_REG8(0x5f11), 0xc6 },
	{ CCI_REG8(0x5f17), 0x5e },
	{ CCI_REG8(0x7990), 0x01 },
	{ CCI_REG8(0x7993), 0x5d },
	{ CCI_REG8(0x7994), 0x5d },
	{ CCI_REG8(0x7995), 0xa1 },
	{ CCI_REG8(0x799a), 0x01 },
	{ CCI_REG8(0x799d), 0x00 },
	{ CCI_REG8(0x8169), 0x01 },
	{ CCI_REG8(0x8359), 0x01 },
	{ CCI_REG8(0x88c7), 0x00 },
	{ CCI_REG8(0x88d4), 0x03 },
	{ CCI_REG8(0x9302), 0x1e },
	{ CCI_REG8(0x9306), 0x1f },
	{ CCI_REG8(0x930a), 0x26 },
	{ CCI_REG8(0x930e), 0x23 },
	{ CCI_REG8(0x9312), 0x23 },
	{ CCI_REG8(0x9316), 0x2c },
	{ CCI_REG8(0x9317), 0x19 },
	{ CCI_REG8(0x9960), 0x00 },
	{ CCI_REG8(0x9963), 0x64 },
	{ CCI_REG8(0x9964), 0x50 },
	{ CCI_REG8(0xa391), 0x04 },
	{ CCI_REG8(0xb046), 0x01 },
	{ CCI_REG8(0xb048), 0x01 },
	/* Image quality adjustment */
	{ CCI_REG8(0x8145), 0x00 },
	{ CCI_REG8(0x8146), 0x04 },
	{ CCI_REG8(0x8341), 0x00 },
	{ CCI_REG8(0x8343), 0x08 },
	{ CCI_REG8(0xa801), 0x00 },
	{ CCI_REG8(0xa802), 0x00 },
	{ CCI_REG8(0xa903), 0x00 },
	{ CCI_REG8(0xa905), 0x00 },
	{ CCI_REG8(0xa909), 0x00 },
	{ CCI_REG8(0xa90b), 0x00 },
	{ CCI_REG8(0xa925), 0x02 },
	{ CCI_REG8(0xa927), 0x02 },
	{ CCI_REG8(0xa929), 0x02 },
	{ CCI_REG8(0xa92b), 0x00 },
	{ CCI_REG8(0xa92d), 0x00 },
	{ CCI_REG8(0xa92f), 0x00 },
	{ CCI_REG8(0xa933), 0x00 },
	{ CCI_REG8(0xa935), 0x00 },
	{ CCI_REG8(0xa939), 0x00 },
	{ CCI_REG8(0xa93b), 0x00 },
	{ CCI_REG8(0xa955), 0x02 },
	{ CCI_REG8(0xa957), 0x02 },
	{ CCI_REG8(0xa959), 0x02 },
	{ CCI_REG8(0xa95b), 0x00 },
	{ CCI_REG8(0xa95d), 0x00 },
	{ CCI_REG8(0xa95f), 0x00 },
	{ CCI_REG8(0xa963), 0x00 },
	{ CCI_REG8(0xa965), 0x00 },
	{ CCI_REG8(0xa969), 0x00 },
	{ CCI_REG8(0xa96b), 0x00 },
	{ CCI_REG8(0xa985), 0x02 },
	{ CCI_REG8(0xa987), 0x02 },
	{ CCI_REG8(0xa989), 0x02 },
	{ CCI_REG8(0xa98b), 0x00 },
	{ CCI_REG8(0xa98d), 0x00 },
	{ CCI_REG8(0xa98f), 0x00 },
	{ CCI_REG8(0xaa06), 0x3f },
	{ CCI_REG8(0xaa07), 0x05 },
	{ CCI_REG8(0xaa08), 0x04 },
	{ CCI_REG8(0xaa12), 0x3f },
	{ CCI_REG8(0xaa13), 0x04 },
	{ CCI_REG8(0xaa14), 0x03 },
	{ CCI_REG8(0xab55), 0x02 },
	{ CCI_REG8(0xab57), 0x01 },
	{ CCI_REG8(0xab59), 0x01 },
	{ CCI_REG8(0xabb4), 0x00 },
	{ CCI_REG8(0xabb5), 0x01 },
	{ CCI_REG8(0xabb6), 0x00 },
	{ CCI_REG8(0xabb7), 0x01 },
	{ CCI_REG8(0xabb8), 0x00 },
	{ CCI_REG8(0xabb9), 0x01 },
	{ CCI_REG8(0xae08), 0x00 },
	{ CCI_REG8(0xae0b), 0x00 },
	{ CCI_REG8(0xae0e), 0x00 },
	{ CCI_REG8(0xae11), 0x00 },
	{ CCI_REG8(0xae14), 0x00 },
	{ CCI_REG8(0xae1a), 0x00 },
	{ CCI_REG8(0xae2e), 0x00 },
	{ CCI_REG8(0xae31), 0x00 },
	{ CCI_REG8(0xae37), 0x00 },
	{ CCI_REG8(0xae40), 0x00 },
	{ CCI_REG8(0xae54), 0x00 },
	{ CCI_REG8(0xae57), 0x00 },
	{ CCI_REG8(0xae5d), 0x00 },
	{ CCI_REG8(0xae66), 0x00 },

	/* Settings the two modes share */
	/* CSI-2 output: RAW10 on four lanes */
	{ CCI_REG16(0x0112), 0x0a0a },
	{ CCI_REG8(0x0114), IMX481_DATA_LANES - 1 },
	/* Analogue crop: the whole pixel array */
	{ CCI_REG16(0x0344), 0 },
	{ CCI_REG16(0x0346), 0 },
	{ CCI_REG16(0x0348), IMX481_PIXEL_ARRAY_WIDTH - 1 },
	{ CCI_REG16(0x034a), IMX481_PIXEL_ARRAY_HEIGHT - 1 },
	/* No subsampling */
	{ CCI_REG8(0x0381), 0x01 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0385), 0x01 },
	{ CCI_REG8(0x0387), 0x01 },
	{ CCI_REG8(0x0902), 0x0a },
	{ CCI_REG8(0x3f4c), 0x01 },
	/* Digital crop offset */
	{ CCI_REG16(0x0408), 0 },
	{ CCI_REG16(0x040a), 0 },
	/* PDAF */
	{ CCI_REG8(0x3e20), 0x01 },
	{ CCI_REG8(0x3e37), 0x00 },
	{ CCI_REG8(0x3e3b), 0x00 },
	/* Other settings */
	{ CCI_REG8(0x3f78), 0x02 },
	{ CCI_REG8(0x3f79), 0x0a },
	{ CCI_REG8(0x5f0a), 0xb2 },
};

/* PLL and D-PHY timing for 1836 Mbps per lane */
static const struct cci_reg_sequence imx481_link_918mhz_regs[] = {
	{ CCI_REG8(0x0301), 0x06 },	/* VTPXCK_DIV */
	{ CCI_REG8(0x0303), 0x02 },	/* VTSYCK_DIV */
	{ CCI_REG8(0x0305), 0x04 },	/* PREPLLCK_VT_DIV */
	{ CCI_REG16(0x0306), 0x0122 },	/* PLL_VT_MPY */
	{ CCI_REG8(0x030b), 0x01 },	/* OPSYCK_DIV */
	{ CCI_REG8(0x030d), 0x02 },	/* PREPLLCK_OP_DIV */
	{ CCI_REG16(0x030e), 0x0099 },	/* PLL_OP_MPY */
	{ CCI_REG8(0x0310), 0x01 },	/* PLL_MULT_DRIV: dual PLL */
	/* Manual D-PHY timing */
	{ CCI_REG8(0x0808), 0x02 },
	{ CCI_REG16(0x080a), 0x00bf },
	{ CCI_REG16(0x080c), 0x0077 },
	{ CCI_REG16(0x080e), 0x00cf },
	{ CCI_REG16(0x0810), 0x00e0 },
	{ CCI_REG16(0x0812), 0x006f },
	{ CCI_REG16(0x0814), 0x006f },
	{ CCI_REG16(0x0816), 0x01ef },
	{ CCI_REG16(0x0818), 0x005f },
	{ CCI_REG16(0x0824), 0x00bf },
	{ CCI_REG16(0x0826), 0x000f },
};

/* 2328x1748: 2x2 binning of the whole array */
static const struct cci_reg_sequence imx481_mode_2328x1748_regs[] = {
	{ CCI_REG8(0x0900), 0x01 },	/* BINNING_MODE */
	{ CCI_REG8(0x0901), 0x22 },	/* BINNING_TYPE: H2 V2 */
	{ CCI_REG8(0x3f4d), 0x03 },
	/* Digital crop size */
	{ CCI_REG16(0x040c), 2328 },
	{ CCI_REG16(0x040e), 1748 },
	/* Output size */
	{ CCI_REG16(0x034c), 2328 },
	{ CCI_REG16(0x034e), 1748 },
	{ CCI_REG16(0x3ffe), 0x000f },
	{ CCI_REG8(0xa828), 0x02 },
	{ CCI_REG8(0xa829), 0x02 },
	{ CCI_REG8(0xa84f), 0x01 },
	{ CCI_REG8(0xa850), 0x01 },
	{ CCI_REG8(0xb2df), 0x12 },
	{ CCI_REG8(0xb2e5), 0x06 },
};

/* 4656x3496: the whole array */
static const struct cci_reg_sequence imx481_mode_4656x3496_regs[] = {
	{ CCI_REG8(0x0900), 0x00 },	/* BINNING_MODE */
	{ CCI_REG8(0x0901), 0x11 },	/* BINNING_TYPE: none */
	{ CCI_REG8(0x3f4d), 0x01 },
	/* Digital crop size */
	{ CCI_REG16(0x040c), 4656 },
	{ CCI_REG16(0x040e), 3496 },
	/* Output size */
	{ CCI_REG16(0x034c), 4656 },
	{ CCI_REG16(0x034e), 3496 },
	{ CCI_REG16(0x3ffe), 0x0018 },
	{ CCI_REG8(0xa828), 0x00 },
	{ CCI_REG8(0xa829), 0x00 },
	{ CCI_REG8(0xa84f), 0x00 },
	{ CCI_REG8(0xa850), 0x00 },
	{ CCI_REG8(0xb2df), 0x00 },
	{ CCI_REG8(0xb2e5), 0x00 },
};

struct imx481_reg_list {
	unsigned int num_regs;
	const struct cci_reg_sequence *regs;
};

struct imx481_mode {
	u32 width;
	u32 height;
	/* Line length, in pixels at the pixel rate */
	u32 hts;
	/* Shortest frame length, in lines */
	u32 vts_min;
	/* Analogue crop, in pixel array coordinates */
	struct v4l2_rect crop;
	unsigned int link_freq_index;
	struct imx481_reg_list reg_list;
};

static const struct imx481_reg_list imx481_link_freq_regs[] = {
	{
		.num_regs = ARRAY_SIZE(imx481_link_918mhz_regs),
		.regs = imx481_link_918mhz_regs,
	},
};

/*
 * Both modes keep the reference driver's 3776-line frame: 30 fps at most.
 * A longer frame (VBLANK) lowers the frame rate and allows longer
 * exposures.
 */
static const struct imx481_mode imx481_modes[] = {
	{
		.width = 4656,
		.height = 3496,
		.hts = 5120,
		.vts_min = 3776,
		.crop = {
			.left = IMX481_PIXEL_ARRAY_LEFT,
			.top = IMX481_PIXEL_ARRAY_TOP,
			.width = 4656,
			.height = 3496,
		},
		.link_freq_index = 0,
		.reg_list = {
			.num_regs = ARRAY_SIZE(imx481_mode_4656x3496_regs),
			.regs = imx481_mode_4656x3496_regs,
		},
	},
	{
		.width = 2328,
		.height = 1748,
		.hts = 5120,
		.vts_min = 3776,
		.crop = {
			.left = IMX481_PIXEL_ARRAY_LEFT,
			.top = IMX481_PIXEL_ARRAY_TOP,
			.width = 4656,
			.height = 3496,
		},
		.link_freq_index = 0,
		.reg_list = {
			.num_regs = ARRAY_SIZE(imx481_mode_2328x1748_regs),
			.regs = imx481_mode_2328x1748_regs,
		},
	},
};

#define IMX481_DEFAULT_MODE		(&imx481_modes[1])

/*
 * The Bayer order of the output changes with the flips, as the readout
 * window does not move: index with hflip | vflip << 1.
 */
static const u32 imx481_mbus_codes[] = {
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

/* The menu index is the CCS test pattern mode */
static const char * const imx481_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"Eight Vertical Colour Bars",
	"Colour Bars With Fade to Grey",
	"Pseudorandom Sequence (PN9)",
};

enum imx481_supply {
	IMX481_SUPPLY_DOVDD,
	IMX481_SUPPLY_AVDD,
	IMX481_SUPPLY_DVDD,
	IMX481_NUM_SUPPLIES
};

/* In power-up order */
static const char * const imx481_supply_names[IMX481_NUM_SUPPLIES] = {
	[IMX481_SUPPLY_DOVDD] = "dovdd",
	[IMX481_SUPPLY_AVDD] = "avdd",
	[IMX481_SUPPLY_DVDD] = "dvdd",
};

struct imx481 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[IMX481_NUM_SUPPLIES];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;

	/* Current mode, protected by the control handler lock */
	const struct imx481_mode *mode;
};

static inline struct imx481 *to_imx481(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx481, sd);
}

static u32 imx481_mbus_code(struct imx481 *imx481)
{
	unsigned int i = 0;

	if (imx481->hflip && imx481->hflip->val)
		i |= 1;
	if (imx481->vflip && imx481->vflip->val)
		i |= 2;

	return imx481_mbus_codes[i];
}

/* -----------------------------------------------------------------------------
 * Controls
 */

static void imx481_update_exposure_range(struct imx481 *imx481, s32 vblank)
{
	s64 max = imx481->mode->height + vblank - IMX481_EXPOSURE_MARGIN;

	__v4l2_ctrl_modify_range(imx481->exposure, imx481->exposure->minimum,
				 max, imx481->exposure->step,
				 min_t(s64, imx481->exposure->val, max));
}

static int imx481_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx481 *imx481 = container_of(ctrl->handler, struct imx481,
					     ctrls);
	struct v4l2_subdev_state *state;
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		imx481_update_exposure_range(imx481, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		/* The flips change the Bayer order of the active format */
		if (imx481->sd.active_state) {
			state = v4l2_subdev_get_locked_active_state(&imx481->sd);
			v4l2_subdev_state_get_format(state, 0)->code =
				imx481_mbus_code(imx481);
		}
		break;
	default:
		break;
	}

	/* The registers are written when streaming starts */
	if (pm_runtime_get_if_active(imx481->dev) <= 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(imx481->regmap, IMX481_REG_EXPOSURE, ctrl->val, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(imx481->regmap, IMX481_REG_ANALOG_GAIN, ctrl->val,
			  &ret);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		cci_write(imx481->regmap, IMX481_REG_DIG_GAIN_GR, ctrl->val,
			  &ret);
		cci_write(imx481->regmap, IMX481_REG_DIG_GAIN_R, ctrl->val,
			  &ret);
		cci_write(imx481->regmap, IMX481_REG_DIG_GAIN_B, ctrl->val,
			  &ret);
		cci_write(imx481->regmap, IMX481_REG_DIG_GAIN_GB, ctrl->val,
			  &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(imx481->regmap, IMX481_REG_FRAME_LENGTH,
			  imx481->mode->height + ctrl->val, &ret);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		cci_write(imx481->regmap, IMX481_REG_ORIENTATION,
			  (imx481->hflip->val ? IMX481_ORIENTATION_HFLIP : 0) |
			  (imx481->vflip->val ? IMX481_ORIENTATION_VFLIP : 0),
			  &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(imx481->regmap, IMX481_REG_TEST_PATTERN, ctrl->val,
			  &ret);
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		cci_write(imx481->regmap, IMX481_REG_TEST_DATA_RED, ctrl->val,
			  &ret);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		cci_write(imx481->regmap, IMX481_REG_TEST_DATA_GREENR,
			  ctrl->val, &ret);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		cci_write(imx481->regmap, IMX481_REG_TEST_DATA_BLUE, ctrl->val,
			  &ret);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		cci_write(imx481->regmap, IMX481_REG_TEST_DATA_GREENB,
			  ctrl->val, &ret);
		break;
	case V4L2_CID_HBLANK:
	case V4L2_CID_LINK_FREQ:
		/* Read-only, follow the mode: nothing to write */
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(imx481->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx481_ctrl_ops = {
	.s_ctrl = imx481_set_ctrl,
};

/* Update the mode dependent controls; called with the handler lock held */
static int imx481_update_mode_ctrls(struct imx481 *imx481)
{
	const struct imx481_mode *mode = imx481->mode;
	u32 hblank = mode->hts - mode->width;
	u32 vblank = mode->vts_min - mode->height;
	int ret;

	ret = __v4l2_ctrl_s_ctrl(imx481->link_freq, mode->link_freq_index);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_modify_range(imx481->hblank, hblank, hblank, 1,
				       hblank);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_modify_range(imx481->vblank, vblank,
				       IMX481_FRAME_LENGTH_MAX - mode->height,
				       1, vblank);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_s_ctrl(imx481->vblank, vblank);
	if (ret)
		return ret;

	/* Default to the longest exposure the frame allows */
	return __v4l2_ctrl_modify_range(imx481->exposure,
					IMX481_EXPOSURE_MIN,
					mode->vts_min - IMX481_EXPOSURE_MARGIN,
					IMX481_EXPOSURE_STEP,
					mode->vts_min - IMX481_EXPOSURE_MARGIN);
}

static int imx481_init_controls(struct imx481 *imx481)
{
	struct v4l2_ctrl_handler *hdl = &imx481->ctrls;
	const struct imx481_mode *mode = imx481->mode;
	struct v4l2_fwnode_device_properties props;
	u32 hblank = mode->hts - mode->width;
	u32 vblank = mode->vts_min - mode->height;
	u32 exposure_max = mode->vts_min - IMX481_EXPOSURE_MARGIN;
	unsigned int i;
	int ret;

	ret = v4l2_fwnode_device_parse(imx481->dev, &props);
	if (ret)
		return ret;

	v4l2_ctrl_handler_init(hdl, 16);

	imx481->link_freq = v4l2_ctrl_new_int_menu(hdl, &imx481_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(imx481_link_freqs) - 1,
						   mode->link_freq_index,
						   imx481_link_freqs);
	if (imx481->link_freq)
		imx481->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx481->pixel_rate = v4l2_ctrl_new_std(hdl, &imx481_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       IMX481_PIXEL_RATE,
					       IMX481_PIXEL_RATE, 1,
					       IMX481_PIXEL_RATE);

	imx481->hblank = v4l2_ctrl_new_std(hdl, &imx481_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank, 1,
					   hblank);
	if (imx481->hblank)
		imx481->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx481->vblank = v4l2_ctrl_new_std(hdl, &imx481_ctrl_ops,
					   V4L2_CID_VBLANK, vblank,
					   IMX481_FRAME_LENGTH_MAX - mode->height,
					   1, vblank);

	imx481->exposure = v4l2_ctrl_new_std(hdl, &imx481_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX481_EXPOSURE_MIN, exposure_max,
					     IMX481_EXPOSURE_STEP,
					     exposure_max);

	v4l2_ctrl_new_std(hdl, &imx481_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX481_ANA_GAIN_MIN, IMX481_ANA_GAIN_MAX,
			  IMX481_ANA_GAIN_STEP, IMX481_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(hdl, &imx481_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX481_DGTL_GAIN_MIN, IMX481_DGTL_GAIN_MAX,
			  IMX481_DGTL_GAIN_STEP, IMX481_DGTL_GAIN_DEFAULT);

	imx481->hflip = v4l2_ctrl_new_std(hdl, &imx481_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (imx481->hflip)
		imx481->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	imx481->vflip = v4l2_ctrl_new_std(hdl, &imx481_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (imx481->vflip)
		imx481->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(hdl, &imx481_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx481_test_pattern_menu) - 1,
				     0, 0, imx481_test_pattern_menu);
	for (i = 0; i < 4; i++)
		v4l2_ctrl_new_std(hdl, &imx481_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED + i,
				  0, IMX481_TEST_DATA_MAX, 1,
				  IMX481_TEST_DATA_MAX);

	v4l2_ctrl_new_fwnode_properties(hdl, &imx481_ctrl_ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		dev_err(imx481->dev, "failed to add controls: %d\n", ret);
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	imx481->sd.ctrl_handler = hdl;

	return 0;
}

/* -----------------------------------------------------------------------------
 * Power
 */

static int imx481_power_on(struct device *dev)
{
	struct imx481 *imx481 = to_imx481(dev_get_drvdata(dev));
	int ret;
	int i;

	/*
	 * Enable the rails one at a time: regulator_bulk_enable() does not
	 * keep the order.
	 */
	for (i = 0; i < IMX481_NUM_SUPPLIES; i++) {
		ret = regulator_enable(imx481->supplies[i].consumer);
		if (ret) {
			dev_err(dev, "failed to enable %s: %d\n",
				imx481->supplies[i].supply, ret);
			goto err_supplies;
		}
		usleep_range(1000, 1100);
	}

	ret = clk_prepare_enable(imx481->mclk);
	if (ret) {
		dev_err(dev, "failed to enable MCLK: %d\n", ret);
		goto err_supplies;
	}
	usleep_range(1000, 1100);

	gpiod_set_value_cansleep(imx481->reset_gpio, 0);
	usleep_range(IMX481_XCLR_DELAY_US, IMX481_XCLR_DELAY_US + 1000);

	return 0;

err_supplies:
	while (--i >= 0)
		regulator_disable(imx481->supplies[i].consumer);

	return ret;
}

static int imx481_power_off(struct device *dev)
{
	struct imx481 *imx481 = to_imx481(dev_get_drvdata(dev));
	int i;

	gpiod_set_value_cansleep(imx481->reset_gpio, 1);
	usleep_range(1000, 1100);

	clk_disable_unprepare(imx481->mclk);

	for (i = IMX481_NUM_SUPPLIES - 1; i >= 0; i--)
		regulator_disable(imx481->supplies[i].consumer);

	return 0;
}

/* -----------------------------------------------------------------------------
 * Subdev
 */

static int imx481_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct imx481 *imx481 = to_imx481(sd);
	const struct imx481_mode *mode = imx481->mode;
	const struct imx481_reg_list *link_regs;
	int ret;

	ret = pm_runtime_resume_and_get(imx481->dev);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(imx481->regmap, imx481_init_regs,
				  ARRAY_SIZE(imx481_init_regs), NULL);
	if (ret) {
		dev_err(imx481->dev, "failed to write init registers: %d\n",
			ret);
		goto err_rpm_put;
	}

	link_regs = &imx481_link_freq_regs[mode->link_freq_index];
	ret = cci_multi_reg_write(imx481->regmap, link_regs->regs,
				  link_regs->num_regs, NULL);
	if (ret) {
		dev_err(imx481->dev, "failed to write PLL registers: %d\n",
			ret);
		goto err_rpm_put;
	}

	ret = cci_multi_reg_write(imx481->regmap, mode->reg_list.regs,
				  mode->reg_list.num_regs, NULL);
	if (!ret)
		ret = cci_write(imx481->regmap, IMX481_REG_LINE_LENGTH,
				mode->hts, NULL);
	if (ret) {
		dev_err(imx481->dev, "failed to write mode registers: %d\n",
			ret);
		goto err_rpm_put;
	}

	/* Exposure, gains, frame length, flips and test pattern */
	ret = __v4l2_ctrl_handler_setup(&imx481->ctrls);
	if (ret)
		goto err_rpm_put;

	ret = cci_write(imx481->regmap, IMX481_REG_MODE_SELECT,
			IMX481_MODE_STREAMING, NULL);
	if (ret) {
		dev_err(imx481->dev, "failed to start streaming: %d\n", ret);
		goto err_rpm_put;
	}

	/* The flips change the Bayer order: not while streaming */
	__v4l2_ctrl_grab(imx481->hflip, true);
	__v4l2_ctrl_grab(imx481->vflip, true);

	return 0;

err_rpm_put:
	pm_runtime_put_autosuspend(imx481->dev);

	return ret;
}

static int imx481_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct imx481 *imx481 = to_imx481(sd);
	int ret;

	ret = cci_write(imx481->regmap, IMX481_REG_MODE_SELECT,
			IMX481_MODE_STANDBY, NULL);
	if (ret)
		dev_err(imx481->dev, "failed to stop streaming: %d\n", ret);

	__v4l2_ctrl_grab(imx481->hflip, false);
	__v4l2_ctrl_grab(imx481->vflip, false);

	pm_runtime_put_autosuspend(imx481->dev);

	return ret;
}

static void imx481_fill_format(struct imx481 *imx481,
			       const struct imx481_mode *mode,
			       struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = imx481_mbus_code(imx481);
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int imx481_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = imx481_mbus_code(to_imx481(sd));

	return 0;
}

static int imx481_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx481 *imx481 = to_imx481(sd);

	if (fse->index >= ARRAY_SIZE(imx481_modes) ||
	    fse->code != imx481_mbus_code(imx481))
		return -EINVAL;

	fse->min_width = imx481_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = imx481_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int imx481_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct imx481 *imx481 = to_imx481(sd);
	const struct imx481_mode *mode;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    v4l2_subdev_is_streaming(sd))
		return -EBUSY;

	mode = v4l2_find_nearest_size(imx481_modes, ARRAY_SIZE(imx481_modes),
				      width, height, fmt->format.width,
				      fmt->format.height);

	imx481_fill_format(imx481, mode, &fmt->format);
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;
	*v4l2_subdev_state_get_crop(state, 0) = mode->crop;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		imx481->mode = mode;
		return imx481_update_mode_ctrls(imx481);
	}

	return 0;
}

static int imx481_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, 0);
		return 0;

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX481_NATIVE_WIDTH;
		sel->r.height = IMX481_NATIVE_HEIGHT;
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = IMX481_PIXEL_ARRAY_LEFT;
		sel->r.top = IMX481_PIXEL_ARRAY_TOP;
		sel->r.width = IMX481_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX481_PIXEL_ARRAY_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static int imx481_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.width = IMX481_DEFAULT_MODE->width,
			.height = IMX481_DEFAULT_MODE->height,
		},
	};

	return imx481_set_fmt(sd, state, &fmt);
}

static const struct v4l2_subdev_video_ops imx481_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops imx481_pad_ops = {
	.enum_mbus_code = imx481_enum_mbus_code,
	.enum_frame_size = imx481_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx481_set_fmt,
	.get_selection = imx481_get_selection,
	.enable_streams = imx481_enable_streams,
	.disable_streams = imx481_disable_streams,
};

static const struct v4l2_subdev_ops imx481_subdev_ops = {
	.video = &imx481_video_ops,
	.pad = &imx481_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx481_internal_ops = {
	.init_state = imx481_init_state,
};

/* -----------------------------------------------------------------------------
 * Probe
 */

static int imx481_read_otp_id(struct imx481 *imx481, u64 *id)
{
	u64 status = 0;
	int ret = 0;
	int i;

	cci_write(imx481->regmap, IMX481_REG_OTP_PAGE, IMX481_OTP_ID_PAGE, &ret);
	cci_write(imx481->regmap, IMX481_REG_OTP_CTRL, IMX481_OTP_CTRL_READ,
		  &ret);
	if (ret)
		return ret;

	for (i = 0; i < 10; i++) {
		ret = cci_read(imx481->regmap, IMX481_REG_OTP_STATUS, &status,
			       NULL);
		if (ret)
			return ret;
		if (status & IMX481_OTP_STATUS_READY)
			break;
		usleep_range(1000, 1100);
	}

	if (!(status & IMX481_OTP_STATUS_READY)) {
		dev_err(imx481->dev, "OTP read timed out (status 0x%02llx)\n",
			status);
		return -ETIMEDOUT;
	}

	ret = cci_read(imx481->regmap, IMX481_REG_OTP_ID, id, NULL);
	if (ret)
		return ret;

	*id >>= IMX481_OTP_ID_SHIFT;

	return 0;
}

static int imx481_identify(struct imx481 *imx481)
{
	u64 id;
	int ret;

	ret = cci_read(imx481->regmap, IMX481_REG_CHIP_ID, &id, NULL);
	if (ret)
		return dev_err_probe(imx481->dev, ret,
				     "failed to read chip id\n");

	if (id == IMX481_CHIP_ID) {
		dev_dbg(imx481->dev, "chip id 0x%04llx\n", id);
		return 0;
	}

	dev_dbg(imx481->dev, "register 0x0016 reads 0x%04llx, trying OTP\n",
		id);

	ret = imx481_read_otp_id(imx481, &id);
	if (ret)
		return dev_err_probe(imx481->dev, ret,
				     "failed to read chip id from OTP\n");

	if (id != IMX481_CHIP_ID)
		return dev_err_probe(imx481->dev, -ENODEV,
				     "chip id mismatch: 0x%04llx, expected 0x%04x\n",
				     id, IMX481_CHIP_ID);

	dev_dbg(imx481->dev, "chip id 0x%04llx (OTP)\n", id);

	return 0;
}

static int imx481_check_hwcfg(struct device *dev)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	unsigned long link_freq_bitmap;
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!ep)
		return dev_err_probe(dev, -EINVAL, "no endpoint\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(dev, ret, "failed to parse endpoint\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != IMX481_DATA_LANES) {
		ret = dev_err_probe(dev, -EINVAL,
				    "%u data lanes not supported\n",
				    bus_cfg.bus.mipi_csi2.num_data_lanes);
		goto out;
	}

	/* All modes use the one link frequency */
	ret = v4l2_link_freq_to_bitmap(dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       imx481_link_freqs,
				       ARRAY_SIZE(imx481_link_freqs),
				       &link_freq_bitmap);

out:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int imx481_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx481 *imx481;
	unsigned long rate;
	int ret;
	int i;

	imx481 = devm_kzalloc(dev, sizeof(*imx481), GFP_KERNEL);
	if (!imx481)
		return -ENOMEM;

	imx481->dev = dev;

	ret = imx481_check_hwcfg(dev);
	if (ret)
		return ret;

	/* Also sets the client data to the subdev */
	v4l2_i2c_subdev_init(&imx481->sd, client, &imx481_subdev_ops);

	imx481->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx481->regmap))
		return dev_err_probe(dev, PTR_ERR(imx481->regmap),
				     "failed to initialize CCI\n");

	for (i = 0; i < IMX481_NUM_SUPPLIES; i++)
		imx481->supplies[i].supply = imx481_supply_names[i];

	ret = devm_regulator_bulk_get(dev, IMX481_NUM_SUPPLIES,
				      imx481->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get supplies\n");

	imx481->mclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(imx481->mclk))
		return dev_err_probe(dev, PTR_ERR(imx481->mclk),
				     "failed to get MCLK\n");

	rate = clk_get_rate(imx481->mclk);
	if (rate != IMX481_MCLK_RATE)
		return dev_err_probe(dev, -EINVAL,
				     "unsupported MCLK rate %lu\n", rate);

	/* Hold the sensor in reset until it is powered */
	imx481->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(imx481->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(imx481->reset_gpio),
				     "failed to get reset GPIO\n");

	ret = imx481_power_on(dev);
	if (ret)
		return ret;

	ret = imx481_identify(imx481);
	if (ret)
		goto err_power_off;

	imx481->mode = IMX481_DEFAULT_MODE;

	ret = imx481_init_controls(imx481);
	if (ret)
		goto err_power_off;

	imx481->sd.internal_ops = &imx481_internal_ops;
	imx481->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx481->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx481->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx481->sd.entity, 1, &imx481->pad);
	if (ret)
		goto err_ctrls;

	imx481->sd.state_lock = imx481->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&imx481->sd);
	if (ret)
		goto err_entity;

	/* Keep the sensor powered until it is idled below */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&imx481->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register subdev\n");
		goto err_pm;
	}

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_idle(dev);

	return 0;

err_pm:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	v4l2_subdev_cleanup(&imx481->sd);
err_entity:
	media_entity_cleanup(&imx481->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&imx481->ctrls);
err_power_off:
	imx481_power_off(dev);

	return ret;
}

static void imx481_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx481 *imx481 = to_imx481(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&imx481->ctrls);

	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		imx481_power_off(dev);
	pm_runtime_set_suspended(dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx481_pm_ops, imx481_power_off,
				 imx481_power_on, NULL);

static const struct of_device_id imx481_of_match[] = {
	{ .compatible = "sony,imx481" },
	{ }
};
MODULE_DEVICE_TABLE(of, imx481_of_match);

static struct i2c_driver imx481_driver = {
	.driver = {
		.name = "imx481",
		.of_match_table = imx481_of_match,
		.pm = pm_ptr(&imx481_pm_ops),
	},
	.probe = imx481_probe,
	.remove = imx481_remove,
};
module_i2c_driver(imx481_driver);

MODULE_DESCRIPTION("Sony IMX481 image sensor driver");
MODULE_LICENSE("GPL");

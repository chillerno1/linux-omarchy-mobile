// SPDX-License-Identifier: GPL-2.0
/*
 * Sony IMX563 image sensor driver
 *
 * The IMX563 is a 12 Mpixel (4032x3024 active) Bayer sensor with a MIPI
 * CSI-2 output over C-PHY and an SMIA++ style register map. The standard
 * registers (mode select, frame and line length, integration time, gains,
 * orientation, test pattern) are driven from here. The global settings and
 * the two readout modes, for a 19.2 MHz external clock and three C-PHY
 * trios, come from the GPL-2.0 Samsung Exynos camera driver for this
 * sensor.
 */

#include <linux/bitops.h>
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

#define IMX563_REG_MODE_SELECT		CCI_REG8(0x0100)
#define IMX563_MODE_STANDBY		0x00
#define IMX563_MODE_STREAMING		0x01

#define IMX563_REG_ORIENTATION		CCI_REG8(0x0101)
#define IMX563_ORIENTATION_HFLIP	BIT(0)
#define IMX563_ORIENTATION_VFLIP	BIT(1)

#define IMX563_REG_MODEL_ID		CCI_REG16(0x0016)
#define IMX563_MODEL_ID			0x0563

/* Coarse integration time, in lines */
#define IMX563_REG_EXPOSURE		CCI_REG16(0x0202)
#define IMX563_EXPOSURE_STEP		1
/* Lines between the end of the integration time and the frame length */
#define IMX563_EXPOSURE_MARGIN		96

/* Analogue gain = 1024 / (1024 - code), 1x to 8x */
#define IMX563_REG_ANALOG_GAIN		CCI_REG16(0x0204)
#define IMX563_ANA_GAIN_MIN		0
#define IMX563_ANA_GAIN_MAX		896
#define IMX563_ANA_GAIN_STEP		1
#define IMX563_ANA_GAIN_DEFAULT		IMX563_ANA_GAIN_MIN

/* Digital gain, U4.8, applied to all colour channels */
#define IMX563_REG_DIG_GAIN		CCI_REG16(0x020e)
#define IMX563_DGTL_GAIN_MIN		0x0100
#define IMX563_DGTL_GAIN_MAX		0x0fd9
#define IMX563_DGTL_GAIN_STEP		1
#define IMX563_DGTL_GAIN_DEFAULT	0x0100

#define IMX563_REG_FRAME_LENGTH		CCI_REG16(0x0340)
#define IMX563_FRAME_LENGTH_MAX		0xffff
#define IMX563_REG_LINE_LENGTH		CCI_REG16(0x0342)

#define IMX563_REG_TEST_PATTERN		CCI_REG16(0x0600)
#define IMX563_REG_TEST_DATA_RED	CCI_REG16(0x0602)
#define IMX563_REG_TEST_DATA_GREENR	CCI_REG16(0x0604)
#define IMX563_REG_TEST_DATA_BLUE	CCI_REG16(0x0606)
#define IMX563_REG_TEST_DATA_GREENB	CCI_REG16(0x0608)
#define IMX563_TEST_DATA_MAX		0x03ff
/* Written as the reference driver does before selecting a test pattern */
#define IMX563_REG_TEST_PATTERN_PREP	CCI_REG8(0xb000)

#define IMX563_MCLK_RATE		19200000

/* Delay from XCLR release to the first I2C access */
#define IMX563_XCLR_DELAY_US		10000

#define IMX563_DATA_LANES		3

/*
 * Clocks, from the 19.2 MHz EXCK:
 *  - video timing: 19.2 MHz / 3 (PREPLLCK_VT_DIV) * 322 (PLL_VT_MPY) =
 *    2060.8 MHz. The line length counter runs at twice that, 4121.6 MHz,
 *    the pixel rate here: both modes give 30.0 fps with it.
 *  - output: EXCK / PREPLLCK_OP_DIV * PLL_OP_MPY / OPSYCK_DIV is the
 *    symbol rate on each trio:
 *      4000x3000: 19.2 MHz / 2 * 395 / 2 = 1896 Msym/s
 *      1984x1488: 19.2 MHz / 3 * 359 / 1 = 2297.6 Msym/s
 *    The CSI-2 link frequency of a C-PHY link is half the symbol rate.
 */
#define IMX563_PIXEL_RATE		4121600000LL
#define IMX563_LINK_FREQ_1148M8		1148800000LL
#define IMX563_LINK_FREQ_948M		948000000LL

enum imx563_link_freq {
	IMX563_LINK_FREQ_INDEX_1148M8,
	IMX563_LINK_FREQ_INDEX_948M,
};

static const s64 imx563_link_freqs[] = {
	[IMX563_LINK_FREQ_INDEX_1148M8] = IMX563_LINK_FREQ_1148M8,
	[IMX563_LINK_FREQ_INDEX_948M] = IMX563_LINK_FREQ_948M,
};

/* Pixel array */
#define IMX563_NATIVE_WIDTH		4032
#define IMX563_NATIVE_HEIGHT		3024
#define IMX563_PIXEL_ARRAY_LEFT		0
#define IMX563_PIXEL_ARRAY_TOP		0
#define IMX563_PIXEL_ARRAY_WIDTH	4032
#define IMX563_PIXEL_ARRAY_HEIGHT	3024

/* Global settings, written once after every power up */
static const struct cci_reg_sequence imx563_init_regs[] = {
	/* External Clock Setting */
	{ CCI_REG8(0x0136), 0x13 },	/* EXCK_FREQ: 19.2 MHz */
	{ CCI_REG8(0x0137), 0x33 },
	/* Register version */
	{ CCI_REG8(0x33f0), 0x01 },
	{ CCI_REG8(0x33f1), 0x01 },
	/* Signaling Mode Setting */
	{ CCI_REG8(0x0111), 0x03 },	/* CSI_SIGNALING_MODE: C-PHY */
	/* Additional Setting */
	{ CCI_REG8(0xf800), 0x9b },
	{ CCI_REG8(0xf801), 0x9b },
	{ CCI_REG8(0xf802), 0xea },
	{ CCI_REG8(0xf803), 0xdb },
	{ CCI_REG8(0xf804), 0x55 },
	{ CCI_REG8(0xf805), 0x48 },
	{ CCI_REG8(0xf806), 0x6f },
	{ CCI_REG8(0xf807), 0x00 },
	{ CCI_REG8(0xf808), 0x3c },
	{ CCI_REG8(0xf809), 0x91 },
	{ CCI_REG8(0xf80a), 0x17 },
	{ CCI_REG8(0xf80b), 0xfa },
	{ CCI_REG8(0xf80c), 0x55 },
	{ CCI_REG8(0xf80d), 0x80 },
	{ CCI_REG8(0xf80e), 0x35 },
	{ CCI_REG8(0xf80f), 0x00 },
	{ CCI_REG8(0xf810), 0x1a },
	{ CCI_REG8(0xf811), 0xcb },
	{ CCI_REG8(0xf812), 0x17 },
	{ CCI_REG8(0xf813), 0xfa },
	{ CCI_REG8(0xf884), 0x4f },
	{ CCI_REG8(0xf885), 0x86 },
	{ CCI_REG8(0xf886), 0xac },
	{ CCI_REG8(0xf887), 0x17 },
	{ CCI_REG8(0xf888), 0xd0 },
	{ CCI_REG8(0xf889), 0x18 },
	{ CCI_REG8(0xf88a), 0x0b },
	{ CCI_REG8(0xf88b), 0x00 },
	{ CCI_REG8(0xf88c), 0xf6 },
	{ CCI_REG8(0xf88d), 0x00 },
	{ CCI_REG8(0xf88e), 0x03 },
	{ CCI_REG8(0xf88f), 0x10 },
	{ CCI_REG8(0xf890), 0xbc },
	{ CCI_REG8(0xf891), 0x03 },
	{ CCI_REG8(0xf892), 0xfa },
	{ CCI_REG8(0xf893), 0xe8 },
	{ CCI_REG8(0xf894), 0x98 },
	{ CCI_REG8(0xf895), 0x2c },
	{ CCI_REG8(0xf896), 0xa8 },
	{ CCI_REG8(0xf897), 0x17 },
	{ CCI_REG8(0xf898), 0xa0 },
	{ CCI_REG8(0xf899), 0x05 },
	{ CCI_REG8(0xf89a), 0xd0 },
	{ CCI_REG8(0xf89b), 0x18 },
	{ CCI_REG8(0xf89c), 0x0b },
	{ CCI_REG8(0xf89d), 0x00 },
	{ CCI_REG8(0xf89e), 0xf0 },
	{ CCI_REG8(0xf89f), 0x00 },
	{ CCI_REG8(0xf8a0), 0x03 },
	{ CCI_REG8(0xf8a1), 0x10 },
	{ CCI_REG8(0xf8a2), 0xfc },
	{ CCI_REG8(0xf8a3), 0x41 },
	{ CCI_REG8(0xf8a4), 0x01 },
	{ CCI_REG8(0xf8a5), 0x00 },
	{ CCI_REG8(0xf8a6), 0x24 },
	{ CCI_REG8(0xf8a7), 0x10 },
	{ CCI_REG8(0xf8a8), 0x90 },
	{ CCI_REG8(0xf8a9), 0x1a },
	{ CCI_REG8(0xf8aa), 0xd1 },
	{ CCI_REG8(0xf8ab), 0x20 },
	{ CCI_REG8(0xf8ac), 0xd3 },
	{ CCI_REG8(0xf8ad), 0x48 },
	{ CCI_REG8(0xf8ae), 0x1e },
	{ CCI_REG8(0xf8af), 0x11 },
	{ CCI_REG8(0xf8b0), 0x50 },
	{ CCI_REG8(0xf8b1), 0x82 },
	{ CCI_REG8(0xf8b2), 0x51 },
	{ CCI_REG8(0xf8b3), 0x03 },
	{ CCI_REG8(0xf8b4), 0x28 },
	{ CCI_REG8(0xf8b5), 0x01 },
	{ CCI_REG8(0xf8b6), 0xa4 },
	{ CCI_REG8(0xf8b7), 0x31 },
	{ CCI_REG8(0xf8b8), 0xa5 },
	{ CCI_REG8(0xf8b9), 0x21 },
	{ CCI_REG8(0xf8ba), 0x09 },
	{ CCI_REG8(0xf8bb), 0x10 },
	{ CCI_REG8(0xf8bc), 0xd1 },
	{ CCI_REG8(0xf8bd), 0x20 },
	{ CCI_REG8(0xf8be), 0xe4 },
	{ CCI_REG8(0xf8bf), 0x1a },
	{ CCI_REG8(0xf8c0), 0x1f },
	{ CCI_REG8(0xf8c1), 0x10 },
	{ CCI_REG8(0xf8c2), 0xa0 },
	{ CCI_REG8(0xf8c3), 0x70 },
	{ CCI_REG8(0x9391), 0x02 },
	{ CCI_REG8(0x9392), 0x03 },
	{ CCI_REG8(0x4339), 0x01 },
	/* Global Setting */
	{ CCI_REG8(0x9f00), 0x01 },
	{ CCI_REG8(0x9f04), 0x02 },
	{ CCI_REG8(0x9f05), 0x3f },
	{ CCI_REG8(0xbc40), 0x03 },
	{ CCI_REG8(0xbc41), 0x03 },
	{ CCI_REG8(0xbc4c), 0x03 },
	{ CCI_REG8(0x3058), 0x00 },
};

/*
 * Written after the mode registers: only the image goes out, on virtual
 * channel 0, with plain CSI-2 packet framing.
 */
static const struct cci_reg_sequence imx563_output_regs[] = {
	/* No embedded data lines */
	{ CCI_REG8(0xbcf0), 0x00 },
	{ CCI_REG8(0xbcf1), 0x00 },
	/* No phase detection data after the image */
	{ CCI_REG8(0x3081), 0x00 },
	/* No Efficient Packet Delimiter (LRTE) on the C-PHY link */
	{ CCI_REG16(0x0860), 0x0000 },
	{ CCI_REG16(0x0862), 0x0000 },
};

/* 4000x3000: the centre of the pixel array, 1896 Msym/s */
static const struct cci_reg_sequence imx563_mode_4000x3000_regs[] = {
	/* Mode Setting */
	{ CCI_REG8(0x0112), 0x0a },	/* CSI_DATA_FORMAT: RAW10 */
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x02 },	/* CSI_LANE_MODE: 3 trios */
	{ CCI_REG8(0x0220), 0x00 },
	{ CCI_REG8(0x0221), 0x11 },
	{ CCI_REG8(0x0222), 0x01 },
	{ CCI_REG8(0x0900), 0x00 },	/* BINNING_MODE */
	{ CCI_REG8(0x0901), 0x11 },	/* BINNING_TYPE */
	{ CCI_REG8(0x0902), 0x00 },
	{ CCI_REG8(0x3004), 0x03 },
	{ CCI_REG8(0x30a5), 0x00 },
	{ CCI_REG8(0x30a8), 0x06 },
	{ CCI_REG8(0x3200), 0x01 },
	{ CCI_REG8(0x4060), 0x00 },
	{ CCI_REG8(0x4061), 0x00 },
	/* Output Size Setting */
	{ CCI_REG8(0x0344), 0x00 },	/* X_ADDR_START */
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x00 },	/* Y_ADDR_START */
	{ CCI_REG8(0x0347), 0x0c },
	{ CCI_REG8(0x0348), 0x0f },	/* X_ADDR_END */
	{ CCI_REG8(0x0349), 0xbf },
	{ CCI_REG8(0x034a), 0x0b },	/* Y_ADDR_END */
	{ CCI_REG8(0x034b), 0xc3 },
	{ CCI_REG8(0x034c), 0x0f },	/* X_OUTPUT_SIZE */
	{ CCI_REG8(0x034d), 0xa0 },
	{ CCI_REG8(0x034e), 0x0b },	/* Y_OUTPUT_SIZE */
	{ CCI_REG8(0x034f), 0xb8 },
	{ CCI_REG8(0x0408), 0x00 },	/* DIG_CROP_X_OFFSET */
	{ CCI_REG8(0x0409), 0x10 },
	{ CCI_REG8(0x040a), 0x00 },	/* DIG_CROP_Y_OFFSET */
	{ CCI_REG8(0x040b), 0x00 },
	{ CCI_REG8(0x040c), 0x0f },	/* DIG_CROP_IMAGE_WIDTH */
	{ CCI_REG8(0x040d), 0xa0 },
	{ CCI_REG8(0x040e), 0x0b },	/* DIG_CROP_IMAGE_HEIGHT */
	{ CCI_REG8(0x040f), 0xb8 },
	/* Clock Setting */
	{ CCI_REG8(0x0303), 0x02 },	/* VTSYCK_DIV */
	{ CCI_REG8(0x0305), 0x03 },	/* PREPLLCK_VT_DIV */
	{ CCI_REG8(0x0306), 0x01 },	/* PLL_VT_MPY */
	{ CCI_REG8(0x0307), 0x42 },
	{ CCI_REG8(0x030b), 0x02 },	/* OPSYCK_DIV */
	{ CCI_REG8(0x030d), 0x02 },	/* PREPLLCK_OP_DIV */
	{ CCI_REG8(0x030e), 0x01 },	/* PLL_OP_MPY */
	{ CCI_REG8(0x030f), 0x8b },
	{ CCI_REG8(0x0310), 0x01 },	/* PLL_MULT_DRIV */
	{ CCI_REG8(0x3240), 0x00 },
	/* Other Setting */
	{ CCI_REG8(0x40b8), 0x03 },
	{ CCI_REG8(0x40b9), 0x30 },
	{ CCI_REG8(0x40bc), 0x01 },
	{ CCI_REG8(0x40bd), 0xa4 },
	{ CCI_REG8(0x40be), 0x01 },
	{ CCI_REG8(0x40bf), 0xa4 },
	{ CCI_REG8(0x7500), 0x02 },
	{ CCI_REG8(0x57aa), 0x08 },
	{ CCI_REG8(0x57ab), 0x62 },
	{ CCI_REG8(0x5620), 0x01 },
	{ CCI_REG8(0x5621), 0x9a },
	{ CCI_REG8(0x57a6), 0x02 },
	{ CCI_REG8(0x57a7), 0x08 },
	{ CCI_REG8(0x6d41), 0x13 },
	{ CCI_REG8(0x6d4c), 0x13 },
	{ CCI_REG8(0x6d57), 0x13 },
	{ CCI_REG8(0x6d62), 0x13 },
	{ CCI_REG8(0x6a54), 0x04 },
	{ CCI_REG8(0x6a55), 0x05 },
	{ CCI_REG8(0x6a60), 0x04 },
	{ CCI_REG8(0x6a6b), 0x05 },
	{ CCI_REG8(0x6a75), 0x09 },
	{ CCI_REG8(0x6a76), 0x0b },
	{ CCI_REG8(0x6a80), 0x01 },
	{ CCI_REG8(0x6a81), 0x00 },
	{ CCI_REG8(0x6a8c), 0x01 },
	{ CCI_REG8(0x6a97), 0x02 },
	{ CCI_REG8(0x6aa1), 0x02 },
	{ CCI_REG8(0x6aa2), 0x03 },
	/* Integration Time Setting */
	{ CCI_REG8(0x0224), 0x01 },
	{ CCI_REG8(0x0225), 0xf4 },
	/* Gain Setting */
	{ CCI_REG8(0x0216), 0x00 },
	{ CCI_REG8(0x0217), 0x00 },
	{ CCI_REG8(0x0218), 0x01 },
	{ CCI_REG8(0x0219), 0x00 },
	/* Additional Patch Setting */
	{ CCI_REG8(0x9390), 0x00 },
	/* AEB_LUT */
	{ CCI_REG8(0x0e10), 0x00 },
	{ CCI_REG8(0x0e11), 0x00 },
	{ CCI_REG8(0x0e12), 0x00 },
	{ CCI_REG8(0x0e13), 0x00 },
	{ CCI_REG8(0x0e14), 0x00 },
	{ CCI_REG8(0x0e15), 0x00 },
	{ CCI_REG8(0x0e1e), 0x00 },
	{ CCI_REG8(0x0e1f), 0x00 },
	{ CCI_REG8(0x0e20), 0x00 },
	{ CCI_REG8(0x0e23), 0x00 },
	{ CCI_REG8(0x0e26), 0x00 },
	{ CCI_REG8(0x0e28), 0x00 },
	{ CCI_REG8(0x0e29), 0x00 },
	{ CCI_REG8(0x0e2a), 0x00 },
	{ CCI_REG8(0x0e2b), 0x00 },
	{ CCI_REG8(0x0e2c), 0x00 },
	{ CCI_REG8(0x0e2d), 0x00 },
	{ CCI_REG8(0x0e36), 0x00 },
	{ CCI_REG8(0x0e37), 0x00 },
	{ CCI_REG8(0x0e38), 0x00 },
	{ CCI_REG8(0x0e3b), 0x00 },
	{ CCI_REG8(0x0e3e), 0x00 },
	{ CCI_REG8(0x0e01), 0x00 },
	{ CCI_REG8(0x0e00), 0x00 },
};

/* 1984x1488: 2x2 binning of the centre of the array, 2297.6 Msym/s */
static const struct cci_reg_sequence imx563_mode_1984x1488_regs[] = {
	/* Mode Setting */
	{ CCI_REG8(0x0112), 0x0a },	/* CSI_DATA_FORMAT: RAW10 */
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x02 },	/* CSI_LANE_MODE: 3 trios */
	{ CCI_REG8(0x0220), 0x00 },
	{ CCI_REG8(0x0221), 0x11 },
	{ CCI_REG8(0x0222), 0x01 },
	{ CCI_REG8(0x0900), 0x01 },	/* BINNING_MODE */
	{ CCI_REG8(0x0901), 0x22 },	/* BINNING_TYPE */
	{ CCI_REG8(0x0902), 0x00 },
	{ CCI_REG8(0x3004), 0x02 },
	{ CCI_REG8(0x30a5), 0x00 },
	{ CCI_REG8(0x30a8), 0x02 },
	{ CCI_REG8(0x3200), 0x02 },
	{ CCI_REG8(0x4060), 0x00 },
	{ CCI_REG8(0x4061), 0x00 },
	/* Output Size Setting */
	{ CCI_REG8(0x0344), 0x00 },	/* X_ADDR_START */
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x00 },	/* Y_ADDR_START */
	{ CCI_REG8(0x0347), 0x00 },
	{ CCI_REG8(0x0348), 0x0f },	/* X_ADDR_END */
	{ CCI_REG8(0x0349), 0xbf },
	{ CCI_REG8(0x034a), 0x0b },	/* Y_ADDR_END */
	{ CCI_REG8(0x034b), 0xcf },
	{ CCI_REG8(0x034c), 0x07 },	/* X_OUTPUT_SIZE */
	{ CCI_REG8(0x034d), 0xc0 },
	{ CCI_REG8(0x034e), 0x05 },	/* Y_OUTPUT_SIZE */
	{ CCI_REG8(0x034f), 0xd0 },
	{ CCI_REG8(0x0408), 0x00 },	/* DIG_CROP_X_OFFSET */
	{ CCI_REG8(0x0409), 0x10 },
	{ CCI_REG8(0x040a), 0x00 },	/* DIG_CROP_Y_OFFSET */
	{ CCI_REG8(0x040b), 0x0c },
	{ CCI_REG8(0x040c), 0x07 },	/* DIG_CROP_IMAGE_WIDTH */
	{ CCI_REG8(0x040d), 0xc0 },
	{ CCI_REG8(0x040e), 0x05 },	/* DIG_CROP_IMAGE_HEIGHT */
	{ CCI_REG8(0x040f), 0xd0 },
	/* Clock Setting */
	{ CCI_REG8(0x0303), 0x02 },	/* VTSYCK_DIV */
	{ CCI_REG8(0x0305), 0x03 },	/* PREPLLCK_VT_DIV */
	{ CCI_REG8(0x0306), 0x01 },	/* PLL_VT_MPY */
	{ CCI_REG8(0x0307), 0x42 },
	{ CCI_REG8(0x030b), 0x01 },	/* OPSYCK_DIV */
	{ CCI_REG8(0x030d), 0x03 },	/* PREPLLCK_OP_DIV */
	{ CCI_REG8(0x030e), 0x01 },	/* PLL_OP_MPY */
	{ CCI_REG8(0x030f), 0x67 },
	{ CCI_REG8(0x0310), 0x01 },	/* PLL_MULT_DRIV */
	{ CCI_REG8(0x3240), 0x00 },
	/* Other Setting */
	{ CCI_REG8(0x40b8), 0x00 },
	{ CCI_REG8(0x40b9), 0x5c },
	{ CCI_REG8(0x40bc), 0x01 },
	{ CCI_REG8(0x40bd), 0x7a },
	{ CCI_REG8(0x40be), 0x01 },
	{ CCI_REG8(0x40bf), 0x7a },
	{ CCI_REG8(0x7500), 0x02 },
	{ CCI_REG8(0x57aa), 0x06 },
	{ CCI_REG8(0x57ab), 0x5a },
	{ CCI_REG8(0x5620), 0x01 },
	{ CCI_REG8(0x5621), 0xac },
	{ CCI_REG8(0x57a6), 0x00 },
	{ CCI_REG8(0x57a7), 0x00 },
	{ CCI_REG8(0x6d41), 0x13 },
	{ CCI_REG8(0x6d4c), 0x13 },
	{ CCI_REG8(0x6d57), 0x13 },
	{ CCI_REG8(0x6d62), 0x11 },
	{ CCI_REG8(0x6a54), 0x04 },
	{ CCI_REG8(0x6a55), 0x04 },
	{ CCI_REG8(0x6a60), 0x04 },
	{ CCI_REG8(0x6a6b), 0x05 },
	{ CCI_REG8(0x6a75), 0x09 },
	{ CCI_REG8(0x6a76), 0x0a },
	{ CCI_REG8(0x6a80), 0x01 },
	{ CCI_REG8(0x6a81), 0x01 },
	{ CCI_REG8(0x6a8c), 0x01 },
	{ CCI_REG8(0x6a97), 0x02 },
	{ CCI_REG8(0x6aa1), 0x02 },
	{ CCI_REG8(0x6aa2), 0x02 },
	/* Integration Time Setting */
	{ CCI_REG8(0x0224), 0x01 },
	{ CCI_REG8(0x0225), 0xf4 },
	/* Gain Setting */
	{ CCI_REG8(0x0216), 0x00 },
	{ CCI_REG8(0x0217), 0x00 },
	{ CCI_REG8(0x0218), 0x01 },
	{ CCI_REG8(0x0219), 0x00 },
	/* Additional Patch Setting */
	{ CCI_REG8(0x9390), 0x00 },
	/* AEB_LUT */
	{ CCI_REG8(0x0e10), 0x00 },
	{ CCI_REG8(0x0e11), 0x00 },
	{ CCI_REG8(0x0e12), 0x00 },
	{ CCI_REG8(0x0e13), 0x00 },
	{ CCI_REG8(0x0e14), 0x00 },
	{ CCI_REG8(0x0e15), 0x00 },
	{ CCI_REG8(0x0e1e), 0x00 },
	{ CCI_REG8(0x0e1f), 0x00 },
	{ CCI_REG8(0x0e20), 0x00 },
	{ CCI_REG8(0x0e23), 0x00 },
	{ CCI_REG8(0x0e26), 0x00 },
	{ CCI_REG8(0x0e28), 0x00 },
	{ CCI_REG8(0x0e29), 0x00 },
	{ CCI_REG8(0x0e2a), 0x00 },
	{ CCI_REG8(0x0e2b), 0x00 },
	{ CCI_REG8(0x0e2c), 0x00 },
	{ CCI_REG8(0x0e2d), 0x00 },
	{ CCI_REG8(0x0e36), 0x00 },
	{ CCI_REG8(0x0e37), 0x00 },
	{ CCI_REG8(0x0e38), 0x00 },
	{ CCI_REG8(0x0e3b), 0x00 },
	{ CCI_REG8(0x0e3e), 0x00 },
	{ CCI_REG8(0x0e01), 0x00 },
	{ CCI_REG8(0x0e00), 0x00 },
};

struct imx563_reg_list {
	unsigned int num_regs;
	const struct cci_reg_sequence *regs;
};

struct imx563_mode {
	u32 width;
	u32 height;
	/* Line length, in pixels at the pixel rate */
	u32 hts;
	/* Shortest frame length, in lines */
	u32 vts_min;
	/* Shortest integration time, in lines */
	u32 exposure_min;
	/* Readout window, in pixel array coordinates */
	struct v4l2_rect crop;
	unsigned int link_freq_index;
	struct imx563_reg_list reg_list;
};

/*
 * Both modes keep the reference frame length: 30 fps at most. A longer
 * frame (VBLANK) lowers the frame rate and allows longer exposures.
 */
static const struct imx563_mode imx563_modes[] = {
	{
		.width = 4000,
		.height = 3000,
		.hts = 17168,
		.vts_min = 8002,
		.exposure_min = 2,
		.crop = {
			.left = 16,
			.top = 12,
			.width = 4000,
			.height = 3000,
		},
		.link_freq_index = IMX563_LINK_FREQ_INDEX_948M,
		.reg_list = {
			.num_regs = ARRAY_SIZE(imx563_mode_4000x3000_regs),
			.regs = imx563_mode_4000x3000_regs,
		},
	},
	{
		.width = 1984,
		.height = 1488,
		.hts = 7872,
		.vts_min = 17452,
		.exposure_min = 32,
		.crop = {
			.left = 32,
			.top = 24,
			.width = 3968,
			.height = 2976,
		},
		.link_freq_index = IMX563_LINK_FREQ_INDEX_1148M8,
		.reg_list = {
			.num_regs = ARRAY_SIZE(imx563_mode_1984x1488_regs),
			.regs = imx563_mode_1984x1488_regs,
		},
	},
};

/* The preferred default mode, when its link frequency is available */
#define IMX563_DEFAULT_MODE		(&imx563_modes[1])

/*
 * The Bayer order of the output changes with the flips, as the readout
 * window does not move: index with hflip | vflip << 1.
 */
static const u32 imx563_mbus_codes[] = {
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

/* The menu index is the CCS test pattern mode */
static const char * const imx563_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"Eight Vertical Colour Bars",
	"Colour Bars With Fade to Grey",
	"Pseudorandom Sequence (PN9)",
};

enum imx563_supply {
	IMX563_SUPPLY_DOVDD,
	IMX563_SUPPLY_AVDD,
	IMX563_SUPPLY_AVDD2,
	IMX563_SUPPLY_DVDD,
	IMX563_NUM_SUPPLIES
};

/* In power-up order */
static const char * const imx563_supply_names[IMX563_NUM_SUPPLIES] = {
	[IMX563_SUPPLY_DOVDD] = "dovdd",
	[IMX563_SUPPLY_AVDD] = "avdd",
	[IMX563_SUPPLY_AVDD2] = "avdd2",
	[IMX563_SUPPLY_DVDD] = "dvdd",
};

struct imx563 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[IMX563_NUM_SUPPLIES];

	struct v4l2_subdev sd;
	struct media_pad pad;

	/* Link frequencies listed in the endpoint, by imx563_link_freqs index */
	unsigned long link_freq_bitmap;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;

	/* Current mode, protected by the control handler lock */
	const struct imx563_mode *mode;
};

static inline struct imx563 *to_imx563(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx563, sd);
}

static u32 imx563_mbus_code(struct imx563 *imx563)
{
	unsigned int i = 0;

	if (imx563->hflip && imx563->hflip->val)
		i |= 1;
	if (imx563->vflip && imx563->vflip->val)
		i |= 2;

	return imx563_mbus_codes[i];
}

static bool imx563_mode_usable(struct imx563 *imx563,
			       const struct imx563_mode *mode)
{
	return imx563->link_freq_bitmap & BIT(mode->link_freq_index);
}

/* The usable mode closest in size to width x height */
static const struct imx563_mode *imx563_find_mode(struct imx563 *imx563,
						  u32 width, u32 height)
{
	const struct imx563_mode *best = NULL;
	u32 dist, best_dist = U32_MAX;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(imx563_modes); i++) {
		const struct imx563_mode *mode = &imx563_modes[i];

		if (!imx563_mode_usable(imx563, mode))
			continue;

		dist = abs((s32)mode->width - (s32)width) +
		       abs((s32)mode->height - (s32)height);
		if (dist < best_dist) {
			best_dist = dist;
			best = mode;
		}
	}

	return best;
}

static const struct imx563_mode *imx563_default_mode(struct imx563 *imx563)
{
	return imx563_find_mode(imx563, IMX563_DEFAULT_MODE->width,
				IMX563_DEFAULT_MODE->height);
}

/* -----------------------------------------------------------------------------
 * Controls
 */

static void imx563_update_exposure_range(struct imx563 *imx563, s32 vblank)
{
	s64 max = imx563->mode->height + vblank - IMX563_EXPOSURE_MARGIN;

	__v4l2_ctrl_modify_range(imx563->exposure, imx563->exposure->minimum,
				 max, imx563->exposure->step,
				 min_t(s64, imx563->exposure->val, max));
}

static int imx563_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx563 *imx563 = container_of(ctrl->handler, struct imx563,
					     ctrls);
	struct v4l2_subdev_state *state;
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		imx563_update_exposure_range(imx563, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		/* The flips change the Bayer order of the active format */
		if (imx563->sd.active_state) {
			state = v4l2_subdev_get_locked_active_state(&imx563->sd);
			v4l2_subdev_state_get_format(state, 0)->code =
				imx563_mbus_code(imx563);
		}
		break;
	default:
		break;
	}

	/* The registers are written when streaming starts */
	if (pm_runtime_get_if_active(imx563->dev) <= 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(imx563->regmap, IMX563_REG_EXPOSURE, ctrl->val, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(imx563->regmap, IMX563_REG_ANALOG_GAIN, ctrl->val,
			  &ret);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		cci_write(imx563->regmap, IMX563_REG_DIG_GAIN, ctrl->val, &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(imx563->regmap, IMX563_REG_FRAME_LENGTH,
			  imx563->mode->height + ctrl->val, &ret);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		cci_write(imx563->regmap, IMX563_REG_ORIENTATION,
			  (imx563->hflip->val ? IMX563_ORIENTATION_HFLIP : 0) |
			  (imx563->vflip->val ? IMX563_ORIENTATION_VFLIP : 0),
			  &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		if (ctrl->val)
			cci_write(imx563->regmap, IMX563_REG_TEST_PATTERN_PREP,
				  0x00, &ret);
		cci_write(imx563->regmap, IMX563_REG_TEST_PATTERN, ctrl->val,
			  &ret);
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		cci_write(imx563->regmap, IMX563_REG_TEST_DATA_RED, ctrl->val,
			  &ret);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		cci_write(imx563->regmap, IMX563_REG_TEST_DATA_GREENR,
			  ctrl->val, &ret);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		cci_write(imx563->regmap, IMX563_REG_TEST_DATA_BLUE, ctrl->val,
			  &ret);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		cci_write(imx563->regmap, IMX563_REG_TEST_DATA_GREENB,
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

	pm_runtime_put(imx563->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx563_ctrl_ops = {
	.s_ctrl = imx563_set_ctrl,
};

/* Update the mode dependent controls; called with the handler lock held */
static int imx563_update_mode_ctrls(struct imx563 *imx563)
{
	const struct imx563_mode *mode = imx563->mode;
	u32 hblank = mode->hts - mode->width;
	u32 vblank = mode->vts_min - mode->height;
	int ret;

	ret = __v4l2_ctrl_s_ctrl(imx563->link_freq, mode->link_freq_index);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_modify_range(imx563->hblank, hblank, hblank, 1,
				       hblank);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_modify_range(imx563->vblank, vblank,
				       IMX563_FRAME_LENGTH_MAX - mode->height,
				       1, vblank);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_s_ctrl(imx563->vblank, vblank);
	if (ret)
		return ret;

	/* Default to the longest exposure the frame allows */
	return __v4l2_ctrl_modify_range(imx563->exposure, mode->exposure_min,
					mode->vts_min - IMX563_EXPOSURE_MARGIN,
					IMX563_EXPOSURE_STEP,
					mode->vts_min - IMX563_EXPOSURE_MARGIN);
}

static int imx563_init_controls(struct imx563 *imx563)
{
	struct v4l2_ctrl_handler *hdl = &imx563->ctrls;
	const struct imx563_mode *mode = imx563->mode;
	struct v4l2_fwnode_device_properties props;
	u32 hblank = mode->hts - mode->width;
	u32 vblank = mode->vts_min - mode->height;
	u32 exposure_max = mode->vts_min - IMX563_EXPOSURE_MARGIN;
	unsigned int i;
	int ret;

	ret = v4l2_fwnode_device_parse(imx563->dev, &props);
	if (ret)
		return ret;

	v4l2_ctrl_handler_init(hdl, 16);

	imx563->link_freq = v4l2_ctrl_new_int_menu(hdl, &imx563_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(imx563_link_freqs) - 1,
						   mode->link_freq_index,
						   imx563_link_freqs);
	if (imx563->link_freq) {
		imx563->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
		/* Only the link frequencies of the endpoint */
		imx563->link_freq->menu_skip_mask = ~imx563->link_freq_bitmap;
	}

	imx563->pixel_rate = v4l2_ctrl_new_std(hdl, &imx563_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       IMX563_PIXEL_RATE,
					       IMX563_PIXEL_RATE, 1,
					       IMX563_PIXEL_RATE);

	imx563->hblank = v4l2_ctrl_new_std(hdl, &imx563_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank, 1,
					   hblank);
	if (imx563->hblank)
		imx563->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx563->vblank = v4l2_ctrl_new_std(hdl, &imx563_ctrl_ops,
					   V4L2_CID_VBLANK, vblank,
					   IMX563_FRAME_LENGTH_MAX - mode->height,
					   1, vblank);

	imx563->exposure = v4l2_ctrl_new_std(hdl, &imx563_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     mode->exposure_min, exposure_max,
					     IMX563_EXPOSURE_STEP,
					     exposure_max);

	v4l2_ctrl_new_std(hdl, &imx563_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX563_ANA_GAIN_MIN, IMX563_ANA_GAIN_MAX,
			  IMX563_ANA_GAIN_STEP, IMX563_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(hdl, &imx563_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX563_DGTL_GAIN_MIN, IMX563_DGTL_GAIN_MAX,
			  IMX563_DGTL_GAIN_STEP, IMX563_DGTL_GAIN_DEFAULT);

	imx563->hflip = v4l2_ctrl_new_std(hdl, &imx563_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (imx563->hflip)
		imx563->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	imx563->vflip = v4l2_ctrl_new_std(hdl, &imx563_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (imx563->vflip)
		imx563->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(hdl, &imx563_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx563_test_pattern_menu) - 1,
				     0, 0, imx563_test_pattern_menu);
	for (i = 0; i < 4; i++)
		v4l2_ctrl_new_std(hdl, &imx563_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED + i,
				  0, IMX563_TEST_DATA_MAX, 1,
				  IMX563_TEST_DATA_MAX);

	v4l2_ctrl_new_fwnode_properties(hdl, &imx563_ctrl_ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		dev_err(imx563->dev, "failed to add controls: %d\n", ret);
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	imx563->sd.ctrl_handler = hdl;

	return 0;
}

/* -----------------------------------------------------------------------------
 * Power
 */

static int imx563_power_on(struct device *dev)
{
	struct imx563 *imx563 = to_imx563(dev_get_drvdata(dev));
	int ret;
	int i;

	/*
	 * Enable the rails one at a time: regulator_bulk_enable() does not
	 * keep the order.
	 */
	for (i = 0; i < IMX563_NUM_SUPPLIES; i++) {
		ret = regulator_enable(imx563->supplies[i].consumer);
		if (ret) {
			dev_err(dev, "failed to enable %s: %d\n",
				imx563->supplies[i].supply, ret);
			goto err_supplies;
		}
		usleep_range(1000, 1100);
	}

	ret = clk_prepare_enable(imx563->mclk);
	if (ret) {
		dev_err(dev, "failed to enable MCLK: %d\n", ret);
		goto err_supplies;
	}
	usleep_range(1000, 1100);

	gpiod_set_value_cansleep(imx563->reset_gpio, 0);
	usleep_range(IMX563_XCLR_DELAY_US, IMX563_XCLR_DELAY_US + 1000);

	return 0;

err_supplies:
	while (--i >= 0)
		regulator_disable(imx563->supplies[i].consumer);

	return ret;
}

static int imx563_power_off(struct device *dev)
{
	struct imx563 *imx563 = to_imx563(dev_get_drvdata(dev));
	int i;

	gpiod_set_value_cansleep(imx563->reset_gpio, 1);
	usleep_range(1000, 1100);

	clk_disable_unprepare(imx563->mclk);

	for (i = IMX563_NUM_SUPPLIES - 1; i >= 0; i--)
		regulator_disable(imx563->supplies[i].consumer);

	return 0;
}

/* -----------------------------------------------------------------------------
 * Subdev
 */

static int imx563_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct imx563 *imx563 = to_imx563(sd);
	const struct imx563_mode *mode = imx563->mode;
	int ret;

	ret = pm_runtime_resume_and_get(imx563->dev);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(imx563->regmap, imx563_init_regs,
				  ARRAY_SIZE(imx563_init_regs), NULL);
	if (ret) {
		dev_err(imx563->dev, "failed to write init registers: %d\n",
			ret);
		goto err_rpm_put;
	}

	ret = cci_multi_reg_write(imx563->regmap, mode->reg_list.regs,
				  mode->reg_list.num_regs, NULL);
	if (!ret)
		ret = cci_multi_reg_write(imx563->regmap, imx563_output_regs,
					  ARRAY_SIZE(imx563_output_regs), NULL);
	if (!ret)
		ret = cci_write(imx563->regmap, IMX563_REG_LINE_LENGTH,
				mode->hts, NULL);
	if (ret) {
		dev_err(imx563->dev, "failed to write mode registers: %d\n",
			ret);
		goto err_rpm_put;
	}

	/* Exposure, gains, frame length, flips and test pattern */
	ret = __v4l2_ctrl_handler_setup(&imx563->ctrls);
	if (ret)
		goto err_rpm_put;

	ret = cci_write(imx563->regmap, IMX563_REG_MODE_SELECT,
			IMX563_MODE_STREAMING, NULL);
	if (ret) {
		dev_err(imx563->dev, "failed to start streaming: %d\n", ret);
		goto err_rpm_put;
	}

	/* The flips change the Bayer order: not while streaming */
	__v4l2_ctrl_grab(imx563->hflip, true);
	__v4l2_ctrl_grab(imx563->vflip, true);

	return 0;

err_rpm_put:
	pm_runtime_put_autosuspend(imx563->dev);

	return ret;
}

static int imx563_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct imx563 *imx563 = to_imx563(sd);
	int ret;

	ret = cci_write(imx563->regmap, IMX563_REG_MODE_SELECT,
			IMX563_MODE_STANDBY, NULL);
	if (ret)
		dev_err(imx563->dev, "failed to stop streaming: %d\n", ret);

	__v4l2_ctrl_grab(imx563->hflip, false);
	__v4l2_ctrl_grab(imx563->vflip, false);

	pm_runtime_put_autosuspend(imx563->dev);

	return ret;
}

static void imx563_fill_format(struct imx563 *imx563,
			       const struct imx563_mode *mode,
			       struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = imx563_mbus_code(imx563);
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int imx563_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = imx563_mbus_code(to_imx563(sd));

	return 0;
}

static int imx563_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx563 *imx563 = to_imx563(sd);
	unsigned int i, index = 0;

	if (fse->code != imx563_mbus_code(imx563))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(imx563_modes); i++) {
		const struct imx563_mode *mode = &imx563_modes[i];

		if (!imx563_mode_usable(imx563, mode))
			continue;

		if (index++ == fse->index) {
			fse->min_width = mode->width;
			fse->max_width = mode->width;
			fse->min_height = mode->height;
			fse->max_height = mode->height;
			return 0;
		}
	}

	return -EINVAL;
}

static int imx563_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct imx563 *imx563 = to_imx563(sd);
	const struct imx563_mode *mode;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    v4l2_subdev_is_streaming(sd))
		return -EBUSY;

	mode = imx563_find_mode(imx563, fmt->format.width, fmt->format.height);

	imx563_fill_format(imx563, mode, &fmt->format);
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;
	*v4l2_subdev_state_get_crop(state, 0) = mode->crop;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		imx563->mode = mode;
		return imx563_update_mode_ctrls(imx563);
	}

	return 0;
}

static int imx563_get_selection(struct v4l2_subdev *sd,
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
		sel->r.width = IMX563_NATIVE_WIDTH;
		sel->r.height = IMX563_NATIVE_HEIGHT;
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = IMX563_PIXEL_ARRAY_LEFT;
		sel->r.top = IMX563_PIXEL_ARRAY_TOP;
		sel->r.width = IMX563_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX563_PIXEL_ARRAY_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static int imx563_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	const struct imx563_mode *mode = imx563_default_mode(to_imx563(sd));
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.width = mode->width,
			.height = mode->height,
		},
	};

	return imx563_set_fmt(sd, state, &fmt);
}

static const struct v4l2_subdev_video_ops imx563_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops imx563_pad_ops = {
	.enum_mbus_code = imx563_enum_mbus_code,
	.enum_frame_size = imx563_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx563_set_fmt,
	.get_selection = imx563_get_selection,
	.enable_streams = imx563_enable_streams,
	.disable_streams = imx563_disable_streams,
};

static const struct v4l2_subdev_ops imx563_subdev_ops = {
	.video = &imx563_video_ops,
	.pad = &imx563_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx563_internal_ops = {
	.init_state = imx563_init_state,
};

/* -----------------------------------------------------------------------------
 * Probe
 */

static int imx563_identify(struct imx563 *imx563)
{
	u64 id;
	int ret;

	ret = cci_read(imx563->regmap, IMX563_REG_MODEL_ID, &id, NULL);
	if (ret)
		return dev_err_probe(imx563->dev, ret,
				     "failed to read model ID\n");

	if (id != IMX563_MODEL_ID)
		return dev_err_probe(imx563->dev, -ENODEV,
				     "unexpected model ID 0x%04llx (want 0x%04x)\n",
				     id, IMX563_MODEL_ID);

	dev_info(imx563->dev, "Sony IMX563 detected, model ID 0x%04llx\n", id);

	return 0;
}

static int imx563_check_hwcfg(struct imx563 *imx563)
{
	struct device *dev = imx563->dev;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_UNKNOWN,
	};
	struct fwnode_handle *ep;
	unsigned int i;
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!ep)
		return dev_err_probe(dev, -EINVAL, "no endpoint\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(dev, ret, "failed to parse endpoint\n");

	/* Without a bus-type, an endpoint with data-lanes parses as D-PHY */
	if (bus_cfg.bus_type != V4L2_MBUS_CSI2_CPHY) {
		ret = dev_err_probe(dev, -EINVAL,
				    "bus type %u not supported, need C-PHY\n",
				    bus_cfg.bus_type);
		goto out;
	}

	/* The trios are not remapped: the sensor's default order only */
	if (bus_cfg.bus.mipi_csi2.num_data_lanes != IMX563_DATA_LANES) {
		ret = dev_err_probe(dev, -EINVAL,
				    "%u data lanes not supported, need %u\n",
				    bus_cfg.bus.mipi_csi2.num_data_lanes,
				    IMX563_DATA_LANES);
		goto out;
	}

	for (i = 0; i < IMX563_DATA_LANES; i++) {
		if (bus_cfg.bus.mipi_csi2.data_lanes[i] != i + 1) {
			ret = dev_err_probe(dev, -EINVAL,
					    "data lane remapping not supported\n");
			goto out;
		}
	}

	ret = v4l2_link_freq_to_bitmap(dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       imx563_link_freqs,
				       ARRAY_SIZE(imx563_link_freqs),
				       &imx563->link_freq_bitmap);

out:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int imx563_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx563 *imx563;
	unsigned long rate;
	int ret;
	int i;

	imx563 = devm_kzalloc(dev, sizeof(*imx563), GFP_KERNEL);
	if (!imx563)
		return -ENOMEM;

	imx563->dev = dev;

	ret = imx563_check_hwcfg(imx563);
	if (ret)
		return ret;

	/* Also sets the client data to the subdev */
	v4l2_i2c_subdev_init(&imx563->sd, client, &imx563_subdev_ops);

	imx563->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx563->regmap))
		return dev_err_probe(dev, PTR_ERR(imx563->regmap),
				     "failed to initialize CCI\n");

	for (i = 0; i < IMX563_NUM_SUPPLIES; i++)
		imx563->supplies[i].supply = imx563_supply_names[i];

	ret = devm_regulator_bulk_get(dev, IMX563_NUM_SUPPLIES,
				      imx563->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get supplies\n");

	imx563->mclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(imx563->mclk))
		return dev_err_probe(dev, PTR_ERR(imx563->mclk),
				     "failed to get MCLK\n");

	rate = clk_get_rate(imx563->mclk);
	if (rate != IMX563_MCLK_RATE)
		return dev_err_probe(dev, -EINVAL,
				     "unsupported MCLK rate %lu, need %u\n",
				     rate, IMX563_MCLK_RATE);

	/* Hold the sensor in reset until it is powered */
	imx563->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(imx563->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(imx563->reset_gpio),
				     "failed to get reset GPIO\n");

	ret = imx563_power_on(dev);
	if (ret)
		return ret;

	ret = imx563_identify(imx563);
	if (ret)
		goto err_power_off;

	imx563->mode = imx563_default_mode(imx563);

	ret = imx563_init_controls(imx563);
	if (ret)
		goto err_power_off;

	imx563->sd.internal_ops = &imx563_internal_ops;
	imx563->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx563->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx563->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx563->sd.entity, 1, &imx563->pad);
	if (ret)
		goto err_ctrls;

	imx563->sd.state_lock = imx563->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&imx563->sd);
	if (ret)
		goto err_entity;

	/* Keep the sensor powered until it is idled below */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&imx563->sd);
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
	v4l2_subdev_cleanup(&imx563->sd);
err_entity:
	media_entity_cleanup(&imx563->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&imx563->ctrls);
err_power_off:
	imx563_power_off(dev);

	return ret;
}

static void imx563_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx563 *imx563 = to_imx563(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&imx563->ctrls);

	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		imx563_power_off(dev);
	pm_runtime_set_suspended(dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx563_pm_ops, imx563_power_off,
				 imx563_power_on, NULL);

static const struct of_device_id imx563_of_match[] = {
	{ .compatible = "sony,imx563" },
	{ }
};
MODULE_DEVICE_TABLE(of, imx563_of_match);

static struct i2c_driver imx563_driver = {
	.driver = {
		.name = "imx563",
		.of_match_table = imx563_of_match,
		.pm = pm_ptr(&imx563_pm_ops),
	},
	.probe = imx563_probe,
	.remove = imx563_remove,
};
module_i2c_driver(imx563_driver);

MODULE_DESCRIPTION("Sony IMX563 image sensor driver");
MODULE_LICENSE("GPL");

// SPDX-License-Identifier: GPL-2.0
/*
 * Sony IMX766 image sensor driver
 *
 * The IMX766 is a 50 Mpixel (8192x6144) Quad Bayer sensor with a MIPI CSI-2
 * output over three C-PHY trios and an SMIA++ style register map. Only the
 * binned modes, which output ordinary Bayer data, are supported. The
 * standard registers (mode select, frame and line length, integration
 * time, gains, orientation, test pattern) are driven from here. The global
 * settings and the mode tables, including the PLL and C-PHY timing, come
 * from the GPL-2.0 MediaTek imgsensor driver for this sensor.
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

#define IMX766_REG_MODEL_ID		CCI_REG16(0x0016)
#define IMX766_MODEL_ID			0x0766

#define IMX766_REG_MODE_SELECT		CCI_REG8(0x0100)
#define IMX766_MODE_STANDBY		0x00
#define IMX766_MODE_STREAMING		0x01

#define IMX766_REG_ORIENTATION		CCI_REG8(0x0101)
#define IMX766_ORIENTATION_HFLIP	BIT(0)
#define IMX766_ORIENTATION_VFLIP	BIT(1)

/* Coarse integration time, in lines */
#define IMX766_REG_EXPOSURE		CCI_REG16(0x0202)
#define IMX766_EXPOSURE_MIN		24
#define IMX766_EXPOSURE_STEP		4
/* Lines between the end of the integration time and the frame length */
#define IMX766_EXPOSURE_MARGIN		48

/* Analogue gain = 16384 / (16384 - code), 1x to 64x */
#define IMX766_REG_ANALOG_GAIN		CCI_REG16(0x0204)
#define IMX766_ANA_GAIN_MIN		0
#define IMX766_ANA_GAIN_MAX		16128
#define IMX766_ANA_GAIN_STEP		1
#define IMX766_ANA_GAIN_DEFAULT		IMX766_ANA_GAIN_MIN

/* Global digital gain, U8.8 */
#define IMX766_REG_DIG_GAIN		CCI_REG16(0x020e)
#define IMX766_DGTL_GAIN_MIN		0x0100
#define IMX766_DGTL_GAIN_MAX		0x0fff
#define IMX766_DGTL_GAIN_STEP		1
#define IMX766_DGTL_GAIN_DEFAULT	0x0100

#define IMX766_REG_FRAME_LENGTH		CCI_REG16(0x0340)
#define IMX766_FRAME_LENGTH_MAX		0xffff

#define IMX766_REG_TEST_PATTERN		CCI_REG16(0x0600)

#define IMX766_MCLK_RATE		24000000

/* Delay from XCLR release to the first I2C access */
#define IMX766_XCLR_DELAY_US		10000

/* Three C-PHY trios */
#define IMX766_DATA_LANES		3

/*
 * PLL, from a 24 MHz INCK:
 *  - video timing (both modes): 24 MHz / 4 * 310 = 1860 MHz, / 5 = 372 MHz,
 *    four pixels per clock: a 1488 Mpixel/s pixel rate in line length
 *    units.
 *  - output, 4096x3072: 24 MHz / 3 * 287 = 2296 MHz, / 2 = 1148 Msym/s per
 *    trio, a 574 MHz link frequency.
 *  - output, 2048x1536: 24 MHz / 3 * 431 = 3448 MHz, / 4 = 862 Msym/s per
 *    trio, a 431 MHz link frequency.
 * The C-PHY link frequency is half the symbol rate per trio.
 */
#define IMX766_PIXEL_RATE		1488000000LL
#define IMX766_LINK_FREQ_431MHZ		431000000LL
#define IMX766_LINK_FREQ_574MHZ		574000000LL

/* Pixel array */
#define IMX766_NATIVE_WIDTH		8192
#define IMX766_NATIVE_HEIGHT		6144
#define IMX766_PIXEL_ARRAY_LEFT		0
#define IMX766_PIXEL_ARRAY_TOP		0
#define IMX766_PIXEL_ARRAY_WIDTH	8192
#define IMX766_PIXEL_ARRAY_HEIGHT	6144

enum {
	IMX766_LINK_FREQ_431MHZ_INDEX,
	IMX766_LINK_FREQ_574MHZ_INDEX,
};

static const s64 imx766_link_freqs[] = {
	[IMX766_LINK_FREQ_431MHZ_INDEX] = IMX766_LINK_FREQ_431MHZ,
	[IMX766_LINK_FREQ_574MHZ_INDEX] = IMX766_LINK_FREQ_574MHZ,
};

/*
 * Global settings, written after every power up. The orientation register
 * (0x0101) is left to the flip controls.
 */
static const struct cci_reg_sequence imx766_init_regs[] = {
	/* External clock: 24 MHz */
	{ CCI_REG8(0x0136), 0x18 },
	{ CCI_REG8(0x0137), 0x00 },
	{ CCI_REG8(0x33f0), 0x0d },
	{ CCI_REG8(0x33f1), 0x08 },
	/* CSI-2 signalling mode: C-PHY */
	{ CCI_REG8(0x0111), 0x03 },
	/* Global and image quality settings */
	{ CCI_REG8(0x33d3), 0x01 },
	{ CCI_REG8(0x3892), 0x01 },
	{ CCI_REG8(0x4c14), 0x00 },
	{ CCI_REG8(0x4c15), 0x07 },
	{ CCI_REG8(0x4c16), 0x00 },
	{ CCI_REG8(0x4c17), 0x1b },
	{ CCI_REG8(0x4c1a), 0x00 },
	{ CCI_REG8(0x4c1b), 0x03 },
	{ CCI_REG8(0x4c1c), 0x00 },
	{ CCI_REG8(0x4c1d), 0x00 },
	{ CCI_REG8(0x4c1e), 0x00 },
	{ CCI_REG8(0x4c1f), 0x02 },
	{ CCI_REG8(0x4c20), 0x00 },
	{ CCI_REG8(0x4c21), 0x5f },
	{ CCI_REG8(0x4c26), 0x00 },
	{ CCI_REG8(0x4c27), 0x43 },
	{ CCI_REG8(0x4c28), 0x00 },
	{ CCI_REG8(0x4c29), 0x09 },
	{ CCI_REG8(0x4c2a), 0x00 },
	{ CCI_REG8(0x4c2b), 0x4a },
	{ CCI_REG8(0x4c2c), 0x00 },
	{ CCI_REG8(0x4c2d), 0x00 },
	{ CCI_REG8(0x4c2e), 0x00 },
	{ CCI_REG8(0x4c2f), 0x02 },
	{ CCI_REG8(0x4c30), 0x00 },
	{ CCI_REG8(0x4c31), 0xc6 },
	{ CCI_REG8(0x4c3e), 0x00 },
	{ CCI_REG8(0x4c3f), 0x55 },
	{ CCI_REG8(0x4c52), 0x00 },
	{ CCI_REG8(0x4c53), 0x97 },
	{ CCI_REG8(0x4cb4), 0x00 },
	{ CCI_REG8(0x4cb5), 0x55 },
	{ CCI_REG8(0x4cc8), 0x00 },
	{ CCI_REG8(0x4cc9), 0x97 },
	{ CCI_REG8(0x4d04), 0x00 },
	{ CCI_REG8(0x4d05), 0x4f },
	{ CCI_REG8(0x4d74), 0x00 },
	{ CCI_REG8(0x4d75), 0x55 },
	{ CCI_REG8(0x4f06), 0x00 },
	{ CCI_REG8(0x4f07), 0x5f },
	{ CCI_REG8(0x4f48), 0x00 },
	{ CCI_REG8(0x4f49), 0xc6 },
	{ CCI_REG8(0x544a), 0xff },
	{ CCI_REG8(0x544b), 0xff },
	{ CCI_REG8(0x544e), 0x01 },
	{ CCI_REG8(0x544f), 0xbd },
	{ CCI_REG8(0x5452), 0xff },
	{ CCI_REG8(0x5453), 0xff },
	{ CCI_REG8(0x5456), 0x00 },
	{ CCI_REG8(0x5457), 0xa5 },
	{ CCI_REG8(0x545a), 0xff },
	{ CCI_REG8(0x545b), 0xff },
	{ CCI_REG8(0x545e), 0x00 },
	{ CCI_REG8(0x545f), 0xa5 },
	{ CCI_REG8(0x5496), 0x00 },
	{ CCI_REG8(0x5497), 0xa2 },
	{ CCI_REG8(0x54f6), 0x01 },
	{ CCI_REG8(0x54f7), 0x55 },
	{ CCI_REG8(0x54f8), 0x01 },
	{ CCI_REG8(0x54f9), 0x61 },
	{ CCI_REG8(0x5670), 0x00 },
	{ CCI_REG8(0x5671), 0x85 },
	{ CCI_REG8(0x5672), 0x01 },
	{ CCI_REG8(0x5673), 0x77 },
	{ CCI_REG8(0x5674), 0x01 },
	{ CCI_REG8(0x5675), 0x2f },
	{ CCI_REG8(0x5676), 0x02 },
	{ CCI_REG8(0x5677), 0x55 },
	{ CCI_REG8(0x5678), 0x00 },
	{ CCI_REG8(0x5679), 0x85 },
	{ CCI_REG8(0x567a), 0x01 },
	{ CCI_REG8(0x567b), 0x77 },
	{ CCI_REG8(0x567c), 0x01 },
	{ CCI_REG8(0x567d), 0x2f },
	{ CCI_REG8(0x567e), 0x02 },
	{ CCI_REG8(0x567f), 0x55 },
	{ CCI_REG8(0x5680), 0x00 },
	{ CCI_REG8(0x5681), 0x85 },
	{ CCI_REG8(0x5682), 0x01 },
	{ CCI_REG8(0x5683), 0x77 },
	{ CCI_REG8(0x5684), 0x01 },
	{ CCI_REG8(0x5685), 0x2f },
	{ CCI_REG8(0x5686), 0x02 },
	{ CCI_REG8(0x5687), 0x55 },
	{ CCI_REG8(0x5688), 0x00 },
	{ CCI_REG8(0x5689), 0x85 },
	{ CCI_REG8(0x568a), 0x01 },
	{ CCI_REG8(0x568b), 0x77 },
	{ CCI_REG8(0x568c), 0x01 },
	{ CCI_REG8(0x568d), 0x2f },
	{ CCI_REG8(0x568e), 0x02 },
	{ CCI_REG8(0x568f), 0x55 },
	{ CCI_REG8(0x5690), 0x01 },
	{ CCI_REG8(0x5691), 0x7a },
	{ CCI_REG8(0x5692), 0x02 },
	{ CCI_REG8(0x5693), 0x6c },
	{ CCI_REG8(0x5694), 0x01 },
	{ CCI_REG8(0x5695), 0x35 },
	{ CCI_REG8(0x5696), 0x02 },
	{ CCI_REG8(0x5697), 0x5b },
	{ CCI_REG8(0x5698), 0x01 },
	{ CCI_REG8(0x5699), 0x7a },
	{ CCI_REG8(0x569a), 0x02 },
	{ CCI_REG8(0x569b), 0x6c },
	{ CCI_REG8(0x569c), 0x01 },
	{ CCI_REG8(0x569d), 0x35 },
	{ CCI_REG8(0x569e), 0x02 },
	{ CCI_REG8(0x569f), 0x5b },
	{ CCI_REG8(0x56a0), 0x01 },
	{ CCI_REG8(0x56a1), 0x7a },
	{ CCI_REG8(0x56a2), 0x02 },
	{ CCI_REG8(0x56a3), 0x6c },
	{ CCI_REG8(0x56a4), 0x01 },
	{ CCI_REG8(0x56a5), 0x35 },
	{ CCI_REG8(0x56a6), 0x02 },
	{ CCI_REG8(0x56a7), 0x5b },
	{ CCI_REG8(0x56a8), 0x01 },
	{ CCI_REG8(0x56a9), 0x80 },
	{ CCI_REG8(0x56aa), 0x02 },
	{ CCI_REG8(0x56ab), 0x72 },
	{ CCI_REG8(0x56ac), 0x01 },
	{ CCI_REG8(0x56ad), 0x2f },
	{ CCI_REG8(0x56ae), 0x02 },
	{ CCI_REG8(0x56af), 0x55 },
	{ CCI_REG8(0x5902), 0x0e },
	{ CCI_REG8(0x5a50), 0x04 },
	{ CCI_REG8(0x5a51), 0x04 },
	{ CCI_REG8(0x5a69), 0x01 },
	{ CCI_REG8(0x5c49), 0x0d },
	{ CCI_REG8(0x5d60), 0x08 },
	{ CCI_REG8(0x5d61), 0x08 },
	{ CCI_REG8(0x5d62), 0x08 },
	{ CCI_REG8(0x5d63), 0x08 },
	{ CCI_REG8(0x5d64), 0x08 },
	{ CCI_REG8(0x5d67), 0x08 },
	{ CCI_REG8(0x5d6c), 0x08 },
	{ CCI_REG8(0x5d6e), 0x08 },
	{ CCI_REG8(0x5d71), 0x08 },
	{ CCI_REG8(0x5d8e), 0x14 },
	{ CCI_REG8(0x5d90), 0x03 },
	{ CCI_REG8(0x5d91), 0x0a },
	{ CCI_REG8(0x5d92), 0x1f },
	{ CCI_REG8(0x5d93), 0x05 },
	{ CCI_REG8(0x5d97), 0x1f },
	{ CCI_REG8(0x5d9a), 0x06 },
	{ CCI_REG8(0x5d9c), 0x1f },
	{ CCI_REG8(0x5da1), 0x1f },
	{ CCI_REG8(0x5da6), 0x1f },
	{ CCI_REG8(0x5da8), 0x1f },
	{ CCI_REG8(0x5dab), 0x1f },
	{ CCI_REG8(0x5dc0), 0x06 },
	{ CCI_REG8(0x5dc1), 0x06 },
	{ CCI_REG8(0x5dc2), 0x07 },
	{ CCI_REG8(0x5dc3), 0x06 },
	{ CCI_REG8(0x5dc4), 0x07 },
	{ CCI_REG8(0x5dc7), 0x07 },
	{ CCI_REG8(0x5dcc), 0x07 },
	{ CCI_REG8(0x5dce), 0x07 },
	{ CCI_REG8(0x5dd1), 0x07 },
	{ CCI_REG8(0x5e3e), 0x00 },
	{ CCI_REG8(0x5e3f), 0x00 },
	{ CCI_REG8(0x5e41), 0x00 },
	{ CCI_REG8(0x5e48), 0x00 },
	{ CCI_REG8(0x5e49), 0x00 },
	{ CCI_REG8(0x5e4a), 0x00 },
	{ CCI_REG8(0x5e4c), 0x00 },
	{ CCI_REG8(0x5e4d), 0x00 },
	{ CCI_REG8(0x5e4e), 0x00 },
	{ CCI_REG8(0x6026), 0x03 },
	{ CCI_REG8(0x6028), 0x03 },
	{ CCI_REG8(0x602a), 0x03 },
	{ CCI_REG8(0x602c), 0x03 },
	{ CCI_REG8(0x602f), 0x03 },
	{ CCI_REG8(0x6036), 0x03 },
	{ CCI_REG8(0x6038), 0x03 },
	{ CCI_REG8(0x603a), 0x03 },
	{ CCI_REG8(0x603c), 0x03 },
	{ CCI_REG8(0x603f), 0x03 },
	{ CCI_REG8(0x6074), 0x19 },
	{ CCI_REG8(0x6076), 0x19 },
	{ CCI_REG8(0x6078), 0x19 },
	{ CCI_REG8(0x607a), 0x19 },
	{ CCI_REG8(0x607d), 0x19 },
	{ CCI_REG8(0x6084), 0x32 },
	{ CCI_REG8(0x6086), 0x32 },
	{ CCI_REG8(0x6088), 0x32 },
	{ CCI_REG8(0x608a), 0x32 },
	{ CCI_REG8(0x608d), 0x32 },
	{ CCI_REG8(0x60c2), 0x4a },
	{ CCI_REG8(0x60c4), 0x4a },
	{ CCI_REG8(0x60cb), 0x4a },
	{ CCI_REG8(0x60d2), 0x4a },
	{ CCI_REG8(0x60d4), 0x4a },
	{ CCI_REG8(0x60db), 0x4a },
	{ CCI_REG8(0x62f9), 0x14 },
	{ CCI_REG8(0x6305), 0x13 },
	{ CCI_REG8(0x6307), 0x13 },
	{ CCI_REG8(0x630a), 0x13 },
	{ CCI_REG8(0x630d), 0x0d },
	{ CCI_REG8(0x6317), 0x0d },
	{ CCI_REG8(0x632f), 0x2e },
	{ CCI_REG8(0x6333), 0x2e },
	{ CCI_REG8(0x6339), 0x2e },
	{ CCI_REG8(0x6343), 0x2e },
	{ CCI_REG8(0x6347), 0x2e },
	{ CCI_REG8(0x634d), 0x2e },
	{ CCI_REG8(0x6352), 0x00 },
	{ CCI_REG8(0x6353), 0x5f },
	{ CCI_REG8(0x6366), 0x00 },
	{ CCI_REG8(0x6367), 0x5f },
	{ CCI_REG8(0x638f), 0x95 },
	{ CCI_REG8(0x6393), 0x95 },
	{ CCI_REG8(0x6399), 0x95 },
	{ CCI_REG8(0x63a3), 0x95 },
	{ CCI_REG8(0x63a7), 0x95 },
	{ CCI_REG8(0x63ad), 0x95 },
	{ CCI_REG8(0x63b2), 0x00 },
	{ CCI_REG8(0x63b3), 0xc6 },
	{ CCI_REG8(0x63c6), 0x00 },
	{ CCI_REG8(0x63c7), 0xc6 },
	{ CCI_REG8(0x8bdb), 0x02 },
	{ CCI_REG8(0x8bde), 0x02 },
	{ CCI_REG8(0x8be1), 0x2d },
	{ CCI_REG8(0x8be4), 0x00 },
	{ CCI_REG8(0x8be5), 0x00 },
	{ CCI_REG8(0x8be6), 0x01 },
	{ CCI_REG8(0x9002), 0x14 },
	{ CCI_REG8(0x9200), 0xb5 },
	{ CCI_REG8(0x9201), 0x9e },
	{ CCI_REG8(0x9202), 0xb5 },
	{ CCI_REG8(0x9203), 0x42 },
	{ CCI_REG8(0x9204), 0xb5 },
	{ CCI_REG8(0x9205), 0x43 },
	{ CCI_REG8(0x9206), 0xbd },
	{ CCI_REG8(0x9207), 0x20 },
	{ CCI_REG8(0x9208), 0xbd },
	{ CCI_REG8(0x9209), 0x22 },
	{ CCI_REG8(0x920a), 0xbd },
	{ CCI_REG8(0x920b), 0x23 },
	{ CCI_REG8(0xb5d7), 0x10 },
	{ CCI_REG8(0xbd24), 0x00 },
	{ CCI_REG8(0xbd25), 0x00 },
	{ CCI_REG8(0xbd26), 0x00 },
	{ CCI_REG8(0xbd27), 0x00 },
	{ CCI_REG8(0xbd28), 0x00 },
	{ CCI_REG8(0xbd29), 0x00 },
	{ CCI_REG8(0xbd2a), 0x00 },
	{ CCI_REG8(0xbd2b), 0x00 },
	{ CCI_REG8(0xbd2c), 0x32 },
	{ CCI_REG8(0xbd2d), 0x70 },
	{ CCI_REG8(0xbd2e), 0x25 },
	{ CCI_REG8(0xbd2f), 0x30 },
	{ CCI_REG8(0xbd30), 0x3b },
	{ CCI_REG8(0xbd31), 0xe0 },
	{ CCI_REG8(0xbd32), 0x69 },
	{ CCI_REG8(0xbd33), 0x40 },
	{ CCI_REG8(0xbd34), 0x25 },
	{ CCI_REG8(0xbd35), 0x90 },
	{ CCI_REG8(0xbd36), 0x58 },
	{ CCI_REG8(0xbd37), 0x00 },
	{ CCI_REG8(0xbd38), 0x00 },
	{ CCI_REG8(0xbd39), 0x00 },
	{ CCI_REG8(0xbd3a), 0x00 },
	{ CCI_REG8(0xbd3b), 0x00 },
	{ CCI_REG8(0xbd3c), 0x32 },
	{ CCI_REG8(0xbd3d), 0x70 },
	{ CCI_REG8(0xbd3e), 0x25 },
	{ CCI_REG8(0xbd3f), 0x90 },
	{ CCI_REG8(0xbd40), 0x58 },
	{ CCI_REG8(0xbd41), 0x00 },
	{ CCI_REG8(0x793b), 0x01 },
	{ CCI_REG8(0xacc6), 0x00 },
	{ CCI_REG8(0xacf5), 0x00 },
	{ CCI_REG8(0x793b), 0x00 },
	{ CCI_REG8(0x1f04), 0xb3 },
	{ CCI_REG8(0x1f05), 0x01 },
	{ CCI_REG8(0x1f06), 0x07 },
	{ CCI_REG8(0x1f07), 0x66 },
	{ CCI_REG8(0x1f08), 0x01 },
	{ CCI_REG8(0x4d18), 0x00 },
	{ CCI_REG8(0x4d19), 0x9d },
	{ CCI_REG8(0x4d88), 0x00 },
	{ CCI_REG8(0x4d89), 0x97 },
	{ CCI_REG8(0x5c57), 0x0a },
	{ CCI_REG8(0x5d94), 0x1f },
	{ CCI_REG8(0x5d9e), 0x1f },
	{ CCI_REG8(0x5e50), 0x23 },
	{ CCI_REG8(0x5e51), 0x20 },
	{ CCI_REG8(0x5e52), 0x07 },
	{ CCI_REG8(0x5e53), 0x20 },
	{ CCI_REG8(0x5e54), 0x07 },
	{ CCI_REG8(0x5e55), 0x27 },
	{ CCI_REG8(0x5e56), 0x0b },
	{ CCI_REG8(0x5e57), 0x24 },
	{ CCI_REG8(0x5e58), 0x0b },
	{ CCI_REG8(0x5e60), 0x24 },
	{ CCI_REG8(0x5e61), 0x24 },
	{ CCI_REG8(0x5e62), 0x1b },
	{ CCI_REG8(0x5e63), 0x23 },
	{ CCI_REG8(0x5e64), 0x1b },
	{ CCI_REG8(0x5e65), 0x28 },
	{ CCI_REG8(0x5e66), 0x22 },
	{ CCI_REG8(0x5e67), 0x28 },
	{ CCI_REG8(0x5e68), 0x23 },
	{ CCI_REG8(0x5e70), 0x25 },
	{ CCI_REG8(0x5e71), 0x24 },
	{ CCI_REG8(0x5e72), 0x20 },
	{ CCI_REG8(0x5e73), 0x24 },
	{ CCI_REG8(0x5e74), 0x20 },
	{ CCI_REG8(0x5e75), 0x28 },
	{ CCI_REG8(0x5e76), 0x27 },
	{ CCI_REG8(0x5e77), 0x29 },
	{ CCI_REG8(0x5e78), 0x24 },
	{ CCI_REG8(0x5e80), 0x25 },
	{ CCI_REG8(0x5e81), 0x25 },
	{ CCI_REG8(0x5e82), 0x24 },
	{ CCI_REG8(0x5e83), 0x25 },
	{ CCI_REG8(0x5e84), 0x23 },
	{ CCI_REG8(0x5e85), 0x2a },
	{ CCI_REG8(0x5e86), 0x28 },
	{ CCI_REG8(0x5e87), 0x2a },
	{ CCI_REG8(0x5e88), 0x28 },
	{ CCI_REG8(0x5e90), 0x24 },
	{ CCI_REG8(0x5e91), 0x24 },
	{ CCI_REG8(0x5e92), 0x28 },
	{ CCI_REG8(0x5e93), 0x29 },
	{ CCI_REG8(0x5e97), 0x25 },
	{ CCI_REG8(0x5e98), 0x25 },
	{ CCI_REG8(0x5e99), 0x2a },
	{ CCI_REG8(0x5e9a), 0x2a },
	{ CCI_REG8(0x5e9e), 0x3a },
	{ CCI_REG8(0x5e9f), 0x3f },
	{ CCI_REG8(0x5ea0), 0x17 },
	{ CCI_REG8(0x5ea1), 0x3f },
	{ CCI_REG8(0x5ea2), 0x17 },
	{ CCI_REG8(0x5ea3), 0x32 },
	{ CCI_REG8(0x5ea4), 0x10 },
	{ CCI_REG8(0x5ea5), 0x33 },
	{ CCI_REG8(0x5ea6), 0x10 },
	{ CCI_REG8(0x5eae), 0x3d },
	{ CCI_REG8(0x5eaf), 0x48 },
	{ CCI_REG8(0x5eb0), 0x3b },
	{ CCI_REG8(0x5eb1), 0x45 },
	{ CCI_REG8(0x5eb2), 0x37 },
	{ CCI_REG8(0x5eb3), 0x3a },
	{ CCI_REG8(0x5eb4), 0x31 },
	{ CCI_REG8(0x5eb5), 0x3a },
	{ CCI_REG8(0x5eb6), 0x31 },
	{ CCI_REG8(0x5ebe), 0x40 },
	{ CCI_REG8(0x5ebf), 0x48 },
	{ CCI_REG8(0x5ec0), 0x3f },
	{ CCI_REG8(0x5ec1), 0x45 },
	{ CCI_REG8(0x5ec2), 0x3f },
	{ CCI_REG8(0x5ec3), 0x3a },
	{ CCI_REG8(0x5ec4), 0x32 },
	{ CCI_REG8(0x5ec5), 0x3a },
	{ CCI_REG8(0x5ec6), 0x33 },
	{ CCI_REG8(0x5ece), 0x4b },
	{ CCI_REG8(0x5ecf), 0x4a },
	{ CCI_REG8(0x5ed0), 0x48 },
	{ CCI_REG8(0x5ed1), 0x4c },
	{ CCI_REG8(0x5ed2), 0x45 },
	{ CCI_REG8(0x5ed3), 0x3f },
	{ CCI_REG8(0x5ed4), 0x3a },
	{ CCI_REG8(0x5ed5), 0x3f },
	{ CCI_REG8(0x5ed6), 0x3a },
	{ CCI_REG8(0x5ede), 0x48 },
	{ CCI_REG8(0x5edf), 0x45 },
	{ CCI_REG8(0x5ee0), 0x3a },
	{ CCI_REG8(0x5ee1), 0x3a },
	{ CCI_REG8(0x5ee5), 0x4a },
	{ CCI_REG8(0x5ee6), 0x4c },
	{ CCI_REG8(0x5ee7), 0x3f },
	{ CCI_REG8(0x5ee8), 0x3f },
	{ CCI_REG8(0x5eec), 0x06 },
	{ CCI_REG8(0x5eed), 0x06 },
	{ CCI_REG8(0x5eee), 0x02 },
	{ CCI_REG8(0x5eef), 0x06 },
	{ CCI_REG8(0x5ef0), 0x01 },
	{ CCI_REG8(0x5ef1), 0x09 },
	{ CCI_REG8(0x5ef2), 0x05 },
	{ CCI_REG8(0x5ef3), 0x06 },
	{ CCI_REG8(0x5ef4), 0x04 },
	{ CCI_REG8(0x5efc), 0x07 },
	{ CCI_REG8(0x5efd), 0x09 },
	{ CCI_REG8(0x5efe), 0x05 },
	{ CCI_REG8(0x5eff), 0x08 },
	{ CCI_REG8(0x5f00), 0x04 },
	{ CCI_REG8(0x5f01), 0x09 },
	{ CCI_REG8(0x5f02), 0x05 },
	{ CCI_REG8(0x5f03), 0x09 },
	{ CCI_REG8(0x5f04), 0x04 },
	{ CCI_REG8(0x5f0c), 0x08 },
	{ CCI_REG8(0x5f0d), 0x09 },
	{ CCI_REG8(0x5f0e), 0x06 },
	{ CCI_REG8(0x5f0f), 0x09 },
	{ CCI_REG8(0x5f10), 0x06 },
	{ CCI_REG8(0x5f11), 0x09 },
	{ CCI_REG8(0x5f12), 0x09 },
	{ CCI_REG8(0x5f13), 0x09 },
	{ CCI_REG8(0x5f14), 0x06 },
	{ CCI_REG8(0x5f1c), 0x09 },
	{ CCI_REG8(0x5f1d), 0x09 },
	{ CCI_REG8(0x5f1e), 0x09 },
	{ CCI_REG8(0x5f1f), 0x09 },
	{ CCI_REG8(0x5f20), 0x08 },
	{ CCI_REG8(0x5f21), 0x09 },
	{ CCI_REG8(0x5f22), 0x09 },
	{ CCI_REG8(0x5f23), 0x09 },
	{ CCI_REG8(0x5f24), 0x09 },
	{ CCI_REG8(0x5f2c), 0x09 },
	{ CCI_REG8(0x5f2d), 0x09 },
	{ CCI_REG8(0x5f2e), 0x09 },
	{ CCI_REG8(0x5f2f), 0x09 },
	{ CCI_REG8(0x5f33), 0x09 },
	{ CCI_REG8(0x5f34), 0x09 },
	{ CCI_REG8(0x5f35), 0x09 },
	{ CCI_REG8(0x5f36), 0x09 },
	{ CCI_REG8(0x5f3a), 0x01 },
	{ CCI_REG8(0x5f3d), 0x07 },
	{ CCI_REG8(0x5f3f), 0x01 },
	{ CCI_REG8(0x5f4b), 0x01 },
	{ CCI_REG8(0x5f4d), 0x04 },
	{ CCI_REG8(0x5f4f), 0x02 },
	{ CCI_REG8(0x5f51), 0x02 },
	{ CCI_REG8(0x5f5a), 0x02 },
	{ CCI_REG8(0x5f5b), 0x01 },
	{ CCI_REG8(0x5f5d), 0x03 },
	{ CCI_REG8(0x5f5e), 0x07 },
	{ CCI_REG8(0x5f5f), 0x01 },
	{ CCI_REG8(0x5f60), 0x01 },
	{ CCI_REG8(0x5f61), 0x01 },
	{ CCI_REG8(0x5f6a), 0x01 },
	{ CCI_REG8(0x5f6c), 0x01 },
	{ CCI_REG8(0x5f6d), 0x01 },
	{ CCI_REG8(0x5f6e), 0x04 },
	{ CCI_REG8(0x5f70), 0x02 },
	{ CCI_REG8(0x5f72), 0x02 },
	{ CCI_REG8(0x5f7a), 0x01 },
	{ CCI_REG8(0x5f7b), 0x03 },
	{ CCI_REG8(0x5f7c), 0x01 },
	{ CCI_REG8(0x5f7d), 0x01 },
	{ CCI_REG8(0x5f82), 0x01 },
	{ CCI_REG8(0x60c6), 0x4a },
	{ CCI_REG8(0x60c8), 0x4a },
	{ CCI_REG8(0x60d6), 0x4a },
	{ CCI_REG8(0x60d8), 0x4a },
	{ CCI_REG8(0x62e4), 0x33 },
	{ CCI_REG8(0x62e9), 0x33 },
	{ CCI_REG8(0x62ee), 0x1c },
	{ CCI_REG8(0x62ef), 0x33 },
	{ CCI_REG8(0x62f3), 0x33 },
	{ CCI_REG8(0x62f6), 0x1c },
	{ CCI_REG8(0x33f2), 0x01 },
	{ CCI_REG8(0x1f04), 0xa3 },
	{ CCI_REG8(0x1f05), 0x01 },
	{ CCI_REG8(0x406e), 0x00 },
	{ CCI_REG8(0x406f), 0x08 },
	{ CCI_REG8(0x4d08), 0x00 },
	{ CCI_REG8(0x4d09), 0x2c },
	{ CCI_REG8(0x4d0e), 0x00 },
	{ CCI_REG8(0x4d0f), 0x64 },
	{ CCI_REG8(0x4d18), 0x00 },
	{ CCI_REG8(0x4d19), 0xb1 },
	{ CCI_REG8(0x4d1e), 0x00 },
	{ CCI_REG8(0x4d1f), 0xcb },
	{ CCI_REG8(0x4d3a), 0x00 },
	{ CCI_REG8(0x4d3b), 0x91 },
	{ CCI_REG8(0x4d40), 0x00 },
	{ CCI_REG8(0x4d41), 0x64 },
	{ CCI_REG8(0x4d4c), 0x00 },
	{ CCI_REG8(0x4d4d), 0xe8 },
	{ CCI_REG8(0x4d52), 0x00 },
	{ CCI_REG8(0x4d53), 0xcb },
	{ CCI_REG8(0x4d78), 0x00 },
	{ CCI_REG8(0x4d79), 0x2c },
	{ CCI_REG8(0x4d7e), 0x00 },
	{ CCI_REG8(0x4d7f), 0x64 },
	{ CCI_REG8(0x4d88), 0x00 },
	{ CCI_REG8(0x4d89), 0xab },
	{ CCI_REG8(0x4d8e), 0x00 },
	{ CCI_REG8(0x4d8f), 0xcb },
	{ CCI_REG8(0x4da6), 0x00 },
	{ CCI_REG8(0x4da7), 0xe7 },
	{ CCI_REG8(0x4dac), 0x00 },
	{ CCI_REG8(0x4dad), 0xcb },
	{ CCI_REG8(0x5b98), 0x00 },
	{ CCI_REG8(0x5c52), 0x05 },
	{ CCI_REG8(0x5c57), 0x09 },
	{ CCI_REG8(0x5d94), 0x0a },
	{ CCI_REG8(0x5d9e), 0x0a },
	{ CCI_REG8(0x5e50), 0x22 },
	{ CCI_REG8(0x5e51), 0x22 },
	{ CCI_REG8(0x5e52), 0x07 },
	{ CCI_REG8(0x5e53), 0x20 },
	{ CCI_REG8(0x5e54), 0x06 },
	{ CCI_REG8(0x5e55), 0x23 },
	{ CCI_REG8(0x5e56), 0x0a },
	{ CCI_REG8(0x5e57), 0x23 },
	{ CCI_REG8(0x5e58), 0x0a },
	{ CCI_REG8(0x5e60), 0x25 },
	{ CCI_REG8(0x5e61), 0x29 },
	{ CCI_REG8(0x5e62), 0x1c },
	{ CCI_REG8(0x5e63), 0x26 },
	{ CCI_REG8(0x5e64), 0x1c },
	{ CCI_REG8(0x5e65), 0x2d },
	{ CCI_REG8(0x5e66), 0x1e },
	{ CCI_REG8(0x5e67), 0x2a },
	{ CCI_REG8(0x5e68), 0x1e },
	{ CCI_REG8(0x5e70), 0x26 },
	{ CCI_REG8(0x5e71), 0x26 },
	{ CCI_REG8(0x5e72), 0x22 },
	{ CCI_REG8(0x5e73), 0x23 },
	{ CCI_REG8(0x5e74), 0x20 },
	{ CCI_REG8(0x5e75), 0x28 },
	{ CCI_REG8(0x5e76), 0x23 },
	{ CCI_REG8(0x5e77), 0x28 },
	{ CCI_REG8(0x5e78), 0x23 },
	{ CCI_REG8(0x5e80), 0x28 },
	{ CCI_REG8(0x5e81), 0x28 },
	{ CCI_REG8(0x5e82), 0x29 },
	{ CCI_REG8(0x5e83), 0x27 },
	{ CCI_REG8(0x5e84), 0x26 },
	{ CCI_REG8(0x5e85), 0x2a },
	{ CCI_REG8(0x5e86), 0x2d },
	{ CCI_REG8(0x5e87), 0x2a },
	{ CCI_REG8(0x5e88), 0x2a },
	{ CCI_REG8(0x5e90), 0x26 },
	{ CCI_REG8(0x5e91), 0x23 },
	{ CCI_REG8(0x5e92), 0x28 },
	{ CCI_REG8(0x5e93), 0x28 },
	{ CCI_REG8(0x5e97), 0x2f },
	{ CCI_REG8(0x5e98), 0x2e },
	{ CCI_REG8(0x5e99), 0x32 },
	{ CCI_REG8(0x5e9a), 0x32 },
	{ CCI_REG8(0x5e9e), 0x50 },
	{ CCI_REG8(0x5e9f), 0x50 },
	{ CCI_REG8(0x5ea0), 0x1e },
	{ CCI_REG8(0x5ea1), 0x50 },
	{ CCI_REG8(0x5ea2), 0x1d },
	{ CCI_REG8(0x5ea3), 0x3e },
	{ CCI_REG8(0x5ea4), 0x14 },
	{ CCI_REG8(0x5ea5), 0x3e },
	{ CCI_REG8(0x5ea6), 0x14 },
	{ CCI_REG8(0x5eae), 0x58 },
	{ CCI_REG8(0x5eaf), 0x5e },
	{ CCI_REG8(0x5eb0), 0x4b },
	{ CCI_REG8(0x5eb1), 0x5a },
	{ CCI_REG8(0x5eb2), 0x4b },
	{ CCI_REG8(0x5eb3), 0x4c },
	{ CCI_REG8(0x5eb4), 0x3a },
	{ CCI_REG8(0x5eb5), 0x4c },
	{ CCI_REG8(0x5eb6), 0x38 },
	{ CCI_REG8(0x5ebe), 0x56 },
	{ CCI_REG8(0x5ebf), 0x57 },
	{ CCI_REG8(0x5ec0), 0x50 },
	{ CCI_REG8(0x5ec1), 0x55 },
	{ CCI_REG8(0x5ec2), 0x50 },
	{ CCI_REG8(0x5ec3), 0x46 },
	{ CCI_REG8(0x5ec4), 0x3e },
	{ CCI_REG8(0x5ec5), 0x46 },
	{ CCI_REG8(0x5ec6), 0x3e },
	{ CCI_REG8(0x5ece), 0x5a },
	{ CCI_REG8(0x5ecf), 0x5f },
	{ CCI_REG8(0x5ed0), 0x5e },
	{ CCI_REG8(0x5ed1), 0x5a },
	{ CCI_REG8(0x5ed2), 0x5a },
	{ CCI_REG8(0x5ed3), 0x50 },
	{ CCI_REG8(0x5ed4), 0x4c },
	{ CCI_REG8(0x5ed5), 0x50 },
	{ CCI_REG8(0x5ed6), 0x4c },
	{ CCI_REG8(0x5ede), 0x57 },
	{ CCI_REG8(0x5edf), 0x55 },
	{ CCI_REG8(0x5ee0), 0x46 },
	{ CCI_REG8(0x5ee1), 0x46 },
	{ CCI_REG8(0x5ee5), 0x73 },
	{ CCI_REG8(0x5ee6), 0x6e },
	{ CCI_REG8(0x5ee7), 0x5f },
	{ CCI_REG8(0x5ee8), 0x5a },
	{ CCI_REG8(0x5eec), 0x0a },
	{ CCI_REG8(0x5eed), 0x0a },
	{ CCI_REG8(0x5eee), 0x0f },
	{ CCI_REG8(0x5eef), 0x0a },
	{ CCI_REG8(0x5ef0), 0x0e },
	{ CCI_REG8(0x5ef1), 0x08 },
	{ CCI_REG8(0x5ef2), 0x0c },
	{ CCI_REG8(0x5ef3), 0x0c },
	{ CCI_REG8(0x5ef4), 0x0f },
	{ CCI_REG8(0x5efc), 0x0a },
	{ CCI_REG8(0x5efd), 0x0a },
	{ CCI_REG8(0x5efe), 0x14 },
	{ CCI_REG8(0x5eff), 0x0a },
	{ CCI_REG8(0x5f00), 0x14 },
	{ CCI_REG8(0x5f01), 0x0a },
	{ CCI_REG8(0x5f02), 0x14 },
	{ CCI_REG8(0x5f03), 0x0a },
	{ CCI_REG8(0x5f04), 0x19 },
	{ CCI_REG8(0x5f0c), 0x0a },
	{ CCI_REG8(0x5f0d), 0x0a },
	{ CCI_REG8(0x5f0e), 0x0a },
	{ CCI_REG8(0x5f0f), 0x05 },
	{ CCI_REG8(0x5f10), 0x0a },
	{ CCI_REG8(0x5f11), 0x06 },
	{ CCI_REG8(0x5f12), 0x08 },
	{ CCI_REG8(0x5f13), 0x0a },
	{ CCI_REG8(0x5f14), 0x0c },
	{ CCI_REG8(0x5f1c), 0x0a },
	{ CCI_REG8(0x5f1d), 0x0a },
	{ CCI_REG8(0x5f1e), 0x0a },
	{ CCI_REG8(0x5f1f), 0x0a },
	{ CCI_REG8(0x5f20), 0x0a },
	{ CCI_REG8(0x5f21), 0x0a },
	{ CCI_REG8(0x5f22), 0x0a },
	{ CCI_REG8(0x5f23), 0x0a },
	{ CCI_REG8(0x5f24), 0x0a },
	{ CCI_REG8(0x5f2c), 0x0a },
	{ CCI_REG8(0x5f2d), 0x05 },
	{ CCI_REG8(0x5f2e), 0x06 },
	{ CCI_REG8(0x5f2f), 0x0a },
	{ CCI_REG8(0x5f33), 0x0a },
	{ CCI_REG8(0x5f34), 0x0a },
	{ CCI_REG8(0x5f35), 0x0a },
	{ CCI_REG8(0x5f36), 0x0a },
	{ CCI_REG8(0x5f3a), 0x00 },
	{ CCI_REG8(0x5f3d), 0x02 },
	{ CCI_REG8(0x5f3f), 0x0a },
	{ CCI_REG8(0x5f4a), 0x0a },
	{ CCI_REG8(0x5f4b), 0x0a },
	{ CCI_REG8(0x5f4d), 0x0f },
	{ CCI_REG8(0x5f4f), 0x00 },
	{ CCI_REG8(0x5f51), 0x00 },
	{ CCI_REG8(0x5f5a), 0x00 },
	{ CCI_REG8(0x5f5b), 0x00 },
	{ CCI_REG8(0x5f5d), 0x0a },
	{ CCI_REG8(0x5f5e), 0x02 },
	{ CCI_REG8(0x5f5f), 0x0a },
	{ CCI_REG8(0x5f60), 0x0a },
	{ CCI_REG8(0x5f61), 0x00 },
	{ CCI_REG8(0x5f6a), 0x00 },
	{ CCI_REG8(0x5f6c), 0x0a },
	{ CCI_REG8(0x5f6d), 0x06 },
	{ CCI_REG8(0x5f6e), 0x0f },
	{ CCI_REG8(0x5f70), 0x00 },
	{ CCI_REG8(0x5f72), 0x00 },
	{ CCI_REG8(0x5f7a), 0x00 },
	{ CCI_REG8(0x5f7b), 0x0a },
	{ CCI_REG8(0x5f7c), 0x0a },
	{ CCI_REG8(0x5f7d), 0x00 },
	{ CCI_REG8(0x5f82), 0x06 },
	{ CCI_REG8(0x60c6), 0x36 },
	{ CCI_REG8(0x60c8), 0x36 },
	{ CCI_REG8(0x60d6), 0x36 },
	{ CCI_REG8(0x60d8), 0x36 },
	{ CCI_REG8(0x62df), 0x56 },
	{ CCI_REG8(0x62e0), 0x52 },
	{ CCI_REG8(0x62e4), 0x38 },
	{ CCI_REG8(0x62e5), 0x51 },
	{ CCI_REG8(0x62e9), 0x35 },
	{ CCI_REG8(0x62ea), 0x54 },
	{ CCI_REG8(0x62ee), 0x1d },
	{ CCI_REG8(0x62ef), 0x38 },
	{ CCI_REG8(0x62f3), 0x33 },
	{ CCI_REG8(0x62f6), 0x26 },
	{ CCI_REG8(0x6412), 0x1e },
	{ CCI_REG8(0x6413), 0x1e },
	{ CCI_REG8(0x6414), 0x1e },
	{ CCI_REG8(0x6415), 0x1e },
	{ CCI_REG8(0x6416), 0x1e },
	{ CCI_REG8(0x6417), 0x1e },
	{ CCI_REG8(0x6418), 0x1e },
	{ CCI_REG8(0x641a), 0x1e },
	{ CCI_REG8(0x641b), 0x1e },
	{ CCI_REG8(0x641c), 0x1e },
	{ CCI_REG8(0x641d), 0x1e },
	{ CCI_REG8(0x641e), 0x1e },
	{ CCI_REG8(0x641f), 0x1e },
	{ CCI_REG8(0x6420), 0x1e },
	{ CCI_REG8(0x6421), 0x1e },
	{ CCI_REG8(0x6422), 0x1e },
	{ CCI_REG8(0x6424), 0x1e },
	{ CCI_REG8(0x6425), 0x1e },
	{ CCI_REG8(0x6426), 0x1e },
	{ CCI_REG8(0x6427), 0x1e },
	{ CCI_REG8(0x6428), 0x1e },
	{ CCI_REG8(0x6429), 0x1e },
	{ CCI_REG8(0x642a), 0x1e },
	{ CCI_REG8(0x642b), 0x1e },
	{ CCI_REG8(0x642c), 0x1e },
	{ CCI_REG8(0x642e), 0x1e },
	{ CCI_REG8(0x642f), 0x1e },
	{ CCI_REG8(0x6430), 0x1e },
	{ CCI_REG8(0x6431), 0x1e },
	{ CCI_REG8(0x6432), 0x1e },
	{ CCI_REG8(0x6433), 0x1e },
	{ CCI_REG8(0x6434), 0x1e },
	{ CCI_REG8(0x6435), 0x1e },
	{ CCI_REG8(0x6436), 0x1e },
	{ CCI_REG8(0x6438), 0x1e },
	{ CCI_REG8(0x6439), 0x1e },
	{ CCI_REG8(0x643a), 0x1e },
	{ CCI_REG8(0x643b), 0x1e },
	{ CCI_REG8(0x643d), 0x1e },
	{ CCI_REG8(0x643e), 0x1e },
	{ CCI_REG8(0x643f), 0x1e },
	{ CCI_REG8(0x6441), 0x1e },
	{ CCI_REG8(0x33f2), 0x02 },
	{ CCI_REG8(0x1f08), 0x00 },
	{ CCI_REG8(0xa307), 0x30 },
	{ CCI_REG8(0xa309), 0x30 },
	{ CCI_REG8(0xa30b), 0x30 },
	{ CCI_REG8(0xa406), 0x03 },
	{ CCI_REG8(0xa407), 0x48 },
	{ CCI_REG8(0xa408), 0x03 },
	{ CCI_REG8(0xa409), 0x48 },
	{ CCI_REG8(0xa40a), 0x03 },
	{ CCI_REG8(0xa40b), 0x48 },
	{ CCI_REG8(0x86a9), 0x4e },
	/* No QSC calibration data is loaded: keep QSC off */
	{ CCI_REG8(0x32d2), 0x00 },
};

/*
 * 2048x1536: binning of the whole array (Quad Bayer 2x2, then H2V2), RAW10
 * on three C-PHY trios at 862 Msym/s. The frame length, exposure and gains
 * come from the controls.
 */
static const struct cci_reg_sequence imx766_mode_2048x1536_regs[] = {
	/* CSI_DATA_FORMAT: RAW10 */
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	/* CSI_LANE_MODE: 3 trios */
	{ CCI_REG8(0x0114), 0x02 },
	/* LINE_LENGTH_PCK */
	{ CCI_REG8(0x0342), 0x22 },
	{ CCI_REG8(0x0343), 0x70 },
	/* X_ADDR_START */
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x00 },
	/* Y_ADDR_START */
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x00 },
	/* X_ADDR_END */
	{ CCI_REG8(0x0348), 0x1f },
	{ CCI_REG8(0x0349), 0xff },
	/* Y_ADDR_END */
	{ CCI_REG8(0x034a), 0x17 },
	{ CCI_REG8(0x034b), 0xff },
	/* BINNING_MODE */
	{ CCI_REG8(0x0900), 0x01 },
	/* BINNING_TYPE */
	{ CCI_REG8(0x0901), 0x44 },
	/* BINNING_WEIGHTING */
	{ CCI_REG8(0x0902), 0x08 },
	{ CCI_REG8(0x3005), 0x02 },
	{ CCI_REG8(0x3120), 0x04 },
	{ CCI_REG8(0x3121), 0x00 },
	{ CCI_REG8(0x3200), 0x43 },
	{ CCI_REG8(0x3201), 0x43 },
	{ CCI_REG8(0x32d6), 0x00 },
	/* DIG_CROP_X_OFFSET */
	{ CCI_REG8(0x0408), 0x00 },
	{ CCI_REG8(0x0409), 0x00 },
	/* DIG_CROP_Y_OFFSET */
	{ CCI_REG8(0x040a), 0x00 },
	{ CCI_REG8(0x040b), 0x00 },
	/* DIG_CROP_IMAGE_WIDTH */
	{ CCI_REG8(0x040c), 0x08 },
	{ CCI_REG8(0x040d), 0x00 },
	/* DIG_CROP_IMAGE_HEIGHT */
	{ CCI_REG8(0x040e), 0x06 },
	{ CCI_REG8(0x040f), 0x00 },
	/* X_OUTPUT_SIZE */
	{ CCI_REG8(0x034c), 0x08 },
	{ CCI_REG8(0x034d), 0x00 },
	/* Y_OUTPUT_SIZE */
	{ CCI_REG8(0x034e), 0x06 },
	{ CCI_REG8(0x034f), 0x00 },
	/* IVTPXCK_DIV */
	{ CCI_REG8(0x0301), 0x05 },
	/* IVTSYCK_DIV */
	{ CCI_REG8(0x0303), 0x04 },
	/* IVT_PREPLLCK_DIV */
	{ CCI_REG8(0x0305), 0x04 },
	/* IVT_PLL_MPY */
	{ CCI_REG8(0x0306), 0x01 },
	{ CCI_REG8(0x0307), 0x36 },
	/* IOPSYCK_DIV */
	{ CCI_REG8(0x030b), 0x04 },
	/* IOP_PREPLLCK_DIV */
	{ CCI_REG8(0x030d), 0x03 },
	/* IOP_PLL_MPY */
	{ CCI_REG8(0x030e), 0x01 },
	{ CCI_REG8(0x030f), 0xaf },
	{ CCI_REG8(0x30cb), 0x00 },
	{ CCI_REG8(0x30cc), 0x10 },
	{ CCI_REG8(0x30cd), 0x00 },
	{ CCI_REG8(0x30ce), 0x03 },
	{ CCI_REG8(0x30cf), 0x00 },
	{ CCI_REG8(0x319c), 0x01 },
	{ CCI_REG8(0x3800), 0x01 },
	{ CCI_REG8(0x3801), 0x01 },
	{ CCI_REG8(0x3802), 0x02 },
	{ CCI_REG8(0x3847), 0x03 },
	{ CCI_REG8(0x38b0), 0x00 },
	{ CCI_REG8(0x38b1), 0x64 },
	{ CCI_REG8(0x38b2), 0x00 },
	{ CCI_REG8(0x38b3), 0x64 },
	{ CCI_REG8(0x38c4), 0x00 },
	{ CCI_REG8(0x38c5), 0x64 },
	{ CCI_REG8(0x4c3a), 0x02 },
	{ CCI_REG8(0x4c3b), 0xd2 },
	{ CCI_REG8(0x4c68), 0x04 },
	{ CCI_REG8(0x4c69), 0x7e },
	{ CCI_REG8(0x4cf8), 0x07 },
	{ CCI_REG8(0x4cf9), 0x9e },
	{ CCI_REG8(0x4db8), 0x08 },
	{ CCI_REG8(0x4db9), 0x98 },
	/* short exposure (DOL, unused) */
	{ CCI_REG8(0x0224), 0x01 },
	{ CCI_REG8(0x0225), 0xf4 },
	{ CCI_REG8(0x313a), 0x01 },
	{ CCI_REG8(0x313b), 0xf4 },
	{ CCI_REG8(0x3803), 0x00 },
	{ CCI_REG8(0x3804), 0x17 },
	{ CCI_REG8(0x3805), 0xc0 },
	/* short analogue gain (DOL, unused) */
	{ CCI_REG8(0x0216), 0x00 },
	{ CCI_REG8(0x0217), 0x00 },
	/* short digital gain (DOL, unused) */
	{ CCI_REG8(0x0218), 0x01 },
	{ CCI_REG8(0x0219), 0x00 },
	{ CCI_REG8(0x313c), 0x00 },
	{ CCI_REG8(0x313d), 0x00 },
	{ CCI_REG8(0x313e), 0x01 },
	{ CCI_REG8(0x313f), 0x00 },
	/* PDAF output: off */
	{ CCI_REG8(0x30b4), 0x00 },
	{ CCI_REG8(0x3066), 0x03 },
	{ CCI_REG8(0x3067), 0x2b },
	{ CCI_REG8(0x3068), 0x06 },
	{ CCI_REG8(0x3069), 0x2b },
	/* DOL off */
	{ CCI_REG8(0x33d0), 0x00 },
	{ CCI_REG8(0x33d1), 0x00 },
	{ CCI_REG8(0x33d4), 0x01 },
	{ CCI_REG8(0x33dc), 0x0a },
	{ CCI_REG8(0x33dd), 0x0a },
	{ CCI_REG8(0x33de), 0x0a },
	{ CCI_REG8(0x33df), 0x0a },
	{ CCI_REG8(0x3070), 0x01 },
	{ CCI_REG8(0x3077), 0x04 },
	{ CCI_REG8(0x3078), 0x2b },
	{ CCI_REG8(0x3079), 0x07 },
	{ CCI_REG8(0x307a), 0x2b },
	{ CCI_REG8(0x307b), 0x01 },
	{ CCI_REG8(0x3080), 0x02 },
	{ CCI_REG8(0x3087), 0x05 },
	{ CCI_REG8(0x3088), 0x2b },
	{ CCI_REG8(0x3089), 0x08 },
	{ CCI_REG8(0x308a), 0x2b },
	{ CCI_REG8(0x308b), 0x02 },
	{ CCI_REG8(0x3901), 0x2b },
	{ CCI_REG8(0x3902), 0x00 },
	{ CCI_REG8(0x3903), 0x12 },
	{ CCI_REG8(0x3905), 0x2b },
	{ CCI_REG8(0x3906), 0x01 },
	{ CCI_REG8(0x3907), 0x12 },
	{ CCI_REG8(0x3909), 0x2b },
	{ CCI_REG8(0x390a), 0x02 },
	{ CCI_REG8(0x390b), 0x12 },
	{ CCI_REG8(0x3911), 0x00 },
	/* PHY_CTRL: manual timing */
	{ CCI_REG8(0x0808), 0x02 },
	{ CCI_REG8(0x084e), 0x00 },
	{ CCI_REG8(0x084f), 0x07 },
	{ CCI_REG8(0x0850), 0x00 },
	{ CCI_REG8(0x0851), 0x07 },
	{ CCI_REG8(0x0852), 0x00 },
	{ CCI_REG8(0x0853), 0x0d },
	{ CCI_REG8(0x0854), 0x00 },
	{ CCI_REG8(0x0855), 0x29 },
	{ CCI_REG8(0x0858), 0x00 },
	{ CCI_REG8(0x0859), 0x1f },
};

/*
 * 4096x3072: Quad Bayer binning of the whole array, RAW10 on three C-PHY
 * trios at 1148 Msym/s.
 */
static const struct cci_reg_sequence imx766_mode_4096x3072_regs[] = {
	/* CSI_DATA_FORMAT: RAW10 */
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	/* CSI_LANE_MODE: 3 trios */
	{ CCI_REG8(0x0114), 0x02 },
	/* LINE_LENGTH_PCK */
	{ CCI_REG8(0x0342), 0x3d },
	{ CCI_REG8(0x0343), 0x00 },
	/* X_ADDR_START */
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x00 },
	/* Y_ADDR_START */
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x00 },
	/* X_ADDR_END */
	{ CCI_REG8(0x0348), 0x1f },
	{ CCI_REG8(0x0349), 0xff },
	/* Y_ADDR_END */
	{ CCI_REG8(0x034a), 0x17 },
	{ CCI_REG8(0x034b), 0xff },
	/* BINNING_MODE */
	{ CCI_REG8(0x0900), 0x01 },
	/* BINNING_TYPE */
	{ CCI_REG8(0x0901), 0x22 },
	/* BINNING_WEIGHTING */
	{ CCI_REG8(0x0902), 0x08 },
	{ CCI_REG8(0x3005), 0x02 },
	{ CCI_REG8(0x3120), 0x04 },
	{ CCI_REG8(0x3121), 0x01 },
	{ CCI_REG8(0x3200), 0x41 },
	{ CCI_REG8(0x3201), 0x41 },
	{ CCI_REG8(0x32d6), 0x00 },
	/* DIG_CROP_X_OFFSET */
	{ CCI_REG8(0x0408), 0x00 },
	{ CCI_REG8(0x0409), 0x00 },
	/* DIG_CROP_Y_OFFSET */
	{ CCI_REG8(0x040a), 0x00 },
	{ CCI_REG8(0x040b), 0x00 },
	/* DIG_CROP_IMAGE_WIDTH */
	{ CCI_REG8(0x040c), 0x10 },
	{ CCI_REG8(0x040d), 0x00 },
	/* DIG_CROP_IMAGE_HEIGHT */
	{ CCI_REG8(0x040e), 0x0c },
	{ CCI_REG8(0x040f), 0x00 },
	/* X_OUTPUT_SIZE */
	{ CCI_REG8(0x034c), 0x10 },
	{ CCI_REG8(0x034d), 0x00 },
	/* Y_OUTPUT_SIZE */
	{ CCI_REG8(0x034e), 0x0c },
	{ CCI_REG8(0x034f), 0x00 },
	/* IVTPXCK_DIV */
	{ CCI_REG8(0x0301), 0x05 },
	/* IVTSYCK_DIV */
	{ CCI_REG8(0x0303), 0x04 },
	/* IVT_PREPLLCK_DIV */
	{ CCI_REG8(0x0305), 0x04 },
	/* IVT_PLL_MPY */
	{ CCI_REG8(0x0306), 0x01 },
	{ CCI_REG8(0x0307), 0x36 },
	/* IOPSYCK_DIV */
	{ CCI_REG8(0x030b), 0x02 },
	/* IOP_PREPLLCK_DIV */
	{ CCI_REG8(0x030d), 0x03 },
	/* IOP_PLL_MPY */
	{ CCI_REG8(0x030e), 0x01 },
	{ CCI_REG8(0x030f), 0x1f },
	{ CCI_REG8(0x30cb), 0x00 },
	{ CCI_REG8(0x30cc), 0x10 },
	{ CCI_REG8(0x30cd), 0x00 },
	{ CCI_REG8(0x30ce), 0x03 },
	{ CCI_REG8(0x30cf), 0x00 },
	{ CCI_REG8(0x319c), 0x01 },
	{ CCI_REG8(0x3800), 0x01 },
	{ CCI_REG8(0x3801), 0x01 },
	{ CCI_REG8(0x3802), 0x02 },
	{ CCI_REG8(0x3847), 0x03 },
	{ CCI_REG8(0x38b0), 0x00 },
	{ CCI_REG8(0x38b1), 0x00 },
	{ CCI_REG8(0x38b2), 0x00 },
	{ CCI_REG8(0x38b3), 0x00 },
	{ CCI_REG8(0x38c4), 0x01 },
	{ CCI_REG8(0x38c5), 0x2c },
	{ CCI_REG8(0x4c3a), 0x02 },
	{ CCI_REG8(0x4c3b), 0xd2 },
	{ CCI_REG8(0x4c68), 0x04 },
	{ CCI_REG8(0x4c69), 0x7e },
	{ CCI_REG8(0x4cf8), 0x07 },
	{ CCI_REG8(0x4cf9), 0x9e },
	{ CCI_REG8(0x4db8), 0x08 },
	{ CCI_REG8(0x4db9), 0x98 },
	/* short exposure (DOL, unused) */
	{ CCI_REG8(0x0224), 0x01 },
	{ CCI_REG8(0x0225), 0xf4 },
	{ CCI_REG8(0x313a), 0x01 },
	{ CCI_REG8(0x313b), 0xf4 },
	{ CCI_REG8(0x3803), 0x00 },
	{ CCI_REG8(0x3804), 0x17 },
	{ CCI_REG8(0x3805), 0xc0 },
	/* short analogue gain (DOL, unused) */
	{ CCI_REG8(0x0216), 0x00 },
	{ CCI_REG8(0x0217), 0x00 },
	/* short digital gain (DOL, unused) */
	{ CCI_REG8(0x0218), 0x01 },
	{ CCI_REG8(0x0219), 0x00 },
	{ CCI_REG8(0x313c), 0x00 },
	{ CCI_REG8(0x313d), 0x00 },
	{ CCI_REG8(0x313e), 0x01 },
	{ CCI_REG8(0x313f), 0x00 },
	{ CCI_REG8(0x0860), 0x01 },
	{ CCI_REG8(0x0861), 0x2d },
	{ CCI_REG8(0x0862), 0x01 },
	{ CCI_REG8(0x0863), 0x2d },
	/* PDAF output: off */
	{ CCI_REG8(0x30b4), 0x00 },
	{ CCI_REG8(0x3066), 0x03 },
	{ CCI_REG8(0x3067), 0x2b },
	{ CCI_REG8(0x3068), 0x06 },
	{ CCI_REG8(0x3069), 0x2b },
	/* DOL off */
	{ CCI_REG8(0x33d0), 0x00 },
	{ CCI_REG8(0x33d1), 0x00 },
	{ CCI_REG8(0x33d4), 0x01 },
	{ CCI_REG8(0x33dc), 0x0a },
	{ CCI_REG8(0x33dd), 0x0a },
	{ CCI_REG8(0x33de), 0x0a },
	{ CCI_REG8(0x33df), 0x0a },
	{ CCI_REG8(0x3070), 0x01 },
	{ CCI_REG8(0x3077), 0x04 },
	{ CCI_REG8(0x3078), 0x2b },
	{ CCI_REG8(0x3079), 0x07 },
	{ CCI_REG8(0x307a), 0x2b },
	{ CCI_REG8(0x307b), 0x01 },
	{ CCI_REG8(0x3080), 0x02 },
	{ CCI_REG8(0x3087), 0x05 },
	{ CCI_REG8(0x3088), 0x2b },
	{ CCI_REG8(0x3089), 0x08 },
	{ CCI_REG8(0x308a), 0x2b },
	{ CCI_REG8(0x308b), 0x02 },
	{ CCI_REG8(0x3901), 0x2b },
	{ CCI_REG8(0x3902), 0x00 },
	{ CCI_REG8(0x3903), 0x12 },
	{ CCI_REG8(0x3905), 0x2b },
	{ CCI_REG8(0x3906), 0x01 },
	{ CCI_REG8(0x3907), 0x12 },
	{ CCI_REG8(0x3909), 0x2b },
	{ CCI_REG8(0x390a), 0x02 },
	{ CCI_REG8(0x390b), 0x12 },
	{ CCI_REG8(0x3911), 0x00 },
	/* PHY_CTRL: manual timing */
	{ CCI_REG8(0x0808), 0x02 },
	{ CCI_REG8(0x084e), 0x00 },
	{ CCI_REG8(0x084f), 0x0b },
	{ CCI_REG8(0x0850), 0x00 },
	{ CCI_REG8(0x0851), 0x09 },
	{ CCI_REG8(0x0852), 0x00 },
	{ CCI_REG8(0x0853), 0x13 },
	{ CCI_REG8(0x0854), 0x00 },
	{ CCI_REG8(0x0855), 0x29 },
	{ CCI_REG8(0x0858), 0x00 },
	{ CCI_REG8(0x0859), 0x1f },
};

struct imx766_reg_list {
	unsigned int num_regs;
	const struct cci_reg_sequence *regs;
};

struct imx766_mode {
	u32 width;
	u32 height;
	/* Line length, in pixels at the pixel rate */
	u32 hts;
	/* Shortest frame length, in lines */
	u32 vts_min;
	/* Analogue crop, in pixel array coordinates */
	struct v4l2_rect crop;
	unsigned int link_freq_index;
	struct imx766_reg_list reg_list;
};

/*
 * Both modes keep the reference driver's frame timing: 30 fps at most. A
 * longer frame (VBLANK) lowers the frame rate and allows longer exposures.
 */
static const struct imx766_mode imx766_modes[] = {
	{
		.width = 4096,
		.height = 3072,
		.hts = 15616,
		.vts_min = 3176,
		.crop = {
			.left = IMX766_PIXEL_ARRAY_LEFT,
			.top = IMX766_PIXEL_ARRAY_TOP,
			.width = IMX766_PIXEL_ARRAY_WIDTH,
			.height = IMX766_PIXEL_ARRAY_HEIGHT,
		},
		.link_freq_index = IMX766_LINK_FREQ_574MHZ_INDEX,
		.reg_list = {
			.num_regs = ARRAY_SIZE(imx766_mode_4096x3072_regs),
			.regs = imx766_mode_4096x3072_regs,
		},
	},
	{
		.width = 2048,
		.height = 1536,
		.hts = 8816,
		.vts_min = 5624,
		.crop = {
			.left = IMX766_PIXEL_ARRAY_LEFT,
			.top = IMX766_PIXEL_ARRAY_TOP,
			.width = IMX766_PIXEL_ARRAY_WIDTH,
			.height = IMX766_PIXEL_ARRAY_HEIGHT,
		},
		.link_freq_index = IMX766_LINK_FREQ_431MHZ_INDEX,
		.reg_list = {
			.num_regs = ARRAY_SIZE(imx766_mode_2048x1536_regs),
			.regs = imx766_mode_2048x1536_regs,
		},
	},
};

/* The preferred default mode, if its link frequency is available */
#define IMX766_DEFAULT_MODE		(&imx766_modes[1])

/*
 * The Bayer order of the output changes with the flips, as the readout
 * window does not move: index with hflip | vflip << 1.
 */
static const u32 imx766_mbus_codes[] = {
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

/* The menu index is the CCS test pattern mode */
static const char * const imx766_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"Eight Vertical Colour Bars",
	"Colour Bars With Fade to Grey",
	"Pseudorandom Sequence (PN9)",
};

enum imx766_supply {
	IMX766_SUPPLY_DOVDD,
	IMX766_SUPPLY_AVDD,
	IMX766_SUPPLY_AVDD2,
	IMX766_SUPPLY_DVDD,
	IMX766_NUM_SUPPLIES
};

/* In power-up order */
static const char * const imx766_supply_names[IMX766_NUM_SUPPLIES] = {
	[IMX766_SUPPLY_DOVDD] = "dovdd",
	[IMX766_SUPPLY_AVDD] = "avdd",
	[IMX766_SUPPLY_AVDD2] = "avdd2",
	[IMX766_SUPPLY_DVDD] = "dvdd",
};

struct imx766 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[IMX766_NUM_SUPPLIES];

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

	/* Link frequencies listed in the endpoint, indexing imx766_link_freqs */
	unsigned long link_freq_bitmap;

	/* Current mode, protected by the control handler lock */
	const struct imx766_mode *mode;
};

static inline struct imx766 *to_imx766(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx766, sd);
}

static u32 imx766_mbus_code(struct imx766 *imx766)
{
	unsigned int i = 0;

	if (imx766->hflip && imx766->hflip->val)
		i |= 1;
	if (imx766->vflip && imx766->vflip->val)
		i |= 2;

	return imx766_mbus_codes[i];
}

/* Only the modes whose link frequency the endpoint lists can be used */
static bool imx766_mode_usable(struct imx766 *imx766,
			       const struct imx766_mode *mode)
{
	return imx766->link_freq_bitmap & BIT(mode->link_freq_index);
}

static const struct imx766_mode *imx766_nth_mode(struct imx766 *imx766,
						 unsigned int n)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(imx766_modes); i++) {
		if (!imx766_mode_usable(imx766, &imx766_modes[i]))
			continue;
		if (!n--)
			return &imx766_modes[i];
	}

	return NULL;
}

static const struct imx766_mode *
imx766_find_mode(struct imx766 *imx766, u32 width, u32 height)
{
	const struct imx766_mode *best = NULL;
	u32 best_dist = U32_MAX;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(imx766_modes); i++) {
		const struct imx766_mode *mode = &imx766_modes[i];
		u32 dist;

		if (!imx766_mode_usable(imx766, mode))
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

/* -----------------------------------------------------------------------------
 * Controls
 */

static void imx766_update_exposure_range(struct imx766 *imx766, s32 vblank)
{
	s64 max = imx766->mode->height + vblank - IMX766_EXPOSURE_MARGIN;

	__v4l2_ctrl_modify_range(imx766->exposure, imx766->exposure->minimum,
				 max, imx766->exposure->step,
				 min_t(s64, imx766->exposure->val, max));
}

static int imx766_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx766 *imx766 = container_of(ctrl->handler, struct imx766,
					     ctrls);
	struct v4l2_subdev_state *state;
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		imx766_update_exposure_range(imx766, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		/* The flips change the Bayer order of the active format */
		if (imx766->sd.active_state) {
			state = v4l2_subdev_get_locked_active_state(&imx766->sd);
			v4l2_subdev_state_get_format(state, 0)->code =
				imx766_mbus_code(imx766);
		}
		break;
	default:
		break;
	}

	/* The registers are written when streaming starts */
	if (pm_runtime_get_if_active(imx766->dev) <= 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(imx766->regmap, IMX766_REG_EXPOSURE, ctrl->val, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(imx766->regmap, IMX766_REG_ANALOG_GAIN, ctrl->val,
			  &ret);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		cci_write(imx766->regmap, IMX766_REG_DIG_GAIN, ctrl->val, &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(imx766->regmap, IMX766_REG_FRAME_LENGTH,
			  imx766->mode->height + ctrl->val, &ret);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		cci_write(imx766->regmap, IMX766_REG_ORIENTATION,
			  (imx766->hflip->val ? IMX766_ORIENTATION_HFLIP : 0) |
			  (imx766->vflip->val ? IMX766_ORIENTATION_VFLIP : 0),
			  &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(imx766->regmap, IMX766_REG_TEST_PATTERN, ctrl->val,
			  &ret);
		break;
	case V4L2_CID_HBLANK:
	case V4L2_CID_LINK_FREQ:
		/* Read-only, follow the mode: nothing to write */
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(imx766->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx766_ctrl_ops = {
	.s_ctrl = imx766_set_ctrl,
};

/* Update the mode dependent controls; called with the handler lock held */
static int imx766_update_mode_ctrls(struct imx766 *imx766)
{
	const struct imx766_mode *mode = imx766->mode;
	u32 hblank = mode->hts - mode->width;
	u32 vblank = mode->vts_min - mode->height;
	int ret;

	ret = __v4l2_ctrl_s_ctrl(imx766->link_freq, mode->link_freq_index);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_modify_range(imx766->hblank, hblank, hblank, 1,
				       hblank);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_modify_range(imx766->vblank, vblank,
				       IMX766_FRAME_LENGTH_MAX - mode->height,
				       1, vblank);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_s_ctrl(imx766->vblank, vblank);
	if (ret)
		return ret;

	/* Default to the longest exposure the frame allows */
	return __v4l2_ctrl_modify_range(imx766->exposure,
					IMX766_EXPOSURE_MIN,
					mode->vts_min - IMX766_EXPOSURE_MARGIN,
					IMX766_EXPOSURE_STEP,
					mode->vts_min - IMX766_EXPOSURE_MARGIN);
}

static int imx766_init_controls(struct imx766 *imx766)
{
	struct v4l2_ctrl_handler *hdl = &imx766->ctrls;
	const struct imx766_mode *mode = imx766->mode;
	struct v4l2_fwnode_device_properties props;
	u32 hblank = mode->hts - mode->width;
	u32 vblank = mode->vts_min - mode->height;
	u32 exposure_max = mode->vts_min - IMX766_EXPOSURE_MARGIN;
	int ret;

	ret = v4l2_fwnode_device_parse(imx766->dev, &props);
	if (ret)
		return ret;

	v4l2_ctrl_handler_init(hdl, 12);

	imx766->link_freq = v4l2_ctrl_new_int_menu(hdl, &imx766_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(imx766_link_freqs) - 1,
						   mode->link_freq_index,
						   imx766_link_freqs);
	if (imx766->link_freq)
		imx766->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx766->pixel_rate = v4l2_ctrl_new_std(hdl, &imx766_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       IMX766_PIXEL_RATE,
					       IMX766_PIXEL_RATE, 1,
					       IMX766_PIXEL_RATE);

	imx766->hblank = v4l2_ctrl_new_std(hdl, &imx766_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank, 1,
					   hblank);
	if (imx766->hblank)
		imx766->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx766->vblank = v4l2_ctrl_new_std(hdl, &imx766_ctrl_ops,
					   V4L2_CID_VBLANK, vblank,
					   IMX766_FRAME_LENGTH_MAX - mode->height,
					   1, vblank);

	imx766->exposure = v4l2_ctrl_new_std(hdl, &imx766_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX766_EXPOSURE_MIN, exposure_max,
					     IMX766_EXPOSURE_STEP,
					     exposure_max);

	v4l2_ctrl_new_std(hdl, &imx766_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX766_ANA_GAIN_MIN, IMX766_ANA_GAIN_MAX,
			  IMX766_ANA_GAIN_STEP, IMX766_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(hdl, &imx766_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX766_DGTL_GAIN_MIN, IMX766_DGTL_GAIN_MAX,
			  IMX766_DGTL_GAIN_STEP, IMX766_DGTL_GAIN_DEFAULT);

	imx766->hflip = v4l2_ctrl_new_std(hdl, &imx766_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (imx766->hflip)
		imx766->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	imx766->vflip = v4l2_ctrl_new_std(hdl, &imx766_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (imx766->vflip)
		imx766->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(hdl, &imx766_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx766_test_pattern_menu) - 1,
				     0, 0, imx766_test_pattern_menu);

	v4l2_ctrl_new_fwnode_properties(hdl, &imx766_ctrl_ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		dev_err(imx766->dev, "failed to add controls: %d\n", ret);
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	imx766->sd.ctrl_handler = hdl;

	return 0;
}

/* -----------------------------------------------------------------------------
 * Power
 */

static int imx766_power_on(struct device *dev)
{
	struct imx766 *imx766 = to_imx766(dev_get_drvdata(dev));
	int ret;
	int i;

	/*
	 * Enable the rails one at a time: regulator_bulk_enable() does not
	 * keep the order.
	 */
	for (i = 0; i < IMX766_NUM_SUPPLIES; i++) {
		ret = regulator_enable(imx766->supplies[i].consumer);
		if (ret) {
			dev_err(dev, "failed to enable %s: %d\n",
				imx766->supplies[i].supply, ret);
			goto err_supplies;
		}
		usleep_range(1000, 1100);
	}

	ret = clk_prepare_enable(imx766->mclk);
	if (ret) {
		dev_err(dev, "failed to enable MCLK: %d\n", ret);
		goto err_supplies;
	}
	usleep_range(1000, 1100);

	gpiod_set_value_cansleep(imx766->reset_gpio, 0);
	usleep_range(IMX766_XCLR_DELAY_US, IMX766_XCLR_DELAY_US + 1000);

	return 0;

err_supplies:
	while (--i >= 0)
		regulator_disable(imx766->supplies[i].consumer);

	return ret;
}

static int imx766_power_off(struct device *dev)
{
	struct imx766 *imx766 = to_imx766(dev_get_drvdata(dev));
	int i;

	gpiod_set_value_cansleep(imx766->reset_gpio, 1);
	usleep_range(1000, 1100);

	clk_disable_unprepare(imx766->mclk);

	for (i = IMX766_NUM_SUPPLIES - 1; i >= 0; i--)
		regulator_disable(imx766->supplies[i].consumer);

	return 0;
}

/* -----------------------------------------------------------------------------
 * Subdev
 */

static int imx766_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct imx766 *imx766 = to_imx766(sd);
	const struct imx766_mode *mode = imx766->mode;
	int ret;

	ret = pm_runtime_resume_and_get(imx766->dev);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(imx766->regmap, imx766_init_regs,
				  ARRAY_SIZE(imx766_init_regs), NULL);
	if (ret) {
		dev_err(imx766->dev, "failed to write init registers: %d\n",
			ret);
		goto err_rpm_put;
	}

	/* Output format, readout, PLL and C-PHY timing */
	ret = cci_multi_reg_write(imx766->regmap, mode->reg_list.regs,
				  mode->reg_list.num_regs, NULL);
	if (ret) {
		dev_err(imx766->dev, "failed to write mode registers: %d\n",
			ret);
		goto err_rpm_put;
	}

	/* Exposure, gains, frame length, flips and test pattern */
	ret = __v4l2_ctrl_handler_setup(&imx766->ctrls);
	if (ret)
		goto err_rpm_put;

	ret = cci_write(imx766->regmap, IMX766_REG_MODE_SELECT,
			IMX766_MODE_STREAMING, NULL);
	if (ret) {
		dev_err(imx766->dev, "failed to start streaming: %d\n", ret);
		goto err_rpm_put;
	}

	/* The flips change the Bayer order: not while streaming */
	__v4l2_ctrl_grab(imx766->hflip, true);
	__v4l2_ctrl_grab(imx766->vflip, true);

	return 0;

err_rpm_put:
	pm_runtime_put_autosuspend(imx766->dev);

	return ret;
}

static int imx766_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct imx766 *imx766 = to_imx766(sd);
	int ret;

	ret = cci_write(imx766->regmap, IMX766_REG_MODE_SELECT,
			IMX766_MODE_STANDBY, NULL);
	if (ret)
		dev_err(imx766->dev, "failed to stop streaming: %d\n", ret);

	__v4l2_ctrl_grab(imx766->hflip, false);
	__v4l2_ctrl_grab(imx766->vflip, false);

	pm_runtime_put_autosuspend(imx766->dev);

	return ret;
}

static void imx766_fill_format(struct imx766 *imx766,
			       const struct imx766_mode *mode,
			       struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = imx766_mbus_code(imx766);
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int imx766_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = imx766_mbus_code(to_imx766(sd));

	return 0;
}

static int imx766_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx766 *imx766 = to_imx766(sd);
	const struct imx766_mode *mode;

	if (fse->code != imx766_mbus_code(imx766))
		return -EINVAL;

	mode = imx766_nth_mode(imx766, fse->index);
	if (!mode)
		return -EINVAL;

	fse->min_width = mode->width;
	fse->max_width = mode->width;
	fse->min_height = mode->height;
	fse->max_height = mode->height;

	return 0;
}

static int imx766_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct imx766 *imx766 = to_imx766(sd);
	const struct imx766_mode *mode;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    v4l2_subdev_is_streaming(sd))
		return -EBUSY;

	mode = imx766_find_mode(imx766, fmt->format.width, fmt->format.height);

	imx766_fill_format(imx766, mode, &fmt->format);
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;
	*v4l2_subdev_state_get_crop(state, 0) = mode->crop;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		imx766->mode = mode;
		return imx766_update_mode_ctrls(imx766);
	}

	return 0;
}

static int imx766_get_selection(struct v4l2_subdev *sd,
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
		sel->r.width = IMX766_NATIVE_WIDTH;
		sel->r.height = IMX766_NATIVE_HEIGHT;
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = IMX766_PIXEL_ARRAY_LEFT;
		sel->r.top = IMX766_PIXEL_ARRAY_TOP;
		sel->r.width = IMX766_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX766_PIXEL_ARRAY_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static int imx766_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct imx766 *imx766 = to_imx766(sd);
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.width = imx766->mode->width,
			.height = imx766->mode->height,
		},
	};

	return imx766_set_fmt(sd, state, &fmt);
}

static const struct v4l2_subdev_video_ops imx766_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops imx766_pad_ops = {
	.enum_mbus_code = imx766_enum_mbus_code,
	.enum_frame_size = imx766_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx766_set_fmt,
	.get_selection = imx766_get_selection,
	.enable_streams = imx766_enable_streams,
	.disable_streams = imx766_disable_streams,
};

static const struct v4l2_subdev_ops imx766_subdev_ops = {
	.video = &imx766_video_ops,
	.pad = &imx766_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx766_internal_ops = {
	.init_state = imx766_init_state,
};

/* -----------------------------------------------------------------------------
 * Probe
 */

static int imx766_identify(struct imx766 *imx766)
{
	u64 id;
	int ret;

	ret = cci_read(imx766->regmap, IMX766_REG_MODEL_ID, &id, NULL);
	if (ret)
		return dev_err_probe(imx766->dev, ret,
				     "failed to read model ID\n");

	if (id != IMX766_MODEL_ID)
		return dev_err_probe(imx766->dev, -ENODEV,
				     "unexpected model ID 0x%04llx (want 0x%04x)\n",
				     id, IMX766_MODEL_ID);

	dev_info(imx766->dev, "Sony IMX766 detected, model ID 0x%04llx\n", id);

	return 0;
}

static int imx766_check_hwcfg(struct imx766 *imx766)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_CPHY,
	};
	struct device *dev = imx766->dev;
	struct fwnode_handle *ep;
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!ep)
		return dev_err_probe(dev, -EINVAL, "no endpoint\n");

	/* Fails unless the endpoint's bus-type is C-PHY */
	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to parse endpoint (C-PHY bus-type required)\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != IMX766_DATA_LANES) {
		ret = dev_err_probe(dev, -EINVAL,
				    "%u data lanes not supported, need %u trios\n",
				    bus_cfg.bus.mipi_csi2.num_data_lanes,
				    IMX766_DATA_LANES);
		goto out;
	}

	/*
	 * Each mode has its own link frequency, half the C-PHY symbol rate
	 * per trio; the modes whose frequency is not listed are hidden.
	 */
	ret = v4l2_link_freq_to_bitmap(dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       imx766_link_freqs,
				       ARRAY_SIZE(imx766_link_freqs),
				       &imx766->link_freq_bitmap);

out:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int imx766_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx766 *imx766;
	unsigned long rate;
	int ret;
	int i;

	imx766 = devm_kzalloc(dev, sizeof(*imx766), GFP_KERNEL);
	if (!imx766)
		return -ENOMEM;

	imx766->dev = dev;

	ret = imx766_check_hwcfg(imx766);
	if (ret)
		return ret;

	/* Also sets the client data to the subdev */
	v4l2_i2c_subdev_init(&imx766->sd, client, &imx766_subdev_ops);

	imx766->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx766->regmap))
		return dev_err_probe(dev, PTR_ERR(imx766->regmap),
				     "failed to initialize CCI\n");

	for (i = 0; i < IMX766_NUM_SUPPLIES; i++)
		imx766->supplies[i].supply = imx766_supply_names[i];

	ret = devm_regulator_bulk_get(dev, IMX766_NUM_SUPPLIES,
				      imx766->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get supplies\n");

	imx766->mclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(imx766->mclk))
		return dev_err_probe(dev, PTR_ERR(imx766->mclk),
				     "failed to get MCLK\n");

	rate = clk_get_rate(imx766->mclk);
	if (rate != IMX766_MCLK_RATE)
		return dev_err_probe(dev, -EINVAL,
				     "unsupported MCLK rate %lu\n", rate);

	/* Hold the sensor in reset until it is powered */
	imx766->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(imx766->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(imx766->reset_gpio),
				     "failed to get reset GPIO\n");

	ret = imx766_power_on(dev);
	if (ret)
		return ret;

	ret = imx766_identify(imx766);
	if (ret)
		goto err_power_off;

	if (imx766_mode_usable(imx766, IMX766_DEFAULT_MODE))
		imx766->mode = IMX766_DEFAULT_MODE;
	else
		imx766->mode = imx766_nth_mode(imx766, 0);

	ret = imx766_init_controls(imx766);
	if (ret)
		goto err_power_off;

	imx766->sd.internal_ops = &imx766_internal_ops;
	imx766->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx766->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx766->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx766->sd.entity, 1, &imx766->pad);
	if (ret)
		goto err_ctrls;

	imx766->sd.state_lock = imx766->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&imx766->sd);
	if (ret)
		goto err_entity;

	/* Keep the sensor powered until it is idled below */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&imx766->sd);
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
	v4l2_subdev_cleanup(&imx766->sd);
err_entity:
	media_entity_cleanup(&imx766->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&imx766->ctrls);
err_power_off:
	imx766_power_off(dev);

	return ret;
}

static void imx766_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx766 *imx766 = to_imx766(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&imx766->ctrls);

	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		imx766_power_off(dev);
	pm_runtime_set_suspended(dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx766_pm_ops, imx766_power_off,
				 imx766_power_on, NULL);

static const struct of_device_id imx766_of_match[] = {
	{ .compatible = "sony,imx766" },
	{ }
};
MODULE_DEVICE_TABLE(of, imx766_of_match);

static struct i2c_driver imx766_driver = {
	.driver = {
		.name = "imx766",
		.of_match_table = imx766_of_match,
		.pm = pm_ptr(&imx766_pm_ops),
	},
	.probe = imx766_probe,
	.remove = imx766_remove,
};
module_i2c_driver(imx766_driver);

MODULE_DESCRIPTION("Sony IMX766 image sensor driver");
MODULE_LICENSE("GPL");

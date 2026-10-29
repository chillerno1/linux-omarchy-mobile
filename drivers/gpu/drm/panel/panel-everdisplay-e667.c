// SPDX-License-Identifier: GPL-2.0-only
// Generated with linux-mdss-dsi-panel-driver-generator from the Solana Mobile
// Saga's vendor device tree (qcom,mdss_dsi_e667_everdisplay_fhd_plus_cmd).

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/device-id/of.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct e667_everdisplay_plus {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data *supplies;
};

static const struct regulator_bulk_data e667_everdisplay_plus_supplies[] = {
	{ .supply = "vddio" },
	{ .supply = "vci" },
	{ .supply = "extvdd" },
};

static inline
struct e667_everdisplay_plus *to_e667_everdisplay_plus(struct drm_panel *panel)
{
	return container_of_const(panel, struct e667_everdisplay_plus, panel);
}

static void e667_everdisplay_plus_reset(struct e667_everdisplay_plus *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(15000, 16000);
}

static int e667_everdisplay_plus_on(struct e667_everdisplay_plus *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x03, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x59, 0x09);
	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xaa, 0x18);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0,
				     0x13, 0x7c, 0x01, 0x20, 0x05, 0xd6, 0x01,
				     0x00, 0x5a, 0xa3, 0xd4, 0x01, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb1,
				     0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb2,
				     0x13, 0x01, 0x00, 0xfe, 0x1b, 0x3c, 0x3c,
				     0x87, 0x00, 0x10, 0x28);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb3,
				     0x1f, 0x81, 0x09, 0x01, 0x00, 0x01, 0x00,
				     0x16, 0x2b, 0x34, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb5,
				     0x95, 0x1d, 0x1e, 0x18, 0x1d, 0x1d, 0x04,
				     0x1e, 0x18, 0x1d, 0x04, 0x20, 0x14, 0x1e,
				     0x1d, 0x04, 0x1c, 0x16, 0x1c, 0x32, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0,
				     0x80, 0xec, 0x6a, 0xf1, 0xc8, 0xfb, 0x89,
				     0xc8, 0xc7, 0x54, 0xb2, 0x95, 0x8a, 0xd4,
				     0x78);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc1,
				     0x48, 0x24, 0x52, 0x49, 0x24, 0x9a, 0x50,
				     0xea, 0x95, 0x3b, 0x25, 0x8e, 0xe7, 0x73,
				     0xb8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc2,
				     0x4b, 0x9e, 0x0f, 0x07, 0x83, 0xc1, 0xe0,
				     0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83,
				     0xc0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3,
				     0x2c, 0x16, 0x0e, 0xe7, 0x73, 0xb9, 0x5c,
				     0xae, 0x76, 0x2b, 0x15, 0x8e, 0xa5, 0x52,
				     0xa8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc4,
				     0x2a, 0x15, 0x0e, 0x65, 0x33, 0x91, 0x48,
				     0xa4, 0x51, 0x28, 0x94, 0x0a, 0x04, 0xf2,
				     0x78);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5,
				     0x27, 0x1b, 0x49, 0xa4, 0xc2, 0x61, 0x2c,
				     0xd4, 0x4a, 0x24, 0x92, 0x49, 0x04, 0x72,
				     0x38);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc6,
				     0x23, 0x11, 0x4c, 0x84, 0x42, 0x19, 0x08,
				     0x84, 0x41, 0x20, 0x17, 0xcb, 0xe5, 0xe2,
				     0xe8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc7,
				     0x2e, 0x17, 0x0b, 0x65, 0xa2, 0xc9, 0x60,
				     0xae, 0x57, 0x2b, 0x15, 0x4a, 0x85, 0x32,
				     0x90);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc8,
				     0x28, 0x8c, 0x46, 0x02, 0xf1, 0x70, 0xb4,
				     0x58, 0x2b, 0x15, 0x0a, 0x45, 0x02, 0x71,
				     0x30);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc9,
				     0x12, 0x89, 0x04, 0x62, 0x21, 0x08, 0x04,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe0,
				     0x39, 0xcc, 0x52, 0x0c, 0x2b, 0x49, 0x86,
				     0x04, 0x98, 0x4b, 0x30, 0x54, 0x56, 0x9c,
				     0x49);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe1,
				     0x1a, 0xc8, 0xb2, 0x2c, 0x8a, 0x12, 0x40,
				     0x76, 0x91, 0x41, 0x33, 0x04, 0x76, 0x08,
				     0xeb);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe2,
				     0x09, 0x54, 0xe1, 0x9d, 0x4e, 0x11, 0x92,
				     0xd0, 0x0c, 0xc9, 0x63, 0x82, 0x43, 0x21,
				     0x4c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe3,
				     0x74, 0x25, 0x3a, 0xda, 0xf9, 0xd6, 0xf9,
				     0xce, 0xfb, 0xdf, 0xff, 0xe5, 0x29, 0x46,
				     0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe4,
				     0x6d, 0xeb, 0x38, 0x36, 0xd3, 0x7e, 0x26,
				     0xfb, 0xcb, 0x75, 0x7d, 0xe3, 0x99, 0x66,
				     0x59);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe5,
				     0x96, 0x63, 0x77, 0xd7, 0x93, 0xcb, 0xeb,
				     0xb8, 0xdb, 0x71, 0xb6, 0xe1, 0x5c, 0xf6,
				     0x56);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe6,
				     0xd7, 0xa5, 0x5c, 0xf2, 0x13, 0xb6, 0x79,
				     0xe8, 0xce, 0xd8, 0xd7, 0x3d, 0xde, 0xf7,
				     0xbc);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe7,
				     0xde, 0x40, 0x0f, 0xf3, 0x20, 0x07, 0x00,
				     0x0e, 0x00, 0x00, 0x07, 0x41, 0xf0, 0x00,
				     0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe8,
				     0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x3c,
				     0x00, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x00,
				     0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x65, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0,
				     0x00, 0xc0, 0x28, 0x0c, 0x64, 0x42, 0x24,
				     0xd4, 0x8a, 0x35, 0x9a, 0xcd, 0x66, 0xb2,
				     0x58);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc1,
				     0x25, 0x9a, 0x89, 0x44, 0xa3, 0x49, 0x24,
				     0x90, 0x48, 0x23, 0x91, 0x88, 0xc4, 0x52,
				     0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc2,
				     0x21, 0x90, 0x88, 0x24, 0x01, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe0,
				     0x21, 0x04, 0xc4, 0x95, 0xa7, 0x01, 0xda,
				     0x44, 0xb8, 0x46, 0x4a, 0xda, 0xe7, 0xc2,
				     0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe1,
				     0x76, 0x6d, 0x2d, 0xd5, 0xd5, 0xe4, 0xf1,
				     0xd9, 0x57, 0x1b, 0xef, 0xbf, 0xad, 0xe8,
				     0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe2,
				     0xf8, 0x01, 0xd0, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6c, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6d, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xaa, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd0,
				     0x84, 0x13, 0x10, 0x20, 0x20, 0x00, 0x35,
				     0x4e, 0x1d, 0x23, 0x00, 0x00, 0x4e, 0x23,
				     0x50, 0x51, 0x00, 0x00, 0x03, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xaa, 0x12);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc7, 0x2f, 0x73, 0x32, 0x74);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xaa, 0x15);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbd, 0x22);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbe,
				     0x33, 0x11, 0x11, 0x11, 0x11, 0x11, 0x44,
				     0x44, 0x44, 0x44, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x5a, 0x82);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf9, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x5a, 0x83);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x65, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf8, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xaa, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x5a, 0x00);
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 96);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_usleep_range(&dsi_ctx, 17000, 18000);

	return dsi_ctx.accum_err;
}

static int e667_everdisplay_plus_off(struct e667_everdisplay_plus *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_usleep_range(&dsi_ctx, 17000, 18000);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_usleep_range(&dsi_ctx, 9000, 10000);

	return dsi_ctx.accum_err;
}

static int e667_everdisplay_plus_prepare(struct drm_panel *panel)
{
	struct e667_everdisplay_plus *ctx = to_e667_everdisplay_plus(panel);
	struct device *dev = &ctx->dsi->dev;
	struct drm_dsc_picture_parameter_set pps;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(e667_everdisplay_plus_supplies),
				    ctx->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators: %d\n", ret);
		return ret;
	}
	usleep_range(1000, 2000);

	e667_everdisplay_plus_reset(ctx);

	ret = e667_everdisplay_plus_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_bulk_disable(ARRAY_SIZE(e667_everdisplay_plus_supplies),
				       ctx->supplies);
		return ret;
	}

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);

	ret = mipi_dsi_picture_parameter_set(ctx->dsi, &pps);
	if (ret < 0) {
		dev_err(panel->dev, "failed to transmit PPS: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_compression_mode(ctx->dsi, true);
	if (ret < 0) {
		dev_err(dev, "failed to enable compression mode: %d\n", ret);
		return ret;
	}

	msleep(28); /* TODO: Is this panel-dependent? */

	return 0;
}

static int e667_everdisplay_plus_unprepare(struct drm_panel *panel)
{
	struct e667_everdisplay_plus *ctx = to_e667_everdisplay_plus(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = e667_everdisplay_plus_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(e667_everdisplay_plus_supplies),
			       ctx->supplies);

	return 0;
}

static const struct drm_display_mode e667_everdisplay_plus_mode = {
	.clock = (1080 + 32 + 2 + 32) * (2400 + 16 + 2 + 16) * 120 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 32,
	.hsync_end = 1080 + 32 + 2,
	.htotal = 1080 + 32 + 2 + 32,
	.vdisplay = 2400,
	.vsync_start = 2400 + 16,
	.vsync_end = 2400 + 16 + 2,
	.vtotal = 2400 + 16 + 2 + 16,
	.width_mm = 70,
	.height_mm = 156,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int e667_everdisplay_plus_get_modes(struct drm_panel *panel,
					   struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &e667_everdisplay_plus_mode);
}

static const struct drm_panel_funcs e667_everdisplay_plus_panel_funcs = {
	.prepare = e667_everdisplay_plus_prepare,
	.unprepare = e667_everdisplay_plus_unprepare,
	.get_modes = e667_everdisplay_plus_get_modes,
};

static int e667_everdisplay_plus_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

// TODO: Check if /sys/class/backlight/.../actual_brightness actually returns
// correct values. If not, remove this function.
static int e667_everdisplay_plus_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness_large(dsi, &brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return brightness;
}

static const struct backlight_ops e667_everdisplay_plus_bl_ops = {
	.update_status = e667_everdisplay_plus_bl_update_status,
	.get_brightness = e667_everdisplay_plus_bl_get_brightness,
};

static struct backlight_device *
e667_everdisplay_plus_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 4095,
		.max_brightness = 4095,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &e667_everdisplay_plus_bl_ops, &props);
}

static int e667_everdisplay_plus_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct e667_everdisplay_plus *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct e667_everdisplay_plus, panel,
				   &e667_everdisplay_plus_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ret = devm_regulator_bulk_get_const(dev,
					    ARRAY_SIZE(e667_everdisplay_plus_supplies),
					    e667_everdisplay_plus_supplies,
					    &ctx->supplies);
	if (ret < 0)
		return ret;

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS |
			  MIPI_DSI_MODE_DSC_ALL_SLICES_IN_PKT;

	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = e667_everdisplay_plus_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	ret = devm_drm_panel_add(dev, &ctx->panel);
	if (ret)
		return ret;

	/* This panel only supports DSC; unconditionally enable it */
	dsi->dsc = &ctx->dsc;

	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;

	/* Both slices of a line go in one packet (MIPI_DSI_MODE_DSC_ALL_SLICES_IN_PKT) */
	ctx->dsc.slice_height = 40;
	ctx->dsc.slice_width = 540;
	/*
	 * TODO: hdisplay should be read from the selected mode once
	 * it is passed back to drm_panel (in prepare?)
	 */
	WARN_ON(1080 % ctx->dsc.slice_width);
	ctx->dsc.slice_count = 1080 / ctx->dsc.slice_width;
	ctx->dsc.bits_per_component = 8;
	ctx->dsc.bits_per_pixel = 8 << 4; /* 4 fractional bits */
	ctx->dsc.block_pred_enable = true;

	return devm_mipi_dsi_attach(dev, dsi);
}

static const struct of_device_id e667_everdisplay_plus_of_match[] = {
	{ .compatible = "everdisplay,e667" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, e667_everdisplay_plus_of_match);

static struct mipi_dsi_driver e667_everdisplay_plus_driver = {
	.probe = e667_everdisplay_plus_probe,
	.driver = {
		.name = "panel-everdisplay-e667",
		.of_match_table = e667_everdisplay_plus_of_match,
	},
};
module_mipi_dsi_driver(e667_everdisplay_plus_driver);

MODULE_DESCRIPTION("DRM driver for e667 amoled cmd mode dsi everdisplay panel");
MODULE_LICENSE("GPL");

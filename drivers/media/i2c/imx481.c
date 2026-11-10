// SPDX-License-Identifier: GPL-2.0
/*
 * Sony IMX481 image sensor driver
 *
 * For now the driver powers the sensor up, identifies it and powers it down
 * again, and registers a V4L2 subdev with one fixed placeholder format so
 * that a CSI-2 receiver can build its media graph. It cannot stream yet.
 * The register layout follows the SMIA++ style map shared by Sony's mobile
 * sensors; the OTP identification sequence is based on the GPL-2.0 MediaTek
 * imgsensor driver for this sensor.
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

#define IMX481_REG_CHIP_ID		CCI_REG16(0x0016)
#define IMX481_CHIP_ID			0x0481

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

/*
 * Placeholder mode until mode tables are added: full resolution RAW10 on
 * four D-PHY lanes at a 918 MHz link frequency (734.4 Mpixel/s). Nothing
 * programs the sensor for it yet.
 */
#define IMX481_DATA_LANES		4
#define IMX481_BPP			10
#define IMX481_LINK_FREQ		918000000LL
#define IMX481_PIXEL_RATE		(IMX481_LINK_FREQ * 2 * IMX481_DATA_LANES / \
					 IMX481_BPP)
#define IMX481_WIDTH			4656
#define IMX481_HEIGHT			3496
#define IMX481_MBUS_CODE		MEDIA_BUS_FMT_SRGGB10_1X10

static const s64 imx481_link_freqs[] = {
	IMX481_LINK_FREQ,
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
};

static inline struct imx481 *to_imx481(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx481, sd);
}

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
		dev_info(imx481->dev, "chip id 0x%04llx\n", id);
		return 0;
	}

	dev_info(imx481->dev, "register 0x0016 reads 0x%04llx, trying OTP\n",
		id);

	ret = imx481_read_otp_id(imx481, &id);
	if (ret)
		return dev_err_probe(imx481->dev, ret,
				     "failed to read chip id from OTP\n");

	dev_info(imx481->dev, "chip id 0x%04llx (OTP)\n", id);

	if (id != IMX481_CHIP_ID)
		return dev_err_probe(imx481->dev, -ENODEV,
				     "chip id mismatch: expected 0x%04x\n",
				     IMX481_CHIP_ID);

	return 0;
}

/*
 * Placeholder subdev: one fixed format and no stream operations. It lets a
 * CSI-2 receiver link to the sensor, but no frames come out yet.
 */
static void imx481_fill_format(struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = IMX481_WIDTH;
	fmt->height = IMX481_HEIGHT;
	fmt->code = IMX481_MBUS_CODE;
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

	code->code = IMX481_MBUS_CODE;

	return 0;
}

static int imx481_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index || fse->code != IMX481_MBUS_CODE)
		return -EINVAL;

	fse->min_width = IMX481_WIDTH;
	fse->max_width = IMX481_WIDTH;
	fse->min_height = IMX481_HEIGHT;
	fse->max_height = IMX481_HEIGHT;

	return 0;
}

static const struct v4l2_subdev_pad_ops imx481_pad_ops = {
	.enum_mbus_code = imx481_enum_mbus_code,
	.enum_frame_size = imx481_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = v4l2_subdev_get_fmt,	/* one fixed format */
};

static const struct v4l2_subdev_ops imx481_subdev_ops = {
	.pad = &imx481_pad_ops,
};

static int imx481_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	imx481_fill_format(v4l2_subdev_state_get_format(state, 0));

	return 0;
}

static const struct v4l2_subdev_internal_ops imx481_internal_ops = {
	.init_state = imx481_init_state,
};

static int imx481_init_controls(struct imx481 *imx481)
{
	struct v4l2_ctrl_handler *hdl = &imx481->ctrls;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	int ret;

	ret = v4l2_fwnode_device_parse(imx481->dev, &props);
	if (ret)
		return ret;

	v4l2_ctrl_handler_init(hdl, 4);

	ctrl = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(imx481_link_freqs) - 1, 0,
				      imx481_link_freqs);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE, IMX481_PIXEL_RATE,
			  IMX481_PIXEL_RATE, 1, IMX481_PIXEL_RATE);

	v4l2_ctrl_new_fwnode_properties(hdl, NULL, &props);

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	imx481->sd.ctrl_handler = hdl;

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

	/* Let runtime PM power the sensor down until it is needed */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&imx481->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register subdev\n");
		goto err_pm;
	}

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

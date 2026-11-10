// SPDX-License-Identifier: GPL-2.0
/*
 * Sony IMX563 image sensor driver
 *
 * For now the driver only powers the sensor up, identifies it and powers it
 * down again. The model ID register follows the SMIA++ style map shared by
 * Sony's mobile sensors.
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

#define IMX563_REG_MODEL_ID		CCI_REG16(0x0016)
#define IMX563_MODEL_ID			0x0563

#define IMX563_MCLK_RATE		24000000

/* Delay from XCLR release to the first I2C access */
#define IMX563_XCLR_DELAY_US		10000

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
};

static int imx563_power_on(struct device *dev)
{
	struct imx563 *imx563 = dev_get_drvdata(dev);
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
	struct imx563 *imx563 = dev_get_drvdata(dev);
	int i;

	gpiod_set_value_cansleep(imx563->reset_gpio, 1);
	usleep_range(1000, 1100);

	clk_disable_unprepare(imx563->mclk);

	for (i = IMX563_NUM_SUPPLIES - 1; i >= 0; i--)
		regulator_disable(imx563->supplies[i].consumer);

	return 0;
}

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
	i2c_set_clientdata(client, imx563);

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
				     "unsupported MCLK rate %lu\n", rate);

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
	if (ret) {
		imx563_power_off(dev);
		return ret;
	}

	/* Let runtime PM power the sensor down until it is needed */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;
}

static void imx563_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;

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

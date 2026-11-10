// SPDX-License-Identifier: GPL-2.0
/*
 * Sony IMX766 image sensor driver
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

#define IMX766_REG_MODEL_ID		CCI_REG16(0x0016)
#define IMX766_MODEL_ID			0x0766

#define IMX766_MCLK_RATE		24000000

/* Delay from XCLR release to the first I2C access */
#define IMX766_XCLR_DELAY_US		10000

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
};

static int imx766_power_on(struct device *dev)
{
	struct imx766 *imx766 = dev_get_drvdata(dev);
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
	struct imx766 *imx766 = dev_get_drvdata(dev);
	int i;

	gpiod_set_value_cansleep(imx766->reset_gpio, 1);
	usleep_range(1000, 1100);

	clk_disable_unprepare(imx766->mclk);

	for (i = IMX766_NUM_SUPPLIES - 1; i >= 0; i--)
		regulator_disable(imx766->supplies[i].consumer);

	return 0;
}

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
	i2c_set_clientdata(client, imx766);

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
	if (ret) {
		imx766_power_off(dev);
		return ret;
	}

	/* Let runtime PM power the sensor down until it is needed */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;
}

static void imx766_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;

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

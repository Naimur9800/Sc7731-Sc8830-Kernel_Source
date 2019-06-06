/*
 * SPDX-License-Identifier: GPL-2.0
 * Regulator device driver for SC2703
 *
 * Copyright (c) 2018 Dialog Semiconductor.
 */

#include <linux/debugfs.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include "sc2703-regulator.h"

#define SC2703_M_IRQ_ALL (SC2703_M_PWRGOOD1_MASK)
#define SC2703_M_IRQ_MAKS (SC2703_M_VSYS_UV_OT_VREF_FLT_MASK \
			   | SC2703_M_OVCURR1_MASK)
#define SC2703_IRQ_FLAGS (IRQF_TRIGGER_LOW | IRQF_ONESHOT \
	 | IRQF_SHARED)

#define SC2703_BUCK_MODE_PFM	0		/* PFM */
#define SC2703_BUCK_MODE_PWM_FULL	1	/* PWM FULL phase */
#define SC2703_BUCK_MODE_PWM_SHED	2	/* PWM with phase shedding */
#define SC2703_BUCK_MODE_AUTO	3		/* Auto */

static struct dentry *regu_debugfs;

struct sc2703_regulator {
	struct regulator_init_data init_data;
};

struct sc2703_buck {
	struct device *dev;
	struct regmap *regmap;
	struct regulator_dev *rdev;
	struct sc2703_regulator *pdata;

	int ch1_b_vout;
	int irq;
};

struct sc2703_buck_voltage {
	int min_uV;
	int max_uV;
	int uV_step;
};

static bool sc2703_buck_volatile_reg(struct device *dev, u32 reg)
{
	switch (reg) {
	case SC2703_COREBUCK_EVENT:
	case SC2703_COREBUCK_STATUS:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config sc2703_buck_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_reg = sc2703_buck_volatile_reg,
};


/* Current limit for SC2703 Core-BUCK */
static const int sc2703_buck_limits[] = {
	5600000, 6000000, 6400000, 6800000, 7200000, /* 1 -5 */
	7600000, 8000000, 8400000, 8800000, 9200000, /* 6 - 10 */
	9600000, 10000000, 10400000, 10800000 /* 11 - 14 */
};

/* Default limits measured in millivolts and milliamps */
#define SC2703_BUCK_MIN_MV		300
#define SC2703_BUCK_MAX_MV		1570
#define SC2703_BUCK_STEP_MV		10
#define SC2703_BUCK_CH1_B_DEF	0x46 /* 1V */

static const struct sc2703_buck_voltage sc2703_buck_vol = {
	.min_uV = SC2703_BUCK_MIN_MV * 1000,
	.max_uV = SC2703_BUCK_MAX_MV * 1000,
	.uV_step = SC2703_BUCK_STEP_MV * 1000,
};

static u32 sc2703_buck_get_mode(struct regulator_dev *rdev)
{
	struct sc2703_buck *chip = rdev_get_drvdata(rdev);
	u32 data;
	int ret, mode = 0;

	ret = regmap_read(chip->regmap, SC2703_COREBUCK_BUCK1_7, &data);
	if (ret < 0)
		return ret;

	switch (data & SC2703_CH1_A_CMD_MASK) {
	case SC2703_BUCK_MODE_PWM_FULL:
		mode = REGULATOR_MODE_FAST;
		break;
	case SC2703_BUCK_MODE_PWM_SHED:
		mode = REGULATOR_MODE_IDLE;
		break;
	case SC2703_BUCK_MODE_AUTO:
		mode = REGULATOR_MODE_NORMAL;
		break;
	case SC2703_BUCK_MODE_PFM:
		mode = REGULATOR_MODE_STANDBY;
		break;
	}

	return mode;
}

static int sc2703_buck_set_mode(struct regulator_dev *rdev, u32 mode)
{
	struct sc2703_buck *chip = rdev_get_drvdata(rdev);
	u32 val;

	switch (mode) {
	case REGULATOR_MODE_FAST:
		val = SC2703_BUCK_MODE_PWM_FULL;
		break;
	case REGULATOR_MODE_IDLE:
		val = SC2703_BUCK_MODE_PWM_SHED;
		break;
	case REGULATOR_MODE_NORMAL:
		val = SC2703_BUCK_MODE_AUTO;
		break;
	case REGULATOR_MODE_STANDBY:
		val = SC2703_BUCK_MODE_PFM;
		break;
	default:
		return -EINVAL;
	}

	return regmap_update_bits(chip->regmap,
			SC2703_COREBUCK_BUCK1_7,
			SC2703_CH1_A_CMD_MASK, val);
}

static int sc2703_set_current_limit(struct regulator_dev *rdev, int min_uA,
				    int max_uA)
{
	struct sc2703_buck *chip = rdev_get_drvdata(rdev);
	u32 sel;
	int i;

	/* search for closest to maximum */
	for (i = ARRAY_SIZE(sc2703_buck_limits) - 1; i >= 0; i--) {
		if (min_uA <= sc2703_buck_limits[i] &&
		    max_uA >= sc2703_buck_limits[i]) {
			sel = (i + 1) << SC2703_CH1_ILIM_SHIFT;
			dev_info(chip->dev,
				"current limit val(%d), current(%d)\n",
				sel, sc2703_buck_limits[i]);
			return regmap_update_bits(chip->regmap,
					SC2703_COREBUCK_BUCK1_3,
					SC2703_CH1_ILIM_MASK, sel);
		}
	}

	return -EINVAL;
}

static int sc2703_get_current_limit(struct regulator_dev *rdev)
{
	struct sc2703_buck *chip = rdev_get_drvdata(rdev);
	u32 data;
	int ret;

	ret = regmap_read(chip->regmap,	SC2703_COREBUCK_BUCK1_3, &data);
	if (ret < 0)
		return ret;

	data = (data & SC2703_CH1_ILIM_MASK) >> SC2703_CH1_ILIM_SHIFT;
	return sc2703_buck_limits[data - 1];
}

static struct regulator_ops sc2703_buck_ops = {
	.get_mode = sc2703_buck_get_mode,
	.set_mode = sc2703_buck_set_mode,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.list_voltage = regulator_list_voltage_linear,
	.set_current_limit = sc2703_set_current_limit,
	.get_current_limit = sc2703_get_current_limit,
};

static const struct regulator_desc sc2703_buck_reg = {
	.name = "SC2703_BUCK_CH1",
	.id = 0,
	.ops = &sc2703_buck_ops,
	.type = REGULATOR_VOLTAGE,
	.n_voltages = ((SC2703_BUCK_MAX_MV - SC2703_BUCK_MIN_MV)
				/ SC2703_BUCK_STEP_MV) + 1,
	.min_uV = (SC2703_BUCK_MIN_MV * 1000),
	.uV_step = (SC2703_BUCK_STEP_MV * 1000),
	.vsel_reg = SC2703_COREBUCK_BUCK1_0,
	.vsel_mask = SC2703_CH1_A_VOUT_MASK,
	.enable_reg = SC2703_COREBUCK_BUCK1_5,
	.enable_mask = SC2703_CH1_EN_MASK,
	.owner = THIS_MODULE,
};

static int sc2703_regulators_parse_dt(
		struct sc2703_buck *chip, struct regulator_config *config)
{
	struct device *dev = chip->dev;
	struct device_node *node = dev->of_node;
	struct sc2703_regulator *pdata = chip->pdata;
	u32 val;
	struct gpio_desc *gpio_buck, *gpio_enable;
	int ret;

	gpio_buck = devm_gpiod_get(dev, "buck", GPIOD_IN);
	ret = PTR_ERR_OR_ZERO(gpio_buck);

	switch (ret) {
	case 0:
		chip->irq = gpiod_to_irq(gpio_buck);
		break;
	case -ENOENT:
		ret = 0;
		break;
	default:
		dev_err(dev, "Failed to find buck gpio: %d\n", ret);
		/* fall through */
	case -EPROBE_DEFER:
		return ret;
	}

	if (of_property_read_u32(node,
		"sprd,buck-ch1-b-out", &val) >= 0) {
		if (val <= (SC2703_BUCK_MAX_MV * 1000) &&
			val >= (SC2703_BUCK_MIN_MV * 1000))
			chip->ch1_b_vout =
				(val - SC2703_BUCK_MIN_MV * 1000) / 10000;
		else
			chip->ch1_b_vout = SC2703_BUCK_CH1_B_DEF;
	} else {
		chip->ch1_b_vout = SC2703_BUCK_CH1_B_DEF;
	}

	config->dev = dev;
	config->init_data = pdata ? &pdata->init_data :
		of_get_regulator_init_data(dev, dev->of_node,
			&sc2703_buck_reg);
	config->driver_data = chip;
	config->regmap = chip->regmap;
	config->of_node = dev->of_node;
	gpio_enable = devm_gpiod_get(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(gpio_enable)) {
		config->ena_gpio = -EINVAL;
		config->ena_gpio_initialized = false;
	} else {
		config->ena_gpio = desc_to_gpio(gpio_enable);
		config->ena_gpio_initialized = true;
		config->ena_gpio_flags |= GPIOD_FLAGS_BIT_DIR_OUT;
		if (config->init_data->constraints.boot_on ||
			config->init_data->constraints.always_on)
			config->ena_gpio_flags |= GPIOD_OUT_HIGH;
	}

	return ret;
}

static int sc2703_buck_init(struct sc2703_buck *chip)
{
	return regmap_update_bits(chip->regmap,
				  SC2703_COREBUCK_BUCK1_6,
				  SC2703_CH1_B_VOUT_MASK,
				  chip->ch1_b_vout &
				  SC2703_CH1_B_VOUT_MASK);
}

static irqreturn_t sc2703_buck_irq_handler(int irq, void *data)
{
	struct sc2703_buck *chip = data;
	int reg_val, ret, mask = 0;

	ret = regmap_read(chip->regmap, SC2703_COREBUCK_EVENT, &reg_val);
	if (ret < 0)
		return IRQ_RETVAL(ret);

	if (reg_val & SC2703_EVT_PWRGOOD1_MASK) {
		mask = SC2703_EVT_PWRGOOD1_MASK;
		if (chip->rdev != NULL)
			regulator_notifier_call_chain(chip->rdev,
				REGULATOR_EVENT_VOLTAGE_CHANGE,
				NULL);
	}

	if (reg_val & SC2703_EVT_OVCURR1_MASK) {
		mask |= SC2703_EVT_OVCURR1_MASK;
		if (chip->rdev != NULL)
			regulator_notifier_call_chain(chip->rdev,
				REGULATOR_EVENT_OVER_CURRENT,
				NULL);
	}

	if (reg_val & SC2703_EVT_VSYS_UV_OT_VREF_FLT_MASK) {
		mask |= SC2703_EVT_VSYS_UV_OT_VREF_FLT_MASK;
		if (chip->rdev != NULL)
			regulator_notifier_call_chain(chip->rdev,
				REGULATOR_EVENT_FAIL,
				NULL);
	}
	if (mask) {
		ret = regmap_write(chip->regmap,
				   SC2703_COREBUCK_EVENT, reg_val);
		if (ret < 0)
			return IRQ_RETVAL(ret);
	} else
		dev_warn(chip->dev, "No interrupts(0x%x)\n", reg_val);

	return IRQ_HANDLED;
}

static int debugfs_voltage_get(void *data, u64 *val)
{
	int sel, ret;
	struct regulator_dev *rdev = data;

	sel = rdev->desc->ops->get_voltage_sel(rdev);
	if (sel < 0)
		return sel;
	ret = rdev->desc->ops->list_voltage(rdev, sel);

	*val = ret / 1000;

	return 0;
}

static int debugfs_voltage_set(void *data, u64 val)
{
	int selector;
	struct regulator_dev *rdev = data;

	val = val * 1000;
	selector = regulator_map_voltage_linear(rdev,
						val - rdev->desc->uV_step / 2,
						val + rdev->desc->uV_step / 2);

	return rdev->desc->ops->set_voltage_sel(rdev, selector);
}

DEFINE_SIMPLE_ATTRIBUTE(fops_ldo,
			debugfs_voltage_get, debugfs_voltage_set, "%llu\n");

static void sc2703_debugfs_init(struct regulator_dev *rdev)
{

	regu_debugfs = debugfs_create_dir(rdev->desc->name, NULL);
	if (IS_ERR_OR_NULL(regu_debugfs)) {
		dev_warn(&rdev->dev, "Failed to create (%s) debugfs directory\n",
			rdev->desc->name);
		rdev->debugfs = NULL;
		return;
	}

	debugfs_create_file("voltage", S_IRUGO | S_IWUSR,
			    regu_debugfs, rdev, &fops_ldo);

}

/*
 * I2C driver interface functions
 */
static int sc2703_buck_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	struct sc2703_buck *chip;
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	int ret;

	chip = devm_kzalloc(&i2c->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &i2c->dev;
	chip->regmap = devm_regmap_init_i2c(i2c, &sc2703_buck_regmap_config);
	if (IS_ERR(chip->regmap)) {
		ret = PTR_ERR(chip->regmap);
		dev_err(chip->dev,
			"Failed to allocate register map: %d\n", ret);
		return ret;
	}

	i2c_set_clientdata(i2c, chip);
	chip->pdata = i2c->dev.platform_data;
	config.dev = &i2c->dev;
	config.regmap = chip->regmap;
	ret = sc2703_regulators_parse_dt(chip, &config);
	if (ret < 0)
		return ret;
	ret = sc2703_buck_init(chip);
	if (ret < 0)
		return ret;

	if (chip->irq == 0) {
		dev_warn(chip->dev, "No IRQ configured\n");
	} else {
		ret = regmap_write(chip->regmap,
			SC2703_COREBUCK_IRQ_MASK,
			SC2703_M_IRQ_ALL);
		if (ret < 0)
			return ret;

		ret = devm_request_threaded_irq(chip->dev, chip->irq, NULL,
				sc2703_buck_irq_handler,
				SC2703_IRQ_FLAGS,
				"sc2703-buck", chip);
		if (ret != 0) {
			dev_err(chip->dev, "Failed to request IRQ: %d\n",
				i2c->irq);
			return ret;
		}

		ret = regmap_update_bits(chip->regmap,
			SC2703_COREBUCK_IRQ_MASK,
			SC2703_M_IRQ_MAKS, 0);
		if (ret < 0)
			return ret;
	}

	rdev = devm_regulator_register(&i2c->dev, &sc2703_buck_reg, &config);
	if (IS_ERR(rdev)) {
		dev_err(&i2c->dev, "Failed to register sc2703-regulator\n");
		return PTR_ERR(rdev);
	}
	sc2703_debugfs_init(rdev);
	chip->rdev = rdev;
	return 0;
}

static int sc2703_buck_remove(struct i2c_client *client)
{
	debugfs_remove_recursive(regu_debugfs);
	return 0;
}

static const struct i2c_device_id sc2703_buck_i2c_id[] = {
	{"sc2703-buck",},
	{ },
};
MODULE_DEVICE_TABLE(i2c, sc2703_buck_i2c_id);

static const struct of_device_id sc2703_buck_dt_id[] = {
	{.compatible = "sprd,sc2703-buck",},
	{ },
};

static struct i2c_driver sc2703_regulator_driver = {
	.driver = {
		.name = "sc2703-buck",
		.owner = THIS_MODULE,
		.of_match_table = sc2703_buck_dt_id,
	},
	.probe = sc2703_buck_probe,
	.remove = sc2703_buck_remove,
	.id_table = sc2703_buck_i2c_id,
};

module_i2c_driver(sc2703_regulator_driver);

MODULE_AUTHOR("Roy Im <Roy.Im.opensource@diasemi.com>");
MODULE_DESCRIPTION("Regulator device driver for Powerventure SC2703");
MODULE_LICENSE("GPL V2");

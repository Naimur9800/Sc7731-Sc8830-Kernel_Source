/*
 * Copyright (C) 2016 Spreadtrum Communications Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Fixes:
 *	0.2 unprotect dcdc/ldo before turn on and off
 *		remove dcdc/ldo calibration
 * To Fix:
 *
 *
 */
#include <linux/init.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/spinlock.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/regulator/of_regulator.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/i2c.h>

#undef debug
#define debug(format, arg...) pr_info("regu: ""@@@%s: " format, __func__, ##arg)
#define FAIRCHILD_DCDC_NAME			"fairchild_fan53555"
#define DEF_VOL 1020000

enum fan53555_regs {
	FAN53555_REG_VSEL_0 = 0x0,
	FAN53555_REG_VSEL_1 = 0x1,
	FAN53555_REG_CONTROL = 0x2,
	FAN53555_REG_ID1 = 0x3,
	FAN53555_REG_ID2 = 0x4,
	FAN53555_REG_MONITOR = 0x5,
	FAN53555_REG_MAX_NUM = FAN53555_REG_MONITOR
};

struct fan5xx_regs {
	int opt;
	u8 vsel;
	u8 vsel_msk;
	u8 enable_reg;
	u8 enable_msk;
	u32 min_uV, step_uV;
	u32 vol_def;
};

struct fan5xx_regulator_info {
	struct regulator_desc desc;
	struct regulator_init_data *init_data;
	struct fan5xx_regs regs;
	struct i2c_client *client;
	int enable;
};
static struct dentry *debugfs_root;
static struct regulator_dev *rdev_glb;
static atomic_t idx = ATOMIC_INIT(1);	/* 0: dummy */
static int fan5xx_write_reg(struct fan5xx_regulator_info *fan5xx_dcdc, u8 addr,
			    u8 para)
{
	struct i2c_msg msgs[1];
	u8 buf[2] = { 0 };
	int ret = -1, msg_len = 0;

	buf[0] = addr;
	buf[1] = para;

	msgs[0].addr = fan5xx_dcdc->client->addr;
	msgs[0].flags = 0;
	msgs[0].buf = buf;
	msgs[0].len = ARRAY_SIZE(buf);

	msg_len = ARRAY_SIZE(msgs);

	ret = i2c_transfer(fan5xx_dcdc->client->adapter, msgs, msg_len);
	if (ret != 1) {
		pr_err("%s i2c write error, ret %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int fan5xx_read_reg(struct fan5xx_regulator_info *fan5xx_dcdc,
			   u8 reg_addr, u8 *pdata)
{
	int ret = 0, msg_len = 0;
	u8 *write_buf = &reg_addr;
	u8 *read_buf = pdata;

	struct i2c_msg msgs[] = {
		{
		 .addr = fan5xx_dcdc->client->addr,
		 .flags = 0,	/*i2c write*/
		 .len = 1,
		 .buf = write_buf,
		 },
		{
		 .addr = fan5xx_dcdc->client->addr,
		 .flags = I2C_M_RD, /*i2c read*/
		 .len = 1,
		 .buf = read_buf,
		 },
	};
	msg_len = ARRAY_SIZE(msgs);

	ret = i2c_transfer(fan5xx_dcdc->client->adapter, msgs, msg_len);
	if (ret != msg_len) {
		pr_err("%s i2c read error, ret %d, msg_len %d\n", __func__, ret,
		       msg_len);
	}

	return ret;
}

static int fan5xx_dcdc_turn_on(struct regulator_dev *rdev)
{
	struct fan5xx_regulator_info *fan5xx_dcdc = rdev_get_drvdata(rdev);
	struct fan5xx_regs *regs = &fan5xx_dcdc->regs;
	int ret = -EINVAL;
	u8 reg_addr = (u8) (regs->enable_reg);
	u8 reg_val = 0;

	ret = fan5xx_read_reg(fan5xx_dcdc, reg_addr, &reg_val);
	if (ret <= 0) {
		debug("read reg(%#x) data error! ret = %d\n", reg_addr, ret);
		return ret;
	}

	reg_val |= regs->enable_msk;

	ret = fan5xx_write_reg(fan5xx_dcdc, reg_addr, reg_val);
	if (ret) {
		debug("write reg(%#x) data(%#x) error! ret = %d\n", reg_addr,
		      reg_val, ret);
		return ret;
	}

	return 0;
}

static int fan5xx_dcdc_turn_off(struct regulator_dev *rdev)
{
	struct fan5xx_regulator_info *fan5xx_dcdc = rdev_get_drvdata(rdev);
	struct fan5xx_regs *regs = &fan5xx_dcdc->regs;
	int ret = -EINVAL;
	u8 reg_addr = (u8) (regs->enable_reg);
	u8 reg_val = 0;

	ret = fan5xx_read_reg(fan5xx_dcdc, reg_addr, &reg_val);
	if (ret <= 0) {
		debug("read reg(%#x) data error! ret = %d\n", reg_addr, ret);
		return ret;
	}

	reg_val &= ~regs->enable_msk;

	ret = fan5xx_write_reg(fan5xx_dcdc, reg_addr, reg_val);
	if (ret) {
		debug("write reg(%#x) data(%#x) error! ret = %d\n", reg_addr,
		      reg_val, ret);
		return ret;
	}

	return 0;
}

static int fan5xx_dcdc_is_on(struct regulator_dev *rdev)
{
	struct fan5xx_regulator_info *fan5xx_dcdc = rdev_get_drvdata(rdev);
	struct fan5xx_regs *regs = &fan5xx_dcdc->regs;
	int ret = -EINVAL;
	u8 reg_addr = (u8) (regs->enable_reg);
	u8 reg_val = 0, shft = __ffs(regs->enable_msk);

	ret = fan5xx_read_reg(fan5xx_dcdc, reg_addr, &reg_val);
	if (ret <= 0) {
		debug("read reg(%#x) data error! ret = %d\n", reg_addr, ret);
		return ret;
	}

	ret = (reg_val & regs->enable_msk) >> shft;

	return ret;
}

static int fan5xx_dcdc_get_voltage(struct regulator_dev *rdev)
{
	struct fan5xx_regulator_info *fan5xx_dcdc = rdev_get_drvdata(rdev);
	struct fan5xx_regs *regs = &fan5xx_dcdc->regs;
	int vol_uV = 0, ret = -EINVAL;
	u8 reg_addr = (u8) (regs->vsel);
	u8 reg_val = 0, shft = __ffs(regs->vsel_msk);

	ret = fan5xx_read_reg(fan5xx_dcdc, reg_addr, &reg_val);
	if (ret <= 0) {
		debug("read reg(%#x) data error! ret = %d\n", reg_addr, ret);
		return ret;
	}

	reg_val = (reg_val & regs->vsel_msk) >> shft;
	vol_uV = regs->min_uV + reg_val * regs->step_uV;

	return vol_uV;
}

static int fan5xx_dcdc_set_voltage(struct regulator_dev *rdev, int min_uV,
				   int max_uV, unsigned *selector)
{
	struct fan5xx_regulator_info *fan5xx_dcdc = rdev_get_drvdata(rdev);
	struct fan5xx_regs *regs = &fan5xx_dcdc->regs;
	int uV = min_uV, ret = -EINVAL;
	u8 reg_addr = (u8) (regs->vsel);
	u8 reg_val = 0, shft = __ffs(regs->vsel_msk);
	int old_vol = rdev->desc->ops->get_voltage(rdev);

	if (uV < regs->min_uV)
		return -EINVAL;

	ret = fan5xx_read_reg(fan5xx_dcdc, reg_addr, &reg_val);
	if (ret <= 0) {
		debug("read reg(%#x) data error! ret = %d\n", reg_addr, ret);
		return ret;
	}

	uV = (int)(uV - (int)regs->min_uV) / regs->step_uV;
	if (uV > (regs->vsel_msk >> shft))
		uV = regs->vsel_msk >> shft;

	reg_val &= ~regs->vsel_msk;
	reg_val |= (uV << shft);
	ret = fan5xx_write_reg(fan5xx_dcdc, reg_addr, reg_val);
	if (ret) {
		debug("write reg(%#x) data(%#x) error! ret = %d\n", reg_addr,
		      reg_val, ret);
		return ret;
	}

	/* dcdc boost delay */
	if (min_uV > old_vol) {
		int dly = 0;
		int slew_rate = 0;

		ret =
		    fan5xx_read_reg(fan5xx_dcdc, FAN53555_REG_CONTROL,
				    &reg_val);
		if (ret <= 0) {
			debug("read reg(0x2) data error! ret = %d\n", ret);
			return ret;
		}
		reg_val = (reg_val & 0x70) >> 4;
		slew_rate = ((64 * 1000) << reg_val);
		dly = (min_uV - old_vol) / slew_rate;
		WARN_ON(dly > 1000);
		udelay(dly);
	}

	return 0;
}

static struct regulator_ops fan5xx_dcdc_ops = {
	.enable = fan5xx_dcdc_turn_on,
	.disable = fan5xx_dcdc_turn_off,
	.is_enabled = fan5xx_dcdc_is_on,
	.set_voltage = fan5xx_dcdc_set_voltage,
	.get_voltage = fan5xx_dcdc_get_voltage,
};

static int debugfs_voltage_get(void *data, u64 *val)
{
	struct regulator_dev *rdev = data;

	if (rdev) {
		if (rdev->desc->ops->get_voltage)
			*val = rdev->desc->ops->get_voltage(rdev)/1000;
		else
			*val = ~0ULL;
	}
	return 0;
}

static int debugfs_enable_get(void *data, u64 *val)
{
	struct regulator_dev *rdev = data;

	if (rdev && rdev->desc->ops->is_enabled)
		*val = rdev->desc->ops->is_enabled(rdev);
	else
		*val = ~0ULL;
	return 0;
}

static int debugfs_enable_set(void *data, u64 val)
{
	struct regulator_dev *rdev = data;

	if (rdev && rdev->desc->ops->enable)
		(val) ? rdev->desc->ops->enable(rdev)
		    : rdev->desc->ops->disable(rdev);
	return 0;
}

static int debugfs_voltage_set(void *data, u64 val)
{
	struct regulator_dev *rdev = data;
	u32 min_uV;

	if (rdev && rdev->desc->ops->set_voltage) {
		if (strcmp(rdev->desc->name, "fan53555") != 0)
			min_uV = (u32) val*1000;
		else
			min_uV = (u32) val*1000;
		rdev->desc->ops->set_voltage(rdev, min_uV, min_uV, 0);
	}
	return 0;
}

static int debugfs_cal_get(void *data, u64 *val)
{
	*val = 1;

	return 0;
}

static int debugfs_cal_set(void *data, u64 val)
{
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_enable,
			debugfs_enable_get, debugfs_enable_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_ldo,
			debugfs_voltage_get, debugfs_voltage_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_cal,
			debugfs_cal_get, debugfs_cal_set, "%llu\n");

static void ext_dcdc_init_debugfs(struct regulator_dev *rdev)
{
	struct dentry *regu_debugfs = NULL;

	regu_debugfs = debugfs_create_dir(rdev->desc->name, debugfs_root);
	if (IS_ERR_OR_NULL(regu_debugfs)) {
		pr_warn("Failed to create (%s) debugfs directory\n",
			rdev->desc->name);
		rdev->debugfs = NULL;
		return;
	}

	debugfs_create_file("enable", S_IRUGO | S_IWUSR,
			    regu_debugfs, rdev, &fops_enable);

	debugfs_create_file("voltage", S_IRUGO | S_IWUSR,
			    regu_debugfs, rdev, &fops_ldo);

	debugfs_create_file("calibrated", S_IRUGO | S_IWUSR,
			    regu_debugfs, rdev, &fops_cal);
}

static const unsigned short fan5xx_addr_list[] = {
	0x60,			/* 0xc0 >> 1 */
	I2C_CLIENT_END
};

static const struct i2c_device_id fan53555_i2c_id[] = {
	{FAIRCHILD_DCDC_NAME, 0},
	{}
};

static const struct of_device_id fan53555_of_match[] = {
	{.compatible = "fairchild,fairchild_fan53555",},
	{}
};

static struct regulator_consumer_supply *set_supply_map(struct device *dev,
							const char *supply_name,
							int *num)
{
	char **map = (char **)dev_get_platdata(dev);
	int i, n;
	struct regulator_consumer_supply *consumer_supplies = NULL;

	if (!supply_name || !(map && map[0]))
		return NULL;

	for (i = 0; map[i] || map[i + 1]; i++) {
		if (map[i] && 0 == strcmp(map[i], supply_name))
			break;
	}

	for (n = 0; map[i + n]; n++)
		;

	if (n) {
		debug("supply %s consumers %d - %d\n", supply_name, i, n);
		consumer_supplies =
		    kzalloc(n * sizeof(*consumer_supplies), GFP_KERNEL);
		if (!consumer_supplies) {
			dev_err(dev,
				"failed allocate memory for consumer_supplies");
			return NULL;
		}

		for (n = 0; map[i]; i++, n++)
			consumer_supplies[n].supply = map[i];

		if (num)
			*num = n;
	}
	return consumer_supplies;
}

static int fan53555_regulator_parse_dt(struct device *dev,
				       struct device_node *np,
				       struct fan5xx_regulator_info *desc,
				       struct regulator_consumer_supply *supply,
				       int sz)
{
	/*struct sci_regulator_regs *regs = &desc->regs; */
	struct fan5xx_regs *regs = &desc->regs;
	u32 tmp_val_u32;
	int type = 0, ret = 0;

	if (!dev || !np || !desc || !supply)
		return -EINVAL;

	desc->desc.name = np->name;
	desc->desc.id = (atomic_inc_return(&idx) - 1);
	desc->desc.type = REGULATOR_VOLTAGE;
	desc->desc.owner = THIS_MODULE;

	supply[0].dev_name = NULL;
	supply[0].supply = np->name;
	desc->init_data = of_get_regulator_init_data(dev, np,
						     &desc->desc);

	desc->init_data->supply_regulator = 0;
	desc->init_data->constraints.valid_modes_mask =
	    REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY;
	desc->init_data->constraints.valid_ops_mask =
	    REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS |
	    REGULATOR_CHANGE_VOLTAGE;
	desc->init_data->num_consumer_supplies = sz;

	desc->init_data->consumer_supplies =
	    set_supply_map(dev, desc->desc.name,
			   &desc->init_data->num_consumer_supplies);

	if (!desc->init_data->consumer_supplies)
		desc->init_data->consumer_supplies = supply;

	/* Fill struct sci_regulator_regs variable desc->regs */

	regs->min_uV = desc->init_data->constraints.min_uV;

	ret =
	    of_property_read_u32(np, "regulator-step-microvolt", &tmp_val_u32);
	if (!ret)
		regs->step_uV = tmp_val_u32;

	ret =
	    of_property_read_u32(np, "sprd,default-microvolt",
				 &tmp_val_u32);
	if (!ret)
		regs->vol_def = tmp_val_u32;
	debug("[%d] %s type %d, range %d(uV) - %d(uV), step %d(uV)\n",
		(idx.counter - 1), np->name, type,
		desc->init_data->constraints.min_uV,
		desc->init_data->constraints.max_uV, regs->step_uV);

	desc->enable = of_get_named_gpio(np, "enable-gpios", 0);

	return 0;
}

static int fan53335_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int ret = 0;
	u8 chipid = 0;

	struct fan5xx_regulator_info *fan5xx_dcdc = NULL;
	struct regulator_dev *rdev = NULL;
	struct regulator_consumer_supply consumer_supplies_default[1] = { };
	struct regulator_config config = { };
	struct device *dev = &client->dev;
	struct device_node *dev_np = dev->of_node;
	struct device_node *child_np;

	pr_info("%s -- enter probe -->\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	fan5xx_dcdc = devm_kzalloc(dev, sizeof(*fan5xx_dcdc), GFP_KERNEL);
	if (!fan5xx_dcdc)
		return -ENOMEM;

	for_each_child_of_node(dev_np, child_np) {
		ret =
		    fan53555_regulator_parse_dt(dev, child_np, fan5xx_dcdc,
						consumer_supplies_default,
						ARRAY_SIZE
						(consumer_supplies_default));
	}

	fan5xx_dcdc->client = client;
	i2c_set_clientdata(client, fan5xx_dcdc);
	fan5xx_dcdc->regs.opt = 18;	/* FAN53555 BUC 18x */
	fan5xx_dcdc->regs.vsel = FAN53555_REG_VSEL_0;
	fan5xx_dcdc->regs.vsel_msk = 0x3F;
	fan5xx_dcdc->regs.enable_reg = FAN53555_REG_VSEL_0;
	fan5xx_dcdc->regs.enable_msk = 0x80;

	/*
	 *enable fan53555 via gpio
	 */
	if (!gpio_is_valid(fan5xx_dcdc->enable)) {
		dev_err(dev, "invalid gpio:%d\n", fan5xx_dcdc->enable);
		return -EINVAL;
	}

	ret = gpio_request(fan5xx_dcdc->enable, "fan53555_enable");
	if (ret < 0) {
		dev_err(dev, "Fail gpio_request gpio fan53555_enable\n");
		return ret;
	}

	ret = gpio_direction_output(fan5xx_dcdc->enable, 1);
	if (ret) {
		dev_err(dev, "Fail gpio_direction gpio fan53555_enable\n");
		return ret;
	}

	/*
	 *read fan53555 chipid chipid = 0x88
	 */
	ret = fan5xx_read_reg(fan5xx_dcdc, FAN53555_REG_ID1, &chipid);

	if (ret > 0)
		pr_info("%s fan53555 chipid %#x\n", __func__, chipid);
	else
		pr_err("%s fan53555 read chipid error!\n", __func__);

	fan5xx_dcdc->desc.type = REGULATOR_VOLTAGE;
	fan5xx_dcdc->desc.ops = &fan5xx_dcdc_ops;
	config.dev = &client->dev;
	config.init_data = fan5xx_dcdc->init_data;
	config.driver_data = fan5xx_dcdc;
	config.of_node = NULL;

	rdev = regulator_register(&fan5xx_dcdc->desc, &config);
	if (!IS_ERR_OR_NULL(rdev)) {
		pr_info("%s regulator fan53555 ok!\n", __func__);
		rdev->reg_data = fan5xx_dcdc;
		ext_dcdc_init_debugfs(rdev);
		rdev_glb = rdev;
		fan5xx_dcdc->desc.ops->set_voltage(rdev,
						   fan5xx_dcdc->regs.vol_def,
						   fan5xx_dcdc->regs.vol_def,
						   0);
	}

	pr_info("%s -- exit probe -->\n", __func__);

	return 0;
}

static int fan53335_remove(struct i2c_client *client)
{
	return 0;
}

static struct i2c_driver fan5xx_i2c_driver = {
	.probe = fan53335_probe,
	.remove = fan53335_remove,
	.id_table = fan53555_i2c_id,
	.driver = {
		   .name = FAIRCHILD_DCDC_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = fan53555_of_match,
		   },
};

static int __init fan53555_regulator_init(void)
{
	return i2c_add_driver(&fan5xx_i2c_driver);
}

static void __exit fan53555_regulator_exit(void)
{
	i2c_del_driver(&fan5xx_i2c_driver);
}

subsys_initcall(fan53555_regulator_init);
module_exit(fan53555_regulator_exit);
MODULE_DESCRIPTION("fairchild fan53555 regulator driver");
MODULE_LICENSE("GPL");

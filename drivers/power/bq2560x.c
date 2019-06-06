/* SPDX-License-Identifier: GPL-2.0
 *
 * Charger device driver for BQ2560X
 *
 * Copyright (c) 2018 UNISOC.
 */

#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include "sprd_charge_helper.h"
#include "bq2560x.h"

static struct bq2560x *bq2560x_data;
static bool otg_enable_flag;

static int bq2560x_write_reg(u8 reg, u8 val)
{
	int ret;

	dev_dbg(&bq2560x_data->client->dev, "writer reg=0x%x, val=0x%x\n",
			 reg, val);
	ret = i2c_smbus_write_byte_data(bq2560x_data->client, reg, val);
	if (ret < 0)
		dev_err(&bq2560x_data->client->dev, "error=%d\n", ret);

	return ret;
}

static int bq2560x_read_reg(u8 reg, u8 *dest)
{
	int ret;

	ret = i2c_smbus_read_byte_data(bq2560x_data->client, reg);
	if (ret < 0) {
		dev_err(&bq2560x_data->client->dev,
			"reg=0x%x, ret=%d\n", reg, ret);
		return ret;
	}

	*dest = ret & 0xff;
	return 0;
}

static void bq2560x_set_value(u8 reg, u8 reg_bit,
			      u8 reg_shift, u8 val)
{
	u8 tmp;

	bq2560x_read_reg(reg, &tmp);
	tmp = (tmp & (~reg_bit)) | (val << reg_shift);
	bq2560x_write_reg(reg, tmp);
}

void bq2560x_reset_timer(void)
{
	bq2560x_set_value(BQ2560X_REG_01, REG01_WDT_RESET_MASK,
			  REG01_WDT_RESET_SHIFT, REG01_WDT_RESET);
}

void bq2560x_sw_reset(void)
{
	bq2560x_set_value(BQ2560X_REG_0B, REG0B_RESET_MASK,
			  REG0B_RESET_SHIFT, REG0B_RESET);
}

void bq2560x_set_vindpm(u8 reg_val)
{
	bq2560x_set_value(BQ2560X_REG_06, REG06_VINDPM_MASK,
			  REG06_VINDPM_SHIFT, reg_val);
}

void bq2560x_termina_cur_set(u8 reg_val)
{
	bq2560x_set_value(BQ2560X_REG_03, REG03_ITERM_MASK,
			  REG03_ITERM_SHIFT, reg_val);
}

void bq2560x_termina_vol_set(u8 reg_val)
{
	bq2560x_set_value(BQ2560X_REG_04, REG04_VREG_MASK,
			  REG04_VREG_SHIFT, reg_val);
}

void bq2560x_init(void)
{
	bq2560x_sw_reset();
}

void bq2560x_otg_enable(int enable)
{
	if (enable) {
		otg_enable_flag = true;
		bq2560x_set_value(BQ2560X_REG_01, REG01_OTG_CONFIG_MASK,
				  REG01_OTG_CONFIG_SHIFT, REG01_OTG_ENABLE);
		schedule_delayed_work(&bq2560x_data->feed_watchdog_work,
				      msecs_to_jiffies(50));
		schedule_delayed_work(&bq2560x_data->vbus_detect_work,
				      msecs_to_jiffies(500));
	} else {
		otg_enable_flag = false;
		cancel_delayed_work_sync(&bq2560x_data->feed_watchdog_work);
		cancel_delayed_work_sync(&bq2560x_data->vbus_detect_work);
		bq2560x_set_value(BQ2560X_REG_01, REG01_OTG_CONFIG_MASK,
				  REG01_OTG_CONFIG_SHIFT, REG01_OTG_DISABLE);
	}
}

void bq2560x_stop_charging(u32 flag)
{

	bq2560x_set_value(BQ2560X_REG_01, REG01_CHG_CONFIG_MASK,
			  REG01_CHG_CONFIG_SHIFT, REG01_CHG_DISABLE);
}

void bq2560x_enable_chg(void)
{
	bq2560x_set_value(BQ2560X_REG_01, REG01_CHG_CONFIG_MASK,
			  REG01_CHG_CONFIG_SHIFT, REG01_CHG_ENABLE);
}

u8 bq2560x_get_fault_val(void)
{
	u8 data;

	bq2560x_read_reg(BQ2560X_REG_09, &data);
	return data;
}

void bq2560x_set_chg_current_limit(u32 limit)
{
	bq2560x_set_value(BQ2560X_REG_00, REG00_IINLIM_MASK,
			  REG00_IINLIM_SHIFT, limit);
}

void bq2560x_set_chg_current(u8 reg_val)
{
	bq2560x_set_value(BQ2560X_REG_02, REG02_ICHG_MASK,
			  REG02_ICHG_SHIFT, reg_val);
}

void bq2560x_set_ship_mode(void)
{
	bq2560x_set_value(BQ2560X_REG_05, REG05_WDT_MASK,
			REG05_WDT_SHIFT, REG05_WDT_DISABLE);

	bq2560x_set_value(BQ2560X_REG_07, REG07_BATFET_DIS_MASK,
			REG07_BATFET_DIS_SHIFT, REG07_BATFET_OFF);
}

int bq2560x_get_ship_mode(void)
{
	u8 data = 0;

	bq2560x_read_reg(BQ2560X_REG_07, &data);
	data = (data & 0x20) >> REG07_BATFET_DIS_SHIFT;

	return !data;
}

int bq2560x_get_charge_status(void)
{
	u8 data = 0;

	bq2560x_read_reg(BQ2560X_REG_01, &data);
	data = (data & 0x10) >> REG01_CHG_CONFIG_SHIFT;

	return data;
}

u8 bq2560x_get_chgcur(void)
{
	u8 data;

	bq2560x_read_reg(BQ2560X_REG_02, &data);
	data = (data & REG02_ICHG_MASK) >> REG02_ICHG_SHIFT;

	return data;
}

u8 bq2560x_get_sys_status(void)
{
	u8 data;

	bq2560x_read_reg(BQ2560X_REG_08, &data);
	data = (data & REG08_CHRG_STAT_MASK) >> REG08_CHRG_STAT_SHIFT;

	return data;
}

static void bq2560x_15s_work(struct work_struct *work)
{
	bq2560x_reset_timer();
	schedule_delayed_work(&bq2560x_data->feed_watchdog_work, HZ * 15);
}

static void bq2560x_vbus_det_work(struct work_struct *work)
{
	if (!bq2560x_data->vbus_detect) {
		if (!gpiod_get_value(bq2560x_data->vbus_detect) &&
			otg_enable_flag == true)
			bq2560x_set_value(BQ2560X_REG_01, REG01_OTG_CONFIG_MASK,
					  REG01_OTG_CONFIG_SHIFT,
					  REG01_OTG_ENABLE);

		schedule_delayed_work(&bq2560x_data->vbus_detect_work,
				      msecs_to_jiffies(1500));
	}
}

static ssize_t set_regs_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	unsigned long set_value;
	int reg, val, ret;

	if (kstrtoul(buf, 16, &set_value))
		return -EINVAL;

	reg = (set_value & 0xff00) >> 8;
	val = set_value & 0xff;
	dev_dbg(dev, "set reg=0x%x, value=%d\n", reg, val);

	ret = bq2560x_write_reg(reg, val);
	if (ret < 0) {
		dev_err(dev, "set_regs_store error\n");
		return ret;
	}

	return count;
}
static DEVICE_ATTR_WO(set_regs);

static ssize_t dump_regs_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	const int regaddrs[] = {0x00, 0x01, 0x02, 0x03, 0x4,
		0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b};
	const char str[] = "0123456789abcdef";
	u8 bq2560x_regs[0x60];
	int i, index;
	char val = 0;

	for (i = 0; i < 0x60; i++) {
		if ((i % 3) == 2)
			buf[i] = ' ';
		else
			buf[i] = 'x';
	}
	buf[0x5d] = '\n';
	buf[0x5e] = 0;
	buf[0x5f] = 0;

	for (i = 0; i <= 12; i++)
		bq2560x_read_reg(i, &bq2560x_regs[i]);

	for (i = 0; i < ARRAY_SIZE(regaddrs); i++) {
		index = regaddrs[i];
		val = bq2560x_regs[index];
		buf[3 * index] = str[(val & 0xf0) >> 4];
		buf[3 * index + 1] = str[val & 0x0f];
	}

	return 0x60;
}
static DEVICE_ATTR_RO(dump_regs);

static struct attribute *bq2560x_class_attrs[] = {
	&dev_attr_dump_regs.attr,
	&dev_attr_set_regs.attr,
	NULL,
};
ATTRIBUTE_GROUPS(bq2560x_class);

static int bq2560x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{

	struct sprd_ext_ic_operations *bq2560x_ops = NULL;
	int ret;

	bq2560x_data = devm_kzalloc(&client->dev, sizeof(struct bq2560x),
				    GFP_KERNEL);
	if (!bq2560x_data)
		return -ENOMEM;
	bq2560x_data->client = client;

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "i2c check functionality error.\n");
		return -ENODEV;
	}

	bq2560x_data->vbus_detect =
			devm_gpiod_get(&client->dev, "vbus-det", 0);
	if (IS_ERR(bq2560x_data->vbus_detect)) {
		dev_err(&client->dev, "unable to claim vbus-det-gpios\n");
		return PTR_ERR(bq2560x_data->vbus_detect);
	}

	INIT_DELAYED_WORK(&bq2560x_data->feed_watchdog_work, bq2560x_15s_work);
	INIT_DELAYED_WORK(&bq2560x_data->vbus_detect_work,
			  bq2560x_vbus_det_work);

	ret = sysfs_create_group(&client->dev.kobj, bq2560x_class_groups[0]);
	if (ret)
		dev_warn(&client->dev, "failed to create bq2560x sysfs attr\n");

	bq2560x_ops = sprd_get_bq2560x_ops();
	if (!bq2560x_ops)
		return -EINVAL;

	sprdbat_register_ext_ops(bq2560x_ops);

	return 0;
}

static int bq2560x_remove(struct i2c_client *client)
{
	flush_scheduled_work();
	sysfs_remove_group(&client->dev.kobj, bq2560x_class_groups[0]);
	return 0;
}

static const struct i2c_device_id bq2560x_i2c_id[] = {
	{"bq2560x_chg", 0},
	{}
};

static const struct of_device_id bq2560x_of_match[] = {
	{.compatible = "ti,bq2560x_chg",},
	{ }
};

static struct i2c_driver bq2560x_i2c_driver = {
	.driver = {
		   .name = "bq2560x_chg",
		   .owner = THIS_MODULE,
		   .of_match_table = bq2560x_of_match,
		   },
	.probe = bq2560x_probe,
	.remove = bq2560x_remove,
	.id_table = bq2560x_i2c_id,
};

static int bq2560x_i2c_init(void)
{
	return i2c_add_driver(&bq2560x_i2c_driver);
}

static void bq2560x_i2c_exit(void)
{
	i2c_del_driver(&bq2560x_i2c_driver);
}

subsys_initcall_sync(bq2560x_i2c_init);
module_exit(bq2560x_i2c_exit);

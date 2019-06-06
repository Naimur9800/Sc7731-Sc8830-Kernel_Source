#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/timer.h>
#include <linux/param.h>
#include <linux/stat.h>
#include <linux/irq.h>
#include <linux/of_gpio.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include "sprd_2700.h"
#include "sprd_charge_helper.h"

#define SPRD_EX_DEBUG(format, arg...) pr_info("sprd 2700: "format, ## arg)

static struct i2c_client *this_client;

static int sprd_2700_write_reg(int reg, u8 val)
{
	int ret;

	SPRD_EX_DEBUG("#### writereg reg = %d val = %d\n", reg,	val);
	ret = i2c_smbus_write_byte_data(this_client, reg, val);
	if (ret < 0)
		SPRD_EX_DEBUG("%s: error = %d", __func__, ret);
	return ret;
}

static int sprd_2700_read_reg(int reg, u8 *dest)
{
	int ret;

	ret = i2c_smbus_read_byte_data(this_client, reg);
	if (ret < 0) {
		SPRD_EX_DEBUG("%s reg(0x%x), ret(%d)\n", __func__, reg, ret);
		return ret;
	}
	ret &= 0xff;
	*dest = (u8)(ret & 0xff);
	SPRD_EX_DEBUG("%s reg(0x%x), ret(%d)\n", __func__, reg, ret);
	return 0;
}

static void sprd_2700_set_value(BYTE reg, BYTE reg_bit,
	BYTE reg_shift, BYTE val)
{
	BYTE tmp = 0x0;

	sprd_2700_read_reg(reg, &tmp);
	tmp = (tmp & (~reg_bit)) | (val << reg_shift);
	SPRD_EX_DEBUG("continue test tmp =0x%x,val=0x%x\n", tmp, val);
	sprd_2700_write_reg(reg, tmp);
}

static BYTE sprd_2700_get_value(BYTE reg, BYTE reg_bit, BYTE reg_shift)
{
	BYTE data = 0;
	int ret = 0;

	sprd_2700_read_reg(reg, &data);
	ret = (data & reg_bit) >> reg_shift;
	return ret;
}

void sprd_2700_otg_enable(int enable)
{
	SPRD_EX_DEBUG("%s enable =%d\n", __func__, enable);
	if (enable) {
		sprd_2700_set_value(POWER_ON_CTL, OTG_EN_BIT,
			CHG_OTG_EN_SHIFT, CHG_OTG_VAL);
	} else {
		sprd_2700_set_value(POWER_ON_CTL, OTG_EN_BIT,
			CHG_OTG_EN_SHIFT, CHG_OTG_DISABLE_VAL);
	}
}

void sprd_2700_start_chg(void)
{
	sprd_2700_set_value(POWER_ON_CTL, CHG_EN_BIT,
		CHG_EN_SHIFT, CHG_BAT_VAL);
}

void sprd_2700_stop_chg(void)
{
	sprd_2700_set_value(POWER_ON_CTL, CHG_EN_BIT,
		CHG_EN_SHIFT, CHG_DISABLE_VAL);
}

void sprd_2700_set_chg_current(unsigned int cur)
{
	BYTE reg_val = 0;

	if (cur > 100)
		reg_val = (cur / 100) - 1;
	else
		reg_val = 0;
	sprd_2700_set_value(CHG_CUR_CTL, CHG_CUR_BIT, CHG_CUR_SHIFT, reg_val);
}

unsigned int sprd_2700_get_chg_current(void)
{
	BYTE reg_val = 0;

	reg_val = sprd_2700_get_value(CHG_CUR_CTL, CHG_CUR_BIT, CHG_CUR_SHIFT);
	return (reg_val * 100) + 100;
}

BYTE sprd_2700_get_chg_status(void)
{
	BYTE data = 0;

	sprd_2700_read_reg(SYS_STATUS_REG, &data);
	return data;
}

BYTE sprd_2700_get_fault_val(void)
{
	BYTE data = 0;

	sprd_2700_read_reg(SYS_STATUS_REG, &data);
	return data;
}

void sprd_2700_termina_cur_set(unsigned int cur)
{
	BYTE reg_val = 0x0;

	SPRD_EX_DEBUG("%s cur =%d\n", __func__, cur);
	if (cur < 100)
		reg_val = 0x0;
	else if (cur >= 400)
		reg_val = 0x7;
	else
		reg_val = (cur - 50) / 50;
	sprd_2700_set_value(CHG_CUR_CTL, TERMINA_CUR_BIT,
		TERMINA_CUR_SHIFT, reg_val);
}

void sprd_2700_usb_aic_set(int usb_aic_vol)
{
	BYTE reg_val;

	SPRD_EX_DEBUG("%s sprdchg_usb_aic_set =%d\n", __func__, usb_aic_vol);
	if (usb_aic_vol < 4300)
		reg_val = 0x3;
	else if (usb_aic_vol < 4400)
		reg_val = 0x2;
	else
		reg_val = 0x1;
	SPRD_EX_DEBUG("%s reg_val=0x%x\n", __func__, reg_val);
	sprd_2700_set_value(CHG_VOL_CTL, USB_AIC_LIMIT_BIT,
		USB_AIC_LIMIT_SHIFT, reg_val);
}

void sprd_2700_sprdchg_cvl_set(unsigned int cvl_vol)
{
	unsigned char reg_val;

	SPRD_EX_DEBUG("%s cvl_vol =%d\n", __func__, cvl_vol);
	if (cvl_vol <= 4200)
		reg_val = 0x00;
	else if (cvl_vol >= 4500)
		reg_val = 0x06;
	else
		reg_val = (cvl_vol - 4200) / 50;
	SPRD_EX_DEBUG("%s reg_val=0x%x\n", __func__, reg_val);
	sprd_2700_set_value(CHG_VOL_CTL, USB_CVL_LIMIT_BIT,
		USB_CVL_LIMIT_SHIFT, reg_val);
}

void sprd_2700_reset_timer(void)
{
}

void sprd_2700_init(void)
{
	BYTE data = 0;
	int i = 0;

	SPRD_EX_DEBUG("sprd_2700_init\n");
	for (i = 0; i < 5; i++) {
		sprd_2700_read_reg(i, &data);
		SPRD_EX_DEBUG("sprd_2700_ReadReg i = %d, data = %x\n", i, data);
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
	SPRD_EX_DEBUG("set reg = %d value = %d\n", reg, val);
	ret = sprd_2700_write_reg(reg, val);
	if (ret < 0) {
		SPRD_EX_DEBUG("set_regs_store error\n");
		return -EINVAL;
	}
	return count;
}
static DEVICE_ATTR_WO(set_regs);

static ssize_t dump_regs_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	const int regaddrs[] = { 0x00, 0x01, 0x02, 0x03, 0x4};
	const char str[] = "0123456789abcdef";
	BYTE sprd_2700_regs[0x60];
	int i = 0, index;
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

	for (i = 0; i < 5; i++) {
		sprd_2700_read_reg(i, &sprd_2700_regs[i]);
		SPRD_EX_DEBUG("read reg i = %d sprd_2700_regs[i] = %x\n",
			i, sprd_2700_regs[i]);
	}

	for (i = 0; i < ARRAY_SIZE(regaddrs); i++) {
		index = regaddrs[i];
		val = sprd_2700_regs[index];
		buf[3 * index] = str[(val & 0xf0) >> 4];
		buf[3 * index + 1] = str[val & 0x0f];
		buf[3 * index + 1] = str[val & 0x0f];
	}
	return 0x60;
}
static DEVICE_ATTR_RO(dump_regs);

static struct attribute *sprd2700_class_attrs[] = {
	&dev_attr_dump_regs.attr,
	&dev_attr_set_regs.attr,
	NULL,
};
ATTRIBUTE_GROUPS(sprd2700_class);

static __used irqreturn_t sprd_2700_fault_handler(int irq, void *dev_id)
{
	struct sprd_2700 *p_sprd_2700 = (struct sprd_2700 *)dev_id;
	struct sprd_2700_platform_data *pdata =
		i2c_get_clientdata(p_sprd_2700->client);
	int value = 0;

	SPRD_EX_DEBUG("enter sprdbat_vbat_chg_fault\n");
	value = gpio_get_value(pdata->irq_gpio_number);
	SPRD_EX_DEBUG("gpio_number = %d,value=%d\n",
		pdata->irq_gpio_number, value);
	if (value) {
		SPRD_EX_DEBUG("gpio is high charge ic ok\n");
		irq_set_irq_type(p_sprd_2700->client->irq, IRQ_TYPE_LEVEL_LOW);
	} else {
		SPRD_EX_DEBUG("gpio is low some fault\n");
		irq_set_irq_type(p_sprd_2700->client->irq, IRQ_TYPE_LEVEL_HIGH);
	}
	schedule_work(&p_sprd_2700->chg_fault_irq_work);
	return IRQ_HANDLED;
}

#if 0
static void sprdbat_chg_fault_irq_works(struct work_struct *work)
{
	SPRD_EX_DEBUG("enter sprdbat_chg_fault_irq_works\n");
	/* TODO: maybe need add */
}
#endif

static int sprd_2700_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	int rc = 0;
	struct sprd_2700 *sprd_2700_data = NULL;
	struct sprd_2700_platform_data *pdata = NULL;
	struct device_node *np = client->dev.of_node;

	SPRD_EX_DEBUG("sprd_2700_probe_start\n");

	if (np && !pdata) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			SPRD_EX_DEBUG("%s alloc mem fail\n", __func__);
			return -ENOMEM;
		}
#if 0
		pdata->irq_gpio_number =
			of_get_named_gpio(np, "chg-fault-gpios", 0);
		if (pdata->irq_gpio_number < 0) {
			SPRD_EX_DEBUG("fail to get irq_gpio_number\n");
			return -ENODEV;
		}
#endif
	}
#if 0
	rc = devm_gpio_request(&client->dev,
			       pdata->irq_gpio_number, "sprd_2700_gpio");
	if (rc) {
		SPRD_EX_DEBUG("gpio_request failed!\n");
		return rc;
	}
	gpio_direction_input(pdata->irq_gpio_number);
	client->irq = gpio_to_irq(pdata->irq_gpio_number);
#endif
	sprd_2700_data = devm_kzalloc(&client->dev,
				      sizeof(struct sprd_2700), GFP_KERNEL);
	if (!sprd_2700_data) {
		SPRD_EX_DEBUG("devm kzalloc failed!\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, pdata);
	sprd_2700_data->client = client;
	this_client = client;

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_BYTE_DATA)) {
		SPRD_EX_DEBUG("%s: i2c check functionality error\n", __func__);
		return -ENODEV;
	}

	{
		BYTE data = 0;

		if (sprd_2700_read_reg(0, &data)) {
			SPRD_EX_DEBUG("%s: NO 2700 deivce!\n", __func__);
			return -ENXIO;
		}
	}

#if 0
	INIT_WORK(&sprd_2700_data->chg_fault_irq_work,
		  sprdbat_chg_fault_irq_works);
	if (client->irq > 0) {
		rc = devm_request_irq(&client->dev, client->irq,
				      sprd_2700_fault_handler,
				      IRQ_TYPE_LEVEL_LOW | IRQF_NO_SUSPEND,
				      "sprd_2700_chg_fault", sprd_2700_data);
		if (rc < 0)
			SPRD_EX_DEBUG("request_irq failed!\n");
	}
#endif
	rc = sysfs_create_group(&client->dev.kobj, sprd2700_class_groups[0]);
	if (rc)
		pr_err("failed to create 2700 sysfs device attributes\n");

	pdata->sprd_2700_ops = sprd_get_2700_ops();
	sprdbat_register_ext_ops(pdata->sprd_2700_ops);

	SPRD_EX_DEBUG("sprd_2700_probe_end\n");
	return 0;
}

static int sprd_2700_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, sprd2700_class_groups[0]);
	return 0;
}

#ifdef CONFIG_PM
static int sprd_2700_suspend(struct device *dev)
{
	return 0;
}

static int sprd_2700_resume(struct device *dev)
{
	return 0;
}
#endif

static UNIVERSAL_DEV_PM_OPS(sprd_2700_pm, sprd_2700_suspend,
	sprd_2700_resume, NULL);

static const struct i2c_device_id sprd_2700_i2c_id[] = {
	{"sc2700_chg", 0},
	{}
};

static const struct of_device_id sprd_2700_of_match[] = {
	{.compatible = "sprd,sc2700_chg",},
	{}
};

static struct i2c_driver sprd_2700_i2c_driver = {
	.driver = {
		   .name = "sc2700_chg",
		   .of_match_table = of_match_ptr(sprd_2700_of_match),
		   .pm = &sprd_2700_pm,
		   },
	.probe = sprd_2700_probe,
	.remove = sprd_2700_remove,
	.id_table = sprd_2700_i2c_id,
};

static int __init sprd_2700_i2c_init(void)
{
	return i2c_add_driver(&sprd_2700_i2c_driver);
}

static void __exit sprd_2700_i2c_exit(void)
{
	i2c_del_driver(&sprd_2700_i2c_driver);
}

subsys_initcall_sync(sprd_2700_i2c_init);
module_exit(sprd_2700_i2c_exit);

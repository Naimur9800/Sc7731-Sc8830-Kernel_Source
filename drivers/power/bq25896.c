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
#include "bq25896.h"
#include "sprd_charge_helper.h"

#define SPRD_EX_DEBUG(format, arg...) pr_info("sprd bq25896: "format, ## arg)

static struct i2c_client *this_client;

static int bq25896_write_reg(int reg, u8 val)
{
	int ret;

	SPRD_EX_DEBUG("#### writereg reg = %x val = 0x %x\n", reg, val);

	ret = i2c_smbus_write_byte_data(this_client, reg, val);
	if (ret < 0)
		SPRD_EX_DEBUG("%s: error = %d", __func__, ret);

	return ret;
}

int bq25896_read_reg(int reg, u8 *dest)
{
	int ret;

	ret = i2c_smbus_read_byte_data(this_client, reg);
	if (ret < 0) {
		SPRD_EX_DEBUG("%s reg(0x%x), ret(%d)\n", __func__, reg, ret);
		return ret;
	}

	*dest = ret & 0xff;
	return 0;
}

static void bq25896_set_value(unsigned char reg,
				unsigned char reg_bit, unsigned char reg_shift,
				unsigned char val)
{
	unsigned char tmp = 0;

	bq25896_read_reg(reg, &tmp);
	tmp = (tmp & (~reg_bit)) | (val << reg_shift);
	SPRD_EX_DEBUG("continue test tmp =0x%x,val=0x%x\n", tmp, val);
	bq25896_write_reg(reg, tmp);
}

static unsigned char bq25896_get_value(unsigned char reg,
					 unsigned char reg_bit,
					 unsigned char reg_shift)
{
	unsigned char data = 0;
	int ret = 0;

	bq25896_read_reg(reg, &data);
	ret = (data & reg_bit) >> reg_shift;

	return ret;
}

void bq25896_reset_timer(void)
{
	SPRD_EX_DEBUG("bq25896 reset timer 160s\n");
	bq25896_set_value(REG07, WATCHDOG_BIT,
			    WATCHDOG_SHIFT, 0x03);
	bq25896_set_value(REG03, WD_RST_BIT,
			    WD_RST_SHIFT, 0x01);
}

void bq25896_sw_reset(void)
{
	SPRD_EX_DEBUG("bq25896_sw_reset\n");
	bq25896_set_value(REG14, REG_RST_BIT, REG_RST_SHIFT, 1);
}

void bq25896_set_vindpm(unsigned char reg_val)
{
	bq25896_set_value(REG01, VINDPM_OS_BIT, VINDPM_OS_SHIFT, reg_val);
}

void bq25896_termina_cur_set(unsigned char reg_val)
{
	bq25896_set_value(REG05, ITERM_BIT,
			    ITERM_SHIFT, reg_val);
}

void bq25896_termina_vol_set(unsigned char reg_val)
{
	bq25896_set_value(REG06, VREG_BIT,
			    VREG_SHIFT, reg_val);
}

void bq25896_termina_time_set(unsigned char reg_val)
{
	bq25896_set_value(REG07, CHG_TIMER_BIT,
			    CHG_TIMER_SHIFT, reg_val);
}

void bq25896_init(void)
{
	unsigned char data = 0;
	int i = 0;

	SPRD_EX_DEBUG("bq25896_init\n");
	bq25896_sw_reset();
	for (i = 0; i <= 0x14; i++) {
		bq25896_read_reg(i, &data);
		SPRD_EX_DEBUG("bq25896_ReadReg i = %x, data = 0x%x\n", i, data);
	}

	bq25896_set_value(REG07, WATCHDOG_BIT,
			    WATCHDOG_SHIFT, 0);
	bq25896_set_value(REG07, EN_TERM_BIT,
			    EN_TERM_SHIFT, 0);
	bq25896_set_value(REG07, EN_TIMER_BIT,
			    EN_TIMER_SHIFT, 0);
	bq25896_set_value(REG08, 0xff,
			    0x00, 0x53);
}

void bq25896_otg_enable(int enable)
{
	uint32_t vchg_val;

	SPRD_EX_DEBUG("%s enable =%d\n", __func__, enable);
	if (enable) {
		bq25896_set_value(REG03, OTG_CONFIG_BIT,
				    OTG_CONFIG_SHIFT, CHG_OTG_VAL);
		vchg_val = sprdchg_read_vchg_vol();
		SPRD_EX_DEBUG("vchg_val = %d\n", vchg_val);
		/* no LED in bq25896 */
		/*
		 * bq25896_set_value(LED_DRV_CTL, OTG_RESTART_BIT,
		 *		    OTG_RESTART_SHIFT, 1);
		 */
	} else {
		bq25896_set_value(REG03, OTG_CONFIG_BIT,
				    OTG_CONFIG_SHIFT, 0x0);
	}
}
/* no flash and torch in bq25896 */
#if 0
void bq25896_enable_flash(int enable)
{
	SPRD_EX_DEBUG("bq25896_enable_flash enable =%d\n", enable);

	if (enable) {
		bq25896_set_value(LED_DRV_CTL, LED_MODE_CTL_BIT,
				    LED_MODE_CTL_SHIFT, LED_FLASH_EN_VAL);
	} else {
		bq25896_set_value(LED_DRV_CTL, LED_MODE_CTL_BIT,
				    LED_MODE_CTL_SHIFT, 0);
	}
}
EXPORT_SYMBOL_GPL(bq25896_enable_flash);
void bq25896_set_flash_brightness(unsigned char brightness)
{
	SPRD_EX_DEBUG(" brightness =%d\n", brightness);

	if (brightness > 0xf)
		brightness = 0xf;

	bq25896_set_value(LED_CUR_REF_CTL,
			    FLASH_MODE_CUR_BIT, FLASH_MODE_CUR_SHIFT,
			    brightness);
}
EXPORT_SYMBOL_GPL(bq25896_set_flash_brightness);

void bq25896_enable_torch(int enable)
{
	SPRD_EX_DEBUG("%s enable =%d\n", __func__, enable);

	if (enable) {
		bq25896_set_value(LED_DRV_CTL,
				    LED_MODE_CTL_BIT, LED_MODE_CTL_SHIFT,
				    LED_TORCH_EN_VAL);
	} else {
		bq25896_set_value(LED_DRV_CTL,
				    LED_MODE_CTL_BIT, LED_MODE_CTL_SHIFT, 0);
	}
}
EXPORT_SYMBOL_GPL(bq25896_enable_torch);

void bq25896_set_torch_brightness(unsigned char brightness)
{
	SPRD_EX_DEBUG("%s brightness =%d\n", __func__, brightness);

	if (brightness > 9)
		brightness = 0x9;

	bq25896_set_value(LED_CUR_REF_CTL, TORCH_MODE_CUR_BIT,
			    TORCH_MODE_CUR_SHIFT, brightness);

}
EXPORT_SYMBOL_GPL(bq25896_set_torch_brightness);
#endif

void bq25896_stop_charging(unsigned int flag)
{
	if (flag & (SPRDBAT_CHG_END_OVP_BIT | SPRDBAT_CHG_END_UNSPEC
	    | SPRDBAT_CHG_END_FORCE_STOP_BIT | SPRDBAT_CHG_END_CHGR_OTP_BIT))
		bq25896_set_value(REG00, EN_HIZ_BIT,
				EN_HIZ_SHIFT, 1);
	else
		bq25896_set_value(REG03, CHG_CONFIG_BIT,
				    CHG_CONFIG_SHIFT, CHG_DISABLE_VAL);
}

unsigned char bq25896_get_chg_config(void)
{
	return bq25896_get_value(REG03,
		CHG_CONFIG_BIT, CHG_CONFIG_SHIFT);
}

unsigned char bq25896_get_sys_status(void)
{
	unsigned char data = 0;

	bq25896_read_reg(REG0B, &data);
	return data;
}

unsigned char bq25896_get_fault_val(void)
{
	unsigned char data = 0;

	bq25896_read_reg(REG0C, &data);
	return data;
}

void bq25896_enable_chg(void)
{
	bq25896_set_value(REG03, CHG_CONFIG_BIT, CHG_CONFIG_SHIFT,
			    CHG_ENABLE_VAL);
	bq25896_set_value(REG00, EN_HIZ_BIT,
			    EN_HIZ_SHIFT, 0);
}

void bq25896_set_chg_current_limit(uint32_t limit)
{
	bq25896_set_value(REG00, IINLIM_BIT, IINLIM_SHIFT, limit / 50 + 1);
}

void bq25896_set_chg_current(unsigned char reg_val)
{
	bq25896_set_value(REG04, ICHG_BIT, ICHG_SHIFT, reg_val);
}

unsigned char bq25896_get_chgcur(void)
{
	unsigned char data = 0;

	bq25896_read_reg(REG04, &data);
	data = (data & ICHG_BIT) >> ICHG_SHIFT;
	return data;
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

	ret = bq25896_write_reg(reg, val);
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
	const int regaddrs[] = { 0x00, 0x01, 0x02, 0x03, 0x4,
		0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c,
		0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14
	};
	const char str[] = "0123456789abcdef";
	unsigned char bq25896_regs[0x60];
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

	for (i = 0; i <= 0x14; i++)
		bq25896_read_reg(i, &bq25896_regs[i]);

	for (i = 0; i < ARRAY_SIZE(regaddrs); i++) {
		index = regaddrs[i];
		val = bq25896_regs[index];
		buf[3 * index] = str[(val & 0xf0) >> 4];
		buf[3 * index + 1] = str[val & 0x0f];
	}

	return 0x60;
}
static DEVICE_ATTR_RO(dump_regs);

static struct attribute *bq25896_class_attrs[] = {
	&dev_attr_dump_regs.attr,
	&dev_attr_set_regs.attr,
	NULL,
};
ATTRIBUTE_GROUPS(bq25896_class);

static __used irqreturn_t bq25896_fault_handler(int irq, void *dev_id)
{
	struct bq25896 *p_bq25896 = (struct bq25896 *)dev_id;
	struct bq25896_platform_data *pdata =
	    i2c_get_clientdata(p_bq25896->client);
	int value = 0;

	SPRD_EX_DEBUG("enter sprdbat_vbat_chg_fault\n");
	value = gpio_get_value(pdata->irq_gpio_number);

	SPRD_EX_DEBUG("gpio_number = %d,value=%d\n",
		      pdata->irq_gpio_number, value);
	if (value) {
		SPRD_EX_DEBUG("gpio is high charge ic ok\n");
		irq_set_irq_type(p_bq25896->client->irq, IRQ_TYPE_LEVEL_LOW);
	} else {
		SPRD_EX_DEBUG("gpio is low some fault\n");
		irq_set_irq_type(p_bq25896->client->irq, IRQ_TYPE_LEVEL_HIGH);
	}
	schedule_work(&p_bq25896->chg_fault_irq_work);
	return IRQ_HANDLED;
}

static void sprdbat_chg_fault_irq_works(struct work_struct *work)
{
	unsigned char pre_data = 0, cur_data = 0;

	SPRD_EX_DEBUG("enter sprdbat_chg_fault_irq_works\n");

	/* read two times according to bq25896 spec */
	bq25896_read_reg(0x0C, &pre_data);
	bq25896_read_reg(0x0C, &cur_data);
	SPRD_EX_DEBUG("bq25896Reg0C,pre_data=0x%x,cur_data=0x%x\n",
		pre_data, cur_data);
	/* TODO: maybe need add */
}

static int bq25896_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	int rc = 0;
	struct bq25896 *bq25896_data = NULL;
	struct bq25896_platform_data *pdata = NULL;
	struct device_node *np = client->dev.of_node;

	SPRD_EX_DEBUG("@@@@@@@bq25896_probe\n");

	if (np && !pdata) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			SPRD_EX_DEBUG("%s alloc mem fail\n", __func__);
			return -ENOMEM;
		}

		pdata->irq_gpio_number =
			of_get_named_gpio(np, "chg-fault-gpios", 0);
		if (pdata->irq_gpio_number < 0) {
			SPRD_EX_DEBUG("fail to get irq_gpio_number\n");
			return -ENODEV;
		}

		pdata->vbat_detect =
			of_get_named_gpio(np, "battery-det-gpios", 0);
		if (pdata->vbat_detect < 0) {
			SPRD_EX_DEBUG("fail to get vbat_detect\n");
			return -ENODEV;
		}
	}

	rc = devm_gpio_request(&client->dev,
			       pdata->irq_gpio_number, "bq25896_gpio");
	if (rc) {
		SPRD_EX_DEBUG("gpio_request failed!\n");
		return -EINVAL;
	}
	gpio_direction_input(pdata->irq_gpio_number);
	client->irq = gpio_to_irq(pdata->irq_gpio_number);

	bq25896_data = devm_kzalloc(&client->dev,
				      sizeof(struct bq25896), GFP_KERNEL);
	if (!bq25896_data) {
		SPRD_EX_DEBUG("devm kzalloc failed!\n");
		return -ENOMEM;
	}

	pdata->bq25896_ops = sprd_get_bq25896_ops();
	sprdbat_register_ext_ops(pdata->bq25896_ops);
	i2c_set_clientdata(client, pdata);
	bq25896_data->client = client;
	this_client = client;

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_BYTE_DATA)) {
		SPRD_EX_DEBUG("%s: i2c check functionality error\n", __func__);
		return -ENODEV;
	}

	INIT_WORK(&bq25896_data->chg_fault_irq_work,
		  sprdbat_chg_fault_irq_works);
	if (client->irq > 0) {
		rc = devm_request_irq(&client->dev, client->irq,
				      bq25896_fault_handler,
				      IRQ_TYPE_LEVEL_LOW | IRQF_NO_SUSPEND,
				      "bq25896_chg_fault", bq25896_data);
		if (rc < 0)
			SPRD_EX_DEBUG("request_irq failed!\n");
	}

	rc = sysfs_create_group(&client->dev.kobj, bq25896_class_groups[0]);
	if (rc)
		pr_err("failed to create bq25896 sysfs device attributes\n");

	SPRD_EX_DEBUG("@@@@@@@bq25896_probe ok\n");
	return 0;
}

static int bq25896_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, bq25896_class_groups[0]);
	return 0;
}

#ifdef CONFIG_PM
static int bq25896_suspend(struct device *dev)
{
	return 0;
}

static int bq25896_resume(struct device *dev)
{
	return 0;
}
#endif

static UNIVERSAL_DEV_PM_OPS(bq25896_pm, bq25896_suspend, bq25896_resume, NULL);

static const struct i2c_device_id bq25896_i2c_id[] = {
	{"bq25896-chg", 0},
	{}
};

static const struct of_device_id bq25896_of_match[] = {
	{.compatible = "bq25896-chg",},
	{}
};

static struct i2c_driver bq25896_i2c_driver = {
	.driver = {
		   .name = "bq25896_chg",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(bq25896_of_match),
		   .pm	= &bq25896_pm,
		   },
	.probe = bq25896_probe,
	.remove = bq25896_remove,
	.id_table = bq25896_i2c_id,
};

static int __init bq25896_i2c_init(void)
{
	return i2c_add_driver(&bq25896_i2c_driver);
}

static void __exit bq25896_i2c_exit(void)
{
	i2c_del_driver(&bq25896_i2c_driver);
}

subsys_initcall_sync(bq25896_i2c_init);
module_exit(bq25896_i2c_exit);

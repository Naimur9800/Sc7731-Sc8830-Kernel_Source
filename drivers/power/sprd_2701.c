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
#include "sprd_2701.h"
#include "sprd_charge_helper.h"

#define SPRD_EX_DEBUG(format, arg...) pr_info("sprd 2701: "format, ## arg)

static struct i2c_client *this_client;

static int sprd_2701_write_reg(int reg, u8 val)
{
	int ret;

	SPRD_EX_DEBUG("#### writereg reg = %d val = %d\n", reg, val);

	ret = i2c_smbus_write_byte_data(this_client, reg, val);
	if (ret < 0)
		SPRD_EX_DEBUG("%s: error = %d", __func__, ret);

	return ret;
}

int sprd_2701_read_reg(int reg, u8 *dest)
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

static void sprd_2701_set_value(unsigned char reg,
				unsigned char reg_bit, unsigned char reg_shift,
				unsigned char val)
{
	unsigned char tmp = 0x0;

	sprd_2701_read_reg(reg, &tmp);
	tmp = (tmp & (~reg_bit)) | (val << reg_shift);
	SPRD_EX_DEBUG("continue test tmp =0x%x,val=0x%x\n", tmp, val);
	sprd_2701_write_reg(reg, tmp);
}

static unsigned char sprd_2701_get_value(unsigned char reg,
					 unsigned char reg_bit,
					 unsigned char reg_shift)
{
	unsigned char data = 0;
	int ret = 0;

	sprd_2701_read_reg(reg, &data);
	ret = (data & reg_bit) >> reg_shift;

	return ret;
}

void sprd_2701_reset_timer(void)
{
	SPRD_EX_DEBUG("sprd_2701 reset timer\n");
	sprd_2701_set_value(TIMER_CTL, WATCH_DOG_TIMER_BIT,
			    WATCH_DOG_TIMER_SHIFT, 0x00);
	sprd_2701_set_value(POWER_ON_CTL, TMR_RST_BIT,
			    TMR_RST_SHIFT, 0x00);
}

void sprd_2701_sw_reset(void)
{
	SPRD_EX_DEBUG("sprd_2701_sw_reset\n");
	sprd_2701_set_value(POWER_ON_CTL, SW_RESET_BIT, SW_RESET_SHIFT, 1);
}

void sprd_2701_set_vindpm(unsigned char reg_val)
{
	sprd_2701_set_value(INPUT_SRC_CTL, VIN_DPM_BIT, VIN_DPM_SHIFT, reg_val);
}

void sprd_2701_termina_cur_set(unsigned char reg_val)
{
	sprd_2701_set_value(TRICK_CHG_CTL, TERMINA_CUR_BIT,
			    TERMINA_CUR_SHIFT, reg_val);
}

void sprd_2701_termina_vol_set(unsigned char reg_val)
{
	sprd_2701_set_value(CHG_VOL_CTL, CHG_VOL_LIMIT_BIT,
			    CHG_VOL_LIMIT_SHIFT, reg_val);
}

void sprd_2701_termina_time_set(unsigned char reg_val)
{
	sprd_2701_set_value(TIMER_CTL, CHG_SAFE_TIMER_BIT,
			    CHG_SAFE_TIMER_SHIFT, reg_val);
}

void sprd_2701_init(void)
{
	unsigned char data = 0;
	int i = 0;

	SPRD_EX_DEBUG("sprd_2701_init\n");
	sprd_2701_sw_reset();
	for (i = 0; i < 11; i++) {
		sprd_2701_read_reg(i, &data);
		SPRD_EX_DEBUG("sprd_2701_ReadReg i = %d, data = %x\n", i, data);
	}

	sprd_2701_set_value(TIMER_CTL, WATCH_DOG_TIMER_BIT,
			    WATCH_DOG_TIMER_SHIFT, 0);
	sprd_2701_set_value(TIMER_CTL, CHG_TERMINA_EN_BIT,
			    CHG_TERMINA_EN_SHIFT, 0);
	sprd_2701_set_value(TIMER_CTL, CHG_SAFE_TIMER_EN_BIT,
			    CHG_SAFE_TIMER_EN_SHIFT, 0);
}

void sprd_2701_otg_enable(int enable)
{
	uint32_t vchg_val;

	SPRD_EX_DEBUG("%s enable =%d\n", __func__, enable);
	if (enable) {
		sprd_2701_set_value(POWER_ON_CTL, CHG_EN_BIT,
				    CHG_EN_SHIFT, CHG_OTG_VAL);
		vchg_val = sprdchg_read_vchg_vol();
		SPRD_EX_DEBUG("vchg_val = %d\n", vchg_val);
		sprd_2701_set_value(LED_DRV_CTL, OTG_RESTART_BIT,
				    OTG_RESTART_SHIFT, 1);
	} else {
		sprd_2701_set_value(POWER_ON_CTL, CHG_EN_BIT,
				    CHG_EN_SHIFT, CHG_DISABLE_VAL);
	}
}

void sprd_2701_enable_flash(int enable)
{
	SPRD_EX_DEBUG("sprd_2701_enable_flash enable =%d\n", enable);

	if (enable) {
		sprd_2701_set_value(LED_DRV_CTL, LED_MODE_CTL_BIT,
				    LED_MODE_CTL_SHIFT, LED_FLASH_EN_VAL);
	} else {
		sprd_2701_set_value(LED_DRV_CTL, LED_MODE_CTL_BIT,
				    LED_MODE_CTL_SHIFT, 0);
	}
}
EXPORT_SYMBOL_GPL(sprd_2701_enable_flash);

void sprd_2701_set_flash_brightness(unsigned char brightness)
{
	SPRD_EX_DEBUG(" brightness =%d\n", brightness);

	if (brightness > 0xf)
		brightness = 0xf;

	sprd_2701_set_value(LED_CUR_REF_CTL,
			    FLASH_MODE_CUR_BIT, FLASH_MODE_CUR_SHIFT,
			    brightness);
}
EXPORT_SYMBOL_GPL(sprd_2701_set_flash_brightness);

void sprd_2701_enable_torch(int enable)
{
	SPRD_EX_DEBUG("%s enable =%d\n", __func__, enable);

	if (enable) {
		sprd_2701_set_value(LED_DRV_CTL,
				    LED_MODE_CTL_BIT, LED_MODE_CTL_SHIFT,
				    LED_TORCH_EN_VAL);
	} else {
		sprd_2701_set_value(LED_DRV_CTL,
				    LED_MODE_CTL_BIT, LED_MODE_CTL_SHIFT, 0);
	}
}
EXPORT_SYMBOL_GPL(sprd_2701_enable_torch);

void sprd_2701_set_torch_brightness(unsigned char brightness)
{
	SPRD_EX_DEBUG("%s brightness =%d\n", __func__, brightness);

	if (brightness > 9)
		brightness = 0x9;

	sprd_2701_set_value(LED_CUR_REF_CTL, TORCH_MODE_CUR_BIT,
			    TORCH_MODE_CUR_SHIFT, brightness);

}
EXPORT_SYMBOL_GPL(sprd_2701_set_torch_brightness);

void sprd_2701_stop_charging(unsigned int flag)
{
	if (flag & (SPRDBAT_CHG_END_OVP_BIT | SPRDBAT_CHG_END_UNSPEC
	    | SPRDBAT_CHG_END_FORCE_STOP_BIT | SPRDBAT_CHG_END_CHGR_OTP_BIT))
		sprd_2701_set_value(POWER_ON_CTL, CHG_EN_BIT, CHG_EN_SHIFT, 3);
	else
		sprd_2701_set_value(POWER_ON_CTL, CHG_EN_BIT,
				    CHG_EN_SHIFT, CHG_DISABLE_VAL);
}

unsigned char sprd_2701_get_chg_config(void)
{
	return sprd_2701_get_value(POWER_ON_CTL,
		CHG_EN_BIT, CHG_EN_SHIFT);
}

unsigned char sprd_2701_get_sys_status(void)
{
	unsigned char data = 0;

	sprd_2701_read_reg(SYS_STATUS_REG, &data);
	return data;
}

unsigned char sprd_2701_get_fault_val(void)
{
	unsigned char data = 0;

	sprd_2701_read_reg(FAULT_REG, &data);
	return data;
}

void sprd_2701_enable_chg(void)
{
	sprd_2701_set_value(POWER_ON_CTL, CHG_EN_BIT, CHG_EN_SHIFT,
			    CHG_BAT_VAL);
}

void sprd_2701_set_chg_current_limit(uint32_t cur)
{
	unsigned char reg_val;

	if (cur <= 300)
		reg_val = 0;
	else if (cur <= 500)
		reg_val = 1;
	else if (cur <= 900)
		reg_val = 2;
	else if (cur <= 1200)
		reg_val = 3;
	else if (cur <= 1500)
		reg_val = 4;
	else if (cur <= 2000)
		reg_val = 5;
	else if (cur <= 3000)
		reg_val = 6;
	else
		reg_val = 7;
	sprd_2701_set_value(INPUT_SRC_CTL,
		IN_CUR_LIMIT_BIT, IN_CUR_LIMIT_SHIFT, reg_val);
}

void sprd_2701_set_chg_current(unsigned char reg_val)
{
	sprd_2701_set_value(CHG_CUR_CTL, CHG_CUR_BIT, CHG_CUR_SHIFT, reg_val);
}

unsigned char sprd_2701_get_chgcur(void)
{
	unsigned char data = 0;

	sprd_2701_read_reg(CHG_CUR_CTL, &data);
	data = (data & CHG_CUR_BIT) >> CHG_CUR_SHIFT;
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

	ret = sprd_2701_write_reg(reg, val);
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
		0x05, 0x06, 0x07, 0x08, 0x09, 0x0a
	};
	const char str[] = "0123456789abcdef";
	unsigned char sprd_2701_regs[0x60];
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

	for (i = 0; i < 11; i++)
		sprd_2701_read_reg(i, &sprd_2701_regs[i]);

	for (i = 0; i < ARRAY_SIZE(regaddrs); i++) {
		index = regaddrs[i];
		val = sprd_2701_regs[index];
		buf[3 * index] = str[(val & 0xf0) >> 4];
		buf[3 * index + 1] = str[val & 0x0f];
		buf[3 * index + 1] = str[val & 0x0f];
	}

	return 0x60;
}
static DEVICE_ATTR_RO(dump_regs);

static struct attribute *sprd2701_class_attrs[] = {
	&dev_attr_dump_regs.attr,
	&dev_attr_set_regs.attr,
	NULL,
};
ATTRIBUTE_GROUPS(sprd2701_class);

static __used irqreturn_t sprd_2701_fault_handler(int irq, void *dev_id)
{
	struct sprd_2701 *p_sprd_2701 = (struct sprd_2701 *)dev_id;
	struct sprd_2701_platform_data *pdata =
	    i2c_get_clientdata(p_sprd_2701->client);
	int value = 0;

	SPRD_EX_DEBUG("enter sprdbat_vbat_chg_fault\n");
	value = gpio_get_value(pdata->irq_gpio_number);

	SPRD_EX_DEBUG("gpio_number = %d,value=%d\n",
		      pdata->irq_gpio_number, value);
	if (value) {
		SPRD_EX_DEBUG("gpio is high charge ic ok\n");
		irq_set_irq_type(p_sprd_2701->client->irq, IRQ_TYPE_LEVEL_LOW);
	} else {
		SPRD_EX_DEBUG("gpio is low some fault\n");
		irq_set_irq_type(p_sprd_2701->client->irq, IRQ_TYPE_LEVEL_HIGH);
	}
	schedule_work(&p_sprd_2701->chg_fault_irq_work);
	return IRQ_HANDLED;
}

static void sprdbat_chg_fault_irq_works(struct work_struct *work)
{
	SPRD_EX_DEBUG("enter sprdbat_chg_fault_irq_works\n");
	/* TODO: maybe need add */
}

static int sprd_2701_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	int rc = 0;
	struct sprd_2701 *sprd_2701_data = NULL;
	struct sprd_2701_platform_data *pdata = NULL;
	struct device_node *np = client->dev.of_node;

	SPRD_EX_DEBUG("@@@@@@@sprd_2701_probe\n");

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
			       pdata->irq_gpio_number, "sprd_2701_gpio");
	if (rc) {
		SPRD_EX_DEBUG("gpio_request failed!\n");
		return -EINVAL;
	}
	gpio_direction_input(pdata->irq_gpio_number);
	client->irq = gpio_to_irq(pdata->irq_gpio_number);

	sprd_2701_data = devm_kzalloc(&client->dev,
				      sizeof(struct sprd_2701), GFP_KERNEL);
	if (!sprd_2701_data) {
		SPRD_EX_DEBUG("devm kzalloc failed!\n");
		return -ENOMEM;
	}

	pdata->sprd_2701_ops = sprd_get_2701_ops();
	sprdbat_register_ext_ops(pdata->sprd_2701_ops);
	i2c_set_clientdata(client, pdata);
	sprd_2701_data->client = client;
	this_client = client;

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_BYTE_DATA)) {
		SPRD_EX_DEBUG("%s: i2c check functionality error\n", __func__);
		return -ENODEV;
	}

	INIT_WORK(&sprd_2701_data->chg_fault_irq_work,
		  sprdbat_chg_fault_irq_works);
	if (client->irq > 0) {
		rc = devm_request_irq(&client->dev, client->irq,
				      sprd_2701_fault_handler,
				      IRQ_TYPE_LEVEL_LOW | IRQF_NO_SUSPEND,
				      "sprd_2701_chg_fault", sprd_2701_data);
		if (rc < 0)
			SPRD_EX_DEBUG("request_irq failed!\n");
	}

	rc = sysfs_create_group(&client->dev.kobj, sprd2701_class_groups[0]);
	if (rc)
		pr_err("failed to create 2701 sysfs device attributes\n");

	SPRD_EX_DEBUG("@@@@@@@sprd_2701_probe ok\n");
	return 0;
}

static int sprd_2701_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, sprd2701_class_groups[0]);
	return 0;
}

#ifdef CONFIG_PM
static int sprd_2701_suspend(struct device *dev)
{
	return 0;
}

static int sprd_2701_resume(struct device *dev)
{
	return 0;
}
#endif

static UNIVERSAL_DEV_PM_OPS(sprd_2701_pm, sprd_2701_suspend, sprd_2701_resume, NULL);

static const struct i2c_device_id sprd_2701_i2c_id[] = {
	{"2701-chg", 0},
	{}
};

static const struct of_device_id sprd_2701_of_match[] = {
	{.compatible = "sprd,2701-chg",},
	{}
};

static struct i2c_driver sprd_2701_i2c_driver = {
	.driver = {
		   .name = "sprd_2701_chg",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(sprd_2701_of_match),
		   .pm	= &sprd_2701_pm,
		   },
	.probe = sprd_2701_probe,
	.remove = sprd_2701_remove,
	.id_table = sprd_2701_i2c_id,
};

static int __init sprd_2701_i2c_init(void)
{
	return i2c_add_driver(&sprd_2701_i2c_driver);
}

static void __exit sprd_2701_i2c_exit(void)
{
	i2c_del_driver(&sprd_2701_i2c_driver);
}

subsys_initcall_sync(sprd_2701_i2c_init);
module_exit(sprd_2701_i2c_exit);

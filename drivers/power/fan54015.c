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
#include <linux/mfd/sprd/pmic_glb_reg.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include "fan54015.h"
#include "sprd_charge_helper.h"

#define SPRD_EX_DEBUG(format, arg...) pr_info("fan54015: " format, ## arg)
#define BIT_DP_DM_SW_EN                      BIT(0)

static struct fan54015 *fan54015_data;
static uint32_t vbus_gpio;
static uint32_t otg_enable_flag;

static int fan54015_write_reg(int reg, u8 val)
{
	int ret;

	SPRD_EX_DEBUG("#### writereg reg = %d val = %d\n", reg, val);
	ret = i2c_smbus_write_byte_data(fan54015_data->client, reg, val);

	if (ret < 0)
		SPRD_EX_DEBUG("%s: error = %d", __func__, ret);

	return ret;
}

int fan54015_read_reg(int reg, u8 *dest)
{
	int ret;

	ret = i2c_smbus_read_byte_data(fan54015_data->client, reg);
	if (ret < 0) {
		SPRD_EX_DEBUG("%s reg(0x%x), ret(%d)\n", __func__, reg, ret);
		return ret;
	}

	*dest = (u8)(ret & 0xff);
	SPRD_EX_DEBUG("%s reg  = %d value =%d/%x\n", __func__, reg, ret, ret);
	return 0;
}

static void fan54015_set_value(unsigned char reg, unsigned char reg_bit,
		unsigned char reg_shift, unsigned char val)
{
	unsigned char tmp = 0x0;

	fan54015_read_reg(reg, &tmp);
	tmp = (tmp & (~reg_bit)) | (val << reg_shift);
	if ((0x04 == reg) && (FAN5405_RESET != reg_bit))
		tmp &= 0x7f;

	SPRD_EX_DEBUG("setvalue  =0x%x,val=0x%x\n", tmp, val);
	fan54015_write_reg(reg, tmp);
}

static unsigned char fan54015_get_value(unsigned char reg,
		unsigned char reg_bit, unsigned char reg_shift)
{
	unsigned char data = 0;
	int ret = 0;

	ret = fan54015_read_reg(reg, &data);
	if (ret < 0) {
		SPRD_EX_DEBUG("%s reg(0x%x), ret(%d)\n", __func__, reg, ret);
		return 0;
	}
	ret = (data & reg_bit) >> reg_shift;

	return ret;
}

static void fan54015_dump_regs(void)
{
	int i;
	unsigned char data = 0;

	for (i = 0; i < FAN54015_REGS_NUM; i++) {
		fan54015_read_reg(i, &data);
		SPRD_EX_DEBUG("%s i = %d, data = %x\n",
			__func__, i, data);
	}
}

void fan54015_reset_timer(void)
{
	fan54015_set_value(FAN5405_REG_CONTROL0, FAN5405_TMR_RST_OTG,
			   FAN5405_TMR_RST_OTG_SHIFT, RESET32S);
}

void fan54015_sw_reset(void)
{
	SPRD_EX_DEBUG("fan54015_sw_reset\n");
	fan54015_set_value(FAN5405_REG_IBAT, FAN5405_RESET, FAN5405_RESET_SHIFT,
			   1);
}

void fan54015_enable_cur_terminal(int enable)
{
	if (enable)
		fan54015_set_value(FAN5405_REG_CONTROL1, FAN5405_TE,
				   FAN5405_TE_SHIFT, ENTE);
	else
		fan54015_set_value(FAN5405_REG_CONTROL1, FAN5405_TE,
				   FAN5405_TE_SHIFT, DISTE);
}

void fan54015_set_vindpm(unsigned char reg_val)
{
	fan54015_set_value(FAN5405_REG_SP_CHARGER, FAN5405_VSP,
			   FAN5405_VSP_SHIFT, reg_val);
}

void fan54015_termina_cur_set(unsigned char reg_val)
{
	fan54015_set_value(FAN5405_REG_IBAT, FAN5405_ITERM, FAN5405_ITERM_SHIFT,
			   reg_val);
}

void fan54015_termina_vol_set(unsigned char reg_val)
{
	fan54015_set_value(FAN5405_REG_OREG, FAN5405_OREG, FAN5405_OREG_SHIFT,
			   reg_val);
}

static void fan54015_15s_work(struct work_struct *work)
{
	fan54015_reset_timer();
	schedule_delayed_work(&fan54015_data->feed_watchdog_work, HZ * 15);
}

static void fan54015_vbus_det_work(struct work_struct *work)
{
	uint32_t vbus_gpio_value;

	if (vbus_gpio) {
		vbus_gpio_value = gpio_get_value(vbus_gpio);
		if ((!vbus_gpio_value) && (otg_enable_flag == 1)) {
			SPRD_EX_DEBUG("restart otg\n");
			fan54015_set_value(FAN5405_REG_CONTROL1,
				FAN5405_OPA_MODE | FAN5405_HZ_MODE,
				FAN5405_OPA_MODE_SHIFT, 1);
		}
		schedule_delayed_work(&fan54015_data->vbus_detect_work,
			msecs_to_jiffies(1500));
	}
}

void fan54015_init(void)
{
	SPRD_EX_DEBUG("fan54015_init\n");

	fan54015_sw_reset();

	fan54015_set_value(FAN5405_REG_CONTROL1,
		FAN5405_VLOWV, FAN5405_VLOWV_SHIFT, VLOWV3P4);
	fan54015_set_value(FAN5405_REG_SP_CHARGER,
		FAN5405_IO_LEVEL, FAN5405_IO_LEVEL_SHIFT, ENIOLEVEL);
	fan54015_dump_regs();
}

void fan54015_set_safety_vol(unsigned char reg_val)
{
	fan54015_set_value(FAN5405_REG_SAFETY, FAN5405_VSAFE,
			   FAN5405_VSAFE_SHIFT, reg_val);
}

void fan54015_set_safety_cur(unsigned char reg_val)
{
	fan54015_set_value(FAN5405_REG_SAFETY, FAN5405_ISAFE,
			   FAN5405_ISAFE_SHIFT, reg_val);
}

void fan54015_otg_enable(int enable)
{
	SPRD_EX_DEBUG("%s enable =%d\n", __func__, enable);

	if (enable) {
		otg_enable_flag = 1;
		fan54015_set_value(FAN5405_REG_CONTROL1,
			FAN5405_OPA_MODE | FAN5405_HZ_MODE,
			FAN5405_OPA_MODE_SHIFT, enable);
		schedule_delayed_work(&fan54015_data->feed_watchdog_work,
				      msecs_to_jiffies(50));
		schedule_delayed_work(&fan54015_data->vbus_detect_work,
				      msecs_to_jiffies(500));
	} else {
		otg_enable_flag = 0;
		cancel_delayed_work_sync(&fan54015_data->feed_watchdog_work);
		cancel_delayed_work_sync(&fan54015_data->vbus_detect_work);
		fan54015_set_value(FAN5405_REG_CONTROL1,
			FAN5405_OPA_MODE | FAN5405_HZ_MODE,
			FAN5405_OPA_MODE_SHIFT, enable);
	}
}

void fan54015_stop_charging(void)
{
	sprd_charge_pd_control(false);
}

unsigned char fan54015_get_vendor_id(void)
{
	return fan54015_get_value(FAN5405_REG_IC_INFO,
				  FAN5405_VENDOR_CODE,
				  FAN5405_VENDOR_CODE_SHIFT);
}

unsigned char fan54015_get_chg_status(void)
{
	return fan54015_get_value(FAN5405_REG_CONTROL0,
				  FAN5405_STAT, FAN5405_STAT_SHIFT);
}

unsigned char fan54015_get_fault_val(void)
{
	return fan54015_get_value(FAN5405_REG_CONTROL0,
				  FAN5405_FAULT, FAN5405_FAULT_SHIFT);
}

void fan54015_enable_chg(void)
{
	sprd_charge_pd_control(true);
}

void fan54015_set_chg_current(unsigned char reg_val)
{
	if (reg_val == 0)
		fan54015_set_value(FAN5405_REG_CONTROL1,
				   FAN5405_IINLIM, FAN5405_IINLIM_SHIFT,
				   IINLIM500);
	else
		fan54015_set_value(FAN5405_REG_CONTROL1,
				   FAN5405_IINLIM, FAN5405_IINLIM_SHIFT,
				   NOLIMIT);
	fan54015_set_value(FAN5405_REG_IBAT, FAN5405_IOCHARGE,
			   FAN5405_IOCHARGE_SHIFT, reg_val);
}

unsigned char fan54015_get_chg_current(void)
{
	return fan54015_get_value(FAN5405_REG_IBAT,
				  FAN5405_IOCHARGE, FAN5405_IOCHARGE_SHIFT);
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
	ret = fan54015_write_reg(reg, val);

	if (ret < 0) {
		pr_err("set_regs_store error\n");
		return -EINVAL;
	}

	return count;
}

static ssize_t dump_regs_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	int regaddrs[] = { 0x00, 0x01, 0x02, 0x03, 0x4, 0x05,
		0x06, 0x07, 0x08, 0x09, 0x10
	};
	char str[] = "0123456789abcdef";
	unsigned char fan54015_regs[0x60];
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
		fan54015_read_reg(i, &fan54015_regs[i]);

	for (i = 0; i < ARRAY_SIZE(regaddrs); i++) {
		index = regaddrs[i];
		val = fan54015_regs[index];
		buf[3 * index] = str[(val & 0xf0) >> 4];
		buf[3 * index + 1] = str[val & 0x0f];
		buf[3 * index + 1] = str[val & 0x0f];
	}

	return 0x60;
}

static DEVICE_ATTR_RO(dump_regs);
static DEVICE_ATTR_WO(set_regs);

static struct attribute *fan54015_class_attrs[] = {
	&dev_attr_dump_regs.attr,
	&dev_attr_set_regs.attr,
	NULL,
};

ATTRIBUTE_GROUPS(fan54015_class);

static int fan54015_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int ret = 0;
	int rc;
	uint32_t temp;
	struct sprd_ext_ic_operations *fan54015_ops = NULL;
	struct device_node *np = client->dev.of_node;

	fan54015_data = devm_kzalloc(&client->dev,
				     sizeof(struct fan54015), GFP_KERNEL);
	if (!fan54015_data) {
		SPRD_EX_DEBUG("kzalloc failed!\n");
		return -ENOMEM;
	}

	fan54015_data->client = client;

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("%s: i2c check functionality error\n", __func__);
		return -ENODEV;
	}

	{
		unsigned char data = 0;

		if (fan54015_read_reg(0, &data)) {
			SPRD_EX_DEBUG("%s: NO fan54015 deivce!\n", __func__);
			return -ENXIO;
		}
	}

	temp = of_get_named_gpio(np, "vbus-det-gpios", 0);
	if (gpio_is_valid(temp)) {
		vbus_gpio = temp;
		SPRD_EX_DEBUG("vbus_gpio_detect =%d\n", vbus_gpio);
		rc = gpio_request(vbus_gpio, "musb vbus detect");
		if (rc) {
			SPRD_EX_DEBUG("already request vbus_gpio_detect gpio:%d\n",
				vbus_gpio);
		}
		gpio_direction_input(vbus_gpio);
	} else {
		vbus_gpio = 0;
	}

	INIT_DELAYED_WORK(&fan54015_data->feed_watchdog_work,
			  fan54015_15s_work);
	INIT_DELAYED_WORK(&fan54015_data->vbus_detect_work,
			  fan54015_vbus_det_work);

	ret = sysfs_create_group(&client->dev.kobj,
		fan54015_class_groups[0]);
	if (ret)
		pr_err("failed to create 54015 sysfs device attributes\n");

	fan54015_ops = sprd_get_54015_ops();
	if (!fan54015_ops)
		return -EINVAL;

	sprdbat_register_ext_ops(fan54015_ops);

	SPRD_EX_DEBUG("fan54015 probe ok\n");
	return ret;
}

static int fan54015_remove(struct i2c_client *client)
{
	flush_scheduled_work();
	sysfs_remove_group(&client->dev.kobj, fan54015_class_groups[0]);
	return 0;
}

#ifdef CONFIG_PM
static int fan54015_suspend(struct device *dev)
{
	return 0;
}

static int fan54015_resume(struct device *dev)
{
	return 0;
}
#endif

static UNIVERSAL_DEV_PM_OPS(fan54015_pm, fan54015_suspend, fan54015_resume, NULL);

static const struct i2c_device_id fan54015_i2c_id[] = {
	{"fan54015_chg", 0},
	{}
};

static const struct of_device_id fan54015_of_match[] = {
	{.compatible = "fairchild, fan54015_chg",},
	{}
};

static struct i2c_driver fan54015_i2c_driver = {
	.driver = {
		   .name = "fan54015_chg",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(fan54015_of_match),
		   .pm	= &fan54015_pm,
		   },
	.probe = fan54015_probe,
	.remove = fan54015_remove,
	.id_table = fan54015_i2c_id,
};

static int __init fan54015_i2c_init(void)
{
	return i2c_add_driver(&fan54015_i2c_driver);
}

static void __exit fan54015_i2c_exit(void)
{
	i2c_del_driver(&fan54015_i2c_driver);
}

subsys_initcall_sync(fan54015_i2c_init);
module_exit(fan54015_i2c_exit);

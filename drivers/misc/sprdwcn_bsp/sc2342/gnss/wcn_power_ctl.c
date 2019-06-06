/*
* Copyright (C) 2015 Spreadtrum Communications Inc.
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*/

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kdev_t.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/major.h>
#include <linux/marlin_platform.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/proc_fs.h>
#include <linux/poll.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/suspend.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>
#include <linux/wait.h>

#include "./../platform/mdbg_main.h"

#define	POWER_CTL_NAME		"power_ctl"
#define DOWNLOAD_IOCTL_BASE	'z'
#define DOWNLOAD_POWER_ON	_IO(DOWNLOAD_IOCTL_BASE, 0x01)
#define DOWNLOAD_POWER_OFF	_IO(DOWNLOAD_IOCTL_BASE, 0x02)
#define DOWNLOAD_POWER_RST	_IO(DOWNLOAD_IOCTL_BASE, 0x03)
#define GNSS_CHIP_EN		_IO(DOWNLOAD_IOCTL_BASE, 0x04)
#define GNSS_CHIP_DIS		_IO(DOWNLOAD_IOCTL_BASE, 0x05)
#define GNSS_LNA_EN		_IO(DOWNLOAD_IOCTL_BASE, 0x06)
#define GNSS_LNA_DIS		_IO(DOWNLOAD_IOCTL_BASE, 0x07)
#define MARLIN_SET_VERSION	_IO(DOWNLOAD_IOCTL_BASE, 0x08)
#define MARLIN_NOWAIT_GNSS	_IO(DOWNLOAD_IOCTL_BASE, 0x09)
#define GNSS_SLP_INFO       _IO(GNSS_POWER_IOCTL_BASE, 0x0a)
#define GNSS_DATA_MAX_LEN	11

struct sprd_marlin {
	u32 gpio_rst;
	bool flag_marlin_version;
};

struct sprd_gnss {
	u32 chip_en;
	bool gnss_flag_sleep;
	bool gnss_flag_resume;
	char gnss_status[16];
	struct regulator *vdd_lna;
	wait_queue_head_t gnss_sleep_wait;
};

struct sprd_wcn {
	struct regulator *download_vdd;
	struct clk *download_clk;
	struct clk *clk_parent;
	struct clk *clk_enable;
};

static struct sprd_wcn wcn_ctl;
static struct sprd_marlin marlin_dev;
static struct sprd_gnss gnss_dev;
static int g_rfhwid = 0xff;
static unsigned int gnss_ctl_cmd;
static int gnss_delay_cancel;

static int sprd_power_ctl_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int sprd_power_ctl_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static void download_clk_init(bool enable)
{
	if ((wcn_ctl.download_clk) && (wcn_ctl.clk_enable)) {
		if (enable) {
			clk_prepare_enable(wcn_ctl.download_clk);
			clk_prepare_enable(wcn_ctl.clk_enable);
		} else {
			clk_disable_unprepare(wcn_ctl.clk_enable);
			clk_disable_unprepare(wcn_ctl.download_clk);
		}
	}
}

static DEFINE_MUTEX(power_ctl_lock);
static void sprd_download_poweron(bool enable)
{
	int ret;
	static unsigned int power_count;
	struct device_node *np_marlin2 = NULL;
	int voltage_value;

	if (!wcn_ctl.download_vdd)
		return;

	voltage_value = 1600000;
	np_marlin2 = of_find_node_by_name(NULL, "sprd-marlin2");
	if (np_marlin2) {
		voltage_value = 1500000;
		pr_info("Found marlin2 chip\n");
	}
	mutex_lock(&power_ctl_lock);
	if (enable) {
		if (power_count == 0) {
			regulator_set_voltage(wcn_ctl.download_vdd,
					      voltage_value, voltage_value);
			ret = regulator_enable(wcn_ctl.download_vdd);
			pr_info("sprd_download_poweron on\n");
		}
		power_count++;
	} else {
		power_count--;
		if (power_count == 0) {
			if (regulator_is_enabled(wcn_ctl.download_vdd))
				ret = regulator_disable(wcn_ctl.download_vdd);
			pr_info("sprd_download_poweron off\n");
		}
	}
	mutex_unlock(&power_ctl_lock);
}

static void sprd_download_rst(void)
{
	gpio_direction_output(marlin_dev.gpio_rst, 1);
	mdelay(1);
	gpio_direction_output(marlin_dev.gpio_rst, 0);
	mdelay(1);
	gpio_direction_output(marlin_dev.gpio_rst, 1);
	mdelay(1);
}

/* true: marlin 15c,false: marlin 15a */
static void sprd_set_marlin_version(bool value)
{
	pr_info("sprd_set_marlin_version:%d\n", value);
	marlin_dev.flag_marlin_version = value;
}

bool sprd_get_marlin_version(void)
{
	return marlin_dev.flag_marlin_version;
}
EXPORT_SYMBOL_GPL(sprd_get_marlin_version);

static void sprd_gnss_chip_en(bool enable)
{
	struct device_node *np_marlin2 = NULL;

	np_marlin2 = of_find_node_by_name(NULL, "sprd-marlin2");
	if (np_marlin2) {
		if (of_get_property(np_marlin2, "common_chip_en", NULL)) {
			pr_info("common_chip_en\n");
			marlin_chip_en(enable, false);
			return;
		}
	}

	if (enable) {
		gpio_direction_output(gnss_dev.chip_en, 0);
		mdelay(1);
		gpio_direction_output(gnss_dev.chip_en, 1);
		mdelay(1);
	} else {
		gpio_direction_output(gnss_dev.chip_en, 0);
	}
}

static int __init early_rfhwid(char *p)
{
	if (!p)
		return 1;
	g_rfhwid = memparse(p, &p);
	return 0;
}
early_param("rfhw.id",  early_rfhwid);

static void gnss_lna_enable(bool on)
{
	int ret;

	if ((g_rfhwid == 0) || (g_rfhwid == 3))
		return;

	if (gnss_dev.vdd_lna) {
		if (on) {
			regulator_set_voltage(gnss_dev.vdd_lna,
					      1800000, 1800000);
			ret = regulator_enable(gnss_dev.vdd_lna);
		} else if (regulator_is_enabled(gnss_dev.vdd_lna)) {
			ret = regulator_disable(gnss_dev.vdd_lna);
		}
	}
}

static long sprd_power_ctl_ioctl(struct file *file,
				 unsigned int cmd, unsigned long arg)
{
	pr_info("DOWNLOAD IOCTL: 0x%x.\n", cmd);

	gnss_ctl_cmd = cmd;
	switch (cmd) {
	case DOWNLOAD_POWER_ON:
		sprd_download_poweron(1);
		download_clk_init(1);
		break;
	case DOWNLOAD_POWER_OFF:
		sprd_download_poweron(0);
		break;
	case DOWNLOAD_POWER_RST:
		sprd_download_rst();
		break;
	case GNSS_CHIP_EN:
		sprd_gnss_chip_en(1);
		break;
	case GNSS_CHIP_DIS:
		sprd_gnss_chip_en(0);
		break;
	case GNSS_LNA_EN:
		gnss_lna_enable(1);
		break;
	case GNSS_LNA_DIS:
		gnss_lna_enable(0);
		break;
	case MARLIN_SET_VERSION:
		sprd_set_marlin_version(true);
		break;
	case MARLIN_NOWAIT_GNSS:
		complete(&ge2_completion);
		break;
	}

	return 0;
}

static void sprd_download_dts_init(void)
{
	int ret;
	struct platform_device *pdev, *gnss_pdev;
	struct device_node *marlin_np, *gnss_np;
	struct device_node *np = NULL;

	marlin_np = of_find_node_by_name(NULL, "sprd-marlin");
	if (!marlin_np)
		pr_err("sprd-marlin not found");
	else {
		np = marlin_np;
		marlin_dev.gpio_rst = of_get_named_gpio(marlin_np,
							"cp-reset-gpios", 0);
		ret = gpio_request(marlin_dev.gpio_rst, "download");
		if (ret)
			pr_err("gpio_rst request err: %d\n",
					marlin_dev.gpio_rst);
	}

	gnss_np = of_find_node_by_name(NULL, "sprd-ge2");
	if (!gnss_np)
		pr_err("sprd-ge2 not found");
	else {
		np = gnss_np;
		gnss_pdev = of_find_device_by_node(gnss_np);
		if (!gnss_pdev)
			pr_err("ge2 get platform device failed!\n");

		gnss_dev.vdd_lna = devm_regulator_get_optional(&gnss_pdev->dev,
												"vddlna");
		if (IS_ERR(gnss_dev.vdd_lna)) {
			pr_err("Get regulator of vddlna error:%p\n",
					gnss_dev.vdd_lna);
			gnss_dev.vdd_lna = NULL;
		}

		gnss_dev.chip_en = of_get_named_gpio(gnss_np,
							"chip-en-gpios", 0);
		ret = gpio_request(gnss_dev.chip_en,
						"ge2_chip_en");
		if (ret)
			pr_err("gnss_dev.chip_en request err: %d\n",
				gnss_dev.chip_en);
	}

	if (!np) {
		pr_err("not found sprd-ge2 and sprd-marlin");
		return;
	}
		pdev = of_find_device_by_node(np);
		if (!pdev) {
			pr_err("wcn get platform device failed!\n");
			return;
		}

		wcn_ctl.download_vdd = devm_regulator_get(&pdev->dev, "vddwcn");
		if (IS_ERR(wcn_ctl.download_vdd)) {
			pr_err("Get regulator of vddwcn  error!\n");
			return;
		}

		wcn_ctl.download_clk = of_clk_get_by_name(np, "clk_32k");
		if (IS_ERR(wcn_ctl.download_clk)) {
			pr_err("can't get wcn clock dts config: clk_32k\n");
			return;
		}

		wcn_ctl.clk_parent = of_clk_get_by_name(np, "source");
		if (IS_ERR(wcn_ctl.clk_parent)) {
			pr_err("can't get wcn clock dts config: source\n");
			return;
		}
		clk_set_parent(wcn_ctl.download_clk, wcn_ctl.clk_parent);

		wcn_ctl.clk_enable = of_clk_get_by_name(np, "enable");
		if (IS_ERR(wcn_ctl.clk_enable)) {
			pr_err("can't get wcn clock dts config: enable\n");
			return;
	}
}

bool gnss_delay_ctl(void)
{
	return (gnss_ctl_cmd == GNSS_LNA_DIS);
}
static int gnss_pm_notify(struct notifier_block *nb,
			  unsigned long event, void *dummy)
{
	int i = 10;

	if (event == PM_SUSPEND_PREPARE) {
		gnss_dev.gnss_flag_sleep = true;
		wake_up_interruptible(&gnss_dev.gnss_sleep_wait);
		gnss_delay_cancel = 0;
		while (gnss_delay_ctl() && (i--) &&
				(gnss_delay_cancel != 1)) {
			mdelay(2);
		}
	} else
		gnss_dev.gnss_flag_sleep = false;
	pr_info("%s event:%ld\n", __func__, event);

	return NOTIFY_OK;
}

static struct notifier_block gnss_pm_notifier = {
	.notifier_call = gnss_pm_notify,
};

static unsigned int gnss_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;

	poll_wait(filp, &gnss_dev.gnss_sleep_wait, wait);
	if (gnss_dev.gnss_flag_sleep == true)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static ssize_t sprd_gnss_read(struct file *filp,
			  char __user *buf, size_t count, loff_t *pos)
{
	gnss_delay_cancel = 1;

	return (gnss_dev.gnss_flag_sleep == true) ? 1:0;
}

static const struct file_operations sprd_power_ctl__fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = sprd_power_ctl_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = sprd_power_ctl_ioctl,
#endif
	.open = sprd_power_ctl_open,
	.release = sprd_power_ctl_release,
	.read = sprd_gnss_read,
	.poll = gnss_poll,
};

static struct miscdevice sprd_power_ctl_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = POWER_CTL_NAME,
	.fops = &sprd_power_ctl__fops,
};

static int __init sprd_power_ctl_init(void)
{
	int err = 0;

	pr_info("sprd_power_ctl_init\n");
	err = misc_register(&sprd_power_ctl_device);
	if (err)
		pr_err("download power control dev add failed!!!\n");

	sprd_download_dts_init();

	register_pm_notifier(&gnss_pm_notifier);
	init_waitqueue_head(&gnss_dev.gnss_sleep_wait);

	return err;
}

static void __exit sprd_power_ctl_cleanup(void)
{
	misc_deregister(&sprd_power_ctl_device);
}

module_init(sprd_power_ctl_init);
module_exit(sprd_power_ctl_cleanup);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("sprd wcn power ctl driver");

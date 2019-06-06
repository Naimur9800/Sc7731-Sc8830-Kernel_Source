/*
 * FM Radio  driver  with SPREADTRUM SC2331FM Radio chip
 *
 * Copyright (c) 2015 Spreadtrum
 * Author: Songhe Wei <songhe.wei@spreadtrum.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include  <linux/module.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include "fmdrv.h"
#include "fmdrv_main.h"

long fm_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	long ret = 0;
	u32 iarg = 0;

	pr_info("FM_IOCTL cmd: 0x%x.", cmd);
	switch (cmd) {
	case FM_IOCTL_POWERUP:
		pr_info("----power up----");
		fm_powerup();
		break;

	case FM_IOCTL_POWERDOWN:
		pr_info("----power down----");
		fm_powerdown();
		break;

	case FM_IOCTL_TUNE:
		pr_info("----TUNE----");
		fm_tune(argp);
		break;

	case FM_IOCTL_SEEK:
		pr_info("----SEEK----");
		fm_seek(argp);
		break;

	case FM_IOCTL_SETVOL:
		pr_info("----SETVOL----");
		ret = 0;
		break;

	case FM_IOCTL_GETVOL:
		pr_info("----GETVOL----");
		ret = 0;
		break;

	case FM_IOCTL_MUTE:
		pr_info("----MUTE----");
		ret = fm_mute(argp);
		break;

	case FM_IOCTL_GETRSSI:
		pr_info("----GETRSSI----");
		ret = 0;
		break;

	case FM_IOCTL_SCAN:
		pr_info("----SCAN----");
		ret = 0;
		break;

	case FM_IOCTL_STOP_SCAN:
		pr_info("----STOP_SCAN----");
		ret = 0;
		break;

	case FM_IOCTL_GETCHIPID:
		pr_info("----GETCHIPID----");
		iarg = 0x2341;
		if (copy_to_user(argp, &iarg, sizeof(iarg)))
			ret = -EFAULT;
		else
			ret = 0;
		break;

	case FM_IOCTL_EM_TEST:
		pr_info("----EM_TEST----");
		ret = 0;
		break;

	case FM_IOCTL_RW_REG:
		pr_info("----RW_REG----");
		ret = 0;
		break;

	case FM_IOCTL_GETMONOSTERO:
		pr_info("----GETMONOSTERO----");
		ret = 0;
		break;
	case FM_IOCTL_GETCURPAMD:
		pr_info("----GETCURPAMD----");
		ret = 0;
		break;

	case FM_IOCTL_GETGOODBCNT:
		pr_info("----GETGOODBCNT----");
		ret = 0;
		break;

	case FM_IOCTL_GETBADBNT:
		pr_info("----GETBADBNT----");
		ret = 0;
		break;

	case FM_IOCTL_GETBLERRATIO:
		pr_info("----GETBLERRATIO----");
		ret = 0;
		break;

	case FM_IOCTL_RDS_ONOFF:
		pr_info("----RDS_ONOFF----");
		ret = fm_rds_onoff(argp);
		break;

	case FM_IOCTL_RDS_SUPPORT:
		pr_info("----RDS_SUPPORT----");
		ret = 0;
		if (copy_from_user(&iarg, argp, sizeof(iarg))) {
			pr_info("fm RDS support 's ret value is -eFAULT\n");
			return -EFAULT;
		}
		iarg = 1;
		if (copy_to_user(argp, &iarg, sizeof(iarg)))
			ret = -EFAULT;
		break;

	case FM_IOCTL_RDS_SIM_DATA:
		pr_info("----SIM_DATA----");
		ret = 0;
		break;

	case FM_IOCTL_IS_FM_POWERED_UP:
		pr_info("----IS_FM_POWERED_UP----");
		ret = 0;
		break;

	case FM_IOCTL_OVER_BT_ENABLE:
		pr_info("----OVER_BT_ENABLE----");
		ret = 0;
		break;

	case FM_IOCTL_ANA_SWITCH:
		pr_info("----ANA_SWITCH----");
		ret = 0;
		break;

	case FM_IOCTL_GETCAPARRAY:
		pr_info("----GETCAPARRAY----");
		ret = 0;
		break;

	case FM_IOCTL_I2S_SETTING:
		pr_info("----I2S_SETTING----");
		ret = 0;
		break;

	case FM_IOCTL_RDS_GROUPCNT:
		pr_info("----RDS_GROUPCNT----");
		ret = 0;
		break;

	case FM_IOCTL_RDS_GET_LOG:
		pr_info("----GET_LOG----");
		ret = 0;
		break;

	case FM_IOCTL_SCAN_GETRSSI:
		pr_info("----SCAN_GETRSSI----");
		ret = 0;
		break;

	case FM_IOCTL_SETMONOSTERO:
		pr_info("----SETMONOSTERO----");
		ret = 0;
		break;

	case FM_IOCTL_RDS_BC_RST:
		pr_info("----RDS_BC_RST----");
		ret = 0;
		break;

	case FM_IOCTL_CQI_GET:
		pr_info("----CQI_GET----");
		ret = 0;
		break;

	case FM_IOCTL_GET_HW_INFO:
		pr_info("----GET_HW_INFO----");
		ret = 0;
		break;

	case FM_IOCTL_GET_I2S_INFO:
		pr_info("----GET_I2S_INFO----");
		ret = 0;
		break;

	case FM_IOCTL_IS_DESE_CHAN:
		pr_info("----IS_DESE_CHAN----");
		ret = 0;
		break;

	case FM_IOCTL_TOP_RDWR:
		pr_info("----TOP_RDWR----");
		ret = 0;
		break;

	case FM_IOCTL_HOST_RDWR:
		pr_info("----HOST_RDWR----");
		ret = 0;
		break;

	case FM_IOCTL_PRE_SEARCH:
		pr_info("----PRE_SEARCH----");
		ret = 0;
		break;

	case FM_IOCTL_RESTORE_SEARCH:
		pr_info("----RESTORE_SEARCH----");
		ret = 0;
		break;
	case FM_IOCTL_GET_AUDIO_INFO:
		pr_info("----GET_AUDIO_INFO----");
		ret = 0;
		break;

	case FM_IOCTL_SCAN_NEW:
		pr_info("----SCAN_NEW----");
		ret = 0;
		break;

	case FM_IOCTL_SEEK_NEW:
		pr_info("----SEEK_NEW----");
		ret = 0;
		break;

	case FM_IOCTL_TUNE_NEW:
		pr_info("----TUNE_NEW----");
		ret = 0;
		break;

	case FM_IOCTL_SOFT_MUTE_TUNE:
		pr_info("----SOFT_MUTE_TUNE----");
		ret = 0;
		break;

	case FM_IOCTL_DESENSE_CHECK:
		pr_info("----DESENSE_CHECK----");
		ret = 0;
		break;

	case FM_IOCTL_FULL_CQI_LOG:
		pr_info("----FULL_CQI_LOG----");
		ret = 0;
		break;

	case FM_IOCTL_DUMP_REG:
		pr_info("----DUMP_REG----");
		ret = 0;
		break;

	default:
		pr_info("Unknown FM IOCTL!:0x%x", cmd);
		return -EINVAL;
	}

	return ret;
}

int fm_release(struct inode *inode, struct file *filep)
{
	pr_info("fm_misc_release.");
	return 0;
}

#ifdef CONFIG_COMPAT
static long fm_compat_ioctl(struct file *file,
			unsigned int cmd, unsigned long data)
{
	pr_info("start_fm_compat_ioctl FM_IOCTL cmd: 0x%x.", cmd);
	cmd = cmd & 0xFFF0FFFF;
	cmd = cmd | 0x00080000;
	pr_info("fm_compat_ioctl FM_IOCTL cmd: 0x%x.", cmd);
	return fm_ioctl(file, cmd, (unsigned long)compat_ptr(data));
}
#endif

const struct file_operations fm_misc_fops = {
	.owner = THIS_MODULE,
	.open = fm_open,
	.read = fm_read_rds_data,
	.unlocked_ioctl = fm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = fm_compat_ioctl,
#endif
	.release = fm_release,
};

struct miscdevice fm_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = FM_DEV_NAME,
	.fops = &fm_misc_fops,
};

int  fm_device_init_driver(void)
{
	int ret = -EINVAL;
	char *ver_str = FM_VERSION;

	pr_info("**********************************************\n");
	pr_info(" marlin2 FM driver\n ");
	pr_info(" Version: %s\n", ver_str);
	pr_info(" Build date: %s %s\n", __DATE__, __TIME__);
	pr_info("**********************************************\n");

	ret = misc_register(&fm_misc_device);
	if (ret < 0) {
		pr_info("misc_register failed!");
		return ret;
	}

	pr_info("fm_init success.\n");

	return 0;

}

int fm_device_exit_driver(void)
{
	pr_info("exit_fm_driver!\n");
	misc_deregister(&fm_misc_device);
	return 0;
}

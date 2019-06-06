/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
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
 */
#include <linux/bug.h>
#include <linux/gnss.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mfd/sprd/pmic_glb_reg.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>
#include <linux/wcn_integrate_platform.h>

#include "gnss_dump.h"

#define GNSS_MAX_STRING_LEN	10
/* gnss mem dump success return value is 3 */
#define GNSS_DUMP_DATA_SUCCESS	3

struct gnss_common_ctl {
	struct device *dev;
	unsigned long chip_ver;
	unsigned int gnss_status;
	enum wcn_gnss_sub_sys gnss_subsys;
	char firmware_path[FIRMWARE_FILEPATHNAME_LENGTH_MAX];
};

static struct gnss_common_ctl gnss_common_ctl_dev;

enum gnss_status_e {
	GNSS_STATUS_POWEROFF = 0,
	GNSS_STATUS_POWERON,
	GNSS_STATUS_ASSERT,
	GNSS_STATUS_POWEROFF_GOING,
	GNSS_STATUS_POWERON_GOING,
	GNSS_STATUS_MAX,
};

enum gnss_cp_status_subtype {
	GNSS_CP_STATUS_CALI = 1,
	GNSS_CP_STATUS_INIT = 2,
	GNSS_CP_STATUS_INIT_DONE = 3,
	GNSS_CP_STATUS_IDLEOFF = 4,
	GNSS_CP_STATUS_IDLEON = 5,
	GNSS_CP_STATUS_SLEEP = 6,
	GNSS_CP_STATUS__MAX,
};
static struct completion gnss_dump_complete;

static const struct of_device_id gnss_common_ctl_of_match[] = {
	{.compatible = "sprd,gnss_common_ctl", .data = (void *)0x22},
	{},
};

static void gnss_power_on(bool enable)
{
	int ret;

	pr_info("%s:enable=%d,current gnss_status=%d\n", __func__,
			enable, gnss_common_ctl_dev.gnss_status);
	if (enable && gnss_common_ctl_dev.gnss_status == GNSS_STATUS_POWEROFF) {
		gnss_common_ctl_dev.gnss_status = GNSS_STATUS_POWERON_GOING;
		ret = start_marlin(gnss_common_ctl_dev.gnss_subsys);
		if (ret != 0)
			pr_info("%s: start marlin failed ret=%d\n",
					__func__, ret);
		else
			gnss_common_ctl_dev.gnss_status = GNSS_STATUS_POWERON;
	} else if (!enable
			&& gnss_common_ctl_dev.gnss_status
			== GNSS_STATUS_POWERON) {
		gnss_common_ctl_dev.gnss_status = GNSS_STATUS_POWEROFF_GOING;
		ret = stop_marlin(gnss_common_ctl_dev.gnss_subsys);
		if (ret != 0)
			pr_info("%s: stop marlin failed ret=%d\n",
				 __func__, ret);
		else
			gnss_common_ctl_dev.gnss_status = GNSS_STATUS_POWEROFF;
	} else {
		pr_info("%s: status is not match\n", __func__);
	}
}

static ssize_t gnss_power_enable_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long set_value;

	if (kstrtoul(buf, GNSS_MAX_STRING_LEN, &set_value)) {
		pr_err("%s, Maybe store string is too long\n", __func__);
		return -EINVAL;
	}
	pr_info("%s,%lu\n", __func__, set_value);
	if (set_value == 1) {
		gnss_power_on(1);
	} else if (set_value == 0) {
		gnss_power_on(0);
	} else {
		count = -EINVAL;
		pr_info("%s,unknown control\n", __func__);
	}

	return count;
}

static DEVICE_ATTR_WO(gnss_power_enable);

static ssize_t gnss_subsys_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long set_value;

	if (kstrtoul(buf, GNSS_MAX_STRING_LEN, &set_value))
		return -EINVAL;

	pr_info("%s,%lu\n", __func__, set_value);
	if (set_value == WCN_GNSS)
		gnss_common_ctl_dev.gnss_subsys = WCN_GNSS;
	else if (set_value == WCN_GNSS_BD)
		gnss_common_ctl_dev.gnss_subsys  = WCN_GNSS_BD;
	else
		count = -EINVAL;

	return count;
}

void gnss_file_path_set(char *buf)
{
	strcpy(&gnss_common_ctl_dev.firmware_path[0], buf);
}

static ssize_t gnss_subsys_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int i = 0;

	pr_info("%s\n", __func__);
	if (gnss_common_ctl_dev.gnss_status == GNSS_STATUS_POWERON) {
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d:%s\n",
				gnss_common_ctl_dev.gnss_subsys,
				&gnss_common_ctl_dev.firmware_path[0]);
	} else {
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				gnss_common_ctl_dev.gnss_subsys);
	}

	return i;
}

static DEVICE_ATTR_RW(gnss_subsys);

static int gnss_status_get(void)
{
	phys_addr_t phy_addr;
	u32 magic_value;

	phy_addr = wcn_get_gnss_base_addr() + GNSS_STATUS_OFFSET;
	wcn_read_data_from_phy_addr(phy_addr, &magic_value, sizeof(u32));
	pr_info("[%s] magic_value=%d\n", __func__, magic_value);

	return magic_value;
}

void gnss_dump_mem_ctrl_co(void)
{
	char flag = 0; /* 0: default, all, 1: only data, pmu, aon */
	unsigned int temp_status = 0;
	static char dump_flag;

	pr_info("[%s], flag is %d\n", __func__, dump_flag);
	if (dump_flag == 1)
		return;
	dump_flag = 1;
	temp_status = gnss_common_ctl_dev.gnss_status;
	if ((temp_status == GNSS_STATUS_POWERON_GOING) ||
		((temp_status == GNSS_STATUS_POWERON) &&
		(gnss_status_get() != GNSS_CP_STATUS_SLEEP))) {
		flag = (temp_status == GNSS_STATUS_POWERON) ? 0 : 1;
		gnss_dump_mem(flag);
		gnss_common_ctl_dev.gnss_status = GNSS_STATUS_ASSERT;
	}
	complete(&gnss_dump_complete);
}

static ssize_t gnss_dump_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long set_value;
	int ret = -1;
	int waitret = 0;

	if (kstrtoul(buf, GNSS_MAX_STRING_LEN, &set_value)) {
		pr_err("%s, store string is too long\n", __func__);
		return -EINVAL;
	}
	pr_info("%s enter set %lu\n", __func__, set_value);
	if (set_value == 1) {
		waitret = wait_for_completion_timeout(
			&gnss_dump_complete, msecs_to_jiffies(7000));
		pr_info("%s exit %d\n", __func__, jiffies_to_msecs(waitret));
		if (waitret > 0)
			ret = GNSS_DUMP_DATA_SUCCESS;
		else
			gnss_dump_mem_ctrl_co();
	} else
		count = -EINVAL;

	return ret;
}

static DEVICE_ATTR_WO(gnss_dump);

static ssize_t gnss_status_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int i = 0;

	pr_info("%s\n", __func__);

	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			gnss_common_ctl_dev.gnss_status);
	return i;
}
static DEVICE_ATTR_RO(gnss_status);

bool gnss_delay_ctl(void)
{
	return (gnss_common_ctl_dev.gnss_status == GNSS_STATUS_POWERON);
}

static struct attribute *gnss_common_ctl_attrs[] = {
	&dev_attr_gnss_power_enable.attr,
	&dev_attr_gnss_dump.attr,
	&dev_attr_gnss_status.attr,
	&dev_attr_gnss_subsys.attr,
	NULL,
};

static struct attribute_group gnss_common_ctl_group = {
	.name = NULL,
	.attrs = gnss_common_ctl_attrs,
};

static struct miscdevice gnss_common_ctl_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gnss_common_ctl",
	.fops = NULL,
};

static int gnss_common_ctl_probe(struct platform_device *pdev)
{
	int ret;
	const struct of_device_id *of_id;


	gnss_common_ctl_dev.dev = &pdev->dev;

	gnss_common_ctl_dev.gnss_status = GNSS_STATUS_POWEROFF;
	gnss_common_ctl_dev.gnss_subsys = WCN_GNSS;

	/* considering backward compatibility, it's not use now  start */
	of_id = of_match_node(gnss_common_ctl_of_match,
		pdev->dev.of_node);
	if (!of_id) {
		dev_err(&pdev->dev, "get gnss_common_ctl of device id failed!\n");
		return -ENODEV;
	}
	gnss_common_ctl_dev.chip_ver = (unsigned long)(of_id->data);
	/* considering backward compatibility, it's not use now  end */

	platform_set_drvdata(pdev, &gnss_common_ctl_dev);
	ret = misc_register(&gnss_common_ctl_miscdev);
	if (ret) {
		pr_err("%s failed to register gnss_common_ctl.\n", __func__);
		return ret;
	}

	ret = sysfs_create_group(&gnss_common_ctl_miscdev.this_device->kobj,
			&gnss_common_ctl_group);
	if (ret) {
		pr_err("%s failed to create device attributes.\n", __func__);
		goto err_attr_failed;
	}

	/* register dump callback func for mdbg */
	mdbg_dump_gnss_register(gnss_dump_mem_ctrl_co, NULL);
	init_completion(&gnss_dump_complete);

	return 0;

err_attr_failed:
	misc_deregister(&gnss_common_ctl_miscdev);

	return ret;
}

static int gnss_common_ctl_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&gnss_common_ctl_miscdev.this_device->kobj,
				&gnss_common_ctl_group);

	misc_deregister(&gnss_common_ctl_miscdev);
	return 0;
}
static struct platform_driver gnss_common_ctl_drv = {
	.driver = {
		   .name = "gnss_common_ctl",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(gnss_common_ctl_of_match),
		   },
	.probe = gnss_common_ctl_probe,
	.remove = gnss_common_ctl_remove
};

static int __init gnss_common_ctl_init(void)
{
	return platform_driver_register(&gnss_common_ctl_drv);
}

static void __exit gnss_common_ctl_exit(void)
{
	platform_driver_unregister(&gnss_common_ctl_drv);
}

module_init(gnss_common_ctl_init);
module_exit(gnss_common_ctl_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Spreadtrum Gnss Driver");
MODULE_AUTHOR("Jun.an<jun.an@spreadtrum.com>");

/*
 * Copyright (C) 2014 Spreadtrum Communications Inc.
 *
 * Author: Haibing.Yang <haibing.yang@spreadtrum.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)		"sprd-mipi-log: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include "sprd_mipi_log.h"

#define SPRD_DEVICE_ATTR(name)  \
	static ssize_t name##_show(struct device *dev,\
			       struct device_attribute *attr, char *buf);\
	static ssize_t name##_store(struct device *dev,\
				  struct device_attribute *attr,\
				  const char *buf, size_t count); \
	static DEVICE_ATTR(name, S_IRUGO | S_IWUSR, name##_show, name##_store)

SPRD_DEVICE_ATTR(channel);
SPRD_DEVICE_ATTR(freq);

static ssize_t channel_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct mipi_log_device *mipi_log = dev_get_drvdata(dev);
	int ret;

	ret = snprintf(buf, PAGE_SIZE,
			"current value: %u\n", mipi_log->channel);
	return ret;
}

static ssize_t channel_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mipi_log_device *mipi_log = dev_get_drvdata(dev);
	int ret;
	u32 channel;

	ret = kstrtouint(buf, 10, &channel);
	if (ret) {
		pr_err("error: Invalid input!\n");
		return -EINVAL;
	}

	if (channel >= CH_EN_MAX) {
		pr_err("error: Channel not support!\n");
		return -EINVAL;
	}

	if (channel != mipi_log->channel) {
		if (channel)
			mipi_log_start(mipi_log, channel);
		else
			mipi_log_stop(mipi_log);
		mipi_log->channel = channel;
	}

	return count;
}

static ssize_t freq_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int ret = 0;
	struct mipi_log_device *mipi_log = dev_get_drvdata(dev);

	ret = snprintf(buf, PAGE_SIZE,
			"current mipi frequency: %u\n",
			mipi_log->phy_freq);
	return ret;
}

static ssize_t freq_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	int ret = 0;
	u32 dphy_freq;
	struct mipi_log_device *mipi_log = dev_get_drvdata(dev);

	if (!mipi_log)
		return -EINVAL;

	ret = kstrtouint(buf, 10, &dphy_freq);
	if (ret) {
		pr_err("Invalid input for dphy_freq\n");
		return -EINVAL;
	}

	pr_info("input dphy_freq is %d\n", dphy_freq);

	if (dphy_freq <= 90000 || dphy_freq >= 3000000) {
		pr_err("input mipi frequency is out of range.\n");
		return -EINVAL;
	}

	if (dphy_freq >= 90000 && dphy_freq <= 1100000)
		dphy_freq = 1000000;
	else if (dphy_freq >= 1100000 && dphy_freq <= 1250000)
		dphy_freq = 1200000;
	else if (dphy_freq >= 1250000 && dphy_freq <= 1350000)
		dphy_freq = 1300000;
	else if (dphy_freq >= 1300000 && dphy_freq <= 1450000)
		dphy_freq = 1400000;
	else
		dphy_freq = 1500000;

	if (dphy_freq == mipi_log->phy_freq) {
		/* Do nothing */
		pr_info("new dphy_freq is the same as current freq\n");
		return count;
	}

	mipi_log->phy_freq = dphy_freq;

	if (mipi_log->running) {
		mipi_log_stop(mipi_log);
		mipi_log_start(mipi_log, mipi_log->channel);
	}

	return count;
}

static struct attribute *attrs[] = {
	&dev_attr_channel.attr,
	&dev_attr_freq.attr,
	NULL,
};

static struct attribute_group attrs_group = {
	.attrs = attrs,
};

int mipi_log_create_sysfs(struct mipi_log_device *mipi_log)
{
	int rc = 0;

	rc = sysfs_create_group(&mipi_log->dev->kobj, &attrs_group);
	if (rc)
		pr_err("sysfs group creation failed, rc=%d\n", rc);

	return rc;
}

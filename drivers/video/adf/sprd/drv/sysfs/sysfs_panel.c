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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>

#include "disp_lib.h"
#include "sprd_panel.h"
#include "sysfs_display.h"

static ssize_t name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct panel_device *pd = dev_get_drvdata(dev);
	struct panel_info *panel = pd->panel;

	if (!panel) {
		pr_err("panel doesn't exist.");
		return -ENXIO;
	}
	ret = snprintf(buf, PAGE_SIZE, "%s\n", panel->lcd_name);

	return ret;
}
static DEVICE_ATTR_RO(name);

static ssize_t lane_num_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct panel_device *pd = dev_get_drvdata(dev);
	struct panel_info *panel = pd->panel;

	if (!panel) {
		pr_err("panel doesn't exist.");
		return -ENXIO;
	}
	ret = snprintf(buf, PAGE_SIZE, "%u\n", panel->lane_num);

	return ret;
}
static DEVICE_ATTR_RO(lane_num);

static ssize_t phy_freq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct panel_device *pd = dev_get_drvdata(dev);
	struct panel_info *panel = pd->panel;

	if (!panel) {
		pr_err("panel doesn't exist.");
		return -ENXIO;
	}
	ret = snprintf(buf, PAGE_SIZE, "%u\n", panel->phy_freq);

	return ret;
}
static DEVICE_ATTR_RO(phy_freq);

static ssize_t resolution_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct panel_device *pd = dev_get_drvdata(dev);
	struct panel_info *panel = pd->panel;

	if (!panel) {
		pr_err("panel doesn't exist.");
		return -ENXIO;
	}
	ret = snprintf(buf, PAGE_SIZE, "%ux%u\n", panel->width,
						panel->height);

	return ret;
}
static DEVICE_ATTR_RO(resolution);

static ssize_t screen_size_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct panel_device *pd = dev_get_drvdata(dev);
	struct panel_info *panel = pd->panel;

	if (!panel) {
		pr_err("panel doesn't exist.");
		return -ENXIO;
	}
	ret = snprintf(buf, PAGE_SIZE, "%umm x %umm\n", panel->width_mm,
						panel->height_mm);

	return ret;
}
static DEVICE_ATTR_RO(screen_size);

static ssize_t hporch_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct panel_device *pd = dev_get_drvdata(dev);
	struct panel_info *panel = pd->panel;
	int ret;

	ret = snprintf(buf, PAGE_SIZE, "hfp=%u hbp=%u hsync=%u\n",
					panel->rgb_timing.hfp,
					panel->rgb_timing.hbp,
					panel->rgb_timing.hsync);

	return ret;
}

static ssize_t hporch_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count) {
	struct panel_device *pd = dev_get_drvdata(dev);
	struct panel_info *panel = pd->panel;
	u32 val[4] = {0};
	int len;

	len = str_to_u32_array(buf, 0, val);

	switch (len) {
	/* Fall through */
	case 3:
		panel->rgb_timing.hsync = val[2];
	/* Fall through */
	case 2:
		panel->rgb_timing.hbp = val[1];
	/* Fall through */
	case 1:
		panel->rgb_timing.hfp = val[0];
	/* Fall through */
	default:
		break;
	}

	return count;
}
static DEVICE_ATTR_RW(hporch);

static ssize_t vporch_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct panel_device *pd = dev_get_drvdata(dev);
	struct panel_info *panel = pd->panel;
	int ret;

	ret = snprintf(buf, PAGE_SIZE, "vfp=%u vbp=%u vsync=%u\n",
					panel->rgb_timing.vfp,
					panel->rgb_timing.vbp,
					panel->rgb_timing.vsync);

	return ret;
}

static ssize_t vporch_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count) {
	struct panel_device *pd = dev_get_drvdata(dev);
	struct panel_info *panel = pd->panel;
	u32 val[4] = {0};
	int len;

	len = str_to_u32_array(buf, 0, val);

	switch (len) {
	/* Fall through */
	case 3:
		panel->rgb_timing.vsync = val[2];
	/* Fall through */
	case 2:
		panel->rgb_timing.vbp = val[1];
	/* Fall through */
	case 1:
		panel->rgb_timing.vfp = val[0];
	/* Fall through */
	default:
		break;
	}

	return count;
}
static DEVICE_ATTR_RW(vporch);

static ssize_t esd_check_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct panel_device *pd = dev_get_drvdata(dev);
	int ret;

	ret = snprintf(buf, PAGE_SIZE,
			"current esd: %u\n", pd->panel->esd_check_en);
	return ret;
}

static ssize_t esd_check_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count) {
	struct panel_device *pd = dev_get_drvdata(dev);
	struct panel_info *panel = pd->panel;
	int ret;
	int enable;

	ret = kstrtoint(buf, 10, &enable);
	if (ret) {
		pr_err("Invalid input for esd check\n");
		return -EINVAL;
	}

	if (enable) {
		panel->esd_check_en = enable;
		if (!pm_runtime_suspended(dev->parent) &&
			!pd->esd_work_start) {
			pr_info("schedule ESD work queue!\n");
			schedule_delayed_work(&pd->esd_work,
					      msecs_to_jiffies
					      (panel->esd_timeout));
			pd->esd_work_start = true;
		}
	} else {
		if (pd->esd_work_start) {
			pr_info("cancel ESD work queue!\n");
			cancel_delayed_work_sync(&pd->esd_work);
			pd->esd_work_start = false;
		}
		panel->esd_check_en = false;
	}

	return count;

}
static DEVICE_ATTR_RW(esd_check);

static ssize_t esd_reg_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct panel_device *pd = dev_get_drvdata(dev);
	struct panel_info *panel = pd->panel;
	int ret;

	ret = snprintf(buf, PAGE_SIZE, "0x%02x\n", panel->esd_reg);
	return ret;
}

static ssize_t esd_reg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count) {
	struct panel_device *pd = dev_get_drvdata(dev);
	struct panel_info *panel = pd->panel;
	int ret;
	uint32_t val;

	ret = kstrtouint(buf, 16, &val);
	if (ret) {
		pr_err("Invalid input for esd reg addr\n");
		return -EINVAL;
	}

	panel->esd_reg = val;

	return count;

}
static DEVICE_ATTR_RW(esd_reg);

static ssize_t esd_time_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct panel_device *pd = dev_get_drvdata(dev);
	struct panel_info *panel = pd->panel;
	int ret;

	ret = snprintf(buf, PAGE_SIZE, "%u\n", panel->esd_timeout);
	return ret;
}

static ssize_t esd_time_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count) {
	struct panel_device *pd = dev_get_drvdata(dev);
	struct panel_info *panel = pd->panel;
	int ret;
	uint32_t val;

	ret = kstrtouint(buf, 10, &val);
	if (ret) {
		pr_err("Invalid input for esd time cycle\n");
		return -EINVAL;
	}

	panel->esd_timeout = val;

	return count;

}
static DEVICE_ATTR_RW(esd_time);

static ssize_t esd_return_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct panel_device *pd = dev_get_drvdata(dev);
	struct panel_info *panel = pd->panel;
	int ret = 0;
	int i;

	for (i = 0; i < panel->esd_read_count; i++)
		ret += snprintf(buf, PAGE_SIZE, "0x%02x\n",
				panel->esd_return_code[i]);

	return ret;
}

static ssize_t esd_return_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count) {
	struct panel_device *pd = dev_get_drvdata(dev);
	struct panel_info *panel = pd->panel;
	uint32_t val[4] = { 0 };
	int len;
	int i;

	len = str_to_u32_array(buf, 16, val);

	panel->esd_read_count = len;

	for (i = 0; i < len; i++)
		panel->esd_return_code[i] = val[i];

	return count;

}
static DEVICE_ATTR_RW(esd_return);

static ssize_t power_ctrl_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count) {
	struct panel_device *pd = dev_get_drvdata(dev);
	int enable = 0;
	int ret;

	ret = kstrtoint(buf, 10, &enable);
	if (ret) {
		pr_err("invalid input\n");
		return -EINVAL;
	}

	if (enable)
		panel_power_ctrl(&pd->panel->pwr_on_seq);
	else
		panel_power_ctrl(&pd->panel->pwr_off_seq);

	return count;
}
static DEVICE_ATTR_WO(power_ctrl);

static ssize_t send_cmd_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count) {
	struct panel_device *pd = dev_get_drvdata(dev);
	struct panel_ops *ops = pd->ops;

	if (!ops->send_cmd) {
		pr_err("ops->send_cmd() was not implemented\n");
		return -EINVAL;
	}

	if (!strncmp(buf, "init", strlen("init")))
		ops->send_cmd(pd, CMD_CODE_INIT);
	else if (!strncmp(buf, "sleep-in", strlen("sleep-in")))
		ops->send_cmd(pd, CMD_CODE_SLEEP_IN);
	else if (!strncmp(buf, "sleep-out", strlen("sleep-out")))
		ops->send_cmd(pd, CMD_CODE_SLEEP_OUT);
	else if (!strncmp(buf, "reserved0", strlen("reserved0")))
		ops->send_cmd(pd, CMD_CODE_RESERVED0);
	else if (!strncmp(buf, "reserved1", strlen("reserved1")))
		ops->send_cmd(pd, CMD_CODE_RESERVED1);
	else if (!strncmp(buf, "reserved2", strlen("reserved2")))
		ops->send_cmd(pd, CMD_CODE_RESERVED2);
	else
		pr_err("input cmd was not supported\n");

	return count;
}
static DEVICE_ATTR_WO(send_cmd);

static ssize_t suspend_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count) {
	pm_runtime_put_sync(dev->parent);
	return count;
}
static DEVICE_ATTR_WO(suspend);

static ssize_t resume_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count) {
	pm_runtime_get_sync(dev->parent);
	return count;
}
static DEVICE_ATTR_WO(resume);

static struct attribute *panel_attrs[] = {
	&dev_attr_name.attr,
	&dev_attr_lane_num.attr,
	&dev_attr_phy_freq.attr,
	&dev_attr_resolution.attr,
	&dev_attr_screen_size.attr,
	&dev_attr_hporch.attr,
	&dev_attr_vporch.attr,
	&dev_attr_esd_check.attr,
	&dev_attr_esd_reg.attr,
	&dev_attr_esd_time.attr,
	&dev_attr_esd_return.attr,
	&dev_attr_power_ctrl.attr,
	&dev_attr_send_cmd.attr,
	&dev_attr_suspend.attr,
	&dev_attr_resume.attr,
	NULL,
};
ATTRIBUTE_GROUPS(panel);

int sprd_panel_sysfs_init(struct device *dev)
{
	int rc;

	rc = sysfs_create_groups(&(dev->kobj), panel_groups);
	if (rc)
		pr_err("create panel attr node failed, rc=%d\n", rc);

	return rc;
}
EXPORT_SYMBOL(sprd_panel_sysfs_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("leon.he@spreadtrum.com");
MODULE_DESCRIPTION("Add panel attribute nodes for userspace");

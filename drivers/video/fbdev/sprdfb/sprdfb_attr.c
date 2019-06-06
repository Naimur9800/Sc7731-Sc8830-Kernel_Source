/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
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

#define pr_fmt(fmt)		"sprdfb: " fmt

#include <linux/fb.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include "sprdfb.h"
#include "sprdfb_panel.h"

struct attr_info {
	struct sprd_fbdev *fb_dev;
	/* clock management */
	u32 origin_pclk;
	u32 curr_pclk;
	u32 origin_fps;
	u32 curr_fps;
	u32 origin_mipi_clk;
	u32 curr_mipi_clk;
	struct semaphore sem;
};

static ssize_t sysfs_rd_current_pclk(struct device *dev,
			struct device_attribute *attr, char *buf);
static ssize_t sysfs_write_pclk(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count);
static ssize_t sysfs_rd_current_fps(struct device *dev,
			struct device_attribute *attr, char *buf);
static ssize_t sysfs_write_fps(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count);
static ssize_t sysfs_rd_current_mipi_clk(struct device *dev,
			struct device_attribute *attr, char *buf);
static ssize_t sysfs_write_mipi_clk(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count);
static ssize_t sysfs_rd_current_frame_count(struct device *dev,
			struct device_attribute *attr, char *buf);

#ifdef CONFIG_FB_ESD_SUPPORT
static ssize_t sysfs_rd_current_esd(struct device *dev,
			struct device_attribute *attr, char *buf);
static ssize_t sysfs_write_esd(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count);
#endif

static DEVICE_ATTR(dynamic_pclk, S_IRUGO | S_IWUSR, sysfs_rd_current_pclk,
			sysfs_write_pclk);
static DEVICE_ATTR(dynamic_fps, S_IRUGO | S_IWUSR, sysfs_rd_current_fps,
			sysfs_write_fps);
static DEVICE_ATTR(dynamic_mipi_clk, S_IRUGO | S_IWUSR,
			sysfs_rd_current_mipi_clk, sysfs_write_mipi_clk);
static DEVICE_ATTR(dynamic_frame_count, S_IRUGO,
			sysfs_rd_current_frame_count, NULL);

#ifdef CONFIG_FB_ESD_SUPPORT
static DEVICE_ATTR(dynamic_esd, S_IRUGO | S_IWUSR,
				sysfs_rd_current_esd, sysfs_write_esd);
#endif

static struct attribute *sprd_fb_fs_attrs[] = {
	&dev_attr_dynamic_pclk.attr,
	&dev_attr_dynamic_fps.attr,
	&dev_attr_dynamic_mipi_clk.attr,
	&dev_attr_dynamic_frame_count.attr,

#ifdef CONFIG_FB_ESD_SUPPORT
	&dev_attr_dynamic_esd.attr,
#endif
	NULL,
};

static struct attribute_group sprd_fb_attrs_group = {
	.attrs = sprd_fb_fs_attrs,
};

/**
 * sprd_fb_chg_clk_intf - sprd fb change clock interface
 *
 * @fb_dev - sprd fb specific device
 * @type: SPRD_FB_DYNAMIC_PCLK, SPRD_FB_DYNAMIC_FPS, SPRD_FB_DYNAMIC_MIPI_CLK
 * @new_val: new required fps, dpi clock or mipi clock
 *
 * If clk is set unsuccessfully, this function returns minus.
 * It returns 0 if clk is set successfully.
 */
int sprd_fb_chg_clk_intf(struct sprd_fbdev *fb_dev, int type, u32 new_val)
{
	int ret;
	struct attr_info *attr = fb_dev->priv1;

	down(&attr->sem);
	ret = sprd_dispc_chg_clk(fb_dev, type, new_val);
	up(&attr->sem);

	return ret;
}

static ssize_t sysfs_rd_current_pclk(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int ret;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct sprd_fbdev *fb_dev = (struct sprd_fbdev *)fbi->par;
	struct attr_info *attr_info = fb_dev->priv1;

	if (!fb_dev) {
		pr_err("fb_dev can't be found\n");
		return -ENXIO;
	}
	ret = snprintf(buf, PAGE_SIZE,
		       "current dpi_clk: %u\nnew dpi_clk: %u\norigin dpi_clk: %u\n",
		       fb_dev->dpi_clock, attr_info->curr_pclk,
		       attr_info->origin_pclk);

	return ret;
}

static ssize_t sysfs_write_pclk(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	int divider;
	u32 dpi_clk_src, new_pclk;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct sprd_fbdev *fb_dev = (struct sprd_fbdev *)fbi->par;
	struct attr_info *attr_info = fb_dev->priv1;

	ret = sscanf(buf, "%u,%d\n", &dpi_clk_src, &divider);
	if (divider == 0 && dpi_clk_src == 0) {
		new_pclk = attr_info->origin_pclk;
		goto DIRECT_GO;
	}

	if (divider < 1 || divider > 0xff || dpi_clk_src < 1) {
		pr_err("divider:[%d], clk_src:[%d] is invalid\n",
		       divider, dpi_clk_src);
		return count;
	}

	if (dpi_clk_src < 1000)
		dpi_clk_src *= 1000000;	/* MHz */

	new_pclk = dpi_clk_src / divider;

DIRECT_GO:
	if (new_pclk == fb_dev->dpi_clock) {
		/* Do nothing */
		pr_warn("new pclk is the same as current pclk\n");
		return count;
	}

	ret = sprd_fb_chg_clk_intf(fb_dev, SPRD_FB_DYNAMIC_PCLK, new_pclk);
	if (ret) {
		pr_err("failed to change dpi clk. ret=%d\n", ret);
		return ret;
	}
	attr_info->curr_pclk = new_pclk;

	return count;
}

static ssize_t sysfs_rd_current_fps(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int ret;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct sprd_fbdev *fb_dev = (struct sprd_fbdev *)fbi->par;
	struct attr_info *attr_info = fb_dev->priv1;
	struct panel_spec *panel = fb_dev->panel;

	if (!panel) {
		pr_err("[%s]: panel doesn't exist.", __func__);
		return -ENXIO;
	}
	ret = snprintf(buf, PAGE_SIZE,
		       "current fps: %u\nnew fps: %u\norigin fps: %u\n",
		       panel->fps, attr_info->curr_fps, attr_info->origin_fps);

	return ret;
}

static ssize_t sysfs_write_fps(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int ret, fps;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct sprd_fbdev *fb_dev = (struct sprd_fbdev *)fbi->par;
	struct attr_info *attr_info = fb_dev->priv1;
	struct panel_spec *panel = fb_dev->panel;

	if (!panel) {
		pr_err("[%s]: panel doesn't exist.", __func__);
		return -ENXIO;
	}
	ret = kstrtoint(buf, 10, &fps);

	fps = fps > 0 ? fps : attr_info->origin_fps;

	if (panel->fps == fps) {
		/* Do nothing */
		pr_warn("new fps is the same as current fps\n");
		return count;
	}

	ret = sprd_fb_chg_clk_intf(fb_dev, SPRD_FB_DYNAMIC_FPS, (u32) fps);
	if (ret) {
		pr_err("[%s]: failed to change dpi clock. ret=%d\n",
		       __func__, ret);
		return ret;
	}
	attr_info->curr_fps = fps;

	return count;
}

static ssize_t sysfs_rd_current_mipi_clk(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int ret = 0;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct sprd_fbdev *fb_dev = (struct sprd_fbdev *)fbi->par;
	struct attr_info *attr_info = fb_dev->priv1;
	struct panel_spec *panel = fb_dev->panel;

	if (!panel) {
		pr_err("[%s]: panel doesn't exist.", __func__);
		return -ENXIO;
	}
	if (panel->type != LCD_MODE_DSI) {
		pr_warn("Current panel is not mipi dsi\n");
		ret = snprintf(buf, PAGE_SIZE,
			       "Current panel is not mipi dsi\n");
		return ret;
	}

	ret = snprintf(buf, PAGE_SIZE,
		       "current mipi d-phy frequency: %u\n"
		       "new mipi d-phy frequency: %u\n"
		       "origin mipi d-phy frequency: %u\n",
		       panel->info.mipi->phy_feq,
		       attr_info->curr_mipi_clk, attr_info->origin_mipi_clk);

	return ret;
}

static ssize_t sysfs_write_mipi_clk(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int ret = 0;
	u32 dphy_freq;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct sprd_fbdev *fb_dev = (struct sprd_fbdev *)fbi->par;
	struct attr_info *attr_info = fb_dev->priv1;
	struct panel_spec *panel = fb_dev->panel;

	if (!panel) {
		pr_err("[%s]: panel doesn't exist.", __func__);
		return -ENXIO;
	}
	if (panel->type != LCD_MODE_DSI) {
		pr_warn("sys write failure. Current panel is not mipi dsi\n");
		return count;
	}
	ret = kstrtoint(buf, 10, &dphy_freq);
	if (ret) {
		pr_err("Invalid input for dphy_freq\n");
		return -EINVAL;
	}

	if (dphy_freq > 0) {
		/* Because of double edge trigger, the rule is */
		/* actual freq * 10 / 2, Eg: Required freq is 500M */
		/* Equation: 2500*2*1000/10=500*1000=2500*200=500M */
		dphy_freq *= 200;
	} else
		dphy_freq = attr_info->origin_mipi_clk;

	/* dphy supported freq ranges is 90M-1500M */
	pr_debug("input dphy_freq is %d\n", dphy_freq);
	if (dphy_freq == attr_info->curr_mipi_clk) {
		/* Do nothing */
		pr_warn("new dphy_freq is the same as current freq\n");
		return count;
	}

	if (dphy_freq <= 1500000 && dphy_freq >= 90000) {
		ret =
		    sprd_fb_chg_clk_intf(fb_dev, SPRD_FB_DYNAMIC_MIPI_CLK,
					dphy_freq);
		if (ret) {
			pr_err("sprd_fb_chg_clk_intf change d-phy freq fail.\n");
			return count;
		}
		attr_info->curr_mipi_clk = dphy_freq;
	} else {
		pr_warn("input mipi frequency:%d is out of range.\n",
			dphy_freq);
	}

	return count;
}

static ssize_t sysfs_rd_current_frame_count(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	int ret;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct sprd_fbdev *fb_dev = (struct sprd_fbdev *)fbi->par;

	if (!fb_dev) {
		pr_err("[%s]: fb_dev can't be found\n", __func__);
		return -ENXIO;
	}
	ret = snprintf(buf, PAGE_SIZE,
		       "current frame_count: %lld\n", fb_dev->frame_count);

	return ret;
}

#ifdef CONFIG_FB_ESD_SUPPORT
static ssize_t sysfs_rd_current_esd(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int ret;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct sprd_fbdev *fb_dev = (struct sprd_fbdev *)fbi->par;

	if (!fb_dev) {
		pr_err("[%s]: fb_dev can't be found\n", __func__);
		return -ENXIO;
	}
	ret = snprintf(buf, PAGE_SIZE,
		       "current esd: %u\n", fb_dev->ESD_work_start);

	return ret;
}

static ssize_t sysfs_write_esd(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int ret, esd;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct sprd_fbdev *fb_dev = (struct sprd_fbdev *)fbi->par;

	ret = kstrtoint(buf, 10, &esd);
	esd = (esd == 1) ? 1 : 0;
	if ((1 == esd) && (fb_dev->enable == 1)) {
		if (!fb_dev->ESD_work_start) {
			pr_info("schedule ESD work queue!\n");
			schedule_delayed_work(&fb_dev->ESD_work,
				msecs_to_jiffies(fb_dev->ESD_timeout_val));
			fb_dev->ESD_work_start = true;
		}
	} else {
		if (fb_dev->ESD_work_start == true) {
			pr_info("cancel ESD work queue\n");
			cancel_delayed_work_sync(&fb_dev->ESD_work);
			fb_dev->ESD_work_start = false;
		}
	}

	return count;
}
#endif

int sprd_fb_create_sysfs(struct sprd_fbdev *fb_dev)
{
	int rc;
	struct panel_spec *panel = fb_dev->panel;
	struct attr_info *attr;

	fb_dev->priv1 = kzalloc(sizeof(struct attr_info), GFP_KERNEL);
	if (!fb_dev->priv1)
		return -ENOMEM;
	attr = fb_dev->priv1;

	rc = sysfs_create_group(&fb_dev->fb->dev->kobj, &sprd_fb_attrs_group);
	if (rc)
		pr_err("sysfs group creation failed, rc=%d\n", rc);

	attr->origin_pclk = fb_dev->dpi_clock;
	if (panel) {
		attr->origin_fps = panel->fps;
		if (panel->type == LCD_MODE_DSI)
			attr->origin_mipi_clk = panel->info.mipi->phy_feq;
	}
	sema_init(&attr->sem, 1);

	return rc;
}

void sprd_fb_remove_sysfs(struct sprd_fbdev *fb_dev)
{
	struct attr_info *attr = fb_dev->priv1;

	sysfs_remove_group(&fb_dev->fb->dev->kobj, &sprd_fb_attrs_group);
	devm_kfree(fb_dev->of_dev, attr);
}

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

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>

#include "sprd_dispc.h"
#include "sprd_panel.h"
#include "sysfs_display.h"

static uint32_t bg_color;
static uint32_t irq_status;

static ssize_t run_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct sprd_dispc *dispc = dev_get_drvdata(dev);

	ret = snprintf(buf, PAGE_SIZE, "%d\n", !dispc->ctx.is_stopped);

	return ret;
}

static ssize_t run_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int ret;
	int enable;
	struct sprd_dispc *dispc = dev_get_drvdata(dev);

	ret = kstrtoint(buf, 10, &enable);
	if (ret) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	if (enable)
		sprd_dispc_run(dispc);
	else
		sprd_dispc_stop(dispc);

	return count;
}
static DEVICE_ATTR_RW(run);

static ssize_t refresh_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int ret;
	uint32_t value;
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct dispc_context *ctx = &dispc->ctx;


	ctx->disable_flip = false;
	ret = kstrtou32(buf, 10, &value);
	if (ret) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	if (value) {
		down(&ctx->refresh_lock);

		if (!ctx->is_inited) {
			pr_err("dispc is not initialized!\n");
			up(&ctx->refresh_lock);
			return -1;
		}

		dispc->core->flip(ctx, config_saved);

		if (!ctx->is_stopped ||
			(dispc->ctx.if_type == SPRD_DISPC_IF_EDPI))
			dispc->core->run(ctx);
		up(&ctx->refresh_lock);
	}

	return count;
}
static DEVICE_ATTR_WO(refresh);

static ssize_t bg_color_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, PAGE_SIZE, "%x\n", bg_color);

	return ret;
}

static ssize_t bg_color_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int ret;
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct dispc_context *ctx = &dispc->ctx;

	if (!dispc->core->bg_color)
		return -EIO;

	ret = kstrtou32(buf, 16, &bg_color);
	if (ret) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	down(&ctx->refresh_lock);
	if (!ctx->is_inited) {
		pr_err("dispc is not initialized\n");
		up(&ctx->refresh_lock);
		return -EINVAL;
	}

	ctx->disable_flip = true;
	dispc->core->bg_color(ctx, bg_color);
	dispc->core->run(ctx);
	up(&ctx->refresh_lock);

	return count;
}
static DEVICE_ATTR_RW(bg_color);

static ssize_t actual_fps_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct panel_info *panel = dispc->ctx.panel;
	uint32_t act_fps_int, act_fps_frac;
	uint32_t total_pixels;
	struct rgb_timing *t;
	int ret;

	if (!panel) {
		pr_err("panel doesn't exist.");
		return -ENXIO;
	}

	t = &panel->rgb_timing;

	total_pixels = (t->hsync + t->hbp + t->hfp + panel->width) *
			(t->vsync + t->vbp + t->vfp + panel->height);

	act_fps_int = panel->pixel_clk / total_pixels;
	act_fps_frac = panel->pixel_clk % total_pixels;
	act_fps_frac = act_fps_frac * 100 / total_pixels;

	ret = snprintf(buf, PAGE_SIZE, "%u.%u\n", act_fps_int, act_fps_frac);

	return ret;
}
static DEVICE_ATTR_RO(actual_fps);

static ssize_t dynamic_fps_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct panel_info *panel = dispc->ctx.panel;

	if (!panel) {
		pr_err("panel doesn't exist.");
		return -ENXIO;
	}
	ret = snprintf(buf, PAGE_SIZE, "current fps: %u\n", panel->fps);

	return ret;
}

static ssize_t dynamic_fps_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int ret = 0;
	int fps = 0;
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct panel_info *panel = dispc->ctx.panel;

	if (!panel) {
		pr_err("panel doesn't exist.");
		return -ENXIO;
	}

	if (panel->type != SPRD_PANEL_TYPE_MIPI) {
		pr_err("sys write failure. Current panel is not mipi dsi\n");
		return 0;
	}

	ret = kstrtoint(buf, 10, &fps);
	if (ret || !fps) {
		pr_err("Invalid input for fps\n");
		return -EINVAL;
	}

	if (panel->fps == fps) {
		pr_warn("new fps is the same as current fps\n");
		return count;
	}

	ret = sprd_dispc_dynamic_clk(dispc, SPRD_DYNAMIC_FPS, fps);
	if (ret) {
		pr_err("failed to change dpi clock. ret=%d\n", ret);
		return ret;
	}

	return count;
}
static DEVICE_ATTR_RW(dynamic_fps);

static ssize_t vsync_rate_report_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct panel_info *panel = dispc->ctx.panel;

	if (!panel) {
		pr_err("panel doesn't exist.");
		return -ENXIO;
	}

	ret = snprintf(buf, PAGE_SIZE, "current fps: %u\n",
		dispc->ctx.vsync_report_rate);

	return ret;


}
static ssize_t vsync_rate_report_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct dispc_context *ctx = &dispc->ctx;
	struct panel_info *panel = ctx->panel;

	if (!panel) {
		pr_err("panel doesn't exist.");
		return -ENXIO;
	}

	ret = kstrtoint(buf, 10, &ctx->vsync_report_rate);
	if (ret || !ctx->vsync_report_rate) {
		pr_err("Invalid input for fps\n");
		return -EINVAL;
	}
	if (ctx->vsync_report_rate > panel->fps)
		ctx->vsync_report_rate = panel->fps;

	ctx->vsync_ratio_to_panel = panel->fps / ctx->vsync_report_rate;

	return count;
}
static DEVICE_ATTR_RW(vsync_rate_report);

static ssize_t regs_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct sprd_dispc *dispc = dev_get_drvdata(dev);

	ret = snprintf(buf, PAGE_SIZE,
			"Dispc reg offset: %x\n"
			"Dispc reg length: %x\n",
			dispc->ctx.base_offset[0],
			dispc->ctx.base_offset[1]);

	return ret;
}

static ssize_t regs_offset_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct sprd_dispc *dispc = dev_get_drvdata(dev);

	str_to_u32_array(buf, 16, dispc->ctx.base_offset);

	return count;
}
static DEVICE_ATTR_RW(regs_offset);

static ssize_t wr_regs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned int i;
	unsigned int reg;
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	unsigned int offset = dispc->ctx.base_offset[0];
	unsigned int length = dispc->ctx.base_offset[1];

	for (i = 0; i < length; i++) {
		reg = readl((void __iomem *)(dispc->ctx.base + offset));
		ret += snprintf(buf + ret, PAGE_SIZE, "%x ", reg);
	}

	return ret;
}

static ssize_t wr_regs_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	uint32_t temp = dispc->ctx.base_offset[0];
	uint32_t length = dispc->ctx.base_offset[1];
	uint32_t *value;
	uint32_t i;

	value = kzalloc(length * 4, GFP_KERNEL);
	if (!value)
		return -ENOMEM;

	str_to_u32_array(buf, 16, value);

	for (i = 0; i < length; i++) {
		writel(value[i], (void __iomem *)(dispc->ctx.base + temp));
		temp += 0x04;
	}

	kfree(value);

	return count;
}
static DEVICE_ATTR_RW(wr_regs);

static ssize_t disable_flip_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct sprd_dispc *dispc = dev_get_drvdata(dev);

	ret = snprintf(buf, PAGE_SIZE, "%d\n",
			dispc->ctx.disable_flip);

	return ret;
}

static ssize_t disable_flip_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t value;
	struct sprd_dispc *dispc = dev_get_drvdata(dev);

	if (kstrtou32(buf, 10, &value)) {
		pr_err("Invalid input for disable_flip\n");
		return -EINVAL;
	}

	dispc->ctx.disable_flip = value ? true : false;

	return count;
}
static DEVICE_ATTR_RW(disable_flip);

static ssize_t low_res_simu_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct panel_info *panel = dispc->ctx.panel;

	ret = snprintf(buf, PAGE_SIZE, "%d\n",
			panel->low_res_simu);

	return ret;
}

static ssize_t low_res_simu_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t value;
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct panel_info *panel = dispc->ctx.panel;

	if (kstrtou32(buf, 10, &value)) {
		pr_err("Invalid input for low_res_simu\n");
		return -EINVAL;
	}

	panel->low_res_simu = value;
	file_write(LOW_RES_SIMU_EN_PATH, O_WRONLY, 0,
		(char *)&(panel->low_res_simu), sizeof(panel->low_res_simu));

	return count;
}
static DEVICE_ATTR_RW(low_res_simu);

static ssize_t irq_register_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t value;
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct dispc_context *ctx = &dispc->ctx;
	int ret;

	if (kstrtou32(buf, 10, &value)) {
		pr_err("Invalid input for irq_register\n");
		return -EINVAL;
	}

	if (value > 0 && ctx->irq_num) {
		down(&ctx->refresh_lock);
		if (!ctx->is_inited) {
			pr_err("dispc is not initialized!\n");
			up(&ctx->refresh_lock);
			return -1;
		}
		if (dispc->core->enable_vsync)
			if (dispc->core->enable_vsync(ctx)) {
				up(&ctx->refresh_lock);
				return -1;
			}
		ret = request_irq(ctx->irq_num, ctx->dispc_isr,
				0, "DISPC", dispc);
		if (ret) {
			up(&ctx->refresh_lock);
			pr_err("error: dispc request irq failed\n");
			return -EINVAL;
		}
		up(&ctx->refresh_lock);
	}

	return count;
}
static DEVICE_ATTR_WO(irq_register);

static ssize_t irq_unregister_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret = snprintf(buf, PAGE_SIZE, "%d\n",
			irq_status);
	return ret;
}

static ssize_t irq_unregister_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t value;
	struct irq_desc *desc;
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct dispc_context *ctx = &dispc->ctx;

	if (kstrtou32(buf, 10, &value)) {
		pr_err("Invalid input for irq_unregister\n");
		return -EINVAL;
	}
	if (value > 0) {
		down(&ctx->refresh_lock);
		if (!ctx->is_inited) {
			pr_err("dispc is not initialized!\n");
			up(&ctx->refresh_lock);
			return -1;
		}
		if (dispc->core->disable_vsync) {
			if (dispc->core->disable_vsync(ctx)) {
				up(&ctx->refresh_lock);
				return -1;
			}
		}

		desc = irq_to_desc(ctx->irq_num);
		remove_irq(ctx->irq_num, desc->action);
		pr_info("remove dispc irq_num = %d\n", ctx->irq_num);
		up(&ctx->refresh_lock);
	}

	return count;
}

static DEVICE_ATTR_RW(irq_unregister);

static ssize_t disable_timeout_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int32_t value;
	struct sprd_dispc *dispc = dev_get_drvdata(dev);

	if (kstrtos32(buf, 10, &value)) {
		pr_err("Invalid input for disable_flip\n");
		return -EINVAL;
	}

	dispc->ctx.disable_timeout = value;

	return count;
}

static ssize_t disable_timeout_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct sprd_dispc *dispc = dev_get_drvdata(dev);

	ret = snprintf(buf, PAGE_SIZE, "%d\n",
		dispc->ctx.disable_timeout);

	return ret;
}

static DEVICE_ATTR_RW(disable_timeout);

static ssize_t dpu_version_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct dispc_context *ctx = &dispc->ctx;

	ret = snprintf(buf, PAGE_SIZE, "%s\n", ctx->version);

	return ret;
}

static DEVICE_ATTR_RO(dpu_version);

static struct attribute *dispc_attrs[] = {
	&dev_attr_run.attr,
	&dev_attr_bg_color.attr,
	&dev_attr_actual_fps.attr,
	&dev_attr_dynamic_fps.attr,
	&dev_attr_vsync_rate_report.attr,
	&dev_attr_regs_offset.attr,
	&dev_attr_wr_regs.attr,
	&dev_attr_refresh.attr,
	&dev_attr_disable_flip.attr,
	&dev_attr_low_res_simu.attr,
	&dev_attr_irq_register.attr,
	&dev_attr_irq_unregister.attr,
	&dev_attr_disable_timeout.attr,
	&dev_attr_dpu_version.attr,
	NULL,
};
static const struct attribute_group dispc_group = {
	.attrs = dispc_attrs,
};

#define BIN_ATTR_WO(_name, _size) BIN_ATTR(_name, S_IWUSR, NULL,\
					_name##_write, _size)

static ssize_t gamma_read(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count) {
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct dispc_context *ctx = &dispc->ctx;

	if (!dispc->core->enhance_get)
		return -EIO;

	if (off >= attr->size)
		return 0;

	if (off + count > attr->size)
		count = attr->size - off;

	down(&ctx->refresh_lock);
	if (!ctx->is_inited) {
		pr_err("dispc is not initialized\n");
		up(&ctx->refresh_lock);
		return -EINVAL;
	}
	dispc->core->enhance_get(ctx, ENHANCE_CFG_ID_GAMMA, buf);
	up(&ctx->refresh_lock);

	return count;
}

static ssize_t gamma_write(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count) {
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct dispc_context *ctx = &dispc->ctx;

	if (!dispc->core->enhance_set)
		return -EIO;

	/* I need to get my data in one piece */
	if (off != 0 || count != attr->size)
		return -EINVAL;

	down(&ctx->refresh_lock);
	dispc->core->enhance_set(ctx, ENHANCE_CFG_ID_GAMMA, buf);
	up(&ctx->refresh_lock);

	return count;
}
static BIN_ATTR_RW(gamma, 1536);

static ssize_t slp_read(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count) {
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct dispc_context *ctx = &dispc->ctx;

	if (!dispc->core->enhance_get)
		return -EIO;

	if (off >= attr->size)
		return 0;

	if (off + count > attr->size)
		count = attr->size - off;

	down(&ctx->refresh_lock);
	if (!ctx->is_inited) {
		pr_err("dispc is not initialized\n");
		up(&ctx->refresh_lock);
		return -EINVAL;
	}
	dispc->core->enhance_get(ctx, ENHANCE_CFG_ID_SLP, buf);
	up(&ctx->refresh_lock);

	return count;
}

static ssize_t slp_write(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count) {
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct dispc_context *ctx = &dispc->ctx;

	if (!dispc->core->enhance_set)
		return -EIO;

	/* I need to get my data in one piece */
	if (off != 0 || count != attr->size)
		return -EINVAL;

	down(&ctx->refresh_lock);
	dispc->core->enhance_set(ctx, ENHANCE_CFG_ID_SLP, buf);
	up(&ctx->refresh_lock);

	return count;
}
static BIN_ATTR_RW(slp, 6);

static ssize_t cm_read(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count) {
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct dispc_context *ctx = &dispc->ctx;

	if (!dispc->core->enhance_get)
		return -EIO;

	if (off >= attr->size)
		return 0;

	if (off + count > attr->size)
		count = attr->size - off;

	down(&ctx->refresh_lock);
	if (!ctx->is_inited) {
		pr_err("dispc is not initialized\n");
		up(&ctx->refresh_lock);
		return -EINVAL;
	}
	dispc->core->enhance_get(ctx, ENHANCE_CFG_ID_CM, buf);
	up(&ctx->refresh_lock);

	return count;
}

static ssize_t cm_write(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count) {
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct dispc_context *ctx = &dispc->ctx;

	if (!dispc->core->enhance_set)
		return -EIO;

	/* I need to get my data in one piece */
	if (off != 0 || count != attr->size)
		return -EINVAL;

	down(&ctx->refresh_lock);
	dispc->core->enhance_set(ctx, ENHANCE_CFG_ID_CM, buf);
	up(&ctx->refresh_lock);

	return count;
}
static BIN_ATTR_RW(cm, 24);

static ssize_t epf_read(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count) {
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct dispc_context *ctx = &dispc->ctx;

	if (!dispc->core->enhance_get)
		return -EIO;

	if (off >= attr->size)
		return 0;

	if (off + count > attr->size)
		count = attr->size - off;

	down(&ctx->refresh_lock);
	if (!ctx->is_inited) {
		pr_err("dispc is not initialized\n");
		up(&ctx->refresh_lock);
		return -EINVAL;
	}
	dispc->core->enhance_get(ctx, ENHANCE_CFG_ID_EPF, buf);
	up(&ctx->refresh_lock);

	return count;
}

static ssize_t epf_write(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count) {
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct dispc_context *ctx = &dispc->ctx;

	if (!dispc->core->enhance_set)
		return -EIO;

	/* I need to get my data in one piece */
	if (off != 0 || count != attr->size)
		return -EINVAL;

	down(&ctx->refresh_lock);
	dispc->core->enhance_set(ctx, ENHANCE_CFG_ID_EPF, buf);
	up(&ctx->refresh_lock);

	return count;
}
static BIN_ATTR_RW(epf, 14);


static ssize_t hsv_read(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count) {
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct dispc_context *ctx = &dispc->ctx;

	if (!dispc->core->enhance_get)
		return -EIO;

	if (off >= attr->size)
		return 0;

	if (off + count > attr->size)
		count = attr->size - off;

	down(&ctx->refresh_lock);
	if (!ctx->is_inited) {
		pr_err("dispc is not initialized\n");
		up(&ctx->refresh_lock);
		return -EINVAL;
	}
	dispc->core->enhance_get(ctx, ENHANCE_CFG_ID_HSV, buf);
	up(&ctx->refresh_lock);

	return count;
}

static ssize_t hsv_write(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count) {
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct dispc_context *ctx = &dispc->ctx;

	if (!dispc->core->enhance_set)
		return -EIO;

	/* I need to get my data in one piece */
	if (off != 0 || count != attr->size)
		return -EINVAL;

	down(&ctx->refresh_lock);
	dispc->core->enhance_set(ctx, ENHANCE_CFG_ID_HSV, buf);
	up(&ctx->refresh_lock);

	return count;
}
static BIN_ATTR_RW(hsv, 1440);


static ssize_t scl_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct dispc_context *ctx = &dispc->ctx;
	uint32_t param[2] = {};

	if (!dispc->core->enhance_get)
		return -EIO;

	down(&ctx->refresh_lock);
	if (!ctx->is_inited) {
		pr_err("dispc is not initialized\n");
		up(&ctx->refresh_lock);
		return -EINVAL;
	}
	dispc->core->enhance_get(ctx, ENHANCE_CFG_ID_SCL, param);
	up(&ctx->refresh_lock);

	ret = snprintf(buf, PAGE_SIZE, "%d x %d\n", param[0], param[1]);

	return ret;
}

static ssize_t scl_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct dispc_context *ctx = &dispc->ctx;
	uint32_t param[2] = {};

	if (!dispc->core->enhance_set)
		return -EIO;

	down(&ctx->refresh_lock);
	str_to_u32_array(buf, 10, param);
	dispc->core->enhance_set(ctx, ENHANCE_CFG_ID_SCL, param);
	up(&ctx->refresh_lock);

	return count;
}
static DEVICE_ATTR_RW(scl);

static ssize_t enable_write(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count) {
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct dispc_context *ctx = &dispc->ctx;

	if (!dispc->core->enhance_set)
		return -EIO;

	/* I need to get my data in one piece */
	if (off != 0 || count != attr->size)
		return -EINVAL;

	down(&ctx->refresh_lock);
	dispc->core->enhance_set(ctx, ENHANCE_CFG_ID_ENABLE, buf);
	up(&ctx->refresh_lock);

	return count;
}
static BIN_ATTR_WO(enable, 4);

static ssize_t disable_write(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count) {
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct dispc_context *ctx = &dispc->ctx;

	if (!dispc->core->enhance_set)
		return -EIO;

	/* I need to get my data in one piece */
	if (off != 0 || count != attr->size)
		return -EINVAL;

	down(&ctx->refresh_lock);
	dispc->core->enhance_set(ctx, ENHANCE_CFG_ID_DISABLE, buf);
	up(&ctx->refresh_lock);

	return count;
}
static BIN_ATTR_WO(disable, 4);

static ssize_t status_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct dispc_context *ctx = &dispc->ctx;
	uint32_t en = 0;
	int ret = 0;

	if (!dispc->core->enhance_get)
		return -EIO;

	down(&ctx->refresh_lock);
	if (!ctx->is_inited) {
		pr_err("dispc is not initialized\n");
		up(&ctx->refresh_lock);
		return -EINVAL;
	}
	dispc->core->enhance_get(ctx, ENHANCE_CFG_ID_ENABLE, &en);
	up(&ctx->refresh_lock);

	ret += snprintf(buf + ret, PAGE_SIZE, "value = 0x%08x\n", en);
	ret += snprintf(buf + ret, PAGE_SIZE, "scl: %d\n", !!(en & BIT(0)));
	ret += snprintf(buf + ret, PAGE_SIZE, "epf: %d\n", !!(en & BIT(1)));
	ret += snprintf(buf + ret, PAGE_SIZE, "hsv: %d\n", !!(en & BIT(2)));
	ret += snprintf(buf + ret, PAGE_SIZE, "cm: %d\n", !!(en & BIT(3)));
	ret += snprintf(buf + ret, PAGE_SIZE, "slp: %d\n", !!(en & BIT(4)));
	ret += snprintf(buf + ret, PAGE_SIZE, "gamma: %d\n", !!(en & BIT(5)));

	return ret;
}
static DEVICE_ATTR_RO(status);

static struct attribute *pq_ascii_attrs[] = {
	&dev_attr_scl.attr,
	&dev_attr_status.attr,
	NULL,
};
static struct bin_attribute *pq_bin_attrs[] = {
	&bin_attr_gamma,
	&bin_attr_slp,
	&bin_attr_cm,
	&bin_attr_hsv,
	&bin_attr_epf,
	&bin_attr_enable,
	&bin_attr_disable,
	NULL,
};
static const struct attribute_group pq_group = {
	.name = "PQ",
	.attrs = pq_ascii_attrs,
	.bin_attrs = pq_bin_attrs,
};

int sprd_dispc_sysfs_init(struct device *dev)
{
	int rc;
	struct sprd_dispc *dispc = dev_get_drvdata(dev);

	rc = sysfs_create_group(&(dev->kobj), &dispc_group);
	if (rc)
		pr_err("create dispc attr node failed, rc=%d\n", rc);

	if (!dispc->core->enhance_set)
		return rc;

	rc = sysfs_create_group(&(dev->kobj), &pq_group);
	if (rc)
		pr_err("create dispc PQ node failed, rc=%d\n", rc);

	return rc;
}
EXPORT_SYMBOL(sprd_dispc_sysfs_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("leon.he@spreadtrum.com");
MODULE_DESCRIPTION("Add dispc attribute nodes for userspace");

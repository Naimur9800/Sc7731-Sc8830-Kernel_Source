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

#include <asm/pgtable.h>
#include <linux/compat.h>
#include <linux/dma-mapping.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/uaccess.h>
#include "sprdfb_chip_common.h"
#include "sprdfb.h"
#include "sprdfb_panel.h"

#define SPRD_FB_ESD_TIMEOUT_CMD	(2000)

#define SPRD_FB_ESD_TIMEOUT_VIDEO	(1000)

#define SPRDFB_FRAMES_TO_SKIP	(1)

static int frame_count;
static unsigned PP[16];

static int sprd_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *fb)
{
	if ((var->xres != fb->var.xres) ||
	    (var->yres != fb->var.yres) ||
	    (var->xres_virtual != fb->var.xres_virtual) ||
	    (var->yres_virtual != fb->var.yres_virtual) ||
	    (var->xoffset != fb->var.xoffset) ||
#ifndef BIT_PER_PIXEL_SUPPORT
	    (var->bits_per_pixel != fb->var.bits_per_pixel) ||
#endif
	    (var->grayscale != fb->var.grayscale))
		return -EINVAL;
	return 0;
}

static int sprd_fb_pan_display(struct fb_var_screeninfo *var,
				struct fb_info *fb)
{
	int32_t ret;
	struct sprd_fbdev *dev = fb->par;

	if (frame_count < SPRDFB_FRAMES_TO_SKIP) {
		frame_count++;
		return 0;
	}

	if (0 == dev->enable) {
		pr_err("[%s]: Invalid Device status %d", __func__,
		       dev->enable);
		return -1;
	}

	ret = dev->ctrl->refresh(dev);
	if (ret) {
		pr_err("[%s]: failed to refresh !!\n", __func__);
		return -1;
	}

	return 0;
}

static int sprd_fb_ioctl(struct fb_info *info, unsigned int cmd,
			unsigned long arg)
{
	int result = -1;
	struct sprd_fbdev *dev = NULL;
#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
	struct overlay_info ovl_info;
	struct overlay_display ovl_display;
#endif
#ifdef CONFIG_FB_DYNAMIC_FREQ_SCALING
	int fps;
#endif
	int power_mode;
	void __user *argp = (void __user *)arg;

	if (NULL == info) {
		pr_err("[%s]: error. (Invalid Parameter)", __func__);
		return -1;
	}

	dev = info->par;

	switch (cmd) {
#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
	case SPRD_FB_SET_OVERLAY:
		memset(&ovl_info, 0, sizeof(ovl_info));

		if (copy_from_user(&ovl_info, argp, sizeof(ovl_info))) {
			pr_err("SET_OVERLAY copy failed!\n");
			return -EFAULT;
		}

		if (dev->ctrl->enable_overlay)
			result = dev->ctrl->enable_overlay(dev, &ovl_info, 1);
		break;

	case SPRD_FB_DISPLAY_OVERLAY:
		memset(&ovl_display, 0, sizeof(ovl_display));

		if (copy_from_user(&ovl_display, argp, sizeof(ovl_display))) {
			pr_err("DISPLAY_OVERLAY copy failed!\n");
			return -EFAULT;
		}

		if (NULL != dev->ctrl->display_overlay)
			result = dev->ctrl->display_overlay(dev, &ovl_display);
		break;
#endif

#ifdef CONFIG_FB_VSYNC_SUPPORT
	case FBIO_WAITFORVSYNC:
		if (dev->ctrl->wait_for_vsync)
			result = dev->ctrl->wait_for_vsync(dev);
		break;
#endif

#ifdef CONFIG_FB_DYNAMIC_FREQ_SCALING
	case SPRD_FB_CHANGE_FPS:
		result = copy_from_user(&fps, argp, sizeof(fps));

		if (result) {
			pr_err("%s: copy_from_user failed", __func__);
			return result;
		}

		pr_info("[%s]: fps changed to %d via ioctl\n", __func__, fps);
		result = sprd_fb_chg_clk_intf(dev, SPRD_FB_DYNAMIC_FPS, fps);

		if (result) {
			pr_err("%s: fps set fail.ret=%d\n", __func__, result);
			return result;
		}

		break;
#endif

	case SPRD_FB_IS_REFRESH_DONE:
		if (NULL != dev->ctrl->is_refresh_done)
			result = dev->ctrl->is_refresh_done(dev);
		break;

	case SPRD_FB_SET_POWER_MODE:
		result = copy_from_user(&power_mode, argp, sizeof(power_mode));
		pr_info("[%s] : SPRD_FB_SET_POWER_MODE (%d)\n",
			__func__, power_mode);
		break;

	default:
		pr_info("[%s]: unknown cmd(%d)\n", __func__, cmd);
		break;
	}

	return result;
}

#ifdef CONFIG_COMPAT
#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
static int sprd_fb_set_overlayinfo(struct fb_info *info, unsigned int cmd,
				  unsigned long arg)
{
	u32 buf;
	int err;
	struct overlay_info __user *ovl_info;
	struct overlay_info32 __user *ovl_info32;

	ovl_info = compat_alloc_user_space(sizeof(struct overlay_info));
	ovl_info32 = compat_ptr(arg);

	if (copy_in_user(&(ovl_info->layer_index), &(ovl_info32->layer_index),
			4 * sizeof(int)))
		return -EFAULT;

	if (copy_in_user(&(ovl_info->rb_switch), &(ovl_info32->rb_switch),
			sizeof(bool)))
		return -EFAULT;

	if (copy_in_user(&(ovl_info->rect), &(ovl_info32->rect),
			sizeof(struct overlay_rect)))
		return -EFAULT;

	if (get_user(buf, &ovl_info32->buffer) ||
			put_user(compat_ptr(buf), &ovl_info->buffer))
		return -EFAULT;

	err = sprd_fb_ioctl(info, cmd, (unsigned long)ovl_info);

	return err;
}
#endif

static int sprd_fb_compat_ioctl(struct fb_info *info, unsigned int cmd,
				unsigned long arg)
{
	int result = -1;

	if (NULL == info) {
		pr_err("sprd_fb_ioctl error. (Invalid Parameter)");
		return -1;
	}

	switch (cmd) {
#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
	case SPRD_FB_SET_OVERLAY:
		result = sprd_fb_set_overlayinfo(info, cmd, arg);
		break;

	case SPRD_FB_DISPLAY_OVERLAY:
		arg = (unsigned long)compat_ptr(arg);
		result = sprd_fb_ioctl(info, cmd, arg);
		break;
#endif

#ifdef CONFIG_FB_VSYNC_SUPPORT
	case FBIO_WAITFORVSYNC:
		result = sprd_fb_ioctl(info, cmd, arg);
		break;
#endif

#ifdef CONFIG_FB_DYNAMIC_FREQ_SCALING
	case SPRD_FB_CHANGE_FPS:
#endif

	case SPRD_FB_SET_POWER_MODE:
		arg = (unsigned long)compat_ptr(arg);
		result = sprd_fb_ioctl(info, cmd, arg);
		break;

	case SPRD_FB_IS_REFRESH_DONE:
	default:
		result = sprd_fb_ioctl(info, cmd, arg);
		break;
	}

	return result;
}
#endif

static struct fb_ops sprd_fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = sprd_fb_check_var,
	.fb_pan_display = sprd_fb_pan_display,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_ioctl = sprd_fb_ioctl,
#ifdef CONFIG_COMPAT
	.fb_compat_ioctl = sprd_fb_compat_ioctl,
#endif
};

static int setup_fb_mem(struct sprd_fbdev *dev, struct platform_device *pdev)
{
	u64 len;
	struct device_node *mem;

	mem = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!mem) {
		pr_err("memory-region: of_parse_phandle fail\n");
		return -ENODEV;
	}

	dev->fb->fix.smem_start = of_translate_address(mem,
		of_get_address(mem, 0, &len, NULL));
	pr_info("setup_fb_mem--smem_start:%lx, len:%llu\n",
		dev->fb->fix.smem_start, len);
	dev->fb->screen_base = phys_to_virt(dev->fb->fix.smem_start);
	if (!dev->fb->screen_base) {
		pr_err("Unable to map framebuffer base: 0x%lx\n",
			(unsigned long)dev->fb->screen_base);
		return -ENOMEM;
	}
	dev->fb->fix.smem_len = len;

	return 0;

}

static int setup_fb_info(struct sprd_fbdev *dev)
{
	struct fb_info *fb = dev->fb;
	struct panel_spec *panel = dev->panel;
	int ret;
	int default_fps;

	ret = of_property_read_u32(dev->of_dev->of_node,
			"sprd,default-fps", &default_fps);
	if (ret)
		pr_warn("read sprd,default-fps fail (%d)\n", ret);

	fb->fbops = &sprd_fb_ops;
	fb->flags = FBINFO_DEFAULT;

	/* finish setting up the fb_info struct */
	strncpy(fb->fix.id, "sprd_fb", 16);
	fb->fix.ypanstep = 1;
	fb->fix.type = FB_TYPE_PACKED_PIXELS;
	fb->fix.visual = FB_VISUAL_TRUECOLOR;
	fb->fix.line_length = panel->width * dev->bpp / 8;
	fb->var.xres = panel->width;
	fb->var.yres = panel->height;
	fb->var.width = panel->width;
	fb->var.height = panel->height;
	fb->var.xres_virtual = panel->width;
	if (dev->is_triple_fb)
		fb->var.yres_virtual = panel->height * 3;
	else
		fb->var.yres_virtual = panel->height * 2;
	fb->var.bits_per_pixel = dev->bpp;
	if (0 != dev->panel->fps) {
		fb->var.pixclock =
		    ((1000000000 / panel->width) * 1000) /
		    (dev->panel->fps * panel->height);
	} else {
		fb->var.pixclock =
		    ((1000000000 / panel->width) * 1000) /
		    (default_fps * panel->height);
	}

	fb->var.accel_flags = 0;
	fb->var.yoffset = 0;

	/* only support two pixel format */
	if (dev->bpp == 32) {
		/* ABGR */
		fb->var.red.offset = 16;
		fb->var.red.length = 8;
		fb->var.red.msb_right = 0;
		fb->var.green.offset = 8;
		fb->var.green.length = 8;
		fb->var.green.msb_right = 0;
		fb->var.blue.offset = 0;
		fb->var.blue.length = 8;
		fb->var.blue.msb_right = 0;
	} else {
		/* BGR */
		fb->var.red.offset = 11;
		fb->var.red.length = 5;
		fb->var.red.msb_right = 0;
		fb->var.green.offset = 5;
		fb->var.green.length = 6;
		fb->var.green.msb_right = 0;
		fb->var.blue.offset = 0;
		fb->var.blue.length = 5;
		fb->var.blue.msb_right = 0;
	}

	ret = fb_alloc_cmap(&fb->cmap, 16, 0);

	if (ret)	{
		pr_err("[%s] can't allocate color map", __func__);
		return -ENOMEM;
	}

	fb->pseudo_palette = PP;

	PP[0] = 0;
	for (ret = 1; ret < 16; ret++)
		PP[ret] = 0xffffffff;

	return 0;
}

static void fb_free_resources(struct sprd_fbdev *dev)
{
	if (dev == NULL)
		return;

	if (&dev->fb->cmap != NULL)
		fb_dealloc_cmap(&dev->fb->cmap);

	if (dev->fb->screen_base)
		devm_free_pages(dev->of_dev,
			(unsigned long)dev->fb->screen_base);

	unregister_framebuffer(dev->fb);
	framebuffer_release(dev->fb);
}

static int sprd_fb_runtime_suspend(struct device *dev)
{
	struct sprd_fbdev *fb_dev = dev_get_drvdata(dev);

	fb_dev->ctrl->suspend(fb_dev);

	return 0;
}

static int sprd_fb_runtime_resume(struct device *dev)
{
	struct sprd_fbdev *fb_dev = dev_get_drvdata(dev);

	fb_dev->ctrl->resume(fb_dev);

	return 0;
}

#ifdef CONFIG_FB_ESD_SUPPORT
static void ESD_work_func(struct work_struct *work)
{
	struct sprd_fbdev *dev =
	    container_of(work, struct sprd_fbdev, ESD_work.work);

	/* do real ESD check */
	if (NULL != dev->ctrl->ESD_check)
		dev->ctrl->ESD_check(dev);

	if (0 != dev->enable) {
		pr_debug("reschedule ESD workqueue!\n");
		schedule_delayed_work(&dev->ESD_work,
				      msecs_to_jiffies(dev->ESD_timeout_val));
		dev->ESD_work_start = true;
	} else {
		pr_warn(
		  "ESD workqueue reschedule failed device unavailable!!\n");
	}
}
#endif

static struct fb_capability sprd_fb_config_capability(struct sprd_fbdev *dev)
{
	uint32_t pll_type;
	int ret;
	static struct fb_capability s_fb_capability = { 0 };

	memset((void *)&s_fb_capability, 0, sizeof(s_fb_capability));

	ret = of_property_read_u32(dev->of_dev->of_node,
			"sprd,pll_type", &pll_type);
	if (ret) {
		pr_warn("[%s]: sprd,pll_type dt fail(0x%d)\n", __func__, ret);
		s_fb_capability.mipi_pll_type = 2;
	}
	s_fb_capability.mipi_pll_type = pll_type;

	/* TODO: Will be used for cpuidle */
	s_fb_capability.need_light_sleep = of_property_read_bool(
			dev->of_dev->of_node, "sprd,light-sleep");

	return s_fb_capability;
}

static const struct of_device_id sprd_fb_dt_ids[] = {
	{.compatible = "sprd,sc9838-dispc", .data = (void *) 0x1 },
	{.compatible = "sprd,fpga-dispc", .data = (void *) 0x2 },
	{},
};
MODULE_DEVICE_TABLE(of, sprd_fb_dt_ids);

static const void *sprd_get_fb_data(struct platform_device *pdev)
{
	const struct of_device_id *match;

	match = of_match_node(sprd_fb_dt_ids, pdev->dev.of_node);
	return match->data;
}

static int sprd_fb_read_dt(struct sprd_fbdev *dev)
{
	int ret = 0;
	struct resource r;

	dev->is_triple_fb = of_property_read_bool(dev->of_dev->of_node,
			"sprd,fb_triple_framebuffer");

	ret = of_property_read_u32(dev->of_dev->of_node,
			"sprd,input-data-type", &dev->bpp);
	if (ret) {
		/* Default data type is ABGR888 */
		dev->bpp = 32;
	}

	ret = of_address_to_resource(dev->of_dev->of_node, 0, &r);
	if (ret) {
		pr_err(" Can't get dispc register base address)\n");
		goto OUT;
	}

	sprd_dispc_base = (unsigned long)devm_ioremap_nocache(dev->of_dev,
				r.start, resource_size(&r));
	if (!sprd_dispc_base) {
		pr_err("sprd_dispc_base = (0x%lx)\n", (unsigned long)r.start);
		goto OUT;
	}

	dev->aon_apb_gpr = syscon_regmap_lookup_by_phandle(
				dev->of_dev->of_node, "sprd,syscon-aon-apb");
	if (IS_ERR(dev->aon_apb_gpr)) {
		pr_err("[%s]:failed to find sprd reg\n", __func__);
		goto OUT;
	}

	dev->ap_ahb_gpr = syscon_regmap_lookup_by_phandle(
				dev->of_dev->of_node, "sprd,syscon-ap-ahb");
	if (IS_ERR(dev->ap_ahb_gpr)) {
		pr_err("[%s]:failed to find sprd reg\n", __func__);
		goto OUT;
	}

	dev->ap_apb_gpr = syscon_regmap_lookup_by_phandle(
				dev->of_dev->of_node, "sprd,syscon-ap-apb");
	if (IS_ERR(dev->ap_apb_gpr)) {
		pr_err("%s:failed to find sprd reg\n", __func__);
		goto OUT;
	}

	return 0;

OUT:
	return -1;
}

static int sprd_fb_probe(struct platform_device *pdev)
{
	struct fb_info *fb = NULL;
	struct sprd_fbdev *dev = NULL;
	int ret = 0;

	pr_info("[%s]\n", __func__);

	fb = framebuffer_alloc(sizeof(struct sprd_fbdev), &pdev->dev);
	if (!fb) {
		pr_err("sprd_fb_probe allocate buffer fail.\n");
		ret = -ENOMEM;
		goto ERR;
	}

	dev = fb->par;
	dev->fb = fb;
	dev->of_dev = &(pdev->dev);
	dev->dev_id = (unsigned long)sprd_get_fb_data(pdev);

	ret = sprd_fb_read_dt(dev);
	if (ret)
		goto ERR;

	dev->ctrl = &sprd_fb_dispc_ctrl;
	dev->frame_count = 0;
	dev->capability = sprd_fb_config_capability(dev);
	dev->ctrl->early_init(dev);

	if (!sprd_panel_probe(dev)) {
		ret = -EIO;
		goto cleanup;
	}

	ret = setup_fb_mem(dev, pdev);
	if (ret)
		goto cleanup;

	ret = setup_fb_info(dev);
	if (ret)
		goto cleanup;

	/* register framebuffer device */
	ret = register_framebuffer(fb);
	if (ret) {
		pr_err("sprd_fb_probe register framebuffer fail.\n");
		goto cleanup;
	}

	platform_set_drvdata(pdev, dev);
	sprd_fb_create_sysfs(dev);
	/* Initialize dispc */
	dev->ctrl->init(dev);
	pm_runtime_enable(&pdev->dev);

#ifdef CONFIG_FB_ESD_SUPPORT
	INIT_DELAYED_WORK(&dev->ESD_work, ESD_work_func);
	if (SPRD_PANEL_IF_DPI == dev->panel_if_type)
		dev->ESD_timeout_val = SPRD_FB_ESD_TIMEOUT_VIDEO;
	else
		dev->ESD_timeout_val = SPRD_FB_ESD_TIMEOUT_CMD;

	dev->ESD_work_start = false;
	dev->check_esd_time = 0;
	dev->reset_dsi_time = 0;
	dev->panel_reset_time = 0;
#endif

	return 0;

cleanup:
	sprd_panel_remove(dev);
	dev->ctrl->uninit(dev);
	fb_free_resources(dev);
ERR:
	dev_err(&pdev->dev, "sprd fb probe failed\n");
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static int sprd_fb_remove(struct platform_device *pdev)
{
	struct sprd_fbdev *dev = platform_get_drvdata(pdev);

	pr_debug("[%s]\n", __func__);

	pm_runtime_disable(&pdev->dev);
	sprd_fb_remove_sysfs(dev);
	sprd_panel_remove(dev);
	dev->ctrl->uninit(dev);
	fb_free_resources(dev);
	return 0;
}

static const struct dev_pm_ops sprd_fb_pm_ops = {
	.runtime_suspend = sprd_fb_runtime_suspend,
	.runtime_resume = sprd_fb_runtime_resume,
};

static struct platform_driver sprd_fb_driver = {
	.probe = sprd_fb_probe,
	.remove = sprd_fb_remove,
	.driver = {
		   .name = "sprd_fb",
		   .pm	= &sprd_fb_pm_ops,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(sprd_fb_dt_ids),
		   },
};

static int __init sprd_fb_init(void)
{
	return platform_driver_register(&sprd_fb_driver);
}

static void __exit sprd_fb_exit(void)
{
	return platform_driver_unregister(&sprd_fb_driver);
}

module_init(sprd_fb_init);
module_exit(sprd_fb_exit);

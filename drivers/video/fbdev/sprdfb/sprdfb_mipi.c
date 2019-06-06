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

#include <linux/kernel.h>
#include "sprdfb.h"
#include "sprdfb_dispc_reg.h"
#include "sprdfb_panel.h"

static uint32_t sprd_mipi_readid(struct panel_spec *self)
{
	uint32_t id = 0;

	return id;
}

static void mipi_dispc_init_config(struct panel_spec *panel)
{
	uint32_t reg_val = dispc_read(DISPC_DPI_CTRL);

	pr_debug("[%s]\n", __func__);

	if (NULL == panel) {
		pr_err("[%s] fail.(Invalid Param)\n", __func__);
		return;
	}

	if (SPRD_PANEL_TYPE_MIPI != panel->type) {
		pr_err("[%s] fail.(not  mcu panel)\n", __func__);
		return;
	}

	if (SPRD_MIPI_MODE_CMD == panel->info.mipi->work_mode)
		/* use edpi as interface */
		dispc_set_bits(BIT_DISPC_DISPC_IF(1), DISPC_CTRL);
	else
		/* use dpi as interface */
		dispc_clear_bits(BIT_DISPC_DISPC_IF(3), DISPC_CTRL);

	/* h sync pol */
	if (SPRD_POLARITY_NEG == panel->info.mipi->h_sync_pol)
		reg_val |= (1 << 0);

	/* v sync pol */
	if (SPRD_POLARITY_NEG == panel->info.mipi->v_sync_pol)
		reg_val |= (1 << 1);

	/* de sync pol */
	if (SPRD_POLARITY_NEG == panel->info.mipi->de_pol)
		reg_val |= (1 << 2);

	if (SPRD_MIPI_MODE_VIDEO == panel->info.mipi->work_mode) {
#ifdef CONFIG_DPI_SINGLE_RUN
		/* Single run mode */
		reg_val |= (1 << 3);
#endif
	} else {
		if (!(panel->cap & PANEL_CAP_NOT_TEAR_SYNC)) {
			pr_info
			    ("mipi_dispc_init_config not support TE\n");
			/* enable te */
			reg_val |= (1 << 8);
		}

		/* te pol */
		if (SPRD_POLARITY_NEG == panel->info.mipi->te_pol)
			reg_val |= (1 << 9);

		/* use external te */
		reg_val |= (1 << 10);
	}

	/* dpi bits */
	switch (panel->info.rgb->video_bus_width) {
	case 16:
		break;
	case 18:
		reg_val |= (1 << 6);
		break;
	case 24:
		reg_val |= (2 << 6);
		break;
	default:
		break;
	}

	dispc_write(reg_val, DISPC_DPI_CTRL);
	pr_debug("[%s]: DISPC_DPI_CTRL = %d\n", __func__,
		 dispc_read(DISPC_DPI_CTRL));
}

static void mipi_dispc_set_timing(struct sprd_fbdev *dev)
{
	pr_debug("[%s]\n", __func__);

	dispc_write(dev->panel_timing.rgb_timing[RGB_LCD_H_TIMING],
		    DISPC_DPI_H_TIMING);
	dispc_write(dev->panel_timing.rgb_timing[RGB_LCD_V_TIMING],
		    DISPC_DPI_V_TIMING);
}

static int32_t sprd_mipi_panel_check(struct panel_spec *panel)
{
	if (NULL == panel) {
		pr_info("[%s] fail. (Invalid param)\n", __func__);
		return false;
	}

	if (SPRD_PANEL_TYPE_MIPI != panel->type) {
		pr_info("[%s] fail. (not mipi param)\n", __func__);
		return false;
	}

	pr_debug("[%s]\n", __func__);

	return true;
}

static void sprd_mipi_panel_mount(struct sprd_fbdev *dev)
{
	if ((NULL == dev) || (NULL == dev->panel)) {
		pr_err("[%s]: Invalid Param\n", __func__);
		return;
	}

	pr_info("[%s]\n", __func__);

	if (SPRD_MIPI_MODE_CMD == dev->panel->info.mipi->work_mode)
		dev->panel_if_type = SPRD_PANEL_IF_EDPI;
	else
		dev->panel_if_type = SPRD_PANEL_IF_DPI;

	dev->panel->info.mipi->ops = &sprd_fb_mipi_ops;

	if (NULL == dev->panel->ops->panel_readid)
		dev->panel->ops->panel_readid = sprd_mipi_readid;

	dev->panel_timing.rgb_timing[RGB_LCD_H_TIMING] =
	    rgb_calc_h_timing(dev->panel->info.mipi->timing);
	dev->panel_timing.rgb_timing[RGB_LCD_V_TIMING] =
	    rgb_calc_v_timing(dev->panel->info.mipi->timing);
}

static bool sprd_mipi_panel_init(struct sprd_fbdev *dev)
{
	bool ret = 0;

	ret = sprd_dsi_init(dev);
	if (ret) {
		pr_err("[%s]: Failed to initialize DSI\n", __func__);
		return ret;
	}

	mipi_dispc_init_config(dev->panel);
	mipi_dispc_set_timing(dev);

	return ret;
}

static void sprd_mipi_panel_uninit(struct sprd_fbdev *dev)
{
	sprd_dsi_uninit(dev);
}

static void sprd_mipi_panel_ready(struct sprd_fbdev *dev)
{
	sprd_dsi_ready(dev);
}

static void sprd_mipi_panel_before_reset(struct sprd_fbdev *dev)
{
	sprd_dsi_before_panel_reset(dev);
}

static void sprd_mipi_panel_enter_ulps(struct sprd_fbdev *dev)
{
	sprd_dsi_enter_ulps(dev);
}

static void sprd_mipi_panel_suspend(struct sprd_fbdev *dev)
{
	pr_info("[%s]\n", __func__);
	sprd_dsi_uninit(dev);
	/* sprd_dsi_suspend(dev); */
}

static void sprd_mipi_panel_resume(struct sprd_fbdev *dev)
{
	pr_info("[%s]\n", __func__);
	sprd_dsi_init(dev);
	/* sprd_dsi_resume(dev); */
}

static uint32_t sprd_mipi_get_status(struct sprd_fbdev *dev)
{
	return sprd_dsi_get_status(dev);
}

struct panel_if_ctrl sprd_fb_mipi_ctrl = {
	.if_name = "mipi",
	.panel_if_check = sprd_mipi_panel_check,
	.panel_if_mount = sprd_mipi_panel_mount,
	.panel_if_init = sprd_mipi_panel_init,
	.panel_if_uninit = sprd_mipi_panel_uninit,
	.panel_if_ready = sprd_mipi_panel_ready,
	.panel_if_before_refresh = NULL,
	.panel_if_after_refresh = NULL,
	.panel_if_before_panel_reset = sprd_mipi_panel_before_reset,
	.panel_if_enter_ulps = sprd_mipi_panel_enter_ulps,
	.panel_if_suspend = sprd_mipi_panel_suspend,
	.panel_if_resume = sprd_mipi_panel_resume,
	.panel_if_get_status = sprd_mipi_get_status,
};

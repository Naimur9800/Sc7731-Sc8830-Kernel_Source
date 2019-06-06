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

#define pr_fmt(fmt) "sprdfb: " fmt

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include "sprdfb.h"
#include "sprdfb_dispc_reg.h"
#include "sprdfb_panel.h"

static uint32_t rgb_readid(struct panel_spec *self)
{
	uint32_t id = 0;
	struct info_rgb *rgb;

	rgb = self->info.rgb;
	/* default id reg is 0 */
#ifdef CONFIG_I2C
	if (SPRD_RGB_BUS_TYPE_I2C == rgb->cmd_bus_mode) {
		rgb->bus_info.i2c->ops->i2c_read_16bits(0x0, false,
				(uint16_t *)&id, false);
	}
#endif
	return id;
}

static void rgb_dispc_init_config(struct panel_spec *panel)
{
	uint32_t reg_val = dispc_read(DISPC_DPI_CTRL);

	pr_debug("[%s]\n", __func__);

	if (NULL == panel) {
		pr_err("[%s] fail.(Invalid Param)\n", __func__);
		return;
	}

	if (SPRD_PANEL_TYPE_RGB != panel->type &&
	    SPRD_PANEL_TYPE_LVDS != panel->type) {
		pr_err("[%s] fail.(not  mcu panel)\n", __func__);
		return;
	}

	/* use dpi as interface */
	dispc_clear_bits(BIT_DISPC_DISPC_IF(3), DISPC_CTRL);

	/* h sync pol */
	if (SPRD_POLARITY_NEG == panel->info.rgb->h_sync_pol)
		reg_val |= (1 << 0);

	/* v sync pol */
	if (SPRD_POLARITY_NEG == panel->info.rgb->v_sync_pol)
		reg_val |= (1 << 1);

	/* de sync pol */
	if (SPRD_POLARITY_NEG == panel->info.rgb->de_pol)
		reg_val |= (1 << 2);
#ifdef CONFIG_DPI_SINGLE_RUN
	/* single run mode */
	reg_val |= (1 << 3);
#endif

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
	pr_debug("[%s] DISPC_DPI_CTRL = %d\n", __func__,
		 dispc_read(DISPC_DPI_CTRL));
}

static void rgb_dispc_set_timing(struct sprd_fbdev *dev)
{
	pr_debug("[%s]\n", __func__);

	dispc_write(dev->panel_timing.rgb_timing[RGB_LCD_H_TIMING],
		    DISPC_DPI_H_TIMING);
	dispc_write(dev->panel_timing.rgb_timing[RGB_LCD_V_TIMING],
		    DISPC_DPI_V_TIMING);
}

static int32_t sprdfb_rgb_panel_check(struct panel_spec *panel)
{
	if (NULL == panel) {
		pr_err("[%s] fail.(Invalid param)\n", __func__);
		return false;
	}

	if (SPRD_PANEL_TYPE_RGB != panel->type &&
	    SPRD_PANEL_TYPE_LVDS != panel->type) {
		pr_err("[%s] fail. (not rgb or lvds param)\n",
			__func__);
		return false;
	}

	pr_debug("[%s]\n", __func__);

	return true;
}

static void sprdfb_rgb_panel_mount(struct sprd_fbdev *dev)
{
	if ((NULL == dev) || (NULL == dev->panel)) {
		pr_err("[%s]: Invalid Param\n", __func__);
		return;
	}

	pr_debug("[%s]\n", __func__);

	dev->panel_if_type = SPRD_PANEL_IF_DPI;

	if (NULL == dev->panel->ops->panel_readid)
		dev->panel->ops->panel_readid = rgb_readid;

	dev->panel_timing.rgb_timing[RGB_LCD_H_TIMING] =
	    rgb_calc_h_timing(dev->panel->info.rgb->timing);
	dev->panel_timing.rgb_timing[RGB_LCD_V_TIMING] =
	    rgb_calc_v_timing(dev->panel->info.rgb->timing);
}

static bool sprdfb_rgb_panel_init(struct sprd_fbdev *dev)
{
	bool ret = false;

	if (SPRD_RGB_BUS_TYPE_SPI == dev->panel->info.rgb->cmd_bus_mode)
		ret = true;
	if (SPRD_RGB_BUS_TYPE_I2C == dev->panel->info.rgb->cmd_bus_mode)
		ret = true;
	if (dev->panel->info.rgb->cmd_bus_mode == SPRD_RGB_BUS_TYPE_LVDS)
		ret = true;

	if (!ret) {
		pr_err("[%s]: bus init fail!\n", __func__);
		return false;
	}

	rgb_dispc_init_config(dev->panel);
	rgb_dispc_set_timing(dev);

	if (!dev->panel_ready) {
		/* Nt35516 need vsync before panel init */
		dispc_set_bits(BIT_DISPC_DISPC_RUN, DISPC_CTRL);
		udelay(100);
		dispc_clear_bits(BIT_DISPC_DISPC_RUN, DISPC_CTRL);
	}
#ifdef SPRD_SUPPORT_LVDS_PANEL
	dispc_write((((dispc_read(DISPC_LVDS_CTRL) | BIT(21))
		      & 0xffff0000) | (0x460a)), DISPC_LVDS_CTRL);
#endif

	return true;
}

static void sprdfb_rgb_panel_uninit(struct sprd_fbdev *dev)
{
	return;
}

struct panel_if_ctrl sprd_fb_rgb_ctrl = {
	.if_name = "rgb",
	.panel_if_check = sprdfb_rgb_panel_check,
	.panel_if_mount = sprdfb_rgb_panel_mount,
	.panel_if_init = sprdfb_rgb_panel_init,
	.panel_if_uninit = sprdfb_rgb_panel_uninit,
	.panel_if_before_refresh = NULL,
	.panel_if_after_refresh = NULL,
};

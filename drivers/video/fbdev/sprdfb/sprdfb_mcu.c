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

#include "sprdfb.h"
#include "sprdfb_dispc_reg.h"
#include "sprdfb_panel.h"
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/kernel.h>

static int32_t sprd_dispc_mcu_send_cmd(uint32_t cmd)
{
	int wait_count = 0;

	/* busy wait for ahb fifo full sign's disappearance */
	while ((dispc_read(DISPC_DBI_QUEUE) & BIT(5)) && (wait_count < 20000)) {
		udelay(5);
		wait_count++;
	}
	if (wait_count >= 20000) {
		pr_err("[%s] send cmd not finish!!!\n", __func__);
		return -1;
	}
	dispc_write(cmd, DISPC_DBI_CMD);

	return 0;
}

static int32_t sprd_dispc_mcu_send_cmd_data(uint32_t cmd, uint32_t data)
{
	int wait_count = 0;

	/* busy wait for ahb fifo full sign's disappearance */
	while ((dispc_read(DISPC_DBI_QUEUE) & BIT(5)) && (wait_count < 20000)) {
		udelay(5);
		wait_count++;
	}
	if (wait_count >= 20000) {
		pr_err("[%s] send cmd data not finish 1!!!\n",
			__func__);
		return -1;
	}

	dispc_write(cmd, DISPC_DBI_CMD);
	wait_count = 0;
	/* busy wait for ahb fifo full sign's disappearance */
	while ((dispc_read(DISPC_DBI_QUEUE) & BIT(5)) && (wait_count < 20000)) {
		udelay(5);
		wait_count++;
	}
	if (wait_count >= 20000) {
		pr_info("[%s] send cmd data not finish 2!!!\n",
			__func__);
		return -1;
	}
	dispc_write(data, DISPC_DBI_DATA);

	return 0;
}

static int32_t sprd_dispc_mcu_send_data(uint32_t data)
{
	int wait_count = 0;

	/* busy wait for ahb fifo full sign's disappearance */
	while ((dispc_read(DISPC_DBI_QUEUE) & BIT(5)) && (wait_count < 20000)) {
		udelay(5);
		wait_count++;
	}
	if (wait_count >= 20000) {
		pr_info("[%s] send data not finish!!!\n", __func__);
		return -1;
	}

	dispc_write(data, DISPC_DBI_DATA);

	return 0;
}

static uint32_t sprd_dispc_mcu_read_data(void)
{
	int wait_count = 0;

	/* busy wait for ahb fifo full sign's disappearance */
	while ((dispc_read(DISPC_DBI_QUEUE) & BIT(5)) && (wait_count < 20000)) {
		udelay(5);
		wait_count++;
	}
	if (wait_count >= 20000) {
		pr_info("[%s] read data not finish!!!\n", __func__);
		return -1;
	}
	dispc_write(1 << 24, DISPC_DBI_DATA);
	udelay(50);

	return dispc_read(DISPC_DBI_RDATA);
}

static struct ops_mcu dispc_mcu_ops = {
	.send_cmd = sprd_dispc_mcu_send_cmd,
	.send_cmd_data = sprd_dispc_mcu_send_cmd_data,
	.send_data = sprd_dispc_mcu_send_data,
	.read_data = sprd_dispc_mcu_read_data,
};

static uint32_t mcu_calc_timing(struct timing_mcu *timing,
				struct sprd_fbdev *dev)
{
	uint32_t clk_rate;
	uint32_t rcss, rlpw, rhpw, wcss, wlpw, whpw;
	struct clk *clk = NULL;

	if (NULL == timing) {
		pr_err("[%s]: Invalid Param\n", __func__);
		return 0;
	}

	clk = of_clk_get_by_name(dev->of_dev->of_node, "dispc_dbi_clk");
	if (IS_ERR(clk))
		pr_warn("sprdfb: get clk_dispc_dbi fail!\n");
	else
		pr_info("sprdfb: get clk_dispc_dbi ok!\n");

	clk_rate = 250;

	pr_debug("[%s] clk_rate: 0x%x\n", __func__, clk_rate);
	/**
	* we assume : t = ? ns, dispc_dbi = ? MHz   so
	*      1ns need cycle  :  dispc_dbi /1000
	*      tns need cycles :  t * dispc_dbi / 1000
	*
	**/
#define MAX_DBI_RWCSS_TIMING_VALUE	15
#define MAX_DBI_RWLPW_TIMING_VALUE	63
#define MAX_DBI_RWHPW_TIMING_VALUE	63
#define DBI_CYCLES(ns) (((ns) * clk_rate + 1000 - 1) / 1000)

	/* ceiling */
	rcss = DBI_CYCLES(timing->rcss);
	if (rcss > MAX_DBI_RWCSS_TIMING_VALUE)
		rcss = MAX_DBI_RWCSS_TIMING_VALUE;

	rlpw = DBI_CYCLES(timing->rlpw);
	if (rlpw > MAX_DBI_RWLPW_TIMING_VALUE)
		rlpw = MAX_DBI_RWLPW_TIMING_VALUE;

	rhpw = DBI_CYCLES(timing->rhpw);
	if (rhpw > MAX_DBI_RWHPW_TIMING_VALUE)
		rhpw = MAX_DBI_RWHPW_TIMING_VALUE;

	wcss = DBI_CYCLES(timing->wcss);
	if (wcss > MAX_DBI_RWCSS_TIMING_VALUE)
		wcss = MAX_DBI_RWCSS_TIMING_VALUE;

	wlpw = DBI_CYCLES(timing->wlpw);
	if (wlpw > MAX_DBI_RWLPW_TIMING_VALUE)
		wlpw = MAX_DBI_RWLPW_TIMING_VALUE;

#ifndef CONFIG_LCD_CS_ALWAYS_LOW
	/* dispc/lcdc will waste one cycle because
	 * CS pulse will use one cycle */
	whpw = DBI_CYCLES(timing->whpw) - 1;
#else
	whpw = DBI_CYCLES(timing->whpw);
#endif
	if (whpw > MAX_DBI_RWHPW_TIMING_VALUE)
		whpw = MAX_DBI_RWHPW_TIMING_VALUE;

	return (whpw | (wlpw << 6) | (wcss << 12)
		| (rhpw << 16) | (rlpw << 22) | (rcss << 28));
}

static uint32_t sprd_mcu_readid(struct panel_spec *self)
{
	uint32_t id = 0;

	/* default id reg is 0 */
	self->info.mcu->ops->send_cmd(0x0);

	if (self->info.mcu->bus_width == 8) {
		id = (self->info.mcu->ops->read_data()) & 0xff;
		id <<= 8;
		id |= (self->info.mcu->ops->read_data()) & 0xff;
	} else {
		id = self->info.mcu->ops->read_data();
	}

	return id;
}

/* cs0 */
void sprd_fb_mcu_dispc_init_config(struct panel_spec *panel)
{
	uint32_t reg_val = 0;

	pr_debug("[%s] for cs0\n", __func__);

	if (NULL == panel) {
		pr_err("[%s] fail.(Invalid Param)\n", __func__);
		return;
	}

	if (SPRD_PANEL_TYPE_MCU != panel->type) {
		pr_err("[%s] fail.(not  mcu panel)\n", __func__);
		return;
	}

	/* use dbi as interface */
	dispc_set_bits(BIT_DISPC_DISPC_IF(2), DISPC_CTRL);

	/* CS0 bus mode [BIT0]: 8080/6800 */
	switch (panel->info.mcu->bus_mode) {
	case LCD_BUS_8080:
		break;
	case LCD_BUS_6800:
		reg_val |= 1;
		break;
	default:
		break;
	}
	/* CS0 bus width [BIT3:1] */
	switch (panel->info.mcu->bus_width) {
	case 8:
		break;
	case 9:
		reg_val |= (1 << 1);
		break;
	case 16:
		reg_val |= (2 << 1);
		break;
	case 18:
		reg_val |= (3 << 1);
		break;
	case 24:
		reg_val |= (4 << 1);
		break;
	default:
		break;
	}

	/* CS0 pixel bits [BIT5:4] */
	switch (panel->info.mcu->bpp) {
	case 16:
		break;
	case 18:
		reg_val |= (1 << 4);
		break;
	case 24:
		reg_val |= (2 << 4);
		break;
	default:
		break;
	}

	/* TE enable */
	reg_val |= (1 << 16);
	if (SPRD_POLARITY_NEG == panel->info.mcu->te_pol)
		reg_val |= (1 << 17);
	dispc_write(panel->info.mcu->te_sync_delay, DISPC_TE_SYNC_DELAY);

#ifdef CONFIG_LCD_CS_ALWAYS_LOW
	/* CS alway low mode */
	reg_val |= (1 << 21);
#else
	/* CS not alway low mode */
#endif

	/* CS0 selected */
	dispc_write(reg_val, DISPC_DBI_CTRL);
	pr_debug("[%s] DISPC_DBI_CTRL = %d\n", __func__,
		 dispc_read(DISPC_DBI_CTRL));
}

void sprd_fb_mcu_dispc_set_timing(struct sprd_fbdev *dev, uint32_t type)
{
	pr_debug("[%s] for cs0, type = %d\n", __func__, type);

	switch (type) {
	case MCU_LCD_REGISTER_TIMING:
		dispc_write(dev->panel_timing.
			    mcu_timing[MCU_LCD_REGISTER_TIMING],
			    DISPC_DBI_TIMING0);
		break;
	case MCU_LCD_GRAM_TIMING:
		dispc_write(dev->panel_timing.mcu_timing[MCU_LCD_GRAM_TIMING],
			    DISPC_DBI_TIMING0);
		break;
	default:
		break;
	}
}


static int32_t sprd_fb_mcu_panel_check(struct panel_spec *panel)
{
	struct info_mcu *mcu_info = NULL;
	bool rval = true;

	if (NULL == panel) {
		pr_info("[%s] fail. (Invalid param)\n", __func__);
		return false;
	}

	if (SPRD_PANEL_TYPE_MCU != panel->type) {
		pr_info("[%s] fail. (not mcu param)\n", __func__);
		return false;
	}

	mcu_info = panel->info.mcu;

	pr_debug("[%s]: bus width= %d, bpp = %d\n", __func__,
		 mcu_info->bus_width, mcu_info->bpp);

	switch (mcu_info->bus_width) {
	case 8:
		if ((16 != mcu_info->bpp) && (24 != mcu_info->bpp))
			rval = false;
		break;
	case 9:
		if (18 != mcu_info->bpp)
			rval = false;
		break;
	case 16:
		if ((16 != mcu_info->bpp) && (18 != mcu_info->bpp) &&
		    (24 != mcu_info->bpp))
			rval = false;
		break;
	case 18:
		if (18 != mcu_info->bpp)
			rval = false;
		break;
	case 24:
		if (24 != mcu_info->bpp)
			rval = false;
		break;
	default:
		rval = false;
		break;
	}

	if (!rval)
		pr_err("sprdfb: mcu_panel_check return false!\n");

	return rval;
}

static void sprd_fb_mcu_panel_mount(struct sprd_fbdev *dev)
{
	struct timing_mcu *timing = NULL;

	if ((NULL == dev) || (NULL == dev->panel)) {
		pr_err("[%s]: Invalid Param\n", __func__);
		return;
	}

	pr_debug("[%s]\n", __func__);

	dev->panel_if_type = SPRD_PANEL_IF_DBI;

	dev->panel->info.mcu->ops = &dispc_mcu_ops;

	if (NULL == dev->panel->ops->panel_readid)
		dev->panel->ops->panel_readid = sprd_mcu_readid;

	timing = dev->panel->info.mcu->timing;
	dev->panel_timing.mcu_timing[MCU_LCD_REGISTER_TIMING] =
	    mcu_calc_timing(timing, dev);
	timing++;
	dev->panel_timing.mcu_timing[MCU_LCD_GRAM_TIMING] =
	    mcu_calc_timing(timing, dev);
}

static bool sprd_fb_mcu_panel_init(struct sprd_fbdev *dev)
{
	sprd_fb_mcu_dispc_init_config(dev->panel);
	sprd_fb_mcu_dispc_set_timing(dev, MCU_LCD_REGISTER_TIMING);

	return true;
}

static void sprd_fb_mcu_panel_before_refresh(struct sprd_fbdev *dev)
{
	sprd_fb_mcu_dispc_set_timing(dev, MCU_LCD_GRAM_TIMING);
}

static void sprd_fb_mcu_panel_after_refresh(struct sprd_fbdev *dev)
{
	sprd_fb_mcu_dispc_set_timing(dev, MCU_LCD_REGISTER_TIMING);
}

struct panel_if_ctrl sprd_fb_mcu_ctrl = {
	.if_name = "mcu",
	.panel_if_check = sprd_fb_mcu_panel_check,
	.panel_if_mount = sprd_fb_mcu_panel_mount,
	.panel_if_init = sprd_fb_mcu_panel_init,
	.panel_if_before_refresh = sprd_fb_mcu_panel_before_refresh,
	.panel_if_after_refresh = sprd_fb_mcu_panel_after_refresh,
};

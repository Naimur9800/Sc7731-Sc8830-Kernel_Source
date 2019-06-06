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

#include "sprdfb.h"
#include "sprdfb_dispc_reg.h"
#include "sprdfb_panel.h"
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mutex.h>

/* for main_lcd */
static LIST_HEAD(panel_list_main);

static DEFINE_MUTEX(panel_mutex);

static int32_t sprd_panel_reset_dispc(struct panel_spec *self)
{
uint32_t timing1, timing2, timing3;

	if ((NULL != self) && (0 != self->reset_timing.time[0]) &&
		(0 != self->reset_timing.time[1]) &&
		(0 != self->reset_timing.time[2])) {
		timing1 = self->reset_timing.time[0];
		timing2 = self->reset_timing.time[1];
		timing3 = self->reset_timing.time[2];
	} else {
		timing1 = 20;
		timing2 = 20;
		timing3 = 120;
	}

	dispc_write(1, DISPC_RSTN);
	usleep_range(timing1 * 1000, timing1 * 1000 + 500);
	dispc_write(0, DISPC_RSTN);
	usleep_range(timing2 * 1000, timing2 * 1000 + 500);
	dispc_write(1, DISPC_RSTN);

	/* wait 10ms until the lcd is stable */
	usleep_range(timing3 * 1000, timing3 * 1000 + 500);

	return 0;
}

static int32_t sprd_panel_set_resetpin_dispc(uint32_t status)
{
	if (0 == status)
		dispc_write(0, DISPC_RSTN);
	else
		dispc_write(1, DISPC_RSTN);

	return 0;
}

static int sprd_panel_reset(struct sprd_fbdev *dev)
{
	if ((NULL == dev) || (NULL == dev->panel)) {
		pr_err("[%s]: Invalid param\n", __func__);
		return -1;
	}

	/* clk/data lane enter LP */
	if (NULL != dev->panel->if_ctrl->panel_if_before_panel_reset)
		dev->panel->if_ctrl->panel_if_before_panel_reset(dev);
	usleep_range(5000, 5500);

	/* reset panel */
	dev->panel->ops->sprd_panel_reset(dev->panel);

	return 0;
}

static int sprd_panel_sleep(struct sprd_fbdev *dev)
{
	if ((NULL == dev) || (NULL == dev->panel)) {
		pr_err("[%s]: Invalid param\n", __func__);
		return -1;
	}

	/* send sleep cmd to lcd */
	if (dev->panel->ops->panel_enter_sleep != NULL)
		dev->panel->ops->panel_enter_sleep(dev->panel, 1);

	msleep(100);

	/* clk/data lane enter LP */
	if ((NULL != dev->panel->if_ctrl->panel_if_before_panel_reset)
	    && (SPRD_PANEL_TYPE_MIPI == dev->panel->type))
		dev->panel->if_ctrl->panel_if_before_panel_reset(dev);

	return 0;
}

static void sprd_panel_set_resetpin(uint32_t status, struct panel_spec *panel)
{
	/* panel set reset pin status */
	sprd_panel_set_resetpin_dispc(status);
}

static int32_t sprd_panel_before_resume(struct sprd_fbdev *dev)
{
	/* restore  the reset pin to high */
	sprd_panel_set_resetpin(1, dev->panel);

	return 0;
}

static int32_t sprd_panel_after_suspend(struct sprd_fbdev *dev)
{
	/* set the reset pin to low */
	sprd_panel_set_resetpin(0, dev->panel);

	return 0;
}

static bool sprd_panel_check(struct panel_cfg *cfg)
{
	bool rval = true;

	if (NULL == cfg || NULL == cfg->panel) {
		pr_err("[%s]: Invalid Param!\n", __func__);
		return false;
	}

	pr_debug("[%s], lcd_id = 0x%x, type = %d\n",
		 __func__, cfg->lcd_id, cfg->panel->type);

	switch (cfg->panel->type) {
	case SPRD_PANEL_TYPE_MCU:
		cfg->panel->if_ctrl = &sprd_fb_mcu_ctrl;
		break;
	case SPRD_PANEL_TYPE_RGB:
		cfg->panel->if_ctrl = &sprd_fb_rgb_ctrl;
	case SPRD_PANEL_TYPE_LVDS:
		break;
	case SPRD_PANEL_TYPE_MIPI:
		cfg->panel->if_ctrl = &sprd_fb_mipi_ctrl;
		break;
	default:
		pr_err("[%s]: erro panel type.(%d, %d)", __func__,
			cfg->lcd_id, cfg->panel->type);
		cfg->panel->if_ctrl = NULL;
		break;
	};

	if (cfg->panel->if_ctrl->panel_if_check)
		rval = cfg->panel->if_ctrl->panel_if_check(cfg->panel);

	return rval;
}

static int sprd_panel_mount(struct sprd_fbdev *dev, struct panel_spec *panel)
{
	/* TODO: check whether the mode/res are supported */
	dev->panel = panel;

	if (NULL == dev->panel->ops->sprd_panel_reset)
		dev->panel->ops->sprd_panel_reset = sprd_panel_reset_dispc;

	pr_debug("[%s] calling panel_if_mount\n", __func__);
	panel->if_ctrl->panel_if_mount(dev);

	return 0;
}

int sprd_panel_init(struct sprd_fbdev *dev)
{
	if ((NULL == dev) || (NULL == dev->panel)) {
		pr_err("[%s]: Invalid param\n", __func__);
		return -1;
	}

	pr_info("[%s], type = %d\n", __func__, dev->panel->type);

	if (dev->panel->if_ctrl->panel_if_init(dev)) {
		pr_err("[%s]: panel_if_init fail!\n", __func__);
		return -1;
	}

	return 0;
}

int sprd_panel_ready(struct sprd_fbdev *dev)
{
	if ((NULL == dev) || (NULL == dev->panel)) {
		pr_err("[%s]: Invalid param\n", __func__);
		return -1;
	}

	pr_info("[%s], type = %d\n", __func__, dev->panel->type);

	if (NULL != dev->panel->if_ctrl->panel_if_ready)
		dev->panel->if_ctrl->panel_if_ready(dev);

	return 0;
}

static struct panel_spec *sprd_adapt_panel_from_readid(struct sprd_fbdev *dev)
{
	struct panel_cfg *cfg;
	struct list_head *panel_list;
	uint32_t id;

	pr_info("[%s]\n", __func__);

	panel_list = &panel_list_main;

	list_for_each_entry(cfg, panel_list, list) {
		pr_info("[%s]: try panel 0x%x\n", __func__,
			cfg->lcd_id);
		cur_panel_cfg = cfg;
		sprd_panel_mount(dev, cfg->panel);
		dev->ctrl->update_clk(dev);
		sprd_panel_init(dev);
		sprd_panel_reset(dev);
		pr_info("[%s]: panel ID 0x%x of panel %s\n", __func__,
			cfg->lcd_id, cfg->lcd_name);
		pr_info("[%s] reading panel ID\n", __func__);
		id = dev->panel->ops->panel_readid(dev->panel);
		if (id == cfg->lcd_id) {
			pr_info(
				 "[%s]: LCD Panel 0x%x is attached!\n",
				 __func__, cfg->lcd_id);
			dev->panel->ops->panel_init(dev->panel);
			sprd_panel_ready(dev);
			return cfg->panel;
		}
		pr_info("[%s]removing panel\n", __func__);
		sprd_panel_remove(dev);
	}
	pr_info("[%s] EXIT\n", __func__);

	return NULL;
}

bool sprd_panel_probe(struct sprd_fbdev *dev)
{
	struct panel_spec *panel;

	if (NULL == dev) {
		pr_err("[%s]: Invalid param\n", __func__);
		return false;
	}

	/* Read Panel Id */
	panel = sprd_adapt_panel_from_readid(dev);
	if (panel) {
		dev->panel_ready = true;
		return true;
	}

	pr_err("[%s] Failed to get LCD panel\n", __func__);

	return false;
}

void sprd_panel_invalidate_rect(struct panel_spec *self,
				  uint16_t left, uint16_t top,
				  uint16_t right, uint16_t bottom)
{
	/* Jessica TODO: */
	if (NULL != self->ops->panel_invalidate_rect)
		self->ops->panel_invalidate_rect(self, left, top, right,
						 bottom);
	/* Jessica TODO: Need set timing to GRAM timing */
}

void sprd_panel_invalidate(struct panel_spec *self)
{
	/* Jessica TODO: */
	if (NULL != self->ops->panel_invalidate)
		self->ops->panel_invalidate(self);
	/* Jessica TODO: Need set timing to GRAM timing */
}

void sprd_panel_before_refresh(struct sprd_fbdev *dev)
{
	if (NULL != dev->panel->if_ctrl->panel_if_before_refresh)
		dev->panel->if_ctrl->panel_if_before_refresh(dev);
}

void sprd_panel_after_refresh(struct sprd_fbdev *dev)
{
	if (NULL != dev->panel->if_ctrl->panel_if_after_refresh)
		dev->panel->if_ctrl->panel_if_after_refresh(dev);
}

#ifdef CONFIG_FB_DYNAMIC_FREQ_SCALING
void sprd_panel_change_fps(struct sprd_fbdev *dev, int fps_level)
{
	if (dev->panel->ops->panel_change_fps != NULL) {
		pr_info("[%s] fps_level= %d\n", __func__, fps_level);
		dev->panel->ops->panel_change_fps(dev->panel, fps_level);
	}
}
#endif

#ifdef CONFIG_FB_ESD_SUPPORT
/* Return value:  0--panel OK.1-panel has been reset */
uint32_t sprd_panel_ESD_check(struct sprd_fbdev *dev)
{
	int32_t result = 0;
	uint32_t if_status = 0;

	dev->check_esd_time++;

	if (SPRD_PANEL_IF_EDPI == dev->panel_if_type) {
		if (dev->panel->ops->panel_esd_check != NULL) {
			result = dev->panel->ops->panel_esd_check(dev->panel);
			pr_debug("[%s] panel check %d\n", __func__, result);
		}
	} else if (SPRD_PANEL_IF_DPI == dev->panel_if_type) {
#ifdef FB_CHECK_ESD_BY_TE_SUPPORT
		dev->esd_te_waiter++;
		dev->esd_te_done = 0;
		dispc_set_bits(DISPC_INT_TE_EN, DISPC_INT_EN);
		result = wait_event_interruptible_timeout(dev->esd_te_queue,
				dev->esd_te_done, msecs_to_jiffies(600));
		pr_debug("after wait (%d)\n", result);
		dispc_clear_bits(DISPC_INT_TE_EN, DISPC_INT_EN);
		if (!result) {	/*time out */
			pr_err("[%s] esd check not got te signal!\n", __func__);
			dev->esd_te_waiter = 0;
			result = 0;
		} else {
			pr_debug("[%s] esd check  got te signal!\n", __func__);
			result = 1;
		}
#else
		if (dev->panel->ops->panel_esd_check != NULL)
			result = dev->panel->ops->panel_esd_check(dev->panel);
#endif
	}

	if (0 == dev->enable) {
		pr_info("[%s] leave (Invalid device status)!\n",
			__func__);
		return 0;
	}

	if (result == 0) {
		dev->panel_reset_time++;

		if (SPRD_PANEL_IF_EDPI == dev->panel_if_type) {
			if (NULL != dev->panel->if_ctrl->panel_if_get_status) {
				if_status = dev->panel->if_ctrl->
				    panel_if_get_status(dev);
			}
		} else if (SPRD_PANEL_IF_DPI == dev->panel_if_type) {
			/* need reset dsi as default for dpi mode */
			if_status = 2;
		}

		if (0 == if_status) {
			pr_err("[%s] fail! Need reset panel.(%d,%d,%d)\n",
			     __func__, dev->check_esd_time,
			     dev->panel_reset_time, dev->reset_dsi_time);
			sprd_panel_reset(dev);

			if (0 == dev->enable) {
				pr_err("leave (Invalid device status)!\n");
				return 0;
			}

			dev->panel->ops->panel_init(dev->panel);
			sprd_panel_ready(dev);
		} else {
			pr_err("fail! Need reset panel and panel if!!!!(%d,%d,%d)\n",
				dev->check_esd_time,
			     dev->panel_reset_time, dev->reset_dsi_time);
			dev->reset_dsi_time++;
			if (NULL != dev->panel->if_ctrl->panel_if_suspend)
				dev->panel->if_ctrl->panel_if_suspend(dev);

			mdelay(10);

			sprd_panel_init(dev);
			sprd_panel_reset(dev);
			dev->panel->ops->panel_init(dev->panel);
			sprd_panel_ready(dev);
		}
		pr_debug("[%s]return 1\n", __func__);
		return 1;
	}

	return 0;
}
#endif

void sprd_panel_suspend(struct sprd_fbdev *dev)
{
	if (NULL == dev->panel)
		return;

	pr_debug("[%s]\n", __func__);

	/* Step1: Send lcd sleep cmd or reset panel directly */
	if (dev->panel->suspend_mode == SEND_SLEEP_CMD)
		sprd_panel_sleep(dev);
	else
		sprd_panel_reset(dev);

	/* Step2: Clk/data lane enter ulps */
	if (NULL != dev->panel->if_ctrl->panel_if_enter_ulps)
		dev->panel->if_ctrl->panel_if_enter_ulps(dev);

	/* Step3: Turn off mipi */
	if (NULL != dev->panel->if_ctrl->panel_if_suspend)
		dev->panel->if_ctrl->panel_if_suspend(dev);

	/* Step4: Reset pin to low */
	if (dev->panel->ops->sprd_panel_after_suspend != NULL)
		/* himax mipi lcd may define empty function */
		dev->panel->ops->sprd_panel_after_suspend(dev->panel);
	else
		sprd_panel_after_suspend(dev);
}

void sprd_panel_resume(struct sprd_fbdev *dev, bool from_deep_sleep)
{
	if (NULL == dev->panel)
		return;

	pr_debug("[%s], dev->enable= %d, from_deep_sleep = %d\n",
		__func__, dev->enable, from_deep_sleep);
	/* Step1: Set reset pin to high */
	if (dev->panel->ops->sprd_panel_before_resume != NULL) {
		/* himax mipi lcd may define empty function */
		dev->panel->ops->sprd_panel_before_resume(dev->panel);
	} else {
		sprd_panel_before_resume(dev);
	}

	if (from_deep_sleep) {
		/* Step2: Turn on mipi */
		sprd_panel_init(dev);

		/* Step3: Reset panel */
		sprd_panel_reset(dev);

		/* Step4: Panel init*/
		dev->panel->ops->panel_init(dev->panel);

		/* Step5: Clk/data lane enter HS */
		sprd_panel_ready(dev);
	} else {
		/* Step2: Turn on mipi */
		/* Jessica TODO: resume i2c, mipi */
		if (NULL != dev->panel->if_ctrl->panel_if_resume)
			dev->panel->if_ctrl->panel_if_resume(dev);

		/* Step3: Sleep out */
		if (NULL != dev->panel->ops->panel_enter_sleep)
			dev->panel->ops->panel_enter_sleep(dev->panel, 0);

		/* Step4: Clk/data lane enter HS */
		sprd_panel_ready(dev);
	}

}

void sprd_panel_remove(struct sprd_fbdev *dev)
{
	if (NULL == dev->panel)
		return;

	/* Jessica TODO:close panel, i2c, mipi */
	if (NULL != dev->panel->if_ctrl->panel_if_uninit)
		dev->panel->if_ctrl->panel_if_uninit(dev);
	dev->panel = NULL;
}

int sprd_panel_register(struct panel_cfg *cfg)
{
	pr_debug("[%s]\n", __func__);

	if (!sprd_panel_check(cfg)) {
		pr_info("[%s]: panel check fail!\n", __func__);
		return -1;
	}

	mutex_lock(&panel_mutex);

	list_add_tail(&cfg->list, &panel_list_main);

	mutex_unlock(&panel_mutex);

	return 0;
}

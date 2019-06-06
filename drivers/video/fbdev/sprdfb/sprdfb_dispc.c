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

#include <asm/cacheflush.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irqreturn.h>
#include <linux/mfd/syscon.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include "sprdfb_dispc_reg.h"
#include "sprdfb_panel.h"
#include "sprdfb.h"
#include "sprdfb_chip_common.h"

/* Dispc parent Clocks*/
#define DISPC_CLK_PARENT		("dispc_clk_parent")
#define DISPC_DBI_CLK_PARENT		("dispc_dbi_clk_parent")
#define DISPC_DPI_CLK_PARENT		("dispc_dpi_clk_parent")
#define DISPC_EMC_EN_PARENT		("dispc_emc_clk_parent")

/* Dispc clocks*/
#define DISPC_PLL_CLK			("dispc_clk")
#define DISPC_DBI_CLK			("dispc_dbi_clk")
#define DISPC_DPI_CLK			("dispc_dpi_clk")
#define DISPC_EMC_CLK			("dispc_emc_clk")

/* Y2R_Y param*/
#define SPRD_FB_BRIGHTNESS		(0x00 << 16)
#define SPRD_FB_CONTRAST		(0x100 << 0)

/* Y2R_U param*/
#define SPRD_FB_OFFSET_U		(0x81 << 16)
#define SPRD_FB_SATURATION_U		(0x100 << 0)

/* Y2R_V param*/
#define SPRD_FB_OFFSET_V		(0x82 << 16)
#define SPRD_FB_SATURATION_V		(0x100 << 0)

unsigned long sprd_dispc_base;

struct sprd_dispc_data {

	/* Dispc Clocks */
	struct clk *clk_dispc;
	struct clk *clk_dispc_dpi;
	struct clk *clk_dispc_dbi;
	struct clk *clk_dispc_emc;

	bool is_inited;
	bool is_first_frame;
	bool is_clk_open;
	bool is_clk_refresh;
	int clk_open_count;

	struct semaphore clk_sem;
	struct sprd_fbdev *dev;

	uint32_t vsync_waiter;
	wait_queue_head_t vsync_queue;
	uint32_t vsync_done;

#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
	/* overlay: 0-closed, 1-configed, 2-started */
	uint32_t overlay_state;
#endif

#ifdef CONFIG_FB_VSYNC_SUPPORT
	uint32_t waitfor_vsync_waiter;
	wait_queue_head_t waitfor_vsync_queue;
	uint32_t waitfor_vsync_done;
#endif
};

static struct sprd_dispc_data dispc_data = { 0 };

#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
static int sprd_overlay_start(struct sprd_fbdev *dev, uint32_t layer_index);
static int sprd_overlay_close(struct sprd_fbdev *dev);
#endif

static int32_t sprd_dispc_uninit(struct sprd_fbdev *dev);

#if defined(CONFIG_FB_DYNAMIC_FREQ_SCALING) || defined(CONFIG_FB_ESD_SUPPORT)
static void sprd_dispc_stop_feature(struct sprd_fbdev *dev);
static void sprd_dispc_run_feature(struct sprd_fbdev *dev);
#endif

static int sprd_dispc_clk_disable(struct sprd_dispc_data *dispc_ctx_ptr,
		enum SPRD_DYNAMIC_CLK_SWITCH clock_switch_type);

/**
 * sprd_dispc_check_new_clk - check and convert new clock to
 *				the value that can be set
 * @fb_dev: spreadtrum specific fb device
 * @new_pclk: actual settable clock via calculating new_val and fps
 * @pclk_src: clock source of dpi clock
 * @new_val: expected clock
 * @type: check the type is fps or pclk
 *
 * Returns 0 on success, MINUS on error.
 */
static int sprd_dispc_check_new_clk(struct sprd_fbdev *fb_dev,
		u32 *new_pclk, u32 pclk_src, u32 new_val, int type)
{
	int divider;
	u32 hpixels, vlines, pclk, fps;
	struct panel_spec *panel = fb_dev->panel;
	struct info_mipi *mipi;
	struct info_rgb *rgb;

	if (!panel) {
		pr_err("No panel is specified!\n");
		return -ENXIO;
	}

	mipi = panel->info.mipi;
	rgb = panel->info.rgb;

	if (fb_dev->panel_if_type != SPRD_PANEL_IF_DPI) {
		pr_err("[%s]: panel interface should be DPI\n", __func__);
		return -EINVAL;
	}
	if (new_val <= 0 || !new_pclk) {
		pr_err("[%s]: new parameter is invalid\n", __func__);
		return -EINVAL;
	}

	if (fb_dev->panel->type == LCD_MODE_DSI) {
		hpixels = panel->width + mipi->timing->hsync +
			mipi->timing->hbp + mipi->timing->hfp;
		vlines = panel->height + mipi->timing->vsync +
			mipi->timing->vbp + mipi->timing->vfp;
	} else if (fb_dev->panel->type == LCD_MODE_RGB ||
		   fb_dev->panel->type == LCD_MODE_LVDS) {
		hpixels = panel->width + rgb->timing->hsync +
			rgb->timing->hbp + rgb->timing->hfp;
		vlines = panel->height + rgb->timing->vsync +
			rgb->timing->vbp + rgb->timing->vfp;
	} else {
		pr_err("[%s]: unexpected panel type (%d)\n",
		       __func__, fb_dev->panel->type);
		return -EINVAL;
	}

	switch (type) {
		/*
		 * FIXME: TODO: SPRD_FB_FORCE_FPS will be used for
		 * accurate fps someday. But now it is the same
		 * as SPRD_FB_DYNAMIC_FPS.
		 */
	case SPRD_FB_FORCE_FPS:
	case SPRD_FB_DYNAMIC_FPS:
		if (new_val < LCD_MIN_FPS || new_val > LCD_MAX_FPS) {
			pr_err
			    ("Unsupported FPS. fps range should be [%d, %d]\n",
			     LCD_MIN_FPS, LCD_MAX_FPS);
			return -EINVAL;
		}
		pclk = hpixels * vlines * new_val;
		divider = ROUND(pclk_src, pclk);
		*new_pclk = pclk_src / divider;
		pr_debug("[%s]: new dynamic pclk=%d =fps=%d\n",
			__func__, *new_pclk, new_val);
		panel->fps = new_val;
		break;

	case SPRD_FB_DYNAMIC_PCLK:
		divider = ROUND(pclk_src, new_val);
		pclk = pclk_src / divider;
		fps = pclk / (hpixels * vlines);

		if (fps < LCD_MIN_FPS || fps > LCD_MAX_FPS) {
			pr_err
			    ("Unsupported FPS. fps range should be [%d, %d]\n",
			     LCD_MIN_FPS, LCD_MAX_FPS);
			return -EINVAL;
		}

		*new_pclk = pclk;
		pr_debug("[%s]: new dynamic pclk=%d =fps=%d\n",
			__func__, *new_pclk, fps);
		panel->fps = fps;
		break;

	default:
		pr_err("This checked type is unsupported.\n");
		*new_pclk = 0;
		return -EINVAL;
	}

	return 0;
}

static irqreturn_t sprd_dispc_isr(int irq, void *data)
{
	struct sprd_dispc_data *dispc_data = (struct sprd_dispc_data *)data;
	struct sprd_fbdev *dev = dispc_data->dev;
	bool done = false;
	uint32_t reg_val;
#ifdef CONFIG_FB_VSYNC_SUPPORT
	bool vsync = false;
#endif

	reg_val = dispc_read(DISPC_INT_STATUS);
	pr_debug("[%s](0x%x)\n", __func__, reg_val);

	if (NULL == dev)
		return -ENODEV;

	/* Check for error status */
	if (reg_val & DISPC_INT_ERR_STS)
		dispc_write(DISPC_INT_ERR_CLR, DISPC_INT_CLR);

	/* Check for dispc update done isr */
	if ((reg_val & DISPC_INT_UPDATE_DONE_STS)
		&& (SPRD_PANEL_IF_DPI == dev->panel_if_type)) {
		dispc_write(DISPC_INT_UPDATE_DONE_CLR, DISPC_INT_CLR);
		done = true;
	} else if ((reg_val & DISPC_INT_DONE_STS)
		&& (SPRD_PANEL_IF_DPI != dev->panel_if_type)) {
		/* dispc done isr */
		dispc_write(DISPC_INT_DONE_CLR, DISPC_INT_CLR);
		dispc_data->is_first_frame = false;
		done = true;
	}

#ifdef CONFIG_FB_ESD_SUPPORT
#ifdef FB_CHECK_ESD_BY_TE_SUPPORT
	/* Check for dispc external TE isr */
	if ((reg_val & DISPC_INT_TE_STS)
		&& (SPRD_PANEL_IF_DPI == dev->panel_if_type)) {
		dispc_write(DISPC_INT_TE_CLR, DISPC_INT_CLR);
		if (0 != dev->esd_te_waiter) {
			pr_debug("[%s] esd_te_done!\n", __func__);
			dev->esd_te_done = 1;
			wake_up_interruptible_all(&(dev->esd_te_queue));
			dev->esd_te_waiter = 0;
		}
	}
#endif
#endif

#ifdef CONFIG_FB_VSYNC_SUPPORT
	/* Check for dispc done isr */
	if ((reg_val & DISPC_INT_DPI_VSYNC_STS)
	    && (SPRD_PANEL_IF_DPI == dev->panel_if_type)) {
		dispc_write(DISPC_INT_DPI_VSYNC_CLR, DISPC_INT_CLR);
		vsync = true;
	} else if ((reg_val & DISPC_INT_TE_STS)
		   && (SPRD_PANEL_IF_EDPI == dev->panel_if_type)) {
		/* dispc te isr */
		dispc_write(DISPC_INT_TE_CLR, DISPC_INT_CLR);
		vsync = true;
	}

	if (vsync) {
		dispc_data->waitfor_vsync_done = 1;
		if (dispc_data->waitfor_vsync_waiter) {
			wake_up_interruptible_all(
				&(dispc_data->waitfor_vsync_queue));
		}
	}
#endif

	if (done) {
		dispc_data->vsync_done = 1;

#ifdef CONFIG_FB_DYNAMIC_CLK_SUPPORT
		if (SPRD_PANEL_IF_DPI != dev->panel_if_type) {
			sprd_dispc_clk_disable(dispc_data,
						 SPRD_DYNAMIC_CLK_REFRESH);
		}
#endif

		if (dispc_data->vsync_waiter) {
			wake_up_interruptible_all(&(dispc_data->vsync_queue));
			dispc_data->vsync_waiter = 0;
		}
		sprd_panel_after_refresh(dev);
	}

	return IRQ_HANDLED;
}

/* dispc soft reset */
static void sprd_dispc_reset(struct sprd_fbdev *dev)
{
	regmap_update_bits(dev->ap_ahb_gpr, REG_AP_AHB_AHB_RST,
		BIT_AP_AHB_DISPC_SOFT_RST, (u32)BIT_AP_AHB_DISPC_SOFT_RST);
	udelay(10);
	regmap_update_bits(dev->ap_ahb_gpr, REG_AP_AHB_AHB_RST,
		BIT_AP_AHB_DISPC_SOFT_RST, (u32)(~BIT_AP_AHB_DISPC_SOFT_RST));
}

static inline void sprd_dispc_set_bg_color(uint32_t bg_color)
{
	dispc_write(bg_color, DISPC_BG_COLOR);
}

static inline void sprd_dispc_set_osd_ck(uint32_t ck_color)
{
	dispc_write(ck_color, DISPC_OSD_CK);
}

static inline void sprd_dispc_osd_enable(bool is_enable)
{
	uint32_t reg_val;

	reg_val = dispc_read(DISPC_OSD_CTRL);

	if (is_enable)
		reg_val = reg_val | BIT_DISPC_OSD_OSD_EN;
	else
		reg_val = reg_val & (~BIT_DISPC_OSD_OSD_EN);

	dispc_write(reg_val, DISPC_OSD_CTRL);
}

static inline void sprd_dispc_set_osd_alpha(uint8_t alpha)
{
	dispc_write(alpha, DISPC_OSD_ALPHA);
}

static void sprd_dispc_dithering_enable(bool enable)
{
	if (enable)
		dispc_set_bits(BIT_DISPC_DITHER_EN, DISPC_CTRL);
	else
		dispc_clear_bits(BIT_DISPC_DITHER_EN, DISPC_CTRL);
}

static void sprd_dispc_pwr_enable(bool enable)
{
	if (enable)
		dispc_set_bits(BIT_DISPC_PWR_CTRL, DISPC_CTRL);
	else
		dispc_clear_bits(BIT_DISPC_PWR_CTRL, DISPC_CTRL);
}

static void sprd_dispc_set_exp_mode(uint16_t exp_mode)
{
	uint32_t reg_val = dispc_read(DISPC_CTRL);

	reg_val &= ~(0x3 << 16);
	reg_val |= BIT_DISPC_EXP_MODE(exp_mode);
	dispc_write(reg_val, DISPC_CTRL);
}

/**
 * sprd_dispc_set_threshold: set the threshold value for buffer
 *
 * thres0: module fetch data when buffer depth <= thres0
 * thres1: module release busy and close AXI clock
 * thres2: module open AXI clock and prepare to fetch data
**/
static void sprd_dispc_set_threshold(uint16_t thres0, uint16_t thres1,
				uint16_t thres2)
{
	dispc_write((thres0) | (thres1 << 14) | (thres2 << 16),
		    DISPC_BUF_THRES);
}

static void sprd_dispc_module_enable(void)
{
	/* dispc module enable */
	dispc_write(BIT_DISPC_DISPC_EN, DISPC_CTRL);

	/* disable dispc INT */
	dispc_write(0x0, DISPC_INT_EN);

	/* clear dispc INT */
	dispc_write(0x3F, DISPC_INT_CLR);
}

static inline int32_t sprd_dispc_set_disp_size(struct fb_var_screeninfo *var)
{
	uint32_t reg_val;

	reg_val = BIT_DISPC_SIZE_X(var->xres) | BIT_DISPC_SIZE_Y(var->yres);
	dispc_write(reg_val, DISPC_SIZE_XY);

	return 0;
}

static void sprd_dispc_layer_init(struct fb_var_screeninfo *var)
{
	uint32_t reg_val = 0;

	dispc_write(0x0, DISPC_IMG_CTRL);
	dispc_clear_bits(BIT_DISPC_OSD_OSD_EN, DISPC_OSD_CTRL);

	/******************* OSD layer setting **********************/

	reg_val |= BIT_DISPC_OSD_OSD_EN;

	/* disable  color key */

	/* alpha mode select  - block alpha */
	reg_val |= BIT_DISPC_OSD_ALPHA_SEL(SPRD_OSD_BLOCK_ALPHA);

	/* data format */
	if (var->bits_per_pixel == 32) {
		/* RGB888 */
		reg_val |= BIT_DISPC_OSD_FORMAT(SPRD_OSD_LAYER_RGB888);
		/* rb switch */
		reg_val |= BIT_DISPC_OSD_RB_SWITCH;
	} else {
		/* RGB565 */
		reg_val |= BIT_DISPC_OSD_FORMAT(SPRD_OSD_LAYER_RGB565);
		/* B2B3B0B1 */
		reg_val |= BIT_DISPC_OSD_SWITCH(SPRD_OSD_SWITCH_B2B3B0B1);
	}

	dispc_write(reg_val, DISPC_OSD_CTRL);

	/* OSD layer alpha value */
	sprd_dispc_set_osd_alpha(0xff);

	/* OSD layer size */
	reg_val = BIT_DISPC_OSD_SIZE_X(var->xres) |
			BIT_DISPC_OSD_SIZE_Y(var->yres);
	dispc_write(reg_val, DISPC_OSD_SIZE_XY);

	/* OSD layer start position */
	dispc_write(0, DISPC_OSD_DISP_XY);

	/* OSD layer pitch */
	reg_val = (var->xres & 0xfff);
	dispc_write(reg_val, DISPC_OSD_PITCH);

	/* OSD color_key value */
	sprd_dispc_set_osd_ck(0x0);
}

static void sprd_dispc_layer_update(struct fb_var_screeninfo *var)
{
	uint32_t reg_val = 0;

	/******************* OSD layer setting **********************/

	/* enable OSD layer */
	reg_val |= BIT_DISPC_OSD_OSD_EN;

	/* disable  color key */

	/* alpha mode select  - block alpha */
	reg_val |= BIT_DISPC_OSD_ALPHA_SEL(SPRD_OSD_BLOCK_ALPHA);

	/* data format */
	if (var->bits_per_pixel == 32) {
		/* ABGR */
		reg_val |= BIT_DISPC_OSD_FORMAT(SPRD_OSD_LAYER_RGB888);
		/* rb switch */
		reg_val |= BIT_DISPC_OSD_RB_SWITCH;
	} else {
		/* RGB565 */
		reg_val |= BIT_DISPC_OSD_FORMAT(SPRD_OSD_LAYER_RGB565);
		/* B2B3B0B1 */
		reg_val |= BIT_DISPC_OSD_SWITCH(SPRD_OSD_SWITCH_B2B3B0B1);
	}

	dispc_write(reg_val, DISPC_OSD_CTRL);
}

static int32_t sprd_dispc_sync(struct sprd_fbdev *dev)
{
	int ret;

	if (!dev->enable) {
		pr_err("[%s]: fb suspeneded already!!\n", __func__);
		return -1;
	}

	if (dev->dev_id == SPRD_FB_FPGA)
		ret = wait_event_interruptible_timeout(dispc_data.vsync_queue,
			dispc_data.vsync_done, msecs_to_jiffies(500));
	else
		ret = wait_event_interruptible_timeout(dispc_data.vsync_queue,
			dispc_data.vsync_done, msecs_to_jiffies(100));

	/* time out */
	if (!ret) {
		/* error recovery */
		dispc_data.vsync_done = 1;
		pr_err("sprd_dispc_sync time out!!!!!\n");
		return -1;
	}

	return 0;
}

static void sprd_dispc_run(struct sprd_fbdev *dev)
{

	if (!dev->enable)
		return;

	if (SPRD_PANEL_IF_DPI == dev->panel_if_type) {
		if (!dispc_data.is_first_frame) {
			dispc_data.vsync_done = 0;
			dispc_data.vsync_waiter++;
		}
		/* dpi register update */
		dispc_set_bits(BIT_DISPC_DPI_REG_UPDATE, DISPC_DPI_CTRL);
		udelay(30);

		if (dispc_data.is_first_frame) {
			/* dpi register update with SW and VSync */
			dispc_clear_bits(BIT_DISPC_DPI_REG_UPDATE_MODE,
						DISPC_DPI_CTRL);

			/* start refresh */
			dispc_set_bits(BIT_DISPC_DISPC_RUN, DISPC_CTRL);

			dispc_data.is_first_frame = false;
		} else {
			if (!dev->is_triple_fb)
				sprd_dispc_sync(dev);
#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
			else
				if (SPRD_OVERLAY_STATUS_STARTED ==
				    dispc_data.overlay_state)
					sprd_dispc_sync(dev);
#endif
		}
	} else {
		dispc_data.vsync_done = 0;
		/* start refresh */
		dispc_set_bits(BIT_DISPC_DISPC_RUN, DISPC_CTRL);
	}

	if (dev->dev_id != SPRD_FB_FPGA)
		dsi_irq_trick();
}

static void sprd_dispc_stop(struct sprd_fbdev *dev)
{
	if (SPRD_PANEL_IF_DPI == dev->panel_if_type) {
		/* dpi register update with SW only */
		dispc_set_bits(BIT_DISPC_DPI_REG_UPDATE_MODE, DISPC_DPI_CTRL);

		/* stop refresh */
		dispc_clear_bits(BIT_DISPC_DISPC_RUN, DISPC_CTRL);

		dispc_data.is_first_frame = true;
	}
}

static void sprd_dispc_clean_lcd(struct sprd_fbdev *dev)
{
	struct fb_info *fb = NULL;
	uint32_t size = 0;

	if ((NULL == dev) || (NULL == dev->fb)) {
		pr_err("[%s] Invalid parameter!\n", __func__);
		return;
	}

	down(&dev->refresh_lock);
	if (!dispc_data.is_first_frame || NULL == dev) {
		pr_debug("[%s] not first_frame\n", __func__);
		up(&dev->refresh_lock);
		return;
	}

	fb = dev->fb;

	size = BIT_DISPC_SIZE_X(dev->panel->width) |
			BIT_DISPC_SIZE_Y(dev->panel->height);
	if (SPRD_PANEL_IF_DPI != dev->panel_if_type)
		sprd_panel_invalidate(dev->panel);
	pr_debug("[%s] clean lcd!\n", __func__);

	dispc_write(size, DISPC_SIZE_XY);

	sprd_dispc_set_bg_color(0x00);
	dispc_write(dev->fb->fix.smem_start, DISPC_OSD_BASE_ADDR);
	/* sprd_dispc_osd_enable(false); */
	sprd_dispc_set_osd_alpha(0x00);
	sprd_dispc_run(dev);
	/* sprd_dispc_osd_enable(false); */
	up(&dev->refresh_lock);
	if (SPRD_PANEL_IF_DPI != dev->panel_if_type)
		sprd_dispc_sync(dev);
	else if (dev->is_triple_fb)
		sprd_dispc_sync(dev);
	mdelay(30);
}

static int sprd_dispc_clk_disable(struct sprd_dispc_data *dispc_ctx_ptr,
		enum SPRD_DYNAMIC_CLK_SWITCH
				    clock_switch_type)
{
	bool is_need_disable = false;

	if (!dispc_ctx_ptr)
		return 0;

	down(&dispc_data.clk_sem);
	switch (clock_switch_type) {
	case SPRD_DYNAMIC_CLK_FORCE:
		is_need_disable = true;
		break;
	case SPRD_DYNAMIC_CLK_REFRESH:
		dispc_ctx_ptr->is_clk_refresh = false;
		if (dispc_ctx_ptr->clk_open_count <= 0)
			is_need_disable = true;
		break;
	case SPRD_DYNAMIC_CLK_COUNT:
		if (dispc_ctx_ptr->clk_open_count > 0) {
			dispc_ctx_ptr->clk_open_count--;
			if (dispc_ctx_ptr->clk_open_count == 0) {
				if (!dispc_ctx_ptr->is_clk_refresh)
					is_need_disable = true;
			}
		}
		break;
	default:
		break;
	}

	if (dispc_ctx_ptr->is_clk_open && is_need_disable) {
		pr_debug("sprd_dispc_clk_disable real\n");
		clk_disable_unprepare(dispc_ctx_ptr->clk_dispc);
		clk_disable_unprepare(dispc_ctx_ptr->clk_dispc_dpi);
		clk_disable_unprepare(dispc_ctx_ptr->clk_dispc_dbi);
		dispc_ctx_ptr->is_clk_open = false;
		dispc_ctx_ptr->is_clk_refresh = false;
		dispc_ctx_ptr->clk_open_count = 0;
	}

	up(&dispc_data.clk_sem);

	pr_debug(
		 "sprd_dispc_clk_disable type=%d refresh=%d,count=%d\n",
		 clock_switch_type, dispc_ctx_ptr->is_clk_refresh,
		 dispc_ctx_ptr->clk_open_count);

	return 0;
}

static int sprd_dispc_clk_enable(struct sprd_dispc_data *dispc_ctx_ptr,
		enum  SPRD_DYNAMIC_CLK_SWITCH
				   clock_switch_type)
{
	int ret = 0;
	bool is_dispc_enable = false;
	bool is_dispc_dpi_enable = false;

	if (!dispc_ctx_ptr)
		return -1;

	down(&dispc_data.clk_sem);

	if (!dispc_ctx_ptr->is_clk_open) {
		pr_debug("sprd_dispc_clk_enable real\n");

		ret = clk_prepare_enable(dispc_ctx_ptr->clk_dispc);
		if (ret) {
			pr_err("enable clk_dispc error!!!\n");
			ret = -1;
			goto ERROR_CLK_ENABLE;
		}
		is_dispc_enable = true;

		ret = clk_prepare_enable(dispc_ctx_ptr->clk_dispc_dpi);
		if (ret) {
			pr_err("enable clk_dispc_dpi error!!!\n");
			ret = -1;
			goto ERROR_CLK_ENABLE;
		}
		is_dispc_dpi_enable = true;

		ret = clk_prepare_enable(dispc_ctx_ptr->clk_dispc_dbi);
		if (ret) {
			pr_err("enable clk_dispc_dbi error!!!\n");
			ret = -1;
			goto ERROR_CLK_ENABLE;
		}
		ret = clk_prepare_enable(dispc_data.clk_dispc_emc);
		if (ret) {
			pr_err("enable clk_dispc_emc error!!!\n");
			ret = -1;
			goto ERROR_CLK_ENABLE;
		}

		dispc_ctx_ptr->is_clk_open = true;
	}

	switch (clock_switch_type) {
	case SPRD_DYNAMIC_CLK_FORCE:
		break;
	case SPRD_DYNAMIC_CLK_REFRESH:
		dispc_ctx_ptr->is_clk_refresh = true;
		break;
	case SPRD_DYNAMIC_CLK_COUNT:
		dispc_ctx_ptr->clk_open_count++;
		break;
	default:
		break;
	}

	up(&dispc_data.clk_sem);

	pr_debug(
		 "sprd_dispc_clk_enable type=%d refresh=%d,count=%d,ret=%d\n",
		 clock_switch_type, dispc_ctx_ptr->is_clk_refresh,
		 dispc_ctx_ptr->clk_open_count, ret);

	return ret;

ERROR_CLK_ENABLE:
	if (is_dispc_enable)
		clk_disable_unprepare(dispc_ctx_ptr->clk_dispc);

	if (is_dispc_dpi_enable)
		clk_disable_unprepare(dispc_ctx_ptr->clk_dispc_dpi);

	/* spin_unlock_irqrestore(&dispc_data.clk_spinlock,irqflags); */
	up(&dispc_data.clk_sem);

	return ret;
}

/**
 * author: Yang.Haibing haibing.yang@spreadtrum.com
 *
 * sprd_dispc_update_clk - update dpi clock via @howto and @new_val
 * @fb_dev: spreadtrum specific fb device
 * @new_val: fps or new dpi clock
 * @howto: check the type is fps or pclk or mipi dphy freq
 *
 * Returns 0 on success, MINUS on error.
 */
static int sprd_dispc_update_clk(struct sprd_fbdev *fb_dev,
			    u32 new_val, int howto)
{
	int ret;
	u32 new_pclk;
	u32 old_val;
	u32 dpi_clk_src;
	struct panel_spec *panel = fb_dev->panel;

	ret = of_property_read_u32(fb_dev->of_dev->of_node,
			"sprd,dpi-clk", &dpi_clk_src);
	if (ret) {
		pr_err("[%s] read sprd,dpi-clk fail (%d)\n", __func__, ret);
		return -EINVAL;
	}

	switch (howto) {
	case SPRD_FB_FORCE_FPS:
		ret = sprd_dispc_check_new_clk(fb_dev, &new_pclk,
					  dpi_clk_src, new_val,
					  SPRD_FB_FORCE_FPS);
		if (ret) {
			pr_err("%s: new forced fps is invalid.", __func__);
			return -EINVAL;
		}
		break;

	case SPRD_FB_DYNAMIC_FPS:
		ret = sprd_dispc_check_new_clk(fb_dev, &new_pclk,
					  dpi_clk_src, new_val,
					  SPRD_FB_DYNAMIC_FPS);
		if (ret) {
			pr_err("%s: new dynamic fps is invalid.", __func__);
			return -EINVAL;
		}
		break;

	case SPRD_FB_FORCE_PCLK:
		new_pclk = new_val;
		break;

	case SPRD_FB_DYNAMIC_PCLK:
		ret = sprd_dispc_check_new_clk(fb_dev, &new_pclk,
					  dpi_clk_src, new_val,
					  SPRD_FB_DYNAMIC_PCLK);
		if (ret) {
			pr_err("%s: new dpi clock is invalid.", __func__);
			return -EINVAL;
		}
		break;

	case SPRD_FB_DYNAMIC_MIPI_CLK:
		if (!panel) {
			pr_err("%s: there is no panel!", __func__);
			return -ENXIO;
		}
		if (panel->type != LCD_MODE_DSI) {
			pr_err("%s: panel type isn't dsi mode", __func__);
			return -ENXIO;
		}
		old_val = panel->info.mipi->phy_feq;
		ret = sprd_dsi_chg_dphy_freq(fb_dev, new_val);
		if (ret) {
			pr_err("%s: new dphy freq is invalid.", __func__);
			return -EINVAL;
		}
		pr_info("dphy frequency is switched from %dHz to %dHz\n",
			old_val * 1000, new_val * 1000);
		return ret;

	default:
		pr_err("%s: Unsupported clock type.\n", __func__);
		return -EINVAL;
	}

	if (howto != SPRD_FB_FORCE_PCLK &&
	    howto != SPRD_FB_FORCE_FPS && !fb_dev->enable) {
		pr_warn(
		    "After fb_dev is resumed, dpi or mipi clk will be updated\n");
		return ret;
	}

	/* Now let's do update dpi clock */
	ret = clk_set_rate(dispc_data.clk_dispc_dpi, new_pclk);
	if (ret) {
		pr_err("Failed to set pixel clock.\n");
		return ret;
	}

	pr_info("dpi clock is switched from %dHz to %dHz\n",
		fb_dev->dpi_clock, new_pclk);
	/* Save the updated clock */
	fb_dev->dpi_clock = new_pclk;

	return ret;
}

static void sprd_dispc_print_clk(struct sprd_fbdev *dev)
{
	u32 reg_val0, reg_val1, reg_val2;

	regmap_read(dev->aon_apb_gpr, REG_AON_APB_APB_EB1, &reg_val0);
	regmap_read(dev->ap_ahb_gpr, REG_AP_AHB_AHB_EB, &reg_val1);
	regmap_read(dev->ap_apb_gpr, REG_AP_APB_APB_EB, &reg_val2);
	pr_debug
	    ("0x402e0004 = 0x%x 0x20e00000 = 0x%x 0x71300000 = 0x%x\n",
	    reg_val0, reg_val1, reg_val2);
}

static int32_t sprd_dispc_clk_init(struct sprd_fbdev *dev)
{
	int ret = 0;
	struct clk *clk_parent1, *clk_parent2, *clk_parent3, *clk_parent4;

	regmap_update_bits(dev->ap_apb_gpr, REG_AP_APB_APB_EB,
			BIT_AP_APB_CKG_EB, BIT_AP_APB_CKG_EB);

	clk_parent1 =
	    of_clk_get_by_name(dev->of_dev->of_node, DISPC_CLK_PARENT);
	if (IS_ERR(clk_parent1)) {
		pr_err("get clk_parent1 fail!\n");
		return -1;
	}

	clk_parent2 =
	    of_clk_get_by_name(dev->of_dev->of_node, DISPC_DBI_CLK_PARENT);
	if (IS_ERR(clk_parent2)) {
		pr_err("get clk_parent2 fail!\n");
		return -1;
	}

	clk_parent3 =
	    of_clk_get_by_name(dev->of_dev->of_node, DISPC_DPI_CLK_PARENT);
	if (IS_ERR(clk_parent3)) {
		pr_err("get clk_parent3 fail!\n");
		return -1;
	}
	clk_parent4 =
	    of_clk_get_by_name(dev->of_dev->of_node, DISPC_EMC_EN_PARENT);
	if (IS_ERR(clk_parent3)) {
		pr_err("get clk_parent4 fail!\n");
		return -1;
	}

	dispc_data.clk_dispc =
	    of_clk_get_by_name(dev->of_dev->of_node, DISPC_PLL_CLK);
	if (IS_ERR(dispc_data.clk_dispc)) {
		pr_warn("get clk_dispc fail!\n");
		return -1;
	}

	dispc_data.clk_dispc_dbi =
	    of_clk_get_by_name(dev->of_dev->of_node, DISPC_DBI_CLK);
	if (IS_ERR(dispc_data.clk_dispc_dbi)) {
		pr_warn("get clk_dispc_dbi fail!\n");
		return -1;
	}

	dispc_data.clk_dispc_dpi =
	    of_clk_get_by_name(dev->of_dev->of_node, DISPC_DPI_CLK);
	if (IS_ERR(dispc_data.clk_dispc_dpi)) {
		pr_warn("get clk_dispc_dpi fail!\n");
		return -1;
	}

	dispc_data.clk_dispc_emc =
	    of_clk_get_by_name(dev->of_dev->of_node, DISPC_EMC_CLK);
	if (IS_ERR(dispc_data.clk_dispc_emc)) {
		pr_warn("get clk_dispc_emc fail!\n");
		return -1;
	}

	ret = clk_set_parent(dispc_data.clk_dispc, clk_parent1);
	if (ret)
		pr_err("dispc set clk parent fail\n");

	ret = clk_set_parent(dispc_data.clk_dispc_dbi, clk_parent2);
	if (ret)
		pr_err("dispc set dbi clk parent fail!!\n");

	ret = clk_set_parent(dispc_data.clk_dispc_dpi, clk_parent3);
	if (ret)
		pr_err("dispc set dpi clk parent fail\n");

	ret = clk_set_parent(dispc_data.clk_dispc_emc, clk_parent4);
	if (ret)
		pr_err("dispc set emc clk parent fail\n");

	if (dev->panel && dev->panel->fps)
		sprd_dispc_update_clk(dev, dev->panel->fps, SPRD_FB_FORCE_FPS);

	ret = sprd_dispc_clk_enable(&dispc_data, SPRD_DYNAMIC_CLK_FORCE);
	if (ret) {
		pr_warn("[%s] enable dispc_clk fail!\n",
		       __func__);
		return -1;
	}

	sprd_dispc_print_clk(dev);

	return 0;
}

static int32_t sprd_dispc_module_init(struct sprd_fbdev *dev)
{
	int ret = 0;
	int irq_num = 0;

	if (dispc_data.is_inited) {
		pr_warn(
		       " dispc_module has already initialized! warning!!");
		return 0;
	}
	dispc_data.vsync_done = 1;
	dispc_data.vsync_waiter = 0;
	init_waitqueue_head(&(dispc_data.vsync_queue));

#ifdef CONFIG_FB_ESD_SUPPORT
#ifdef FB_CHECK_ESD_BY_TE_SUPPORT
	init_waitqueue_head(&(dev->esd_te_queue));
	dev->esd_te_waiter = 0;
	dev->esd_te_done = 0;
#endif
#endif

#ifdef CONFIG_FB_VSYNC_SUPPORT
	dispc_data.waitfor_vsync_done = 1;
	dispc_data.waitfor_vsync_waiter = 0;
	init_waitqueue_head(&(dispc_data.waitfor_vsync_queue));
#endif
	sema_init(&dev->refresh_lock, 1);

	irq_num = irq_of_parse_and_map(dev->of_dev->of_node, 0);
	pr_info("dispc irq_num = %d\n", irq_num);

	ret = devm_request_irq(dev->of_dev, irq_num, sprd_dispc_isr,
			IRQF_DISABLED, "DISPC", &dispc_data);
	if (ret) {
		pr_err("dispc failed to request irq!\n");
		sprd_dispc_uninit(dev);
		return -1;
	}

	dispc_data.is_inited = true;

	return 0;
}

static int32_t sprd_dispc_early_init(struct sprd_fbdev *dev)
{
	int ret = 0;

	if (!dispc_data.is_inited)
		sema_init(&dispc_data.clk_sem, 1);

	ret = sprd_dispc_clk_init(dev);
	if (ret) {
		pr_warn("sprd_dispc_clk_init fail!\n");
		return -1;
	}

	if (!dispc_data.is_inited) {
		/* panel not ready */
		pr_info("[%s]: Initializing dispc\n", __func__);
		sprd_dispc_reset(dev);
		sprd_dispc_module_enable();
		dispc_data.is_first_frame = true;
		ret = sprd_dispc_module_init(dev);
	} else {
		/* resume */
		pr_info(
		       "[%s]: sprd_dispc_early_init resume\n",
		       __func__);
		sprd_dispc_reset(dev);
		sprd_dispc_module_enable();
		dispc_data.is_first_frame = true;
	}

	return ret;
}

static int32_t sprd_dispc_init(struct sprd_fbdev *dev)
{
	uint32_t dispc_int_en_reg_val = 0x00;

	if (NULL == dev) {
		pr_err("[%s] Invalid parameter!\n", __func__);
		return -1;
	}

	dispc_data.dev = dev;

	/* set bg color */
	sprd_dispc_set_bg_color(0x0);
	/* enable dithering */
	sprd_dispc_dithering_enable(true);
	/* use MSBs as img exp mode */
	sprd_dispc_set_exp_mode(0x0);
	/* enable DISPC Power Control */
	sprd_dispc_pwr_enable(true);
	/* Linebuffer size: 5K */
	sprd_dispc_set_threshold(0x1388, 0x00, 0x1388);

	if (dispc_data.is_first_frame)
		sprd_dispc_layer_init(&(dev->fb->var));
	else
		sprd_dispc_layer_update(&(dev->fb->var));

	if (SPRD_PANEL_IF_DPI == dev->panel_if_type) {
		if (dispc_data.is_first_frame) {
			/* set dpi register update only with SW */
			dispc_set_bits(BIT_DISPC_DPI_REG_UPDATE_MODE,
						DISPC_DPI_CTRL);
		} else {
			/* set dpi register update with SW & VSYNC */
			dispc_clear_bits(BIT_DISPC_DPI_REG_UPDATE_MODE,
						DISPC_DPI_CTRL);
		}
		/* enable dispc update done INT */
		dispc_int_en_reg_val |= DISPC_INT_UPDATE_DONE_EN;
		/* enable hw vsync */
		dispc_int_en_reg_val |= DISPC_INT_DPI_VSYNC_EN;
	} else {
		/* enable dispc DONE INT */
		dispc_int_en_reg_val |= DISPC_INT_DONE_EN;
	}
	dispc_int_en_reg_val |= DISPC_INT_ERR_EN;
	dispc_write(dispc_int_en_reg_val, DISPC_INT_EN);
	dev->enable = 1;

	return 0;
}

static int32_t sprd_dispc_uninit(struct sprd_fbdev *dev)
{
	dev->enable = 0;
	sprd_dispc_clk_disable(&dispc_data, SPRD_DYNAMIC_CLK_FORCE);

	return 0;
}

static int32_t sprd_dispc_refresh(struct sprd_fbdev *dev)
{
	uint32_t reg_val = 0;
	struct fb_info *fb = dev->fb;
	uint32_t size = 0;

	uint32_t base =
	    fb->fix.smem_start + fb->fix.line_length * fb->var.yoffset;

	down(&dev->refresh_lock);
	if (0 == dev->enable) {
		pr_err("[%s]: do not refresh in suspend!!!\n",
		       __func__);
		goto ERROR_REFRESH;
	}

	if (SPRD_PANEL_IF_DPI != dev->panel_if_type) {
		dispc_data.vsync_waiter++;
		sprd_dispc_sync(dev);
	/* dispc_data.vsync_done = 0; */
#ifdef CONFIG_FB_DYNAMIC_CLK_SUPPORT
		if (sprd_dispc_clk_enable
		    (&dispc_data, SPRD_DYNAMIC_CLK_REFRESH)) {
			pr_warn(
			       "enable dispc_clk fail in refresh!\n");
			goto ERROR_REFRESH;
		}
#endif
	} else if (dev->is_triple_fb) {
		if ((dispc_read(DISPC_OSD_BASE_ADDR) !=
		     dispc_read(SHDW_OSD_BASE_ADDR))
		    && dispc_read(SHDW_OSD_BASE_ADDR) != 0) {
			dispc_data.vsync_waiter++;
			sprd_dispc_sync(dev);
		}
	}

	pr_debug("[%s] got sync\n", __func__);

	sprd_dispc_set_osd_alpha(0xff);

#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
	if (SPRD_OVERLAY_STATUS_STARTED == dispc_data.overlay_state)
		sprd_overlay_close(dev);
#endif

	size = BIT_DISPC_OSD_SIZE_X(fb->var.xres) |
			BIT_DISPC_OSD_SIZE_Y(fb->var.yres);
	dispc_write(base, DISPC_OSD_BASE_ADDR);
	dispc_write(0, DISPC_OSD_DISP_XY);
	dispc_write(size, DISPC_OSD_SIZE_XY);
	dispc_write(fb->var.xres, DISPC_OSD_PITCH);
	dispc_write(size, DISPC_SIZE_XY);

#ifdef BIT_PER_PIXEL_SUPPORT
	/* data format */
	if (fb->var.bits_per_pixel == 32) {
		/* ABGR */
		reg_val |= BIT_DISPC_OSD_FORMAT(SPRD_OSD_LAYER_RGB888);
		/* rb switch */
		reg_val |= BIT_DISPC_OSD_RB_SWITCH;
		dispc_clear_bits(0x30000, DISPC_CTRL);
	} else {
		/* RGB565 */
		reg_val |= BIT_DISPC_OSD_FORMAT(SPRD_OSD_LAYER_RGB565);
		/* B2B3B0B1 */
		reg_val |= BIT_DISPC_OSD_SWITCH(SPRD_OSD_SWITCH_B2B3B0B1);

		dispc_clear_bits(0x30000, DISPC_CTRL);
		dispc_set_bits(0x10000, DISPC_CTRL);
	}
	reg_val |= BIT_DISPC_OSD_OSD_EN;

	/* alpha mode select  - block alpha */
	reg_val |= BIT_DISPC_OSD_ALPHA_SEL(SPRD_OSD_BLOCK_ALPHA);

	dispc_write(reg_val, DISPC_OSD_CTRL);
#endif

	if (SPRD_PANEL_IF_DPI != dev->panel_if_type)
		sprd_panel_invalidate(dev->panel);

	sprd_panel_before_refresh(dev);

#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
	dispc_set_bits(BIT_DISPC_OSD_OSD_EN, DISPC_OSD_CTRL);
	if (SPRD_OVERLAY_STATUS_ON == dispc_data.overlay_state)
		sprd_overlay_start(dev, (SPRD_LAYER_IMG));
#endif

	dev->frame_count += 1;
	sprd_dispc_run(dev);

#ifdef CONFIG_FB_ESD_SUPPORT
	if (!dev->ESD_work_start) {
		pr_debug("schedule ESD work queue!\n");
		schedule_delayed_work(&dev->ESD_work,
				      msecs_to_jiffies(dev->ESD_timeout_val));
		dev->ESD_work_start = true;
	}
#endif

ERROR_REFRESH:
	up(&dev->refresh_lock);

	pr_debug("DISPC_CTRL: 0x%x\n", dispc_read(DISPC_CTRL));
	pr_debug("DISPC_SIZE_XY: 0x%x\n", dispc_read(DISPC_SIZE_XY));

	pr_debug("DISPC_BG_COLOR: 0x%x\n", dispc_read(DISPC_BG_COLOR));

	pr_debug("DISPC_INT_EN: 0x%x\n", dispc_read(DISPC_INT_EN));

	pr_debug("DISPC_OSD_CTRL: 0x%x\n", dispc_read(DISPC_OSD_CTRL));
	pr_debug("DISPC_OSD_BASE_ADDR: 0x%x\n",
		 dispc_read(DISPC_OSD_BASE_ADDR));
	pr_debug("DISPC_OSD_SIZE_XY: 0x%x\n",
		 dispc_read(DISPC_OSD_SIZE_XY));
	pr_debug("DISPC_OSD_PITCH: 0x%x\n",
		 dispc_read(DISPC_OSD_PITCH));
	pr_debug("DISPC_OSD_DISP_XY: 0x%x\n",
		 dispc_read(DISPC_OSD_DISP_XY));
	pr_debug("DISPC_OSD_ALPHA	: 0x%x\n",
		 dispc_read(DISPC_OSD_ALPHA));

	return 0;
}

static int32_t sprd_dispc_suspend(struct sprd_fbdev *dev)
{
	pr_info("[%s], dev->enable = %d\n", __func__,
	       dev->enable);

	if (0 != dev->enable) {
		down(&dev->refresh_lock);
		if (SPRD_PANEL_IF_DPI != dev->panel_if_type) {
			/* must wait ,sprd_dispc_sync() */
			dispc_data.vsync_waiter++;
			sprd_dispc_sync(dev);
#ifdef CONFIG_FB_DYNAMIC_CLK_SUPPORT
			pr_info("open clk in suspend\n");
			if (sprd_dispc_clk_enable
			    (&dispc_data, SPRD_DYNAMIC_CLK_COUNT)) {
				pr_warn(
				       "[%s] clk enable fail!!!\n",
				       __func__);
			}
#endif
			pr_info("[%s] got sync\n",
			       __func__);
		}

		dev->enable = 0;
		up(&dev->refresh_lock);

#ifdef CONFIG_FB_ESD_SUPPORT
		if (dev->ESD_work_start == true) {
			pr_info("cancel ESD work queue\n");
			cancel_delayed_work_sync(&dev->ESD_work);
			dev->ESD_work_start = false;
		}
#endif

		sprd_panel_suspend(dev);

		sprd_dispc_stop(dev);
		dispc_write(0, DISPC_INT_EN);
		/* fps > 20 */
		msleep(50);

		sprd_dispc_clk_disable(&dispc_data, SPRD_DYNAMIC_CLK_FORCE);
		clk_disable_unprepare(dispc_data.clk_dispc_emc);
	} else {
		pr_err("[%s]: Invalid device status %d\n",
		       __func__, dev->enable);
	}
	return 0;
}

static int32_t sprd_dispc_resume(struct sprd_fbdev *dev)
{
	pr_info("[%s], dev->enable= %d\n", __func__,
	       dev->enable);

	clk_prepare_enable(dispc_data.clk_dispc_emc);
	if (dev->enable == 0) {
		if (sprd_dispc_clk_enable
		    (&dispc_data, SPRD_DYNAMIC_CLK_FORCE)) {
			pr_warn("[%s] clk enable fail!!\n",
			       __func__);
		}

		dispc_data.vsync_done = 1;
		if (1) {
			pr_info("[%s] from deep sleep\n",
			       __func__);
			sprd_dispc_early_init(dev);
			sprd_panel_resume(dev, true);
			sprd_dispc_init(dev);
		} else {
			pr_info("[%s]  not from deep sleep\n",
			       __func__);

			sprd_panel_resume(dev, true);
		}

		dev->enable = 1;
		if (dev->panel->is_clean_lcd)
			sprd_dispc_clean_lcd(dev);

	}
	pr_info("[%s], leave dev->enable= %d\n", __func__,
	       dev->enable);

	return 0;
}

static int sprd_dispc_update_clk_intf(struct sprd_fbdev *fb_dev)
{
	return sprd_dispc_update_clk(fb_dev, fb_dev->panel->fps,
			SPRD_FB_FORCE_FPS);
}

#ifdef CONFIG_FB_ESD_SUPPORT
/* for video esd check */
static int32_t sprd_dispc_check_esd_dpi(struct sprd_fbdev *dev)
{
	uint32_t ret = 0;
#if (!defined(FB_CHECK_ESD_BY_TE_SUPPORT) && !defined(FB_CHECK_ESD_IN_VFP))
	unsigned long flags;
#endif

#ifdef FB_CHECK_ESD_BY_TE_SUPPORT
	ret = sprd_panel_ESD_check(dev);
	if (0 != ret)
		sprd_dispc_run_feature(dev);
#else
#ifdef FB_CHECK_ESD_IN_VFP
	ret = sprd_panel_ESD_check(dev);
#else
	local_irq_save(flags);
	sprd_dispc_stop_feature(dev);

	ret = sprd_panel_ESD_check(dev);

	sprd_dispc_run_feature(dev);
	local_irq_restore(flags);
#endif
#endif

	return ret;
}

/* for cmd esd check */
static int32_t sprd_dispc_check_esd_edpi(struct sprd_fbdev *dev)
{
	uint32_t ret = 0;

	dispc_data.vsync_waiter++;
	sprd_dispc_sync(dev);

#ifdef CONFIG_FB_DYNAMIC_CLK_SUPPORT
	if (sprd_dispc_clk_enable(&dispc_data, SPRD_DYNAMIC_CLK_COUNT)) {
		pr_warn("[%s] clk enable fail!!!\n",
		       __func__);
		return -1;
	}
#endif

	ret = sprd_panel_ESD_check(dev);

	if (0 != ret)
		sprd_dispc_run_feature(dev);

#ifdef CONFIG_FB_DYNAMIC_CLK_SUPPORT
	sprd_dispc_clk_disable(&dispc_data, SPRD_DYNAMIC_CLK_COUNT);
#endif

	return ret;
}

static int32_t sprd_dispc_check_esd(struct sprd_fbdev *dev)
{
	uint32_t ret = 0;

	if (SPRD_PANEL_IF_DBI == dev->panel_if_type) {
		pr_err("[%s] leave (not support dbi mode now)!\n",
		       __func__);
		ret = -1;
		goto ERROR_CHECK_ESD;
	}
	down(&dev->refresh_lock);
	if (0 == dev->enable) {
		pr_err("[%s] leave (Invalid device status)!\n",
		       __func__);
		ret = -1;
		goto ERROR_CHECK_ESD;
	}

	if (0 == (dev->check_esd_time % 30)) {
		pr_info("[%s] (%d, %d, %d)\n", __func__,
		       dev->check_esd_time, dev->panel_reset_time,
		       dev->reset_dsi_time);
	} else {
		pr_debug("[%s] (%d, %d, %d)\n", __func__,
			 dev->check_esd_time, dev->panel_reset_time,
			 dev->reset_dsi_time);
	}
	if (SPRD_PANEL_IF_DPI == dev->panel_if_type)
		ret = sprd_dispc_check_esd_dpi(dev);
	else
		ret = sprd_dispc_check_esd_edpi(dev);

ERROR_CHECK_ESD:
	up(&dev->refresh_lock);

	return ret;
}
#endif

#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
static int sprd_overlay_open(void)
{
	dispc_data.overlay_state = SPRD_OVERLAY_STATUS_ON;
	return 0;
}

static int sprd_overlay_start(struct sprd_fbdev *dev, uint32_t layer_index)
{
	pr_debug("[%s] : %d, %d\n", __func__,
		 dispc_data.overlay_state, layer_index);

	if (SPRD_OVERLAY_STATUS_ON != dispc_data.overlay_state) {
		pr_err("overlay start fail. (not opened)");
		return -1;
	}

	if ((0 == dispc_read(DISPC_IMG_Y_BASE_ADDR))
	    && (0 == dispc_read(DISPC_OSD_BASE_ADDR))) {
		pr_err("overlay start fail. (not configged)");
		return -1;
	}

	sprd_dispc_set_bg_color(0x0);
	/* use pixel alpha */
	dispc_clear_bits(BIT(2), DISPC_OSD_CTRL);
	sprd_dispc_set_osd_alpha(0x80);

	if ((layer_index & SPRD_LAYER_IMG)
	    && (0 != dispc_read(DISPC_IMG_Y_BASE_ADDR))) {
		/* enable the image layer */
		dispc_set_bits(BIT(0), DISPC_IMG_CTRL);
	}
	if ((layer_index & SPRD_LAYER_OSD)
	    && (0 != dispc_read(DISPC_OSD_BASE_ADDR))) {
		/* enable the osd layer */
		dispc_set_bits(BIT_DISPC_OSD_OSD_EN, DISPC_OSD_CTRL);
	}
	dispc_data.overlay_state = SPRD_OVERLAY_STATUS_STARTED;
	return 0;
}

static int sprd_overlay_img_configure(struct sprd_fbdev *dev, int type,
		struct overlay_rect *rect, unsigned char *buffer,
		int y_endian, int uv_endian, bool rb_switch)
{
	uint32_t reg_value;
	long lng_addr = (long)buffer;
	uint32_t phys_addr  = (uint32_t)lng_addr;

	pr_debug("[%s] : %d, (%d, %d,%d,%d), 0x%x\n", __func__,
		 type, rect->x, rect->y, rect->h, rect->w,
		 (unsigned int)phys_addr);

	if (SPRD_OVERLAY_STATUS_ON != dispc_data.overlay_state) {
		pr_err("Overlay config fail (not opened)");
		return -1;
	}

	if (type >= SPRD_OSD_LAYER_LIMIT) {
		pr_err("Overlay config fail (type error)");
		return -1;
	}

	if ((y_endian >= SPRD_IMG_DATA_ENDIAN_LIMIT)
	    || (uv_endian >= SPRD_IMG_DATA_ENDIAN_LIMIT)) {
		pr_err
		       ("Overlay config fail (y, uv endian error)");
		return -1;
	}

	reg_value = (y_endian << 8) | (uv_endian << 10) | (type << 4);
	if (rb_switch)
		reg_value |= (1 << 15);
	dispc_write(reg_value, DISPC_IMG_CTRL);

	dispc_write((uint32_t)phys_addr, DISPC_IMG_Y_BASE_ADDR);
	if (type < SPRD_OSD_LAYER_RGB888) {
		uint32_t size = rect->w * rect->h;

		dispc_write((uint32_t)(phys_addr + size),
					DISPC_IMG_UV_BASE_ADDR);
	}

	reg_value = (rect->h << 16) | (rect->w);
	dispc_write(reg_value, DISPC_IMG_SIZE_XY);

	dispc_write(rect->w, DISPC_IMG_PITCH);

	reg_value = (rect->y << 16) | (rect->x);
	dispc_write(reg_value, DISPC_IMG_DISP_XY);

	if (type < SPRD_OSD_LAYER_RGB888) {
		dispc_write(1, DISPC_Y2R_CTRL);
		dispc_write(SPRD_FB_BRIGHTNESS | SPRD_FB_CONTRAST,
			    DISPC_Y2R_Y_PARAM);
		dispc_write(SPRD_FB_OFFSET_U | SPRD_FB_SATURATION_U,
			    DISPC_Y2R_U_PARAM);
		dispc_write(SPRD_FB_OFFSET_V | SPRD_FB_SATURATION_V,
			    DISPC_Y2R_V_PARAM);
	}

	pr_debug("DISPC_IMG_CTRL: 0x%x\n", dispc_read(DISPC_IMG_CTRL));
	pr_debug("DISPC_IMG_Y_BASE_ADDR: 0x%x\n",
		 dispc_read(DISPC_IMG_Y_BASE_ADDR));
	pr_debug("DISPC_IMG_UV_BASE_ADDR: 0x%x\n",
		 dispc_read(DISPC_IMG_UV_BASE_ADDR));
	pr_debug("DISPC_IMG_SIZE_XY: 0x%x\n",
		 dispc_read(DISPC_IMG_SIZE_XY));
	pr_debug("DISPC_IMG_PITCH: 0x%x\n",
		 dispc_read(DISPC_IMG_PITCH));
	pr_debug("DISPC_IMG_DISP_XY: 0x%x\n",
		 dispc_read(DISPC_IMG_DISP_XY));
	pr_debug("DISPC_Y2R_CTRL: 0x%x\n", dispc_read(DISPC_Y2R_CTRL));
	pr_debug("DISPC_Y2R_Y_PARAM: 0x%x\n",
		 dispc_read(DISPC_Y2R_Y_PARAM));
	pr_debug("DISPC_Y2R_U_PARAM: 0x%x\n",
		 dispc_read(DISPC_Y2R_U_PARAM));
	pr_debug("DISPC_Y2R_V_PARAM: 0x%x\n",
		 dispc_read(DISPC_Y2R_V_PARAM));
	return 0;
}

static int sprd_overlay_osd_configure(struct sprd_fbdev *dev, int type,
			struct overlay_rect *rect, unsigned char *buffer,
			int y_endian, int uv_endian, bool rb_switch)
{
	uint32_t reg_value;
	long lng_addr = (long)buffer;
	uint32_t phys_addr  = (uint32_t)lng_addr;

	pr_debug("[%s] : %d, (%d, %d,%d,%d), 0x%x\n", __func__,
		 type, rect->x, rect->y, rect->h, rect->w,
		 (unsigned int)phys_addr);

	if (SPRD_OVERLAY_STATUS_ON != dispc_data.overlay_state) {
		pr_err("Overlay config fail (not opened)");
		return -1;
	}

	if ((type >= SPRD_OSD_LAYER_LIMIT) || (type <= SPRD_OSD_LAYER_YUV400)) {
		pr_err("Overlay config fail (type error)");
		return -1;
	}

	if (y_endian >= SPRD_IMG_DATA_ENDIAN_LIMIT) {
		pr_err
		       ("Overlay config fail (rgb endian error)");
		return -1;
	}

	/* use premultiply pixel alpha */
	reg_value = (y_endian << 8) | (type << 4 | (1 << 2)) | (2 << 16);
	if (rb_switch)
		reg_value |= BIT_DISPC_OSD_RB_SWITCH;
	dispc_write(reg_value, DISPC_OSD_CTRL);

	dispc_write((uint32_t)phys_addr, DISPC_OSD_BASE_ADDR);

	reg_value = BIT_DISPC_OSD_SIZE_X(rect->w) |
			BIT_DISPC_OSD_SIZE_Y(rect->h << 16);
	dispc_write(reg_value, DISPC_OSD_SIZE_XY);

	dispc_write(rect->w, DISPC_OSD_PITCH);

	reg_value = (rect->y << 16) | (rect->x);
	dispc_write(reg_value, DISPC_OSD_DISP_XY);

	pr_debug("DISPC_OSD_CTRL: 0x%x\n", dispc_read(DISPC_OSD_CTRL));
	pr_debug("DISPC_OSD_BASE_ADDR: 0x%x\n",
		 dispc_read(DISPC_OSD_BASE_ADDR));
	pr_debug("DISPC_OSD_SIZE_XY: 0x%x\n",
		 dispc_read(DISPC_OSD_SIZE_XY));
	pr_debug("DISPC_OSD_PITCH: 0x%x\n",
		 dispc_read(DISPC_OSD_PITCH));
	pr_debug("DISPC_OSD_DISP_XY: 0x%x\n",
		 dispc_read(DISPC_OSD_DISP_XY));

	return 0;
}

static int sprd_overlay_close(struct sprd_fbdev *dev)
{
	if (SPRD_OVERLAY_STATUS_OFF == dispc_data.overlay_state) {
		pr_err
		       ("overlay close fail. (has been closed)");
		return 0;
	}

	sprd_dispc_set_bg_color(0x0);
	/* use block alpha */
	dispc_set_bits(BIT(2), DISPC_OSD_CTRL);
	sprd_dispc_set_osd_alpha(0xff);
	/* disable the image layer */
	dispc_clear_bits(BIT(0), DISPC_IMG_CTRL);
	dispc_write(0, DISPC_IMG_Y_BASE_ADDR);
	dispc_write(0, DISPC_OSD_BASE_ADDR);
	sprd_dispc_layer_init(&(dev->fb->var));
	dispc_data.overlay_state = SPRD_OVERLAY_STATUS_OFF;

	return 0;
}

/*TODO: need mutext with suspend, resume*/
static int32_t sprd_dispc_enable_overlay(struct sprd_fbdev *dev,
					   struct overlay_info *info,
					   int enable)
{
	int result = -1;
#ifdef CONFIG_FB_DYNAMIC_CLK_SUPPORT
	bool is_clk_enable = false;
#endif

	pr_debug("[%s]: %d, %d\n", __func__, enable, dev->enable);

	if (enable) {
		if (NULL == info) {
			pr_err(
			       "sprd_dispc_enable_overlay fail (Invalid parameter)\n");
			goto ERROR_ENABLE_OVERLAY;
		}

		down(&dev->refresh_lock);

		if (0 == dev->enable) {
			pr_err(
			       "sprd_dispc_enable_overlay fail. (dev not enable)\n");
			goto ERROR_ENABLE_OVERLAY;
		}

		if (0 != sprd_dispc_sync(dev)) {
			pr_err(
			       "sprd_dispc_enable_overlay fail. (wait done fail)\n");
			goto ERROR_ENABLE_OVERLAY;
		}
#ifdef CONFIG_FB_DYNAMIC_CLK_SUPPORT
		if (SPRD_PANEL_IF_DPI != dev->panel_if_type) {
			if (sprd_dispc_clk_enable
			    (&dispc_data, SPRD_DYNAMIC_CLK_COUNT)) {
				pr_warn(
				       "[%s] clk enable fail!!!\n",
				       __func__);
				goto ERROR_ENABLE_OVERLAY;
			}
			is_clk_enable = true;
		}
#endif
		if (SPRD_OVERLAY_STATUS_STARTED == dispc_data.overlay_state)
			sprd_overlay_close(dev);
		result = sprd_overlay_open();
		if (0 != result) {
			result = -1;
			goto ERROR_ENABLE_OVERLAY;
		}

		if (SPRD_LAYER_IMG == info->layer_index) {
			result =
			    sprd_overlay_img_configure(dev, info->data_type,
						  &(info->rect), info->buffer,
						  info->y_endian,
						  info->uv_endian,
						  info->rb_switch);
		} else if (SPRD_LAYER_OSD == info->layer_index) {
			result =
			    sprd_overlay_osd_configure(dev, info->data_type,
						  &(info->rect), info->buffer,
						  info->y_endian,
						  info->uv_endian,
						  info->rb_switch);
		} else {
			pr_err(
			       "sprd_dispc_enable_overlay fail. (invalid layer index)\n");
		}
		if (0 != result) {
			result = -1;
			goto ERROR_ENABLE_OVERLAY;
		}
	}
ERROR_ENABLE_OVERLAY:
#ifdef CONFIG_FB_DYNAMIC_CLK_SUPPORT
	if (is_clk_enable)
		sprd_dispc_clk_disable(&dispc_data, SPRD_DYNAMIC_CLK_COUNT);
#endif
	up(&dev->refresh_lock);

	return result;
}

static int32_t sprd_dispc_display_overlay(struct sprd_fbdev *dev,
					    struct overlay_display *setting)
{
	struct overlay_rect *rect = &(setting->rect);
	uint32_t size;

	dispc_data.dev = dev;

	pr_debug
	    ("sprd_dispc_display_overlay: layer:%d, (%d, %d,%d,%d)\n",
	     setting->layer_index, setting->rect.x, setting->rect.y,
	     setting->rect.h, setting->rect.w);

	down(&dev->refresh_lock);
	if (0 == dev->enable) {
		pr_err("[%s] leave (Invalid device status)!\n",
		       __func__);
		goto ERROR_DISPLAY_OVERLAY;
	}
	if (SPRD_PANEL_IF_DPI != dev->panel_if_type) {
		dispc_data.vsync_waiter++;
		sprd_dispc_sync(dev);
#ifdef CONFIG_FB_DYNAMIC_CLK_SUPPORT
		if (sprd_dispc_clk_enable
		    (&dispc_data, SPRD_DYNAMIC_CLK_REFRESH)) {
			pr_warn("[%s] clk enable fail!!!\n",
			       __func__);
			goto ERROR_DISPLAY_OVERLAY;
		}
#endif

	}
	pr_debug("[%s] got sync\n", __func__);

	dispc_data.dev = dev;
	size = BIT_DISPC_SIZE_X(rect->w) | BIT_DISPC_SIZE_Y(rect->h);
	dispc_write(size, DISPC_SIZE_XY);

	if (SPRD_PANEL_IF_DPI != dev->panel_if_type)
		sprd_panel_invalidate(dev->panel);

	sprd_panel_before_refresh(dev);

	dispc_clear_bits(BIT(0), DISPC_OSD_CTRL);
	if (SPRD_OVERLAY_STATUS_ON == dispc_data.overlay_state) {
		if (sprd_overlay_start(dev, setting->layer_index) != 0) {
			pr_err("%s[%d] err, return without run dispc!\n",
			     __func__, __LINE__);
			goto ERROR_DISPLAY_OVERLAY;
		}
	}

	dev->frame_count += 1;
	sprd_dispc_run(dev);

	if ((SPRD_OVERLAY_DISPLAY_SYNC == setting->display_mode)
	    && (SPRD_PANEL_IF_DPI != dev->panel_if_type)) {
		dispc_data.vsync_waiter++;
		/* time out -?? disable -?? */
		if (sprd_dispc_sync(dev) != 0)
			pr_err("do sprd_lcdc_display_overlay  time out!\n");
	/* dispc_data.vsync_done = 0; */
	}

ERROR_DISPLAY_OVERLAY:
	up(&dev->refresh_lock);

	pr_debug("DISPC_CTRL: 0x%x\n", dispc_read(DISPC_CTRL));
	pr_debug("DISPC_SIZE_XY: 0x%x\n", dispc_read(DISPC_SIZE_XY));

	pr_debug("DISPC_BG_COLOR: 0x%x\n", dispc_read(DISPC_BG_COLOR));

	pr_debug("DISPC_INT_EN: 0x%x\n", dispc_read(DISPC_INT_EN));

	pr_debug("DISPC_OSD_CTRL: 0x%x\n", dispc_read(DISPC_OSD_CTRL));
	pr_debug("DISPC_OSD_BASE_ADDR: 0x%x\n",
		 dispc_read(DISPC_OSD_BASE_ADDR));
	pr_debug("DISPC_OSD_SIZE_XY: 0x%x\n",
		 dispc_read(DISPC_OSD_SIZE_XY));
	pr_debug("DISPC_OSD_PITCH: 0x%x\n",
		 dispc_read(DISPC_OSD_PITCH));
	pr_debug("DISPC_OSD_DISP_XY: 0x%x\n",
		 dispc_read(DISPC_OSD_DISP_XY));
	pr_debug("DISPC_OSD_ALPHA	: 0x%x\n",
		 dispc_read(DISPC_OSD_ALPHA));
	return 0;
}

#endif

#ifdef CONFIG_FB_VSYNC_SUPPORT
static int32_t sprd_dispc_wait_for_vsync(struct sprd_fbdev *dev)
{
	int ret = 0;
	uint32_t reg_val0, reg_val1;

	if (SPRD_PANEL_IF_DPI == dev->panel_if_type) {
		if (!dispc_data.is_first_frame) {
			dispc_data.waitfor_vsync_done = 0;
			dispc_data.waitfor_vsync_waiter++;
			if (dev->dev_id == SPRD_FB_FPGA)
				/* fpga dpi clock is 6-7MHz, timeout should
				* be longer than reference phone */
				ret = wait_event_interruptible_timeout
					(dispc_data.waitfor_vsync_queue,
					dispc_data.waitfor_vsync_done,
					msecs_to_jiffies(900));
			else
				ret = wait_event_interruptible_timeout
					(dispc_data.waitfor_vsync_queue,
					dispc_data.waitfor_vsync_done,
					msecs_to_jiffies(100));

			if (!ret) {
				/* time out */
				dispc_data.waitfor_vsync_done = 1;
				reg_val0 = dispc_read(DISPC_INT_RAW);
				reg_val1 = dispc_read(DISPC_DPI_STS1);
				pr_err(
				       "vsync time out!!!!!(0x%x, 0x%x)\n",
				       reg_val0, reg_val1);
				{
					/* for debug */
					int32_t i = 0;

					for (i = 0; i < 256; i += 16) {
						pr_debug(
						    "%x: 0x%x, 0x%x, 0x%x, 0x%x\n",
						     i, dispc_read(i),
						     dispc_read(i + 4),
						     dispc_read(i + 8),
						     dispc_read(i + 12));
					}
					pr_debug(
					    "**************************************\n");
				}
			}
			dispc_data.waitfor_vsync_waiter = 0;
		} else {
			msleep(16);
		}

	} else {
		dispc_data.waitfor_vsync_done = 0;
		dispc_set_bits(BIT(1), DISPC_INT_EN);
		dispc_data.waitfor_vsync_waiter++;
		ret =
		    wait_event_interruptible_timeout
		    (dispc_data.waitfor_vsync_queue,
		     dispc_data.waitfor_vsync_done, msecs_to_jiffies(100));

		if (!ret) {
			/* time out */
			dispc_data.waitfor_vsync_done = 1;
			pr_err("vsync time out!!!!!\n");
			{
				/* for debug */
				int32_t i = 0;

				for (i = 0; i < 256; i += 16) {
					pr_debug(
					    "%x: 0x%x, 0x%x, 0x%x, 0x%x\n",
					     i, dispc_read(i),
					     dispc_read(i + 4),
					     dispc_read(i + 8),
					     dispc_read(i + 12));
				}
				pr_debug(
				    "**************************************\n");
			}
		}
		dispc_data.waitfor_vsync_waiter = 0;
	}
	pr_debug("[%s] (%d)\n", __func__, ret);

	return 0;
}
#endif

#if defined(CONFIG_FB_DYNAMIC_FREQ_SCALING) || defined(CONFIG_FB_ESD_SUPPORT)
static void sprd_dispc_stop_feature(struct sprd_fbdev *dev)
{
	int wait_count = 0;

	if (SPRD_PANEL_IF_DPI == dev->panel_if_type) {
		sprd_dispc_stop(dev);
		while ((dispc_read(DISPC_DPI_STS1) & BIT(16))
		       && (wait_count < 100)) {
			udelay(1000);
			wait_count++;
		}
		if (wait_count >= 100) {
			pr_warn("[%s] can't wait till dispc stop!!!\n",
			       __func__);
		}
		udelay(25);
	}
}

static void sprd_dispc_run_feature(struct sprd_fbdev *dev)
{
#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
	if (dispc_data.overlay_state != SPRD_OVERLAY_STATUS_ON)
#endif
		sprd_dispc_run(dev);
}
#endif

#ifdef CONFIG_FB_DYNAMIC_FREQ_SCALING
/**
 * sprd_dispc_chg_clk - interface for sysfs to change dpi or dphy clock
 *
 * @fb_dev: spreadtrum specific fb device
 * @type: check the type is fps or pclk or mipi dphy freq
 * @new_val: fps or new dpi clock
 *
 * Returns 0 on success, MINUS on parsing error.
 */
int sprd_dispc_chg_clk(struct sprd_fbdev *fb_dev, int type, u32 new_val)
{
	int ret = 0;
	unsigned long flags = 0;
	struct panel_spec *panel = fb_dev->panel;

	/* check if new value is valid */
	if (new_val <= 0) {
		pr_err("new value is invalid\n");
		return -EINVAL;
	}

	down(&fb_dev->refresh_lock);
	/* Now let's do update dpi clock */
	switch (type) {
	case SPRD_FB_DYNAMIC_PCLK:
		if (fb_dev->panel_if_type != SPRD_PANEL_IF_DPI) {
			pr_err("current dispc interface isn't DPI\n");
			ret = -EINVAL;
			goto exit;
		}
		/* Note: local interrupt will be disabled */
		local_irq_save(flags);
		sprd_dispc_stop_feature(fb_dev);
		ret = sprd_dispc_update_clk(fb_dev, new_val,
				SPRD_FB_DYNAMIC_PCLK);
		break;

	case SPRD_FB_DYNAMIC_FPS:
		if (fb_dev->panel_if_type != SPRD_PANEL_IF_DPI) {
			dispc_data.vsync_waiter++;
			sprd_dispc_sync(fb_dev);
			sprd_panel_change_fps(fb_dev, new_val);
			ret = 0;
			goto exit;
		}
		/* Note: local interrupt will be disabled */
		local_irq_save(flags);
		sprd_dispc_stop_feature(fb_dev);
		ret = sprd_dispc_update_clk(fb_dev, new_val,
				SPRD_FB_DYNAMIC_FPS);
		break;

	case SPRD_FB_DYNAMIC_MIPI_CLK:
		/* Note: local interrupt will be disabled */
		local_irq_save(flags);
		sprd_dispc_stop_feature(fb_dev);
		ret = sprd_dispc_update_clk(fb_dev, new_val,
				SPRD_FB_DYNAMIC_MIPI_CLK);
		break;

	default:
		pr_err("[%s]: Invalid CMD type\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	if (ret) {
		pr_err("Failed to set pixel clock, fps or dphy freq.\n");
		goto DONE;
	}
	if (type != SPRD_FB_DYNAMIC_MIPI_CLK &&
		panel->type == LCD_MODE_DSI &&
		panel->info.mipi->work_mode ==
		SPRD_MIPI_MODE_VIDEO && fb_dev->enable == true) {
		ret = dsi_dpi_init(fb_dev);
		if (ret)
			pr_err("[%s]: dsi_dpi_init fail\n", __func__);
	}

DONE:
	sprd_dispc_run_feature(fb_dev);
	local_irq_restore(flags);
	up(&fb_dev->refresh_lock);

	return ret;

exit:
	up(&fb_dev->refresh_lock);
	return ret;

}
#endif

int32_t sprd_is_refresh_done(struct sprd_fbdev *dev)
{
	pr_debug("sprd_is_refresh_done vsync_done=%d",
	       dispc_data.vsync_done);
	return (int32_t) dispc_data.vsync_done;
}

struct display_ctrl sprd_fb_dispc_ctrl = {
	.name = "dispc",
	.early_init = sprd_dispc_early_init,
	.init = sprd_dispc_init,
	.uninit = sprd_dispc_uninit,
	.refresh = sprd_dispc_refresh,
	.suspend = sprd_dispc_suspend,
	.resume = sprd_dispc_resume,
	.update_clk = sprd_dispc_update_clk_intf,
#ifdef CONFIG_FB_ESD_SUPPORT
	.ESD_check = sprd_dispc_check_esd,
#endif
#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
	.enable_overlay = sprd_dispc_enable_overlay,
	.display_overlay = sprd_dispc_display_overlay,
#endif
#ifdef CONFIG_FB_VSYNC_SUPPORT
	.wait_for_vsync = sprd_dispc_wait_for_vsync,
#endif
	.is_refresh_done = sprd_is_refresh_done,

};

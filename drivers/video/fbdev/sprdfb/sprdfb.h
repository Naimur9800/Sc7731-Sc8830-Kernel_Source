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

#ifndef _SPRDFB_H_
#define _SPRDFB_H_

#include <linux/compat.h>
#include <linux/semaphore.h>
#include <linux/workqueue.h>
#include <linux/wait.h>

#define FB_CHECK_ESD_IN_VFP
#define BIT_PER_PIXEL_SUPPORT

enum {
	SPRD_FB_NORMAL = 0x1,
	SPRD_FB_FPGA = 0x2
};

enum {
	SPRD_PANEL_IF_DBI = 0,
	SPRD_PANEL_IF_DPI,
	SPRD_PANEL_IF_EDPI,
	SPRD_PANEL_IF_LIMIT
};

enum {
	SPRD_FB_UNKNOWN = 0,
	SPRD_FB_DYNAMIC_PCLK = 0x1,
	SPRD_FB_DYNAMIC_FPS,
	SPRD_FB_DYNAMIC_MIPI_CLK,
	SPRD_FB_FORCE_FPS,
	SPRD_FB_FORCE_PCLK,
};

enum {
	MCU_LCD_REGISTER_TIMING = 0,
	MCU_LCD_GRAM_TIMING,
	MCU_LCD_TIMING_KIND_MAX
};

enum {
	RGB_LCD_H_TIMING = 0,
	RGB_LCD_V_TIMING,
	RGB_LCD_TIMING_KIND_MAX
};

enum{
	MIPI_PLL_TYPE_NONE = 0,
	MIPI_PLL_TYPE_1,
	MIPI_PLL_TYPE_2,
	MIPI_PLL_TYPE_LIMIT
};

enum SPRD_DYNAMIC_CLK_SWITCH {
	SPRD_DYNAMIC_CLK_FORCE,
	SPRD_DYNAMIC_CLK_REFRESH,
	SPRD_DYNAMIC_CLK_COUNT,
	SPRD_DYNAMIC_CLK_MAX,
};

enum SPRD_OSD_ALPHA_SEL {
	SPRD_OSD_PIXEL_ALPHA = 0,
	SPRD_OSD_BLOCK_ALPHA,
	SPRD_OSD_COMBO_ALPHA,
	SPRD_OSD_RESERVED,
};

enum SPRD_OSD_DATA_FORMAT {
	SPRD_OSD_LAYER_YUV422 = 0,
	SPRD_OSD_LAYER_YUV420 = 1,
	SPRD_OSD_LAYER_YUV400 = 2,
	SPRD_OSD_LAYER_RGB888 = 3,
	SPRD_OSD_LAYER_RGB666 = 4,
	SPRD_OSD_LAYER_RGB565 = 5,
	SPRD_OSD_LAYER_RGB555 = 6,
	SPRD_OSD_LAYER_LIMIT = 7,
};

enum SPRD_OSD_SWITCH__ENDIAN {
	SPRD_OSD_SWITCH_B0B1B2B3 = 0,
	SPRD_OSD_SWITCH_B3B2B1B0,
	SPRD_OSD_SWITCH_B2B3B0B1,
	SPRD_OSD_SWITCH_B1B0B3B2,
	SPRD_OSD_SWITCH_LIMIT
};

enum SPRD_IMG_DATA_ENDIAN {
	SPRD_IMG_DATA_ENDIAN_B0B1B2B3 = 0,
	SPRD_IMG_DATA_ENDIAN_B3B2B1B0,
	SPRD_IMG_DATA_ENDIAN_B2B3B0B1,
	SPRD_IMG_DATA_ENDIAN_B1B0B3B2,
	SPRD_IMG_DATA_ENDIAN_LIMIT
};

#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
/* support YUV & RGB */
#define SPRD_LAYER_IMG (0x01)
/* support RGB only */
#define SPRD_LAYER_OSD (0x02)

enum {
	SPRD_OVERLAY_STATUS_OFF = 0,
	SPRD_OVERLAY_STATUS_ON,
	SPRD_OVERLAY_STATUS_STARTED,
	SPRD_OVERLAY_STATUS_MAX
};

enum {
	SPRD_OVERLAY_DISPLAY_ASYNC = 0,
	SPRD_OVERLAY_DISPLAY_SYNC,
	SPRD_OVERLAY_DISPLAY_MAX
};

struct overlay_rect {
	uint16_t x;
	uint16_t y;
	uint16_t w;
	uint16_t h;
};

struct overlay_info {
	int layer_index;
	int data_type;
	int y_endian;
	int uv_endian;
	bool rb_switch;
	struct overlay_rect rect;
	unsigned char *buffer;
};

struct overlay_display {
	int layer_index;
	struct overlay_rect rect;
	int display_mode;
};

#ifdef CONFIG_COMPAT
struct overlay_info32 {
	int layer_index;
	int data_type;
	int y_endian;
	int uv_endian;
	bool rb_switch;
	struct overlay_rect rect;
	compat_caddr_t buffer;
};
#endif
#endif

/**
 * struct fb_capability - define the capability of fb
 * @need_light_sleep: enable light_sleep in fb driver
 * @mipi_pll_type: 0:none, 1:for pike/pikel, 2:for sharkl64/sharklT8
 */
struct fb_capability {
	uint16_t need_light_sleep;
	uint16_t mipi_pll_type;
};

struct sprd_fbdev {
	struct fb_info	*fb;
	struct panel_spec	*panel;
	struct display_ctrl	*ctrl;
	struct device *of_dev;
	struct fb_capability capability;
	struct semaphore refresh_lock;

	/* Use to access global register*/
	struct regmap *aon_apb_gpr;
	struct regmap *ap_ahb_gpr;
	struct regmap *ap_apb_gpr;

	void *priv1;

	/* 0x1 for Normal and 0x2 for FPGA */
	unsigned long dev_id;
	/* Use triple frame buffer*/
	bool is_triple_fb;
	uint16_t enable;
	/* input bit per pixel*/
	uint32_t bpp;
	/* panel has been inited by uboot*/
	uint16_t panel_ready;
	 /* panel IF*/
	uint16_t panel_if_type;

	union{
		uint32_t mcu_timing[MCU_LCD_TIMING_KIND_MAX];
		uint32_t rgb_timing[RGB_LCD_TIMING_KIND_MAX];
	} panel_timing;

	uint32_t dpi_clock;
	uint64_t frame_count;

#ifdef CONFIG_FB_ESD_SUPPORT
	struct delayed_work ESD_work;
	uint32_t ESD_timeout_val;
	bool ESD_work_start;
	/* for debug only */
	uint32_t check_esd_time;
	uint32_t panel_reset_time;
	uint32_t reset_dsi_time;
#ifdef FB_CHECK_ESD_BY_TE_SUPPORT
	uint32_t esd_te_waiter;
	wait_queue_head_t  esd_te_queue;
	uint32_t esd_te_done;
#endif
#endif
};

extern unsigned long sprd_dispc_base;

struct display_ctrl {
	const char *name;

	int32_t	(*early_init)(struct sprd_fbdev *dev);
	int32_t	(*init)(struct sprd_fbdev *dev);
	int32_t	(*uninit)(struct sprd_fbdev *dev);

	int32_t (*refresh)(struct sprd_fbdev *dev);
	void (*logo_proc)(struct sprd_fbdev *dev);

	int32_t	(*suspend)(struct sprd_fbdev *dev);
	int32_t	(*resume)(struct sprd_fbdev *dev);
	int32_t (*update_clk)(struct sprd_fbdev *dev);

#ifdef CONFIG_FB_ESD_SUPPORT
	int32_t	(*ESD_check)(struct sprd_fbdev *dev);
#endif

#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
	int32_t (*enable_overlay)(struct sprd_fbdev *dev,
			struct overlay_info *info, int enable);
	int32_t	(*display_overlay)(struct sprd_fbdev *dev,
			struct overlay_display *setting);
#endif

#ifdef CONFIG_FB_VSYNC_SUPPORT
	int32_t (*wait_for_vsync)(struct sprd_fbdev *dev);
#endif

	int32_t	(*is_refresh_done)(struct sprd_fbdev *dev);
};

#ifndef ROUND
#define ROUND(a, b) (((a) + (b) / 2) / (b))
#endif

/* FB sysfs */
int sprd_fb_create_sysfs(struct sprd_fbdev *fb_dev);
void sprd_fb_remove_sysfs(struct sprd_fbdev *fb_dev);

#ifdef CONFIG_FB_DYNAMIC_FREQ_SCALING
int sprd_dispc_chg_clk(struct sprd_fbdev *fb_dev, int type, u32 new_val);
#else
static inline int sprd_dispc_chg_clk(struct sprd_fbdev *fb_dev, int type, u32 new_val)
{
	int ret = -ENXIO;
	pr_err("dynamical clock feature hasn't been enabled yet\n");
	return ret;
}
#endif

int sprd_dsi_chg_dphy_freq(struct sprd_fbdev *fb_dev, u32 dphy_freq);
int sprd_fb_chg_clk_intf(struct sprd_fbdev *fb_dev, int type, u32 new_val);
int32_t dsi_dpi_init(struct sprd_fbdev *dev);

extern struct display_ctrl sprd_fb_dispc_ctrl;

extern bool sprd_fb_i2c_init(struct sprd_fbdev *dev);
extern bool sprd_fb_i2c_uninit(struct sprd_fbdev *dev);

/* DSI Operations */
extern int32_t sprd_dsi_init(struct sprd_fbdev *dev);
extern int32_t sprd_dsi_uninit(struct sprd_fbdev *dev);
extern int32_t sprd_dsi_ready(struct sprd_fbdev *dev);
extern int32_t sprd_dsi_suspend(struct sprd_fbdev *dev);
extern int32_t sprd_dsi_resume(struct sprd_fbdev *dev);
extern int32_t sprd_dsi_before_panel_reset(struct sprd_fbdev *dev);
extern uint32_t sprd_dsi_get_status(struct sprd_fbdev *dev);
extern int32_t sprd_dsi_enter_ulps(struct sprd_fbdev *dev);
extern void dsi_irq_trick(void);

/* Panel Operations */
extern bool sprd_panel_probe(struct sprd_fbdev *dev);
extern void sprd_panel_remove(struct sprd_fbdev *dev);
extern void sprd_panel_suspend(struct sprd_fbdev *dev);
extern void sprd_panel_resume(struct sprd_fbdev *dev,
			bool from_deep_sleep);
extern void sprd_panel_before_refresh(struct sprd_fbdev *dev);
extern void sprd_panel_after_refresh(struct sprd_fbdev *dev);
#ifdef CONFIG_FB_ESD_SUPPORT
extern uint32_t sprd_panel_ESD_check(struct sprd_fbdev *dev);
#endif

#define SPRD_FB_IOCTL_MAGIC 'm'
#define SPRD_FB_SET_OVERLAY _IOW(SPRD_FB_IOCTL_MAGIC, 1, unsigned int)
#define SPRD_FB_DISPLAY_OVERLAY _IOW(SPRD_FB_IOCTL_MAGIC, 2, unsigned int)
#define SPRD_FB_CHANGE_FPS _IOW(SPRD_FB_IOCTL_MAGIC, 3, unsigned int)
#define SPRD_FB_IS_REFRESH_DONE _IOW(SPRD_FB_IOCTL_MAGIC, 4, unsigned int)
#define SPRD_FB_SET_POWER_MODE _IOW(SPRD_FB_IOCTL_MAGIC, 5, unsigned int)

#endif

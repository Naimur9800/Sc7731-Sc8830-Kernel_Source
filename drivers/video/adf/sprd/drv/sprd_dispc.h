/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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

#ifndef _SPRD_H_
#define _SPRD_H_

#include <linux/compat.h>
#include <linux/workqueue.h>
#include <linux/semaphore.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/irqreturn.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "disp_lib.h"
#include "sprd_panel.h"
#include "sprd_adf_common.h"

#define DISPC_INT_DONE_MASK		BIT(0)
#define DISPC_INT_TE_MASK		BIT(1)
#define DISPC_INT_ERR_MASK		BIT(2)
#define DISPC_INT_EDPI_TE_MASK		BIT(3)
#define DISPC_INT_UPDATE_DONE_MASK	BIT(4)
#define DISPC_INT_DPI_VSYNC_MASK	BIT(5)
#define DISPC_INT_WB_DONE_MASK		BIT(6)
#define DISPC_INT_WB_FAIL_MASK		BIT(7)

/* NOTE: this mask is not a realy dispc interrupt mask */
#define DISPC_INT_FENCE_SIGNAL_REQUEST	BIT(31)

#define ROUND(a, b) (((a) + (b) / 2) / (b))

enum {
	NORMAL_MODE = 0,
	FULL_MODE,
	MIXED_MODE
};

enum {
	SIZE_64 = 0,
	SIZE_128
};

enum {
	SPRD_DISPC_IF_DBI = 0,
	SPRD_DISPC_IF_DPI,
	SPRD_DISPC_IF_EDPI,
	SPRD_DISPC_IF_LIMIT
};

enum {
	SPRD_UNKNOWN = 0,
	SPRD_DYNAMIC_PCLK = 0x1,
	SPRD_DYNAMIC_FPS,
	SPRD_DYNAMIC_MIPI_CLK,
	SPRD_FORCE_FPS,
	SPRD_FORCE_PCLK,
	SPRD_MIPI_SSC,
};

enum {
	SPRD_DATA_TYPE_YUV422 = 0x00,
	SPRD_DATA_TYPE_YUV420 = 0x01,
	SPRD_DATA_TYPE_YUV400 = 0x02,
	SPRD_DATA_TYPE_RGB888 = 0x03,
	SPRD_DATA_TYPE_RGB666 = 0x04,
	SPRD_DATA_TYPE_RGB565 = 0x05,
	SPRD_DATA_TYPE_RGB555 = 0x06,
	SPRD_DATA_TYPE_COMPRESSED = 0x07,
	SPRD_DATA_TYPE_YUV422_3P = 0x08,
	SPRD_DATA_TYPE_YUV420_3P = 0x09,
	SPRD_DATA_TYPE_RGB888_PACK = 0x0A,
	SPRD_DATA_TYPE_AFBC_888 = 0x0B,
	SPRD_DATA_TYPE_LIMIT
};

enum {
	SPRD_IMG_DATA_ENDIAN_B0B1B2B3 = 0,
	SPRD_IMG_DATA_ENDIAN_B3B2B1B0,
	SPRD_IMG_DATA_ENDIAN_B2B3B0B1,
	SPRD_IMG_DATA_ENDIAN_B1B0B3B2,
	SPRD_IMG_DATA_ENDIAN_LIMIT
};

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

enum {
	DISPC_CLK_ID_CORE = 0,
	DISPC_CLK_ID_DBI,
	DISPC_CLK_ID_DPI,
	DISPC_CLK_ID_MAX
};

enum {
	ENHANCE_CFG_ID_ENABLE,
	ENHANCE_CFG_ID_DISABLE,
	ENHANCE_CFG_ID_SCL,
	ENHANCE_CFG_ID_EPF,
	ENHANCE_CFG_ID_HSV,
	ENHANCE_CFG_ID_CM,
	ENHANCE_CFG_ID_SLP,
	ENHANCE_CFG_ID_GAMMA,
	ENHANCE_CFG_ID_MAX
};

struct encoder_context {
	unsigned long base;
	struct panel_info *panel;
	void *data;
};

struct encoder_ops {
	int (*init)(struct encoder_context *ctx);
	void (*uninit)(struct encoder_context *ctx);
	void (*enable)(struct encoder_context *ctx);
	void (*disable)(struct encoder_context *ctx);
};

struct dispc_encoder {
	struct encoder_context ctx;
	struct encoder_ops *ops;
};

struct dispc_context {
	unsigned long base;
	unsigned int  base_offset[2];
	const char *version;
	bool is_inited;
	bool is_stopped;
	bool disable_flip;
	int irq_num;
	int disable_timeout;
	uint32_t id;	/*main_lcd, sub_lcd */
	uint16_t if_type;
	uint32_t dpi_clk_src;
	void *logo_vaddr;
	size_t logo_size;
	dma_addr_t dma_handle;
	struct ion_client *buffer_client;
	struct ion_handle *handle;
	struct panel_info *panel;
	struct semaphore refresh_lock;
	int  vsync_report_rate;
	int  vsync_ratio_to_panel;
	wait_queue_head_t wait_queue;
	struct work_struct wb_work;
	irqreturn_t (*dispc_isr)(int irq, void *data);
};

struct dispc_core_ops {
	int (*parse_dt)(struct dispc_context *ctx,
			struct device_node *np);
	uint32_t (*version)(struct dispc_context *ctx);
	int (*init)(struct dispc_context *ctx);
	void (*uninit)(struct dispc_context *ctx);
	void (*run)(struct dispc_context *ctx);
	void (*stop)(struct dispc_context *ctx);
	int (*disable_vsync)(struct dispc_context *ctx);
	int (*enable_vsync)(struct dispc_context *ctx);
	uint32_t (*isr)(struct dispc_context *ctx);
	void (*ifconfig)(struct dispc_context *ctx);
	void (*write_back)(struct dispc_context *ctx, int enable);
	void (*flip)(struct dispc_context *ctx,
		struct sprd_restruct_config *config);
	void (*bg_color)(struct dispc_context *ctx, uint32_t color);
	void (*enhance_set)(struct dispc_context *ctx, u32 id, void *param);
	void (*enhance_get)(struct dispc_context *ctx, u32 id, void *param);
	int (*modeset)(struct dispc_context *ctx,
			struct drm_mode_modeinfo *mode);
};

struct dispc_clk_ops {
	int (*parse_dt)(struct dispc_context *ctx,
			struct device_node *np);
	int (*init)(struct dispc_context *ctx);
	int (*uinit)(struct dispc_context *ctx);
	int (*enable)(struct dispc_context *ctx);
	int (*disable)(struct dispc_context *ctx);
	int (*update)(struct dispc_context *ctx, int clk_id, int val);
};

struct dispc_glb_ops {
	int (*parse_dt)(struct dispc_context *ctx,
			struct device_node *np);
	void (*enable)(struct dispc_context *ctx);
	void (*disable)(struct dispc_context *ctx);
	void (*reset)(struct dispc_context *ctx);
	void (*noc)(struct dispc_context *ctx, uint32_t mode);
	void (*power)(struct dispc_context *ctx, int enable);
};

#ifdef CONFIG_ZTE_LCD_COMMON_FUNCTION
struct zte_lcd_ctrl_data;
#endif
struct sprd_dispc {
	struct device dev;
	struct device *adf;
	struct dispc_context ctx;
	struct dispc_core_ops *core;
	struct dispc_clk_ops *clk;
	struct dispc_glb_ops *glb;
	struct dispc_encoder *dsc;
	struct dispc_encoder *bv3;
	struct display_ctrl *ctrl;
	struct notifier_block nb;
#ifdef CONFIG_ZTE_LCD_COMMON_FUNCTION
	struct zte_lcd_ctrl_data *zte_lcd_ctrl;
#endif
};

struct display_ctrl {
	const char *name;
	int32_t (*flip)(struct sprd_dispc *dispc,
			struct sprd_restruct_config *config);
	void (*logo_proc)(struct sprd_dispc *dispc);
	void (*enable_hw_vsync)(struct sprd_dispc *dispc);
	void (*disable_hw_vsync)(struct sprd_dispc *dispc);
	int (*modeset)(struct sprd_dispc *dispc,
			struct drm_mode_modeinfo *mode);
};

struct pipe {
	int n_sdev;
	struct sprd_dispc *sdev[2];
};

extern struct list_head dispc_core_head;
extern struct list_head dispc_clk_head;
extern struct list_head dispc_glb_head;
extern struct list_head dispc_enc_head;
extern struct sprd_restruct_config *config_saved;

#define dispc_core_ops_register(entry) \
	disp_ops_register(entry, &dispc_core_head)
#define dispc_clk_ops_register(entry) \
	disp_ops_register(entry, &dispc_clk_head)
#define dispc_glb_ops_register(entry) \
	disp_ops_register(entry, &dispc_glb_head)
#define dispc_enc_ops_register(entry) \
	disp_ops_register(entry, &dispc_enc_head)

#define dispc_core_ops_attach(str) \
	disp_ops_attach(str, &dispc_core_head)
#define dispc_clk_ops_attach(str) \
	disp_ops_attach(str, &dispc_clk_head)
#define dispc_glb_ops_attach(str) \
	disp_ops_attach(str, &dispc_glb_head)
#define dispc_enc_ops_attach(str) \
	disp_ops_attach(str, &dispc_enc_head)

void sprd_dispc_run(struct sprd_dispc *dispc);
void sprd_dispc_stop(struct sprd_dispc *dispc);
void sprd_dispc_black(struct sprd_dispc *dispc);
int sprd_dispc_dynamic_clk(struct sprd_dispc *dispc, int type, u32 new_val);
int sprd_dispc_wb_buf_alloc(struct sprd_dispc *dispc,
			    int heap_type, size_t *size, u32 *buffer);

void sprd_dispc_iommu_map(struct sprd_dispc *dispc,
			  struct sprd_adf_hwlayer *layer);
void sprd_dispc_iommu_unmap(struct sprd_dispc *dispc,
			    struct sprd_adf_hwlayer *hwlayer);
int sprd_dispc_bgcolor(struct sprd_dispc *dispc, u32 color);
void sprd_dispc_disable_flip(struct sprd_dispc *dispc, bool disable);
void sprd_dispc_disable_timeout(struct sprd_dispc *dispc, u32 timeout);
int sprd_dispc_refresh_restore(struct sprd_dispc *dispc);
int sprd_spi_refresh(struct sprd_adf_hwlayer *hwlayer, uint8_t bg_flag);
#endif

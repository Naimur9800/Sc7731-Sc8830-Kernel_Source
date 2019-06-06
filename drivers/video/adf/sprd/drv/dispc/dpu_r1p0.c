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

#include "ion.h"
#include <linux/delay.h>
#include <linux/sprd_ion.h>
#include <linux/sprd_dfs_drv.h>
#include <linux/workqueue.h>
#include "sprd_adf_adapter.h"
#include "sprd_dispc.h"

#define DISPC_BRIGHTNESS           (0x00 << 16)
#define DISPC_CONTRAST             (0x100 << 0)
#define DISPC_OFFSET_U             (0x80 << 16)
#define DISPC_SATURATION_U         (0x100 << 0)
#define DISPC_OFFSET_V             (0x80 << 16)
#define DISPC_SATURATION_V         (0x100 << 0)

typedef struct {
	/* size:12 words*/
	uint32_t addr[4];
	uint32_t ctrl;
	uint32_t size;
	uint32_t pitch;
	uint32_t pos;
	uint32_t alpha;
	uint32_t ck;
	uint32_t pallete;
	uint32_t crop_start;
} reg_layer_t;

typedef struct {
	uint32_t dpu_version;
	uint32_t dpu_ctrl;
	uint32_t dpu_size;
	uint32_t dpu_rstn;
	uint32_t dpu_secure;
	uint32_t dpu_qos;
	uint32_t reserved_0x0018;
	uint32_t bg_color;
	reg_layer_t layers[6];
	/*wb offset: 0x0140*/
	uint32_t wb_base_addr;
	uint32_t wb_ctrl;
	uint32_t wb_pitch;
	uint32_t reserved_0x014c;
	uint32_t y2r_ctrl;
	uint32_t y2r_y_param;
	uint32_t y2r_u_param;
	uint32_t y2r_v_param;
	uint32_t dpu_int_en;
	uint32_t dpu_int_clr;
	uint32_t dpu_int_sts;
	uint32_t dpu_int_raw;
	/*dpi_ctrl offset: 0x0170*/
	uint32_t dpi_ctrl;
	uint32_t dpi_h_timing;
	uint32_t dpi_v_timing;
	uint32_t reserved_0x017c[4];
	uint32_t dpi_sts0;
	uint32_t dpi_sts1;
	uint32_t dpu_sts0;
	uint32_t dpu_sts1;
	uint32_t dpu_sts2;
	uint32_t reserved_0x01a0[24];
	uint32_t dpu_sfbcd_cfg;
	uint32_t dpu_sfbcd_sts;
	uint32_t reserved_0x0208[6];
	reg_layer_t shadow_layers[6];
} dpu_reg_t;

static bool evt_update;
static bool evt_stop;
static int wb_en;
static int max_vsync_count;
static int vsync_count;
static struct sprd_adf_hwlayer *wb_layer;
static bool write_back_afbc = true;

static void dpu_clean(struct dispc_context *ctx);
static void dpu_clean_lite(struct dispc_context *ctx);
static void dpu_write_back(struct dispc_context *ctx, int enable);
static void dpu_layer(struct dispc_context *ctx,
		    struct sprd_adf_hwlayer *hwlayer);

static int32_t dpu_wait_update_done(struct dispc_context *ctx);

static u32 dpu_get_version(struct dispc_context *ctx)
{
	dpu_reg_t *reg = (dpu_reg_t *)ctx->base;

	return reg->dpu_version;
}

static void writeback_update_handler(struct work_struct *data)
{
	int ret;
	struct dispc_context *ctx =
		container_of(data, struct dispc_context, wb_work);
	dpu_reg_t *reg = (dpu_reg_t *)ctx->base;

	ret = down_trylock(&ctx->refresh_lock);
	if (ret != 1) {
		reg->dpu_ctrl |= BIT(5);
		dpu_wait_update_done(ctx);
		up(&ctx->refresh_lock);
	} else
		pr_debug("cannot acquire lock for wb_lock\n");
}

static u32 dpu_isr(struct dispc_context *ctx)
{
	dpu_reg_t *reg = (dpu_reg_t *)ctx->base;
	u32 reg_val;

	if (!reg) {
		pr_err("invalid reg\n");
		return 0;
	}

	reg_val = reg->dpu_int_sts;
	reg->dpu_int_clr = reg_val;

	/*disable err interrupt */
	if (reg_val & DISPC_INT_ERR_MASK)
		reg->dpu_int_en &= ~DISPC_INT_ERR_MASK;

	/*dpu update done isr */
	if (reg_val & DISPC_INT_UPDATE_DONE_MASK) {
		evt_update = true;
		wake_up_interruptible_all(&ctx->wait_queue);
	}

	/*dpu vsync isr */
	if (reg_val & DISPC_INT_DPI_VSYNC_MASK) {
		/*write back feature*/
		if (vsync_count == max_vsync_count && wb_en)
			dpu_write_back(ctx, true);
		vsync_count++;
	}

	/* dpu stop done isr */
	if (reg_val & DISPC_INT_DONE_MASK) {
		ctx->is_stopped = true;
		evt_stop = true;
		wake_up_interruptible_all(&ctx->wait_queue);
	}

	/* dpu write back done isr */
	if (reg_val & DISPC_INT_WB_DONE_MASK) {
		wb_en = false;
		if (vsync_count > max_vsync_count) {
			dpu_layer(ctx, wb_layer);
			dpu_clean_lite(ctx);
		}
		dpu_write_back(ctx, false);
	}

	/* dpu write back error isr */
	if (reg_val & DISPC_INT_WB_FAIL_MASK) {
		pr_err("dpu write back fail\n");
		/*give a new chance to write back*/
		wb_en = true;
		vsync_count = 0;
	}

	return reg_val;
}

static int32_t dpu_wait_stop_done(struct dispc_context *ctx)
{
	int rc;

	/* if this function was called more than once without */
	/* calling dpu_run() in the middle, return directly */
	if (ctx->is_stopped && (!evt_stop)) {
		pr_info("dpu has already stopped!\n");
		return 0;
	}

	/*wait for stop done interrupt*/
	rc = wait_event_interruptible_timeout(ctx->wait_queue, evt_stop,
					       msecs_to_jiffies(2000));
	evt_stop = false;

	if (!rc) {
		/* time out */
		pr_err("dpu wait for stop done time out!\n");
		ctx->is_stopped = true;
		return -1;
	}

	return 0;
}

static int32_t dpu_wait_update_done(struct dispc_context *ctx)
{
	int rc;

	/*wait for reg update done interrupt*/
	rc = wait_event_interruptible_timeout(ctx->wait_queue, evt_update,
					       msecs_to_jiffies(2000));
	evt_update = false;

	if (!rc) {
		/* time out */
		pr_err("dpu wait for reg update done time out!\n");
		return -1;
	}

	return 0;
}

static void dpu_stop(struct dispc_context *ctx)
{
	dpu_reg_t *reg = (dpu_reg_t *)ctx->base;

	if (!reg)
		return;

	if (ctx->if_type == SPRD_DISPC_IF_DPI)
		reg->dpu_ctrl |= BIT(1);

	dpu_wait_stop_done(ctx);
	pr_info("dpu stop\n");

#ifdef CONFIG_DEVFREQ_SPRD_DFS
	scene_exit("dpuhigh");
	scene_exit("dpulow");
#endif
}

static void dpu_run(struct dispc_context *ctx)
{
	dpu_reg_t *reg = (dpu_reg_t *)ctx->base;

	if (!reg)
		return;

	evt_update = false;
	if (ctx->if_type == SPRD_DISPC_IF_DPI) {
		if (ctx->is_stopped) {
			/* set update mode with SW only */
			reg->dpu_ctrl |= BIT(4);

			/* update regs to shadow */
			reg->dpu_ctrl |= BIT(5);

			/* this delay is needed for registers update */
			udelay(10);

			/* start refresh */
			reg->dpu_ctrl |= BIT(0);

			/* set update mode with SW and VSync */
			reg->dpu_ctrl &= ~BIT(4);
			evt_update = false;

			ctx->is_stopped = false;
			pr_info("dpu run\n");
		} else {
			/*dpi register update trigger*/
			reg->dpu_ctrl |= BIT(5);

			/*make sure all the regs are updated to the shadow*/
			dpu_wait_update_done(ctx);
		}

		/* if the underflow err was disabled in isr, re-enable it */
		reg->dpu_int_en |= DISPC_INT_ERR_MASK;

	} else if (ctx->if_type == SPRD_DISPC_IF_EDPI) {
		/* make sure the dpu is in stop status, because shadow regs */
		/* can only be updated in the rising edge of dpu_RUN bit */
		dpu_wait_stop_done(ctx);

		/* start refresh */
		reg->dpu_ctrl |= BIT(0);

		ctx->is_stopped = false;
	}
}

static int dpu_write_back_config(struct dispc_context *ctx)
{
	int ret;
	u32 wb_addr_v;
	static int need_config = 1;
	struct panel_info *panel = ctx->panel;
	struct sprd_dispc *dispc =
		(struct sprd_dispc *)container_of(ctx, struct sprd_dispc, ctx);
	/* afbc buffer need a header 128k */
	u32 header_size = ALIGN(ALIGN(panel->width, 16)
		      * ALIGN(panel->height, 16) / 16, 1024);
	size_t logo_size = ALIGN(panel->width, 16)
		      * ALIGN(panel->height, 16) * 4 + header_size;

	if (!need_config) {
		pr_debug("write back has configed\n");

		return 0;
	}

	wb_en = 0;
	max_vsync_count = 0;
	vsync_count = 0;
	ret = sprd_dispc_wb_buf_alloc(dispc, ION_HEAP_ID_MASK_FB,
					&logo_size, &(wb_addr_v));
	if (!ret) {
		pr_debug("dispc support afbc ?(%d)\n", write_back_afbc);
		wb_layer = devm_kmalloc(&dispc->dev, sizeof(*wb_layer),
					     GFP_KERNEL);
		if (!wb_layer) {
			/* write back fail*/
			max_vsync_count = 0;
			pr_err("cannot use write back\n");

			return -1;
		}

		memset(wb_layer, 0, sizeof(*wb_layer));

		wb_layer->hwlayer_id = 5;
		wb_layer->interface_id = 0;
		wb_layer->n_planes = 1;
		wb_layer->iova_plane[0] = wb_addr_v;
		wb_layer->pitch[0] = ALIGN(panel->width, 16) * 4;
		wb_layer->alpha = 0xff;
		wb_layer->format = DRM_FORMAT_RGBA8888;
		wb_layer->compression = write_back_afbc ? 1 : 0;
		wb_layer->dst_w = panel->width;
		wb_layer->dst_h = panel->height;

		max_vsync_count = 3;
	}
	need_config = 0;

	return 0;
}

static int dpu_init(struct dispc_context *ctx)
{
	dpu_reg_t *reg = (dpu_reg_t *)ctx->base;
	struct panel_info *panel = ctx->panel;
	uint32_t size;

	if (reg == NULL) {
		pr_err("dpu base address is null!");
		return -1;
	}

	if (panel == NULL) {
		pr_err("ctx->panel is null!");
		return -1;
	}

	/* set bg color */
	reg->bg_color = 0;

	/* enable dithering */
	reg->dpu_ctrl |= BIT(6);

	/* enable dpu Power Control */
	reg->dpu_ctrl |= BIT(7);

	/* set dpu output size */
	size = (panel->width & 0xffff) | ((panel->height) << 16);
	reg->dpu_size = size;
	reg->dpu_qos = (14 << 12)|(6 << 8)|(14 << 4)|(11 << 0);
	reg->y2r_ctrl = 1;
	reg->y2r_y_param = DISPC_BRIGHTNESS | DISPC_CONTRAST;
	reg->y2r_u_param = DISPC_OFFSET_U | DISPC_SATURATION_U;
	reg->y2r_v_param = DISPC_OFFSET_V | DISPC_SATURATION_V;
	dpu_write_back_config(ctx);
	INIT_WORK(&ctx->wb_work, writeback_update_handler);

	return 0;
}

static void dpu_uninit(struct dispc_context *ctx)
{
	dpu_reg_t *reg = (dpu_reg_t *)ctx->base;

	if (!reg)
		return;

	reg->dpu_int_en = 0;
	reg->dpu_int_clr = 0xff;
}

enum {
	DPU_LAYER_FORMAT_YUV422,
	DPU_LAYER_FORMAT_YUV420,
	DPU_LAYER_FORMAT_ARGB888,
	DPU_LAYER_FORMAT_RGB565,
	DPU_LAYER_FORMAT_AFBC,
	DPU_LAYER_FORMAT_MAX_TYPES,
};

static uint32_t dpu_img_ctrl(u32 format, u32 blending, u32 compression)
{
	int reg_val = 0;

	/*IMG or OSD layer enable */
	reg_val |= BIT(0);

	switch (format) {
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_RGBX8888:
		/* rb switch */
		reg_val |= BIT(15);
	case DRM_FORMAT_BGRA8888:
		if (compression)
			reg_val |= (DPU_LAYER_FORMAT_AFBC << 4);
		else
		/* ABGR */
			reg_val |= (DPU_LAYER_FORMAT_ARGB888 << 4);
		break;
	case DRM_FORMAT_BGR888:
		/* rb switch */
		reg_val |= BIT(15);
	case DRM_FORMAT_RGB888:
		pr_err("isharkl2 not support rgb888 format\n");
		break;
	case DRM_FORMAT_BGR565:
		/* rb switch */
		reg_val |= BIT(15);
	case DRM_FORMAT_RGB565:
		/* rgb565 */
		reg_val |= (DPU_LAYER_FORMAT_RGB565 << 4);
		/* B2B3B0B1 */
		reg_val |= (SPRD_IMG_DATA_ENDIAN_B0B1B2B3 << 8);
		break;
	case DRM_FORMAT_NV12:
		/*2-Lane: Yuv420 */
		reg_val |= DPU_LAYER_FORMAT_YUV420 << 4;
		/*Y endian */
		reg_val |= SPRD_IMG_DATA_ENDIAN_B3B2B1B0 << 8;
		/*UV endian */
		reg_val |= SPRD_IMG_DATA_ENDIAN_B3B2B1B0 << 10;
		break;
	case DRM_FORMAT_NV21:
		/*2-Lane: Yuv420 */
		reg_val |= DPU_LAYER_FORMAT_YUV420 << 4;
		/*Y endian */
		reg_val |= SPRD_IMG_DATA_ENDIAN_B0B1B2B3 << 8;
		/*UV endian */
		reg_val |= SPRD_IMG_DATA_ENDIAN_B0B1B2B3 << 10;
		break;
	case DRM_FORMAT_NV16:
		/*2-Lane: Yuv422 */
		reg_val |= DPU_LAYER_FORMAT_YUV422 << 4;
		/*Y endian */
		reg_val |= SPRD_IMG_DATA_ENDIAN_B3B2B1B0 << 8;
		/*UV endian */
		reg_val |= SPRD_IMG_DATA_ENDIAN_B3B2B1B0 << 10;
		break;
	default:
		pr_err("Invalid format 0x%x\n", format);
		break;
	}

	switch (blending) {
	case HWC_BLENDING_NONE:
		/*don't do blending, maybe rgbx*/
		/* alpha mode select  - block alpha */
		reg_val |= BIT(2);
		break;
	case HWC_BLENDING_COVERAGE:
		/*Normal mode*/
		reg_val &= (~BIT(16));
		break;
	case HWC_BLENDING_PREMULT:
		/*Pre-mult mode*/
		reg_val |= BIT(16);
		break;
	default:
		/* alpha mode select  - block alpha */
		reg_val |= BIT(2);
		break;
	}

	/*rgbx not support blending alpha*/
	if (format ==  DRM_FORMAT_RGBX8888)
		reg_val |= BIT(2);

	return reg_val;
}

static void dpu_clean(struct dispc_context *ctx)
{
	int i;
	dpu_reg_t *reg = (dpu_reg_t *)ctx->base;

	/*memset(reg->layers, 0, sizeof(reg_layer_t) * 6);*/
	for (i = 0; i < 6; i++)
		reg->layers[i].ctrl = 0;

}
/* clean layer except wb layer */
static void dpu_clean_lite(struct dispc_context *ctx)
{
	int i;
	dpu_reg_t *reg = (dpu_reg_t *)ctx->base;

	for (i = 0; i < 5; i++)
		reg->layers[i].ctrl = 0;

}
static void dpu_layer(struct dispc_context *ctx,
		    struct sprd_adf_hwlayer *hwlayer)
{
	dpu_reg_t *reg = (dpu_reg_t *)ctx->base;
	reg_layer_t *layer;
	int size;
	int offset;
	int wd;

	if (!reg)
		return;
	layer = &reg->layers[hwlayer->hwlayer_id];
	size = (hwlayer->dst_w & 0xffff) | ((hwlayer->dst_h) << 16);
	offset = (hwlayer->dst_x & 0xffff) | ((hwlayer->dst_y) << 16);

	if (adf_format_is_rgb(hwlayer->format))
		wd = adf_format_bpp(hwlayer->format) / 8;
	else
		wd = adf_format_plane_cpp(hwlayer->format, 0);

	memcpy(layer->addr, hwlayer->iova_plane,
	       sizeof(uint32_t) * hwlayer->n_planes);
	layer->pos = offset;
	layer->size = size;
	layer->crop_start = (hwlayer->start_y << 16) | hwlayer->start_x;
	layer->alpha = hwlayer->alpha;
	layer->pitch = hwlayer->pitch[0] / wd;
	layer->ctrl = dpu_img_ctrl(hwlayer->format, hwlayer->blending,
		hwlayer->compression);
}

static void dpu_dpi_init(struct dispc_context *ctx)
{
	dpu_reg_t *reg = (dpu_reg_t *)ctx->base;
	struct panel_info *panel = ctx->panel;
	struct rgb_timing *rgb;
	uint32_t reg_val = 0;
	uint32_t int_mask = 0;
	uint8_t bus_width;

	if (!reg || !panel)
		return;

	/*dpi bits */
	switch (panel->bpp) {
	case 16:
		bus_width = 0;
		break;
	case 18:
		bus_width = 1;
		break;
	case 24:
		bus_width = 2;
		break;
	default:
		bus_width = 2;
		break;
	}

	if (ctx->if_type == SPRD_DISPC_IF_DPI) {
		/*use dpi as interface */
		reg->dpu_ctrl &= ~(BIT(2));
		/* bus width */
		reg_val |= bus_width << 6;
		/* DPI Halt function enable for Synopsys DSI */
		reg_val |= BIT(16);

		reg->dpi_ctrl = reg_val;

		rgb = &panel->rgb_timing;
		/* set dpi timing */
		reg->dpi_h_timing = (rgb->hsync << 0) |
				(rgb->hbp << 8) |
				(rgb->hfp << 20);
		reg->dpi_v_timing = (rgb->vsync << 0) |
				(rgb->vbp << 8) |
				(rgb->vfp << 20);

		/*enable dpu update done INT */
		int_mask |= DISPC_INT_UPDATE_DONE_MASK;
		/* enable dpu DONE  INT */
		int_mask |= DISPC_INT_DONE_MASK;
		/* enable dpu dpi vsync */
		int_mask |= DISPC_INT_DPI_VSYNC_MASK;
		/* enable underflow err INT */
		int_mask |= DISPC_INT_ERR_MASK;
		int_mask |= DISPC_INT_WB_DONE_MASK;
		int_mask |= DISPC_INT_WB_FAIL_MASK;
		int_mask |= DISPC_INT_TE_MASK;

		reg->dpu_int_en = int_mask;
		pr_info("dpu int en (0x%04x) : 0x%x\n",
		      (uint32_t)(&reg->dpu_int_en - &reg->dpu_version),
		      reg->dpu_int_en);

	} else if (ctx->if_type == SPRD_DISPC_IF_EDPI) {
		/*use edpi as interface */
		reg->dpu_ctrl |= BIT(2);

		/*te pol */
		if (panel->te_pol == SPRD_POLARITY_NEG)
			reg_val |= BIT(9);

		/*use external te */
		reg_val |= BIT(10);

		/*enable te */
		reg_val |= BIT(8);

		/* bus width */
		reg_val |= bus_width << 6;

		reg->dpi_ctrl = reg_val;

		/* enable dpu DONE  INT */
		int_mask |= DISPC_INT_DONE_MASK;
		/* enable DISPC TE  INT for edpi*/
		int_mask |= DISPC_INT_TE_MASK;

		reg->dpu_int_en = int_mask;
	}
}

static void dpu_write_back(struct dispc_context *ctx, int enable)
{
	dpu_reg_t *reg = (dpu_reg_t *)ctx->base;
	struct panel_info *panel = ctx->panel;
	u32 header_size;

	reg->wb_base_addr = (u32)wb_layer->iova_plane[0];
	reg->wb_pitch = ALIGN(panel->width, 16);
	/*size unit: 128Byte*/
	header_size = ALIGN(ALIGN(panel->width, 16)
			    * ALIGN(panel->height, 16) / 16, 1024) >> 7;
	if (enable)
		reg->wb_ctrl = write_back_afbc ?
			(BIT(0) | BIT(1) | header_size << 16) : BIT(0);
	else
		reg->wb_ctrl = 0;
	/*update trigger*/
	schedule_work(&ctx->wb_work);
}

#ifdef CONFIG_DEVFREQ_SPRD_DFS
enum {
	DFS_SCENE_INVALID = -1,
	DFS_SCENE_HIGH,
	DFS_SCENE_LOW,
};

static void dpu_scene_frequency(int level)
{
	static int ddr_level = DFS_SCENE_LOW;

	if (level == ddr_level)
		return;

	switch (level) {
	case DFS_SCENE_LOW:
		pr_info("switch to low level ddr frequency\n");
		scene_dfs_request("dpulow");
		scene_exit("dpuhigh");
		break;
	case DFS_SCENE_HIGH:
		pr_info("switch to high level ddr frequency\n");
		scene_dfs_request("dpuhigh");
		scene_exit("dpulow");
		break;
	default:
		pr_err("Invalid dfs scene detected\n");
		break;
	}

	ddr_level = level;
}
#endif

static void dpu_flip(struct dispc_context *ctx,
			struct sprd_restruct_config *config)
{
	int i;
	struct sprd_adf_hwlayer *hwlayer;
#ifdef CONFIG_DEVFREQ_SPRD_DFS
	int dfs_level;
#endif

	vsync_count = 0;
	if (max_vsync_count && (config->number_hwlayer > 1))
		wb_en = true;
	else
		wb_en = false;

	dpu_clean(ctx);

	for (i = 0; i < config->number_hwlayer; i++) {
		hwlayer = &config->hwlayers[i];
		dpu_layer(ctx, hwlayer);
	}

#ifdef CONFIG_DEVFREQ_SPRD_DFS
	/* switch to high level ddr freq when do 5 layers blending*/
	if (config->number_hwlayer < 5)
		dfs_level = DFS_SCENE_LOW;
	else
		dfs_level = DFS_SCENE_HIGH;
	dpu_scene_frequency(dfs_level);
#endif
}

static struct dispc_core_ops dpu_r1p0_ops = {
	.version = dpu_get_version,
	.init = dpu_init,
	.uninit = dpu_uninit,
	.run = dpu_run,
	.stop = dpu_stop,
	.isr = dpu_isr,
	.ifconfig = dpu_dpi_init,
	.flip = dpu_flip,
};

static struct ops_entry entry = {
	.ver = "dpu-r1p0",
	.ops = &dpu_r1p0_ops,
};

static int __init dispc_core_register(void)
{
	return dispc_core_ops_register(&entry);
}

subsys_initcall(dispc_core_register);

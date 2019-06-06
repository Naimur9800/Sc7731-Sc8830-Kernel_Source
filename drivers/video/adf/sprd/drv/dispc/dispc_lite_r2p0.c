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
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd/sharklj1/glb.h>
#include <linux/regmap.h>
#include <linux/sprd_dfs_drv.h>
#include <linux/sprd_ion.h>
#include "sprd_adf_adapter.h"
#include "sprd_dispc.h"

#define DISPC_BRIGHTNESS           (0x00 << 16)
#define DISPC_CONTRAST             (0x100 << 0)
#define DISPC_OFFSET_U             (0x80 << 16)
#define DISPC_SATURATION_U         (0x100 << 0)
#define DISPC_OFFSET_V             (0x80 << 16)
#define DISPC_SATURATION_V         (0x100 << 0)

#define WIDTH_ALIGN(value, base) (((value) + ((base) - 1)) & (~((base) - 1)))
struct reg_layer_t {
	uint32_t addr[4];
	uint32_t ctrl;
	uint32_t size;
	uint32_t pitch;
	uint32_t pos;
	uint32_t alpha;
	uint32_t ck;
	uint32_t pallete;
	uint32_t reserved;
};

typedef struct {
	uint32_t dispc_version;
	uint32_t dispc_ctrl;
	uint32_t dispc_size;
	uint32_t dispc_rstn;
	uint32_t dispc_secure;
	uint32_t dispc_qos;
	uint32_t reserved_0x0018;
	uint32_t bg_color;
	struct	reg_layer_t layers[6];
	uint32_t wb_base_addr;
	uint32_t wb_ctrl;
	uint32_t wb_pitch;
	uint32_t reserved_0x014c;
	uint32_t y2r_ctrl;
	uint32_t y2r_y_param;
	uint32_t y2r_u_param;
	uint32_t y2r_v_param;
	uint32_t dispc_int_en;
	uint32_t dispc_int_clr;
	uint32_t dispc_int_sts;
	uint32_t dispc_int_raw;
	uint32_t dpi_ctrl;
	uint32_t dpi_h_timing;
	uint32_t dpi_v_timing;
	uint32_t reserved_0x017c[4];
	uint32_t dpi_sts0;
	uint32_t dpi_sts1;
	uint32_t dispc_sts0;
	uint32_t dispc_sts1;
	uint32_t dispc_sts2;
} dispc_reg_t;

static bool evt_update;
static bool evt_stop;
static int wb_en;
static int max_vsync_count;
static int vsync_count;
static struct sprd_adf_hwlayer *wb_layer;

static void dispc_clean(struct dispc_context *ctx);
static void dispc_clean_lite(struct dispc_context *ctx);
static void dispc_write_back(struct dispc_context *ctx, int enable);
static void dispc_layer(struct dispc_context *ctx,
			struct sprd_adf_hwlayer *hwlayer);
static int32_t dispc_wait_update_done(struct dispc_context *ctx);

static u32 dispc_get_version(struct dispc_context *ctx)
{
	return 0x0100;
}

static void writeback_update_handler(struct work_struct *data)
{
	int ret;
	struct dispc_context *ctx =
		container_of(data, struct dispc_context, wb_work);
	dispc_reg_t *reg = (dispc_reg_t *)ctx->base;

	ret = down_trylock(&ctx->refresh_lock);
	if (!ctx->is_inited) {
		pr_err("dpu is suspended\n");
		up(&ctx->refresh_lock);
		return;
	}

	if (ret != 1) {
		reg->dispc_ctrl |= BIT(5);
		dispc_wait_update_done(ctx);
		up(&ctx->refresh_lock);
	} else
		pr_debug("cannot acquire lock for wb_lock\n");
}

static u32 dispc_isr(struct dispc_context *ctx)
{
	dispc_reg_t *reg = (dispc_reg_t *)ctx->base;
	u32 reg_val;

	if (!reg) {
		pr_err("invalid reg\n");
		return 0;
	}

	reg_val = reg->dispc_int_sts;
	reg->dispc_int_clr = reg_val;

	/*disable err interrupt */
	if (reg_val & DISPC_INT_ERR_MASK)
		reg->dispc_int_en &= ~DISPC_INT_ERR_MASK;

	/*dispc update done isr */
	if (reg_val & DISPC_INT_UPDATE_DONE_MASK) {
		evt_update = true;
		wake_up_interruptible_all(&ctx->wait_queue);
	}

	/*dispc vsync isr */
	if (reg_val & DISPC_INT_DPI_VSYNC_MASK) {
		/*write back feature*/
		if (vsync_count == max_vsync_count && wb_en)
			dispc_write_back(ctx, true);

		vsync_count++;
	}

	/* dispc stop done isr */
	if (reg_val & DISPC_INT_DONE_MASK) {
		ctx->is_stopped = true;
		evt_stop = true;
		wake_up_interruptible_all(&ctx->wait_queue);
	}

	/* dispc write back done isr */
	if (reg_val & DISPC_INT_WB_DONE_MASK) {
		wb_en = false;
		if (vsync_count > max_vsync_count) {
			dispc_layer(ctx, wb_layer);
			dispc_clean_lite(ctx);
			reg_val |= DISPC_INT_FENCE_SIGNAL_REQUEST;
		}
		dispc_write_back(ctx, false);
	}

	/* dispc write back error isr */
	if (reg_val & DISPC_INT_WB_FAIL_MASK) {
		pr_emerg("dpu write back fail\n");
		/*give a new chance to write back*/
		wb_en = true;
		vsync_count = 0;
	}

	return reg_val;
}

static int32_t dispc_wait_stop_done(struct dispc_context *ctx)
{
	int rc;

	/* if this function was called more than once without */
	/* calling dispc_run() in the middle, return directly */
	if (ctx->is_stopped && (!evt_stop)) {
		pr_info("dispc has already stopped!\n");
		return 0;
	}

	/*wait for stop done interrupt*/
	rc = wait_event_interruptible_timeout(ctx->wait_queue, evt_stop,
					       msecs_to_jiffies(500));
	evt_stop = false;

	if (!rc) {
		/* time out */
		pr_err("dispc wait for stop done time out!\n");
		ctx->is_stopped = true;
		return -1;
	}

	return 0;
}

static int32_t dispc_wait_update_done(struct dispc_context *ctx)
{
	int rc;

	/*wait for reg update done interrupt*/
	rc = wait_event_interruptible_timeout(ctx->wait_queue, evt_update,
					       msecs_to_jiffies(500));
	evt_update = false;

	if (!rc) {
		/* time out */
		pr_err("dispc wait for reg update done time out!\n");
		return -1;
	}

	return 0;
}

static void dispc_stop(struct dispc_context *ctx)
{
	dispc_reg_t *reg = (dispc_reg_t *)ctx->base;

	if (!reg)
		return;

	if (ctx->if_type == SPRD_DISPC_IF_DPI)
		reg->dispc_ctrl |= BIT(1);

	dispc_wait_stop_done(ctx);
	pr_info("dispc stop\n");

#ifdef CONFIG_DEVFREQ_SPRD_DFS
	scene_exit("dpuhigh");
	scene_exit("dpulow");
#endif
}

static void dispc_run(struct dispc_context *ctx)
{
	dispc_reg_t *reg = (dispc_reg_t *)ctx->base;

	if (!reg)
		return;

	evt_update = false;
	if (ctx->if_type == SPRD_DISPC_IF_DPI) {
		if (ctx->is_stopped) {
			/* set update mode with SW only */
			reg->dispc_ctrl |= BIT(4);

			/* update regs to shadow */
			reg->dispc_ctrl |= BIT(5);

			/* this delay is needed for registers update */
			udelay(10);

			/* start refresh */
			reg->dispc_ctrl |= BIT(0);

			/* set update mode with SW and VSync */
			reg->dispc_ctrl &= ~BIT(4);

			evt_update = false;
			ctx->is_stopped = false;

			pr_info("dispc run\n");
		} else {
			/*dpi register update trigger*/
			reg->dispc_ctrl |= BIT(5);

			/*make sure all the regs are updated to the shadow*/
			dispc_wait_update_done(ctx);
		}

		/* if the underflow err was disabled in isr, re-enable it */
		reg->dispc_int_en |= DISPC_INT_ERR_MASK;

	} else if (ctx->if_type == SPRD_DISPC_IF_EDPI) {
		/* make sure the dispc is in stop status, because shadow regs */
		/* can only be updated in the rising edge of DISPC_RUN bit */
		dispc_wait_stop_done(ctx);

		/* start refresh */
		reg->dispc_ctrl |= BIT(0);

		ctx->is_stopped = false;
	}
}

static int dispc_write_back_config(struct dispc_context *ctx)
{
	int ret;
	static int need_config = 1;
	struct panel_info *panel = ctx->panel;
	struct regmap *aon_apb;
	u32 wb_addr_v;
	struct sprd_dispc *dispc =
		(struct sprd_dispc *)container_of(ctx, struct sprd_dispc, ctx);
	size_t logo_size = ALIGN(panel->width, 16) * panel->height * 4;
	u32 chipid = 0;

	if (!need_config) {
		pr_debug("write back info has configed\n");
		return 0;
	}

	aon_apb = syscon_regmap_lookup_by_compatible("sprd,sys-aon-apb");
	if (IS_ERR(aon_apb))
		return PTR_ERR(aon_apb);

	regmap_read(aon_apb,
		REG_AON_APB_AON_VER_ID, &chipid);
	if (chipid == 0) {
		need_config = 0;
		pr_debug("Awrite back info has configed\n");
		return 0;
	}

	wb_en = 0;
	max_vsync_count = 0;
	vsync_count = 0;
	ret = sprd_dispc_wb_buf_alloc(dispc, ION_HEAP_ID_MASK_FB,
					&logo_size, &wb_addr_v);
	if (!ret) {
		wb_layer = devm_kmalloc(&dispc->dev,
					     sizeof(*wb_layer),
					     GFP_KERNEL);
		if (!wb_layer) {
			/* write back fail*/
			max_vsync_count = 0;
			pr_err("cannot use write back\n");
			return -1;
		}

		memset(wb_layer, 0, sizeof(struct sprd_adf_hwlayer));
		wb_layer->hwlayer_id = 5;
		wb_layer->interface_id = 0;
		wb_layer->n_planes = 1;
		wb_layer->iova_plane[0] = wb_addr_v - 0x80000000;
		wb_layer->pitch[0] = ALIGN(panel->width, 16) * 4;
		wb_layer->alpha = 0xff;
		wb_layer->format = DRM_FORMAT_RGBA8888;
		wb_layer->compression = 0;
		wb_layer->dst_w = panel->width;
		wb_layer->dst_h = panel->height;

		/*disable write back feature*/
		max_vsync_count = 3;
	}

	need_config = 0;
	return 0;
}

static int dispc_init(struct dispc_context *ctx)
{
	dispc_reg_t *reg = (dispc_reg_t *)ctx->base;
	struct panel_info *panel = ctx->panel;
	uint32_t size;

	if (reg == NULL) {
		pr_err("dispc base address is null!");
		return -1;
	}

	if (panel == NULL) {
		pr_err("ctx->panel is null!");
		return -1;
	}

	/* set bg color */
	reg->bg_color = 0;

	/* enable dithering */
	reg->dispc_ctrl |= BIT(6);

	/* enable DISPC Power Control */
	reg->dispc_ctrl |= BIT(7);

	/* set dispc output size */
	size = (panel->width & 0xffff) | ((panel->height) << 16);
	reg->dispc_size = size;
	reg->dispc_qos = (14 << 12)|(6 << 8)|(14 << 4)|(11 << 0);
	reg->y2r_ctrl = 1;
	dispc_write_back_config(ctx);
	INIT_WORK(&ctx->wb_work, writeback_update_handler);

	return 0;
}

static void dispc_uninit(struct dispc_context *ctx)
{
	dispc_reg_t *reg = (dispc_reg_t *)ctx->base;

	if (!reg)
		return;

	reg->dispc_int_en = 0;
	reg->dispc_int_clr = 0xff;
}

static uint32_t dispc_img_ctrl(u32 format, u32 blending, u32 compression)
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
		/* ABGR */
		reg_val |= (SPRD_DATA_TYPE_RGB888 << 4);
		break;
	case DRM_FORMAT_BGR888:
		/* rb switch */
		reg_val |= BIT(15);
	case DRM_FORMAT_RGB888:
		if (compression) {
			/* AFBC bit [7:4] 1011 AFBC RGB888*/
			reg_val |= (SPRD_DATA_TYPE_AFBC_888 << 4);
			reg_val &= ~BIT(15);
		} else/* packed rgb888 */
			reg_val |= (SPRD_DATA_TYPE_RGB888_PACK << 4);
		break;
	case DRM_FORMAT_BGR565:
		/* rb switch */
		reg_val |= BIT(15);
	case DRM_FORMAT_RGB565:
		/* rgb565 */
		reg_val |= (SPRD_DATA_TYPE_RGB565 << 4);
		/* B2B3B0B1 */
		reg_val |= (SPRD_IMG_DATA_ENDIAN_B2B3B0B1 << 8);
		break;
	case DRM_FORMAT_NV12:
		/*2-Lane: Yuv420 */
		reg_val |= SPRD_DATA_TYPE_YUV420 << 4;
		/*Y endian */
		reg_val |= SPRD_IMG_DATA_ENDIAN_B3B2B1B0 << 8;
		/*UV endian */
		reg_val |= SPRD_IMG_DATA_ENDIAN_B3B2B1B0 << 10;
		break;
	case DRM_FORMAT_NV21:
		/*2-Lane: Yuv420 */
		reg_val |= SPRD_DATA_TYPE_YUV420 << 4;
		/*Y endian */
		reg_val |= SPRD_IMG_DATA_ENDIAN_B3B2B1B0 << 8;
		/*UV endian */
		reg_val |= SPRD_IMG_DATA_ENDIAN_B2B3B0B1 << 10;
		break;
	case DRM_FORMAT_NV16:
		/*2-Lane: Yuv422 */
		reg_val |= SPRD_DATA_TYPE_YUV422 << 4;
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

	return reg_val;
}

static void dispc_layer(struct dispc_context *ctx,
		    struct sprd_adf_hwlayer *hwlayer)
{
	dispc_reg_t *reg = (dispc_reg_t *)ctx->base;
	int i;
	uint32_t crop_offset;
	int size;
	int offset;
	int wd;
	struct reg_layer_t *layer;

	if (!reg)
		return;

	layer = &(reg->layers[hwlayer->hwlayer_id]);

	size = (hwlayer->dst_w & 0xffff) | ((hwlayer->dst_h) << 16);
	offset = (hwlayer->dst_x & 0xffff) | ((hwlayer->dst_y) << 16);

	if (adf_format_is_rgb(hwlayer->format))
		wd = adf_format_bpp(hwlayer->format) / 8;
	else
		wd = adf_format_plane_cpp(hwlayer->format, 0);
	for (i = 0; i < hwlayer->n_planes; i++) {
		if (i == 0)
			/* x * BPP + y * pitch*/
			crop_offset = hwlayer->start_x * wd
				+ hwlayer->start_y * hwlayer->pitch[0];
		else
			crop_offset = hwlayer->start_x * wd
				+ hwlayer->start_y * hwlayer->pitch[0] / 2;

		hwlayer->iova_plane[i] +=  crop_offset;
		layer->addr[i] = hwlayer->iova_plane[i];
	}

	layer->pos = offset;
	layer->size = size;
	layer->alpha = hwlayer->alpha;
	layer->pitch = hwlayer->pitch[0] / wd;

	layer->ctrl = dispc_img_ctrl(hwlayer->format, hwlayer->blending,
				     hwlayer->compression);
}

static void dispc_clean(struct dispc_context *ctx)
{
	dispc_reg_t *reg = (dispc_reg_t *)ctx->base;
	int i;

	for (i = 0; i < 6; i++)
		reg->layers[i].ctrl = 0;
}

/* clean layer except wb layer */
static void dispc_clean_lite(struct dispc_context *ctx)
{
	int i;
	dispc_reg_t *reg = (dispc_reg_t *)ctx->base;

	for (i = 0; i < 5; i++)
		reg->layers[i].ctrl = 0;

}

static void dispc_dpi_init(struct dispc_context *ctx)
{
	dispc_reg_t *reg = (dispc_reg_t *)ctx->base;
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
		reg->dispc_ctrl &= ~(BIT(1) | BIT(2));

		/*h sync pol */
		if (panel->h_sync_pol == SPRD_POLARITY_NEG)
			reg_val |= BIT(0);

		/*v sync pol */
		if (panel->v_sync_pol == SPRD_POLARITY_NEG)
			reg_val |= BIT(1);

		/*de sync pol */
		if (panel->de_pol == SPRD_POLARITY_NEG)
			reg_val |= BIT(2);

		/* bus width */
		reg_val |= bus_width << 6;

		reg->dpi_ctrl = reg_val;

		rgb = &panel->rgb_timing;
		/* set dpi timing */
		reg->dpi_h_timing = (rgb->hsync << 0) |
				(rgb->hbp << 8) |
				(rgb->hfp << 20);
		reg->dpi_v_timing = (rgb->vsync << 0) |
				(rgb->vbp << 8) |
				(rgb->vfp << 20);

		/*enable dispc update done INT */
		int_mask |= DISPC_INT_UPDATE_DONE_MASK;
		/* enable dispc DONE  INT */
		int_mask |= DISPC_INT_DONE_MASK;
		/* enable dispc dpi vsync */
		int_mask |= DISPC_INT_DPI_VSYNC_MASK;
		/* enable underflow err INT */
		int_mask |= DISPC_INT_ERR_MASK;

		int_mask |= DISPC_INT_WB_DONE_MASK;

		int_mask |= DISPC_INT_WB_FAIL_MASK;

		int_mask |= DISPC_INT_TE_MASK;

		reg->dispc_int_en = int_mask;

	} else if (ctx->if_type == SPRD_DISPC_IF_EDPI) {
		/*use edpi as interface */
		reg->dispc_ctrl |= BIT(1);

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

		/* enable dispc DONE  INT */
		int_mask |= DISPC_INT_DONE_MASK;
		/* enable dispc TE  INT for edpi*/
		int_mask |= DISPC_INT_TE_MASK;

		reg->dispc_int_en = int_mask;
	}

}

static void dispc_write_back(struct dispc_context *ctx, int enable)
{
	dispc_reg_t *reg = (dispc_reg_t *)ctx->base;
	struct panel_info *panel = ctx->panel;

	reg->wb_base_addr = (u32)wb_layer->iova_plane[0];
	reg->wb_pitch = ALIGN(panel->width, 16);

	if (enable)
		reg->wb_ctrl = 1;
	else
		reg->wb_ctrl = 0;

	 schedule_work(&ctx->wb_work);
}

static int disable_vsync(struct dispc_context *ctx)
{
	dispc_reg_t *reg = (dispc_reg_t *)ctx->base;

	if (wb_en)
		return -1;

	reg->dispc_int_en &= ~DISPC_INT_DPI_VSYNC_MASK;
	return 0;
}

static int enable_vsync(struct dispc_context *ctx)
{
	dispc_reg_t *reg = (dispc_reg_t *)ctx->base;

	reg->dispc_int_en |= DISPC_INT_DPI_VSYNC_MASK;
	return 0;
}

#ifdef CONFIG_DEVFREQ_SPRD_DFS
enum {
	DFS_SCENE_INVALID = -1,
	DFS_SCENE_HIGH,
	DFS_SCENE_LOW,
};

static void dispc_scene_frequency(int level)
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

static void dispc_flip(struct dispc_context *ctx,
			struct sprd_restruct_config *config)
{
	dispc_reg_t *reg = (dispc_reg_t *)ctx->base;
	int i;
	struct sprd_adf_hwlayer *hwlayer;
	struct reg_layer_t *layer;
	int swap_id;
#ifdef CONFIG_DEVFREQ_SPRD_DFS
	int dfs_level;
#endif

	vsync_count = 0;
	/*Only one layer, don't do write back operation*/
	if (max_vsync_count && (config->number_hwlayer > 1))
		wb_en = true;
	else
		wb_en = false;

	layer = &(reg->layers[0]);
	swap_id = 0;
	/*check if yuv data on osd layers*/
	for (i = 1; i < config->number_hwlayer; i++) {
		hwlayer = &config->hwlayers[i];
		if (unlikely(!adf_format_is_rgb(hwlayer->format))) {
			swap_id = hwlayer->hwlayer_id;
			hwlayer->hwlayer_id = 0;
			config->hwlayers[0].hwlayer_id = swap_id;
			break;
		}
	}

	dispc_clean(ctx);

	for (i = 0; i < config->number_hwlayer; i++) {
		hwlayer = &config->hwlayers[i];
		dispc_layer(ctx, hwlayer);
	}
	if (unlikely(swap_id))
		layer->ctrl |= (swap_id << 17);

#ifdef CONFIG_DEVFREQ_SPRD_DFS
	/* switch to high level ddr freq when do 4 layers blending*/
	if (config->number_hwlayer < 3)
		dfs_level = DFS_SCENE_LOW;
	else
		dfs_level = DFS_SCENE_HIGH;
	dispc_scene_frequency(dfs_level);
#endif

}

static struct dispc_core_ops dispc_lite_r2p0_ops = {
	.version = dispc_get_version,
	.init = dispc_init,
	.uninit = dispc_uninit,
	.run = dispc_run,
	.stop = dispc_stop,
	.disable_vsync = disable_vsync,
	.enable_vsync = enable_vsync,
	.isr = dispc_isr,
	.flip = dispc_flip,
	.ifconfig = dispc_dpi_init,
	.write_back = dispc_write_back,
};

static struct ops_entry entry = {
	.ver = "dispc-lite-r2p0",
	.ops = &dispc_lite_r2p0_ops,
};

static int __init dispc_core_register(void)
{
	return dispc_core_ops_register(&entry);
}

subsys_initcall(dispc_core_register);

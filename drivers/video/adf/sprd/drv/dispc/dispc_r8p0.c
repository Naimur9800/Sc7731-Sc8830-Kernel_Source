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

#include <linux/delay.h>

#include "sprd_dispc.h"

#define DISPC_BRIGHTNESS           (0x00 << 16)
#define DISPC_CONTRAST             (0x100 << 0)
#define DISPC_OFFSET_U             (0x81 << 16)
#define DISPC_SATURATION_U         (0x100 << 0)
#define DISPC_OFFSET_V             (0x82 << 16)
#define DISPC_SATURATION_V         (0x100 << 0)

#define WIDTH_ALIGN(value, base) (((value) + ((base) - 1)) & (~((base) - 1)))


typedef struct {
	uint32_t dispc_version;
	uint32_t dispc_ctrl;
	uint32_t dispc_size;
	uint32_t dispc_rstn;
	uint32_t reserved_0x0010;
	uint32_t dispc_sts;
	uint32_t dispc_3d_ctrl;
	uint32_t reserved_0x001c;
	uint32_t img_base_addr0;
	uint32_t img_base_addr1;
	uint32_t img_base_addr2;
	uint32_t reserved_0x002c;
	uint32_t img_ctrl;
	uint32_t img_size;
	uint32_t img_pitch;
	uint32_t img_pos;
	uint32_t bg_color;
	uint32_t reserved_0x0044[3];
	uint32_t osd_base_addr;
	uint32_t reserved_0x0054[3];
	uint32_t osd_ctrl;
	uint32_t osd_size;
	uint32_t osd_pitch;
	uint32_t osd_pos;
	uint32_t osd_alpha;
	uint32_t osd_ck;
	uint32_t reserved_0x0078[2];
	uint32_t y2r_ctrl;
	uint32_t y2r_y_param;
	uint32_t y2r_u_param;
	uint32_t y2r_v_param;
	uint32_t y2r_coef_00;
	uint32_t y2r_coef_01;
	uint32_t y2r_coef_10;
	uint32_t y2r_coef_11;
	uint32_t y2r_coef_20;
	uint32_t y2r_coef_21;
	uint32_t reserved_0x00a8[2];
	uint32_t dispc_int_en;
	uint32_t dispc_int_clr;
	uint32_t dispc_int_sts;
	uint32_t dispc_int_raw;
	uint32_t dpi_ctrl;
	uint32_t dpi_h_timing;
	uint32_t dpi_v_timing;
	uint32_t dpi_sts0;
	uint32_t dpi_sts1;
	uint32_t reserved_0x00d4[3];
	uint32_t dbi_ctrl;
	uint32_t dbi_rd_timing0;
	uint32_t dbi_wr_timing0;
	uint32_t dbi_rd_timing1;
	uint32_t dbi_er_timing1;
	uint32_t dbi_rdata;
	uint32_t dbi_cmd;
	uint32_t dbi_data;
	uint32_t dbi_queue;
	uint32_t te_sync_delay;
	uint32_t reserved_0x0100[68];
	uint32_t shdw_3d_ctrl;
	uint32_t reserved_0x021c;
	uint32_t shdw_img_base_addr0;
	uint32_t shdw_img_base_addr1;
	uint32_t shdw_img_base_addr2;
	uint32_t reserved_0x022c;
	uint32_t shdw_img_ctrl;
	uint32_t shdw_img_size;
	uint32_t shdw_img_pitch;
	uint32_t shdw_img_pos;
	uint32_t shdw_bg_color;
	uint32_t reserved_0x0244[3];
	uint32_t shdw_osd_base_addr0;
	uint32_t reserved_0x0254[3];
	uint32_t shdw_osd_ctrl;
	uint32_t shdw_osd_size;
	uint32_t shdw_osd_pitch;
	uint32_t shdw_osd_pos;
	uint32_t shdw_osd_alpha;
	uint32_t shdw_osd_ck;
	uint32_t reserved_0x0278[2];
	uint32_t shdw_y2r_ctrl;
	uint32_t shdw_y2r_y_param;
	uint32_t shdw_y2r_u_param;
	uint32_t shdw_y2r_v_param;
	uint32_t shdw_y2r_coef_00;
	uint32_t shdw_y2r_coef_01;
	uint32_t shdw_y2r_coef_10;
	uint32_t shdw_y2r_coef_11;
	uint32_t shdw_y2r_coef_20;
	uint32_t shdw_y2r_coef_21;
	uint32_t reserved_0x02a8[7];
	uint32_t shdw_dpi_h_timing;
	uint32_t shdw_dpi_v_timing;
} dispc_reg_t;

static bool evt_update;
static bool evt_stop;

static u32 dispc_get_version(struct dispc_context *ctx)
{
	return 0x0800;
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

	/* dispc stop done isr */
	if (reg_val & DISPC_INT_DONE_MASK) {
		ctx->is_stopped = true;
		evt_stop = true;
		wake_up_interruptible_all(&ctx->wait_queue);
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
					       msecs_to_jiffies(100));
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
					       msecs_to_jiffies(100));
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
		reg->dispc_ctrl &= ~BIT(4);

	dispc_wait_stop_done(ctx);
	pr_info("dispc stop\n");
}

static void dispc_run(struct dispc_context *ctx)
{
	dispc_reg_t *reg = (dispc_reg_t *)ctx->base;

	if (!reg)
		return;

	if (ctx->if_type == SPRD_DISPC_IF_DPI) {
		if (ctx->is_stopped) {
			/* set update mode with SW only */
			reg->dpi_ctrl |= BIT(4);

			/* update regs to shadow */
			reg->dpi_ctrl |= BIT(5);

			/* this delay is needed for registers update */
			udelay(10);

			/* start refresh */
			reg->dispc_ctrl |= BIT(4);

			/* set update mode with SW and VSync */
			reg->dpi_ctrl &= ~BIT(4);

			ctx->is_stopped = false;
			pr_info("dispc run\n");
		} else {
			/*dpi register update trigger*/
			reg->dpi_ctrl |= BIT(5);

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
		reg->dispc_ctrl |= BIT(4);

		ctx->is_stopped = false;
	}
}


static int dispc_init(struct dispc_context *ctx)
{
	dispc_reg_t *reg = (dispc_reg_t *)ctx->base;
	struct panel_info *panel = ctx->panel;
	uint32_t size;
	uint32_t width_strike;

	if (reg == NULL) {
		pr_err("dispc base address is null!");
		return -1;
	}

	if (panel == NULL) {
		pr_err("ctx->panel is null!");
		return -1;
	}

	/* enable dispc module */
	reg->dispc_ctrl |= BIT(0);

	/* set bg color */
	reg->bg_color = 0;

	/* enable dithering */
	reg->dispc_ctrl |= BIT(6);

	/* enable DISPC Power Control */
	reg->dispc_ctrl |= BIT(7);

	/* set dispc output size */
	if (ctx->if_type == SPRD_DISPC_IF_DPI) {
		width_strike = WIDTH_ALIGN(panel->width, 1);
		size = (width_strike & 0xffff) | ((panel->height) << 16);
	} else
		size = (panel->width & 0xffff) | ((panel->height) << 16);
	reg->dispc_size = size;

	/* set osd alpha to 0xff */
	reg->osd_alpha = 0xff;

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

static void dispc_img_ctrl(struct dispc_context *ctx, u32 layer_id,
			      u32 format, u32 compression)
{
	dispc_reg_t *reg = (dispc_reg_t *)ctx->base;
	int reg_val = 0;

	if (!reg)
		return;

	/*IMG or OSD layer enable */
	reg_val |= BIT(0);

	switch (format) {
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_RGBX8888:
		/* rb switch */
		reg_val |= BIT(15);
	case DRM_FORMAT_BGRA8888:
		if (compression)
			reg_val |= (SPRD_DATA_TYPE_AFBC_888 << 4);
		else
			/* ABGR */
			reg_val |= (SPRD_DATA_TYPE_RGB888 << 4);
		/* alpha mode select  - block alpha */
		reg_val |= (1 << 2);
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
		/* alpha mode select  - block alpha */
		reg_val |= (1 << 2);
		break;
	case DRM_FORMAT_BGR565:
		/* rb switch */
		reg_val |= BIT(15);
	case DRM_FORMAT_RGB565:
		/* rgb565 */
		reg_val |= (SPRD_DATA_TYPE_RGB565 << 4);
		/* B2B3B0B1 */
		reg_val |= (SPRD_IMG_DATA_ENDIAN_B2B3B0B1 << 8);
		/* alpha mode select  - block alpha */
		reg_val |= (1 << 2);
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
		reg_val |= SPRD_IMG_DATA_ENDIAN_B0B1B2B3 << 8;
		/*UV endian */
		reg_val |= SPRD_IMG_DATA_ENDIAN_B0B1B2B3 << 10;
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

	if (layer_id == SPRD_LAYER_IMG0 || layer_id == SPRD_LAYER_IMG1)
		reg->img_ctrl = reg_val;
	else
		reg->osd_ctrl = reg_val;
}

static void dispc_layer(struct dispc_context *ctx,
		    struct sprd_adf_hwlayer *hwlayer)
{
	dispc_reg_t *reg = (dispc_reg_t *)ctx->base;
	int size;
	int offset;
	int wd;

	if (!reg)
		return;

	if (ctx->if_type == SPRD_DISPC_IF_DPI)
		hwlayer->dst_w = WIDTH_ALIGN(hwlayer->dst_w, 1);
	size = (hwlayer->dst_w & 0xffff) | ((hwlayer->dst_h) << 16);
	offset = (hwlayer->dst_x & 0xffff) | ((hwlayer->dst_y) << 16);

	switch (hwlayer->hwlayer_id) {
	case SPRD_LAYER_OSD0:
	case SPRD_LAYER_OSD1:
		reg->osd_pos = offset;
		reg->osd_size = size;
		reg->osd_base_addr = hwlayer->iova_plane[0];
		wd = adf_format_bpp(hwlayer->format) / 8;
		reg->osd_pitch = hwlayer->pitch[0] / wd;
		break;

	case SPRD_LAYER_IMG0:
	case SPRD_LAYER_IMG1:
		reg->img_pos = offset;
		reg->img_size = size;
		reg->img_base_addr0 = hwlayer->iova_plane[0];
		reg->img_base_addr1 = hwlayer->iova_plane[1];
		reg->img_base_addr2 = hwlayer->iova_plane[2];
		reg->img_pitch = hwlayer->pitch[0];
		reg->y2r_ctrl = 1;
		reg->y2r_y_param = DISPC_BRIGHTNESS | DISPC_CONTRAST;
		reg->y2r_u_param = DISPC_OFFSET_U | DISPC_SATURATION_U;
		reg->y2r_v_param = DISPC_OFFSET_V | DISPC_SATURATION_V;
		break;
	}
	dispc_img_ctrl(ctx, hwlayer->hwlayer_id, hwlayer->format,
		       hwlayer->compression);

}

static void dispc_flip(struct dispc_context *ctx,
			struct sprd_restruct_config *config)
{
	int i;
	struct sprd_adf_hwlayer *hwlayer;

	for (i = 0; i < config->number_hwlayer; i++) {
		hwlayer = &config->hwlayers[i];
		dispc_layer(ctx, hwlayer);
	}
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
		/* halt function */
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

		/*enable dispc update done INT */
		int_mask |= DISPC_INT_UPDATE_DONE_MASK;
		/* enable dispc DONE  INT */
		int_mask |= DISPC_INT_DONE_MASK;
		/* enable dispc dpi vsync */
		int_mask |= DISPC_INT_DPI_VSYNC_MASK;
		/* enable underflow err INT */
		int_mask |= DISPC_INT_ERR_MASK;

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

static void dispc_bg_color(struct dispc_context *ctx, uint32_t color)
{
	dispc_reg_t *reg = (dispc_reg_t *)ctx->base;

	if (!reg)
		return;

	reg->bg_color = color;
	reg->img_ctrl &= ~BIT(0);
	reg->osd_ctrl &= ~BIT(0);
}

static struct dispc_core_ops dispc_r8p0_ops = {
	.version = dispc_get_version,
	.init = dispc_init,
	.uninit = dispc_uninit,
	.run = dispc_run,
	.stop = dispc_stop,
	.isr = dispc_isr,
	.flip = dispc_flip,
	.ifconfig = dispc_dpi_init,
	.bg_color = dispc_bg_color,
};

static struct ops_entry entry = {
	.ver = "dispc-r8p0",
	.ops = &dispc_r8p0_ops,
};

static int __init dispc_core_register(void)
{
	return dispc_core_ops_register(&entry);
}

subsys_initcall(dispc_core_register);

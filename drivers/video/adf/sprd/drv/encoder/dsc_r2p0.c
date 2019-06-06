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
#include <linux/module.h>

#include "sprd_dispc.h"

typedef struct {
	uint32_t dsc_ctrl;
	uint32_t dsc_pic_size;
	uint32_t dsc_grp_size;
	uint32_t dsc_grp_num;
	uint32_t dsc_h_timing;
	uint32_t dsc_v_timing;
	uint32_t dsc_cfg0;
	uint32_t dsc_cfg1;
	uint32_t dsc_cfg2;
	uint32_t dsc_cfg3;
	uint32_t dsc_cfg4;
	uint32_t dsc_cfg5;
	uint32_t dsc_cfg6;
	uint32_t dsc_cfg7;
	uint32_t dsc_cfg8;
	uint32_t dsc_cfg9;
	uint32_t dsc_cfg10;
	uint32_t dsc_cfg11;
	uint32_t dsc_cfg12;
	uint32_t dsc_cfg13;
	uint32_t dsc_cfg14;
	uint32_t dsc_cfg15;
	uint32_t dsc_cfg16;
	uint32_t dsc_sts0;
	uint32_t dsc_sts1;
	uint32_t dsc_version;
} dsc_reg_t;

static int32_t dsc_init(struct encoder_context *ctx)
{
	dsc_reg_t *reg = (dsc_reg_t *)ctx->base;
	struct panel_info *panel = ctx->panel;
	struct rgb_timing *timing = &panel->rgb_timing;

	reg->dsc_pic_size = (panel->height << 16) |
				(panel->width << 0);
	reg->dsc_h_timing = (timing->hsync << 0) |
			(timing->hbp << 8) |
			(timing->hfp << 20);
	reg->dsc_v_timing = (timing->vsync << 0) |
			(timing->vbp << 8) |
			(timing->vfp << 20);
	reg->dsc_grp_size = 0x000000f0;
	reg->dsc_grp_num = 0x04096000;

	reg->dsc_cfg0 = 0x306c81db;
	reg->dsc_cfg1 = 0x000ae4bd;
	reg->dsc_cfg2 = 0x0008000a;
	reg->dsc_cfg3 = 0x12181800;
	reg->dsc_cfg4 = 0x003316b6;
	reg->dsc_cfg5 = 0x382a1c0e;
	reg->dsc_cfg6 = 0x69625446;
	reg->dsc_cfg7 = 0x7b797770;
	reg->dsc_cfg8 = 0x00007e7d;
	reg->dsc_cfg9 = 0x01000102;
	reg->dsc_cfg10 = 0x09be0940;
	reg->dsc_cfg11 = 0x19fa19fc;
	reg->dsc_cfg12 = 0x1a3819f8;
	reg->dsc_cfg13 = 0x1ab61a78;
	reg->dsc_cfg14 = 0x2b342af6;
	reg->dsc_cfg15 = 0x3b742b74;
	reg->dsc_cfg16 = 0x00006bf4;

	reg->dsc_ctrl = 0x2000000b;

	return 0;
}

static void dsc_uninit(struct encoder_context *ctx)
{
}

static void dsc_enable(struct encoder_context *ctx)
{
	dsc_reg_t *reg = (dsc_reg_t *)ctx->base;

	reg->dsc_ctrl |= BIT(0);
}

static void dsc_disable(struct encoder_context *ctx)
{
	dsc_reg_t *reg = (dsc_reg_t *)ctx->base;

	reg->dsc_ctrl &= ~BIT(0);
}

static struct encoder_ops dsc_r2p0_ops = {
	.init = dsc_init,
	.uninit = dsc_uninit,
	.enable = dsc_enable,
	.disable = dsc_disable,
};

static struct ops_entry entry = {
	.ver = "dsc-r2p0",
	.ops = &dsc_r2p0_ops,
};

static int __init dispc_encoder_register(void)
{
	return dispc_enc_ops_register(&entry);
}

subsys_initcall(dispc_encoder_register);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("leon.he@spreadtrum.com");
MODULE_DESCRIPTION("SPRD DSC Encoder Register Operation");

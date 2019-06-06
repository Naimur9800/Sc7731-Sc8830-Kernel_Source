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
	uint32_t chip_id;
	uint32_t resolution;
	uint32_t if_ctrl;
	uint32_t reserved0;
	uint32_t process_ctrl;
	uint32_t reserved1;
	uint32_t dtw_ctrl;
	uint32_t reserved[10];
	uint32_t cfg_regs[54];
} bv3_reg_t;

static int32_t bv3_init(struct encoder_context *ctx)
{
	int i;
	bv3_reg_t *reg = (bv3_reg_t *)ctx->base;
	struct panel_info *panel = ctx->panel;
	uint32_t cfg_val[54] = {
		0x8c000e,
		0x480020f0, 0x08f8383a, 0x20, 0x20004000,
		0x20000020, 0x0a2c0a20, 0x08182800, 0x20000830,
		0x20, 0x0, 0x0, 0x0,
		0x0, 0x0, 0x0, 0x0,
		0x0, 0x0, 0x0, 0x0,
		0x0, 0x04010000, 0x03000001, 0x10,
		0x0, 0x0, 0x0, 0x0,
		0x0, 0x0, 0x0, 0x0,
		0x0, 0x0, 0x0, 0x35231200,
		0x7d6b5947, 0xc5b3a18f, 0xfffbe9d7, 0x15abf0ff,
		0x03c0, 0x06020000, 0x291d130c, 0x705b4837,
		0xdebea288, 0x47adb8ff, 0x0334, 0x32221200,
		0x72625242, 0xb2a29282, 0xf2e2d2c2, 0x000000ff,
		0x0300,
	};

	reg->chip_id = 0x9461;
	reg->resolution = (panel->height << 16) | panel->width;
	reg->if_ctrl = 0;
	reg->process_ctrl = 0x01;
	reg->dtw_ctrl = 0;

	for (i = 0; i < 54; i++)
		reg->cfg_regs[i] = cfg_val[i];

	return 0;
}

static void bv3_uninit(struct encoder_context *ctx)
{
}

static void bv3_enable(struct encoder_context *ctx)
{
}

static void bv3_disable(struct encoder_context *ctx)
{
}

static struct encoder_ops bv3_yyg_ops = {
	.init = bv3_init,
	.uninit = bv3_uninit,
	.enable = bv3_enable,
	.disable = bv3_disable,
};

static struct ops_entry entry = {
	.ver = "YYG,Half-RGB-IP",
	.ops = &bv3_yyg_ops,
};

static int __init dispc_encoder_register(void)
{
	return dispc_enc_ops_register(&entry);
}

subsys_initcall(dispc_encoder_register);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("leon.he@spreadtrum.com");
MODULE_DESCRIPTION("YYG BV3 Half-RGB-IP Hardware level config");

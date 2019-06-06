/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd/pike2/glb.h>
#include <linux/regmap.h>

#include "sprd_dispc.h"

static struct dispc_clk_context {
	struct clk *clk_src_96m;
	struct clk *clk_src_100m;
	struct clk *clk_src_128m;
	struct clk *clk_src_153m6;
	struct clk *clk_src_192m;
	struct clk *clk_src_256m;
	struct clk *clk_src_320m;
	struct clk *clk_src_384m;
	struct clk *clk_dispc_core;
	struct clk *clk_dispc_dpi;
} dispc_clk_ctx;

static struct dispc_glb_context {
	struct regmap *ap_ahb;
	struct clk *clk_dispc_ahb_eb;
} dispc_glb_ctx;

static uint32_t dispc_core_clk[] = {
	128000000,
	153600000,
	192000000,
	256000000,
	320000000,
	384000000
};

static uint32_t dpi_clk_src[] = {
	96000000,
	100000000,
	128000000,
	153600000,
	192000000
};

static struct clk *val_to_clk(struct dispc_clk_context *ctx, u32 val)
{
	switch (val) {
	case 96000000:
		return ctx->clk_src_96m;
	case 100000000:
		return ctx->clk_src_100m;
	case 128000000:
		return ctx->clk_src_128m;
	case 153600000:
		return ctx->clk_src_153m6;
	case 192000000:
		return ctx->clk_src_192m;
	case 256000000:
		return ctx->clk_src_256m;
	case 320000000:
		return ctx->clk_src_320m;
	case 384000000:
		return ctx->clk_src_384m;
	default:
		pr_err("invalid clock value %u\n", val);
		return NULL;
	}
}

static int dispc_clk_parse_dt(struct dispc_context *ctx,
				struct device_node *np)
{
	int status = 0;
	struct dispc_clk_context *clk_ctx = &dispc_clk_ctx;

	clk_ctx->clk_src_96m =
		of_clk_get_by_name(np, "clk_src_96m");
	clk_ctx->clk_src_100m =
		of_clk_get_by_name(np, "clk_src_100m");
	clk_ctx->clk_src_128m =
		of_clk_get_by_name(np, "clk_src_128m");
	clk_ctx->clk_src_153m6 =
		of_clk_get_by_name(np, "clk_src_153m6");
	clk_ctx->clk_src_192m =
		of_clk_get_by_name(np, "clk_src_192m");
	clk_ctx->clk_src_256m =
		of_clk_get_by_name(np, "clk_src_256m");
	clk_ctx->clk_src_320m =
		of_clk_get_by_name(np, "clk_src_320m");
	clk_ctx->clk_src_384m =
		of_clk_get_by_name(np, "clk_src_384m");
	clk_ctx->clk_dispc_core =
		of_clk_get_by_name(np, "clk_dispc_core");
	clk_ctx->clk_dispc_dpi =
		of_clk_get_by_name(np, "clk_dispc_dpi");

	if (IS_ERR(clk_ctx->clk_src_96m)) {
		pr_warn("read clk_src_96m failed\n");
		clk_ctx->clk_src_96m = NULL;
		status |= -1;
	}

	if (IS_ERR(clk_ctx->clk_src_100m)) {
		pr_warn("warning: clk_src_100m not found\n");
		clk_ctx->clk_src_100m = NULL;
		status |= -1;
	}

	if (IS_ERR(clk_ctx->clk_src_128m)) {
		pr_warn("read clk_src_128m failed\n");
		clk_ctx->clk_src_128m = NULL;
		status |= -1;
	}

	if (IS_ERR(clk_ctx->clk_src_153m6)) {
		pr_warn("read clk_src_153m6 failed\n");
		clk_ctx->clk_src_153m6 = NULL;
		status |= -1;
	}

	if (IS_ERR(clk_ctx->clk_src_192m)) {
		pr_warn("read clk_src_192m failed\n");
		clk_ctx->clk_src_192m = NULL;
		status |= -1;
	}

	if (IS_ERR(clk_ctx->clk_src_256m)) {
		pr_warn("read clk_src_256m failed\n");
		clk_ctx->clk_src_256m = NULL;
		status |= -1;
	}

	if (IS_ERR(clk_ctx->clk_src_320m)) {
		pr_warn("warning: clk_src_320m not found\n");
		clk_ctx->clk_src_320m = NULL;
		status |= -1;
	}

	if (IS_ERR(clk_ctx->clk_src_384m)) {
		pr_warn("read clk_src_384m failed\n");
		clk_ctx->clk_src_384m = NULL;
		status |= -1;
	}

	if (IS_ERR_OR_NULL(clk_ctx->clk_dispc_core)) {
		pr_err("read clk_dispc_core failed\n");
		clk_ctx->clk_dispc_core = NULL;
		status |= -1;
	}

	if (IS_ERR_OR_NULL(clk_ctx->clk_dispc_dpi)) {
		pr_err("read clk_dispc_dpi failed\n");
		clk_ctx->clk_dispc_dpi = NULL;
		status |= -1;
	}

	return status;
}

static u32 calc_dispc_core_clk(u32 pclk)
{
	int i;

	pclk *= 3;

	for (i = 0; i < ARRAY_SIZE(dispc_core_clk); i++) {
		if (pclk <= dispc_core_clk[i])
			return dispc_core_clk[i];
	}

	pr_err("calc DPU_CORE_CLK failed, use default\n");
	return 128000000;
}

static u32 calc_dpi_clk_src(u32 pclk)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dpi_clk_src); i++) {
		if ((dpi_clk_src[i] % pclk) == 0)
			return dpi_clk_src[i];
	}

	pr_err("calc DPI_CLK_SRC failed, use default\n");
	return 64000000;
}

static int dispc_clk_init(struct dispc_context *ctx)
{
	int ret;
	u32 dispc_core_val;
	u32 dpi_src_val;
	struct clk *clk_src;
	struct dispc_clk_context *clk_ctx = &dispc_clk_ctx;

	dispc_core_val = calc_dispc_core_clk(ctx->panel->pixel_clk);
	dpi_src_val = calc_dpi_clk_src(ctx->panel->pixel_clk);

	pr_info("DPU_CORE_CLK = %u, DPI_CLK_SRC = %u\n",
		dispc_core_val, dpi_src_val);
	pr_info("dpi clock is %u\n", ctx->panel->pixel_clk);

	clk_src = val_to_clk(clk_ctx, dispc_core_val);
	ret = clk_set_parent(clk_ctx->clk_dispc_core, clk_src);
	if (ret)
		pr_warn("set dpu core clk source failed\n");

	clk_src = val_to_clk(clk_ctx, dpi_src_val);
	ret = clk_set_parent(clk_ctx->clk_dispc_dpi, clk_src);
	if (ret)
		pr_warn("set dpi clk source failed\n");

	ret = clk_set_rate(clk_ctx->clk_dispc_dpi, ctx->panel->pixel_clk);
	if (ret)
		pr_err("dpu update dpi clk rate failed\n");

	return ret;
}

static int dispc_clk_enable(struct dispc_context *ctx)
{
	int ret = -1;
	struct dispc_clk_context *clk_ctx = &dispc_clk_ctx;

	ret = clk_prepare_enable(clk_ctx->clk_dispc_core);
	if (ret) {
		pr_err("enable clk_dispc_core error\n");
		goto ERR_CLK_DISPC_CORE;
	}
	ret = clk_prepare_enable(clk_ctx->clk_dispc_dpi);
	if (ret) {
		pr_err("enable clk_dispc_dpi error\n");
		goto ERR_CLK_DISPC_DPI;
	}

	return 0;

ERR_CLK_DISPC_DPI:
	clk_disable_unprepare(clk_ctx->clk_dispc_dpi);
ERR_CLK_DISPC_CORE:
	clk_disable_unprepare(clk_ctx->clk_dispc_core);

	return ret;
}

static int dispc_clk_disable(struct dispc_context *ctx)
{
	struct dispc_clk_context *clk_ctx = &dispc_clk_ctx;

	clk_disable_unprepare(clk_ctx->clk_dispc_dpi);
	clk_disable_unprepare(clk_ctx->clk_dispc_core);

	clk_set_parent(clk_ctx->clk_dispc_dpi, NULL);
	clk_set_parent(clk_ctx->clk_dispc_core, NULL);

	return 0;
}

static int dispc_glb_parse_dt(struct dispc_context *ctx,
				struct device_node *np)
{
	int status = 0;
	struct dispc_glb_context *glb_ctx = &dispc_glb_ctx;

	glb_ctx->ap_ahb = syscon_regmap_lookup_by_compatible(
					    "sprd,sys-ap-ahb");
	glb_ctx->clk_dispc_ahb_eb =
		of_clk_get_by_name(np, "clk_dispc_ahb_eb");

	if (IS_ERR_OR_NULL(glb_ctx->clk_dispc_ahb_eb)) {
		pr_err("read clk_dispc_ahb_eb failed\n");
		glb_ctx->clk_dispc_ahb_eb = NULL;
		status |= -1;
	}
	return status;
}

static void dispc_glb_enable(struct dispc_context *ctx)
{
	int ret = 0;
	struct dispc_glb_context *glb_ctx = &dispc_glb_ctx;

	ret = clk_prepare_enable(glb_ctx->clk_dispc_ahb_eb);
	if (ret) {
		pr_err("enable clk_dispc_ahb_eb failed!\n");
		goto ERR_CLK_DISPC_AHB_EB;
	}
	return;
ERR_CLK_DISPC_AHB_EB:
	clk_disable_unprepare(glb_ctx->clk_dispc_ahb_eb);
}

static void dispc_glb_disable(struct dispc_context *ctx)
{
	struct dispc_glb_context *glb_ctx = &dispc_glb_ctx;

	clk_disable_unprepare(glb_ctx->clk_dispc_ahb_eb);
}

static void dispc_reset(struct dispc_context *ctx)
{
	struct dispc_glb_context *glb_ctx = &dispc_glb_ctx;

	regmap_update_bits(glb_ctx->ap_ahb,
		    REG_AP_AHB_AHB_RST, BIT_AP_AHB_DISPC_SOFT_RST,
		    BIT_AP_AHB_DISPC_SOFT_RST);
	udelay(10);
	regmap_update_bits(glb_ctx->ap_ahb,
		    REG_AP_AHB_AHB_RST, BIT_AP_AHB_DISPC_SOFT_RST,
		    (unsigned int)(~BIT_AP_AHB_DISPC_SOFT_RST));
}

static void dispc_power_domain(struct dispc_context *ctx, int enable)
{
}

static struct dispc_clk_ops dispc_clk_ops = {
	.parse_dt = dispc_clk_parse_dt,
	.init = dispc_clk_init,
	.enable = dispc_clk_enable,
	.disable = dispc_clk_disable,
};

static struct dispc_glb_ops dispc_glb_ops = {
	.parse_dt = dispc_glb_parse_dt,
	.reset = dispc_reset,
	.enable = dispc_glb_enable,
	.disable = dispc_glb_disable,
	.power = dispc_power_domain,
};

static struct ops_entry clk_entry = {
	.ver = "pike2",
	.ops = &dispc_clk_ops,
};

static struct ops_entry glb_entry = {
	.ver = "pike2",
	.ops = &dispc_glb_ops,
};

static int __init dispc_glb_register(void)
{
	dispc_clk_ops_register(&clk_entry);
	dispc_glb_ops_register(&glb_entry);
	return 0;
}

subsys_initcall(dispc_glb_register);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("albert.zhang@spreadtrum.com");
MODULE_DESCRIPTION("sprd pike2 dispc global and clk regs config");

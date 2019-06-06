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
#include <linux/mfd/syscon/sprd/sharkl3/glb.h>
#include <linux/regmap.h>

#include "sprd_dispc.h"

static struct dispc_clk_context {
	struct clk *clk_src_128m;
	struct clk *clk_src_153m6;
	struct clk *clk_src_192m;
	struct clk *clk_src_256m;
	struct clk *clk_src_384m;
	struct clk *clk_dpu_core;
	struct clk *clk_dpu_dpi;
} dispc_clk_ctx;

static struct dispc_glb_context {
	struct clk *clk_aon_apb_disp_eb;
	struct regmap *aon_apb;
} dispc_glb_ctx;

static const u32 dpu_core_clk[] = {
	153600000,
	192000000,
	256000000,
	384000000
};

static const u32 dpi_clk_src[] = {
	128000000,
	153600000,
	192000000
};

static struct clk *val_to_clk(struct dispc_clk_context *ctx, u32 val)
{
	switch (val) {
	case 128000000:
		return ctx->clk_src_128m;
	case 153600000:
		return ctx->clk_src_153m6;
	case 192000000:
		return ctx->clk_src_192m;
	case 256000000:
		return ctx->clk_src_256m;
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
	struct dispc_clk_context *clk_ctx = &dispc_clk_ctx;

	clk_ctx->clk_src_128m =
		of_clk_get_by_name(np, "clk_src_128m");
	clk_ctx->clk_src_153m6 =
		of_clk_get_by_name(np, "clk_src_153m6");
	clk_ctx->clk_src_192m =
		of_clk_get_by_name(np, "clk_src_192m");
	clk_ctx->clk_src_256m =
		of_clk_get_by_name(np, "clk_src_256m");
	clk_ctx->clk_src_384m =
		of_clk_get_by_name(np, "clk_src_384m");
	clk_ctx->clk_dpu_core =
		of_clk_get_by_name(np, "clk_dpu_core");
	clk_ctx->clk_dpu_dpi =
		of_clk_get_by_name(np, "clk_dpu_dpi");

	if (IS_ERR(clk_ctx->clk_src_128m)) {
		pr_warn("read clk_src_128m failed\n");
		clk_ctx->clk_src_128m = NULL;
	}

	if (IS_ERR(clk_ctx->clk_src_153m6)) {
		pr_warn("read clk_src_153m6 failed\n");
		clk_ctx->clk_src_153m6 = NULL;
	}

	if (IS_ERR(clk_ctx->clk_src_192m)) {
		pr_warn("read clk_src_192m failed\n");
		clk_ctx->clk_src_192m = NULL;
	}

	if (IS_ERR(clk_ctx->clk_src_256m)) {
		pr_warn("read clk_src_256m failed\n");
		clk_ctx->clk_src_256m = NULL;
	}

	if (IS_ERR(clk_ctx->clk_src_384m)) {
		pr_warn("read clk_src_384m failed\n");
		clk_ctx->clk_src_384m = NULL;
	}

	if (IS_ERR(clk_ctx->clk_dpu_core)) {
		pr_warn("read clk_dpu_core failed\n");
		clk_ctx->clk_dpu_core = NULL;
	}

	if (IS_ERR(clk_ctx->clk_dpu_dpi)) {
		pr_warn("read clk_dpu_dpi failed\n");
		clk_ctx->clk_dpu_dpi = NULL;
	}

	return 0;
}

static u32 calc_dpu_core_clk(u32 pclk)
{
	int i;

	pclk *= 3;

	for (i = 0; i < ARRAY_SIZE(dpu_core_clk); i++) {
		if (pclk <= dpu_core_clk[i])
			return dpu_core_clk[i];
	}

	pr_err("calc DPU_CORE_CLK failed, use default\n");
	return 384000000;
}

static u32 calc_dpi_clk_src(u32 pclk)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dpi_clk_src); i++) {
		if ((dpi_clk_src[i] % pclk) == 0)
			return dpi_clk_src[i];
	}

	pr_err("calc DPI_CLK_SRC failed, use default\n");
	return 128000000;
}

static int dispc_clk_init(struct dispc_context *ctx)
{
	int ret;
	u32 dpu_core_val;
	u32 dpi_src_val;
	struct clk *clk_src;
	struct dispc_clk_context *clk_ctx = &dispc_clk_ctx;

	dpu_core_val = calc_dpu_core_clk(ctx->panel->pixel_clk);
	dpi_src_val = calc_dpi_clk_src(ctx->panel->pixel_clk);

	pr_info("DPU_CORE_CLK = %u, DPI_CLK_SRC = %u\n",
		dpu_core_val, dpi_src_val);
	pr_info("dpi clock is %u\n", ctx->panel->pixel_clk);

	clk_src = val_to_clk(clk_ctx, dpu_core_val);
	ret = clk_set_parent(clk_ctx->clk_dpu_core, clk_src);
	if (ret)
		pr_warn("set dpu core clk source failed\n");

	clk_src = val_to_clk(clk_ctx, dpi_src_val);
	ret = clk_set_parent(clk_ctx->clk_dpu_dpi, clk_src);
	if (ret)
		pr_warn("set dpi clk source failed\n");

	ret = clk_set_rate(clk_ctx->clk_dpu_dpi, ctx->panel->pixel_clk);
	if (ret)
		pr_err("dpu update dpi clk rate failed\n");

	return ret;
}

static int dispc_clk_enable(struct dispc_context *ctx)
{
	int ret;
	struct dispc_clk_context *clk_ctx = &dispc_clk_ctx;

	ret = clk_prepare_enable(clk_ctx->clk_dpu_core);
	if (ret) {
		pr_err("enable clk_dpu_core error\n");
		return ret;
	}

	ret = clk_prepare_enable(clk_ctx->clk_dpu_dpi);
	if (ret) {
		pr_err("enable clk_dpu_dpi error\n");
		clk_disable_unprepare(clk_ctx->clk_dpu_core);
		return ret;
	}

	return 0;
}

static int dispc_clk_disable(struct dispc_context *ctx)
{
	struct dispc_clk_context *clk_ctx = &dispc_clk_ctx;

	clk_disable_unprepare(clk_ctx->clk_dpu_dpi);
	clk_disable_unprepare(clk_ctx->clk_dpu_core);

	clk_set_parent(clk_ctx->clk_dpu_dpi, clk_ctx->clk_src_128m);
	clk_set_parent(clk_ctx->clk_dpu_core, clk_ctx->clk_src_153m6);

	return 0;
}

static int dispc_glb_parse_dt(struct dispc_context *ctx,
				struct device_node *np)
{
	struct dispc_glb_context *glb_ctx = &dispc_glb_ctx;

	glb_ctx->aon_apb = syscon_regmap_lookup_by_phandle(np,
					    "sprd,syscon-aon-apb");
	if (IS_ERR(glb_ctx->aon_apb))
		pr_warn("parse syscon-aon-apb failed\n");

	glb_ctx->clk_aon_apb_disp_eb =
		of_clk_get_by_name(np, "clk_aon_apb_disp_eb");
	if (IS_ERR(glb_ctx->clk_aon_apb_disp_eb)) {
		pr_warn("read clk_aon_apb_disp_eb failed\n");
		glb_ctx->clk_aon_apb_disp_eb = NULL;
	}

	return 0;
}

static void dispc_glb_enable(struct dispc_context *ctx)
{
	int ret;
	struct dispc_glb_context *glb_ctx = &dispc_glb_ctx;

	ret = clk_prepare_enable(glb_ctx->clk_aon_apb_disp_eb);
	if (ret) {
		pr_err("enable clk_aon_apb_disp_eb failed!\n");
		return;
	}
}

static void dispc_glb_disable(struct dispc_context *ctx)
{
	struct dispc_glb_context *glb_ctx = &dispc_glb_ctx;

	clk_disable_unprepare(glb_ctx->clk_aon_apb_disp_eb);
}

static void dispc_reset(struct dispc_context *ctx)
{
	struct dispc_glb_context *glb_ctx = &dispc_glb_ctx;

	regmap_update_bits(glb_ctx->aon_apb,
		    REG_AON_APB_APB_RST1, BIT_AON_APB_DISP_SOFT_RST,
		    BIT_AON_APB_DISP_SOFT_RST);
	udelay(10);
	regmap_update_bits(glb_ctx->aon_apb,
		    REG_AON_APB_APB_RST1, BIT_AON_APB_DISP_SOFT_RST,
		    (unsigned int)(~BIT_AON_APB_DISP_SOFT_RST));
}

static void dispc_power_domain(struct dispc_context *ctx, int enable)
{
	/* The dispc power domain code is in video/disp_pw_domain. */
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
	.ver = "sharkl3",
	.ops = &dispc_clk_ops,
};

static struct ops_entry glb_entry = {
	.ver = "sharkl3",
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
MODULE_AUTHOR("leon.he@spreadtrum.com");
MODULE_DESCRIPTION("sprd sharkl3 dispc global and clk regs config");

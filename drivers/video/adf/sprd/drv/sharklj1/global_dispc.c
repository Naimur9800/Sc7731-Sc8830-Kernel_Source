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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd/sharklj1/glb.h>
#include <linux/regmap.h>
#include <video/disp_pw_domain.h>

#include "sprd_dispc.h"

static struct dispc_clk_context {
	uint32_t clk_src[DISPC_CLK_ID_MAX];
	struct clk *clk_dispc_core_parent;
	struct clk *clk_dispc_dpi_parent;
	struct clk *clk_dispc_core;
	struct clk *clk_dispc_dpi;
} dispc_clk_ctx;

static struct dispc_glb_context {
	struct regmap *ap_ahb;
	struct clk *clk_dispc_ahb_eb;
} dispc_glb_ctx;

static int dispc_clk_parse_dt(struct dispc_context *ctx,
				struct device_node *np)
{
	int ret = 0;
	int status = 0;
	struct dispc_clk_context *clk_ctx = &dispc_clk_ctx;

	clk_ctx->clk_dispc_core_parent =
		of_clk_get_by_name(np, "clk_dispc_core_parent");
	clk_ctx->clk_dispc_dpi_parent =
		of_clk_get_by_name(np, "clk_dispc_dpi_parent");
	clk_ctx->clk_dispc_core =
		of_clk_get_by_name(np, "clk_dispc_core");
	clk_ctx->clk_dispc_dpi =
		of_clk_get_by_name(np, "clk_dispc_dpi");

	ret = of_property_read_u32_array(np,
			 "clock-src", clk_ctx->clk_src, 2);
	if (ret) {
		pr_err("read clock-src fail (%d)\n", ret);
		status |= -1;
	}

	ctx->dpi_clk_src = clk_ctx->clk_src[1];
	pr_info("the dpi clock source from dts is %d\n", ctx->dpi_clk_src);

	if (IS_ERR_OR_NULL(clk_ctx->clk_dispc_core_parent)) {
		pr_err("read clk_dispc_core_parent failed\n");
		clk_ctx->clk_dispc_core_parent = NULL;
		status |= -1;
	}

	if (IS_ERR_OR_NULL(clk_ctx->clk_dispc_dpi_parent)) {
		pr_err("read clk_dispc_dpi_parent failed\n");
		clk_ctx->clk_dispc_dpi_parent = NULL;
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


static int dispc_clk_init(struct dispc_context *ctx)
{
	int ret = 0;
	struct dispc_clk_context *clk_ctx = &dispc_clk_ctx;

	ret = clk_set_parent(clk_ctx->clk_dispc_core,
			clk_ctx->clk_dispc_core_parent);
	if (ret)
		pr_err("dispc set clk parent fail\n");

	ret = clk_set_parent(clk_ctx->clk_dispc_dpi,
			clk_ctx->clk_dispc_dpi_parent);
	if (ret)
		pr_err("dispc set dpi clk parent fail\n");

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


static int dispc_clk_update(struct dispc_context *ctx, int clk_id, int val)
{
	int ret = -1;
	struct dispc_clk_context *clk_ctx = &dispc_clk_ctx;

	switch (clk_id) {
	case DISPC_CLK_ID_CORE:
		pr_err("dispc core clk doesn't support update\n");
		break;

	case DISPC_CLK_ID_DPI:
		pr_info("dpi_clk = %d\n", val);
		ret = clk_set_rate(clk_ctx->clk_dispc_dpi, val);
		if (ret)
			pr_err("dispc update dbi clk rate fail\n");
		break;

	default:
		pr_err("clk id %d doesn't support\n", clk_id);
		break;
	}

	return ret;
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
	.update = dispc_clk_update,
};

static struct dispc_glb_ops dispc_glb_ops = {
	.parse_dt = dispc_glb_parse_dt,
	.reset = dispc_reset,
	.enable = dispc_glb_enable,
	.disable = dispc_glb_disable,
	.power = dispc_power_domain,
};

static struct ops_entry clk_entry = {
	.ver = "sharklj1",
	.ops = &dispc_clk_ops,
};

static struct ops_entry glb_entry = {
	.ver = "sharklj1",
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
MODULE_AUTHOR("junxiao.feng@spreadtrum.com");
MODULE_DESCRIPTION("sprd sharklj1 dispc global and clk regs config");

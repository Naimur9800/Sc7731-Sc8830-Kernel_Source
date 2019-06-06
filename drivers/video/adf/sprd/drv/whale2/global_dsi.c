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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd/whale2/whale2_glb.h>
#include <linux/regmap.h>
#include <video/disp_pw_domain.h>

#include "sprd_dsi.h"

static struct dsi_glb_context {
	struct regmap *disp_ahb;
	struct clk *clk_dphy0_ckg_eb;
	struct clk *clk_disp_ckg_ahb_eb;
	struct clk *clk_dsi0_ahb_eb;
	struct clk *clk_disp_apb_eb;
} dsi_glb_ctx[2];

static int dsi_glb_parse_dt(struct dsi_context *ctx,
				struct device_node *np)
{
	int status = 0;
	struct dsi_glb_context *glb_ctx = &dsi_glb_ctx[ctx->id];

	glb_ctx->disp_ahb = syscon_regmap_lookup_by_compatible(
					    "sprd,sys-disp-ahb");

	glb_ctx->clk_dphy0_ckg_eb =
		of_clk_get_by_name(np, "clk_dphy0_ckg_eb");
	glb_ctx->clk_disp_ckg_ahb_eb =
		of_clk_get_by_name(np, "clk_disp_ckg_ahb_eb");
	glb_ctx->clk_dsi0_ahb_eb =
		of_clk_get_by_name(np, "clk_dsi0_ahb_eb");
	glb_ctx->clk_disp_apb_eb =
		of_clk_get_by_name(np, "clk_disp_apb_eb");

	if (IS_ERR_OR_NULL(glb_ctx->clk_dphy0_ckg_eb)) {
		pr_err("read clk_dphy0_ckg_eb failed\n");
		glb_ctx->clk_dphy0_ckg_eb = NULL;
		status |= -1;
	}

	if (IS_ERR_OR_NULL(glb_ctx->clk_disp_ckg_ahb_eb)) {
		pr_err("read clk_disp_ckg_ahb_eb failed\n");
		glb_ctx->clk_disp_ckg_ahb_eb = NULL;
		status |= -1;
	}

	if (IS_ERR_OR_NULL(glb_ctx->clk_dsi0_ahb_eb)) {
		pr_err("read clk_dsi0_ahb_eb failed\n");
		glb_ctx->clk_dsi0_ahb_eb = NULL;
		status |= -1;
	}

	if (IS_ERR_OR_NULL(glb_ctx->clk_disp_apb_eb)) {
		pr_err("read clk_disp_apb_eb failed\n");
		glb_ctx->clk_disp_apb_eb = NULL;
		status |= -1;
	}

	return status;
}

static void dsi_glb_enable(struct dsi_context *ctx)
{
	int ret = 0;
	struct dsi_glb_context *glb_ctx = &dsi_glb_ctx[ctx->id];

	ret = clk_prepare_enable(glb_ctx->clk_disp_apb_eb);
	if (ret) {
		pr_err("enable clk_disp_apb_eb failed!\n");
		goto ERR_CLK_DISP_APB_EB;
	}
	ret = clk_prepare_enable(glb_ctx->clk_dsi0_ahb_eb);
	if (ret) {
		pr_err("enable clk_dsi0_ahb_eb failed!\n");
		goto ERR_CLK_DSI_AHB_EB;
	}
	ret = clk_prepare_enable(glb_ctx->clk_disp_ckg_ahb_eb);
	if (ret) {
		pr_err("enable clk_disp_ckg_ahb_eb failed!\n");
		goto ERR_CLK_DISP_CKG_AHB_EB;
	}
	ret = clk_prepare_enable(glb_ctx->clk_dphy0_ckg_eb);
	if (ret) {
		clk_disable_unprepare(glb_ctx->clk_dphy0_ckg_eb);
		pr_err("enable clk_dphy0_ckg_eb failed!\n");
		goto ERR_CLK_DPHY0_CKG_EB;
	}
	return;

ERR_CLK_DPHY0_CKG_EB:
	clk_disable_unprepare(glb_ctx->clk_dphy0_ckg_eb);
ERR_CLK_DISP_CKG_AHB_EB:
	clk_disable_unprepare(glb_ctx->clk_disp_ckg_ahb_eb);
ERR_CLK_DSI_AHB_EB:
	clk_disable_unprepare(glb_ctx->clk_dsi0_ahb_eb);
ERR_CLK_DISP_APB_EB:
	clk_disable_unprepare(glb_ctx->clk_disp_apb_eb);
}

static void dsi_glb_disable(struct dsi_context *ctx)
{
	struct dsi_glb_context *glb_ctx = &dsi_glb_ctx[ctx->id];

	clk_disable_unprepare(glb_ctx->clk_dphy0_ckg_eb);
	clk_disable_unprepare(glb_ctx->clk_disp_ckg_ahb_eb);
	clk_disable_unprepare(glb_ctx->clk_dsi0_ahb_eb);
	clk_disable_unprepare(glb_ctx->clk_disp_apb_eb);
}

static void dsi_reset(struct dsi_context *ctx)
{
	struct dsi_glb_context *glb_ctx = &dsi_glb_ctx[ctx->id];

	regmap_update_bits(glb_ctx->disp_ahb,
		    REG_DISP_AHB_AHB_RST, BIT_DISP_AHB_DSI0_SOFT_RST,
		    BIT_DISP_AHB_DSI0_SOFT_RST);
	udelay(10);
	regmap_update_bits(glb_ctx->disp_ahb,
		    REG_DISP_AHB_AHB_RST, BIT_DISP_AHB_DSI0_SOFT_RST,
		    (unsigned int)(~BIT_DISP_AHB_DSI0_SOFT_RST));
}

static void dsi_power_domain(struct dsi_context *ctx, int enable)
{
	if (enable)
		disp_pw_on(DISP_PW_DOMAIN_DSI);
	else
		disp_pw_off(DISP_PW_DOMAIN_DSI);
}

static struct dsi_glb_ops dsi_glb_ops = {
	.parse_dt = dsi_glb_parse_dt,
	.reset = dsi_reset,
	.enable = dsi_glb_enable,
	.disable = dsi_glb_disable,
	.power = dsi_power_domain,
};

static struct ops_entry entry = {
	.ver = "whale2",
	.ops = &dsi_glb_ops,
};

static int __init dsi_glb_register(void)
{
	return dsi_glb_ops_register(&entry);
}

subsys_initcall(dsi_glb_register);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("leon.he@spreadtrum.com");
MODULE_DESCRIPTION("sprd dsi global AHB regs low-level config");

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
#include <linux/mfd/syscon/sprd/isharkl2/isharkl2_glb.h>
#include <linux/regmap.h>
#include <video/mm_pw_domain.h>

#include "sprd_dsi.h"

static struct dsi_glb_context {
	struct regmap *aon_apb;
	struct regmap *ap_cam;
	struct regmap *lpc_ahb;
	struct regmap *disp_ahb;

	struct clk *clk_dphy0_ckg_eb;
	struct clk *clk_disp_ckg_ahb_eb;
	struct clk *clk_dsi0_ahb_eb;
	struct clk *clk_disp_apb_eb;
	struct clk *clk_disp_cam_eb;
} dsi_glb_ctx;

static struct clk *clk_check_dt(struct device_node *np, const char *text,
				int *status)
{
	struct clk *clk = NULL;

	clk = of_clk_get_by_name(np, text);
	if (IS_ERR_OR_NULL(clk)) {
		pr_err("read %s failed\n", text);
		*status |= -1;
	}

	return clk;
}

static int dsi_glb_parse_dt(struct dsi_context *ctx, struct device_node *np)
{
	int status = 0;
	struct dsi_glb_context *glb = &dsi_glb_ctx;

	pr_info("dsi parsing dt...\n");
	glb->aon_apb = syscon_regmap_lookup_by_compatible(
					  "sprd,iwhale2-aon-apb");
	glb->disp_ahb = syscon_regmap_lookup_by_compatible(
					   "sprd,iwhale2-dispc-ahb");
	glb->ap_cam = syscon_regmap_lookup_by_compatible(
					    "sprd,isharkl2-ap-cam-clk");
	glb->lpc_ahb = syscon_regmap_lookup_by_compatible(
					    "sprd,iwhale2-lpc-ahb");
	glb->clk_dphy0_ckg_eb =
		clk_check_dt(np, "clk_dphy0_ckg_eb", &status);
	glb->clk_disp_ckg_ahb_eb =
		clk_check_dt(np, "clk_disp_ckg_ahb_eb", &status);
	glb->clk_dsi0_ahb_eb =
		clk_check_dt(np, "clk_dsi0_ahb_eb", &status);
	glb->clk_disp_apb_eb =
		clk_check_dt(np, "clk_disp_apb_eb", &status);
	glb->clk_disp_cam_eb =
		clk_check_dt(np, "clk_disp_cam_eb", &status);
	return 0;
}

static void dsi_glb_enable(struct dsi_context *ctx)
{
	int ret;
	struct dsi_glb_context *glb_ctx = &dsi_glb_ctx;

	ret = clk_prepare_enable(glb_ctx->clk_disp_cam_eb);
	if (ret) {
		pr_err("enable clk_dispc_cam_eb failed!\n");
		goto ERR_CLK_DISPC_CAM_EB;
	}

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
		pr_err("enable clk_dphy0_ckg_eb failed!\n");
		goto ERR_CLK_DPHY_CKG_EB;
	}
	regmap_write(glb_ctx->disp_ahb, REG_DISP_AHB_DSI_CTRL,
				(ctx->dln_timer << 16)|(0x87 << 0));
	return;

ERR_CLK_DPHY_CKG_EB:
	clk_disable_unprepare(glb_ctx->clk_dphy0_ckg_eb);
ERR_CLK_DISP_CKG_AHB_EB:
	clk_disable_unprepare(glb_ctx->clk_disp_ckg_ahb_eb);
ERR_CLK_DSI_AHB_EB:
	clk_disable_unprepare(glb_ctx->clk_dsi0_ahb_eb);
ERR_CLK_DISP_APB_EB:
	clk_disable_unprepare(glb_ctx->clk_disp_apb_eb);
ERR_CLK_DISPC_CAM_EB:
	clk_disable_unprepare(glb_ctx->clk_disp_cam_eb);
}

static void dsi_glb_disable(struct dsi_context *ctx)
{
	struct dsi_glb_context *glb_ctx = &dsi_glb_ctx;

	clk_disable_unprepare(glb_ctx->clk_dphy0_ckg_eb);
	clk_disable_unprepare(glb_ctx->clk_disp_ckg_ahb_eb);
	clk_disable_unprepare(glb_ctx->clk_dsi0_ahb_eb);
	clk_disable_unprepare(glb_ctx->clk_disp_apb_eb);
	clk_disable_unprepare(glb_ctx->clk_disp_cam_eb);
}

static void dsi_reset(struct dsi_context *ctx)
{
	int mask, value;
	struct dsi_glb_context *glb_ctx = &dsi_glb_ctx;

	mask = BIT_DISP_AHB_DSI0_SOFT_RST;
	value = mask;

	regmap_update_bits(glb_ctx->disp_ahb,
			   REG_DISP_AHB_AHB_EB, mask, value);
	udelay(10);
	regmap_update_bits(glb_ctx->disp_ahb,
			   REG_DISP_AHB_AHB_EB, mask, ~value);
}


static void dsi_power_domain(struct dsi_context *ctx, int enable)
{
#ifdef CONFIG_SPRD_CAM_PW_DOMAIN_R3P0V2
	if (enable)
		sprd_cam_pw_on();
	else
		sprd_cam_pw_off();
#endif
}

static struct dsi_glb_ops dsi_glb_ops = {
	.parse_dt = dsi_glb_parse_dt,
	.reset = dsi_reset,
	.enable = dsi_glb_enable,
	.disable = dsi_glb_disable,
	.power = dsi_power_domain,
};

static struct ops_entry entry = {
	.ver = "isharkl2",
	.ops = &dsi_glb_ops,
};

static int __init dsi_glb_register(void)
{
	return dsi_glb_ops_register(&entry);
}

subsys_initcall(dsi_glb_register);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("infi.chen@spreadtrum.com");
MODULE_DESCRIPTION("sprd isharkl2 dsi global AHB regs low-level config");

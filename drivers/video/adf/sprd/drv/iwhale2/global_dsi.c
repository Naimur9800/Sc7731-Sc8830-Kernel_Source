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
#include <linux/mfd/syscon/sprd/iwhale2/iwhale2_glb.h>
#include <linux/regmap.h>
#include <video/mm_pw_domain.h>

#include "sprd_dsi.h"

static struct dsi_glb_context {
	struct regmap *lpc_ahb;
	struct regmap *aon_apb;
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
	unsigned int chipid = 0;
	struct dsi_glb_context *glb_ctx = &dsi_glb_ctx[ctx->id];
	struct sprd_dsi *dsi = container_of(ctx, struct sprd_dsi, ctx);

	glb_ctx->aon_apb = syscon_regmap_lookup_by_compatible(
					    "sprd,iwhale2-aon-apb");
	glb_ctx->disp_ahb = syscon_regmap_lookup_by_compatible(
					    "sprd,iwhale2-dispc-ahb");
	glb_ctx->lpc_ahb = syscon_regmap_lookup_by_compatible(
					    "sprd,iwhale2-lpc-ahb");

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

	/* WORKAROUND: A0 chip DSI BTA has a bug, avoid esd check */
	regmap_read(glb_ctx->aon_apb, REG_AON_APB_AON_VER_ID, &chipid);
	if (chipid == 0)
		dsi->panel->esd_check_en = 0;

	return status;
}

static void dsi_glb_enable(struct dsi_context *ctx)
{
	int ret = 0;
	struct dsi_glb_context *glb_ctx = &dsi_glb_ctx[ctx->id];

	regmap_hwlock_update_bits(glb_ctx->lpc_ahb,
			REG_CAM_AHB_DSI0_CNT_CTRL,
			BIT_CAM_AHB_DSI0_DLN_TIMER_VALUE(0x36),
			BIT_CAM_AHB_DSI0_DLN_TIMER_VALUE(0x36));
	regmap_hwlock_update_bits(glb_ctx->lpc_ahb,
			REG_CAM_AHB_DSI0_CNT_CTRL,
			BIT_CAM_AHB_DSI0_CLN_TIMER_VALUE(0x87),
			BIT_CAM_AHB_DSI0_CLN_TIMER_VALUE(0x87));

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
	ret = clk_prepare_enable(glb_ctx->clk_dphy0_ckg_eb);
	if (ret) {
		pr_err("enable clk_dphy0_ckg_eb failed!\n");
		goto ERR_CLK_DPHY_CKG_EB;
	}

	return;

ERR_CLK_DPHY_CKG_EB:
	clk_disable_unprepare(glb_ctx->clk_dphy0_ckg_eb);
ERR_CLK_DSI_AHB_EB:
	clk_disable_unprepare(glb_ctx->clk_dsi0_ahb_eb);
ERR_CLK_DISP_APB_EB:
	clk_disable_unprepare(glb_ctx->clk_disp_apb_eb);
}

static void dsi_glb_disable(struct dsi_context *ctx)
{
	struct dsi_glb_context *glb_ctx = &dsi_glb_ctx[ctx->id];

	clk_disable_unprepare(glb_ctx->clk_dphy0_ckg_eb);
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
#ifdef CONFIG_SPRD_CAM_PW_DOMAIN_R2P0
	if (enable)
		sprd_mm_pw_on(SPRD_PW_DOMAIN_DSI);
	else
		sprd_mm_pw_off(SPRD_PW_DOMAIN_DSI);
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
	.ver = "iwhale2",
	.ops = &dsi_glb_ops,
};

static int __init dsi_glb_register(void)
{
	return dsi_glb_ops_register(&entry);
}

subsys_initcall(dsi_glb_register);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("sam.liu@spreadtrum.com");
MODULE_DESCRIPTION("sprd iwhale2 dsi global AHB regs low-level config");

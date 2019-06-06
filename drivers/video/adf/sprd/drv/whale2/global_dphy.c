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

#include "sprd_dphy.h"

static struct dphy_glb_context {
	struct regmap *ana_apb;
} dphy_glb_ctx[2];

static int dphy_glb_parse_dt(struct dphy_context *ctx,
				struct device_node *np)
{
	struct dphy_glb_context *glb_ctx = &dphy_glb_ctx[ctx->id];

	glb_ctx->ana_apb = syscon_regmap_lookup_by_compatible(
					    "sprd,sys-ana-apb");

	return 0;
}

static void dphy_pw_on(struct dphy_context *ctx)
{
	struct dphy_glb_context *glb_ctx = &dphy_glb_ctx[ctx->id];

	regmap_update_bits(glb_ctx->ana_apb,
		    REG_ANA_APB_PWR_CTRL1,
		    BIT_ANA_APB_FORCE_DSI_PHY_SHUTDOWNZ_M |
		    BIT_ANA_APB_FORCE_DSI_PHY_SHUTDOWNZ_S,
		    BIT_ANA_APB_FORCE_DSI_PHY_SHUTDOWNZ_M |
		    BIT_ANA_APB_FORCE_DSI_PHY_SHUTDOWNZ_S
		    );
	regmap_update_bits(glb_ctx->ana_apb,
		    REG_ANA_APB_PWR_CTRL1,
		    BIT_ANA_APB_MIPI_DSI_PS_PD_S_M |
		    BIT_ANA_APB_MIPI_DSI_PS_PD_L_M |
		    BIT_ANA_APB_MIPI_DSI_PS_PD_S_S |
		    BIT_ANA_APB_MIPI_DSI_PS_PD_L_S,
		    (unsigned int)(~(BIT_ANA_APB_MIPI_DSI_PS_PD_S_M |
		    BIT_ANA_APB_MIPI_DSI_PS_PD_L_M |
		    BIT_ANA_APB_MIPI_DSI_PS_PD_S_S |
		    BIT_ANA_APB_MIPI_DSI_PS_PD_L_S
		    ))
		    );
}

static void dphy_pw_off(struct dphy_context *ctx)
{
	struct dphy_glb_context *glb_ctx = &dphy_glb_ctx[ctx->id];

	regmap_update_bits(glb_ctx->ana_apb,
		    REG_ANA_APB_PWR_CTRL1,
		    BIT_ANA_APB_FORCE_DSI_PHY_SHUTDOWNZ_M |
		    BIT_ANA_APB_FORCE_DSI_PHY_SHUTDOWNZ_S,
		    (unsigned int)(~(BIT_ANA_APB_FORCE_DSI_PHY_SHUTDOWNZ_M |
		    BIT_ANA_APB_FORCE_DSI_PHY_SHUTDOWNZ_S))
		    );
	regmap_update_bits(glb_ctx->ana_apb,
		    REG_ANA_APB_PWR_CTRL1,
		    BIT_ANA_APB_MIPI_DSI_PS_PD_S_M |
		    BIT_ANA_APB_MIPI_DSI_PS_PD_L_M |
		    BIT_ANA_APB_MIPI_DSI_PS_PD_S_S |
		    BIT_ANA_APB_MIPI_DSI_PS_PD_L_S,
		    BIT_ANA_APB_MIPI_DSI_PS_PD_S_M |
		    BIT_ANA_APB_MIPI_DSI_PS_PD_L_M |
		    BIT_ANA_APB_MIPI_DSI_PS_PD_S_S |
		    BIT_ANA_APB_MIPI_DSI_PS_PD_L_S
		    );
}

static void dphy_power_domain(struct dphy_context *ctx, int enable)
{
	if (enable)
		dphy_pw_on(ctx);
	else
		dphy_pw_off(ctx);
}

static struct dphy_glb_ops dphy_glb_ops = {
	.parse_dt = dphy_glb_parse_dt,
	.power = dphy_power_domain,
};

static struct ops_entry entry = {
	.ver = "whale2",
	.ops = &dphy_glb_ops,
};

static int __init dphy_glb_register(void)
{
	return dphy_glb_ops_register(&entry);
}

subsys_initcall(dphy_glb_register);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("leon.he@spreadtrum.com");
MODULE_DESCRIPTION("sprd dphy global AHB regs low-level config");

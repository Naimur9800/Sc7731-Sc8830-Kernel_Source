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
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd/sc9833/sc9833_glb.h>
#include <linux/regmap.h>

#include "sprd_dphy.h"

static struct dphy_glb_context {
	struct regmap *ap_ahb;
	struct regmap *aon_apb;
} dphy_glb_ctx;


static int dphy_glb_parse_dt(struct dphy_context *ctx,
				struct device_node *np)
{
	struct dphy_glb_context *glb_ctx = &dphy_glb_ctx;

	glb_ctx->ap_ahb = syscon_regmap_lookup_by_compatible(
					    "sprd,sys-ap-ahb");

	glb_ctx->aon_apb = syscon_regmap_lookup_by_compatible(
					    "sprd,sys-aon-apb");

	return 0;
}

static void dphy_glb_enable(struct dphy_context *ctx)
{
	struct dphy_glb_context *glb_ctx = &dphy_glb_ctx;

	regmap_update_bits(glb_ctx->ap_ahb, REG_AP_AHB_MISC_CKG_EN,
		BIT_AP_AHB_DPHY_REF_CKG_EN | BIT_AP_AHB_DPHY_CFG_CKG_EN,
		BIT_AP_AHB_DPHY_REF_CKG_EN | BIT_AP_AHB_DPHY_CFG_CKG_EN);
}

static void dphy_glb_disable(struct dphy_context *ctx)
{
	struct dphy_glb_context *glb_ctx = &dphy_glb_ctx;

	regmap_update_bits(glb_ctx->ap_ahb,
		REG_AP_AHB_MISC_CKG_EN,
		BIT_AP_AHB_DPHY_REF_CKG_EN |
		BIT_AP_AHB_DPHY_CFG_CKG_EN,
		(unsigned int)
		(~(BIT_AP_AHB_DPHY_REF_CKG_EN |
		BIT_AP_AHB_DPHY_CFG_CKG_EN)));
}

static void dphy_power_domain(struct dphy_context *ctx, int enable)
{
	struct dphy_glb_context *glb_ctx = &dphy_glb_ctx;

	if (enable) {
		regmap_update_bits(glb_ctx->aon_apb,
			REG_AON_APB_PWR_CTRL, BIT_AON_APB_MIPI_DSI_PS_PD_S,
			(unsigned int)(~BIT_AON_APB_MIPI_DSI_PS_PD_S));
		udelay(10);
		regmap_update_bits(glb_ctx->aon_apb,
			REG_AON_APB_PWR_CTRL, BIT_AON_APB_MIPI_DSI_PS_PD_L,
			(unsigned int)(~BIT_AON_APB_MIPI_DSI_PS_PD_L));
	} else {
		regmap_update_bits(glb_ctx->aon_apb,
			REG_AON_APB_PWR_CTRL, BIT_AON_APB_MIPI_DSI_PS_PD_L,
			BIT_AON_APB_MIPI_DSI_PS_PD_L);
		udelay(10);
		regmap_update_bits(glb_ctx->aon_apb,
			REG_AON_APB_PWR_CTRL, BIT_AON_APB_MIPI_DSI_PS_PD_S,
			BIT_AON_APB_MIPI_DSI_PS_PD_S);
	}
}

static struct dphy_glb_ops dphy_glb_ops = {
	.parse_dt = dphy_glb_parse_dt,
	.enable = dphy_glb_enable,
	.disable = dphy_glb_disable,
	.power = dphy_power_domain,
};

static struct ops_entry entry = {
	.ver = "sharkl2",
	.ops = &dphy_glb_ops,
};

static int __init dphy_glb_register(void)
{
	return dphy_glb_ops_register(&entry);
}

subsys_initcall(dphy_glb_register);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("billy.zhang@spreadtrum.com");
MODULE_DESCRIPTION("sprd sharkl2 dphy global AHB regs low-level config");

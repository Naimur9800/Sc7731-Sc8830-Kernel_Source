/*
 * Copyright (C) 2018 Spreadtrum Communications Inc.
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
#include <linux/mfd/syscon/sprd/sharkl5/glb.h>
#include <linux/regmap.h>

#include "sprd_dphy.h"

static struct dphy_glb_context {
	struct regmap *ap_ahb;
	struct regmap *pmu_apb;
} dphy_glb_ctx;


static int dphy_glb_parse_dt(struct dphy_context *ctx,
				struct device_node *np)
{
	struct dphy_glb_context *glb_ctx = &dphy_glb_ctx;

	glb_ctx->ap_ahb = syscon_regmap_lookup_by_compatible(
					    "sprd,sys-ap-ahb");

	glb_ctx->pmu_apb = syscon_regmap_lookup_by_compatible(
					    "sprd,sys-pmu-apb");

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
		regmap_update_bits(glb_ctx->pmu_apb,
			REG_PMU_APB_ANALOG_PHY_PD_CFG, BIT_PMU_APB_DSI_PD_REG,
			(unsigned int)(~BIT_PMU_APB_DSI_PD_REG));
	} else {
		regmap_update_bits(glb_ctx->pmu_apb,
			REG_PMU_APB_ANALOG_PHY_PD_CFG, BIT_PMU_APB_DSI_PD_REG,
			(BIT_PMU_APB_DSI_PD_REG));
	}

}

static struct dphy_glb_ops dphy_glb_ops = {
	.parse_dt = dphy_glb_parse_dt,
	.enable = dphy_glb_enable,
	.disable = dphy_glb_disable,
	.power = dphy_power_domain,
};

static struct ops_entry entry = {
	.ver = "sharkl5",
	.ops = &dphy_glb_ops,
};

static int __init dphy_glb_register(void)
{
	return dphy_glb_ops_register(&entry);
}

subsys_initcall(dphy_glb_register);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("kevin.tang@spreadtrum.com");
MODULE_DESCRIPTION("sprd sharkl5 dphy global AHB regs low-level config");

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
#include <linux/mfd/syscon/sprd/sharklj1/glb.h>
#include <linux/regmap.h>

#include "sprd_dphy.h"

static struct dphy_glb_context {
	struct regmap *aon_apb;
	struct regmap *anlg_phy_g2;
} dphy_glb_ctx;

static int dphy_glb_parse_dt(struct dphy_context *ctx,
				struct device_node *np)
{
	struct dphy_glb_context *glb_ctx = &dphy_glb_ctx;

	glb_ctx->aon_apb = syscon_regmap_lookup_by_compatible(
					    "sprd,sys-aon-apb");

	glb_ctx->anlg_phy_g2 = syscon_regmap_lookup_by_compatible(
						"sprd,anlg_phy_g2");

	return 0;
}

static void dphy_glb_enable(struct dphy_context *ctx)
{
	struct dphy_glb_context *glb_ctx = &dphy_glb_ctx;

	regmap_update_bits(glb_ctx->aon_apb, REG_AON_APB_APB_EB2,
		BIT_AON_APB_DPHY_REF_EB | BIT_AON_APB_DPHY_CFG_EB,
		BIT_AON_APB_DPHY_REF_EB | BIT_AON_APB_DPHY_CFG_EB);
	regmap_update_bits(glb_ctx->anlg_phy_g2,
		REG_ANLG_PHY_G2_ANALOG_MIPI_DSI_4LANE_REG_SEL_CFG_1,
		BIT(27) | BIT(28), (unsigned int)(BIT(27) | BIT(28)));
	regmap_update_bits(glb_ctx->anlg_phy_g2,
		REG_ANLG_PHY_G2_ANALOG_MIPI_DSI_4LANE_DSI_4L_CTRL,
		BIT(20) | BIT(21), (unsigned int)~(BIT(20) | BIT(21)));
	regmap_update_bits(glb_ctx->anlg_phy_g2,
		REG_ANLG_PHY_G2_ANALOG_MIPI_DSI_4LANE_DSI_4L_RES,
		BIT(16), (unsigned int)BIT(16));
	regmap_update_bits(glb_ctx->anlg_phy_g2,
		REG_ANLG_PHY_G2_ANALOG_MIPI_DSI_4LANE_REG_SEL_CFG_1,
		BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_MIPI_DSI_4LANE_DSI_RESERVED,
		(unsigned int)
		BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_MIPI_DSI_4LANE_DSI_RESERVED);
	regmap_update_bits(glb_ctx->anlg_phy_g2,
		REG_ANLG_PHY_G2_ANALOG_MIPI_DSI_4LANE_REG_SEL_CFG_1,
		BIT(25) | BIT(26), (unsigned int)(BIT(25) | BIT(26)));
	regmap_update_bits(glb_ctx->anlg_phy_g2,
		REG_ANLG_PHY_G2_ANALOG_MIPI_DSI_4LANE_DSI_4L_CTRL,
		BIT(18) | BIT(19), (unsigned int)~(BIT(18) | BIT(19)));
	mdelay(1);
	regmap_update_bits(glb_ctx->anlg_phy_g2,
		REG_ANLG_PHY_G2_ANALOG_MIPI_DSI_4LANE_DSI_4L_CTRL,
		BIT(18) | BIT(19), (unsigned int)(BIT(18) | BIT(19)));
}

static void dphy_glb_disable(struct dphy_context *ctx)
{
	struct dphy_glb_context *glb_ctx = &dphy_glb_ctx;

	regmap_update_bits(glb_ctx->anlg_phy_g2,
		REG_ANLG_PHY_G2_ANALOG_MIPI_DSI_4LANE_REG_SEL_CFG_1,
		BIT(27) | BIT(28), (unsigned int)(BIT(27) | BIT(28)));
	regmap_update_bits(glb_ctx->anlg_phy_g2,
		REG_ANLG_PHY_G2_ANALOG_MIPI_DSI_4LANE_DSI_4L_CTRL,
		BIT_ANLG_PHY_G2_ANALOG_MIPI_DSI_4LANE_DSI_PS_PD_S |
		BIT_ANLG_PHY_G2_ANALOG_MIPI_DSI_4LANE_DSI_PS_PD_L,
		(unsigned int)
		(BIT_ANLG_PHY_G2_ANALOG_MIPI_DSI_4LANE_DSI_PS_PD_S |
			BIT_ANLG_PHY_G2_ANALOG_MIPI_DSI_4LANE_DSI_PS_PD_L));
	regmap_update_bits(glb_ctx->anlg_phy_g2,
		REG_ANLG_PHY_G2_ANALOG_MIPI_DSI_4LANE_REG_SEL_CFG_1,
		BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_MIPI_DSI_4LANE_DSI_RESERVED,
		(unsigned int)
		BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_MIPI_DSI_4LANE_DSI_RESERVED);
	regmap_update_bits(glb_ctx->anlg_phy_g2,
		REG_ANLG_PHY_G2_ANALOG_MIPI_DSI_4LANE_DSI_4L_RES,
		BIT(16), (unsigned int)(~BIT(16)));
	regmap_update_bits(glb_ctx->aon_apb,
		REG_AON_APB_APB_EB2,
		BIT_AON_APB_DPHY_REF_EB |
		BIT_AON_APB_DPHY_CFG_EB,
		(unsigned int)
		(~(BIT_AON_APB_DPHY_REF_EB |
		BIT_AON_APB_DPHY_CFG_EB)));
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
	.ver = "sharklj1",
	.ops = &dphy_glb_ops,
};

static int __init dphy_glb_register(void)
{
	return dphy_glb_ops_register(&entry);
}

subsys_initcall(dphy_glb_register);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("junxiao.feng@spreadtrum.com");
MODULE_DESCRIPTION("sprd sharklj1 dsi global AHB regs low-level config");

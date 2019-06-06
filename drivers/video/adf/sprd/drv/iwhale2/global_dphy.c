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
#include <linux/mfd/syscon/sprd/iwhale2/iwhale2_glb.h>
#include <linux/regmap.h>

#include "sprd_dphy.h"

static struct dphy_glb_context {
	struct regmap *anlg_phy_g8;
} dphy_glb_ctx[2];

static int dphy_glb_parse_dt(struct dphy_context *ctx,
				struct device_node *np)
{
	struct dphy_glb_context *glb_ctx = &dphy_glb_ctx[ctx->id];

	glb_ctx->anlg_phy_g8 = syscon_regmap_lookup_by_compatible(
					    "sprd,anlg_phy_g8");

	return 0;
}

static void dphy_pw_on(struct dphy_context *ctx)
{
	struct dphy_glb_context *glb_ctx = &dphy_glb_ctx[ctx->id];
	unsigned int value = 0;

	regmap_update_bits(glb_ctx->anlg_phy_g8,
			REG_ANLG_PHY_G8_ANALOG_DSI_0_CTRL_REG0,
			BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_CTL_PGEN,
			(BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_CTL_PGEN));
	regmap_update_bits(glb_ctx->anlg_phy_g8,
			REG_ANLG_PHY_G8_ANALOG_DSI_0_CTRL_REG0,
			BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_CTL_FWEN_B,
			(BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_CTL_FWEN_B));
	regmap_update_bits(glb_ctx->anlg_phy_g8,
			REG_ANLG_PHY_G8_ANALOG_DSI_0_REG_SEL_CFG_0,
			BIT_ANLG_PHY_G8_ANALOG_DSI_0_O_STS_PLLOK,
			(unsigned int)
			~BIT_ANLG_PHY_G8_ANALOG_DSI_0_O_STS_PLLOK);
	regmap_update_bits(glb_ctx->anlg_phy_g8,
			REG_ANLG_PHY_G8_ANALOG_DSI_0_REG_SEL_CFG_0,
			BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_0_I_RST_SYS_N,
			(unsigned int)
			~BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_0_I_RST_SYS_N);
	regmap_update_bits(glb_ctx->anlg_phy_g8,
			REG_ANLG_PHY_G8_ANALOG_DSI_0_REG_SEL_CFG_0,
			BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_0_I_CTL_ENABLE_CLN |
			BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_0_I_CTL_ENABLE_DLN,
			(unsigned int) (~
			(BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_0_I_CTL_ENABLE_CLN |
			BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_0_I_CTL_ENABLE_DLN
			)));
	regmap_update_bits(glb_ctx->anlg_phy_g8,
			REG_ANLG_PHY_G8_ANALOG_DSI_0_CTRL_REG0,
			BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_RST_SYS_N |
			BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_RST_APB_N |
			BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_CTL_LATCHRESET_B |
			BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_CTL_TXLATCHEN_N,
			BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_RST_SYS_N |
			BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_RST_APB_N |
			BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_CTL_LATCHRESET_B |
			BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_CTL_TXLATCHEN_N);
	regmap_read(glb_ctx->anlg_phy_g8,
		REG_ANLG_PHY_G8_ANALOG_DSI_0_CTRL_REG0, &value);
	pr_err("dsi_enable 0xe4109000 =  0x%x\n", value);
}

static void dphy_pw_off(struct dphy_context *ctx)
{
	struct dphy_glb_context *glb_ctx = &dphy_glb_ctx[ctx->id];

	regmap_update_bits(glb_ctx->anlg_phy_g8,
			REG_ANLG_PHY_G8_ANALOG_DSI_0_REG_SEL_CFG_0,
			BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_0_I_CTL_ENABLE_CLN |
			BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_0_I_CTL_ENABLE_DLN,
			BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_0_I_CTL_ENABLE_CLN |
			BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_0_I_CTL_ENABLE_DLN);
	regmap_update_bits(glb_ctx->anlg_phy_g8,
			REG_ANLG_PHY_G8_ANALOG_DSI_0_CTRL_REG2,
			BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_CTL_ENABLE_CLN,
			(unsigned int)
			~BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_CTL_ENABLE_CLN);
	regmap_update_bits(glb_ctx->anlg_phy_g8,
			REG_ANLG_PHY_G8_ANALOG_DSI_0_CTRL_REG2,
			BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_CTL_ENABLE_DLN(0),
			BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_CTL_ENABLE_DLN(0));
	regmap_update_bits(glb_ctx->anlg_phy_g8,
			REG_ANLG_PHY_G8_ANALOG_DSI_0_CTRL_REG2,
			BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_0_I_RST_SYS_N,
			BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_0_I_RST_SYS_N);
	regmap_update_bits(glb_ctx->anlg_phy_g8,
			REG_ANLG_PHY_G8_ANALOG_DSI_0_REG_SEL_CFG_0,
			BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_0_I_RST_SYS_N,
			BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_0_I_RST_SYS_N);
	regmap_update_bits(glb_ctx->anlg_phy_g8,
			REG_ANLG_PHY_G8_ANALOG_DSI_0_CTRL_REG0,
			BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_RST_SYS_N,
			(unsigned int)
			~BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_RST_SYS_N);
	regmap_update_bits(glb_ctx->anlg_phy_g8,
			REG_ANLG_PHY_G8_ANALOG_DSI_0_CTRL_REG0,
			BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_RST_APB_N,
			(unsigned int)
			~BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_RST_APB_N);
	regmap_update_bits(glb_ctx->anlg_phy_g8,
			REG_ANLG_PHY_G8_ANALOG_DSI_0_REG_SEL_CFG_0,
			BIT_ANLG_PHY_G8_ANALOG_DSI_0_O_STS_PLLOK,
			BIT_ANLG_PHY_G8_ANALOG_DSI_0_O_STS_PLLOK);
	regmap_update_bits(glb_ctx->anlg_phy_g8,
			REG_ANLG_PHY_G8_ANALOG_DSI_0_CTRL_REG0,
			BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_CTL_PHYFORCEPLL,
			(unsigned int)
			(~BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_CTL_PHYFORCEPLL));
	regmap_update_bits(glb_ctx->anlg_phy_g8,
			REG_ANLG_PHY_G8_ANALOG_DSI_0_CTRL_REG0,
			BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_CTL_FWEN_B,
			(BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_CTL_FWEN_B));
	regmap_update_bits(glb_ctx->anlg_phy_g8,
			REG_ANLG_PHY_G8_ANALOG_DSI_0_CTRL_REG0,
			BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_CTL_PGEN,
			(BIT_ANLG_PHY_G8_ANALOG_DSI_0_I_CTL_PGEN));
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
	.ver = "iwhale2",
	.ops = &dphy_glb_ops,
};

static int __init dphy_glb_register(void)
{
	return dphy_glb_ops_register(&entry);
}

subsys_initcall(dphy_glb_register);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("sam.liu@spreadtrum.com");
MODULE_DESCRIPTION("sprd iwhale2 dphy global AHB regs low-level config");

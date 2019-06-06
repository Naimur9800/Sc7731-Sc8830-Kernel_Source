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
#include <linux/mfd/syscon/sprd/isharkl2/isharkl2_glb.h>
#include <linux/of_address.h>
#include <linux/regmap.h>

#include "sprd_dphy.h"

static bool dphy_hop_en;
static bool dphy_suspend;
static struct dphy_glb_context {
	unsigned long pll_base;
	struct regmap *pmu_apb;
	struct regmap *anlg_phy_g2;
	struct regmap *aon_apb;
} dphy_glb_ctx;

static inline u32 phy_readl(unsigned long base, u32 addr)
{
	return readl((void __iomem *)(base + addr));
}

static inline void phy_writel(unsigned long base, u32 val, u32 addr)
{
	writel(val, (void __iomem *)(base + addr));
}

static inline void writel_nbits(unsigned long base, u32 val,
				u8 nbits, u8 offset, u32 addr)
{
	u32 tmp;

	tmp = phy_readl(base, addr);
	tmp &= ~(((1 << nbits) - 1) << offset);
	tmp |= val << offset;
	phy_writel(base, tmp, addr);
}
static int dphy_hop_config(struct dphy_context *ctx, int delta, int period)
{
	u32 synthfdiv_hopmaxcode[2] = {0, 0};
	u32 synthfdiv_stepsize[2] = {0, 0};
	u8 synthfdiv_sscupdatecycle = 0;
	u8 synthfdiv_locwren = 0;
	u8 synthfdiv_hop_dir = 0;
	u8 synthfdiv_hop_req = 0;
	unsigned long base = dphy_glb_ctx.pll_base;

	mutex_lock(&ctx->hop_lock);
	if (dphy_suspend) {
		pr_emerg("dphy status is suspend\n");
		mutex_unlock(&ctx->hop_lock);
		return 0;
	}

	if (delta < 0) {
		delta = -delta;
		if (delta < (ctx->freq * 12 / 1012)) {
			pr_emerg("dphy hop down 1.03 percent\n");
			synthfdiv_hopmaxcode[0] = 0;
			synthfdiv_hopmaxcode[1] = 0x3f4a00;
			synthfdiv_stepsize[0] = 0x25;
			synthfdiv_stepsize[1] = 0x7e;
			synthfdiv_sscupdatecycle = 5;
		} else {
			pr_emerg("dphy hop down 2 percent\n");
			synthfdiv_hopmaxcode[0] = 2;
			synthfdiv_hopmaxcode[1] = 0x7ae147;
			synthfdiv_stepsize[0] = 0x32;
			synthfdiv_stepsize[1] = 0x3c9;
			synthfdiv_sscupdatecycle = 5;
		}
		writel_nbits(base, synthfdiv_hopmaxcode[0], 2, 30, 0x2a0);
		writel_nbits(base, synthfdiv_hopmaxcode[1], 25, 0, 0x2a4);
		writel_nbits(base, synthfdiv_stepsize[0], 6, 26, 0x2a4);
		writel_nbits(base, synthfdiv_stepsize[1], 14, 0, 0x2a8);
		writel_nbits(base, synthfdiv_sscupdatecycle, 8, 24, 0x248);

		synthfdiv_locwren = 1;
		synthfdiv_hop_dir = 2;
		synthfdiv_hop_req = 1;
		writel_nbits(base, synthfdiv_locwren, 1, 14, 0x2a8);
		writel_nbits(base, synthfdiv_hop_dir, 2, 26, 0x2a0);
		writel_nbits(base, synthfdiv_hop_req, 1, 29, 0x2a0);
		synthfdiv_hop_req = 0;
		writel_nbits(base, synthfdiv_hop_req, 1, 29, 0x2a0);
		dphy_hop_en = true;
	} else {
		if (dphy_hop_en) {
			pr_emerg("dphy hop up\n");
			synthfdiv_locwren = 1;
			synthfdiv_hop_dir = 1;
			synthfdiv_hop_req = 1;
			writel_nbits(base, synthfdiv_locwren, 1, 14, 0x2a8);
			writel_nbits(base, synthfdiv_hop_dir, 2, 26, 0x2a0);
			writel_nbits(base, synthfdiv_hop_req, 1, 29, 0x2a0);
			synthfdiv_hop_req = 0;
			writel_nbits(base, synthfdiv_hop_req, 1, 29, 0x2a0);
			dphy_hop_en = false;
		} else
			pr_emerg("now dphy has hop up\n");
	}
	mutex_unlock(&ctx->hop_lock);
	return 0;
}
static int chipid;
static int dphy_glb_parse_dt(struct dphy_context *ctx, struct device_node *np)
{
	struct dphy_glb_context *glb = &dphy_glb_ctx;
	struct resource r;
	struct sprd_dphy *dphy = container_of(ctx, struct sprd_dphy, ctx);

	pr_info("dsi parsing dt...\n");
	glb->pmu_apb = syscon_regmap_lookup_by_compatible(
					    "sprd,iwhale2-aon-pwu-apb");
	glb->anlg_phy_g2 = syscon_regmap_lookup_by_compatible(
					    "sprd,anlg_phy_g2");
	glb->aon_apb = syscon_regmap_lookup_by_compatible(
					    "sprd,iwhale2-aon-apb");
	if (of_address_to_resource(np, 2, &r)) {
		pr_err("can't get hoppin pll base address)\n");
		return 0;
	}

	glb->pll_base =	(unsigned long)
			ioremap_nocache(r.start, resource_size(&r));
	if (glb->pll_base == 0)
		pr_err("hoppin pll base ioremap failed!\n");
	else
		pr_info("hoppin pll base addr 0x%lx\n", glb->pll_base);

	if (dphy->pll)
		dphy->pll->hop_config = dphy_hop_config;
	regmap_read(glb->aon_apb, REG_AON_APB_AON_VER_ID, &chipid);

	return 0;
}

static void dphy_power_domain(struct dphy_context *ctx, int enable)
{
	int mask, value;
	struct dphy_glb_context *glb = &dphy_glb_ctx;

	mutex_lock(&ctx->hop_lock);
	mask = BIT_ANLG_PHY_G2_ANALOG_DSI_0_I_CTL_PGEN |
		BIT_ANLG_PHY_G2_ANALOG_DSI_0_I_CTL_FWEN_B |
		BIT_ANLG_PHY_G2_ANALOG_DSI_0_I_RST_SYS_N |
		BIT_ANLG_PHY_G2_ANALOG_DSI_0_I_RST_APB_N |
		BIT_ANLG_PHY_G2_ANALOG_DSI_0_I_CTL_LATCHRESET_B |
		BIT_ANLG_PHY_G2_ANALOG_DSI_0_I_CTL_TXLATCHEN_N;

	if (enable) {
		value = mask;
		regmap_update_bits(glb->anlg_phy_g2,
			REG_ANLG_PHY_G2_ANALOG_DSI_0_REG_SEL_CFG_0,
			BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_DSI_0_I_RST_APB_N |
			BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_DSI_0_I_RST_SYS_N,
			BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_DSI_0_I_RST_APB_N |
			BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_DSI_0_I_RST_SYS_N
		);
	} else
		value = ~mask;

	if ((enable == 0) &&
		((chipid == 0) || (chipid == 1)))
		value |= BIT_ANLG_PHY_G2_ANALOG_DSI_0_I_CTL_PGEN |
			BIT_ANLG_PHY_G2_ANALOG_DSI_0_I_CTL_FWEN_B;

	regmap_update_bits(glb->anlg_phy_g2,
		REG_ANLG_PHY_G2_ANALOG_DSI_0_CTRL_REG0, mask, value);

	mask = BIT_PMU_APB_DSPLL_FRC_ON;
	if (enable)
		value = mask;
	else
		value = ~mask;
	regmap_update_bits(glb->pmu_apb,
		REG_PMU_APB_DSPLL_REL_CFG, mask, value);
	dphy_suspend = !enable;
	mutex_unlock(&ctx->hop_lock);
}

static struct dphy_glb_ops dphy_glb_ops = {
	.parse_dt = dphy_glb_parse_dt,
	.power = dphy_power_domain,
};

static struct ops_entry entry = {
	.ver = "isharkl2",
	.ops = &dphy_glb_ops,
};

static int __init dphy_glb_register(void)
{
	return dphy_glb_ops_register(&entry);
}

subsys_initcall(dphy_glb_register);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("infi.chen@spreadtrum.com");
MODULE_DESCRIPTION("sprd isharkl2 dsi global AHB regs low-level config");

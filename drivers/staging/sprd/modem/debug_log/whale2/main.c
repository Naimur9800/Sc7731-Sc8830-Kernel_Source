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
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd/whale2/whale2_glb.h>

#include "core.h"
#include "phy.h"

#define REG_SERDES_APB_FUNC_EN			(0x00)
#define REG_SERDES_APB_CH_EN			(0x04)
#define REG_SERDES_APB_FSM_CUT_OFF_LEN		(0x10)

#define CHANNEL_MDAR			BIT(0)
#define CHANNEL_WTL			BIT(1)
#define CHANNEL_DBG_SYS			BIT(2)
#define CHANNEL_DBG_BUS			BIT(3)
#define CHANNEL_TRAINING		BIT(15)

static struct regmap *aon_apb;
static struct regmap *ana_apb;
static unsigned long serdes_apb;

static void dbg_log_init(struct dbg_log_device *dbg)
{
	regmap_update_bits(aon_apb,
		    REG_AON_APB_CGM_REG1, BIT_AON_APB_CGM_DPHY_REF_EN,
		    BIT_AON_APB_CGM_DPHY_REF_EN);
	regmap_update_bits(ana_apb,
		    REG_ANA_APB_MIPI_PHY_CTRL2,
		    BIT_ANA_APB_DSI_ENABLECLK |
		    BIT_ANA_APB_DSI_TXREQUESTHSCLK_S |
		    BIT_ANA_APB_DSI_ENABLE_0_S |
		    BIT_ANA_APB_DSI_ENABLE_1_S |
		    BIT_ANA_APB_DSI_ENABLE_2_S |
		    BIT_ANA_APB_DSI_ENABLE_3_S,
		    BIT_ANA_APB_DSI_ENABLECLK |
		    BIT_ANA_APB_DSI_TXREQUESTHSCLK_S |
		    BIT_ANA_APB_DSI_ENABLE_0_S |
		    BIT_ANA_APB_DSI_ENABLE_1_S |
		    BIT_ANA_APB_DSI_ENABLE_2_S |
		    BIT_ANA_APB_DSI_ENABLE_3_S);
	regmap_update_bits(ana_apb,
		    REG_ANA_APB_MIPI_PHY_CTRL2,
		    BIT_ANA_APB_DSI_SHUTDOWNZ |
		    BIT_ANA_APB_DSI_RSTZ |
		    BIT_ANA_APB_DSI_FORCEPLL_S,
		    BIT_ANA_APB_DSI_SHUTDOWNZ |
		    BIT_ANA_APB_DSI_RSTZ |
		    BIT_ANA_APB_DSI_FORCEPLL_S);
	regmap_update_bits(ana_apb,
		    REG_ANA_APB_DEBUG_LOG_CTRL,
		    BIT_ANA_APB_DSI_IF_SEL_S |
		    BIT_ANA_APB_LOG_MODE_EN,
		    BIT_ANA_APB_DSI_IF_SEL_S |
		    BIT_ANA_APB_LOG_MODE_EN);

	dsi_phy1_init(dbg->phy);
}

static void dbg_log_exit(struct dbg_log_device *dbg)
{
	dsi_phy1_exit(dbg->phy);
}

static void dbg_log_chn_sel(struct dbg_log_device *dbg)
{
	u32 ch = dbg->channel;

	reg_bits_clr(serdes_apb + REG_SERDES_APB_FUNC_EN, BIT(0));
	regmap_write(aon_apb, REG_AON_APB_SUB_SYS_DBG_SIG6, (1 << 2));
	reg_write(serdes_apb + REG_SERDES_APB_FSM_CUT_OFF_LEN, 0x20);

	switch (ch) {
	case CH_EN_TRAINING:
		reg_write(serdes_apb + REG_SERDES_APB_CH_EN, CHANNEL_TRAINING);
		break;
	case CH_EN_WTL:
		reg_write(serdes_apb + REG_SERDES_APB_CH_EN, CHANNEL_WTL);
		break;
	case CH_EN_DBG_SYS:
		reg_write(serdes_apb + REG_SERDES_APB_CH_EN, CHANNEL_DBG_SYS);
		break;
	case CH_EN_DBG_BUS:
		reg_write(serdes_apb + REG_SERDES_APB_CH_EN, CHANNEL_DBG_BUS);
		break;
	case CH_EN_MDAR:
		reg_write(serdes_apb + REG_SERDES_APB_CH_EN, CHANNEL_MDAR);
		break;
	default:
		return;
	}
	reg_bits_set(serdes_apb + REG_SERDES_APB_FUNC_EN, BIT(0));
	regmap_update_bits(aon_apb,
		    REG_AON_APB_SUB_SYS_DBG_SIG6,
		    BIT_AON_APB_AP_DBG_MOD_SEL(4),
		    BIT_AON_APB_AP_DBG_MOD_SEL(4));
}

static struct dbg_log_ops ops = {
	.init = dbg_log_init,
	.exit = dbg_log_exit,
	.select = dbg_log_chn_sel,
};

static struct phy_ctx phy = {
	.freq = 1000000,
	.lanes = 4,
};

static int dbg_log_probe(struct platform_device *pdev)
{
	struct resource r;
	void __iomem *addr;

	if (of_address_to_resource(pdev->dev.of_node, 0, &r) != 0) {
		pr_err("can't get serdes register base address\n");
		return -EINVAL;
	}

	addr = ioremap_nocache(r.start, resource_size(&r));
	if (IS_ERR(addr)) {
		pr_err("serdes_apb ioremap failed!\n");
		return PTR_ERR(addr);
	}
	serdes_apb = (unsigned long)addr;

	if (of_address_to_resource(pdev->dev.of_node, 1, &r) != 0) {
		pr_err("can't get dphy ctrl base address\n");
		return -EINVAL;
	}

	addr = ioremap_nocache(r.start, resource_size(&r));
	if (IS_ERR(addr)) {
		pr_err("dphy ctrl base ioremap failed!\n");
		return PTR_ERR(addr);
	}
	phy.base = (unsigned long)addr;

	aon_apb = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						"sprd,syscon-aon-apb");
	if (IS_ERR(aon_apb))
		return PTR_ERR(aon_apb);

	ana_apb = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						"sprd,syscon-ana-apb");
	if (IS_ERR(ana_apb))
		return PTR_ERR(ana_apb);

	dbg_log_device_register(&pdev->dev, &ops, &phy);

	return 0;
}

static const struct of_device_id dt_ids[] = {
	{ .compatible = "sprd,dbg-log-whale2", },
	{},
};

static struct platform_driver dbg_log_driver = {
	.probe = dbg_log_probe,
	.driver = {
		.name = "modem-dbg-log",
		.of_match_table = dt_ids,
	},
};

module_platform_driver(dbg_log_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("leon.he <leon.he@spreadtrum.com>");
MODULE_DESCRIPTION("Spreadtrum SoC Modem Debug Log Driver");

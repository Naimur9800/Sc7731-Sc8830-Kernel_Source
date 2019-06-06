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
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd/sharkle/glb.h>
#include <linux/mfd/syscon/sprd/sharkle/serdes_apb_glb.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "core.h"
#include "phy.h"

#define CUT_OFF_LEN			0x20

#define CHANNEL_MDAR			BIT(0)
#define CHANNEL_WTL			BIT(1)
#define CHANNEL_DBG_SYS			BIT(2)
#define CHANNEL_DBG_BUS			BIT(3)
#define CHANNEL_WCN			BIT(4)
#define CHANNEL_TRAINING		BIT(15)

static struct regmap *aon_apb;
static struct regmap *pmu_apb;
static struct regmap *mm_ahb;
static struct regmap *anlg_phy_g7;

static unsigned long serdes_apb;
static unsigned long anlg_phy_top;

static void dbg_log_init(struct dbg_log_device *dbg)
{
	regmap_update_bits(aon_apb, REG_AON_APB_APB_RST2,
			   BIT_AON_APB_SERDES_DPHY_APB_SOFT_RST,
			   (u32)~BIT_AON_APB_SERDES_DPHY_APB_SOFT_RST);

	regmap_update_bits(aon_apb, REG_AON_APB_APB_EB1,
			   BIT_AON_APB_SERDES_DPHY_EB,
			   BIT_AON_APB_SERDES_DPHY_EB);

	regmap_update_bits(aon_apb, REG_AON_APB_PWR_CTRL,
			   BIT_AON_APB_MIPI_CSI_2P2LANE_PS_PD_L |
			   BIT_AON_APB_MIPI_CSI_2P2LANE_PS_PD_S,
			   (u32)~(BIT_AON_APB_MIPI_CSI_2P2LANE_PS_PD_L |
				    BIT_AON_APB_MIPI_CSI_2P2LANE_PS_PD_S));

	regmap_update_bits(aon_apb, REG_AON_APB_APB_EB2,
			   BIT_AON_APB_SERDES_DPHY_CFG_EB |
			   BIT_AON_APB_SERDES_DPHY_REF_EB,
			   BIT_AON_APB_SERDES_DPHY_CFG_EB |
			   BIT_AON_APB_SERDES_DPHY_REF_EB);

	regmap_update_bits(aon_apb, REG_AON_APB_CSI_2P2L_DBG_PHY_CTRL,
			   BIT_AON_APB_CSI_2P2L_DBG_EN,
			   BIT_AON_APB_CSI_2P2L_DBG_EN);

	regmap_update_bits(aon_apb, REG_AON_APB_APB_EB2,
			   BIT_AON_APB_BUSMON_DMA_EB,
			   BIT_AON_APB_BUSMON_DMA_EB);

	regmap_update_bits(aon_apb, REG_AON_APB_CSI_2P2L_S_PHY_CTRL,
			   BIT_AON_APB_CSI_2P2L_TESTCLR_S_SEL,
			   BIT_AON_APB_CSI_2P2L_TESTCLR_S_SEL);

	regmap_update_bits(aon_apb, REG_AON_APB_CSI_2P2L_M_PHY_CTRL,
			   BIT_AON_APB_CSI_2P2L_TESTCLR_M_SEL,
			   BIT_AON_APB_CSI_2P2L_TESTCLR_M_SEL);

	regmap_update_bits(aon_apb, REG_AON_APB_CSI_2P2L_S_PHY_CTRL,
			   BIT_AON_APB_CSI_2P2L_TESTCLR_S,
			   BIT_AON_APB_CSI_2P2L_TESTCLR_S);

	regmap_update_bits(aon_apb, REG_AON_APB_CSI_2P2L_M_PHY_CTRL,
			   BIT_AON_APB_CSI_2P2L_TESTCLR_M,
			   BIT_AON_APB_CSI_2P2L_TESTCLR_M);

	regmap_update_bits(aon_apb, REG_AON_APB_CSI_2P2L_S_PHY_CTRL,
			   BIT_AON_APB_CSI_2P2L_TESTCLR_S,
			   (u32)~BIT_AON_APB_CSI_2P2L_TESTCLR_S);

	regmap_update_bits(aon_apb, REG_AON_APB_CSI_2P2L_M_PHY_CTRL,
			   BIT_AON_APB_CSI_2P2L_TESTCLR_M,
			   (u32)~BIT_AON_APB_CSI_2P2L_TESTCLR_M);

	regmap_update_bits(pmu_apb, REG_PMU_APB_PD_MM_TOP_CFG,
			   BIT_PMU_APB_PD_MM_TOP_FORCE_SHUTDOWN |
			   BIT_PMU_APB_PD_MM_TOP_AUTO_SHUTDOWN_EN,
			   (u32)~(BIT_PMU_APB_PD_MM_TOP_FORCE_SHUTDOWN |
				    BIT_PMU_APB_PD_MM_TOP_AUTO_SHUTDOWN_EN));

	regmap_update_bits(aon_apb, REG_AON_APB_APB_EB0,
			   BIT_AON_APB_MM_EB, BIT_AON_APB_MM_EB);
	/*wait mm power up*/
	mdelay(1);

	regmap_update_bits(mm_ahb, REG_MM_AHB_GEN_CKG_CFG,
			   BIT_MM_AHB_CPHY_CFG_CKG_EN,
			   BIT_MM_AHB_CPHY_CFG_CKG_EN);
	regmap_update_bits(aon_apb, REG_AON_APB_APB_EB2,
			   BIT_AON_APB_ANLG_EB |
			   BIT_AON_APB_ANLG_APB_EB,
			   BIT_AON_APB_ANLG_EB | BIT_AON_APB_ANLG_APB_EB);
	regmap_update_bits(anlg_phy_g7,
		REG_ANLG_PHY_G7_ANALOG_MIPI_CSI_2P2LANE_CSI_2P2L_CTRL_DB,
		BIT_ANLG_PHY_G7_ANALOG_MIPI_CSI_2P2LANE_DEBUG_EN_DB,
		BIT_ANLG_PHY_G7_ANALOG_MIPI_CSI_2P2LANE_DEBUG_EN_DB);
	regmap_update_bits(anlg_phy_g7,
		REG_ANLG_PHY_G7_ANALOG_MIPI_CSI_2P2LANE_REG_SEL_CFG_0,
		BIT_ANLG_PHY_G7_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_CSI_PS_PD_L |
		BIT_ANLG_PHY_G7_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_CSI_PS_PD_S,
		BIT_ANLG_PHY_G7_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_CSI_PS_PD_L |
		BIT_ANLG_PHY_G7_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_CSI_PS_PD_S);
	mdelay(1);
	regmap_update_bits(anlg_phy_g7,
	REG_ANLG_PHY_G7_ANALOG_MIPI_CSI_2P2LANE_REG_SEL_CFG_0,
	BIT_ANLG_PHY_G7_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_CSI_PS_PD_L |
	BIT_ANLG_PHY_G7_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_CSI_PS_PD_S,
	(u32)~(BIT_ANLG_PHY_G7_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_CSI_PS_PD_L |
	BIT_ANLG_PHY_G7_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_CSI_PS_PD_S));
	regmap_update_bits(anlg_phy_g7,
	REG_ANLG_PHY_G7_ANALOG_MIPI_CSI_2P2LANE_REG_SEL_CFG_0,
	BIT_ANLG_PHY_G7_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_DSI_SHUTDOWNZ_DB |
	BIT_ANLG_PHY_G7_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_DSI_IF_SEL_DB |
	BIT_ANLG_PHY_G7_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_DEBUG_EN_DB |
	BIT_ANLG_PHY_G7_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_DSI_TRIMBG_DB,
	BIT_ANLG_PHY_G7_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_DSI_SHUTDOWNZ_DB |
	BIT_ANLG_PHY_G7_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_DSI_IF_SEL_DB |
	BIT_ANLG_PHY_G7_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_DEBUG_EN_DB |
	BIT_ANLG_PHY_G7_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_DSI_TRIMBG_DB);

	reg_write(anlg_phy_top + REG_ANLG_PHY_TOP_ANALOG_TOP_REG_CTRL_2,
		  BIT_ANLG_PHY_TOP_CSI_2P2LANE_ISO_SW_EN);
	dbg_phy_init(dbg->phy);
}

static void dbg_log_exit(struct dbg_log_device *dbg)
{
	dbg_phy_exit(dbg->phy);

	regmap_update_bits(aon_apb, REG_AON_APB_APB_EB2,
			   BIT_AON_APB_BUSMON_DMA_EB,
			   (u32)~BIT_AON_APB_BUSMON_DMA_EB);

	regmap_update_bits(aon_apb, REG_AON_APB_PWR_CTRL,
			   BIT_AON_APB_MIPI_CSI_2P2LANE_PS_PD_L |
			   BIT_AON_APB_MIPI_CSI_2P2LANE_PS_PD_S,
			   BIT_AON_APB_MIPI_CSI_2P2LANE_PS_PD_L |
			   BIT_AON_APB_MIPI_CSI_2P2LANE_PS_PD_S);
}

static void dbg_log_chn_sel(struct dbg_log_device *dbg)
{
	u32 ch = dbg->channel;

	reg_bits_clr(serdes_apb + REG_SERDES_APB_FUNC_EN,
		     BIT_SERDES_APB_FUNC_EN_FUNC_EN);
	reg_bits_set(serdes_apb + REG_SERDES_APB_FUNC_EN,
		     BIT_SERDES_APB_FUNC_EN_FUNC_EN);
	reg_write(serdes_apb + REG_SERDES_APB_FSM_CUT_OFF_LEN, CUT_OFF_LEN);
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
	reg_write(serdes_apb + REG_SERDES_APB_FUNNEL_EN,
			BIT_SERDES_APB_FUNNEL_EN);
}

static struct dbg_log_ops ops = {
	.init = dbg_log_init,
	.exit = dbg_log_exit,
	.select = dbg_log_chn_sel,
};

static struct phy_ctx phy = {
	.freq = 1500000,
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
	addr = devm_ioremap_nocache(&pdev->dev, r.start, resource_size(&r));
	if (IS_ERR(addr)) {
		pr_err("serdes_apb ioremap failed!\n");
		return PTR_ERR(addr);
	}
	serdes_apb = (unsigned long)addr;
	if (of_address_to_resource(pdev->dev.of_node, 1, &r) != 0) {
		pr_err("can't get anlg_phy_top base address\n");
		return -EINVAL;
	}
	addr = devm_ioremap_nocache(&pdev->dev, r.start, resource_size(&r));
	if (IS_ERR(addr)) {
		pr_err("anlg_phy_top ioremap failed!\n");
		return PTR_ERR(addr);
	}
	anlg_phy_top = (unsigned long)addr;
	if (of_address_to_resource(pdev->dev.of_node, 2, &r) != 0) {
		pr_err("can't get anlg_phy_g7 base address\n");
		return -EINVAL;
	}
	addr = devm_ioremap_nocache(&pdev->dev, r.start, resource_size(&r));
	if (IS_ERR(addr)) {
		pr_err("anlg_phy_g7 ioremap failed!\n");
		return PTR_ERR(addr);
	}
	phy.base = (unsigned long)addr;
	aon_apb = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						  "sprd,syscon-aon-apb");
	pmu_apb = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						  "sprd,syscon-pmu-apb");
	mm_ahb = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						 "sprd,syscon-mm-ahb");
	anlg_phy_g7 = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						      "sprd,syscon-anlg-phy-g7");
	if (IS_ERR(aon_apb))
		return PTR_ERR(aon_apb);
	dbg_log_device_register(&pdev->dev, &ops, &phy);

	return 0;
}

static const struct of_device_id dt_ids[] = {
	{.compatible = "sprd,dbg-log-sharkle",},
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
MODULE_AUTHOR("Huigui Liu <huigui.liu@spreadtrum.com>");
MODULE_DESCRIPTION("Spreadtrum SoC Modem Debug Log Driver");

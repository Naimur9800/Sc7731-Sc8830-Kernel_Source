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

#include "phy.h"
#ifdef pr_fmt
#undef pr_fmt
#define pr_fmt(fmt) "[dsiphy] "fmt
#endif

static int dbg_phy_testclr(struct phy_ctx *phy, int h)
{
	regmap_u_bits(phy->dsi_apb,
		      REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_MIPI_PHY_BIST_TEST,
		      BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_CSI_2P2L_TESTCLR_M,
		      h);
	regmap_u_bits(phy->dsi_apb,
		      REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_MIPI_PHY_BIST_TEST,
		      BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_CSI_2P2L_TESTCLR_S,
		      h);
	regmap_u_bits(phy->dsi_apb,
		      REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_2P2L_TEST_DB,
		      BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_DSI_TESTCLR_DB,
		      h);
	return 0;
}

static int dbg_phy_shutdownz(struct phy_ctx *phy, int h)
{
	regmap_u_bits(phy->dsi_apb,
		      REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_2P2L_CTRL_DB,
		      BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_DSI_SHUTDOWNZ_DB,
		      h);
	regmap_u_bits(phy->dsi_apb,
		      REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_2P2L_CTRL_M,
		      BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_SHUTDOWNZ_M,
		      h);
	regmap_u_bits(phy->dsi_apb,
		      REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_2P2L_CTRL_S,
		      BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_SHUTDOWNZ_S,
		      h);
	return 0;
}

static int dbg_phy_resetz(struct phy_ctx *phy, int h)
{
	regmap_u_bits(phy->dsi_apb,
		      REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_2P2L_CTRL_DB,
		      BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_DSI_RSTZ_DB, h);
	regmap_u_bits(phy->dsi_apb,
		      REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_2P2L_CTRL_M,
		      BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_RSTZ_M, h);
	regmap_u_bits(phy->dsi_apb,
		      REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_2P2L_CTRL_S,
		      BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_RSTZ_S, h);
	return 0;
}

static int dbg_phy_enable_db(struct phy_ctx *phy, int h)
{
	regmap_u_bits(phy->dsi_apb,
		      REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_2P2L_CTRL_DB,
		      (BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_DSI_ENABLE_3_DB |
		       BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_DSI_ENABLE_2_DB |
		       BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_DSI_ENABLE_1_DB |
		       BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_DSI_ENABLE_0_DB),
		      h);
	return 0;
}

static int dbg_phy_enableclk(struct phy_ctx *phy, int h)
{
	regmap_u_bits(phy->dsi_apb,
		      REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_2P2L_CTRL_DB,
		      BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_DSI_ENABLECLK_DB,
		      h);
	return 0;
}

static int dbg_phy_clk_source(struct phy_ctx *phy, int h)
{
	if (h) {
		regmap_u_bits(phy->pll_apb,
			      REG_ANLG_PHY_G2_ANALOG_PLL_TOP_PLL_TO_CSI_CLK_CRTL,
			      BIT_ANLG_PHY_G2_ANALOG_PLL_TOP_CSI_TXBITCLK_SEL
			      (0x3), 0);
		regmap_u_bits(phy->pll_apb,
			      REG_ANLG_PHY_G2_ANALOG_PLL_TOP_PLL_TO_CSI_CLK_CRTL,
			      BIT_ANLG_PHY_G2_ANALOG_PLL_TOP_CSI_TXBITCLK_SEL
			      (phy->clk_sel), 1);
	}
	regmap_u_bits(phy->pll_apb,
		      REG_ANLG_PHY_G2_ANALOG_PLL_TOP_REG_SEL_CFG_0,
		      BIT(phy->div1_map[phy->clk_sel]), h);
	regmap_u_bits(phy->pll_apb,
		      REG_ANLG_PHY_G2_ANALOG_PLL_TOP_PLL_TO_CSI_CLK_CRTL,
		      BIT_ANLG_PHY_G2_ANALOG_PLL_TOP_PLL_CK2CSI_EN, h);
	return 0;
}

int dbg_phy_init(struct phy_ctx *phy)
{
	dbg_phy_clk_source(phy, 1);
	dbg_phy_testclr(phy, 0);
	regmap_u_bits(phy->dsi_apb,
		      REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_MIPI_PHY_BIST_TEST,
		      (BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_CSI_2P2L_TESTCLR_S_SEL
		       | BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_CSI_2P2L_TESTCLR_M_SEL),
		      1);
	dbg_phy_shutdownz(phy, 1);
	regmap_u_bits(phy->dsi_apb,
		      REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_REG_SEL_CFG_0,
		      (BIT_ANLG_PHY_G1_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_CSI_SHUTDOWNZ_M
		       | BIT_ANLG_PHY_G1_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_CSI_SHUTDOWNZ_S
		       | BIT_ANLG_PHY_G1_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_DSI_SHUTDOWNZ_DB),
		      1);
	dbg_phy_resetz(phy, 1);
	regmap_u_bits(phy->dsi_apb,
		      REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_REG_SEL_CFG_0,
		      (BIT_ANLG_PHY_G1_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_CSI_RSTZ_M
		       | BIT_ANLG_PHY_G1_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_CSI_RSTZ_S),
		      1);
	dbg_phy_enableclk(phy, 0);
	dbg_phy_enable_db(phy, 0);
	return 0;
}

int dbg_phy_exit(struct phy_ctx *phy)
{
	dbg_phy_clk_source(phy, 0);
	return 0;
}

int dbg_phy_enable(struct phy_ctx *phy, int enable, void (*ext_ctrl) (void *),
		   void *ext_para)
{
	static int is_enabled;

	if (is_enabled == enable)
		return 0;
	is_enabled = enable;

	if (enable) {
		dbg_phy_shutdownz(phy, 0);
		dbg_phy_resetz(phy, 0);
		mdelay(1);
		dbg_phy_testclr(phy, 1);
		mdelay(5);
		ext_ctrl(ext_para);
		dbg_phy_testclr(phy, 0);
		mdelay(5);
		dbg_phy_shutdownz(phy, 1);
		dbg_phy_resetz(phy, 1);
		mdelay(5);
		dbg_phy_enableclk(phy, 1);
		dbg_phy_enable_db(phy, 1);
	} else {
		dbg_phy_enable_db(phy, 0);
		dbg_phy_enableclk(phy, 0);
	}
	return 0;
}

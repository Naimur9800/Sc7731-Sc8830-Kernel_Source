/*
 * Copyright (C) 2018 Spreadtrum Communications Inc.
 *
 * This file is dual-licensed: you can use it either under the terms
 * of the GPL or the X11 license, at your option. Note that this dual
 * licensing only applies to this file, and not this project as a
 * whole.
 *
 * updated at 2018-06-28 19:05:47
 *
 */


#ifndef ANLG_TOP_1_H
#define ANLG_TOP_1_H



#define REG_ANLG_TOP_1_ANALOG_MPLL_CLK_JITTER_CTRL  (0x0000)
#define REG_ANLG_TOP_1_ANALOG_PHY_POWER_DOWN_CTRL   (0x0004)

/* REG_ANLG_TOP_1_ANALOG_MPLL_CLK_JITTER_CTRL */

#define BIT_ANLG_TOP_1_MPLL1_CLK_JITTER_MON_SEL(x)     (((x) & 0x3) << 4)
#define BIT_ANLG_TOP_1_MPLL1_CLK_JITTER_MON_EN         BIT(3)
#define BIT_ANLG_TOP_1_MPLL0_CLK_JITTER_MON_SEL(x)     (((x) & 0x3) << 1)
#define BIT_ANLG_TOP_1_MPLL0_CLK_JITTER_MON_EN         BIT(0)

/* REG_ANLG_TOP_1_ANALOG_PHY_POWER_DOWN_CTRL */

#define BIT_ANLG_TOP_1_R2G_ANALOG_USB30_ISO_SW_EN      BIT(5)
#define BIT_ANLG_TOP_1_R2G_ANALOG_USB30_PS_PD_L        BIT(4)
#define BIT_ANLG_TOP_1_R2G_ANALOG_USB30_PS_PD_S        BIT(3)
#define BIT_ANLG_TOP_1_R2G_ANALOG_PCIE_GEN2_ISO_SW_EN  BIT(2)
#define BIT_ANLG_TOP_1_R2G_ANALOG_PCIE_GEN2_PS_PD_L    BIT(1)
#define BIT_ANLG_TOP_1_R2G_ANALOG_PCIE_GEN2_PS_PD_S    BIT(0)


#endif /* ANLG_TOP_1_H */



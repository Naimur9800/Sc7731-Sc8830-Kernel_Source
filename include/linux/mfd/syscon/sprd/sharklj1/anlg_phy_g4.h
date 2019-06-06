/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This file is dual-licensed: you can use it either under the terms
 * of the GPL or the X11 license, at your option. Note that this dual
 * licensing only applies to this file, and not this project as a
 * whole.
 *
 */


#ifndef _ANLG_PHY_G4_REG_H
#define _ANLG_PHY_G4_REG_H



#define REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_CTRL0          0x0000
#define REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_CTRL1          0x0004
#define REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_CTRL2          0x0008
#define REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_BIST_CTRL      0x000C
#define REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_CTRL0            0x0010
#define REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_CTRL1            0x0014
#define REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_CTRL2            0x0018
#define REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_BIST_CTRL        0x001C
#define REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_THM_CTRL         0x0020
#define REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_THM1_CTRL_0           0x0024
#define REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_THM1_CTRL_1           0x0028
#define REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_REG_SEL_CFG_0         0x002C

/*---------------------------------------------------------------------------
// Register Name   : REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_CTRL0
// Register Offset : 0x0000
// Description     :
---------------------------------------------------------------------------*/

#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_IN_ISO_EN                             BIT(18)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_CLKOUT_SEL(x)                         (((x) & 0x3) << 16)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_DIVSEL(x)                             (((x) & 0xFF) << 8)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_IBIAS(x)                              (((x) & 0x3) << 6)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_LPF(x)                                (((x) & 0x7) << 3)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_REFIN(x)                              (((x) & 0x3) << 1)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_REFCK_SEL                             BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_CTRL1
// Register Offset : 0x0004
// Description     :
---------------------------------------------------------------------------*/

#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_PD                                    BIT(1)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_RST                                   BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_CTRL2
// Register Offset : 0x0008
// Description     :
---------------------------------------------------------------------------*/

#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_RESERVED(x)                           (((x) & 0xFF) << 4)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_POSTDIV(x)                            (((x) & 0x3) << 2)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_CLKOUTEN                              BIT(1)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_LOCK_DET                              BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_BIST_CTRL
// Register Offset : 0x000C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_BIST_EN                               BIT(24)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_BIST_CTRL(x)                          (((x) & 0xFF) << 16)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DESPLL_BIST_CNT(x)                           (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_CTRL0
// Register Offset : 0x0010
// Description     :
---------------------------------------------------------------------------*/

#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_N(x)                                    (((x) & 0x7FF) << 18)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_IBIAS(x)                                (((x) & 0x3) << 16)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_LPF(x)                                  (((x) & 0x7) << 13)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_SDM_EN                                  BIT(12)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_MOD_EN                                  BIT(11)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_DIV_S                                   BIT(10)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_RESERVED(x)                             (((x) & 0xFF) << 2)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_CLKOUT_EN                               BIT(1)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_LOCK_DONE                               BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_CTRL1
// Register Offset : 0x0014
// Description     :
---------------------------------------------------------------------------*/

#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_NINT(x)                                 (((x) & 0x7F) << 25)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_KINT(x)                                 (((x) & 0x7FFFFF) << 2)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_PD                                      BIT(1)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_RST                                     BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_CTRL2
// Register Offset : 0x0018
// Description     :
---------------------------------------------------------------------------*/

#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_CCS_CTRL(x)                             (((x) & 0xFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_BIST_CTRL
// Register Offset : 0x001C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_BIST_EN                                 BIT(24)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_BIST_CTRL(x)                            (((x) & 0xFF) << 16)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_BIST_CNT(x)                             (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_THM_CTRL
// Register Offset : 0x0020
// Description     :
---------------------------------------------------------------------------*/

#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_TEST_THM1_SEL                           BIT(13)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_ANALOG_PLL_RESERVED(x)                       (((x) & 0x7FF) << 2)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_TEST_CLK_EN                             BIT(1)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_DPLL_TEST_SEL                                BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_THM1_CTRL_0
// Register Offset : 0x0024
// Description     :
---------------------------------------------------------------------------*/

#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_THM1_PD                                      BIT(23)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_THM1_RSTN                                    BIT(22)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_THM1_RUN                                     BIT(21)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_THM1_CALI_EN                                 BIT(20)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_THM1_ITUNE(x)                                (((x) & 0xF) << 16)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_THM1_RESERVED(x)                             (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_THM1_CTRL_1
// Register Offset : 0x0028
// Description     :
---------------------------------------------------------------------------*/

#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_THM1_DATA(x)                                 (((x) & 0xFF) << 1)
#define BIT_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_THM1_VALID                                   BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_ANLG_PHY_G4_ANALOG_DPLL_THM_TOP_REG_SEL_CFG_0
// Register Offset : 0x002C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_ANLG_PHY_G4_DBG_SEL_ANALOG_DPLL_THM_TOP_DESPLL_CLKOUTEN                      BIT(9)
#define BIT_ANLG_PHY_G4_DBG_SEL_ANALOG_DPLL_THM_TOP_DPLL_CLKOUT_EN                       BIT(8)
#define BIT_ANLG_PHY_G4_DBG_SEL_ANALOG_DPLL_THM_TOP_DPLL_PD                              BIT(7)
#define BIT_ANLG_PHY_G4_DBG_SEL_ANALOG_DPLL_THM_TOP_DPLL_RST                             BIT(6)
#define BIT_ANLG_PHY_G4_DBG_SEL_ANALOG_DPLL_THM_TOP_THM1_PD                              BIT(5)
#define BIT_ANLG_PHY_G4_DBG_SEL_ANALOG_DPLL_THM_TOP_THM1_RSTN                            BIT(4)
#define BIT_ANLG_PHY_G4_DBG_SEL_ANALOG_DPLL_THM_TOP_THM1_RUN                             BIT(3)
#define BIT_ANLG_PHY_G4_DBG_SEL_ANALOG_DPLL_THM_TOP_THM1_CALI_EN                         BIT(2)
#define BIT_ANLG_PHY_G4_DBG_SEL_ANALOG_DPLL_THM_TOP_THM1_ITUNE                           BIT(1)
#define BIT_ANLG_PHY_G4_DBG_SEL_ANALOG_DPLL_THM_TOP_THM1_RESERVED                        BIT(0)


#endif

/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This file is dual-licensed: you can use it either under the terms
 * of the GPL or the X11 license, at your option. Note that this dual
 * licensing only applies to this file, and not this project as a
 * whole.
 *
 */


#ifndef _AP_AHB_REG_H
#define _AP_AHB_REG_H



#define REG_AP_AHB_AHB_EB                      0x0000
#define REG_AP_AHB_AHB_RST                     0x0004
#define REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG      0x000C
#define REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG       0x0010
#define REG_AP_AHB_RESERVED0_REG               0x0018
#define REG_AP_AHB_RESERVED1_REG               0x001C
#define REG_AP_AHB_RESERVED2_REG               0x0020
#define REG_AP_AHB_AP_MTX_WQOS_CFG             0x3008
#define REG_AP_AHB_AP_MTX_RQOS_CFG             0x3010
#define REG_AP_AHB_AP_QOS_CFG                  0x301C
#define REG_AP_AHB_OTG_PHY_TUNE                0x3020
#define REG_AP_AHB_OTG_PHY_TEST                0x3024
#define REG_AP_AHB_OTG_PHY_CTRL                0x3028
#define REG_AP_AHB_OTG_CTRL0                   0x302C
#define REG_AP_AHB_OTG_CTRL1                   0x3030
#define REG_AP_AHB_MST_FRC_LSLP                0x3034
#define REG_AP_AHB_MST_PUB_FRC_DSLP            0x3038
#define REG_AP_AHB_CE_EFUSE_STATUS             0x303C
#define REG_AP_AHB_M1_LPC                      0x3064
#define REG_AP_AHB_M2_LPC                      0x3068
#define REG_AP_AHB_M3_LPC                      0x306C
#define REG_AP_AHB_M4_LPC                      0x3070
#define REG_AP_AHB_M5_LPC                      0x3074
#define REG_AP_AHB_M6_LPC                      0x3078
#define REG_AP_AHB_M7_LPC                      0x307C
#define REG_AP_AHB_M8_LPC                      0x3080
#define REG_AP_AHB_M9_LPC                      0x3084
#define REG_AP_AHB_MAIN_LPC                    0x3088
#define REG_AP_AHB_S0_LPC                      0x308C
#define REG_AP_AHB_S1_LPC                      0x3090
#define REG_AP_AHB_S2_LPC                      0x3094
#define REG_AP_AHB_S3_LPC                      0x3098
#define REG_AP_AHB_S4_LPC                      0x309C
#define REG_AP_AHB_S5_LPC                      0x30A0
#define REG_AP_AHB_M10_LPC                     0x30A4
#define REG_AP_AHB_DISP_GSP_FRC_LSLP           0x30A8
#define REG_AP_AHB_DISP_M0_LPC                 0x30AC
#define REG_AP_AHB_DISP_M1_LPC                 0x30B0
#define REG_AP_AHB_DISP_S0_LPC                 0x30B8
#define REG_AP_AHB_GSP_M0_LPC                  0x30BC

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_AHB_EB
// Register Offset : 0x0000
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_SDIO2_32K_EB                               BIT(30)
#define BIT_AP_AHB_SDIO1_32K_EB                               BIT(29)
#define BIT_AP_AHB_SDIO0_32K_EB                               BIT(28)
#define BIT_AP_AHB_EMMC_32K_EB                                BIT(27)
#define BIT_AP_AHB_CE_EFUSE_EB                                BIT(23)
#define BIT_AP_AHB_CE_SEC_EB                                  BIT(12)
#define BIT_AP_AHB_EMMC_EB                                    BIT(11)
#define BIT_AP_AHB_SDIO2_EB                                   BIT(10)
#define BIT_AP_AHB_SDIO1_EB                                   BIT(9)
#define BIT_AP_AHB_SDIO0_EB                                   BIT(8)
#define BIT_AP_AHB_NANDC_EB                                   BIT(7)
#define BIT_AP_AHB_CE_PUB_EB                                  BIT(6)
#define BIT_AP_AHB_DMA_EB                                     BIT(5)
#define BIT_AP_AHB_OTG_EB                                     BIT(4)
#define BIT_AP_AHB_GSP_EB                                     BIT(3)
#define BIT_AP_AHB_DISPC_EB                                   BIT(1)
#define BIT_AP_AHB_DSI_EB                                     BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_AHB_RST
// Register Offset : 0x0004
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_CE_EFUSE_SOFT_RST                          BIT(29)
#define BIT_AP_AHB_EMMC_32K_SOFT_RST                          BIT(20)
#define BIT_AP_AHB_SDIO2_32K_SOFT_RST                         BIT(19)
#define BIT_AP_AHB_SDIO1_32K_SOFT_RST                         BIT(18)
#define BIT_AP_AHB_SDIO0_32K_SOFT_RST                         BIT(17)
#define BIT_AP_AHB_CE_SEC_SOFT_RST                            BIT(15)
#define BIT_AP_AHB_EMMC_SOFT_RST                              BIT(14)
#define BIT_AP_AHB_SDIO2_SOFT_RST                             BIT(13)
#define BIT_AP_AHB_SDIO1_SOFT_RST                             BIT(12)
#define BIT_AP_AHB_SDIO0_SOFT_RST                             BIT(11)
#define BIT_AP_AHB_NANDC_SOFT_RST                             BIT(10)
#define BIT_AP_AHB_CE_PUB_SOFT_RST                            BIT(9)
#define BIT_AP_AHB_DMA_SOFT_RST                               BIT(8)
#define BIT_AP_AHB_OTG_UTMI_SOFT_RST                          BIT(5)
#define BIT_OTG_UTMI_SOFT_RST			BIT_AP_AHB_OTG_UTMI_SOFT_RST
#define BIT_AP_AHB_OTG_SOFT_RST                               BIT(4)
#define BIT_OTG_SOFT_RST			BIT_AP_AHB_OTG_SOFT_RST
#define BIT_AP_AHB_GSP_SOFT_RST                               BIT(3)
#define BIT_AP_AHB_DISP_MTX_SOFT_RST                          BIT(2)
#define BIT_AP_AHB_DISPC_SOFT_RST                             BIT(1)
#define BIT_AP_AHB_DSI_SOFT_RST                               BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG
// Register Offset : 0x000C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_FRC_DISP_LIGHT                             BIT(7)
#define BIT_AP_AHB_FRC_GSP_LIGHT                              BIT(6)
#define BIT_AP_AHB_MCU_SLEEP_FOLLOW_CA53_EN                   BIT(5)
#define BIT_AP_AHB_MCU_CORE_SLEEP                             BIT(4)
#define BIT_AP_AHB_DMA_ACT_LIGHT_EN                           BIT(3)
#define BIT_AP_AHB_AP_PERI_FORCE_ON                           BIT(2)
#define BIT_AP_AHB_AP_PERI_FORCE_SLP                          BIT(1)
#define BIT_AP_AHB_AP_APB_SLEEP                               BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG
// Register Offset : 0x0010
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_CACTIVE_SLV3_WAKEUP_EN                     BIT(10)
#define BIT_AP_AHB_AP_CLK_GATE_LPC_BYP                        BIT(8)
#define BIT_AP_AHB_LP_AUTO_CTRL_EN                            BIT(7)
#define BIT_AP_AHB_AP_MAINMTX_LP_DISABLE                      BIT(6)
#define BIT_AP_AHB_DISP_CLK_GATE_LPC_BYP                      BIT(5)
#define BIT_AP_AHB_DISP_LP_AUTO_CTRL_EN                       BIT(4)
#define BIT_AP_AHB_DISP_MAINMTX_LP_DISABLE                    BIT(3)
#define BIT_AP_AHB_GSP_CLK_GATE_LPC_BYP                       BIT(2)
#define BIT_AP_AHB_GSP_LP_AUTO_CTRL_EN                        BIT(1)
#define BIT_AP_AHB_GSP_MAINMTX_LP_DISABLE                     BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_RESERVED0_REG
// Register Offset : 0x0018
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_RESERVED0_REG(x)                           (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_RESERVED1_REG
// Register Offset : 0x001C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_RESERVED1_REG(x)                           (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_RESERVED2_REG
// Register Offset : 0x0020
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_RESERVED2_REG(x)                           (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_AP_MTX_WQOS_CFG
// Register Offset : 0x3008
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_AWQOS_M9(x)                                (((x) & 0xF) << 28)
#define BIT_AP_AHB_AWQOS_M8(x)                                (((x) & 0xF) << 24)
#define BIT_AP_AHB_AWQOS_M7(x)                                (((x) & 0xF) << 20)
#define BIT_AP_AHB_AWQOS_M6(x)                                (((x) & 0xF) << 16)
#define BIT_AP_AHB_AWQOS_M5(x)                                (((x) & 0xF) << 12)
#define BIT_AP_AHB_AWQOS_M4(x)                                (((x) & 0xF) << 8)
#define BIT_AP_AHB_AWQOS_M3(x)                                (((x) & 0xF) << 4)
#define BIT_AP_AHB_AWQOS_M2(x)                                (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_AP_MTX_RQOS_CFG
// Register Offset : 0x3010
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_ARQOS_M9(x)                                (((x) & 0xF) << 28)
#define BIT_AP_AHB_ARQOS_M8(x)                                (((x) & 0xF) << 24)
#define BIT_AP_AHB_ARQOS_M7(x)                                (((x) & 0xF) << 20)
#define BIT_AP_AHB_ARQOS_M6(x)                                (((x) & 0xF) << 16)
#define BIT_AP_AHB_ARQOS_M5(x)                                (((x) & 0xF) << 12)
#define BIT_AP_AHB_ARQOS_M4(x)                                (((x) & 0xF) << 8)
#define BIT_AP_AHB_ARQOS_M3(x)                                (((x) & 0xF) << 4)
#define BIT_AP_AHB_ARQOS_M2(x)                                (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_AP_QOS_CFG
// Register Offset : 0x301C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_ARQOS_TMC(x)                               (((x) & 0xF) << 20)
#define BIT_AP_AHB_AWQOS_TMC(x)                               (((x) & 0xF) << 16)
#define BIT_AP_AHB_ARQOS_M10(x)                               (((x) & 0xF) << 4)
#define BIT_AP_AHB_AWQOS_M10(x)                               (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_OTG_PHY_TUNE
// Register Offset : 0x3020
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_OTG_TXPREEMPPULSETUNE                      BIT(20)
#define BIT_AP_AHB_OTG_TXRESTUNE(x)                           (((x) & 0x3) << 18)
#define BIT_AP_AHB_OTG_TXHSXVTUNE(x)                          (((x) & 0x3) << 16)
#define BIT_AP_AHB_OTG_TXVREFTUNE(x)                          (((x) & 0xF) << 12)
#define BIT_AP_AHB_OTG_TXPREEMPAMPTUNE(x)                     (((x) & 0x3) << 10)
#define BIT_AP_AHB_OTG_TXRISETUNE(x)                          (((x) & 0x3) << 8)
#define BIT_AP_AHB_OTG_TXFSLSTUNE(x)                          (((x) & 0xF) << 4)
#define BIT_AP_AHB_OTG_SQRXTUNE(x)                            (((x) & 0x7))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_OTG_PHY_TEST
// Register Offset : 0x3024
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_OTG_ATERESET                               BIT(31)
#define BIT_AP_AHB_OTG_VBUS_VALID_PHYREG                      BIT(28)
#define BIT_AP_AHB_OTG_VBUS_VALID_PHYREG_SEL                  BIT(27)
#define BIT_AP_AHB_OTG_VBUS_VALID_EXT_SEL                     BIT(26)
#define BIT_AP_AHB_OTG_VBUS_VALID_EXT                         BIT(25)
#define BIT_AP_AHB_OTG_OTGDISABLE                             BIT(24)
#define BIT_AP_AHB_OTG_BIST_T2R                               BIT(23)
#define BIT_AP_AHB_OTG_TESTBURNIN                             BIT(21)
#define BIT_AP_AHB_OTG_LOOPBACKENB                            BIT(20)
#define BIT_AP_AHB_OTG_TESTDATAOUT(x)                         (((x) & 0xF) << 16)
#define BIT_AP_AHB_OTG_VATESTENB(x)                           (((x) & 0x3) << 14)
#define BIT_AP_AHB_OTG_TESTCLK                                BIT(13)
#define BIT_AP_AHB_OTG_TESTDATAOUTSEL                         BIT(12)
#define BIT_AP_AHB_OTG_TESTADDR(x)                            (((x) & 0xF) << 8)
#define BIT_AP_AHB_OTG_TESTDATAIN(x)                          (((x) & 0xFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_OTG_PHY_CTRL
// Register Offset : 0x3028
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_USB2_CON_TESTMODE                          BIT(31)
#define BIT_AP_AHB_UTMI_WIDTH_SEL                             BIT(30)
#define BIT_AP_AHB_USB2_DATABUS16_8                           BIT(29)
#define BIT_AP_AHB_OTG_SS_SCALEDOWNMODE(x)                    (((x) & 0x3) << 25)
#define BIT_AP_AHB_OTG_TXBITSTUFFENH                          BIT(23)
#define BIT_AP_AHB_OTG_TXBITSTUFFEN                           BIT(22)
#define BIT_AP_AHB_OTG_DMPULLDOWN                             BIT(21)
#define BIT_AP_AHB_OTG_DPPULLDOWN                             BIT(20)
#define BIT_AP_AHB_OTG_DMPULLUP                               BIT(9)
#define BIT_AP_AHB_OTG_COMMONONN                              BIT(8)
#define BIT_AP_AHB_USB2_PHY_IDDIG                             BIT(3)
#define BIT_AP_AHB_OTG_FSEL(x)                                (((x) & 0x7))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_OTG_CTRL0
// Register Offset : 0x302C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_USB20_TUNEHSAMP(x)                         (((x) & 0x3) << 30)
#define BIT_AP_AHB_USB20_TUNEPLLS(x)                          (((x) & 0x3) << 28)
#define BIT_AP_AHB_USB20_TUNERISE(x)                          (((x) & 0x3) << 26)
#define BIT_AP_AHB_USB20_TUNEDSC(x)                           (((x) & 0x3) << 24)
#define BIT_AP_AHB_USB20_TUNEOTG(x)                           (((x) & 0x7) << 21)
#define BIT_AP_AHB_USB20_TUNESQ(x)                            (((x) & 0xF) << 17)
#define BIT_AP_AHB_USB20_RESERVED(x)                          (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_OTG_CTRL1
// Register Offset : 0x3030
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_USB20_BYPASS_DRV_DM                        BIT(22)
#define BIT_AP_AHB_USB20_BYPASS_DRV_DP                        BIT(21)
#define BIT_AP_AHB_USB20_SAMPLER_SEL                          BIT(20)
#define BIT_AP_AHB_USB20_BIST_MODE(x)                         (((x) & 0x7) << 17)
#define BIT_AP_AHB_HSIC_PLLON                                 BIT(16)
#define BIT_AP_AHB_USB20_REXTENABLE                           BIT(15)
#define BIT_AP_AHB_USB20_S_ID                                 BIT(14)
#define BIT_AP_AHB_USB20_TF12KRES(x)                          (((x) & 0x3F) << 8)
#define BIT_AP_AHB_USB20_TFHSRES(x)                           (((x) & 0x1F) << 3)
#define BIT_AP_AHB_USB20_TUNEEQ(x)                            (((x) & 0x7))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_MST_FRC_LSLP
// Register Offset : 0x3034
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_SLV3_FRC_LSLP                              BIT(11)
#define BIT_AP_AHB_MST10_FRC_LSLP                             BIT(10)
#define BIT_AP_AHB_MST9_FRC_LSLP                              BIT(9)
#define BIT_AP_AHB_MST8_FRC_LSLP                              BIT(8)
#define BIT_AP_AHB_MST7_FRC_LSLP                              BIT(7)
#define BIT_AP_AHB_MST6_FRC_LSLP                              BIT(6)
#define BIT_AP_AHB_MST5_FRC_LSLP                              BIT(5)
#define BIT_AP_AHB_MST4_FRC_LSLP                              BIT(4)
#define BIT_AP_AHB_MST3_FRC_LSLP                              BIT(3)
#define BIT_AP_AHB_MST2_FRC_LSLP                              BIT(2)
#define BIT_AP_AHB_MST1_FRC_LSLP                              BIT(1)

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_MST_PUB_FRC_DSLP
// Register Offset : 0x3038
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_MST10_PUB_FRC_DSLP                         BIT(10)
#define BIT_AP_AHB_MST9_PUB_FRC_DSLP                          BIT(9)
#define BIT_AP_AHB_MST8_PUB_FRC_DSLP                          BIT(8)
#define BIT_AP_AHB_MST7_PUB_FRC_DSLP                          BIT(7)
#define BIT_AP_AHB_MST6_PUB_FRC_DSLP                          BIT(6)
#define BIT_AP_AHB_MST5_PUB_FRC_DSLP                          BIT(5)
#define BIT_AP_AHB_MST4_PUB_FRC_DSLP                          BIT(4)
#define BIT_AP_AHB_MST3_PUB_FRC_DSLP                          BIT(3)
#define BIT_AP_AHB_MST2_PUB_FRC_DSLP                          BIT(2)
#define BIT_AP_AHB_MST1_PUB_FRC_DSLP                          BIT(1)

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_CE_EFUSE_STATUS
// Register Offset : 0x303C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_EFUSE_PWON_RD_END_FLAG                     BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_M1_LPC
// Register Offset : 0x3064
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_M1_LP_ACTIVE_SYNC_SEL                      BIT(18)
#define BIT_AP_AHB_M1_LP_EB                                   BIT(16)
#define BIT_AP_AHB_M1_LP_NUM(x)                               (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_M2_LPC
// Register Offset : 0x3068
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_M2_LP_ACTIVE_SYNC_SEL                      BIT(18)
#define BIT_AP_AHB_M2_LP_FORCE                                BIT(17)
#define BIT_AP_AHB_M2_LP_EB                                   BIT(16)
#define BIT_AP_AHB_M2_LP_NUM(x)                               (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_M3_LPC
// Register Offset : 0x306C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_M3_LP_ACTIVE_SYNC_SEL                      BIT(18)
#define BIT_AP_AHB_M3_LP_FORCE                                BIT(17)
#define BIT_AP_AHB_M3_LP_EB                                   BIT(16)
#define BIT_AP_AHB_M3_LP_NUM(x)                               (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_M4_LPC
// Register Offset : 0x3070
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_M4_LP_ACTIVE_SYNC_SEL                      BIT(18)
#define BIT_AP_AHB_M4_LP_FORCE                                BIT(17)
#define BIT_AP_AHB_M4_LP_EB                                   BIT(16)
#define BIT_AP_AHB_M4_LP_NUM(x)                               (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_M5_LPC
// Register Offset : 0x3074
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_M5_LP_ACTIVE_SYNC_SEL                      BIT(18)
#define BIT_AP_AHB_M5_LP_FORCE                                BIT(17)
#define BIT_AP_AHB_M5_LP_EB                                   BIT(16)
#define BIT_AP_AHB_M5_LP_NUM(x)                               (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_M6_LPC
// Register Offset : 0x3078
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_M6_LP_ACTIVE_SYNC_SEL                      BIT(18)
#define BIT_AP_AHB_M6_LP_FORCE                                BIT(17)
#define BIT_AP_AHB_M6_LP_EB                                   BIT(16)
#define BIT_AP_AHB_M6_LP_NUM(x)                               (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_M7_LPC
// Register Offset : 0x307C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_M7_LP_ACTIVE_SYNC_SEL                      BIT(18)
#define BIT_AP_AHB_M7_LP_FORCE                                BIT(17)
#define BIT_AP_AHB_M7_LP_EB                                   BIT(16)
#define BIT_AP_AHB_M7_LP_NUM(x)                               (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_M8_LPC
// Register Offset : 0x3080
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_M8_LP_ACTIVE_SYNC_SEL                      BIT(18)
#define BIT_AP_AHB_M8_LP_FORCE                                BIT(17)
#define BIT_AP_AHB_M8_LP_EB                                   BIT(16)
#define BIT_AP_AHB_M8_LP_NUM(x)                               (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_M9_LPC
// Register Offset : 0x3084
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_M9_LP_ACTIVE_SYNC_SEL                      BIT(18)
#define BIT_AP_AHB_M9_LP_FORCE                                BIT(17)
#define BIT_AP_AHB_M9_LP_EB                                   BIT(16)
#define BIT_AP_AHB_M9_LP_NUM(x)                               (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_MAIN_LPC
// Register Offset : 0x3088
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_MAIN_LP_ACTIVE_SYNC_SEL                    BIT(18)
#define BIT_AP_AHB_MAIN_LP_EB                                 BIT(16)
#define BIT_AP_AHB_MAIN_LP_NUM(x)                             (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_S0_LPC
// Register Offset : 0x308C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_S0_LP_ACTIVE_SYNC_SEL                      BIT(18)
#define BIT_AP_AHB_S0_LP_EB                                   BIT(16)
#define BIT_AP_AHB_S0_LP_NUM(x)                               (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_S1_LPC
// Register Offset : 0x3090
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_S1_LP_ACTIVE_SYNC_SEL                      BIT(18)
#define BIT_AP_AHB_S1_LP_FORCE                                BIT(17)
#define BIT_AP_AHB_S1_LP_EB                                   BIT(16)
#define BIT_AP_AHB_S1_LP_NUM(x)                               (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_S2_LPC
// Register Offset : 0x3094
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_S2_LP_ACTIVE_SYNC_SEL                      BIT(18)
#define BIT_AP_AHB_S2_LP_FORCE                                BIT(17)
#define BIT_AP_AHB_S2_LP_EB                                   BIT(16)
#define BIT_AP_AHB_S2_LP_NUM(x)                               (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_S3_LPC
// Register Offset : 0x3098
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_S3_LP_ACTIVE_SYNC_SEL                      BIT(18)
#define BIT_AP_AHB_S3_LP_FORCE                                BIT(17)
#define BIT_AP_AHB_S3_LP_EB                                   BIT(16)
#define BIT_AP_AHB_S3_LP_NUM(x)                               (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_S4_LPC
// Register Offset : 0x309C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_S4_LP_ACTIVE_SYNC_SEL                      BIT(18)
#define BIT_AP_AHB_S4_LP_FORCE                                BIT(17)
#define BIT_AP_AHB_S4_LP_EB                                   BIT(16)
#define BIT_AP_AHB_S4_LP_NUM(x)                               (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_S5_LPC
// Register Offset : 0x30A0
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_S5_LP_ACTIVE_SYNC_SEL                      BIT(18)
#define BIT_AP_AHB_S5_LP_FORCE                                BIT(17)
#define BIT_AP_AHB_S5_LP_EB                                   BIT(16)
#define BIT_AP_AHB_S5_LP_NUM(x)                               (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_M10_LPC
// Register Offset : 0x30A4
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_M10_LP_ACTIVE_SYNC_SEL                     BIT(18)
#define BIT_AP_AHB_M10_LP_FORCE                               BIT(17)
#define BIT_AP_AHB_M10_LP_EB                                  BIT(16)
#define BIT_AP_AHB_M10_LP_NUM(x)                              (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_DISP_GSP_FRC_LSLP
// Register Offset : 0x30A8
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_GSP_MST0_FRC_LSLP                          BIT(4)
#define BIT_AP_AHB_DISP_SLV0_FRC_LSLP                         BIT(3)
#define BIT_AP_AHB_DISP_MST1_FRC_LSLP                         BIT(1)

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_DISP_M0_LPC
// Register Offset : 0x30AC
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_DISP_M0_LP_ACTIVE_SYNC_SEL                 BIT(18)
#define BIT_AP_AHB_DISP_M0_LP_FORCE                           BIT(17)
#define BIT_AP_AHB_DISP_M0_LP_EB                              BIT(16)
#define BIT_AP_AHB_DISP_M0_LP_NUM(x)                          (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_DISP_M1_LPC
// Register Offset : 0x30B0
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_DISP_M1_LP_ACTIVE_SYNC_SEL                 BIT(18)
#define BIT_AP_AHB_DISP_M1_LP_FORCE                           BIT(17)
#define BIT_AP_AHB_DISP_M1_LP_EB                              BIT(16)
#define BIT_AP_AHB_DISP_M1_LP_NUM(x)                          (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_DISP_S0_LPC
// Register Offset : 0x30B8
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_DISP_S0_LP_ACTIVE_SYNC_SEL                 BIT(18)
#define BIT_AP_AHB_DISP_S0_LP_FORCE                           BIT(17)
#define BIT_AP_AHB_DISP_S0_LP_EB                              BIT(16)
#define BIT_AP_AHB_DISP_S0_LP_NUM(x)                          (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AP_AHB_GSP_M0_LPC
// Register Offset : 0x30BC
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_GSP_M0_LP_ACTIVE_SYNC_SEL                  BIT(18)
#define BIT_AP_AHB_GSP_M0_LP_FORCE                            BIT(17)
#define BIT_AP_AHB_GSP_M0_LP_EB                               BIT(16)
#define BIT_AP_AHB_GSP_M0_LP_NUM(x)                           (((x) & 0xFFFF))


#endif

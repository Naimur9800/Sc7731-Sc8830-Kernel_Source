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


#define REG_AP_AHB_AHB_EB                               0x0000
#define REG_AP_AHB_AHB_RST                              0x0004
#define REG_AP_AHB_APCPU_RST_SET                        0x0008
#define REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG               0x000C
#define REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG                0x0010
#define REG_AP_AHB_HOLDING_PEN                          0x0014
#define REG_AP_AHB_JMP_ADDR_APCPU_C0                    0x0018
#define REG_AP_AHB_JMP_ADDR_APCPU_C1                    0x001C
#define REG_AP_AHB_JMP_ADDR_APCPU_C2                    0x0020
#define REG_AP_AHB_JMP_ADDR_APCPU_C3                    0x0024
#define REG_AP_AHB_APCPU_C0_PU_LOCK                     0x0028
#define REG_AP_AHB_APCPU_C1_PU_LOCK                     0x002C
#define REG_AP_AHB_APCPU_C2_PU_LOCK                     0x0030
#define REG_AP_AHB_APCPU_C3_PU_LOCK                     0x0034
#define REG_AP_AHB_APCPU_CKG_DIV_CFG                    0x0038
#define REG_AP_AHB_MCU_PAUSE                            0x003C
#define REG_AP_AHB_MISC_CKG_EN                          0x0040
#define REG_AP_AHB_APCPU_C0_AUTO_FORCE_SHUTDOWN_EN      0x0044
#define REG_AP_AHB_APCPU_C1_AUTO_FORCE_SHUTDOWN_EN      0x0048
#define REG_AP_AHB_APCPU_C2_AUTO_FORCE_SHUTDOWN_EN      0x004C
#define REG_AP_AHB_APCPU_C3_AUTO_FORCE_SHUTDOWN_EN      0x0050
#define REG_AP_AHB_APCPU_CKG_SEL_CFG                    0x0054
#define REG_AP_AHB_APCPU_AUTO_GATE_EN                   0x0058
#define REG_AP_AHB_APCPU_GIC_CKG_CFG                    0x005C
#define REG_AP_AHB_APCPU_GIC_CLK_CFG                    0x0060
#define REG_AP_AHB_APCPU_LIT_CKG_EN                     0x0064
#define REG_AP_AHB_APCPU_BIG_CKG_EN                     0x0068
#define REG_AP_AHB_APCPU_LIT_L2FLUSH                    0x006C
#define REG_AP_AHB_APCPU_BIG_L2FLUSH                    0x0070
#define REG_AP_AHB_APCPU_CCI_BASE_H25                   0x0074
#define REG_AP_AHB_APCPU_CCI_CTRL                       0x0078
#define REG_AP_AHB_MISC_CFG                             0x3000
#define REG_AP_AHB_AP_MAIN_MTX_HPROT_CFG                0x3004
#define REG_AP_AHB_APCPU_STANDBY_STATUS                 0x3008
#define REG_AP_AHB_NANC_CLK_CFG                         0x300C
#define REG_AP_AHB_LVDS_CFG                             0x3010
#define REG_AP_AHB_LVDS_PLL_CFG0                        0x3014
#define REG_AP_AHB_LVDS_PLL_CFG1                        0x3018
#define REG_AP_AHB_AP_QOS_CFG                           0x301C
#define REG_AP_AHB_OTG_PHY_TUNE                         0x3020
#define REG_AP_AHB_OTG_PHY_TEST                         0x3024
#define REG_AP_AHB_OTG_PHY_CTRL                         0x3028
#define REG_AP_AHB_HSIC_PHY_TUNE                        0x302C
#define REG_AP_AHB_HSIC_PHY_TEST                        0x3030
#define REG_AP_AHB_HSIC_PHY_CTRL                        0x3034
#define REG_AP_AHB_ZIP_MTX_QOS_CFG                      0x3038
#define REG_AP_AHB_APCPU_QCHN_CTL_BIG                   0x303C
#define REG_AP_AHB_APCPU_QCHN_STA_BIG                   0x3040
#define REG_AP_AHB_APCPU_QCHN_CTL_LIT                   0x3044
#define REG_AP_AHB_APCPU_QCHN_STA_LIT                   0x3048
#define REG_AP_AHB_GSP_SEL                              0x304C
#define REG_AP_AHB_RES_REG                              0x30F0
#define REG_AP_AHB_CHIP_ID                              0x30FC

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_AHB_EB
** Register Offset : 0x0000
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_ZIPMTX_EB                            BIT(23)
#define BIT_AP_AHB_LVDS_EB                              BIT(22)
#define BIT_AP_AHB_ZIPDEC_EB                            BIT(21)
#define BIT_AP_AHB_ZIPENC_EB                            BIT(20)
#define BIT_AP_AHB_NANDC_ECC_EB                         BIT(19)
#define BIT_AP_AHB_NANDC_2X_EB                          BIT(18)
#define BIT_AP_AHB_NANDC_EB                             BIT(17)
#define BIT_AP_AHB_BUSMON2_EB                           BIT(16)
#define BIT_AP_AHB_BUSMON1_EB                           BIT(15)
#define BIT_AP_AHB_BUSMON0_EB                           BIT(14)
#define BIT_AP_AHB_SPINLOCK_EB                          BIT(13)
#define BIT_AP_AHB_EMMC_EB                              BIT(11)
#define BIT_AP_AHB_SDIO2_EB                             BIT(10)
#define BIT_AP_AHB_SDIO1_EB                             BIT(9)
#define BIT_AP_AHB_SDIO0_EB                             BIT(8)
#define BIT_AP_AHB_DRM_EB                               BIT(7)
#define BIT_AP_AHB_NFC_EB                               BIT(6)
#define BIT_AP_AHB_DMA_EB                               BIT(5)
#define BIT_AP_AHB_OTG_EB                               BIT(4)
#define BIT_AP_AHB_GSP_EB                               BIT(3)
#define BIT_AP_AHB_HSIC_EB                              BIT(2)
#define BIT_AP_AHB_DISPC_EB                             BIT(1)
#define BIT_AP_AHB_DSI_EB                               BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_AHB_RST
** Register Offset : 0x0004
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_HSIC_PHY_SOFT_RST                    BIT(30)
#define BIT_AP_AHB_HSIC_UTMI_SOFT_RST                   BIT(29)
#define BIT_AP_AHB_HSIC_SOFT_RST                        BIT(28)
#define BIT_AP_AHB_LVDS_SOFT_RST                        BIT(25)
#define BIT_AP_AHB_ZIP_MTX_SOFT_RST                     BIT(24)
#define BIT_AP_AHB_ZIPDEC_SOFT_RST                      BIT(23)
#define BIT_AP_AHB_ZIPENC_SOFT_RST                      BIT(22)
#define BIT_AP_AHB_CCI_SOFT_RST                         BIT(21)
#define BIT_AP_AHB_NANDC_SOFT_RST                       BIT(20)
#define BIT_AP_AHB_BUSMON2_SOFT_RST                     BIT(19)
#define BIT_AP_AHB_BUSMON1_SOFT_RST                     BIT(18)
#define BIT_AP_AHB_BUSMON0_SOFT_RST                     BIT(17)
#define BIT_AP_AHB_SPINLOCK_SOFT_RST                    BIT(16)
#define BIT_AP_AHB_EMMC_SOFT_RST                        BIT(14)
#define BIT_AP_AHB_SDIO2_SOFT_RST                       BIT(13)
#define BIT_AP_AHB_SDIO1_SOFT_RST                       BIT(12)
#define BIT_AP_AHB_SDIO0_SOFT_RST                       BIT(11)
#define BIT_AP_AHB_DRM_SOFT_RST                         BIT(10)
#define BIT_AP_AHB_NFC_SOFT_RST                         BIT(9)
#define BIT_AP_AHB_DMA_SOFT_RST                         BIT(8)
#define BIT_AP_AHB_CSSYS_SOFT_RST                       BIT(7)
#define BIT_AP_AHB_OTG_PHY_SOFT_RST                     BIT(6)
#define BIT_AP_AHB_OTG_UTMI_SOFT_RST                    BIT(5)
#define BIT_AP_AHB_OTG_SOFT_RST                         BIT(4)
#define BIT_AP_AHB_GSP_SOFT_RST                         BIT(3)
#define BIT_AP_AHB_DISP_MTX_SOFT_RST                    BIT(2)
#define BIT_AP_AHB_DISPC_SOFT_RST                       BIT(1)
#define BIT_AP_AHB_DSI_SOFT_RST                         BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_RST_SET
** Register Offset : 0x0008
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_AP_QCHN_BIG_SOFT_RST                 BIT(17)
#define BIT_AP_AHB_AP_QCHN_LIT_SOFT_RST                 BIT(16)
#define BIT_AP_AHB_APCPU_GIC_SOFT_RST                   BIT(15)
#define BIT_AP_AHB_APCPU_CS_DBG_SOFT_RST                BIT(14)
#define BIT_AP_AHB_APCPU_SOCDBG_SOFT_RST                BIT(12)
#define BIT_AP_AHB_APCPU_ETM_SOFT_RST(x)                (((x) & 0xF) << 8)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG
** Register Offset : 0x000C
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_C3_AUTO_SLP_EN                 BIT(15)
#define BIT_AP_AHB_APCPU_C2_AUTO_SLP_EN                 BIT(14)
#define BIT_AP_AHB_APCPU_C1_AUTO_SLP_EN                 BIT(13)
#define BIT_AP_AHB_APCPU_C0_AUTO_SLP_EN                 BIT(12)
#define BIT_AP_AHB_APCPU_C3_WFI_SHUTDOWN_EN             BIT(11)
#define BIT_AP_AHB_APCPU_C2_WFI_SHUTDOWN_EN             BIT(10)
#define BIT_AP_AHB_APCPU_C1_WFI_SHUTDOWN_EN             BIT(9)
#define BIT_AP_AHB_APCPU_C0_WFI_SHUTDOWN_EN             BIT(8)
#define BIT_AP_AHB_MCU_APCPU_C3_SLEEP                   BIT(7)
#define BIT_AP_AHB_MCU_APCPU_C2_SLEEP                   BIT(6)
#define BIT_AP_AHB_MCU_APCPU_C1_SLEEP                   BIT(5)
#define BIT_AP_AHB_MCU_APCPU_C0_SLEEP                   BIT(4)
#define BIT_AP_AHB_AP_PERI_FORCE_ON                     BIT(2)
#define BIT_AP_AHB_AP_PERI_FORCE_SLP                    BIT(1)
#define BIT_AP_AHB_AP_APB_SLEEP                         BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG
** Register Offset : 0x0010
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_GSP_CKG_FORCE_EN                     BIT(9)
#define BIT_AP_AHB_GSP_AUTO_GATE_EN                     BIT(8)
#define BIT_AP_AHB_APCPU_TRACE_FORCE_SLEEP              BIT(7)
#define BIT_AP_AHB_APCPU_TRACE_AUTO_GATE_EN             BIT(6)
#define BIT_AP_AHB_AP_AHB_AUTO_GATE_EN                  BIT(5)
#define BIT_AP_AHB_AP_EMC_AUTO_GATE_EN                  BIT(4)
#define BIT_AP_AHB_APCPU_EMC_AUTO_GATE_EN               BIT(3)
#define BIT_AP_AHB_APCPU_DBG_FORCE_SLEEP                BIT(2)
#define BIT_AP_AHB_APCPU_DBG_AUTO_GATE_EN               BIT(1)
#define BIT_AP_AHB_APCPU_CORE_AUTO_GATE_EN              BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_HOLDING_PEN
** Register Offset : 0x0014
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_HOLDING_PEN(x)                       (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_JMP_ADDR_APCPU_C0
** Register Offset : 0x0018
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_JMP_ADDR_APCPU_C0(x)                 (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_JMP_ADDR_APCPU_C1
** Register Offset : 0x001C
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_JMP_ADDR_APCPU_C1(x)                 (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_JMP_ADDR_APCPU_C2
** Register Offset : 0x0020
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_JMP_ADDR_APCPU_C2(x)                 (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_JMP_ADDR_APCPU_C3
** Register Offset : 0x0024
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_JMP_ADDR_APCPU_C3(x)                 (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_C0_PU_LOCK
** Register Offset : 0x0028
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_C0_PU_LOCK                     BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_C1_PU_LOCK
** Register Offset : 0x002C
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_C1_PU_LOCK                     BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_C2_PU_LOCK
** Register Offset : 0x0030
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_C2_PU_LOCK                     BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_C3_PU_LOCK
** Register Offset : 0x0034
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_C3_PU_LOCK                     BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_CKG_DIV_CFG
** Register Offset : 0x0038
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_LIT_ATB_CKG_DIV(x)             (((x) & 0x7) << 23)
#define BIT_AP_AHB_APCPU_BIG_ATB_CKG_DIV(x)             (((x) & 0x7) << 20)
#define BIT_AP_AHB_APCPU_LIT_DBG_CKG_DIV(x)             (((x) & 0x7) << 17)
#define BIT_AP_AHB_APCPU_BIG_DBG_CKG_DIV(x)             (((x) & 0x7) << 14)
#define BIT_AP_AHB_APCPU_LIT_ACE_CKG_DIV(x)             (((x) & 0x7) << 11)
#define BIT_AP_AHB_APCPU_BIG_ACE_CKG_DIV(x)             (((x) & 0x7) << 8)
#define BIT_AP_AHB_APCPU_LIT_MCU_CKG_DIV(x)             (((x) & 0x7) << 4)
#define BIT_AP_AHB_APCPU_BIG_MCU_CKG_DIV(x)             (((x) & 0x7) << 1)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_MCU_PAUSE
** Register Offset : 0x003C
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_DMA_ACT_LIGHT_EN                     BIT(5)
#define BIT_AP_AHB_MCU_SLEEP_FOLLOW_APCPU_EN            BIT(4)
#define BIT_AP_AHB_MCU_LIGHT_SLEEP_EN                   BIT(3)
#define BIT_AP_AHB_MCU_DEEP_SLEEP_EN                    BIT(2)
#define BIT_AP_AHB_MCU_SYS_SLEEP_EN                     BIT(1)
#define BIT_AP_AHB_MCU_CORE_SLEEP                       BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_MISC_CKG_EN
** Register Offset : 0x0040
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_ASHB_APCPU_DBG_VLD                   BIT(9)
#define BIT_AP_AHB_ASHB_APCPU_DBG_EN                    BIT(8)
#define BIT_AP_AHB_DISP_TMC_CKG_EN                      BIT(4)
#define BIT_AP_AHB_DPHY_REF_CKG_EN                      BIT(1)
#define BIT_AP_AHB_DPHY_CFG_CKG_EN                      BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_C0_AUTO_FORCE_SHUTDOWN_EN
** Register Offset : 0x0044
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_C0_AUTO_FORCE_SHUTDOWN_EN      BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_C1_AUTO_FORCE_SHUTDOWN_EN
** Register Offset : 0x0048
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_C1_AUTO_FORCE_SHUTDOWN_EN      BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_C2_AUTO_FORCE_SHUTDOWN_EN
** Register Offset : 0x004C
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_C2_AUTO_FORCE_SHUTDOWN_EN      BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_C3_AUTO_FORCE_SHUTDOWN_EN
** Register Offset : 0x0050
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_C3_AUTO_FORCE_SHUTDOWN_EN      BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_CKG_SEL_CFG
** Register Offset : 0x0054
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_LIT_MCU_CKG_SEL(x)             (((x) & 0x7) << 3)
#define BIT_AP_AHB_APCPU_BIG_MCU_CKG_SEL(x)             (((x) & 0x7))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_AUTO_GATE_EN
** Register Offset : 0x0058
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_AUTO_GATE_EN                   BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_GIC_CKG_CFG
** Register Offset : 0x005C
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_GIC_CKG_EN                     BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_GIC_CLK_CFG
** Register Offset : 0x0060
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_GIC_CLK_DIV(x)                 (((x) & 0x3) << 2)
#define BIT_AP_AHB_APCPU_GIC_CLK_SEL(x)                 (((x) & 0x3))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_LIT_CKG_EN
** Register Offset : 0x0064
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_LIT_ATB_EN                     BIT(2)
#define BIT_AP_AHB_APCPU_LIT_DBG_EN                     BIT(1)
#define BIT_AP_AHB_APCPU_LIT_CORE_EN                    BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_BIG_CKG_EN
** Register Offset : 0x0068
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_BIG_ATB_EN                     BIT(2)
#define BIT_AP_AHB_APCPU_BIG_DBG_EN                     BIT(1)
#define BIT_AP_AHB_APCPU_BIG_CORE_EN                    BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_LIT_L2FLUSH
** Register Offset : 0x006C
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_LIT_L2FLUSHDONE                BIT(1)
#define BIT_AP_AHB_APCPU_LIT_L2FLUSHREQ                 BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_BIG_L2FLUSH
** Register Offset : 0x0070
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_BIG_L2FLUSHDONE                BIT(1)
#define BIT_AP_AHB_APCPU_BIG_L2FLUSHREQ                 BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_CCI_BASE_H25
** Register Offset : 0x0074
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_CCI_BASE_H25(x)                (((x) & 0x1FFFFFF))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_CCI_CTRL
** Register Offset : 0x0078
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_CCI_ARQOS_LIT(x)               (((x) & 0xF) << 20)
#define BIT_AP_AHB_APCPU_CCI_AWQOS_LIT(x)               (((x) & 0xF) << 16)
#define BIT_AP_AHB_APCPU_CCI_ARQOS_BIG(x)               (((x) & 0xF) << 12)
#define BIT_AP_AHB_APCPU_CCI_AWQOS_BIG(x)               (((x) & 0xF) << 8)
#define BIT_AP_AHB_APCPU_CCI_BUFOVRD(x)                 (((x) & 0x7) << 5)
#define BIT_AP_AHB_APCPU_CCI_QOSOVRD(x)                 (((x) & 0x1F))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_MISC_CFG
** Register Offset : 0x3000
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_DAP_SNOOP_DISABLE              BIT(25)
#define BIT_AP_AHB_APCPU_BIG_INT_DISABLE                BIT(24)
#define BIT_AP_AHB_APCPU_IRAM_REMAP                     BIT(23)
#define BIT_AP_AHB_APCPU_L2RAM_LIGHT_SLEEP_EN           BIT(22)
#define BIT_AP_AHB_APCPU_AUTO_REG_SAVE_TRIG_SEL         BIT(21)
#define BIT_AP_AHB_APCPU_AUTO_REG_SAVE_EN               BIT(20)
#define BIT_AP_AHB_EMMC_SLOT_SEL(x)                     (((x) & 0x3) << 18)
#define BIT_AP_AHB_SDIO0_SLOT_SEL(x)                    (((x) & 0x3) << 16)
#define BIT_AP_AHB_BUSMON2_CHN_SEL(x)                   (((x) & 0x3) << 10)
#define BIT_AP_AHB_BUSMON1_CHN_SEL(x)                   (((x) & 0x3) << 8)
#define BIT_AP_AHB_BUSMON0_CHN_SEL(x)                   (((x) & 0x3) << 4)
#define BIT_AP_AHB_SDIO2_SLOT_SEL(x)                    (((x) & 0x3) << 2)
#define BIT_AP_AHB_SDIO1_SLOT_SEL(x)                    (((x) & 0x3))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_AP_MAIN_MTX_HPROT_CFG
** Register Offset : 0x3004
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_HPROT_NFC(x)                        (((x) & 0xF) << 24)
#define BIT_AP_AHB_HPROT_EMMC(x)                       (((x) & 0xF) << 20)
#define BIT_AP_AHB_HPROT_SDIO2(x)                      (((x) & 0xF) << 16)
#define BIT_AP_AHB_HPROT_SDIO1(x)                      (((x) & 0xF) << 12)
#define BIT_AP_AHB_HPROT_SDIO0(x)                      (((x) & 0xF) << 8)
#define BIT_AP_AHB_HPROT_DMAW(x)                       (((x) & 0xF) << 4)
#define BIT_AP_AHB_HPROT_DMAR(x)                       (((x) & 0xF))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_STANDBY_STATUS
** Register Offset : 0x3008
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_STANDBYWFIL2_BIG                    BIT(24)
#define BIT_AP_AHB_STANDBYWFE_BIG(x)                   (((x) & 0xF) << 20)
#define BIT_AP_AHB_STANDBYWFI_BIG(x)                   (((x) & 0xF) << 16)
#define BIT_AP_AHB_STANDBYWFIL2_LIT                    BIT(8)
#define BIT_AP_AHB_STANDBYWFE_LIT(x)                   (((x) & 0xF) << 4)
#define BIT_AP_AHB_STANDBYWFI_LIT(x)                   (((x) & 0xF))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_NANC_CLK_CFG
** Register Offset : 0x300C
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_CLK_NANDC2X_DIV(x)                  (((x) & 0x3) << 2)
#define BIT_AP_AHB_CLK_NANDC2X_SEL(x)                  (((x) & 0x3))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_LVDS_CFG
** Register Offset : 0x3010
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_LVDS_TXCLKDATA(x)                   (((x) & 0x7F) << 16)
#define BIT_AP_AHB_LVDS_TXCOM(x)                       (((x) & 0x3) << 12)
#define BIT_AP_AHB_LVDS_TXSLEW(x)                      (((x) & 0x3) << 10)
#define BIT_AP_AHB_LVDS_TXSW(x)                        (((x) & 0x3) << 8)
#define BIT_AP_AHB_LVDS_TXRERSER(x)                    (((x) & 0x1F) << 3)
#define BIT_AP_AHB_LVDS_PRE_EMP(x)                     (((x) & 0x3) << 1)
#define BIT_AP_AHB_LVDS_TXPD                           BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_LVDS_PLL_CFG0
** Register Offset : 0x3014
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_LVDS_PLL_LOCK_DET                  BIT(31)
#define BIT_AP_AHB_LVDS_PLL_REFIN(x)                  (((x) & 0x3) << 24)
#define BIT_AP_AHB_LVDS_PLL_LPF(x)                    (((x) & 0x7) << 20)
#define BIT_AP_AHB_LVDS_PLL_DIV_S                     BIT(18)
#define BIT_AP_AHB_LVDS_PLL_IBIAS(x)                  (((x) & 0x3) << 16)
#define BIT_AP_AHB_LVDS_PLLN(x)                       (((x) & 0x7FF))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_LVDS_PLL_CFG1
** Register Offset : 0x3018
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_LVDS_PLL_KINT(x)                   (((x) & 0xFFFFF) << 12)
#define BIT_AP_AHB_LVDS_PLL_RSV(x)                    (((x) & 0x3) << 8)
#define BIT_AP_AHB_LVDS_PLL_MOD_EN                    BIT(7)
#define BIT_AP_AHB_LVDS_PLL_SDM_EN                    BIT(6)
#define BIT_AP_AHB_LVDS_PLL_NINT(x)                   (((x) & 0x3F))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_AP_QOS_CFG
** Register Offset : 0x301C
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_QOS_R_TMC(x)                      (((x) & 0xF) << 20)
#define BIT_AP_AHB_QOS_W_TMC(x)                      (((x) & 0xF) << 16)
#define BIT_AP_AHB_QOS_R_DISPC(x)                    (((x) & 0xF) << 4)
#define BIT_AP_AHB_QOS_W_DISPC(x)                    (((x) & 0xF))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_OTG_PHY_TUNE
** Register Offset : 0x3020
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_OTG_TXPREEMPPULSETUNE             BIT(20)
#define BIT_AP_AHB_OTG_TXRESTUNE(x)                  (((x) & 0x3) << 18)
#define BIT_AP_AHB_OTG_TXHSXVTUNE(x)                 (((x) & 0x3) << 16)
#define BIT_AP_AHB_OTG_TXVREFTUNE(x)                 (((x) & 0xF) << 12)
#define BIT_AP_AHB_OTG_TXPREEMPAMPTUNE(x)            (((x) & 0x3) << 10)
#define BIT_AP_AHB_OTG_TXRISETUNE(x)                 (((x) & 0x3) << 8)
#define BIT_AP_AHB_OTG_TXFSLSTUNE(x)                 (((x) & 0xF) << 4)
#define BIT_AP_AHB_OTG_SQRXTUNE(x)                   (((x) & 0x7))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_OTG_PHY_TEST
** Register Offset : 0x3024
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_OTG_ATERESET                      BIT(31)
#define BIT_AP_AHB_OTG_VBUS_VALID_EXT_SEL            BIT(26)
#define BIT_AP_AHB_OTG_VBUS_VALID_EXT                BIT(25)
#define BIT_AP_AHB_OTG_OTGDISABLE                    BIT(24)
#define BIT_AP_AHB_OTG_TESTBURNIN                    BIT(21)
#define BIT_AP_AHB_OTG_LOOPBACKENB                   BIT(20)
#define BIT_AP_AHB_OTG_TESTDATAOUT(x)                (((x) & 0xF) << 16)
#define BIT_AP_AHB_OTG_VATESTENB(x)                  (((x) & 0x3) << 14)
#define BIT_AP_AHB_OTG_TESTCLK                       BIT(13)
#define BIT_AP_AHB_OTG_TESTDATAOUTSEL                BIT(12)
#define BIT_AP_AHB_OTG_TESTADDR(x)                   (((x) & 0xF) << 8)
#define BIT_AP_AHB_OTG_TESTDATAIN(x)                 (((x) & 0xFF))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_OTG_PHY_CTRL
** Register Offset : 0x3028
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_OTG_SS_SCALEDOWNMODE(x)           (((x) & 0x3) << 25)
#define BIT_AP_AHB_OTG_TXBITSTUFFENH                 BIT(23)
#define BIT_AP_AHB_OTG_TXBITSTUFFEN                  BIT(22)
#define BIT_AP_AHB_OTG_DMPULLDOWN                    BIT(21)
#define BIT_AP_AHB_OTG_DPPULLDOWN                    BIT(20)
#define BIT_AP_AHB_OTG_DMPULLUP                      BIT(9)
#define BIT_AP_AHB_OTG_COMMONONN                     BIT(8)
#define BIT_AP_AHB_OTG_REFCLKSEL(x)                  (((x) & 0x3) << 4)
#define BIT_AP_AHB_OTG_FSEL(x)                       (((x) & 0x7))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_HSIC_PHY_TUNE
** Register Offset : 0x302C
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_HSIC_REFCLK_DIV(x)                (((x) & 0x7F) << 24)
#define BIT_AP_AHB_HSIC_TXPREEMPPULSETUNE            BIT(20)
#define BIT_AP_AHB_HSIC_TXRESTUNE(x)                 (((x) & 0x3) << 18)
#define BIT_AP_AHB_HSIC_TXHSXVTUNE(x)                (((x) & 0x3) << 16)
#define BIT_AP_AHB_HSIC_TXVREFTUNE(x)                (((x) & 0xF) << 12)
#define BIT_AP_AHB_HSIC_TXPREEMPAMPTUNE(x)           (((x) & 0x3) << 10)
#define BIT_AP_AHB_HSIC_TXRISETUNE(x)                (((x) & 0x3) << 8)
#define BIT_AP_AHB_HSIC_TXFSLSTUNE(x)                (((x) & 0xF) << 4)
#define BIT_AP_AHB_HSIC_SQRXTUNE(x)                  (((x) & 0x7))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_HSIC_PHY_TEST
** Register Offset : 0x3030
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_HSIC_ATERESET                     BIT(31)
#define BIT_AP_AHB_HSIC_VBUS_VALID_EXT_SEL           BIT(26)
#define BIT_AP_AHB_HSIC_VBUS_VALID_EXT               BIT(25)
#define BIT_AP_AHB_HSIC_OTGDISABLE                   BIT(24)
#define BIT_AP_AHB_HSIC_TESTBURNIN                   BIT(21)
#define BIT_AP_AHB_HSIC_LOOPBACKENB                  BIT(20)
#define BIT_AP_AHB_HSIC_TESTDATAOUT(x)               (((x) & 0xF) << 16)
#define BIT_AP_AHB_HSIC_VATESTENB(x)                 (((x) & 0x3) << 14)
#define BIT_AP_AHB_HSIC_TESTCLK                      BIT(13)
#define BIT_AP_AHB_HSIC_TESTDATAOUTSEL               BIT(12)
#define BIT_AP_AHB_HSIC_TESTADDR(x)                  (((x) & 0xF) << 8)
#define BIT_AP_AHB_HSIC_TESTDATAIN(x)                (((x) & 0xFF))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_HSIC_PHY_CTRL
** Register Offset : 0x3034
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_HSIC_SS_SCALEDOWNMODE(x)          (((x) & 0x3) << 25)
#define BIT_AP_AHB_HSIC_TXBITSTUFFENH                BIT(23)
#define BIT_AP_AHB_HSIC_TXBITSTUFFEN                 BIT(22)
#define BIT_AP_AHB_HSIC_DMPULLDOWN                   BIT(21)
#define BIT_AP_AHB_HSIC_DPPULLDOWN                   BIT(20)
#define BIT_AP_AHB_HSIC_IF_MODE                      BIT(16)
#define BIT_AP_AHB_IF_SELECT_HSIC                    BIT(13)
#define BIT_AP_AHB_HSIC_DBNCE_FLTR_BYPASS            BIT(12)
#define BIT_AP_AHB_HSIC_DMPULLUP                     BIT(9)
#define BIT_AP_AHB_HSIC_COMMONONN                    BIT(8)
#define BIT_AP_AHB_HSIC_REFCLKSEL(x)                 (((x) & 0x3) << 4)
#define BIT_AP_AHB_HSIC_FSEL(x)                      (((x) & 0x7))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_ZIP_MTX_QOS_CFG
** Register Offset : 0x3038
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_ZIPMTX_S0_ARQOS(x)               (((x) & 0xF) << 20)
#define BIT_AP_AHB_ZIPMTX_S0_AWQOS(x)               (((x) & 0xF) << 16)
#define BIT_AP_AHB_ZIPDEC_ARQOS(x)                  (((x) & 0xF) << 12)
#define BIT_AP_AHB_ZIPDEC_AWQOS(x)                  (((x) & 0xF) << 8)
#define BIT_AP_AHB_ZIPENC_ARQOS(x)                  (((x) & 0xF) << 4)
#define BIT_AP_AHB_ZIPENC_AWQOS(x)                  (((x) & 0xF))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_QCHN_CTL_BIG
** Register Offset : 0x303C
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_Q_NEG_THD_BIG(x)           (((x) & 0xFFFF) << 16)
#define BIT_AP_AHB_APCPU_Q_WAIT_GAP_BIG(x)          (((x) & 0xF) << 12)
#define BIT_AP_AHB_APCPU_Q_PGEN_DUR_BIG(x)          (((x) & 0xF) << 8)
#define BIT_AP_AHB_APCPU_Q_WAIT_REQ_BIG(x)          (((x) & 0xF) << 4)
#define BIT_AP_AHB_APCPU_Q_RETN_BYP_BIG             BIT(2)
#define BIT_AP_AHB_APCPU_Q_RETN_SEL_BIG             BIT(1)
#define BIT_AP_AHB_APCPU_Q_LP_EN_BIG                BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_QCHN_STA_BIG
** Register Offset : 0x3040
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_Q_STATUS_BIG(x)           (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_QCHN_CTL_LIT
** Register Offset : 0x3044
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_Q_NEG_THD_LIT(x)          (((x) & 0xFFFF) << 16)
#define BIT_AP_AHB_APCPU_Q_WAIT_GAP_LIT(x)         (((x) & 0xF) << 12)
#define BIT_AP_AHB_APCPU_Q_PGEN_DUR_LIT(x)         (((x) & 0xF) << 8)
#define BIT_AP_AHB_APCPU_Q_WAIT_REQ_LIT(x)         (((x) & 0xF) << 4)
#define BIT_AP_AHB_APCPU_Q_RETN_BYP_LIT            BIT(2)
#define BIT_AP_AHB_APCPU_Q_RETN_SEL_LIT            BIT(1)
#define BIT_AP_AHB_APCPU_Q_LP_EN_LIT               BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_APCPU_QCHN_STA_LIT
** Register Offset : 0x3048
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_APCPU_Q_STATUS_LIT(x)          (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_GSP_SEL
** Register Offset : 0x304C
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_GSP_VER_SEL                    BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_RES_REG
** Register Offset : 0x30F0
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_RES_REG(x)                    (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
** Register Name   : REG_AP_AHB_CHIP_ID
** Register Offset : 0x30FC
** Description     :
---------------------------------------------------------------------------*/

#define BIT_AP_AHB_CHIP_ID(x)                    (((x) & 0xFFFFFFFF))


#endif

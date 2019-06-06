/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This file is dual-licensed: you can use it either under the terms
 * of the GPL or the X11 license, at your option. Note that this dual
 * licensing only applies to this file, and not this project as a
 * whole.
 *
 */

#ifndef _SC2723_GLB_H
#define _SC2723_GLB_H

#define CTL_BASE_ANA_GLB 0x0800

#define ANA_REG_GLB_ARM_MODULE_EN        (CTL_BASE_ANA_GLB + 0x0000)
#define ANA_REG_GLB_ARM_CLK_EN           (CTL_BASE_ANA_GLB + 0x0004)
#define ANA_REG_GLB_RTC_CLK_EN           (CTL_BASE_ANA_GLB + 0x0008)
#define ANA_REG_GLB_ARM_RST              (CTL_BASE_ANA_GLB + 0x000C)
#define ANA_REG_GLB_LDO_DCDC_PD          (CTL_BASE_ANA_GLB + 0x0010)
#define ANA_REG_GLB_LDO_PD_CTRL          (CTL_BASE_ANA_GLB + 0x0014)
#define ANA_REG_GLB_LDO_V_CTRL0          (CTL_BASE_ANA_GLB + 0x0018)
#define ANA_REG_GLB_LDO_V_CTRL1          (CTL_BASE_ANA_GLB + 0x001C)
#define ANA_REG_GLB_LDO_V_CTRL2          (CTL_BASE_ANA_GLB + 0x0020)
#define ANA_REG_GLB_LDO_V_CTRL3          (CTL_BASE_ANA_GLB + 0x0024)
#define ANA_REG_GLB_LDO_V_CTRL4          (CTL_BASE_ANA_GLB + 0x0028)
#define ANA_REG_GLB_LDO_V_CTRL5          (CTL_BASE_ANA_GLB + 0x002C)
#define ANA_REG_GLB_LDO_V_CTRL6          (CTL_BASE_ANA_GLB + 0x0030)
#define ANA_REG_GLB_LDO_V_CTRL7          (CTL_BASE_ANA_GLB + 0x0034)
#define ANA_REG_GLB_LDO_V_CTRL8          (CTL_BASE_ANA_GLB + 0x0038)
#define ANA_REG_GLB_LDO_V_CTRL9          (CTL_BASE_ANA_GLB + 0x003C)
#define ANA_REG_GLB_LDO_V_CTRL10         (CTL_BASE_ANA_GLB + 0x0040)
#define ANA_REG_GLB_LDO_V_CTRL11         (CTL_BASE_ANA_GLB + 0x0044)
#define ANA_REG_GLB_LDO_LP_CTRL          (CTL_BASE_ANA_GLB + 0x0048)
#define ANA_REG_GLB_DCDC_CTRL0           (CTL_BASE_ANA_GLB + 0x004C)
#define ANA_REG_GLB_DCDC_CTRL1           (CTL_BASE_ANA_GLB + 0x0050)
#define ANA_REG_GLB_DCDC_CTRL2           (CTL_BASE_ANA_GLB + 0x0054)
#define ANA_REG_GLB_DCDC_CTRL3           (CTL_BASE_ANA_GLB + 0x0058)
#define ANA_REG_GLB_DCDC_CTRL4           (CTL_BASE_ANA_GLB + 0x005C)
#define ANA_REG_GLB_DCDC_CTRL5           (CTL_BASE_ANA_GLB + 0x0060)
#define ANA_REG_GLB_DCDC_CTRL6           (CTL_BASE_ANA_GLB + 0x0064)
#define ANA_REG_GLB_DCDC_CTRL7           (CTL_BASE_ANA_GLB + 0x0068)
#define ANA_REG_GLB_DCDC_CTRL8           (CTL_BASE_ANA_GLB + 0x006C)
#define ANA_REG_GLB_DCDC_CTRL9           (CTL_BASE_ANA_GLB + 0x0070)
#define ANA_REG_GLB_DCDC_CTRL10          (CTL_BASE_ANA_GLB + 0x0074)
#define ANA_REG_GLB_DCDC_CTRL11          (CTL_BASE_ANA_GLB + 0x0078)
#define ANA_REG_GLB_DCDC_CTRL12          (CTL_BASE_ANA_GLB + 0x007C)
#define ANA_REG_GLB_DCDC_CTRL13          (CTL_BASE_ANA_GLB + 0x0080)
#define ANA_REG_GLB_DCDC_CTRL14          (CTL_BASE_ANA_GLB + 0x0084)
#define ANA_REG_GLB_DCDC_CTRL15          (CTL_BASE_ANA_GLB + 0x0088)
#define ANA_REG_GLB_SLP_WAIT_DCDCARM     (CTL_BASE_ANA_GLB + 0x008C)
#define ANA_REG_GLB_PWR_SLP_CTRL0        (CTL_BASE_ANA_GLB + 0x0090)
#define ANA_REG_GLB_PWR_SLP_CTRL1        (CTL_BASE_ANA_GLB + 0x0094)
#define ANA_REG_GLB_PWR_SLP_CTRL2        (CTL_BASE_ANA_GLB + 0x0098)
#define ANA_REG_GLB_PWR_SLP_CTRL3        (CTL_BASE_ANA_GLB + 0x009C)
#define ANA_REG_GLB_PWR_SLP_CTRL4        (CTL_BASE_ANA_GLB + 0x00A0)
#define ANA_REG_GLB_AUD_SLP_CTRL         (CTL_BASE_ANA_GLB + 0x00A4)
#define ANA_REG_GLB_DCDC_SLP_CTRL0       (CTL_BASE_ANA_GLB + 0x00A8)
#define ANA_REG_GLB_DCDC_SLP_CTRL1       (CTL_BASE_ANA_GLB + 0x00AC)
#define ANA_REG_GLB_DCDC_SLP_CTRL2       (CTL_BASE_ANA_GLB + 0x00B0)
#define ANA_REG_GLB_DCDC_SLP_CTRL3       (CTL_BASE_ANA_GLB + 0x00B4)
#define ANA_REG_GLB_DCDC_SLP_CTRL4       (CTL_BASE_ANA_GLB + 0x00B8)
#define ANA_REG_GLB_DCDC_SLP_CTRL5       (CTL_BASE_ANA_GLB + 0x00BC)
#define ANA_REG_GLB_PWR_SEL              (CTL_BASE_ANA_GLB + 0x00C0)
#define ANA_REG_GLB_PWR_XTL_EN0          (CTL_BASE_ANA_GLB + 0x00C4)
#define ANA_REG_GLB_PWR_XTL_EN1          (CTL_BASE_ANA_GLB + 0x00C8)
#define ANA_REG_GLB_PWR_XTL_EN2          (CTL_BASE_ANA_GLB + 0x00CC)
#define ANA_REG_GLB_PWR_XTL_EN3          (CTL_BASE_ANA_GLB + 0x00D0)
#define ANA_REG_GLB_32KLESS_CTRL0        (CTL_BASE_ANA_GLB + 0x00D4)
#define ANA_REG_GLB_32KLESS_CTRL1        (CTL_BASE_ANA_GLB + 0x00D8)
#define ANA_REG_GLB_32KLESS_CTRL2        (CTL_BASE_ANA_GLB + 0x00DC)
#define ANA_REG_GLB_32KLESS_CTRL3        (CTL_BASE_ANA_GLB + 0x00E0)
#define ANA_REG_GLB_AUXAD_CTL            (CTL_BASE_ANA_GLB + 0x00E4)
#define ANA_REG_GLB_XTL_WAIT_CTRL        (CTL_BASE_ANA_GLB + 0x00E8)
#define ANA_REG_GLB_RGB_CTRL             (CTL_BASE_ANA_GLB + 0x00EC)
#define ANA_REG_GLB_WHTLED_CTRL          (CTL_BASE_ANA_GLB + 0x00F0)
#define ANA_REG_GLB_KPLED_CTRL           (CTL_BASE_ANA_GLB + 0x00F4)
#define ANA_REG_GLB_VIBR_CTRL0           (CTL_BASE_ANA_GLB + 0x00F8)
#define ANA_REG_GLB_AUDIO_CTRL0          (CTL_BASE_ANA_GLB + 0x00FC)
#define ANA_REG_GLB_AUDIO_CTRL1          (CTL_BASE_ANA_GLB + 0x0100)
#define ANA_REG_GLB_CHGR_CTRL0           (CTL_BASE_ANA_GLB + 0x0104)
#define ANA_REG_GLB_CHGR_CTRL1           (CTL_BASE_ANA_GLB + 0x0108)
#define ANA_REG_GLB_CHGR_CTRL2           (CTL_BASE_ANA_GLB + 0x010C)
#define ANA_REG_GLB_CHGR_DET_FGU_CTRL    (CTL_BASE_ANA_GLB + 0x0110)
#define ANA_REG_GLB_CHGR_STATUS          (CTL_BASE_ANA_GLB + 0x0114)
#define ANA_REG_GLB_MIXED_CTRL0          (CTL_BASE_ANA_GLB + 0x0118)
#define ANA_REG_GLB_MIXED_CTRL1          (CTL_BASE_ANA_GLB + 0x011C)
#define ANA_REG_GLB_SWRST_CTRL           (CTL_BASE_ANA_GLB + 0x0120)
#define ANA_REG_GLB_POR_RST_MONITOR      (CTL_BASE_ANA_GLB + 0x0124)
#define ANA_REG_GLB_WDG_RST_MONITOR      (CTL_BASE_ANA_GLB + 0x0128)
#define ANA_REG_GLB_POR_PIN_RST_MONITOR  (CTL_BASE_ANA_GLB + 0x012C)
#define ANA_REG_GLB_POR_SRC_FLAG         (CTL_BASE_ANA_GLB + 0x0130)
#define ANA_REG_GLB_POR_7S_CTRL          (CTL_BASE_ANA_GLB + 0x0134)
#define ANA_REG_GLB_HWRST_RTC            (CTL_BASE_ANA_GLB + 0x0138)
#define ANA_REG_GLB_CHIP_ID_LOW          (CTL_BASE_ANA_GLB + 0x013C)
#define ANA_REG_GLB_CHIP_ID_HIGH         (CTL_BASE_ANA_GLB + 0x0140)
#define ANA_REG_GLB_ARM_MF_REG           (CTL_BASE_ANA_GLB + 0x0144)
#define ANA_REG_GLB_ARCH_EN              (CTL_BASE_ANA_GLB + 0x0148)
#define ANA_REG_GLB_MCU_WR_PROT_VALUE    (CTL_BASE_ANA_GLB + 0x014C)
#define ANA_REG_GLB_PWR_WR_PROT_VALUE    (CTL_BASE_ANA_GLB + 0x0150)
#define ANA_REG_GLB_SMPL_CTRL0           (CTL_BASE_ANA_GLB + 0x0154)
#define ANA_REG_GLB_SMPL_CTRL1           (CTL_BASE_ANA_GLB + 0x0158)
#define ANA_REG_GLB_RTC_RST0             (CTL_BASE_ANA_GLB + 0x015C)
#define ANA_REG_GLB_RTC_RST1             (CTL_BASE_ANA_GLB + 0x0160)
#define ANA_REG_GLB_RTC_RST2             (CTL_BASE_ANA_GLB + 0x0164)
#define ANA_REG_GLB_LDO_SHPT_PD1         (CTL_BASE_ANA_GLB + 0x0188)
#define ANA_REG_GLB_LDO_SHPT_PD2         (CTL_BASE_ANA_GLB + 0x018C)
#define ANA_REG_GLB_BATDET_CUR_CTRL      (CTL_BASE_ANA_GLB + 0x0190)
#define ANA_REG_GLB_RTC_CLK_STOP         (CTL_BASE_ANA_GLB + 0x0194)
#define ANA_REG_GLB_VBAT_DROP_CNT        (CTL_BASE_ANA_GLB + 0x0198)
#define ANA_REG_GLB_DCDC_DISCHRG         (CTL_BASE_ANA_GLB + 0x019C)
#define ANA_REG_GLB_DCDC_CORE_ADI        (CTL_BASE_ANA_GLB + 0x0200)
#define ANA_REG_GLB_DCDC_ARM_ADI         (CTL_BASE_ANA_GLB + 0x0204)
#define ANA_REG_GLB_DCDC_MEM_ADI         (CTL_BASE_ANA_GLB + 0x0208)
#define ANA_REG_GLB_DCDC_GEN_ADI         (CTL_BASE_ANA_GLB + 0x020C)
#define ANA_REG_GLB_DCDC_WPA_ADI         (CTL_BASE_ANA_GLB + 0x0210)
#define ANA_REG_GLB_DCDC_WPA_DCM_ADI     (CTL_BASE_ANA_GLB + 0x0214)
#define ANA_REG_GLB_DCDC_CON_ADI         (CTL_BASE_ANA_GLB + 0x0218)
#define ANA_REG_GLB_DCDC_RF_ADI          (CTL_BASE_ANA_GLB + 0x021C)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_ARM_MODULE_EN
* Register Offset : 0x0000
* Description     :
---------------------------------------------------------------------------*/

#define BIT_ANA_IMPD_ADC_EN              BIT(12)
#define BIT_ANA_THM_EN                   BIT(11)
#define BIT_ANA_BLTC_EN                  BIT(10)
#define BIT_ANA_PINREG_EN                BIT(9)
#define BIT_ANA_FGU_EN                   BIT(8)
#define BIT_ANA_EFS_EN                   BIT(7)
#define BIT_ANA_ADC_EN                   BIT(6)
#define BIT_ANA_HDT_EN                   BIT(5)
#define BIT_ANA_AUD_EN                   BIT(4)
#define BIT_ANA_EIC_EN                   BIT(3)
#define BIT_ANA_WDG_EN                   BIT(2)
#define BIT_ANA_RTC_EN                   BIT(1)
#define BIT_ANA_CAL_EN                   BIT(0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_ARM_CLK_EN
* Register Offset : 0x0004
* Description     :
---------------------------------------------------------------------------*/

#define BIT_CLK_IMPD_ADC_EN              BIT(10)
#define BIT_CLK_AUXAD_EN                 BIT(9)
#define BIT_CLK_AUXADC_EN                BIT(8)
#define BITS_CLK_CAL_SRC_SEL(x)          (((x) & 0x3) << 6)
#define BIT_CLK_CAL_EN                   BIT(5)
#define BIT_CLK_AUD_HID_EN               BIT(4)
#define BIT_CLK_AUD_HBD_EN               BIT(3)
#define BIT_CLK_AUD_LOOP_EN              BIT(2)
#define BIT_CLK_AUD_6P5M_EN              BIT(1)
#define BIT_CLK_AUDIF_EN                 BIT(0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_RTC_CLK_EN
* Register Offset : 0x0008
* Description     :
---------------------------------------------------------------------------*/

#define BIT_RTC_EFS_EN                   BIT(12)
#define BIT_RTC_THMA_AUTO_EN             BIT(11)
#define BIT_RTC_THMA_EN                  BIT(10)
#define BIT_RTC_THM_EN                   BIT(9)
#define BIT_RTC_BLTC_EN                  BIT(8)
#define BIT_RTC_FGU_EN                   BIT(7)
#define BIT_RTC_FGUA_EN                  BIT(6)
#define BIT_RTC_VIBR_EN                  BIT(5)
#define BIT_RTC_AUD_EN                   BIT(4)
#define BIT_RTC_EIC_EN                   BIT(3)
#define BIT_RTC_WDG_EN                   BIT(2)
#define BIT_RTC_RTC_EN                   BIT(1)
#define BIT_RTC_ARCH_EN                  BIT(0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_ARM_RST
* Register Offset : 0x000C
* Description     :
---------------------------------------------------------------------------*/

#define BIT_ANA_THMA_SOFT_RST            BIT(15)
#define BIT_ANA_THM_SOFT_RST             BIT(14)
#define BIT_ANA_BLTC_SOFT_RST            BIT(13)
#define BIT_ANA_AUD_32K_SOFT_RST         BIT(12)
#define BIT_ANA_AUDTX_SOFT_RST           BIT(11)
#define BIT_ANA_AUDRX_SOFT_RST           BIT(10)
#define BIT_ANA_AUD_SOFT_RST             BIT(9)
#define BIT_ANA_AUD_HDT_SOFT_RST         BIT(8)
#define BIT_ANA_EFS_SOFT_RST             BIT(7)
#define BIT_ANA_ADC_SOFT_RST             BIT(6)
#define BIT_ANA_PWM0_SOFT_RST            BIT(5)
#define BIT_ANA_FGU_SOFT_RST             BIT(4)
#define BIT_ANA_EIC_SOFT_RST             BIT(3)
#define BIT_ANA_WDG_SOFT_RST             BIT(2)
#define BIT_ANA_RTC_SOFT_RST             BIT(1)
#define BIT_ANA_CAL_SOFT_RST             BIT(0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_LDO_DCDC_PD
* Register Offset : 0x0010
* Description     :
---------------------------------------------------------------------------*/

#define BIT_LDO_EMM_PD                   BIT(15)
#define BIT_DCDC_TOPCLK6M_PD             BIT(14)
#define BIT_DCDC_RF_PD                   BIT(13)
#define BIT_DCDC_GEN_PD                  BIT(12)
#define BIT_DCDC_MEM_PD                  BIT(11)
#define BIT_DCDC_ARM_PD                  BIT(10)
#define BIT_DCDC_CORE_PD                 BIT(9)
#define BIT_LDO_RF0_PD                   BIT(8)
#define BIT_LDO_EMMCCORE_PD              BIT(7)
#define BIT_LDO_GEN1_PD                  BIT(6)
#define BIT_LDO_DCXO_PD                  BIT(5)
#define BIT_LDO_GEN0_PD                  BIT(4)
#define BIT_LDO_VDD25_PD                 BIT(3)
#define BIT_LDO_VDD28_PD                 BIT(2)
#define BIT_LDO_VDD18_PD                 BIT(1)
#define BIT_BG_PD                        BIT(0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_LDO_PD_CTRL
* Register Offset : 0x0014
* Description     :
---------------------------------------------------------------------------*/

#define BIT_LDO_LPREF_PD_SW              BIT(15)
#define BIT_DCDC_WPA_PD                  BIT(14)
#define BIT_DCDC_CON_PD                  BIT(13)
#define BIT_LDO_WIFIPA_PD                BIT(11)
#define BIT_LDO_SDCORE_PD                BIT(10)
#define BIT_LDO_USB_PD                   BIT(8)
#define BIT_LDO_CAMMOT_PD                BIT(7)
#define BIT_LDO_CAMIO_PD                 BIT(6)
#define BIT_LDO_CAMD_PD                  BIT(5)
#define BIT_LDO_CAMA_PD                  BIT(4)
#define BIT_LDO_SIM2_PD                  BIT(3)
#define BIT_LDO_SIM1_PD                  BIT(2)
#define BIT_LDO_SIM0_PD                  BIT(1)
#define BIT_LDO_SDIO_PD                  BIT(0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_LDO_V_CTRL0
* Register Offset : 0x0018
* Description     :
---------------------------------------------------------------------------*/

#define BITS_LDO_RF0_V(x)                (((x) & 0xFF) << 8)
#define BITS_LDO_WIFIPA_V(x)             (((x) & 0xFF) << 0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_LDO_V_CTRL1
* Register Offset : 0x001C
* Description     :
---------------------------------------------------------------------------*/

#define BITS_LDO_CAMIO_V(x)              (((x) & 0x7F) << 8)
#define BITS_LDO_CAMD_V(x)               (((x) & 0x7F) << 0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_LDO_V_CTRL2
* Register Offset : 0x0020
* Description     :
---------------------------------------------------------------------------*/

#define BITS_LDO_GEN0_V(x)               (((x) & 0x7F) << 0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_LDO_V_CTRL3
* Register Offset : 0x0024
* Description     :
---------------------------------------------------------------------------*/

#define BITS_LDO_GEN1_V(x)               (((x) & 0x7F) << 8)
#define BITS_LDO_VDD28_V(x)              (((x) & 0xFF) << 0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_LDO_V_CTRL4
* Register Offset : 0x0028
* Description     :
---------------------------------------------------------------------------*/

#define BITS_LDO_SIM0_V(x)               (((x) & 0xFF) << 8)
#define BITS_LDO_SDIO_V(x)               (((x) & 0xFF) << 0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_LDO_V_CTRL5
* Register Offset : 0x002C
* Description     :
---------------------------------------------------------------------------*/

#define BITS_LDO_SIM2_V(x)               (((x) & 0xFF) << 8)
#define BITS_LDO_SIM1_V(x)               (((x) & 0xFF) << 0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_LDO_V_CTRL6
* Register Offset : 0x0030
* Description     :
---------------------------------------------------------------------------*/

#define BITS_LDO_SDCORE_V(x)             (((x) & 0xFF) << 8)
#define BITS_LDO_CAMA_V(x)               (((x) & 0xFF) << 0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_LDO_V_CTRL7
* Register Offset : 0x0034
* Description     :
---------------------------------------------------------------------------*/

#define BITS_LDO_EMMCCORE_V(x)           (((x) & 0xFF) << 8)
#define BITS_LDO_CAMMOT_V(x)             (((x) & 0xFF) << 0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_LDO_V_CTRL8
* Register Offset : 0x0038
* Description     :
---------------------------------------------------------------------------*/

#define BITS_LDO_DCXO_V(x)               (((x) & 0xFF) << 8)
#define BITS_LDO_DCXO_LP_V(x)            (((x) & 0xFF) << 0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_LDO_V_CTRL9
* Register Offset : 0x003C
* Description     :
---------------------------------------------------------------------------*/

#define BITS_LDO_VDD18_V(x)              (((x) & 0x7F) << 8)
#define BITS_LDO_USB_V(x)                (((x) & 0xFF) << 0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_LDO_V_CTRL10
* Register Offset : 0x0040
* Description     :
---------------------------------------------------------------------------*/

#define BITS_LDO_VDD25_V(x)              (((x) & 0xFF) << 0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_LDO_V_CTRL11
* Register Offset : 0x0044
* Description     :
---------------------------------------------------------------------------*/

#define BITS_VBATBK_V(x)                 (((x) & 0x3) << 12)
#define BITS_LDOB_CAL_SEL(x)             (((x) & 0x7) << 8)
#define BITS_LDOA_CAL_SEL(x)             (((x) & 0x7) << 5)
#define BITS_LDOD_CAL_SEL(x)             (((x) & 0x3) << 3)
#define BITS_LDODCDC_CAL_SEL(x)          (((x) & 0x7) << 0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_LDO_LP_CTRL
* Register Offset : 0x0048
* Description     :
---------------------------------------------------------------------------*/

#define BITS_LDOB_LP_CAL(x)              (((x) & 0x1F) << 10)
#define BITS_LDOA_LP_CAL(x)              (((x) & 0x1F) << 5)
#define BITS_LDODCDC_LP_CAL(x)           (((x) & 0x1F) << 0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_DCDC_CTRL0
* Register Offset : 0x004C
* Description     :
---------------------------------------------------------------------------*/

#define BITS_DCDC_CORE_DEADTIME(x)       (((x) & 0x3) << 14)
#define BITS_DCDC_CORE_STBOP(x)          (((x) & 0x3F) << 8)
#define BITS_DCDC_CORE_PDRSLOW(x)        (((x) & 0xF) << 4)
#define BIT_DCDC_CORE_QKRSPS             BIT(3)
#define BIT_DCDC_CORE_PFM                BIT(2)
#define BIT_DCDC_CORE_DCM                BIT(1)
#define BIT_DCDC_CORE_LP_EN              BIT(0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_DCDC_CTRL1
* Register Offset : 0x0050
* Description     :
---------------------------------------------------------------------------*/

#define BITS_DCDC_MEM_DEADTIME(x)        (((x) & 0x3) << 14)
#define BITS_DCDC_MEM_STBOP(x)           (((x) & 0x3F) << 8)
#define BITS_DCDC_MEM_PDRSLOW(x)         (((x) & 0xF) << 4)
#define BIT_DCDC_MEM_QKRSPS              BIT(3)
#define BIT_DCDC_MEM_PFM                 BIT(2)
#define BIT_DCDC_MEM_DCM                 BIT(1)
#define BIT_DCDC_MEM_LP_EN               BIT(0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_DCDC_CTRL2
* Register Offset : 0x0054
* Description     :
---------------------------------------------------------------------------*/

#define BITS_DCDC_GEN_DEADTIME(x)        (((x) & 0x3) << 14)
#define BITS_DCDC_GEN_STBOP(x)           (((x) & 0x3F) << 8)
#define BITS_DCDC_GEN_PDRSLOW(x)         (((x) & 0xF) << 4)
#define BIT_DCDC_GEN_QKRSPS              BIT(3)
#define BIT_DCDC_GEN_PFM                 BIT(2)
#define BIT_DCDC_GEN_DCM                 BIT(1)
#define BIT_DCDC_GEN_LP_EN               BIT(0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_DCDC_CTRL3
* Register Offset : 0x0058
* Description     :
---------------------------------------------------------------------------*/

#define BITS_DCDC_ARM_DEADTIME(x)        (((x) & 0x3) << 14)
#define BITS_DCDC_ARM_STBOP(x)           (((x) & 0x3F) << 8)
#define BITS_DCDC_ARM_PDRSLOW(x)         (((x) & 0xF) << 4)
#define BIT_DCDC_ARM_QKRSPS              BIT(3)
#define BIT_DCDC_ARM_PFM                 BIT(2)
#define BIT_DCDC_ARM_DCM                 BIT(1)
#define BIT_DCDC_ARM_LP_EN               BIT(0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_DCDC_CTRL4
* Register Offset : 0x005C
* Description     :
---------------------------------------------------------------------------*/

#define BITS_DCDC_WPA_DEADTIME(x)        (((x) & 0x3) << 14)
#define BITS_DCDC_WPA_STBOP(x)           (((x) & 0x3F) << 8)
#define BITS_DCDC_WPA_PDRSLOW(x)         (((x) & 0xF) << 4)
#define BIT_DCDC_WPA_QKRSPS              BIT(3)
#define BIT_DCDC_WPA_PFM                 BIT(2)
#define BIT_DCDC_WPA_LP_EN               BIT(0)

/*---------------------------------------------------------------------------
* Register Name   : ANA_REG_GLB_DCDC_CTRL5
* Register Offset : 0x0060
* Description     :
---------------------------------------------------------------------------*/

#define BITS_DCDC_RF_DEADTIME(x)         (((x) & 0x3) << 14)
#define BITS_DCDC_RF_STBOP(x)            (((x) & 0x3F) << 8)
#define BITS_DCDC_RF_PDRSLOW(x)          (((x) & 0xF) << 4)
#define BIT_DCDC_RF_QKRSPS               BIT(3)
#define BIT_DCDC_RF_PFM                  BIT(2)
#define BIT_DCDC_RF_DCM                  BIT(1)
#define BIT_DCDC_RF_LP_EN                BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_CTRL6
//Register Offset        :    0x0064
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_DCDC_CON_DEADTIME(x)        (((x) & 0x3) << 14)
#define BITS_DCDC_CON_STBOP(x)           (((x) & 0x3F) << 8)
#define BITS_DCDC_CON_PDRSLOW(x)         (((x) & 0xF) << 4)
#define BIT_DCDC_CON_QKRSPS              BIT(3)
#define BIT_DCDC_CON_PFM                 BIT(2)
#define BIT_DCDC_CON_DCM                 BIT(1)
#define BIT_DCDC_CON_LP_EN               BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_CTRL7
//Register Offset        :    0x0068
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_DCDC_MEM_CF(x)              (((x) & 0x3) << 12)
#define BITS_DCDC_MEM_PFMAD(x)           (((x) & 0x3) << 10)
#define BITS_DCDC_MEM_CL_CTRL(x)         (((x) & 0x3) << 8)
#define BIT_DCDC_CORE_DUALSEL            BIT(7)
#define BIT_DCDC_CORE_MERGEEN            BIT(6)
#define BITS_DCDC_CORE_CF(x)             (((x) & 0x3) << 4)
#define BITS_DCDC_CORE_PFMAD(x)          (((x) & 0x3) << 2)
#define BITS_DCDC_CORE_CL_CTRL(x)        (((x) & 0x3) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_CTRL8
//Register Offset        :    0x006C
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_DCDC_ARM_DUALSEL             BIT(15)
#define BIT_DCDC_ARM_MERGEEN             BIT(14)
#define BITS_DCDC_ARM_CF(x)              (((x) & 0x3) << 12)
#define BITS_DCDC_ARM_PFMAD(x)           (((x) & 0x3) << 10)
#define BITS_DCDC_ARM_CL_CTRL(x)         (((x) & 0x3) << 8)
#define BITS_DCDC_GEN_CF(x)              (((x) & 0x3) << 4)
#define BITS_DCDC_GEN_PFMAD(x)           (((x) & 0x3) << 2)
#define BITS_DCDC_GEN_CL_CTRL(x)         (((x) & 0x3) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_CTRL9
//Register Offset        :    0x0070
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_DCDC_CON_MERGEEN             BIT(14)
#define BITS_DCDC_CON_CF(x)              (((x) & 0x3) << 12)
#define BITS_DCDC_CON_PFMAD(x)           (((x) & 0x3) << 10)
#define BITS_DCDC_CON_CL_CTRL(x)         (((x) & 0x3) << 8)
#define BIT_DCDC_RF_MERGEEN              BIT(6)
#define BITS_DCDC_RF_CF(x)               (((x) & 0x3) << 4)
#define BITS_DCDC_RF_PFMAD(x)            (((x) & 0x3) << 2)
#define BITS_DCDC_RF_CL_CTRL(x)          (((x) & 0x3) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_CTRL10
//Register Offset        :    0x0074
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_DCDC_WPA_BPOUT_SOFTW         BIT(15)
#define BITS_DCDC_WPA_VBAT_DIV(x)        (((x) & 0x7) << 12)
#define BIT_DCDC_WPA_BPEN                BIT(11)
#define BIT_DCDC_WPA_BPMODE              BIT(10)
#define BIT_DCDC_WPA_DEGEN               BIT(9)
#define BIT_DCDC_WPA_APTEN               BIT(8)
#define BITS_DCDC_WPA_DEBC_SEL(x)        (((x) & 0x3) << 6)
#define BITS_DCDC_WPA_CF(x)              (((x) & 0x3) << 4)
#define BITS_DCDC_WPA_PFMAD(x)           (((x) & 0x3) << 2)
#define BITS_DCDC_WPA_CL_CTRL(x)         (((x) & 0x3) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_CTRL11
//Register Offset        :    0x0078
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_DCDC_3MCLKCAL_EN             BIT(13)
#define BIT_DCDC_2MCLKCAL_EN             BIT(12)
#define BITS_DCDC_6MFRECAL_SW(x)         (((x) & 0x1F) << 7)
#define BITS_DCDC_4MFRECAL_SW(x)         (((x) & 0x1F) << 2)
#define BIT_DCDC_CLK_SP_SEL              BIT(1)
#define BIT_DCDC_CLK_SP_EN               BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_CTRL12
//Register Offset        :    0x007C
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_DCDC_CORE_OSCSYCEN_SW        BIT(15)
#define BIT_DCDC_CORE_OSCSYCEN_HW_EN     BIT(14)
#define BIT_DCDC_CORE_OSCSYC_DIV_EN      BIT(13)
#define BITS_DCDC_CORE_OSCSYC_DIV(x)     (((x) & 0x1F) << 8)
#define BIT_DCDC_ARM_OSCSYCEN_SW         BIT(7)
#define BIT_DCDC_ARM_OSCSYCEN_HW_EN      BIT(6)
#define BIT_DCDC_ARM_OSCSYC_DIV_EN       BIT(5)
#define BITS_DCDC_ARM_OSCSYC_DIV(x)      (((x) & 0x1F) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_CTRL13
//Register Offset        :    0x0080
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_DCDC_MEM_OSCSYCEN_SW         BIT(15)
#define BIT_DCDC_MEM_OSCSYCEN_HW_EN      BIT(14)
#define BIT_DCDC_MEM_OSCSYC_DIV_EN       BIT(13)
#define BITS_DCDC_MEM_OSCSYC_DIV(x)      (((x) & 0x1F) << 8)
#define BIT_DCDC_GEN_OSCSYCEN_SW         BIT(7)
#define BIT_DCDC_GEN_OSCSYCEN_HW_EN      BIT(6)
#define BIT_DCDC_GEN_OSCSYC_DIV_EN       BIT(5)
#define BITS_DCDC_GEN_OSCSYC_DIV(x)      (((x) & 0x1F) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_CTRL14
//Register Offset        :    0x0084
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_DCDC_WPA_OSCSYCEN_SW         BIT(15)
#define BIT_DCDC_WPA_OSCSYCEN_HW_EN      BIT(14)
#define BIT_DCDC_WPA_OSCSYC_DIV_EN       BIT(13)
#define BITS_DCDC_WPA_OSCSYC_DIV(x)      (((x) & 0x1F) << 8)
#define BIT_DCDC_RF_OSCSYCEN_SW          BIT(7)
#define BIT_DCDC_RF_OSCSYCEN_HW_EN       BIT(6)
#define BIT_DCDC_RF_OSCSYC_DIV_EN        BIT(5)
#define BITS_DCDC_RF_OSCSYC_DIV(x)       (((x) & 0x1F) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_CTRL15
//Register Offset        :    0x0088
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_DCDC_CAL_SEL(x)             (((x) & 0x7) << 13)
#define BIT_DCDC_CON_OSCSYCEN_SW         BIT(7)
#define BIT_DCDC_CON_OSCSYCEN_HW_EN      BIT(6)
#define BIT_DCDC_CON_OSCSYC_DIV_EN       BIT(5)
#define BITS_DCDC_CON_OSCSYC_DIV(x)      (((x) & 0x1F) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_SLP_WAIT_DCDCARM
//Register Offset        :    0x008C
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_SLP_IN_WAIT_DCDCARM(x)      (((x) & 0xFF) << 8)
#define BITS_SLP_OUT_WAIT_DCDCARM(x)     (((x) & 0xFF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_PWR_SLP_CTRL0
//Register Offset        :    0x0090
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_SLP_IO_EN                    BIT(15)
#define BIT_SLP_DCDCRF_PD_EN             BIT(13)
#define BIT_SLP_DCDCCON_PD_EN            BIT(12)
#define BIT_SLP_DCDCGEN_PD_EN            BIT(11)
#define BIT_SLP_DCDCWPA_PD_EN            BIT(10)
#define BIT_SLP_DCDCARM_PD_EN            BIT(9)
#define BIT_SLP_LDOVDD25_PD_EN           BIT(8)
#define BIT_SLP_LDORF0_PD_EN             BIT(7)
#define BIT_SLP_LDOEMMCCORE_PD_EN        BIT(6)
#define BIT_SLP_LDOGEN0_PD_EN            BIT(5)
#define BIT_SLP_LDODCXO_PD_EN            BIT(4)
#define BIT_SLP_LDOGEN1_PD_EN            BIT(3)
#define BIT_SLP_LDOWIFIPA_PD_EN          BIT(2)
#define BIT_SLP_LDOVDD28_PD_EN           BIT(1)
#define BIT_SLP_LDOVDD18_PD_EN           BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_PWR_SLP_CTRL1
//Register Offset        :    0x0094
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_SLP_LDO_PD_EN                BIT(15)
#define BIT_SLP_LDOLPREF_PD_EN           BIT(14)
#define BIT_SLP_LDOSDCORE_PD_EN          BIT(9)
#define BIT_SLP_LDOUSB_PD_EN             BIT(8)
#define BIT_SLP_LDOCAMMOT_PD_EN          BIT(7)
#define BIT_SLP_LDOCAMIO_PD_EN           BIT(6)
#define BIT_SLP_LDOCAMD_PD_EN            BIT(5)
#define BIT_SLP_LDOCAMA_PD_EN            BIT(4)
#define BIT_SLP_LDOSIM2_PD_EN            BIT(3)
#define BIT_SLP_LDOSIM1_PD_EN            BIT(2)
#define BIT_SLP_LDOSIM0_PD_EN            BIT(1)
#define BIT_SLP_LDOSDIO_PD_EN            BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_PWR_SLP_CTRL2
//Register Offset        :    0x0098
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_SLP_DCDCRF_LP_EN             BIT(14)
#define BIT_SLP_DCDCCON_LP_EN            BIT(13)
#define BIT_SLP_DCDCCORE_LP_EN           BIT(12)
#define BIT_SLP_DCDCMEM_LP_EN            BIT(11)
#define BIT_SLP_DCDCARM_LP_EN            BIT(10)
#define BIT_SLP_DCDCGEN_LP_EN            BIT(9)
#define BIT_SLP_DCDCWPA_LP_EN            BIT(8)
#define BIT_SLP_LDORF0_LP_EN             BIT(7)
#define BIT_SLP_LDOEMMCCORE_LP_EN        BIT(6)
#define BIT_SLP_LDOGEN0_LP_EN            BIT(5)
#define BIT_SLP_LDODCXO_LP_EN            BIT(4)
#define BIT_SLP_LDOGEN1_LP_EN            BIT(3)
#define BIT_SLP_LDOWIFIPA_LP_EN          BIT(2)
#define BIT_SLP_LDOVDD28_LP_EN           BIT(1)
#define BIT_SLP_LDOVDD18_LP_EN           BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_PWR_SLP_CTRL3
//Register Offset        :    0x009C
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_SLP_BG_LP_EN                 BIT(15)
#define BIT_LDOVDD25_LP_EN_SW            BIT(14)
#define BIT_LDOSDCORE_LP_EN_SW           BIT(13)
#define BIT_LDOUSB_LP_EN_SW              BIT(12)
#define BIT_SLP_LDOVDD25_LP_EN           BIT(10)
#define BIT_SLP_LDOSDCORE_LP_EN          BIT(9)
#define BIT_SLP_LDOUSB_LP_EN             BIT(8)
#define BIT_SLP_LDOCAMMOT_LP_EN          BIT(7)
#define BIT_SLP_LDOCAMIO_LP_EN           BIT(6)
#define BIT_SLP_LDOCAMD_LP_EN            BIT(5)
#define BIT_SLP_LDOCAMA_LP_EN            BIT(4)
#define BIT_SLP_LDOSIM2_LP_EN            BIT(3)
#define BIT_SLP_LDOSIM1_LP_EN            BIT(2)
#define BIT_SLP_LDOSIM0_LP_EN            BIT(1)
#define BIT_SLP_LDOSDIO_LP_EN            BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_PWR_SLP_CTRL4
//Register Offset        :    0x00A0
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_LDOCAMIO_LP_EN_SW            BIT(15)
#define BIT_LDOCAMMOT_LP_EN_SW           BIT(14)
#define BIT_LDOCAMD_LP_EN_SW             BIT(13)
#define BIT_LDOCAMA_LP_EN_SW             BIT(12)
#define BIT_LDOSIM2_LP_EN_SW             BIT(11)
#define BIT_LDOSIM1_LP_EN_SW             BIT(10)
#define BIT_LDOSIM0_LP_EN_SW             BIT(9)
#define BIT_LDOSDIO_LP_EN_SW             BIT(8)
#define BIT_LDORF0_LP_EN_SW              BIT(7)
#define BIT_LDOEMMCCORE_LP_EN_SW         BIT(6)
#define BIT_LDOGEN0_LP_EN_SW             BIT(5)
#define BIT_LDODCXO_LP_EN_SW             BIT(4)
#define BIT_LDOGEN1_LP_EN_SW             BIT(3)
#define BIT_LDOWIFIPA_LP_EN_SW           BIT(2)
#define BIT_LDOVDD28_LP_EN_SW            BIT(1)
#define BIT_LDOVDD18_LP_EN_SW            BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_AUD_SLP_CTRL
//Register Offset        :    0x00A4
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_SLP_AUD_PMUR1_PD_EN          BIT(14)
#define BIT_SLP_AUD_PA_SW_PD_EN          BIT(13)
#define BIT_SLP_AUD_PA_LDO_PD_EN         BIT(12)
#define BIT_SLP_AUD_PA_PD_EN             BIT(11)
#define BIT_SLP_AUD_OVP_PD_PD_EN         BIT(10)
#define BIT_SLP_AUD_OVP_LDO_PD_EN        BIT(9)
#define BIT_SLP_AUD_LDOCG_PD_PD_EN       BIT(8)
#define BIT_SLP_AUD_VB_PD_EN             BIT(7)
#define BIT_SLP_AUD_VBO_PD_EN            BIT(6)
#define BIT_SLP_AUD_HEADMICBIAS_PD_EN    BIT(5)
#define BIT_SLP_AUD_HEADMIC_SLEEP_PD_EN  BIT(4)
#define BIT_SLP_AUD_PLGPD_PD_EN          BIT(3)
#define BIT_SLP_AUD_VB_NLEAK_PD          BIT(2)
#define BITS_SLP_AUD_PMUR0_PD_EN(x)      (((x) & 0x3) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_SLP_CTRL0
//Register Offset        :    0x00A8
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_SLP_DCDCCORE_VOL_DROP_CNT(x)    (((x) & 0xFF) << 8)
#define BIT_PWR_OFF_SEQ_EN                   BIT(2)
#define BIT_DCDC_CORE_SLP_OUT_STEP_EN        BIT(1)
#define BIT_DCDC_CORE_SLP_IN_STEP_EN         BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_SLP_CTRL1
//Register Offset        :    0x00AC
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_DCDC_CORE_CAL_DS_SW(x)          (((x) & 0x1F) << 5)
#define BITS_DCDC_CORE_CTRL_DS_SW(x)         (((x) & 0x1F) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_SLP_CTRL2
//Register Offset        :    0x00B0
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_DCDC_CORE_CTRL_SLP_STEP3(x)     (((x) & 0x1F) << 10)
#define BITS_DCDC_CORE_CTRL_SLP_STEP2(x)     (((x) & 0x1F) << 5)
#define BITS_DCDC_CORE_CTRL_SLP_STEP1(x)     (((x) & 0x1F) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_SLP_CTRL3
//Register Offset        :    0x00B4
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_DCDC_CORE_CTRL_SLP_STEP5(x)     (((x) & 0x1F) << 5)
#define BITS_DCDC_CORE_CTRL_SLP_STEP4(x)     (((x) & 0x1F) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_SLP_CTRL4
//Register Offset        :    0x00B8
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_DCDC_CORE_CAL_SLP_STEP3(x)      (((x) & 0x1F) << 10)
#define BITS_DCDC_CORE_CAL_SLP_STEP2(x)      (((x) & 0x1F) << 5)
#define BITS_DCDC_CORE_CAL_SLP_STEP1(x)      (((x) & 0x1F) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_SLP_CTRL5
//Register Offset        :    0x00BC
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_DCDC_CORE_CAL_SLP_STEP5(x)      (((x) & 0x1F) << 5)
#define BITS_DCDC_CORE_CAL_SLP_STEP4(x)      (((x) & 0x1F) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_PWR_SEL
//Register Offset        :    0x00C0
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_LDO_GEN0_SW_SEL                  BIT(13)
#define BIT_LDO_GEN1_SW_SEL                  BIT(12)
#define BIT_LDO_RF0_SW_SEL                   BIT(11)
#define BIT_LDO_VDD18_SW_SEL                 BIT(10)
#define BIT_LDO_VDD28_SW_SEL                 BIT(9)
#define BIT_LDO_DCXO_SW_SEL                  BIT(8)
#define BIT_LDO_EMMCCORE_SW_SEL              BIT(7)
#define BIT_LDO_VDD25_SW_SEL                 BIT(6)
#define BIT_DCDC_RF_SW_SEL                   BIT(5)
#define BIT_DCDC_GEN_SW_SEL                  BIT(4)
#define BIT_DCDC_MEM_SW_SEL                  BIT(3)
#define BIT_DCDC_ARM_SW_SEL                  BIT(2)
#define BIT_DCDC_CORE_SLP_SW_SEL             BIT(1)
#define BIT_DCDC_CORE_NOR_SW_SEL             BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_PWR_XTL_EN0
//Register Offset        :    0x00C4
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_LDO_XTL_EN                       BIT(15)
#define BIT_LDO_GEN0_EXT_XTL0_EN             BIT(14)
#define BIT_LDO_GEN0_XTL1_EN                 BIT(13)
#define BIT_LDO_GEN0_XTL0_EN                 BIT(12)
#define BIT_LDO_GEN1_EXT_XTL0_EN             BIT(11)
#define BIT_LDO_GEN1_XTL1_EN                 BIT(10)
#define BIT_LDO_GEN1_XTL0_EN                 BIT(9)
#define BIT_LDO_DCXO_EXT_XTL0_EN             BIT(8)
#define BIT_LDO_DCXO_XTL1_EN                 BIT(7)
#define BIT_LDO_DCXO_XTL0_EN                 BIT(6)
#define BIT_LDO_VDD18_EXT_XTL0_EN            BIT(5)
#define BIT_LDO_VDD18_XTL1_EN                BIT(4)
#define BIT_LDO_VDD18_XTL0_EN                BIT(3)
#define BIT_LDO_VDD28_EXT_XTL0_EN            BIT(2)
#define BIT_LDO_VDD28_XTL1_EN                BIT(1)
#define BIT_LDO_VDD28_XTL0_EN                BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_PWR_XTL_EN1
//Register Offset        :    0x00C8
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_LDO_RF0_EXT_XTL0_EN              BIT(14)
#define BIT_LDO_RF0_XTL1_EN                  BIT(13)
#define BIT_LDO_RF0_XTL0_EN                  BIT(12)
#define BIT_LDO_WIFIPA_EXT_XTL0_EN           BIT(11)
#define BIT_LDO_WIFIPA_XTL1_EN               BIT(10)
#define BIT_LDO_WIFIPA_XTL0_EN               BIT(9)
#define BIT_LDO_SIM2_EXT_XTL0_EN             BIT(8)
#define BIT_LDO_SIM2_XTL1_EN                 BIT(7)
#define BIT_LDO_SIM2_XTL0_EN                 BIT(6)
#define BIT_LDO_SIM1_EXT_XTL0_EN             BIT(5)
#define BIT_LDO_SIM1_XTL1_EN                 BIT(4)
#define BIT_LDO_SIM1_XTL0_EN                 BIT(3)
#define BIT_LDO_SIM0_EXT_XTL0_EN             BIT(2)
#define BIT_LDO_SIM0_XTL1_EN                 BIT(1)
#define BIT_LDO_SIM0_XTL0_EN                 BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_PWR_XTL_EN2
//Register Offset        :    0x00CC
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_LDO_VDD25_EXT_XTL0_EN            BIT(11)
#define BIT_LDO_VDD25_XTL1_EN                BIT(10)
#define BIT_LDO_VDD25_XTL0_EN                BIT(9)
#define BIT_DCDC_RF_EXT_XTL0_EN              BIT(8)
#define BIT_DCDC_RF_XTL1_EN                  BIT(7)
#define BIT_DCDC_RF_XTL0_EN                  BIT(6)
#define BIT_XO_EXT_XTL0_EN                   BIT(5)
#define BIT_XO_XTL1_EN                       BIT(4)
#define BIT_XO_XTL0_EN                       BIT(3)
#define BIT_BG_EXT_XTL0_EN                   BIT(2)
#define BIT_BG_XTL1_EN                       BIT(1)
#define BIT_BG_XTL0_EN                       BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_PWR_XTL_EN3
//Register Offset        :    0x00D0
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_DCDC_CON_EXT_XTL0_EN             BIT(14)
#define BIT_DCDC_CON_XTL1_EN                 BIT(13)
#define BIT_DCDC_CON_XTL0_EN                 BIT(12)
#define BIT_DCDC_WPA_EXT_XTL0_EN             BIT(11)
#define BIT_DCDC_WPA_XTL1_EN                 BIT(10)
#define BIT_DCDC_WPA_XTL0_EN                 BIT(9)
#define BIT_DCDC_MEM_EXT_XTL0_EN             BIT(8)
#define BIT_DCDC_MEM_XTL1_EN                 BIT(7)
#define BIT_DCDC_MEM_XTL0_EN                 BIT(6)
#define BIT_DCDC_GEN_EXT_XTL0_EN             BIT(5)
#define BIT_DCDC_GEN_XTL1_EN                 BIT(4)
#define BIT_DCDC_GEN_XTL0_EN                 BIT(3)
#define BIT_DCDC_CORE_EXT_XTL0_EN            BIT(2)
#define BIT_DCDC_CORE_XTL1_EN                BIT(1)
#define BIT_DCDC_CORE_XTL0_EN                BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_32KLESS_CTRL0
//Register Offset        :    0x00D4
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_RC_MODE_WR_ACK_FLAG              BIT(14)
#define BIT_XO_LOW_CUR_FLAG                  BIT(13)
#define BIT_XO_LOW_CUR_FRC_RTCSET            BIT(12)
#define BIT_XO_LOW_CUR_FRC_RTCCLR            BIT(11)
#define BIT_RC_MODE_WR_ACK_FLAG_CLR          BIT(10)
#define BIT_XO_LOW_CUR_FLAG_CLR              BIT(9)
#define BIT_XO_LOW_CUR_CNT_CLR               BIT(8)
#define BIT_LDO_DCXO_LP_PD_RTCSET            BIT(7)
#define BIT_LDO_DCXO_LP_PD_RTCCLR            BIT(6)
#define BIT_SLP_XO_LOW_CUR_EN                BIT(5)
#define BIT_XO_LOW_CUR_EN                    BIT(3)
#define BIT_EXT_32K_PD                       BIT(2)
#define BIT_RC_32K_SEL                       BIT(1)
#define BIT_RC_32K_EN                        BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_32KLESS_CTRL1
//Register Offset        :    0x00D8
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_RC_MODE(x)                      (((x) & 0xFFFF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_32KLESS_CTRL2
//Register Offset        :    0x00DC
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_XO_LOW_CUR_CNT_LOW(x)           (((x) & 0xFFFF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_32KLESS_CTRL3
//Register Offset        :    0x00E0
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_XO_LOW_CUR_CNT_HIGH(x)          (((x) & 0xFFFF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_AUXAD_CTL
//Register Offset        :    0x00E4
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_AUXAD_CURRENTSEN_EN              BIT(6)
#define BIT_AUXAD_CURRENTSEL                 BIT(5)
#define BITS_AUXAD_CURRENT_IBS(x)            (((x) & 0x1F) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_XTL_WAIT_CTRL
//Register Offset        :    0x00E8
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_SLP_XTLBUF_PD_EN                 BIT(9)
#define BIT_XTL_EN                           BIT(8)
#define BITS_XTL_WAIT(x)                     (((x) & 0xFF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_RGB_CTRL
//Register Offset        :    0x00EC
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_RTC_PWM0_EN                      BIT(15)
#define BIT_PWM0_EN                          BIT(14)
#define BIT_IB_REX_EN                        BIT(12)
#define BIT_IB_TRIM_EM_SEL                   BIT(11)
#define BITS_RGB_V(x)                        (((x) & 0x1F) << 4)
#define BIT_SLP_RGB_PD_EN                    BIT(2)
#define BIT_RGB_PD_HW_EN                     BIT(1)
#define BIT_RGB_PD_SW                        BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_WHTLED_CTRL
//Register Offset        :    0x00F0
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_IB_TRIM(x)                      (((x) & 0x7F) << 9)
#define BIT_WHTLED_SERIES_EN                 BIT(8)
#define BIT_WHTLED_PD_SEL                    BIT(7)
#define BITS_WHTLED_V(x)                     (((x) & 0x3F) << 1)
#define BIT_WHTLED_PD                        BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_KPLED_CTRL
//Register Offset        :    0x00F4
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_KPLED_V(x)                      (((x) & 0xF) << 12)
#define BIT_KPLED_PD                         BIT(11)
#define BIT_KPLED_PULLDOWN_EN                BIT(10)
#define BIT_SLP_LDOKPLED_PD_EN               BIT(9)
#define BIT_LDO_KPLED_PD                     BIT(8)
#define BITS_LDO_KPLED_V(x)                  (((x) & 0xFF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_VIBR_CTRL0
//Register Offset        :    0x00F8
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_CUR_DRV_CAL_SEL(x)              (((x) & 0x3) << 12)
#define BIT_VIBR_PULLDOWN_EN                 BIT(11)
#define BIT_VIBR_PULLUP_EN                   BIT(10)
#define BIT_SLP_LDOVIBR_PD_EN                BIT(9)
#define BIT_LDO_VIBR_PD                      BIT(8)
#define BITS_LDO_VIBR_V(x)                   (((x) & 0xFF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_AUDIO_CTRL0
//Register Offset        :    0x00FC
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_AUD_SLP_APP_RST_EN               BIT(15)
#define BITS_CLK_AUD_HBD_DIV(x)              (((x) & 0x1F) << 8)
#define BIT_CLK_AUD_LOOP_INV_EN              BIT(4)
#define BIT_CLK_AUDIF_TX_INV_EN              BIT(3)
#define BIT_CLK_AUDIF_RX_INV_EN              BIT(2)
#define BIT_CLK_AUD_6P5M_TX_INV_EN           BIT(1)
#define BIT_CLK_AUD_6P5M_RX_INV_EN           BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_AUDIO_CTRL1
//Register Offset        :    0x0100
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_IMPD_ADC_SOFT_RST                BIT(7)
#define BIT_HEAD_INSERT_EIC_EN               BIT(6)
#define BIT_AUDIO_CHP_CLK_DIV_EN             BIT(5)
#define BITS_AUDIO_CHP_CLK_DIV(x)            (((x) & 0x1F) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_CHGR_CTRL0
//Register Offset        :    0x0104
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_CHGLDO_DIS                       BIT(15)
#define BITS_CHGR_CV_V(x)                    (((x) & 0x3F) << 6)
#define BITS_CHGR_END_V(x)                   (((x) & 0x3) << 4)
#define BIT_CHGR_PD                          BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_CHGR_CTRL1
//Register Offset        :    0x0108
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_CHGR_CC_I(x)                    (((x) & 0x1F) << 10)
#define BITS_VBAT_OVP_V(x)                   (((x) & 0xF) << 6)
#define BITS_VCHG_OVP_V(x)                   (((x) & 0x3F) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_CHGR_CTRL2
//Register Offset        :    0x010C
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_CHGR_INT_EN                      BIT(15)
#define BIT_CHGR_DRV                         BIT(7)
#define BIT_CHGR_OSC                         BIT(6)
#define BITS_CHGR_DPM(x)                     (((x) & 0x3) << 4)
#define BITS_CHGR_ITERM(x)                   (((x) & 0x3) << 2)
#define BIT_CHGR_CC_EN                       BIT(1)
#define BIT_RECHG                            BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_CHGR_DET_FGU_CTRL
//Register Offset        :    0x0110
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_FGUA_SOFT_RST                    BIT(13)
#define BIT_LDO_FGU_PD                       BIT(12)
#define BIT_SD_CHOP_CAP_EN                   BIT(8)
#define BITS_SD_CLK_P(x)                     (((x) & 0x3) << 6)
#define BIT_SD_DCOFFSET_EN                   BIT(5)
#define BIT_SD_CHOP_EN                       BIT(4)
#define BIT_DP_DM_AUX_EN                     BIT(1)
#define BIT_DP_DM_SW_EN                      BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_CHGR_STATUS
//Register Offset        :    0x0114
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_CHG_DET_DONE                     BIT(11)
#define BIT_DP_LOW                           BIT(10)
#define BIT_DCP_DET                          BIT(9)
#define BIT_CHG_DET                          BIT(8)
#define BIT_SDP_INT                          BIT(7)
#define BIT_DCP_INT                          BIT(6)
#define BIT_CDP_INT                          BIT(5)
#define BIT_CHGR_CV_STATUS                   BIT(4)
#define BIT_CHGR_ON                          BIT(3)
#define BIT_CHGR_INT                         BIT(2)
#define BIT_VBAT_OVI                         BIT(1)
#define BIT_VCHG_OVI                         BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_MIXED_CTRL0
//Register Offset        :    0x0118
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_PTEST_PD_RTCSET                  BIT(15)
#define BIT_DCDC_V_CTRL_MODE                 BIT(14)
#define BIT_LDO_RAMP_EN                      BIT(13)
#define BIT_BG_LP_EN                         BIT(12)
#define BITS_VBAT_CRASH_V(x)                 (((x) & 0x3) << 10)
#define BIT_OVLO_EN                          BIT(9)
#define BITS_OVLO_CAL(x)                     (((x) & 0x1F) << 4)
#define BITS_OVLO_V(x)                       (((x) & 0x3) << 2)
#define BITS_OVLO_T(x)                       (((x) & 0x3) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_MIXED_CTRL1
//Register Offset        :    0x011C
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_XOSC32K_CTL(x)                  (((x) & 0xF) << 12)
#define BITS_BATON_T(x)                      (((x) & 0x3) << 10)
#define BIT_BATDET_LDO_SEL                   BIT(9)
#define BIT_THM_CHIP_PD_FLAG                 BIT(8)
#define BIT_THM_CHIP_PD_FLAG_CLR             BIT(7)
#define BITS_THM_CAL_SEL(x)                  (((x) & 0x3) << 5)
#define BIT_THM_AUTO_PD_EN                   BIT(4)
#define BIT_ALL_GPI_DEB                      BIT(3)
#define BIT_GPI_DEBUG_EN                     BIT(2)
#define BIT_ALL_INT_DEB                      BIT(1)
#define BIT_INT_DEBUG_EN                     BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_SWRST_CTRL
//Register Offset        :    0x0120
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_POR_RTC_PD                       BIT(15)
#define BITS_POR_RTC_I(x)                    (((x) & 0x3) << 13)
#define BIT_SW_RST_GEN1_PD_EN                BIT(12)
#define BIT_SW_RST_GEN0_PD_EN                BIT(11)
#define BIT_EXT_RSTN_PD_EN                   BIT(10)
#define BIT_PB_7S_RST_PD_EN                  BIT(9)
#define BIT_SW_RST_EMMCCORE_PD_EN            BIT(8)
#define BIT_KEY2_7S_RST_EN                   BIT(7)
#define BIT_WDG_RST_PD_EN                    BIT(6)
#define BITS_SW_RST_PD_THRESHOLD(x)          (((x) & 0xF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_POR_RST_MONITOR
//Register Offset        :    0x0124
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_POR_RST_MONITOR(x)              (((x) & 0xFFFF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_WDG_RST_MONITOR
//Register Offset        :    0x0128
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_WDG_RST_MONITOR(x)              (((x) & 0xFFFF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_POR_PIN_RST_MONITOR
//Register Offset        :    0x012C
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_POR_PIN_RST_MONITOR(x)          (((x) & 0xFFFF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_POR_SRC_FLAG
//Register Offset        :    0x0130
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_POR_SW_FORCE_ON                  BIT(15)
#define BITS_POR_SRC_FLAG(x)                 (((x) & 0x7FFF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_POR_7S_CTRL
//Register Offset        :    0x0134
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_PBINT_7S_FLAG_CLR                BIT(15)
#define BIT_EXT_RSTN_FLAG_CLR                BIT(14)
#define BIT_CHGR_INT_FLAG_CLR                BIT(13)
#define BIT_PBINT2_FLAG_CLR                  BIT(12)
#define BIT_PBINT_FLAG_CLR                   BIT(11)
#define BIT_PBINT_7S_RST_SWMODE              BIT(8)
#define BITS_PBINT_7S_RST_THRESHOLD(x)       (((x) & 0x7) << 5)
#define BIT_EXT_RSTN_MODE                    BIT(4)
#define BIT_PBINT_7S_AUTO_ON_EN              BIT(2)
#define BIT_PBINT_7S_RST_DISABLE             BIT(1)
#define BIT_PBINT_7S_RST_MODE                BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_HWRST_RTC
//Register Offset        :    0x0138
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_HWRST_RTC_REG_STS(x)            (((x) & 0xFF) << 8)
#define BITS_HWRST_RTC_REG_SET(x)            (((x) & 0xFF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_CHIP_ID_LOW
//Register Offset        :    0x013C
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_CHIP_ID_LOW(x)                  (((x) & 0xFFFF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_CHIP_ID_HIGH
//Register Offset        :    0x0140
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_CHIP_ID_HIGH(x)                 (((x) & 0xFFFF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_ARM_MF_REG
//Register Offset        :    0x0144
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_ARM_MF_REG(x)                   (((x) & 0xFFFF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_ARCH_EN
//Register Offset        :    0x0148
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_ARCH_EN                          BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_MCU_WR_PROT_VALUE
//Register Offset        :    0x014C
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_MCU_WR_PROT                      BIT(15)
#define BITS_MCU_WR_PROT_VALUE(x)            (((x) & 0x7FFF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_PWR_WR_PROT_VALUE
//Register Offset        :    0x0150
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_PWR_WR_PROT                      BIT(15)
#define BITS_PWR_WR_PROT_VALUE(x)            (((x) & 0x7FFF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_SMPL_CTRL0
//Register Offset        :    0x0154
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_SMPL_MODE(x)                    (((x) & 0xFFFF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_SMPL_CTRL1
//Register Offset        :    0x0158
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_SMPL_PWR_ON_FLAG                 BIT(15)
#define BIT_SMPL_MODE_WR_ACK_FLAG            BIT(14)
#define BIT_SMPL_PWR_ON_FLAG_CLR             BIT(13)
#define BIT_SMPL_MODE_WR_ACK_FLAG_CLR        BIT(12)
#define BIT_SMPL_PWR_ON_SET                  BIT(11)
#define BIT_SMPL_EN                          BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_RTC_RST0
//Register Offset        :    0x015C
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_RTC_CLK_FLAG_SET(x)             (((x) & 0xFFFF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_RTC_RST1
//Register Offset        :    0x0160
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_RTC_CLK_FLAG_CLR(x)             (((x) & 0xFFFF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_RTC_RST2
//Register Offset        :    0x0164
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_RTC_CLK_FLAG_RTC(x)             (((x) & 0xFFFF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_LDO_SHPT_PD1
//Register Offset        :    0x0188
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_LDO_USB_SHPT_PD                  BIT(12)
#define BIT_LDO_DCXO_SHPT_PD                 BIT(11)
#define BIT_LDO_WIFIPA_SHPT_PD               BIT(10)
#define BIT_LDO_VDD25_SHPT_PD                BIT(9)
#define BIT_LDO_VDD28_SHPT_PD                BIT(8)
#define BIT_LDO_SDIO_SHPT_PD                 BIT(7)
#define BIT_LDO_SDCORE_SHPT_PD               BIT(6)
#define BIT_LDO_EMMCCORE_SHPT_PD             BIT(5)
#define BIT_LDO_SIM2_SHPT_PD                 BIT(4)
#define BIT_LDO_SIM1_SHPT_PD                 BIT(3)
#define BIT_LDO_SIM0_SHPT_PD                 BIT(2)
#define BIT_LDO_CAMMOT_SHPT_PD               BIT(1)
#define BIT_LDO_CAMA_SHPT_PD                 BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_LDO_SHPT_PD2
//Register Offset        :    0x018C
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_LDO_RF0_SHPT_PD                  BIT(7)
#define BIT_LDO_GEN1_SHPT_PD                 BIT(6)
#define BIT_LDO_GEN0_SHPT_PD                 BIT(5)
#define BIT_LDO_CAMD_SHPT_PD                 BIT(4)
#define BIT_LDO_VDD18_SHPT_PD                BIT(3)
#define BIT_LDO_CAMIO_SHPT_PD                BIT(2)
#define BIT_LDO_KPLED_SHPT_PD                BIT(1)
#define BIT_LDO_VIBR_SHPT_PD                 BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_BATDET_CUR_CTRL
//Register Offset        :    0x0190
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_BATDET_CUR_EN                    BIT(4)
#define BITS_BATDET_CUR_I(x)                 (((x) & 0xF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_RTC_CLK_STOP
//Register Offset        :    0x0194
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_RTC_CLK_STOP_FLAG                BIT(7)
#define BITS_RTC_CLK_STOP_THRESHOLD(x)       (((x) & 0x7F) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_VBAT_DROP_CNT
//Register Offset        :    0x0198
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_VBAT_DROP_CNT(x)                (((x) & 0xFFF) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_DISCHRG
//Register Offset        :    0x019C
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_DCDC_ARM_DISCHRG(x)             (((x) & 0x3) << 12)
#define BITS_DCDC_CORE_DISCHRG(x)            (((x) & 0x3) << 10)
#define BITS_DCDC_MEM_DISCHRG(x)             (((x) & 0x3) << 8)
#define BITS_DCDC_GEN_DISCHRG(x)             (((x) & 0x3) << 6)
#define BITS_DCDC_RF_DISCHRG(x)              (((x) & 0x3) << 4)
#define BITS_DCDC_CON_DISCHRG(x)             (((x) & 0x3) << 2)
#define BITS_DCDC_WPA_DISCHRG(x)             (((x) & 0x3) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_CORE_ADI
//Register Offset        :    0x0200
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_DCDC_CORE_CTL_SW_ADI(x)         (((x) & 0x1F) << 5)
#define BITS_DCDC_CORE_CAL_SW_ADI(x)         (((x) & 0x1F) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_ARM_ADI
//Register Offset        :    0x0204
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_DCDC_ARM_CTL_ADI(x)             (((x) & 0x1F) << 5)
#define BITS_DCDC_ARM_CAL_ADI(x)             (((x) & 0x1F) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_MEM_ADI
//Register Offset        :    0x0208
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_DCDC_MEM_CTRL_ADI(x)            (((x) & 0x1F) << 5)
#define BITS_DCDC_MEM_CAL_ADI(x)             (((x) & 0x1F) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_GEN_ADI
//Register Offset        :    0x020C
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_DCDC_GEN_CTRL_ADI(x)            (((x) & 0x1F) << 5)
#define BITS_DCDC_GEN_CAL_ADI(x)             (((x) & 0x1F) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_WPA_ADI
//Register Offset        :    0x0210
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_DCDC_WPA_CAL_ADI(x)             (((x) & 0x7) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_WPA_DCM_ADI
//Register Offset        :    0x0214
//Register Description   : 
-----------------------------------------------------------*/

#define BIT_DCDC_WPA_DCM_ADI                 BIT(0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_CON_ADI
//Register Offset        :    0x0218
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_DCDC_CON_CTRL_ADI(x)            (((x) & 0x1F) << 5)
#define BITS_DCDC_CON_CAL_ADI(x)             (((x) & 0x1F) << 0)


/*-----------------------------------------------------------
//Register Name          :    ANA_REG_GLB_DCDC_RF_ADI
//Register Offset        :    0x021C
//Register Description   : 
-----------------------------------------------------------*/

#define BITS_DCDC_RF_CTRL_ADI(x)             (((x) & 0x1F) << 5)
#define BITS_DCDC_RF_CAL_ADI(x)              (((x) & 0x1F) << 0)

#endif

/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This file is dual-licensed: you can use it either under the terms
 * of the GPL or the X11 license, at your option. Note that this dual
 * licensing only applies to this file, and not this project as a
 * whole.
 *
 */


#ifndef _SC9830_AON_APB_GLB_H
#define _SC9830_AON_APB_GLB_H


#define REG_AON_APB_APB_EB0                0x0000
#define REG_AON_APB_APB_EB1                0x0004
#define REG_AON_APB_APB_RST0               0x0008
#define REG_AON_APB_APB_RST1               0x000C
#define REG_AON_APB_APB_RTC_EB             0x0010
#define REG_AON_APB_REC_26MHZ_BUF_CFG      0x0014
#define REG_AON_APB_SINDRV_CTRL            0x0018
#define REG_AON_APB_ADA_SEL_CTRL           0x001C
#define REG_AON_APB_VBC_CTRL               0x0020
#define REG_AON_APB_PWR_CTRL               0x0024
#define REG_AON_APB_TS_CFG                 0x0028
#define REG_AON_APB_BOOT_MODE              0x002C
#define REG_AON_APB_BB_BG_CTRL             0x0030
#define REG_AON_APB_CP_ARM_JTAG_CTRL       0x0034
#define REG_AON_APB_PLL_SOFT_CNT_DONE      0x0038
#define REG_AON_APB_DCXO_LC_REG0           0x003C
#define REG_AON_APB_DCXO_LC_REG1           0x0040
#define REG_AON_APB_MPLL_CFG1              0x0044
#define REG_AON_APB_MPLL_CFG2              0x0048
#define REG_AON_APB_DPLL_CFG1              0x004C
#define REG_AON_APB_DPLL_CFG2              0x0050
#define REG_AON_APB_TWPLL_CFG1             0x0054
#define REG_AON_APB_TWPLL_CFG2             0x0058
#define REG_AON_APB_LTEPLL_CFG1            0x005C
#define REG_AON_APB_LTEPLL_CFG2            0x0060
#define REG_AON_APB_LVDSDISPLL_CFG1        0x0064
#define REG_AON_APB_LVDSDISPLL_CFG2        0x0068
#define REG_AON_APB_AON_REG_PROT           0x006C
#define REG_AON_APB_LDSP_BOOT_EN           0x0070
#define REG_AON_APB_LDSP_BOOT_VEC          0x0074
#define REG_AON_APB_LDSP_RST               0x0078
#define REG_AON_APB_LDSP_MTX_CTRL1         0x007C
#define REG_AON_APB_LDSP_MTX_CTRL2         0x0080
#define REG_AON_APB_LDSP_MTX_CTRL3         0x0084
#define REG_AON_APB_AON_CGM_CFG            0x0088
#define REG_AON_APB_LACC_MTX_CTRL          0x008C
#define REG_AON_APB_CORTEX_MTX_CTRL1       0x0090
#define REG_AON_APB_CORTEX_MTX_CTRL2       0x0094
#define REG_AON_APB_CORTEX_MTX_CTRL3       0x0098
#define REG_AON_APB_CA5_TCLK_DLY_LEN       0x009C
#define REG_AON_APB_APB_EB2		   0x00B0
#define REG_AON_APB_AON_CHIP_ID            0x00FC
#define REG_AON_APB_CCIR_RCVR_CFG          0x0100
#define REG_AON_APB_PLL_BG_CFG             0x0108
#define REG_AON_APB_LVDSDIS_SEL            0x010C
#define REG_AON_APB_DJTAG_MUX_SEL          0x0110
#define REG_AON_APB_ARM7_SYS_SOFT_RST      0x0114
#define REG_AON_APB_CP1_CP0_ADDR_MSB       0x0118
#define REG_AON_APB_AON_DMA_INT_EN         0x011C
#define REG_AON_APB_EMC_AUTO_GATE_EN       0x0120
#define REG_AON_APB_ARM7_CFG_BUS           0x0124
#define REG_AON_APB_RTC4M_0_CFG            0x0128
#define REG_AON_APB_RTC4M_1_CFG            0x012C
#define REG_AON_APB_APB_RST2               0x0130
#define REG_AON_APB_AP_WPROT_EN1           0x3004
#define REG_AON_APB_CP0_WPROT_EN1          0x3008
#define REG_AON_APB_CP1_WPROT_EN1          0x300C
#define REG_AON_APB_IO_DLY_CTRL            0x3014
#define REG_AON_APB_AP_WPROT_EN0           0x3018
#define REG_AON_APB_CP0_WPROT_EN0          0x3020
#define REG_AON_APB_CP1_WPROT_EN0          0x3024
#define REG_AON_APB_PMU_RST_MONITOR        0x302C
#define REG_AON_APB_THM_RST_MONITOR        0x3030
#define REG_AON_APB_AP_RST_MONITOR         0x3034
#define REG_AON_APB_CA7_RST_MONITOR        0x3038
#define REG_AON_APB_BOND_OPT0              0x303C
#define REG_AON_APB_BOND_OPT1              0x3040
#define REG_AON_APB_RES_REG0               0x3044
#define REG_AON_APB_RES_REG1               0x3048
#define REG_AON_APB_AON_QOS_CFG            0x304C
#define REG_AON_APB_BB_LDO_CAL_START       0x3050
#define REG_AON_APB_AON_MTX_PROT_CFG       0x3058
#define REG_AON_APB_LVDS_CFG               0x3060
#define REG_AON_APB_PLL_LOCK_OUT_SEL       0x3064
#define REG_AON_APB_RTC4M_RC_VAL           0x3068
#define REG_AON_APB_AON_APB_RSV            0x30F0

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_APB_EB0
// Register Offset : 0x0000
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_I2C_EB                BIT(31)
#define BIT_AON_APB_CA7_DAP_EB            BIT(30)
#define BIT_AON_APB_CA7_TS1_EB            BIT(29)
#define BIT_AON_APB_CA7_TS0_EB            BIT(28)
#define BIT_AON_APB_GPU_EB                BIT(27)
#define BIT_AON_APB_CKG_EB                BIT(26)
#define BIT_AON_APB_MM_EB                 BIT(25)
#define BIT_AON_APB_AP_WDG_EB             BIT(24)
#define BIT_AON_APB_MSPI_EB               BIT(23)
#define BIT_AON_APB_SPLK_EB               BIT(22)
#define BIT_AON_APB_IPI_EB                BIT(21)
#define BIT_AON_APB_PIN_EB                BIT(20)
#define BIT_AON_APB_VBC_EB                BIT(19)
#define BIT_AON_APB_AUD_EB                BIT(18)
#define BIT_AON_APB_AUDIF_EB              BIT(17)
#define BIT_AON_APB_ADI_EB                BIT(16)
#define BIT_AON_APB_INTC_EB               BIT(15)
#define BIT_AON_APB_EIC_EB                BIT(14)
#define BIT_AON_APB_EFUSE_EB              BIT(13)
#define BIT_AON_APB_AP_TMR0_EB            BIT(12)
#define BIT_AON_APB_AON_TMR_EB            BIT(11)
#define BIT_AON_APB_AP_SYST_EB            BIT(10)
#define BIT_AON_APB_AON_SYST_EB           BIT(9)
#define BIT_AON_APB_KPD_EB                BIT(8)
#define BIT_AON_APB_PWM3_EB               BIT(7)
#define BIT_AON_APB_PWM2_EB               BIT(6)
#define BIT_AON_APB_PWM1_EB               BIT(5)
#define BIT_AON_APB_PWM0_EB               BIT(4)
#define BIT_AON_APB_GPIO_EB               BIT(3)
#define BIT_AON_APB_TPC_EB                BIT(2)
#define BIT_AON_APB_FM_EB                 BIT(1)
#define BIT_AON_APB_ADC_EB                BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_APB_EB1
// Register Offset : 0x0004
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_ORP_JTAG_EB           BIT(27)
#define BIT_AON_APB_CA5_TS0_EB            BIT(26)
#define BIT_AON_APB_DEF_EB                BIT(25)
#define BIT_AON_APB_LVDS_PLL_DIV_EN       BIT(24)
#define BIT_AON_APB_ARM7_JTAG_EB          BIT(23)
#define BIT_AON_APB_AON_DMA_EB            BIT(22)
#define BIT_AON_APB_MBOX_EB               BIT(21)
#define BIT_AON_APB_DJTAG_EB              BIT(20)
#define BIT_AON_APB_RTC4M1_CAL_EB         BIT(19)
#define BIT_AON_APB_RTC4M0_CAL_EB         BIT(18)
#define BIT_AON_APB_MDAR_EB               BIT(17)
#define BIT_AON_APB_LVDS_TCXO_EB          BIT(16)
#define BIT_AON_APB_LVDS_TRX_EB           BIT(15)
#define BIT_AON_APB_CA5_DAP_EB            BIT(14)
#define BIT_AON_APB_GSP_EMC_EB            BIT(13)
#define BIT_AON_APB_ZIP_EMC_EB            BIT(12)
#define BIT_AON_APB_DISP_EMC_EB           BIT(11)
#define BIT_AON_APB_AP_TMR2_EB            BIT(10)
#define BIT_AON_APB_AP_TMR1_EB            BIT(9)
#define BIT_AON_APB_APCPU_WDG_EB          BIT(8)
#define BIT_AON_APB_AVS_EB                BIT(6)
#define BIT_AON_APB_PROBE_EB              BIT(5)
#define BIT_AON_APB_AUX2_EB               BIT(4)
#define BIT_AON_APB_AUX1_EB               BIT(3)
#define BIT_AON_APB_AUX0_EB               BIT(2)
#define BIT_AON_APB_THM_EB                BIT(1)
#define BIT_AON_APB_PMU_EB                BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_APB_RST0
// Register Offset : 0x0008
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_CA5_TS0_SOFT_RST      BIT(31)
#define BIT_AON_APB_I2C_SOFT_RST          BIT(30)
#define BIT_AON_APB_CA7_TS1_SOFT_RST      BIT(29)
#define BIT_AON_APB_CA7_TS0_SOFT_RST      BIT(28)
#define BIT_AON_APB_DAP_MTX_SOFT_RST      BIT(27)
#define BIT_AON_APB_MSPI1_SOFT_RST        BIT(26)
#define BIT_AON_APB_MSPI0_SOFT_RST        BIT(25)
#define BIT_AON_APB_SPLK_SOFT_RST         BIT(24)
#define BIT_AON_APB_IPI_SOFT_RST          BIT(23)
#define BIT_AON_APB_CKG_SOFT_RST          BIT(22)
#define BIT_AON_APB_PIN_SOFT_RST          BIT(21)
#define BIT_AON_APB_VBC_SOFT_RST          BIT(20)
#define BIT_AON_APB_AUD_SOFT_RST          BIT(19)
#define BIT_AON_APB_AUDIF_SOFT_RST        BIT(18)
#define BIT_AON_APB_ADI_SOFT_RST          BIT(17)
#define BIT_AON_APB_INTC_SOFT_RST         BIT(16)
#define BIT_AON_APB_EIC_SOFT_RST          BIT(15)
#define BIT_AON_APB_EFUSE_SOFT_RST        BIT(14)
#define BIT_AON_APB_AP_WDG_SOFT_RST       BIT(13)
#define BIT_AON_APB_AP_TMR0_SOFT_RST      BIT(12)
#define BIT_AON_APB_AON_TMR_SOFT_RST      BIT(11)
#define BIT_AON_APB_AP_SYST_SOFT_RST      BIT(10)
#define BIT_AON_APB_AON_SYST_SOFT_RST     BIT(9)
#define BIT_AON_APB_KPD_SOFT_RST          BIT(8)
#define BIT_AON_APB_PWM3_SOFT_RST         BIT(7)
#define BIT_AON_APB_PWM2_SOFT_RST         BIT(6)
#define BIT_AON_APB_PWM1_SOFT_RST         BIT(5)
#define BIT_AON_APB_PWM0_SOFT_RST         BIT(4)
#define BIT_AON_APB_GPIO_SOFT_RST         BIT(3)
#define BIT_AON_APB_TPC_SOFT_RST          BIT(2)
#define BIT_AON_APB_FM_SOFT_RST           BIT(1)
#define BIT_AON_APB_ADC_SOFT_RST          BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_APB_RST1
// Register Offset : 0x000C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_RTC4M_ANA_SOFT_RST     BIT(31)
#define BIT_AON_APB_DEF_SLV_INT_SOFT_CLR   BIT(30)
#define BIT_AON_APB_DEF_SOFT_RST           BIT(29)
#define BIT_AON_APB_ADC3_SOFT_RST          BIT(28)
#define BIT_AON_APB_ADC2_SOFT_RST          BIT(27)
#define BIT_AON_APB_ADC1_SOFT_RST          BIT(26)
#define BIT_AON_APB_MBOX_SOFT_RST          BIT(25)
#define BIT_AON_APB_RTC4M1_CAL_SOFT_RST    BIT(23)
#define BIT_AON_APB_RTC4M0_CAL_SOFT_RST    BIT(22)
#define BIT_AON_APB_LDSP_SYS_SOFT_RST      BIT(21)
#define BIT_AON_APB_LCP_SYS_SOFT_RST       BIT(20)
#define BIT_AON_APB_DAC3_SOFT_RST          BIT(19)
#define BIT_AON_APB_DAC2_SOFT_RST          BIT(18)
#define BIT_AON_APB_DAC1_SOFT_RST          BIT(17)
#define BIT_AON_APB_ADC3_CAL_SOFT_RST      BIT(16)
#define BIT_AON_APB_ADC2_CAL_SOFT_RST      BIT(15)
#define BIT_AON_APB_ADC1_CAL_SOFT_RST      BIT(14)
#define BIT_AON_APB_MDAR_SOFT_RST          BIT(13)
#define BIT_AON_APB_LVDSDIS_SOFT_RST       BIT(12)
#define BIT_AON_APB_BB_CAL_SOFT_RST        BIT(11)
#define BIT_AON_APB_DCXO_LC_SOFT_RST       BIT(10)
#define BIT_AON_APB_AP_TMR2_SOFT_RST       BIT(9)
#define BIT_AON_APB_AP_TMR1_SOFT_RST       BIT(8)
#define BIT_AON_APB_APCPU_WDG_SOFT_RST     BIT(7)
#define BIT_AON_APB_AON_DMA_SOFT_RST       BIT(6)
#define BIT_AON_APB_AVS_SOFT_RST           BIT(5)
#define BIT_AON_APB_DMC_PHY_SOFT_RST       BIT(4)
#define BIT_AON_APB_GPU_THMA_SOFT_RST      BIT(3)
#define BIT_AON_APB_ARM_THMA_SOFT_RST      BIT(2)
#define BIT_AON_APB_THM_SOFT_RST           BIT(1)
#define BIT_AON_APB_PMU_SOFT_RST           BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_APB_RTC_EB
// Register Offset : 0x0010
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_CP0_LTE_EB             BIT(19)
#define BIT_AON_APB_BB_CAL_RTC_EB          BIT(18)
#define BIT_AON_APB_DCXO_LC_RTC_EB         BIT(17)
#define BIT_AON_APB_AP_TMR2_RTC_EB         BIT(16)
#define BIT_AON_APB_AP_TMR1_RTC_EB         BIT(15)
#define BIT_AON_APB_GPU_THMA_RTC_AUTO_EN   BIT(14)
#define BIT_AON_APB_ARM_THMA_RTC_AUTO_EN   BIT(13)
#define BIT_AON_APB_GPU_THMA_RTC_EB        BIT(12)
#define BIT_AON_APB_ARM_THMA_RTC_EB        BIT(11)
#define BIT_AON_APB_THM_RTC_EB             BIT(10)
#define BIT_AON_APB_APCPU_WDG_RTC_EB       BIT(9)
#define BIT_AON_APB_AP_WDG_RTC_EB          BIT(8)
#define BIT_AON_APB_EIC_RTCDV5_EB          BIT(7)
#define BIT_AON_APB_EIC_RTC_EB             BIT(6)
#define BIT_AON_APB_AP_TMR0_RTC_EB         BIT(5)
#define BIT_AON_APB_AON_TMR_RTC_EB         BIT(4)
#define BIT_AON_APB_AP_SYST_RTC_EB         BIT(3)
#define BIT_AON_APB_AON_SYST_RTC_EB        BIT(2)
#define BIT_AON_APB_KPD_RTC_EB             BIT(1)
#define BIT_AON_APB_ARCH_RTC_EB            BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_REC_26MHZ_BUF_CFG
// Register Offset : 0x0014
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_PLL_PROBE_SEL(x)       (((x) & 0x3F) << 8)
#define BIT_AON_APB_REC_26MHZ_1_CUR_SEL    BIT(4)
#define BIT_AON_APB_REC_26MHZ_0_CUR_SEL    BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_SINDRV_CTRL
// Register Offset : 0x0018
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_SINDRV_LVL(x)          (((x) & 0x3) << 3)
#define BIT_AON_APB_SINDRV_CLIP_MODE       BIT(2)
#define BIT_AON_APB_SINDRV_ENA_SQUARE      BIT(1)
#define BIT_AON_APB_SINDRV_ENA             BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_ADA_SEL_CTRL
// Register Offset : 0x001C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_TW_MODE_SEL            BIT(3)
#define BIT_AON_APB_WGADC_DIV_EN           BIT(2)
#define BIT_AON_APB_AFCDAC_SYS_SEL         BIT(1)
#define BIT_AON_APB_APCDAC_SYS_SEL         BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_VBC_CTRL
// Register Offset : 0x0020
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_AUDIF_CKG_AUTO_EN          BIT(20)
#define BIT_AON_APB_AUD_INT_SYS_SEL(x)         (((x) & 0x3) << 18)
#define BIT_AON_APB_VBC_AFIFO_INT_SYS_SEL(x)   (((x) & 0x3) << 16)
#define BIT_AON_APB_VBC_AD23_INT_SYS_SEL(x)    (((x) & 0x3) << 14)
#define BIT_AON_APB_VBC_AD01_INT_SYS_SEL(x)    (((x) & 0x3) << 12)
#define BIT_AON_APB_VBC_DA01_INT_SYS_SEL(x)    (((x) & 0x3) << 10)
#define BIT_AON_APB_VBC_AD23_DMA_SYS_SEL(x)    (((x) & 0x3) << 8)
#define BIT_AON_APB_VBC_AD01_DMA_SYS_SEL(x)    (((x) & 0x3) << 6)
#define BIT_AON_APB_VBC_DA01_DMA_SYS_SEL(x)    (((x) & 0x3) << 4)
#define BIT_AON_APB_VBC_INT_CP0_ARM_SEL        BIT(3)
#define BIT_AON_APB_VBC_INT_CP1_ARM_SEL        BIT(2)
#define BIT_AON_APB_VBC_DMA_CP0_ARM_SEL        BIT(1)
#define BIT_AON_APB_VBC_DMA_CP1_ARM_SEL        BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_PWR_CTRL
// Register Offset : 0x0024
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_HSIC_PLL_EN                BIT(19)
#define BIT_AON_APB_HSIC_PHY_PD                BIT(18)
#define BIT_AON_APB_HSIC_PS_PD_S               BIT(17)
#define BIT_AON_APB_HSIC_PS_PD_L               BIT(16)
#define BIT_AON_APB_MIPI_DSI_PS_PD_S           BIT(15)
#define BIT_AON_APB_MIPI_DSI_PS_PD_L           BIT(14)
#define BIT_AON_APB_MIPI_CSI_4LANE_PS_PD_S     BIT(13)
#define BIT_AON_APB_MIPI_CSI_4LANE_PS_PD_L     BIT(12)
#define BIT_AON_APB_MIPI_CSI_2LANE_PS_PD_S     BIT(11)
#define BIT_AON_APB_MIPI_CSI_2LANE_PS_PD_L     BIT(10)
#define BIT_AON_APB_CA7_TS1_STOP               BIT(9)
#define BIT_AON_APB_CA7_TS0_STOP               BIT(8)
#define BIT_AON_APB_EFUSE_BIST_PWR_ON          BIT(3)
#define BIT_AON_APB_FORCE_DSI_PHY_SHUTDOWNZ    BIT(2)
#define BIT_AON_APB_FORCE_CSI_PHY_SHUTDOWNZ    BIT(1)
#define BIT_AON_APB_USB_PHY_PD                 BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_TS_CFG
// Register Offset : 0x0028
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_CSYSACK_TS_LP_2            BIT(13)
#define BIT_AON_APB_CSYSREQ_TS_LP_2            BIT(12)
#define BIT_AON_APB_CSYSACK_TS_LP_1            BIT(11)
#define BIT_AON_APB_CSYSREQ_TS_LP_1            BIT(10)
#define BIT_AON_APB_CSYSACK_TS_LP_0            BIT(9)
#define BIT_AON_APB_CSYSREQ_TS_LP_0            BIT(8)
#define BIT_AON_APB_EVENTACK_RESTARTREQ_TS01   BIT(4)
#define BIT_AON_APB_EVENT_RESTARTREQ_TS01      BIT(1)
#define BIT_AON_APB_EVENT_HALTREQ_TS01         BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_BOOT_MODE
// Register Offset : 0x002C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_ARM_JTAG_EN                 BIT(13)
#define BIT_AON_APB_WPLL_OVR_FREQ_SEL           BIT(12)
#define BIT_AON_APB_PTEST_FUNC_ATSPEED_SEL      BIT(8)
#define BIT_AON_APB_PTEST_FUNC_MODE             BIT(7)
#define BIT_AON_APB_USB_DLOAD_EN                BIT(4)
#define BIT_AON_APB_ARM_BOOT_MD3                BIT(3)
#define BIT_AON_APB_ARM_BOOT_MD2                BIT(2)
#define BIT_AON_APB_ARM_BOOT_MD1                BIT(1)
#define BIT_AON_APB_ARM_BOOT_MD0                BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_BB_BG_CTRL
// Register Offset : 0x0030
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_BB_CON_BG                   BIT(22)
#define BIT_AON_APB_BB_BG_RSV(x)                (((x) & 0x3) << 20)
#define BIT_AON_APB_BB_LDO_V(x)                 (((x) & 0xF) << 16)
#define BIT_AON_APB_BB_BG_RBIAS_EN              BIT(15)
#define BIT_AON_APB_BB_BG_IEXT_IB_EN            BIT(14)
#define BIT_AON_APB_BB_LDO_REFCTRL(x)           (((x) & 0x3) << 12)
#define BIT_AON_APB_BB_LDO_AUTO_PD_EN           BIT(11)
#define BIT_AON_APB_BB_LDO_SLP_PD_EN            BIT(10)
#define BIT_AON_APB_BB_LDO_FORCE_ON             BIT(9)
#define BIT_AON_APB_BB_LDO_FORCE_PD             BIT(8)
#define BIT_AON_APB_BB_BG_AUTO_PD_EN            BIT(3)
#define BIT_AON_APB_BB_BG_SLP_PD_EN             BIT(2)
#define BIT_AON_APB_BB_BG_FORCE_ON              BIT(1)
#define BIT_AON_APB_BB_BG_FORCE_PD              BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_CP_ARM_JTAG_CTRL
// Register Offset : 0x0034
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_CP_ARM_JTAG_PIN_SEL(x)      (((x) & 0x7))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_PLL_SOFT_CNT_DONE
// Register Offset : 0x0038
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_RC1_SOFT_CNT_DONE          BIT(13)
#define BIT_AON_APB_RC0_SOFT_CNT_DONE          BIT(12)
#define BIT_AON_APB_XTLBUF1_SOFT_CNT_DONE      BIT(9)
#define BIT_AON_APB_XTLBUF0_SOFT_CNT_DONE      BIT(8)
#define BIT_AON_APB_LVDSPLL_SOFT_CNT_DONE      BIT(4)
#define BIT_AON_APB_LPLL_SOFT_CNT_DONE         BIT(3)
#define BIT_AON_APB_TWPLL_SOFT_CNT_DONE        BIT(2)
#define BIT_AON_APB_DPLL_SOFT_CNT_DONE         BIT(1)
#define BIT_AON_APB_MPLL_SOFT_CNT_DONE         BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_DCXO_LC_REG0
// Register Offset : 0x003C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_DCXO_LC_FLAG              BIT(8)
#define BIT_AON_APB_DCXO_LC_FLAG_CLR          BIT(1)
#define BIT_AON_APB_DCXO_LC_CNT_CLR           BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_DCXO_LC_REG1
// Register Offset : 0x0040
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_DCXO_LC_CNT(x)           (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_MPLL_CFG1
// Register Offset : 0x0044
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_MPLL_RES(x)            (((x) & 0x3) << 28)
#define BIT_AON_APB_MPLL_LOCK_DONE         BIT(27)
#define BIT_AON_APB_MPLL_DIV_S             BIT(26)
#define BIT_AON_APB_MPLL_MOD_EN            BIT(25)
#define BIT_AON_APB_MPLL_SDM_EN            BIT(24)
#define BIT_AON_APB_MPLL_LPF(x)            (((x) & 0x7) << 20)
#define BIT_AON_APB_MPLL_REFIN(x)          (((x) & 0x3) << 18)
#define BIT_AON_APB_MPLL_IBIAS(x)          (((x) & 0x3) << 16)
#define BIT_AON_APB_MPLL_N(x)              (((x) & 0x7FF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_MPLL_CFG2
// Register Offset : 0x0048
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_MPLL_NINT(x)         (((x) & 0x3F) << 24)
#define BIT_AON_APB_MPLL_KINT(x)         (((x) & 0xFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_DPLL_CFG1
// Register Offset : 0x004C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_DPLL_RES(x)           (((x) & 0x3) << 28)
#define BIT_AON_APB_DPLL_LOCK_DONE        BIT(27)
#define BIT_AON_APB_DPLL_DIV_S            BIT(26)
#define BIT_AON_APB_DPLL_MOD_EN           BIT(25)
#define BIT_AON_APB_DPLL_SDM_EN           BIT(24)
#define BIT_AON_APB_DPLL_LPF(x)           (((x) & 0x7) << 20)
#define BIT_AON_APB_DPLL_REFIN(x)         (((x) & 0x3) << 18)
#define BIT_AON_APB_DPLL_IBIAS(x)         (((x) & 0x3) << 16)
#define BIT_AON_APB_DPLL_N(x)             (((x) & 0x7FF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_DPLL_CFG2
// Register Offset : 0x0050
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_DPLL_NINT(x)         (((x) & 0x3F) << 24)
#define BIT_AON_APB_DPLL_KINT(x)         (((x) & 0xFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_TWPLL_CFG1
// Register Offset : 0x0054
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_TWPLL_RES(x)         (((x) & 0x3) << 28)
#define BIT_AON_APB_TWPLL_LOCK_DONE      BIT(27)
#define BIT_AON_APB_TWPLL_DIV_S          BIT(26)
#define BIT_AON_APB_TWPLL_MOD_EN         BIT(25)
#define BIT_AON_APB_TWPLL_SDM_EN         BIT(24)
#define BIT_AON_APB_TWPLL_LPF(x)         (((x) & 0x7) << 20)
#define BIT_AON_APB_TWPLL_REFIN(x)       (((x) & 0x3) << 18)
#define BIT_AON_APB_TWPLL_IBIAS(x)       (((x) & 0x3) << 16)
#define BIT_AON_APB_TWPLL_N(x)           (((x) & 0x7FF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_TWPLL_CFG2
// Register Offset : 0x0058
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_TWPLL_NINT(x)        (((x) & 0x3F) << 24)
#define BIT_AON_APB_TWPLL_KINT(x)        (((x) & 0xFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_LTEPLL_CFG1
// Register Offset : 0x005C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_LTEPLL_RES(x)        (((x) & 0x3) << 28)
#define BIT_AON_APB_LTEPLL_LOCK_DONE     BIT(27)
#define BIT_AON_APB_LTEPLL_DIV_S         BIT(26)
#define BIT_AON_APB_LTEPLL_MOD_EN        BIT(25)
#define BIT_AON_APB_LTEPLL_SDM_EN        BIT(24)
#define BIT_AON_APB_LTEPLL_LPF(x)        (((x) & 0x7) << 20)
#define BIT_AON_APB_LTEPLL_REFIN(x)      (((x) & 0x3) << 18)
#define BIT_AON_APB_LTEPLL_IBIAS(x)      (((x) & 0x3) << 16)
#define BIT_AON_APB_LTEPLL_N(x)          (((x) & 0x7FF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_LTEPLL_CFG2
// Register Offset : 0x0060
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_LTEPLL_NINT(x)       (((x) & 0x3F) << 24)
#define BIT_AON_APB_LTEPLL_KINT(x)       (((x) & 0xFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_LVDSDISPLL_CFG1
// Register Offset : 0x0064
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_LVDSDISPLL_RES(x)       (((x) & 0xF) << 28)
#define BIT_AON_APB_LVDSDISPLL_LOCK_DONE    BIT(27)
#define BIT_AON_APB_LVDSDISPLL_DIV_S        BIT(26)
#define BIT_AON_APB_LVDSDISPLL_MOD_EN       BIT(25)
#define BIT_AON_APB_LVDSDISPLL_SDM_EN       BIT(24)
#define BIT_AON_APB_LVDSDISPLL_LPF(x)       (((x) & 0x7) << 20)
#define BIT_AON_APB_LVDSDISPLL_REFIN(x)     (((x) & 0x3) << 18)
#define BIT_AON_APB_LVDSDISPLL_IBIAS(x)     (((x) & 0x3) << 16)
#define BIT_AON_APB_LVDSDISPLL_N(x)         (((x) & 0x7FF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_LVDSDISPLL_CFG2
// Register Offset : 0x0068
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_LVDSDISPLL_NINT(x)     (((x) & 0x3F) << 24)
#define BIT_AON_APB_LVDSDISPLL_KINT(x)     (((x) & 0xFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_AON_REG_PROT
// Register Offset : 0x006C
// Description     :Big endian protect register
---------------------------------------------------------------------------*/

#define BIT_AON_APB_LDSP_CTRL_PROT          BIT(31)
#define BIT_AON_APB_REG_PROT_VAL(x)         (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_LDSP_BOOT_EN
// Register Offset : 0x0070
// Description     :DSP boot enable
---------------------------------------------------------------------------*/

#define BIT_AON_APB_FRC_CLK_LDSP_EN        BIT(1)
#define BIT_AON_APB_LDSP_BOOT_EN           BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_LDSP_BOOT_VEC
// Register Offset : 0x0074
// Description     :DSP boot vector
---------------------------------------------------------------------------*/

#define BIT_AON_APB_LDSP_BOOT_VECTOR(x)    (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_LDSP_RST
// Register Offset : 0x0078
// Description     :DSP reset
---------------------------------------------------------------------------*/

#define BIT_AON_APB_LDSP_SYS_SRST          BIT(1)
#define BIT_AON_APB_LDSP_CORE_SRST_N       BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_LDSP_MTX_CTRL1
// Register Offset : 0x007C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_LDSP_MTX_CTRL1(x)     (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_LDSP_MTX_CTRL2
// Register Offset : 0x0080
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_LDSP_MTX_CTRL2(x)     (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_LDSP_MTX_CTRL3
// Register Offset : 0x0084
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_LDSP_MTX_CTRL3(x)     (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_AON_CGM_CFG
// Register Offset : 0x0088
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_PROBE_CKG_DIV(x)       (((x) & 0xF) << 28)
#define BIT_AON_APB_AUX2_CKG_DIV(x)        (((x) & 0xF) << 24)
#define BIT_AON_APB_AUX1_CKG_DIV(x)        (((x) & 0xF) << 20)
#define BIT_AON_APB_AUX0_CKG_DIV(x)        (((x) & 0xF) << 16)
#define BIT_AON_APB_PROBE_CKG_SEL(x)       (((x) & 0xF) << 12)
#define BIT_AON_APB_AUX2_CKG_SEL(x)        (((x) & 0xF) << 8)
#define BIT_AON_APB_AUX1_CKG_SEL(x)        (((x) & 0xF) << 4)
#define BIT_AON_APB_AUX0_CKG_SEL(x)        (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_LACC_MTX_CTRL
// Register Offset : 0x008C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_LACC_MTX_CTRL(x)       (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_CORTEX_MTX_CTRL1
// Register Offset : 0x0090
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_CORTEX_MTX_CTRL1(x)     (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_CORTEX_MTX_CTRL2
// Register Offset : 0x0094
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_CORTEX_MTX_CTRL2(x)     (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_CORTEX_MTX_CTRL3
// Register Offset : 0x0098
// Description     :DSP reset
---------------------------------------------------------------------------*/

#define BIT_AON_APB_CORTEX_MTX_CTRL3(x)     (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_CA5_TCLK_DLY_LEN
// Register Offset : 0x009C
// Description     :APB clock control
---------------------------------------------------------------------------*/

#define BIT_AON_APB_CA5_TCLK_DLY_LEN(x)     (((x) & 0xFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_APB_EB2
// Register Offset : 0x00B0
// Description     : APB_EB2
---------------------------------------------------------------------------*/

#define BIT_AON_APB_AP_DAP_EB          BIT(15)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_AON_CHIP_ID
// Register Offset : 0x00FC
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_AON_CHIP_ID(x)          (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_CCIR_RCVR_CFG
// Register Offset : 0x0100
// Description     :APB clock control
---------------------------------------------------------------------------*/

#define BIT_AON_APB_ANALOG_PLL_RSV(x)       (((x) & 0xFF) << 16)
#define BIT_AON_APB_ANALOG_TESTMUX(x)       (((x) & 0xFF) << 8)
#define BIT_AON_APB_CCIR_SE                 BIT(1)
#define BIT_AON_APB_CCIR_IE                 BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_PLL_BG_CFG
// Register Offset : 0x0108
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_PLL_BG_RSV(x)          (((x) & 0x3) << 4)
#define BIT_AON_APB_PLL_BG_RBIAS_EN        BIT(3)
#define BIT_AON_APB_PLL_BG_PD              BIT(2)
#define BIT_AON_APB_PLL_BG_IEXT_IBEN       BIT(1)
#define BIT_AON_APB_PLL_CON_BG             BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_LVDSDIS_SEL
// Register Offset : 0x010C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_LVDSDIS_LOG_SEL(x)    (((x) & 0x3) << 1)
#define BIT_AON_APB_LVDSDIS_DBG_SEL       BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_DJTAG_MUX_SEL
// Register Offset : 0x0110
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_DJTAG_AON_SEL        BIT(6)
#define BIT_AON_APB_DJTAG_PUB_SEL        BIT(5)
#define BIT_AON_APB_DJTAG_CP1_SEL        BIT(4)
#define BIT_AON_APB_DJTAG_CP0_SEL        BIT(3)
#define BIT_AON_APB_DJTAG_GPU_SEL        BIT(2)
#define BIT_AON_APB_DJTAG_MM_SEL         BIT(1)
#define BIT_AON_APB_DJTAG_AP_SEL         BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_ARM7_SYS_SOFT_RST
// Register Offset : 0x0114
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_ARM7_SYS_SOFT_RST       BIT(4)
#define BIT_AON_APB_ARM7_CORE_SOFT_RST      BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_CP1_CP0_ADDR_MSB
// Register Offset : 0x0118
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_CP1_CP0_ADDR_MSB(x)     (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_AON_DMA_INT_EN
// Register Offset : 0x011C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_AON_DMA_INT_ARM7_EN          BIT(6)
#define BIT_AON_APB_AON_DMA_INT_CP1_DSP_EN       BIT(5)
#define BIT_AON_APB_AON_DMA_INT_CP1_CA5_EN       BIT(4)
#define BIT_AON_APB_AON_DMA_INT_CP0_DSP_1_EN     BIT(3)
#define BIT_AON_APB_AON_DMA_INT_CP0_DSP_0_EN     BIT(2)
#define BIT_AON_APB_AON_DMA_INT_CP0_ARM9_0_EN    BIT(1)
#define BIT_AON_APB_AON_DMA_INT_AP_EN            BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_EMC_AUTO_GATE_EN
// Register Offset : 0x0120
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_CP1_PUB_AUTO_GATE_EN        BIT(19)
#define BIT_AON_APB_CP0_PUB_AUTO_GATE_EN        BIT(18)
#define BIT_AON_APB_AP_PUB_AUTO_GATE_EN         BIT(17)
#define BIT_AON_APB_AON_APB_PUB_AUTO_GATE_EN    BIT(16)
#define BIT_AON_APB_CP1_EMC_AUTO_GATE_EN        BIT(3)
#define BIT_AON_APB_CP0_EMC_AUTO_GATE_EN        BIT(2)
#define BIT_AON_APB_AP_EMC_AUTO_GATE_EN         BIT(1)
#define BIT_AON_APB_CA7_EMC_AUTO_GATE_EN        BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_ARM7_CFG_BUS
// Register Offset : 0x0124
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_ARM7_CFG_BUS_SLEEP         BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_RTC4M_0_CFG
// Register Offset : 0x0128
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_RTC4M0_RSV(x)              (((x) & 0xFF) << 16)
#define BIT_AON_APB_RTC4M0_I_C(x)              (((x) & 0xF) << 8)
#define BIT_AON_APB_RTC4M0_CAL_DONE            BIT(6)
#define BIT_AON_APB_RTC4M0_CAL_START           BIT(5)
#define BIT_AON_APB_RTC4M0_CHOP_EN             BIT(4)
#define BIT_AON_APB_RTC4M0_FORCE_EN            BIT(1)
#define BIT_AON_APB_RTC4M0_AUTO_GATE_EN        BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_RTC4M_1_CFG
// Register Offset : 0x012C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_RTC4M1_RSV(x)              (((x) & 0xFF) << 16)
#define BIT_AON_APB_RTC4M1_I_C(x)              (((x) & 0xF) << 8)
#define BIT_AON_APB_RTC4M1_CAL_DONE            BIT(6)
#define BIT_AON_APB_RTC4M1_CAL_START           BIT(5)
#define BIT_AON_APB_RTC4M1_CHOP_EN             BIT(4)
#define BIT_AON_APB_RTC4M1_FORCE_EN            BIT(1)
#define BIT_AON_APB_RTC4M1_AUTO_GATE_EN        BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_APB_RST2
// Register Offset : 0x0130
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_AON_DJTAG_SOFT_RST         BIT(6)
#define BIT_AON_APB_PUB_DJTAG_SOFT_RST         BIT(5)
#define BIT_AON_APB_GPU_DJTAG_SOFT_RST         BIT(4)
#define BIT_AON_APB_MM_DJTAG_SOFT_RST          BIT(3)
#define BIT_AON_APB_CP1_DJTAG_SOFT_RST         BIT(2)
#define BIT_AON_APB_CP0_DJTAG_SOFT_RST         BIT(1)
#define BIT_AON_APB_AP_DJTAG_SOFT_RST          BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_AP_WPROT_EN1
// Register Offset : 0x3004
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_AP_AWADDR_WPROT_EN1(x)     (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_CP0_WPROT_EN1
// Register Offset : 0x3008
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_CP0_AWADDR_WPROT_EN1(x)    (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_CP1_WPROT_EN1
// Register Offset : 0x300C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_CP1_AWADDR_WPROT_EN1(x)    (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_IO_DLY_CTRL
// Register Offset : 0x3014
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_CLK_CCIR_DLY_SEL(x)        (((x) & 0xF) << 8)
#define BIT_AON_APB_CLK_CP1DSP_DLY_SEL(x)      (((x) & 0xF) << 4)
#define BIT_AON_APB_CLK_CP0DSP_DLY_SEL(x)      (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_AP_WPROT_EN0
// Register Offset : 0x3018
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_AP_AWADDR_WPROT_EN0(x)     (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_CP0_WPROT_EN0
// Register Offset : 0x3020
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_CP0_AWADDR_WPROT_EN0(x)    (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_CP1_WPROT_EN0
// Register Offset : 0x3024
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_CP1_AWADDR_WPROT_EN0(x)     (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_PMU_RST_MONITOR
// Register Offset : 0x302C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_PMU_RST_MONITOR(x)          (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_THM_RST_MONITOR
// Register Offset : 0x3030
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_THM_RST_MONITOR(x)         (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_AP_RST_MONITOR
// Register Offset : 0x3034
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_AP_RST_MONITOR(x)          (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_CA7_RST_MONITOR
// Register Offset : 0x3038
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_CA7_RST_MONITOR(x)        (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_BOND_OPT0
// Register Offset : 0x303C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_BOND_OPTION0(x)          (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_BOND_OPT1
// Register Offset : 0x3040
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_BOND_OPTION1(x)          (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_RES_REG0
// Register Offset : 0x3044
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_ANA_INT_POL_SEL          BIT(7)
#define BIT_AON_APB_XTLBUF_COMB_POL_SEL      BIT(6)
#define BIT_AON_APB_XTLBUF1_COMB_EN          BIT(4)
#define BIT_AON_APB_EXT_XTL3_COMB_EN         BIT(3)
#define BIT_AON_APB_EXT_XTL2_COMB_EN         BIT(2)
#define BIT_AON_APB_EXT_XTL1_COMB_EN         BIT(1)
#define BIT_AON_APB_EXT_XTL0_COMB_EN         BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_RES_REG1
// Register Offset : 0x3048
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_RES_REG1(x)             (((x) & 0xFFFFFFFF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_AON_QOS_CFG
// Register Offset : 0x304C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_QOS_R_GPU(x)           (((x) & 0xF) << 12)
#define BIT_AON_APB_QOS_W_GPU(x)           (((x) & 0xF) << 8)
#define BIT_AON_APB_QOS_R_GSP(x)           (((x) & 0xF) << 4)
#define BIT_AON_APB_QOS_W_GSP(x)           (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_BB_LDO_CAL_START
// Register Offset : 0x3050
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_BB_LDO_CAL_START       BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_AON_MTX_PROT_CFG
// Register Offset : 0x3058
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_HPROT_DMAW(x)         (((x) & 0xF) << 4)
#define BIT_AON_APB_HPROT_DMAR(x)         (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_LVDS_CFG
// Register Offset : 0x3060
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_LVDSDIS_TXCLKDATA(x)       (((x) & 0x7F) << 16)
#define BIT_AON_APB_LVDSDIS_TXCOM(x)           (((x) & 0x3) << 12)
#define BIT_AON_APB_LVDSDIS_TXSLEW(x)          (((x) & 0x3) << 10)
#define BIT_AON_APB_LVDSDIS_TXSW(x)            (((x) & 0x3) << 8)
#define BIT_AON_APB_LVDSDIS_TXRERSER(x)        (((x) & 0x1F) << 3)
#define BIT_AON_APB_LVDSDIS_PRE_EMP(x)         (((x) & 0x3) << 1)
#define BIT_AON_APB_LVDSDIS_TXPD               BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_PLL_LOCK_OUT_SEL
// Register Offset : 0x3064
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_SLEEP_PLLLOCK_SEL         BIT(7)
#define BIT_AON_APB_PLL_LOCK_SEL(x)           (((x) & 0x7) << 4)
#define BIT_AON_APB_SLEEP_DBG_SEL(x)          (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_RTC4M_RC_VAL
// Register Offset : 0x3068
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_RTC4M1_RC_SEL            BIT(31)
#define BIT_AON_APB_RTC4M1_RC_VAL(x)         (((x) & 0x1FF) << 16)
#define BIT_AON_APB_RTC4M0_RC_SEL            BIT(15)
#define BIT_AON_APB_RTC4M0_RC_VAL(x)         (((x) & 0x1FF))

/*---------------------------------------------------------------------------
// Register Name   : REG_AON_APB_AON_APB_RSV
// Register Offset : 0x30F0
// Description     :
---------------------------------------------------------------------------*/

#define BIT_AON_APB_AON_APB_RSV(x)          (((x) & 0xFFFFFFFF))


#endif

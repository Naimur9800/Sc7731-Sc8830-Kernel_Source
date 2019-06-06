/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This file is dual-licensed: you can use it either under the terms
 * of the GPL or the X11 license, at your option. Note that this dual
 * licensing only applies to this file, and not this project as a
 * whole.
 *
 */


#ifndef _SC9830_PMU_APB_GLB_H
#define _SC9830_PMU_APB_GLB_H



#define REG_PMU_APB_PD_CA7_TOP_CFG                       0x0000
#define REG_PMU_APB_PD_CA7_C0_CFG                        0x0004
#define REG_PMU_APB_PD_CA7_C1_CFG                        0x0008
#define REG_PMU_APB_PD_CA7_C2_CFG                        0x000C
#define REG_PMU_APB_PD_CA7_C3_CFG                        0x0010
#define REG_PMU_APB_PD_AP_SYS_CFG                        0x0018
#define REG_PMU_APB_PD_MM_TOP_CFG                        0x001C
#define REG_PMU_APB_PD_GPU_TOP_CFG                       0x0020
#define REG_PMU_APB_PD_CP0_ARM9_0_CFG                    0x0024
#define REG_PMU_APB_PD_CP0_ARM9_1_CFG                    0x0028
#define REG_PMU_APB_PD_CP0_HU3GE_CFG                     0x002C
#define REG_PMU_APB_PD_CP0_GSM_0_CFG                     0x0030
#define REG_PMU_APB_PD_CP0_GSM_1_CFG                     0x0034
#define REG_PMU_APB_PD_CP0_TD_CFG                        0x0038
#define REG_PMU_APB_PD_CP0_CEVA_0_CFG                    0x003C
#define REG_PMU_APB_PD_CP0_CEVA_1_CFG                    0x0040
#define REG_PMU_APB_PD_CP0_SYS_CFG                       0x0044
#define REG_PMU_APB_PD_CP1_CA5_CFG                       0x0048
#define REG_PMU_APB_PD_CP1_LTE_P1_CFG                    0x004C
#define REG_PMU_APB_PD_CP1_LTE_P2_CFG                    0x0050
#define REG_PMU_APB_PD_CP1_CEVA_CFG                      0x0054
#define REG_PMU_APB_PD_CP1_COMWRAP_CFG                   0x0058
#define REG_PMU_APB_PD_PUB_SYS_CFG                       0x005C
#define REG_PMU_APB_AP_WAKEUP_POR_CFG                    0x0060
#define REG_PMU_APB_XTL_WAIT_CNT                         0x0070
#define REG_PMU_APB_XTLBUF_WAIT_CNT                      0x0074
#define REG_PMU_APB_PLL_WAIT_CNT1                        0x0078
#define REG_PMU_APB_PLL_WAIT_CNT2                        0x007C
#define REG_PMU_APB_XTL0_REL_CFG                         0x0080
#define REG_PMU_APB_XTL1_REL_CFG                         0x0084
#define REG_PMU_APB_XTLBUF0_REL_CFG                      0x008C
#define REG_PMU_APB_XTLBUF1_REL_CFG                      0x0090
#define REG_PMU_APB_MPLL_REL_CFG                         0x0094
#define REG_PMU_APB_DPLL_REL_CFG                         0x0098
#define REG_PMU_APB_LTEPLL_REL_CFG                       0x009C
#define REG_PMU_APB_TWPLL_REL_CFG                        0x00A0
#define REG_PMU_APB_LVDSDIS_PLL_REL_CFG                  0x00A4
#define REG_PMU_APB_CP_SOFT_RST                          0x00B0
#define REG_PMU_APB_CP_SLP_STATUS_DBG0                   0x00B4
#define REG_PMU_APB_PWR_STATUS0_DBG                      0x00BC
#define REG_PMU_APB_PWR_STATUS1_DBG                      0x00C0
#define REG_PMU_APB_PWR_STATUS2_DBG                      0x00C4
#define REG_PMU_APB_SLEEP_CTRL                           0x00CC
#define REG_PMU_APB_DDR_SLEEP_CTRL                       0x00D0
#define REG_PMU_APB_SLEEP_STATUS                         0x00D4
#define REG_PMU_APB_CA7_TOP_CFG                          0x00E4
#define REG_PMU_APB_CA7_C0_CFG                           0x00E8
#define REG_PMU_APB_CA7_C1_CFG                           0x00EC
#define REG_PMU_APB_CA7_C2_CFG                           0x00F0
#define REG_PMU_APB_CA7_C3_CFG                           0x00F4
#define REG_PMU_APB_DDR_CHN_SLEEP_CTRL0                  0x00F8
#define REG_PMU_APB_DDR_CHN_SLEEP_CTRL1                  0x00FC
#define REG_PMU_APB_DDR_OP_MODE_CFG                      0x012C
#define REG_PMU_APB_DDR_PHY_RET_CFG                      0x0130
#define REG_PMU_APB_C26M_SEL_CFG                         0x0134
#define REG_PMU_APB_BISR_DONE_STATUS                     0x0138
#define REG_PMU_APB_BISR_BUSY_STATUS                     0x013C
#define REG_PMU_APB_BISR_BYP_CFG                         0x0140
#define REG_PMU_APB_BISR_EN_CFG                          0x0144
#define REG_PMU_APB_CGM_AUTO_GATE_SEL_CFG0               0x0148
#define REG_PMU_APB_CGM_AUTO_GATE_SEL_CFG1               0x014C
#define REG_PMU_APB_CGM_AUTO_GATE_SEL_CFG2               0x0150
#define REG_PMU_APB_CGM_AUTO_GATE_SEL_CFG3               0x0154
#define REG_PMU_APB_CGM_FORCE_EN_CFG0                    0x0158
#define REG_PMU_APB_CGM_FORCE_EN_CFG1                    0x015C
#define REG_PMU_APB_CGM_FORCE_EN_CFG2                    0x0160
#define REG_PMU_APB_CGM_FORCE_EN_CFG3                    0x0164
#define REG_PMU_APB_SLEEP_XTLON_CTRL                     0x0168
#define REG_PMU_APB_MEM_SLP_CFG                          0x016C
#define REG_PMU_APB_MEM_SD_CFG                           0x0170
#define REG_PMU_APB_CA7_CORE_PU_LOCK                     0x0174
#define REG_PMU_APB_ARM7_HOLD_CGM_EN                     0x0178
#define REG_PMU_APB_PWR_CNT_WAIT_CFG0                    0x017C
#define REG_PMU_APB_PWR_CNT_WAIT_CFG1                    0x0180
#define REG_PMU_APB_RC0_REL_CFG                          0x0184
#define REG_PMU_APB_RC1_REL_CFG                          0x0188
#define REG_PMU_APB_RC_CNT_WAIT_CFG                      0x018C
#define REG_PMU_APB_MEM_AUTO_SLP_CFG                     0x0190
#define REG_PMU_APB_MEM_AUTO_SD_CFG                      0x0194
#define REG_PMU_APB_CP0_PD_SHUTDOWN_CFG                  0x0198
#define REG_PMU_APB_CP1_PD_SHUTDOWN_CFG                  0x019C
#define REG_PMU_APB_WAKEUP_LOCK_EN                       0x01A0
#define REG_PMU_APB_PD_CA7_C0_SHUTDOWN_MARK_STATUS       0x3000
#define REG_PMU_APB_PD_CA7_C1_SHUTDOWN_MARK_STATUS       0x3004
#define REG_PMU_APB_PD_CA7_C2_SHUTDOWN_MARK_STATUS       0x3008
#define REG_PMU_APB_PD_CA7_C3_SHUTDOWN_MARK_STATUS       0x300C
#define REG_PMU_APB_PD_CA7_TOP_SHUTDOWN_MARK_STATUS      0x3010
#define REG_PMU_APB_PD_AP_SYS_SHUTDOWN_MARK_STATUS       0x3014
#define REG_PMU_APB_PD_GPU_TOP_SHUTDOWN_MARK_STATUS      0x3018
#define REG_PMU_APB_PD_MM_TOP_SHUTDOWN_MARK_STATUS       0x301C
#define REG_PMU_APB_PD_CP0_ARM9_0_SHUTDOWN_MARK_STATUS   0x3020
#define REG_PMU_APB_PD_CP0_ARM9_1_SHUTDOWN_MARK_STATUS   0x3024
#define REG_PMU_APB_PD_CP0_CEVA_0_SHUTDOWN_MARK_STATUS   0x3028
#define REG_PMU_APB_PD_CP0_CEVA_1_SHUTDOWN_MARK_STATUS   0x302C
#define REG_PMU_APB_PD_CP0_GSM_0_SHUTDOWN_MARK_STATUS    0x3030
#define REG_PMU_APB_PD_CP0_GSM_1_SHUTDOWN_MARK_STATUS    0x3034
#define REG_PMU_APB_PD_CP0_TD_SHUTDOWN_MARK_STATUS       0x3038
#define REG_PMU_APB_PD_CP0_HU3GE_SHUTDOWN_MARK_STATUS    0x303C
#define REG_PMU_APB_PD_CP1_CA5_SHUTDOWN_MARK_STATUS      0x3040
#define REG_PMU_APB_PD_CP1_CEVA_SHUTDOWN_MARK_STATUS     0x3044
#define REG_PMU_APB_PD_CP1_LTE_P1_SHUTDOWN_MARK_STATUS   0x3048
#define REG_PMU_APB_PD_CP1_LTE_P2_SHUTDOWN_MARK_STATUS   0x304C
#define REG_PMU_APB_PD_CP1_COMWRAP_SHUTDOWN_MARK_STATUS  0x3050
#define REG_PMU_APB_PD_PUB_SYS_SHUTDOWN_MARK_STATUS      0x3054

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CA7_TOP_CFG
// Register Offset : 0x0000
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CA7_TOP_DBG_SHUTDOWN_EN           BIT(28)
#define BIT_PMU_APB_PD_CA7_TOP_FORCE_SHUTDOWN            BIT(25)
#define BIT_PMU_APB_PD_CA7_TOP_AUTO_SHUTDOWN_EN          BIT(24)
#define BIT_PMU_APB_PD_CA7_TOP_PWR_ON_DLY(x)             (((x) & 0xFF) << 16)
#define BIT_PMU_APB_PD_CA7_TOP_PWR_ON_SEQ_DLY(x)         (((x) & 0xFF) << 8)
#define BIT_PMU_APB_PD_CA7_TOP_ISO_ON_DLY(x)             (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CA7_C0_CFG
// Register Offset : 0x0004
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CA7_C0_WFI_SHUTDOWN_EN            BIT(29)
#define BIT_PMU_APB_PD_CA7_C0_DBG_SHUTDOWN_EN            BIT(28)
#define BIT_PMU_APB_PD_CA7_C0_FORCE_SHUTDOWN             BIT(25)
#define BIT_PMU_APB_PD_CA7_C0_AUTO_SHUTDOWN_EN           BIT(24)
#define BIT_PMU_APB_PD_CA7_C0_PWR_ON_DLY(x)              (((x) & 0xFF) << 16)
#define BIT_PMU_APB_PD_CA7_C0_PWR_ON_SEQ_DLY(x)          (((x) & 0xFF) << 8)
#define BIT_PMU_APB_PD_CA7_C0_ISO_ON_DLY(x)              (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CA7_C1_CFG
// Register Offset : 0x0008
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CA7_C1_WFI_SHUTDOWN_EN            BIT(29)
#define BIT_PMU_APB_PD_CA7_C1_DBG_SHUTDOWN_EN            BIT(28)
#define BIT_PMU_APB_PD_CA7_C1_FORCE_SHUTDOWN             BIT(25)
#define BIT_PMU_APB_PD_CA7_C1_AUTO_SHUTDOWN_EN           BIT(24)
#define BIT_PMU_APB_PD_CA7_C1_PWR_ON_DLY(x)              (((x) & 0xFF) << 16)
#define BIT_PMU_APB_PD_CA7_C1_PWR_ON_SEQ_DLY(x)          (((x) & 0xFF) << 8)
#define BIT_PMU_APB_PD_CA7_C1_ISO_ON_DLY(x)              (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CA7_C2_CFG
// Register Offset : 0x000C
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CA7_C2_WFI_SHUTDOWN_EN            BIT(29)
#define BIT_PMU_APB_PD_CA7_C2_DBG_SHUTDOWN_EN            BIT(28)
#define BIT_PMU_APB_PD_CA7_C2_FORCE_SHUTDOWN             BIT(25)
#define BIT_PMU_APB_PD_CA7_C2_AUTO_SHUTDOWN_EN           BIT(24)
#define BIT_PMU_APB_PD_CA7_C2_PWR_ON_DLY(x)              (((x) & 0xFF) << 16)
#define BIT_PMU_APB_PD_CA7_C2_PWR_ON_SEQ_DLY(x)          (((x) & 0xFF) << 8)
#define BIT_PMU_APB_PD_CA7_C2_ISO_ON_DLY(x)              (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CA7_C3_CFG
// Register Offset : 0x0010
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CA7_C3_WFI_SHUTDOWN_EN            BIT(29)
#define BIT_PMU_APB_PD_CA7_C3_DBG_SHUTDOWN_EN            BIT(28)
#define BIT_PMU_APB_PD_CA7_C3_FORCE_SHUTDOWN             BIT(25)
#define BIT_PMU_APB_PD_CA7_C3_AUTO_SHUTDOWN_EN           BIT(24)
#define BIT_PMU_APB_PD_CA7_C3_PWR_ON_DLY(x)              (((x) & 0xFF) << 16)
#define BIT_PMU_APB_PD_CA7_C3_PWR_ON_SEQ_DLY(x)          (((x) & 0xFF) << 8)
#define BIT_PMU_APB_PD_CA7_C3_ISO_ON_DLY(x)              (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_AP_SYS_CFG
// Register Offset : 0x0018
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_AP_SYS_FORCE_SHUTDOWN             BIT(25)
#define BIT_PMU_APB_PD_AP_SYS_AUTO_SHUTDOWN_EN           BIT(24)
#define BIT_PMU_APB_PD_AP_SYS_PWR_ON_DLY(x)              (((x) & 0xFF) << 16)
#define BIT_PMU_APB_PD_AP_SYS_PWR_ON_SEQ_DLY(x)          (((x) & 0xFF) << 8)
#define BIT_PMU_APB_PD_AP_SYS_ISO_ON_DLY(x)              (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_MM_TOP_CFG
// Register Offset : 0x001C
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_MM_TOP_FORCE_SHUTDOWN             BIT(25)
#define BIT_PMU_APB_PD_MM_TOP_AUTO_SHUTDOWN_EN           BIT(24)
#define BIT_PMU_APB_PD_MM_TOP_PWR_ON_DLY(x)              (((x) & 0xFF) << 16)
#define BIT_PMU_APB_PD_MM_TOP_PWR_ON_SEQ_DLY(x)          (((x) & 0xFF) << 8)
#define BIT_PMU_APB_PD_MM_TOP_ISO_ON_DLY(x)              (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_GPU_TOP_CFG
// Register Offset : 0x0020
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_GPU_TOP_FORCE_SHUTDOWN            BIT(25)
#define BIT_PMU_APB_PD_GPU_TOP_AUTO_SHUTDOWN_EN          BIT(24)
#define BIT_PMU_APB_PD_GPU_TOP_PWR_ON_DLY(x)             (((x) & 0xFF) << 16)
#define BIT_PMU_APB_PD_GPU_TOP_PWR_ON_SEQ_DLY(x)         (((x) & 0xFF) << 8)
#define BIT_PMU_APB_PD_GPU_TOP_ISO_ON_DLY(x)             (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP0_ARM9_0_CFG
// Register Offset : 0x0024
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP0_ARM9_0_FORCE_SHUTDOWN         BIT(25)
#define BIT_PMU_APB_PD_CP0_ARM9_0_AUTO_SHUTDOWN_EN       BIT(24)
#define BIT_PMU_APB_PD_CP0_ARM9_0_PWR_ON_DLY(x)          (((x) & 0xFF) << 16)
#define BIT_PMU_APB_PD_CP0_ARM9_0_PWR_ON_SEQ_DLY(x)      (((x) & 0xFF) << 8)
#define BIT_PMU_APB_PD_CP0_ARM9_0_ISO_ON_DLY(x)          (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP0_ARM9_1_CFG
// Register Offset : 0x0028
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP0_ARM9_1_FORCE_SHUTDOWN         BIT(25)
#define BIT_PMU_APB_PD_CP0_ARM9_1_AUTO_SHUTDOWN_EN       BIT(24)
#define BIT_PMU_APB_PD_CP0_ARM9_1_PWR_ON_DLY(x)          (((x) & 0xFF) << 16)
#define BIT_PMU_APB_PD_CP0_ARM9_1_PWR_ON_SEQ_DLY(x)      (((x) & 0xFF) << 8)
#define BIT_PMU_APB_PD_CP0_ARM9_1_ISO_ON_DLY(x)          (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP0_HU3GE_CFG
// Register Offset : 0x002C
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP0_HU3GE_FORCE_SHUTDOWN          BIT(25)
#define BIT_PMU_APB_PD_CP0_HU3GE_AUTO_SHUTDOWN_EN        BIT(24)
#define BIT_PMU_APB_PD_CP0_HU3GE_PWR_ON_DLY(x)           (((x) & 0xFF) << 16)
#define BIT_PMU_APB_PD_CP0_HU3GE_PWR_ON_SEQ_DLY(x)       (((x) & 0xFF) << 8)
#define BIT_PMU_APB_PD_CP0_HU3GE_ISO_ON_DLY(x)           (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP0_GSM_0_CFG
// Register Offset : 0x0030
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP0_GSM_0_FORCE_SHUTDOWN          BIT(25)
#define BIT_PMU_APB_PD_CP0_GSM_0_AUTO_SHUTDOWN_EN        BIT(24)
#define BIT_PMU_APB_PD_CP0_GSM_0_PWR_ON_DLY(x)           (((x) & 0xFF) << 16)
#define BIT_PMU_APB_PD_CP0_GSM_0_PWR_ON_SEQ_DLY(x)       (((x) & 0xFF) << 8)
#define BIT_PMU_APB_PD_CP0_GSM_0_ISO_ON_DLY(x)           (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP0_GSM_1_CFG
// Register Offset : 0x0034
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP0_GSM_1_FORCE_SHUTDOWN          BIT(25)
#define BIT_PMU_APB_PD_CP0_GSM_1_AUTO_SHUTDOWN_EN        BIT(24)
#define BIT_PMU_APB_PD_CP0_GSM_1_PWR_ON_DLY(x)           (((x) & 0xFF) << 16)
#define BIT_PMU_APB_PD_CP0_GSM_1_PWR_ON_SEQ_DLY(x)       (((x) & 0xFF) << 8)
#define BIT_PMU_APB_PD_CP0_GSM_1_ISO_ON_DLY(x)           (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP0_TD_CFG
// Register Offset : 0x0038
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP0_TD_FORCE_SHUTDOWN             BIT(25)
#define BIT_PMU_APB_PD_CP0_TD_AUTO_SHUTDOWN_EN           BIT(24)
#define BIT_PMU_APB_PD_CP0_TD_PWR_ON_DLY(x)              (((x) & 0xFF) << 16)
#define BIT_PMU_APB_PD_CP0_TD_PWR_ON_SEQ_DLY(x)          (((x) & 0xFF) << 8)
#define BIT_PMU_APB_PD_CP0_TD_ISO_ON_DLY(x)              (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP0_CEVA_0_CFG
// Register Offset : 0x003C
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP0_CEVA_0_FORCE_SHUTDOWN         BIT(25)
#define BIT_PMU_APB_PD_CP0_CEVA_0_AUTO_SHUTDOWN_EN       BIT(24)
#define BIT_PMU_APB_PD_CP0_CEVA_0_PWR_ON_DLY(x)          (((x) & 0xFF) << 16)
#define BIT_PMU_APB_PD_CP0_CEVA_0_PWR_ON_SEQ_DLY(x)      (((x) & 0xFF) << 8)
#define BIT_PMU_APB_PD_CP0_CEVA_0_ISO_ON_DLY(x)          (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP0_CEVA_1_CFG
// Register Offset : 0x0040
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP0_CEVA_1_FORCE_SHUTDOWN         BIT(25)
#define BIT_PMU_APB_PD_CP0_CEVA_1_AUTO_SHUTDOWN_EN       BIT(24)
#define BIT_PMU_APB_PD_CP0_CEVA_1_PWR_ON_DLY(x)          (((x) & 0xFF) << 16)
#define BIT_PMU_APB_PD_CP0_CEVA_1_PWR_ON_SEQ_DLY(x)      (((x) & 0xFF) << 8)
#define BIT_PMU_APB_PD_CP0_CEVA_1_ISO_ON_DLY(x)          (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP0_SYS_CFG
// Register Offset : 0x0044
// Description     :
-------------------------------------------------------------*/


/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP1_CA5_CFG
// Register Offset : 0x0048
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP1_CA5_FORCE_SHUTDOWN            BIT(25)
#define BIT_PMU_APB_PD_CP1_CA5_AUTO_SHUTDOWN_EN          BIT(24)
#define BIT_PMU_APB_PD_CP1_CA5_PWR_ON_DLY(x)             (((x) & 0xFF) << 16)
#define BIT_PMU_APB_PD_CP1_CA5_PWR_ON_SEQ_DLY(x)         (((x) & 0xFF) << 8)
#define BIT_PMU_APB_PD_CP1_CA5_ISO_ON_DLY(x)             (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP1_LTE_P1_CFG
// Register Offset : 0x004C
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP1_LTE_P1_FORCE_SHUTDOWN         BIT(25)
#define BIT_PMU_APB_PD_CP1_LTE_P1_AUTO_SHUTDOWN_EN       BIT(24)
#define BIT_PMU_APB_PD_CP1_LTE_P1_PWR_ON_DLY(x)          (((x) & 0xFF) << 16)
#define BIT_PMU_APB_PD_CP1_LTE_P1_PWR_ON_SEQ_DLY(x)      (((x) & 0xFF) << 8)
#define BIT_PMU_APB_PD_CP1_LTE_P1_ISO_ON_DLY(x)          (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP1_LTE_P2_CFG
// Register Offset : 0x0050
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP1_LTE_P2_FORCE_SHUTDOWN         BIT(25)
#define BIT_PMU_APB_PD_CP1_LTE_P2_AUTO_SHUTDOWN_EN       BIT(24)
#define BIT_PMU_APB_PD_CP1_LTE_P2_PWR_ON_DLY(x)          (((x) & 0xFF) << 16)
#define BIT_PMU_APB_PD_CP1_LTE_P2_PWR_ON_SEQ_DLY(x)      (((x) & 0xFF) << 8)
#define BIT_PMU_APB_PD_CP1_LTE_P2_ISO_ON_DLY(x)          (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP1_CEVA_CFG
// Register Offset : 0x0054
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP1_CEVA_FORCE_SHUTDOWN           BIT(25)
#define BIT_PMU_APB_PD_CP1_CEVA_AUTO_SHUTDOWN_EN         BIT(24)
#define BIT_PMU_APB_PD_CP1_CEVA_PWR_ON_DLY(x)            (((x) & 0xFF) << 16)
#define BIT_PMU_APB_PD_CP1_CEVA_PWR_ON_SEQ_DLY(x)        (((x) & 0xFF) << 8)
#define BIT_PMU_APB_PD_CP1_CEVA_ISO_ON_DLY(x)            (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP1_COMWRAP_CFG
// Register Offset : 0x0058
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP1_COMWRAP_FORCE_SHUTDOWN        BIT(25)
#define BIT_PMU_APB_PD_CP1_COMWRAP_AUTO_SHUTDOWN_EN      BIT(24)
#define BIT_PMU_APB_PD_CP1_COMWRAP_PWR_ON_DLY(x)         (((x) & 0xFF) << 16)
#define BIT_PMU_APB_PD_CP1_COMWRAP_PWR_ON_SEQ_DLY(x)     (((x) & 0xFF) << 8)
#define BIT_PMU_APB_PD_CP1_COMWRAP_ISO_ON_DLY(x)         (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_PUB_SYS_CFG
// Register Offset : 0x005C
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_PUB_SYS_FORCE_SHUTDOWN            BIT(25)
#define BIT_PMU_APB_PD_PUB_SYS_AUTO_SHUTDOWN_EN          BIT(24)
#define BIT_PMU_APB_PD_PUB_SYS_PWR_ON_DLY(x)             (((x) & 0xFF) << 16)
#define BIT_PMU_APB_PD_PUB_SYS_PWR_ON_SEQ_DLY(x)         (((x) & 0xFF) << 8)
#define BIT_PMU_APB_PD_PUB_SYS_ISO_ON_DLY(x)             (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_AP_WAKEUP_POR_CFG
// Register Offset : 0x0060
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_AP_WAKEUP_POR_N                      BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_XTL_WAIT_CNT
// Register Offset : 0x0070
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_XTL1_WAIT_CNT(x)                     (((x) & 0xFF) << 8)
#define BIT_PMU_APB_XTL0_WAIT_CNT(x)                     (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_XTLBUF_WAIT_CNT
// Register Offset : 0x0074
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_XTLBUF1_WAIT_CNT(x)                  (((x) & 0xFF) << 8)
#define BIT_PMU_APB_XTLBUF0_WAIT_CNT(x)                  (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PLL_WAIT_CNT1
// Register Offset : 0x0078
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_LTEPLL_WAIT_CNT(x)                   (((x) & 0xFF) << 24)
#define BIT_PMU_APB_TWPLL_WAIT_CNT(x)                    (((x) & 0xFF) << 16)
#define BIT_PMU_APB_DPLL_WAIT_CNT(x)                     (((x) & 0xFF) << 8)
#define BIT_PMU_APB_MPLL_WAIT_CNT(x)                     (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PLL_WAIT_CNT2
// Register Offset : 0x007C
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_LVDSDIS_PLL_WAIT_CNT(x)              (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_XTL0_REL_CFG
// Register Offset : 0x0080
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_XTL0_ARM7_SEL                        BIT(5)
#define BIT_PMU_APB_XTL0_VCP1_SEL                        BIT(4)
#define BIT_PMU_APB_XTL0_VCP0_SEL                        BIT(3)
#define BIT_PMU_APB_XTL0_CP1_SEL                         BIT(2)
#define BIT_PMU_APB_XTL0_CP0_SEL                         BIT(1)
#define BIT_PMU_APB_XTL0_AP_SEL                          BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_XTL1_REL_CFG
// Register Offset : 0x0084
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_XTL1_ARM7_SEL                        BIT(5)
#define BIT_PMU_APB_XTL1_VCP1_SEL                        BIT(4)
#define BIT_PMU_APB_XTL1_VCP0_SEL                        BIT(3)
#define BIT_PMU_APB_XTL1_CP1_SEL                         BIT(2)
#define BIT_PMU_APB_XTL1_CP0_SEL                         BIT(1)
#define BIT_PMU_APB_XTL1_AP_SEL                          BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_XTLBUF0_REL_CFG
// Register Offset : 0x008C
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_XTLBUF0_ARM7_SEL                     BIT(5)
#define BIT_PMU_APB_XTLBUF0_VCP1_SEL                     BIT(4)
#define BIT_PMU_APB_XTLBUF0_VCP0_SEL                     BIT(3)
#define BIT_PMU_APB_XTLBUF0_CP1_SEL                      BIT(2)
#define BIT_PMU_APB_XTLBUF0_CP0_SEL                      BIT(1)
#define BIT_PMU_APB_XTLBUF0_AP_SEL                       BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_XTLBUF1_REL_CFG
// Register Offset : 0x0090
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_XTLBUF1_ARM7_SEL                     BIT(5)
#define BIT_PMU_APB_XTLBUF1_VCP1_SEL                     BIT(4)
#define BIT_PMU_APB_XTLBUF1_VCP0_SEL                     BIT(3)
#define BIT_PMU_APB_XTLBUF1_CP1_SEL                      BIT(2)
#define BIT_PMU_APB_XTLBUF1_CP0_SEL                      BIT(1)
#define BIT_PMU_APB_XTLBUF1_AP_SEL                       BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_MPLL_REL_CFG
// Register Offset : 0x0094
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_MPLL_REF_SEL                         BIT(8)
#define BIT_PMU_APB_MPLL_ARM7_SEL                        BIT(5)
#define BIT_PMU_APB_MPLL_VCP1_SEL                        BIT(4)
#define BIT_PMU_APB_MPLL_VCP0_SEL                        BIT(3)
#define BIT_PMU_APB_MPLL_CP1_SEL                         BIT(2)
#define BIT_PMU_APB_MPLL_CP0_SEL                         BIT(1)
#define BIT_PMU_APB_MPLL_AP_SEL                          BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_DPLL_REL_CFG
// Register Offset : 0x0098
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_DPLL_REF_SEL                         BIT(8)
#define BIT_PMU_APB_DPLL_ARM7_SEL                        BIT(5)
#define BIT_PMU_APB_DPLL_VCP1_SEL                        BIT(4)
#define BIT_PMU_APB_DPLL_VCP0_SEL                        BIT(3)
#define BIT_PMU_APB_DPLL_CP1_SEL                         BIT(2)
#define BIT_PMU_APB_DPLL_CP0_SEL                         BIT(1)
#define BIT_PMU_APB_DPLL_AP_SEL                          BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_LTEPLL_REL_CFG
// Register Offset : 0x009C
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_LTEPLL_REF_SEL                       BIT(8)
#define BIT_PMU_APB_LTEPLL_ARM7_SEL                      BIT(5)
#define BIT_PMU_APB_LTEPLL_VCP1_SEL                      BIT(4)
#define BIT_PMU_APB_LTEPLL_VCP0_SEL                      BIT(3)
#define BIT_PMU_APB_LTEPLL_CP1_SEL                       BIT(2)
#define BIT_PMU_APB_LTEPLL_CP0_SEL                       BIT(1)
#define BIT_PMU_APB_LTEPLL_AP_SEL                        BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_TWPLL_REL_CFG
// Register Offset : 0x00A0
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_TWPLL_REF_SEL                        BIT(8)
#define BIT_PMU_APB_TWPLL_ARM7_SEL                       BIT(5)
#define BIT_PMU_APB_TWPLL_VCP1_SEL                       BIT(4)
#define BIT_PMU_APB_TWPLL_VCP0_SEL                       BIT(3)
#define BIT_PMU_APB_TWPLL_CP1_SEL                        BIT(2)
#define BIT_PMU_APB_TWPLL_CP0_SEL                        BIT(1)
#define BIT_PMU_APB_TWPLL_AP_SEL                         BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_LVDSDIS_PLL_REL_CFG
// Register Offset : 0x00A4
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_LVDSDIS_PLL_REF_SEL                  BIT(8)
#define BIT_PMU_APB_LVDSDIS_PLL_ARM7_SEL                 BIT(5)
#define BIT_PMU_APB_LVDSDIS_PLL_VCP1_SEL                 BIT(4)
#define BIT_PMU_APB_LVDSDIS_PLL_VCP0_SEL                 BIT(3)
#define BIT_PMU_APB_LVDSDIS_PLL_CP1_SEL                  BIT(2)
#define BIT_PMU_APB_LVDSDIS_PLL_CP0_SEL                  BIT(1)
#define BIT_PMU_APB_LVDSDIS_PLL_AP_SEL                   BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_CP_SOFT_RST
// Register Offset : 0x00B0
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_ARM7_SOFT_RST                        BIT(8)
#define BIT_PMU_APB_PUB_SOFT_RST                         BIT(6)
#define BIT_PMU_APB_AP_SOFT_RST                          BIT(5)
#define BIT_PMU_APB_GPU_SOFT_RST                         BIT(4)
#define BIT_PMU_APB_MM_SOFT_RST                          BIT(3)
#define BIT_PMU_APB_CP1_SOFT_RST                         BIT(1)
#define BIT_PMU_APB_CP0_SOFT_RST                         BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_CP_SLP_STATUS_DBG0
// Register Offset : 0x00B4
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_CP1_DEEP_SLP_DBG(x)                  (((x) & 0xFFFF) << 16)
#define BIT_PMU_APB_CP0_DEEP_SLP_DBG(x)                  (((x) & 0xFFFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PWR_STATUS0_DBG
// Register Offset : 0x00BC
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_MM_TOP_STATE(x)                   (((x) & 0xF) << 28)
#define BIT_PMU_APB_PD_GPU_TOP_STATE(x)                  (((x) & 0xF) << 24)
#define BIT_PMU_APB_PD_AP_SYS_STATE(x)                   (((x) & 0xF) << 20)
#define BIT_PMU_APB_PD_CA7_C3_STATE(x)                   (((x) & 0xF) << 16)
#define BIT_PMU_APB_PD_CA7_C2_STATE(x)                   (((x) & 0xF) << 12)
#define BIT_PMU_APB_PD_CA7_C1_STATE(x)                   (((x) & 0xF) << 8)
#define BIT_PMU_APB_PD_CA7_C0_STATE(x)                   (((x) & 0xF) << 4)
#define BIT_PMU_APB_PD_CA7_TOP_STATE(x)                  (((x) & 0xF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PWR_STATUS1_DBG
// Register Offset : 0x00C0
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP0_CEVA_1_STATE(x)               (((x) & 0xF) << 28)
#define BIT_PMU_APB_PD_CP0_CEVA_0_STATE(x)               (((x) & 0xF) << 24)
#define BIT_PMU_APB_PD_CP0_GSM_0_STATE(x)                (((x) & 0xF) << 20)
#define BIT_PMU_APB_PD_CP0_GSM_1_STATE(x)                (((x) & 0xF) << 16)
#define BIT_PMU_APB_PD_CP0_HU3GE_STATE(x)                (((x) & 0xF) << 12)
#define BIT_PMU_APB_PD_CP0_ARM9_1_STATE(x)               (((x) & 0xF) << 8)
#define BIT_PMU_APB_PD_CP0_ARM9_0_STATE(x)               (((x) & 0xF) << 4)
#define BIT_PMU_APB_PD_CP0_TD_STATE(x)                   (((x) & 0xF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PWR_STATUS2_DBG
// Register Offset : 0x00C4
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_PUB_SYS_STATE(x)                  (((x) & 0xF) << 24)
#define BIT_PMU_APB_PD_CP1_COMWRAP_STATE(x)              (((x) & 0xF) << 20)
#define BIT_PMU_APB_PD_CP1_LTE_P2_STATE(x)               (((x) & 0xF) << 16)
#define BIT_PMU_APB_PD_CP1_LTE_P1_STATE(x)               (((x) & 0xF) << 12)
#define BIT_PMU_APB_PD_CP1_CEVA_STATE(x)                 (((x) & 0xF) << 8)
#define BIT_PMU_APB_PD_CP1_CA5_STATE(x)                  (((x) & 0xF) << 4)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_SLEEP_CTRL
// Register Offset : 0x00CC
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_VCP1_FORCE_LIGHT_SLEEP               BIT(28)
#define BIT_PMU_APB_VCP0_FORCE_LIGHT_SLEEP               BIT(27)
#define BIT_PMU_APB_CP1_FORCE_LIGHT_SLEEP                BIT(26)
#define BIT_PMU_APB_CP0_FORCE_LIGHT_SLEEP                BIT(25)
#define BIT_PMU_APB_AP_FORCE_LIGHT_SLEEP                 BIT(24)
#define BIT_PMU_APB_ARM7_FORCE_DEEP_SLEEP                BIT(21)
#define BIT_PMU_APB_VCP1_FORCE_DEEP_SLEEP                BIT(20)
#define BIT_PMU_APB_VCP0_FORCE_DEEP_SLEEP                BIT(19)
#define BIT_PMU_APB_CP1_FORCE_DEEP_SLEEP                 BIT(18)
#define BIT_PMU_APB_CP0_FORCE_DEEP_SLEEP                 BIT(17)
#define BIT_PMU_APB_AP_FORCE_DEEP_SLEEP                  BIT(16)
#define BIT_PMU_APB_VCP1_LIGHT_SLEEP                     BIT(12)
#define BIT_PMU_APB_VCP0_LIGHT_SLEEP                     BIT(11)
#define BIT_PMU_APB_CP1_LIGHT_SLEEP                      BIT(10)
#define BIT_PMU_APB_CP0_LIGHT_SLEEP                      BIT(9)
#define BIT_PMU_APB_AP_LIGHT_SLEEP                       BIT(8)
#define BIT_PMU_APB_VCP1_DEEP_SLEEP                      BIT(4)
#define BIT_PMU_APB_VCP0_DEEP_SLEEP                      BIT(3)
#define BIT_PMU_APB_CP1_DEEP_SLEEP                       BIT(2)
#define BIT_PMU_APB_CP0_DEEP_SLEEP                       BIT(1)
#define BIT_PMU_APB_AP_DEEP_SLEEP                        BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_DDR_SLEEP_CTRL
// Register Offset : 0x00D0
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_BUSY_TRANSFER_HWDATA_SEL             BIT(16)
#define BIT_PMU_APB_DDR_PUBL_APB_SOFT_RST                BIT(12)
#define BIT_PMU_APB_DDR_UMCTL_APB_SOFT_RST               BIT(11)
#define BIT_PMU_APB_DDR_PUBL_SOFT_RST                    BIT(10)
#define BIT_PMU_APB_DDR_PHY_SOFT_RST                     BIT(8)
#define BIT_PMU_APB_DDR_PHY_AUTO_GATE_EN                 BIT(6)
#define BIT_PMU_APB_DDR_PUBL_AUTO_GATE_EN                BIT(5)
#define BIT_PMU_APB_DDR_UMCTL_AUTO_GATE_EN               BIT(4)
#define BIT_PMU_APB_DDR_PHY_EB                           BIT(2)
#define BIT_PMU_APB_DDR_UMCTL_EB                         BIT(1)
#define BIT_PMU_APB_DDR_PUBL_EB                          BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_SLEEP_STATUS
// Register Offset : 0x00D4
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_ARM7_SLP_STATUS(x)                   (((x) & 0xF) << 20)
#define BIT_PMU_APB_VCP1_SLP_STATUS(x)                   (((x) & 0xF) << 16)
#define BIT_PMU_APB_VCP0_SLP_STATUS(x)                   (((x) & 0xF) << 12)
#define BIT_PMU_APB_CP1_SLP_STATUS(x)                    (((x) & 0xF) << 8)
#define BIT_PMU_APB_CP0_SLP_STATUS(x)                    (((x) & 0xF) << 4)
#define BIT_PMU_APB_AP_SLP_STATUS(x)                     (((x) & 0xF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_CA7_TOP_CFG
// Register Offset : 0x00E4
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_CA7_L2RSTDISABLE                     BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_CA7_C0_CFG
// Register Offset : 0x00E8
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_CA7_VINITHI_C0                       BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_CA7_C1_CFG
// Register Offset : 0x00EC
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_CA7_VINITHI_C1                       BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_CA7_C2_CFG
// Register Offset : 0x00F0
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_CA7_VINITHI_C2                       BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_CA7_C3_CFG
// Register Offset : 0x00F4
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_CA7_VINITHI_C3                       BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_DDR_CHN_SLEEP_CTRL0
// Register Offset : 0x00F8
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_DDR_CTRL_AXI_LP_EN                   BIT(31)
#define BIT_PMU_APB_DDR_CTRL_CGM_SEL                     BIT(30)
#define BIT_PMU_APB_DDR_CHN9_AXI_LP_EN                   BIT(25)
#define BIT_PMU_APB_DDR_CHN8_AXI_LP_EN                   BIT(24)
#define BIT_PMU_APB_DDR_CHN7_AXI_LP_EN                   BIT(23)
#define BIT_PMU_APB_DDR_CHN6_AXI_LP_EN                   BIT(22)
#define BIT_PMU_APB_DDR_CHN5_AXI_LP_EN                   BIT(21)
#define BIT_PMU_APB_DDR_CHN4_AXI_LP_EN                   BIT(20)
#define BIT_PMU_APB_DDR_CHN3_AXI_LP_EN                   BIT(19)
#define BIT_PMU_APB_DDR_CHN2_AXI_LP_EN                   BIT(18)
#define BIT_PMU_APB_DDR_CHN1_AXI_LP_EN                   BIT(17)
#define BIT_PMU_APB_DDR_CHN0_AXI_LP_EN                   BIT(16)
#define BIT_PMU_APB_DDR_CHN9_CGM_SEL                     BIT(9)
#define BIT_PMU_APB_DDR_CHN8_CGM_SEL                     BIT(8)
#define BIT_PMU_APB_DDR_CHN7_CGM_SEL                     BIT(7)
#define BIT_PMU_APB_DDR_CHN6_CGM_SEL                     BIT(6)
#define BIT_PMU_APB_DDR_CHN5_CGM_SEL                     BIT(5)
#define BIT_PMU_APB_DDR_CHN4_CGM_SEL                     BIT(4)
#define BIT_PMU_APB_DDR_CHN3_CGM_SEL                     BIT(3)
#define BIT_PMU_APB_DDR_CHN2_CGM_SEL                     BIT(2)
#define BIT_PMU_APB_DDR_CHN1_CGM_SEL                     BIT(1)
#define BIT_PMU_APB_DDR_CHN0_CGM_SEL                     BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_DDR_CHN_SLEEP_CTRL1
// Register Offset : 0x00FC
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_DDR_CHN9_AXI_STOP_SEL                BIT(9)
#define BIT_PMU_APB_DDR_CHN8_AXI_STOP_SEL                BIT(8)
#define BIT_PMU_APB_DDR_CHN7_AXI_STOP_SEL                BIT(7)
#define BIT_PMU_APB_DDR_CHN6_AXI_STOP_SEL                BIT(6)
#define BIT_PMU_APB_DDR_CHN5_AXI_STOP_SEL                BIT(5)
#define BIT_PMU_APB_DDR_CHN4_AXI_STOP_SEL                BIT(4)
#define BIT_PMU_APB_DDR_CHN3_AXI_STOP_SEL                BIT(3)
#define BIT_PMU_APB_DDR_CHN2_AXI_STOP_SEL                BIT(2)
#define BIT_PMU_APB_DDR_CHN1_AXI_STOP_SEL                BIT(1)
#define BIT_PMU_APB_DDR_CHN0_AXI_STOP_SEL                BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_DDR_OP_MODE_CFG
// Register Offset : 0x012C
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_DDR_OPERATE_MODE_BUSY                BIT(28)
#define BIT_PMU_APB_DDR_PUBL_RET_EN                      BIT(27)
#define BIT_PMU_APB_DDR_PHY_ISO_RST_EN                   BIT(26)
#define BIT_PMU_APB_DDR_UMCTL_RET_EN                     BIT(25)
#define BIT_PMU_APB_DDR_PHY_AUTO_RET_EN                  BIT(24)
#define BIT_PMU_APB_DDR_OPERATE_MODE_CNT_LMT(x)          (((x) & 0xFF) << 16)
#define BIT_PMU_APB_DDR_OPERATE_MODE(x)                  (((x) & 0x7) << 8)
#define BIT_PMU_APB_DDR_OPERATE_MODE_IDLE(x)             (((x) & 0x7))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_DDR_PHY_RET_CFG
// Register Offset : 0x0130
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_DDR_UMCTL_SOFT_RST                   BIT(16)
#define BIT_PMU_APB_DDR_PHY_CKE_RET_EN                   BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_C26M_SEL_CFG
// Register Offset : 0x0134
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_AON_RC_4M_SEL                        BIT(8)
#define BIT_PMU_APB_GGE_26M_SEL                          BIT(6)
#define BIT_PMU_APB_PUB_26M_SEL                          BIT(5)
#define BIT_PMU_APB_AON_26M_SEL                          BIT(4)
#define BIT_PMU_APB_CP1_26M_SEL                          BIT(2)
#define BIT_PMU_APB_CP0_26M_SEL                          BIT(1)
#define BIT_PMU_APB_AP_26M_SEL                           BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_BISR_DONE_STATUS
// Register Offset : 0x0138
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP1_COMWRAP_BISR_DONE             BIT(21)
#define BIT_PMU_APB_PD_CP1_LTE_P2_BISR_DONE              BIT(20)
#define BIT_PMU_APB_PD_CP1_LTE_P1_BISR_DONE              BIT(19)
#define BIT_PMU_APB_PD_CP1_CEVA_BISR_DONE                BIT(18)
#define BIT_PMU_APB_PD_CP1_CA5_BISR_DONE                 BIT(17)
#define BIT_PMU_APB_PD_CP0_HU3GE_BISR_DONE               BIT(15)
#define BIT_PMU_APB_PD_CP0_TD_BISR_DONE                  BIT(14)
#define BIT_PMU_APB_PD_CP0_GSM_1_BISR_DONE               BIT(13)
#define BIT_PMU_APB_PD_CP0_GSM_0_BISR_DONE               BIT(12)
#define BIT_PMU_APB_PD_CP0_CEVA_1_BISR_DONE              BIT(11)
#define BIT_PMU_APB_PD_CP0_CEVA_0_BISR_DONE              BIT(10)
#define BIT_PMU_APB_PD_CP0_ARM9_1_BISR_DONE              BIT(9)
#define BIT_PMU_APB_PD_CP0_ARM9_0_BISR_DONE              BIT(8)
#define BIT_PMU_APB_PD_MM_TOP_BISR_DONE                  BIT(7)
#define BIT_PMU_APB_PD_GPU_TOP_BISR_DONE                 BIT(6)
#define BIT_PMU_APB_PD_AP_SYS_BISR_DONE                  BIT(5)
#define BIT_PMU_APB_PD_CA7_TOP_BISR_DONE                 BIT(4)
#define BIT_PMU_APB_PD_CA7_C3_BISR_DONE                  BIT(3)
#define BIT_PMU_APB_PD_CA7_C2_BISR_DONE                  BIT(2)
#define BIT_PMU_APB_PD_CA7_C1_BISR_DONE                  BIT(1)
#define BIT_PMU_APB_PD_CA7_C0_BISR_DONE                  BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_BISR_BUSY_STATUS
// Register Offset : 0x013C
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP1_COMWRAP_BISR_BUSY             BIT(21)
#define BIT_PMU_APB_PD_CP1_LTE_P2_BISR_BUSY              BIT(20)
#define BIT_PMU_APB_PD_CP1_LTE_P1_BISR_BUSY              BIT(19)
#define BIT_PMU_APB_PD_CP1_CEVA_BISR_BUSY                BIT(18)
#define BIT_PMU_APB_PD_CP1_CA5_BISR_BUSY                 BIT(17)
#define BIT_PMU_APB_PD_CP0_HU3GE_BISR_BUSY               BIT(15)
#define BIT_PMU_APB_PD_CP0_TD_BISR_BUSY                  BIT(14)
#define BIT_PMU_APB_PD_CP0_GSM_1_BISR_BUSY               BIT(13)
#define BIT_PMU_APB_PD_CP0_GSM_0_BISR_BUSY               BIT(12)
#define BIT_PMU_APB_PD_CP0_CEVA_1_BISR_BUSY              BIT(11)
#define BIT_PMU_APB_PD_CP0_CEVA_0_BISR_BUSY              BIT(10)
#define BIT_PMU_APB_PD_CP0_ARM9_1_BISR_BUSY              BIT(9)
#define BIT_PMU_APB_PD_CP0_ARM9_0_BISR_BUSY              BIT(8)
#define BIT_PMU_APB_PD_MM_TOP_BISR_BUSY                  BIT(7)
#define BIT_PMU_APB_PD_GPU_TOP_BISR_BUSY                 BIT(6)
#define BIT_PMU_APB_PD_AP_SYS_BISR_BUSY                  BIT(5)
#define BIT_PMU_APB_PD_CA7_TOP_BISR_BUSY                 BIT(4)
#define BIT_PMU_APB_PD_CA7_C3_BISR_BUSY                  BIT(3)
#define BIT_PMU_APB_PD_CA7_C2_BISR_BUSY                  BIT(2)
#define BIT_PMU_APB_PD_CA7_C1_BISR_BUSY                  BIT(1)
#define BIT_PMU_APB_PD_CA7_C0_BISR_BUSY                  BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_BISR_BYP_CFG
// Register Offset : 0x0140
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP1_COMWRAP_BISR_FORCE_BYP        BIT(21)
#define BIT_PMU_APB_PD_CP1_LTE_P2_BISR_FORCE_BYP         BIT(20)
#define BIT_PMU_APB_PD_CP1_LTE_P1_BISR_FORCE_BYP         BIT(19)
#define BIT_PMU_APB_PD_CP1_CEVA_BISR_FORCE_BYP           BIT(18)
#define BIT_PMU_APB_PD_CP1_CA5_BISR_FORCE_BYP            BIT(17)
#define BIT_PMU_APB_PD_CP0_HU3GE_BISR_FORCE_BYP          BIT(15)
#define BIT_PMU_APB_PD_CP0_TD_BISR_FORCE_BYP             BIT(14)
#define BIT_PMU_APB_PD_CP0_GSM_1_BISR_FORCE_BYP          BIT(13)
#define BIT_PMU_APB_PD_CP0_GSM_0_BISR_FORCE_BYP          BIT(12)
#define BIT_PMU_APB_PD_CP0_CEVA_1_BISR_FORCE_BYP         BIT(11)
#define BIT_PMU_APB_PD_CP0_CEVA_0_BISR_FORCE_BYP         BIT(10)
#define BIT_PMU_APB_PD_CP0_ARM9_1_BISR_FORCE_BYP         BIT(9)
#define BIT_PMU_APB_PD_CP0_ARM9_0_BISR_FORCE_BYP         BIT(8)
#define BIT_PMU_APB_PD_MM_TOP_BISR_FORCE_BYP             BIT(7)
#define BIT_PMU_APB_PD_GPU_TOP_BISR_FORCE_BYP            BIT(6)
#define BIT_PMU_APB_PD_AP_SYS_BISR_FORCE_BYP             BIT(5)
#define BIT_PMU_APB_PD_CA7_TOP_BISR_FORCE_BYP            BIT(4)
#define BIT_PMU_APB_PD_CA7_C3_BISR_FORCE_BYP             BIT(3)
#define BIT_PMU_APB_PD_CA7_C2_BISR_FORCE_BYP             BIT(2)
#define BIT_PMU_APB_PD_CA7_C1_BISR_FORCE_BYP             BIT(1)
#define BIT_PMU_APB_PD_CA7_C0_BISR_FORCE_BYP             BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_BISR_EN_CFG
// Register Offset : 0x0144
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP1_COMWRAP_BISR_FORCE_EN         BIT(21)
#define BIT_PMU_APB_PD_CP1_LTE_P2_BISR_FORCE_EN          BIT(20)
#define BIT_PMU_APB_PD_CP1_LTE_P1_BISR_FORCE_EN          BIT(19)
#define BIT_PMU_APB_PD_CP1_CEVA_BISR_FORCE_EN            BIT(18)
#define BIT_PMU_APB_PD_CP1_CA5_BISR_FORCE_EN             BIT(17)
#define BIT_PMU_APB_PD_CP0_HU3GE_BISR_FORCE_EN           BIT(15)
#define BIT_PMU_APB_PD_CP0_TD_BISR_FORCE_EN              BIT(14)
#define BIT_PMU_APB_PD_CP0_GSM_1_BISR_FORCE_EN           BIT(13)
#define BIT_PMU_APB_PD_CP0_GSM_0_BISR_FORCE_EN           BIT(12)
#define BIT_PMU_APB_PD_CP0_CEVA_1_BISR_FORCE_EN          BIT(11)
#define BIT_PMU_APB_PD_CP0_CEVA_0_BISR_FORCE_EN          BIT(10)
#define BIT_PMU_APB_PD_CP0_ARM9_1_BISR_FORCE_EN          BIT(9)
#define BIT_PMU_APB_PD_CP0_ARM9_0_BISR_FORCE_EN          BIT(8)
#define BIT_PMU_APB_PD_MM_TOP_BISR_FORCE_EN              BIT(7)
#define BIT_PMU_APB_PD_GPU_TOP_BISR_FORCE_EN             BIT(6)
#define BIT_PMU_APB_PD_AP_SYS_BISR_FORCE_EN              BIT(5)
#define BIT_PMU_APB_PD_CA7_TOP_BISR_FORCE_EN             BIT(4)
#define BIT_PMU_APB_PD_CA7_C3_BISR_FORCE_EN              BIT(3)
#define BIT_PMU_APB_PD_CA7_C2_BISR_FORCE_EN              BIT(2)
#define BIT_PMU_APB_PD_CA7_C1_BISR_FORCE_EN              BIT(1)
#define BIT_PMU_APB_PD_CA7_C0_BISR_FORCE_EN              BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_CGM_AUTO_GATE_SEL_CFG0
// Register Offset : 0x0148
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_LTEPLL_DIV_61M44_AUTO_GATE_SEL       BIT(31)
#define BIT_PMU_APB_LTEPLL_DIV_122M88_AUTO_GATE_SEL      BIT(30)
#define BIT_PMU_APB_LTEPLL_DIV_245M76_AUTO_GATE_SEL      BIT(29)
#define BIT_PMU_APB_LTEPLL_DIV_68M26_AUTO_GATE_SEL       BIT(28)
#define BIT_PMU_APB_LTEPLL_DIV_102M4_AUTO_GATE_SEL       BIT(27)
#define BIT_PMU_APB_LTEPLL_DIV_204M8_AUTO_GATE_SEL       BIT(26)
#define BIT_PMU_APB_LTEPLL_DIV_409M6_AUTO_GATE_SEL       BIT(25)
#define BIT_PMU_APB_LTEPLL_DIV_153M6_AUTO_GATE_SEL       BIT(24)
#define BIT_PMU_APB_LTEPLL_DIV_307M2_AUTO_GATE_SEL       BIT(23)
#define BIT_PMU_APB_LTEPLL_DIV_614M4_AUTO_GATE_SEL       BIT(22)
#define BIT_PMU_APB_TWPLL_DIV_5M12_AUTO_GATE_SEL         BIT(21)
#define BIT_PMU_APB_TWPLL_DIV_10M24_AUTO_GATE_SEL        BIT(20)
#define BIT_PMU_APB_TWPLL_DIV_38M4_AUTO_GATE_SEL         BIT(19)
#define BIT_PMU_APB_TWPLL_DIV_76M8_AUTO_GATE_SEL         BIT(18)
#define BIT_PMU_APB_TWPLL_DIV_51M2_AUTO_GATE_SEL         BIT(17)
#define BIT_PMU_APB_TWPLL_DIV_64M_AUTO_GATE_SEL          BIT(16)
#define BIT_PMU_APB_TWPLL_DIV_153M6_AUTO_GATE_SEL        BIT(15)
#define BIT_PMU_APB_TWPLL_DIV_128M_AUTO_GATE_SEL         BIT(14)
#define BIT_PMU_APB_TWPLL_DIV_256M_AUTO_GATE_SEL         BIT(13)
#define BIT_PMU_APB_TWPLL_DIV_12M_AUTO_GATE_SEL          BIT(12)
#define BIT_PMU_APB_TWPLL_DIV_24M_AUTO_GATE_SEL          BIT(11)
#define BIT_PMU_APB_TWPLL_DIV_48M_AUTO_GATE_SEL          BIT(10)
#define BIT_PMU_APB_TWPLL_DIV_96M_AUTO_GATE_SEL          BIT(9)
#define BIT_PMU_APB_TWPLL_DIV_102M4_AUTO_GATE_SEL        BIT(8)
#define BIT_PMU_APB_TWPLL_DIV_192M_AUTO_GATE_SEL         BIT(7)
#define BIT_PMU_APB_TWPLL_DIV_219M4_AUTO_GATE_SEL        BIT(6)
#define BIT_PMU_APB_TWPLL_DIV_307M2_AUTO_GATE_SEL        BIT(5)
#define BIT_PMU_APB_TWPLL_DIV_384M_AUTO_GATE_SEL         BIT(4)
#define BIT_PMU_APB_TWPLL_DIV_512M_AUTO_GATE_SEL         BIT(3)
#define BIT_PMU_APB_TWPLL_DIV_768M_AUTO_GATE_SEL         BIT(2)
#define BIT_PMU_APB_DPLL_DIV_33M_AUTO_GATE_SEL           BIT(1)
#define BIT_PMU_APB_MPLL_DIV_50M_AUTO_GATE_SEL           BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_CGM_AUTO_GATE_SEL_CFG1
// Register Offset : 0x014C
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_CGM_TWPLL_128M_MM_AUTO_GATE_SEL_CFG1 BIT(31)
#define BIT_PMU_APB_CGM_TWPLL_153M6_MM_AUTO_GATE_SEL     BIT(30)
#define BIT_PMU_APB_CGM_TWPLL_192M_MM_AUTO_GATE_SEL      BIT(29)
#define BIT_PMU_APB_CGM_TWPLL_256M_MM_AUTO_GATE_SEL      BIT(28)
#define BIT_PMU_APB_CGM_TWPLL_307M2_MM_AUTO_GATE_SEL     BIT(27)
#define BIT_PMU_APB_CGM_26M_MM_AUTO_GATE_SEL             BIT(26)
#define BIT_PMU_APB_CGM_TWPLL_12M_AP_AUTO_GATE_SEL       BIT(25)
#define BIT_PMU_APB_CGM_TWPLL_24M_AP_AUTO_GATE_SEL       BIT(24)
#define BIT_PMU_APB_CGM_TWPLL_48M_AP_AUTO_GATE_SEL       BIT(23)
#define BIT_PMU_APB_CGM_TWPLL_51M2_AP_AUTO_GATE_SEL      BIT(22)
#define BIT_PMU_APB_CGM_TWPLL_64M_AP_AUTO_GATE_SEL       BIT(21)
#define BIT_PMU_APB_CGM_TWPLL_76M8_AP_AUTO_GATE_SEL      BIT(20)
#define BIT_PMU_APB_CGM_TWPLL_96M_AP_AUTO_GATE_SEL       BIT(19)
#define BIT_PMU_APB_CGM_TWPLL_128M_AP_AUTO_GATE_SEL      BIT(18)
#define BIT_PMU_APB_CGM_TWPLL_153M6_AP_AUTO_GATE_SEL     BIT(17)
#define BIT_PMU_APB_CGM_TWPLL_192M_AP_AUTO_GATE_SEL      BIT(16)
#define BIT_PMU_APB_CGM_TWPLL_256M_AP_AUTO_GATE_SEL      BIT(15)
#define BIT_PMU_APB_CGM_TWPLL_307M2_AP_AUTO_GATE_SEL     BIT(14)
#define BIT_PMU_APB_CGM_TWPLL_384M_AP_AUTO_GATE_SEL      BIT(13)
#define BIT_PMU_APB_CGM_TWPLL_512M_AP_AUTO_GATE_SEL      BIT(12)
#define BIT_PMU_APB_CGM_TWPLL_768M_AP_AUTO_GATE_SEL      BIT(11)
#define BIT_PMU_APB_CGM_LTEPLL_AP_AUTO_GATE_SEL          BIT(10)
#define BIT_PMU_APB_CGM_TWPLL_AP_AUTO_GATE_SEL           BIT(9)
#define BIT_PMU_APB_CGM_DPLL_AP_AUTO_GATE_SEL            BIT(8)
#define BIT_PMU_APB_CGM_MPLL_AP_AUTO_GATE_SEL            BIT(7)
#define BIT_PMU_APB_CGM_26M_AP_AUTO_GATE_SEL             BIT(6)
#define BIT_PMU_APB_RC1_2M_AUTO_GATE_SEL                 BIT(5)
#define BIT_PMU_APB_RC0_2M_AUTO_GATE_SEL                 BIT(4)
#define BIT_PMU_APB_AON_1M_AUTO_GATE_SEL                 BIT(3)
#define BIT_PMU_APB_AON_2M_AUTO_GATE_SEL                 BIT(2)
#define BIT_PMU_APB_AON_13M_AUTO_GATE_SEL                BIT(1)
#define BIT_PMU_APB_LTEPLL_DIV_30M72_AUTO_GATE_SEL       BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_CGM_AUTO_GATE_SEL_CFG2
// Register Offset : 0x0150
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_CGM_LTEPLL_245M76_CP1_AUTO_GATE_SEL  BIT(31)
#define BIT_PMU_APB_CGM_LTEPLL_307M2_CP1_AUTO_GATE_SEL   BIT(30)
#define BIT_PMU_APB_CGM_LTEPLL_614M4_CP1_AUTO_GATE_SEL   BIT(29)
#define BIT_PMU_APB_CGM_26M_CP1_AUTO_GATE_SEL            BIT(28)
#define BIT_PMU_APB_CGM_TWPLL_5M12_CP0_AUTO_GATE_SEL     BIT(27)
#define BIT_PMU_APB_CGM_TWPLL_10M24_CP0_AUTO_GATE_SEL    BIT(26)
#define BIT_PMU_APB_CGM_TWPLL_51M2_CP0_AUTO_GATE_SEL     BIT(25)
#define BIT_PMU_APB_CGM_TWPLL_76M8_CP0_AUTO_GATE_SEL     BIT(24)
#define BIT_PMU_APB_CGM_TWPLL_153M6_CP0_AUTO_GATE_SEL    BIT(23)
#define BIT_PMU_APB_CGM_TWPLL_48M_CP0_AUTO_GATE_SEL      BIT(22)
#define BIT_PMU_APB_CGM_TWPLL_96M_CP0_AUTO_GATE_SEL      BIT(21)
#define BIT_PMU_APB_CGM_TWPLL_192M_CP0_AUTO_GATE_SEL     BIT(20)
#define BIT_PMU_APB_CGM_TWPLL_64M_CP0_AUTO_GATE_SEL      BIT(19)
#define BIT_PMU_APB_CGM_TWPLL_102M4_CP0_AUTO_GATE_SEL    BIT(18)
#define BIT_PMU_APB_CGM_TWPLL_128M_CP0_AUTO_GATE_SEL     BIT(17)
#define BIT_PMU_APB_CGM_TWPLL_256M_CP0_AUTO_GATE_SEL     BIT(16)
#define BIT_PMU_APB_CGM_TWPLL_219M_CP0_AUTO_GATE_SEL     BIT(15)
#define BIT_PMU_APB_CGM_TWPLL_219M4_CP0_AUTO_GATE_SEL    BIT(14)
#define BIT_PMU_APB_CGM_TWPLL_307M2_CP0_AUTO_GATE_SEL    BIT(13)
#define BIT_PMU_APB_CGM_TWPLL_512M_CP0_AUTO_GATE_SEL     BIT(12)
#define BIT_PMU_APB_CGM_26M_CP0_AUTO_GATE_SEL            BIT(11)
#define BIT_PMU_APB_CGM_TWPLL_153M6_GPU_AUTO_GATE_SEL    BIT(10)
#define BIT_PMU_APB_CGM_TWPLL_192M_GPU_AUTO_GATE_SEL     BIT(9)
#define BIT_PMU_APB_CGM_TWPLL_256M_GPU_AUTO_GATE_SEL     BIT(8)
#define BIT_PMU_APB_CGM_TWPLL_307M2_GPU_AUTO_GATE_SEL    BIT(7)
#define BIT_PMU_APB_CGM_TWPLL_384M_GPU_AUTO_GATE_SEL     BIT(6)
#define BIT_PMU_APB_CGM_TWPLL_512M_GPU_AUTO_GATE_SEL     BIT(5)
#define BIT_PMU_APB_CGM_TWPLL_24M_MM_AUTO_GATE_SEL       BIT(4)
#define BIT_PMU_APB_CGM_TWPLL_48M_MM_AUTO_GATE_SEL       BIT(3)
#define BIT_PMU_APB_CGM_TWPLL_64M_MM_AUTO_GATE_SEL       BIT(2)
#define BIT_PMU_APB_CGM_TWPLL_76M8_MM_AUTO_GATE_SEL      BIT(1)
#define BIT_PMU_APB_CGM_TWPLL_96M_MM_AUTO_GATE_SEL       BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_CGM_AUTO_GATE_SEL_CFG3
// Register Offset : 0x0154
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_CA5_TOP_AUTO_GATE_SEL                BIT(19)
#define BIT_PMU_APB_CA7_TOP_AUTO_GATE_SEL                BIT(18)
#define BIT_PMU_APB_CGM_TWPLL_48M_CP1_AUTO_GATE_SEL      BIT(17)
#define BIT_PMU_APB_CGM_TWPLL_51M2_CP1_AUTO_GATE_SEL     BIT(16)
#define BIT_PMU_APB_CGM_TWPLL_64M_CP1_AUTO_GATE_SEL      BIT(15)
#define BIT_PMU_APB_CGM_TWPLL_76M8_CP1_AUTO_GATE_SEL     BIT(14)
#define BIT_PMU_APB_CGM_TWPLL_96M_CP1_AUTO_GATE_SEL      BIT(13)
#define BIT_PMU_APB_CGM_TWPLL_128M_CP1_AUTO_GATE_SEL     BIT(12)
#define BIT_PMU_APB_CGM_TWPLL_153M6_CP1_AUTO_GATE_SEL    BIT(11)
#define BIT_PMU_APB_CGM_TWPLL_192M_CP1_AUTO_GATE_SEL     BIT(10)
#define BIT_PMU_APB_CGM_TWPLL_256M_CP1_AUTO_GATE_SEL     BIT(9)
#define BIT_PMU_APB_CGM_TWPLL_512M_CP1_AUTO_GATE_SEL     BIT(8)
#define BIT_PMU_APB_CGM_TWPLL_768M_CP1_AUTO_GATE_SEL     BIT(7)
#define BIT_PMU_APB_CGM_LTEPLL_30M72_CP1_AUTO_GATE_SEL   BIT(6)
#define BIT_PMU_APB_CGM_LTEPLL_61M44_CP1_AUTO_GATE_SEL   BIT(5)
#define BIT_PMU_APB_CGM_LTEPLL_68M26_CP1_AUTO_GATE_SEL   BIT(4)
#define BIT_PMU_APB_CGM_LTEPLL_102M4_CP1_AUTO_GATE_SEL   BIT(3)
#define BIT_PMU_APB_CGM_LTEPLL_122M88_CP1_AUTO_GATE_SEL  BIT(2)
#define BIT_PMU_APB_CGM_LTEPLL_153M6_CP1_AUTO_GATE_SEL   BIT(1)
#define BIT_PMU_APB_CGM_LTEPLL_204M8_CP1_AUTO_GATE_SEL   BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_CGM_FORCE_EN_CFG0
// Register Offset : 0x0158
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_CGM_TWPLL_512M_GPU_FORCE_EN          BIT(31)
#define BIT_PMU_APB_CGM_TWPLL_24M_MM_FORCE_EN            BIT(30)
#define BIT_PMU_APB_CGM_TWPLL_48M_MM_FORCE_EN            BIT(29)
#define BIT_PMU_APB_CGM_TWPLL_64M_MM_FORCE_EN            BIT(28)
#define BIT_PMU_APB_CGM_TWPLL_76M8_MM_FORCE_EN           BIT(27)
#define BIT_PMU_APB_CGM_TWPLL_96M_MM_FORCE_EN            BIT(26)
#define BIT_PMU_APB_CGM_TWPLL_128M_MM_FORCE_EN_CFG1      BIT(25)
#define BIT_PMU_APB_CGM_TWPLL_153M6_MM_FORCE_EN          BIT(24)
#define BIT_PMU_APB_CGM_TWPLL_192M_MM_FORCE_EN           BIT(23)
#define BIT_PMU_APB_CGM_TWPLL_256M_MM_FORCE_EN           BIT(22)
#define BIT_PMU_APB_CGM_TWPLL_307M2_MM_FORCE_EN          BIT(21)
#define BIT_PMU_APB_CGM_26M_MM_FORCE_EN                  BIT(20)
#define BIT_PMU_APB_CGM_TWPLL_12M_AP_FORCE_EN            BIT(19)
#define BIT_PMU_APB_CGM_TWPLL_24M_AP_FORCE_EN            BIT(18)
#define BIT_PMU_APB_CGM_TWPLL_48M_AP_FORCE_EN            BIT(17)
#define BIT_PMU_APB_CGM_TWPLL_51M2_AP_FORCE_EN           BIT(16)
#define BIT_PMU_APB_CGM_TWPLL_64M_AP_FORCE_EN            BIT(15)
#define BIT_PMU_APB_CGM_TWPLL_76M8_AP_FORCE_EN           BIT(14)
#define BIT_PMU_APB_CGM_TWPLL_96M_AP_FORCE_EN            BIT(13)
#define BIT_PMU_APB_CGM_TWPLL_128M_AP_FORCE_EN           BIT(12)
#define BIT_PMU_APB_CGM_TWPLL_153M6_AP_FORCE_EN          BIT(11)
#define BIT_PMU_APB_CGM_TWPLL_192M_AP_FORCE_EN           BIT(10)
#define BIT_PMU_APB_CGM_TWPLL_256M_AP_FORCE_EN           BIT(9)
#define BIT_PMU_APB_CGM_TWPLL_307M2_AP_FORCE_EN          BIT(8)
#define BIT_PMU_APB_CGM_TWPLL_384M_AP_FORCE_EN           BIT(7)
#define BIT_PMU_APB_CGM_TWPLL_512M_AP_FORCE_EN           BIT(6)
#define BIT_PMU_APB_CGM_TWPLL_768M_AP_FORCE_EN           BIT(5)
#define BIT_PMU_APB_CGM_LTEPLL_AP_FORCE_EN               BIT(4)
#define BIT_PMU_APB_CGM_TWPLL_AP_FORCE_EN                BIT(3)
#define BIT_PMU_APB_CGM_DPLL_AP_FORCE_EN                 BIT(2)
#define BIT_PMU_APB_CGM_MPLL_AP_FORCE_EN                 BIT(1)
#define BIT_PMU_APB_CGM_26M_AP_FORCE_EN                  BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_CGM_FORCE_EN_CFG1
// Register Offset : 0x015C
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_CGM_LTEPLL_61M44_CP1_FORCE_EN        BIT(31)
#define BIT_PMU_APB_CGM_LTEPLL_68M26_CP1_FORCE_EN        BIT(30)
#define BIT_PMU_APB_CGM_LTEPLL_102M4_CP1_FORCE_EN        BIT(29)
#define BIT_PMU_APB_CGM_LTEPLL_122M88_CP1_FORCE_EN       BIT(28)
#define BIT_PMU_APB_CGM_LTEPLL_153M6_CP1_FORCE_EN        BIT(27)
#define BIT_PMU_APB_CGM_LTEPLL_204M8_CP1_FORCE_EN        BIT(26)
#define BIT_PMU_APB_CGM_LTEPLL_245M76_CP1_FORCE_EN       BIT(25)
#define BIT_PMU_APB_CGM_LTEPLL_307M2_CP1_FORCE_EN        BIT(24)
#define BIT_PMU_APB_CGM_LTEPLL_614M4_CP1_FORCE_EN        BIT(23)
#define BIT_PMU_APB_CGM_26M_CP1_FORCE_EN                 BIT(22)
#define BIT_PMU_APB_CGM_TWPLL_5M12_CP0_FORCE_EN          BIT(21)
#define BIT_PMU_APB_CGM_TWPLL_10M24_CP0_FORCE_EN         BIT(20)
#define BIT_PMU_APB_CGM_TWPLL_51M2_CP0_FORCE_EN          BIT(19)
#define BIT_PMU_APB_CGM_TWPLL_76M8_CP0_FORCE_EN          BIT(18)
#define BIT_PMU_APB_CGM_TWPLL_153M6_CP0_FORCE_EN         BIT(17)
#define BIT_PMU_APB_CGM_TWPLL_48M_CP0_FORCE_EN           BIT(16)
#define BIT_PMU_APB_CGM_TWPLL_96M_CP0_FORCE_EN           BIT(15)
#define BIT_PMU_APB_CGM_TWPLL_192M_CP0_FORCE_EN          BIT(14)
#define BIT_PMU_APB_CGM_TWPLL_64M_CP0_FORCE_EN           BIT(13)
#define BIT_PMU_APB_CGM_TWPLL_102M4_CP0_FORCE_EN         BIT(12)
#define BIT_PMU_APB_CGM_TWPLL_128M_CP0_FORCE_EN          BIT(11)
#define BIT_PMU_APB_CGM_TWPLL_256M_CP0_FORCE_EN          BIT(10)
#define BIT_PMU_APB_CGM_TWPLL_219M_CP0_FORCE_EN          BIT(9)
#define BIT_PMU_APB_CGM_TWPLL_219M4_CP0_FORCE_EN         BIT(8)
#define BIT_PMU_APB_CGM_TWPLL_307M2_CP0_FORCE_EN         BIT(7)
#define BIT_PMU_APB_CGM_TWPLL_512M_CP0_FORCE_EN          BIT(6)
#define BIT_PMU_APB_CGM_26M_CP0_FORCE_EN                 BIT(5)
#define BIT_PMU_APB_CGM_TWPLL_153M6_GPU_FORCE_EN         BIT(4)
#define BIT_PMU_APB_CGM_TWPLL_192M_GPU_FORCE_EN          BIT(3)
#define BIT_PMU_APB_CGM_TWPLL_256M_GPU_FORCE_EN          BIT(2)
#define BIT_PMU_APB_CGM_TWPLL_307M2_GPU_FORCE_EN         BIT(1)
#define BIT_PMU_APB_CGM_TWPLL_384M_GPU_FORCE_EN          BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_CGM_FORCE_EN_CFG2
// Register Offset : 0x0160
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_TWPLL_DIV_38M4_FORCE_EN              BIT(31)
#define BIT_PMU_APB_TWPLL_DIV_76M8_FORCE_EN              BIT(30)
#define BIT_PMU_APB_TWPLL_DIV_51M2_FORCE_EN              BIT(29)
#define BIT_PMU_APB_TWPLL_DIV_153M6_FORCE_EN             BIT(28)
#define BIT_PMU_APB_TWPLL_DIV_64M_FORCE_EN               BIT(27)
#define BIT_PMU_APB_TWPLL_DIV_128M_FORCE_EN              BIT(26)
#define BIT_PMU_APB_TWPLL_DIV_256M_FORCE_EN              BIT(25)
#define BIT_PMU_APB_TWPLL_DIV_12M_FORCE_EN               BIT(24)
#define BIT_PMU_APB_TWPLL_DIV_24M_FORCE_EN               BIT(23)
#define BIT_PMU_APB_TWPLL_DIV_48M_FORCE_EN               BIT(22)
#define BIT_PMU_APB_TWPLL_DIV_96M_FORCE_EN               BIT(21)
#define BIT_PMU_APB_TWPLL_DIV_102M4_FORCE_EN             BIT(20)
#define BIT_PMU_APB_TWPLL_DIV_192M_FORCE_EN              BIT(19)
#define BIT_PMU_APB_TWPLL_DIV_219M4_FORCE_EN             BIT(18)
#define BIT_PMU_APB_TWPLL_DIV_307M2_FORCE_EN             BIT(17)
#define BIT_PMU_APB_TWPLL_DIV_384M_FORCE_EN              BIT(16)
#define BIT_PMU_APB_TWPLL_DIV_512M_FORCE_EN              BIT(15)
#define BIT_PMU_APB_TWPLL_DIV_768M_FORCE_EN              BIT(14)
#define BIT_PMU_APB_DPLL_DIV_33M_FORCE_EN                BIT(13)
#define BIT_PMU_APB_MPLL_DIV_50M_FORCE_EN                BIT(12)
#define BIT_PMU_APB_CGM_TWPLL_48M_CP1_FORCE_EN           BIT(11)
#define BIT_PMU_APB_CGM_TWPLL_51M2_CP1_FORCE_EN          BIT(10)
#define BIT_PMU_APB_CGM_TWPLL_64M_CP1_FORCE_EN           BIT(9)
#define BIT_PMU_APB_CGM_TWPLL_76M8_CP1_FORCE_EN          BIT(8)
#define BIT_PMU_APB_CGM_TWPLL_96M_CP1_FORCE_EN           BIT(7)
#define BIT_PMU_APB_CGM_TWPLL_128M_CP1_FORCE_EN          BIT(6)
#define BIT_PMU_APB_CGM_TWPLL_153M6_CP1_FORCE_EN         BIT(5)
#define BIT_PMU_APB_CGM_TWPLL_192M_CP1_FORCE_EN          BIT(4)
#define BIT_PMU_APB_CGM_TWPLL_256M_CP1_FORCE_EN          BIT(3)
#define BIT_PMU_APB_CGM_TWPLL_512M_CP1_FORCE_EN          BIT(2)
#define BIT_PMU_APB_CGM_TWPLL_768M_CP1_FORCE_EN          BIT(1)
#define BIT_PMU_APB_CGM_LTEPLL_30M72_CP1_FORCE_EN        BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_CGM_FORCE_EN_CFG3
// Register Offset : 0x0164
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_RC1_2M_FORCE_EN                      BIT(17)
#define BIT_PMU_APB_RC0_2M_FORCE_EN                      BIT(16)
#define BIT_PMU_APB_AON_1M_FORCE_EN                      BIT(15)
#define BIT_PMU_APB_AON_2M_FORCE_EN                      BIT(14)
#define BIT_PMU_APB_AON_13M_FORCE_EN                     BIT(13)
#define BIT_PMU_APB_LTEPLL_DIV_30M72_FORCE_EN            BIT(12)
#define BIT_PMU_APB_LTEPLL_DIV_61M44_FORCE_EN            BIT(11)
#define BIT_PMU_APB_LTEPLL_DIV_122M88_FORCE_EN           BIT(10)
#define BIT_PMU_APB_LTEPLL_DIV_245M76_FORCE_EN           BIT(9)
#define BIT_PMU_APB_LTEPLL_DIV_68M26_FORCE_EN            BIT(8)
#define BIT_PMU_APB_LTEPLL_DIV_102M4_FORCE_EN            BIT(7)
#define BIT_PMU_APB_LTEPLL_DIV_204M8_FORCE_EN            BIT(6)
#define BIT_PMU_APB_LTEPLL_DIV_409M6_FORCE_EN            BIT(5)
#define BIT_PMU_APB_LTEPLL_DIV_153M6_FORCE_EN            BIT(4)
#define BIT_PMU_APB_LTEPLL_DIV_307M2_FORCE_EN            BIT(3)
#define BIT_PMU_APB_LTEPLL_DIV_614M4_FORCE_EN            BIT(2)
#define BIT_PMU_APB_TWPLL_DIV_5M12_FORCE_EN              BIT(1)
#define BIT_PMU_APB_TWPLL_DIV_10M24_FORCE_EN             BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_SLEEP_XTLON_CTRL
// Register Offset : 0x0168
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_ARM7_SLEEP_XTL_ON                    BIT(5)
#define BIT_PMU_APB_VCP1_SLEEP_XTL_ON                    BIT(4)
#define BIT_PMU_APB_VCP0_SLEEP_XTL_ON                    BIT(3)
#define BIT_PMU_APB_CP1_SLEEP_XTL_ON                     BIT(2)
#define BIT_PMU_APB_CP0_SLEEP_XTL_ON                     BIT(1)
#define BIT_PMU_APB_AP_SLEEP_XTL_ON                      BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_MEM_SLP_CFG
// Register Offset : 0x016C
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_ARM7_MEM_SLP_UART                    BIT(28)
#define BIT_PMU_APB_ARM7_MEM_SLP_IMC                     BIT(27)
#define BIT_PMU_APB_AON_MEM_SLP_IMC                      BIT(26)
#define BIT_PMU_APB_AON_MEM_SLP_DMA                      BIT(25)
#define BIT_PMU_APB_AON_MEM_SLP_VBC                      BIT(24)
#define BIT_PMU_APB_AON_MEM_SLP_AUD                      BIT(23)
#define BIT_PMU_APB_AON_MEM_SLP_FM                       BIT(22)
#define BIT_PMU_APB_CP0_GGE_DSP_MEM_SLP_RFT              BIT(21)
#define BIT_PMU_APB_CP0_GGE_DSP_MEM_SLP_DMA              BIT(20)
#define BIT_PMU_APB_CP0_GGE_DSP_MEM_SLP_SHM              BIT(19)
#define BIT_PMU_APB_CP0_GGE_DSP_MEM_SLP_STC              BIT(18)
#define BIT_PMU_APB_CP0_W_MEM_SLP_UART1                  BIT(17)
#define BIT_PMU_APB_CP0_W_MEM_SLP_UART0                  BIT(16)
#define BIT_PMU_APB_CP0_W_MEM_SLP_IRAM                   BIT(15)
#define BIT_PMU_APB_CP0_W_MEM_SLP_PERIF                  BIT(14)
#define BIT_PMU_APB_CP0_DSP_MEM_SLP_DMA                  BIT(13)
#define BIT_PMU_APB_CP0_DSP_MEM_SLP_SHM                  BIT(12)
#define BIT_PMU_APB_CP0_DSP_MEM_SLP_RFT                  BIT(11)
#define BIT_PMU_APB_CP0_DSP_MEM_SLP_STC                  BIT(10)
#define BIT_PMU_APB_CP0_ARM_MEM_SLP_IIS3                 BIT(9)
#define BIT_PMU_APB_CP0_ARM_MEM_SLP_IIS2                 BIT(8)
#define BIT_PMU_APB_CP0_ARM_MEM_SLP_IIS1                 BIT(7)
#define BIT_PMU_APB_CP0_ARM_MEM_SLP_IIS0                 BIT(6)
#define BIT_PMU_APB_CP0_ARM_MEM_SLP_UART2                BIT(5)
#define BIT_PMU_APB_CP0_ARM_MEM_SLP_UART1                BIT(4)
#define BIT_PMU_APB_CP0_ARM_MEM_SLP_UART0                BIT(3)
#define BIT_PMU_APB_CP0_ARM_MEM_SLP_EPT                  BIT(2)
#define BIT_PMU_APB_CP0_ARM_MEM_SLP_LZMA                 BIT(1)
#define BIT_PMU_APB_CP0_ARM_MEM_SLP_DMA                  BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_MEM_SD_CFG
// Register Offset : 0x0170
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_ARM7_MEM_SD_UART                     BIT(28)
#define BIT_PMU_APB_ARM7_MEM_SD_IMC                      BIT(27)
#define BIT_PMU_APB_AON_MEM_SD_IMC                       BIT(26)
#define BIT_PMU_APB_AON_MEM_SD_DMA                       BIT(25)
#define BIT_PMU_APB_AON_MEM_SD_VBC                       BIT(24)
#define BIT_PMU_APB_AON_MEM_SD_AUD                       BIT(23)
#define BIT_PMU_APB_AON_MEM_SD_FM                        BIT(22)
#define BIT_PMU_APB_CP0_GGE_DSP_MEM_SD_RFT               BIT(21)
#define BIT_PMU_APB_CP0_GGE_DSP_MEM_SD_DMA               BIT(20)
#define BIT_PMU_APB_CP0_GGE_DSP_MEM_SD_SHM               BIT(19)
#define BIT_PMU_APB_CP0_GGE_DSP_MEM_SD_STC               BIT(18)
#define BIT_PMU_APB_CP0_W_MEM_SD_UART1                   BIT(17)
#define BIT_PMU_APB_CP0_W_MEM_SD_UART0                   BIT(16)
#define BIT_PMU_APB_CP0_W_MEM_SD_IRAM                    BIT(15)
#define BIT_PMU_APB_CP0_W_MEM_SD_PERIF                   BIT(14)
#define BIT_PMU_APB_CP0_DSP_MEM_SD_DMA                   BIT(13)
#define BIT_PMU_APB_CP0_DSP_MEM_SD_SHM                   BIT(12)
#define BIT_PMU_APB_CP0_DSP_MEM_SD_RFT                   BIT(11)
#define BIT_PMU_APB_CP0_DSP_MEM_SD_STC                   BIT(10)
#define BIT_PMU_APB_CP0_ARM_MEM_SD_IIS3                  BIT(9)
#define BIT_PMU_APB_CP0_ARM_MEM_SD_IIS2                  BIT(8)
#define BIT_PMU_APB_CP0_ARM_MEM_SD_IIS1                  BIT(7)
#define BIT_PMU_APB_CP0_ARM_MEM_SD_IIS0                  BIT(6)
#define BIT_PMU_APB_CP0_ARM_MEM_SD_UART2                 BIT(5)
#define BIT_PMU_APB_CP0_ARM_MEM_SD_UART1                 BIT(4)
#define BIT_PMU_APB_CP0_ARM_MEM_SD_UART0                 BIT(3)
#define BIT_PMU_APB_CP0_ARM_MEM_SD_EPT                   BIT(2)
#define BIT_PMU_APB_CP0_ARM_MEM_SD_LZMA                  BIT(1)
#define BIT_PMU_APB_CP0_ARM_MEM_SD_DMA                   BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_CA7_CORE_PU_LOCK
// Register Offset : 0x0174
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_CA7_C3_GIC_WAKEUP_EN                 BIT(11)
#define BIT_PMU_APB_CA7_C2_GIC_WAKEUP_EN                 BIT(10)
#define BIT_PMU_APB_CA7_C1_GIC_WAKEUP_EN                 BIT(9)
#define BIT_PMU_APB_CA7_C0_GIC_WAKEUP_EN                 BIT(8)
#define BIT_PMU_APB_CA7_C3_PU_LOCK                       BIT(3)
#define BIT_PMU_APB_CA7_C2_PU_LOCK                       BIT(2)
#define BIT_PMU_APB_CA7_C1_PU_LOCK                       BIT(1)
#define BIT_PMU_APB_CA7_C0_PU_LOCK                       BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_ARM7_HOLD_CGM_EN
// Register Offset : 0x0178
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP1_CEVA_CGM_HOLD_EN              BIT(10)
#define BIT_PMU_APB_PD_CP1_CA5_CGM_HOLD_EN               BIT(9)
#define BIT_PMU_APB_PD_CP0_CEVA_1_CGM_HOLD_EN            BIT(8)
#define BIT_PMU_APB_PD_CP0_CEVA_0_CGM_HOLD_EN            BIT(7)
#define BIT_PMU_APB_PD_CP0_ARM9_1_CGM_HOLD_EN            BIT(6)
#define BIT_PMU_APB_PD_CP0_ARM9_0_CGM_HOLD_EN            BIT(5)
#define BIT_PMU_APB_PD_CA7_TOP_CMG_HOLD_EN               BIT(4)
#define BIT_PMU_APB_PD_CA7_C3_CMG_HOLD_EN                BIT(3)
#define BIT_PMU_APB_PD_CA7_C2_CMG_HOLD_EN                BIT(2)
#define BIT_PMU_APB_PD_CA7_C1_CMG_HOLD_EN                BIT(1)
#define BIT_PMU_APB_PD_CA7_C0_CMG_HOLD_EN                BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PWR_CNT_WAIT_CFG0
// Register Offset : 0x017C
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_VCP0_PWR_WAIT_CNT(x)                 (((x) & 0xFF) << 24)
#define BIT_PMU_APB_CP1_PWR_WAIT_CNT(x)                  (((x) & 0xFF) << 16)
#define BIT_PMU_APB_CP0_PWR_WAIT_CNT(x)                  (((x) & 0xFF) << 8)
#define BIT_PMU_APB_AP_PWR_WAIT_CNT(x)                   (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_PWR_CNT_WAIT_CFG1
// Register Offset : 0x0180
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_ARM7_PWR_WAIT_CNT(x)                 (((x) & 0xFF) << 8)
#define BIT_PMU_APB_VCP1_PWR_WAIT_CNT(x)                 (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_RC0_REL_CFG
// Register Offset : 0x0184
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_RC0_ARM7_SEL                         BIT(5)
#define BIT_PMU_APB_RC0_VCP1_SEL                         BIT(4)
#define BIT_PMU_APB_RC0_VCP0_SEL                         BIT(3)
#define BIT_PMU_APB_RC0_CP1_SEL                          BIT(2)
#define BIT_PMU_APB_RC0_CP0_SEL                          BIT(1)
#define BIT_PMU_APB_RC0_AP_SEL                           BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_RC1_REL_CFG
// Register Offset : 0x0188
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_RC1_ARM7_SEL                         BIT(5)
#define BIT_PMU_APB_RC1_VCP1_SEL                         BIT(4)
#define BIT_PMU_APB_RC1_VCP0_SEL                         BIT(3)
#define BIT_PMU_APB_RC1_CP1_SEL                          BIT(2)
#define BIT_PMU_APB_RC1_CP0_SEL                          BIT(1)
#define BIT_PMU_APB_RC1_AP_SEL                           BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_RC_CNT_WAIT_CFG
// Register Offset : 0x018C
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_RC1_WAIT_CNT(x)                      (((x) & 0xFF) << 8)
#define BIT_PMU_APB_RC0_WAIT_CNT(x)                      (((x) & 0xFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_MEM_AUTO_SLP_CFG
// Register Offset : 0x0190
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_MEM_AUTO_SLP_EN(x)                   (((x) & 0xFFFFFFFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_MEM_AUTO_SD_CFG
// Register Offset : 0x0194
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_MEM_AUTO_SD_EN(x)                    (((x) & 0xFFFFFFFF))

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_CP0_PD_SHUTDOWN_CFG
// Register Offset : 0x0198
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP0_HU3GE_VCP1_SEL                BIT(20)
#define BIT_PMU_APB_PD_CP0_TD_VCP1_SEL                   BIT(19)
#define BIT_PMU_APB_PD_CP0_GSM_0_VCP1_SEL                BIT(18)
#define BIT_PMU_APB_PD_CP0_CEVA_0_VCP1_SEL               BIT(17)
#define BIT_PMU_APB_PD_CP0_ARM9_1_VCP1_SEL               BIT(16)
#define BIT_PMU_APB_PD_CP0_ARM9_0_VCP0_SEL               BIT(8)
#define BIT_PMU_APB_PD_CP0_HU3GE_CP0_SEL                 BIT(5)
#define BIT_PMU_APB_PD_CP0_TD_CP0_SEL                    BIT(4)
#define BIT_PMU_APB_PD_CP0_GSM_0_CP0_SEL                 BIT(3)
#define BIT_PMU_APB_PD_CP0_CEVA_0_CP0_SEL                BIT(2)
#define BIT_PMU_APB_PD_CP0_ARM9_1_CP0_SEL                BIT(1)
#define BIT_PMU_APB_PD_CP0_ARM9_0_CP0_SEL                BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_CP1_PD_SHUTDOWN_CFG
// Register Offset : 0x019C
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP1_COMWRAP_VCP1_SEL              BIT(20)
#define BIT_PMU_APB_PD_CP1_CEVA_VCP1_SEL                 BIT(19)
#define BIT_PMU_APB_PD_CP1_LTE_P2_VCP1_SEL               BIT(18)
#define BIT_PMU_APB_PD_CP1_LTE_P1_VCP1_SEL               BIT(17)
#define BIT_PMU_APB_PD_CP1_CA5_VCP1_SEL                  BIT(16)
#define BIT_PMU_APB_PD_CP1_COMWRAP_CP1_SEL               BIT(4)
#define BIT_PMU_APB_PD_CP1_CEVA_CP1_SEL                  BIT(3)
#define BIT_PMU_APB_PD_CP1_LTE_P2_CP1_SEL                BIT(2)
#define BIT_PMU_APB_PD_CP1_LTE_P1_CP1_SEL                BIT(1)
#define BIT_PMU_APB_PD_CP1_CA5_CP1_SEL                   BIT(0)

/*-------------------------------------------------------------
// Register Name   : REG_PMU_APB_WAKEUP_LOCK_EN
// Register Offset : 0x01A0
// Description     :
-------------------------------------------------------------*/

#define BIT_PMU_APB_VCP1_SYS_WAKEUP_LOCK_EN              BIT(26)
#define BIT_PMU_APB_VCP0_SYS_WAKEUP_LOCK_EN              BIT(25)
#define BIT_PMU_APB_CP1_SYS_WAKEUP_LOCK_EN               BIT(24)
#define BIT_PMU_APB_CP0_SYS_WAKEUP_LOCK_EN               BIT(23)
#define BIT_PMU_APB_AP_SYS_WAKEUP_LOCK_EN                BIT(22)
#define BIT_PMU_APB_PD_PUB_SYS_WAKEUP_LOCK_EN            BIT(21)
#define BIT_PMU_APB_PD_CP1_COMWRAP_WAKEUP_LOCK_EN        BIT(20)
#define BIT_PMU_APB_PD_CP1_CEVA_WAKEUP_LOCK_EN           BIT(19)
#define BIT_PMU_APB_PD_CP1_LTE_P2_WAKEUP_LOCK_EN         BIT(18)
#define BIT_PMU_APB_PD_CP1_LTE_P1_WAKEUP_LOCK_EN         BIT(17)
#define BIT_PMU_APB_PD_CP1_CA5_WAKEUP_LOCK_EN            BIT(16)
#define BIT_PMU_APB_PD_CP0_CEVA_1_WAKEUP_LOCK_EN         BIT(15)
#define BIT_PMU_APB_PD_CP0_CEVA_0_WAKEUP_LOCK_EN         BIT(14)
#define BIT_PMU_APB_PD_CP0_TD_WAKEUP_LOCK_EN             BIT(13)
#define BIT_PMU_APB_PD_CP0_GSM_1_WAKEUP_LOCK_EN          BIT(12)
#define BIT_PMU_APB_PD_CP0_GSM_0_WAKEUP_LOCK_EN          BIT(11)
#define BIT_PMU_APB_PD_CP0_HU3GE_WAKEUP_LOCK_EN          BIT(10)
#define BIT_PMU_APB_PD_CP0_ARM9_1_WAKEUP_LOCK_EN         BIT(9)
#define BIT_PMU_APB_PD_CP0_ARM9_0_WAKEUP_LOCK_EN         BIT(8)
#define BIT_PMU_APB_PD_MM_TOP_WAKEUP_LOCK_EN             BIT(7)
#define BIT_PMU_APB_PD_GPU_TOP_WAKEUP_LOCK_EN            BIT(6)
#define BIT_PMU_APB_PD_AP_SYS_WAKEUP_LOCK_EN             BIT(5)
#define BIT_PMU_APB_PD_CA7_TOP_WAKEUP_LOCK_EN            BIT(4)
#define BIT_PMU_APB_PD_CA7_C3_WAKEUP_LOCK_EN             BIT(3)
#define BIT_PMU_APB_PD_CA7_C2_WAKEUP_LOCK_EN             BIT(2)
#define BIT_PMU_APB_PD_CA7_C1_WAKEUP_LOCK_EN             BIT(1)
#define BIT_PMU_APB_PD_CA7_C0_WAKEUP_LOCK_EN             BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CA7_C0_SHUTDOWN_MARK_STATUS
// Register Offset : 0x3000
// Description     :
---------------------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CA7_C0_SHUTDOWN_MARK(x)           (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CA7_C1_SHUTDOWN_MARK_STATUS
// Register Offset : 0x3004
// Description     :
---------------------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CA7_C1_SHUTDOWN_MARK(x)           (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CA7_C2_SHUTDOWN_MARK_STATUS
// Register Offset : 0x3008
// Description     :
---------------------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CA7_C2_SHUTDOWN_MARK(x)           (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CA7_C3_SHUTDOWN_MARK_STATUS
// Register Offset : 0x300C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CA7_C3_SHUTDOWN_MARK(x)           (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CA7_TOP_SHUTDOWN_MARK_STATUS
// Register Offset : 0x3010
// Description     :
---------------------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CA7_TOP_SHUTDOWN_MARK(x)          (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_AP_SYS_SHUTDOWN_MARK_STATUS
// Register Offset : 0x3014
// Description     :
---------------------------------------------------------------------------*/

#define BIT_PMU_APB_PD_AP_SYS_SHUTDOWN_MARK(x)           (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_GPU_TOP_SHUTDOWN_MARK_STATUS
// Register Offset : 0x3018
// Description     :
---------------------------------------------------------------------------*/

#define BIT_PMU_APB_PD_GPU_TOP_SHUTDOWN_MARK(x)           (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_MM_TOP_SHUTDOWN_MARK_STATUS
// Register Offset : 0x301C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_PMU_APB_PD_MM_TOP_SHUTDOWN_MARK(x)            (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP0_ARM9_0_SHUTDOWN_MARK_STATUS
// Register Offset : 0x3020
// Description     :
---------------------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP0_ARM9_0_SHUTDOWN_MARK(x)        (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP0_ARM9_1_SHUTDOWN_MARK_STATUS
// Register Offset : 0x3024
// Description     :
---------------------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP0_ARM9_1_SHUTDOWN_MARK(x)        (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP0_CEVA_0_SHUTDOWN_MARK_STATUS
// Register Offset : 0x3028
// Description     :
---------------------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP0_CEVA_0_SHUTDOWN_MARK(x)         (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP0_CEVA_1_SHUTDOWN_MARK_STATUS
// Register Offset : 0x302C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP0_CEVA_1_SHUTDOWN_MARK(x)         (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP0_GSM_0_SHUTDOWN_MARK_STATUS
// Register Offset : 0x3030
// Description     :
---------------------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP0_GSM_0_SHUTDOWN_MARK(x)          (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP0_GSM_1_SHUTDOWN_MARK_STATUS
// Register Offset : 0x3034
// Description     :
---------------------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP0_GSM_1_SHUTDOWN_MARK(x)          (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP0_TD_SHUTDOWN_MARK_STATUS
// Register Offset : 0x3038
// Description     :
---------------------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP0_TD_SHUTDOWN_MARK(x)             (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP0_HU3GE_SHUTDOWN_MARK_STATUS
// Register Offset : 0x303C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP0_HU3GE_SHUTDOWN_MARK(x)          (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP1_CA5_SHUTDOWN_MARK_STATUS
// Register Offset : 0x3040
// Description     :
---------------------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP1_CA5_SHUTDOWN_MARK(x)            (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP1_CEVA_SHUTDOWN_MARK_STATUS
// Register Offset : 0x3044
// Description     :
---------------------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP1_CEVA_SHUTDOWN_MARK(x)           (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP1_LTE_P1_SHUTDOWN_MARK_STATUS
// Register Offset : 0x3048
// Description     :
---------------------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP1_LTE_P1_SHUTDOWN_MARK(x)         (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP1_LTE_P2_SHUTDOWN_MARK_STATUS
// Register Offset : 0x304C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP1_LTE_P2_SHUTDOWN_MARK(x)          (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_CP1_COMWRAP_SHUTDOWN_MARK_STATUS
// Register Offset : 0x3050
// Description     :
---------------------------------------------------------------------------*/

#define BIT_PMU_APB_PD_CP1_COMWRAP_SHUTDOWN_MARK(x)         (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_PMU_APB_PD_PUB_SYS_SHUTDOWN_MARK_STATUS
// Register Offset : 0x3054
// Description     :
---------------------------------------------------------------------------*/

#define BIT_PMU_APB_PD_PUB_SYS_SHUTDOWN_MARK(x)             (((x) & 0xF))


#endif

/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
 *
 * This file is dual-licensed: you can use it either under the terms
 * of the GPL or the X11 license, at your option. Note that this dual
 * licensing only applies to this file, and not this project as a
 * whole.
 *
 * updated at 2017-12-27 19:24:16
 *
 */


#ifndef DBG_APB_H
#define DBG_APB_H



#define REG_DBG_APB_SUB_SYS_DBG_RST_CTRL  (0x0000)
#define REG_DBG_APB_SUB_SYS_DBG_CLK_CTRL  (0x0004)
#define REG_DBG_APB_DEBUG_SUBSYS_SEL_A    (0x0100)
#define REG_DBG_APB_DEBUG_SUBSYS_SEL_B    (0x0104)
#define REG_DBG_APB_DEBUG_BUS_SEL         (0x0108)
#define REG_DBG_APB_AON_DBG_MOD_SEL       (0x010C)
#define REG_DBG_APB_AON_DBG_SIG_SEL       (0x0110)
#define REG_DBG_APB_APCPU_DBG_MOD_SEL     (0x0114)
#define REG_DBG_APB_APCPU_DBG_SIG_SEL     (0x0118)
#define REG_DBG_APB_WTL_DBG_MOD_SEL       (0x011C)
#define REG_DBG_APB_WTL_DBG_SIG_SEL       (0x0120)
#define REG_DBG_APB_ANLG_DBG_MOD_SEL      (0x0124)
#define REG_DBG_APB_ANLG_DBG_SIG_SEL      (0x0128)
#define REG_DBG_APB_PUB_DBG_MOD_SEL       (0x012C)
#define REG_DBG_APB_PUB_DBG_SIG_SEL       (0x0130)
#define REG_DBG_APB_PUBCP_DBG_MOD_SEL     (0x0134)
#define REG_DBG_APB_PUBCP_DBG_SIG_SEL     (0x0138)
#define REG_DBG_APB_GPU_DBG_MOD_SEL       (0x013C)
#define REG_DBG_APB_GPU_DBG_SIG_SEL       (0x0140)
#define REG_DBG_APB_MM_DBG_MOD_SEL        (0x0144)
#define REG_DBG_APB_MM_DBG_SIG_SEL        (0x0148)
#define REG_DBG_APB_AP_DBG_MOD_SEL        (0x014C)
#define REG_DBG_APB_AP_DBG_SIG_SEL        (0x0150)
#define REG_DBG_APB_MDAR_DBG_MOD_SEL      (0x0154)
#define REG_DBG_APB_MDAR_DBG_SIG_SEL      (0x0158)
#define REG_DBG_APB_DISP_DBG_MOD_SEL      (0x015C)
#define REG_DBG_APB_DISP_DBG_SIG_SEL      (0x0160)
#define REG_DBG_APB_WCN_DBG_MOD_SEL       (0x0164)
#define REG_DBG_APB_WCN_DBG_SIG_SEL       (0x0168)
#define REG_DBG_APB_VSP_DBG_MOD_SEL       (0x016C)
#define REG_DBG_APB_VSP_DBG_SIG_SEL       (0x0170)
#define REG_DBG_APB_DBG_BUS0_SEL          (0x0200)
#define REG_DBG_APB_DBG_BUS1_SEL          (0x0204)
#define REG_DBG_APB_DBG_BUS2_SEL          (0x0208)
#define REG_DBG_APB_DBG_BUS3_SEL          (0x020C)
#define REG_DBG_APB_DBG_BUS4_SEL          (0x0210)
#define REG_DBG_APB_DBG_BUS5_SEL          (0x0214)
#define REG_DBG_APB_DBG_BUS6_SEL          (0x0218)
#define REG_DBG_APB_DBG_BUS7_SEL          (0x021C)
#define REG_DBG_APB_DBG_BUS8_SEL          (0x0220)
#define REG_DBG_APB_DBG_BUS9_SEL          (0x0224)
#define REG_DBG_APB_DBG_BUS10_SEL         (0x0228)
#define REG_DBG_APB_DBG_BUS11_SEL         (0x022C)
#define REG_DBG_APB_DBG_BUS12_SEL         (0x0230)
#define REG_DBG_APB_DBG_BUS13_SEL         (0x0234)
#define REG_DBG_APB_DBG_BUS14_SEL         (0x0238)
#define REG_DBG_APB_DBG_BUS15_SEL         (0x023C)
#define REG_DBG_APB_DBG_BUS16_SEL         (0x0240)
#define REG_DBG_APB_DBG_BUS17_SEL         (0x0244)
#define REG_DBG_APB_DBG_BUS18_SEL         (0x0248)
#define REG_DBG_APB_DBG_BUS19_SEL         (0x024C)
#define REG_DBG_APB_DBG_BUS20_SEL         (0x0250)
#define REG_DBG_APB_DBG_BUS21_SEL         (0x0254)
#define REG_DBG_APB_DBG_BUS22_SEL         (0x0258)
#define REG_DBG_APB_DBG_BUS23_SEL         (0x025C)
#define REG_DBG_APB_DBG_BUS24_SEL         (0x0260)
#define REG_DBG_APB_DBG_BUS25_SEL         (0x0264)
#define REG_DBG_APB_DBG_BUS26_SEL         (0x0268)
#define REG_DBG_APB_DBG_BUS27_SEL         (0x026C)
#define REG_DBG_APB_DBG_BUS28_SEL         (0x0270)
#define REG_DBG_APB_DBG_BUS29_SEL         (0x0274)
#define REG_DBG_APB_DBG_BUS30_SEL         (0x0278)
#define REG_DBG_APB_DBG_BUS31_SEL         (0x027C)
#define REG_DBG_APB_DBG_BUS_DATA_A        (0x0300)
#define REG_DBG_APB_DBG_BUS_DATA_B        (0x0304)
#define REG_DBG_APB_DBG_BUS_DATA          (0x0308)

/* REG_DBG_APB_SUB_SYS_DBG_RST_CTRL */

#define BIT_DBG_APB_SERDES_SOFT_RST       BIT(0)

/* REG_DBG_APB_SUB_SYS_DBG_CLK_CTRL */

#define BIT_DBG_APB_TPIU2SERDES_CGM_EN    BIT(0)

/* REG_DBG_APB_DEBUG_SUBSYS_SEL_A */

#define BIT_DBG_APB_DBG_SUBSYS_SEL_A(x)   (((x) & 0xFF))

/* REG_DBG_APB_DEBUG_SUBSYS_SEL_B */

#define BIT_DBG_APB_DBG_SUBSYS_SEL_B(x)   (((x) & 0xFF))

/* REG_DBG_APB_DEBUG_BUS_SEL */

#define BIT_DBG_APB_DBG_BUS_SEL(x)        (((x) & 0xFFFFFFFF))

/* REG_DBG_APB_AON_DBG_MOD_SEL */

#define BIT_DBG_APB_AON_DBG_MOD_SEL(x)    (((x) & 0xFF))

/* REG_DBG_APB_AON_DBG_SIG_SEL */

#define BIT_DBG_APB_AON_DBG_SIG_SEL(x)    (((x) & 0xFF))

/* REG_DBG_APB_APCPU_DBG_MOD_SEL */

#define BIT_DBG_APB_APCPU_DBG_MOD_SEL(x)  (((x) & 0xFF))

/* REG_DBG_APB_APCPU_DBG_SIG_SEL */

#define BIT_DBG_APB_APCPU_DBG_SIG_SEL(x)  (((x) & 0xFF))

/* REG_DBG_APB_WTL_DBG_MOD_SEL */

#define BIT_DBG_APB_WTL_DBG_MOD_SEL(x)    (((x) & 0xFF))

/* REG_DBG_APB_WTL_DBG_SIG_SEL */

#define BIT_DBG_APB_WTL_DBG_SIG_SEL(x)    (((x) & 0xFF))

/* REG_DBG_APB_ANLG_DBG_MOD_SEL */

#define BIT_DBG_APB_ANLG_DBG_MOD_SEL(x)   (((x) & 0xFF))

/* REG_DBG_APB_ANLG_DBG_SIG_SEL */

#define BIT_DBG_APB_ANLG_DBG_SIG_SEL(x)   (((x) & 0xFF))

/* REG_DBG_APB_PUB_DBG_MOD_SEL */

#define BIT_DBG_APB_PUB_DBG_MOD_SEL(x)    (((x) & 0xFF))

/* REG_DBG_APB_PUB_DBG_SIG_SEL */

#define BIT_DBG_APB_PUB_DBG_SIG_SEL(x)    (((x) & 0xFF))

/* REG_DBG_APB_PUBCP_DBG_MOD_SEL */

#define BIT_DBG_APB_PUBCP_DBG_MOD_SEL(x)  (((x) & 0xFF))

/* REG_DBG_APB_PUBCP_DBG_SIG_SEL */

#define BIT_DBG_APB_PUBCP_DBG_SIG_SEL(x)  (((x) & 0xFF))

/* REG_DBG_APB_GPU_DBG_MOD_SEL */

#define BIT_DBG_APB_GPU_DBG_MOD_SEL(x)    (((x) & 0xFF))

/* REG_DBG_APB_GPU_DBG_SIG_SEL */

#define BIT_DBG_APB_GPU_DBG_SIG_SEL(x)    (((x) & 0xFF))

/* REG_DBG_APB_MM_DBG_MOD_SEL */

#define BIT_DBG_APB_MM_DBG_MOD_SEL(x)     (((x) & 0xFF))

/* REG_DBG_APB_MM_DBG_SIG_SEL */

#define BIT_DBG_APB_MM_DBG_SIG_SEL(x)     (((x) & 0xFF))

/* REG_DBG_APB_AP_DBG_MOD_SEL */

#define BIT_DBG_APB_AP_DBG_MOD_SEL(x)     (((x) & 0xFF))

/* REG_DBG_APB_AP_DBG_SIG_SEL */

#define BIT_DBG_APB_AP_DBG_SIG_SEL(x)     (((x) & 0xFF))

/* REG_DBG_APB_MDAR_DBG_MOD_SEL */

#define BIT_DBG_APB_MDAR_DBG_MOD_SEL(x)   (((x) & 0xFF))

/* REG_DBG_APB_MDAR_DBG_SIG_SEL */

#define BIT_DBG_APB_MDAR_DBG_SIG_SEL(x)   (((x) & 0xFF))

/* REG_DBG_APB_DISP_DBG_MOD_SEL */

#define BIT_DBG_APB_DISP_DBG_MOD_SEL(x)   (((x) & 0xFF))

/* REG_DBG_APB_DISP_DBG_SIG_SEL */

#define BIT_DBG_APB_DISP_DBG_SIG_SEL(x)   (((x) & 0xFF))

/* REG_DBG_APB_WCN_DBG_MOD_SEL */

#define BIT_DBG_APB_WCN_DBG_MOD_SEL(x)    (((x) & 0xFF))

/* REG_DBG_APB_WCN_DBG_SIG_SEL */

#define BIT_DBG_APB_WCN_DBG_SIG_SEL(x)    (((x) & 0xFF))

/* REG_DBG_APB_VSP_DBG_MOD_SEL */

#define BIT_DBG_APB_VSP_DBG_MOD_SEL(x)    (((x) & 0xFF))

/* REG_DBG_APB_VSP_DBG_SIG_SEL */

#define BIT_DBG_APB_VSP_DBG_SIG_SEL(x)    (((x) & 0xFF))

/* REG_DBG_APB_DBG_BUS0_SEL */

#define BIT_DBG_APB_DBG_BUS0_SEL(x)       (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS1_SEL */

#define BIT_DBG_APB_DBG_BUS1_SEL(x)       (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS2_SEL */

#define BIT_DBG_APB_DBG_BUS2_SEL(x)       (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS3_SEL */

#define BIT_DBG_APB_DBG_BUS3_SEL(x)       (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS4_SEL */

#define BIT_DBG_APB_DBG_BUS4_SEL(x)       (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS5_SEL */

#define BIT_DBG_APB_DBG_BUS5_SEL(x)       (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS6_SEL */

#define BIT_DBG_APB_DBG_BUS6_SEL(x)       (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS7_SEL */

#define BIT_DBG_APB_DBG_BUS7_SEL(x)       (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS8_SEL */

#define BIT_DBG_APB_DBG_BUS8_SEL(x)       (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS9_SEL */

#define BIT_DBG_APB_DBG_BUS9_SEL(x)       (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS10_SEL */

#define BIT_DBG_APB_DBG_BUS10_SEL(x)      (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS11_SEL */

#define BIT_DBG_APB_DBG_BUS11_SEL(x)      (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS12_SEL */

#define BIT_DBG_APB_DBG_BUS12_SEL(x)      (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS13_SEL */

#define BIT_DBG_APB_DBG_BUS13_SEL(x)      (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS14_SEL */

#define BIT_DBG_APB_DBG_BUS14_SEL(x)      (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS15_SEL */

#define BIT_DBG_APB_DBG_BUS15_SEL(x)      (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS16_SEL */

#define BIT_DBG_APB_DBG_BUS16_SEL(x)      (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS17_SEL */

#define BIT_DBG_APB_DBG_BUS17_SEL(x)      (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS18_SEL */

#define BIT_DBG_APB_DBG_BUS18_SEL(x)      (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS19_SEL */

#define BIT_DBG_APB_DBG_BUS19_SEL(x)      (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS20_SEL */

#define BIT_DBG_APB_DBG_BUS20_SEL(x)      (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS21_SEL */

#define BIT_DBG_APB_DBG_BUS21_SEL(x)      (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS22_SEL */

#define BIT_DBG_APB_DBG_BUS22_SEL(x)      (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS23_SEL */

#define BIT_DBG_APB_DBG_BUS23_SEL(x)      (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS24_SEL */

#define BIT_DBG_APB_DBG_BUS24_SEL(x)      (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS25_SEL */

#define BIT_DBG_APB_DBG_BUS25_SEL(x)      (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS26_SEL */

#define BIT_DBG_APB_DBG_BUS26_SEL(x)      (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS27_SEL */

#define BIT_DBG_APB_DBG_BUS27_SEL(x)      (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS28_SEL */

#define BIT_DBG_APB_DBG_BUS28_SEL(x)      (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS29_SEL */

#define BIT_DBG_APB_DBG_BUS29_SEL(x)      (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS30_SEL */

#define BIT_DBG_APB_DBG_BUS30_SEL(x)      (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS31_SEL */

#define BIT_DBG_APB_DBG_BUS31_SEL(x)      (((x) & 0x3F))

/* REG_DBG_APB_DBG_BUS_DATA_A */

#define BIT_DBG_APB_DBG_BUS_DATA_A(x)     (((x) & 0xFFFFFFFF))

/* REG_DBG_APB_DBG_BUS_DATA_B */

#define BIT_DBG_APB_DBG_BUS_DATA_B(x)     (((x) & 0xFFFFFFFF))

/* REG_DBG_APB_DBG_BUS_DATA */

#define BIT_DBG_APB_DBG_BUS_DATA(x)       (((x) & 0xFFFFFFFF))


#endif /* DBG_APB_H */


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


#ifndef GPU_APB_H
#define GPU_APB_H



#define REG_GPU_APB_APB_RST              (0x0000)
#define REG_GPU_APB_CLK_GPU_CORE         (0x0004)
#define REG_GPU_APB_CLK_GPU_MEM          (0x0008)
#define REG_GPU_APB_CLK_GPU_SYS          (0x000C)
#define REG_GPU_APB_GPU_MEMIF0_QOS       (0x0010)
#define REG_GPU_APB_LPC_AB_W0            (0x0014)
#define REG_GPU_APB_LPC_M0               (0x0018)
#define REG_GPU_APB_LPC_S0               (0x001C)
#define REG_GPU_APB_LPC_S1               (0x0020)
#define REG_GPU_APB_ASYBC_BRIDGE_TOP_W0  (0x0024)
#define REG_GPU_APB_GPU_MEMIF1_QOS       (0x0028)
#define REG_GPU_APB_LPC_AB_W1            (0x002C)
#define REG_GPU_APB_ASYBC_BRIDGE_TOP_W1  (0x0030)
#define REG_GPU_APB_CLK_GPU_DVFS         (0x0034)

/* REG_GPU_APB_APB_RST */

#define BIT_GPU_APB_GPU_DUSTB_SOFT_RST          BIT(2)
#define BIT_GPU_APB_SYS_SOFT_RST_REQ_CORE       BIT(1)
#define BIT_GPU_APB_GPU_CORE_SOFT_RST           BIT(0)

/* REG_GPU_APB_CLK_GPU_CORE */

#define BIT_GPU_APB_CGM_GPU_CORE_FDIV_DENOM(x)  (((x) & 0xF) << 20)
#define BIT_GPU_APB_CGM_GPU_CORE_FDIV_NUM(x)    (((x) & 0xF) << 16)
#define BIT_GPU_APB_CGM_GPU_CORE_DIV(x)         (((x) & 0x7) << 8)
#define BIT_GPU_APB_CGM_GPU_CORE_SEL(x)         (((x) & 0x7) << 4)
#define BIT_GPU_APB_CGM_GPU_CORE_EB             BIT(0)

/* REG_GPU_APB_CLK_GPU_MEM */

#define BIT_GPU_APB_CGM_GPU_MEM_FDIV_DENOM(x)   (((x) & 0xF) << 20)
#define BIT_GPU_APB_CGM_GPU_MEM_FDIV_NUM(x)     (((x) & 0xF) << 16)
#define BIT_GPU_APB_CGM_GPU_MEM_DIV(x)          (((x) & 0x7) << 8)
#define BIT_GPU_APB_CGM_GPU_MEM_SEL(x)          (((x) & 0x7) << 4)
#define BIT_GPU_APB_CGM_GPU_MEM_EB              BIT(0)

/* REG_GPU_APB_CLK_GPU_SYS */

#define BIT_GPU_APB_CGM_GPU_SYS_DIV(x)          (((x) & 0x7) << 4)
#define BIT_GPU_APB_CGM_GPU_SYS_EB              BIT(0)

/* REG_GPU_APB_GPU_MEMIF0_QOS */

#define BIT_GPU_APB_GPU_QOS_SEL0                BIT(16)
#define BIT_GPU_APB_AWQOS_THRESHOLD_GPU0(x)     (((x) & 0xF) << 12)
#define BIT_GPU_APB_ARQOS_THRESHOLD_GPU0(x)     (((x) & 0xF) << 8)
#define BIT_GPU_APB_AWQOS_GPU0(x)               (((x) & 0xF) << 4)
#define BIT_GPU_APB_ARQOS_GPU0(x)               (((x) & 0xF))

/* REG_GPU_APB_LPC_AB_W0 */

#define BIT_GPU_APB_AXI_LP_CTRL_DISABLE0        BIT(20)
#define BIT_GPU_APB_LP_EB_AB_W0                 BIT(16)
#define BIT_GPU_APB_LP_NUM_AB_W0(x)             (((x) & 0xFFFF))

/* REG_GPU_APB_LPC_M0 */

#define BIT_GPU_APB_LP_EB_M0                    BIT(16)
#define BIT_GPU_APB_LP_NUM_M0(x)                (((x) & 0xFFFF))

/* REG_GPU_APB_LPC_S0 */

#define BIT_GPU_APB_LP_EB_S0                    BIT(16)
#define BIT_GPU_APB_LP_NUM_S0(x)                (((x) & 0xFFFF))

/* REG_GPU_APB_LPC_S1 */

#define BIT_GPU_APB_LP_EB_S1                    BIT(16)
#define BIT_GPU_APB_LP_NUM_S1(x)                (((x) & 0xFFFF))

/* REG_GPU_APB_ASYBC_BRIDGE_TOP_W0 */

#define BIT_GPU_APB_RST_SUBSYS0                 BIT(2)
#define BIT_GPU_APB_BRIDGE_TRANS_IDLE0          BIT(1)
#define BIT_GPU_APB_AXI_DETECTOR_OVERFLOW0      BIT(0)

/* REG_GPU_APB_GPU_MEMIF1_QOS */

#define BIT_GPU_APB_GPU_QOS_SEL1                BIT(16)
#define BIT_GPU_APB_AWQOS_THRESHOLD_GPU1(x)     (((x) & 0xF) << 12)
#define BIT_GPU_APB_ARQOS_THRESHOLD_GPU1(x)     (((x) & 0xF) << 8)
#define BIT_GPU_APB_AWQOS_GPU1(x)               (((x) & 0xF) << 4)
#define BIT_GPU_APB_ARQOS_GPU1(x)               (((x) & 0xF))

/* REG_GPU_APB_LPC_AB_W1 */

#define BIT_GPU_APB_AXI_LP_CTRL_DISABLE1        BIT(20)
#define BIT_GPU_APB_LP_EB_AB_W1                 BIT(16)
#define BIT_GPU_APB_LP_NUM_AB_W1(x)             (((x) & 0xFFFF))

/* REG_GPU_APB_ASYBC_BRIDGE_TOP_W1 */

#define BIT_GPU_APB_RST_SUBSYS1                 BIT(2)
#define BIT_GPU_APB_BRIDGE_TRANS_IDLE1          BIT(1)
#define BIT_GPU_APB_AXI_DETECTOR_OVERFLOW1      BIT(0)

/* REG_GPU_APB_CLK_GPU_DVFS */

#define BIT_GPU_APB_CGM_GPU_DVFS_SEL            BIT(4)
#define BIT_GPU_APB_CGM_GPU_DVFS_EN             BIT(0)


#endif /* GPU_APB_H */



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


#ifndef MM_CLK_CORE_H
#define MM_CLK_CORE_H



#define REG_MM_CLK_CORE_CGM_MIPI_CSI_CFG    (0x0020)
#define REG_MM_CLK_CORE_CGM_MIPI_CSI_S_CFG  (0x0024)
#define REG_MM_CLK_CORE_CGM_MIPI_CSI_T_CFG  (0x0028)

/* REG_MM_CLK_CORE_CGM_MIPI_CSI_CFG */

#define BIT_MM_CLK_CORE_CGM_MIPI_CSI_CFG_CGM_MIPI_CSI_PAD_SEL      BIT(16)
#define BIT_MM_CLK_CORE_CGM_MIPI_CSI_CFG_CGM_MIPI_CSI_SEL          BIT(0)

/* REG_MM_CLK_CORE_CGM_MIPI_CSI_S_CFG */

#define BIT_MM_CLK_CORE_CGM_MIPI_CSI_S_CFG_CGM_MIPI_CSI_S_PAD_SEL  BIT(16)
#define BIT_MM_CLK_CORE_CGM_MIPI_CSI_S_CFG_CGM_MIPI_CSI_S_SEL      BIT(0)

/* REG_MM_CLK_CORE_CGM_MIPI_CSI_T_CFG */

#define BIT_MM_CLK_CORE_CGM_MIPI_CSI_T_CFG_CGM_MIPI_CSI_T_PAD_SEL  BIT(16)
#define BIT_MM_CLK_CORE_CGM_MIPI_CSI_T_CFG_CGM_MIPI_CSI_T_SEL      BIT(0)


#endif /* MM_CLK_CORE_H */



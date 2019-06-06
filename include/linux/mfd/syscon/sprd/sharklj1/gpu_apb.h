/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This file is dual-licensed: you can use it either under the terms
 * of the GPL or the X11 license, at your option. Note that this dual
 * licensing only applies to this file, and not this project as a
 * whole.
 *
 */


#ifndef _GPU_APB_REG_H
#define _GPU_APB_REG_H



#define REG_GPU_APB_APB_RST                 0x0000
#define REG_GPU_APB_APB_CLK_CTRL            0x0004
#define REG_GPU_APB_APB_BARRIER_CTRL        0x0008
#define REG_GPU_APB_GPU_PATH                0x000C
#define REG_GPU_APB_GPUD_ACTIVE             0x0010
#define REG_GPU_APB_GPU_MTX_SYNC_STAGE      0x0020
#define REG_GPU_APB_CGM_GPU_FDIV            0x0080
#define REG_GPU_APB_GPU_PATH_SEL            0x0084

/*---------------------------------------------------------------------------
// Register Name   : REG_GPU_APB_APB_RST
// Register Offset : 0x0000
// Description     :
---------------------------------------------------------------------------*/

#define BIT_GPU_APB_GPU_SOFT_RST                            BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_GPU_APB_APB_CLK_CTRL
// Register Offset : 0x0004
// Description     :
---------------------------------------------------------------------------*/

#define BIT_GPU_APB_CLK_GPU_DIV(x)                          (((x) & 0x3) << 4)
#define BIT_GPU_APB_CLK_GPU_SEL(x)                          (((x) & 0x7))

/*---------------------------------------------------------------------------
// Register Name   : REG_GPU_APB_APB_BARRIER_CTRL
// Register Offset : 0x0008
// Description     :
---------------------------------------------------------------------------*/

#define BIT_GPU_APB_GPU_BARRIER_DISABLE_EN                  BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_GPU_APB_GPU_PATH
// Register Offset : 0x000C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_GPU_APB_GPU_PATH                                BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_GPU_APB_GPUD_ACTIVE
// Register Offset : 0x0010
// Description     :
---------------------------------------------------------------------------*/

#define BIT_GPU_APB_GDC_ACTIVE                              BIT(2)
#define BIT_GPU_APB_GDG_ACTIVE                              BIT(1)
#define BIT_GPU_APB_GDL_ACTIVE                              BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_GPU_APB_GPU_MTX_SYNC_STAGE
// Register Offset : 0x0020
// Description     :
---------------------------------------------------------------------------*/

#define BIT_GPU_APB_GPU_MTX_SYNC_STAGE                      BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_GPU_APB_CGM_GPU_FDIV
// Register Offset : 0x0080
// Description     :
---------------------------------------------------------------------------*/

#define BIT_GPU_APB_CGM_GPU_FDIV_NUM(x)                     (((x) & 0xF) << 16)
#define BIT_GPU_APB_CGM_GPU_FDIV_DENOM(x)                   (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_GPU_APB_GPU_PATH_SEL
// Register Offset : 0x0084
// Description     :
---------------------------------------------------------------------------*/

#define BIT_GPU_APB_GPU_PATH_SEL                            BIT(0)


#endif

/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This file is dual-licensed: you can use it either under the terms
 * of the GPL or the X11 license, at your option. Note that this dual
 * licensing only applies to this file, and not this project as a
 * whole.
 *
 */


#ifndef _MM_AHB_REG_H
#define _MM_AHB_REG_H



#define REG_MM_AHB_AHB_EB              0x0000
#define REG_MM_AHB_AHB_RST             0x0004
#define REG_MM_AHB_GEN_CKG_CFG         0x0008
#define REG_MM_AHB_MIPI_CSI2_CTRL      0x000C
#define REG_MM_AHB_CSI_IDI_SW_CFG      0x0010
#define REG_MM_AHB_CSI_IDI_SW_INT_EN   0x0014
#define REG_MM_AHB_CSI_IDI_SW_INT_CLR  0x0018
#define REG_MM_AHB_CSI_IDI_SW_STATE    0x001C
#define REG_MM_AHB_CSI_IDI_SW_INT      0x0020
#define REG_MM_AHB_CGM_ISP_FDIV        0x0024
#define REG_MM_AHB_CGM_DCAM_FDIV       0x0028
#define REG_MM_AHB_CGM_CPP_FDIV        0x002C
#define REG_MM_AHB_CGM_JPG_FDIV        0x0030
#define REG_MM_AHB_CGM_VSP_FDIV        0x0034

/*---------------------------------------------------------------------------
// Register Name   : REG_MM_AHB_AHB_EB
// Register Offset : 0x0000
// Description     :
---------------------------------------------------------------------------*/

#define BIT_MM_AHB_CBM_EB                                   BIT(8)
#define BIT_MM_AHB_CPP_EB                                   BIT(7)
#define BIT_MM_AHB_CKG_EB                                   BIT(6)
#define BIT_MM_AHB_JPG_EB                                   BIT(5)
#define BIT_MM_AHB_CSI_S_EB                                 BIT(4)
#define BIT_MM_AHB_CSI_EB                                   BIT(3)
#define BIT_MM_AHB_VSP_EB                                   BIT(2)
#define BIT_MM_AHB_ISP_EB                                   BIT(1)
#define BIT_MM_AHB_DCAM_EB                                  BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_MM_AHB_AHB_RST
// Register Offset : 0x0004
// Description     :
---------------------------------------------------------------------------*/

#define BIT_MM_AHB_ISP_FMCU_SOFT_RST                        BIT(15)
#define BIT_MM_AHB_AXI_MTX_VSP_SOFT_RST                     BIT(14)
#define BIT_MM_AHB_AXI_MTX_CAM_SOFT_RST                     BIT(13)
#define BIT_MM_AHB_MIPI_S_SOFT_RST                          BIT(12)
#define BIT_MM_AHB_CSI_IDI_SW_SOFT_RST                      BIT(11)
#define BIT_MM_AHB_CSI_S_SOFT_RST                           BIT(10)
#define BIT_MM_AHB_CPP_SOFT_RST                             BIT(9)
#define BIT_MM_AHB_CKG_SOFT_RST                             BIT(8)
#define BIT_MM_AHB_CAM0_SOFT_RST                            BIT(7)
#define BIT_MM_AHB_JPG_SOFT_RST                             BIT(6)
#define BIT_MM_AHB_CSI_SOFT_RST                             BIT(5)
#define BIT_MM_AHB_VSP_SOFT_RST                             BIT(4)
#define BIT_MM_AHB_ISP_CFG_SOFT_RST                         BIT(3)
#define BIT_MM_AHB_ISP_LOG_SOFT_RST                         BIT(2)
#define BIT_MM_AHB_MIPI_SOFT_RST                            BIT(1)
#define BIT_MM_AHB_DCAM_SOFT_RST                            BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_MM_AHB_GEN_CKG_CFG
// Register Offset : 0x0008
// Description     :
---------------------------------------------------------------------------*/

#define BIT_MM_AHB_MIPI_CSI_S_CKG_EN                        BIT(5)
#define BIT_MM_AHB_MIPI_CSI_CKG_EN                          BIT(4)
#define BIT_MM_AHB_ISP_AXI_CKG_EN                           BIT(3)
#define BIT_MM_AHB_SENSOR1_CKG_EN                           BIT(2)
#define BIT_MM_AHB_SENSOR0_CKG_EN                           BIT(1)
#define BIT_MM_AHB_CPHY_CFG_CKG_EN                          BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_MM_AHB_MIPI_CSI2_CTRL
// Register Offset : 0x000C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_MM_AHB_CSI_2P2L_TESTDOUT_MS_SEL                 BIT(1)
#define BIT_MM_AHB_MIPI_CPHY_SEL                            BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_MM_AHB_CSI_IDI_SW_CFG
// Register Offset : 0x0010
// Description     :
---------------------------------------------------------------------------*/

#define BIT_MM_AHB_AUTO_SMOOTH_SW                           BIT(1)
#define BIT_MM_AHB_FORCE_TO_CSI1                            BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_MM_AHB_CSI_IDI_SW_INT_EN
// Register Offset : 0x0014
// Description     :
---------------------------------------------------------------------------*/

#define BIT_MM_AHB_CSI_IDI_SW_INT_EN                        BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_MM_AHB_CSI_IDI_SW_INT_CLR
// Register Offset : 0x0018
// Description     :
---------------------------------------------------------------------------*/

#define BIT_MM_AHB_CSI_IDI_SW_INT_CLR                       BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_MM_AHB_CSI_IDI_SW_STATE
// Register Offset : 0x001C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_MM_AHB_CSI_IDI_SW_STATE(x)                      (((x) & 0x7))

/*---------------------------------------------------------------------------
// Register Name   : REG_MM_AHB_CSI_IDI_SW_INT
// Register Offset : 0x0020
// Description     :
---------------------------------------------------------------------------*/

#define BIT_MM_AHB_CSI_IDI_SW_DONE_STS                      BIT(1)
#define BIT_MM_AHB_CSI_IDI_SW_DONE_RAW                      BIT(0)

/*---------------------------------------------------------------------------
// Register Name   : REG_MM_AHB_CGM_ISP_FDIV
// Register Offset : 0x0024
// Description     :
---------------------------------------------------------------------------*/

#define BIT_MM_AHB_CGM_ISP_FDIV_NUM(x)                      (((x) & 0xF) << 16)
#define BIT_MM_AHB_CGM_ISP_FDIV_DENOM(x)                    (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_MM_AHB_CGM_DCAM_FDIV
// Register Offset : 0x0028
// Description     :
---------------------------------------------------------------------------*/

#define BIT_MM_AHB_CGM_DCAM_FDIV_NUM(x)                     (((x) & 0xF) << 16)
#define BIT_MM_AHB_CGM_DCAM_FDIV_DENOM(x)                   (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_MM_AHB_CGM_CPP_FDIV
// Register Offset : 0x002C
// Description     :
---------------------------------------------------------------------------*/

#define BIT_MM_AHB_CGM_CPP_FDIV_NUM(x)                      (((x) & 0xF) << 16)
#define BIT_MM_AHB_CGM_CPP_FDIV_DENOM(x)                    (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_MM_AHB_CGM_JPG_FDIV
// Register Offset : 0x0030
// Description     :
---------------------------------------------------------------------------*/

#define BIT_MM_AHB_CGM_JPG_FDIV_NUM(x)                      (((x) & 0xF) << 16)
#define BIT_MM_AHB_CGM_JPG_FDIV_DENOM(x)                    (((x) & 0xF))

/*---------------------------------------------------------------------------
// Register Name   : REG_MM_AHB_CGM_VSP_FDIV
// Register Offset : 0x0034
// Description     :
---------------------------------------------------------------------------*/

#define BIT_MM_AHB_CGM_VSP_FDIV_NUM(x)                      (((x) & 0xF) << 16)
#define BIT_MM_AHB_CGM_VSP_FDIV_DENOM(x)                    (((x) & 0xF))


#endif

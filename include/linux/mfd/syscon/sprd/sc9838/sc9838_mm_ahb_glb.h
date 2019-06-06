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


#define REG_MM_AHB_AHB_EB                     0x0000
#define REG_MM_AHB_AHB_RST                    0x0004
#define REG_MM_AHB_GEN_CKG_CFG                0x0008
#define REG_MM_AHB_MIPI_CSI2_CTRL             0x000C
#define REG_MM_AHB_MM_QOS_CFG0                0x0010
#define REG_MM_AHB_MM_QOS_CFG1                0x0014

/*---------------------------------------------------------------------------
** Register Name   : REG_MM_AHB_AHB_EB
** Register Offset : 0x0000
** Description     :
---------------------------------------------------------------------------*/

#define BIT_MM_AHB_VPP_EB                     BIT(8)
#define BIT_MM_AHB_MMU_EB                     BIT(7)
#define BIT_MM_AHB_CKG_EB                     BIT(6)
#define BIT_MM_AHB_JPG_EB                     BIT(5)
#define BIT_MM_AHB_CSI_EB                     BIT(4)
#define BIT_MM_AHB_VSP_EB                     BIT(3)
#define BIT_MM_AHB_ISP_EB                     BIT(2)
#define BIT_MM_AHB_CCIR_EB                    BIT(1)
#define BIT_MM_AHB_DCAM_EB                    BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_MM_AHB_AHB_RST
** Register Offset : 0x0004
** Description     :
---------------------------------------------------------------------------*/

#define BIT_MM_AHB_VPP_SOFT_RST               BIT(15)
#define BIT_MM_AHB_MMU_SOFT_RST               BIT(14)
#define BIT_MM_AHB_CKG_SOFT_RST               BIT(13)
#define BIT_MM_AHB_MM_MTX_SOFT_RST            BIT(12)
#define BIT_MM_AHB_OR1200_SOFT_RST            BIT(11)
#define BIT_MM_AHB_ROT_SOFT_RST               BIT(10)
#define BIT_MM_AHB_CAM2_SOFT_RST              BIT(9)
#define BIT_MM_AHB_CAM1_SOFT_RST              BIT(8)
#define BIT_MM_AHB_CAM0_SOFT_RST              BIT(7)
#define BIT_MM_AHB_JPG_SOFT_RST               BIT(6)
#define BIT_MM_AHB_CSI_SOFT_RST               BIT(5)
#define BIT_MM_AHB_VSP_SOFT_RST               BIT(4)
#define BIT_MM_AHB_ISP_CFG_SOFT_RST           BIT(3)
#define BIT_MM_AHB_ISP_LOG_SOFT_RST           BIT(2)
#define BIT_MM_AHB_CCIR_SOFT_RST              BIT(1)
#define BIT_MM_AHB_DCAM_SOFT_RST              BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_MM_AHB_GEN_CKG_CFG
** Register Offset : 0x0008
** Description     :
---------------------------------------------------------------------------*/

#define BIT_MM_AHB_VPP_AXI_CKG_EN             BIT(9)
#define BIT_MM_AHB_MM_MTX_AXI_CKG_EN          BIT(8)
#define BIT_MM_AHB_MM_AXI_CKG_EN              BIT(7)
#define BIT_MM_AHB_JPG_AXI_CKG_EN             BIT(6)
#define BIT_MM_AHB_VSP_AXI_CKG_EN             BIT(5)
#define BIT_MM_AHB_ISP_AXI_CKG_EN             BIT(4)
#define BIT_MM_AHB_DCAM_AXI_CKG_EN            BIT(3)
#define BIT_MM_AHB_SENSOR_CKG_EN              BIT(2)
#define BIT_MM_AHB_MIPI_CSI_CKG_EN            BIT(1)
#define BIT_MM_AHB_CPHY_CFG_CKG_EN            BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_MM_AHB_MIPI_CSI2_CTRL
** Register Offset : 0x000C
** Description     :
---------------------------------------------------------------------------*/

#define BIT_MM_AHB_MIPI_CPHY_SAMPLE_SEL(x)    (((x) & 0x3) << 3)
#define BIT_MM_AHB_MIPI_CPHY_SYNC_MODE        BIT(2)
#define BIT_MM_AHB_MIPI_CPHY_TEST_CTL         BIT(1)
#define BIT_MM_AHB_MIPI_CPHY_SEL              BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : REG_MM_AHB_MM_QOS_CFG0
** Register Offset : 0x0010
** Description     :
---------------------------------------------------------------------------*/

#define BIT_MM_AHB_QOS_R_DCAM(x)              (((x) & 0xF) << 28)
#define BIT_MM_AHB_QOS_W_DCAM(x)              (((x) & 0xF) << 24)
#define BIT_MM_AHB_QOS_R_JPG(x)               (((x) & 0xF) << 20)
#define BIT_MM_AHB_QOS_W_JPG(x)               (((x) & 0xF) << 16)
#define BIT_MM_AHB_QOS_R_ISP(x)               (((x) & 0xF) << 12)
#define BIT_MM_AHB_QOS_W_ISP(x)               (((x) & 0xF) << 8)
#define BIT_MM_AHB_QOS_R_VSP(x)               (((x) & 0xF) << 4)
#define BIT_MM_AHB_QOS_W_VSP(x)               (((x) & 0xF))

/*---------------------------------------------------------------------------
** Register Name   : REG_MM_AHB_MM_QOS_CFG1
** Register Offset : 0x0014
** Description     :
---------------------------------------------------------------------------*/

#define BIT_MM_AHB_QOS_R_VPP(x)               (((x) & 0xF) << 12)
#define BIT_MM_AHB_QOS_W_VPP(x)               (((x) & 0xF) << 8)
#define BIT_MM_AHB_QOS_R_CODEC(x)             (((x) & 0xF) << 4)
#define BIT_MM_AHB_QOS_W_CODEC(x)             (((x) & 0xF))


#endif

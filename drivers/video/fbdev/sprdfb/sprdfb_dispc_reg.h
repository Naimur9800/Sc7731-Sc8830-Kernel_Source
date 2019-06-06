/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _SPRD_DISPC_REG_H_
#define _SPRD_DISPC_REG_H_

#include <linux/io.h>
#include "sprdfb_chip_common.h"

/* DISPC regs offset */
#define DISPC_CTRL			(0x0000)
#define DISPC_SIZE_XY			(0x0004)
#define DISPC_RSTN			(0x0008)
#define DISPC_BUF_THRES			(0x000C)

#define DISPC_STS			(0x0010)
#define DISPC_LVDS_CTRL			(0x0018)
#define DISPC_IMG_CTRL			(0x0020)
#define DISPC_IMG_Y_BASE_ADDR		(0x0024)
#define DISPC_IMG_UV_BASE_ADDR		(0x0028)
#define DISPC_IMG_V_BASE_ADDR		(0x002c)

#define DISPC_IMG_SIZE_XY		(0x0030)
#define DISPC_IMG_PITCH			(0x0034)
#define DISPC_IMG_DISP_XY		(0x0038)
#define DISPC_BG_COLOR			(0x003c)

#define DISPC_OSD_CTRL			(0x0040)
#define DISPC_OSD_BASE_ADDR		(0x0044)
#define DISPC_OSD_SIZE_XY		(0x0048)
#define DISPC_OSD_PITCH			(0x004c)
#define DISPC_OSD_DISP_XY		(0x0050)
#define DISPC_OSD_ALPHA			(0x0054)
#define DISPC_OSD_CK			(0x0058)

#define DISPC_Y2R_CTRL			(0x0060)
#define DISPC_Y2R_Y_PARAM		(0x0064)
#define DISPC_Y2R_U_PARAM		(0x0068)
#define DISPC_Y2R_V_PARAM		(0x006c)

#define DISPC_INT_EN			(0x0070)
#define DISPC_INT_CLR			(0x0074)
#define DISPC_INT_STATUS		(0x0078)
#define DISPC_INT_RAW			(0x007c)

#define DISPC_DPI_CTRL			(0x0080)
#define DISPC_DPI_H_TIMING		(0x0084)
#define DISPC_DPI_V_TIMING		(0x0088)
#define DISPC_DPI_STS0			(0x008c)
#define DISPC_DPI_STS1			(0x0090)

#define DISPC_DBI_CTRL			(0x00a0)
#define DISPC_DBI_TIMING0		(0x00a4)
#define DISPC_DBI_TIMING1		(0x00a8)
#define DISPC_DBI_RDATA			(0x00ac)
#define DISPC_DBI_CMD			(0x00b0)
#define DISPC_DBI_DATA			(0x00b4)
#define DISPC_DBI_QUEUE			(0x00b8)
#define DISPC_TE_SYNC_DELAY		(0x00bc)

/* shadow register , read only */
#define SHDW_IMG_CTRL			(0x00E0)
#define SHDW_IMG_Y_BASE_ADDR		(0x00E4)
#define SHDW_IMG_UV_BASE_ADDR		(0x00E8)
#define SHDW_IMG_V_BASE_ADDR		(0x00EC)
#define SHDW_IMG_SIZE_XY		(0x00F0)
#define SHDW_IMG_PITCH			(0x00F4)
#define SHDW_IMG_DISP_XY		(0x00F8)
#define SHDW_BG_COLOR			(0x00FC)
#define SHDW_OSD_CTRL			(0x0100)
#define SHDW_OSD_BASE_ADDR		(0x0104)
#define SHDW_OSD_SIZE_XY		(0x0108)
#define SHDW_OSD_PITCH			(0x010C)
#define SHDW_OSD_DISP_XY		(0x0110)
#define SHDW_OSD_ALPHA			(0x0114)
#define SHDW_OSD_CK			(0x0118)
#define SHDW_Y2R_CTRL			(0x0120)
#define SHDW_Y2R_CONTRAST		(0x0124)
#define SHDW_Y2R_SATURATION		(0x0128)
#define SHDW_Y2R_BRIGHTNESS		(0x012C)
#define SHDW_DPI_H_TIMING		(0x0130)
#define SHDW_DPI_V_TIMING		(0x0134)

/*---------------------------------------------------------------------------
** Register Name   : DISPC_CTRL
** Register Offset : 0x0000
** Description     :
---------------------------------------------------------------------------*/
#define BIT_DISPC_REQ_MODE		BIT(18)
#define BIT_DISPC_EXP_MODE(x)		(((x) & 0x3) << 16)
#define BIT_DISPC_DISPC_GAP(x)		(((x) & 0xFF) << 8)
#define BIT_DISPC_PWR_CTRL		BIT(7)
#define BIT_DISPC_DITHER_EN		BIT(6)
#define BIT_DISPC_AXI_BUS_SWT		BIT(5)
#define BIT_DISPC_DISPC_RUN		BIT(4)
#define BIT_DISPC_EXT_DBIDPI_BYPASS	BIT(3)
#define BIT_DISPC_DISPC_IF(x)		(((x) & 0x3) << 1)
#define BIT_DISPC_DISPC_EN		BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : DISPC_SIZE_XY
** Register Offset : 0x0004
** Description     :
---------------------------------------------------------------------------*/
#define BIT_DISPC_SIZE_Y(x)	(((x) & 0xfff) << 16)
#define BIT_DISPC_SIZE_X(x)	((x) & 0xfff)

/*---------------------------------------------------------------------------
** Register Name   : DISPC_OSD_CTRL
** Register Offset : 0x0040
** Description     :
---------------------------------------------------------------------------*/
#define BIT_DISPC_OSD_BLEND_MODE(x)	(((x) & 0x3) << 16)
#define BIT_DISPC_OSD_RB_SWITCH	BIT(15)
#define BIT_DISPC_OSD_SWITCH(x)	(((x) & 0x3) << 8)
#define BIT_DISPC_OSD_FORMAT(x)	(((x) & 0xF) << 4)
#define BIT_DISPC_OSD_ALPHA_SEL(x)	(((x) & 0x3) << 2)
#define BIT_DISPC_OSD_CK_EN		BIT(1)
#define BIT_DISPC_OSD_OSD_EN		BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : DISPC_OSD_SIZE_XY
** Register Offset : 0x0040
** Description     :
---------------------------------------------------------------------------*/
#define BIT_DISPC_OSD_SIZE_Y(x)	(((x) & 0xfff) << 16)
#define BIT_DISPC_OSD_SIZE_X(x)	((x) & 0xfff)

/*---------------------------------------------------------------------------
** Register Name   : DISPC_INT_EN
** Register Offset : 0x0070
** Description     :
---------------------------------------------------------------------------*/
#define DISPC_INT_DPI_VSYNC_EN		BIT(5)
#define DISPC_INT_UPDATE_DONE_EN	BIT(4)
#define DISPC_INT_EDPI_TE_EN		BIT(3)
#define DISPC_INT_ERR_EN	BIT(2)
#define DISPC_INT_TE_EN		BIT(1)
#define DISPC_INT_DONE_EN		BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : DISPC_INT_CLR
** Register Offset : 0x0074
** Description     :
---------------------------------------------------------------------------*/
#define DISPC_INT_DPI_VSYNC_CLR		BIT(5)
#define DISPC_INT_UPDATE_DONE_CLR	BIT(4)
#define DISPC_INT_EDPI_TE_CLR		BIT(3)
#define DISPC_INT_ERR_CLR	BIT(2)
#define DISPC_INT_TE_CLR	BIT(1)
#define DISPC_INT_DONE_CLR		BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : DISPC_INT_STATUS
** Register Offset : 0x0078
** Description     :
---------------------------------------------------------------------------*/
#define DISPC_INT_DPI_VSYNC_STS	BIT(5)
#define DISPC_INT_UPDATE_DONE_STS	BIT(4)
#define DISPC_INT_EDPI_TE_STS	BIT(3)
#define DISPC_INT_ERR_STS	BIT(2)
#define DISPC_INT_TE_STS	 BIT(1)
#define DISPC_INT_DONE_STS		BIT(0)

/*---------------------------------------------------------------------------
** Register Name   : DISPC_DPI_CTRL
** Register Offset : 0x0080
** Description     :
---------------------------------------------------------------------------*/
#define BIT_DISPC_DPI_HALT_EN	BIT(16)
#define BIT_DISPC_DSI_DPI_COLORM	BIT(15)
#define BIT_DISPC_DSI_DPI_SHDN		BIT(14)
#define BIT_DISPC_EDPI_TE_SEL		BIT(10)
#define BIT_DISPC_EDPI_TE_POL		BIT(9)
#define BIT_DISPC_EDPI_TE_EN		BIT(8)
#define BIT_DISPC_DPI_BITS(x)		(((x) & 0x3) << 6)
#define BIT_DISPC_DPI_REG_UPDATE	BIT(5)
#define BIT_DISPC_DPI_REG_UPDATE_MODE	BIT(4)
#define BIT_DISPC_DPI_RUN_MODE	BIT(3)
#define BIT_DISPC_DPI_DE_POL		BIT(2)
#define BIT_DISPC_DPI_VSYNC_POL	BIT(1)
#define BIT_DISPC_DPI_HSYNC_POL	BIT(0)

/* Dispc register operations */
static inline uint32_t dispc_read(uint32_t reg)
{
	return readl_relaxed((void __iomem *)(sprd_dispc_base + reg));
}

static inline void dispc_write(uint32_t value, uint32_t reg)
{
	writel_relaxed(value, (void __iomem *)(sprd_dispc_base + reg));
}

static inline void dispc_set_bits(uint32_t bits, uint32_t reg)
{
	dispc_write(dispc_read(reg) | bits, reg);
}

static inline void dispc_clear_bits(uint32_t bits, uint32_t reg)
{
	dispc_write(dispc_read(reg) & ~bits, reg);
}

#endif

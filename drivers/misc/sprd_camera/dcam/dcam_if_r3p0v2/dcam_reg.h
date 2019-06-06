/*
 * Copyright (C) 2015-2016 Spreadtrum Communications Inc.
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
#ifndef _DCAM_REG_H_
#define _DCAM_REG_H_

#define DCAM_BASE(idx)				(s_dcam_regbase[idx])
#define DCAM_CFG				(0x0000UL)
#define DCAM_CONTROL				(0x0004UL)
#define DCAM_PATH0_CFG				(0x0008UL)
#define DCAM_INT_STS				(0x003CUL)
#define DCAM_INT_MASK				(0x0040UL)
#define DCAM_INT_CLR				(0x0044UL)
#define DCAM_INT_RAW				(0x0048UL)
#define DCAM_FRM_ADDR0				(0x004CUL)
#define DCAM_AW_QOS				(0x0064UL)
#define DCAM_ENDIAN_SEL				(0x006CUL)
#define DCAM_AMBA_STS				(0x0070UL)
#define DCAM_AXIM_DBG_STS			(0x0074UL)

#define CAP_MIPI_CFG				(0x0128UL)
#define CAP_MIPI_FRM_CTRL			(0x012CUL)
#define CAP_MIPI_START				(0x0130UL)
#define CAP_MIPI_END				(0x0134UL)
#define CAP_MIPI_IMG_DECI			(0x0138UL)
#define CAP_MIPI_JPG_CTRL			(0x013CUL)
#define CAP_MIPI_FRM_SIZE			(0x0140UL)
#define CAP_SENSOR_CTRL				(0x0144UL)

#define PACK_CROP_START				(0x0164UL)
#define PACK_CROP_END				(0x0168UL)
#define DCAM_PDAF_CTRL				(0x016CUL)
#define FRM_ADDR_PDAF				(0x0170UL)

#define MIPI_REDUNDANT				(0x0174UL)
#define IP_REVISION				(0x0178UL)
#define ISP_OUT_FRM_SIZE			(0x017CUL)

#define DCAM_CAP_SKIP_FRM_MAX			16
#define DCAM_FRM_DECI_FAC_MAX			4
#define DCAM_CAP_FRAME_WIDTH_MAX		4640
#define DCAM_CAP_FRAME_HEIGHT_MAX		3488

#define DCAM_ISP_LINE_BUF_LENGTH		4640
#define DCAM1_ISP_LINE_BUF_LENGTH		3280
#define DCAM_SCALING_THRESHOLD			2600
#define DCAM1_SCALING_THRESHOLD			1536

#define DCAM_CAP_X_DECI_FAC_MAX			4
#define DCAM_CAP_Y_DECI_FAC_MAX			4
#define CAMERA_SC_COEFF_UP_MAX			4
#define CAMERA_SC_COEFF_DOWN_MAX		4
#define CAMERA_PATH_DECI_FAC_MAX		4

#define DCAM_IRQ				IRQ_DCAM_INT

enum dcam_id {
	DCAM_ID_0 = 0,
	DCAM_ID_1 = 1,
	DCAM_ID_MAX = 2,
};

enum camera_path_id {
	CAMERA_PATH0 = 0,
	CAMERA_PRE_PATH,
	CAMERA_VID_PATH,
	CAMERA_CAP_PATH,
	CAMERA_PDAF_PATH,
	CAMERA_MAX_PATH,
};

#define DCAM_REG_WR(idx, reg, val)  (REG_WR(DCAM_BASE(idx)+reg, val))
#define DCAM_REG_RD(idx, reg)  (REG_RD(DCAM_BASE(idx)+reg))
#define DCAM_REG_MWR(idx, reg, msk, val)  DCAM_REG_WR(idx, reg, \
	((val) & (msk)) | (DCAM_REG_RD(idx, reg) & (~(msk))))

#endif /* _DCAM_REG_H_ */

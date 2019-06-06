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
#define DCAM_PATH0_SRC_SIZE			(0x000CUL)
#define DCAM_PATH1_CFG				(0x0010UL)
#define DCAM_PATH1_SRC_SIZE			(0x0014UL)
#define DCAM_PATH1_DST_SIZE			(0x0018UL)
#define DCAM_PATH1_TRIM0_START			(0x001CUL)
#define DCAM_PATH1_TRIM0_SIZE			(0x0020UL)
#define DCAM_PATH2_CFG				(0x0024UL)
#define DCAM_PATH2_SRC_SIZE			(0x0028UL)
#define DCAM_PATH2_DST_SIZE			(0x002CUL)
#define DCAM_PATH2_TRIM0_START			(0x0030UL)
#define DCAM_PATH2_TRIM0_SIZE			(0x0034UL)
#define REV_SLICE_VER				(0x0038UL)
#define DCAM_INT_STS				(0x003CUL)
#define DCAM_INT_MASK				(0x0040UL)
#define DCAM_INT_CLR				(0x0044UL)
#define DCAM_INT_RAW				(0x0048UL)
#define DCAM_FRM_ADDR0				(0x004CUL)
#define DCAM_FRM_ADDR1				(0x0050UL)
#define DCAM_FRM_ADDR2				(0x0054UL)
#define DCAM_FRM_ADDR3				(0x0058UL)
#define DCAM_FRM_ADDR4				(0x005CUL)
#define DCAM_FRM_ADDR5				(0x0060UL)
#define DCAM_FRM_ADDR6				(0x0064UL)
#define DCAM_BURST_GAP				(0x0068UL)
#define DCAM_ENDIAN_SEL				(0x006CUL)
#define DCAM_AHBM_STS				(0x0070UL)

#define DCAM_YUV_SHRINK				(0x00B0UL)
#define DCAM_YUV_EFFECT				(0x00B4UL)
#define DCAM_FRM_ADDR12				(0x00C0UL)
#define DCAM_YUV_REGULAR			(0x00C4UL)

#define CAP_MIPI_CTRL				(0x0128UL)
#define CAP_MIPI_FRM_CTRL			(0x012CUL)
#define CAP_MIPI_START				(0x0130UL)
#define CAP_MIPI_END				(0x0134UL)
#define CAP_MIPI_IMG_DECI			(0x0138UL)
#define CAP_MIPI_JPG_CTRL			(0x013CUL)
#define CAP_MIPI_FRM_SIZE			(0x0140UL)
#define CAP_SENSOR_CTRL				(0x0144UL)

#define BLC_CFG					(0x0148UL)
#define BLC_START				(0x014CUL)
#define BLC_END					(0x0150UL)
#define BLC_TARGET				(0x0154UL)
#define BLC_MANUAL				(0x0158UL)
#define BLC_VALUE1				(0x015CUL)
#define BLC_VALUE2				(0x0160UL)

#define PACK_CROP_START				(0x0164UL)
#define PACK_CROP_END				(0x0168UL)
#define DCAM_PDAF_CTRL				(0x016CUL)

#define DCAM_PATH3_CFG				(0x0200UL)
#define DCAM_PATH1_PITCH			(0x0204UL)
#define DCAM_PATH2_PITCH			(0x0208UL)
#define DCAM_PATH3_PITCH			(0x020CUL)
#define DCAM_PATH3_SRC_SIZE			(0x0210UL)
#define DCAM_PATH3_DST_SIZE			(0x0214UL)
#define DCAM_PATH3_TRIM0_START			(0x0218UL)
#define DCAM_PATH3_TRIM0_SIZE			(0x021CUL)

#define DCAM_PATH1_SCL_IP			(0x0220UL)
#define DCAM_PATH1_SCL_CIP			(0x0224UL)
#define DCAM_PATH1_SCL_FACTOR			(0x0228UL)
#define DCAM_PATH1_TRIM1_START			(0x0230UL)
#define DCAM_PATH1_TRIM1_SIZE			(0x0234UL)

#define DCAM_PATH2_SCL_IP			(0x0240UL)
#define DCAM_PATH2_SCL_CIP			(0x0244UL)
#define DCAM_PATH2_SCL_FACTOR			(0x0248UL)
#define DCAM_PATH2_TRIM1_START			(0x0250UL)
#define DCAM_PATH2_TRIM1_SIZE			(0x0254UL)

#define DCAM_PATH3_SCL_IP			(0x0260UL)
#define DCAM_PATH3_SCL_CIP			(0x0264UL)
#define DCAM_PATH3_SCL_FACTOR			(0x0268UL)
#define DCAM_PATH3_TRIM1_START			(0x0270UL)
#define DCAM_PATH3_TRIM1_SIZE			(0x0274UL)

#define BURST_GAP2				(0x0280UL)

#define DCAM_FRM_ADDR13				(0x0284UL)
#define DCAM_FRM_ADDR14				(0x0288UL)
#define DCAM_FRM_ADDR15				(0x028CUL)
#define DCAM_FRM_ADDR16				(0x0290UL)
#define DCAM_FRM_ADDR17				(0x0294UL)
#define DCAM_FRM_ADDR18				(0x0298UL)

#define JPEGLS_BSM_SIZE_Y			(0x0300UL)
#define JPEGLS_BSM_SIZE_U			(0x0304UL)
#define JPEGLS_BSM_SIZE_V			(0x0308UL)
#define JPEGLS_BSM_SIZE_THD_Y			(0x0310UL)
#define JPEGLS_BSM_SIZE_THD_U			(0x0314UL)
#define JPEGLS_BSM_SIZE_THD_V			(0x0318UL)

#define MIPI_REDUNDANT				(0x8010UL)
#define IP_REVISION				(0x8020UL)


#define DCAM_END				IP_REVISION

/* bits definitions for register REG_AHB_RST */
#define DCAM0_RST_BIT				BIT(0)
#define DCAM0_IF_RST_BIT			BIT(24)
#define DCAM0_PATH0_RST_BIT			BIT(8)
#define DCAM0_PATH1_RST_BIT			BIT(9)
#define DCAM0_PATH2_RST_BIT			BIT(10)
#define DCAM0_PATH3_RST_BIT			BIT(23)
#define DCAM0_ROT_RST_BIT			BIT(11)

#define DCAM1_RST_BIT				BIT(1)
#define DCAM1_IF_RST_BIT			BIT(26)
#define DCAM1_PATH0_RST_BIT			BIT(12)
#define DCAM1_PATH1_RST_BIT			BIT(13)
#define DCAM1_PATH2_RST_BIT			BIT(14)
#define DCAM1_PATH3_RST_BIT			BIT(25)
#define DCAM1_ROT_RST_BIT			BIT(15)

#define DCAM_CAP_SKIP_FRM_MAX			16
#define DCAM_FRM_DECI_FAC_MAX			4
#define DCAM_CAP_FRAME_WIDTH_MAX		5904
#define DCAM_CAP_FRAME_HEIGHT_MAX		4428

#define DCAM_PATH1_FRAME_WIDTH_MAX		5904
#define DCAM_PATH1_FRAME_HEIGHT_MAX		5000
#define DCAM_PATH2_FRAME_WIDTH_MAX		5904
#define DCAM_PATH2_FRAME_HEIGHT_MAX		5000
#define DCAM_PATH3_FRAME_WIDTH_MAX		5904
#define DCAM_PATH3_FRAME_HEIGHT_MAX		5000

#define DCAM_PATH1_LINE_BUF_LENGTH		2560
#define DCAM_PATH2_LINE_BUF_LENGTH		5904
#define DCAM_PATH3_LINE_BUF_LENGTH		2048

#define DCAM_ISP_LINE_BUF_LENGTH		5904
#define DCAM_SCALING_THRESHOLD			2600
#define DCAM_SIGNLE_MAX_LENGTH			4416

#define DCAM_CAP_X_DECI_FAC_MAX			4
#define DCAM_CAP_Y_DECI_FAC_MAX			4
#define DCAM_PATH_FRAME_ROT_MAX			4
#define DCAM_JPG_BUF_UNIT			(1 << 15)
#define DCAM_JPG_UNITS				(1 << 10)
#define DCAM_SC_COEFF_UP_MAX			4
#define DCAM_SC_COEFF_DOWN_MAX			4
#define DCAM_PATH_DECI_FAC_MAX			4

#define DCAM_IRQ				IRQ_DCAM_INT

#define DCAM_PIXEL_ALIGN_WIDTH			4
#define DCAM_PIXEL_ALIGN_HEIGHT			2

#define DCAM_PATH_NUM				4

enum dcam_id {
	DCAM_ID_0 = 0,
	DCAM_ID_1 = 1,
	DCAM_ID_MAX = 2,
};

enum dcam_path_id {
	DCAM_PATH0 = 0,
	DCAM_PATH1,
	DCAM_PATH2,
	DCAM_PATH3,
	DCAM_PATH_MAX
};

enum dcam_path_index {
	DCAM_PATH_IDX_NONE = 0,
	DCAM_PATH_IDX_0 = 1,
	DCAM_PATH_IDX_1 = 2,
	DCAM_PATH_IDX_2 = 4,
	DCAM_PATH_IDX_3 = 8,
	DCAM_PATH_IDX_ALL = 0xf,
};

#define DCAM_REG_WR(idx, reg, val)  (REG_WR(DCAM_BASE(idx) + reg, val))
#define DCAM_REG_RD(idx, reg)  (REG_RD(DCAM_BASE(idx) + reg))
#define DCAM_REG_MWR(idx, reg, msk, val)  DCAM_REG_WR(idx, reg, \
	((val) & (msk)) | (DCAM_REG_RD(idx, reg) & (~(msk))))

#endif /* _DCAM_REG_H_ */

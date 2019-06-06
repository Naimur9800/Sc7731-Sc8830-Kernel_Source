/*
 * Copyright (C) 2017-2018 Spreadtrum Communications Inc.
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

#include <linux/bitops.h>

extern unsigned long s_dcam_regbase[];
extern unsigned long s_dcam_aximbase;

#ifdef TEST_SHARKL3

#include "dcam_reg_l3.h"

 #else  /* SharkL5/ROC1 */

#define DCAM_BASE(idx)                 (s_dcam_regbase[idx])


/*  DCAM0/DCAM1 module registers define*/
#define DCAM_IP_REVISION            (0x0000UL)
#define DCAM_CONTROL                   (0x0004UL)
#define DCAM_CFG                            (0x0008UL)
#define DCAM_SPARE_CTRL             (0x000CUL)

struct dcam_control_field {
	uint32_t cap_frc_copy: 1;
	uint32_t cap_auto_copy: 1;
	uint32_t reserved: 2;
	uint32_t coeff_frc_copy: 1;
	uint32_t coeff_auto_copy: 1;
	uint32_t rds_frc_copy: 1;
	uint32_t rds_aoto_copy: 1;

	uint32_t full_frc_copy: 1;
	uint32_t full_aoto_copy: 1;
	uint32_t bin_frc_copy: 1;
	uint32_t bin_aoto_copy: 1;
	uint32_t pdaf_frc_copy: 1;
	uint32_t pdaf_aoto_copy: 1;
	uint32_t vch2_frc_copy: 1;
	uint32_t vch2_aoto_copy: 1;

	uint32_t vch3_frc_copy: 1;
	uint32_t vch3_aoto_copy: 1;
};
struct dcam_cfg_field {
	uint32_t cap_eb: 1;
	uint32_t full_path_eb : 1;
	uint32_t bin_path_eb: 1;
	uint32_t pdaf_path_eb: 1;
	uint32_t vch2_path_eb: 1;
	uint32_t vch3_path_eb: 1;
};

#define DCAM_FULL_PATH_STATUS          (0x0010UL)
#define DCAM_BIN_PATH_STATUS           (0x0014UL)
#define DCAM_PDAF_PATH_STATUS          (0x0018UL)
#define DCAM_VCH2_PATH_STATUS          (0x001CUL)
#define DCAM_VCH3_PATH_STATUS          (0x0020UL)

#define DCAM_PATH_BUSY                 (0x0030UL)
#define DCAM_PATH_STOP                 (0x0034UL)

struct path_stop_field {
	uint32_t full_path_stop: 1;
	uint32_t bin_path_stop: 1;
};


#define DCAM_INT_MASK                  (0x003CUL)
#define DCAM_INT_EN                       (0x0040UL)
#define DCAM_INT_CLR                     (0x0044UL)
#define DCAM_INT_RAW                   (0x0048UL)

#define DCAM_PATH_OVERFLOW          (0x004CUL)

#define DCAM_FULL_CFG                       (0x0050UL)
#define DCAM_FULL_CROP_START        (0x0054UL)
#define DCAM_FULL_CROP_SIZE           (0x0058UL)

struct full_cfg_field {
	uint32_t is_loose: 1;
	uint32_t crop_eb: 1;
	uint32_t src_sel: 1;
};

#define DCAM_CAM_BIN_CFG                (0x005CUL)
#define DCAM_BIN_CROP_START          (0x0060UL)
#define DCAM_BIN_CROP_SIZE             (0x0064UL)

struct bin_cfg_field {
	uint32_t is_loose: 1;
	uint32_t bin_ratio: 1;
	uint32_t scaler_sel: 2;
	uint32_t reserved: 12;

	uint32_t slw_en: 1;
	uint32_t slw_addr_num: 3;
};

#define DCAM_RDS_DES_SIZE               (0x0068UL)
#define DCAM_PATH_ENDIAN                (0x0070UL)

struct rds_des_field {
	uint32_t raw_downsizer_with: 13;
	uint32_t resersed0: 3;
	uint32_t raw_downsizer_height: 12;
};
struct endian_field {
	uint32_t reserved: 16;
	uint32_t full_endian: 2;
	uint32_t bin_endian: 2;
	uint32_t pdaf_endian: 2;
	uint32_t vch2_endian: 2;
	uint32_t vch3_endian: 2;
};

#define DCAM_FULL_BASE_WADDR           (0x0084UL)
#define DCAM_BIN_BASE_WADDR0           (0x0088UL)
#define DCAM_BIN_BASE_WADDR1           (0x008CUL)
#define DCAM_BIN_BASE_WADDR2           (0x0090UL)
#define DCAM_BIN_BASE_WADDR3           (0x0094UL)
#define DCAM_PDAF_BASE_WADDR           (0x0098UL)
#define DCAM_VCH2_BASE_WADDR           (0x009CUL)
#define DCAM_VCH3_BASE_WADDR           (0x00A0UL)
#define DCAM_LENS_BASE_WADDR            (0x00A4UL)
#define DCAM_AEM_BASE_WADDR             (0x00A8UL)
#define DCAM_AEM_BASE_WADDR1               (0x01D0UL)
#define DCAM_AEM_BASE_WADDR2               (0x01D4UL)
#define DCAM_AEM_BASE_WADDR3               (0x01D8UL)
#define DCAM_HIST_BASE_WADDR            (0x00B0UL)
#define DCAM_HIST_BASE_WADDR1              (0x01DCUL)
#define DCAM_HIST_BASE_WADDR2              (0x01E0UL)
#define DCAM_HIST_BASE_WADDR3              (0x01E4UL)
#define DCAM_PPE_RIGHT_WADDR            (0x00B4UL)
#define ISP_AFL_GLB_WADDR                    (0x00B8UL)
#define ISP_AFL_REGION_WADDR             (0x00BCUL)
#define ISP_BPC_MAP_ADDR                      (0x00C0UL)
#define ISP_BPC_OUT_ADDR                      (0x00C4UL)
#define ISP_AFM_BASE_WADDR                (0x00C8UL)
#define ISP_NR3_WADDR                           (0x00CCUL)

#define DCAM_MIPI_REDUNDANT             (0x00D0UL)
#define DCAM_MIPI_CAP_WORD_CNT      (0x00D4UL)
#define DCAM_MIPI_CAP_FRM_CLR          (0x00D8UL)
#define DCAM_MIPI_CAP_RAW_SIZE        (0x00DCUL)
#define DCAM_MIPI_CAP_PDAF_SIZE       (0x00E0UL)
#define DCAM_CAP_VCH2_SIZE                 (0x00E4UL)
#define DCAM_CAP_VCH3_SIZE                 (0x00E8UL)

#define DCAM_IMAGE_CONTROL            (0x00F0UL)
#define DCAM_PDAF_CONTROL              (0x00F4UL)
#define DCAM_VCH2_CONTROL                (0x00F8UL)

#define DCAM_MIPI_CAP_CFG               (0x0100UL)
#define DCAM_MIPI_CAP_FRM_CTRL    (0x0104UL)
#define DCAM_MIPI_CAP_START           (0x0108UL)
#define DCAM_MIPI_CAP_END               (0x010CUL)

#define ISP_RGBG_YRANDOM_STATUS0       (0x0110UL)
#define ISP_RGBG_YRANDOM_STATUS1       (0x0110UL)
#define ISP_RGBG_YRANDOM_STATUS2       (0x0110UL)

#define DCAM_PPE_FRM_CTRL0   (0x0120UL)
#define DCAM_PPE_FRM_CTRL1   (0x0124UL)
#define DCAM_PPE_STATUS          (0x0128UL)

#define DCAM_LENS_GRID_NUMBER      (0x0130UL)
#define DCAM_LENS_GRID_SIZE             (0x0134UL)
#define DCAM_LENS_LOAD_EB                (0x0138UL)
#define DCAM_LENS_LOAD_CLR              (0x013CUL)
#define DCAM_APB_SRAM_CTRL             (0x0140UL)
#define DCAM_LENS_STATS                     (0x0144UL)

#define DCAM_AEM_FRM_CTRL0     (0x0150UL)
#define DCAM_AEM_FRM_CTRL1     (0x0154UL)
#define DCAM_AEM_STATUS            (0x0158UL)

#define DCAM_HIST_FRM_CTRL0     (0x0160UL)
#define DCAM_HIST_FRM_CTRL1     (0x0164UL)
#define DCAM_HIST_STATUS            (0x016CUL)

#define ISP_AFL_FRM_CTRL0       (0x0170UL)
#define ISP_AFL_FRM_CTRL1       (0x0174UL)
#define ISP_AFL_STATUS0            (0x0178UL)
#define ISP_AFL_STATUS1            (0x017CUL)
#define ISP_AFL_STATUS2            (0x0180UL)

#define ISP_AWBC_STATUS         (0x0188UL)

#define ISP_BPC_LAST_ADDR       (0x0190UL)
#define ISP_BPC_STATUS0            (0x0194UL)
#define ISP_BPC_STATUS1            (0x0198UL)

#define ISP_AFM_FRM_CTRL0       (0x01A0UL)
#define ISP_AFM_FRM_CTRL1       (0x01A4UL)
#define ISP_AFM_TILE_CNT_OUT (0x01A8UL)
#define ISP_AFM_CNT_OUT           (0x01ACUL)
#define ISP_AFM_STATUS0            (0x01B0UL)
#define ISP_AFM_STATUS1            (0x01B4UL)


#define NR3_FAST_ME_PARAM             (0x03F0UL)
#define NR3_FAST_ME_ROI_PARAM0  (0x03F4UL)
#define NR3_FAST_ME_ROI_PARAM1  (0x03F8UL)
#define NR3_FAST_ME_OUT0       (0x01C0UL)
#define NR3_FAST_ME_OUT1       (0x01C4UL)
#define NR3_FAST_ME_STATUS   (0x01C8UL)

#define ISP_BPC_PARAM                    (0x0200UL)
#define ISP_BPC_BAD_PIXEL_TH0          (0x0208UL)
#define ISP_BPC_BAD_PIXEL_TH1          (0x020cUL)
#define ISP_BPC_BAD_PIXEL_TH2          (0x0210UL)
#define ISP_BPC_BAD_PIXEL_TH3          (0x0214UL)
#define ISP_BPC_FLAT_TH                       (0x0218UL)
#define ISP_BPC_EDGE_RATIO                (0x021CUL)
#define ISP_BPC_BAD_PIXEL_PARAM     (0x0220UL)
#define ISP_BPC_BAD_PIXEL_COEF        (0x0224UL)
#define ISP_BPC_LUTWORD0                   (0x0228UL)
#define ISP_BPC_LUTWORD1                   (0x022CUL)
#define ISP_BPC_LUTWORD2                   (0x0230UL)
#define ISP_BPC_LUTWORD3                   (0x0234UL)
#define ISP_BPC_LUTWORD4                   (0x0238UL)
#define ISP_BPC_LUTWORD5                   (0x023CUL)
#define ISP_BPC_LUTWORD6                   (0x0240UL)
#define ISP_BPC_LUTWORD7                   (0x0244UL)
#define ISP_ZZBPC_PPI_RANG            (0x025C)
#define ISP_ZZBPC_PPI_RANG1          (0x0260)

#define DCAM_BLC_PARA_R_B            (0x0268UL)
#define DCAM_BLC_PARA_G                (0x026CUL)

#define ISP_RGBG_RB                    (0x0270UL)
#define ISP_RGBG_G                      (0x0274UL)
#define ISP_RGBG_YRANDOM_PARAM0        (0x0278UL)
#define ISP_RGBG_YRANDOM_PARAM1        (0x027CUL)
#define ISP_RGBG_YRANDOM_PARAM2        (0x0280UL)

#define ISP_PPI_PARAM               (0x0284UL)
#define ISP_PPI_BLOCK_COL       (0x0288UL)
#define ISP_PPI_BLOCK_ROW      (0x028CUL)
#define ISP_PPI_GAIN_PARA0     (0x0290UL)
#define ISP_PPI_PATTERN01       (0x02A8UL)
#define ISP_PPI_PATTERN32       (0x0324UL)

#define DCAM_AEM_OFFSET                   (0x0328UL)
#define DCAM_AEM_BLK_NUM                (0x032CUL)
#define DCAM_AEM_BLK_SIZE                (0x0330UL)
#define DCAM_AEM_RED_THR                 (0x0334UL)
#define DCAM_AEM_BLUE_THR               (0x0338UL)
#define DCAM_AEM_GREEN_THR             (0x033CUL)

#define DCAM_BAYER_HIST_START  (0x034CUL)
#define DCAM_BAYER_HIST_END     (0x0350UL)

#define ISP_AFL_PARAM0     (0x0354UL)
#define ISP_AFL_PARAM1     (0x0358UL)
#define ISP_AFL_PARAM2     (0x035CUL)

#define ISP_AFL_COL_POS     (0x0360UL)
#define ISP_AFL_REGION0     (0x0364UL)
#define ISP_AFL_REGION1     (0x0368UL)
#define ISP_AFL_REGION2     (0x036CUL)
#define ISP_AFL_SUM1           (0x0370UL)
#define ISP_AFL_SUM2           (0x0374UL)

#define DCAM_CROP0_START  (0x0378UL)
#define DCAM_CROP0_X          (0x037CUL)

#define ISP_AWBC_GAIN0                 (0x0380UL)
#define ISP_AWBC_GAIN1                 (0x0384UL)
#define ISP_AWBC_THRD                   (0x0388UL)
#define ISP_AWBC_OFFSET0              (0x038CUL)
#define ISP_AWBC_OFFSET1              (0x0390UL)

#define ISP_AFM_PARAMS               (0x0394UL)
#define ISP_AFM_ENHANCE_CTRL   (0x0398UL)
#define ISP_AFM_CROP_START       (0x039CUL)
#define ISP_AFM_CROP_SIZE          (0x03A0UL)
#define ISP_AFM_WIN_RANGE0S        (0x03A4UL)
#define ISP_AFM_WIN_RANGE0E        (0x03A8UL)
#define ISP_AFM_WIN_RANGE1S        (0x03ACUL)
#define ISP_AFM_IIR_FILTER0           (0x02B0UL)
#define ISP_AFM_IIR_FILTER1           (0x02B4UL)
#define ISP_AFM_IIR_FILTER2           (0x02B8UL)
#define ISP_AFM_IIR_FILTER3           (0x02BCUL)
#define ISP_AFM_IIR_FILTER4           (0x02C0UL)
#define ISP_AFM_IIR_FILTER5           (0x02C4UL)
#define ISP_AFM_ENHANCE_FV0_THD         (0x03C8UL)
#define ISP_AFM_ENHANCE_FV1_THD           (0x03CCUL)
#define ISP_AFM_ENHANCE_FV1_COEFF00   (0x03D0UL)
#define ISP_AFM_ENHANCE_FV1_COEFF31   (0x03ECUL)

#define DCAM_RDS_COEFF_TABLE       (0x0400UL)
#define DCAM_LSC_WEIGHT_TABLE    (0x0800UL)
#define DCAM_PDAF_CORR_TABLE      (0x0E00UL)


#define DCAM_LSC_WEI_LAST0    (0x0C00UL)
#define DCAM_LSC_WEI_LAST1        (0x0C04UL)


/*  DCAM AXIM registers define*/
#define DCAM_AXIM_BASE                 s_dcam_aximbase

#define DCAM_AXIM_CTRL                   (0x0000UL)
#define DCAM_AXIM_DBG_STS            (0x0004UL)
#define DCAM_CAP_SENSOR_CTRL      (0x0008UL)
#define DCAM_MMU_CTRL                    (0x0010UL)
#define DCAM_SPARE_REG_0        (0x0014UL)
#define AXIM_SPARE_REG_0         (0x0018UL)
#define SPARE_REG_ICG                (0x001CUL)

#define IMG_FETCH_CTRL            (0x0020UL)
#define IMG_FETCH_SIZE            (0x0024UL)
#define IMG_FETCH_X                  (0x0028UL)
#define IMG_FETCH_RADDR        (0x002CUL)
#define IMG_FETCH_START          (0x0030UL)
#define DCAM_LBUF_SHARE_MODE (0x0040UL)

#define DCAM_MME_EN            (0x9000UL)
#define DCAM_MME_UPDATE   (0x9004UL)

 #endif



#define DCAM_REG_WR(idx, reg, val) (REG_WR(DCAM_BASE(idx)+reg, val))
#define DCAM_REG_RD(idx, reg) (REG_RD(DCAM_BASE(idx)+reg))
#define DCAM_REG_MWR(idx, reg, msk, val) DCAM_REG_WR(idx, reg, \
	((val) & (msk)) | (DCAM_REG_RD(idx, reg) & (~(msk))))

#define DCAM_AXIM_WR(reg, val) (REG_WR(DCAM_AXIM_BASE+reg, val))
#define DCAM_AXIM_RD(reg) (REG_RD(DCAM_AXIM_BASE+reg))
#define DCAM_AXIM_MWR(reg, msk, val) DCAM_AXIM_WR(reg, \
	((val) & (msk)) | (DCAM_AXIM_RD(reg) & (~(msk))))

#endif /* _DCAM_REG_H_ */

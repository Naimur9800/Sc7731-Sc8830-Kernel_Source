/*
 * Copyright (C) 2016 Spreadtrum Communications Inc.
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

#include <linux/uaccess.h>
#include <video/sprd_mm.h>
#include <video/sprd_isp_r6p10v2.h>
#include "../isp_block.h"
#include "../isp_drv.h"

static int isp_k_pdaf_correct_param(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	unsigned int i = 0;
	unsigned long dst_addr_right = 0;
	unsigned long dst_addr_left = 0;
	struct pdaf_correction_param pdaf;

	memset(&pdaf, 0x00, sizeof(pdaf));
	ret = copy_from_user((void *)&pdaf,
		param->property_param, sizeof(pdaf));
	if (ret != 0) {
		pr_err("isp_k_pdaf_correct_param: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	ISP_REG_MWR(idx, ISP_PDAF_PARAM, BIT_2, pdaf.ppi_corrector_bypass << 2);
	if (pdaf.ppi_corrector_bypass) {
		ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL1,
				BIT_0, 1);
		return 0;
	}

	val = (((pdaf.ppi_phase_map_corr_en & 0x1) << 3)
		| ((pdaf.ppi_grid & 0x1) << 8));
	ISP_REG_MWR(idx, ISP_PDAF_PARAM, 0x108, val);

	val = (pdaf.ppi_upperbound_gb & 0x3FF)
		| ((pdaf.ppi_upperbound_gr & 0x3FF) << 16);
	ISP_REG_WR(idx, ISP_PDAF_GAIN_PARA0, val);

	val = (pdaf.ppi_upperbound_b & 0x3FF)
		| ((pdaf.ppi_upperbound_r & 0x3FF) << 16);
	ISP_REG_WR(idx, ISP_PDAF_GAIN_PARA1, val);

	val = (pdaf.ppi_phase_txt_smoother & 0xF)
		| ((pdaf.ppi_phase_gfilter & 0x1) << 4)
		| ((pdaf.ppi_phase_flat_smoother & 0x7) << 5);
	ISP_REG_WR(idx, ISP_PDAF_PHASE_BPC_PARA, val);

	val = (pdaf.ppi_hot_1pixel_th & 0x3FF)
		| ((pdaf.ppi_hot_2pixel_th & 0x3FF) << 16);
	ISP_REG_WR(idx, ISP_PDAF_HOT_PIXEL_TH1, val);

	val = pdaf.ppi_hot_3pixel_th & 0x3FF;
	ISP_REG_WR(idx, ISP_PDAF_HOT_PIXEL_TH2, val);

	val = (pdaf.ppi_dead_1pixel_th & 0x3FF)
		| ((pdaf.ppi_dead_2pixel_th & 0x3FF) << 16);
	ISP_REG_WR(idx, ISP_PDAF_DEAD_PIXEL_TH1, val);

	val = pdaf.ppi_dead_3pixel_th & 0x3FF;
	ISP_REG_WR(idx, ISP_PDAF_DEAD_PIXEL_TH2, val);

	val = pdaf.ppi_flat_th & 0x3FF;
	ISP_REG_WR(idx, ISP_PDAF_FLAT_TH, val);

	val = (pdaf.ppi_edgeRatio_hv & 0x1FF)
		| ((pdaf.ppi_edgeRatio_rd & 0x1FF) << 10)
		| ((pdaf.ppi_edgeRatio_hv_rd & 0x1FF) << 20);
	ISP_REG_WR(idx, ISP_PDAF_EDGE_RATIO, val);


	val = (pdaf.l_gain[0] & 0x3FFF)
		| ((pdaf.r_gain[0] & 0x3FFF) << 16);
	ISP_REG_WR(idx, ISP_PDAF_GAIN_ORI0, val);

	val = (pdaf.l_gain[1] & 0x3FFF)
		| ((pdaf.r_gain[1] & 0x3FFF) << 16);
	ISP_REG_WR(idx, ISP_PDAF_GAIN_ORI1, val);

	val = (pdaf.ppi_blacklevel_r & 0x3FF)
		| ((pdaf.ppi_blacklevel_b & 0x3FF) << 10);
	ISP_REG_WR(idx, ISP_PDAF_BLC_PARA0, val);

	val = (pdaf.ppi_blacklevel_gr & 0x3FF)
		| ((pdaf.ppi_blacklevel_gb & 0x3FF) << 10);
	ISP_REG_WR(idx, ISP_PDAF_BLC_PARA1, val);

	dst_addr_left = ISP_BASE_ADDR(idx) + ISP_PDAF_GAIN_LEFT;
	dst_addr_right = ISP_BASE_ADDR(idx) + ISP_PDAF_GAIN_RIGHT;

	for (i = 0; i < PDAF_CORRECT_GAIN_NUM; i++) {
		*((unsigned int *)dst_addr_left + i) =
			((unsigned int)(pdaf.data_ptr_left[i])) & 0x3FFF;
		*((unsigned int *)dst_addr_right + i) =
			((unsigned int)(pdaf.data_ptr_right[i])) & 0x3FFF;
	}

	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL1, BIT_0, 1);

	return ret;
}

static int isp_k_pdaf_correct_bypass(struct isp_io_param *param,
					enum isp_id idx)
{
	int ret = 0;
	unsigned int bypass = 0;

	ret = copy_from_user((void *)&bypass,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_pdaf_correct_bypass: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (bypass)
		ISP_REG_OWR(idx, ISP_PDAF_PARAM, BIT_2);
	else
		ISP_REG_MWR(idx, ISP_PDAF_PARAM, BIT_2, 0);
	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL1, BIT_0, 1);

	return ret;
}

int isp_k_cfg_pdaf_correct(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_pdaf_correct: param is null error.\n");
		return -1;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_pdaf_correct: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_PDAF_SET_CORRECT_BYPASS:
		ret = isp_k_pdaf_correct_bypass(param, idx);
		break;
	case ISP_PRO_PDAF_SET_CORRECT_PARAM:
		ret = isp_k_pdaf_correct_param(param, idx);
		break;
	default:
		pr_err("isp_k_cfg_pdaf_correct: fail cmd id:%d, not supported.\n",
			param->property);
		break;
	}

	return ret;
}

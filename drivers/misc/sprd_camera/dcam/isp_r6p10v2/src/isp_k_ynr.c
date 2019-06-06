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
#include "../isp_drv.h"

static int isp_k_ynr_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dev_ynr_info ynr_info;
	unsigned int i;

	memset(&ynr_info, 0x00, sizeof(ynr_info));
	ret = copy_from_user((void *)&ynr_info,
		param->property_param, sizeof(ynr_info));
	if (ret != 0) {
		pr_err("isp_k_ynr_param: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_YNR_CONTRL0, BIT_0, ynr_info.bypass);
	ISP_REG_MWR(idx, ISP_YNR_CONTRL0 + ISP_CH1_ADDR_OFFSET, BIT_0,
		ynr_info.bypass);
	ISP_REG_MWR(idx, ISP_YNR_CONTRL0, BIT_1, ynr_info.lowlux_bypass << 1);
	ISP_REG_MWR(idx, ISP_YNR_CONTRL0 + ISP_CH1_ADDR_OFFSET, BIT_1,
		ynr_info.lowlux_bypass << 1);

	if (ynr_info.bypass)
		return 0;

	ISP_REG_MWR(idx, ISP_YNR_CONTRL0, BIT_2, ynr_info.nr_enable   << 2);
	ISP_REG_MWR(idx, ISP_YNR_CONTRL0, BIT_3, ynr_info.l_blf_en[2] << 3);
	ISP_REG_MWR(idx, ISP_YNR_CONTRL0, BIT_4, ynr_info.l_blf_en[1] << 4);
	ISP_REG_MWR(idx, ISP_YNR_CONTRL0, BIT_5, ynr_info.l_blf_en[0] << 5);
	ISP_REG_MWR(idx, ISP_YNR_CONTRL0 + ISP_CH1_ADDR_OFFSET, BIT_2,
		ynr_info.nr_enable   << 2);
	ISP_REG_MWR(idx, ISP_YNR_CONTRL0 + ISP_CH1_ADDR_OFFSET, BIT_3,
		ynr_info.l_blf_en[2] << 3);
	ISP_REG_MWR(idx, ISP_YNR_CONTRL0 + ISP_CH1_ADDR_OFFSET, BIT_4,
		ynr_info.l_blf_en[1] << 4);
	ISP_REG_MWR(idx, ISP_YNR_CONTRL0 + ISP_CH1_ADDR_OFFSET, BIT_5,
		ynr_info.l_blf_en[0] << 5);

	val = ((ynr_info.edge_th    & 0xFF) << 24) |
		  ((ynr_info.txt_th     & 0xFF) << 16) |
		  ((ynr_info.flat_th[0] & 0xFF) << 8) |
		   (ynr_info.flat_th[1] & 0xFF);
	ISP_REG_WR(idx, ISP_YNR_CFG0, val);
	ISP_REG_WR(idx, ISP_YNR_CFG0 + ISP_CH1_ADDR_OFFSET, val);

	val = ((ynr_info.flat_th[2] & 0xFF) << 24) |
		  ((ynr_info.flat_th[3] & 0xFF) << 16) |
		  ((ynr_info.flat_th[4] & 0xFF) << 8) |
		   (ynr_info.flat_th[5] & 0xFF);
	ISP_REG_WR(idx, ISP_YNR_CFG1, val);
	ISP_REG_WR(idx, ISP_YNR_CFG1 + ISP_CH1_ADDR_OFFSET, val);

	val = ((ynr_info.flat_th[6] & 0xFF) << 24) |
		  ((ynr_info.lut_th[0]  & 0xFF) << 16) |
		  ((ynr_info.lut_th[1]  & 0xFF) << 8) |
		   (ynr_info.lut_th[2]  & 0xFF);
	ISP_REG_WR(idx, ISP_YNR_CFG2, val);
	ISP_REG_WR(idx, ISP_YNR_CFG2 + ISP_CH1_ADDR_OFFSET, val);

	val = ((ynr_info.lut_th[3] & 0xFF) << 24) |
		  ((ynr_info.lut_th[4] & 0xFF) << 16) |
		  ((ynr_info.lut_th[5] & 0xFF) << 8) |
		   (ynr_info.lut_th[6] & 0xFF);
	ISP_REG_WR(idx, ISP_YNR_CFG3, val);
	ISP_REG_WR(idx, ISP_YNR_CFG3 + ISP_CH1_ADDR_OFFSET, val);

	for (i = 0; i < 2; i++) {
		val = ((ynr_info.addback[i*4] & 0xFF) << 24) |
			  ((ynr_info.addback[i*4+1] & 0xFF) << 16) |
			  ((ynr_info.addback[i*4+2] & 0xFF) << 8) |
			   (ynr_info.addback[i*4+3] & 0xFF);
		ISP_REG_WR(idx, ISP_YNR_CFG4 + 4*i, val);
		ISP_REG_WR(idx, ISP_YNR_CFG4 + 4*i + ISP_CH1_ADDR_OFFSET, val);
	}

	val = ((ynr_info.addback[8] & 0xFF) << 24) |
		  ((ynr_info.sub_th[0] & 0x7F) << 16) |
		  ((ynr_info.sub_th[1] & 0x7F) << 8) |
		   (ynr_info.sub_th[2] & 0x7F);
	ISP_REG_WR(idx, ISP_YNR_CFG6, val);
	ISP_REG_WR(idx, ISP_YNR_CFG6 + ISP_CH1_ADDR_OFFSET, val);

	val = ((ynr_info.sub_th[3] & 0x7F) << 24) |
		  ((ynr_info.sub_th[4] & 0x7F) << 16) |
		  ((ynr_info.sub_th[5] & 0x7F) << 8) |
		   (ynr_info.sub_th[6] & 0x7F);
	ISP_REG_WR(idx, ISP_YNR_CFG7, val);
	ISP_REG_WR(idx, ISP_YNR_CFG7 + ISP_CH1_ADDR_OFFSET, val);

	val = ((ynr_info.sub_th[7] & 0x7F) << 24) |
		  ((ynr_info.sub_th[8] & 0x7F) << 16) |
		  ((ynr_info.l_euroweight[0][0] & 0xF) << 8) |
		  ((ynr_info.l_euroweight[0][1] & 0xF) << 4) |
		   (ynr_info.l_euroweight[0][2] & 0xF);
	ISP_REG_WR(idx, ISP_YNR_CFG8, val);
	ISP_REG_WR(idx, ISP_YNR_CFG8 + ISP_CH1_ADDR_OFFSET, val);

	val = ((ynr_info.l_euroweight[1][0] & 0xF) << 20) |
		  ((ynr_info.l_euroweight[1][1] & 0xF) << 16) |
		  ((ynr_info.l_euroweight[1][2] & 0xF) << 12) |
		  ((ynr_info.l_euroweight[2][0] & 0xF) << 8) |
		  ((ynr_info.l_euroweight[2][1] & 0xF) << 4) |
		   (ynr_info.l_euroweight[2][2] & 0xF);
	ISP_REG_WR(idx, ISP_YNR_CFG9, val);
	ISP_REG_WR(idx, ISP_YNR_CFG9 + ISP_CH1_ADDR_OFFSET, val);

	val = ((ynr_info.l_wf_index[0] & 0xF) << 24) |
		  ((ynr_info.l_wf_index[1] & 0xF) << 20) |
		  ((ynr_info.l_wf_index[2] & 0xF) << 16) |
		  ((ynr_info.l1_txt_th0 & 0x7) << 12) |
		  ((ynr_info.l1_txt_th1 & 0x7) << 8) |
		  ((ynr_info.l0_lut_th0 & 0x7) << 4) |
		   (ynr_info.l0_lut_th1  & 0x7);
	ISP_REG_WR(idx, ISP_YNR_CFG10, val);
	ISP_REG_WR(idx, ISP_YNR_CFG10 + ISP_CH1_ADDR_OFFSET, val);

	for (i = 0; i < 6; i++) {
		val = ((ynr_info.wlt_th[i*4] & 0x7F) << 24) |
			  ((ynr_info.wlt_th[i*4+1] & 0x7F) << 16) |
			  ((ynr_info.wlt_th[i*4+2] & 0x7F) << 8) |
			   (ynr_info.wlt_th[i*4+3] & 0x7F);
		ISP_REG_WR(idx, ISP_YNR_WLT0 + 4 * i, val);
		ISP_REG_WR(idx,
			ISP_YNR_WLT0 + 4 * i + ISP_CH1_ADDR_OFFSET, val);
	}

	for (i = 0; i < 8; i++) {
		val = ((ynr_info.freq_ratio[i*3+0] & 0x3FF) << 20) |
			  ((ynr_info.freq_ratio[i*3+1] & 0x3FF) << 10) |
			   (ynr_info.freq_ratio[i*3+2] & 0x3FF);
		ISP_REG_WR(idx, ISP_YNR_FRATIO0 +	4 * i, val);
		ISP_REG_WR(idx,
			ISP_YNR_FRATIO0 +	4 * i +
			ISP_CH1_ADDR_OFFSET, val);
	}

	/*for slice config*/
	ISP_REG_WR(idx, ISP_YNR_CFG11, 0);
	ISP_REG_WR(idx, ISP_YNR_CFG11 + ISP_CH1_ADDR_OFFSET, 0);

	val = ((ynr_info.center.x & 0xFFFF) << 16) |
		   (ynr_info.center.y & 0xFFFF);
	ISP_REG_WR(idx, ISP_YNR_CFG12, val);
	ISP_REG_WR(idx, ISP_YNR_CFG12 + ISP_CH1_ADDR_OFFSET, val);

	val = ((ynr_info.radius & 0xFFFF) << 16) |
		   (ynr_info.dist_interval & 0xFFFF);
	ISP_REG_WR(idx, ISP_YNR_CFG13, val);
	ISP_REG_WR(idx, ISP_YNR_CFG13 + ISP_CH1_ADDR_OFFSET, val);

	for (i = 0; i < 2; i++) {
		val = ((ynr_info.sal_nr_str[i*4+0] & 0x7F) << 24) |
			  ((ynr_info.sal_nr_str[i*4+1] & 0x7F) << 16) |
			  ((ynr_info.sal_nr_str[i*4+2] & 0x7F) << 8) |
			   (ynr_info.sal_nr_str[i*4+3] & 0x7F);
		ISP_REG_WR(idx, ISP_YNR_CFG14 + 4*i, val);
		ISP_REG_WR(idx, ISP_YNR_CFG14 + 4*i + ISP_CH1_ADDR_OFFSET, val);
	}

	for (i = 0; i < 2; i++) {
		val = ((ynr_info.sal_offset[i*4+0] & 0xFF) << 24) |
			  ((ynr_info.sal_offset[i*4+1] & 0xFF) << 16) |
			  ((ynr_info.sal_offset[i*4+2] & 0xFF) << 8) |
			   (ynr_info.sal_offset[i*4+3] & 0xFF);
		ISP_REG_WR(idx, ISP_YNR_CFG16 + 4*i, val);
		ISP_REG_WR(idx, ISP_YNR_CFG16 + 4*i + ISP_CH1_ADDR_OFFSET, val);
	}

	return ret;
}

static int isp_k_ynr_bypass(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int bypass = 0;

	ret = copy_from_user((void *)&bypass,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_ynr_bypass: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (bypass)
		ISP_REG_OWR(idx, ISP_YNR_CONTRL0, BIT_0);
	else
		ISP_REG_MWR(idx, ISP_YNR_CONTRL0, BIT_0, 0);

	return ret;
}

static int isp_k_ynr_param(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct ynr_param ynr;
	unsigned int i;

	ret = copy_from_user((void *)&ynr,
		param->property_param, sizeof(ynr));
	if (ret != 0) {
		pr_err("isp_k_ynr_param: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	val = ((ynr.yDenoise_lowlux_bypass & 0x1) << 1)
		| ((ynr.yDenoise_wv_nr_enable & 0x1) << 2)
		| ((ynr.yDenoise_l3_blf_enable & 0x1) << 3)
		| ((ynr.yDenoise_l2_blf_enable & 0x1) << 4)
		| ((ynr.yDenoise_l1_blf_enable & 0x1) << 5);
	ISP_REG_MWR(idx, ISP_YNR_CONTRL0, 0x3E, val);

	val = (ynr.yDenoise_flat[1] & 0xFF)
		| ((ynr.yDenoise_flat[0] & 0xFF) << 8)
		| ((ynr.yDenoise_txtThresh & 0xFF) << 16)
		| ((ynr.yDenoise_SEdgethresh & 0xFF) << 24);
	ISP_REG_WR(idx, ISP_YNR_CFG0, val);

	val = (ynr.yDenoise_flat[5] & 0xFF)
		| ((ynr.yDenoise_flat[4] & 0xFF) << 8)
		| ((ynr.yDenoise_flat[3] & 0xFF) << 16)
		| ((ynr.yDenoise_flat[2] & 0xFF) << 24);
	ISP_REG_WR(idx, ISP_YNR_CFG1, val);

	val = (ynr.yDenoise_lut_thresh[2] & 0xFF)
		| ((ynr.yDenoise_lut_thresh[1] & 0xFF) << 8)
		| ((ynr.yDenoise_lut_thresh[0] & 0xFF) << 16)
		| ((ynr.yDenoise_flat[6] & 0xFF) << 24);
	ISP_REG_WR(idx, ISP_YNR_CFG2, val);

	val = (ynr.yDenoise_lut_thresh[6] & 0xFF)
		| ((ynr.yDenoise_lut_thresh[5] & 0xFF) << 8)
		| ((ynr.yDenoise_lut_thresh[4] & 0xFF) << 16)
		| ((ynr.yDenoise_lut_thresh[3] & 0xFF) << 24);
	ISP_REG_WR(idx, ISP_YNR_CFG3, val);

	val = (ynr.yDenoise_addback[3] & 0xFF)
		| ((ynr.yDenoise_addback[2] & 0xFF) << 8)
		| ((ynr.yDenoise_addback[1] & 0xFF) << 16)
		| ((ynr.yDenoise_addback[0] & 0xFF) << 24);
	ISP_REG_WR(idx, ISP_YNR_CFG4, val);

	val = (ynr.yDenoise_addback[7] & 0xFF)
		| ((ynr.yDenoise_addback[6] & 0xFF) << 8)
		| ((ynr.yDenoise_addback[5] & 0xFF) << 16)
		| ((ynr.yDenoise_addback[4] & 0xFF) << 24);
	ISP_REG_WR(idx, ISP_YNR_CFG5, val);

	val = (ynr.yDenoise_subThresh[2] & 0x7F)
		| ((ynr.yDenoise_subThresh[1] & 0x7F) << 8)
		| ((ynr.yDenoise_subThresh[0] & 0x7F) << 16)
		| ((ynr.yDenoise_addback[8] & 0xFF) << 24);
	ISP_REG_WR(idx, ISP_YNR_CFG6, val);

	val = (ynr.yDenoise_subThresh[6] & 0x7F)
		| ((ynr.yDenoise_subThresh[5] & 0x7F) << 8)
		| ((ynr.yDenoise_subThresh[4] & 0x7F) << 16)
		| ((ynr.yDenoise_subThresh[3] & 0x7F) << 24);
	ISP_REG_WR(idx, ISP_YNR_CFG7, val);

	val = (ynr.yDenoise_l1_euroDist[2] & 0xF)
		| ((ynr.yDenoise_l1_euroDist[1] & 0xF) << 4)
		| ((ynr.yDenoise_l1_euroDist[0] & 0xF) << 8)
		| ((ynr.yDenoise_subThresh[8] & 0x7F) << 16)
		| ((ynr.yDenoise_subThresh[7] & 0x7F) << 24);
	ISP_REG_WR(idx, ISP_YNR_CFG8, val);

	val = (ynr.yDenoise_l3_euroDist[2] & 0xF)
		| ((ynr.yDenoise_l3_euroDist[1] & 0xF) << 4)
		| ((ynr.yDenoise_l3_euroDist[0] & 0xF) << 8)
		| ((ynr.yDenoise_l2_euroDist[2] & 0xF) << 12)
		| ((ynr.yDenoise_l2_euroDist[1] & 0xF) << 16)
		| ((ynr.yDenoise_l2_euroDist[0] & 0xF) << 20);
	ISP_REG_WR(idx, ISP_YNR_CFG9, val);

	val = (ynr.yDenoise_l0_lut_thresh1 & 0x7)
		| ((ynr.yDenoise_l0_lut_thresh0 & 0x7) << 4)
		| ((ynr.yDenoise_l1_txt_thresh1 & 0x7) << 8)
		| ((ynr.yDenoise_l1_txt_thresh0 & 0x7) << 12)
		| ((ynr.yDenoise_l3_WFIndex & 0xF) << 16)
		| ((ynr.yDenoise_l2_WFIndex & 0xF) << 20)
		| ((ynr.yDenoise_l1_WFIndex & 0xF) << 24);
	ISP_REG_WR(idx, ISP_YNR_CFG10, val);

	for (i = 0; i < 6; i++) {
		val = (ynr.wltT[i*4+3] & 0x7F)
			| ((ynr.wltT[i*4+2] & 0x7F) << 8)
			| ((ynr.wltT[i*4+1] & 0x7F) << 16)
			| ((ynr.wltT[i*4] & 0x7F) << 24);
		ISP_REG_WR(idx, ISP_YNR_WLT0 + i * 4, val);
	}

	for (i = 0; i < 8; i++) {
		val = (ynr.freqRatio[i*3+2] & 0x3FF)
			| ((ynr.freqRatio[i*3+1] & 0x3FF) << 10)
			| ((ynr.freqRatio[i*3] & 0x3FF) << 20);
		ISP_REG_WR(idx, ISP_YNR_FRATIO0 + i * 4, val);
	}

	val = (ynr.yDenoise_imgCenterY & 0xFFFF)
		| ((ynr.yDenoise_imgCenterX & 0xFFFF) << 16);
	ISP_REG_WR(idx, ISP_YNR_CFG12, val);

	val = (ynr.dist_interval & 0xFFFF)
		| ((ynr.yDenoise_Radius & 0xFFFF) << 16);
	ISP_REG_WR(idx, ISP_YNR_CFG13, val);

	val = (ynr.yDenoise_Sal_nr_str[3] & 0x7F)
		| ((ynr.yDenoise_Sal_nr_str[2] & 0x7F) << 8)
		| ((ynr.yDenoise_Sal_nr_str[1] & 0x7F) << 16)
		| ((ynr.yDenoise_Sal_nr_str[0] & 0x7F) << 24);
	ISP_REG_WR(idx, ISP_YNR_CFG14, val);

	val = (ynr.yDenoise_Sal_nr_str[7] & 0x7F)
		| ((ynr.yDenoise_Sal_nr_str[6] & 0x7F) << 8)
		| ((ynr.yDenoise_Sal_nr_str[5] & 0x7F) << 16)
		| ((ynr.yDenoise_Sal_nr_str[4] & 0x7F) << 24);
	ISP_REG_WR(idx, ISP_YNR_CFG15, val);

	val = (ynr.yDenoise_Sal_offset[3] & 0xFF)
		| ((ynr.yDenoise_Sal_offset[2] & 0xFF) << 8)
		| ((ynr.yDenoise_Sal_offset[1] & 0xFF) << 16)
		| ((ynr.yDenoise_Sal_offset[0] & 0xFF) << 24);
	ISP_REG_WR(idx, ISP_YNR_CFG16, val);

	val = (ynr.yDenoise_Sal_offset[7] & 0xFF)
		| ((ynr.yDenoise_Sal_offset[6] & 0xFF) << 8)
		| ((ynr.yDenoise_Sal_offset[5] & 0xFF) << 16)
		| ((ynr.yDenoise_Sal_offset[4] & 0xFF) << 24);
	ISP_REG_WR(idx, ISP_YNR_CFG17, val);

	return ret;
}

static int isp_k_ynr_start_pos(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct img_offset img;

	ret = copy_from_user((void *)&img,
		param->property_param, sizeof(img));
	if (ret != 0) {
		pr_err("isp_k_ynr_start_pos: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	val = (img.y & 0xFFFF) | ((img.x & 0xFFFF) << 16);
	ISP_REG_WR(idx, ISP_YNR_CFG11, val);

	return ret;
}

int isp_k_cfg_ynr(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_ynr: param is null error.\n");
		return -1;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_ynr: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_YNR_BLOCK:
	    ret = isp_k_ynr_block(param, idx);
	    break;
	case ISP_PRO_YNR_BYPASS:
		ret = isp_k_ynr_bypass(param, idx);
		break;
	case ISP_PRO_YNR_PARAM:
		ret = isp_k_ynr_param(param, idx);
		break;
	case ISP_PRO_YNR_START_POS:
		ret = isp_k_ynr_start_pos(param, idx);
		break;
	default:
		pr_err("isp_k_cfg_ynr: fail cmd id:%d, not supported.\n",
			param->property);
		break;
	}

	return ret;
}

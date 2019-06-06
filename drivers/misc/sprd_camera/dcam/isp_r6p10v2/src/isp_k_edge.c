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

static int isp_k_edge_bypass(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int bypass = 0;

	ret = copy_from_user((void *)&bypass,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_edge_bypass: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (bypass)
		ISP_REG_OWR(idx, ISP_EE_PARAM, BIT_0);
	else
		ISP_REG_MWR(idx, ISP_EE_PARAM, BIT_0, 0);

	return ret;
}

static int isp_k_edge_param(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct ee_param ee_param;

	memset(&ee_param, 0x00, sizeof(ee_param));
	ret = copy_from_user((void *)&ee_param,
		param->property_param, sizeof(struct ee_param));
	if (ret != 0) {
		pr_err("isp_k_edge_param: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	val = (ee_param.ee_flat_smooth_mode & 0x3)
		| ((ee_param.ee_edge_smooth_mode & 0x3) << 2)
		| ((ee_param.ee_str_d_n & 0x7F) << 14)
		| ((ee_param.ee_str_d_p & 0x7F) << 21)
		| ((ee_param.ee_mode & 0x1) << 28);
	ISP_REG_WR(idx, ISP_EE_CFG0, val);

	val = (ee_param.ee_incr_d_n & 0xFF)
		| ((ee_param.ee_incr_d_p & 0xFF) << 8)
		| ((ee_param.ee_edge_thr_d_n & 0xFF) << 16)
		| ((ee_param.ee_edge_thr_d_p & 0xFF) << 24);
	ISP_REG_WR(idx, ISP_EE_CFG1, val);

	val = (ee_param.ee_corner_sm_n & 0x3)
		|((ee_param.ee_corner_sm_p & 0x3) << 2)
		| ((ee_param.ee_corner_gain_n & 0x7F) << 4)
		| ((ee_param.ee_corner_gain_p & 0x7F) << 11)
		| ((ee_param.ee_corner_th_n & 0x1F) << 18)
		| ((ee_param.ee_corner_th_p & 0x1F) << 23)
		| ((ee_param.ee_corner_cor & 0x1) << 28);
	ISP_REG_WR(idx, ISP_EE_CFG2, val);

	val = (ee_param.ee_cv_t1 & 0x3FF)
		| ((ee_param.ee_cv_t2 & 0x3FF) << 10)
		| ((ee_param.ee_cv_t3 & 0x3FF) << 20);
	ISP_REG_WR(idx, ISP_EE_ADP_CFG0, val);

	val = (ee_param.ee_cv_t4 & 0x3FF)
		| ((ee_param.ee_cv_clip_n & 0xFF) << 16)
		| ((ee_param.ee_cv_clip_p & 0xFF) << 24);
	ISP_REG_WR(idx, ISP_EE_ADP_CFG1, val);

	val = (ee_param.ee_cv_r1 & 0xFF)
		| ((ee_param.ee_cv_r2 & 0xFF) << 8)
		| ((ee_param.ee_cv_r3 & 0xFF) << 16);
	ISP_REG_WR(idx, ISP_EE_ADP_CFG2, val);

	val = (ee_param.ipd_bypass & 0x1)
		| ((ee_param.ipd_mask_mode & 0x1) << 1)
		| ((ee_param.ipd_less_thr_p & 0xF) << 2)
		| ((ee_param.ipd_less_thr_n & 0xF) << 6)
		| ((ee_param.ipd_smooth_en & 0x1) << 10)
		| ((ee_param.ipd_smooth_mode_p & 0x7) << 11)
		| ((ee_param.ipd_smooth_mode_n & 0x7) << 14);
	ISP_REG_WR(idx, ISP_EE_IPD_CFG0, val);

	val = (ee_param.ipd_flat_thr_p & 0xFF)
		| ((ee_param.ipd_flat_thr_n & 0xFF) << 8)
		| ((ee_param.ipd_eq_thr_p & 0xF) << 16)
		| ((ee_param.ipd_eq_thr_n & 0xF) << 20)
		| ((ee_param.ipd_more_thr_p & 0xF) << 24)
		| ((ee_param.ipd_more_thr_n & 0xF) << 28);
	ISP_REG_WR(idx, ISP_EE_IPD_CFG1, val);

	val = (ee_param.ipd_smooth_edge_thr_p & 0xFF)
		| ((ee_param.ipd_smooth_edge_thr_n & 0xFF) << 8)
		| ((ee_param.ipd_smooth_edge_diff_p & 0xFF) << 16)
		| ((ee_param.ipd_smooth_edge_diff_n & 0xFF) << 24);
	ISP_REG_WR(idx, ISP_EE_IPD_CFG2, val);

	val = (ee_param.ee_ratio_hv_3 & 0x7F)
		| ((ee_param.ee_ratio_hv_5 & 0x7F) << 7)
		| ((ee_param.ee_ratio_diag_3 & 0x7F) << 14)
		| ((ee_param.ee_weightt_hv2diag & 0x1F) << 21)
		| ((ee_param.ee_gradient_computation_type & 0x1) << 26)
			| ((ee_param.ee_weightt_diag2hv & 0x1F) << 27);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG0, val);

	val = (ee_param.ee_gain_hv_1_t1 & 0x3FF)
		| ((ee_param.ee_gain_hv_1_t2 & 0x3FF) << 10)
			| ((ee_param.ee_gain_hv_1_t3 & 0x3FF) << 20);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG1, val);

	val = (ee_param.ee_gain_hv_1_t4 & 0x3FF)
		| ((ee_param.ee_gain_hv_1_r1 & 0x1F) << 10)
		| ((ee_param.ee_gain_hv_1_r2 & 0x1F) << 15)
		| ((ee_param.ee_gain_hv_1_r3 & 0x1F) << 20)
		| ((ee_param.ee_ratio_diag_5 & 0x7F) << 25);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG2, val);

	val = (ee_param.ee_gain_hv_2_t1 & 0x3FF)
		| ((ee_param.ee_gain_hv_2_t2 & 0x3FF) << 10)
		| ((ee_param.ee_gain_hv_2_t3 & 0x3FF) << 20);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG3, val);

	val = (ee_param.ee_gain_hv_2_t4 & 0x3FF)
		| ((ee_param.ee_gain_hv_2_r1 & 0x1F) << 10)
		| ((ee_param.ee_gain_hv_2_r2 & 0x1F) << 15)
		| ((ee_param.ee_gain_hv_2_r3 & 0x1F) << 20);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG4, val);

	val = (ee_param.ee_gain_diag_1_t1 & 0x3FF)
		| ((ee_param.ee_gain_diag_1_t2 & 0x3FF) << 10)
		| ((ee_param.ee_gain_diag_1_t3 & 0x3FF) << 20);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG5, val);

	val = (ee_param.ee_gain_diag_1_t4 & 0x3FF)
		| ((ee_param.ee_gain_diag_1_r1 & 0x1F) << 10)
		| ((ee_param.ee_gain_diag_1_r2 & 0x1F) << 15)
		| ((ee_param.ee_gain_diag_1_r3 & 0x1F) << 20);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG6, val);

	val = (ee_param.ee_gain_diag_2_t1 & 0x3FF)
		| ((ee_param.ee_gain_diag_2_t2 & 0x3FF) << 10)
		| ((ee_param.ee_gain_diag_2_t3 & 0x3FF) << 20);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG7, val);

	val = (ee_param.ee_gain_diag_2_t4 & 0x3FF)
		| ((ee_param.ee_gain_diag_2_r1 & 0x1F) << 10)
		| ((ee_param.ee_gain_diag_2_r2 & 0x1F) << 15)
		| ((ee_param.ee_gain_diag_2_r3 & 0x1F) << 20);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG8, val);

	val = (ee_param.ee_lum_t1 & 0xFF)
		| ((ee_param.ee_lum_t2 & 0xFF) << 8)
		| ((ee_param.ee_lum_t3 & 0xFF) << 16)
		| ((ee_param.ee_lum_t4 & 0xFF) << 24);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG9, val);

	val = (ee_param.ee_lum_r1 & 0x7F)
		| ((ee_param.ee_lum_r2 & 0x7F) << 7)
		| ((ee_param.ee_lum_r3 & 0x7F) << 14);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG10, val);

	val = (ee_param.ee_pos_t1 & 0x3FF)
		| ((ee_param.ee_pos_t2 & 0x3FF) << 10)
		| ((ee_param.ee_pos_t3 & 0x3FF) << 20);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG11, val);

	val = (ee_param.ee_pos_t4 & 0x3FF)
		| ((ee_param.ee_pos_r1 & 0x7F) << 10)
		| ((ee_param.ee_pos_r2 & 0x7F) << 17)
		| ((ee_param.ee_pos_r3 & 0x7F) << 24);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG12, val);

	val = (ee_param.ee_pos_c1 & 0x7F)
		| ((ee_param.ee_pos_c2 & 0x7F) << 7)
		| ((ee_param.ee_pos_c3 & 0x7F) << 14);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG13, val);

	val = (ee_param.ee_neg_t1 & 0x3FF)
		| ((ee_param.ee_neg_t2 & 0x3FF) << 10)
		| ((ee_param.ee_neg_t3 & 0x3FF) << 20);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG14, val);

	val = (ee_param.ee_neg_t4 & 0x3FF)
		| ((ee_param.ee_neg_r1 & 0x7F) << 10)
		| ((ee_param.ee_neg_r2 & 0x7F) << 17)
		| ((ee_param.ee_neg_r3 & 0x7F) << 24);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG15, val);

	val = (ee_param.ee_neg_c1 & 0xFF)
		| ((ee_param.ee_neg_c2 & 0xFF) << 8)
		| ((ee_param.ee_neg_c3 & 0xFF) << 16);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG16, val);
	val = (ee_param.ee_freq_t1 & 0x3FF)
		| ((ee_param.ee_freq_t2 & 0x3FF) << 10)
		| ((ee_param.ee_freq_t3 & 0x3FF) << 20);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG17, val);

	val = (ee_param.ee_freq_r1 & 0x3F)
		| ((ee_param.ee_freq_r2 & 0x3F) << 6)
		| ((ee_param.ee_freq_r3 & 0x3F) << 12)
		| ((ee_param.ee_freq_t4 & 0x3FF) << 18);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG18, val);

	return ret;
}

static int isp_k_edge_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	struct isp_dev_edge_info edge_info;
	unsigned int val = 0;
	unsigned int i   = 0;

	memset(&edge_info, 0x00, sizeof(edge_info));
	ret = copy_from_user((void *)&edge_info,
		param->property_param, sizeof(edge_info));
	if (ret != 0) {
		pr_err("isp_k_edge_block: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	ISP_REG_MWR(idx, ISP_EE_PARAM, BIT_0, edge_info.bypass);
	ISP_REG_MWR(idx, ISP_EE_PARAM + ISP_CH1_ADDR_OFFSET, BIT_0,
		edge_info.bypass);
	if (edge_info.bypass)
		return 0;

	val	= ((edge_info.mode			   & 0x1)  << 28) |
		  ((edge_info.ee_str_d.p	   & 0x7F) << 21) |
		  ((edge_info.ee_str_d.n	   & 0x7F) << 14) |
		  ((edge_info.edge_smooth_mode & 0x3)  << 2) |
		   (edge_info.flat_smooth_mode & 0x3);
	ISP_REG_WR(idx, ISP_EE_CFG0, val);
	ISP_REG_WR(idx, ISP_EE_CFG0 + ISP_CH1_ADDR_OFFSET, val);

	val	= ((edge_info.ee_edge_thr_d.p & 0xFF) << 24) |
		  ((edge_info.ee_edge_thr_d.n & 0xFF) << 16) |
		  ((edge_info.ee_incr_d.p	  & 0xFF) << 8) |
		   (edge_info.ee_incr_d.n	  & 0xFF);
	ISP_REG_WR(idx, ISP_EE_CFG1, val);
	ISP_REG_WR(idx, ISP_EE_CFG1 + ISP_CH1_ADDR_OFFSET, val);

	val	= ((edge_info.ee_corner_cor	   & 0x1)  << 28) |
		  ((edge_info.ee_corner_th.p   & 0x1F) << 23) |
		  ((edge_info.ee_corner_th.n   & 0x1F) << 18) |
		  ((edge_info.ee_corner_gain.p & 0x3F) << 11) |
		  ((edge_info.ee_corner_gain.n & 0x3F) << 4) |
		  ((edge_info.ee_corner_sm.p   & 0x3)  << 2) |
		   (edge_info.ee_corner_sm.n   & 0x3);
	ISP_REG_WR(idx, ISP_EE_CFG2, val);
	ISP_REG_WR(idx, ISP_EE_CFG2 + ISP_CH1_ADDR_OFFSET, val);

	val	= ((edge_info.ee_cv_t[2] & 0x3FF) << 20) |
		  ((edge_info.ee_cv_t[1] & 0x3FF) << 10) |
		   (edge_info.ee_cv_t[0] & 0x3FF);
	ISP_REG_WR(idx, ISP_EE_ADP_CFG0, val);
	ISP_REG_WR(idx, ISP_EE_ADP_CFG0 + ISP_CH1_ADDR_OFFSET, val);

	val	= ((edge_info.ee_cv_clip.p & 0xFF) << 24) |
		  ((edge_info.ee_cv_clip.n & 0xFF) << 16) |
		   (edge_info.ee_cv_t[3] & 0x3FF);
	ISP_REG_WR(idx, ISP_EE_ADP_CFG1, val);
	ISP_REG_WR(idx, ISP_EE_ADP_CFG1 + ISP_CH1_ADDR_OFFSET, val);

	val	= ((edge_info.ee_cv_r[2] & 0xFF) << 16) |
		  ((edge_info.ee_cv_r[1] & 0xFF) << 8) |
		   (edge_info.ee_cv_r[0] & 0xFF);
	ISP_REG_WR(idx, ISP_EE_ADP_CFG2, val);
	ISP_REG_WR(idx, ISP_EE_ADP_CFG2 + ISP_CH1_ADDR_OFFSET, val);

	val	= ((edge_info.ipd_smooth_mode.n	& 0x7) << 14) |
		  ((edge_info.ipd_smooth_mode.p	& 0x7) << 11) |
		  ((edge_info.ipd_smooth_en		& 0x1) << 10) |
		  ((edge_info.ipd_less_thr.n	& 0xF) << 6) |
		  ((edge_info.ipd_less_thr.p	& 0xF) << 2) |
		  ((edge_info.ipd_mask_mode		& 0x1) << 1) |
		   (edge_info.ipd_bypass		& 0x1);
	ISP_REG_WR(idx, ISP_EE_IPD_CFG0, val);
	ISP_REG_WR(idx, ISP_EE_IPD_CFG0 + ISP_CH1_ADDR_OFFSET, val);

	val	= ((edge_info.ipd_more_thr.n   & 0xF)  << 28) |
		  ((edge_info.ipd_more_thr.p   & 0xF)  << 24) |
		  ((edge_info.ipd_eq_thr.n	   & 0xF)  << 20) |
		  ((edge_info.ipd_eq_thr.p	   & 0xF)  << 16) |
		  ((edge_info.ipd_flat_thr.n   & 0xFF) << 8) |
		   (edge_info.ipd_flat_thr.p   & 0xFF);
	ISP_REG_WR(idx, ISP_EE_IPD_CFG1, val);
	ISP_REG_WR(idx, ISP_EE_IPD_CFG1 + ISP_CH1_ADDR_OFFSET, val);

	val	= ((edge_info.ipd_smooth_edge_diff.n  & 0xFF) << 24) |
		  ((edge_info.ipd_smooth_edge_diff.p  & 0xFF) << 16) |
		  ((edge_info.ipd_smooth_edge_thr.n	  & 0xFF) << 8) |
		   (edge_info.ipd_smooth_edge_thr.p	  & 0xFF);
	ISP_REG_WR(idx, ISP_EE_IPD_CFG2, val);
	ISP_REG_WR(idx, ISP_EE_IPD_CFG2 + ISP_CH1_ADDR_OFFSET, val);

	val	= ((edge_info.ee_weight_diag2hv	& 0x1F)	<< 27) |
		  ((edge_info.ee_gradient_computation_type & 0x1) << 26) |
		  ((edge_info.ee_weight_hv2diag	& 0x1F)	<< 21) |
		  ((edge_info.ee_ratio_diag_3 & 0x7F)	<< 14) |
		  ((edge_info.ee_ratio_hv_5 & 0x7F)	<< 7) |
		   (edge_info.ee_ratio_hv_3 & 0x7F);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG0, val);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG0 + ISP_CH1_ADDR_OFFSET, val);

	for (i = 0; i < 2; i++) {
		val = ((edge_info.ee_gain_hv_t[i][2]  & 0x3FF) << 20) |
			  ((edge_info.ee_gain_hv_t[i][1]  & 0x3FF) << 10) |
			   (edge_info.ee_gain_hv_t[i][0]  & 0x3FF);
		ISP_REG_WR(idx, ISP_EE_LUM_CFG1 + 8*i, val);
		ISP_REG_WR(idx,
			ISP_EE_LUM_CFG1 + ISP_CH1_ADDR_OFFSET + 8*i, val);
	}

	val = ((edge_info.ee_ratio_diag_5	  & 0x7F)  << 25) |
		  ((edge_info.ee_gain_hv_r[0][2]  & 0x1F)  << 20) |
		  ((edge_info.ee_gain_hv_r[0][1]  & 0x1F)  << 15) |
		  ((edge_info.ee_gain_hv_r[0][0]  & 0x1F) << 10) |
		   (edge_info.ee_gain_hv_t[0][3]  & 0x3FF);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG2, val);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG2 + ISP_CH1_ADDR_OFFSET, val);

	val = ((edge_info.ee_gain_hv_r[1][2]  & 0x1F)  << 20) |
		  ((edge_info.ee_gain_hv_r[1][1]  & 0x1F)  << 15) |
		  ((edge_info.ee_gain_hv_r[1][0]  & 0x1F) << 10) |
		   (edge_info.ee_gain_hv_t[1][3]  & 0x3FF);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG4, val);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG4 + ISP_CH1_ADDR_OFFSET, val);

	for (i = 0; i < 2; i++) {
		val = ((edge_info.ee_gain_diag_t[i][2]	& 0x3FF) << 20) |
			  ((edge_info.ee_gain_diag_t[i][1] & 0x3FF) << 10) |
			   (edge_info.ee_gain_diag_t[i][0] & 0x3FF);
		ISP_REG_WR(idx, ISP_EE_LUM_CFG5 + 8*i, val);
		ISP_REG_WR(idx,
			ISP_EE_LUM_CFG5 + ISP_CH1_ADDR_OFFSET + 8*i, val);

		val = ((edge_info.ee_gain_diag_r[i][2]	& 0x1F)	 << 20) |
			  ((edge_info.ee_gain_diag_r[i][1] & 0x1F) << 15) |
			  ((edge_info.ee_gain_diag_r[i][0] & 0x1F) << 10) |
			   (edge_info.ee_gain_diag_t[i][3] & 0x3FF);
		ISP_REG_WR(idx, ISP_EE_LUM_CFG6 + 8*i, val);
		ISP_REG_WR(idx,
			ISP_EE_LUM_CFG6 + ISP_CH1_ADDR_OFFSET + 8*i, val);
	}

	val = ((edge_info.ee_lum_t[3]  & 0xFF) << 24) |
		  ((edge_info.ee_lum_t[2]  & 0xFF) << 16) |
		  ((edge_info.ee_lum_t[1]  & 0xFF) << 8) |
		   (edge_info.ee_lum_t[0]  & 0xFF);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG9, val);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG9 + ISP_CH1_ADDR_OFFSET, val);

	val = ((edge_info.ee_lum_r[2]  & 0x7F) << 14) |
		  ((edge_info.ee_lum_r[1]  & 0x7F) << 7) |
		   (edge_info.ee_lum_r[0]  & 0x7F);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG10, val);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG10 + ISP_CH1_ADDR_OFFSET, val);

	val = ((edge_info.ee_pos_t[2]  & 0x3FF)	<< 20) |
		  ((edge_info.ee_pos_t[1]  & 0x3FF)	<< 10) |
		   (edge_info.ee_pos_t[0]  & 0x3FF);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG11, val);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG11 + ISP_CH1_ADDR_OFFSET, val);

	val = ((edge_info.ee_pos_r[2]  & 0x7F)	<< 24) |
		  ((edge_info.ee_pos_r[1]  & 0x7F)	<< 17) |
		  ((edge_info.ee_pos_r[0]  & 0x7F)	<< 10) |
		   (edge_info.ee_pos_t[3]  & 0x3FF);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG12, val);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG12 + ISP_CH1_ADDR_OFFSET, val);

	val = ((edge_info.ee_pos_c[2]  & 0x7F) << 14) |
		  ((edge_info.ee_pos_c[1]  & 0x7F) << 7) |
		   (edge_info.ee_pos_c[0]  & 0x7F);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG13, val);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG13 + ISP_CH1_ADDR_OFFSET, val);

	val = ((edge_info.ee_neg_t[2]  & 0x3FF)	<< 20) |
		  ((edge_info.ee_neg_t[1]  & 0x3FF)	<< 10) |
		   (edge_info.ee_neg_t[0]  & 0x3FF);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG14, val);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG14 + ISP_CH1_ADDR_OFFSET, val);

	val = ((edge_info.ee_neg_r[2]  & 0x7F)	<< 24) |
		  ((edge_info.ee_neg_r[1]  & 0x7F)	<< 17) |
		  ((edge_info.ee_neg_r[0]  & 0x7F)	<< 10) |
		   (edge_info.ee_neg_t[3]  & 0x3FF);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG15, val);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG15 + ISP_CH1_ADDR_OFFSET, val);

	val = ((edge_info.ee_neg_c[2]  & 0xFF) << 16) |
		  ((edge_info.ee_neg_c[1]  & 0xFF) << 8) |
		   (edge_info.ee_neg_c[0]  & 0xFF);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG16, val);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG16 + ISP_CH1_ADDR_OFFSET, val);

	val = ((edge_info.ee_freq_t[2]	& 0x3FF) << 20) |
		  ((edge_info.ee_freq_t[1]	& 0x3FF) << 10) |
		   (edge_info.ee_freq_t[0]	& 0x3FF);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG17, val);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG17 + ISP_CH1_ADDR_OFFSET, val);

	val = ((edge_info.ee_freq_t[3]	& 0x3FF) << 18) |
		  ((edge_info.ee_freq_r[2]	& 0x3F)	 << 12) |
		  ((edge_info.ee_freq_r[1]	& 0x3F)	 << 6) |
		   (edge_info.ee_freq_r[0]	& 0x3F);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG18, val);
	ISP_REG_WR(idx, ISP_EE_LUM_CFG18 + ISP_CH1_ADDR_OFFSET, val);

	return ret;
}

int isp_k_cfg_edge(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_edge: param is null error.\n");
		return -1;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_edge: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_EDGE_BLOCK:
		ret = isp_k_edge_block(param, idx);
		break;
	case ISP_PRO_EDGE_BYPASS:
		ret = isp_k_edge_bypass(param, idx);
		break;
	case ISP_PRO_EDGE_PARAM:
		ret = isp_k_edge_param(param, idx);
		break;

	default:
		pr_err("isp_k_cfg_edge: fail cmd id:%d, not supported.\n",
			param->property);
		break;
	}

	return ret;
}


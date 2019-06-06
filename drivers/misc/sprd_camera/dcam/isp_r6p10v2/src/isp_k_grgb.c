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

static int isp_k_grgb_bypass
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int bypass = 0;

	ret = copy_from_user((void *)&bypass,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_grgb_bypass: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (bypass)
		ISP_REG_OWR(idx, ISP_GRGB_CTRL, BIT_0);
	else
		ISP_REG_MWR(idx, ISP_GRGB_CTRL, BIT_0, 0);

	return ret;
}

static int isp_k_grgb_param(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct grgb_param_info grgb;

	memset(&grgb, 0x00, sizeof(grgb));
	ret = copy_from_user((void *)&grgb,
		param->property_param, sizeof(struct grgb_param_info));
	if (ret != 0) {
		pr_err("isp_k_grgb_param: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	val = ((grgb.grgb_diff_th & 0x3FF) << 1)
		| ((grgb.grgb_hv_edge_thr & 0x7F) << 16)
		| ((grgb.grgb_slash_edge_thr & 0x7F) << 24);
	ISP_REG_MWR(idx, ISP_GRGB_CTRL, 0x7F7F07FE, val);

	val = (grgb.grgb_slash_flat_thr & 0x3FF)
		| ((grgb.grgb_gr_ratio & 0x1F) << 10)
		| ((grgb.grgb_hv_flat_thr & 0x3FF) << 16)
		| ((grgb.grgb_gb_ratio & 0x1F) << 26);
	ISP_REG_WR(idx, ISP_GRGB_CFG0, val);

	val = (grgb.grgb_lum_curve_flat_t3 & 0x3FF)
		| ((grgb.grgb_lum_curve_flat_t2 & 0x3FF) << 10)
		| ((grgb.grgb_lum_curve_flat_t1 & 0x3FF) << 20);
	ISP_REG_WR(idx, ISP_GRGB_LUM_FLAT_T, val);

	val = (grgb.grgb_lum_curve_flat_r3 & 0x1F)
		| ((grgb.grgb_lum_curve_flat_r2 & 0x1F) << 5)
		| ((grgb.grgb_lum_curve_flat_r1 & 0x1F) << 10)
		| ((grgb.grgb_lum_curve_flat_t4 & 0x3FF) << 15);
	ISP_REG_WR(idx, ISP_GRGB_LUM_FLAT_R, val);

	val = (grgb.grgb_lum_curve_edge_t3 & 0x3FF)
		| ((grgb.grgb_lum_curve_edge_t2 & 0x3FF) << 10)
		| ((grgb.grgb_lum_curve_edge_t1 & 0x3FF) << 20);
	ISP_REG_WR(idx, ISP_GRGB_LUM_EDGE_T, val);

	val = (grgb.grgb_lum_curve_edge_r3 & 0x1F)
		| ((grgb.grgb_lum_curve_edge_r2 & 0x1F) << 5)
		| ((grgb.grgb_lum_curve_edge_r1 & 0x1F) << 10)
		| ((grgb.grgb_lum_curve_edge_t4 & 0x3FF) << 15);
	ISP_REG_WR(idx, ISP_GRGB_LUM_EDGE_R, val);

	val = (grgb.grgb_lum_curve_texture_t3 & 0x3FF)
		| ((grgb.grgb_lum_curve_texture_t2 & 0x3FF) << 10)
		| ((grgb.grgb_lum_curve_texture_t1 & 0x3FF) << 20);
	ISP_REG_WR(idx, ISP_GRGB_LUM_TEX_T, val);

	val = (grgb.grgb_lum_curve_texture_r3 & 0x1F)
		| ((grgb.grgb_lum_curve_texture_r2 & 0x1F) << 5)
		| ((grgb.grgb_lum_curve_texture_r1 & 0x1F) << 10)
		| ((grgb.grgb_lum_curve_texture_t4 & 0x3FF) << 15);
	ISP_REG_WR(idx, ISP_GRGB_LUM_TEX_R, val);

	val = (grgb.grgb_frez_curve_flat_t3 & 0x3FF)
		| ((grgb.grgb_frez_curve_flat_t2 & 0x3FF) << 10)
		| ((grgb.grgb_frez_curve_flat_t1 & 0x3FF) << 20);
	ISP_REG_WR(idx, ISP_GRGB_FREZ_FLAT_T, val);

	val = (grgb.grgb_frez_curve_flat_r3 & 0x1F)
		| ((grgb.grgb_frez_curve_flat_r2 & 0x1F) << 5)
		| ((grgb.grgb_frez_curve_flat_r1 & 0x1F) << 10)
		| ((grgb.grgb_frez_curve_flat_t4 & 0x3FF) << 15);
	ISP_REG_WR(idx, ISP_GRGB_FREZ_FLAT_R, val);

	val = (grgb.grgb_frez_curve_edge_t3 & 0x3FF)
		| ((grgb.grgb_frez_curve_edge_t2 & 0x3FF) << 10)
		| ((grgb.grgb_frez_curve_edge_t1 & 0x3FF) << 20);
	ISP_REG_WR(idx, ISP_GRGB_FREZ_EDGE_T, val);

	val = (grgb.grgb_frez_curve_edge_r3 & 0x1F)
		| ((grgb.grgb_frez_curve_edge_r2 & 0x1F) << 5)
		| ((grgb.grgb_frez_curve_edge_r1 & 0x1F) << 10)
		| ((grgb.grgb_frez_curve_edge_t4 & 0x3FF) << 15);
	ISP_REG_WR(idx, ISP_GRGB_FREZ_EDGE_R, val);

	val = (grgb.grgb_frez_curve_texture_t3 & 0x3FF)
		| ((grgb.grgb_frez_curve_texture_t2 & 0x3FF) << 10)
		| ((grgb.grgb_frez_curve_texture_t1 & 0x3FF) << 20);
	ISP_REG_WR(idx, ISP_GRGB_FREZ_TEX_T, val);

	val = (grgb.grgb_frez_curve_texture_r3 & 0x1F)
		| ((grgb.grgb_frez_curve_texture_r2 & 0x1F) << 5)
		| ((grgb.grgb_frez_curve_texture_r1 & 0x1F) << 10)
		| ((grgb.grgb_frez_curve_texture_t4 & 0x3FF) << 15);
	ISP_REG_WR(idx, ISP_GRGB_FREZ_TEX_R, val);

	return ret;
}

static int isp_k_grgb_chk_sum_clr(struct isp_io_param *param,
							enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_grgb_chk_sum_clr: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (val)
		ISP_REG_OWR(idx, ISP_GRGB_CTRL, BIT_23);
	else
		ISP_REG_MWR(idx, ISP_GRGB_CTRL, BIT_23, 0);

	return ret;
}

static int isp_k_grgb_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	int  i = 0;
	struct isp_dev_grgb_info grgb_info;

	memset(&grgb_info, 0x00, sizeof(grgb_info));
	ret = copy_from_user((void *)&grgb_info,
		param->property_param, sizeof(grgb_info));
	if (ret != 0) {
		pr_err("isp_k_grgb_block: copy error, ret=0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	ISP_REG_MWR(idx, ISP_GRGB_CTRL, BIT_0, grgb_info.bypass);
	if (grgb_info.bypass)
		return 0;

	ISP_REG_MWR(idx, ISP_GRGB_CTRL, BIT_23, grgb_info.check_sum_clr << 23);
	val = ((grgb_info.slash_edge_thr & 0x7F)  << 24) |
		  (grgb_info.check_sum_clr & 0x01) << 23 |
		  ((grgb_info.hv_edge_thr & 0x7F)  << 16) |
		  ((grgb_info.diff_thd & 0x3FF) << 1);
	ISP_REG_MWR(idx, ISP_GRGB_CTRL, 0x7FFF07FE, val);

	val = ((grgb_info.gb_ratio & 0x1F)  << 26) |
		  ((grgb_info.hv_flat_thr & 0x3FF) << 16) |
		  ((grgb_info.gr_ratio & 0x1F)  << 10) |
		  (grgb_info.slash_flat_thr & 0x3FF);
	ISP_REG_WR(idx, ISP_GRGB_CFG0, val);

	for (i = 0; i < 3; i++) {
		val = ((grgb_info.lum.curve_t[i][0] & 0x3FF) << 20) |
			  ((grgb_info.lum.curve_t[i][1] & 0x3FF) << 10) |
			   (grgb_info.lum.curve_t[i][2] & 0x3FF);
		ISP_REG_WR(idx, ISP_GRGB_LUM_FLAT_T + i * 8, val);

		val = ((grgb_info.lum.curve_t[i][3] & 0x3FF) << 15) |
			  ((grgb_info.lum.curve_r[i][0] & 0x1F)	 << 10) |
			  ((grgb_info.lum.curve_r[i][1] & 0x1F)	 << 5) |
			   (grgb_info.lum.curve_r[i][2] & 0x1F);
		ISP_REG_WR(idx, ISP_GRGB_LUM_FLAT_T_R + i * 8, val);
	}

	for (i = 0; i < 3; i++) {
		val = ((grgb_info.frez.curve_t[i][0] & 0x3FF) << 20) |
			  ((grgb_info.frez.curve_t[i][1] & 0x3FF) << 10) |
			   (grgb_info.frez.curve_t[i][2] & 0x3FF);
		ISP_REG_WR(idx, ISP_GRGB_FREZ_FLAT_T + i * 8, val);

		val = ((grgb_info.frez.curve_t[i][3] & 0x3FF) << 15) |
			  ((grgb_info.frez.curve_r[i][0] & 0x1F)  << 10) |
			  ((grgb_info.frez.curve_r[i][1] & 0x1F)  << 5) |
			   (grgb_info.frez.curve_r[i][2] & 0x1F);
		ISP_REG_WR(idx, ISP_GRGB_FREZ_FLAT_T_R + i * 8, val);
	}

	return ret;
}

int isp_k_cfg_grgb(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_grgb: param is null error.\n");
		return -1;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_grgb: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_GRGB_BLOCK:
		ret = isp_k_grgb_block(param, idx);
		break;
	case ISP_PRO_GRGB_BYPASS:
		ret = isp_k_grgb_bypass(param, idx);
		break;
	case ISP_PRO_GRGB_PARAM:
		ret = isp_k_grgb_param(param, idx);
		break;
	case ISP_PRO_GRGB_CHK_SUM_CLR:
		ret = isp_k_grgb_chk_sum_clr(param, idx);
		break;
	default:
		pr_err("isp_k_cfg_grgb: fail cmd id:%d, not supported.\n",
			param->property);
		break;
	}

	return ret;
}


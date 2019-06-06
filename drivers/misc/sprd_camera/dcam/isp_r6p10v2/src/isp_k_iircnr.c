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

static int isp_k_iircnr_bypass(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int bypass = 0;

	ret = copy_from_user((void *)&bypass,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_iircnr_bypass: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (bypass)
		ISP_REG_OWR(idx, ISP_IIRCNR_PARAM, BIT_0);
	else
		ISP_REG_MWR(idx, ISP_IIRCNR_PARAM, BIT_0, 0);

	return ret;
}

static int isp_k_iircnr_chk_sum_clr
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_iircnr_chk_sum_clr: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (val)
		ISP_REG_OWR(idx, ISP_IIRCNR_PARAM, BIT_1);
	else
		ISP_REG_MWR(idx, ISP_IIRCNR_PARAM, BIT_1, 0);

	return ret;
}

static int isp_k_iircnr_mode(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_iircnr_mode: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (val)
		ISP_REG_OWR(idx, ISP_IIRCNR_PARAM1, BIT_0);
	else
		ISP_REG_MWR(idx, ISP_IIRCNR_PARAM1, BIT_0, 0);

	return ret;
}

static int isp_k_iircnr_thr(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	unsigned int i = 0;
	struct iircnr_thr thr = {0};

	ret = copy_from_user((void *)&thr,
		param->property_param, sizeof(thr));
	if (ret != 0) {
		pr_err("isp_k_iircnr_thr: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	val = ((thr.y_min_th & 0xFF) << 20)
		| ((thr.y_max_th & 0xFF) << 12)
		| ((thr.uv_th & 0x7FF) << 1);
	ISP_REG_MWR(idx, ISP_IIRCNR_PARAM1, 0xFFFFFFE, val);

	val = ((thr.sat_ratio & 0x7F) << 24)
		| ((thr.uv_pg_th & 0x3FFF) << 10)
		| (thr.uv_dist & 0x3FF);
	ISP_REG_WR(idx, ISP_IIRCNR_PARAM2, val);

	val = ((thr.uv_low_thr2_tbl[0] & 0x1FFFF) << 14)
		| (thr.uv_low_thr1_tbl[0] & 0x3FFF);
	ISP_REG_WR(idx, ISP_IIRCNR_PARAM3, val);

	val = ((thr.y_th & 0xFF) << 19)
		| ((thr.slope_y[0] & 0x7FF) << 8)
		| (thr.uv_s_th & 0xFF);
	ISP_REG_MWR(idx, ISP_IIRCNR_PARAM6, 0x7FFFFFF, val);

	val = (thr.alpha_low_v & 0x3FFF)
		| ((thr.alpha_low_u & 0x3FFF) << 14);
	ISP_REG_WR(idx, ISP_IIRCNR_PARAM7, val);

	val = thr.middle_factor_y[0] & 0x7FFFFFF;
	ISP_REG_WR(idx, ISP_IIRCNR_PARAM8, val);

	val = thr.uv_high_thr2_tbl[0] & 0x1FFFF;
	ISP_REG_WR(idx, ISP_IIRCNR_PARAM9, val);

	for (i = 0; i < 8; i++) {
		if (i > 0) {
			val = ((thr.uv_low_thr2_tbl[i] & 0x1FFFF) << 14)
				| (thr.uv_low_thr1_tbl[i] & 0x3FFF);
			ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_2 + (i-1)*4, val);

			val = ((thr.uv_high_thr2_tbl[i] & 0x1FFFF) << 11)
				| (thr.slope_y[i] & 0x7FF);
			ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_17 + (i-1)*4, val);

			val = thr.middle_factor_y[i] & 0x7FFFFFF;
			ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_24 + (i-1)*4, val);
		}
		if (i < 4) {
			val = (thr.y_edge_thr_max[2*i] & 0xFFFF)
				| ((thr.y_edge_thr_max[2*i+1] & 0xFFFF) << 16);
			ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_9 + i*4, val);

			val = (thr.y_edge_thr_min[2*i] & 0xFFFF)
				| ((thr.y_edge_thr_min[2*i+1] & 0xFFFF) << 16);
			ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_13 + i*4, val);
		}
		val = ((thr.middle_factor_uv[i] & 0xFFFFFF) << 7)
			| (thr.slope_uv[i] & 0x7F);
		ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_31 + i*4, val);
	}
	val = ((thr.pre_uv_th & 0xFF) << 16)
		| ((thr.css_lum_thr & 0xFF) << 8)
		| (thr.uv_diff_thr & 0xFF);
	ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_39, val);

	return ret;
}

static int isp_k_iircnr_ymd(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct iircnr_ymd ymd = {0};

	ret = copy_from_user((void *)&ymd,
		param->property_param, sizeof(ymd));
	if (ret != 0) {
		pr_err("isp_k_iircnr_ymd: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	val = ymd.ymd_u & 0x3FFFFFFF;
	ISP_REG_WR(idx, ISP_IIRCNR_PARAM4, val);

	val = ymd.ymd_v & 0x3FFFFFFF;
	ISP_REG_WR(idx, ISP_IIRCNR_PARAM5, val);

	val = ymd.ymd_min_u & 0x3FFFFFFF;
	ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_0, val);

	val = ymd.ymd_min_v & 0x3FFFFFFF;
	ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_1, val);

	return ret;
}

static int isp_k_yrandom_seed(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_yrandom_seed: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_YRANDOM_PARAM0, 0xFFFFFF00, val << 8);

	return ret;
}

static int isp_k_yrandom_mode(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_yrandom_mode: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_YRANDOM_PARAM0, BIT_2, val << 2);

	return ret;
}

static int isp_k_yrandom_init(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_yrandom_init: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_WR(idx, ISP_YRANDOM_INIT, val);

	return ret;
}

static int isp_k_yrandom_bypass(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_yrandom_bypass: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_YRANDOM_PARAM0, BIT_0, val);

	return ret;
}

static int isp_k_yrandom_offset(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_yrandom_offset: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_YRANDOM_PARAM1, 0x7FF0000, val << 16);

	return ret;
}

static int isp_k_yrandom_shift(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_yrandom_shift: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_YRANDOM_PARAM1, 0xF, val);

	return ret;
}

static int isp_k_yrandom_takebit
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct yrandom_takebit takebit;

	memset(&takebit, 0x00, sizeof(takebit));
	ret = copy_from_user((void *)&takebit,
		param->property_param, sizeof(takebit));
	if (ret != 0) {
		pr_err("isp_k_yrandom_takebit: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	val = (takebit.takeBit[0] & 0xF)
		| ((takebit.takeBit[1] & 0xF) << 4)
		| ((takebit.takeBit[2] & 0xF) << 8)
		| ((takebit.takeBit[3] & 0xF) << 12)
		| ((takebit.takeBit[4] & 0xF) << 16)
		| ((takebit.takeBit[5] & 0xF) << 20)
		| ((takebit.takeBit[6] & 0xF) << 24)
		| ((takebit.takeBit[7] & 0xF) << 28);
	ISP_REG_WR(idx, ISP_YRANDOM_PARAM2, val);

	return ret;
}

static int isp_k_iircnr_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	struct isp_dev_iircnr_info iircnr_info;
	unsigned int val = 0;
	unsigned int i = 0;

	memset(&iircnr_info, 0x00, sizeof(iircnr_info));

	ret = copy_from_user((void *)&iircnr_info,
		param->property_param, sizeof(iircnr_info));
	if (ret != 0) {
		pr_err("isp_k_iircnr_block: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	ISP_REG_MWR(idx, ISP_IIRCNR_PARAM, BIT_0, iircnr_info.bypass);
	ISP_REG_MWR(idx, ISP_IIRCNR_PARAM + ISP_CH1_ADDR_OFFSET,
		BIT_0, iircnr_info.bypass);

	if (iircnr_info.bypass)
		return 0;
	ISP_REG_MWR(idx, ISP_IIRCNR_PARAM1,
		BIT_0, iircnr_info.mode);
	ISP_REG_MWR(idx, ISP_IIRCNR_PARAM1 + ISP_CH1_ADDR_OFFSET,
		BIT_0, iircnr_info.mode);

	val = ((iircnr_info.y_min_th & 0xFF)  << 20) |
		  ((iircnr_info.y_max_th & 0xFF)  << 12) |
		  ((iircnr_info.uv_th & 0x7FF) << 1);
	ISP_REG_MWR(idx, ISP_IIRCNR_PARAM1, 0xFFFFFFE, val);
	ISP_REG_MWR(idx, ISP_IIRCNR_PARAM1 + ISP_CH1_ADDR_OFFSET,
		0xFFFFFFE, val);

	val = ((iircnr_info.sat_ratio & 0x7F) << 24) |
		  ((iircnr_info.uv_pg_th  & 0x3FFF) << 10) |
		   (iircnr_info.uv_dist	  & 0x3FF);
	ISP_REG_WR(idx, ISP_IIRCNR_PARAM2, val);
	ISP_REG_WR(idx, ISP_IIRCNR_PARAM2 + ISP_CH1_ADDR_OFFSET, val);

	val = ((iircnr_info.uv_low_thr2 & 0x1FFFF) << 14) |
		(iircnr_info.uv_low_thr1 & 0x3FFF);
	ISP_REG_WR(idx, ISP_IIRCNR_PARAM3, val);
	ISP_REG_WR(idx, ISP_IIRCNR_PARAM3 + ISP_CH1_ADDR_OFFSET, val);

	val = iircnr_info.ymd_u & 0x3FFFFFFF;
	ISP_REG_WR(idx, ISP_IIRCNR_PARAM4, val);
	ISP_REG_WR(idx, ISP_IIRCNR_PARAM4 + ISP_CH1_ADDR_OFFSET, val);

	val = iircnr_info.ymd_v & 0x3FFFFFFF;
	ISP_REG_WR(idx, ISP_IIRCNR_PARAM5, val);
	ISP_REG_WR(idx, ISP_IIRCNR_PARAM5 + ISP_CH1_ADDR_OFFSET, val);

	val = ((iircnr_info.y_th & 0xFF)  << 19) |
		  ((iircnr_info.slope_y_0 & 0x7FF) << 8) |
		   (iircnr_info.uv_s_th & 0xFF);
	ISP_REG_WR(idx, ISP_IIRCNR_PARAM6, val);
	ISP_REG_WR(idx, ISP_IIRCNR_PARAM6 + ISP_CH1_ADDR_OFFSET, val);

	val = (iircnr_info.alpha_low_v & 0x3FFF)
		| ((iircnr_info.alpha_low_u & 0x3FFF) << 14);
	ISP_REG_WR(idx, ISP_IIRCNR_PARAM7, val);
	ISP_REG_WR(idx, ISP_IIRCNR_PARAM7 + ISP_CH1_ADDR_OFFSET, val);

	ISP_REG_WR(idx, ISP_IIRCNR_PARAM8,
		iircnr_info.middle_factor_y_0 & 0x7FFFFFF);
	ISP_REG_WR(idx, ISP_IIRCNR_PARAM8 + ISP_CH1_ADDR_OFFSET,
		iircnr_info.middle_factor_y_0 & 0x7FFFFFF);

	val = iircnr_info.uv_high_thr2_0 & 0x1FFFF;
	ISP_REG_WR(idx, ISP_IIRCNR_PARAM9, val);
	ISP_REG_WR(idx, ISP_IIRCNR_PARAM9 + ISP_CH1_ADDR_OFFSET, val);

	ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_0,
			iircnr_info.ymd_min_u & 0x3FFFFFFF);
	ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_0 + ISP_CH1_ADDR_OFFSET,
			iircnr_info.ymd_min_u & 0x3FFFFFFF);
	ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_1,
			iircnr_info.ymd_min_v & 0x3FFFFFFF);
	ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_1 + ISP_CH1_ADDR_OFFSET,
			iircnr_info.ymd_min_v & 0x3FFFFFFF);

	for (i = 0; i < 7; i++) {
		val = ((iircnr_info.uv_low_thr[i][0] & 0x1FFFF) << 14) |
			   (iircnr_info.uv_low_thr[i][1] & 0x3FFF);
		ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_2 + 4*i, val);
		ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_2 + 4*i
			+ ISP_CH1_ADDR_OFFSET, val);
	}

	for (i = 0; i < 8; i += 2) {
		val = ((iircnr_info.y_edge_thr_max[i+1] & 0xFFFF) << 16) |
			   (iircnr_info.y_edge_thr_max[i] & 0xFFFF);
		ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_9 + 4*(i>>1), val);
		ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_9 + 4*(i>>1)
			 + ISP_CH1_ADDR_OFFSET, val);

		val = ((iircnr_info.y_edge_thr_min[i+1] & 0xFFFF) << 16) |
			   (iircnr_info.y_edge_thr_min[i] & 0xFFFF);
		ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_13 + 4*(i>>1), val);
		ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_13 + 4*(i>>1)
			 + ISP_CH1_ADDR_OFFSET, val);
	}

	for (i = 0; i < 7; i++) {
		val  = ((iircnr_info.uv_high_thr2[i] & 0x1FFFF) << 11) |
			   (iircnr_info.slope_y[i] & 0x7FF);
		ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_17 + 4*i, val);
		ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_17 + 4*i
			 + ISP_CH1_ADDR_OFFSET, val);

		ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_24 + 4*i,
			iircnr_info.middle_factor_y[i] & 0x7FFFFFF);
		ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_24 + 4*i
			 + ISP_CH1_ADDR_OFFSET,
			iircnr_info.middle_factor_y[i] & 0x7FFFFFF);
	}

	for (i = 0; i < 8; i++) {
		val = ((iircnr_info.middle_factor_uv[i] & 0xFFFFFF) << 7) |
			   (iircnr_info.slope_uv[i] & 0x7F);
		ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_31 + 4*i, val);
		ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_31 + 4*i
			 + ISP_CH1_ADDR_OFFSET, val);
	}

	val = ((iircnr_info.pre_uv_th   & 0xFF) << 16) |
		  ((iircnr_info.css_lum_thr & 0xFF) << 8) |
		   (iircnr_info.uv_diff_thr & 0xFF);
	ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_39, val);
	ISP_REG_WR(idx, ISP_YUV_IIRCNR_NEW_39 + ISP_CH1_ADDR_OFFSET, val);


	return ret;
}

static int32_t isp_k_yrandom_block(struct isp_io_param *param, enum isp_id idx)
{
	int32_t ret = 0;
	struct isp_dev_yrandom_info yrandom_info;
	uint32_t val = 0;

	memset(&yrandom_info, 0x00, sizeof(yrandom_info));

	ret = copy_from_user((void *)&yrandom_info,
			param->property_param, sizeof(yrandom_info));
	if (ret != 0) {
		pr_err("isp_k_yrandom_block: err, ret = 0x%x\n", (uint32_t)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_YRANDOM_PARAM0, BIT_0, yrandom_info.bypass);
	ISP_REG_MWR(idx, ISP_YRANDOM_PARAM0 + ISP_CH1_ADDR_OFFSET, BIT_0,
		yrandom_info.bypass);
	if (yrandom_info.bypass)
		return 0;

	ISP_REG_MWR(idx, ISP_YRANDOM_PARAM0, 0xFFFFFF00,
		yrandom_info.seed << 8);
	ISP_REG_MWR(idx, ISP_YRANDOM_PARAM0, BIT_2, yrandom_info.mode << 2);
	ISP_REG_MWR(idx, ISP_YRANDOM_PARAM0, BIT_0, yrandom_info.init);
	ISP_REG_MWR(idx, ISP_YRANDOM_PARAM1, 0x7FF0000,
		yrandom_info.offset << 16);
	ISP_REG_MWR(idx, ISP_YRANDOM_PARAM1, 0xF, yrandom_info.shift);
	ISP_REG_MWR(idx, ISP_YRANDOM_PARAM0 + ISP_CH1_ADDR_OFFSET, 0xFFFFFF00,
		yrandom_info.seed << 8);
	ISP_REG_MWR(idx, ISP_YRANDOM_PARAM0 + ISP_CH1_ADDR_OFFSET, BIT_2,
		yrandom_info.mode << 2);
	ISP_REG_MWR(idx, ISP_YRANDOM_PARAM0 + ISP_CH1_ADDR_OFFSET, BIT_0,
		yrandom_info.init);
	ISP_REG_MWR(idx, ISP_YRANDOM_PARAM1 + ISP_CH1_ADDR_OFFSET, 0x7FF0000,
		yrandom_info.offset << 16);
	ISP_REG_MWR(idx, ISP_YRANDOM_PARAM1 + ISP_CH1_ADDR_OFFSET, 0xF,
		yrandom_info.shift);

	val = (yrandom_info.takeBit[0] & 0xF)
		| ((yrandom_info.takeBit[1] & 0xF) << 4)
		| ((yrandom_info.takeBit[2] & 0xF) << 8)
		| ((yrandom_info.takeBit[3] & 0xF) << 12)
		| ((yrandom_info.takeBit[4] & 0xF) << 16)
		| ((yrandom_info.takeBit[5] & 0xF) << 20)
		| ((yrandom_info.takeBit[6] & 0xF) << 24)
		| ((yrandom_info.takeBit[7] & 0xF) << 28);
	ISP_REG_WR(idx, ISP_YRANDOM_PARAM2, val);
	ISP_REG_WR(idx, ISP_YRANDOM_PARAM2 + ISP_CH1_ADDR_OFFSET, val);

	return ret;
}

int isp_k_cfg_iircnr(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_iircnr: param is null error.\n");
		return -1;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_iircnr: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_IIRCNR_BLOCK:
		ret = isp_k_iircnr_block(param, idx);
		break;
	case ISP_PRO_IIRCNR_BYPASS:
		ret = isp_k_iircnr_bypass(param, idx);
		break;
	case ISP_PRO_IIRCNR_MODE:
		ret = isp_k_iircnr_mode(param, idx);
		break;
	case ISP_PRO_IIRCNR_THR:
		ret = isp_k_iircnr_thr(param, idx);
		break;
	case ISP_PRO_IIRCNR_YMD:
		ret = isp_k_iircnr_ymd(param, idx);
		break;
	case ISP_PRO_IIRCNR_CHK_SUM_CLR:
		ret = isp_k_iircnr_chk_sum_clr(param, idx);
		break;
	case ISP_PRO_YRANDOM_SEED:
		ret = isp_k_yrandom_seed(param, idx);
		break;
	case ISP_PRO_YRANDOM_MODE:
		ret = isp_k_yrandom_mode(param, idx);
		break;
	case ISP_PRO_YRANDOM_INIT:
		ret = isp_k_yrandom_init(param, idx);
		break;
	case ISP_PRO_YRANDOM_BYPASS:
		ret = isp_k_yrandom_bypass(param, idx);
		break;
	case ISP_PRO_YRANDOM_OFFSET:
		ret = isp_k_yrandom_offset(param, idx);
		break;
	case ISP_PRO_YRANDOM_SHIFT:
		ret = isp_k_yrandom_shift(param, idx);
		break;
	case ISP_PRO_YRANDOM_TAKEBIT:
		ret = isp_k_yrandom_takebit(param, idx);
		break;
	case ISP_PRO_YRANDOM_BLOCK:
		ret = isp_k_yrandom_block(param, idx);
		break;
	default:
		pr_err("isp_k_cfg_iircnr: fail cmd id:%d, not supported.\n",
			param->property);
		break;
	}

	return ret;
}

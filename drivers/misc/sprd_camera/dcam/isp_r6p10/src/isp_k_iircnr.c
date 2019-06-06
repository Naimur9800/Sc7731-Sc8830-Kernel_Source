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
#include <video/sprd_isp_r6p10.h>
#include "../isp_drv.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "IIRCNR: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

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
		pr_err("fail to copy from user, ret = %d\n", ret);
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
		pr_err("fail to copy from user, ret = %d\n", ret);
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
		pr_err("fail to get param\n");
		return -1;
	}

	if (param->property_param == NULL) {
		pr_err("fail to get property_param\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_IIRCNR_BLOCK:
		ret = isp_k_iircnr_block(param, idx);
		break;
	case ISP_PRO_YRANDOM_BLOCK:
		ret = isp_k_yrandom_block(param, idx);
		break;
	default:
		pr_err("fail to support cmd id = %d\n",
			param->property);
		break;
	}

	return ret;
}

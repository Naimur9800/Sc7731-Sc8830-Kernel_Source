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

static int isp_k_yuv_precdn_block(struct isp_io_param *param,
							enum isp_id idx)
{
	int ret = 0;
	struct isp_dev_yuv_precdn_info pre_cdn_info;
	unsigned int val = 0;
	unsigned int i = 0;

	memset(&pre_cdn_info, 0x00, sizeof(pre_cdn_info));

	ret = copy_from_user((void *)&pre_cdn_info,
		param->property_param, sizeof(pre_cdn_info));
	if (ret != 0) {
		pr_err("isp_k_pre_cdn_den_stren: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	ISP_REG_MWR(idx, ISP_PRECDN_PARAM, BIT_0, pre_cdn_info.bypass);
	ISP_REG_MWR(idx, ISP_PRECDN_PARAM + ISP_CH1_ADDR_OFFSET, BIT_0,
		pre_cdn_info.bypass);
	if (pre_cdn_info.bypass)
		return 0;

	if (pre_cdn_info.mode) {
		ISP_REG_OWR(idx, ISP_PRECDN_PARAM, BIT_4);
		ISP_REG_OWR(idx, ISP_PRECDN_PARAM + ISP_CH1_ADDR_OFFSET, BIT_4);
	} else {
		ISP_REG_MWR(idx, ISP_PRECDN_PARAM, BIT_4, 0);
		ISP_REG_MWR(idx, ISP_PRECDN_PARAM + ISP_CH1_ADDR_OFFSET,
			BIT_4, 0);
	}

	if (pre_cdn_info.median_writeback_en) {
		ISP_REG_OWR(idx, ISP_PRECDN_PARAM, BIT_8);
		ISP_REG_OWR(idx, ISP_PRECDN_PARAM + ISP_CH1_ADDR_OFFSET,
			BIT_8);
	} else {
		ISP_REG_MWR(idx, ISP_PRECDN_PARAM, BIT_8, 0);
		ISP_REG_MWR(idx, ISP_PRECDN_PARAM + ISP_CH1_ADDR_OFFSET,
			BIT_8, 0);
	}

	ISP_REG_MWR(idx, ISP_PRECDN_PARAM, BIT_13 | BIT_12,
		pre_cdn_info.median_mode << 12);
	ISP_REG_MWR(idx, ISP_PRECDN_PARAM + ISP_CH1_ADDR_OFFSET,
		BIT_13 | BIT_12, pre_cdn_info.median_mode << 12);

	ISP_REG_MWR(idx, ISP_PRECDN_PARAM, BIT_17 | BIT_16,
		pre_cdn_info.den_stren << 16);
	ISP_REG_MWR(idx, ISP_PRECDN_PARAM + ISP_CH1_ADDR_OFFSET,
		BIT_17 | BIT_16, pre_cdn_info.den_stren << 16);

	if (pre_cdn_info.uv_joint) {
		ISP_REG_OWR(idx, ISP_PRECDN_PARAM, BIT_20);
		ISP_REG_OWR(idx, ISP_PRECDN_PARAM + ISP_CH1_ADDR_OFFSET,
			BIT_20);
	} else {
		ISP_REG_MWR(idx, ISP_PRECDN_PARAM, BIT_20, 0);
		ISP_REG_MWR(idx,
			ISP_PRECDN_PARAM + ISP_CH1_ADDR_OFFSET, BIT_20, 0);
	}

	val = (pre_cdn_info.median_thr_uv.thru0 & 0x7F) |
		  ((pre_cdn_info.median_thr_uv.thru1 & 0xFF) << 8) |
		  ((pre_cdn_info.median_thr_uv.thrv0 & 0x7F) << 16) |
		  ((pre_cdn_info.median_thr_uv.thrv1 & 0xFF) << 24);
	ISP_REG_WR(idx, ISP_PRECDN_MEDIAN_THRUV01, val);
	ISP_REG_WR(idx,
		ISP_PRECDN_MEDIAN_THRUV01 + ISP_CH1_ADDR_OFFSET, val);

	val = (pre_cdn_info.median_thr & 0x1FF) |
		  ((pre_cdn_info.uv_thr & 0xFF) << 16) |
		  ((pre_cdn_info.y_thr & 0xFF) << 24);
	ISP_REG_WR(idx, ISP_PRECDN_THRYUV, val);
	ISP_REG_WR(idx, ISP_PRECDN_THRYUV + ISP_CH1_ADDR_OFFSET, val);

	for (i = 0; i < 3; i++) {
		val = (pre_cdn_info.r_segu[0][i*2] & 0xFF) |
			  ((pre_cdn_info.r_segu[1][i*2] & 0x7)  << 8) |
			  ((pre_cdn_info.r_segu[0][i*2+1] & 0xFF) << 16) |
			  ((pre_cdn_info.r_segu[1][i*2+1] & 0x7)  << 24);
		ISP_REG_WR(idx, ISP_PRECDN_SEGU_0 + i * 4, val);
		ISP_REG_WR(idx,
			ISP_PRECDN_SEGU_0 + i * 4 + ISP_CH1_ADDR_OFFSET, val);
	}
	val = (pre_cdn_info.r_segu[0][6] & 0xFF)
		| ((pre_cdn_info.r_segu[1][6] & 0x7) << 8);
	ISP_REG_WR(idx, ISP_PRECDN_SEGU_3, val);
	ISP_REG_WR(idx, ISP_PRECDN_SEGU_3 + ISP_CH1_ADDR_OFFSET, val);

	for (i = 0; i < 3; i++) {
		val = (pre_cdn_info.r_segv[0][i*2] & 0xFF) |
			  ((pre_cdn_info.r_segv[1][i*2]   & 0x7) << 8) |
			  ((pre_cdn_info.r_segv[0][i*2+1] & 0xFF) << 16) |
			  ((pre_cdn_info.r_segv[1][i*2+1] & 0x7) << 24);
		ISP_REG_WR(idx, ISP_PRECDN_SEGV_0 + i * 4, val);
		ISP_REG_WR(idx,
			ISP_PRECDN_SEGV_0 + i * 4 + ISP_CH1_ADDR_OFFSET, val);
	}
	val = (pre_cdn_info.r_segv[0][6] & 0xFF)
		| ((pre_cdn_info.r_segv[1][6] & 0x7) << 8);
	ISP_REG_WR(idx, ISP_PRECDN_SEGV_3, val);
	ISP_REG_WR(idx, ISP_PRECDN_SEGV_3 + ISP_CH1_ADDR_OFFSET, val);

	for (i = 0; i < 3; i++) {
		val = (pre_cdn_info.r_segy[0][i*2] & 0xFF) |
			  ((pre_cdn_info.r_segy[1][i*2] & 0x7) << 8) |
			  ((pre_cdn_info.r_segy[0][i*2+1] & 0xFF) << 16) |
			  ((pre_cdn_info.r_segy[1][i*2+1] & 0x7) << 24);
		ISP_REG_WR(idx, ISP_PRECDN_SEGY_0 + i * 4, val);
		ISP_REG_WR(idx,
			ISP_PRECDN_SEGY_0 + i * 4 + ISP_CH1_ADDR_OFFSET, val);
	}
	val = (pre_cdn_info.r_segy[0][6] & 0xFF)
		| ((pre_cdn_info.r_segy[1][6] & 0x7) << 8);
	ISP_REG_WR(idx, ISP_PRECDN_SEGY_3, val);
	ISP_REG_WR(idx, ISP_PRECDN_SEGY_3 + ISP_CH1_ADDR_OFFSET, val);

	for (i = 0; i < 3; i++) {
		val =  (pre_cdn_info.r_distw[i*8]   & 0x7) |
			  ((pre_cdn_info.r_distw[i*8+1] & 0x7) << 4) |
			  ((pre_cdn_info.r_distw[i*8+2] & 0x7) << 8) |
			  ((pre_cdn_info.r_distw[i*8+3] & 0x7) << 12) |
			  ((pre_cdn_info.r_distw[i*8+4] & 0x7) << 16) |
			  ((pre_cdn_info.r_distw[i*8+5] & 0x7) << 20) |
			  ((pre_cdn_info.r_distw[i*8+6] & 0x7) << 24) |
			  ((pre_cdn_info.r_distw[i*8+7] & 0x7) << 28);
		ISP_REG_WR(idx, ISP_PRECDN_DISTW0 + i * 4, val);
		ISP_REG_WR(idx,
			ISP_PRECDN_DISTW0 + i * 4 + ISP_CH1_ADDR_OFFSET, val);
	}
	val = pre_cdn_info.r_distw[24] & 0x7;
	ISP_REG_WR(idx, ISP_PRECDN_DISTW3, val);
	ISP_REG_WR(idx,
		ISP_PRECDN_DISTW3 + ISP_CH1_ADDR_OFFSET, val);

	return ret;
}

static int isp_k_pre_cdn_bypass(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int bypass = 0;

	ret = copy_from_user((void *)&bypass,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_pre_cdn_bypass: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (bypass)
		ISP_REG_OWR(idx, ISP_PRECDN_PARAM, BIT_0);
	else
		ISP_REG_MWR(idx, ISP_PRECDN_PARAM, BIT_0, 0);

	return ret;
}

static int isp_k_pre_cdn_mode(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int mode = 0;

	ret = copy_from_user((void *)&mode,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_pre_cdn_mode: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (mode)
		ISP_REG_OWR(idx, ISP_PRECDN_PARAM, BIT_4);
	else
		ISP_REG_MWR(idx, ISP_PRECDN_PARAM, BIT_4, 0);

	return ret;
}

static int isp_k_pre_cdn_median_writeback(struct isp_io_param *param,
								enum isp_id idx)
{
	int ret = 0;
	unsigned int writeback = 0;

	ret = copy_from_user((void *)&writeback,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_pre_cdn_median_writeback: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (writeback)
		ISP_REG_OWR(idx, ISP_PRECDN_PARAM, BIT_8);
	else
		ISP_REG_MWR(idx, ISP_PRECDN_PARAM, BIT_8, 0);

	return ret;
}

static int isp_k_pre_cdn_median_mode(struct isp_io_param *param,
								enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_pre_cdn_median_mode: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_PRECDN_PARAM, BIT_13 | BIT_12, val << 12);

	return ret;
}

static int isp_k_pre_cdn_uv_joint(struct isp_io_param *param,
							enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_pre_cdn_uv_joint: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (val)
		ISP_REG_OWR(idx, ISP_PRECDN_PARAM, BIT_20);
	else
		ISP_REG_MWR(idx, ISP_PRECDN_PARAM, BIT_20, 0);

	return ret;
}

static int isp_k_pre_cdn_den_stren(struct isp_io_param *param,
								enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_pre_cdn_den_stren: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_PRECDN_PARAM, BIT_17 | BIT_16, val << 16);

	return ret;
}

static int isp_k_pre_cdn_thr(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct pre_cdn_thr thr = {0};

	ret = copy_from_user((void *)&thr,
		param->property_param, sizeof(struct pre_cdn_thr));
	if (ret != 0) {
		pr_err("isp_k_pre_cdn_thr: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	val = (thr.median_thr_u[0] & 0x7F)
		| ((thr.median_thr_u[1] & 0xFF) << 8)
		| ((thr.median_thr_v[0] & 0x7F) << 16)
		| ((thr.median_thr_v[1] & 0xFF) << 24);
	ISP_REG_WR(idx, ISP_PRECDN_MEDIAN_THRUV01, val);

	val = (thr.median_thr & 0x1FF)
		| ((thr.uv_thr & 0xFF) << 16)
		| ((thr.y_thr & 0xFF) << 24);
	ISP_REG_WR(idx, ISP_PRECDN_THRYUV, val);

	return ret;
}

static int isp_k_pre_cdn_segyuv(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct pre_cdn_seg seg;
	unsigned int i;

	memset(&seg, 0x00, sizeof(seg));
	ret = copy_from_user((void *)&seg,
		param->property_param, sizeof(struct pre_cdn_seg));
	if (ret != 0) {
		pr_err("isp_k_pre_cdn_segyuv: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	for (i = 0; i < 3; i++) {
		val = (seg.r_segu[0][i*2] & 0xFF)
			| ((seg.r_segu[1][i*2] & 0x7) << 8)
			| ((seg.r_segu[0][i*2+1] & 0xFF) << 16)
			| ((seg.r_segu[1][i*2+1] & 0x7) << 24);
		ISP_REG_WR(idx, ISP_PRECDN_SEGU_0 + i * 4, val);
	}

	val = (seg.r_segu[0][6] & 0xFF) | ((seg.r_segu[1][6] & 0x7) << 8);
	ISP_REG_WR(idx, ISP_PRECDN_SEGU_3, val);

	for (i = 0; i < 3; i++) {
		val = (seg.r_segv[0][i*2] & 0xFF)
			| ((seg.r_segv[1][i*2] & 0x7) << 8)
			| ((seg.r_segv[0][i*2+1] & 0xFF) << 16)
			| ((seg.r_segv[1][i*2+1] & 0x7) << 24);
		ISP_REG_WR(idx, ISP_PRECDN_SEGV_0 + i * 4, val);
	}

	val = (seg.r_segv[0][6] & 0xFF) | ((seg.r_segv[1][6] & 0x7) << 8);
	ISP_REG_WR(idx, ISP_PRECDN_SEGV_3, val);

	for (i = 0; i < 3; i++) {
		val = (seg.r_segy[0][i*2] & 0xFF)
			| ((seg.r_segy[1][i*2] & 0x7) << 8)
			| ((seg.r_segy[0][i*2+1] & 0xFF) << 16)
			| ((seg.r_segy[1][i*2+1] & 0x7) << 24);
		ISP_REG_WR(idx, ISP_PRECDN_SEGY_0 + i * 4, val);
	}

	val = (seg.r_segy[0][6] & 0xFF) | ((seg.r_segy[1][6] & 0x7) << 8);
	ISP_REG_WR(idx, ISP_PRECDN_SEGY_3, val);

	return ret;
}

static int isp_k_pre_cdn_distw(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	unsigned int i;
	struct pre_cdn_dist dist;

	memset(&dist, 0x00, sizeof(dist));
	ret = copy_from_user((void *)&dist,
		param->property_param, sizeof(struct pre_cdn_dist));
	if (ret != 0) {
		pr_err("isp_k_pre_cdn_distw: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	for (i = 0; i < 3; i++) {
		val = (dist.r_distw[i*8] & 0x7)
			| ((dist.r_distw[i*8+1] & 0x7) << 4)
			| ((dist.r_distw[i*8+2] & 0x7) << 8)
			| ((dist.r_distw[i*8+3] & 0x7) << 12)
			| ((dist.r_distw[i*8+4] & 0x7) << 16)
			| ((dist.r_distw[i*8+5] & 0x7) << 20)
			| ((dist.r_distw[i*8+6] & 0x7) << 24)
			| ((dist.r_distw[i*8+7] & 0x7) << 28);
		ISP_REG_WR(idx, ISP_PRECDN_DISTW0 + i * 4, val);
	}

	val = dist.r_distw[24] & 0x7;
	ISP_REG_WR(idx, ISP_PRECDN_DISTW3, val);

	return ret;
}

int isp_k_cfg_pre_cdn(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_pre_cdn: param is null error.\n");
		return -1;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_pre_cdn: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_PRE_CDN_BLOCK:
		ret = isp_k_yuv_precdn_block(param, idx);
		break;
	case ISP_PRO_PRE_CDN_BYPASS:
		ret = isp_k_pre_cdn_bypass(param, idx);
		break;
	case ISP_PRO_PRE_CDN_MODE:
		ret = isp_k_pre_cdn_mode(param, idx);
		break;
	case ISP_PRO_PRE_CDN_MEDIAN_WRITEBACK:
		ret = isp_k_pre_cdn_median_writeback(param, idx);
		break;
	case ISP_PRO_PRE_CDN_MEDIAN_MODE:
		ret = isp_k_pre_cdn_median_mode(param, idx);
		break;
	case ISP_PRO_PRE_CDN_DEN_STREN:
		ret = isp_k_pre_cdn_den_stren(param, idx);
		break;
	case ISP_PRO_PRE_CDN_UV_JOINT:
		ret = isp_k_pre_cdn_uv_joint(param, idx);
		break;
	case ISP_PRO_PRE_CDN_THR:
		ret = isp_k_pre_cdn_thr(param, idx);
		break;
	case ISP_PRO_PRE_CDN_SEGYUV:
		ret = isp_k_pre_cdn_segyuv(param, idx);
		break;
	case ISP_PRO_PRE_CDN_DISTW:
		ret = isp_k_pre_cdn_distw(param, idx);
		break;
	default:
		pr_err("isp_k_cfg_pre_cdn: fail cmd id:%d, not supported.\n",
			param->property);
		break;
	}

	return ret;
}

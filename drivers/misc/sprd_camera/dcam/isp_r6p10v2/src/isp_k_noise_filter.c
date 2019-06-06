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

static int isp_k_noise_filter_block
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	int i = 0;
	struct isp_dev_noise_filter_info nf_info;

	memset(&nf_info, 0x00, sizeof(nf_info));
	ret = copy_from_user((void *)&nf_info,
		param->property_param, sizeof(nf_info));
	if (ret != 0) {
		pr_err("isp_k_noise_filter_bypass: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}


	ISP_REG_MWR(idx, ISP_YUV_NF_CTRL, BIT_0, nf_info.yrandom_bypass);
	ISP_REG_MWR(idx, ISP_YUV_NF_CTRL + ISP_CH1_ADDR_OFFSET,
		BIT_0, nf_info.yrandom_bypass);
	if (nf_info.yrandom_bypass)
		return 0;
	ISP_REG_MWR(idx, ISP_YUV_NF_CTRL, BIT_1, nf_info.shape_mode << 1);
	ISP_REG_MWR(idx, ISP_YUV_NF_CTRL + ISP_CH1_ADDR_OFFSET,
		BIT_1, nf_info.shape_mode << 1);
	ISP_REG_MWR(idx, ISP_YUV_NF_CTRL, 0xC, nf_info.filter_thr_mode << 2);
	ISP_REG_MWR(idx, ISP_YUV_NF_CTRL + ISP_CH1_ADDR_OFFSET,
		0xC, nf_info.filter_thr_mode << 2);
	ISP_REG_MWR(idx, ISP_YUV_NF_CTRL, BIT_5, nf_info.yrandom_mode << 5);
	ISP_REG_MWR(idx, ISP_YUV_NF_CTRL + ISP_CH1_ADDR_OFFSET,
		BIT_5, nf_info.yrandom_mode << 5);

	for (i = 0; i < 4; i++) {
		ISP_REG_WR(idx, ISP_YUV_NF_SEED0 + 4*i,
					nf_info.yrandom_seed[i]);
		ISP_REG_WR(idx, ISP_YUV_NF_SEED0 + 4*i+
			ISP_CH1_ADDR_OFFSET, nf_info.yrandom_seed[i]);
	}

	val = (nf_info.takebit[0]  & 0xF) |
		  ((nf_info.takebit[1] & 0xF) << 4) |
		  ((nf_info.takebit[2] & 0xF) << 8) |
		  ((nf_info.takebit[3] & 0xF) << 12) |
		  ((nf_info.takebit[4] & 0xF) << 16) |
		  ((nf_info.takebit[5] & 0xF) << 20) |
		  ((nf_info.takebit[6] & 0xF) << 24) |
		  ((nf_info.takebit[7] & 0xF) << 28);
	ISP_REG_WR(idx, ISP_YUV_NF_TB4, val);
	ISP_REG_WR(idx, ISP_YUV_NF_TB4 + ISP_CH1_ADDR_OFFSET, val);

	val = ((nf_info.r_shift & 0xF) << 16) |
		   (nf_info.r_offset & 0x7FF);
	ISP_REG_WR(idx, ISP_YUV_NF_SF, val);
	ISP_REG_WR(idx, ISP_YUV_NF_SF + ISP_CH1_ADDR_OFFSET, val);

	ISP_REG_MWR(idx, ISP_YUV_NF_THR, 0xFF, nf_info.filter_thr);
	ISP_REG_MWR(idx, ISP_YUV_NF_THR + ISP_CH1_ADDR_OFFSET,
		0xFF, nf_info.filter_thr);

	val = ((nf_info.cv_t[1] & 0x3FF) << 16)
		| (nf_info.cv_t[0] & 0x3FF);
	ISP_REG_WR(idx, ISP_YUV_NF_CV_T12, val);
	ISP_REG_WR(idx, ISP_YUV_NF_CV_T12 + ISP_CH1_ADDR_OFFSET, val);

	val = ((nf_info.cv_r[2] & 0xFF) << 16) |
		((nf_info.cv_r[1] & 0xFF) << 8) |
		(nf_info.cv_r[0] & 0xFF);
	ISP_REG_WR(idx, ISP_YUV_NF_CV_R, val);
	ISP_REG_WR(idx, ISP_YUV_NF_CV_R + ISP_CH1_ADDR_OFFSET, val);

	val = ((nf_info.noise_clip.p & 0xFF) << 8)
		| (nf_info.noise_clip.n & 0xFF);
	ISP_REG_WR(idx, ISP_YUV_NF_CLIP, val);
	ISP_REG_WR(idx, ISP_YUV_NF_CLIP + ISP_CH1_ADDR_OFFSET, val);

	ISP_REG_MWR(idx, ISP_YUV_NF_SEED_INIT, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_YUV_NF_SEED_INIT + ISP_CH1_ADDR_OFFSET, BIT_0, 1);
	val = ((nf_info.cv_t[3] & 0x3FF) << 16)
		| (nf_info.cv_t[2] & 0x3FF);
	ISP_REG_WR(idx, ISP_YUV_NF_CV_T34, val);
	ISP_REG_WR(idx, ISP_YUV_NF_CV_T34 + ISP_CH1_ADDR_OFFSET, val);

	return ret;
}

static int isp_k_noise_filter_bypass
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int bypass = 0;

	ret = copy_from_user((void *)&bypass,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_noise_filter_bypass: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (bypass)
		ISP_REG_OWR(idx, ISP_YUV_NF_CTRL, BIT_0);
	else
		ISP_REG_MWR(idx, ISP_YUV_NF_CTRL, BIT_0, 0);

	return ret;
}

static int isp_k_noise_filter_param
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct noise_filter_param nf;

	ret = copy_from_user((void *)&nf,
		param->property_param, sizeof(nf));
	if (ret != 0) {
		pr_err("isp_k_noise_filter_param: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	val = ((nf.noisefilter_shape_mode & 0x1) << 1)
		| ((nf.noisefilter_filter_thr_m & 0x3) << 2);
	ISP_REG_MWR(idx, ISP_YUV_NF_CTRL, 0xE, val);

	val = (nf.noisefilter_takeBit[0] & 0xF)
		| ((nf.noisefilter_takeBit[1] & 0xF) << 4)
		| ((nf.noisefilter_takeBit[2] & 0xF) << 8)
		| ((nf.noisefilter_takeBit[3] & 0xF) << 12)
		| ((nf.noisefilter_takeBit[4] & 0xF) << 16)
		| ((nf.noisefilter_takeBit[5] & 0xF) << 20)
		| ((nf.noisefilter_takeBit[6] & 0xF) << 24)
		| ((nf.noisefilter_takeBit[7] & 0xF) << 28);
	ISP_REG_WR(idx, ISP_YUV_NF_TB4, val);

	val = (nf.noisefilter_r_offset & 0x7FF)
		| ((nf.noisefilter_r_shift & 0xF) << 16);
	ISP_REG_WR(idx, ISP_YUV_NF_SF, val);

	val = nf.noisefilter_filter_thr & 0xFF;
	ISP_REG_WR(idx, ISP_YUV_NF_THR, val);

	val = (nf.noisefilter_av_t1 & 0x3FF)
		| ((nf.noisefilter_av_t2 & 0x3FF) << 16);
	ISP_REG_WR(idx, ISP_YUV_NF_CV_T12, val);

	val = (nf.noisefilter_av_r1 & 0xFF)
		| ((nf.noisefilter_av_r2 & 0xFF) << 8)
		| ((nf.noisefilter_av_r3 & 0xFF) << 16);
	ISP_REG_WR(idx, ISP_YUV_NF_CV_R, val);

	val = (nf.noisefilter_clip_n & 0xFF)
		| ((nf.noisefilter_clip_p & 0xFF) << 8);
	ISP_REG_WR(idx, ISP_YUV_NF_CLIP, val);

	val = (nf.noisefilter_av_t3 & 0x3FF)
		| ((nf.noisefilter_av_t4 & 0x3FF) << 16);
	ISP_REG_WR(idx, ISP_YUV_NF_CV_T34, val);

	return ret;
}

static int isp_k_noise_filter_seed
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	unsigned int seed[4];

	ret = copy_from_user((void *)seed,
		param->property_param, sizeof(seed));
	if (ret != 0) {
		pr_err("isp_k_noise_filter_seed: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	val = seed[0] & 0xFFFFFF;
	ISP_REG_WR(idx, ISP_YUV_NF_SEED0, val);

	val = seed[1] & 0xFFFFFF;
	ISP_REG_WR(idx, ISP_YUV_NF_SEED1, val);

	val = seed[2] & 0xFFFFFF;
	ISP_REG_WR(idx, ISP_YUV_NF_SEED2, val);

	val = seed[3] & 0xFFFFFF;
	ISP_REG_WR(idx, ISP_YUV_NF_SEED3, val);

	return ret;
}

static int isp_k_noise_filter_seed_init
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_noise_filter_seed_init: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_WR(idx, ISP_YUV_NF_SEED_INIT, val);

	return ret;
}

int isp_k_cfg_noise_filter(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_noise_filter: param is null error.\n");
		return -1;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_noise_filter: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_NOISE_FILTER_BLOCK:
		ret = isp_k_noise_filter_block(param, idx);
		break;
	case ISP_PRO_NOISE_FILTER_BYPASS:
		ret = isp_k_noise_filter_bypass(param, idx);
		break;
	case ISP_PRO_NOISE_FILTER_PARAM:
		ret = isp_k_noise_filter_param(param, idx);
		break;
	case ISP_PRO_NOISE_FILTER_SEED_INIT:
		ret = isp_k_noise_filter_seed_init(param, idx);
		break;
	case ISP_PRO_NOISE_FILTER_SEED:
		ret = isp_k_noise_filter_seed(param, idx);
		break;
	default:
		pr_err("isp_k_cfg_noise_filter: fail cmd id:%d, not supported.\n",
			param->property);
		break;
	}

	return ret;
}

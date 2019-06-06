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
#define pr_fmt(fmt) "RGBG: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

static int isp_k_rgb_gain_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dev_rgb_gain_info gain_info;

	memset(&gain_info, 0x00, sizeof(gain_info));
	ret = copy_from_user((void *)&gain_info,
		param->property_param, sizeof(gain_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_RGBG_PARAM, BIT_0, gain_info.bypass);
	if (gain_info.bypass)
		return 0;

	if (!gain_info.bypass) {
		val = (gain_info.global_gain & 0xFFFF) << 16;
		ISP_REG_MWR(idx, ISP_RGBG_PARAM, 0xFFFF0000, val);

		val = ((gain_info.r_gain & 0xFFFF) << 16)
			| (gain_info.b_gain & 0xFFFF);
		ISP_REG_WR(idx, ISP_RGBG_RB, val);
		val = gain_info.g_gain & 0xFFFF;
		ISP_REG_WR(idx, ISP_RGBG_G, val);
	}

	return ret;
}

static int isp_k_rgb_gain_bypass
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int bypass = 0;

	ret = copy_from_user((void *)&bypass,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	if (bypass)
		ISP_REG_OWR(idx, ISP_RGBG_PARAM, BIT_0);
	else
		ISP_REG_MWR(idx, ISP_RGBG_PARAM, BIT_0, 0);

	return ret;
}

static int isp_k_rgb_gain_global_gain
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int global_gain = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&global_gain,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	val = (global_gain & 0xFFFF) << 16;

	ISP_REG_MWR(idx, ISP_RGBG_PARAM, 0xFFFF0000, val);

	return ret;
}

static int isp_k_rgb_gain_rgb_gain
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct rgbg_gain gain_para;

	memset(&gain_para, 0x00, sizeof(gain_para));

	ret = copy_from_user((void *)&gain_para,
		param->property_param, sizeof(struct rgbg_gain));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	val = ((gain_para.r_gain & 0xFFFF) << 16) | (gain_para.b_gain & 0xFFFF);
	ISP_REG_WR(idx, ISP_RGBG_RB, val);

	val = gain_para.g_gain & 0xFFFF;
	ISP_REG_WR(idx, ISP_RGBG_G, val);

	return ret;
}

int isp_k_cfg_rgb_gain(struct isp_io_param *param, enum isp_id idx)
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
	case ISP_PRO_RGB_GAIN_BLOCK:
		ret = isp_k_rgb_gain_block(param, idx);
		break;
	case ISP_PRO_RGB_GAIN_BYPASS:
		ret = isp_k_rgb_gain_bypass(param, idx);
		break;
	case ISP_PRO_RGB_GAIN_GLOBAL_GAIN:
		ret = isp_k_rgb_gain_global_gain(param, idx);
		break;
	case ISP_PRO_RGB_GAIN_RGB_GAIN:
		ret = isp_k_rgb_gain_rgb_gain(param, idx);
		break;
	default:
		pr_err("fail to support cmd id = %d\n",
			param->property);
		break;
	}

	return ret;
}

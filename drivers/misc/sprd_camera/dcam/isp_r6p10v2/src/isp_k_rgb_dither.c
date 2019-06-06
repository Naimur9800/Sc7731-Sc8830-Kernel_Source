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

static int isp_k_rgb_dither_random_block
		(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dev_rgb_dither_info rgb_gain;

	memset(&rgb_gain, 0x00, sizeof(rgb_gain));

	ret = copy_from_user((void *)&rgb_gain,
		param->property_param, sizeof(struct isp_dev_rgb_dither_info));
	if (ret != 0) {
		pr_err("isp_k_rgb_dither_random_block: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_RGBG_PARAM0, BIT_0, rgb_gain.random_bypass);
	if (rgb_gain.random_bypass)
		return 0;

	val = ((rgb_gain.seed & 0xFFFFFF) << 8)
		| ((rgb_gain.random_mode & 0x1) << 2);
	ISP_REG_MWR(idx, ISP_RGBG_PARAM0, 0xFFFFFF04, val);

	val = (rgb_gain.r_shift & 0xF)
		| ((rgb_gain.range & 0x7) << 4)
		| ((rgb_gain.r_offset & 0x7FF) << 16);
	ISP_REG_WR(idx, ISP_RGBG_PARAM1, val);

	val = (rgb_gain.takebit[0] & 0xF)
		| ((rgb_gain.takebit[1] & 0xF) << 4)
		| ((rgb_gain.takebit[2] & 0xF) << 8)
		| ((rgb_gain.takebit[3] & 0xF) << 12)
		| ((rgb_gain.takebit[4] & 0xF) << 16)
		| ((rgb_gain.takebit[5] & 0xF) << 20)
		| ((rgb_gain.takebit[6] & 0xF) << 24)
		| ((rgb_gain.takebit[7] & 0xF) << 28);
	ISP_REG_WR(idx, ISP_RGBG_PARAM2, val);

	return ret;
}

static int isp_k_rgb_dither_random_init
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(val));
	if (ret != 0) {
		pr_err("isp_k_rgb_dither_random_init: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_WR(idx, ISP_RGBG_RANDOM_INIT, val);

	return ret;
}

int isp_k_cfg_rgb_dither(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_rgb_gain: param is null error.\n");
		return -1;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_rgb_gain: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_RGB_EDITHER_RANDOM_BLOCK:
		ret = isp_k_rgb_dither_random_block(param, idx);
		break;
	case ISP_PRO_RGB_EDITHER_RANDOM_INIT:
		ret = isp_k_rgb_dither_random_init(param, idx);
		break;
	default:
		pr_err("isp_k_cfg_rgb_gain: fail cmd id:%d, not supported.\n",
			param->property);
		break;
	}

	return ret;
}

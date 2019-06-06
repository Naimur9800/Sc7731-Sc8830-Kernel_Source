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

static int isp_k_rgb2y_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dev_rgb2y_info rgb2y_info;

	memset(&rgb2y_info, 0x00, sizeof(rgb2y_info));

	ret = copy_from_user((void *)&rgb2y_info,
		param->property_param, sizeof(rgb2y_info));
	if (ret != 0) {
		pr_err("isp_k_rgb2y_blockt: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	val = ((rgb2y_info.coef.g & 0x7FF) << 16) | (rgb2y_info.coef.r & 0x7FF);
	ISP_REG_WR(idx, ISP_RGB2Y_COEF_GR, val);

	ISP_REG_MWR(idx, ISP_RGB2Y_COEF_B, 0x7FF, rgb2y_info.coef.b);

	return ret;
}

int isp_k_cfg_rgb2y(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_rgb2y: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_rgb2y: property_param is null error\n");
		return -1;
	}
	switch (param->property) {
	case ISP_PRO_RGB2Y_BLOCK:
		ret = isp_k_rgb2y_block(param, idx);
		break;
	default:
		pr_err("isp_k_cfg_rgb2y: fail cmd id %d, not supported\n",
			param->property);
		break;
	}

	return ret;
}

/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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

static int isp_k_blc_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dev_blc_info blc_info;

	memset(&blc_info, 0x00, sizeof(blc_info));
	ret = copy_from_user((void *)&blc_info, param->property_param,
			sizeof(blc_info));
	if (ret != 0) {
		pr_err("blc_block: copy error, ret=0x%x\n", (unsigned int)ret);
		return -EPERM;
	}

	ISP_REG_MWR(idx, ISP_BLC_PARAM, BIT_0, blc_info.bypass);
	if (blc_info.bypass)
		return 0;

	val = ((blc_info.b & 0x3FF) << 10) | (blc_info.r & 0x3FF);
	ISP_REG_WR(idx, ISP_BLC_B_PARAM_R_B, val);

	val = ((blc_info.gb & 0x3FF) << 10) | (blc_info.gr & 0x3FF);
	ISP_REG_WR(idx, ISP_BLC_B_PARAM_G, val);

	return ret;
}

int isp_k_cfg_blc(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_blc: param is null error.\n");
		return -EPERM;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_blc: property_param is null error.\n");
		return -EPERM;
	}

	switch (param->property) {
	case ISP_PRO_BLC_BLOCK:
		ret = isp_k_blc_block(param, idx);
		break;
	default:
		pr_err("cfg_blc:id:%d, not supported.\n", param->property);
		break;
	}

	return ret;
}

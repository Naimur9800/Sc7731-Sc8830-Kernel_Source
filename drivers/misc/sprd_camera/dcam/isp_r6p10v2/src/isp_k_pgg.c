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

static int isp_k_pgg_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	struct isp_dev_pre_glb_gain_info pgg_info;

	memset(&pgg_info, 0x00, sizeof(pgg_info));

	ret = copy_from_user((void *)&pgg_info,
		param->property_param, sizeof(pgg_info));
	if (ret != 0) {
		pr_err("isp_k_pgg_block: copy error, ret=0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	ISP_REG_MWR(idx, ISP_PGG_PARAM, BIT_0, pgg_info.bypass);
	if (pgg_info.bypass)
		return 0;

	ISP_REG_MWR(idx, ISP_PGG_PARAM, 0xFFFF00, (pgg_info.gain << 8));

	return ret;
}

int isp_k_cfg_pgg(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_pgg: param is null error.\n");
		return -1;
	}

	if (!param->property_param) {
		pr_err("isp_k_cfg_pgg: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_PRE_GLB_GAIN_BLOCK:
		ret = isp_k_pgg_block(param, idx);
		break;
	default:
		pr_err("isp_k_cfg_pgg: fail cmd id:%d, not supported.\n",
			param->property);
		break;
	}

	return ret;
}

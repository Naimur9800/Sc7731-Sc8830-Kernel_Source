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

static int isp_k_csa_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	struct isp_dev_csa_info csa_info;
	unsigned int val = 0;

	memset(&csa_info, 0x00, sizeof(csa_info));

	ret = copy_from_user((void *)&csa_info, param->property_param,
			sizeof(csa_info));
	if (ret != 0) {
		pr_err("copy_from_user error, ret = 0x%x\n", (unsigned int)ret);
		return -EPERM;
	}

	ISP_REG_MWR(idx, ISP_CSA_PARAM, BIT_0, csa_info.bypass);
	ISP_REG_MWR(idx, ISP_CSA_PARAM + ISP_CH1_ADDR_OFFSET,
					BIT_0, csa_info.bypass);
	if (csa_info.bypass)
		return 0;

	val = ((csa_info.factor_v & 0xFF) << 8) |
		((csa_info.factor_u & 0xFF) << 16);
	ISP_REG_MWR(idx, ISP_CSA_PARAM, 0xFFFF00, val);
	ISP_REG_MWR(idx, ISP_CSA_PARAM + ISP_CH1_ADDR_OFFSET, 0xFFFF00, val);

	return ret;
}

int isp_k_cfg_csa(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_csa: param is null error.\n");
		return -EPERM;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_csa: property_param is null error.\n");
		return -EPERM;
	}

	switch (param->property) {
	case ISP_PRO_CSA_BLOCK:
		ret = isp_k_csa_block(param, idx);
		break;
	default:
		pr_err("csa:cmd id:%d, not supported.\n", param->property);
		break;
	}

	return ret;
}

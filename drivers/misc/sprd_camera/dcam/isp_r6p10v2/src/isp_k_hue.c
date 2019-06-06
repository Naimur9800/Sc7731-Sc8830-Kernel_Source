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

static int isp_k_hue_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	struct isp_dev_hue_info hue_info;

	memset(&hue_info, 0x00, sizeof(hue_info));

	ret = copy_from_user((void *)&hue_info,
		param->property_param, sizeof(hue_info));
	if (ret != 0) {
		pr_err("isp_k_hue_block: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	ISP_REG_MWR(idx, ISP_HUE_PARAM, BIT_0, hue_info.bypass);
	ISP_REG_MWR(idx, ISP_HUE_PARAM + ISP_CH1_ADDR_OFFSET, BIT_0,
		hue_info.bypass);
	if (hue_info.bypass)
		return 0;

	ISP_REG_MWR(idx, ISP_HUE_PARAM, 0x1FF0, hue_info.theta << 4);
	ISP_REG_MWR(idx, ISP_HUE_PARAM + ISP_CH1_ADDR_OFFSET, 0x1FF0,
		hue_info.theta << 4);

	return ret;
}

static int isp_k_hue_bypass(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int bypass = 0;

	ret = copy_from_user((void *)&bypass,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_hue_bypass: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (bypass)
		ISP_REG_OWR(idx, ISP_HUE_PARAM, BIT_0);
	else
		ISP_REG_MWR(idx, ISP_HUE_PARAM, BIT_0, 0);

	return ret;
}

static int isp_k_hue_theta(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_hue_theta: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	ISP_REG_MWR(idx, ISP_HUE_PARAM, 0x1FF0, val << 4);

	return ret;
}


int isp_k_cfg_hue(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_hue: param is null error.\n");
		return -1;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_hue: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_HUE_BLOCK:
		ret = isp_k_hue_block(param, idx);
		break;
	case ISP_PRO_HUE_BYPASS:
		ret = isp_k_hue_bypass(param, idx);
		break;
	case ISP_PRO_HUE_THETA:
		ret = isp_k_hue_theta(param, idx);
		break;

	default:
		pr_err("isp_k_cfg_hue: fail cmd id:%d, not supported.\n",
			param->property);
		break;
	}

	return ret;
}

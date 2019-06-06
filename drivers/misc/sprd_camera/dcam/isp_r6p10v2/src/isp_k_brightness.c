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

static int isp_k_brightness_block
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	struct isp_dev_brightness_info brightness_info;

	memset(&brightness_info, 0x00, sizeof(brightness_info));

	ret = copy_from_user((void *)&brightness_info, param->property_param,
			sizeof(brightness_info));
	if (ret != 0) {
		pr_err("brightness: copy error, ret=0x%x\n", (unsigned int)ret);
		return -EPERM;
	}

	ISP_REG_MWR(idx, ISP_BRIGHT_PARAM, BIT_0, brightness_info.bypass);
	ISP_REG_MWR(idx, ISP_BRIGHT_PARAM + ISP_CH1_ADDR_OFFSET,
					BIT_0, brightness_info.bypass);
	if ((brightness_info.bypass))
		return 0;

	ISP_REG_MWR(idx, ISP_BRIGHT_PARAM, 0x1FE,
		((brightness_info.factor & 0xFF) << 1));
	ISP_REG_MWR(idx, ISP_BRIGHT_PARAM + ISP_CH1_ADDR_OFFSET, 0x1FE,
		((brightness_info.factor & 0xFF) << 1));

	return ret;
}

int isp_k_cfg_brightness(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_brightness: param is null error.\n");
		return -EPERM;
	}

	if (param->property_param == NULL) {
		pr_err("brightness: property_param is null error.\n");
		return -EPERM;
	}

	switch (param->property) {
	case ISP_PRO_BRIGHT_BLOCK:
		ret = isp_k_brightness_block(param, idx);
		break;
	default:
		pr_err("isp_k_cfg_brightness: fail cmd id:%d, not supported.\n",
				param->property);
		break;
	}

	return ret;
}

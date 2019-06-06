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
#define pr_fmt(fmt) "DISPATCH: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

static int isp_k_dispatch_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_img_size size;
	struct isp_dev_dispatch_info dispatch_info;

	memset(&dispatch_info, 0x00, sizeof(dispatch_info));
	ret = copy_from_user((void *)&dispatch_info,
		param->property_param, sizeof(struct isp_dev_dispatch_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	ISP_REG_WR(idx, ISP_DISPATCH_CH0_BAYER, dispatch_info.bayer_ch0 & 0x3);
	size = dispatch_info.ch0_size;
	val = ((size.height & 0xFFFF) << 16) | (size.width & 0xFFFF);
	ISP_REG_WR(idx, ISP_DISPATCH_CH0_SIZE, val);

	return ret;
}

static int isp_k_dispatch_ch0_size
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_img_size size = {0};

	ret = copy_from_user((void *)&size,
		param->property_param, sizeof(size));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}
	pr_debug("dispatch_ch0_size W 0x%x, H 0x%x.\n",
		 size.width, size.height);
	val = ((size.height & 0xFFFF) << 16) | (size.width & 0xFFFF);
	ISP_REG_WR(idx, ISP_DISPATCH_CH0_SIZE, val);

	return ret;
}

int isp_k_cfg_dispatch(struct isp_io_param *param, enum isp_id idx)
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
	case ISP_PRO_DISPATCH_BLOCK:
		ret = isp_k_dispatch_block(param, idx);
		break;
	case ISP_PRO_DISPATCH_CH0_SIZE:
		ret = isp_k_dispatch_ch0_size(param, idx);
		break;
	default:
		pr_err("fail to support cmd id = %d\n",
			param->property);
		break;
	}

	return ret;
}


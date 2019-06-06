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

static int isp_k_arbiter_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	struct isp_dev_arbiter_info arbiter_info;
	unsigned int val = 0;

	memset(&arbiter_info, 0x00, sizeof(arbiter_info));

	ret = copy_from_user((void *)&arbiter_info, param->property_param,
			sizeof(arbiter_info));
	if (ret != 0) {
		pr_err("arbiter: copy error, ret = 0x%x\n", (unsigned int)ret);
		return -1;
	}

	val = ((arbiter_info.fetch_raw_word_change & 0x1) << 3) |
			((arbiter_info.fetch_bit_reorder & 0x1) << 2) |
			(arbiter_info.fetch_raw_endian & 0x3);
	ISP_REG_WR(idx, ISP_ARBITER_ENDIAN_CH0, val);

	val = ((arbiter_info.fetch_yuv_word_change & 0x1) << 2) |
			(arbiter_info.fetch_yuv_endian & 0x3);
	ISP_REG_WR(idx, ISP_ARBITER_ENDIAN_CH1, val);

	return ret;
}

static int isp_k_arbiter_wr_status
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int status = 0;

	status = ISP_REG_RD(idx, ISP_ARBITER_WR_STATUS);

	ret = copy_to_user(param->property_param, (void *)&status,
			sizeof(status));
	if (ret != 0) {
		ret = -EPERM;
		pr_err("arbiter: copy error, ret = 0x%x\n", (unsigned int)ret);
	}

	return ret;
}

static int isp_k_arbiter_rd_status
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int status = 0;

	status = ISP_REG_RD(idx, ISP_ARBITER_RD_STATUS);

	ret = copy_to_user(param->property_param, (void *)&status,
			sizeof(status));
	if (ret != 0) {
		ret = -EPERM;
		pr_err("arbiter: copy error, ret = 0x%x\n", (unsigned int)ret);
	}

	return ret;
}

int isp_k_cfg_arbiter(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_arbiter: param is null error.\n");
		return -EPERM;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_arbiter: property_param is null error.\n");
		return -EPERM;
	}

	switch (param->property) {
	case ISP_PRO_ARBITER_BLOCK:
		ret = isp_k_arbiter_block(param, idx);
		break;
	case ISP_PRO_ARBITER_WR_STATUS:
		ret = isp_k_arbiter_wr_status(param, idx);
		break;
	case ISP_PRO_ARBITER_RD_STATUS:
		ret = isp_k_arbiter_rd_status(param, idx);
		break;
	case ISP_PRO_ARBITER_PARAM:
		break;
	case ISP_PRO_ARBITER_ENDIAN_CH0:
		break;
	case ISP_PRO_ARBITER_ENDIAN_CH1:
		break;
	default:
		pr_err("fail cmd id:%d, not supported.\n", param->property);
		break;
	}

	return ret;
}


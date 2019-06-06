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

static int isp_k_post_blc_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dev_post_blc_info post_blc_info;

	memset(&post_blc_info, 0x00, sizeof(post_blc_info));
	ret = copy_from_user((void *)&post_blc_info,
		param->property_param, sizeof(post_blc_info));
	if (ret != 0) {
		pr_err("isp_k_post_blc_block: copy error, ret=0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_POST_BLC_PARA, BIT_0, post_blc_info.bypass);
	if (post_blc_info.bypass)
		return 0;

	val = ((post_blc_info.b_para & 0x3FF) << 10)
		| (post_blc_info.r_para & 0x3FF);
	ISP_REG_MWR(idx, ISP_POST_BLC_B_PARA_R_B, 0xFFFFF, val);

	val = ((post_blc_info.gb_para & 0x3FF) << 10)
		| (post_blc_info.gr_para & 0x3FF);
	ISP_REG_MWR(idx, ISP_POST_BLC_PARA_G, 0xFFFFF, val);


	return ret;

}

int isp_k_cfg_post_blc(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_post_blc: param is null error.\n");
		return -1;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_post_blc: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_POST_BLC_BLOCK:
		ret = isp_k_post_blc_block(param, idx);
		break;
	default:
		pr_err("isp_k_cfg_post_blc: fail cmd id:%d, not supported.\n",
			param->property);
		break;
	}

	return ret;
}

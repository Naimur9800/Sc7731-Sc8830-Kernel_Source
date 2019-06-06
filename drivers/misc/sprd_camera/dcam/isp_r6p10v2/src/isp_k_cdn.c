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

static int isp_k_yuv_cdn_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	struct isp_dev_yuv_cdn_info cdn_info;
	unsigned int val = 0;
	unsigned int i = 0;

	memset(&cdn_info, 0x00, sizeof(cdn_info));

	ret = copy_from_user((void *)&cdn_info, param->property_param,
			sizeof(cdn_info));
	if (ret != 0) {
		pr_err("yuv_cdn: copy error, ret = 0x%x\n", (unsigned int)ret);
		return -EPERM;
	}

	ISP_REG_MWR(idx, ISP_CDN_PARAM, BIT_0, cdn_info.bypass);
	ISP_REG_MWR(idx, ISP_CDN_PARAM + ISP_CH1_ADDR_OFFSET, BIT_0,
		cdn_info.bypass);
	if (cdn_info.bypass)
		return 0;

	if (cdn_info.filter_bypass) {
		ISP_REG_OWR(idx, ISP_CDN_PARAM, BIT_1);
		ISP_REG_OWR(idx, ISP_CDN_PARAM + ISP_CH1_ADDR_OFFSET, BIT_1);
	} else {
		ISP_REG_MWR(idx, ISP_CDN_PARAM, BIT_1, 0);
		ISP_REG_MWR(idx, ISP_CDN_PARAM + ISP_CH1_ADDR_OFFSET, BIT_1, 0);
	}

	if (cdn_info.median_writeback_en) {
		ISP_REG_OWR(idx, ISP_CDN_PARAM, BIT_2);
		ISP_REG_OWR(idx, ISP_CDN_PARAM + ISP_CH1_ADDR_OFFSET, BIT_2);
	} else {
		ISP_REG_MWR(idx, ISP_CDN_PARAM, BIT_2, 0);
		ISP_REG_MWR(idx, ISP_CDN_PARAM + ISP_CH1_ADDR_OFFSET, BIT_2, 0);
	}

	ISP_REG_MWR(idx, ISP_CDN_PARAM, 0x38, cdn_info.median_mode << 3);
	ISP_REG_MWR(idx, ISP_CDN_PARAM + ISP_CH1_ADDR_OFFSET, 0x38,
		cdn_info.median_mode << 3);

	ISP_REG_MWR(idx, ISP_CDN_PARAM, BIT_7 |
			BIT_6, cdn_info.gaussian_mode << 6);
	ISP_REG_MWR(idx, ISP_CDN_PARAM + ISP_CH1_ADDR_OFFSET, BIT_7 |
			BIT_6, cdn_info.gaussian_mode << 6);

	ISP_REG_MWR(idx, ISP_CDN_PARAM, 0x3FFF00, cdn_info.median_thr << 8);
	ISP_REG_MWR(idx, ISP_CDN_PARAM + ISP_CH1_ADDR_OFFSET, 0x3FFF00,
		cdn_info.median_thr << 8);

	val = (cdn_info.median_thru0 & 0x7F) |
		((cdn_info.median_thru1 & 0xFF) << 8) |
		((cdn_info.median_thrv0 & 0x7F) << 16) |
		((cdn_info.median_thrv1 & 0xFF) << 24);
	ISP_REG_WR(idx, ISP_CDN_THRUV, val);
	ISP_REG_WR(idx, ISP_CDN_THRUV + ISP_CH1_ADDR_OFFSET, val);

	for (i = 0; i < 7; i++) {
		val = (cdn_info.rangewu[i*4] & 0x3F) |
			((cdn_info.rangewu[i*4+1] & 0x3F) << 8) |
			((cdn_info.rangewu[i*4+2] & 0x3F) << 16) |
			((cdn_info.rangewu[i*4+3] & 0x3F) << 24);
		ISP_REG_WR(idx, ISP_CDN_U_RANWEI_0 + i * 4, val);
		ISP_REG_WR(idx,
			ISP_CDN_U_RANWEI_0 + ISP_CH1_ADDR_OFFSET + i * 4, val);
	}

	val = (cdn_info.rangewu[28] & 0x3F) |
		((cdn_info.rangewu[29] & 0x3F) << 8) |
		((cdn_info.rangewu[30] & 0x3F) << 16);
	ISP_REG_WR(idx, ISP_CDN_U_RANWEI_7, val);
	ISP_REG_WR(idx, ISP_CDN_U_RANWEI_7 + ISP_CH1_ADDR_OFFSET, val);

	for (i = 0; i < 7; i++) {
		val = (cdn_info.rangewv[i*4] & 0x3F) |
				((cdn_info.rangewv[i*4+1] & 0x3F) << 8) |
				((cdn_info.rangewv[i*4+2] & 0x3F) << 16) |
				((cdn_info.rangewv[i*4+3] & 0x3F) << 24);
		ISP_REG_WR(idx, ISP_CDN_V_RANWEI_0 + i * 4, val);
		ISP_REG_WR(idx,
			ISP_CDN_V_RANWEI_0 + ISP_CH1_ADDR_OFFSET + i * 4, val);
	}

	val = (cdn_info.rangewv[28] & 0x3F) |
		((cdn_info.rangewv[29] & 0x3F) << 8) |
		((cdn_info.rangewv[30] & 0x3F) << 16);
	ISP_REG_WR(idx, ISP_CDN_V_RANWEI_7, val);
	ISP_REG_WR(idx, ISP_CDN_V_RANWEI_7 + ISP_CH1_ADDR_OFFSET, val);

	return ret;
}

int isp_k_cfg_yuv_cdn(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_yuv_cdn: param is null error.\n");
		return -EPERM;
	}

	if (param->property_param == NULL) {
		pr_err("yuv_cdn: property_param is null error.\n");
		return -EPERM;
	}

	switch (param->property) {
	case ISP_PRO_YUV_CDN_BLOCK:
		ret = isp_k_yuv_cdn_block(param, idx);
		break;
	default:
		pr_err("isp_k_cfg_yuv_cdn: fail cmd id:%d, not supported.\n",
				param->property);
		break;
	}

	return ret;
}

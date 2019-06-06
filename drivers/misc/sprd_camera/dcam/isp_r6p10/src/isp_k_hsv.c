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
#define pr_fmt(fmt) "HSV: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

#define	ISP_FRGB_HSV_BUF0				 0
#define	ISP_FRGB_HSV_BUF1				 1
#define ISP_HSV_ITEM                       360

static int isp_pingpang_frgb_hsv(struct isp_dev_hsv_info *hsv_info,
			struct isp_k_block *isp_k_param, enum isp_id idx)
{
	int ret = 0;
	unsigned int i;
	unsigned int val = 0;
	unsigned long dst_addr = 0;
	struct isp_hsv_region_info region_info;
	void *data_ptr = NULL;
	int32_t buf_size = 0;

	memset(&region_info, 0x00, sizeof(region_info));
	if (!hsv_info)
		return -1;

	if (isp_k_param->hsv_buf_id == ISP_FRGB_HSV_BUF1) {
		dst_addr = ISP_BASE_ADDR(idx) + ISP_HSV_BUF0_CH0;
		isp_k_param->hsv_buf_id = ISP_FRGB_HSV_BUF0;
	} else {
		dst_addr = ISP_BASE_ADDR(idx) + ISP_HSV_BUF1_CH0;
		isp_k_param->hsv_buf_id = ISP_FRGB_HSV_BUF1;
	}

	ISP_REG_MWR(idx, ISP_HSV_PARAM, BIT_1, isp_k_param->hsv_buf_id << 1);
	region_info = hsv_info->region_info[isp_k_param->hsv_buf_id];

	for (i = 0; i < 5; i++) {
		val = ((region_info.hrange_left[i] & 0x1FF) << 23) |
			  ((region_info.s_curve[i][1] & 0x7FF) << 11) |
			   (region_info.s_curve[i][0] & 0x7FF);
		ISP_REG_WR(idx, ISP_HSV_CFG0 + i*12, val);

		val = ((region_info.hrange_right[i] & 0x1FF) << 23) |
			  ((region_info.s_curve[i][3] & 0x7FF) << 11) |
			   (region_info.s_curve[i][2] & 0x7FF);
		ISP_REG_WR(idx, ISP_HSV_CFG1 + i*12, val);

		val = ((region_info.v_curve[i][3] & 0xFF) << 24) |
			  ((region_info.v_curve[i][2] & 0xFF) << 16) |
			  ((region_info.v_curve[i][1] & 0xFF) << 8) |
			   (region_info.v_curve[i][0] & 0xFF);
		ISP_REG_WR(idx, ISP_HSV_CFG2 + i*12, val);
	}
#ifdef CONFIG_64BIT
	data_ptr = (void *)(((unsigned long)hsv_info->data_ptr[1] << 32)
				| hsv_info->data_ptr[0]);
#else
	data_ptr = (void *)(hsv_info->data_ptr[0]);
#endif
	buf_size = ISP_HSV_ITEM * 4;
	if (data_ptr == NULL || hsv_info->size > buf_size) {
		pr_err("fail to get param or buf_size: data_ptr == %p, buf_size = %d\n",
			data_ptr, hsv_info->size);
		return -1;
	}
	ret = copy_from_user((void *)dst_addr, data_ptr, hsv_info->size);
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	return ret;
}

static int isp_k_hsv_block(struct isp_io_param *param,
			struct isp_k_block *isp_k_param, enum isp_id idx)
{
	int ret = 0;
	struct isp_dev_hsv_info hsv_info;

	memset(&hsv_info, 0x00, sizeof(hsv_info));
	ret = copy_from_user((void *)&hsv_info,
		param->property_param, sizeof(struct isp_dev_hsv_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}
	ISP_REG_MWR(idx, ISP_HSV_PARAM, BIT_0, hsv_info.bypass);
	if (hsv_info.bypass)
		return 0;
	isp_pingpang_frgb_hsv(&hsv_info, isp_k_param, idx);

	return ret;
}

int isp_k_cfg_hsv(struct isp_io_param *param,
		struct isp_k_block *isp_k_param, enum isp_id idx)
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
	case ISP_PRO_HSV_BLOCK:
		ret = isp_k_hsv_block(param, isp_k_param, idx);
		break;
	default:
		pr_err("fail to support cmd id = %d\n",
			param->property);
		break;
	}

	return ret;
}

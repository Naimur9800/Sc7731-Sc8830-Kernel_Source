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
#include "../isp_block.h"

#define	ISP_FRGB_HSV_BUF0				 0
#define	ISP_FRGB_HSV_BUF1				 1
#define ISP_HSV_ITEM                       360

static int isp_load_hsvbuf(struct isp_dev_hsv_info *hsv_info, enum isp_id idx)
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

	val = (ISP_REG_RD(idx, ISP_HSV_PARAM) & BIT_1) >> 1;
	if (val == ISP_FRGB_HSV_BUF1) {
		dst_addr = ISP_BASE_ADDR(idx) + ISP_HSV_BUF0_CH0;
		hsv_info->buf_sel = ISP_FRGB_HSV_BUF0;
	} else {
		dst_addr = ISP_BASE_ADDR(idx) + ISP_HSV_BUF1_CH0;
		hsv_info->buf_sel = ISP_FRGB_HSV_BUF1;
	}

	ISP_REG_MWR(idx, ISP_HSV_PARAM, BIT_1, hsv_info->buf_sel << 1);
	region_info = hsv_info->region_info[hsv_info->buf_sel];

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
		pr_err("isp_k_hsv_block : memory error %p %x\n",
			data_ptr, hsv_info->size);
		return -1;
	}
	ret = copy_from_user((void *)dst_addr, data_ptr, hsv_info->size);
	if (ret != 0) {
		pr_err("isp_k_hsv_block: copy error 0x%x\n", (unsigned int)ret);
		return -1;
	}

	return ret;
}

static int isp_k_hsv_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	struct isp_dev_hsv_info hsv_info;

	memset(&hsv_info, 0x00, sizeof(hsv_info));
	ret = copy_from_user((void *)&hsv_info,
		param->property_param, sizeof(struct isp_dev_hsv_info));
	if (ret != 0) {
		pr_err("isp_k_hsv_block: copy error, ret=0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	ISP_REG_MWR(idx, ISP_HSV_PARAM, BIT_0, hsv_info.bypass);
	if (hsv_info.bypass)
		return 0;
	isp_load_hsvbuf(&hsv_info, idx);

	return ret;
}

static int isp_k_hsv_bypass(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int bypass = 0;

	ret = copy_from_user((void *)&bypass,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_hsv_bypass: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (bypass)
		ISP_REG_OWR(idx, ISP_HSV_PARAM, BIT_0);
	else
		ISP_REG_MWR(idx, ISP_HSV_PARAM, BIT_0, 0);

	return ret;
}

static int isp_k_hsv_buf_sel(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int buf_sel = 0;

	ret = copy_from_user((void *)&buf_sel,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_hsv_buf_sel: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (buf_sel)
		ISP_REG_OWR(idx, ISP_HSV_PARAM, BIT_1);
	else
		ISP_REG_MWR(idx, ISP_HSV_PARAM, BIT_1, 0);

	return ret;
}

static int isp_k_hsv_curve(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct hsv_curve h_c;
	unsigned int i = 0;

	ret = copy_from_user((void *)&h_c,
		param->property_param, sizeof(struct hsv_curve));
	if (ret != 0) {
		pr_err("isp_k_hsv_curve: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	for (i = 0; i < 5; i++) {
		val = ((h_c.h[i][0] & 0x1FF) << 23)
			| ((h_c.s[i][1] & 0x7FF) << 11)
			| (h_c.s[i][0] & 0x7FF);
		ISP_REG_WR(idx, ISP_HSV_CFG0 + i * 12, val);
		val = ((h_c.h[i][1] & 0x1FF) << 23)
			| ((h_c.s[i][3] & 0x7FF) << 11)
			| (h_c.s[i][2] & 0x7FF);
		ISP_REG_WR(idx, ISP_HSV_CFG1 + i * 12, val);
		val = (h_c.v[i][0] & 0xFF)
			| ((h_c.v[i][1] & 0xFF) << 8)
			| ((h_c.v[i][2] & 0xFF) << 16)
			| ((h_c.v[i][3] & 0xFF) << 24);
		ISP_REG_WR(idx, ISP_HSV_CFG2 + i * 12, val);
	}

	return ret;
}

int isp_k_cfg_hsv(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_hsv: param is null error.\n");
		return -1;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_hsv: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_HSV_BLOCK:
		ret = isp_k_hsv_block(param, idx);
		break;
	case ISP_PRO_HSV_BYPASS:
		ret = isp_k_hsv_bypass(param, idx);
		break;
	case ISP_PRO_HSV_BUF_SEL:
		ret = isp_k_hsv_buf_sel(param, idx);
		break;
	case ISP_PRO_HSV_CURVE:
		ret = isp_k_hsv_curve(param, idx);
		break;
	default:
		pr_err("isp_k_cfg_hsv: fail cmd id:%d, not supported.\n",
			param->property);
		break;
	}

	return ret;
}

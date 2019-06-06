/* Copyright (C) 2016 Spreadtrum Communications Inc.
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

static int isp_k_dispatch_yuv_block
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	struct isp_dev_dispatch_yuv_info dispatch_yuv_info;
	unsigned int val = 0;

	memset(&dispatch_yuv_info, 0x00, sizeof(dispatch_yuv_info));
	ret = copy_from_user((void *)&dispatch_yuv_info,
		param->property_param, sizeof(dispatch_yuv_info));
	if (ret != 0) {
		pr_err("isp_k_dispatch_yuv_block: copy_from_user error, ret = 0x%x\n",
				(unsigned int)ret);
		return -1;
	}

	pr_info("dispatch_yuv_info CH0 W 0x%x, H 0x%x",
		dispatch_yuv_info.ch0_size.width,
		dispatch_yuv_info.ch0_size.height);
	val = ((dispatch_yuv_info.ch0_size.height & 0xFFFF) << 16)
			| (dispatch_yuv_info.ch0_size.width & 0xFFFF);
	ISP_REG_WR(idx, ISP_DISPATCH_YUV_CH0_SIZE, val);

	val = ((dispatch_yuv_info.ch1_size.height & 0xFFFF) << 16)
			| (dispatch_yuv_info.ch1_size.width & 0xFFFF);
	ISP_REG_WR(idx, ISP_DISPATCH_YUV_CH1_SIZE, val);

	return ret;
}

static int isp_k_dispatch_yuv_ch0_bayer
		(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_dispatch_yuv_ch0_bayer: copy_from_user error, ret = 0x%x\n",
				(unsigned int)ret);
		return -1;
	}
	ISP_REG_WR(idx, ISP_DISPATCH_YUV_CH0_BAYER, val & 0x3);

	return ret;
}

static int isp_k_dispatch_yuv_ch1_bayer
		(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_dispatch_yuv_ch1_bayer: copy_from_user error, ret = 0x%x\n",
				(unsigned int)ret);
		return -1;
	}
	ISP_REG_WR(idx, ISP_DISPATCH_YUV_CH1_BAYER, val & 0x3);

	return ret;
}

static int isp_k_dispatch_yuv_dly
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dispatch_yuv_dly_info isp_dispatch_yuv_dly;

	memset(&isp_dispatch_yuv_dly, 0x00,
		sizeof(isp_dispatch_yuv_dly));
	ret = copy_from_user((void *)&isp_dispatch_yuv_dly,
		param->property_param, sizeof(isp_dispatch_yuv_dly));
	if (ret != 0) {
		pr_err("isp_k_dispatch_yuv_dly: copy_from_user error, ret = 0x%x\n",
		(unsigned int)ret);
		return -1;
	}
	val = ((isp_dispatch_yuv_dly.height_dly_num_ch0 & 0xFF) << 16)
			| (isp_dispatch_yuv_dly.width_dly_num_ch0 & 0x1FFF);
	ISP_REG_WR(idx, ISP_DISPATCH_YUV_DLY_CH0, val);

	return ret;
}

static int isp_k_dispatch_yuv_dly1
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dispatch_yuv_dly_info isp_dispatch_yuv_dly1;

	memset(&isp_dispatch_yuv_dly1, 0x00, sizeof(isp_dispatch_yuv_dly1));
	ret = copy_from_user((void *)&isp_dispatch_yuv_dly1,
		param->property_param, sizeof(isp_dispatch_yuv_dly1));
	if (ret != 0) {
		pr_err("isp_k_dispatch_yuv_dly: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	val	= (((isp_dispatch_yuv_dly1.height_dly_num_ch0 & 0xFF) << 16)
			| (isp_dispatch_yuv_dly1.width_dly_num_ch0 & 0x1FFF));
	ISP_REG_WR(idx, ISP_DISPATCH_YUV_DLY_CH1, val);

	return ret;
}

static int isp_k_dispatch_yuv_hw_ctrl_ch0
		(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	struct isp_dispatch_yuv_hw_ctrl_info isp_dispatch_yuv_hw_ctrl_ch0;

	memset(&isp_dispatch_yuv_hw_ctrl_ch0, 0x00,
		sizeof(isp_dispatch_yuv_hw_ctrl_ch0));
	ret = copy_from_user((void *)&isp_dispatch_yuv_hw_ctrl_ch0,
		param->property_param, sizeof(isp_dispatch_yuv_hw_ctrl_ch0));
	if (ret != 0) {
		pr_err("isp_k_dispatch_yuv_hw_ctrl_ch0: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	ISP_REG_MWR(idx, ISP_DISPATCH_YUV_HW_CTRL_CH0, BIT_15,
		isp_dispatch_yuv_hw_ctrl_ch0.nready_cfg_ch0 << 15);
	ISP_REG_MWR(idx, ISP_DISPATCH_YUV_HW_CTRL_CH0, 0x1FFF,
		isp_dispatch_yuv_hw_ctrl_ch0.nready_width_ch0 & 0x1FFF);

	return ret;
}

static int isp_k_dispatch_yuv_hw_ctrl_ch1
		(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	struct isp_dispatch_yuv_hw_ctrl_info isp_dispatch_yuv_hw_ctrl_ch1;

	memset(&isp_dispatch_yuv_hw_ctrl_ch1, 0x00,
		sizeof(isp_dispatch_yuv_hw_ctrl_ch1));
	ret = copy_from_user((void *)&isp_dispatch_yuv_hw_ctrl_ch1,
		param->property_param, sizeof(isp_dispatch_yuv_hw_ctrl_ch1));
	if (ret != 0) {
		pr_err("isp_k_dispatch_yuv_hw_ctrl_ch0: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	ISP_REG_MWR(idx, ISP_DISPATCH_YUV_HW_CTRL_CH1, BIT_15,
		isp_dispatch_yuv_hw_ctrl_ch1.nready_cfg_ch0 << 15);
	ISP_REG_MWR(idx, ISP_DISPATCH_YUV_HW_CTRL_CH1, 0x1FFF,
		isp_dispatch_yuv_hw_ctrl_ch1.nready_width_ch0 & 0x1FFF);

	return ret;
}

static int isp_k_dispatch_yuv_ch0_size
		(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dispatch_yuv_size_info isp_dispatch_yuv_size_ch0;

	memset(&isp_dispatch_yuv_size_ch0, 0x00,
		sizeof(isp_dispatch_yuv_size_ch0));
	ret = copy_from_user((void *)&isp_dispatch_yuv_size_ch0,
		param->property_param, sizeof(isp_dispatch_yuv_size_ch0));
	if (ret != 0) {
		pr_err("isp_k_dispatch_yuv_ch0_size: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	val	= ((isp_dispatch_yuv_size_ch0.height & 0xFFFF) << 16)
			| (isp_dispatch_yuv_size_ch0.width & 0xFFFF);
	ISP_REG_WR(idx, ISP_DISPATCH_YUV_CH0_SIZE, val);

	return ret;
}

static int isp_k_dispatch_yuv_ch1_size
		(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dispatch_yuv_size_info isp_dispatch_yuv_size_ch1;

	memset(&isp_dispatch_yuv_size_ch1, 0x00,
		sizeof(isp_dispatch_yuv_size_ch1));
	ret = copy_from_user((void *)&isp_dispatch_yuv_size_ch1,
		param->property_param, sizeof(isp_dispatch_yuv_size_ch1));
	if (ret != 0) {
		pr_err("isp_k_dispatch_yuv_ch0_size: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	val	= ((isp_dispatch_yuv_size_ch1.height & 0xFFFF) << 16)
			| (isp_dispatch_yuv_size_ch1.width & 0xFFFF);
	ISP_REG_WR(idx, ISP_DISPATCH_YUV_CH1_SIZE, val);

	return ret;
}

static int isp_k_dispatch_yuv_cap_mode
		(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user(&val, param->property_param, sizeof(val));
	if (ret != 0) {
		pr_err("isp_k_dispatch_yuv_cap_mode: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	ISP_REG_WR(idx, ISP_DISPATCH_YUV_CAP_MODE, val);

	return ret;
}

static int isp_k_dispatch_yuv_cap_ctrl1
		(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dispatch_yuv_cap_ctrl_info isp_dispatch_yuv_cap_ctrl1;

	memset(&isp_dispatch_yuv_cap_ctrl1, 0x00,
		sizeof(isp_dispatch_yuv_cap_ctrl1));
	ret = copy_from_user((void *)&isp_dispatch_yuv_cap_ctrl1,
		param->property_param, sizeof(isp_dispatch_yuv_cap_ctrl1));
	if (ret != 0) {
		pr_err("isp_k_dispatch_yuv_cap_ctrl1: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	val	= ((isp_dispatch_yuv_cap_ctrl1.cap_line_scl_dly_cycle_ctrl
		& 0xFFFF) << 16)
		| (isp_dispatch_yuv_cap_ctrl1.pre_line_scl_dly_cycle_ctrl
		& 0xFFFF);
	ISP_REG_WR(idx, ISP_DISPATCH_YUV_CAP_CTRL_1, val);

	return ret;
}


static int isp_k_dispatch_yuv_cap_ctrl2
		(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dispatch_yuv_cap_ctrl2_info isp_dispatch_yuv_cap_ctrl2;

	memset(&isp_dispatch_yuv_cap_ctrl2, 0x00,
		sizeof(isp_dispatch_yuv_cap_ctrl2));
	ret = copy_from_user((void *)&isp_dispatch_yuv_cap_ctrl2,
		param->property_param, sizeof(isp_dispatch_yuv_cap_ctrl2));
	if (ret != 0) {
		pr_err("isp_k_dispatch_yuv_cap_ctrl1: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	val	= ((isp_dispatch_yuv_cap_ctrl2.pre_line_hole_pix_num & 0x1FFF)
		<< 16)
		| (isp_dispatch_yuv_cap_ctrl2.pre_line_whole_cyle & 0xFFFF);
	ISP_REG_WR(idx, ISP_DISPATCH_YUV_CAP_CTRL_2, val);

	return ret;
}

static int isp_k_dispatch_yuv_cap_ctrl3
		(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dispatch_yuv_cap_ctrl_info isp_dispatch_yuv_cap_ctrl3;

	memset(&isp_dispatch_yuv_cap_ctrl3, 0x00,
		sizeof(isp_dispatch_yuv_cap_ctrl3));
	ret = copy_from_user((void *)&isp_dispatch_yuv_cap_ctrl3,
		param->property_param, sizeof(isp_dispatch_yuv_cap_ctrl3));
	if (ret != 0) {
		pr_err("isp_k_dispatch_yuv_cap_ctrl1: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	val	= (((isp_dispatch_yuv_cap_ctrl3.cap_line_scl_dly_cycle_ctrl
		& 0xFFFF) << 16)
		| (isp_dispatch_yuv_cap_ctrl3.pre_line_scl_dly_cycle_ctrl
		& 0xFFFF));
	ISP_REG_WR(idx, ISP_DISPATCH_YUV_CAP_CTRL_3, val);

	return ret;
}

static int isp_k_dispatch_yuv_dly_num_flash_ch0
			(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val, param->property_param, sizeof(val));
	if (ret != 0) {
		pr_err("isp_k_dispatch_yuv_dly_num_flash_ch0: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	ISP_REG_WR(idx, ISP_DISPATCH_YUV_DLY_NUM_FLASH_CH0, (val & 0x1FFF));

	return ret;
}

static int isp_k_dispatch_yuv_dly_num_flash_ch1
			(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val, param->property_param, sizeof(val));
	if (ret != 0) {
		pr_err("isp_k_dispatch_yuv_dly_num_flash_ch1: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	ISP_REG_WR(idx, ISP_DISPATCH_YUV_DLY_NUM_FLASH_CH1, (val & 0x1FFF));

	return ret;
}


int isp_k_cfg_dispatch_yuv(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_dispatch_yuv: param is null error.\n");
		return -1;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_dispatch_yuv: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_DISPATCH_YUV_BLOCK:
		ret = isp_k_dispatch_yuv_block(param, idx);
		break;
	case ISP_PRO_DISPATCH_YUV_CH0_BAYER:
		ret = isp_k_dispatch_yuv_ch0_bayer(param, idx);
		break;
	case ISP_PRO_DISPATCH_YUV_CH1_BAYER:
		ret = isp_k_dispatch_yuv_ch1_bayer(param, idx);
		break;
	case ISP_PRO_DISPATCH_YUV_DLY:
		ret = isp_k_dispatch_yuv_dly(param, idx);
		break;
	case ISP_PRO_DISPATCH_YUV_DLY1:
		ret = isp_k_dispatch_yuv_dly1(param, idx);
		break;
	case ISP_PRO_DISPATCH_YUV_HW_CTRL_CH0:
		ret = isp_k_dispatch_yuv_hw_ctrl_ch0(param, idx);
		break;
	case ISP_PRO_DISPATCH_YUV_HW_CTRL_CH1:
		ret = isp_k_dispatch_yuv_hw_ctrl_ch1(param, idx);
		break;
	case ISP_PRO_DISPATCH_YUV_CH0_SIZE:
		ret = isp_k_dispatch_yuv_ch0_size(param, idx);
		break;
	case ISP_PRO_DISPATCH_YUV_CH1_SIZE:
		ret = isp_k_dispatch_yuv_ch1_size(param, idx);
		break;
	case ISP_PRO_DISPATCH_YUV_CAP_MODE:
		ret = isp_k_dispatch_yuv_cap_mode(param, idx);
		break;
	case ISP_PRO_DISPATCH_YUV_CAP_CTRL_1:
		ret = isp_k_dispatch_yuv_cap_ctrl1(param, idx);
		break;
	case ISP_PRO_DISPATCH_YUV_CAP_CTRL_2:
		ret = isp_k_dispatch_yuv_cap_ctrl2(param, idx);
		break;
	case ISP_PRO_DISPATCH_YUV_CAP_CTRL_3:
		ret = isp_k_dispatch_yuv_cap_ctrl3(param, idx);
		break;
	case ISP_PRO_DISPATCH_YUV_DLY_NUM_FLASH_CH0:
		ret = isp_k_dispatch_yuv_dly_num_flash_ch0(param, idx);
		break;
	case ISP_PRO_DISPATCH_YUV_DLY_NUM_FLASH_CH1:
		ret = isp_k_dispatch_yuv_dly_num_flash_ch1(param, idx);
		break;

	default:
		pr_err("s: fail cmd id:%d, not supported.\n", param->property);
		break;
	}

	return ret;
}



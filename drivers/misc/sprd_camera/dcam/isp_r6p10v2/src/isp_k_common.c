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

static int isp_k_common_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val;
	struct isp_dev_common_info common_info;

	memset(&common_info, 0x00, sizeof(common_info));

	ret = copy_from_user((void *)&common_info, param->property_param,
			sizeof(common_info));
	if (ret != 0) {
		pr_err("common_block: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -EPERM;
	}

	ISP_REG_MWR(idx, ISP_COMMON_CTRL_CH0,
			3, common_info.fetch_sel_0);

	ISP_REG_MWR(idx, ISP_COMMON_CTRL_CH0,
			3 << 16, common_info.store_sel_0  << 16);

	ISP_REG_MWR(idx, ISP_COMMON_CTRL_CH1,
			3, common_info.fetch_sel_1);

	ISP_REG_MWR(idx, ISP_COMMON_CTRL_CH1,
			3 << 16, common_info.store_sel_1  << 16);

	ISP_REG_MWR(idx, ISP_COMMON_SPACE_SEL,
			3, common_info.fetch_color_space_sel);

	ISP_REG_MWR(idx, ISP_COMMON_SPACE_SEL, 3 << 2,
		common_info.store_color_space_sel << 2);

	ISP_REG_MWR(idx, ISP_COMMON_SPACE_SEL,
			BIT_4, common_info.ch0_path_ctrl << 4);

	ISP_REG_MWR(idx, ISP_COMMON_CTRL_BIN,
			BIT_0, common_info.bin_pos_sel);

	ISP_REG_MWR(idx, ISP_COMMON_PMU_RAM_MASK,
			BIT_0, common_info.ram_mask);

	ISP_REG_WR(idx, ISP_COMMON_GCLK_CTRL_RRGB,
				common_info.gclk_ctrl_rrgb);
	ISP_REG_WR(idx, ISP_COMMON_GCLK_CTRL_FRGB_YIQ,
				common_info.gclk_ctrl_yiq_frgb);
	ISP_REG_WR(idx, ISP_COMMON_GCLK_CTRL_YUV,
				common_info.gclk_ctrl_yuv);

	ISP_REG_MWR(idx, ISP_COMMON_GCLK_CTRL_SCL_3DNR, 0xFFFF,
		common_info.gclk_ctrl_scaler_3dnr);

	if (idx == ISP_ID_0) {
		ISP_REG_MWR(idx, ISP_COMMON_LBUF_OFFSET0, 0xFFFF,
			common_info.lbuf_off.comm_lbuf_offset);
		ISP_REG_MWR(idx, ISP_COMMON_LBUF_OFFSET1, 0xFFFF0000,
			common_info.lbuf_off.ydly_lbuf_offset << 16);
	}

	pr_info("isp_k_common_block:idx=%d, comm_lbuf_offset=0x%x, ydly_lbuf_offset=0x%x\n",
	       idx,
	       common_info.lbuf_off.comm_lbuf_offset,
	       (common_info.lbuf_off.ydly_lbuf_offset << 16));

	ISP_REG_MWR(idx, ISP_COMMON_YUV_PATH_SEL_CH1, 0x3,
		common_info.yuv_disp_path_sel_ch1 & 0x3);

	/* JPG TO DO */
	val = ((common_info.isp_cfg_sof_rst & 0x1) << 1) |
			(common_info.isp_soft_rst & 0x1);
	ISP_REG_WR(idx, ISP_COMMON_SOFT_RST_CTRL, val);

	ISP_REG_MWR(idx, ISP_COMMON_SHADOW_CTRL_CH0, BIT_16,
		common_info.shadow_ctrl_ch0.shadow_mctrl << 16);
	ISP_REG_MWR(idx, ISP_COMMON_SHADOW_CTRL_CH1, BIT_16,
		common_info.shadow_ctrl_ch1.shadow_mctrl << 16);

	ISP_REG_MWR(idx, ISP_COMMON_SHADOW_CTRL_CH0, 1 << 21,
		common_info.shadow_ctrl_ch0.comm_cfg_rdy << 21);
	ISP_REG_MWR(idx, ISP_COMMON_SHADOW_CTRL_CH1, 1 << 21,
		common_info.shadow_ctrl_ch1.comm_cfg_rdy << 21);

	return ret;
}

static int isp_k_common_version(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int version = 0;

	version = ISP_REG_RD(idx, ISP_COMMON_VERSION);

	ret = copy_to_user(param->property_param, (void *)&version,
			sizeof(version));
	if (ret != 0) {
		ret = -EPERM;
		pr_err("common: copy error, ret = 0x%x\n", (unsigned int)ret);
	}
	ISP_REG_MWR(idx, ISP_COMMON_SHADOW_CTRL_CH1, 1 << 21, 1 << 21);

	return ret;
}

static int isp_k_common_status0(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int status = 0;

	status = ISP_REG_RD(idx, ISP_COMMON_STATUS0);

	ret = copy_to_user(param->property_param, (void *)&status,
			sizeof(status));
	if (ret != 0) {
		ret = -EPERM;
		pr_err("common: copy error, ret = 0x%x\n", (unsigned int)ret);
	}

	return ret;
}

static int isp_k_common_status1(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int status = 0;

	status = ISP_REG_RD(idx, ISP_COMMON_STATUS1);

	ret = copy_to_user(param->property_param, (void *)&status,
			sizeof(status));
	if (ret != 0) {
		ret = -EPERM;
		pr_err("common: copy error, ret = 0x%x\n", (unsigned int)ret);
	}

	return ret;
}

static int isp_k_common_channel0_fetch_sel
		(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int fetch_sel = 0;

	ret = copy_from_user((void *)&fetch_sel, param->property_param,
			sizeof(fetch_sel));
	if (ret != 0) {
		pr_err("common:copy error, ret = 0x%x\n", (unsigned int)ret);
		return -EPERM;
	}

	ISP_REG_MWR(idx, ISP_COMMON_CTRL_CH0, 3, fetch_sel);
	ISP_REG_MWR(idx, ISP_COMMON_SHADOW_CTRL_CH1, 1 << 21, 1 << 21);

	return ret;
}

static int isp_k_common_channel0_store_sel
		(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int store_sel = 0;

	ret = copy_from_user((void *)&store_sel, param->property_param,
		sizeof(store_sel));
	if (ret != 0) {
		pr_err("copy_from_user error, ret = 0x%x\n", (unsigned int)ret);
		return -EPERM;
	}

	ISP_REG_MWR(idx, ISP_COMMON_CTRL_CH0, 3 << 16, store_sel  << 16);
	ISP_REG_MWR(idx, ISP_COMMON_SHADOW_CTRL_CH1, 1 << 21, 1 << 21);

	return ret;
}

static int isp_k_common_channel1_fetch_sel
		(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int fetch_sel = 0;

	ret = copy_from_user((void *)&fetch_sel, param->property_param,
			sizeof(fetch_sel));
	if (ret != 0) {
		pr_err("copy_from_user error, ret = 0x%x\n", (unsigned int)ret);
		return -EPERM;
	}

	ISP_REG_MWR(idx, ISP_COMMON_CTRL_CH1, 3, fetch_sel);
	ISP_REG_MWR(idx, ISP_COMMON_SHADOW_CTRL_CH1, 1 << 21, 1 << 21);

	return ret;
}

static int isp_k_common_channel1_store_sel
		(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int store_sel = 0;

	ret = copy_from_user((void *)&store_sel, param->property_param,
			sizeof(store_sel));
	if (ret != 0) {
		pr_err("copy_from_user error, ret = 0x%x\n", (unsigned int)ret);
		return -EPERM;
	}

	ISP_REG_MWR(idx, ISP_COMMON_CTRL_CH1, 3 << 16, store_sel  << 16);
	ISP_REG_MWR(idx, ISP_COMMON_SHADOW_CTRL_CH1, 1 << 21, 1 << 21);

	return ret;
}

static int isp_k_common_fetch_color_space_sel
		(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int color_space = 0;

	ret = copy_from_user((void *)&color_space, param->property_param,
			sizeof(color_space));
	if (ret != 0) {
		pr_err("copy_from_user error, ret = 0x%x\n", (unsigned int)ret);
		return -EPERM;
	}

	ISP_REG_MWR(idx, ISP_COMMON_SPACE_SEL, 3, color_space);
	ISP_REG_MWR(idx, ISP_COMMON_SHADOW_CTRL_CH1, 1 << 21, 1 << 21);

	return ret;
}

static int isp_k_common_store_color_space_sel
		(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int color_space = 0;

	ret = copy_from_user((void *)&color_space, param->property_param,
			sizeof(color_space));
	if (ret != 0) {
		pr_err("copy_from_user error, ret = 0x%x\n", (unsigned int)ret);
		return -EPERM;
	}

	ISP_REG_MWR(idx, ISP_COMMON_SPACE_SEL, 3 << 2, color_space << 2);
	ISP_REG_MWR(idx, ISP_COMMON_SHADOW_CTRL_CH1, 1 << 21, 1 << 21);

	return ret;
}

static int isp_k_3a_shadow_ctrl(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int enable = 0;

	ret = copy_from_user((void *)&enable, param->property_param,
			sizeof(enable));
	if (ret != 0) {
		pr_err("copy_from_user error, ret = 0x%x\n", (unsigned int)ret);
		return -EPERM;
	}

	ISP_REG_MWR(idx, ISP_COMMON_RESERVED0, 1 << 2, enable  << 2);
	ISP_REG_MWR(idx, ISP_COMMON_SHADOW_CTRL_CH1, 1 << 21, 1 << 21);

	return ret;
}

static int isp_k_common_shadow_ctrl(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int enable = 0;

	ret = copy_from_user((void *)&enable, param->property_param,
			sizeof(enable));
	if (ret != 0) {
		pr_err("copy_from_user error, ret = 0x%x\n", (unsigned int)ret);
		return -EPERM;
	}

	ISP_REG_MWR(idx, ISP_COMMON_SHADOW_CTRL_CH0, BIT_16, enable << 16);
	ISP_REG_MWR(idx, ISP_COMMON_SHADOW_CTRL_CH1, BIT_16, enable << 16);
	ISP_REG_MWR(idx, ISP_COMMON_SHADOW_CTRL_CH0, 1 << 21, 1 << 21);
	ISP_REG_MWR(idx, ISP_COMMON_SHADOW_CTRL_CH1, 1 << 21, 1 << 21);

	return ret;
}

int isp_k_cfg_common(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_common: param is null error.\n");
		return -EPERM;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_common: property_param is null error.\n");
		return -EPERM;
	}

	switch (param->property) {
	case ISP_PRO_COMMON_BLOCK:
		ret = isp_k_common_block(param, idx);
		break;
	case ISP_PRO_COMMON_VERSION:
		ret = isp_k_common_version(param, idx);
		break;
	case ISP_PRO_COMMON_STATUS0:
		ret = isp_k_common_status0(param, idx);
		break;
	case ISP_PRO_COMMON_STATUS1:
		ret = isp_k_common_status1(param, idx);
		break;
	case ISP_PRO_COMMON_CH0_FETCH_SEL:
		ret = isp_k_common_channel0_fetch_sel(param, idx);
		break;
	case ISP_PRO_COMMON_CH0_SIZER_SEL:
		break;
	case ISP_PRO_COMMON_CH0_STORE_SEL:
		ret = isp_k_common_channel0_store_sel(param, idx);
		break;
	case ISP_PRO_COMMON_CH1_FETCH_SEL:
		ret = isp_k_common_channel1_fetch_sel(param, idx);
		break;
	case ISP_PRO_COMMON_CH1_SIZER_SEL:
		break;
	case ISP_PRO_COMMON_CH1_STORE_SEL:
		ret = isp_k_common_channel1_store_sel(param, idx);
		break;
	case ISP_PRO_COMMON_FETCH_COLOR_SPACE_SEL:
		ret = isp_k_common_fetch_color_space_sel(param, idx);
		break;
	case ISP_PRO_COMMON_STORE_COLOR_SPACE_SEL:
		ret = isp_k_common_store_color_space_sel(param, idx);
		break;
	case ISP_PRO_COMMON_AWBM_POS_SEL:
		break;
	case ISP_PRO_COMMON_CH0_AEM2_POS:
		break;
	case ISP_PRO_COMMON_CH0_Y_AFM_POS:
		break;
	case ISP_PRO_COMMON_CH1_AEM2_POS:
		break;
	case ISP_PRO_COMMON_CH1_Y_AFM_POS:
		break;
	case ISP_PRO_COMMON_LBUF_OFFSET:
		break;
	case ISP_PRO_COMMON_SHADOW_ALL_CTRL:
		break;
	case ISP_PRO_COMMON_AWBM_SHADOW:
		break;
	case ISP_PRO_COMMON_AE_SHADOW:
		break;
	case ISP_PRO_COMMON_AF_SHADOW:
		break;
	case ISP_PRO_COMMON_AFL_SHADOW:
		break;
	case ISP_PRO_COMMON_COMM_SHADOW:
		ret = isp_k_common_shadow_ctrl(param, idx);
		break;
	case ISP_PRO_COMMON_3A_SINGLE_FRAME_CTRL:
		ret = isp_k_3a_shadow_ctrl(param, idx);
		break;
	default:
		pr_err("common:id:%d, not supported.\n", param->property);
		break;
	}

	return ret;
}

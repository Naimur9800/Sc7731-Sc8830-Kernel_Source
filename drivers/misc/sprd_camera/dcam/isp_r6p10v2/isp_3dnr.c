/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
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

#include <linux/pagemap.h>
#include <linux/sprd_iommu.h>
#include "isp_3dnr.h"
#include "isp_drv.h"
#include "isp_block.h"
#include <video/sprd_isp_r6p10v2.h>

static struct isp_3dnr_const_param g_const_param_cap = {
	0, 1,
	5, 3, 3, 255, 255, 255,
	0, 255, 0, 255,
	30, 30, 30, 30, 30, 30, 30, 30, 30,
	30, 30, 30, 30, 30, 30, 30, 30, 30,
	30, 30, 30, 30, 30, 30, 30, 30, 30,
	63, 63, 63, 63, 63, 63, 63, 63, 63,
	63, 63, 63, 63, 63, 63, 63, 63, 63,
	63, 63, 63, 63, 63, 63, 63, 63, 63,
	127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
};

static struct isp_3dnr_const_param g_const_param_pre = {
	0, 1,
	5, 3, 3, 255, 255, 255,
	0, 255, 0, 255,
	20, 20, 20, 20, 20, 20, 20, 20, 20,
	15, 15, 15, 15, 15, 15, 15, 15, 15,
	15, 15, 15, 15, 15, 15, 15, 15, 15,
	63, 63, 63, 63, 63, 63, 63, 63, 63,
	63, 63, 63, 63, 63, 63, 63, 63, 63,
	63, 63, 63, 63, 63, 63, 63, 63, 63,
	127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
};

static int isp_k_3dnr_update_pre_param(struct isp_io_param *param,
					enum isp_id idx)
{
	int ret = 0;
	struct isp_3dnr_const_param pre_param;

	memset(&pre_param, 0x00, sizeof(pre_param));
	ret = copy_from_user((void *)&pre_param, param->property_param,
			sizeof(pre_param));
	if (ret != 0) {
		pr_err("pre_info: copy error, ret=0x%x\n",
						(unsigned int)ret);
		return -EPERM;
	}

	memcpy(&g_const_param_pre, &pre_param, sizeof(pre_param));

	return ret;
}

static int isp_k_3dnr_update_cap_param(struct isp_io_param *param,
					enum isp_id idx)
{
	int ret = 0;
	struct isp_3dnr_const_param cap_param;

	memset(&cap_param, 0x00, sizeof(cap_param));
	ret = copy_from_user((void *)&cap_param, param->property_param,
			sizeof(cap_param));
	if (ret != 0) {
		pr_err("isp_k_3dnr_update_cap_param: copy error, ret=0x%x\n",
						(unsigned int)ret);
		return -EPERM;
	}

	memcpy(&g_const_param_cap, &cap_param, sizeof(cap_param));

	return ret;
}

int isp_k_cfg_3dnr_param(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_3dnr_param: param is null error.\n");
		return -EPERM;
	}

	if (param->property_param == NULL) {
		pr_err("3dnr: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_3DNR_UPDATE_PRE_PARAM:
		ret = isp_k_3dnr_update_pre_param(param, idx);
		break;
	case ISP_PRO_3DNR_UPDATE_CAP_PARAM:
		ret = isp_k_3dnr_update_cap_param(param, idx);
		break;
	default:
		pr_err("fail cmd id:%d, not supported.\n", param->property);
		break;
	}

	return ret;
}

static int isp_set_3dnr_common_reg(struct _3dnr_param *param)
{
	unsigned int isp_reg = param->isp_reg_base;
	unsigned int val;
	int blending_no = param->blending_no;
	int i;
	unsigned int store_reg[] = {ISP_STORE_PRE_CTRL, ISP_STORE_VID_CTRL,
		ISP_STORE_CAP_CTRL, ISP_STORE_B_CTRL};
	struct isp_3dnr_const_param *param_ptr = &g_const_param_cap;

	/* preview mode: the blending_no is bigger than 10,
	 * such as 10, 11, 12, 13;
	 * capture mode: the blending_no is smaller than 10,
	 * such as 0, 1, 2, 3;
	 */
	if (blending_no > 10) {
		param_ptr = &g_const_param_pre;
		blending_no -= 10;
	}

	/*common*/
	val = reg_rd(isp_reg + ISP_REG_LEN + ISP_COMMON_PATH_SEL);
	for (i = 0; i < 4; i++) {
		if (3 & (val >> (i * 2))) {
			val |=  (3 << (i * 2));
			reg_or(isp_reg + ISP_REG_LEN + store_reg[i], 1);
		}
	}
	reg_set(isp_reg + ISP_REG_LEN + ISP_COMMON_PATH_SEL, val);
	reg_set(isp_reg + ISP_REG_LEN + ISP_COMMON_DISPATCH_SEL, 1);


	/* nr3 preview */
	reg_set(isp_reg + ISP_STORE0_PARAM, 1);
	reg_set(isp_reg + ISP_3DNR_STORE_PARAM, 1);
	reg_set(isp_reg + ISP_NR3_FRM_CNT_CLR, 1);
	reg_set(isp_reg + ISP_MEM_CTRL_PRE_FRAME_CNT_CLR, 1);
	val = 16 | BIT_16;
	reg_set(isp_reg + ISP_NR3_FAST_ME_PARAM0, val);
	reg_set(isp_reg + ISP_NR3_FAST_ME_PARAM1, val);
	reg_set(isp_reg + ISP_MEM_CTRL_PRE_PARAM1, val);
	reg_set(isp_reg + ISP_MEM_CTRL_PRE_FT_CUR_LUMA_ADDR0,
		param->fetch_cur_addr[0]);
	reg_set(isp_reg + ISP_MEM_CTRL_PRE_FT_CUR_CHROMA_ADDR0,
		param->fetch_cur_addr[1]);
	reg_set(isp_reg + ISP_MEM_CTRL_PRE_FT_REF_LUMA_ADDR0,
		param->fetch_ref_addr[0]);
	reg_set(isp_reg + ISP_MEM_CTRL_PRE_FT_REF_CHROMA_ADDR0,
		param->fetch_ref_addr[1]);
	reg_set(isp_reg + ISP_MEM_CTRL_PRE_PITCH, val);

	/* nr3 common */
	reg_and(isp_reg + ISP_REG_LEN + ISP_NR3_COMMON_PARAM0, ~BIT_0);
	reg_or(isp_reg + ISP_REG_LEN + ISP_NR3_COMMON_PARAM0, BIT_3);
	reg_or(isp_reg + ISP_REG_LEN + ISP_NR3_COMMON_PARAM0, BIT_8);

	/* nr3 scl */
	reg_or(isp_reg + ISP_REG_LEN + ISP_NR3_SCL_CFG,
	       BIT_9 | BIT_10 | BIT_31);

	/* arbiter */
	if (!param->store_ref_endian)
		reg_or(isp_reg + ISP_REG_LEN + ISP_ARBITER_ENDIAN_COMM,
		       BIT_16);
	else
		reg_and(isp_reg + ISP_REG_LEN + ISP_ARBITER_ENDIAN_COMM,
			~BIT_16);

	if (!param->fetch_ref_endian)
		reg_or(isp_reg + ISP_REG_LEN + ISP_ARBITER_ENDIAN_COMM, 1);
	else
		reg_and(isp_reg + ISP_REG_LEN + ISP_ARBITER_ENDIAN_COMM, ~1);

	reg_and(isp_reg + ISP_REG_LEN + ISP_ARBITER_ENDIAN_CH1, ~3);
	reg_or(isp_reg + ISP_REG_LEN + ISP_ARBITER_ENDIAN_CH1,
	       param->fetch_cur_endian & 3);

	/* int */
	reg_or(isp_reg + ISP_REG_LEN + ISP_INT_DONE_CTRL, BIT_30);
	reg_and(isp_reg + ISP_REG_LEN + ISP_INT_DONE_CTRL, ~BIT_8);

	reg_or(isp_reg + ISP_REG_LEN + ISP_INT_EN0, 0x3);
	reg_or(isp_reg + ISP_REG_LEN + ISP_INT_CLR0, 0x3);

	reg_or(isp_reg + ISP_REG_LEN + ISP_INT_EN2,
	       BIT_14 | BIT_22 | BIT_23);
	reg_or(isp_reg + ISP_REG_LEN + ISP_INT_CLR2,
	       BIT_14 | BIT_22 | BIT_23);

	/* blend cap */
	val = ((param_ptr->fusion_mode & 0x1) << 1) |
		((param_ptr->filter_switch & 0x1) << 2) | (1 << 3);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CONTROL0, val);

	val = (param->image_height & 0xFFFF) |
		((param->image_width & 0xFFFF) << 16);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG0, val);

	if (blending_no > 3)
		blending_no = 3;
	val = (param_ptr->y_pixel_noise_threshold & 0xFF)
		|((param_ptr->v_pixel_src_weight[blending_no] & 0xFF) << 8)
		|((param_ptr->u_pixel_src_weight[blending_no] & 0xFF) << 16)
		|((param_ptr->y_pixel_src_weight[blending_no] & 0xFF) << 24);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG1, val);

	val = ((param_ptr->u_pixel_noise_threshold & 0xFF) << 24) |
		((param_ptr->v_pixel_noise_threshold & 0xFF) << 16)
		| ((param_ptr->y_pixel_noise_weight & 0xFF) << 8) |
		(param_ptr->u_pixel_noise_weight & 0xFF);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG2, val);

	val = ((param_ptr->v_pixel_noise_weight & 0xFF) << 24) |
		((param_ptr->threshold_radial_variation_u_range_min & 0xFF)
		 << 16)
		| ((param_ptr->threshold_radial_variation_u_range_max &
		    0xFF) << 8)
		| (param_ptr->threshold_radial_variation_v_range_min & 0xFF);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG3, val);

	val = ((param_ptr->threshold_radial_variation_v_range_max & 0xFF)
	       << 24) | ((param_ptr->y_threshold_polyline_0 & 0xFF) << 16)
		| ((param_ptr->y_threshold_polyline_1 & 0xFF) << 8) |
		(param_ptr->y_threshold_polyline_2 & 0xFF);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG4, val);

	val = ((param_ptr->y_threshold_polyline_3 & 0xFF) << 24) |
		((param_ptr->y_threshold_polyline_4 & 0xFF) << 16)
		| ((param_ptr->y_threshold_polyline_5 & 0xFF) << 8) |
		(param_ptr->y_threshold_polyline_6 & 0xFF);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG5, val);

	val = ((param_ptr->y_threshold_polyline_7 & 0xFF) << 24) |
		((param_ptr->y_threshold_polyline_8 & 0xFF) << 16)
		| ((param_ptr->u_threshold_polyline_0 & 0xFF) << 8) |
		(param_ptr->u_threshold_polyline_1 & 0xFF);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG6, val);

	val = ((param_ptr->u_threshold_polyline_2 & 0xFF) << 24) |
		((param_ptr->u_threshold_polyline_3 & 0xFF) << 16)
		| ((param_ptr->u_threshold_polyline_4 & 0xFF) << 8) |
		(param_ptr->u_threshold_polyline_5 & 0xFF);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG7, val);

	val = ((param_ptr->u_threshold_polyline_6 & 0xFF) << 24) |
		((param_ptr->u_threshold_polyline_7 & 0xFF) << 16)
		| ((param_ptr->u_threshold_polyline_8 & 0xFF) << 8) |
		(param_ptr->v_threshold_polyline_0 & 0xFF);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG8, val);

	val = ((param_ptr->v_threshold_polyline_1 & 0xFF) << 24) |
		((param_ptr->v_threshold_polyline_2 & 0xFF) << 16)
		| ((param_ptr->v_threshold_polyline_3 & 0xFF) << 8) |
		(param_ptr->v_threshold_polyline_4 & 0xFF);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG9, val);

	val = ((param_ptr->v_threshold_polyline_5 & 0xFF) << 24) |
		((param_ptr->v_threshold_polyline_6 & 0xFF) << 16)
		| ((param_ptr->v_threshold_polyline_7 & 0xFF) << 8) |
		(param_ptr->v_threshold_polyline_8 & 0xFF);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG10, val);

	val = ((param_ptr->y_intensity_gain_polyline_0 & 0x7F) << 24) |
		((param_ptr->y_intensity_gain_polyline_1 & 0x7F) << 16)
		| ((param_ptr->y_intensity_gain_polyline_2 & 0x7F) << 8) |
		(param_ptr->y_intensity_gain_polyline_3 & 0x7F);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG11, val);

	val = ((param_ptr->y_intensity_gain_polyline_4 & 0x7F) << 24) |
		((param_ptr->y_intensity_gain_polyline_5 & 0x7F) << 16)
		| ((param_ptr->y_intensity_gain_polyline_6 & 0x7F) << 8) |
		(param_ptr->y_intensity_gain_polyline_7 & 0x7F);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG12, val);

	val = ((param_ptr->y_intensity_gain_polyline_8 & 0x7F) << 24) |
		((param_ptr->u_intensity_gain_polyline_0 & 0x7F) << 16)
		| ((param_ptr->u_intensity_gain_polyline_1 & 0x7F) << 8) |
		(param_ptr->u_intensity_gain_polyline_2 & 0x7F);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG13, val);

	val = ((param_ptr->u_intensity_gain_polyline_3 & 0x7F) << 24) |
		((param_ptr->u_intensity_gain_polyline_4 & 0x7F) << 16)
		| ((param_ptr->u_intensity_gain_polyline_5 & 0x7F) << 8) |
		(param_ptr->u_intensity_gain_polyline_6 & 0x7F);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG14, val);

	val = ((param_ptr->u_intensity_gain_polyline_7 & 0x7F) << 24) |
		((param_ptr->u_intensity_gain_polyline_8 & 0x7F) << 16)
		| ((param_ptr->v_intensity_gain_polyline_0 & 0x7F) << 8) |
		(param_ptr->v_intensity_gain_polyline_1 & 0x7F);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG15, val);

	val = ((param_ptr->v_intensity_gain_polyline_2 & 0x7F) << 24) |
		((param_ptr->v_intensity_gain_polyline_3 & 0x7F) << 16)
		| ((param_ptr->v_intensity_gain_polyline_4 & 0x7F) << 8) |
		(param_ptr->v_intensity_gain_polyline_5 & 0x7F);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG16, val);

	val = ((param_ptr->v_intensity_gain_polyline_6 & 0x7F) << 24) |
		((param_ptr->v_intensity_gain_polyline_7 & 0x7F) << 16)
		| ((param_ptr->v_intensity_gain_polyline_8 & 0x7F) << 8) |
		(param_ptr->gradient_weight_polyline_0 & 0x7F);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG17, val);

	val = ((param_ptr->gradient_weight_polyline_1 & 0x7F) << 24) |
		((param_ptr->gradient_weight_polyline_2 & 0x7F) << 16)
		| ((param_ptr->gradient_weight_polyline_3 & 0x7F) << 8) |
		(param_ptr->gradient_weight_polyline_4 & 0x7F);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG18, val);

	val = ((param_ptr->gradient_weight_polyline_5 & 0x7F) << 24) |
		((param_ptr->gradient_weight_polyline_6 & 0x7F) << 16)
		| ((param_ptr->gradient_weight_polyline_7 & 0x7F) << 8) |
		(param_ptr->gradient_weight_polyline_8 & 0x7F);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG19, val);

	val = ((param_ptr->gradient_weight_polyline_9 & 0x7F) << 24) |
		((param_ptr->gradient_weight_polyline_10 & 0x7F) << 16)
		| ((param_ptr->u_threshold_factor[0] & 0x7F) << 8) |
		(param_ptr->u_threshold_factor[1] & 0x7F);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG20, val);

	val = ((param_ptr->u_threshold_factor[2] & 0x7F) << 24) |
		((param_ptr->u_threshold_factor[3] & 0x7F) << 16)
		| ((param_ptr->v_threshold_factor[0] & 0x7F) << 8) |
		(param_ptr->v_threshold_factor[1] & 0x7F);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG21, val);

	val = ((param_ptr->v_threshold_factor[2] & 0x7F) << 24) |
		((param_ptr->v_threshold_factor[3] & 0x7F) << 16)
		| ((param_ptr->u_divisor_factor[0] & 0x7) << 12) |
		((param_ptr->u_divisor_factor[1] & 0x7) << 8)
		| ((param_ptr->u_divisor_factor[2] & 0x7) << 4) |
		(param_ptr->u_divisor_factor[3] & 0x7);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG22, val);

	val = ((param_ptr->v_divisor_factor[0] & 0x7) << 28) |
		((param_ptr->v_divisor_factor[1] & 0x7) << 24)
		| ((param_ptr->v_divisor_factor[2] & 0x7) << 20) |
		((param_ptr->v_divisor_factor[3] & 0x7) << 16)
		| (param_ptr->r1_circle & 0xFFF);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG23, val);

	val = ((param_ptr->r2_circle & 0xFFF) << 16)
			| (param_ptr->r3_circle & 0xFFF);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_BLEND_CFG24, val);

	/* mem ctrl cap frame */
	val = (param->mv_y & 0xFF) |
		((param->mv_x & 0xFF) << 8) | (1 << 17) | (1 << 24) |
		(1 << 25) | (1 << 29);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_MEM_CTRL_CAP_PARAM0, val);

	val = (param->image_width & 0xFFFF) |
		((param->image_width & 0xFFFF) << 16);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_MEM_FT_CTRL_PITCH, val);

	val = param->image_width & 0xFFFF;
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_MEM_ST_CTRL_PITCH, val);

	val = (param->mv_y & 0xFF) |
		((param->mv_x & 0xFF) << 8);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_MEM_CTRL_CAP_PARAM7, val);

	val = ((param->image_height & 0xFFFF) << 16) |
		(param->image_width & 0xFFFF);
	reg_set(isp_reg + ISP_REG_LEN + ISP_NR3_MEM_CTRL_CAP_PARAM13, val);

	return 0;
}

static int isp_generate_fmcu_cmd(struct _3dnr_param *param)
{
	unsigned int isp_reg = 0xd2e00000;
	int slice_id, slice_num;

	unsigned int *cmd_buf = param->fmcu_cmd_vbuf;
	struct isp_3dnr_slice_param *slice_param =
		param->isp_3dnr_slice_param;
	int cmd_num = 0;

	slice_num = param->slice_row_num * param->slice_col_num;

	for (slice_id = 0; slice_id < slice_num; slice_id++) {
		/* clear the interrupt */
		*cmd_buf++ = 0xFFFFFFFF;
		*cmd_buf++ = isp_reg + ISP_REG_LEN + ISP_INT_CLR0;
		cmd_num++;
		*cmd_buf++ = 0xFFFFFFFF;
		*cmd_buf++ = isp_reg + ISP_REG_LEN + ISP_INT_CLR1;
		cmd_num++;
		*cmd_buf++ = 0xFFFFFFFF;
		*cmd_buf++ = isp_reg + ISP_REG_LEN + ISP_INT_CLR2;
		cmd_num++;
		*cmd_buf++ = 0xFFFFFFFF;
		*cmd_buf++ = isp_reg + ISP_REG_LEN + ISP_INT_CLR3;
		cmd_num++;

		if (slice_id == slice_num - 1)
			*cmd_buf++ = 3;
		else
			*cmd_buf++ = 0;
		*cmd_buf++ = isp_reg + ISP_REG_LEN + ISP_INT_EN0;
		cmd_num++;

		if (slice_id == slice_num - 1)
			*cmd_buf++ = BIT_14 | BIT_22 | BIT_23;
		else
			*cmd_buf++ = BIT_22 | BIT_23;
		*cmd_buf++ = isp_reg + ISP_REG_LEN + ISP_INT_EN2;
		cmd_num++;

		*cmd_buf++ = (slice_param[slice_id].trim1_size_x & 0x1FFF) |
			((slice_param[slice_id].trim1_size_y & 0x1FFF) << 16);
		*cmd_buf++ = isp_reg + ISP_REG_LEN + ISP_DISPATCH_YUV_CH1_SIZE;
		cmd_num++;

		*cmd_buf++ = (slice_param[slice_id].trim1_size_x & 0x1FFF) |
			((slice_param[slice_id].trim1_size_y & 0x1FFF) << 16);
		*cmd_buf++ = isp_reg + ISP_REG_LEN + ISP_NR3_SCL_TRIM1_SIZE;
		cmd_num++;

		*cmd_buf++ = (slice_param[slice_id].cur_img_width_fetch &
			      0xFFFF) |
			((slice_param[slice_id].cur_img_height_fetch & 0xFFFF)
			 << 16);
		*cmd_buf++ = isp_reg + ISP_REG_LEN +
			ISP_NR3_MEM_CTRL_CAP_FT_CUR_SIZE;
		cmd_num++;

		*cmd_buf++ = slice_param[slice_id].comb2scl_hblank_num & 0xFFFF;
		*cmd_buf++ = isp_reg + ISP_REG_LEN +
			ISP_NR3_MEM_CTRL_CAP_HBLANK;
		cmd_num++;

		*cmd_buf++ = slice_param[slice_id].ref_img_addr_store[0];
		*cmd_buf++ = isp_reg + ISP_REG_LEN +
			ISP_NR3_MEM_CTRL_CAP_ST_REF_ADDR0;
		cmd_num++;

		*cmd_buf++ = slice_param[slice_id].ref_img_addr_store[1];
		*cmd_buf++ = isp_reg + ISP_REG_LEN +
			ISP_NR3_MEM_CTRL_CAP_ST_REF_ADDR1;
		cmd_num++;

		*cmd_buf++ = slice_param[slice_id].ref_img_addr_fetch[0];
		*cmd_buf++ = isp_reg + ISP_REG_LEN +
			ISP_NR3_MEM_CTRL_CAP_FT_REF_ADDR0;
		cmd_num++;

		*cmd_buf++ = slice_param[slice_id].ref_img_addr_fetch[1];
		*cmd_buf++ = isp_reg + ISP_REG_LEN +
			ISP_NR3_MEM_CTRL_CAP_FT_REF_ADDR1;
		cmd_num++;

		*cmd_buf++ = slice_param[slice_id].cur_img_addr_fetch[0];
		*cmd_buf++ = isp_reg + ISP_REG_LEN +
			ISP_NR3_MEM_CTRL_CAP_FT_CUR_ADDR0;
		cmd_num++;

		*cmd_buf++ = slice_param[slice_id].cur_img_addr_fetch[1];
		*cmd_buf++ = isp_reg + ISP_REG_LEN +
			ISP_NR3_MEM_CTRL_CAP_FT_CUR_ADDR1;
		cmd_num++;

		*cmd_buf++ = (slice_param[slice_id].ref_img_width_fetch[0] &
			      0xFFFF) |
			((slice_param[slice_id].ref_img_height_fetch[0] &
			  0xFFFF) << 16);
		*cmd_buf++ = isp_reg + ISP_REG_LEN +
			ISP_NR3_MEM_CTRL_CAP_FT_REF_SIZE0;
		cmd_num++;

		*cmd_buf++ = (slice_param[slice_id].ref_img_width_fetch[1] &
			      0xFFFF) |
			((slice_param[slice_id].ref_img_height_fetch[1] &
			  0xFFFF) << 16);
		*cmd_buf++ = isp_reg + ISP_REG_LEN +
			ISP_NR3_MEM_CTRL_CAP_FT_REF_SIZE1;
		cmd_num++;

		*cmd_buf++ = (slice_param[slice_id].ref_img_overlap_up &
			      0xFF) |
			((slice_param[slice_id].ref_img_overlap_down & 0xFF)
			 << 8) | ((slice_param[slice_id].ref_img_overlap_left &
			  0xFF) << 16) |
			 ((slice_param[slice_id].ref_img_overlap_right & 0xFF)
			  << 24);
		*cmd_buf++ = isp_reg + ISP_REG_LEN +
			ISP_NR3_MEM_CTRL_CAP_ST_REF_BORDER;
		cmd_num++;

		*cmd_buf++ = (slice_param[slice_id].ref_img_width_store &
			      0xFFFF) |
			((slice_param[slice_id].ref_img_height_store & 0xFFFF)
			 << 16);
		*cmd_buf++ = isp_reg + ISP_REG_LEN +
			ISP_NR3_MEM_CTRL_CAP_ST_REF_SIZE;
		cmd_num++;

		*cmd_buf++ = (slice_param[slice_id].global_start_row & 0xFFFF) |
			((slice_param[slice_id].global_start_col & 0xFFFF)
			 << 16);
		*cmd_buf++ = isp_reg + ISP_REG_LEN +
			ISP_NR3_BLEND_CAP_GLOBAL_START_POS;
		cmd_num++;

		*cmd_buf++ = (slice_param[slice_id].global_start_row &
			      0xFFFF) |
			((slice_param[slice_id].global_start_col & 0xFFFF)
			 << 16);
		*cmd_buf++ = isp_reg + ISP_REG_LEN +
			ISP_NR3_MEM_CTRL_CAP_GLOBAL_START_POS;
		cmd_num++;

		*cmd_buf++ = (slice_param[slice_id].first_line_mode & 0x1) |
			((slice_param[slice_id].last_line_mode & 0x1) << 1);
		*cmd_buf++ = isp_reg + ISP_REG_LEN +
			ISP_NR3_MEM_CTRL_CAP_LINE_MODE;
		cmd_num++;

		*cmd_buf++ = 1;
		*cmd_buf++ = isp_reg + ISP_REG_LEN + ISP_NR3_CAP_SHADOW_CLR;
		cmd_num++;

		*cmd_buf++ = 1;
		*cmd_buf++ = isp_reg + ISP_REG_LEN + ISP_NR3_COMMON_CAP_SOF;
		cmd_num++;

		*cmd_buf++ = 0x14;
		*cmd_buf++ =  isp_reg + ISP_REG_LEN + ISP_FMCU_CMD;
		cmd_num++;
	}

	param->fmcu_cmd_num = cmd_num;

	return 0;
}

static int isp_generate_slice_param(struct _3dnr_param *param)
{
	unsigned int i, j;
	unsigned int overlap_up = YUV_OVERLAP_UP;
	unsigned int overlap_down = YUV_OVERLAP_DOWN;
	unsigned int overlap_left = YUV_OVERLAP_LEFT;
	unsigned int overlap_right = YUV_OVERLAP_RIGHT;
	unsigned int img_height = param->image_height;
	unsigned int img_width = param->image_width;
	unsigned int slice_total_row = SLICE_ROW_NUM;
	unsigned int slice_height = (img_height + slice_total_row - 1) /
		slice_total_row;
	unsigned int slice_width;
	unsigned int slice_total_col;

	if (img_width <= SLICE_DIV)
		slice_total_col = 1;
	else if (img_width <= SLICE_DIV * 2)
		slice_total_col = 2;
	else
		slice_total_col = 3;
	slice_width = (img_width + slice_total_col - 1) / slice_total_col;
	slice_width = (slice_width + 1) / 2 * 2;

	param->slice_width = slice_width;
	param->slice_height = slice_height;
	param->slice_row_num = slice_total_row;
	param->slice_col_num = slice_total_col;

	for (i = 0; i < slice_total_row; i++) {
		for (j = 0; j < slice_total_col; j++) {
			struct slice_pos temp_win = {0};
			struct slice_overlap temp_overlap = {0};

			temp_win.start_col = j * slice_width;
			temp_win.start_row = i * slice_height;

			if (i != 0) {
				temp_win.start_row -= overlap_up;
				temp_overlap.overlap_up = overlap_up;
			}

			if (j != 0) {
				temp_win.start_col -= overlap_left;
				temp_overlap.overlap_left = overlap_left;
			}

			if (i != slice_total_row - 1) {
				temp_win.end_row = (i + 1) * slice_height -
					1 + overlap_down;
				temp_overlap.overlap_down = overlap_down;
			} else {
				temp_win.end_row = img_height - 1;
			}

			if (j != slice_total_col - 1) {
				temp_win.end_col = (j + 1) * slice_width -
					1 + overlap_right;
				temp_overlap.overlap_right = overlap_right;
			} else {
				temp_win.end_col = img_width - 1;
			}

			param->slice_pos[i * slice_total_col + j] = temp_win;
			param->slice_overlap[i * slice_total_col + j] =
				temp_overlap;
		}
	}

	return 0;
}

static int isp_generate_3dnr_slice_param(struct _3dnr_param *param)
{
	int slice_id, slice_num;
	struct slice_pos *slice_pos = param->slice_pos;
	struct slice_overlap *slice_overlap = param->slice_overlap;

	unsigned int slice_row_num, slice_col_num;
	unsigned int slice_width, slice_height;

	struct isp_3dnr_slice_param *slice_param = param->isp_3dnr_slice_param;

	int mv_x = param->mv_x;
	int mv_y = param->mv_y;

	int ref_frame_height = param->image_height;
	int ref_frame_width = param->image_width;

	unsigned int cur_frame_width = param->image_width;

	slice_row_num = param->slice_row_num;
	slice_col_num = param->slice_col_num;
	slice_num = slice_row_num * slice_col_num;
	slice_width = param->slice_width;
	slice_height = param->slice_height;

	for (slice_id = 0; slice_id < slice_num; slice_id++) {
		int start_row = slice_pos[slice_id].start_row;
		int end_row = slice_pos[slice_id].end_row;
		int start_col = slice_pos[slice_id].start_col;
		int end_col = slice_pos[slice_id].end_col;

		unsigned int fetch_ref_start_row_y422, fetch_ref_overlap_up_uv,
			     fetch_ref_start_row_uv420;
		unsigned int fetch_ref_end_row_y422, fetch_ref_overlap_down_uv,
			     fetch_ref_end_row_uv420, fetch_ref_start_col_y422,
			     fetch_ref_start_col_uv420;
		unsigned int fetch_ref_end_col_y422, fetch_ref_end_col_uv420;

		unsigned int fetch_addr, fetch_pitch;

		unsigned int overlap_up, overlap_down, overlap_left,
			     overlap_right;

		unsigned int cur_slice_row, cur_slice_col;
		unsigned int store_buf, store_pitch;

		slice_param[slice_id].trim1_size_x = end_col - start_col + 1;
		slice_param[slice_id].trim1_size_y = end_row - start_row + 1;

		slice_param[slice_id].global_start_row = start_row;
		slice_param[slice_id].global_start_col = start_col;

		/*ref slice fetch start row*/
		fetch_ref_start_row_y422 = (start_row + mv_y) < 0 ?
			0 : (start_row + mv_y);
		fetch_ref_overlap_up_uv = 0;
		if (fetch_ref_start_row_y422 & 1)
			fetch_ref_overlap_up_uv = 1;
		fetch_ref_start_row_uv420 = fetch_ref_start_row_y422 / 2;

		/*ref slice fetch end row*/
		fetch_ref_end_row_y422 = (end_row + mv_y >= ref_frame_height) ?
			(ref_frame_height-1) : (end_row + mv_y);
		fetch_ref_overlap_down_uv = 0;
		if ((fetch_ref_end_row_y422 & 1) && fetch_ref_end_row_y422 !=
		    ref_frame_height-1)
			fetch_ref_overlap_down_uv = 1;
		fetch_ref_end_row_uv420 = (fetch_ref_end_row_y422 + 1) / 2;

		/*ref slice height*/
		slice_param[slice_id].ref_img_height_fetch[0] =
			fetch_ref_end_row_y422 - fetch_ref_start_row_y422 + 1;
		slice_param[slice_id].ref_img_height_fetch[1] =
			fetch_ref_end_row_uv420 - fetch_ref_start_row_uv420 + 1;

		slice_param[slice_id].first_line_mode =
			fetch_ref_overlap_up_uv;
		slice_param[slice_id].last_line_mode =
			fetch_ref_overlap_down_uv;

		/*ref slice fetch start col*/
		fetch_ref_start_col_y422 = (start_col + mv_x) < 0 ?
			0 : (start_col + mv_x);
		if (start_col + mv_x < 0) {
			if (mv_x & 1)
				fetch_ref_start_col_uv420 = 1;
			else
				fetch_ref_start_col_uv420 = 0;
		} else {
			fetch_ref_start_col_uv420 = start_col + mv_x;
			if (mv_x > 0 && (mv_x & 1))
				fetch_ref_start_col_uv420 -= 1;
			if (mv_x < 0 && (mv_x & 1))
				fetch_ref_start_col_uv420 += 1;
		}

		/*ref slice fetch end col*/
		if (end_col + mv_x >= ref_frame_width) {
			fetch_ref_end_col_y422 = ref_frame_width - 1;
			fetch_ref_end_col_uv420 = ref_frame_width - 1;
			if (mv_x > 0 && (mv_x & 1))
				fetch_ref_end_col_y422 += 1;
		} else {
			fetch_ref_end_col_y422 = end_col + mv_x;
			fetch_ref_end_col_uv420 = end_col + mv_x;
			if (mv_x > 0 && (mv_x & 1))
				fetch_ref_end_col_uv420 -= 1;
			if (mv_x < 0 && (mv_x & 1))
				fetch_ref_end_col_uv420 += 1;
		}

		/*ref slice height*/
		slice_param[slice_id].ref_img_width_fetch[0] =
			fetch_ref_end_col_y422 - fetch_ref_start_col_y422 + 1;
		slice_param[slice_id].ref_img_width_fetch[1] =
			fetch_ref_end_col_uv420 - fetch_ref_start_col_uv420 + 1;

		/*ref fetch addr*/
		fetch_addr = param->fetch_ref_addr[0];
		fetch_pitch = ref_frame_width;
		slice_param[slice_id].ref_img_addr_fetch[0] =
			fetch_addr + fetch_ref_start_row_y422 * fetch_pitch +
			fetch_ref_start_col_y422;

		fetch_addr = param->fetch_ref_addr[1];
		fetch_pitch = ref_frame_width;
		slice_param[slice_id].ref_img_addr_fetch[1] =
			fetch_addr + fetch_ref_start_row_uv420 * fetch_pitch +
			fetch_ref_start_col_uv420;

		/*cur fetch*/
		fetch_addr = param->fetch_cur_addr[0];
		fetch_pitch = cur_frame_width;
		slice_param[slice_id].cur_img_addr_fetch[0] =
			fetch_addr + start_row * fetch_pitch + start_col;

		fetch_addr = param->fetch_cur_addr[1];
		fetch_pitch = cur_frame_width;
		slice_param[slice_id].cur_img_addr_fetch[1] =
			fetch_addr + start_row / 2 * fetch_pitch + start_col;

		slice_param[slice_id].cur_img_width_fetch = end_col -
			start_col + 1;
		slice_param[slice_id].cur_img_height_fetch = end_row -
			start_row + 1;

		/*ref store*/
		overlap_up = slice_overlap[slice_id].overlap_up;
		overlap_down = slice_overlap[slice_id].overlap_down;
		overlap_left = slice_overlap[slice_id].overlap_left;
		overlap_right = slice_overlap[slice_id].overlap_right;
		slice_param[slice_id].ref_img_overlap_up = overlap_up;
		slice_param[slice_id].ref_img_overlap_down = overlap_down;
		slice_param[slice_id].ref_img_overlap_left = overlap_left;
		slice_param[slice_id].ref_img_overlap_right = overlap_right;

		slice_param[slice_id].ref_img_width_store = end_col -
			start_col + 1 - overlap_left - overlap_right;
		slice_param[slice_id].ref_img_height_store = end_row -
			start_row + 1 - overlap_up - overlap_down;

		cur_slice_row = slice_id / slice_col_num;
		cur_slice_col = slice_id % slice_col_num;
		store_buf = param->store_ref_addr[0];
		store_pitch = ref_frame_width;
		slice_param[slice_id].ref_img_addr_store[0] =
			store_buf + cur_slice_row * slice_height * store_pitch +
			cur_slice_col * slice_width;

		store_buf = param->store_ref_addr[1];
		store_pitch = ref_frame_width;
		slice_param[slice_id].ref_img_addr_store[1] =
			store_buf + (cur_slice_row * slice_height + 1) / 2 *
			store_pitch + cur_slice_col * slice_width;

		slice_param[slice_id].comb2scl_hblank_num = 256;
	}

	return 0;
}

static int isp_generate_3dnr_cmd_queue(struct _3dnr_param *param)
{
	isp_generate_slice_param(param);

	isp_generate_3dnr_slice_param(param);

	isp_generate_fmcu_cmd(param);

	return 0;
}

static int isp_start_fmcu(struct _3dnr_param *param)
{
	unsigned int isp_reg = param->isp_reg_base;

	/* flush_dcache_page(virt_to_page(param->fmcu_cmd_vbuf)); */
	clflush_cache_range((void *)param->fmcu_cmd_vbuf, PAGE_SIZE);
	reg_set(isp_reg + ISP_REG_LEN + ISP_FMCU_CTRL,
		param->fmcu_cmd_num << 16);
	reg_set(isp_reg + ISP_REG_LEN + ISP_FMCU_ADDR, param->fmcu_cmd_pbuf);
	reg_set(isp_reg + ISP_REG_LEN + ISP_FETCH2_LINE_DLY_CTRL, 0x200);

	reg_set(isp_reg + ISP_REG_LEN + ISP_FMCU_START, 1);

	return 0;
}

int isp_3dnr_process_one_frame(struct isp_3dnr_info *param)
{
	struct _3dnr_param _param;

	pr_debug("isp_3dnr_process_one_frame in.\n");

	_param.mv_x = param->mv_x;
	_param.mv_y = param->mv_y;

	_param.fetch_cur_addr[0] = param->fetch_cur_addr[0];
	_param.fetch_ref_addr[0] = param->fetch_ref_addr[0];
	_param.store_ref_addr[0] = param->store_ref_addr[0];

	_param.fetch_cur_addr[1] = param->fetch_cur_addr[1];
	_param.fetch_ref_addr[1] = param->fetch_ref_addr[1];
	_param.store_ref_addr[1] = param->store_ref_addr[1];

	_param.fetch_cur_endian = param->fetch_cur_endian;
	_param.fetch_ref_endian = param->fetch_ref_endian;
	_param.store_ref_endian = param->store_ref_endian;

	_param.image_width = param->image_width;
	_param.image_height = param->image_height;
	_param.blending_no = param->blending_no;

	_param.isp_reg_base = param->isp_reg_base;

	_param.fmcu_cmd_vbuf = param->fmcu_cmd_vbuf;
	_param.fmcu_cmd_pbuf = param->fmcu_cmd_pbuf;

	isp_set_3dnr_common_reg(&_param);

	isp_generate_3dnr_cmd_queue(&_param);

	isp_start_fmcu(&_param);

	pr_debug("isp_3dnr_process_one_frame done.\n");

	return 0;
}

int isp_3dnr_stop_preview(void)
{
	reg_set(ISP_NR3_COMMON_PARAM0, 1);

	return 0;
}

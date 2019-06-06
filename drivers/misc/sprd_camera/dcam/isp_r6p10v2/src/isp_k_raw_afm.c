/*
 * Copyright(C) 2012 Spreadtrum Communications	Inc.
 *
 * This	software is licensed under the terms of	the GNU	General	Public
 * License version 2, as published by the Free Software	Foundation, and
 * may be copied, distributed, and modified under those	terms.
 *
 * This	program	is distributed in the hope that	it will	be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 */

#include <linux/uaccess.h>
#include <video/sprd_mm.h>
#include <video/sprd_isp_r6p10v2.h>
#include "../isp_drv.h"

static int isp_k_raw_afm_iir_nr_cfg(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct af_iir_nr_info af_iir_nr;

	memset(&af_iir_nr, 0x00, sizeof(af_iir_nr));
	ret = copy_from_user((void *)&af_iir_nr, param->property_param,
			sizeof(struct af_iir_nr_info));

	if (ret != 0) {
		pr_err("isp_k_rgb_dither_random_block: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_RGB_AFM_PARAM, BIT_7, af_iir_nr.iir_nr_en << 7);
	val = ((af_iir_nr.iir_g1 & 0xFFF) << 16) |
		   (af_iir_nr.iir_g0 & 0xFFF);
	ISP_REG_WR(idx, ISP_RGB_AFM_IIR_FILTER0, val);

	val = ((af_iir_nr.iir_c2 & 0xFFF) << 16) |
		   (af_iir_nr.iir_c1	 & 0xFFF);
	ISP_REG_WR(idx, ISP_RGB_AFM_IIR_FILTER1, val);

	val = ((af_iir_nr.iir_c4 & 0xFFF) << 16) |
		   (af_iir_nr.iir_c3	 & 0xFFF);
	ISP_REG_WR(idx, ISP_RGB_AFM_IIR_FILTER2, val);

	val = ((af_iir_nr.iir_c6 & 0xFFF) << 16) |
		   (af_iir_nr.iir_c5	 & 0xFFF);
	ISP_REG_WR(idx, ISP_RGB_AFM_IIR_FILTER3, val);

	val = ((af_iir_nr.iir_c8 & 0xFFF) << 16) |
		   (af_iir_nr.iir_c7	 & 0xFFF);
	ISP_REG_WR(idx, ISP_RGB_AFM_IIR_FILTER4, val);

	val = ((af_iir_nr.iir_c10 & 0xFFF) << 16) |
		   (af_iir_nr.iir_c9	 & 0xFFF);
	ISP_REG_WR(idx, ISP_RGB_AFM_IIR_FILTER5, val);
	ISP_REG_MWR(idx, ISP_RGB_AFM_CFG_READY, BIT_0, 1);

	return ret;
}

static int32_t isp_k_raw_afm_module_config(struct isp_io_param *param,
					enum isp_id idx)
{
	unsigned int  val = 0;
	int ret = 0, i = 0;
	struct af_enhanced_module_info af_enhanced_module;

	memset(&af_enhanced_module, 0x00, sizeof(af_enhanced_module));
	ret = copy_from_user((void *)&af_enhanced_module, param->property_param,
				sizeof(struct af_enhanced_module_info));

	if (ret != 0) {
		pr_err("isp_k_rgb_dither_random_block: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	val = ((af_enhanced_module.fv_shift[1] & 0x7)		  << 19) |
	  ((af_enhanced_module.fv_shift[0] & 0x7)		  << 16) |
	  ((af_enhanced_module.clip_en[1] & 0x1)		  << 15) |
	  ((af_enhanced_module.clip_en[0] & 0x1)		  << 14) |
	  ((af_enhanced_module.center_weight & 0x3)	  << 12) |
	  ((af_enhanced_module.nr_mode & 0x3)	  << 10) |
	  ((af_enhanced_module.fv_enhanced_mode[1] & 0xF) << 6) |
	  ((af_enhanced_module.fv_enhanced_mode[0] & 0xF) << 2) |
	   (af_enhanced_module.chl_sel & 0x3);
	ISP_REG_WR(idx, ISP_RGB_AFM_ENHANCE_CTRL, val);

	val = ((af_enhanced_module.max_th[0] & 0xFFFFF) << 12)	  |
		   (af_enhanced_module.min_th[0] & 0xFFF);
	ISP_REG_WR(idx, ISP_RGB_AFM_ENHANCE_FV0_THD, val);

	val = ((af_enhanced_module.max_th[1] & 0xFFFFF) << 12)	  |
		   (af_enhanced_module.min_th[0] & 0xFFF);
	ISP_REG_WR(idx, ISP_RGB_AFM_ENHANCE_FV1_THD, val);

	for (i = 0; i < 4; i++)	{
		val = ((af_enhanced_module.fv1_coeff[4 + i * 9] & 0x3F) << 24)
		| ((af_enhanced_module.fv1_coeff[3 + i * 9] & 0x3F) << 18)
		| ((af_enhanced_module.fv1_coeff[2 + i * 9] & 0x3F) << 12)
		| ((af_enhanced_module.fv1_coeff[1 + i * 9] & 0x3F) << 6)
		| (af_enhanced_module.fv1_coeff[0 + i * 9] & 0x3F);

		ISP_REG_WR(idx, ISP_RGB_AFM_ENHANCE_FV1_COEFF00 + 8 * i, val);

		val = ((af_enhanced_module.fv1_coeff[8 + i * 9] & 0x3F) << 18)
		| ((af_enhanced_module.fv1_coeff[7 + i * 9] & 0x3F) << 12)
		| ((af_enhanced_module.fv1_coeff[6 + i * 9] & 0x3F) << 6)
		| (af_enhanced_module.fv1_coeff[5 + i * 9] & 0x3F);

		ISP_REG_WR(idx, ISP_RGB_AFM_ENHANCE_FV1_COEFF01 + 8 * i, val);
	}
	ISP_REG_MWR(idx, ISP_RGB_AFM_CFG_READY, BIT_0, 1);

	return ret;
}

int isp_k_raw_afm_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0, i = 0, j = 0;
	unsigned int  val = 0;
	struct isp_dev_rgb_afm_info rafm_info;
	unsigned short iir_c[] = {2047, 0, -2048, 2047,
				-2048, 2, -2048, 2047, 3, 2047};
	unsigned int fv1_coeff[][9] = {
		{-7, 1, -3, -6, 0, 2, 7, 6, 0,},
		{6, -4, -1, 10, 0, -11, 0, 0, 0,},
		{-1, 10, 2, 0, -10, 1, 3, -3, -2,},
		{0, -2, -6, -8, 1, 2, 10, 1, 2,}, };

	memset(&rafm_info, 0x00, sizeof(rafm_info));
	if (param == NULL) {
		pr_err("isp_raw_afm_block: param is null error\n");
		return -1;
	}

	ret = copy_from_user((void *)&rafm_info, param->property_param,
			     sizeof(struct isp_dev_rgb_afm_info));

	pr_info("XSHG: in isp_k_raw_afm_block\n");

	rafm_info.bypass = 0;
	rafm_info.mode = 1;
	rafm_info.skip_num = 1;
	rafm_info.iir_eb = 0;
	rafm_info.data_update_sel = 0;
	rafm_info.overflow_protect_en = 0;
	rafm_info.touch_mode = 1;
	rafm_info.frame_size.width = 4208;
	rafm_info.frame_size.height = 3120;
	rafm_info.iir_g0 = 2047;
	rafm_info.iir_g1 = 1;
	for (i = 0; i < 10; i++)
		rafm_info.iir_c[i] = iir_c[i];

	rafm_info.channel_sel = 0;
	rafm_info.fv0_enhance_mode = 4;
	rafm_info.fv1_enhance_mode = 4;
	rafm_info.denoise_mode = 1;
	rafm_info.center_weight = 2;
	rafm_info.clip_en0 = 1;
	rafm_info.clip_en1 = 1;
	rafm_info.fv0_shift = 4;
	rafm_info.fv1_shift = 4;
	rafm_info.fv0_th.min = 128;
	rafm_info.fv0_th.max = 1023547;
	rafm_info.fv1_th.min = 3548;
	rafm_info.fv1_th.max = 84511;
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 9; j++)
			rafm_info.fv1_coeff[i][j] = fv1_coeff[i][j];
	}

	ISP_REG_MWR(idx, ISP_RGB_AFM_PARAM, BIT_0, rafm_info.bypass);
	if (rafm_info.bypass) {
		ISP_REG_MWR(idx, ISP_RGB_AFM_CFG_READY, BIT_0, 1);
		return 0;
	}

	ISP_REG_MWR(idx, ISP_RGB_AFM_PARAM, BIT_1, rafm_info.mode << 1);

	ISP_REG_MWR(idx, ISP_RGB_AFM_PARAM, 0xF<<2, (rafm_info.skip_num << 2));

	ISP_REG_MWR(idx, ISP_RGB_AFM_PARAM, BIT_7, rafm_info.iir_eb << 7);

	ISP_REG_MWR(idx, ISP_RGB_AFM_PARAM, BIT_11,
		    rafm_info.data_update_sel << 11);

	ISP_REG_MWR(idx, ISP_RGB_AFM_PARAM, BIT_12,
		    rafm_info.overflow_protect_en << 12);

	ISP_REG_MWR(idx, ISP_RGB_AFM_PARAM, BIT_13,
		    rafm_info.touch_mode << 13);

	val = (rafm_info.frame_size.height & 0xFFFF) |
		((rafm_info.frame_size.width & 0xFFFF) << 16);
	ISP_REG_WR(idx, ISP_RGB_AFM_FRAME_RANGE, val);
	for (i = 0; i < ISP_AFM_WIN_NUM; i++) {
		rafm_info.win[i].start_y = 0;
		rafm_info.win[i].start_x = 0;
		rafm_info.win[i].end_x = 4208;
		rafm_info.win[i].end_y = 3120;
		val = ((rafm_info.win[i].start_y << 16) & 0xFFFF0000) |
			   (rafm_info.win[i].start_x & 0xFFFF);
		ISP_REG_WR(idx, ISP_RGB_AFM_WIN_RANGE0S + 8 * i, val);

		val = ((rafm_info.win[i].end_y << 16) & 0xFFFF0000) |
			   (rafm_info.win[i].end_x & 0xFFFF);
		ISP_REG_WR(idx, ISP_RGB_AFM_WIN_RANGE0E + 8 * i, val);
	}

	val = ((rafm_info.iir_g1 & 0xFFF) << 16) |
		   (rafm_info.iir_g0 & 0xFFF);
	ISP_REG_WR(idx, ISP_RGB_AFM_IIR_FILTER0, val);

	for (i = 0; i < 10; i += 2) {
		val = ((rafm_info.iir_c[i+1] & 0xFFF) << 16) |
			   (rafm_info.iir_c[i]	 & 0xFFF);
		ISP_REG_WR(idx, ISP_RGB_AFM_IIR_FILTER1 + 4 * (i>>1), val);
	}

	val = ((rafm_info.fv1_shift & 0x7)		  << 19) |
		  ((rafm_info.fv0_shift & 0x7)		  << 16) |
		  ((rafm_info.clip_en1 & 0x1)		  << 15) |
		  ((rafm_info.clip_en0 & 0x1)		  << 14) |
		  ((rafm_info.center_weight & 0x3)	  << 12) |
		  ((rafm_info.denoise_mode & 0x3)	  << 10) |
		  ((rafm_info.fv1_enhance_mode & 0xF) << 6) |
		  ((rafm_info.fv0_enhance_mode & 0xF) << 2) |
		   (rafm_info.channel_sel & 0x3);
	ISP_REG_WR(idx, ISP_RGB_AFM_ENHANCE_CTRL, val);

	val = ((rafm_info.fv0_th.max & 0xFFFFF) << 12)	  |
		   (rafm_info.fv0_th.min & 0xFFF);
	ISP_REG_WR(idx, ISP_RGB_AFM_ENHANCE_FV0_THD, val);

	val = ((rafm_info.fv1_th.max & 0xFFFFF) << 12)	  |
		   (rafm_info.fv1_th.min & 0xFFF);
	ISP_REG_WR(idx, ISP_RGB_AFM_ENHANCE_FV1_THD, val);

	for (i = 0; i < 4; i++) {
		val = ((rafm_info.fv1_coeff[i][4] & 0x3F) << 24) |
			  ((rafm_info.fv1_coeff[i][3] & 0x3F) << 18) |
			  ((rafm_info.fv1_coeff[i][2] & 0x3F) << 12) |
			  ((rafm_info.fv1_coeff[i][1] & 0x3F) << 6) |
			   (rafm_info.fv1_coeff[i][0] & 0x3F);
		ISP_REG_WR(idx, ISP_RGB_AFM_ENHANCE_FV1_COEFF00 + 8 * i, val);

		val = ((rafm_info.fv1_coeff[i][8] & 0x3F) << 18) |
			  ((rafm_info.fv1_coeff[i][7] & 0x3F) << 12) |
			  ((rafm_info.fv1_coeff[i][6] & 0x3F) << 6) |
			   (rafm_info.fv1_coeff[i][5] & 0x3F);
		ISP_REG_WR(idx, ISP_RGB_AFM_ENHANCE_FV1_COEFF01 + 8 * i, val);
	}

	ISP_REG_MWR(idx, ISP_RGB_AFM_SKIP_NUM_CLR, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_RGB_AFM_CFG_READY, BIT_0, 1);

	return ret;
}

static int32_t isp_k_raw_afm_frame_range(struct isp_io_param *param,
					enum isp_id idx)
{
	int32_t ret = 0;
	uint32_t val = 0;
	struct isp_img_size  frame_size = {0, 0};

	ret = copy_from_user((void *)&frame_size,
			param->property_param, sizeof(frame_size));
	if (ret != 0) {
		pr_err("isp_k_raw_afm_frame_range: error, ret = 0x%x\n",
				(uint32_t)ret);
		return -1;
	}

	val = (frame_size.height & 0xFFFF)
		| ((frame_size.width & 0xFFFF) << 16);
	ISP_REG_WR(idx, ISP_RGB_AFM_FRAME_RANGE, val);
	ISP_REG_MWR(idx, ISP_RGB_AFM_CFG_READY, BIT_0, 1);

	return ret;
}

static int isp_k_raw_afm_bypass(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int  bypass = 0;

	ret = copy_from_user((void *)&bypass,
			param->property_param, sizeof(bypass));
	if (ret != 0) {
		pr_err("isp_k_raw_afm_bypass: copy error, ret=0x%x\n",
		       (unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_RGB_AFM_PARAM, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_RGB_AFM_PARAM, BIT_11, 0x01 << 11);

	ISP_REG_MWR(idx, ISP_RGB_AFM_PARAM, BIT_12, 0x00 << 12);

	ISP_REG_MWR(idx, ISP_RGB_AFM_PARAM, BIT_13, 0x00 << 13);
	ISP_REG_MWR(idx, ISP_RGB_AFM_CFG_READY, BIT_0, 1);

	return ret;
}

static int32_t isp_k_raw_afm_win(struct isp_io_param *param, enum isp_id idx)
{
	unsigned int ret = 0;
	int i = 0;
	unsigned int val = 0;
	struct isp_coord coord[ISP_AFM_WIN_NUM];

	memset(coord, 0, sizeof(coord));
	ret = copy_from_user((void *)&coord,
		param->property_param, sizeof(coord));
	if (ret != 0) {
		pr_err("isp_k_raw_afm_win: error, ret = 0x%x\n",
				(unsigned int)ret);
		return -1;
	}

	for (i = 0; i < ISP_AFM_WIN_NUM; i++) {
		val = ((coord[i].start_y << 16) & 0xFFFF0000) |
				(coord[i].start_x & 0xFFFF);
		ISP_REG_WR(idx, ISP_RGB_AFM_WIN_RANGE0S + 8 * i, val);

		val = ((coord[i].end_y << 16) & 0xFFFF0000) |
				(coord[i].end_x & 0xFFFF);
		ISP_REG_WR(idx, ISP_RGB_AFM_WIN_RANGE0E + 8 * i, val);
		}
	ISP_REG_MWR(idx, ISP_RGB_AFM_CFG_READY, BIT_0, 1);

	return ret;
}

static int isp_k_raw_afm_win_num(struct isp_io_param *param)
{
	int ret = 0;
	unsigned int num = 0;

	num = ISP_AFM_WIN_NUM;

	ret = copy_to_user(param->property_param, (void *)&num, sizeof(num));
	if (ret != 0) {
		ret = -1;
		pr_err("isp_k_raw_afm_win_num: copy_to_user error, ret = 0x%x\n",
				(unsigned int)ret);
	}

	return ret;
}

static int isp_k_raw_afm_mode(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int mode = 0;

	ret = copy_from_user((void *)&mode,
			param->property_param, sizeof(mode));
	if (ret != 0) {
		pr_err("isp_k_raw_afm_mode: error, ret = 0x%x\n",
			(unsigned int) ret);
		return -1;
	}

	if (mode)
		ISP_REG_OWR(idx, ISP_RGB_AFM_PARAM, BIT_1);
	else
		ISP_REG_MWR(idx, ISP_RGB_AFM_PARAM, BIT_1, 0);

	ISP_REG_MWR(idx, ISP_RGB_AFM_CFG_READY, BIT_0, 1);

	return ret;
}

static int isp_k_raw_afm_skip_num(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int skip_num = 0;

	ret = copy_from_user((void *)&skip_num,
			param->property_param, sizeof(skip_num));
	if (ret != 0) {
		pr_err("isp_k_raw_afm_skip_num: error, ret = 0x%x\n",
				(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_RGB_AFM_PARAM, 0xF << 2, (skip_num << 2));
	ISP_REG_MWR(idx, ISP_RGB_AFM_CFG_READY, BIT_0, 1);

	return ret;
}

static int32_t isp_k_raw_afm_skip_num_clr(struct isp_io_param *param,
							enum isp_id idx)
{
	int32_t ret = 0;
	unsigned int is_clear = 0;

	ret = copy_from_user((void *)&is_clear,
		param->property_param, sizeof(is_clear));
	if (ret != 0) {
		pr_err("isp_k_raw_afm_skip_num_clr: error, ret = 0x%x\n",
				(unsigned int)ret);
		return -1;
	}

	if (is_clear)
		ISP_REG_OWR(idx, ISP_RGB_AFM_PARAM, BIT_6);
	else
		ISP_REG_MWR(idx, ISP_RGB_AFM_PARAM, BIT_6, 0);

	ISP_REG_MWR(idx, ISP_RGB_AFM_CFG_READY, BIT_0, 1);

	return ret;
}

int isp_k_cfg_rgb_afm(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_afm: param is null error.\n");
		return -1;
	}

	if (!param->property_param) {
		pr_err("isp_k_cfg_afm: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_RGB_AFM_BYPASS:
		ret = isp_k_raw_afm_bypass(param, idx);
		break;
	case ISP_PRO_RGB_AFM_BLOCK:
		ret = isp_k_raw_afm_block(param, idx);
		break;
	case ISP_PRO_RGB_AFM_FRAME_SIZE:
		ret = isp_k_raw_afm_frame_range(param, idx);
		break;
	case ISP_PRO_RGB_AFM_IIR_NR_CFG:
		ret = isp_k_raw_afm_iir_nr_cfg(param, idx);
		break;
	case ISP_PRO_RGB_AFM_MODULE_CFG:
		ret = isp_k_raw_afm_module_config(param, idx);
		break;
	case ISP_PRO_RGB_AFM_WIN:
		ret = isp_k_raw_afm_win(param, idx);
		break;
	case ISP_PRO_RGB_AFM_WIN_NUM:
		ret = isp_k_raw_afm_win_num(param);
		break;
	case ISP_PRO_RGB_AFM_MODE:
		ret = isp_k_raw_afm_mode(param, idx);
		break;
	case ISP_PRO_RGB_AFM_SKIP_NUM:
		ret = isp_k_raw_afm_skip_num(param, idx);
		break;
	case ISP_PRO_RGB_AFM_SKIP_NUM_CLR:
		ret = isp_k_raw_afm_skip_num_clr(param, idx);
		break;

	default:
		pr_err("isp_k_cfg_afm: fail cmd id:%d, not supported.\n",
		       param->property);
		break;
	}

	return ret;
}

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

#include "isp_ynr.h"
#include "sin_cos.h"
#include "gen_scale_coef.h"
#include <linux/vmalloc.h>

#define COEF_SIZE (1 << 10)
#define SC_COEFF_TMP_SIZE (21 << 10)

static unsigned char YUV_DECI_MAP[] = {2, 4, 8, 16};
static unsigned char ROTATE_MODE_MAP[] = {ROTATE0, FLIP, ROTATE180, MIRROR};

static void isp_start_fetch(struct isp_drv_ynr_info *param)
{
	unsigned  int isp_reg = param->isp_reg_base;
	unsigned int val = 1;

	reg_set(isp_reg + ISP_REG_LEN + ISP_FETCH2_START, val);
}

static int isp_set_common_reg(struct isp_drv_ynr_info *param)
{
	unsigned  int isp_reg = param->isp_reg_base;
	unsigned int val;

	val = 3 | (3 << 16);
	reg_set(isp_reg + ISP_REG_LEN + ISP_COMMON_CTRL_CH1, val);
	val = 0;
	reg_set(isp_reg + ISP_REG_LEN + ISP_COMMON_YUV_PATH_SEL_CH1, val);
	val = reg_rd(isp_reg + ISP_REG_LEN + ISP_COMMON_SCL_PATH_SEL);
	val &= ~0xF0;
	val |= (1 << 4) | (3 << 6);
	reg_set(isp_reg + ISP_REG_LEN + ISP_COMMON_SCL_PATH_SEL, val);
	val = 0x1EA;
	reg_set(isp_reg + ISP_REG_LEN + ISP_YDELAY_STEP, val);

	val = 1;
	reg_set(isp_reg + ISP_REG_LEN + ISP_ALL_DONE_SRC_CTRL, val);
	val = 0;
	reg_set(isp_reg + ISP_REG_LEN + ISP_INT_EN0, val);
	reg_set(isp_reg + ISP_REG_LEN + ISP_INT_EN2, val);
	reg_set(isp_reg + ISP_REG_LEN + ISP_INT_EN3, val);
	val = 4;
	reg_set(isp_reg + ISP_REG_LEN + ISP_INT_EN1, val);
	val = 0xFFFFFFFF;
	reg_set(isp_reg + ISP_REG_LEN + ISP_INT_CLR0, val);
	reg_set(isp_reg + ISP_REG_LEN + ISP_INT_CLR1, val);
	reg_set(isp_reg + ISP_REG_LEN + ISP_INT_CLR2, val);
	reg_set(isp_reg + ISP_REG_LEN + ISP_INT_CLR3, val);
	val = 0x800003FC;
	reg_set(isp_reg + ISP_REG_LEN + ISP_INT_DONE_CTRL, val);
	val = 0;
	reg_set(isp_reg + ISP_REG_LEN + ISP_INT_SKIP_CTRL, val);
	reg_set(isp_reg + ISP_REG_LEN + ISP_INT_SKIP_CTRL1, val);

	reg_or(isp_reg + ISP_REG_LEN + ISP_PRECDN_PARAM, BIT_0);
	reg_or(isp_reg + ISP_REG_LEN + ISP_BRIGHT_PARAM, BIT_0);
	reg_or(isp_reg + ISP_REG_LEN + ISP_CONTRAST_PARAM, BIT_0);
	reg_or(isp_reg + ISP_REG_LEN + ISP_HIST_PARAM, BIT_0);
	reg_or(isp_reg + ISP_REG_LEN + ISP_HIST_CFG_READY, BIT_0);
	reg_or(isp_reg + ISP_REG_LEN + ISP_HIST2_PARAM, BIT_0);
	reg_or(isp_reg + ISP_REG_LEN + ISP_HIST2_CFG_RDY, BIT_0);
	reg_or(isp_reg + ISP_REG_LEN + ISP_CDN_PARAM, BIT_0);
	reg_or(isp_reg + ISP_REG_LEN + ISP_EE_PARAM, BIT_0);
	reg_or(isp_reg + ISP_REG_LEN + ISP_CSA_PARAM, BIT_0);
	reg_or(isp_reg + ISP_REG_LEN + ISP_HUE_PARAM, BIT_0);
	reg_or(isp_reg + ISP_REG_LEN + ISP_POSTCDN_COMMON_CTRL, BIT_0);
	reg_or(isp_reg + ISP_REG_LEN + ISP_YGAMMA_PARAM, BIT_0);
	reg_or(isp_reg + ISP_REG_LEN + ISP_IIRCNR_PARAM, BIT_0);
	reg_or(isp_reg + ISP_REG_LEN + ISP_YRANDOM_PARAM0, BIT_0);
	reg_or(isp_reg + ISP_REG_LEN + ISP_3DNR_COMMON_PARAM0, BIT_0);
	reg_or(isp_reg + ISP_3DNR_CAP_BLEND_CONTRL0, BIT_0);
	reg_or(isp_reg + ISP_REG_LEN + ISP_3DNR_PRE_BLEND_CONTRL0, BIT_0);
	reg_or(isp_reg + ISP_REG_LEN + ISP_YUV_NF_CTRL, BIT_0);
	reg_or(isp_reg + ISP_REG_LEN + ISP_STORE_BASE + ISP_STORE_PARAM,
		BIT_0);

	val = 0;
	reg_set(isp_reg + ISP_REG_LEN + ISP_ARBITER_ENDIAN_CH1, val);

	val = (param->src_img_w & 0xFFFF) |
		((param->src_img_h & 0xFFFF) << 16);
	reg_set(isp_reg + ISP_REG_LEN + ISP_DISPATCH_YUV_CH1_SIZE, val);

	val = reg_rd(isp_reg + ISP_REG_LEN + ISP_FETCH2_PARAM);
	val &= ~0xF0;
	val |= (0xB << 4);
	reg_set(isp_reg + ISP_REG_LEN + ISP_FETCH2_PARAM, val);
	val = (param->src_img_w & 0xFFFF) |
		((param->src_img_h & 0xFFFF) << 16);
	reg_set(isp_reg + ISP_REG_LEN + ISP_FETCH2_SLICE_SIZE, val);
	val = param->src_img_w;
	reg_set(isp_reg + ISP_REG_LEN + ISP_FETCH2_SLICE_Y_PITCH, val);
	reg_set(isp_reg + ISP_REG_LEN + ISP_FETCH2_SLICE_U_PITCH, val);

	val = param->src_buf_addr;
	reg_set(isp_reg + ISP_REG_LEN + ISP_FETCH2_SLICE_Y_ADDR, val);
	val = param->src_buf_addr + param->src_img_w * param->src_img_h;
	reg_set(isp_reg + ISP_REG_LEN + ISP_FETCH2_SLICE_U_ADDR, val);
	val = 0;
	reg_set(isp_reg + ISP_REG_LEN + ISP_FETCH2_SLICE_V_ADDR, val);

	return 0;
}

static int isp_set_ynr_reg(struct isp_drv_ynr_info *param)
{
	unsigned  int isp_reg = param->isp_reg_base;
	unsigned int val;
	int i;
	struct ynr_param *param_ptr = &param->ynr_param;

	val = (0&0x1)|((param_ptr->ydenoise_lowlux_bypass & 0x1) << 1)
		| ((param_ptr->ydenoise_wv_nr_enable & 0x1) << 2)
		| ((param_ptr->ydenoise_l3_blf_enable & 0x1) << 3)
		| ((param_ptr->ydenoise_l2_blf_enable & 0x1) << 4)
		| ((param_ptr->ydenoise_l1_blf_enable & 0x1) << 5);
	reg_set(isp_reg + ISP_REG_LEN + ISP_YNR_CONTRL0, val);

	val = (param_ptr->ydenoise_flat[1] & 0xFF)
		| ((param_ptr->ydenoise_flat[0] & 0xFF) << 8)
		| ((param_ptr->ydenoise_txtthresh & 0xFF) << 16)
		| ((param_ptr->ydenoise_sedgethresh & 0xFF) << 24);
	reg_set(isp_reg + ISP_REG_LEN + ISP_YNR_CFG0, val);

	val = (param_ptr->ydenoise_flat[5] & 0xFF)
		| ((param_ptr->ydenoise_flat[4] & 0xFF) << 8)
		| ((param_ptr->ydenoise_flat[3] & 0xFF) << 16)
		| ((param_ptr->ydenoise_flat[2] & 0xFF) << 24);
	reg_set(isp_reg + ISP_REG_LEN + ISP_YNR_CFG1, val);

	val = (param_ptr->ydenoise_lut_thresh[2] & 0xFF)
		| ((param_ptr->ydenoise_lut_thresh[1] & 0xFF) << 8)
		| ((param_ptr->ydenoise_lut_thresh[0] & 0xFF) << 16)
		| ((param_ptr->ydenoise_flat[6] & 0xFF) << 24);
	reg_set(isp_reg + ISP_REG_LEN + ISP_YNR_CFG2, val);

	val = (param_ptr->ydenoise_lut_thresh[6] & 0xFF)
		| ((param_ptr->ydenoise_lut_thresh[5] & 0xFF) << 8)
		| ((param_ptr->ydenoise_lut_thresh[4] & 0xFF) << 16)
		| ((param_ptr->ydenoise_lut_thresh[3] & 0xFF) << 24);
	reg_set(isp_reg + ISP_REG_LEN + ISP_YNR_CFG3, val);

	val = (param_ptr->ydenoise_addback[3] & 0xFF)
		| ((param_ptr->ydenoise_addback[2] & 0xFF) << 8)
		| ((param_ptr->ydenoise_addback[1] & 0xFF) << 16)
		| ((param_ptr->ydenoise_addback[0] & 0xFF) << 24);
	reg_set(isp_reg + ISP_REG_LEN + ISP_YNR_CFG4, val);

	val = (param_ptr->ydenoise_addback[7] & 0xFF)
		| ((param_ptr->ydenoise_addback[6] & 0xFF) << 8)
		| ((param_ptr->ydenoise_addback[5] & 0xFF) << 16)
		| ((param_ptr->ydenoise_addback[4] & 0xFF) << 24);
	reg_set(isp_reg + ISP_REG_LEN + ISP_YNR_CFG5, val);

	val = (param_ptr->ydenoise_subthresh[2] & 0x7F)
		| ((param_ptr->ydenoise_subthresh[1] & 0x7F) << 8)
		| ((param_ptr->ydenoise_subthresh[0] & 0x7F) << 16)
		| ((param_ptr->ydenoise_addback[8] & 0xFF) << 24);
	reg_set(isp_reg + ISP_REG_LEN + ISP_YNR_CFG6, val);

	val = (param_ptr->ydenoise_subthresh[6] & 0x7F)
		| ((param_ptr->ydenoise_subthresh[5] & 0x7F) << 8)
		| ((param_ptr->ydenoise_subthresh[4] & 0x7F) << 16)
		| ((param_ptr->ydenoise_subthresh[3] & 0x7F) << 24);
	reg_set(isp_reg + ISP_REG_LEN + ISP_YNR_CFG7, val);

	val = (param_ptr->ydenoise_l1_eurodist[2] & 0xF)
		| ((param_ptr->ydenoise_l1_eurodist[1] & 0xF) << 4)
		| ((param_ptr->ydenoise_l1_eurodist[0] & 0xF) << 8)
		| ((param_ptr->ydenoise_subthresh[8] & 0x7F) << 16)
		| ((param_ptr->ydenoise_subthresh[7] & 0x7F) << 24);
	reg_set(isp_reg + ISP_REG_LEN + ISP_YNR_CFG8, val);

	val = (param_ptr->ydenoise_l3_eurodist[2] & 0xF)
		| ((param_ptr->ydenoise_l3_eurodist[1] & 0xF) << 4)
		| ((param_ptr->ydenoise_l3_eurodist[0] & 0xF) << 8)
		| ((param_ptr->ydenoise_l2_eurodist[2] & 0xF) << 12)
		| ((param_ptr->ydenoise_l2_eurodist[1] & 0xF) << 16)
		| ((param_ptr->ydenoise_l2_eurodist[0] & 0xF) << 20);
	reg_set(isp_reg + ISP_REG_LEN + ISP_YNR_CFG9, val);

	val = (param_ptr->ydenoise_l0_lut_thresh1 & 0x7)
	| ((param_ptr->ydenoise_l0_lut_thresh0 & 0x7) << 4)
		| ((param_ptr->ydenoise_l1_txt_thresh1 & 0x7) << 8)
		| ((param_ptr->ydenoise_l1_txt_thresh0 & 0x7) << 12)
		| ((param_ptr->ydenoise_l3_wfindex & 0xF) << 16)
		| ((param_ptr->ydenoise_l2_wfindex & 0xF) << 20)
		| ((param_ptr->ydenoise_l1_wfindex & 0xF) << 24);
	reg_set(isp_reg + ISP_REG_LEN + ISP_YNR_CFG10, val);

	for (i = 0; i < 6; i++) {
		val = (param_ptr->wltt[i*4+3] & 0x7F)
		| ((param_ptr->wltt[i*4+2] & 0x7F) << 8)
		| ((param_ptr->wltt[i*4+1] & 0x7F) << 16)
		| ((param_ptr->wltt[i*4] & 0x7F) << 24);
		reg_set(isp_reg + ISP_REG_LEN + ISP_YNR_WLT0 + i * 4, val);
	}

	for (i = 0; i < 8; i++) {
		val = (param_ptr->freqratio[i*3+2] & 0x3FF)
		| ((param_ptr->freqratio[i*3+1] & 0x3FF) << 10)
		| ((param_ptr->freqratio[i*3] & 0x3FF) << 20);
		reg_set(isp_reg + ISP_REG_LEN + ISP_YNR_FRATIO0 + i * 4, val);
	}

	val = (param_ptr->ydenoise_imgcentery & 0xFFFF)
		| ((param_ptr->ydenoise_imgcenterx & 0xFFFF) << 16);
	reg_set(isp_reg + ISP_REG_LEN + ISP_YNR_CFG12, val);

	val = (param_ptr->dist_interval & 0xFFFF)
		| ((param_ptr->ydenoise_radius & 0xFFFF) << 16);
	reg_set(isp_reg + ISP_REG_LEN + ISP_YNR_CFG13, val);

	val = (param_ptr->ydenoise_sal_nr_str[3] & 0x7F)
		| ((param_ptr->ydenoise_sal_nr_str[2] & 0x7F) << 8)
		| ((param_ptr->ydenoise_sal_nr_str[1] & 0x7F) << 16)
		| ((param_ptr->ydenoise_sal_nr_str[0] & 0x7F) << 24);
	reg_set(isp_reg + ISP_REG_LEN + ISP_YNR_CFG14, val);

	val = (param_ptr->ydenoise_sal_nr_str[7] & 0x7F)
		| ((param_ptr->ydenoise_sal_nr_str[6] & 0x7F) << 8)
		| ((param_ptr->ydenoise_sal_nr_str[5] & 0x7F) << 16)
		| ((param_ptr->ydenoise_sal_nr_str[4] & 0x7F) << 24);
	reg_set(isp_reg + ISP_REG_LEN + ISP_YNR_CFG15, val);

	val = (param_ptr->ydenoise_sal_offset[3] & 0xFF)
		| ((param_ptr->ydenoise_sal_offset[2] & 0xFF) << 8)
		| ((param_ptr->ydenoise_sal_offset[1] & 0xFF) << 16)
		| ((param_ptr->ydenoise_sal_offset[0] & 0xFF) << 24);
	reg_set(isp_reg + ISP_REG_LEN + ISP_YNR_CFG16, val);

	val = (param_ptr->ydenoise_sal_offset[7] & 0xFF)
		| ((param_ptr->ydenoise_sal_offset[6] & 0xFF) << 8)
		| ((param_ptr->ydenoise_sal_offset[5] & 0xFF) << 16)
		| ((param_ptr->ydenoise_sal_offset[4] & 0xFF) << 24);
	reg_set(isp_reg + ISP_REG_LEN + ISP_YNR_CFG17, val);

	/*slice start x/y:0*/
	val = (0 & 0xFFFF) | ((0 & 0xFFFF) << 16);
	reg_set(isp_reg + ISP_REG_LEN + ISP_YNR_CFG11, val);

	return 0;
}

static void calc_scaler_phase(signed int phase, unsigned short factor,
	signed short *phase_int, unsigned short *phase_rmd)
{
	phase_int[0] = (signed short)(phase/factor);
	phase_rmd[0] = (unsigned short)(phase - factor*phase_int[0]);
}

static void InitFrameInfo(struct yuv_path_info *pYuvPathInfo)
{
	struct scaler_info *pScalerInfo = NULL;
	unsigned short cur_width = 0, cur_height = 0;
	int i;
	unsigned int *y_hor_coef =
		vmalloc(sizeof(unsigned int) * COEF_SIZE);
	unsigned int *y_ver_coef =
		vmalloc(sizeof(unsigned int) * COEF_SIZE);
	unsigned int *c_ver_coef =
		vmalloc(sizeof(unsigned int) * COEF_SIZE);
	unsigned int *buf_coef = vmalloc(sizeof(unsigned int) *
		SC_COEFF_TMP_SIZE);

	if (!pYuvPathInfo->yuv_scaler_bypass) {
		/*init deci info*/
		if (pYuvPathInfo->deci_info.deci_x_en)
			pYuvPathInfo->deci_info.deci_x =
			YUV_DECI_MAP[pYuvPathInfo->deci_info.deci_x_parse];
		else
			pYuvPathInfo->deci_info.deci_x = 1;
		if (pYuvPathInfo->deci_info.deci_y_en)
			pYuvPathInfo->deci_info.deci_y =
			YUV_DECI_MAP[pYuvPathInfo->deci_info.deci_y_parse];
		else
			pYuvPathInfo->deci_info.deci_y = 1;

		if (pYuvPathInfo->deci_info.deci_x_en)
			cur_width = pYuvPathInfo->trim0_info.trim_size_x /
			pYuvPathInfo->deci_info.deci_x;
		else
			cur_width = pYuvPathInfo->trim0_info.trim_size_x;

		if (pYuvPathInfo->deci_info.deci_y_en)
			cur_height = pYuvPathInfo->trim0_info.trim_size_y /
			pYuvPathInfo->deci_info.deci_y;
		else
			cur_height = pYuvPathInfo->trim0_info.trim_size_y;

		/*init scaler bypass info*/
		if (pYuvPathInfo->scaler_info.scaler_bypass == 1)
			pYuvPathInfo->scaler_info.scaler_en = 0;
		else
			pYuvPathInfo->scaler_info.scaler_en = 1;

		/*init outdata info*/
		switch (pYuvPathInfo->yuv_output_format) {
		case 0:
			pYuvPathInfo->outdata_mode = YUV422;
			break;
		case 1:
			pYuvPathInfo->outdata_mode = YUV420;
			break;
		default:
			break;
		}
		pYuvPathInfo->outdata_format = PLANAR;
		pScalerInfo = &pYuvPathInfo->scaler_info;
		pScalerInfo->scaler_in_width = cur_width;
		pScalerInfo->scaler_in_height = cur_height;
		if (pScalerInfo->scaler_en) {
			signed int scl_initial_phase_hor;
			signed int scl_initial_phase_ver;
			unsigned short scl_factor_in_hor, scl_factor_out_hor;
			unsigned short scl_factor_in_ver, scl_factor_out_ver;
			unsigned char  tap_luma_ver = 8, tap_chrome_ver = 8;
			unsigned short i_w, o_w, i_h, o_h;

			if (pYuvPathInfo->outdata_mode == YUV420)
				pScalerInfo->scaling2yuv420 = 1;
			else
				pScalerInfo->scaling2yuv420 = 0;

			i_w = pScalerInfo->scaler_in_width;
			o_w = pScalerInfo->scaler_out_width;
			i_h = pScalerInfo->scaler_in_height;
			o_h = pScalerInfo->scaler_out_height;

			scl_factor_in_hor  = i_w;
			scl_factor_out_hor = o_w;
			scl_factor_in_ver  = i_h;
			scl_factor_out_ver = o_h;

			pScalerInfo->scaler_factor_in_hor = scl_factor_in_hor;
			pScalerInfo->scaler_factor_out_hor =
				scl_factor_out_hor;
			pScalerInfo->scaler_factor_in_ver = scl_factor_in_ver;
			pScalerInfo->scaler_factor_out_ver =
				scl_factor_out_ver;

			scl_initial_phase_hor = 0;
			scl_initial_phase_ver = 0;
			pScalerInfo->init_phase_info.scaler_initial_phase[0] =
				scl_initial_phase_hor;
			pScalerInfo->init_phase_info.scaler_initial_phase[1] =
				scl_initial_phase_ver;

			calc_scaler_phase(scl_initial_phase_hor,
				scl_factor_out_hor,
				&pScalerInfo->init_phase_info.
					scaler_init_phase_int[0][0],
				&pScalerInfo->init_phase_info.
					scaler_init_phase_rmd[0][0]);
			calc_scaler_phase(scl_initial_phase_hor/2,
				scl_factor_out_hor/2,
				&pScalerInfo->init_phase_info.
					scaler_init_phase_int[0][1],
				&pScalerInfo->init_phase_info.
					scaler_init_phase_rmd[0][1]);

			calc_scaler_phase(scl_initial_phase_ver,
				scl_factor_out_ver,
				&pScalerInfo->init_phase_info.
					scaler_init_phase_int[1][0],
				&pScalerInfo->init_phase_info.
					scaler_init_phase_rmd[1][0]);
			calc_scaler_phase(scl_initial_phase_ver,
				scl_factor_out_ver,
				&pScalerInfo->init_phase_info.
					scaler_init_phase_int[1][1],
				&pScalerInfo->init_phase_info.
					scaler_init_phase_rmd[1][1]);

			if (!(dcam_gen_scale_coeff(scl_factor_in_hor,
					scl_factor_in_ver,
					scl_factor_out_hor, scl_factor_out_ver,
					y_hor_coef,
					y_ver_coef,
					c_ver_coef,
					pScalerInfo->scaling2yuv420,
					&tap_luma_ver,
					&tap_chrome_ver,
					buf_coef,
					SC_COEFF_TMP_SIZE))) {
				pr_err("fail to gen_scale_coeff\n");
				goto exit;
			}

			for (i = 0; i < 48; i++)
				pScalerInfo->scaler_coef_info.y_hor_coef[i] =
					*(y_hor_coef + i);
			for (i = 0; i < 132; i++)
				pScalerInfo->scaler_coef_info.y_ver_coef[i] =
					*(y_ver_coef + i);
			for (i = 0; i < 132; i++)
				pScalerInfo->scaler_coef_info.c_ver_coef[i] =
					*(c_ver_coef + i);

			pScalerInfo->scaler_tap = tap_luma_ver;
			pScalerInfo->scaler_y_ver_tap = tap_luma_ver;
			pScalerInfo->scaler_uv_ver_tap = tap_chrome_ver;

			cur_width = pScalerInfo->scaler_out_width;
			cur_height = pScalerInfo->scaler_out_height;
		} else {
			pScalerInfo->scaler_out_width =
				pScalerInfo->scaler_in_width;
			pScalerInfo->scaler_out_height =
				pScalerInfo->scaler_in_height;
		}
		pYuvPathInfo->trim1_info.trim_start_x = 0;
		pYuvPathInfo->trim1_info.trim_start_y = 0;
		pYuvPathInfo->trim1_info.trim_size_x = cur_width;
		pYuvPathInfo->trim1_info.trim_size_y = cur_height;

		/*init rotation info*/
		pYuvPathInfo->rotation_info.rot_dir
			= ROTATE_MODE_MAP[pYuvPathInfo->rotation_info.
				rot_parse];
		pYuvPathInfo->rotation_info.rot_en
			= (pYuvPathInfo->rotation_info.rot_parse > 0
			&& pYuvPathInfo->rotation_info.rot_parse <= 3);
	}
exit:
	vfree(y_hor_coef);
	vfree(c_ver_coef);
	vfree(y_ver_coef);
	vfree(buf_coef);
}

int isp_set_scaler_store_cap_reg(struct isp_drv_ynr_info *param)
{
	unsigned  int isp_reg = param->isp_reg_base;
	unsigned int val, tmp;
	signed int *coef0 = vmalloc(sizeof(signed int) * 32);
	signed int *coef1 = vmalloc(sizeof(signed int) * 16);
	signed int *coef2 = vmalloc(sizeof(signed int) * 32);
	signed int *coef21 = vmalloc(sizeof(signed int) * 132);
	signed int *coef3 = vmalloc(sizeof(signed int) * 32);
	signed int *coef31 = vmalloc(sizeof(signed int) * 132);
	int i;
	struct coef_info *pCoef = NULL;
	unsigned int scaler_y_ver_tap;
	unsigned int scaler_uv_ver_tap;
	unsigned int scaler_ip_int;
	unsigned int scaler_ip_rmd;
	unsigned int scaler_cip_int;
	unsigned int scaler_cip_rmd;
	unsigned int scaler_factor_in;
	unsigned int scaler_factor_out;
	unsigned int scaler_ip_int_ver;
	unsigned int scaler_ip_rmd_ver;
	unsigned int scaler_cip_int_ver;
	unsigned int scaler_cip_rmd_ver;
	unsigned int scaler_factor_in_ver;
	unsigned int scaler_factor_out_ver;
	unsigned int h_coeff_addr = 0;
	unsigned int v_coeff_addr = 0;
	unsigned int v_chroma_coeff_addr = 0;
	unsigned int *h_coeff = NULL;
	unsigned int *v_coeff = NULL;
	unsigned int *v_chroma_coeff = NULL;
	struct yuv_info YuvInfo;
	struct yuv_path_info *pYuvPathInfo = &(YuvInfo.yuv_path);

	YuvInfo.scaler_id = 3;
	pYuvPathInfo->yuv_scaler_bypass = 0;
	pYuvPathInfo->yuv_output_format = 1;
	pYuvPathInfo->rotation_info.rot_parse = 0;
	pYuvPathInfo->trim0_info.trim_eb = 0;
	pYuvPathInfo->trim0_info.trim_size_x = param->src_img_w;
	pYuvPathInfo->trim0_info.trim_size_y = param->src_img_h;
	pYuvPathInfo->trim0_info.trim_start_x = 0;
	pYuvPathInfo->trim0_info.trim_start_y = 0;
	pYuvPathInfo->deci_info.deci_x_en = 0;
	pYuvPathInfo->deci_info.deci_x_parse = 0;
	pYuvPathInfo->deci_info.deci_y_en = 0;
	pYuvPathInfo->deci_info.deci_y_parse = 0;
	pYuvPathInfo->scaler_info.scaler_bypass = 0;
	pYuvPathInfo->scaler_info.scaler_out_height = param->dst_img_h;
	pYuvPathInfo->scaler_info.scaler_out_width = param->dst_img_w;
	InitFrameInfo(pYuvPathInfo);

	scaler_y_ver_tap = pYuvPathInfo->scaler_info.scaler_y_ver_tap;
	scaler_uv_ver_tap = pYuvPathInfo->scaler_info.scaler_uv_ver_tap;
	scaler_ip_int = pYuvPathInfo->scaler_info.init_phase_info.
		scaler_init_phase_int[0][0];
	scaler_ip_rmd = pYuvPathInfo->scaler_info.init_phase_info.
		scaler_init_phase_rmd[0][0];
	scaler_cip_int = pYuvPathInfo->scaler_info.init_phase_info.
		scaler_init_phase_int[0][1];
	scaler_cip_rmd = pYuvPathInfo->scaler_info.init_phase_info.
		scaler_init_phase_rmd[0][1];
	scaler_factor_in = pYuvPathInfo->scaler_info.scaler_factor_in_hor;
	scaler_factor_out = pYuvPathInfo->scaler_info.scaler_factor_out_hor;
	scaler_ip_int_ver = pYuvPathInfo->scaler_info.init_phase_info.
		scaler_init_phase_int[1][0];
	scaler_ip_rmd_ver = pYuvPathInfo->scaler_info.init_phase_info.
		scaler_init_phase_rmd[1][0];
	scaler_cip_int_ver = pYuvPathInfo->scaler_info.init_phase_info.
		scaler_init_phase_int[1][1];
	scaler_cip_rmd_ver = pYuvPathInfo->scaler_info.init_phase_info.
		scaler_init_phase_rmd[1][1];
	scaler_factor_in_ver = pYuvPathInfo->scaler_info.scaler_factor_in_ver;
	scaler_factor_out_ver = pYuvPathInfo->scaler_info.
		scaler_factor_out_ver;

	reg_or(isp_reg + ISP_REG_LEN + ISP_SCALER_CAP_BASE
		+ ISP_SCALER_FRAME_CNT_CLR, BIT_0); /*frame counter clear*/

	val = reg_rd(isp_reg + ISP_REG_LEN + ISP_SCALER_CAP_CFG);
	tmp = (~val >> 30) & 1;
	val = 0x80000040 | (tmp << 30) | ((scaler_y_ver_tap & 0xF) << 16)
		| ((scaler_uv_ver_tap & 0x1F) << 11);
	reg_set(isp_reg + ISP_REG_LEN + ISP_SCALER_CAP_CFG, val);
	val = (param->src_img_w & 0xFFFF) |
		((param->src_img_h & 0xFFFF) << 16);
	reg_set(isp_reg + ISP_REG_LEN + ISP_SCALER_CAP_BASE
		+ ISP_SCALER_SRC_SIZE, val);
	val = (param->dst_img_w & 0xFFFF) |
		((param->dst_img_h & 0xFFFF) << 16);
	reg_set(isp_reg + ISP_REG_LEN + ISP_SCALER_CAP_BASE
		+ ISP_SCALER_DES_SIZE, val);
	val = 0;
	reg_set(isp_reg + ISP_REG_LEN + ISP_SCALER_CAP_BASE
		+ ISP_SCALER_TRIM0_START, val);
	val = (param->src_img_w & 0xFFFF) |
		((param->src_img_h & 0xFFFF) << 16);
	reg_set(isp_reg + ISP_REG_LEN + ISP_SCALER_CAP_BASE
		+ ISP_SCALER_TRIM0_SIZE, val);
	val = 0;
	reg_set(isp_reg + ISP_REG_LEN + ISP_SCALER_CAP_BASE
		+ ISP_SCALER_TRIM1_START, val);
	val = (param->dst_img_w & 0xFFFF) |
		((param->dst_img_h & 0xFFFF) << 16);
	reg_set(isp_reg + ISP_REG_LEN + ISP_SCALER_CAP_BASE
		+ ISP_SCALER_TRIM1_SIZE, val);
	val = ((scaler_ip_int & 0xF) << 16) | (scaler_ip_rmd & 0x3FFF);
	reg_set(isp_reg + ISP_REG_LEN + ISP_SCALER_CAP_BASE + ISP_SCALER_IP,
		val);
	val = ((scaler_cip_int & 0xF) << 16) | (scaler_cip_rmd & 0x3FFF);
	reg_set(isp_reg + ISP_REG_LEN + ISP_SCALER_CAP_BASE +
		ISP_SCALER_CIP, val);
	val = ((scaler_factor_in & 0x3FFF) << 16) |
		(scaler_factor_out & 0x3FFF);
	reg_set(isp_reg + ISP_REG_LEN + ISP_SCALER_CAP_BASE
		+ ISP_SCALER_FACTOR, val);
	val = ((scaler_ip_int_ver & 0xF) << 16) | (scaler_ip_rmd_ver & 0x3FFF);
	reg_set(isp_reg + ISP_REG_LEN + ISP_SCALER_CAP_BASE
		+ ISP_SCALER_VER_IP, val);
	val = ((scaler_cip_int_ver & 0xF) << 16) |
		(scaler_cip_rmd_ver & 0x3FFF);
	reg_set(isp_reg + ISP_REG_LEN + ISP_SCALER_CAP_BASE
		+ ISP_SCALER_VER_CIP, val);
	val = ((scaler_factor_in_ver & 0x3FFF) << 16)
		| (scaler_factor_out_ver & 0x3FFF);
	reg_set(isp_reg + ISP_REG_LEN + ISP_SCALER_CAP_BASE
		+ ISP_SCALER_VER_FACTOR, val);

	val = 1;
	reg_set(isp_reg + ISP_REG_LEN + ISP_SCALER_CAP_BASE
		+ ISP_SCALER_DEBUG, val);

	pCoef = &pYuvPathInfo->scaler_info.scaler_coef_info;

	h_coeff = pCoef->y_hor_coef;
	v_coeff = pCoef->y_ver_coef;
	v_chroma_coeff = pCoef->c_ver_coef;

	h_coeff_addr = isp_reg + ISP_REG_LEN + ISP_SCALER_CAP_BASE
			+ ISP_SCALER_LUMA_HCOEFF;
	v_coeff_addr = isp_reg + ISP_REG_LEN + ISP_SCALER_CAP_BASE
				+ ISP_SCALER_LUMA_VCOEFF;
	v_chroma_coeff_addr = isp_reg + ISP_REG_LEN + ISP_SCALER_CAP_BASE
				+ ISP_SCALER_CHROMA_VCOEFF;

	for (i = 0; i < 48; i++) {
		reg_set(h_coeff_addr, *h_coeff);
		h_coeff_addr += 4;
		h_coeff++;
	}

	for (i = 0; i < 132; i++) {
		reg_set(v_coeff_addr, *v_coeff);
		v_coeff_addr += 4;
		v_coeff++;
	}

	for (i = 0; i < 132; i++) {
		reg_set(v_chroma_coeff_addr, *v_chroma_coeff);
		v_chroma_coeff_addr += 4;
		v_chroma_coeff++;
	}

	val = 0x54;
	reg_set(isp_reg + ISP_REG_LEN + ISP_STORE_CAPTURE_PARAM, val);
	val = param->dst_img_w;
	reg_set(isp_reg + ISP_REG_LEN + ISP_STORE_CAPTURE_Y_PITCH, val);
	reg_set(isp_reg + ISP_REG_LEN + ISP_STORE_CAPTURE_U_PITCH, val);
	val = param->dst_buf_addr;
	reg_set(isp_reg + ISP_REG_LEN + ISP_STORE_CAPTURE_Y_ADDR, val);
	val = param->dst_buf_addr + param->dst_img_w * param->dst_img_h;
	reg_set(isp_reg + ISP_REG_LEN + ISP_STORE_CAPTURE_U_ADDR, val);
	val = 0;
	reg_set(isp_reg + ISP_REG_LEN + ISP_STORE_CAPTURE_V_ADDR, val);
	val = (param->dst_img_w & 0xFFFF) |
		((param->dst_img_h & 0xFFFF) << 16);
	reg_set(isp_reg + ISP_REG_LEN + ISP_STORE_CAPTURE_SLICE_SIZE, val);
	val = 0;
	reg_set(isp_reg + ISP_REG_LEN + ISP_STORE_CAPTURE_BORDER, val);
	val = 2;
	reg_set(isp_reg + ISP_REG_LEN + ISP_STORE_CAPTURE_SHADOW_CLR_SEL, val);
	val = 1;
	reg_set(isp_reg + ISP_REG_LEN + ISP_STORE_CAPTURE_SHADOW_CLR, val);

	vfree(coef0);
	vfree(coef1);
	vfree(coef2);
	vfree(coef21);
	vfree(coef3);
	vfree(coef31);

	return 0;
}

int isp_ynr_process_one_frame(struct isp_drv_ynr_info *param)
{
	pr_debug("in.\n");

	isp_set_common_reg(param);
	isp_set_ynr_reg(param);
	isp_set_scaler_store_cap_reg(param);
	isp_start_fetch(param);

	pr_debug("done.\n");

	return 0;
}

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
#define pr_fmt(fmt) "PDAF: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

static int isp_k_pdaf_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	int i = 0;
	unsigned int val = 0;
	unsigned long dst_addr_right = 0;
	unsigned long dst_addr_left = 0;
	void *data_ptr_left = NULL;
	void *data_ptr_right = NULL;
	struct isp_dev_pdaf_info pdaf_info;

	memset(&pdaf_info, 0x00, sizeof(pdaf_info));
	ret = copy_from_user((void *)&pdaf_info,
		param->property_param, sizeof(pdaf_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}
	ISP_REG_MWR(idx, ISP_PDAF_PARAM, BIT_0, pdaf_info.bypass);
	if (pdaf_info.bypass) {
		ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL1,
				BIT_0, 1);
		return 0;
	}

	ISP_REG_MWR(idx, ISP_PDAF_PARAM,
			BIT_2, pdaf_info.corrector_bypass << 2);
	ISP_REG_MWR(idx, ISP_PDAF_PARAM,
			BIT_3, pdaf_info.phase_map_corr_en << 3);
	ISP_REG_MWR(idx, ISP_PDAF_PARAM, BIT_8, pdaf_info.grid_mode << 8);

	val = ((pdaf_info.block_size.height & 0x3) << 6)
		| ((pdaf_info.block_size.width & 0x3) << 4);
	ISP_REG_MWR(idx, ISP_PDAF_PARAM, 0xF0, val);

	val = ((pdaf_info.win.end_x & 0xFFFF) << 16) |
		(pdaf_info.win.start_x & 0xFFFF);
	ISP_REG_WR(idx, ISP_PDAF_WIN_X, val);

	val = ((pdaf_info.win.end_y & 0xFFFF) << 16) |
		(pdaf_info.win.start_y & 0xFFFF);
	ISP_REG_WR(idx, ISP_PDAF_WIN_Y, val);

	val = ((pdaf_info.block.end_x & 0xFFFF) << 16) |
		(pdaf_info.block.start_x & 0xFFFF);
	ISP_REG_WR(idx, ISP_PDAF_BLOCK_COL, val);

	val = ((pdaf_info.block.end_y & 0xFFFF) << 16) |
		(pdaf_info.block.start_y & 0xFFFF);
	ISP_REG_WR(idx, ISP_PDAF_BLOCK_ROW, val);

	val = ((pdaf_info.gain_upperbound.gr & 0x3FF) << 16) |
		(pdaf_info.gain_upperbound.gb & 0x3FF);
	ISP_REG_WR(idx, ISP_PDAF_GAIN_PARA0, val);

	val = ((pdaf_info.gain_upperbound.r & 0x3FF) << 16) |
		(pdaf_info.gain_upperbound.b & 0x3FF);
	ISP_REG_WR(idx, ISP_PDAF_GAIN_PARA1, val);

	val = ((pdaf_info.phase_flat_smoother & 0x7) << 5) |
		((pdaf_info.phase_gfilter & 0x1) << 4) |
		(pdaf_info.phase_txt_smooth & 0xF);
	ISP_REG_WR(idx, ISP_PDAF_PHASE_BPC_PARA, val);

	val = ((pdaf_info.hot_pixel_th[1] & 0x3FF) << 16) |
		(pdaf_info.hot_pixel_th[0] & 0x3FF);
	ISP_REG_WR(idx, ISP_PDAF_HOT_PIXEL_TH1, val);
	ISP_REG_WR(idx, ISP_PDAF_HOT_PIXEL_TH2,
			pdaf_info.hot_pixel_th[2] & 0x3FF);

	val = ((pdaf_info.dead_pixel_th[1] & 0x3FF) << 16) |
		(pdaf_info.dead_pixel_th[0] & 0x3FF);
	ISP_REG_WR(idx, ISP_PDAF_DEAD_PIXEL_TH1, val);
	ISP_REG_WR(idx, ISP_PDAF_DEAD_PIXEL_TH2,
			pdaf_info.dead_pixel_th[2] & 0x3FF);
	ISP_REG_WR(idx, ISP_PDAF_FLAT_TH, pdaf_info.flat_th & 0x3FF);

	val = ((pdaf_info.edge_ratio_hv_rd & 0x1FF) << 20) |
		((pdaf_info.edge_ratio_rd & 0x1FF) << 10) |
		(pdaf_info.edge_ratio_hv & 0x1FF);
	ISP_REG_WR(idx, ISP_PDAF_EDGE_RATIO, val);
	ISP_REG_WR(idx, ISP_PDAF_PHASE_LEFT_ADDR, pdaf_info.phase_left_addr);
	ISP_REG_WR(idx, ISP_PDAF_PHASE_RIGHT_ADDR, pdaf_info.phase_right_addr);
	ISP_REG_WR(idx, ISP_PDAF_PHASE_PITCH, pdaf_info.phase_pitch & 0xFFFF);

	for (i = 0; i < PDAF_PPI_NUM; i += 2) {
		val = ((pdaf_info.pattern_pixel_is_right[i+1] & 0x1)  << 28) |
			((pdaf_info.pattern_pixel_row[i+1] & 0x3F) << 22) |
			((pdaf_info.pattern_pixel_col[i+1] & 0x3F) << 16) |
			((pdaf_info.pattern_pixel_is_right[i] & 0x1)  << 12) |
			((pdaf_info.pattern_pixel_row[i] & 0x3F) << 6) |
			(pdaf_info.pattern_pixel_col[i] & 0x3F);
		ISP_REG_WR(idx, ISP_PDAF_PATTERN01 + 4 * (i>>1), val);
	}
	val = ((pdaf_info.gain_ori_right[0] & 0x3FFF) << 16) |
		(pdaf_info.gain_ori_left[0]  & 0x3FFF);
	ISP_REG_WR(idx, ISP_PDAF_GAIN_ORI0, val);

	val = ((pdaf_info.gain_ori_right[1] & 0x3FFF) << 16) |
		(pdaf_info.gain_ori_left[1]  & 0x3FFF);
	ISP_REG_WR(idx, ISP_PDAF_GAIN_ORI1, val);
	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL, BIT_0, pdaf_info.extractor_bypass);
	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL, BIT_1, pdaf_info.mode_sel << 1);
	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL, 0x3C, pdaf_info.skip_num << 2);

	val = ((pdaf_info.pdaf_blc.b & 0x3FF) << 10)
		| (pdaf_info.pdaf_blc.r & 0x3FF);
	ISP_REG_WR(idx, ISP_PDAF_BLC_PARA0, val);

	val = ((pdaf_info.pdaf_blc.gb & 0x3FF) << 10)
		|(pdaf_info.pdaf_blc.gr & 0x3FF);
	ISP_REG_WR(idx, ISP_PDAF_BLC_PARA1, val);
	ISP_REG_WR(idx, ISP_PDAF_REQ_DWORD_NUM, pdaf_info.phase_data_dword_num);
	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL2, BIT_0, 1);

	dst_addr_left = ISP_BASE_ADDR(idx) + ISP_PDAF_GAIN_LEFT;
	dst_addr_right = ISP_BASE_ADDR(idx) + ISP_PDAF_GAIN_RIGHT;

#ifdef CONFIG_64BIT
	data_ptr_left =
		(void *)(((unsigned long)pdaf_info.data_ptr_left[1] << 32)
					| pdaf_info.data_ptr_left[0]);
#else
	data_ptr_left = (void *)(pdaf_info.data_ptr_left[0]);
#endif

	ret = copy_from_user((void *)dst_addr_left,
			data_ptr_left, 128 * sizeof(unsigned int));

#ifdef CONFIG_64BIT
	data_ptr_right =
		(void *)(((unsigned long)pdaf_info.data_ptr_right[1] << 32)
					| pdaf_info.data_ptr_right[0]);
#else
	data_ptr_right = (void *)(pdaf_info.data_ptr_right[0]);
#endif

	ret = copy_from_user((void *)dst_addr_right,
		data_ptr_right, 128 * sizeof(unsigned int));

	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}


	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL1, BIT_0, 1);

	return ret;
}

static int isp_k_pdaf_bypass(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int bypass = 0;

	ret = copy_from_user((void *)&bypass,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	if (bypass)
		ISP_REG_OWR(idx, ISP_PDAF_PARAM, BIT_0);
	else
		ISP_REG_MWR(idx, ISP_PDAF_PARAM, BIT_0, 0);
	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL1, BIT_0, 1);

	return ret;
}

int isp_k_pdaf_cfg_addr(unsigned long phy_addr, int buf_size, enum isp_id idx)
{
	int ret = 0;

	if (phy_addr == 0) {
		pr_err("fail to get phy_addr\n");
		return 1;
	}
	ISP_REG_WR(idx, ISP_PDAF_PHASE_LEFT_ADDR, phy_addr);
	ISP_REG_WR(idx, ISP_PDAF_PHASE_RIGHT_ADDR, phy_addr + buf_size/2);

	return ret;
}

static int isp_k_pdaf_set_ppi_info(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	int i = 0;
	struct pdaf_ppi_info pdaf_info;

	memset(&pdaf_info, 0x00, sizeof(pdaf_info));
	ret = copy_from_user((void *)&pdaf_info,
		param->property_param, sizeof(pdaf_info));
	if (ret !=  0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}
	val = ((pdaf_info.block_size.height & 0x3) << 6)
		| ((pdaf_info.block_size.width & 0x3) << 4);
	ISP_REG_MWR(idx, ISP_PDAF_PARAM, 0xF0, val);

	val = ((pdaf_info.block.end_x & 0xFFFF) << 16) |
	(pdaf_info.block.start_x & 0xFFFF);
	ISP_REG_WR(idx, ISP_PDAF_BLOCK_COL, val);

	val = ((pdaf_info.block.end_y & 0xFFFF) << 16) |
		(pdaf_info.block.start_y & 0xFFFF);
	ISP_REG_WR(idx, ISP_PDAF_BLOCK_ROW, val);
	for (i = 0; i < PDAF_PPI_NUM; i += 2) {
		val = ((pdaf_info.pattern_pixel_is_right[i+1] & 0x1)  << 28) |
			((pdaf_info.pattern_pixel_row[i+1] & 0x3F) << 22) |
			((pdaf_info.pattern_pixel_col[i+1] & 0x3F) << 16) |
			((pdaf_info.pattern_pixel_is_right[i] & 0x1)  << 12) |
			((pdaf_info.pattern_pixel_row[i] & 0x3F) << 6) |
			(pdaf_info.pattern_pixel_col[i] & 0x3F);
		ISP_REG_WR(idx, ISP_PDAF_PATTERN01 + 4 * (i>>1), val);
	}

	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL1, BIT_0, 1);

	return ret;
}

static int isp_k_pdaf_set_roi(struct isp_io_param *param, enum isp_id idx)
{

	int ret = 0;
	unsigned int val = 0;
	struct pdaf_roi_info roi_info;

	memset(&roi_info, 0x00, sizeof(roi_info));
	ret = copy_from_user((void *)&roi_info,
		param->property_param, sizeof(roi_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}
	val = ((roi_info.win.end_x & 0xFFFF) << 16) |
	(roi_info.win.start_x & 0xFFFF);
	ISP_REG_WR(idx, ISP_PDAF_WIN_X, val);

	val = ((roi_info.win.end_y & 0xFFFF) << 16) |
		(roi_info.win.start_y & 0xFFFF);
	ISP_REG_WR(idx, ISP_PDAF_WIN_Y, val);
	ISP_REG_WR(idx, ISP_PDAF_REQ_DWORD_NUM, roi_info.phase_data_write_num);

	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL1, BIT_0, 1);

	return ret;
}

static int isp_k_pdaf_set_skip_num(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int skip_num = 0;

	ret = copy_from_user((void *)&skip_num,
			param->property_param, sizeof(skip_num));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL, 0x3C, skip_num << 2);
	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL1, BIT_0, 1);

	return ret;
}

static int isp_k_pdaf_set_mode(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int work_mode = 0;

	ret = copy_from_user((void *)&work_mode,
			param->property_param, sizeof(work_mode));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL, BIT_1, work_mode << 1);
	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL1, BIT_0, 1);

	return ret;
}

static int isp_k_pdaf_set_extractor_bypass(struct isp_io_param *param,
				enum isp_id idx)
{
	int ret = 0;
	unsigned int extractor_bypass = 0;

	ret = copy_from_user((void *)&extractor_bypass,
			param->property_param, sizeof(extractor_bypass));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL, BIT_0, extractor_bypass);
	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL1, BIT_0, 1);

	return ret;
}

int isp_k_cfg_pdaf(struct isp_io_param *param, enum isp_id idx)
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
	case ISP_PRO_PDAF_BLOCK:
		ret = isp_k_pdaf_block(param, idx);
		break;
	case ISP_PRO_PDAF_BYPASS:
		ret = isp_k_pdaf_bypass(param, idx);
		break;
	case ISP_PRO_PDAF_SET_MODE:
		ret = isp_k_pdaf_set_mode(param, idx);
		break;
	case ISP_PRO_PDAF_SET_SKIP_NUM:
		ret = isp_k_pdaf_set_skip_num(param, idx);
		break;
	case ISP_PRO_PDAF_SET_ROI:
		ret = isp_k_pdaf_set_roi(param, idx);
		break;
	case ISP_PRO_PDAF_SET_PPI_INFO:
		ret = isp_k_pdaf_set_ppi_info(param, idx);
		break;
	case ISP_PRO_PDAF_SET_EXTRACTOR_BYPASS:
		ret = isp_k_pdaf_set_extractor_bypass(param, idx);
		break;
	default:
		pr_err("fail to support cmd id = %d\n",
			param->property);
		break;
	}

	return ret;
}

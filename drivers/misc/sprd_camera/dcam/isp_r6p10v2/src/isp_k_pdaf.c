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
#include "../isp_block.h"
#include "../isp_drv.h"

static int isp_k_pdaf_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	int i = 0;
	unsigned int val = 0;
	struct isp_dev_pdaf_info pdaf_info;

	memset(&pdaf_info, 0x00, sizeof(pdaf_info));
	ret = copy_from_user((void *)&pdaf_info,
		param->property_param, sizeof(pdaf_info));
	if (ret != 0) {
		pr_err("isp_k_pdaf_block: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	ISP_REG_MWR(idx, ISP_PDAF_PARAM, BIT_0, pdaf_info.bypass);
	if (pdaf_info.bypass) {
		ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL1,
				BIT_0, 1);
		return 0;
	}

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
	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL, BIT_0, pdaf_info.extractor_bypass);
	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL, BIT_1, pdaf_info.mode_sel << 1);
	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL, 0x3C, pdaf_info.skip_num << 2);

	ISP_REG_WR(idx, ISP_PDAF_REQ_DWORD_NUM, pdaf_info.phase_data_dword_num);
	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL2, BIT_0, 1);


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
		pr_err("isp_k_pdaf_bypass: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (bypass)
		ISP_REG_OWR(idx, ISP_PDAF_PARAM, BIT_0);
	else
		ISP_REG_MWR(idx, ISP_PDAF_PARAM, BIT_0, 0);
	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL1, BIT_0, 1);

	return ret;
}

static int isp_k_pdaf_skip_num_clr
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int clr = 0;

	ret = copy_from_user((void *)&clr,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_pdaf_skip_num_clr: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_WR(idx, ISP_PDAF_AF_CTRL2, clr);
	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL1, BIT_0, 1);

	return ret;
}

static int isp_k_pdaf_addr(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	struct pdaf_addr_info pdaf_addr;

	memset(&pdaf_addr, 0x00, sizeof(pdaf_addr));

	ret = copy_from_user((void *)&pdaf_addr,
		param->property_param, sizeof(struct pdaf_addr_info));
	if (ret != 0) {
		pr_err("isp_k_pdaf_addr: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_WR(idx, ISP_PDAF_PHASE_LEFT_ADDR, pdaf_addr.addr_l);
	ISP_REG_WR(idx, ISP_PDAF_PHASE_RIGHT_ADDR, pdaf_addr.addr_r);
	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL1, BIT_0, 1);

	return ret;
}

static int isp_k_pdaf_cfg_rdy(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_pdaf_cfg_rdy: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL1, BIT_0, val);

	return ret;
}

int isp_k_pdaf_cfg_addr(unsigned long phy_addr, int buf_size, enum isp_id idx)
{
	if (phy_addr == 0) {
		pr_err("isp_k_ra_aem_cfg_addr: Invalid address\n");
		return 1;
	}
	ISP_REG_WR(idx, ISP_PDAF_PHASE_LEFT_ADDR, phy_addr);
	ISP_REG_WR(idx, ISP_PDAF_PHASE_RIGHT_ADDR, phy_addr + buf_size/2);

	return 0;
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
		pr_err("isp_k_pdaf_set_ppi_info: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
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
		pr_err("isp_k_pdaf_set_roi: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
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
		pr_err("isp_k_pdaf_set_skip_num: error, ret = 0x%x\n",
				(unsigned int)ret);
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
		pr_err("isp_k_pdaf_set_mode: error, ret = 0x%x\n",
				(unsigned int)ret);
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
		pr_err("isp_k_pdaf_set_extractor_bypass: error, ret = 0x%x\n",
				(unsigned int)ret);
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
		pr_err("isp_k_cfg_pdaf: param is null error.\n");
		return -1;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_pdaf: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_PDAF_BLOCK:
		ret = isp_k_pdaf_block(param, idx);
		break;
	case ISP_PRO_PDAF_BYPASS:
		ret = isp_k_pdaf_bypass(param, idx);
		break;
	case ISP_PRO_PDAF_SKIP_NUM_CLR:
		ret = isp_k_pdaf_skip_num_clr(param, idx);
		break;
	case ISP_PRO_PDAF_CFG_RDY:
		ret = isp_k_pdaf_cfg_rdy(param, idx);
		break;
	case ISP_PRO_PDAF_ADDR:
		ret = isp_k_pdaf_addr(param, idx);
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
		pr_err("isp_k_cfg_pdaf: fail cmd id:%d, not supported.\n",
			param->property);
		break;
	}

	return ret;
}

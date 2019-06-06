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
#include "../isp_buf.h"

static int isp_k_fetch_raw_block(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, enum isp_id idx)
{
	int ret = 0;
	struct isp_dev_fetch_info fetch_info;
	unsigned int size = 0;

	memset(&fetch_info, 0x00, sizeof(fetch_info));
	ret = copy_from_user((void *)&fetch_info,
		param->property_param, sizeof(fetch_info));
	if (ret != 0) {
		pr_err("isp_k_fetch_raw_block: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	fetch_info.addr.chn0 = isp_k_param->fetch_raw_phys_addr;

	ISP_REG_MWR(idx, ISP_FETCH_PARAM, BIT_0, fetch_info.bypass);
	if (fetch_info.bypass)
		return 0;

	if (fetch_info.subtract)
		ISP_REG_OWR(idx, ISP_FETCH_PARAM, BIT_1);
	else
		ISP_REG_MWR(idx, ISP_FETCH_PARAM, BIT_1, 0);

	ISP_REG_MWR(idx, ISP_FETCH_PARAM,
		0xF0, (fetch_info.color_format << 4));

	size = ((fetch_info.size.height & 0xFFFF) << 16)
		| (fetch_info.size.width & 0xFFFF);
	ISP_REG_WR(idx, ISP_FETCH_SLICE_SIZE, size);

	ISP_REG_WR(idx, ISP_FETCH_SLICE_Y_ADDR,
		fetch_info.addr.chn0);

	ISP_REG_WR(idx, ISP_FETCH_SLICE_Y_PITCH,
		(fetch_info.pitch.chn0 & 0xFFFF));

	ISP_REG_WR(idx, ISP_FETCH_SLICE_U_ADDR,
		fetch_info.addr.chn1);

	ISP_REG_WR(idx, ISP_FETCH_SLICE_U_PITCH,
		(fetch_info.pitch.chn1 & 0xFFFF));

	ISP_REG_WR(idx, ISP_FETCH_SLICE_V_ADDR,
		fetch_info.addr.chn2);

	ISP_REG_WR(idx, ISP_FETCH_SLICE_V_PITCH,
		(fetch_info.pitch.chn2 & 0xFFFF));

	ISP_REG_WR(idx, ISP_FETCH_MIPI_WORD_INFO,
		(fetch_info.mipi_word_num & 0xFFFF));

	ISP_REG_WR(idx, ISP_FETCH_MIPI_BYTE_INFO,
		(fetch_info.mipi_byte_rel_pos & 0x0F));

	return ret;
}

static int isp_k_fetch_yuv_block
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int size = 0;
	struct isp_dev_fetch2_info fetch2_info;

	memset(&fetch2_info, 0x00, sizeof(fetch2_info));
	ret = copy_from_user((void *)&fetch2_info,
		param->property_param, sizeof(fetch2_info));
	if (ret != 0) {
		pr_err("isp_k_fetch_yuv_block: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	ISP_REG_MWR(idx, ISP_FETCH2_PARAM, BIT_0, fetch2_info.bypass);

	if (fetch2_info.bypass)
		return 0;

	if (fetch2_info.subtract)
		ISP_REG_OWR(idx, ISP_FETCH2_PARAM, BIT_1);
	else
		ISP_REG_MWR(idx, ISP_FETCH2_PARAM, BIT_1, 0);

	ISP_REG_MWR(idx, ISP_FETCH2_PARAM, 0xF0,
			(fetch2_info.color_format << 4));

	ISP_REG_MWR(idx, ISP_FETCH2_PARAM, BIT_8,
			(fetch2_info.ft0_axi_reorder_en << 8));

	ISP_REG_MWR(idx, ISP_FETCH2_PARAM, BIT_9,
			(fetch2_info.ft1_axi_reorder_en << 9));
	ISP_REG_MWR(idx, ISP_FETCH2_PARAM, BIT_10,
			(fetch2_info.ft2_axi_reorder_en << 10));

	ISP_REG_MWR(idx, ISP_FETCH2_PARAM, BIT_11,
			(fetch2_info.chk_sum_clr_en << 11));

	ISP_REG_MWR(idx, ISP_FETCH2_PARAM, BIT_12,
			(fetch2_info.first_line_mode << 12));

	ISP_REG_MWR(idx, ISP_FETCH2_PARAM, BIT_13,
			(fetch2_info.last_line_mode << 13));

	size = ((fetch2_info.size.height & 0xFFFF) << 16)
			| (fetch2_info.size.width & 0xFFFF);
	ISP_REG_WR(idx, ISP_FETCH2_SLICE_SIZE, size);

	ISP_REG_WR(idx, ISP_FETCH2_SLICE_Y_ADDR,
		fetch2_info.addr.chn0);

	ISP_REG_WR(idx, ISP_FETCH2_SLICE_Y_PITCH,
		fetch2_info.pitch.chn0);

	ISP_REG_WR(idx, ISP_FETCH2_SLICE_U_ADDR,
		fetch2_info.addr.chn1);

	ISP_REG_WR(idx, ISP_FETCH2_SLICE_U_PITCH,
		fetch2_info.pitch.chn1);

	ISP_REG_WR(idx, ISP_FETCH2_SLICE_V_ADDR,
		fetch2_info.addr.chn2);

	ISP_REG_WR(idx, ISP_FETCH2_SLICE_V_PITCH,
		fetch2_info.pitch.chn2);

	return ret;
}

static int isp_k_fetch_bypass(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int bypass = 0;
	unsigned int addr = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_FETCH:
		addr = ISP_FETCH_PARAM;
		break;
	case ISP_BLOCK_YUV_FETCH:
		addr = ISP_FETCH2_PARAM;
		break;
	default:
		pr_err("isp_k_fetch_bypass: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&bypass,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_fetch_bypass: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (bypass)
		ISP_REG_OWR(idx, addr, BIT_0);
	else
		ISP_REG_MWR(idx, addr, BIT_0, 0);

	return ret;
}

static int isp_k_fetch_subtract(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int subtract = 0;
	unsigned int addr = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_FETCH:
		addr = ISP_FETCH_PARAM;
		break;
	case ISP_BLOCK_YUV_FETCH:
		addr = ISP_FETCH2_PARAM;
		break;
	default:
		pr_err("isp_k_fetch_subtract: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&subtract,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_fetch_subtract: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (subtract)
		ISP_REG_OWR(idx, addr, BIT_1);
	else
		ISP_REG_MWR(idx, addr, BIT_1, 0);

	return ret;
}

static int isp_k_fetch_color_format
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int color_format = 0;
	unsigned int addr = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_FETCH:
		addr = ISP_FETCH_PARAM;
		break;
	case ISP_BLOCK_YUV_FETCH:
		addr = ISP_FETCH2_PARAM;
		break;
	default:
		pr_err("isp_k_fetch_color_format: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&color_format,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_fetch_color_format: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	ISP_REG_MWR(idx, addr, 0xF << 4, color_format << 4);

	return ret;
}

static int isp_k_fetch_start(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int start = 0;
	unsigned int addr = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_FETCH:
		addr = ISP_FETCH_START;
		break;
	case ISP_BLOCK_YUV_FETCH:
		addr = ISP_FETCH2_START;
		break;
	default:
		pr_err("isp_k_fetch_start: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&start,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_fetch_start: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	pr_info("fetch raw addr %x start %d\n", addr, start);
	ISP_REG_WR(idx, addr, start);

	return ret;
}

static int isp_k_fetch_slice_size(struct isp_io_param *param,
							enum isp_id idx)
{
	int ret = 0;
	unsigned int size = 0;
	struct isp_img_size slice_size = {0, 0};
	unsigned int addr = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_FETCH:
		addr = ISP_FETCH_SLICE_SIZE;
		break;
	case ISP_BLOCK_YUV_FETCH:
		addr = ISP_FETCH2_SLICE_SIZE;
		break;
	default:
		pr_err("isp_k_fetch_slice_size: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&slice_size,
		param->property_param, sizeof(struct isp_img_size));
	if (ret != 0) {
		pr_err("isp_k_fetch_slice_size: read copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	size = ((slice_size.height & 0xFFFF) << 16)
		| (slice_size.width & 0xFFFF);

	ISP_REG_WR(idx, addr+ISP_FETCH_SLICE_SIZE, size);

	return ret;
}

static int isp_k_fetch_y_addr(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int y_addr = 0;
	unsigned int addr = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_FETCH:
		addr = ISP_FETCH_SLICE_Y_ADDR;
		break;
	case ISP_BLOCK_YUV_FETCH:
		addr = ISP_FETCH2_SLICE_Y_ADDR;
		break;
	default:
		pr_err("isp_k_fetch_y_addr: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&y_addr,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_fetch_y_addr: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_WR(idx, addr, y_addr);

	return ret;
}

static int isp_k_fetch_y_pitch(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int pitch = 0;
	unsigned int addr = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_FETCH:
		addr = ISP_FETCH_SLICE_Y_PITCH;
		break;
	case ISP_BLOCK_YUV_FETCH:
		addr = ISP_FETCH2_SLICE_Y_PITCH;
		break;
	default:
		pr_err("isp_k_fetch_y_pitch: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&pitch,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_fetch_y_pitch: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_WR(idx, addr, (pitch & 0xFFFF));

	return ret;
}

static int isp_k_fetch_u_addr(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int u_addr = 0;
	unsigned int addr = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_FETCH:
		addr = ISP_FETCH_SLICE_U_ADDR;
		break;
	case ISP_BLOCK_YUV_FETCH:
		addr = ISP_FETCH2_SLICE_U_ADDR;
		break;
	default:
		pr_err("isp_k_fetch_u_addr: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&u_addr,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_fetch_u_addr: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_WR(idx, addr, u_addr);

	return ret;
}

static int isp_k_fetch_u_pitch(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int pitch = 0;
	unsigned int addr = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_FETCH:
		addr = ISP_FETCH_SLICE_U_PITCH;
		break;
	case ISP_BLOCK_YUV_FETCH:
		addr = ISP_FETCH2_SLICE_U_PITCH;
		break;
	default:
		pr_err("isp_k_fetch_u_pitch: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&pitch,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_fetch_u_pitch: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_WR(idx, addr+ISP_FETCH_SLICE_U_PITCH, (pitch & 0xFFFF));

	return ret;
}

static int isp_k_fetch_mipi_word_info
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int info = 0;
	unsigned int addr = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_FETCH:
		addr = ISP_FETCH_MIPI_WORD_INFO;
		break;
	default:
		pr_err("isp_k_fetch_mipi_word_info: fail cmd id:%d, not supported.\n",
			param->sub_block);
		break;
	}

	ret = copy_from_user((void *)&info,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_fetch_mipi_word_info: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_WR(idx, addr, (info & 0xFFFF));

	return ret;
}

static int isp_k_fetch_mipi_byte_info
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int info = 0;
	unsigned int addr = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_FETCH:
		addr = ISP_FETCH_MIPI_BYTE_INFO;
		break;
	default:
		pr_err("isp_k_fetch_mipi_byte_info: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&info,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_fetch_mipi_byte_info: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_WR(idx, addr, (info & 0x0F));

	return ret;
}

static int isp_k_fetch_v_addr(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int v_addr = 0;
	unsigned int addr = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_FETCH:
		addr = ISP_FETCH_SLICE_V_ADDR;
		break;
	case ISP_BLOCK_YUV_FETCH:
		addr = ISP_FETCH2_SLICE_V_ADDR;
		break;
	default:
		pr_err("isp_k_fetch_v_addr: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&v_addr,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_fetch_v_addr: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_WR(idx, addr+ISP_FETCH_SLICE_V_ADDR, v_addr);

	return ret;
}

static int isp_k_fetch_v_pitch(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int pitch = 0;
	unsigned int addr = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_FETCH:
		addr = ISP_FETCH_SLICE_V_PITCH;
		break;
	case ISP_BLOCK_YUV_FETCH:
		addr = ISP_FETCH2_SLICE_V_PITCH;
		break;
	default:
		pr_err("isp_k_fetch_v_pitch: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&pitch,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_fetch_v_pitch: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_WR(idx, addr, (pitch & 0xFFFF));

	return ret;
}

static int isp_k_fetch_transaddr(struct isp_io_param *param,
			struct isp_k_block *isp_k_param, enum isp_id idx)
{
	int ret = 0;
	struct isp_dev_block_addr fetch_buf;
	struct isp_statis_buf fetch_remap_buf;

	memset(&fetch_buf, 0x00, sizeof(struct isp_dev_block_addr));
	memset(&fetch_remap_buf, 0x00, sizeof(struct isp_statis_buf));
	ret = copy_from_user(&fetch_buf, param->property_param,
				sizeof(fetch_buf));

	if (idx == ISP_ID_0)
		fetch_remap_buf.pfinfo.dev = &s_isp_pdev->dev;
	else
		fetch_remap_buf.pfinfo.dev = &s_isp1_pdev->dev;

	fetch_remap_buf.pfinfo.mfd[0] = fetch_buf.img_fd;

	/*mapping iommu buffer*/
	ret = pfiommu_get_sg_table(&fetch_remap_buf.pfinfo);
	if (ret) {
		pr_err("map iommu fetch buffer failed.\n");
		ret = -1;
		return ret;
	}

	ret = pfiommu_get_addr(&fetch_remap_buf.pfinfo);
	if (ret) {
		pr_err("map iommu fetch buffer failed: get address error!\n");
		return -1;
	}

	isp_k_param->fetch_pfinfo.iova[0] =
		fetch_remap_buf.pfinfo.iova[0];
	isp_k_param->fetch_raw_phys_addr = fetch_remap_buf.pfinfo.iova[0]
		+ fetch_buf.img_offset.chn0;
	isp_k_param->fetch_pfinfo.size[0] = fetch_remap_buf.pfinfo.size[0];
	if (idx == ISP_ID_0)
		isp_k_param->fetch_pfinfo.dev = &s_isp_pdev->dev;
	else
		isp_k_param->fetch_pfinfo.dev = &s_isp1_pdev->dev;

	pr_debug("fetch addr 0x%lx, size 0x%x\n",
		isp_k_param->fetch_pfinfo.iova[0],
		(unsigned int)fetch_remap_buf.pfinfo.size[0]);

	return ret;
}

int isp_k_cfg_fetch(struct isp_io_param *param,
			struct isp_k_block *isp_k_param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_fetch: param is null error.\n");
		return -1;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_fetch: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_FETCH_RAW_BLOCK:
		ret = isp_k_fetch_raw_block(param, isp_k_param, idx);
		break;
	case ISP_PRO_FETCH_YUV_BLOCK:
		ret = isp_k_fetch_yuv_block(param, idx);
		break;
	case ISP_PRO_FETCH_BYPASS:
		ret = isp_k_fetch_bypass(param, idx);
		break;
	case ISP_PRO_FETCH_SUBTRACT:
		ret = isp_k_fetch_subtract(param, idx);
		break;
	case ISP_PRO_FETCH_COLOR_FORMAT:
		ret = isp_k_fetch_color_format(param, idx);
		break;
	case ISP_PRO_FETCH_START:
		ret = isp_k_fetch_start(param, idx);
		break;
	case ISP_PRO_FETCH_SLICE_SIZE:
		ret = isp_k_fetch_slice_size(param, idx);
		break;
	case ISP_PRO_FETCH_SLICE_Y_ADDR:
		ret = isp_k_fetch_y_addr(param, idx);
		break;
	case ISP_PRO_FETCH_SLICE_Y_PITCH:
		ret = isp_k_fetch_y_pitch(param, idx);
		break;
	case ISP_PRO_FETCH_SLICE_U_ADDR:
		ret = isp_k_fetch_u_addr(param, idx);
		break;
	case ISP_PRO_FETCH_SLICE_U_PITCH:
		ret = isp_k_fetch_u_pitch(param, idx);
		break;
	case ISP_PRO_FETCH_MIPI_WORD_INFO:
		ret = isp_k_fetch_mipi_word_info(param, idx);
		break;
	case ISP_PRO_FETCH_MIPI_BYTE_INFO:
		ret = isp_k_fetch_mipi_byte_info(param, idx);
		break;
	case ISP_PRO_FETCH_SLICE_V_ADDR:
		ret = isp_k_fetch_v_addr(param, idx);
		break;
	case ISP_PRO_FETCH_SLICE_V_PITCH:
		ret = isp_k_fetch_v_pitch(param, idx);
		break;
	case ISP_PRO_FETCH_TRANSADDR:
		ret = isp_k_fetch_transaddr(param, isp_k_param, idx);
		break;
	default:
		pr_err("isp_k_cfg_fetch: fail cmd id:%d, not supported.\n",
			param->property);
		break;
	}

	return ret;
}


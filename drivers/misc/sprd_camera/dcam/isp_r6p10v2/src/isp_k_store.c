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

static int isp_store_block(struct isp_io_param *param, enum isp_id idx)
{
	int  ret = 0;
	unsigned int val = 0;
	struct isp_img_size size;
	struct isp_dev_store_info store_info;

	memset(&store_info, 0x00, sizeof(store_info));
	ret = copy_from_user((void *)&store_info,
		param->property_param, sizeof(store_info));
	if (ret != 0) {
		pr_err("isp_store_block: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_STORE_BASE+ISP_STORE_PARAM,
					BIT_0, store_info.bypass);
	if (store_info.bypass)
		return 0;

	ISP_REG_MWR(idx, ISP_STORE_BASE+ISP_STORE_PARAM,
		0xF0, (store_info.color_format << 4));

	ISP_REG_MWR(idx, ISP_STORE_BASE+ISP_STORE_PARAM,
					0x300, store_info.endian << 8);

	size = store_info.size;
	val = ((size.height & 0xFFFF) << 16) | (size.width & 0xFFFF);
	ISP_REG_WR(idx, ISP_STORE_BASE+ISP_STORE_SLICE_SIZE, val);

	val = ((store_info.border.right_border & 0xFF) << 24) |
		  ((store_info.border.left_border & 0xFF) << 16) |
		  ((store_info.border.down_border & 0xFF) << 8) |
		  (store_info.border.up_border & 0xFF);
	ISP_REG_WR(idx, ISP_STORE_BORDER, val);
	ISP_REG_WR(idx, ISP_STORE_BASE+ISP_STORE_Y_ADDR,
					store_info.addr.chn0);

	ISP_REG_WR(idx, ISP_STORE_BASE+ISP_STORE_Y_PITCH,
					store_info.pitch.chn0);
	ISP_REG_WR(idx, ISP_STORE_BASE+ISP_STORE_U_ADDR,
					store_info.addr.chn1);
	ISP_REG_WR(idx, ISP_STORE_BASE+ISP_STORE_U_PITCH,
					store_info.pitch.chn1);

	ISP_REG_WR(idx, ISP_STORE_BASE+ISP_STORE_V_ADDR,
					store_info.addr.chn2);
	ISP_REG_WR(idx, ISP_STORE_BASE+ISP_STORE_V_PITCH,
					store_info.pitch.chn2);

	ISP_REG_MWR(idx, ISP_STORE_READ_CTRL,
				0x3, store_info.rd_ctrl);
	ISP_REG_MWR(idx, ISP_STORE_READ_CTRL,
				0xFFFC, store_info.store_res << 2);

	ISP_REG_MWR(idx, ISP_STORE_BASE+ISP_STORE_SHADOW_CLR_SEL,
		BIT_1, store_info.shadow_clr_sel << 1);
	ISP_REG_MWR(idx, ISP_STORE_BASE+ISP_STORE_SHADOW_CLR,
		BIT_0, store_info.shadow_clr);

	return ret;
}

static int isp_store_cce_bypass(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int store_cce_bypass = 0;

	ret = copy_from_user((void *)&store_cce_bypass,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_store_cce_bypass: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_STORE_CCE_PARAM, BIT_0, store_cce_bypass);
	ISP_REG_MWR(idx, ISP_STORE_CCE_SHADOW_CLR, BIT_0, 1);

	return ret;
}

static int isp_store_cce_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dev_store_cce_info store_cce_info;

	memset(&store_cce_info, 0x00, sizeof(store_cce_info));
	ret = copy_from_user((void *)&store_cce_info,
		param->property_param, sizeof(store_cce_info));
	if (ret != 0) {
		pr_err("isp_store_cce_block: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_STORE_CCE_PARAM, BIT_0, store_cce_info.bypass);
	if (store_cce_info.bypass)
		return 0;
	ISP_REG_MWR(idx, ISP_STORE_CCE_PARAM,
			BIT_1, (store_cce_info.max_len_sel << 1));
	ISP_REG_MWR(idx, ISP_STORE_CCE_PARAM,
			BIT_2, (store_cce_info.speed_2x << 2));
	ISP_REG_MWR(idx, ISP_STORE_CCE_PARAM,
			BIT_3, (store_cce_info.mirror_en << 3));

	ISP_REG_MWR(idx, ISP_STORE_CCE_PARAM,
			0xF0,  (store_cce_info.color_format << 4));
	ISP_REG_MWR(idx, ISP_STORE_CCE_PARAM,
			0x300, (store_cce_info.endian << 8));

	val = ((store_cce_info.size.height & 0xFFFF) << 16) |
		   (store_cce_info.size.width & 0xFFFF);
	ISP_REG_WR(idx, ISP_STORE_CCE_SLICE_SIZE, val);

	val = ((store_cce_info.border.right_border & 0x1FFF) << 16) |
		  ((store_cce_info.border.left_border  & 0x1FFF));
	ISP_REG_WR(idx, ISP_STORE_CCE_BOARDER, val);

	ISP_REG_WR(idx, ISP_STORE_CCE_SLICE_Y_ADDR,
				store_cce_info.addr[0].chn0);
	ISP_REG_WR(idx, ISP_STORE_CCE_Y_PITCH, store_cce_info.pitch.chn0);
	ISP_REG_WR(idx, ISP_STORE_CCE_SLICE_U_ADDR,
				store_cce_info.addr[0].chn1);

	ISP_REG_WR(idx, ISP_STORE_CCE_U_PITCH, store_cce_info.pitch.chn1);
	ISP_REG_WR(idx, ISP_STORE_CCE_SLICE_V_ADDR,
				store_cce_info.addr[0].chn2);
	ISP_REG_WR(idx, ISP_STORE_CCE_V_PITCH, store_cce_info.pitch.chn2);

	ISP_REG_MWR(idx, ISP_STORE_CCE_READ_CTRL, 0x3,
				store_cce_info.rd_ctrl);
	ISP_REG_MWR(idx, ISP_STORE_CCE_READ_CTRL,
			0xFFFFFFFC, store_cce_info.store_res << 2);

	ISP_REG_MWR(idx, ISP_STORE_CCE_MIPI0,
			0xFFFFFF, store_cce_info.total_word);

	val = ((store_cce_info.border.down_border & 0x1FFF) << 16) |
		   (store_cce_info.border.up_border & 0x1FFF);
	ISP_REG_WR(idx, ISP_STORE_CCE_BORDER2, val);

	ISP_REG_MWR(idx, ISP_STORE_CCE_SHADOW_CLR_SEL, BIT_1,
		store_cce_info.shadow_clr_sel << 1);
	ISP_REG_MWR(idx, ISP_STORE_CCE_SHADOW_CLR,
				BIT_0, store_cce_info.shadow_clr);

	return ret;
}

static int isp_store0_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dev_store0_info store0_info;

	memset(&store0_info, 0x00, sizeof(store0_info));
	ret = copy_from_user((void *)&store0_info,
		param->property_param, sizeof(store0_info));
	if (ret != 0) {
		pr_err("isp_store0_block: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_STORE0_PARAM, BIT_0, store0_info.bypass);
	if (store0_info.bypass)
		return 0;

	ISP_REG_MWR(idx, ISP_STORE0_PARAM,
		BIT_1, (store0_info.st_max_len_sel << 1));

	ISP_REG_MWR(idx, ISP_STORE0_PARAM,
		BIT_2, (store0_info.yuv_mode << 2));

	ISP_REG_MWR(idx, ISP_STORE0_PARAM,
		BIT_3, (store0_info.shadow_clr_sel << 3));

	ISP_REG_MWR(idx, ISP_STORE0_PARAM,
		BIT_4, (store0_info.st_y_axi_reorder_en << 4));

	ISP_REG_MWR(idx, ISP_STORE0_PARAM,
		BIT_5, (store0_info.st_uv_axi_reorder_en << 5));

	val = ((store0_info.size.height & 0xFFFF) << 16) |
		   (store0_info.size.width & 0xFFFF);
	ISP_REG_WR(idx, ISP_STORE0_SIZE, val);

	ISP_REG_WR(idx, ISP_STORE0_LUMA_ADDR0,
		store0_info.st_luma_addr[store0_info.buf_sel]);

	ISP_REG_WR(idx, ISP_STORE0_CHROMA_ADDR0,
		store0_info.st_chroma_addr[store0_info.buf_sel]);

	ISP_REG_WR(idx, ISP_STORE0_PITCH, store0_info.st_pitch);

	ISP_REG_MWR(idx, ISP_STORE0_SHADOW_CLR,
			BIT_0, store0_info.shadow_clr);

	return ret;
}

static int isp_store_preview_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dev_store_preview_info store_pre_info;

	memset(&store_pre_info, 0x00, sizeof(store_pre_info));
	ret = copy_from_user((void *)&store_pre_info,
		param->property_param, sizeof(store_pre_info));
	if (ret != 0) {
		pr_err("isp_store_preview_block: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_STORE_PREVIEW_PARAM,
			BIT_0, store_pre_info.bypass);
	if (store_pre_info.bypass)
		return 0;

	ISP_REG_MWR(idx, ISP_STORE_PREVIEW_PARAM,
		BIT_1, (store_pre_info.max_len_sel << 1));

	ISP_REG_MWR(idx, ISP_STORE_PREVIEW_PARAM,
		BIT_2, (store_pre_info.speed_2x << 2));

	ISP_REG_MWR(idx, ISP_STORE_PREVIEW_PARAM,
		BIT_3, (store_pre_info.mirror_en << 3));

	ISP_REG_MWR(idx, ISP_STORE_PREVIEW_PARAM,
		0xF0, (store_pre_info.color_format << 4));

	ISP_REG_MWR(idx, ISP_STORE_PREVIEW_PARAM,
		0x300, (store_pre_info.endian << 8));

	val = ((store_pre_info.size.height & 0xFFFF) << 16) |
		   (store_pre_info.size.width & 0xFFFF);
	ISP_REG_WR(idx, ISP_STORE_PREVIEW_SLICE_SIZE, val);

	val = ((store_pre_info.border.right_border & 0xFF) << 24) |
		  ((store_pre_info.border.left_border & 0xFF) << 16) |
		  ((store_pre_info.border.down_border & 0xFF) << 8) |
		   (store_pre_info.border.up_border & 0xFF);
	ISP_REG_WR(idx, ISP_STORE_PREVIEW_BORDER, val);

	ISP_REG_WR(idx, ISP_STORE_PREVIEW_Y_ADDR,
				store_pre_info.addr.chn0);
	ISP_REG_WR(idx, ISP_STORE_PREVIEW_Y_PITCH,
				store_pre_info.pitch.chn0);
	ISP_REG_WR(idx, ISP_STORE_PREVIEW_U_ADDR,
				store_pre_info.addr.chn1);
	ISP_REG_WR(idx, ISP_STORE_PREVIEW_U_PITCH,
				store_pre_info.pitch.chn1);
	ISP_REG_WR(idx, ISP_STORE_PREVIEW_V_ADDR,
				store_pre_info.addr.chn2);
	ISP_REG_WR(idx, ISP_STORE_PREVIEW_V_PITCH,
				store_pre_info.pitch.chn2);

	ISP_REG_MWR(idx, ISP_STORE_PREVIEW_READ_CTRL,
		0x3, store_pre_info.rd_ctrl);
	ISP_REG_MWR(idx, ISP_STORE_PREVIEW_READ_CTRL,
		0xFFFFFFFC, store_pre_info.store_res << 2);

	ISP_REG_MWR(idx, ISP_STORE_PREVIEW_SHADOW_CLR_SEL,
		BIT_1, store_pre_info.shadow_clr_sel << 1);
	ISP_REG_MWR(idx, ISP_STORE_PREVIEW_SHADOW_CLR,
		BIT_0, store_pre_info.shadow_clr);

	return ret;
}

static int isp_store_video_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dev_store_video_info store_vid_info;

	memset(&store_vid_info, 0x00, sizeof(store_vid_info));
	ret = copy_from_user((void *)&store_vid_info,
		param->property_param, sizeof(store_vid_info));
	if (ret != 0) {
		pr_err("isp_store_video_block: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_STORE_VIDEO_PARAM,
			BIT_0, store_vid_info.bypass);
	if (store_vid_info.bypass)
		return 0;

	ISP_REG_MWR(idx, ISP_STORE_VIDEO_PARAM,
		BIT_1, (store_vid_info.max_len_sel << 1));

	ISP_REG_MWR(idx, ISP_STORE_VIDEO_PARAM,
		BIT_2, (store_vid_info.speed_2x << 2));

	ISP_REG_MWR(idx, ISP_STORE_VIDEO_PARAM,
		BIT_3, (store_vid_info.mirror_en << 3));

	ISP_REG_MWR(idx, ISP_STORE_VIDEO_PARAM,
		0xF0,  (store_vid_info.color_format << 4));

	ISP_REG_MWR(idx, ISP_STORE_VIDEO_PARAM,
		0x300, (store_vid_info.endian << 8));

	val = ((store_vid_info.size.height & 0xFFFF) << 16) |
		   (store_vid_info.size.width & 0xFFFF);
	ISP_REG_WR(idx, ISP_STORE_VIDEO_SLICE_SIZE, val);

	val = ((store_vid_info.border.right_border & 0xFF) << 24) |
		  ((store_vid_info.border.left_border & 0xFF) << 16) |
		  ((store_vid_info.border.down_border & 0xFF) << 8) |
		   (store_vid_info.border.up_border & 0xFF);
	ISP_REG_WR(idx, ISP_STORE_VIDEO_BORDER, val);

	ISP_REG_WR(idx, ISP_STORE_VIDEO_Y_ADDR,
				store_vid_info.addr.chn0);
	ISP_REG_WR(idx, ISP_STORE_VIDEO_Y_PITCH,
				store_vid_info.pitch.chn0);
	ISP_REG_WR(idx, ISP_STORE_VIDEO_U_ADDR,
				store_vid_info.addr.chn1);
	ISP_REG_WR(idx, ISP_STORE_VIDEO_U_PITCH,
				store_vid_info.pitch.chn1);
	ISP_REG_WR(idx, ISP_STORE_VIDEO_V_ADDR,
				store_vid_info.addr.chn2);
	ISP_REG_WR(idx, ISP_STORE_VIDEO_V_PITCH,
				store_vid_info.pitch.chn2);

	ISP_REG_MWR(idx, ISP_STORE_VIDEO_READ_CTRL,
		0x3, store_vid_info.rd_ctrl);
	ISP_REG_MWR(idx, ISP_STORE_VIDEO_READ_CTRL,
		0xFFFFFFFC, store_vid_info.store_res << 2);

	ISP_REG_MWR(idx, ISP_STORE_VIDEO_SHADOW_CLR_SEL,
		BIT_1, store_vid_info.shadow_clr_sel << 1);
	ISP_REG_MWR(idx, ISP_STORE_VIDEO_SHADOW_CLR,
		BIT_0, store_vid_info.shadow_clr);

	return ret;
}

static int isp_store_capture_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dev_store_capture_info store_cap_info;

	memset(&store_cap_info, 0x00, sizeof(store_cap_info));
	ret = copy_from_user((void *)&store_cap_info,
		param->property_param, sizeof(store_cap_info));
	if (ret != 0) {
		pr_err("isp_store_capture_block: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_STORE_CAPTURE_PARAM,
		BIT_0, store_cap_info.bypass);
	if (store_cap_info.bypass)
		return 0;

	ISP_REG_MWR(idx, ISP_STORE_CAPTURE_PARAM,
		BIT_1, (store_cap_info.max_len_sel << 1));
	ISP_REG_MWR(idx, ISP_STORE_CAPTURE_PARAM,
		BIT_2, (store_cap_info.speed_2x << 2));
	ISP_REG_MWR(idx, ISP_STORE_CAPTURE_PARAM,
		BIT_3, (store_cap_info.mirror_en << 3));
	ISP_REG_MWR(idx, ISP_STORE_CAPTURE_PARAM,
		0xF0, (store_cap_info.color_format << 4));
	ISP_REG_MWR(idx, ISP_STORE_CAPTURE_PARAM,
		0x300, (store_cap_info.endian << 8));

	val = ((store_cap_info.size.height & 0xFFFF) << 16) |
		   (store_cap_info.size.width & 0xFFFF);
	ISP_REG_WR(idx, ISP_STORE_CAPTURE_SLICE_SIZE, val);

	val = ((store_cap_info.border.right_border & 0xFF) << 24) |
		  ((store_cap_info.border.left_border & 0xFF) << 16) |
		  ((store_cap_info.border.down_border & 0xFF) << 8) |
		   (store_cap_info.border.up_border & 0xFF);
	ISP_REG_WR(idx, ISP_STORE_CAPTURE_BORDER, val);

	ISP_REG_WR(idx, ISP_STORE_CAPTURE_Y_ADDR, store_cap_info.addr.chn0);
	ISP_REG_WR(idx, ISP_STORE_CAPTURE_Y_PITCH, store_cap_info.pitch.chn0);
	ISP_REG_WR(idx, ISP_STORE_CAPTURE_U_ADDR, store_cap_info.addr.chn1);
	ISP_REG_WR(idx, ISP_STORE_CAPTURE_U_PITCH, store_cap_info.pitch.chn1);
	ISP_REG_WR(idx, ISP_STORE_CAPTURE_V_ADDR, store_cap_info.addr.chn2);
	ISP_REG_WR(idx, ISP_STORE_CAPTURE_V_PITCH, store_cap_info.pitch.chn2);

	ISP_REG_MWR(idx, ISP_STORE_CAPTURE_READ_CTRL,
		0x3, store_cap_info.rd_ctrl);

	ISP_REG_MWR(idx, ISP_STORE_CAPTURE_READ_CTRL,
		0xFFFFFFFC, store_cap_info.store_res << 2);

	ISP_REG_MWR(idx, ISP_STORE_CAPTURE_SHADOW_CLR_SEL,
		BIT_1, store_cap_info.shadow_clr_sel << 1);

	ISP_REG_MWR(idx, ISP_STORE_CAPTURE_SHADOW_CLR,
		BIT_0, store_cap_info.shadow_clr);

	return ret;
}

static int isp_scl0_pre_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dev_scaler0_pre_info scl0_pre_info;

	memset(&scl0_pre_info, 0x00, sizeof(scl0_pre_info));
	ret = copy_from_user((void *)&scl0_pre_info,
		param->property_param, sizeof(scl0_pre_info));
	if (ret != 0) {
		pr_err("isp_scl0_pre_block: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	ISP_REG_MWR(idx, ISP_SCALER0_PRE_CFG,
		BIT_9, scl0_pre_info.isp_sacler_all_bypass << 9);

	ISP_REG_MWR(idx, ISP_SCALER0_PRE_CFG,
		BIT_10, scl0_pre_info.isp_uv_sync_y << 10);

	val = ((scl0_pre_info.size.height & 0x3FFF) << 16) |
		   (scl0_pre_info.size.width & 0x3FFF);
	ISP_REG_WR(idx, ISP_SCALER0_PRE_TRIM1_SIZE, val);

	return ret;
}

static int isp_scl_pre_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dev_scaler_pre_info scl_pre_info;

	memset(&scl_pre_info, 0x00, sizeof(scl_pre_info));
	ret = copy_from_user((void *)&scl_pre_info,
		param->property_param, sizeof(scl_pre_info));
	if (ret != 0) {
		pr_err("isp_scl_pre_block: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_SCALER_PRE_CFG,
		BIT_9, scl_pre_info.bypass << 9);

	val = ((scl_pre_info.size.height & 0x3FFF) << 16) |
		   (scl_pre_info.size.width & 0x3FFF);
	ISP_REG_WR(idx, ISP_SCALER_PRE_TRIM1_SIZE, val);

	return ret;
}

static int isp_scl_vid_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dev_scaler_vid_info scl_vid_info;

	memset(&scl_vid_info, 0x00, sizeof(scl_vid_info));
	ret = copy_from_user((void *)&scl_vid_info,
		param->property_param, sizeof(scl_vid_info));
	if (ret != 0) {
		pr_err("isp_scl_vid_block: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_SCALER_VID_CFG,
		BIT_9, scl_vid_info.bypass << 9);

	val = ((scl_vid_info.size.height & 0x3FFF) << 16)	 |
		   (scl_vid_info.size.width & 0x3FFF);
	ISP_REG_WR(idx, ISP_SCALER_VID_TRIM1_SIZE, val);

	return ret;
}

static int isp_scl_cap_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dev_scaler_cap_info scl_cap_info;

	memset(&scl_cap_info, 0x00, sizeof(scl_cap_info));
	ret = copy_from_user((void *)&scl_cap_info,
		param->property_param, sizeof(scl_cap_info));
	if (ret != 0) {
		pr_err("isp_scl_cap_block: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	ISP_REG_MWR(idx, ISP_SCALER_CAP_CFG,
		BIT_9, scl_cap_info.bypass << 9);

	val = ((scl_cap_info.size.height & 0x3FFF) << 16)	 |
		   (scl_cap_info.size.width & 0x3FFF);
	ISP_REG_WR(idx, ISP_SCALER_CAP_TRIM1_SIZE, val);

	return ret;
}

static int isp_k_store_bypass(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int bypass = 0;
	unsigned int base = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_STORE:
		base = ISP_STORE_BASE;
		break;
	case ISP_BLOCK_STOREA:
		base = ISP_STOREA_BASE;
		break;
	case ISP_BLOCK_STORE1:
		base = ISP_STORE1_BASE;
		break;
	case ISP_BLOCK_STORE2:
		base = ISP_STORE2_BASE;
		break;
	case ISP_BLOCK_STORE3:
		base = ISP_STORE3_BASE;
		break;
	case ISP_BLOCK_STORE4:
		base = ISP_STORE4_BASE;
		break;
	default:
		pr_err("isp_k_store_bypass: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&bypass,
		param->property_param, sizeof(bypass));
	if (ret != 0) {
		pr_err("isp_k_store_bypass: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (bypass)
		ISP_REG_OWR(idx, base+ISP_STORE_PARAM, BIT_0);
	else
		ISP_REG_MWR(idx, base+ISP_STORE_PARAM, BIT_0, 0);

	return ret;
}

static int isp_k_store_subtract(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int subtract = 0;
	unsigned int base = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_STORE:
		base = ISP_STORE_BASE;
		break;
	case ISP_BLOCK_STOREA:
		base = ISP_STOREA_BASE;
		break;
	case ISP_BLOCK_STORE1:
		base = ISP_STORE1_BASE;
		break;
	case ISP_BLOCK_STORE2:
		base = ISP_STORE2_BASE;
		break;
	case ISP_BLOCK_STORE3:
		base = ISP_STORE3_BASE;
		break;
	case ISP_BLOCK_STORE4:
		base = ISP_STORE4_BASE;
		break;
	default:
		pr_err("isp_k_store_subtract: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&subtract,
		param->property_param, sizeof(subtract));
	if (ret != 0) {
		pr_err("isp_k_store_subtract: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (subtract)
		ISP_REG_OWR(idx, base+ISP_STORE_PARAM, BIT_1);
	else
		ISP_REG_MWR(idx, base+ISP_STORE_PARAM, BIT_1, 0);

	return ret;
}

static int isp_k_store_burst_len_sel
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	unsigned int base = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_STOREA:
		base = ISP_STOREA_BASE;
		break;
	case ISP_BLOCK_STORE1:
		base = ISP_STORE1_BASE;
		break;
	case ISP_BLOCK_STORE2:
		base = ISP_STORE2_BASE;
		break;
	case ISP_BLOCK_STORE3:
		base = ISP_STORE3_BASE;
		break;
	case ISP_BLOCK_STORE4:
		base = ISP_STORE4_BASE;
		break;
	default:
		pr_err("isp_k_store_burst_len_sel: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(val));
	if (ret != 0) {
		pr_err("isp_k_store_burst_len_sel: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (val)
		ISP_REG_OWR(idx, base+ISP_STORE_PARAM, BIT_1);
	else
		ISP_REG_MWR(idx, base+ISP_STORE_PARAM, BIT_1, 0);

	return ret;
}

static int isp_k_store_color_format
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int color_format = 0;
	unsigned int base = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_STORE:
		base = ISP_STORE_BASE;
		break;
	case ISP_BLOCK_STOREA:
		base = ISP_STOREA_BASE;
		break;
	case ISP_BLOCK_STORE1:
		base = ISP_STORE1_BASE;
		break;
	case ISP_BLOCK_STORE2:
		base = ISP_STORE2_BASE;
		break;
	case ISP_BLOCK_STORE3:
		base = ISP_STORE3_BASE;
		break;
	case ISP_BLOCK_STORE4:
		base = ISP_STORE4_BASE;
		break;
	default:
		pr_err("isp_k_store_color_format: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&color_format,
		param->property_param, sizeof(color_format));
	if (ret != 0) {
		pr_err("isp_k_fetch_color_format: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	ISP_REG_MWR(idx, base+ISP_STORE_PARAM, 0xF << 4, color_format << 4);

	return ret;
}

static int isp_k_store_slice_size
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_img_size size = {0, 0};
	unsigned int base = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_STORE:
		base = ISP_STORE_BASE;
		break;
	case ISP_BLOCK_STOREA:
		base = ISP_STOREA_BASE;
		break;
	case ISP_BLOCK_STORE1:
		base = ISP_STORE1_BASE;
		break;
	case ISP_BLOCK_STORE2:
		base = ISP_STORE2_BASE;
		break;
	case ISP_BLOCK_STORE3:
		base = ISP_STORE3_BASE;
		break;
	case ISP_BLOCK_STORE4:
		base = ISP_STORE4_BASE;
		break;
	default:
		pr_err("isp_k_store_slice_size: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&size,
		param->property_param, sizeof(size));
	if (ret != 0) {
		pr_err("isp_k_store_slice_size: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	val = ((size.height & 0xFFFF) << 16) | (size.width & 0xFFFF);
	ISP_REG_WR(idx, base+ISP_STORE_SLICE_SIZE, val);

	return ret;
}

static int isp_k_store_y_addr(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int addr = 0;
	unsigned int base = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_STORE:
		base = ISP_STORE_BASE;
		break;
	case ISP_BLOCK_STOREA:
		base = ISP_STOREA_BASE;
		break;
	case ISP_BLOCK_STORE1:
		base = ISP_STORE1_BASE;
		break;
	case ISP_BLOCK_STORE2:
		base = ISP_STORE2_BASE;
		break;
	case ISP_BLOCK_STORE3:
		base = ISP_STORE3_BASE;
		break;
	case ISP_BLOCK_STORE4:
		base = ISP_STORE4_BASE;
		break;
	default:
		pr_err("isp_k_store_y_addr: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&addr,
		param->property_param, sizeof(addr));
	if (ret != 0) {
		pr_err("isp_k_store_y_addr: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_WR(idx, base+ISP_STORE_Y_ADDR, addr);

	return ret;
}

static int isp_k_store_y_pitch(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int pitch = 0;
	unsigned int base = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_STORE:
		base = ISP_STORE_BASE;
		break;
	case ISP_BLOCK_STOREA:
		base = ISP_STOREA_BASE;
		break;
	case ISP_BLOCK_STORE1:
		base = ISP_STORE1_BASE;
		break;
	case ISP_BLOCK_STORE2:
		base = ISP_STORE2_BASE;
		break;
	case ISP_BLOCK_STORE3:
		base = ISP_STORE3_BASE;
		break;
	case ISP_BLOCK_STORE4:
		base = ISP_STORE4_BASE;
		break;
	default:
		pr_err("isp_k_store_y_pitch: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&pitch,
		param->property_param, sizeof(pitch));
	if (ret != 0) {
		pr_err("isp_k_store_y_pitch: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_WR(idx, base+ISP_STORE_Y_PITCH, pitch);

	return ret;
}

static int isp_k_store_u_addr(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int addr = 0;
	unsigned int base = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_STORE:
		base = ISP_STORE_BASE;
		break;
	case ISP_BLOCK_STOREA:
		base = ISP_STOREA_BASE;
		break;
	case ISP_BLOCK_STORE1:
		base = ISP_STORE1_BASE;
		break;
	case ISP_BLOCK_STORE2:
		base = ISP_STORE2_BASE;
		break;
	case ISP_BLOCK_STORE3:
		base = ISP_STORE3_BASE;
		break;
	case ISP_BLOCK_STORE4:
		base = ISP_STORE4_BASE;
		break;
	default:
		pr_err("isp_k_store_u_addr: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&addr,
		param->property_param, sizeof(addr));
	if (ret != 0) {
		pr_err("isp_k_store_u_addr: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_WR(idx, base+ISP_STORE_U_ADDR, addr);

	return ret;
}

static int isp_k_store_u_pitch(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int pitch = 0;
	unsigned int base = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_STORE:
		base = ISP_STORE_BASE;
		break;
	case ISP_BLOCK_STOREA:
		base = ISP_STOREA_BASE;
		break;
	case ISP_BLOCK_STORE1:
		base = ISP_STORE1_BASE;
		break;
	case ISP_BLOCK_STORE2:
		base = ISP_STORE2_BASE;
		break;
	case ISP_BLOCK_STORE3:
		base = ISP_STORE3_BASE;
		break;
	case ISP_BLOCK_STORE4:
		base = ISP_STORE4_BASE;
		break;
	default:
		pr_err("isp_k_store_u_pitch: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&pitch,
		param->property_param, sizeof(pitch));
	if (ret != 0) {
		pr_err("isp_k_store_u_pitch: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_WR(idx, base+ISP_STORE_U_PITCH, pitch);

	return ret;
}

static int isp_k_store_v_addr(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int addr = 0;
	unsigned int base = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_STORE:
		base = ISP_STORE_BASE;
		break;
	case ISP_BLOCK_STOREA:
		base = ISP_STOREA_BASE;
		break;
	case ISP_BLOCK_STORE1:
		base = ISP_STORE1_BASE;
		break;
	case ISP_BLOCK_STORE2:
		base = ISP_STORE2_BASE;
		break;
	case ISP_BLOCK_STORE3:
		base = ISP_STORE3_BASE;
		break;
	case ISP_BLOCK_STORE4:
		base = ISP_STORE4_BASE;
		break;
	default:
		pr_err("isp_k_store_v_addr: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&addr,
		param->property_param, sizeof(addr));
	if (ret != 0) {
		pr_err("isp_k_store_v_addr: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_WR(idx, base+ISP_STORE_V_ADDR, addr);

	return ret;
}

static int isp_k_store_v_pitch(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int pitch = 0;
	unsigned int base = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_STORE:
		base = ISP_STORE_BASE;
		break;
	case ISP_BLOCK_STOREA:
		base = ISP_STOREA_BASE;
		break;
	case ISP_BLOCK_STORE1:
		base = ISP_STORE1_BASE;
		break;
	case ISP_BLOCK_STORE2:
		base = ISP_STORE2_BASE;
		break;
	case ISP_BLOCK_STORE3:
		base = ISP_STORE3_BASE;
		break;
	case ISP_BLOCK_STORE4:
		base = ISP_STORE4_BASE;
		break;
	default:
		pr_err("isp_k_store_v_pitch: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&pitch,
		param->property_param, sizeof(pitch));
	if (ret != 0) {
		pr_err("isp_k_store_v_pitch: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_WR(idx, base+ISP_STORE_V_PITCH, pitch);

	return ret;
}

static int isp_k_store_border(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	struct slice_overlap overlap = {0, 0, 0, 0};
	unsigned int val = 0;
	unsigned int base = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_STORE:
		base = ISP_STORE_BASE;
		break;
	case ISP_BLOCK_STOREA:
		base = ISP_STOREA_BASE;
		break;
	case ISP_BLOCK_STORE1:
		base = ISP_STORE1_BASE;
		break;
	case ISP_BLOCK_STORE2:
		base = ISP_STORE2_BASE;
		break;
	case ISP_BLOCK_STORE3:
		base = ISP_STORE3_BASE;
		break;
	case ISP_BLOCK_STORE4:
		base = ISP_STORE4_BASE;
		break;
	default:
		pr_err("isp_k_store_border: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&overlap,
		param->property_param, sizeof(overlap));
	if (ret != 0) {
		pr_err("isp_k_store_border: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (param->sub_block != ISP_BLOCK_STOREA) {
		val = (overlap.overlap_up & 0xFF)
			| ((overlap.overlap_down & 0xFF) << 8)
			| ((overlap.overlap_left & 0xFF) << 16)
			| ((overlap.overlap_right & 0xFF) << 24);
		ISP_REG_WR(idx, base+ISP_STORE_BORDER1, val);
	} else {
		val = (overlap.overlap_left & 0x1FFF)
			| ((overlap.overlap_right & 0x1FFF) << 16);
		ISP_REG_WR(idx, base+ISP_STORE_BORDER1, val);

		val = (overlap.overlap_up & 0x1FFF)
			| ((overlap.overlap_down & 0x1FFF) << 16);
		ISP_REG_WR(idx, base+ISP_STORE_BORDER2, val);
	}

	return ret;
}

static int isp_k_store_shadow_clr_sel
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int clr_sel = 0;
	unsigned int base = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_STORE:
		base = ISP_STORE_BASE;
		break;
	case ISP_BLOCK_STOREA:
		base = ISP_STOREA_BASE;
		break;
	case ISP_BLOCK_STORE1:
		base = ISP_STORE1_BASE;
		break;
	case ISP_BLOCK_STORE2:
		base = ISP_STORE2_BASE;
		break;
	case ISP_BLOCK_STORE3:
		base = ISP_STORE3_BASE;
		break;
	case ISP_BLOCK_STORE4:
		base = ISP_STORE4_BASE;
		break;
	default:
		pr_err("isp_k_store_shadow_clr_sel: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&clr_sel,
		param->property_param, sizeof(clr_sel));
	if (ret != 0) {
		pr_err("isp_k_store_shadow_clr_sel: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (clr_sel)
		ISP_REG_OWR(idx, base+ISP_STORE_SHADOW_CLR_SEL, BIT_1);
	else
		ISP_REG_MWR(idx, base+ISP_STORE_SHADOW_CLR_SEL, BIT_1, 0);

	return ret;
}

static int isp_k_store_shadow_clr
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int clr = 0;
	unsigned int base = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_STORE:
		base = ISP_STORE_BASE;
		break;
	case ISP_BLOCK_STOREA:
		base = ISP_STOREA_BASE;
		break;
	case ISP_BLOCK_STORE1:
		base = ISP_STORE1_BASE;
		break;
	case ISP_BLOCK_STORE2:
		base = ISP_STORE2_BASE;
		break;
	case ISP_BLOCK_STORE3:
		base = ISP_STORE3_BASE;
		break;
	case ISP_BLOCK_STORE4:
		base = ISP_STORE4_BASE;
		break;
	default:
		pr_err("isp_k_store_shadow_clr: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&clr,
		param->property_param, sizeof(clr));
	if (ret != 0) {
		pr_err("isp_k_store_shadow_clr: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_WR(idx, base+ISP_STORE_SHADOW_CLR, clr);

	return ret;
}

static int isp_k_store_mirror_en
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	unsigned int base = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_STORE:
		base = ISP_STORE_BASE;
		break;
	case ISP_BLOCK_STOREA:
		base = ISP_STOREA_BASE;
		break;
	case ISP_BLOCK_STORE1:
		base = ISP_STORE1_BASE;
		break;
	case ISP_BLOCK_STORE2:
		base = ISP_STORE2_BASE;
		break;
	case ISP_BLOCK_STORE3:
		base = ISP_STORE3_BASE;
		break;
	case ISP_BLOCK_STORE4:
		base = ISP_STORE4_BASE;
		break;
	default:
		pr_err("isp_k_store_mirror_en: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(val));
	if (ret != 0) {
		pr_err("isp_k_store_mirror_en: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, base+ISP_STORE_PARAM, BIT_3, val << 3);

	return ret;
}

static int isp_k_store_speed2x(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	unsigned int base = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_STORE:
		base = ISP_STORE_BASE;
		break;
	case ISP_BLOCK_STOREA:
		base = ISP_STOREA_BASE;
		break;
	case ISP_BLOCK_STORE1:
		base = ISP_STORE1_BASE;
		break;
	case ISP_BLOCK_STORE2:
		base = ISP_STORE2_BASE;
		break;
	case ISP_BLOCK_STORE3:
		base = ISP_STORE3_BASE;
		break;
	case ISP_BLOCK_STORE4:
		base = ISP_STORE4_BASE;
		break;
	default:
		pr_err("isp_k_store_speed2x: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(val));
	if (ret != 0) {
		pr_err("isp_k_store_speed2x: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, base+ISP_STORE_PARAM, BIT_2, val << 2);

	return ret;
}

static int isp_k_store_endian(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	unsigned int base = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_STORE:
		base = ISP_STORE_BASE;
		break;
	case ISP_BLOCK_STOREA:
		base = ISP_STOREA_BASE;
		break;
	case ISP_BLOCK_STORE1:
		base = ISP_STORE1_BASE;
		break;
	case ISP_BLOCK_STORE2:
		base = ISP_STORE2_BASE;
		break;
	case ISP_BLOCK_STORE3:
		base = ISP_STORE3_BASE;
		break;
	case ISP_BLOCK_STORE4:
		base = ISP_STORE4_BASE;
		break;
	default:
		pr_err("isp_k_store_endian: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(val));
	if (ret != 0) {
		pr_err("isp_k_store_endian: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, base+ISP_STORE_PARAM, 0x3 << 8, val << 8);

	return ret;
}

static int isp_k_store_mipi(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	unsigned int base = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_STORE:
		base = ISP_STORE_BASE;
		break;
	case ISP_BLOCK_STOREA:
		base = ISP_STOREA_BASE;
		break;
	case ISP_BLOCK_STORE1:
		base = ISP_STORE1_BASE;
		break;
	case ISP_BLOCK_STORE2:
		base = ISP_STORE2_BASE;
		break;
	case ISP_BLOCK_STORE3:
		base = ISP_STORE3_BASE;
		break;
	case ISP_BLOCK_STORE4:
		base = ISP_STORE4_BASE;
		break;
	default:
		pr_err("isp_k_store_mipi: fail cmd id:%d, not supported.\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(val));
	if (ret != 0) {
		pr_err("isp_k_store_mipi: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_WR(idx, base+ISP_STORE_MIPI, val & 0xFFFFFF);

	return ret;
}

int isp_k_cfg_store(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_store: param is null error.\n");
		return -1;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_store: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_STORE_BLOCK:
		ret = isp_store_block(param, idx);
		break;
	case ISP_PRO_STORE_CEE_BYPASS:
		ret = isp_store_cce_bypass(param, idx);
		break;
	case ISP_PRO_STORE_CEE_BLOCK:
		ret = isp_store_cce_block(param, idx);
		break;
	case ISP_PRO_STORE0_BLOCK:
		ret = isp_store0_block(param, idx);
		break;
	case ISP_PRO_STORE_PREVIEW_BLOCK:
		ret = isp_store_preview_block(param, idx);
		break;
	case ISP_PRO_STORE_VIDEO_BLOCK:
		ret = isp_store_video_block(param, idx);
		break;
	case ISP_PRO_STORE_CAPTURE_BLOCK:
		ret = isp_store_capture_block(param, idx);
		break;
	case ISP_PRO_SCL0_PRE_BLOCK:
		ret = isp_scl0_pre_block(param, idx);
		break;
	case ISP_PRO_SCL_PRE_BLOCK:
		ret = isp_scl_pre_block(param, idx);
		break;
	case ISP_PRO_SCL_VID_BLOCK:
		ret = isp_scl_vid_block(param, idx);
		break;
	case ISP_PRO_SCL_CAP_BLOCK:
		ret = isp_scl_cap_block(param, idx);
		break;
	case ISP_PRO_STORE_BYPASS:
		ret = isp_k_store_bypass(param, idx);
		break;
	case ISP_PRO_STORE_BURST_LEN_SEL:
		ret = isp_k_store_burst_len_sel(param, idx);
		break;
	case ISP_PRO_STORE_SUBTRACT:
		ret = isp_k_store_subtract(param, idx);
		break;
	case ISP_PRO_STORE_COLOR_FORMAT:
		ret = isp_k_store_color_format(param, idx);
		break;
	case ISP_PRO_STORE_SLICE_SIZE:
		ret = isp_k_store_slice_size(param, idx);
		break;
	case ISP_PRO_STORE_Y_ADDR:
		ret = isp_k_store_y_addr(param, idx);
		break;
	case ISP_PRO_STORE_Y_PITCH:
		ret = isp_k_store_y_pitch(param, idx);
		break;
	case ISP_PRO_STORE_U_ADDR:
		ret = isp_k_store_u_addr(param, idx);
		break;
	case ISP_PRO_STORE_U_PITCH:
		ret = isp_k_store_u_pitch(param, idx);
		break;
	case ISP_PRO_STORE_V_ADDR:
		ret = isp_k_store_v_addr(param, idx);
		break;
	case ISP_PRO_STORE_V_PITCH:
		ret = isp_k_store_v_pitch(param, idx);
		break;
	case ISP_PRO_STORE_BORDER:
		ret = isp_k_store_border(param, idx);
		break;
	case ISP_PRO_STORE_SHADOW_CLR_SEL:
		ret = isp_k_store_shadow_clr_sel(param, idx);
		break;
	case ISP_PRO_STORE_SHADOW_CLR:
		ret = isp_k_store_shadow_clr(param, idx);
		break;
	case ISP_PRO_STORE_MIRROR_EN:
		ret = isp_k_store_mirror_en(param, idx);
		break;
	case ISP_PRO_STORE_SPEED2X:
		ret = isp_k_store_speed2x(param, idx);
		break;
	case ISP_PRO_STORE_ENDIAN:
		ret = isp_k_store_endian(param, idx);
		break;
	case ISP_PRO_STORE_MIPI:
		ret = isp_k_store_mipi(param, idx);
		break;
	default:
		pr_err("isp_k_cfg_store: fail cmd id:%d, not supported.\n",
			param->property);
		break;
	}

	return ret;
}

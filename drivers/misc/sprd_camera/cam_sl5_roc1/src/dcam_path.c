/*
 * Copyright (C) 2017-2018 Spreadtrum Communications Inc.
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


#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>

#include <video/sprd_mm.h>


#include "cam_hw.h"
#include "cam_types.h"
#include "cam_queue.h"
#include "cam_buf.h"

#include "dcam_reg.h"
#include "dcam_int.h"
#include "dcam_core.h"
#include "dcam_path.h"
#include "dcam_interface.h"


/* Macro Definitions */
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "dcam_path: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

#define DCAM_RDS_MAX_BUF_SIZE  0x400


unsigned long dcam_store_addr[DCAM_PATH_MAX] = {
	DCAM_FULL_BASE_WADDR,
	DCAM_BIN_BASE_WADDR0,
	DCAM_PDAF_BASE_WADDR,
	DCAM_VCH2_BASE_WADDR,
	DCAM_VCH3_BASE_WADDR,
	DCAM_AEM_BASE_WADDR,
	ISP_AFM_BASE_WADDR,
	ISP_AFL_GLB_WADDR,
	DCAM_HIST_BASE_WADDR,
	ISP_NR3_WADDR,
	ISP_BPC_OUT_ADDR,
};


int dcam_cfg_path(void *dcam_handle,
				struct dcam_path_desc *path,
				void *param)
{
	int ret = 0;
	uint32_t idx;
	uint32_t invalid;
	struct img_size size_bin_in, size_bin_out;
	struct dcam_pipe_dev *dev = NULL;
	struct dcam_path_cfg_param *ch_desc;

	if (!dcam_handle || !path || !param) {
		pr_err("error input ptr.\n");
		return -EFAULT;
	}
	dev = (struct dcam_pipe_dev *)dcam_handle;
	ch_desc = (struct dcam_path_cfg_param *)param;
	idx = dev->idx;

	switch (path->path_id) {
	case  DCAM_PATH_FULL:
		path->frm_deci = ch_desc->frm_deci;
		path->frm_skip = ch_desc->frm_skip;

		path->is_loose = ch_desc->is_loose;
		path->endian = ch_desc->endian;

		path->in_size = ch_desc->input_size;
		path->in_trim = ch_desc->input_trim;

		invalid = 0;
		invalid |= ((path->in_size.w == 0) || (path->in_size.h == 0));
		invalid |= (path->in_size.w > DCAM_PATH_WMAX);
		invalid |= (path->in_size.h > DCAM_PATH_WMAX);
		invalid |= ((path->in_trim.start_x +
				path->in_trim.size_x) > path->in_size.w);
		invalid |= ((path->in_trim.start_y +
				path->in_trim.size_y) > path->in_size.h);
		if (invalid) {
			pr_err("error size:%d %d, trim %d %d %d %d\n",
					path->in_size.w, path->in_size.h,
					path->in_trim.start_x, path->in_trim.start_y,
					path->in_trim.size_x, path->in_trim.size_y);
			return -EINVAL;
		}
		if ((path->in_size.w > path->in_trim.size_x) ||
			(path->in_size.h > path->in_trim.size_y)) {
			path->out_size.w = path->in_trim.size_x;
			path->out_size.h = path->in_trim.size_y;
		} else {
			path->out_size.w = path->in_size.w;
			path->out_size.h = path->in_size.h;
		}

		path->src_sel = ch_desc->is_raw ? 0 : 1;
		path->src_sel = 0;
		pr_info("cfg full path done. size %d %d %d %d\n",
			path->in_size.w, path->in_size.h,
			path->out_size.w, path->out_size.h);
		pr_info("sel %d. trim %d %d %d %d\n", path->src_sel,
			path->in_trim.start_x, path->in_trim.start_y,
			path->in_trim.size_x, path->in_trim.size_y);
		break;

	case  DCAM_PATH_BIN:
		path->frm_deci = ch_desc->frm_deci;
		path->frm_skip = ch_desc->frm_skip;

		path->is_loose = ch_desc->is_loose;
		path->endian = ch_desc->endian;

		path->in_size = ch_desc->input_size;
		path->in_trim = ch_desc->input_trim;
		path->out_size = ch_desc->output_size;

		invalid = 0;
		invalid |= ((path->in_size.w == 0) || (path->in_size.h == 0));
		invalid |= (path->in_size.w > DCAM_PATH_WMAX);
		invalid |= (path->in_size.h > DCAM_PATH_WMAX);
		invalid |= ((path->in_trim.start_x +
				path->in_trim.size_x) > path->in_size.w);
		invalid |= ((path->in_trim.start_y +
				path->in_trim.size_y) > path->in_size.h);
		if (invalid) {
			pr_err("error size:%d %d, trim %d %d %d %d\n",
					path->in_size.w, path->in_size.h,
					path->in_trim.start_x, path->in_trim.start_y,
					path->in_trim.size_x, path->in_trim.size_y);
			return -EINVAL;
		}
		if ((path->in_size.w > path->in_trim.size_x) ||
			(path->in_size.h > path->in_trim.size_y)) {
			size_bin_in.w = path->in_trim.size_x;
			size_bin_in.h = path->in_trim.size_y;
		} else {
			size_bin_in.w = path->in_size.w;
			size_bin_in.h = path->in_size.h;
		}

		size_bin_out = path->out_size;
		if (size_bin_out.w > size_bin_in.w)
			size_bin_out.w = size_bin_in.w;
		if (size_bin_out.h > size_bin_in.h)
			size_bin_out.h = size_bin_in.h;

		if ((size_bin_out.w == size_bin_in.w) &&
			(size_bin_out.h == size_bin_in.h))
			path->scaler_sel = DCAM_SCALER_BYPASS;
		else if ((size_bin_out.w * 2 == size_bin_in.w) &&
			(size_bin_out.h * 2 == size_bin_in.h)) {
			path->scaler_sel = DCAM_SCALER_BINNING;
			path->bin_ratio = 0;
		} else {
			path->scaler_sel = DCAM_SCALER_BINNING;
			if (path->rds_coeff_buf == NULL)
				path->rds_coeff_buf =
					vzalloc(DCAM_RDS_MAX_BUF_SIZE);

			if (path->rds_coeff_buf) {
				pr_err("fail to alloc rds_coeff buffer.\n");
				return -ENOMEM;
			}
			path->rds_coeff_size = DCAM_RDS_MAX_BUF_SIZE;
		}
		pr_info("cfg bin path done. size %d %d %d %d\n",
			path->in_size.w, path->in_size.h,
			path->out_size.w, path->out_size.h);
		pr_info("ratio %d. trim %d %d %d %d\n", path->scaler_sel,
			path->in_trim.start_x, path->in_trim.start_y,
			path->in_trim.size_x, path->in_trim.size_y);
		break;

	case DCAM_PATH_VCH3:
		pr_info("config path VCH3\n");
		break;
	default:
		pr_err("unknown path %d\n", path->path_id);
		break;
	}
	return ret;
}


int dcam_start_path(void *dcam_handle, struct dcam_path_desc *path)
{
	int ret = 0;
	uint32_t idx;
	uint32_t path_id;
	uint32_t reg_val;
	struct dcam_pipe_dev *dev = NULL;

	pr_debug("enter.");

	if (!path || !dcam_handle) {
		pr_err("error input ptr.\n");
		return -EFAULT;
	}

	dev = (struct dcam_pipe_dev *)dcam_handle;
	idx = dev->idx;
	path_id = path->path_id;

	switch (path_id) {
	case  DCAM_PATH_FULL:

#ifdef TEST_SHARKL3
		DCAM_REG_MWR(idx, DCAM_PATH_ENDIAN,
			BIT_1 |  BIT_0, path->endian.y_endian << 0);
#else
		DCAM_REG_MWR(idx, DCAM_PATH_ENDIAN,
			BIT_17 |  BIT_16, path->endian.y_endian << 16);
#endif

		DCAM_REG_MWR(idx, DCAM_FULL_CFG, BIT_0, path->is_loose);
		DCAM_REG_MWR(idx, DCAM_FULL_CFG, BIT_2, path->src_sel << 2);

		if ((path->in_size.w > path->in_trim.size_x) ||
			(path->in_size.h > path->in_trim.size_y)) {

			DCAM_REG_MWR(idx, DCAM_FULL_CFG, BIT_1, 1 << 1);

			reg_val = (path->in_trim.start_y << 16) |
						path->in_trim.start_x;
			DCAM_REG_WR(idx, DCAM_FULL_CROP_START, reg_val);

			reg_val = (path->in_trim.size_y << 16) |
						path->in_trim.size_x;
			DCAM_REG_WR(idx, DCAM_FULL_CROP_SIZE, reg_val);

		} else {
			DCAM_REG_MWR(idx, DCAM_FULL_CFG, BIT_1, 0 << 1);
		}
		/* full_path_en */
		DCAM_REG_MWR(idx, DCAM_CFG, BIT_1, (1 << 1));
		break;


	case  DCAM_PATH_BIN:
		DCAM_REG_MWR(idx, DCAM_PATH_ENDIAN,
			BIT_19 |  BIT_18, path->endian.y_endian << 18);

		DCAM_REG_MWR(idx,
			DCAM_CAM_BIN_CFG, BIT_0, path->is_loose);

#ifdef TEST_SHARKL3
		DCAM_REG_MWR(idx,
			DCAM_CAM_BIN_CFG, BIT_4, 0); /* disable crop0 */

		if (path->scaler_sel >= DCAM_SCALER_BYPASS) {
			/* bypass scaler */
			DCAM_REG_MWR(idx, DCAM_CAM_BIN_CFG, BIT_5, 1 << 5);
		} else {
			DCAM_REG_MWR(idx, DCAM_CAM_BIN_CFG, BIT_5, 0);
			if (path->scaler_sel == DCAM_SCALER_RAW_DOWNSISER)
				DCAM_REG_MWR(idx,
					DCAM_CAM_BIN_CFG, BIT_3, 1 << 1); /* RDS */
			else {
				DCAM_REG_MWR(idx, DCAM_CAM_BIN_CFG, BIT_3, 0);
				DCAM_REG_MWR(idx, DCAM_CAM_BIN_CFG,
						BIT_2, path->bin_ratio << 1);
			}
		}

		if ((path->in_size.w > path->in_trim.size_x) ||
			(path->in_size.h > path->in_trim.size_y)) {
			DCAM_REG_MWR(idx, DCAM_CAM_BIN_CFG,
							BIT_1, 1 << 1);

			reg_val = (path->in_trim.start_y << 16) |
						path->in_trim.start_x;
			DCAM_REG_WR(idx, DCAM_BIN_CROP_START, reg_val);

			reg_val = (path->in_trim.size_y << 16) |
						path->in_trim.size_x;
			DCAM_REG_WR(idx, DCAM_BIN_CROP_SIZE, reg_val);

			path->out_size.w = path->in_trim.size_x;
			path->out_size.h = path->in_trim.size_y;
		} else {
			DCAM_REG_MWR(idx, DCAM_CAM_BIN_CFG, BIT_1, 0);
			path->out_size.w = path->in_size.w;
			path->out_size.h = path->in_size.h;
		}
#else
		DCAM_REG_MWR(idx, DCAM_CAM_BIN_CFG,
					BIT_1, path->bin_ratio << 1);
		DCAM_REG_MWR(idx, DCAM_CAM_BIN_CFG,
					BIT_3 | BIT_2, (path->scaler_sel & 3) << 2);
		DCAM_REG_MWR(idx, DCAM_CAM_BIN_CFG,
					BIT_16, path->is_slw << 16);
		DCAM_REG_MWR(idx, DCAM_CAM_BIN_CFG,
					BIT_19 | BIT_18 | BIT_17,
					(path->slw_frm_num & 7) << 17);

		if ((path->in_size.w > path->in_trim.size_x) ||
			(path->in_size.h > path->in_trim.size_y)) {

			reg_val = (path->in_trim.start_y << 16) |
						path->in_trim.start_x;
			reg_val |= (1 << 31);
			DCAM_REG_WR(idx, DCAM_BIN_CROP_START, reg_val);

			reg_val = (path->in_trim.size_y << 16) |
						path->in_trim.size_x;
			DCAM_REG_WR(idx, DCAM_BIN_CROP_SIZE, reg_val);

			path->out_size.w = path->in_trim.size_x;
			path->out_size.h = path->in_trim.size_y;
		} else {
			DCAM_REG_MWR(idx, DCAM_CAM_BIN_CFG, BIT_31, 0 << 31);
			path->out_size.w = path->in_size.w;
			path->out_size.h = path->in_size.h;
		}
#endif

		if (path->scaler_sel == DCAM_SCALER_RAW_DOWNSISER) {
			uint32_t cnt;
			uint32_t *ptr = (uint32_t *)path->rds_coeff_buf;
			unsigned long addr = DCAM_RDS_COEFF_TABLE;

			for (cnt = 0; cnt < path->rds_coeff_size; cnt += 4, addr += 4)
				DCAM_REG_WR(idx, addr, *ptr++);

			reg_val = ((path->out_size.h & 0xfff) << 12) |
						(path->out_size.w & 0x1fff);
			DCAM_REG_WR(idx, DCAM_RDS_DES_SIZE, reg_val);
		}

		/* bin_path_en */
		DCAM_REG_MWR(idx, DCAM_CFG, BIT_2, (1 << 2));
		break;

	case DCAM_PATH_VCH2:
		DCAM_REG_WR(idx, DCAM_VCH2_CONTROL, 0x2b << 8 | 0x01);

		/*vch2 path en */
		DCAM_REG_MWR(idx, DCAM_CFG, BIT_4, (1 << 4));
		break;

	case DCAM_PATH_VCH3:
		/*vch3 path en */
		DCAM_REG_MWR(idx, DCAM_CFG, BIT_5, (1 << 5));
		break;

	default:
		break;
	}

	pr_debug("done\n");
	return ret;
}




int dcam_stop_path(void *dcam_handle, struct dcam_path_desc *path)
{
	int ret = 0;
	uint32_t idx;
	uint32_t path_id;
	uint32_t reg_val;
	struct dcam_pipe_dev *dev = NULL;

	pr_debug("enter.");

	if (!path || !dcam_handle) {
		pr_err("error input ptr.\n");
		return -EFAULT;
	}

	dev = (struct dcam_pipe_dev *)dcam_handle;
	idx = dev->idx;
	path_id = path->path_id;

	switch (path_id) {
	case  DCAM_PATH_FULL:
		reg_val = 0;
		DCAM_REG_MWR(idx, DCAM_PATH_STOP, BIT_0, 1);
		DCAM_REG_MWR(idx, DCAM_CFG, BIT_1, (0 << 1));
		break;

	case  DCAM_PATH_BIN:
		DCAM_REG_MWR(idx, DCAM_PATH_STOP, BIT_1, 1 << 1);
		DCAM_REG_MWR(idx, DCAM_CFG, BIT_2, (0 << 2));
		break;

	case  DCAM_PATH_VCH2:
		DCAM_REG_MWR(idx, DCAM_PATH_STOP, BIT_4, 1 << 4);
		DCAM_REG_MWR(idx, DCAM_CFG, BIT_4, (0 << 4));
		break;

	case  DCAM_PATH_VCH3:
		DCAM_REG_MWR(idx, DCAM_PATH_STOP, BIT_5, 1 << 5);
		DCAM_REG_MWR(idx, DCAM_CFG, BIT_5, (0 << 5));
		break;

	default:
		break;
	}

	pr_debug("done\n");
	return ret;
}


int dcam_start_fetch(void *dcam_handle, struct dcam_fetch_info *fetch)
{
	int ret = 0;
	uint32_t pitch;
	uint32_t mod16_pixel, mod16_bytes, mod16_words;
	struct dcam_pipe_dev *dev = NULL;

	pr_debug("enter.\n");

	if (!dcam_handle || !fetch) {
		pr_err("error input ptr.\n");
		return -EFAULT;
	}

	dev = (struct dcam_pipe_dev *)dcam_handle;

	if (fetch->is_loose == 0) {
		pitch = fetch->size.w * 2;
	} else {
		mod16_pixel = fetch->size.w & 0xF;
		mod16_bytes = (mod16_pixel + 3) / 4 * 5;
		mod16_words = (mod16_bytes + 3) / 4;
		pitch = (fetch->size.w >> 4) * 20 + mod16_words * 4;
	}

	DCAM_REG_MWR(dev->idx,
		DCAM_MIPI_CAP_CFG, BIT_0, 1);
	DCAM_REG_MWR(dev->idx,
		DCAM_MIPI_CAP_CFG, (0xF << 16), fetch->pattern);

	DCAM_AXIM_MWR(IMG_FETCH_CTRL,
				BIT_1, fetch->is_loose << 1);
	DCAM_AXIM_MWR(IMG_FETCH_CTRL,
				BIT_3 | BIT_2, fetch->endian << 2);

	DCAM_AXIM_WR(IMG_FETCH_SIZE,
		(fetch->trim.size_y << 16) | (fetch->trim.size_x & 0xffff));
	DCAM_AXIM_WR(IMG_FETCH_X,
		(pitch << 16) | (fetch->trim.start_x & 0xffff));

	DCAM_AXIM_WR(IMG_FETCH_RADDR,
		fetch->addr.addr_ch0 << 3);

	DCAM_AXIM_WR(IMG_FETCH_START, 1);

	pr_info("done.\n");
	return ret;
}

int dcam_path_set_store_frm(
			void *dcam_handle,
			struct dcam_path_desc *path)
{
	int ret = 0;
	uint32_t idx;
	uint32_t path_id;
	struct dcam_pipe_dev *dev = NULL;
	struct camera_frame *frame;
	unsigned long reg_addr;
	struct timespec cur_ts;

	pr_debug("enter. path %d\n", path->path_id);

	if (!path || !dcam_handle) {
		pr_err("error input ptr.\n");
		return -EFAULT;
	}

	dev = (struct dcam_pipe_dev *)dcam_handle;
	idx = dev->idx;
	path_id = path->path_id;

	frame = camera_dequeue(&path->out_buf_queue);
	if (frame == NULL)
		frame = camera_dequeue(
				&path->reserved_buf_queue);

	if (frame == NULL) {
		pr_err("fail to get available output buffer.\n");
		ret = -EINVAL;
		goto no_buf;
	}

	ktime_get_ts(&cur_ts);
	frame->time.tv_sec = cur_ts.tv_sec;
	frame->time.tv_usec = cur_ts.tv_nsec / NSEC_PER_USEC;

	frame->fid = path->frm_cnt;
	ret = camera_enqueue(&path->result_queue, frame);
	if (ret) {
		pr_err("dcam%d path %d output queue overflow.\n",
				idx, path_id);
		ret = -EINVAL;
		goto overflow;
	}
	reg_addr = dcam_store_addr[path->path_id];
	DCAM_REG_WR(idx, reg_addr, frame->buf.iova[0]);

	pr_debug("done. reg %08x,  addr %08x\n",
		 (uint32_t)reg_addr, (uint32_t)frame->buf.iova[0]);

	return 0;

overflow:
	if (frame->is_reserved)
		camera_enqueue(&path->reserved_buf_queue, frame);
	else
		camera_enqueue(&path->out_buf_queue, frame);
no_buf:
	return ret;
}


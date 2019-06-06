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

#include <linux/kernel.h>
#include <linux/vmalloc.h>
#include <video/sprd_mm.h>
#include "isp_slice.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_SLICE: %d " fmt, __LINE__

#define ISP_SLICE_ALIGN_SIZE		2
#define ISP_ALIGNED(size)	((size) & ~(ISP_SLICE_ALIGN_SIZE - 1))

#define SLICE_HEIGHT_NUM			1
#define SLICE_WIDTH_MAX				1408
#define SLICE_SCL_WIDTH_MAX			2304
#define ISP1_SLICE_WIDTH_MAX			640
#define ISP1_SLICE_SCL_WIDTH_MAX		1536
#define YUV_OVERLAP_UP				46
#define YUV_OVERLAP_DOWN			68
#define YUV_OVERLAP_LEFT			74
#define YUV_OVERLAP_RIGHT                      126
/* YUV_OVERLAP_RIGHT_PLUS used to 1/4 deci */
#define YUV_OVERLAP_RIGHT_PLUS			192

#define YUVSCALER_OVERLAP_UP			32
#define YUVSCALER_OVERLAP_DOWN			52
#define YUVSCALER_OVERLAP_LEFT			16
#define YUVSCALER_OVERLAP_RIGHT                        68
/* YUVSCALER_OVERLAP_RIGHT_PLUS used to 1/4 deci */
#define YUVSCALER_OVERLAP_RIGHT_PLUS		134

#define SCL_HOR_MAX_TAP				8
#define SCL_VER_MAX_TAP				16
#define SCL_OVERLAP_UP				(SCL_VER_MAX_TAP/2)
#define SCL_OVERLAP_DOWN			(SCL_VER_MAX_TAP/2+1)
#define SCL_OVERLAP_LEFT			(SCL_HOR_MAX_TAP/2)
#define SCL_OVERLAP_RIGHT			(SCL_HOR_MAX_TAP/2+1)

struct isp_scaler_slice_tmp {
	unsigned int cur_slice_id;
	unsigned int slice_row_num;
	unsigned int slice_col_num;
	unsigned int start_col;
	unsigned int start_row;
	unsigned int end_col;
	unsigned int end_row;
	unsigned int cur_row;
	unsigned int cur_col;
	unsigned int overlap_bad_up;
	unsigned int overlap_bad_down;
	unsigned int overlap_bad_left;
	unsigned int overlap_bad_right;
	unsigned int trim0_end_x;
	unsigned int trim0_end_y;
	unsigned int trim0_start_adjust_x;
	unsigned int trim0_start_adjust_y;
	unsigned int deci_x;
	unsigned int deci_y;
	unsigned int deci_x_align;
	unsigned int deci_y_align;
	unsigned int scaler_out_height_temp;
	unsigned int scaler_out_width_temp;
	unsigned int *scaler_slice;
	unsigned int *scaler_yuv;
};

int fmcu_push_back(unsigned int *p, unsigned int addr,
	unsigned int cmd, unsigned int num, enum isp_channel_idx ch_idx)
{
	p[0] = cmd;
	p[1] = addr;

	if (ch_idx == ISP_CH_1)
		p[1] += ISP_CH1_ADDR_OFFSET;
	num += 2;

	return num;
}

unsigned int noisefilter_24b_shift8(unsigned int seed, unsigned int *data_out)
{
	unsigned int bit_0, bit_1, bit_2, bit_3;
	unsigned int bit_in[8], bit_in8b;
	unsigned int out;
	unsigned int i = 0;

	for (i = 0; i < 8; i++) {
		bit_0 = (seed>>(0+i)) & 0x1;
		bit_1 = (seed>>(1+i)) & 0x1;
		bit_2 = (seed>>(2+i)) & 0x1;
		bit_3 = (seed>>(7+i)) & 0x1;
		bit_in[i] = bit_0^bit_1^bit_2^bit_3;
	}
	bit_in8b = (bit_in[7]<<7) | (bit_in[6]<<6) | (bit_in[5]<<5) |
		(bit_in[4]<<4) | (bit_in[3]<<3) | (bit_in[2]<<2) |
		(bit_in[1]<<1) | bit_in[0];

	out = seed & 0xffffff;
	out = out | (bit_in8b<<24);
	if (data_out)
		*data_out = out;

	out = out>>8;

	return out;
}
void slice_noisefilter_seeds(unsigned int image_width, unsigned int seed0,
	unsigned int *seed1, unsigned int *seed2, unsigned int *seed3)
{
	unsigned int i = 0;

	*seed1 = noisefilter_24b_shift8(seed0, 0);
	*seed2 = seed0;

	for (i = 0; i < image_width; i++)
		*seed2 = noisefilter_24b_shift8(*seed2, 0);

	*seed3 = noisefilter_24b_shift8(*seed2, 0);
}

static void calc_scaler_phase(unsigned int phase, unsigned int factor,
	unsigned int *phase_int, unsigned int *phase_rmd)
{
	phase_int[0] = (unsigned int)(phase/factor);
	phase_rmd[0] = (unsigned int)(phase-factor*phase_int[0]);
}

static int get_slice_size_info(enum isp_id idx, struct slice_param_in *in_ptr,
	unsigned int *h, unsigned int *w)
{
	int rtn = 0;
	unsigned int tempw = 0;
	struct slice_img_size *input = NULL;
	struct slice_img_size *output = NULL;
	unsigned int slice_w_max = SLICE_WIDTH_MAX;
	unsigned int slice_scl_w_max = SLICE_SCL_WIDTH_MAX;


	if (!in_ptr || !h || !w) {
		pr_err("input ptr is NULL!\n");
		rtn = -1;
		goto exit;
	}

	if (idx  ==  ISP_ID_1) {
		slice_w_max = ISP1_SLICE_WIDTH_MAX;
		slice_scl_w_max = ISP1_SLICE_SCL_WIDTH_MAX;
	} else{
		 slice_w_max = SLICE_WIDTH_MAX;
		 slice_scl_w_max = SLICE_SCL_WIDTH_MAX;
	}

	input = &in_ptr->img_size;
	output = &in_ptr->store_frame[SLICE_PATH_CAP].size;

	if (in_ptr->cap_slice_need) {
		if (input->width == output->width &&
			input->height == output->height) {
			*w = (input->width > slice_w_max) ?
				slice_w_max : input->width;
		} else {
			if (output->width > slice_scl_w_max) {
				tempw = (output->width + slice_scl_w_max
					- 1) / slice_scl_w_max;
				*w = input->width / tempw;
				*w = (*w > slice_w_max) ?  slice_w_max : *w;
			} else {
				*w = (input->width > slice_w_max) ?
					slice_w_max : input->width;
			}
		}
		*h = input->height / SLICE_HEIGHT_NUM;
	} else {
		*w = (input->width > slice_w_max) ?  slice_w_max : input->width;
		*h = input->height;
	}

	*w = ISP_ALIGNED(*w);
	*h = ISP_ALIGNED(*h);

exit:
	return rtn;
}
static void get_slice_fetch_pitch(struct slice_pitch *pitch_ptr,
	enum isp_store_format format, unsigned int width)
{

	switch (format) {
	case ISP_STORE_YUV422_3FRAME:
		pitch_ptr->chn0 = width;
		pitch_ptr->chn1 = width >> 1;
		pitch_ptr->chn2 = width >> 1;
		break;
	case ISP_STORE_YUV422_2FRAME:
	case ISP_STORE_YVU422_2FRAME:
	case ISP_STORE_YUV420_2FRAME:
	case ISP_STORE_YVU420_2FRAME:
		pitch_ptr->chn0 = width;
		pitch_ptr->chn1 = width;
		break;
	case ISP_STORE_UYVY:
		pitch_ptr->chn0 = width << 1;
		break;
	case ISP_STORE_RAW10:
		pitch_ptr->chn0 = width << 1;
		break;
	default:
		break;
	}

}

static int set_slice_base_info(enum isp_id idx, struct slice_param_in *in_ptr,
	struct slice_base_info *base_info)
{
	int rtn = 0;
	unsigned int i = 0, j = 0;
	unsigned int img_height, img_width;
	unsigned int slice_height = 0, slice_width = 0;
	unsigned int slice_total_row, slice_total_col, slice_num;

	if (!in_ptr || !base_info) {
		pr_err("input ptr is NULL!\n");
		rtn = -1;
		goto exit;
	}

	rtn = get_slice_size_info(idx, in_ptr, &slice_height, &slice_width);
	if (rtn)
		goto exit;

	pr_debug("fmcu slice idx %d, height %d, width %d\n",
			idx, slice_height, slice_width);
	img_height = in_ptr->img_size.height;
	img_width = in_ptr->img_size.width;
	slice_total_row = (img_height + slice_height - 1) / slice_height;
	slice_total_col = (img_width + slice_width - 1) / slice_width;
	slice_num = slice_total_col * slice_total_row;
	base_info->cur_slice_id = 0;
	base_info->slice_num = slice_num;
	base_info->slice_col_num = slice_total_col;
	base_info->slice_row_num = slice_total_row;
	base_info->yuv_overlap_up = YUV_OVERLAP_UP;
	base_info->yuv_overlap_down = YUV_OVERLAP_DOWN;
	base_info->yuv_overlap_left = YUV_OVERLAP_LEFT;
	base_info->yuv_overlap_right = YUV_OVERLAP_RIGHT;
	base_info->yuvscaler_overlap_right = YUVSCALER_OVERLAP_RIGHT;
	if (in_ptr->scaler_frame[SLICE_PATH_CAP].deci_x > 0) {
		base_info->yuv_overlap_right =
			YUV_OVERLAP_RIGHT_PLUS;
		base_info->yuvscaler_overlap_right =
			YUVSCALER_OVERLAP_RIGHT_PLUS;
	}
	base_info->slice_height = slice_height;
	base_info->slice_width = slice_width;
	base_info->img_height = img_height;
	base_info->img_width = img_width;
	base_info->store_height =
		in_ptr->store_frame[SLICE_PATH_CAP].size.height;
	base_info->store_width =
		in_ptr->store_frame[SLICE_PATH_CAP].size.width;
	base_info->isp_jpg_cowork = 0;
	get_slice_fetch_pitch(&base_info->pitch,
			      in_ptr->fetchYUV_format, base_info->img_width);
	if (base_info->slice_num > SLICE_NUM_MAX)
		pr_info("slice num is too large %d\n", base_info->slice_num);

	pr_debug("slice row %d, col %d, num %d", slice_total_row,
		slice_total_col, slice_num);
	for (i = 0; i < slice_total_row; i++) {
		for (j = 0; j < slice_total_col; j++) {
			struct slice_pos_info temp_win = {0};
			struct slice_overlap_info temp_overlap = {0};

			temp_win.start_col = j * slice_width;
			temp_win.start_row = i * slice_height;
			if (i != 0) {
				temp_win.start_row -= YUV_OVERLAP_UP;
				temp_overlap.overlap_up = YUV_OVERLAP_UP;
			}

			if (j != 0) {
				temp_win.start_col -= YUV_OVERLAP_LEFT;
				temp_overlap.overlap_left = YUV_OVERLAP_LEFT;
			}

			if (i != slice_total_row - 1) {
				temp_win.end_row = (i + 1) * slice_height
					- 1 + YUV_OVERLAP_DOWN;
				temp_overlap.overlap_down = YUV_OVERLAP_DOWN;
			} else {
				temp_win.end_row = img_height - 1;
			}

			if (j != slice_total_col - 1) {
				temp_win.end_col = (j + 1) * slice_width
					- 1 + base_info->yuv_overlap_right;
				temp_overlap.overlap_right =
					base_info->yuv_overlap_right;
			} else {
				temp_win.end_col = img_width - 1;
			}

			base_info->slice_pos_array[i * slice_total_col
				+ j] = temp_win;
			base_info->slice_overlap_array[i * slice_total_col
				+ j] = temp_overlap;
		}
	}

exit:
	return rtn;
}


static int set_slice_fetchyuv_info(struct slice_param_in *in_ptr,
	struct slice_context_info *cxt)
{
	int rtn = 0;
	struct slice_base_info *base_info = NULL;
	struct slice_fetchYUV_info *fetchYUV_info = NULL;
	struct slice_addr *address = NULL;
	unsigned int cur_slice_id, slice_num;
	struct slice_pitch fetchYUV_pitch = {0};
	unsigned int start_col, end_col, start_row, end_row;
	unsigned int ch0_offset = 0;
	unsigned int ch1_offset = 0;
	unsigned int ch2_offset = 0;

	if (!in_ptr || !cxt) {
		pr_err("input handle is NULL!\n");
		rtn = -1;
		goto exit;
	}

	base_info = &cxt->base_info;
	address = &in_ptr->fetchYUV_addr;
	cur_slice_id = base_info->cur_slice_id;
	slice_num = base_info->slice_num;

	get_slice_fetch_pitch(&fetchYUV_pitch,
		in_ptr->fetchYUV_format, base_info->img_width);
	for (; cur_slice_id < slice_num; cur_slice_id++) {
		fetchYUV_info = &cxt->fetchYUV_info[cur_slice_id];
		fetchYUV_info->addr.chn0 = address->chn0;
		fetchYUV_info->addr.chn1 = address->chn1;
		fetchYUV_info->addr.chn2 = address->chn2;
		start_col = base_info->slice_pos_array[cur_slice_id].start_col;
		end_col = base_info->slice_pos_array[cur_slice_id].end_col;
		start_row = base_info->slice_pos_array[cur_slice_id].start_row;
		end_row = base_info->slice_pos_array[cur_slice_id].end_row;
		switch (in_ptr->fetchYUV_format) {
		case ISP_STORE_YUV422_3FRAME:
			ch0_offset = start_row * fetchYUV_pitch.chn0
				+ start_col;
			ch1_offset = start_row * fetchYUV_pitch.chn1
				+ ((start_col + 1) >> 1);
			ch2_offset = start_row * fetchYUV_pitch.chn2
				+ ((start_col + 1) >> 1);
			break;
		case ISP_STORE_UYVY:
			ch0_offset = start_row * fetchYUV_pitch.chn0
				+ start_col * 2;
			break;
		case ISP_STORE_YUV422_2FRAME:
		case ISP_STORE_YVU422_2FRAME:
			ch0_offset = start_row * fetchYUV_pitch.chn0
				+ start_col;
			ch1_offset = start_row * fetchYUV_pitch.chn1
				+ start_col;
			break;
		case ISP_STORE_YUV420_2FRAME:
		case ISP_STORE_YVU420_2FRAME:
			ch0_offset = start_row * fetchYUV_pitch.chn0
				+ start_col;
			ch1_offset = ((start_row * fetchYUV_pitch.chn1
				+ 1) >> 1) + start_col;
			break;
		default:
			break;
		}

		fetchYUV_info->addr.chn0 += ch0_offset;
		fetchYUV_info->addr.chn1 += ch1_offset;
		fetchYUV_info->addr.chn2 += ch2_offset;
		fetchYUV_info->size.height = end_row - start_row + 1;
		fetchYUV_info->size.width = end_col - start_col + 1;
	}
exit:
	return rtn;
}

static int set_slice_dispatchyuv_info(struct slice_context_info *cxt)
{
	int rtn = 0;
	unsigned int cur_slice_id, slice_num;
	unsigned int start_col, end_col, start_row, end_row;
	struct slice_base_info *base_info = NULL;
	struct slice_dispatchYUV_info *dispathYUV_info = NULL;

	if (!cxt) {
		pr_err("input handle is NULL!\n");
		rtn = -1;
		goto exit;
	}

	base_info = &cxt->base_info;
	cur_slice_id = base_info->cur_slice_id;
	slice_num = base_info->slice_num;

	for (; cur_slice_id < slice_num; cur_slice_id++) {
		dispathYUV_info = &cxt->dispatchYUV_info[cur_slice_id];
		start_col = base_info->slice_pos_array[cur_slice_id].start_col;
		end_col = base_info->slice_pos_array[cur_slice_id].end_col;
		start_row = base_info->slice_pos_array[cur_slice_id].start_row;
		end_row = base_info->slice_pos_array[cur_slice_id].end_row;
		dispathYUV_info->size.height = end_row - start_row + 1;
		dispathYUV_info->size.width = end_col - start_col + 1;
	}
exit:
	return rtn;
}

static int set_slice_postcnr_info(struct slice_context_info *cxt)
{
	int rtn = 0;
	unsigned int cur_slice_id, slice_num;
	struct slice_base_info *base_info = NULL;
	struct slice_postcnr_info *postcnr_info = NULL;

	if (!cxt) {
		pr_err("input handle is NULL!\n");
		rtn = -1;
		goto exit;
	}

	base_info = &cxt->base_info;
	cur_slice_id = base_info->cur_slice_id;
	slice_num = base_info->slice_num;

	for (; cur_slice_id < slice_num; cur_slice_id++) {
		postcnr_info = &cxt->postcnr_info[cur_slice_id];
		postcnr_info->start_row_mod4 =
			base_info->slice_pos_array[cur_slice_id].start_row
			& 0x3;
	}
exit:
	return rtn;
}

static int set_slice_ynr_info(struct slice_context_info *cxt)
{
	int rtn = 0;
	unsigned int cur_slice_id, slice_num;
	struct slice_base_info *base_info = NULL;
	struct slice_ynr_info *ynr_info = NULL;

	if (!cxt) {
		pr_err("input handle is NULL!\n");
		rtn = -1;
		goto exit;
	}

	base_info = &cxt->base_info;
	cur_slice_id = base_info->cur_slice_id;
	slice_num = base_info->slice_num;

	for (; cur_slice_id < slice_num; cur_slice_id++) {
		ynr_info = &cxt->ynr_info[cur_slice_id];
		ynr_info->start_col =
			base_info->slice_pos_array[cur_slice_id].start_col;
		ynr_info->start_row =
			base_info->slice_pos_array[cur_slice_id].start_row;
	}
exit:
	return rtn;
}

static void set_path_trim0_info(struct isp_scaler_slice_tmp *slice)
{
	unsigned int start, end;
	struct slice_scaler_info *out =
		(struct slice_scaler_info *)slice->scaler_slice;
	struct slice_scaler_path *in =
		(struct slice_scaler_path *)slice->scaler_yuv;

	/* trim0 x */
	start = slice->start_col+slice->overlap_bad_left;
	end = slice->end_col+1-slice->overlap_bad_right;
	if (slice->slice_col_num == 1) {
		out->trim0_start_x = in->trim0_start_x;
		out->trim0_size_x = in->trim0_size_x;
	} else {
		if (slice->cur_col == 0) {
			out->trim0_start_x = in->trim0_start_x;
			if (slice->trim0_end_x < end)
				out->trim0_size_x = in->trim0_size_x;
			else
				out->trim0_size_x = end - in->trim0_start_x;
		} else if ((slice->slice_col_num - 1) == slice->cur_col) {
			if (in->trim0_start_x > start) {
				out->trim0_start_x =
					in->trim0_start_x - slice->start_col;
				out->trim0_size_x = in->trim0_size_x;
			} else {
				out->trim0_start_x = slice->overlap_bad_left;
				out->trim0_size_x = slice->trim0_end_x-start;
			}
		} else {
			if (in->trim0_start_x < start) {
				out->trim0_start_x = slice->overlap_bad_left;
				if (slice->trim0_end_x < end)
					out->trim0_size_x =
					slice->trim0_end_x-start;
				else
					out->trim0_size_x = end - start;
			} else {
				out->trim0_start_x =
					in->trim0_start_x - slice->start_col;
				if (slice->trim0_end_x < end)
					out->trim0_size_x = in->trim0_size_x;
				else
					out->trim0_size_x =
					end - in->trim0_start_x;
			}
		}
	}

	/* trim0 y */
	start = slice->start_row + slice->overlap_bad_up;
	end = slice->end_row+1 - slice->overlap_bad_down;
	if (slice->slice_row_num == 1) {
		out->trim0_start_y = in->trim0_start_y;
		out->trim0_size_y = in->trim0_size_y;
	} else {
		if (slice->cur_row == 0) {
			out->trim0_start_y = in->trim0_start_y;
			if (slice->trim0_end_y < end)
				out->trim0_size_y = in->trim0_size_y;
			else
				out->trim0_size_y = end - in->trim0_start_y;
		} else if ((slice->slice_row_num - 1) == slice->cur_row) {
			if (in->trim0_start_y < start) {
				out->trim0_start_y = slice->overlap_bad_up;
				out->trim0_size_y = slice->trim0_end_y - start;
			} else {
				out->trim0_start_y =
					in->trim0_start_y - slice->start_row;
				out->trim0_size_y = in->trim0_size_y;
			}
		} else {
			if (in->trim0_start_y < start) {
				out->trim0_start_y = slice->overlap_bad_up;
				if (slice->trim0_end_y < end)
					out->trim0_size_y =
					slice->trim0_end_y-start;
				else
					out->trim0_size_y = end - start;
			} else {
				out->trim0_start_y =
					in->trim0_start_y - slice->start_row;
				if (slice->trim0_end_y < end)
					out->trim0_size_y = in->trim0_size_y;
				else
					out->trim0_size_y =
					end - in->trim0_start_y;
			}
		}
	}

}

static void set_path_deci_info(struct isp_scaler_slice_tmp *slice)
{
	unsigned int start;
	struct slice_scaler_info *out =
		(struct slice_scaler_info *)slice->scaler_slice;
	struct slice_scaler_path *in =
		(struct slice_scaler_path *)slice->scaler_yuv;

	slice->deci_x = in->deci_x;
	slice->deci_y = in->deci_y;
	slice->deci_x_align = slice->deci_x * 2;

	start = slice->start_col + slice->overlap_bad_left;
	if (in->trim0_start_x >= slice->start_col &&
		(in->trim0_start_x <= slice->end_col+1)) {
		out->trim0_size_x = out->trim0_size_x/
			slice->deci_x_align*slice->deci_x_align;
	} else {
		slice->trim0_start_adjust_x = (start+slice->deci_x_align-1)/
			slice->deci_x_align*slice->deci_x_align-start;
		out->trim0_start_x += slice->trim0_start_adjust_x;
		out->trim0_size_x -= slice->trim0_start_adjust_x;
		out->trim0_size_x = out->trim0_size_x/
			slice->deci_x_align*slice->deci_x_align;
	}

	if (in->odata_mode == 0)
		slice->deci_y_align = slice->deci_y;		/* 422 */
	else
		slice->deci_y_align = slice->deci_y * 2;	/* 420 */

	start = slice->start_row + slice->overlap_bad_up;
	if (in->trim0_start_y >= slice->start_row &&
		(in->trim0_start_y <= slice->end_row+1)) {
		out->trim0_size_y = out->trim0_size_y/
			slice->deci_y_align*slice->deci_y_align;
	} else {
		slice->trim0_start_adjust_y = (start+slice->deci_y_align-1)/
			slice->deci_y_align*slice->deci_y_align-start;
		out->trim0_start_y += slice->trim0_start_adjust_y;
		out->trim0_size_y -= slice->trim0_start_adjust_y;
		out->trim0_size_y = out->trim0_size_y/
			slice->deci_y_align*slice->deci_y_align;
	}

	out->scaler_in_width = out->trim0_size_x/slice->deci_x;
	out->scaler_in_height = out->trim0_size_y/slice->deci_y;
}

void set_path_scaler_info(struct isp_scaler_slice_tmp *slice)
{
	struct slice_scaler_info *out =
		(struct slice_scaler_info *)slice->scaler_slice;
	struct slice_scaler_path *in =
		(struct slice_scaler_path *)slice->scaler_yuv;
	unsigned int scl_factor_in, scl_factor_out;
	unsigned int  initial_phase, last_phase, phase_in;
	unsigned int phase_tmp, scl_temp, out_tmp;
	unsigned int start, end;
	unsigned int tap_hor, tap_ver, tap_hor_uv, tap_ver_uv;
	unsigned int tmp, n;

	if (in->scaler_bypass == 0) {
		scl_factor_in = in->scaler_factor_in/2;
		scl_factor_out = in->scaler_factor_out/2;
		initial_phase = 0;
		last_phase = initial_phase+
			scl_factor_in*(in->scaler_out_width/2-1);
		tap_hor = 8;
		tap_hor_uv = tap_hor / 2;

		start = slice->start_col+slice->overlap_bad_left+
			slice->deci_x_align-1;
		end = slice->end_col+1-slice->overlap_bad_right+
			slice->deci_x_align-1;
		if (in->trim0_start_x >= slice->start_col &&
			(in->trim0_start_x <= slice->end_col+1)) {
			phase_in = 0;
			if (out->scaler_in_width ==
				in->trim0_size_x/slice->deci_x)
				phase_tmp = last_phase;
			else
				phase_tmp = (out->scaler_in_width/2-
				tap_hor_uv/2)*scl_factor_out -
				scl_factor_in/2 - 1;
			out_tmp = (phase_tmp - phase_in)/scl_factor_in+1;
			out->scaler_out_width = out_tmp*2;
		} else {
			phase_in = (tap_hor_uv/2)*scl_factor_out;
			if (slice->cur_col == slice->slice_col_num - 1) {
				phase_tmp = last_phase-
					((in->trim0_size_x/2)/slice->deci_x-
					out->scaler_in_width/2)*scl_factor_out;
				out_tmp = (phase_tmp-phase_in)/scl_factor_in+1;
				out->scaler_out_width = out_tmp*2;
				phase_in = phase_tmp-(out_tmp-1)*scl_factor_in;
			} else {
				if (slice->trim0_end_x >= slice->start_col
					&& (slice->trim0_end_x <= slice->end_col
					+1-slice->overlap_bad_right)) {
					phase_tmp = last_phase-
					((in->trim0_size_x/2)/slice->deci_x-
					out->scaler_in_width/2)*scl_factor_out;
					out_tmp = (phase_tmp-phase_in)/
						scl_factor_in+1;
					out->scaler_out_width = out_tmp*2;
					phase_in = phase_tmp-(out_tmp-1)
						*scl_factor_in;
				} else {
					initial_phase = ((((start/
					slice->deci_x_align*slice->deci_x_align
					-in->trim0_start_x)/slice->deci_x)/2+
					(tap_hor_uv/2))*(scl_factor_out)+
					(scl_factor_in-1))/scl_factor_in*
					scl_factor_in;
					slice->scaler_out_width_temp =
					((last_phase-initial_phase)/
					scl_factor_in+1)*2;

					scl_temp = ((end/slice->deci_x_align*
					slice->deci_x_align-in->trim0_start_x)/
					slice->deci_x)/2;
					last_phase = ((scl_temp-tap_hor_uv/2)*
					(scl_factor_out)-scl_factor_in/2-1)/
					scl_factor_in*scl_factor_in;

					out_tmp = (last_phase-initial_phase)/
						scl_factor_in+1;
					out->scaler_out_width = out_tmp*2;
					phase_in = initial_phase-(((start/
					slice->deci_x_align*slice->deci_x_align-
					in->trim0_start_x)/slice->deci_x)/2)*
					scl_factor_out;
				}
			}
		}

		calc_scaler_phase(phase_in*4, scl_factor_out*2,
			&out->scaler_ip_int, &out->scaler_ip_rmd);
		calc_scaler_phase(phase_in, scl_factor_out,
			&out->scaler_cip_int, &out->scaler_cip_rmd);

		scl_factor_in = in->scaler_ver_factor_in;
		scl_factor_out = in->scaler_ver_factor_out;
		initial_phase = 0;
		last_phase = initial_phase+
			scl_factor_in*(in->scaler_out_height-1);
		tap_ver = in->scaler_y_ver_tap > in->scaler_uv_ver_tap ?
			in->scaler_y_ver_tap : in->scaler_uv_ver_tap;
		tap_ver += 2;
		tap_ver_uv = tap_ver;

		start = slice->start_row+slice->overlap_bad_up+
			slice->deci_y_align-1;
		end = slice->end_row+1-slice->overlap_bad_down+
			slice->deci_y_align-1;
		if (in->trim0_start_y >= slice->start_row &&
			(in->trim0_start_y <= slice->end_row+1)) {
			phase_in = 0;
			if (out->scaler_in_height ==
				in->trim0_size_y/slice->deci_y)
				phase_tmp = last_phase;
			else
				phase_tmp = (out->scaler_in_height-
				tap_ver_uv/2)*scl_factor_out-1;
			out_tmp = (phase_tmp-phase_in)/scl_factor_in+1;
			if (out_tmp%2 == 1)
				out_tmp -= 1;
			out->scaler_out_height = out_tmp;
		} else {
			phase_in = (tap_ver_uv/2)*scl_factor_out;
			if (slice->cur_row == slice->slice_row_num-1) {
				phase_tmp = last_phase-
					(in->trim0_size_y/slice->deci_y-
					out->scaler_in_height)*scl_factor_out;
				out_tmp = (phase_tmp-phase_in)/scl_factor_in+1;
				if (out_tmp%2 == 1)
					out_tmp -= 1;
				if (in->odata_mode == 1 && out_tmp%4 != 0)
					out_tmp = out_tmp/4*4;
				out->scaler_out_height = out_tmp;
				phase_in = phase_tmp-(out_tmp-1)*scl_factor_in;
			} else {
				if (slice->trim0_end_y >= slice->start_row &&
					(slice->trim0_end_y <= slice->end_row+1
					-slice->overlap_bad_down)) {
					phase_tmp = last_phase-
					(in->trim0_size_y/slice->deci_y-
					out->scaler_in_height)*scl_factor_out;
					out_tmp = (phase_tmp-phase_in)/
						scl_factor_in+1;
					if (out_tmp%2 == 1)
						out_tmp -= 1;
					if (in->odata_mode == 1 &&
						out_tmp % 4 != 0)
						out_tmp = out_tmp/4*4;
					out->scaler_out_height = out_tmp;
					phase_in = phase_tmp-(out_tmp-1)
						*scl_factor_in;
				} else {
					initial_phase = (((start/
					slice->deci_y_align*slice->deci_y_align
					-in->trim0_start_y)/slice->deci_y+
					(tap_ver_uv/2))*(scl_factor_out)+
					(scl_factor_in-1))/(scl_factor_in*2)
					*(scl_factor_in*2);
					slice->scaler_out_height_temp =
						(last_phase-initial_phase)/
						scl_factor_in+1;
					scl_temp = (end/slice->deci_y_align*
					slice->deci_y_align-in->trim0_start_y)/
					slice->deci_y;
					last_phase = ((scl_temp-tap_ver_uv/2)*
					(scl_factor_out)-1)/scl_factor_in
					*scl_factor_in;
					out_tmp = (last_phase-initial_phase)/
						scl_factor_in+1;
					if (out_tmp%2 == 1)
						out_tmp -= 1;
					if (in->odata_mode == 1 &&
						out_tmp%4 != 0)
						out_tmp = out_tmp/4*4;
					out->scaler_out_height = out_tmp;
					phase_in = initial_phase-(start/
					slice->deci_y_align*slice->deci_y_align-
					in->trim0_start_y)/slice->deci_y
					*scl_factor_out;
				}
			}
		}

		calc_scaler_phase(phase_in, scl_factor_out,
			&out->scaler_ip_int_ver, &out->scaler_ip_rmd_ver);
		if (in->odata_mode == 1) {
			phase_in /= 2;
			scl_factor_out /= 2;
		}
		calc_scaler_phase(phase_in, scl_factor_out,
			&out->scaler_cip_int_ver, &out->scaler_cip_rmd_ver);

		if (out->scaler_ip_int >= 16) {
			tmp = out->scaler_ip_int;
			n = (tmp >> 3) - 1;
			out->trim0_start_x += 8*n*slice->deci_x;
			out->trim0_size_x -= 8*n*slice->deci_x;
			out->scaler_ip_int -= 8*n;
			out->scaler_cip_int -= 4*n;
		}
		if (out->scaler_ip_int >= 16)
			pr_err("Horizontal slice initial phase overflowed!\n");
		if (out->scaler_ip_int_ver >= 16) {
			tmp = out->scaler_ip_int_ver;
			n = (tmp >> 3) - 1;
			out->trim0_start_y += 8*n*slice->deci_y;
			out->trim0_size_y -= 8*n*slice->deci_y;
			out->scaler_ip_int_ver -= 8*n;
			out->scaler_cip_int_ver -= 8*n;
		}
		if (out->scaler_ip_int_ver >= 16)
			pr_err("Vertical slice initial phase overflowed!\n");
	} else {
		out->scaler_out_width = out->scaler_in_width;
		out->scaler_out_height = out->scaler_in_height;
		start = slice->start_col+slice->overlap_bad_left+
			slice->trim0_start_adjust_x+slice->deci_x_align-1;
		slice->scaler_out_width_temp = (in->trim0_size_x-(start/
			slice->deci_x_align*slice->deci_x_align-
			in->trim0_start_x))/slice->deci_x;
		start = slice->start_row+slice->overlap_bad_up+
			slice->trim0_start_adjust_y+slice->deci_y_align-1;
		slice->scaler_out_height_temp = (in->trim0_size_y-(start/
			slice->deci_y_align*slice->deci_y_align-
			in->trim0_start_y))/slice->deci_y;
	}
}

void set_path_trim1_info(struct isp_scaler_slice_tmp *slice,
	struct slice_scaler_info *scaler_info)
{
	struct slice_scaler_info *out =
		(struct slice_scaler_info *)slice->scaler_slice;
	struct slice_scaler_path *in =
		(struct slice_scaler_path *)slice->scaler_yuv;
	uint32_t trim_sum_x = 0;
	uint32_t trim_sum_y = 0;
	uint32_t pix_align = 8;
	uint32_t i = 0;

	if (in->trim0_start_x >= slice->start_col &&
		(in->trim0_start_x <= slice->end_col+1)) {
		out->trim1_start_x = 0;
		if (out->scaler_in_width == in->trim0_size_x)
			out->trim1_size_x = out->scaler_out_width;
		else
			out->trim1_size_x = out->scaler_out_width/
			pix_align*pix_align;
	} else {
		for (i = 1; i < slice->cur_col+1; i++)
			trim_sum_x +=
			scaler_info[slice->cur_slice_id-i].trim1_size_x;

		if (slice->cur_col == slice->slice_col_num - 1) {
			out->trim1_size_x = in->scaler_out_width-trim_sum_x;
			out->trim1_start_x = out->scaler_out_width-
				out->trim1_size_x;
		} else {
			if (slice->trim0_end_x >= slice->start_col &&
				(slice->trim0_end_x <= slice->end_col+1
				-slice->overlap_bad_right)) {
				out->trim1_size_x = in->scaler_out_width
					-trim_sum_x;
				out->trim1_start_x = out->scaler_out_width-
					out->trim1_size_x;
			} else {
				out->trim1_start_x =
					slice->scaler_out_width_temp-
					(in->scaler_out_width-trim_sum_x);
				out->trim1_size_x = (out->scaler_out_width-
					out->trim1_start_x)/pix_align*pix_align;
			}
		}
	}

	if (in->trim0_start_y >= slice->start_row &&
		(in->trim0_start_y <= slice->end_row+1)) {
		out->trim1_start_y = 0;
		if (out->scaler_in_height == in->trim0_size_y)
			out->trim1_size_y = out->scaler_out_height;
		else
			out->trim1_size_y = out->scaler_out_height/
			pix_align*pix_align;
	} else {
		for (i = 1; i < slice->cur_row+1; i++)
			trim_sum_y += scaler_info[slice->cur_slice_id-
			i*slice->slice_col_num].trim1_size_y;

		if (slice->cur_row == slice->slice_row_num - 1) {
			out->trim1_size_y = in->scaler_out_height-trim_sum_y;
			out->trim1_start_y = out->scaler_out_height-
				out->trim1_size_y;
		} else {
			if (slice->trim0_end_y >= slice->start_row &&
				(slice->trim0_end_y <= slice->end_row+1
				-slice->overlap_bad_down)) {
				out->trim1_size_y = in->scaler_out_height
					-trim_sum_y;
				out->trim1_start_y = out->scaler_out_height-
					out->trim1_size_y;
			} else {
				out->trim1_start_y =
					slice->scaler_out_height_temp-
					(in->scaler_out_height-trim_sum_y);
				out->trim1_size_y = (out->scaler_out_height-
					out->trim1_start_y)/pix_align*pix_align;
			}
		}
	}
}

static int set_path_info(struct slice_scaler_info *scaler_info,
	struct slice_scaler_path *scaler_frame,
	struct slice_base_info *base_info,
	unsigned int row, unsigned int col)
{
	int rtn = 0;
	unsigned int cur_slice_id;
	struct isp_scaler_slice_tmp slice = {0};

	if (!scaler_info || !scaler_frame || !base_info) {
		pr_err("input ptr is NULL!\n");
		rtn = -1;
		goto exit;
	}

	cur_slice_id = base_info->cur_slice_id;
	slice.cur_slice_id = cur_slice_id;
	slice.cur_col = col;
	slice.cur_row = row;
	slice.slice_col_num = base_info->slice_col_num;
	slice.slice_row_num = base_info->slice_row_num;
	slice.start_col = base_info->slice_pos_array[cur_slice_id].start_col;
	slice.end_col = base_info->slice_pos_array[cur_slice_id].end_col;
	slice.start_row = base_info->slice_pos_array[cur_slice_id].start_row;
	slice.end_row = base_info->slice_pos_array[cur_slice_id].end_row;
	slice.trim0_end_x = scaler_frame->trim0_start_x +
		scaler_frame->trim0_size_x;
	slice.trim0_end_y = scaler_frame->trim0_start_y +
		scaler_frame->trim0_size_y;
	slice.overlap_bad_up = base_info->yuv_overlap_up -
		YUVSCALER_OVERLAP_UP;
	slice.overlap_bad_down = base_info->yuv_overlap_down -
		YUVSCALER_OVERLAP_DOWN;
	slice.overlap_bad_left = base_info->yuv_overlap_left -
		YUVSCALER_OVERLAP_LEFT;
	slice.overlap_bad_right = base_info->yuv_overlap_right -
		base_info->yuvscaler_overlap_right;
	slice.scaler_slice = (unsigned int *)&scaler_info[cur_slice_id];
	slice.scaler_yuv = (unsigned int *)scaler_frame;

	set_path_trim0_info(&slice);
	set_path_deci_info(&slice);
	set_path_scaler_info(&slice);
	set_path_trim1_info(&slice, scaler_info);

	scaler_info[cur_slice_id].src_size_x = slice.end_col -
		slice.start_col + 1;
	scaler_info[cur_slice_id].src_size_y = slice.end_row -
		slice.start_row + 1;
	scaler_info[cur_slice_id].dst_size_x =
		scaler_info[cur_slice_id].scaler_out_width;
	scaler_info[cur_slice_id].dst_size_y =
		scaler_info[cur_slice_id].scaler_out_height;

exit:
	return rtn;
}

static int set_slice_scaler_info(struct slice_param_in *in_ptr,
	struct slice_context_info *cxt)
{
	int rtn = 0;
	struct slice_base_info *base_info = NULL;
	struct slice_scaler_info *scaler_info = NULL;
	struct slice_scaler_path *scaler_frame = NULL;
	unsigned int slice_col_num, slice_row_num;
	unsigned int r = 0, c = 0;

	if (!in_ptr || !cxt) {
		pr_err("input ptr is NULL!\n");
		rtn = -1;
		goto exit;
	}

	base_info = &cxt->base_info;
	slice_col_num = base_info->slice_col_num;
	slice_row_num = base_info->slice_row_num;

	for (r = 0; r < slice_row_num; r++) {
		for (c = 0; c < slice_col_num; c++) {
			base_info->cur_slice_id = r * slice_col_num + c;
			if (in_ptr->pre_slice_need == 1) {
				scaler_info = cxt->scaler_info[SLICE_PATH_PRE];
				scaler_frame =
					&in_ptr->scaler_frame[SLICE_PATH_PRE];
				set_path_info(scaler_info,
					scaler_frame, base_info, r, c);
				if (rtn)
					goto exit;
			}

			if (in_ptr->vid_slice_need == 1) {
				scaler_info = cxt->scaler_info[SLICE_PATH_VID];
				scaler_frame =
					&in_ptr->scaler_frame[SLICE_PATH_VID];
				set_path_info(scaler_info,
					scaler_frame, base_info, r, c);
				if (rtn)
					goto exit;
			}

			if (in_ptr->cap_slice_need == 1) {
				scaler_info = cxt->scaler_info[SLICE_PATH_CAP];
				scaler_frame =
					&in_ptr->scaler_frame[SLICE_PATH_CAP];
				set_path_info(scaler_info,
					scaler_frame, base_info, r, c);
				if (rtn)
					goto exit;
			}
		}
	}

	base_info->cur_slice_id = 0;

exit:
	return rtn;
}

static int set_slice_noisefliter_info(struct slice_context_info *cxt)
{
	int rtn = 0;
	unsigned int cur_slice_id, slice_num;
	unsigned int slice_width;
	struct slice_base_info *base_info = NULL;
	struct slice_noisefilter_info *noisefilter_info = NULL;
	struct slice_scaler_info *scaler_info = NULL;

	if (!cxt) {
		pr_err("input handle is NULL!\n");
		rtn = -1;
		goto exit;
	}

	base_info = &cxt->base_info;
	cur_slice_id = base_info->cur_slice_id;
	slice_num = base_info->slice_num;
	scaler_info = cxt->scaler_info[SLICE_PATH_CAP];

	for (; cur_slice_id < slice_num; cur_slice_id++) {
		noisefilter_info = &cxt->noisefilter_info[cur_slice_id];
		slice_width = scaler_info[cur_slice_id].trim1_size_x;
		noisefilter_info->seed0 = 0xff;
		slice_noisefilter_seeds(slice_width, noisefilter_info->seed0,
			&noisefilter_info->seed1, &noisefilter_info->seed2,
			&noisefilter_info->seed3);
		noisefilter_info->seed_int = 1;
	}
exit:
	return rtn;
}

static int get_slice_store_pitch(struct slice_pitch *pitch_ptr,
	enum isp_store_format format, unsigned int width)
{

	switch (format) {
	case ISP_STORE_YUV422_3FRAME:
		pitch_ptr->chn0 = width;
		pitch_ptr->chn1 = width >> 1;
		pitch_ptr->chn2 = width >> 1;
		break;
	case ISP_STORE_YUV422_2FRAME:
	case ISP_STORE_YVU422_2FRAME:
	case ISP_STORE_YUV420_2FRAME:
	case ISP_STORE_YVU420_2FRAME:
		pitch_ptr->chn0 = width;
		pitch_ptr->chn1 = width;
		break;
	case ISP_STORE_UYVY:
		pitch_ptr->chn0 = width << 1;
		break;
	case ISP_STORE_RAW10:
		pitch_ptr->chn0 = width << 1;
		break;
	default:
		break;
	}

	return 0;
}

static int set_store_info(struct slice_store_info *store_info,
	struct slice_store_path *store_frame,
	struct slice_base_info *base_info,
	struct slice_scaler_info *scaler_info, unsigned int scl_bypass)
{
	int rtn = 0;
	unsigned int cur_slice_id, cur_slice_row;
	unsigned int scl_out_width, scl_out_height;
	unsigned int overlap_left, overlap_up,
		overlap_right, overlap_down;
	unsigned int start_col, end_col, start_row, end_row;
	unsigned int start_row_out, start_col_out;
	unsigned int tmp_slice_id;
	unsigned int ch0_offset = 0;
	unsigned int ch1_offset = 0;
	unsigned int ch2_offset = 0;
	struct slice_pitch store_pitch = {0};
	struct slice_addr *address = NULL;

	if (!store_info || !store_frame || !base_info || !scaler_info) {
		pr_err("input ptr is NULL!\n");
		rtn = -1;
		goto exit;
	}

	cur_slice_id = base_info->cur_slice_id;
	address = &store_frame->addr;

	rtn = get_slice_store_pitch(&store_pitch, store_frame->format,
		store_frame->size.width);
	for (; cur_slice_id < base_info->slice_num; cur_slice_id++) {
		cur_slice_row = cur_slice_id/base_info->slice_col_num;
		store_info[cur_slice_id].addr.chn0 = address->chn0;
		store_info[cur_slice_id].addr.chn1 = address->chn1;
		store_info[cur_slice_id].addr.chn2 = address->chn2;
		if (scl_bypass == 0) {
			scl_out_width = scaler_info[cur_slice_id].trim1_size_x;
			scl_out_height = scaler_info[cur_slice_id].trim1_size_y;
			overlap_left = 0;
			overlap_up = 0;
			overlap_down = 0;
			overlap_right = 0;

			store_info[cur_slice_id].size.width =
				scl_out_width-overlap_left-overlap_right;
			store_info[cur_slice_id].size.height =
				scl_out_height-overlap_up-overlap_down;

			tmp_slice_id = cur_slice_id;
			start_col_out = 0;
			while ((int)(tmp_slice_id - 1) >=
				(int)(cur_slice_row*base_info->slice_col_num)) {
				tmp_slice_id--;
				start_col_out +=
					store_info[tmp_slice_id].size.width;
			}

			tmp_slice_id = cur_slice_id;
			start_row_out = 0;
			while ((int)(tmp_slice_id-
				base_info->slice_col_num) >= 0) {
				tmp_slice_id -= base_info->slice_col_num;
				start_row_out +=
					store_info[tmp_slice_id].size.height;
			}

			store_info[cur_slice_id].border.left_border = 0;
			store_info[cur_slice_id].border.right_border = 0;
			store_info[cur_slice_id].border.up_border = 0;
			store_info[cur_slice_id].border.down_border = 0;
		} else {
			start_col = base_info->slice_pos_array
				[cur_slice_id].start_col;
			end_col = base_info->slice_pos_array
				[cur_slice_id].end_col;
			start_row = base_info->slice_pos_array
				[cur_slice_id].start_row;
			end_row = base_info->slice_pos_array
				[cur_slice_id].end_row;
			overlap_left = base_info->slice_overlap_array
				[cur_slice_id].overlap_left;
			overlap_up = base_info->slice_overlap_array
				[cur_slice_id].overlap_up;
			overlap_right = base_info->slice_overlap_array
				[cur_slice_id].overlap_right;
			overlap_down = base_info->slice_overlap_array
				[cur_slice_id].overlap_down;
			start_row_out = start_row + overlap_up;
			start_col_out = start_col + overlap_left;
			store_info[cur_slice_id].size.height =
				end_row-start_row+1-overlap_up-overlap_down;
			store_info[cur_slice_id].size.width =
				end_col-start_col+1-overlap_left-overlap_right;
			store_info[cur_slice_id].border.left_border =
				overlap_left;
			store_info[cur_slice_id].border.right_border =
				overlap_right;
			store_info[cur_slice_id].border.up_border =
				overlap_up;
			store_info[cur_slice_id].border.down_border =
				overlap_down;
		}

		switch (store_frame->format) {
		case ISP_STORE_YUV422_3FRAME:
			ch0_offset = start_row_out*store_pitch.chn0+
				start_col_out;
			ch1_offset = start_row_out*store_pitch.chn1+
				((start_col_out+1)>>1);
			ch2_offset = start_row_out*store_pitch.chn2+
				((start_col_out+1)>>1);
			break;
		case ISP_STORE_UYVY:
			ch0_offset = start_row_out*store_pitch.chn0+
				start_col_out*2;
			break;
		case ISP_STORE_YUV422_2FRAME:
		case ISP_STORE_YVU422_2FRAME:
			ch0_offset = start_row_out*store_pitch.chn0+
				start_col_out;
			ch1_offset = start_row_out*store_pitch.chn1+
				start_col_out;
			break;
		case ISP_STORE_YUV420_2FRAME:
		case ISP_STORE_YVU420_2FRAME:
			ch0_offset = start_row_out*store_pitch.chn0+
				start_col_out;
			ch1_offset = ((start_row_out*store_pitch.chn1+1)>>1)+
				start_col_out;
			break;
		case ISP_STORE_YUV420_3FRAME:
			ch0_offset = start_row_out*store_pitch.chn0+
				start_col_out;
			ch1_offset = ((start_row_out*store_pitch.chn1+1)>>1)+
				((start_col_out+1)>>1);
			ch2_offset = ((start_row_out*store_pitch.chn2+1)>>1)+
				((start_col_out+1)>>1);
			break;
		default:
			break;
		}

		store_info[cur_slice_id].addr.chn0 += ch0_offset;
		store_info[cur_slice_id].addr.chn1 += ch1_offset;
		store_info[cur_slice_id].addr.chn2 += ch2_offset;
	}

exit:
	return rtn;
}

static int set_slice_store_info(struct slice_param_in *in_ptr,
	struct slice_context_info *cxt)
{
	int rtn = 0;
	unsigned int scl_bypass;
	struct slice_base_info *base_info = NULL;
	struct slice_store_info *store_info = NULL;
	struct slice_store_path *store_frame = NULL;
	struct slice_scaler_info *scaler_info = NULL;

	if (!in_ptr || !cxt) {
		pr_err("input ptr is NULL!\n");
		rtn = -1;
		goto exit;
	}

	base_info = &cxt->base_info;
	if (in_ptr->pre_slice_need == 1) {
		store_info = cxt->store_info[SLICE_PATH_PRE];
		scaler_info = cxt->scaler_info[SLICE_PATH_PRE];
		store_frame = &in_ptr->store_frame[SLICE_PATH_PRE];
		scl_bypass = in_ptr->scaler_frame[SLICE_PATH_PRE].scaler_bypass;
		set_store_info(store_info, store_frame, base_info,
			scaler_info, scl_bypass);
		if (rtn)
			goto exit;
	}

	if (in_ptr->vid_slice_need == 1) {
		store_info = cxt->store_info[SLICE_PATH_VID];
		scaler_info = cxt->scaler_info[SLICE_PATH_VID];
		store_frame = &in_ptr->store_frame[SLICE_PATH_VID];
		scl_bypass = in_ptr->scaler_frame[SLICE_PATH_VID].scaler_bypass;
		set_store_info(store_info, store_frame, base_info,
			scaler_info, scl_bypass);
		if (rtn)
			goto exit;
	}

	if (in_ptr->cap_slice_need == 1) {
		store_info = cxt->store_info[SLICE_PATH_CAP];
		scaler_info = cxt->scaler_info[SLICE_PATH_CAP];
		store_frame = &in_ptr->store_frame[SLICE_PATH_CAP];
		scl_bypass = in_ptr->scaler_frame[SLICE_PATH_CAP].scaler_bypass;
		set_store_info(store_info, store_frame, base_info,
			scaler_info, scl_bypass);
		if (rtn)
			goto exit;
	}

exit:
	return rtn;
}

static int set_fmcu_fetchyuv(unsigned int *fmcu_buf,
	struct slice_fetchYUV_info *fetch_info,
	unsigned int num, enum isp_id idx)
{
	unsigned int addr = 0, cmd = 0;

	addr = ISP_GET_REG(idx, ISP_FETCH2_SLICE_SIZE);
	cmd = ((fetch_info->size.height & 0xFFFF) << 16) |
		(fetch_info->size.width & 0xFFFF);
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);
	addr = ISP_GET_REG(idx, ISP_FETCH2_SLICE_Y_ADDR);
	cmd = fetch_info->addr.chn0;
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);
	addr = ISP_GET_REG(idx, ISP_FETCH2_SLICE_U_ADDR);
	cmd = fetch_info->addr.chn1;
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);
	addr = ISP_GET_REG(idx, ISP_FETCH2_SLICE_V_ADDR);
	cmd = fetch_info->addr.chn2;
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);

	return num;
}

static int set_fmcu_dispatchyuv(unsigned int *fmcu_buf,
	struct slice_dispatchYUV_info *dispatchYUV_info,
	unsigned int num, enum isp_id idx)
{
	unsigned int addr = 0, cmd = 0;

	addr = ISP_GET_REG(idx, ISP_DISPATCH_YUV_CH1_SIZE);
	cmd = ((dispatchYUV_info->size.height & 0xFFFF) << 16) |
		(dispatchYUV_info->size.width & 0xFFFF);
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);

	return num;
}

static int set_fmcu_postcnr(unsigned int *fmcu_buf,
	struct slice_postcnr_info *postcnr_info,
	unsigned int num, enum isp_id idx)
{
	unsigned int addr = 0, cmd = 0;

	addr = ISP_GET_REG(idx, ISP_POSTCDN_START_ROW_MOD4);
	cmd = postcnr_info->start_row_mod4 & 0x3;
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);

	return num;
}

static int set_fmcu_ynr(unsigned int *fmcu_buf,
	struct slice_ynr_info *ynr_info, unsigned int num, enum isp_id idx)
{
	unsigned int addr = 0, cmd = 0;

	addr = ISP_GET_REG(idx, ISP_YNR_CFG11);
	cmd = ((ynr_info->start_col & 0xFFFF) << 16) |
		(ynr_info->start_row & 0xFFFF);
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);

	return num;
}

static int set_fmcu_noisefilter(unsigned int *fmcu_buf,
	struct slice_noisefilter_info *noisefilter_info,
	unsigned int num, enum isp_id idx)
{
	unsigned int addr = 0, cmd = 0;

	addr = ISP_GET_REG(idx, ISP_YUV_NF_SEED0);
	cmd = noisefilter_info->seed0 & 0xFFFFFF;
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);
	addr = ISP_GET_REG(idx, ISP_YUV_NF_SEED1);
	cmd = noisefilter_info->seed1 & 0xFFFFFF;
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);
	addr = ISP_GET_REG(idx, ISP_YUV_NF_SEED2);
	cmd = noisefilter_info->seed2 & 0xFFFFFF;
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);
	addr = ISP_GET_REG(idx, ISP_YUV_NF_SEED3);
	cmd = noisefilter_info->seed3 & 0xFFFFFF;
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);
	addr = ISP_GET_REG(idx, ISP_YUV_NF_SEED_INIT);
	cmd = noisefilter_info->seed_int;
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);

	return num;
}

static int set_fmcu_scaler(unsigned int *fmcu_buf,
	struct slice_scaler_info *scaler_info,
	unsigned int num, enum isp_id idx, unsigned int base)
{
	uint32_t addr = 0, cmd = 0;

	addr = ISP_GET_REG(idx, ISP_SCALER_SRC_SIZE) + base;
	cmd = (scaler_info->src_size_x & 0x3FFF) |
		((scaler_info->src_size_y & 0x3FFF) << 16);
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);
	addr = ISP_GET_REG(idx, ISP_SCALER_DES_SIZE) + base;
	cmd = (scaler_info->dst_size_x & 0x3FFF) |
		((scaler_info->dst_size_y & 0x3FFF) << 16);
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);
	addr = ISP_GET_REG(idx, ISP_SCALER_TRIM0_START) + base;
	cmd = (scaler_info->trim0_start_x & 0x1FFF) |
		((scaler_info->trim0_start_y & 0x1FFF) << 16);
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);
	addr = ISP_GET_REG(idx, ISP_SCALER_TRIM0_SIZE) + base;
	cmd = (scaler_info->trim0_size_x & 0x1FFF) |
		((scaler_info->trim0_size_y & 0x1FFF) << 16);
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);
	addr = ISP_GET_REG(idx, ISP_SCALER_IP) + base;
	cmd = (scaler_info->scaler_ip_rmd & 0x1FFF) |
		((scaler_info->scaler_ip_int & 0xF) << 16);
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);
	addr = ISP_GET_REG(idx, ISP_SCALER_CIP) + base;
	cmd = (scaler_info->scaler_cip_rmd & 0x1FFF) |
		((scaler_info->scaler_cip_int & 0xF) << 16);
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);
	addr = ISP_GET_REG(idx, ISP_SCALER_TRIM1_START) + base;
	cmd = (scaler_info->trim1_start_x & 0x1FFF) |
		((scaler_info->trim1_start_y & 0x1FFF) << 16);
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);
	addr = ISP_GET_REG(idx, ISP_SCALER_VER_IP) + base;
	cmd = (scaler_info->scaler_ip_rmd_ver & 0x1FFF) |
		((scaler_info->scaler_ip_int_ver & 0xF) << 16);
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);
	addr = ISP_GET_REG(idx, ISP_SCALER_VER_CIP) + base;
	cmd = (scaler_info->scaler_cip_rmd_ver & 0x1FFF) |
		((scaler_info->scaler_cip_int_ver & 0xF) << 16);
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);
	addr = ISP_GET_REG(idx, ISP_SCALER_TRIM1_SIZE) + base;
	cmd = (scaler_info->trim1_size_x & 0x1FFF) |
		((scaler_info->trim1_size_y & 0x1FFF) << 16);
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);

	return num;
}

static int set_fmcu_store(unsigned int *fmcu_buf,
	struct slice_store_info *store_info,
	unsigned int num, enum isp_id idx, unsigned int base)
{
	unsigned int addr = 0, cmd = 0;

	addr = ISP_GET_REG(idx, ISP_STORE_SLICE_SIZE) + base;
	cmd = ((store_info->size.height & 0xFFFF) << 16) |
		(store_info->size.width & 0xFFFF);
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);
	addr = ISP_GET_REG(idx, ISP_STORE_BORDER1) + base;
	cmd = (store_info->border.up_border & 0xFF) |
		((store_info->border.down_border & 0xFF) << 8)
		| ((store_info->border.left_border & 0xFF) << 16) |
		((store_info->border.right_border & 0xFF) << 24);
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);
	addr = ISP_GET_REG(idx, ISP_STORE_Y_ADDR) + base;
	cmd = store_info->addr.chn0;
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);
	addr = ISP_GET_REG(idx, ISP_STORE_U_ADDR) + base;
	cmd = store_info->addr.chn1;
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);
	addr = ISP_GET_REG(idx, ISP_STORE_V_ADDR) + base;
	cmd = store_info->addr.chn2;
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);
	addr = ISP_GET_REG(idx, ISP_STORE_SHADOW_CLR) + base;
	cmd = 1;
	num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);

	return num;
}

static int set_slice_fmcu_info(struct slice_param_in *in_ptr,
	struct slice_context_info *cxt, unsigned int *fmcu_num)
{
	int rtn = 0;
	struct slice_fetchYUV_info *fetchYUV_info = NULL;
	struct slice_dispatchYUV_info *dispatchYUV_info = NULL;
	struct slice_postcnr_info *postcnr_info = NULL;
	struct slice_ynr_info *ynr_info = NULL;
	struct slice_noisefilter_info *noisefilter_info = NULL;
	struct slice_base_info *base_info = NULL;
	struct slice_scaler_info *scaler_info = NULL;
	struct slice_store_info *store_info = NULL;
	unsigned int shadow_done_cmd[] = {0x10, 0x13};
	unsigned int all_done_cmd[] = {0x11, 0x14};
	unsigned int num = 0, addr = 0, cmd = 0;
	unsigned int slice_id = 0;
	unsigned int *fmcu_buf;
	enum isp_id idx = 0;
	unsigned int scl_base, store_base;

	if (!in_ptr || !cxt) {
		pr_err("input ptr is NULL!\n");
		rtn = -1;
		goto exit;
	}

	base_info = &cxt->base_info;
	fmcu_buf = in_ptr->fmcu_addr_vir;
	idx = in_ptr->idx;

	pr_debug("%s in, is_raw_capture：%d, fmcu_addr_vir:%p\n", __func__,
			in_ptr->is_raw_capture, in_ptr->fmcu_addr_vir);

	for (slice_id = 0; slice_id < base_info->slice_num; slice_id++) {
		fetchYUV_info = &cxt->fetchYUV_info[slice_id];
		dispatchYUV_info = &cxt->dispatchYUV_info[slice_id];
		postcnr_info = &cxt->postcnr_info[slice_id];
		ynr_info = &cxt->ynr_info[slice_id];
		noisefilter_info = &cxt->noisefilter_info[slice_id];
		if (slice_id == 0 && !in_ptr->is_raw_capture) {
			addr = ISP_GET_REG(idx, ISP_STORE_CCE_PARAM);
			cmd = ISP_REG_RD(idx, ISP_STORE_CCE_PARAM) | BIT_0;
			num = fmcu_push_back(&fmcu_buf[num], addr,
				cmd, num, ISP_CH_0);
			addr = ISP_GET_REG(idx, ISP_FMCU_CMD);
			cmd = shadow_done_cmd[0];
			num = fmcu_push_back(&fmcu_buf[num], addr,
				cmd, num, ISP_CH_1);
			addr = ISP_GET_REG(idx, ISP_FETCH2_SLICE_Y_PITCH);
			cmd = base_info->pitch.chn0;
			num = fmcu_push_back(&fmcu_buf[num], addr,
				cmd, num, ISP_CH_1);
			addr = ISP_GET_REG(idx, ISP_FETCH2_SLICE_U_PITCH);
			cmd = base_info->pitch.chn1;
			num = fmcu_push_back(&fmcu_buf[num], addr,
				cmd, num, ISP_CH_1);
			addr = ISP_GET_REG(idx, ISP_FETCH2_SLICE_V_PITCH);
			cmd = base_info->pitch.chn2;
			num = fmcu_push_back(&fmcu_buf[num], addr,
				cmd, num, ISP_CH_1);
		}
		num = set_fmcu_fetchyuv(fmcu_buf, fetchYUV_info, num, idx);
		num = set_fmcu_dispatchyuv(fmcu_buf,
			dispatchYUV_info, num, idx);
		num = set_fmcu_postcnr(fmcu_buf, postcnr_info, num, idx);
		num = set_fmcu_ynr(fmcu_buf, ynr_info, num, idx);
		num = set_fmcu_noisefilter(fmcu_buf,
			noisefilter_info, num, idx);

		if (base_info->isp_jpg_cowork) {
			if ((slice_id%base_info->slice_col_num) ==
				base_info->slice_col_num - 1) {
				addr = ISP_GET_REG(idx, ISP_COMMON_JPG_CTRL);
				cmd = 0x12;
				num = fmcu_push_back(&fmcu_buf[num], addr,
					cmd, num, ISP_CH_1);
			} else {
				addr = ISP_GET_REG(idx, ISP_COMMON_JPG_CTRL);
				cmd = 0x02;
				num = fmcu_push_back(&fmcu_buf[num], addr,
					cmd, num, ISP_CH_1);
			}
			if (slice_id == base_info->slice_num - 1) {
				addr = ISP_GET_REG(idx, ISP_COMMON_JPG_CTRL);
				cmd = 0x03;
				num = fmcu_push_back(&fmcu_buf[num], addr,
					cmd, num, ISP_CH_1);
			}
		}

		if (in_ptr->pre_slice_need == 1) {
			scaler_info = cxt->scaler_info[SLICE_PATH_PRE];
			store_info = cxt->store_info[SLICE_PATH_PRE];
			scl_base = ISP_SCALER_PRE_BASE;
			store_base = ISP_STORE1_BASE;
			num = set_fmcu_scaler(fmcu_buf,
				&scaler_info[slice_id], num, idx, scl_base);
			if (!slice_id) {
				addr = ISP_GET_REG(idx,
					ISP_STORE_PREVIEW_SHADOW_CLR_SEL);
				cmd = 2;
				num = fmcu_push_back(&fmcu_buf[num], addr,
					cmd, num, ISP_CH_1);
			}
			num = set_fmcu_store(fmcu_buf,
				&store_info[slice_id], num, idx, store_base);
		}

		if (in_ptr->vid_slice_need == 1) {
			scaler_info = cxt->scaler_info[SLICE_PATH_VID];
			store_info = cxt->store_info[SLICE_PATH_VID];
			scl_base = ISP_SCALER_VID_BASE;
			store_base = ISP_STORE2_BASE;
			num = set_fmcu_scaler(fmcu_buf,
				&scaler_info[slice_id], num, idx, scl_base);
			if (!slice_id) {
				addr = ISP_GET_REG(idx,
					ISP_STORE_VIDEO_SHADOW_CLR_SEL);
				cmd = 2;
				num = fmcu_push_back(&fmcu_buf[num], addr,
					cmd, num, ISP_CH_1);
			}
			num = set_fmcu_store(fmcu_buf,
				&store_info[slice_id], num, idx, store_base);
		}

		if (in_ptr->cap_slice_need == 1) {
			scaler_info = cxt->scaler_info[SLICE_PATH_CAP];
			store_info = cxt->store_info[SLICE_PATH_CAP];
			scl_base = ISP_SCALER_CAP_BASE;
			store_base = ISP_STORE3_BASE;
			num = set_fmcu_scaler(fmcu_buf,
				&scaler_info[slice_id], num, idx, scl_base);
			if (!slice_id) {
				addr = ISP_GET_REG(idx,
					ISP_STORE_CAPTURE_SHADOW_CLR_SEL);
				cmd = 2;
				num = fmcu_push_back(&fmcu_buf[num], addr,
					cmd, num, ISP_CH_1);
			}
			num = set_fmcu_store(fmcu_buf,
				&store_info[slice_id], num, idx, store_base);
		}
		addr = ISP_GET_REG(idx, ISP_FETCH2_START);
		cmd = 1;
		num = fmcu_push_back(&fmcu_buf[num], addr, cmd, num, ISP_CH_1);
		addr = ISP_GET_REG(idx, ISP_FMCU_CMD);
		cmd = shadow_done_cmd[1];
		num = fmcu_push_back(&fmcu_buf[num], addr,
			cmd, num, ISP_CH_1);
		addr = ISP_GET_REG(idx, ISP_FMCU_CMD);
		cmd = all_done_cmd[1];
		num = fmcu_push_back(&fmcu_buf[num], addr,
			cmd, num, ISP_CH_1);
	}
	*fmcu_num = num / 2;
exit:
	return rtn;
}

int isp_fmcu_slice_cfg(enum isp_id idx, void *fmcu_handler,
	struct slice_param_in *in_ptr, unsigned int *fmcu_num)
{
	int rtn = 0;
	struct slice_context_info *cxt = NULL;

	if (!fmcu_handler || !in_ptr || !fmcu_num) {
		pr_err("input handle is NULL!\n");
		rtn = -1;
		goto exit;
	}
	cxt = (struct slice_context_info *)fmcu_handler;

	rtn = set_slice_base_info(idx, in_ptr, &cxt->base_info);
	if (rtn) {
		pr_err("fail to set slice base info!\n");
		goto exit;
	}

	rtn = set_slice_fetchyuv_info(in_ptr, cxt);
	if (rtn) {
		pr_err("fail to set slice fetchYUV info!\n");
		goto exit;
	}

	rtn = set_slice_dispatchyuv_info(cxt);
	if (rtn) {
		pr_err("fail to set slice dispatchYUV info!\n");
		goto exit;
	}

	rtn = set_slice_postcnr_info(cxt);
	if (rtn) {
		pr_err("fail to set slice postcnr info!\n");
		goto exit;
	}

	rtn = set_slice_ynr_info(cxt);
	if (rtn) {
		pr_err("fail to set slice ynr info!\n");
		goto exit;
	}

	rtn = set_slice_scaler_info(in_ptr, cxt);
	if (rtn) {
		pr_err("fail to set slice scaler info!\n");
		goto exit;
	}

	rtn = set_slice_noisefliter_info(cxt);
	if (rtn) {
		pr_err("fail to set slice noise fliter info!\n");
		goto exit;
	}

	rtn = set_slice_store_info(in_ptr, cxt);
	if (rtn) {
		pr_err("fail to set slice store info!\n");
		goto exit;
	}

	rtn = set_slice_fmcu_info(in_ptr, cxt, fmcu_num);
	if (rtn) {
		pr_err("fail to set slice fmcu info!\n");
		goto exit;
	}
exit:
	return rtn;
}

int isp_fmcu_slice_init(void **fmcu_handler)
{
	int rtn = 0;
	struct slice_context_info *cxt = NULL;

	if (!fmcu_handler) {
		pr_err("fmcu_handle is NULL!\n");
		rtn = -1;
		goto exit;
	}
	*fmcu_handler = NULL;

	cxt = vzalloc(sizeof(struct slice_context_info));
	*fmcu_handler = (void *)cxt;
exit:
	return rtn;
}

int isp_fmcu_slice_deinit(void *fmcu_handler)
{
	int rtn = 0;
	struct slice_context_info *cxt = NULL;

	if (!fmcu_handler) {
		pr_err("fmcu_handle is NULL!\n");
		return -1;
	}
	cxt = (struct slice_context_info *)fmcu_handler;

	if (cxt != NULL)
		vfree(cxt);

	return rtn;
}

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

#include <linux/kernel.h>
#include <linux/vmalloc.h>
#include <video/sprd_mm.h>

#include "isp_reg.h"
#include "isp_core.h"
#include "isp_slice.h"
#include "isp_fmcu.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_SLICE: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

#define ISP_SLICE_ALIGN_SIZE           2
#define ISP_ALIGNED(size)              ((size) & ~(ISP_SLICE_ALIGN_SIZE - 1))

extern uint32_t line_buffer_len;

struct isp_scaler_slice_tmp {
	uint32_t slice_row_num;
	uint32_t slice_col_num;
	uint32_t start_col;
	uint32_t start_row;
	uint32_t end_col;
	uint32_t end_row;
	uint32_t start_col_orig;
	uint32_t start_row_orig;
	uint32_t end_col_orig;
	uint32_t end_row_orig;
	uint32_t x;
	uint32_t y;
	uint32_t overlap_bad_up;
	uint32_t overlap_bad_down;
	uint32_t overlap_bad_left;
	uint32_t overlap_bad_right;
	uint32_t trim0_end_x;
	uint32_t trim0_end_y;
	uint32_t trim0_start_adjust_x;
	uint32_t trim0_start_adjust_y;
	uint32_t trim1_sum_x;
	uint32_t trim1_sum_y;
	uint32_t deci_x;
	uint32_t deci_y;
	uint32_t deci_x_align;
	uint32_t deci_y_align;
	uint32_t scaler_out_height_temp;
	uint32_t scaler_out_width_temp;
};

static int get_slice_size_info(
			struct slice_cfg_input *in_ptr,
			uint32_t *w, uint32_t *h)
{
	int rtn = 0;
	uint32_t j;
	uint32_t w1, w2, slice_max_w;
	struct img_size *input = &in_ptr->frame_in_size;
	struct img_size *output;

	w1 = w2 = input->w;
	slice_max_w = line_buffer_len - SLICE_OVERLAP_W_MAX;

	if (input->w > line_buffer_len)
		w1 = slice_max_w;
	w1 = (input->w + 47) / 3;

	for (j = 0; j < ISP_SPATH_NUM; j++) {
		output = in_ptr->frame_out_size[j];
		if (output && output->w > line_buffer_len) {
			slice_max_w = (input->w *
				slice_max_w + output->w - 1) / output->w;
			w2 = (slice_max_w < w2) ? slice_max_w :  w2;
		}
	}

	*w = (w1 < w2) ? w1 : w2;
	*h = input->h / SLICE_H_NUM_MAX;

	*w = ISP_ALIGNED(*w);
	*h = ISP_ALIGNED(*h);

	return rtn;
}

static int get_slice_overlap_info(
			struct slice_cfg_input *in_ptr,
			struct isp_slice_context *slc_ctx)
{
	switch (in_ptr->frame_fetch->fetch_fmt) {
	case ISP_FETCH_RAW10:
	case ISP_FETCH_CSI2_RAW10:
		slc_ctx->overlap_up = RAW_OVERLAP_UP;
		slc_ctx->overlap_down = RAW_OVERLAP_DOWN;
		slc_ctx->overlap_left = RAW_OVERLAP_LEFT;
		slc_ctx->overlap_right = RAW_OVERLAP_RIGHT;
		break;
	default:
		slc_ctx->overlap_up = YUV_OVERLAP_UP;
		slc_ctx->overlap_down = YUV_OVERLAP_DOWN;
		slc_ctx->overlap_left = YUV_OVERLAP_LEFT;
		slc_ctx->overlap_right = YUV_OVERLAP_RIGHT;
		break;
	}

	return 0;
}

static int cfg_slice_base_info(
			struct slice_cfg_input *in_ptr,
			struct isp_slice_context *slc_ctx)
{
	int rtn = 0;
	uint32_t i = 0, j = 0;
	uint32_t img_height, img_width;
	uint32_t slice_height = 0, slice_width = 0;
	uint32_t slice_total_row, slice_total_col, slice_num;

	struct isp_slice_desc *cur_slc;

	rtn = get_slice_size_info(in_ptr, &slice_width, &slice_height);

	rtn = get_slice_overlap_info(in_ptr, slc_ctx);

	img_height = in_ptr->frame_in_size.h;
	img_width = in_ptr->frame_in_size.w;
	slice_total_row = (img_height + slice_height - 1) / slice_height;
	slice_total_col = (img_width + slice_width - 1) / slice_width;
	slice_num = slice_total_col * slice_total_row;
	slc_ctx->slice_num = slice_num;
	slc_ctx->slice_col_num = slice_total_col;
	slc_ctx->slice_row_num = slice_total_row;
	slc_ctx->slice_height = slice_height;
	slc_ctx->slice_width = slice_width;
	slc_ctx->img_height = img_height;
	slc_ctx->img_width = img_width;

	pr_info("img w %d, h %d, slice w %d, h %d, slice num %d\n",
			img_width, img_height,
			slice_width, slice_height, slice_num);

	for (i = 0; i < SLICE_NUM_MAX; i++)
		pr_debug("slice %d valid %d\n", i, slc_ctx->slices[i].valid);

	cur_slc = &slc_ctx->slices[0];
	for (i = 0; i < slice_total_row; i++) {
		for (j = 0; j < slice_total_col; j++) {
			uint32_t start_col;
			uint32_t start_row;
			uint32_t end_col;
			uint32_t end_row;

			cur_slc->valid = 1;
			cur_slc->y = i;
			cur_slc->x = j;
			start_col = j * slice_width;
			start_row = i * slice_height;
			end_col = start_col + slice_width - 1;
			end_row = start_row + slice_height - 1;
			if (i != 0)
				cur_slc->slice_overlap.overlap_up =
							slc_ctx->overlap_up;
			if (j != 0)
				cur_slc->slice_overlap.overlap_left =
							slc_ctx->overlap_left;
			if (i != (slice_total_row - 1))
				cur_slc->slice_overlap.overlap_down =
							slc_ctx->overlap_down;
			else
				end_row = img_height - 1;

			if (j != (slice_total_col - 1))
				cur_slc->slice_overlap.overlap_right =
							slc_ctx->overlap_right;
			else
				end_col = img_width - 1;
			cur_slc->slice_pos_orig.start_col = start_col;
			cur_slc->slice_pos_orig.start_row = start_row;
			cur_slc->slice_pos_orig.end_col = end_col;
			cur_slc->slice_pos_orig.end_row = end_row;

			cur_slc->slice_pos.start_col =
				start_col - cur_slc->slice_overlap.overlap_left;
			cur_slc->slice_pos.start_row =
				start_row - cur_slc->slice_overlap.overlap_up;
			cur_slc->slice_pos.end_col =
				end_col + cur_slc->slice_overlap.overlap_right;
			cur_slc->slice_pos.end_row =
				end_row + cur_slc->slice_overlap.overlap_down;

			pr_debug("slice %d %d pos_orig [%d %d %d %d]\n", i, j,
					cur_slc->slice_pos_orig.start_col,
					cur_slc->slice_pos_orig.end_col,
					cur_slc->slice_pos_orig.start_row,
					cur_slc->slice_pos_orig.end_row);
			pr_debug("slice %d %d pos [%d %d %d %d]\n", i, j,
					cur_slc->slice_pos.start_col,
					cur_slc->slice_pos.end_col,
					cur_slc->slice_pos.start_row,
					cur_slc->slice_pos.end_row);
			pr_debug("slice %d %d ovl [%d %d %d %d]\n", i, j,
					cur_slc->slice_overlap.overlap_up,
					cur_slc->slice_overlap.overlap_down,
					cur_slc->slice_overlap.overlap_left,
					cur_slc->slice_overlap.overlap_right);

			cur_slc++;
		}
	}

	for (i = 0; i < SLICE_NUM_MAX; i++) {
		pr_debug("slice %d valid %d. xy (%d %d)  %p\n",
			i, slc_ctx->slices[i].valid,
			slc_ctx->slices[i].x, slc_ctx->slices[i].y,
			&slc_ctx->slices[i]);
	}

	return rtn;
}

static int cfg_slice_cfa_info(
		struct slice_cfg_input *in_ptr,
		struct isp_slice_context *slc_ctx)
{
	int i;
	uint32_t start_col;
	uint32_t end_col;
	struct isp_slice_desc *cur_slc;

	cur_slc = &slc_ctx->slices[0];
	for (i = 0; i < SLICE_NUM_MAX; i++, cur_slc++) {
		if (cur_slc->valid == 0)
			continue;
		end_col = cur_slc->slice_pos.end_col;
		start_col = cur_slc->slice_pos.start_col;
		cur_slc->slice_cfa.gbuf_addr_max =
			((end_col - start_col + 1) / 2) + 1;
	}

	return 0;
}

static void cfg_spath_trim0_info(
		struct isp_scaler_slice_tmp *sinfo,
		struct img_trim *frm_trim0,
		struct slice_scaler_info *slc_scaler)
{
	uint32_t start, end;

	/* trim0 x */
	start = sinfo->start_col + sinfo->overlap_bad_left;
	end = sinfo->end_col + 1 - sinfo->overlap_bad_right;

	pr_debug("start end %d, %d\n", start, end);
	pr_debug("frame %d %d %d %d\n",
		frm_trim0->start_x,
		frm_trim0->start_y,
		frm_trim0->size_x,
		frm_trim0->size_y);

	if (sinfo->slice_col_num == 1) {
		slc_scaler->trim0_start_x = frm_trim0->start_x;
		slc_scaler->trim0_size_x = frm_trim0->size_x;
		pr_debug("%d, %d\n",
			slc_scaler->trim0_start_x,
			slc_scaler->trim0_size_x);
	} else {
		if ((sinfo->end_col_orig < frm_trim0->start_x) ||
			(sinfo->start_col_orig >
			(frm_trim0->start_x + frm_trim0->size_x - 1))) {
			slc_scaler->out_of_range = 1;
			return;
		}

		if (sinfo->x == 0) {
			/* first slice */
			slc_scaler->trim0_start_x = frm_trim0->start_x;
			if (sinfo->trim0_end_x < end)
				slc_scaler->trim0_size_x = frm_trim0->size_x;
			else
				slc_scaler->trim0_size_x = end - frm_trim0->start_x;
		} else if ((sinfo->slice_col_num - 1) == sinfo->x) {
			/* last slice */
			if (frm_trim0->start_x > start) {
				slc_scaler->trim0_start_x =
					frm_trim0->start_x - sinfo->start_col;
				slc_scaler->trim0_size_x = frm_trim0->size_x;
			} else {
				slc_scaler->trim0_start_x = sinfo->overlap_bad_left;
				slc_scaler->trim0_size_x = sinfo->trim0_end_x-start;
			}
		} else {
			if (frm_trim0->start_x < start) {
				slc_scaler->trim0_start_x = sinfo->overlap_bad_left;
				if (sinfo->trim0_end_x < end)
					slc_scaler->trim0_size_x = sinfo->trim0_end_x - start;
				else
					slc_scaler->trim0_size_x = end - start;
			} else {
				slc_scaler->trim0_start_x =
					frm_trim0->start_x - sinfo->start_col;
				if (sinfo->trim0_end_x < end)
					slc_scaler->trim0_size_x = frm_trim0->size_x;
				else
					slc_scaler->trim0_size_x = end - frm_trim0->start_x;
			}
		}
	}

	/* trim0 y */
	start = sinfo->start_row + sinfo->overlap_bad_up;
	end = sinfo->end_row + 1 - sinfo->overlap_bad_down;
	if (sinfo->slice_row_num == 1) {
		slc_scaler->trim0_start_y = frm_trim0->start_y;
		slc_scaler->trim0_size_y = frm_trim0->size_y;
	} else {
		pr_err("error: not support vertical slices.\n");
	}
}


static void cfg_spath_deci_info(
		struct isp_scaler_slice_tmp *sinfo,
		struct img_deci_info *frm_deci,
		struct img_trim *frm_trim0,
		struct slice_scaler_info *slc_scaler)
{
	uint32_t start;

	sinfo->deci_x = frm_deci->deci_x;
	sinfo->deci_y = frm_deci->deci_y;
	sinfo->deci_x_align = sinfo->deci_x * 2;

	start = sinfo->start_col + sinfo->overlap_bad_left;

	if (frm_trim0->start_x >= sinfo->start_col &&
		(frm_trim0->start_x <= (sinfo->end_col+1))) {
		slc_scaler->trim0_size_x = slc_scaler->trim0_size_x /
			sinfo->deci_x_align * sinfo->deci_x_align;
	} else {
		sinfo->trim0_start_adjust_x = (start+sinfo->deci_x_align-1)/
			sinfo->deci_x_align * sinfo->deci_x_align-start;
		slc_scaler->trim0_start_x += sinfo->trim0_start_adjust_x;
		slc_scaler->trim0_size_x -= sinfo->trim0_start_adjust_x;
		slc_scaler->trim0_size_x = slc_scaler->trim0_size_x/
			sinfo->deci_x_align * sinfo->deci_x_align;
	}

	if (slc_scaler->odata_mode == ODATA_YUV422)
		sinfo->deci_y_align = sinfo->deci_y;
	else
		sinfo->deci_y_align = sinfo->deci_y * 2;

	start = sinfo->start_row + sinfo->overlap_bad_up;

	if (frm_trim0->start_y >= sinfo->start_row &&
		(frm_trim0->start_y  <= (sinfo->end_row + 1))) {
		slc_scaler->trim0_size_y = slc_scaler->trim0_size_y/
			sinfo->deci_y_align * sinfo->deci_y_align;
	} else {
		sinfo->trim0_start_adjust_y = (start + sinfo->deci_y_align-1)/
			sinfo->deci_y_align * sinfo->deci_y_align - start;
		slc_scaler->trim0_start_y += sinfo->trim0_start_adjust_y;
		slc_scaler->trim0_size_y -= sinfo->trim0_start_adjust_y;
		slc_scaler->trim0_size_y = slc_scaler->trim0_size_y/
			sinfo->deci_y_align * sinfo->deci_y_align;
	}

	slc_scaler->scaler_in_width =
		slc_scaler->trim0_size_x / sinfo->deci_x;
	slc_scaler->scaler_in_height =
		slc_scaler->trim0_size_y / sinfo->deci_y;

}


static void calc_scaler_phase(uint32_t phase, uint32_t factor,
	uint32_t *phase_int, uint32_t *phase_rmd)
{
	phase_int[0] = (uint32_t)(phase/factor);
	phase_rmd[0] = (uint32_t)(phase-factor*phase_int[0]);
}

static void cfg_spath_scaler_info(
		struct isp_scaler_slice_tmp *slice,
		struct img_trim *frm_trim0,
		struct isp_scaler_info *in,
		struct slice_scaler_info *out)
{
	uint32_t scl_factor_in, scl_factor_out;
	uint32_t  initial_phase, last_phase, phase_in;
	uint32_t phase_tmp, scl_temp, out_tmp;
	uint32_t start, end;
	uint32_t tap_hor, tap_ver, tap_hor_uv, tap_ver_uv;
	uint32_t tmp, n;

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
		if (frm_trim0->start_x >= slice->start_col &&
			(frm_trim0->start_x <= slice->end_col+1)) {
			phase_in = 0;
			if (out->scaler_in_width ==
				frm_trim0->size_x / slice->deci_x)
				phase_tmp = last_phase;
			else
				phase_tmp = (out->scaler_in_width/2-
				tap_hor_uv/2)*scl_factor_out -
				scl_factor_in/2 - 1;
			out_tmp = (phase_tmp - phase_in)/scl_factor_in+1;
			out->scaler_out_width = out_tmp*2;
		} else {
			phase_in = (tap_hor_uv/2)*scl_factor_out;
			if (slice->x == slice->slice_col_num - 1) {
				phase_tmp = last_phase-
					((frm_trim0->size_x/2)/slice->deci_x-
					out->scaler_in_width/2)*scl_factor_out;
				out_tmp = (phase_tmp-phase_in)/scl_factor_in+1;
				out->scaler_out_width = out_tmp*2;
				phase_in = phase_tmp-(out_tmp-1)*scl_factor_in;
			} else {
				if (slice->trim0_end_x >= slice->start_col
					&& (slice->trim0_end_x <= slice->end_col
					+1-slice->overlap_bad_right)) {
					phase_tmp = last_phase-
					((frm_trim0->size_x/2)/slice->deci_x-
					out->scaler_in_width/2)*scl_factor_out;
					out_tmp = (phase_tmp-phase_in)/
						scl_factor_in+1;
					out->scaler_out_width = out_tmp*2;
					phase_in = phase_tmp-(out_tmp-1)
						*scl_factor_in;
				} else {
					initial_phase = ((((start/
					slice->deci_x_align*slice->deci_x_align
					-frm_trim0->start_x)/slice->deci_x)/2+
					(tap_hor_uv/2))*(scl_factor_out)+
					(scl_factor_in-1))/scl_factor_in*
					scl_factor_in;
					slice->scaler_out_width_temp =
					((last_phase-initial_phase)/
					scl_factor_in+1)*2;

					scl_temp = ((end/slice->deci_x_align*
					slice->deci_x_align - frm_trim0->start_x)/
					slice->deci_x)/2;
					last_phase = ((scl_temp-tap_hor_uv/2)*
					(scl_factor_out)-scl_factor_in/2-1)/
					scl_factor_in*scl_factor_in;

					out_tmp = (last_phase-initial_phase)/
						scl_factor_in+1;
					out->scaler_out_width = out_tmp*2;
					phase_in = initial_phase-(((start/
					slice->deci_x_align*slice->deci_x_align-
					frm_trim0->start_x)/slice->deci_x)/2)*
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

		if (frm_trim0->start_y >= slice->start_row &&
			(frm_trim0->start_y <= slice->end_row+1)) {
			phase_in = 0;
			if (out->scaler_in_height ==
				frm_trim0->size_y/slice->deci_y)
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
			if (slice->y == slice->slice_row_num-1) {
				phase_tmp = last_phase-
					(frm_trim0->size_y/slice->deci_y-
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
					(frm_trim0->size_y/slice->deci_y-
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
					-frm_trim0->start_y)/slice->deci_y+
					(tap_ver_uv/2))*(scl_factor_out)+
					(scl_factor_in-1))/(scl_factor_in*2)
					*(scl_factor_in*2);
					slice->scaler_out_height_temp =
						(last_phase-initial_phase)/
						scl_factor_in+1;
					scl_temp = (end/slice->deci_y_align*
					slice->deci_y_align-frm_trim0->start_y)/
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
					frm_trim0->start_y)/slice->deci_y
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
		slice->scaler_out_width_temp = (frm_trim0->size_x-(start/
			slice->deci_x_align*slice->deci_x_align-
			frm_trim0->start_x))/slice->deci_x;
		start = slice->start_row+slice->overlap_bad_up+
			slice->trim0_start_adjust_y+slice->deci_y_align-1;
		slice->scaler_out_height_temp = (frm_trim0->size_y-(start/
			slice->deci_y_align*slice->deci_y_align-
			frm_trim0->start_y))/slice->deci_y;
	}
}

void cfg_spath_trim1_info(
		struct isp_scaler_slice_tmp *slice,
		struct img_trim *frm_trim0,
		struct isp_scaler_info *in,
		struct slice_scaler_info *out)
{
	uint32_t trim_sum_x = slice->trim1_sum_x;
	uint32_t trim_sum_y = slice->trim1_sum_y;
	uint32_t pix_align = 8;

	if (frm_trim0->start_x >= slice->start_col &&
		(frm_trim0->start_x <= slice->end_col + 1)) {
		out->trim1_start_x = 0;
		if (out->scaler_in_width == frm_trim0->size_x)
			out->trim1_size_x = out->scaler_out_width;
		else
			out->trim1_size_x = out->scaler_out_width & ~(pix_align - 1);
	} else {
		if (slice->x == slice->slice_col_num - 1) {
			out->trim1_size_x =
				in->scaler_out_width - trim_sum_x;
			out->trim1_start_x =
				out->scaler_out_width - out->trim1_size_x;
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
					out->trim1_start_x) & ~(pix_align-1);
			}
		}
	}

	if (frm_trim0->start_y >= slice->start_row &&
		(frm_trim0->start_y <= slice->end_row+1)) {
		out->trim1_start_y = 0;
		if (out->scaler_in_height == frm_trim0->size_y)
			out->trim1_size_y = out->scaler_out_height;
		else
			out->trim1_size_y = out->scaler_out_height & ~(pix_align-1);
	} else {
		if (slice->y == slice->slice_row_num - 1) {
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
					out->trim1_start_y) & ~(pix_align-1);
			}
		}
	}
}


static int cfg_slice_scaler_info(
		struct slice_cfg_input *in_ptr,
		struct isp_slice_context *slc_ctx)
{
	int i, j;
	uint32_t trim1_sum_x[ISP_SPATH_NUM][SLICE_W_NUM_MAX] = { { 0 }, { 0 } };
	uint32_t trim1_sum_y[ISP_SPATH_NUM][SLICE_H_NUM_MAX] = { { 0 }, { 0 } };
	struct isp_scaler_info  *frm_scaler;
	struct img_deci_info *frm_deci;
	struct img_trim *frm_trim0;
	struct img_trim *frm_trim1;

	struct slice_scaler_info *slc_scaler;
	struct isp_slice_desc *cur_slc;
	struct isp_scaler_slice_tmp sinfo = {0};

	sinfo.slice_col_num = slc_ctx->slice_col_num;
	sinfo.slice_row_num = slc_ctx->slice_row_num;
	sinfo.trim1_sum_x = 0;
	sinfo.trim1_sum_y = 0;
	sinfo.overlap_bad_up =
		slc_ctx->overlap_up - YUVSCALER_OVERLAP_UP;
	sinfo.overlap_bad_down =
		slc_ctx->overlap_down - YUVSCALER_OVERLAP_DOWN;
	sinfo.overlap_bad_left =
		slc_ctx->overlap_left - YUVSCALER_OVERLAP_LEFT;
	sinfo.overlap_bad_right =
		slc_ctx->overlap_right - YUVSCALER_OVERLAP_RIGHT;

	for (i = 0; i < SLICE_NUM_MAX; i++) {
		pr_debug("slice %d valid %d. xy (%d %d)  %p\n",
			i, slc_ctx->slices[i].valid,
			slc_ctx->slices[i].x, slc_ctx->slices[i].y,
			&slc_ctx->slices[i]);
	}

	cur_slc = &slc_ctx->slices[0];
	for (i = 0; i < SLICE_NUM_MAX; i++, cur_slc++) {
		if (cur_slc->valid == 0) {
			pr_debug("slice %d not valid. %p\n", i, cur_slc);
			continue;
		}

		for (j = 0; j < ISP_SPATH_NUM; j++) {

			frm_scaler = in_ptr->frame_scaler[j];
			if (frm_scaler == NULL) {
				/* path is not valid. */
				cur_slc->path_en[j] = 0;
				pr_debug("path %d not enable.\n", j);
				continue;
			}
			cur_slc->path_en[j] = 1;
			pr_debug("path %d  enable.\n", j);

			frm_deci = in_ptr->frame_deci[j];
			frm_trim0 = in_ptr->frame_trim0[j];
			frm_trim1 = in_ptr->frame_trim1[j];

			slc_scaler = &cur_slc->slice_scaler[j];
			slc_scaler->scaler_bypass = frm_scaler->scaler_bypass;
			slc_scaler->odata_mode = frm_scaler->odata_mode;

			sinfo.x = cur_slc->x;
			sinfo.y = cur_slc->y;

			sinfo.start_col = cur_slc->slice_pos.start_col;
			sinfo.end_col = cur_slc->slice_pos.end_col;
			sinfo.start_row = cur_slc->slice_pos.start_row;
			sinfo.end_row = cur_slc->slice_pos.end_row;

			sinfo.start_col_orig = cur_slc->slice_pos_orig.start_col;
			sinfo.end_col_orig = cur_slc->slice_pos_orig.end_col;
			sinfo.start_row_orig = cur_slc->slice_pos_orig.start_row;
			sinfo.end_row_orig = cur_slc->slice_pos_orig.end_row;

			sinfo.trim0_end_x = frm_trim0->start_x + frm_trim0->size_x;
			sinfo.trim0_end_y = frm_trim0->start_y + frm_trim0->size_y;

			cfg_spath_trim0_info(&sinfo, frm_trim0, slc_scaler);
			if (slc_scaler->out_of_range) {
				cur_slc->path_en[j] = 0;
				continue;
			}

			cfg_spath_deci_info(&sinfo, frm_deci, frm_trim0, slc_scaler);
			cfg_spath_scaler_info(&sinfo, frm_trim0, frm_scaler, slc_scaler);

			sinfo.trim1_sum_x = trim1_sum_x[j][cur_slc->x];
			sinfo.trim1_sum_y = trim1_sum_y[j][cur_slc->y];
			cfg_spath_trim1_info(&sinfo, frm_trim0, frm_scaler, slc_scaler);

			if (cur_slc->y == 0 && (cur_slc->x+1) < SLICE_W_NUM_MAX)
				trim1_sum_x[j][cur_slc->x + 1] =
					slc_scaler->trim1_size_x + trim1_sum_x[j][cur_slc->x];

			if (cur_slc->x == 0 && (cur_slc->y+1) < SLICE_H_NUM_MAX)
				trim1_sum_y[j][cur_slc->y + 1] =
					slc_scaler->trim1_size_y + trim1_sum_y[j][cur_slc->y];

			slc_scaler->src_size_x = sinfo.end_col - sinfo.start_col + 1;
			slc_scaler->src_size_y = sinfo.end_row - sinfo.start_row + 1;
			slc_scaler->dst_size_x = slc_scaler->scaler_out_width;
			slc_scaler->dst_size_y = slc_scaler->scaler_out_height;
		}

		/* check if all path scaler out of range. */
		/* if yes, invalid this slice. */
		cur_slc->valid = 0;
		for (j = 0; j < ISP_SPATH_NUM; j++)
			cur_slc->valid |= cur_slc->path_en[j];
		pr_debug("final slice (%d, %d) valid = %d\n",
			cur_slc->x, cur_slc->y, cur_slc->valid);
	}

	return 0;
}



int isp_cfg_slice_fetch_info(
			void *cfg_in, struct isp_slice_context *slc_ctx)
{
	int i;
	uint32_t start_col, start_row;
	uint32_t end_col, end_row;
	uint32_t ch_offset[3] = { 0 };
	struct slice_cfg_input *in_ptr = (struct slice_cfg_input *)cfg_in;
	struct isp_fetch_info *frm_fetch;
	struct slice_fetch_info *slc_fetch;
	struct isp_slice_desc *cur_slc;
	struct img_pitch *pitch;
	uint32_t mipi_word_num_start[16] = {
		0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5};
	uint32_t mipi_word_num_end[16] = {
		0, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5};

	frm_fetch = in_ptr->frame_fetch;
	cur_slc = &slc_ctx->slices[0];
	for (i = 0; i < SLICE_NUM_MAX; i++, cur_slc++) {
		if (cur_slc->valid == 0)
			continue;
		slc_fetch = &cur_slc->slice_fetch;

		start_col = cur_slc->slice_pos.start_col;
		start_row = cur_slc->slice_pos.start_row;
		end_col = cur_slc->slice_pos.end_col;
		end_row = cur_slc->slice_pos.end_row;

		pitch = &frm_fetch->pitch;

		switch (frm_fetch->fetch_fmt) {
		case ISP_FETCH_YUV422_3FRAME:
			ch_offset[0] = start_row * pitch->pitch_ch0 + start_col;
			ch_offset[1] = start_row * pitch->pitch_ch1 + (start_col >> 1);
			ch_offset[2] = start_row * pitch->pitch_ch2 + (start_col >> 1);
			break;

		case ISP_FETCH_YUV422_2FRAME:
		case ISP_FETCH_YVU422_2FRAME:
			ch_offset[0] = start_row * pitch->pitch_ch0 + start_col;
			ch_offset[1] = start_row * pitch->pitch_ch1 + start_col;
			break;

		case ISP_FETCH_YUV420_2FRAME:
		case ISP_FETCH_YVU420_2FRAME:
			ch_offset[0] = start_row * pitch->pitch_ch0 + start_col;
			ch_offset[1] = (start_row>>1) * pitch->pitch_ch1 + start_col;
			break;

		case ISP_FETCH_CSI2_RAW10:
			ch_offset[0] = start_row * pitch->pitch_ch0
							+ (start_col >> 2) * 5
							+ (start_col & 0x3);

			slc_fetch->mipi_byte_rel_pos = start_col & 0x0f;
			slc_fetch->mipi_word_num = (
					(((end_col + 1) >> 4) * 5 +
					mipi_word_num_end[(end_col + 1) & 0x0f])
					-(((start_col + 1) >> 4) * 5 +
					mipi_word_num_start[(start_col + 1) & 0x0f])
					+ 1);
			break;

		default:
			ch_offset[0] = start_row * pitch->pitch_ch0 + start_col * 2;
			break;
		}

		slc_fetch->addr.addr_ch0 = frm_fetch->addr.addr_ch0 + ch_offset[0];
		slc_fetch->addr.addr_ch1 = frm_fetch->addr.addr_ch1 + ch_offset[1];
		slc_fetch->addr.addr_ch2 = frm_fetch->addr.addr_ch2 + ch_offset[2];
		slc_fetch->size.h = end_row - start_row + 1;
		slc_fetch->size.w = end_col - start_col + 1;
	}

	return 0;
}

int isp_cfg_slice_store_info(
		void *cfg_in, struct isp_slice_context *slc_ctx)
{
	int i, j;
	struct slice_cfg_input *in_ptr = (struct slice_cfg_input *)cfg_in;
	struct isp_store_info *frm_store;
	struct isp_slice_desc *cur_slc;
	struct slice_store_info *slc_store;
	struct slice_scaler_info *slc_scaler;

	uint32_t overlap_left, overlap_up;
	uint32_t overlap_right, overlap_down;

	uint32_t start_col, end_col, start_row, end_row;
	uint32_t start_row_out[ISP_SPATH_NUM][SLICE_W_NUM_MAX] = { { 0 }, { 0 } };
	uint32_t start_col_out[ISP_SPATH_NUM][SLICE_W_NUM_MAX] = { { 0 }, { 0 } };
	uint32_t ch0_offset = 0;
	uint32_t ch1_offset = 0;
	uint32_t ch2_offset = 0;

	cur_slc = &slc_ctx->slices[0];
	for (i = 0; i < SLICE_NUM_MAX; i++, cur_slc++) {
		if (cur_slc->valid == 0)
			continue;
		for (j = 0; j < ISP_SPATH_NUM; j++) {
			frm_store = in_ptr->frame_store[j];
			if (frm_store == NULL || cur_slc->path_en[j] == 0) {
				/* path is not valid. */
				continue;
			}
			slc_store = &cur_slc->slice_store[j];
			slc_scaler = &cur_slc->slice_scaler[j];

			if (slc_scaler->scaler_bypass == 0) {
				slc_store->size.w = slc_scaler->trim1_size_x;
				slc_store->size.h = slc_scaler->trim1_size_y;

				if (cur_slc->y == 0 && (cur_slc->x + 1) < SLICE_W_NUM_MAX)
					start_col_out[j][cur_slc->x + 1] =
						slc_store->size.w + start_col_out[j][cur_slc->x];

				if (cur_slc->x == 0 && (cur_slc->y + 1) < SLICE_H_NUM_MAX)
					start_row_out[j][cur_slc->y + 1] =
						slc_store->size.h + start_row_out[j][cur_slc->y];
			} else {
				start_col = cur_slc->slice_pos.start_col;
				start_row = cur_slc->slice_pos.start_row;
				end_col = cur_slc->slice_pos.end_col;
				end_row = cur_slc->slice_pos.end_row;
				overlap_left = cur_slc->slice_overlap.overlap_left;
				overlap_right = cur_slc->slice_overlap.overlap_right;
				overlap_up = cur_slc->slice_overlap.overlap_up;
				overlap_down = cur_slc->slice_overlap.overlap_down;

				pr_debug("slice %d. pos %d %d %d %d, ovl %d %d %d %d\n",
					i, start_col, end_col,
					start_row, end_row,
					overlap_up, overlap_down,
					overlap_left, overlap_right);

				slc_store->size.w = end_col - start_col + 1 -
						overlap_left - overlap_right;
				slc_store->size.h = end_row - start_row + 1 -
						overlap_up - overlap_down;
				slc_store->border.up_border = overlap_left;
				slc_store->border.down_border = overlap_left;
				slc_store->border.left_border = overlap_left;
				slc_store->border.right_border = overlap_left;
				if (cur_slc->y == 0)
					start_col_out[j][cur_slc->x] =
						start_col + overlap_left;
				if (cur_slc->x == 0)
					start_row_out[j][cur_slc->y] =
						start_row + overlap_up;

				pr_debug("scaler bypass .  %d   %d",
						start_row_out[j][cur_slc->y],
						start_col_out[j][cur_slc->x]);
			}

			switch (frm_store->color_fmt) {
			case ISP_STORE_UYVY:
				ch0_offset = start_col_out[j][cur_slc->x] * 2 +
					start_row_out[j][cur_slc->y] *
					frm_store->pitch.pitch_ch0;
				break;
			case ISP_STORE_YUV422_2FRAME:
			case ISP_STORE_YVU422_2FRAME:
				ch0_offset = start_col_out[j][cur_slc->x] +
					start_row_out[j][cur_slc->y] *
					frm_store->pitch.pitch_ch0;
				ch1_offset = start_col_out[j][cur_slc->x] +
					start_row_out[j][cur_slc->y] *
					frm_store->pitch.pitch_ch1;
				break;
			case ISP_STORE_YUV422_3FRAME:
				ch0_offset = start_col_out[j][cur_slc->x] +
					start_row_out[j][cur_slc->y] *
					frm_store->pitch.pitch_ch0;
				ch1_offset = (start_col_out[j][cur_slc->x] >> 1) +
					start_row_out[j][cur_slc->y] *
					frm_store->pitch.pitch_ch1;
				ch2_offset = (start_col_out[j][cur_slc->x] >> 1) +
					start_row_out[j][cur_slc->y] *
					frm_store->pitch.pitch_ch2;
				break;
			case ISP_STORE_YUV420_2FRAME:
			case ISP_STORE_YVU420_2FRAME:
				ch0_offset = start_col_out[j][cur_slc->x] +
					start_row_out[j][cur_slc->y] *
					frm_store->pitch.pitch_ch0;
				ch1_offset = start_col_out[j][cur_slc->x] +
					start_row_out[j][cur_slc->y] *
					frm_store->pitch.pitch_ch1 / 2;
				break;
			case ISP_STORE_YUV420_3FRAME:
				ch0_offset = start_col_out[j][cur_slc->x] +
					start_row_out[j][cur_slc->y] *
					frm_store->pitch.pitch_ch0;
				ch1_offset = (start_col_out[j][cur_slc->x] >> 1) +
					start_row_out[j][cur_slc->y] *
					frm_store->pitch.pitch_ch1 / 2;
				ch2_offset = (start_col_out[j][cur_slc->x] >> 1) +
					start_row_out[j][cur_slc->y] *
					frm_store->pitch.pitch_ch2 / 2;
				break;
			default:
				break;
			}
			slc_store->addr.addr_ch0 =
				frm_store->addr.addr_ch0 + ch0_offset;
			slc_store->addr.addr_ch1 =
				frm_store->addr.addr_ch1 + ch1_offset;
			slc_store->addr.addr_ch2 =
				frm_store->addr.addr_ch2 + ch2_offset;
		}
	}

	return 0;
}


int isp_cfg_slices(void *cfg_in,
		struct isp_slice_context *slc_ctx)
{
	int ret = 0;
	struct slice_cfg_input *in_ptr = (struct slice_cfg_input *)cfg_in;

	if (!in_ptr || !slc_ctx) {
		pr_err("error: null input ptr.\n");
		return -EFAULT;
	}
	memset(slc_ctx, 0, sizeof(struct isp_slice_context));

	cfg_slice_base_info(in_ptr, slc_ctx);

	cfg_slice_cfa_info(in_ptr, slc_ctx);

	cfg_slice_scaler_info(in_ptr, slc_ctx);

	return ret;
}

void *get_isp_slice_ctx()
{
	struct isp_slice_context *ptr;

	ptr = vzalloc(sizeof(struct isp_slice_context));
	if (IS_ERR_OR_NULL(ptr))
		return NULL;

	return ptr;
}

int put_isp_slice_ctx(void **slc_ctx)
{
	if (*slc_ctx)
		vfree(*slc_ctx);
	*slc_ctx = NULL;
	return 0;
}



/*  following section is fmcu cmds of register update for slice */

#define FMCU_PUSH(fmcu, addr, cmd) \
		fmcu->ops->push_cmdq(fmcu, addr, cmd)

static unsigned long store_base[ISP_SPATH_NUM] = {
	ISP_STORE_PRE_CAP_BASE,
	ISP_STORE_VID_BASE,
	ISP_STORE_VID_BASE,
};

static unsigned long scaler_base[ISP_SPATH_NUM] = {
	ISP_SCALER_PRE_CAP_BASE,
	ISP_SCALER_VID_BASE,
	ISP_SCALER_VID_BASE,
};

static int set_fmcu_cfg(struct isp_fmcu_ctx_desc *fmcu,
	enum isp_context_id ctx_id)
{
	uint32_t addr = 0, cmd = 0;
	unsigned long base;
	unsigned long reg_addr[ISP_CONTEXT_NUM] = {
		ISP_CFG_PRE0_START,
		ISP_CFG_CAP0_START,
		ISP_CFG_PRE1_START,
		ISP_CFG_CAP1_START,
	};

	addr = ISP_GET_REG(reg_addr[ctx_id]);
	cmd = 1;
	FMCU_PUSH(fmcu, addr, cmd);

	/*
	 * When setting CFG_TRIGGER_PULSE cmd, fmcu will wait
	 * until CFG module configs isp registers done.
	 */
	if (fmcu->fid == 0)
		base =  ISP_FMCU0_BASE;
	else
		base =  ISP_FMCU1_BASE;
	addr = ISP_GET_REG(base + ISP_FMCU_CMD);
	cmd = CFG_TRIGGER_PULSE;
	FMCU_PUSH(fmcu, addr, cmd);

	return 0;
}

static int set_slice_fetch(struct isp_fmcu_ctx_desc *fmcu,
		struct slice_fetch_info *fetch_info)
{
	uint32_t addr = 0, cmd = 0;

	addr = ISP_GET_REG(ISP_FETCH_MEM_SLICE_SIZE);
	cmd = ((fetch_info->size.h & 0xFFFF) << 16) |
			(fetch_info->size.w & 0xFFFF);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FETCH_SLICE_Y_ADDR);
	cmd = fetch_info->addr.addr_ch0;
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FETCH_SLICE_U_ADDR);
	cmd = fetch_info->addr.addr_ch1;
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FETCH_SLICE_V_ADDR);
	cmd = fetch_info->addr.addr_ch2;
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FETCH_MIPI_INFO);
	cmd = fetch_info->mipi_word_num |
			(fetch_info->mipi_byte_rel_pos << 16);
	FMCU_PUSH(fmcu, addr, cmd);

	/* dispatch size same as fetch size */
	addr = ISP_GET_REG(ISP_DISPATCH_CH0_SIZE);
	cmd = ((fetch_info->size.h & 0xFFFF) << 16) |
			(fetch_info->size.w & 0xFFFF);
	FMCU_PUSH(fmcu, addr, cmd);

	return 0;
}

static int set_slice_spath_scaler(
		struct isp_fmcu_ctx_desc *fmcu,
		uint32_t path_en,
		uint32_t ctx_idx,
		enum isp_sub_path_id spath_id,
		struct slice_scaler_info *slc_scaler)
{
	uint32_t addr = 0, cmd = 0;
	unsigned long base = scaler_base[spath_id];

	if (!path_en) {
		addr = ISP_GET_REG(ISP_SCALER_CFG) + base;
		cmd = (0 << 31) | (1 << 8) | (1 << 9);
		FMCU_PUSH(fmcu, addr, cmd);
		return 0;
	}

	/* bit31 enable path */
	addr = ISP_GET_REG(ISP_SCALER_CFG) + base;
	cmd = ISP_REG_RD(ctx_idx, base + ISP_SCALER_CFG);
	cmd |= (1 << 31);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_SCALER_SRC_SIZE) + base;
	cmd = (slc_scaler->src_size_x & 0x3FFF) |
			((slc_scaler->src_size_y & 0x3FFF) << 16);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_SCALER_DES_SIZE) + base;
	cmd = (slc_scaler->dst_size_x & 0x3FFF) |
			((slc_scaler->dst_size_y & 0x3FFF) << 16);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_SCALER_TRIM0_START) + base;
	cmd = (slc_scaler->trim0_start_x & 0x1FFF) |
			((slc_scaler->trim0_start_y & 0x1FFF) << 16);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_SCALER_TRIM0_SIZE) + base;
	cmd = (slc_scaler->trim0_size_x & 0x1FFF) |
			((slc_scaler->trim0_size_y & 0x1FFF) << 16);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_SCALER_IP) + base;
	cmd = (slc_scaler->scaler_ip_rmd & 0x1FFF) |
			((slc_scaler->scaler_ip_int & 0xF) << 16);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_SCALER_CIP) + base;
	cmd = (slc_scaler->scaler_cip_rmd & 0x1FFF) |
			((slc_scaler->scaler_cip_int & 0xF) << 16);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_SCALER_TRIM1_START) + base;
	cmd = (slc_scaler->trim1_start_x & 0x1FFF) |
			((slc_scaler->trim1_start_y & 0x1FFF) << 16);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_SCALER_TRIM1_SIZE) + base;
	cmd = (slc_scaler->trim1_size_x & 0x1FFF) |
			((slc_scaler->trim1_size_y & 0x1FFF) << 16);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_SCALER_VER_IP) + base;
	cmd = (slc_scaler->scaler_ip_rmd_ver & 0x1FFF) |
			((slc_scaler->scaler_ip_int_ver & 0xF) << 16);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_SCALER_VER_CIP) + base;
	cmd = (slc_scaler->scaler_cip_rmd_ver & 0x1FFF) |
			((slc_scaler->scaler_cip_int_ver & 0xF) << 16);
	FMCU_PUSH(fmcu, addr, cmd);

	return 0;
}


static int set_slice_spath_store(
		struct isp_fmcu_ctx_desc *fmcu,
		uint32_t path_en,
		uint32_t ctx_idx,
		enum isp_sub_path_id spath_id,
		struct slice_store_info *slc_store)
{
	uint32_t addr = 0, cmd = 0;
	unsigned long base = store_base[spath_id];

	if (!path_en) {
		/* bit0 bypass store */
		addr = ISP_GET_REG(ISP_STORE_PARAM) + base;
		cmd = 1;
		FMCU_PUSH(fmcu, addr, cmd);
		return 0;
	}
	addr = ISP_GET_REG(ISP_STORE_PARAM) + base;
	cmd = ISP_REG_RD(ctx_idx, base + ISP_STORE_PARAM) & ~1;
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_STORE_SLICE_SIZE) + base;
	cmd = ((slc_store->size.h & 0xFFFF) << 16) |
			(slc_store->size.w & 0xFFFF);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_STORE_BORDER) + base;
	cmd = (slc_store->border.up_border & 0xFF) |
			((slc_store->border.down_border & 0xFF) << 8) |
			((slc_store->border.left_border & 0xFF) << 16) |
			((slc_store->border.right_border & 0xFF) << 24);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_STORE_SLICE_Y_ADDR) + base;
	cmd = slc_store->addr.addr_ch0;
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_STORE_SLICE_U_ADDR) + base;
	cmd = slc_store->addr.addr_ch1;
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_STORE_SLICE_V_ADDR) + base;
	cmd = slc_store->addr.addr_ch2;
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_STORE_SHADOW_CLR) + base;
	cmd = 1;
	FMCU_PUSH(fmcu, addr, cmd);

	return 0;
}

int isp_set_slices_fmcu_cmds(
		void *fmcu_handle, void *slc_handle,
		uint32_t ctx_idx, uint32_t wmode)
{
	int i, j;
	unsigned long base;
	struct isp_slice_desc *cur_slc;
	struct slice_store_info *slc_store;
	struct slice_scaler_info *slc_scaler;
	struct isp_slice_context *slc_ctx;
	struct isp_fmcu_ctx_desc *fmcu;
	uint32_t reg_off, addr = 0, cmd = 0;
	uint32_t shadow_done_cmd[ISP_CONTEXT_NUM] = {
		PRE0_SHADOW_DONE, CAP0_SHADOW_DONE,
		PRE1_SHADOW_DONE, CAP1_SHADOW_DONE,
	};
	uint32_t all_done_cmd[ISP_CONTEXT_NUM] = {
		PRE0_ALL_DONE, CAP0_ALL_DONE,
		PRE1_ALL_DONE, CAP1_ALL_DONE,
	};

	if (!fmcu_handle || !slc_handle) {
		pr_err("error: null input ptr.\n");
		return -EFAULT;
	}

	slc_ctx = (struct isp_slice_context *)slc_handle;
	if (slc_ctx->slice_num < 1) {
		pr_err("warn: should not use slices.\n");
		return -EINVAL;
	}

	fmcu = (struct isp_fmcu_ctx_desc *)fmcu_handle;
	if (fmcu->fid == 0)
		base =  ISP_FMCU0_BASE;
	else
		base =  ISP_FMCU1_BASE;

	cur_slc = &slc_ctx->slices[0];
	for (i = 0; i < SLICE_NUM_MAX; i++, cur_slc++) {
		if (cur_slc->valid == 0)
			continue;
		if (wmode == ISP_CFG_MODE)
			set_fmcu_cfg(fmcu, ctx_idx);

		set_slice_fetch(fmcu, &cur_slc->slice_fetch);

		for (j = 0; j < ISP_SPATH_NUM - 1; j++) {
			slc_store = &cur_slc->slice_store[j];
			slc_scaler = &cur_slc->slice_scaler[j];
			set_slice_spath_scaler(fmcu,
				cur_slc->path_en[j], ctx_idx, j, slc_scaler);
			set_slice_spath_store(fmcu,
				cur_slc->path_en[j], ctx_idx, j, slc_store);
		}

		if (wmode == ISP_CFG_MODE) {
			reg_off = ISP_CFG_CAP_FMCU_RDY;
			addr = ISP_GET_REG(reg_off);
		} else
			addr = ISP_GET_REG(ISP_FETCH_START);
		cmd = 1;
		FMCU_PUSH(fmcu, addr, cmd);

		addr = ISP_GET_REG(base + ISP_FMCU_CMD);
		cmd = shadow_done_cmd[ctx_idx];
		FMCU_PUSH(fmcu, addr, cmd);

		addr = ISP_GET_REG(base + ISP_FMCU_CMD);
		cmd = all_done_cmd[ctx_idx];
		FMCU_PUSH(fmcu, addr, cmd);
	}

	return 0;
}


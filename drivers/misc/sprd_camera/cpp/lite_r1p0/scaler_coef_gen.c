/*
 * Copyright (C) 2015-2016 Spreadtrum Communications Inc.
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
#include <linux/io.h>
#include <linux/kernel.h>

#include "scaler_coef_cal.h"
#include "scaler_coef_gen.h"

#define MIN_POOL_SIZE  (6 * 1024)

#ifdef TRUE
#undef TRUE
#endif

#ifdef FALSE
#undef FALSE
#endif
#define TRUE           1
#define FALSE          0

#define UP             1
#define DOWN           0

#define COUNT          256
#define SIGN2(input, p) {if (p >= 0) input = 1; if (p < 0) input = -1; }
#define MAX(_x, _y)    (((_x) > (_y)) ? (_x) : (_y))

#define pi             3.14159265357

struct scaler_coef {
	int n_phase;
	int luma_hor_tap;
	int luma_ver_tap;
	int chrome_hor_tap;
	int chrome_ver_tap;
	int *luma_hor_coef;
	int *luma_ver_coef;
	int *chrome_hor_coef;
	int *chrome_ver_coef;
	int row_in;
	int col_in;
	int row_out;
	int col_out;
	int InitFlag;
};

struct gsc_mem_pool {
	unsigned long begin_addr;
	unsigned int total_size;
	unsigned int used_size;
};

struct scaler_coef_info {
	/* Luma horizontal coefficients table */
	short y_hor_coef[8][8];
	/* Chroma horizontal coefficients table */
	short c_hor_coef[8][4];
	/* Luma vertical up coefficients table */
	short y_ver_up_coef[8][4];
	/* Chroma vertical up coefficients table */
	short c_ver_up_coef[8][4];
	/* Luma vertical down coefficients table */
	short y_ver_down_coef[9][16];
	/* Chroma veritical down coefficients table */
	short c_ver_down_coef[9][16];
};

const int up_scaling_8_4[32] = {
	0, 255, 1, 0,
	-12, 246, 23, -1,
	-18, 222, 58, -6,
	-18, 186, 99, -11,
	-16, 144, 144, -16,
	-11, 99, 186, -18,
	-6, 58, 222, -18,
	-1, 23, 246, -12
};

const int up_scaling_8_8[64] = {
	0, 0, 0, 255, 1, 0, 0, 0,
	0, 0, -12, 246, 23, -1, 0, 0,
	0, 0, -18, 222, 58, -6, 0, 0,
	0, 0, -18, 186, 99, -11, 0, 0,
	0, 0, -16, 144, 144, -16, 0, 0,
	0, 0, -11, 99, 186, -18, 0, 0,
	0, 0, -6, 58, 222, -18, 0, 0,
	0, 0, -1, 23, 246, -12, 0, 0
};

static unsigned char init_pool(void *buffer_ptr, unsigned int buffer_size,
			       struct gsc_mem_pool *pool_ptr)
{
	if (NULL == buffer_ptr || 0 == buffer_size || NULL == pool_ptr)
		return FALSE;

	if (buffer_size < MIN_POOL_SIZE)
		return FALSE;

	pool_ptr->begin_addr = (unsigned long)buffer_ptr;
	pool_ptr->total_size = buffer_size;
	pool_ptr->used_size = 0;

	return TRUE;
}

static void *alloc_mem(unsigned int size, unsigned int align_shift,
		       struct gsc_mem_pool *pool_ptr)
{
	unsigned long begin_addr = 0;
	unsigned long temp_addr = 0;

	if (pool_ptr == NULL)
		return NULL;

	begin_addr = pool_ptr->begin_addr;
	temp_addr = begin_addr + pool_ptr->used_size;
	temp_addr =
	    ((temp_addr + (1 << align_shift) -
	      1) >> align_shift) << align_shift;

	if ((temp_addr + size) > (begin_addr + pool_ptr->total_size)) {
		panic("here");
		return NULL;
	}

	pool_ptr->used_size = (temp_addr + size) - begin_addr;
	memset((void *)temp_addr, 0, size);

	return (void *)temp_addr;
}

static unsigned char free_mem(void *buffer_ptr, unsigned int buffer_size,
			      struct gsc_mem_pool *pool_ptr)
{
	if (NULL == buffer_ptr || 0 >= buffer_size || NULL == pool_ptr)
		return FALSE;

	if (buffer_ptr ==
	    (void *)((pool_ptr->used_size - buffer_size) +
		     pool_ptr->begin_addr))
		pool_ptr->used_size -= buffer_size;

	return TRUE;
}

static void check_coef_range_ycom_h(short cong_ycom_hor[8][32])
{
	short tmpi, tmp_s;
	short tmpval;
	short i, j;

	for (i = 0; i < 8; i++) {
		for (j = 0; j < 8; j++) {
			tmpval = cong_ycom_hor[i][j];
			if (tmpval > 255) {
				tmpi = tmpval - 255;
				cong_ycom_hor[i][j] = 255;
				tmp_s = abs(tmpi);
				if ((tmp_s & 1) == 1) {
					cong_ycom_hor[i][j + 1] =
					    cong_ycom_hor[i][j + 1] + (tmpi +
								       1) / 2;
					cong_ycom_hor[i][j - 1] =
					    cong_ycom_hor[i][j - 1] + (tmpi -
								       1) / 2;
				} else {
					cong_ycom_hor[i][j + 1] =
					    cong_ycom_hor[i][j + 1] +
					    (tmpi) / 2;
					cong_ycom_hor[i][j - 1] =
					    cong_ycom_hor[i][j - 1] +
					    (tmpi) / 2;
				}
			}
		}
	}
}

static void check_coef_range_uvcom_h(short cong_uvcom_hor[8][32])
{
	short tmpi, tmp_s;
	short tmpval;
	short i, j;

	for (i = 0; i < 8; i++) {
		for (j = 0; j < 4; j++) {
			tmpval = cong_uvcom_hor[i][j];
			if (tmpval > 255) {
				tmpi = tmpval - 255;
				cong_uvcom_hor[i][j] = 255;
				tmp_s = abs(tmpi);
				if ((tmp_s & 1) == 1) {
					cong_uvcom_hor[i][j + 1] =
					    cong_uvcom_hor[i][j + 1] + (tmpi +
									1) / 2;
					cong_uvcom_hor[i][j - 1] =
					    cong_uvcom_hor[i][j - 1] + (tmpi -
									1) / 2;
				} else {
					cong_uvcom_hor[i][j + 1] =
					    cong_uvcom_hor[i][j + 1] +
					    (tmpi) / 2;
					cong_uvcom_hor[i][j - 1] =
					    cong_uvcom_hor[i][j - 1] +
					    (tmpi) / 2;
				}
			}
		}
	}
}

static void check_coef_range_ycom_v(short cong_ycom_ver[9][32],
				    unsigned short luma_ver_tap)
{
	short tmpi, tmp_s;
	short tmpval;
	short i, j;

	for (i = 0; i < 8; i++) {
		for (j = 0; j < luma_ver_tap; j++) {
			tmpval = cong_ycom_ver[i][j];
			if (tmpval > 255) {
				tmpi = tmpval - 255;
				cong_ycom_ver[i][j] = 255;
				tmp_s = abs(tmpi);
				if ((tmp_s & 1) == 1) {
					cong_ycom_ver[i][j + 1] =
					    cong_ycom_ver[i][j + 1] + (tmpi +
								       1) / 2;
					cong_ycom_ver[i][j - 1] =
					    cong_ycom_ver[i][j - 1] + (tmpi -
								       1) / 2;
				} else {
					cong_ycom_ver[i][j + 1] =
					    cong_ycom_ver[i][j + 1] +
					    (tmpi) / 2;
					cong_ycom_ver[i][j - 1] =
					    cong_ycom_ver[i][j - 1] +
					    (tmpi) / 2;
				}
			}
		}
	}
}

static void check_coef_range_uvcom_v(short cong_uvcom_ver[9][32],
				     unsigned short chrome_ver_tap)
{
	short tmpi, tmp_s;
	short tmpval;
	short i, j;

	for (i = 0; i < 8; i++) {
		for (j = 0; j < chrome_ver_tap; j++) {
			tmpval = cong_uvcom_ver[i][j];
			if (tmpval > 255) {
				tmpi = tmpval - 255;
				cong_uvcom_ver[i][j] = 255;
				tmp_s = abs(tmpi);
				if ((tmp_s & 1) == 1) {
					cong_uvcom_ver[i][j + 1] =
					    cong_uvcom_ver[i][j + 1] + (tmpi +
									1) / 2;
					cong_uvcom_ver[i][j - 1] =
					    cong_uvcom_ver[i][j - 1] + (tmpi -
									1) / 2;
				} else {
					cong_uvcom_ver[i][j + 1] =
					    cong_uvcom_ver[i][j + 1] +
					    (tmpi) / 2;
					cong_uvcom_ver[i][j - 1] =
					    cong_uvcom_ver[i][j - 1] +
					    (tmpi) / 2;
				}
			}
		}
	}
}

static void calc_ver_edge_coef_y(short cong_ycom_ver[9][32], short i_ver_bak,
				 short d_ver_bak, unsigned short luma_ver_tap)
{
	int phase_temp[9];
	int acc, i_sample_cnt;
	int l;
	unsigned char phase, spec_tap;
	short i, j;

	for (j = 0; j <= 8; j++)
		phase_temp[j] = j * i_ver_bak / 8;

	acc = 0;
	i_sample_cnt = 0;
	phase = 0;

	for (i = 0; i < i_ver_bak; i++) {
		spec_tap = i % 2;
		while (acc >= i_ver_bak) {
			acc -= i_ver_bak;
			i_sample_cnt++;
		}

		for (j = 0; j < 8; j++) {
			if (acc >= phase_temp[j] && acc < phase_temp[j + 1]) {
				phase = (unsigned char)j;
				break;
			}
		}

		for (j = 1 - luma_ver_tap / 2; j <= luma_ver_tap / 2; j++) {
			l = i_sample_cnt + j;
			if (l <= 0) {
				cong_ycom_ver[8][spec_tap] +=
				    cong_ycom_ver[phase][j + luma_ver_tap / 2 -
							 1];
			} else {
				if (l >= d_ver_bak - 1)
					cong_ycom_ver[8][spec_tap + 2] +=
					    cong_ycom_ver[phase][j +
								 luma_ver_tap /
								 2 - 1];
			}
		}
		acc += d_ver_bak;
	}
}

static void calc_ver_edge_coef_uv(short cong_uvcom_ver[9][32],
				  short i_ver_bak_uv, short d_ver_bak,
				  unsigned short chrome_ver_tap)
{
	int phase_temp[9];
	int acc, i_sample_cnt;
	int l;
	unsigned char phase, spec_tap;
	short i, j;

	for (j = 0; j <= 8; j++)
		phase_temp[j] = j * i_ver_bak_uv / 8;

	acc = 0;
	i_sample_cnt = 0;
	phase = 0;

	for (i = 0; i < i_ver_bak_uv; i++) {
		spec_tap = i % 2;
		while (acc >= i_ver_bak_uv) {
			acc -= i_ver_bak_uv;
			i_sample_cnt++;
		}

		for (j = 0; j < 8; j++) {
			if (acc >= phase_temp[j] && acc < phase_temp[j + 1]) {
				phase = (unsigned char)j;
				break;
			}
		}

		for (j = 1 - chrome_ver_tap / 2; j <= chrome_ver_tap / 2; j++) {
			l = i_sample_cnt + j;
			if (l <= 0) {
				cong_uvcom_ver[8][spec_tap] +=
				    cong_uvcom_ver[phase][j +
							  chrome_ver_tap / 2 -
							  1];
			} else {
				if (l >= d_ver_bak - 1)
					cong_uvcom_ver[8][spec_tap + 2] +=
					    cong_uvcom_ver[phase][j +
								  chrome_ver_tap
								  / 2 - 1];
			}
		}
		acc += d_ver_bak;
	}
}

#if 0
static void config_reg_scaling_coef(unsigned int *reg_scaling_hor,
				    int hor_buf_len,
				    unsigned int *reg_scaling_ver,
				    int ver_buf_len,
				    short cong_ycom_hor[8][32],
				    short cong_uvcom_hor[8][32],
				    short cong_ycom_ver[9][32],
				    short cong_uvcom_ver[9][32],
				    unsigned char bScalingUp)
{
	unsigned char i, j;
	unsigned int y_reg_hor[8][4];
	unsigned int uv_reg_hor[8][2];
	unsigned int yuv_reg_ver[9][8];
	unsigned short p0, p1;
	unsigned int reg;

	/* horizontal Y Scaling Coef Config register */
	for (i = 0; i < 8; i++) {
		p0 = (unsigned short)cong_ycom_hor[i][7];
		p1 = (unsigned short)cong_ycom_hor[i][6];

		reg = ((p0 & 0x1ff)) | ((p1 & 0x1ff) << 9);
		y_reg_hor[i][0] = reg;

		p0 = (unsigned short)cong_ycom_hor[i][5];
		p1 = (unsigned short)cong_ycom_hor[i][4];

		reg = ((p0 & 0x1ff)) | ((p1 & 0x1ff) << 9);
		y_reg_hor[i][1] = reg;

		p0 = (unsigned short)cong_ycom_hor[i][3];
		p1 = (unsigned short)cong_ycom_hor[i][2];

		reg = ((p0 & 0x1ff)) | ((p1 & 0x1ff) << 9);
		y_reg_hor[i][2] = reg;

		p0 = (unsigned short)cong_ycom_hor[i][1];
		p1 = (unsigned short)cong_ycom_hor[i][0];

		reg = ((p0 & 0x1ff)) | ((p1 & 0x1ff) << 9);
		y_reg_hor[i][3] = reg;
	}

	/* horizontal UV Scaling Coef Config register */
	for (i = 0; i < 8; i++) {
		p0 = (unsigned short)cong_uvcom_hor[i][3];
		p1 = (unsigned short)cong_uvcom_hor[i][2];

		reg = ((p0 & 0x1ff)) | ((p1 & 0x1ff) << 9);
		uv_reg_hor[i][0] = reg;

		p0 = (unsigned short)cong_uvcom_hor[i][1];
		p1 = (unsigned short)cong_uvcom_hor[i][0];

		reg = ((p0 & 0x1ff)) | ((p1 & 0x1ff) << 9);
		uv_reg_hor[i][1] = reg;
	}

	/* vertical  YUV Scaling Coef Config register */
	for (i = 0; i < 8; i++) {
		for (j = 0; j < 8; j++) {
			p0 = (unsigned short)cong_ycom_ver[i][j];
			p1 = (unsigned short)cong_uvcom_ver[i][j];

			reg = ((p0 & 0x1ff)) | ((p1 & 0x1ff) << 9);
			yuv_reg_ver[i][j] = reg;
		}
	}

	/* vertical special border */
	for (j = 0; j < 4; j++) {
		p0 = (unsigned short)cong_ycom_ver[8][j];
		p1 = (unsigned short)cong_uvcom_ver[8][j];

		reg = ((p0 & 0x1ff)) | ((p1 & 0x1ff) << 9);
		yuv_reg_ver[8][j] = reg;
	}

	/* config to line buffer */

	/* y component horizontal */
	for (i = 0; i < 8; i++)
		for (j = 0; j < 4; j++)
			reg_scaling_hor[i * 4 + j] = y_reg_hor[i][j];

	/* uv cmoponent horizontal */
	for (i = 0; i < 8; i++)
		for (j = 0; j < 2; j++)
			reg_scaling_hor[i * 2 + j + 32] = uv_reg_hor[i][j];

	/* yuv components vertical */
	memset(reg_scaling_ver, 0, ver_buf_len * sizeof(unsigned int));

	if (bScalingUp) {
		for (i = 0; i < 8; i++)
			for (j = 0; j < 4; j++)
				reg_scaling_ver[i * 4 + j] = yuv_reg_ver[i][j];

		/* vertical special border */
		for (j = 0; j < 4; j++)
			reg_scaling_ver[64 + j] = yuv_reg_ver[8][j];
	} else {
		for (i = 0; i < 8; i++)
			for (j = 0; j < 8; j++)
				reg_scaling_ver[i * 8 + j] = yuv_reg_ver[i][j];

		/* vertical special border */
		for (j = 0; j < 4; j++)
			reg_scaling_ver[64 + j] = yuv_reg_ver[8][j];
	}
}
#endif

/* normalize the array */
static void dcam_normalize_inter(int *data, short *int_data, unsigned char ilen)
{
	int i;
	int sum_val;

	sum_val = 0;

	for (i = 0; i < ilen; i++)
		sum_val += data[i];

	for (i = 0; i < ilen; i++)
		int_data[i] = (short)(data[i] * 256 / sum_val);
}

static short dcam_sum_fun(short *data, signed char ilen)
{
	short tmp_sum;
	signed char i;

	tmp_sum = 0;

	for (i = 0; i < ilen; i++)
		tmp_sum += *data++;

	return tmp_sum;
}

/* push down the peak when the sum is over 256. */
static void dcam_adjust_filter_inter(short *filter, unsigned char ilen)
{
	unsigned char i, midi;
	int tmpi, tmp_s;
	int tmp_val = 0;

	tmpi = dcam_sum_fun(filter, ilen) - 256;
	midi = ilen >> 1;
	SIGN2(tmp_val, tmpi);

	if ((tmpi & 1) == 1) {
		filter[midi] = filter[midi] - tmp_val;
		tmpi -= tmp_val;
	}

	tmp_s = abs(tmpi >> 1);

	if ((ilen & 1) == 1) {
		for (i = 0; i < tmp_s; i++) {
			filter[midi - (i + 1)] =
			    filter[midi - (i + 1)] - tmp_val;
			filter[midi + (i + 1)] =
			    filter[midi + (i + 1)] - tmp_val;
		}
	} else {
		for (i = 0; i < tmp_s; i++) {
			filter[midi - (i + 1)] =
			    filter[midi - (i + 1)] - tmp_val;
			filter[midi + i] = filter[midi + i] - tmp_val;
		}
	}

	if (filter[midi] > 255) {
		tmp_val = filter[midi];
		filter[midi] = 255;
		filter[midi - 1] = filter[midi - 1] + tmp_val - 255;
	}
}

static short cal_y_model_coef(short coef_length,
			      short *coef_data_ptr,
			      short n, short m, struct gsc_mem_pool *pool_ptr)
{
	signed char mount;
	short i, mid_i, kk, j, sum_val;
	int *d_filter_fix = alloc_mem(COUNT * sizeof(int), 2, pool_ptr);
	int *d_tmp_filter_fix = alloc_mem(COUNT * sizeof(int), 2, pool_ptr);
	short *normalized_filter_fix =
	    alloc_mem(COUNT * sizeof(short), 2, pool_ptr);
	long long out_size = (long long)n;
	long long in_size = (long long)m;

	mid_i = coef_length >> 1;

	/* generate coef */
	for (i = -1; i < mid_i; i++) {
		int index0 =
		    (int)(((i + 1) * out_size * 4096 * 128) / (4 * MAX(in_size,
								   out_size)));
		int index1 = (int)(((i + 1) * 4096 * 128) / (4 * in_size));
		int sin_value0 = scale_sin_32(index0);
		int sin_value1 = scale_sin_32(index1);
		int win_index =
		    (int)((mid_i - i - 1) * 4 * 4096 * 128 / coef_length);
		int cos_value = scale_cos_32(win_index);

		if (-1 == i)
			d_filter_fix[mid_i + i + 1] =
			    (int)(out_size *
				  (54 * 1024 * 1024 -
				   46 * (long long)cos_value) / (MAX(in_size,
								     out_size) *
								 100));
		else
			d_filter_fix[mid_i + i + 1] =
			    (int)((long long)sin_value0 *
				  (54 * 1024 * 1024 -
				   46 * (long long)cos_value) / (in_size *
								 sin_value1 *
								 100));

		d_filter_fix[mid_i - (i + 1)] = d_filter_fix[mid_i + i + 1];
	}

	/* normalize and adjust */
	for (i = 0; i < 8; i++) {
		memset((void *)d_tmp_filter_fix, 0, COUNT * sizeof(int));
		memset((void *)normalized_filter_fix, 0, COUNT * sizeof(short));

		mount = 0;
		for (j = i; j < coef_length; j += 8) {
			d_tmp_filter_fix[mount] = d_filter_fix[j];
			mount++;
		}

		dcam_normalize_inter(d_tmp_filter_fix, normalized_filter_fix,
				     (signed char)mount);
		sum_val = dcam_sum_fun(normalized_filter_fix, mount);
		if (sum_val != 256)
			dcam_adjust_filter_inter(normalized_filter_fix, mount);

		mount = 0;
		for (kk = i; kk < coef_length; kk += 8) {
			coef_data_ptr[kk] = normalized_filter_fix[mount];
			mount++;
		}
	}

	free_mem(normalized_filter_fix, COUNT * sizeof(short), pool_ptr);
	free_mem(d_tmp_filter_fix, COUNT * sizeof(int), pool_ptr);
	free_mem(d_filter_fix, COUNT * sizeof(int), pool_ptr);

	return 0;
}

/* cal Y model */
static short cal_y_scaling_coef(short tap,
				short d,
				short i,
				short *y_coef_data_ptr,
				short dir, struct gsc_mem_pool *pool_ptr)
{
	const short phase = 8;
	short coef_length;

	/* x direction */
	if (dir == 1) {
		coef_length = (short)(tap * phase);
		cal_y_model_coef(coef_length, y_coef_data_ptr, i, d, pool_ptr);
	} else {
		if (d > i) {
			/* down */
			coef_length = (short)(tap * phase);
			cal_y_model_coef(coef_length, y_coef_data_ptr, i, d,
					 pool_ptr);
		} else {
			/* up */
			coef_length = (short)(4 * phase);
			cal_y_model_coef(coef_length, y_coef_data_ptr, i, d,
					 pool_ptr);
		}
	}

	return coef_length;
}

/* cal UV model */
static short cal_uv_scaling_coef(short tap,
				 short d,
				 short i,
				 short *uv_coef_data_ptr,
				 short dir, struct gsc_mem_pool *pool_ptr)
{
	short uv_coef_length;

	if (dir == 1) {
		/* x direction */
		uv_coef_length = (short)(tap * 8);
		cal_y_model_coef(uv_coef_length, uv_coef_data_ptr, i, d,
				 pool_ptr);
	} else {
		/* y direction */
		if (d > i)
			uv_coef_length = (short)(tap * 8);
		else
			uv_coef_length = (short)(4 * 8);
		cal_y_model_coef(uv_coef_length, uv_coef_data_ptr, i, d,
				 pool_ptr);
	}

	return uv_coef_length;
}

static void get_filter(short pos_start,
		       short *coef_data_ptr,
		       short *out_filter,
		       short i_hor, short coef_len, short *filter_len)
{
	short i;

	for (i = 0; i < i_hor; i++) {
		short len = 0;
		short j;
		short pos = pos_start + i;

		while (pos >= i_hor)
			pos -= i_hor;

		for (j = 0; j < coef_len; j += i_hor) {
			*out_filter++ = coef_data_ptr[j + pos];
			len++;
		}
		*filter_len++ = len;
	}
}

static void write_scaler_coef(short scale_coef[8][32],
			      short *coef_ptr, short len)
{
	int i, j;

	for (i = 0; i < 8; i++)
		for (j = 0; j < len; j++)
			scale_coef[i][j] = *(coef_ptr + i * len + len - 1 - j);
}

static void dcam_gen_scaler_coef(short i_w,
				 short i_h,
				 short o_w,
				 short o_h,
				 unsigned char scaling2yuv420,
				 unsigned char *scaler_tap,
				 unsigned char *pCchrome_tap,
				 struct scaler_coef_info *pCmdInfo,
				 int flag, struct gsc_mem_pool *pool_ptr)
{
	/*short iD_hor = i_w;*/
	short d_hor_bak = i_w;
	short i_hor, i_hor_bak = o_w;

	short d_ver = i_h, d_ver_bak = i_h;
	short i_ver = o_h, i_ver_bak = o_h;
	short i_ver_bak_uv = o_h;

	short *cong_ycom_hor = alloc_mem(8 * 32 * sizeof(short), 2, pool_ptr);
	short *cong_uvcom_hor = alloc_mem(8 * 32 * sizeof(short), 2, pool_ptr);
	short *cong_ycom_ver = alloc_mem(9 * 32 * sizeof(short), 2, pool_ptr);
	short *cong_uvcom_ver = alloc_mem(9 * 32 * sizeof(short), 2, pool_ptr);

	/*char *coef_txt2 = alloc_mem(COUNT * sizeof(char), 0, pool_ptr);*/
	/*char *coef_txt4 = alloc_mem(COUNT * sizeof(char), 0, pool_ptr);*/
	/*char *coef_txt5 = alloc_mem(COUNT * sizeof(char), 0, pool_ptr);*/

	short *y_coef_data_x = alloc_mem(COUNT * sizeof(short), 2, pool_ptr);
	short *uv_coef_data_x = alloc_mem(COUNT * sizeof(short), 2, pool_ptr);
	short *y_coef_data_y = alloc_mem(COUNT * sizeof(short), 2, pool_ptr);
	short *uv_coef_data_y = alloc_mem(COUNT * sizeof(short), 2, pool_ptr);

	short coef_len, i, j;
	short *y_hor_filter = alloc_mem(COUNT * sizeof(short), 2, pool_ptr);
	short *uv_hor_filter = alloc_mem(COUNT * sizeof(short), 2, pool_ptr);
	short *y_ver_filter = alloc_mem(COUNT * sizeof(short), 2, pool_ptr);
	short *uv_ver_filter = alloc_mem(COUNT * sizeof(short), 2, pool_ptr);
	short *y_hor_filter_len = alloc_mem(100 * sizeof(short), 2, pool_ptr);
	short *uv_hor_filter_len = alloc_mem(100 * sizeof(short), 2, pool_ptr);
	short *y_ver_filter_len = alloc_mem(100 * sizeof(short), 2, pool_ptr);
	short *uv_ver_filter_len = alloc_mem(100 * sizeof(short), 2, pool_ptr);

	short pos_start;
	/*short tmpi, tmp_s;*/
	/*short tmpval;*/
	/*unsigned short tmpshift;*/

	/*int scaler_coef_buf_len_hor;*/
	/*int scaler_coef_buf_len_ver;*/
	/*unsigned char bScalingUp = FALSE;*/

	unsigned short luma_ver_tap, chrome_ver_tap;
	unsigned short luma_ver_maxtap = 8, chrome_ver_maxtap = 8;
	/*unsigned long bit_check = 0x80000000;*/

	pr_debug("%s: %p, %p, %p, %p\n", __func__,
		 cong_ycom_hor, cong_uvcom_hor, cong_ycom_ver, cong_uvcom_ver);
	pr_debug("%s: %p, %p, %p, %p\n", __func__,
		 y_coef_data_x, uv_coef_data_x, y_coef_data_y, uv_coef_data_y);
	pr_debug("%s: %p, %p, %p, %p\n", __func__,
		 y_hor_filter, uv_hor_filter, y_ver_filter, uv_ver_filter);

	i_hor = 8;

	/* horizontal direction */
	/* calculate coefficients of Y component in horizontal direction */
	coef_len =
	    cal_y_scaling_coef(8, d_hor_bak, i_hor_bak, y_coef_data_x, 1,
			       pool_ptr);

	pos_start = coef_len / 2;
	while (pos_start >= i_hor)
		pos_start -= i_hor;

	get_filter(pos_start, y_coef_data_x, y_hor_filter, i_hor, coef_len,
		   y_hor_filter_len);

	write_scaler_coef((short (*)[32])cong_ycom_hor, y_hor_filter, 8);

	check_coef_range_ycom_h((short (*)[32])cong_ycom_hor);

	/* calculate coefficients of UV component in horizontal direction */
	coef_len =
	    cal_uv_scaling_coef(4, d_hor_bak, i_hor_bak, uv_coef_data_x, 1,
				pool_ptr);

	pos_start = coef_len / 2;
	while (pos_start >= i_hor)
		pos_start -= i_hor;

	get_filter(pos_start, uv_coef_data_x, uv_hor_filter, i_hor, coef_len,
		   uv_hor_filter_len);

	write_scaler_coef((short (*)[32])cong_uvcom_hor, uv_hor_filter, 4);

	check_coef_range_uvcom_h((short (*)[32])cong_uvcom_hor);

	/* vertical scaling coef */
	luma_ver_tap = ((unsigned char)(d_ver / i_ver)) * 2;
	chrome_ver_tap = ((unsigned char)(d_ver / i_ver)) * 2;

	if (luma_ver_tap > luma_ver_maxtap)
		luma_ver_tap = luma_ver_maxtap;

	if (luma_ver_tap <= 2)
		luma_ver_tap = 4;

	i_ver_bak_uv = i_ver_bak;

	*scaler_tap = (unsigned char)luma_ver_tap;
	i_ver = 8;

	/* vertical direction */
	/* calculate coefficients of Y component in vertical direction */
	coef_len =
	    cal_y_scaling_coef(luma_ver_tap, d_ver_bak, i_ver_bak,
			       y_coef_data_y, 0, pool_ptr);

	pos_start = coef_len / 2;
	while (pos_start >= i_ver)
		pos_start -= i_ver;

	get_filter(pos_start, y_coef_data_y, y_ver_filter, i_ver, coef_len,
		   y_ver_filter_len);

	write_scaler_coef((short (*)[32])cong_ycom_ver, y_ver_filter,
			  y_ver_filter_len[0]);

	check_coef_range_ycom_v((short (*)[32])cong_ycom_ver, luma_ver_tap);

	/* calculate coefficients of UV component in vertical direction */
	if (scaling2yuv420 == 1) {
		i_ver_bak_uv /= 2;
		chrome_ver_tap *= 2;
		chrome_ver_maxtap = 16;
	}

	if (chrome_ver_tap > chrome_ver_maxtap)
		chrome_ver_tap = chrome_ver_maxtap;

	if (chrome_ver_tap <= 2)
		chrome_ver_tap = 4;

	*pCchrome_tap = (unsigned char)chrome_ver_tap;

	if (scaling2yuv420 == 1)
		i_ver = 8;

	coef_len =
	    cal_uv_scaling_coef((short)(chrome_ver_tap), d_ver_bak,
				i_ver_bak_uv, uv_coef_data_y, 0, pool_ptr);

	pos_start = coef_len / 2;
	while (pos_start >= i_ver)
		pos_start -= i_ver;

	get_filter(pos_start, uv_coef_data_y, uv_ver_filter, i_ver, coef_len,
		   uv_ver_filter_len);

	write_scaler_coef((short (*)[32])cong_uvcom_ver, uv_ver_filter,
			  uv_ver_filter_len[0]);

	check_coef_range_uvcom_v((short (*)[32])cong_uvcom_ver, chrome_ver_tap);

	/* edge processing
	 * Y vertical
	 * scaling down and ratio<=1/2 using scaling down scaler,
	 * otherwise use scaling up scaler,
	 * scaling up doesn't use this special edge phase
	 */
	if (2 * i_ver_bak <= d_ver_bak)
		calc_ver_edge_coef_y((short (*)[32])cong_ycom_ver, i_ver_bak,
				     d_ver_bak, luma_ver_tap);

	/* calculate edge coefficients of UV component in vertical direction
	 * scaling down and ratio<=1/2 using scaling down scaler,
	 * otherwise use scaling up scaler,
	 * scaling up doesn't use this special edge phase
	 */
	if (2 * i_ver_bak_uv <= d_ver_bak)
		calc_ver_edge_coef_uv((short (*)[32])cong_uvcom_ver,
				      i_ver_bak_uv, d_ver_bak, chrome_ver_tap);

	if (LUMA_COEF == flag || ALL_COEF == flag) {
		for (i = 0; i < 8; i++) {
			for (j = 0; j < 8; j++)
				pCmdInfo->y_hor_coef[i][j] =
				    (short)*(cong_ycom_hor + (i << 5) + j);
		}

		for (i = 0; i < 9; i++) {
			for (j = 0; j < luma_ver_tap; j++)
				pCmdInfo->y_ver_down_coef[i][j] =
				    (short)*(cong_ycom_ver + (i << 5) + j);
		}
	}

	if (CHROMA_COEF == flag || ALL_COEF == flag) {
		for (i = 0; i < 8; i++) {
			for (j = 0; j < 4; j++)
				pCmdInfo->c_hor_coef[i][j] =
				    (short)*(cong_uvcom_hor + (i << 5) + j);
		}

		for (i = 0; i < 9; i++) {
			for (j = 0; j < chrome_ver_tap; j++)
				pCmdInfo->c_ver_down_coef[i][j] =
				    (short)*(cong_uvcom_ver + (i << 5) + j);
		}
	}
}

unsigned char Scale_GenScaleCoeff(short i_w,
				  short i_h,
				  short o_w,
				  short o_h,
				  unsigned int *coeff_h_ptr,
				  unsigned int *coeff_h_chroma_ptr,
				  unsigned int *v_ch_ptr,
				  unsigned char scaling2yuv420,
				  unsigned char *scaler_tap,
				  unsigned char *chrome_tap,
				  void *temp_buf_ptr,
				  unsigned int temp_buf_size, unsigned char dec,
				  bool bCowork, unsigned char cur_path)
{
	struct gsc_mem_pool pool = { 0 };
	struct scaler_coef_info *pScalingCoefInfo;
	int *luma_hcoef = coeff_h_ptr;
	int *chroma_hcoef = coeff_h_chroma_ptr;
	int *vcoef = v_ch_ptr;
	int i, j;
	int input_format = 0;
	int output_format = 0;
	int ylabel;
	int uvlabel;
	int hor_deci = dec;
	int ver_deci = dec;
	int y_ver_tap;
	int uv_ver_tap;
	int row_in;
	int col_in;
	int row_out;
	int col_out;
	int row_deci;
	int col_deci;
	int row_out_uv;
	int row_deci_uv;
	unsigned char tap1;
	unsigned char tap2;
	struct scaler_coef coef;
	int deci_height_y;
	int deci_height_uv;
	int src_height_uv;
	int des_height_uv;
	int tmp;
	/*int rdata;*/
	/*int wdata;*/
	int *luma_hor;
	int *chroma_hor;

	/* init pool and allocate static array */
	if (!init_pool(temp_buf_ptr, temp_buf_size, &pool))
		return FALSE;

	pScalingCoefInfo = alloc_mem(sizeof(struct scaler_coef_info), 3, &pool);
	if (pScalingCoefInfo == NULL)
		return FALSE;

	coef.row_in = i_h;
	coef.col_in = i_w;
	coef.row_out = o_h;
	coef.col_out = o_w;

	row_in = coef.row_in;
	col_in = coef.col_in;
	row_out = coef.row_out;
	col_out = coef.col_out;

	if (bCowork) {
		col_deci = 256;
		row_deci = 256;
	} else {
		col_deci = col_in >> hor_deci;
		row_deci = row_in >> ver_deci;
	}

	pr_debug("row_in %d col_in %d hor_deci %d ver_deci %d, bCowork=%d, cur_path=%d\n",
			row_in, col_in, hor_deci, ver_deci, bCowork, cur_path);
	pr_debug("row_deci, col_deci, row_out, col_out: %x, %x, %x, %x\n",
		row_deci, col_deci, row_out, col_out);

	/* yuv420 */
	if (input_format < 2 || 3 == input_format) {
		/* 420 to 420 */
		if (output_format < 2) {
			/* down scaling */
			if (2 * row_out <= row_deci) {
				dcam_gen_scaler_coef(col_deci, row_deci,
						     col_out, row_out, 0, &tap1,
						     &tap2, pScalingCoefInfo,
						     ALL_COEF, &pool);
				y_ver_tap = tap1;
				uv_ver_tap = tap2;
				ylabel = DOWN;
				uvlabel = DOWN;
			} else {
				y_ver_tap = 4;
				uv_ver_tap = 4;
				ylabel = UP;
				uvlabel = UP;
			}
		} else if (output_format == 2) {
			/* 420 to 422 */
			if (2 * row_out <= row_deci) {
				/* y channel down scaling */
				dcam_gen_scaler_coef(col_deci, row_deci,
						     col_out, row_out, 0, &tap1,
						     &tap2, pScalingCoefInfo,
						     LUMA_COEF, &pool);
				y_ver_tap = tap1;
				ylabel = DOWN;
			} else {
				/* y channel up scaling  */
				y_ver_tap = 4;
				ylabel = UP;
			}

			row_out_uv = row_out;
			row_deci_uv = row_deci >> 1;

			if (2 * row_out_uv <= row_deci_uv) {
				/* uv channel down scaling */
				dcam_gen_scaler_coef(col_deci, row_deci_uv,
						     col_out, row_out_uv, 0,
						     &tap1, &tap2,
						     pScalingCoefInfo,
						     CHROMA_COEF, &pool);
				uv_ver_tap = tap2;
				uvlabel = DOWN;
			} else {
				/* uv channel up scaling */
				uv_ver_tap = 4;
				uvlabel = UP;
			}
		}
	} else if (input_format == 2) {
		/* yuv422 */
		if (output_format < 2) {
			/* 422 to 420 */
			dcam_gen_scaler_coef(col_deci, row_deci, col_out,
					     row_out, 1, &tap1, &tap2,
					     pScalingCoefInfo, ALL_COEF, &pool);
			if (2 * row_out <= row_deci) {
				/* y channel down scaling */
				y_ver_tap = tap1;
				ylabel = DOWN;
			} else {
				/* y channel up scaling */
				y_ver_tap = 4;
				ylabel = UP;
			}
			row_out_uv = row_out >> 1;
			row_deci_uv = row_deci;

			if (2 * row_out_uv <= row_deci_uv) {
				/* uv channel down scaling */
				uv_ver_tap = tap2;
				uvlabel = DOWN;
			} else {
				/* uv channel up scaling */
				uv_ver_tap = 4;
				uvlabel = UP;
			}
		} else if (output_format == 2) {
			/* 422 to 422 */
			if (2 * row_out <= row_deci) {
				dcam_gen_scaler_coef(col_deci, row_deci,
						     col_out, row_out, 0, &tap1,
						     &tap2, pScalingCoefInfo,
						     ALL_COEF, &pool);
				y_ver_tap = tap1;
				uv_ver_tap = tap2;
				ylabel = DOWN;
				uvlabel = DOWN;
			} else {
				y_ver_tap = 4;
				uv_ver_tap = 4;
				ylabel = UP;
				uvlabel = UP;
			}
		}
	}

	luma_hor = (int *)alloc_mem(64 * sizeof(int), 2, &pool);
	chroma_hor = (int *)alloc_mem(32 * sizeof(int), 2, &pool);

	/* luma_hor */
	for (i = 0; i < 8; i++) {
		for (j = 0; j < 8; j++) {
			if (ylabel == UP)
				luma_hor[i * 8 + j] = up_scaling_8_8[i * 8 + j];
			else
				luma_hor[i * 8 + j] =
				    pScalingCoefInfo->y_hor_coef[i][j];
		}
	}

	for (i = 0; i < 32; i++)
		*(luma_hcoef + i) =
		    ((luma_hor[2 * i] & 0x000001ff) << 9) | (luma_hor[2 * i + 1]
							     & 0x000001ff);

	/* chroma_hor */
	for (i = 0; i < 8; i++)
		for (j = 0; j < 4; j++)
			if (uvlabel == UP)
				chroma_hor[i * 4 + j] =
				    up_scaling_8_4[i * 4 + j];
			else
				chroma_hor[i * 4 + j] =
				    pScalingCoefInfo->c_hor_coef[i][j];

	for (i = 0; i < 16; i++)
		*(chroma_hcoef + i) =
		    ((chroma_hor[2 * i] & 0x000001ff) << 9) |
		    (chroma_hor[2 * i + 1] & 0x000001ff);

	/* vcoef */
	if (bCowork && (cur_path == 3)) {
		deci_height_y = 256;
		if (input_format < 2 || 3 == input_format)
			src_height_uv = deci_height_y >> 1;
		else
			src_height_uv = deci_height_y;

		deci_height_uv = src_height_uv;
	} else {
		deci_height_y = coef.row_in >> ver_deci;
		if (input_format < 2 || 3 == input_format)
			src_height_uv = coef.row_in >> 1;
		else if (input_format == 2)
			src_height_uv = coef.row_in;

		deci_height_uv = src_height_uv >> ver_deci;
	}
	pr_debug("deci_height_y=%d,deci_height_uv=%d\n",
		deci_height_y, deci_height_uv);

	if (output_format < 2)
		des_height_uv = coef.row_out >> 1;
	else if (output_format == 2)
		des_height_uv = coef.row_out;

	if ((2 * coef.row_out) <= deci_height_y) {
		/* vcoef -132 elements */
		for (i = 0; i < 9; i++) {
			for (j = 0; j < y_ver_tap; j++) {
				if (i < 8) {
					vcoef[i * 16 + j] =
					    pScalingCoefInfo->
					    y_ver_down_coef[i][j];
				} else {
					if (j < 4)
						vcoef[i * 16 + j] =
						    pScalingCoefInfo->
						    y_ver_down_coef[i][j];
					else
						break;
				}
			}
		}
	} else {
		for (i = 0; i < 32; i++)
			vcoef[i] = up_scaling_8_4[i];
	}

	if (des_height_uv * 2 <= deci_height_uv) {
		for (i = 0; i < 9; i++) {
			for (j = 0; j < uv_ver_tap; j++) {
				tmp = pScalingCoefInfo->c_ver_down_coef[i][j];
				if (i < 8) {
					vcoef[i * 16 + j] =
					    ((tmp & 0x000001ff) << 9) |
					    (vcoef[i * 16 + j] & 0x000001ff);
				} else {
					if (j < 4)
						vcoef[i * 16 + j] =
						    ((tmp & 0x000001ff) << 9) |
						    (vcoef[i * 16 + j] &
						     0x000001ff);
					else
						break;
				}
			}
		}
	} else {
		for (i = 0; i < 32; i++) {
			tmp = up_scaling_8_4[i];
			vcoef[i] =
			    ((tmp & 0x000001ff) << 9) | (vcoef[i] & 0x000001ff);
		}
	}

	*scaler_tap = y_ver_tap;
	*chrome_tap = uv_ver_tap;

	return TRUE;
}

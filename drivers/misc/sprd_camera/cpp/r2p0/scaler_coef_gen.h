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
#ifndef SCALER_COEF_GEN_
#define SCALER_COEF_GEN_

/*#include <linux/types.h>*/

#define MAX_TAP                  16
#define LUMA_COEF                1
#define CHROMA_COEF              2
#define ALL_COEF                 3
#define LUMA                     1

#define SCALER_COEF_TAB_LEN_HOR  48
#define SCALER_COEF_TAB_LEN_VER  68

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
				  unsigned int temp_buf_size,
				  unsigned char dec,
				  bool bCowork,
				  unsigned char cur_path);
#endif

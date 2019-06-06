/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
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

#ifndef _GSP_R3P0_COEF_GENERATE_H_
#define _GSP_R3P0_COEF_GENERATE_H_

#include "gsp_r3p0_core.h"
#include "sin_cos.h"
#include "../gsp_debug.h"

#define SIGN2(input, p)	{if (p >= 0) input = 1; if (p < 0) input = -1; }
#define COUNT			256
#define COEF_ARR_ROWS		   9
/* #define COEF_ARR_COLUMNS		8 */
#define COEF_ARR_COL_MAX		32
#define GSC_ABS(_a)			 ((_a) < 0 ? -(_a) : (_a))
/* #define pi						3.14159265357 */
#define GSC_FIX					24
#define MAX(_x, _y)		   (((_x) > (_y)) ? (_x) : (_y))

#define LIST_SET_ENTRY_KEY(pEntry, i_w, i_h, o_w, o_h, h_t, v_t)\
{\
	pEntry->in_w = i_w;\
	pEntry->in_h = i_h;\
	pEntry->out_w = o_w;\
	pEntry->out_h = o_h;\
	pEntry->hor_tap = h_t;\
	pEntry->ver_tap = v_t;\
}

struct GSC_MEM_POOL {
	ulong begin_addr;
	ulong total_size;
	ulong used_size;
};

uint32_t *gsp_r3p0_gen_block_scaler_coef(struct gsp_r3p0_core *core,
						 uint32_t i_w,
						 uint32_t i_h,
						 uint32_t o_w,
						 uint32_t o_h,
						 uint32_t hor_tap,
						 uint32_t ver_tap);

#endif

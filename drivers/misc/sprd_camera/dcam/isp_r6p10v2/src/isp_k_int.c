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

unsigned int isp_k_int_status(enum isp_id idx)
{
	unsigned int val = 0;

	val = ISP_REG_RD(idx, ISP_INT_STATUS);

	return val;
}

int isp_k_int_en0(enum isp_id idx, unsigned int val)
{
	ISP_REG_WR(idx, ISP_INT_EN0, val);

	return 0;
}

unsigned int isp_k_int_get_en0_status(enum isp_id idx)
{
	unsigned int val = 0;

	val = ISP_REG_RD(idx, ISP_INT_EN0);

	return val;
}

int isp_k_int_clr0(enum isp_id idx, unsigned int val)
{
	ISP_REG_WR(idx, ISP_INT_CLR0, val);

	return 0;
}

unsigned int isp_k_int_int0(enum isp_id idx)
{
	unsigned int val = 0;

	val = ISP_REG_RD(idx, ISP_INT_INT0);

	return val;
}

int isp_K_int_raw0(enum isp_id idx)
{
	unsigned int val = 0;

	val = ISP_REG_RD(idx, ISP_INT_RAW0);

	return val;
}

int isp_k_int_en1(enum isp_id idx, unsigned int val)
{
	ISP_REG_WR(idx, ISP_INT_EN1, val);

	return 0;
}

unsigned int isp_k_int_get_en1_status(enum isp_id idx)
{
	unsigned int val = 0;

	val = ISP_REG_RD(idx, ISP_INT_EN1);

	return val;
}

int isp_k_int_clr1(enum isp_id idx, unsigned int val)
{
	ISP_REG_WR(idx, ISP_INT_CLR1, val);

	return 0;
}

int isp_K_int_raw1(enum isp_id idx)
{
	unsigned int val = 0;

	val = ISP_REG_RD(idx, ISP_INT_RAW1);

	return val;
}

unsigned int isp_k_int_int1(enum isp_id idx)
{
	unsigned int val = 0;

	val = ISP_REG_RD(idx, ISP_INT_INT1);

	return val;
}

int isp_k_int_en2(enum isp_id idx, unsigned int val)
{
	ISP_REG_WR(idx, ISP_INT_EN2, val);

	return 0;
}

unsigned int isp_k_int_get_en2_status(enum isp_id idx)
{
	unsigned int val = 0;

	val = ISP_REG_RD(idx, ISP_INT_EN2);

	return val;
}

int isp_k_int_clr2(enum isp_id idx, unsigned int val)
{
	ISP_REG_WR(idx, ISP_INT_CLR2, val);

	return 0;
}

int isp_k_int_raw2(enum isp_id idx)
{
	unsigned int val = 0;

	val = ISP_REG_RD(idx, ISP_INT_RAW2);

	return val;
}

unsigned int isp_k_int_int2(enum isp_id idx)
{
	unsigned int val = 0;

	val = ISP_REG_RD(idx, ISP_INT_INT2);

	return val;
}

int isp_k_int_en3(enum isp_id idx, unsigned int val)
{
	ISP_REG_WR(idx, ISP_INT_EN3, val);

	return 0;
}

unsigned int isp_k_int_get_en3_status(enum isp_id idx)
{
	unsigned int val = 0;

	val = ISP_REG_RD(idx, ISP_INT_EN3);

	return val;
}

int isp_k_int_clr3(enum isp_id idx, unsigned int val)
{
	ISP_REG_WR(idx, ISP_INT_CLR3, val);

	return 0;
}

int isp_K_int_raw3(enum isp_id idx)
{
	unsigned int val = 0;

	val = ISP_REG_RD(idx, ISP_INT_RAW3);

	return val;
}

unsigned int isp_k_int_int3(enum isp_id idx)
{
	unsigned int val = 0;

	val = ISP_REG_RD(idx, ISP_INT_INT3);

	return val;
}

int isp_k_int_all_done_ctrl(enum isp_id idx, unsigned int val)
{
	ISP_REG_WR(idx, ISP_INT_DONE_CTRL, val);

	return 0;
}

int isp_k_int_all_done_src_ctrl(enum isp_id idx, unsigned int val)
{
	ISP_REG_WR(idx, ISP_ALL_DONE_SRC_CTRL, val);

	return 0;
}

int isp_k_int_skip_ctrl_en(enum isp_id idx, unsigned int val)
{
	if (val)
		ISP_REG_OWR(idx, ISP_INT_SKIP_CTRL, BIT_0 | BIT_16);
	else
		ISP_REG_MWR(idx, ISP_INT_SKIP_CTRL, BIT_0 | BIT_16, 0);

	return 0;
}

int isp_k_int_skip_ctrl_clr(enum isp_id idx, unsigned int val)
{
	if (val)
		ISP_REG_OWR(idx, ISP_INT_SKIP_CTRL, BIT_1 | BIT_17);
	else
		ISP_REG_MWR(idx, ISP_INT_SKIP_CTRL, BIT_1 | BIT_17, 0);

	return 0;
}

int isp_k_int_skip_ctrl_param(enum isp_id idx, struct skip_ctrl_param skip_ctrl)
{
	unsigned val = 0;

	val = (((skip_ctrl.sdw_done_int_cnt_num & 0xF) << 24)
			| ((skip_ctrl.sdw_down_skip_cnt_num & 0xF) << 20)
			| ((skip_ctrl.all_done_int_cnt_num & 0xF) << 8)
			| ((skip_ctrl.all_done_skip_cnt_num) << 4));

	ISP_REG_MWR(idx, ISP_INT_SKIP_CTRL, 0xFF00FF0, val);

	return 0;
}

int isp_k_int_skip_ctrl1_en(enum isp_id idx, unsigned int val)
{
	if (val)
		ISP_REG_OWR(idx, ISP_INT_SKIP_CTRL1, BIT_0);
	else
		ISP_REG_MWR(idx, ISP_INT_SKIP_CTRL1, BIT_0, 0);

	return 0;
}

int isp_k_int_skip_ctrl1_clr(enum isp_id idx, unsigned int val)
{
	if (val)
		ISP_REG_OWR(idx, ISP_INT_SKIP_CTRL1, BIT_1);
	else
		ISP_REG_MWR(idx, ISP_INT_SKIP_CTRL1, BIT_1, 0);

	return 0;
}

int isp_k_int_skip_ctrl1_param(enum isp_id idx,
			struct skip_ctrl1_param skip_ctrl1)
{
	unsigned int val = 0;

	val = ((skip_ctrl1.vid_done_int_cnt_num & 0xF) << 8)
			| ((skip_ctrl1.vid_done_skip_cnt_num & 0xF) << 4);

	ISP_REG_MWR(idx, ISP_INT_SKIP_CTRL1, 0xFF0, val);

	return 0;
}

























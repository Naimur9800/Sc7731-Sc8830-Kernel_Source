/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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
#include <video/sprd_isp_r6p10.h>
#include "../isp_drv.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "1D LSC: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

#define	ISP_1D_LSC_BUF0				  0
#define	ISP_1D_LSC_BUF1				  1
#define ISP_PINGPANG_1D_LSC_NUM		   256

static int isp_k_1d_lsc_block(struct isp_io_param *param,
			struct isp_k_block *isp_k_param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dev_1d_lsc_info lnc_info;
	unsigned long dst_addr = 0;
	void *data_ptr = NULL;
	unsigned short *w_buff = NULL;

	memset(&lnc_info, 0x00, sizeof(lnc_info));
	ret = copy_from_user((void *)&lnc_info, param->property_param,
			sizeof(lnc_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -EPERM;
	}

	ISP_REG_MWR(idx, ISP_1D_LNC_CTRL, BIT_0, lnc_info.bypass);
	if (lnc_info.bypass)
		return 0;

	val = (lnc_info.radius_step & 0x7) << 0x1;
	ISP_REG_MWR(idx, ISP_1D_LNC_CTRL, (0x7 << 0x1), val);

	val = ((lnc_info.center_r0c0_col_x & 0x1fff) << 12) |
			(lnc_info.center_r0c0_row_y & 0xfff);
	ISP_REG_WR(idx, ISP_1D_LNC_CFG0, val);

	val = ((lnc_info.center_r0c1_col_x & 0x1fff) << 12) |
			(lnc_info.center_r0c1_row_y & 0xfff);
	ISP_REG_WR(idx, ISP_1D_LNC_CFG1, val);

	val = ((lnc_info.center_r1c0_col_x & 0x1fff) << 12) |
			(lnc_info.center_r1c0_row_y & 0xfff);
	ISP_REG_WR(idx, ISP_1D_LNC_CFG2, val);

	val = ((lnc_info.center_r1c1_col_x & 0x1fff) << 12) |
			(lnc_info.center_r1c1_row_y & 0xfff);
	ISP_REG_WR(idx, ISP_1D_LNC_CFG3, val);
	ISP_REG_WR(idx, ISP_1D_LNC_CFG4, 0);

	val = ((lnc_info.init_r_r0c1 & 0x1fff) << 13) |
			(lnc_info.init_r_r0c0 & 0x1fff);
	ISP_REG_WR(idx, ISP_1D_LNC_R_CFG0, val);

	val = ((lnc_info.init_r_r1c1 & 0x1fff) << 13) |
			(lnc_info.init_r_r1c0 & 0x1fff);
	ISP_REG_WR(idx, ISP_1D_LNC_R_CFG1, val);

	ISP_REG_WR(idx, ISP_1D_LNC_R2_CFG0,
			(lnc_info.init_r2_r0c0 & 0x3ffffff));
	ISP_REG_WR(idx, ISP_1D_LNC_DR2_CFG0,
			(lnc_info.init_dr2_r0c0 & 0x3ffffff));
	ISP_REG_WR(idx, ISP_1D_LNC_R2_CFG1,
			(lnc_info.init_r2_r0c1 & 0x3ffffff));
	ISP_REG_WR(idx, ISP_1D_LNC_DR2_CFG1,
			(lnc_info.init_dr2_r0c1 & 0x3ffffff));
	ISP_REG_WR(idx, ISP_1D_LNC_R2_CFG2,
			(lnc_info.init_r2_r1c0 & 0x3ffffff));
	ISP_REG_WR(idx, ISP_1D_LNC_DR2_CFG2,
			(lnc_info.init_dr2_r1c0 & 0x3ffffff));
	ISP_REG_WR(idx, ISP_1D_LNC_R2_CFG3,
			(lnc_info.init_r2_r1c1 & 0x3ffffff));
	ISP_REG_WR(idx, ISP_1D_LNC_DR2_CFG3,
			(lnc_info.init_dr2_r1c1 & 0x3ffffff));

#ifdef CONFIG_64BIT
	data_ptr = (void *)(((unsigned long)lnc_info.data_ptr[1] << 32)
					| lnc_info.data_ptr[0]);
#else
	data_ptr = (void *)(lnc_info.data_ptr[0]);
#endif

	if (isp_k_param->lsc_1d_buf_id == ISP_1D_LSC_BUF1) {
		dst_addr = ISP_BASE_ADDR(idx) + ISP_1D_LENS_GR_BUF0_CH0;
		isp_k_param->lsc_1d_buf_id = ISP_1D_LSC_BUF0;
	} else {
		dst_addr = ISP_BASE_ADDR(idx) + ISP_1D_LENS_GR_BUF0_CH0;
		isp_k_param->lsc_1d_buf_id = ISP_1D_LSC_BUF1;
	}

	w_buff = vzalloc(ISP_PINGPANG_1D_LSC_NUM * 4 * sizeof(unsigned int));
	if (w_buff == NULL)
		return -ENOMEM;

	ret = copy_from_user((void *)w_buff,
			(const void __user *)data_ptr,
			ISP_PINGPANG_1D_LSC_NUM * 4 * sizeof(unsigned int));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		vfree(w_buff);
		return -1;
	}

	*(unsigned int *)dst_addr = (unsigned int)(*w_buff);
	vfree(w_buff);

	ISP_REG_MWR(idx, ISP_1D_LNC_CTRL, BIT_4,
			(isp_k_param->lsc_1d_buf_id) << 4);

	return ret;
}

static int isp_k_1d_lsc_pos(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct img_offset pos;

	ret = copy_from_user((void *)&pos, param->property_param,
			sizeof(struct img_offset));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -EPERM;
	}

	val = ((pos.x & 0xffff) << 16) | (pos.y & 0xffff);

	ISP_REG_WR(idx, ISP_1D_LNC_CFG4, val);

	return ret;
}

int isp_k_cfg_1d_lsc(struct isp_io_param *param,
			struct isp_k_block *isp_k_param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("fail to get param\n");
		return -EPERM;
	}

	if (param->property_param == NULL) {
		pr_err("fail to get property_param\n");
		return -EPERM;
	}

	switch (param->property) {
	case ISP_PRO_1D_LSC_BLOCK:
		ret = isp_k_1d_lsc_block(param, isp_k_param, idx);
		break;
	case ISP_PRO_1D_LSC_SLICE_SIZE:
		break;
	case ISP_PRO_1D_LSC_POS:
		ret = isp_k_1d_lsc_pos(param, idx);
		break;
	default:
		pr_err("fail to support cmd id = %d\n",
			param->property);
		break;
	}

	return ret;
}



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
#include <video/sprd_isp_r6p10v2.h>
#include "../isp_drv.h"

static int isp_k_binning_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dev_binning4awb_info binning_info;
	unsigned int binning_h = 0;
	unsigned int binning_w = 0;

	memset(&binning_info, 0x00, sizeof(binning_info));
	ret = copy_from_user((void *)&binning_info, param->property_param,
			sizeof(binning_info));
	if (ret != 0) {
		pr_err("binning_block: copy error, ret=0x%x\n",
						(unsigned int)ret);
		return -EPERM;
	}

	binning_info.bin_mode_sel = 1;
	binning_info.bin_skip_num = 0;
	binning_info.mem_fifo_clr = 0;
	binning_h = (binning_info.img_size.height >> binning_info.vx) & ~0x1;
	binning_w = (binning_info.img_size.width >> binning_info.hx) & ~0x1;
	binning_info.ddr_wr_num = (binning_h * binning_w + 5) / 6;
	binning_info.skip_num_clr = 1;

	ISP_REG_MWR(idx, ISP_BINNING_PARAM, BIT_0, binning_info.bypass);
	if (binning_info.bypass) {
		ISP_REG_MWR(idx, ISP_BINNING_CFG_READY, BIT_0, 0x1);
		return 0;
	}

	val = (binning_info.bin_mode_sel & 0x1) << 1;
	ISP_REG_MWR(idx, ISP_BINNING_PARAM, BIT_1, val);

	val = ((binning_info.hx & 0x7) << 4) |
			((binning_info.vx & 0x7) << 8);
	ISP_REG_MWR(idx, ISP_BINNING_PARAM, 0x770, val);

	ISP_REG_MWR(idx, ISP_BINNING_PARAM, 0x80,
		(binning_info.mem_fifo_clr & 0x1) << 7);
	ISP_REG_MWR(idx, ISP_BINNING_PARAM, 0xF000,
		(binning_info.bin_skip_num & 0xF) << 12);

	/*ISP_REG_WR(idx, ISP_BINNING_MEM_ADDR, binning_info.addr_ddr_init);*/
	ISP_REG_WR(idx, ISP_BINNING_DDR_WR_NUM, binning_info.ddr_wr_num);
	val = (binning_info.skip_num_clr & 0x1);
	ISP_REG_MWR(idx, ISP_BINNING_SKIP_NUM_CLR, BIT_0, val);

	ISP_REG_MWR(idx, ISP_BINNING_CFG_READY, BIT_0, 0x1);

	return ret;
}

static int isp_k_binging_bypass(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int bypass = 0;

	ret = copy_from_user((void *)&bypass, param->property_param,
			sizeof(unsigned int));
	if (ret != 0) {
		pr_err("binging: copy error, ret = 0x%x\n", (unsigned int)ret);
		return -EPERM;
	}

	ISP_REG_MWR(idx, ISP_BINNING_PARAM, BIT_0, bypass);

	return ret;
}

static int isp_k_binning_addr(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int addr = 0;

	ret = copy_from_user((void *)&addr,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_binging_mem_addr:error,ret = 0x%x\n",
				(unsigned int)ret);
		return -1;
	}

	ISP_REG_WR(idx, ISP_BINNING_MEM_ADDR, addr);
	ISP_REG_WR(idx, ISP_BINNING_CFG_READY, 1);

	return ret;
}


static int isp_k_binning_scaling_ratio
		(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_scaling_ratio scaling_ratio;

	ret = copy_from_user((void *)&scaling_ratio, param->property_param,
			sizeof(struct isp_scaling_ratio));
	if (ret != 0) {
		pr_err("binning: copy error, ret = 0x%x\n", (unsigned int)ret);
		return -EPERM;
	}

	val = ((scaling_ratio.vertical & 0x7) << 8)|
		((scaling_ratio.horizontal & 0x7) << 4);
	ISP_REG_MWR(idx, ISP_BINNING_PARAM, 0x770, val);

	return ret;
}

static int isp_k_binning_get_scaling_ratio
		(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_scaling_ratio scaling_ratio;

	val = ISP_REG_RD(idx, ISP_BINNING_PARAM);

	scaling_ratio.vertical = (val >> 8) & 0x7;
	scaling_ratio.horizontal = (val >> 4) & 0x7;

	ret = copy_to_user(param->property_param, (void *)&scaling_ratio,
			sizeof(struct isp_scaling_ratio));
	if (ret != 0) {
		ret = -EPERM;
		pr_err("binning_ratio: copy error, ret=0x%x\n",
						(unsigned int)ret);
	}

	return ret;
}

int isp_k_cfg_binning(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_binning: param is null error.\n");
		return -EPERM;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_binning: property_param is null error.\n");
		return -EPERM;
	}

	switch (param->property) {
	case ISP_PRO_BINNING_BLOCK:
		ret = isp_k_binning_block(param, idx);
		break;
	case ISP_PRO_BINNING_BYPASS:
		ret = isp_k_binging_bypass(param, idx);
		break;
	case ISP_PRO_BINNING_SCALING_RATIO:
		ret = isp_k_binning_scaling_ratio(param, idx);
		break;
	case ISP_PRO_BINNING_GET_SCALING_RATIO:
		ret = isp_k_binning_get_scaling_ratio(param, idx);
		break;
	case ISP_PRO_BINNING_MEM_ADDR:
		ret = isp_k_binning_addr(param, idx);
		break;
	default:
		pr_err("binning:cmd id:%d, not supported.\n", param->property);
		break;
	}

	return ret;
}

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

static int isp_k_hist_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	struct isp_dev_hist_info hist_info;

	memset(&hist_info, 0x00, sizeof(hist_info));
	ret = copy_from_user((void *)&hist_info,
		param->property_param, sizeof(hist_info));
	if (ret != 0) {
		pr_err("isp_k_hist_block: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_HIST_PARAM, BIT_0, hist_info.bypass);
	ISP_REG_MWR(idx, ISP_HIST_PARAM + ISP_CH1_ADDR_OFFSET, BIT_0,
		hist_info.bypass);
	if (hist_info.bypass) {
		ISP_REG_MWR(idx, ISP_HIST_CFG_READY, BIT_0, 1);
		ISP_REG_MWR(idx, ISP_HIST_CFG_READY + ISP_CH1_ADDR_OFFSET,
			BIT_0, 1);
		return 0;
	}
	ISP_REG_MWR(idx, ISP_HIST_PARAM, BIT_1, hist_info.mode << 1);
	ISP_REG_MWR(idx, ISP_HIST_PARAM + ISP_CH1_ADDR_OFFSET, BIT_1,
		hist_info.mode << 1);
	ISP_REG_MWR(idx, ISP_HIST_PARAM, 0xF0,  hist_info.skip_num << 4);
	ISP_REG_MWR(idx, ISP_HIST_PARAM + ISP_CH1_ADDR_OFFSET, 0xF0,
		hist_info.skip_num << 4);

	ISP_REG_MWR(idx, ISP_HIST_SKIP_NUM_CLR, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_HIST_SKIP_NUM_CLR + ISP_CH1_ADDR_OFFSET, BIT_0, 1);

	ISP_REG_MWR(idx, ISP_HIST_CFG_READY, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_HIST_CFG_READY + ISP_CH1_ADDR_OFFSET, BIT_0, 1);

	return ret;
}

static int isp_k_hist_bypass(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int bypass = 0;

	ret = copy_from_user((void *)&bypass,
		param->property_param, sizeof(bypass));
	if (ret != 0) {
		pr_err("isp_k_hist_bypass: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (bypass)
		ISP_REG_OWR(idx, ISP_HIST_PARAM, BIT_0);
	else
		ISP_REG_MWR(idx, ISP_HIST_PARAM, BIT_0, 0);
	ISP_REG_MWR(idx, ISP_HIST_CFG_READY, BIT_0, 1);

	return ret;
}

static int isp_k_hist_skip_num(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_hist_skip_num: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_HIST_PARAM, 0xF0, val << 4);
	ISP_REG_MWR(idx, ISP_HIST_CFG_READY, BIT_0, 1);

	return ret;
}

static int isp_k_hist_skip_num_clr
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_hist_skip_num_clr: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_WR(idx, ISP_HIST_SKIP_NUM_CLR, val);
	ISP_REG_MWR(idx, ISP_HIST_CFG_READY, BIT_0, 1);

	return ret;
}

static int isp_k_hist_cfg_rdy(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_hist_cfg_rdy: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_HIST_CFG_READY, BIT_0, val);

	return ret;
}

static int isp_k_hist_mode(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int mode = 0;

	ret = copy_from_user((void *)&mode,
		param->property_param, sizeof(mode));
	if (ret != 0) {
		pr_err("isp_k_hist_mode: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (mode)
		ISP_REG_OWR(idx, ISP_HIST_PARAM, BIT_1);
	else
		ISP_REG_MWR(idx, ISP_HIST_PARAM, BIT_1, 0);
	ISP_REG_MWR(idx, ISP_HIST_CFG_READY, BIT_0, 1);

	return ret;
}

int isp_k_cfg_hist(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_hist: param is null error.\n");
		return -1;
	}
	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_hist: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_HIST_BLOCK:
		ret = isp_k_hist_block(param, idx);
		break;
	case ISP_PRO_HIST_BYPASS:
		ret = isp_k_hist_bypass(param, idx);
		break;
	case ISP_PRO_HIST_MODE:
		ret = isp_k_hist_mode(param, idx);
		break;
	case ISP_PRO_HIST_SKIP_NUM:
		ret = isp_k_hist_skip_num(param, idx);
		break;
	case ISP_PRO_HIST_SKIP_NUM_CLR:
		ret = isp_k_hist_skip_num_clr(param, idx);
		break;
	case ISP_PRO_HIST_CFG_RDY:
		ret = isp_k_hist_cfg_rdy(param, idx);
		break;
	default:
		pr_err("isp_k_cfg_hist:fail id:%d,not supported.\n",
			param->property);
		break;
	}

	return ret;
}

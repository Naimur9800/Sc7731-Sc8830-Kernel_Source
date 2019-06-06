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
#include <asm/cacheflush.h>
#include <video/sprd_mm.h>
#include <video/sprd_isp_r6p10v2.h>
#include "../isp_drv.h"

static int isp_k_afl_bypass(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int bypass = 0;

	ret = copy_from_user((void *)&bypass, param->property_param,
			sizeof(bypass));
	if (ret != 0) {
		pr_err("afl:copy_user error, ret = 0x%x\n", (unsigned int)ret);
		return -1;
	}

	if (bypass)
		ISP_REG_OWR(idx, ISP_ANTI_FLICKER_PARAM0, BIT_0);
	else
		ISP_REG_MWR(idx, ISP_ANTI_FLICKER_PARAM0, BIT_0, 0);

	ISP_REG_MWR(idx, ISP_ANTI_FLICKER_CFG_READY, BIT_0, 1);

	return ret;
}

static unsigned int filcker_get_total_num
		(struct isp_dev_anti_flicker_info afl_info)
{
	unsigned int data_long = 0;
	unsigned int data_long_frame[255] = {0};
	unsigned int input_long = afl_info.vheight * afl_info.skip_frame_num;
	unsigned int step = 1 << afl_info.line_step;
	unsigned int i = 0;
	unsigned int ii = 0;

	for (ii = 0; ii < input_long; ii++) {
		if ((ii%afl_info.vheight) >= afl_info.img_size.height)
			continue;
		if (ii%step == 0)
			data_long_frame[ii/afl_info.vheight] =
					data_long_frame[ii/afl_info.vheight]+1;
	}

	for (i = 0; i < afl_info.skip_frame_num; i++) {
		if (data_long_frame[i]%2 == 0)
			data_long = data_long + data_long_frame[i]/2;
		else
			data_long = data_long + data_long_frame[i]/2 + 1;
	}

	return data_long;
}

static int isp_k_anti_flicker_block(struct isp_io_param *param, enum isp_id idx)
{
	int32_t ret = 0;
	struct isp_dev_anti_flicker_info afl_info;

	memset(&afl_info, 0x00, sizeof(afl_info));

	ret = copy_from_user((void *)&afl_info, param->property_param,
			sizeof(afl_info));
	if (ret != 0) {
		pr_err("anti_flicker: copy error, ret=0x%x\n",
						(unsigned int)ret);
		return -EPERM;
	}


	ISP_REG_MWR(idx, ISP_ANTI_FLICKER_PARAM0, BIT_0, afl_info.bypass);
	if (afl_info.bypass) {
		ISP_REG_MWR(idx, ISP_ANTI_FLICKER_CFG_READY, BIT_0, 1);
		return 0;
	}
	/* AFL new or old */
	ISP_REG_MWR(idx, ISP_COMMON_3A_CTRL0, BIT_1, 0 << 1);

	ISP_REG_MWR(idx, ISP_ANTI_FLICKER_PARAM0, BIT_1, afl_info.mode << 1);
	ISP_REG_MWR(idx, ISP_ANTI_FLICKER_PARAM0,
			0xF0,  afl_info.skip_frame_num << 4);

	ISP_REG_MWR(idx, ISP_ANTI_FLICKER_PARAM1, 0xF, afl_info.line_step);
	ISP_REG_MWR(idx, ISP_ANTI_FLICKER_PARAM1,
			0xFF00, afl_info.frame_num << 8);
	ISP_REG_MWR(idx, ISP_ANTI_FLICKER_PARAM1,
			0xFFFF0000, afl_info.vheight << 16);

	ISP_REG_MWR(idx, ISP_ANTI_FLICKER_COL_POS,
				0xFFFF, afl_info.start_col);
	ISP_REG_MWR(idx, ISP_ANTI_FLICKER_COL_POS,
			0xFFFF0000, afl_info.end_col << 16);

	ISP_REG_MWR(idx, ISP_ANTI_FLICKER_SKIP_NUM_CLR, BIT_0, 1);


	afl_info.afl_total_num = filcker_get_total_num(afl_info);
	ISP_REG_MWR(idx, ISP_ANTI_FLICKER_SUM,
			0xFFFFF, afl_info.afl_total_num);

	ISP_REG_MWR(idx, ISP_ANTI_FLICKER_CFG_READY, BIT_0, 1);

	return ret;
}

int isp_k_anti_flicker_cfg_addr(unsigned long phy_addr, enum isp_id idx)
{
	int ret = 0;

	if (phy_addr == 0) {
		pr_err("isp_k_ra_aem_cfg_addr: Invalid address\n");
		return 1;
	}

	ISP_REG_WR(idx, ISP_ANTI_FLICKER_DDR_INIT_ADDR, phy_addr);
	ISP_REG_MWR(idx, ISP_ANTI_FLICKER_CFG_READY, BIT_0, 1);

	return ret;
}



int isp_k_cfg_anti_flicker(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_anti_flicker: param is null error.\n");
		return -EPERM;
	}

	if (param->property_param == NULL) {
		pr_err("anti_flicker: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_ANTI_FLICKER_BLOCK:
		ret = isp_k_anti_flicker_block(param, idx);
		break;
	case ISP_PRO_ANTI_FLICKER_BYPASS:
		ret = isp_k_afl_bypass(param, idx);
		break;
	default:
		pr_err("fail cmd id:%d, not supported.\n", param->property);
		break;
	}

	return ret;
}


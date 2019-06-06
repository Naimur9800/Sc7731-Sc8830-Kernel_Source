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

static int isp_k_posterize_block
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	unsigned int i = 0;
	struct isp_dev_posterize_info pstrz_info;

	memset(&pstrz_info, 0x00, sizeof(pstrz_info));

	ret = copy_from_user((void *)&pstrz_info,
		param->property_param, sizeof(pstrz_info));
	if (ret != 0) {
		pr_err("isp_k_posterize_block: copy error, ret=0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_PSTRZ_PARAM, BIT_0, pstrz_info.bypass);
	if (pstrz_info.bypass)
		return 0;

	for (i = 0; i < POSTERIZE_NUM; i++) {
		val = (pstrz_info.posterize_level_out[i] & 0xFF) |
			  ((pstrz_info.posterize_level_top[i] &	0xFF) << 8) |
			  ((pstrz_info.posterize_level_bottom[i] & 0xFF) << 16);
		ISP_REG_WR(idx, ISP_PSTRZ_LEVEL0 + i * 4, val);
	}

	return ret;

}

static int isp_k_pstrz_bypass(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int bypass = 0;

	ret = copy_from_user((void *)&bypass,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_pstrz_bypass: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (bypass)
		ISP_REG_OWR(idx, ISP_PSTRZ_PARAM, BIT_0);
	else
		ISP_REG_MWR(idx, ISP_PSTRZ_PARAM, BIT_0, 0);

	return ret;
}

static int isp_k_pstrz_chk_sum_clr
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;

	ret = copy_from_user((void *)&val,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_pstrz_chk_sum_clr: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (val)
		ISP_REG_OWR(idx, ISP_PSTRZ_PARAM, BIT_1);
	else
		ISP_REG_MWR(idx, ISP_PSTRZ_PARAM, BIT_1, 0);

	return ret;
}

static int isp_k_pstrz_level(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	unsigned int i = 0;
	struct pstrz_level level;

	ret = copy_from_user((void *)&level,
		param->property_param, sizeof(level));
	if (ret != 0) {
		pr_err("isp_k_pstrz_level: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	for (i = 0; i < 8; i++) {
		val = (level.posterize_level_out[i] & 0xFF)
			| ((level.posterize_level_top[i] & 0xFF) << 8)
			| ((level.posterize_level_bottom[i] & 0xFF) << 16);
		ISP_REG_WR(idx, ISP_PSTRZ_LEVEL0 + i * 4, val);
	}

	return ret;
}

int isp_k_cfg_pstrz(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_pstrz: param is null error.\n");
		return -1;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_pstrz: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_POSTERIZE_BLOCK:
		ret = isp_k_posterize_block(param, idx);
		break;
	case ISP_PRO_PSTRZ_BYPASS:
		ret = isp_k_pstrz_bypass(param, idx);
		break;
	case ISP_PRO_PSTRZ_LEVEL:
		ret = isp_k_pstrz_level(param, idx);
		break;
	case ISP_PRO_PSTRZ_CHK_SUM_CLR:
		ret = isp_k_pstrz_chk_sum_clr(param, idx);
		break;
	default:
		pr_err("isp_k_cfg_pstrz: fail cmd id:%d, not supported.\n",
			param->property);
		break;
	}

	return ret;
}

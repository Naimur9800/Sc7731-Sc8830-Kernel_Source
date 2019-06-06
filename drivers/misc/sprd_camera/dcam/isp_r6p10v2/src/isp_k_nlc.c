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

static int isp_k_nlc_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	unsigned int i = 0;
	struct isp_dev_nlc_info nlc_info;

	memset(&nlc_info, 0x00, sizeof(nlc_info));

	ret = copy_from_user((void *)&nlc_info,
		param->property_param, sizeof(nlc_info));
	if (ret != 0) {
		pr_err("isp_k_nlc_block: copy error, ret=0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_NLC_PARA, BIT_0, nlc_info.bypass);
	if (nlc_info.bypass)
		return 0;

	for (i = 0; i < 9; i++) {
		val = ((nlc_info.node.r_node[i*3] & 0x3FF) << 20)
			| ((nlc_info.node.r_node[i*3+1] & 0x3FF) << 10)
			| (nlc_info.node.r_node[i*3+2] & 0x3FF);
		ISP_REG_WR(idx, ISP_NLC_PARA_R0 + i * 4, val);

		val = ((nlc_info.node.g_node[i*3] & 0x3FF) << 20)
			| ((nlc_info.node.g_node[i*3+1] & 0x3FF) << 10)
			| (nlc_info.node.g_node[i*3+2] & 0x3FF);
		ISP_REG_WR(idx, ISP_NLC_PARA_G0 + i * 4, val);

		val = ((nlc_info.node.b_node[i*3] & 0x3FF) << 20)
			| ((nlc_info.node.b_node[i*3+1] & 0x3FF) << 10)
			| (nlc_info.node.b_node[i*3+2] & 0x3FF);
		ISP_REG_WR(idx, ISP_NLC_PARA_B0 + i * 4, val);

		val = ((nlc_info.node.l_node[i*3] & 0x3FF) << 20)
			| ((nlc_info.node.l_node[i*3+1] & 0x3FF) << 10)
			| (nlc_info.node.l_node[i*3+2] & 0x3FF);
		ISP_REG_WR(idx, ISP_NLC_PARA_L0 + i * 4, val);
	}

	val = ((nlc_info.node.r_node[27] & 0x3FF) << 20)
		| ((nlc_info.node.r_node[28] & 0x3FF) << 10);
	ISP_REG_WR(idx, ISP_NLC_PARA_R9, val);

	val = ((nlc_info.node.g_node[27] & 0x3FF) << 20)
		| ((nlc_info.node.g_node[28] & 0x3FF) << 10);
	ISP_REG_WR(idx, ISP_NLC_PARA_G9, val);

	val = ((nlc_info.node.b_node[27] & 0x3FF) << 20)
		| ((nlc_info.node.b_node[28] & 0x3FF) << 10);
	ISP_REG_WR(idx, ISP_NLC_PARA_B9, val);

	return ret;
}

static int isp_k_nlc_bypass(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int bypass = 0;

	ret = copy_from_user((void *)&bypass,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_nlc_bypass: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (bypass)
		ISP_REG_OWR(idx, ISP_NLC_PARA, BIT_0);
	else
		ISP_REG_MWR(idx, ISP_NLC_PARA, BIT_0, 0);

	return ret;
}

static int isp_k_nlc_node(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	struct nlc_node node;
	int i;
	unsigned int val = 0;

	memset(&node, 0x00, sizeof(node));
	ret = copy_from_user((void *)&node,
		param->property_param, sizeof(struct nlc_node));
	if (ret != 0) {

		pr_err("isp_k_nlc_node: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	for (i = 0; i < 9; i++) {
		val = ((node.r_node[i*3] & 0x3FF) << 20)
			| ((node.r_node[i*3+1] & 0x3FF) << 10)
			| (node.r_node[i*3+2] & 0x3FF);
		ISP_REG_WR(idx, ISP_NLC_PARA_R0 + i * 4, val);

		val = ((node.g_node[i*3] & 0x3FF) << 20)
			| ((node.g_node[i*3+1] & 0x3FF) << 10)
			| (node.g_node[i*3+2] & 0x3FF);
		ISP_REG_WR(idx, ISP_NLC_PARA_G0 + i * 4, val);

		val = ((node.b_node[i*3] & 0x3FF) << 20)
			| ((node.b_node[i*3+1] & 0x3FF) << 10)
			| (node.b_node[i*3+2] & 0x3FF);
		ISP_REG_WR(idx, ISP_NLC_PARA_B0 + i * 4, val);

		val = ((node.l_node[i*3] & 0x3FF) << 20)
			| ((node.l_node[i*3+1] & 0x3FF) << 10)
			| (node.l_node[i*3+2] & 0x3FF);
		ISP_REG_WR(idx, ISP_NLC_PARA_L0 + i * 4, val);
	}

	val = ((node.r_node[27] & 0x3FF) << 20)
		| ((node.r_node[28] & 0x3FF) << 10);
	ISP_REG_WR(idx, ISP_NLC_PARA_R9, val);

	val = ((node.g_node[27] & 0x3FF) << 20)
		| ((node.g_node[28] & 0x3FF) << 10);
	ISP_REG_WR(idx, ISP_NLC_PARA_G9, val);

	val = ((node.b_node[27] & 0x3FF) << 20)
		| ((node.b_node[28] & 0x3FF) << 10);
	ISP_REG_WR(idx, ISP_NLC_PARA_B9, val);

	return ret;
}

int isp_k_cfg_nlc(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_nlc: param is null error.\n");
		return -1;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_nlc: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_NLC_BLOCK:
		ret = isp_k_nlc_block(param, idx);
		break;
	case ISP_PRO_NLC_BYPASS:
		ret = isp_k_nlc_bypass(param, idx);
		break;
	case ISP_PRO_NLC_NODE:
		ret = isp_k_nlc_node(param, idx);
		break;

	default:
		pr_err("isp_k_cfg_nlc: fail cmd id:%d, not supported.\n",
			param->property);
		break;
	}

	return ret;
}



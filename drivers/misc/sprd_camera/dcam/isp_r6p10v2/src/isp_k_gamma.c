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

#define NON_3DNR_TEST
#define	ISP_FRGB_GAMC_BUF0				  0
#define	ISP_FRGB_GAMC_BUF1				  1

static int isp_k_gamma_bypass(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int bypass = 0;

	ret = copy_from_user((void *)&bypass,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("isp_k_gamma_bypass: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	if (bypass)
		ISP_REG_OWR(idx, ISP_GAMMA_PARAM, BIT_0);
	else
		ISP_REG_MWR(idx, ISP_GAMMA_PARAM, BIT_0, 0);

	return ret;
}

static int isp_k_gamma_buf_sel(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct gamma_buf_sel buf_sel = {0};

	ret = copy_from_user((void *)&buf_sel,
		param->property_param, sizeof(buf_sel));
	if (ret != 0) {
		pr_err("isp_k_gamma_buf_sel: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	val = ((buf_sel.r_buf_sel & 0x1) << 1)
		| ((buf_sel.g_buf_sel & 0x1) << 2)
		| ((buf_sel.b_buf_sel & 0x1) << 3);
	ISP_REG_MWR(idx, ISP_GAMMA_PARAM, 0x0000000E, val);

	return ret;
}

int isp_k_pingpang_frgb_gamc(struct gamc_curve_info *nodes,
		struct isp_k_block *isp_k_param, enum isp_id idx)

{
	int ret = 0;
	unsigned int val = 0;
	unsigned int i, j;
	int gamma_node_r = 0;
	int gamma_node_g = 0;
	int gamma_node_b = 0;
	unsigned long r_buf_addr;
	unsigned long g_buf_addr;
	unsigned long b_buf_addr;
	struct coordinate_xy *p_nodes_r = NULL;
	struct coordinate_xy *p_nodes_g = NULL;
	struct coordinate_xy *p_nodes_b = NULL;

	if (isp_k_param->full_gamma_buf_id
			== ISP_FRGB_GAMC_BUF0) {
		r_buf_addr = ISP_FGAMMA_R_BUF1_CH0;
		g_buf_addr = ISP_FGAMMA_G_BUF1_CH0;
		b_buf_addr = ISP_FGAMMA_B_BUF1_CH0;
		isp_k_param->full_gamma_buf_id = ISP_FRGB_GAMC_BUF1;
	} else {
		r_buf_addr = ISP_FGAMMA_R_BUF0_CH0;
		g_buf_addr = ISP_FGAMMA_G_BUF0_CH0;
		b_buf_addr = ISP_FGAMMA_B_BUF0_CH0;
		isp_k_param->full_gamma_buf_id = ISP_FRGB_GAMC_BUF0;
	}

	p_nodes_r = nodes->nodes_r;
	p_nodes_g = nodes->nodes_g;
	p_nodes_b = nodes->nodes_b;

	for (i = 0, j = 0; i < (ISP_PINGPANG_FRGB_GAMC_NUM - 1); i++, j += 4) {
		gamma_node_r = (((p_nodes_r[i].node_y & 0xFF) << 8)
				| (p_nodes_r[i + 1].node_y & 0xFF)) & 0xFFFF;
		gamma_node_g = (((p_nodes_g[i].node_y & 0xFF) << 8)
				| (p_nodes_g[i + 1].node_y & 0xFF)) & 0xFFFF;
		gamma_node_b = (((p_nodes_b[i].node_y & 0xFF) << 8)
				| (p_nodes_b[i + 1].node_y & 0xFF)) & 0xFFFF;

		ISP_REG_WR(idx, r_buf_addr + j, gamma_node_r);
		ISP_REG_WR(idx, g_buf_addr + j, gamma_node_g);
		ISP_REG_WR(idx, b_buf_addr + j, gamma_node_b);
	}

	val = ((isp_k_param->full_gamma_buf_id & 0x1) << 1) |
		  ((isp_k_param->full_gamma_buf_id & 0x1) << 2) |
		  ((isp_k_param->full_gamma_buf_id & 0x1) << 3);
	ISP_REG_MWR(idx, ISP_GAMMA_PARAM, 0x0000000E, val);

	return ret;
}

static int isp_k_gamma_block(struct isp_io_param *param,
			struct isp_k_block *isp_k_param, enum isp_id idx)
{
	int ret = 0;
	struct isp_dev_gamma_info *gamma_info_ptr = NULL;

	gamma_info_ptr = (struct isp_dev_gamma_info *)
				isp_k_param->full_gamma_buf_addr;

	ret = copy_from_user((void *)gamma_info_ptr,
		param->property_param, sizeof(struct isp_dev_gamma_info));
	if (ret != 0) {
		pr_err("isp_k_gamma_block: copy_from_user error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	ISP_REG_MWR(idx, ISP_GAMMA_PARAM, BIT_0,
				gamma_info_ptr->bypass);

	if (gamma_info_ptr->bypass)
		return 0;

	ret = isp_k_pingpang_frgb_gamc(&gamma_info_ptr->gamc_nodes,
					isp_k_param, idx);
	if (ret != 0) {
		pr_err("isp_gamma_block: pingpang error, ret=0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	return ret;
}

int isp_k_cfg_gamma(struct isp_io_param *param,
		struct isp_k_block *isp_k_param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_gamma: param is null error.\n");
		return -1;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_gamma: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_GAMMA_BLOCK:
		ret = isp_k_gamma_block(param, isp_k_param, idx);
		break;
	case ISP_PRO_GAMMA_BYPASS:
		ret = isp_k_gamma_bypass(param, idx);
		break;
	case ISP_PRO_GAMMA_BUF_SEL:
		ret = isp_k_gamma_buf_sel(param, idx);
		break;
	default:
		pr_err("isp_k_cfg_gamma: fail cmd id:%d, not supported.\n",
			param->property);
		break;
	}

	return ret;
}


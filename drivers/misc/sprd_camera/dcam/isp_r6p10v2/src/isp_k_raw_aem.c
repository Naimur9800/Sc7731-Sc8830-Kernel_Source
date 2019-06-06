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

static int isp_k_raw_aem_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int  val = 0;
	struct isp_dev_raw_aem_info aem_info;

	memset(&aem_info, 0x00, sizeof(aem_info));
	ret = copy_from_user((void *)&aem_info, param->property_param,
			     sizeof(struct isp_dev_raw_aem_info));

	if (ret != 0) {
		pr_err("aem_block: copy error, ret=0x%x\n", (unsigned int)ret);
		return -EPERM;
	}

	ISP_REG_MWR(idx, ISP_AEM_PARAM, BIT_0, aem_info.bypass);
	if (aem_info.bypass) {
		ISP_REG_MWR(idx, ISP_AEM_CFG_READY, BIT_0, 1);
		return 0;
	}

	/*aem work at continue frame mode*/
	ISP_REG_MWR(idx, ISP_AEM_PARAM, BIT_1, (0x1 << 1));
	val = (0x01 & 0xF) << 4;
	ISP_REG_MWR(idx, ISP_AEM_PARAM, 0xF0, val);

	val = ((0x18 & 0xFFFF) << 16) |
		(0x18 & 0xFFFF);
	ISP_REG_WR(idx, ISP_AEM_OFFSET, val);

	val = ((0x60 & 0x1FF) << 9) |
		(0x82 & 0x1FF);
	ISP_REG_MWR(idx, ISP_AEM_BLK_SIZE, 0x3FFFF, val);

	val = ((0x1070 & 0xFFFF) << 16) |
		(0xc30 & 0x1FF);
	ISP_REG_MWR(idx, ISP_AEM_SLICE_SIZE, 0x7C0000, val);

	ISP_REG_WR(idx, ISP_AEM_DDR_WR_NUM, 0x400);/*0x2000/8*/

	ISP_REG_MWR(idx, ISP_AEM_SKIP_NUM_CLR, BIT_0, 1);

	ISP_REG_MWR(idx, ISP_AEM_CFG_READY, BIT_0, 1);

	return ret;
}

static int isp_k_raw_aem_bypass(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int  bypass = 0;

	ret = copy_from_user((void *)&bypass, param->property_param,
			     sizeof(bypass));
	if (ret != 0) {
		pr_err("isp_k_raw_aem_bypass: copy error, ret=0x%x\n",
		       (unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_AEM_PARAM, BIT_0, bypass);
	ISP_REG_WR(idx, ISP_AEM_DDR_WR_NUM, 0x400);/*0x2000/8*/
	ISP_REG_MWR(idx, ISP_AEM_SKIP_NUM_CLR, BIT_0, 1);

	ISP_REG_MWR(idx, ISP_AEM_CFG_READY, BIT_0, 1);

	return ret;
}

static int32_t isp_k_raw_aem_slice_size(struct isp_io_param *param,
					enum isp_id idx)
{
	int32_t ret = 0;
	struct isp_img_size size;
	uint32_t val = 0;

	ret = copy_from_user((void *)&size, param->property_param,
				sizeof(struct isp_img_size));
	if (ret != 0) {
		pr_err("isp_k_raw_aem_slice_size: err, ret = 0x%x\n",
				(uint32_t)ret);
		return -1;
	}

	val = ((size.height & 0xFFFF) << 16) | (size.width & 0xFFFF);
	ISP_REG_WR(idx, ISP_AEM_SLICE_SIZE, val);
	ISP_REG_MWR(idx, ISP_AEM_CFG_READY, BIT_0, 1);

	return ret;
}

static int32_t isp_k_raw_aem_offset(struct isp_io_param *param, enum isp_id idx)
{
	int32_t ret = 0;
	uint32_t val = 0;
	struct img_offset offset;

	ret = copy_from_user((void *)&offset,
		param->property_param, sizeof(struct img_offset));
	if (ret != 0) {
		pr_err("isp_k_raw_aem_offset:read ret = 0x%x\n", (uint32_t)ret);
		return -1;
	}

	val = ((offset.y & 0xFFFF) << 16) | (offset.x & 0xFFFF);
	ISP_REG_WR(idx, ISP_AEM_OFFSET, val);
	ISP_REG_MWR(idx, ISP_AEM_CFG_READY, BIT_0, 1);

	return ret;
}

static int32_t isp_k_raw_aem_shift(struct isp_io_param *param, enum isp_id idx)
{
	int32_t ret = 0;
	uint32_t shift = 0;
	uint32_t val = 0;

	ret = copy_from_user((void *)&shift,
			param->property_param, sizeof(shift));
	if (ret != 0) {
		pr_err("isp_k_raw_aem_shift:error, ret = 0x%x\n",
				(uint32_t)ret);
		return -1;
	}

	val = (shift & 0x1F) << 18;
	if (val != 0)
		pr_debug("isp_k_raw_aem_shift: 0x%x\n", val);
	ISP_REG_MWR(idx, ISP_AEM_BLK_SIZE, 0x7C0000, val);
	ISP_REG_MWR(idx, ISP_AEM_CFG_READY, BIT_0, 1);

	return ret;
}

static int isp_k_raw_aem_mode(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int  mode = 0;

	ret = copy_from_user((void *)&mode,
			param->property_param, sizeof(mode));
	if (ret != 0) {
		pr_err("isp_k_raw_aem_mode: copy error, ret=0x%x\n",
		       (unsigned int)ret);
		return -1;
	}

	ISP_REG_MWR(idx, ISP_AEM_PARAM, BIT_1, mode << 1);
	ISP_REG_MWR(idx, ISP_AEM_CFG_READY, BIT_0, 1);

	return ret;
}

int isp_k_raw_aem_cfg_addr(unsigned long phy_addr, enum isp_id idx)
{
	int ret = 0;

	if (phy_addr == 0) {
		pr_err("isp_k_ra_aem_cfg_addr: Invalid address\n");
		return 1;
	}
	ISP_REG_WR(idx, ISP_AEM_DDR_ADDR, phy_addr);
	ISP_REG_MWR(idx, ISP_AEM_CFG_READY, BIT_0, 1);

	return ret;
}

static int isp_k_raw_aem_blk_size(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	struct isp_img_size size;
	unsigned int val = 0;

	ret = copy_from_user((void *)&size,
		param->property_param, sizeof(struct isp_img_size));
	if (ret != 0) {
		pr_err("isp_k_raw_aem_blk_size:ret = 0x%x\n",
				(unsigned int)ret);
		return ret;
	}

	val = ((size.height & 0x1FF) << 9) | (size.width & 0x1FF);
	ISP_REG_MWR(idx, ISP_AEM_BLK_SIZE, 0x3FFFF, val);
	ISP_REG_MWR(idx, ISP_AEM_CFG_READY, BIT_0, 1);

	return ret;
}

static int isp_k_raw_aem_skip_num(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int  val = 0;
	unsigned int  skip_num = 0;

	ret = copy_from_user((void *)&skip_num,
			param->property_param, sizeof(skip_num));
	if (ret != 0) {
		pr_err("isp_k_raw_aem_skip_num: copy error, ret=0x%x\n",
		       (unsigned int)ret);
		return -1;
	}

	val = (skip_num & 0xF) << 4;
	ISP_REG_MWR(idx, ISP_AEM_PARAM, 0xF0, val);
	ISP_REG_MWR(idx, ISP_AEM_CFG_READY, BIT_0, 1);

	return ret;
}

int isp_k_cfg_raw_aem(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_raw_aem: param is null error.\n");
		return -1;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_raw_aem: property_param is null error.\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_RAW_AEM_BLOCK:
		ret = isp_k_raw_aem_block(param, idx);
		break;
	case ISP_PRO_RAW_AEM_BYPASS:
		ret = isp_k_raw_aem_bypass(param, idx);
		break;
	case ISP_PRO_RAW_AEM_MODE:
		ret = isp_k_raw_aem_mode(param, idx);
		break;
	case ISP_PRO_RAW_AEM_SLICE_SIZE:
		ret = isp_k_raw_aem_slice_size(param, idx);
		break;
	case ISP_PRO_RAW_AEM_OFFSET:
		ret = isp_k_raw_aem_offset(param, idx);
		break;
	case ISP_PRO_RAW_AEM_SHIFT:
		ret = isp_k_raw_aem_shift(param, idx);
		break;
	case ISP_PRO_RAW_AEM_BLK_SIZE:
		ret = isp_k_raw_aem_blk_size(param, idx);
		break;
	case ISP_PRO_RAW_AEM_SKIP_NUM:
		ret = isp_k_raw_aem_skip_num(param, idx);
		break;

	default:
		pr_err("isp_k_cfg_raw_aem: cmd id:%d, not supported.\n",
		       param->property);
		break;
	}

	return ret;
}

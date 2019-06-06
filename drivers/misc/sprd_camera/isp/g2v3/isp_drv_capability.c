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
#include <video/sprd_isp_altek.h>
#include "isp_drv.h"

static int32_t isp_k_fw_buf_size(struct isp_capability *param,
				 struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint32_t buf_size = 0;

	buf_size = ISP_BUF_SIZE;
	ret = copy_to_user(param->property_param,
			&buf_size, sizeof(uint32_t));
	if (ret != 0) {
		pr_err("isp_k_fw_buf_size: copy error, ret=0x%x\n", ret);
		return -1;
	}
	pr_info("isp firmware buffer size 0x%x\n", buf_size);

	return ret;
}

static int32_t isp_k_statis_buf_size(struct isp_capability *param,
				     struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint32_t buf_size = 0;

	buf_size = ISP_STATISTICS_BUF_SIZE;
	ret = copy_to_user(param->property_param,
			&buf_size, sizeof(uint32_t));
	if (ret != 0) {
		pr_err("isp_k_statis_buf_size: copy error, ret=0x%x\n", ret);
		return -1;
	}
	pr_info("isp statistics buffer size 0x%x\n", buf_size);

	return ret;
}

static int32_t isp_k_dram_buf_size(struct isp_capability *param,
				   struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint32_t buf_size = 0;

	buf_size = ISP_DRAM_MODE_BUF_SIZE;
	ret = copy_to_user(param->property_param,
			&buf_size, sizeof(uint32_t));
	if (ret != 0) {
		pr_err("isp_k_dram_buf_size: copy error, ret=0x%x\n", ret);
		return -1;
	}
	pr_info("isp dram buffer size 0x%x\n", buf_size);

	return ret;
}

static int32_t isp_k_high_iso_buf_size(struct isp_capability *param,
				       struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint32_t buf_size = 0;

	buf_size = ISP_HIGH_QUALITY_MODE_BUF_SIZE;
	ret = copy_to_user(param->property_param,
			&buf_size, sizeof(uint32_t));
	if (ret != 0) {
		pr_err("isp_k_high_iso_buf_size: copy error, ret=0x%x\n", ret);
		return -1;
	}
	pr_info("isp dram buffer size 0x%x\n", buf_size);

	return ret;
}

static int32_t isp_k_get_continue_size(struct isp_capability *param,
				       struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	struct isp_img_size size = { 0, 0 };

	size.width = ISP_CONTINUE_WIDTH_MAX;
	size.height = ISP_CONTINUE_HEIGHT_MAX;

	ret = copy_to_user(param->property_param, &size, sizeof(size));
	if (ret != 0) {
		pr_err("isp_k_get_continue_size: copy error, ret=0x%x\n", ret);
		return -1;
	}
	pr_info("isp continue size 0x%x 0x%x\n", size.width, size.height);

	return ret;
}

static int32_t isp_k_get_single_size(struct isp_capability *param,
				     struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	struct isp_img_size size = { 0, 0 };

	size.width = ISP_SINGLE_WIDTH_MAX;
	size.height = ISP_SINGLE_HEIGHT_MAX;

	ret = copy_to_user(param->property_param, &size, sizeof(size));
	if (ret != 0) {
		pr_err("isp_k_get_single_size: copy error, ret=0x%x\n", ret);
		return -1;
	}
	pr_info("isp single size 0x%x 0x%x\n", size.width, size.height);

	return ret;
}

int32_t isp_capability(void __user *param, struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	struct isp_capability cap_param = { 0, NULL };

	if (!param) {
		pr_err("isp_capability: param is null error.\n");
		return -1;
	}

	ret = copy_from_user(&cap_param, param, sizeof(cap_param));
	if (ret != 0) {
		pr_err("isp_capability: copy_from_user error, ret = 0x%x\n",
				ret);
		return -1;
	}

	if (cap_param.property_param == NULL) {
		pr_err("isp_capability: property_param is null error.\n");
		return -1;
	}

	switch (cap_param.index) {
	case ISP_GET_FW_BUF_SIZE:
		ret = isp_k_fw_buf_size(&cap_param, isp_pipeline);
		break;
	case ISP_GET_STATIS_BUF_SIZE:
		ret = isp_k_statis_buf_size(&cap_param, isp_pipeline);
		break;
	case ISP_GET_DRAM_BUF_SIZE:
		ret = isp_k_dram_buf_size(&cap_param, isp_pipeline);
		break;
	case ISP_GET_HIGH_ISO_BUF_SIZE:
		ret = isp_k_high_iso_buf_size(&cap_param, isp_pipeline);
		break;
	case ISP_GET_CONTINUE_SIZE:
		ret = isp_k_get_continue_size(&cap_param, isp_pipeline);
		break;
	case ISP_GET_SINGLE_SIZE:
		ret = isp_k_get_single_size(&cap_param, isp_pipeline);
		break;
	default:
		break;
	}
	return ret;
}

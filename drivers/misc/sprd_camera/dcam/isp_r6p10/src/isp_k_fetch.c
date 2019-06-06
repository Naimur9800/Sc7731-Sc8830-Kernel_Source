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
#include <video/sprd_isp_r6p10.h>
#include "../isp_buf.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "FETCH: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

static int isp_k_fetch_raw_block(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, enum isp_id idx)
{
	int ret = 0;
	struct isp_dev_fetch_info fetch_info;
	unsigned int size = 0;

	memset(&fetch_info, 0x00, sizeof(fetch_info));
	ret = copy_from_user((void *)&fetch_info,
		param->property_param, sizeof(fetch_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	fetch_info.addr.chn0 = isp_k_param->fetch_raw_phys_addr;

	ISP_REG_MWR(idx, ISP_FETCH_PARAM, BIT_0, fetch_info.bypass);
	if (fetch_info.bypass)
		return 0;

	if (fetch_info.subtract)
		ISP_REG_OWR(idx, ISP_FETCH_PARAM, BIT_1);
	else
		ISP_REG_MWR(idx, ISP_FETCH_PARAM, BIT_1, 0);

	ISP_REG_MWR(idx, ISP_FETCH_PARAM,
		0xF0, (fetch_info.color_format << 4));

	size = ((fetch_info.size.height & 0xFFFF) << 16)
		| (fetch_info.size.width & 0xFFFF);
	ISP_REG_WR(idx, ISP_FETCH_SLICE_SIZE, size);

	ISP_REG_WR(idx, ISP_FETCH_SLICE_Y_ADDR,
		fetch_info.addr.chn0);

	ISP_REG_WR(idx, ISP_FETCH_SLICE_Y_PITCH,
		(fetch_info.pitch.chn0 & 0xFFFF));

	ISP_REG_WR(idx, ISP_FETCH_SLICE_U_ADDR,
		fetch_info.addr.chn1);

	ISP_REG_WR(idx, ISP_FETCH_SLICE_U_PITCH,
		(fetch_info.pitch.chn1 & 0xFFFF));

	ISP_REG_WR(idx, ISP_FETCH_SLICE_V_ADDR,
		fetch_info.addr.chn2);

	ISP_REG_WR(idx, ISP_FETCH_SLICE_V_PITCH,
		(fetch_info.pitch.chn2 & 0xFFFF));

	ISP_REG_WR(idx, ISP_FETCH_MIPI_WORD_INFO,
		(fetch_info.mipi_word_num & 0xFFFF));

	ISP_REG_WR(idx, ISP_FETCH_MIPI_BYTE_INFO,
		(fetch_info.mipi_byte_rel_pos & 0x0F));

	return ret;
}

static int isp_k_fetch_start(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int start = 0;
	unsigned int addr = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_FETCH:
		addr = ISP_FETCH_START;
		break;
	case ISP_BLOCK_YUV_FETCH:
		addr = ISP_FETCH2_START;
		break;
	default:
		pr_err("fail to support cmd id = %d\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&start,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	pr_info("fetch raw addr %x start %d\n", addr, start);
	ISP_REG_WR(idx, addr, start);

	return ret;
}

static int isp_k_fetch_slice_size(struct isp_io_param *param,
							enum isp_id idx)
{
	int ret = 0;
	unsigned int size = 0;
	struct isp_img_size slice_size = {0, 0};
	unsigned int addr = 0;

	switch (param->sub_block) {
	case ISP_BLOCK_FETCH:
		addr = ISP_FETCH_SLICE_SIZE;
		break;
	case ISP_BLOCK_YUV_FETCH:
		addr = ISP_FETCH2_SLICE_SIZE;
		break;
	default:
		pr_err("fail to support cmd id = %d\n",
			param->sub_block);
		return -1;
	}

	ret = copy_from_user((void *)&slice_size,
		param->property_param, sizeof(struct isp_img_size));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	size = ((slice_size.height & 0xFFFF) << 16)
		| (slice_size.width & 0xFFFF);

	ISP_REG_WR(idx, addr+ISP_FETCH_SLICE_SIZE, size);

	return ret;
}

static int isp_k_fetch_transaddr(struct isp_io_param *param,
			struct isp_k_block *isp_k_param)
{
	int ret = 0;
	struct isp_dev_block_addr fetch_buf;
	struct isp_statis_buf fetch_remap_buf;

	memset(&fetch_buf, 0x00, sizeof(struct isp_dev_block_addr));
	memset(&fetch_remap_buf, 0x00, sizeof(struct isp_statis_buf));
	ret = copy_from_user(&fetch_buf, param->property_param,
				sizeof(fetch_buf));

	fetch_remap_buf.pfinfo.dev = &s_isp_pdev->dev;
	fetch_remap_buf.pfinfo.mfd[0] = fetch_buf.img_fd;

	/*mapping iommu buffer*/
	ret = pfiommu_get_sg_table(&fetch_remap_buf.pfinfo);
	if (ret) {
		pr_err("fail to map iommu fetch buffer\n");
		ret = -1;
		return ret;
	}

	ret = pfiommu_get_addr(&fetch_remap_buf.pfinfo);
	if (ret) {
		pr_err("fail to get fetch param info addr\n");
		return ret;
	}
	isp_k_param->fetch_pfinfo.iova[0] =
		fetch_remap_buf.pfinfo.iova[0];
	isp_k_param->fetch_raw_phys_addr = fetch_remap_buf.pfinfo.iova[0]
		+ fetch_buf.img_offset.chn0;
	isp_k_param->fetch_pfinfo.size[0] = fetch_remap_buf.pfinfo.size[0];
	isp_k_param->fetch_pfinfo.dev = &s_isp_pdev->dev;

	pr_debug("fetch addr 0x%lx, size 0x%zx\n",
		isp_k_param->fetch_pfinfo.iova[0],
		fetch_remap_buf.pfinfo.size[0]);

	return ret;
}

int isp_k_cfg_fetch(struct isp_io_param *param,
			struct isp_k_block *isp_k_param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("fail to get param\n");
		return -1;
	}

	if (param->property_param == NULL) {
		pr_err("fail to get property_param\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_FETCH_RAW_BLOCK:
		ret = isp_k_fetch_raw_block(param, isp_k_param, idx);
		break;
	case ISP_PRO_FETCH_START:
		ret = isp_k_fetch_start(param, idx);
		break;
	case ISP_PRO_FETCH_SLICE_SIZE:
		ret = isp_k_fetch_slice_size(param, idx);
		break;
	case ISP_PRO_FETCH_TRANSADDR:
		ret = isp_k_fetch_transaddr(param, isp_k_param);
		break;
	default:
		pr_err("fail to support cmd id = %d\n",
			param->property);
		break;
	}

	return ret;
}


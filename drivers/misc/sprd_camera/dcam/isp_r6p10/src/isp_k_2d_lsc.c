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
#include <linux/delay.h>
#include <video/sprd_mm.h>
#include <video/sprd_isp_r6p10.h>
#include "../isp_drv.h"
#include "../isp_buf.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "2D LSC: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

#define ISP_LSC_TIME_OUT_MAX        500
#define ISP_LSC_BUF0                0
#define ISP_LSC_BUF1                1
#define ISP_BYPASS_EB					   1
#define  ISP_INT_EVT_LSC_LOAD (1<<7)

static int isp_k_2d_lsc_param_load
			(struct isp_k_block *isp_k_param, enum isp_id idx)
{
	int ret = 0;
	unsigned int time_out_cnt = 0;
	unsigned int reg_value = 0;

	reg_value = ISP_REG_RD(idx, ISP_LENS_PARAM);
	isp_k_param->lsc_load_buf_id = !((reg_value >> 1) & 1);

	ISP_REG_MWR(idx, ISP_LENS_LOAD_BUF,
			BIT_0, isp_k_param->lsc_load_buf_id);

	ISP_REG_WR(idx, ISP_LENS_PARAM_ADDR, isp_k_param->lsc_buf_phys_addr);
	ISP_REG_OWR(idx, ISP_LENS_LOAD_EB, BIT_0);

	isp_k_param->lsc_update_buf_id = isp_k_param->lsc_load_buf_id;

	reg_value = ISP_REG_RD(idx, ISP_INT_RAW0);

	while (((reg_value & ISP_INT_EVT_LSC_LOAD) == 0x00)
		&& (time_out_cnt < ISP_LSC_TIME_OUT_MAX)) {
		udelay(1);
		reg_value = ISP_REG_RD(idx, ISP_INT_RAW0);
		time_out_cnt++;
	}
	if (time_out_cnt >= ISP_LSC_TIME_OUT_MAX) {
		ret = -EPERM;
		pr_err("fail to load lsc param,lsc load time out\n");
	}
	ISP_REG_OWR(idx, ISP_INT_CLR0, ISP_INT_EVT_LSC_LOAD);

	ISP_REG_MWR(idx, ISP_LENS_PARAM, BIT_1,
			isp_k_param->lsc_update_buf_id << 1);

	return ret;
}

static int isp_k_2d_lsc_block(struct isp_io_param *param,
		struct isp_k_block *isp_k_param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	unsigned int i = 0;
	unsigned int weight_num;
	unsigned long dst_addr = 0;
	void *data_ptr = NULL;
	unsigned short *w_buff = NULL;
	struct isp_dev_2d_lsc_info lens_info;
	unsigned long flag;

	memset(&lens_info, 0x00, sizeof(lens_info));

	spin_lock_irqsave(&isp_mod_lock[idx], flag);
	if (isp_k_param->lsc_updated) {
		spin_unlock_irqrestore(&isp_mod_lock[idx], flag);
		pr_info("already updated for current frame\n");
		ret = 0;
		goto exit;
	}
	spin_unlock_irqrestore(&isp_mod_lock[idx], flag);

	ret = copy_from_user((void *)&lens_info, param->property_param,
			sizeof(lens_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		ret = -EPERM;
		goto exit;
	}
	ISP_REG_MWR(idx, ISP_LENS_PARAM, BIT_0, lens_info.bypass);
	if (lens_info.bypass) {
		ret = 0;
		goto exit;
	}

	ISP_REG_WR(idx, ISP_LENS_SLICE_POS, 0);

	val = lens_info.grid_pitch & 0x1FF;
	ISP_REG_MWR(idx, ISP_LENS_GRID_PITCH, 0x1FF, val);

	val = (lens_info.grid_width & 0x1FF) << 16;
	ISP_REG_MWR(idx, ISP_LENS_GRID_PITCH, (0x1FF << 16), val);

	val = ((lens_info.grid_num_t & 0xFFFF) << 16) |
		((lens_info.grid_y_num & 0xFF) << 8) |
		(lens_info.grid_x_num & 0xFF);
	ISP_REG_WR(idx, ISP_LENS_GRID_SIZE, val);

	ISP_REG_MWR(idx, ISP_LENS_MISC, (BIT_1 | BIT_0),
				(lens_info.endian & 0x03));

	val = ((lens_info.slice_size.height & 0xFFFF) << 16) |
		(lens_info.slice_size.width & 0xFFFF);
	ISP_REG_WR(idx, ISP_LENS_SLICE_SIZE, val);

	val = ((lens_info.relative_y & 0x3FF) << 16) |
		(lens_info.relative_x & 0x3FF);
	ISP_REG_WR(idx, ISP_LENS_INIT_COOR, val);

	ret = isp_k_2d_lsc_param_load(isp_k_param, idx);
	if (ret != 0) {
		pr_err("fail to load lsc param, ret = %d\n", ret);
		ISP_REG_MWR(idx, ISP_LENS_PARAM, BIT_0, ISP_BYPASS_EB);
		ret = -EPERM;
		goto exit;
	}

	for (i = 0; i < 5; i++) {
		val = ((lens_info.q_value[0][i] & 0x3FFF) << 16) |
			(lens_info.q_value[1][i] & 0x3FFF);
		ISP_REG_WR(idx, ISP_LENS_Q0_VALUE + i * 4, val);
	}

	if (!isp_k_param->lsc_2d_weight_en) {
#ifdef CONFIG_64BIT
		data_ptr = (void *)(((unsigned long)lens_info.data_ptr[1] << 32)
						| lens_info.data_ptr[0]);
#else
		data_ptr = (void *)(lens_info.data_ptr[0]);
#endif
		isp_k_param->lsc_2d_weight_en = 0;
		dst_addr = ISP_BASE_ADDR(idx) + ISP_REG_LENS_WEIGHT_ADDR;
		w_buff = vzalloc(lens_info.weight_num);

		if (w_buff == NULL) {
			pr_err("fail to vzalloc buff\n");
			ret = -ENOMEM;
			goto exit;
		}

		ret = copy_from_user((void *)w_buff,  data_ptr,
			lens_info.weight_num);

		if (ret != 0) {
			pr_err("fail to copy from user, ret = %d\n",
				ret);
			vfree(w_buff);
			ret = -1;
			goto exit;
		}

		weight_num = lens_info.grid_width / 2 + 1;

		for (i = 0; i < weight_num ; i++) {
			*((unsigned int *)dst_addr + i * 2) =
				(((unsigned int)(*(w_buff + i * 3))) & 0xFFFF) |
				((((unsigned int)(*(w_buff+i * 3 + 1))) &
				0xFFFF) << 16);

			*((unsigned int *)dst_addr + i * 2 + 1) =
				((unsigned int)(*(w_buff + i * 3 + 2))) &
				0xFFFF;
		}
		vfree(w_buff);
	}

	spin_lock_irqsave(&isp_mod_lock[idx], flag);
	isp_k_param->lsc_updated = 1;
	spin_unlock_irqrestore(&isp_mod_lock[idx], flag);

exit:
	return ret;
}

static int isp_k_2d_lsc_slice_size
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_img_size size = {0, 0};

	ret = copy_from_user((void *)&size, param->property_param,
			sizeof(size));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -EPERM;
	}

	val = ((size.height & 0xFFFF) << 16) | (size.width & 0xFFFF);

	ISP_REG_WR(idx, ISP_LENS_SLICE_SIZE, val);

	return ret;
}

static int isp_k_2d_lsc_transaddr(struct isp_io_param *param,
			struct isp_k_block *isp_k_param)
{
	int ret = 0;
	struct isp_dev_block_addr lsc_buf;
	struct isp_statis_buf lsc_remap_buf;

	memset(&lsc_buf, 0x00, sizeof(struct isp_dev_block_addr));
	memset(&lsc_remap_buf, 0x00, sizeof(struct isp_statis_buf));
	ret = copy_from_user(&lsc_buf, param->property_param,
			     sizeof(lsc_buf));

	lsc_remap_buf.pfinfo.dev = &s_isp_pdev->dev;
	lsc_remap_buf.pfinfo.mfd[0] = lsc_buf.img_fd;

	/*mapping iommu buffer*/
	ret = pfiommu_get_sg_table(&lsc_remap_buf.pfinfo);
	if (ret) {
		pr_err("fail to map iommu lsc buffer\n");
		ret = -1;
		return ret;
	}

	ret = pfiommu_get_addr(&lsc_remap_buf.pfinfo);
	if (ret) {
		pr_err("fail to get 2d lsc info addr\n");
		return ret;
	}

	isp_k_param->lsc_pfinfo.iova[0] =
		lsc_remap_buf.pfinfo.iova[0];
	isp_k_param->lsc_buf_phys_addr = lsc_remap_buf.pfinfo.iova[0]
		+ lsc_buf.img_offset.chn0;
	isp_k_param->lsc_pfinfo.size[0] = lsc_remap_buf.pfinfo.size[0];
	isp_k_param->lsc_pfinfo.dev = &s_isp_pdev->dev;

	pr_debug("lsc addr 0x%lx, size 0x%zx\n",
		isp_k_param->lsc_pfinfo.iova[0], lsc_remap_buf.pfinfo.size[0]);

	return ret;
}

int isp_k_cfg_2d_lsc(struct isp_io_param *param,
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
	case ISP_PRO_2D_LSC_BLOCK:
		ret = isp_k_2d_lsc_block(param, isp_k_param, idx);
		break;
	case ISP_PRO_2D_LSC_SLICE_SIZE:
		ret = isp_k_2d_lsc_slice_size(param, idx);
		break;
	case ISP_PRO_2D_LSC_TRANSADDR:
		ret = isp_k_2d_lsc_transaddr(param, isp_k_param);
		break;
	default:
		pr_err("fail to support cmd id = %d\n",
			param->property);
		break;
	}

	return ret;
}

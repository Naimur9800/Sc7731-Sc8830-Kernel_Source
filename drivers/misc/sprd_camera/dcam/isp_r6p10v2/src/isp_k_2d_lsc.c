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
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <video/sprd_mm.h>
#include <video/sprd_isp_r6p10v2.h>
#include "../isp_drv.h"
#include "../isp_buf.h"
#include "../isp_int.h"

#define ISP_LSC_TIME_OUT_MAX        500
#define ISP_LSC_BUF0                0
#define ISP_LSC_BUF1                1
#define ISP_BYPASS_EB					   1
#define  ISP_INT_EVT_LSC_LOAD (1<<7)
static int s_dbg_cnt;
static int s_dbg_skip_cnt;

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
		pr_err("isp_k_2d_lsc_param_load: lsc load time out error.\n");
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
	unsigned int irq_line;
	unsigned int val = 0;
	unsigned int i = 0;
	unsigned int weight_num;
	unsigned long dst_addr = 0, flag;
	void *data_ptr = NULL;
	unsigned short *w_buff = NULL;
	struct isp_dev_2d_lsc_info lens_info;
	int sof_cnt, val_lsc;

	/* Make atomic op for SOF status and lsc_updated */
	spin_lock_irqsave(&isp_k_param->lsc_lock, flag);
	irq_line = ISP_REG_RD(idx, ISP_INT_INT0);
	sof_cnt = atomic_read(&isp_k_param->lsc_updated);
	spin_unlock_irqrestore(&isp_k_param->lsc_lock, flag);

	s_dbg_cnt++;
	if (irq_line & (1 << ISP_INT_DCAM_SOF)) {
		s_dbg_skip_cnt++;
		pr_debug("next frame start. cnt: %d, %d\n",
					s_dbg_cnt, s_dbg_skip_cnt);
		return 0;
	} else if (sof_cnt <= 0) {
		s_dbg_skip_cnt++;
		pr_debug("lsc buffer is updated for current frame. cnt: %d, %d\n",
					s_dbg_cnt, s_dbg_skip_cnt);
		return 0;
	}

	memset(&lens_info, 0x00, sizeof(lens_info));

	ret = copy_from_user((void *)&lens_info, param->property_param,
			sizeof(lens_info));
	if (ret != 0) {
		pr_err("2d_lsc_block: copy error, ret=0x%x\n",
						(unsigned int)ret);
		return -EPERM;
	}
	ISP_REG_MWR(idx, ISP_LENS_PARAM, BIT_0, lens_info.bypass);
	if (lens_info.bypass)
		return 0;

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
		pr_err("lsc load para error, ret=0x%x\n", (unsigned int)ret);
		ISP_REG_MWR(idx, ISP_LENS_PARAM, BIT_0, ISP_BYPASS_EB);
		return -EPERM;
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
			/*pr_err("isp_k_2d_lsc_block:2d_lsc vzalloc error\n");*/
			return -ENOMEM;
		}

		ret = copy_from_user((void *)w_buff,  data_ptr,
			lens_info.weight_num);

		if (ret != 0) {
			pr_err("isp_k_2d_lsc_block: copy error 0x%x\n",
				(unsigned int)ret);
			vfree(w_buff);
			return -1;
		}

		weight_num = lens_info.grid_width/2 + 1;

		for (i = 0; i < weight_num ; i++) {
			*((unsigned int *)dst_addr + i*2) =
				(((unsigned int)(*(w_buff+i*3))) & 0xFFFF) |
				((((unsigned int)(*(w_buff+i*3+1))) & 0xFFFF)
				<< 16);

			*((unsigned int *)dst_addr + i*2+1) =
				((unsigned int)(*(w_buff+i*3+2))) & 0xFFFF;
		}
		vfree(w_buff);
	}

	spin_lock_irqsave(&isp_k_param->lsc_lock, flag);
	irq_line = ISP_REG_RD(idx, ISP_INT_INT0);
	val_lsc = (irq_line & (1 << ISP_INT_DCAM_SOF)) ? -1 : 0;
	atomic_set(&isp_k_param->lsc_updated, val_lsc);
	spin_unlock_irqrestore(&isp_k_param->lsc_lock, flag);

	pr_debug("lsc updated end. cnt: %d, %d\n",
			val_lsc, atomic_read(&isp_k_param->lsc_updated));
	return ret;
}

static int isp_k_2d_lsc_param_update
		(struct isp_io_param *param,
		struct isp_k_block *isp_k_param, enum isp_id idx)
{
	int ret = 0;
	unsigned int flag = 0;

	ret = copy_from_user((void *)&flag,
		param->property_param, sizeof(flag));
	if (ret != 0) {
		pr_err("lsc_param_update copy error, ret=0x%x\n",
			(unsigned int)ret);
		return -1;
	}

	isp_k_param->is_lsc_param_init_flag = flag;

	return ret;
}

static int isp_k_2d_lsc_pos(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_img_offset offset = {0, 0};

	ret = copy_from_user((void *)&offset, param->property_param,
			sizeof(offset));
	if (ret != 0) {
		pr_err("isp_lens_pos: copy error, ret=0x%x\n",
						(unsigned int)ret);
		return -EPERM;
	}

	val = ((offset.y & 0xFFFF) << 16) | (offset.x & 0xFFFF);

	ISP_REG_WR(idx, ISP_LENS_SLICE_POS, val);

	return ret;
}

static int isp_k_2d_lsc_grid_size
	(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	unsigned int grid_total = 0;
	struct isp_img_size size = {0, 0};

	ret = copy_from_user((void *)&size, param->property_param,
			sizeof(size));
	if (ret != 0) {
		pr_err("grid_size: copy error, ret=0x%x\n", (unsigned int)ret);
		return -EPERM;
	}

	grid_total = (size.height + 2) * (size.width + 2);
	val = ((grid_total & 0xFFFF) << 16) |
		((size.height & 0xFF) << 8) |
		(size.width & 0xFF);
	ISP_REG_WR(idx, ISP_LENS_GRID_SIZE, val);

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
		pr_err("slice_size: copy error, ret=0x%x\n", (unsigned int)ret);
		return -EPERM;
	}

	val = ((size.height & 0xFFFF) << 16) | (size.width & 0xFFFF);

	ISP_REG_WR(idx, ISP_LENS_SLICE_SIZE, val);

	return ret;
}

static int isp_k_2d_lsc_transaddr(struct isp_io_param *param,
			struct isp_k_block *isp_k_param, enum isp_id idx)
{
	int ret = 0;
	struct isp_dev_block_addr lsc_buf;
	struct isp_statis_buf lsc_remap_buf;

	memset(&lsc_buf, 0x00, sizeof(struct isp_dev_block_addr));
	memset(&lsc_remap_buf, 0x00, sizeof(struct isp_statis_buf));
	ret = copy_from_user(&lsc_buf, param->property_param,
			     sizeof(lsc_buf));

	if (idx == ISP_ID_0)
		lsc_remap_buf.pfinfo.dev = &s_isp_pdev->dev;
	else
		lsc_remap_buf.pfinfo.dev = &s_isp1_pdev->dev;
	lsc_remap_buf.pfinfo.mfd[0] = lsc_buf.img_fd;

	/*mapping iommu buffer*/
	ret = pfiommu_get_sg_table(&lsc_remap_buf.pfinfo);
	if (ret) {
		pr_err("map iommu lsc buffer failed.\n");
		ret = -1;
		return ret;
	}

	ret = pfiommu_get_addr(&lsc_remap_buf.pfinfo);
	if (ret) {
		pr_err("map iommu lsc buffer failed: get address error!\n");
		return -1;
	}

	isp_k_param->lsc_pfinfo.iova[0] =
		lsc_remap_buf.pfinfo.iova[0];
	isp_k_param->lsc_buf_phys_addr = lsc_remap_buf.pfinfo.iova[0]
		+ lsc_buf.img_offset.chn0;
	isp_k_param->lsc_pfinfo.size[0] = lsc_remap_buf.pfinfo.size[0];
	if (idx == ISP_ID_0)
		isp_k_param->lsc_pfinfo.dev = &s_isp_pdev->dev;
	else
		isp_k_param->lsc_pfinfo.dev = &s_isp1_pdev->dev;

	pr_debug("lsc addr 0x%lx, size 0x%x\n", isp_k_param->lsc_pfinfo.iova[0],
		(unsigned int)lsc_remap_buf.pfinfo.size[0]);

	return ret;
}

static int isp_k_2d_lsc_slice(struct isp_io_param *param,
		struct isp_k_block *isp_k_param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	unsigned int i = 0;
	struct isp_dev_2d_lsc_info lens_info;

	memset(&lens_info, 0x00, sizeof(lens_info));

	ret = copy_from_user((void *)&lens_info,
		param->property_param, sizeof(lens_info));
	if (ret != 0) {
		pr_err("2d_lsc_slice: copy error, ret=0x%x\n",
			(unsigned int)ret);
		return -EPERM;
	}

	/*should be 0,0 when slice = fram_size*/
	ISP_REG_WR(idx, ISP_LENS_SLICE_POS, 0);

	val = ((lens_info.grid_num_t & 0xFFFF)) << 16 |
		(lens_info.grid_y_num & 0xFF) << 8 |
		(lens_info.grid_x_num & 0xFF);
	ISP_REG_WR(idx, ISP_LENS_GRID_SIZE, val);

	val = ((lens_info.slice_size.height & 0xFFFF) << 16) |
			(lens_info.slice_size.width & 0xFFFF);
		ISP_REG_WR(idx, ISP_LENS_SLICE_SIZE, val);

	val = ((lens_info.relative_y & 0x3FF) << 16) |
		(lens_info.relative_x & 0x3FF);
	ISP_REG_WR(idx, ISP_LENS_INIT_COOR, val);

	for (i = 0; i < 5; i++) {
		val = ((lens_info.q_value[0][i] & 0x3FFF) << 16) |
			(lens_info.q_value[1][i] & 0x3FFF);
		ISP_REG_WR(idx, ISP_LENS_Q0_VALUE + i * 4, val);
	}

	ret = isp_k_2d_lsc_param_load(isp_k_param, idx);
	if (ret != 0) {
		pr_err("lsc load para error,ret=0x%x\n", (unsigned int)ret);
		ISP_REG_MWR(idx, ISP_LENS_PARAM, BIT_0, ISP_BYPASS_EB);
		return -EPERM;
	}

	return ret;
}
int isp_k_cfg_2d_lsc(struct isp_io_param *param,
			struct isp_k_block *isp_k_param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_2d_lsc: param is null error.\n");
		return -EPERM;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_2d_lsc: property_param is null error.\n");
		return -EPERM;
	}

	switch (param->property) {
	case ISP_PRO_2D_LSC_BLOCK:
		ret = isp_k_2d_lsc_block(param, isp_k_param, idx);
		break;
	case ISP_PRO_2D_LSC_PARAM_UPDATE:
		ret = isp_k_2d_lsc_param_update(param, isp_k_param, idx);
		break;
	case ISP_PRO_2D_LSC_POS:
		ret = isp_k_2d_lsc_pos(param, idx);
		break;
	case ISP_PRO_2D_LSC_GRID_SIZE:
		ret = isp_k_2d_lsc_grid_size(param, idx);
		break;
	case ISP_PRO_2D_LSC_SLICE_SIZE:
		ret = isp_k_2d_lsc_slice_size(param, idx);
		break;
	case ISP_PRO_2D_LSC_TRANSADDR:
		ret = isp_k_2d_lsc_transaddr(param, isp_k_param, idx);
		break;
	case ISP_PRO_2D_LSC_SLICE:
		ret = isp_k_2d_lsc_slice(param, isp_k_param, idx);
		break;
	default:
		pr_err("2d: fail cmd id:%d, not supported.\n", param->property);
		break;
	}

	return ret;
}

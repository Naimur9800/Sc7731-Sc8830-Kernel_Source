/*
 * Copyright (C) 2015-2016 Spreadtrum Communications Inc.
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
#ifndef _CPP_CORE_H_
#define _CPP_CORE_H_

#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <video/sprd_mm.h>
#include <linux/dma-buf.h>
#include <linux/scatterlist.h>
#include <linux/sprd_iommu.h>
#include <linux/sprd_ion.h>

struct cpp_iommu_info {
	struct device *dev;
	unsigned int mfd[3];
	struct sg_table *table[3];
	size_t size[3];
	unsigned long iova[3];
	struct dma_buf *dmabuf_p[3];
	unsigned int offset[3];
};
void cpp_print_reg(void *);
int cpp_get_sg_table(struct cpp_iommu_info *pfinfo);
int cpp_get_addr(struct cpp_iommu_info *pfinfo, enum sprd_iommu_chtype ch_type);
int cpp_free_addr(struct cpp_iommu_info *pfinfo,
						enum sprd_iommu_chtype ch_type);

/* Interfaces to DCAM*/
int cpp_k_open_device(void);
int cpp_k_close_device(void);
int cpp_k_start_scale(struct sprd_dcam_img_frm *src_img,
	struct sprd_dcam_img_frm *dst_img1,
	struct sprd_dcam_img_frm *dst_img2, int *path_id);
int cpp_k_stop_scale(int path_id);

#endif

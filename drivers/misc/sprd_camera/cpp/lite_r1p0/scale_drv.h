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
#ifndef _SCALE_DRV_H_
#define _SCALE_DRV_H_

#include "cpp_core.h"

#define SC_COEFF_BUF_SIZE			(16 << 10)

struct scale_drv_private {
	struct sprd_cpp_scale_cfg_parm cfg_parm;

	struct sprd_cpp_size sc_input_size;
	unsigned int slice_in_height;
	unsigned int slice_out_height;
	unsigned int sc_deci_val;
	void *coeff_addr;

	void *priv;

	struct cpp_iommu_info iommu_src;
	struct cpp_iommu_info iommu_dst;

	void __iomem *io_base;

	struct platform_device *pdev;
};

void get_cpp_max_size(unsigned int *max_width,
		      unsigned int *max_height);
int cpp_scale_start(struct sprd_cpp_scale_cfg_parm *parm,
		    struct scale_drv_private *p);
void cpp_scale_stop(struct scale_drv_private *p);

#endif /* _SCALE_DRV_H_ */


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
#ifndef _CPP_PATH2_DRV_H_
#define _CPP_PATH2_DRV_H_

#include "cpp_core.h"

#define SC_COEFF_BUF_SIZE			(16 << 10)

struct cpp_path2_drv_private {
	struct sprd_cpp_path2_cfg_parm cfg_parm;

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
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
		int is_from_isp;
#endif
};

int cpp_path2_start(struct sprd_cpp_path2_cfg_parm *parm,
		    struct cpp_path2_drv_private *p);
void cpp_path2_stop(struct cpp_path2_drv_private *p);

#endif /* _PREVIEW_DRV_H_ */

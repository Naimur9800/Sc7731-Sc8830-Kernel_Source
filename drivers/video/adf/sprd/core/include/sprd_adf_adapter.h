/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
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

#ifndef _SPRD_ADF_H_
#define _SPRD_ADF_H_

#include "sprd_adf_common.h"

void sprd_adf_report_vsync(struct device *dev, u32 flags);
void sprd_adf_fence_signal(struct device *dev);
void sprd_adf_adapter_shutdown(struct platform_device *pdev);
struct sprd_adf_context *sprd_adf_context_create(struct platform_device *pdev);
void sprd_adf_context_destroy(struct sprd_adf_context *ctx);
#endif

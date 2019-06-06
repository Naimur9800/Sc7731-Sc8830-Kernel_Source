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
#ifndef _CAM_PW_DOMAIN_H_
#define _CAM_PW_DOMAIN_H_

#include <video/mm_pw_domain.h>

#define REG_PMU_CAM_SYS_PD_STATE	(0x7 << 20) /* bit[24:20] */
#define REG_PMU_ISP_TOP_PD_STATE	(0x7 << 5) /* bit[9:5] */

#define REG_MM_AHB_AHB_EB	(0x0000)
#define REG_MM_AHB_AHB_RST	(0x0004)
#define REG_MM_AHB_MODULE_RST	(0x0030)

int sprd_cam_pw_domain_init(struct platform_device *pdev);
int sprd_cam_pw_domain_deinit(void);
int sprd_isp_pw_on(void);
int sprd_isp_pw_off(void);

int sprd_cam_pw_on(void);
int sprd_cam_pw_off(void);
int sprd_cam_domain_eb(void);
int sprd_cam_domain_disable(void);

#endif /* _CAM_PW_DOMAIN_H_ */

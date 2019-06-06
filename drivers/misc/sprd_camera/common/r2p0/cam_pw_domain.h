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

int sprd_cam_pw_domain_init(struct platform_device *pdev);
int sprd_cam_pw_domain_deinit(void);
void sprd_isp_pw_on(void);
void sprd_isp_pw_off(void);

/* workarount for power domain, r2p0, only can be called by sensor driver */
int sprd_cam_pw_on(void);
int sprd_cam_pw_off(void);
int sprd_cam_domain_eb(void);
int sprd_cam_domain_disable(void);

#endif /* _CAM_PW_DOMAIN_H_ */

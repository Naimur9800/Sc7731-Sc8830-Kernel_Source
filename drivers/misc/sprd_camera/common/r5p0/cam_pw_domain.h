/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
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

#define REG_PMU_APB_PD_MM_SYS_CFG  (0x001c)
#define BIT_PMU_APB_PD_MM_SYS_AUTO_SHUTDOWN_EN	BIT(24)
#define BIT_PMU_APB_PD_MM_SYS_FORCE_SHUTDOWN	BIT(25)

#define REG_PMU_APB_PD_STATE	(0x010c)
#define PD_MM_STAT_BIT_SHIFT 10
#define BIT_PMU_APB_PD_MM_SYS_STATE(x)	(((x) & 0xf) << PD_MM_STAT_BIT_SHIFT)
#define PD_MM_DOWN_FLAG (0x7 << PD_MM_STAT_BIT_SHIFT)

#define QOS_THREHOLD_MM (0x0010)
#define QOS_THREHOLD_MM_MASK (0xff)

int sprd_cam_pw_domain_init(struct platform_device *pdev);
int sprd_cam_pw_domain_deinit(void);
int sprd_cam_pw_on(void);
int sprd_cam_pw_off(void);
int sprd_cam_domain_eb(void);
int sprd_cam_domain_disable(void);

#endif /* _CAM_PW_DOMAIN_H_ */

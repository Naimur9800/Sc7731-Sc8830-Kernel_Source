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
#ifndef _MM_PW_DOMAIN_H_
#define _MM_PW_DOMAIN_H_

enum {
	SPRD_PW_DOMAIN_DISPC = 0,
	SPRD_PW_DOMAIN_DSI,
	SPRD_PW_DOMAIN_DCAM,
	SPRD_PW_DOMAIN_ISP,
	SPRD_PW_DOMAIN_CPP,
	SPRD_PW_DOMAIN_SENSOR,
	SPRD_PW_DOMAIN_JPG,
	SPRD_PW_DOMAIN_COUNT_MAX,
};
int sprd_cam_pw_on(void);
int sprd_cam_pw_off(void);
void sprd_mm_pw_on(unsigned int client);
void sprd_mm_pw_off(unsigned int client);

#endif /* _MM_PW_DOMAIN_H_ */


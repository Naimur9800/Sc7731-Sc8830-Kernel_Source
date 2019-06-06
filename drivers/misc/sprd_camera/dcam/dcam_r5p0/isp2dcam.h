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
#ifndef _ISP2DCAM_H_
#define _ISP2DCAM_H_

#define ISP2DCAM_INT_STATUS            0x00
#define ISP2DCAM_INT_EN0               0x14
#define ISP2DCAM_INT_CLR0              0x18
#define ISP2DCAM_INT_RAW0              0x1c
#define ISP2DCAM_INT0                  0x20

#define ISP2DCAM0_STATUS0              0x200
#define ISP2DCAM0_STATUS1              0x204
#define ISP2DCAM1_STATUS0              0x208
#define ISP2DCAM1_STATUS1              0x20c
#define ISP2DCAM0_SLICE_SIZE           0x218
#define ISP2DCAM1_SLICE_SIZE           0x21c
#define ISP2DCAM_PMU_RAM_MASK          0x220
#define ISP2DCAM_RESERVED              0x224

struct isp2dcam_size {
	unsigned int width;
	unsigned int height;
};

struct isp2dcam_cfg_parm {
	unsigned char dcam0_eb;
	unsigned char dcam1_eb;
	struct isp2dcam_size slice_size[2];
};

void sprd_isp2dcam_config_parm(struct isp2dcam_cfg_parm *cfg_parm);
void sprd_isp2dcam_enable(void);
void sprd_isp2dcam_disable(void);
void sprd_isp2dcam_dump_reg(void);

#endif


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

struct sprd_clockevent {
	int freq;
	void __iomem *base;
	struct clock_event_device evt;
};

#define to_sprd_evt(evt) container_of(evt, struct sprd_clockevent, evt)

/* register define of R3P0 */
#define LOAD_R3P0		(0x0000)
#define VALUE_R3P0		(0x0004)

#define CTL_R3P0		(0x0008)
#define CTL_PERIOD_MODE_R3P0	(1 << 6)
#define CTL_ENABLE_R3P0		(1 << 7)
#define CTL_NEW_R3P0		(1 << 8)

#define INT_R3P0		(0x000C)
#define INT_EN_R3P0		(1 << 0)
#define INT_RAW_STS_R3P0	(1 << 1)
#define INT_MASK_STS_R3P0	(1 << 2)
#define INT_CLR_R3P0		(1 << 3)
#define INT_BUSY_R3P0		(1 << 4)

#define CNT_RD_R3P0		(0x0010)

/* register define of R4P0 */
#define LOAD_LO_R4P0		(0x0000)
#define LOAD_HI_R4P0		(0x0004)
#define VALUE_LO_R4P0		(0x0008)
#define VALUE_HI_R4P0		(0x000C)

#define CTL_R4P0		(0x0010)
#define CTL_PERIOD_MODE_R4P0	(1 << 0)
#define CTL_ENABLE_R4P0		(1 << 1)
#define CTL_WIDTH_SEL_R4P0	(1 << 16)

#define INT_R4P0		(0x0014)
#define INT_EN_R4P0		(1 << 0)
#define INT_RAW_STS_R4P0	(1 << 1)
#define INT_MASK_STS_R4P0	(1 << 2)
#define INT_CLR_R4P0		(1 << 3)

#define VALUE_SHDW_LO_R4P0	(0x0018)
#define VALUE_SHDW_HI_R4P0	(0x001C)

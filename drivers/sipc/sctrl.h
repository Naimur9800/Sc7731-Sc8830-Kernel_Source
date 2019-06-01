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

#ifndef __SCTRL_H
#define __SCTRL_H

/* flag for CMD/DONE msg type */
#define SMSG_CMD_SBUF_INIT	0x0001
#define SMSG_DONE_SBUF_INIT	0x0002

/* flag for EVENT msg type */
#define SMSG_EVENT_SBUF_WRPTR	0x0001
#define SMSG_EVENT_SBUF_RDPTR	0x0002
#define SCTL_STATE_IDLE		0
#define SCTL_STATE_READY	1

struct sctrl_mgr {
	uint8_t			dst;
	uint8_t			channel;
	uint32_t		state;
	struct task_struct	*thread;
};

#endif

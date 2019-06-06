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

#ifndef __SMSGC_H
#define __SMSGC_H

#define SMSGC_STATE_IDLE		0
#define SMSGC_STATE_READY		1

struct smsgc_init_data {
	char			*name;
	u8			dst;
	u8			channel;
};

struct smsgc_mgr {
	u8			dst;
	u8			channel;
	u32		state;

	struct mutex		txlock;
	struct mutex		rxlock;

	wait_queue_head_t	rxwait;

	/* txbuf ptr for smsgc */
	void 		*txbuf;
	size_t 		txmsg_cnt;

	/* cached msgs for smsgc */
	size_t		wrptr[1];
	size_t		rdptr[1];
	struct smsg 	caches[SMSG_CACHE_NR];

	struct task_struct	*thread;
};

#endif


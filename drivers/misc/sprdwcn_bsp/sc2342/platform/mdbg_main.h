/*
* Copyright (C) 2015 Spreadtrum Communications Inc.
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*/

#ifndef _MARLIN_DEBUG_H
#define _MARLIN_DEBUG_H

#include "mdbg_type.h"

#define MDBG_MAX_BUFFER_LEN (64*1024)
#define MDBG_WRITE_SIZE			(64)
#define MDBG_ASSERT_SIZE		(1024)
#define MDBG_LOOPCHECK_SIZE		(128)
#define MDBG_AT_CMD_SIZE		(128)
#define MDBG_SNAP_SHOOT_SIZE		(32*1024)

struct mdbg_device_t {
	int			open_count;
	struct mutex		mdbg_lock;
	wait_queue_head_t	rxwait;
};

extern wait_queue_head_t	mdbg_wait;
extern struct mdbg_device_t	*mdbg_dev;
extern int wcn_open_module;
extern int wcn_module_state_change;
extern unsigned char flag_download;
extern unsigned char flag_reset;
extern struct completion ge2_completion;
extern struct ring_device *ring_dev;
int mdbg_init(void);
void mdbg_exit(void);
void power_state_notify(bool state);
void open_mdbg_loopcheck_interru(void);
int get_loopcheck_status(void);
void marlin_hold_cpu(void);
extern void print_ring(void);
#endif

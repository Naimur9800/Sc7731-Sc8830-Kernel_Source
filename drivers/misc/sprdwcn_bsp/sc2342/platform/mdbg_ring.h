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

#ifndef _MDBG_RING_H
#define _MDBG_RING_H

#include <linux/mutex.h>
#include <linux/sched.h>
#include "mdbg_type.h"

#define MDBG_RING_R		0
#define MDBG_RING_W		1

#define MDBG_RING_MIN_SIZE	(1024 * 4)

#define MDBG_RING_LOCK_SIZE (sizeof(struct mutex))

struct mdbg_ring_t {
	long int size;
	char *pbuff;
	char *rp;
	char *wp;
	char *end;
	struct mutex *plock;
};

struct mdbg_ring_t *mdbg_ring_alloc(long int size);
void mdbg_ring_destroy(struct mdbg_ring_t *ring);
int mdbg_ring_read(struct mdbg_ring_t *ring, void *buf, int len);
int mdbg_ring_write(struct mdbg_ring_t *ring, void *buf, int len);
char *mdbg_ring_write_ext(struct mdbg_ring_t *ring, long int len);
bool mdbg_ring_will_full(struct mdbg_ring_t *ring, int len);
long int mdbg_ring_content_len(struct mdbg_ring_t *ring);
void mdbg_ring_clear(struct mdbg_ring_t *ring);
void mdbg_ring_reset(struct mdbg_ring_t *ring);
bool mdbg_ring_over_loop(struct mdbg_ring_t *ring, u_long len, int rw);
void mdbg_ring_print(struct mdbg_ring_t *ring);
#endif


/*
 * Copyright (C) 2017-2018 Spreadtrum Communications Inc.
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

#ifndef _CAM_QUEUE_H_
#define _CAM_QUEUE_H_

#include <linux/types.h>
#include <linux/spinlock.h>

#include "cam_buf.h"

#define CAM_EMP_Q_LEN_INC  16
#define CAM_EMP_Q_LEN_MAX 128

enum {
	CAM_Q_INIT,
	CAM_Q_EMPTY,
	CAM_Q_FULL,
	CAM_Q_CLEAR
};

struct camera_frame {
	struct list_head list;
	uint32_t fid;
	uint32_t width;
	uint32_t height;
	uint32_t evt;
	uint32_t channel_id;
	uint32_t irq_type;
	uint32_t irq_property;
	uint32_t is_reserved;
	void *priv_data;
	struct timeval time;
	ktime_t boot_time;
	struct camera_buf  buf;
};

struct camera_queue {
	uint32_t type;
	uint32_t state;
	uint32_t max;
	uint32_t cnt;
	spinlock_t lock;
	struct list_head head;
	void (*destroy)(void *param);
};

int camera_enqueue(struct camera_queue *q, struct camera_frame *pframe);
struct camera_frame *camera_dequeue(struct camera_queue *q);
struct camera_frame *camera_dequeue_tail(struct camera_queue *q);

int camera_queue_init(struct camera_queue *q,
			uint32_t max, uint32_t type,
			void (*cb_func)(void *));
int camera_queue_clear(struct camera_queue *q);

struct camera_frame *get_empty_frame(void);
int put_empty_frame(struct camera_frame *pframe);
void free_empty_frame(void *param);

#endif /* _CAM_QUEUE_H_ */

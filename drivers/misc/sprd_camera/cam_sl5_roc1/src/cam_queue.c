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

#include <linux/types.h>
#include <linux/list.h>
#include <linux/slab.h>

#include "cam_types.h"
#include "cam_buf.h"
#include "cam_queue.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "cam_queue: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__


int camera_enqueue(struct camera_queue *q, struct camera_frame *pframe)
{
	int ret = 0;
	unsigned long flags;

	if (q == NULL || pframe == NULL) {
		pr_err("error: input ptr is NULL\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&q->lock, flags);
	if (q->state == CAM_Q_CLEAR) {
		pr_err("error: q is not inited.\n");
		ret = -EPERM;
		goto unlock;
	}

	if (q->cnt >= q->max) {
		pr_debug("warn: queue full %d\n", q->cnt);
		ret = -EPERM;
		goto unlock;
	}
	q->cnt++;
	list_add_tail(&pframe->list, &q->head);

unlock:
	spin_unlock_irqrestore(&q->lock, flags);

	return ret;
}

struct camera_frame *camera_dequeue(struct camera_queue *q)
{
	int fatal_err;
	unsigned long flags;
	struct camera_frame *pframe = NULL;

	if (q == NULL) {
		pr_err("error: input ptr is NULL\n");
		return NULL;
	}

	spin_lock_irqsave(&q->lock, flags);
	if (q->state == CAM_Q_CLEAR) {
		pr_err("error: q is not inited.\n");
		goto unlock;
	}

	if (list_empty(&q->head) || q->cnt == 0) {
		pr_debug("queue empty %d, %d\n",
			list_empty(&q->head), q->cnt);
		fatal_err = (list_empty(&q->head) ^ (q->cnt == 0));
		if (fatal_err)
			pr_debug("error:  empty %d, cnt %d\n",
					list_empty(&q->head), q->cnt);
		goto unlock;
	}

	pframe = list_first_entry(&q->head, struct camera_frame, list);
	list_del(&pframe->list);
	q->cnt--;
unlock:
	spin_unlock_irqrestore(&q->lock, flags);

	return pframe;
}

/* dequeue frame from tail of queue */
struct camera_frame *camera_dequeue_tail(struct camera_queue *q)
{
	int fatal_err;
	unsigned long flags;
	struct camera_frame *pframe = NULL;

	if (q == NULL) {
		pr_err("error: input ptr is NULL\n");
		return NULL;
	}

	spin_lock_irqsave(&q->lock, flags);
	if (q->state == CAM_Q_CLEAR) {
		pr_err("error: q is not inited.\n");
		goto unlock;
	}

	if (list_empty(&q->head) || q->cnt == 0) {
		pr_debug("queue empty %d, %d\n",
			list_empty(&q->head), q->cnt);
		fatal_err = (list_empty(&q->head) ^ (q->cnt == 0));
		if (fatal_err)
			pr_debug("error:  empty %d, cnt %d\n",
					list_empty(&q->head), q->cnt);
		goto unlock;
	}

	pframe = list_last_entry(&q->head, struct camera_frame, list);
	list_del(&pframe->list);
	q->cnt--;
unlock:
	spin_unlock_irqrestore(&q->lock, flags);

	return pframe;
}

int camera_queue_init(struct camera_queue *q,
			uint32_t max, uint32_t type,
			void (*cb_func)(void *))
{
	int ret = 0;

	if (q == NULL) {
		pr_err("error: input ptr is NULL\n");
		return -EINVAL;
	}
	q->cnt = 0;
	q->max = max;
	q->type = type;
	q->state = CAM_Q_INIT;
	q->destroy = cb_func;
	spin_lock_init(&q->lock);
	INIT_LIST_HEAD(&q->head);

	return ret;
}


int camera_queue_clear(struct camera_queue *q)
{
	int ret = 0;
	int fatal_err;
	unsigned long flags;
	struct camera_frame *pframe = NULL;

	if (q == NULL) {
		pr_err("error: input ptr is NULL\n");
		return -EINVAL;
	}
	spin_lock_irqsave(&q->lock, flags);

	do {
		if (list_empty(&q->head) || q->cnt == 0) {
			pr_debug("queue empty %d, %d\n",
				list_empty(&q->head), q->cnt);
			fatal_err = (list_empty(&q->head) ^ (q->cnt == 0));
			if (fatal_err)
				pr_debug("error:  empty %d, cnt %d\n",
						list_empty(&q->head), q->cnt);
			break;
		}
		pframe = list_first_entry(&q->head, struct camera_frame, list);
		list_del(&pframe->list);
		q->cnt--;
		if (pframe == NULL)
			break;

		if (q->destroy) {
			spin_unlock_irqrestore(&q->lock, flags);
			q->destroy(pframe);
			spin_lock_irqsave(&q->lock, flags);
		}
	} while (1);

	q->cnt = 0;
	q->max = 0;
	q->type = 0;
	q->state = CAM_Q_CLEAR;
	q->destroy = NULL;
	INIT_LIST_HEAD(&q->head);

	spin_unlock_irqrestore(&q->lock, flags);

	return ret;
}


struct camera_frame *get_empty_frame(void)
{
	int ret = 0;
	uint32_t i;
	struct camera_queue *q = g_empty_frm_q;
	struct camera_frame *pframe = NULL;

	pr_debug("Enter.\n");
	do {
		pframe = camera_dequeue(q);
		if (pframe == NULL) {
			for (i = 0; i < CAM_EMP_Q_LEN_INC; i++) {
				pframe = kzalloc(sizeof(*pframe), GFP_KERNEL);
				if (pframe) {
					pr_debug("alloc frame %p\n", pframe);
					ret = camera_enqueue(q, pframe);
					if (ret) {
						pr_err("fail to in empty q.  %p\n", pframe);
						kfree(pframe);
						pframe = NULL;
						break;
					}
					pframe = NULL;
					atomic_inc(&g_mem_dbg->empty_frm_cnt);
				} else {
					pr_err("error: no memory.\n");
					break;
				}
			}
			pr_info("alloc %d empty frames\n", i);
		}
	} while (pframe == NULL);

	pr_debug("Done. get frame %p\n", pframe);
	return pframe;
}


int put_empty_frame(struct camera_frame *pframe)
{
	int ret = 0;

	if (pframe == NULL) {
		pr_err("error: null input.\n");
		return -EINVAL;
	}

	memset(pframe, 0, sizeof(struct camera_frame));
	ret = camera_enqueue(g_empty_frm_q, pframe);
	if (ret) {
		pr_err("fail to input frame to empty queue.\n");
		ret = -EINVAL;
	}
	pr_debug("put frame %p\n", pframe);

	return ret;
}

void free_empty_frame(void *param)
{
	struct camera_frame *pframe;

	pframe = (struct camera_frame *)param;
	pr_debug("free frame %p\n", pframe);
	atomic_dec(&g_mem_dbg->empty_frm_cnt);
	kfree(pframe);
}


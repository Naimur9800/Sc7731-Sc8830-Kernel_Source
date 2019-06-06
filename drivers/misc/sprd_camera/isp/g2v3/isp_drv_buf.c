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

#include <linux/fs.h>
#include <linux/uaccess.h>
#include <video/sprd_isp_altek.h>
#include "altek_isp/altek_isp_drv.h"
#include "isp_drv.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "[isp_drv_buff]: %d: line=%d: " fmt, current->pid, __LINE__

int32_t _isp_img_buf_queue_init(struct isp_img_buf_queue *queue, uint8_t *qname)
{
	if (queue == NULL) {
		pr_err("queue is null error.\n");
		return -EINVAL;
	}

	memset(queue, 0x0, sizeof(struct isp_img_buf_queue));
	strcpy(queue->name, qname);
	queue->write = &queue->buff[0];
	queue->read = &queue->buff[0];
	queue->num = 0;
	spin_lock_init(&queue->lock);

	return 0;
}

int32_t _isp_img_buf_queue_write(struct isp_img_buf_queue *queue,
		struct isp_img_buf *buf)
{
	struct isp_img_buf *ori_buf = NULL;
	unsigned long flag;

	if (queue == NULL || buf == NULL) {
		pr_err("queue or buff is null.\n");
		return -EINVAL;
	}
	spin_lock_irqsave(&queue->lock, flag);

	ori_buf = queue->write;
	*queue->write++ = *buf;
	queue->num++;

	if (queue->write > &queue->buff[ISP_IMG_QUEUE_LEN - 1])
		queue->write = &queue->buff[0];

	if (queue->write == queue->read) {
		queue->write = ori_buf;
		queue->num--;
		pr_err("Warning,[%s] is full.\n", queue->name);
	}

	spin_unlock_irqrestore(&queue->lock, flag);

	return 0;
}

int32_t _isp_img_buf_queue_read(struct isp_img_buf_queue *queue,
		struct isp_img_buf *buf)
{
	unsigned long flag;

	if (queue == NULL || buf == NULL) {
		pr_err("queue or buf is null error.\n");
		return -EAGAIN;
	}

	spin_lock_irqsave(&queue->lock, flag);
	if (queue->read != queue->write) {
		*buf = *queue->read++;
		queue->num--;
		if (queue->read > &queue->buff[ISP_IMG_QUEUE_LEN - 1])
			queue->read = &queue->buff[0];
	} else {

		spin_unlock_irqrestore(&queue->lock, flag);
		return -EAGAIN;
	}
	spin_unlock_irqrestore(&queue->lock, flag);

	return 0;
}

uint32_t _isp_img_buf_queue_nodenum(struct isp_img_buf_queue *queue)
{
	unsigned long flag;
	uint32_t num;

	if (queue == NULL) {
		pr_err("queue is null error.\n");
		return -EAGAIN;
	}

	spin_lock_irqsave(&queue->lock, flag);
	num = queue->num;
	spin_unlock_irqrestore(&queue->lock, flag);
	return num;
}

int32_t _isp_img_frame_queue_init(struct isp_img_frame_queue *queue)
{
	if (queue == NULL) {
		pr_err("queue is null error.\n");
		return -EINVAL;
	}

	memset(queue, 0x0, sizeof(struct isp_img_buf_queue));
	queue->write = &queue->buff[0];
	queue->read = &queue->buff[0];
	spin_lock_init(&queue->lock);

	return 0;
}

int32_t _isp_img_frame_queue_write(struct isp_img_frame_queue *queue,
		struct isp_img_frame *buf)
{
	struct isp_img_frame *ori_buf = NULL;
	unsigned long flag;

	if (queue == NULL || buf == NULL) {
		pr_err("queue or buf is null error.\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&queue->lock, flag);

	ori_buf = queue->write;
	*queue->write++ = *buf;

	if (queue->write > &queue->buff[ISP_IMG_QUEUE_LEN - 1])
		queue->write = &queue->buff[0];

	if (queue->write == queue->read) {
		queue->write = ori_buf;
		pr_err("Warning,image frame queue is full.\n");
	}

	spin_unlock_irqrestore(&queue->lock, flag);

	return 0;
}

int32_t _isp_img_frame_queue_read(struct isp_img_frame_queue *queue,
		struct isp_img_frame *buf)
{
	unsigned long flag;

	if (queue == NULL || buf == NULL) {
		pr_err("queue or buf is null error.\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&queue->lock, flag);
	if (queue->read != queue->write) {
		*buf = *queue->read++;

		if (queue->read > &queue->buff[ISP_IMG_QUEUE_LEN - 1])
			queue->read = &queue->buff[0];
	} else {
		spin_unlock_irqrestore(&queue->lock, flag);
		return -EAGAIN;
	}
	spin_unlock_irqrestore(&queue->lock, flag);

	return 0;
}

int32_t _isp_img_buf_queue_unread(struct isp_img_buf_queue *queue)
{
	unsigned long flag;

	if (queue == NULL) {
		pr_err("queue or buf is null error.\n");
		return -EAGAIN;
	}

	spin_lock_irqsave(&queue->lock, flag);
	queue->read--;
	queue->num++;
	if (queue->read < &queue->buff[0])
		queue->read = &queue->buff[ISP_IMG_QUEUE_LEN - 1];

	spin_unlock_irqrestore(&queue->lock, flag);

	return 0;
}


int32_t _isp_statis_queue_init(struct img_statis_buf_queue *queue)
{
	if (queue == NULL) {
		pr_err("queue is null error.\n");
		return -EINVAL;
	}

	memset(queue, 0x0, sizeof(struct img_statis_buf_queue));
	queue->write = &queue->buff[0];
	queue->read = &queue->buff[0];
	spin_lock_init(&queue->lock);

	return 0;
}

int32_t _isp_statis_queue_write(struct img_statis_buf_queue *queue,
		struct img_statis_frame *buf)
{
	struct img_statis_frame *ori_buf = NULL;
	unsigned long flag;

	if (queue == NULL || buf == NULL) {
		pr_err("queue is null error.\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&queue->lock, flag);

	ori_buf = queue->write;
	*queue->write++ = *buf;

	if (queue->write > &queue->buff[ISP_STATISTICS_QUEUE_LEN - 1])
		queue->write = &queue->buff[0];

	if (queue->write == queue->read) {
		queue->write = ori_buf;
		pr_err("Warning,isp_img_queue is full.\n");
	}

	spin_unlock_irqrestore(&queue->lock, flag);

	return 0;
}

int32_t _isp_statis_queue_read(struct img_statis_buf_queue *queue,
		struct img_statis_frame *buf)
{
	unsigned long flag;

	if (queue == NULL || buf == NULL) {
		pr_err("queue or buff is null error.\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&queue->lock, flag);
	if (queue->read != queue->write) {
		*buf = *queue->read++;
		if (queue->read > &queue->buff[ISP_STATISTICS_QUEUE_LEN - 1])
			queue->read = &queue->buff[0];
	} else {
		spin_unlock_irqrestore(&queue->lock, flag);
		return -EAGAIN;
	}
	spin_unlock_irqrestore(&queue->lock, flag);

	return 0;
}

int32_t _isp_irq_queue_init(struct isp_queue *queue)
{
	if (queue == NULL) {
		pr_err("queue is null error.\n");
		return -EINVAL;
	}

	memset(queue, 0x00, sizeof(*queue));
	queue->write = &queue->node[0];
	queue->read = &queue->node[0];

	spin_lock_init(&queue->lock);

	return 0;
}

int32_t _isp_irq_queue_write(struct isp_queue *queue, struct isp_irq_info *node)
{
	unsigned long flag;
	struct isp_irq_info *ori_node = NULL;

	if (queue == NULL || node == NULL) {
		pr_err("queue or node is null error %p %p\n", queue, node);
		return -EINVAL;
	}

	spin_lock_irqsave(&queue->lock, flag);
	ori_node = queue->write;

	*queue->write++ = *node;
	if (queue->write > &queue->node[ISP_QUEUE_LENGTH - 1])
		queue->write = &queue->node[0];

	if (queue->write == queue->read)
		queue->write = ori_node;

	spin_unlock_irqrestore(&queue->lock, flag);

	return 0;
}

int32_t _isp_irq_queue_read(struct isp_queue *queue, struct isp_irq_info *node)
{
	unsigned long flag;

	if (queue == NULL || node == NULL) {
		pr_err("queue or node is null error %p %p\n", queue, node);
		return -EINVAL;
	}

	spin_lock_irqsave(&queue->lock, flag);

	if (queue->read != queue->write) {
		*node = *queue->read++;
		if (queue->read > &queue->node[ISP_QUEUE_LENGTH - 1])
			queue->read = &queue->node[0];
	}

	spin_unlock_irqrestore(&queue->lock, flag);

	return 0;
}

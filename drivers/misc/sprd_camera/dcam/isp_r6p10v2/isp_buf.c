/*
 * Copyright (C) 2016 Spreadtrum Communications Inc.
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

#include "isp_buf.h"
#include "sprd_k_cpp.h"
#include <linux/sprd_ion.h>
#include <linux/sprd_iommu.h>

#define ION
#ifdef ION
#include "ion.h"
#include "ion_priv.h"
#endif

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_BUF: %d " fmt, __LINE__

#define STATIS_QUEUE_LENGTH		8
#define ISP_IMG_QUEUE_LEN		8
#define ISP_IRQ_DONE_LENGTH		8

#define abs64(x) ({ \
s64 __x = (x); \
(__x < 0) ? -__x : __x; \
})

static int irq_done_num[ISP_ID_MAX][IRQ_ALL_DONE];
static spinlock_t irq_done_lock[ISP_ID_MAX][IRQ_ALL_DONE];
int isp_irq_done_init(enum isp_id isp_idx)
{
	int i;

	for (i = 0; i < IRQ_ALL_DONE; i++) {
		irq_done_num[isp_idx][i] = 0;
		spin_lock_init(&irq_done_lock[isp_idx][i]);
	}

	return 0;
}

int isp_irq_done_reset(enum isp_id isp_idx)
{
	int i;

	for (i = 0; i < IRQ_ALL_DONE; i++) {
		irq_done_num[isp_idx][i] = 0;
		pr_debug("isp %d, irq%d", isp_idx, i);
	}

	return 0;
}

int isp_irq_done_read(enum isp_id isp_idx, enum isp_irq_done_id idx)
{
	unsigned long flag;

	if (idx < IRQ_ALL_DONE) {
		spin_lock_irqsave(&irq_done_lock[isp_idx][idx], flag);
		if (irq_done_num[isp_idx][idx] > 0) {
			irq_done_num[isp_idx][idx]--;
		} else {
			pr_debug("irq done isp %d irq %d is empty\n",
				isp_idx, idx);
		}
		spin_unlock_irqrestore(&irq_done_lock[isp_idx][idx], flag);
	}

	return 0;
}

int isp_irq_done_write(enum isp_id isp_idx, enum isp_irq_done_id idx)
{
	unsigned long flag;
	int ret = 0;

	if (idx < IRQ_ALL_DONE) {
		spin_lock_irqsave(&irq_done_lock[isp_idx][idx], flag);
		if (irq_done_num[isp_idx][idx] < ISP_IRQ_DONE_LENGTH) {
			ret = ISP_IRQ_DONE_LENGTH
				- irq_done_num[isp_idx][idx]++;
		} else {
			pr_debug("irq done isp %d irq %d is full\n",
				isp_idx, idx);
			ret = 0;
		}
		spin_unlock_irqrestore(&irq_done_lock[isp_idx][idx], flag);
	}

	return ret;
}

int isp_statis_queue_init(struct isp_statis_buf_queue *queue)
{
	if (queue == NULL) {
		pr_err("isp_statis_queue_init: queue is null error.\n");
		return -EINVAL;
	}

	memset(queue, 0x0, sizeof(struct isp_statis_buf_queue));
	queue->write = &queue->buff[0];
	queue->read = &queue->buff[0];
	spin_lock_init(&queue->lock);

	return 0;
}

int isp_statis_queue_read(struct isp_statis_buf_queue *queue,
	struct isp_statis_buf *buf)
{
	unsigned long flag;

	if (queue == NULL || buf == NULL) {
		pr_err("isp_statis_queue_read: queue is null error\n");
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

static int isp_statis_queue_write(struct isp_statis_buf_queue *queue,
				struct isp_statis_buf *buf)
{
	struct isp_statis_buf *ori_buf = NULL;
	unsigned long flag;

	if (queue == NULL || buf == NULL) {
		pr_err("isp_statis_queue_write: queue is null error.\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&queue->lock, flag);

	ori_buf = queue->write;
	*queue->write++ = *buf;

	if (queue->write > &queue->buff[ISP_IMG_QUEUE_LEN - 1])
		queue->write = &queue->buff[0];

	if (queue->write == queue->read) {
		queue->write = ori_buf;
		pr_err("isp_statis_queue_write:Warning, queue is full\n");
	}

	spin_unlock_irqrestore(&queue->lock, flag);

	return 0;
}

void isp_statis_frm_queue_clear(struct isp_statis_frm_queue *queue)
{
	if (ISP_ADDR_INVALID(queue)) {
		pr_err("invalid heap %p\n", queue);
		return;
	}

	memset((void *)queue, 0, sizeof(struct isp_statis_frm_queue));
}

void isp_statis_reset_all_queue(struct isp_pipe_dev *dev)
{
	struct isp_statis_module *module = NULL;

	if (dev == NULL) {
		pr_err("%s dev is null\n", __func__);
		return;
	}

	pr_info("%s in\n", __func__);
	module = &dev->statis_module_info;
	isp_irq_done_reset(dev->idx);

	isp_statis_queue_init(&module->aem_statis_queue);
	isp_statis_queue_init(&module->afl_statis_queue);
	isp_statis_queue_init(&module->afm_statis_queue);
	isp_statis_queue_init(&module->binning_statis_queue);
	isp_statis_queue_init(&module->pdaf_statis_queue);
	isp_statis_frm_queue_clear(&module->aem_statis_frm_queue);
	isp_statis_frm_queue_clear(&module->afl_statis_frm_queue);
	isp_statis_frm_queue_clear(&module->afm_statis_frm_queue);
	isp_statis_frm_queue_clear(&module->binning_statis_frm_queue);
	isp_statis_frm_queue_clear(&module->pdaf_statis_frm_queue);

}

int isp_statis_enqueue(struct isp_statis_frm_queue *queue,
	struct isp_statis_buf *frame)
{
	if (ISP_ADDR_INVALID(queue) || ISP_ADDR_INVALID(frame)) {
		pr_err("enq, invalid parm %p, %p\n", queue, frame);
		return -1;
	}

	if (queue->valid_cnt >= STATIS_QUEUE_LENGTH) {
		pr_debug("queue is full\n");
		return -1;
	}

	memcpy(&queue->buf_array[queue->valid_cnt], frame,
	       sizeof(struct isp_statis_buf));
	queue->valid_cnt++;
	pr_debug("en queue, %d, 0x%lx, 0x%lx\n",
		   queue->valid_cnt, frame->phy_addr, frame->vir_addr);

	return 0;
}

int isp_statis_dequeue(struct isp_statis_frm_queue *queue,
	struct isp_statis_buf *frame)
{
	unsigned int i = 0;

	if (ISP_ADDR_INVALID(queue) || ISP_ADDR_INVALID(frame)) {
		pr_err("deq, invalid parm %p, %p\n", queue, frame);
		return -1;
	}

	if (queue->valid_cnt == 0) {
		pr_debug("queue is empty\n");
		return -1;
	}

	memcpy(frame, &queue->buf_array[0], sizeof(struct isp_statis_buf));
	queue->valid_cnt--;
	for (i = 0; i < queue->valid_cnt; i++) {
		memcpy(&queue->buf_array[i], &queue->buf_array[i + 1],
		       sizeof(struct isp_statis_buf));
	}
	pr_debug("de queue, %d\n", queue->valid_cnt);

	return 0;
}

int sprd_isp_cfg_statis_buf(struct isp_pipe_dev *dev,
	struct isp_statis_buf_input *parm)
{
	int ret = ISP_RTN_SUCCESS;
	int cnt = 0;
	uint32_t aem_iova_addr = 0, aem_vir_addr = 0;
	uint64_t aem_kaddr = 0;
	uint32_t afl_iova_addr = 0, afl_vir_addr = 0;
	uint64_t afl_kaddr = 0;
	uint32_t afm_iova_addr = 0, afm_vir_addr = 0;
	uint64_t afm_kaddr = 0;
	uint32_t pdaf_iova_addr = 0, pdaf_vir_addr = 0;
	uint64_t pdaf_kaddr = 0;
	uint32_t binning_iova_addr = 0, binning_vir_addr = 0;
	uint64_t binning_kaddr = 0;
	struct isp_statis_buf frm_statis;
	struct isp_statis_buf aem_frm_statis;
	struct isp_statis_buf afl_frm_statis;
	struct isp_statis_buf afm_frm_statis;
	struct isp_statis_buf pdaf_frm_statis;
	struct isp_statis_buf binning_frm_statis;
	struct isp_statis_module *module = NULL;
	size_t statis_mem_size = 0;

	module = &dev->statis_module_info;

	pr_info("cfg statis buf in.\n");
	isp_statis_reset_all_queue(dev);
	memset((void *)&module->img_statis_buf, 0,
	       sizeof(struct isp_statis_buf));
	memset((void *)&frm_statis, 0, sizeof(struct isp_statis_buf));

	memset((void *)&aem_frm_statis, 0, sizeof(struct isp_statis_buf));
	memset((void *)&afl_frm_statis, 0, sizeof(struct isp_statis_buf));
	memset((void *)&afm_frm_statis, 0, sizeof(struct isp_statis_buf));
	memset((void *)&pdaf_frm_statis, 0, sizeof(struct isp_statis_buf));
	memset((void *)&binning_frm_statis, 0, sizeof(struct isp_statis_buf));
	frm_statis.phy_addr = parm->phy_addr;
	frm_statis.vir_addr = parm->vir_addr;
	frm_statis.buf_size = parm->buf_size;
	frm_statis.buf_property = parm->buf_property;

	if (dev->idx == ISP_ID_0)
		frm_statis.pfinfo.dev = &s_isp_pdev->dev;
	else
		frm_statis.pfinfo.dev = &s_isp1_pdev->dev;
	frm_statis.pfinfo.mfd[0] = parm->mfd;
	/*mapping iommu buffer*/
	ret = pfiommu_get_sg_table(&frm_statis.pfinfo);
	if (ret) {
		pr_err("cfg statis buf addr failed.\n");
		ret = -1;
		return ret;
	}

	ret = pfiommu_get_addr(&frm_statis.pfinfo);
	if (ret) {
		pr_err("cfg statis buf addr failed: iommu mapping error.\n");
		ret = -1;
		return ret;
	}

	memcpy(&module->img_statis_buf, &frm_statis,
	       sizeof(struct isp_statis_buf));
	aem_iova_addr = frm_statis.pfinfo.iova[0];
	aem_vir_addr = parm->vir_addr;
	aem_kaddr =
		(uint64_t)parm->kaddr[0] | ((uint64_t)parm->kaddr[1] << 32);
	pr_info("statis kaddr[0]=0x%x, kaddr[1]= 0x%x, aem: 0x%llx\n",
			parm->kaddr[0],
			parm->kaddr[1],
			aem_kaddr);
	statis_mem_size = frm_statis.pfinfo.size[0];
	/*split the big buffer to some little buffer*/
	for (cnt = 0; cnt < ISP_AEM_STATIS_BUF_NUM; cnt++) {
		aem_frm_statis.phy_addr = aem_iova_addr;
		aem_frm_statis.vir_addr = aem_vir_addr;
		aem_frm_statis.kaddr[0] = aem_kaddr;
		aem_frm_statis.kaddr[1] = aem_kaddr >> 32;
		aem_frm_statis.pfinfo.mfd[0] = frm_statis.pfinfo.mfd[0];
		aem_frm_statis.buf_size = ISP_AEM_STATIS_BUF_SIZE;
		aem_frm_statis.buf_property = ISP_AEM_BLOCK;

		ret = isp_statis_queue_write(&module->aem_statis_queue,
					     &aem_frm_statis);
		aem_iova_addr += ISP_AEM_STATIS_BUF_SIZE;
		aem_vir_addr += ISP_AEM_STATIS_BUF_SIZE;
		aem_kaddr += ISP_AEM_STATIS_BUF_SIZE;
	}
	/*init reserved aem statis buf*/
	module->aem_buf_reserved.phy_addr = aem_iova_addr;
	module->aem_buf_reserved.vir_addr = aem_vir_addr;
	module->aem_buf_reserved.kaddr[0] = aem_kaddr;
	module->aem_buf_reserved.kaddr[1] = aem_kaddr >> 32;
	module->aem_buf_reserved.pfinfo.mfd[0] = frm_statis.pfinfo.mfd[0];
	module->aem_buf_reserved.buf_size = ISP_AEM_STATIS_BUF_SIZE;
	module->aem_buf_reserved.buf_property = ISP_AEM_BLOCK;

	/*afl statis buf cfg*/
	afl_iova_addr = aem_iova_addr + ISP_AEM_STATIS_BUF_SIZE;
	afl_vir_addr = aem_vir_addr + ISP_AEM_STATIS_BUF_SIZE;
	afl_kaddr = aem_kaddr + ISP_AEM_STATIS_BUF_SIZE;
	/*split the big buffer to some little buffer*/
	for (cnt = 0; cnt < ISP_AFL_STATIS_BUF_NUM; cnt++) {
		afl_frm_statis.phy_addr = afl_iova_addr;
		afl_frm_statis.vir_addr = afl_vir_addr;
		afl_frm_statis.kaddr[0] = afl_kaddr;
		afl_frm_statis.kaddr[1] = afl_kaddr >> 32;
		afl_frm_statis.pfinfo.mfd[0] = frm_statis.pfinfo.mfd[0];
		afl_frm_statis.buf_size = ISP_AFL_STATIS_BUF_SIZE;
		afl_frm_statis.buf_property = ISP_AFL_BLOCK;
		ret = isp_statis_queue_write(&module->afl_statis_queue,
					     &afl_frm_statis);
		afl_iova_addr += ISP_AFL_STATIS_BUF_SIZE;
		afl_vir_addr += ISP_AFL_STATIS_BUF_SIZE;
		afl_kaddr += ISP_AFL_STATIS_BUF_SIZE;
	}
	/*init reserved afl statis buf*/
	module->afl_buf_reserved.phy_addr = afl_iova_addr;
	module->afl_buf_reserved.vir_addr = afl_vir_addr;
	module->afl_buf_reserved.kaddr[0] = afl_kaddr;
	module->afl_buf_reserved.kaddr[1] = afl_kaddr >> 32;
	module->afl_buf_reserved.pfinfo.mfd[0] = frm_statis.pfinfo.mfd[0];
	module->afl_buf_reserved.buf_size = ISP_AFL_STATIS_BUF_SIZE;
	module->afl_buf_reserved.buf_property = ISP_AFL_BLOCK;

	/*afm statis buf cfg*/
	afm_iova_addr = afl_iova_addr + ISP_AFL_STATIS_BUF_SIZE;
	afm_vir_addr = afl_vir_addr + ISP_AFL_STATIS_BUF_SIZE;
	afm_kaddr = afl_kaddr + ISP_AFL_STATIS_BUF_SIZE;
	/*slip the big buffer to some little buffer*/
	for (cnt = 0; cnt < ISP_AFM_STATIS_BUF_NUM; cnt++) {
		afm_frm_statis.phy_addr = afm_iova_addr;
		afm_frm_statis.vir_addr = afm_vir_addr;
		afm_frm_statis.kaddr[0] = afm_kaddr;
		afm_frm_statis.kaddr[1] = afm_kaddr >> 32;
		afm_frm_statis.pfinfo.mfd[0] = frm_statis.pfinfo.mfd[0];
		afm_frm_statis.buf_size = ISP_AFM_STATIS_BUF_SIZE;
		afm_frm_statis.buf_property = ISP_AFM_BLOCK;
		ret = isp_statis_queue_write(&module->afm_statis_queue,
					     &afm_frm_statis);
		afm_iova_addr += ISP_AFM_STATIS_BUF_SIZE;
		afm_vir_addr += ISP_AFM_STATIS_BUF_SIZE;
		afm_kaddr += ISP_AFM_STATIS_BUF_SIZE;
	}
	/*init reserved afl statis buf*/
	module->afm_buf_reserved.phy_addr = afm_iova_addr;
	module->afm_buf_reserved.vir_addr = afm_vir_addr;
	module->afm_buf_reserved.kaddr[0] = afm_kaddr;
	module->afm_buf_reserved.kaddr[1] = afm_kaddr >> 32;
	module->afm_buf_reserved.pfinfo.mfd[0] = frm_statis.pfinfo.mfd[0];
	module->afm_buf_reserved.buf_size = ISP_AFM_STATIS_BUF_SIZE;
	module->afm_buf_reserved.buf_property = ISP_AFM_BLOCK;

	/*pdaf statis buf cfg*/
	pdaf_iova_addr = afm_iova_addr + ISP_AFM_STATIS_BUF_SIZE;
	pdaf_vir_addr = afm_vir_addr + ISP_AFM_STATIS_BUF_SIZE;
	pdaf_kaddr = afm_kaddr + ISP_AFM_STATIS_BUF_SIZE;
	/*slip the big buffer to some little buffer*/
	for (cnt = 0; cnt < ISP_PDAF_STATIS_BUF_NUM; cnt++) {
		pdaf_frm_statis.phy_addr = pdaf_iova_addr;
		pdaf_frm_statis.vir_addr = pdaf_vir_addr;
		pdaf_frm_statis.kaddr[0] = pdaf_kaddr;
		pdaf_frm_statis.kaddr[1] = pdaf_kaddr >> 32;
		pdaf_frm_statis.pfinfo.mfd[0] = frm_statis.pfinfo.mfd[0];
		pdaf_frm_statis.buf_size = ISP_PDAF_STATIS_BUF_SIZE;
		pdaf_frm_statis.buf_property = ISP_PDAF_BLOCK;
		ret = isp_statis_queue_write(&module->pdaf_statis_queue,
					     &pdaf_frm_statis);
		pdaf_iova_addr += ISP_PDAF_STATIS_BUF_SIZE;
		pdaf_vir_addr += ISP_PDAF_STATIS_BUF_SIZE;
		pdaf_kaddr += ISP_PDAF_STATIS_BUF_SIZE;
	}
	/*init reserved pdaf statis buf*/
	module->pdaf_buf_reserved.phy_addr = pdaf_iova_addr;
	module->pdaf_buf_reserved.vir_addr = pdaf_vir_addr;
	module->pdaf_buf_reserved.kaddr[0] = pdaf_kaddr;
	module->pdaf_buf_reserved.kaddr[1] = pdaf_kaddr >> 32;
	module->pdaf_buf_reserved.pfinfo.mfd[0] = frm_statis.pfinfo.mfd[0];
	module->pdaf_buf_reserved.buf_size = ISP_PDAF_STATIS_BUF_SIZE;
	module->pdaf_buf_reserved.buf_property = ISP_PDAF_BLOCK;

	/*binning statis buf cfg*/
	binning_iova_addr = pdaf_iova_addr + ISP_PDAF_STATIS_BUF_SIZE;
	binning_vir_addr = pdaf_vir_addr + ISP_PDAF_STATIS_BUF_SIZE;
	binning_kaddr = pdaf_kaddr + ISP_PDAF_STATIS_BUF_SIZE;
	/*slip the big buffer to some little buffer*/
	for (cnt = 0; cnt < ISP_BINNING_STATIS_BUF_NUM; cnt++) {
		binning_frm_statis.phy_addr = binning_iova_addr;
		binning_frm_statis.vir_addr = binning_vir_addr;
		binning_frm_statis.kaddr[0] = binning_kaddr;
		binning_frm_statis.kaddr[1] = binning_kaddr >> 32;
		binning_frm_statis.pfinfo.mfd[0] = frm_statis.pfinfo.mfd[0];
		binning_frm_statis.buf_size = ISP_BINNING_STATIS_BUF_SIZE;
		binning_frm_statis.buf_property = ISP_BINNING_BLOCK;
		ret = isp_statis_queue_write(&module->binning_statis_queue,
					     &binning_frm_statis);
		binning_iova_addr += ISP_BINNING_STATIS_BUF_SIZE;
		binning_vir_addr += ISP_BINNING_STATIS_BUF_SIZE;
		binning_kaddr += ISP_BINNING_STATIS_BUF_SIZE;
	}
	/*init reserved binning statis buf*/
	module->binning_buf_reserved.phy_addr = binning_iova_addr;
	module->binning_buf_reserved.vir_addr = binning_vir_addr;
	module->binning_buf_reserved.kaddr[0] = binning_kaddr;
	module->binning_buf_reserved.kaddr[1] = binning_kaddr;
	module->binning_buf_reserved.pfinfo.mfd[0] = frm_statis.pfinfo.mfd[0];
	module->binning_buf_reserved.buf_size = ISP_BINNING_STATIS_BUF_SIZE;
	module->binning_buf_reserved.buf_property = ISP_BINNING_BLOCK;

	pr_debug("cfg statis buf out.\n");

	return ret;
}

int sprd_isp_set_statis_addr(struct isp_pipe_dev *dev,
	struct isp_statis_buf_input *parm)
{
	int ret = 0;
	struct isp_statis_buf frm_statis;
	struct isp_statis_buf *statis_buf_reserved = NULL;
	struct isp_statis_module *module = NULL;
	struct isp_statis_buf_queue *statis_queue = NULL;

	module = &dev->statis_module_info;

	switch (parm->buf_property) {
	case ISP_AEM_BLOCK:
		statis_queue = &module->aem_statis_queue;
		statis_buf_reserved = &module->aem_buf_reserved;
		break;
	case ISP_AFL_BLOCK:
		statis_queue = &module->afl_statis_queue;
		statis_buf_reserved = &module->afl_buf_reserved;
		break;
	case ISP_AFM_BLOCK:
		statis_queue = &module->afm_statis_queue;
		statis_buf_reserved = &module->afm_buf_reserved;
		break;
	case ISP_BINNING_BLOCK:
		statis_queue = &module->binning_statis_queue;
		statis_buf_reserved = &module->binning_buf_reserved;
		break;
	case ISP_PDAF_BLOCK:
		statis_queue = &module->pdaf_statis_queue;
		statis_buf_reserved = &module->pdaf_buf_reserved;
		break;
	default:
		pr_err("statis block %d is unsupported,\n", parm->buf_property);
		return -EFAULT;
	}

	/*config statis buf*/
	if (parm->is_statis_buf_reserved == 1) {
		statis_buf_reserved->phy_addr = parm->phy_addr;
		statis_buf_reserved->vir_addr = parm->vir_addr;
		statis_buf_reserved->kaddr[0] = parm->kaddr[0];
		statis_buf_reserved->kaddr[1] = parm->kaddr[1];
		statis_buf_reserved->buf_size = parm->buf_size;
		statis_buf_reserved->buf_property = parm->buf_property;

		if (dev->idx == ISP_ID_0)
			statis_buf_reserved->pfinfo.dev = &s_isp_pdev->dev;
		else
			statis_buf_reserved->pfinfo.dev = &s_isp1_pdev->dev;
		statis_buf_reserved->pfinfo.mfd[0] = parm->reserved[0];
	} else {
		memset((void *)&frm_statis, 0, sizeof(struct isp_statis_buf));
		frm_statis.phy_addr = parm->phy_addr;
		frm_statis.vir_addr = parm->vir_addr;
		frm_statis.kaddr[0] = parm->kaddr[0];
		frm_statis.kaddr[1] = parm->kaddr[1];
		frm_statis.buf_size = parm->buf_size;
		frm_statis.buf_property = parm->buf_property;

		if (dev->idx == ISP_ID_0)
			frm_statis.pfinfo.dev = &s_isp_pdev->dev;
		else
			frm_statis.pfinfo.dev = &s_isp1_pdev->dev;
		frm_statis.pfinfo.mfd[0] = parm->reserved[0];
		/*when the statis is running, we need not map again*/
		ret = isp_statis_queue_write(statis_queue, &frm_statis);

	}

	pr_debug("set statis buf addr done.\n");
	return ret;
}

void isp_buf_queue_init(struct isp_buf_queue *queue)
{
	if (ISP_ADDR_INVALID(queue)) {
		pr_err("invalid heap %p\n", queue);
		return;
	}

	memset((void *)queue, 0, sizeof(struct isp_buf_queue));
	queue->write = &queue->frame[0];
	queue->read = &queue->frame[0];
	spin_lock_init(&queue->lock);
}

int isp_frame_enqueue(struct isp_frm_queue *queue,
	struct camera_frame *frame)
{
	if (ISP_ADDR_INVALID(queue) || ISP_ADDR_INVALID(frame)) {
		pr_err("enq, invalid parm %p, %p\n", queue, frame);
		return -1;
	}

	if (queue->valid_cnt >= ISP_FRM_QUEUE_LENGTH) {
		pr_debug("queue is full\n");
		return -1;
	}

	memcpy(&queue->frm_array[queue->valid_cnt], frame,
	       sizeof(struct camera_frame));
	queue->valid_cnt++;
	pr_debug("en queue, %d, %d, 0x%x, 0x%x\n",
		   (0xF & frame->fid),
		   queue->valid_cnt, frame->yaddr, frame->uaddr);
	return 0;
}

int isp_frame_dequeue(struct isp_frm_queue *queue,
	struct camera_frame *frame)
{
	unsigned int i = 0;

	if (ISP_ADDR_INVALID(queue) || ISP_ADDR_INVALID(frame)) {
		pr_err("deq, invalid parm %p, %p\n", queue, frame);
		return -1;
	}

	if (queue->valid_cnt == 0) {
		pr_err("frame queue is empty\n");
		return -1;
	}
	memcpy(frame, &queue->frm_array[0], sizeof(struct camera_frame));
	queue->valid_cnt--;
	for (i = 0; i < queue->valid_cnt; i++) {
		memcpy(&queue->frm_array[i], &queue->frm_array[i + 1],
		       sizeof(struct camera_frame));
	}
	pr_debug("de queue, %d, %d\n",
		   (0xF & (frame)->fid), queue->valid_cnt);

	return 0;
}

int isp_frame_dequeue_withtime(struct isp_frm_queue *queue,
	struct camera_frame *frame,  int64_t timestamp)
{
	unsigned int i = 0;
	unsigned int frame_idx = 0;
	int64_t frame_dt = 0;
	int64_t min_frame_dt = 0;

	if (ISP_ADDR_INVALID(queue) || ISP_ADDR_INVALID(frame)) {
		pr_err("deq, invalid parm %p, %p\n", queue, frame);
		return -1;
	}

	if (queue->valid_cnt == 0) {
		pr_debug("queue is empty\n");
		return -1;
	}

	min_frame_dt = timestamp - queue->frm_array[0].boot_time.tv64;

	for (i = 1; i < queue->valid_cnt; i++) {
		frame_dt = timestamp - queue->frm_array[i].boot_time.tv64;

		if (abs64(frame_dt) < abs64(min_frame_dt)) {
			min_frame_dt = frame_dt;
			frame_idx = i;
		}
	}
	/*un irq case add trace*/
	pr_debug("timestamp=%lld,frm[%d]=%lld,dt=%lld,min_dt=%lld,frame_idx=%d\n",
		timestamp, frame_idx,
		queue->frm_array[frame_idx].boot_time.tv64,
		abs64(frame_dt)/1000000,
		abs64(min_frame_dt)/1000000,
		frame_idx);

	memcpy(frame, &queue->frm_array[frame_idx],
		sizeof(struct camera_frame));

	queue->valid_cnt--;

	for (i = frame_idx; i < queue->valid_cnt; i++)
		memcpy(&queue->frm_array[i], &queue->frm_array[i + 1],
			sizeof(struct camera_frame));

	pr_debug("de queue, %d, %d\n",
		   (0xF & (frame)->fid), queue->valid_cnt);

	return 0;
}

void isp_frm_queue_clear(struct isp_frm_queue *queue)
{
	if (ISP_ADDR_INVALID(queue)) {
		pr_err("invalid heap %p\n", queue);
		return;
	}

	memset((void *)queue, 0, sizeof(struct isp_frm_queue));
}

int isp_queue_init(struct isp_queue *queue)
{
	if (queue == NULL)
		return -EINVAL;

	memset(queue, 0x00, sizeof(*queue));
	queue->write = &queue->node[0];
	queue->read  = &queue->node[0];

	return 0;
}

int32_t isp_queue_read(struct isp_queue *queue, struct isp_node *node)
{
	if (NULL == queue || NULL == node) {
		pr_err("isp_queue_read: queue or node is null error %p %p\n",
			queue, node);
		return -1;
	}

	if (queue->read != queue->write) {
		*node = *queue->read++;
		if (queue->read > &queue->node[ISP_QUEUE_LENGTH-1])
			queue->read = &queue->node[0];
	}

	return 0;
}

int isp_buf_queue_read(struct isp_buf_queue *queue,
	struct camera_frame *frame)
{
	int ret = ISP_RTN_SUCCESS;
	unsigned long flags;

	if (ISP_ADDR_INVALID(queue) || ISP_ADDR_INVALID(frame)) {
		pr_err("deq, invalid parm %p, %p\n", queue, frame);
		return -EINVAL;
	}

	pr_debug("read buf queue\n");

	spin_lock_irqsave(&queue->lock, flags);
	if (queue->read != queue->write) {
		*frame = *queue->read++;
		queue->r_index++;
		if (queue->read > &queue->frame[DCAM_FRM_CNT_MAX - 1]) {
			queue->read = &queue->frame[0];
			queue->r_index = 0;
		}
	} else {
		ret = -EAGAIN;
		pr_debug("warning, read wait new node write\n");
	}
	spin_unlock_irqrestore(&queue->lock, flags);

	pr_debug("read buf queue %d index %x\n",
		   frame->type, queue->r_index);

	return ret;
}

int isp_buf_queue_write(struct isp_buf_queue *queue,
	struct camera_frame *frame)
{
	int ret = ISP_RTN_SUCCESS;
	struct camera_frame *ori_frame;
	unsigned long flags;

	if (ISP_ADDR_INVALID(queue) || ISP_ADDR_INVALID(frame)) {
		pr_err("enq, invalid parm %p, %p\n", queue, frame);
		return -EINVAL;
	}

	pr_debug("write buf queue\n");
	spin_lock_irqsave(&queue->lock, flags);

	ori_frame = queue->write;
	*queue->write++ = *frame;
	queue->w_index++;
	if (queue->write > &queue->frame[DCAM_FRM_CNT_MAX - 1]) {
		queue->write = &queue->frame[0];
		queue->w_index = 0;
	}

	if (queue->write == queue->read) {
		queue->write = ori_frame;
		pr_warn("warning, queue is full, can't write 0x%x\n",
			frame->yaddr);
		ret = -EAGAIN;
	}
	spin_unlock_irqrestore(&queue->lock, flags);

	pr_debug("write buf queue type %d index %x\n",
		   frame->type, queue->w_index);

	return ret;
}

void isp_coeff_queue_init(struct isp_sc_coeff_queue *queue)
{
	int i = 0;

	if (ISP_ADDR_INVALID(queue)) {
		pr_err("invalid heap %p\n", queue);
		return;
	}
	for (i = 0; i < ISP_SC_COEFF_BUF_COUNT; i++) {
		memset((void *)(&queue->coeff[i]), 0,
			sizeof(struct isp_sc_coeff));
	}
	queue->write = &queue->coeff[0];
	queue->read = &queue->coeff[0];
	queue->w_index = 0;
	queue->r_index = 0;
	spin_lock_init(&queue->lock);
}

int isp_coeff_get_new_node(struct isp_sc_coeff_queue *queue,
			   struct isp_sc_coeff **coeff, int type)
{
	int ret = ISP_RTN_SUCCESS;
	unsigned long flags;
	struct isp_sc_coeff *ori;

	if (ISP_ADDR_INVALID(queue) || ISP_ADDR_INVALID(coeff)) {
		pr_err("deq, invalid parm %p %p\n", queue, coeff);
		return -EINVAL;
	}

	pr_debug("get new node\n");

	spin_lock_irqsave(&queue->lock, flags);

	*coeff = queue->write;
	ori = queue->write;
	if (type) {
		queue->write++;
		queue->w_index++;
		if (queue->write > &queue->coeff[ISP_SC_COEFF_BUF_COUNT - 1]) {
			queue->write = &queue->coeff[0];
			queue->w_index = 0;
		}
		if (queue->write == queue->read) {
			queue->write = ori;
			pr_warn("warning, queue is full\n");
			ret = -EAGAIN;
		}
	}
	spin_unlock_irqrestore(&queue->lock, flags);

	return ret;
}

int isp_coeff_get_valid_node(struct isp_sc_coeff_queue *queue,
			     struct isp_sc_coeff **coeff, int type)
{
	int ret = ISP_RTN_SUCCESS;
	unsigned long flags;

	if (ISP_ADDR_INVALID(queue) || ISP_ADDR_INVALID(coeff)) {
		pr_err("deq, invalid parm %p %p\n", queue, coeff);
		return -EINVAL;
	}

	pr_debug("read buf queue\n");

	spin_lock_irqsave(&queue->lock, flags);

	if (queue->read != queue->write) {
		*coeff = queue->read;
		if (type) {
			queue->read++;
			queue->r_index++;
			if (queue->read
			    > &queue->coeff[ISP_SC_COEFF_BUF_COUNT - 1]) {
				queue->read = &queue->coeff[0];
				queue->r_index = 0;
			}
		}
	} else {
		ret = -EAGAIN;
		pr_debug("warning, queue is null\n");
	}
	spin_unlock_irqrestore(&queue->lock, flags);

	return ret;
}

void isp_frm_clear(struct isp_pipe_dev *dev, enum isp_path_index path_index)
{
	struct camera_frame frame, *res_frame;
	struct isp_path_desc *path;
	struct isp_module *module = NULL;
	struct isp_statis_module *statis_module = NULL;

	if (!dev)
		return;

	module = &dev->module_info;
	statis_module = &dev->statis_module_info;
	if (ISP_PATH_IDX_PRE & path_index) {
		path = &module->isp_path[ISP_SCL_PRE];
		res_frame = &module->path_reserved_frame[ISP_SCL_PRE];

#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
		if (path->path_mode != ISP_PRE_ONLINE_CPP &&
				path->path_mode != ISP_PRE_VID_ONLINE_CPP)
#endif
		{
			while (!isp_frame_dequeue(&path->frame_queue, &frame)) {
				if (frame.pfinfo.mfd[0] !=
						res_frame->pfinfo.mfd[0])
					pfiommu_free_addr(&frame.pfinfo);
			}

			isp_frm_queue_clear(&path->frame_queue);
			isp_buf_queue_init(&path->buf_queue);
			if (res_frame->pfinfo.mfd[0] != 0 &&
					res_frame->pfinfo.iova[0])
				pfiommu_free_addr(&res_frame->pfinfo);
			memset((void *)res_frame, 0,
					sizeof(struct camera_frame));
		}
	}

	if (ISP_PATH_IDX_VID & path_index) {
		path = &module->isp_path[ISP_SCL_VID];
		res_frame = &module->path_reserved_frame[ISP_SCL_VID];

#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
		if (path->path_mode != ISP_VID_ONLINE_CPP &&
				path->path_mode != ISP_PRE_VID_ONLINE_CPP)
#endif
		{
			while (!isp_frame_dequeue(&path->frame_queue,
						&frame)) {
				if (frame.pfinfo.mfd[0] !=
						res_frame->pfinfo.mfd[0])
					pfiommu_free_addr(&frame.pfinfo);
			}

			isp_frm_queue_clear(&path->frame_queue);
			isp_buf_queue_init(&path->buf_queue);

			if (res_frame->pfinfo.mfd[0] != 0 &&
					res_frame->pfinfo.iova[0])
				pfiommu_free_addr(&res_frame->pfinfo);
			memset((void *)res_frame, 0,
					sizeof(struct camera_frame));
		}
	}

	if (ISP_PATH_IDX_CAP & path_index) {
		path = &module->isp_path[ISP_SCL_CAP];
		res_frame = &module->path_reserved_frame[ISP_SCL_CAP];
		while (!isp_frame_dequeue(&path->frame_queue, &frame)) {
			if (frame.pfinfo.mfd[0] != res_frame->pfinfo.mfd[0])
				pfiommu_free_addr(&frame.pfinfo);
		}

		isp_frm_queue_clear(&path->frame_queue);
		isp_buf_queue_init(&path->buf_queue);

		if (res_frame->pfinfo.mfd[0] != 0 && res_frame->pfinfo.iova[0])
			pfiommu_free_addr(&res_frame->pfinfo);
		memset((void *)res_frame, 0, sizeof(struct camera_frame));
	}
}

int isp_fmcu_get_buf(struct isp_fmcu_slice_desc *fmcu, enum isp_id idx)
{
	enum isp_rtn rtn = ISP_RTN_SUCCESS;
	char name[32];
	unsigned int num = 0;
	size_t fmcu_buf_len = PAGE_SIZE;
	unsigned long phys_yaddr = 0;
	struct device *dev;

	if (!fmcu) {
		pr_err("Input ptr is NULL\n");
		return -EFAULT;
	}

	if (idx == ISP_ID_0)
		dev = &s_isp_pdev->dev;
	else
		dev = &s_isp1_pdev->dev;

	if (fmcu->fmcu_client != NULL && fmcu->fmcu_handle != NULL) {
		pr_info("fmcu buffer has alloced.\n");
		if (sprd_iommu_attach_device(dev) != 0)
			return rtn;
	}

#ifdef ION
	sprintf(name, "sprd-fmcu%d", num);
	fmcu->fmcu_client = sprd_ion_client_create(name);
	if (IS_ERR(fmcu->fmcu_client)) {
		pr_err("failed to create storecce_client\n");
		return -EPERM;
	}

	/*iommu map*/
	if (sprd_iommu_attach_device(dev) == 0) {
		fmcu->fmcu_handle = ion_alloc(fmcu->fmcu_client,
					fmcu_buf_len, 0,
					ION_HEAP_ID_MASK_SYSTEM,
					ION_FLAG_NOCLEAR);
		if (IS_ERR(fmcu->fmcu_handle)) {
			pr_err("fail to alloc fmcu_handle\n");
			ion_client_destroy(fmcu->fmcu_client);
			fmcu->fmcu_client = NULL;
			return -EPERM;
		}
		fmcu->iommu_addr_vir =
			(unsigned int *)ion_map_kernel(fmcu->fmcu_client,
				fmcu->fmcu_handle);
		if (IS_ERR(fmcu->iommu_addr_vir)) {
			pr_err("fail to get fmcu map viraddr\n");
			goto ERROR;
		}
		pr_info("%s ISP%d iommu suc: fmcu vir:%p, fmcu_addr_phy:%lx\n",
				__func__, idx,
				fmcu->iommu_addr_vir, fmcu->iommu_addr_phy);
	} else {
		fmcu->fmcu_handle = ion_alloc(fmcu->fmcu_client,
				    fmcu_buf_len,
				    0,
				    ION_HEAP_ID_MASK_MM,
				    ION_FLAG_NOCLEAR);
		if (IS_ERR(fmcu->fmcu_handle)) {
			pr_err("failed to alloc fmcu_handle\n");
			ion_client_destroy(fmcu->fmcu_client);
			fmcu->fmcu_client = NULL;
			return -EPERM;
		}
		if (ion_phys(fmcu->fmcu_client, fmcu->fmcu_handle,
			     &phys_yaddr, &fmcu_buf_len)) {
			pr_err("failed to phys fmcu buf\n");
			goto ERROR;
		}

		fmcu->iommu_addr_phy = phys_yaddr;
		fmcu->iommu_addr_vir =
			(unsigned int *)ion_map_kernel(fmcu->fmcu_client,
				fmcu->fmcu_handle);
		if (IS_ERR(fmcu->iommu_addr_vir)) {
			pr_err("Get fmcu map viraddr error\n");
			goto ERROR;
		}
		pr_info("%s ISP%d ion suc: fmcu_buf_len:%zx, fmcu vir:%p, fmcu_addr_phy:%lx\n",
				__func__, idx, fmcu_buf_len,
				fmcu->iommu_addr_vir, fmcu->iommu_addr_phy);
	}
#else
	pr_info("Not support ion buf operation\n");
	return -EFAULT;
#endif

	return ISP_RTN_SUCCESS;

ERROR:
	ion_free(fmcu->fmcu_client, fmcu->fmcu_handle);
	ion_client_destroy(fmcu->fmcu_client);
	fmcu->fmcu_client = NULL;
	fmcu->fmcu_handle = NULL;
	return -EPERM;
}

int isp_fmcu_clear_buf(struct isp_fmcu_slice_desc *fmcu, enum isp_id idx)
{
	int rtn = 0;

	if (!fmcu) {
		pr_err("Input ptr is NULL\n");
		return -EFAULT;
	}

#ifdef ION
	if (fmcu->fmcu_client == NULL || fmcu->fmcu_handle == NULL)
		return -EPERM;

	if (fmcu->iommu_addr_vir != NULL) {
		ion_unmap_kernel(fmcu->fmcu_client,
			fmcu->fmcu_handle);
		pr_info("%s ISP%d: iommu_addr_vir: %p\n", __func__, idx,
					fmcu->iommu_addr_vir);
		fmcu->iommu_addr_vir = NULL;
	}
	ion_free(fmcu->fmcu_client, fmcu->fmcu_handle);
	ion_client_destroy(fmcu->fmcu_client);
	fmcu->fmcu_client = NULL;
	fmcu->fmcu_handle = NULL;
#else
	pr_info("Not support ion buf operation\n");
#endif
	return rtn;
}

int isp_fmcu_iommu_map(struct isp_fmcu_slice_desc *fmcu, enum isp_id idx)
{
	enum isp_rtn rtn = ISP_RTN_SUCCESS;
	unsigned long phys_yaddr = 0;
	struct ion_buffer *ionbuffer = NULL;
	struct sprd_iommu_map_data iommu_data;
	struct device *dev;

	if (idx == ISP_ID_0)
		dev = &s_isp_pdev->dev;
	else
		dev = &s_isp1_pdev->dev;

	if (sprd_iommu_attach_device(dev) != 0 ||
		fmcu->iommu_addr_phy != 0)
		return rtn;

	if (fmcu->fmcu_client == NULL || fmcu->fmcu_handle == NULL)
		return -EPERM;

	memset(&iommu_data, 0, sizeof(struct sprd_iommu_map_data));
	ionbuffer = ion_handle_buffer(fmcu->fmcu_handle);
	iommu_data.buf = (void *)ionbuffer;
	iommu_data.iova_size = ionbuffer->size;
	iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
	iommu_data.sg_offset = 0;

	if (!fmcu->iommu_addr_phy) {
		rtn = sprd_iommu_map(dev, &iommu_data);
		if (rtn) {
			pr_err("failed to get iommu y_kaddr\n");
			return -EFAULT;
		}
		phys_yaddr = iommu_data.iova_addr;

		fmcu->iommu_addr_phy = phys_yaddr;
	}

	return rtn;
}

int isp_fmcu_iommu_unmap(struct isp_fmcu_slice_desc *fmcu, enum isp_id idx)
{
	enum isp_rtn rtn = ISP_RTN_SUCCESS;
	struct ion_buffer *ionbuffer = NULL;
	struct sprd_iommu_unmap_data iommu_data;
	struct device *dev;

	if (idx == ISP_ID_0)
		dev = &s_isp_pdev->dev;
	else
		dev = &s_isp1_pdev->dev;

	if (sprd_iommu_attach_device(dev) != 0)
		return rtn;

	memset(&iommu_data, 0, sizeof(struct sprd_iommu_unmap_data));
	if (fmcu->fmcu_client == NULL || fmcu->fmcu_handle == NULL)
		return -EPERM;

	ionbuffer = ion_handle_buffer(fmcu->fmcu_handle);
	iommu_data.iova_addr = fmcu->iommu_addr_phy;
	iommu_data.iova_size = ionbuffer->size;
	iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
	iommu_data.buf = NULL;

	if (fmcu->iommu_addr_phy > 0) {
		rtn = sprd_iommu_unmap(dev,
					    &iommu_data);
		if (rtn) {
			pr_err("failed to unmap iommu y_kaddr\n");
			return -EFAULT;
		}
		fmcu->iommu_addr_phy = 0;
	}

	pr_debug("isp_storecce_buf_iommu_unmap\n");

	return rtn;
}

int isp_storecce_get_buf(struct isp_store_cce_desc *store_cce,
	enum isp_id idx)
{
	enum isp_rtn rtn = ISP_RTN_SUCCESS;
#ifdef ION
	unsigned long phys_yaddr = 0, phys_uvaddr = 0;
	unsigned int num = 0;
	char name1[32], name2[32];
	size_t y_buf_len, uv_buf_len;
	struct storecce_ion_buf *ion_buf = NULL;
	struct camera_frame frame = {0};
	struct device *dev;
#endif
	if (!store_cce) {
		pr_err("Input ptr is NULL\n");
		return -EFAULT;
	}

#ifdef ION
	if (!store_cce->y_buf_len || !store_cce->uv_buf_len) {
		pr_err("Input buf size error\n");
		return -EFAULT;
	}

	if (idx == ISP_ID_0)
		dev = &s_isp_pdev->dev;
	else
		dev = &s_isp1_pdev->dev;

	for (num = 0; num < ISP_STORE_CCE_BUF_NUM; num++) {
		ion_buf = &store_cce->ion_buf[num];

		if (ion_buf == NULL)
			return -EPERM;
		if (ion_buf->y_client != NULL
		    && ion_buf->y_handle != NULL
		    && ion_buf->uv_handle != NULL
		    && ion_buf->uv_client != NULL) {
			pr_info("storecce buffer %d has alloced.\n", num);
			if (sprd_iommu_attach_device(dev) != 0) {
				frame.yaddr = ion_buf->addr.yaddr;
				frame.uaddr = ion_buf->addr.uaddr;
				isp_buf_queue_write(&store_cce->tmp_buf_queue,
						    &frame);
			}
			continue;
		}

		sprintf(name1, "sprd-storecce-y%d", num);
		sprintf(name2, "sprd-storecce-uv%d", num);
		ion_buf->y_client = sprd_ion_client_create(name1);
		ion_buf->uv_client = sprd_ion_client_create(name2);
		if (IS_ERR(ion_buf->y_client) || IS_ERR(ion_buf->uv_client)) {
			pr_err("failed to create storecce_client\n");
			return -EPERM;
		}

		/*iommu map*/
		if (sprd_iommu_attach_device(dev) == 0) {
			ion_buf->y_handle = ion_alloc(ion_buf->y_client,
						store_cce->y_buf_len, 0,
						ION_HEAP_ID_MASK_SYSTEM,
						ION_FLAG_NOCLEAR);
			ion_buf->uv_handle = ion_alloc(ion_buf->uv_client,
						store_cce->uv_buf_len, 0,
						ION_HEAP_ID_MASK_SYSTEM,
						ION_FLAG_NOCLEAR);
			if (IS_ERR(ion_buf->y_handle) ||
			    IS_ERR(ion_buf->uv_handle)) {
				pr_err("failed to alloc storecce tmp buf\n");
				return -EPERM;
			}
		} else {
			ion_buf->y_handle = ion_alloc(ion_buf->y_client,
					    store_cce->y_buf_len,
					    0,
					    ION_HEAP_ID_MASK_MM,
					    ION_FLAG_NOCLEAR);
			ion_buf->uv_handle = ion_alloc(ion_buf->uv_client,
						       store_cce->uv_buf_len,
						       0,
						       ION_HEAP_ID_MASK_MM,
						       ION_FLAG_NOCLEAR);
			if (IS_ERR(ion_buf->y_handle)
			    || IS_ERR(ion_buf->uv_handle)) {
				pr_err("failed to alloc storecce tmp buf\n");
				return -EPERM;
			}
			if (ion_phys(ion_buf->y_client, ion_buf->y_handle,
				     &phys_yaddr, &y_buf_len)) {
				pr_err("failed to phys storecce y tmp buf\n");
				return -EPERM;
			}
			pr_info("storecce ion tmp buf phy_yaddr:0x%lx, y_buf_len:0x%x\n",
				phys_yaddr, (unsigned int)y_buf_len);

			if (ion_phys(ion_buf->uv_client, ion_buf->uv_handle,
				     &phys_uvaddr, &uv_buf_len)) {
				pr_err("failed to phys storecce uv tmp buf\n");
				return -EPERM;
			}
			pr_info("storecce ion tmp buf phy_uvaddr:0x%lx, uv_buf_len:0x%x\n",
				phys_uvaddr, (unsigned int)uv_buf_len);

			frame.yaddr = phys_yaddr;
			frame.uaddr = phys_uvaddr;
			ion_buf->addr.yaddr = frame.yaddr;
			ion_buf->addr.uaddr = frame.uaddr;
			rtn = isp_buf_queue_write(&store_cce->tmp_buf_queue,
						  &frame);
		}
	}
#else
	pr_info("Not support ion buf operation\n");
	rtn = -EFAULT;
#endif
	return rtn;
}

int isp_storecce_clear_buf(struct isp_store_cce_desc *store_cce)
{
	int rtn = 0;
	unsigned int num = 0;
	struct storecce_ion_buf *ion_buf = NULL;

	if (!store_cce) {
		pr_err("Input ptr is NULL\n");
		return -EFAULT;
	}
#ifdef ION
	for (num = 0; num < ISP_STORE_CCE_BUF_NUM; num++) {
		ion_buf = &store_cce->ion_buf[num];

		if (ion_buf == NULL)
			return -EPERM;
		if (ion_buf->y_client == NULL || ion_buf->y_handle == NULL
		    || ion_buf->uv_handle == NULL || ion_buf->uv_client == NULL)
			return -EPERM;

		ion_free(ion_buf->y_client, ion_buf->y_handle);
		ion_free(ion_buf->uv_client, ion_buf->uv_handle);
		ion_client_destroy(ion_buf->y_client);
		ion_client_destroy(ion_buf->uv_client);
		ion_buf->y_client = NULL;
		ion_buf->y_handle = NULL;
		ion_buf->uv_client = NULL;
		ion_buf->uv_handle = NULL;
	}
#else
	pr_info("Not support ion buf operation\n");
#endif
	return rtn;
}

int isp_storecce_buf_iommu_map(
	struct isp_store_cce_desc *store_cce, enum isp_id idx)
{
	enum isp_rtn rtn = ISP_RTN_SUCCESS;
	unsigned long phys_yaddr = 0, phys_uvaddr = 0;
	unsigned int num = 0;
	struct camera_frame frame = {0};
	struct storecce_ion_buf *ion_buf = NULL;
	struct ion_buffer *ionbuffer = NULL;
	struct sprd_iommu_map_data iommu_data;
	struct device *dev;

	if (idx == ISP_ID_0)
		dev = &s_isp_pdev->dev;
	else
		dev = &s_isp1_pdev->dev;

	if (sprd_iommu_attach_device(dev) != 0)
		return rtn;

	memset(&iommu_data, 0, sizeof(struct sprd_iommu_map_data));
	for (num = 0; num < ISP_STORE_CCE_BUF_NUM; num++) {
		ion_buf = &store_cce->ion_buf[num];

		if (ion_buf == NULL)
			return -EPERM;
		if (ion_buf->y_client == NULL || ion_buf->y_handle == NULL
		    || ion_buf->uv_handle == NULL || ion_buf->uv_client == NULL)
			return -EPERM;

		ionbuffer = ion_handle_buffer(ion_buf->y_handle);
		iommu_data.buf = (void *)ionbuffer;
		iommu_data.iova_size = ionbuffer->size;
		iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
		iommu_data.sg_offset = 0;

		rtn = sprd_iommu_map(dev, &iommu_data);
		if (rtn) {
			pr_err("failed to get iommu y_kaddr\n");
			return -EFAULT;
		}
		phys_yaddr = iommu_data.iova_addr;

		ionbuffer = ion_handle_buffer(ion_buf->uv_handle);
		iommu_data.buf = (void *)ionbuffer;
		iommu_data.iova_size = ionbuffer->size;
		iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
		iommu_data.sg_offset = 0;

		rtn = sprd_iommu_map(dev, &iommu_data);
		if (rtn) {
			pr_err("failed to get iommu uv_kaddr\n");
			return -EFAULT;
		}
		phys_uvaddr = iommu_data.iova_addr;

		frame.yaddr = phys_yaddr;
		frame.uaddr = phys_uvaddr;
		ion_buf->addr.yaddr = frame.yaddr;
		ion_buf->addr.uaddr = frame.uaddr;
		rtn = isp_buf_queue_write(&store_cce->tmp_buf_queue, &frame);
	}

	pr_debug("isp_storecce_buf_iommu_map\n");

	return rtn;
}

int isp_storecce_buf_iommu_unmap(
	struct isp_store_cce_desc *store_cce, enum isp_id idx)
{
	enum isp_rtn rtn = ISP_RTN_SUCCESS;
	unsigned int num = 0;
	struct storecce_ion_buf *ion_buf = NULL;
	struct ion_buffer *ionbuffer = NULL;
	struct sprd_iommu_unmap_data iommu_data;
	struct device *dev;

	if (idx == ISP_ID_0)
		dev = &s_isp_pdev->dev;
	else
		dev = &s_isp1_pdev->dev;

	if (sprd_iommu_attach_device(dev) != 0)
		return rtn;

	memset(&iommu_data, 0, sizeof(struct sprd_iommu_unmap_data));
	for (num = 0; num < ISP_STORE_CCE_BUF_NUM; num++) {
		ion_buf = &store_cce->ion_buf[num];

		if (ion_buf == NULL)
			return -EPERM;
		if (ion_buf->y_client == NULL || ion_buf->y_handle == NULL
		    || ion_buf->uv_handle == NULL || ion_buf->uv_client == NULL)
			return -EPERM;

		ionbuffer = ion_handle_buffer(ion_buf->y_handle);
		iommu_data.iova_addr = ion_buf->addr.yaddr;
		iommu_data.iova_size = ionbuffer->size;
		iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
		iommu_data.buf = NULL;

		if (ion_buf->addr.yaddr > 0) {
			rtn = sprd_iommu_unmap(dev, &iommu_data);
			if (rtn) {
				pr_err("failed to unmap iommu y_kaddr\n");
				return -EFAULT;
			}
			ion_buf->addr.yaddr = 0;
		}

		ionbuffer = ion_handle_buffer(ion_buf->uv_handle);
		iommu_data.iova_size = ionbuffer->size;
		iommu_data.iova_addr = ion_buf->addr.uaddr;
		iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
		iommu_data.buf = NULL;

		if (ion_buf->addr.uaddr > 0) {
			rtn = sprd_iommu_unmap(dev, &iommu_data);
			if (rtn) {
				pr_err("failed to unmap iommu uv_kaddr\n");
				return -EFAULT;
			}
			ion_buf->addr.uaddr = 0;
		}
	}

	pr_debug("isp_storecce_buf_iommu_unmap\n");

	return rtn;
}

#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
int isp_ion_cpp_buf_iommu_map(
	struct device *dev, struct camera_frame  *frame)
{
	enum isp_rtn rtn = ISP_RTN_SUCCESS;
	unsigned long phys_yaddr = 0;
	struct sprd_iommu_map_data iommu_data;


	if (sprd_iommu_attach_device(dev) != 0)
		return rtn;

	memset(&iommu_data, 0, sizeof(struct sprd_iommu_map_data));
	if (frame->pfinfo.table[0] == NULL) {
		pr_err("failed to get sg table\n");
		return -EPERM;
	}
	if (frame->pfinfo.size[0] == 0 ||
			frame->pfinfo.size[1] == 0) {
		pr_err("failed to get iommu size\n");
		return -EPERM;
	}

	iommu_data.buf = frame->pfinfo.buf[0];
	iommu_data.iova_size = frame->pfinfo.size[0] + frame->pfinfo.size[1];
	iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
	iommu_data.sg_offset = frame->pfinfo.offset[0];
	rtn = sprd_iommu_map(dev, &iommu_data);
	if (rtn) {
		pr_err("failed to get iommu y_kaddr\n");
		return -EFAULT;
	}
	phys_yaddr = iommu_data.iova_addr;
	pr_debug("isp cpp map phys_yaddr 0x%lx\n", phys_yaddr);
	frame->yaddr = phys_yaddr;
	frame->uaddr = phys_yaddr + frame->pfinfo.size[0];
	pr_debug("isp_cpp ion buf_iommu_map\n");
	return rtn;
}

int isp_ion_cpp_buf_iommu_unmap(
	struct device *dev, struct camera_frame  *frame,
	unsigned int ch_id)
{
	enum isp_rtn rtn = ISP_RTN_SUCCESS;
	struct sprd_iommu_unmap_data iommu_data;


	if (sprd_iommu_attach_device(dev) != 0)
		return rtn;

	memset(&iommu_data, 0, sizeof(struct sprd_iommu_unmap_data));


	if (frame->pfinfo.table[0] == NULL) {
		pr_err("failed to get sg table\n");
		return -EPERM;
	}
	if (frame->pfinfo.size[0] == 0 ||
			frame->pfinfo.size[1] == 0) {
		pr_err("failed to get iommu size\n");
		return -EPERM;
	}
	iommu_data.iova_size = frame->pfinfo.size[0] + frame->pfinfo.size[1];
	iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
	iommu_data.iova_addr = frame->yaddr;
	iommu_data.buf = NULL;


	pr_debug("isp cpp  unmap phys_Yaddr 0x%lx\n", iommu_data.iova_addr);
	if (iommu_data.iova_addr > 0) {
		if (ch_id)
			iommu_data.ch_type = SPRD_IOMMU_EX_CH_WRITE;
		else
			iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
		iommu_data.channel_id = ch_id;
		rtn = sprd_iommu_unmap(dev, &iommu_data);
		if (rtn) {
			pr_err("failed to unmap iommu y_kaddr\n");
			return -EFAULT;
		}
		frame->yaddr = 0;
		frame->uaddr = 0;
	}
	pr_debug("isp_cpp ion buf_iommu_unmap\n");

	return rtn;
}

static int isp_cpp_attach_iommu_frame(struct path_cpp_ion_buf *ion_buf,
		struct camera_frame  *frame, unsigned int y_len,
		unsigned int uv_len)
{
	struct ion_buffer *ionbuffer = NULL;

	ionbuffer = ion_handle_buffer(ion_buf->y_handle);
	if (IS_ERR(ionbuffer)) {
		pr_err("isp_cpp_ionbuffer errr\n");
		return -1;
	}
	ion_buf->sg_table = ionbuffer->sg_table;
	frame->pfinfo.table[0] = ionbuffer->sg_table;
	frame->pfinfo.buf[0] = (void *)ionbuffer;
	frame->pfinfo.size[0] = y_len;
	frame->pfinfo.offset[0] = 0;
	frame->pfinfo.size[1] = uv_len;
	return 0;
}

int isp_cpp_get_buf(struct isp_cpp_desc *isp_cpp_dev,
		int idx)
{

	unsigned int num = 0;
	char name1[32] = {0};
	struct path_cpp_ion_buf *ion_buf = NULL;
	struct device *isp_dev;
	struct device *cpp_dev;
	struct camera_frame  frame;
	size_t size  = 0;

	if (idx == ISP_ID_0)
		isp_dev = &s_isp_pdev->dev;
	else
		isp_dev = &s_isp1_pdev->dev;

	if (!isp_cpp_dev->y_buf_len || !isp_cpp_dev->uv_buf_len) {
		pr_err("Input buf size error\n");
		return -EFAULT;
	}
	size = isp_cpp_dev->y_buf_len + isp_cpp_dev->uv_buf_len;
	if (IS_ERR(isp_dev)) {
		pr_err("isp_cpp device is error\n");
		return -1;
	}
	for (num = 0; num < PATH_CPP_ION_BUF_NUM; num++) {
		ion_buf = &isp_cpp_dev->ion_buf[num];
		if (IS_ERR(ion_buf)) {
			pr_err("ion error\n");
			return -EFAULT;
		}
		if (ion_buf->y_client != NULL
				&& ion_buf->y_handle != NULL
				) {
			pr_info("cpp ion buffer %d has alloced.\n", num);
			isp_cpp_attach_iommu_frame(ion_buf, &frame,
					isp_cpp_dev->y_buf_len,
					isp_cpp_dev->uv_buf_len);
			isp_buf_queue_write(&isp_cpp_dev->cpp_buf_queue,
					&frame);
			continue;
		}
		sprintf(name1, "sprd-path_cpp-y%d", num);
		ion_buf->y_client = sprd_ion_client_create(name1);
		if (IS_ERR(ion_buf->y_client)) {
			pr_err("failed to alloc storecce tmp buf\n");
			return -EPERM;
		}
		/*iommu map*/
		if (sprd_iommu_attach_device(isp_dev) == 0) {
			ion_buf->y_handle = ion_alloc(ion_buf->y_client,
					size, 0,
					ION_HEAP_ID_MASK_SYSTEM,
					ION_FLAG_NOCLEAR);
			if (IS_ERR(ion_buf->y_handle)
					) {
				pr_err("failed to alloc storecce tmp buf\n");
				return -EPERM;
			}
		}
		isp_cpp_attach_iommu_frame(ion_buf, &frame,
				isp_cpp_dev->y_buf_len,
				isp_cpp_dev->uv_buf_len);

		/*map the buffer with ISP device*/
		isp_ion_cpp_buf_iommu_map(isp_dev, &frame);
		ion_buf->isp_addr.yaddr = frame.yaddr;
		ion_buf->isp_addr.uaddr = frame.uaddr;

		/*map the buffer with ISP device*/
		isp_cpp_get_cpp_dev(&cpp_dev);
		isp_ion_cpp_buf_iommu_map(cpp_dev, &frame);
		ion_buf->cpp_addr.yaddr = frame.yaddr;
		ion_buf->cpp_addr.uaddr = frame.uaddr;

		/*use last frame as reserved frame */
		if (num == PATH_CPP_ION_BUF_NUM - 1)
			memcpy(&isp_cpp_dev->cpp_reserved_frame, &frame,
			    sizeof(struct camera_frame));
		 else
			isp_buf_queue_write(&isp_cpp_dev->cpp_buf_queue,
						&frame);

	}

	return 0;
}
int isp_cpp_clear_buf(int idx, struct isp_cpp_desc *isp_cpp_dev)
{
	int rtn = 0;
	unsigned int num = 0;
	struct path_cpp_ion_buf *ion_buf = NULL;
	struct device *isp_dev;
	struct device *cpp_dev;
	struct sprd_iommu_unmap_data iommu_data;

	if (!isp_cpp_dev) {
		pr_err("Input ptr is NULL\n");
		return -EFAULT;
	}
	if (idx == ISP_ID_0)
		isp_dev = &s_isp_pdev->dev;
	else
		isp_dev = &s_isp1_pdev->dev;
	isp_cpp_get_cpp_dev(&cpp_dev);
	memset(&iommu_data, 0, sizeof(struct sprd_iommu_unmap_data));
#ifdef ION
	for (num = 0; num < PATH_CPP_ION_BUF_NUM; num++) {
		ion_buf = &isp_cpp_dev->ion_buf[num];

		if (ion_buf == NULL)
			return -EPERM;
		if (ion_buf->y_client == NULL || ion_buf->y_handle == NULL)
			return -EPERM;
		/*umap the isp iommu*/
		iommu_data.iova_size = isp_cpp_dev->y_buf_len +
					    isp_cpp_dev->uv_buf_len;
		iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
		iommu_data.iova_addr = ion_buf->isp_addr.yaddr;
		sprd_iommu_free_kaddr(isp_dev, &iommu_data);

		/*umap the cpp iommu*/
		iommu_data.iova_size = isp_cpp_dev->y_buf_len +
					isp_cpp_dev->uv_buf_len;
		iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
		iommu_data.iova_addr = ion_buf->cpp_addr.yaddr;
		sprd_iommu_free_kaddr(cpp_dev, &iommu_data);

		ion_free(ion_buf->y_client, ion_buf->y_handle);
		ion_client_destroy(ion_buf->y_client);
		ion_buf->y_client = NULL;
		ion_buf->y_handle = NULL;
		ion_buf->isp_addr.yaddr  = 0;
		ion_buf->isp_addr.uaddr  = 0;
		ion_buf->cpp_addr.yaddr  = 0;
		ion_buf->cpp_addr.uaddr  = 0;
	}
	pr_info("clear cpp bufoperation\n");
#else
	pr_info("Not support ion buf operation\n");
#endif
	return rtn;
}
#endif


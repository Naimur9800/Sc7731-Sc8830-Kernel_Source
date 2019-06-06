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

#include <linux/err.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/sprd_iommu.h>
#include <linux/sprd_ion.h>

#include "cam_types.h"
#include "cam_buf.h"
#include "cam_queue.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "cam_buf: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__


int cambuf_get_ionbuf(struct camera_buf *buf_info)
{
	int i;
	int ret = 0;
	void *ionbuf[3];

	if (!buf_info) {
		pr_err("error: input ptr is NULL\n");
		return -EFAULT;
	}
	if (buf_info->type != CAM_BUF_USER) {
		pr_err("error buffer type: %d\n", buf_info->type);
		return -EFAULT;
	}

	pr_debug("enter.\n");
	for (i = 0; i < 3; i++) {
		if (buf_info->mfd[i] <= 0)
			continue;

		pr_debug("user buf:  %d,  %d\n", i, buf_info->mfd[i]);
		ret = sprd_ion_get_buffer(buf_info->mfd[i],
				NULL,
				&ionbuf[i],
				&buf_info->size[i]);
		if (ret) {
			pr_err("failed to get ionbuf for user buffer %d\n",
					buf_info->mfd[i]);
			goto failed;
		}
		buf_info->ionbuf[i] = ionbuf[i];
		pr_debug("size %d, ionbuf %p\n",
			(int)buf_info->size[i], ionbuf[i]);

		/*
		* get dambuf to avoid ion buf is freed during hardware access it.
		* In fact, this situation will happen when application exit
		* exceptionally and for user buffer only.
		* Just get dmabuf when hardware ready to access it
		* for user buffer can avoid previous situation.
		* We get dmabuf here because if the buffer is passed to kernel,
		* there is potential hardware access.
		* And it must be put before return to user space,
		* or else it may cause ion memory leak.
		*/
		if (buf_info->dmabuf_p[i] == NULL) {
			buf_info->dmabuf_p[i] =
					dma_buf_get(buf_info->mfd[i]);
			if (IS_ERR_OR_NULL(buf_info->dmabuf_p[i])) {
				pr_err("failed to get dma buf %p\n",
						buf_info->dmabuf_p[i]);
				goto failed;
			}
			if (g_mem_dbg)
				atomic_inc(&g_mem_dbg->ion_dma_cnt);
			pr_debug("dmabuf %p\n", buf_info->dmabuf_p[i]);
		}

	}
	return 0;

failed:
	for (i = 0; i < 3; i++) {
		if (buf_info->mfd[i] <= 0)
			continue;
		if (buf_info->dmabuf_p[i]) {
			dma_buf_put(buf_info->dmabuf_p[i]);
			buf_info->dmabuf_p[i] = NULL;
			if (g_mem_dbg)
				atomic_dec(&g_mem_dbg->ion_dma_cnt);
		}
		buf_info->ionbuf[i] = NULL;
	}
	return ret;
}

int cambuf_put_ionbuf(struct camera_buf *buf_info)
{
	int i;
	int ret = 0;

	if (!buf_info) {
		pr_err("error: input ptr is NULL\n");
		return -EFAULT;
	}
	if (buf_info->type != CAM_BUF_USER) {
		pr_info("buffer type: %d is not user buf.\n", buf_info->type);
		return 0;
	}

	pr_debug("enter.\n");
	for (i = 0; i < 3; i++) {
		if (buf_info->mfd[i] <= 0)
			continue;
		if (buf_info->dmabuf_p[i]) {
			dma_buf_put(buf_info->dmabuf_p[i]);
			buf_info->dmabuf_p[i] = NULL;
			if (g_mem_dbg)
				atomic_dec(&g_mem_dbg->ion_dma_cnt);
		}
		buf_info->ionbuf[i] = NULL;
	}
	return ret;
}


int cambuf_iommu_map(
			struct camera_buf *buf_info,
			struct device *dev)
{
	int i;
	int ret = 0;
	void *ionbuf[3];
	struct sprd_iommu_map_data iommu_data;

	if (!buf_info || !dev) {
		pr_err("error: input ptr is NULL\n");
		return -EFAULT;
	}

	pr_debug("enter.\n");
	for (i = 0; i < 3; i++) {
		if (buf_info->ionbuf[i] == NULL)
			continue;

		ionbuf[i] = buf_info->ionbuf[i];
		if (sprd_iommu_attach_device(dev) == 0) {
			memset(&iommu_data, 0,
				sizeof(struct sprd_iommu_map_data));
			iommu_data.buf = ionbuf[i];
			iommu_data.iova_size = buf_info->size[i];
			iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
			iommu_data.sg_offset = buf_info->offset[i];
			pr_debug("start map buf: %p, size: %d\n",
					ionbuf[i], (int)iommu_data.iova_size);
			ret = sprd_iommu_map(dev, &iommu_data);
			if (ret) {
				pr_err("failed to get iommu kaddr %d\n", i);
				ret = -EFAULT;
				goto failed;
			}
			if (g_mem_dbg)
				atomic_inc(&g_mem_dbg->iommu_map_cnt);
			pr_debug("done map buf addr: %lx\n",
					iommu_data.iova_addr);
			buf_info->iova[i] = iommu_data.iova_addr;
		} else {
			if (buf_info->type == CAM_BUF_USER)
				ret = sprd_ion_get_phys_addr(-1,
						buf_info->dmabuf_p[i],
						&buf_info->iova[i],
						&buf_info->size[i]);
			else
				ret = ion_phys(buf_info->client[i],
							buf_info->handle[i],
							&buf_info->iova[i],
							&buf_info->size[i]);
			if (ret) {
				pr_err("failed to get iommu kaddr %d\n", i);
				ret = -EFAULT;
				goto failed;
			}
		}
	}
	buf_info->dev = dev;
	buf_info->mapping_state |= CAM_BUF_MAPPING_DEV;
	return 0;

failed:
	for (i = 0; i < 3; i++) {
		if (buf_info->size[i] <= 0 || buf_info->iova[i] == 0)
			continue;

		if (sprd_iommu_attach_device(dev) == 0) {
			struct sprd_iommu_unmap_data unmap_data;

			unmap_data.iova_addr = buf_info->iova[i];
			unmap_data.iova_size = buf_info->size[i];
			unmap_data.ch_type = SPRD_IOMMU_FM_CH_RW;
			unmap_data.table = NULL;
			unmap_data.buf = NULL;
			ret = sprd_iommu_unmap(dev, &unmap_data);
			if (ret)
				pr_err("failed to free iommu %d\n", i);
			if (g_mem_dbg)
				atomic_dec(&g_mem_dbg->iommu_map_cnt);
		}
		buf_info->iova[i] = 0;
	}

	return ret;
}


int cambuf_iommu_unmap(
		struct camera_buf *buf_info)
{
	int i;
	int ret = 0;
	struct sprd_iommu_unmap_data unmap_data;

	if (!buf_info || !buf_info->dev) {
		pr_err("error: input ptr is NULL\n");
		return -EFAULT;
	}

	if ((buf_info->mapping_state & CAM_BUF_MAPPING_DEV) == 0) {
		pr_err("error map status: %x, %p",
			buf_info->mapping_state, buf_info->dev);
		return -EFAULT;
	}

	for (i = 0; i < 3; i++) {
		if (buf_info->size[i] <= 0 || buf_info->iova[i] == 0)
			continue;

		if (sprd_iommu_attach_device(buf_info->dev) == 0) {
			unmap_data.iova_addr = buf_info->iova[i];
			unmap_data.iova_size = buf_info->size[i];
			unmap_data.ch_type = SPRD_IOMMU_FM_CH_RW;
			unmap_data.table = NULL;
			unmap_data.buf = NULL;
			pr_debug("upmap buf addr: %lx\n", unmap_data.iova_addr);
			ret = sprd_iommu_unmap(buf_info->dev, &unmap_data);
			if (ret)
				pr_err("failed to free iommu %d\n", i);
			if (g_mem_dbg)
				atomic_dec(&g_mem_dbg->iommu_map_cnt);
		}
		buf_info->iova[i] = 0;
	}

	buf_info->dev = NULL;
	buf_info->mapping_state &= ~(CAM_BUF_MAPPING_DEV);
	pr_debug("unmap done.\n");
	return 0;
}

int cambuf_kmap(struct camera_buf *buf_info)
{
	int i;
	int ret = 0;

	if (!buf_info) {
		pr_err("error: input ptr is NULL\n");
		return -EFAULT;
	}

	if ((buf_info->type != CAM_BUF_KERNEL) ||
		(buf_info->mapping_state & CAM_BUF_MAPPING_KERNEL)) {
		pr_err("error buf type %d status: %x",
			buf_info->type, buf_info->mapping_state);
		return -EFAULT;
	}

	for (i = 0; i < 3; i++) {
		if (buf_info->size[i] <= 0)
			continue;

		if (buf_info->client[i] && buf_info->handle[i]) {
			buf_info->addr_k[i] = (unsigned long)ion_map_kernel(
					buf_info->client[i],
					buf_info->handle[i]);
			if (IS_ERR_OR_NULL((void *)buf_info->addr_k[i])) {
				pr_err("FATAL: can't map kernel virtual address\n");
				ret = -EINVAL;
				goto map_fail;
			}
			if (g_mem_dbg)
				atomic_inc(&g_mem_dbg->ion_kmap_cnt);
		}
	}
	buf_info->mapping_state |= CAM_BUF_MAPPING_KERNEL;

	pr_info("done: %p\n", (void *)buf_info->addr_k[0]);
	return 0;

map_fail:
	for (i = 0; i < 3; i++) {
		if (buf_info->size[i] <= 0 || buf_info->addr_k[i] == 0)
			continue;

		ion_unmap_kernel(buf_info->client[i], buf_info->handle[i]);
		buf_info->addr_k[i] = 0;
		if (g_mem_dbg)
			atomic_dec(&g_mem_dbg->ion_kmap_cnt);
	}
	return ret;
}


int cambuf_kunmap(struct camera_buf *buf_info)
{
	int i;
	int ret = 0;

	if (!buf_info) {
		pr_err("error: input ptr is NULL\n");
		return -EFAULT;
	}

	if ((buf_info->type != CAM_BUF_KERNEL) ||
		!(buf_info->mapping_state & CAM_BUF_MAPPING_KERNEL)) {
		pr_err("error buf type %d status: %x",
			buf_info->type, buf_info->mapping_state);
		return -EFAULT;
	}

	for (i = 0; i < 3; i++) {
		if (buf_info->size[i] <= 0 || buf_info->addr_k[i] == 0)
			continue;
		pr_info("kunmap: %p\n", (void *)buf_info->addr_k[i]);
		ion_unmap_kernel(buf_info->client[i], buf_info->handle[i]);
		buf_info->addr_k[i] = 0;
		if (g_mem_dbg)
			atomic_dec(&g_mem_dbg->ion_kmap_cnt);
	}

	buf_info->mapping_state &= ~CAM_BUF_MAPPING_KERNEL;
	return ret;

}


int  cambuf_alloc(struct camera_buf *buf_info,
		size_t size, size_t align, int iommu_enable)
{
	int ret = 0;
	int heap_type;
	char name[64];
	struct ion_client *client;
	struct ion_handle *handle;

	if (!buf_info) {
		pr_err("error: input ptr is NULL\n");
		return -EFAULT;
	}
	pr_debug("alloc size %d, align %d, iommu %d\n",
			(uint32_t)size, (uint32_t)align,  iommu_enable);

	snprintf(name, 16+CAM_BUF_NAME_LEN,
			"camera-buf-%s", buf_info->name);
	client = sprd_ion_client_create(name);
	if (IS_ERR_OR_NULL(client)) {
		pr_err("failed to create ION client\n");
		return -EPERM;
	}

	heap_type = iommu_enable ?
		ION_HEAP_ID_MASK_SYSTEM : ION_HEAP_ID_MASK_MM;

	handle = ion_alloc(client, size, 0, heap_type, ION_FLAG_NOCLEAR);
	if (IS_ERR_OR_NULL(handle)) {
		pr_err("failed to alloc ion buf size = 0x%x\n", (int)size);
		ret = -ENOMEM;
		goto alloc_fail;
	}
	buf_info->type = CAM_BUF_KERNEL;
	buf_info->size[0] = size;
	buf_info->client[0] = client;
	buf_info->handle[0] = handle;
	buf_info->ionbuf[0] = ion_handle_buffer(buf_info->handle[0]);

	if (g_mem_dbg)
		atomic_inc(&g_mem_dbg->ion_alloc_cnt);

	pr_info("alloc done. %p\n", buf_info);
	return 0;

alloc_fail:
	ion_client_destroy(client);

	return ret;
}


int cambuf_free(struct camera_buf *buf_info)
{
	int rtn = 0;
	struct ion_client *client;
	struct ion_handle *handle;

	if (!buf_info) {
		pr_err("error: input ptr is NULL\n");
		return -EFAULT;
	}

	client = buf_info->client[0];
	handle = buf_info->handle[0];
	if (buf_info->type != CAM_BUF_KERNEL ||
		client == NULL || handle == NULL) {
		pr_err("error buffer type: %d\n", buf_info->type);
		return -EPERM;
	}

	if (buf_info->mapping_state != CAM_BUF_MAPPING_NULL) {
		pr_err("error: unmapping %x. iova or kernel addr may be leak.\n",
				buf_info->mapping_state);
	}

	ion_free(client, handle);
	ion_client_destroy(client);
	buf_info->size[0] = 0;
	buf_info->client[0] = NULL;
	buf_info->handle[0] = NULL;
	buf_info->ionbuf[0] = NULL;
	if (g_mem_dbg)
		atomic_dec(&g_mem_dbg->ion_alloc_cnt);

	pr_info("free done: %p\n",  buf_info);

	return rtn;
}

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

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/sipc.h>
#include <linux/debugfs.h>
#include <asm/cacheflush.h>

#include "../include/sprd_modem_control.h"

/* used for ioremap to limit vmalloc size */
#define CPROC_VMALLOC_SIZE_LIMIT 4096

#ifdef CONFIG_DEBUG_FS
static const struct file_operations sprd_cproc_debug_fops;
struct dentry *cproc_root;

static void sprd_cproc_debugfs_create(struct cproc_controller *controller);

#endif

enum {
	NORMAL_STATUS = 0,
	STOP_STATUS,
	WDTIRQ_STATUS,
	MAX_STATUS,
};

static int cp_boot_mode;

const char *status_info[] = {
	"started\n",
	"stopped\n",
	"wdtirq\n",
};

static int list_each_dump_info(struct cproc_dump_info *base,
			       struct cproc_dump_info **info)
{
	struct cproc_dump_info *next;

	if (!info)
		return 0;

	next = *info;
	if (!next)
		next = base;
	else
		next++;

	if (next->parent_name[0] != '\0') {
		*info = next;
	} else {
		*info = NULL;
		return 0;
	}

	return 1;
}

static ssize_t sprd_cproc_seg_dump(u32 base,
				   u32 maxsz,
				   char __user *buf,
				   size_t count,
				   loff_t offset)
{
	u32 i;
	u32 start_addr;
	u32 total;
	u32 blk_sz;
	u32 copy_sz;
	void *vmem;

	if (offset >= maxsz)
		return 0;

	if ((offset + count) > maxsz)
		count = maxsz - offset;

	start_addr = base + offset;

	blk_sz = CPROC_VMALLOC_SIZE_LIMIT;
	for (total = count, i = 0; count > 0; i++) {
		u32 addr;

		copy_sz = blk_sz;
		addr = start_addr + blk_sz * i;
		vmem = shmem_ram_vmap_nocache(addr, blk_sz);
		if (!vmem) {
			pr_err(CPROC_TAG "unable to map cproc base: 0x%08x\n",
			       addr);
			if (i > 0)
				return blk_sz * i;
			else
				return -ENOMEM;
		}

		if (count < blk_sz)
			copy_sz = count;

		if (unalign_copy_to_user(buf, vmem, copy_sz)) {
			pr_err(CPROC_TAG "copy mini dump data to user error\n");
			shmem_ram_unmap(vmem);
			return -EFAULT;
		}

		shmem_ram_unmap(vmem);

		count -= copy_sz;
		buf += copy_sz;
	}

	return total;
}

static ssize_t mini_dump_read(struct cproc_proc_entry *entry,
			      char __user *buf,
			      size_t count,
			      loff_t *ppos)
{
	static struct cproc_dump_info *s_cur_info;
	u8 head[sizeof(struct cproc_dump_info) + 32];
	int len, index, total = 0, offset = 0;
	ssize_t written = 0;
	void *vmem;
	struct cproc_segment *seg;
	struct cproc_controller *con;

	if (!s_cur_info && *ppos)
		return 0;

	con = entry->cproc->controller;
	index = entry->index;
	if (index > con->segnr)
		return -EBADF;
	seg = &con->segs[index];
	if (seg->attr != DUMP_FIELD)
		return -EPERM;

	vmem = seg->v_base;

	if (!s_cur_info)
		list_each_dump_info(vmem, &s_cur_info);

	while (s_cur_info) {
		if (!count)
			break;
		len = snprintf(head, sizeof(head), "%s_%s_0x%8x_0x%x.bin",
			       s_cur_info->parent_name, s_cur_info->name,
			       s_cur_info->start_addr, s_cur_info->size);

		if (*ppos > len) {
			offset = *ppos - len;
		} else {
			if (*ppos + count > len)
				written = len - *ppos;
			else
				written = count;

			if (unalign_copy_to_user(buf + total,
						 head + *ppos,
						 written)) {
				pr_err("copy mini-dump data to user error !\n");
				return -EFAULT;
			}
			*ppos += written;
		}
		total += written;
		count -= written;
		if (count) {
			u32 base;

			if (con->map[0] > con->map[1])
				base = s_cur_info->start_addr +
					(con->map[0] - con->map[1]);
			else
				base = s_cur_info->start_addr -
					(con->map[1] - con->map[0]);

			written = sprd_cproc_seg_dump(base,
						      s_cur_info->size,
						      buf + total,
						      count,
						      offset);
			if (written > 0) {
				total += written;
				count -= written;
				*ppos += written;
			} else if (written == 0) {
				if (list_each_dump_info(vmem, &s_cur_info))
					*ppos = 0;
			} else {
				return written;
			}

		} else {
			break;
		}

		written = 0;
		offset = 0;
	}

	*ppos += total;

	return total;
}

static ssize_t load_info_dump(struct cproc_controller *controller,
			      char __user *buf,
			      size_t count,
			      loff_t *ppos)
{
	int i;
	size_t len;
	char *p = buf;
	struct cproc_segment *seg;
	struct load_node lnode;

	for (i = 0; i * sizeof(struct load_node) < *ppos;)
		i++;

	for (len = 0, seg = controller->segs;
	     i < controller->segnr;
	     i++, seg++) {
		if (seg->attr != PARAM_FIELD && seg->attr != LOAD_FIELD)
			continue;

		memset(&lnode, 0, sizeof(lnode));
		pr_info(CPROC_TAG "region[%d] name=%s base=0x%x,size=0x%0x\n",
			i, seg->name, seg->base, seg->size);
		memcpy(lnode.name, seg->name, sizeof(seg->name));
		lnode.base = seg->base;
		lnode.size = seg->size;
		len += sizeof(lnode);

		if (count < len) {
			len -= sizeof(lnode);
			break;
		}

		if (unalign_copy_to_user(p, &lnode, sizeof(lnode))) {
			pr_err(CPROC_TAG "copy load info to user err\n");
			return -EFAULT;
		}

		p += sizeof(lnode);
	}

	*ppos += len;

	return len;
}

int load_cp_boot_code(void *info, void *loader)
{
	int ret = -EAGAIN;
	struct img_desc *i_desc;
	struct iheader_desc *ih_desc;
	union composite_header *c_header;
	struct cp_mem_coupling_info *decoup_data;

	if (!loader || !info) {
		ret = -EINVAL;
		goto err;
	}

	pr_info(CPROC_TAG "load cp boot code, addr = 0x%p\n", loader);

	c_header = (union composite_header *)info;

	ih_desc = &c_header->iheader;
	if (ih_desc->magic != HEADER_MAGIC_INFO) {
		struct boot_info_conf *bconf;

		pr_debug(CPROC_TAG "magic = 0x%x, it's not new format\n",
			 ih_desc->magic);
		decoup_data = &c_header->mem_decoup;
		if (decoup_data->magic != DECOUPLE_MAGIC_INFO) {
			ret = -EMEDIUMTYPE;
			goto err;
		}

		bconf = &decoup_data->boot_info;
		memcpy(loader,
		       bconf->bcode.code,
		       bconf->bcode.count * sizeof(u32));
		return 0;
	}

	for (i_desc = ih_desc->desc; i_desc->size != 0; i_desc++) {
		if (!strcmp("boot-code", i_desc->desc)) {
			pr_debug(CPROC_TAG "boot-code was found\n");
			memcpy(loader,
			       (char *)info + i_desc->offs,
			       i_desc->size);
			return 0;
		}
	}

err:
	pr_err(CPROC_TAG "load cp boot code error, err = %d\n", ret);
	return ret;
}
EXPORT_SYMBOL(load_cp_boot_code);

static int __init early_mode(char *str)
{
	if (!memcmp("shutdown", str, 8))
		cp_boot_mode = 1;

	return 0;
}

early_param("modem", early_mode);

static int sprd_cproc_open(struct inode *inode, struct file *filp)
{
	struct cproc_device *cproc = container_of(filp->private_data,
			struct cproc_device, miscdev);

	filp->private_data = cproc;

	pr_info(CPROC_TAG "open the %s\n", cproc->controller->devname);

	return 0;
}

static int sprd_cproc_release(struct inode *inode, struct file *filp)
{
	struct cproc_device *cproc = filp->private_data;

	pr_info(CPROC_TAG "release the %s\n", cproc->controller->devname);

	return 0;
}

static long sprd_cproc_ioctl(struct file *filp,
			     unsigned int cmd,
			     unsigned long arg)
{
	return 0;
}

static int sprd_cproc_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return 0;
}

static const struct file_operations sprd_cproc_fops = {
	.owner = THIS_MODULE,
	.open = sprd_cproc_open,
	.release = sprd_cproc_release,
	.unlocked_ioctl = sprd_cproc_ioctl,
	.mmap = sprd_cproc_mmap,
};

static int cproc_proc_open(struct inode *inode, struct file *filp)
{
	struct cproc_proc_entry
	*entry = (struct cproc_proc_entry *)PDE_DATA(inode);

	filp->private_data = entry;

	return 0;
}

static int cproc_proc_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t cproc_proc_read(struct file *filp,
			       char __user *buf,
			       size_t count,
			       loff_t *ppos)
{
	int i, rval;
	unsigned index, len;
	u32 size;
	size_t left, blk_sz, copy_sz;
	const char *src;
	char *dest;
	char *name;
	struct cproc_segment *seg;
	struct cproc_proc_entry *entry;
	struct cproc_device *cproc;

	entry = (struct cproc_proc_entry *)filp->private_data;
	cproc = entry->cproc;
	name = entry->name;
	pr_info("enter cproc read, name = %s\n", name);

	if (!strcmp(name, "status")) {
		if (cproc->status >= MAX_STATUS)
			return -EINVAL;

		len = strlen(status_info[cproc->status]);
		if (*ppos >= len)
			return 0;

		count = (len > count) ? count : len;
		if (unalign_copy_to_user(buf,
					 status_info[cproc->status],
					 count)) {
			pr_err(CPROC_TAG "copy status to user error\n");
			return -EFAULT;
		}

	} else if (!strcmp(name, "wdtirq")) {
		struct cproc_controller *controller;

		controller = cproc->controller;
		rval = wait_event_interruptible(controller->wdtwait,
						controller->wdtcnt > 0);
		if (rval < 0)
			pr_err(CPROC_TAG "wdtwait interrupted error\n");

		len = strlen(status_info[WDTIRQ_STATUS]);
		if (*ppos >= len)
			return 0;

		count = (len > count) ? count : len;
		if (unalign_copy_to_user(buf,
					 status_info[WDTIRQ_STATUS],
					 count)) {
			pr_err("copy wdtirq status to user error\n");
			return -EFAULT;
		}

	} else if (!strcmp(name, "ldinfo")) {
		return load_info_dump(cproc->controller, buf, count, ppos);

	} else if (!strcmp(name, "mini_dump")) {
		return mini_dump_read(entry, buf, count, ppos);

	} else {
		if (!strcmp(name, "mem")) {
			src = cproc->controller->v_base;
			size = cproc->controller->maxsz;
		} else {
			index = entry->index;
			if (index > cproc->controller->segnr)
				return -EBADF;

			seg = &cproc->controller->segs[index];
			if (seg->attr != DUMP_FIELD)
				return -EPERM;

			src = seg->v_base;
			size = seg->size;
		}

		if (*ppos >= size)
			return 0;

		if ((*ppos + count) > size)
			count = size - *ppos;

		blk_sz = CPROC_VMALLOC_SIZE_LIMIT;
		left = count;
		src += *ppos;
		dest = buf;
		pr_debug(CPROC_TAG "copyed data size: 0x%x, max size: 0x%x, src: 0x%p\n",
			 (unsigned)*ppos, size, src);

		for (i = 0; left > 0; i++) {
			copy_sz = blk_sz;

			if (left < blk_sz)
				copy_sz = left;

			rval = unalign_copy_to_user(dest, src, copy_sz);
			if (rval) {
				pr_err(CPROC_TAG "copy %s data to user err, src: 0x%p, copyed size: 0x%x, copy_sz: 0x%x, rval: 0x%x, left: 0x%x\n",
				       name,
				       src,
				       (unsigned)*ppos,
				       (unsigned)copy_sz,
				       rval,
				       (unsigned)left);
				return -EFAULT;
			}

			src += copy_sz;
			dest += copy_sz;
			left -= copy_sz;
		}
	}

	*ppos += count;

	return count;
}

static int clflush_cache_segs(struct cproc_controller *controller)
{
	int i;
	struct cproc_segment *seg;

	for (i = 0, seg = controller->segs; i < controller->segnr; i++) {
		if (seg->attr == LOAD_FIELD && seg->attr == PARAM_FIELD)
			clflush_cache_range(seg->v_base, seg->size);
		seg++;
	}

	return 0;
}

static ssize_t cproc_proc_write(struct file *filp,
				const char __user *buf,
				size_t count, loff_t *ppos)
{
	int i, rval;
	unsigned index;
	u32 size;
	size_t left, blk_sz, copy_sz;
	char *dest;
	char *name;
	const char *src;
	struct cproc_segment *seg;
	struct cproc_proc_entry *entry;
	struct cproc_device *cproc;
	static u32 s_bootcode_load_flag;

	entry = (struct cproc_proc_entry *)filp->private_data;
	cproc = entry->cproc;
	name = entry->name;

	pr_debug(CPROC_TAG "enter cproc write, entry name: %s\n", name);

	if (!strcmp(name, "start")) {
		if (cproc->controller->start) {
			clflush_cache_segs(cproc->controller);
			cproc->controller->start(cproc);
		}
		cproc->controller->wdtcnt = 0;
		cproc->status = NORMAL_STATUS;
		return count;

	} else if (!strcmp(name, "stop")) {
		if (cproc->controller->start)
			cproc->controller->stop(cproc);
		cproc->status = STOP_STATUS;
		return count;
	}

	index = entry->index;
	seg = &cproc->controller->segs[index];
	if (seg->attr != LOAD_FIELD && seg->attr != PARAM_FIELD) {
		pr_err(CPROC_TAG "write %s operation not permitted\n", name);
		return -EPERM;
	}

	/* move boot code load from cr5 start */
	if (strstr(seg->name, "modem")) {
		if (*ppos == 0)
			s_bootcode_load_flag = 0;

		if (s_bootcode_load_flag == 0 &&
		    *ppos >= sizeof(union composite_header)) {
			if (load_cp_boot_code(seg->v_base,
					      cproc->controller->v_iram))
				return -EFAULT;
			s_bootcode_load_flag = 1;
			pr_info(CPROC_TAG "copy boot code success\n");
		}
	}

	size = seg->size;

	if (size <= *ppos) {
		pr_info(CPROC_TAG "copyed data size(0x%x) will exceed the max size(0x%x), write %s over!\n",
			(unsigned)*ppos, size, name);
		return 0;
	}

	count = min((size_t)(size - *ppos), count);
	blk_sz = CPROC_VMALLOC_SIZE_LIMIT;
	dest = seg->v_base + *ppos;
	left = count;
	src = buf;
	pr_debug(CPROC_TAG "copyed data size: 0x%x, max size: 0x%x, dest: 0x%p\n",
		 (unsigned)*ppos, size, dest);

	for (i = 0; left > 0; i++) {
		copy_sz = blk_sz;
		if (left < blk_sz)
			copy_sz = left;

		rval = unalign_copy_from_user(dest, src, copy_sz);
		if (rval) {
			pr_err(CPROC_TAG "copy data to %s from user err, start: 0x%p, current: 0x%p, copy_sz: 0x%x, rval: 0x%x, left: 0x%x\n",
			       name,
			       (char *)(seg->v_base + *ppos),
			       dest,
			       (unsigned)copy_sz,
			       rval,
			       (unsigned)left);
			return -EFAULT;
		}

		dest += copy_sz;
		src += copy_sz;
		left -= copy_sz;
	}

	*ppos += count;

	return count;
}

static loff_t cproc_proc_lseek(struct file *filp, loff_t off, int whence)
{
	loff_t new;
	char *name;
	struct cproc_proc_entry *entry;
	struct cproc_device *cproc;

	entry = (struct cproc_proc_entry *)filp->private_data;
	name = entry->name;
	cproc = entry->cproc;

	switch (whence) {
	case SEEK_SET:
		new = off;
		filp->f_pos = new;
		break;

	case SEEK_CUR:
		new = filp->f_pos + off;
		filp->f_pos = new;
		break;

	case SEEK_END:
		if (strcmp(name, "mem") == 0) {
			new = cproc->controller->maxsz + off;
			filp->f_pos = new;
		} else {
			return -EINVAL;
		}
		break;

	default:
		return -EINVAL;
	}

	return new;
}

static unsigned int cproc_proc_poll(struct file *filp, poll_table *wait)
{
	unsigned mask;
	char *name;
	struct cproc_proc_entry *entry;
	struct cproc_device *cproc;

	entry = (struct cproc_proc_entry *)filp->private_data;
	name = entry->name;
	cproc = entry->cproc;

	pr_info(CPROC_TAG "enter cproc poll, name = %s\n", name);

	if (strcmp(name, "wdtirq") == 0) {
		mask = 0;
		poll_wait(filp, &cproc->controller->wdtwait, wait);
		if (cproc->controller->wdtcnt > 0)
			mask |= POLLIN | POLLRDNORM;

	} else {
		pr_err(CPROC_TAG "file don't support poll\n");
		return -EINVAL;
	}

	return mask;
}

static const struct file_operations cpproc_fs_fops = {
	.open		= cproc_proc_open,
	.release	= cproc_proc_release,
	.llseek		= cproc_proc_lseek,
	.read		= cproc_proc_read,
	.write		= cproc_proc_write,
	.poll		= cproc_proc_poll,
};

static inline void sprd_cproc_fs_init(struct cproc_device *cproc)
{
	int index;
	struct cproc_proc_entry *entry;
	struct cproc_segment *seg;

	cproc->procfs.procdir = proc_mkdir(cproc->controller->devname, NULL);

	memset(cproc->procfs.entrys, 0, sizeof(cproc->procfs.entrys));

	entry = cproc->procfs.entrys;

	entry->name = "start";
	entry->mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP;
	entry++;

	entry->name = "stop";
	entry->mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP;
	entry++;

	entry->name = "mem";
	entry->mode = S_IRUSR | S_IRGRP;
	entry++;

	entry->name = "ldinfo";
	entry->mode = S_IRUSR | S_IRGRP;
	entry++;

	entry->name = "status";
	entry->mode = S_IRUSR | S_IRGRP;
	entry++;

	if (cproc->controller->wdtirq > 0) {
		entry->name = "wdtirq";
		entry->mode = S_IRUSR | S_IRGRP;
		entry++;
	}

	seg = cproc->controller->segs;
	for (index = 0; index < cproc->controller->segnr; index++, entry++) {
		entry->name = seg[index].name;
		if (seg[index].attr == DUMP_FIELD)
			entry->mode = S_IRUSR | S_IRGRP | S_IROTH;
		else
			entry->mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP;
		entry->index = index;
	}

	for (entry = cproc->procfs.entrys; entry->name; entry++) {
		pr_info(CPROC_TAG "proc entry name: %s, mode: 0x%x\n",
			entry->name, entry->mode);
		entry->entry = proc_create_data(entry->name, entry->mode,
						cproc->procfs.procdir,
						&cpproc_fs_fops,
						entry);
		entry->cproc = cproc;
	}
}

static inline void sprd_cproc_fs_exit(struct cproc_device *cproc)
{
	u8 i = 0;

	for (i = 0; i < MAX_CPROC_ENTRY_NUM; i++) {
		if (!cproc->procfs.entrys[i].name)
			break;

		remove_proc_entry(cproc->procfs.entrys[i].name,
				  cproc->procfs.procdir);
	}

	remove_proc_entry(cproc->controller->devname, NULL);
}

static void unmap_virtual_addr(struct cproc_controller *controller)
{
	int i;

	if (controller) {
		if (controller->v_base)
			shmem_ram_unmap(controller->v_base);

		if (controller->v_iram)
			shmem_ram_unmap(controller->v_iram);

		for (i = 0; i < controller->segnr; i++) {
			if (controller->segs[i].v_base)
				shmem_ram_unmap(controller->v_iram);
		}
	}
}

static irqreturn_t sprd_cproc_irq_handler(int irq, void *param)
{
	struct cproc_device *cproc;
	struct cproc_controller *controller;

	cproc = (struct cproc_device *)param;
	controller = cproc->controller;

	pr_debug(CPROC_TAG "%s watchdog timeout!\n", controller->devname);
	controller->wdtcnt = 1;
	cproc->status = WDTIRQ_STATUS;
	wake_up_interruptible_all(&controller->wdtwait);

	return IRQ_HANDLED;
}

static int sprd_cproc_parse_dt(struct platform_device *pdev,
			       struct cproc_controller **controller)
{
	int ret, segnr, index;
	u32	tmp[2];
	char *p;
	struct resource res;
	struct cproc_segment *seg;
	struct device_node *np, *c_np;
	struct cproc_controller *con;

	np = pdev->dev.of_node;

	/* calculate segnr number for memory alloction */
	segnr = of_get_child_count(np);
	if (of_find_property(np, "sprd,mini-dump", NULL))
		segnr++;
	if (of_find_property(np, "sprd,aon-iram", NULL))
		segnr++;
	if (of_find_property(np, "sprd,pubcp-iram", NULL))
		segnr++;

	pr_info(CPROC_TAG "segnr = %d\n", segnr);
	con = devm_kzalloc(&pdev->dev,
			   sizeof(struct cproc_controller) +
			   segnr * sizeof(struct cproc_segment),
			   GFP_KERNEL);
	if (!con)
		return -ENOMEM;

	ret = of_property_read_string(np, "sprd,name", (const char **)&p);
	if (ret)
		goto err;
	strncpy(con->devname, p, sizeof(con->devname) - 1);

	ret = of_property_read_u32_array(np, "sprd,ap-cp-addr-map", tmp, 2);
	if (!ret) {
		con->map[0] = tmp[0];
		con->map[1] = tmp[1];
	}

	/* get cp base */
	ret = of_address_to_resource(np, 0, &res);
	if (ret)
		goto err;
	con->base = res.start;
	con->maxsz = res.end - res.start + 1;
	con->v_base = shmem_ram_vmap_nocache(con->base, con->maxsz);
	if (!con->v_base) {
		pr_err(CPROC_TAG "vmap the base failed, base: 0x%lx\n",
		       (unsigned long)con->base);
		goto err;
	}
	pr_info(CPROC_TAG "cp base = 0x%p, size = 0x%x\n",
		(void *)con->base, con->maxsz);

	/* get iram info */
	ret = of_property_read_u32_array(np, "sprd,iram", tmp, 2);
	if (ret == 0) {
		con->iram = tmp[0];
		con->iramsz = tmp[1];
		con->v_iram = shmem_ram_vmap_nocache(con->iram, con->iramsz);
		if (!con->v_iram) {
			pr_err(CPROC_TAG "vmap the iram failed, base: 0x%lx\n",
			       (unsigned long)con->iram);
			goto err;
		}
		pr_info(CPROC_TAG "iram = 0x%lx, size = 0x%x\n",
			(unsigned long)con->iram, con->iramsz);
	}

	/* get mini-dump info */
	index = 0;
	if (of_property_read_u32_array(np, "sprd,mini-dump", tmp, 2) == 0) {
		seg = &con->segs[index];
		seg->base = tmp[0];
		seg->size = tmp[1];
		seg->attr = DUMP_FIELD;
		memcpy(con->segs[index].name, "mini_dump", strlen("mini_dump"));
		seg->v_base = shmem_ram_vmap_nocache(seg->base, seg->size);
		if (!seg->v_base) {
			pr_err(CPROC_TAG "vmap the %s failed, base: 0x%lx\n",
			       seg->name, (unsigned long)seg->base);
			goto err;
		}
		pr_info(CPROC_TAG "name = %s, base = 0x%x, size = 0x%x\n",
			seg->name, seg->base, seg->size);
		index++;
	}

	/* get ano-iram info */
	if (of_property_read_u32_array(np, "sprd,aon-iram", tmp, 2) == 0)  {
		seg = &con->segs[index];
		seg->base = tmp[0];
		seg->size = tmp[1];
		seg->attr = DUMP_FIELD;
		strcpy(con->segs[index].name, "aon-iram");
		seg->v_base = shmem_ram_vmap_nocache(seg->base, seg->size);
		if (!seg->v_base) {
			pr_err(CPROC_TAG "vmap the %s failed, base: 0x%lx\n",
			       seg->name, (unsigned long)seg->base);
			goto err;
		}
		pr_info(CPROC_TAG "name = %s, base = 0x%x, size = 0x%x\n",
			seg->name, seg->base, seg->size);
		index++;
	}

	/* get pubcp-iram info */
	if (of_property_read_u32_array(np, "sprd,pubcp-iram", tmp, 2) == 0)  {
		seg = &con->segs[index];
		seg->base = tmp[0];
		seg->size = tmp[1];
		seg->attr = DUMP_FIELD;
		strcpy(con->segs[index].name, "pubcp-iram");
		seg->v_base = shmem_ram_vmap_nocache(seg->base, seg->size);
		if (!seg->v_base) {
			pr_err(CPROC_TAG "vmap the %s failed, base: 0x%lx\n",
			       seg->name, (unsigned long)seg->base);
			goto err;
		}
		pr_info(CPROC_TAG "name = %s, base = 0x%x, size = 0x%x\n",
			seg->name, seg->base, seg->size);
		index++;
	}

	/* get irq */
	con->wdtirq = irq_of_parse_and_map(np, 0);
	pr_info(CPROC_TAG "wdt irq %u\n", con->wdtirq);

	/* get load info */
	for_each_child_of_node(np, c_np) {
		seg = &con->segs[index];
		ret = of_address_to_resource(c_np, 0, &res);
		if (ret)
			goto err;

		seg->base = res.start;
		seg->size = res.end - res.start + 1;
		seg->v_base = shmem_ram_vmap_cache(seg->base, seg->size);
		if (!seg->v_base) {
			pr_err(CPROC_TAG "vmap the %s failed, base: 0x%lx\n",
			       seg->name, (unsigned long)seg->base);
			goto err;
		}

		ret = of_property_read_string(c_np,
					      "cproc,name",
					      (const char **)&p);
		if (!ret)
			strncpy(seg->name, p, sizeof(seg->name) - 1);
		else
			strncpy(seg->name, res.name, sizeof(seg->name) - 1);

		if (strstr(seg->name, "cmdline"))
			seg->attr = PARAM_FIELD;
		else
			seg->attr = LOAD_FIELD;

		pr_info(CPROC_TAG "name = %s, base = 0x%x, size = 0x%x\n",
			seg->name, seg->base, seg->size);
		index++;
	}

	con->segnr = segnr;
	*controller = con;

	return 0;

err:
	unmap_virtual_addr(con);

	return -1;
}

static const struct of_device_id sprd_cproc_match_table[] = {
	{ .compatible = "sprd,scproc_pubcp", .data = sprd_cproc_parse_dt },
};

static int sprd_cproc_probe(struct platform_device *pdev)
{
	int rval;
	struct cproc_controller *controller;
	struct cproc_device *cproc;
	const struct of_device_id *of_id;
	int (*parse)(struct platform_device *, struct cproc_controller **);

	of_id = of_match_node(sprd_cproc_match_table, pdev->dev.of_node);
	if (!of_id) {
		pr_err(CPROC_TAG "failed to get of_id\n");
		return -ENODEV;
	}
	parse = (int (*)(struct platform_device *,
			 struct cproc_controller **))of_id->data;
	if (parse && parse(pdev, &controller)) {
		pr_err(CPROC_TAG "failed to parse dt, parse(0x%p)\n", parse);
		return -ENODEV;
	}

	cproc = devm_kzalloc(&pdev->dev,
			     sizeof(struct cproc_device),
			     GFP_KERNEL);
	if (!cproc)
		return -ENOMEM;

	pr_info(CPROC_TAG "cp boot mode: 0x%x\n", cp_boot_mode);
	if (cp_boot_mode && !strstr(controller->devname, "ag"))
		cproc->status = STOP_STATUS;
	else
		cproc->status = NORMAL_STATUS;

	controller->wdtcnt = 0;

	cproc->controller = controller;

	/* register wdt irq */
	if (controller->wdtirq > 32) {
		init_waitqueue_head(&controller->wdtwait);
		rval = request_irq(controller->wdtirq,
				   sprd_cproc_irq_handler,
				   0,
				   controller->devname,
				   cproc);
		if (rval != 0)
			return rval;
	}

	cproc->miscdev.minor = MISC_DYNAMIC_MINOR;
	cproc->miscdev.name = cproc->controller->devname;
	cproc->miscdev.fops = &sprd_cproc_fops;
	cproc->miscdev.parent = NULL;
	rval = misc_register(&cproc->miscdev);
	if (rval) {
		pr_err(CPROC_TAG "register miscdev failed\n");
		return rval;
	}

	sprd_cproc_fs_init(cproc);

	register_cproc_controller(cproc->controller);

	cproc->pdev = pdev;

	platform_set_drvdata(pdev, cproc);

#ifdef CONFIG_DEBUG_FS
	sprd_cproc_debugfs_create(cproc->controller);
#endif

	pr_info(CPROC_TAG "%s probe ok\n", cproc->controller->devname);

	return 0;
}

static int sprd_cproc_remove(struct platform_device *pdev)
{
	struct cproc_device *cproc = platform_get_drvdata(pdev);

	sprd_cproc_fs_exit(cproc);

	misc_deregister(&cproc->miscdev);

	unmap_virtual_addr(cproc->controller);

	pr_info(CPROC_TAG "%s remove\n", cproc->controller->devname);

	return 0;
}

static struct platform_driver sprd_cproc_driver = {
	.probe    = sprd_cproc_probe,
	.remove   = sprd_cproc_remove,
	.driver   = {
		.owner = THIS_MODULE,
		.name = "sprd_cproc",
		.of_match_table = sprd_cproc_match_table,
	},
};

static int __init sprd_cproc_init(void)
{
	if (platform_driver_register(&sprd_cproc_driver) != 0) {
		pr_err(CPROC_TAG "cproc platform drv register failed\n");
		return -1;
	}

	return 0;
}

static void __exit sprd_cproc_exit(void)
{
	platform_driver_unregister(&sprd_cproc_driver);
}

#if defined(CONFIG_DEBUG_FS)
static int sprd_cproc_debug_show(struct seq_file *m, void *private)
{
	int i;
	struct cproc_controller *controller;
	struct cproc_segment *seg;

	controller = (struct cproc_controller *)m->private;

	seq_puts(m, "********************************************************************************\n");
	seq_printf(m, "cproc %s info:\n", controller->devname);
	seq_printf(m, "ap: 0x%lx <==> cp: 0x%lx\n",
		   (unsigned long)controller->map[0],
		   (unsigned long)controller->map[1]);
	seq_printf(m, "v_base: 0x%p, base: 0x%lx, size: 0x%x\n",
		   controller->v_base,
		   (unsigned long)controller->base,
		   controller->maxsz);
	if (controller->iramsz > 0)
		seq_printf(m, "v_iram: 0x%p, iram: 0x%lx, size: 0x%x\n",
			   controller->v_iram,
			   (unsigned long)controller->iram,
			   controller->iramsz);

	for (i = 0, seg = controller->segs; i < controller->segnr; i++) {
		seq_printf(m, "seg[%d]: name: %s, v_base: 0x%p, base: 0x%lx, size: 0x%x, attr: 0x%x\n",
			   i,
			   seg->name,
			   seg->v_base,
			   (unsigned long)seg->base,
			   seg->size,
			   seg->attr);
		seg++;
	}

	if (controller->wdtirq > 0)
		seq_printf(m, "wdtirq: 0x%x, wdtcnt: 0x%x", controller->wdtirq,
			   controller->wdtcnt);

	seq_puts(m, "\n********************************************************************************\n");

	return 0;
}

static int sprd_cproc_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, sprd_cproc_debug_show, inode->i_private);
}

static const struct file_operations sprd_cproc_debug_fops = {
	.open = sprd_cproc_debug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void sprd_cproc_debugfs_create(struct cproc_controller *controller)
{
	if (!cproc_root)
		cproc_root = debugfs_create_dir("cproc", NULL);

	if (cproc_root) {
		debugfs_create_file(controller->devname,
				    S_IRUGO,
				    cproc_root,
				    controller,
				    &sprd_cproc_debug_fops);
	}
}
#endif /* CONFIG_DEBUG_FS */

module_init(sprd_cproc_init);
module_exit(sprd_cproc_exit);

MODULE_DESCRIPTION("SPRD Communication Processor Driver");
MODULE_LICENSE("GPL");

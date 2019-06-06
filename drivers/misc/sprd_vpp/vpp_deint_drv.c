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

#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/sprd_iommu.h>
#include <linux/sprd_ion.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <linux/compat.h>
#include <uapi/video/vpp_deint_drv.h>

#ifndef VM_RESERVED
#define  VM_RESERVED   (VM_DONTEXPAND | VM_DONTDUMP)
#endif

#define SPRD_MMAHB_BASE_DT SPRD_MMAHB_BASE

#define VPP_MINOR MISC_DYNAMIC_MINOR
#define VPP_AQUIRE_TIMEOUT_MS 500
#define VPP_INIT_TIMEOUT_MS 200

#define DEFAULT_FREQ_DIV 0x0

static DEFINE_SEMAPHORE(s_vpp_sem);

struct deint_fh {
	int is_deint_acquired;
	int is_clock_enabled;

	wait_queue_head_t wait_queue_work;
	int condition_work;
	int deint_int_status;
};

struct sprd_vpp_cfg_data {
	unsigned int version;
	unsigned int max_freq_level;
	unsigned int softreset_reg_offset;
	unsigned int reset_mask;
};

struct deint_dev_t {
	unsigned int freq_div;

	struct semaphore deint_mutex;
	struct sprd_vpp_cfg_data *vpp_cfg_data;

	struct clk *vsp_clk;
	struct clk *vsp_parent_clk;
	struct clk *clk_mm_eb;
	struct clk *clk_vsp_ckg;
	struct clk *clk_axi_gate_vsp;
	struct clk *clk_ahb_gate_vsp_eb;
	struct clk *clk_ahb_vsp;
	struct clk *ahb_clk_parent;

	unsigned int irq;
	unsigned int version;

	struct deint_fh *deint_fp;
	struct device_node *dev_np;
	struct device *deint_dev;
	bool light_sleep_en;
	bool iommu_exist_flag;
};

static struct deint_dev_t deint_hw_dev;
static struct wake_lock deint_wakelock;
static atomic_t deint_instance_cnt = ATOMIC_INIT(0);

static unsigned long SPRD_VPP_PHYS;
static void __iomem *SPRD_VPP_BASE;

struct clock_name_map_t {
	unsigned long freq;
	char *name;
	struct clk *clk_parent;
};

static struct clock_name_map_t clock_name_map[DEINT_CLK_LEVEL_NUM];

static int max_freq_level;

static int vpp_clk_enable(struct deint_dev_t *deint_hw_dev)
{
	int ret = 0;

	ret = clk_prepare_enable(deint_hw_dev->clk_mm_eb);
	if (ret) {
		pr_err("vpp clk_mm_eb: clk_prepare_enable failed!\n");
		return ret;
	}
	pr_debug("vpp clk_mm_eb: clk_prepare_enable ok.\n");

	ret = clk_prepare_enable(deint_hw_dev->clk_vsp_ckg);
	if (ret) {
		pr_err("clk_vpp_ckg: clk_prepare_enable failed!\n");
		return ret;
	}
	pr_debug("clk_vpp_ckg: clk_prepare_enable ok.\n");

	if (deint_hw_dev->clk_axi_gate_vsp) {
		ret = clk_prepare_enable(deint_hw_dev->clk_axi_gate_vsp);
		if (ret) {
			pr_err("clk_axi_gate_vpp: clk_prepare_enable failed");
			return ret;
		}
		pr_debug("clk_axi_gate_vpp: clk_prepare_enable ok");
	}

	ret = clk_prepare_enable(deint_hw_dev->clk_ahb_gate_vsp_eb);
	if (ret) {
		pr_err("clk_ahb_gate_vpp_eb: clk_prepare_enable failed!\n");
		return ret;
	}
	pr_debug("clk_ahb_gate_vpp_eb: clk_prepare_enable ok.\n");

	ret =
	    clk_set_parent(deint_hw_dev->vsp_clk, deint_hw_dev->vsp_parent_clk);
	if (ret) {
		pr_err("clock[%s]: clk_set_parent() failed!", "clk_vsp");
		return -EINVAL;
	}

	ret = clk_prepare_enable(deint_hw_dev->vsp_clk);
	if (ret) {
		pr_err("vpp_clk: clk_prepare_enable failed!\n");
		return ret;
	}
	pr_debug("vpp_clk: clk_prepare_enable ok.\n");

	ret = clk_prepare_enable(deint_hw_dev->clk_ahb_vsp);
	if (ret) {
		pr_err("clk_ahb_vpp: clk_prepare_enable failed!\n");
		return ret;
	}
	pr_debug("clk_ahb_vpp: clk_prepare_enable ok.\n");

	return ret;

}

static void vpp_clk_disable(struct deint_dev_t *deint_hw_dev)
{
	clk_disable_unprepare(deint_hw_dev->clk_ahb_vsp);
	clk_disable_unprepare(deint_hw_dev->vsp_clk);
	clk_disable_unprepare(deint_hw_dev->clk_ahb_gate_vsp_eb);
	if (deint_hw_dev->clk_axi_gate_vsp)
		clk_disable_unprepare(deint_hw_dev->clk_axi_gate_vsp);
	clk_disable_unprepare(deint_hw_dev->clk_vsp_ckg);
	clk_disable_unprepare(deint_hw_dev->clk_mm_eb);
}

#ifdef CONFIG_COMPAT
static int compat_get_mmu_map_data(struct compat_vsp_iommu_map_data __user
				   *data32,
				   struct vsp_iommu_map_data __user *data)
{
	compat_int_t i;
	compat_size_t s;
	compat_ulong_t ul;
	int err;

	err = get_user(i, &data32->fd);
	err |= put_user(i, &data->fd);
	err |= get_user(s, &data32->size);
	err |= put_user(s, &data->size);
	err |= get_user(ul, &data32->iova_addr);
	err |= put_user(ul, &data->iova_addr);

	return err;
};

static int compat_put_mmu_map_data(struct compat_vsp_iommu_map_data __user
				   *data32,
				   struct vsp_iommu_map_data __user *data)
{
	compat_int_t i;
	compat_size_t s;
	compat_ulong_t ul;
	int err;

	err = get_user(i, &data->fd);
	err |= put_user(i, &data32->fd);
	err |= get_user(s, &data->size);
	err |= put_user(s, &data32->size);
	err |= get_user(ul, &data->iova_addr);
	err |= put_user(ul, &data32->iova_addr);

	return err;
};

static long compat_deint_ioctl(struct file *filp, unsigned int cmd,
			       unsigned long arg)
{
	long ret = 0;
	int err;
	struct compat_vsp_iommu_map_data __user *data32;
	struct vsp_iommu_map_data __user *data;

	struct deint_fh *deint_fp = filp->private_data;

	if (!filp->f_op->unlocked_ioctl)
		return -ENOTTY;

	if (deint_fp == NULL) {
		pr_err("%s, vsp_ioctl error occurred, vsp_fp == NULL\n",
		       __func__);
		return -EINVAL;
	}

	switch (cmd) {
	case COMPAT_VPP_GET_IOVA:

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL) {
			pr_err("%s %d, compat_alloc_user_space failed",
			       __func__, __LINE__);
			return -EFAULT;
		}

		err = compat_get_mmu_map_data(data32, data);
		if (err) {
			pr_err("%s %d, compat_get_mmu_map_data failed",
			       __func__, __LINE__);
			return err;
		}
		ret = filp->f_op->unlocked_ioctl(filp, SPRD_VPP_GET_IOVA,
						 (unsigned long)data);
		err = compat_put_mmu_map_data(data32, data);
		return ret ? ret : err;

	case COMPAT_VPP_FREE_IOVA:

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL) {
			pr_err("%s %d, compat_alloc_user_space failed",
			       __func__, __LINE__);
			return -EFAULT;
		}

		err = compat_get_mmu_map_data(data32, data);
		if (err) {
			pr_err("%s %d, compat_get_mmu_map_data failed",
			       __func__, __LINE__);
			return err;
		}
		ret = filp->f_op->unlocked_ioctl(filp, SPRD_VPP_FREE_IOVA,
						 (unsigned long)data);
		err = compat_put_mmu_map_data(data32, data);
		return ret ? ret : err;

	default:
		return filp->f_op->unlocked_ioctl(filp, cmd, (unsigned long)
						  compat_ptr(arg));
	}

	return ret;
}
#endif

static int vpp_get_iova(struct deint_dev_t *deint_hw_dev,
		 struct vsp_iommu_map_data *mapdata, void __user *arg)
{
	int ret = 0;
	struct sprd_iommu_map_data iommu_map_data;

	ret = sprd_ion_get_buffer(mapdata->fd, NULL,
				    &(iommu_map_data.buf),
				    &iommu_map_data.iova_size);
	if (0 != ret) {
		pr_err("get_sg_table failed, ret %d\n", ret);
		return ret;
	}

	iommu_map_data.ch_type = SPRD_IOMMU_FM_CH_RW;
	ret = sprd_iommu_map(deint_hw_dev->deint_dev, &iommu_map_data);
	if (!ret) {
		mapdata->iova_addr = iommu_map_data.iova_addr;
		mapdata->size = iommu_map_data.iova_size;
		ret = copy_to_user((void __user *)arg, (void *)mapdata,
				   sizeof(struct vsp_iommu_map_data));
		if (0 != ret) {
			pr_err("copy_to_user failed, ret %d\n", ret);
			return -EFAULT;
		}
	} else
		pr_err("vsp iommu map failed, ret %d, map size 0x%zx\n",
		       ret, iommu_map_data.iova_size);

	return ret;
}

static int vpp_free_iova(struct deint_dev_t *deint_hw_dev,
		  struct vsp_iommu_map_data *ummapdata)
{
	int ret = 0;
	struct sprd_iommu_unmap_data iommu_ummap_data;

	iommu_ummap_data.iova_addr = ummapdata->iova_addr;
	iommu_ummap_data.iova_size = ummapdata->size;
	iommu_ummap_data.ch_type = SPRD_IOMMU_FM_CH_RW;
	iommu_ummap_data.buf = NULL;

	ret = sprd_iommu_unmap(deint_hw_dev->deint_dev, &iommu_ummap_data);

	if (ret)
		pr_err("vsp iommu-unmap failed ret %d addr&size 0x%lx 0x%zx\n",
		       ret, ummapdata->iova_addr, ummapdata->size);
	return ret;
}

static long deint_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret;
	struct vsp_iommu_map_data mapdata;
	struct vsp_iommu_map_data ummapdata;
	struct deint_fh *deint_fp = filp->private_data;

	if (deint_fp == NULL) {
		pr_err("deint_ioctl error occurred, deint_fp == NULL\n");
		return -EINVAL;
	}

	switch (cmd) {
	case SPRD_VPP_DEINT_COMPLETE:
		{
			ret =
			    wait_event_interruptible_timeout
			    (deint_fp->wait_queue_work,
			     deint_fp->condition_work,
			     msecs_to_jiffies(VPP_INIT_TIMEOUT_MS));

			if (ret == -ERESTARTSYS) {
				pr_err("deint complete -ERESTARTSYS\n");
				put_user(ret, (int __user *)arg);
				ret = -EINVAL;
			} else {
				if (ret == 0) {
					pr_err("vpp complete  timeout\n");
					ret = -ETIMEDOUT;

					/*clear vpp int */
					writel_relaxed((1 << 1),
						(SPRD_VPP_BASE + VPP_INT_CLR));
				}

				put_user(ret, (int __user *)arg);
				deint_fp->condition_work = 0;
			}

			break;
		}

	case SPRD_VPP_DEINT_ACQUIRE:
		{
			pr_debug("deint ioctl SPRD_VPP_DEINT_ACQUIRE begin\n");
			wake_lock(&deint_wakelock);
			ret =
			    down_timeout(&deint_hw_dev.deint_mutex,
					 msecs_to_jiffies
					 (VPP_AQUIRE_TIMEOUT_MS));
			if (ret) {
				pr_err("deint error timeout\n");
				return ret;
			}

			deint_hw_dev.deint_fp = deint_fp;
			deint_fp->is_deint_acquired = 1;

			ret = vpp_clk_enable(&deint_hw_dev);
			if (ret) {
				pr_err
				    ("vpp_clk: clk_prepare_enable() failed!\n");
				return ret;
			}

			deint_fp->is_clock_enabled = 1;

			break;
		}

	case SPRD_VPP_DEINT_RELEASE:
		{
			pr_debug("SPRD_VPP_DEINT_RELEASE\n");

			if (1 == deint_fp->is_clock_enabled)
				vpp_clk_disable(&deint_hw_dev);

			deint_fp->is_clock_enabled = 0;
			deint_fp->is_deint_acquired = 0;
			deint_hw_dev.deint_fp = NULL;

			up(&deint_hw_dev.deint_mutex);
			wake_unlock(&deint_wakelock);

			break;
		}

	case SPRD_VPP_GET_IOMMU_STATUS:
		{

			ret = sprd_iommu_attach_device(deint_hw_dev.deint_dev);

			break;
		}

	case SPRD_VPP_GET_IOVA:
		{
			ret =
			    copy_from_user((void *)&mapdata,
					   (const void __user *)arg,
					   sizeof(struct vsp_iommu_map_data));
			if (0 != ret) {
				pr_err("copy mapdata failed, ret %d\n", ret);
				return -EFAULT;
			}

			ret = vpp_get_iova(&deint_hw_dev, &mapdata,
					   (void __user *)arg);

			break;
		}

	case SPRD_VPP_FREE_IOVA:
		{
			ret =
			    copy_from_user((void *)&ummapdata,
					   (const void __user *)arg,
					   sizeof(struct vsp_iommu_map_data));
			if (0 != ret) {
				pr_err("copy ummapdata failed, ret %d\n", ret);
				return -EFAULT;
			}

			ret = vpp_free_iova(&deint_hw_dev, &ummapdata);

			break;
		}
	case SPRD_VPP_RESET:
		pr_debug("vpp ioctl SPRD_VPP_RESET\n");
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static irqreturn_t deint_isr(int irq, void *data)
{
	int ret = 0xff;		/* 0xff : invalid */
	struct deint_fh *deint_fp = ((struct deint_dev_t *)data)->deint_fp;

	if (NULL == deint_fp)
		return IRQ_NONE;

	ret = readl_relaxed((SPRD_VPP_BASE + VPP_INT_STS));
	if ((ret >> 1) & 0x1)
		writel_relaxed((1 << 1), (SPRD_VPP_BASE + VPP_INT_CLR));
	else
		return IRQ_NONE;

	deint_fp->condition_work = 1;
	wake_up_interruptible(&deint_fp->wait_queue_work);

	return IRQ_HANDLED;
}

static int vpp_get_mm_clk(struct device *deint_dev)
{
	struct clk *clk_mm_eb;
	struct clk *clk_vsp_ckg;
	struct clk *clk_axi_gate_vsp;
	struct clk *clk_ahb_gate_vsp_eb;
	struct clk *clk_vsp;
	struct clk *clk_ahb_vsp;
	int instance_cnt = atomic_read(&deint_instance_cnt);

	pr_info("vpp_get_mm_clk: vpp_instance_cnt %d\n", instance_cnt);

	clk_mm_eb = devm_clk_get(deint_dev, "clk_mm_eb");
	if (IS_ERR_OR_NULL(clk_mm_eb)) {
		pr_err("Failed: Can't get clock [%s]! %p\n", "clk_mm_eb",
		       clk_mm_eb);
		deint_hw_dev.clk_mm_eb = NULL;
		return -EINVAL;
	}

	deint_hw_dev.clk_mm_eb = clk_mm_eb;

	clk_axi_gate_vsp = devm_clk_get(deint_dev, "clk_axi_gate_vsp");
	if (IS_ERR_OR_NULL(clk_axi_gate_vsp)) {
		pr_err("Failed: Can't get clock [%s]! %p\n", "clk_axi_gate_vsp",
		       clk_axi_gate_vsp);
		deint_hw_dev.clk_axi_gate_vsp = NULL;
		return -EINVAL;
	}

	deint_hw_dev.clk_axi_gate_vsp = clk_axi_gate_vsp;
	if (deint_hw_dev.version == IWHALE2)
		deint_hw_dev.clk_axi_gate_vsp = NULL;

	clk_vsp_ckg = devm_clk_get(deint_dev, "clk_vsp_ckg");
	if (IS_ERR_OR_NULL(clk_vsp_ckg)) {
		pr_err("Failed: Can't get clock [%s]! %p\n",
		       "clk_vsp_ckg", clk_vsp_ckg);
		deint_hw_dev.clk_vsp_ckg = NULL;
		return -EINVAL;
	}

	deint_hw_dev.clk_vsp_ckg = clk_vsp_ckg;

	clk_ahb_gate_vsp_eb = devm_clk_get(deint_dev, "clk_ahb_gate_vsp_eb");
	if (IS_ERR_OR_NULL(clk_ahb_gate_vsp_eb)) {
		pr_err("Failed: Can't get clock [%s]! %p\n",
		       "clk_ahb_gate_vsp_eb", clk_ahb_gate_vsp_eb);
		deint_hw_dev.clk_ahb_gate_vsp_eb = NULL;
		return -EINVAL;
	}

	deint_hw_dev.clk_ahb_gate_vsp_eb = clk_ahb_gate_vsp_eb;

	clk_vsp = devm_clk_get(deint_dev, "clk_vsp");
	if (IS_ERR_OR_NULL(clk_vsp)) {
		pr_err("Failed: Can't get clock [%s]! %p\n", "clk_vsp",
		       clk_vsp);
		deint_hw_dev.vsp_clk = NULL;
		return -EINVAL;
	}

	deint_hw_dev.vsp_clk = clk_vsp;

	clk_ahb_vsp = devm_clk_get(deint_dev, "clk_ahb_vsp");
	if (IS_ERR_OR_NULL(clk_ahb_vsp)) {
		pr_err("Failed: Can't get clock [%s]! %p\n",
		       "clk_ahb_vsp", clk_ahb_vsp);
		deint_hw_dev.clk_ahb_vsp = NULL;
		return -EINVAL;
	}

	deint_hw_dev.clk_ahb_vsp = clk_ahb_vsp;

	return 0;
}

static int deint_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct deint_fh *deint_fp =
	    kmalloc(sizeof(struct deint_fh), GFP_KERNEL);

	if (deint_fp == NULL) {
		pr_err("deint open error occurred\n");
		return -EINVAL;
	}
	filp->private_data = deint_fp;
	deint_fp->is_clock_enabled = 0;
	deint_fp->is_deint_acquired = 0;
	deint_fp->condition_work = 0;
	deint_fp->deint_int_status = 0;

	init_waitqueue_head(&deint_fp->wait_queue_work);

	ret = vpp_get_mm_clk(deint_hw_dev.deint_dev);

	atomic_inc_return(&deint_instance_cnt);
	pr_info("deint_open: ret %d\n", ret);

	return ret;

}

static int deint_release(struct inode *inode, struct file *filp)
{
	struct deint_fh *deint_fp = filp->private_data;
	int instance_cnt = atomic_read(&deint_instance_cnt);

	if (deint_fp == NULL) {
		pr_err("deint_release error occurred, deint_fp == NULL\n");
		return -EINVAL;
	}

	if (deint_fp->is_deint_acquired) {
		pr_err("error occurred and up deint_mutex\n");
		up(&deint_hw_dev.deint_mutex);
	}

	kfree(filp->private_data);
	filp->private_data = NULL;

	atomic_dec_return(&deint_instance_cnt);
	pr_info("deint_release: instance_cnt = %d\n", instance_cnt);

	return 0;
}

static int deint_map_to_register(struct file *fp, struct vm_area_struct *vm)
{
	unsigned long pfn;

	vm->vm_flags |= VM_IO | VM_RESERVED;
	vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);
	pfn = SPRD_VPP_PHYS >> PAGE_SHIFT;

	return remap_pfn_range(vm, vm->vm_start, pfn, vm->vm_end - vm->vm_start,
			       vm->vm_page_prot) ? -EAGAIN : 0;
}

static int deint_map_to_physical_memory(struct file *fp,
					struct vm_area_struct *vm)
{
	vm->vm_flags |= VM_IO | VM_RESERVED;
	vm->vm_page_prot = pgprot_writecombine(vm->vm_page_prot);

	return remap_pfn_range(vm, vm->vm_start, vm->vm_pgoff,
			       vm->vm_end - vm->vm_start,
			       vm->vm_page_prot) ? -EAGAIN : 0;
}

static int deint_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret = 0;

	pr_debug("@deint[%s], vm_pgoff = %ld\n", __func__, vma->vm_pgoff);

	if (vma->vm_pgoff)
		ret = deint_map_to_physical_memory(filp, vma);
	else
		ret = deint_map_to_register(filp, vma);

	pr_debug("@deint mmap start: %p,size: %ld\n",
		 (void *)vma->vm_start,
		 (unsigned long)(vma->vm_end - vma->vm_start));

	return 0;
}

static const struct file_operations deint_fops = {
	.owner = THIS_MODULE,
	.mmap = deint_mmap,
	.open = deint_open,
	.release = deint_release,
	.unlocked_ioctl = deint_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = compat_deint_ioctl,
#endif
};

static struct miscdevice deint_dev = {
	.minor = VPP_MINOR,
	.name = "sprd_vpp",
	.fops = &deint_fops,
};

static const struct sprd_vpp_cfg_data whale2_vpp_data = {
	.version = WHALE2,
	.max_freq_level = 4,
	.softreset_reg_offset = 0x4,
	.reset_mask = BIT(5),
};

static const struct sprd_vpp_cfg_data iwhale2_vpp_data = {
	.version = IWHALE2,
	.max_freq_level = 4,
	.softreset_reg_offset = 0x4,
	.reset_mask = BIT(5),
};

static const struct of_device_id of_match_table_vpp[] = {
	{.compatible = "sprd,whale2-vpp", .data = &whale2_vpp_data},
	{.compatible = "sprd,iwhale2-vpp", .data = &iwhale2_vpp_data},
	{},
};

static int deint_parse_dt(struct platform_device *pdev)
{
	struct device *dev = &(pdev->dev);
	struct device_node *np = dev->of_node;
	struct device_node *deint_clk_np = NULL;
	const char *clk_compitale = "sprd,muxed-clock";
	const char *clk_node_name = NULL;
	struct resource *res;
	int i, clk_count = 0;

	pr_info("deint_parse_dt called !\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no reg of property specified\n");
		pr_err("vsp: failed to parse_dt!\n");
		return -EINVAL;
	}

	SPRD_VPP_PHYS = res->start;
	SPRD_VPP_BASE = devm_ioremap_resource(&pdev->dev, res);
	WARN_ON(!SPRD_VPP_BASE);

	deint_hw_dev.irq = platform_get_irq(pdev, 0);
	deint_hw_dev.dev_np = np;
	deint_hw_dev.deint_dev = dev;

	deint_hw_dev.version = deint_hw_dev.vpp_cfg_data->version;
	max_freq_level = deint_hw_dev.vpp_cfg_data->max_freq_level;

	pr_debug(
		"SPRD_VPP_PHYS = 0x%lx,SPRD_VPP_BASE = %p, irq = %d\n",
		SPRD_VPP_PHYS, SPRD_VPP_BASE, deint_hw_dev.irq);

	for_each_compatible_node(deint_clk_np, NULL, clk_compitale) {
		if (of_property_read_string_index
		    (deint_clk_np, "clock-output-names", 0, &clk_node_name) < 0)
			continue;
		if (!strcmp(clk_node_name, "clk_vpp")) {
			pr_info("clk [%s], device node: %p\n", clk_node_name,
				deint_clk_np);
			break;
		}
	}

	clk_count = of_clk_get_parent_count(deint_clk_np);
	if (clk_count != max_freq_level) {
		pr_err("failed to get vsp clock count\n");
		return -EINVAL;
	}

	for (i = 0; i < clk_count; i++) {
		struct clk *clk_parent;
		char *name_parent;
		unsigned long frequency;

		name_parent = (char *)of_clk_get_parent_name(deint_clk_np, i);
		clk_parent = of_clk_get(deint_clk_np, i);
		frequency = clk_get_rate(clk_parent);
		pr_info("vsp clk in dts file: clk[%d] = (%ld, %s)\n", i,
			frequency, name_parent);

		clock_name_map[i].name = name_parent;
		clock_name_map[i].freq = frequency;
		clock_name_map[i].clk_parent = clk_parent;
	}

	return 0;
}

static int deint_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node *node = pdev->dev.of_node;
	const struct of_device_id *of_id;

	pr_info("deint_probe called !\n");

	of_id = of_match_node(of_match_table_vpp, node);
	if (of_id)
		deint_hw_dev.vpp_cfg_data =
		    (struct sprd_vpp_cfg_data *)of_id->data;
	else
		panic("%s: Not find matched id!", __func__);

	if (pdev->dev.of_node) {
		if (deint_parse_dt(pdev)) {
			pr_err("deint_parse_dt failed\n");
			return -EINVAL;
		}
	}

	deint_hw_dev.vsp_clk = NULL;
	deint_hw_dev.clk_ahb_vsp = NULL;
	deint_hw_dev.clk_mm_eb = NULL;
	deint_hw_dev.clk_axi_gate_vsp = NULL;
	deint_hw_dev.clk_ahb_gate_vsp_eb = NULL;
	deint_hw_dev.clk_vsp_ckg = NULL;
	deint_hw_dev.deint_fp = NULL;
	deint_hw_dev.light_sleep_en = false;
	deint_hw_dev.vsp_parent_clk =
	    clock_name_map[max_freq_level - 1].clk_parent;

	sema_init(&deint_hw_dev.deint_mutex, 1);
	wake_lock_init(&deint_wakelock, WAKE_LOCK_SUSPEND,
		       "pm_message_wakelock_vpp_deint");

	ret = misc_register(&deint_dev);
	if (ret) {
		pr_err("cannot register miscdev on minor=%d (%d)\n", VPP_MINOR,
		       ret);
		goto errout;
	}

	/* register isr */
	ret = devm_request_irq(&pdev->dev, deint_hw_dev.irq, deint_isr,
			       0, "deinterlace", &deint_hw_dev);
	if (ret) {
		pr_err("vpp: failed to request deint irq!\n");
		ret = -EINVAL;
		goto errout;
	}

	return 0;

errout:
	misc_deregister(&deint_dev);

	return ret;
}

static int deint_remove(struct platform_device *pdev)
{

	misc_deregister(&deint_dev);

	free_irq(deint_hw_dev.irq, &deint_hw_dev);

	if (deint_hw_dev.vsp_parent_clk)
		clk_put(deint_hw_dev.vsp_parent_clk);

	if (deint_hw_dev.vsp_clk)
		clk_put(deint_hw_dev.vsp_clk);

	if (deint_hw_dev.clk_ahb_vsp)
		clk_put(deint_hw_dev.clk_ahb_vsp);

	if (deint_hw_dev.clk_ahb_gate_vsp_eb)
		clk_put(deint_hw_dev.clk_ahb_gate_vsp_eb);

	if (deint_hw_dev.clk_axi_gate_vsp)
		clk_put(deint_hw_dev.clk_axi_gate_vsp);

	if (deint_hw_dev.clk_vsp_ckg)
		clk_put(deint_hw_dev.clk_vsp_ckg);

	if (deint_hw_dev.clk_mm_eb)
		clk_put(deint_hw_dev.clk_mm_eb);

	pr_info("deint_remove Success !\n");

	return 0;
}

static struct platform_driver deint_driver = {
	.probe = deint_probe,
	.remove = deint_remove,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "sprd_vpp",
		   .of_match_table = of_match_ptr(of_match_table_vpp),
		   },
};

module_platform_driver(deint_driver);

MODULE_DESCRIPTION("SPRD VPP Driver");
MODULE_LICENSE("GPL");

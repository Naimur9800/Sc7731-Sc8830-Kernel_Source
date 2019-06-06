/*
 * Copyright (C) 2012--2015 Spreadtrum Communications Inc.
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
#include <uapi/video/sprd_jpg.h>
#include "sprd_jpg_common.h"

static struct jpg_dev_t jpg_hw_dev;

static struct clock_name_map_t clock_name_map[SPRD_JPG_CLK_LEVEL_NUM];

static irqreturn_t jpg_isr(int irq, void *data);

static irqreturn_t jpg_isr(int irq, void *data)
{
	int int_status;

	int_status =
		readl_relaxed((void __iomem *)(jpg_hw_dev.sprd_jpg_virt +
			GLB_INT_RAW_OFFSET));
	if ((int_status) & 0xb) {
		if ((int_status >> 3) & 0x1) {
			/* JPEG ENC  MBIO DONE */
			writel_relaxed((1 << 3),
				(void __iomem *)(jpg_hw_dev.sprd_jpg_virt +
						GLB_INT_CLR_OFFSET));

			jpg_hw_dev.jpg_int_status |= 0x8;
			jpg_hw_dev.condition_work_MBIO = 1;
			wake_up_interruptible(&jpg_hw_dev.wait_queue_work_MBIO);
			pr_debug("jpg1_isr MBIO\n");
		}
		if ((int_status >> 0) & 0x1) {
			/* JPEG ENC BSM INIT */
			writel_relaxed((1 << 0),
				(void __iomem *)(jpg_hw_dev.sprd_jpg_virt +
						GLB_INT_CLR_OFFSET));
			jpg_hw_dev.jpg_int_status |= 0x1;

			jpg_hw_dev.condition_work_BSM = 1;
			wake_up_interruptible(&jpg_hw_dev.wait_queue_work_BSM);
			pr_err("jpg1_isr BSM\n");
		}
		if ((int_status >> 1) & 0x1) {
			/* JPEG ENC VLC DONE INIT */
			writel_relaxed((1 << 1),
				(void __iomem *)(jpg_hw_dev.sprd_jpg_virt +
						GLB_INT_CLR_OFFSET));
			jpg_hw_dev.jpg_int_status |= 0x2;

			jpg_hw_dev.condition_work_VLC = 1;
			wake_up_interruptible(&jpg_hw_dev.wait_queue_work_VLC);
			pr_debug("jpg1_isr VLC\n");
		}

	}

	return IRQ_HANDLED;
}

static const struct sprd_jpg_cfg_data whale2_jpg1_data = {
	.version = WHALE2,
	.max_freq_level = 4,
	.softreset_reg_offset = 0x4,
	.reset_mask = BIT(7),
};

static const struct of_device_id of_match_table_jpg1[] = {
	{.compatible = "sprd,whale2-jpg1", .data = &whale2_jpg1_data},
	{},
};

static int jpg_parse_dt(struct platform_device *pdev)
{
	struct device *dev = &(pdev->dev);
	struct device_node *np = dev->of_node;
	struct device_node *jpg_clk_np = NULL;
	struct resource *res;
	int i, clk_count = 0;
	const char *clk_name;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no reg of property specified\n");
		pr_err("jpg: failed to parse_dt!\n");
		return -EINVAL;
	}

	jpg_hw_dev.sprd_jpg_phys = res->start;
	jpg_hw_dev.sprd_jpg_virt =
		(unsigned long)devm_ioremap_resource(&pdev->dev, res);
	if (!jpg_hw_dev.sprd_jpg_virt)
		panic("ioremap failed!\n");

	jpg_hw_dev.irq = platform_get_irq(pdev, 0);
	jpg_hw_dev.dev_np = np;
	jpg_hw_dev.jpg_dev = dev;

	pr_info("sprd_jpg_phys jpg:  0X%lx   0x%lx\n", jpg_hw_dev.sprd_jpg_phys,
		jpg_hw_dev.sprd_jpg_virt);
	pr_info(" jpg_hw_dev.irq  0X%x\n", jpg_hw_dev.irq);

	jpg_hw_dev.gpr_jpg_ahb =
		syscon_regmap_lookup_by_phandle(np, "sprd,syscon-mm-ahb");
	if (IS_ERR(jpg_hw_dev.gpr_jpg_ahb))
		pr_err("%s:failed to find jpg,mm-ahb\n", __func__);

	for_each_compatible_node(jpg_clk_np, NULL, "sprd,muxed-clock") {
		if (of_property_read_string_index
		    (jpg_clk_np, "clock-output-names", 0, &clk_name) < 0)
			continue;
		if (!strcmp(clk_name, "clk_jpg1")) {
			pr_info("clk %s\n", clk_name);
			pr_info("get device node jpg_clk_np %p\n", jpg_clk_np);
			break;
		}
	}

	clk_count = of_clk_get_parent_count(jpg_clk_np);

	for (i = 0; i < clk_count; i++) {
		struct clk *clk_parent;
		char *name_parent;
		unsigned long frequency;

		name_parent = (char *)of_clk_get_parent_name(jpg_clk_np, i);
		clk_parent = of_clk_get(jpg_clk_np, i);
		frequency = clk_get_rate(clk_parent);
		pr_info("jpg clk order in dts file: clk[%d] = (%ld, %s)\n", i,
			frequency, name_parent);

		clock_name_map[i].name = name_parent;
		clock_name_map[i].freq = frequency;
		clock_name_map[i].clk_parent = clk_parent;
	}
	jpg_hw_dev.clock_name_map = clock_name_map;

	return 0;
}

static int jpg_nocache_mmap(struct file *filp, struct vm_area_struct *vma)
{
	pr_info("@jpg[%s]\n", __func__);
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_pgoff = (jpg_hw_dev.sprd_jpg_phys >> PAGE_SHIFT);
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		pr_err("jpg_nocache_mmap failed\n");
		return -EAGAIN;
	}
	pr_info("@jpg mmap %x,%x,%x,%lx\n", (unsigned int)PAGE_SHIFT,
		(unsigned int)vma->vm_start,
		(unsigned int)(vma->vm_end - vma->vm_start),
		jpg_hw_dev.sprd_jpg_phys);
	return 0;
}

static int jpg_open(struct inode *inode, struct file *filp)
{
	int ret;
	struct jpg_fh *jpg_fp = kmalloc(sizeof(struct jpg_fh), GFP_KERNEL);

	if (jpg_fp == NULL) {
		pr_err("jpg open error occurred\n");
		return -EINVAL;
	}
	filp->private_data = jpg_fp;
	jpg_fp->is_clock_enabled = 0;
	jpg_fp->is_jpg_acquired = 0;
	jpg_fp->jpg_hw_dev = &jpg_hw_dev;

	jpg_hw_dev.condition_work_MBIO = 0;
	jpg_hw_dev.condition_work_VLC = 0;
	jpg_hw_dev.condition_work_BSM = 0;
	jpg_hw_dev.jpg_int_status = 0;

	ret = jpg_get_mm_clk(&jpg_hw_dev);
	pr_info("jpg_open: ret %d jpg_fp %p\n", ret, jpg_fp);

	return ret;
}

static int jpg_release(struct inode *inode, struct file *filp)
{
	struct jpg_fh *jpg_fp = filp->private_data;

	if (!jpg_fp)
		pr_err("%s,%d jpg_fp NULL !\n", __func__, __LINE__);

	if (jpg_fp->is_clock_enabled) {
		pr_err("error occurred and close clock\n");
		clk_disable_unprepare(jpg_hw_dev.jpg_clk);
		clk_disable_unprepare(jpg_hw_dev.clk_ahb_gate_jpg_eb);
		clk_disable_unprepare(jpg_hw_dev.clk_axi_gate_jpg);
		clk_disable_unprepare(jpg_hw_dev.clk_mm_eb);
		jpg_fp->is_clock_enabled = 0;
	}

	if (jpg_fp->is_jpg_acquired) {
		pr_err("error occurred and up jpg_mutex\n");
		up(&jpg_hw_dev.jpg_mutex);
	}

	kfree(filp->private_data);
	pr_info("jpg_release %p\n", jpg_fp);

	return 0;
}

static const struct file_operations jpg_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = jpg_ioctl,
	.mmap = jpg_nocache_mmap,
	.open = jpg_open,
	.release = jpg_release,
	.unlocked_ioctl = jpg_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = compat_jpg_ioctl,
#endif
};

static struct miscdevice jpg_dev = {
	.minor = JPG_MINOR,
	.name = "sprd_jpg1",
	.fops = &jpg_fops,
};

static int jpg_probe(struct platform_device *pdev)
{
	int ret;
	const struct of_device_id *of_id;
	struct device_node *node = pdev->dev.of_node;

	pr_info("jpg_probe called !\n");

	of_id = of_match_node(of_match_table_jpg1, node);
	if (of_id)
		jpg_hw_dev.jpg_cfg_data =
		    (struct sprd_jpg_cfg_data *)of_id->data;
	else
		panic("%s: Not find matched id!", __func__);

	if (jpg_parse_dt(pdev)) {
		pr_err("jpg_parse_dt failed\n");
		return -EINVAL;
	}

	jpg_hw_dev.version = jpg_hw_dev.jpg_cfg_data->version;
	jpg_hw_dev.max_freq_level = jpg_hw_dev.jpg_cfg_data->max_freq_level;
	jpg_hw_dev.jpg_softreset_reg_offset =
	    jpg_hw_dev.jpg_cfg_data->softreset_reg_offset;
	jpg_hw_dev.jpg_reset_mask = jpg_hw_dev.jpg_cfg_data->reset_mask;

	sema_init(&jpg_hw_dev.jpg_mutex, 1);

	init_waitqueue_head(&jpg_hw_dev.wait_queue_work_MBIO);
	jpg_hw_dev.condition_work_MBIO = 0;
	init_waitqueue_head(&jpg_hw_dev.wait_queue_work_VLC);
	jpg_hw_dev.condition_work_VLC = 0;
	init_waitqueue_head(&jpg_hw_dev.wait_queue_work_BSM);
	jpg_hw_dev.condition_work_BSM = 0;
	jpg_hw_dev.jpg_int_status = 0;

	jpg_hw_dev.freq_div = DEFAULT_FREQ_DIV;

	jpg_hw_dev.jpg_clk = NULL;
	jpg_hw_dev.jpg_parent_clk = NULL;
	jpg_hw_dev.clk_mm_eb = NULL;
	jpg_hw_dev.clk_axi_gate_jpg = NULL;
	jpg_hw_dev.clk_ahb_gate_jpg_eb = NULL;
	jpg_hw_dev.jpg_fp = NULL;

	ret = misc_register(&jpg_dev);
	if (ret) {
		pr_err("cannot register miscdev on minor=%d (%d)\n",
		       JPG_MINOR, ret);
		goto errout;
	}

	/* register isr */
	ret =
	    devm_request_irq(&pdev->dev, jpg_hw_dev.irq, jpg_isr, 0, "JPG",
			     &jpg_hw_dev);
	if (ret) {
		pr_err("jpg: failed to request irq!\n");
		ret = -EINVAL;
		goto errout2;
	}

	return 0;

errout2:
	misc_deregister(&jpg_dev);

errout:
	return ret;
}

static int jpg_remove(struct platform_device *pdev)
{
	pr_info("jpg_remove called !\n");

	misc_deregister(&jpg_dev);

	free_irq(jpg_hw_dev.irq, &jpg_hw_dev);

	if (jpg_hw_dev.jpg_parent_clk)
		clk_put(jpg_hw_dev.jpg_parent_clk);

	if (jpg_hw_dev.jpg_clk)
		clk_put(jpg_hw_dev.jpg_clk);

	if (jpg_hw_dev.clk_ahb_gate_jpg_eb)
		clk_put(jpg_hw_dev.clk_ahb_gate_jpg_eb);

	if (jpg_hw_dev.clk_axi_gate_jpg)
		clk_put(jpg_hw_dev.clk_axi_gate_jpg);

	if (jpg_hw_dev.clk_mm_eb)
		clk_put(jpg_hw_dev.clk_mm_eb);

	pr_info("jpg_remove Success !\n");
	return 0;
}

static struct platform_driver jpg_driver = {
	.probe = jpg_probe,
	.remove = jpg_remove,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "sprd_jpg1",
		   .of_match_table = of_match_ptr(of_match_table_jpg1),
		   },
};

module_platform_driver(jpg_driver);

MODULE_DESCRIPTION("SPRD JPG1 Driver");
MODULE_LICENSE("GPL");

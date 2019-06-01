/*
 * drivers/gpu/iommu/iommu.c *
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include"../sprd_iommu_common.h"

int sprd_iommu_gsp_enable(struct sprd_iommu_dev *dev);
int sprd_iommu_gsp_disable(struct sprd_iommu_dev *dev);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sprd_iommu_gsp_early_suspend(struct early_suspend* es);
static void sprd_iommu_gsp_late_resume(struct early_suspend* es);
#endif

void sprd_iommu_gsp_open(struct sprd_iommu_dev *dev)
{
	pr_debug("%s\n",__FUNCTION__);
	mutex_lock(&dev->mutex_pgt);
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_EN(0),MMU_EN_MASK);
#ifdef CONFIG_ARCH_SCX35L
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_RAMCLK_DIV2_EN(1),MMU_RAMCLK_DIV2_EN_MASK);
#endif
	mmu_reg_write(dev->init_data->ctrl_reg,dev->init_data->iova_base,MMU_START_MB_ADDR_MASK);
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_TLB_EN(1),MMU_TLB_EN_MASK);
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_EN(1),MMU_EN_MASK);
	mutex_unlock(&dev->mutex_pgt);
	return;
}

void sprd_iommu_gsp_release(struct sprd_iommu_dev *dev)
{
	pr_debug("%s\n",__FUNCTION__);
	mutex_lock(&dev->mutex_pgt);
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_RAMCLK_DIV2_EN(0),MMU_RAMCLK_DIV2_EN_MASK);
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_TLB_EN(0),MMU_TLB_EN_MASK);
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_EN(0),MMU_EN_MASK);
	mutex_unlock(&dev->mutex_pgt);
	return;
}

int sprd_iommu_gsp_init(struct sprd_iommu_dev *dev, struct sprd_iommu_init_data *data)
{
	int err = -1;
#ifndef CONFIG_SCX35L64BIT_FPGA
	struct device_node *np;
	np = dev->misc_dev.this_device->of_node;
	if(!np) {
		return -1;
	}

	dev->mmu_mclock=of_clk_get(np, 0) ;
	dev->mmu_clock=of_clk_get(np, 2) ;

	if (IS_ERR(dev->mmu_clock) || IS_ERR(dev->mmu_mclock)) {
		printk ("%s, can't get clock:%p, %p\n", __FUNCTION__, dev->mmu_clock, dev->mmu_mclock);
		goto errorout;
	}
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	dev->early_suspend.suspend = sprd_iommu_gsp_early_suspend;
	dev->early_suspend.resume  = sprd_iommu_gsp_late_resume;
	dev->early_suspend.level   = EARLY_SUSPEND_LEVEL_STOP_DRAWING;
	register_early_suspend(&dev->early_suspend);
#endif

	sprd_iommu_gsp_enable(dev);
	err = sprd_iommu_init(dev,data);
	sprd_iommu_gsp_disable(dev);
	return err;

errorout:
    if (dev->mmu_clock) {
        clk_put(dev->mmu_clock);
    }

    if (dev->mmu_mclock) {
        clk_put(dev->mmu_mclock);
    }

    return -1;
}

int sprd_iommu_gsp_exit(struct sprd_iommu_dev *dev)
{
	int err = -1;
	sprd_iommu_gsp_enable(dev);
	err  =sprd_iommu_exit(dev);
	sprd_iommu_gsp_disable(dev);
	return err;
}

unsigned long sprd_iommu_gsp_iova_alloc(struct sprd_iommu_dev *dev, size_t iova_length)
{
	return sprd_iommu_iova_alloc(dev,iova_length);
}

void sprd_iommu_gsp_iova_free(struct sprd_iommu_dev *dev, unsigned long iova, size_t iova_length)
{
	sprd_iommu_iova_free(dev,iova,iova_length);
	return;
}

int sprd_iommu_gsp_iova_map(struct sprd_iommu_dev *dev, unsigned long iova, size_t iova_length,
				struct ion_buffer *handle)
{
	int err = -1;

	mutex_lock(&dev->mutex_map);
	if (0 == dev->map_count)
		sprd_iommu_gsp_open(dev);
	dev->map_count++;

	err = sprd_iommu_iova_map(dev,iova,iova_length,handle);
	mutex_unlock(&dev->mutex_map);

	return err;
}

int sprd_iommu_gsp_iova_unmap(struct sprd_iommu_dev *dev, unsigned long iova, size_t iova_length,
				struct ion_buffer *handle)
{
	int err = -1;

	mutex_lock(&dev->mutex_map);
	err = sprd_iommu_iova_unmap(dev,iova,iova_length,handle);

	dev->map_count--;
	if (0 == dev->map_count)
		sprd_iommu_gsp_release(dev);
	mutex_unlock(&dev->mutex_map);

	return err;
}

int sprd_iommu_gsp_backup(struct sprd_iommu_dev *dev)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	return 0;
#else
	int err = -1;

	mutex_lock(&dev->mutex_map);
	printk("%s, map_count:%d\n", __FUNCTION__, dev->map_count);
	if (dev->map_count > 0)
		err = sprd_iommu_backup(dev);
	mutex_unlock(&dev->mutex_map);

	return err;
#endif
}

int sprd_iommu_gsp_restore(struct sprd_iommu_dev *dev)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	return 0;
#else
	int err = -1;

	mutex_lock(&dev->mutex_map);
	printk("%s, map_count:%d\n", __FUNCTION__, dev->map_count);
	if (dev->map_count > 0)
		err = sprd_iommu_restore(dev);
	mutex_unlock(&dev->mutex_map);
#endif

}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sprd_iommu_gsp_early_suspend(struct early_suspend* es)
{
	int err = -1;
	struct sprd_iommu_dev *dev = container_of(es, struct sprd_iommu_dev, early_suspend);

	mutex_lock(&dev->mutex_map);
	printk("%s, map_count:%d\n", __FUNCTION__, dev->map_count);
	if (dev->map_count > 0)
		err = sprd_iommu_backup(dev);
	mutex_unlock(&dev->mutex_map);
}

static void sprd_iommu_gsp_late_resume(struct early_suspend* es)
{
	int err = -1;
	struct sprd_iommu_dev *dev = container_of(es, struct sprd_iommu_dev, early_suspend);

	mutex_lock(&dev->mutex_map);
	printk("%s, map_count:%d\n", __FUNCTION__, dev->map_count);
	if (dev->map_count > 0)
		err = sprd_iommu_restore(dev);
	mutex_unlock(&dev->mutex_map);
}
#endif

int sprd_iommu_gsp_disable(struct sprd_iommu_dev *dev)
{
	pr_debug("%s\n",__FUNCTION__);
	sprd_iommu_disable(dev);
#ifndef CONFIG_SCX35L64BIT_FPGA
	clk_disable_unprepare(dev->mmu_clock);
	clk_disable_unprepare(dev->mmu_mclock);
#endif
	return 0;
}

int sprd_iommu_gsp_enable(struct sprd_iommu_dev *dev)
{
	pr_debug("%s\n",__FUNCTION__);
#ifndef CONFIG_SCX35L64BIT_FPGA
	clk_prepare_enable(dev->mmu_mclock);
	clk_prepare_enable(dev->mmu_clock);
#endif
	sprd_iommu_enable(dev);
	return 0;
}

int sprd_iommu_gsp_dump(struct sprd_iommu_dev *dev, unsigned long iova, size_t iova_length)
{
	return sprd_iommu_dump(dev,iova,iova_length);
}

struct sprd_iommu_ops iommu_gsp_ops={
	.init=sprd_iommu_gsp_init,
	.exit=sprd_iommu_gsp_exit,
	.iova_alloc=sprd_iommu_gsp_iova_alloc,
	.iova_free=sprd_iommu_gsp_iova_free,
	.iova_map=sprd_iommu_gsp_iova_map,
	.iova_unmap=sprd_iommu_gsp_iova_unmap,
	.backup=sprd_iommu_gsp_backup,
	.restore=sprd_iommu_gsp_restore,
	.disable=sprd_iommu_gsp_disable,
	.enable=sprd_iommu_gsp_enable,
	.dump=sprd_iommu_gsp_dump,
	.open=sprd_iommu_gsp_open,
	.release=sprd_iommu_gsp_release,
};


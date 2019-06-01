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

#include <linux/module.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/ion.h>
#include <linux/clk.h>
#include <linux/miscdevice.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include <linux/genalloc.h>
#include <linux/sprd_iommu.h>
#include <linux/scatterlist.h>
#include <asm/io.h>
#if 0
#ifdef CONFIG_ARCH_SCX35L64
#include <soc/sprd/sci_glb_regs.h>
#else
#include <soc/sprd/hardware.h>
#include <soc/sprd/sci.h>
#include <soc/sprd/sci_glb_regs.h>
#endif
#endif
#include "../ion/ion_priv.h"
#include "sprd_iommu_sysfs.h"

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>


struct sprd_iommu_dev *iommu_devs[IOMMU_MAX]={NULL,NULL};
extern struct sprd_iommu_ops iommu_gsp_ops;
extern struct sprd_iommu_ops iommu_mm_ops;

static int sprd_iommu_probe(struct platform_device *pdev);
static int sprd_iommu_remove(struct platform_device *pdev);
static int sprd_iommu_suspend(struct platform_device *pdev, pm_message_t state);
static int sprd_iommu_resume(struct platform_device *pdev);

static struct sprd_iommu_init_data sprd_iommu_gsp_data = {
	.id=0,
	.name="sprd_iommu_gsp",
};

static struct sprd_iommu_init_data sprd_iommu_mm_data = {
	.id=1,
	.name="sprd_iommu_mm",
};

const struct of_device_id iommu_ids[] __initconst = {
	{ .compatible = "sprd,sprd_iommu", },
	{},
};

static struct platform_driver iommu_driver = {
	.probe = sprd_iommu_probe,
	.remove = sprd_iommu_remove,
#ifndef CONFIG_ARCH_SCX35L
	.suspend = sprd_iommu_suspend,
	.resume = sprd_iommu_resume,
#endif
	.driver = {
		.owner=THIS_MODULE,
		.name="sprd_iommu",
		.of_match_table = iommu_ids,
	},
};

unsigned long sprd_iova_alloc(int iommu_id, unsigned long iova_length)
{
	return iommu_devs[iommu_id]->ops->iova_alloc(iommu_devs[iommu_id],iova_length);
}

void sprd_iova_free(int iommu_id, unsigned long iova, unsigned long iova_length)
{
	iommu_devs[iommu_id]->ops->iova_free(iommu_devs[iommu_id],iova,iova_length);
}

int sprd_iova_map(int iommu_id, unsigned long iova, struct ion_buffer *handle)
{
	return iommu_devs[iommu_id]->ops->iova_map(iommu_devs[iommu_id],iova, handle->size, handle);
}
int sprd_iova_unmap(int iommu_id, unsigned long iova,  struct ion_buffer *handle)
{
	return iommu_devs[iommu_id]->ops->iova_unmap(iommu_devs[iommu_id],iova,handle->size,handle);
}

void sprd_iommu_module_enable(uint32_t iommu_id)
{
	if (IOMMU_GSP == iommu_id || iommu_devs[iommu_id]->light_sleep_en)
		iommu_devs[iommu_id]->ops->enable(iommu_devs[iommu_id]);
}

void sprd_iommu_module_disable(uint32_t iommu_id)
{
        if (IOMMU_GSP == iommu_id || iommu_devs[iommu_id]->light_sleep_en)
		iommu_devs[iommu_id]->ops->disable(iommu_devs[iommu_id]);
}

static int sprd_iommu_suspend(struct platform_device *pdev, pm_message_t state)
{
	int err=-1;
	struct sprd_iommu_dev *iommu_dev=platform_get_drvdata(pdev); 
	printk("sprd_iommu id:%d, name:%s mmu_clock:0x%p  mmu_mclock:0x%p suspend\n",
		iommu_dev->init_data->id,iommu_dev->init_data->name,iommu_dev->mmu_clock,iommu_dev->mmu_mclock);
	err=iommu_dev->ops->backup(iommu_dev);
	return err;
}

static int sprd_iommu_resume(struct platform_device *pdev)
{
	int err=-1;
	struct sprd_iommu_dev *iommu_dev=platform_get_drvdata(pdev); 
	printk("sprd_iommu id:%d, name:%s mmu_clock:0x%p  mmu_mclock:0x%p resume\n",
		iommu_dev->init_data->id,iommu_dev->init_data->name,iommu_dev->mmu_clock,iommu_dev->mmu_mclock);
	err=iommu_dev->ops->restore(iommu_dev);
	return err;
}

static inline int sprd_iommu_get_description(struct device_node *np,char* desc_name, char**desc)
{
	return  of_property_read_string(np, (const char*)desc_name,(const char**) desc);
}

static int sprd_iommu_get_resource(struct device_node *np, struct sprd_iommu_init_data *pdata)
{
	int err=0;
	uint32_t val;
	struct resource res;

	err = of_address_to_resource(np, 0, &res);
	if(err < 0) return err;
	printk("%s,name:%s\n",__FUNCTION__,pdata->name);
	pdata->iova_base = res.start;
	pdata->iova_size = resource_size(&res);
	printk("%s,iova_base:0x%lx,iova_size:%zx\n",__FUNCTION__,pdata->iova_base,pdata->iova_size);

	err = of_address_to_resource(np, 1, &res);
	if (err < 0) return err;
	printk("%s,pgt_base phy:0x%lx\n", __FUNCTION__, res.start);
	pdata->pgt_base = (unsigned long)ioremap_nocache(res.start,
		resource_size(&res));
	if (!pdata->pgt_base)
		BUG();
	pdata->pgt_size = resource_size(&res);
	printk("%s,pgt_base:%lx,pgt_size:%zx\n", __FUNCTION__, pdata->pgt_base,
		pdata->pgt_size);

	err = of_address_to_resource(np, 2, &res);
	if(err < 0) return err;
	printk("%s,ctrl_reg phy:0x%lx\n",__FUNCTION__,res.start);
	pdata->ctrl_reg = (unsigned long)ioremap_nocache(res.start,
		resource_size(&res));
	if (!pdata->ctrl_reg)
		BUG();
	printk("%s,ctrl_reg:0x%lx\n",__FUNCTION__,pdata->ctrl_reg);

	err = of_property_read_u32(np, "sprd,reserved-fault-page", &val);
	if (err) {
		if (0 == strncmp("sprd_iommu_mm", pdata->name, 13)) {
			unsigned long page =  0;
			uint32_t i;

			page = __get_free_page(GFP_KERNEL);
			if (page)
				pdata->fault_page = virt_to_phys((void *)page);
			else
				pdata->fault_page = 0;
		} else {
			pdata->fault_page = 0;
		}
		pr_info("%s, fault_page: 0x%lx\n", __func__, pdata->fault_page);
	} else {
		pr_info("%s, reserved fault page phy:0x%x\n", __func__, val);
		pdata->fault_page = val;
	}

	return 0;
}

static int sprd_iommu_probe(struct platform_device *pdev)
{
	char func_name[] = "func-name";
	char func_desc[128];
	char* pdesc=&(func_desc[0]);
	struct device_node *np=pdev->dev.of_node;
	int err=-1;
	struct sprd_iommu_init_data *pdata;
	struct sprd_iommu_dev *iommu_dev;

	printk("%s,begin\n",__FUNCTION__);
	iommu_dev = kzalloc(sizeof(struct sprd_iommu_dev), GFP_KERNEL);
	if (NULL==iommu_dev) {
		printk("%s,fail to kzalloc\n",__FUNCTION__);
		return -1;
	}

	if (-1==sprd_iommu_get_description(np, func_name,&pdesc)) {
		kfree(iommu_dev);
		printk("%s,fail to get description\n",__FUNCTION__);
		return -1;
	}
	printk("%s, function name is %s\n",__FUNCTION__,pdesc);
	if(0==strncmp("sprd_iommu_gsp",pdesc,14))
	{
		pdata = &sprd_iommu_gsp_data;
		iommu_dev->ops=&iommu_gsp_ops;
		err = sprd_iommu_get_resource(np, pdata);
		if(err < 0) {
		    dev_err(&pdev->dev, "no reg of property specified\n");
		    printk(KERN_ERR "iommu: failed to get reg!\n");
		    kfree(iommu_dev);
		    return -EINVAL;
		}
	}
	else if(0==strncmp("sprd_iommu_mm",pdesc,13))
	{
		pdata = &sprd_iommu_mm_data;
		iommu_dev->ops=&iommu_mm_ops;
		err = sprd_iommu_get_resource(np, pdata);
		if(err < 0) {
		    dev_err(&pdev->dev, "no reg of property specified\n");
		    printk(KERN_ERR "iommu: failed to get reg!\n");
		    kfree(iommu_dev);
		    return -EINVAL;
		}
	}
	else {
		kfree(iommu_dev);
		printk("%s, function name mismatch\n",__FUNCTION__);
		return -1;
	}
	iommu_devs[pdata->id]=iommu_dev;
	iommu_dev->init_data = pdata;
	mutex_init(&iommu_dev->mutex_pgt);
	mutex_init(&iommu_dev->mutex_map);
        /*If ddr frequency >= 500Mz, iommu need div2 frequency,
         * because frq of iommu is lower than ddr frq.*/
        iommu_dev->div2_frq = 500;
        iommu_dev->light_sleep_en = false;
	iommu_dev->misc_dev.minor = MISC_DYNAMIC_MINOR;
	iommu_dev->misc_dev.name = pdata->name;
	iommu_dev->misc_dev.fops = NULL;
	iommu_dev->misc_dev.parent = NULL;
	err = misc_register(&iommu_dev->misc_dev);
	if (err) {
		pr_err("iommu %s : failed to register misc device.\n",pdata->name);
		return err;
	}
	iommu_dev->misc_dev.this_device->of_node = pdev->dev.of_node;
	err=iommu_dev->ops->init(iommu_dev,pdata);
	if(err<0)
	{
		kfree(iommu_dev);
		pr_err("iommu %s : failed to init device %d.\n",pdata->name, err);
		return err;
	}

	sprd_iommu_sysfs_register(iommu_dev,iommu_dev->init_data->name);
	platform_set_drvdata(pdev, iommu_dev);
	printk("sprd_iommu id:%d, name:%s, dev->pgt:0x%lx, dev->pool:0x%p mmu_clock:0x%p\n",
		pdata->id,pdata->name,iommu_dev->pgt,iommu_dev->pool,iommu_dev->mmu_clock);
	printk("%s,end\n",__FUNCTION__);
	return err;
}

static int sprd_iommu_remove(struct platform_device *pdev)
{
	struct sprd_iommu_dev *iommu_dev=platform_get_drvdata(pdev); 
	misc_deregister(&iommu_dev->misc_dev);
	sprd_iommu_sysfs_unregister(iommu_dev,iommu_dev->init_data->name);
	iommu_dev->ops->exit(iommu_dev);
	gen_pool_destroy(iommu_dev->pool);
	kfree(iommu_dev);
	return 0;
}

static int __init iommu_init(void)
{
	int err=-1;
	printk("%s,begin\n",__FUNCTION__);
	err=platform_driver_register(&iommu_driver);
	if(err<0)
	{
		pr_err("sprd_iommu register err: %d\n",err);
	}
	printk("%s,end\n",__FUNCTION__);
	return err;
}

static void __exit iommu_exit(void)
{
	platform_driver_unregister(&iommu_driver);
}

module_init(iommu_init);
module_exit(iommu_exit);

MODULE_DESCRIPTION("SPRD IOMMU Driver");
MODULE_LICENSE("GPL");

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

#include <linux/module.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include <linux/genalloc.h>
#include <linux/sprd_iommu.h>
#include <linux/scatterlist.h>
#include <linux/of.h>
#include <linux/device.h>
#include "api/sprd_iommu_api.h"

static void sprd_iommu_clk_enable(struct sprd_iommu_dev *dev)
{
	if (dev->mmu_mclock)
		clk_prepare_enable(dev->mmu_mclock);

	if (dev->mmu_clock)
		clk_prepare_enable(dev->mmu_clock);

	if (dev->mmu_axiclock)
		clk_prepare_enable(dev->mmu_axiclock);
}

static void sprd_iommu_clk_disable(struct sprd_iommu_dev *dev)
{
	if (dev->mmu_clock)
		clk_disable_unprepare(dev->mmu_clock);

	if (dev->mmu_mclock)
		clk_disable_unprepare(dev->mmu_mclock);

	if (dev->mmu_axiclock)
		clk_disable_unprepare(dev->mmu_axiclock);
}

static int get_iommu_type(int revision)
{
	enum sprd_iommu_type type = SPRD_IOMMU_NOT_SUPPORT;

	switch (revision) {
	case 1:
	{
		type = SPRD_IOMMU_R1P0;
		break;
	}
	case 2:
	{
		type = SPRD_IOMMU_R2P0;
		break;
	}
	case 5:
	{
		type = SPRD_IOMMU_R4P0;
		break;
	}
	default:
	{
		type  = SPRD_IOMMU_NOT_SUPPORT;
		break;
	}
	}
	return type;
}

static int sprd_iommu_hw_init(struct sprd_iommu_dev *dev,
			struct sprd_iommu_init_data *data)
{
	void *p_iommu_hdl = NULL;
	struct device *dev_priv = (struct device *)(dev->drv_dev);
	struct sprd_iommu_init_param iommu_init_param;
	struct device_node *np = NULL;

	IOMMU_INFO("begin\n");

	np = dev_priv->of_node;
	if (!np)
		return -1;

	if (data->iommu_rev == 1) {
		if (IOMMU_GSP == data->id) {
			dev->mmu_mclock = of_clk_get(np, 0);
			dev->mmu_clock = of_clk_get(np, 2);

			if (IS_ERR(dev->mmu_clock) || IS_ERR(dev->mmu_mclock)) {
				IOMMU_ERR("can't get clock:%p, %p\n",
					dev->mmu_clock, dev->mmu_mclock);
				goto errorout;
			}
		} else if (IOMMU_MM == data->id) {
			dev->mmu_clock = of_clk_get(np, 0);
			dev->mmu_mclock = of_clk_get(np, 1);
			dev->mmu_axiclock = of_clk_get(np, 2);
			if (IS_ERR(dev->mmu_axiclock))
				IOMMU_ERR("can't get mm axi clock:%p\n",
					dev->mmu_axiclock);
			else
				dev->light_sleep_en = true;

			if (IS_ERR(dev->mmu_clock) || IS_ERR(dev->mmu_mclock)) {
				IOMMU_ERR("can't get clock:%p, %p\n",
					dev->mmu_clock, dev->mmu_mclock);
				goto errorout;
			}
		} else {
				IOMMU_ERR("unknownd iommu id\n");
				goto errorout;
			}
	} else if (data->iommu_rev == 2) {
		dev->mmu_clock = of_clk_get(np, 0);
		dev->mmu_mclock = of_clk_get(np, 1);

		if (IS_ERR(dev->mmu_clock)
			|| IS_ERR(dev->mmu_mclock)) {
			IOMMU_ERR("can't get clock:%p, %p\n",
				dev->mmu_clock, dev->mmu_mclock);
			goto errorout;
		}
	} else if (data->iommu_rev == 5) {
		dev->mmu_clock = of_clk_get(np, 0);
		dev->mmu_mclock = of_clk_get(np, 1);
		if (IS_ERR(dev->mmu_clock)
			|| IS_ERR(dev->mmu_mclock)) {
			IOMMU_ERR("can't get clock:%p, %p\n",
				dev->mmu_clock, dev->mmu_mclock);
			goto errorout;
		}
	} else {
			IOMMU_ERR("unknownd iommu rev %d\n", data->iommu_rev);
			goto errorout;
	}

	memset(&iommu_init_param, 0 , sizeof(struct sprd_iommu_init_param));

	dev->pool = gen_pool_create(12, -1);
	if (!dev->pool) {
		IOMMU_ERR("%s gen_pool_create error\n", data->name);
		return -1;
	}
	gen_pool_set_algo(dev->pool, gen_pool_best_fit, NULL);
	gen_pool_add(dev->pool, data->iova_base, data->iova_size, -1);

	iommu_init_param.iommu_type = get_iommu_type(data->iommu_rev);
	iommu_init_param.base_reg_addr = data->pgt_base;
	iommu_init_param.pgt_size = data->pgt_size;
	iommu_init_param.ctrl_reg_addr = data->ctrl_reg;
	iommu_init_param.fm_base_addr = data->iova_base;
	iommu_init_param.pagt_base_ddr = data->pagt_base_ddr;
	iommu_init_param.pagt_ddr_size = data->pagt_ddr_size;

	if (SPRD_IOMMU_R1P0 == iommu_init_param.iommu_type)
		iommu_init_param.ram_clk_div = dev->div2_frq;

	if (0 != data->re_route_page) {
		iommu_init_param.reroute_enable = 1;
		iommu_init_param.reroute_addr = data->re_route_page;
	}

	if (0 != data->fault_page) {
		iommu_init_param.faultpage_enable = 1;
		iommu_init_param.faultpage_addr = data->fault_page;
	}

	sprd_iommudrv_init(&iommu_init_param, (sprd_iommu_hdl *)&p_iommu_hdl);

	dev->private = p_iommu_hdl;
	IOMMU_INFO("done\n");

	return 0;

errorout:
	if (dev->mmu_clock)
		clk_put(dev->mmu_clock);

	if (dev->mmu_mclock)
		clk_put(dev->mmu_mclock);

	if (dev->mmu_axiclock)
		clk_put(dev->mmu_axiclock);

	return -1;
}

static int sprd_iommu_hw_exit(struct sprd_iommu_dev *dev)
{
	sprd_iommudrv_uninit(dev->private);
	dev->private = NULL;
	return 0;
}

static unsigned long sprd_iommu_iova_alloc(struct sprd_iommu_dev *dev,
					size_t iova_length,
					struct sprd_iommu_map_data  *p_param)
{
	unsigned long iova = 0;

	iova = gen_pool_alloc(dev->pool, iova_length);
	return iova;
}

static void sprd_iommu_iova_free(struct sprd_iommu_dev *dev, unsigned long iova,
				size_t iova_length)
{
	gen_pool_free(dev->pool, iova, iova_length);
}

static int sprd_iommu_iova_map(struct sprd_iommu_dev *dev, unsigned long iova,
				size_t iova_length,
				struct sg_table *table,
				struct sprd_iommu_map_data  *p_param)
{
	int err = -1;
	struct sprd_iommu_map_param map_param;

	memset(&map_param, 0 , sizeof(map_param));
	map_param.channel_type = p_param->ch_type;
	map_param.channel_bypass = 0;
	map_param.start_virt_addr = iova;
	map_param.total_map_size = iova_length;

	map_param.p_sg_table = table;
	err = sprd_iommudrv_map(dev->private, &map_param);
	if (SPRD_NO_ERR == err)
		return 0;
	else
		return -1;
}

static int sprd_iommu_hw_suspend(struct sprd_iommu_dev *dev)
{
	sprd_iommu_clk_disable(dev);
	return 0;
}

static int sprd_iommu_hw_resume(struct sprd_iommu_dev *dev)
{
	/*set clk back first*/
	sprd_iommu_clk_enable(dev);
	return 0;
}

static int sprd_iommu_iova_unmap(struct sprd_iommu_dev *dev, unsigned long iova,
				size_t iova_length)
{
	int err = -1;

	struct sprd_iommu_unmap_param unmap_param;

	memset(&unmap_param, 0 , sizeof(struct sprd_iommu_unmap_param));
	unmap_param.start_virt_addr = iova;
	unmap_param.total_map_size = iova_length;

	err = sprd_iommudrv_unmap(dev->private, &unmap_param);

	if (SPRD_NO_ERR == err)
		return 0;
	else
		return -1;
}

struct sprd_iommu_ops sprd_iommu_hw_ops = {
	.init = sprd_iommu_hw_init,
	.exit = sprd_iommu_hw_exit,
	.iova_alloc = sprd_iommu_iova_alloc,
	.iova_free = sprd_iommu_iova_free,
	.iova_map = sprd_iommu_iova_map,
	.iova_unmap = sprd_iommu_iova_unmap,
	.backup = NULL,
	.restore = NULL,
	.enable = NULL,
	.disable = NULL,
	.dump = NULL,
	.open = NULL,
	.release = NULL,
	.suspend = sprd_iommu_hw_suspend,
	.resume = sprd_iommu_hw_resume,
	.iova_unmap_orphaned = NULL,
};

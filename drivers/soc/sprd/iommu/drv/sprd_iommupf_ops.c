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

static void sprd_iommupf_clk_enable(struct sprd_iommu_dev *dev)
{
	if (dev->mmu_mclock)
		clk_prepare_enable(dev->mmu_mclock);

	if (dev->mmu_clock)
		clk_prepare_enable(dev->mmu_clock);
}

static void sprd_iommupf_clk_disable(struct sprd_iommu_dev *dev)
{
	if (dev->mmu_clock)
		clk_disable_unprepare(dev->mmu_clock);

	if (dev->mmu_mclock)
		clk_disable_unprepare(dev->mmu_mclock);
}

static int get_iommupf_type(int revision)
{
	enum sprd_iommu_type type = SPRD_IOMMU_NOT_SUPPORT;

	switch (revision) {
	case 1:
	{
		type = SPRD_IOMMUPF_R1P0;
		break;
	}
	case 2:
	{
		type = SPRD_IOMMUPF_R2P0;
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

/*resource init, */
static int sprd_iommupf_hw_init(struct sprd_iommu_dev *dev,
			struct sprd_iommu_init_data *data)
{
	void *p_iommu_hdl = NULL;
	struct device *dev_priv = (struct device *)(dev->drv_dev);
	struct sprd_iommu_init_param hw_init_param;
	struct device_node *np;

	np = dev_priv->of_node;
	if (!np)
		return -1;

	dev->mmu_clock = of_clk_get(np, 0);
	dev->mmu_mclock = of_clk_get(np, 1);

	if (IS_ERR(dev->mmu_clock) ||
		IS_ERR(dev->mmu_mclock)) {
		IOMMU_ERR("can't get clock:%p\n", dev->mmu_clock);
		goto errorout;
	}

	memset(&hw_init_param, 0 , sizeof(hw_init_param));

	hw_init_param.iommu_type = get_iommupf_type(data->iommupf_rev);
	hw_init_param.iommu_id = data->id;
	hw_init_param.base_reg_addr = data->pgt_base;
	hw_init_param.ctrl_reg_addr = data->ctrl_reg;

	hw_init_param.fm_base_addr = data->iova_base;
	hw_init_param.fm_ram_size = data->iova_size;

	hw_init_param.fullpage_mode = data->fullmode_en;
	hw_init_param.total_ch_num = data->total_ch_num;
	hw_init_param.pfch_rstart = data->pf_ch_read_start;
	hw_init_param.pfch_rend = data->pf_ch_read_end;
	hw_init_param.pfch_wstart = data->pf_ch_write_start;
	hw_init_param.pfch_wend = data->pf_ch_write_end;
	hw_init_param.fullch_write = data->fullmode_ch_write;
	hw_init_param.fullch_read = data->fullmode_ch_read;

	hw_init_param.pagt_base_ddr = data->pagt_base_ddr;
	hw_init_param.pagt_ddr_size = data->pagt_ddr_size;

	if (0 != data->re_route_page) {
		hw_init_param.reroute_enable = 1;
		hw_init_param.reroute_addr = data->re_route_page;
	}

	if (0 != data->fault_page) {
		hw_init_param.faultpage_enable = 1;
		hw_init_param.faultpage_addr = data->fault_page;
	}

	if (1 == hw_init_param.fullpage_mode) {
		dev->pool = gen_pool_create(12, -1);
		if (!dev->pool) {
			IOMMU_ERR("%s fm create pool error\n", data->name);
			return -1;
		}
		gen_pool_add(dev->pool, data->iova_base, data->iova_size, -1);
	}

	if (1 == data->iommupf_rev)
		dev->pf_ch_pool = gen_pool_create(20, -1);
	else if (2 == data->iommupf_rev)
		dev->pf_ch_pool = gen_pool_create(12, -1);
	else {
		IOMMU_ERR("iommupf ver %d err\n", data->iommupf_rev);
		return -1;
		}

	if (!dev->pf_ch_pool) {
		IOMMU_ERR("%s pf gen_pool_create error\n", data->name);
		return -1;
	}
	gen_pool_add(dev->pf_ch_pool, data->pfch_iova_base,
		     data->pfch_iova_size, -1);

	sprd_iommudrv_init(&hw_init_param, (sprd_iommu_hdl *)&p_iommu_hdl);

	dev->private = p_iommu_hdl;

	return 0;

errorout:
	if (dev->mmu_clock)
		clk_put(dev->mmu_clock);
	if (dev->mmu_mclock)
		clk_put(dev->mmu_mclock);
	return -1;
}

static int sprd_iommupf_hw_exit(struct sprd_iommu_dev *dev)
{
	sprd_iommudrv_uninit(dev->private);
	dev->private = NULL;
	return 0;
}


static unsigned long sprd_iommupf_iova_alloc(struct sprd_iommu_dev *dev,
					size_t iova_length,
					struct sprd_iommu_map_data  *p_param)
{
	unsigned long iova = 0;

	if ((SPRD_IOMMU_PF_CH_READ == p_param->ch_type)
	    || (SPRD_IOMMU_PF_CH_WRITE == p_param->ch_type)) {
		iova = gen_pool_alloc(dev->pf_ch_pool, iova_length);
	} else if (SPRD_IOMMU_FM_CH_RW == p_param->ch_type) {
		/*fullmode read/write share one pool*/
		iova = gen_pool_alloc(dev->pool, iova_length);
	}

	return iova;
}

static void sprd_iommupf_iova_free(struct sprd_iommu_dev *dev,
				   unsigned long iova,
				size_t iova_length)
{
	unsigned long iova_pool_start = 0;
	unsigned long iova_pool_end = 0;
	unsigned long iova_pf_pool_start = 0;
	unsigned long iova_pf_pool_end = 0;
	struct gen_pool *pool = NULL;

	iova_pool_start = dev->init_data->iova_base;
	iova_pool_end =  iova_pool_start+dev->init_data->iova_size;
	iova_pf_pool_start = dev->init_data->pfch_iova_base;
	iova_pf_pool_end = iova_pf_pool_start + dev->init_data->pfch_iova_size;

	if ((iova >= iova_pool_start)
		&& ((iova + iova_length) <= iova_pool_end)) {
		pool = dev->pool;
	} else if ((iova >= iova_pf_pool_start)
		&& ((iova + iova_length) <= iova_pf_pool_end)) {
		pool = dev->pf_ch_pool;
	} else {
		IOMMU_ERR("free err iova 0x%lx len 0x%zx \
				pf pool 0x%lx end 0x%lx \
				fm pool 0x%lx end 0x%lx\n",
				iova, iova_length,
				iova_pf_pool_start,
				iova_pf_pool_end,
				iova_pool_start,
				iova_pool_end);
		return;
	}

	gen_pool_free(pool, iova, iova_length);
	return;

}

static int sprd_iommupf_iova_map(struct sprd_iommu_dev *dev, unsigned long iova,
				size_t iova_length, struct sg_table *table,
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
	map_param.sg_offset = p_param->sg_offset;

	err = sprd_iommudrv_map(dev->private, &map_param);
	if (SPRD_NO_ERR == err)
		return 0;
	else
		return -1;


}

static int sprd_iommupf_iova_unmap(struct sprd_iommu_dev *dev,
				  unsigned long iova,
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

static int sprd_iommupf_hw_suspend(struct sprd_iommu_dev *dev)
{
	sprd_iommupf_clk_disable(dev);
	return 0;
}

static int sprd_iommupf_hw_resume(struct sprd_iommu_dev *dev)
{
	/*set clk back first*/
	sprd_iommupf_clk_enable(dev);
	return 0;
}

struct sprd_iommu_ops sprd_iommupf_hw_ops = {
	.init = sprd_iommupf_hw_init,
	.exit = sprd_iommupf_hw_exit,
	.iova_alloc = sprd_iommupf_iova_alloc,
	.iova_free = sprd_iommupf_iova_free,
	.iova_map = sprd_iommupf_iova_map,
	.iova_unmap = sprd_iommupf_iova_unmap,
	.backup = NULL,
	.restore = NULL,
	.enable = NULL,
	.disable = NULL,
	.dump = NULL,
	.open = NULL,
	.release = NULL,
	.suspend = sprd_iommupf_hw_suspend,
	.resume = sprd_iommupf_hw_resume,
	.iova_unmap_orphaned = NULL,
};

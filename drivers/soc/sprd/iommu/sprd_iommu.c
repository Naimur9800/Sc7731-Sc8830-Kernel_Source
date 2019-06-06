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
#include <linux/miscdevice.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include <linux/genalloc.h>
#include <linux/sprd_iommu.h>
#include <linux/sprd_ion.h>
#include <linux/scatterlist.h>
#include <asm/io.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/dma-buf.h>
#include <linux/pm_runtime.h>
#include <linux/pm.h>

#include "sprd_iommu_sysfs.h"
#include "drv/com/sprd_com.h"

static int sprd_iommu_probe(struct platform_device *pdev);
static int sprd_iommu_remove(struct platform_device *pdev);
static void sprd_iommu_calc_pgt_addr(struct device_node *np,
		struct sprd_iommu_init_data *pdata);

static struct sprd_iommu_list_data sprd_iommu_list[SPRD_IOMMU_MAX] = {
	{ .iommu_id = SPRD_IOMMU_VSP,
	   .enabled = false,
	   .iommu_dev = NULL},

	{ .iommu_id = SPRD_IOMMU_DCAM,
	   .enabled = false,
	   .iommu_dev = NULL},

	{ .iommu_id = SPRD_IOMMU_DCAM1,
	   .enabled = false,
	   .iommu_dev = NULL},

	{ .iommu_id = SPRD_IOMMU_CPP,
	   .enabled = false,
	   .iommu_dev = NULL},

	{ .iommu_id = SPRD_IOMMU_GSP,
	   .enabled = false,
	   .iommu_dev = NULL},

	{ .iommu_id = SPRD_IOMMU_JPG,
	   .enabled = false,
	   .iommu_dev = NULL},

	{ .iommu_id = SPRD_IOMMU_DISP,
	   .enabled = false,
	   .iommu_dev = NULL},

	{ .iommu_id = SPRD_IOMMU_ISP,
	   .enabled = false,
	   .iommu_dev = NULL},

	{ .iommu_id = SPRD_IOMMU_ISP1,
	   .enabled = false,
	   .iommu_dev = NULL},
};

static const struct of_device_id sprd_iommu_ids[] = {
	{ .compatible = "sprd,iommu-vsp",
	   .data = (void *)(IOMMU_VSP)},

	{ .compatible = "sprd,iommu-gsp0",
	   .data = (void *)(IOMMU_GSP0)},

	{ .compatible = "sprd,iommu-gsp1",
	   .data = (void *)(IOMMU_GSP1)},

	{ .compatible = "sprd,iommu-gsp",
	   .data = (void *)(IOMMU_GSP)},

	{ .compatible = "sprd,iommu-mm",
	   .data = (void *)(IOMMU_MM)},

	{ .compatible = "sprd,iommu-dcam",
	   .data = (void *)(IOMMU_DCAM)},

	{ .compatible = "sprd,iommu-dispc",
	   .data = (void *)(IOMMU_DISPC)},

	{ .compatible = "sprd,iommu-vpp",
	   .data = (void *)(IOMMU_VPP)},

	{ .compatible = "sprd,iommupf-r1-dcam",
	   .data = (void *)(IOMMUPF_R1_DCAM)},

	{ .compatible = "sprd,iommupf-r2-dispc",
	   .data = (void *)(IOMMUPF_R2_DISPC)},

	{ .compatible = "sprd,iommupf-r2-dcam",
	   .data = (void *)(IOMMUPF_R2_DCAM)},

	{ .compatible = "sprd,iommupf-r2-cpp_jpg",
	   .data = (void *)(IOMMUPF_R2_CPP_JPG)},

	{ .compatible = "sprd,iommu-cpp-jpg",
	   .data = (void *)(IOMMU_CPP_JPG)},

	{ .compatible = "sprd,iommuex-gsp",
	   .data = (void *)(IOMMU_EX_GSP)},

	{ .compatible = "sprd,iommuex-vsp",
	   .data = (void *)(IOMMU_EX_VSP)},

	{ .compatible = "sprd,iommu-r4-vsp",
	   .data = (void *)(IOMMU_R4_VSP)},

	{ .compatible = "sprd,iommu-r4-gsp",
	   .data = (void *)(IOMMU_R4_GSP)},

	{ .compatible = "sprd,iommuex-dcam",
	   .data = (void *)(IOMMU_EX_DCAM)},

	{ .compatible = "sprd,iommuex-isp",
	   .data = (void *)(IOMMU_EX_ISP)},

	{ .compatible = "sprd,iommuex-cpp",
	   .data = (void *)(IOMMU_EX_CPP)},

	{ .compatible = "sprd,iommuex-jpg",
	   .data = (void *)(IOMMU_EX_JPG)},

	{ .compatible = "sprd,iommuex-dispc",
	   .data = (void *)(IOMMU_EX_DISP)},

	{ .compatible = "sprd,iommuexi-gsp",
	   .data = (void *)(IOMMU_EXI_GSP)},

	{ .compatible = "sprd,iommuexi-vsp",
	   .data = (void *)(IOMMU_EXI_VSP)},

	{ .compatible = "sprd,iommuexi-dcam",
	   .data = (void *)(IOMMU_EXI_DCAM)},

	{ .compatible = "sprd,iommuexi-dcam1",
	   .data = (void *)(IOMMU_EXI_DCAM1)},

	{ .compatible = "sprd,iommuexi-isp",
	   .data = (void *)(IOMMU_EXI_ISP)},

	{ .compatible = "sprd,iommuexi-isp1",
	   .data = (void *)(IOMMU_EXI_ISP1)},

	{ .compatible = "sprd,iommuexi-cpp",
	   .data = (void *)(IOMMU_EXI_CPP)},

	{ .compatible = "sprd,iommuexi-jpg",
	   .data = (void *)(IOMMU_EXI_JPG)},

	{ .compatible = "sprd,iommuexi-dispc",
	   .data = (void *)(IOMMU_EXI_DISP)},

	{ .compatible = "sprd,iommuexj1-gsp",
	   .data = (void *)(IOMMU_EXJ1_GSP)},

	{ .compatible = "sprd,iommuexj1-dispc",
	   .data = (void *)(IOMMU_EXJ1_DISP)},

	{ .compatible = "sprd,iommuexj1-vsp",
	   .data = (void *)(IOMMU_EXJ1_VSP)},

	{ .compatible = "sprd,iommuexj1-dcam",
	   .data = (void *)(IOMMU_EXJ1_DCAM)},

	{ .compatible = "sprd,iommuexj1-isp",
	   .data = (void *)(IOMMU_EXJ1_ISP)},

	{ .compatible = "sprd,iommuexj1-cpp",
	   .data = (void *)(IOMMU_EXJ1_CPP)},

	{ .compatible = "sprd,iommuexj1-jpg",
	   .data = (void *)(IOMMU_EXJ1_JPG)},

	{ .compatible = "sprd,iommuexle-gsp",
	   .data = (void *)(IOMMU_EXLE_GSP)},

	{ .compatible = "sprd,iommuexle-dispc",
	   .data = (void *)(IOMMU_EXLE_DISP)},

	{ .compatible = "sprd,iommuexle-vsp",
	   .data = (void *)(IOMMU_EXLE_VSP)},

	{ .compatible = "sprd,iommuexle-dcam",
	   .data = (void *)(IOMMU_EXLE_DCAM)},

	{ .compatible = "sprd,iommuexle-isp",
	   .data = (void *)(IOMMU_EXLE_ISP)},

	{ .compatible = "sprd,iommuexle-cpp",
	   .data = (void *)(IOMMU_EXLE_CPP)},

	{ .compatible = "sprd,iommuexle-jpg",
	   .data = (void *)(IOMMU_EXLE_JPG)},

	{ .compatible = "sprd,iommuexpk2-gsp",
	   .data = (void *)(IOMMU_EXPK2_GSP)},

	{ .compatible = "sprd,iommuexpk2-dispc",
	   .data = (void *)(IOMMU_EXPK2_DISP)},

	{ .compatible = "sprd,iommuexpk2-vsp",
	   .data = (void *)(IOMMU_EXPK2_VSP)},

	{ .compatible = "sprd,iommuexpk2-dcam",
	   .data = (void *)(IOMMU_EXPK2_DCAM)},

	{ .compatible = "sprd,iommuexpk2-isp",
	   .data = (void *)(IOMMU_EXPK2_ISP)},

	{ .compatible = "sprd,iommuexpk2-cpp",
	   .data = (void *)(IOMMU_EXPK2_CPP)},

	{ .compatible = "sprd,iommuexpk2-jpg",
	   .data = (void *)(IOMMU_EXPK2_JPG)},

	{ .compatible = "sprd,iommuexl3-dispc",
	   .data = (void *)(IOMMU_EXL3_DISP)},

	{ .compatible = "sprd,iommuexl3-vsp",
	   .data = (void *)(IOMMU_EXL3_VSP)},

	{ .compatible = "sprd,iommuexl3-dcam",
	   .data = (void *)(IOMMU_EXL3_DCAM)},

	{ .compatible = "sprd,iommuexl3-isp",
	   .data = (void *)(IOMMU_EXL3_ISP)},

	{ .compatible = "sprd,iommuexl3-cpp",
	   .data = (void *)(IOMMU_EXL3_CPP)},

	{ .compatible = "sprd,iommuexl3-jpg",
	   .data = (void *)(IOMMU_EXL3_JPG)},

	{ .compatible = "sprd,iommuexl5-dispc",
	   .data = (void *)(IOMMU_EXL5_DISP)},

	{ .compatible = "sprd,iommuexl5-vsp",
	   .data = (void *)(IOMMU_EXL5_VSP)},

	{ .compatible = "sprd,iommuexl5-dcam",
	   .data = (void *)(IOMMU_EXL5_DCAM)},

	{ .compatible = "sprd,iommuexl5-isp",
	   .data = (void *)(IOMMU_EXL5_ISP)},

	{ .compatible = "sprd,iommuexl5-cpp",
	   .data = (void *)(IOMMU_EXL5_CPP)},

	{ .compatible = "sprd,iommuexl5-jpg",
	   .data = (void *)(IOMMU_EXL5_JPG)},

};

static struct platform_driver iommu_driver = {
	.probe = sprd_iommu_probe,
	.remove = sprd_iommu_remove,
	.driver = {
		.name = "sprd_iommu_drv",
		.of_match_table = sprd_iommu_ids,
	},
};

static void sprd_iommu_set_list(struct sprd_iommu_dev *iommu_dev)
{
	int id;

	if (iommu_dev == NULL) {
		pr_err("%s, iommu_dev == NULL!\n", __func__);
		return;
	}

	switch (iommu_dev->init_data->id) {
	case IOMMU_EX_VSP:
		sprd_iommu_list[SPRD_IOMMU_VSP].enabled = true;
		sprd_iommu_list[SPRD_IOMMU_VSP].iommu_dev = iommu_dev;
		iommu_dev->id = SPRD_IOMMU_VSP;
		break;
	case IOMMU_EX_DCAM:
		id = SPRD_IOMMU_DCAM;
		if (iommu_dev->init_data->iommuex_rev == 4 &&
		    iommu_dev->org_id == IOMMU_EXI_DCAM1)
			id = SPRD_IOMMU_DCAM1;
		sprd_iommu_list[id].enabled = true;
		sprd_iommu_list[id].iommu_dev = iommu_dev;
		iommu_dev->id = id;
		break;
	case IOMMU_EX_CPP:
		sprd_iommu_list[SPRD_IOMMU_CPP].enabled = true;
		sprd_iommu_list[SPRD_IOMMU_CPP].iommu_dev = iommu_dev;
		iommu_dev->id = SPRD_IOMMU_CPP;
		break;
	case IOMMU_EX_GSP:
		sprd_iommu_list[SPRD_IOMMU_GSP].enabled = true;
		sprd_iommu_list[SPRD_IOMMU_GSP].iommu_dev = iommu_dev;
		iommu_dev->id = SPRD_IOMMU_GSP;
		break;
	case IOMMU_EX_JPG:
		sprd_iommu_list[SPRD_IOMMU_JPG].enabled = true;
		sprd_iommu_list[SPRD_IOMMU_JPG].iommu_dev = iommu_dev;
		iommu_dev->id = SPRD_IOMMU_JPG;
		break;
	case IOMMU_EX_DISP:
		sprd_iommu_list[SPRD_IOMMU_DISP].enabled = true;
		sprd_iommu_list[SPRD_IOMMU_DISP].iommu_dev = iommu_dev;
		iommu_dev->id = SPRD_IOMMU_DISP;
		break;
	case IOMMU_EX_ISP:
		id = SPRD_IOMMU_ISP;
		if (iommu_dev->init_data->iommuex_rev == 4 &&
		    iommu_dev->org_id == IOMMU_EXI_ISP1)
			id = SPRD_IOMMU_ISP1;
		sprd_iommu_list[id].enabled = true;
		sprd_iommu_list[id].iommu_dev = iommu_dev;
		iommu_dev->id = id;
		break;
	default:
		pr_err("%s, no iommu id: %d\n", __func__,
			iommu_dev->init_data->id);
		iommu_dev->id = -1;
		break;
	}
}

static bool sprd_iommu_is_dev_valid_master(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret;

	/*
	 * An iommu master has an iommus property containing a list of phandles
	 * to iommu nodes, each with an #iommu-cells property with value 0.
	 */
	ret = of_count_phandle_with_args(np, "iommus", "#iommu-cells");
	return (ret > 0);
}

static struct sprd_iommu_dev *sprd_iommu_get_subnode(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret;
	struct of_phandle_args args;

	ret = of_parse_phandle_with_args(np, "iommus", "#iommu-cells", 0,
					 &args);
	if (ret) {
		IOMMU_ERR("of_parse_phandle_with_args(%s) => %d\n",
			np->full_name, ret);
		return NULL;
	}

	if (args.args_count != 0) {
		IOMMU_ERR("err param for %s (found %d, expected 0)\n",
			args.np->full_name, args.args_count);
		return NULL;
	}

	return args.np->data;
}

/**
*if sg already mapped, return mapped iova directly.otherwise, alloc new iova for sg
*/
static bool sprd_iommu_target_sg(struct sprd_iommu_dev *iommu_dev,
							unsigned long sg_table_addr,
							unsigned long *iova_addr)
{
	int index = 0;
	for (index = 0; index < SPRD_MAX_SG_CACHED_CNT; index++) {
		if (SG_SLOT_USED == iommu_dev->sg_pool.slot[index].status) {
			if (iommu_dev->sg_pool.slot[index].sg_table_addr == sg_table_addr) {
				*iova_addr = iommu_dev->sg_pool.slot[index].iova_addr;
				iommu_dev->sg_pool.slot[index].map_usrs++;
				break;
			}
		}
	}
	if (index < SPRD_MAX_SG_CACHED_CNT)
		return true;
	else
		return false;
}

static bool sprd_iommu_target_buf(struct sprd_iommu_dev *iommu_dev,
						void *buf_addr,
						unsigned long *iova_addr)
{
	int index = 0;

	for (index = 0; index < SPRD_MAX_SG_CACHED_CNT; index++) {
		if (iommu_dev->sg_pool.slot[index].status == SG_SLOT_USED &&
		   iommu_dev->sg_pool.slot[index].buf_addr == buf_addr) {
			*iova_addr = iommu_dev->sg_pool.slot[index].iova_addr;
			iommu_dev->sg_pool.slot[index].map_usrs++;
			break;
		}
	}
	if (index < SPRD_MAX_SG_CACHED_CNT)
		return true;
	else
		return false;
}

static bool sprd_iommu_target_iova_find_buf(struct sprd_iommu_dev *iommu_dev,
						unsigned long iova_addr,
						size_t iova_size,
						void **buf)
{
	int index = 0;

	for (index = 0; index < SPRD_MAX_SG_CACHED_CNT; index++) {
		if (iommu_dev->sg_pool.slot[index].status == SG_SLOT_USED &&
		   iommu_dev->sg_pool.slot[index].iova_addr == iova_addr &&
		   iommu_dev->sg_pool.slot[index].iova_size == iova_size) {
			*buf = iommu_dev->sg_pool.slot[index].buf_addr;
			break;
		}
	}
	if (index < SPRD_MAX_SG_CACHED_CNT)
		return true;
	else
		return false;
}

/*
*find iova by sg_table, used by camera HAL
*/
static bool sprd_iommu_target_sg_find_iova(struct sprd_iommu_dev *iommu_dev,
							unsigned long sg_table_addr,
							size_t iova_size,
							unsigned long *iova_addr)
{
	int index = 0;
	for (index = 0; index < SPRD_MAX_SG_CACHED_CNT; index++) {
		if (SG_SLOT_USED == iommu_dev->sg_pool.slot[index].status) {
			if ((iommu_dev->sg_pool.slot[index].sg_table_addr == sg_table_addr)
				&& (iommu_dev->sg_pool.slot[index].iova_size == iova_size)) {
				*iova_addr = iommu_dev->sg_pool.slot[index].iova_addr;
				break;
			}
		}
	}
	if (index < SPRD_MAX_SG_CACHED_CNT)
		return true;
	else
		return false;
}

static bool sprd_iommu_target_buf_find_iova(struct sprd_iommu_dev *iommu_dev,
						void *buf_addr,
						size_t iova_size,
						unsigned long *iova_addr)
{
	int index;

	for (index = 0; index < SPRD_MAX_SG_CACHED_CNT; index++) {
		if (iommu_dev->sg_pool.slot[index].status == SG_SLOT_USED &&
		   iommu_dev->sg_pool.slot[index].buf_addr == buf_addr &&
		   iommu_dev->sg_pool.slot[index].iova_size == iova_size) {
			*iova_addr = iommu_dev->sg_pool.slot[index].iova_addr;
			break;
		}
	}
	if (index < SPRD_MAX_SG_CACHED_CNT)
		return true;
	else
		return false;
}

static bool sprd_iommu_target_iova(struct sprd_iommu_dev *iommu_dev,
							unsigned long iova_addr,
							unsigned long iova_size)
{
	int index = 0;

	for (index = 0; index < SPRD_MAX_SG_CACHED_CNT; index++) {
		if (SG_SLOT_USED == iommu_dev->sg_pool.slot[index].status) {
			if ((iommu_dev->sg_pool.slot[index].iova_addr == iova_addr)
				&& (iommu_dev->sg_pool.slot[index].iova_size == iova_size)) {
				break;
			}
		}
	}

	if (index < SPRD_MAX_SG_CACHED_CNT)
		return true;
	else
		return false;
}

/**
* insert fresh sg table addr into sg record pool
*/
static bool sprd_iommu_insert_sg_iova(struct sprd_iommu_dev *iommu_dev,
							unsigned long sg_table_addr,
							unsigned long iova_addr,
							unsigned long iova_size)
{
	int index = 0;

	for (index = 0; index < SPRD_MAX_SG_CACHED_CNT; index++) {
		if (SG_SLOT_FREE == iommu_dev->sg_pool.slot[index].status) {
			iommu_dev->sg_pool.slot[index].sg_table_addr = sg_table_addr;
			iommu_dev->sg_pool.slot[index].iova_addr = iova_addr;
			iommu_dev->sg_pool.slot[index].iova_size = iova_size;
			iommu_dev->sg_pool.slot[index].status = SG_SLOT_USED;
			iommu_dev->sg_pool.pool_cnt++;
			iommu_dev->sg_pool.slot[index].map_usrs++;
			break;
		}
	}

	if (index < SPRD_MAX_SG_CACHED_CNT)
		return true;
	else
		return false;
}

static bool sprd_iommu_insert_slot(struct sprd_iommu_dev *iommu_dev,
						unsigned long sg_table_addr,
						void *buf_addr,
						unsigned long iova_addr,
						unsigned long iova_size)
{
	int index = 0;
	struct sprd_iommu_sg_rec *rec;

	for (index = 0; index < SPRD_MAX_SG_CACHED_CNT; index++) {
		rec = &(iommu_dev->sg_pool.slot[index]);
		if (rec->status == SG_SLOT_FREE) {
			rec->sg_table_addr = sg_table_addr;
			rec->buf_addr = buf_addr;
			rec->iova_addr = iova_addr;
			rec->iova_size = iova_size;
			rec->status = SG_SLOT_USED;
			rec->map_usrs++;
			iommu_dev->sg_pool.pool_cnt++;
			break;
		}
	}

	if (index < SPRD_MAX_SG_CACHED_CNT)
		return true;
	else
		return false;
}

static void sprd_iommupf_sign_alloc_iova(
	struct sprd_iommu_dev *iommu_dev, unsigned long iova_addr)
{
	int index = 0;

	for (index = 0; index < SPRD_MAX_SG_CACHED_CNT; index++) {
		if (iommu_dev->mmupf_iovaarray[index] == 0) {
			iommu_dev->mmupf_iovaarray[index] = iova_addr;
			return;
		}
	}
}

static bool sprd_iommupf_check_free_iova(
	struct sprd_iommu_dev *iommu_dev, unsigned long iova_addr)
{
	int index = 0;

	for (index = 0; index < SPRD_MAX_SG_CACHED_CNT; index++) {
		if (iommu_dev->mmupf_iovaarray[index] == iova_addr) {
			iommu_dev->mmupf_iovaarray[index] = 0;
			return true;
		}
	}
	return false;
}

/**
* remove timeout sg table addr from sg pool
*/
static bool sprd_iommu_remove_sg_iova(struct sprd_iommu_dev *iommu_dev,
							unsigned long iova_addr,
							bool *be_free)
{
	int index = 0;

	for (index = 0; index < SPRD_MAX_SG_CACHED_CNT; index++) {
		if (SG_SLOT_USED == iommu_dev->sg_pool.slot[index].status) {
			if (iommu_dev->sg_pool.slot[index].iova_addr == iova_addr) {
				iommu_dev->sg_pool.slot[index].map_usrs--;

				if (0 == iommu_dev->sg_pool.slot[index].map_usrs) {
					iommu_dev->sg_pool.slot[index].sg_table_addr = 0;
					iommu_dev->sg_pool.slot[index].iova_addr = 0;
					iommu_dev->sg_pool.slot[index].iova_size = 0;
					iommu_dev->sg_pool.slot[index].status = SG_SLOT_FREE;
					iommu_dev->sg_pool.pool_cnt--;
					*be_free = true;
				} else
					*be_free = false;
				break;
			}
		}
	}

	if (index < SPRD_MAX_SG_CACHED_CNT)
		return true;
	else
		return false;
}

static bool sprd_iommu_clear_sg_iova(struct sprd_iommu_dev *iommu_dev,
			void *buf_addr,
			unsigned long sg_addr, unsigned long size,
			unsigned long *iova)
{
	int index;
	struct sprd_iommu_sg_rec *rec;
	bool ret = false;

	for (index = 0; index < SPRD_MAX_SG_CACHED_CNT; index++) {
		rec = &(iommu_dev->sg_pool.slot[index]);
		if (rec->status == SG_SLOT_USED &&
		    rec->buf_addr == buf_addr &&
		    rec->sg_table_addr == sg_addr &&
		    rec->iova_size == size) {
			rec->map_usrs = 0;
			rec->status = SG_SLOT_FREE;
			iommu_dev->sg_pool.pool_cnt--;
			*iova = rec->iova_addr;
			ret = true;
			break;
		}
	}

	return ret;
}

static void sprd_iommu_pool_show(struct sprd_iommu_dev *iommu_dev)
{
	int index;
	struct sprd_iommu_sg_rec *rec;

	if (iommu_dev->id == SPRD_IOMMU_VSP ||
	    iommu_dev->id == SPRD_IOMMU_DISP)
		return;

	IOMMU_ERR("%s restore, map_count %u\n",
		iommu_dev->init_data->name,
		iommu_dev->map_count);

	if (iommu_dev->map_count > 0)
		for (index = 0; index < SPRD_MAX_SG_CACHED_CNT; index++) {
			rec = &(iommu_dev->sg_pool.slot[index]);
			if (rec->status == SG_SLOT_USED) {
				IOMMU_ERR("Warning! buffer iova 0x%lx size 0x%lx sg 0x%lx buf %p map_usrs %d should be unmapped!\n",
					rec->iova_addr, rec->iova_size,
					rec->sg_table_addr, rec->buf_addr,
					rec->map_usrs);
			}
		}
}

int sprd_iommu_attach_device(struct device *dev)
{
	struct device_node *np = NULL;
	struct of_phandle_args args;
	const char *status;
	int ret;
	int err = -1;

	if (NULL == dev) {
		IOMMU_ERR("null parameter err!\n");
		return -EINVAL;
	}

	np = dev->of_node;
	/*
	 * An iommu master has an iommus property containing a list of phandles
	 * to iommu nodes, each with an #iommu-cells property with value 0.
	 */
	ret = of_parse_phandle_with_args(np, "iommus", "#iommu-cells", 0,
					 &args);
	if (!ret) {
		ret = of_property_read_string(args.np, "status", &status);
		if (!ret) {
			if (!strcmp("disabled", status))
				err = -EPERM;
			else
				err = 0;
		} else
			err = -EPERM;
	} else
		err = -EPERM;

	return err;
}

int sprd_iommu_dettach_device(struct device *dev)
{
	struct device_node *np = NULL;

	if (NULL == dev) {
		IOMMU_ERR("null parameter err!\n");
		return -EINVAL;
	}

	np = dev->of_node;

	return 0;
}

int sprd_iommu_get_kaddr(struct device *dev,
			struct sprd_iommu_map_data *data)
{
	int ret = 0;
	struct sprd_iommu_dev *iommu_dev = NULL;
	unsigned long iova = 0;
	unsigned long flag = 0;
	bool sg_cached = false;
	bool sg_update_result = true;

	if ((NULL == dev) || (NULL == data)) {
		IOMMU_ERR("null parameter err! dev %p data %p\n", dev, data);
		return -EINVAL;
	}

	if (NULL == data->table) {
		IOMMU_ERR("null sg tabl pointer!\n");
		return -EINVAL;
	}

	if (!sprd_iommu_is_dev_valid_master(dev)) {
		IOMMU_ERR("illegal master\n");
		return -EINVAL;
	}

	iommu_dev = sprd_iommu_get_subnode(dev);
	if (NULL == iommu_dev) {
		IOMMU_ERR("get null iommu dev\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&iommu_dev->pgt_lock, flag);
	if ((SPRD_IOMMU_PF_CH_READ == data->ch_type)
		|| (SPRD_IOMMU_PF_CH_WRITE == data->ch_type)) {
		/*prefetch channel ,always alloc new iova*/
		iova = iommu_dev->ops->iova_alloc(iommu_dev,
				data->iova_size, data);
		if (0 == iova) {
			IOMMU_ERR("%s alloc iova fail,iova: 0x%lx\n",
				iommu_dev->init_data->name, iova);
			data->iova_addr = 0;
			ret = -ENOMEM;
			goto out;
		}

		sprd_iommupf_sign_alloc_iova(iommu_dev, iova);
		ret = iommu_dev->ops->iova_map(iommu_dev,
				iova, data->iova_size, data->table, data);
		if (ret) {
			IOMMU_ERR("%s map error, iova: 0x%lx, ret: %d!\n",
				iommu_dev->init_data->name, iova, ret);
			sprd_iommupf_check_free_iova(iommu_dev, iova);
			iommu_dev->ops->iova_free(iommu_dev, iova, data->iova_size);
			data->iova_addr = 0;
			ret = -ENOMEM;
			goto out;
		}
		iommu_dev->map_count++;
		data->iova_addr = iova + (data->sg_offset & 0xfff);
	} else {
		/**search the sg_cache_pool to identify if sg_table already mapped;
		* if yes, return cached iova directly, otherwise, alloc new iova for it;
		*/
		sg_cached = sprd_iommu_target_sg(iommu_dev,
					(unsigned long)data->table,
					(unsigned long *)&iova);
		if (sg_cached) {
			data->iova_addr = iova;
			ret = 0;
			goto out;
		}

		/*new sg, alloc for it*/
		iova = iommu_dev->ops->iova_alloc(iommu_dev,
				data->iova_size, data);
		if (0 == iova) {
			IOMMU_ERR("%s alloc iova fail,iova: 0x%lx\n",
				iommu_dev->init_data->name, iova);
			data->iova_addr = 0;
			ret = -ENOMEM;
			goto out;
		}

		ret = iommu_dev->ops->iova_map(iommu_dev,
				iova, data->iova_size, data->table, data);
		if (ret) {
			IOMMU_ERR("%s map error, iova: 0x%lx, ret: %d!\n",
				iommu_dev->init_data->name, iova, ret);
			iommu_dev->ops->iova_free(iommu_dev, iova, data->iova_size);
			data->iova_addr = 0;
			ret = -ENOMEM;
			goto out;
		}
		iommu_dev->map_count++;
		data->iova_addr = iova;
		sg_update_result = sprd_iommu_insert_sg_iova(iommu_dev,
					(unsigned long)data->table,
					data->iova_addr,
					data->iova_size);
		if (!sg_update_result) {
			IOMMU_ERR("fail, sg pool full\n");
			ret = -ENOMEM;
			goto out;
		}
	}

out:
	spin_unlock_irqrestore(&iommu_dev->pgt_lock, flag);
	return ret;
}


int sprd_iommu_get_kaddr_with_sg(struct device *dev,
			struct sprd_iommu_map_data *data)
{
	int ret = 0;
	struct sprd_iommu_dev *iommu_dev = NULL;
	unsigned long iova = 0;
	unsigned long flag = 0;
	bool sg_cached = false;
	bool sg_update_result = true;

	if ((NULL == dev) || (NULL == data)) {
		IOMMU_ERR("null parameter err! dev %p data %p\n", dev, data);
		return -EINVAL;
	}

	if (NULL == data->table) {
		IOMMU_ERR("null sg tabl pointer!\n");
		return -EINVAL;
	}

	if (!sprd_iommu_is_dev_valid_master(dev)) {
		IOMMU_ERR("illegal master\n");
		return -EINVAL;
	}

	iommu_dev = sprd_iommu_get_subnode(dev);
	if (NULL == iommu_dev) {
		IOMMU_ERR("get null iommu dev\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&iommu_dev->pgt_lock, flag);
	if ((SPRD_IOMMU_PF_CH_READ == data->ch_type)
		|| (SPRD_IOMMU_PF_CH_WRITE == data->ch_type)) {
		/*prefetch channel ,always alloc new iova*/
		iova = iommu_dev->ops->iova_alloc(iommu_dev,
				data->iova_size, data);
		if (0 == iova) {
			IOMMU_ERR("%s alloc iova fail,iova: 0x%lx\n",
				iommu_dev->init_data->name, iova);
			data->iova_addr = 0;
			ret = -ENOMEM;
			goto out;
		}

		sprd_iommupf_sign_alloc_iova(iommu_dev, iova);
		ret = iommu_dev->ops->iova_map(iommu_dev,
				iova, data->iova_size, data->table, data);
		if (ret) {
			IOMMU_ERR("%s map error, iova: 0x%lx, ret: %d!\n",
				iommu_dev->init_data->name, iova, ret);
			sprd_iommupf_check_free_iova(iommu_dev, iova);
			iommu_dev->ops->iova_free(iommu_dev, iova, data->iova_size);
			data->iova_addr = 0;
			ret = -ENOMEM;
			goto out;
		}
		iommu_dev->map_count++;
		data->iova_addr = iova + (data->sg_offset & 0xfff);
	} else {
		/**search the sg_cache_pool to identify if sg_table already mapped;
		* if yes, return cached iova directly, otherwise, alloc new iova for it;
		*/
		sg_cached = sprd_iommu_target_sg(iommu_dev,
					(unsigned long)data->table,
					(unsigned long *)&iova);
		if (sg_cached) {
			data->iova_addr = iova;
			ret = 0;
			goto out;
		}

		/*new sg, alloc for it*/
		iova = iommu_dev->ops->iova_alloc(iommu_dev,
				data->iova_size, data);
		if (0 == iova) {
			IOMMU_ERR("%s alloc iova fail,iova: 0x%lx\n",
				iommu_dev->init_data->name, iova);
			data->iova_addr = 0;
			ret = -ENOMEM;
			goto out;
		}

		ret = iommu_dev->ops->iova_map(iommu_dev,
				iova, data->iova_size, data->table, data);
		if (ret) {
			IOMMU_ERR("%s map error, iova: 0x%lx, ret: %d!\n",
				iommu_dev->init_data->name, iova, ret);
			iommu_dev->ops->iova_free(iommu_dev, iova, data->iova_size);
			data->iova_addr = 0;
			ret = -ENOMEM;
			goto out;
		}
		iommu_dev->map_count++;
		data->iova_addr = iova;
		sg_update_result = sprd_iommu_insert_sg_iova(iommu_dev,
					(unsigned long)data->table,
					data->iova_addr,
					data->iova_size);
		if (!sg_update_result) {
			IOMMU_ERR("fail, sg pool full\n");
			ret = -ENOMEM;
			goto out;
		}
	}

out:
	spin_unlock_irqrestore(&iommu_dev->pgt_lock, flag);
	return ret;
}

int sprd_iommu_map(struct device *dev, struct sprd_iommu_map_data *data)
{
	int ret = 0;
	struct sprd_iommu_dev *iommu_dev = NULL;
	unsigned long iova = 0;
	unsigned long flag = 0;
	bool buf_cached = false;
	bool buf_insert = true;
	struct sg_table *table = NULL;

	if (dev == NULL || data == NULL) {
		IOMMU_ERR("null parameter err! dev %p data %p\n", dev, data);
		return -EINVAL;
	}

	if (data->buf == NULL) {
		IOMMU_ERR("null buf pointer!\n");
		return -EINVAL;
	}

	if (!sprd_iommu_is_dev_valid_master(dev)) {
		IOMMU_ERR("illegal master\n");
		return -EINVAL;
	}

	iommu_dev = sprd_iommu_get_subnode(dev);
	if (iommu_dev == NULL) {
		IOMMU_ERR("get null iommu dev\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&iommu_dev->pgt_lock, flag);

	ret = sprd_ion_get_sg(data->buf, &table);
	if (ret || table == NULL) {
		IOMMU_ERR("%s get sg error, buf %p size 0x%zx ret %d table %p\n",
			  iommu_dev->init_data->name,
			  data->buf, data->iova_size, ret, table);
		data->iova_addr = 0;
		ret = -EINVAL;
		goto out;
	}

	/*record iommu map count in ion buffer for checking iova leak*/
	sprd_ion_set_dma(data->buf, iommu_dev->id);

	/**search the sg_cache_pool to identify if buf already mapped;
	* if yes, return cached iova directly, otherwise, alloc new iova for it;
	*/
	buf_cached = sprd_iommu_target_buf(iommu_dev,
				data->buf,
				(unsigned long *)&iova);
	if (buf_cached) {
		data->iova_addr = iova;
		ret = 0;
		IOMMU_DEBUG("%s cached iova 0x%lx size 0x%zx buf %p\n",
			  iommu_dev->init_data->name, iova,
			  data->iova_size, data->buf);
		goto out;
	}

	/*new sg, alloc for it*/
	iova = iommu_dev->ops->iova_alloc(iommu_dev,
			data->iova_size, data);
	if (iova == 0) {
		IOMMU_ERR("%s alloc error iova 0x%lx size 0x%zx buf %p\n",
			  iommu_dev->init_data->name, iova,
			  data->iova_size, data->buf);
		data->iova_addr = 0;
		ret = -ENOMEM;
		goto out1;
	}

	ret = iommu_dev->ops->iova_map(iommu_dev,
			iova, data->iova_size, table, data);
	if (ret) {
		IOMMU_ERR("%s error, iova 0x%lx size 0x%zx ret %d buf %p\n",
			  iommu_dev->init_data->name,
			  iova, data->iova_size, ret, data->buf);
		iommu_dev->ops->iova_free(iommu_dev, iova, data->iova_size);
		data->iova_addr = 0;
		ret = -ENOMEM;
		goto out1;
	}
	iommu_dev->map_count++;
	data->iova_addr = iova;
	buf_insert = sprd_iommu_insert_slot(iommu_dev,
				(unsigned long)table,
				data->buf,
				data->iova_addr,
				data->iova_size);
	if (!buf_insert) {
		IOMMU_ERR("%s error pool full iova 0x%lx size 0x%zx buf %p\n",
				iommu_dev->init_data->name, iova,
				data->iova_size, data->buf);
		iommu_dev->ops->iova_unmap(iommu_dev,
				iova, data->iova_size);
		iommu_dev->ops->iova_free(iommu_dev, iova, data->iova_size);
		ret = -ENOMEM;
		goto out1;
	}

	IOMMU_DEBUG("%s iova 0x%lx size 0x%zx buf %p\n",
		  iommu_dev->init_data->name, iova,
		  data->iova_size, data->buf);

	spin_unlock_irqrestore(&iommu_dev->pgt_lock, flag);
	return ret;

out1:
	sprd_ion_put_dma(data->buf, iommu_dev->id);
out:
	spin_unlock_irqrestore(&iommu_dev->pgt_lock, flag);
	return ret;
}

int sprd_iommu_free_kaddr(struct device *dev,
			struct sprd_iommu_unmap_data *data)
{
	int ret = 0;
	struct sprd_iommu_dev *iommu_dev = NULL;
	bool be_free = false;
	unsigned long flag = 0;
	bool valid_iova = false;

	if ((NULL == dev) || (NULL == data)) {
		IOMMU_ERR("null parameter err! dev %p data %p\n", dev, data);
		return -EINVAL;
	}

	if (!sprd_iommu_is_dev_valid_master(dev)) {
		IOMMU_ERR("illegal master\n");
		return -EINVAL;
	}

	iommu_dev = sprd_iommu_get_subnode(dev);
	if (NULL == iommu_dev) {
		IOMMU_ERR("get null iommu dev\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&iommu_dev->pgt_lock, flag);
	if ((SPRD_IOMMU_PF_CH_READ == data->ch_type)
		|| (SPRD_IOMMU_PF_CH_WRITE == data->ch_type)) {
		data->iova_addr &= ~0xfff;
		be_free = sprd_iommupf_check_free_iova(iommu_dev,
					data->iova_addr);
		if (be_free) {
			ret = iommu_dev->ops->iova_unmap(iommu_dev,
					data->iova_addr,
					data->iova_size);
			iommu_dev->map_count--;
			iommu_dev->ops->iova_free(iommu_dev,
					data->iova_addr,
					data->iova_size);
		} else {
			IOMMU_ERR("%s iommupf free iova 0x%lx again!\n",
				iommu_dev->init_data->name, data->iova_addr);
			ret = -EINVAL;
		}
	} else {
		/*check iova legal*/
		valid_iova = sprd_iommu_target_iova(iommu_dev, data->iova_addr, data->iova_size);
		if (valid_iova) {
			sprd_iommu_remove_sg_iova(iommu_dev, data->iova_addr, &be_free);
			if (be_free) {
				ret = iommu_dev->ops->iova_unmap(iommu_dev,
					data->iova_addr, data->iova_size);
				iommu_dev->map_count--;
				iommu_dev->ops->iova_free(iommu_dev,
					data->iova_addr,
					data->iova_size);
			} else {
				ret = 0;
			}
		} else {
			IOMMU_ERR("%s illegal iova 0x%lx len 0x%zx sg %p\n",
				iommu_dev->init_data->name,
				data->iova_addr, data->iova_size, data->table);
			ret = -EINVAL;
		}
	}

	spin_unlock_irqrestore(&iommu_dev->pgt_lock, flag);
	return ret;
}

int sprd_iommu_free_kaddr_with_id(struct device *dev,
			struct sprd_iommu_unmap_data *data, u32 channel_id)
{
	int ret = 0;
	struct sprd_iommu_dev *iommu_dev = NULL;
	bool be_free = false;
	unsigned long flag = 0;
	bool valid_iova = false;

	if ((dev == NULL) || (data == NULL)) {
		IOMMU_ERR("null parameter err! dev %p data %p\n", dev, data);
		return -EINVAL;
	}

	if (!sprd_iommu_is_dev_valid_master(dev)) {
		IOMMU_ERR("illegal master\n");
		return -EINVAL;
	}

	iommu_dev = sprd_iommu_get_subnode(dev);
	if (iommu_dev == NULL) {
		IOMMU_ERR("get null iommu dev\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&iommu_dev->pgt_lock, flag);
	if ((data->ch_type == SPRD_IOMMU_PF_CH_READ)
		|| (data->ch_type == SPRD_IOMMU_PF_CH_WRITE)) {
		data->iova_addr &= ~0xfff;
		be_free = sprd_iommupf_check_free_iova(iommu_dev,
					data->iova_addr);
		if (be_free) {
			ret = iommu_dev->ops->iova_unmap(iommu_dev,
					data->iova_addr,
					data->iova_size);
			iommu_dev->map_count--;
			iommu_dev->ops->iova_free(iommu_dev,
					data->iova_addr,
					data->iova_size);
		} else {
			IOMMU_ERR("%s iommupf free iova 0x%lx again!\n",
				iommu_dev->init_data->name, data->iova_addr);
			ret = -EINVAL;
		}
	} else {
		/*check iova legal*/
		valid_iova = sprd_iommu_target_iova(iommu_dev, data->iova_addr,
						    data->iova_size);
		if (valid_iova) {
			sprd_iommu_remove_sg_iova(iommu_dev, data->iova_addr,
						  &be_free);
			if (be_free) {
				iommu_dev->ch_type = data->ch_type;
				iommu_dev->channel_id = channel_id;
				ret = iommu_dev->ops->iova_unmap(iommu_dev,
					data->iova_addr, data->iova_size);
				iommu_dev->map_count--;
				iommu_dev->ops->iova_free(iommu_dev,
					data->iova_addr,
					data->iova_size);
				iommu_dev->ch_type = SPRD_IOMMU_CH_TYPE_INVALID;
			} else {
				ret = 0;
			}
		} else {
			IOMMU_ERR("%s illega iova 0x%lx len 0x%zx\n",
				iommu_dev->init_data->name,
				data->iova_addr, data->iova_size);
			ret = -EINVAL;
		}
	}

	spin_unlock_irqrestore(&iommu_dev->pgt_lock, flag);
	return ret;
}

int sprd_iommu_free_kaddr_with_sg(struct device *dev,
			struct sg_table *sg, size_t iova_size)
{
	int ret = 0;
	struct sprd_iommu_dev *iommu_dev = NULL;
	unsigned long iova = 0;
	bool be_free = false;
	unsigned long flag = 0;
	bool valid_iova = false;

	if (NULL == dev) {
		IOMMU_ERR("null parameter err! dev %p\n", dev);
		return -EINVAL;
	}

	if (!sprd_iommu_is_dev_valid_master(dev)) {
		IOMMU_ERR("illegal master\n");
		return -EINVAL;
	}

	iommu_dev = sprd_iommu_get_subnode(dev);
	if (NULL == iommu_dev) {
		IOMMU_ERR("get null iommu dev\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&iommu_dev->pgt_lock, flag);

	/*check iova legal*/
	valid_iova = sprd_iommu_target_sg_find_iova(iommu_dev,
					(unsigned long)sg, iova_size, &iova);
	if (valid_iova) {
		sprd_iommu_remove_sg_iova(iommu_dev, iova, &be_free);
		if (be_free) {
			ret = iommu_dev->ops->iova_unmap(iommu_dev,
							 iova, iova_size);
			iommu_dev->map_count--;
			iommu_dev->ops->iova_free(iommu_dev, iova, iova_size);
		} else {
			ret = 0;
		}
	} else {
		IOMMU_ERR("%s illegal sg %p len 0x%zx\n",
			iommu_dev->init_data->name, sg, iova_size);
		ret = -EINVAL;
	}

	spin_unlock_irqrestore(&iommu_dev->pgt_lock, flag);

	return ret;
}

int sprd_iommu_unmap(struct device *dev, struct sprd_iommu_unmap_data *data)
{
	int ret = 0;
	struct sprd_iommu_dev *iommu_dev = NULL;
	bool be_free = false;
	unsigned long flag = 0;
	bool valid_iova = false;
	bool valid_buf = false;
	void *buf;
	unsigned long iova;

	if (dev == NULL || data == NULL) {
		IOMMU_ERR("null parameter err! dev %p data %p\n", dev, data);
		return -EINVAL;
	}

	if (!sprd_iommu_is_dev_valid_master(dev)) {
		IOMMU_ERR("illegal master\n");
		return -EINVAL;
	}

	iommu_dev = sprd_iommu_get_subnode(dev);
	if (iommu_dev == NULL) {
		IOMMU_ERR("get null iommu dev\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&iommu_dev->pgt_lock, flag);

	/*check iova legal*/
	valid_iova = sprd_iommu_target_iova_find_buf(iommu_dev,
				data->iova_addr, data->iova_size, &buf);
	if (valid_iova) {
		iova = data->iova_addr;
	} else {
		valid_buf = sprd_iommu_target_buf_find_iova(iommu_dev,
					data->buf,
					data->iova_size, &iova);
		if (valid_buf) {
			buf = data->buf;
			data->iova_addr = iova;
		} else {
			IOMMU_ERR("%s illegal error iova 0x%lx buf %p size 0x%zx\n",
				iommu_dev->init_data->name,
				data->iova_addr, data->buf,
				data->iova_size);
			ret = -EINVAL;
			goto out;
		}
	}

	sprd_ion_put_dma(buf, iommu_dev->id);

	sprd_iommu_remove_sg_iova(iommu_dev, iova, &be_free);
	if (be_free) {
		iommu_dev->ch_type = data->ch_type;
		iommu_dev->channel_id = data->channel_id;
		ret = iommu_dev->ops->iova_unmap(iommu_dev,
				iova, data->iova_size);
		if (ret)
			IOMMU_ERR("%s error iova 0x%lx 0x%zx buf %p\n",
				iommu_dev->init_data->name,
				iova, data->iova_size, buf);
		iommu_dev->map_count--;
		iommu_dev->ops->iova_free(iommu_dev,
			iova, data->iova_size);
		iommu_dev->ch_type = SPRD_IOMMU_CH_TYPE_INVALID;
		IOMMU_DEBUG("%s iova 0x%lx size 0x%zx buf %p\n",
			  iommu_dev->init_data->name,
			  iova, data->iova_size, buf);
	} else {
		IOMMU_DEBUG("%s cached iova 0x%lx size 0x%zx buf %p\n",
			  iommu_dev->init_data->name,
			  iova, data->iova_size, buf);
		ret = 0;
	}

out:
	spin_unlock_irqrestore(&iommu_dev->pgt_lock, flag);
	return ret;
}

int sprd_iommu_unmap_orphaned(struct sprd_iommu_unmap_data *data)
{
	int ret;
	struct sprd_iommu_dev *iommu_dev;
	unsigned long iova;
	unsigned long flag = 0;

	if (data == NULL) {
		IOMMU_ERR("null parameter error! data %p\n", data);
		return -EINVAL;
	}

	if (data->dev_id >= SPRD_IOMMU_MAX) {
		IOMMU_ERR("dev id error %d\n", data->dev_id);
		return -EINVAL;
	}

	iommu_dev = sprd_iommu_list[data->dev_id].iommu_dev;

	spin_lock_irqsave(&iommu_dev->pgt_lock, flag);

	ret = sprd_iommu_clear_sg_iova(iommu_dev, data->buf,
					(unsigned long)(data->table),
					data->iova_size, &iova);
	if (ret) {
		ret = iommu_dev->ops->iova_unmap_orphaned(iommu_dev,
						 iova, data->iova_size);
		iommu_dev->map_count--;
		iommu_dev->ops->iova_free(iommu_dev, iova, data->iova_size);
		IOMMU_ERR("%s iova leak error, buf %p id %d iova 0x%lx size 0x%zx\n",
			iommu_dev->init_data->name, data->buf, data->dev_id,
			iova, data->iova_size);
	} else
		IOMMU_ERR("%s illegal error buf %p id %d size 0x%zx\n",
			iommu_dev->init_data->name, data->buf, data->dev_id,
			data->iova_size);

	spin_unlock_irqrestore(&iommu_dev->pgt_lock, flag);

	return ret;
}

int sprd_iommu_suspend(struct device *dev)
{
	struct sprd_iommu_dev *iommu_dev = NULL;
	int ret = 0;

	if (NULL == dev) {
		IOMMU_ERR("null parameter err\n");
		return -EINVAL;
	}

	if (!sprd_iommu_is_dev_valid_master(dev)) {
		IOMMU_ERR("illegal master\n");
		return -EINVAL;
	}

	iommu_dev = sprd_iommu_get_subnode(dev);
	if (NULL == iommu_dev) {
		IOMMU_ERR("get null iommu dev\n");
		return -EINVAL;
	}

	mutex_lock(&iommu_dev->status_mutex);
	if (iommu_dev->status_count) {
		iommu_dev->ops->suspend(iommu_dev);
		iommu_dev->status_count--;
	} else
		IOMMU_ERR("%s have not enable!\n", iommu_dev->init_data->name);

	mutex_unlock(&iommu_dev->status_mutex);
	return ret;
}

int sprd_iommu_resume(struct device *dev)
{
	struct sprd_iommu_dev *iommu_dev = NULL;
	int ret = 0;

	if (NULL == dev) {
		IOMMU_ERR("%s null parameter err!\n",
			iommu_dev->init_data->name);
		return -EINVAL;
	}

	if (!sprd_iommu_is_dev_valid_master(dev)) {
		IOMMU_ERR("illegal master\n");
		return -EINVAL;
	}

	iommu_dev = sprd_iommu_get_subnode(dev);
	if (NULL == iommu_dev) {
		IOMMU_ERR("get null iommu dev\n");
		return -EINVAL;
	}

	mutex_lock(&iommu_dev->status_mutex);
	iommu_dev->ops->resume(iommu_dev);
	iommu_dev->status_count++;
	mutex_unlock(&iommu_dev->status_mutex);

	return ret;
}

int sprd_iommu_restore(struct device *dev)
{
	struct sprd_iommu_dev *iommu_dev = NULL;
	int ret = 0;

	if (dev == NULL) {
		IOMMU_ERR("null parameter err!\n");
		return -EINVAL;
	}

	if (!sprd_iommu_is_dev_valid_master(dev)) {
		IOMMU_ERR("illegal master\n");
		return -EINVAL;
	}

	iommu_dev = sprd_iommu_get_subnode(dev);
	if (iommu_dev == NULL) {
		IOMMU_ERR("get null iommu dev\n");
		return -EINVAL;
	}

	if (iommu_dev->ops->restore)
		iommu_dev->ops->restore(iommu_dev);
	else
		ret = -1;

	sprd_iommu_pool_show(iommu_dev);
	return ret;
}

int sprd_iommu_open(struct device *dev)
{
	struct sprd_iommu_dev *iommu_dev = NULL;
	int ret = 0;

	if (dev == NULL) {
		IOMMU_ERR("null parameter err!\n");
		return -EINVAL;
	}

	if (!sprd_iommu_is_dev_valid_master(dev)) {
		IOMMU_ERR("illegal master\n");
		return -EINVAL;
	}

	iommu_dev = sprd_iommu_get_subnode(dev);
	if (iommu_dev == NULL) {
		IOMMU_ERR("get null iommu dev\n");
		return -EINVAL;
	}

	atomic_inc(&iommu_dev->iommu_dev_cnt);

	return ret;

}

int sprd_iommu_release(struct device *dev)
{
	struct sprd_iommu_dev *iommu_dev = NULL;
	int ret = 0;
	int index;
	unsigned long iova_addr;
	unsigned long iova_size;

	if (dev == NULL) {
		IOMMU_ERR("null parameter err!\n");
		return -EINVAL;
	}

	if (!sprd_iommu_is_dev_valid_master(dev)) {
		IOMMU_ERR("illegal master\n");
		return -EINVAL;
	}

	iommu_dev = sprd_iommu_get_subnode(dev);
	if (iommu_dev == NULL) {
		IOMMU_ERR("get null iommu dev\n");
		return -EINVAL;
	}
	if (atomic_dec_return(&iommu_dev->iommu_dev_cnt))
		return ret;

	for (index = 0; index < SPRD_MAX_SG_CACHED_CNT; index++) {
		if (iommu_dev->sg_pool.slot[index].status == SG_SLOT_USED) {
			iova_addr = iommu_dev->sg_pool.slot[index].iova_addr;
			iova_size = iommu_dev->sg_pool.slot[index].iova_size;
			iommu_dev->ops->iova_free(iommu_dev,
				iova_addr, iova_size);
			IOMMU_WARN("%s iova:0x%lx size:0x%lx not free,we do!\n",
				iommu_dev->init_data->name,
				iova_addr, iova_size);
			iommu_dev->sg_pool.slot[index].sg_table_addr = 0;
			iommu_dev->sg_pool.slot[index].iova_addr = 0;
			iommu_dev->sg_pool.slot[index].iova_size = 0;
			iommu_dev->sg_pool.slot[index].status = SG_SLOT_FREE;
			iommu_dev->sg_pool.pool_cnt--;
			iommu_dev->sg_pool.slot[index].map_usrs = 0;
		}
	}

	iommu_dev->map_count = 0;
	if (iommu_dev->ops->release)
		iommu_dev->ops->release(iommu_dev);
	else
		ret = -1;

	return ret;

}

static int sprd_iommu_get_resource(struct device_node *np,
				struct sprd_iommu_init_data *pdata)
{
	int err = 0;
	uint32_t val;
	struct resource res;
	struct device_node *np_memory;
	uint32_t out_values[4];

	err = of_address_to_resource(np, 0, &res);
	if (err < 0)
		return err;

	IOMMU_INFO("pgt_base phy:0x%lx\n", (unsigned long)(res.start));
	pdata->pgt_base = (unsigned long)ioremap_nocache(res.start,
		resource_size(&res));
	BUG_ON(pdata->pgt_base == 0);
	/*sharkl2 pgt_size is va range*/
	pdata->pgt_size = resource_size(&res);
	IOMMU_INFO("pgt_base:%lx,pgt_size:%zx\n", pdata->pgt_base,
		pdata->pgt_size);

	err = of_address_to_resource(np, 1, &res);
	if (err < 0)
		return err;

	/*sharkl2 ctrl_reg is iommu base reg addr*/
	IOMMU_INFO("ctrl_reg phy:0x%lx\n", (unsigned long)(res.start));
	pdata->ctrl_reg = (unsigned long)ioremap_nocache(res.start,
		resource_size(&res));
	BUG_ON(!pdata->ctrl_reg);
	IOMMU_INFO("ctrl_reg:0x%lx\n", pdata->ctrl_reg);

	err = of_address_to_resource(np, 2, &res);
	if (err < 0)
		return err;

	pdata->iova_base = res.start;
	pdata->iova_size = resource_size(&res);
	IOMMU_INFO("iova_base:0x%lx,iova_size:%zx\n",
		pdata->iova_base,
		pdata->iova_size);

	err = of_property_read_u32(np, "sprd,reserved-fault-page", &val);
	if (err) {
		unsigned long page =  0;

		page = __get_free_page(GFP_KERNEL);
		if (page)
			pdata->fault_page = virt_to_phys((void *)page);
		else
			pdata->fault_page = 0;

		IOMMU_INFO("fault_page: 0x%lx\n", pdata->fault_page);
	} else {
		IOMMU_INFO("reserved fault page phy:0x%x\n", val);
		pdata->fault_page = val;
	}

	err = of_property_read_u32(np, "sprd,reserved-rr-page", &val);
	if (err) {
		IOMMU_INFO("no reserved rr page addr\n");
		pdata->re_route_page = 0;
	} else {
		IOMMU_INFO("reserved rr page phy:0x%x\n", val);
		pdata->re_route_page = val;
	}

	if (0 <  pdata->iommupf_rev) {
		err = of_address_to_resource(np, 3, &res);
		if (err < 0)
			return err;

		pdata->pfch_iova_base = res.start;
		pdata->pfch_iova_size = resource_size(&res);
		IOMMU_INFO("prefetch channel iova_base:0x%lx,iova_size:%zx\n",
			pdata->pfch_iova_base, pdata->pfch_iova_size);

		err = of_property_read_u32(np, "fullmode-en", &val);
		if (err) {
			IOMMU_INFO("no sprd,fullmode_en\n");
			pdata->fullmode_en = 0;
		} else {
			IOMMU_INFO("fullmode_en %d\n", val);
			pdata->fullmode_en = val;
		}

		err = of_property_read_u32(np, "total-ch-num", &val);
		if (err) {
			IOMMU_INFO("no sprd,total_ch_num\n");
			pdata->total_ch_num = 0;
		} else {
			IOMMU_INFO("total_ch_num %d\n", val);
			pdata->total_ch_num = val;
		}

		err = of_property_read_u32(np, "pf-ch-write-start", &val);
		if (err) {
			IOMMU_INFO("no sprd,pf_ch_write_start\n");
			pdata->pf_ch_write_start = 0;
		} else {
			IOMMU_INFO("pf_ch_write_start %d\n", val);
			pdata->pf_ch_write_start = val;
		}

		err = of_property_read_u32(np, "pf-ch-write-end", &val);
		if (err) {
			IOMMU_INFO("no sprd,pf_ch_write_end\n");
			pdata->pf_ch_write_end = 0;
		} else {
			IOMMU_INFO("pf_ch_write_end %d\n", val);
			pdata->pf_ch_write_end = val;
		}

		err = of_property_read_u32(np, "pf-ch-read-start", &val);
		if (err) {
			IOMMU_INFO("no sprd,pf_ch_read_start\n");
			pdata->pf_ch_read_start = 0;
		} else {
			IOMMU_INFO("pf_ch_read_start %d\n", val);
			pdata->pf_ch_read_start = val;
		}

		err = of_property_read_u32(np, "pf-ch-read-end", &val);
		if (err) {
			IOMMU_INFO("no sprd,pf_ch_read_end\n");
			pdata->pf_ch_read_end = 0;
		} else {
			IOMMU_INFO("pf_ch_read_end %d\n", val);
			pdata->pf_ch_read_end = val;
		}

		err = of_property_read_u32(np, "fullmode-ch-write", &val);
		if (err) {
			IOMMU_INFO("no sprd,fullmode_ch_write\n");
			pdata->fullmode_ch_write = 0;
		} else {
			IOMMU_INFO("fullmode_ch_write %d\n", val);
			pdata->fullmode_ch_write = val;
		}

		err = of_property_read_u32(np, "fullmode-ch-read", &val);
		if (err) {
			IOMMU_INFO("no sprd,fullmode_ch_read\n");
			pdata->fullmode_ch_read = 0;
		} else {
			IOMMU_INFO("fullmode_ch_read %d\n", val);
			pdata->fullmode_ch_read = val;
		}
	}

	/*get mmu page table reserved memory*/
	np_memory = of_parse_phandle(np, "memory-region", 0);
	if (!np_memory) {
		pdata->pagt_base_ddr = 0;
		pdata->pagt_ddr_size = 0;
	} else {
#ifdef CONFIG_64BIT
		err = of_property_read_u32_array(np_memory, "reg",
				out_values, 4);
		if (!err) {
			pdata->pagt_base_ddr = out_values[0];
			pdata->pagt_base_ddr =
					pdata->pagt_base_ddr << 32;
			pdata->pagt_base_ddr |= out_values[1];

			pdata->pagt_ddr_size = out_values[2];
			pdata->pagt_ddr_size =
					pdata->pagt_ddr_size << 32;
			pdata->pagt_ddr_size |= out_values[3];
		} else {
			pdata->pagt_base_ddr = 0;
			pdata->pagt_ddr_size = 0;
		}
#else
		err = of_property_read_u32_array(np_memory, "reg",
				out_values, 2);
		if (!err) {
			pdata->pagt_base_ddr = out_values[0];
			pdata->pagt_ddr_size = out_values[1];
		} else {
			pdata->pagt_base_ddr = 0;
			pdata->pagt_ddr_size = 0;
		}
#endif
		/*iwhale2 all pagetable in reserved memory,
		*we must be calculate start addr
		*/
		sprd_iommu_calc_pgt_addr(np, pdata);
	}

	return 0;
}

static void sprd_iommu_calc_pgt_addr(struct device_node *np,
	struct sprd_iommu_init_data *pdata)
{
	int err = 0;
	uint32_t page_table_offset = 0;
	uint32_t page_table_size = 0;
	ulong pagt_base_ddr = pdata->pagt_base_ddr;

	if (pagt_base_ddr == 0)
		return;

	err = of_property_read_u32(np, "page-table-offset", &page_table_offset);
	if (err)
		return;

	err = of_property_read_u32(np, "page-table-size", &page_table_size);
	if (!err) {
		switch (page_table_offset) {
		case 0: /*gsp iommu*/
		case 1: /*vsp iommu*/
			pagt_base_ddr = pagt_base_ddr +
				page_table_offset * page_table_size * 1024;
			break;
		case 2: /*dcam/isp*/
			pagt_base_ddr = pagt_base_ddr + GSP_VSP_IOMMU_SIZE;
			page_table_size = page_table_size *
					(pdata->total_ch_num - 2);
			break;
		case 3: /*cpp/jpeg*/
			pagt_base_ddr = pagt_base_ddr + GSP_VSP_IOMMU_SIZE
						+ DCAM_ISP_IOMMU_SIZE;
			page_table_size = page_table_size *
						(pdata->total_ch_num - 2);
			break;
		case 4: /*dispc*/
			pagt_base_ddr = pagt_base_ddr + GSP_VSP_IOMMU_SIZE
				+ DCAM_ISP_IOMMU_SIZE + CPP_JPEG_IOMMU_SIZE;
			/*don't have fullmode and no write mode*/
			page_table_size = page_table_size *
					(pdata->total_ch_num);
			break;
		}
		pdata->pagt_base_ddr = pagt_base_ddr;
		pdata->pagt_ddr_size = page_table_size * 1024;
	}
}

static int sprd_iommu_probe(struct platform_device *pdev)
{
	int err = 0;
	struct device_node *np = pdev->dev.of_node;
	struct sprd_iommu_dev *iommu_dev = NULL;
	struct sprd_iommu_init_data *pdata = NULL;

	IOMMU_INFO("start\n");

	iommu_dev = kzalloc(sizeof(struct sprd_iommu_dev), GFP_KERNEL);
	if (NULL == iommu_dev) {
		IOMMU_ERR("fail to kzalloc\n");
		return -ENOMEM;
	}

	pdata = kzalloc(sizeof(struct sprd_iommu_init_data), GFP_KERNEL);
	if (NULL == pdata) {
		IOMMU_ERR("fail to kzalloc\n");
		kfree(iommu_dev);
		return -ENOMEM;
	}

	pdata->id = (int)((enum IOMMU_ID)
		((of_match_node(sprd_iommu_ids, np))->data));

	switch (pdata->id) {
	case IOMMU_GSP:
	case IOMMU_MM:
	{
		pdata->iommu_rev = 1;
		iommu_dev->ops = &sprd_iommu_hw_ops;
		break;
	}
	case IOMMU_VSP:
	case IOMMU_DCAM:
	case IOMMU_DISPC:
	case IOMMU_GSP0:
	case IOMMU_GSP1:
	case IOMMU_VPP:
	case IOMMU_CPP_JPG:
	{
		pdata->iommu_rev = 2;
		iommu_dev->ops = &sprd_iommu_hw_ops;
		break;
	}
	case IOMMUPF_R1_DCAM:
	{
		pdata->iommupf_rev = 1;
		iommu_dev->ops = &sprd_iommupf_hw_ops;
		break;
	}
	case IOMMUPF_R2_DISPC:
	case IOMMUPF_R2_DCAM:
	case IOMMUPF_R2_CPP_JPG:
	{
		pdata->iommupf_rev = 2;
		iommu_dev->ops = &sprd_iommupf_hw_ops;
		break;
	}
	/*for iwhale2 iommu r4p0*/
	case IOMMU_R4_VSP:
	case IOMMU_R4_GSP:
	{
		pdata->iommu_rev = 5;
		iommu_dev->ops = &sprd_iommu_hw_ops;
		break;
	}
	/*for sharkl2/isharkl2 iommu*/
	case IOMMU_EX_VSP:
	case IOMMU_EX_DCAM:
	case IOMMU_EX_CPP:
	case IOMMU_EX_GSP:
	case IOMMU_EX_JPG:
	case IOMMU_EX_DISP:
	case IOMMU_EX_ISP:
	{
		pdata->iommuex_rev = 3;
		iommu_dev->ops = &sprd_iommuex_hw_ops;
		break;
	}
	case IOMMU_EXI_VSP:
	case IOMMU_EXI_DCAM:
	case IOMMU_EXI_DCAM1:
	case IOMMU_EXI_CPP:
	case IOMMU_EXI_GSP:
	case IOMMU_EXI_JPG:
	case IOMMU_EXI_DISP:
	case IOMMU_EXI_ISP:
	case IOMMU_EXI_ISP1:
	{
		pdata->iommuex_rev = 4;
		iommu_dev->ops = &sprd_iommuex_hw_ops;
		iommu_dev->org_id = pdata->id;
		if (pdata->id == IOMMU_EXI_GSP)
			pdata->id = IOMMU_EX_GSP;
		else if (pdata->id == IOMMU_EXI_VSP)
			pdata->id = IOMMU_EX_VSP;
		else if (pdata->id == IOMMU_EXI_DISP)
			pdata->id = IOMMU_EX_DISP;
		else if (pdata->id == IOMMU_EXI_DCAM
				|| pdata->id == IOMMU_EXI_DCAM1)
			pdata->id = IOMMU_EX_DCAM;
		else if (pdata->id == IOMMU_EXI_CPP)
			pdata->id = IOMMU_EX_CPP;
		else if (pdata->id == IOMMU_EXI_JPG)
			pdata->id = IOMMU_EX_JPG;
		else if (pdata->id == IOMMU_EXI_ISP
				|| pdata->id == IOMMU_EXI_ISP1)
			pdata->id = IOMMU_EX_ISP;
		break;
	}
	/*for sharklj1 iommu*/
	case IOMMU_EXJ1_VSP:
	case IOMMU_EXJ1_DCAM:
	case IOMMU_EXJ1_CPP:
	case IOMMU_EXJ1_GSP:
	case IOMMU_EXJ1_JPG:
	case IOMMU_EXJ1_DISP:
	case IOMMU_EXJ1_ISP:
	{
		pdata->iommuex_rev = 6;
		iommu_dev->ops = &sprd_iommuex_hw_ops;
		if (pdata->id == IOMMU_EXJ1_GSP)
			pdata->id = IOMMU_EX_GSP;
		else if (pdata->id == IOMMU_EXJ1_DISP)
			pdata->id = IOMMU_EX_DISP;
		else if (pdata->id == IOMMU_EXJ1_VSP)
			pdata->id = IOMMU_EX_VSP;
		else if (pdata->id == IOMMU_EXJ1_DCAM)
			pdata->id = IOMMU_EX_DCAM;
		else if (pdata->id == IOMMU_EXJ1_ISP)
			pdata->id = IOMMU_EX_ISP;
		else if (pdata->id == IOMMU_EXJ1_CPP)
			pdata->id = IOMMU_EX_CPP;
		else if (pdata->id == IOMMU_EXJ1_JPG)
			pdata->id = IOMMU_EX_JPG;

		break;
	}
	/*for sharkle iommu*/
	case IOMMU_EXLE_VSP:
	case IOMMU_EXLE_DCAM:
	case IOMMU_EXLE_CPP:
	case IOMMU_EXLE_GSP:
	case IOMMU_EXLE_JPG:
	case IOMMU_EXLE_DISP:
	case IOMMU_EXLE_ISP:
	{
		pdata->iommuex_rev = 7;
		iommu_dev->ops = &sprd_iommuex_hw_ops;
		if (pdata->id == IOMMU_EXLE_GSP)
			pdata->id = IOMMU_EX_GSP;
		else if (pdata->id == IOMMU_EXLE_DISP)
			pdata->id = IOMMU_EX_DISP;
		else if (pdata->id == IOMMU_EXLE_VSP)
			pdata->id = IOMMU_EX_VSP;
		else if (pdata->id == IOMMU_EXLE_DCAM)
			pdata->id = IOMMU_EX_DCAM;
		else if (pdata->id == IOMMU_EXLE_ISP)
			pdata->id = IOMMU_EX_ISP;
		else if (pdata->id == IOMMU_EXLE_CPP)
			pdata->id = IOMMU_EX_CPP;
		else if (pdata->id == IOMMU_EXLE_JPG)
			pdata->id = IOMMU_EX_JPG;

		break;
	}
	/*for pike2 iommu*/
	case IOMMU_EXPK2_VSP:
	case IOMMU_EXPK2_DCAM:
	case IOMMU_EXPK2_CPP:
	case IOMMU_EXPK2_GSP:
	case IOMMU_EXPK2_JPG:
	case IOMMU_EXPK2_DISP:
	case IOMMU_EXPK2_ISP:
	{
		pdata->iommuex_rev = 8;
		iommu_dev->ops = &sprd_iommuex_hw_ops;
		if (pdata->id == IOMMU_EXPK2_GSP)
			pdata->id = IOMMU_EX_GSP;
		else if (pdata->id == IOMMU_EXPK2_DISP)
			pdata->id = IOMMU_EX_DISP;
		else if (pdata->id == IOMMU_EXPK2_VSP)
			pdata->id = IOMMU_EX_VSP;
		else if (pdata->id == IOMMU_EXPK2_DCAM)
			pdata->id = IOMMU_EX_DCAM;
		else if (pdata->id == IOMMU_EXPK2_ISP)
			pdata->id = IOMMU_EX_ISP;
		else if (pdata->id == IOMMU_EXPK2_CPP)
			pdata->id = IOMMU_EX_CPP;
		else if (pdata->id == IOMMU_EXPK2_JPG)
			pdata->id = IOMMU_EX_JPG;

		break;
	}
	/*for sharkl3 iommu*/
	case IOMMU_EXL3_VSP:
	case IOMMU_EXL3_DCAM:
	case IOMMU_EXL3_CPP:
	case IOMMU_EXL3_JPG:
	case IOMMU_EXL3_DISP:
	case IOMMU_EXL3_ISP:
	{
		pdata->iommuex_rev = 9;
		iommu_dev->ops = &sprd_iommuex_hw_ops;
		if (pdata->id == IOMMU_EXL3_DISP)
			pdata->id = IOMMU_EX_DISP;
		else if (pdata->id == IOMMU_EXL3_VSP)
			pdata->id = IOMMU_EX_VSP;
		else if (pdata->id == IOMMU_EXL3_DCAM)
			pdata->id = IOMMU_EX_DCAM;
		else if (pdata->id == IOMMU_EXL3_ISP)
			pdata->id = IOMMU_EX_ISP;
		else if (pdata->id == IOMMU_EXL3_CPP)
			pdata->id = IOMMU_EX_CPP;
		else if (pdata->id == IOMMU_EXL3_JPG)
			pdata->id = IOMMU_EX_JPG;

		break;
	}
	/*for sharkl5 iommu*/
	case IOMMU_EXL5_VSP:
	case IOMMU_EXL5_DCAM:
	case IOMMU_EXL5_CPP:
	case IOMMU_EXL5_JPG:
	case IOMMU_EXL5_DISP:
	case IOMMU_EXL5_ISP:
	{
		pdata->iommuex_rev = 10;
		iommu_dev->ops = &sprd_iommuex_hw_ops;
		if (pdata->id == IOMMU_EXL5_DISP)
			pdata->id = IOMMU_EX_DISP;
		else if (pdata->id == IOMMU_EXL5_VSP)
			pdata->id = IOMMU_EX_VSP;
		else if (pdata->id == IOMMU_EXL5_DCAM)
			pdata->id = IOMMU_EX_DCAM;
		else if (pdata->id == IOMMU_EXL5_ISP)
			pdata->id = IOMMU_EX_NEWISP;
		else if (pdata->id == IOMMU_EXL5_CPP)
			pdata->id = IOMMU_EX_CPP;
		else if (pdata->id == IOMMU_EXL5_JPG)
			pdata->id = IOMMU_EX_JPG;

		break;
	}
	default:
	{
		IOMMU_ERR("unknown iommu id %d\n", pdata->id);
		err = -ENOMEM;
		goto errout;
	}
	}

	err = sprd_iommu_get_resource(np, pdata);
	if (err) {
		IOMMU_ERR("no reg of property specified\n");
		goto errout;
	}

	/*If ddr frequency >= 500Mz, iommu need enable div2 frequency,
	 * because of limit of iommu iram frq.*/
	iommu_dev->div2_frq = 500;
	iommu_dev->light_sleep_en = false;
	iommu_dev->drv_dev = &pdev->dev;
	iommu_dev->init_data = pdata;
	iommu_dev->map_count = 0;
	iommu_dev->status_count = 0;
	iommu_dev->ch_type = SPRD_IOMMU_CH_TYPE_INVALID;
	iommu_dev->channel_id = 0;
	iommu_dev->init_data->name = (char *)(
		(of_match_node(sprd_iommu_ids, np))->compatible
		);

	err = iommu_dev->ops->init(iommu_dev, pdata);
	if (err) {
		IOMMU_ERR("iommu %s : failed init %d.\n", pdata->name, err);
		goto errout;
	}
	atomic_set(&iommu_dev->iommu_dev_cnt, 0);
	spin_lock_init(&iommu_dev->pgt_lock);
	mutex_init(&iommu_dev->status_mutex);
	memset(&iommu_dev->sg_pool, 0, sizeof(struct sprd_iommu_sg_pool));
	sprd_iommu_sysfs_create(iommu_dev, iommu_dev->init_data->name);
	platform_set_drvdata(pdev, iommu_dev);

	np->data  = iommu_dev;
	sprd_iommu_set_list(iommu_dev);
	pm_runtime_enable(&pdev->dev);
	IOMMU_INFO("%s end\n", iommu_dev->init_data->name);
	return 0;

errout:
	kfree(iommu_dev);
	kfree(pdata);
	return err;
}

static int sprd_iommu_remove(struct platform_device *pdev)
{
	struct sprd_iommu_dev *iommu_dev = platform_get_drvdata(pdev);

	sprd_iommu_sysfs_destroy(iommu_dev, iommu_dev->init_data->name);
	iommu_dev->ops->exit(iommu_dev);
	gen_pool_destroy(iommu_dev->pool);
	kfree(iommu_dev);
	return 0;
}

static int __init sprd_iommu_init(void)
{
	int err = -1;

	IOMMU_INFO("begin\n");
	err = platform_driver_register(&iommu_driver);
	if (err < 0)
		IOMMU_ERR("sprd_iommu register err: %d\n", err);
	IOMMU_INFO("end\n");
	return err;
}

static void __exit sprd_iommu_exit(void)
{
	platform_driver_unregister(&iommu_driver);
}

module_init(sprd_iommu_init);
module_exit(sprd_iommu_exit);

MODULE_DESCRIPTION("SPRD IOMMU Driver");
MODULE_LICENSE("GPL v2");

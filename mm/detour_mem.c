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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/memblock.h>
#include <linux/wait.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/mod_devicetable.h>
#include <linux/mm.h>
#include <linux/gfp.h>
#include <linux/miscdevice.h>
#include <linux/debugfs.h>

#define MAX_RMEM_DETOUR 5
#define DETOUR_MEM_TAG	"detour_mm "

struct mem_detour {
	const char	 *name;
	phys_addr_t	 base;
	phys_addr_t	 size;
	struct reserved_mem *rmem;
};

static struct mem_detour rmem_array[MAX_RMEM_DETOUR];
static int rmem_count;
static struct zone *zone;

static int __init activate_area_base_align(phys_addr_t base, phys_addr_t size)
{
	unsigned long base_pfn, pfn;
	unsigned pb_num;
	unsigned long page_total;
	unsigned long free_num;
	unsigned long remain_pages;
	phys_addr_t end = base + size;

	if (!size)
		return 0;

	pfn = base_pfn = __phys_to_pfn(base);
	page_total = size >> PAGE_SHIFT;
	pb_num = (page_total >> pageblock_order) + 1;

	pr_info(DETOUR_MEM_TAG "activate_area_base_align: [%pa-%pa].\n",
			&base, &end);

	remain_pages = page_total;

	do {
		unsigned j;

		free_num = min_t(unsigned long, remain_pages,
				pageblock_nr_pages);

		if (!free_num)
			return 0;

		base_pfn = pfn;
		for (j = free_num; j; --j, pfn++) {
			WARN_ON_ONCE(!pfn_valid(pfn));

			if (page_zone(pfn_to_page(pfn)) != zone)
				return -EINVAL;
		}

		init_detour_reserved(pfn_to_page(base_pfn), free_num);
		remain_pages -= free_num;
	} while (--pb_num);

	return 0;
}

static int __init activate_area_base_noalign(phys_addr_t base, phys_addr_t size)
{
	unsigned long base_pfn, pfn;
	unsigned long page_total;
	int i;
	phys_addr_t end;

	pfn = base_pfn = __phys_to_pfn(base);
	page_total = size >> PAGE_SHIFT;
	end = base + size;

	if (!size)
		return 0;

	pr_info(DETOUR_MEM_TAG "activate_area_base_noalign: [%pa-%pa].\n",
			&base, &end);

	for (i = page_total; i; --i, pfn++) {
		WARN_ON_ONCE(!pfn_valid(pfn));
		if (page_zone(pfn_to_page(pfn)) != zone)
			return -EINVAL;
	}
	pfn = base_pfn;

	for (i = page_total; i; --i, pfn++)
		init_detour_reserved(pfn_to_page(pfn), 1);

	return 0;
}

static int __init activate_area_all(phys_addr_t base, phys_addr_t size)
{
	phys_addr_t alignment;
	phys_addr_t align_base;
	phys_addr_t end = base + size;
	phys_addr_t remain_size = size;

	if (!size)
		return 0;

	alignment = (phys_addr_t)PAGE_SIZE << pageblock_order;
	align_base = ALIGN(base, alignment);

	pr_info(DETOUR_MEM_TAG "activate_area_all base:%pa-%pa, align_base:%pa.\n",
		&base, &end, &align_base);

	totaldetour_pages += (size / PAGE_SIZE);

	if (align_base != base) {
		end = min(end, align_base);
		remain_size -= (end - base);
		activate_area_base_noalign(base, end - base);
	}

	if (!remain_size)
		return 0;

	activate_area_base_align(align_base, remain_size);

	return 0;
}

static int __init detour_activate_area(struct mem_detour *rmem)
{
	int i;
	struct reserved_mem *this;
	unsigned long base_pfn;

	phys_addr_t base, end, cbase, cend;

	if (!rmem || !(rmem->size))
		return 0;

	base = rmem->base;
	end = rmem->base + rmem->size;

	pr_info(DETOUR_MEM_TAG "activate area:%pa end:%pa\n",
		&base, &end);

	base_pfn = __phys_to_pfn(base);
	WARN_ON_ONCE(!pfn_valid(base_pfn));
	zone = page_zone(pfn_to_page(base_pfn));

	for (i = 0; i < reserved_mem_count; i++) {
		this = &reserved_mem[i];
		cbase = this->base;
		cend = cbase + this->size;

		if (cend <= base)
			continue;

		if (cbase >= end)
			break;

		if (rmem->name && this->name &&
			!strcmp(rmem->name, this->name))
			continue;

		/*
		 * overlaps.
		 */
		if (cbase > base)
			activate_area_all(base, cbase - base);

		base = min(cend, end);
	}

	if (base < end)
		activate_area_all(base, end - base);

	pr_info(DETOUR_MEM_TAG "totaldetour pages:%ld\n",
		totaldetour_pages);

	return 0;
}

static int __init detour_init_rmem_areas(void)
{
	int num;
	int ret;

	pr_info(DETOUR_MEM_TAG "enter detour_init_rmem_areas\n");
	if (rmem_count == 0)
		return 0;

	for (num = 0; num < rmem_count; num++) {
		pr_info(DETOUR_MEM_TAG
			"Freeing mem-detour uname %s,base %pa,size 0x%lx\n",
			rmem_array[num].name,
			&(rmem_array[num].base),
			(unsigned long)(rmem_array[num].size));

		ret = detour_activate_area(&rmem_array[num]);
		if (ret)
			return ret;
	}
	return 0;
}

static void __exit detour_mem_exit(void)
{
	pr_info(DETOUR_MEM_TAG "detour_mem_exit\n");
}

/*
 * rmem reserved(denoted by compaitable in dts with string "detour-mem")
 * here will be reused as MIGRATE_MOVABLE memory.
 *
 */
static int __init detour_rmem_setup(struct reserved_mem *rmem)
{
	phys_addr_t alignment;
	phys_addr_t align_base;
	phys_addr_t memblock_end = memblock_end_of_DRAM();

	pr_info(DETOUR_MEM_TAG "enter detour_rmem_setup\n");

	if (!rmem || rmem->size == 0)
		return -EINVAL;

	if (rmem->base > memblock_end ||
		(rmem->base + rmem->size) > memblock_end) {
		pr_err(DETOUR_MEM_TAG "exceeds the ram end:%pa (rmem: %pa,0x%lx)\n",
			&memblock_end, &rmem->base, (unsigned long)rmem->size);
		return -EINVAL;
	}

	if (rmem_count == MAX_RMEM_DETOUR) {
		pr_err(DETOUR_MEM_TAG "nodes exceed the max(%d)\n",
				 MAX_RMEM_DETOUR);
		return -EINVAL;
	}

	alignment = (phys_addr_t)PAGE_SIZE << pageblock_order;
	align_base = ALIGN(rmem->base, alignment);
	if (align_base != rmem->base) {
		pr_err(DETOUR_MEM_TAG "base=0x%lx should be aligned with 0x%lx\n",
			(unsigned long)rmem->base, (unsigned long)alignment);
		return -EINVAL;
	}

	rmem_array[rmem_count].name = rmem->name;
	rmem_array[rmem_count].base = rmem->base;
	rmem_array[rmem_count].size = rmem->size;
	rmem_array[rmem_count].rmem = rmem;

	pr_info(DETOUR_MEM_TAG "setup detour_rmem at %pa, size 0x%lx, uname %s\n",
		&(rmem_array[rmem_count].base),
		(unsigned long)(rmem_array[rmem_count].size),
		rmem_array[rmem_count].name);

	rmem_count++;

	return 0;
}

RESERVEDMEM_OF_DECLARE(detour_mem, "detour-mem", detour_rmem_setup);

core_initcall(detour_init_rmem_areas);
module_exit(detour_mem_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("xiajing <xiajing@spreadst.com>");
MODULE_DESCRIPTION("free back detour-memory");

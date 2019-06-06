/* Glue code to lib/swiotlb.c */

#include <linux/pci.h>
#include <linux/cache.h>
#include <linux/module.h>
#include <linux/swiotlb.h>
#include <linux/bootmem.h>
#include <linux/dma-mapping.h>

#include <asm/iommu.h>
#include <asm/swiotlb.h>
#include <asm/dma.h>
#include <asm/xen/swiotlb-xen.h>
#include <asm/iommu_table.h>
#include <asm/cacheflush.h>
#include <asm/io.h>

int swiotlb __read_mostly;

void *x86_swiotlb_alloc_coherent(struct device *hwdev, size_t size,
					dma_addr_t *dma_handle, gfp_t flags,
					struct dma_attrs *attrs)
{
	void *vaddr;

	/*
	 * Don't print a warning when the first allocation attempt fails.
	 * swiotlb_alloc_coherent() will print a warning when the DMA
	 * memory allocation ultimately failed.
	 */
	flags |= __GFP_NOWARN;

	vaddr = dma_generic_alloc_coherent(hwdev, size, dma_handle, flags,
					   attrs);
	if (vaddr)
		return vaddr;

	return swiotlb_alloc_coherent(hwdev, size, dma_handle, flags);
}

void x86_swiotlb_free_coherent(struct device *dev, size_t size,
				      void *vaddr, dma_addr_t dma_addr,
				      struct dma_attrs *attrs)
{
	if (is_swiotlb_buffer(dma_to_phys(dev, dma_addr)))
		swiotlb_free_coherent(dev, size, vaddr, dma_addr);
	else
		dma_generic_free_coherent(dev, size, vaddr, dma_addr, attrs);
}

#ifdef CONFIG_X86_DMA_INCOHERENT
void *x86_swiotlb_alloc_incoherent(struct device *hwdev, size_t size,
					dma_addr_t *dma_handle, gfp_t flags,
					struct dma_attrs *attrs)
{
	void *vaddr;
	unsigned int nr_pages;
	struct page *page;

	/*
	 * Don't print a warning when the first allocation attempt fails.
	 * swiotlb_alloc_coherent() will print a warning when the DMA
	 * memory allocation ultimately failed.
	 */
	flags |= __GFP_NOWARN;

	nr_pages = PAGE_ALIGN(size) >> PAGE_SHIFT;
	vaddr = dma_generic_alloc_coherent(hwdev, size, dma_handle, flags,
					   attrs);
	if (vaddr) {
		page = virt_to_page(vaddr);
		set_pages_cache_type(page, nr_pages, attrs);
		return vaddr;
	}

	vaddr = swiotlb_alloc_coherent(hwdev, size, dma_handle, flags);

	if (vaddr) {
		page = virt_to_page(vaddr);
		set_pages_cache_type(page, nr_pages, attrs);
	}

	return vaddr;
}

void x86_swiotlb_free_incoherent(struct device *dev, size_t size,
				      void *vaddr, dma_addr_t dma_addr,
				      struct dma_attrs *attrs)
{
	unsigned int nr_pages = PAGE_ALIGN(size) >> PAGE_SHIFT;
	struct page *page = virt_to_page(vaddr);

	set_pages_wb(page, nr_pages);

	if (is_swiotlb_buffer(dma_to_phys(dev, dma_addr)))
		swiotlb_free_coherent(dev, size, vaddr, dma_addr);
	else
		dma_generic_free_coherent(dev, size, vaddr, dma_addr, attrs);
}

static dma_addr_t swiotlb_incoherent_map_page(struct device *dev, struct page *page,
				     unsigned long offset, size_t size,
				     enum dma_data_direction dir,
				     struct dma_attrs *attrs)
{
	dma_addr_t dev_addr;

	dev_addr = swiotlb_map_page(dev, page, offset, size, dir, attrs);
	if (!dma_get_attr(DMA_ATTR_SKIP_CPU_SYNC, attrs))
		pages_sync(page, offset, size);

	return dev_addr;
}


static void swiotlb_incoherent_unmap_page(struct device *dev, dma_addr_t dev_addr,
				 size_t size, enum dma_data_direction dir,
				 struct dma_attrs *attrs)
{
	if (!dma_get_attr(DMA_ATTR_SKIP_CPU_SYNC, attrs))
		pages_sync(phys_to_page(dev_addr), dev_addr & ~PAGE_MASK, size);

	swiotlb_unmap_page(dev, dev_addr, size, dir, attrs);
}

static int swiotlb_incoherent_map_sg_attrs(struct device *dev, struct scatterlist *sgl,
				  int nelems, enum dma_data_direction dir,
				  struct dma_attrs *attrs)
{
	struct scatterlist *sg;
	int i, ret;

	ret = swiotlb_map_sg_attrs(dev, sgl, nelems, dir, attrs);
	if (!dma_get_attr(DMA_ATTR_SKIP_CPU_SYNC, attrs)) {
		for_each_sg(sgl, sg, ret, i)
			pages_sync(sg_page(sg), sg->offset, sg->length);
	}

	return ret;
}

static void swiotlb_incoherent_unmap_sg_attrs(struct device *dev,
				     struct scatterlist *sgl, int nelems,
				     enum dma_data_direction dir,
				     struct dma_attrs *attrs)
{
	struct scatterlist *sg;
	int i;

	if (!dma_get_attr(DMA_ATTR_SKIP_CPU_SYNC, attrs)) {
		for_each_sg(sgl, sg, nelems, i)
			pages_sync(sg_page(sg), sg->offset, sg->length);
	}
	swiotlb_unmap_sg_attrs(dev, sgl, nelems, dir, attrs);
}

static void swiotlb_incoherent_sync_single_for_cpu(struct device *dev,
					  dma_addr_t dev_addr, size_t size,
					  enum dma_data_direction dir)
{
	pages_sync(phys_to_page(dev_addr), dev_addr & ~PAGE_MASK, size);
	swiotlb_sync_single_for_cpu(dev, dev_addr, size, dir);
}

static void swiotlb_incoherent_sync_single_for_device(struct device *dev,
					     dma_addr_t dev_addr, size_t size,
					     enum dma_data_direction dir)
{
	swiotlb_sync_single_for_device(dev, dev_addr, size, dir);
	pages_sync(phys_to_page(dev_addr), dev_addr & ~PAGE_MASK, size);
}

static void swiotlb_incoherent_sync_sg_for_cpu(struct device *dev,
				      struct scatterlist *sgl, int nelems,
				      enum dma_data_direction dir)
{
	struct scatterlist *sg;
	int i;

		for_each_sg(sgl, sg, nelems, i)
			pages_sync(sg_page(sg), sg->offset, sg->length);
	swiotlb_sync_sg_for_cpu(dev, sgl, nelems, dir);
}

static void swiotlb_incoherent_sync_sg_for_device(struct device *dev,
					 struct scatterlist *sgl, int nelems,
					 enum dma_data_direction dir)
{
	struct scatterlist *sg;
	int i;

	swiotlb_sync_sg_for_device(dev, sgl, nelems, dir);
	for_each_sg(sgl, sg, nelems, i)
		pages_sync(sg_page(sg), sg->offset, sg->length);
}
#endif

#ifdef CONFIG_X86_DMA_INCOHERENT
static struct dma_map_ops swiotlb_dma_ops = {
	.mapping_error = swiotlb_dma_mapping_error,
	.alloc = x86_swiotlb_alloc_incoherent,
	.free = x86_swiotlb_free_incoherent,
	.sync_single_for_cpu = swiotlb_incoherent_sync_single_for_cpu,
	.sync_single_for_device = swiotlb_incoherent_sync_single_for_device,
	.sync_sg_for_cpu = swiotlb_incoherent_sync_sg_for_cpu,
	.sync_sg_for_device = swiotlb_incoherent_sync_sg_for_device,
	.map_sg = swiotlb_incoherent_map_sg_attrs,
	.unmap_sg = swiotlb_incoherent_unmap_sg_attrs,
	.map_page = swiotlb_incoherent_map_page,
	.unmap_page = swiotlb_incoherent_unmap_page,
	.dma_supported = NULL,
};
#else

static struct dma_map_ops swiotlb_dma_ops = {
	.mapping_error = swiotlb_dma_mapping_error,
	.alloc = x86_swiotlb_alloc_coherent,
	.free = x86_swiotlb_free_coherent,
	.sync_single_for_cpu = swiotlb_sync_single_for_cpu,
	.sync_single_for_device = swiotlb_sync_single_for_device,
	.sync_sg_for_cpu = swiotlb_sync_sg_for_cpu,
	.sync_sg_for_device = swiotlb_sync_sg_for_device,
	.map_sg = swiotlb_map_sg_attrs,
	.unmap_sg = swiotlb_unmap_sg_attrs,
	.map_page = swiotlb_map_page,
	.unmap_page = swiotlb_unmap_page,
	.dma_supported = NULL,
};
#endif

/*
 * pci_swiotlb_detect_override - set swiotlb to 1 if necessary
 *
 * This returns non-zero if we are forced to use swiotlb (by the boot
 * option).
 */
int __init pci_swiotlb_detect_override(void)
{
	int use_swiotlb = swiotlb | swiotlb_force;

	if (swiotlb_force)
		swiotlb = 1;

	return use_swiotlb;
}
IOMMU_INIT_FINISH(pci_swiotlb_detect_override,
		  pci_xen_swiotlb_detect,
		  pci_swiotlb_init,
		  pci_swiotlb_late_init);

/*
 * if 4GB or more detected (and iommu=off not set) return 1
 * and set swiotlb to 1.
 */
int __init pci_swiotlb_detect_4gb(void)
{
	/* don't initialize swiotlb if iommu=off (no_iommu=1) */
#ifdef CONFIG_X86_64
	/* No pci deviec on butter soc, all device can access >4GB */
#ifndef CONFIG_X86_SPRD_ISOC
	if (!no_iommu && max_pfn > MAX_DMA32_PFN)
		swiotlb = 1;
#endif
#endif
	return swiotlb;
}
IOMMU_INIT(pci_swiotlb_detect_4gb,
	   pci_swiotlb_detect_override,
	   pci_swiotlb_init,
	   pci_swiotlb_late_init);

void __init pci_swiotlb_init(void)
{
	if (swiotlb) {
		swiotlb_init(0);
		dma_ops = &swiotlb_dma_ops;
	}
}

void __init pci_swiotlb_late_init(void)
{
	/* An IOMMU turned us off. */
	if (!swiotlb)
		swiotlb_free();
	else {
		printk(KERN_INFO "PCI-DMA: "
		       "Using software bounce buffering for IO (SWIOTLB)\n");
		swiotlb_print_info();
	}
}

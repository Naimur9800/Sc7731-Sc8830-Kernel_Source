/* Fallback functions when the main IOMMU code is not compiled in. This
 * code is roughly equivalent to i386.
 *
 * DMA incoherent cache support by Bin Gao <bin.gao@intel.com> Jul 2015
 */
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/string.h>
#include <linux/gfp.h>
#include <linux/pci.h>
#include <linux/mm.h>

#include <asm/processor.h>
#include <asm/iommu.h>
#include <asm/dma.h>
#include <asm/cacheflush.h>

static int
check_addr(char *name, struct device *hwdev, dma_addr_t bus, size_t size)
{
	if (hwdev && !dma_capable(hwdev, bus, size)) {
		if (*hwdev->dma_mask >= DMA_BIT_MASK(32))
			printk(KERN_ERR
			    "nommu_%s: overflow %Lx+%zu of device mask %Lx\n",
				name, (long long)bus, size,
				(long long)*hwdev->dma_mask);
		return 0;
	}
	return 1;
}

static dma_addr_t nommu_map_page(struct device *dev, struct page *page,
				 unsigned long offset, size_t size,
				 enum dma_data_direction dir,
				 struct dma_attrs *attrs)
{
	dma_addr_t bus = page_to_phys(page) + offset;
	WARN_ON(size == 0);
	if (!check_addr("map_single", dev, bus, size))
		return DMA_ERROR_CODE;

#ifdef CONFIG_X86_DMA_INCOHERENT
	if (!dma_get_attr(DMA_ATTR_SKIP_CPU_SYNC, attrs))
		pages_sync(page, offset, size);
#endif

	flush_write_buffers();
	return bus;
}

/* Map a set of buffers described by scatterlist in streaming
 * mode for DMA.  This is the scatter-gather version of the
 * above pci_map_single interface.  Here the scatter gather list
 * elements are each tagged with the appropriate dma address
 * and length.  They are obtained via sg_dma_{address,length}(SG).
 *
 * NOTE: An implementation may be able to use a smaller number of
 *       DMA address/length pairs than there are SG table elements.
 *       (for example via virtual mapping capabilities)
 *       The routine returns the number of addr/length pairs actually
 *       used, at most nents.
 *
 * Device ownership issues as mentioned above for pci_map_single are
 * the same here.
 */
static int nommu_map_sg(struct device *hwdev, struct scatterlist *sg,
			int nents, enum dma_data_direction dir,
			struct dma_attrs *attrs)
{
	struct scatterlist *s;
	int i;

#ifdef CONFIG_X86_DMA_INCOHERENT
	const struct dma_map_ops *ops = get_dma_ops(hwdev);
#endif

	WARN_ON(nents == 0 || sg[0].length == 0);

	for_each_sg(sg, s, nents, i) {
		BUG_ON(!sg_page(s));
#ifdef CONFIG_X86_DMA_INCOHERENT
		s->dma_address = ops->map_page(hwdev, sg_page(s), s->offset,
						s->length, dir, attrs);
#else
		s->dma_address = sg_phys(s);
#endif
		if (!check_addr("map_sg", hwdev, s->dma_address, s->length))
			return 0;
		s->dma_length = s->length;
	}
	flush_write_buffers();
	return nents;
}

static void nommu_sync_single_for_device(struct device *dev,
			dma_addr_t addr, size_t size,
			enum dma_data_direction dir)
{
#ifdef CONFIG_X86_DMA_INCOHERENT
	pages_sync(phys_to_page(addr), addr & ~PAGE_MASK, size);
#endif
	flush_write_buffers();
}


static void nommu_sync_sg_for_device(struct device *dev,
			struct scatterlist *sg, int nelems,
			enum dma_data_direction dir)
{
#ifdef CONFIG_X86_DMA_INCOHERENT
	const struct dma_map_ops *ops = get_dma_ops(dev);
	int i;
	struct scatterlist *s;

	for_each_sg(sg, s, nelems, i)
		ops->sync_single_for_device(dev, sg_dma_address(s),
					sg_dma_len(s), dir);
#endif
	flush_write_buffers();
}

#ifdef CONFIG_X86_DMA_INCOHERENT
void *incoherent_dma_alloc_coherent(struct device *dev, size_t size,
			dma_addr_t *dma_addr, gfp_t flag,
			struct dma_attrs *attrs)
{
	unsigned int nr_pages = PAGE_ALIGN(size) >> PAGE_SHIFT;
	void *va = dma_generic_alloc_coherent(dev, size,
					dma_addr, flag, attrs);

	if (va) {
		struct page *page = virt_to_page(va);

		set_pages_cache_type(page, nr_pages, attrs);
	}

	return va;
}

void incoherent_dma_free_coherent(struct device *dev, size_t size, void *vaddr,
				dma_addr_t dma_addr, struct dma_attrs *attrs)
{
	unsigned int nr_pages = PAGE_ALIGN(size) >> PAGE_SHIFT;
	struct page *page = virt_to_page(vaddr);

	set_pages_wb(page, nr_pages);
	dma_generic_free_coherent(dev, size, vaddr, dma_addr, attrs);
}

static void incoherent_unmap_sg(struct device *dev, struct scatterlist *sg,
	int nents, enum dma_data_direction dir, struct dma_attrs *attrs)
{
	int i;
	struct scatterlist *s;
	const struct dma_map_ops *ops = get_dma_ops(dev);

	for_each_sg(sg, s, nents, i)
		ops->unmap_page(dev, sg_dma_address(s), sg_dma_len(s),
							dir, attrs);
}

static void incoherent_unmap_page(struct device *dev, dma_addr_t addr,
			   size_t size, enum dma_data_direction dir,
			   struct dma_attrs *attrs)
{
	if (!dma_get_attr(DMA_ATTR_SKIP_CPU_SYNC, attrs))
		pages_sync(phys_to_page(addr), addr & ~PAGE_MASK, size);
}

static int incoherent_mmap(struct device *dev,
		struct vm_area_struct *vma, void *cpu_addr,
	       dma_addr_t dma_addr, size_t size, struct dma_attrs *attrs)
{
	int ret = -ENXIO;
	unsigned long user_count = (vma->vm_end - vma->vm_start) >> PAGE_SHIFT;
	unsigned long count = PAGE_ALIGN(size) >> PAGE_SHIFT;
	unsigned long pfn = page_to_pfn(virt_to_page(cpu_addr));
	unsigned long off = vma->vm_pgoff;

	if (dma_get_attr(DMA_ATTR_WRITE_COMBINE, attrs))
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	else if (dma_get_attr(DMA_ATTR_NON_CONSISTENT, attrs))
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	else {
		pr_err("%s(): DMA attribute %p is not supported\n",
				__func__, attrs->flags);
		return ret;
	}

	if (dma_mmap_from_coherent(dev, vma, cpu_addr, size, &ret))
		return ret;

	if (off < count && user_count <= (count - off))
		ret = remap_pfn_range(vma, vma->vm_start,
				      pfn + off,
				      user_count << PAGE_SHIFT,
				      vma->vm_page_prot);

	return ret;
}
#endif

#ifdef CONFIG_X86_DMA_INCOHERENT
/*
 * Drivers running on DMA incoherent systems are pretty much ported from ARM.
 * So we'd better implement as much big set of dma ops as possible.
 */
struct dma_map_ops nommu_dma_ops = {
	.alloc			= incoherent_dma_alloc_coherent,
	.free			= incoherent_dma_free_coherent,
	.mmap			= incoherent_mmap,
	.map_sg			= nommu_map_sg,
	.map_page		= nommu_map_page,
	.unmap_sg		= incoherent_unmap_sg,
	.unmap_page		= incoherent_unmap_page,
	.sync_single_for_device = nommu_sync_single_for_device,
	.sync_sg_for_device	= nommu_sync_sg_for_device,
	.sync_single_for_cpu	= nommu_sync_single_for_device,
	.sync_sg_for_cpu	= nommu_sync_sg_for_device,
	.is_phys		= 1,
};
#else
struct dma_map_ops nommu_dma_ops = {
	.alloc			= dma_generic_alloc_coherent,
	.free			= dma_generic_free_coherent,
	.map_sg			= nommu_map_sg,
	.map_page		= nommu_map_page,
	.sync_single_for_device = nommu_sync_single_for_device,
	.sync_sg_for_device	= nommu_sync_sg_for_device,
	.is_phys		= 1,
};
#endif

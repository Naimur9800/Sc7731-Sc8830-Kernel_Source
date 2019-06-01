#include "sprd_iommu_common.h"

inline void mmu_reg_write(unsigned long reg, unsigned long val, unsigned long msk)
{
	__raw_writel((__raw_readl((void *)reg) & ~msk) | val, (void *)reg);
}

unsigned long get_phys_addr(struct scatterlist *sg)
{
	/*
	 * Try sg_dma_address first so that we can
	 * map carveout regions that do not have a
	 * struct page associated with them.
	 */
	unsigned long pa = sg_dma_address(sg);
	if (pa == 0)
		pa = sg_phys(sg);
	return pa;
}

static inline void sprd_iommu_update_pgt(unsigned long pgt_base, unsigned long iova, unsigned long paddr, size_t size)
{
	unsigned long end_iova=iova+size;
	for(;iova<end_iova;iova+=SPRD_IOMMU_PAGE_SIZE,paddr+=SPRD_IOMMU_PAGE_SIZE) {
		//printk("sprd_iommu pgt_base:0x%x offset:0x%x iova:0x%x paddr:0x%x\n",pgt_base,SPRD_IOMMU_PTE_ENTRY(iova),iova,paddr);
		/*64-bit virtual addr and 32bit phy addr in arm 64-bit of TSharkL.
		  *paddr: 32-bit in TSharkL actually.
		  */
		*((unsigned int *)(pgt_base)+SPRD_IOMMU_PTE_ENTRY(iova))=(unsigned int)paddr;
		//printk("sprd_iommu pgt_addr:0x%x paddr:0x%x\n",
		//	(uint32_t)((uint32_t *)(pgt_base)+SPRD_IOMMU_PTE_ENTRY(iova)),*((uint32_t *)(pgt_base)+SPRD_IOMMU_PTE_ENTRY(iova)) );
	}
}

static inline void sprd_iommu_clear_pgt(struct sprd_iommu_init_data *data, unsigned long iova, size_t size)
{
	unsigned long end_iova=iova+size;
	unsigned long pgt_start = (unsigned long)((unsigned int *)(data->pgt_base)+SPRD_IOMMU_PTE_ENTRY(iova));
	pr_debug("sprd_iommu_clear_pgt, pgt_start:0x%lx, offset:0x%lx, iova:0x%lx, size:0x%zx, %zd, %ld\n",
	    pgt_start, SPRD_IOMMU_PTE_ENTRY(iova), iova, size, sizeof(unsigned long), pgt_start % sizeof(unsigned long));

	if (data->fault_page) {
		for(;iova<end_iova;iova+=SPRD_IOMMU_PAGE_SIZE)
			*((unsigned int *)(data->pgt_base)+SPRD_IOMMU_PTE_ENTRY(iova))=(unsigned int)(data->fault_page);
	} else {
	        if (pgt_start % sizeof(unsigned long)) {
	            *((unsigned int *)pgt_start) =  (unsigned int)-1;
	            memset((void *)((unsigned int *)pgt_start + 1), 0xFF, SPRD_IOMMU_IOVA_SIZE_TO_PGT(size-SPRD_IOMMU_PAGE_SIZE));
	        } else {
	            memset((void *)pgt_start, 0xFF, SPRD_IOMMU_IOVA_SIZE_TO_PGT(size));
	        }
	}
}

int sprd_iommu_init(struct sprd_iommu_dev *dev, struct sprd_iommu_init_data *data)
{
	int i;

	printk("%s, %s, iova_base:0x%lx, size:0x%zx, pgt_base:0x%lx, pgt_size:0x%zx,ctrl_reg:0x%lx\n",
		__FUNCTION__, dev->init_data->name, data->iova_base, data->iova_size, data->pgt_base,
		data->pgt_size, dev->init_data->ctrl_reg);

	if (data->fault_page) {
		void *p = __va(data->fault_page);
		pr_info("fault_page va: %p", p);
		for (i = 0; i < SPRD_IOMMU_PAGE_SIZE / sizeof(u32); i++)
			((u32 *)p)[i] = 0xDEADDEAD;
	}
	sprd_iommu_clear_pgt(data, data->iova_base, data->iova_size);

	dev->pgt=__get_free_pages(GFP_KERNEL,get_order(data->pgt_size));
	if(!dev->pgt) {
		pr_err("%s get_free_pages fail\n", dev->init_data->name);
		return -1;
	}
	dev->pool = gen_pool_create(12, -1);
	if (!dev->pool) {
		free_pages((unsigned long)dev->pgt, get_order(data->pgt_size));
		pr_err("%s gen_pool_create err\n",dev->init_data->name);
		return -1;
	}
	gen_pool_add(dev->pool, data->iova_base, data->iova_size, -1);
	//write start_addr to MMU_CTL register
	//TLB enable
	//MMU enable
#if 0
	printk("%s, ddr frq:%d, div2 frq:%d\n", __FUNCTION__, emc_clk_get(), dev->div2_frq);
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_RAMCLK_DIV2_EN(1),MMU_RAMCLK_DIV2_EN_MASK);
#endif
	mmu_reg_write(dev->init_data->ctrl_reg,dev->init_data->iova_base,MMU_START_MB_ADDR_MASK);
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_TLB_EN(1),MMU_TLB_EN_MASK);
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_EN(1),MMU_EN_MASK);

	return 0;
}

int sprd_iommu_exit(struct sprd_iommu_dev *dev)
{
	//TLB disable
	//MMU disable
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_RAMCLK_DIV2_EN(0),MMU_RAMCLK_DIV2_EN_MASK);
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_TLB_EN(0),MMU_TLB_EN_MASK);
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_EN(0),MMU_EN_MASK);
	gen_pool_destroy(dev->pool);
	free_pages(dev->pgt, get_order(dev->init_data->pgt_size));
	dev->pgt=0;
	return 0;
}

unsigned long sprd_iommu_iova_alloc(struct sprd_iommu_dev *dev, size_t iova_length)
{
	unsigned long iova = 0;

	if(0==iova_length) {
		pr_err("%s, %s, iova_length:0x%zx\n", __FUNCTION__, dev->init_data->name, iova_length);
		return 0;
	}

	iova =  gen_pool_alloc(dev->pool, iova_length);
	pr_debug("%s, %s, iova:0x%lx, iova_length:0x%zx\n", __FUNCTION__, dev->init_data->name, iova, iova_length);
	if (0 == iova) {
		pr_err("%s, %s, iova:0x%lx, iova_length:0x%zx\n",
			__FUNCTION__, dev->init_data->name, iova, iova_length);
	}

	return (iova);
}

void sprd_iommu_iova_free(struct sprd_iommu_dev *dev, unsigned long iova, size_t iova_length)
{
	if(((dev->init_data->iova_base + dev->init_data->iova_size) < (iova + iova_length))
		|| (dev->init_data->iova_base > iova) || (0 == iova) || (0 == iova_length)) {
		pr_err("%s, %s, iova_base:0x%lx, iova_size:0x%zx, iova:0x%lx, iova_length:0x%zx\n",
			__FUNCTION__, dev->init_data->name, dev->init_data->iova_base, dev->init_data->iova_size, iova, iova_length);
		return;
	}
	gen_pool_free(dev->pool, iova, iova_length);
	pr_debug("%s, %s, iova:0x%lx, iova_length:0x%zx\n", __FUNCTION__, dev->init_data->name, iova, iova_length);
}

int sprd_iommu_iova_map(struct sprd_iommu_dev *dev, unsigned long iova, size_t iova_length, struct ion_buffer *handle)
{
	struct sg_table *table = handle->sg_table;
	struct scatterlist *sg;
	int i=0;
	unsigned long iova_cur=0;

	if((0 == iova) || (0 == iova_length) || IS_ERR(handle)) {
		pr_err("%s, %s, iova:0x%lx, iova_length:0x%zx, handle:0x%p\n",
			__FUNCTION__, dev->init_data->name, iova, iova_length, handle);
		return -1;
	}
	iova_cur=iova;
	mutex_lock(&dev->mutex_pgt);
	//disable TLB enable in MMU ctrl register
	//enable TLB
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_TLB_EN(0),MMU_TLB_EN_MASK);
	for_each_sg(table->sgl, sg, table->nents, i) {
		sprd_iommu_update_pgt(dev->init_data->pgt_base,iova_cur,get_phys_addr(sg),PAGE_ALIGN(sg_dma_len(sg)));
		iova_cur += PAGE_ALIGN(sg_dma_len(sg));
		if (iova_cur >=(iova+iova_length))
			break;
	}
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_TLB_EN(1),MMU_TLB_EN_MASK);
	mutex_unlock(&dev->mutex_pgt);

	pr_debug("%s, %s, iova:0x%lx, iova_length:0x%zx, count:0x%x\n", __FUNCTION__, dev->init_data->name,
		iova, iova_length, handle->iomap_cnt[dev->init_data->id]);
	return 0;
}

int sprd_iommu_iova_unmap(struct sprd_iommu_dev *dev, unsigned long iova, size_t iova_length, struct ion_buffer *handle)
{
	if((0 == iova) || (0 == iova_length) || IS_ERR(handle)) {
		pr_err("%s, %s, iova:0x%lx, iova_length:0x%zx, handle:0x%p\n", __FUNCTION__, dev->init_data->name,
			iova, iova_length, handle);
		return -1;
	}

	mutex_lock(&dev->mutex_pgt);
	//disable TLB enable in MMU ctrl register
	//enable TLB
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_TLB_EN(0),MMU_TLB_EN_MASK);

	sprd_iommu_clear_pgt(dev->init_data,iova,iova_length);

	mmu_reg_write(dev->init_data->ctrl_reg,MMU_TLB_EN(1),MMU_TLB_EN_MASK);
	mutex_unlock(&dev->mutex_pgt);

	pr_debug("%s, %s, iova:0x%lx, iova_length:0x%zx, count:0x%x\n", __FUNCTION__, dev->init_data->name,
		iova, iova_length, handle->iomap_cnt[dev->init_data->id]);
	return 0;
}

int sprd_iommu_backup(struct sprd_iommu_dev *dev)
{
	mutex_lock(&dev->mutex_pgt);
	memcpy((unsigned long*)dev->pgt,(unsigned long*)dev->init_data->pgt_base,PAGE_ALIGN(dev->init_data->pgt_size));
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_RAMCLK_DIV2_EN(0),MMU_RAMCLK_DIV2_EN_MASK);
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_TLB_EN(0),MMU_TLB_EN_MASK);
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_EN(0),MMU_EN_MASK);
	mutex_unlock(&dev->mutex_pgt);
	return 0;
}

int sprd_iommu_restore(struct sprd_iommu_dev *dev)
{
	mutex_lock(&dev->mutex_pgt);
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_EN(0),MMU_EN_MASK);
	memcpy((unsigned long*)dev->init_data->pgt_base,(unsigned long*)dev->pgt,PAGE_ALIGN(dev->init_data->pgt_size));
#ifdef CONFIG_ARCH_SCX35L
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_RAMCLK_DIV2_EN(1),MMU_RAMCLK_DIV2_EN_MASK);
#endif
	mmu_reg_write(dev->init_data->ctrl_reg,dev->init_data->iova_base,MMU_START_MB_ADDR_MASK);
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_TLB_EN(1),MMU_TLB_EN_MASK);
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_EN(1),MMU_EN_MASK);
	mutex_unlock(&dev->mutex_pgt);
	return 0;
}

void sprd_iommu_reset(struct sprd_iommu_dev *dev)
{
	mutex_lock(&dev->mutex_pgt);
	mmu_reg_write(dev->init_data->ctrl_reg, MMU_EN(0), MMU_EN_MASK);
	sprd_iommu_clear_pgt(dev->init_data, dev->init_data->iova_base, dev->init_data->iova_size);
#ifdef CONFIG_ARCH_SCX35L
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_RAMCLK_DIV2_EN(1),MMU_RAMCLK_DIV2_EN_MASK);
#endif
	mmu_reg_write(dev->init_data->ctrl_reg,dev->init_data->iova_base,MMU_START_MB_ADDR_MASK);
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_TLB_EN(1),MMU_TLB_EN_MASK);
	mmu_reg_write(dev->init_data->ctrl_reg,MMU_EN(1),MMU_EN_MASK);
	mutex_unlock(&dev->mutex_pgt);
}

int sprd_iommu_disable(struct sprd_iommu_dev *dev)
{
	return 0;
}

int sprd_iommu_enable(struct sprd_iommu_dev *dev)
{
	return 0;
}

int sprd_iommu_dump(struct sprd_iommu_dev *dev, unsigned long iova, size_t iova_length)
{
	return 0;
}

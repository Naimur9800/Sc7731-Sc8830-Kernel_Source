#include "sprd_iommu_cll.h"

struct sprd_iommu_func_tbl iommu_func_tbl = {
	sprd_iommu_cll_init,
	sprd_iommu_cll_uninit,

	sprd_iommu_cll_map,
	sprd_iommu_cll_unmap,

	sprd_iommu_cll_enable,
	sprd_iommu_cll_disable,

	sprd_iommu_cll_suspend,
	sprd_iommu_cll_resume,
	NULL,

	sprd_iommu_cll_reset,
	sprd_iommu_cll_virt_to_phy,

	NULL,
};

static void sprd_iommu_debug_register_show(ulong ctrl_reg_addr)
{
	int i = 0;
	u32 reg_value = 0;
	ulong reg_addr = ctrl_reg_addr;

	IOMMU_ERR("IOMMU register print:\n");
	for (i = 0; i < 5; i++) {
		reg_addr = ctrl_reg_addr + i * 4;
		reg_value = reg_read_dword(reg_addr);
		IOMMU_ERR("offset:0x%x,value=0x%x\n", i * 4, reg_value);
	}

	for (i = 0; i < 5; i++) {
		reg_addr = ctrl_reg_addr + 0x34 + i * 4;
		reg_value = reg_read_dword(reg_addr);
		IOMMU_ERR("offset:0x%x,value=0x%x\n",
			0x34 + i * 4, reg_value);
	}

	reg_value = reg_read_dword(ctrl_reg_addr+0x50);
	IOMMU_ERR("offset:0x50,value=0x%x\n", reg_value);

	for (i = 0; i < 17; i++) {
		reg_addr = ctrl_reg_addr + 0x100 + i * 4;
		reg_value = reg_read_dword(reg_addr);
		IOMMU_ERR("offset:0x%x,value=0x%x\n",
			0x100 + i * 4, reg_value);
	}
}

u32 sprd_iommu_cll_init(struct sprd_iommu_init_param *p_init_param ,
						sprd_iommu_hdl p_iommu_hdl)
{
	struct sprd_iommu_widget *p_iommu_data = NULL;
	struct sprd_iommu_priv *p_iommu_priv = NULL;

	if (NULL == p_iommu_hdl)
		return SPRD_ERR_INVALID_PARAM;

	p_iommu_data = (struct sprd_iommu_widget *)p_iommu_hdl;

	p_iommu_priv = (struct sprd_iommu_priv *)sprd_malloc(
						sizeof(struct sprd_iommu_priv));
	sprd_memset((void *)p_iommu_priv, 0 , sizeof(struct sprd_iommu_priv));
	p_iommu_priv->base_reg_addr = p_init_param->base_reg_addr;
	p_iommu_priv->pgt_size = p_init_param->pgt_size;
	p_iommu_priv->ctrl_reg_addr = p_init_param->ctrl_reg_addr;
	p_iommu_priv->fm_base_addr = p_init_param->fm_base_addr;

	p_iommu_priv->iommu_type = p_init_param->iommu_type;
	p_iommu_priv->ram_clk_div = p_init_param->ram_clk_div;
	p_iommu_priv->reroute_enable = p_init_param->reroute_enable;
	p_iommu_priv->faultpage_enable = p_init_param->faultpage_enable;
	p_iommu_priv->faultpage_addr = p_init_param->faultpage_addr;
	p_iommu_priv->map_cnt = 0;
	p_iommu_priv->reroute_addr = p_init_param->reroute_addr;

	if (p_init_param->pagt_base_ddr > 0) {
		p_iommu_priv->pagt_base_phy_ddr = p_init_param->pagt_base_ddr;
		p_iommu_priv->pagt_ddr_size = p_init_param->pagt_ddr_size;
		p_iommu_priv->pgt_table_addr =
			(ulong)ioremap_nocache(p_iommu_priv->pagt_base_phy_ddr,
					p_iommu_priv->pagt_ddr_size);
	} else {
		p_iommu_priv->pagt_base_phy_ddr = 0;
		p_iommu_priv->pagt_ddr_size = 0;
		p_iommu_priv->pagt_base_ddr = 0;
		p_iommu_priv->pgt_table_addr =
				(ulong)sprd_aligned_malloc(
					p_iommu_priv->pgt_size,
					MMU_MAPING_PAGESIZE);
	}
	p_iommu_data->p_priv = (void *)(p_iommu_priv);
	return SPRD_NO_ERR;
}

u32 sprd_iommu_cll_uninit(sprd_iommu_hdl  p_iommu_hdl)
{
	struct sprd_iommu_widget *p_iommu_data = NULL;
	struct sprd_iommu_priv *p_iommu_priv = NULL;

	if (NULL == p_iommu_hdl)
		return SPRD_ERR_INVALID_PARAM;

	p_iommu_data = (struct sprd_iommu_widget *)p_iommu_hdl;
	if (NULL == p_iommu_data->p_priv)
		return SPRD_ERR_INITIALIZED;

	p_iommu_priv = (struct sprd_iommu_priv *)(p_iommu_data->p_priv);

	if (p_iommu_priv->pagt_base_phy_ddr > 0)
		iounmap((void __iomem *)p_iommu_priv->pgt_table_addr);
	else
		sprd_free((void *)p_iommu_priv->pgt_table_addr);

	sprd_memset(p_iommu_data->p_priv, 0 , sizeof(struct sprd_iommu_priv));
	p_iommu_data->p_priv = NULL;

	return SPRD_NO_ERR;
}


u32 sprd_iommu_cll_map(sprd_iommu_hdl  p_iommu_hdl ,
				struct sprd_iommu_map_param *p_map_param)
{
	u32 entry_index = 0;
	u32 valid_page_entrys = 0;
	ulong phy_addr = 0;
	u32 vir_base_entry = 0;
	u32 align_map_size = 0;
	u32 dma_dst_entry = 0;
	u32 dma_cpy_highaddr = 0;
	struct sprd_iommu_widget *p_iommu_data = NULL;
	struct sprd_iommu_priv *p_iommu_priv = NULL;
	struct scatterlist *sg;
	u32 sg_index = 0;
	ulong pgt_addr_phy = 0;
	ulong flush_addr = 0;
	int ret = 0;

	if ((NULL == p_iommu_hdl) || (NULL == p_map_param))
		return SPRD_ERR_INVALID_PARAM;

	p_iommu_data = (struct sprd_iommu_widget *)p_iommu_hdl;
	if (NULL == p_iommu_data->p_priv)
		return SPRD_ERR_INITIALIZED;

	p_iommu_priv = (struct sprd_iommu_priv *)(p_iommu_data->p_priv);

	if (p_iommu_priv->map_cnt == 0) {
		if (p_iommu_priv->iommu_type == SPRD_IOMMU_R4P0) {
			mmu_ram_clkdiv(p_iommu_priv->ctrl_reg_addr, 1);
			mmu_dma_engine_cfg(p_iommu_priv->ctrl_reg_addr);
		}
		mmu_enable(p_iommu_priv->ctrl_reg_addr);
		if (p_iommu_priv->iommu_type == SPRD_IOMMU_R1P0) {
			mmu_write_startmb_addr_r1p0(
					p_iommu_priv->ctrl_reg_addr,
					p_iommu_priv->fm_base_addr);
		} else {
			/*config start MB address*/
			mmu_write_startmb_addr_low(p_iommu_priv->ctrl_reg_addr,
				(u32)(p_iommu_priv->fm_base_addr
					& 0x00000000FFF00000));
#ifdef CONFIG_64BIT
			mmu_write_startmb_addr_high(p_iommu_priv->ctrl_reg_addr,
				(u32)((p_iommu_priv->fm_base_addr >> 32)
					& 0xff));
#endif
		}
		if (p_iommu_priv->reroute_enable == 1) {
			if (p_iommu_priv->iommu_type == SPRD_IOMMU_R4P0)
				mmu_rr_cfg_r4p0(p_iommu_priv->ctrl_reg_addr,
						p_iommu_priv->reroute_enable,
						p_iommu_priv->reroute_addr);

			else
				mmu_reroute_cfg(p_iommu_priv->ctrl_reg_addr,
						p_iommu_priv->reroute_enable,
						p_iommu_priv->reroute_addr);
		}

		if (p_iommu_priv->faultpage_enable
		&& p_iommu_priv->iommu_type == SPRD_IOMMU_R4P0) {
#ifdef IOMMU_R4P0_DEBUG
			mmu_pt_auto_init(p_iommu_priv->ctrl_reg_addr,
				0, 0x100 << 2,
				p_iommu_priv->faultpage_addr);
#endif
			mmu_faultpage_en(p_iommu_priv->ctrl_reg_addr);
			mmu_interrupt_en(p_iommu_priv->ctrl_reg_addr);
		}

	}

	p_iommu_priv->map_cnt++;

	vir_base_entry = (u32)VIR_TO_ENTRY_IDX(p_map_param->start_virt_addr,
					      p_iommu_priv->fm_base_addr);
	dma_dst_entry = vir_base_entry;
	mmu_tlb_disable(p_iommu_priv->ctrl_reg_addr);
	for_each_sg(p_map_param->p_sg_table->sgl, sg,
		     p_map_param->p_sg_table->nents, sg_index) {

		align_map_size = MAP_SIZE_PAGE_ALIGN_UP(sg->length);
		valid_page_entrys  = (u32)SIZE_TO_ENTRYS(align_map_size);

		for (entry_index = 0; entry_index < valid_page_entrys;
		      entry_index++) {
			phy_addr = sg_to_phys(sg) +
				(entry_index << MMU_MAPING_PAGESIZE_SHIFFT);

			if (SPRD_IOMMU_R1P0 == p_iommu_priv->iommu_type)
				mmu_write_page_entry(
					p_iommu_priv->base_reg_addr,
					vir_base_entry + entry_index, phy_addr);
			else if (p_iommu_priv->iommu_type == SPRD_IOMMU_R4P0)
				mmu_write_page_toDDR(
					p_iommu_priv->pgt_table_addr,
					vir_base_entry + entry_index,
					phy_addr >> 12);
			else
				mmu_write_page_entry(
				p_iommu_priv->base_reg_addr,
				vir_base_entry + entry_index , phy_addr >> 12);

		}
		vir_base_entry += entry_index;
	}

	if (p_iommu_priv->iommu_type == SPRD_IOMMU_R4P0) {
		if (p_iommu_priv->pagt_base_phy_ddr  == 0) {
			flush_addr = p_iommu_priv->pgt_table_addr
					+ dma_dst_entry * 4;
#ifdef CONFIG_ARM64
			__dma_flush_range((void *)flush_addr,
					(void *)(flush_addr +
					(vir_base_entry - dma_dst_entry) * 4));
#elif defined CONFIG_ARM
			dmac_flush_range((void *)flush_addr,
					(void *)(flush_addr +
					(vir_base_entry - dma_dst_entry) * 4));
#elif defined CONFIG_X86
			/*kmalloc buf, x86 cache need flush*/
			clflush_cache_range((void *)flush_addr,
				(vir_base_entry - dma_dst_entry) * 4);
#endif
			pgt_addr_phy = virt_to_phys(
					(void *)p_iommu_priv->pgt_table_addr
					+ dma_dst_entry * 4);
		} else
			pgt_addr_phy = p_iommu_priv->pagt_base_phy_ddr
						+ dma_dst_entry * 4;
#ifdef CONFIG_64BIT
		dma_cpy_highaddr = pgt_addr_phy >> 32;
#endif
		mmu_dma_copy_cfg(p_iommu_priv->ctrl_reg_addr,
			pgt_addr_phy & 0xffffffff,
			dma_cpy_highaddr, dma_dst_entry << 2,
			(vir_base_entry - dma_dst_entry - 1));

		mmu_dma_start(p_iommu_priv->ctrl_reg_addr);
		ret = mmu_dma_done_poll(p_iommu_priv->ctrl_reg_addr);
		if (ret < 0) {
			IOMMU_ERR("mmu_dma_done_poll time out!\n");
			IOMMU_ERR("pgt_addr_phy=0x%lx,dst_entry=%d\n",
				pgt_addr_phy, dma_dst_entry);
			sprd_iommu_debug_register_show(
				p_iommu_priv->ctrl_reg_addr);
			p_iommu_priv->map_cnt--;
			return SPRD_ERR_STATUS;
		}
		mmu_dma_done_clear(p_iommu_priv->ctrl_reg_addr);
	}

	mmu_tlb_enable(p_iommu_priv->ctrl_reg_addr);

	return SPRD_NO_ERR;
}

u32 sprd_iommu_cll_unmap(sprd_iommu_hdl p_iommu_hdl ,
			struct sprd_iommu_unmap_param *p_unmap_param)
{
	struct sprd_iommu_widget *p_iommu_data = NULL;
	struct sprd_iommu_priv *p_iommu_priv = NULL;

	if ((NULL == p_iommu_hdl) || (NULL == p_unmap_param))
		return SPRD_ERR_INVALID_PARAM;

	p_iommu_data = (struct sprd_iommu_widget *)p_iommu_hdl;
	if (NULL == p_iommu_data->p_priv)
		return SPRD_ERR_INITIALIZED;

	p_iommu_priv = (struct sprd_iommu_priv *)(p_iommu_data->p_priv);
	p_iommu_priv->map_cnt--;

	if (0 == p_iommu_priv->map_cnt)
		mmu_disable(p_iommu_priv->ctrl_reg_addr);
	return SPRD_NO_ERR;
}

u32 sprd_iommu_cll_enable(sprd_iommu_hdl p_iommu_hdl)
{
	struct sprd_iommu_widget *p_iommu_data = NULL;
	struct sprd_iommu_priv *p_iommu_priv = NULL;

	if (NULL == p_iommu_hdl)
		return SPRD_ERR_INVALID_PARAM;

	p_iommu_data = (struct sprd_iommu_widget *)p_iommu_hdl;
	if (NULL == p_iommu_data->p_priv)
		return SPRD_ERR_INITIALIZED;

	p_iommu_priv = (struct sprd_iommu_priv *)(p_iommu_data->p_priv);
	mmu_enable(p_iommu_priv->ctrl_reg_addr);

	return SPRD_NO_ERR;
}

u32 sprd_iommu_cll_disable(sprd_iommu_hdl  p_iommu_hdl)
{
	struct sprd_iommu_widget *p_iommu_data = NULL;
	struct sprd_iommu_priv *p_iommu_priv = NULL;

	if (NULL == p_iommu_hdl)
		return SPRD_ERR_INVALID_PARAM;

	p_iommu_data = (struct sprd_iommu_widget *)p_iommu_hdl;
	if (NULL == p_iommu_data->p_priv)
		return SPRD_ERR_INITIALIZED;

	p_iommu_priv = (struct sprd_iommu_priv *)(p_iommu_data->p_priv);
	mmu_disable(p_iommu_priv->ctrl_reg_addr);
	return SPRD_NO_ERR;
}

u32 sprd_iommu_cll_suspend(sprd_iommu_hdl p_iommu_hdl)
{
	return 0;
}

u32 sprd_iommu_cll_resume(sprd_iommu_hdl  p_iommu_hdl)
{
	return 0;
}

u32 sprd_iommu_cll_reset(sprd_iommu_hdl  p_iommu_hdl, u32 channel_num)
{
	return 0;
}

u32  sprd_iommu_cll_virt_to_phy(sprd_iommu_hdl p_iommu_hdl ,
			u64 virt_addr , u64 *dest_addr)
{
	u64 entry_index = 0;
	u64 phy_page_addr = 0;
	u64 page_in_offset = 0;
	u64 real_phy_addr = 0;

	struct sprd_iommu_widget *p_iommu_data = NULL;
	struct sprd_iommu_priv *p_iommu_priv = NULL;

	if (NULL == p_iommu_hdl)
		return SPRD_ERR_INVALID_PARAM;

	p_iommu_data = (struct sprd_iommu_widget *)p_iommu_hdl;
	if  (NULL == p_iommu_data->p_priv)
		return SPRD_ERR_INITIALIZED;

	p_iommu_priv = (struct sprd_iommu_priv *)(p_iommu_data->p_priv);

	entry_index = VIR_TO_ENTRY_IDX(virt_addr ,
				p_iommu_priv->fm_base_addr);
	phy_page_addr = mmu_read_page_entry(p_iommu_priv->base_reg_addr,
					    entry_index);
	page_in_offset = virt_addr & MMU_MAPING_PAGE_MASK;
	real_phy_addr = (phy_page_addr << MMU_MAPING_PAGESIZE_SHIFFT)
				+ page_in_offset;

	*dest_addr = real_phy_addr;
	return SPRD_NO_ERR;
}

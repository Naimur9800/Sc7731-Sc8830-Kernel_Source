#include "sprd_iommu_sample.h"

void sprd_random_map_output(u32 vir_start_addr, u32 size ,
			u64 phy_dis_conti_base_addr,
			struct sg_table *p_sg_table)
{
	u32 virt_addr_start = 0;
	u32 virt_addr_end = 0;
	u32 virt_addr_walk = 0;
	u64 phy_addr_walk = 0;
	u32 random_addr_walk = 0;
	u32 mmu_entry_index_walk = 0;
	u32 random_table[] = {3, 5, 9, 2, 1, 6, 0, 4, 7, 8};
	u32 table_size = sizeof(random_table) / sizeof(random_table[0]);
	struct scatterlist *sgl;	/* the list */
	u32 nents = 0;
	u32 sgl_index = 0;

	virt_addr_start = vir_start_addr;
	virt_addr_end = (vir_start_addr + size + 4095) & (~4095);

	mmu_entry_index_walk = 0;

	nents = (virt_addr_end - virt_addr_end)/MMU_MAPING_PAGESIZE;

	p_sg_table->sgl = sprd_malloc(nents * sizeof(struct scatterlist));
	sprd_memset(p_sg_table->sgl , 0 , nents * sizeof(struct scatterlist));
	sgl = p_sg_table->sgl;

	while (sgl_index < nents) {
		random_addr_walk = MMU_MAPING_PAGESIZE * ((mmu_entry_index_walk
			/ table_size) * 2 * table_size
		   + random_table[mmu_entry_index_walk % table_size]);

		phy_addr_walk = random_addr_walk + phy_dis_conti_base_addr;

		mmu_entry_index_walk++;
		virt_addr_walk += MMU_MAPING_PAGESIZE;

		sgl[sgl_index].dma_address = phy_addr_walk;
		sgl[sgl_index].length = MMU_MAPING_PAGESIZE;
		sgl[sgl_index].offset = 0;
		sgl[sgl_index].page_link = 0x0;

		if ((sgl_index + 1) == nents)
			sgl[sgl_index].page_link = 0x2;
	}
}


void sprd_random_map_input(u32 vir_start_addr, u32 size , u64 phy_base_addr ,
			u64 phy_new_base_addr , struct sg_table *p_sg_table)
{
	u32 virt_addr_start = 0;
	u32 virt_addr_end = 0;
	u32 virt_addr_walk = 0;
	u64 phy_addr_walk = 0;
	u32 random_addr_walk = 0;
	u32 mmu_entry_index_walk = 0;
	u32 index  = 0;
	u32 random_table[] = {7, 5, 9, 2, 1, 6, 0, 4, 3, 8};
	u32 table_size = sizeof(random_table) / sizeof(random_table[0]);
	struct scatterlist *sgl;	/* the list */
	u32 nents = 0;
	u32 sgl_index = 0;

	virt_addr_start = vir_start_addr;
	virt_addr_end = (vir_start_addr + size + 4095) & (~4095);

	mmu_entry_index_walk = 0;

	nents = (virt_addr_end - virt_addr_end)/MMU_MAPING_PAGESIZE;

	p_sg_table->sgl = sprd_malloc(nents * sizeof(struct scatterlist));
	sprd_memset(p_sg_table->sgl , 0 , nents * sizeof(struct scatterlist));
	sgl = p_sg_table->sgl;

	while (sgl_index < nents) {
		random_addr_walk = MMU_MAPING_PAGESIZE *
			((mmu_entry_index_walk/table_size) * 2 * table_size
			+ random_table[mmu_entry_index_walk % table_size]);

		phy_addr_walk = random_addr_walk + phy_new_base_addr;

		for (index = 0; index < MMU_MAPING_PAGESIZE; index += 8) {
			*(u64 *)(phy_addr_walk + index) =
			*(u64 *)(phy_base_addr + virt_addr_walk + index);
		}

		mmu_entry_index_walk++;

		virt_addr_walk += MMU_MAPING_PAGESIZE;

		sgl[sgl_index].dma_address = phy_addr_walk;
		sgl[sgl_index].length = MMU_MAPING_PAGESIZE;
		sgl[sgl_index].offset = 0;
		sgl[sgl_index].page_link = 0x0;

		if ((sgl_index + 1) == nents)
			sgl[sgl_index].page_link = 0x2;
	}

}



void sprd_iommu_map_single_disconti(void *handle,
				    struct map_single_param *p_map_param)
{
	void *p_iommu_hdl = NULL;

	struct sprd_iommu_map_param iommu_map;

	sprd_memset(&iommu_map, 0, sizeof(struct sprd_iommu_map_param));

	p_iommu_hdl = handle;


	iommu_map.channel_type = p_map_param->channel_type;
	iommu_map.channel_bypass = 0;
	iommu_map.start_virt_addr = (u64)p_map_param->virt_addr;
	iommu_map.total_map_size = p_map_param->map_len;
	iommu_map.p_sg_table = sprd_malloc(sizeof(struct sg_table));

	sprd_random_map_input(iommu_map.start_virt_addr, 0x60000 ,
			      0x97000000 , 0xb0000000 , iommu_map.p_sg_table);

	sprd_iommudrv_map(p_iommu_hdl , &iommu_map);

}

void sprd_iommu_init_hdl(void **handle)
{
	void *p_iommu_hdl = NULL;
	struct sprd_iommu_init_param iommu_init_param;

	u32 virtual_aligned = 0x100000;
	u32 page_aligned = 0x1000;

	sprd_memset(&iommu_init_param, 0 , sizeof(iommu_init_param));

	iommu_init_param.base_reg_addr = 0x63700000;
	iommu_init_param.ctrl_reg_addr = 0x63702000;

	iommu_init_param.fullpage_mode = 1;
	iommu_init_param.fm_base_addr = 0x100000000;

	iommu_init_param.iommu_type = SPRD_IOMMUPF_R2P0;
	iommu_init_param.total_ch_num = 24;
	iommu_init_param.pfch_rstart = 21;
	iommu_init_param.pfch_rend = 23;
	iommu_init_param.pfch_wstart = 0;
	iommu_init_param.pfch_wend = 21;
	iommu_init_param.fullch_write = 0;
	iommu_init_param.fullch_read = 21;

	iommu_init_param.reroute_enable = 1;
	iommu_init_param.reroute_addr =
		(ulong)sprd_aligned_malloc(0x1000 , page_aligned);
	iommu_init_param.faultpage_enable = 1;
	iommu_init_param.faultpage_addr = 0xABCD1234;
	iommu_init_param.ram_clk_div = 0;

	sprd_iommudrv_init(&iommu_init_param, (sprd_iommu_hdl *)&p_iommu_hdl);
	*handle = p_iommu_hdl;
}


void sprd_iommu_map_single(void *handle,  struct map_single_param *p_map_param)
{
	void *p_iommu_hdl = NULL;

	struct sprd_iommu_map_param iommu_map;

	sprd_memset(&iommu_map, 0, sizeof(struct sprd_iommu_map_param));

	p_iommu_hdl = handle;

	iommu_map.channel_type = p_map_param->channel_type;
	iommu_map.start_virt_addr = (u64)p_map_param->virt_addr;
	iommu_map.total_map_size = p_map_param->map_len;

	iommu_map.p_sg_table = sprd_malloc(sizeof(struct sg_table));
	iommu_map.p_sg_table->nents = 1;
	iommu_map.p_sg_table->sgl = sprd_malloc(sizeof(struct scatterlist));
	iommu_map.p_sg_table->sgl->length = p_map_param->map_len;
	iommu_map.p_sg_table->sgl->dma_address = p_map_param->phy_addr;

	sprd_iommudrv_map(p_iommu_hdl , &iommu_map);

}

void sprd_iommuex_init_hdl(void **handle)
{
	void *p_iommu_hdl = NULL;
	struct sprd_iommu_init_param iommu_init_param;

	sprd_memset(&iommu_init_param, 0, sizeof(iommu_init_param));

	iommu_init_param->master_reg_addr = ioremap_nocache(0xD1600000, 4);
	iommu_init_param->ctrl_reg_addr = ioremap_nocache(0xD1600804, 0x60);
	iommu_init_param->iommu_id = IOMMU_EXI_GSP;
	iommu_init_param->fm_base_addr = 0x10000000;
	iommu_init_param->fm_ram_size = 0; /*not use*/
	iommu_init_param.iommu_type = SPRD_IOMMUEX_ISHARKL2;
	iommu_init_param.faultpage_addr = 0xABCD1234;

	sprd_iommudrv_init(&iommu_init_param, (sprd_iommu_hdl *)&p_iommu_hdl);
	*handle = p_iommu_hdl;
}

int main(int argc, char *argv[])
{
	void *iommu_handle = NULL;
	struct map_single_param map_param;

	sprd_iommuex_init_hdl(&iommu_handle);
	sprd_memset(&map_param, 0, sizeof(struct map_single_param));
	map_param.virt_addr = 0x10000000;
	map_param.map_len = 0x1000000; /*16M*/
	map_param.phy_addr = 0x88000000;
	sprd_iommu_map_single(iommu_handle, &map_param);

	return 0;
}

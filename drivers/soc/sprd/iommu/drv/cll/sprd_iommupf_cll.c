#include "sprd_iommupf_cll.h"

struct sprd_iommu_func_tbl iommupf_func_tbl = {
	sprd_iommupf_cll_init,
	sprd_iommupf_cll_uninit,

	sprd_iommupf_cll_map,
	sprd_iommupf_cll_unmap,

	sprd_iommupf_cll_enable,
	sprd_iommupf_cll_disable,

	sprd_iommupf_cll_suspend,
	sprd_iommupf_cll_resume,
	NULL,
	sprd_iommupf_cll_reset,

	NULL,
	NULL,
};

static u32 iommupf_cll_map_fmch(sprd_iommu_hdl p_iommupf_hdl ,
				struct sprd_iommu_map_param *param ,
				u32 ch_num);
static u32 iommupf_cll_map_pfch(sprd_iommu_hdl p_iommupf_hdl ,
				struct sprd_iommu_map_param *param ,
				u32 ch_num);
static u32 iommupf_cll_alloc_ch(sprd_iommu_hdl p_iommupf_hdl ,
				enum sprd_iommu_ch_type ch_type);
static u32 iommupf_cll_target_ch(sprd_iommu_hdl p_iommupf_hdl ,
				u64 virt_addr,
				u64 length);
static u32 iommupf_cll_start_ch(sprd_iommu_hdl  p_iommupf_hdl,
				u32 ch_num);
static u32 iommupf_cll_stop_ch(sprd_iommu_hdl  p_iommupf_hdl,
				u32 ch_num);

u32 sprd_iommupf_cll_init(struct sprd_iommu_init_param *p_init_param ,
				sprd_iommu_hdl  p_iommupf_hdl)
{
	u32 index = 0;
	struct sprd_iommu_widget *p_com = NULL;
	struct sprd_iommupf_priv *priv = NULL;

	if (NULL == p_iommupf_hdl)
		return SPRD_ERR_INVALID_PARAM;

	priv = (struct sprd_iommupf_priv *)sprd_malloc(
					sizeof(struct sprd_iommupf_priv));

	p_com = (struct sprd_iommu_widget *)p_iommupf_hdl;

	sprd_memset((void *)&priv->glb_attr, 0 , sizeof(priv->glb_attr));

	priv->base_reg_addr = p_init_param->base_reg_addr;
	priv->ctrl_reg_addr = p_init_param->ctrl_reg_addr;

	priv->fullpage_mode = p_init_param->fullpage_mode;
	priv->fm_base_addr = p_init_param->fm_base_addr;
	priv->fm_ram_size = p_init_param->fm_ram_size;
	priv->iommu_type = p_init_param->iommu_type;
	priv->iommu_id = p_init_param->iommu_id;
	/*init channel number allocated list*/
	priv->total_ch_num = p_init_param->total_ch_num;
	priv->pfch_rstart = p_init_param->pfch_rstart;
	priv->pfch_rend = p_init_param->pfch_rend;
	priv->pfch_wstart = p_init_param->pfch_wstart;
	priv->pfch_wend = p_init_param->pfch_wend;
	priv->fullch_write = p_init_param->fullch_write;
	priv->fullch_read = p_init_param->fullch_read;
	priv->fm_map_cnt = 0;
	priv->toal_use_cnt = 0;

	if (p_init_param->pagt_base_ddr > 0) {
		priv->pagt_base_phy_ddr = p_init_param->pagt_base_ddr;
		priv->pagt_ddr_size = p_init_param->pagt_ddr_size;
		priv->pagt_base_ddr = (ulong)ioremap_nocache(
					priv->pagt_base_phy_ddr,
					priv->pagt_ddr_size);
		priv->pagt_current_index = 0;
	} else {
		priv->pagt_base_phy_ddr = 0;
		priv->pagt_ddr_size = 0;
		priv->pagt_base_ddr = 0;
		priv->pagt_current_index = 0;
	}

	/*init channel status*/
	for (index = 0; index < priv->total_ch_num; index++) {
		sprd_memset((void *)&priv->ch[index] , 0 ,
			    sizeof(struct sprd_iommu_channel_attr));
		priv->ch[index].ch_stat = IOMMU_CHANNEL_IDLE;
		priv->ch[index].ch_type = CH_TYPE_INVALID;
		/*kmalloc page table memory space*/
		if (priv->fullpage_mode && (index == priv->fullch_write
			|| index == priv->fullch_read)) {
			/*full mode page table in ram, no need ddr*/
			priv->ch[index].pgt_base_addr = 0;
			continue;
		} else {
			if (priv->pagt_base_phy_ddr > 0) {
				/*page table in ddr use reserved memory*/
				priv->ch[index].pgt_base_addr =
					priv->pagt_base_ddr +
					priv->pagt_current_index *
					((MMU_CH_MAX_MAP_SIZE /
					MMU_MAPING_PAGESIZE) * 4);
				priv->ch[index].pgt_base_phy_addr =
					priv->pagt_base_phy_ddr +
					priv->pagt_current_index *
					((MMU_CH_MAX_MAP_SIZE /
					MMU_MAPING_PAGESIZE) * 4);
				priv->pagt_current_index++;
			} else {
				priv->ch[index].pgt_base_addr =
					(ulong)sprd_aligned_malloc(
						(MMU_CH_MAX_MAP_SIZE /
						MMU_MAPING_PAGESIZE) * 4,
						MMU_MAPING_PAGESIZE);
				priv->ch[index].pgt_base_phy_addr = 0;
			}
		}

	}

	/*enable all interrupt*/
	priv->glb_attr.fullpage_mode = priv->fullpage_mode;
	priv->glb_attr.int_overlap_read_en = 1;
	priv->glb_attr.int_overlap_write_en = 1;
	priv->glb_attr.int_overstep_read_en = 1;
	priv->glb_attr.int_overstep_write_en = 1;
	priv->glb_attr.int_faultpage_read_en = 1;
	priv->glb_attr.int_faultpage_write_en = 1;
	priv->glb_attr.int_outofrange_read_en = 1;
	priv->glb_attr.int_outofrange_write_en = 1;
	/*init rroute and fault page control*/
	priv->glb_attr.reroute_enable = p_init_param->reroute_enable;
	priv->glb_attr.reroute_addr = p_init_param->reroute_addr;
	priv->glb_attr.faultpage_enable = p_init_param->faultpage_enable;
	priv->glb_attr.faultpage_addr = p_init_param->faultpage_addr;

	/*init each channel type*/
	if (priv->pfch_wend > priv->pfch_wstart) {
		for (index = priv->pfch_wstart; index <= priv->pfch_wend;
				index++)
			priv->ch[index].ch_type = PF_CH_WRITE;
	}

	if (priv->pfch_rend > priv->pfch_rstart) {
		for (index = priv->pfch_rstart; index <= priv->pfch_rend;
				index++)
			priv->ch[index].ch_type = PF_CH_READ;
	}

	if (priv->fullpage_mode == 1) {
		priv->ch[priv->fullch_write].ch_type = FM_CH_RW;
		priv->ch[priv->fullch_read].ch_type = FM_CH_RW;
	}

	p_com->p_priv = (void *)(priv);

	return SPRD_NO_ERR;
}

u32 sprd_iommupf_cll_uninit(sprd_iommu_hdl  p_iommupf_hdl)
{
	struct sprd_iommu_widget *p_com = NULL;
	struct sprd_iommupf_priv *priv = NULL;

	if (NULL == p_iommupf_hdl)
		return SPRD_ERR_INVALID_PARAM;

	p_com = (struct sprd_iommu_widget *)p_iommupf_hdl;

	if (NULL == p_com->p_priv)
		return SPRD_ERR_INVALID_PARAM;

	priv = (struct sprd_iommupf_priv *)(p_com->p_priv);


	if (priv->pagt_base_phy_ddr > 0)
		iounmap((void __iomem *)priv->pagt_base_ddr);

	sprd_memset((void *) p_com->p_priv, 0 ,
		    sizeof(struct sprd_iommupf_priv));

	sprd_free(p_com->p_priv);
	sprd_free(p_com);

	return SPRD_NO_ERR;
}

u32 sprd_iommupf_cll_map(sprd_iommu_hdl  p_iommupf_hdl ,
				struct sprd_iommu_map_param *param)
{
	u32 ch_num = 0;
	struct sprd_iommu_widget *p_com = NULL;
	struct sprd_iommupf_priv *priv = NULL;

	if ((NULL == p_iommupf_hdl) || (NULL == param))
		return SPRD_ERR_INVALID_PARAM;


	p_com = (struct sprd_iommu_widget *)p_iommupf_hdl;
	if (NULL == p_com->p_priv)
		return SPRD_ERR_INVALID_PARAM;


	priv = (struct sprd_iommupf_priv *)(p_com->p_priv);

	/*save config channel information to private channel structure*/
	ch_num = iommupf_cll_alloc_ch(p_iommupf_hdl , param->channel_type);
	if ((MMU_PF_CH0_ID > ch_num) || (MMU_PF_CH31_ID < ch_num))
		return SPRD_ERR_INVALID_PARAM;

	if (priv->toal_use_cnt == 0)
		sprd_iommupf_cll_enable(p_iommupf_hdl);

	priv->toal_use_cnt++;
	if (FM_CH_RW == param->channel_type)
		iommupf_cll_map_fmch(p_iommupf_hdl, param, ch_num);
	 else
		iommupf_cll_map_pfch(p_iommupf_hdl, param, ch_num);


	return SPRD_NO_ERR;
}

u32 sprd_iommupf_cll_unmap(sprd_iommu_hdl p_iommupf_hdl ,
		struct sprd_iommu_unmap_param *param)
{
	u32 ch_num = 0;
	struct sprd_iommu_widget *p_com = NULL;
	struct sprd_iommupf_priv *priv = NULL;
	u32 vir_base_entry = 0;


	if (NULL == p_iommupf_hdl)
		return SPRD_ERR_INVALID_PARAM;


	p_com = (struct sprd_iommu_widget *)p_iommupf_hdl;
	if (NULL == p_com->p_priv)
		return SPRD_ERR_INITIALIZED;


	priv = (struct sprd_iommupf_priv *)(p_com->p_priv);

	/*target which channel this virt addr + length belongs to ...*/
	ch_num = iommupf_cll_target_ch(
					p_iommupf_hdl ,
					param->start_virt_addr,
					param->total_map_size);

	if ((MMU_PF_CH0_ID > ch_num) || (MMU_PF_CH31_ID < ch_num))
		return SPRD_ERR_INVALID_PARAM;

	priv->toal_use_cnt--;
	if ((PF_CH_READ == priv->ch[ch_num].ch_type)
	|| (PF_CH_WRITE == priv->ch[ch_num].ch_type)) {
		/*disable pf channel and free channel page table*/
		iommupf_cll_stop_ch(p_iommupf_hdl, ch_num);
	} else {
		mmupf_set_ch_microtlb(priv->ctrl_reg_addr ,
				       priv->fullch_read, 0);
		mmupf_set_ch_microtlb(priv->ctrl_reg_addr ,
				       priv->fullch_write, 0);

		vir_base_entry = (u32)VIR_TO_ENTRY_IDX(
					param->start_virt_addr,
					priv->fm_base_addr);

		mmupf_set_ram_write_addr(priv->base_reg_addr ,
					 vir_base_entry * 4);

		/*fullmode channel, clear page table in TLB RAM*/
		if (1 == priv->glb_attr.faultpage_enable) {
			mmupf_write_ram_page_entry(priv->base_reg_addr,
				ADDR_TO_PAGE(priv->glb_attr.faultpage_addr));
		} else {
			mmupf_write_ram_page_entry(priv->base_reg_addr,
				0xFFFFFFFF);
		}

		mmupf_set_ch_microtlb(priv->ctrl_reg_addr,
					priv->fullch_read, 1);
		mmupf_set_ch_microtlb(priv->ctrl_reg_addr,
					priv->fullch_write, 1);
		priv->fm_map_cnt--;

		if (0 == priv->fm_map_cnt) {
			iommupf_cll_stop_ch(p_iommupf_hdl, priv->fullch_write);
			iommupf_cll_stop_ch(p_iommupf_hdl, priv->fullch_read);
		}

	}
	return SPRD_NO_ERR;
}

u32 sprd_iommupf_cll_enable(sprd_iommu_hdl p_iommupf_hdl)
{
	u32 index = 0;
	struct sprd_iommu_widget *p_com = NULL;
	struct sprd_iommupf_priv *priv = NULL;
	int roll_back_distance = 4;

	if (NULL == p_iommupf_hdl)
		return SPRD_ERR_INVALID_PARAM;


	p_com = (struct sprd_iommu_widget *)p_iommupf_hdl;
	if (NULL == p_com->p_priv)
		return SPRD_ERR_INITIALIZED;


	priv = (struct sprd_iommupf_priv *)(p_com->p_priv);

	mmupf_outofrange_read_int_enable(priv->base_reg_addr ,
			 priv->glb_attr.int_outofrange_read_en);
	mmupf_outofrange_write_int_enable(priv->base_reg_addr ,
			priv->glb_attr.int_outofrange_write_en);

	mmupf_overstep_read_int_enable(priv->base_reg_addr ,
			priv->glb_attr.int_overstep_read_en);
	mmupf_overstep_write_int_enable(priv->base_reg_addr ,
			priv->glb_attr.int_overstep_write_en);

	mmupf_faultpage_read_int_enable(priv->base_reg_addr ,
			priv->glb_attr.int_faultpage_read_en);
	mmupf_faultpage_write_int_enable(priv->base_reg_addr ,
			priv->glb_attr.int_faultpage_write_en);

	mmupf_overlap_read_int_enable(priv->base_reg_addr ,
			priv->glb_attr.int_overlap_read_en);
	mmupf_overlap_write_int_enable(priv->base_reg_addr ,
			priv->glb_attr.int_overlap_write_en);

	mmupf_faultpage_enable(priv->base_reg_addr ,
			priv->glb_attr.faultpage_enable);
	mmupf_reroute_enable(priv->base_reg_addr ,
			     priv->glb_attr.reroute_enable);

	if (1 == priv->glb_attr.reroute_enable) {
		mmupf_set_reroutepage_addr(priv->base_reg_addr ,
					priv->glb_attr.reroute_addr);
	}

	if (1 == priv->glb_attr.faultpage_enable) {
		mmupf_set_faultpage_addr(priv->base_reg_addr ,
				priv->glb_attr.faultpage_addr);
	}

	/*init each channel type*/
	if (priv->iommu_type == SPRD_IOMMUPF_R2P0
		&& (priv->iommu_id != IOMMUPF_R2_DISPC))
		roll_back_distance = 5;

	for (index = priv->pfch_rstart;
	     index <= priv->pfch_rend; index++) {
		mmupf_set_ch_rollback_distance(priv->ctrl_reg_addr ,
					       index, roll_back_distance);
	}

	for (index = priv->pfch_wstart;
	     index <= priv->pfch_wend; index++) {
		mmupf_set_ch_rollback_distance(priv->ctrl_reg_addr ,
					       index, roll_back_distance);
	}

	if (1 == priv->fullpage_mode) {
		/*set each channel rollback instance to MAX*/
		mmupf_set_ch_rollback_distance(priv->ctrl_reg_addr,
			priv->fullch_write, roll_back_distance);
		mmupf_set_ch_rollback_distance(priv->ctrl_reg_addr ,
			priv->fullch_read, roll_back_distance);
	}

	return SPRD_NO_ERR;
}

u32 sprd_iommupf_cll_disable(sprd_iommu_hdl  p_iommupf_hdl)
{
	u32 index = 0;
	struct sprd_iommu_widget *p_com = NULL;
	struct sprd_iommupf_priv *priv = NULL;

	if (NULL == p_iommupf_hdl)
		return SPRD_ERR_INVALID_PARAM;


	p_com = (struct sprd_iommu_widget *)p_iommupf_hdl;

	if (NULL == p_com->p_priv)
		return SPRD_ERR_INVALID_PARAM;


	priv = (struct sprd_iommupf_priv *)(p_com->p_priv);
	for (index = priv->pfch_rstart;
	     index <= priv->pfch_rend; index++) {
		mmupf_set_ch_rollback_distance(priv->ctrl_reg_addr ,
					       index, 0);
		mmupf_set_ch_microtlb(priv->ctrl_reg_addr, index, 1);
		mmupf_set_ch_enable(priv->ctrl_reg_addr, index , 0);
		mmupf_set_ch_bypass(priv->ctrl_reg_addr, index , 1);
	}

	for (index = priv->pfch_wstart;
	     index <= priv->pfch_wend; index++) {
		mmupf_set_ch_rollback_distance(priv->ctrl_reg_addr ,
					       index, 0);
		mmupf_set_ch_microtlb(priv->ctrl_reg_addr, index, 1);
		mmupf_set_ch_enable(priv->ctrl_reg_addr, index, 0);
		mmupf_set_ch_bypass(priv->ctrl_reg_addr, index, 1);
	}

	if (1 == priv->fullpage_mode) {
		/*set each channel rollback instance to MAX*/
		mmupf_set_ch_rollback_distance(priv->ctrl_reg_addr ,
					       priv->fullch_write, 0);
		mmupf_set_ch_rollback_distance(priv->ctrl_reg_addr ,
					       priv->fullch_read, 0);
		mmupf_set_ch_microtlb(priv->ctrl_reg_addr ,
				       priv->fullch_write , 1);
		mmupf_set_ch_enable(priv->ctrl_reg_addr ,
				     priv->fullch_write , 0);
		mmupf_set_ch_bypass(priv->ctrl_reg_addr ,
				     priv->fullch_write , 1);

		mmupf_set_ch_microtlb(priv->ctrl_reg_addr ,
				       priv->fullch_read , 1);
		mmupf_set_ch_enable(priv->ctrl_reg_addr ,
				     priv->fullch_read , 0);
		mmupf_set_ch_bypass(priv->ctrl_reg_addr ,
				     priv->fullch_read , 1);
	}

	/*disable all interrupt*/
	mmupf_outofrange_read_int_enable(priv->base_reg_addr , 0);
	mmupf_outofrange_write_int_enable(priv->base_reg_addr, 0);
	mmupf_overstep_read_int_enable(priv->base_reg_addr , 0);
	mmupf_overstep_write_int_enable(priv->base_reg_addr , 0);

	mmupf_faultpage_read_int_enable(priv->base_reg_addr , 0);
	mmupf_faultpage_write_int_enable(priv->base_reg_addr , 0);

	mmupf_overlap_read_int_enable(priv->base_reg_addr , 0);
	mmupf_overlap_write_int_enable(priv->base_reg_addr , 0);

	/*clear all interrupt status*/
	mmupf_outofrange_read_int_clear(priv->base_reg_addr , 0);
	mmupf_outofrange_write_int_clear(priv->base_reg_addr , 0);

	mmupf_overstep_read_int_clear(priv->base_reg_addr , 0);
	mmupf_overstep_write_int_clear(priv->base_reg_addr , 0);

	mmupf_faultpage_read_int_clear(priv->base_reg_addr , 0);
	mmupf_faultpage_write_int_clear(priv->base_reg_addr , 0);

	mmupf_overlap_read_int_clear(priv->base_reg_addr , 0);
	mmupf_overlap_write_int_clear(priv->base_reg_addr , 0);

	/*disable fault page and rroute*/
	mmupf_faultpage_enable(priv->base_reg_addr , 0);
	mmupf_reroute_enable(priv->base_reg_addr , 0);

	mmupf_set_reroutepage_addr(priv->base_reg_addr , 0x0);
	mmupf_set_faultpage_addr(priv->base_reg_addr , 0xFFFFFFFF);

	return SPRD_NO_ERR;
}

u32 sprd_iommupf_cll_suspend(sprd_iommu_hdl p_iommupf_hdl)
{
	struct sprd_iommu_widget *p_com = NULL;
	struct sprd_iommupf_priv *priv = NULL;

	if (NULL == p_iommupf_hdl)
		return SPRD_ERR_INVALID_PARAM;

	p_com = (struct sprd_iommu_widget *)p_iommupf_hdl;
	if (NULL == p_com->p_priv)
		return SPRD_ERR_INITIALIZED;

	priv = (struct sprd_iommupf_priv *)(p_com->p_priv);

	return SPRD_NO_ERR;
}

u32 sprd_iommupf_cll_resume(sprd_iommu_hdl  p_iommupf_hdl)
{
	struct sprd_iommu_widget *p_com = NULL;

	if (NULL == p_iommupf_hdl)
		return SPRD_ERR_INVALID_PARAM;

	p_com = (struct sprd_iommu_widget *)p_iommupf_hdl;
	if (NULL == p_com->p_priv)
		return SPRD_ERR_INITIALIZED;

	return SPRD_NO_ERR;
}

u32 sprd_iommupf_cll_reset(sprd_iommu_hdl  p_iommupf_hdl ,
				u32 ch_num)
{
	sprd_iommupf_cll_disable(p_iommupf_hdl);
	return SPRD_NO_ERR;
}

static void sprd_iommupf_set_ch_firstpage(struct sprd_iommupf_priv *priv)
{
	u32 type = priv->iommu_type;
	ulong base_addr = priv->fm_base_addr;

	if (type == SPRD_IOMMUPF_R1P0) {
		mmupf_set_ch_firstpage_lowaddr(priv->ctrl_reg_addr,
			priv->fullch_write, (u32)(base_addr & 0xfff00000));
		mmupf_set_ch_firstpage_lowaddr(priv->ctrl_reg_addr,
			priv->fullch_read, (u32)(base_addr & 0xfff00000));
#ifdef CONFIG_64BIT
		mmupf_set_ch_firstpage_highaddr(priv->ctrl_reg_addr,
			priv->fullch_write, (u32)((base_addr >> 32) & 0xfff));
		mmupf_set_ch_firstpage_highaddr(priv->ctrl_reg_addr,
			priv->fullch_read, (u32)((base_addr >> 32) & 0xfff));
#endif
	} else if (type == SPRD_IOMMUPF_R2P0) {
		mmupf_set_ch_firstpage_lowaddr(priv->ctrl_reg_addr,
			priv->fullch_write, ADDR_TO_PAGE(base_addr));
		mmupf_set_ch_firstpage_lowaddr(priv->ctrl_reg_addr,
			priv->fullch_read, ADDR_TO_PAGE(base_addr));
	}
}

static u32 iommupf_cll_map_fmch(sprd_iommu_hdl p_iommupf_hdl ,
		struct sprd_iommu_map_param *param  , u32 ch_num)
{
	u64 phy_addr = 0;
	struct sprd_iommu_widget *p_com = NULL;
	struct sprd_iommupf_priv *priv = NULL;
	u32 entry_index = 0;
	u32 valid_page_entrys = 0;
	u32 total_page_entrys = 0;
	u32 vir_base_entry = 0;
	u32 align_map_size = 0;
	struct scatterlist *sg;
	u32 sg_index = 0;

	if ((NULL == p_iommupf_hdl) || (NULL == param))
		return SPRD_ERR_INVALID_PARAM;

	p_com = (struct sprd_iommu_widget *)p_iommupf_hdl;
	if (NULL == p_com->p_priv)
		return SPRD_ERR_INITIALIZED;

	priv = (struct sprd_iommupf_priv *)(p_com->p_priv);

	if (0 ==  priv->fm_map_cnt) {
		iommupf_cll_start_ch(p_iommupf_hdl , priv->fullch_write);
		iommupf_cll_start_ch(p_iommupf_hdl , priv->fullch_read);
		/*config each fm channel base address here*/
		sprd_iommupf_set_ch_firstpage(priv);
	}

	priv->fm_map_cnt++;

	mmupf_set_ch_microtlb(priv->ctrl_reg_addr, priv->fullch_write, 0);
	mmupf_set_ch_microtlb(priv->ctrl_reg_addr, priv->fullch_read, 0);


	vir_base_entry = (u32)VIR_TO_ENTRY_IDX(param->start_virt_addr,
					priv->fm_base_addr);

	mmupf_set_ram_write_addr(priv->base_reg_addr,
		 vir_base_entry * 4);/*clear ram write pointer first*/

	/*use scatter list*/
	for_each_sg(param->p_sg_table->sgl , sg ,
		    param->p_sg_table->nents , sg_index) {
		align_map_size = MAP_SIZE_PAGE_ALIGN_UP(sg->length);
		valid_page_entrys = (u32)SIZE_TO_ENTRYS(align_map_size);

		for (entry_index = 0; entry_index < valid_page_entrys;
						entry_index++) {
			phy_addr  = sg_to_phys(sg) +
				(entry_index << MMU_MAPING_PAGESIZE_SHIFFT);
			mmupf_write_ram_page_entry(
				priv->base_reg_addr ,
				(u32)(phy_addr >> MMU_MAPING_PAGESIZE_SHIFFT));
		}
		total_page_entrys += valid_page_entrys;
	}

	mmupf_set_ch_microtlb(priv->ctrl_reg_addr, priv->fullch_write, 1);
	mmupf_set_ch_microtlb(priv->ctrl_reg_addr, priv->fullch_read, 1);
	return SPRD_NO_ERR;
}

static u32 iommupf_cll_map_pfch(sprd_iommu_hdl p_iommupf_hdl,
				struct sprd_iommu_map_param *param ,
				u32 ch_num)
{
	u32 index = 0;
	u64 phy_addr  = 0;
	u64 align_map_size = 0;
	struct sprd_iommu_widget *p_com = NULL;
	struct sprd_iommupf_priv *priv = NULL;
	u32 valid_page_entrys = 0;
	u32 total_page_entrys = 0;
	struct scatterlist *sg;
	u32 sg_index = 0;
	u64 pgt_addr_phy = 0;
	u32 sg_offset = 0;
	u32 start_entry = 0;
	u32 update_entrys = 0;
	int offset_remainder = 0;
	int mapsize_remainder = 0;

	if ((NULL == p_iommupf_hdl) || (NULL == param))
		return SPRD_ERR_INVALID_PARAM;

	p_com = (struct sprd_iommu_widget *)p_iommupf_hdl;
	if (NULL == p_com->p_priv)
		return SPRD_ERR_INITIALIZED;


	priv = (struct sprd_iommupf_priv *)(p_com->p_priv);

	if (priv->pagt_base_phy_ddr > 0)
		pgt_addr_phy = priv->ch[ch_num].pgt_base_phy_addr;
	else
		pgt_addr_phy = virt_to_phys(
				(void *)priv->ch[ch_num].pgt_base_addr);

	priv->ch[ch_num].start_virt_addr = param->start_virt_addr;
	priv->ch[ch_num].map_size = param->total_map_size;
	sg_offset = param->sg_offset;

	if (SPRD_IOMMUPF_R1P0 == priv->iommu_type) {
		/*r1po iommupf request virtual start address must be 1M align*/
		priv->ch[ch_num].firstpage_low_addr
			= priv->ch[ch_num].start_virt_addr & 0xfff00000;
#ifdef CONFIG_64BIT
		priv->ch[ch_num].firstpage_high_addr =
			((priv->ch[ch_num].start_virt_addr >> 32) & 0xfff);
#else
		priv->ch[ch_num].firstpage_high_addr = 0;
#endif
	} else if (SPRD_IOMMUPF_R2P0 == priv->iommu_type) {
		priv->ch[ch_num].firstpage_low_addr
			= ADDR_TO_PAGE(priv->ch[ch_num].start_virt_addr);
		priv->ch[ch_num].firstpage_high_addr = 0;
	} else
		return SPRD_ERR_INVALID_PARAM;


	priv->ch[ch_num].page_entrys =
		SIZE_TO_ENTRYS(MAP_SIZE_PAGE_ALIGN_UP(param->total_map_size));

	if (sg_offset > 0) {
		offset_remainder = sg_offset % MMU_MAPING_PAGESIZE;
		mapsize_remainder = param->total_map_size % MMU_MAPING_PAGESIZE;
		if (offset_remainder > mapsize_remainder)
			priv->ch[ch_num].page_entrys++;
	}

	priv->ch[ch_num].pgt_base_low_addr =
		(pgt_addr_phy & 0x00000000ffffffc0);
	priv->ch[ch_num].pgt_base_high_addr =
		((pgt_addr_phy >> 32) & 0x00000fff);

	iommupf_cll_stop_ch(p_iommupf_hdl, ch_num);

	for_each_sg(param->p_sg_table->sgl, sg,
			param->p_sg_table->nents, sg_index) {
		align_map_size = MAP_SIZE_PAGE_ALIGN_UP(sg->length);
		valid_page_entrys  = (u32)SIZE_TO_ENTRYS(align_map_size);

		if (sg_offset > align_map_size) {
			sg_offset -= align_map_size;
			continue;
		}
		/*sg_offset maybe zero, or less than one page,
		or between one page size and align_map_size area*/
		start_entry = (u32)SIZE_TO_ENTRYS(sg_offset);
		update_entrys = 0;
		sg_offset = 0;

		for (index = start_entry; index < valid_page_entrys; index++, update_entrys++) {
			if (update_entrys >= priv->ch[ch_num].page_entrys)
				break;

			phy_addr  = sg_to_phys(sg) +
					(index << MMU_MAPING_PAGESIZE_SHIFFT);
			mmupf_write_pageentry_to_ddr(
				(ulong)(priv->ch[ch_num].pgt_base_addr
					+ total_page_entrys * 4 + update_entrys * 4),
				(u32)(phy_addr >> MMU_MAPING_PAGESIZE_SHIFFT));
		}
		total_page_entrys += update_entrys;
		if (total_page_entrys >= priv->ch[ch_num].page_entrys)
			break;
	}

	if (priv->pagt_base_phy_ddr == 0) {
		/*reserved memory no need*/
#ifdef CONFIG_ARM64
		__dma_flush_range((void *)priv->ch[ch_num].pgt_base_addr,
					(void *)(priv->ch[ch_num].pgt_base_addr
					+ total_page_entrys * 4));
#elif defined CONFIG_ARM
		dmac_flush_range((void *)priv->ch[ch_num].pgt_base_addr,
					(void *)(priv->ch[ch_num].pgt_base_addr
					+ total_page_entrys * 4));
#elif defined CONFIG_X86
		clflush_cache_range((void *)priv->ch[ch_num].pgt_base_addr,
							total_page_entrys * 4);
#endif
	}

	mmupf_set_ch_pagetable_baselowaddr(priv->ctrl_reg_addr,
				ch_num,
				(priv->ch[ch_num].pgt_base_low_addr));
	mmupf_set_ch_pagetable_basehighaddr(priv->ctrl_reg_addr,
				ch_num,
				(priv->ch[ch_num].pgt_base_high_addr));

	mmupf_set_ch_pagetable_size(priv->ctrl_reg_addr,
				ch_num,
				priv->ch[ch_num].page_entrys - 1);

	mmupf_set_ch_firstpage_lowaddr(priv->ctrl_reg_addr ,
				ch_num,
				(u32)(priv->ch[ch_num].firstpage_low_addr));
	mmupf_set_ch_firstpage_highaddr(priv->ctrl_reg_addr,
				ch_num,
				(u32)(priv->ch[ch_num].firstpage_high_addr));

	iommupf_cll_start_ch(p_iommupf_hdl, ch_num);

	return SPRD_NO_ERR;
}

static u32 iommupf_cll_alloc_ch(sprd_iommu_hdl p_iommupf_hdl ,
				enum sprd_iommu_ch_type ch_type)
{
	u8 ch_num = 0xff;
	u8 alloc_success = 0;
	struct sprd_iommu_widget *p_com = NULL;
	struct sprd_iommupf_priv *priv = NULL;

	if (NULL == p_iommupf_hdl)
		return SPRD_ERR_INVALID_PARAM;

	p_com = (struct sprd_iommu_widget *)p_iommupf_hdl;
	if (NULL == p_com->p_priv)
		return SPRD_ERR_INITIALIZED;

	priv = (struct sprd_iommupf_priv *)(p_com->p_priv);

	if (CH_TYPE_INVALID == ch_type)
		return SPRD_ERR_INVALID_PARAM;

	if (FM_CH_RW == ch_type) {
		if (1 == priv->fullpage_mode) {
			alloc_success = 1;
			/*always return fullch_w or fullch_r,
			*because fullch_w/r works in same full mode
			*/
			ch_num = priv->fullch_write;
		} else
			return ch_num;
	} else {
		for (ch_num = 0; ch_num < priv->total_ch_num; ch_num++) {
			if (priv->ch[ch_num].ch_type == ch_type) {
				if (IOMMU_CHANNEL_IDLE == priv->ch[ch_num].ch_stat) {
					priv->ch[ch_num].ch_stat = IOMMU_CHANNEL_USED;
					alloc_success = 1;
					break;
				}
			}
		}
	}

	if (alloc_success == 0) {
		/*alloc channel failure, all channel busy*/
		return SPRD_ERR_RESOURCE_BUSY;
	}

	return ch_num;
}

static u32 iommupf_cll_target_ch(sprd_iommu_hdl p_iommupf_hdl ,
				 u64 virt_addr, u64 length)
{
	u8 ch_index = 0xff;
	u8 target_success = 0;
	u64 ch_start_addr = 0;
	u64 ch_end_addr = 0;
	struct sprd_iommu_widget *p_com = NULL;
	struct sprd_iommupf_priv *priv = NULL;

	if (NULL == p_iommupf_hdl)
		return SPRD_ERR_INVALID_PARAM;

	p_com = (struct sprd_iommu_widget *)p_iommupf_hdl;

	if (NULL == p_com->p_priv)
		return SPRD_ERR_INITIALIZED;

	priv = (struct sprd_iommupf_priv *)(p_com->p_priv);

	if (1 == priv->fullpage_mode) {
		ch_start_addr = priv->fm_base_addr;
		ch_end_addr = priv->fm_base_addr + priv->fm_ram_size;
		if ((virt_addr >= ch_start_addr) && ((virt_addr + length) <= ch_end_addr)) {
			/*belongs to fullmode range*/
			ch_index = priv->fullch_write;
			priv->ch[priv->fullch_write].ch_stat = IOMMU_CHANNEL_IDLE;
			priv->ch[priv->fullch_read].ch_stat = IOMMU_CHANNEL_IDLE;
			target_success = 1;
		} else {
			for (ch_index = 0; ch_index < priv->total_ch_num; ch_index++) {
				if (IOMMU_CHANNEL_USED  == priv->ch[ch_index].ch_stat) {
					ch_start_addr = priv->ch[ch_index].start_virt_addr;
					ch_end_addr = priv->ch[ch_index].start_virt_addr + priv->ch[ch_index].map_size;
					if ((virt_addr < ch_start_addr) || ((virt_addr + length) >  ch_end_addr)) {
						continue;
					} else {
						priv->ch[ch_index].ch_stat = IOMMU_CHANNEL_IDLE;
						target_success = 1;
						break;
					}
				}
			}
		}

	} else {
		for (ch_index = 0; ch_index < priv->total_ch_num; ch_index++) {
			if (IOMMU_CHANNEL_USED  == priv->ch[ch_index].ch_stat) {
				ch_start_addr = priv->ch[ch_index].start_virt_addr;
				ch_end_addr = priv->ch[ch_index].start_virt_addr + priv->ch[ch_index].map_size;
				if ((virt_addr < ch_start_addr) || ((virt_addr + length) >  ch_end_addr)) {
					continue;
				} else {
					priv->ch[ch_index].ch_stat = IOMMU_CHANNEL_IDLE;
					target_success = 1;
					break;
				}
			}
		}
	}

	if (0 == target_success)
		return SPRD_ERR_INVALID_PARAM;

	return ch_index;
}

static u32 iommupf_cll_start_ch(sprd_iommu_hdl p_iommupf_hdl ,
				u32 ch_num)
{
	struct sprd_iommu_widget *p_com = NULL;
	struct sprd_iommupf_priv *priv = NULL;

	if (NULL == p_iommupf_hdl)
		return SPRD_ERR_INVALID_PARAM;

	p_com = (struct sprd_iommu_widget *)p_iommupf_hdl;
	if (NULL == p_com->p_priv)
		return SPRD_ERR_INITIALIZED;

	priv = (struct sprd_iommupf_priv *)(p_com->p_priv);

	mmupf_set_ch_enable(priv->ctrl_reg_addr , ch_num , 1);
	mmupf_set_ch_microtlb(priv->ctrl_reg_addr , ch_num , 1);

	return SPRD_NO_ERR;
}

static u32 iommupf_cll_stop_ch(sprd_iommu_hdl  p_iommupf_hdl ,
			       u32 ch_num)
{
	struct sprd_iommu_widget *p_com = NULL;
	struct sprd_iommupf_priv *priv = NULL;
	u32 misc_stat = 0;
	u32 time_out = 0;

	if (NULL == p_iommupf_hdl)
		return SPRD_ERR_INVALID_PARAM;

	p_com = (struct sprd_iommu_widget *)p_iommupf_hdl;
	if (NULL == p_com->p_priv)
		return SPRD_ERR_INITIALIZED;

	priv = (struct sprd_iommupf_priv *)(p_com->p_priv);

	mmupf_set_ch_microtlb(priv->ctrl_reg_addr , ch_num , 0);
	mmupf_set_ch_enable(priv->ctrl_reg_addr , ch_num , 0);
	misc_stat = mmupf_get_ch_miscstat(priv->ctrl_reg_addr , ch_num);

	while (0 != (misc_stat & 0x1)) {
		/*Add time out here*/
		time_out++;
		if (time_out > 100000)
			break;
	}

	return SPRD_NO_ERR;
}

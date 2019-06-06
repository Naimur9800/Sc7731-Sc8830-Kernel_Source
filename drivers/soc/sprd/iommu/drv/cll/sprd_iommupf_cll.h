#ifndef _SPRD_IOMMUPF_CLL_H_
#define _SPRD_IOMMUPF_CLL_H_

#include "../inc/sprd_defs.h"
#include "../com/sprd_com.h"
#include "../api/sprd_iommu_api.h"
#include "../hal/sprd_iommupf_hal_register.h"


/**************************************************
* @enum SPRDIommuChStat
* @brief Define Iommu each channel status , idle or already used
*
* The structure includes a list for SPRDIommuChStat .
***************************************************/
enum sprd_iommu_ch_stat {
	IOMMU_CHANNEL_IDLE = 0x100,/*channel not init yet*/
	IOMMU_CHANNEL_USED,/*channel was inited already*/

	IOMMU_CHANNEL_INVALID,/*invalid value*/
};


/****************************************************
* @struct SPRDIommuGlbAttr
* @brief Iommu global common attribute definition, compatilbe
	with iommu and iommu_pf
*
* The structure includes variables for iommu global attributes.
******************************************************/
struct sprd_iommu_glb_attr {
	u8 fullpage_mode;/* full mode or prefetch mode, set by RTL code*/

	u8 int_overlap_read_en;
	u8 int_overlap_write_en;

	u8 int_overstep_read_en;
	u8 int_overstep_write_en;

	u8 int_faultpage_read_en;
	u8 int_faultpage_write_en;

	u8 int_outofrange_read_en;
	u8 int_outofrange_write_en;

	u8 reroute_enable;
	u8 faultpage_enable;

	u32 reroute_addr;

	u32 tlb_ram_speed;
	u32 faultpage_addr;

	/*tlb ram related*/
	u32 tlb_ram_init_start_addr;
	u32 tlb_ram_init_end_addr;
	u32 tlb_total_ram_size;
	u32 tlb_fullmode_ch0size;
	u32 tlb_prefetchch_ramsize;

	u32 ch_sel;

	u32 fsm_stat;
	u32 axi_stat;
	u32 misc_stat;
	u32 miss_cnt;

	u32 pf_param0;
	u32 pf_param1;
};

struct sprd_iommu_channel_attr {
	enum sprd_iommu_ch_type ch_type;
	enum sprd_iommu_ch_stat ch_stat;

	ulong start_virt_addr;
	u32 map_size;

	u8 channel_enable;
	u8 channel_bypass;
	u8 microtlb_enable;

	u32 firstpage_low_addr;
	u32 firstpage_high_addr;

	ulong pgt_base_addr;
	ulong pgt_base_phy_addr;
	u32 pgt_base_low_addr;
	u32 pgt_base_high_addr;

	u64 page_entrys;

	u32 rollback_distance;

	u32 fsm_stat;
	u32 axi_stat;
	u32 misc_stat;
	u32 miss_cnt;
};

struct sprd_iommupf_priv {
	enum sprd_iommu_type iommu_type;
	enum IOMMU_ID iommu_id;
	ulong base_reg_addr;/*Iommu base address in */
	ulong ctrl_reg_addr;

	/*mode definition*/
	u32 fullpage_mode;

	u32 fm_map_cnt;
	ulong fm_base_addr;
	u32 fm_ram_size;

	/*channel definition*/
	u32 total_ch_num;
	u32 toal_use_cnt;

	u8 pfch_rstart;
	u8 pfch_rend;
	u8 pfch_wstart;
	u8 pfch_wend;

	u8 fullch_read;/*channel number used for fullmode read, special*/
	u8 fullch_write;/*channel number used for fullmode write, special*/

	/*add for 9860 interlace ddr*/
	/*iommu reserved memory of pf page table*/
	unsigned long pagt_base_ddr;
	unsigned int pagt_ddr_size;
	unsigned long pagt_current_index;
	unsigned long pagt_base_phy_ddr;

	struct sprd_iommu_glb_attr glb_attr;
	struct sprd_iommu_channel_attr ch[MMU_PF_MAX_CHANNEL];
};

u32 sprd_iommupf_cll_init(struct sprd_iommu_init_param *p_init_param ,
			sprd_iommu_hdl  p_iommupf_hdl);
u32 sprd_iommupf_cll_uninit(sprd_iommu_hdl  p_iommupf_hdl);
u32 sprd_iommupf_cll_map(sprd_iommu_hdl  p_iommupf_hdl ,
			struct sprd_iommu_map_param *p_map_param);
u32 sprd_iommupf_cll_unmap(sprd_iommu_hdl p_iommupf_hdl ,
		struct sprd_iommu_unmap_param *p_unmap_param);
u32 sprd_iommupf_cll_enable(sprd_iommu_hdl p_iommupf_hdl);
u32 sprd_iommupf_cll_disable(sprd_iommu_hdl  p_iommupf_hdl);
u32 sprd_iommupf_cll_suspend(sprd_iommu_hdl p_iommupf_hdl);
u32 sprd_iommupf_cll_resume(sprd_iommu_hdl  p_iommupf_hdl);
u32 sprd_iommupf_cll_reset(sprd_iommu_hdl  p_iommupf_hdl ,
				u32 channel_num);

#endif  /* _SPRD_IOMMUPF_CLL_H_ */

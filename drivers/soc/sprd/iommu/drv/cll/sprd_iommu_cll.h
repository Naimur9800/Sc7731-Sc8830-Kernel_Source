#ifndef _SPRD_IOMMU_CLL_H_
#define _SPRD_IOMMU_CLL_H_

#include "../inc/sprd_defs.h"
#include "../com/sprd_com.h"
#include "../api/sprd_iommu_api.h"
#include "../hal/sprd_iommu_hal_register.h"

/*#define IOMMU_R4P0_DEBUG*/

struct sprd_iommu_priv {
	ulong base_reg_addr;/* Iommu base address in */
	u32 pgt_size;
	ulong ctrl_reg_addr;/*ctrl register offset with base address*/

	ulong fm_base_addr;

	u8 reroute_enable;/* Enabel rereoute function*/
	u64 reroute_addr;/* Enabel rereoute function */

	u8 faultpage_enable;/*Enabel fault page function*/
	u64 faultpage_addr;/* Enabel fault page function */
	ulong pgt_table_addr;/*paget table in ddr*/
	u8 ram_clk_div;/*Clock divisor*/

	u8 map_cnt;
	enum sprd_iommu_type iommu_type;
	/*add for 9860 interlace ddr*/
	/*iommu reserved memory of pf page table*/
	unsigned long pagt_base_ddr;
	unsigned int pagt_ddr_size;
	unsigned long pagt_base_phy_ddr;
};


u32 sprd_iommu_cll_init(struct sprd_iommu_init_param *p_init_param ,
			sprd_iommu_hdl  p_iommu_hdl);
u32 sprd_iommu_cll_uninit(sprd_iommu_hdl  p_iommu_hdl);
u32 sprd_iommu_cll_map(sprd_iommu_hdl  p_iommu_hdl ,
				struct sprd_iommu_map_param *p_map_param);
u32 sprd_iommu_cll_unmap(sprd_iommu_hdl p_iommu_hdl ,
			struct sprd_iommu_unmap_param *p_unmap_param);
u32 sprd_iommu_cll_enable(sprd_iommu_hdl p_iommu_hdl);
u32 sprd_iommu_cll_disable(sprd_iommu_hdl  p_iommu_hdl);
u32 sprd_iommu_cll_suspend(sprd_iommu_hdl p_iommu_hdl);
u32 sprd_iommu_cll_resume(sprd_iommu_hdl  p_iommu_hdl);
u32 sprd_iommu_cll_reset(sprd_iommu_hdl  p_iommu_hdl, u32 channel_num);
u32 sprd_iommu_cll_virt_to_phy(sprd_iommu_hdl p_iommu_hdl ,
			u64 virt_addr , u64 *dest_addr);

#endif  /* _SPRD_IOMMU_CLL_H_ */

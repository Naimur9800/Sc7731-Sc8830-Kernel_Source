#ifndef _SPRD_IOMMU_SAMPLE_H_
#define _SPRD_IOMMU_SAMPLE_H_

#include "../inc/sprd_defs.h"
#include "../com/sprd_com.h"
#include "../api/sprd_iommu_api.h"

struct map_single_param {
	enum sprd_iommu_ch_type channel_type;
	u8 channel_num;
	u64 virt_addr;
	u64 map_len;
	u64 phy_addr;
	u64 pagetable_addr;
};

void sprd_iommu_init_hdl(void **handle);
void sprd_iommuex_init_hdl(void **handle);
void sprd_iommu_map_single(void *handle,  struct map_single_param *p_map_param);
void sprd_iommu_map_single_disconti(void *handle,
				struct map_single_param *p_map_param);
void sprd_random_map_output(u32 vir_start_addr, u32 size,
			u64 phy_dis_conti_base_addr,
			 struct sg_table *p_sg_table);
void sprd_random_map_input(u32 vir_start_addr, u32 size,
			 u64 phy_base_addr, u64 phy_new_base_addr,
			 struct sg_table *p_sg_table);

#endif

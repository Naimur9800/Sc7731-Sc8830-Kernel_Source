#ifndef _SPRD_IOMMU_HAL_REG_H_
#define _SPRD_IOMMU_HAL_REG_H_

#include "../inc/sprd_defs.h"
#include "../com/sprd_com.h"

/*IOMMU register definition*/
#define MMU_CTRL                                   0x000
#define MMU_CFG                                     0x004
#define MMU_START_MB_ADDR_LO          0x008
#define MMU_START_MB_ADDR_HI          0x00c
#define MMU_RR_DEST_LO                     0x010
#define MMU_RR_DEST_HI                        0x014

#define MMU_PT_RAM_INIT_CTRL	0x20
#define MMU_PT_RAM_INIT_SA		0x24
#define MMU_PT_RAM_INIT_EA		0x28
#define MMU_PT_RAM_INIT_PAGE	0x2c

#define MMU_DMA_CTRL	0x30
#define MMU_DMA_SRC_LO	0x34
#define MMU_DMA_SRC_HI	0x38
#define MMU_DMA_LEN		0x3c
#define MMU_DMA_DST		0x40
#define MMU_DMA_ENGINE_CFG 0x44
#define MMU_DMA_INT_EN	0x48
#define MMU_DMA_INT_CLR	0x4c
#define MMU_DMA_INT_RAW_STS	0x50
#define MMU_DMA_INT_MASK_STS	0x54

#define MMU_OFR_ADDR_LO_R                 0x100
#define MMU_OFR_ADDR_HI_R                0x104
#define MMU_TRANS_CNT_R                   0x108
#define MMU_PT_MISS_CNT_R                 0x10c
#define MMU_TLB_MISS_CNT_R               0x110
#define MMU_OFR_ADDR_LO_W               0x114
#define MMU_OFR_ADDR_HI_W                0x118
#define MMU_TRANS_CNT_W                    0x11c
#define MMU_PT_MISS_CNT_W                0x120
#define MMU_TLB_MISS_CNT_W               0x124
#define MMU_AXI_STAT                          0x128
#define MMU_FSM_STAT                          0x12c
#define MMU_PARAM                             0x3f8
#define MMU_REV                                     0x3fc



/*-----------------------------------------------------------------------*/
/*                          FUNCTIONS HEADERS                            */
/*-----------------------------------------------------------------------*/
void mmu_enable(ulong ctrl_base_addr);
void mmu_disable(ulong ctrl_base_addr);
void mmu_tlb_enable(ulong ctrl_base_addr);
void mmu_tlb_disable(ulong ctrl_base_addr);
void mmu_write_page_entry(ulong io_base_addr , u32 entry_index , u32 value);
void mmu_write_startmb_addr_low(ulong ctrl_base_addr , u32 value);
void mmu_write_startmb_addr_high(ulong ctrl_base_addr , u32 value);
u8 mmu_get_status(ulong ctrl_base_addr);
void mmu_reroute_cfg(ulong ctrl_base_addr , u32 enable , u32 dest_addr);
void mmu_rr_cfg_r4p0(ulong ctrl_base_addr, u32 enable, u32 dest_addr);
void mmu_ram_clkdiv(ulong ctrl_base_addr , u32 clk_div);
u32 mmu_read_page_entry(ulong io_base_addr , u32 entry_index);
void mmu_write_startmb_addr_r1p0(ulong ctrl_base_addr , u32 value);
void mmu_faultpage_en(ulong ctrl_base_addr);
void mmu_write_page_toDDR(ulong pgt_base_addr, u32 entry_index, u32 value);
void mmu_dma_engine_cfg(ulong ctrl_reg);
void mmu_dma_start(ulong ctrl_base_addr);
void mmu_dma_copy_cfg(ulong ctrl_base_addr, u32 src_lo, u32 src_hi,
		u32 dst_addr, u32 dma_len);
int mmu_dma_done_poll(ulong ctrl_base_addr);
void mmu_dma_done_clear(ulong ctrl_base_addr);
int mmu_pt_auto_init(ulong ctrl_base_addr, u32 start_addr,
	u32 end_addr, u32 init_value);
void mmu_interrupt_en(ulong ctrl_base_addr);

#endif  /* _SPRD_IOMMU_HAL_REG_H_ */

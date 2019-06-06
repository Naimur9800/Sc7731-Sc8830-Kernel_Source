#ifndef _SPRD_IOMMUPF_HAL_REG_H_
#define _SPRD_IOMMUPF_HAL_REG_H_

#include "../inc/sprd_defs.h"
#include "../com/sprd_com.h"

#define FULL_MASK 0xFFFFFFFF

#define MMU_PF_MAX_CHANNEL 32

/*Global and channel 0~31 group base address*/
#define MMU_PF_GLB_BASE 0x0000   /*global control registers , 8K size*/
#define MMU_PF_CH0_BASE 0x0000  /*channel 0 control register, 256B size*/
#define MMU_PF_CH1_BASE 0x0100  /*channel 1 control register, 256B size*/
#define MMU_PF_CH2_BASE 0x0200  /*channel 2 control register, 256B size*/
#define MMU_PF_CH3_BASE 0x0300  /*channel 3 control register, 256B size*/
#define MMU_PF_CH4_BASE 0x0400  /*channel 4 control register, 256B size*/
#define MMU_PF_CH5_BASE 0x0500  /*channel 5 control register, 256B size*/
#define MMU_PF_CH6_BASE 0x0600  /*channel 6 control register, 256B size*/
#define MMU_PF_CH7_BASE 0x0700  /*channel 7 control register, 256B size*/
#define MMU_PF_CH8_BASE 0x0800  /*channel 8 control register, 256B size*/
#define MMU_PF_CH9_BASE 0x0900  /*channel 9 control register, 256B size*/
#define MMU_PF_CH10_BASE 0x0a00  /*channel 10 control register, 256B size*/
#define MMU_PF_CH11_BASE 0x0b00  /*channel 11 control register, 256B size*/
#define MMU_PF_CH12_BASE 0x0c00  /*channel 12 control register, 256B size*/
#define MMU_PF_CH13_BASE 0x0d00  /*channel 13 control register, 256B size*/
#define MMU_PF_CH14_BASE 0x0e00  /*channel 14 control register, 256B size*/
#define MMU_PF_CH15_BASE 0x0f00  /*channel 15 control register, 256B size*/
#define MMU_PF_CH16_BASE 0x1000  /*channel 16 control register, 256B size*/
#define MMU_PF_CH17_BASE 0x1100  /*channel 17 control register, 256B size*/
#define MMU_PF_CH18_BASE 0x1200  /*channel 18 control register, 256B size*/
#define MMU_PF_CH19_BASE 0x1300  /*channel 19 control register, 256B size*/
#define MMU_PF_CH20_BASE 0x1400  /*channel 20 control register, 256B size*/
#define MMU_PF_CH21_BASE 0x1500  /*channel 21 control register, 256B size*/
#define MMU_PF_CH22_BASE 0x1600  /*channel 22 control register, 256B size*/
#define MMU_PF_CH23_BASE 0x1700  /*channel 23 control register, 256B size*/
#define MMU_PF_CH24_BASE 0x1800  /*channel 24 control register, 256B size*/
#define MMU_PF_CH25_BASE 0x1900  /*channel 25 control register, 256B size*/
#define MMU_PF_CH26_BASE 0x1a00  /*channel 26 control register, 256B size*/
#define MMU_PF_CH27_BASE 0x1b00  /*channel 27 control register, 256B size*/
#define MMU_PF_CH28_BASE 0x1c00  /*channel 28 control register, 256B size*/
#define MMU_PF_CH29_BASE 0x1d00  /*channel 29 control register, 256B size*/
#define MMU_PF_CH30_BASE 0x1e00  /*channel 30 control register, 256B size*/
#define MMU_PF_CH31_BASE 0x1f00  /*channel 31 control register, 256B size*/

/*following descripe register groups details*/
/*Global control  registers map*/
#define MMU_PF_INT_EN                        0x0000
#define MMU_PF_INT_CLR                       0x0004
#define MMU_PF_INT_RAW_STS               0x0008
#define MMU_PF_INT_MASKED_STS          0x000c
#define MMU_PF_CFG                              0x0010
#define MMU_PF_RR_DST_PAGE                0x0014
#define MMU_PF_AXIM_CFG                     0x001c
#define MMU_PF_RAM_RPTR                     0x0020
#define MMU_PF_RAM_RDATA                    0x0024
#define MMU_PF_RAM_WPTR                   0x0028
#define MMU_PF_RAM_WDATA                0x002c  /*0x2c ~ 0x68 range*/
#define MMU_PF_TLB_RAM_SPEED            0x006c
#define MMU_PF_FAULT_PAGE                  0x0070
#define MMU_PF_TLB_RAM_INIT                 0x0074
#define MMU_PF_TLB_RAM_INIT_SA           0x0078
#define MMU_PF_TLB_RAM_INIT_EA           0x007c
#define MMU_PF_AXI_DLM_TIMEOUT          0x0080
#define MMU_PF_LCH_CNT_SEL                  0x0084
#define MMU_PF_OFR_ADDR_LO_R              0x1000
#define MMU_PF_OFR_ADDR_HI_R               0x1004
#define MMU_PF_OFR_ADDR_LO_W              0x1008
#define MMU_PF_OFR_ADDR_HI_W                0x100c
#define MMU_PF_OVLP_ADDR_LO_R             0x1010
#define MMU_PF_OVLP_ADDR_HI_R               0x1014
#define MMU_PF_OVLP_ADDR_LO_W               0x1018
#define MMU_PF_OVLP_ADDR_HI_W               0x101c
#define MMU_PF_OVSTP_ADDR_LO_R              0x1020
#define MMU_PF_OVSTP_ADDR_HI_R              0x1024
#define MMU_PF_OVSTP_ADDR_LO_W              0x1028
#define MMU_PF_OVSTP_ADDR_HI_W              0x102c
#define MMU_PF_FAULT_ADDR_LO_R              0x1030
#define MMU_PF_FAULT_ADDR_HI_R              0x1034
#define MMU_PF_FAULT_ADDR_LO_W              0x1038
#define MMU_PF_FAULT_ADDR_HI_W              0x103c
#define MMU_PF_GLCH_TRANS_CNT                0x1040
#define MMU_PF_GLCH_MISS_CNT                     0x1044
#define MMU_PF_MISC_STAT                            0x1048
#define MMU_PF_AXI_STAT                             0x104c
#define MMU_PF_VA_AXI_DLM_STAT                 0x1050
#define MMU_PF_PA_AXI_DLM_STAT                  0x1054
#define MMU_PF_PARAM0                               0x1ff4
#define MMU_PF_PARAM1                               0x1ff8
#define MMU_PF_REV                                      0x1ffc


/*Logical channel control register map*/
#define MMU_PF_LCH_CTRL     0x0000
#define MMU_PF_LCH_FPG_LO   0x0004
#define MMU_PF_LCH_FPG_HI   0x0008
#define MMU_PF_LCH_PTBA_LO  0x000c
#define MMU_PF_LCH_PTBA_HI  0x0010
#define MMU_PF_LCH_PT_SIZE  0x0014
#define MMU_PF_LCH_RLLBACK_DIST      0x0018
#define MMU_PF_LCH_FSM_STAT 0x00a0
#define MMU_PF_LCH_AXI_STAT 0x00a4
#define MMU_PF_LCH_MISC_STAT 0x00a8
#define MMU_PF_LCH_MISS_CNT  0x00ac


enum {
	MMU_PF_CH0_ID = 0x0,
	MMU_PF_CH1_ID = 0x1,
	MMU_PF_CH2_ID = 0x2,
	MMU_PF_CH3_ID = 0x3,
	MMU_PF_CH4_ID = 0x4,
	MMU_PF_CH5_ID = 0x5,
	MMU_PF_CH6_ID = 0x6,
	MMU_PF_CH7_ID = 0x7,
	MMU_PF_CH8_ID = 0x8,
	MMU_PF_CH9_ID = 0x9,
	MMU_PF_CH10_ID = 0xa,
	MMU_PF_CH11_ID = 0xb,
	MMU_PF_CH12_ID = 0xc,
	MMU_PF_CH13_ID = 0xd,
	MMU_PF_CH14_ID = 0xe,
	MMU_PF_CH15_ID = 0xf,
	MMU_PF_CH16_ID = 0x10,
	MMU_PF_CH17_ID = 0x11,
	MMU_PF_CH18_ID = 0x12,
	MMU_PF_CH19_ID = 0x13,
	MMU_PF_CH20_ID = 0x14,
	MMU_PF_CH21_ID = 0x15,
	MMU_PF_CH22_ID = 0x16,
	MMU_PF_CH23_ID = 0x17,
	MMU_PF_CH24_ID = 0x18,
	MMU_PF_CH25_ID = 0x19,
	MMU_PF_CH26_ID = 0x1a,
	MMU_PF_CH27_ID = 0x1b,
	MMU_PF_CH28_ID = 0x1c,
	MMU_PF_CH29_ID = 0x1d,
	MMU_PF_CH30_ID = 0x1e,
	MMU_PF_CH31_ID = 0x1f,
};

/*-----------------------------------------------*/
/*                          FUNCTIONS HEADERS                      */
/*----------------------------------------------*/
void mmupf_start_init_tlb_ram(ulong io_base_addr);
void mmupf_set_tlb_ram_start_addr(ulong io_base_addr , u32 startentry_index);
void mmupf_set_tlb_ram_end_addr(ulong io_base_addr, u32 endentry_index);
void mmupf_outofrange_read_int_enable(ulong io_base_addr , u8 enable);
void mmupf_outofrange_write_int_enable(ulong io_base_addr , u8 enable);
void mmupf_faultpage_read_int_enable(ulong io_base_addr , u8 enable);
void mmupf_faultpage_write_int_enable(ulong io_base_addr , u8 enable);
void mmupf_overstep_read_int_enable(ulong io_base_addr , u8 enable);
void mmupf_overstep_write_int_enable(ulong io_base_addr , u8 enable);
void mmupf_overlap_read_int_enable(ulong io_base_addr , u8 enable);
void mmupf_overlap_write_int_enable(ulong io_base_addr , u8 enable);
void mmupf_outofrange_read_int_clear(ulong io_base_addr , u8 enable);
void mmupf_outofrange_write_int_clear(ulong io_base_addr , u8 enable);
void mmupf_faultpage_read_int_clear(ulong io_base_addr , u8 enable);
void mmupf_faultpage_write_int_clear(ulong io_base_addr , u8 enable);
void mmupf_overstep_read_int_clear(ulong io_base_addr , u8 enable);
void mmupf_overstep_write_int_clear(ulong io_base_addr , u8 enable);
void mmupf_overlap_read_int_clear(ulong io_base_addr , u8 enable);
void mmupf_overlap_write_int_clear(ulong io_base_addr , u8 enable);
void mmupf_faultpage_enable(ulong io_base_addr , u8 enable);
void mmupf_reroute_enable(ulong io_base_addr , u8 enable);
void mmupf_set_reroutepage_addr(ulong io_base_addr , u32 reroute_addr);
void mmupf_set_faultpage_addr(ulong io_base_addr , u32 faultpage_addr);
void mmupf_set_axim_config(ulong io_base_addr);
void mmupf_set_tlb_ram_speed(ulong io_base_addr , u8 clk_div);
void mmupf_write_pageentry_to_ddr(ulong addr , u32 value);
void mmupf_set_ram_write_addr(ulong io_base_addr , u32 value);
void mmupf_write_ram_page_entry(ulong io_base_addr , u32 value);
void mmupf_set_lchcnt_sel(ulong io_base_addr , u8 value);
void mmupf_set_axidlm_cfg(ulong io_base_addr);
void mmupf_ram_clkdiv(ulong io_base_addr ,  u32 clk_div);
void mmupf_set_ch_enable(ulong io_base_addr ,  u8 ch_num, u8 ch_en);
void mmupf_set_ch_microtlb(ulong io_base_addr , u8 ch_num, u8 ch_mtlb_en);
void mmupf_set_ch_bypass(ulong io_base_addr , u8 ch_num, u8 ch_bypass_en);
void mmupf_set_ch_firstpage_lowaddr(ulong io_base_addr , u8 ch_num, u32 value);
void mmupf_set_ch_firstpage_highaddr(ulong io_base_addr , u8 ch_num, u32 value);
void mmupf_set_ch_pagetable_baselowaddr(ulong io_base_addr , u8 ch_num,
					u32 value);
void mmupf_set_ch_pagetable_basehighaddr(ulong io_base_addr , u8 ch_num,
					 u32 value);
void mmupf_set_ch_pagetable_size(ulong io_base_addr , u8 ch_num, u32 value);
void mmupf_set_ch_rollback_distance(ulong io_base_addr , u8 ch_num, u8 value);
u32 mmupf_get_ch_fsmstat(ulong io_base_addr , u8 ch_num);
u32 mmupf_get_ch_axistat(ulong io_base_addr , u8 ch_num);
u32 mmupf_get_ch_miscstat(ulong io_base_addr , u8 ch_num);
u32 mmupf_get_ch_misscnt(ulong io_base_addr , u8 ch_num);

#endif  /* _SPRD_IOMMUPF_HAL_REG_H_ */

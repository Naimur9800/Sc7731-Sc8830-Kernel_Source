#include "sprd_iommupf_hal_register.h"

void mmupf_outofrange_read_int_enable(ulong  io_base_addr , u8 enable)
{
	putbit(io_base_addr + MMU_PF_GLB_BASE + MMU_PF_INT_EN , enable , 6);
}

void mmupf_outofrange_write_int_enable(ulong  io_base_addr , u8 enable)
{
	putbit(io_base_addr + MMU_PF_GLB_BASE + MMU_PF_INT_EN , enable , 7);
}

void mmupf_faultpage_read_int_enable(ulong  io_base_addr , u8 enable)
{
	putbit(io_base_addr + MMU_PF_GLB_BASE + MMU_PF_INT_EN , enable , 4);
}


void mmupf_faultpage_write_int_enable(ulong  io_base_addr , u8 enable)
{
	putbit(io_base_addr + MMU_PF_GLB_BASE + MMU_PF_INT_EN , enable , 5);
}

void mmupf_overstep_read_int_enable(ulong io_base_addr , u8 enable)
{
	putbit(io_base_addr + MMU_PF_GLB_BASE + MMU_PF_INT_EN , enable , 2);
}

void mmupf_overstep_write_int_enable(ulong io_base_addr , u8 enable)
{
	putbit(io_base_addr + MMU_PF_GLB_BASE + MMU_PF_INT_EN , enable , 3);
}

void mmupf_overlap_read_int_enable(ulong io_base_addr , u8 enable)
{
	putbit(io_base_addr + MMU_PF_GLB_BASE + MMU_PF_INT_EN , enable , 0);
}

void mmupf_overlap_write_int_enable(ulong io_base_addr , u8 enable)
{
	putbit(io_base_addr + MMU_PF_GLB_BASE + MMU_PF_INT_EN , enable , 1);
}

void mmupf_outofrange_read_int_clear(ulong io_base_addr , u8 enable)
{
	putbit(io_base_addr + MMU_PF_GLB_BASE + MMU_PF_INT_CLR , enable , 6);
}

void mmupf_outofrange_write_int_clear(ulong io_base_addr , u8 enable)
{
	putbit(io_base_addr + MMU_PF_GLB_BASE + MMU_PF_INT_CLR , enable , 7);
}

void mmupf_faultpage_read_int_clear(ulong io_base_addr , u8 enable)
{
	putbit(io_base_addr + MMU_PF_GLB_BASE + MMU_PF_INT_CLR , enable , 4);
}

void mmupf_faultpage_write_int_clear(ulong io_base_addr , u8 enable)
{
	putbit(io_base_addr + MMU_PF_GLB_BASE + MMU_PF_INT_CLR , enable , 5);
}

void mmupf_overstep_read_int_clear(ulong io_base_addr , u8 enable)
{
	putbit(io_base_addr + MMU_PF_GLB_BASE + MMU_PF_INT_CLR , enable , 2);
}

void mmupf_overstep_write_int_clear(ulong io_base_addr , u8 enable)
{
	putbit(io_base_addr + MMU_PF_GLB_BASE + MMU_PF_INT_CLR , enable , 3);
}

void mmupf_overlap_read_int_clear(ulong io_base_addr , u8 enable)
{
	putbit(io_base_addr + MMU_PF_GLB_BASE + MMU_PF_INT_CLR , enable , 0);
}

void mmupf_overlap_write_int_clear(ulong io_base_addr , u8 enable)
{
	putbit(io_base_addr + MMU_PF_GLB_BASE + MMU_PF_INT_CLR , enable , 1);
}

void mmupf_faultpage_enable(ulong io_base_addr , u8 enable)
{
	putbit(io_base_addr + MMU_PF_GLB_BASE + MMU_PF_CFG , enable , 1);
}

void mmupf_reroute_enable(ulong io_base_addr , u8 enable)
{
	putbit(io_base_addr + MMU_PF_GLB_BASE + MMU_PF_CFG , enable , 0);
}


void mmupf_set_reroutepage_addr(ulong io_base_addr , u32 reroute_addr)
{
	ulong reg_addr = io_base_addr + MMU_PF_GLB_BASE + MMU_PF_RR_DST_PAGE;

	reg_write_dword(reg_addr , reroute_addr);
}

void mmupf_set_faultpage_addr(ulong io_base_addr , u32 faultpage_addr)
{
	ulong reg_addr = io_base_addr + MMU_PF_GLB_BASE + MMU_PF_FAULT_PAGE;

	reg_write_dword(reg_addr , faultpage_addr);
}


void mmupf_start_init_tlb_ram(ulong io_base_addr)
{
	putbit(io_base_addr + MMU_PF_GLB_BASE + MMU_PF_TLB_RAM_INIT , 1 , 0);
}

void mmupf_set_tlb_ram_start_addr(ulong io_base_addr , u32 startentry_index)
{
	ulong reg_addr = io_base_addr + MMU_PF_GLB_BASE
				+ MMU_PF_TLB_RAM_INIT_SA;

	reg_write_dword(reg_addr , startentry_index);
}

void mmupf_set_tlb_ram_end_addr(ulong io_base_addr , u32 endentry_index)
{
	ulong reg_addr = io_base_addr + MMU_PF_GLB_BASE
				+ MMU_PF_TLB_RAM_INIT_EA;

	reg_write_dword(reg_addr , endentry_index);
}

void mmupf_set_axim_config(ulong io_base_addr)
{
}


void mmupf_set_tlb_ram_speed(ulong io_base_addr , u8 clk_div)
{
	ulong reg_addr = io_base_addr + MMU_PF_GLB_BASE + MMU_PF_TLB_RAM_SPEED;

	reg_write_dword(reg_addr , clk_div);
}

void mmupf_write_pageentry_to_ddr(ulong addr , u32 value)
{
	reg_write_dword(addr , value);
}

void mmupf_set_ram_write_addr(ulong io_base_addr , u32 value)
{
	ulong reg_addr = io_base_addr + MMU_PF_GLB_BASE + MMU_PF_RAM_WPTR;

	reg_write_dword(reg_addr , value);
}

void mmupf_write_ram_page_entry(ulong io_base_addr , u32 value)
{
	ulong reg_addr = io_base_addr + MMU_PF_GLB_BASE + MMU_PF_RAM_WDATA;

	reg_write_dword(reg_addr , value);
}

void mmupf_set_lchcnt_sel(ulong io_base_addr , u8 value)
{
	ulong reg_addr = io_base_addr + MMU_PF_GLB_BASE + MMU_PF_LCH_CNT_SEL;

	reg_write_dword(reg_addr , value);
}


void mmupf_set_axidlm_cfg(ulong io_base_addr)
{

}

void mmupf_ram_clkdiv(ulong io_base_addr ,  u32 clk_div)
{

}


u32 mmupf_get_ch_base(u8 ch_num)
{
	u32 channel_base = 0;

	switch (ch_num) {
	case 0:
	channel_base = MMU_PF_CH0_BASE;
	break;
	case 1:
	channel_base = MMU_PF_CH1_BASE;
	break;
	case 2:
	channel_base = MMU_PF_CH2_BASE;
	break;
	case 3:
	channel_base = MMU_PF_CH3_BASE;
	break;
	case 4:
	channel_base = MMU_PF_CH4_BASE;
	break;
	case 5:
	channel_base = MMU_PF_CH5_BASE;
	break;
	case 6:
	channel_base = MMU_PF_CH6_BASE;
	break;
	case 7:
	channel_base = MMU_PF_CH7_BASE;
	break;
	case 8:
	channel_base = MMU_PF_CH8_BASE;
	break;
	case 9:
	channel_base = MMU_PF_CH9_BASE;
	break;
	case 10:
	channel_base = MMU_PF_CH10_BASE;
	break;
	case 11:
	channel_base = MMU_PF_CH11_BASE;
	break;
	case 12:
	channel_base = MMU_PF_CH12_BASE;
	break;
	case 13:
	channel_base = MMU_PF_CH13_BASE;
	break;
	case 14:
	channel_base = MMU_PF_CH14_BASE;
	break;
	case 15:
	channel_base = MMU_PF_CH15_BASE;
	break;
	case 16:
	channel_base = MMU_PF_CH16_BASE;
	break;
	case 17:
	channel_base = MMU_PF_CH17_BASE;
	break;
	case 18:
	channel_base = MMU_PF_CH18_BASE;
	break;
	case 19:
	channel_base = MMU_PF_CH19_BASE;
	break;
	case 20:
	channel_base = MMU_PF_CH20_BASE;
	break;
	case 21:
	channel_base = MMU_PF_CH21_BASE;
	break;
	case 22:
	channel_base = MMU_PF_CH22_BASE;
	break;
	case 23:
	channel_base = MMU_PF_CH23_BASE;
	break;
	case 24:
	channel_base = MMU_PF_CH24_BASE;
	break;
	case 25:
	channel_base = MMU_PF_CH25_BASE;
	break;
	case 26:
	channel_base = MMU_PF_CH26_BASE;
	break;
	case 27:
	channel_base = MMU_PF_CH27_BASE;
	break;
	case 28:
	channel_base = MMU_PF_CH28_BASE;
	break;
	case 29:
	channel_base = MMU_PF_CH29_BASE;
	break;
	case 30:
	channel_base = MMU_PF_CH30_BASE;
	break;
	case 31:
	channel_base = MMU_PF_CH31_BASE;
	break;
	default:
	channel_base = 0xffffffff;
	break;
	}

	return channel_base;
}

void mmupf_set_ch_enable(ulong io_base_addr ,  u8 ch_num, u8 ch_en)
{
	u32 ch_base = mmupf_get_ch_base(ch_num);
	ulong reg_addr = io_base_addr + ch_base + MMU_PF_LCH_CTRL;

	putbit(reg_addr , ch_en , 0);
}

void mmupf_set_ch_microtlb(ulong io_base_addr , u8 ch_num, u8 ch_mtlb_en)
{
	u32 ch_base = mmupf_get_ch_base(ch_num);
	ulong reg_addr = io_base_addr + ch_base + MMU_PF_LCH_CTRL;

	putbit(reg_addr , ch_mtlb_en , 1);

}

void mmupf_set_ch_bypass(ulong io_base_addr , u8 ch_num, u8 ch_bypass_en)
{
	u32 ch_base = mmupf_get_ch_base(ch_num);
	ulong reg_addr = io_base_addr + ch_base + MMU_PF_LCH_CTRL;

	putbit(reg_addr , ch_bypass_en , 2);
}

void mmupf_set_ch_firstpage_lowaddr(ulong io_base_addr , u8 ch_num, u32 value)
{
	u32 ch_base = mmupf_get_ch_base(ch_num);
	ulong reg_addr = io_base_addr + ch_base + MMU_PF_LCH_FPG_LO;

	reg_write_dword(reg_addr , value);
}

void mmupf_set_ch_firstpage_highaddr(ulong io_base_addr , u8 ch_num, u32 value)
{
	u32 ch_base = mmupf_get_ch_base(ch_num);
	ulong reg_addr = io_base_addr + ch_base + MMU_PF_LCH_FPG_HI;

	reg_write_dword(reg_addr , value);
}

void mmupf_set_ch_pagetable_baselowaddr(ulong io_base_addr,
					u8 ch_num, u32 value)
{
	u32 ch_base = mmupf_get_ch_base(ch_num);
	ulong reg_addr = io_base_addr + ch_base + MMU_PF_LCH_PTBA_LO;

	reg_write_dword(reg_addr , value);
}

void mmupf_set_ch_pagetable_basehighaddr(ulong io_base_addr ,
				  u8 ch_num, u32 value)
{
	u32 ch_base = mmupf_get_ch_base(ch_num);
	ulong reg_addr = io_base_addr + ch_base + MMU_PF_LCH_PTBA_HI;

	reg_write_dword(reg_addr , value);
}

void mmupf_set_ch_pagetable_size(ulong io_base_addr , u8 ch_num, u32 value)
{
	u32 ch_base = mmupf_get_ch_base(ch_num);
	ulong reg_addr = io_base_addr + ch_base + MMU_PF_LCH_PT_SIZE;

	reg_write_dword(reg_addr , value);
}

void mmupf_set_ch_rollback_distance(ulong io_base_addr , u8 ch_num, u8 value)
{
	u32 ch_base = mmupf_get_ch_base(ch_num);
	ulong reg_addr = io_base_addr + ch_base + MMU_PF_LCH_RLLBACK_DIST;

	reg_write_dword(reg_addr , value);
}

u32 mmupf_get_ch_fsmstat(ulong io_base_addr , u8 ch_num)
{
	u32 stat = 0;
	u32 ch_base = mmupf_get_ch_base(ch_num);
	ulong reg_addr = io_base_addr + ch_base + MMU_PF_LCH_FSM_STAT;

	stat = reg_read_dword(reg_addr);

	return stat;
}

u32 mmupf_get_ch_axistat(ulong io_base_addr , u8 ch_num)
{
	u32 stat = 0;
	u32 ch_base = mmupf_get_ch_base(ch_num);
	ulong reg_addr = io_base_addr + ch_base + MMU_PF_LCH_AXI_STAT;

	stat = reg_read_dword(reg_addr);

	return stat;
}

u32 mmupf_get_ch_miscstat(ulong io_base_addr , u8 ch_num)
{
	u32 stat = 0;
	u32 ch_base = mmupf_get_ch_base(ch_num);
	ulong reg_addr = io_base_addr + ch_base + MMU_PF_LCH_MISC_STAT;

	stat = reg_read_dword(reg_addr);

	return stat;
}

u32 mmupf_get_ch_misscnt(ulong io_base_addr , u8 ch_num)
{
	u32 stat = 0;
	u32 ch_base = mmupf_get_ch_base(ch_num);
	ulong reg_addr = io_base_addr + ch_base + MMU_PF_LCH_MISS_CNT;

	stat = reg_read_dword(reg_addr);

	return stat;
}

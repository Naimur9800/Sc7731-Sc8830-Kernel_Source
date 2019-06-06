#include "sprd_iommu_hal_register.h"
#include <linux/delay.h>

void mmu_enable(ulong ctrl_base_addr)
{
	ulong reg_addr = ctrl_base_addr + MMU_CTRL;

	putbit(reg_addr, 1, 0);/*mmu enable first*/
	putbit(reg_addr, 1, 1);/*then tlb enable */
}

void mmu_disable(ulong ctrl_base_addr)
{
	ulong reg_addr = ctrl_base_addr + MMU_CTRL;

	putbit(reg_addr, 0, 0);
	putbit(reg_addr, 0, 1);
}

void mmu_tlb_enable(ulong ctrl_base_addr)
{
	ulong reg_addr = ctrl_base_addr + MMU_CTRL;

	putbit(reg_addr, 1, 1);
}

void mmu_tlb_disable(ulong ctrl_base_addr)
{
	ulong reg_addr = ctrl_base_addr + MMU_CTRL;

	putbit(reg_addr, 0, 1);
}

void mmu_write_page_entry(ulong io_base_addr , u32 entry_index , u32 value)
{
	ulong reg_addr = io_base_addr + entry_index * 4;

	reg_write_dword(reg_addr, value);
}


u32 mmu_read_page_entry(ulong io_base_addr , u32 entry_index)
{
	ulong reg_addr = io_base_addr + entry_index * 4;
	u32 phy_addr = 0;

	phy_addr = reg_read_dword(reg_addr);
	return phy_addr;
}

void mmu_write_startmb_addr_low(ulong ctrl_base_addr , u32 value)
{
	ulong reg_addr = ctrl_base_addr + MMU_START_MB_ADDR_LO;

	reg_write_dword(reg_addr, value);
}

void mmu_write_startmb_addr_high(ulong ctrl_base_addr , u32 value)
{
	ulong reg_addr = ctrl_base_addr + MMU_START_MB_ADDR_HI;

	reg_write_dword(reg_addr, value);
}

u8 mmu_get_status(ulong ctrl_base_addr)
{
	return 0;
}

void mmu_reroute_cfg(ulong ctrl_base_addr , u32 enable , u32 dest_addr)
{
	ulong cfg_reg = ctrl_base_addr + MMU_CFG;
	ulong rr_low_reg = ctrl_base_addr + MMU_RR_DEST_LO;
	ulong rr_high_reg  = ctrl_base_addr + MMU_RR_DEST_HI;

	putbit(cfg_reg, enable, 6);
	reg_write_dword(rr_low_reg, (dest_addr & 0xfffffc00));
	reg_write_dword(rr_high_reg, (dest_addr & 0xf));
}

void mmu_rr_cfg_r4p0(ulong ctrl_base_addr, u32 enable, u32 dest_addr)
{
	ulong cfg_reg = ctrl_base_addr + MMU_CFG;
	ulong rr_low_reg = ctrl_base_addr + MMU_RR_DEST_LO;

	putbit(cfg_reg, enable, 6);
	reg_write_dword(rr_low_reg, (dest_addr >> 12));
}

void mmu_ram_clkdiv(ulong ctrl_base_addr , u32 clk_div)
{
	ulong cfg_reg = ctrl_base_addr + MMU_CFG;

	putbit(cfg_reg, clk_div, 0);
	putbit(cfg_reg, clk_div, 7);
}

void mmu_write_startmb_addr_r1p0(ulong ctrl_base_addr , u32 value)
{
	ulong reg_addr = ctrl_base_addr;

	reg_write_dword(reg_addr, value);
}

/*enable mmu detect fault page error*/
void mmu_faultpage_en(ulong ctrl_base_addr)
{
	ulong cfg_reg = ctrl_base_addr + MMU_CFG;

	putbit(cfg_reg, 1, 8);
}

void mmu_write_page_toDDR(ulong pgt_base_addr, u32 entry_index, u32 value)
{
	ulong pgt_addr = pgt_base_addr + entry_index * 4;

	reg_write_dword(pgt_addr, value);
}

void mmu_dma_engine_cfg(ulong ctrl_reg)
{
	ulong cfg_reg = ctrl_reg + MMU_DMA_ENGINE_CFG;
	u32 value = 0;

	value = reg_read_dword(cfg_reg);
	value &= 0xfffffff8;
	value |= 0x4;
	reg_write_dword(cfg_reg, value);
}

void mmu_dma_start(ulong ctrl_base_addr)
{
	ulong cfg_reg = ctrl_base_addr + MMU_DMA_CTRL;

	putbit(cfg_reg, 1, 0);
}

void mmu_dma_copy_cfg(ulong ctrl_base_addr, u32 src_lo, u32 src_hi,
	u32 dst_addr, u32 dma_len)
{
	ulong reg_src_lo_addr = ctrl_base_addr + MMU_DMA_SRC_LO;
	ulong reg_src_hi_addr = ctrl_base_addr + MMU_DMA_SRC_HI;
	ulong reg_dst_addr = ctrl_base_addr + MMU_DMA_DST;
	ulong reg_dma_len_addr = ctrl_base_addr + MMU_DMA_LEN;

	reg_write_dword(reg_src_lo_addr, src_lo);
	reg_write_dword(reg_src_hi_addr, src_hi);
	reg_write_dword(reg_dst_addr, dst_addr);
	reg_write_dword(reg_dma_len_addr, dma_len);
}

int mmu_dma_done_poll(ulong ctrl_base_addr)
{
	ulong cfg_reg = ctrl_base_addr + MMU_DMA_INT_RAW_STS;
	u32 int_val = 0;
	int ret = -1;
	int i = 0;

	do {
		i++;
		int_val = reg_read_dword(cfg_reg);
		if (int_val & 0x1)
			return 0;
		udelay(5);
	} while (i < 800);

	IOMMU_ERR("dmastatus=0x%x,dma_cpy_cnt=0x%x,i=%d\n",
		int_val, reg_read_dword(ctrl_base_addr + 0x130), i);
	return ret;
}

void mmu_dma_done_clear(ulong ctrl_base_addr)
{
	ulong cfg_reg = ctrl_base_addr + MMU_DMA_INT_CLR;

	putbit(cfg_reg, 1, 0);
}

int mmu_pt_auto_init(ulong ctrl_base_addr, u32 start_addr,
	u32 end_addr, u32 init_value)
{
	u32 int_val = 0;
	int ret = -1;
	ulong reg_ctrl = ctrl_base_addr + MMU_PT_RAM_INIT_CTRL;
	ulong reg_start_addr = ctrl_base_addr + MMU_PT_RAM_INIT_SA;
	ulong reg_end_addr = ctrl_base_addr + MMU_PT_RAM_INIT_EA;
	ulong reg_init_page = ctrl_base_addr + MMU_PT_RAM_INIT_PAGE;
	ulong time_out = 0;

	reg_write_dword(reg_start_addr, start_addr);
	reg_write_dword(reg_end_addr, end_addr);
	reg_write_dword(reg_init_page, init_value);

	/*trigger*/
	putbit(reg_ctrl, 1, 0);

	time_out = jiffies + 6;
	do {
		int_val = reg_read_dword(reg_ctrl);
	} while (!(int_val & 0x2) && time_before(jiffies, time_out));

	if (int_val & 0x2)
		ret = 0;

	return ret;
}

void mmu_interrupt_en(ulong ctrl_base_addr)
{
	ulong reg_interrupt = ctrl_base_addr + MMU_DMA_INT_EN;

	reg_write_dword(reg_interrupt, 0xFF);
}

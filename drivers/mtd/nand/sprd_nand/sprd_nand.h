#ifndef _SPRD_NAND_H_
#define _SPRD_NAND_H_

#include <linux/io.h>
#include <linux/kernel.h>

#define sprd_nand_32reg_write(reg, val) writel_relaxed((val), (reg))
#define NFC_REG_BASE				(host->ioaddr)
/* reg memory map --checked */
#define NFC_START_REG				(NFC_REG_BASE + 0x00)
#define NFC_CFG0_REG				(NFC_REG_BASE + 0x04)
#define NFC_CFG1_REG				(NFC_REG_BASE + 0x08)
#define NFC_CFG2_REG				(NFC_REG_BASE + 0x0C)
#define NFC_INT_REG				(NFC_REG_BASE + 0x10)
#define NFC_TIMING0_REG				(NFC_REG_BASE + 0x14)
#define NFC_TIMING1_REG				(NFC_REG_BASE + 0x18)
#define NFC_TIMING2_REG				(NFC_REG_BASE + 0x1C)
#define NFC_STAT_STSMCH_REG			(NFC_REG_BASE + 0x30)
#define NFC_TIMEOUT_REG				(NFC_REG_BASE + 0x34)
#define NFC_CFG3_REG				(NFC_REG_BASE + 0x38)
#define NFC_STATUS0_REG				(NFC_REG_BASE + 0x40)
#define NFC_STATUS1_REG				(NFC_REG_BASE + 0x44)
#define NFC_STATUS2_REG				(NFC_REG_BASE + 0x48)
#define NFC_STATUS3_REG				(NFC_REG_BASE + 0x4C)
#define NFC_STATUS4_REG				(NFC_REG_BASE + 0x50)
#define NFC_STATUS5_REG				(NFC_REG_BASE + 0x54)
#define NFC_STATUS6_REG				(NFC_REG_BASE + 0x58)
#define NFC_STATUS7_REG				(NFC_REG_BASE + 0x5C)
#define NFC_STATUS8_REG				(NFC_REG_BASE + 0xA4)

#define NFC_POLY0_REG				(NFC_REG_BASE + 0xB0)
#define NFC_POLY1_REG				(NFC_REG_BASE + 0xB4)
#define NFC_POLY2_REG				(NFC_REG_BASE + 0xB8)
#define NFC_POLY3_REG				(NFC_REG_BASE + 0xBC)
#define NFC_POLY4_REG				(NFC_REG_BASE + 0x60)
#define NFC_POLY5_REG				(NFC_REG_BASE + 0x64)
#define NFC_POLY6_REG				(NFC_REG_BASE + 0x68)
#define NFC_POLY7_REG				(NFC_REG_BASE + 0x6C)

#define NFC_DLL0_CFG				(NFC_REG_BASE + 0xE0)
#define NFC_DLL1_CFG				(NFC_REG_BASE + 0xE4)
#define NFC_DLL2_CFG				(NFC_REG_BASE + 0xE8)
#define NFC_DLL_REG				(NFC_REG_BASE + 0xEC)
#define NFC_CFG4_REG				(NFC_REG_BASE + 0xF8)

#define NFC_FREE_COUNT0_REG			(NFC_REG_BASE + 0x160)
#define NFC_FREE_COUNT1_REG			(NFC_REG_BASE + 0x164)
#define NFC_FREE_COUNT2_REG			(NFC_REG_BASE + 0x168)
#define NFC_FREE_COUNT3_REG			(NFC_REG_BASE + 0x16C)
#define NFC_FREE_COUNT4_REG			(NFC_REG_BASE + 0x170)

#define NFC_MAIN_ADDRH_REG			(NFC_REG_BASE + 0x200)
#define NFC_MAIN_ADDRL_REG			(NFC_REG_BASE + 0x204)
#define NFC_SPAR_ADDRH_REG			(NFC_REG_BASE + 0x208)
#define NFC_SPAR_ADDRL_REG			(NFC_REG_BASE + 0x20C)
#define NFC_STAT_ADDRH_REG			(NFC_REG_BASE + 0x210)
#define NFC_STAT_ADDRL_REG			(NFC_REG_BASE + 0x214)
#define NFC_SEED_ADDRH_REG			(NFC_REG_BASE + 0x218)
#define NFC_SEED_ADDRL_REG			(NFC_REG_BASE + 0x21C)
#define NFC_INST00_REG				(NFC_REG_BASE + 0x220)
#define NFC_INST14_REG				(NFC_REG_BASE + 0x258)
#define NFC_INST15_REG				(NFC_REG_BASE + 0x25C)
#define NFC_INST23_REG				(NFC_REG_BASE + 0x270)

#define NFC_MAX_MC_INST_NUM			(24)

/* NFC_START bit define --checked */
#define CTRL_NFC_VALID				(1 << 31)
#define CTRL_DEF1_ECC_MODE(mode)		(((mode)&0xF)<<11)
#define CTRL_NFC_CMD_CLR			(1 << 1)
#define CTRL_NFC_CMD_START			(1 << 0)

/* NFC_CFG0 bit define --checked */
#define CFG0_DEF0_MAST_ENDIAN			(0)
#define CFG0_DEF1_SECT_NUM(num)		(((num-1)&0x1F)<<24)
#define CFG0_SET_SECT_NUM_MSK			(0x1F<<24)
#define CFG0_SET_REPEAT_NUM(num)		(((num-1)&0xFF)<<16)
#define CFG0_SET_WPN				(1<<15)
#define CFG0_DEF1_BUS_WIDTH(width)		(((!!(width != BW_08))&0x1)<<14)
#define CFG0_SET_SPARE_ONLY_INFO_PROD_EN	(1<<13)
#define CFG0_DEF0_SECT_NUM_IN_INST		(0<<12)
#define CFG0_DEF0_DETECT_ALL_FF		(1<<11)
#define CFG0_SET_CS_SEL(cs)			(((cs)&0x3)<<9)
#define CFG0_CS_MAX				4
#define CFG0_CS_MSKCLR				(~(0x3<<9))
#define CFG0_SET_NFC_RW			(1 << 8)
#define CFG0_DEF1_MAIN_SPAR_APT(sectPerPage) \
					((sectPerPage == 1) ? 0 : (1 << 6))
#define CFG0_SET_SPAR_USE			(1 << 5)
#define CFG0_SET_MAIN_USE			(1 << 4)
#define CFG0_SET_ECC_EN			(1 << 2)
#define CFG0_SET_NFC_MODE(mode)		((mode)&0x3)

/* NFC_CFG1 bit define --checked */
#define CFG1_DEF1_SPAR_INFO_SIZE(size)		(((size)&0x7F)<<24)
#define CFG1_DEF1_SPAR_SIZE(size)		((((size)-1)&0x7F)<<16)
#define CFG1_INTF_TYPE(type)			(((type)&0x07)<<12)
#define CFG1_DEF_MAIN_SIZE(size)		((((size)-1))&0x7FF)
#define CFG1_temp_MIAN_SIZE_MSK		(0x7FF)

/* NFC_CFG2 bit define --checked */
#define CFG2_DEF1_SPAR_SECTOR_NUM(num)		(((num-1)&0x1F)<<24)
#define CFG2_DEF1_SPAR_INFO_POS(pos)		(((pos)&0x7F)<<16)
#define CFG2_DEF1_ECC_POSITION(pos)		((pos)&0x7F)

/* NFC_CFG3 --checked may be unused  */
#define CFG3_SEED_LOOP_CNT(cnt)			(((cnt)&0x3FF)<<16)
#define CFG3_SEED_LOOP_EN					(1<<2)
#define CFG3_DETECT_ALL_FF				(0x1<<3)
#define CFG3_POLY_4R1_EN				(0x1<<1)
#define CFG3_RANDOM_EN					0x1

/* NFC_CFG4 --checked may be unused  */
#define CFG4_PHY_DLL_CLK_2X_EN				(0x1<<2)
#define CFG4_SLICE_CLK_EN				(0x1<<1)

/* NFC_POLYNOMIALS --checked may be unused  */
#define NFC_POLYNOMIALS0				0x100d
#define NFC_POLYNOMIALS1				0x10004
#define NFC_POLYNOMIALS2				0x40013
#define NFC_POLYNOMIALS3				0x400010

/* NFC register --checked */
#define INT_STSMCH					(0x1<<3)
#define INT_WP						(0x1<<2)
#define INT_TO						(0x1<<1)
#define INT_DONE					(0x1)

#define int_clr_all()			writel_relaxed((0xF<<8), NFC_INT_REG)
#define int_dis_all()			writel_relaxed(0, NFC_INT_REG)
#define int_en(bits)			writel_relaxed((bits&0xF), NFC_INT_REG)
#define int_sts_get()			((readl_relaxed(NFC_INT_REG)>>16)&0xF)
#define int_rawsts_get()		((readl_relaxed(NFC_INT_REG)>>24)&0xF)

/* NFC_TIMING0 --checked */
#define NFC_ACS_OFFSET              (27)
#define NFC_ACS_MASK                (0x1f << NFC_ACS_OFFSET)
#define NFC_ACE_OFFSET              (22)
#define NFC_ACE_MASK                (0x1f << NFC_ACE_OFFSET)
#define NFC_RWS_OFFSET              (16)
#define NFC_RWS_MASK                (0x3f << NFC_RWS_OFFSET)
#define NFC_RWE_OFFSET              (11)
#define NFC_RWE_MASK                (0x3f << NFC_RWS_OFFSET)
#define NFC_RWH_OFFSET              (6)
#define NFC_RWH_MASK                (0x1f << NFC_RWH_OFFSET)
#define NFC_RWL_MASK                (0x3f)

#define TIME0_ACS(acs)				(((acs-1)&0x1F)<<27)
#define TIME0_ACE(ace)				(((ace-1)&0x1F)<<22)
#define TIME0_RDS(rds)				(((rds-1)&0x3F)<<16)
#define TIME0_RDE(rde)				(((rde-1)&0x1F)<<11)
#define TIME0_RWH(rwh)				(((rwh-1)&0x1F)<<6)
#define TIME0_RWL(rwl)				((rwl-1)&0x3F)	/* must >= 2  */

/* NFC_TIMING1 --checked may be unused */
#define TIME1_WTE(wte)				(((wte-1)&0x1F)<<26)
#define TIME1_WTS(wts)				(((wts-1)&0x1F)<<21)
#define TIME1_WTI(wti)				(((wti-1)&0x1F)<<16)
#define TIME1_CL0(cl0)				(((cl0-1)&0x1F)<<10)
#define TIME1_CL1(cl1)				(((cl1-1)&0x1F)<<5)
#define TIME1_RDI(rdi)				(((rdi-1)&0x1F)<<0)

/* NFC_TIMEOUT bit define --checked */
#define TIMEOUT_REPT_EN			(1<<31)
#define TIMEOUT(val)				(val&0x7FFFFFFF)
/* NFC Ram address --checked */
/* 0xFFFF'FFFF means not to move data to read buf, when read. */
#define RAM_MAIN_ADDR(addr)		((uint32_t)(addr))
/* 0xFFFF'FFFF means not to move data to read buf, when read. */
#define RAM_SPAR_ADDR(addr)		((uint32_t)(addr))
/* 0xFFFF'FFFF means not to move data to read buf, when read. */
#define RAM_STAT_ADDR(addr)		((uint32_t)(addr))
#define RAM_SEED_ADDR(addr)		((uint32_t)(addr))
/* NFC status mach --checked */
/* check IO 0_bit, stop if error */
#define MACH_ERASE					((1)|(1<<8)|(1<<16))
/* check IO 0_bit, stop if error */
#define MACH_WRITE					((1)|(1<<8)|(1<<16))
#define MACH_READ					(0)
#define DEF0_MATCH					(0)
/* NFC_STATUS0_REG bit define --checked */
#define ECC_STAT(status)			(((status)>>9)&0x3)
#define ECC_NUM(status)			((status)&0x1FF)
/* NFC Micro-Instrction Register --checked */
#define CMD_TYPE1(cmdId, param) \
			((uint16_t)(((param & 0xff) << 8) | (cmdId & 0xff)))
#define CMD_TYPE2(cmdId)			((uint16_t)(cmdId & 0xff))
#define CMD_TYPE3(cmdId, param1, param0) \
					((uint16_t)(((param1 & 0xff) << 8) | \
					((cmdId & 0xf) << 4)|(param0 & 0xf)))
#define INST_CMD(cmd)			CMD_TYPE1(0xCD, (cmd))
#define INST_ADDR(addr, step)	CMD_TYPE3(0x0A, (addr), (step))
#define INST_WRB0()			CMD_TYPE2(0xB0)
#define INST_WRB1(cycle)		CMD_TYPE1(0xB1, (cycle))
#define INST_MRDT()			CMD_TYPE2(0xD0)
#define INST_MWDT()			CMD_TYPE2(0xD1)
#define INST_SRDT()			CMD_TYPE2(0xD2)
#define INST_SWDT()			CMD_TYPE2(0xD3)
#define INST_IDST(num)			CMD_TYPE1(0xDD, (num-1))
/* 0 or 1, priority > _CFG0_CS_SEL */
#define INST_CSEN(en)			CMD_TYPE1(0xCE, (en))
#define INST_INOP(num)			CMD_TYPE1(0xF0, (num-1))
#define INST_DONE()			CMD_TYPE2(0xFF)
/* Other define */
#define NFC_MAX_CHIP				4
#define NFC_TIMEOUT_VAL				3000000	/* usecs */
#define NFC_RESET_TIMEOUT_VAL       500000  /* usecs */

#define NFC_DEFAULT_TIMING		((7)  | (6 << NFC_RWH_OFFSET) | \
	(7 << NFC_RWE_OFFSET) | (7 << NFC_RWS_OFFSET) | \
	(7 << NFC_ACE_OFFSET) | (7 << NFC_ACS_OFFSET))
#define GET_CYCLE(ns) \
	((uint32_t)(((uint32_t)(((host->frequence/1000000)*ns)/1000))+1))
#define NFC_MBUF_SIZE				4096
#define SEED_TBL_SIZE				64
#define SEED_BUF_SIZE				68
#define SPL_MAX_SIZE				(64 << 10)

#endif

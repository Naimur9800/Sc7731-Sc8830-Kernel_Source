/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
 *
 * host software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * host program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/flashchip.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

#include "sprd_nand.h"
#include "nfc_base.h"
#include "sprd_nand_param.h"

struct sprd_nand_param {
	uint8_t id[NFC_MAX_ID_LEN];
	uint32_t nblkcnt;
	uint32_t npage_per_blk;
	uint32_t nsect_per_page;
	uint32_t nsect_size;
	uint32_t nspare_size;

	uint32_t page_size; /* no contain spare */
	uint32_t main_size; /* page_size */
	uint32_t spare_size;

	uint32_t badflag_pos;
	uint32_t badflag_len;
	uint32_t ecc_mode;
	uint32_t ecc_pos;
	uint32_t ecc_size;
	uint32_t info_pos;
	uint32_t info_size;

	uint8_t nbus_width;
	uint8_t ncycle;

	/* ACS */
	uint32_t t_als;
	uint32_t t_cls;
	/* ACE */
	uint32_t t_clh;
	uint32_t t_alh;
	/* RWS */
	uint32_t t_rr;
	uint32_t t_adl;
	/* RWE */

	/* RWH & RWL */
	uint32_t t_wh;		/* we high when write */
	uint32_t t_wp;		/* we low  when write */
	uint32_t t_reh;		/* re high when read */
	uint32_t t_rp;		/* re low when read */
};

struct nand_inst {
	uint8_t *program_name;
	uint32_t cnt;
	uint32_t step_tag;
	uint32_t int_bits;
	uint32_t inst[NFC_MAX_MC_INST_NUM];
};

struct sprd_nand_host {
	/* resource0: interrupt id & clock name &  register bass address */
	uint32_t irq;
	uint32_t frequence;
	void __iomem *ioaddr;
	struct clk *clk_ahb_enable_gate;
	struct clk *clk_nand;
	struct clk *clk_ecc;
	struct clk *clk_parent_sdr;
	struct clk *clk_parent_ddr;

	/* resource1: nand param & mtd ecclayout & risk threshold */
	struct sprd_nand_param param;
	uint32_t eccmode_reg;
	uint32_t risk_threshold;
	uint32_t cs[CFG0_CS_MAX];
	uint32_t csnum;

	/* uint32_t csDis; */
	uint32_t sect_perpage_shift;
	uint32_t sectshift;
	uint32_t sectmsk;
	uint32_t pageshift;
	uint32_t blkshift;
	uint32_t bufshift;
	uint32_t badblkcnt;
	/* resource3: local DMA buffer */
	uint8_t *mbuf_p;
	uint8_t *mbuf_v;
	uint8_t *sbuf_p;
	uint8_t *sbuf_v;
	uint8_t *stsbuf_p;
	uint8_t *stsbuf_v;
	uint32_t *seedbuf_p;
	uint32_t *seedbuf_v;
	/*
	 * resource4: register base value. some value is const
	 * while operate on nand flash, we store it to local valuable.
	 * Time & cfg & status mach. It is different with operation R W E.
	 */
	uint32_t nfc_time0_r;
	uint32_t nfc_time0_w;
	uint32_t nfc_time0_e;

	uint32_t nfc_start;
	uint32_t nfc_cfg0;
	uint32_t nfc_cfg1;
	uint32_t nfc_cfg2;
	uint32_t nfc_cfg3;
	uint32_t nfc_cfg4;

	uint32_t nfc_time0_val;
	uint32_t nfc_cfg0_val;
	uint32_t nfc_sts_mach_val;

	struct nand_inst inst_read_main_spare;
	struct nand_inst inst_read_main_raw;
	struct nand_inst inst_read_spare_raw;
	struct nand_inst inst_write_main_spare;
	struct nand_inst inst_write_main_raw;
	struct nand_inst inst_write_spare_raw;
	struct nand_inst inst_erase;
};

#define GETCS(page) \
host->cs[((page) >> ((nfc_base->chip_shift) - (nfc_base->pageshift)))]

#define sprd_nand_cmd_init(x, name, int_bits_p)	{\
	memset(&x, 0, sizeof(struct nand_inst));\
	x.program_name = name;\
	x.int_bits = (int_bits_p);\
}
#define sprd_nand_cmd_add(x, inst_p)	{\
	x.inst[x.cnt] = (inst_p);\
	x.cnt++;\
}
#define sprd_nand_cmd_tag(x)	{x.step_tag = x.cnt; }

#define sprd_nand_cmd_change(x, pg)	{\
	x->inst[x->step_tag] = INST_ADDR(((pg)&0xFF), 1);\
	x->inst[x->step_tag+1] = INST_ADDR((((pg)>>8)&0xFF), 0);\
	if (host->param.ncycle == 5)	{\
		x->inst[x->step_tag+2] = INST_ADDR((((pg)>>16)&0xFF), 0);\
	} \
}

struct nand_ecc_stats {
	uint16_t ecc_stats[16];
	uint32_t layout4_ecc_stats;
	uint32_t freecount[5];
};

static struct nand_inst inst_reset = {
	"_inst_reset",
	3, 0, INT_TO | INT_DONE, {
		INST_CMD(0xFF),
		INST_WRB0(),
		INST_DONE()
	}
};

static struct nand_inst inst_readid = {
	"_inst_readid",
	5, 0, INT_TO | INT_DONE, {
		INST_CMD(0x90),
		INST_ADDR(0, 0),
		INST_INOP(10),
		INST_IDST(0x08),
		INST_DONE()
	}
};

static struct sprd_nand_timing default_timing = {10, 25, 15};

static const uint32_t seedtbl[64] = {
	0x056c,		0x1bc77,	0x5d356,	0x1f645d,
	0x0fbc,		0x0090c,	0x7f880,	0x3d9e86,
	0x1717,		0x1e1ad,	0x6db67,	0x7d7ea0,
	0x0a52,		0x0d564,	0x6fbac,	0x6823dd,
	0x07cf,		0x1cb3b,	0x37cd1,	0x5c91f0,
	0x064e,		0x167a7,	0x0f1d2,	0x506be8,
	0x098c,		0x1bd54,	0x2c2af,	0x4b5fb7,
	0x1399,		0x11690,	0x1d310,	0x27e53b,
	0x1246,		0x14794,	0x0f34f,	0x347bc4,
	0x0150,		0x00787,	0x73450,	0x3d8927,
	0x11f1,		0x17bad,	0x46eaa,	0x5403f5,
	0x1026,		0x173ab,	0x79634,	0x01b987,
	0x1c45,		0x08b63,	0x42924,	0x4bf708,
	0x012a,		0x03a3a,	0x435d5,	0x1a7baa,
	0x0849,		0x1cb9b,	0x28350,	0x1e8309,
	0x1d4c,		0x0af6e,	0x0949e,	0x00193a,
};

static int8_t bit_num8[256] = {
	8, 7, 7, 6, 7, 6, 6, 5, 7, 6, 6, 5, 6, 5, 5, 4,
	7, 6, 6, 5, 6, 5, 5, 4, 6, 5, 5, 4, 5, 4, 4, 3,
	7, 6, 6, 5, 6, 5, 5, 4, 6, 5, 5, 4, 5, 4, 4, 3,
	6, 5, 5, 4, 5, 4, 4, 3, 5, 4, 4, 3, 4, 3, 3, 2,
	7, 6, 6, 5, 6, 5, 5, 4, 6, 5, 5, 4, 5, 4, 4, 3,
	6, 5, 5, 4, 5, 4, 4, 3, 5, 4, 4, 3, 4, 3, 3, 2,
	6, 5, 5, 4, 5, 4, 4, 3, 5, 4, 4, 3, 4, 3, 3, 2,
	5, 4, 4, 3, 4, 3, 3, 2, 4, 3, 3, 2, 3, 2, 2, 1,
	7, 6, 6, 5, 6, 5, 5, 4, 6, 5, 5, 4, 5, 4, 4, 3,
	6, 5, 5, 4, 5, 4, 4, 3, 5, 4, 4, 3, 4, 3, 3, 2,
	6, 5, 5, 4, 5, 4, 4, 3, 5, 4, 4, 3, 4, 3, 3, 2,
	5, 4, 4, 3, 4, 3, 3, 2, 4, 3, 3, 2, 3, 2, 2, 1,
	6, 5, 5, 4, 5, 4, 4, 3, 5, 4, 4, 3, 4, 3, 3, 2,
	5, 4, 4, 3, 4, 3, 3, 2, 4, 3, 3, 2, 3, 2, 2, 1,
	5, 4, 4, 3, 4, 3, 3, 2, 4, 3, 3, 2, 3, 2, 2, 1,
	4, 3, 3, 2, 3, 2, 2, 1, 3, 2, 2, 1, 2, 1, 1, 0
};

static void sprd_nand_pageseed(struct sprd_nand_host *host, uint32_t page)
{
	uint32_t offset = 0;
	uint32_t numbit = 0;
	uint32_t mask = 0;
	uint32_t shift = 0;
	uint32_t remain = 0;
	uint32_t i = 0;
	uint32_t j = 0;

	if (page & (~0xff))
		return;

	memset(host->seedbuf_v, 0, SEED_BUF_SIZE * 4);
	offset = page >> 4;
	for (i = 0; i < SEED_TBL_SIZE; i++) {
		switch (i & 0x3) {
		case 0:
			numbit = 13; break;
		case 1:
			numbit = 17; break;
		case 2:
			numbit = 19; break;
		case 3:
			numbit = 23; break;
		}
		for (j = 0; j <= numbit - 1; j++) {
			if (seedtbl[i] & (1 << j))
				host->seedbuf_v[i] |= 1 << (numbit - 1 - j);
		}
		if (offset) {
			if (offset > numbit - 1)
				shift = offset - numbit;
			else
				shift = offset;
			mask = ((1 << numbit) - 1) >> shift;
			remain = host->seedbuf_v[i] & ~mask;
			remain >>= numbit - shift;
			host->seedbuf_v[i] &= mask;
			host->seedbuf_v[i] <<= shift;
			host->seedbuf_v[i] |= remain;
		}
	}
	host->seedbuf_v[SEED_TBL_SIZE] = host->seedbuf_v[0];
	host->seedbuf_v[SEED_TBL_SIZE+1] = host->seedbuf_v[1];
	host->seedbuf_v[SEED_TBL_SIZE+2] = host->seedbuf_v[2];
	host->seedbuf_v[SEED_TBL_SIZE+3] = host->seedbuf_v[3];
}

static void sprd_nand_enable_randomizer(struct sprd_nand_host *host,
				uint32_t page, uint32_t mode)
{
	uint32_t spl_page = 0;
	uint32_t *seedaddr =  host->seedbuf_p + ((page & 0xf) << 2);

	spl_page = SPL_MAX_SIZE / host->param.page_size;
	if (page < spl_page && mode != MTD_OPS_RAW) {
		sprd_nand_pageseed(host, page);

		writel_relaxed(NFC_POLYNOMIALS0, NFC_POLY0_REG);
		writel_relaxed(NFC_POLYNOMIALS1, NFC_POLY1_REG);
		writel_relaxed(NFC_POLYNOMIALS2, NFC_POLY2_REG);
		writel_relaxed(NFC_POLYNOMIALS3, NFC_POLY3_REG);
		writel_relaxed(NFC_POLYNOMIALS0, NFC_POLY4_REG);
		writel_relaxed(NFC_POLYNOMIALS1, NFC_POLY5_REG);
		writel_relaxed(NFC_POLYNOMIALS2, NFC_POLY6_REG);
		writel_relaxed(NFC_POLYNOMIALS3, NFC_POLY7_REG);
		writel_relaxed(0x0, NFC_SEED_ADDRH_REG);
		writel_relaxed(seedaddr, NFC_SEED_ADDRL_REG);
		host->nfc_cfg3 |= CFG3_RANDOM_EN | CFG3_POLY_4R1_EN;
		writel_relaxed(host->nfc_cfg3, NFC_CFG3_REG);
	}
}

static void sprd_nand_disable_randomizer(struct sprd_nand_host *host,
				uint32_t page, uint32_t mode)
{
	uint32_t spl_page = 0;

	spl_page = SPL_MAX_SIZE / host->param.page_size;
	if (page < spl_page && mode != MTD_OPS_RAW) {
		host->nfc_cfg3 &= ~(CFG3_POLY_4R1_EN | CFG3_RANDOM_EN);
		writel_relaxed(host->nfc_cfg3, NFC_CFG3_REG);
	}
}

static void sprd_nand_cmd_exec(struct sprd_nand_host *host,
		struct nand_inst *program, uint32_t repeat, uint32_t if_use_int)
{
	uint32_t i;

	host->nfc_cfg0_val |= CFG0_SET_REPEAT_NUM(repeat);
	writel_relaxed(host->nfc_cfg0_val, NFC_CFG0_REG);
	writel_relaxed(host->nfc_time0_val, NFC_TIMING0_REG);
	writel_relaxed(host->nfc_sts_mach_val, NFC_STAT_STSMCH_REG);
	for (i = 0; i < program->cnt; i += 2) {
		writel_relaxed((program->inst[i] |
		(program->inst[i + 1] << 16)), NFC_INST00_REG + (i << 1));
	}

	if (!if_use_int) {
		int_dis_all();
		int_clr_all();
	} else {
		int_dis_all();
		int_clr_all();
		int_en(program->int_bits);
	}

	writel_relaxed(host->nfc_start | CTRL_NFC_CMD_START, NFC_START_REG);
}

static int sprd_nand_cmd_wait(struct sprd_nand_host *host,
	struct nand_inst *program, uint32_t if_int)
{
	uint32_t regVal;
	int ret = -EIO;
	uint32_t soft_timeout = jiffies_to_usecs(jiffies);
	uint32_t nfc_timeout_val = 0;

	if (strcmp(program->program_name, "_inst_reset") == 0)
		nfc_timeout_val = NFC_RESET_TIMEOUT_VAL;
	else
		nfc_timeout_val = NFC_TIMEOUT_VAL;

	if (!if_int) {
		do {
			regVal = int_rawsts_get();
			if ((INT_TO | INT_STSMCH | INT_WP)
					& regVal & program->int_bits) {
				ret = -EIO;
				pr_err("nand:sprd_nand_cmd_wait failed.\n");
				pr_err("regval = 0x%x,programname = %s\n",
						regVal, program->program_name);
				break;
			} else if (INT_DONE & regVal & program->int_bits) {
				ret = 0;
				break;
			}
		} while ((jiffies_to_usecs(jiffies) - soft_timeout)
							< nfc_timeout_val);
	} else {
		/* to do something. */
	}
	if (ret) {
		pr_err("sprd nand cmd %s fail", program->program_name);
		print_hex_dump(KERN_ERR, "", DUMP_PREFIX_OFFSET, 32, 4,
				(void *)host->ioaddr, 0x140, 1);
		dump_stack();
	}

	return ret;
}

/* host state0 is used for nand id and reset cmd */
static void init_reg_state0(struct sprd_nand_host *host)
{
	host->nfc_cfg0 = CFG0_DEF0_MAST_ENDIAN | CFG0_DEF0_SECT_NUM_IN_INST |
		CFG0_DEF0_DETECT_ALL_FF | CFG0_SET_WPN;
	host->nfc_cfg3 = CFG3_DETECT_ALL_FF;
	host->nfc_cfg4 = CFG4_SLICE_CLK_EN | CFG4_PHY_DLL_CLK_2X_EN;
	writel_relaxed(host->nfc_cfg0, NFC_CFG0_REG);
	writel_relaxed(host->nfc_cfg3, NFC_CFG3_REG);
	writel_relaxed(host->nfc_cfg4, NFC_CFG4_REG);

	writel_relaxed(RAM_MAIN_ADDR(-1), NFC_MAIN_ADDRH_REG);
	writel_relaxed(RAM_MAIN_ADDR(-1), NFC_MAIN_ADDRL_REG);
	writel_relaxed(RAM_SPAR_ADDR(-1), NFC_SPAR_ADDRH_REG);
	writel_relaxed(RAM_SPAR_ADDR(-1), NFC_SPAR_ADDRL_REG);
	writel_relaxed(RAM_STAT_ADDR(-1), NFC_STAT_ADDRH_REG);
	writel_relaxed(RAM_STAT_ADDR(-1), NFC_STAT_ADDRL_REG);
	/* need delay time after set NFC_CFG0_REG */
	mdelay(1);
}

static void sprd_nand_select_cs(struct sprd_nand_host *host, int cs)
{
	host->nfc_cfg0 = (host->nfc_cfg0 & CFG0_CS_MSKCLR)
		| CFG0_SET_CS_SEL(cs);
}

static int sprd_nand_reset(struct sprd_nand_host *host)
{

	struct nand_inst *inst = &inst_reset;

	host->nfc_sts_mach_val = DEF0_MATCH;
	host->nfc_time0_val = host->nfc_time0_r;
	host->nfc_cfg0_val = host->nfc_cfg0 | CFG0_SET_NFC_MODE(0x2);
	sprd_nand_cmd_exec(host, inst, 1, 0);
	return sprd_nand_cmd_wait(host, inst, 0);
}

static int sprd_nand_readid(struct sprd_nand_host *host)
{
	struct nand_inst *inst = &inst_readid;
	int ret;
	uint32_t id0, id1;
	static uint32_t if_has_read;

	host->nfc_sts_mach_val = DEF0_MATCH;
	host->nfc_cfg0_val = host->nfc_cfg0 | CFG0_SET_NFC_MODE(0x2);
	host->nfc_time0_val = 0x1f | (7 << 6) | (0x1f << 11) |
				(0x1f << 16) | (0x1f << 22) | (0x1f << 27);
	sprd_nand_cmd_exec(host, inst, 1, 0);
	ret = sprd_nand_cmd_wait(host, inst, 0);
	if (ret != 0)
		return ret;

	id0 = readl_relaxed(NFC_STATUS0_REG);
	id1 = readl_relaxed(NFC_STATUS1_REG);
	if (if_has_read == 0) {
		if_has_read = 1;
		host->param.id[0] = (uint8_t) (id0 & 0xFF);
		host->param.id[1] = (uint8_t) ((id0 >> 8) & 0xFF);
		host->param.id[2] = (uint8_t) ((id0 >> 16) & 0xFF);
		host->param.id[3] = (uint8_t) ((id0 >> 24) & 0xFF);
		host->param.id[4] = (uint8_t) (id1 & 0xFF);
		host->param.id[5] = (uint8_t) ((id1 >> 8) & 0xFF);
		host->param.id[6] = (uint8_t) ((id1 >> 16) & 0xFF);
		host->param.id[7] = (uint8_t) ((id1 >> 24) & 0xFF);
	} else if ((host->param.id[0] !=
		(uint8_t) (id0 & 0xFF)) ||
		(host->param.id[1] !=
				(uint8_t) ((id0 >> 8) & 0xFF)) ||
		(host->param.id[2] !=
				(uint8_t) ((id0 >> 16) & 0xFF)) ||
		(host->param.id[3] !=
				(uint8_t) ((id0 >> 24) & 0xFF)) ||
		(host->param.id[4] !=
				(uint8_t) (id1 & 0xFF)) ||
		(host->param.id[5] !=
				(uint8_t) ((id1 >> 8) & 0xFF)) ||
		(host->param.id[6] !=
				(uint8_t) ((id1 >> 16) & 0xFF)) ||
		(host->param.id[7] !=
				(uint8_t) ((id1 >> 24) & 0xFF))) {
		return -EINVAL;
	}

	return ret;
}

static void sprd_nand_delect_cs(struct sprd_nand_host *host, int ret)
{
	if (ret == -EIO) {
		writel_relaxed(CTRL_NFC_CMD_CLR, NFC_START_REG);
		sprd_nand_reset(host);
	}
}
static void sprd_nand_set_timing_config(struct sprd_nand_host *host,
			struct sprd_nand_timing *timing, uint32_t clk_hz)
{
	uint32_t reg_val, temp_val, clk_mhz;

	reg_val = 0;
	/* the clock source is 2x clock */
	clk_mhz = clk_hz / 2000000;
	/* get acs value : 0ns */
	reg_val |= ((2 & 0x1F) << NFC_ACS_OFFSET);

	/* temp_val: 0: 1clock, 1: 2clocks... */
	temp_val = timing->ace_ns * clk_mhz / 1000 - 1;
	if (((timing->ace_ns * clk_mhz) % 1000)  != 0)
		temp_val++;

	reg_val |= ((temp_val & 0x1F) << NFC_ACE_OFFSET);

	/* get rws value : 20 ns */
	temp_val = 20 * clk_mhz / 1000 - 1;
	reg_val |= ((temp_val & 0x3F) << NFC_RWS_OFFSET);

	/* get rws value : 0 ns */
	reg_val |= ((2 & 0x1F) << NFC_RWE_OFFSET);

	/* get rwh value */
	temp_val = timing->rwh_ns * clk_mhz / 1000 - 1;
	if (((timing->rwh_ns * clk_mhz) % 1000)  != 0)
		temp_val++;

	reg_val |= ((temp_val & 0x1F) << NFC_RWH_OFFSET);

	/* get rwl value */
	temp_val = timing->rwl_ns * clk_mhz / 1000 - 1;
	if (((timing->rwl_ns * clk_mhz) % 1000)  != 0)
		temp_val++;

	reg_val |= (temp_val & 0x3F);

	pr_info("%s nand timing val: 0x%x\n\r", __func__, reg_val);

	host->nfc_time0_r = reg_val;
	host->nfc_time0_w = reg_val;
	host->nfc_time0_e = reg_val;
}

static struct sprd_nand_maker *sprd_nand_find_maker(uint8_t idmaker)
{
	struct sprd_nand_maker *pmaker = maker_table;

	while (pmaker->idmaker != 0) {
		if (pmaker->idmaker == idmaker)
			return pmaker;
		pmaker++;
	}

	return NULL;
}

static struct sprd_nand_device *sprd_nand_find_device(
			struct sprd_nand_maker *pmaker, uint8_t id_device)
{
	struct sprd_nand_device *pdevice = pmaker->p_devtab;

	while (pdevice->id_device != 0) {
		if (pdevice->id_device == id_device)
			return pdevice;
		pdevice++;
	}

	return NULL;
}

static void sprd_nand_print_info(struct sprd_nand_vendor_param *p)
{
	struct sprd_nand_device *pdevice;
	struct sprd_nand_maker *pmaker;

	pmaker = sprd_nand_find_maker(p->idmaker);
	pdevice = sprd_nand_find_device(pmaker, p->id_device);

	pr_info("device is %s:%s\n", pmaker->p_name, pdevice->p_name);
	pr_info("block size is %d\n", p->blk_size);
	pr_info("page size is %d\n", p->page_size);
	pr_info("spare size is %d\n", p->nspare_size);
	pr_info("eccbits is %d\n", p->s_oob.ecc_bits);
}

struct sprd_nand_vendor_param *sprd_get_nand_param(uint8_t *id)
{
	struct sprd_nand_vendor_param *param = sprd_nand_vendor_param_table;
	uint32_t i;

	for (i = 0; i < 5; i++)
		pr_info("id[%d] is %x\n", i, id[i]);

	while (param->idmaker != 0) {
		if ((param->id[0] == id[0]) &&
			(param->id[1] == id[1]) &&
			(param->id[2] == id[2]) &&
			(param->id[3] == id[3]) &&
			(param->id[4] == id[4])) {

			sprd_nand_print_info(param);
			return param;
		}
		param++;
	}
	pr_info("Nand params unconfigured, please check it. Halt on booting!!!\n");

	return NULL;
}

static int sprd_nand_param_init_nandhw(struct sprd_nand_host *host)
{
	struct sprd_nand_vendor_param *param;

	/* get base param info */
	param = sprd_get_nand_param(host->param.id);
	if (!param)
		return -EINVAL;

	sprd_nand_set_timing_config(host, &(param->s_timing), host->frequence);
	host->param.nblkcnt = param->blk_num;
	host->param.npage_per_blk = param->blk_size / param->page_size;
	host->param.nsect_per_page = param->page_size / param->nsect_size;
	host->param.nsect_size = param->nsect_size;
	host->param.nspare_size = param->s_oob.oob_size;
	host->param.page_size = param->page_size; /* no contain spare */
	host->param.main_size = param->page_size;
	host->param.spare_size = param->nspare_size;
	host->param.badflag_pos = 0; /* --- default:need fixed */
	host->param.badflag_len = 2; /* --- default:need fixed */
	host->param.ecc_mode = param->s_oob.ecc_bits;
	host->param.ecc_pos = param->s_oob.ecc_pos;
	host->param.ecc_size = param->s_oob.ecc_size;
	host->param.info_pos = param->s_oob.info_pos;
	host->param.info_size = param->s_oob.info_size;

	host->param.nbus_width = param->nbus_width;
	host->param.ncycle = param->ncycles;

	pr_info("nand:nBlkcnt = [%d], npageperblk = [%d]\n",
			host->param.nblkcnt, host->param.npage_per_blk);
	pr_info("nand:nsectperpage = [%d], nsecsize = [%d]\n",
			host->param.nsect_per_page, host->param.nsect_size);
	pr_info("nand:nspare_size = [%d], page_size = [%d]\n",
			host->param.nspare_size, host->param.page_size);
	pr_info("nand:page_size =  [%d], spare_size = [%d]\n",
			host->param.page_size, host->param.spare_size);
	pr_info("nand:ecc_mode = [%d], ecc_pos = [%d]\n",
			host->param.ecc_mode, host->param.ecc_pos);
	pr_info("nand:ecc_size = [%d], info_pos = [%d], info_size = [%d]\n",
			host->param.ecc_size, host->param.info_pos,
			host->param.info_size);

	/* setting globe param seting */
	switch (host->param.ecc_mode) {
	case 1:
		host->risk_threshold = 1;
		host->eccmode_reg = 0;
		break;
	case 2:
		host->risk_threshold = 1;
		host->eccmode_reg = 1;
		break;
	case 4:
		host->risk_threshold = 2;
		host->eccmode_reg = 2;
		break;
	case 8:
		host->risk_threshold = 4;
		host->eccmode_reg = 3;
		break;
	case 12:
		host->risk_threshold = 6;
		host->eccmode_reg = 4;
		break;
	case 16:
		host->risk_threshold = 8;
		host->eccmode_reg = 5;
		break;
	case 24:
		host->risk_threshold = 12;
		host->eccmode_reg = 6;
		break;
	case 40:
		host->risk_threshold = 20;
		host->eccmode_reg = 7;
		break;
	case 60:
		host->risk_threshold = 30;
		host->eccmode_reg = 8;
		break;
	default:
		panic("nand:sprd nand ecc mode not support!\n");
		return -EINVAL;
			break;
	}

	host->sect_perpage_shift = ffs(host->param.nsect_per_page) - 1;
	host->sectshift = ffs(host->param.nsect_size) - 1;
	host->sectmsk = (1 << host->sectshift) - 1;
	host->pageshift = ffs(host->param.nsect_per_page <<
					host->sectshift) - 1;
	host->blkshift = ffs(host->param.npage_per_blk <<
					host->pageshift) - 1;
	host->bufshift = ffs(NFC_MBUF_SIZE) - 1;
	host->bufshift = min(host->bufshift, host->blkshift);
	pr_info("nand: sectperpgshift %d, sectshift %d\n",
		host->sect_perpage_shift, host->sectshift);
	pr_info("nand:secmsk %d, pageshift %d,blkshift %d, bufshift %d\n",
		host->sectmsk, host->pageshift, host->blkshift, host->bufshift);

	return 0;
}

static void sprd_nand_param_init_inst(struct sprd_nand_host *host)
{
	uint32_t column = (1 << host->pageshift);

	pr_debug("nand:param init inst column %d,host->pageshift %d\n",
			column, host->pageshift);
	if (host->param.nbus_width == BW_16) {
		pr_debug("nand:buswidth = 16\n");
		column >>= 1;
	}
	/* erase */
	sprd_nand_cmd_init(host->inst_erase, "_inst_erase",
			INT_TO | INT_DONE | INT_WP | INT_STSMCH);
	sprd_nand_cmd_add(host->inst_erase, INST_CMD(0x60));
	sprd_nand_cmd_tag(host->inst_erase);
	sprd_nand_cmd_add(host->inst_erase, INST_ADDR(0, 1));
	sprd_nand_cmd_add(host->inst_erase, INST_ADDR(0, 0));
	if (host->param.ncycle == 5)
		sprd_nand_cmd_add(host->inst_erase, INST_ADDR(0, 0));
	sprd_nand_cmd_add(host->inst_erase, INST_CMD(0xD0));
	sprd_nand_cmd_add(host->inst_erase, INST_WRB0());
	sprd_nand_cmd_add(host->inst_erase, INST_CMD(0x70));
	sprd_nand_cmd_add(host->inst_erase, INST_IDST(1));
	sprd_nand_cmd_add(host->inst_erase, INST_DONE());
	/* read main+spare(info)+Ecc or Raw */
	sprd_nand_cmd_init(host->inst_read_main_spare, "_inst_read_main_spare",
			INT_TO | INT_DONE);
	sprd_nand_cmd_add(host->inst_read_main_spare, INST_CMD(0x00));
	sprd_nand_cmd_add(host->inst_read_main_spare,
			INST_ADDR((0xFF & (uint8_t)column), 0));
	sprd_nand_cmd_add(host->inst_read_main_spare,
			INST_ADDR((0xFF & (uint8_t)(column >> 8)), 0));
	sprd_nand_cmd_tag(host->inst_read_main_spare);
	sprd_nand_cmd_add(host->inst_read_main_spare, INST_ADDR(0, 1));
	sprd_nand_cmd_add(host->inst_read_main_spare, INST_ADDR(0, 0));
	if (host->param.ncycle == 5)
		sprd_nand_cmd_add(host->inst_read_main_spare, INST_ADDR(0, 0));
	sprd_nand_cmd_add(host->inst_read_main_spare, INST_CMD(0x30));
	sprd_nand_cmd_add(host->inst_read_main_spare, INST_WRB0());
	sprd_nand_cmd_add(host->inst_read_main_spare, INST_SRDT());

	sprd_nand_cmd_add(host->inst_read_main_spare, INST_CMD(0x05));
	sprd_nand_cmd_add(host->inst_read_main_spare, INST_ADDR(0, 0));
	sprd_nand_cmd_add(host->inst_read_main_spare, INST_ADDR(0, 0));
	sprd_nand_cmd_add(host->inst_read_main_spare, INST_CMD(0xE0));
	sprd_nand_cmd_add(host->inst_read_main_spare, INST_MRDT());

	sprd_nand_cmd_add(host->inst_read_main_spare, INST_DONE());
	/* read main raw */
	sprd_nand_cmd_init(host->inst_read_main_raw, "_inst_read_main_raw",
			INT_TO | INT_DONE);
	sprd_nand_cmd_add(host->inst_read_main_raw, INST_CMD(0x00));
	sprd_nand_cmd_add(host->inst_read_main_raw, INST_ADDR(0, 0));
	sprd_nand_cmd_add(host->inst_read_main_raw, INST_ADDR(0, 0));
	sprd_nand_cmd_tag(host->inst_read_main_raw);
	sprd_nand_cmd_add(host->inst_read_main_raw, INST_ADDR(0, 1));
	sprd_nand_cmd_add(host->inst_read_main_raw, INST_ADDR(0, 0));
	if (host->param.ncycle == 5)
		sprd_nand_cmd_add(host->inst_read_main_raw, INST_ADDR(0, 0));
	sprd_nand_cmd_add(host->inst_read_main_raw, INST_CMD(0x30));
	sprd_nand_cmd_add(host->inst_read_main_raw, INST_WRB0());
	sprd_nand_cmd_add(host->inst_read_main_raw, INST_MRDT());
	sprd_nand_cmd_add(host->inst_read_main_raw, INST_DONE());
	/*
	 * read spare raw: read only main or only spare data,
	 * it is read to main addr.
	 */
	sprd_nand_cmd_init(host->inst_read_spare_raw, "_inst_read_spare_raw",
			INT_TO | INT_DONE);
	sprd_nand_cmd_add(host->inst_read_spare_raw, INST_CMD(0x00));
	sprd_nand_cmd_add(host->inst_read_spare_raw,
			INST_ADDR((0xFF & (uint8_t)column), 0));
	sprd_nand_cmd_add(host->inst_read_spare_raw,
			INST_ADDR((0xFF & (uint8_t)(column >> 8)), 0));
	sprd_nand_cmd_tag(host->inst_read_spare_raw);
	sprd_nand_cmd_add(host->inst_read_spare_raw, INST_ADDR(0, 1));
	sprd_nand_cmd_add(host->inst_read_spare_raw, INST_ADDR(0, 0));
	if (host->param.ncycle == 5)
		sprd_nand_cmd_add(host->inst_read_spare_raw, INST_ADDR(0, 0));
	sprd_nand_cmd_add(host->inst_read_spare_raw, INST_CMD(0x30));
	sprd_nand_cmd_add(host->inst_read_spare_raw, INST_WRB0());
	sprd_nand_cmd_add(host->inst_read_spare_raw, INST_MRDT());
	sprd_nand_cmd_add(host->inst_read_spare_raw, INST_DONE());
	/* write main+spare(info)+ecc */
	sprd_nand_cmd_init(host->inst_write_main_spare, "_inst_write_main_spare",
			INT_TO | INT_DONE | INT_WP | INT_STSMCH);
	sprd_nand_cmd_add(host->inst_write_main_spare, INST_CMD(0x80));
	sprd_nand_cmd_add(host->inst_write_main_spare, INST_ADDR(0, 0));
	sprd_nand_cmd_add(host->inst_write_main_spare, INST_ADDR(0, 0));
	sprd_nand_cmd_tag(host->inst_write_main_spare);
	sprd_nand_cmd_add(host->inst_write_main_spare, INST_ADDR(0, 1));
	sprd_nand_cmd_add(host->inst_write_main_spare, INST_ADDR(0, 0));
	if (host->param.ncycle == 5)
		sprd_nand_cmd_add(host->inst_write_main_spare, INST_ADDR(0, 0));
	sprd_nand_cmd_add(host->inst_write_main_spare, INST_MWDT());

	/* just need input column addr */
	sprd_nand_cmd_add(host->inst_write_main_spare, INST_CMD(0x85));
	sprd_nand_cmd_add(host->inst_write_main_spare,
			INST_ADDR((0xFF & (uint8_t)column), 0));
	sprd_nand_cmd_add(host->inst_write_main_spare,
			INST_ADDR((0xFF & (uint8_t)(column >> 8)), 0));
	sprd_nand_cmd_add(host->inst_write_main_spare, INST_SWDT());

	sprd_nand_cmd_add(host->inst_write_main_spare, INST_CMD(0x10));
	sprd_nand_cmd_add(host->inst_write_main_spare, INST_WRB0());
	sprd_nand_cmd_add(host->inst_write_main_spare, INST_CMD(0x70));
	sprd_nand_cmd_add(host->inst_write_main_spare, INST_IDST(1));
	sprd_nand_cmd_add(host->inst_write_main_spare, INST_DONE());
	/* write main raw */
	sprd_nand_cmd_init(host->inst_write_main_raw, "_inst_write_main_raw",
			INT_TO | INT_DONE | INT_WP | INT_STSMCH);
	sprd_nand_cmd_add(host->inst_write_main_raw, INST_CMD(0x80));
	sprd_nand_cmd_add(host->inst_write_main_raw, INST_ADDR(0, 0));
	sprd_nand_cmd_add(host->inst_write_main_raw, INST_ADDR(0, 0));
	sprd_nand_cmd_tag(host->inst_write_main_raw);
	sprd_nand_cmd_add(host->inst_write_main_raw, INST_ADDR(0, 1));
	sprd_nand_cmd_add(host->inst_write_main_raw, INST_ADDR(0, 0));
	if (host->param.ncycle == 5)
		sprd_nand_cmd_add(host->inst_write_main_raw, INST_ADDR(0, 0));
	sprd_nand_cmd_add(host->inst_write_main_raw, INST_MWDT());
	sprd_nand_cmd_add(host->inst_write_main_raw, INST_CMD(0x10));
	sprd_nand_cmd_add(host->inst_write_main_raw, INST_WRB0());
	sprd_nand_cmd_add(host->inst_write_main_raw, INST_CMD(0x70));
	sprd_nand_cmd_add(host->inst_write_main_raw, INST_IDST(1));
	sprd_nand_cmd_add(host->inst_write_main_raw, INST_DONE());
	/* write spare raw */
	sprd_nand_cmd_init(host->inst_write_spare_raw, "_inst_write_spare_raw",
			INT_TO | INT_DONE | INT_WP | INT_STSMCH);
	sprd_nand_cmd_add(host->inst_write_spare_raw, INST_CMD(0x80));
	sprd_nand_cmd_add(host->inst_write_spare_raw,
					INST_ADDR((0xFF & column), 0));
	sprd_nand_cmd_add(host->inst_write_spare_raw,
					INST_ADDR((0xFF & (column >> 8)), 0));
	sprd_nand_cmd_tag(host->inst_write_spare_raw);
	sprd_nand_cmd_add(host->inst_write_spare_raw, INST_ADDR(0, 1));
	sprd_nand_cmd_add(host->inst_write_spare_raw, INST_ADDR(0, 0));
	if (host->param.ncycle == 5)
		sprd_nand_cmd_add(host->inst_write_spare_raw, INST_ADDR(0, 0));
	sprd_nand_cmd_add(host->inst_write_spare_raw, INST_SWDT());
	sprd_nand_cmd_add(host->inst_write_spare_raw, INST_CMD(0x10));
	sprd_nand_cmd_add(host->inst_write_spare_raw, INST_WRB0());
	sprd_nand_cmd_add(host->inst_write_spare_raw, INST_CMD(0x70));
	sprd_nand_cmd_add(host->inst_write_spare_raw, INST_IDST(1));
	sprd_nand_cmd_add(host->inst_write_spare_raw, INST_DONE());
}

static int sprd_nand_param_init_buf(struct sprd_nand_host *host)
{
	dma_addr_t phys_addr = 0;
	void *virt_ptr = 0;

	uint32_t mbuf_size = (1 << host->bufshift);
	uint32_t sbuf_size = (1 << (host->bufshift - host->sectshift)) *
			host->param.nspare_size;
	uint32_t stsbuf_size = ((1 << (host->bufshift - host->sectshift)) *
			(sizeof(struct nand_ecc_stats)));
	uint32_t seedbuf_size = SEED_BUF_SIZE * 4;

	pr_debug("nand:mbuf_size %d, sbuf_size %d, stsbuf_size %d\n",
		mbuf_size, sbuf_size, stsbuf_size);
	pr_debug("nand:host->bufshift %d, param.nspare_size %d,sectshift %d\n",
		host->bufshift, host->param.nspare_size, host->sectshift);

	virt_ptr = dma_alloc_coherent(NULL, mbuf_size, &phys_addr, GFP_KERNEL);
	if (!virt_ptr) {
		pr_info("nand:Failed to allocate memory for DMA main buffer\n");
		return -ENOMEM;
	}
	host->mbuf_p = (uint8_t *) phys_addr;
	host->mbuf_v = (uint8_t *) virt_ptr;

	virt_ptr = dma_alloc_coherent(NULL, sbuf_size,
							&phys_addr, GFP_KERNEL);
	if (!virt_ptr) {
		pr_info("nand:Failed to allocate memory for DMA spare buffer\n");
		dma_free_coherent(NULL, mbuf_size, (void *)host->mbuf_v,
						(dma_addr_t) host->mbuf_p);
		return -ENOMEM;
	}
	host->sbuf_p = (uint8_t *) phys_addr;
	host->sbuf_v = (uint8_t *) virt_ptr;

	virt_ptr = dma_alloc_coherent(NULL, stsbuf_size,
						&phys_addr, GFP_KERNEL);
	if (!virt_ptr) {
		pr_info("nand:Failed to allocate memory for DMA sts buffer\n");
		dma_free_coherent(NULL, mbuf_size, (void *)host->mbuf_v,
				(dma_addr_t) host->mbuf_p);
		dma_free_coherent(NULL, sbuf_size, (void *)host->sbuf_v,
				(dma_addr_t) host->sbuf_p);
		return -ENOMEM;
	}
	host->stsbuf_p = (uint8_t *) phys_addr;
	host->stsbuf_v = (uint8_t *) virt_ptr;

	virt_ptr = dma_alloc_coherent(NULL, seedbuf_size,
						&phys_addr, GFP_KERNEL);
	if (!virt_ptr) {
		pr_info("nand:Failed to allocate memory for DMA seed buffer\n");
		return -ENOMEM;
	}
	host->seedbuf_p = (uint32_t *) phys_addr;
	host->seedbuf_v = (uint32_t *) virt_ptr;
	return 0;
}

static int sprd_nand_param_init(struct sprd_nand_host *host)
{
	int ret = 0;

	ret = sprd_nand_param_init_nandhw(host);
	if (ret)
		return ret;

	sprd_nand_param_init_inst(host);
	/* sprd_nand_param_init_timing(host); */
	return sprd_nand_param_init_buf(host);
}

static void sprd_nand_init_reg_state1(struct sprd_nand_host *host)
{
	host->nfc_start |= CTRL_DEF1_ECC_MODE(host->eccmode_reg);
	host->nfc_cfg0 |= CFG0_DEF1_SECT_NUM(host->param.nsect_per_page)
		| CFG0_DEF1_BUS_WIDTH(host->param.nbus_width)
		| CFG0_DEF1_MAIN_SPAR_APT(host->param.nsect_per_page);
	host->nfc_cfg1 = CFG1_DEF1_SPAR_INFO_SIZE(host->param.info_size)
		| CFG1_DEF1_SPAR_SIZE(host->param.nspare_size)
		| CFG1_DEF_MAIN_SIZE(host->param.nsect_size);
	host->nfc_cfg2 = CFG2_DEF1_SPAR_SECTOR_NUM(host->param.nsect_per_page)
		| CFG2_DEF1_SPAR_INFO_POS(host->param.info_pos)
		| CFG2_DEF1_ECC_POSITION(host->param.ecc_pos);

	host->nfc_cfg0 |= CFG0_SET_WPN;
	writel_relaxed(host->nfc_cfg0, NFC_CFG0_REG);
	writel_relaxed(host->nfc_cfg1, NFC_CFG1_REG);
	writel_relaxed(host->nfc_cfg2, NFC_CFG2_REG);
	writel_relaxed(0xFFFFFFFF, NFC_TIMEOUT_REG);

	writel_relaxed(0x0, NFC_STAT_ADDRH_REG);
	writel_relaxed(RAM_STAT_ADDR(host->stsbuf_p), NFC_STAT_ADDRL_REG);
	writel_relaxed(0x0, NFC_MAIN_ADDRH_REG);
	writel_relaxed(RAM_MAIN_ADDR(host->mbuf_p), NFC_MAIN_ADDRL_REG);
	writel_relaxed(0x0, NFC_SPAR_ADDRH_REG);
	writel_relaxed(RAM_SPAR_ADDR(host->sbuf_p), NFC_SPAR_ADDRL_REG);
	mdelay(1);
}

static int sprd_nand_debugfs_show(struct sprd_nfc_base *nfcbase,
		struct seq_file *s, void *data)
{
	uint32_t i;
	struct sprd_nand_host *host = (struct sprd_nand_host *) nfcbase->priv;

	seq_printf(s, "nand drv irq = %d\n", host->irq);
	seq_printf(s, "nand drv freq = %d\n", host->frequence);
	seq_printf(s, "nand drv ioaddr = 0x%x\n", (uint32_t) (host->ioaddr));
	/*
	 * resource1: nand param & mtd ecclayout & risk
	 * threshold & nessisary const value
	 */
	seq_printf(s, "drv eccmode_reg = 0x%x\n", host->eccmode_reg);
	seq_printf(s, "drv risk_threshold = %d\n", host->risk_threshold);
	seq_printf(s, "drv csnum = %d:", host->csnum);
	for (i = 0; i < host->csnum; i++)
		seq_printf(s, "\t%d", host->cs[i]);

	seq_puts(s, "\n");
	/* seq_printf(s, "drv csDis = %d\n", host->csDis); */
	seq_printf(s, "drv sect_perpage_shift = %d\n",
					host->sect_perpage_shift);
	seq_printf(s, "drv sectshift = %d\n", host->sectshift);
	seq_printf(s, "drv sectmsk = 0x%x\n", host->sectmsk);
	seq_printf(s, "drv pageshift = %d\n", host->pageshift);
	seq_printf(s, "drv blkshift = %d\n", host->blkshift);
	seq_printf(s, "drv bufshift = %d\n", host->bufshift);
	/* resource3: local DMA buffer */
	seq_printf(s, "drv mbuf_p = 0x%x\n", (uint32_t) host->mbuf_p);
	seq_printf(s, "drv mbuf_v = 0x%x\n", (uint32_t) host->mbuf_v);
	seq_printf(s, "drv sbuf_p = 0x%x\n", (uint32_t) host->sbuf_p);
	seq_printf(s, "drv sbuf_v = 0x%x\n", (uint32_t) host->sbuf_v);
	seq_printf(s, "drv stsbuf_p = 0x%x\n", (uint32_t) host->stsbuf_p);
	seq_printf(s, "drv stsbuf_v = 0x%x\n", (uint32_t) host->stsbuf_v);
	/*
	 * resource4: register base value. some value is const while operate
	 * on nand flash, we store it to local valuable.
	 * Time & cfg & status mach. It is different with operation R W E.
	 */
	seq_printf(s, "drv nfc_time0_r = 0x%x\n", host->nfc_time0_r);
	seq_printf(s, "drv nfc_time0_w = 0x%x\n", host->nfc_time0_w);
	seq_printf(s, "drv nfc_time0_e = 0x%x\n", host->nfc_time0_e);

	seq_printf(s, "drv nfc_cfg0 = 0x%x\n", host->nfc_cfg0);
	seq_printf(s, "drv nfc_cfg1 = 0x%x\n", host->nfc_cfg1);
	seq_printf(s, "drv nfc_cfg2 = 0x%x\n", host->nfc_cfg2);

	/* seq_printf(s, "---Nand HW Param---\n"); */
	seq_puts(s, "drv param id:");
	for (i = 0; i < NFC_MAX_ID_LEN; i++)
		seq_printf(s, "\t0x%x", host->param.id[i]);

	seq_puts(s, "\n");
	seq_printf(s, "drv param nblkcnt = %d\n", host->param.nblkcnt);
	seq_printf(s, "drv param npage_per_blk = %d\n",
					host->param.npage_per_blk);
	seq_printf(s, "drv param nsect_per_page = %d\n",
					host->param.nsect_per_page);
	seq_printf(s, "drv param nsect_size = %d\n",
					host->param.nsect_size);
	seq_printf(s, "drv param nspare_size = %d\n",
					host->param.nspare_size);
	seq_printf(s, "drv param badflag_pos = %d\n",
					host->param.badflag_pos);
	seq_printf(s, "drv param badflag_len = %d\n",
					host->param.badflag_len);
	seq_printf(s, "drv param ecc_mode = %d\n",
					host->param.ecc_mode);
	seq_printf(s, "drv param ecc_pos = %d\n",
					host->param.ecc_pos);
	seq_printf(s, "drv param ecc_size = %d\n",
					host->param.ecc_size);
	seq_printf(s, "drv param info_pos = %d\n",
					host->param.info_pos);
	seq_printf(s, "drv param info_size = %d\n",
					host->param.info_size);
	seq_printf(s, "drv param nbus_width = %d\n",
					host->param.nbus_width);
	seq_printf(s, "drv param ncycle = %d\n",
					host->param.ncycle);
	/* ACS */
	seq_printf(s, "drv param t_als = %d\n",
					host->param.t_als);
	seq_printf(s, "drv param t_cls = %d\n",
					host->param.t_cls);
	/* ACE */
	seq_printf(s, "drv param t_clh = %d\n",
					host->param.t_clh);
	seq_printf(s, "drv param t_alh = %d\n",
					host->param.t_alh);
	/* RWS */
	seq_printf(s, "drv param t_rr = %d\n",
					host->param.t_rr);
	seq_printf(s, "drv param t_adl = %d\n",
					host->param.t_adl);
	/* RWH & RWL */
	seq_printf(s, "drv param t_wh = %d\n",
					host->param.t_wh);
	seq_printf(s, "drv param t_wp = %d\n",
					host->param.t_wp);
	seq_printf(s, "drv param t_reh = %d\n",
					host->param.t_reh);
	seq_printf(s, "drv param t_rp = %d\n",
					host->param.t_rp);
	return 0;
}

/*
 * 0: ecc pass
 * -EBADMSG: ecc fail
 * -EUCLEAN: ecc risk
 */
static int sprd_nand_check_ff(struct sprd_nand_host *host, uint32_t sects,
				uint32_t mode)
{
	uint32_t i, sectsize, obb_size;
	uint8_t *main_buf, *spare_buf;
	int32_t bit0_num, bit0_total;
	uint32_t mbit_0pos[60];
	uint32_t mbit_0arridx = 0;
	uint32_t sbit_0pos[60];
	uint32_t sbit_0arridx = 0;
	int32_t risk_num;

	risk_num = min_t(int32_t, 4, host->risk_threshold);
	sectsize = (1 << host->sectshift);
	if (mode == MTD_OPS_AUTO_OOB)
		obb_size = host->param.info_size;
	else
		obb_size = host->param.nspare_size;

	main_buf = host->mbuf_v + (sects << host->sectshift);
	spare_buf = host->sbuf_v + (sects * obb_size);

	bit0_total = 0;
	mbit_0arridx = 0;
	for (i = 0; i < sectsize; i++) {
		bit0_num = (int32_t)bit_num8[main_buf[i]];
		if (bit0_num) {
			pr_err("main_buf[i] = 0x%x\n", main_buf[i]);
			bit0_total += bit0_num;
			if (bit0_total > risk_num)
				return -EBADMSG;

			mbit_0pos[mbit_0arridx] = i;
			mbit_0arridx++;
		}
	}
	sbit_0arridx = 0;
	for (i = 0; i < obb_size; i++) {
		bit0_num = (int32_t) bit_num8[spare_buf[i]];
		if (bit0_num) {
			pr_err("spare_buf[i] = 0x%x\n", spare_buf[i]);
			bit0_total += bit0_num;
			if (bit0_total > risk_num)
				return -EBADMSG;

			sbit_0pos[sbit_0arridx] = i;
			sbit_0arridx++;
		}
	}
	for (i = 0; i < mbit_0arridx; i++)
		main_buf[mbit_0pos[i]] = 0xFF;

	for (i = 0; i < sbit_0arridx; i++)
		spare_buf[sbit_0pos[i]] = 0xFF;

	return bit0_total;
}

static int sprd_nand_ecc_analyze(struct sprd_nand_host *host,
		uint32_t num, struct mtd_ecc_stats *ecc_sts, uint32_t mode)
{
	uint32_t i;
	uint32_t n;
	struct nand_ecc_stats *nand_ecc_sts =
				(struct nand_ecc_stats *)(host->stsbuf_v);
	uint32_t sector_num = 16;
	uint32_t ecc_banknum = num / sector_num;
	uint32_t sector = 0;
	int ret = 0;

	for (i = 0; i <= ecc_banknum; i++) {
		for (n = 0; n < min((num - sector_num * i), sector_num); n++) {
			sector = n + sector_num * i;
			switch (ECC_STAT(nand_ecc_sts->ecc_stats[n])) {
			case 0x00:
				/* pass */
				break;
			case 0x02:
			case 0x03:
				/* fail */
				ret = sprd_nand_check_ff(host, sector, mode);
				if (ret == -EBADMSG) {
					ecc_sts->failed++;
				} else {
					ecc_sts->corrected += ret;
				}
				break;
			case 0x01:
				if (ECC_NUM(nand_ecc_sts->ecc_stats[n]) ==
							0x1FF) {
					ret =
					sprd_nand_check_ff(host, sector, mode);
					if (ret == -EBADMSG)
						ecc_sts->failed++;
					else
						ecc_sts->corrected += ret;
				};
				if (host->param.ecc_mode <=
					ECC_NUM(nand_ecc_sts->ecc_stats[n])) {
					ecc_sts->failed++;
					ret = -EBADMSG;
				} else {
					ecc_sts->corrected +=
					ECC_NUM(nand_ecc_sts->ecc_stats[n]);
				}
				break;
			}
			if (-EBADMSG == ret)
				goto err;
		}
		nand_ecc_sts++;
	}
	if (ret) {
		pr_err("sprd nand read ecc sts %d\n", ret);
		print_hex_dump(KERN_ERR, "", DUMP_PREFIX_OFFSET, 32, 4,
				(void *)host->stsbuf_v, 0x100, 1);
	}
	return ret;
err:
	pr_err("sprd nand read ecc sts %d\n", ret);
	return ret;
}


/*
 * if(0!=mBuf) then read main area
 * if(0!=sBuf) then read spare area or read spare info
 * if(MTD_OPS_PLACE_OOB == mode) then just read main area with Ecc
 * if(MTD_OPS_AUTO_OOB == mode) then read main area(if(0!=mBuf)),
 * and spare info(if(0!=sbuf)), with Ecc
 * if(MTD_OPS_RAW == mode) then read main area(if(0!=mBuf)),
 * and spare info(if(0!=sbuf)), without Ecc
 * return
 * 0: ecc pass
 * -EBADMSG	: ecc fail
 * -EUCLEAN	: ecc risk
 * -EIO		: read fail
 * mtd->ecc_stats
 */
static int sprd_nand_read_page_in_block(
			struct sprd_nfc_base *nfc_base,
			uint32_t page, uint32_t num, uint32_t *ret_num,
			uint32_t if_has_mbuf, uint32_t if_has_sbuf,
			uint32_t mode, struct mtd_ecc_stats *ecc_sts)
{
	struct nand_inst *inst;
	int ret = 0;
	uint32_t if_change_buf = 0;

	struct sprd_nand_host *host = (struct sprd_nand_host *) nfc_base->priv;
	struct nand_inst inst_read_main_spare = host->inst_read_main_spare;
	struct nand_inst inst_read_main_raw = host->inst_read_main_raw;
	struct nand_inst inst_read_spare_raw = host->inst_read_spare_raw;

	sprd_nand_select_cs(host, GETCS(page));
	host->nfc_sts_mach_val = MACH_READ;
	host->nfc_time0_val = host->nfc_time0_r;
	host->nfc_cfg0_val = (host->nfc_cfg0 | CFG0_SET_NFC_MODE(0));
	sprd_nand_enable_randomizer(host, page, mode);

	switch (mode) {
	case MTD_OPS_AUTO_OOB:
			host->nfc_cfg0_val |= CFG0_SET_SPARE_ONLY_INFO_PROD_EN;
	case MTD_OPS_PLACE_OOB:
			host->nfc_cfg0_val |= (CFG0_SET_ECC_EN |
				CFG0_SET_MAIN_USE | CFG0_SET_SPAR_USE);
			inst = &inst_read_main_spare;
			break;
	case MTD_OPS_RAW:
			if (if_has_mbuf & if_has_sbuf) {
				host->nfc_cfg0_val |= (CFG0_SET_MAIN_USE |
							CFG0_SET_SPAR_USE);
				inst = &inst_read_main_spare;
			} else if (if_has_mbuf) {
				host->nfc_cfg0_val |= (CFG0_SET_MAIN_USE);
				inst = &inst_read_main_raw;
			} else if (if_has_sbuf) {
				host->nfc_cfg0_val |= (CFG0_SET_MAIN_USE);
				inst = &inst_read_spare_raw;
				/*
			 * 0 nand controller use mainAddr to send spare data,
			 * So i have to change some globe config here
			 * if_change_buf = 1;
			 * 1 change to main buf
			 */
				if_change_buf = 1;
				writel_relaxed(0x0, NFC_MAIN_ADDRH_REG);
				writel_relaxed(RAM_MAIN_ADDR(host->sbuf_p),
					NFC_MAIN_ADDRL_REG);
				/* 2 change page_size */
				host->nfc_cfg1 &= (~CFG1_temp_MIAN_SIZE_MSK);
				host->nfc_cfg1 |=
				CFG1_DEF_MAIN_SIZE(host->param.nspare_size <<
					host->sect_perpage_shift);
				writel_relaxed(host->nfc_cfg1, NFC_CFG1_REG);
				/* 3 change sect number */
				host->nfc_cfg0_val &= (~CFG0_SET_SECT_NUM_MSK);
				host->nfc_cfg0_val |= CFG0_DEF1_SECT_NUM(1);
			} else {
				sprd_nand_delect_cs(host, -EINVAL);
				return -EINVAL;
			}
			break;
	default:
		pr_err("nand:sprd nand ops mode error!\n");
		break;
	}

	sprd_nand_cmd_change(inst, page);
	sprd_nand_cmd_exec(host, inst, num, 0);
	ret = sprd_nand_cmd_wait(host, inst, 0);
	if (if_change_buf) {
		/* 1 change to main buf */
		writel_relaxed(0x0, NFC_MAIN_ADDRH_REG);
		writel_relaxed(RAM_MAIN_ADDR(host->mbuf_p), NFC_MAIN_ADDRL_REG);
		/* 2 change page_size */
		host->nfc_cfg1 &= (~CFG1_temp_MIAN_SIZE_MSK);
		host->nfc_cfg1 |= CFG1_DEF_MAIN_SIZE(host->param.nsect_size);
		writel_relaxed(host->nfc_cfg1, NFC_CFG1_REG);
		/* 3 change sect number */
		writel_relaxed(host->nfc_cfg0, NFC_CFG0_REG);
	}
	sprd_nand_disable_randomizer(host, page, mode);

	if (ret) {
		*ret_num = 0;
		sprd_nand_delect_cs(host, -EIO);
		return -EIO;
	}
	*ret_num = num;

	switch (mode) {
	case MTD_OPS_AUTO_OOB:
	case MTD_OPS_PLACE_OOB:
		ret = sprd_nand_ecc_analyze(host,
					num * host->param.nsect_per_page,
					ecc_sts, mode);
		if (ret)
			pr_err("nand:page %d, ecc error ret %d\n", page, ret);
		break;
	case MTD_OPS_RAW:
		break;
	default:
		panic("nand:sprd nand ops mode error!\n");
		break;
	}
	sprd_nand_delect_cs(host, ret);
	return ret;
}

static int sprd_nand_read_page_retry(
	struct sprd_nfc_base *nfc_base,
	uint32_t page, uint32_t num, uint32_t *ret_num,
	uint32_t if_has_mbuf, uint32_t if_has_sbuf,
	uint32_t mode, struct mtd_ecc_stats *ecc_sts)
{
	int ret, i;

	i = 0;
	do {
		ret = sprd_nand_read_page_in_block(nfc_base, page, num, ret_num,
			if_has_mbuf, if_has_sbuf, mode, ecc_sts);
		if ((!ret) || (-EINVAL == ret) || (-EUCLEAN == ret))
			break;
		i++;
		pr_err("sprd_nand:retry %d\n", i);
	} while (i < 3);

	return 0;
}

static int sprd_nand_write_page_in_blk(
	struct sprd_nfc_base *nfc_base, uint32_t page,
	uint32_t num, uint32_t *ret_num, uint32_t if_has_mbuf,
	uint32_t if_has_sbuf, uint32_t mode)
{
	struct nand_inst *inst;
	int ret = 0;
	uint32_t if_change_buf = 0;
	struct sprd_nand_host *host = (struct sprd_nand_host *) nfc_base->priv;
	struct nand_inst inst_write_main_spare = host->inst_write_main_spare;
	struct nand_inst inst_write_spare_raw = host->inst_write_spare_raw;
	struct nand_inst inst_write_main_raw = host->inst_write_main_raw;

	sprd_nand_select_cs(host, GETCS(page));
	host->nfc_sts_mach_val = MACH_WRITE;
	host->nfc_time0_val = host->nfc_time0_w;
	host->nfc_cfg0_val = (host->nfc_cfg0 |
			CFG0_SET_NFC_MODE(0) | CFG0_SET_NFC_RW);
	sprd_nand_enable_randomizer(host, page, mode);

	switch (mode) {
	case MTD_OPS_AUTO_OOB:
		host->nfc_cfg0_val |= CFG0_SET_SPARE_ONLY_INFO_PROD_EN;
	case MTD_OPS_PLACE_OOB:
		host->nfc_cfg0_val |= (CFG0_SET_ECC_EN |
			CFG0_SET_MAIN_USE | CFG0_SET_SPAR_USE);
		inst = &inst_write_main_spare;
		break;
	case MTD_OPS_RAW:
		if (if_has_mbuf & if_has_sbuf) {
			host->nfc_cfg0_val |=
				(CFG0_SET_MAIN_USE | CFG0_SET_SPAR_USE);
			inst = &inst_write_main_spare;
		} else if (if_has_mbuf) {
			host->nfc_cfg0_val |= (CFG0_SET_MAIN_USE);
			inst = &inst_write_main_raw;
		} else if (if_has_sbuf) {
			/* use main to write spare area */
			host->nfc_cfg0_val |= (CFG0_SET_MAIN_USE);
			inst = &inst_write_spare_raw;
			/*
			 * 0 nand controller use mainAddr to send spare data,
			 * So i have to change some globe config here
			 * if_change_buf = 1;
			 * 1 change to main buf
			 */
			writel_relaxed(0x0, NFC_MAIN_ADDRH_REG);
			writel_relaxed(RAM_MAIN_ADDR(host->sbuf_p),
				NFC_MAIN_ADDRL_REG);
			/* 2 change page_size */
			host->nfc_cfg1 &= (~CFG1_temp_MIAN_SIZE_MSK);
			host->nfc_cfg1 |=
				CFG1_DEF_MAIN_SIZE(host->param.nspare_size <<
				host->sect_perpage_shift);
			writel_relaxed(host->nfc_cfg1, NFC_CFG1_REG);
			/* 3 change sect number */
			host->nfc_cfg0_val &= (~CFG0_SET_SECT_NUM_MSK);
			host->nfc_cfg0_val |= CFG0_DEF1_SECT_NUM(1);
		} else {
			sprd_nand_delect_cs(host, -EINVAL);
			return -EINVAL;
		}
		break;
	default:
		pr_err("nand:sprd nand ops mode error!\n");
		break;
	}

	sprd_nand_cmd_change(inst, page);
	sprd_nand_cmd_exec(host, inst, num, 0);
	ret = sprd_nand_cmd_wait(host, inst, 0);
	if (if_change_buf) {
		/* 1 change to main buf */
		writel_relaxed(0x0, NFC_MAIN_ADDRH_REG);
		writel_relaxed(RAM_MAIN_ADDR(host->mbuf_p), NFC_MAIN_ADDRL_REG);
		/* 2 change page_size */
		host->nfc_cfg1 &= (~CFG1_temp_MIAN_SIZE_MSK);
		host->nfc_cfg1 |= CFG1_DEF_MAIN_SIZE(host->param.nsect_size);
		writel_relaxed(host->nfc_cfg1, NFC_CFG1_REG);
		/* 3 change sect number */
		writel_relaxed(host->nfc_cfg0, NFC_CFG0_REG);
	}
	sprd_nand_disable_randomizer(host, page, mode);

	if (ret) {
		*ret_num = 0;
		sprd_nand_delect_cs(host, -EIO);
		return -EIO;
	}
	*ret_num = num;
	sprd_nand_delect_cs(host, 0);

	return 0;
}

static int sprd_nand_eraseblk(struct sprd_nfc_base *nfc_base, uint32_t page)
{
	int ret = 0;
	struct sprd_nand_host *host = (struct sprd_nand_host *) nfc_base->priv;
	struct nand_inst inst_erase_blk = host->inst_erase;
	struct nand_inst *inst = &inst_erase_blk;

	sprd_nand_select_cs(host, GETCS(page));
	host->nfc_sts_mach_val = MACH_ERASE;
	host->nfc_time0_val = host->nfc_time0_e;
	host->nfc_cfg0_val = (host->nfc_cfg0 | CFG0_SET_NFC_MODE(2));

	sprd_nand_cmd_change(inst, page);
	sprd_nand_cmd_exec(host, inst, 1, 0);
	ret = sprd_nand_cmd_wait(host, inst, 0);
	sprd_nand_delect_cs(host, ret);

	return ret;
}

static int sprd_nand_check_badflag(uint8_t *flag, uint32_t len)
{
	/* caculate zero bit number; */
	uint32_t i, k, num;

	num = 0;
	for (i = 0; i < len; i++) {
		for (k = 0; k < 8; k++) {
			if (flag[i] & (1 << k))
				num++;

		}
	}

	return (num < (len << 2));
}

int sprd_nand_check_badblk(struct sprd_nfc_base *nfc_base, uint32_t page)
{
	int ret;
	uint32_t ret_num;
	struct sprd_nand_host *host = (struct sprd_nand_host *) nfc_base->priv;

	ret = nfc_base->read_page_in_blk(nfc_base, page, 1,
			&ret_num, 0, 1, MTD_OPS_RAW, 0);
	if (ret)
		return ret;

	return sprd_nand_check_badflag(host->sbuf_v + host->param.badflag_pos,
			host->param.badflag_len);
}

int sprd_nand_mark_badblk(struct sprd_nfc_base *nfc_base, uint32_t page)
{
	int ret;
	uint32_t ret_num;
	struct sprd_nand_host *host = (struct sprd_nand_host *) nfc_base->priv;

	memset(host->sbuf_v, 0xFF,
			host->param.nspare_size * host->param.nsect_per_page);
	memset(host->sbuf_v + host->param.badflag_pos,
			0, host->param.badflag_len);
	ret = nfc_base->write_page_in_blk(nfc_base, page, 1,
			&ret_num, 0, 1, MTD_OPS_RAW);

	return 0;
}

static int sprd_nand_ctrl_en(struct sprd_nfc_base *nfc_base, uint32_t en)
{
	struct sprd_nand_host *host = (struct sprd_nand_host *) nfc_base->priv;

	if (en) {
		clk_prepare_enable(host->clk_ahb_enable_gate);
		clk_set_parent(host->clk_nand, host->clk_parent_sdr);
		clk_set_parent(host->clk_ecc, host->clk_parent_sdr);
		clk_prepare_enable(host->clk_nand);
		clk_prepare_enable(host->clk_ecc);
	} else {
		clk_disable_unprepare(host->clk_ahb_enable_gate);
		clk_disable_unprepare(host->clk_nand);
		clk_disable_unprepare(host->clk_ecc);
	}

	return 0;
}

static void nandc_set_mode(struct sprd_nand_host *host, uint8_t Infi_type,
						uint32_t delay)
{
	host->nfc_cfg1 &= ~(CFG1_INTF_TYPE(7));
	host->nfc_cfg1 |= CFG1_INTF_TYPE(Infi_type);
	writel_relaxed(host->nfc_cfg1, NFC_CFG1_REG);
	writel_relaxed(delay, NFC_DLL0_CFG);
	writel_relaxed(delay, NFC_DLL1_CFG);
	writel_relaxed(delay, NFC_DLL2_CFG);
}

static void sprd_nand_suspend(struct sprd_nfc_base *host)
{

}

static int sprd_nand_resume(struct sprd_nfc_base *nfc_base)
{
	struct sprd_nand_host *host = (struct sprd_nand_host *) nfc_base->priv;
	int ret = 0;
	int i = 0;

	ret = sprd_nand_ctrl_en(nfc_base, 1);
	if (ret) {
		pr_err("nand resume enable controller clock fail\n");
		return ret;
	}
	nandc_set_mode(host, 0, 0);
	init_reg_state0(host);

	for (i = 0; i < host->csnum; i++) {
		pr_info("try to probe flash%d\n", i);
		sprd_nand_select_cs(host, i);
		ret = sprd_nand_reset(host);
		if (ret) {
			writel_relaxed(CTRL_NFC_CMD_CLR, NFC_START_REG);
			pr_err("flash%d reset fail\n", i);
			break;
		}
		sprd_nand_delect_cs(host, ret);
	}
	sprd_nand_init_reg_state1(host);
	ret = sprd_nand_ctrl_en(nfc_base, 0);
	if (ret) {
		pr_err("nand resume disable controller clock fail\n");
		return ret;
	}

	return 0;
}

static void sprd_nfc_base_init(
		struct sprd_nfc_base *base, struct sprd_nand_host *host)
{
	uint32_t nsect, pos, i;

	memset(base, 0, sizeof(struct sprd_nfc_base));

	for (i = 0; i < NFC_MAX_ID_LEN; i++)
		base->id[i] = host->param.id[i];

	base->ecc_mode = host->param.ecc_mode;
	base->risk_threshold = host->risk_threshold;
	base->obb_size = host->param.nspare_size
		<< (host->pageshift - host->sectshift);
	base->layout.eccbytes = host->param.ecc_size
		<< (host->pageshift - host->sectshift);
	base->layout.oobavail = host->param.info_size
		<< (host->pageshift - host->sectshift);
	for (nsect = 0, i = 0; nsect < host->param.nsect_per_page; nsect++) {
		for (pos = host->param.ecc_pos; pos < host->param.ecc_pos
				+ host->param.ecc_size; pos++) {
			base->layout.eccpos[i] =
				nsect * host->param.nspare_size + pos;
			i++;
		}
		base->layout.oobfree[nsect].offset =
			nsect * host->param.nspare_size + host->param.info_pos;
		base->layout.oobfree[nsect].length = host->param.info_size;
	}
	base->pageshift = host->pageshift;
	base->blkshift = host->blkshift;
	base->chip_shift = ffs(host->param.nblkcnt << host->blkshift) - 1;
	base->chip_num = host->csnum;
	base->bufshift = host->bufshift;
	base->ctrl_en = sprd_nand_ctrl_en;
	base->ctrl_suspend = sprd_nand_suspend;
	base->ctrl_resume = sprd_nand_resume;
	base->read_page_in_blk = sprd_nand_read_page_retry;
	base->write_page_in_blk = sprd_nand_write_page_in_blk;
	base->nand_erase_blk = sprd_nand_eraseblk;
	base->nand_is_bad_blk = sprd_nand_check_badblk;
	base->nand_mark_bad_blk = sprd_nand_mark_badblk;
	base->mbuf_v = host->mbuf_v;
	base->sbuf_v = host->sbuf_v;
	base->debugfs_drv_show = sprd_nand_debugfs_show;
	base->priv = host;
	pr_info("nand: base->risk_threshold [%d]\n",
			base->risk_threshold);
	pr_info("nand: base->obb_size[%d]\n", base->obb_size);
	pr_info("nand: base->layout.eccbytes [%d]\n",
			base->layout.eccbytes);
	pr_info("nand: base->layout.oobavail [%d]\n",
			base->layout.oobavail);
	pr_info("nand: base->pageshift [%d]\n", base->pageshift);
	pr_info("nand: base->blkshift [%d]\n", base->blkshift);
	pr_info("nand: base->chip_shift[%d]\n", base->chip_shift);
	pr_info("nand: base->chip_num [%d]\n", base->chip_num);
	pr_info("nand: base->bufshift [%d]\n", base->bufshift);
}

static int sprd_nand_parse_dt(
		struct platform_device *pdev, struct sprd_nand_host *host)
{
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOENT;

	ret = of_address_to_resource(np, 0, res);
	if (ret < 0) {
		pr_err("nand:no reg ,failed to parse dt.\n");
		return -EINVAL;
	}

	host->ioaddr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(host->ioaddr)) {
		ret = PTR_ERR(host->ioaddr);
		dev_err(&pdev->dev, "can not map iomem: %d\n", ret);
	}

	host->clk_nand = of_clk_get(np, 0);
	host->clk_ecc = of_clk_get(np, 1);
	host->clk_parent_sdr = of_clk_get(np, 2);
	host->clk_parent_ddr = of_clk_get(np, 3);
	host->clk_ahb_enable_gate = of_clk_get(np, 4);

	clk_prepare_enable(host->clk_ahb_enable_gate);
	clk_set_parent(host->clk_nand, host->clk_parent_sdr);
	host->frequence = clk_get_rate(host->clk_parent_sdr);
	pr_info("nand:sprd nand freq = %d\n", host->frequence);
	clk_prepare_enable(host->clk_nand);
	clk_set_parent(host->clk_ecc, host->clk_parent_sdr);
	clk_prepare_enable(host->clk_ecc);

	return 1;
}

static int sprd_nand_init_global_resource(struct platform_device *pdev,
						struct sprd_nand_host *host)
{
	int ret;

	ret = sprd_nand_parse_dt(pdev, host);
	if (!ret)
		return ret;

	return 0;
}

static int sprd_nand_get_global_resource(struct platform_device *pdev,
						struct sprd_nand_host *host)
{
	host->irq = platform_get_irq(pdev, 0);
	pr_info("nand:sprd nand irq = %d\n", (uint32_t) host->irq);

	return 0;
}

static int sprd_nand_test_is_badblock(struct sprd_nfc_base *nfc_base,
	struct sprd_nand_host *host, uint16_t block, uint16_t page)
{
	uint32_t testpage;
	int ret = 0;

	testpage = (block << 6) + page;
	ret = sprd_nand_check_badblk(nfc_base, testpage);
	if (ret) {
		host->badblkcnt++;
		pr_err("nand:block [%d] is bad.\n", block);
	}

	return ret;
}

static void sprd_nand_test_scan_badblk(struct sprd_nfc_base *nfcbase,
			struct sprd_nand_host *host)
{
	int i = 0;
	int cnt;

	cnt = host->param.nblkcnt;
	pr_info("nand:scan the %d blocks to find bad block.\n", cnt);
	for (i = 0; i < cnt; i++)
		sprd_nand_test_is_badblock(nfcbase, host, i, 0);
}

static int sprd_nand_drv_probe(struct platform_device *pdev)
{
	struct sprd_nand_host *host;
	struct sprd_nfc_base *nfc_base;
	uint32_t i;
	int ret;
	struct resource *res;

	host = kzalloc(sizeof(struct sprd_nand_host), GFP_KERNEL);
	if (!host)
		return -ENOENT;

	nfc_base = kzalloc(sizeof(struct sprd_nfc_base), GFP_KERNEL);
	if (!nfc_base)
		return -ENOENT;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("nand:probe get_resource\n");
		return -ENOENT;
	}
	ret = sprd_nand_init_global_resource(pdev, host);
	if (ret)
		return ret;

	ret = sprd_nand_get_global_resource(pdev, host);
	if (ret)
		return ret;

	nandc_set_mode(host, 0, 0);
	init_reg_state0(host);

	sprd_nand_set_timing_config(host, &default_timing, host->frequence);
	host->csnum = 1;
	host->cs[0] = 0;
	sprd_nand_select_cs(host, 0);
	ret = sprd_nand_reset(host);
	if (ret) {
		pr_err("nand_drv_probe  flash0 reset fail\n");
		sprd_nand_delect_cs(host, ret);
		return ret;
	}
	ret = sprd_nand_readid(host);
	if (ret) {
		pr_err("nand_drv_probe  flash0 sprd_nand_readid fail\n");
		sprd_nand_delect_cs(host, ret);
		return ret;
	}
	sprd_nand_delect_cs(host, ret);

	for (i = 1; i < CFG0_CS_MAX; i++) {
		pr_info("try to probe flash%d\n", i);
		sprd_nand_select_cs(host, i);
		ret = sprd_nand_reset(host);
		if (ret) {
			writel_relaxed(CTRL_NFC_CMD_CLR, NFC_START_REG);
			pr_err("flash%d reset fail\n", i);
					break;
		} else {
			ret = sprd_nand_readid(host);
			if (ret) {
				pr_err("flash%d sprd_nand_readid fail\n", i);
					break;
			} else {
				pr_err("find flash%d,id[] is %x %x %x %x %x\n",
					i, host->param.id[0], host->param.id[1],
					host->param.id[2], host->param.id[3],
					host->param.id[4]);
				host->csnum = host->csnum + 1;
				host->cs[i] = i;
			}
		}
		sprd_nand_delect_cs(host, ret);
	}

	ret = sprd_nand_param_init(host);
	if (ret)
		return ret;

	sprd_nand_init_reg_state1(host);
	sprd_nfc_base_init(nfc_base, host);
	sprd_nand_test_scan_badblk(nfc_base, host);
	pr_err("nand: host nand have [ %d ] bad blocks.\n", host->badblkcnt);
	sprd_nfc_base_register(nfc_base);
	ret = sprd_nand_ctrl_en(nfc_base, 0);
	return 0;
}
static const struct of_device_id sprd_nand_of_match[] = {
	{.compatible = "sprd,nandc-r5"},
	{ }
};

MODULE_DEVICE_TABLE(of, sprd_nand_of_match);
static struct platform_driver sprd_nand_driver = {
	.probe = sprd_nand_drv_probe,
	.driver = {
		.name = "sprd-nand",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sprd_nand_of_match),
	},
};

module_platform_driver(sprd_nand_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("jason.wu@spreadtrum.com");
MODULE_DESCRIPTION("SPRD MTD NAND driver");


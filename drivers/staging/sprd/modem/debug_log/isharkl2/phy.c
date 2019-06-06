/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <asm/div64.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd/isharkl2/isharkl2_glb.h>
#include <linux/regmap.h>

#include "core.h"
#include "phy.h"

static struct regmap *ana_g8;

#define REG_DBG_APB_CLK_SRC_SELECT			0x0000
#define REG_DBG_APB_CLK_DIV_CTL				0x0004
#define REG_DBG_APB_REF_CTL_REG				0x0018
#define REG_DBG_APB_TX_DFX_TOP_CFG_REG			0x0058

#define REG_DBG_APB_TX_DEF_PORT0_DLN_CNT_LPX		0xe0
#define REG_DBG_APB_TX_DEF_PORT0_DLN_CNT_HS_PREP	0xe4
#define REG_DBG_APB_TX_DEF_PORT0_DLN_CNT_HS_ZERO	0xe8
#define REG_DBG_APB_TX_DEF_PORT0_DLN_CNT_HS_TRAIL	0xec
#define REG_DBG_APB_TX_DEF_PORT0_DLN_CNT_HS_EXIT	0xf0
#define REG_DBG_APB_TX_DFE_PORT0_CLN_CNT_HS_TRAIL	0xfc

#define REG_DBG_APB_TX_DFE_PORT0_CLN_CNT_HS_EXIT	0x100
#define REG_DBG_APB_TX_DFE_PORT0_CLN_CNT_LPX		0x104
#define REG_DBG_APB_TX_DFE_PORT0_CLN_CNT_PREP		0x108
#define REG_DBG_APB_TX_DFE_PORT0_CLN_CNT_ZERO		0x10c
#define REG_DBG_APB_TX_DFE_PORT0_CLN_CNT_PLL		0x110
#define REG_DBG_APB_TX_DFE_PORT0_CLN_CNT_POST		0x114
#define REG_DBG_APB_TX_TX_DFE_PORT0_WAIT_CFG		0x118

#define REG_DBG_APB_PLL_X00				0x300
#define REG_DBG_APB_PLL_X01				0x304
#define REG_DBG_APB_PLL_X02				0x308
#define REG_DBG_APB_PLL_X03				0x30c
#define REG_DBG_APB_PLL_X04				0x310
#define REG_DBG_APB_PLL_X05				0x314
#define REG_DBG_APB_PLL_X06				0x318
#define REG_DBG_APB_PLL_X07				0x31c
#define REG_DBG_APB_PLL_X08				0x320
#define REG_DBG_APB_PLL_X09				0x324
#define REG_DBG_APB_PLL_X0A				0x328

/* REG_DBG_APB_REF_CTL_REG  0x0018*/
#define REG_DBG_APB_VREG_REFEN				BIT(0)
#define REG_DBG_APB_VREG_REFSEL(x)			(((x) & 0x1F) << 1)
#define REG_DBG_APB_LPRX_REFSEL(x)			(((x) & 0x3) << 6)

/* REG_DBG_APB_TX_DFX_TOP_CFG_REG  0x0058*/
#define REG_DBG_APB_DFX_DPHY_LATCH_EN			BIT(5)
#define REG_DBG_APB_HSTXBIASSEL				BIT(30)

struct mphy_pll {
	u32 phy_clk;
	u32 div5b;
	u32 div2or3;
	u32 div5bbypass;
	u32 i_fbdivratio;
	u32 i_fracdiv2;
	u32 i_fracdiv1;
	u32 i_fracdiv0;
	u32 i_feedfwrdgain;
	u32 i_fracnen_h;
	u32 i_sscstepsize0;
	u32 i_sscstepsize1;
	u32 i_sscsteplength0;
	u32 i_sscsteplength1;
	u32 i_sscstepnum;
	u32 i_ssctype;
	u32 i_sscen_h;
	u32 i_prop_coeff;
	u32 i_int_coeff;
	u32 i_gainctrl;
	u32 i_tdctargetcnt0;
	u32 i_tdctargetcnt1;
	u32 i_tdcsel;
	u32 i_tdccalsetupdeten;
	u32 i_lockthreshsel;
	u32 i_lockthresh;
	u32 i_afccntsel;
	u32 i_dcoditheren;
	u32 i_useidvdata;
	u32 i_pllwait_cntr;
	u32 cln_cnt_post;
	u32 cln_cnt_prep;
	u32 cln_hs_trail;
	u32 cln_cnt_zero;
	u32 cln_cnt_lpx;
	u32 cln_hs_exit;
	u32 dln_cnt_hs_exit;
	u32 dln_cnt_hs_prep;
	u32 dln_cnt_hs_zero;
	u32 dln_cnt_hs_trail;
	u32 dln_cnt_lpx;
};

static struct mphy_pll pll_array[] = {
	{
		.phy_clk = 1500 * 1000,
		.div5b = 0,
		.div2or3 = 0,
		.div5bbypass = 0,
		.i_fbdivratio = 0x73,
		.i_fracdiv2 = 0x18,
		.i_fracdiv1 = 0x9d,
		.i_fracdiv0 = 0x19,
		.i_feedfwrdgain = 0x2,
		.i_fracnen_h = 0x1,
		.i_sscstepsize0 = 0,
		.i_sscstepsize1 = 0,
		.i_sscsteplength0 = 0,
		.i_sscsteplength1 = 0,
		.i_sscstepnum = 0,
		.i_ssctype = 0,
		.i_sscen_h = 0,
		.i_prop_coeff = 0x5,
		.i_int_coeff = 0x9,
		.i_gainctrl = 0x1,
		.i_tdctargetcnt0 = 0x22,
		.i_tdctargetcnt1 = 0,
		.i_tdcsel = 0x3,
		.i_tdccalsetupdeten = 1,
		.i_lockthreshsel = 0,
		.i_lockthresh = 5,
		.i_afccntsel = 1,
		.i_dcoditheren = 1,
		.i_useidvdata = 0,
		.i_pllwait_cntr = 0x2,
		.cln_cnt_post = 0x1B,
		.cln_cnt_prep = 0xe,
		.cln_hs_trail = 18,
		.cln_cnt_zero = 0x53,
		.cln_cnt_lpx = 0x10,
		.cln_hs_exit = 0x21,
		.dln_cnt_hs_exit = 0,
		.dln_cnt_hs_prep = 0xf,
		.dln_cnt_hs_zero = 0x23,
		.dln_cnt_hs_trail = 0x13,
		.dln_cnt_lpx = 0x10,
	},
	{
		.phy_clk = 2000 * 1000,
		.div5b = 4,
		.div2or3 = 1,
		.div5bbypass = 1,
		.i_fbdivratio = 0x73,
		.i_fracdiv2 = 0x18,
		.i_fracdiv1 = 0x9d,
		.i_fracdiv0 = 0x89,
		.i_feedfwrdgain = 0x2,
		.i_fracnen_h = 0x1,
		.i_sscstepsize0 = 0,
		.i_sscstepsize1 = 0,
		.i_sscsteplength0 = 0,
		.i_sscsteplength1 = 0,
		.i_sscstepnum = 0,
		.i_ssctype = 0,
		.i_sscen_h = 0,
		.i_prop_coeff = 0x5,
		.i_int_coeff = 0x9,
		.i_gainctrl = 0x1,
		.i_tdctargetcnt0 = 0x22,
		.i_tdctargetcnt1 = 0,
		.i_tdcsel = 0x3,
		.i_tdccalsetupdeten = 1,
		.i_lockthreshsel = 0,
		.i_lockthresh = 5,
		.i_afccntsel = 1,
		.i_dcoditheren = 1,
		.i_useidvdata = 0,
		.i_pllwait_cntr = 0x2,
		.cln_cnt_post = 0x16,
		.cln_cnt_prep = 0x9,
		.cln_hs_trail = 10,
		.cln_cnt_zero = 0x42,
		.cln_cnt_lpx = 0xc,
		.cln_hs_exit = 0x1a,
		.dln_cnt_hs_exit = 0x1a,
		.dln_cnt_hs_prep = 0xa,
		.dln_cnt_hs_zero = 0x1c,
		.dln_cnt_hs_trail = 0x10,
		.dln_cnt_lpx = 0xc,
	},
	{
		.phy_clk = 2500 * 1000,
		.div5b = 4,
		.div2or3 = 0,
		.div5bbypass = 1,
		.i_fbdivratio = 0x60,
		.i_fracdiv2 = 0x9,
		.i_fracdiv1 = 0xd8,
		.i_fracdiv0 = 0x9d,
		.i_feedfwrdgain = 0x3,
		.i_fracnen_h = 0x1,
		.i_sscstepsize0 = 0,
		.i_sscstepsize1 = 0,
		.i_sscsteplength0 = 0,
		.i_sscsteplength1 = 0,
		.i_sscstepnum = 0,
		.i_ssctype = 0,
		.i_sscen_h = 0,
		.i_prop_coeff = 0x4,
		.i_int_coeff = 0x8,
		.i_gainctrl = 0x1,
		.i_tdctargetcnt0 = 0x22,
		.i_tdctargetcnt1 = 0,
		.i_tdcsel = 0x3,
		.i_tdccalsetupdeten = 1,
		.i_lockthreshsel = 0,
		.i_lockthresh = 5,
		.i_afccntsel = 1,
		.i_dcoditheren = 1,
		.i_useidvdata = 0,
		.i_pllwait_cntr = 0x2,
		.cln_cnt_post = 0x1a,
		.cln_cnt_prep = 0xb,
		.cln_hs_trail = 0x13,
		.cln_cnt_zero = 0x53,
		.cln_cnt_lpx = 0xf,
		.cln_hs_exit = 0x20,
		.dln_cnt_hs_exit = 0x20,
		.dln_cnt_hs_prep = 0xd,
		.dln_cnt_hs_zero = 0x22,
		.dln_cnt_hs_trail = 0x13,
		.dln_cnt_lpx = 0xf,
	},
};

static inline u32 phy_readl(unsigned long base, u32 addr)
{
	return readl((void __iomem *)(base + addr));
}

static inline void phy_writel(unsigned long base, u32 val, u32 addr)
{
	return writel(val, (void __iomem *)(base + addr));
}

static inline void writel_nbits(unsigned long base, u32 val,
				u8 nbits, u8 offset, u32 addr)
{
	u32 tmp;

	tmp = phy_readl(base, addr);
	tmp &= ~(((1 << nbits) - 1) << offset);
	tmp |= val << offset;
	phy_writel(base, tmp, addr);
	mdelay(1);
}

static int mphy_set_pll_reg(unsigned long base, u32 freq)
{
	struct mphy_pll *pll;
	u8 i;

	switch (freq) {
	case 1500000:
		for (i = 0; i < ARRAY_SIZE(pll_array); i++)
			if (pll_array[i].phy_clk == freq)
				break;
		break;
	case 2000000:
		for (i = 0; i < ARRAY_SIZE(pll_array); i++)
			if (pll_array[i].phy_clk == freq)
				break;
		break;
	case 2500000:
		for (i = 0; i < ARRAY_SIZE(pll_array); i++)
			if (pll_array[i].phy_clk == freq)
				break;
		break;
	default:
		i = 0;
		pr_err("the target freq %u is not supported, set to the default freq 1.5G\n",
		       freq);
		break;
	}

	pll = &pll_array[i];

	writel_nbits(base, pll->div5b, 3, 1, 0x04);
	writel_nbits(base, pll->div2or3, 1, 4, 0x04);
	writel_nbits(base, pll->div5bbypass, 1, 5, 0x04);
	writel_nbits(base, pll->i_fbdivratio, 8, 0, 0x300);
	writel_nbits(base, pll->i_fracdiv2, 6, 16, 0x308);
	writel_nbits(base, pll->i_fracdiv1, 8, 8, 0x308);
	writel_nbits(base, pll->i_fracdiv0, 8, 0, 0x308);
	writel_nbits(base, pll->i_feedfwrdgain, 4, 0, 0x30c);
	writel_nbits(base, pll->i_fracnen_h, 1, 16, 0x30c);
	writel_nbits(base, pll->i_sscstepsize0, 8, 0, 0x310);
	writel_nbits(base, pll->i_sscstepsize1, 2, 8, 0x310);
	writel_nbits(base, pll->i_sscsteplength0, 8, 16, 0x310);
	writel_nbits(base, pll->i_sscsteplength1, 2, 24, 0x310);
	writel_nbits(base, pll->i_sscstepnum, 3, 0, 0x314);
	writel_nbits(base, pll->i_ssctype, 2, 8, 0x314);
	writel_nbits(base, pll->i_sscen_h, 1, 16, 0x314);
	writel_nbits(base, pll->i_prop_coeff, 4, 0, 0x318);
	writel_nbits(base, pll->i_int_coeff, 5, 8, 0x318);
	writel_nbits(base, pll->i_gainctrl, 3, 16, 0x318);
	writel_nbits(base, pll->i_tdctargetcnt0, 8, 0, 0x320);
	writel_nbits(base, pll->i_tdctargetcnt1, 2, 8, 0x320);
	writel_nbits(base, pll->i_tdcsel, 2, 16, 0x320);
	writel_nbits(base, pll->i_tdccalsetupdeten, 1, 18, 0x320);
	writel_nbits(base, pll->i_lockthreshsel, 1, 0, 0x324);
	writel_nbits(base, pll->i_lockthresh, 3, 1, 0x324);
	writel_nbits(base, pll->i_afccntsel, 1, 8, 0x324);
	writel_nbits(base, pll->i_dcoditheren, 1, 9, 0x324);
	writel_nbits(base, pll->i_useidvdata, 1, 10, 0x324);
	writel_nbits(base, pll->i_pllwait_cntr, 3, 16, 0x324);
	writel_nbits(base, pll->cln_cnt_post, 8, 0, 0x114);
	writel_nbits(base, pll->cln_cnt_prep, 8, 0, 0x108);
	writel_nbits(base, pll->cln_hs_trail, 24, 8, 0xfc);
	writel_nbits(base, pll->cln_cnt_zero, 8, 0, 0x10c);
	writel_nbits(base, pll->cln_cnt_lpx, 8, 0, 0x104);
	writel_nbits(base, pll->cln_hs_exit, 8, 0, 0x100);
	writel_nbits(base, pll->dln_cnt_hs_exit, 8, 0, 0xf0);
	writel_nbits(base, pll->dln_cnt_hs_prep, 8, 0, 0xe4);
	writel_nbits(base, pll->dln_cnt_hs_zero, 8, 0, 0xe8);
	writel_nbits(base, pll->dln_cnt_hs_trail, 8, 0, 0xec);
	writel_nbits(base, pll->dln_cnt_lpx, 8, 0, 0xe0);

	return 0;
}

/**
 * Get D-PHY PPI status
 * @param phy pointer to structure which holds information about the d-phy
 * module
 * @param mask
 * @return status
 */
static u8 dbg_phy_is_pll_locked(struct regmap *base)
{
	u32 val;

	regmap_raw_read(base,
			REG_ANLG_PHY_G8_ANALOG_DSI_1_CTRL_REG0, &val, sizeof(val));

	return (val & BIT_ANLG_PHY_G8_ANALOG_DSI_1_O_STS_PLLOK ? 1 : 0);
}

static int dbg_phy_wait_pll_locked(struct regmap *base)
{
	unsigned i = 0;

	for (i = 0; i < 50000; i++) {
		if (dbg_phy_is_pll_locked(base))
			return 0;
		udelay(3);
	}

	pr_err("error: base pll can not be locked\n");

	return -1;
}

static void mphy_close(void)
{
	regmap_update_bits(ana_g8, REG_ANLG_PHY_G8_ANALOG_DSI_1_REG_SEL_CFG_0,
			   BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_1_I_RST_APB_N |
			   BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_1_I_RST_SYS_N |
			   BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_1_I_CTL_PHYFORCEPLL |
			   BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_1_I_CTL_TXSKEWCALHS,
			   BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_1_I_RST_APB_N |
			   BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_1_I_RST_SYS_N |
			   BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_1_I_CTL_PHYFORCEPLL |
			   BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_1_I_CTL_TXSKEWCALHS);
	mdelay(1);

	regmap_update_bits(ana_g8, REG_ANLG_PHY_G8_ANALOG_DSI_1_CTRL_REG0,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_TXLATCHEN_N,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_TXLATCHEN_N);
	mdelay(1);

	regmap_update_bits(ana_g8, REG_ANLG_PHY_G8_ANALOG_DSI_1_CTRL_REG0,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_LATCHRESET_B,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_LATCHRESET_B);
	mdelay(1);

	regmap_write(ana_g8,
		     REG_ANLG_PHY_G8_ANALOG_DSI_1_WRAP_GLUE_CTRL, 0x0000000f);
	mdelay(1);

	regmap_update_bits(ana_g8, REG_ANLG_PHY_G8_ANALOG_DSI_1_CTRL_REG0,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_RST_APB_N |
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_RST_SYS_N |
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_PHYFORCEPLL,
			   (u32)~(BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_RST_APB_N |
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_RST_SYS_N |
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_PHYFORCEPLL));

	regmap_update_bits(ana_g8, REG_ANLG_PHY_G8_ANALOG_DSI_1_CTRL_REG0,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_PGEN,
			   (u32)~BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_PGEN);
	mdelay(1);

	regmap_update_bits(ana_g8, REG_ANLG_PHY_G8_ANALOG_DSI_1_CTRL_REG0,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_FWEN_B,
			   (u32)~BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_FWEN_B);
	mdelay(1);
}

int dbg_phy_init(struct phy_ctx *phy)
{
	unsigned long base = phy->base;

	ana_g8 =  syscon_regmap_lookup_by_compatible("sprd,anlg_phy_g8");

	regmap_update_bits(ana_g8, REG_ANLG_PHY_G8_ANALOG_DSI_1_CTRL_REG0,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_TXLATCHEN_N,
			   (u32)~BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_TXLATCHEN_N);
	mdelay(1);

	regmap_update_bits(ana_g8, REG_ANLG_PHY_G8_ANALOG_DSI_1_CTRL_REG0,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_LATCHRESET_B,
			   (u32)~BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_LATCHRESET_B);
	mdelay(1);

	/*clear Ana REC 26M PD*/
	regmap_write(ana_g8,
		     REG_ANLG_PHY_G8_ANALOG_26M_RECEIVER_PHY_CTRL, 0x22002);
	mdelay(1);

	regmap_update_bits(ana_g8, REG_ANLG_PHY_G8_ANALOG_DSI_1_REG_SEL_CFG_0,
			   BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_1_I_RST_APB_N |
			   BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_1_I_RST_SYS_N |
			   BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_1_I_CTL_PHYFORCEPLL |
			   BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_1_I_CTL_TXSKEWCALHS,
			   BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_1_I_RST_APB_N |
			   BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_1_I_RST_SYS_N |
			   BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_1_I_CTL_PHYFORCEPLL |
			   BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_1_I_CTL_TXSKEWCALHS);
	mdelay(1);

	regmap_update_bits(ana_g8, REG_ANLG_PHY_G8_ANALOG_DSI_1_CTRL_REG0,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_RST_SYS_N,
			   (u32)~BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_RST_SYS_N);

	regmap_update_bits(ana_g8, REG_ANLG_PHY_G8_ANALOG_DSI_1_CTRL_REG0,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_RST_APB_N,
			   (u32)~BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_RST_APB_N);
	mdelay(1);

	regmap_update_bits(ana_g8, REG_ANLG_PHY_G8_ANALOG_DSI_1_CTRL_REG0,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_PGEN,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_PGEN);
	mdelay(1);

	regmap_update_bits(ana_g8, REG_ANLG_PHY_G8_ANALOG_DSI_1_CTRL_REG0,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_FWEN_B,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_FWEN_B);
	mdelay(1);

	regmap_update_bits(ana_g8, REG_ANLG_PHY_G8_ANALOG_DSI_1_CTRL_REG0,
			   BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_1_I_RST_SYS_N,
			   BIT_ANLG_PHY_G8_DBG_SEL_ANALOG_DSI_1_I_RST_SYS_N);
	mdelay(1);

	regmap_update_bits(ana_g8, REG_ANLG_PHY_G8_ANALOG_DSI_1_CTRL_REG0,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_RST_APB_N,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_RST_APB_N);
	mdelay(1);

	/*enable latch in AFE, HSTXBIASSEL*/
	phy_writel(base, 0x40000020, REG_DBG_APB_TX_DFX_TOP_CFG_REG);
	/*Regulator Reference Select*/
	phy_writel(base, 0x13, REG_DBG_APB_REF_CTL_REG);

	regmap_update_bits(ana_g8,
			   REG_ANLG_PHY_G8_ANALOG_DSI_1_CTRL_REG0,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_RST_SYS_N,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_RST_SYS_N);

	/*enable latch in AFE, HSTXBIASSEL*/
	phy_writel(base, 0x40000020, REG_DBG_APB_TX_DFX_TOP_CFG_REG);
	/*Regulator Reference Select*/
	phy_writel(base, 0x13, REG_DBG_APB_REF_CTL_REG);

	mphy_set_pll_reg(base, phy->freq);
	mdelay(1);

	regmap_update_bits(ana_g8, REG_ANLG_PHY_G8_ANALOG_DSI_1_CTRL_REG0,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_PHYFORCEPLL,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_PHYFORCEPLL);
	mdelay(1);

	phy_writel(base, 0x00000004, REG_DBG_APB_CLK_SRC_SELECT);

	/*enable DSI2 I DLN*/
	regmap_write(ana_g8,
		     REG_ANLG_PHY_G8_ANALOG_DSI_1_WRAP_GLUE_CTRL, 0x040f000f);
	mdelay(1);

	regmap_update_bits(ana_g8, REG_ANLG_PHY_G8_ANALOG_DSI_1_WRAP_GLUE_CTRL,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_DSI2_I_CTL_PHYFORCEPLL,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_DSI2_I_CTL_PHYFORCEPLL);
	mdelay(1);

	if (dbg_phy_wait_pll_locked(ana_g8))
		return -1;

	regmap_update_bits(ana_g8, REG_ANLG_PHY_G8_ANALOG_DSI_1_CTRL_REG0,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_TXLATCHEN_N,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_TXLATCHEN_N);
	mdelay(1);

	regmap_update_bits(ana_g8, REG_ANLG_PHY_G8_ANALOG_DSI_1_CTRL_REG0,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_LATCHRESET_B,
			   BIT_ANLG_PHY_G8_ANALOG_DSI_1_I_CTL_LATCHRESET_B);
	mdelay(1);

	return 0;
}

int dbg_phy_exit(struct phy_ctx *phy)
{
	mphy_close();
	return 0;
}

/*
 * Copyright (C) 2016 Spreadtrum Communications Inc.
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
#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <asm/irqflags.h>
#include <asm/cacheflush.h>
#include <linux/clockchips.h>
#include <linux/wakelock.h>
#include <asm/suspend.h>
#include <asm/barrier.h>
#include <linux/cpu_pm.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include "pm_arm.h"
#include "pm_debug_arm.h"
#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/io.h>
#include "pm_debug.h"
#include <asm/irqflags.h>
#ifdef CONFIG_SPRD_SIPC
#include <linux/sipc.h>
#endif


#define BITS_CHECK(index, __flag, __bits) do {\
	uint32_t mid ;\
	regmap_read(ds->syscon_map[index], __flag, &mid); \
	if (mid & __bits) \
		pr_info(#__bits " not 0\n"); \
	} while (0)

#define PRINTK_REG(index, _phys) do {\
	uint32_t mid ;\
	regmap_read(ds->syscon_map[index], _phys, &mid); \
	if (mid) \
		pr_info("##--"#_phys": 0x%08x\n", mid); \
	} while (0)


#define SAVE_GLOBAL_REG() do { \
		sprd_deep_bak_restore_apb(1); \
		sprd_deep_bak_ap_clk_reg(1); \
		sprd_deep_bak_restore_ahb(1); \
		sprd_deep_bak_restore_aon(1); \
		sprd_deep_bak_restore_ap_intc(1);\
		} while (0) \

#define RESTORE_GLOBAL_REG() do { \
		sprd_deep_bak_restore_apb(0); \
		sprd_deep_bak_ap_clk_reg(0); \
		sprd_deep_bak_restore_aon(0); \
		sprd_deep_bak_restore_ahb(0); \
		sprd_deep_bak_restore_ap_intc(0);\
		} while (0) \

struct deep_sleep_ap_clk_state {
	struct clk *clk;
	struct clk *parent;
	unsigned long current_rate;
};

enum {
	AP_APB = 0,
	AP_AHB,
	PMU_APB,
	AON_APB,
};

static struct sprd_deep {
	struct device_node *np;
	struct regmap *syscon_map[4];
	struct regmap *intc[5]; /*ap int0~3, aon intc*/

	struct ap_ahb_reg_bak ap_ahb_reg_saved;
	struct ap_apb_reg_bak ap_apb_reg_saved;

	int clk_cnt;
	struct deep_sleep_ap_clk_state *ap_clk;
} sprd_deep;

u32 ap_ahb_reg[] = {
		REG_AP_AHB_AHB_EB,
		REG_AP_AHB_CA7_CKG_DIV_CFG,
		REG_AP_AHB_MISC_CKG_EN,
		REG_AP_AHB_CA7_CKG_SEL_CFG,
		REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG,
		REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG,
		REG_AP_AHB_OTG_PHY_TUNE,
		REG_AP_AHB_OTG_PHY_TEST,
		REG_AP_AHB_OTG_PHY_CTRL,
		REG_AP_AHB_OTG_CTRL0,
		REG_AP_AHB_OTG_CTRL1,
		REG_AP_AHB_M0_LPC,
		REG_AP_AHB_M1_LPC,
		REG_AP_AHB_M2_LPC,
		REG_AP_AHB_M3_LPC,
		REG_AP_AHB_M4_LPC,
		REG_AP_AHB_M5_LPC,
		REG_AP_AHB_M6_LPC,
		REG_AP_AHB_M7_LPC,
		REG_AP_AHB_M8_LPC,
		REG_AP_AHB_M9_LPC,
		REG_AP_AHB_MAIN_LPC,
		REG_AP_AHB_S0_LPC,
		REG_AP_AHB_S1_LPC,
		REG_AP_AHB_S2_LPC,
		REG_AP_AHB_S3_LPC,
		REG_AP_AHB_S4_LPC,
};

static int
sprd_deep_get_ap_clk(struct sprd_deep *ds)
{
	int i, count = 0;
	struct deep_sleep_ap_clk_state *ap_clk;

	if (IS_ERR(ds->np)) {
		pr_err("%s, deepsleep_np is not initialized\n", __func__);
		return -ENODEV;
	}

	if (!of_get_property(ds->np, "clocks", &count)) {
		pr_info("%s, get clocks property fail !\n", __func__);
		return -EINVAL;
	}

	ds->clk_cnt = count >> 2;
	if (!ds->clk_cnt) {
		pr_err("%s, clk_cnt = 0\n", __func__);
		return -EINVAL;
	}

	pr_info("%s, get %d ap_clk\n", __func__, ds->clk_cnt);

	ap_clk = kcalloc(ds->clk_cnt,
		sizeof(struct deep_sleep_ap_clk_state), GFP_KERNEL);
	if (!ap_clk)
		return -ENOMEM;

	ds->ap_clk = ap_clk;

	for (i = 0; i < ds->clk_cnt; i++) {
		ap_clk[i].clk = of_clk_get(ds->np, i);
		if (IS_ERR(ds->ap_clk[i].clk)) {

			pr_info("%s,  the clock (index =%d) can't get\n",
				__func__, i);
			kfree(ap_clk);
			return -EINVAL;
		}
		pr_info("%s, get (index = %d) clock is ok !\n", __func__, i);
	}

	return 0;
}

static void sprd_deep_store_ap_clk(void)
{
	int i;
	struct sprd_deep *ds = &sprd_deep;
	struct deep_sleep_ap_clk_state *ap_clk = ds->ap_clk;

	pr_info("start %s\n", __func__);

	for (i = 1; i < ds->clk_cnt; i++) {
		ap_clk[i].parent = clk_get_parent(
				ap_clk[i].clk);
		ap_clk[i].current_rate = clk_get_rate(
				ap_clk[i].clk);
		pr_info("%s,bak clock index = %d\n", __func__, i);
	}

	pr_info("end %s\n", __func__);
}

static void sprd_deep_restore_ap_clk(void)
{
	int i;
	struct sprd_deep *ds = &sprd_deep;
	struct deep_sleep_ap_clk_state *ap_clk = ds->ap_clk;

	pr_info("start %s\n", __func__);

	for (i = 1; i < ds->clk_cnt; i++) {
		clk_set_parent(ap_clk[i].clk,
				ap_clk[0].clk);
		clk_set_parent(ap_clk[i].clk,
				ap_clk[i].parent);
		clk_set_rate(ap_clk[i].clk,
				ap_clk[i].current_rate);
		pr_info("%s,resume clock index = %d\n", __func__, i);
	}

	pr_info("end %s\n", __func__);
}

void sprd_deep_bak_restore_ahb(int bak)
{
	int i = 0, size = sizeof(ap_ahb_reg) / sizeof(u32);
	u32 *ahb_reg = (u32 *)&sprd_deep.ap_ahb_reg_saved;

	pr_err("sprd_deep size = %d\n", size);
	if (bak) {
		for (i = 0; i < size; i++)
			regmap_read(sprd_deep.syscon_map[AP_AHB],
				ap_ahb_reg[i], &ahb_reg[i]);
	} else {
		for (i = 0; i < size; i++)
			regmap_write(sprd_deep.syscon_map[AP_AHB],
				ap_ahb_reg[i], ahb_reg[i]);
	}
}

void sprd_deep_bak_restore_apb(int bak)
{
	if (bak) {
		regmap_read(sprd_deep.syscon_map[AP_APB],
				REG_AP_APB_APB_EB,
				&(sprd_deep.ap_apb_reg_saved.apb_eb));
		regmap_read(sprd_deep.syscon_map[AP_APB],
				REG_AP_APB_APB_MISC_CTRL,
				&(sprd_deep.ap_apb_reg_saved.apb_misc_ctrl));
	} else {
		regmap_write(sprd_deep.syscon_map[AP_APB],
				REG_AP_APB_APB_EB,
				sprd_deep.ap_apb_reg_saved.apb_eb);
		regmap_write(sprd_deep.syscon_map[AP_APB],
				REG_AP_APB_APB_MISC_CTRL,
				sprd_deep.ap_apb_reg_saved.apb_misc_ctrl);
	}
}

static void sprd_deep_bak_ap_clk_reg(int bak)
{
	if (bak)
		sprd_deep_store_ap_clk();
	else
		sprd_deep_restore_ap_clk();
}

static void sprd_deep_bak_restore_aon(int bak)
{
	static uint32_t apb_eb1, apb_eb0;

	if (bak) {
		regmap_read(
				sprd_deep.syscon_map[AON_APB],
				REG_AON_APB_APB_EB0, &apb_eb0);
		regmap_read(
				sprd_deep.syscon_map[AON_APB],
				REG_AON_APB_APB_EB1, &apb_eb1);
	} else {
		regmap_write(
				sprd_deep.syscon_map[AON_APB],
				REG_AON_APB_APB_EB0, apb_eb0);
		regmap_write(
				sprd_deep.syscon_map[AON_APB],
				REG_AON_APB_APB_EB1, apb_eb1);
	}
}

static void sprd_deep_bak_restore_ap_intc(int bak)
{
	int i = 0;
	static u32 intc_eb[4];

	if (bak) {
		for (i = 0; i < 4; i++)
			regmap_read(sprd_deep.intc[i],
				INTCV_IRQ_EN, &intc_eb[i]);
	} else {
		for (i = 0; i < 4; i++)
			regmap_write(sprd_deep.intc[i],
				INTCV_IRQ_EN, intc_eb[i]);
	}

}

void sprd_deep_pwr_domain_status2(struct sprd_deep *ds)
{
	uint32_t apb_pwrstatus2;

	regmap_read(ds->syscon_map[PMU_APB], REG_PMU_APB_PWR_STATUS2_DBG,
			&apb_pwrstatus2);

	pr_info("##--status of power domain 'PUB_SYS' :0x%08x\n",
		(apb_pwrstatus2 & BIT_PMU_APB_PD_PUB_SYS_STATE(15)) >> 24);
	pr_info("##--status of power domain 'WTLCP_LTE_P4' :0x%08x\n",
		(apb_pwrstatus2 & BIT_PMU_APB_PD_WTLCP_LTE_P4_STATE(15)) >> 16);
	pr_info("##--status of power domain 'WTLCP_LTE_P3' :0x%08x\n",
		(apb_pwrstatus2 & BIT_PMU_APB_PD_WTLCP_LTE_P3_STATE(15)) >> 12);
	pr_info("##--status of power domain 'PUBCP_SYS' :0x%08x\n",
		(apb_pwrstatus2 & BIT_PMU_APB_PD_PUBCP_SYS_STATE(15)) >> 8);
	pr_info("##--status of power domain 'WTLCP_SYS' :0x%08x\n",
		(apb_pwrstatus2 & BIT_PMU_APB_PD_WTLCP_SYS_STATE(15)) >> 4);
}
void sprd_deep_pwr_domain_status1(struct sprd_deep *ds)
{
	uint32_t apb_pwrstatus1;

	regmap_read(ds->syscon_map[PMU_APB], REG_PMU_APB_PWR_STATUS1_DBG,
			&apb_pwrstatus1);

	pr_info("##--status of power domain 'WTLCP_TD' :0x%08x\n",
		(apb_pwrstatus1 & BIT_PMU_APB_PD_WTLCP_TD_STATE(15)) >> 28);
	pr_info("##--status of power domain 'WTLCP_LTE_P1' :0x%08x\n",
		(apb_pwrstatus1 & BIT_PMU_APB_PD_WTLCP_LTE_P1_STATE(15)) >> 24);
	pr_info("##--status of power domain 'WTLCP_LTE_P2' :0x%08x\n",
		(apb_pwrstatus1 & BIT_PMU_APB_PD_WTLCP_LTE_P2_STATE(15)) >> 20);
	pr_info("##--status of power domain 'WTLCP_LDSP' :0x%08x\n",
		(apb_pwrstatus1 & BIT_PMU_APB_PD_WTLCP_LDSP_STATE(15)) >> 12);
	pr_info("##--status of power domain 'WTLCP_TGDSP' :0x%08x\n",
		(apb_pwrstatus1 & BIT_PMU_APB_PD_WTLCP_TGDSP_STATE(15)) >> 8);
	pr_info("##--status of power domain 'HU3GE_A' :0x%08x\n",
		(apb_pwrstatus1 & BIT_PMU_APB_PD_WTLCP_HU3GE_A_STATE(15)) >> 4);
	pr_info("##--status of power domain 'HU3GE_B' :0x%08x\n",
		apb_pwrstatus1 & BIT_PMU_APB_PD_WTLCP_HU3GE_B_STATE(15));
}

void sprd_deep_pwr_domain_status0(struct sprd_deep *ds)
{
	uint32_t apb_pwrstatus0;

	regmap_read(ds->syscon_map[PMU_APB], REG_PMU_APB_PWR_STATUS0_DBG,
			&apb_pwrstatus0);

	pr_info("##--status of power domain 'MM_TOP' :0x%08x\n",
		(apb_pwrstatus0 & BIT_PMU_APB_PD_MM_TOP_STATE(15)) >> 28);
	pr_info("##--status of power domain 'GPU_TOP' :0x%08x\n",
		(apb_pwrstatus0 & BIT_PMU_APB_PD_GPU_TOP_STATE(15)) >> 24);
	pr_info("##--status of power domain 'AP_SYS' :0x%08x\n",
		(apb_pwrstatus0 & BIT_PMU_APB_PD_AP_SYS_STATE(15)) >> 20);
	pr_info("##--status of power domain 'CA7_C3' :0x%08x\n",
		(apb_pwrstatus0 & BIT_PMU_APB_PD_CA7_C3_STATE(15)) >> 16);
	pr_info("##--status of power domain 'CA7_C2' :0x%08x\n",
		(apb_pwrstatus0 & BIT_PMU_APB_PD_CA7_C2_STATE(15)) >> 12);
	pr_info("##--status of power domain 'CA7_C1' :0x%08x\n",
		(apb_pwrstatus0 & BIT_PMU_APB_PD_CA7_C1_STATE(15)) >> 8);
	pr_info("##--status of power domain 'CA7_C0' :0x%08x\n",
		(apb_pwrstatus0 & BIT_PMU_APB_PD_CA7_C0_STATE(15)) >> 4);
	pr_info("##--status of power domain 'CA7_TOP' :0x%08x\n",
		apb_pwrstatus0 & BIT_PMU_APB_PD_CA7_TOP_STATE(15));
}

void sprd_deep_pwr_domain_status(struct sprd_deep *ds)
{
	pr_info("#####PWR_STATUS0 POWER DOMAIN#####\n");
	sprd_deep_pwr_domain_status0(ds);
	pr_info("#####PWR_STATUS1 POWER DOMAIN#####\n");
	sprd_deep_pwr_domain_status1(ds);
	pr_info("#####PWR_STATUS2 POWER DOMAIN#####\n");
	sprd_deep_pwr_domain_status2(ds);
}


void sprd_deep_print_last_reg(struct sprd_deep *ds)
{
	pr_info("ap ahb reg\n");
	PRINTK_REG(AP_AHB, REG_AP_AHB_AHB_EB);
	PRINTK_REG(AP_AHB, REG_AP_AHB_MCU_PAUSE);
	PRINTK_REG(AP_AHB, REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG);
	PRINTK_REG(AP_AHB, REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG);
	PRINTK_REG(AP_AHB, REG_AP_AHB_CA7_STANDBY_STATUS);
	pr_info("ap apb reg\n");
	PRINTK_REG(AP_APB, REG_AP_APB_APB_EB);
	pr_info("aon apb reg\n");
	PRINTK_REG(AON_APB, REG_AON_APB_APB_EB0);
	PRINTK_REG(AON_APB, REG_AON_APB_APB_EB1);
	PRINTK_REG(AON_APB, REG_AON_APB_APB_EB2);
	PRINTK_REG(AON_APB, REG_AON_APB_PWR_CTRL);
	PRINTK_REG(AON_APB, REG_AON_APB_BB_BG_CTRL);
	PRINTK_REG(AON_APB, REG_AON_APB_MPLL_CFG1);
	PRINTK_REG(AON_APB, REG_AON_APB_DPLL_CFG1);
	PRINTK_REG(AON_APB, REG_AON_APB_MPLL_CFG2);
	PRINTK_REG(AON_APB, REG_AON_APB_DPLL_CFG2);
	pr_info("aon pmu status reg\n");
	PRINTK_REG(PMU_APB, REG_PMU_APB_PD_PUB_SYS_CFG);
	PRINTK_REG(PMU_APB, REG_PMU_APB_PD_MM_TOP_CFG);
	PRINTK_REG(PMU_APB, REG_PMU_APB_PD_AP_SYS_CFG);
	PRINTK_REG(PMU_APB, REG_PMU_APB_CP_SLP_STATUS_DBG0);
	PRINTK_REG(PMU_APB, REG_PMU_APB_DDR_SLEEP_CTRL);
	PRINTK_REG(PMU_APB, REG_PMU_APB_SLEEP_STATUS);
	PRINTK_REG(PMU_APB, REG_PMU_APB_SLEEP_CTRL);
	pr_info("#####PWR_STATUS reg #####\n");
	PRINTK_REG(PMU_APB, REG_PMU_APB_PWR_STATUS0_DBG);
	PRINTK_REG(PMU_APB, REG_PMU_APB_PWR_STATUS1_DBG);
	PRINTK_REG(PMU_APB, REG_PMU_APB_PWR_STATUS2_DBG);
	sprd_deep_pwr_domain_status(ds);
}

void sprd_deep_print_int_status(struct sprd_deep *ds)
{
	int i = 0;
	uint32_t intc_bits[5], intc_irq_raw[5], intc_irq_en[5];
	struct regmap **map = ds->intc;

	for (i = 0; i < 5; i++) {
		regmap_read(map[i], INTCV_IRQ_MSKSTS, &intc_bits[i]);
		regmap_read(map[i], INTCV_IRQ_RAW, &intc_irq_raw[i]);
		regmap_read(map[i], INTCV_IRQ_EN, &intc_irq_en[i]);
		pr_info("INTC%d mask:0x%08x raw:0x%08x en:0x%08x\n",
			i, intc_bits[i], intc_irq_raw[i], intc_irq_en[i]);
	}

}

void sprd_deep_ap_ahb_check(struct sprd_deep *ds)
{
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_USB_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_DMA_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_SDIO0_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_SDIO1_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_SDIO2_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_EMMC_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_CE_SEC_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_CE_PUB_EB);
}

void sprd_deep_ap_apb_check(struct sprd_deep *ds)
{
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_SIM0_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_IIS0_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_IIS1_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_IIS2_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_IIS3_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_I2C0_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_I2C1_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_I2C2_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_I2C3_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_I2C4_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_UART0_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_UART1_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_UART2_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_UART3_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_UART4_EB);
}

void sprd_deep_aon_apb_check(struct sprd_deep *ds)
{
	BITS_CHECK(AON_APB, REG_AON_APB_CLK_EB0,
		BIT_AON_APB_AP_HS_SPI_EB);
	BITS_CHECK(AON_APB, REG_AON_APB_APB_EB0,
		BIT_AON_APB_CA7_DAP_EB);
	BITS_CHECK(AON_APB, REG_AON_APB_APB_EB2,
		BIT_AON_APB_AP_DAP_EB);
}

void sprd_deep_condition_check(struct sprd_deep *ds)
{
	pr_info("#####Enter Deepsleep  Condition Check!######\n");
	pr_info("#####Enter AP_AHB  Condition Check!#####\n");
	sprd_deep_ap_ahb_check(ds);
	pr_info("#####Enter AP_APB  Condition Check!#####\n");
	sprd_deep_ap_apb_check(ds);
	pr_info("#####Enter AON_APB  Condition Check!#####\n");
	sprd_deep_aon_apb_check(ds);
}

static void sprd_deep_print_status(void)
{
	struct sprd_deep *ds = &sprd_deep;

	sprd_deep_condition_check(ds);
	sprd_deep_print_last_reg(ds);
	sprd_deep_print_int_status(ds);
}

#define   SPRD_HARD_INT_NUM		160
#define   INT_CHANNEL_PER_INTC	32

static u32 sprd_hard_irq[SPRD_HARD_INT_NUM] = { 0, };

static void hard_irq_reset(void)
{
	int i = SPRD_HARD_INT_NUM - 1;

	do {
		sprd_hard_irq[i] = 0;
	} while (--i >= 0);
}

static void parse_hard_irq(unsigned long val, unsigned long intc)
{
	int i;

	for (i = 0; i < INT_CHANNEL_PER_INTC; i++) {
		if (test_and_clear_bit(i, &val))
			sprd_hard_irq[INT_CHANNEL_PER_INTC * intc + i]++;
	}
}

void sprd_deep_print_wakeup_info(void)
{
	int i;
	uint32_t intc_bits[5], intc_irq_raw[5], intc_irq_en[5];
	struct sprd_deep *ds = &sprd_deep;
	struct regmap **map = ds->intc;

	hard_irq_reset();
	for (i = 0; i < 5; i++) {
		regmap_read(map[i], INTCV_IRQ_MSKSTS, &intc_bits[i]);
		regmap_read(map[i], INTCV_IRQ_RAW, &intc_irq_raw[i]);
		regmap_read(map[i], INTCV_IRQ_EN, &intc_irq_en[i]);

		if ( i < 4 && intc_bits[i] ) {
			pr_info("wakeup by INTC%d !!"
				" mask:0x%08x raw:0x%08x en:0x%08x\n",
			i, intc_bits[i], intc_irq_raw[i], intc_irq_en[i]);
			parse_hard_irq(intc_bits[i], i);
		}
	}

	if (intc_bits[0]) {
		if (sprd_hard_irq[31])
			pr_info("wake up by ap syst!\n ");
		if (sprd_hard_irq[30])
			pr_info("wake up by aon syst\n");
		if (sprd_hard_irq[29])
			pr_info("wake up by ap tmr0\n");
		if (sprd_hard_irq[28])
			pr_info("wake up by aon tmr\n");
		if (sprd_hard_irq[26])
			pr_info("wake up by thm\n");
		if (sprd_hard_irq[25])
			pr_info("wake up by adi\n");
		if (sprd_hard_irq[24])
			pr_info("wake up by vbc ad23\n");
		if (sprd_hard_irq[23])
			pr_info("wake up by vbc ad01!\n");
		if (sprd_hard_irq[22])
			pr_info("wake up by vbc da\n");
		if (sprd_hard_irq[21])
			pr_info("wake up by vbc_afifo_err\n");
		if (sprd_hard_irq[20])
			pr_info("wake up by aud\n");
	}
	if (intc_bits[1]) {
		if (sprd_hard_irq[63])
			pr_info("wake up by busmon2\n");
		if (sprd_hard_irq[62])
			pr_info("wake up by busmon1\n");
		if (sprd_hard_irq[61])
			pr_info("wake up by busmon0\n");
		if (sprd_hard_irq[60])
			pr_info("wake up by emmc\n");
		if (sprd_hard_irq[56])
			pr_info("wake up by nfc\n");
		if (sprd_hard_irq[55])
			pr_info("wake up by usb\n");
		if (sprd_hard_irq[54])
			pr_info("wake up by hsic\n");
		if (sprd_hard_irq[51])
			pr_info("wake up by gsp\n");
		if (sprd_hard_irq[50])
			pr_info("wake up by dma\n");
		if (sprd_hard_irq[39])
			pr_info("wake up by gpu\n");
		if (sprd_hard_irq[38])
			pr_info("wake up by ana\n");
		if (sprd_hard_irq[37])
			pr_info("wake up by eic\n");
		if (sprd_hard_irq[36])
			pr_info("wake up by kpd\n");
		if (sprd_hard_irq[35])
			pr_info("wake up by gpio\n");
	}
	if (intc_bits[2]) {
		if (sprd_hard_irq[86])
			pr_info("wake up by pub_busmon\n");
		if (sprd_hard_irq[84])
			pr_info("wake up by cp1_wdg\n");
		if (sprd_hard_irq[83])
			pr_info("wake up by cp0_wdg\n");
		if (sprd_hard_irq[70])
			pr_info("wake up by aon_dma\n");
		if (sprd_hard_irq[69]) {
			pr_info("wake up by mbox_tar_ap\n");
#ifdef CONFIG_SPRD_SIPC
			sipc_set_wakeup_flag();
#endif
		}
		if (sprd_hard_irq[68])
			pr_info("wake up by mbox_src_ap\n");
		if (sprd_hard_irq[67])
			pr_info("wake up by djtag\n");
		if (sprd_hard_irq[66])
			pr_info("wake up by drm\n");
	}
	if (intc_bits[3]) {
		if (sprd_hard_irq[124])
			pr_info("wake up by ca7_wtg\n");
		if (sprd_hard_irq[123])
			pr_info("wake up by ap_wdg\n");
		if (sprd_hard_irq[122])
			pr_info("wake up by avs\n");
		if (sprd_hard_irq[121])
			pr_info("wake up by ap_tmr4\n");
		if (sprd_hard_irq[120])
			pr_info("wake up by ap_tmr3\n");
		if (sprd_hard_irq[119])
			pr_info("wake up by ap_tmr2\n");
		if (sprd_hard_irq[118])
			pr_info("wake up by ap_tmr1\n");
	}
	pr_info("AON INTC mask:0x%08x raw:0x%08x en:0x%08x\n",
		intc_bits[4], intc_irq_raw[4], intc_irq_en[4]);
}

static int
sprd_deep_intc_vaddr_init(struct sprd_deep *ds)
{
	int i, count = 0;
	struct device_node *syscon_np;

	if (!of_get_property(ds->np, "sprd,sys-intc", &count)) {
		pr_info("%s, get sprd,sys-intc property fail !\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < count >> 2; i++) {

		syscon_np = of_parse_phandle(ds->np,  "sprd,sys-intc", i);
		if (!syscon_np)
			return -ENODEV;

		ds->intc[i] = syscon_node_to_regmap(syscon_np);
		of_node_put(syscon_np);

		if (IS_ERR_OR_NULL(ds->intc[i])) {
			pr_err("%s,failed to find sys intc[%d]\n",
					__func__, i);
			return -ENODEV;
		}
	}

	return 0;
}

static int
sprd_deep_glb_vaddr_init(struct sprd_deep *ds)
{
	int i, count = 0;
	struct device_node *syscon_np;

	if (!of_get_property(ds->np, "sprd,sys-base", &count)) {
		pr_info("%s, get sprd,sys-base property fail !\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < count >> 2; i++) {

		syscon_np = of_parse_phandle(ds->np,  "sprd,sys-base", i);
		if (!syscon_np)
			return -ENODEV;

		ds->syscon_map[i] = syscon_node_to_regmap(syscon_np);
		of_node_put(syscon_np);

		if (IS_ERR_OR_NULL(ds->syscon_map[i])) {
			pr_err("%s,failed to find deepsleep_syscon base[%d]\n",
					__func__, i);
			return -ENODEV;
		}
	}

	return 0;
}

static int sprd_deep_init_glb_regmap(struct sprd_deep *ds)
{
	int ret = 0;

	pr_info("###### deepsleep_init_regmap start ###\n");

	ds->np = of_find_node_by_name(NULL, "deep-sleep");
	if (!ds->np) {
		pr_err("%s, failed to find deepsleep_np\n", __func__);
		return -ENODEV;
	}

	ret = sprd_deep_glb_vaddr_init(ds);
	if (ret)
		return ret;

	ret = sprd_deep_intc_vaddr_init(ds);
	if  (ret)
		return ret;

	pr_info("###### deepsleep_init_regmap end ###\n");
	return 0;
}

static int
sprd_deep_pm_notifier(struct notifier_block *self, unsigned long cmd, void *v)
{
	switch (cmd) {
	case CPU_CLUSTER_PM_ENTER:
		SAVE_GLOBAL_REG();
		sprd_deep_print_status();
		break;
	case CPU_CLUSTER_PM_ENTER_FAILED:
	case CPU_CLUSTER_PM_EXIT:
		RESTORE_GLOBAL_REG();
		sprd_deep_print_wakeup_info();
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block sprd_pm_notifier_block = {
	.notifier_call = sprd_deep_pm_notifier,
};

static int __init sprd_deep_init(void)
{
	struct sprd_deep *ds = &sprd_deep;
	int ret = sprd_deep_init_glb_regmap(ds);

	if  (ret)
		return ret;

	ret = sprd_deep_get_ap_clk(ds);
	if (ret) {
		pr_err("%s,failed to find ap clk\n", __func__);
		return ret;
	}

	cpu_pm_register_notifier(&sprd_pm_notifier_block);
	return 0;
}

subsys_initcall(sprd_deep_init);

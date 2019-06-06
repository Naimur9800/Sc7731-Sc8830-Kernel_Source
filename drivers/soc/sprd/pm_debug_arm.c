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

#include <asm/irqflags.h>
#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/kthread.h>
#include <linux/debugfs.h>
#ifdef CONFIG_SPRD_SIPC
#include <linux/sipc.h>
#endif
#include "pm_debug_arm.h"
static struct dentry *dentry_debug_root;
static unsigned int is_print_sleep_mode;
static unsigned int is_print_linux_clock = 1;
static unsigned int is_print_modem_clock = 1;
static unsigned int is_print_irq = 1;
static unsigned int is_print_wakeup = 1;
static unsigned int is_print_irq_runtime;
static unsigned int is_print_time = 1;
static unsigned int print_thread_enable = 1;
static unsigned int print_thread_interval = 30;
static unsigned int core_time;
static unsigned int mcu_time;
static unsigned int lit_time;
static unsigned int deep_time_successed;
static unsigned int deep_time_failed;
static unsigned int sleep_time;
static u32 sprd_hard_irq[SPRD_HARD_INTERRUPT_NUM] = { 0, };
static u32 sprd_irqs_sts[SPRD_INTC_NUM] = { 0, };
static int is_wakeup;
static u32 irq_status;
static int sleep_mode = SLP_MODE_NON;
static char *sleep_mode_str[] = {
		"[ARM]",
		"[MCU]",
		"[DEP]",
		"[LIT]",
		"[NON]" };

static void hard_irq_reset(void)
{
	int i = SPRD_HARD_INTERRUPT_NUM - 1;

	do {
		sprd_hard_irq[i] = 0;
	} while (--i >= 0);
}

static void parse_hard_irq(unsigned long val, unsigned long intc)
{
	int i;

	if (intc == 0) {
		for (i = 0; i < SPRD_HARD_INT_NUM_EACH_INTC; i++) {
			if (test_and_clear_bit(i, &val))
				sprd_hard_irq[i]++;
		}
	}
	if (intc == 1) {
		for (i = 0; i < SPRD_HARD_INT_NUM_EACH_INTC; i++) {
			if (test_and_clear_bit(i, &val))
				sprd_hard_irq[32 + i]++;
		}
	}
	if (intc == 2) {
		for (i = 0; i < SPRD_HARD_INT_NUM_EACH_INTC; i++) {
			if (test_and_clear_bit(i, &val))
				sprd_hard_irq[64 + i]++;
		}
	}
	if (intc == 3) {
		for (i = 0; i < SPRD_HARD_INT_NUM_EACH_INTC; i++) {
			if (test_and_clear_bit(i, &val))
				sprd_hard_irq[96 + i]++;
		}
	}
	if (intc == 4) {
		for (i = 0; i < SPRD_HARD_INT_NUM_EACH_INTC; i++) {
			if (test_and_clear_bit(i, &val))
				sprd_hard_irq[128 + i]++;
		}
	}
}

void hard_irq_set(void)
{
	if (is_intc_base_vaddr_ok) {
		pr_err("%s,INTC register base vaddr is not ok!\n", __func__);
		return;
	}

	regmap_read(deepsleep_syscon_ap_intc0, INTCV_IRQ_MSKSTS,
			&sprd_irqs_sts[0]);
	regmap_read(deepsleep_syscon_ap_intc0, INTCV_FIQ_STS,
			&sprd_irqs_sts[1]);
	regmap_read(deepsleep_syscon_ap_intc1, INTCV_IRQ_MSKSTS,
			&sprd_irqs_sts[2]);
	regmap_read(deepsleep_syscon_ap_intc1, INTCV_FIQ_STS,
			&sprd_irqs_sts[3]);
	regmap_read(deepsleep_syscon_ap_intc2, INTCV_IRQ_MSKSTS,
			&sprd_irqs_sts[4]);
	regmap_read(deepsleep_syscon_ap_intc2, INTCV_FIQ_STS,
			&sprd_irqs_sts[5]);
	regmap_read(deepsleep_syscon_ap_intc3, INTCV_IRQ_MSKSTS,
			&sprd_irqs_sts[6]);
	regmap_read(deepsleep_syscon_ap_intc3, INTCV_FIQ_STS,
			&sprd_irqs_sts[7]);
	regmap_read(deepsleep_syscon_aon_intc, INTCV_IRQ_MSKSTS,
			&sprd_irqs_sts[8]);
	regmap_read(deepsleep_syscon_aon_intc, INTCV_FIQ_STS,
			&sprd_irqs_sts[9]);
	regmap_read(deepsleep_syscon_ap_intc0, INTCV_IRQ_RAW, &irq_status);
	parse_hard_irq(irq_status, 0);
	regmap_read(deepsleep_syscon_ap_intc1, INTCV_IRQ_RAW, &irq_status);
	parse_hard_irq(irq_status, 1);
	regmap_read(deepsleep_syscon_ap_intc2, INTCV_IRQ_RAW, &irq_status);
	parse_hard_irq(irq_status, 2);
	regmap_read(deepsleep_syscon_ap_intc3, INTCV_IRQ_RAW, &irq_status);
	parse_hard_irq(irq_status, 3);
	regmap_read(deepsleep_syscon_aon_intc, INTCV_IRQ_RAW, &irq_status);
	parse_hard_irq(irq_status, 4);
}
void print_int_status(void)
{
	uint32_t intc0_bits = 0, intc1_bits = 0, intc2_bits = 0, intc3_bits = 0,
			intc4_bits = 0;
	uint32_t intc0_irq_raw = 0, intc1_irq_raw = 0, intc2_irq_raw = 0,
			intc3_irq_raw = 0, intc4_irq_raw = 0;
	uint32_t intc0_irq_en = 0, intc1_irq_en = 0, intc2_irq_en = 0,
			intc3_irq_en = 0, intc4_irq_en = 0;

	if (is_intc_base_vaddr_ok) {
		pr_err("%s,INTC register base vaddr is not ok!\n", __func__);
		return;
	}

	if (deepsleep_syscon_ap_intc0 == NULL
			|| deepsleep_syscon_ap_intc1 == NULL
			|| deepsleep_syscon_ap_intc2 == NULL
			|| deepsleep_syscon_ap_intc3 == NULL
			|| deepsleep_syscon_aon_intc == NULL) {
		pr_err("%s,failed find to INTC base vaddr\n", __func__);
		return;
	}
	regmap_read(deepsleep_syscon_ap_intc0, INTCV_IRQ_MSKSTS, &intc0_bits);
	regmap_read(deepsleep_syscon_ap_intc1, INTCV_IRQ_MSKSTS, &intc1_bits);
	regmap_read(deepsleep_syscon_ap_intc2, INTCV_IRQ_MSKSTS, &intc2_bits);
	regmap_read(deepsleep_syscon_ap_intc3, INTCV_IRQ_MSKSTS, &intc3_bits);
	regmap_read(deepsleep_syscon_aon_intc, INTCV_IRQ_MSKSTS, &intc4_bits);

	regmap_read(deepsleep_syscon_ap_intc0, INTCV_IRQ_RAW, &intc0_irq_raw);
	regmap_read(deepsleep_syscon_ap_intc1, INTCV_IRQ_RAW, &intc1_irq_raw);
	regmap_read(deepsleep_syscon_ap_intc2, INTCV_IRQ_RAW, &intc2_irq_raw);
	regmap_read(deepsleep_syscon_ap_intc3, INTCV_IRQ_RAW, &intc3_irq_raw);
	regmap_read(deepsleep_syscon_aon_intc, INTCV_IRQ_RAW, &intc4_irq_raw);

	regmap_read(deepsleep_syscon_ap_intc0, INTCV_IRQ_EN, &intc0_irq_en);
	regmap_read(deepsleep_syscon_ap_intc1, INTCV_IRQ_EN, &intc1_irq_en);
	regmap_read(deepsleep_syscon_ap_intc2, INTCV_IRQ_EN, &intc2_irq_en);
	regmap_read(deepsleep_syscon_ap_intc3, INTCV_IRQ_EN, &intc3_irq_en);
	regmap_read(deepsleep_syscon_aon_intc, INTCV_IRQ_EN, &intc4_irq_en);

	pr_info("INTC0 mask:0x%08x raw:0x%08x en:0x%08x\n", intc0_bits,
			intc0_irq_raw, intc0_irq_en);
	pr_info("INTC1 mask:0x%08x raw:0x%08x en:0x%08x\n", intc1_bits,
			intc1_irq_raw, intc1_irq_en);
	pr_info("INTC2 mask:0x%08x raw:0x%08x en:0x%08x\n", intc2_bits,
			intc2_irq_raw, intc2_irq_en);
	pr_info("INTC3 mask:0x%08x raw:0x%08x en:0x%08x\n", intc3_bits,
			intc3_irq_raw, intc3_irq_en);
	pr_info("AON_INTC mask:0x%08x raw:0x%08x en:0x%08x\n", intc4_bits,
			intc4_irq_raw, intc4_irq_en);
}

void print_hard_irq_inloop(int ret)
{
	if (sprd_irqs_sts[0] != 0)
		pr_info("%c#:INTC0: %08x\n", ret ? 'S' : 'F',
				sprd_irqs_sts[0]);
	if (sprd_irqs_sts[1] != 0)
		pr_info("%c#:INTC0 FIQ: %08x\n", ret ? 'S' : 'F',
				sprd_irqs_sts[1]);
	if (sprd_irqs_sts[2] != 0)
		pr_info("%c#:INTC1: %08x\n", ret ? 'S' : 'F',
				sprd_irqs_sts[2]);
	if (sprd_irqs_sts[3] != 0)
		pr_info("%c#:INTC1 FIQ: %08x\n", ret ? 'S' : 'F',
				sprd_irqs_sts[3]);
	if (sprd_irqs_sts[4] != 0)
		pr_info("%c#:INTC2: %08x\n", ret ? 'S' : 'F',
				sprd_irqs_sts[4]);
	if (sprd_irqs_sts[5] != 0)
		pr_info("%c#:INTC2 FIQ: %08x\n", ret ? 'S' : 'F',
				sprd_irqs_sts[5]);
	if (sprd_irqs_sts[6] != 0)
		pr_info("%c#:INTC3: %08x\n", ret ? 'S' : 'F',
				sprd_irqs_sts[6]);
	if (sprd_irqs_sts[7] != 0)
		pr_info("%c#:INTC3 FIQ: %08x\n", ret ? 'S' : 'F',
				sprd_irqs_sts[7]);
	if (sprd_irqs_sts[8] != 0)
		pr_info("%c#:INTC4: %08x\n", ret ? 'S' : 'F',
				sprd_irqs_sts[8]);
	if (sprd_irqs_sts[9] != 0)
		pr_info("%c#:INTC4 FIQ: %08x\n", ret ? 'S' : 'F',
				sprd_irqs_sts[9]);
	if (sprd_irqs_sts[0]) {
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
	if (sprd_irqs_sts[2]) {
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
	if (sprd_irqs_sts[4]) {
		if (sprd_hard_irq[84])
			pr_info("wake up by cp1_wdg\n");
		if (sprd_hard_irq[83])
			pr_info("wake up by cp0_wdg\n");
		if (sprd_hard_irq[70])
			pr_info("wake up by aon_dma\n");
		if (sprd_hard_irq[69]) {
#ifdef CONFIG_SPRD_SIPC
			sipc_set_wakeup_flag();
#endif
			pr_info("wake up by mbox_tar_ap\n");
		}
		if (sprd_hard_irq[68])
			pr_info("wake up by mbox_src_ap\n");
		if (sprd_hard_irq[67])
			pr_info("wake up by djtag\n");
		if (sprd_hard_irq[66])
			pr_info("wake up by drm\n");
	}
	if (sprd_irqs_sts[6]) {
		if (sprd_hard_irq[126])
			pr_info("wake up by zipenc\n");
		if (sprd_hard_irq[125])
			pr_info("wake up by zipdec\n");
		if (sprd_hard_irq[124])
			pr_info("wake up by ca7_wtg\n");
		if (sprd_hard_irq[123])
			pr_info("wake up by ap_wdg\n");
		if (sprd_hard_irq[121])
			pr_info("wake up by ap_tmr4\n");
		if (sprd_hard_irq[120])
			pr_info("wake up by ap_tmr3\n");
		if (sprd_hard_irq[119])
			pr_info("wake up by ap_tmr2\n");
		if (sprd_hard_irq[118])
			pr_info("wake up by ap_tmr1\n");
	}
	if (sprd_irqs_sts[8]) {
		if (sprd_hard_irq[159])
			pr_info("wake up by eic\n");
		if (sprd_hard_irq[140]) {
#ifdef CONFIG_SPRD_SIPC
			sipc_set_wakeup_flag();
#endif
			pr_info("wake up by mbox_tar_ap\n");
		}
		if (sprd_hard_irq[139])
			pr_info("wake up by mbox_tar_arm7\n");
		if (sprd_hard_irq[138])
			pr_info("wake up by pub_busmon\n");
	}
}

static void print_hard_irq(void)
{
	int i = SPRD_HARD_INTERRUPT_NUM - 1;

	if (!is_print_irq)
		return;
	do {
		if (sprd_hard_irq[i] != 0)
			pr_info("##: sprd_hard_irq[%d] = %d.\n",
				i, sprd_hard_irq[i]);
	} while (--i >= 0);
}

static void print_irq(void)
{
}

void irq_wakeup_set(void)
{
	is_wakeup = 1;
}

void time_add(unsigned int time, int ret)
{
	switch (sleep_mode) {
	case SLP_MODE_ARM:
		core_time += time;
		break;
	case SLP_MODE_MCU:
		mcu_time += time;
		break;
	case SLP_MODE_LIT:
		lit_time += time;
		break;
	case SLP_MODE_DEP:
		if (ret)
			deep_time_successed += time;
		else
			deep_time_failed += time;
		break;
	default:
		break;
	}
}

void time_statisic_begin(void)
{
	core_time = 0;
	mcu_time = 0;
	lit_time = 0;
	deep_time_successed = 0;
	deep_time_failed = 0;
	hard_irq_reset();
}

void time_statisic_end(void)
{
}

static void print_time(void)
{
	if (!is_print_time)
		return;
	pr_info(
			"time statisics : sleep_time=%d, core_time=%d, mcu_time=%d, lit_time=%d, deep_sus=%d, dep_fail=%d\n",
			sleep_time, core_time, mcu_time, lit_time,
			deep_time_successed, deep_time_failed);
}

void set_sleep_mode(int sm)
{
	int is_print = (sm == sleep_mode);

	sleep_mode = sm;
	if (is_print_sleep_mode == 0 || is_print)
		return;
	switch (sm) {
	case SLP_MODE_ARM:
		pr_info("\n[ARM]\n");
		break;
	case SLP_MODE_MCU:
		pr_info("\n[MCU]\n");
		break;
	case SLP_MODE_LIT:
		pr_info("\n[LIT]\n");
		break;
	case SLP_MODE_DEP:
		pr_info("\n[DEP]\n");
		break;
	default:
		pr_info("\nNONE\n");
	}
}
void clr_sleep_mode(void)
{
	sleep_mode = SLP_MODE_NON;
}
void print_statisic(void)
{
	print_time();
	print_hard_irq();
	print_irq();
	if (is_print_wakeup) {
		pr_info("###wake up form %s : %08x\n",
				sleep_mode_str[sleep_mode],
				sprd_irqs_sts[0]);
		pr_info("###wake up form %s : %08x\n",
				sleep_mode_str[sleep_mode],
				sprd_irqs_sts[1]);
	}
}
#ifdef PM_PRINT_ENABLE
static struct wake_lock messages_wakelock;
#endif

#define PM_PRINT_ENABLE

static uint32_t ahb_eb, ap_apb_eb, sys_force_sleep, mcu_pause,
		ap_sys_auto_sleep_cfg, ca7_standby_status;
static uint32_t aon_apb_eb0, aon_apb_eb1, aon_apb_eb2, pwr_ctrl, bb_bg_ctrl,
		mpll_cfg1, dpll_cfg1, mpll_cfg2, dpll_cfg2;
static uint32_t pd_pub_sys, pd_mm_top_cfg, pd_ap_sys_cfg, cp_slp_status0,
		apb_pwrstatus0, apb_pwrstatus1, apb_pwrstatus2;
static uint32_t apb_slp_status, sleep_ctrl, ddr_slp_cfg;

static void read_reg_value(void)
{

	regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_AHB_EB, &ahb_eb);
	regmap_read(deepsleep_syscon_ap_apb, REG_AP_APB_APB_EB, &ap_apb_eb);
	regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG,
			&sys_force_sleep);
	regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_MCU_PAUSE, &mcu_pause);
	regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG,
			&ap_sys_auto_sleep_cfg);
	regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_CA7_STANDBY_STATUS,
			&ca7_standby_status);
	regmap_read(deepsleep_syscon_aon_apb, REG_AON_APB_APB_EB0,
			&aon_apb_eb0);
	regmap_read(deepsleep_syscon_aon_apb, REG_AON_APB_APB_EB1,
			&aon_apb_eb1);
	regmap_read(deepsleep_syscon_aon_apb, REG_AON_APB_APB_EB2,
			&aon_apb_eb2);
	regmap_read(deepsleep_syscon_aon_apb, REG_AON_APB_PWR_CTRL,
			&pwr_ctrl);
	regmap_read(deepsleep_syscon_aon_apb, REG_AON_APB_BB_BG_CTRL,
			&bb_bg_ctrl);
	regmap_read(deepsleep_syscon_aon_apb, REG_AON_APB_MPLL_CFG1,
			&mpll_cfg1);
	regmap_read(deepsleep_syscon_aon_apb, REG_AON_APB_DPLL_CFG1,
			&dpll_cfg1);
	regmap_read(deepsleep_syscon_aon_apb, REG_AON_APB_MPLL_CFG2,
			&mpll_cfg2);
	regmap_read(deepsleep_syscon_aon_apb, REG_AON_APB_DPLL_CFG2,
			&dpll_cfg2);
	regmap_read(deepsleep_syscon_pmu_apb, REG_PMU_APB_PD_PUB_SYS_CFG,
			&pd_pub_sys);
	regmap_read(deepsleep_syscon_pmu_apb, REG_PMU_APB_PD_MM_TOP_CFG,
			&pd_mm_top_cfg);
	regmap_read(deepsleep_syscon_pmu_apb, REG_PMU_APB_PD_AP_SYS_CFG,
			&pd_ap_sys_cfg);
	regmap_read(deepsleep_syscon_pmu_apb, REG_PMU_APB_CP_SLP_STATUS_DBG0,
			&cp_slp_status0);
	regmap_read(deepsleep_syscon_pmu_apb, REG_PMU_APB_PWR_STATUS0_DBG,
			&apb_pwrstatus0);
	regmap_read(deepsleep_syscon_pmu_apb, REG_PMU_APB_PWR_STATUS1_DBG,
			&apb_pwrstatus1);
	regmap_read(deepsleep_syscon_pmu_apb, REG_PMU_APB_PWR_STATUS2_DBG,
			&apb_pwrstatus2);
	regmap_read(deepsleep_syscon_pmu_apb, REG_PMU_APB_SLEEP_STATUS,
			&apb_slp_status);
	regmap_read(deepsleep_syscon_pmu_apb, REG_PMU_APB_SLEEP_CTRL,
			&sleep_ctrl);
	regmap_read(deepsleep_syscon_pmu_apb, REG_PMU_APB_DDR_SLEEP_CTRL,
			&ddr_slp_cfg);

}

static void print_debug_info(void)
{
	read_reg_value();
	show_deep_reg_status();
	pr_info("#####Enter AP_AHB  Condition Check!#####\n");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_USB_EB, "usb_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_DMA_EB, "dma_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_SDIO0_EB,
			"sdio0_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_SDIO1_EB,
			"sdio1_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_SDIO2_EB,
			"sdio2_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_EMMC_EB,
			"emmc_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_CE_SEC_EB,
			"ce_sec_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_CE_PUB_EB,
			"ce_pub_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_CE_EFUSE_EB,
			"ce_efuse_eb");
	pr_info("#####Enter AON_APB  Condition Check!#####\n");
	AON_APB_BITS_CLEAR_CHECK(REG_AON_APB_APB_EB0, BIT_AON_APB_CA7_DAP_EB,
			"c7_dap_eb");
}

static int print_thread(void *data)
{
	unsigned int cnt = 0;

	while (1) {
		wake_lock(&messages_wakelock);
		if (print_thread_enable)
			print_debug_info();
		if (!pm_get_wakeup_count(&cnt, false)) {
			pr_info("PM: has wakeup events in progress\n");
			pm_print_active_wakeup_sources();
		}
		msleep(100);
		wake_unlock(&messages_wakelock);
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(print_thread_interval * HZ);
	}
	return 0;
}
static void debugfs_init(void)
{
	dentry_debug_root = debugfs_create_dir("power", NULL);
	if (IS_ERR(dentry_debug_root) || !dentry_debug_root) {
		pr_info("!!!powermanager Failed to create debugfs directory\n");
		dentry_debug_root = NULL;
		return;
	}
	debugfs_create_u32("print_sleep_mode", 0644, dentry_debug_root,
			&is_print_sleep_mode);
	debugfs_create_u32("print_linux_clock", 0644, dentry_debug_root,
			&is_print_linux_clock);
	debugfs_create_u32("print_modem_clock", 0644, dentry_debug_root,
			&is_print_modem_clock);
	debugfs_create_u32("print_irq", 0644, dentry_debug_root, &is_print_irq);
	debugfs_create_u32("print_wakeup", 0644, dentry_debug_root,
			&is_print_wakeup);
	debugfs_create_u32("print_irq_runtime", 0644, dentry_debug_root,
			&is_print_irq_runtime);
	debugfs_create_u32("print_time", 0644, dentry_debug_root,
			&is_print_time);
	debugfs_create_u32("print_thread_enable", 0644, dentry_debug_root,
			&print_thread_enable);
	debugfs_create_u32("print_thread_interval", 0644, dentry_debug_root,
			&print_thread_interval);
}
void pm_debug_init(void)
{
#ifdef PM_PRINT_ENABLE
	struct task_struct *task;

	wake_lock_init(&messages_wakelock, WAKE_LOCK_SUSPEND,
			"pm_message_wakelock");
	task = kthread_create(print_thread, NULL, "pm_print");
	if (task == NULL)
		pr_info("Can't crate power manager print thread!\n");
	else
		wake_up_process(task);
#endif
	debugfs_init();
}

void deepsleep_condition_check(void)
{
	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}

	pr_info("#####Enter Deepsleep  Condition Check!######\n");
	pr_info("#####Enter AP_AHB  Condition Check!#####\n");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_USB_EB, "usb_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_DMA_EB, "dma_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_SDIO0_EB,
			"sdio0_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_SDIO1_EB,
			"sdio1_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_SDIO2_EB,
			"sdio2_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_EMMC_EB,
			"emmc_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_CE_SEC_EB,
			"ce_sec_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_CE_PUB_EB,
			"ce_pub_eb");
	pr_info("#####Enter AP_APB  Condition Check!#####\n");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_SIM0_EB,
			"sim0_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_IIS0_EB,
			"iis0_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_IIS1_EB,
			"iis1_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_IIS2_EB,
			"iis2_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_IIS3_EB,
			"iis3_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_I2C0_EB,
			"i2c0_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_I2C1_EB,
			"i2c1_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_I2C2_EB,
			"i2c2_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_I2C3_EB,
			"i2c3_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_I2C4_EB,
			"i2c4_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_UART0_EB,
			"uart0_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_UART1_EB,
			"uart1_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_UART2_EB,
			"uart2_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_UART3_EB,
			"uart3_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_UART4_EB,
			"uart4_eb");
	pr_info("#####Enter AON_APB  Condition Check!#####\n");
	AON_APB_BITS_CLEAR_CHECK(REG_AON_APB_CLK_EB0, BIT_AON_APB_AP_HS_SPI_EB,
			"sip1_eb_hs_spi_eb");
	AON_APB_BITS_CLEAR_CHECK(REG_AON_APB_APB_EB0, BIT_AON_APB_CA7_DAP_EB,
			"ca7_dab_eb");
	AON_APB_BITS_CLEAR_CHECK(REG_AON_APB_APB_EB2, BIT_AON_APB_AP_DAP_EB,
			"ap_dab_eb");
}

void bak_last_reg(void)
{
	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}
	read_reg_value();
}

void print_last_reg(void)
{
	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}
	pr_info("ap ahb reg\n");
	pr_info("##--REG_AP_AHB_AHB_EB : 0x%08x\n", ahb_eb);
	pr_info("ap apb reg\n");
	pr_info("##--REG_AP_APB_APB_EB : 0x%08x\n", ap_apb_eb);
	pr_info("##--REG_AP_AHB_MCU_PAUSE ---   0x%08x\n", mcu_pause);
	pr_info("##--REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG --- 0x%08x\n",
			sys_force_sleep);
	pr_info("##--REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG : 0x%08x\n",
			ap_sys_auto_sleep_cfg);
	pr_info("##--REG_AP_AHB_CA7_STANDBY_STATUS ---- 0x%08x\n",
			ca7_standby_status);
	pr_info("aon apb reg\n");
	pr_info("##--REG_AON_APB_APB_EB0 : 0x%08x\n", aon_apb_eb0);
	pr_info("##--REG_AON_APB_APB_EB1 : 0x%08x\n", aon_apb_eb1);
	pr_info("##--REG_AON_APB_APB_EB2 : 0x%08x\n", aon_apb_eb2);
	pr_info("##--REG_AON_APB_PWR_CTRL ----- 0x%08x\n", pwr_ctrl);
	pr_info("##--REG_AON_APB_BB_BG_CTRL ------ 0x%08x\n", bb_bg_ctrl);
	pr_info("##--REG_AON_APB_MPLL_CFG1 : 0x%08x\n", mpll_cfg1);
	pr_info("##--REG_AON_APB_DPLL_CFG1 : 0x%08x\n", dpll_cfg1);
	pr_info("##--REG_AON_APB_MPLL_CFG2 : 0x%08x\n", mpll_cfg2);
	pr_info("##--REG_AON_APB_DPLL_CFG2 : 0x%08x\n", dpll_cfg2);
	pr_info("aon pmu status reg\n");
	pr_info("REG_PMU_APB_PD_PUB_SYS_CFG ------ 0x%08x\n", pd_pub_sys);
	pr_info("REG_PMU_APB_PD_MM_TOP_CFG ------ 0x%08x\n", pd_mm_top_cfg);
	pr_info("REG_PMU_APB_PD_AP_SYS_CFG ------ 0x%08x\n", pd_ap_sys_cfg);
	pr_info("##--REG_PMU_APB_CP_SLP_STATUS_DBG0 : 0x%08x\n",
			cp_slp_status0);
	pr_info("REG_PMU_APB_SLEEP_CTRL ----- 0x%08x\n", sleep_ctrl);
	pr_info("##--REG_PMU_APB_DDR_SLEEP_CTRL : 0x%08x\n", ddr_slp_cfg);
	pr_info("##--REG_PMU_APB_SLEEP_STATUS : 0x%08x\n", apb_slp_status);
	pr_info("##--REG_PMU_APB_SLEEP_CTRL : 0x%08x\n", sleep_ctrl);
	show_deep_reg_status();
}

void show_deep_reg_status(void)
{

	pr_info("#####PWR_STATUS reg #####\n");
	pr_info("##--REG_PMU_APB_PWR_STATUS0_DBG : 0x%08x\n", apb_pwrstatus0);
	pr_info("##--REG_PMU_APB_PWR_STATUS1_DBG : 0x%08x\n", apb_pwrstatus1);
	pr_info("##--REG_PMU_APB_PWR_STATUS2_DBG : 0x%08x\n", apb_pwrstatus2);
	pr_info("#####PWR_STATUS0 POWER DOMAIN#####\n");
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
	pr_info("#####PWR_STATUS1 POWER DOMAIN#####\n");
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
	pr_info("#####PWR_STATUS2 POWER DOMAIN#####\n");
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

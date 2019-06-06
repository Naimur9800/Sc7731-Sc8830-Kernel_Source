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
#include <linux/interrupt.h>
#ifdef CONFIG_SPRD_SIPC
#include <linux/sipc.h>
#endif
/*#include <soc/sprd/gpio.h>*/
#include "pm_debug_arm64.h"

struct dentry *dentry_debug_root;
static int is_print_sleep_mode;
int is_print_linux_clock = 1;
int is_print_modem_clock = 1;
static int is_print_irq = 1;
static int is_print_wakeup = 1;
static int is_print_irq_runtime;
static int is_print_time;
static int print_thread_enable = 1;
static int print_thread_interval = 30;
static unsigned int core_time;
static unsigned int mcu_time;
static unsigned int lit_time;
static unsigned int deep_time_successed;
static unsigned int deep_time_failed;
static unsigned int sleep_time;

static u32 sprd_hard_irq[SPRD_HARD_INTERRUPT_NUM] = {0, };
static u32 sprd_irqs_sts[SPRD_INTC_NUM] = {0, };
static int is_wakeup;
static int irq_status;
static int sleep_mode = SLP_MODE_NON;
static char *sleep_mode_str[]  = {
	"[ARM]",
	"[MCU]",
	"[DEP]",
	"[LIT]",
	"[NON]"
};

#define BUS_NUM_MASK                    (0xf0ffffff)
static void show_bus_reg_status(void)
{
	int i = 0;
	int bus_reg = 0, reg = 0;

	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}

	for (i = 1; i <= 13; i++) {
		regmap_read(deepsleep_syscon_aon_apb,
			REG_AON_APB_SUB_SYS_DBG_SIG0,
			&reg);
		reg = (reg & BUS_NUM_MASK) | (i << 24);
		regmap_write(deepsleep_syscon_aon_apb,
			REG_AON_APB_SUB_SYS_DBG_SIG0,
			reg);

		regmap_read(deepsleep_syscon_pmu_apb,
			REG_PMU_APB_PMU_DEBUG,
			&bus_reg);
		pr_info("##--BUS_%d_REG_STATUS: 0x%08x\n", i, bus_reg);
	}
}

/*
 *TODO: Now it can not access A-Die registers by regmap API in disable
 *irq contents, but it will be useful for printing some A-die registers to
 *analyse problems when processing the deepsleep routine. So just leave
 *these code here in case there are other ways to access A-Die registers.
 */
#if 0
static struct regmap *sprd_get_ana_regmap_by_compatible(const char *compatible)
{
	struct device_node *node;
	struct platform_device *pdev;
	struct regmap *regmap;

	node = of_find_compatible_node(NULL, NULL, compatible);
	if (!node) {
		pr_err("%s, can't find node\n", __func__);
		return NULL;
	}

	pdev = of_find_device_by_node(node);
	if (!pdev) {
		pr_err("%s, can't find pdev\n", __func__);
		return NULL;
	}

	regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!pdev) {
		pr_err("%s, regmap err\n", __func__);
		return NULL;
	}

	return regmap;
}
#endif
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
				sprd_hard_irq[32+i]++;
		}
	}
	if (intc == 2) {
		for (i = 0; i < SPRD_HARD_INT_NUM_EACH_INTC; i++) {
			if (test_and_clear_bit(i, &val))
				sprd_hard_irq[64+i]++;
		}
	}
	if (intc == 3) {
		for (i = 0; i < SPRD_HARD_INT_NUM_EACH_INTC; i++) {
			if (test_and_clear_bit(i, &val))
				sprd_hard_irq[96+i]++;
		}
	}
	if (intc == 4) {
		for (i = 0; i < SPRD_HARD_INT_NUM_EACH_INTC; i++) {
			if (test_and_clear_bit(i, &val))
				sprd_hard_irq[128+i]++;
		}
	}
	if (intc == 5) {
		for (i = 0; i < SPRD_HARD_INT_NUM_EACH_INTC; i++) {
			if (test_and_clear_bit(i, &val))
				sprd_hard_irq[160+i]++;
		}
	}

}

void hard_irq_set(void)
{
	if (is_intc_base_vaddr_ok) {
		pr_err("%s,INTC register base vaddr is not ok!\n", __func__);
		return;
	}

	regmap_read(deepsleep_syscon_aon_intc0,
			INTCV0_IRQ_MSKSTS, &sprd_irqs_sts[0]);
	regmap_read(deepsleep_syscon_aon_intc0,
			INTCV0_FIQ_STS, &sprd_irqs_sts[1]);
	regmap_read(deepsleep_syscon_aon_intc1,
			INTCV1_IRQ_MSKSTS, &sprd_irqs_sts[2]);
	regmap_read(deepsleep_syscon_aon_intc1,
			INTCV1_FIQ_STS, &sprd_irqs_sts[3]);
	regmap_read(deepsleep_syscon_aon_intc2,
			INTCV2_IRQ_MSKSTS, &sprd_irqs_sts[4]);
	regmap_read(deepsleep_syscon_aon_intc2,
			INTCV2_FIQ_STS, &sprd_irqs_sts[5]);
	regmap_read(deepsleep_syscon_aon_intc3,
			INTCV3_IRQ_MSKSTS, &sprd_irqs_sts[6]);
	regmap_read(deepsleep_syscon_aon_intc3,
			INTCV3_FIQ_STS, &sprd_irqs_sts[7]);
	regmap_read(deepsleep_syscon_aon_intc4,
			INTCV4_IRQ_MSKSTS, &sprd_irqs_sts[8]);
	regmap_read(deepsleep_syscon_aon_intc4,
			INTCV4_FIQ_STS, &sprd_irqs_sts[9]);
	regmap_read(deepsleep_syscon_aon_intc5,
			INTCV5_IRQ_MSKSTS, &sprd_irqs_sts[10]);
	regmap_read(deepsleep_syscon_aon_intc5, INTCV5_FIQ_STS,
			&sprd_irqs_sts[11]);

	regmap_read(deepsleep_syscon_aon_intc0, INTCV0_IRQ_RAW, &irq_status);
	parse_hard_irq(irq_status, 0);

	regmap_read(deepsleep_syscon_aon_intc1, INTCV1_IRQ_RAW, &irq_status);
	parse_hard_irq(irq_status, 1);

	regmap_read(deepsleep_syscon_aon_intc2, INTCV2_IRQ_RAW, &irq_status);
	parse_hard_irq(irq_status, 2);

	regmap_read(deepsleep_syscon_aon_intc3, INTCV3_IRQ_RAW, &irq_status);
	parse_hard_irq(irq_status, 3);

	regmap_read(deepsleep_syscon_aon_intc4, INTCV4_IRQ_RAW, &irq_status);
	parse_hard_irq(irq_status, 4);

	regmap_read(deepsleep_syscon_aon_intc5, INTCV5_IRQ_RAW, &irq_status);
	parse_hard_irq(irq_status, 5);
}


void print_int_status(void)
{
	uint32_t intc0_bits = 0, intc1_bits = 0, intc2_bits = 0,
			intc3_bits = 0,	intc4_bits = 0, intc5_bits = 0;
	uint32_t intc0_irq_raw = 0, intc1_irq_raw = 0, intc2_irq_raw = 0,
			intc3_irq_raw = 0, intc4_irq_raw = 0, intc5_irq_raw = 0;
	uint32_t intc0_irq_en = 0, intc1_irq_en = 0, intc2_irq_en = 0,
			intc3_irq_en = 0, intc4_irq_en = 0, intc5_irq_en = 0;
#if 0
	uint32_t ana_intc_mask_status = 0, ana_intc_raw_status = 0,
			ana_intc_en = 0;
	uint32_t ana_eic_int_event = 0, ana_eic_int_en = 0,
			ana_eic_mask_int_status = 0;
	struct regmap *ana_regmap;
#endif
	if (is_intc_base_vaddr_ok) {
		pr_err("%s,INTC register base vaddr is not ok!\n", __func__);
		return;
	}

	regmap_read(deepsleep_syscon_aon_intc0, INTCV0_IRQ_MSKSTS, &intc0_bits);
	regmap_read(deepsleep_syscon_aon_intc1, INTCV1_IRQ_MSKSTS, &intc1_bits);
	regmap_read(deepsleep_syscon_aon_intc2, INTCV2_IRQ_MSKSTS, &intc2_bits);
	regmap_read(deepsleep_syscon_aon_intc3, INTCV3_IRQ_MSKSTS, &intc3_bits);
	regmap_read(deepsleep_syscon_aon_intc4, INTCV4_IRQ_MSKSTS, &intc4_bits);
	regmap_read(deepsleep_syscon_aon_intc5, INTCV5_IRQ_MSKSTS, &intc5_bits);

	regmap_read(deepsleep_syscon_aon_intc0, INTCV0_IRQ_RAW, &intc0_irq_raw);
	regmap_read(deepsleep_syscon_aon_intc1, INTCV1_IRQ_RAW, &intc1_irq_raw);
	regmap_read(deepsleep_syscon_aon_intc2, INTCV2_IRQ_RAW, &intc2_irq_raw);
	regmap_read(deepsleep_syscon_aon_intc3, INTCV3_IRQ_RAW, &intc3_irq_raw);
	regmap_read(deepsleep_syscon_aon_intc4, INTCV4_IRQ_RAW, &intc4_irq_raw);
	regmap_read(deepsleep_syscon_aon_intc5, INTCV5_IRQ_RAW, &intc5_irq_raw);

	regmap_read(deepsleep_syscon_aon_intc0, INTCV0_IRQ_EN, &intc0_irq_en);
	regmap_read(deepsleep_syscon_aon_intc1, INTCV1_IRQ_EN, &intc1_irq_en);
	regmap_read(deepsleep_syscon_aon_intc2, INTCV2_IRQ_EN, &intc2_irq_en);
	regmap_read(deepsleep_syscon_aon_intc3, INTCV3_IRQ_EN, &intc3_irq_en);
	regmap_read(deepsleep_syscon_aon_intc4, INTCV4_IRQ_EN, &intc4_irq_en);
	regmap_read(deepsleep_syscon_aon_intc5, INTCV5_IRQ_EN, &intc5_irq_en);

	pr_info("########INT STATUS########\n");
	pr_info("##--INTC0 mask:0x%08x raw:0x%08x en:0x%08x\n",
		intc0_bits, intc0_irq_raw, intc0_irq_en);
	pr_info("##--INTC1 mask:0x%08x raw:0x%08x en:0x%08x\n",
		intc1_bits, intc1_irq_raw, intc1_irq_en);
	pr_info("##--INTC2 mask:0x%08x raw:0x%08x en:0x%08x\n",
		intc2_bits, intc2_irq_raw, intc2_irq_en);
	pr_info("##--INTC3 mask:0x%08x raw:0x%08x en:0x%08x\n",
		intc3_bits, intc3_irq_raw, intc3_irq_en);
	pr_info("##--INTC4 mask:0x%08x raw:0x%08x en:0x%08x\n",
		intc4_bits, intc4_irq_raw, intc4_irq_en);
	pr_info("##--INTC5 mask:0x%08x raw:0x%08x en:0x%08x\n",
		intc5_bits, intc5_irq_raw, intc5_irq_en);

/*
 *TODO: Now it can not access A-Die registers by regmap API in disable
 *irq contents, but it will be useful for printing some A-die registers to
 *analyse problems when processing the deepsleep routine. So just leave
 *these code here in case there are other ways to access A-Die registers.
 */
#if 0
	ana_regmap = sprd_get_ana_regmap_by_compatible("sprd,sc2731-eic");
	if (!ana_regmap) {
		pr_err("%s,failed to find ana_regmap\n", __func__);
		return;
	}

	regmap_read(ana_regmap, ANA_REG_INT_MASK_STATUS, &ana_intc_mask_status);
	regmap_read(ana_regmap, ANA_REG_INT_RAW_STATUS, &ana_intc_raw_status);
	regmap_read(ana_regmap, ANA_REG_INT_EN, &ana_intc_en);
	regmap_read(ana_regmap, ANA_REG_EIC_INT_EVENT, &ana_eic_int_event);
	regmap_read(ana_regmap, ANA_REG_EIC_INT_EN, &ana_eic_int_en);
	regmap_read(ana_regmap, ANA_REG_EIC_MASK_INT_STATUS,
			&ana_eic_mask_int_status);

	pr_info("##--ANA EIC INT_EN: 0x%08x\n", ana_eic_int_en);
	pr_info("##--ANA EIC MASK_INT_STATUS: 0x%08x\n",
		ana_eic_mask_int_status);
	pr_info("##--ANA EIC INT_ENVENT: 0x%08x\n", ana_eic_int_event);
	pr_info("##--ANA INT MASK_STATUS: 0x%08x\n", ana_intc_mask_status);
	pr_info("##--ANA INT RAW_STATUS: 0x%08x\n", ana_intc_raw_status);
	pr_info("##--ANA INT EN: 0x%08x\n", ana_intc_en);
#endif
}

void print_hard_irq_inloop(int ret)
{
/*	unsigned int i, j, val;
	unsigned int gpio_irq[GPIO_GROUP_NUM];*/
/*
 *TODO: Now it can not access A-Die registers by regmap API in disable
 *irq contents, but it will be useful for printing some A-die registers to
 *analyse problems when processing the deepsleep routine. So just leave
 *these code here in case there are other ways to access A-Die registers.
 */
#if 0
	struct regmap *ana_regmap;
	unsigned int ana_intc_raw_status, ana_eic_raw_int_status;

	ana_regmap = sprd_get_ana_regmap_by_compatible("sprd,sc2731-eic");
#endif
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
	if (sprd_irqs_sts[10] != 0)
		pr_info("%c#:INTC5: %08x\n", ret ? 'S' : 'F',
			sprd_irqs_sts[10]);
	if (sprd_irqs_sts[11] != 0)
		pr_info("%c#:INTC5 FIQ: %08x\n", ret ? 'S' : 'F',
			sprd_irqs_sts[11]);
	if (sprd_irqs_sts[0]) {
		if (sprd_hard_irq[31]) {
			pr_info("wake up by ana ");
/* it is no use now*/
#if 0
			if (ana_regmap) {
				regmap_read(ana_regmap, ANA_REG_INT_RAW_STATUS,
					&ana_intc_raw_status);
				if (ana_intc_raw_status & BIT(0))
					pr_info("adc\n");
				if (ana_intc_raw_status & BIT(1))
					pr_info("gpio\n");
				if (ana_intc_raw_status & BIT(2))
					pr_info("rtc\n");
				if (ana_intc_raw_status & BIT(3))
					pr_info("wdg\n");
				if (ana_intc_raw_status & BIT(4))
					pr_info("fgu\n");
				if (ana_intc_raw_status & BIT(5)) {
					pr_info("eic\n");
					regmap_read(ana_regmap,
						ANA_REG_EIC_RAW_INT_STATUS,
						&ana_eic_raw_int_status);
					pr_info("##--ana eic 0x%08x\n",
						ana_eic_raw_int_status);
				}
				if (ana_intc_raw_status & BIT(6))
					pr_info("fast charge\n");
				if (ana_intc_raw_status & BIT(7))
					pr_info("aud head button\n");
				if (ana_intc_raw_status & BIT(8))
					pr_info("aud protect\n");
				if (ana_intc_raw_status & BIT(9))
					pr_info("tmr\n");
				if (ana_intc_raw_status & BIT(10))
					pr_info("Charger wdg\n");
				if (ana_intc_raw_status & BIT(11))
					pr_info("calibration\n");
				if (ana_intc_raw_status & BIT(12))
					pr_info("thermal\n");
				if (ana_intc_raw_status & BIT(14))
					pr_info("BIF\n");
				if (ana_intc_raw_status & BIT(15))
					pr_info("Switch Charger\n");
				pr_info("##--ANA INT 0x%08x\n",
					ana_intc_raw_status);
			}
#endif
		}
		if (sprd_hard_irq[30])
			pr_info("wake up by i2c\n");
		if (sprd_hard_irq[29]) {
#ifdef CONFIG_SPRD_SIPC
			sipc_set_wakeup_flag();
#endif
			pr_info("wake up by mbox_tar_ca53\n");
		}
		if (sprd_hard_irq[28])
			pr_info("wake up by mbox_src_ca53\n");
		if (sprd_hard_irq[27])
			pr_info("wake up by aon_syst\n");
		if (sprd_hard_irq[26])
			pr_info("wake up by aon_tmr\n");
		if (sprd_hard_irq[24])
			pr_info("wake up by adi\n");
	}
	if (sprd_irqs_sts[2]) {
		if (sprd_hard_irq[63])
			pr_info("wake up by thermal\n");
		if (sprd_hard_irq[61])
			pr_info("wake up by ca53_wtg\n");
		if (sprd_hard_irq[58])
			pr_info("wake up by ap_syst\n");
		if (sprd_hard_irq[57])
			pr_info("wake up by ap_tmr4\n");
		if (sprd_hard_irq[56])
			pr_info("wake up by ap_tmr3\n");
		if (sprd_hard_irq[55])
			pr_info("wake up by ap_tmr2\n");
		if (sprd_hard_irq[54])
			pr_info("wake up by ap_tmr1\n");
		if (sprd_hard_irq[53])
			pr_info("wake up by ap_tmr0\n");
		if (sprd_hard_irq[52])
			pr_info("wake up by eic\n");
		if (sprd_hard_irq[51])
			pr_info("wake up by kpd\n");
		if (sprd_hard_irq[50])
			pr_info("wake up by gpio\n");
	}
	if (sprd_irqs_sts[4]) {
		if (sprd_hard_irq[93])
			pr_info("wake up by vbc_audply_agcp\n");
		if (sprd_hard_irq[92])
			pr_info("wake up by vbc_audrcd_agcp\n");
		if (sprd_hard_irq[66])
			pr_info("wake up by gpu\n");
	}
	if (sprd_irqs_sts[10]) {
		if (sprd_hard_irq[166])
			pr_info("wake up by agcp_wdg_ret\n");
		if (sprd_hard_irq[165])
			pr_info("wake up by wtlcp_tg_wdg_rst\n");
		if (sprd_hard_irq[164])
			pr_info("wake up by wtlcp_lte_wdg_set\n");
	}

}

static void print_hard_irq(void)
{
	int i = SPRD_HARD_INTERRUPT_NUM - 1;

	if (!is_print_irq)
		return;
	do {
		if (0 != sprd_hard_irq[i])
			pr_info("##: sprd_hard_irq[%d] = %d.\n",
				i, sprd_hard_irq[i]);
	} while (--i >= 0);
}

#if 0
static void irq_reset(void)
{
	int i = SPRD_IRQ_NUM - 1;

	do {
		sprd_irqs[i] = 0;
	} while (--i >= 0);
}


void inc_irq(int irq)
{
	if (is_wakeup) {
		if (irq >= SPRD_IRQ_NUM) {
			pr_info("## bad irq number %d.\n", irq);
			return;
		}
		sprd_irqs[irq]++;
		if (is_print_wakeup)
			pr_info("\n#####: wakeup irq = %d.\n", irq);
		is_wakeup = 0;
	}
}
EXPORT_SYMBOL(inc_irq);


static void print_irq(void)
{
	int i = SPRD_IRQ_NUM - 1;

	if (!is_print_irq)
		return;
	do {
		if (0 != sprd_irqssprd_irqs[i])
			pr_info("##: sprd_irqs[%d] = %d.\n",
				i, sprd_irqs[i]);
	} while (--i >= 0);
}
#else
static void print_irq(void){}
#endif
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
	/*sleep_time = get_sys_cnt();*/
	hard_irq_reset();
}

void time_statisic_end(void)
{
	/*sleep_time = get_sys_cnt() - sleep_time;*/
}

void print_time(void)
{
	if (!is_print_time)
		return;
	pr_info("time statisics : sleep_time=%d, ", sleep_time);
	pr_info("core_time=%d, mcu_time=%d, ", core_time, mcu_time);
	pr_info("lit_time=%d, deep_sus=%d, ", lit_time, deep_time_successed);
	pr_info("dep_fail=%d\n", deep_time_failed);
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
			sleep_mode_str[sleep_mode], sprd_irqs_sts[0]);
		pr_info("###wake up form %s : %08x\n",
			sleep_mode_str[sleep_mode], sprd_irqs_sts[1]);
	}
}
#ifdef PM_PRINT_ENABLE
static struct wake_lock messages_wakelock;
#endif

#define PM_PRINT_ENABLE

uint32_t apb_pwrstatus0_dbg, apb_pwrstatus1_dbg, apb_pwrstatus2_dbg,
		apb_pwrstatus3_dbg, apb_pwrstatus4_dbg, apb_pwrstatus6_dbg;
uint32_t aon_apb_eb0, aon_apb_eb1, ahb_eb, ap_apb_eb,
		sys_force_sleep, ap_sys_auto_sleep_cfg;
uint32_t pubcp_slp_status_dbg0, wtlcp_slp_status_dbg0, agcp_slp_status_dbg0,
		sleep_ctrl, ddr_sleep_ctrl, apb_slp_status, pwr_ctrl;
uint32_t pd_pub0_sys, pd_pub1_sys, pubcp_sys_cfg, wtlcp_sys_cfg;


static void read_reg_value(void)
{
	regmap_read(deepsleep_syscon_ap_ahb,
		REG_AP_AHB_AHB_EB,
		&ahb_eb);
	regmap_read(deepsleep_syscon_ap_ahb,
		REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG,
		&ap_sys_auto_sleep_cfg);
	regmap_read(deepsleep_syscon_ap_apb,
		REG_AP_APB_APB_EB,
		&ap_apb_eb);
	regmap_read(deepsleep_syscon_aon_apb,
		REG_AON_APB_APB_EB0,
		&aon_apb_eb0);
	regmap_read(deepsleep_syscon_aon_apb,
		REG_AON_APB_APB_EB1,
		&aon_apb_eb1);
	regmap_read(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PUBCP_SLP_STATUS_DBG0,
		&pubcp_slp_status_dbg0);
	regmap_read(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_WTLCP_SLP_STATUS_DBG0,
		&wtlcp_slp_status_dbg0);
	regmap_read(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_AGCP_SLP_STATUS_DBG0,
		&agcp_slp_status_dbg0);
	regmap_read(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PWR_STATUS0_DBG,
		&apb_pwrstatus0_dbg);
	regmap_read(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PWR_STATUS1_DBG,
		&apb_pwrstatus1_dbg);
	regmap_read(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PWR_STATUS2_DBG,
		&apb_pwrstatus2_dbg);
	regmap_read(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PWR_STATUS3_DBG,
		&apb_pwrstatus3_dbg);
	regmap_read(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PWR_STATUS4_DBG,
		&apb_pwrstatus4_dbg);
	regmap_read(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PWR_STATUS6_DBG,
		&apb_pwrstatus6_dbg);
	regmap_read(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_SLEEP_STATUS,
		&apb_slp_status);
	regmap_read(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_DDR_SLEEP_CTRL,
		&ddr_sleep_ctrl);
	regmap_read(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_SLEEP_CTRL,
		&sleep_ctrl);
}
static void print_debug_info(void)
{
	unsigned int  mpll0_cfg, mpll1_cfg;

	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}

	read_reg_value();

	regmap_read(deepsleep_syscon_aon_apb,
		REG_AON_APB_MPLL0_CTRL,
		&mpll0_cfg);
	regmap_read(deepsleep_syscon_aon_apb,
		REG_AON_APB_MPLL1_CTRL,
		&mpll1_cfg);
	show_bus_reg_status();
	pr_info("##--REG_AON_APB_MPLL0_CTRL : 0x%08x\n", mpll0_cfg);
	pr_info("##--REG_AON_APB_MPLL1_CTRL : 0x%08x\n", mpll1_cfg);
	show_deep_reg_status();
	pr_info("########Enter Deepsleep  Condition Check!########\n");
	if (ahb_eb & BIT_AP_AHB_USB3_EB)
		pr_info("##-- BIT_USB_EB still set --##\n");
	if (ahb_eb & BIT_AP_AHB_DMA_EB)
		pr_info("##-- BIT_AP_AHB_DMA_EB still set --##\n");
	if (ahb_eb & BIT_AP_AHB_SDIO0_EB)
		pr_info("##-- BIT_AP_AHB_SDIO0_EB still set --##\n");
	if (ahb_eb & BIT_AP_AHB_SDIO1_EB)
		pr_info("##-- BIT_AP_AHB_SDIO1_EB still set --##\n");
	if (ahb_eb & BIT_AP_AHB_SDIO2_EB)
		pr_info("##-- BIT_AP_AHB_SDIO2_EB still set --##\n");
	if (ahb_eb & BIT_AP_AHB_EMMC_EB)
		pr_info("##-- BIT_AP_AHB_EMMC_EB still set --##\n");
	if (ahb_eb & BIT_AP_AHB_CC63S_EB)
		pr_info("##-- BIT_AP_AHB_CC63S_EB set --##\n");
	if (ahb_eb & BIT_AP_AHB_CC63P_EB)
		pr_info("##-- BIT_AP_AHB_CC63P_EB set --##\n");
	if (ahb_eb & BIT_AP_AHB_CE0_EB)
		pr_info("###---- BIT_AP_AHB_CE0_EB set ----###\n");
	if (ahb_eb & BIT_AP_AHB_CE1_EB)
		pr_info("###---- BIT_AP_AHB_CE1_EB set ----###\n");
	if (ap_apb_eb & BIT_AP_APB_SIM0_EB)
		pr_info("##-- BIT_AP_APB_SIM0_EB still set --##\n");
	if (ap_apb_eb & BIT_AP_APB_IIS0_EB)
		pr_info("##-- BIT_AP_APB_IIS0_EB set --##\n");
	if (ap_apb_eb & BIT_AP_APB_IIS1_EB)
		pr_info("##-- BIT_AP_APB_IIS1_EB set --##\n");
	if (ap_apb_eb & BIT_AP_APB_IIS2_EB)
		pr_info("##-- BIT_AP_APB_IIS2_EB set --##\n");
	if (ap_apb_eb & BIT_AP_APB_IIS3_EB)
		pr_info("##-- BIT_AP_APB_IIS3_EB set --##\n");
	if (ap_apb_eb & BIT_AP_APB_SPI0_EB)
		pr_info("##-- BIT_AP_APB_SPI0_EB set --##\n");
	if (ap_apb_eb & BIT_AP_APB_SPI1_EB)
		pr_info("##-- BIT_AP_APB_SPI1_EB set --##\n");
	if (ap_apb_eb & BIT_AP_APB_SPI2_EB)
		pr_info("##-- BIT_AP_APB_SPI2_EB set --##\n");
	if (ap_apb_eb & BIT_AP_APB_SPI3_EB)
		pr_info("###---- BIT_AP_APB_SPI3_EB set ----###\n");
	if (ap_apb_eb & BIT_AP_APB_I2C0_EB)
		pr_info("##-- BIT_AP_APB_I2C0_EB still set --##\n");
	if (ap_apb_eb & BIT_AP_APB_I2C1_EB)
		pr_info("##-- BIT_AP_APB_I2C1_EB still set --##\n");
	if (ap_apb_eb & BIT_AP_APB_I2C2_EB)
		pr_info("##-- BIT_AP_APB_I2C2_EB still set --##\n");
	if (ap_apb_eb & BIT_AP_APB_I2C3_EB)
		pr_info("##-- BIT_AP_APB_I2C3_EB still set --##\n");
	if (ap_apb_eb & BIT_AP_APB_I2C4_EB)
		pr_info("##-- BIT_AP_APB_I2C4_EB still set --##\n");
	if (ap_apb_eb & BIT_AP_APB_I2C5_EB)
		pr_info("##-- BIT_AP_APB_I2C5_EB still set --##\n");
	if (ap_apb_eb & BIT_AP_APB_UART0_EB)
		pr_info("##-- BIT_AP_APB_UART0_EB still set --##\n");
	if (ap_apb_eb & BIT_AP_APB_UART1_EB)
		pr_info("##-- BIT_AP_APB_UART1_EB still set --##\n");
	if (ap_apb_eb & BIT_AP_APB_UART2_EB)
		pr_info("##-- BIT_AP_APB_UART2_EB still set --##\n");
	if (ap_apb_eb & BIT_AP_APB_UART3_EB)
		pr_info("##-- BIT_AP_APB_UART3_EB still set --##\n");
	if (ap_apb_eb & BIT_AP_APB_UART4_EB)
		pr_info("##-- BIT_AP_APB_UART4_EB still set --##\n");
	if (aon_apb_eb0 & BIT_AON_APB_GPU_EB)
		pr_info("##-- BIT_AON_APB_GPU_EB still set --##\n");
	if (aon_apb_eb0 & BIT_AON_APB_CA53_DAP_EB)
		pr_info("##-- BIT_AON_APB_CA53_DAP_EB still set --##\n");
}

static int print_thread(void *data)
{
	int cnt = 0;

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
		pr_info("!!powermanager Failed to create debugfs directory\n");
		dentry_debug_root = NULL;
		return;
	}
	debugfs_create_u32("print_sleep_mode", 0644, dentry_debug_root,
			   &is_print_sleep_mode);
	debugfs_create_u32("print_linux_clock", 0644, dentry_debug_root,
			   &is_print_linux_clock);
	debugfs_create_u32("print_modem_clock", 0644, dentry_debug_root,
			   &is_print_modem_clock);
	debugfs_create_u32("print_irq", 0644, dentry_debug_root,
			   &is_print_irq);
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
	if (task == 0)
		pr_info("Can't crate power manager print thread!\n");
	else
		wake_up_process(task);
#endif
	debugfs_init();
}
void pm_debug_clr(void)
{
	if (dentry_debug_root != NULL)
		debugfs_remove_recursive(dentry_debug_root);
}

void deepsleep_condition_check(void)
{
	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}

	pr_info("#####Enter Deepsleep  Condition Check!######\n");
	pr_info("#####Enter AP_AHB  Condition Check!#####\n");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_USB3_EB,
				"usb3_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_DMA_EB,
				"dma_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_SDIO0_EB,
				"sdio0_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_SDIO1_EB,
				"sdio1_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_SDIO2_EB,
				"sdio2_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_EMMC_EB,
				"emmc_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_CC63S_EB,
				"cc63s_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_CC63P_EB,
				"cc63p_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_CE0_EB,
				"ce0_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_CE1_EB,
				"ce1_eb");
	pr_info("#####Enter AP_APB  Condition Check!#####\n");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_SIM0_EB,
				"sim0_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_IIS0_EB,
				"iis0_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_IIS1_EB,
				"iis1_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_IIS2_EB,
				"iis2_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_SPI0_EB,
				"spi0_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_SPI1_EB,
				"spi1_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_SPI2_EB,
				"spi2_eb");
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
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_I2C5_EB,
				"i2c5_eb");
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
	AON_APB_BITS_CLEAR_CHECK(REG_AON_APB_APB_EB0, BIT_AON_APB_GPU_EB,
				"gpu_eb");
	pr_info("#####Deepsleep  Condition Check End #####\n");
}


void bak_last_reg(void)
{
	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}

	read_reg_value();

	regmap_read(deepsleep_syscon_pmu_apb, REG_PMU_APB_PD_PUB0_SYS_CFG,
			&pd_pub0_sys);
	regmap_read(deepsleep_syscon_pmu_apb, REG_PMU_APB_PD_PUB1_SYS_CFG,
			&pd_pub1_sys);
	regmap_read(deepsleep_syscon_pmu_apb, REG_PMU_APB_PD_PUBCP_SYS_CFG,
			&pubcp_sys_cfg);
	regmap_read(deepsleep_syscon_pmu_apb, REG_PMU_APB_PD_WTLCP_SYS_CFG,
			&wtlcp_sys_cfg);
	regmap_read(deepsleep_syscon_aon_apb, REG_AON_APB_PWR_CTRL, &pwr_ctrl);
	regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG,
			&sys_force_sleep);
}

void print_last_reg(void)
{
	uint32_t pd_apsys_cfg, aon_apb_bb_bg_ctrl;

	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}
	regmap_read(deepsleep_syscon_pmu_apb, REG_PMU_APB_PD_AP_SYS_CFG,
			&pd_apsys_cfg);
	regmap_read(deepsleep_syscon_aon_apb, REG_AON_APB_BB_BG_CTRL,
			&aon_apb_bb_bg_ctrl);
	pr_info("##--REG_AON_APB_PWR_CTRL : 0x%08x\n", pwr_ctrl);
	pr_info("##--REG_PMU_APB_PD_PUB0_SYS_CFG : 0x%08x\n", pd_pub0_sys);
	pr_info("##--REG_PMU_APB_PD_PUB1_SYS_CFG : 0x%08x\n", pd_pub1_sys);
	pr_info("##--REG_PMU_APB_PD_PUBCP_SYS_CFG : 0x%08x\n", pubcp_sys_cfg);
	pr_info("##--REG_PMU_APB_PD_WTLCP_SYS_CFG : 0x%08x\n", wtlcp_sys_cfg);
	pr_info("##--REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG : 0x%08x\n",
			sys_force_sleep);
	pr_info("##--REG_PMU_APB_PD_AP_SYS_CFG : 0x%08x\n", pd_apsys_cfg);
	pr_info("##--REG_AON_APB_BB_BG_CTRL : 0x%08x\n", aon_apb_bb_bg_ctrl);

	show_deep_reg_status();
}


void show_pin_reg(void)
{
	uint32_t xtl0_rel_cfg, xtlbuf0_rel_cfg, xtlbuf1_rel_cfg,
		xtlbuf2_rel_cfg;

	if (is_intc_base_vaddr_ok || is_syscon_base_vaddr_ok) {
		pr_err("%s,INTC and SYSCON register base vaddr is not ok!\n",
			__func__);
		return;
	}

	regmap_read(deepsleep_syscon_pmu_apb, REG_PMU_APB_XTL0_REL_CFG,
				&xtl0_rel_cfg);
	regmap_read(deepsleep_syscon_pmu_apb, REG_PMU_APB_XTLBUF0_REL_CFG,
				&xtlbuf0_rel_cfg);
	regmap_read(deepsleep_syscon_pmu_apb, REG_PMU_APB_XTLBUF1_REL_CFG,
				&xtlbuf1_rel_cfg);
	regmap_read(deepsleep_syscon_pmu_apb, REG_PMU_APB_XTLBUF2_REL_CFG,
				&xtlbuf2_rel_cfg);
	/*ana & i2c & adi*/
	regmap_update_bits(deepsleep_syscon_aon_intc0, 0x8,
			BIT(24) | BIT(30) | BIT(31),
			BIT(24) | BIT(30) | BIT(31));
	/*kpd & gpio*/
	regmap_update_bits(deepsleep_syscon_aon_intc1, 0x8,
			BIT(18) | BIT(19), BIT(18) | BIT(19));

	pr_info("#######PIN REG#######\n");
	pr_info("##--REG_PMU_APB_XTL0_REL_CFG : 0x%08x\n", xtl0_rel_cfg);
	pr_info("##--REG_PMU_APB_XTLBUF0_REL_CFG : 0x%08x\n", xtlbuf0_rel_cfg);
	pr_info("##--REG_PMU_APB_XTLBUF1_REL_CFG : 0x%08x\n", xtlbuf1_rel_cfg);
	pr_info("##--REG_PMU_APB_XTLBUF2_REL_CFG : 0x%08x\n", xtlbuf2_rel_cfg);
}
void show_deep_reg_status(void)
{

	pr_info("##--REG_AP_APB_APB_EB : 0x%08x\n", ap_apb_eb);
	pr_info("##--REG_AP_AHB_AHB_EB : 0x%08x\n", ahb_eb);
	pr_info("##--REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG : 0x%08x\n",
					ap_sys_auto_sleep_cfg);

	pr_info("##--REG_AON_APB_APB_EB1 : 0x%08x\n", aon_apb_eb1);
	pr_info("##--REG_AON_APB_APB_EB0 : 0x%08x\n", aon_apb_eb0);

	pr_info("##--REG_PMU_APB_SLEEP_STATUS : 0x%08x\n", apb_slp_status);
	pr_info("##--REG_PMU_APB_AGCP_SLP_STATUS_DBG0 : 0x%08x\n",
						agcp_slp_status_dbg0);
	pr_info("##--REG_PMU_APB_WTLCP_SLP_STATUS_DBG0 : 0x%08x\n",
						wtlcp_slp_status_dbg0);
	pr_info("##--REG_PMU_APB_PUBCP_SLP_STATUS_DBG0 : 0x%08x\n",
						pubcp_slp_status_dbg0);

	pr_info("##--REG_PMU_APB_SLEEP_CTRL : 0x%08x\n", sleep_ctrl);
	pr_info("##--REG_PMU_APB_DDR_SLEEP_CTRL : 0x%08x\n", ddr_sleep_ctrl);

	pr_info("#####PWR_STATUS reg #####\n");
	pr_info("##--REG_PMU_APB_PWR_STATUS0_DBG : 0x%08x\n",
					apb_pwrstatus0_dbg);
	pr_info("##--REG_PMU_APB_PWR_STATUS1_DBG : 0x%08x\n",
					apb_pwrstatus1_dbg);
	pr_info("##--REG_PMU_APB_PWR_STATUS2_DBG : 0x%08x\n",
					apb_pwrstatus2_dbg);
	pr_info("##--REG_PMU_APB_PWR_STATUS3_DBG : 0x%08x\n",
					apb_pwrstatus3_dbg);
	pr_info("##--REG_PMU_APB_PWR_STATUS4_DBG : 0x%08x\n",
					apb_pwrstatus4_dbg);
	pr_info("##--REG_PMU_APB_PWR_STATUS6_DBG : 0x%08x\n",
					apb_pwrstatus6_dbg);
	pr_info("#####PWR_STATUS0 POWER DOMAIN#####\n");
	pr_info("##--status of power domain 'WTLCP_SYS' :0x%08x\n",
		(apb_pwrstatus0_dbg & BIT_PMU_APB_PD_WTLCP_SYS_STATE(31))
		>> 25);
	pr_info("##--status of power domain 'WTLCP_TGDSP' :0x%08x\n",
		(apb_pwrstatus0_dbg & BIT_PMU_APB_PD_WTLCP_TGDSP_STATE(31))
		>> 20);
	pr_info("##--status of power domain 'WTLCP_HU3GE_B' :0x%08x\n",
		(apb_pwrstatus0_dbg & BIT_PMU_APB_PD_WTLCP_HU3GE_B_STATE(31))
		>> 15);
	pr_info("##--status of power domain 'CA53_BIG_MP4' :0x%08x\n",
		(apb_pwrstatus0_dbg & BIT_PMU_APB_PD_CA53_BIG_MP4_STATE(31))
		>> 10);
	pr_info("##--status of power domain 'CA53_LIT_MP4' :0x%08x\n",
		(apb_pwrstatus0_dbg & BIT_PMU_APB_PD_CA53_LIT_MP4_STATE(31))
		>> 5);
	pr_info("##--status of power domain 'CA53_TOP' :0x%08x\n",
		apb_pwrstatus0_dbg & BIT_PMU_APB_PD_CA53_TOP_STATE(31));
	pr_info("#####PWR_STATUS1 POWER DOMAIN#####\n");
	pr_info("##--status of power domain 'CA53_BIG_C1' :0x%08x\n",
		(apb_pwrstatus1_dbg & BIT_PMU_APB_PD_CA53_BIG_C1_STATE(31))
		>> 25);
	pr_info("##--status of power domain 'CA53_BIG_C0' :0x%08x\n",
		(apb_pwrstatus1_dbg & BIT_PMU_APB_PD_CA53_BIG_C0_STATE(31))
		>> 20);
	pr_info("##--status of power domain 'CA53_LIT_C3' :0x%08x\n",
		(apb_pwrstatus1_dbg & BIT_PMU_APB_PD_CA53_LIT_C3_STATE(31))
		>> 15);
	pr_info("##--status of power domain 'CA53_LIT_C2' :0x%08x\n",
		(apb_pwrstatus1_dbg & BIT_PMU_APB_PD_CA53_LIT_C2_STATE(31))
		>> 10);
	pr_info("##--status of power domain 'CA53_LIT_C1' :0x%08x\n",
		(apb_pwrstatus1_dbg & BIT_PMU_APB_PD_CA53_LIT_C1_STATE(31))
		>> 5);
	pr_info("##--status of power domain 'CA53_LIT_C0' :0x%08x\n",
		apb_pwrstatus1_dbg & BIT_PMU_APB_PD_CA53_LIT_C0_STATE(31));
	pr_info("#####PWR_STATUS2 POWER DOMAIN#####\n");
	pr_info("##--status of power domain 'WTLCP_LDSP' :0x%08x\n",
		(apb_pwrstatus2_dbg & BIT_PMU_APB_PD_WTLCP_LDSP_STATE(31))
		>> 25);
	pr_info("##--status of power domain 'WTLCP_HU3GE_A' :0x%08x\n",
		(apb_pwrstatus2_dbg & BIT_PMU_APB_PD_WTLCP_HU3GE_A_STATE(31))
		>> 20);
	pr_info("##--status of power domain 'WTLCP_GSM' :0x%08x\n",
		(apb_pwrstatus2_dbg & BIT_PMU_APB_PD_WTLCP_GSM_STATE(31))
		>> 15);
	pr_info("##--status of power domain 'WTLCP_TD' :0x%08x\n",
		(apb_pwrstatus2_dbg & BIT_PMU_APB_PD_WTLCP_TD_STATE(31))
		>> 10);
	pr_info("##--status of power domain 'WTLCP_LTE_P2' :0x%08x\n",
		(apb_pwrstatus2_dbg & BIT_PMU_APB_PD_WTLCP_LTE_P2_STATE(31))
		>> 5);
	pr_info("##--status of power domain 'WTLCP_LTE_P1' :0x%08x\n",
		apb_pwrstatus2_dbg & BIT_PMU_APB_PD_WTLCP_LTE_P1_STATE(31));
	pr_info("#####PWR_STATUS3 POWER DOMAIN#####\n");
	pr_info("##--status of power domain 'AGCP_SYS' :0x%08x\n",
		(apb_pwrstatus3_dbg & BIT_PMU_APB_PD_AGCP_SYS_STATE(31))
		>> 25);
	pr_info("##--status of power domain 'AGCP_DSP' :0x%08x\n",
		(apb_pwrstatus3_dbg & BIT_PMU_APB_PD_AGCP_DSP_STATE(31))
		>> 20);
	pr_info("##--status of power domain 'AGCP_GSM' :0x%08x\n",
		(apb_pwrstatus3_dbg & BIT_PMU_APB_PD_AGCP_GSM_STATE(31))
		>> 15);
	pr_info("##--status of power domain 'PUBCP_SYS' :0x%08x\n",
		(apb_pwrstatus3_dbg & BIT_PMU_APB_PD_PUBCP_SYS_STATE(31))
		>> 10);
	pr_info("##--status of power domain 'PUB0_SYS' :0x%08x\n",
		(apb_pwrstatus3_dbg & BIT_PMU_APB_PD_PUB0_SYS_STATE(31))
		>> 5);
	pr_info("##--status of power domain 'PUB1_SYS' :0x%08x\n",
		apb_pwrstatus3_dbg & BIT_PMU_APB_PD_PUB1_SYS_STATE(31));
	pr_info("#####PWR_STATUS4 POWER DOMAIN#####\n");
	pr_info("##--status of power domain 'CA53_BIG_C3' :0x%08x\n",
		(apb_pwrstatus4_dbg & BIT_PMU_APB_PD_CA53_BIG_C3_STATE(31))
		>> 25);
	pr_info("##--status of power domain 'CA53_BIG_C2' :0x%08x\n",
		(apb_pwrstatus4_dbg & BIT_PMU_APB_PD_CA53_BIG_C2_STATE(31))
		>> 20);
	pr_info("##--status of power domain 'AP_SYS' :0x%08x\n",
		(apb_pwrstatus4_dbg & BIT_PMU_APB_PD_AP_SYS_STATE(31))
		>> 15);
	pr_info("##--status of power domain 'DISP_SYS' :0x%08x\n",
		(apb_pwrstatus4_dbg & BIT_PMU_APB_PD_DISP_SYS_STATE(31))
		>> 10);
	pr_info("##--status of power domain 'CAM_SYS' :0x%08x\n",
		(apb_pwrstatus4_dbg & BIT_PMU_APB_PD_CAM_SYS_STATE(31))
		>> 5);
	pr_info("##--status of power domain 'VSP_SYS' :0x%08x\n",
		apb_pwrstatus4_dbg & BIT_PMU_APB_PD_VSP_SYS_STATE(31));
	pr_info("#####PWR_STATUS6 POWER DOMAIN#####\n");
	pr_info("##--status of power domain 'WTLCP_LTE_P4' :0x%08x\n",
		(apb_pwrstatus6_dbg & BIT_PMU_APB_PD_WTLCP_LTE_P4_STATE(31))
		>> 5);
	pr_info("##--status of power domain 'WTLCP_LTE_P3' :0x%08x\n",
		apb_pwrstatus6_dbg & BIT_PMU_APB_PD_WTLCP_LTE_P3_STATE(31));
}

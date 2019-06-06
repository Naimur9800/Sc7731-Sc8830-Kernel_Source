/*
 * Lightsleep config for Spreadtrum.
 *
 * Copyright (C) 2015 Spreadtrum Ltd.
 * Author: Icy Zhu <icy.zhu@spreadtrum.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/sched.h>
#include <linux/io.h>


#ifdef CONFIG_ARM
#include "pm_debug_arm.h"
#endif

#ifdef CONFIG_ARM
#define DIS_ALL 0xFFFFFFFF

struct IntcGroup {
	u32 intc0;
	u32 intc1;
	u32 intc2;
	u32 intc3;
};
static struct IntcGroup g_intc;
#endif

static int light_sleep;
static int trace_debug;
static int test_power;
static int mcu_sleep_debug;
static int heavy_sleep;


struct regmap *cpuidle_syscon_apahb;
struct regmap *cpuidle_syscon_apapb;
struct regmap *cpuidle_syscon_aonapb;
struct regmap *cpuidle_syscon_pmuapb;

#ifdef CONFIG_ARM
struct regmap *cpuidle_syscon_ap_intc0;
struct regmap *cpuidle_syscon_ap_intc1;
struct regmap *cpuidle_syscon_ap_intc2;
struct regmap *cpuidle_syscon_ap_intc3;

static u32 uart1_eb_temp;
static int doze_flag_en;
static void __iomem *sprd_ap_gic_base_vaddr;
#endif


/* Enable Lightsleep Function */
module_param_named(light_sleep, light_sleep, int, S_IRUGO | S_IWUSR);
/* Used to trace32 debug, because the value of trace32 is jumpy-changing while
   DAP clock be gatenning */
module_param_named(trace_debug, trace_debug, int, S_IRUGO | S_IWUSR);
/* Used to lightsleep state power test,if test_power be set to 1, the system
   will keep lightsleep state until to press powerkey */
module_param_named(test_power, test_power, int, S_IRUGO | S_IWUSR);
/* Used to MCU_SYS_SLEEP debug */
module_param_named(mcu_sleep_debug, mcu_sleep_debug, int, S_IRUGO | S_IWUSR);

#ifdef CONFIG_ARM
static void pm_ca7_core_auto_gate_enable(int enable)
{
	if (enable)
		regmap_update_bits(cpuidle_syscon_apahb,
				  REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG,
				  BIT_AP_AHB_CA7_CORE_AUTO_GATE_EN,
				  BIT_AP_AHB_CA7_CORE_AUTO_GATE_EN);
	else
		regmap_update_bits(cpuidle_syscon_apahb,
				  REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG,
				  BIT_AP_AHB_CA7_CORE_AUTO_GATE_EN,
				  ~BIT_AP_AHB_CA7_CORE_AUTO_GATE_EN);
}

static void pm_sync_gic_intc(void)
{
	u32 temp;

	temp = readl_relaxed(sprd_ap_gic_base_vaddr + 0x100 + 4);
	if (temp != g_intc.intc0) {
		regmap_write(cpuidle_syscon_ap_intc0, INTCV_IRQ_DIS, DIS_ALL);
		regmap_write(cpuidle_syscon_ap_intc0, INTCV_IRQ_EN, temp);
	}

	temp = readl_relaxed(sprd_ap_gic_base_vaddr + 0x100 + 8);
	if (temp != g_intc.intc1) {
		regmap_write(cpuidle_syscon_ap_intc1, INTCV_IRQ_DIS, DIS_ALL);
		regmap_write(cpuidle_syscon_ap_intc1, INTCV_IRQ_EN, temp);
	}

	temp = readl_relaxed(sprd_ap_gic_base_vaddr + 0x100 + 12);
	if (temp != g_intc.intc2) {
		regmap_write(cpuidle_syscon_ap_intc2, INTCV_IRQ_DIS, DIS_ALL);
		regmap_write(cpuidle_syscon_ap_intc2, INTCV_IRQ_EN, temp);
	}

	temp = readl_relaxed(sprd_ap_gic_base_vaddr + 0x100 + 16);
	if (temp != g_intc.intc3) {
		regmap_write(cpuidle_syscon_ap_intc3, INTCV_IRQ_DIS, DIS_ALL);
		regmap_write(cpuidle_syscon_ap_intc3, INTCV_IRQ_EN, temp);
	}
}

static bool apsys_master_slave_check(void)
{
	 u32 aon_apb_eb0 = 0;
	 u32 aon_apb_eb1 = 0;
	 u32 ap_ahb_eb = 0;
	 u32 ap_apb_eb = 0;

	regmap_read(cpuidle_syscon_apahb, REG_AP_AHB_AHB_EB, &ap_ahb_eb);
	if (ap_ahb_eb)
		return 1;

	regmap_read(cpuidle_syscon_apapb, REG_AP_APB_APB_EB, &ap_apb_eb);
	ap_apb_eb &= ~(BIT_AP_APB_UART1_EB|
			BIT_AP_APB_INTC3_EB|
			BIT_AP_APB_INTC2_EB|
			BIT_AP_APB_INTC1_EB|
			BIT_AP_APB_INTC0_EB);
	if (ap_apb_eb)
		return 1;

	regmap_read(cpuidle_syscon_aonapb, REG_AON_APB_APB_EB0, &aon_apb_eb0);
	regmap_read(cpuidle_syscon_aonapb, REG_AON_APB_APB_EB1, &aon_apb_eb1);

	if ((aon_apb_eb0 & (BIT_AON_APB_MM_EB|BIT_AON_APB_GPU_EB))|
			(aon_apb_eb1 & BIT_AON_APB_MM_VSP_EB))
		return 1;

	return 0;
}

static void doze_sleep_bak_restore_uarteb(int bak)
{
	if (bak) {
		u32 ap_apb_eb_temp;

		regmap_read(cpuidle_syscon_apapb,
				REG_AP_APB_APB_EB,
				&ap_apb_eb_temp);
		uart1_eb_temp = BIT_AP_APB_UART1_EB & ap_apb_eb_temp;
	} else {
		if (uart1_eb_temp)
			regmap_update_bits(
					cpuidle_syscon_apapb,
					REG_AP_APB_APB_EB,
					BIT_AP_APB_UART1_EB,
					BIT_AP_APB_UART1_EB);
	}
}

#endif

void light_sleep_en(struct device_node *np)
{
	if (light_sleep) {
#ifdef CONFIG_ARM
		u32 val;
		if (smp_processor_id() == 0) {
			regmap_update_bits(cpuidle_syscon_aonapb,
				REG_AON_APB_APB_EB2,
				BIT_AON_APB_AP_DAP_EB,
				0);
			regmap_read(cpuidle_syscon_apahb,
					REG_AP_AHB_AHB_EB, &val);
			val &= BIT_AP_AHB_DMA_EB;
			if (!val && (num_online_cpus() == 1))
				regmap_update_bits(
					cpuidle_syscon_apahb,
					REG_AP_AHB_MCU_PAUSE,
					BIT_AP_AHB_MCU_SYS_SLEEP_EN,
					BIT_AP_AHB_MCU_SYS_SLEEP_EN);

			pm_sync_gic_intc();
			pm_ca7_core_auto_gate_enable(1);

			regmap_update_bits(cpuidle_syscon_apahb,
				REG_AP_AHB_MCU_PAUSE,
				BIT_AP_AHB_MCU_LIGHT_SLEEP_EN,
				BIT_AP_AHB_MCU_LIGHT_SLEEP_EN);
			regmap_update_bits(cpuidle_syscon_pmuapb,
				REG_PMU_APB_LIGHT_SLEEP_ENABLE,
				BIT_PMU_APB_AP_LSLP_ENA,
				BIT_PMU_APB_AP_LSLP_ENA);
		}
#endif

#ifdef CONFIG_ARM64
		/* Cluster Gating */
		regmap_update_bits(cpuidle_syscon_apahb,
				REG_AP_AHB_APCPU_AUTO_GATE_EN,
				BIT_AP_AHB_APCPU_AUTO_GATE_EN,
				BIT_AP_AHB_APCPU_AUTO_GATE_EN);
		/* Light sleep */
		regmap_update_bits(cpuidle_syscon_apahb, REG_AP_AHB_MCU_PAUSE,
				BIT_AP_AHB_MCU_LIGHT_SLEEP_EN,
				BIT_AP_AHB_MCU_LIGHT_SLEEP_EN);
		/* Auto Gating */
		if (!trace_debug) {
			regmap_update_bits(cpuidle_syscon_apahb,
					REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG,
					BIT_AP_AHB_APCPU_DBG_AUTO_GATE_EN,
					BIT_AP_AHB_APCPU_DBG_AUTO_GATE_EN);

			regmap_update_bits(cpuidle_syscon_apahb,
					REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG,
					BIT_AP_AHB_APCPU_TRACE_AUTO_GATE_EN,
					BIT_AP_AHB_APCPU_TRACE_AUTO_GATE_EN);

			/* Disable DAP */
			regmap_update_bits(cpuidle_syscon_aonapb,
					REG_AON_APB_APB_EB0,
					BIT_AON_APB_APCPU_DAP_EB, 0);
		}
#endif
	}
}

void light_sleep_dis(struct device_node *np)
{
	if (light_sleep) {
#ifdef CONFIG_ARM
		u32 cpu = smp_processor_id();
		if (cpu == 0) {
			pm_sync_gic_intc();
			pm_ca7_core_auto_gate_enable(0);
		}

		regmap_update_bits(cpuidle_syscon_aonapb,
				REG_AON_APB_APB_EB2,
				BIT_AON_APB_AP_DAP_EB,
				BIT_AON_APB_AP_DAP_EB);
		regmap_update_bits(cpuidle_syscon_apahb,
				REG_AP_AHB_MCU_PAUSE,
		BIT_AP_AHB_MCU_SYS_SLEEP_EN|BIT_AP_AHB_MCU_LIGHT_SLEEP_EN,
				0);
		regmap_update_bits(cpuidle_syscon_pmuapb,
				REG_PMU_APB_LIGHT_SLEEP_ENABLE,
				BIT_PMU_APB_AP_LSLP_ENA,
				0);
#endif
#ifdef CONFIG_ARM64
		/* Enable DAP */
		regmap_update_bits(cpuidle_syscon_aonapb, REG_AON_APB_APB_EB0,
				BIT_AON_APB_APCPU_DAP_EB,
				BIT_AON_APB_APCPU_DAP_EB);
		regmap_update_bits(cpuidle_syscon_apahb,
				REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG,
				BIT_AP_AHB_APCPU_DBG_AUTO_GATE_EN, 0);
		regmap_update_bits(cpuidle_syscon_apahb,
				REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG,
				BIT_AP_AHB_APCPU_TRACE_AUTO_GATE_EN, 0);
#endif
	}
}
#ifdef CONFIG_ARM
static void doze_sleep_en(struct device_node *np)
{
	u32 dma_eb;
	if (heavy_sleep && smp_processor_id() == 0) {
		doze_flag_en = 1;
		doze_sleep_bak_restore_uarteb(1);

		regmap_update_bits(
			cpuidle_syscon_aonapb,
			REG_AON_APB_APB_EB2,
			BIT_AON_APB_AP_DAP_EB,
			0);
		regmap_read(cpuidle_syscon_apahb,
				REG_AP_AHB_AHB_EB, &dma_eb);
		dma_eb &= BIT_AP_AHB_DMA_EB;
		if (!dma_eb && (num_online_cpus() == 1)) {
			regmap_update_bits(
				cpuidle_syscon_apahb,
				REG_AP_AHB_MCU_PAUSE,
				BIT_AP_AHB_MCU_SYS_SLEEP_EN,
				BIT_AP_AHB_MCU_SYS_SLEEP_EN);
			regmap_update_bits(
				cpuidle_syscon_apapb,
				REG_AP_APB_APB_EB,
				BIT_AP_APB_UART1_EB,
				0);
		}
		pm_sync_gic_intc();
		pm_ca7_core_auto_gate_enable(1);

		/* Set mtx_lp_disable to 0 (default value 0) */
		regmap_update_bits(cpuidle_syscon_apahb,
				REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG,
				BIT_AP_AHB_AP_MAINMTX_LP_DISABLE,
				0);
		/* To bypass LPC force channel by H/W (default value 0) */
		regmap_update_bits(cpuidle_syscon_apahb,
				REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG,
				BIT_AP_AHB_LP_AUTO_CTRL_EN,
				0);
		regmap_update_bits(cpuidle_syscon_apahb,
				REG_AP_AHB_MCU_PAUSE,
				BIT_AP_AHB_MCU_DEEP_SLEEP_EN,
				BIT_AP_AHB_MCU_DEEP_SLEEP_EN);

		regmap_update_bits(cpuidle_syscon_pmuapb,
				REG_PMU_APB_AP_DSLP_ENA,
				BIT_PMU_APB_AP_DSLP_ENA,
				0);

		regmap_update_bits(cpuidle_syscon_pmuapb,
				REG_PMU_APB_SYS_DOZE_DSLP_ENA,
				BIT_PMU_APB_AP_DOZE_DSLP_ENA,
				BIT_PMU_APB_AP_DOZE_DSLP_ENA);
	}
}

static void doze_sleep_dis(struct device_node *np)
{

	if (heavy_sleep && smp_processor_id() == 0) {

		doze_flag_en = 0;

		doze_sleep_bak_restore_uarteb(0);

		pm_sync_gic_intc();
		pm_ca7_core_auto_gate_enable(0);

		regmap_update_bits(cpuidle_syscon_aonapb,
				REG_AON_APB_APB_EB2,
				BIT_AON_APB_AP_DAP_EB,
				BIT_AON_APB_AP_DAP_EB);

		regmap_update_bits(cpuidle_syscon_apahb,
				REG_AP_AHB_MCU_PAUSE,
				BIT_AP_AHB_MCU_DEEP_SLEEP_EN,
				0);
		regmap_update_bits(cpuidle_syscon_apahb,
				REG_AP_AHB_MCU_PAUSE,
				BIT_AP_AHB_MCU_SYS_SLEEP_EN,
				0);
		/* BIT_AP_AHB_AP_PERI_FORCE_SLP_EB */
		regmap_update_bits(cpuidle_syscon_apahb,
				REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG,
				BIT_AP_AHB_AP_PERI_FORCE_SLP,
				0);
		regmap_update_bits(cpuidle_syscon_pmuapb,
				REG_PMU_APB_AP_DSLP_ENA,
				BIT_PMU_APB_AP_DSLP_ENA,
				BIT_PMU_APB_AP_DSLP_ENA);
		regmap_update_bits(cpuidle_syscon_pmuapb,
				REG_PMU_APB_SYS_DOZE_DSLP_ENA,
				BIT_PMU_APB_AP_DOZE_DSLP_ENA,
				0);
	}
}

void light_doze_sleep_en(struct device_node *np)
{
	if (apsys_master_slave_check() == 0)
		doze_sleep_en(np);
	else
		light_sleep_en(np);
}

void light_doze_sleep_dis(struct device_node *np)
{
	if (doze_flag_en)
		doze_sleep_dis(np);
	else
		light_sleep_dis(np);
}
#endif

static int __init light_sleep_init(void)
{
#ifdef CONFIG_ARM
	struct device_node *node = NULL;
#endif
	struct device_node *np = of_find_node_by_name(NULL, "idle-states");

	cpuidle_syscon_apahb = syscon_regmap_lookup_by_phandle(np,
			"sprd,sys-ap-ahb");
	cpuidle_syscon_apapb = syscon_regmap_lookup_by_phandle(np,
			"sprd,sys-ap-apb");
	cpuidle_syscon_aonapb = syscon_regmap_lookup_by_phandle(np,
			"sprd,sys-aon-apb");
	cpuidle_syscon_pmuapb = syscon_regmap_lookup_by_phandle(np,
			"sprd,sys-pmu-apb");

	if (IS_ERR(cpuidle_syscon_apahb) ||
		IS_ERR(cpuidle_syscon_aonapb) ||
		IS_ERR(cpuidle_syscon_pmuapb)) {
		pr_err("%s:failed to find sprd,sys-syscon\n", __func__);
		return -EINVAL;
	}

#ifdef CONFIG_ARM
	node = of_find_compatible_node(NULL, NULL, "arm,cortex-a9-gic");
	if (!node)
		pr_err("failed to get gic node\n");

	sprd_ap_gic_base_vaddr = of_iomap(node, 0);

	cpuidle_syscon_ap_intc0 = syscon_regmap_lookup_by_phandle(
			np,
			"sprd,sys-ap-intc0");
	cpuidle_syscon_ap_intc1 = syscon_regmap_lookup_by_phandle(
			np,
			"sprd,sys-ap-intc1");
	cpuidle_syscon_ap_intc2 = syscon_regmap_lookup_by_phandle(
			np,
			"sprd,sys-ap-intc2");
	cpuidle_syscon_ap_intc3 = syscon_regmap_lookup_by_phandle(
			np,
			"sprd,sys-ap-intc3");

	if (IS_ERR(cpuidle_syscon_ap_intc0) ||
		IS_ERR(cpuidle_syscon_ap_intc1) ||
		IS_ERR(cpuidle_syscon_ap_intc2) ||
		IS_ERR(cpuidle_syscon_ap_intc1)) {
		pr_err("%s:failed to find sprd,sys-ap-intc\n", __func__);
		return -EINVAL;
	}
#endif

#ifdef CONFIG_ARM64
	/* Cluster Gating */
	regmap_update_bits(cpuidle_syscon_apahb,
			REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG,
			BIT_AP_AHB_APCPU_CORE_AUTO_GATE_EN,
			BIT_AP_AHB_APCPU_CORE_AUTO_GATE_EN);
	/* AP_AHB clock Auto gating while ap doesn't into mcu_sys_sleep */
	regmap_update_bits(cpuidle_syscon_apahb,
			REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG,
			BIT_AP_AHB_AP_AHB_AUTO_GATE_EN,
			BIT_AP_AHB_AP_AHB_AUTO_GATE_EN);
	/* For wakeup lightsleep */
	regmap_update_bits(cpuidle_syscon_pmuapb, REG_PMU_APB_AP_WAKEUP_POR_CFG,
			BIT_PMU_APB_AP_WAKEUP_POR_N,
			BIT_PMU_APB_AP_WAKEUP_POR_N);
#endif

	light_sleep = 1;
	trace_debug = 0;
	test_power = 0;
	mcu_sleep_debug = 0;
	heavy_sleep = 1;
	return 0;
}

arch_initcall(light_sleep_init);

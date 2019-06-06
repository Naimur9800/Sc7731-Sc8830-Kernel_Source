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
#ifdef CONFIG_TRUSTY
#include <linux/trusty/smcall.h>
#include <linux/trusty/sm_err.h>
#include <linux/trusty/trusty.h>
#endif
#include <linux/clk.h>
#include <linux/slab.h>
#include "pm_arm.h"
#include "pm_debug_arm.h"
#define CONFIG_DEEP_DEBUG
static struct device_node *deepsleep_np;
struct regmap *deepsleep_syscon_ap_ahb;
struct regmap *deepsleep_syscon_pmu_apb;
struct regmap *deepsleep_syscon_aon_apb;
struct regmap *deepsleep_syscon_ap_apb;
struct regmap *deepsleep_syscon_ap_intc0;
struct regmap *deepsleep_syscon_ap_intc1;
struct regmap *deepsleep_syscon_ap_intc2;
struct regmap *deepsleep_syscon_ap_intc3;
struct regmap *deepsleep_syscon_aon_intc;

static void __iomem *sprd_pmu_apb_base_vaddr;
static void __iomem *sprd_ap_ahb_base_vaddr;
static void __iomem *sprd_aon_apb_base_vaddr;
static void __iomem *sprd_aon_iram0_base_vaddr;
static void __iomem *sprd_ap_uart1_base_vaddr;
static void __iomem *sprd_ap_apb_base_vaddr;

static struct ap_ahb_reg_bak ap_ahb_reg_saved;
static struct ap_apb_reg_bak ap_apb_reg_saved;
int is_intc_base_vaddr_ok;
int is_syscon_base_vaddr_ok;
static int is_deep_ap_clk_initialized;
static unsigned int sleep_cnt;

/* bits definition
 * bit_0 : ca7 top
 * bit_1 : ca7 c0
 * bit_2 : pub
 * bit_3 : mm
 * bit_4 : ap sys
 * bit_5 : dcdc arm
 */
#if 0
static struct auto_pd_en pd_config = {
	0x6a6aa6a6, 0x3f, 0xa6a66a6a,
	.pd_config_menu = {
		"ca7_top",
		"ca7_c0",
		"pub",
		"mm",
		"ap_sys",
		"dcdc_arm",
	},
};
#endif

struct deep_sleep_ap_clk_state {
	const char *name;
	struct clk *clk;
	struct clk *old_parent;
	struct clk *default_parent;
	unsigned long current_rate;
	unsigned int is_used;
};
static struct deep_sleep_ap_clk_state *deep_ap_clk_state;
static int clk_cnt;
static int get_ap_clk_count(void)
{
	int i, j;
	int count = 0;
	struct device_node *state_node;
	char *clk_group_name = "sprd,deep-ap-clk0";

	for (i = 0;; i++) {
		sprintf(clk_group_name, "sprd,deep-ap-clk%d", i);
		state_node = of_parse_phandle(deepsleep_np, clk_group_name, 0);

		if (!state_node)
			break;

		for (j = 0;; j++) {
			state_node = of_parse_phandle(
					deepsleep_np, clk_group_name, j);
			if (!state_node)
				break;

			count++;
		}

		pr_info("%s, clk_group_name is %s\n", __func__, clk_group_name);
	}

	return count;
}
static struct clk *get_ap_clk_default_parent_by_idx(int index)
{
	struct device_node *parent_node;
	struct clk *default_parent;
	const char *parent_clk_name = "default";

	parent_node = of_parse_phandle(
			deepsleep_np, "sprd,deep-ap-clkp", index);
	if (!parent_node) {
		pr_err("%s, Failed to find sprd,deep-ap-clkp node\n", __func__);
		return NULL;
	}

	of_property_read_string_index(parent_node, "clock-output-names", 0,
			&parent_clk_name);
	if (parent_clk_name == NULL) {
		pr_err("%s,Fail to get parent_clk[%d]\n", __func__, index);
		return NULL;
	}

	default_parent = clk_get(NULL, parent_clk_name);
	if (default_parent == NULL) {
		pr_err("%s,Fail to get default_parent[%d]\n", __func__, index);
		return NULL;
	}

	return default_parent;
}

static int sprd_deep_get_ap_clk(void)
{
	int i, j;
	int clk_idx = 0;
	struct device_node *state_node;
	char *clk_group_name = "sprd,deep-ap-clk0";

	pr_info("start %s\n", __func__);

	if (is_deep_ap_clk_initialized)
		return 0;

	if (IS_ERR(deepsleep_np)) {
		pr_err("%s, deepsleep_np is not initialized\n", __func__);
		return 1;
	}

	clk_cnt = get_ap_clk_count();
	if (!clk_cnt) {
		pr_err("%s, clk_cnt = 0\n", __func__);
		return 1;
	}
	pr_info("%s, get %d ap_clk\n", __func__, clk_cnt);

	deep_ap_clk_state = kcalloc(clk_cnt, sizeof(*deep_ap_clk_state),
			GFP_KERNEL);
	if (!deep_ap_clk_state)
		return 1;

	for (i = 0;; i++) {
		sprintf(clk_group_name, "sprd,deep-ap-clk%d", i);
		state_node = of_parse_phandle(deepsleep_np, clk_group_name, 0);
		if (!state_node)
			break;

		for (j = 0;; j++) {
			state_node = of_parse_phandle(
					deepsleep_np, clk_group_name, j);
			if (!state_node)
				break;

			deep_ap_clk_state[clk_idx].is_used = 0;

			of_property_read_string_index(state_node,
					"clock-output-names", 0,
					&(deep_ap_clk_state[clk_idx].name));
			if (deep_ap_clk_state[clk_idx].name == NULL) {
				pr_err("%s,Fail to get deep_ap_clk_state[%d].name\n",
						__func__, clk_idx);
				kfree(deep_ap_clk_state);
				deep_ap_clk_state = NULL;
				break;
			}

			deep_ap_clk_state[clk_idx].clk = clk_get(NULL,
					deep_ap_clk_state[clk_idx].name);
			if (deep_ap_clk_state[clk_idx].clk == NULL) {
				pr_err("%s,Fail to get deep_ap_clk_state[%d].clk\n",
						__func__, clk_idx);
				kfree(deep_ap_clk_state);
				deep_ap_clk_state = NULL;
				break;
			}

			deep_ap_clk_state[clk_idx].default_parent =
					get_ap_clk_default_parent_by_idx(i);
			if (deep_ap_clk_state[clk_idx].default_parent == NULL) {
				pr_err(
						"%s,Fail to get default parent of deep_ap_clk_state[%d]\n",
						__func__, clk_idx);
				kfree(deep_ap_clk_state);
				deep_ap_clk_state = NULL;
				break;
			}
			deep_ap_clk_state[clk_idx].is_used = 1;
			pr_info("%s,deep_ap_clk_state[%d].name is %s\n",
				__func__,
				clk_idx, deep_ap_clk_state[clk_idx].name);
			clk_idx++;
		}
	}

	if (deep_ap_clk_state)
		is_deep_ap_clk_initialized = 1;
	else {
		pr_err("%s deep_ap_clk_state is NULL\n", __func__);
		return 1;
	}

	pr_info("end %s\n", __func__);
	return 0;
}

static void sprd_deep_store_ap_clk(void)
{
	int i;

	pr_info("start %s\n", __func__);

	if (!deep_ap_clk_state) {
		pr_err("%s deep_ap_clk_state is NULL\n", __func__);
		return;
	}

	for (i = 0; i < clk_cnt; i++) {
		if (!(deep_ap_clk_state[i].is_used)) {
			pr_err("%s deep_ap_clk_state[%d].is_used =0\n",
				__func__, i);
			break;
		}

		deep_ap_clk_state[i].old_parent = clk_get_parent(
				deep_ap_clk_state[i].clk);
		deep_ap_clk_state[i].current_rate = clk_get_rate(
				deep_ap_clk_state[i].clk);
		pr_info("%s,bak %s\n", __func__, deep_ap_clk_state[i].name);
#ifdef AP_CLK_DEBUG
		pr_info("current_rate = %ld, parent_rat=%ld\n",
				deep_ap_clk_state[i].current_rate,
				clk_get_rate(deep_ap_clk_state[i].old_parent));
#endif
	}

	pr_info("end %s\n", __func__);
}

static void sprd_deep_restore_ap_clk(void)
{
	int i, j;

	pr_info("start %s\n", __func__);
	if (!deep_ap_clk_state) {
		pr_err("%s deep_ap_clk_state is NULL\n", __func__);
		return;
	}

	for (i = 0; i < clk_cnt; i++) {
		if (!(deep_ap_clk_state[i].is_used))
			break;

		clk_set_parent(deep_ap_clk_state[i].clk,
				deep_ap_clk_state[i].default_parent);
		clk_set_rate(deep_ap_clk_state[i].clk,
			clk_get_rate(deep_ap_clk_state[i].default_parent));
		for (j = 0; j < 10; j++)
			;
		clk_set_parent(deep_ap_clk_state[i].clk,
				deep_ap_clk_state[i].old_parent);
		clk_set_rate(deep_ap_clk_state[i].clk,
				deep_ap_clk_state[i].current_rate);

		pr_info("%s,resume %s\n", __func__, deep_ap_clk_state[i].name);
	}
#ifdef AP_CLK_DEBUG
	sprd_deep_store_ap_clk();
#endif

	pr_info("end %s\n", __func__);
}

static void setup_autopd_mode(void)
{
/*
*	regmap_update_bits(deepsleep_syscon_ap_ahb,
*		REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG,
*		BIT_AP_AHB_AP_AHB_AUTO_GATE_EN |
*		BIT_AP_AHB_AP_EMC_AUTO_GATE_EN |
*		BIT_AP_AHB_CA7_EMC_AUTO_GATE_EN |
*		BIT_AP_AHB_CA7_DBG_AUTO_GATE_EN |
*		BIT_AP_AHB_CA7_CORE_AUTO_GATE_EN,
*		BIT_AP_AHB_AP_AHB_AUTO_GATE_EN |
*		BIT_AP_AHB_AP_EMC_AUTO_GATE_EN |
*		BIT_AP_AHB_CA7_EMC_AUTO_GATE_EN |
*		BIT_AP_AHB_CA7_DBG_AUTO_GATE_EN |
*		BIT_AP_AHB_CA7_CORE_AUTO_GATE_EN);
*/
	regmap_update_bits(deepsleep_syscon_pmu_apb, REG_PMU_APB_PD_CA7_TOP_CFG,
			BIT_PMU_APB_PD_CA7_TOP_AUTO_SHUTDOWN_EN,
			BIT_PMU_APB_PD_CA7_TOP_AUTO_SHUTDOWN_EN);
	regmap_update_bits(deepsleep_syscon_pmu_apb, REG_PMU_APB_PD_CA7_C0_CFG,
			BIT_PMU_APB_PD_CA7_C0_AUTO_SHUTDOWN_EN,
			BIT_PMU_APB_PD_CA7_C0_AUTO_SHUTDOWN_EN);
	regmap_update_bits(deepsleep_syscon_pmu_apb, REG_PMU_APB_PD_AP_SYS_CFG,
			BIT_PMU_APB_PD_AP_SYS_AUTO_SHUTDOWN_EN,
			BIT_PMU_APB_PD_AP_SYS_AUTO_SHUTDOWN_EN);
	regmap_update_bits(deepsleep_syscon_pmu_apb, REG_PMU_APB_PD_MM_TOP_CFG,
			BIT_PMU_APB_PD_MM_TOP_AUTO_SHUTDOWN_EN,
			(~BIT_PMU_APB_PD_MM_TOP_AUTO_SHUTDOWN_EN));
	regmap_update_bits(deepsleep_syscon_pmu_apb, REG_PMU_APB_AP_DSLP_ENA,
			BIT_PMU_APB_AP_DSLP_ENA, BIT_PMU_APB_AP_DSLP_ENA);
	regmap_update_bits(deepsleep_syscon_ap_apb, REG_AP_APB_APB_EB,
			BIT_AP_APB_INTC3_EB | BIT_AP_APB_INTC2_EB |
			BIT_AP_APB_INTC1_EB | BIT_AP_APB_INTC0_EB,
			BIT_AP_APB_INTC3_EB | BIT_AP_APB_INTC2_EB |
			BIT_AP_APB_INTC1_EB | BIT_AP_APB_INTC0_EB);
	regmap_update_bits(deepsleep_syscon_aon_intc, INTCV_IRQ_EN,
			BIT(14) | BIT(4) | BIT(2), BIT(14) | BIT(4) | BIT(2));
	regmap_update_bits(deepsleep_syscon_ap_intc0,
			INTCV_IRQ_EN, 0x10007800,
			0x10007800);
	regmap_update_bits(deepsleep_syscon_ap_intc1,
			INTCV_IRQ_EN, 0x800068,
			0x800068);
	regmap_update_bits(deepsleep_syscon_ap_intc2,
			INTCV_IRQ_EN, 0x30,
			0x30);
	regmap_update_bits(deepsleep_syscon_ap_ahb,
			REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG,
			BIT_AP_AHB_AP_PERI_FORCE_ON,
			(unsigned int) (~BIT_AP_AHB_AP_PERI_FORCE_ON));
}
static void disable_ahb_module(void)
{
	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}
	regmap_update_bits(deepsleep_syscon_ap_ahb,
			REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG,
			BIT_AP_AHB_AP_MAINMTX_LP_DISABLE,
			BIT_AP_AHB_AP_MAINMTX_LP_DISABLE);
	regmap_update_bits(deepsleep_syscon_ap_ahb,
			REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG,
			BIT_AP_AHB_AP_PERI_FORCE_ON
			| BIT_AP_AHB_AP_PERI_FORCE_SLP,
			(~BIT_AP_AHB_AP_PERI_FORCE_ON)
			| BIT_AP_AHB_AP_PERI_FORCE_SLP);
}
static void bak_restore_ahb(int bak)
{
	u32 i;

	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}

	if (bak) {
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_AHB_EB,
				&(ap_ahb_reg_saved.ahb_eb));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_CA7_CKG_DIV_CFG,
				&(ap_ahb_reg_saved.ca7_ckg_div_cfg));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_MISC_CKG_EN,
				&(ap_ahb_reg_saved.misc_ckg_en));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_CA7_CKG_SEL_CFG,
				&(ap_ahb_reg_saved.ca7_ckg_sel_cfg));
		regmap_read(deepsleep_syscon_ap_ahb,
				REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG,
				&(ap_ahb_reg_saved.force_sleep_cfg));
		regmap_read(deepsleep_syscon_ap_ahb,
				REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG,
				&(ap_ahb_reg_saved.auto_sleep_cfg));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_OTG_PHY_TUNE,
				&(ap_ahb_reg_saved.otg_phy_tune));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_OTG_PHY_TEST,
				&(ap_ahb_reg_saved.otg_phy_test));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_OTG_PHY_CTRL,
				&(ap_ahb_reg_saved.otg_phy_ctrl));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_OTG_CTRL0,
				&(ap_ahb_reg_saved.otg_ctrl0));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_OTG_CTRL1,
				&(ap_ahb_reg_saved.otg_ctrl1));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_M0_LPC,
				&(ap_ahb_reg_saved.m0_lpc));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_M1_LPC,
				&(ap_ahb_reg_saved.m1_lpc));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_M2_LPC,
				&(ap_ahb_reg_saved.m2_lpc));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_M3_LPC,
				&(ap_ahb_reg_saved.m3_lpc));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_M4_LPC,
				&(ap_ahb_reg_saved.m4_lpc));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_M5_LPC,
				&(ap_ahb_reg_saved.m5_lpc));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_M6_LPC,
				&(ap_ahb_reg_saved.m6_lpc));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_M7_LPC,
				&(ap_ahb_reg_saved.m7_lpc));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_M8_LPC,
				&(ap_ahb_reg_saved.m8_lpc));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_M9_LPC,
				&(ap_ahb_reg_saved.m9_lpc));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_MAIN_LPC,
				&(ap_ahb_reg_saved.main_lpc));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_S0_LPC,
				&(ap_ahb_reg_saved.s0_lpc));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_S1_LPC,
				&(ap_ahb_reg_saved.s1_lpc));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_S2_LPC,
				&(ap_ahb_reg_saved.s2_lpc));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_S3_LPC,
				&(ap_ahb_reg_saved.s3_lpc));
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_S4_LPC,
				&(ap_ahb_reg_saved.s4_lpc));
	} else {
		regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_AHB_EB,
				ap_ahb_reg_saved.ahb_eb);
		regmap_write(deepsleep_syscon_ap_ahb,
				REG_AP_AHB_CA7_CKG_DIV_CFG,
				(ap_ahb_reg_saved.ca7_ckg_div_cfg & (~0x7)));
		for (i = 0; i < 20; i++)
			;
		regmap_write(deepsleep_syscon_ap_ahb,
				REG_AP_AHB_CA7_CKG_DIV_CFG,
				ap_ahb_reg_saved.ca7_ckg_div_cfg);
		regmap_write(deepsleep_syscon_ap_ahb,
				REG_AP_AHB_MISC_CKG_EN,
				ap_ahb_reg_saved.misc_ckg_en);
		regmap_write(deepsleep_syscon_ap_ahb,
				REG_AP_AHB_CA7_CKG_SEL_CFG,
				(ap_ahb_reg_saved.ca7_ckg_sel_cfg & (~0x7)));
		for (i = 0; i < 20; i++)
			;
		regmap_write(deepsleep_syscon_ap_ahb,
				REG_AP_AHB_CA7_CKG_SEL_CFG,
				ap_ahb_reg_saved.ca7_ckg_sel_cfg);
		regmap_write(deepsleep_syscon_ap_ahb,
				REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG,
				ap_ahb_reg_saved.force_sleep_cfg);
		regmap_write(deepsleep_syscon_ap_ahb,
				REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG,
				ap_ahb_reg_saved.auto_sleep_cfg);
		regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_OTG_PHY_TUNE,
				ap_ahb_reg_saved.otg_phy_tune);
		regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_OTG_PHY_TEST,
				ap_ahb_reg_saved.otg_phy_test);
		regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_OTG_PHY_CTRL,
				ap_ahb_reg_saved.otg_phy_ctrl);
		regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_OTG_CTRL0,
				ap_ahb_reg_saved.otg_ctrl0);
		regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_OTG_CTRL1,
				ap_ahb_reg_saved.otg_ctrl1);
		regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_M0_LPC,
				ap_ahb_reg_saved.m0_lpc);
		regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_M1_LPC,
				ap_ahb_reg_saved.m1_lpc);
		regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_M2_LPC,
				ap_ahb_reg_saved.m2_lpc);
		regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_M3_LPC,
				ap_ahb_reg_saved.m3_lpc);
		regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_M4_LPC,
				ap_ahb_reg_saved.m4_lpc);
		regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_M5_LPC,
				ap_ahb_reg_saved.m5_lpc);
		regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_M6_LPC,
				ap_ahb_reg_saved.m6_lpc);
		regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_M7_LPC,
				ap_ahb_reg_saved.m7_lpc);
		regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_M8_LPC,
				ap_ahb_reg_saved.m8_lpc);
		regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_M9_LPC,
				ap_ahb_reg_saved.m9_lpc);
		regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_MAIN_LPC,
				ap_ahb_reg_saved.main_lpc);
		regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_S0_LPC,
				ap_ahb_reg_saved.s0_lpc);
		regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_S1_LPC,
				ap_ahb_reg_saved.s1_lpc);
		regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_S2_LPC,
				ap_ahb_reg_saved.s2_lpc);
		regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_S3_LPC,
				ap_ahb_reg_saved.s3_lpc);
		regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_S4_LPC,
				ap_ahb_reg_saved.s4_lpc);
	}
}

static void disable_apb_module(void)
{
	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}
}

static void bak_restore_apb(int bak)
{
	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}

	if (bak) {
		regmap_read(deepsleep_syscon_ap_apb, REG_AP_APB_APB_EB,
				&(ap_apb_reg_saved.apb_eb));
		regmap_read(deepsleep_syscon_ap_apb, REG_AP_APB_APB_MISC_CTRL,
				&(ap_apb_reg_saved.apb_misc_ctrl));
	} else {
		regmap_write(deepsleep_syscon_ap_apb, REG_AP_APB_APB_EB,
				ap_apb_reg_saved.apb_eb);
		regmap_write(deepsleep_syscon_ap_apb, REG_AP_APB_APB_MISC_CTRL,
				ap_apb_reg_saved.apb_misc_ctrl);
	}
}

static void bak_ap_clk_reg(int bak)
{
	if (!is_deep_ap_clk_initialized) {
		pr_err("%s,ap clk is not initialized\n", __func__);
		return;
	}

	if (bak)
		sprd_deep_store_ap_clk();
	else
		sprd_deep_restore_ap_clk();
}

static void disable_aon_module(void)
{
	regmap_update_bits(deepsleep_syscon_aon_apb, REG_AON_APB_APB_EB0,
			BIT_AON_TMR_EB | BIT_AP_TMR0_EB,
			(unsigned int) (~(BIT_AON_TMR_EB | BIT_AP_TMR0_EB)));
	regmap_update_bits(deepsleep_syscon_aon_apb, REG_AON_APB_APB_EB1,
			BIT_AP_TMR2_EB | BIT_AP_TMR1_EB,
			(unsigned int) (~(BIT_AP_TMR2_EB | BIT_AP_TMR1_EB)));
	regmap_update_bits(deepsleep_syscon_aon_apb, REG_AON_APB_APB_EB0,
			BIT_AON_APB_CA7_DAP_EB, BIT_AON_APB_CA7_DAP_EB);
	regmap_update_bits(deepsleep_syscon_aon_apb, REG_AON_APB_APB_EB2,
			BIT_AON_APB_AP_DAP_EB,
			(unsigned int) (~BIT_AON_APB_AP_DAP_EB));
	regmap_update_bits(deepsleep_syscon_aon_intc, INTCV_IRQ_EN,
			BIT(2) | BIT(4) | BIT(5) | BIT(11) | BIT(12) | BIT(14)
			| BIT(30) | BIT(31),
			BIT(2) | BIT(4) | BIT(5) | BIT(11) | BIT(12) | BIT(14)
			| BIT(30) | BIT(31));
}

static void bak_restore_aon(int bak)
{
	static uint32_t apb_eb0, apb_eb1;

	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}

	if (bak) {
		regmap_read(
				deepsleep_syscon_aon_apb,
				REG_AON_APB_APB_EB0, &apb_eb0);
		regmap_read(
				deepsleep_syscon_aon_apb,
				REG_AON_APB_APB_EB1, &apb_eb1);
	} else {
		regmap_write(
				deepsleep_syscon_aon_apb,
				REG_AON_APB_APB_EB0, apb_eb0);
		regmap_write(
				deepsleep_syscon_aon_apb,
				REG_AON_APB_APB_EB1, apb_eb1);
		regmap_update_bits(deepsleep_syscon_aon_apb,
				REG_AON_APB_APB_EB2,
				BIT_AON_APB_AP_DAP_EB,
				(unsigned int) (BIT_AON_APB_AP_DAP_EB));
	}
}

static void bak_restore_ap_intc(int bak)
{
	static u32 intc0_eb, intc1_eb, intc2_eb, intc3_eb;

	if (bak) {
		regmap_read(deepsleep_syscon_ap_intc0, INTCV_IRQ_EN, &intc0_eb);
		regmap_read(deepsleep_syscon_ap_intc1, INTCV_IRQ_EN, &intc1_eb);
		regmap_read(deepsleep_syscon_ap_intc2, INTCV_IRQ_EN, &intc2_eb);
		regmap_read(deepsleep_syscon_ap_intc3, INTCV_IRQ_EN, &intc3_eb);
	} else {
		regmap_write(deepsleep_syscon_ap_intc0, INTCV_IRQ_EN, intc0_eb);
		regmap_write(deepsleep_syscon_ap_intc1, INTCV_IRQ_EN, intc1_eb);
		regmap_write(deepsleep_syscon_ap_intc2, INTCV_IRQ_EN, intc2_eb);
		regmap_write(deepsleep_syscon_ap_intc3, INTCV_IRQ_EN, intc3_eb);
	}

}

static void enable_mcu_deep_sleep(void)
{
	regmap_update_bits(deepsleep_syscon_ap_ahb, REG_AP_AHB_MCU_PAUSE,
			BIT_AP_AHB_MCU_DEEP_SLEEP_EN |
			BIT_AP_AHB_MCU_LIGHT_SLEEP_EN |
			BIT_AP_AHB_MCU_SYS_SLEEP_EN,
			BIT_AP_AHB_MCU_DEEP_SLEEP_EN |
			BIT_AP_AHB_MCU_LIGHT_SLEEP_EN |
			BIT_AP_AHB_MCU_SYS_SLEEP_EN);
}

static void disable_mcu_deep_sleep(void)
{
	regmap_update_bits(deepsleep_syscon_ap_ahb, REG_AP_AHB_MCU_PAUSE,
			BIT_AP_AHB_MCU_DEEP_SLEEP_EN |
			BIT_AP_AHB_MCU_LIGHT_SLEEP_EN |
			BIT_AP_AHB_MCU_SYS_SLEEP_EN,
			(unsigned int) (~(BIT_AP_AHB_MCU_DEEP_SLEEP_EN
					| BIT_AP_AHB_MCU_LIGHT_SLEEP_EN
					| BIT_AP_AHB_MCU_SYS_SLEEP_EN)));
}

unsigned int sprd_irq_pending(void)
{
	uint32_t bits = 0;

	regmap_read(deepsleep_syscon_aon_intc, INTCV_IRQ_MSKSTS, &bits);
	if (bits)
		return 1;
	else
		return 0;
}

/*copy code for deepsleep return */
#define SAVED_VECTOR_SIZE 64
static uint32_t *sp_pm_reset_vector;
static void __iomem *iram_start;

#define SPRD_RESET_VECTORS 0X00000000
#define SLEEP_RESUME_CODE_PHYS	0X400
#define SLEEP_CODE_SIZE 0x800
#define SPRD_IRAM0_PHYS		0x0

static int init_reset_vector(void)
{
	sp_pm_reset_vector = (uint32_t __force *) sprd_aon_iram0_base_vaddr;

	iram_start = (sprd_aon_iram0_base_vaddr
			+ SLEEP_RESUME_CODE_PHYS);
	/* copy sleep code to (IRAM). */
	if (((u32) sc8830_standby_iram_end -
		(u32) sc8830_standby_iram) > SLEEP_CODE_SIZE)
		panic("##: code size is larger than expected, need more memory!\n");

	memcpy_toio(iram_start, sc8830_standby_iram, SLEEP_CODE_SIZE);
	/* just make sure*/
	flush_cache_all();
	outer_flush_all();
	sp_pm_reset_vector[65] = virt_to_phys(sp_pm_collapse_exit);
	return 0;
}

void set_reset_vector(void)
{
	int i = 0;

	pr_info("SAVED_VECTOR_SIZE 0x%x\n", SAVED_VECTOR_SIZE);
	for (i = 0; i < SAVED_VECTOR_SIZE; i++)
		sp_pm_reset_vector[i] = 0xe320f000; /* nop*/

	sp_pm_reset_vector[SAVED_VECTOR_SIZE - 2] = 0xE51FF004; /* ldr pc, 4 */
	/* place v7_standby_iram here */
	sp_pm_reset_vector[SAVED_VECTOR_SIZE - 1] =
			((u32) sc8830_standby_exit_iram
			- (u32) sc8830_standby_iram + SLEEP_RESUME_CODE_PHYS);
}

/* irq functions */
#define hw_raw_irqs_disabled_flags(flags)	\
({						\
	(int)((flags) & PSR_I_BIT);		\
})

#define hw_irqs_disabled()			\
({						\
	unsigned long _flags;			\
	local_irq_save(_flags);			\
	hw_raw_irqs_disabled_flags(_flags);	\
})

u32 __attribute__ ((naked)) read_cpsr(void)
{
	__asm__ __volatile__("mrs r0, cpsr\nbx lr");
}

/* make sure printk is end, if not maybe some messy code  in SERIAL1 output */
#define UART_TRANSFER_REALLY_OVER (0x1UL << 15)
#define UART_STS0 (SPRD_UART1_BASE + 0x08)
#define UART_STS1 (SPRD_UART1_BASE + 0x0c)

#define SAVE_GLOBAL_REG do { \
	bak_restore_apb(1); \
	bak_ap_clk_reg(1); \
	bak_restore_ahb(1); \
	bak_restore_aon(1); \
	bak_restore_ap_intc(1);\
	} while (0)
#define RESTORE_GLOBAL_REG do { \
	bak_restore_apb(0); \
	bak_ap_clk_reg(0); \
	bak_restore_aon(0); \
	bak_restore_ahb(0); \
	bak_restore_ap_intc(0);\
	} while (0)

void show_pin_reg(void)
{
	uint32_t xtl0_rel_cfg, xtlbuf0_rel_cfg, xtlbuf1_rel_cfg;

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
	pr_info("#######PIN REG#######\n");
	pr_info("##--REG_PMU_APB_XTL0_REL_CFG : 0x%08x\n", xtl0_rel_cfg);
	pr_info("##--REG_PMU_APB_XTLBUF0_REL_CFG : 0x%08x\n", xtlbuf0_rel_cfg);
	pr_info("##--REG_PMU_APB_XTLBUF1_REL_CFG : 0x%08x\n", xtlbuf1_rel_cfg);
}

typedef u32 (*iram_standby_entry_ptr)(u32, u32);
static int sc_sleep_call(unsigned long flag)
{
	u32 ret = 0;
	iram_standby_entry_ptr func_ptr;

	regmap_write(deepsleep_syscon_pmu_apb,
			REG_PMU_APB_PD_CA7_C0_SHUTDOWN_MARK_STATUS,
			0xF);

	regmap_write(deepsleep_syscon_ap_ahb,
			REG_AP_AHB_CA7_CKG_SEL_CFG,
			0x0);

	func_ptr = (iram_standby_entry_ptr) (SLEEP_RESUME_CODE_PHYS
			+ (u32) sp_pm_reset_vector
			+ (u32) sp_pm_collapse - (u32) sc8830_standby_iram);
	ret = (func_ptr)(0, flag);

	return ret;
}
static void __iomem *sprd_deepsleep_get_iomem(struct device_node *from,
		const char *type, const char *compatible)
{
	struct device_node *node;
	void __iomem *base;

	node = of_find_compatible_node(from, type, compatible);
	if (!node) {
		pr_err("%s: can't find node\n", __func__);
		return NULL;
	}

	base = of_iomap(node, 0);
	if (!base) {
		pr_err("%s: iomap node error\n", __func__);
		return NULL;
	}

	return base;
}
static int deep_sleep(int from_idle)
{
	int ret = 0;

	if (!from_idle) {
		SAVE_GLOBAL_REG;
		show_pin_reg();
		disable_ahb_module();
		disable_aon_module();
		disable_apb_module();
		enable_mcu_deep_sleep();
		regmap_update_bits(deepsleep_syscon_pmu_apb,
				REG_PMU_APB_CA7_C0_CFG,
				BIT_PMU_APB_CA7_VINITHI_C0,
				(unsigned int) (~BIT_PMU_APB_CA7_VINITHI_C0));
		deepsleep_condition_check();
		bak_last_reg();
		print_last_reg();
		print_int_status();
	}
#ifdef CONFIG_TRUSTY
	pr_info("entry trusty_fast_call32_power - SMC_FC_CPU_SUSPEND\n");
	ret = trusty_fast_call32_power(SMC_FC_CPU_SUSPEND, 0, 0, 0);
	if (ret <= 0)
		pr_info("trusty_fast_call32_power error!\n");
	pr_info("exit trusty_fast_call32_power - SMC_FC_CPU_SUSPEND\n");
#endif
	ret = sc_sleep_call(from_idle);
	udelay(50);

	if (!from_idle) {
		regmap_update_bits(deepsleep_syscon_pmu_apb,
				REG_PMU_APB_CA7_C0_CFG,
				BIT_PMU_APB_CA7_VINITHI_C0,
				BIT_PMU_APB_CA7_VINITHI_C0);
		pr_info("ret %d not from idle\n", ret);
		if (ret) {
			pr_info("deep sleep %u times\n", sleep_cnt);
			sleep_cnt++;
		}
		if (!IS_ERR(deepsleep_syscon_ap_apb)) {
			regmap_update_bits(deepsleep_syscon_ap_apb,
					REG_AP_APB_APB_EB,
				BIT_AP_APB_INTC3_EB | BIT_AP_APB_INTC2_EB
				| BIT_AP_APB_INTC1_EB | BIT_AP_APB_INTC0_EB,
				BIT_AP_APB_INTC3_EB | BIT_AP_APB_INTC2_EB
				| BIT_AP_APB_INTC1_EB | BIT_AP_APB_INTC0_EB);
			hard_irq_set();
		}
		disable_mcu_deep_sleep();
		RESTORE_GLOBAL_REG;
	} else {
		pr_info("ret %d  from idle\n", ret);
	}

	udelay(5);
	if (ret)
		cpu_init();
	regmap_update_bits(deepsleep_syscon_ap_ahb,
			REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG,
			BIT_AP_AHB_CA7_CORE_AUTO_GATE_EN,
			~BIT_AP_AHB_CA7_CORE_AUTO_GATE_EN);
	return ret;
}

int sprd_cpu_deep_sleep(unsigned int cpu)
{
	int ret = 0;
	unsigned long flags;

	if (!hw_irqs_disabled()) {
		flags = read_cpsr();
		pr_info("##: Error(%s): IRQ is enabled(%08lx)!\n",
				"wakelock_suspend",
				flags);
	}
	set_sleep_mode(SLP_MODE_DEP);
	ret = deep_sleep(0);
	flush_cache_all();

	print_hard_irq_inloop(ret);
	return ret;
}

static void resume_code_remap(void)
{
	struct resource res;
	struct device_node *np;

	sprd_aon_iram0_base_vaddr = NULL;
	np = of_parse_phandle(deepsleep_np, "sprd,sys-aon-iram0", 0);
	if (!np) {
		pr_err("%s: get aon iram0 error\n", __func__);
		return;
	}

	if (of_address_to_resource(np, 0, &res)) {
		pr_err("aon iram0 address error\n");
		return;
	}

	sprd_aon_iram0_base_vaddr = __arm_ioremap_exec(
			res.start, res.end - res.start + 1, true);
	if (sprd_aon_iram0_base_vaddr == NULL)
		pr_err("resume_code_remap err");
}

static void sprd_deepsleep_init_regmap(void)
{
	is_intc_base_vaddr_ok = 0;
	is_syscon_base_vaddr_ok = 0;
	pr_info("###### deepsleep_init_regmap start ###\n");

	deepsleep_np = of_find_node_by_name(NULL, "deep-sleep");
	if (IS_ERR(deepsleep_np)) {
		pr_err("%s, failed to find deepsleep_np\n", __func__);
		is_syscon_base_vaddr_ok = 1;
	}

	deepsleep_syscon_ap_ahb = syscon_regmap_lookup_by_phandle(
			deepsleep_np,
			"sprd,sys-ap-ahb");
	if ((!deepsleep_syscon_ap_ahb) || IS_ERR(deepsleep_syscon_ap_ahb)) {
		pr_err("%s,failed to find deepsleep_syscon_ap_ahb\n",
				__func__);
		is_syscon_base_vaddr_ok = 1;
	}
	deepsleep_syscon_pmu_apb = syscon_regmap_lookup_by_phandle(
			deepsleep_np,
			"sprd,sys-pmu-apb");
	if ((!deepsleep_syscon_pmu_apb) || IS_ERR(deepsleep_syscon_pmu_apb)) {
		pr_err("%s,failed to find deepsleep_syscon_pmu_apb\n",
				__func__);
		is_syscon_base_vaddr_ok = 1;
	}
	deepsleep_syscon_aon_apb = syscon_regmap_lookup_by_phandle(
			deepsleep_np,
			"sprd,sys-aon-apb");
	if ((!deepsleep_syscon_aon_apb) || IS_ERR(deepsleep_syscon_aon_apb)) {
		pr_err("%s,failed find to deepsleep_syscon_aon_apb\n",
				__func__);
		is_syscon_base_vaddr_ok = 1;
	}
	deepsleep_syscon_ap_apb = syscon_regmap_lookup_by_phandle(
			deepsleep_np,
			"sprd,sys-ap-apb");
	if ((!deepsleep_syscon_ap_apb) || IS_ERR(deepsleep_syscon_ap_apb)) {
		pr_err("%s,failed to find deepsleep_syscon_ap_apb\n",
				__func__);
		is_syscon_base_vaddr_ok = 1;
	}

	deepsleep_syscon_ap_intc0 = syscon_regmap_lookup_by_phandle(
			deepsleep_np,
			"sprd,sys-ap-intc0");
	if ((!deepsleep_syscon_ap_intc0) || IS_ERR(deepsleep_syscon_ap_intc0)) {
		pr_err("%s,failed to find deepsleep_syscon_ap_intc0\n",
				__func__);
		is_intc_base_vaddr_ok = 1;
	}

	deepsleep_syscon_ap_intc1 = syscon_regmap_lookup_by_phandle(
			deepsleep_np,
			"sprd,sys-ap-intc1");
	if ((!deepsleep_syscon_ap_intc1) || IS_ERR(deepsleep_syscon_ap_intc1)) {
		pr_err("%s,failed to find deepsleep_syscon_ap_intc1\n",
				__func__);
		is_intc_base_vaddr_ok = 1;
	}

	deepsleep_syscon_ap_intc2 = syscon_regmap_lookup_by_phandle(
			deepsleep_np,
			"sprd,sys-ap-intc2");
	if ((!deepsleep_syscon_ap_intc2) || IS_ERR(deepsleep_syscon_ap_intc2)) {
		pr_err("%s,failed to find deepsleep_syscon_ap_intc2\n",
				__func__);
		is_intc_base_vaddr_ok = 1;
	}

	deepsleep_syscon_ap_intc3 = syscon_regmap_lookup_by_phandle(
			deepsleep_np,
			"sprd,sys-ap-intc3");
	if ((!deepsleep_syscon_ap_intc3) || IS_ERR(deepsleep_syscon_ap_intc3)) {
		pr_err("%s,failed to find deepsleep_syscon_ap_intc3\n",
				__func__);
		is_intc_base_vaddr_ok = 1;
	}

	deepsleep_syscon_aon_intc = syscon_regmap_lookup_by_phandle(
			deepsleep_np,
			"sprd,sys-aon-intc");
	if ((!deepsleep_syscon_aon_intc) || IS_ERR(deepsleep_syscon_aon_intc)) {
		pr_err("%s,failed to find deepsleep_syscon_aon_intc\n",
				__func__);
		is_intc_base_vaddr_ok = 1;
	}

	sprd_pmu_apb_base_vaddr = sprd_deepsleep_get_iomem(NULL, NULL,
			"sprd,sys-pmu-apb");
	if (sprd_pmu_apb_base_vaddr == NULL)
		pr_err("%s: get pmu apb io memory error\n", __func__);

	sprd_ap_ahb_base_vaddr = sprd_deepsleep_get_iomem(NULL, NULL,
			"sprd,sys-ap-ahb");
	if (sprd_ap_ahb_base_vaddr == NULL)
		pr_err("%s: get ap ahb io memory error\n", __func__);

	sprd_aon_apb_base_vaddr = sprd_deepsleep_get_iomem(NULL, NULL,
			"sprd,sys-aon-apb");
	if (sprd_ap_ahb_base_vaddr == NULL)
		pr_err("%s: get aon apb io memory error\n", __func__);

	sprd_ap_apb_base_vaddr = sprd_deepsleep_get_iomem(NULL, NULL,
			"sprd,sys-ap-apb");
	if (sprd_ap_apb_base_vaddr == NULL)
		pr_err("%s: get ap apb io memory error\n", __func__);

	sprd_ap_uart1_base_vaddr = sprd_deepsleep_get_iomem(NULL, NULL,
			"sprd,ap-uart1");
	if (sprd_ap_uart1_base_vaddr == NULL)
		pr_err("%s: get ap uart1 io memory error\n", __func__);

	reg_sprd_uart1_base = sprd_ap_uart1_base_vaddr;
	reg_uart1_enable_phy = 0x71300000;
	reg_uart1_enable_vir = sprd_ap_apb_base_vaddr;

	pr_info("###### deepsleep_init_regmap end ###\n");
}

void __init sc_pm_init(void)
{
	sprd_deepsleep_init_regmap();
	resume_code_remap();
	if (sprd_deep_get_ap_clk())
		pr_err("%s,failed to find ap clk\n", __func__);
	init_reset_vector();
	setup_autopd_mode();
	/* disable all sleep mode */
	regmap_update_bits(deepsleep_syscon_ap_ahb,
			REG_AP_AHB_MCU_PAUSE,
			BIT_AP_AHB_MCU_DEEP_SLEEP_EN |
			BIT_AP_AHB_MCU_LIGHT_SLEEP_EN |
			BIT_AP_AHB_MCU_SYS_SLEEP_EN |
			BIT_AP_AHB_MCU_CORE_SLEEP,
			(unsigned int)(~(BIT_AP_AHB_MCU_DEEP_SLEEP_EN |
			BIT_AP_AHB_MCU_LIGHT_SLEEP_EN |
			BIT_AP_AHB_MCU_SYS_SLEEP_EN |
			BIT_AP_AHB_MCU_CORE_SLEEP)));
	set_reset_vector();
#ifndef CONFIG_SPRD_PM_DEBUG
	pm_debug_init();
#endif
	sleep_cnt = 0;
}

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
#include <asm/suspend.h>
#include <asm/barrier.h>
#include <linux/cpu_pm.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/mfd/syscon.h>
#include <asm/mwait.h>
#include <linux/pm.h>
#include <asm/mwait.h>
#include <asm/mv/mobilevisor.h>
#include <asm/mv/mv_gal.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/freq-vid.h>
#include "../sleep.h"
#include "pm_debug_x86_64.h"
#include "pm_x86_64.h"
static struct device_node *deepsleep_np;
struct regmap *deep_sys_ap_ahb;
struct regmap *deep_sys_aon_pwu_apb;
struct regmap *deep_sys_aon_apb;
struct regmap *deep_sys_agcp_ahb;
struct regmap *deep_sys_ap_apb;
struct regmap *deep_sys_com_pmu_apb;
struct regmap *deepsleep_syscon_aon_intc0;
struct regmap *deepsleep_syscon_aon_intc1;
struct regmap *deepsleep_syscon_aon_intc2;
struct regmap *deepsleep_syscon_aon_intc3;

static void __iomem *deep_bia_vd_cpu0;
static void __iomem *deep_bia_vd_cpu1;

int is_syscon_base_vaddr_ok;
static int is_ioremap_cpu0_vaddr_ok;
static int is_ioremap_cpu1_vaddr_ok;
static struct ap_ahb_reg_bak ap_ahb_reg_saved;
static struct ap_apb_reg_bak ap_apb_reg_saved;
static int is_deep_ap_clk_initialized;

struct regmap *deep_sys_bia_intc;

static struct	deep_sleep_ap_clk_state	*deep_ap_clk_state;
static int clk_cnt;
struct deep_sleep_ap_clk_state {
	const char	*name;
	struct	clk	*clk;
	struct	clk	*old_parent;
	struct	clk	*default_parent;
	unsigned long	current_rate;
	unsigned int	is_used;
};

static	int get_ap_clk_count(void)
{
	int i, j;
	int count = 0;
	struct device_node *state_node;
	char	*clk_group_name = "sprd,deep-ap-clk0";

	for (i = 0; ; i++) {
		sprintf(clk_group_name, "sprd,deep-ap-clk%d", i);
		state_node = of_parse_phandle(deepsleep_np,
				clk_group_name, 0);

		if (!state_node)
			break;

		for (j = 0; ; j++) {
			state_node = of_parse_phandle(deepsleep_np,
				clk_group_name, j);
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
	struct clk	*default_parent;
	const char	*parent_clk_name = "default";

	parent_node = of_parse_phandle(deepsleep_np,
					"sprd,deep-ap-clkp", index);
	if (!parent_node) {
		pr_err("%s, Failed to find sprd,deep-ap-clkp node\n", __func__);
		return NULL;
	}

	of_property_read_string_index(parent_node, "clock-output-names",
					 0, &parent_clk_name);
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
	char	*clk_group_name = "sprd,deep-ap-clk0";

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

	for (i = 0; ; i++) {
		sprintf(clk_group_name, "sprd,deep-ap-clk%d", i);
		state_node = of_parse_phandle(deepsleep_np,
				clk_group_name, 0);
		if (!state_node)
			break;

		for (j = 0; ; j++) {
			state_node = of_parse_phandle(deepsleep_np,
				clk_group_name, j);
			if (!state_node)
				break;

			if (deep_ap_clk_state == NULL)
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
				pr_err("%s,Fail to get default parent of deep_ap_clk_state[%d]\n",
					__func__, clk_idx);
				kfree(deep_ap_clk_state);
				deep_ap_clk_state = NULL;
				break;
			}
			deep_ap_clk_state[clk_idx].is_used = 1;
			pr_info("%s,deep_ap_clk_state[%d].name is %s\n",
				__func__, clk_idx,
				deep_ap_clk_state[clk_idx].name);
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

		deep_ap_clk_state[i].old_parent =
			clk_get_parent(deep_ap_clk_state[i].clk);
		deep_ap_clk_state[i].current_rate =
			clk_get_rate(deep_ap_clk_state[i].clk);
		pr_info("%s,bak %s\n", __func__, deep_ap_clk_state[i].name);
		pr_info("current_rate = %ld, parent_rat=%ld\n",
			deep_ap_clk_state[i].current_rate,
			clk_get_rate(deep_ap_clk_state[i].old_parent));
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

		pr_debug("%s,resume %s\n", __func__, deep_ap_clk_state[i].name);
	}
	sprd_deep_store_ap_clk();
	pr_info("end %s\n", __func__);
}


static void disable_ahb_module(void)
{
	if (IS_ERR(deep_sys_ap_ahb)) {
		pr_err("%s,failed to find deep_sys_ap_ahb\n", __func__);
		return;
	}

	/* AP_PERI_FORCE_SLP */
	regmap_update_bits(deep_sys_ap_ahb,
		REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG,
		BIT_AP_AHB_AP_PERI_FRC_OFF |
		BIT_AP_AHB_DMA_FRC_SLEEP,
		BIT_AP_AHB_AP_PERI_FRC_OFF |
		BIT_AP_AHB_DMA_FRC_SLEEP);
}


static void bak_restore_ahb(int bak)
{
	if (IS_ERR(deep_sys_ap_ahb)) {
		pr_err("%s,failed to find deep_sys_ap_ahb\n", __func__);
		return;
	}

	if (bak) {
		regmap_read(deep_sys_ap_ahb, REG_AP_AHB_AHB_EB,
			&(ap_ahb_reg_saved.ahb_eb));
		regmap_read(deep_sys_ap_ahb, REG_AP_AHB_MISC_CFG,
			&(ap_ahb_reg_saved.misc_cfg));
		regmap_read(deep_sys_ap_ahb, REG_AP_AHB_PUB0_FRC_LSLP,
			&(ap_ahb_reg_saved.pub0_frc_lslp));
		regmap_read(deep_sys_ap_ahb, REG_AP_AHB_NOC_CTRL3,
			&(ap_ahb_reg_saved.noc_ctrl3));
	} else {
		regmap_write(deep_sys_ap_ahb, REG_AP_AHB_AHB_EB,
			ap_ahb_reg_saved.ahb_eb);
		regmap_write(deep_sys_ap_ahb, REG_AP_AHB_MISC_CFG,
			ap_ahb_reg_saved.misc_cfg);
		regmap_write(deep_sys_ap_ahb, REG_AP_AHB_PUB0_FRC_LSLP,
			ap_ahb_reg_saved.pub0_frc_lslp);
		regmap_write(deep_sys_ap_ahb, REG_AP_AHB_NOC_CTRL3,
			ap_ahb_reg_saved.noc_ctrl3);
	}
}


static void bak_restore_apb(int bak)
{
	if (IS_ERR(deep_sys_ap_apb)) {
		pr_err("%s,failed to find deep_sys_ap_apb\n", __func__);
		return;
	}

	if (bak)
		regmap_read(deep_sys_ap_apb, REG_AP_APB_APB_EB,
			&(ap_apb_reg_saved.apb_eb));
	else
		regmap_write(deep_sys_ap_apb, REG_AP_APB_APB_EB,
			ap_apb_reg_saved.apb_eb);
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
	if (IS_ERR(deep_sys_aon_apb)) {
		pr_err("%s,failed to find deep_sys_ap_apb\n", __func__);
		return;
	}

	regmap_update_bits(deep_sys_aon_apb,
		REG_AON_APB_APB_EB0,
		BIT_AON_APB_AON_TMR_EB |
		BIT_AON_APB_AP_TMR0_EB,
		(unsigned int)(~(BIT_AON_APB_AON_TMR_EB |
		BIT_AON_APB_AP_TMR0_EB)));
	regmap_update_bits(deep_sys_aon_apb,
		REG_AON_APB_APB_EB1,
		BIT_AON_APB_AP_TMR2_EB |
		BIT_AON_APB_AP_TMR1_EB,
		(unsigned int)(~(BIT_AON_APB_AP_TMR2_EB |
		BIT_AON_APB_AP_TMR1_EB)));
	regmap_update_bits(deep_sys_aon_apb,
		REG_AON_APB_APB_EB1,
		BIT_AON_APB_DISP_EMC_EB,
		(unsigned int)(~BIT_AON_APB_DISP_EMC_EB));
}

static void enable_aon_module(void)
{
	if (IS_ERR(deep_sys_aon_apb)) {
		pr_err("%s,failed to find deep_sys_ap_apb\n", __func__);
		return;
	}

	regmap_update_bits(deep_sys_aon_apb,
		REG_AON_APB_APB_EB0,
		BIT_AON_APB_AON_TMR_EB |
		BIT_AON_APB_AP_TMR0_EB,
		BIT_AON_APB_AON_TMR_EB |
		BIT_AON_APB_AP_TMR0_EB);

	regmap_update_bits(deep_sys_aon_apb,
		REG_AON_APB_APB_EB1,
		BIT_AON_APB_AP_TMR2_EB |
		BIT_AON_APB_AP_TMR1_EB,
		BIT_AON_APB_AP_TMR2_EB |
		BIT_AON_APB_AP_TMR1_EB);

	regmap_update_bits(deep_sys_aon_apb,
		REG_AON_APB_APB_EB1,
		BIT_AON_APB_DISP_EMC_EB,
		BIT_AON_APB_DISP_EMC_EB);
}


#define SAVE_GLOBAL_REG do { \
	bak_restore_apb(1); \
	bak_ap_clk_reg(1); \
	bak_restore_ahb(1); \
	} while (0)
#define RESTORE_GLOBAL_REG do { \
	bak_restore_ahb(0); \
	bak_restore_apb(0); \
	bak_ap_clk_reg(0); \
	} while (0)

static void deep_sleep_enable(u32 type)
{
	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,INTC and SYSCON register base vaddr is not ok!\n",
			__func__);
		return;
	}

	if (type) {
		/* ap deep slp enable */
		regmap_update_bits(deep_sys_aon_pwu_apb,
			REG_PMU_APB_AP_SYS_DSLP_ENA,
			BIT_PMU_APB_AP_SYS_DSLP_ENA,
			BIT_PMU_APB_AP_SYS_DSLP_ENA);

		/* ap auto shutdown */
		regmap_update_bits(deep_sys_aon_pwu_apb,
			REG_PMU_APB_PD_AP_SYS_PWR_CFG,
			BIT_PMU_APB_PD_AP_SYS_AUTO_SHUTDOWN_EN,
			BIT_PMU_APB_PD_AP_SYS_AUTO_SHUTDOWN_EN);

		/* clr doze mode ap doze and deep mode not together enable*/
		regmap_update_bits(deep_sys_aon_pwu_apb,
			REG_PMU_APB_DOZE_SLEEP_ENABLE0,
			BIT_PMU_APB_AP_SYS_DOZE_ENA,
			(unsigned int)~BIT_PMU_APB_AP_SYS_DOZE_ENA);
	} else {
		regmap_update_bits(deep_sys_aon_pwu_apb,
			REG_PMU_APB_DOZE_SLEEP_ENABLE0,
			BIT_PMU_APB_AP_SYS_DOZE_ENA,
			BIT_PMU_APB_AP_SYS_DOZE_ENA);
	}
}

static void configure_for_ap_deepsleep(void)
{
	u32 rdata = 0;

	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,INTC and SYSCON register base vaddr is not ok!\n",
			__func__);
		return;
	}

	/* clr bia & dap eb */
	regmap_update_bits(deep_sys_ap_ahb, REG_AP_AHB_AHB_EB,
			BIT_AP_AHB_BIA_LP_EB |
			BIT_AP_AHB_DAP_EB,
	(unsigned int)~(BIT_AP_AHB_DAP_EB |
			BIT_AP_AHB_BIA_LP_EB
			));

	/*vmi clear cpu0 bit5*/
	rdata = readl((void __iomem *)deep_bia_vd_cpu0);
	rdata &= ~(0x1 << 5);
	writel(rdata, (void __iomem *)deep_bia_vd_cpu0);

	/*vmi clear cpu1 bit5*/
	rdata = readl((void __iomem *)deep_bia_vd_cpu1);
	rdata &= ~(0x1 << 5);
	writel(rdata, (void __iomem *)deep_bia_vd_cpu1);
}

static void configure_for_cpu_deepsleep(void)
{
	u32 rdata = 0;

	if ((is_ioremap_cpu0_vaddr_ok || is_ioremap_cpu1_vaddr_ok)) {
		pr_err("%s,failed to find deep_bia_vd\n", __func__);
		return;
	}
	/* Prevent adi sent error voltage(2.2v) for cpu */
	/*as discussed with intel,the voltage should set 0x8D not or 0x8D*/
	rdata = readl((void __iomem *)(deep_bia_vd_cpu0 + VT_BASE_OFF));
	rdata = (rdata & 0xFFFF0000) | 0x8D;
	writel(rdata, (void __iomem *)(deep_bia_vd_cpu0 + VT_BASE_OFF));

	rdata = readl((void __iomem *)(deep_bia_vd_cpu1 + VT_BASE_OFF));
	rdata = (rdata & 0xFFFF0000) | 0x8D;
	writel(rdata, (void __iomem *)(deep_bia_vd_cpu1 + VT_BASE_OFF));

	/*vmi set cpu0 bit5*/
	rdata = readl((void __iomem *)deep_bia_vd_cpu0);
	rdata |= 0x1 << 5;
	writel(rdata, (void __iomem *)deep_bia_vd_cpu0);

	/*vmi set cpu1 bit5*/
	rdata = readl((void __iomem *)deep_bia_vd_cpu1);
	rdata |= 0x1 << 5;
	writel(rdata, (void __iomem *)deep_bia_vd_cpu1);
}

static void bak_restore_light_reg(void)
{
	/* PUB0_FRC_LSLP dap_ep*/
	regmap_update_bits(deep_sys_ap_ahb,
		REG_AP_AHB_PUB0_FRC_LSLP,
		BIT_AP_AHB_PUB0_FRC_LSLP(0x3),
		BIT_AP_AHB_PUB0_FRC_LSLP(0x3));

	/* FRC_DOZE DAP_eb and bia_lp_eb*/
	regmap_update_bits(deep_sys_ap_ahb,
		REG_AP_AHB_FRC_DOZE,
		BIT_AP_AHB_FRC_DOZE(0x3),
		BIT_AP_AHB_FRC_DOZE(0x3));
}

int sprd_s3_deep_sleep(unsigned int cpu)
{
	static unsigned int cnt;

	SAVE_GLOBAL_REG;
	mdelay(5);
	disable_ahb_module();
	/* disable_apb_module(); */
	disable_aon_module();
	deep_sleep_enable(1);
	configure_for_ap_deepsleep();
	pr_info("deep sleep %u times\n", ++cnt);
	pr_info("DEBUG:#################################\n");
	deepsleep_condition_check();
	show_pin_reg();
	bak_last_reg();
	print_last_reg();
	print_pmu_wakeup_sources();
	return 0;
}

int sprd_s3_wake_up(void)
{
	pr_info("enter %s...\n", __func__);

	/* get wakeup_interrupt info*/
	get_hard_irq();
	configure_for_cpu_deepsleep();
	enable_aon_module();
	deep_sleep_enable(0);
	RESTORE_GLOBAL_REG;
	bak_restore_light_reg();
	/* print wakeup_interrupt info*/
	print_hard_irq();
	return 0;
}


static void sprd_deepsleep_init_regmap(void)
{
	pr_info("###### deepsleep_init_regmap start ###\n");
	is_syscon_base_vaddr_ok = 0;

	deepsleep_np = of_find_node_by_name(NULL, "soc-pm");
	if (IS_ERR(deepsleep_np)) {
		pr_err("%s, failed to find deepsleep_np\n", __func__);
		is_syscon_base_vaddr_ok = 1;
	}

	deep_sys_ap_ahb = syscon_regmap_lookup_by_phandle(deepsleep_np,
							"sprd,sys-ap-ahb");
	if (IS_ERR(deep_sys_ap_ahb)) {
		pr_err("%s,failed to find deep_sys_ap_ahb\n", __func__);
		is_syscon_base_vaddr_ok = 1;
	}
	deep_sys_aon_pwu_apb = syscon_regmap_lookup_by_phandle(deepsleep_np,
							"sprd,sys-aon-pwu-apb");
	if (IS_ERR(deep_sys_aon_pwu_apb)) {
		pr_err("%s,failed to find deep_sys_pwu_apb\n",
			__func__);
		is_syscon_base_vaddr_ok = 1;
	}
	deep_sys_aon_apb = syscon_regmap_lookup_by_phandle(deepsleep_np,
							"sprd,sys-aon-apb");
	if (IS_ERR(deep_sys_aon_apb)) {
		pr_err("%s,failed find to deep_sys_aon_apb\n",
			__func__);
		is_syscon_base_vaddr_ok = 1;
	}
	deep_sys_com_pmu_apb = syscon_regmap_lookup_by_phandle(deepsleep_np,
							"sprd,sys-aon-com-pmu-apb");
	if (IS_ERR(deep_sys_com_pmu_apb)) {
		pr_err("%s,failed find to deep_sys_aon_apb\n",
			__func__);
		is_syscon_base_vaddr_ok = 1;
	}
	deep_sys_ap_apb = syscon_regmap_lookup_by_phandle(deepsleep_np,
							"sprd,sys-ap-apb");
	if (IS_ERR(deep_sys_ap_apb)) {
		pr_err("%s,failed to find deep_sys_ap_apb\n", __func__);
		is_syscon_base_vaddr_ok = 1;
	}

	deepsleep_syscon_aon_intc0 =
		syscon_regmap_lookup_by_phandle(deepsleep_np,
						"sprd,sys-aon-intc0");
	if (IS_ERR(deepsleep_syscon_aon_intc0)) {
		pr_err("%s,failed to find deepsleep_syscon_aon_intc0\n",
			__func__);
		is_syscon_base_vaddr_ok = 1;
	}

	deepsleep_syscon_aon_intc1 =
		syscon_regmap_lookup_by_phandle(deepsleep_np,
						"sprd,sys-aon-intc1");
	if (IS_ERR(deepsleep_syscon_aon_intc1)) {
		pr_err("%s,failed to find deepsleep_syscon_aon_intc1\n",
			__func__);
		is_syscon_base_vaddr_ok = 1;
	}

	deepsleep_syscon_aon_intc2 =
		syscon_regmap_lookup_by_phandle(deepsleep_np,
						"sprd,sys-aon-intc2");
	if (IS_ERR(deepsleep_syscon_aon_intc2)) {
		pr_err("%s,failed to find deepsleep_syscon_aon_intc2\n",
			__func__);
		is_syscon_base_vaddr_ok = 1;
	}

	deepsleep_syscon_aon_intc3 =
		syscon_regmap_lookup_by_phandle(deepsleep_np,
						"sprd,sys-aon-intc3");
	if (IS_ERR(deepsleep_syscon_aon_intc3)) {
		pr_err("%s,failed to find deepsleep_syscon_aon_intc3\n",
			__func__);
		is_syscon_base_vaddr_ok = 1;
	}

	pr_info("###### deepsleep_init_regmap end ###\n");
}

static void sprd_deepsleep_init_ioremap(void)
{
	pr_info("###### deepsleep_init_int_ioremap start ###\n");

	deep_bia_vd_cpu0 = ioremap_nocache(ADDR_DVFS_EN_MOD0, 0x1000);
	if (IS_ERR(deep_bia_vd_cpu0)) {
		pr_err("%s,failed to find deep_bia_vd_cpu0\n", __func__);
		is_ioremap_cpu0_vaddr_ok = 1;
	}
	deep_bia_vd_cpu1 = ioremap_nocache(ADDR_DVFS_EN_MOD1, 0x1000);
	if (IS_ERR(deep_bia_vd_cpu1)) {
		pr_err("%s,failed to find deep_bia_vd_cpu1\n", __func__);
		is_ioremap_cpu1_vaddr_ok = 1;
	}
	pr_info("###### deepsleep_init_int_ioremap end ###\n");
}


static int __init sc_pm_init(void)
{
	sprd_deepsleep_init_regmap();
	sprd_deepsleep_init_ioremap();

	if (sprd_deep_get_ap_clk())
		pr_err("%s,failed to find ap clk\n", __func__);

	sprd_soc_deepsleep_enter = (void *)sprd_s3_deep_sleep;
	sprd_soc_wakeup_enter = (void *)sprd_s3_wake_up;


	pm_debug_init();

	return 0;
}

arch_initcall(sc_pm_init);

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
#include <asm/cpu_ops.h>
#include <asm/suspend.h>
#include <asm/barrier.h>
#include <linux/cpu_pm.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include "pm-debug-sharklj1.h"


//#define SHARKLJ1_DEBUG_AP_DEEPSLEEP

extern unsigned long long mread_isr(void);
extern unsigned long long mread_esr(void);
extern unsigned long long mread_msr(void);
extern unsigned long long mread_far(void);
struct device_node *deepsleep_np;
struct regmap *deepsleep_syscon_ap_ahb;
struct regmap *deepsleep_syscon_pmu_apb;
struct regmap *deepsleep_syscon_aon_apb;
struct regmap *deepsleep_syscon_anlg_phy_g1;
struct regmap *deepsleep_syscon_anlg_phy_g2;
struct regmap *deepsleep_syscon_anlg_phy_g3;
struct regmap *deepsleep_syscon_anlg_phy_g4;
struct regmap *deepsleep_syscon_anlg_phy_g5;
struct regmap *deepsleep_syscon_ap_apb;
struct regmap *deepsleep_syscon_aon_intc0;
struct regmap *deepsleep_syscon_aon_intc1;
struct regmap *deepsleep_syscon_aon_intc2;
struct regmap *deepsleep_syscon_aon_intc3;
struct regmap *deepsleep_syscon_aon_intc4;
struct regmap *deepsleep_syscon_aon_intc5;
struct regmap *deepsleep_syscon_ap_syst;

static struct ap_ahb_reg_bak ap_ahb_reg_saved;
static struct ap_apb_reg_bak ap_apb_reg_saved;

int is_intc_base_vaddr_ok;
int is_syscon_base_vaddr_ok;
int is_deep_ap_clk_initialized;

struct deep_sleep_ap_clk_state {
	const char	*name;
	struct	clk	*clk;
	struct	clk	*old_parent;
	struct	clk	*default_parent;
	unsigned long	current_rate;
	unsigned int	is_used;
};
struct	deep_sleep_ap_clk_state	*deep_ap_clk_state;
static int clk_cnt;
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

static bool sprd_deep_get_ap_clk(void)
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
	if (!deep_ap_clk_state) {
		pr_err("%s, Fail to malloc space\n", __func__);
		return 1;
	}
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

			deep_ap_clk_state[clk_idx].is_used = 0;

			of_property_read_string_index(state_node,
				"clock-output-names", 0,
				&(deep_ap_clk_state[clk_idx].name));
			if (deep_ap_clk_state[clk_idx].name == NULL) {
				pr_err("%s,Fail to get deep_ap_clk_state[%d].name\n",
					__func__, clk_idx);
				kfree(deep_ap_clk_state);
				break;
			}

			deep_ap_clk_state[clk_idx].clk = clk_get(NULL,
				deep_ap_clk_state[clk_idx].name);
			if (deep_ap_clk_state[clk_idx].clk == NULL) {
				pr_err("%s,Fail to get deep_ap_clk_state[%d].clk\n",
					__func__, clk_idx);
				kfree(deep_ap_clk_state);
				break;
			}

			deep_ap_clk_state[clk_idx].default_parent =
				get_ap_clk_default_parent_by_idx(i);
			if (deep_ap_clk_state[clk_idx].default_parent == NULL) {
				pr_err("%s,Fail to get default parent of deep_ap_clk_state[%d]\n",
					__func__, clk_idx);
				kfree(deep_ap_clk_state);
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
		pr_debug("%s,bak %s\n", __func__, deep_ap_clk_state[i].name);
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

		pr_debug("%s,resume %s\n", __func__, deep_ap_clk_state[i].name);
	}
#ifdef AP_CLK_DEBUG
		sprd_deep_store_ap_clk();
#endif

	pr_info("end %s\n", __func__);
}

static void sprd_deepsleep_init_intc(void)
{
	uint32_t wakeup_irq_en = 0;

	if (is_intc_base_vaddr_ok || is_syscon_base_vaddr_ok) {
		pr_info("is_intc_base_vaddr_ok=%d,is_syscon_base_vaddr_ok=%d\n",is_intc_base_vaddr_ok,is_syscon_base_vaddr_ok);
		pr_err("%s,INTC and SYSCON register base vaddr is not ok!\n",
			__func__);
		return;
	}

	/*INTC0_EB, INTC1_EB, INTC2_EB, INTC3_EB, INTC4_EB, INTC5_EB*/
	regmap_update_bits(deepsleep_syscon_aon_apb, REG_AON_APB_APB_EB2,
		BIT_AON_APB_AP_INTC0_EB|
		BIT_AON_APB_AP_INTC1_EB|
		BIT_AON_APB_AP_INTC2_EB|
		BIT_AON_APB_AP_INTC3_EB|
		BIT_AON_APB_AP_INTC4_EB|
		BIT_AON_APB_AP_INTC5_EB,
		BIT_AON_APB_AP_INTC0_EB|
		BIT_AON_APB_AP_INTC1_EB|
		BIT_AON_APB_AP_INTC2_EB|
		BIT_AON_APB_AP_INTC3_EB|
		BIT_AON_APB_AP_INTC4_EB|
		BIT_AON_APB_AP_INTC5_EB);


	/*set deepsleep wakeup sources*/
	regmap_update_bits(deepsleep_syscon_pmu_apb,
				REG_PMU_APB_CA53_WAKEUP_IRQ_EN1,
				BIT(3)|BIT(5)|BIT(6)|BIT(29),
				BIT(3)|BIT(5)|BIT(6)|BIT(29));//gpio,eic,ana,dmc_mpu_vio

	regmap_update_bits(deepsleep_syscon_pmu_apb,
				REG_PMU_APB_CA53_WAKEUP_IRQ_EN2,
				BIT(4)|BIT(5),
				BIT(4)|BIT(5));//mbox_src_ap,mbox_tar_ap

	regmap_read(deepsleep_syscon_pmu_apb,
			REG_PMU_APB_CA53_WAKEUP_IRQ_EN0,
			&wakeup_irq_en);
	regmap_write(deepsleep_syscon_aon_intc0,
			INTCV0_IRQ_EN,
			wakeup_irq_en);

	regmap_read(deepsleep_syscon_pmu_apb,
			REG_PMU_APB_CA53_WAKEUP_IRQ_EN1,
			&wakeup_irq_en);
	regmap_write(deepsleep_syscon_aon_intc1,
			INTCV1_IRQ_EN,
			wakeup_irq_en);

	regmap_read(deepsleep_syscon_pmu_apb,
			REG_PMU_APB_CA53_WAKEUP_IRQ_EN2,
			&wakeup_irq_en);
	regmap_write(deepsleep_syscon_aon_intc2,
			INTCV2_IRQ_EN,
			wakeup_irq_en);

	regmap_read(deepsleep_syscon_pmu_apb,
			REG_PMU_APB_CA53_WAKEUP_IRQ_EN3,
			&wakeup_irq_en);
	regmap_write(deepsleep_syscon_aon_intc3,
			INTCV3_IRQ_EN,
			wakeup_irq_en);

	regmap_read(deepsleep_syscon_pmu_apb,
			REG_PMU_APB_CA53_WAKEUP_IRQ_EN4,
			&wakeup_irq_en);
	regmap_write(deepsleep_syscon_aon_intc4,
			INTCV4_IRQ_EN,
			wakeup_irq_en);

	regmap_read(deepsleep_syscon_pmu_apb,
			REG_PMU_APB_CA53_WAKEUP_IRQ_EN5,
			&wakeup_irq_en);
	regmap_write(deepsleep_syscon_aon_intc5,
			INTCV5_IRQ_EN,
			wakeup_irq_en);
}
void disable_ahb_module(void)
{
	if (IS_ERR(deepsleep_syscon_ap_ahb)) {
		pr_err("%s,failed to find deepsleep_syscon_ap_ahb\n", __func__);
		return;
	}

	/*AP_PERI_FORCE_ON CLR*/
	regmap_update_bits(deepsleep_syscon_ap_ahb,
		REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG,
		BIT_AP_AHB_AP_PERI_FORCE_ON,
		(unsigned int)(~BIT_AP_AHB_AP_PERI_FORCE_ON));
	/* AP_PERI_FORCE_SLP*/
	regmap_update_bits(deepsleep_syscon_ap_ahb,
		REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG,
		BIT_AP_AHB_AP_PERI_FORCE_SLP,
		BIT_AP_AHB_AP_PERI_FORCE_SLP);

}
void bak_restore_ahb(int bak)
{
	if (IS_ERR(deepsleep_syscon_ap_ahb)) {
		pr_err("%s,failed to find deepsleep_syscon_ap_ahb\n", __func__);
		return;
	}

	if (bak) {
		regmap_read(deepsleep_syscon_ap_ahb, REG_AP_AHB_AHB_EB,
			&(ap_ahb_reg_saved.ahb_eb));

	} else {
		regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_AHB_EB,
			ap_ahb_reg_saved.ahb_eb);

	}
}

void disable_apb_module(void)
{
	int stat;

	if (IS_ERR(deepsleep_syscon_ap_apb)) {
		pr_err("%s,failed to find deepsleep_syscon_ap_apb\n", __func__);
		return;
	}

	regmap_read(deepsleep_syscon_ap_apb, REG_AP_APB_APB_EB, &stat);
	stat &= (BIT_AP_APB_UART1_EB | BIT_AP_APB_UART0_EB);
	regmap_write(deepsleep_syscon_ap_apb, REG_AP_APB_APB_EB, stat);
}
void bak_restore_apb(int bak)
{
	if (IS_ERR(deepsleep_syscon_ap_apb)) {
		pr_err("%s,failed to find deepsleep_syscon_ap_apb\n", __func__);
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
struct clk *clk;

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
void disable_aon_module(void)
{
	if (IS_ERR(deepsleep_syscon_aon_apb)) {
		pr_err("%s,failed to find deepsleep_syscon_aon_apb\n", __func__);
		return;
	}

	regmap_update_bits(deepsleep_syscon_aon_apb, REG_AON_APB_APB_EB0,
		BIT_AON_APB_AON_TMR_EB | BIT_AON_APB_AP_TMR0_EB,
		(unsigned int)(~(BIT_AON_APB_AON_TMR_EB |
		BIT_AON_APB_AP_TMR0_EB)));
	regmap_update_bits(deepsleep_syscon_aon_apb, REG_AON_APB_APB_EB1,
		BIT_AON_APB_AP_TMR2_EB | BIT_AON_APB_AP_TMR1_EB,
		(unsigned int)(~(BIT_AON_APB_AP_TMR2_EB |
		BIT_AON_APB_AP_TMR1_EB)));
	regmap_update_bits(deepsleep_syscon_aon_apb, REG_AON_APB_APB_EB1,
		BIT_AON_APB_DISP_EMC_EB,
		(unsigned int)(~BIT_AON_APB_DISP_EMC_EB));

}

static void enable_aon_module(void)
{
	if (IS_ERR(deepsleep_syscon_aon_apb)) {
		pr_err("%s,failed to find deepsleep_syscon_aon_apb\n", __func__);
		return;
	}

	regmap_update_bits(deepsleep_syscon_aon_apb,
		REG_AON_APB_APB_EB0,
		BIT_AON_APB_AON_TMR_EB |
		BIT_AON_APB_AP_TMR0_EB,
		BIT_AON_APB_AON_TMR_EB |
		BIT_AON_APB_AP_TMR0_EB);

	regmap_update_bits(deepsleep_syscon_aon_apb,
		REG_AON_APB_APB_EB1,
		BIT_AON_APB_AP_TMR2_EB |
		BIT_AON_APB_AP_TMR1_EB,
		BIT_AON_APB_AP_TMR2_EB |
		BIT_AON_APB_AP_TMR1_EB);

	regmap_update_bits(deepsleep_syscon_aon_apb,
		REG_AON_APB_APB_EB1,
		BIT_AON_APB_DISP_EMC_EB,
		BIT_AON_APB_DISP_EMC_EB);
}


void disable_ana_module(void)
{
}
void bak_restore_ana(int bak)
{
}

unsigned int sprd_irq_pending(void)
{
	uint32_t  intc0_bits = 0, intc1_bits = 0, intc2_bits = 0,
		intc3_bits = 0, intc4_bits = 0, intc5_bits = 0;

	if (is_intc_base_vaddr_ok) {
		pr_err("%s,INTC register base vaddr is not ok!\n", __func__);
		return 0;
	}

	print_int_status();

	regmap_read(deepsleep_syscon_aon_intc0, 0, &intc0_bits);
	regmap_read(deepsleep_syscon_aon_intc1, 0, &intc1_bits);
	regmap_read(deepsleep_syscon_aon_intc2, 0, &intc2_bits);
	regmap_read(deepsleep_syscon_aon_intc3, 0, &intc3_bits);
	regmap_read(deepsleep_syscon_aon_intc4, 0, &intc4_bits);
	regmap_read(deepsleep_syscon_aon_intc5, 0, &intc5_bits);

	if (intc0_bits | intc1_bits | intc2_bits | intc3_bits |
		intc4_bits | intc5_bits)
		return 1;
	else
		return 0;
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


#define SAVE_GLOBAL_REG do { \
	bak_restore_apb(1); \
	bak_ap_clk_reg(1); \
	bak_restore_ahb(1); \
	} while (0)
#define RESTORE_GLOBAL_REG do { \
	bak_restore_apb(0); \
	bak_ap_clk_reg(0); \
	bak_restore_ahb(0); \
	} while (0)

#ifdef AP_DEEPSLEEP_DEBUG
void pmu_force_cp(void)
{
	if (IS_ERR(deepsleep_syscon_pmu_apb)) {
		pr_err("%s,failed to find deepsleep_syscon_pmu_apb\n",
			__func__);
		return;
	}
	/*FORCE DEEP SLEEP */
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_SLEEP_CTRL,
		BIT_PMU_APB_AGCP_FORCE_DEEP_SLEEP |
		BIT_PMU_APB_WTLCP_FORCE_DEEP_SLEEP |
		BIT_PMU_APB_PUBCP_FORCE_DEEP_SLEEP,
		BIT_PMU_APB_AGCP_FORCE_DEEP_SLEEP |
		BIT_PMU_APB_WTLCP_FORCE_DEEP_SLEEP |
		BIT_PMU_APB_PUBCP_FORCE_DEEP_SLEEP);
	/*PUBCP FORCE */
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_PUBCP_SYS_CFG,
		BIT_PMU_APB_PD_PUBCP_SYS_AUTO_SHUTDOWN_EN,
		(unsigned int)(~BIT_PMU_APB_PD_PUBCP_SYS_AUTO_SHUTDOWN_EN));
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_PUBCP_SYS_CFG,
		BIT_PMU_APB_PD_PUBCP_SYS_FORCE_SHUTDOWN,
		BIT_PMU_APB_PD_PUBCP_SYS_FORCE_SHUTDOWN);
	/*WTLCP FORCE*/
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_WTLCP_SYS_CFG,
		BIT_PMU_APB_PD_WTLCP_SYS_AUTO_SHUTDOWN_EN,
		(unsigned int)(~BIT_PMU_APB_PD_WTLCP_SYS_AUTO_SHUTDOWN_EN));
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_WTLCP_SYS_CFG,
		BIT_PMU_APB_PD_WTLCP_SYS_FORCE_SHUTDOWN,
		BIT_PMU_APB_PD_WTLCP_SYS_FORCE_SHUTDOWN);
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_WTLCP_TGDSP_CFG,
		BIT_PMU_APB_PD_WTLCP_TGDSP_AUTO_SHUTDOWN_EN,
		(unsigned int)(~BIT_PMU_APB_PD_WTLCP_TGDSP_AUTO_SHUTDOWN_EN));
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_WTLCP_TGDSP_CFG,
		BIT_PMU_APB_PD_WTLCP_TGDSP_FORCE_SHUTDOWN,
		BIT_PMU_APB_PD_WTLCP_TGDSP_FORCE_SHUTDOWN);
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_WTLCP_LDSP_CFG,
		BIT_PMU_APB_PD_WTLCP_LDSP_AUTO_SHUTDOWN_EN,
		(unsigned int)(~BIT_PMU_APB_PD_WTLCP_LDSP_AUTO_SHUTDOWN_EN));
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_WTLCP_LDSP_CFG,
		BIT_PMU_APB_PD_WTLCP_LDSP_FORCE_SHUTDOWN,
		BIT_PMU_APB_PD_WTLCP_LDSP_FORCE_SHUTDOWN);
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_WTLCP_HU3GE_A_CFG,
		BIT_PMU_APB_PD_WTLCP_HU3GE_A_AUTO_SHUTDOWN_EN,
		(unsigned int)(~BIT_PMU_APB_PD_WTLCP_HU3GE_A_AUTO_SHUTDOWN_EN));
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_WTLCP_HU3GE_A_CFG,
		BIT_PMU_APB_PD_WTLCP_HU3GE_A_FORCE_SHUTDOWN,
		BIT_PMU_APB_PD_WTLCP_HU3GE_A_FORCE_SHUTDOWN);
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_WTLCP_HU3GE_B_CFG,
		BIT_PMU_APB_PD_WTLCP_HU3GE_B_AUTO_SHUTDOWN_EN,
		(unsigned int)(~BIT_PMU_APB_PD_WTLCP_HU3GE_B_AUTO_SHUTDOWN_EN));
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_WTLCP_HU3GE_B_CFG,
		BIT_PMU_APB_PD_WTLCP_HU3GE_B_FORCE_SHUTDOWN,
		BIT_PMU_APB_PD_WTLCP_HU3GE_B_FORCE_SHUTDOWN);
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_WTLCP_GSM_CFG,
		BIT_PMU_APB_PD_WTLCP_GSM_AUTO_SHUTDOWN_EN,
		(unsigned int)(~BIT_PMU_APB_PD_WTLCP_GSM_AUTO_SHUTDOWN_EN));
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_WTLCP_GSM_CFG,
		BIT_PMU_APB_PD_WTLCP_GSM_FORCE_SHUTDOWN,
		BIT_PMU_APB_PD_WTLCP_GSM_FORCE_SHUTDOWN);
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_WTLCP_TD_CFG,
		BIT_PMU_APB_PD_WTLCP_TD_AUTO_SHUTDOWN_EN,
		(unsigned int)(~BIT_PMU_APB_PD_WTLCP_TD_AUTO_SHUTDOWN_EN));
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_WTLCP_TD_CFG,
		BIT_PMU_APB_PD_WTLCP_TD_FORCE_SHUTDOWN,
		BIT_PMU_APB_PD_WTLCP_TD_FORCE_SHUTDOWN);
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_WTLCP_LTE_P1_CFG,
		BIT_PMU_APB_PD_WTLCP_LTE_P1_AUTO_SHUTDOWN_EN,
		(unsigned int)(~BIT_PMU_APB_PD_WTLCP_LTE_P1_AUTO_SHUTDOWN_EN));
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_WTLCP_LTE_P1_CFG,
		BIT_PMU_APB_PD_WTLCP_LTE_P1_FORCE_SHUTDOWN,
		BIT_PMU_APB_PD_WTLCP_LTE_P1_FORCE_SHUTDOWN);
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_WTLCP_LTE_P2_CFG,
		BIT_PMU_APB_PD_WTLCP_LTE_P2_AUTO_SHUTDOWN_EN,
		(unsigned int)(~BIT_PMU_APB_PD_WTLCP_LTE_P2_AUTO_SHUTDOWN_EN));
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_WTLCP_LTE_P2_CFG,
		BIT_PMU_APB_PD_WTLCP_LTE_P2_FORCE_SHUTDOWN,
		BIT_PMU_APB_PD_WTLCP_LTE_P2_FORCE_SHUTDOWN);
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_WTLCP_LTE_P3_CFG,
		BIT_PMU_APB_PD_WTLCP_LTE_P3_AUTO_SHUTDOWN_EN,
		(unsigned int)(~BIT_PMU_APB_PD_WTLCP_LTE_P3_AUTO_SHUTDOWN_EN));
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_WTLCP_LTE_P3_CFG,
		BIT_PMU_APB_PD_WTLCP_LTE_P3_FORCE_SHUTDOWN,
		BIT_PMU_APB_PD_WTLCP_LTE_P3_FORCE_SHUTDOWN);
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_WTLCP_LTE_P4_CFG,
		BIT_PMU_APB_PD_WTLCP_LTE_P4_AUTO_SHUTDOWN_EN,
		(unsigned int)(~BIT_PMU_APB_PD_WTLCP_LTE_P4_AUTO_SHUTDOWN_EN));
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_WTLCP_LTE_P4_CFG,
		BIT_PMU_APB_PD_WTLCP_LTE_P4_FORCE_SHUTDOWN,
		BIT_PMU_APB_PD_WTLCP_LTE_P4_FORCE_SHUTDOWN);
	/*AGCP FORCE*/
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_AGCP_SYS_CFG,
		BIT_PMU_APB_PD_AGCP_SYS_AUTO_SHUTDOWN_EN,
		(unsigned int)(~BIT_PMU_APB_PD_AGCP_SYS_AUTO_SHUTDOWN_EN));
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_AGCP_SYS_CFG,
		BIT_PMU_APB_PD_AGCP_SYS_FORCE_SHUTDOWN,
		BIT_PMU_APB_PD_AGCP_SYS_FORCE_SHUTDOWN);
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_AGCP_DSP_CFG,
		BIT_PMU_APB_PD_AGCP_DSP_AUTO_SHUTDOWN_EN,
		(unsigned int)(~BIT_PMU_APB_PD_AGCP_DSP_AUTO_SHUTDOWN_EN));
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_AGCP_DSP_CFG,
		BIT_PMU_APB_PD_AGCP_DSP_FORCE_SHUTDOWN,
		BIT_PMU_APB_PD_AGCP_DSP_FORCE_SHUTDOWN);
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_AGCP_GSM_CFG,
		BIT_PMU_APB_PD_AGCP_GSM_AUTO_SHUTDOWN_EN,
		(unsigned int)(~BIT_PMU_APB_PD_AGCP_GSM_AUTO_SHUTDOWN_EN));
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_PD_AGCP_GSM_CFG,
		BIT_PMU_APB_PD_AGCP_GSM_FORCE_SHUTDOWN,
		BIT_PMU_APB_PD_AGCP_GSM_FORCE_SHUTDOWN);
}
#endif

void print_reg_value(struct regmap *regmap_base, char *reg_name, unsigned int reg_addr_offset)
{
	unsigned int chk_reg;
	regmap_read(regmap_base, reg_addr_offset, &chk_reg);
	pr_info("#--%s[0x%08x]:0x%08x\n",reg_name,reg_addr_offset,chk_reg);
}

void check_registers(void)
{
	if (IS_ERR(deepsleep_syscon_ap_ahb)) {
		pr_err("%s,failed to find deepsleep_syscon_ap_ahb\n", __func__);
		return;
	}
	if (IS_ERR(deepsleep_syscon_ap_apb)) {
		pr_err("%s,failed to find deepsleep_syscon_ap_apb\n", __func__);
		return;
	}
	if (IS_ERR(deepsleep_syscon_aon_apb)) {
		pr_err("%s,failed to find deepsleep_syscon_aon_apb\n", __func__);
		return;
	}
	if (IS_ERR(deepsleep_syscon_pmu_apb)) {
		pr_err("%s,failed to find deepsleep_syscon_pmu_apb\n", __func__);
		return;
	}
	pr_info("*********REGISTERS CHECK START:*******\n");
	pr_info("*********REGISTERS CHECK START:*******\n");
	pr_info("*********REGISTERS CHECK START:*******\n");

	pr_info("----------------------------AP AHB REGS------------------------------\n");
	print_reg_value(deepsleep_syscon_ap_ahb,"REG_AP_AHB_AHB_EB",REG_AP_AHB_AHB_EB);
	print_reg_value(deepsleep_syscon_ap_ahb,"REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG",REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG);
	print_reg_value(deepsleep_syscon_ap_ahb,"REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG",REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG);
	print_reg_value(deepsleep_syscon_ap_ahb,"REG_AP_AHB_MAIN_LPC",REG_AP_AHB_MAIN_LPC);
	print_reg_value(deepsleep_syscon_ap_ahb,"REG_AP_AHB_M1_LPC",REG_AP_AHB_M1_LPC);
	print_reg_value(deepsleep_syscon_ap_ahb,"REG_AP_AHB_M2_LPC",REG_AP_AHB_M2_LPC);
	print_reg_value(deepsleep_syscon_ap_ahb,"REG_AP_AHB_M3_LPC",REG_AP_AHB_M3_LPC);
	print_reg_value(deepsleep_syscon_ap_ahb,"REG_AP_AHB_M4_LPC",REG_AP_AHB_M4_LPC);
	print_reg_value(deepsleep_syscon_ap_ahb,"REG_AP_AHB_M5_LPC",REG_AP_AHB_M5_LPC);
	print_reg_value(deepsleep_syscon_ap_ahb,"REG_AP_AHB_M6_LPC",REG_AP_AHB_M6_LPC);
	print_reg_value(deepsleep_syscon_ap_ahb,"REG_AP_AHB_M7_LPC",REG_AP_AHB_M7_LPC);
	print_reg_value(deepsleep_syscon_ap_ahb,"REG_AP_AHB_M8_LPC",REG_AP_AHB_M8_LPC);
	print_reg_value(deepsleep_syscon_ap_ahb,"REG_AP_AHB_M9_LPC",REG_AP_AHB_M9_LPC);
	print_reg_value(deepsleep_syscon_ap_ahb,"REG_AP_AHB_M10_LPC",REG_AP_AHB_M10_LPC);
	print_reg_value(deepsleep_syscon_ap_ahb,"REG_AP_AHB_S0_LPC",REG_AP_AHB_S0_LPC);
	print_reg_value(deepsleep_syscon_ap_ahb,"REG_AP_AHB_S1_LPC",REG_AP_AHB_S1_LPC);
	print_reg_value(deepsleep_syscon_ap_ahb,"REG_AP_AHB_S2_LPC",REG_AP_AHB_S2_LPC);
	print_reg_value(deepsleep_syscon_ap_ahb,"REG_AP_AHB_S3_LPC",REG_AP_AHB_S3_LPC);
	print_reg_value(deepsleep_syscon_ap_ahb,"REG_AP_AHB_S4_LPC",REG_AP_AHB_S4_LPC);
	print_reg_value(deepsleep_syscon_ap_ahb,"REG_AP_AHB_S5_LPC",REG_AP_AHB_S5_LPC);

	pr_info("----------------------------AP APB REGS------------------------------\n");
	print_reg_value(deepsleep_syscon_ap_apb,"REG_AP_APB_APB_EB",REG_AP_APB_APB_EB);

	pr_info("----------------------------AON APB REGS------------------------------\n");
	print_reg_value(deepsleep_syscon_aon_apb,"REG_AON_APB_APB_EB0",REG_AON_APB_APB_EB0);
	print_reg_value(deepsleep_syscon_aon_apb,"REG_AON_APB_APB_EB1",REG_AON_APB_APB_EB1);
	print_reg_value(deepsleep_syscon_aon_apb,"REG_AON_APB_APB_EB2",REG_AON_APB_APB_EB2);

	pr_info("----------------------------PMU APB REGS------------------------------\n");
	print_reg_value(deepsleep_syscon_pmu_apb,"REG_PMU_APB_AP_DSLP_ENA",REG_PMU_APB_AP_DSLP_ENA);
	print_reg_value(deepsleep_syscon_pmu_apb,"REG_PMU_APB_CA53_WAKEUP_IRQ_EN0",REG_PMU_APB_CA53_WAKEUP_IRQ_EN0);
	print_reg_value(deepsleep_syscon_pmu_apb,"REG_PMU_APB_CA53_WAKEUP_IRQ_EN1",REG_PMU_APB_CA53_WAKEUP_IRQ_EN1);
	print_reg_value(deepsleep_syscon_pmu_apb,"REG_PMU_APB_CA53_WAKEUP_IRQ_EN2",REG_PMU_APB_CA53_WAKEUP_IRQ_EN2);
	print_reg_value(deepsleep_syscon_pmu_apb,"REG_PMU_APB_CA53_WAKEUP_IRQ_EN3",REG_PMU_APB_CA53_WAKEUP_IRQ_EN3);
	print_reg_value(deepsleep_syscon_pmu_apb,"REG_PMU_APB_CA53_WAKEUP_IRQ_EN4",REG_PMU_APB_CA53_WAKEUP_IRQ_EN4);
	print_reg_value(deepsleep_syscon_pmu_apb,"REG_PMU_APB_CA53_WAKEUP_IRQ_EN5",REG_PMU_APB_CA53_WAKEUP_IRQ_EN5);
	print_reg_value(deepsleep_syscon_pmu_apb,"REG_PMU_APB_PWR_STATUS0_DBG",REG_PMU_APB_PWR_STATUS0_DBG);
	print_reg_value(deepsleep_syscon_pmu_apb,"REG_PMU_APB_PWR_STATUS1_DBG",REG_PMU_APB_PWR_STATUS1_DBG);
	print_reg_value(deepsleep_syscon_pmu_apb,"REG_PMU_APB_PWR_STATUS2_DBG",REG_PMU_APB_PWR_STATUS2_DBG);
	print_reg_value(deepsleep_syscon_pmu_apb,"REG_PMU_APB_PWR_STATUS3_DBG",REG_PMU_APB_PWR_STATUS3_DBG);
	print_reg_value(deepsleep_syscon_pmu_apb,"REG_PMU_APB_PAD_OUT_ADIE_CTRL0",REG_PMU_APB_PAD_OUT_ADIE_CTRL0);
	print_reg_value(deepsleep_syscon_pmu_apb,"REG_PMU_APB_PAD_OUT_ADIE_CTRL1",REG_PMU_APB_PAD_OUT_ADIE_CTRL1);

	pr_info("*********REGISTERS CHECK END:*******\n");
	pr_info("*********REGISTERS CHECK END:*******\n");
	pr_info("*********REGISTERS CHECK END:*******\n");

}

void set_ap_deepsleep_enable(void)
{
	regmap_update_bits(deepsleep_syscon_pmu_apb,
		REG_PMU_APB_AP_DSLP_ENA,
		BIT_PMU_APB_AP_DSLP_ENA,
		BIT_PMU_APB_AP_DSLP_ENA);
}
#ifdef SHARKLJ1_DEBUG_AP_DEEPSLEEP
void sharklj1_debug_ap_deepsleep(void)
{
	regmap_update_bits(deepsleep_syscon_ap_ahb,
		REG_AP_AHB_AHB_EB,
		BIT_AP_AHB_GSP_EB,
		(unsigned int)(~BIT_AP_AHB_GSP_EB));

	regmap_update_bits(deepsleep_syscon_ap_ahb,
		REG_AP_AHB_AHB_EB,
		BIT_AP_AHB_DISPC_EB,
		(unsigned int)(~BIT_AP_AHB_DISPC_EB));

	regmap_update_bits(deepsleep_syscon_aon_apb,
		REG_AON_APB_APB_EB0,
		BIT_AON_APB_GPU_EB,
		(unsigned int)(~BIT_AON_APB_GPU_EB));

	/*debug chip deep sleep*/
	regmap_update_bits(deepsleep_syscon_aon_apb,
		REG_PMU_APB_PAD_OUT_ADIE_CTRL0,
		BIT_PMU_APB_PAD_OUT_CHIP_SLEEP_ARM7_DEEP_SLEEP_MASK,
		BIT_PMU_APB_PAD_OUT_CHIP_SLEEP_ARM7_DEEP_SLEEP_MASK);
}
#endif
void set_deepsleep_lpc(void)
{

	if (IS_ERR(deepsleep_syscon_ap_ahb)) {
		pr_err("%s,failed to find deepsleep_syscon_ap_ahb\n", __func__);
		return;
	}

	regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_MAIN_LPC, 0x5ffff);
	regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_M1_LPC, 0x5ffff);
	regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_M2_LPC, 0x5ffff);
	regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_M3_LPC, 0x5ffff);
	regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_M4_LPC, 0x5ffff);
	regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_M5_LPC, 0x5ffff);
	regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_M6_LPC, 0x5ffff);
	regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_M7_LPC, 0x5ffff);
	regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_M8_LPC, 0x5ffff);
	regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_M9_LPC, 0x5ffff);
	regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_M10_LPC, 0x5ffff);
	regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_S0_LPC, 0x5ffff);
	regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_S1_LPC, 0x5ffff);
	regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_S2_LPC, 0x5ffff);
	regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_S3_LPC, 0x5ffff);
	regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_S4_LPC, 0x5ffff);
	regmap_write(deepsleep_syscon_ap_ahb, REG_AP_AHB_S5_LPC, 0x5ffff);
}

int sprd_cpu_deep_sleep(unsigned int cpu)
{
	int ret = 0;
	unsigned long flags = 0;
	static unsigned int cnt;

	/*time = get_sys_cnt();*/
	if (!hw_irqs_disabled())  {
		pr_info("##: Error(%s): IRQ is enabled(%08lx)!\n",
			 "wakelock_suspend", flags);
	}
	SAVE_GLOBAL_REG;
	disable_ahb_module();
	disable_apb_module();
	disable_aon_module();
	sprd_deepsleep_init_intc();
#ifdef SHARKLJ1_DEBUG_AP_DEEPSLEEP
	//pmu_force_cp();
	sharklj1_debug_ap_deepsleep();
#endif
	set_deepsleep_lpc();
	set_ap_deepsleep_enable();
	check_registers();

	ret = cpu_ops[0]->cpu_suspend(3);
	ret = (ret ? 0 : 1);
	udelay(50);
	if (ret)
		pr_info("deep sleep %u times\n", ++cnt);

	enable_aon_module();
	RESTORE_GLOBAL_REG;
	udelay(5);

#if 0
	print_hard_irq_inloop(ret);
#endif
	return ret;
}

void sc_default_idle(void)
{
	cpu_do_idle();
	local_irq_enable();
}

static void sprd_deepsleep_init_regmap(void)
{
	pr_info("###### deepsleep_init_regmap start ###\n");

	is_intc_base_vaddr_ok = 0;
	is_syscon_base_vaddr_ok = 0;

	deepsleep_np = of_find_node_by_name(NULL, "deep-sleep");
	if (IS_ERR(deepsleep_np)) {
		pr_err("%s, failed to find deepsleep_np\n", __func__);
		is_syscon_base_vaddr_ok = 1;
	}

	deepsleep_syscon_ap_ahb = syscon_regmap_lookup_by_phandle(deepsleep_np,
							"sprd,sys-ap-ahb");
	if (IS_ERR(deepsleep_syscon_ap_ahb)) {
		pr_err("%s,failed to find deepsleep_syscon_ap_ahb\n", __func__);
		is_syscon_base_vaddr_ok = 1;
	}
	deepsleep_syscon_pmu_apb = syscon_regmap_lookup_by_phandle(deepsleep_np,
							"sprd,sys-pmu-apb");
	if (IS_ERR(deepsleep_syscon_pmu_apb)) {
		pr_err("%s,failed to find deepsleep_syscon_pmu_apb\n",
			__func__);
		is_syscon_base_vaddr_ok = 1;
	}
	deepsleep_syscon_aon_apb = syscon_regmap_lookup_by_phandle(deepsleep_np,
							"sprd,sys-aon-apb");
	if (IS_ERR(deepsleep_syscon_aon_apb)) {
		pr_err("%s,failed find to deepsleep_syscon_aon_apb\n",
			__func__);
		is_syscon_base_vaddr_ok = 1;
	}

	deepsleep_syscon_anlg_phy_g1 = syscon_regmap_lookup_by_phandle(deepsleep_np,
							"sprd,sys-anlg_phy_g1");
	if (IS_ERR(deepsleep_syscon_anlg_phy_g1)) {
		pr_err("%s,failed find to deepsleep_syscon_anlg_phy_g1\n",
			__func__);
		is_syscon_base_vaddr_ok = 1;
	}
	deepsleep_syscon_anlg_phy_g2 = syscon_regmap_lookup_by_phandle(deepsleep_np,
							"sprd,sys-anlg_phy_g2");
	if (IS_ERR(deepsleep_syscon_anlg_phy_g2)) {
		pr_err("%s,failed find to deepsleep_syscon_anlg_phy_g2\n",
			__func__);
		is_syscon_base_vaddr_ok = 1;
	}

	deepsleep_syscon_anlg_phy_g3 = syscon_regmap_lookup_by_phandle(deepsleep_np,
							"sprd,sys-anlg_phy_g3");
	if (IS_ERR(deepsleep_syscon_anlg_phy_g3)) {
		pr_err("%s,failed find to deepsleep_syscon_anlg_phy_g3\n",
			__func__);
		is_syscon_base_vaddr_ok = 1;
	}

	deepsleep_syscon_anlg_phy_g4 = syscon_regmap_lookup_by_phandle(deepsleep_np,
							"sprd,sys-anlg_phy_g4");
	if (IS_ERR(deepsleep_syscon_anlg_phy_g4)) {
		pr_err("%s,failed find to deepsleep_syscon_anlg_phy_g4\n",
			__func__);
		is_syscon_base_vaddr_ok = 1;
	}

	deepsleep_syscon_anlg_phy_g5 = syscon_regmap_lookup_by_phandle(deepsleep_np,
							"sprd,sys-anlg_phy_g5");
	if (IS_ERR(deepsleep_syscon_anlg_phy_g5)) {
		pr_err("%s,failed find to deepsleep_syscon_anlg_phy_g5\n",
			__func__);
		is_syscon_base_vaddr_ok = 1;
	}

	deepsleep_syscon_ap_apb = syscon_regmap_lookup_by_phandle(deepsleep_np,
							"sprd,sys-ap-apb");
	if (IS_ERR(deepsleep_syscon_ap_apb)) {
		pr_err("%s,failed to find deepsleep_syscon_ap_apb\n", __func__);
		is_syscon_base_vaddr_ok = 1;
	}

	deepsleep_syscon_aon_intc0 =
		syscon_regmap_lookup_by_phandle(deepsleep_np,
						"sprd,sys-aon-intc0");
	if (IS_ERR(deepsleep_syscon_aon_intc0)) {
		pr_err("%s,failed to find deepsleep_syscon_aon_intc0\n",
			__func__);
		is_intc_base_vaddr_ok = 1;
	}

	deepsleep_syscon_aon_intc1 =
		syscon_regmap_lookup_by_phandle(deepsleep_np,
						"sprd,sys-aon-intc1");
	if (IS_ERR(deepsleep_syscon_aon_intc1)) {
		pr_err("%s,failed to find deepsleep_syscon_aon_intc1\n",
			__func__);
		is_intc_base_vaddr_ok = 1;
	}

	deepsleep_syscon_aon_intc2 =
		syscon_regmap_lookup_by_phandle(deepsleep_np,
						"sprd,sys-aon-intc2");
	if (IS_ERR(deepsleep_syscon_aon_intc2)) {
		pr_err("%s,failed to find deepsleep_syscon_aon_intc2\n",
			__func__);
		is_intc_base_vaddr_ok = 1;
	}

	deepsleep_syscon_aon_intc3 =
		syscon_regmap_lookup_by_phandle(deepsleep_np,
						"sprd,sys-aon-intc3");
	if (IS_ERR(deepsleep_syscon_aon_intc3)) {
		pr_err("%s,failed to find deepsleep_syscon_aon_intc3\n",
			__func__);
		is_intc_base_vaddr_ok = 1;
	}

	deepsleep_syscon_aon_intc4 =
		syscon_regmap_lookup_by_phandle(deepsleep_np,
						"sprd,sys-aon-intc4");
	if (IS_ERR(deepsleep_syscon_aon_intc4)) {
		pr_err("%s,failed to find deepsleep_syscon_aon_intc4\n",
			__func__);
		is_intc_base_vaddr_ok = 1;
	}

	deepsleep_syscon_aon_intc5 =
		syscon_regmap_lookup_by_phandle(deepsleep_np,
						"sprd,sys-aon-intc5");
	if (IS_ERR(deepsleep_syscon_aon_intc5)) {
		pr_err("%s,failed to find deepsleep_syscon_aon_intc5\n",
			__func__);
		is_intc_base_vaddr_ok = 1;
	}

	deepsleep_np = of_find_node_by_name(NULL, "deep-sleep");
	if (IS_ERR(deepsleep_np)) {
		pr_err("%s, failed to find deepsleep_np\n", __func__);
		is_syscon_base_vaddr_ok = 1;
	}

	deepsleep_syscon_ap_ahb = syscon_regmap_lookup_by_phandle(deepsleep_np,
							"sprd,sys-ap-ahb");
	if (IS_ERR(deepsleep_syscon_ap_ahb)) {
		pr_err("%s,failed to find deepsleep_syscon_ap_ahb\n", __func__);
		is_syscon_base_vaddr_ok = 1;
	}


	pr_info("###### deepsleep_init_regmap end ###\n");
}

void __init sc_pm_init(void)
{
	pr_info("**************%s called*****************\n", __func__);
	sprd_deepsleep_init_regmap();
	if (sprd_deep_get_ap_clk())
		pr_err("%s,failed to find ap clk\n", __func__);

#ifndef CONFIG_SPRD_PM_DEBUG
	pm_debug_init();
#endif
}

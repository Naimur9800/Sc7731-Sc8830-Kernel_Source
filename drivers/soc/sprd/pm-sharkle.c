/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
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
#include <linux/io.h>
#include <linux/of.h>
#include <linux/kthread.h>
#include <asm/irqflags.h>
#include <linux/cpu_pm.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/wakelock.h>
#include <linux/suspend.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#ifdef CONFIG_SPRD_SIPC
#include <linux/sipc.h>
#endif

#define AP_CLK_DEBUG
#define PM_PRINT_ENABLE

#define	INTCV_IRQ_MSKSTS	(0x0000)
#define	INTCV_IRQ_RAW		(0x0004)
#define	INTCV_IRQ_EN		(0x0008)

#define AP_INTC_NUM	4
#define AON_INTC_NUM	1
#define AON_INTC   4
#define INTC_NUM	(AP_INTC_NUM + AON_INTC_NUM)
#define SPRD_INTC_NUM	12
#define SPRD_HARD_INT_NUM_EACH_INTC	32
#define SPRD_HARD_INTERRUPT_NUM	192

#define SAVE_GLOBAL_REG() do { \
	bak_restore_ap_apb(1); \
	bak_restore_ap_ahb(1); \
	bak_restore_ap_clk(1); \
	bak_restore_ap_intc(1); \
	} while (0)
#define RESTORE_GLOBAL_REG() do { \
	bak_restore_ap_apb(0); \
	bak_restore_ap_ahb(0); \
	bak_restore_ap_clk(0); \
	bak_restore_ap_intc(0); \
	} while (0)

#define BITS_CHECK(idx, __offset, __bits) do { \
	uint32_t mid; \
	if (idx == AP_APB) \
		regmap_read(sprd_deep.ap_apb, __offset, &mid); \
	else if (idx == AP_AHB) \
		regmap_read(sprd_deep.ap_ahb, __offset, &mid); \
	else if (idx == AON_APB) \
		regmap_read(sprd_deep.aon_apb, __offset, &mid); \
	if (mid & __bits) \
		pr_info(#__bits " not 0\n"); \
	} while (0)

#define PRINTK_REG(idx, _offset) do { \
	uint32_t mid; \
	if (idx == AP_APB) \
		regmap_read(sprd_deep.ap_apb, _offset, &mid); \
	else if (idx == AP_AHB) \
		regmap_read(sprd_deep.ap_ahb, _offset, &mid); \
	else if (idx == AON_APB) \
		regmap_read(sprd_deep.aon_apb, _offset, &mid); \
	else if (idx == PMU_APB) \
		regmap_read(sprd_deep.pmu_apb, _offset, &mid); \
	pr_info("##--"#_offset": 0x%08x\n", mid); \
	} while (0)

static u32 print_thread_enable = 1;
static u32 print_thread_interval = 30;

static u32 sprd_hard_irq[SPRD_HARD_INTERRUPT_NUM] = {0, };
static u32 sprd_irqs_sts[SPRD_INTC_NUM] = {0, };

#ifdef PM_PRINT_ENABLE
static struct wake_lock messages_wakelock;
#endif

struct ap_apb_reg_bak {
	u32 apb_eb;
	u32 apb_misc_ctrl;
};

struct ap_ahb_reg_bak {
	u32 ahb_eb;
	u32 ap_sys_frc_sleep;
	u32 ap_sys_auto_sleep;
	u32 ap_async_brg;
	u32 m0_lpc;
	u32 m1_lpc;
	u32 m2_lpc;
	u32 m3_lpc;
	u32 m4_lpc;
	u32 m5_lpc;
	u32 m6_lpc;
	u32 m7_lpc;
	u32 m8_lpc;
	u32 m9_lpc;
	u32 main_lpc;
	u32 s0_lpc;
	u32 s1_lpc;
	u32 s2_lpc;
	u32 s3_lpc;
	u32 s4_lpc;
	u32 s5_lpc;
	u32 otg_phy_tune;
	u32 otg_phy_test;
	u32 otg_phy_ctrl;
	u32 otg_ctrl0;
	u32 otg_ctrl1;
	u32 ca53_ckg_sel_cfg;
	u32 reg_ap_ahb_ap_qos0;
	u32 reg_ap_ahb_ap_qos1;
	u32 reg_ap_ahb_ap_qos2;
};

struct ap_intc_reg_bak {
	u32 ap_intc0;
	u32 ap_intc1;
	u32 ap_intc2;
	u32 ap_intc3;
};

static u32 ap_apb_reg[] = {
	REG_AP_APB_APB_EB,
	REG_AP_APB_APB_MISC_CTRL,
};

static u32 ap_ahb_reg[] = {
	REG_AP_AHB_AHB_EB,
	REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG,
	REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG,
	REG_AP_AHB_AP_ASYNC_BRG,
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
	REG_AP_AHB_S5_LPC,
	REG_AP_AHB_OTG_PHY_TUNE,
	REG_AP_AHB_OTG_PHY_TEST,
	REG_AP_AHB_OTG_PHY_CTRL,
	REG_AP_AHB_OTG_CTRL0,
	REG_AP_AHB_OTG_CTRL1,
	REG_AP_AHB_CA53_CKG_SEL_CFG,
	REG_AP_AHB_AP_QOS0,
	REG_AP_AHB_AP_QOS1,
	REG_AP_AHB_AP_QOS2
};

enum {
	AP_APB = 0,
	AP_AHB,
	PMU_APB,
	AON_APB,
};

struct deepsleep_ap_clk {
	const char *name;

	struct clk *clk;
	struct clk *old_parent;
	struct clk *default_parent;

	unsigned long current_rate;
};

static struct ap_deepsleep_node {
	struct device_node *np;
	struct deepsleep_ap_clk *ap_clk;
	/*ap intc0~intc4*/
	struct regmap *intc[INTC_NUM];
	struct regmap *ap_apb;
	struct regmap *ap_ahb;
	struct regmap *pmu_apb;
	struct regmap *aon_apb;

	struct ap_apb_reg_bak ap_apb_reg_saved;
	struct ap_ahb_reg_bak ap_ahb_reg_saved;
	struct ap_intc_reg_bak ap_intc_reg_saved;

	int ap_clk_cnt;
	int sleep_cnt;
} sprd_deep;

static	int get_ap_clk_count(void)
{
	int i, j, count = 0;
	struct device_node *state_node;
	char	*clk_group_name = "sprd,deep-ap-clk0";

	for (i = 0; ; i++) {
		sprintf(clk_group_name, "sprd,deep-ap-clk%d", i);
		state_node = of_parse_phandle(sprd_deep.np,
				clk_group_name, 0);

		if (!state_node)
			break;

		for (j = 0; ; j++) {
			state_node = of_parse_phandle(sprd_deep.np,
					clk_group_name, j);
			if (!state_node)
				break;

			count++;
		}
	}

	return count;
}

static struct clk *get_ap_clk_default_parent(int idx)
{
	struct device_node *parent_node;
	struct clk *default_parent;
	const char *parent_clk_name = "default";

	parent_node = of_parse_phandle(sprd_deep.np,
			"sprd,deep-ap-clkp", idx);
	if (!parent_node) {
		pr_info("%s, Failed to get parent clk node\n", __func__);
		return NULL;
	}

	of_property_read_string_index(parent_node,
			"clock-output-names", 0,
			&parent_clk_name);
	if (!parent_clk_name) {
		pr_info("%s, Failed to get parent_clk[%d] name\n",
			__func__, idx);
		return NULL;
	}

	default_parent = of_clk_get(sprd_deep.np, sprd_deep.ap_clk_cnt + idx);
	if (!default_parent) {
		pr_info("%s, Failed to get parent clk%d\n", __func__, idx);
		return NULL;
	}

	return default_parent;
}

static int sprd_deep_get_ap_clk(void)
{
	int i, j, clk_idx = 0, flag = 0;
	struct device_node *state_node;
	struct deepsleep_ap_clk *ap_clk;
	char *clk_group_name = "sprd,deep-ap-clk0";

	sprd_deep.ap_clk_cnt = get_ap_clk_count();
	if (!sprd_deep.ap_clk_cnt) {
		pr_info("%s, ap_clk_cnt = 0\n", __func__);
		return -ENODEV;
	}
	pr_info("%s, get %d ap_clk\n", __func__, sprd_deep.ap_clk_cnt);

	ap_clk = kcalloc(sprd_deep.ap_clk_cnt,
			sizeof(struct deepsleep_ap_clk), GFP_KERNEL);
	if (!ap_clk)
		return -ENOMEM;

	sprd_deep.ap_clk = ap_clk;

	for (i = 0; ; i++) {
		sprintf(clk_group_name, "sprd,deep-ap-clk%d", i);
		state_node = of_parse_phandle(sprd_deep.np,
				clk_group_name, 0);
		if (!state_node)
			break;

		for (j = 0; ; j++) {
			state_node = of_parse_phandle(sprd_deep.np,
					clk_group_name, j);
			if (!state_node)
				break;

			of_property_read_string_index(state_node,
					"clock-output-names", 0,
					&(ap_clk[clk_idx].name));

			ap_clk[clk_idx].clk = of_clk_get(sprd_deep.np, clk_idx);
			if (!(ap_clk[clk_idx].clk)) {
				pr_info("%s, Failed to get %s(clk_idx = %d)\n",
						__func__,
						ap_clk[clk_idx].name,
						clk_idx);
				kfree(ap_clk);
				ap_clk = NULL;
				flag = 1;
				break;
			}

			ap_clk[clk_idx].default_parent =
				get_ap_clk_default_parent(i);
			if (!(ap_clk[clk_idx].default_parent)) {
				pr_info("%s,Fail to get defalt parent for %s\n",
						__func__, ap_clk[clk_idx].name);
				kfree(ap_clk);
				ap_clk = NULL;
				flag = 1;
				break;
			}

			pr_info("%s get ap clk (idx = %d) %s\n", __func__,
					clk_idx, ap_clk[clk_idx].name);

			clk_idx++;
		}
	}

	if (flag)
		return -ENODEV;
	else
		return 0;
}

static void sprd_deep_store_ap_clk(void)
{
	int i;

	for (i = 0; i < sprd_deep.ap_clk_cnt; i++) {
		sprd_deep.ap_clk[i].old_parent =
			clk_get_parent(sprd_deep.ap_clk[i].clk);
		sprd_deep.ap_clk[i].current_rate =
			clk_get_rate(sprd_deep.ap_clk[i].clk);
#ifdef AP_CLK_DEBUG
	pr_info("%s, %s, current_rate = %lu\n", __func__,
			sprd_deep.ap_clk[i].name,
			sprd_deep.ap_clk[i].current_rate);
#endif
	}
}

static void sprd_deep_restore_ap_clk(void)
{
	int i;

	for (i = 0; i < sprd_deep.ap_clk_cnt; i++) {
		clk_set_parent(sprd_deep.ap_clk[i].clk,
				sprd_deep.ap_clk[i].default_parent);
		clk_set_parent(sprd_deep.ap_clk[i].clk,
				sprd_deep.ap_clk[i].old_parent);
		clk_set_rate(sprd_deep.ap_clk[i].clk,
				sprd_deep.ap_clk[i].current_rate);
	}
#ifdef AP_CLK_DEBUG
	sprd_deep_store_ap_clk();
#endif
}

static void bak_restore_ap_clk(int bak)
{
	if (bak)
		sprd_deep_store_ap_clk();
	else
		sprd_deep_restore_ap_clk();
}

static void bak_restore_ap_apb(int bak)
{
	int i = 0;
	int size = sizeof(ap_apb_reg) / sizeof(u32);
	u32 *apb_reg = (u32 *)&(sprd_deep.ap_apb_reg_saved);

	if (bak) {/*store*/
		for (i = 0; i < size; i++)
			regmap_read(sprd_deep.ap_apb,
				ap_apb_reg[i], &apb_reg[i]);

	} else {/*restore*/
		for (i = 0; i < size; i++)
			regmap_write(sprd_deep.ap_apb,
				ap_apb_reg[i], apb_reg[i]);
	}
}

static void bak_restore_ap_ahb(int bak)
{
	int i = 0;
	int size = sizeof(ap_ahb_reg) / sizeof(u32);
	u32 *ahb_reg = (u32 *)&(sprd_deep.ap_ahb_reg_saved);

	if (bak) {
		for (i = 0; i < size; i++)
			regmap_read(sprd_deep.ap_ahb,
				ap_ahb_reg[i], &ahb_reg[i]);

	} else {
		for (i = 0; i < size; i++)
			regmap_write(sprd_deep.ap_ahb,
				ap_ahb_reg[i], ahb_reg[i]);
	}
}

static void bak_restore_ap_intc(int bak)
{
	int i = 0;
	u32 *intc_reg = (u32 *)&(sprd_deep.ap_intc_reg_saved);

	if (bak) {
		for (i = 0; i < AP_INTC_NUM; i++)
			regmap_read(sprd_deep.intc[i],
				INTCV_IRQ_EN, &intc_reg[i]);

	} else {
		for (i = 0; i < AP_INTC_NUM; i++)
			regmap_write(sprd_deep.intc[i],
				INTCV_IRQ_EN, intc_reg[i]);
	}
}

static void sprd_deep_disable_ap_apb(void)
{
	/*if need*/
}

static void sprd_deep_disable_ap_ahb(void)
{
	/*AP_PERI_FORCE_ON CLR*/
	regmap_update_bits(sprd_deep.ap_ahb, REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG,
		BIT_AP_AHB_PERI_FORCE_ON,
		(unsigned int)(~BIT_AP_AHB_PERI_FORCE_ON));

	/*SET AP_PERI_FORCE_OFF*/
	regmap_update_bits(sprd_deep.ap_ahb, REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG,
		BIT_AP_AHB_PERI_FORCE_OFF, BIT_AP_AHB_PERI_FORCE_OFF);
}

static void sprd_deep_disable_aon_apb(void)
{
	regmap_update_bits(sprd_deep.aon_apb, REG_AON_APB_APB_EB0,
		BIT_AON_APB_AP_TMR0_EB | BIT_AON_APB_AON_TMR_EB,
		(unsigned int)(~(BIT_AON_APB_AP_TMR0_EB |
		BIT_AON_APB_AON_TMR_EB)));

	regmap_update_bits(sprd_deep.aon_apb, REG_AON_APB_APB_EB1,
		BIT_AON_APB_AP_TMR2_EB | BIT_AON_APB_AP_TMR1_EB,
		(unsigned int)(~(BIT_AON_APB_AP_TMR2_EB |
		BIT_AON_APB_AP_TMR1_EB)));

	regmap_update_bits(sprd_deep.aon_apb, REG_AON_APB_APB_EB1,
		BIT_AON_APB_DISP_EMC_EB,
		(unsigned int)(~BIT_AON_APB_DISP_EMC_EB));
}

static void sprd_deep_enable_aon_apb(void)
{
	regmap_update_bits(sprd_deep.aon_apb, REG_AON_APB_APB_EB0,
		BIT_AON_APB_AP_TMR0_EB | BIT_AON_APB_AON_TMR_EB,
		BIT_AON_APB_AP_TMR0_EB | BIT_AON_APB_AON_TMR_EB);

	regmap_update_bits(sprd_deep.aon_apb, REG_AON_APB_APB_EB1,
		BIT_AON_APB_AP_TMR2_EB | BIT_AON_APB_AP_TMR1_EB,
		BIT_AON_APB_AP_TMR2_EB | BIT_AON_APB_AP_TMR1_EB);

	regmap_update_bits(sprd_deep.aon_apb, REG_AON_APB_APB_EB1,
		BIT_AON_APB_DISP_EMC_EB, BIT_AON_APB_DISP_EMC_EB);
}

static void sprd_deep_disable_module(void)
{
	sprd_deep_disable_ap_apb();
	sprd_deep_disable_ap_ahb();
	sprd_deep_disable_aon_apb();
}

static void sprd_deep_ap_ahb_check(void)
{
	PRINTK_REG(AP_AHB, REG_AP_AHB_AHB_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_DSI_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_DISPC_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_VSP_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_GSP_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_DMA_PUB_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_OTG_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_CE_PUB_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_SDIO0_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_SDIO1_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_NANDC_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_EMMC_EB);
}

static void sprd_deep_ap_apb_check(void)
{
	PRINTK_REG(AP_APB, REG_AP_APB_APB_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_SIM0_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_IIS0_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_SPI0_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_SPI2_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_I2C0_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_I2C1_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_I2C2_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_I2C3_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_I2C4_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_UART1_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_INTC0_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_INTC1_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_INTC2_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_INTC3_EB);
}

static void sprd_deep_aon_apb_check(void)
{
	PRINTK_REG(AON_APB, REG_AON_APB_APB_EB0);
	PRINTK_REG(AON_APB, REG_AON_APB_APB_EB2);
	BITS_CHECK(AON_APB, REG_AON_APB_APB_EB0, BIT_AON_APB_GPU_EB);
	BITS_CHECK(AON_APB, REG_AON_APB_APB_EB0, BIT_AON_APB_MM_EB);
	BITS_CHECK(AON_APB, REG_AON_APB_APB_EB0, BIT_AON_APB_CA53_DAP_EB);
	BITS_CHECK(AON_APB, REG_AON_APB_APB_EB2, BIT_AON_APB_AP_DAP_EB);
}

static void sprd_deep_condition_check(void)
{
	pr_info("#####Enter Deepsleep  Condition Check!######\n");
	pr_info("#####20E0_0000:Enter AP_AHB  Condition Check!#####\n");
	sprd_deep_ap_ahb_check();
	pr_info("#####7130_0000:Enter AP_APB  Condition Check!#####\n");
	sprd_deep_ap_apb_check();
	pr_info("#####402E_0000:402E_00B0:Enter AON_APB  Condition Check!#####\n");
	sprd_deep_aon_apb_check();
}

static void sprd_deep_print_last_reg(void)
{
	PRINTK_REG(AP_AHB, REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG);
	PRINTK_REG(AP_AHB, REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG);
	PRINTK_REG(AP_AHB, REG_AP_AHB_MCU_PAUSE);
	PRINTK_REG(AP_AHB, REG_AP_AHB_CA53_STANDBY_STATUS);
	/*PRINTK_REG(AP_AHB, REG_AP_AHB_AHB_EB);*/
	/*PRINTK_REG(AP_APB, REG_AP_APB_APB_EB);*/
	/*PRINTK_REG(AON_APB, REG_AON_APB_APB_EB0);*/
	PRINTK_REG(AON_APB, REG_AON_APB_APB_EB1);
	/*PRINTK_REG(AON_APB, REG_AON_APB_APB_EB2);*/
	PRINTK_REG(AON_APB, REG_AON_APB_PWR_CTRL);
	/*PRINTK_REG(PMU_APB, REG_PMU_APB_PWR_STATUS0_DBG);*/
	/*PRINTK_REG(PMU_APB, REG_PMU_APB_PWR_STATUS1_DBG);*/
	/*PRINTK_REG(PMU_APB, REG_PMU_APB_PWR_STATUS2_DBG);*/
	/*PRINTK_REG(PMU_APB, REG_PMU_APB_PWR_STATUS3_DBG);*/
	PRINTK_REG(PMU_APB, REG_PMU_APB_SLEEP_STATUS);
	PRINTK_REG(PMU_APB, REG_PMU_APB_WTLCP_TGDSP_DSLP_ENA);
	PRINTK_REG(PMU_APB, REG_PMU_APB_WTLCP_LDSP_DSLP_ENA);
	PRINTK_REG(PMU_APB, REG_PMU_APB_PUBCP_DSLP_ENA);
	PRINTK_REG(PMU_APB, REG_PMU_APB_WTLCP_DSLP_ENA);
	PRINTK_REG(PMU_APB, REG_PMU_APB_SP_SYS_DSLP_ENA);
	PRINTK_REG(PMU_APB, REG_PMU_APB_WCN_SYS_DSLP_ENA);
	PRINTK_REG(PMU_APB, REG_PMU_APB_WIFI_WRAP_DSLP_ENA);
	PRINTK_REG(PMU_APB, REG_PMU_APB_GNSS_WRAP_DSLP_ENA);
	PRINTK_REG(PMU_APB, REG_PMU_APB_CA53_TOP_DSLP_ENA);
	PRINTK_REG(PMU_APB, REG_PMU_APB_AP_DSLP_ENA);
	PRINTK_REG(PMU_APB, REG_PMU_APB_DDR_SLEEP_CTRL);
	PRINTK_REG(PMU_APB, REG_PMU_APB_SLEEP_CTRL);
	PRINTK_REG(PMU_APB, REG_PMU_APB_AP_SYS_SLEEP_CNT);
	PRINTK_REG(PMU_APB, REG_PMU_APB_AP_DEEP_SLEEP_CNT);
}

static void sprd_deep_pwr_status0(void)
{
	uint32_t pwr_status0;

	regmap_read(sprd_deep.pmu_apb,
		REG_PMU_APB_PWR_STATUS0_DBG,
		&pwr_status0);
	PRINTK_REG(PMU_APB, REG_PMU_APB_PWR_STATUS0_DBG);
	pr_info("##--status of power domain 'PD_CA53_TOP' :0x%x\n",
		(pwr_status0)&0x1F);
	pr_info("##--status of power domain 'PD_CA53_C0' :0x%x\n",
		((pwr_status0)&(0x1F<<5))>>5);
	pr_info("##--status of power domain 'PD_CA53_C1' :0x%x\n",
		((pwr_status0)&(0x1F<<10))>>10);
	pr_info("##--status of power domain 'PD_CA53_C2' :0x%x\n",
		((pwr_status0)&(0x1F<<15))>>15);
	pr_info("##--status of power domain 'PD_CA53_C3' :0x%x\n",
		((pwr_status0)&(0x1F<<20))>>20);
	pr_info("##--status of power domain 'PD_AP_SYS' :0x%x\n",
		((pwr_status0)&(0x1F<<25))>>25);
}

static void sprd_deep_pwr_status1(void)
{
	uint32_t pwr_status1;

	regmap_read(sprd_deep.pmu_apb,
		REG_PMU_APB_PWR_STATUS1_DBG,
		&pwr_status1);
	PRINTK_REG(PMU_APB, REG_PMU_APB_PWR_STATUS1_DBG);
	pr_info("##--status of power domain 'PD_WTLCP_HU3GE_A' :0x%x\n",
		(pwr_status1)&0x1F);
	pr_info("##--status of power domain 'PD_WTLCP_TGDSP' :0x%x\n",
		((pwr_status1)&(0x1F<<5))>>5);
	pr_info("##--status of power domain 'PD_WTLCP_LDSP' :0x%x\n",
		((pwr_status1)&(0x1F<<10))>>10);
	pr_info("##--status of power domain 'PD_WIFI_WRAP' :0x%x\n",
		((pwr_status1)&(0x1F<<15))>>15);
	pr_info("##--status of power domain 'PD_WTLCP_LTE_P2' :0x%x\n",
		((pwr_status1)&(0x1F<<20))>>20);
	pr_info("##--status of power domain 'PD_WTLCP_LTE_P1' :0x%x\n",
		((pwr_status1)&(0x1F<<25))>>25);
}

static void sprd_deep_pwr_status2(void)
{
	uint32_t pwr_status2;

	regmap_read(sprd_deep.pmu_apb,
		REG_PMU_APB_PWR_STATUS2_DBG,
		&pwr_status2);
	PRINTK_REG(PMU_APB, REG_PMU_APB_PWR_STATUS2_DBG);
	pr_info("##--status of power domain 'PD_WTLCP_SYS' :0x%x\n",
		(pwr_status2)&0x1F);
	pr_info("##--status of power domain 'PD_PUBCP_SYS' :0x%x\n",
		((pwr_status2)&(0x1F<<5))>>5);
	pr_info("##--status of power domain 'PD_WTLCP_LTE_P3' :0x%x\n",
		((pwr_status2)&(0x1F<<10))>>10);
	pr_info("##--status of power domain 'PD_WTLCP_LTE_P4' :0x%x\n",
		((pwr_status2)&(0x1F<<15))>>15);
	pr_info("##--status of power domain 'PD_PUB_SYS' :0x%x\n",
		((pwr_status2)&(0x1F<<20))>>20);
	pr_info("##--status of power domain 'PD_GNSS_WRAP' :0x%x\n",
		((pwr_status2)&(0x1F<<25))>>25);
}

static void sprd_deep_pwr_status3(void)
{
	uint32_t pwr_status3;

	regmap_read(sprd_deep.pmu_apb,
		REG_PMU_APB_PWR_STATUS3_DBG,
		&pwr_status3);
	PRINTK_REG(PMU_APB, REG_PMU_APB_PWR_STATUS3_DBG);
	pr_info("##--status of power domain 'PD_WCN_SYS' :0x%x\n",
		(pwr_status3)&0x1F);
	pr_info("##--status of power domain 'PD_GPU_TOP' :0x%x\n",
		((pwr_status3)&(0x1F<<5))>>5);
	pr_info("##--status of power domain 'PD_MM_TOP' :0x%x\n",
		((pwr_status3)&(0x1F<<10))>>10);
}
static void sprd_deep_print_power_domain(void)
{
	pr_info("#####402B00BC:PWR_STATUS0 POWER DOMAIN#####\n");
	sprd_deep_pwr_status0();
	pr_info("#####402B00C0:PWR_STATUS1 POWER DOMAIN#####\n");
	sprd_deep_pwr_status1();
	pr_info("#####402B00C4:PWR_STATUS2 POWER DOMAIN#####\n");
	sprd_deep_pwr_status2();
	pr_info("#####402B010C:PWR_STATUS3 POWER DOMAIN#####\n");
	sprd_deep_pwr_status3();
}

static void sprd_deep_intc_status(void)
{
	int i;
	uint32_t intc_bits[INTC_NUM], intc_irq_raw[INTC_NUM],
		 intc_irq_en[INTC_NUM];

	for (i = 0; i < INTC_NUM; i++) {
		regmap_read(sprd_deep.intc[i], INTCV_IRQ_MSKSTS,
			&intc_bits[i]);
		regmap_read(sprd_deep.intc[i], INTCV_IRQ_RAW,
			&intc_irq_raw[i]);
		regmap_read(sprd_deep.intc[i], INTCV_IRQ_EN,
			&intc_irq_en[i]);
		if (i >= AP_INTC_NUM)
			pr_info("AON INTC: MSKSTS 0x%08x, RAW 0x%08x, EN 0x%08x\n",
				intc_bits[i], intc_irq_raw[i], intc_irq_en[i]);
		else
			pr_info("AP INTC%d: MSKSTS 0x%08x, RAW 0x%08x, EN 0x%08x\n",
					i, intc_bits[i],
					intc_irq_raw[i], intc_irq_en[i]
				   );
	}
}

static void sprd_deep_sleep_status(void)
{
	sprd_deep_condition_check();
	sprd_deep_print_last_reg();
	sprd_deep_print_power_domain();
}

static void parse_hard_irq(unsigned long val, unsigned long idx)
{
	int i;

	for (i = 0; i < SPRD_HARD_INT_NUM_EACH_INTC; i++) {
		if (test_and_clear_bit(i, &val))
			sprd_hard_irq[idx * SPRD_HARD_INT_NUM_EACH_INTC + i]++;
	}
}

static void sprd_deep_hard_irq_set(void)
{
	int i;
	u32 irq_status;

	for (i = 0; i < INTC_NUM; i++) {
		regmap_read(sprd_deep.intc[i], INTCV_IRQ_MSKSTS,
			&sprd_irqs_sts[i]);

		regmap_read(sprd_deep.intc[i], INTCV_IRQ_RAW,
			&irq_status);
		parse_hard_irq(irq_status, i);
	}
}

static void sprd_deep_hard_irq_loop(void)
{
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
			pr_info("wake up by thm0\n");
		if (sprd_hard_irq[25])
			pr_info("wake up by adi\n");
		if (sprd_hard_irq[24])
			pr_info("wake up by mdar\n");
		if (sprd_hard_irq[23])
			pr_info("wake up by vbc ad01!\n");
		if (sprd_hard_irq[22])
			pr_info("wake up by vbc da01\n");
		if (sprd_hard_irq[21])
			pr_info("wake up by vbc_afifo_err\n");
		if (sprd_hard_irq[20])
			pr_info("wake up by aud\n");
	}

	if (sprd_irqs_sts[1]) {
		if (sprd_hard_irq[63])
			pr_info("wake up by sec_gpio\n");
		if (sprd_hard_irq[62])
			pr_info("wake up by sec_eic\n");
		if (sprd_hard_irq[61])
			pr_info("wake up by dmc_mpu_vio\n");
		if (sprd_hard_irq[60])
			pr_info("wake up by emmc\n");
		if (sprd_hard_irq[56])
			pr_info("wake up by sec_rtc\n");
		if (sprd_hard_irq[55])
			pr_info("wake up by otg\n");
		if (sprd_hard_irq[54])
			pr_info("wake up by ce_pub\n");
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

	if (sprd_irqs_sts[2]) {
		if (sprd_hard_irq[84])
			pr_info("wake up by cp1_wdg\n");
		if (sprd_hard_irq[83])
			pr_info("wake up by cp0_wdg\n");
		if (sprd_hard_irq[70])
			pr_info("wake up by aon_dma_ap\n");
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
			pr_info("wake up by dcam1\n");
	}

	if (sprd_irqs_sts[3]) {
		if (sprd_hard_irq[126])
			pr_info("wake up by ~nEXTRRIRQ\n");
		if (sprd_hard_irq[125])
			pr_info("wake up by isp_ch1\n");
		if (sprd_hard_irq[124])
			pr_info("wake up by a53_wdg\n");
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

}

static void sprd_deep_wakeup_source(void)
{
	sprd_deep.sleep_cnt++;
	pr_info("Deep Sleep %d Times\n", sprd_deep.sleep_cnt);

	sprd_deep_hard_irq_set();
	sprd_deep_hard_irq_loop();
}

static int sprd_deep_print_thread(void *data)
{
	unsigned int cnt = 0;

	while (1) {
		wake_lock(&messages_wakelock);
		if (print_thread_enable)
			sprd_deep_sleep_status();
		if (!pm_get_wakeup_count(&cnt, false)) {
			pr_info("PM: has wakeup events in progressing\n");
			pm_print_active_wakeup_sources();

		}
		msleep(100);
		wake_unlock(&messages_wakelock);
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(print_thread_interval * HZ);
	}

	return 0;
}

static void sprd_deep_debugfs_init(void)
{
	static struct dentry *dentry_debug_root;

	dentry_debug_root = debugfs_create_dir("power", NULL);
	if (IS_ERR(dentry_debug_root) || !dentry_debug_root) {
		pr_info("%s, Failed to create debugfs directory\n", __func__);
		dentry_debug_root = NULL;
		return;
	}

	debugfs_create_u32("print_thread_enable", 0644, dentry_debug_root,
		   &print_thread_enable);
	debugfs_create_u32("print_thread_interval", 0644, dentry_debug_root,
		   &print_thread_interval);
}

static void sprd_deep_debug_init(void)
{
#ifdef PM_PRINT_ENABLE

	struct task_struct *task;

	wake_lock_init(&messages_wakelock, WAKE_LOCK_SUSPEND,
		"pm_message_wakelock");
	task = kthread_create(sprd_deep_print_thread, NULL,
		"sprd_deep_print_thread");
	if (!task)
		pr_info("%s, Failed to create print thread", __func__);
	else
		wake_up_process(task);
#endif
	sprd_deep_debugfs_init();
}

static void sprd_deep_intc_init(void)
{
	regmap_update_bits(sprd_deep.aon_apb, REG_AON_APB_APB_EB1,
		BIT_AON_APB_AON_FOR_AP_INTC_EB,
		BIT_AON_APB_AON_FOR_AP_INTC_EB);

	regmap_update_bits(sprd_deep.ap_apb, REG_AP_APB_APB_EB,
		BIT_AP_APB_INTC0_EB | BIT_AP_APB_INTC1_EB |
		BIT_AP_APB_INTC2_EB | BIT_AP_APB_INTC3_EB,
		BIT_AP_APB_INTC0_EB | BIT_AP_APB_INTC1_EB |
		BIT_AP_APB_INTC2_EB | BIT_AP_APB_INTC3_EB);

	regmap_update_bits(sprd_deep.intc[AON_INTC], INTCV_IRQ_EN,
		BIT(2) | /* int_req_dma */
		BIT(4) | /* int_req_sec_gpio */
		BIT(9) | /* int_req_ca53_wdg */
		BIT(15) | /* int_req_gpio */
		BIT(17) | /* int_req_kpd */
		BIT(18) | /* int_req_ana */
		BIT(26), /* int_req_aon_tmr */
					BIT(2) | /* int_req_dma */
					BIT(4) | /* int_req_sec_gpio */
					BIT(9) | /* int_req_ca53_wdg */
					BIT(15) | /* int_req_gpio */
					BIT(17) | /* int_req_kpd */
					BIT(18) | /* int_req_ana */
					BIT(26) /* int_req_aon_tmr */
		);

}

static int sprd_deep_init_glb_regmap(void)
{
	int i;
	char *ap_intc_name = "sprd,sys-ap-intc0";

	sprd_deep.np = of_find_node_by_name(NULL, "deep-sleep");
	if (!sprd_deep.np) {
		pr_info("%s, Failed to find deep-sleep node\n", __func__);
		return -ENODEV;
	}

	sprd_deep.ap_apb = syscon_regmap_lookup_by_phandle(sprd_deep.np,
			"sprd,sys-ap-apb");
	sprd_deep.ap_ahb = syscon_regmap_lookup_by_phandle(sprd_deep.np,
			"sprd,sys-ap-ahb");
	sprd_deep.aon_apb = syscon_regmap_lookup_by_phandle(sprd_deep.np,
			"sprd,sys-aon-apb");
	sprd_deep.pmu_apb = syscon_regmap_lookup_by_phandle(sprd_deep.np,
			"sprd,sys-pmu-apb");
	if ((!sprd_deep.ap_apb) || (!sprd_deep.ap_ahb) ||
			(!sprd_deep.aon_apb) || (!sprd_deep.pmu_apb)) {
		pr_info("%s, Failed to init glb reg\n", __func__);
		return -ENODEV;
	}

	for (i = 0; i < INTC_NUM; i++) {
		sprintf(ap_intc_name, "sprd,sys-ap-intc%d", i);

		if (i >= AP_INTC_NUM)
			sprd_deep.intc[i] =
				syscon_regmap_lookup_by_phandle(sprd_deep.np,
				"sprd,sys-aon-intc");
		else
			sprd_deep.intc[i] =
				syscon_regmap_lookup_by_phandle(sprd_deep.np,
				ap_intc_name);
		if (!(sprd_deep.intc[i])) {
			pr_info("%s, failed to find intc[%d]\n",
				__func__, i);
			return -ENODEV;
		}
	}

	sprd_deep_intc_init();

	return 0;
}

static int sprd_deep_pm_notifier(struct notifier_block *self,
		unsigned long cmd, void *v)
{
	switch (cmd) {
	case CPU_CLUSTER_PM_ENTER:
		SAVE_GLOBAL_REG();
		sprd_deep_disable_module();
		sprd_deep_sleep_status();
		sprd_deep_intc_status();
		break;
	case CPU_CLUSTER_PM_ENTER_FAILED:
	case CPU_CLUSTER_PM_EXIT:
		sprd_deep_enable_aon_apb();
		RESTORE_GLOBAL_REG();
		sprd_deep_intc_status();
		sprd_deep_wakeup_source();
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block sprd_pm_notifier_block = {
	.notifier_call = sprd_deep_pm_notifier,
};

static int __init sprd_deep_init(void)
{
	int ret;

	pr_info("##### Deep sleep init start #####\n");

	ret = sprd_deep_init_glb_regmap();
	if  (ret) {
		pr_info("%s,Failed to init glb reg\n", __func__);
		pr_info("##### Deep sleep init failed #####\n");
		return ret;
	}

	ret = sprd_deep_get_ap_clk();
	if (ret) {
		pr_info("%s,Failed to find ap clk\n", __func__);
		pr_info("##### Deep sleep init failed #####\n");
		return ret;
	}

	cpu_pm_register_notifier(&sprd_pm_notifier_block);

	sprd_deep_debug_init();

	pr_info("##### Deep sleep init successfully #####\n");

	return 0;
}

subsys_initcall(sprd_deep_init);

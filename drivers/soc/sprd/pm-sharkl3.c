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

#define AP_INTC_NUM	6
#define SPRD_INTC_NUM	12
#define SPRD_HARD_INT_NUM_EACH_INTC	32
#define SPRD_HARD_INTERRUPT_NUM	192

#define SAVE_GLOBAL_REG() do { \
	bak_restore_ap_apb(1); \
	bak_restore_ap_ahb(1); \
	bak_restore_ap_clk(1); \
	} while (0)
#define RESTORE_GLOBAL_REG() do { \
	bak_restore_ap_apb(0); \
	bak_restore_ap_ahb(0); \
	bak_restore_ap_clk(0); \
	} while (0)

#define BITS_CHECK(idx, __offset, __bits) do { \
	uint32_t mid; \
	if (idx == AP_APB) \
		regmap_read(sprd_deep.ap_apb, __offset, &mid); \
	else if (idx == AP_AHB) \
		regmap_read(sprd_deep.ap_ahb, __offset, &mid); \
	else if (idx == AON_APB) \
		regmap_read(sprd_deep.aon_apb, __offset, &mid); \
	if (mid && __bits) \
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

struct ap_intc_en {
	unsigned int intc_num;
	unsigned int int_channel;
};

struct ap_apb_reg_bak {
	u32 apb_eb;
	u32 apb_misc_ctrl;
};

struct ap_ahb_reg_bak {
	u32 ahb_eb;
	u32 m1_lpc;
	u32 m2_lpc;
	u32 m3_lpc;
	u32 m4_lpc;
	u32 m5_lpc;
	u32 m6_lpc;
	u32 m7_lpc;
	u32 m8_lpc;
	u32 main_lpc;
	u32 s0_lpc;
	u32 s1_lpc;
	u32 s2_lpc;
	u32 s3_lpc;
	u32 s4_lpc;
	u32 s5_lpc;
	u32 m10_lpc;
};

static u32 ap_apb_reg[] = {
	REG_AP_APB_APB_EB,
	REG_AP_APB_APB_MISC_CTRL,
};

static u32 ap_ahb_reg[] = {
	REG_AP_AHB_AHB_EB,
	REG_AP_AHB_M1_LPC,
	REG_AP_AHB_M2_LPC,
	REG_AP_AHB_M3_LPC,
	REG_AP_AHB_M4_LPC,
	REG_AP_AHB_M5_LPC,
	REG_AP_AHB_M6_LPC,
	REG_AP_AHB_M7_LPC,
	REG_AP_AHB_M8_LPC,
	REG_AP_AHB_MAIN_LPC,
	REG_AP_AHB_S0_LPC,
	REG_AP_AHB_S1_LPC,
	REG_AP_AHB_S2_LPC,
	REG_AP_AHB_S3_LPC,
	REG_AP_AHB_S4_LPC,
	REG_AP_AHB_S5_LPC,
	REG_AP_AHB_M10_LPC,
};

enum {
	AP_APB = 0,
	AP_AHB,
	PMU_APB,
	AON_APB,
};

struct deepsleep_ap_clk {
	struct clk *clk;
	struct clk *parent;
	unsigned long current_rate;
};

static struct ap_deepsleep_node {
	struct device_node *np;
	struct deepsleep_ap_clk *ap_clk;
	/*ap intc0~intc5*/
	struct regmap *intc[AP_INTC_NUM];
	struct regmap *ap_apb;
	struct regmap *ap_ahb;
	struct regmap *pmu_apb;
	struct regmap *aon_apb;

	struct ap_apb_reg_bak ap_apb_reg_saved;
	struct ap_ahb_reg_bak ap_ahb_reg_saved;

	int ap_clk_cnt;
	int sleep_cnt;
} sprd_deep;

static int sprd_deep_get_ap_clk(void)
{
	int i, count = 0;
	struct ap_deepsleep_node *ds = &sprd_deep;
	struct deepsleep_ap_clk *ap_clk;

	if (IS_ERR(ds->np)) {
		pr_err("%s, deepsleep_np is not initialized\n", __func__);
		return -ENODEV;
	}

	if (!of_get_property(ds->np, "clocks", &count)) {
		pr_info("%s, get clocks property fail !\n", __func__);
		return -EINVAL;
	}

	ds->ap_clk_cnt = count >> 2;
	if (!ds->ap_clk_cnt) {
		pr_err("%s, clk_cnt = 0\n", __func__);
		return -EINVAL;
	}

	pr_info("%s, get %d ap_clk\n", __func__, ds->ap_clk_cnt);

	ap_clk = kcalloc(ds->ap_clk_cnt,
		sizeof(struct deepsleep_ap_clk), GFP_KERNEL);
	if (!ap_clk)
		return -ENOMEM;

	ds->ap_clk = ap_clk;

	for (i = 0; i < ds->ap_clk_cnt; i++) {
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
	struct ap_deepsleep_node *ds = &sprd_deep;
	struct deepsleep_ap_clk *ap_clk = ds->ap_clk;

	pr_info("start %s\n", __func__);

	for (i = 1; i < ds->ap_clk_cnt; i++) {
		ap_clk[i].parent = clk_get_parent(ap_clk[i].clk);
		ap_clk[i].current_rate = clk_get_rate(ap_clk[i].clk);
		pr_info("%s,bak clock index = %d\n", __func__, i);
	}

	pr_info("end %s\n", __func__);
}

static void sprd_deep_restore_ap_clk(void)
{
	int i;
	struct ap_deepsleep_node *ds = &sprd_deep;
	struct deepsleep_ap_clk *ap_clk = ds->ap_clk;

	pr_info("start %s\n", __func__);

	for (i = 1; i < ds->ap_clk_cnt; i++) {
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

static void sprd_deep_ap_ahb_cfg(void)
{
	/*AP_PERI_FORCE_ON CLR*/
	regmap_update_bits(sprd_deep.ap_ahb, REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG,
		BIT_AP_AHB_AP_PERI_FORCE_ON,
		(unsigned int)(~BIT_AP_AHB_AP_PERI_FORCE_ON));

	/*SET AP_PERI_FORCE_ON*/
	regmap_update_bits(sprd_deep.ap_ahb, REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG,
		BIT_AP_AHB_AP_PERI_FORCE_SLP, BIT_AP_AHB_AP_PERI_FORCE_SLP);
}

static void sprd_deep_ap_ahb_check(void)
{
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_DMA_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_OTG_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_CE_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_SDIO0_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_SDIO1_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_SDIO2_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_NANDC_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_EMMC_EB);
}

static void sprd_deep_ap_apb_check(void)
{
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_SIM0_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_SIM0_32K_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_IIS0_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_IIS1_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_IIS2_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_SPI0_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_SPI1_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_SPI2_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_SPI3_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_I2C0_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_I2C1_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_I2C2_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_I2C3_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_I2C4_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_I2C5_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_I2C6_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_UART0_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_UART1_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_UART2_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_UART3_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_UART4_EB);
}

static void sprd_deep_aon_apb_check(void)
{
	/*if need*/
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
	PRINTK_REG(AP_AHB, REG_AP_AHB_AHB_EB);
	PRINTK_REG(AP_AHB, REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG);
	PRINTK_REG(AP_AHB, REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG);
	PRINTK_REG(AP_APB, REG_AP_APB_APB_EB);
	PRINTK_REG(AON_APB, REG_AON_APB_APB_EB0);
	PRINTK_REG(AON_APB, REG_AON_APB_APB_EB1);
	PRINTK_REG(AON_APB, REG_AON_APB_PWR_CTRL);
	PRINTK_REG(AON_APB, REG_AON_APB_APB_EB2);
	PRINTK_REG(PMU_APB, REG_PMU_APB_PWR_STATUS0_DBG);
	PRINTK_REG(PMU_APB, REG_PMU_APB_PWR_STATUS1_DBG);
	PRINTK_REG(PMU_APB, REG_PMU_APB_PWR_STATUS2_DBG);
	PRINTK_REG(PMU_APB, REG_PMU_APB_PWR_STATUS3_DBG);
	PRINTK_REG(PMU_APB, REG_PMU_APB_PWR_STATUS4_DBG);
	PRINTK_REG(PMU_APB, REG_PMU_APB_SLEEP_CTRL);/*0x402B00CC*/
	PRINTK_REG(PMU_APB, REG_PMU_APB_DDR_SLEEP_CTRL);
	PRINTK_REG(PMU_APB, REG_PMU_APB_SLEEP_STATUS);
	PRINTK_REG(PMU_APB, REG_PMU_APB_WTLCP_TGDSP_DSLP_ENA);
	PRINTK_REG(PMU_APB, REG_PMU_APB_WTLCP_LDSP_DSLP_ENA);
	PRINTK_REG(PMU_APB, REG_PMU_APB_AP_DSLP_ENA);
	PRINTK_REG(PMU_APB, REG_PMU_APB_PUBCP_DSLP_ENA);
	PRINTK_REG(PMU_APB, REG_PMU_APB_WTLCP_DSLP_ENA);
	PRINTK_REG(PMU_APB, REG_PMU_APB_WCN_SYS_DSLP_ENA);
	PRINTK_REG(PMU_APB, REG_PMU_APB_WIFI_WRAP_DSLP_ENA);
	PRINTK_REG(PMU_APB, REG_PMU_APB_GNSS_WRAP_DSLP_ENA);
	pr_info("##--light sleep counter.--##\n");
	PRINTK_REG(PMU_APB, REG_PMU_APB_SLEEP_CNT0);
	pr_info("##--deep sleep counter.--##\n");
	PRINTK_REG(PMU_APB, REG_PMU_APB_SLEEP_CNT3);
	pr_info("##--doze sleep counter.--##\n");
	PRINTK_REG(PMU_APB, REG_PMU_APB_SLEEP_CNT6);
}

static void sprd_deep_pwr_status0(void)
{
	uint32_t pwr_status0;

	regmap_read(sprd_deep.pmu_apb,
		REG_PMU_APB_PWR_STATUS0_DBG,
		&pwr_status0);

	PRINTK_REG(PMU_APB, REG_PMU_APB_PWR_STATUS0_DBG);
	pr_info("##--status of power domain 'PD_CPU_TOP_STATE' :0x%x\n",
		(pwr_status0)&0x1F);
	pr_info("##--status of power domain 'PD_CA55_LIT_MP2_STATE' :0x%x\n",
		((pwr_status0)&(0x1F<<5))>>5);
	pr_info("##--status of power domain 'PD_CA55_BIG_MP2_STATE' :0x%x\n",
		((pwr_status0)&(0x1F<<10))>>10);
	pr_info("##--status of power domain 'PD_AP_SYS_STATE' :0x%x\n",
		((pwr_status0)&(0x1F<<17))>>17);
	pr_info("##--status of power domain 'PD_GPU_TOP_STATE' :0x%x\n",
		((pwr_status0)&(0x1F<<22))>>22);
	pr_info("##--status of power domain 'PD_MM_TOP_STATE' :0x%x\n",
		((pwr_status0)&(0x1F<<27))>>27);

}

static void sprd_deep_pwr_status1(void)
{
	uint32_t pwr_status1;

	regmap_read(sprd_deep.pmu_apb,
		REG_PMU_APB_PWR_STATUS1_DBG,
		&pwr_status1);

	PRINTK_REG(PMU_APB, REG_PMU_APB_PWR_STATUS1_DBG);
	pr_info("##--status of power domain 'PD_WTLCP_HU3GE_B_STATE' :0x%08x\n",
		(pwr_status1)&0x1F);
	pr_info("##--status of power domain 'PD_WTLCP_HU3GE_A_STATE' :0x%08x\n",
		((pwr_status1)&(0x1F<<5))>>5);
	pr_info("##--status of power domain 'PD_WTLCP_TGDSP_STATE' :0x%08x\n",
		((pwr_status1)&(0x1F<<10))>>10);
	pr_info("##--status of power domain 'PD_WTLCP_LDSP_STATE' :0x%08x\n",
		((pwr_status1)&(0x1F<<15))>>15);
	pr_info("##--status of power domain 'PD_WTLCP_LTE_P2_STATE' :0x%08x\n",
		((pwr_status1)&(0x1F<<20))>>20);
	pr_info("##--status of power domain 'PD_WTLCP_LTE_P1_STATE' :0x%08x\n",
		((pwr_status1)&(0x1F<<25))>>25);
}

static void sprd_deep_pwr_status2(void)
{
	uint32_t pwr_status2;

	regmap_read(sprd_deep.pmu_apb,
		REG_PMU_APB_PWR_STATUS2_DBG,
		&pwr_status2);

	PRINTK_REG(PMU_APB, REG_PMU_APB_PWR_STATUS2_DBG);
	pr_info("##--status of power domain 'PD_WTLCP_SYS_STATE' :0x%x\n",
		(pwr_status2)&0x1F);
	pr_info("##--status of power domain 'PD_PUBCP_SYS_STATE' :0x%x\n",
		((pwr_status2)&(0x1F<<5))>>5);
	pr_info("##--status of power domain 'PD_WTLCP_LTE_P3_STATE' :0x%x\n",
		((pwr_status2)&(0x1F<<10))>>10);
	pr_info("##--status of power domain 'PD_DISP_STATE' :0x%x\n",
		((pwr_status2)&(0x1F<<15))>>15);
	pr_info("##--status of power domain 'PD_PUB_SYS_STATE' :0x%x\n",
		((pwr_status2)&(0x1F<<20))>>20);
	pr_info("##--status of power domain 'PD_WTLCP_TD_STATE' :0x%x\n",
		((pwr_status2)&(0x1F<<25))>>25);
}

static void sprd_deep_pwr_status3(void)
{
	uint32_t pwr_status3;

	regmap_read(sprd_deep.pmu_apb,
		REG_PMU_APB_PWR_STATUS3_DBG,
		&pwr_status3);

	PRINTK_REG(PMU_APB, REG_PMU_APB_PWR_STATUS3_DBG);
	pr_info("##--status of power domain 'PD_CPU_LIT_C0_STATE' :0x%x\n",
		(pwr_status3)&0x1F);
	pr_info("##--status of power domain 'PD_CPU_LIT_C1_STATE' :0x%x\n",
		((pwr_status3)&(0x1F<<5))>>5);
	pr_info("##--status of power domain 'PD_CPU_BIG_C0_STATE' :0x%x\n",
		((pwr_status3)&(0x1F<<10))>>10);
	pr_info("##--status of power domain 'PD_CPU_BIG_C1_STATE' :0x%x\n",
		((pwr_status3)&(0x1F<<15))>>15);
}

static void sprd_deep_pwr_status4(void)
{
	uint32_t pwr_status4;

	regmap_read(sprd_deep.pmu_apb,
		REG_PMU_APB_PWR_STATUS4_DBG,
		&pwr_status4);

	PRINTK_REG(PMU_APB, REG_PMU_APB_PWR_STATUS4_DBG);
	pr_info("##--status of power domain 'PD_DBG_SYS_STATE' :0x%x\n",
		(pwr_status4)&0x1F);
	pr_info("##--status of power domain 'PD_CPU_MP8_STATE' :0x%x\n",
		((pwr_status4)&(0x1F<<5))>>5);
	pr_info("##--status of power domain 'PD_WCN_SYS_STATE' :0x%x\n",
		((pwr_status4)&(0x1F<<10))>>10);
	pr_info("##--status of power domain 'PD_WIFI_WRAP_STATE' :0x%x\n",
		((pwr_status4)&(0x1F<<15))>>15);
	pr_info("##--status of power domain 'PD_GNSS_WRAP_STATE' :0x%x\n",
		((pwr_status4)&(0x1F<<20))>>20);
}

static void sprd_deep_pwr_status5(void)
{
	uint32_t pwr_status5;

	regmap_read(sprd_deep.pmu_apb,
		REG_PMU_APB_PWR_STATUS5_DBG,
		&pwr_status5);

	PRINTK_REG(PMU_APB, REG_PMU_APB_PWR_STATUS5_DBG);
	pr_info("##--status of power domain 'PD_MM_VSP_STATE' :0x%x\n",
			(pwr_status5)&0x1F);
	pr_info("##--status of power domain 'PD_GPU_CORE_STATE' :0x%x\n",
			((pwr_status5)&(0x1F<<5))>>5);
	pr_info("##--status of power domain 'PD_CPU_LIT_C2_STATE' :0x%x\n",
		((pwr_status5)&(0x1F<<10))>>10);
	pr_info("##--status of power domain 'PD_CPU_LIT_C3_STATE' :0x%x\n",
		((pwr_status5)&(0x1F<<15))>>15);
	pr_info("##--status of power domain 'PD_CPU_BIG_C2_STATE' :0x%x\n",
		((pwr_status5)&(0x1F<<20))>>20);
	pr_info("##--status of power domain 'PD_CPU_BIG_C3_STATE' :0x%x\n",
		((pwr_status5)&(0x1F<<25))>>25);
}

static void sprd_deep_print_power_domain(void)
{
	pr_info("#####PWR_STATUS POWER DOMAIN#####\n");
	pr_info("#####402B00BC:PWR_STATUS0 POWER DOMAIN#####\n");
	sprd_deep_pwr_status0();
	pr_info("#####402B00C0:PWR_STATUS1 POWER DOMAIN#####\n");
	sprd_deep_pwr_status1();
	pr_info("#####402B00C4:PWR_STATUS2 POWER DOMAIN#####\n");
	sprd_deep_pwr_status2();
	pr_info("#####402B048C:PWR_STATUS3 POWER DOMAIN#####\n");
	sprd_deep_pwr_status3();
	pr_info("#####402B0490:PWR_STATUS4 POWER DOMAIN#####\n");
	sprd_deep_pwr_status4();
	pr_info("#####402B0128:PWR_STATUS5 POWER DOMAIN#####\n");
	sprd_deep_pwr_status5();
}

static void sprd_deep_intc_status(void)
{
	int i;
	uint32_t intc_bits[AP_INTC_NUM], intc_irq_raw[AP_INTC_NUM],
		 intc_irq_en[AP_INTC_NUM];

	for (i = 0; i < AP_INTC_NUM; i++) {
		regmap_read(sprd_deep.intc[i], INTCV_IRQ_MSKSTS,
			&intc_bits[i]);
		regmap_read(sprd_deep.intc[i], INTCV_IRQ_RAW,
			&intc_irq_raw[i]);
		regmap_read(sprd_deep.intc[i], INTCV_IRQ_EN,
			&intc_irq_en[i]);

		pr_info("AP INTC%d: MSKSTS 0x%08x, RAW 0x%08x, EN 0x%08x\n",
				i, intc_bits[i],
				intc_irq_raw[i], intc_irq_en[i]);
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

	for (i = 0; i < AP_INTC_NUM; i++) {
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
			pr_info("wake up by aon for ap\n");
		if (sprd_hard_irq[29])
			pr_info("wake up by aon tmr2\n");
		if (sprd_hard_irq[28])
			pr_info("wake up by aon tmr1\n");
		if (sprd_hard_irq[27])
			pr_info("wake up by aon tmr0\n");
		if (sprd_hard_irq[26])
			pr_info("wake up by thm0\n");
		if (sprd_hard_irq[25])
			pr_info("wake up by adi\n");
		if (sprd_hard_irq[23])
			pr_info("wake up by vbc adc01!\n");
		if (sprd_hard_irq[22])
			pr_info("wake up by vbc dac01\n");
		if (sprd_hard_irq[21])
			pr_info("wake up by vbc_dac23\n");
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
			pr_info("wake up by ap emmc\n");
		if (sprd_hard_irq[56])
			pr_info("wake up by aon sec cnt\n");
		if (sprd_hard_irq[55])
			pr_info("wake up by ap otg\n");
		if (sprd_hard_irq[54])
			pr_info("wake up by ce_pub\n");
		if (sprd_hard_irq[52])
			pr_info("wake up by ap sec dma\n");
		if (sprd_hard_irq[51])
			pr_info("wake up by ap gsp\n");
		if (sprd_hard_irq[50])
			pr_info("wake up by ap dma\n");
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
			pr_info("wake up by pcp_wdg\n");
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
		if (sprd_hard_irq[66])
			pr_info("wake up by ap nandc\n");
	}

	if (sprd_irqs_sts[3]) {
		if (sprd_hard_irq[126])
			pr_info("wake up by cpp\n");
		if (sprd_hard_irq[125])
			pr_info("wake up by isp_ch1\n");
		if (sprd_hard_irq[124])
			pr_info("wake up by ca53_wdg\n");
		if (sprd_hard_irq[123])
			pr_info("wake up by ap_wdg\n");
		if (sprd_hard_irq[121])
			pr_info("wake up by dvfs irq lvl\n");
		if (sprd_hard_irq[119])
			pr_info("wake up by mbox tar ap unwake\n");
	}

	if (sprd_irqs_sts[4]) {
		if (sprd_hard_irq[131])
			pr_info("wake up by gpio plus ap sec\n");
		if (sprd_hard_irq[130])
			pr_info("wake up by gpio plus ap\n");
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
	unsigned int cnt, i;

	struct ap_intc_en deep_intc[] = {
			{0, BIT(27)		/*int_req_aon_tmr0*/
			},
			{1, BIT(3) |	/*int_req_gpio*/
				BIT(4) |	/*int_req_kpd*/
				BIT(5) |	/*int_req_eic*/
				BIT(6) |	/*int_req_ana*/
				BIT(30) |	/*int_req_sec_eic*/
				BIT(31)		/*int_req_sec_gpio*/
			},
			{2, BIT(4) |	/*int_req_mbox_src_ap*/
				BIT(5)		/*int_req_mbox_tar_ap*/
			},
			{3, BIT(28)		/*int_req_ca53_wdg*/
			},
			{4, BIT(2) |	/*int_req_gpio_plus_ap*/
				BIT(3)		/*int_req_gpio_plus_ap_sec*/
			}
	};

	regmap_update_bits(sprd_deep.aon_apb, REG_AON_APB_APB_EB2,
			BIT_AON_APB_AP_INTC0_EB |
			BIT_AON_APB_AP_INTC1_EB |
			BIT_AON_APB_AP_INTC2_EB |
			BIT_AON_APB_AP_INTC3_EB |
			BIT_AON_APB_AP_INTC4_EB |
			BIT_AON_APB_AP_INTC5_EB,
			BIT_AON_APB_AP_INTC0_EB |
			BIT_AON_APB_AP_INTC1_EB |
			BIT_AON_APB_AP_INTC2_EB |
			BIT_AON_APB_AP_INTC3_EB |
			BIT_AON_APB_AP_INTC4_EB |
			BIT_AON_APB_AP_INTC5_EB
	);

	cnt = sizeof(deep_intc)/sizeof(struct ap_intc_en);

	for (i = 0; i < cnt; i++) {
		regmap_update_bits(sprd_deep.intc[i], INTCV_IRQ_EN,
			deep_intc[i].int_channel,
			deep_intc[i].int_channel
		);
	}
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

	for (i = 0; i < AP_INTC_NUM; i++) {
		sprintf(ap_intc_name, "sprd,sys-ap-intc%d", i);
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
		sprd_deep_ap_ahb_cfg();
		sprd_deep_sleep_status();
		sprd_deep_intc_status();
		break;
	case CPU_CLUSTER_PM_ENTER_FAILED:
	case CPU_CLUSTER_PM_EXIT:
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

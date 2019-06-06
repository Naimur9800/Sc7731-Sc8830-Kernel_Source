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
#define   SPRD_HARD_INT_NUM_EACH_INTC	32
#define SPRD_HARD_INTERRUPT_NUM	192


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
		sprd_deep_bak_restore_ap_intc(1);\
		} while (0) \

#define RESTORE_GLOBAL_REG() do { \
		sprd_deep_bak_restore_apb(0); \
		sprd_deep_bak_ap_clk_reg(0); \
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

struct ap_ahb_reg_bak {
	u32 ahb_eb;
	u32 ca7_ckg_div_cfg;
	u32 misc_ckg_en;
	u32 ca7_ckg_sel_cfg;
	u32 ap_sys_force_sleep_cfg;
	u32 ap_sys_auto_sleep_cfg;
	u32 otg_phy_tune;
	u32 otg_phy_test;
	u32 otg_phy_ctrl;
	u32 otg_ctrl0;
	u32 otg_ctrl1;
	u32 ca7_top_m0_lpc;
	u32 ca7_abrg_s0_lpc;
	u32 ca7_top_s1_lpc;
	u32 ca7_top_s2_lpc;
	u32 ca7_emc_reg_slice_lpc;
	u32 s_sync_lpc;
	u32 m0_lpc;
	u32 m_sync_lpc;
	u32 usb_ahbm2axi_s0_lpc;
	u32 m9_lpc;
	u32 s0_lpc;
	u32 s1_lpc;
	u32 ap_imc_main_lpc;
	u32 ap_gsp_m0_lpc;
	u32 ap_gsp_m1_lpc;
	u32 ap_gsp_gpv_lpc;
	u32 ap_gsp_s0_lpc;
	u32 ap_disp_main_lpc;
};
struct ap_apb_reg_bak {
	u32 apb_eb;
	u32 apb_misc_ctrl;
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
		REG_AP_AHB_CA7_TOP_M0_LPC,
		REG_AP_AHB_CA7_ABRG_S0_LPC,
		REG_AP_AHB_CA7_TOP_S1_LPC,
		REG_AP_AHB_CA7_TOP_S2_LPC,
		REG_AP_AHB_CA7_EMC_REG_SLICE_LPC,
		REG_AP_AHB_S_SYNC_LPC,
		REG_AP_AHB_M0_LPC,
		REG_AP_AHB_M_SYNC_LPC,
		REG_AP_AHB_USB_AHBM2AXI_S0_LPC,
		REG_AP_AHB_M9_LPC,
		REG_AP_AHB_S0_LPC,
		REG_AP_AHB_S1_LPC,
		REG_AP_AHB_AP_IMC_MAIN_LPC,
		REG_AP_AHB_AP_GSP_M0_LPC,
		REG_AP_AHB_AP_GSP_M1_LPC,
		REG_AP_AHB_AP_GSP_GPV_LPC,
		REG_AP_AHB_AP_GSP_S0_LPC,
		REG_AP_AHB_AP_DISP_MAIN_LPC
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

static void sprd_deep_disable_ap_apb(void)
{
	/*if need*/
}

static void sprd_deep_disable_ap_ahb(void)
{
	/*AP_PERI_FORCE_ON CLR*/
	regmap_update_bits(sprd_deep.syscon_map[AP_AHB],
			REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG,
			BIT_AP_AHB_AP_PERI_FORCE_ON,
			(unsigned int)(~BIT_AP_AHB_AP_PERI_FORCE_ON));

	/*SET AP_PERI_FORCE_SLP*/
	regmap_update_bits(sprd_deep.syscon_map[AP_AHB],
				REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG,
				BIT_AP_AHB_AP_PERI_FORCE_SLP,
				BIT_AP_AHB_AP_PERI_FORCE_SLP);
}

static void sprd_deep_disable_module(void)
{
	sprd_deep_disable_ap_apb();
	sprd_deep_disable_ap_ahb();
}

void sprd_deep_pwr_domain_status0(struct sprd_deep *ds)
{
	uint32_t pmu_pwr_status0;

	regmap_read(ds->syscon_map[PMU_APB], REG_PMU_APB_PWR_STATUS0_DBG,
			&pmu_pwr_status0);

	pr_info("##--status of power domain 'MM_TOP' :0x%08x\n",
		(pmu_pwr_status0 & BIT_PMU_APB_PD_MM_TOP_STATE(0xF)) >> 28);
	pr_info("##--status of power domain 'GPU_TOP' :0x%08x\n",
		(pmu_pwr_status0 & BIT_PMU_APB_PD_GPU_TOP_STATE(0xF)) >> 24);
	pr_info("##--status of power domain 'AP_SYS' :0x%08x\n",
		(pmu_pwr_status0 & BIT_PMU_APB_PD_AP_SYS_STATE(0xF)) >> 20);
	pr_info("##--status of power domain 'CA7_C3' :0x%08x\n",
		(pmu_pwr_status0 & BIT_PMU_APB_PD_CA7_C3_STATE(0xF)) >> 16);
	pr_info("##--status of power domain 'CA7_C2' :0x%08x\n",
		(pmu_pwr_status0 & BIT_PMU_APB_PD_CA7_C2_STATE(0xF)) >> 12);
	pr_info("##--status of power domain 'CA7_C1' :0x%08x\n",
		(pmu_pwr_status0 & BIT_PMU_APB_PD_CA7_C1_STATE(0xF)) >> 8);
	pr_info("##--status of power domain 'CA7_C0' :0x%08x\n",
		(pmu_pwr_status0 & BIT_PMU_APB_PD_CA7_C0_STATE(0xF)) >> 4);
	pr_info("##--status of power domain 'CA7_TOP' :0x%08x\n",
		pmu_pwr_status0 & BIT_PMU_APB_PD_CA7_TOP_STATE(0xF));
}

void sprd_deep_pwr_domain_status1(struct sprd_deep *ds)
{
	uint32_t pmu_pwr_status1;

	regmap_read(ds->syscon_map[PMU_APB], REG_PMU_APB_PWR_STATUS1_DBG,
			&pmu_pwr_status1);

	pr_info("##--status of power domain 'WCN_GNSS' :0x%08x\n",
		(pmu_pwr_status1 & BIT_PMU_APB_PD_WCN_GNSS_STATE(0xF)) >> 24);
	pr_info("##--status of power domain 'WCN_WIFI' :0x%08x\n",
		(pmu_pwr_status1 & BIT_PMU_APB_PD_WCN_WIFI_STATE(0xF)) >> 20);
	pr_info("##--status of power domain 'WCN_TOP' :0x%08x\n",
		(pmu_pwr_status1 & BIT_PMU_APB_PD_WCN_TOP_STATE(0xF)) >> 16);
	pr_info("##--status of power domain 'PUB_SYS' :0x%08x\n",
		(pmu_pwr_status1 & BIT_PMU_APB_PD_PUB_SYS_STATE(0xF)) >> 12);
	pr_info("##--status of power domain 'WTLCP_TGDSP' :0x%08x\n",
	(pmu_pwr_status1 & BIT_PMU_APB_PD_WTLCP_TGDSP_STATE(0xF)) >> 8);
	pr_info("##--status of power domain 'HU3GE_A' :0x%08x\n",
	(pmu_pwr_status1 & BIT_PMU_APB_PD_WTLCP_HU3GE_A_STATE(0xF)) >> 4);
	pr_info("##--status of power domain 'CP_SYS' :0x%08x\n",
		pmu_pwr_status1 & BIT_PMU_APB_PD_CP_SYS_STATE(0xF));
}

void sprd_deep_sleep_status2(struct sprd_deep *ds)
{
	uint32_t pmu_sleep_status;

	regmap_read(ds->syscon_map[PMU_APB], REG_PMU_APB_SLEEP_STATUS,
			&pmu_sleep_status);

	pr_info("##--status of sleep 'CM4_SLP' :0x%08x\n",
		(pmu_sleep_status & BIT_PMU_APB_CM4_SLP_STATUS(0xF)) >> 20);
	pr_info("##--status of sleep 'WCN_SLP' :0x%08x\n",
		(pmu_sleep_status & BIT_PMU_APB_WCN_SLP_STATUS(0xF)) >> 12);
	pr_info("##--status of sleep 'CP_SLP' :0x%08x\n",
		(pmu_sleep_status & BIT_PMU_APB_CP_SLP_STATUS(0xF)) >> 8);
	pr_info("##--status of sleep 'AP_SLP' :0x%08x\n",
		(pmu_sleep_status & BIT_PMU_APB_AP_SLP_STATUS(0xF)));
}

void sprd_sleep_and_pwr_status(struct sprd_deep *ds)
{
	pr_info("#####PWR_STATUS0#####\n");
	sprd_deep_pwr_domain_status0(ds);
	pr_info("#####PWR_STATUS1#####\n");
	sprd_deep_pwr_domain_status1(ds);
	pr_info("#####SLEEP_STATUS#####\n");
	sprd_deep_sleep_status2(ds);
}

void sprd_deep_print_last_reg(struct sprd_deep *ds)
{
	pr_info("ap ahb reg\n");
	PRINTK_REG(AP_AHB, REG_AP_AHB_AHB_EB);
	PRINTK_REG(AP_AHB, REG_AP_AHB_MCU_PAUSE);
	PRINTK_REG(AP_AHB, REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG);
	PRINTK_REG(AP_AHB, REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG);
	PRINTK_REG(AP_AHB, REG_AP_AHB_CA7_STANDBY_STATUS);
	PRINTK_REG(AP_AHB, REG_AP_AHB_CA7_TOP_M0_LPC);
	PRINTK_REG(AP_AHB, REG_AP_AHB_CA7_ABRG_S0_LPC);
	PRINTK_REG(AP_AHB, REG_AP_AHB_CA7_TOP_S1_LPC);
	PRINTK_REG(AP_AHB, REG_AP_AHB_CA7_TOP_S2_LPC);
	PRINTK_REG(AP_AHB, REG_AP_AHB_CA7_EMC_REG_SLICE_LPC);
	PRINTK_REG(AP_AHB, REG_AP_AHB_S_SYNC_LPC);
	PRINTK_REG(AP_AHB, REG_AP_AHB_M0_LPC);
	PRINTK_REG(AP_AHB, REG_AP_AHB_M_SYNC_LPC);
	PRINTK_REG(AP_AHB, REG_AP_AHB_USB_AHBM2AXI_S0_LPC);
	PRINTK_REG(AP_AHB, REG_AP_AHB_M9_LPC);
	PRINTK_REG(AP_AHB, REG_AP_AHB_S0_LPC);
	PRINTK_REG(AP_AHB, REG_AP_AHB_S1_LPC);
	PRINTK_REG(AP_AHB, REG_AP_AHB_AP_IMC_MAIN_LPC);
	PRINTK_REG(AP_AHB, REG_AP_AHB_AP_GSP_M0_LPC);
	PRINTK_REG(AP_AHB, REG_AP_AHB_AP_GSP_M1_LPC);
	PRINTK_REG(AP_AHB, REG_AP_AHB_AP_GSP_GPV_LPC);
	PRINTK_REG(AP_AHB, REG_AP_AHB_AP_GSP_S0_LPC);
	PRINTK_REG(AP_AHB, REG_AP_AHB_AP_DISP_MAIN_LPC);
	pr_info("ap apb reg\n");
	PRINTK_REG(AP_APB, REG_AP_APB_APB_EB);
	pr_info("aon apb reg\n");
	PRINTK_REG(AON_APB, REG_AON_APB_APB_EB0);
	PRINTK_REG(AON_APB, REG_AON_APB_APB_EB1);
	PRINTK_REG(AON_APB, REG_AON_APB_APB_EB2);
	PRINTK_REG(AON_APB, REG_AON_APB_PWR_CTRL);
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
	sprd_sleep_and_pwr_status(ds);
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
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_REE_DMA_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_SDIO0_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_NANDC_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_EMMC_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_OTG_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_CE_SEC_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_CE_PUB_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_GSP_EB);
	BITS_CHECK(AP_AHB, REG_AP_AHB_AHB_EB, BIT_AP_AHB_DISPC_EB);
}

void sprd_deep_ap_apb_check(struct sprd_deep *ds)
{
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_UART0_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_UART1_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_I2C0_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_I2C1_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_I2C2_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_IIS0_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_SPI0_EB);
	BITS_CHECK(AP_APB, REG_AP_APB_APB_EB, BIT_AP_APB_SIM0_EB);
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

		if (i < 4 && intc_bits[i]) {
			pr_info("wakeup by INTC%d!!mask:0x%08x raw:0x%08x en:0x%08x\n",
		i, intc_bits[i], intc_irq_raw[i], intc_irq_en[i]);

		/*debug wakeup by cp tar ap*/
#ifdef CONFIG_SPRD_SIPC
			if (i == 2 && (intc_bits[i] & 0x20)) {
				sipc_set_wakeup_flag();
			}
#endif
			parse_hard_irq(intc_bits[i], i);

		}
	}
	pr_info("AON INTC mask:0x%08x raw:0x%08x en:0x%08x\n",
		intc_bits[4], intc_irq_raw[4], intc_irq_en[4]);
}

static int sprd_deep_intc_vaddr_init(struct sprd_deep *ds)
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

static int sprd_deep_glb_vaddr_init(struct sprd_deep *ds)
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
		sprd_deep_disable_module();
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

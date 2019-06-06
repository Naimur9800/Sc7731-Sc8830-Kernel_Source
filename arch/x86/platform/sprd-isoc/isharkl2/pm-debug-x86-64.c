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
#include <linux/syscore_ops.h>
#ifdef CONFIG_SPRD_SIPC
#include <linux/sipc.h>
#endif
/*#include <soc/sprd/gpio.h>*/

/* used as a debug */
#include <linux/sipc.h>
#include "pm_debug_x86_64.h"
#include "pm_x86_64.h"

static struct dentry *dentry_debug_root;
static u32 is_print_sleep_mode;
static u32 is_print_linux_clock = 1;
static u32 is_print_modem_clock = 1;
static u32 is_print_irq = 1;
static u32 is_print_wakeup = 1;
static u32 is_print_irq_runtime;
static u32 is_print_time;
static u32 print_thread_enable = 1;
static u32 print_thread_interval = 30;


/* static u32 sprd_hard_irq[SPRD_HARD_INTERRUPT_NUM] = {0, }; */
static u32 sprd_irqs_sts[SPRD_INTC_NUM] = {0, };

static void __iomem *deep_bia_intc;
/* static int is_wakeup; */
/* static int irq_status; */

static u32 intc_raw_status[SPRD_INTC_CONTROLLER_NUM] = {0, };
static u32 sprd_hard_irq[SPRD_HARD_INTERRUPT_NUM] = {0, };
void hard_irq_reset(void)
{
	memset(sprd_hard_irq, 0, sizeof(u32)*SPRD_HARD_INTERRUPT_NUM);
}

static void parse_hard_irq(void)
{
	int i, j;

	for (i = 0; i < SPRD_INTC_CONTROLLER_NUM; i++) {
		for (j = 0; j < SPRD_INT_NUM_EACH_INTC; j++) {
			if (BIT(j) & intc_raw_status[i])
				sprd_hard_irq[i*SPRD_INT_NUM_EACH_INTC+j]++;
		}
	}
}

void get_hard_irq(void)
{
	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,INTC register base vaddr is not ok!\n", __func__);
		return;
	}

	regmap_read(deepsleep_syscon_aon_intc0, INTCV0_IRQ_RAW,
				&intc_raw_status[0]);
	regmap_read(deepsleep_syscon_aon_intc1, INTCV1_IRQ_RAW,
				&intc_raw_status[1]);
	regmap_read(deepsleep_syscon_aon_intc2, INTCV2_IRQ_RAW,
				&intc_raw_status[2]);
	regmap_read(deepsleep_syscon_aon_intc3, INTCV3_IRQ_RAW,
				&intc_raw_status[3]);

	parse_hard_irq();
}


void print_hard_irq(void)
{
	pr_info("intc_raw_status:\n0 :%08x\n1 :%08x\n2 :%08x\n3 :%08x\n",
		intc_raw_status[0], intc_raw_status[1],
		intc_raw_status[2], intc_raw_status[3]);

	if (sprd_hard_irq[3])
		pr_info("wake up by ap_uart1\n");
	if (sprd_hard_irq[7])
		pr_info("wake up by ap_spi0\n");
	if (sprd_hard_irq[21])
		pr_info("wake up by cam_sys_busmon\n");
	if (sprd_hard_irq[22])
		pr_info("wake up by vsp_sys_busmon\n");
	if (sprd_hard_irq[23])
		pr_info("wake up by dma_sec_ap\n");
	if (sprd_hard_irq[26])
		pr_info("wake up by aon_tmr\n");
	if (sprd_hard_irq[28])
		pr_info("wake up by mbox_src_a53\n");
	if (sprd_hard_irq[29]) {
#ifdef CONFIG_SPRD_SIPC
		sipc_set_wakeup_flag();
#endif
		pr_info("wake up by mbox_tar_a53\n");
	}
	if (sprd_hard_irq[31])
		pr_info("wake up by ana\n");
	if (sprd_hard_irq[37])
		pr_info("wake up by dcam2isp_if\n");
	if (sprd_hard_irq[42])
		pr_info("wake up by ap_dma\n");
	if (sprd_hard_irq[43])
		pr_info("wake up by ap_busmon0\n");
	if (sprd_hard_irq[50])
		pr_info("wake up by gpio\n");
	if (sprd_hard_irq[52])
		pr_info("wake up by eic\n");
	if (sprd_hard_irq[62])
		pr_info("wake up by djtag\n");
	if (sprd_hard_irq[65])
		pr_info("wake up by sec_eic\n");
	if (sprd_hard_irq[68])
		pr_info("wake up by vsp\n");
	if (sprd_hard_irq[71])
		pr_info("wake up by isp_ch0\n");
	if (sprd_hard_irq[96])
		pr_info("wake up by sec_gpio\n");
	if (sprd_hard_irq[116])
		pr_info("wake up by pub0_dfs_error\n");
	if (sprd_hard_irq[118])
		pr_info("wake up by bia_tmr\n");
	if (sprd_hard_irq[122])
		pr_info("wake up by 32k_det\n");
}


void print_bia_irq_status(void)
{
	if (IS_ERR(deep_bia_intc))
		return;

	sprd_irqs_sts[0] = ioread32(deep_bia_intc + LINE_INT_STATUS0);
	sprd_irqs_sts[1] = ioread32(deep_bia_intc + LINE_INT_STATUS1);
	sprd_irqs_sts[2] = ioread32(deep_bia_intc + LINE_INT_STATUS2);
	sprd_irqs_sts[3] = ioread32(deep_bia_intc + LINE_INT_STATUS3);
	sprd_irqs_sts[4] = ioread32(deep_bia_intc + LINE_INT_STATUS4);
	sprd_irqs_sts[5] = ioread32(deep_bia_intc + LINE_INT_STATUS5);
	sprd_irqs_sts[6] = ioread32(deep_bia_intc + LINE_INT_STATUS6);
	sprd_irqs_sts[7] = ioread32(deep_bia_intc + LINE_INT_STATUS7);

	pr_info("#####wakeup######\n");
	pr_info("intc status:\n0 :%08x\n1 :%08x\n2 :%08x\n3 :%08x\n",
		sprd_irqs_sts[0], sprd_irqs_sts[1],
		sprd_irqs_sts[2], sprd_irqs_sts[3]);
	pr_info("4 :%08x\n5 :%08x\n6 :%08x\n7 :%08x\n",
		sprd_irqs_sts[4], sprd_irqs_sts[5],
		sprd_irqs_sts[6], sprd_irqs_sts[7]);
}

void show_pin_reg(void)
{
	uint32_t xtl0_rel_cfg, xtlbuf0_rel_cfg, xtlbuf1_rel_cfg,
		xtlbuf2_rel_cfg;

	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,INTC and SYSCON register base vaddr is not ok!\n",
			__func__);
		return;
	}

	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_XTL0_REL_CFG,
				&xtl0_rel_cfg);
	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_XTLBUF0_REL_CFG,
				&xtlbuf0_rel_cfg);
	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_XTLBUF1_REL_CFG,
				&xtlbuf1_rel_cfg);
	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_XTLBUF2_REL_CFG,
				&xtlbuf2_rel_cfg);

	pr_info("#######PIN REG#######\n");
	pr_info("##--REG_PMU_APB_XTL0_REL_CFG : 0x%08x\n", xtl0_rel_cfg);
	pr_info("##--REG_PMU_APB_XTLBUF0_REL_CFG : 0x%08x\n", xtlbuf0_rel_cfg);
	pr_info("##--REG_PMU_APB_XTLBUF1_REL_CFG : 0x%08x\n", xtlbuf1_rel_cfg);
	pr_info("##--REG_PMU_APB_XTLBUF2_REL_CFG : 0x%08x\n", xtlbuf2_rel_cfg);
}


void deepsleep_condition_check(void)
{
	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}

	pr_info("#####Enter Deepsleep  Condition Check!######\n");
	pr_info("#####Enter AP_AHB  Condition Check!#####\n");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_DAP_EB,
				"dap_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_CE_EFS_EB,
				"ce_efs_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_CE1_EB,
				"ce1_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_CE0_EB,
				"ce0_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_BIA_LP_EB,
				"bia_lp_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_EMMC_EB,
				"emmc_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_SDIO2_EB,
				"sdio2_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_SDIO1_EB,
				"sdio1_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_SDIO0_EB,
				"sdio0_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_DMA_EB,
				"dma_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_AP_AHB_USB2_EB,
				"usb2_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_ASYNC_BRG_LPC,
				BIT_AP_AHB_ASYNC_BRG_LP_EB,
				"async_brg_lp_eb");

	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_M0_LPC, BIT_AP_AHB_M0_LP_EB,
				"m0_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_M1_LPC, BIT_AP_AHB_M1_LP_EB,
				"m1_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_M2_LPC, BIT_AP_AHB_M2_LP_EB,
				"m2_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_M3_LPC, BIT_AP_AHB_M3_LP_EB,
				"m3_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_M4_LPC, BIT_AP_AHB_M4_LP_EB,
				"m4_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_M5_LPC, BIT_AP_AHB_M5_LP_EB,
				"m5_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_M6_LPC, BIT_AP_AHB_M6_LP_EB,
				"m6_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_M7_LPC, BIT_AP_AHB_M7_LP_EB,
				"m7_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_M8_LPC, BIT_AP_AHB_M8_LP_EB,
				"m8_eb");
	AP_AHB_BITS_CLEAR_CHECK(REG_AP_AHB_M9_LPC, BIT_AP_AHB_M9_LP_EB,
				"m9_eb");

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
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_SPI0_EB,
				"spi0_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_SPI1_EB,
				"spi1_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_SPI2_EB,
				"spi2_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_SPI3_EB,
				"spi3_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_I2C0_EB,
				"i2c0_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_I2C1_EB,
				"i2c1_eb");
	AP_APB_BITS_CLEAR_CHECK(REG_AP_APB_APB_EB, BIT_AP_APB_I2C2_EB,
				"i2c2_eb");
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
	AON_APB_BITS_CLEAR_CHECK(REG_AON_APB_APB_EB0, BIT_AON_APB_AON_GPU_EB,
				"gpu_eb");
	AON_APB_BITS_CLEAR_CHECK(REG_AON_APB_APB_EB1, BIT_AON_APB_AON_DISP_EB,
				"dispc_eb");
	AON_APB_BITS_CLEAR_CHECK(REG_AON_APB_APB_EB1, BIT_AON_APB_AON_VSP_EB,
				"vsp_eb");
	AON_APB_BITS_CLEAR_CHECK(REG_AON_APB_APB_EB1, BIT_AON_APB_AON_CAM_EB,
				"cam_eb");
	pr_info("#####Deepsleep  Condition Check End #####\n");
}


static uint32_t pub0_sys_sleep_ctrl, ap_sys_sleep_ctrl,
		cm4_sys_sleep_ctrl, wtlcp_sys_sleep_ctrl,
		pubcp_sys_sleep_ctrl;

static void read_subsys_ctrl(void)
{
	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}
	regmap_read(deep_sys_aon_pwu_apb,
		REG_PMU_APB_PUB0_SYS_SLEEP_CTRL,
		&pub0_sys_sleep_ctrl);
	regmap_read(deep_sys_aon_pwu_apb,
		REG_PMU_APB_AP_SYS_SLEEP_CTRL,
		&ap_sys_sleep_ctrl);
	regmap_read(deep_sys_aon_pwu_apb,
		REG_PMU_APB_CM4_SYS_SLEEP_CTRL,
		&cm4_sys_sleep_ctrl);
	regmap_read(deep_sys_aon_pwu_apb,
		REG_PMU_APB_WTLCP_SYS_SLEEP_CTRL,
		&wtlcp_sys_sleep_ctrl);
	regmap_read(deep_sys_aon_pwu_apb,
		REG_PMU_APB_PUBCP_SYS_SLEEP_CTRL,
		&pubcp_sys_sleep_ctrl);

}


static uint32_t pd_top_pwr_cfg, pd_aon_sys_pwr_cfg,
		pd_pubcp_sys_pwr_cfg,
		pd_wtlcp_lte_p4_pwr_cfg, pd_wtlcp_tgdsp_pwr_cfg,
		pd_wtlcp_hu3ge_b_pwr_cfg, pd_ap_sys_pwr_cfg,
		pd_gpu_phantom_pwr_cfg1, pd_pub0_sys_pwr_cfg,
		pd_wtlcp_lte_p1_pwr_cfg, pd_vsp_sys_pwr_cfg,
		pd_wtlcp_ldsp_pwr_cfg, pd_gpu_top_pwr_cfg1,
		pd_wtlcp_td_pwr_cfg, pd_wtlcp_lte_p3_pwr_cfg,
		pd_wtlcp_sys_pwr_cfg, pd_isp_top_pwr_cfg,
		pd_wtlcp_lte_p2_pwr_cfg, pd_vsp_core_pwr_cfg,
		pd_cam_sys_pwr_cfg, pd_wtlcp_hu3ge_a_pwr_cfg;

static void read_subsys_pwr_cfg(void)
{
	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}
	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_PD_TOP_PWR_CFG,
			&pd_top_pwr_cfg);
	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_PD_AON_SYS_PWR_CFG,
			&pd_aon_sys_pwr_cfg);
	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_PD_PUBCP_SYS_PWR_CFG,
			&pd_pubcp_sys_pwr_cfg);
	regmap_read(deep_sys_ap_ahb, REG_PMU_APB_PD_WTLCP_LTE_P4_PWR_CFG,
			&pd_wtlcp_lte_p4_pwr_cfg);
	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_PD_WTLCP_TGDSP_PWR_CFG,
			&pd_wtlcp_tgdsp_pwr_cfg);
	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_PD_WTLCP_HU3GE_B_PWR_CFG,
			&pd_wtlcp_hu3ge_b_pwr_cfg);
	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_PD_AP_SYS_PWR_CFG,
			&pd_ap_sys_pwr_cfg);
	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_PD_GPU_PHANTOM_PWR_CFG1,
			&pd_gpu_phantom_pwr_cfg1);
	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_PD_PUB0_SYS_PWR_CFG,
			&pd_pub0_sys_pwr_cfg);
	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_PD_WTLCP_LTE_P1_PWR_CFG,
			&pd_wtlcp_lte_p1_pwr_cfg);
	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_PD_VSP_SYS_PWR_CFG,
			&pd_vsp_sys_pwr_cfg);
	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_PD_WTLCP_LDSP_PWR_CFG,
			&pd_wtlcp_ldsp_pwr_cfg);
	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_PD_GPU_TOP_PWR_CFG1,
			&pd_gpu_top_pwr_cfg1);
	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_PD_WTLCP_LTE_P3_PWR_CFG,
			&pd_wtlcp_lte_p3_pwr_cfg);
	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_PD_WTLCP_TD_PWR_CFG,
			&pd_wtlcp_td_pwr_cfg);
	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_PD_WTLCP_SYS_PWR_CFG,
			&pd_wtlcp_sys_pwr_cfg);
	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_PD_ISP_TOP_PWR_CFG,
			&pd_isp_top_pwr_cfg);
	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_PD_WTLCP_LTE_P2_PWR_CFG,
			&pd_wtlcp_lte_p2_pwr_cfg);
	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_PD_VSP_CORE_PWR_CFG,
			&pd_vsp_core_pwr_cfg);
	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_PD_CAM_SYS_PWR_CFG,
			&pd_cam_sys_pwr_cfg);
	regmap_read(deep_sys_aon_pwu_apb, REG_PMU_APB_PD_WTLCP_HU3GE_A_PWR_CFG,
			&pd_wtlcp_hu3ge_a_pwr_cfg);
}

static uint32_t ahb_eb, ap_sys_auto_sleep_cfg,
		sys_force_sleep;
static void read_ap_ahb_reg(void)
{
	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}
	regmap_read(deep_sys_ap_ahb,
		REG_AP_AHB_AHB_EB,
		&ahb_eb);
	regmap_read(deep_sys_ap_ahb,
		REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG,
		&ap_sys_auto_sleep_cfg);
	regmap_read(deep_sys_ap_ahb,
		REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG,
		&sys_force_sleep);
}

static uint32_t ap_apb_eb;
static void read_ap_apb_reg(void)
{
	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}
	regmap_read(deep_sys_ap_apb,
		REG_AP_APB_APB_EB,
		&ap_apb_eb);
}

static uint32_t aon_apb_eb0, aon_apb_eb1;
static void read_aon_apb_reg(void)
{
	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}
	regmap_read(deep_sys_aon_apb,
		REG_AON_APB_APB_EB0,
		&aon_apb_eb0);
	regmap_read(deep_sys_aon_apb,
		REG_AON_APB_APB_EB1,
		&aon_apb_eb1);
}

static uint32_t apb_sys_slp_status0, apb_pwr_state_dbg1,
		apb_pwr_state_dbg2, apb_pwr_state_dbg3,
		apb_pwr_state_dbg4;
static void read_aon_pmu_apb_reg(void)
{
	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}
	regmap_read(deep_sys_aon_pwu_apb,
		REG_PMU_APB_SYS_SLP_STATUS0,
		&apb_sys_slp_status0);
	regmap_read(deep_sys_aon_pwu_apb,
		REG_PMU_APB_PWR_STATE_DBG1,
		&apb_pwr_state_dbg1);
	regmap_read(deep_sys_aon_pwu_apb,
		REG_PMU_APB_PWR_STATE_DBG2,
		&apb_pwr_state_dbg2);
	regmap_read(deep_sys_aon_pwu_apb,
		REG_PMU_APB_PWR_STATE_DBG3,
		&apb_pwr_state_dbg3);
	regmap_read(deep_sys_aon_pwu_apb,
		REG_PMU_APB_PWR_STATE_DBG4,
		&apb_pwr_state_dbg4);
}

static uint32_t com_pmu_pub_frc_slp,
		com_pmu_apb_fencing0_ctrl_state,
		com_pmu_apb_fencing1_ctrl_state;
static void read_com_pmu_apb(void)
{
	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}

	regmap_read(deep_sys_com_pmu_apb,
		REG_COM_PMU_APB_AON_PUB_FRC_SLP_CTRL,
		&com_pmu_pub_frc_slp);
	regmap_read(deep_sys_com_pmu_apb,
		REG_COM_PMU_APB_FENCING0_CTRL_STATE,
		&com_pmu_apb_fencing0_ctrl_state);
	regmap_read(deep_sys_com_pmu_apb,
		REG_COM_PMU_APB_FENCING1_CTRL_STATE,
		&com_pmu_apb_fencing1_ctrl_state);
}

static void read_reg_value(void)
{
	read_subsys_ctrl();
	read_subsys_pwr_cfg();
	read_ap_ahb_reg();
	read_ap_apb_reg();
	read_aon_apb_reg();
	read_aon_pmu_apb_reg();
	read_com_pmu_apb();
}

void bak_last_reg(void)
{
	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}
	read_reg_value();
}

void show_deep_reg_status(void)
{
	pr_info("##--REG_AP_APB_APB_EB : 0x%08x\n", ap_apb_eb);
	pr_info("##--REG_AP_AHB_AHB_EB : 0x%08x\n", ahb_eb);
	pr_info("##--REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG : 0x%08x\n",
					ap_sys_auto_sleep_cfg);
	pr_info("##--REG_AON_APB_APB_EB1 : 0x%08x\n", aon_apb_eb1);
	pr_info("##--REG_AON_APB_APB_EB0 : 0x%08x\n", aon_apb_eb0);
	pr_info("##--REG_PMU_APB_SLEEP_STATUS : 0x%08x\n", apb_sys_slp_status0);
	pr_info("#####PWR_STATUS reg #####\n");
	pr_info("##--REG_PMU_APB_PWR_STATE_DBG1 : 0x%08x\n",
					apb_pwr_state_dbg1);
	pr_info("##--REG_PMU_APB_PWR_STATE_DBG2 : 0x%08x\n",
					apb_pwr_state_dbg2);
	pr_info("##--REG_PMU_APB_PWR_STATE_DBG3 : 0x%08x\n",
					apb_pwr_state_dbg3);
	pr_info("##--REG_PMU_APB_PWR_STATE_DBG4 : 0x%08x\n",
					apb_pwr_state_dbg4);

	pr_info("#####PWR_STATUS1 POWER DOMAIN#####\n");
	pr_info("##--status of power domain 'WTLCP_HU3GE_B' :0x%08x\n",
		(apb_pwr_state_dbg1 & BIT_PMU_APB_PD_WTLCP_HU3GE_B_STATE(31))
		>> 25);
	pr_info("##--status of power domain 'WTLCP_TGDSP' :0x%08x\n",
		(apb_pwr_state_dbg1 & BIT_PMU_APB_PD_WTLCP_TGDSP_STATE(31))
		>> 20);
	pr_info("##--status of power domain 'WTLCP_LTE_P4' :0x%08x\n",
		(apb_pwr_state_dbg1 & BIT_PMU_APB_PD_WTLCP_LTE_P4_STATE(31))
		>> 15);
	pr_info("##--status of power domain 'PUBCP_SYS' :0x%08x\n",
		(apb_pwr_state_dbg1 & BIT_PMU_APB_PD_PUBCP_SYS_STATE(31))
		>> 5);
	pr_info("##--status of power domain 'BIA_S0IX' :0x%08x\n",
		apb_pwr_state_dbg1 & BIT_PMU_APB_PD_BIA_S0IX_STATE(31));

	pr_info("#####PWR_STATUS2 POWER DOMAIN#####\n");
	pr_info("##--status of power domain 'VSP_SYS' :0x%08x\n",
		(apb_pwr_state_dbg2 & BIT_PMU_APB_PD_VSP_SYS_STATE(31))
		>> 25);
	pr_info("##--status of power domain 'WTLCP_LTE_P1' :0x%08x\n",
		(apb_pwr_state_dbg2 & BIT_PMU_APB_PD_WTLCP_LTE_P1_STATE(31))
		>> 20);
	pr_info("##--status of power domain 'PUB0_SYS' :0x%08x\n",
		(apb_pwr_state_dbg2 & BIT_PMU_APB_PD_PUB0_SYS_STATE(31))
		>> 15);
	pr_info("##--status of power domain 'CA53_LIT_C1' :0x%08x\n",
		(apb_pwr_state_dbg2 & BIT_PMU_APB_PD_GPU_PHANTOM_STATE(31))
		>> 5);
	pr_info("##--status of power domain 'AP_SYS' :0x%08x\n",
		apb_pwr_state_dbg2 & BIT_PMU_APB_PD_AP_SYS_STATE(31));

	pr_info("#####PWR_STATUS3 POWER DOMAIN#####\n");
	pr_info("##--status of power domain 'WTLCP_LTE_P3' :0x%08x\n",
		(apb_pwr_state_dbg3 & BIT_PMU_APB_PD_WTLCP_LTE_P3_STATE(31))
		>> 25);
	pr_info("##--status of power domain 'PD_BIA' :0x%08x\n",
		(apb_pwr_state_dbg3 & BIT_PMU_APB_PD_BIA_STATE(31))
		>> 20);
	pr_info("##--status of power domain 'WTLCP_TD' :0x%08x\n",
		(apb_pwr_state_dbg3 & BIT_PMU_APB_PD_WTLCP_TD_STATE(31))
		>> 15);
	pr_info("##--status of power domain 'GPU_TOP' :0x%08x\n",
		(apb_pwr_state_dbg3 & BIT_PMU_APB_PD_GPU_TOP_STATE(31))
		>> 5);
	pr_info("##--status of power domain 'WTLCP_LDSP' :0x%08x\n",
		apb_pwr_state_dbg3 & BIT_PMU_APB_PD_WTLCP_LDSP_STATE(31));

	pr_info("#####PWR_STATUS4 POWER DOMAIN#####\n");
	pr_info("##--status of power domain 'WTLCP_HU3GE_A' :0x%08x\n",
		(apb_pwr_state_dbg4 & BIT_PMU_APB_PD_WTLCP_HU3GE_A_STATE(31))
		>> 25);
	pr_info("##--status of power domain 'CAM_SYS' :0x%08x\n",
		(apb_pwr_state_dbg4 & BIT_PMU_APB_PD_CAM_SYS_STATE(31))
		>> 20);
	pr_info("##--status of power domain 'VSP_CORE' :0x%08x\n",
		(apb_pwr_state_dbg4 & BIT_PMU_APB_PD_VSP_CORE_STATE(31))
		>> 15);
	pr_info("##--status of power domain 'WTLCP_LTE_P2' :0x%08x\n",
		(apb_pwr_state_dbg4 & BIT_PMU_APB_PD_WTLCP_LTE_P2_STATE(31))
		>> 10);
	pr_info("##--status of power domain 'CAM_CORE' :0x%08x\n",
		(apb_pwr_state_dbg4 & BIT_PMU_APB_PD_ISP_TOP_STATE(31))
		>> 5);
	pr_info("##--status of power domain 'WTLCP_SYS' :0x%08x\n",
		apb_pwr_state_dbg4 & BIT_PMU_APB_PD_WTLCP_SYS_STATE(31));

}

static void print_subsys_crtl(void)
{
	pr_info("##--REG_PMU_APB_PUB0_SYS_SLEEP_CTRL : 0x%08x\n",
						pub0_sys_sleep_ctrl);
	pr_info("##--REG_PMU_APB_AP_SYS_SLEEP_CTRL : 0x%08x\n",
						ap_sys_sleep_ctrl);
	pr_info("##--REG_PMU_APB_CM4_SYS_SLEEP_CTRL : 0x%08x\n",
						cm4_sys_sleep_ctrl);
	pr_info("##--REG_PMU_APB_WTLCP_SYS_SLEEP_CTRL : 0x%08x\n",
						wtlcp_sys_sleep_ctrl);
	pr_info("##--REG_PMU_APB_PUBCP_SYS_SLEEP_CTRL : 0x%08x\n",
						pubcp_sys_sleep_ctrl);
}

void print_last_reg(void)
{
	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}
	pr_info("##--REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG : 0x%08x\n",
			sys_force_sleep);
	pr_info("##--REG_COM_PMU_APB_AON_PUB_FRC_SLP_CTRL : 0x%08x\n",
						com_pmu_pub_frc_slp);
	pr_info("##--REG_COM_PMU_APB_FENCING0_CTRL_STATE : 0x%08x\n",
				com_pmu_apb_fencing0_ctrl_state);
	pr_info("##--REG_COM_PMU_APB_FENCING1_CTRL_STATE : 0x%08x\n",
				com_pmu_apb_fencing1_ctrl_state);
	pr_info("##--REG_PMU_PUB0_SYS_SLEEP_CTRL : 0x%08x\n",
						pub0_sys_sleep_ctrl);
	pr_info("##--REG_PMU_AP_SYS_SLEEP_CTRL : 0x%08x\n",
						ap_sys_sleep_ctrl);
	pr_info("##--REG_PMU_CM4_SYS_SLEEP_CTRL : 0x%08x\n",
						cm4_sys_sleep_ctrl);
	pr_info("##--REG_PMU_WTLCP_SYS_SLEEP_CTRL : 0x%08x\n",
						wtlcp_sys_sleep_ctrl);
	pr_info("##--REG_PMU_PUBCP_SYS_SLEEP_CTRL : 0x%08x\n",
						pubcp_sys_sleep_ctrl);
	pr_info("##--REG_PMU_APB_PD_TOP_PWR_CFG : 0x%08x\n",
						pd_top_pwr_cfg);
	pr_info("##--REG_PMU_APB_PD_AON_SYS_PWR_CFG : 0x%08x\n",
						pd_aon_sys_pwr_cfg);
	pr_info("##--REG_PMU_APB_PD_PUBCP_SYS_PWR_CFG : 0x%08x\n",
						pd_pubcp_sys_pwr_cfg);
	pr_info("##--REG_PMU_APB_PD_WTLCP_LTE_P4_PWR_CFG : 0x%08x\n",
						pd_wtlcp_lte_p4_pwr_cfg);
	pr_info("##--REG_PMU_APB_PD_WTLCP_TGDSP_PWR_CFG : 0x%08x\n",
						pd_wtlcp_tgdsp_pwr_cfg);
	pr_info("##--REG_PMU_APB_PD_WTLCP_HU3GE_B_PWR_CFG : 0x%08x\n",
						pd_wtlcp_hu3ge_b_pwr_cfg);
	pr_info("##--REG_PMU_APB_PD_AP_SYS_PWR_CFG : 0x%08x\n",
						pd_ap_sys_pwr_cfg);
	pr_info("##--REG_PMU_APB_PD_GPU_PHANTOM_PWR_CFG1 : 0x%08x\n",
						pd_gpu_phantom_pwr_cfg1);
	pr_info("##--REG_PMU_APB_PD_PUB0_SYS_PWR_CFG : 0x%08x\n",
						pd_pub0_sys_pwr_cfg);
	pr_info("##--REG_PMU_APB_PD_WTLCP_LTE_P1_PWR_CFG : 0x%08x\n",
						pd_wtlcp_lte_p1_pwr_cfg);
	pr_info("##--REG_PMU_APB_PD_VSP_SYS_PWR_CFG : 0x%08x\n",
						pd_vsp_sys_pwr_cfg);
	pr_info("##--REG_PMU_APB_PD_WTLCP_LDSP_PWR_CFG : 0x%08x\n",
						pd_wtlcp_ldsp_pwr_cfg);
	pr_info("##--REG_PMU_APB_PD_GPU_TOP_PWR_CFG1 : 0x%08x\n",
						pd_gpu_top_pwr_cfg1);
	pr_info("##--REG_PMU_APB_PD_WTLCP_TD_PWR_CFG : 0x%08x\n",
						pd_wtlcp_td_pwr_cfg);
	pr_info("##--REG_PMU_APB_PD_WTLCP_LTE_P3_PWR_CFG : 0x%08x\n",
						pd_wtlcp_lte_p3_pwr_cfg);
	pr_info("##--REG_PMU_APB_PD_WTLCP_SYS_PWR_CFG : 0x%08x\n",
						pd_wtlcp_sys_pwr_cfg);
	pr_info("##--REG_PMU_APB_PD_CAM_CORE_PWR_CFG : 0x%08x\n",
						pd_isp_top_pwr_cfg);
	pr_info("##--REG_PMU_APB_PD_WTLCP_LTE_P2_PWR_CFG : 0x%08x\n",
						pd_wtlcp_lte_p2_pwr_cfg);
	pr_info("##--REG_PMU_APB_PD_VSP_CORE_PWR_CFG : 0x%08x\n",
						pd_vsp_core_pwr_cfg);
	pr_info("##--REG_PMU_APB_PD_CAM_SYS_PWR_CFG : 0x%08x\n",
						pd_cam_sys_pwr_cfg);
	pr_info("##--REG_PMU_APB_PD_WTLCP_HU3GE_A_PWR_CFG : 0x%08x\n",
						pd_wtlcp_hu3ge_a_pwr_cfg);

	print_subsys_crtl();
	show_deep_reg_status();
}



static void print_debug_info(void)
{
	if (is_syscon_base_vaddr_ok) {
		pr_err("%s,SYSCON register base vaddr is not ok!\n", __func__);
		return;
	}
	read_reg_value();
	show_deep_reg_status();
	pr_info("########Enter Deepsleep  Condition Check!########\n");
	if (ahb_eb & BIT_AP_AHB_DAP_EB)
		pr_info("##-- BIT_AP_AHB_DAP_EB still set --##\n");
	if (ahb_eb & BIT_AP_AHB_CE_EFS_EB)
		pr_info("##-- BIT_AP_AHB_CE_EFS_EB still set --##\n");
	if (ahb_eb & BIT_AP_AHB_CE1_EB)
		pr_info("##-- BIT_AP_AHB_CE1_EB still set --##\n");
	if (ahb_eb & BIT_AP_AHB_CE0_EB)
		pr_info("##-- BIT_AP_AHB_CE0_EB still set --##\n");
	if (ahb_eb & BIT_AP_AHB_BIA_LP_EB)
		pr_info("##-- BIT_AP_AHB_BIA_LP_EB still set --##\n");
	if (ahb_eb & BIT_AP_AHB_EMMC_EB)
		pr_info("###---- BIT_AP_AHB_EMMC_EB still set ----###\n");
	if (ahb_eb & BIT_AP_AHB_SDIO2_EB)
		pr_info("###---- BIT_AP_AHB_SDIO2_EB still set ----###\n");
	if (ahb_eb & BIT_AP_AHB_SDIO1_EB)
		pr_info("###---- BIT_AP_AHB_SDIO1_EB still set ----###\n");
	if (ahb_eb & BIT_AP_AHB_SDIO0_EB)
		pr_info("###---- BIT_AP_AHB_SDIO0_EB still set ----###\n");
	if (ahb_eb & BIT_AP_AHB_DMA_EB)
		pr_info("###---- BIT_AP_AHB_DMA_EB still set ----###\n");
	if (ahb_eb & BIT_AP_AHB_USB2_EB)
		pr_info("###---- BIT_AP_AHB_USB2_EB still set ----###\n");
	if (ap_apb_eb & BIT_AP_APB_SIM0_EB)
		pr_info("##-- BIT_AP_APB_SIM0_EB still set --##\n");
	if (ap_apb_eb & BIT_AP_APB_IIS0_EB)
		pr_info("##-- BIT_AP_APB_IIS0_EB still set --##\n");
	if (ap_apb_eb & BIT_AP_APB_IIS1_EB)
		pr_info("##-- BIT_AP_APB_IIS1_EB still set --##\n");
	if (ap_apb_eb & BIT_AP_APB_IIS2_EB)
		pr_info("##-- BIT_AP_APB_IIS2_EB still set --##\n");
	if (ap_apb_eb & BIT_AP_APB_IIS3_EB)
		pr_info("##-- BIT_AP_APB_IIS3_EB still set --##\n");
	if (ap_apb_eb & BIT_AP_APB_SPI0_EB)
		pr_info("##-- BIT_AP_APB_SPI0_EB still set --##\n");
	if (ap_apb_eb & BIT_AP_APB_SPI1_EB)
		pr_info("##-- BIT_AP_APB_SPI1_EB still set --##\n");
	if (ap_apb_eb & BIT_AP_APB_SPI2_EB)
		pr_info("##-- BIT_AP_APB_SPI2_EB still set --##\n");
	if (ap_apb_eb & BIT_AP_APB_SPI3_EB)
		pr_info("###---- BIT_AP_APB_SPI3_EB still set ----###\n");
	if (ap_apb_eb & BIT_AP_APB_I2C0_EB)
		pr_info("##-- BIT_AP_APB_I2C0_EB still set --##\n");
	if (ap_apb_eb & BIT_AP_APB_I2C1_EB)
		pr_info("##-- BIT_AP_APB_I2C1_EB still set --##\n");
	if (ap_apb_eb & BIT_AP_APB_I2C2_EB)
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
	if (aon_apb_eb0 & BIT_AON_APB_AON_GPU_EB)
		pr_info("##-- BIT_AON_APB_AON_GPU_EB still set --##\n");
	if (aon_apb_eb0 & BIT_AON_APB_AON_DISP_EB)
		pr_info("##-- BIT_AON_APB_AON_DISP_EB still set --##\n");
	if (aon_apb_eb0 & BIT_AON_APB_AON_VSP_EB)
		pr_info("##-- BIT_AON_APB_GPU_EB still set --##\n");
	if (aon_apb_eb0 & BIT_AON_APB_CA53_DAP_EB)
		pr_info("##-- BIT_AON_APB_CA53_DAP_EB still set --##\n");
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

#define PM_PRINT_ENABLE
#ifdef PM_PRINT_ENABLE
static struct wake_lock messages_wakelock;
#endif

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

/*pm_debug_clr function is not used now*/
#if 0
void pm_debug_clr(void)
{
	if (dentry_debug_root != NULL)
		debugfs_remove_recursive(dentry_debug_root);
}
#endif

void print_pmu_wakeup_sources(void)
{
	uint32_t bia_wakeup_en0 = 0, bia_wakeup_en1 = 0, bia_wakeup_en2 = 0,
			bia_wakeup_en3 = 0;

	regmap_read(deep_sys_aon_pwu_apb,
				REG_PMU_APB_BASEIA_WAKEUP_SOURCE_EN0,
				&bia_wakeup_en0);
	regmap_read(deep_sys_aon_pwu_apb,
				REG_PMU_APB_BASEIA_WAKEUP_SOURCE_EN1,
				&bia_wakeup_en1);
	regmap_read(deep_sys_aon_pwu_apb,
				REG_PMU_APB_BASEIA_WAKEUP_SOURCE_EN2,
				&bia_wakeup_en2);
	regmap_read(deep_sys_aon_pwu_apb,
				REG_PMU_APB_BASEIA_WAKEUP_SOURCE_EN3,
				&bia_wakeup_en3);
	pr_info("##--BIA WAKEUP SOURCE :\n en0:0x%08x\n en1:0x%08x\n en2:0x%08x\n en3:0x%08x\n",
		bia_wakeup_en0, bia_wakeup_en1, bia_wakeup_en2, bia_wakeup_en3);

}

static struct syscore_ops sprd_isoc_int_status = {
	.resume = print_bia_irq_status,
};

static __init int sprd_isoc_int_status_dbg_init(void)
{
	deep_bia_intc = ioremap_nocache(IOAPIC_BASE_PADDR, 0x3000);
	if (IS_ERR(deep_bia_intc)) {
		pr_err("%s,failed to find deep_ioremap_bia_intc\n", __func__);
		return -EIO;
	}

	/* Make sure this ops is registered after IOAPIC syscore since ioapic
	 * status only available when ioapic RETs are restored.
	 */
	register_syscore_ops(&sprd_isoc_int_status);
	return 0;
}

device_initcall_sync(sprd_isoc_int_status_dbg_init);

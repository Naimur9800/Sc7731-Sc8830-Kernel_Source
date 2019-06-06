/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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
#include <soc/sprd/pm_debug.h>
#if defined(CONFIG_ARCH_SCX35L64)||defined(CONFIG_ARCH_SCX35LT8)
#include <soc/sprd/sprd_persistent_clock.h>
#endif
#include <soc/sprd/sci_glb_regs.h>
#include <soc/sprd/sci.h>
#include <soc/sprd/adi.h>
#include <linux/interrupt.h>
#include <soc/sprd/irqs.h>
#include <soc/sprd/gpio.h>
#include <soc/sprd/iomap.h>

struct dentry * dentry_debug_root = NULL;
static int is_print_sleep_mode = 0;
int is_print_linux_clock = 1;
int is_print_modem_clock = 1;
static int is_print_irq = 1;
static int is_print_wakeup = 1;
static int is_print_irq_runtime = 0;
static int is_print_time = 1;
static int print_thread_enable = 1;
static int print_thread_interval = 30;
static unsigned int core_time = 0;
static unsigned int mcu_time = 0;
static unsigned int lit_time = 0;
static unsigned int deep_time_successed = 0;
static unsigned int deep_time_failed = 0;
static unsigned int sleep_time = 0;
#define		SPRD_INTC_NUM			4
#define 	SPRD_HARD_INTERRUPT_NUM 128
#define 	SPRD_HARD_INT_NUM_EACH_INTC 32
#define		SPRD_IRQ_NUM			1024
static u32 sprd_hard_irq[SPRD_HARD_INTERRUPT_NUM]= {0, };
//static u32 sprd_irqs[SPRD_IRQ_NUM] = {0, };
static u32 sprd_irqs_sts[SPRD_INTC_NUM] = {0, };
static int is_wakeup = 0;
static int irq_status = 0;
//static int hard_irq_status[SPRD_INTC_NUM] = {0, };
static int sleep_mode = SLP_MODE_NON;
static char * sleep_mode_str[]  = {
	"[ARM]",
	"[MCU]",
	"[DEP]",
	"[LIT]",
	"[NON]"
};
#define INT_REG(off)		(SPRD_INT_BASE + (off))
#define	INTC0_REG(off)		(SPRD_INTC0_BASE + (off))
#define	INTC1_REG(off)		(SPRD_INTC1_BASE + (off))
#define	INTC2_REG(off)		(SPRD_INTC2_BASE + (off))
#define	INTC3_REG(off)		(SPRD_INTC3_BASE + (off))
#define INT_IRQ_STS            INT_REG(0x0000)
#define INT_IRQ_RAW           INT_REG(0x0004)
#define INT_IRQ_ENB           INT_REG(0x0008)
#define INT_IRQ_DIS            INT_REG(0x000c)
#define INT_FIQ_STS            INT_REG(0x0020)
#define	INTCV0_IRQ_MSKSTS	INTC0_REG(0x0000)
#define	INTCV0_IRQ_RAW		INTC0_REG(0x0004)
#define	INTCV0_IRQ_EN		INTC0_REG(0x0008)
#define	INTCV0_IRQ_DIS		INTC0_REG(0x000C)
#define	INTCV0_FIQ_STS		INTC0_REG(0x0020)
#define	INTCV1_IRQ_MSKSTS	INTC1_REG(0x0000)
#define	INTCV1_IRQ_RAW		INTC1_REG(0x0004)
#define	INTCV1_IRQ_EN		INTC1_REG(0x0008)
#define	INTCV1_IRQ_DIS		INTC1_REG(0x000C)
#define	INTCV1_FIQ_STS		INTC1_REG(0x0020)
#define	INTCV2_IRQ_MSKSTS	INTC2_REG(0x0000)
#define	INTCV2_IRQ_RAW		INTC2_REG(0x0004)
#define	INTCV2_IRQ_EN		INTC2_REG(0x0008)
#define	INTCV2_IRQ_DIS		INTC2_REG(0x000C)
#define	INTCV2_FIQ_STS		INTC2_REG(0x0020)
#define	INTCV3_IRQ_MSKSTS	INTC3_REG(0x0000)
#define	INTCV3_IRQ_RAW		INTC3_REG(0x0004)
#define	INTCV3_IRQ_EN		INTC3_REG(0x0008)
#define	INTCV3_IRQ_DIS		INTC3_REG(0x000C)
#define	INTCV3_FIQ_STS		INTC3_REG(0x0020)
#define INT_IRQ_MASK	(1<<3)

#define ANA_REG_INT_MASK_STATUS         (ANA_CTL_INT_BASE + 0x0000)
#define ANA_REG_INT_RAW_STATUS          (ANA_CTL_INT_BASE + 0x0004)
#define ANA_REG_INT_EN                  (ANA_CTL_INT_BASE + 0x0008)
#define ANA_REG_INT_MASK_STATUS_SYNC    (ANA_CTL_INT_BASE + 0x000c)

void pm_debug_dump_ahb_glb_regs(void);

static void hard_irq_reset(void)
{
	int i = SPRD_HARD_INTERRUPT_NUM - 1;
	do{
		sprd_hard_irq[i] = 0;
	}while(--i >= 0);
}

static void parse_hard_irq(unsigned long val, unsigned long intc)
{
	int i;
	if(intc == 0){
		for (i = 0; i < SPRD_HARD_INT_NUM_EACH_INTC; i++) {
			if (test_and_clear_bit(i, &val)) sprd_hard_irq[i]++;
		}
	}
	if(intc == 1){
		for (i = 0; i < SPRD_HARD_INT_NUM_EACH_INTC; i++) {
			if (test_and_clear_bit(i, &val)) sprd_hard_irq[32+i]++;
		}
	}
	if(intc == 2){
		for (i = 0; i < SPRD_HARD_INT_NUM_EACH_INTC; i++) {
			if (test_and_clear_bit(i, &val)) sprd_hard_irq[64+i]++;
		}
	}
	if(intc == 3){
		for (i = 0; i < SPRD_HARD_INT_NUM_EACH_INTC; i++) {
			if (test_and_clear_bit(i, &val)) sprd_hard_irq[96+i]++;
		}
	}
}

void hard_irq_set(void)
{
	sprd_irqs_sts[0] = __raw_readl(INT_IRQ_STS);
	sprd_irqs_sts[1] = __raw_readl(INT_FIQ_STS);
	irq_status = __raw_readl(INTCV0_IRQ_RAW);
	parse_hard_irq(irq_status, 0);
	irq_status = __raw_readl(INTCV1_IRQ_RAW);
	parse_hard_irq(irq_status, 1);
	irq_status = __raw_readl(INTCV2_IRQ_RAW);
	parse_hard_irq(irq_status, 2);
	irq_status = __raw_readl(INTCV3_IRQ_RAW);
	parse_hard_irq(irq_status, 3);
}
void print_int_status(void)
{
	printk("APB_EB 0x%08x\n", __raw_readl(REG_AP_APB_APB_EB));
	printk("INTC0 mask:0x%08x raw:0x%08x en:0x%08x\n", __raw_readl(INTCV0_IRQ_MSKSTS),__raw_readl(INTCV0_IRQ_RAW), __raw_readl(INTCV0_IRQ_EN));
	printk("INTC1 mask:0x%08x raw:0x%08x en:0x%08x\n", __raw_readl(INTCV1_IRQ_MSKSTS),__raw_readl(INTCV1_IRQ_RAW), __raw_readl(INTCV1_IRQ_EN));
	printk("INTC2 mask:0x%08x raw:0x%08x en:0x%08x\n", __raw_readl(INTCV2_IRQ_MSKSTS),__raw_readl(INTCV2_IRQ_RAW), __raw_readl(INTCV2_IRQ_EN));
	printk("INTC3 mask:0x%08x raw:0x%08x en:0x%08x\n", __raw_readl(INTCV3_IRQ_MSKSTS),__raw_readl(INTCV3_IRQ_RAW), __raw_readl(INTCV3_IRQ_EN));
	printk("INT mask:0x%08x raw:0x%08x en:0x%08x\n", __raw_readl(INT_IRQ_STS),__raw_readl(INT_IRQ_RAW), __raw_readl(INT_IRQ_ENB));
	printk("ANA INT mask:0x%08x raw:0x%08x en:0x%08x\n", sci_adi_read(ANA_REG_INT_MASK_STATUS), sci_adi_read(ANA_REG_INT_RAW_STATUS), sci_adi_read(ANA_REG_INT_EN));
	printk("ANA EIC MODULE_EN 0x%08x eic bit(3)\n", sci_adi_read(ANA_REG_GLB_ARM_MODULE_EN));
	printk("ANA EIC int en 0x%08x\n", sci_adi_read(ANA_CTL_EIC_BASE + 0x18));
	printk("ANA EIC int status 0x%08x, 0x%08x\n", sci_adi_read(ANA_CTL_EIC_BASE + 0x20),sci_adi_read(ANA_CTL_EIC_BASE + 0x14));
}

#define GPIO_GROUP_NUM		16
#define IRQ_EIC		(1<<14)
#define IRQ_BUSMON	(1<<13)
#define IRQ_WDG		(1<<11)
#define IRQ_CP2		(1<<10)
#define IRQ_CP1		(1<<9)
#define IRQ_CP0		(1<<8)
#define IRQ_CP_WDG	(1<<7)
#define IRQ_GPU		(1<<6)
#define IRQ_MM		(1<<5)
#define IRQ_4		(1<<4)
#define IRQ_3		(1<<3)
#define IRQ_2		(1<<2)

#define REG_GPIO_MIS            (0x0020)
void print_hard_irq_inloop(int ret)
{
	unsigned int i, j, val;
	unsigned int ana_sts;
	unsigned int gpio_irq[GPIO_GROUP_NUM];

	if(sprd_irqs_sts[0] != 0)
		printk("%c#:INTC0: %08x\n", ret?'S':'F', sprd_irqs_sts[0]);
	if(sprd_irqs_sts[1] != 0)
		printk("%c#:INTC0 FIQ: %08x\n", ret?'S':'F', sprd_irqs_sts[1]);
	if(sprd_irqs_sts[0]&IRQ_EIC){
		printk("wake up by eic\n");
	}
	if(sprd_irqs_sts[0]&IRQ_BUSMON){
		printk("wake up by busmoniter\n");
	}
	if(sprd_irqs_sts[0]&IRQ_WDG){
		printk("wake up by ca7_wdg or ap_wdg\n");
	}
	if(sprd_irqs_sts[0]&IRQ_CP2){
		printk("wake up by cp2\n");
	}
	if(sprd_irqs_sts[0]&IRQ_CP1){
		printk("wake up by cp1\n");
	}
	if(sprd_irqs_sts[0]&IRQ_CP0){
		printk("wake up by cp0\n");
	}
	if(sprd_irqs_sts[0]&IRQ_GPU){
		printk("wake up by gpu\n");
	}
	if(sprd_irqs_sts[0]&IRQ_MM){
		printk("wake up by mm\n");
	}
	if(sprd_irqs_sts[0]&IRQ_4){
		if(sprd_hard_irq[34])
			printk("wake up by i2c\n");
		if(sprd_hard_irq[20])
			printk("wake up by audio\n");
		if(sprd_hard_irq[27])
			printk("wake up by fm\n");
		if(sprd_hard_irq[25]){
			printk("wake up by adi\n");
		}
		if(sprd_hard_irq[38]){
			printk("wake up by ana ");
			ana_sts = sci_adi_read(ANA_REG_INT_RAW_STATUS);
			if(ana_sts & BIT(0))
				printk("adc\n");
			if(ana_sts & BIT(1)){
				printk("gpio\n");
				printk("gpio 0 ~ 16 0x%08x\n", sci_adi_read(SPRD_MISC_BASE + 0x0480 + 0x1c));
				printk("gpio 17 ~ 31 0x%08x\n", sci_adi_read(SPRD_MISC_BASE + 0x0480 + 0x40 + 0x1c));
			}
			if(ana_sts & BIT(2))
				printk("rtc\n");
			if(ana_sts & BIT(3))
				printk("wdg\n");
			if(ana_sts & BIT(4))
				printk("fgu\n");
			if(ana_sts & BIT(5)){
				printk("eic\n");
				printk("ana eic 0x%08x\n", sci_adi_read(ANA_EIC_BASE + 0x1c));
			}
			if(ana_sts & BIT(6))
				printk("aud head button\n");
			if(ana_sts & BIT(7))
				printk("aud protect\n");
			if(ana_sts & BIT(8))
				printk("thermal\n");
			if(ana_sts & BIT(10))
				printk("dcdc otp\n");
			printk("ANA INT 0x%08x\n", ana_sts);
		}
		if(sprd_hard_irq[36])
			printk("wake up by kpd\n");
		if(sprd_hard_irq[35]){
			printk("wake up by gpio\n");
		}
	}
	if(sprd_irqs_sts[0]&IRQ_3){
		if(sprd_hard_irq[23])
			printk("wake up by vbc_ad01\n");
		if(sprd_hard_irq[24])
			printk("wake up by vbc_ad23\n");
		if(sprd_hard_irq[22])
			printk("wake up by vbc_da\n");
		if(sprd_hard_irq[21])
			printk("wake up by afifi_error\n");
	}
	if(sprd_irqs_sts[0]&IRQ_2){
		if(sprd_hard_irq[29]){
			printk("wake up by ap_tmr0\n");
		}
		if(sprd_hard_irq[118]){
			printk("wake up by ap_tmr1\n");
		}
		if(sprd_hard_irq[119]){
			printk("wake up by ap_tmr2\n");
		}
		if(sprd_hard_irq[120]){
			printk("wake up by ap_tmr3\n");
		}
		if(sprd_hard_irq[121]){
			printk("wake up by ap_tmr4\n");
		}
		if(sprd_hard_irq[31]){
			printk("wake up by ap_syst\n");
		}
		if(sprd_hard_irq[30]){
			printk("wake up by aon_syst\n");
		}
		if(sprd_hard_irq[28]){
			printk("wake up by aon_tmr\n");
		}
	}

	if (sprd_hard_irq[35]) {
		for(i=0; i<(GPIO_GROUP_NUM/2); i++){
			j = 2*i;
			gpio_irq[j]= __raw_readl(SPRD_GPIO_BASE + 0x100*i + REG_GPIO_MIS);
			gpio_irq[j+1]= __raw_readl(SPRD_GPIO_BASE + 0x100*i + 0x80 + REG_GPIO_MIS);
	//		printk("gpio_irq[%d]:0x%x, gpio_irq[%d]:0x%x \n", j, gpio_irq[j], j+1, gpio_irq[j+1]);
		}
		for(i=0; i<GPIO_GROUP_NUM; i++){
			if(gpio_irq[i] != 0){
				val = gpio_irq[i];
				while(val){
					j = __ffs(val);
					printk("irq from gpio%d\n", j + 16*i);
					val &= ~(1<<j);
				}
			}
		}
	}
}

static void print_hard_irq(void)
{
	int i = SPRD_HARD_INTERRUPT_NUM -1;
	if(!is_print_irq)
		return;
	do{
		if(0 != sprd_hard_irq[i])
			printk("##: sprd_hard_irq[%d] = %d.\n",
                                i, sprd_hard_irq[i]);
	}while(--i >= 0);
}

#if 0
static void irq_reset(void)
{
	int i = SPRD_IRQ_NUM - 1;
	do{
		sprd_irqs[i] = 0;
	}while(--i >= 0);
}


void inc_irq(int irq)
{
	if(is_wakeup){
		if (irq >= SPRD_IRQ_NUM) {
			printk("## bad irq number %d.\n", irq);
			return;
		}
		sprd_irqs[irq]++;
		if(is_print_wakeup)
			printk("\n#####: wakeup irq = %d.\n", irq);
		is_wakeup = 0;
	}
}
EXPORT_SYMBOL(inc_irq);


static void print_irq(void)
{
	int i = SPRD_IRQ_NUM - 1;
	if(!is_print_irq)
		return;
	do{
		if(0 != sprd_irqs[i])
			printk("##: sprd_irqs[%d] = %d.\n",
                                i, sprd_irqs[i]);
	}while(--i >= 0);
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
	switch(sleep_mode){
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
			if(ret)
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
	sleep_time = get_sys_cnt();
	hard_irq_reset();
}

void time_statisic_end(void)
{
	sleep_time = get_sys_cnt() - sleep_time;
}

void print_time(void)
{
	if(!is_print_time)
		return;
	printk("time statisics : sleep_time=%d, core_time=%d, mcu_time=%d, lit_time=%d, deep_sus=%d, dep_fail=%d\n",
		sleep_time, core_time, mcu_time, lit_time, deep_time_successed, deep_time_failed);
}

void set_sleep_mode(int sm){
	int is_print = (sm == sleep_mode);
	sleep_mode = sm;
	if(is_print_sleep_mode == 0 || is_print )
		return;
	switch(sm){
		case SLP_MODE_ARM:
			printk("\n[ARM]\n");
			break;
		case SLP_MODE_MCU:
			printk("\n[MCU]\n");
			break;
		case SLP_MODE_LIT:
			printk("\n[LIT]\n");
			break;
		case SLP_MODE_DEP:
			printk("\n[DEP]\n");
			break;
		default:
			printk("\nNONE\n");
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
	pm_debug_dump_ahb_glb_regs();
	if(is_print_wakeup){
		printk("###wake up form %s : %08x\n",  sleep_mode_str[sleep_mode],  sprd_irqs_sts[0]);
		printk("###wake up form %s : %08x\n",  sleep_mode_str[sleep_mode],  sprd_irqs_sts[1]);
	}
}
#ifdef PM_PRINT_ENABLE
static struct wake_lock messages_wakelock;
#endif

#define PM_PRINT_ENABLE
#if defined(CONFIG_ARCH_SCX35L)
void cp0_power_domain_debug(void)
{
        unsigned long cp0_arm9_0_cfg = __raw_readl(REG_PMU_APB_PD_CP0_ARM9_0_CFG);
        unsigned long cp0_arm9_1_cfg = __raw_readl(REG_PMU_APB_PD_CP0_ARM9_1_CFG);
        unsigned long cp0_hu3ge_cfg = __raw_readl(REG_PMU_APB_PD_CP0_HU3GE_CFG);
        unsigned long cp0_gsm_0_cfg = __raw_readl(REG_PMU_APB_PD_CP0_GSM_0_CFG);
        unsigned long cp0_gsm_1_cfg = __raw_readl(REG_PMU_APB_PD_CP0_GSM_1_CFG);
        unsigned long cp0_td_cfg = __raw_readl(REG_PMU_APB_PD_CP0_TD_CFG);
        unsigned long cp0_ceva_0_cfg = __raw_readl(REG_PMU_APB_PD_CP0_CEVA_0_CFG);
        unsigned long cp0_ceva_1_cfg = __raw_readl(REG_PMU_APB_PD_CP0_CEVA_1_CFG);

        printk("###---- REG_PMU_APB_PD_CP0_ARM9_0_CFG : 0x%08x\n", cp0_arm9_0_cfg);
        printk("###---- REG_PMU_APB_PD_CP0_ARM9_1_CFG : 0x%08x\n", cp0_arm9_1_cfg);
        printk("###---- REG_PMU_APB_PD_CP0_HU3GE_CFG : 0x%08x\n", cp0_hu3ge_cfg);
        printk("###---- REG_PMU_APB_PD_CP0_GSM_0_CFG : 0x%08x\n", cp0_gsm_0_cfg);
        printk("###---- REG_PMU_APB_PD_CP0_GSM_1_CFG : 0x%08x\n", cp0_gsm_1_cfg);
        printk("###---- REG_PMU_APB_PD_CP0_TD_CFG : 0x%08x\n", cp0_td_cfg);
        printk("###---- REG_PMU_APB_PD_CP0_CEVA_0_CFG : 0x%08x\n", cp0_ceva_0_cfg);
        printk("###---- REG_PMU_APB_PD_CP0_CEVA_1_CFG : 0x%08x\n", cp0_ceva_1_cfg);
}

void cp1_power_domain_debug(void)
{
        unsigned long cp1_ca5_cfg = __raw_readl(REG_PMU_APB_PD_CP1_CA5_CFG);
        unsigned long cp1_lte_p1_cfg = __raw_readl(REG_PMU_APB_PD_CP1_LTE_P1_CFG);
        unsigned long cp1_lte_p2_cfg = __raw_readl(REG_PMU_APB_PD_CP1_LTE_P2_CFG);
        unsigned long cp1_ceva_cfg = __raw_readl(REG_PMU_APB_PD_CP1_CEVA_CFG);
        /*unsigned long cp1_comwrap_cfg = __raw_readl(REG_PMU_APB_PD_COMWRAP_CFG);*/

        printk("###---- REG_PMU_APB_PD_CP1_CA5_CFG : 0x%08x\n", cp1_ca5_cfg);
        printk("###---- REG_PMU_APB_PD_CP1_LTE_P1_CFG : 0x%08x\n", cp1_lte_p1_cfg);
        printk("###---- REG_PMU_APB_PD_CP1_LTE_P2_CFG : 0x%08x\n", cp1_lte_p2_cfg);
        printk("###---- REG_PMU_APB_PD_CP1_CEVA_CFG : 0x%08x\n", cp1_ceva_cfg);
        /*printk("REG_PMU_APB_PD_COMWRAP_CFG ------ 0x%08x\n", cp1_comwrap_cfg);*/
}
#endif

static void print_debug_info(void)
{
	unsigned int ahb_eb, apb_eb0, cp_slp_status0, cp_slp_status1, ldo_pd_ctrl,
			ap_apb_eb, apb_pwrstatus0, apb_pwrstatus1, apb_pwrstatus2,
			apb_pwrstatus3, mpll_cfg, dpll_cfg, emc_clk_cfg,
			apb_slp_status, ap_sys_auto_sleep_cfg, ca5_lte_status;
#if defined(CONFIG_ARCH_SCX15)
	unsigned int ldo_dcdc_pd_ctrl;
#endif
#if defined(CONFIG_ARCH_SCX30G) || defined(CONFIG_ARCH_SCX35L)
	unsigned int mpll_cfg1, dpll_cfg1, aon_apb_eb1;
#endif
#if defined(CONFIG_ARCH_SCX35L)
	unsigned int mpll_cfg2, dpll_cfg2;
#endif
	ahb_eb = sci_glb_read(REG_AP_AHB_AHB_EB, -1UL);
	ap_sys_auto_sleep_cfg = sci_glb_read(REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG, -1UL);
	ap_apb_eb = sci_glb_read(REG_AP_APB_APB_EB, -1UL);
	apb_eb0 = sci_glb_read(REG_AON_APB_APB_EB0, -1UL);
	cp_slp_status0 = sci_glb_read(REG_PMU_APB_CP_SLP_STATUS_DBG0, -1UL);
#if !defined(CONFIG_ARCH_SCX35L)
	cp_slp_status1 = sci_glb_read(REG_PMU_APB_CP_SLP_STATUS_DBG1, -1UL);
#endif
	ca5_lte_status = sci_glb_read(REG_PUB_APB_DDR_ID2QOS_RCFG9, -1UL);
	apb_pwrstatus0 = sci_glb_read(REG_PMU_APB_PWR_STATUS0_DBG, -1UL);
	apb_pwrstatus1 = sci_glb_read(REG_PMU_APB_PWR_STATUS1_DBG, -1UL);
	apb_pwrstatus2 = sci_glb_read(REG_PMU_APB_PWR_STATUS2_DBG, -1UL);
#if !defined(CONFIG_ARCH_SCX35L)
	apb_pwrstatus3 = sci_glb_read(REG_PMU_APB_PWR_STATUS3_DBG, -1UL);
#endif
#if defined(CONFIG_ARCH_SCX35LT8)
	apb_pwrstatus3 = sci_glb_read(REG_PMU_APB_PWR_STATUS3_DBG, -1UL);
#endif
	apb_slp_status = __raw_readl(REG_PMU_APB_SLEEP_STATUS);
#if defined(CONFIG_ARCH_SCX15)
	mpll_cfg = sci_glb_read(REG_AON_APB_MPLL_CFG, -1UL);
	dpll_cfg = sci_glb_read(REG_AON_APB_DPLL_CFG, -1UL);
#endif
	emc_clk_cfg = sci_glb_read(REG_AON_CLK_EMC_CFG, -1UL);
	ldo_pd_ctrl = sci_adi_read(ANA_REG_GLB_LDO_PD_CTRL);
#if defined(CONFIG_ARCH_SCX30G) || defined(CONFIG_ARCH_SCX35L)
	mpll_cfg1 = sci_glb_read(REG_AON_APB_MPLL_CFG1, -1UL);
	dpll_cfg1 = sci_glb_read(REG_AON_APB_DPLL_CFG1, -1UL);
	aon_apb_eb1 = sci_glb_read(REG_AON_APB_APB_EB1, -1UL);
#endif
#if defined(CONFIG_ARCH_SCX35L)
	mpll_cfg2 = sci_glb_read(REG_AON_APB_MPLL_CFG2, -1UL);
    dpll_cfg2 = sci_glb_read(REG_AON_APB_DPLL_CFG2, -1UL);
#endif
#if defined(CONFIG_ARCH_SCX15)
	ldo_dcdc_pd_ctrl = sci_adi_read(ANA_REG_GLB_LDO_DCDC_PD);
#endif
	unsigned long sleep_ctrl = __raw_readl(REG_PMU_APB_SLEEP_CTRL);
	printk("###---- REG_PMU_APB_SLEEP_CTRL : 0x%08x\n", sleep_ctrl);
	printk("###---- REG_AP_AHB_AHB_EB : 0x%08x\n", ahb_eb);
	printk("###---- REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG : 0x%08x\n", ap_sys_auto_sleep_cfg);
	printk("###---- REG_AP_APB_APB_EB : 0x%08x\n", ap_apb_eb);
	printk("###---- REG_AON_APB_APB_EB0 : 0x%08x\n", apb_eb0);
#if defined(CONFIG_ARCH_SCX30G) || defined(CONFIG_ARCH_SCX35L)
	printk("###---- REG_AON_APB_APB_EB1 : 0x%08x\n", aon_apb_eb1);
#endif
	printk("###---- REG_PMU_APB_CP_SLP_STATUS_DBG0 : 0x%08x\n", cp_slp_status0);
#if !defined(CONFIG_ARCH_SCX35L)
	printk("###---- REG_PMU_APB_CP_SLP_STATUS_DBG1 : 0x%08x\n", cp_slp_status1);
#endif
	printk("###---- REG_PMU_APB_PWR_STATUS0_DBG : 0x%08x\n", apb_pwrstatus0);
	printk("###---- REG_PMU_APB_PWR_STATUS1_DBG : 0x%08x\n", apb_pwrstatus1);
	printk("###---- REG_PMU_APB_PWR_STATUS2_DBG : 0x%08x\n", apb_pwrstatus2);
	printk("###---- REG_PMU_APB_PWR_STATUS3_DBG : 0x%08x\n", apb_pwrstatus3);
	printk("###---- REG_PMU_APB_SLEEP_STATUS : 0x%08x\n", apb_slp_status);
	printk("###---- REG_PUB_APB_DDR_ID2QOS_RCFG9 : 0x%08x\n", ca5_lte_status);
#if defined(CONFIG_ARCH_SCX15) || defined(CONFIG_ARCH_SCX35) || defined(CONFIG_ARCH_SCX30G)
	printk("###---- REG_AON_APB_MPLL_CFG : 0x%08x\n", mpll_cfg);
	printk("###---- REG_AON_APB_DPLL_CFG : 0x%08x\n", dpll_cfg);
#endif
	printk("###---- REG_AON_CLK_EMC_CFG : 0x%08x\n", emc_clk_cfg);
	printk("###---- ANA_REG_GLB_LDO_PD_CTRL : 0x%08x\n", ldo_pd_ctrl);
#if defined(CONFIG_ARCH_SCX30G) || defined(CONFIG_ARCH_SCX35L)
	printk("###---- REG_AON_APB_MPLL_CFG1 : 0x%08x\n", mpll_cfg1);
	printk("###---- REG_AON_APB_DPLL_CFG1 : 0x%08x\n", dpll_cfg1);
	printk("###---- REG_PMU_APB_DDR_SLEEP_CTRL : 0x%08x\n",
				sci_glb_read(REG_PMU_APB_DDR_SLEEP_CTRL, -1UL) );
#endif
#if defined(CONFIG_ARCH_SCX35L)
	printk("###---- REG_AON_APB_MPLL_CFG2 : 0x%08x\n", mpll_cfg2);
        printk("###---- REG_AON_APB_DPLL_CFG2 : 0x%08x\n", dpll_cfg2);
#endif
#if defined(CONFIG_ARCH_SCX15)
       printk("###---- ANA_REG_GLB_LDO_DCDC_PD_CTRL : 0x%08x\n", ldo_dcdc_pd_ctrl);
#endif
	if (apb_eb0 & BIT_GPU_EB)
		printk("###---- BIT_GPU_EB still set ----###\n");
	if (apb_eb0 & BIT_MM_EB)
		printk("###---- BIT_MM_EB still set ----###\n");
	if (apb_eb0 & BIT_CA7_DAP_EB)
		printk("###---- BIT_CA7_DAP_EB still set ----###\n");

	if (ahb_eb & BIT_GSP_EB)
		printk("###---- BIT_GSP_EB still set ----###\n");
#if defined(CONFIG_ARCH_SCX15) || defined(CONFIG_ARCH_SCX30G)
	if (ahb_eb & BIT_DISPC1_EB)
		printk("###---- BIT_DISPC1_EB still set ----###\n");
#endif
#if defined(CONFIG_ARCH_SCX35L)
	if (ahb_eb & BIT_HSIC_EB)
                printk("###---- BIT_HSIC_EB still set ----###\n");
	if (ahb_eb & BIT_DISPC_EB)
                printk("###---- BIT_DISPC_EB still set ----###\n");
#else
	if (ahb_eb & BIT_DISPC0_EB)
		printk("###---- BIT_DISPC0_EB still set ----###\n");
#endif
	if (ahb_eb & BIT_SDIO0_EB)
		printk("###---- SDIO0_EB still set ----###\n");
	if (ahb_eb & BIT_SDIO1_EB)
		printk("###---- SDIO1_EB still set ----###\n");
	if (ahb_eb & BIT_SDIO2_EB)
		printk("###---- BIT_SDIO2_EB still set ----###\n");
#if defined(CONFIG_ARCH_SCX35L)
	if (ahb_eb & BIT_OTG_EB)
                printk("###---- BIT_OTG_EB still set ----###\n");
#else
	if (ahb_eb & BIT_USB_EB)
		printk("###---- BIT_USB_EB still set ----###\n");
#endif
	if (ahb_eb & BIT_DMA_EB)
		printk("###---- BIT_DMA_EB still set ----###\n");
	if (ahb_eb & BIT_NFC_EB)
		printk("###---- BIT_NFC_EB still set ----###\n");
	if (ahb_eb & BIT_EMMC_EB)
		printk("###---- BIT_EMMC_EB still set ----###\n");
#if defined(CONFIG_ARCH_SCX15) || defined(CONFIG_ARCH_SCX30G) || defined(CONFIG_64BIT)
	if (ahb_eb & BIT_ZIPDEC_EB)
		printk("###---- BIT_ZIPDEC_EB still set ----###\n");
	if (ahb_eb & BIT_ZIPENC_EB)
		printk("###---- BIT_ZIPENC_EB still set ----###\n");
	if (ahb_eb & BIT_NANDC_ECC_EB)
		printk("###---- BIT_NANDC_ECC_EB still set ----###\n");
	if (ahb_eb & BIT_NANDC_2X_EB)
		printk("###---- BIT_NANDC_2X_EB still set ----###\n");
	if (ahb_eb & BIT_NANDC_EB)
		printk("###---- BIT_NANDC_EB still set ----###\n");
#endif
#if defined(CONFIG_ARCH_SCX30G)
	if (aon_apb_eb1 & BIT_GSP_EMC_EB )
		printk("###---- BIT_GSP_EMC_EB set ----###\n");
	if (aon_apb_eb1 & BIT_ZIP_EMC_EB )
		printk("###---- BIT_ZIP_EMC_EB set ----###\n");
	if (aon_apb_eb1 & BIT_DISP_EMC_EB )
		printk("###---- BIT_DISP_EMC_EB set ----###\n");
#endif

	/*A-die*/
	printk("---------- A-die power status -----------\n");
	if (!(ldo_pd_ctrl & BIT_DCDC_WPA_PD))
		printk("###---- BIT_DCDC_WPA_PD power on! ----###\n");
#if defined(CONFIG_ADIE_SC2713S)
	if (!(ldo_pd_ctrl & BIT_LDO_CLSG_PD))
		printk("###---- BIT_LDO_CLSG_PD power on! ----###\n");
#endif
	if (!(ldo_pd_ctrl & BIT_LDO_USB_PD))
		printk("###---- BIT_LDO_USB_PD power on! ----###\n");
	if (!(ldo_pd_ctrl & BIT_LDO_CAMMOT_PD))
		printk("###---- BIT_LDO_CAMMOT_PD power on! ----###\n");
	if (!(ldo_pd_ctrl & BIT_LDO_CAMIO_PD))
		printk("###---- BIT_LDO_CAMIO_PD power on! ----###\n");
	if (!(ldo_pd_ctrl & BIT_LDO_CAMD_PD))
		printk("###---- BIT_LDO_CAMD_PD power on! ----###\n");
	if (!(ldo_pd_ctrl & BIT_LDO_CAMA_PD))
		printk("###---- BIT_LDO_CAMA_PD power on! ----###\n");
	if (!(ldo_pd_ctrl & BIT_LDO_SIM2_PD))
		printk("###---- BIT_LDO_SIM2_PD power on! ----###\n");
	if (!(ldo_pd_ctrl & BIT_LDO_SIM1_PD))
		printk("###---- BIT_LDO_SIM1_PD power on! ----###\n");
	if (!(ldo_pd_ctrl & BIT_LDO_SIM0_PD))
		printk("###---- BIT_LDO_SIM0_PD power on! ----###\n");
#if defined(CONFIG_ADIE_SC2713S)
	if (!(ldo_pd_ctrl & BIT_LDO_SD_PD))
		printk("###---- BIT_LDO_SD_PD power on! ----###\n");
#endif
#if defined(CONFIG_ADIE_SC2723S)
	if (!(ldo_pd_ctrl & BIT_LDO_SDIO_PD))
		printk("###---- BIT_LDO_SD_PD power on! ----###\n");
#endif
#if defined(CONFIG_ADIE_SC2723)
	if (!(ldo_pd_ctrl & BIT_LDO_SDIO_PD))
		printk("###---- BIT_LDO_SD_PD power on! ----###\n");
#endif
#if defined(CONFIG_ARCH_SCX15)
	if (!(ldo_dcdc_pd_ctrl & BIT_BG_PD))
		printk("###---- BIT_BG_PD power on! ----###\n");
	if (!(ldo_dcdc_pd_ctrl & BIT_LDO_CON_PD))
		printk("###---- BIT_LDO_CON_PD power on! ----###\n");
	if (!(ldo_dcdc_pd_ctrl & BIT_LDO_DCXO_PD))
		printk("###---- BIT_LDO_DCXO_PD power on! ----###\n");
	if (!(ldo_dcdc_pd_ctrl & BIT_LDO_EMMCIO_PD))
		printk("###---- BIT_LDO_EMMCIO_PD power on! ----###\n");
	if (!(ldo_dcdc_pd_ctrl & BIT_LDO_EMMCCORE_PD))
		printk("###---- BIT_LDO_EMMCCORE_PD power on! ----###\n");
#endif

#if defined(CONFIG_ARCH_SCX15)
	if (ap_apb_eb & BIT_UART4_EB)
		printk("###---- BIT_UART4_EB set! ----###\n");
	if (ap_apb_eb & BIT_UART3_EB)
		printk("###---- BIT_UART3_EB set! ----###\n");
	if (ap_apb_eb & BIT_UART2_EB)
		printk("###---- BIT_UART2_EB set! ----###\n");
	if (ap_apb_eb & BIT_UART1_EB)
		printk("###---- BIT_UART1_EB set! ----###\n");
	if (ap_apb_eb & BIT_UART0_EB)
		printk("###---- BIT_UART0_EB set! ----###\n");
	if (ap_apb_eb & BIT_I2C4_EB)
		printk("###---- BIT_I2C4_EB set! ----###\n");
	if (ap_apb_eb & BIT_I2C3_EB)
		printk("###---- BIT_I2C3_EB set! ----###\n");
	if (ap_apb_eb & BIT_I2C2_EB)
		printk("###---- BIT_I2C2_EB set! ----###\n");
	if (ap_apb_eb & BIT_I2C1_EB)
		printk("###---- BIT_I2C1_EB set! ----###\n");
	if (ap_apb_eb & BIT_I2C0_EB)
		printk("###---- BIT_I2C0_EB set! ----###\n");
	if (ap_apb_eb & BIT_IIS3_EB)
		printk("###---- BIT_IIS3_EB set! ----###\n");
	if (ap_apb_eb & BIT_IIS2_EB)
		printk("###---- BIT_IIS2_EB set! ----###\n");
	if (ap_apb_eb & BIT_IIS1_EB)
		printk("###---- BIT_IIS1_EB set! ----###\n");
	if (ap_apb_eb & BIT_IIS0_EB)
		printk("###---- BIT_IIS0_EB set! ----###\n");
#endif

#if defined(CONFIG_ARCH_SCX35LT8)
	if (sleep_ctrl & BIT_VCP1_FORCE_LIGHT_SLEEP)
		printk("###---- force VCP1 in light sleep ----###\n");
	if (sleep_ctrl & BIT_VCP0_FORCE_LIGHT_SLEEP)
		printk("###---- force VCP0 in light sleep ----###\n");
	if (sleep_ctrl & BIT_CP1_FORCE_LIGHT_SLEEP)
		printk("###---- force CP1 in light sleep  ----###\n");
	if (sleep_ctrl & BIT_CP0_FORCE_LIGHT_SLEEP)
		printk("###---- force CP0 in light sleep  ----###\n");
	if (sleep_ctrl & BIT_AP_FORCE_LIGHT_SLEEP)
		printk("###---- force AP in light sleep   ----###\n");
	if (sleep_ctrl & BIT_VCP1_LIGHT_SLEEP)
		printk("###---- VCP1 is in light sleep status ----###\n");
	if (sleep_ctrl & BIT_VCP0_LIGHT_SLEEP)
		printk("###--- VCP0 is in light sleep status ----###\n");
	if (sleep_ctrl & BIT_CP1_LIGHT_SLEEP)
		printk("###---- CP1 is in light sleep status ----###\n");
	if (sleep_ctrl & BIT_CP0_LIGHT_SLEEP)
		printk("###---- CP0 is in light sleep status ----###\n");
	if (sleep_ctrl & BIT_AP_LIGHT_SLEEP)
		printk("###---- AP is in light sleep status ----###\n");

	if (apb_pwrstatus0){
		printk("status of power domain 'MM_TOP' 0x%08x\n", (apb_pwrstatus0 & BITS_PD_MM_TOP_STATE(15)) >> 28);
		printk("status of power domain 'GPU_TOP' 0x%08x\n", (apb_pwrstatus0 & BITS_PD_GPU_TOP_STATE(15)) >> 24);
		printk("status of power domain 'AP_SYS' 0x%08x\n", (apb_pwrstatus0 & BITS_PD_AP_SYS_STATE(15)) >> 20);
		printk("status of power domain 'CA53_LIT_C3' 0x%08x\n", (apb_pwrstatus0 & BITS_PD_CA53_LIT_C3_STATE(15)) >> 16);
		printk("status of power domain 'CA53_LIT_C2' 0x%08x\n", (apb_pwrstatus0 & BITS_PD_CA53_LIT_C2_STATE(15)) >> 12);
		printk("status of power domain 'CA53_LIT_C1' 0x%08x\n", (apb_pwrstatus0 & BITS_PD_CA53_LIT_C1_STATE(15)) >> 8);
		printk("status of power domain 'CA53_LIT_C0' 0x%08x\n", (apb_pwrstatus0 & BITS_PD_CA53_LIT_C0_STATE(15)) >> 4);
		printk("status of power domain 'CA53_TOP' 0x%08x\n", apb_pwrstatus0 & BITS_PD_CA53_TOP_STATE(15));
	}
	if (apb_pwrstatus1){
		printk("status of power domain 'CP0_CEVA_1' 0x%08x\n", (apb_pwrstatus1 & BITS_PD_CP0_CEVA_1_STATE(15)) >> 28);
		printk("status of power domain 'CP_CEVA_0' 0x%08x\n", (apb_pwrstatus1 & BITS_PD_CP0_CEVA_0_STATE(15)) >> 24);
		printk("status of power domain 'CP0_GSM_0' 0x%08x\n", (apb_pwrstatus1 & BITS_PD_CP0_GSM_0_STATE(15)) >> 20);
		printk("status of power domain 'CP0_GSM_1' 0x%08x\n", (apb_pwrstatus1 & BITS_PD_CP0_GSM_1_STATE(15)) >> 16);
		printk("status of power domain 'CP0_HU3GE' 0x%08x\n", (apb_pwrstatus1 & BITS_PD_CP0_HU3GE_STATE(15)) >> 12);
		printk("status of power domain 'CP0_ARM9_1' 0x%08x\n", (apb_pwrstatus1 & BITS_PD_CP0_ARM9_1_STATE(15)) >> 8);
		printk("status of power domain 'CP0_ARM9_0' 0x%08x\n", (apb_pwrstatus1 & BITS_PD_CP0_ARM9_0_STATE(15)) >> 4);
		printk("status of power domain 'CP0_TD' 0x%08x\n", apb_pwrstatus1 & BITS_PD_CP0_TD_STATE(15));
	}
	if (apb_pwrstatus2){
		printk("status of power domain 'CP0_PUB_SYS' 0x%08x\n", (apb_pwrstatus2 & BITS_PD_PUB_SYS_STATE(15)) >> 24);
		printk("status of power domain 'CP1_COMWRAP' 0x%08x\n", (apb_pwrstatus2 & BITS_PD_CP1_COMWRAP_STATE(15)) >> 20);
		printk("status of power domain 'CP1_LTE_P2' 0x%08x\n", (apb_pwrstatus2 & BITS_PD_CP1_LTE_P2_STATE(15)) >> 16);
		printk("status of power domain 'CP1_LTE_P1' 0x%08x\n", (apb_pwrstatus2 & BITS_PD_CP1_LTE_P1_STATE(15)) >> 12);
		printk("status of power domain 'CP1_CEVA' 0x%08x\n", (apb_pwrstatus2 & BITS_PD_CP1_CEVA_STATE(15)) >> 8);
		printk("status of power domain 'CP1_CA5' 0x%08x\n", (apb_pwrstatus2 & BITS_PD_CP1_CA5_STATE(15)) >> 4);
		printk("status of power domain 'CA53_LIT_MP4' 0x%08x\n", apb_pwrstatus2 & BITS_PD_CA53_LIT_MP4_STATE(15));
	}
	if (apb_pwrstatus3){
		printk("###---- status of power domain 'MM_CODEC' 0x%08x----###\n", (apb_pwrstatus3 & BITS_PD_MM_CODEC_STATE(15)) >> 28);
		printk("###---- status of power domain 'CA53_BIG_C3' 0x%08x----###\n", (apb_pwrstatus3 & BITS_PD_CA53_BIG_C3_STATE(15)) >> 24);
		printk("###---- status of power domain 'CA53_BIG_C2' 0x%08x----###\n", (apb_pwrstatus3 & BITS_PD_CA53_BIG_C2_STATE(15)) >> 20);
		printk("###---- status of power domain 'CA53_BIG_C1' 0x%08x----###\n", (apb_pwrstatus3 & BITS_PD_CA53_BIG_C1_STATE(15)) >> 16);
		printk("###---- status of power domain 'CA53_BIG_C0' 0x%08x----###\n", (apb_pwrstatus3 & BITS_PD_CA53_BIG_C0_STATE(15)) >> 12);
		printk("###---- status of power domain 'CA53_BIG_MP4' 0x%08x----###\n", (apb_pwrstatus3 & BITS_PD_CA53_BIG_MP4_STATE(15)) >> 8);
		printk("###---- status of power domain 'GPU_C1' 0x%08x----###\n", (apb_pwrstatus3 & BITS_PD_GPU_C1_STATE(15)) >> 4);
		printk("###---- status of power domain 'GPU_C0' 0x%08x----###\n", apb_pwrstatus3 & BITS_PD_GPU_C0_STATE(15));
	}

	if (mpll_cfg1 & BIT_MPLL_LOCK_DONE)
		printk("###---- MPLL lock_none bit is set ----###\n");
	if (mpll_cfg1 & BIT_MPLL_DIV_S)
		printk("###---- MPLL feedback is fractional divider ----###");
	if (mpll_cfg1 & BIT_MPLL_MOD_EN)
		printk("###---- MPLL modulator MOD enable ----###\n");
	if (mpll_cfg1 & BIT_MPLL_SDM_EN)
		printk("###---- MPLL modulator SDM enable ----###\n");

	if (dpll_cfg1 & BIT_DPLL_LOCK_DONE)
		printk("###---- DPLL lock_node bit is set ----###\n");
	if (dpll_cfg1 & BIT_DPLL_DIV_S)
		printk("###---- DPLL feedback is fractional divider ----###\n");
	if (dpll_cfg1 & BIT_DPLL_MOD_EN)
		printk("###---- DPLL modulator MOD enable ----###\n");
	if (dpll_cfg1 & BIT_DPLL_SDM_EN)
		printk("###---- DPLL modulator SDM enable ----###\n");
#endif


}

static int print_thread(void * data)
{
	while(1){
		wake_lock(&messages_wakelock);
		if (print_thread_enable)
			print_debug_info();
		has_wake_lock(WAKE_LOCK_SUSPEND);
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
		printk("!!!powermanager Failed to create debugfs directory\n");
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
	struct task_struct * task;
	wake_lock_init(&messages_wakelock, WAKE_LOCK_SUSPEND,
			"pm_message_wakelock");
	task = kthread_create(print_thread, NULL, "pm_print");
	if (task == 0) {
		printk("Can't crate power manager print thread!\n");
	}else
		wake_up_process(task);
#endif
	debugfs_init();
}
void pm_debug_clr(void)
{
	if(dentry_debug_root != NULL)
		debugfs_remove_recursive(dentry_debug_root);
}
void pm_debug_dump_ahb_glb_regs(void){}
void pm_debug_save_ahb_glb_regs(void){}
#define WDG_BASE		(ANA_WDG_BASE)
#define SPRD_ANA_BASE		(ANA_CTL_GLB_BASE)

#define WDG_LOAD_LOW        (WDG_BASE + 0x00)
#define WDG_LOAD_HIGH       (WDG_BASE + 0x04)
#define WDG_CTRL            (WDG_BASE + 0x08)
#define WDG_INT_CLR         (WDG_BASE + 0x0C)
#define WDG_INT_RAW         (WDG_BASE + 0x10)
#define WDG_INT_MSK         (WDG_BASE + 0x14)
#define WDG_CNT_LOW         (WDG_BASE + 0x18)
#define WDG_CNT_HIGH        (WDG_BASE + 0x1C)
#define WDG_LOCK            (WDG_BASE + 0x20)
#define WDG_IRQ_LOW            (WDG_BASE + 0x2c)
#define WDG_IRQ_HIGH            (WDG_BASE + 0x30)

#define WDG_INT_EN_BIT          BIT(0)
#define WDG_CNT_EN_BIT          BIT(1)
#define WDG_NEW_VER_EN		BIT(2)
#define WDG_INT_CLEAR_BIT       BIT(0)
#define WDG_RST_CLEAR_BIT       BIT(3)
#define WDG_LD_BUSY_BIT         BIT(4)
#define WDG_RST_EN_BIT		BIT(3)


#define ANA_REG_BASE            SPRD_ANA_BASE	/*  0x82000600 */


#define WDG_NEW_VERSION
#define ANA_RST_STATUS          (ANA_REG_BASE + 0xE8)
#define ANA_AGEN                (ANA_REG_BASE + 0x00)
#define ANA_RTC_CLK_EN		(ANA_REG_BASE + 0x08)

#define AGEN_WDG_EN             BIT(2)
#define AGEN_RTC_WDG_EN         BIT(2)

#define WDG_CLK                 32768
#define WDG_UNLOCK_KEY          0xE551

#define ANA_WDG_LOAD_TIMEOUT_NUM    (10000)

#ifdef WDG_NEW_VERSION
#define WDG_LOAD_TIMER_VALUE(value) \
        do{\
                    sci_adi_raw_write( WDG_LOAD_HIGH, (uint16_t)(((value) >> 16 ) & 0xffff));\
                    sci_adi_raw_write( WDG_LOAD_LOW , (uint16_t)((value)  & 0xffff) );\
        }while(0)
#define WDG_LOAD_TIMER_INT_VALUE(value) \
        do{\
                    sci_adi_raw_write( WDG_IRQ_HIGH, (uint16_t)(((value) >> 16 ) & 0xffff));\
                    sci_adi_raw_write( WDG_IRQ_LOW , (uint16_t)((value)  & 0xffff) );\
        }while(0)

#else

#define WDG_LOAD_TIMER_VALUE(value) \
        do{\
                    uint32_t   cnt          =  0;\
                    sci_adi_raw_write( WDG_LOAD_HIGH, (uint16_t)(((value) >> 16 ) & 0xffff));\
                    sci_adi_raw_write( WDG_LOAD_LOW , (uint16_t)((value)  & 0xffff) );\
                    while((sci_adi_read(WDG_INT_RAW) & WDG_LD_BUSY_BIT) && ( cnt < ANA_WDG_LOAD_TIMEOUT_NUM )) cnt++;\
        }while(0)
#endif
/* use ana watchdog to wake up */
void pm_debug_set_apwdt(void)
{
	uint32_t cnt = 0;
	uint32_t ms = 5000;
	cnt = (ms * WDG_CLK) / 1000;
	sci_glb_set(INT_IRQ_ENB, BIT(11));
	/*enable interface clk*/
	sci_adi_set(ANA_AGEN, AGEN_WDG_EN);
	/*enable work clk*/
	sci_adi_set(ANA_RTC_CLK_EN, AGEN_RTC_WDG_EN);
	sci_adi_raw_write(WDG_LOCK, WDG_UNLOCK_KEY);
	sci_adi_set(WDG_CTRL, WDG_NEW_VER_EN);
	WDG_LOAD_TIMER_VALUE(0x80000);
	WDG_LOAD_TIMER_INT_VALUE(0x40000);
	sci_adi_set(WDG_CTRL, WDG_CNT_EN_BIT | WDG_INT_EN_BIT| WDG_RST_EN_BIT);
	sci_adi_raw_write(WDG_LOCK, (uint16_t) (~WDG_UNLOCK_KEY));
	sci_adi_set(ANA_REG_INT_EN, BIT(3));
#if 0
	do{
		udelay(1000);
		printk("INTC1 mask:0x%08x raw:0x%08x en:0x%08x\n", __raw_readl(INTCV1_IRQ_MSKSTS),__raw_readl(INTCV1_IRQ_RAW), __raw_readl(INTCV1_IRQ_EN));
		printk("INT mask:0x%08x raw:0x%08x en:0x%08x ana 0x%08x\n", __raw_readl(INT_IRQ_STS),__raw_readl(INT_IRQ_RAW), __raw_readl(INT_IRQ_ENB), sci_adi_read(ANA_REG_INT_MASK_STATUS));
		printk("ANA mask:0x%08x raw:0x%08x en:0x%08x\n", sci_adi_read(ANA_REG_INT_MASK_STATUS), sci_adi_read(ANA_REG_INT_RAW_STATUS), sci_adi_read(ANA_REG_INT_EN));
		printk("wdg cnt low 0x%08x high 0x%08x\n", sci_adi_read(WDG_CNT_LOW), sci_adi_read(WDG_CNT_HIGH));
	}while(0);
#endif
}
void pm_debug_clr_apwdt(void)
{
	//sci_adi_raw_write(WDG_LOCK, WDG_UNLOCK_KEY);
	printk("watchdog int raw status: 0x%x\n", sci_adi_read(WDG_INT_RAW));
	printk("watchdog int msk status: 0x%x\n", sci_adi_read(WDG_INT_MSK));
	sci_adi_set(WDG_INT_CLR, WDG_INT_CLEAR_BIT | WDG_RST_CLEAR_BIT);
	printk("watchdog int raw status: 0x%x\n", sci_adi_read(WDG_INT_RAW));
	printk("watchdog int msk status: 0x%x\n", sci_adi_read(WDG_INT_MSK));
	sci_adi_clr(WDG_CTRL, WDG_CNT_EN_BIT | WDG_RST_EN_BIT);
	//sci_adi_raw_write(WDG_LOCK, (uint16_t) (~WDG_UNLOCK_KEY));
}

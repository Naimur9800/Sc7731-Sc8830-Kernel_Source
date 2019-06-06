/*
 * Support for power management features of the SPRD iSoC
 *
 * Based on arch/x86/kernel/acpi/sleep.c.
 *
 * Copyright (C) 2016 Intel Corporation
 * Author Bin Gao <bin.gao@intel.com>
 *
 * This file is released under the GPLv2.
 */

#include <linux/pm.h>
#include <linux/suspend.h>
#include <asm/mwait.h>
#include <asm/mv/mobilevisor.h>
#include <asm/mv/mv_gal.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/init.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/regmap.h>
#include "sleep.h"
#if defined(CONFIG_X86_64_SPRD_DEEPSLEEP)
#ifdef CONFIG_X86_SPRD_ISHARKL2_LPC
#include "isharkl2/pm_x86_64.h"
#elif defined(CONFIG_X86_SPRD_IWHALE2_LPC)
#include "iwhale2/pm_x86_64.h"
#endif
#endif
/*
 * The physical address of wakeup start point (wakeup_start) is programed
 * to SRAM which is on during suspend.
 */
#define WAKEUP_ADDR	0xE6001FE0

/* Registers needed for S3 init. These belong to SoC PMU and are required
 * to show the SoC our intentions to put the BIA to S3
 */
/* PMU Base Address(SoC) */
/* #define PMU_BASE	0xE42B0000 */

/* ACPI PM Registers */
/* #define ACPI_PM_BASE	0xFEC40000 */

/* S3 Hint */
#define C6FS_HINT	0x52

struct wakeup_param {
	u64	wakeup_long64_direct;
	u64	trampoline_pgd;
	u64	wakeup_32;
	/* Resume entry address is passed from uboot to kernel, the address
	 * will finally write to aon_apb_dat1 reg, boot ROM will jmp there
	 * on s3 wake up case.
	 */
	u32	resume_entry;
	u32	unused;
};

static struct wakeup_param *wake_parm;

static struct device_node *isoc_pm_np;
static struct regmap *pmu_addr;
static void __iomem *acpi_pm_addr;
static struct regmap *aon_apb_addr;

void (*sprd_soc_deepsleep_enter)(unsigned int cpu) = NULL;
void (*sprd_soc_wakeup_enter)(void) = NULL;

/*
 * isoc_do_sleep - execute actual hardware level steps to enter sleep state
 * @state: Sleep state to enter.
 *
 */
asmlinkage __visible int isoc_do_sleep(u8 state)
{
	unsigned int mwait_ptr;
	unsigned long long msr_bits;
	u32 value;
	u32 cpu = smp_processor_id();

	if (cpu) {
		__WARN();
		return -EBUSY;
	}

	pr_info("cpu%d enter %s\n", cpu, __func__);
	if (sprd_soc_deepsleep_enter == NULL)
		pr_info("######sprd deepsleep handle is NULL...######\n");
	else {
		pr_info("######sprd deepsleep entry...######\n");
		(*sprd_soc_deepsleep_enter)(cpu);
	}
	/*
	* In the case of mobilevisor call VMM hypercall
	*/
	if (is_x86_mobilevisor()) {
		pr_debug("Hypercall vmm suspend\n");
		mv_idle();
		pr_debug("Hypercall vmm suspend done\n");
		return 0;
	}

	rdmsrl(MSR_NHM_SNB_PKG_CST_CFG_CTL, msr_bits);
	msr_bits |= BIA_EN_MANUAL_PERFCTR
		 |  BIA_DYN_L2_REDUCTION
		 |  BIA_LOWER_CST_CONFIG;
	wrmsrl(MSR_NHM_SNB_PKG_CST_CFG_CTL, msr_bits);

	/* Program PM1_CNT (EN and SUSPEND_TYPE bits) */
	value = ioread32(acpi_pm_addr + PM1_CNT);
	value &= SLEEP_TYPE_MASK;
	value |= SLEEP_TYPE_S3 | SLEEP_ENABLE;
	iowrite32(value, acpi_pm_addr + PM1_CNT);

	/* Read original address incase suspend abort */
	regmap_read(aon_apb_addr,
		REG_AON_APB_BIA_REG_DAT1, &value);
	regmap_write(aon_apb_addr,
		REG_AON_APB_BIA_REG_DAT1, wake_parm->resume_entry);

	pr_debug("%s resume entry point 0x%08x\n", __func__,
			wake_parm->resume_entry);
	pr_info("%s ###start entry mwait...###\n", __func__);

	clflush(&mwait_ptr);
	__monitor(&mwait_ptr, 0, 0);

	__mwait(C6FS_HINT, 1);

	/*
	 * If suspend aborted due to an interrupt or cores busy
	 * We should continue from this point.
	 */

	/* Clear PM1_CNT since we've failed */
	iowrite32(0, acpi_pm_addr + PM1_CNT);
	/* Write the unexpected wake address back */
	regmap_write(aon_apb_addr,
		REG_AON_APB_BIA_REG_DAT1, value);

	pr_debug("Suspend aborted.\n");

	return -EBUSY;
}

static void isoc_suspend_lowlevel(void)
{
	/* Call suspend_lowlevel for native and mobilevisor */
	do_suspend_lowlevel();
}

static void program_cstate_cfg_ctl(void *info)
{
	u64 msr_bits;

	/* check C-State Configuration Control */
	/* Reprogram C-State Configuration Control */
	rdmsrl(MSR_NHM_SNB_PKG_CST_CFG_CTL, msr_bits);
	msr_bits |= BIA_EN_MANUAL_PERFCTR
		 |  BIA_DYN_L2_REDUCTION
		 |  BIA_LOWER_CST_CONFIG;
	wrmsrl(MSR_NHM_SNB_PKG_CST_CFG_CTL, msr_bits);
}

static void enable_lbr(void *info)
{
	u64 msr_bits;

	/* enable lbr */
	rdmsrl(MSR_IA32_DEBUGCTLMSR, msr_bits);
	msr_bits |= DEBUGCTLMSR_LBR;
	wrmsrl(MSR_IA32_DEBUGCTLMSR, msr_bits);
}
static int isoc_power_state_enter(suspend_state_t pm_state)
{

	if (pm_state != PM_SUSPEND_MEM)
		return -EINVAL;
	/*reset the value of stored irq status last time */
	hard_irq_reset();

	/* starting lowlevel suspend */
	isoc_suspend_lowlevel();

	pr_info("mwait exit\n");
	if (sprd_soc_wakeup_enter == NULL)
		pr_info("######sprd wakeup handle is NULL...######\n");
	else {
		pr_info("######sprd wakeup entry...######\n");
		(*sprd_soc_wakeup_enter)();
	}
	/* Resume path starts here */
	return 0;
}

static int isoc_power_state_valid(suspend_state_t pm_state)
{
	/* We support suspend-to-RAM only */
	return pm_state == PM_SUSPEND_MEM;
}

static void isoc_power_state_end(void)
{
	/* We need to restore C-states MSR after coming from S3 */
	on_each_cpu(program_cstate_cfg_ctl, NULL, 1);
	/* restore lbr enable MSR after coming from s3 */
	if (!is_x86_mobilevisor())
		on_each_cpu(enable_lbr, NULL, 1);
}

static const struct platform_suspend_ops isoc_suspend_ops = {
	.valid = isoc_power_state_valid,
	.enter = isoc_power_state_enter,
	.end = isoc_power_state_end,
};


/*
 * A 1:1 mapping table is required for swithing the processor from 32b
 * to 64 bit mode. Create a simple one here for the job.
 */
#ifdef CONFIG_X86_64
static void build_pagetable(u32 *pgtable)
{
	int i;

	memset(pgtable, '\0', PAGE_SIZE * 6);

	/* Level 4 needs a single entry */
	pgtable[0] = __pa(&pgtable[1024]) + 7;

	/* Level 3 has one 64-bit entry for each GiB of memory */
	for (i = 0; i < 4; i++) {
		pgtable[1024 + i * 2] = (u64)__pa(&pgtable[2048])
					+ 0x1000 * i + 7;
	}

	/* Level 2 has 2048 64-bit entries, each repesenting 2MiB */
	for (i = 0; i < 2048; i++)
		pgtable[2048 + i * 2] = 0x183 + (i << 21UL);
}
#endif

#define PXP_MODEL_R0_99 0
static void iwhale_pm_reginit(void)
{
	u32 rdata;

	if (!pmu_addr)
		return;
	/* s0ix init */
	/*open pd_bia_s0ix_slp_ena and pd_bia_auto_shutdown_en*/
	regmap_read(pmu_addr, 0x64, &rdata);
	regmap_write(pmu_addr, 0x64, rdata | 0x21000000);
	/*enable bia deepsleep for SPRD PMU*/
	regmap_read(pmu_addr, 0x1f0, &rdata);
	regmap_write(pmu_addr, 0x1f0, rdata | 0x1);

#if PXP_MODEL_R0_99
	/* registers for r0.99 */
	/*enable s0ix mode for SPRD PMU*/
	regmap_write(pmu_addr, 0x6c, 0x21209804);
	regmap_read(pmu_addr, 0x1f0, &rdata);
	/*enable deep sleep in PMU*/
	regmap_write(pmu_addr, 0x1f0, rdata | 0x1);
	regmap_read(pmu_addr, 0x24c, &rdata);
	/*enable bia_tmr in PMU*/
	regmap_write(pmu_addr, 0x24c, rdata | (0x1<<9));
#endif
	on_each_cpu(program_cstate_cfg_ctl, NULL, 1);
	/* enable lbr */
	if (!is_x86_mobilevisor())
		on_each_cpu(enable_lbr, NULL, 1);
}

static void isoc_pm_init_regmap(void)
{
	struct device_node *acpi_pm_node;
	isoc_pm_np = of_find_node_by_name(NULL, "soc-pm");
	if (!isoc_pm_np)
		pr_err("%s, failed to find isoc_pm_np\n", __func__);

	pmu_addr = syscon_regmap_lookup_by_phandle(isoc_pm_np,
						"sprd,sys-aon-pwu-apb");
	if (IS_ERR(pmu_addr))
		pr_err("%s,failed to find pmu_addr\n", __func__);

	acpi_pm_node = of_find_node_by_name(NULL,
						"acpi-pm");
	if (!acpi_pm_node)
		pr_err("%s,failed to find acpi_pm_node\n", __func__);
	else {
		struct resource res;

		if (of_address_to_resource(acpi_pm_node, 0, &res)) {
			pr_err("%s,failed to find regs in acpi_pm_node\n",
								__func__);
		} else {
			acpi_pm_addr = ioremap_nocache(res.start,
						resource_size(&res));

			if (!acpi_pm_addr)
				pr_err("%s,failed to ioremap acpi_pm\n",
								__func__);
		}
	}
	aon_apb_addr = syscon_regmap_lookup_by_phandle(isoc_pm_np,
						"sprd,sys-aon-apb");

	if (IS_ERR(aon_apb_addr))
		pr_err("%s, failed to find aon_apb_addr\n", __func__);
}

static int __init isoc_pm_init(void)
{
#ifdef CONFIG_X86_64
	/*
	 * Save the required MSRs and PAGE directory to use when switching to
	 * 64 bit mode. This is done only once during boot.
	 */
	u64 *trampoline_pgd = (void *)&wakeup_trampoline_pgd;
	u64 efer, misc;

	rdmsrl(MSR_EFER, efer);
	wakeup_trampoline_header.efer = efer & ~EFER_LMA;

	rdmsrl(MSR_IA32_MISC_ENABLE, misc);
	wakeup_trampoline_header.misc = misc;
	build_pagetable((u32 *)trampoline_pgd);
	trampoline_pgd[511] = init_level4_pgt[511].pgd;
#endif
	isoc_pm_init_regmap();
	wake_parm = (struct wakeup_param *)ioremap_nocache(WAKEUP_ADDR, 32);

	if (!wake_parm) {
		pr_err("ioremap_nocache() failed, suspend won't work.\n");
		return -EIO;
	}

	iwhale_pm_reginit();
	/*
	 * Program wakeup_long64_direct in	[0] for 64b entry from SRAM
	 * Program trampoline_pgd in		[1] for 64b entry from SRAM
	 * Program wakeup address in		[2] for 32b entry from SRAM
	 * SRAM code to jump to kernel entry point on Resume
	 */
#ifdef CONFIG_X86_64
	wake_parm->wakeup_long64_direct = (u64) __pa(wakeup_long64_direct);
	wake_parm->trampoline_pgd = (u64) __pa(&wakeup_trampoline_pgd);
#endif
	wake_parm->wakeup_32 = (u64) __pa(wakeup_start_32);

	suspend_set_ops(&isoc_suspend_ops);

	sc_light_init();
	return 0;
}
arch_initcall(isoc_pm_init);

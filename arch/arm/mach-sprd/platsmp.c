/* linux/arch/arm/mach-sp9830/platsmp.c
 *
 * Copyright (c) 2010-2012 Spreadtrum Co., Ltd.
 *		http://www.spreadtrum.com
 * Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Cloned from linux/arch/arm/mach-vexpress/platsmp.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/sprd_arm_pmu.h>

#include <asm/cacheflush.h>
#include <asm/smp_plat.h>
#include <asm/mach-types.h>
#include <asm/smp_scu.h>
#include <asm/unified.h>
#include <asm/atomic.h>

#include "platsmp.h"
#include "hotplug.h"

#define SCI_ADDR(_b_, _o_)	((u32)(_b_) + (_o_))
#define BIT_PD_CA7_C3_AUTO_SHUTDOWN_EN	(BIT(24))
#define BIT_PD_CA7_C3_FORCE_SHUTDOWN	(BIT(25))
#define BITS_PD_CA7_C3_PWR_ON_DLY(_X_)	\
		((_X_) << 16 & (BIT(16) | \
		BIT(17) | BIT(18) | BIT(19)| \
		BIT(20) | BIT(21) | BIT(22) | \
		BIT(23)))
#define BITS_PD_CA7_C3_PWR_ON_SEQ_DLY(_X_) \
		((_X_) << 8 & (BIT(8) | BIT(9) \
		| BIT(10) | BIT(11) | BIT(12) \
		| BIT(13) | BIT(14) | BIT(15)))
#define BITS_PD_CA7_C3_ISO_ON_DLY(_X_) \
		((_X_) & (BIT(0) | BIT(1) | \
		BIT(2) | BIT(3) | BIT(4) | \
		BIT(5) | BIT(6) | BIT(7)))

#define SPRD_UP_FLAG0	(0x63507530)
#define SPRD_UP_FLAG1	(0x63507531)
#define SPRD_UP_FLAG2	(0x63507532)
#define SPRD_UP_FLAG3	(0x63507533)

static DEFINE_SPINLOCK(boot_lock);
unsigned int sprd_boot_magnum = 0xbadf1a90;

static unsigned int g_sprd_up_flag[4] = {
	SPRD_UP_FLAG0,
	SPRD_UP_FLAG1,
	SPRD_UP_FLAG2,
	SPRD_UP_FLAG3
};
static void __iomem *sprd_pmu_base_vaddr;
static void __iomem *sprd_ahb_base_vaddr;

static unsigned int sci_glb_read(unsigned long reg, unsigned int msk)
{
	return readl_relaxed((void *)reg) & msk;
}

static int sci_glb_write(unsigned long reg, unsigned int val, unsigned int msk)
{
	writel_relaxed((readl_relaxed((void *)reg) & ~msk) | val, (void *)reg);

	return 0;
}

static void __iomem *sprd_smp_get_iomem(struct device_node *from,
					const char *type,
					const char *compatible)
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


static int poweron_cpus(int cpu)
{
	u32 poweron, val;
	int ret = 0;
	u32 REG_PMU_APB_PD_CA7_C1_CFG;
	u32 REG_PMU_APB_PD_CA7_C2_CFG;
	u32 REG_PMU_APB_PD_CA7_C3_CFG;
	u32 REG_AP_AHB_CA7_RST_SET;

	if (sprd_pmu_base_vaddr == NULL)
		sprd_pmu_base_vaddr = sprd_smp_get_iomem(NULL, NULL, "sprd,sys-pmu-apb");
	if (sprd_pmu_base_vaddr == NULL) {
		pr_err("%s: get pmu io memory error\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}
	if (sprd_ahb_base_vaddr == NULL)
		sprd_ahb_base_vaddr = sprd_smp_get_iomem(NULL, NULL, "sprd,sys-ap-ahb");
	if (sprd_ahb_base_vaddr == NULL) {
		pr_err("%s: get ahb io memory error\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	REG_PMU_APB_PD_CA7_C1_CFG = SCI_ADDR(sprd_pmu_base_vaddr, 0x08);
	REG_PMU_APB_PD_CA7_C2_CFG = SCI_ADDR(sprd_pmu_base_vaddr, 0x0C);
	REG_PMU_APB_PD_CA7_C3_CFG = SCI_ADDR(sprd_pmu_base_vaddr, 0x10);
	REG_AP_AHB_CA7_RST_SET = SCI_ADDR(sprd_ahb_base_vaddr, 0x08);

	if (cpu == 1)
		poweron = REG_PMU_APB_PD_CA7_C1_CFG;
	else if (cpu == 2)
		poweron = REG_PMU_APB_PD_CA7_C2_CFG;
	else if (cpu == 3)
		poweron = REG_PMU_APB_PD_CA7_C3_CFG;
	else {
		ret = EINVAL;
		goto exit;
	}

	val = sci_glb_read(REG_AP_AHB_CA7_RST_SET, -1UL);
	val |= (1 << cpu);
	sci_glb_write(REG_AP_AHB_CA7_RST_SET, val, -1UL);

	val =
	    BITS_PD_CA7_C3_PWR_ON_DLY(4) | BITS_PD_CA7_C3_PWR_ON_SEQ_DLY(4) |
	    BITS_PD_CA7_C3_ISO_ON_DLY(4);
	sci_glb_write(poweron, val, -1UL);

	val =
	    (BIT_PD_CA7_C3_AUTO_SHUTDOWN_EN | readl_relaxed((void *)poweron)) &
	    ~(BIT_PD_CA7_C3_FORCE_SHUTDOWN);
	sci_glb_write(poweron, val, -1UL);
	dsb();
	udelay(1000);

	while ((readl_relaxed((void *)poweron) & BIT_PD_CA7_C3_FORCE_SHUTDOWN)
	       || !(readl_relaxed((void *)REG_AP_AHB_CA7_RST_SET) & (1 << cpu))
		   ) {
		pr_debug("??? %s set regs failed ???\n", __func__);
		writel((readl_relaxed((void *)REG_AP_AHB_CA7_RST_SET) |
			(1 << cpu)), (void *)REG_AP_AHB_CA7_RST_SET);
		val =
		    (BIT_PD_CA7_C3_AUTO_SHUTDOWN_EN |
		     readl_relaxed((void *)poweron)) &
		    ~(BIT_PD_CA7_C3_FORCE_SHUTDOWN);
		writel(val, (void *)poweron);
		dmb();
		udelay(500);
	}
	writel((readl_relaxed((void *)REG_AP_AHB_CA7_RST_SET) & ~(1 << cpu)),
	       (void *)REG_AP_AHB_CA7_RST_SET);
	dsb();

exit:

	return ret;
}


static int boot_secondary_cpus(int cpu_id, u32 paddr)
{
	u32 HOLDING_PEN_VADDR;
	u32 CPU_JUMP_VADDR;

	if (sprd_ahb_base_vaddr == NULL)
		sprd_ahb_base_vaddr = sprd_smp_get_iomem(NULL, NULL, "sprd,sys-ap-ahb");
	if (sprd_ahb_base_vaddr == NULL) {
		pr_err("%s: get ahb io memory error\n", __func__);
		return -ENOMEM;
	}

	if (cpu_id < 1 || cpu_id > 3)
		return -EINVAL;

	HOLDING_PEN_VADDR = SCI_ADDR(sprd_ahb_base_vaddr, 0x14);
	CPU_JUMP_VADDR = SCI_ADDR(sprd_ahb_base_vaddr, 0x18);

	writel(paddr, (void *)(CPU_JUMP_VADDR + (cpu_id << 2)));
	writel(readl((void *)HOLDING_PEN_VADDR) | (1 << cpu_id),
	       (void *)HOLDING_PEN_VADDR);
	dsb();

	poweron_cpus(cpu_id);
	sprd_pmu_enable_irq(cpu_id);

	return 0;
}

/*
 * Write pen_release in a way that is guaranteed to be visible to all
 * observers, irrespective of whether they're taking part in coherency
 * or not.  This is necessary for the hotplug code to work reliably.
 */
static void write_pen_release(int val)
{
	pen_release = val;
	/* memory barrier */
	smp_wmb();
	__cpuc_flush_dcache_area((void *)&pen_release, sizeof(pen_release));
	outer_clean_range(__pa(&pen_release), __pa(&pen_release + 1));
}

static void sprd_secondary_init(unsigned int cpu)
{
	/*
	 * let the primary processor know we're out of the
	 * pen, then head off into the C entry point
	 */
	write_pen_release(g_sprd_up_flag[cpu]);

	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}

static int sprd_boot_secondary(unsigned int cpu,
					 struct task_struct *idle)
{
	unsigned long timeout;
	int ret;
	unsigned int val = sprd_boot_magnum;

	/*
	 * Set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	/*
	 * The secondary processor is waiting to be released from
	 * the holding pen - release it, then wait for it to flag
	 * that it has been released by resetting pen_release.
	 *
	 */
	val |= (cpu_logical_map(cpu) & 0x0000000f);
	write_pen_release((int)val);

	ret = boot_secondary_cpus(cpu, virt_to_phys(sci_secondary_startup));
	if (ret < 0)
		pr_warn("SMP: boot_secondary(%u) error\n", cpu);

	dsb_sev();
	timeout = jiffies + (1 * HZ);
	while (time_before(jiffies, timeout)) {
		/* memory barrier */
		smp_rmb();
		if (pen_release == g_sprd_up_flag[cpu])
			break;

		udelay(10);
		dsb_sev();
	}

	spin_unlock(&boot_lock);

	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	if (pen_release != g_sprd_up_flag[cpu])
		pr_err("[%s]pen release error\n", __func__);

	return pen_release != g_sprd_up_flag[cpu] ? -ENOSYS : 0;
}

static void sprd_smp_prepare_cpus(unsigned int max_cpus)
{
	int cpu_count = 0;
	int cpu;

	/*
	 * Initialise the present map, which describes the set of CPUs actually
	 * populated at the present time.
	 */
	for_each_possible_cpu(cpu) {
		struct device_node *np;
		u32 reg_val;

		np = of_get_cpu_node(cpu, NULL);
		if (!np)
			continue;
		if (of_property_read_u32(np, "reg", &reg_val))
			continue;

		if (cpu_count < max_cpus) {
			set_cpu_present(cpu, true);
			cpu_count++;
		}
	}
}

static struct smp_operations sprd_smp_sc9830_ops __initdata = {
	.smp_secondary_init = sprd_secondary_init,
	.smp_prepare_cpus = sprd_smp_prepare_cpus,
	.smp_boot_secondary = sprd_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_kill = sprd_cpu_kill,
	.cpu_disable = sprd_cpu_disable,
	.cpu_die = sprd_cpu_die,
#endif
};

CPU_METHOD_OF_DECLARE(sprd_sc9830_smp, "sprd,sc9830-smp", &sprd_smp_sc9830_ops);

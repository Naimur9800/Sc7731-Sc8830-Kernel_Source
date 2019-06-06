/*
 * Copyright (C) 2016 Spreadtrum Communications Inc.
 *
 * Contact: wei.qiao <wei.qiao@spreadtrum.com>
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/of.h>
#include <linux/of_address.h>

static void __iomem *whale2_pmuwakeup_addr;

static void whale2_pmu_set_wake(struct irq_data *data)
{
	unsigned int val = 0;
	int irq = data->irq - 32;

	if (irq >= 0) {
		val = readl_relaxed(whale2_pmuwakeup_addr + (irq / 32) * 4);
		val |= 1 << (irq % 32);
		writel_relaxed(val, whale2_pmuwakeup_addr + (irq / 32) * 4);
	}
}

static void whale2_pmu_set_unwake(struct irq_data *data)
{
	unsigned int val = 0;
	int irq = data->irq - 32;

	if (irq >= 0) {
		val = readl_relaxed(whale2_pmuwakeup_addr + (irq / 32) * 4);
		val &= ~(1 << (irq % 32));
		writel_relaxed(val, whale2_pmuwakeup_addr + (irq / 32) * 4);
	}
}

static int whale2_pmuwakeup_of_init(struct device_node *node,
				    struct device_node *parent)
{
	whale2_pmuwakeup_addr = of_iomap(node, 0);
	if (whale2_pmuwakeup_addr) {
		/* Platform-specific GIC callback */
		gic_arch_extn.irq_unmask = whale2_pmu_set_wake;
		gic_arch_extn.irq_mask = whale2_pmu_set_unwake;
		pr_notice("whale2 pmuwakeup vaddr = 0x%lx\n",
			  (unsigned long)whale2_pmuwakeup_addr);
	} else
		pr_err("can not get whale2 pmuwakeup phyaddr!\n");

	return 0;
}

/*
 * We cannot use the IRQCHIP_DECLARE macro that lives in
 * drivers/irqchip, so we're forced to roll our own. Not very nice.
 */
OF_DECLARE_2(irqchip, whale2_pmuwakeup_chip, "sprd,whale2-pmuwakeup-chip",
	     whale2_pmuwakeup_of_init);

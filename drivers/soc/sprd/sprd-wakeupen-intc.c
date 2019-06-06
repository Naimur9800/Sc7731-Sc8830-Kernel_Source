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

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>

#define	SPRD_INTC_IRQ_MSK		0x0000
#define	SPRD_INTC_IRQ_RAW		0x0004
#define	SPRD_INTC_IRQ_EN		0x0008
#define	SPRD_INTC_IRQ_DIS		0x000C
#define	SPRD_INTC_IRQ_SOFT		0x0010
#define	SPRD_INTC_FIQ_MSK		0x0020
#define	SPRD_INTC_FIQ_RAW		0x0024
#define	SPRD_INTC_FIQ_EN		0x0028
#define	SPRD_INTC_FIQ_DIS		0x002C

static int sprd_common_wakeup_of_init(struct device_node *node,
				      struct device_node *parent)
{
	int i = 0;
	void __iomem *addr = NULL;

	for (i = 0; i < 4; i++) {
		addr = of_iomap(node, i);
		if (addr) {
			writel_relaxed(~0, addr + SPRD_INTC_IRQ_EN);
			writel_relaxed(~0, addr + SPRD_INTC_FIQ_DIS);
			pr_debug("%s:intc%d vaddr = 0x%lx\n", __func__, i,
				 (unsigned long)addr);
		} else
			pr_notice("can not get intc%d phyaddr!\n", i);
	}

	addr = of_iomap(node, 4);
	if (addr) {
		writel_relaxed(~0, addr + SPRD_INTC_IRQ_EN);
		writel_relaxed(~0, addr + SPRD_INTC_FIQ_DIS);
		pr_debug("%s:aon intc%d vaddr = 0x%lx\n", __func__, i,
			 (unsigned long)addr);
	} else
		pr_notice("can not get aon intc phyaddr!\n");

	return 0;
}

static int whale2_wakeupen_of_init(struct device_node *node,
				   struct device_node *parent)
{
	int i = 0;
	void __iomem *addr;

	addr = of_iomap(node, 0);
	if (addr) {
		for (i = 0; i < 6; i++)
			writel_relaxed(~0, addr + i * 4);
		pr_notice("whale2 wakeup vaddr = 0x%lx\n", (unsigned long)addr);
	} else
		pr_err("can not get whale2 wakeup phyaddr!\n");

	writel_relaxed(0xa4000008, addr);
	writel_relaxed(0x401606d0, addr + 4);
	writel_relaxed(0x0423bc0c, addr + 8);
	writel_relaxed(0x00000000, addr + 12);
	writel_relaxed(0x0001e000, addr + 16);
	writel_relaxed(0x00000084, addr + 20);
	return 0;
}

static int iwhale2_wakeupen_of_init(struct device_node *node,
				   struct device_node *parent)
{
	int i = 0;
	void __iomem *addr;

	addr = of_iomap(node, 0);
	if (addr) {
		for (i = 0; i < 4; i++)
			writel_relaxed(~0, addr + i * 4);
		pr_notice("iwhale2 wakeup vaddr=0x%lx\n", (unsigned long)addr);
	} else
		pr_err("can not get iwhale2 wakeup phyaddr!\n");

	return 0;
}

static int iwhale2_bia_pic_init(struct device_node *node,
				   struct device_node *parent)
{
	return 0;
}
/*
 * We cannot use the IRQCHIP_DECLARE macro that lives in
 * drivers/irqchip, so we're forced to roll our own. Not very nice.
 */
OF_DECLARE_2(irqchip, sc7731_intc_chip, "sprd,sc7731-extn-intc",
	     sprd_common_wakeup_of_init);
OF_DECLARE_2(irqchip, sc9836_intc_chip, "sprd,sc9836-extn-intc",
	     sprd_common_wakeup_of_init);
OF_DECLARE_2(irqchip, sc9838_intc_chip, "sprd,sc9838-extn-intc",
	     sprd_common_wakeup_of_init);
OF_DECLARE_2(irqchip, whale2_wakeupen_intc, "sprd,whale2-pmuwakeup-chip",
	     whale2_wakeupen_of_init);
OF_DECLARE_2(irqchip, iwhale2_wakeupen_intc, "sprd,iwhale2-pmuwakeup-chip",
	     iwhale2_wakeupen_of_init);
OF_DECLARE_2(irqchip, bia_pic, "intel,x86-ioapic", iwhale2_bia_pic_init);

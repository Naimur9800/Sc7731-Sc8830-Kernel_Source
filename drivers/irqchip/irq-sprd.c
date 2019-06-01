/*
 * Copyright (C) 2014 Spreadtrum Communications Inc.
 *
 * Contact: steve.zhan <steve.zhan@spreadtrum.com>
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
#include "irqchip.h"

#define	INTC_IRQ_MSK		0x0000
#define	INTC_IRQ_RAW		0x0004
#define	INTC_IRQ_EN		0x0008
#define	INTC_IRQ_DIS		0x000C
#define	INTC_IRQ_SOFT		0x0010
#define	INTC_FIQ_MSK		0x0020
#define	INTC_FIQ_RAW		0x0024
#define	INTC_FIQ_EN		0x0028
#define	INTC_FIQ_DIS		0x002C

#define IRQ_GIC_START		32
#define IRQ_TO_INTC_NR(irq)	((irq) - IRQ_GIC_START)
#define IRQ_TO_INTC_BIT(irq)	(1 << ((irq) & 0x1f))


struct intc {
	u32 min_int_num;/*global logical number max value in intc*/
	u32 max_min_num;/*global logical number min value in intc*/
	u32 min_bit_offset;/*It is equal to "min_int_num - actual bit offset"*/
	void *reg_base;
};

static struct intc intc_desc[] = {
	{0, 31, 0},
	{32, 63, 32},
	{64, 95, 64},
	{96, 127, 96},
};

static inline int __irq_find_base(u32 irq, u64 *base, u32 *bit)
{
	int s = ARRAY_SIZE(intc_desc);
	while (s--)
		if (irq >= intc_desc[s].min_int_num && \
				irq <= intc_desc[s].max_min_num) {
			*base = (u64)intc_desc[s].reg_base;
			*bit = irq - intc_desc[s].min_bit_offset;
			return 0;
		}
	return -ENXIO;
}

static void __irq_mask(struct irq_data *data)
{
	unsigned int irq = IRQ_TO_INTC_NR(data->irq);
	u64 base;
	u32 bit;
	u32 offset = INTC_IRQ_DIS;

	if (!__irq_find_base(irq, &base, &bit))
		__raw_writel(1 << bit, (void *)(base + offset));
}

static void __irq_unmask(struct irq_data *data)
{
	unsigned int irq = IRQ_TO_INTC_NR(data->irq);
	u64 base;
	u32 bit;
	u32 offset = INTC_IRQ_EN;

	if (!__irq_find_base(irq, &base, &bit))
		__raw_writel(1 << bit, (void *)(base + offset));
}

int __init sci_of_init(struct device_node *node, struct device_node *parent)
{
	unsigned long *addr;
	int i;
	u32 val;

	for (i = 0; i < 4; i++) {
		addr = of_iomap(node, i);
		if (addr)
			intc_desc[i].reg_base = addr;
	}

	gic_arch_extn.irq_mask = __irq_mask;
	gic_arch_extn.irq_unmask = __irq_unmask;

	addr = intc_desc[0].reg_base;
	val = __raw_readl(addr + INTC_IRQ_EN);
	val |= 0xf<<2;
	__raw_writel(val, addr + INTC_IRQ_EN);

	return 0;
}

IRQCHIP_DECLARE(sprd_intc_chip, "sprd,intc", sci_of_init);

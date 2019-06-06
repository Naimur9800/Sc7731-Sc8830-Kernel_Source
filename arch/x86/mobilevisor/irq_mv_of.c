/*
 * Copyright (C) 2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irqchip.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/export.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

#include <asm/mv/mv_irq.h>

/*
 * Get intel,io-access property if any from dts
 */
bool mv_irq_get_io_master(struct device_node *np)
{
	if (of_find_property(np, "intel,io-access-guest", NULL))
		return IRQ_IO_ACCESS_BY_LNX;
	return IRQ_IO_ACCESS_BY_VMM;
}
/*
 * fill irq reg
 */
int __init mv_irq_of_parse(struct irq_reg **datareg,
			struct device_node *np, char *comp)
{
	uint32_t ret = 0, len = 0, idx = 0, j = 0;
	uint32_t *array;
	struct irq_reg *reg;

	if (of_find_property(np, comp, &len)) {
		len /= sizeof(u32);
		array = kcalloc(len, sizeof(u32), GFP_KERNEL);
		if (!array) {
			pr_err("array allocation failed\n");
			return -ENOMEM;
		}
		ret = of_property_read_u32_array(np, comp, array, len);
		if (ret != 0)
			pr_err("read %s property failed: %d\n", comp, ret);

		reg = kzalloc(sizeof(struct irq_reg), GFP_KERNEL);
		if (!reg) {
			kfree(array);
			return -ENOMEM;
		}
		pr_debug("Parsing %s properties:\n", comp);
		for (idx = 0; idx + 2 < len; idx += 3) {
			reg->base = array[idx];
			reg->offset = array[idx+1];
			for (j = 0; j < array[idx+2]; j++)
				reg->mask |= 1 << j;

			reg->mask <<= array[idx+1];
			*datareg = reg;
			pr_debug("%s: %s: base=%#x, offset=%#x, mask=%#x\n",
					__func__, comp, reg->base,
							reg->offset, reg->mask);
		}
		kfree(array);
	}
	return 0;
}

static void mv_display_irq_reg(struct irq_reg *p,
		uint32_t i, char *name)
{
	if (!p)
		return;
	pr_debug("[%02d] %s\t %08x %08x %08x\n",
			i, name, p->base, p->offset, p->mask);
}

static void mv_display_irq_regs(struct mv_irq_chip_data *data)
{
	uint32_t i;

	pr_debug("%s: %s - %d interrupts\n", __func__,
			data->name, data->nr_int);
	pr_debug("[NR] name\t base     offset   mask\n");
	for (i = 0; i < data->nr_int; i++) {
		mv_display_irq_reg(data->mask[i], i, "mask");
		mv_display_irq_reg(data->unmask[i], i, "unmask");
		mv_display_irq_reg(data->slmask[i], i, "slmask");
		mv_display_irq_reg(data->ack[i], i, "ack ");
		mv_display_irq_reg(data->edge[i], i, "edge");
		mv_display_irq_reg(data->level[i], i, "level");
		mv_display_irq_reg(data->status[i], i, "status");
		mv_display_irq_reg(data->set[i], i, "set ");
	}
	mv_display_irq_reg(data->globalmask[0], 0, "globalmask");
}

/*
 * parse mv irq domain description
 */
int __init mv_irq_of_get(struct device_node *np, void *d)
{
	struct mv_irq_chip_data *data = (struct mv_irq_chip_data *)d;
	int32_t i = 0, cplen = 0;
	const char *name = of_get_property(np, "compatible", &cplen);
	char comp[30];
	struct resource regs;
	uint32_t sz = sizeof(struct irq_req *);

	data->np = np;
	if (!name) {
		/* This is not a good point, just warn at this time */
		pr_warn("%s: no compatible found\n", __func__);
	} else {
		pr_debug("%s: Looking for %s in dts\n", VMM_IRQ, name);
		strncpy(data->name, name, MAX_NAME_LENGTH-1);
	}
	/* Get io resources */
	if (of_address_to_resource(np, 0, &regs)) {
		/* it's no killing, we are supported SW IRQ domains */
		pr_debug("%s: no resource found", __func__);
	} else {
		/* Get io access master */
		data->io_master = mv_irq_get_io_master(np);
		data->base_phys = regs.start;
		data->base = of_iomap(np, 0);
		pr_debug("%s: io:%s - v:%p - p:%#x\n", VMM_IRQ,
			data->io_master == IRQ_IO_ACCESS_BY_LNX ?
			"linux" : "mv", data->base, data->base_phys);
	}
	/* Get NR of irqs */
	data->nr_int = of_irq_count(np);
	if (!data->nr_int) {
		/* nr_int passed thru property? */
		of_property_read_u32(np, "intel,nr_int", &data->nr_int);
		if (!data->nr_int) {
			/* This is killing */
			pr_err("%s: no irq found for %s\n", __func__, name);
			return -EINVAL;
		}
	}
	pr_debug("%s: %d interrupts found\n", __func__, data->nr_int);

	of_property_read_u32(np, "intel,hirq", &data->hirq);
	if (data->hirq)
		pr_debug("%s: hirq vector: %d\n", __func__, data->hirq);

	data->mask = kzalloc(data->nr_int * sz, GFP_KERNEL);
	if (!data->mask)
		goto free_them_all;
	data->unmask = kzalloc(data->nr_int * sz, GFP_KERNEL);
	if (!data->unmask)
		goto free_them_all;
	data->slmask = kzalloc(data->nr_int * sz, GFP_KERNEL);
	if (!data->slmask)
		goto free_them_all;
	data->ack = kzalloc(data->nr_int * sz, GFP_KERNEL);
	if (!data->ack)
		goto free_them_all;
	data->edge = kzalloc(data->nr_int * sz, GFP_KERNEL);
	if (!data->edge)
		goto free_them_all;
	data->level = kzalloc(data->nr_int * sz, GFP_KERNEL);
	if (!data->level)
		goto free_them_all;
	data->status = kzalloc(data->nr_int * sz, GFP_KERNEL);
	if (!data->status)
		goto free_them_all;
	data->set = kzalloc(data->nr_int * sz, GFP_KERNEL);
	if (!data->set)
		goto free_them_all;
	data->globalmask = kzalloc(sz, GFP_KERNEL);
	if (!data->globalmask)
		goto free_them_all;
	data->virq = kcalloc(data->nr_int, sizeof(uint32_t), GFP_KERNEL);
	if (!data->virq)
		goto free_them_all;
	data->preack = kcalloc(data->nr_int, sizeof(uint32_t), GFP_KERNEL);
	if (!data->preack)
		goto free_them_all;
	for (i = 0; i < data->nr_int; i++) {
		sprintf(comp, "intel,mask,%d", i);
		mv_irq_of_parse(&data->mask[i], np, comp);
		sprintf(comp, "intel,unmask,%d", i);
		mv_irq_of_parse(&data->unmask[i], np, comp);
		sprintf(comp, "intel,slmask,%d", i);
		mv_irq_of_parse(&data->slmask[i], np, comp);
		sprintf(comp, "intel,ack,%d", i);
		mv_irq_of_parse(&data->ack[i], np, comp);
		sprintf(comp, "intel,edge,%d", i);
		mv_irq_of_parse(&data->edge[i], np, comp);
		sprintf(comp, "intel,level,%d", i);
		mv_irq_of_parse(&data->level[i], np, comp);
		sprintf(comp, "intel,status,%d", i);
		mv_irq_of_parse(&data->status[i], np, comp);
		sprintf(comp, "intel,set,%d", i);
		mv_irq_of_parse(&data->set[i], np, comp);
		sprintf(comp, "intel,virq,%d", i);
		of_property_read_u32(np, comp, &data->virq[i]);
		if (data->virq[i])
			pr_debug("%s: virq[%d]=%d detected !!!\n",
					__func__, i, data->virq[i]);
		sprintf(comp, "intel,preack,%d", i);
		of_property_read_u32(np, comp, &data->preack[i]);
	}
	sprintf(comp, "intel,globalmask");
	mv_irq_of_parse(&data->globalmask[0], np, comp);
	/* Display captured device tree data */
	mv_display_irq_regs(data);
	return 0;

free_them_all:
	kfree(data->globalmask);
	kfree(data->mask);
	kfree(data->unmask);
	kfree(data->slmask);
	kfree(data->ack);
	kfree(data->edge);
	kfree(data->level);
	kfree(data->status);
	kfree(data->set);
	kfree(data->virq);
	kfree(data->preack);
	return -EINVAL;
}

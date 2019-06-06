/*
 * Copyright (C) 2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/cpu_pm.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include <asm/mv/mv_irq.h>
#include <asm/mv/mv_gal.h>
#include <asm/mv/mv_svc_hypercalls.h>

static DEFINE_SPINLOCK(hirq_lock);
static u32 irq_hirq_offset;

static inline uint32_t vpic_irq(struct irq_data *d)
{
	return d->hwirq;
}

static uint32_t mv_irq_to_vector(uint32_t irq)
{
	return irq + irq_hirq_offset;
}

static unsigned int mv_irq_hirq_startup(struct irq_data *data)
{
	uint32_t irq = vpic_irq(data);
	uint32_t vect = mv_irq_to_vector(irq);

	pr_debug("%s: vect %d irq=%d\n", __func__, vect, irq);

	spin_lock(&hirq_lock);
	mv_virq_request(vect, 1);
	mv_virq_unmask(vect);
	spin_unlock(&hirq_lock);

	return 0;
}

static void mv_irq_hirq_shutdown(struct irq_data *data)
{
	uint32_t irq = vpic_irq(data);
	uint32_t vect = mv_irq_to_vector(irq);

	pr_debug("%s: vect %d irq=%d\n", __func__, vect, irq);

	spin_lock(&hirq_lock);
	/* TODO: Free the request vector */
	mv_virq_unmask(vect);
	spin_unlock(&hirq_lock);
}

static inline void mv_irq_hirq_unmask(struct irq_data *data)
{
	uint32_t irq = vpic_irq(data);
	uint32_t vect = mv_irq_to_vector(irq);

	pr_debug("%s: mv_virq_unmask(%d) - hwirq=%d\n",
			__func__, vect, irq);
	spin_lock(&hirq_lock);
	mv_virq_unmask(vect);
	spin_unlock(&hirq_lock);
}

static inline void mv_irq_hirq_mask(struct irq_data *data)
{
	uint32_t irq = vpic_irq(data);
	uint32_t vect = mv_irq_to_vector(irq);

	pr_debug("%s: mv_virq_mask(%d) - hwirq=%d\n",
			__func__, vect, irq);
	spin_lock(&hirq_lock);
	mv_virq_mask(vect);
	spin_unlock(&hirq_lock);
}

static void mv_irq_hirq_enable(struct irq_data *data)
{
	uint32_t irq = vpic_irq(data);
	uint32_t vect = mv_irq_to_vector(irq);

	spin_lock(&hirq_lock);
	pr_debug("%s: mv_virq_unmask(%d) - hwirq=%d\n",
			__func__, vect, irq);
	mv_virq_unmask(vect);
	spin_unlock(&hirq_lock);
}

static void mv_irq_hirq_disable(struct irq_data *data)
{
	uint32_t irq = vpic_irq(data);
	uint32_t vect = mv_irq_to_vector(irq);

	pr_debug("%s: mv_virq_mask(%d) - hwirq=%d\n",
			__func__, vect, irq);
	spin_lock(&hirq_lock);
	mv_virq_mask(vect);
	spin_unlock(&hirq_lock);
}

static int mv_irq_hirq_set_affinity(struct irq_data *data,
				const struct cpumask *mask,
				bool force)
{
	unsigned int dest_id = 0;
	uint32_t irq = vpic_irq(data);
	uint32_t vect = mv_irq_to_vector(irq);
	int err;

	if (!config_enabled(CONFIG_SMP))
		return -EPERM;

	if (!cpumask_intersects(mask, cpu_online_mask))
		return -EINVAL;

	err = apic->cpu_mask_to_apicid_and(mask, cpu_online_mask, &dest_id);
	if (err) {
		pr_err("Failed dest_id for irq %d\n", irq);
		return err;
	}

	cpumask_copy(data->common->affinity, mask);
	pr_debug("%s: vect %d hwirq=%d dest_id 0x%x\n",
			__func__, vect, irq, dest_id);
	spin_lock(&hirq_lock);
	mv_virq_set_affinity(vect, dest_id);
	spin_unlock(&hirq_lock);

	return IRQ_SET_MASK_OK_NOCOPY;
}

void mv_irq_hirq_eoi(struct irq_data *data)
{
	uint32_t irq = vpic_irq(data);
	uint32_t vect = mv_irq_to_vector(irq);

	pr_debug("%s: mv_virq_eoi(%d) - hwirq=%d\n",
			__func__, vect, irq);
	spin_lock(&hirq_lock);
	mv_virq_eoi(vect);
	spin_unlock(&hirq_lock);
}

static struct irq_chip mv_irq_hirq_chip = {
	.name = "HIRQ",
	.irq_startup = mv_irq_hirq_startup,
	.irq_shutdown = mv_irq_hirq_shutdown,
	.irq_mask = mv_irq_hirq_mask,
	.irq_unmask = mv_irq_hirq_unmask,
	.irq_enable = mv_irq_hirq_enable,
	.irq_disable = mv_irq_hirq_disable,
	.irq_set_affinity = mv_irq_hirq_set_affinity,
	.irq_eoi = mv_irq_hirq_eoi,
};

static struct irq_domain_ops mv_irq_hirq_domain_ops = {
	.xlate = irq_domain_xlate_twocell,
	.map = mv_irq_domain_map,
};

/*
 * Run software resend of hirq
 */
static void mv_hirq_resend(unsigned long arg)
{
	struct mv_irq_chip_data *data = (struct mv_irq_chip_data *)arg;
	int irq = data->hirq;
	struct irq_desc *desc = irq_to_desc(irq);

	pr_debug("%s(%d)\n", __func__, irq);
	if (desc) {
		local_irq_disable();
		desc->handle_irq(desc);
		local_irq_enable();
	}
}

static uint32_t mv_irq_hirq_find_mapping(uint32_t irq)
{
	uint32_t index = 0;
	struct mv_irq_chip_data *data = irq_get_handler_data(irq);
	struct mv_shared_data *pdata = mv_gal_get_shared_data();
	struct hirq_info_t *p_hirq = &(pdata->hirq_info);
	uint32_t level1, level2;

	pr_debug("%s(%d)-->\n", __func__, irq);

	if (p_hirq->lvl1) {
		level1 = __ffs(p_hirq->lvl1);
		if (level1 >= 16) {
			pr_err("%s: invalid virq detected\n", __func__);
			BUG();
		}

		if (p_hirq->lvl2[level1] == 0) {
			pr_err("%s: error - lvl2 is null...\n", __func__);
			return 0;
		}

		level2 = __ffs(p_hirq->lvl2[level1]);
		index = (level1 << 5) + level2;

		p_hirq->lvl2[level1] &= ~(1 << level2);
		if (p_hirq->lvl2[level1] == 0)
			p_hirq->lvl1 &= ~(1 << level1);

		/* we need to re-trigger the irq handler
		   since there are still pending virqs */
		if (p_hirq->lvl1)
			tasklet_schedule(&data->hirq_resend);
	} else
		pr_err("spurious virq detected\n");

	if (index >= irq_hirq_offset)
		BUG();

	return index;
}

unsigned __init mv_irq_get_hirq_offset(struct device_node *np)
{
	unsigned hirq_offset = 512;

	of_property_read_u32(np, "intel,hirq_offset", &hirq_offset);
	return hirq_offset;
}

/*
 * Entry point for HIRQ IRQ. called from of_irq_init
 */
static int32_t __init mv_irq_hirq_of_init(struct device_node *np,
					struct device_node *parent)
{
	int32_t ret = 0;
	struct mv_irq_chip_data *data;

	data = kzalloc(sizeof(struct mv_irq_chip_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->chip = &mv_irq_hirq_chip;
	data->type = VMM_IRQ_DOMAIN_CUST;
	data->find_mapping = mv_irq_hirq_find_mapping;
	data->flow_handler = handle_fasteoi_irq;
	irq_hirq_offset = mv_irq_get_hirq_offset(np);

	/* extract info form dt */
	mv_irq_of_get(np, data);

	if (data->hirq)
		tasklet_init(&data->hirq_resend, mv_hirq_resend,
						(unsigned long)data);

	/* add linear domain */
	ret |= mv_irq_domain_add_linear(np,
			data, &mv_irq_hirq_domain_ops);

	/* Parse, Map and Cascade */
	if (parent)
		ret |= mv_irq_parse_map_and_cascade(np, data);

	if (ret < 0)
		kfree(data);
	return ret;
}

OF_DECLARE_2(irqchip, mv_hirq_vpic, "intel,mv-hirq", mv_irq_hirq_of_init);

static void mv_irq_hirq_main_unmask(struct irq_data *data)
{
}

static void mv_irq_hirq_main_mask(struct irq_data *data)
{
}

static void mv_irq_hirq_main_enable(struct irq_data *data)
{
}

static void mv_irq_hirq_main_disable(struct irq_data *data)
{
}

void mv_irq_hirq_main_eoi(struct irq_data *data)
{
}

static struct irq_chip mv_irq_hirq_main_chip = {
	.name = "HIRQ MAIN",
	.irq_mask = mv_irq_hirq_main_mask,
	.irq_unmask = mv_irq_hirq_main_unmask,
	.irq_enable = mv_irq_hirq_main_enable,
	.irq_disable = mv_irq_hirq_main_disable,
	.irq_eoi = mv_irq_hirq_main_eoi,
};

static struct irq_domain_ops mv_irq_hirq_main_domain_ops = {
	.xlate = irq_domain_xlate_twocell,
	.map = mv_irq_domain_map,
};

static int32_t __init mv_irq_hirq_main_of_init(struct device_node *np,
					struct device_node *parent)
{
	int32_t ret = 0;
	struct mv_irq_chip_data *data;

	data = kzalloc(sizeof(struct mv_irq_chip_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->chip = &mv_irq_hirq_main_chip;
	data->type = VMM_IRQ_DOMAIN_N2N;

	/* extract info form dt */
	mv_irq_of_get(np, data);

	/* add linear domain */
	ret |= mv_irq_domain_add_linear(np,
			data, &mv_irq_hirq_main_domain_ops);

	/* Parse, Map and Cascade */
	if (parent)
		ret |= mv_irq_parse_map_and_cascade(np, data);

	if (ret < 0)
		kfree(data);
	return ret;
}

OF_DECLARE_2(irqchip, mv_hirq_main_vpic, "intel,mv-main-hirq",
		mv_irq_hirq_main_of_init);

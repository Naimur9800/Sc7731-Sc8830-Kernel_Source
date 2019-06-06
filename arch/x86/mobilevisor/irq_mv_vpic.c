/*
 * Copyright (C) 2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <asm/prom.h>
#include <asm/hw_irq.h>

#include <asm/mv/cpu.h>
#include <asm/mv/mv_hypercalls.h>
#include <asm/apic.h>
#include <asm/mv/reg_access/mv_svc_reg_access.h>

struct apic_chip_data {
	struct irq_cfg		cfg;
	cpumask_var_t		domain;
	cpumask_var_t		old_domain;
	u8			move_in_progress : 1;
};

static struct vpic_chip_data {
	struct irq_domain *domain;
	unsigned int nr_irqs;
	void __iomem *reg_base;
	void __iomem *entries_base;
} vpic_data __read_mostly;

static struct irq_domain *mv_vpic_domain;

struct irq_domain *mv_vpic_get_domain(void)
{
	return mv_vpic_domain;
}

static inline unsigned int vpic_irq(struct irq_data *d)
{
	return d->hwirq;
}

static inline unsigned int mv_irq_to_vector(unsigned int irq)
{
	return irq;
}

static inline unsigned int mv_vector_to_irq(unsigned int vector)
{
	return vector;
}

static struct apic_chip_data *alloc_irq_cfg_data(void)
{
	struct apic_chip_data *data;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return NULL;
	if (!zalloc_cpumask_var(&data->domain, GFP_KERNEL))
		goto out_data;
	if (!zalloc_cpumask_var(&data->old_domain, GFP_KERNEL))
		goto out_domain;
	return data;
out_domain:
	free_cpumask_var(data->domain);
out_data:
	kfree(data);
	return NULL;
}

static int __mv_assign_irq_vector(int irq,
		struct apic_chip_data *data, const struct cpumask *mask)
{
	int err = -ENOSPC;
	cpumask_var_t tmp_mask;

	if (data == NULL)
		BUG();

	if (data->move_in_progress)
		return -EBUSY;

	if (!alloc_cpumask_var(&tmp_mask, GFP_ATOMIC))
		return -ENOMEM;

	/* Only try and allocate irqs on cpus that are present */
	cpumask_clear(data->old_domain);

	while (true) {
		int new_cpu, vector;
		unsigned core = smp_processor_id();

		apic->vector_allocation_domain(core, tmp_mask, mask);

		cpumask_and(tmp_mask, tmp_mask, cpu_online_mask);
		/* Check whether the vector is already installed,
		 * affinity domain correctly set, etc..*/
		if (cpumask_subset(tmp_mask, data->domain)) {
			err = 0;
			if (cpumask_equal(tmp_mask, data->domain))
				break;
			/*
			 * New cpumask using the vector is a proper subset of
			 * the current in use mask. So cleanup the vector
			 * allocation for the members that are not used anymore.
			 */
			cpumask_andnot(data->old_domain, data->domain, tmp_mask);
			data->move_in_progress =
			   cpumask_intersects(data->old_domain, cpu_online_mask);
			cpumask_and(data->domain, data->domain, tmp_mask);

			break;
		}
		vector = mv_irq_to_vector(irq);
		if (data->cfg.vector && data->cfg.vector != vector) {
			/* Shall not happen as vector is fixed on mv system. */
			pr_notice("Vector %d->%d move in progress?\n",
				data->cfg.vector, vector);
			cpumask_copy(data->old_domain, data->domain);
			data->move_in_progress =
			   cpumask_intersects(data->old_domain, cpu_online_mask);
		}
		for_each_cpu_and(new_cpu, tmp_mask, cpu_online_mask)
			per_cpu(vector_irq, new_cpu)[vector] = irq_to_desc(irq);
		data->cfg.vector = vector;
		cpumask_copy(data->domain, tmp_mask);
		err = 0;
		break;
	}
	free_cpumask_var(tmp_mask);
	return err;
}

static int
mv_assign_irq_vector(int irq, struct apic_chip_data *cfg, const struct cpumask *mask)
{
	int err;
	unsigned long flags;

	local_irq_save(flags);
	lock_vector_lock();
	err = __mv_assign_irq_vector(irq, cfg, mask);
	unlock_vector_lock();
	local_irq_restore(flags);
	return err;
}

static unsigned int mv_vpic_irq_startup(struct irq_data *data)
{
	unsigned int irq = vpic_irq(data);
	struct apic_chip_data *cfg_data;
	unsigned affinity;

	cfg_data = irq_get_chip_data(irq);
	if (!cfg_data)
		return -ENXIO;

	if (irq < FIRST_EXTERNAL_VECTOR) {
		pr_err("%s: invalid irq: %d\n", __func__, irq);
		return -EINVAL;
	}

	/* Vector assignement is fixed in mv platform,
	 * but this will set correctly the domain mask
	 * */
	mv_assign_irq_vector(irq, cfg_data, apic->target_cpus());
	affinity = cpumask_bits(cfg_data->domain)[0];
	mv_virq_request(cfg_data->cfg.vector, affinity);
	mv_virq_unmask(cfg_data->cfg.vector);
	return 0;
}

static void mv_vpic_irq_shutdown(struct irq_data *data)
{
	unsigned int irq = vpic_irq(data);
	unsigned int vect = mv_irq_to_vector(irq);
	struct apic_chip_data *cfg_data = irq_get_chip_data(irq);
	unsigned long flags;
	int cpu;

	mv_virq_mask(vect);

	local_irq_save(flags);
	lock_vector_lock();
	for_each_cpu_and(cpu, cfg_data->domain, cpu_online_mask)
		per_cpu(vector_irq, cpu)[vect] = VECTOR_UNUSED;
	cpumask_clear(cfg_data->old_domain);
	cpumask_clear(cfg_data->domain);
	cfg_data->cfg.vector = 0;
	unlock_vector_lock();
	local_irq_restore(flags);
}

static void mv_vpic_irq_mask(struct irq_data *data)
{
	unsigned int irq = vpic_irq(data);
	unsigned int vect = mv_irq_to_vector(irq);

	mv_virq_mask(vect);
}

static void mv_vpic_irq_unmask(struct irq_data *data)
{
	unsigned int irq = vpic_irq(data);
	unsigned int vect = mv_irq_to_vector(irq);

	mv_virq_unmask(vect);
}

static void mv_vpic_irq_enable(struct irq_data *data)
{
	unsigned int irq = vpic_irq(data);
	unsigned int vect = mv_irq_to_vector(irq);

	mv_virq_unmask(vect);
}

static void mv_vpic_irq_disable(struct irq_data *data)
{
	unsigned int irq = vpic_irq(data);
	unsigned int vect = mv_irq_to_vector(irq);

	mv_virq_mask(vect);
}

static inline bool vpic_irqd_mask(struct irq_data *data)
{
	/* If we are moving the irq we need to mask it */
	if (unlikely(irqd_is_setaffinity_pending(data))) {
		mv_vpic_irq_mask(data);
		return true;
	}
	return false;
}

static inline void vpic_irqd_unmask(struct irq_data *data, bool masked)
{
	int ret;
	unsigned int mask, val, irq, vect;

	if (unlikely(masked)) {
		/* Only migrate the irq if the ack has been received.
		 *
		 * On rare occasions the broadcast level triggered ack gets
		 * delayed going to ioapics, and if we reprogram the
		 * vector while Remote IRR is still set the irq will never
		 * fire again.
		 *
		 * To prevent this scenario we read the Remote IRR bit
		 * of the ioapic.  This has two effects.
		 * - On any sane system the read of the ioapic will
		 *   flush writes (and acks) going to the ioapic from
		 *   this cpu.
		 * - We get to see if the ACK has actually been delivered.
		 *
		 * Based on failed experiments of reprogramming the
		 * ioapic entry from outside of irq context starting
		 * with masking the ioapic entry and then polling until
		 * Remote IRR was clear before reprogramming the
		 * ioapic I don't trust the Remote IRR bit to be
		 * completey accurate.
		 *
		 * However there appears to be no other way to plug
		 * this race, so if the Remote IRR bit is not
		 * accurate and is causing problems then it is a hardware bug
		 * and you can go talk to the chipset vendor about it.
		 */
		/*
		if (!io_apic_level_ack_pending(data->chip_data))
			irq_move_masked_irq(data);
		*/
		mask = 0;
		irq =  vpic_irq(data);
		vect = mv_irq_to_vector(irq);
		ret = mv_svc_reg_read(vect, &val, mask);

		if (ret)
			pr_err("%s: failed vector: %d\n", __func__, vect);
		else if (val == 0) {
			irq_move_masked_irq(data);
		} else
			pr_err("%s: failed value: %d\n", __func__, val);

		mv_vpic_irq_unmask(data);
	}
}

static void mv_vpic_irq_eoi(struct irq_data *data)
{
	unsigned int irq =  vpic_irq(data);
	unsigned int vect = mv_irq_to_vector(irq);
	bool masked;
	int this_cpu;

	masked = vpic_irqd_mask(data);

	this_cpu = get_cpu();
	if (mv_is_cpu_exclusive(this_cpu) && mv_is_irq_bypass())
		ack_APIC_irq();
	else
		mv_virq_eoi(vect);
	put_cpu();

	vpic_irqd_unmask(data, masked);
}

static int mv_vpic_set_affinity(struct irq_data *data,
			       const struct cpumask *mask,
			       bool force)
{
	struct apic_chip_data *cfg_data = data->chip_data;
	unsigned int irq = data->irq;
	unsigned int vect = mv_irq_to_vector(irq);
	unsigned int dest_id;
	int err;

	if (!config_enabled(CONFIG_SMP))
		return -1;

	if (!cpumask_intersects(mask, cpu_online_mask))
		return -EINVAL;

	err = mv_assign_irq_vector(irq, cfg_data, mask);
	if (err) {
		pr_err("%s: Error while assigning vector %d, err %d\n",
				__func__, cfg_data->cfg.vector, err);
		return err;
	}

	err = apic->cpu_mask_to_apicid_and(mask, cfg_data->domain, &dest_id);
	if (err) {
		if (mv_assign_irq_vector(irq, cfg_data, data->common->affinity))
			pr_err("Failed to recover vector for irq %d, err %d\n",
				irq, err);
		return err;
	}
	cpumask_copy(data->common->affinity, mask);

	mv_virq_set_affinity(vect, dest_id);

	return IRQ_SET_MASK_OK_NOCOPY;
}

static struct irq_chip mv_vpic_chip = {
	.name = "VPIC",
	.irq_startup = mv_vpic_irq_startup,
	.irq_shutdown = mv_vpic_irq_shutdown,
	.irq_mask = mv_vpic_irq_mask,
	.irq_unmask = mv_vpic_irq_unmask,
	.irq_enable = mv_vpic_irq_enable,
	.irq_disable = mv_vpic_irq_disable,
	.irq_eoi = mv_vpic_irq_eoi,
	.irq_set_affinity   = mv_vpic_set_affinity,
};

static int mv_vpic_irq_domain_map(struct irq_domain *d, unsigned int irq,
			      irq_hw_number_t hw)
{
	int vector, cpu;
	struct apic_chip_data *data;

	data = alloc_irq_cfg_data();
	if (data == NULL)
		BUG();
	data->cfg.vector = 0;

	irq_set_chip_data(irq, data);
	irq_clear_status_flags(irq, IRQ_NOREQUEST);
	irq_set_chip_and_handler(irq, &mv_vpic_chip, handle_fasteoi_irq);

	if (irq > 31) {
		vector = mv_irq_to_vector(irq);
		for_each_possible_cpu(cpu)
			per_cpu(vector_irq, cpu)[vector] = VECTOR_UNUSED;
	}
	return 0;
}

static const struct irq_domain_ops mv_vpic_irq_domain_ops = {
	.map = mv_vpic_irq_domain_map,
	.xlate = irq_domain_xlate_onecell,
};


/* Return number of vpic irqs */
static unsigned __init vpic_get_nr_of_irqs(struct device_node *np)
{
	unsigned vpic_irqs;
	unsigned vpic_irqs_default = 256;

	if (of_property_read_u32(np, "intel,vpic-irqs", &vpic_irqs)) {
		pr_warn("%s: vpic-irqs not specified in dts - set it to: %d\n",
				__func__, vpic_irqs_default);
		vpic_irqs = vpic_irqs_default;
	}
	return vpic_irqs;
}

static int __init vpic_of_init(struct device_node *np,
		struct device_node *parent)
{
	struct irq_domain *id;
	unsigned int vpic_irqs;
	int ret;

	vpic_data.nr_irqs = vpic_get_nr_of_irqs(np);
	vpic_irqs = vpic_data.nr_irqs;

	/* Map io even if not used - confortable to have it in perfile */
	vpic_data.reg_base = of_iomap(np, 0);
	vpic_data.entries_base = of_iomap(np, 1);
	pr_info("%s: mv-vpic register remapping: Reg: %p, entries %p\n",
			__func__, vpic_data.reg_base, vpic_data.entries_base);

	id = irq_domain_add_linear(np, vpic_irqs, &mv_vpic_irq_domain_ops,
			(void *)&vpic_data);

	BUG_ON(!id);
	vpic_data.domain  = id;

	ret = irq_create_strict_mappings(id, 0, 0, vpic_irqs);
	if (ret)
		pr_err("%s: Error mapping legacy IRQs: %d\n", __func__, ret);

	of_ioapic = 1;

	mv_vpic_domain = id;

	return 0;
}

OF_DECLARE_2(irqchip, mv_vpic, "intel,mv-vpic", vpic_of_init);

/*
 * Copyright (C) 2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _IRQCHIP_IRQ_MV_H
#define _IRQCHIP_IRQ_MV_H

#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>

struct irq_reg {
	unsigned base;
	unsigned offset;
	unsigned mask;
};

#define MAX_NAME_LENGTH 50
#define VMM_IRQ "irq-mv"

#define VMM_IRQ_DOMAIN_N2N 0
#define VMM_IRQ_DOMAIN_N21 1
#define VMM_IRQ_DOMAIN_CUST 2

struct mv_irq_chip_data {
	char name[MAX_NAME_LENGTH];
	struct irq_domain *domain;
	struct irq_chip *chip;
	struct device_node *np;
	void __iomem *base;
	uint32_t base_phys;
	unsigned nr_int;
	unsigned virq_base;
	unsigned *table;
	struct notifier_block irq_nb;
	struct irq_reg **globalmask;
	struct irq_reg **mask;
	struct irq_reg **unmask;
	struct irq_reg **slmask;
	struct irq_reg **ack;
	struct irq_reg **edge;
	struct irq_reg **level;
	struct irq_reg **status;
	struct irq_reg **set;
	struct clk *clk_kernel;
	unsigned int type;
	irq_flow_handler_t flow_handler;
	void (*handle_entry)(struct mv_irq_chip_data *);
	void (*handle_exit)(struct mv_irq_chip_data *);
	unsigned int (*find_mapping)(unsigned int);
	unsigned *virq;
	unsigned *preack;
	bool io_master;
	uint32_t hirq;
	struct tasklet_struct hirq_resend;
};

extern int mv_irq_of_get(struct device_node *np, void *);

extern int __init mv_irq_domain_add_linear(struct device_node *,
					struct mv_irq_chip_data *,
					struct irq_domain_ops *);

extern int __init mv_irq_parse_map_and_cascade(struct device_node *,
					struct mv_irq_chip_data *);

extern int mv_irq_domain_map(struct irq_domain *d, unsigned int virq,
				    irq_hw_number_t hw);
extern int mv_irq_domain_xlate(struct irq_domain *,
				      struct device_node *,
				      const u32 *, unsigned int,
					  unsigned long *, unsigned int *);

/* io accessors */
extern int32_t _mv_irq_write(void __iomem *, struct irq_reg *,
		uint32_t, bool);
extern int32_t _mv_irq_write_mv(uint32_t, struct irq_reg *,
		uint32_t, bool);
extern int32_t mv_irq_write(struct mv_irq_chip_data *, struct irq_reg *,
		uint32_t, bool);

extern int32_t _mv_irq_read(void __iomem *, struct irq_reg *);
extern int32_t _mv_irq_read_mv(uint32_t, struct irq_reg *);
extern int32_t mv_irq_read(struct mv_irq_chip_data *, struct irq_reg *);

extern bool mv_irq_get_io_master(struct device_node *np);
#define IRQ_IO_ACCESS_BY_VMM 0
#define IRQ_IO_ACCESS_BY_LNX 1

#define WAKE_ID_DBB 0
#define WAKE_ID_ABB 1

#ifdef CONFIG_PM
extern int mv_irq_set_wake(struct irq_data *, uint32_t, uint32_t, uint32_t);
#endif

#define VMM_RW 1
#define VMM_WO 0

extern struct irq_domain *mv_irq_eint_get_domain(void);

#endif /* _IRQCHIP_IRQ_MV_H */

/* EOF */

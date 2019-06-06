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
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>

#include <asm/mv/mv_irq.h>
#include <asm/mv/mv_svc_hypercalls.h>
#include <asm/mv/reg_access/mv_svc_reg_access.h>

/* Max Number of SpcuCA5CwkupEn registers per core */
#define NR_OF_REGS 4
/* Max Number of cores supporting the wakeup */
#define NR_OF_CORES 4

#define MAX_WAKE_LENGTH 20

struct mv_irq_wake_table {
	uint32_t reg_id;
	uint32_t offset;
	struct device_node *parent;
	struct irq_domain *domain;
	uint32_t vector;
};

static struct mv_irq_wake_data {
	/* Only cpu0 would be woken-up */
	bool only_cpu0;
	/* Wake table for wake capable interrupts - filled from dts */
	struct mv_irq_wake_table *wake_table[NR_OF_REGS];
	/* Table of wake enable registers */
	uint32_t wake_regs[NR_OF_CORES][NR_OF_REGS];
	/* SPCU Virtual address (ioremapped) */
	void __iomem *base_virt;
	/* SPCU Physical address */
	uint32_t base_phys;
	/* to defined who is doing the final io access */
	bool io_master;
	/* number of cores supporting the hw_wake */
	uint32_t nr_cores;
	/* number of registers per core */
	uint32_t nr_regs;
	/* irq resource */
	uint32_t irq;
	/* pinctrl */
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_inactive;
} wake_pdata[2];

struct mv_irq_wake_data *get_irq_wake_data(uint32_t id)
{
	return &wake_pdata[id];
}
/*
 * mv irq wake write accessor done by VMM
 */
int32_t _mv_irq_wake_write_mv(uint32_t addr, uint32_t val)
{
	if (mv_svc_reg_write(addr, val, -1)) {
		pr_err("%s: mv_svc_reg_write_service fails @%#x\n",
				__func__, addr);
		return -EIO;
	}
	return 0;
}
/*
 * mv irq wake read accessor done by VMM
 */
int32_t _mv_irq_wake_read_mv(uint32_t addr)
{
	uint32_t val;

	if (mv_svc_reg_read(addr, &val, -1)) {
		pr_err("%s: mv_svc_reg_read_service fails @%#x\n",
				__func__, addr);
		return -EIO;
	}
	return val;
}
/*
 * Write accessor function
 */
static int32_t mv_irq_wake_write(struct mv_irq_wake_data *data,
		uint32_t offset, uint32_t val)
{
	if (data->io_master == IRQ_IO_ACCESS_BY_VMM)
		return _mv_irq_wake_write_mv(data->base_phys + offset, val);
	else if (data->io_master == IRQ_IO_ACCESS_BY_LNX) {
		iowrite32(val, data->base_virt + offset);
		return 0;
	} else
		return -EIO;
}
/*
 * Read accessor function
 */
static int32_t mv_irq_wake_read(struct mv_irq_wake_data *data,
		uint32_t offset)
{
	if (data->io_master == IRQ_IO_ACCESS_BY_VMM)
		return _mv_irq_wake_read_mv(data->base_phys + offset);
	else if (data->io_master == IRQ_IO_ACCESS_BY_LNX)
		return ioread32(data->base_virt + offset);
	else
		return -EIO;
}

/*
 * Debug function displaying wake-map dts entries
 */
static void __init mv_irq_wake_display_table(struct mv_irq_wake_table *t,
		char *name)
{
	int32_t i = 0;

	pr_debug("%s: %s table created\n", __func__, name);
	while (t[i].reg_id != -1) {
		pr_debug("%s: %d\t%d\t%s\t%d\n", __func__, t[i].reg_id,
			t[i].offset, t[i].parent->full_name, t[i].vector);
		i++;
	}
}

/*
 * Allocate and Fill wake table according dts input
 */
static int32_t __init mv_irq_wake_fill_table(struct device_node *np,
		int32_t reg_id, struct mv_irq_wake_table **mv_table)
{
	struct mv_irq_wake_table *table;
	struct device_node *intpar;
	const __be32 *imap, *icell;
	int32_t imaplen, i, j = 0, cell, nr;
	char comp[MAX_WAKE_LENGTH];

	snprintf(comp, MAX_WAKE_LENGTH, "intel,wake-map-%d", reg_id);
	imap = of_get_property(np, comp, &imaplen);
	if (!imap) {
		pr_err("%s: no %s prop, exit...\n", __func__, comp);
		return -EINVAL;
	}
	imaplen /= sizeof(u32);
	pr_debug("%s: imaplen=%d\n", __func__, imaplen);

	icell = of_get_property(np, "#intel-wake-cells", NULL);
	if (icell) {
		cell = be32_to_cpu(*icell);
		pr_debug("%s: cell=%d\n", __func__, cell);
	} else {
		pr_err("%s: icell is NULL\n", __func__);
		return -EINVAL;
	}

	if (imaplen%cell) {
		pr_err("%s: wrong formating of wake table\n", __func__);
		return -EINVAL;
	}
	nr = (imaplen/cell)+1; /* on more for EnD oF TaBlE */
	table = kcalloc(nr, sizeof(struct mv_irq_wake_table), GFP_KERNEL);
	if (!table) {
		pr_err("%s: %s table allocation failed, exit...\n",
				__func__, comp);
		return -ENOMEM;
	}

	for (i = 0; i < imaplen; i += cell) {
		j = i / cell;
		table[j].offset = be32_to_cpu(imap[i]);
		intpar = of_find_node_by_phandle(be32_to_cpup(&imap[i+1]));
		if (!intpar) {
			pr_err("%s: no interrupt parent found!, exit...\n",
								__func__);
			kfree(table);
			return -EINVAL;
		}
		table[j].parent = intpar;
		table[j].vector = be32_to_cpu(imap[i+2]);
		table[j].reg_id = reg_id;
	}
	table[j+1].reg_id = -1; /* EnD oF TaBlE */
	*mv_table = table;
	mv_irq_wake_display_table(table, comp);
	return 0;
}

/*
 * Called by mv_irq_set_wake
 * Tell if interrupt is wake-capable
 */
int32_t mv_irq_is_wake_capable(struct irq_domain *domain,
		struct mv_irq_wake_data *data,
		int32_t irq, struct mv_irq_wake_table *find)
{
	int32_t j;
    struct device_node *of_node = irq_domain_get_of_node(domain);

	for (j = 0; j < data->nr_regs; j++) {
		struct mv_irq_wake_table *t = data->wake_table[j];
		int32_t i = 0;

		if (!t) {
			pr_debug("%s: No wake-table available\n", __func__);
			return false;
		}
		while (t[i].reg_id != -1) {
			if (t[i].vector == irq &&
					t[i].parent == of_node) {
				*find = t[i];
				return true;
			}
			i++;
		}
	}
	return false;
}

/*
 * Enable the IRQ node at top interrupt-controller
 */
int32_t mv_irq_mv_set_enable(struct irq_data *data,
		uint32_t irq, uint32_t on)
{
	int32_t retval = 0;
	struct irq_domain *domain = data->domain;
	struct device_node *parent, *child = irq_domain_get_of_node(domain);
	struct mv_irq_chip_data *chipdata = irq_data_get_irq_chip_data(data);
	unsigned level = 0, mv_irq = 0;

	do {
		parent = of_irq_find_parent(child);
		if (parent == child)
			pr_debug("%s: We found top interrupt controller %s\n",
					__func__, child->full_name);
		else {
			pr_debug("%s: IRQ parent available for %s - it's %s\n",
				__func__, child->full_name, parent->full_name);
			child = parent;
			parent = NULL;
		}
		level++;
	} while (parent != child);

	pr_debug("%s: level is %d\n", __func__, level);
	if (level == 1) /* it was an IRQ from top domain */
		mv_irq = irq;
	else if (level == 2) /*  2nd-level */
		mv_irq = chipdata->table[irq];
	else {
		/* No mv hypercall needed level > 2
		 * For PMU (3rd-level) the wake-up is handled
		 * with WUP_DBB (a 2nd-level interrupt)
		 */
		pr_debug("%s: 3rd-level interrupt...\n", __func__);
		return 0;
	}

#ifdef CONFIG_X86_VM_GUEST
	pr_debug("%s: do mv_pm_control(PM_WAKEUP_CONTROL, %d, %d)\n",
			__func__, mv_irq, on);
	retval = mv_svc_pm_control(PM_WAKEUP_CONTROL, mv_irq, on, 0);
	if (retval)
		pr_err("%s: mv_svc_pm_control returns %d\n", __func__, retval);
#endif
	return retval;
}

/*
 * Set wake Entry for most for the mv IRQ domains
 */
int32_t mv_irq_set_wake(struct irq_data *data, uint32_t irq, uint32_t on,
		uint32_t wake_id)
{
	struct mv_irq_wake_data *pdata = get_irq_wake_data(wake_id);
	struct irq_domain *domain = data->domain;
	struct mv_irq_wake_table irq_wake;
	int32_t i, val, ret = 0;
	uint32_t loop, addr;

	if (!pdata) {
		pr_debug("%s: no wake data available for wake_id:%d\n",
				__func__, wake_id);
		return 0;
	}
	loop = pdata->only_cpu0 ? 1 : pdata->nr_cores;
	if (mv_irq_is_wake_capable(domain, pdata, irq, &irq_wake)) {
		for (i = 0; i < loop; i++) {
			addr = pdata->wake_regs[i][irq_wake.reg_id];
			val = mv_irq_wake_read(pdata, addr);
			pr_debug("%s: wake capability %s for irq %d (%s)\n",
					__func__, on ? "enabled" : "disabled",
					irq, irq_wake.parent->full_name);
			if (on)
				set_bit(irq_wake.offset, (ulong *)&val);
			else
				clear_bit(irq_wake.offset, (ulong *)&val);
			mv_irq_wake_write(pdata, addr, val);
		}
		ret = mv_irq_mv_set_enable(data, irq, on);
	} else {
		/* A driver is enabling its interrupt as wake-up source.
		 * This interrupt is not defined as wake-capable
		 *  in irq-wake tables. Return the function with no error
		 * */
		pr_debug("%s: %d is not a known wake-up capable IRQ\n",
				__func__, irq);
		return 0;
	}
	return ret;
}
EXPORT_SYMBOL(mv_irq_set_wake);

/*
 * Get intel,wake-only-cpu-0 property if any from dts
 */
static bool mv_irq_get_cpu_wake_capable(struct device_node *np)
{
	if (of_find_property(np, "intel,wake-only-cpu-0", NULL))
		return true;
	return false;
}

#define mv_IRQ_WAKE_DRV_DATA(ID) ((kernel_ulong_t)&wake_pdata[ID])
static const struct of_device_id mv_irq_wake_of_match[] = {
	{
		.compatible = "intel,mv_irq_wake",
		.data = (void *)mv_IRQ_WAKE_DRV_DATA(WAKE_ID_DBB)
	},
	{}
};
MODULE_DEVICE_TABLE(of, mv_irq_wake_of_match);

static inline struct mv_irq_wake_data *mv_irq_wake_get_driver_data(
			struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	if (np) {
		const struct of_device_id *match;

		match = of_match_node(mv_irq_wake_of_match,
				pdev->dev.of_node);
		if (!match)
			return NULL;
		return (struct mv_irq_wake_data *)match->data;
	}
	return NULL;
}

/*
 *	Interrupt handler for WUP_DBB
 */
static irqreturn_t wup_dbb(int irq, void *dev_id)
{
	pr_debug("%s(%d)\n", __func__, irq);
	return IRQ_HANDLED;
}

/*
 * Platform Driver Entry
 */
static int32_t mv_irq_wake_probe(struct platform_device *pdev)
{
	struct device *_dev = &pdev->dev;
	struct device_node *np = _dev->of_node;
	struct resource *mem_res;
	struct resource *res[NR_OF_CORES][NR_OF_REGS];
	struct mv_irq_wake_data *data;
	char comp[MAX_WAKE_LENGTH];
	uint32_t i, j;
	int32_t ret;

	dev_dbg(_dev, "Probe\n");

	data = mv_irq_wake_get_driver_data(pdev);
	if (!data)
		return -ENOMEM;
	platform_set_drvdata(pdev, data);

	/* Get wake capable cpu */
	data->only_cpu0 = mv_irq_get_cpu_wake_capable(np);
	dev_dbg(_dev, "%s would be woken-up\n",
			data->only_cpu0 ? "cpu0" : "all cpu");

	/* Get io access master */
	data->io_master = mv_irq_get_io_master(np);
	dev_dbg(_dev, "%s will do the io access\n",
		data->io_master == IRQ_IO_ACCESS_BY_VMM ? "VMM" : "LINUX");

	/* Get io resources */
	mem_res = platform_get_resource_byname(pdev,
			IORESOURCE_MEM, "spcu");
	if (!mem_res)
		return -EINVAL;
	data->base_phys = mem_res->start;

	/* Get pcl resources */
	data->pinctrl = devm_pinctrl_get(_dev);
	if (IS_ERR(data->pinctrl)) {
		dev_err(_dev, "devm_pinctrl_get fails...\n");
		return -EINVAL;
	}
	data->pins_default =
		pinctrl_lookup_state(data->pinctrl, "default");
	if (IS_ERR(data->pins_default))
		dev_dbg(_dev, "No default pinstate defined\n");
	data->pins_inactive =
		pinctrl_lookup_state(data->pinctrl, "inactive");
	if (IS_ERR(data->pins_inactive))
		dev_dbg(_dev, "No inactive pinstate defined\n");

	/* Get IRQ resources */
	data->irq = platform_get_irq_byname(pdev, "wup_dbb");
	if (!IS_ERR_VALUE(data->irq)) {
		ret = devm_request_irq(_dev, data->irq, wup_dbb,
				IRQF_SHARED | IRQF_NO_SUSPEND, "wup_dbb", data);
		if (ret) {
			dev_err(_dev, "Setup irq%d failed with ret = %d\n",
				data->irq, ret);
			return -EINVAL;
		}
		/* Enable wake-up capability */
		ret = device_init_wakeup(_dev, true);
		if (ret) {
			free_irq(data->irq, data);
			dev_err(_dev, "device_init_wakeup failed\n");
			return -EINVAL;
		}
	} else
		dev_dbg(_dev, "No irq resource\n");

	for (i = 0; i < NR_OF_CORES; i++) {
		for (j = 0; j < NR_OF_REGS; j++) {
			snprintf(comp, MAX_WAKE_LENGTH, "wake-C%d-R%d", i, j);
			res[i][j] = platform_get_resource_byname(pdev,
				IORESOURCE_MEM, comp);
			if (!res[i][j])
				dev_dbg(_dev, "No wake resource for %s\n",
						comp);
			dev_dbg(_dev, "%s: %pR\n", comp, res[i][j]);
		}
	}

	/* Get numbers HW Wake resources (cores and regs)  */
	for (i = 0; i < NR_OF_CORES; i++)
		if (res[i][0])
			data->nr_cores++;
	for (i = 0; i < NR_OF_REGS; i++)
		if (res[0][i])
			data->nr_regs++;
	dev_dbg(_dev, "HW Wake resources: core:%d - regs:%d\n",
			data->nr_cores, data->nr_regs);

	data->base_virt = ioremap(mem_res->start, resource_size(mem_res));
	if (!data->base_virt) {
		dev_err(_dev, "SPCU IO remap operation failed\n");
		if (data->irq)
			free_irq(data->irq, data);
		return -ENODEV;
	}

	for (i = 0; i < data->nr_cores; i++)
		for (j = 0; j < data->nr_regs; j++) {
			data->wake_regs[i][j] = res[i][j]->start;
			dev_dbg(_dev, "virt: wake_regs[%d:%d]:%p\n",
				i, j, data->base_virt + data->wake_regs[i][j]);
			dev_dbg(_dev, "phys: wake_regs[%d:%d]:%#x\n",
				i, j, data->base_phys + data->wake_regs[i][j]);
		}

	for (i = 0; i < data->nr_regs; i++) {
		ret = mv_irq_wake_fill_table(np, i, &data->wake_table[i]);
		if (ret) {
			dev_err(_dev, "Filling wake-table-%d failed\n", i);
			if (data->irq)
				free_irq(data->irq, data);
			return ret;
		}
	}
	return 0;
}

/*
 *	Platform Driver Exit
 */
static int32_t mv_irq_wake_remove(struct platform_device *pdev)
{
	uint32_t j;
	struct mv_irq_wake_data *data = platform_get_drvdata(pdev);
	struct device *_dev = &pdev->dev;

	dev_dbg(_dev, "remove\n");
	if (data) {
		for (j = 0; j < data->nr_regs; j++)
			kfree(data->wake_table[j]);
		iounmap(data->base_virt);
		if (data->irq)
			free_irq(data->irq, data);
	} else
		dev_err(_dev, "pdata is NULL - freeing not possible\n");
	return 0;
}

#ifdef CONFIG_PM
/*
 *	Platform Driver Suspend
 */
static int32_t mv_irq_wake_suspend(struct platform_device *pdev,
					  pm_message_t state)
{
	int32_t ret = 0;
	struct mv_irq_wake_data *data = platform_get_drvdata(pdev);
	struct device *_dev = &pdev->dev;

	dev_dbg(_dev, "suspend\n");

	if (!data) {
		dev_err(_dev, "data is %p ..return\n", data);
		return -1;
	}

	/* Prepare for suspend with pad active... */
	if (!IS_ERR(data->pins_default)) {
		ret = pinctrl_select_state(data->pinctrl, data->pins_default);
		if (ret)
			dev_err(_dev, "irq_wake pads active failed\n");
	}
	if (device_may_wakeup(_dev))
		if (data->irq)
			enable_irq_wake(data->irq);

	return ret;
}

/*
 * Platform Driver Resume
 */
static int32_t mv_irq_wake_resume(struct platform_device *pdev)
{
	int32_t ret = 0;
	struct mv_irq_wake_data *data = platform_get_drvdata(pdev);
	struct device *_dev = &pdev->dev;

	dev_dbg(_dev, "resume\n");

	if (!data) {
		dev_err(_dev, "data is %p ..return\n", data);
		return -1;
	}

	/* Prepare for resume with pad inactive... */
	if (!IS_ERR(data->pins_inactive)) {
		ret = pinctrl_select_state(data->pinctrl, data->pins_inactive);
		if (ret)
			dev_err(_dev, "irq_wake pads inactive failed\n");
	}
	if (device_may_wakeup(_dev))
		if (data->irq)
			disable_irq_wake(data->irq);
	return ret;
}
#endif

static struct platform_driver mv_irq_wake_driver = {
	.driver = {
		.name = "mv_irq_wake",
		.of_match_table = of_match_ptr(mv_irq_wake_of_match),
	},
	.probe = mv_irq_wake_probe,
	.remove = mv_irq_wake_remove,
#ifdef CONFIG_PM
	.resume = mv_irq_wake_resume,
	.suspend = mv_irq_wake_suspend,
#endif
};

static int32_t __init mv_irq_wake_init(void)
{
	int32_t ret = platform_driver_register(&mv_irq_wake_driver);

	if (ret) {
		pr_err("%s: Unable to register platform driver\n", __func__);
		return ret;
	}
	pr_debug("%s: Platform driver registration ok...\n", __func__);
	return 0;
}

static void __exit mv_irq_wake_exit(void)
{
	platform_driver_unregister(&mv_irq_wake_driver);
	pr_debug("%s: Platform driver unregistration ok...\n", __func__);
}

module_init(mv_irq_wake_init);
module_exit(mv_irq_wake_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("mv_irq_wake driver");

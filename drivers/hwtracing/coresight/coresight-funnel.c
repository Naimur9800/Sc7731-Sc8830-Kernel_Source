/* Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/pm_runtime.h>
#include <linux/coresight.h>
#include <linux/amba/bus.h>
#include <linux/clk.h>
#include <linux/of.h>

#include "coresight-priv.h"

#define FUNNEL_FUNCTL		0x000
#define FUNNEL_PRICTL		0x004

#define FUNNEL_HOLDTIME_MASK	0xf00
#define FUNNEL_HOLDTIME_SHFT	0x8
#define FUNNEL_HOLDTIME		(0x7 << FUNNEL_HOLDTIME_SHFT)

/**
 * struct funnel_drvdata - specifics associated to a funnel component
 * @base:	memory mapped base address for this component.
 * @dev:	the device entity associated to this component.
 * @atclk:	optional clock for the core parts of the funnel.
 * @csdev:	component vitals needed by the framework.
 * @priority:	port selection order.
 * @spinlock:   Only one at a time pls.
 * @inport:	The open inport of funnel device..
 */
struct funnel_drvdata {
	void __iomem		*base;
	struct device		*dev;
	struct clk		*atclk;
	struct coresight_device	*csdev;
	unsigned long		priority;
	int			inport;
	struct notifier_block	nb;
};

static void funnel_enable_hw(struct funnel_drvdata *drvdata, int port)
{
	u32 functl;

	CS_UNLOCK(drvdata->base);

	functl = readl_relaxed(drvdata->base + FUNNEL_FUNCTL);
	functl &= ~FUNNEL_HOLDTIME_MASK;
	functl |= FUNNEL_HOLDTIME;
	functl |= (1 << port);
	writel_relaxed(functl, drvdata->base + FUNNEL_FUNCTL);
	writel_relaxed(drvdata->priority, drvdata->base + FUNNEL_PRICTL);

	CS_LOCK(drvdata->base);
}

static int funnel_enable(struct coresight_device *csdev, int inport,
			 int outport)
{
	struct funnel_drvdata *drvdata = dev_get_drvdata(csdev->dev.parent);

	pm_runtime_get_sync(drvdata->dev);
	funnel_enable_hw(drvdata, inport);
	drvdata->inport |= (1 << inport);

	dev_info(drvdata->dev, "FUNNEL inport %d enabled\n", inport);
	return 0;
}

static void funnel_disable_hw(struct funnel_drvdata *drvdata, int inport)
{
	u32 functl;

	CS_UNLOCK(drvdata->base);

	functl = readl_relaxed(drvdata->base + FUNNEL_FUNCTL);
	functl &= ~(1 << inport);
	writel_relaxed(functl, drvdata->base + FUNNEL_FUNCTL);

	CS_LOCK(drvdata->base);
}

static void funnel_disable(struct coresight_device *csdev, int inport,
			   int outport)
{
	struct funnel_drvdata *drvdata = dev_get_drvdata(csdev->dev.parent);

	funnel_disable_hw(drvdata, inport);
	drvdata->inport &= ~(1 << inport);
	pm_runtime_put(drvdata->dev);

	dev_info(drvdata->dev, "FUNNEL inport %d disabled\n", inport);
}

static const struct coresight_ops_link funnel_link_ops = {
	.enable		= funnel_enable,
	.disable	= funnel_disable,
};

static const struct coresight_ops funnel_cs_ops = {
	.link_ops	= &funnel_link_ops,
};

static int funnel_find_source(struct coresight_device *csdev,
	int cpu)
{
	struct coresight_device *parent_csdev;
	struct coresight_connection *conns;
	int i, ret;

	for (i = csdev->nr_outport; i < csdev->nr_outport +
		csdev->nr_inport; i++) {
		conns = &csdev->conns[i];
		if (conns->parent_dev != NULL) {
			parent_csdev = conns->parent_dev;
			/* Check the parent device */
			if (parent_csdev->type == CORESIGHT_DEV_TYPE_SOURCE) {
				/* It is the source device. */
				if ((parent_csdev->enable == true) &&
					(parent_csdev->cpu == cpu)) {
					/* It is source dev, and enabled,
					 * it needs to enable the pach.
					 */
					return 0;
				}
				/* Check next inport */
				continue;
			} else {
				/* It is't source device,
				 * needs to recursion check.
				 */
				ret = funnel_find_source(conns->parent_dev,
					 cpu);
				if (ret != -1)
					/* Get the enable source. */
					return ret;
			}
		} else {
			/* It is first device, but not source device. */
			return -1;
		}
	}
	/* Do not get enabled source device. */
	return -1;
}

static ssize_t priority_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct funnel_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val = drvdata->priority;

	return sprintf(buf, "%#lx\n", val);
}

static ssize_t priority_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	int ret;
	unsigned long val;
	struct funnel_drvdata *drvdata = dev_get_drvdata(dev->parent);

	ret = kstrtoul(buf, 16, &val);
	if (ret)
		return ret;

	drvdata->priority = val;
	return size;
}
static DEVICE_ATTR_RW(priority);

static u32 get_funnel_ctrl_hw(struct funnel_drvdata *drvdata)
{
	u32 functl;

	CS_UNLOCK(drvdata->base);
	functl = readl_relaxed(drvdata->base + FUNNEL_FUNCTL);
	CS_LOCK(drvdata->base);

	return functl;
}

static ssize_t funnel_ctrl_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	u32 val;
	struct funnel_drvdata *drvdata = dev_get_drvdata(dev->parent);

	pm_runtime_get_sync(drvdata->dev);

	val = get_funnel_ctrl_hw(drvdata);

	pm_runtime_put(drvdata->dev);

	return sprintf(buf, "%#x\n", val);
}
static DEVICE_ATTR_RO(funnel_ctrl);

static struct attribute *coresight_funnel_attrs[] = {
	&dev_attr_funnel_ctrl.attr,
	&dev_attr_priority.attr,
	NULL,
};
ATTRIBUTE_GROUPS(coresight_funnel);

static int funnel_cpu_pm_notify(struct notifier_block *self,
				    unsigned long action, void *hcpu)
{
	int inport, eb_port, port_lgcnum, this_cpu = 0;
	struct funnel_drvdata *pdata =
		container_of(self, struct funnel_drvdata, nb);

	this_cpu = raw_smp_processor_id();

	switch (action) {
	case CPU_PM_ENTER:
		break;
	case CPU_PM_ENTER_FAILED:
	case CPU_PM_EXIT:
		if (pdata->inport) {
			inport = funnel_find_source(pdata->csdev, this_cpu);
			if (!inport) {
				eb_port = pdata->inport;
				while (eb_port) {
					port_lgcnum = __ffs(eb_port);
					eb_port &= (eb_port - 1);
					funnel_enable_hw(pdata, port_lgcnum);
				}
			}
		}
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block funnel_cpu_pm_notifier = {
	.notifier_call = funnel_cpu_pm_notify,
};

static int funnel_probe(struct amba_device *adev, const struct amba_id *id)
{
	int ret;
	void __iomem *base;
	struct device *dev = &adev->dev;
	struct coresight_platform_data *pdata = NULL;
	struct funnel_drvdata *drvdata;
	struct resource *res = &adev->res;
	struct coresight_desc *desc;
	struct device_node *np = adev->dev.of_node;

	if (np) {
		pdata = of_get_coresight_platform_data(dev, np);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
		adev->dev.platform_data = pdata;
	}

	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->dev = &adev->dev;
	drvdata->atclk = devm_clk_get(&adev->dev, "atclk"); /* optional */
	if (!IS_ERR(drvdata->atclk)) {
		ret = clk_prepare_enable(drvdata->atclk);
		if (ret)
			return ret;
	}
	dev_set_drvdata(dev, drvdata);

	/* Validity for the resource is already checked by the AMBA core */
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	drvdata->base = base;
	pm_runtime_put(&adev->dev);

	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	desc->type = CORESIGHT_DEV_TYPE_LINK;
	desc->subtype.link_subtype = CORESIGHT_DEV_SUBTYPE_LINK_MERG;
	desc->ops = &funnel_cs_ops;
	desc->pdata = pdata;
	desc->dev = dev;
	desc->groups = coresight_funnel_groups;
	drvdata->csdev = coresight_register(desc);
	drvdata->nb = funnel_cpu_pm_notifier;
	if (IS_ERR(drvdata->csdev))
		return PTR_ERR(drvdata->csdev);
	cpu_pm_register_notifier(&drvdata->nb);

	dev_info(dev, "FUNNEL initialized\n");
	return 0;
}

static int funnel_remove(struct amba_device *adev)
{
	struct funnel_drvdata *drvdata = amba_get_drvdata(adev);

	cpu_pm_unregister_notifier(&drvdata->nb);
	coresight_unregister(drvdata->csdev);
	return 0;
}

#ifdef CONFIG_PM
static int funnel_runtime_suspend(struct device *dev)
{
	struct funnel_drvdata *drvdata = dev_get_drvdata(dev);

	if (drvdata && !IS_ERR(drvdata->atclk))
		clk_disable_unprepare(drvdata->atclk);

	return 0;
}

static int funnel_runtime_resume(struct device *dev)
{
	struct funnel_drvdata *drvdata = dev_get_drvdata(dev);

	if (drvdata && !IS_ERR(drvdata->atclk))
		clk_prepare_enable(drvdata->atclk);

	return 0;
}
#endif

static const struct dev_pm_ops funnel_dev_pm_ops = {
	SET_RUNTIME_PM_OPS(funnel_runtime_suspend, funnel_runtime_resume,
		 NULL)
};

static struct amba_id funnel_ids[] = {
	{
		.id     = 0x0003b908,
		.mask   = 0x0003ffff,
	},
	{ 0, 0},
};

static struct amba_driver funnel_driver = {
	.drv = {
		.name	= "coresight-funnel",
		.owner	= THIS_MODULE,
		.pm	= &funnel_dev_pm_ops,
	},
	.probe		= funnel_probe,
	.remove		= funnel_remove,
	.id_table	= funnel_ids,
};

module_amba_driver(funnel_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CoreSight Funnel driver");

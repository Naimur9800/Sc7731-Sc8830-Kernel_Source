/*
 * Spreadtrum SoCs Reset Controller driver
 *
 * Copyright 2015 Spreadtrum
 *
 * Baolin Wang <baolin.wang@spreadtrum.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#define SPRD_REG_BITS	(32)

struct sprd_reset_dev {
	struct reset_controller_dev	rcdev;
	spinlock_t			lock;
	void __iomem			*membase;
};

static struct sprd_reset_dev *
rcdev_to_sprd_dev(struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct sprd_reset_dev, rcdev);
}

static int sprd_reset(struct reset_controller_dev *rcdev,
		      unsigned long id)
{
	struct sprd_reset_dev *data = rcdev_to_sprd_dev(rcdev);
	int bank = id / SPRD_REG_BITS;
	int bank_len = SPRD_REG_BITS / 8;
	int offset = id % SPRD_REG_BITS;
	unsigned long flags, reg;

	spin_lock_irqsave(&data->lock, flags);

	reg = readl_relaxed(data->membase + (bank * bank_len));
	writel_relaxed(reg | BIT(offset), data->membase + (bank * bank_len));
	udelay(10);
	writel_relaxed(reg & ~BIT(offset), data->membase + (bank * bank_len));

	spin_unlock_irqrestore(&data->lock, flags);

	return 0;
}

static int sprd_reset_assert(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	return 0;
}

static int sprd_reset_deassert(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	return 0;
}

static struct reset_control_ops sprd_reset_ops = {
	.reset		= sprd_reset,
	.assert		= sprd_reset_assert,
	.deassert	= sprd_reset_deassert,
};

static int sprd_reset_probe(struct platform_device *pdev)
{
	struct sprd_reset_dev *data;
	struct resource *res;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->membase))
		return PTR_ERR(data->membase);

	spin_lock_init(&data->lock);

	data->rcdev.owner = THIS_MODULE;
	data->rcdev.nr_resets = resource_size(res) * 8;
	data->rcdev.ops = &sprd_reset_ops;
	data->rcdev.of_node = pdev->dev.of_node;

	return reset_controller_register(&data->rcdev);
}

static int sprd_reset_remove(struct platform_device *pdev)
{
	struct sprd_reset_dev *data = platform_get_drvdata(pdev);

	reset_controller_unregister(&data->rcdev);

	return 0;
}

static const struct of_device_id sprd_reset_dt_ids[] = {
	 { .compatible = "sprd,ap-ahb-reset", "sprd,aon-apb-reset"},
	 { /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, sprd_reset_dt_ids);

static struct platform_driver sprd_reset_driver = {
	.probe	= sprd_reset_probe,
	.remove	= sprd_reset_remove,
	.driver = {
		.name		= "sprd-reset",
		.owner		= THIS_MODULE,
		.of_match_table	= sprd_reset_dt_ids,
	},
};
module_platform_driver(sprd_reset_driver);

MODULE_AUTHOR("Baolin Wang <baolin.wang@spreadtrum.com");
MODULE_DESCRIPTION("Spreadtrum SoCs Reset Controller Driver");
MODULE_LICENSE("GPL");

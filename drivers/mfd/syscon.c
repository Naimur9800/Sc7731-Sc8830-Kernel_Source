/*
 * System Control Driver
 *
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * Copyright (C) 2012 Linaro Ltd.
 *
 * Author: Dong Aisheng <dong.aisheng@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_data/syscon.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/slab.h>
#include <linux/hwspinlock.h>

static struct platform_driver syscon_driver;

static DEFINE_SPINLOCK(syscon_list_slock);
static LIST_HEAD(syscon_list);

struct syscon {
	struct device_node *np;
	struct regmap *regmap;
	unsigned int hwspinlock_id;
	void *hwspinlock_arg;
	struct list_head list;
};

static struct regmap_config syscon_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
};

static unsigned long syscon_hwlock_flag;
#define HWSPINLOCK_TIMEOUT		(~0U)
static void syscon_lock(void *__lock)
{
	int ret = 0;

	ret = hwspin_lock_timeout_irqsave((struct hwspinlock *)__lock,
					  HWSPINLOCK_TIMEOUT,
					  &syscon_hwlock_flag);
	if (ret)
		pr_err("Syscon:lock the hwlock failed.\n");
}

static void syscon_unlock(void *__lock)
{
	hwspin_unlock_irqrestore((struct hwspinlock *)__lock,
				 &syscon_hwlock_flag);
}

static int syscon_get_hwspinlock_id(struct device_node *np,
				    struct of_phandle_args *hwlock_args)
{
	if (of_parse_phandle_with_args(np, "hwlocks", "#hwlock-cells", 0,
				       hwlock_args))
		return -ENODEV;

	return 0;
}

static struct syscon *of_syscon_register(struct device_node *np)
{
	struct syscon *syscon;
	struct resource res;
	struct regmap *regmap;
	void __iomem *base;
	int ret;
	struct regmap_config syscon_config = syscon_regmap_config;
	struct of_phandle_args hwlock_id;

	if (!of_device_is_compatible(np, "syscon"))
		return ERR_PTR(-EINVAL);

	syscon = kzalloc(sizeof(*syscon), GFP_KERNEL);
	if (!syscon)
		return ERR_PTR(-ENOMEM);

	if (of_address_to_resource(np, 0, &res)) {
		ret = -ENOMEM;
		goto err_map;
	}

	base = of_iomap(np, 0);
	if (!base) {
		ret = -ENOMEM;
		goto err_map;
	}

	/* Parse the device's DT node for an endianness specification */
	if (of_property_read_bool(np, "big-endian"))
		syscon_config.val_format_endian = REGMAP_ENDIAN_BIG;
	else if (of_property_read_bool(np, "little-endian"))
		syscon_config.val_format_endian = REGMAP_ENDIAN_LITTLE;
	if (of_property_read_bool(np, "setclr-offset"))
		of_property_read_s32(np, "setclr-offset",
			&syscon_config.setclr_offset);
	else
		syscon_config.setclr_offset = 0x1000;
	syscon_config.reg_addr = res.start;
	if (syscon_get_hwspinlock_id(np, &hwlock_id) == 0) {
		syscon_config.hw_lock_arg =
			of_hwspin_lock_request(np, "syscon");
		if (!syscon_config.hw_lock_arg)
			syscon_config.hw_lock_arg =
			syscon_regmap_config.hw_lock_arg;
		syscon_config.hw_lock = syscon_lock;
		syscon_config.hw_unlock = syscon_unlock;
		syscon->hwspinlock_id = hwlock_id.args[0];
		syscon->hwspinlock_arg = syscon_config.hw_lock_arg;
	}

	regmap = regmap_init_mmio(NULL, base, &syscon_config);
	if (IS_ERR(regmap)) {
		pr_err("regmap init failed\n");
		ret = PTR_ERR(regmap);
		goto err_regmap;
	}

	syscon->regmap = regmap;
	syscon->np = np;

	spin_lock(&syscon_list_slock);
	list_add_tail(&syscon->list, &syscon_list);
	spin_unlock(&syscon_list_slock);

	return syscon;

err_regmap:
	iounmap(base);
err_map:
	kfree(syscon);
	return ERR_PTR(ret);
}

struct regmap *syscon_node_to_regmap(struct device_node *np)
{
	struct syscon *entry, *syscon = NULL;
	struct of_phandle_args hwlock_id;
	int hwlock_flag = syscon_get_hwspinlock_id(np, &hwlock_id);

	spin_lock(&syscon_list_slock);
	list_for_each_entry(entry, &syscon_list, list) {
		if (hwlock_flag == 0)
			if(entry->hwspinlock_id == hwlock_id.args[0]) {
				syscon_regmap_config.hw_lock_arg =
					entry->hwspinlock_arg;
			}
		if (entry->np == np) {
			syscon = entry;
			break;
		}
	}

	spin_unlock(&syscon_list_slock);

	if (!syscon)
		syscon = of_syscon_register(np);

	if (IS_ERR(syscon))
		return ERR_CAST(syscon);

	return syscon->regmap;
}
EXPORT_SYMBOL_GPL(syscon_node_to_regmap);

struct regmap *syscon_regmap_lookup_by_compatible(const char *s)
{
	struct device_node *syscon_np;
	struct regmap *regmap;

	syscon_np = of_find_compatible_node(NULL, NULL, s);
	if (!syscon_np)
		return ERR_PTR(-ENODEV);

	regmap = syscon_node_to_regmap(syscon_np);
	of_node_put(syscon_np);

	return regmap;
}
EXPORT_SYMBOL_GPL(syscon_regmap_lookup_by_compatible);

static int syscon_match_pdevname(struct device *dev, void *data)
{
	return !strcmp(dev_name(dev), (const char *)data);
}

struct regmap *syscon_regmap_lookup_by_pdevname(const char *s)
{
	struct device *dev;
	struct syscon *syscon;

	dev = driver_find_device(&syscon_driver.driver, NULL, (void *)s,
				 syscon_match_pdevname);
	if (!dev)
		return ERR_PTR(-EPROBE_DEFER);

	syscon = dev_get_drvdata(dev);

	return syscon->regmap;
}
EXPORT_SYMBOL_GPL(syscon_regmap_lookup_by_pdevname);

struct regmap *syscon_regmap_lookup_by_phandle(struct device_node *np,
					const char *property)
{
	struct device_node *syscon_np;
	struct regmap *regmap;

	if (property)
		syscon_np = of_parse_phandle(np, property, 0);
	else
		syscon_np = np;

	if (!syscon_np)
		return ERR_PTR(-ENODEV);

	regmap = syscon_node_to_regmap(syscon_np);
	of_node_put(syscon_np);

	return regmap;
}
EXPORT_SYMBOL_GPL(syscon_regmap_lookup_by_phandle);

static int syscon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct syscon_platform_data *pdata = dev_get_platdata(dev);
	struct syscon *syscon;
	struct resource *res;
	void __iomem *base;

	syscon = devm_kzalloc(dev, sizeof(*syscon), GFP_KERNEL);
	if (!syscon)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOENT;

	base = devm_ioremap(dev, res->start, resource_size(res));
	if (!base)
		return -ENOMEM;

	syscon_regmap_config.max_register = res->end - res->start - 3;
	if (pdata)
		syscon_regmap_config.name = pdata->label;
	syscon->regmap = devm_regmap_init_mmio(dev, base,
					&syscon_regmap_config);
	if (IS_ERR(syscon->regmap)) {
		dev_err(dev, "regmap init failed\n");
		return PTR_ERR(syscon->regmap);
	}

	platform_set_drvdata(pdev, syscon);

	dev_dbg(dev, "regmap %pR registered\n", res);

	return 0;
}

static const struct platform_device_id syscon_ids[] = {
	{ "syscon", },
	{ }
};

static struct platform_driver syscon_driver = {
	.driver = {
		.name = "syscon",
	},
	.probe		= syscon_probe,
	.id_table	= syscon_ids,
};

static int __init syscon_init(void)
{
	return platform_driver_register(&syscon_driver);
}
postcore_initcall(syscon_init);

static void __exit syscon_exit(void)
{
	platform_driver_unregister(&syscon_driver);
}
module_exit(syscon_exit);

MODULE_AUTHOR("Dong Aisheng <dong.aisheng@linaro.org>");
MODULE_DESCRIPTION("System Control driver");
MODULE_LICENSE("GPL v2");

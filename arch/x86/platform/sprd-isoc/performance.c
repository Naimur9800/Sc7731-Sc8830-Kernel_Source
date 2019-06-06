/*
 * Support for performance management features of the SPRD iSoC
 *
 *
 * Copyright (C) 2016 Intel Corporation
 * Author Miguel Vadillo <miguel.vadillo@intel.com>
 *
 * This file is released under the GPLv2.
 */

#include <linux/pm.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/syscore_ops.h>
#include "performance.h"

#define BIA_PERF_DT_NODE	"perf-units"

struct bia_perf_settings {
	const char *name;
	u32 port;
	u32 total_reg;
	u32 *reg_val;
};

static struct bia_perf_settings *bia_perf_list;
static u32 total_units;

static void bia_perf_configure(struct bia_perf_settings *list)
{
	int i, j;

	for (i = 0; i < total_units; i++, list++) {
		for (j = 0; j < list->total_reg; j++) {
			pr_debug("%s: Port %d Offset[0x%x] = Msk[0x%08x] Val[0x%08x]\n",
				__func__, list->port, list->reg_val[j*3],
				list->reg_val[j*3+1], list->reg_val[j*3+2]);
			RORW_HOST_IOSF(list->port, list->reg_val[j*3],
				list->reg_val[j*3+1], list->reg_val[j*3+2]);
		}
	}
}

static int bia_perf_register_dt(struct platform_device *pdev)
{
	struct device_node *dev_np;
	struct device_node *child_np;
	struct bia_perf_settings *bia_perf;
	u32 port, cnt;
	int num;
	u32 *rv;
	const char *name;

	dev_np = of_find_node_by_name(pdev->dev.of_node, BIA_PERF_DT_NODE);
	if (!dev_np)
		return -ENODEV;

	cnt = of_get_child_count(dev_np);
	if (!cnt) {
		dev_err(&pdev->dev, "failed to find childs\n");
		return -ENODEV;
	}

	total_units = cnt;
	bia_perf_list = devm_kzalloc(&pdev->dev,
				cnt * sizeof(struct bia_perf_settings),
				GFP_KERNEL);

	if (!bia_perf_list) {
		dev_err(&pdev->dev,
			"failed to allocate memory for list\n");
		return -ENOMEM;
	}
	bia_perf = bia_perf_list;

	for_each_available_child_of_node(dev_np, child_np) {
		if (IS_ERR(child_np)) {
			dev_err(&pdev->dev, "failed to find next_np\n");
			continue;
		}
		of_property_read_string(child_np, "name", &name);
		of_property_read_u32(child_np, "port", &port);
		num = of_property_count_elems_of_size(child_np,
					"reg-msk-val",
					3 * sizeof(u32));

		bia_perf->name = name;
		bia_perf->port = port;
		bia_perf->total_reg = num < 0 ? 0 : num;
		dev_info(&pdev->dev, "Unit %s Port %d...\n",
				bia_perf->name, bia_perf->port);
		if (bia_perf->total_reg) {
			dev_info(&pdev->dev, "Total Registers %d...\n",
				bia_perf->total_reg);

			bia_perf->reg_val = devm_kzalloc(&pdev->dev,
					 bia_perf->total_reg * 3 * sizeof(rv),
					 GFP_KERNEL);
			if (!bia_perf->reg_val) {
				dev_err(&pdev->dev,
					"failed to allocate memory for reg_val\n");
				goto no_memory;
			}
			rv = bia_perf->reg_val;
			of_property_read_u32_array(child_np,
						"reg-msk-val",
						rv,
						bia_perf->total_reg * 3);
		}
		bia_perf++;
	}

	return 0;

no_memory:
	return -ENOMEM;
}

#ifdef CONFIG_PM
static void bia_perf_resume(void)
{
	return bia_perf_configure(bia_perf_list);
}

static struct syscore_ops bia_perf_ops = {
	.resume = bia_perf_resume,
};
#endif

static int bia_perf_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;

	if (!np) {
		dev_err(&pdev->dev, "bia performance node not found!!\n");
			return -ENODEV;
	}

	ret = bia_perf_register_dt(pdev);
	if (ret) {
		dev_err(&pdev->dev, "probe bia perf failed! %d\n", ret);
		return ret;
	}

	bia_perf_configure(bia_perf_list);

#ifdef CONFIG_PM
	register_syscore_ops(&bia_perf_ops);
#endif
	dev_info(&pdev->dev, "%s finish!\n", __func__);

	return 0;
}

static int bia_perf_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id bia_perf_of_match[] = {
	{ .compatible = "sprd,bia-performance-settings", },
	{},
};
#else
#define bia_perf_of_match NULL
#endif

static struct platform_driver bia_perf_pdriver = {
	.driver	= {
		.name	= "bia_performance",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(bia_perf_of_match),
	},
	.probe = bia_perf_probe,
	.remove = bia_perf_remove
};

static int __init bia_perf_driver_init(void)
{
	return platform_driver_register(&bia_perf_pdriver);
}

arch_initcall(bia_perf_driver_init);

MODULE_AUTHOR("Miguel Vadillo <imiguel.vadillo@intel.com>");
MODULE_DESCRIPTION("BIA Performance Settings");
MODULE_LICENSE("GPL v2");

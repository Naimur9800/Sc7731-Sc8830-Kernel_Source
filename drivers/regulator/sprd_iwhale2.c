/*
 * Copyright (C) 2016 Spreadtrum Communications Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sort.h>
#include <linux/syscore_ops.h>
#include <linux/version.h>
#include <linux/mfd/sprd/pmic_glb_reg.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>

#define REGULAOR_DT_NODE	"regulators"
#define REGULATOR_ROOT_DIR	"Ddie-ldo"

struct sprd_regulator_data {
	struct regulator_dev *rdev;
};

struct sprd_regulator_regs {
	unsigned long pd_set;
	u32 pd_set_bit;
};

struct sprd_regulator_desc {
	struct regulator_desc desc;
	struct regulator_init_data *init_data;
	struct dentry *debug_glb_g2;
	struct sprd_regulator_regs regs;
	struct sprd_regulator_data data;
	struct dentry *debugfs;
	struct regmap *anlg_glb;
};

typedef struct {
	char regu_name[15];
	int index;
	unsigned long pd_reg;
	u32 pd_mask;
} ldo_config;

ldo_config reg_info_iwhale2[] = {
/* regu_name
 * index pd_reg   pd_mask
 */
{ "ldo_0p84", 0, 0x0068, 0x1 },
{ "ldo_1p2", 1, 0x0068, 0x8},
{ "ldo_1p2_pll",  2, 0x0068, 0x1000},
{ "ldo_1p05_hvm",  3, 0x00A4, 0x1},
{ "ldo_1p05_ifp",  4, 0x00A4, 0x40},
{ "ldo_2p4_ifp",  5, 0x00A4, 0x80},
{}
};

ldo_config reg_info_isharkl2_g2[] = {
/* regu_name
 * index pd_reg   pd_mask
 */
{ "ldo1p2_100m", 0, 0x0090, BIT(7) },
{ "ldo1p05hvm_100m", 1, 0x0098, BIT(15)},
{ "ldo1p05ifp_100m",  2, 0x0098, BIT(7)},
{ "ldo2p4ifp_100m",  3, 0x009C, BIT(7)},
{}
};

ldo_config reg_info_isharkl2_g8[] = {
/* regu_name
 * index pd_reg   pd_mask
 */
{ "ldo1p2_26m", 0, 0x0030, BIT(7) },
{}
};

static atomic_t idx = ATOMIC_INIT(0);
static const struct of_device_id sprd_regulator_of_match[] = {
	{.compatible = "sprd,iwhale2-buildin-regulator",
		.data = (void *)reg_info_iwhale2},
	{.compatible = "sprd,isharkl2-buildin-g2-regulator",
		.data = (void *)reg_info_isharkl2_g2},
	{.compatible = "sprd,isharkl2-buildin-g8-regulator",
		.data = (void *)reg_info_isharkl2_g8},
	{}
};

#ifdef REGU_DEBUG
#define debug(format, arg...) pr_info("regu: ""@@@%s: " format, __func__, ##arg)
#else
#define debug(format, arg...)
#endif

static inline int analog_set_bits(struct regmap *anlg_glb, unsigned long reg,
				  unsigned long bits_mask)
{
	return regmap_update_bits(anlg_glb, reg, bits_mask, bits_mask);
}

static inline int analog_clr_bits(struct regmap *anlg_glb, unsigned long reg,
				  unsigned long bits_mask)
{
	return regmap_update_bits(anlg_glb, reg, bits_mask, 0);
}

static inline int analog_reg_read(struct regmap *anlg_glb, unsigned long reg)
{
	int ret, value;

	ret = regmap_read(anlg_glb, reg, (unsigned int *)&value);
	return ret ? ret : value;
}

static struct sprd_regulator_desc *get_desc(struct regulator_dev *rdev)
{
	return (struct sprd_regulator_desc *)rdev->desc;
}

static int acquire_reg_info(struct platform_device *pdev,
			    struct sprd_regulator_desc *desc, int index)
{
	struct sprd_regulator_regs *regs = &desc->regs;
	int err = 0;
	const struct of_device_id *of_id;
	ldo_config *ldo_info;

	of_id = of_match_node(sprd_regulator_of_match,
		pdev->dev.of_node);
	if (!of_id) {
		dev_err(&pdev->dev,
			"Ddie LDO of device id failed!\n");
		return -ENODEV;
	}

	ldo_info = (ldo_config *)of_id->data;

	if (strcmp(desc->desc.name, ldo_info->regu_name) == 0) {
		/* ldo/dcdc power down */
		regs->pd_set = ldo_info->pd_reg;
		regs->pd_set_bit = ldo_info->pd_mask;

		err = 1;
	} else {
		debug("regulator %s does not match with %s !!!\n",
		      desc->desc.name, ldo_info->regu_name);
	}
	return err;
}

static int ldo_turn_on(struct regulator_dev *rdev)
{
	struct sprd_regulator_desc *desc = get_desc(rdev);
	struct sprd_regulator_regs *regs = &desc->regs;

	if (regs->pd_set)
		analog_clr_bits(desc->anlg_glb, regs->pd_set, regs->pd_set_bit);

	debug("regu 0x%p (%s), turn on\n", regs, desc->desc.name);
	return 0;
}

static int ldo_turn_off(struct regulator_dev *rdev)
{
	struct sprd_regulator_desc *desc = get_desc(rdev);
	struct sprd_regulator_regs *regs = &desc->regs;

	if (regs->pd_set)
		analog_set_bits(desc->anlg_glb, regs->pd_set, regs->pd_set_bit);

	debug("regu 0x%p (%s), turn off\n", regs, desc->desc.name);
	return 0;
}

static int ldo_is_on(struct regulator_dev *rdev)
{
	int ret = -EINVAL;
	struct sprd_regulator_desc *desc = get_desc(rdev);
	struct sprd_regulator_regs *regs = &desc->regs;

	if (regs->pd_set)
		ret = !(analog_reg_read(desc->anlg_glb, regs->pd_set)
			& regs->pd_set_bit);

	debug("regu 0x%p (%s) turn on, return %d\n", regs, desc->desc.name,
	      ret);
	return ret;
}

static struct regulator_ops ldo_ops = {
	.enable = ldo_turn_on,
	.disable = ldo_turn_off,
	.is_enabled = ldo_is_on,
};

static struct dentry *debugfs_root;

static int debugfs_enable_get(void *data, u64 *val)
{
	struct regulator_dev *rdev = data;

	if (rdev && rdev->desc->ops->is_enabled)
		*val = rdev->desc->ops->is_enabled(rdev);
	else
		*val = ~0ULL;
	return 0;
}

static int debugfs_enable_set(void *data, u64 val)
{
	struct regulator_dev *rdev = data;

	if (rdev && rdev->desc->ops->enable)
		(val) ? rdev->desc->ops->enable(rdev)
		    : rdev->desc->ops->disable(rdev);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_enable,
			debugfs_enable_get, debugfs_enable_set, "%llu\n");

static void rdev_init_debugfs(struct regulator_dev *rdev)
{
	struct sprd_regulator_desc *desc = get_desc(rdev);

	desc->debugfs = debugfs_create_dir(rdev->desc->name, debugfs_root);

	if (IS_ERR_OR_NULL(rdev->debugfs)) {
		pr_warn("Failed to create debugfs directory\n");
		rdev->debugfs = NULL;
		return;
	}

	debugfs_create_file("enable", S_IRUGO | S_IWUSR,
			    desc->debugfs, rdev, &fops_enable);
}

static struct regulator_consumer_supply *set_supply_map(struct device *dev,
							const char *supply_name,
							int *num)
{
	char **map = (char **)dev_get_platdata(dev);
	int i, n;
	struct regulator_consumer_supply *consumer_supplies = NULL;

	if (!supply_name || !(map && map[0]))
		return NULL;

	for (i = 0; map[i] || map[i + 1]; i++) {
		if (map[i] && 0 == strcmp(map[i], supply_name))
			break;
	}

	/*
	 * i++;
	 * Do not skip supply name
	 */

	for (n = 0 ; map[i + n] ; n++)
		;

	if (n) {
		debug("supply %s consumers %d - %d\n", supply_name, i, n);
		consumer_supplies =
		devm_kzalloc(dev, n*sizeof(*consumer_supplies), GFP_KERNEL);
		if (!consumer_supplies) {
			dev_err(dev,
				"failed allocate memory for consumer_supplies");
			return NULL;
		}

		for (n = 0; map[i]; i++, n++)
			consumer_supplies[n].supply = map[i];

		if (num)
			*num = n;
	}
	return consumer_supplies;
}

static int regulator_parse_dt(struct platform_device *pdev,
				  struct device_node *np,
				  struct sprd_regulator_desc *desc,
				  struct regulator_consumer_supply *supply,
				  int sz)
{
	int index = -1;
	u32 tmp_val_u32;
	int ret = 0;

	if (!pdev || !np || !desc || !supply)
		return -EINVAL;

	desc->init_data = of_get_regulator_init_data(&pdev->dev, np,
						     &desc->desc);

	if (!desc->init_data) {
		dev_err(&pdev->dev,
			"failed to parse regulator(%s) init data!\n",
			np->name);
		return -EINVAL;
	}

	desc->desc.name = desc->init_data->constraints.name;
	desc->desc.id = (atomic_inc_return(&idx));
	desc->desc.type = REGULATOR_VOLTAGE;
	desc->desc.owner = THIS_MODULE;
	desc->desc.continuous_voltage_range = true;

	supply[0].dev_name = NULL;
	supply[0].supply = desc->init_data->constraints.name;

	desc->init_data->supply_regulator =
		of_get_property(np, "parents-regulator", NULL);

	desc->init_data->constraints.valid_modes_mask =
	    REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY;
	desc->init_data->constraints.valid_ops_mask = REGULATOR_CHANGE_STATUS;
	desc->init_data->num_consumer_supplies = sz;

	desc->init_data->consumer_supplies =
	    set_supply_map(&pdev->dev, desc->desc.name,
			   &desc->init_data->num_consumer_supplies);

	if (!desc->init_data->consumer_supplies)
		desc->init_data->consumer_supplies = supply;

	ret = of_property_read_u32(np, "reg", &tmp_val_u32);
	if (!ret)
		index = tmp_val_u32;

	if (-1 != index) {
		if (!acquire_reg_info(pdev, desc, index))
			debug("acquire reg info:%d fail!!!\n", index);
	} else {
		debug("get index:%d fail!!!\n", index);
	}

	return ret;
}

static int regulator_register_dt(struct platform_device *pdev)
{
	struct sprd_regulator_desc *sprd_desc = NULL;
	struct regulator_dev *rdev;
	struct regulator_consumer_supply consumer_supplies_default[1];
	struct regulator_config config;
	struct device_node *dev_np;
	struct device_node *child_np;
	int regu_cnt = 0, ret = 0;

	struct device_node *np = pdev->dev.of_node;

	if (!np) {
		dev_err(&pdev->dev, "build in ldo node not found!!\n");
		return -ENODEV;
	}

	dev_np = of_find_node_by_name(pdev->dev.of_node, REGULAOR_DT_NODE);
	if (!dev_np)
		return -ENODEV;

	regu_cnt = of_get_child_count(dev_np);
	if (!regu_cnt)
		return -EINVAL;

	sprd_desc =
	    devm_kzalloc(&pdev->dev,
			 regu_cnt * sizeof(struct sprd_regulator_desc),
			 GFP_KERNEL);
	if (!sprd_desc) {
		dev_err(&pdev->dev,
			"failed allocate memory for sprd_regulator_desc list\n");
		return -ENOMEM;
	}

	sprd_desc->anlg_glb =
		syscon_regmap_lookup_by_phandle(np, "sprd,syscon-enable");
	if (IS_ERR(sprd_desc->anlg_glb)) {
		dev_err(&pdev->dev,
			"d-die ldo syscon failed!!\n");
		return -ENODEV;
	}

	for_each_available_child_of_node(dev_np, child_np) {
		ret = regulator_parse_dt(pdev, child_np, sprd_desc,
					   consumer_supplies_default,
					   ARRAY_SIZE
					   (consumer_supplies_default));
		if (ret) {
			dev_err(&pdev->dev,
				"failed to parse regulator(%s) dts\n",
				child_np->name);
			continue;
		}

		if (!sprd_desc->desc.ops)
			sprd_desc->desc.ops = &ldo_ops;

		config.dev = &pdev->dev;
		config.init_data = sprd_desc->init_data;
		config.driver_data = NULL;
		config.of_node = child_np;

		rdev = devm_regulator_register(&pdev->dev, &sprd_desc->desc,
					       &config);
		debug("regulator_desc 0x%p, rdev 0x%p\n", &sprd_desc->desc,
		      rdev);
		if (!IS_ERR_OR_NULL(rdev)) {
			rdev->reg_data = rdev;
			sprd_desc->data.rdev = rdev;
			if (rdev->desc->ops->is_enabled(rdev))
				rdev->use_count = 1;

			rdev_init_debugfs(rdev);
		} else {
			dev_err(&pdev->dev, "failed to register regulator %s\n",
				sprd_desc->desc.name);
		}

		sprd_desc++;
	}

	return 0;
}

static int sprd_regulator_probe(struct platform_device *pdev)
{
	pr_info("%s\n", __func__);
	debugfs_root = debugfs_create_dir(REGULATOR_ROOT_DIR, NULL);
	if (IS_ERR_OR_NULL(debugfs_root)) {
		WARN_ONCE(!debugfs_root,
		     "%s: Failed to create debugfs directory\n",
		     REGULATOR_ROOT_DIR);
		debugfs_root = NULL;
	}

	regulator_register_dt(pdev);

	pr_info("%s finish\n", __func__);

	return 0;
}

static int sprd_regulator_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver sprd_regulator_driver = {
	.driver = {
		   .name = "iwhale2-buildin-regulator",
		   .of_match_table = of_match_ptr(sprd_regulator_of_match),
		   },
	.probe = sprd_regulator_probe,
	.remove = sprd_regulator_remove
};

static int __init regu_driver_init(void)
{
	return platform_driver_register(&sprd_regulator_driver);
}

subsys_initcall(regu_driver_init);

MODULE_DESCRIPTION("Spreadtrum iwhale2 ldo driver");
MODULE_AUTHOR("Chen Junhui <erick.chen@spreadtrum.com>");
MODULE_LICENSE("GPL");

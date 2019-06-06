/*
 * Pin controller driver for Spreadtrum Soc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/slab.h>
#include "../core.h"
#include "pinctrl-sprd.h"

#define PINCTRL_BIT_MASK(width)	(~(~0UL << (width)))
#define SPRD_PIN_SIZE		8
#define PIN_USB_DUMMY		0xE4103074
#define PIN_SDIO_SL		0XE42A101C

/* function for pin mux selecting */
struct sprd_pmx_func {
	const char *name;
	unsigned int id;
};

/* pin configuration description */
struct sprd_pin_config {
	u32 driver_strength;
	u32 sleep_mode;
	u32 pull_up;
	u32 schmitt_trigger_input;
	u32 weak_pull_up_for_func;
	u32 weak_pull_down_for_func;
	struct sprd_pmx_func func_select;
	u32 weak_pull_up_for_sleep;
	u32 weak_pull_down_for_sleep;
	u32 input_for_sleep;
	u32 output_for_sleep;
};

/* pin description */
struct sprd_pin {
	const char *name;
	unsigned int number;
	unsigned int is_ctrl_pin;
	unsigned long reg;
	unsigned long bit_offset;
	unsigned long bit_width;
	unsigned long reg_config;
	struct sprd_pin_config config;
};

/* one pin group description for mux selecting */
struct sprd_pin_group {
	const char *name;
	unsigned int npins;
	unsigned int *pins;
	unsigned long *pins_config;
};

/* Soc pins description */
struct sprd_pinctrl_soc_info {
	struct sprd_pin_group *groups;
	unsigned int ngroups;
	unsigned int npins;
	struct sprd_pin *pins;
};

/* pin controller device description */
struct sprd_pinctrl {
	struct device *dev;
	struct pinctrl_dev *pctl;
	void __iomem *base;
	struct sprd_pinctrl_soc_info *info;
};

#ifdef CONFIG_DEBUG_FS
static int sprd_pinctrl_autotest_show(struct seq_file *s, void *what)
{
	/* TODO: */
	return 0;
}

static int sprd_pinctrl_autotest_open(struct inode *inode, struct file *file)
{
	return single_open(file, sprd_pinctrl_autotest_show, inode->i_private);
}

static ssize_t sprd_pinctrl_autotest_write(struct file *file,
					   const char __user *buf,
					   size_t count, loff_t *ppos)
{
	struct seq_file *m = file->private_data;
	struct sprd_pinctrl *pinctl = m->private;
	unsigned long offset = 0;
	unsigned long reg_value = 0;
	int ret = 0;

	ret = kstrtoul_from_user(buf, count, 16, &offset);
	if (ret < 0)
		return ret;

	reg_value = readl_relaxed((void __iomem *)((unsigned long)pinctl->base + offset));
	reg_value |= BIT(4) | BIT(5);
	writel_relaxed(reg_value, ((void __iomem *)((unsigned long)pinctl->base + offset)));

	return count;
}

/* pin file operation*/
static const struct file_operations sprd_pinctrl_autotest_ops = {
	.open		= sprd_pinctrl_autotest_open,
	.write		= sprd_pinctrl_autotest_write,
};
#endif

static unsigned int pinctrl_autotest;

static __init int sprd_pinctrl_autotest_mode(char *str)
{
	if (strcmp(str, "autotest"))
		pinctrl_autotest = 0;
	else
		pinctrl_autotest = 1;

	return 0;
}
__setup("androidboot.mode=", sprd_pinctrl_autotest_mode);

/* TODO: It will be used in future, so just leave it here */
#if 0
static int sprd_pinctrl_get_id_by_name(struct sprd_pinctrl *sprd_pctl,
				       char *name)
{
	struct sprd_pinctrl_soc_info *info = sprd_pctl->info;
	int i;

	for (i = 0; i < info->npins; i++) {
		if (!strcmp(info->pins[i].name, name))
			return info->pins[i].number;
	}

	return -ENODEV;
}
#endif

static  struct sprd_pin *
sprd_pinctrl_get_pin_by_id(struct sprd_pinctrl *sprd_pctl, unsigned int id)
{
	struct sprd_pinctrl_soc_info *info = sprd_pctl->info;
	struct sprd_pin *pin = NULL;
	int i;

	for (i = 0; i < info->npins; i++) {
		if (info->pins[i].number == id) {
			 pin = &info->pins[i];
			 break;
		}
	}

	return pin;
}

static const struct sprd_pin_group *
sprd_pinctrl_find_group_by_name(struct sprd_pinctrl *sprd_pctl,
				const char *name)
{
	struct sprd_pinctrl_soc_info *info = sprd_pctl->info;
	const struct sprd_pin_group *grp = NULL;
	int i;

	for (i = 0; i < info->ngroups; i++) {
		if (!strcmp(info->groups[i].name, name)) {
			grp = &info->groups[i];
			break;
		}
	}

	return grp;
}

static int sprd_pctrl_group_count(struct pinctrl_dev *pctldev)
{
	struct sprd_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	struct sprd_pinctrl_soc_info *info = pctl->info;

	return info->ngroups;
}

static const char *sprd_pctrl_group_name(struct pinctrl_dev *pctldev,
					 unsigned selector)
{
	struct sprd_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	struct sprd_pinctrl_soc_info *info = pctl->info;

	return info->groups[selector].name;
}

static int sprd_pctrl_group_pins(struct pinctrl_dev *pctldev, unsigned selector,
				 const unsigned **pins, unsigned *npins)
{
	struct sprd_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	struct sprd_pinctrl_soc_info *info = pctl->info;

	if (selector >= info->ngroups)
		return -EINVAL;

	*pins = info->groups[selector].pins;
	*npins = info->groups[selector].npins;

	return 0;
}

static int sprd_dt_node_to_map(struct pinctrl_dev *pctldev,
			struct device_node *np,
			struct pinctrl_map **map, unsigned *num_maps)
{
	struct sprd_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	const struct sprd_pin_group *grp;
	struct pinctrl_map *new_map;
	int map_num;

	grp = sprd_pinctrl_find_group_by_name(pctl, np->name);
	if (!grp) {
		pr_err("unable to find group for node %s\n", np->name);
		return -EINVAL;
	}

	/* TODO: Here we only map one to meet one pin configuration or
	 * one group configuration. In our board, the function setting
	 * is also can be considered as one pin configuration or multiple
	 * pins configuration. But it can be re-modified in future if
	 * there are other requirements.
	 */
	map_num = 1;
	new_map = kmalloc(sizeof(struct pinctrl_map) * map_num, GFP_KERNEL);
	if (!new_map)
		return -ENOMEM;

	*map = new_map;
	*num_maps = map_num;

	/* TODO: create mux map if there are requirements */

	/* create config map for one pin */
	if (grp->npins == 1) {
		unsigned int pin_id = grp->pins[0];

		new_map->type = PIN_MAP_TYPE_CONFIGS_PIN;
		new_map->data.configs.group_or_pin =
			pin_get_name(pctldev, pin_id);
		new_map->data.configs.configs = grp->pins_config;
		new_map->data.configs.num_configs = 1;
		goto out;
	}

	/* create config map for multiple pins */
	new_map->type = PIN_MAP_TYPE_CONFIGS_GROUP;
	new_map->data.configs.group_or_pin = grp->name;
	new_map->data.configs.num_configs = grp->npins;
	new_map->data.configs.configs = grp->pins_config;

out:
	return 0;
}

static void sprd_pctrl_dbg_show(struct pinctrl_dev *pctldev, struct seq_file *s,
				unsigned offset)
{
	seq_printf(s, "%s", dev_name(pctldev->dev));
}

static void sprd_dt_free_map(struct pinctrl_dev *pctldev,
			     struct pinctrl_map *map, unsigned num_maps)
{
	kfree(map);
}

static const struct pinctrl_ops sprd_pctrl_ops = {
	.get_groups_count = sprd_pctrl_group_count,
	.get_group_name = sprd_pctrl_group_name,
	.get_group_pins = sprd_pctrl_group_pins,
	.pin_dbg_show = sprd_pctrl_dbg_show,
	.dt_node_to_map = sprd_dt_node_to_map,
	.dt_free_map = sprd_dt_free_map,
};

static int sprd_pmx_func_count(struct pinctrl_dev *pctldev)
{
	/* TODO: */

	return 0;
}

static const char *sprd_pmx_func_name(struct pinctrl_dev *pctldev,
				      unsigned selector)
{
	/* TODO: */

	return NULL;
}

static int sprd_pmx_func_groups(struct pinctrl_dev *pctldev, unsigned selector,
				const char * const **groups,
				unsigned * const num_groups)
{
	/* TODO: */

	return 0;
}

static int sprd_pmx_set_mux(struct pinctrl_dev *pctldev, unsigned func_selector,
			    unsigned group_selector)
{
	struct sprd_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	struct sprd_pinctrl_soc_info *info = pctl->info;
	struct sprd_pin_group *grp = &info->groups[group_selector];
	unsigned int grp_pins = grp->npins;
	unsigned long reg;
	int i;

	if (group_selector > info->ngroups)
		return -EINVAL;

	for (i = 0; i < grp_pins; i++) {
		unsigned int pin_id = grp->pins[i];
		struct sprd_pin *pin = &info->pins[pin_id];

		if (pin->is_ctrl_pin) {
			reg = readl_relaxed((void __iomem *)pin->reg);
			reg |= (pin->reg_config &
				PINCTRL_BIT_MASK(pin->bit_width))
				<< pin->bit_offset;
			writel_relaxed(reg, (void __iomem *)pin->reg);
		} else {
			writel_relaxed(pin->reg_config, (void __iomem *)pin->reg);
		}
	}

	return 0;
}

static int sprd_gpio_req_enable(struct pinctrl_dev *pctldev,
				struct pinctrl_gpio_range *range,
				unsigned offset)
{
	/* TODO: */

	return 0;
}

void sprd_gpio_disable(struct pinctrl_dev *pctldev,
		       struct pinctrl_gpio_range *range,
		       unsigned offset)
{
	/* TODO: */
}

static int sprd_gpio_set_dir(struct pinctrl_dev *pctldev,
			     struct pinctrl_gpio_range *range,
			     unsigned offset, bool input)
{
	/* TODO: */

	return 0;
}

static const struct pinmux_ops sprd_pmx_ops = {
	.get_functions_count = sprd_pmx_func_count,
	.get_function_name = sprd_pmx_func_name,
	.get_function_groups = sprd_pmx_func_groups,
	.set_mux = sprd_pmx_set_mux,
	.gpio_request_enable = sprd_gpio_req_enable,
	.gpio_disable_free = sprd_gpio_disable,
	.gpio_set_direction = sprd_gpio_set_dir,
};

static int sprd_pinconf_get(struct pinctrl_dev *pctldev,
			    unsigned pin_id, unsigned long *config)
{
	struct sprd_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	struct sprd_pinctrl_soc_info *info = pctl->info;
	struct sprd_pin *pin = sprd_pinctrl_get_pin_by_id(pctl, pin_id);

	if (!pin || pin_id > info->npins)
		return -EINVAL;

	if (pin->is_ctrl_pin) {
		*config = (readl_relaxed((void __iomem *)pin->reg) >>
			   pin->bit_offset) & PINCTRL_BIT_MASK(pin->bit_width);
	} else {
		*config = readl_relaxed((void __iomem *)pin->reg);
	}

	return 0;
}

static int sprd_pinconf_set(struct pinctrl_dev *pctldev,
			    unsigned pin_id, unsigned long *configs,
			    unsigned num_configs)
{
	struct sprd_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	struct sprd_pinctrl_soc_info *info = pctl->info;
	struct sprd_pin *pin = sprd_pinctrl_get_pin_by_id(pctl, pin_id);
	unsigned long reg;
	int i;

	if (!pin || pin_id > info->npins)
		return -EINVAL;

	for (i = 0; i < num_configs; i++) {
		if (pin->is_ctrl_pin == 1) {
			reg = readl_relaxed((void __iomem *)pin->reg);
			reg &= ~(PINCTRL_BIT_MASK(pin->bit_width)
				<< pin->bit_offset);
			reg |= (configs[i] & PINCTRL_BIT_MASK(pin->bit_width))
				<< pin->bit_offset;
			writel_relaxed(reg, (void __iomem *)pin->reg);
		} else {
			writel_relaxed(configs[i], (void __iomem *)pin->reg);
		}
		pin->reg_config = configs[i];
	}

	return 0;
}

static int sprd_pinconf_group_get(struct pinctrl_dev *pctldev,
				  unsigned selector, unsigned long *config)
{
	struct sprd_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	struct sprd_pinctrl_soc_info *info = pctl->info;
	struct sprd_pin_group *grp;
	int ret, i;

	if (selector > info->ngroups)
		return -EINVAL;

	grp = &info->groups[selector];

	for (i = 0; i < grp->npins; i++, config++) {
		unsigned int pin_id = grp->pins[i];

		ret = sprd_pinconf_get(pctldev, pin_id, config);
		if (ret)
			return ret;
	}

	return 0;
}

static int sprd_pinconf_group_set(struct pinctrl_dev *pctldev,
				  unsigned selector, unsigned long *configs,
				  unsigned num_configs)
{
	struct sprd_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	struct sprd_pinctrl_soc_info *info = pctl->info;
	struct sprd_pin_group *grp;
	int ret, i;

	if (selector > info->ngroups)
		return -EINVAL;

	grp = &info->groups[selector];
	if (grp->npins != num_configs)
		return -EINVAL;

	for (i = 0; i < grp->npins; i++, configs++) {
		unsigned int pin_id = grp->pins[i];

		ret = sprd_pinconf_set(pctldev, pin_id, configs, 1);
		if (ret)
			return ret;
	}

	return 0;
}

static void sprd_pinconf_dbg_show(struct pinctrl_dev *pctldev,
				  struct seq_file *s, unsigned pin_id)
{
	unsigned long config;
	int ret;

	ret = sprd_pinconf_get(pctldev, pin_id, &config);
	if (ret)
		return;

	seq_printf(s, "0x%lx", config);
}

static void sprd_pinconf_group_dbg_show(struct pinctrl_dev *pctldev,
					struct seq_file *s, unsigned selector)
{
	struct sprd_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	struct sprd_pinctrl_soc_info *info = pctl->info;
	struct sprd_pin_group *grp;
	unsigned long config;
	const char *name;
	int i, ret;

	if (selector > info->ngroups)
		return;

	grp = &info->groups[selector];

	seq_printf(s, "\n");
	for (i = 0; i < grp->npins; i++, config++) {
		unsigned int pin_id = grp->pins[i];

		name = pin_get_name(pctldev, pin_id);
		ret = sprd_pinconf_get(pctldev, pin_id, &config);
		if (ret)
			return;

		seq_printf(s, "%s: 0x%lx", name, config);
	}
}

static const struct pinconf_ops sprd_pinconf_ops = {
	.pin_config_get = sprd_pinconf_get,
	.pin_config_set = sprd_pinconf_set,
	.pin_config_group_get = sprd_pinconf_group_get,
	.pin_config_group_set = sprd_pinconf_group_set,
	.pin_config_dbg_show = sprd_pinconf_dbg_show,
	.pin_config_group_dbg_show = sprd_pinconf_group_dbg_show,
};

static struct pinctrl_desc sprd_pinctrl_desc = {
	.pctlops = &sprd_pctrl_ops,
	.pmxops = &sprd_pmx_ops,
	.confops = &sprd_pinconf_ops,
	.owner = THIS_MODULE,
};

static int sprd_pinctrl_parse_groups(struct device_node *np,
				     struct sprd_pinctrl *sprd_pctl,
				     struct sprd_pin_group *grp)
{
	int i, size, pin_cnt, pin_id;
	const __be32 *list;

	list = of_get_property(np, "pins", &size);
	if (!size || !list) {
		dev_err(sprd_pctl->dev, "no pins property in node %s\n",
			np->full_name);
		return -EINVAL;
	}

	pin_cnt = size / SPRD_PIN_SIZE;
	grp->name = np->name;
	grp->npins = pin_cnt;
	grp->pins = devm_kzalloc(sprd_pctl->dev, grp->npins *
				 sizeof(unsigned int), GFP_KERNEL);
	if (!grp->pins)
		return -ENOMEM;

	grp->pins_config = devm_kzalloc(sprd_pctl->dev, grp->npins *
				 sizeof(unsigned long), GFP_KERNEL);
	if (!grp->pins_config)
		return -ENOMEM;

	for (i = 0; i < grp->npins; i++) {
		u32 pin_info = be32_to_cpu(*list++);

		pin_id = (pin_info >> NUM_OFFSET) & 0xfff;
		if (pin_id < 0)
			return -EINVAL;

		grp->pins[i] = pin_id;
		grp->pins_config[i] = be32_to_cpu(*list++);
	}

	for (i = 0; i < grp->npins; i++) {
		dev_dbg(sprd_pctl->dev, "Group[%s] contains [%d] pins: "
			   "pin id = %d, pin config = %ld\n",
			   grp->name, grp->npins, grp->pins[i],
			   grp->pins_config[i]);
	}

	return 0;
}

static int sprd_pinctrl_probe_dt(struct platform_device *pdev,
				 struct sprd_pinctrl *sprd_pctl)
{
	struct sprd_pinctrl_soc_info *info = sprd_pctl->info;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *child;
	struct sprd_pin *pin;
	struct sprd_pin_group *grp;
	unsigned int pin_cnt = 0, ctrl_pin = 0, com_pin = 0;
	unsigned int reg;
	int ret, i;

	if (!np)
		return -ENODEV;

	pin_cnt = sizeof(sprd_soc_pin_info) / sizeof(struct sprd_pin_data);
	if (!pin_cnt)
		return -EINVAL;

	info->npins = pin_cnt;
	info->pins = devm_kzalloc(&pdev->dev, pin_cnt * sizeof(struct sprd_pin),
				  GFP_KERNEL);
	if (!info->pins)
		return -ENOMEM;

	for (i = 0, pin = info->pins; i < pin_cnt; i++, pin++) {
		pin->name = kstrdup(sprd_soc_pin_info[i].name, GFP_KERNEL);
		pin->is_ctrl_pin = sprd_soc_pin_info[i].mode;
		pin->number = sprd_soc_pin_info[i].num;
		reg = sprd_soc_pin_info[i].reg;
		if (pin->is_ctrl_pin == 1) {
			pin->reg = (unsigned long)sprd_pctl->base + 0x4 * reg;
			pin->bit_offset = sprd_soc_pin_info[i].bit_offset;
			pin->bit_width = sprd_soc_pin_info[i].bit_width;
			/*
			 * The first is the hardware workaroud for iwhale2 chip,
			 * since if we want to set the PIN 63, acturally we
			 * should set the USB_DUMMY register.Second is the asic
			 * workround for isharkl2 chip,since it occurts 1.2V
			 * leakage when power off vddsdio and vddsdcore
			 * It is weried, but that happens.
			 */
			if (pin->number == 63 &&
			    of_device_is_compatible(pdev->dev.of_node,
						"sprd,sc9861-pinctrl")) {
				pin->reg = (unsigned long)ioremap_nocache(PIN_USB_DUMMY,
									 0x4);
				if (!pin->reg)
					return -ENOMEM;
				pin->bit_offset = 1;
				pin->bit_width = 1;
			} else if (pin->number == 63 &&
			    of_device_is_compatible(pdev->dev.of_node,
						"sprd,sc9853i-pinctrl")) {
				pin->reg = (unsigned long)ioremap_nocache(PIN_SDIO_SL,
									 0x4);
				if (!pin->reg)
					return -ENOMEM;
				pin->bit_offset = 3;
				pin->bit_width = 1;
			}
			ctrl_pin++;
		} else if (pin->is_ctrl_pin == 0) {
			pin->reg = (unsigned long)sprd_pctl->base +
				PINCTRL_REG_OFFSET + 0x4 * (i - ctrl_pin);
			com_pin++;
		} else if (pin->is_ctrl_pin == 2) {
			pin->reg = (unsigned long)sprd_pctl->base +
				PINCTRL_REG_MISC_OFFSET +
					0x4 * (i - ctrl_pin - com_pin);
		}
	}

	info->ngroups = of_get_child_count(np);
	if (!info->ngroups)
		return 0;

	info->groups = devm_kzalloc(&pdev->dev, info->ngroups *
				    sizeof(struct sprd_pin_group),
				    GFP_KERNEL);
	if (!info->groups)
		return -ENOMEM;

	grp = info->groups;
	for_each_child_of_node(np, child) {
		ret = sprd_pinctrl_parse_groups(child, sprd_pctl, grp);
		if (ret)
			return ret;
		grp++;
	}

	for (i = 0, pin = info->pins; i < info->npins; pin++, i++) {
		dev_dbg(&pdev->dev, "pin id[%d]: name: %s, is-ctrl-pin: %d, "
			   "bit_offset: %ld, bit_width: %ld, reg_config: %ld\n",
			   pin->number, pin->name, pin->is_ctrl_pin,
			   pin->bit_offset, pin->bit_width, pin->reg_config);
	}

	return 0;
}

static int sprd_pinctrl_probe(struct platform_device *pdev)
{
	struct sprd_pinctrl *sprd_pctl;
	struct sprd_pinctrl_soc_info *pinctrl_info;
	struct pinctrl_pin_desc *pin_desc;
	struct resource *res;
	int ret, i;

	sprd_pctl = devm_kzalloc(&pdev->dev, sizeof(struct sprd_pinctrl),
				 GFP_KERNEL);
	if (!sprd_pctl)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sprd_pctl->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(sprd_pctl->base))
		return PTR_ERR(sprd_pctl->base);

	pinctrl_info = devm_kzalloc(&pdev->dev,
				    sizeof(struct sprd_pinctrl_soc_info),
				    GFP_KERNEL);
	if (!pinctrl_info)
		return -ENOMEM;

	sprd_pctl->info = pinctrl_info;
	sprd_pctl->dev = &pdev->dev;
	platform_set_drvdata(pdev, sprd_pctl);

	ret = sprd_pinctrl_probe_dt(pdev, sprd_pctl);
	if (ret) {
		dev_err(&pdev->dev, "fail to probe dt properties\n");
		return ret;
	}

	pin_desc = devm_kzalloc(&pdev->dev, pinctrl_info->npins *
				sizeof(struct pinctrl_pin_desc),
				GFP_KERNEL);
	if (!pin_desc)
		return -ENOMEM;

	for (i = 0; i < pinctrl_info->npins; i++) {
		pin_desc[i].number = pinctrl_info->pins[i].number;
		pin_desc[i].name = pinctrl_info->pins[i].name;
		pin_desc[i].drv_data = pinctrl_info;
	}

	sprd_pinctrl_desc.pins = pin_desc;
	sprd_pinctrl_desc.name = dev_name(&pdev->dev);
	sprd_pinctrl_desc.npins = pinctrl_info->npins;

	sprd_pctl->pctl = pinctrl_register(&sprd_pinctrl_desc,
					   &pdev->dev, (void *)sprd_pctl);
	if (IS_ERR(sprd_pctl->pctl)) {
		dev_err(&pdev->dev, "could not register pinctrl driver\n");
		return PTR_ERR(sprd_pctl->pctl);
	}

#ifdef CONFIG_DEBUG_FS
	if (pinctrl_autotest) {
		dev_info(&pdev->dev, "pin controller in autotest mode.\n");
		debugfs_create_file("pins_debug", S_IRUGO | S_IWUSR,
				    sprd_pctl->pctl->device_root, sprd_pctl,
				    &sprd_pinctrl_autotest_ops);
	}
#endif

	dev_info(&pdev->dev, "initialized sprd pinctrl driver\n");
	return 0;
}

static int sprd_pinctrl_remove(struct platform_device *pdev)
{
	struct sprd_pinctrl *sprd_pctl = platform_get_drvdata(pdev);

	pinctrl_unregister(sprd_pctl->pctl);
	return 0;
}

void sprd_pinctrl_shutdown(struct platform_device *pdev)
{
	struct pinctrl *pinctl = devm_pinctrl_get(&pdev->dev);
	struct pinctrl_state *state;

	state = pinctrl_lookup_state(pinctl, "pins-shutdown");
	if (!IS_ERR(state))
		pinctrl_select_state(pinctl, state);
}

static const struct of_device_id sprd_pinctrl_of_match[] = {
	{
		.compatible = "sprd,sc9830-pinctrl",
	},
	{
		.compatible = "sprd,sc9850-pinctrl",
	},
	{
		.compatible = "sprd,sc9861-pinctrl",
	},
	{
		.compatible = "sprd,sc9833-pinctrl",
	},
	{
		.compatible = "sprd,sc9853i-pinctrl",
	},
	{
		.compatible = "sprd,sc9832e-pinctrl",
	},
	{
		.compatible = "sprd,sharkl3-pinctrl",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, sprd_pinctrl_of_match);

static struct platform_driver sprd_pinctrl_driver = {
	.driver = {
		.name = "pin-control",
		.owner = THIS_MODULE,
		.of_match_table = sprd_pinctrl_of_match,
	},
	.probe = sprd_pinctrl_probe,
	.remove = sprd_pinctrl_remove,
	.shutdown = sprd_pinctrl_shutdown,
};

static int sprd_pinctrl_init(void)
{
	return platform_driver_register(&sprd_pinctrl_driver);
}

static void sprd_pinctrl_exit(void)
{
	platform_driver_unregister(&sprd_pinctrl_driver);
}

module_init(sprd_pinctrl_init);
module_exit(sprd_pinctrl_exit);

MODULE_DESCRIPTION("SPREADTRUM Pin Controller Driver");
MODULE_AUTHOR("Baolin.Wang <Baolin.Wang@spreadtrum.com>");
MODULE_AUTHOR("Mingming.Ji <Mingming.Ji@spreadtrum.com>");
MODULE_LICENSE("GPL");

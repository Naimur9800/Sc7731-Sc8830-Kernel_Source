/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 * Author: Li Hao <ben.li@spreadtrum.com>
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/bug.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>


#include "gpio-common.h"

static struct sprd_gpio_chip pmic_data;

/* A-Die regs ops */
static int pmic_read_reg(unsigned long addr)
{
	unsigned int value;
	int ret;

	ret = regmap_read(pmic_data.parent_dev, addr, &value);
	if (ret) {
		pr_err("%s ret:%d\n", __func__, ret);
		return ret;
	}

	return value;
}

static int pmic_write_reg(uint32_t value, unsigned long addr)
{
	return regmap_write(pmic_data.parent_dev, addr, value);
}

static int pmic_set_bits(struct gpio_chip *chip,
		uint32_t bits, unsigned long addr)
{
	return regmap_update_bits(pmic_data.parent_dev, addr, bits, bits);
}

static int pmic_clr_bits(struct gpio_chip *chip,
		uint32_t bits, unsigned long addr)
{
	return regmap_update_bits(pmic_data.parent_dev, addr, bits, ~bits);
}

static struct sprd_gpio_chip pmic_gpio = {
	.chip.label = "sprd-pmic-gpio",
	.chip.request = sprd_gpio_request,
	.chip.free = sprd_gpio_free,
	.chip.direction_input = sprd_gpio_direction_input,
	.chip.get = sprd_gpio_get,
	.chip.direction_output = sprd_gpio_direction_output,
	.chip.set = sprd_gpio_set,
	.chip.set_debounce = sprd_gpio_set_debounce,
	.chip.to_irq = sprd_gpio_to_irq,

	.is_eic_dbnc = 0,
	.group_offset = ANA_GPIO_GROUP_OFFSET,
	.read_reg = pmic_read_reg,
	.write_reg = pmic_write_reg,
	.set_bits = pmic_set_bits,
	.clr_bits = pmic_clr_bits,
};

static struct sprd_gpio_chip pmic_eic = {
	.chip.label = "sprd-pmic-eic",
	.chip.request = sprd_gpio_request,
	.chip.free = sprd_gpio_free,
	.chip.direction_input = sprd_eic_direction_input,
	.chip.get = sprd_gpio_get,
	.chip.direction_output = NULL,
	.chip.set = sprd_eic_set,
	.chip.set_debounce = sprd_eic_set_debounce,
	.chip.to_irq = sprd_gpio_to_irq,

	.is_eic_dbnc = 1,
	.group_offset = ANA_GPIO_GROUP_OFFSET,
	.read_reg = pmic_read_reg,
	.write_reg = pmic_write_reg,
	.set_bits = pmic_set_bits,
	.clr_bits = pmic_clr_bits,
};

static inline int sprd_group_check(struct sprd_gpio_chip *sprd_gpio,
			int group)
{
	if ((group < 0) ||
			(group >= sprd_gpio->chip.ngpio / GPIO_GROUP_NR)) {
		pr_err("%s group:%d,ngpio:%d\n", __func__,
				group, sprd_gpio->chip.ngpio);
		return -EINVAL;
	}
	return 0;
}

static void sprd_pmic_irq_mask(struct irq_data *data)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	struct sprd_gpio_chip *sprd_gpio = to_sprd_gpio(chip);
	int offset = sprd_irq_to_gpio(chip, data->irq);
	int bitof = offset & (GPIO_GROUP_NR - 1);
	int group = offset / GPIO_GROUP_NR;

	if (sprd_group_check(sprd_gpio, group))
		return;

	pr_debug("%s %d+%d,bitof=%d,group=%d\n",
		__func__, chip->base, offset, bitof, group);
	sprd_gpio->reg[group][REG_GPIO_IE] &= ~BIT(bitof);
}

static void sprd_pmic_eic_irq_mask(struct irq_data *data)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	struct sprd_gpio_chip *sprd_gpio = to_sprd_gpio(chip);
	int offset = sprd_irq_to_gpio(chip, data->irq);
	int bitof = offset & (GPIO_GROUP_NR - 1);
	int group = offset / GPIO_GROUP_NR;

	if (sprd_group_check(sprd_gpio, group))
		return;

	pr_info("%s %d+%d,bitof=%d,group=%d\n",
		__func__, chip->base, offset, bitof, group);
	sprd_gpio->reg[group][REG_EIC_IE] &= ~BIT(bitof);
	sprd_gpio->reg[group][REG_EIC_TRIG] &= ~BIT(bitof);
}

static void sprd_pmic_gpio_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	struct sprd_gpio_chip *sprd_gpio = to_sprd_gpio(chip);
	int offset = sprd_irq_to_gpio(chip, data->irq);
	int bitof = offset & (GPIO_GROUP_NR - 1);
	int group = offset / GPIO_GROUP_NR;

	if (sprd_group_check(sprd_gpio, group))
		return;

	pr_debug("%s %d+%d,bitof=%d\n", __func__, chip->base, offset, bitof);
	sprd_gpio->reg[group][REG_GPIO_IE] |= BIT(bitof);
}

static int sprd_pmic_gpio_irq_set_type(struct irq_data *data,
				    unsigned int flow_type)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	struct sprd_gpio_chip *sprd_gpio = to_sprd_gpio(chip);
	int offset = sprd_irq_to_gpio(chip, data->irq);
	int bitof = offset & (GPIO_GROUP_NR - 1);
	int group = offset / GPIO_GROUP_NR;

	if (sprd_group_check(sprd_gpio, group))
		return -EINVAL;

	pr_debug("%s %d+%d bitof=%d type=%d\n", __func__,
		 chip->base, offset, bitof, flow_type);
	switch (flow_type) {
	case IRQ_TYPE_EDGE_RISING:
		sprd_gpio->reg[group][REG_GPIO_IS] &= ~BIT(bitof);
		sprd_gpio->reg[group][REG_GPIO_IBE] &= ~BIT(bitof);
		sprd_gpio->reg[group][REG_GPIO_IEV] |= BIT(bitof);
		irq_set_handler_locked(data, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		sprd_gpio->reg[group][REG_GPIO_IS] &= ~BIT(bitof);
		sprd_gpio->reg[group][REG_GPIO_IBE] &= ~BIT(bitof);
		sprd_gpio->reg[group][REG_GPIO_IEV] &= ~BIT(bitof);
		irq_set_handler_locked(data, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		sprd_gpio->reg[group][REG_GPIO_IS] &= ~BIT(bitof);
		sprd_gpio->reg[group][REG_GPIO_IBE] |= BIT(bitof);
		irq_set_handler_locked(data, handle_edge_irq);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		sprd_gpio->reg[group][REG_GPIO_IS] |= BIT(bitof);
		sprd_gpio->reg[group][REG_GPIO_IBE] &= ~BIT(bitof);
		sprd_gpio->reg[group][REG_GPIO_IEV] |= BIT(bitof);
		irq_set_handler_locked(data, handle_level_irq);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		sprd_gpio->reg[group][REG_GPIO_IS] |= BIT(bitof);
		sprd_gpio->reg[group][REG_GPIO_IBE] &= ~BIT(bitof);
		sprd_gpio->reg[group][REG_GPIO_IEV] &= ~BIT(bitof);
		irq_set_handler_locked(data, handle_level_irq);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void sprd_pmic_eic_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	struct sprd_gpio_chip *sprd_gpio = to_sprd_gpio(chip);
	int offset = sprd_irq_to_gpio(chip, data->irq);
	int bitof = offset & (GPIO_GROUP_NR - 1);
	int group = offset / GPIO_GROUP_NR;

	if (sprd_group_check(sprd_gpio, group))
		return;

	pr_info("%s %d+%d,bitof=%d\n", __func__, chip->base, offset, bitof);
	sprd_gpio->reg[group][REG_EIC_IE] |= BIT(bitof);

	/* TODO: the interval of two EIC trigger needs be longer than 2ms */
	sprd_gpio->reg[group][REG_EIC_TRIG] |= BIT(bitof);
}

static int sprd_pmic_eic_irq_set_type(struct irq_data *data,
				   unsigned int flow_type)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	struct sprd_gpio_chip *sprd_gpio = to_sprd_gpio(chip);
	int offset = sprd_irq_to_gpio(chip, data->irq);
	int bitof = offset & (GPIO_GROUP_NR - 1);
	int group = offset / GPIO_GROUP_NR;

	if (sprd_group_check(sprd_gpio, group))
		return -EINVAL;

	pr_debug("%s %d+%d %d\n", __func__, chip->base, offset, flow_type);
	switch (flow_type) {
	case IRQ_TYPE_LEVEL_HIGH:
		sprd_gpio->reg[group][REG_EIC_IEV] |= BIT(bitof);
		irq_set_handler_locked(data, handle_level_irq);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		sprd_gpio->reg[group][REG_EIC_IEV] &= ~BIT(bitof);
		irq_set_handler_locked(data, handle_level_irq);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

void sprd_gpio_irq_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct sprd_gpio_chip *sprd_gpio = to_sprd_gpio(gc);

	mutex_lock(&sprd_gpio->spi_lock);
}

void sprd_gpio_irq_bus_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct sprd_gpio_chip *sprd_gpio = to_sprd_gpio(gc);
	int offset = sprd_irq_to_gpio(gc, d->irq);
	int group = offset / GPIO_GROUP_NR;
	unsigned long addr = sprd_gpio->base_addr
	    + sprd_gpio->group_offset * group;

	if (sprd_group_check(sprd_gpio, group))
		return;
	pr_debug("base_addr=%lx,offset=%d,addr=%lx\n",
		sprd_gpio->base_addr, offset, addr);
	sprd_gpio_dump(sprd_gpio, group, addr, 1);

	/* irq unmask */
	sprd_gpio->write_reg(sprd_gpio->reg[group][REG_GPIO_IE],
			     addr + REG_GPIO_IE);
	/* irq type */
	sprd_gpio->write_reg(sprd_gpio->reg[group][REG_GPIO_IS],
			     addr + REG_GPIO_IS);
	sprd_gpio->write_reg(sprd_gpio->reg[group][REG_GPIO_IBE],
			     addr + REG_GPIO_IBE);
	sprd_gpio->write_reg(sprd_gpio->reg[group][REG_GPIO_IEV],
			     addr + REG_GPIO_IEV);

	mutex_unlock(&sprd_gpio->spi_lock);
}

void sprd_eic_irq_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct sprd_gpio_chip *sprd_gpio = to_sprd_gpio(gc);

	mutex_lock(&sprd_gpio->spi_lock);
}

void sprd_eic_irq_bus_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct sprd_gpio_chip *sprd_gpio = to_sprd_gpio(gc);
	int offset = sprd_irq_to_gpio(gc, d->irq);
	int group = offset / GPIO_GROUP_NR;
	unsigned long addr = sprd_gpio->base_addr
	    + sprd_gpio->group_offset * group;

	if (sprd_group_check(sprd_gpio, group))
		return;
	pr_debug("offset=%d addr=%lx\n", offset, addr);
	sprd_gpio_dump(sprd_gpio, group, addr, 0);

	/* irq type */
	sprd_gpio->write_reg(sprd_gpio->reg[group][REG_EIC_IEV],
			     addr + REG_EIC_IEV);
	/* irq unmask */
	sprd_gpio->write_reg(sprd_gpio->reg[group][REG_EIC_IE],
			     addr + REG_EIC_IE);
	sprd_gpio->write_reg(sprd_gpio->reg[group][REG_EIC_TRIG],
			     addr + REG_EIC_TRIG);

	mutex_unlock(&sprd_gpio->spi_lock);
}
static irqreturn_t pmic_irq_handler(int irq, void *data)
{
	struct sprd_gpio_chip *sprd_gpio = data;
	int group, n, value, virq, value_ie;
	unsigned long addr, addr_ic, addr_trig, addr_ie;
	struct gpio_chip *chip = &sprd_gpio->chip;

	pr_debug("%d+%d %d\n", chip->base, chip->ngpio, irq);
	for (group = 0; group * GPIO_GROUP_NR < chip->ngpio; group++) {
		addr =
		    sprd_gpio->base_addr + sprd_gpio->group_offset * group +
		    REG_EIC_MIS;
		addr_ic =
		    sprd_gpio->base_addr + sprd_gpio->group_offset * group +
		    REG_EIC_IC;
		addr_trig =
		    sprd_gpio->base_addr + sprd_gpio->group_offset * group +
		    REG_EIC_TRIG;
		addr_ie =
		    sprd_gpio->base_addr + sprd_gpio->group_offset * group +
		    REG_EIC_IE;

		value = sprd_gpio->read_reg(addr) & GPIO_GROUP_MASK;
		pr_debug("value=0x%x,addr=0x%lx\n", value, addr);

		while (value) {
			n = __ffs(value);
			value &= ~BIT(n);
			pr_info("n=%d,value=0x%x,addr_ic=0x%lx,val=0x%x\n", n,
			       value, addr_ic, sprd_gpio->read_reg(addr_ic));

			sprd_gpio->write_reg(BIT(n), addr_ic);
			virq = irq_find_mapping(sprd_gpio->irq_domain, n);
			pr_debug("generic_handle_n virq=%d\n", virq);
			handle_nested_irq(virq);
			value_ie =
				sprd_gpio->read_reg(addr_ie) & GPIO_GROUP_MASK;
			if (sprd_gpio->is_eic_dbnc
					&& (value_ie & BIT(n)))
				sprd_gpio->write_reg(BIT(n), addr_trig);
		}
	}

	return IRQ_HANDLED;
}

static struct irq_chip pmic_gpio_irq_chip = {
	.name = "irq-pmic-gpio",
	.irq_disable = sprd_pmic_irq_mask,
	.irq_mask = sprd_pmic_irq_mask,
	.irq_unmask = sprd_pmic_gpio_irq_unmask,
	.irq_set_type = sprd_pmic_gpio_irq_set_type,
	.irq_bus_lock = sprd_gpio_irq_bus_lock,
	.irq_bus_sync_unlock = sprd_gpio_irq_bus_unlock,
	.flags = IRQCHIP_SKIP_SET_WAKE,
};

static struct irq_chip pmic_eic_irq_chip = {
	.name = "irq-pmic-eic",
	.irq_disable = sprd_pmic_eic_irq_mask,
	.irq_mask = sprd_pmic_eic_irq_mask,
	.irq_unmask = sprd_pmic_eic_irq_unmask,
	.irq_set_type = sprd_pmic_eic_irq_set_type,
	.irq_bus_lock = sprd_eic_irq_bus_lock,
	.irq_bus_sync_unlock = sprd_eic_irq_bus_unlock,
	.flags = IRQCHIP_SKIP_SET_WAKE,
};

static struct sprd_gpio_match_data pmic_eic_match = {
	.sprd_gpio_chip = &pmic_eic,
	.request_irq_chip = &pmic_eic_irq_chip,
};

static struct sprd_gpio_match_data pmic_gpio_match = {
	.sprd_gpio_chip = &pmic_gpio,
	.request_irq_chip = &pmic_gpio_irq_chip,
};

static struct of_device_id pmic_gpio_match_table[] = {
	{.compatible = "sprd,sc2723t-eic", .data = &pmic_eic_match},
	{.compatible = "sprd,sc2723t-gpio", .data = &pmic_gpio_match},
	{.compatible = "sprd,sc2731-eic", .data = &pmic_eic_match},
	{.compatible = "sprd,sc2721-eic", .data = &pmic_eic_match},
	{.compatible = "sprd,sc2720-eic", .data = &pmic_eic_match},
	{},
};

static int irq_domain_pmic_map(struct irq_domain *h, unsigned int virq,
			       irq_hw_number_t hw)
{
	struct sprd_gpio_chip *sgc = h->host_data;

	irq_set_chip_data(virq, &sgc->chip);
	irq_set_chip(virq, &sgc->irq_chip);
	irq_set_nested_thread(virq, true);

	return 0;
}

static const struct irq_domain_ops irq_domain_pmic_ops = {
	.xlate = irq_domain_xlate_onetwocell,
	.map = irq_domain_pmic_map,
};
static int pmic_gpio_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct sprd_gpio_match_data *match_data;
	struct device_node *np = pdev->dev.of_node;
	struct sprd_gpio_chip *sgc;
	struct irq_chip *ic;
	struct irq_domain *irq_domain;
	int irq, err;
	u32 value;
	int i, group_num;

	if (!np && !pdev->dev.platform_data)
		return -ENODEV;

	match = of_match_device(pmic_gpio_match_table, &pdev->dev);
	match_data = (struct sprd_gpio_match_data *)(match->data);
	sgc = match_data->sprd_gpio_chip;
	ic = match_data->request_irq_chip;

	pmic_data.parent_dev = dev_get_regmap(pdev->dev.parent, NULL);
	if (!pmic_data.parent_dev) {
		dev_err(&pdev->dev, "pmic eic has no parent!\n");
		return -ENODEV;
	}

	if (of_property_read_u32(np, "reg", &value)) {
		dev_err(&pdev->dev, "no reg of property for pmic eic !");
		return -EINVAL;
	}

	sgc->base_addr = (unsigned long)value;
	if (!sgc->base_addr) {
		dev_err(&pdev->dev, "can not get base address!\n");
		return -EINVAL;
	}

	err = sprd_gpio_common_probe(pdev, sgc);
	if (err)
		return err;

	mutex_init(&sgc->spi_lock);
	group_num = sgc->chip.ngpio / GPIO_GROUP_NR;
	if (group_num > GPIO_GROUP_MAX) {
		dev_err(&pdev->dev, "exceed the max gpio number\n");
		return -EINVAL;
	}
	for (i = 0; i < group_num; i++) {
		sgc->reg[i] = devm_kzalloc(&pdev->dev,
			REG_RECORD_NUM * sizeof(uint32_t), GFP_KERNEL);
		if (!sgc->reg[i]) {
			dev_err(&pdev->dev, "can't alloc reg[%d] space\n", i);
			return -ENOMEM;
		}
	}
	memmove(&sgc->irq_chip, ic, sizeof(struct irq_chip));
	irq_domain = irq_domain_add_linear(np, sgc->chip.ngpio,
					   &irq_domain_pmic_ops, sgc);
	if (!irq_domain) {
		dev_err(&pdev->dev, "irq domain alloc failed!\n");
		err = -ENOMEM;
		goto fail;
	}
	sgc->irq_domain = irq_domain;
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no IRQ resource info !");
		err = -ENODEV;
		goto fail;
	}
	err = devm_request_threaded_irq(sgc->chip.dev, irq, NULL,
				pmic_irq_handler,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_NO_SUSPEND,
				dev_name(sgc->chip.dev), sgc);
	if (err != 0) {
		dev_err(sgc->chip.dev, "unable to request IRQ#%d: %d\n",
			irq, err);
		goto fail;
	}
	dev_info(&pdev->dev, "%s probe ok!group_num=%d\n",
			__func__, group_num);
	return 0;

fail:
	gpiochip_remove(&sgc->chip);
	return err;
}

static int pmic_gpio_remove(struct platform_device *pdev)
{
	return sprd_gpio_common_remove(pdev, pmic_gpio_match_table);
}

static struct platform_driver pmic_gpio_driver = {
	.driver.name = "pmic-gpio",
	.driver.owner = THIS_MODULE,
	.driver.of_match_table = of_match_ptr(pmic_gpio_match_table),
	.probe = pmic_gpio_probe,
	.remove = pmic_gpio_remove,
};

static int __init pmic_gpio_init(void)
{
	return platform_driver_register(&pmic_gpio_driver);
}
subsys_initcall(pmic_gpio_init);

static void __exit pmic_gpio_exit(void)
{
	return platform_driver_unregister(&pmic_gpio_driver);
}
module_exit(pmic_gpio_exit);

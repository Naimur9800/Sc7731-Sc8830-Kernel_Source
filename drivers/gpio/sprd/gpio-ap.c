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
#include <linux/spi/spi-sprd-adi.h>

#include "gpio-common.h"

/* D-Die regs ops */
static int sprd_read_reg(unsigned long addr)
{
	return readl_relaxed((void __iomem *)addr);
}

static int sprd_write_reg(uint32_t value, unsigned long addr)
{
	writel_relaxed(value, (void __iomem *)addr);
	return 0;
}

static int sprd_set_bits(struct gpio_chip *chip,
	uint32_t bits, unsigned long addr)
{
	unsigned long flags;
	struct sprd_gpio_chip *sprd_gpio = to_sprd_gpio(chip);

	spin_lock_irqsave(&sprd_gpio->gpio_lock, flags);
	writel_relaxed(readl_relaxed((void __iomem *)addr) | bits,
		(void __iomem *)addr);
	spin_unlock_irqrestore(&sprd_gpio->gpio_lock, flags);
	return 0;
}

static int sprd_clr_bits(struct gpio_chip *chip,
	uint32_t bits, unsigned long addr)
{
	unsigned long flags;
	struct sprd_gpio_chip *sprd_gpio = to_sprd_gpio(chip);

	spin_lock_irqsave(&sprd_gpio->gpio_lock, flags);
	writel_relaxed(readl_relaxed((void __iomem *)addr) & ~bits,
		(void __iomem *)addr);
	spin_unlock_irqrestore(&sprd_gpio->gpio_lock, flags);
	return 0;
}

static struct sprd_gpio_chip ap_gpio = {
	.chip.label = "sprd-ap-gpio",
	.chip.request = sprd_gpio_request,
	.chip.free = sprd_gpio_free,
	.chip.direction_input = sprd_gpio_direction_input,
	.chip.get = sprd_gpio_get,
	.chip.direction_output = sprd_gpio_direction_output,
	.chip.set = sprd_gpio_set,
	.chip.set_debounce = sprd_gpio_set_debounce,
	.chip.to_irq = sprd_gpio_to_irq,

	.group_offset = GPIO_GROUP_OFFSET,
	.type = SPRD_AP_GPIO,
	.read_reg = sprd_read_reg,
	.write_reg = sprd_write_reg,
	.set_bits = sprd_set_bits,
	.clr_bits = sprd_clr_bits,
};

static struct sprd_gpio_chip ap_eic = {
	.chip.label = "sprd-ap-eic",
	.chip.request = sprd_eic_request,
	.chip.free = sprd_eic_free,
	.chip.direction_input = sprd_eic_direction_input,
	.chip.get = sprd_eic_get,
	.chip.direction_output = NULL,
	.chip.set = sprd_eic_set,
	.chip.set_debounce = sprd_eic_set_debounce,
	.chip.to_irq = sprd_gpio_to_irq,

	.group_offset = GPIO_GROUP_OFFSET,
	.type = SPRD_AP_EIC,
	.read_reg = sprd_read_reg,
	.write_reg = sprd_write_reg,
	.set_bits = sprd_set_bits,
	.clr_bits = sprd_clr_bits,
};

static struct sprd_gpio_chip ap_eic_async = {
	.chip.label = "sprd-ap-eic-async",
	.chip.direction_input = sprd_eic_direction_input,
	.chip.get = sprd_eic_async_get,
	.chip.direction_output = NULL,
	.chip.set = NULL,
	.chip.to_irq = sprd_gpio_to_irq,

	.group_offset = 0,
	.type = SPRD_AP_EIC_ASYNC,
	.read_reg = sprd_read_reg,
	.write_reg = sprd_write_reg,
	.set_bits = sprd_set_bits,
	.clr_bits = sprd_clr_bits,
};

static struct sprd_gpio_chip gpio_plus = {
	.chip.label = "sprd-gpio-plus",
	.chip.request = sprd_gpio_plus_request,
	.chip.free = sprd_gpio_plus_free,
	.chip.direction_input = sprd_gpio_plus_direction_input,
	.chip.get = sprd_gpio_plus_get,
	.chip.direction_output = sprd_gpio_plus_direction_output,
	.chip.set = sprd_gpio_plus_set,
	.chip.set_debounce = sprd_gpio_plus_set_debounce,
	.chip.to_irq = sprd_gpio_to_irq,

	.group_offset = 0,
	.type = SPRD_GPIO_PLUS,
	.gpio_num_active = 0,
	.read_reg = sprd_read_reg,
	.write_reg = sprd_write_reg,
	.set_bits = sprd_set_bits,
	.clr_bits = sprd_clr_bits,
};

static void sprd_ap_irq_mask(struct irq_data *data)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	int offset = sprd_irq_to_gpio(chip, data->irq);

	pr_debug("%s %d+%d\n",	__func__, chip->base, offset);
	sprd_gpio_write(chip, offset, REG_GPIO_IE, 0);
}

static void sprd_ap_eic_irq_mask(struct irq_data *data)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	int offset = sprd_irq_to_gpio(chip, data->irq);

	pr_debug("%s %d+%d\n",	__func__, chip->base, offset);
	sprd_eic_write(chip, offset, REG_GPIO_IE, 0);
}

static void sprd_ap_eic_async_irq_mask(struct irq_data *data)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	int offset = sprd_irq_to_gpio(chip, data->irq);

	pr_debug("%s %d+%d\n", __func__, chip->base, offset);
	sprd_eic_write(chip, offset, REG_EIC_ASYNC_IE, 0);
}

static void sprd_gpio_plus_irq_mask(struct irq_data *data)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	struct device *dev = chip->dev;
	int offset = sprd_irq_to_gpio(chip, data->irq);
	u32 channel = gpio_to_channel(chip, offset);

	dev_dbg(dev, "%s %d %d %d\n", __func__, chip->base, offset, channel);
	sprd_gpio_plus_set_bit(chip, channel, INT_SYSIF0_EN, 0);
}

static void sprd_ap_irq_ack(struct irq_data *data)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	int offset = sprd_irq_to_gpio(chip, data->irq);

	pr_debug("%d+%d\n", chip->base, offset);
	sprd_gpio_write(chip, offset, REG_GPIO_IC, 1);
}

static void sprd_ap_eic_irq_ack(struct irq_data *data)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	int offset = sprd_irq_to_gpio(chip, data->irq);

	pr_debug("%d+%d\n", chip->base, offset);
	sprd_eic_write(chip, offset, REG_GPIO_IC, 1);
}

static void sprd_ap_eic_async_irq_ack(struct irq_data *data)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	int offset = sprd_irq_to_gpio(chip, data->irq);

	pr_debug("%s %d+%d\n", __func__, chip->base, offset);
	sprd_eic_write(chip, offset, REG_EIC_ASYNC_IC, 1);
}

static void sprd_gpio_plus_irq_ack(struct irq_data *data)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	struct device *dev = chip->dev;
	int offset = sprd_irq_to_gpio(chip, data->irq);
	u32 channel = gpio_to_channel(chip, offset);

	dev_dbg(dev, "%s %d %d %d\n", __func__, chip->base, offset, channel);
	sprd_gpio_plus_set_bit(chip, channel, INT_SYSIF0_CLR, 1);
}

static void sprd_ap_gpio_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	int offset = sprd_irq_to_gpio(chip, data->irq);

	pr_debug("%s %d+%d\n", __func__, chip->base, offset);
	sprd_gpio_write(chip, offset, REG_GPIO_IE, 1);
}

static void sprd_gpio_plus_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	struct device *dev = chip->dev;
	int offset = sprd_irq_to_gpio(chip, data->irq);
	u32 channel = gpio_to_channel(chip, offset);
	int mode, dbnc;

	dev_dbg(dev, "%s %d %d %d\n", __func__, chip->base, offset, channel);
	sprd_gpio_plus_set_bit(chip, channel, INT_SYSIF0_EN, 1);

	mode = sprd_gpio_plus_read(chip, channel, REG_INT_CRL, BIT_INT_MODE,
			INT_MODE_WIDTH);
	dbnc = sprd_gpio_plus_read(chip, channel, REG_INT_CRL, BIT_DBC_CYCLE,
			DBC_CYCLE_WIDTH);
	if (dbnc != 0) {
		sprd_gpio_plus_write(chip, channel, REG_INT_CRL,
				BIT_DBC_TRG, 1, 1);
	}
}

static int sprd_ap_gpio_irq_set_type(struct irq_data *data,
				    unsigned int flow_type)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	int offset = sprd_irq_to_gpio(chip, data->irq);

	pr_debug("%s %d+%d %d\n", __func__, chip->base,
			offset, flow_type);
	switch (flow_type) {
	case IRQ_TYPE_EDGE_RISING:
		sprd_gpio_write(chip, offset, REG_GPIO_IS, 0);
		sprd_gpio_write(chip, offset, REG_GPIO_IBE, 0);
		sprd_gpio_write(chip, offset, REG_GPIO_IEV, 1);
		irq_set_handler_locked(data, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		sprd_gpio_write(chip, offset, REG_GPIO_IS, 0);
		sprd_gpio_write(chip, offset, REG_GPIO_IBE, 0);
		sprd_gpio_write(chip, offset, REG_GPIO_IEV, 0);
		irq_set_handler_locked(data, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		sprd_gpio_write(chip, offset, REG_GPIO_IS, 0);
		sprd_gpio_write(chip, offset, REG_GPIO_IBE, 1);
		irq_set_handler_locked(data, handle_edge_irq);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		sprd_gpio_write(chip, offset, REG_GPIO_IS, 1);
		sprd_gpio_write(chip, offset, REG_GPIO_IBE, 0);
		sprd_gpio_write(chip, offset, REG_GPIO_IEV, 1);
		irq_set_handler_locked(data, handle_level_irq);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		sprd_gpio_write(chip, offset, REG_GPIO_IS, 1);
		sprd_gpio_write(chip, offset, REG_GPIO_IBE, 0);
		sprd_gpio_write(chip, offset, REG_GPIO_IEV, 0);
		irq_set_handler_locked(data, handle_level_irq);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sprd_gpio_plus_irq_set_type(struct irq_data *data,
				    unsigned int flow_type)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	struct device *dev = chip->dev;
	int offset = sprd_irq_to_gpio(chip, data->irq);
	u32 channel = gpio_to_channel(chip, offset);
	int mode, dbnc;

	dev_dbg(dev, "%s %d+%d %d %d\n", __func__, chip->base,
			offset, flow_type, channel);
	switch (flow_type) {
	case IRQ_TYPE_EDGE_RISING:
		sprd_gpio_plus_write(chip, channel, REG_INT_CRL,
			BIT_INT_MODE, INT_MODE_WIDTH, GPIO_INT_EDG_MODE);
		sprd_gpio_plus_write(chip, channel, REG_INT_CRL,
			BIT_INT_EDG_DET_MODE, INT_EDG_DET_MODE_WIDTH, 1);
		irq_set_handler_locked(data, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		sprd_gpio_plus_write(chip, channel, REG_INT_CRL,
			BIT_INT_MODE, INT_MODE_WIDTH, GPIO_INT_EDG_MODE);
		sprd_gpio_plus_write(chip, channel, REG_INT_CRL,
			BIT_INT_EDG_DET_MODE, INT_EDG_DET_MODE_WIDTH, 0);
		irq_set_handler_locked(data, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		sprd_gpio_plus_write(chip, channel, REG_INT_CRL,
			BIT_INT_MODE, INT_MODE_WIDTH, GPIO_INT_EDG_MODE);
		sprd_gpio_plus_write(chip, channel, REG_INT_CRL,
			BIT_INT_EDG_DET_MODE, INT_EDG_DET_MODE_WIDTH, 2);
		irq_set_handler_locked(data, handle_edge_irq);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		mode = sprd_gpio_plus_read(chip, channel, REG_INT_CRL,
			BIT_INT_MODE, INT_MODE_WIDTH);
		dbnc = sprd_gpio_plus_read(chip, channel, REG_INT_CRL,
			BIT_DBC_CYCLE, DBC_CYCLE_WIDTH);
		if ((mode == GPIO_INT_DBC_MODE) && (dbnc == 0)) {
			sprd_gpio_plus_write(chip, channel, REG_INT_CRL,
			BIT_INT_MODE, INT_MODE_WIDTH, GPIO_INT_LEVEL_MODE);
		}
		sprd_gpio_plus_write(chip, channel, REG_INT_CRL,
			BIT_INT_LEVEL, 1, 1);
		irq_set_handler_locked(data, handle_level_irq);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		mode = sprd_gpio_plus_read(chip, channel, REG_INT_CRL,
			BIT_INT_MODE, INT_MODE_WIDTH);
		dbnc = sprd_gpio_plus_read(chip, channel, REG_INT_CRL,
			BIT_DBC_CYCLE, DBC_CYCLE_WIDTH);
		if ((mode == GPIO_INT_DBC_MODE) && (dbnc == 0)) {
			sprd_gpio_plus_write(chip, channel, REG_INT_CRL,
			BIT_INT_MODE, INT_MODE_WIDTH, GPIO_INT_LEVEL_MODE);
		}
		sprd_gpio_plus_write(chip, channel, REG_INT_CRL,
			BIT_INT_LEVEL, 1, 0);
		irq_set_handler_locked(data, handle_level_irq);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void sprd_ap_eic_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	int offset = sprd_irq_to_gpio(chip, data->irq);

	pr_debug("%s %d+%d\n", __func__, chip->base, offset);
	sprd_eic_write(chip, offset, REG_EIC_IE, 1);

	/* TODO: the interval of two EIC trigger needs be longer than 2ms */
	sprd_eic_write(chip, offset, REG_EIC_TRIG, 1);
}

static void sprd_ap_eic_async_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	int offset = sprd_irq_to_gpio(chip, data->irq);

	pr_debug("%s %d+%d\n", __func__, chip->base, offset);
	sprd_eic_write(chip, offset, REG_EIC_ASYNC_IE, 1);
}

static int sprd_ap_eic_irq_set_type(struct irq_data *data,
				   unsigned int flow_type)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	int offset = sprd_irq_to_gpio(chip, data->irq);

	pr_debug("%s %d+%d %d\n", __func__, chip->base, offset, flow_type);
	switch (flow_type) {
	case IRQ_TYPE_LEVEL_HIGH:
		sprd_eic_write(chip, offset, REG_EIC_IEV, 1);
		irq_set_handler_locked(data, handle_level_irq);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		sprd_eic_write(chip, offset, REG_EIC_IEV, 0);
		irq_set_handler_locked(data, handle_level_irq);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sprd_ap_eic_async_irq_set_type(struct irq_data *data,
					unsigned int flow_type)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	int offset = sprd_irq_to_gpio(chip, data->irq);

	pr_debug("%s %d+%d %d\n", __func__, chip->base, offset, flow_type);
	switch (flow_type) {
	case IRQ_TYPE_LEVEL_HIGH:
		sprd_eic_write(chip, offset, REG_EIC_ASYNC_IEV, 1);
		sprd_eic_write(chip, offset, REG_EIC_ASYNC_INTBOTH, 0);
		sprd_eic_write(chip, offset, REG_EIC_ASYNC_INTPOL, 1);
		irq_set_handler_locked(data, handle_level_irq);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		sprd_eic_write(chip, offset, REG_EIC_ASYNC_IEV, 1);
		sprd_eic_write(chip, offset, REG_EIC_ASYNC_INTBOTH, 0);
		sprd_eic_write(chip, offset, REG_EIC_ASYNC_INTPOL, 0);
		irq_set_handler_locked(data, handle_level_irq);
		break;
	case IRQ_TYPE_EDGE_RISING:
		sprd_eic_write(chip, offset, REG_EIC_ASYNC_IEV, 0);
		sprd_eic_write(chip, offset, REG_EIC_ASYNC_INTBOTH, 0);
		sprd_eic_write(chip, offset, REG_EIC_ASYNC_INTPOL, 1);
		irq_set_handler_locked(data, handle_level_irq);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		sprd_eic_write(chip, offset, REG_EIC_ASYNC_IEV, 0);
		sprd_eic_write(chip, offset, REG_EIC_ASYNC_INTBOTH, 0);
		sprd_eic_write(chip, offset, REG_EIC_ASYNC_INTPOL, 0);
		irq_set_handler_locked(data, handle_level_irq);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		sprd_eic_write(chip, offset, REG_EIC_ASYNC_IEV, 0);
		sprd_eic_write(chip, offset, REG_EIC_ASYNC_INTBOTH, 1);
		irq_set_handler_locked(data, handle_level_irq);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void ap_irq_handler(struct irq_desc *desc)
{
	struct irq_chip *irq_chip = irq_desc_get_chip(desc);
	unsigned int irq = irq_desc_get_irq(desc);
	struct sprd_gpio_chip *sprd_gpio = irq_get_handler_data(irq);
	struct gpio_chip *chip = &sprd_gpio->chip;
	struct device *dev = chip->dev;
	int group, n, value, group_nr, ngpio, gpio, virq;

	dev_dbg(dev, "%d+%d %d\n", chip->base, chip->ngpio, irq);
	chained_irq_enter(irq_chip, desc);

	if (sprd_gpio->type == SPRD_GPIO_PLUS) {
		group_nr = GPIO_PLUS_GROUP_NR;
		ngpio = sprd_gpio->gpio_num_active;
	} else if (sprd_gpio->type == SPRD_AP_EIC) {
		group_nr = EIC_GROUP_NR;
		ngpio = chip->ngpio;
	} else {
		group_nr = GPIO_GROUP_NR;
		ngpio = chip->ngpio;
	}

	for (group = 0; group * group_nr < ngpio; group++) {
		value = sprd_gpio_get_int_status(sprd_gpio, group);
		if (value == GPIO_UNVALID_STS)
			break;
		while (value) {
			n = __ffs(value);
			value &= ~(1 << n);

			if (sprd_gpio->type == SPRD_GPIO_PLUS)
				gpio = channel_to_gpio(chip, n);
			else
				gpio = n + group * group_nr;
			dev_dbg(dev, "gpio num is %d\n", gpio);
			virq = irq_find_mapping(sprd_gpio->irq_domain, gpio);
			dev_dbg(dev, "generic_handle_n virq %d\n", virq);
			generic_handle_irq(virq);
		}

		if (sprd_gpio->type == SPRD_AP_EIC) {
			value = sprd_gpio_get_int_status(&ap_eic_async, group);
			if (value == GPIO_UNVALID_STS)
				break;
			while (value) {
				n = __ffs(value);
				value &= ~(1 << n);
				dev_dbg(dev, "gpio num is %d\n",
					n + group * group_nr);
				n = irq_find_mapping(ap_eic_async.irq_domain,
					n + group * group_nr);
				dev_dbg(dev, "generic_handle_n virq %d\n", n);
				generic_handle_irq(n);
			}
		}
	}
	chained_irq_exit(irq_chip, desc);
}

static struct irq_chip ap_gpio_irq_chip = {
	.name = "irq-ap-gpio",
	.irq_disable = sprd_ap_irq_mask,
	.irq_ack = sprd_ap_irq_ack,
	.irq_mask = sprd_ap_irq_mask,
	.irq_unmask = sprd_ap_gpio_irq_unmask,
	.irq_set_type = sprd_ap_gpio_irq_set_type,
	.flags = IRQCHIP_SKIP_SET_WAKE,
};

static struct irq_chip ap_eic_irq_chip = {
	.name = "irq-ap-eic",
	.irq_disable = sprd_ap_eic_irq_mask,
	.irq_ack = sprd_ap_eic_irq_ack,
	.irq_mask = sprd_ap_eic_irq_mask,
	.irq_unmask = sprd_ap_eic_irq_unmask,
	.irq_set_type = sprd_ap_eic_irq_set_type,
	.flags = IRQCHIP_SKIP_SET_WAKE,
};

static struct irq_chip ap_eic_async_irq_chip = {
	.name = "irq-ap-eic",
	.irq_disable = sprd_ap_eic_async_irq_mask,
	.irq_ack = sprd_ap_eic_async_irq_ack,
	.irq_mask = sprd_ap_eic_async_irq_mask,
	.irq_unmask = sprd_ap_eic_async_irq_unmask,
	.irq_set_type = sprd_ap_eic_async_irq_set_type,
	.flags = IRQCHIP_SKIP_SET_WAKE,
};

static struct irq_chip gpio_plus_irq_chip = {
	.name = "irq-ap-gpio",
	.irq_disable = sprd_gpio_plus_irq_mask,
	.irq_ack = sprd_gpio_plus_irq_ack,
	.irq_mask = sprd_gpio_plus_irq_mask,
	.irq_unmask = sprd_gpio_plus_irq_unmask,
	.irq_set_type = sprd_gpio_plus_irq_set_type,
	.flags = IRQCHIP_SKIP_SET_WAKE,
};

static struct sprd_gpio_match_data ap_eic_match = {
	.sprd_gpio_chip = &ap_eic,
	.request_irq_chip = &ap_eic_irq_chip,
};

static struct sprd_gpio_match_data ap_gpio_match = {
	.sprd_gpio_chip = &ap_gpio,
	.request_irq_chip = &ap_gpio_irq_chip,
};

static struct sprd_gpio_match_data ap_eic_async_match = {
	.sprd_gpio_chip = &ap_eic_async,
	.request_irq_chip = &ap_eic_async_irq_chip,
};

static struct sprd_gpio_match_data gpio_plus_match = {
	.sprd_gpio_chip = &gpio_plus,
	.request_irq_chip = &gpio_plus_irq_chip,
};

static struct of_device_id ap_gpio_match_table[] = {
	{ .compatible = "sprd,ap-eic", .data = &ap_eic_match },
	{ .compatible = "sprd,ap-gpio", .data = &ap_gpio_match },
	{ .compatible = "sprd,ap-eic-async", .data = &ap_eic_async_match },
	{ .compatible = "sprd,gpio-plus", .data = &gpio_plus_match },
	{},
};

static int irq_domain_ap_map(struct irq_domain *h, unsigned int virq,
			       irq_hw_number_t hw)
{
	struct sprd_gpio_chip *sgc = h->host_data;

	irq_set_chip_data(virq, &sgc->chip);
	irq_set_chip_and_handler(virq, &sgc->irq_chip, handle_level_irq);

	return 0;
}

static const struct irq_domain_ops irq_domain_ap_ops = {
	.xlate = irq_domain_xlate_onetwocell,
	.map = irq_domain_ap_map,
};

static ssize_t channel_num_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct sprd_gpio_chip *sgc = dev_get_drvdata(dev);

	if (!sgc)
		return -EINVAL;

	return sprintf(buf, "0x%x\n", sgc->gpio_num_active);
}

static DEVICE_ATTR_RO(channel_num);

static struct attribute *gpio_plus_attrs[] = {
	&dev_attr_channel_num.attr,
	NULL
};
ATTRIBUTE_GROUPS(gpio_plus);

static int ap_gpio_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct sprd_gpio_match_data *match_data;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct sprd_gpio_chip *sgc;
	struct irq_chip *ic;
	struct resource *res;
	struct irq_domain *irq_domain;
	int irq, err, i;
	int ret;

	if (!np && !dev->platform_data)
		return -ENODEV;

	match = of_match_device(ap_gpio_match_table, dev);
	match_data = (struct sprd_gpio_match_data *)(match->data);
	sgc = match_data->sprd_gpio_chip;
	ic = match_data->request_irq_chip;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "No reg of property specified\n");
		return -ENODEV;
	}

	sgc->base_addr = (unsigned long)ioremap_nocache(res->start,
					 resource_size(res));
	if (!sgc->base_addr) {
		dev_err(dev, "can not get base address!\n");
		return -EIO;
	}

	if ((sgc->type == SPRD_AP_EIC) || (sgc->type == SPRD_AP_EIC_ASYNC)) {
		for (i = 0;; i++) {
			sgc->eic_base_addr[i] = 0;
			res = platform_get_resource(pdev,
					IORESOURCE_MEM, i + 1);
			if (!res) {
				dev_err(dev, "EIC ext no reg property\n");
				break;
			}

			sgc->eic_base_addr[i] =
			(unsigned long)ioremap_nocache(res->start,
					resource_size(res));
			if (!sgc->eic_base_addr[i]) {
				dev_err(dev, "eic error can not get base address!\n");
				return -EIO;
			}
		}
		sgc->eic_ext_num = i;
	}

	err = sprd_gpio_common_probe(pdev, sgc);
	if (err)
		return err;

	spin_lock_init(&sgc->gpio_lock);
	memmove(&sgc->irq_chip, ic, sizeof(struct irq_chip));
	irq_domain = irq_domain_add_linear(np, sgc->chip.ngpio,
					   &irq_domain_ap_ops, sgc);
	if (!irq_domain) {
		dev_err(dev, "irq domain alloc failed!\n");
		err = -ENOMEM;
		goto fail;
	}
	sgc->irq_domain = irq_domain;
	if (sgc->type != SPRD_AP_EIC_ASYNC) {
		irq = platform_get_irq(pdev, 0);
		if (irq < 0) {
			dev_err(dev, "no IRQ resource info !");
			err = -ENODEV;
			goto fail;
		}

		irq_set_chained_handler(irq, ap_irq_handler);
		irq_set_handler_data(irq, sgc);
	}

	if (sgc->type == SPRD_GPIO_PLUS) {
		platform_set_drvdata(pdev, sgc);

		ret = sysfs_create_groups(&dev->kobj, gpio_plus_groups);
		if (ret)
			dev_warn(dev, "failed to create gpio attributes\n");
	}

	dev_info(dev, "%s probe ok!\n", __func__);
	return 0;

fail:
	gpiochip_remove(&sgc->chip);
	return err;
}

static int ap_gpio_remove(struct platform_device *pdev)
{
	return sprd_gpio_common_remove(pdev, ap_gpio_match_table);
}

static struct platform_driver ap_gpio_driver = {
	.driver.name = "ap-gpio",
	.driver.owner = THIS_MODULE,
	.driver.of_match_table = of_match_ptr(ap_gpio_match_table),
	.probe = ap_gpio_probe,
	.remove = ap_gpio_remove,
};

static int __init ap_gpio_init(void)
{
	return platform_driver_register(&ap_gpio_driver);
}
subsys_initcall(ap_gpio_init);

static void __exit ap_gpio_exit(void)
{
	return platform_driver_unregister(&ap_gpio_driver);
}
module_exit(ap_gpio_exit);

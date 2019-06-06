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

int sprd_gpio_read(struct gpio_chip *chip, uint32_t offset, uint32_t reg)
{
	struct sprd_gpio_chip *sprd_gpio = to_sprd_gpio(chip);
	int group = offset / GPIO_GROUP_NR;
	int bitof = offset & (GPIO_GROUP_NR - 1);
	unsigned long addr = sprd_gpio->base_addr
	    + sprd_gpio->group_offset * group + reg;
	int value = sprd_gpio->read_reg(addr) & GPIO_GROUP_MASK;

	return (value >> bitof) & 0x1;
}

int sprd_eic_read(struct gpio_chip *chip, uint32_t offset, uint32_t reg)
{
	struct sprd_gpio_chip *sprd_gpio = to_sprd_gpio(chip);
	int group = offset / EIC_GROUP_NR;
	int bitof = offset & (EIC_GROUP_NR - 1);
	unsigned long addr = 0;
	int value = 0;

	if (group == 0)
		addr = sprd_gpio->base_addr + reg;
	else if (group == 1)
		addr = sprd_gpio->eic_base_addr[0] + reg;
	else if (group == 2)
		addr = sprd_gpio->eic_base_addr[1] + reg;
	else
		addr = sprd_gpio->eic_base_addr[2] + reg;

	value = sprd_gpio->read_reg(addr) & EIC_GROUP_MASK;

	return (value >> bitof) & 0x1;
}

int sprd_gpio_plus_read(struct gpio_chip *chip, u32 offset,
	u32 reg, u32 bit, u32 width)
{
	struct sprd_gpio_chip *sprd_gpio = to_sprd_gpio(chip);
	unsigned long addr = sprd_gpio->base_addr + reg +
					offset * BYTES_PER_REG;
	int value = sprd_gpio->read_reg(addr);

	return GPIO_VALUE_BITS_WIDTH(value, bit, width);
}

int sprd_gpio_write(struct gpio_chip *chip, uint32_t offset,
			    uint32_t reg, int value)
{
	struct sprd_gpio_chip *sprd_gpio = to_sprd_gpio(chip);
	int group = offset / GPIO_GROUP_NR;
	int bitof = offset & (GPIO_GROUP_NR - 1);
	unsigned long addr = sprd_gpio->base_addr +
			sprd_gpio->group_offset * group + reg;
	int ret;

	if (value)
		ret = sprd_gpio->set_bits(chip, 1 << bitof, addr);
	else
		ret = sprd_gpio->clr_bits(chip, 1 << bitof, addr);

	return ret;
}

int sprd_eic_write(struct gpio_chip *chip, uint32_t offset,
			    uint32_t reg, int value)
{
	struct sprd_gpio_chip *sprd_gpio = to_sprd_gpio(chip);
	int group = offset / EIC_GROUP_NR;
	int bitof = offset & (EIC_GROUP_NR - 1);
	unsigned long addr = 0;
	int ret;

	if (group == 0)
		addr = sprd_gpio->base_addr + reg;
	else if (group == 1)
		addr = sprd_gpio->eic_base_addr[0] + reg;
	else if (group == 2)
		addr = sprd_gpio->eic_base_addr[1] + reg;
	else
		addr = sprd_gpio->eic_base_addr[2] + reg;

	if (value)
		ret = sprd_gpio->set_bits(chip, 1 << bitof, addr);
	else
		ret = sprd_gpio->clr_bits(chip, 1 << bitof, addr);

	return ret;
}

int sprd_gpio_plus_write(struct gpio_chip *chip, u32 offset,
	u32 reg, u32 bit, u32 width, int value)
{
	struct sprd_gpio_chip *sprd_gpio = to_sprd_gpio(chip);
	unsigned long addr = sprd_gpio->base_addr + reg
			+ offset * BYTES_PER_REG;
	int ret = sprd_gpio->read_reg(addr);

	ret &= ~GPIO_BITS_WIDTH(bit, width);
	ret |= GPIO_VALUE_BITS(value, bit, width);

	return sprd_gpio->write_reg(ret, addr);
}

int sprd_gpio_plus_set_bit(struct gpio_chip *chip, u32 channel,
			    u32 reg, int value)
{
	struct sprd_gpio_chip *sprd_gpio = to_sprd_gpio(chip);
	unsigned long addr = sprd_gpio->base_addr + reg;
	int ret;

	if (value)
		ret = sprd_gpio->set_bits(chip, 1 << channel, addr);
	else
		ret = sprd_gpio->clr_bits(chip, 1 << channel, addr);

	return ret;
}

/* GPIO/EIC libgpio interfaces */
int sprd_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	pr_debug("%d+%d\n", chip->base, offset);
	sprd_gpio_write(chip, offset, REG_GPIO_DMSK, 1);
	return 0;
}

int sprd_eic_request(struct gpio_chip *chip, unsigned offset)
{
	pr_debug("%d+%d\n", chip->base, offset);
	sprd_eic_write(chip, offset, REG_GPIO_DMSK, 1);
	return 0;
}

int sprd_gpio_plus_request(struct gpio_chip *chip, u32 offset)
{
	struct device *dev = chip->dev;

	dev_dbg(dev, "%d+%d\n", chip->base, offset);
	sprd_gpio_plus_write(chip, offset, REG_GPIO_CRL,
			BIT_GPIO_ENABLE, 1, 1);
	return 0;
}

void sprd_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	pr_debug("%d+%d\n", chip->base, offset);
	sprd_gpio_write(chip, offset, REG_GPIO_DMSK, 0);
}

void sprd_eic_free(struct gpio_chip *chip, unsigned offset)
{
	pr_debug("%d+%d\n", chip->base, offset);
	sprd_eic_write(chip, offset, REG_GPIO_DMSK, 0);
}

void sprd_gpio_plus_free(struct gpio_chip *chip, u32 offset)
{
	struct device *dev = chip->dev;

	dev_dbg(dev, "%d+%d\n", chip->base, offset);
	sprd_gpio_plus_write(chip, offset, REG_GPIO_CRL,
			BIT_GPIO_ENABLE, 1, 0);
}

int sprd_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	pr_debug("%d+%d\n", chip->base, offset);
	sprd_gpio_write(chip, offset, REG_GPIO_DIR, 0);
	sprd_gpio_write(chip, offset, REG_GPIO_INEN, 1);
	return 0;
}

int sprd_eic_direction_input(struct gpio_chip *chip, unsigned offset)
{
	/* do nothing */
	pr_debug("%d+%d\n", chip->base, offset);
	return 0;
}

int sprd_gpio_plus_direction_input(struct gpio_chip *chip, u32 offset)
{
	struct device *dev = chip->dev;

	dev_dbg(dev, "%d+%d\n", chip->base, offset);
	sprd_gpio_plus_write(chip, offset, REG_GPIO_CRL,
			BIT_GPIO_DIR, 1, 0);

	return 0;
}

int sprd_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
				      int value)
{
	pr_debug("%d+%d %d\n", chip->base, offset, value);
	sprd_gpio_write(chip, offset, REG_GPIO_DIR, 1);
	sprd_gpio_write(chip, offset, REG_GPIO_INEN, 0);
	sprd_gpio_write(chip, offset, REG_GPIO_DATA, value);
	return 0;
}

int sprd_gpio_plus_direction_output(struct gpio_chip *chip, u32 offset,
				      int value)
{
	struct device *dev = chip->dev;

	dev_dbg(dev, "%d+%d %d\n", chip->base, offset, value);
	sprd_gpio_plus_write(chip, offset, REG_GPIO_CRL,
			BIT_GPIO_DIR, 1, 1);
	sprd_gpio_plus_write(chip, offset, REG_GPIO_CRL,
			BIT_GPIO_DATA, 1, value);

	return 0;
}

int sprd_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	pr_debug("%d+%d\n", chip->base, offset);
	return sprd_gpio_read(chip, offset, REG_GPIO_DATA);
}

int sprd_eic_get(struct gpio_chip *chip, unsigned offset)
{
	pr_debug("%d+%d\n", chip->base, offset);
	return sprd_eic_read(chip, offset, REG_GPIO_DATA);
}

int sprd_eic_async_get(struct gpio_chip *chip, unsigned offset)
{
	pr_debug("%d+%d\n", chip->base, offset);
	return sprd_eic_read(chip, offset, REG_EIC_ASYNC_DATA);
}

int sprd_gpio_plus_get(struct gpio_chip *chip, u32 offset)
{
	struct device *dev = chip->dev;

	dev_dbg(dev, "%d+%d\n", chip->base, offset);
	return sprd_gpio_plus_read(chip, offset, REG_GPIO_CRL,
			BIT_GPIO_DATA, 1);
}

void sprd_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	pr_debug("%d+%d %d\n", chip->base, offset, value);
	sprd_gpio_write(chip, offset, REG_GPIO_DATA, value);
}

void sprd_gpio_plus_set(struct gpio_chip *chip, u32 offset, int value)
{
	struct device *dev = chip->dev;

	dev_dbg(dev, "%d+%d %d\n", chip->base, offset, value);
	sprd_gpio_plus_write(chip, offset, REG_GPIO_CRL,
		BIT_GPIO_DATA, 1, value);
}

int sprd_gpio_set_debounce(struct gpio_chip *chip, unsigned offset,
				  unsigned debounce)
{
	/* not supported */
	pr_err("%d+%d\n", chip->base, offset);
	return -EINVAL;
}

int sprd_gpio_get_int_status(struct sprd_gpio_chip *sprd_gpio, u32 group)
{
	unsigned long addr;
	int value = 0;
	struct gpio_chip *chip = &sprd_gpio->chip;

	switch (sprd_gpio->type) {
	case SPRD_AP_GPIO:
		addr = sprd_gpio->base_addr +
			sprd_gpio->group_offset * group + REG_GPIO_MIS;
		value = sprd_gpio->read_reg(addr) & GPIO_GROUP_MASK;
		break;

	case SPRD_AP_EIC:
		if (group == 0) {
			addr = sprd_gpio->base_addr + REG_GPIO_MIS;
		} else {
			if (sprd_gpio->eic_base_addr[group - 1] == 0) {
				value = GPIO_UNVALID_STS;
				break;
			}
			addr = sprd_gpio->eic_base_addr[group - 1] +
					REG_GPIO_MIS;
		}
		value = sprd_gpio->read_reg(addr) & GPIO_GROUP_MASK;
		break;

	case SPRD_AP_EIC_ASYNC:
		if (group == 0) {
			if (sprd_gpio->base_addr == 0) {
				value = GPIO_UNVALID_STS;
				break;
			}
			addr = sprd_gpio->base_addr +
					REG_EIC_ASYNC_MIS;
		} else {
			if (sprd_gpio->eic_base_addr[group - 1] == 0) {
				value = GPIO_UNVALID_STS;
				break;
			}
			addr = sprd_gpio->eic_base_addr[group - 1] +
					REG_EIC_ASYNC_MIS;
		}
		value = sprd_gpio->read_reg(addr) & GPIO_GROUP_MASK;
		break;

	case SPRD_GPIO_PLUS:
		value = sprd_gpio_plus_read(chip, 0, INT_SYSIF0_MSK,
				0, GPIO_PLUS_CHANNEL_NR);
		break;

	default:
		break;
	}

	return value;
}

u32 gpio_to_channel(struct gpio_chip *chip, u32 offset)
{
	int i, source;
	unsigned long flags;
	struct sprd_gpio_chip *sprd_gpio = to_sprd_gpio(chip);

	spin_lock_irqsave(&sprd_gpio->gpio_lock, flags);
	for (i = 0; i < sprd_gpio->gpio_num_active; i++) {
		source = sprd_gpio_plus_read(chip, i, REG_INT_SOURCE_SEL,
			BIT_INT_SOURCE_SEL, INT_SOURCE_SEL_WIDTH);
		if (source == offset)
			break;
	}

	if (i == sprd_gpio->gpio_num_active) {
		sprd_gpio_plus_write(chip, i, REG_INT_SOURCE_SEL,
			BIT_INT_SOURCE_SEL, INT_SOURCE_SEL_WIDTH, offset);
		sprd_gpio->gpio_num_active++;
	}
	spin_unlock_irqrestore(&sprd_gpio->gpio_lock, flags);

	return i;
}

u32 channel_to_gpio(struct gpio_chip *chip, u32 channel)
{
	return sprd_gpio_plus_read(chip, channel, REG_INT_SOURCE_SEL,
		BIT_INT_SOURCE_SEL, INT_SOURCE_SEL_WIDTH);
}

int sprd_gpio_plus_set_debounce(struct gpio_chip *chip, u32 offset,
				 u32 debounce)
{
	u32 value;
	u32 channel = gpio_to_channel(chip, offset);
	struct device *dev = chip->dev;

	dev_dbg(dev, "%d+%d %d\n", chip->base, offset, channel);

	value = debounce / 1000;
	sprd_gpio_plus_write(chip, channel, REG_INT_CRL,
			BIT_DBC_CYCLE, DBC_CYCLE_WIDTH, value);
	sprd_gpio_plus_write(chip, channel, REG_INT_CRL, BIT_INT_MODE,
			INT_MODE_WIDTH, GPIO_INT_DBC_MODE);

	return 0;
}

void sprd_eic_set(struct gpio_chip *chip, unsigned offset, int value)
{
	pr_debug("%d+%d %d\n", chip->base, offset, value);
}

int sprd_eic_set_debounce(struct gpio_chip *chip, unsigned offset,
				 unsigned debounce)
{
	struct sprd_gpio_chip *sprd_gpio = to_sprd_gpio(chip);
	int group = offset / GPIO_GROUP_NR;
	int bitof = offset & (GPIO_GROUP_NR - 1);
	unsigned long addr = sprd_gpio->base_addr
	    + sprd_gpio->group_offset * group + (bitof * 4 + REG_EIC_0CTRL);
	uint32_t value;

	pr_debug("%d+%d addr:%lx debounce:%d\n",
		chip->base, offset, addr, debounce / 1000);
	value = sprd_gpio->read_reg(addr);
	value &= (~0xfff);
	value |= debounce / 1000;
	return sprd_gpio->write_reg(value, addr);
}

int sprd_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct irq_domain *irq_domain;
	struct sprd_gpio_chip *sprd_gpio = to_sprd_gpio(chip);

	irq_domain = sprd_gpio->irq_domain;
	pr_debug("%s,%d+%d, irq_domain=%p,name=%s\n",
		__func__, chip->base, offset, irq_domain, irq_domain->name);
	return irq_create_mapping(irq_domain, offset);
}

int sprd_irq_to_gpio(struct gpio_chip *chip, unsigned irq)
{
	struct irq_data *irq_data = irq_get_irq_data(irq);

	return irq_data->hwirq;
}

void sprd_gpio_dump(struct sprd_gpio_chip *sprd_gpio,
			   int group, unsigned long addr, int is_gpio)
{
	if (is_gpio) {
		pr_debug("REG_GPIO_IC=0x%x\n",
			sprd_gpio->read_reg(addr + REG_GPIO_IC));
		pr_debug("REG_EIC_IE=0x%x\n",
			sprd_gpio->read_reg(addr + REG_GPIO_IE));
		pr_debug("REG_GPIO_IS=0x%x\n",
			sprd_gpio->read_reg(addr + REG_GPIO_IS));
		pr_debug("REG_GPIO_IBE=0x%x\n",
			sprd_gpio->read_reg(addr + REG_GPIO_IBE));
		pr_debug("REG_EIC_IEV=0x%x\n",
			sprd_gpio->read_reg(addr + REG_GPIO_IEV));
	} else {
		pr_debug("REG_GPIO_IC=0x%x,reg[stat]=0x%x\n",
			sprd_gpio->read_reg(addr + REG_EIC_IC),
			sprd_gpio->reg[group][REG_EIC_IC]);
		pr_debug("REG_EIC_IE=0x%x,reg[stat]=0x%x\n",
			sprd_gpio->read_reg(addr + REG_EIC_IE),
			sprd_gpio->reg[group][REG_EIC_IE]);
		pr_debug("REG_EIC_TRIG=0x%x,reg[stat]=0x%x\n",
			sprd_gpio->read_reg(addr + REG_EIC_TRIG),
			sprd_gpio->reg[group][REG_EIC_TRIG]);
		pr_debug("REG_EIC_IEV=0x%x,reg[stat]=0x%x\n",
			sprd_gpio->read_reg(addr + REG_EIC_IEV),
			sprd_gpio->reg[group][REG_EIC_IEV]);
	}
}

int sprd_gpio_common_probe(struct platform_device *pdev,
	struct sprd_gpio_chip *sgc)
{
	struct device_node *np = pdev->dev.of_node;

	if (of_property_read_u32(np, "sprd,ngpios",
				 (u32 *) (&sgc->chip.ngpio))) {
		dev_err(&pdev->dev, "No ngpios of property specified\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "sprd,gpiobase",
				 (u32 *) (&sgc->chip.base))) {
		dev_err(&pdev->dev, "No gpiobase of property specified\n");
		sgc->chip.base = -1;
		return -EINVAL;
	}

	dev_info(&pdev->dev, "base_addr %lx, gpio base %d, ngpio %d",
		 sgc->base_addr, sgc->chip.base, sgc->chip.ngpio);

	sgc->chip.of_node = np;
	sgc->chip.dev = &pdev->dev;

	gpiochip_add(&sgc->chip);

	return 0;
}

int sprd_gpio_common_remove(struct platform_device *pdev,
	struct of_device_id *match_table)
{
	int ret = 0;
	const struct of_device_id *match;
	struct sprd_gpio_match_data *match_data;
	struct sprd_gpio_chip *sgc;

	match = of_match_device(match_table, &pdev->dev);
	match_data = (struct sprd_gpio_match_data *)(match->data);
	sgc = match_data->sprd_gpio_chip;
	gpiochip_remove(&sgc->chip);

	return ret;
}

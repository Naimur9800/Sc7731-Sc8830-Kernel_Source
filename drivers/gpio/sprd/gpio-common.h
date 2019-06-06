/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __GPIO_SPRD_H__
#define __GPIO_SPRD_H__

/* 16 GPIO share a group of registers */
#define GPIO_GROUP_NR		(16)
#define GPIO_GROUP_MASK		(0xFFFF)

#define GPIO_GROUP_OFFSET	(0x80)
#define ANA_GPIO_GROUP_OFFSET	(0x40)

/* registers definitions for GPIO controller */
#define REG_GPIO_DATA		(0x0000)
#define REG_GPIO_DMSK		(0x0004)
#define REG_GPIO_DIR		(0x0008)	/* only for gpio */
#define REG_GPIO_IS		(0x000c)	/* only for gpio */
#define REG_GPIO_IBE		(0x0010)	/* only for gpio */
#define REG_GPIO_IEV		(0x0014)
#define REG_GPIO_IE		(0x0018)
#define REG_GPIO_RIS		(0x001c)
#define REG_GPIO_MIS		(0x0020)
#define REG_GPIO_IC		(0x0024)
#define REG_GPIO_INEN		(0x0028)	/* only for gpio */

/* 8 EIC share a group of registers */
#define EIC_GROUP_NR		(8)
#define EIC_GROUP_MASK		(0xFF)

/* registers definitions for EIC controller */
#define REG_EIC_DATA		REG_GPIO_DATA
#define REG_EIC_DMSK		REG_GPIO_DMSK
#define REG_EIC_IEV		REG_GPIO_IEV
#define REG_EIC_IE		REG_GPIO_IE
#define REG_EIC_RIS		REG_GPIO_RIS
#define REG_EIC_MIS		REG_GPIO_MIS
#define REG_EIC_IC		REG_GPIO_IC
#define REG_EIC_TRIG		(0x0028)	/* only for eic */
#define REG_EIC_0CTRL		(0x0040)
#define REG_EIC_1CTRL		(0x0044)
#define REG_EIC_2CTRL		(0x0048)
#define REG_EIC_3CTRL		(0x004c)
#define REG_EIC_4CTRL		(0x0050)
#define REG_EIC_5CTRL		(0x0054)
#define REG_EIC_6CTRL		(0x0058)
#define REG_EIC_7CTRL		(0x005c)
#define REG_EIC_DUMMYCTRL	(0x0000)

#define REG_EIC_ASYNC_IE		(0x0000)	/* only for eic async */
#define REG_EIC_ASYNC_RIS		(0x0004)
#define REG_EIC_ASYNC_MIS		(0x0008)
#define REG_EIC_ASYNC_IC		(0x000C)
#define REG_EIC_ASYNC_IEV		(0x0010)
#define REG_EIC_ASYNC_INTBOTH	(0x0014)
#define REG_EIC_ASYNC_INTPOL	(0x0018)
#define REG_EIC_ASYNC_DATA		(0x001C)

#define REG_RECORD_NUM	(REG_EIC_TRIG + 0x8)
/* bits definitions for register REG_EIC_DUMMYCTRL */
#define BIT_FORCE_CLK_DBNC		BIT(15)
#define BIT_EIC_DBNC_EN		BIT(14)
#define SHIFT_EIC_DBNC_CNT	(0)
#define MASK_EIC_DBNC_CNT	(0xFFF)
#define BITS_EIC_DBNC_CNT(_x_)	((_x) & 0xFFF)

#define	to_sprd_gpio(c)		container_of(c, struct sprd_gpio_chip, chip)
#define GPIO_GROUP_MAX		(1)

#define SPRD_AP_GPIO		(0)
#define SPRD_AP_EIC			(1)
#define SPRD_PMIC_GPIO		(2)
#define SPRD_PMIC_EIC		(3)
#define SPRD_AP_EIC_ASYNC	(4)
#define SPRD_GPIO_PLUS		5

/* registers definitions for GPIO plus controller */
/* GPIO CTL */
#define REG_GPIO_CRL					0x0
#define BIT_GPIO_DATA					0
#define BIT_GPIO_ODATA					1
#define BIT_GPIO_ENABLE					2
#define BIT_GPIO_DIR					3
/* INT CTL */
#define REG_INT_CRL					0x600
#define BIT_INT_LEVEL					0
#define BIT_SLEEP_INT_MODE				1

#define BIT_INT_EDG_DET_MODE				2
#define INT_EDG_DET_MODE_WIDTH				2

#define BIT_INT_MODE					4
#define INT_MODE_WIDTH					2

#define BIT_DBC_TRG					6

#define BIT_DBC_CYCLE					16
#define DBC_CYCLE_WIDTH					12

#define GPIO_INT_DBC_MODE				0x0
#define GPIO_INT_EDG_MODE				0x1
#define GPIO_INT_LATCH_MODE				0x2
#define GPIO_INT_LEVEL_MODE				0x3

/* INT SYS IF0 */
#define INT_SYSIF0_EN					0x700
#define INT_SYSIF0_RAW					0x800
#define INT_SYSIF0_MSK					0x900
#define INT_SYSIF0_CLR					0xA00

/* INT SOURCE SEL */
#define REG_INT_SOURCE_SEL				0xD00
#define BIT_INT_SOURCE_SEL				0
#define INT_SOURCE_SEL_WIDTH				8


#define BYTES_PER_REG					0x4
#define VALUE_SHIFT_LEFT(v, s)				((v) << (s))
#define VALUE_SHIFT_RIGHT(v, s)				((v) >> (s))
#define GPIO_CTL_MASK					GENMASK(4, 0)

#define GPIO_VALUE_BITS(v, b, w) \
	(VALUE_SHIFT_LEFT((v), (b)) & GENMASK(((b) + (w) - 1), (b)))

#define GPIO_BITS_WIDTH(b, w) \
	GENMASK(((b) + (w) - 1), (b))

#define GPIO_VALUE_BITS_WIDTH(v, b, w) \
	VALUE_SHIFT_RIGHT(((v) & GENMASK(((b) + (w) - 1), (b))), (b))

#define GPIO_PLUS_GROUP_NR				32
#define GPIO_PLUS_CHANNEL_NR				32
#define GPIO_UNVALID_STS				-1

struct sprd_gpio_chip {
	struct gpio_chip chip;
	struct irq_chip irq_chip;
	unsigned long base_addr;
	unsigned long eic_base_addr[3];
	int eic_ext_num;
	uint32_t type;
	uint32_t group_offset;
	struct irq_domain *irq_domain;
	struct regmap *parent_dev;

	/* pmic use irq_bus_lock scheme */
	uint32_t *reg[GPIO_GROUP_MAX];
	struct mutex spi_lock;
	spinlock_t gpio_lock;
	int is_eic_dbnc;
	int gpio_num_active;

	int (*read_reg)(unsigned long addr);
	int (*write_reg)(uint32_t value, unsigned long addr);
	int (*set_bits)(struct gpio_chip *chip, uint32_t bits,
			unsigned long addr);
	int (*clr_bits)(struct gpio_chip *chip, uint32_t bits,
			unsigned long addr);
};

struct sprd_gpio_match_data {
	struct sprd_gpio_chip *sprd_gpio_chip;
	struct irq_chip *request_irq_chip;
};

int sprd_gpio_read(struct gpio_chip *chip, uint32_t offset, uint32_t reg);
int sprd_gpio_write(struct gpio_chip *chip, uint32_t offset,
			    uint32_t reg, int value);
int sprd_gpio_request(struct gpio_chip *chip, unsigned offset);
int sprd_eic_read(struct gpio_chip *chip, uint32_t offset, uint32_t reg);
int sprd_eic_write(struct gpio_chip *chip, uint32_t offset,
			    uint32_t reg, int value);
int sprd_eic_request(struct gpio_chip *chip, unsigned offset);
void sprd_gpio_free(struct gpio_chip *chip, unsigned offset);
void sprd_eic_free(struct gpio_chip *chip, unsigned offset);
int sprd_gpio_direction_input(struct gpio_chip *chip, unsigned offset);
int sprd_eic_direction_input(struct gpio_chip *chip, unsigned offset);
int sprd_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
				      int value);
int sprd_gpio_get(struct gpio_chip *chip, unsigned offset);
int sprd_eic_get(struct gpio_chip *chip, unsigned offset);
int sprd_eic_async_get(struct gpio_chip *chip, unsigned offset);
void sprd_gpio_set(struct gpio_chip *chip, unsigned offset, int value);
int sprd_gpio_set_debounce(struct gpio_chip *chip, unsigned offset,
				  unsigned debounce);
void sprd_eic_set(struct gpio_chip *chip, unsigned offset, int value);
int sprd_eic_set_debounce(struct gpio_chip *chip, unsigned offset,
				 unsigned debounce);
int sprd_gpio_to_irq(struct gpio_chip *chip, unsigned offset);
int sprd_irq_to_gpio(struct gpio_chip *chip, unsigned irq);
void sprd_gpio_dump(struct sprd_gpio_chip *sprd_gpio,
			   int group, unsigned long addr, int is_gpio);
int sprd_gpio_common_probe(struct platform_device *pdev,
	struct sprd_gpio_chip *sgc);
int sprd_gpio_common_remove(struct platform_device *pdev,
	struct of_device_id *match_table);
int sprd_gpio_plus_read(struct gpio_chip *chip, u32 offset,
	u32 reg, u32 bit, u32 width);
int sprd_gpio_plus_write(struct gpio_chip *chip, u32 offset,
	u32 reg, u32 bit, u32 width, int value);
int sprd_gpio_plus_set_bit(struct gpio_chip *chip, u32 channel,
			    u32 reg, int value);
int sprd_gpio_plus_request(struct gpio_chip *chip, u32 offset);
void sprd_gpio_plus_free(struct gpio_chip *chip, u32 offset);
int sprd_gpio_plus_direction_input(struct gpio_chip *chip, u32 offset);
int sprd_gpio_plus_direction_output(struct gpio_chip *chip, u32 offset,
				      int value);
int sprd_gpio_plus_get(struct gpio_chip *chip, u32 offset);
void sprd_gpio_plus_set(struct gpio_chip *chip, u32 offset, int value);
int sprd_gpio_get_int_status(struct sprd_gpio_chip *sprd_gpio, u32 group);
u32 gpio_to_channel(struct gpio_chip *chip, u32 offset);
u32 channel_to_gpio(struct gpio_chip *chip, u32 channel);
int sprd_gpio_plus_set_debounce(struct gpio_chip *chip, u32 offset,
				 u32 debounce);

#endif

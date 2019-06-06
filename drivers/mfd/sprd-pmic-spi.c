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

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/of_platform.h>

#define TRANS_BUF_SIZE			(10)

#define SPRD_PMIC_INT_MASK_STATUS	(0x0000)
#define SPRD_PMIC_INT_RAW_STATU		(0x0004)
#define SPRD_PMIC_INT_EN		(0x0008)

/* ADI write type */
enum ADI_WRITE_MODE {
	SET_MODE = 0,
	FAST_SET_MODE,
};

struct sprd_pmic {
	struct regmap *regmap;
	struct device *dev;
	int irq;
	int irqmax;
	int regbase;
	struct regmap_irq *irqs;
	struct regmap_irq_chip irq_chip;
	struct regmap_irq_chip_data *irq_data;
};

static const struct regmap_config sprd_pmic_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0xffff,
};

static const struct of_device_id sprd_pmic_of_match[] = {
	{.compatible = "sprd,sc2723t",},
	{.compatible = "sprd,sc2723",},
	{.compatible = "sprd,sc2731",},
	{.compatible = "sprd,sc2721",},
	{.compatible = "sprd,sc2720",},
	{},
};

MODULE_DEVICE_TABLE(of, sprd_pmic_of_match);

static int regmap_adi_spi_write(void *context, const void *data, size_t count)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	unsigned long tx_buf[TRANS_BUF_SIZE] = { 0 };
	unsigned int *buf = (unsigned int *)data;
	int ret;

	/* The ADI driver only need low 12 bits address, So it makes sure the
	 * PMIC address is aglined with low 12 bits.
	 */
	tx_buf[0] = buf[0] & 0xfff;
	tx_buf[1] = FAST_SET_MODE;
	tx_buf[2] = buf[1];

	ret = spi_write(spi, tx_buf, 3);
	if (ret < 0)
		pr_err("ADI fast write:spi_async --> %d\n", ret);

	return ret;
}

static int regmap_adi_spi_gather_write(void *context,
				       const void *reg, size_t reg_len,
				       const void *val, size_t val_len)
{
	/* TODO: */

	return 0;
}

static int regmap_adi_spi_read(void *context,
			       const void *reg, size_t reg_size,
			       void *val, size_t val_size)
{
	unsigned long rx_buf[TRANS_BUF_SIZE] = { 0 };
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	int ret;

	/* The ADI driver only need low 12 bits address, So it makes sure the
	 * PMIC address is aglined with low 12 bits.
	 */
	rx_buf[0] = *(unsigned int *)reg & 0xfff;

	ret = spi_read(spi, rx_buf, 1);
	if (ret < 0) {
		pr_err("ADI read: spi_sync --> %d\n", ret);
		return ret;
	}

	memcpy(val, rx_buf, val_size);
	return ret;
}

static struct regmap_bus regmap_adi_spi = {
	.write = regmap_adi_spi_write,
	.gather_write = regmap_adi_spi_gather_write,
	.read = regmap_adi_spi_read,
	.reg_format_endian_default = REGMAP_ENDIAN_NATIVE,
	.val_format_endian_default = REGMAP_ENDIAN_NATIVE,
};

static int sprd_pmic_probe(struct spi_device *pmic_dev)
{
	struct device_node *root = pmic_dev->dev.of_node;
	struct regmap *regmap;
	int i;
	int ret;
	struct sprd_pmic *sprd_pmic;
	int regbase;
	int irqmax;

	regmap = devm_regmap_init(&pmic_dev->dev, &regmap_adi_spi,
				  &pmic_dev->dev, &sprd_pmic_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	sprd_pmic =
	    devm_kzalloc(&pmic_dev->dev, sizeof(*sprd_pmic), GFP_KERNEL);
	if (!sprd_pmic)
		return -ENOMEM;

	sprd_pmic->irq = pmic_dev->irq;
	sprd_pmic->regmap = regmap;

	ret =
	    of_property_read_u32_index(root, "sprd,pmic_intc_base", 0,
				       &regbase);
	if (ret)
		return ret;

	ret =
	    of_property_read_u32_index(root, "sprd,pmic_intc_irqmax", 0,
				       &irqmax);
	if (ret)
		return ret;

	sprd_pmic->irq_chip.name = dev_name(&pmic_dev->dev);
	sprd_pmic->irq_chip.status_base = regbase + SPRD_PMIC_INT_MASK_STATUS;
	sprd_pmic->irq_chip.mask_base = regbase + SPRD_PMIC_INT_EN;
	sprd_pmic->irq_chip.ack_base = 0;
	sprd_pmic->irq_chip.num_regs = 1;
	sprd_pmic->irq_chip.num_irqs = irqmax;
	sprd_pmic->irq_chip.mask_invert = true;

	sprd_pmic->irqs =
	    devm_kzalloc(&pmic_dev->dev, sizeof(struct regmap_irq) * irqmax,
			 GFP_KERNEL);
	if (!sprd_pmic->irqs)
		return -ENOMEM;

	sprd_pmic->irq_chip.irqs = sprd_pmic->irqs;
	for (i = 0; i < irqmax; i++) {
		sprd_pmic->irqs[i].reg_offset = i / irqmax;
		sprd_pmic->irqs[i].mask = BIT(i % irqmax);
	}

	dev_set_drvdata(&pmic_dev->dev, sprd_pmic);

	ret =
	    regmap_add_irq_chip(sprd_pmic->regmap, sprd_pmic->irq,
				IRQF_ONESHOT | IRQF_NO_SUSPEND, 0,
				&sprd_pmic->irq_chip, &sprd_pmic->irq_data);
	if (ret)
		return ret;

	ret = of_platform_populate(root, NULL, NULL, &pmic_dev->dev);

	dev_info(&pmic_dev->dev, "PMIC spi probe OK!\n");
	return ret;
}

static int sprd_pmic_remove(struct spi_device *pmic_dev)
{
	of_platform_depopulate(&pmic_dev->dev);
	return 0;
}

static struct spi_driver sprd_pmic_driver = {
	.driver = {
		   .name = "pmic",
		   .bus = &spi_bus_type,
		   .owner = THIS_MODULE,
		   .of_match_table = sprd_pmic_of_match,
		   },
	.probe = sprd_pmic_probe,
	.remove = sprd_pmic_remove,
};

static int __init sprd_pmic_init(void)
{
	return spi_register_driver(&sprd_pmic_driver);
}

subsys_initcall_sync(sprd_pmic_init);

static void __exit sprd_pmic_exit(void)
{
	spi_unregister_driver(&sprd_pmic_driver);
}

module_exit(sprd_pmic_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Spreadtrum SPI PMIC Driver");
MODULE_AUTHOR("Baolin Wang <baolin.wang@spreadtrum.com>");

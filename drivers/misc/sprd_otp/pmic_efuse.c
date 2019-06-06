/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/hwspinlock.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mfd/sprd/pmic_glb_reg.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/stat.h>
#include <linux/spi/spi-sprd-adi.h>
#include <linux/sprd_otp.h>

#define ANA_REG_EFUSE_GLB_CTRL          0x0000
#define ANA_REG_EFUSE_DATA_RD           0x0004
#define ANA_REG_EFUSE_DATA_WR           0x0008
#define ANA_REG_EFUSE_BLOCK_INDEX       0x000c
#define ANA_REG_EFUSE_MODE_CTRL         0x0010
#define ANA_REG_EFUSE_STATUS            0x0014
#define ANA_REG_EFUSE_WR_TIMING_CTRL    0x0028
#define ANA_REG_EFUSE_RD_TIMING_CTRL    0x002c
#define ANA_REG_EFUSE_EFUSE_DEB_CTRL    0x0030

/* bits definitions for register ANA_REG_EFUSE_GLB_CTRL */
#define BIT_EFUSE_PGM_EN	((0))

/* bits definitions for register ANA_REG_EFUSE_DATA_RD */
#define BITS_EFUSE_DATA_RD(_x_)	((_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)\
	|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)))

/* bits definitions for register ANA_REG_EFUSE_DATA_WR */
#define BITS_EFUSE_DATA_WR(_x_)	((_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)\
	|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)))

/* bits definitions for register ANA_REG_EFUSE_BLOCK_INDEX */
#define BITS_READ_WRITE_INDEX(_x_)	((_x_) << 0 & (BIT(0)|BIT(1)\
	|BIT(2)|BIT(3)|BIT(4)))
#define SHFT_READ_WRITE_INDEX	(0)
#define MASK_READ_WRITE_INDEX	(BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4))

/* bits definitions for register ANA_REG_EFUSE_MODE_CTRL */
/* Write 1 to this bit start A_PGM mode(array PGM mode).
 * This bit is self-clear, read this bit will always get 0.
 */
#define BIT_PG_START                    (BIT(0))
#define BIT_RD_START                    (BIT(1))
#define BIT_STANDBY_START               (BIT(2))
#define BIT_NORMAL_RD_FLAG_CLR		(BIT(2))

/* bits definitions for register ANA_REG_EFUSE_STATUS */
#define BIT_PGM_BUSY                    (BIT(0))
#define BIT_READ_BUSY                   (BIT(1))
#define BIT_STANDBY_BUSY                (BIT(2))
#define BIT_GLOBAL_PROT			(BIT(3))
#define BIT_NORMAL_RD_DONE		(BIT(4))

#define EFUSE_MAGIC_NUMBER              (0x2723)
#define HWSPINLOCK_TIMEOUT		(5000)

struct sprd_pmic_efuse {
	enum sprd_pmic_efuse_type type;
	struct regmap *regmap;
	struct hwspinlock *hw_lock;
	struct device *dev;
	unsigned long base;
	unsigned int efuse_block_max;
	unsigned int efuse_block_width;
};

struct sprd_pmic_efuse *pmic_efuse;

static const struct of_device_id pmic_efuse_of_match[] = {
	{.compatible = "sprd,sc2723-efuse", .data = (void *)SC2723_EFUSE,},
	{.compatible = "sprd,sc2731-efuse", .data = (void *)SC2731_EFUSE,},
	{.compatible = "sprd,sc2721-efuse", .data = (void *)SC2721_EFUSE,},
	{.compatible = "sprd,sc2720-efuse", .data = (void *)SC2720_EFUSE,},
	{}
};

static DEFINE_MUTEX(pmic_efuse_mutex);

static inline unsigned int pmic_efuse_reg_read(unsigned long reg)
{
	unsigned int val;

	regmap_read(pmic_efuse->regmap, pmic_efuse->base + reg, &val);
	return val;
}

static inline int pmic_efuse_reg_write(unsigned long reg, unsigned int val)
{
	return regmap_write(pmic_efuse->regmap, pmic_efuse->base + reg, val);
}

static inline int pmic_efuse_reg_set(unsigned long reg, unsigned int mask,
				     unsigned int bits)
{
	return regmap_update_bits(pmic_efuse->regmap, pmic_efuse->base + reg,
				  mask, bits);
}

static inline int sprd_efuse_glb_set(unsigned int reg, unsigned int mask,
				     unsigned int bits)
{
	return regmap_update_bits(pmic_efuse->regmap, reg, mask, bits);
}

static inline int sprd_efuse_glb_clr(unsigned int reg, unsigned int mask,
				     unsigned int bits)
{
	return regmap_update_bits(pmic_efuse->regmap, reg, mask, ~bits);
}

static int pmic_efuse_lock(void)
{
	int ret = 0;

	mutex_lock(&pmic_efuse_mutex);
	ret =
	    hwspin_lock_no_swlock_timeout(pmic_efuse->hw_lock,
					  HWSPINLOCK_TIMEOUT);
	if (ret) {
		pr_err("pmic efuse:lock the hwlock failed.\n");
		mutex_unlock(&pmic_efuse_mutex);
		return ret;
	}
	return ret;
}

static void pmic_efuse_unlock(void)
{
	hwspin_unlock_no_swlock(pmic_efuse->hw_lock);
	mutex_unlock(&pmic_efuse_mutex);
}

static void pmic_efuse_power_on(void)
{
	sprd_efuse_glb_set(ANA_REG_GLB_ARM_MODULE_EN, BIT_ANA_EFS_EN,
			   BIT_ANA_EFS_EN);
}

static void pmic_efuse_power_off(void)
{
	sprd_efuse_glb_clr(ANA_REG_GLB_ARM_MODULE_EN, BIT_ANA_EFS_EN,
			   BIT_ANA_EFS_EN);
}

static int pmic_efuse_wait_clear(u32 bits)
{
	int ret = 0;
	int bit_status = 1;
	unsigned long timeout;
	u32 val;

	pr_info("wait %x\n", pmic_efuse_reg_read(ANA_REG_EFUSE_STATUS));
	udelay(2000);
	/* wait for maximum of 3000 msec */
	timeout = jiffies + msecs_to_jiffies(3000);
	while (bit_status) {
		val = pmic_efuse_reg_read(ANA_REG_EFUSE_STATUS);
		if (pmic_efuse->type == SC2731_EFUSE
		    || pmic_efuse->type == SC2721_EFUSE
		    || pmic_efuse->type == SC2720_EFUSE) {
			bit_status = (~val) & bits;
		} else if (pmic_efuse->type == SC2723_EFUSE) {
			bit_status = val & bits;
		} else {
			pr_err("pmic efuse type unknown\n");
		}
		if (time_after(jiffies, timeout)) {
			WARN_ON(1);
			ret = -ETIMEDOUT;
			break;
		}
		cpu_relax();
	}
	return ret;
}

static u32 pmic_sc2731_efuse_read(int blk_index)
{
	u32 val = 0;
	int ret = 0;

	pr_info("pmic sc2731 efuse read %d\n", blk_index);
	ret = pmic_efuse_lock();
	if (ret)
		return ret;

	pmic_efuse_power_on();
	if (IS_ERR_VALUE(pmic_efuse_wait_clear(BIT_STANDBY_BUSY)))
		goto out;
	pmic_efuse_reg_write(ANA_REG_EFUSE_BLOCK_INDEX,
			     BITS_READ_WRITE_INDEX(blk_index));
	pmic_efuse_reg_set(ANA_REG_EFUSE_MODE_CTRL, BIT_RD_START, BIT_RD_START);
	if (IS_ERR_VALUE(pmic_efuse_wait_clear(BIT_NORMAL_RD_DONE)))
		goto out;
	val = pmic_efuse_reg_read(ANA_REG_EFUSE_DATA_RD);
	pmic_efuse_reg_set(ANA_REG_EFUSE_MODE_CTRL, BIT_NORMAL_RD_FLAG_CLR,
			   BIT_NORMAL_RD_FLAG_CLR);

out:
	pmic_efuse_power_off();
	pmic_efuse_unlock();
	return val;
}

static u32 pmic_sc2723_efuse_read(int blk_index)
{
	u32 val = 0;
	int ret = 0;

	pr_info("pmic sc2723 efuse read %d\n", blk_index);
	ret = pmic_efuse_lock();
	if (ret)
		return ret;

	pmic_efuse_power_on();
	pmic_efuse_reg_write(ANA_REG_EFUSE_BLOCK_INDEX,
			     BITS_READ_WRITE_INDEX(blk_index));
	pmic_efuse_reg_write(ANA_REG_EFUSE_MODE_CTRL, BIT_RD_START);
	if (IS_ERR_VALUE(pmic_efuse_wait_clear(BIT_READ_BUSY)))
		goto out;
	val = pmic_efuse_reg_read(ANA_REG_EFUSE_DATA_RD);

	/* note: reverse the otp value */
	val = BITS_EFUSE_DATA_RD(~val);
out:
	pmic_efuse_power_off();
	pmic_efuse_unlock();
	return val;
}

u32 sprd_pmic_efuse_block_read(int blk_index)
{
	u32 val = 0;

	if (pmic_efuse->type == SC2731_EFUSE
	    || pmic_efuse->type == SC2721_EFUSE
	    || pmic_efuse->type == SC2720_EFUSE)
		val = pmic_sc2731_efuse_read(blk_index);
	else if (pmic_efuse->type == SC2723_EFUSE)
		val = pmic_sc2723_efuse_read(blk_index);
	else
		pr_err("pmic efuse type unknown\n");

	return val;
}
EXPORT_SYMBOL_GPL(sprd_pmic_efuse_block_read);

u32 sprd_pmic_efuse_bits_read(int bit_index, int length)
{
	u32 val = 0;
	int i, blk_index = (int)bit_index / pmic_efuse->efuse_block_width;
	int blk_max = DIV_ROUND_UP(bit_index + length,
				   pmic_efuse->efuse_block_width);

	pr_info("otp read blk %d - %d\n", blk_index, blk_max);
	for (i = blk_index; i < blk_max; i++) {
		val |= sprd_pmic_efuse_block_read(i)
		    << ((i - blk_index) * pmic_efuse->efuse_block_width);
	}
	val >>= (bit_index & (pmic_efuse->efuse_block_width - 1));
	val &= BIT(length) - 1;
	pr_info("otp read bits %d ++ %d 0x%08x\n\n", bit_index, length, val);
	return val;
}
EXPORT_SYMBOL_GPL(sprd_pmic_efuse_bits_read);

static struct sprd_otp_operations pmic_efuse_ops = {
	.read = sprd_pmic_efuse_block_read,
};

static ssize_t dump_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	int i, idx;
	char *p = buf;

	p += sprintf(p, "pmic efuse blocks dump:\n");
	for (idx = 0; idx < pmic_efuse->efuse_block_max; idx++) {
		u32 val = sprd_pmic_efuse_block_read(idx);

		p += sprintf(p, "\n%02d  ", idx);
		for (i = pmic_efuse->efuse_block_width - 1; i >= 0; --i)
			p += sprintf(p, "%s ", (val & BIT(i)) ? "1" : "0");
	}
	p += sprintf(p, "\n");
	return p - buf;
}

static DEVICE_ATTR_RO(dump);

static struct attribute *pmic_efuse_device[] = {
	&dev_attr_dump.attr,
	NULL,
};

static struct attribute_group pmic_efuse_attribute_group = {
	.name = NULL,
	.attrs = pmic_efuse_device,
};

static int sprd_pmic_efuse_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *of_id;
	int ret;
	u32 value;

	if (!np) {
		dev_err(&pdev->dev, "device node not found\n");
		return -ENODEV;
	}

	of_id = of_match_node(pmic_efuse_of_match, pdev->dev.of_node);
	if (!of_id) {
		dev_err(&pdev->dev, "get pmic efuse of device id failed!\n");
		return -ENODEV;
	}

	pmic_efuse = devm_kzalloc(&pdev->dev,
				  sizeof(struct sprd_pmic_efuse), GFP_KERNEL);
	if (!pmic_efuse)
		return -ENOMEM;

	pmic_efuse->type = (enum sprd_pmic_efuse_type)of_id->data;

	pmic_efuse->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!pmic_efuse->regmap) {
		dev_err(&pdev->dev, "fail regmap property for pmic efuse");
		return -ENODEV;
	}

	pmic_efuse->hw_lock =
	    of_hwspin_lock_request(pdev->dev.of_node, "pmic_efuse");
	if (!pmic_efuse->hw_lock) {
		dev_err(&pdev->dev, "pmic efuse can not get hardware spinlock.\n");
		return -ENXIO;
	}

	ret = of_property_read_u32(np, "reg", &value);
	if (ret) {
		dev_err(&pdev->dev, "no reg property for pmic efuse!");
		return ret;
	}

	ret =
	    of_property_read_u32(np, "sprd,block-num",
				 &pmic_efuse->efuse_block_max);
	if (ret) {
		dev_err(&pdev->dev, "parse sprd,block-num err\n");
		return ret;
	}

	ret =
	    of_property_read_u32(np, "sprd,block-width",
				 &pmic_efuse->efuse_block_width);
	if (ret) {
		dev_err(&pdev->dev, "parse sprd,block-width err\n");
		return ret;
	}

	pmic_efuse->base = (unsigned long)value;
	pmic_efuse->dev =
	    sprd_otp_register("sprd_otp_pmic_efuse", &pmic_efuse_ops,
			      pmic_efuse->efuse_block_max,
			      pmic_efuse->efuse_block_width / 8);
	if (IS_ERR_OR_NULL(pmic_efuse->dev)) {
		dev_err(&pdev->dev, "ap efuse register fail\n");
		return PTR_ERR(pmic_efuse->dev);
	}

	ret =
	    sysfs_create_group(&pmic_efuse->dev->kobj,
			       &pmic_efuse_attribute_group);
	if (ret) {
		dev_err(&pdev->dev, "failed to create sysfs device attributes\n");
		return ret;
	}

	dev_info(&pdev->dev, "sprd pmic efuse probe end\n");

	return 0;
}

static int sprd_pmic_efuse_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &pmic_efuse_attribute_group);
	return 0;
}

static struct platform_driver sprd_pmic_efuse_driver = {
	.probe = sprd_pmic_efuse_probe,
	.remove = sprd_pmic_efuse_remove,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "pmic-efuse",
		   .of_match_table = of_match_ptr(pmic_efuse_of_match),
		   },
};

static int __init sprd_pmic_efuse_init(void)
{
	return platform_driver_register(&sprd_pmic_efuse_driver);
}

static void __exit sprd_pmic_efuse_exit(void)
{
	platform_driver_unregister(&sprd_pmic_efuse_driver);
}

subsys_initcall(sprd_pmic_efuse_init);
module_exit(sprd_pmic_efuse_exit);

MODULE_AUTHOR("Freeman Liu <freeman.liu@spreadtrum.com>");
MODULE_DESCRIPTION("Spreadtrum Otp Pmic Efuse Driver");
MODULE_LICENSE("GPL");

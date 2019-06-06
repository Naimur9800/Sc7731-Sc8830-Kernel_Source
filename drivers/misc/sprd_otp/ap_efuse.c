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

/* IMPORTANT:
 * The electrical fuse is a type of non-volatile memory fabricated
 * in standard CMOS logic process. This electrical fuse macro is widely
 * used in chip ID, memory redundancy, security code, configuration setting,
 * and feature selection, etc.
 *
 * TODO:
 * 1. do something when block had been locked with bit31 if need
 * 1. check and clear blk prog/err flag if need
 * 1. wait *300ms* for read/prog ready time-out
 * 1. only mutexlock and hwspinlock for efuse access sync with cp
 * 1. be care not do use efuse API in interrupt function
 * 1. no need soft reset after efuse read/prog and power on/off
 * 1. efuse block width should less than 8bits
 * 1. efuse block count should not bigger than 32!
 * 1. or not should expland the cached otp arrary
 * 1. support efuse DT info (version, blocks, ...) later
 * 1. there is no handle for efuse module removed
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/hwspinlock.h>
#include <linux/module.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/sprd_hwspinlock.h>
#include <linux/sprd_otp.h>
#include <linux/string.h>

#define REG_EFUSE_DATA_RD               0x0000
#define REG_EFUSE_DATA_WR               0x0004
#define REG_EFUSE_READ_WRITE_INDEX      0x0008
#define REG_EFUSE_MODE_CTRL             0x000c
#define REG_EFUSE_CFG0                  0x0010
#define REG_EFUSE_CFG1                  0x0014
#define REG_EFUSE_STATUS                0x0020
#define REG_EFUSE_BLK_FLAG0             0x0024
#define REG_EFUSE_BLK_FLAG1             0x0028
#define REG_EFUSE_BLK_FLAG0_CLR         0x0030
#define REG_EFUSE_BLK_FLAG1_CLR         0x0034
#define REG_EFUSE_MAGIC_NUMBER          0x0040
#define REG_EFUSE_STROBE_LOW_WIDTH      0x0044
#define REG_EFUSE_EFUSE_DEB_CTRL        0x0048

#define MASK_READ_WRITE_INDEX   (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4))

/* bits definitions for register REG_EFUSE_MODE_CTRL */
#define BIT_PG_START                  (BIT(0))
#define BIT_RD_START                  (BIT(1))
#define BIT_STANDBY_START             (BIT(2))

#define BITS_EFS_TYPE(_x_)            ((_x_) << 16 & (BIT(16)|BIT(17)))
#define BIT_EFS_VDDQ_K1_ON            (BIT(28))
#define BIT_EFS_VDDQ_K2_ON            (BIT(29))

/* Set this bit will open 0.9v static power supply for efuse memory,
 * before any operation towards to efuse memory this bit have to set to 1.
 * Once this bit is cleared, the efuse will go to power down mode.
 */
#define BIT_EFS_VDD_ON                  (BIT(30))
#define BIT_PGM_EN                      (BIT(31))

/* bits definitions for register REG_EFUSE_STATUS */
#define BIT_PGM_BUSY                    (BIT(0))
#define BIT_READ_BUSY                   (BIT(1))
#define BIT_STANDBY_BUSY                (BIT(2))

/* Magic number, only when this field is 0x8810, the efuse programming
 * command can be handle. So, if SW want to program efuse memory,
 * except open clocks and power, the follow conditions must be met:
 * 1. PGM_EN = 1;
 * 2. EFUSE_MAGIC_NUMBER = 0x8810
 */
#define BITS_MAGIC_NUMBER(_x_)          ((_x_) << 0 & (BIT(0)|BIT(1)\
	|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)\
	|BIT(11)|BIT(12)|BIT(13)|BIT(14)|BIT(15)))

#define EFUSE_MAGIC_NUMBER      (0x8810)
#define WHALE_LEAK_BIN_BLOCK 15
#define HWSPINLOCK_TIMEOUT		(5000)
static unsigned long efuse_hwlock_flag;

enum sprd_ap_efuse_type {
	UNKNOWN_AP_EFUSE,
	SPRD_R1P0_AP_EFUSE,
	SPRD_R2P0_AP_EFUSE,
};

struct sprd_ap_efuse {
	enum sprd_ap_efuse_type type;
	struct regmap *aon_apb_glb;
	struct device *dev;
	struct clk *clk;
	struct hwspinlock *hw_lock;
	unsigned long base;
	unsigned long efuse_hwlock_flag;
	unsigned int blk_num;
	unsigned int blk_width;
};

static struct sprd_ap_efuse *ap_efuse;
static u32 efuse_magic;

static DEFINE_MUTEX(efuse_mtx);

static int ap_efuse_lock(void)
{
	int ret = 0;

	ret = hwspin_lock_timeout_irqsave(ap_efuse->hw_lock,
					  HWSPINLOCK_TIMEOUT,
					  &efuse_hwlock_flag);
	if (ret)
		pr_err("ap efuse read lock failed.\n");

	return ret;
}

static void ap_efuse_unlock(void)
{
	hwspin_unlock_irqrestore(ap_efuse->hw_lock, &efuse_hwlock_flag);
}

static inline unsigned int ap_efuse_reg_read(unsigned long reg)
{
	return readl_relaxed((void __iomem *)(reg + ap_efuse->base));
}

static inline void ap_efuse_reg_write(unsigned int or_val, unsigned long reg)
{
	writel_relaxed(or_val, (void __iomem *)(reg + ap_efuse->base));
}

static void sprd_ap_efuse_reset(void)
{
	unsigned int mask;

	mask = BIT_AON_APB_EFUSE_SOFT_RST;
	regmap_update_bits(ap_efuse->aon_apb_glb, REG_AON_APB_APB_RST0,
			   mask, mask);
	udelay(5);
	regmap_update_bits(ap_efuse->aon_apb_glb, REG_AON_APB_APB_RST0,
			   mask, ~mask);
}

/* FIXME: Set EFS_VDD_ON will open 0.9v static power supply for efuse memory,
 * before any operation towards to efuse memory this bit have to set to 1.
 * Once this bit is cleared, the efuse will go to power down mode.
 *
 * each time when EFS_VDD_ON changes, software should wait at least 1ms to let
 * VDD become stable.
 *
 * For VDDQ(1.8v) power, to prevent the overshot of VDDQ, a extra power switch
 * connected to ground are controlled by "EFS_VDDQ_K2_ON"
 */
static void ap_efuse_prog_power_on(void)
{
	u32 cfg0;

	cfg0 = ap_efuse_reg_read(REG_EFUSE_CFG0);
	cfg0 &= ~(BIT_EFS_VDDQ_K2_ON | BIT_EFS_VDDQ_K1_ON);
	cfg0 |= BIT_EFS_VDD_ON;
	ap_efuse_reg_write(cfg0, REG_EFUSE_CFG0);
	udelay(1000);
	cfg0 |= BIT_EFS_VDDQ_K1_ON;
	ap_efuse_reg_write(cfg0, REG_EFUSE_CFG0);
	udelay(1000);
}

static void ap_efuse_read_power_on(void)
{
	u32 cfg0;

	cfg0 = ap_efuse_reg_read(REG_EFUSE_CFG0);
	cfg0 &= ~BIT_EFS_VDDQ_K1_ON;
	cfg0 |= BIT_EFS_VDD_ON | BIT_EFS_VDDQ_K2_ON;
	ap_efuse_reg_write(cfg0, REG_EFUSE_CFG0);
	udelay(1000);
}

static void ap_efuse_power_off(void)
{
	u32 cfg0;

	cfg0 = ap_efuse_reg_read(REG_EFUSE_CFG0);
	if (cfg0 & BIT_EFS_VDDQ_K1_ON) {
		cfg0 &= ~BIT_EFS_VDDQ_K1_ON;
		ap_efuse_reg_write(cfg0, REG_EFUSE_CFG0);
		udelay(1000);
	}
	cfg0 |= BIT_EFS_VDDQ_K2_ON;
	cfg0 &= ~BIT_EFS_VDD_ON;
	ap_efuse_reg_write(cfg0, REG_EFUSE_CFG0);
	udelay(1000);
}

static int ap_efuse_wait_clear(u32 bits)
{
	int ret = 0, cnt = 50;

	udelay(2);
	/* wait for maximum of 100 us */
	while ((ap_efuse_reg_read(REG_EFUSE_STATUS) & bits) && cnt--)
		udelay(2);

	if (cnt <= 0) {
		pr_err("efuse read 0x%08x\n",
		       ap_efuse_reg_read(REG_EFUSE_STATUS));
		ret = -ETIMEDOUT;
	}

	return ret;
}

static u32 ap_efuse_read(int blk)
{
	u32 val = 0;

	ap_efuse_reg_write(blk, REG_EFUSE_READ_WRITE_INDEX);
	ap_efuse_reg_write(BIT_RD_START, REG_EFUSE_MODE_CTRL);
	if (IS_ERR_VALUE(ap_efuse_wait_clear(BIT_READ_BUSY)))
		goto out;
	val = ap_efuse_reg_read(REG_EFUSE_DATA_RD);

out:
	return val;
}

static int ap_efuse_prog(int blk, u32 val)
{
	u32 cfg0;

	cfg0 = ap_efuse_reg_read(REG_EFUSE_CFG0);
	if (blk < 0 || blk >= ap_efuse->blk_num)
		goto out;
	/* enable pgm mode and setup magic number before programming */
	cfg0 |= BIT_PGM_EN;
	ap_efuse_reg_write(cfg0, REG_EFUSE_CFG0);
	ap_efuse_reg_write(BITS_MAGIC_NUMBER(efuse_magic),
			   REG_EFUSE_MAGIC_NUMBER);

	ap_efuse_reg_write(val, REG_EFUSE_DATA_WR);
	ap_efuse_reg_write(blk, REG_EFUSE_READ_WRITE_INDEX);
	pr_info("cfg0 0x%x\n", ap_efuse_reg_read(REG_EFUSE_CFG0));
	ap_efuse_reg_write(BIT_PG_START, REG_EFUSE_MODE_CTRL);

	if (IS_ERR_VALUE(ap_efuse_wait_clear(BIT_PGM_BUSY)))
		pr_info("ap efuse prog busy\n");
out:
	ap_efuse_reg_write(0, REG_EFUSE_MAGIC_NUMBER);
	cfg0 &= ~BIT_PGM_EN;
	ap_efuse_reg_write(cfg0, REG_EFUSE_CFG0);
	return 0;
}

u32 sprd_ap_efuse_read(int blk_index)
{
	u32 val = 0;
	int ret = 0;

	pr_info("efuse read %d\n", blk_index);
	ret = ap_efuse_lock();
	if (ret)
		return ret;
	clk_enable(ap_efuse->clk);
	ap_efuse_read_power_on();
	val = ap_efuse_read(blk_index);
	ap_efuse_power_off();
	clk_disable(ap_efuse->clk);
	ap_efuse_unlock();
	pr_info("efuse read 0x%08x\n", val);
	return val;
}
EXPORT_SYMBOL_GPL(sprd_ap_efuse_read);

int sprd_ap_efuse_prog(int blk_index, u32 val)
{
	int ret;

	pr_info("efuse prog %d 0x%08x\n", blk_index, val);
	ret = ap_efuse_lock();
	if (ret)
		return ret;
	clk_enable(ap_efuse->clk);
	ap_efuse_prog_power_on();
	ret = ap_efuse_prog(blk_index, val);
	ap_efuse_power_off();
	clk_disable(ap_efuse->clk);
	ap_efuse_unlock();
	return ret;
}
EXPORT_SYMBOL_GPL(sprd_ap_efuse_prog);

static struct sprd_otp_operations efuse_ops = {
	.reset = sprd_ap_efuse_reset,
	.read = sprd_ap_efuse_read,
	.prog = sprd_ap_efuse_prog,
};

static ssize_t magic_store(struct device *pdev,
			   struct device_attribute *attr,
			   const char *buff, size_t size)
{
	int ret;

	ret = kstrtouint(buff, 16, &efuse_magic);
	if (ret)
		return ret;
	return size;
}
static DEVICE_ATTR_WO(magic);

static ssize_t get_magic_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%x\n", efuse_magic);
}
static DEVICE_ATTR_RO(get_magic);

static ssize_t dump_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	int idx;
	char *p = buf;

	p += sprintf(p, "ap efuse blocks dump:\n");
	for (idx = 0; idx < ap_efuse->blk_num; idx++) {
		p += sprintf(p, "[%02d] %08x\n", idx, sprd_ap_efuse_read(idx));
		pr_info("[%02d] %08x\n", idx, sprd_ap_efuse_read(idx));
	}
	return p - buf;
}
static DEVICE_ATTR_RO(dump);

static ssize_t uid_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u32 block0, block1;
	u32 x, y, wafer_id;
	u32 LOTID_0, LOTID_1, LOTID_2, LOTID_3, LOTID_4, LOTID_5;
	char *p = buf;

	block0 = sprd_ap_efuse_read(0);
	block1 = sprd_ap_efuse_read(1);

	y = block1 & 0x7F;
	x = (block1 >> 7) & 0x7F;
	wafer_id = (block1 >> 14) & 0x1F;
	LOTID_0 = (block1 >> 19) & 0x3F;
	LOTID_1 = (block1 >> 25) & 0x3F;
	LOTID_2 = block0 & 0x3F;
	LOTID_3 = (block0 >> 6) & 0x3F;
	LOTID_4 = (block0 >> 12) & 0x3F;
	LOTID_5 = (block0 >> 18) & 0x3F;

	p += sprintf(p, "ap efuse uid dump:\n");
	p += sprintf(p, "%c%c%c%c%c%c_%d_%d_%d\n", LOTID_5 + 48, LOTID_4 + 48,
		     LOTID_3 + 48, LOTID_2 + 48, LOTID_1 + 48, LOTID_0 + 48,
		     wafer_id, x, y);
	p += sprintf(p, "\n");
	return p - buf;
}
static DEVICE_ATTR_RO(uid);

void sprd_get_chip_uid(char *buf)
{
	u32 block0, block1;
	u32 x, y, wafer_id;
	u32 LOTID_0, LOTID_1, LOTID_2, LOTID_3, LOTID_4, LOTID_5;
	char *p = buf;

	block0 = sprd_ap_efuse_read(0);
	block1 = sprd_ap_efuse_read(1);

	y = block1 & 0x7F;
	x = (block1 >> 7) & 0x7F;
	wafer_id = (block1 >> 14) & 0x1F;
	LOTID_0 = (block1 >> 19) & 0x3F;
	LOTID_1 = (block1 >> 25) & 0x3F;
	LOTID_2 = block0 & 0x3F;
	LOTID_3 = (block0 >> 6) & 0x3F;
	LOTID_4 = (block0 >> 12) & 0x3F;
	LOTID_5 = (block0 >> 18) & 0x3F;

	pr_info("uid is %x%x\n", block0, block1);
	p += sprintf(p, "%c%c%c%c%c%c_%d_%d_%d", LOTID_5 + 48, LOTID_4 + 48,
		     LOTID_3 + 48, LOTID_2 + 48, LOTID_1 + 48, LOTID_0 + 48,
		     wafer_id, x, y);
}

EXPORT_SYMBOL_GPL(sprd_get_chip_uid);

static ssize_t uidval_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	char *p = buf;
	char uid[30];

	sprd_get_chip_uid(uid);
	p += sprintf(p, "%s\n", uid);
	return p - buf;
}
static DEVICE_ATTR_RO(uidval);

static struct attribute *ap_efuse_device[] = {
	&dev_attr_magic.attr,
	&dev_attr_get_magic.attr,
	&dev_attr_dump.attr,
	&dev_attr_uid.attr,
	&dev_attr_uidval.attr,
	NULL,
};

static struct attribute_group ap_efuse_attribute_group = {
	.name = NULL,
	.attrs = ap_efuse_device,
};

static const struct of_device_id ap_efuse_of_match[] = {
	{.compatible = "sprd,ap_efuse", .data = (void *)SPRD_R1P0_AP_EFUSE,},
	{.compatible = "sprd,ap_r1p0_efuse", .data =
	 (void *)SPRD_R1P0_AP_EFUSE,},
	{.compatible = "sprd,ap_256X32_efuse", .data =
	 (void *)SPRD_R1P0_AP_EFUSE,},
	{}
};

static int sprd_ap_efuse_probe(struct platform_device *pdev)
{
	struct resource *res = NULL;
	struct regmap *efuse_glb = NULL;
	const struct of_device_id *of_id;
	void __iomem *efuse_base = NULL;
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;

	if (!np) {
		dev_err(&pdev->dev, "device node not found\n");
		return -ENODEV;
	}

	dev_info(&pdev->dev, "sprd ap efuse probe start\n");
	of_id = of_match_node(ap_efuse_of_match, pdev->dev.of_node);
	if (!of_id) {
		dev_err(&pdev->dev, "get id of ap efuse failed!\n");
		return -ENODEV;
	}

	efuse_glb = syscon_regmap_lookup_by_phandle(np, "sprd,syscon-enable");
	if (IS_ERR(efuse_glb)) {
		dev_err(&pdev->dev, "ap efuse syscon failed!\n");
		return -ENODEV;
	}

	ap_efuse = devm_kzalloc(&pdev->dev,
				sizeof(struct sprd_ap_efuse), GFP_KERNEL);
	if (!ap_efuse)
		return -ENOMEM;

	ap_efuse->type = (enum sprd_ap_efuse_type)of_id->data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	efuse_base = devm_ioremap_resource(&pdev->dev, res);
	if (!efuse_base) {
		dev_err(&pdev->dev, "remap ap efuse address failed!\n");
		return -ENOMEM;
	}

	ap_efuse->clk = of_clk_get_by_name(pdev->dev.of_node, "enable");
	if (IS_ERR(ap_efuse->clk)) {
		dev_err(&pdev->dev, "ap efuse can't get the clock dts config: enable\n");
		return PTR_ERR(ap_efuse->clk);
	}
	clk_prepare(ap_efuse->clk);

	ap_efuse->hw_lock =
	    of_hwspin_lock_request(pdev->dev.of_node, "ap_efuse");
	if (!ap_efuse->hw_lock) {
		dev_err(&pdev->dev, "ap_efuse can not get the hardware spinlock.\n");
		ret = -ENXIO;
		goto clk_fail;
	}

	ret = of_property_read_u32(np, "sprd,block-num", &ap_efuse->blk_num);
	if (ret) {
		dev_err(&pdev->dev, "ap efuse get sprd,block-num err\n");
		goto lock_fail;
	}

	ret =
	    of_property_read_u32(np, "sprd,block-width", &ap_efuse->blk_width);
	if (ret) {
		dev_err(&pdev->dev, "ap efuse get sprd,block-width err\n");
		goto lock_fail;
	}

	ap_efuse->base = (unsigned long)efuse_base;
	ap_efuse->aon_apb_glb = efuse_glb;
	ap_efuse->dev =
	    sprd_otp_register("sprd_otp_ap_efuse", &efuse_ops,
			      ap_efuse->blk_num, ap_efuse->blk_width / 8);
	if (IS_ERR_OR_NULL(ap_efuse->dev)) {
		pr_err("ap efuse register fail\n");
		ret = PTR_ERR(ap_efuse->dev);
		goto lock_fail;
	}

	ret =
	    sysfs_create_group(&ap_efuse->dev->kobj, &ap_efuse_attribute_group);
	if (ret) {
		pr_err("failed to create sysfs device attributes\n");
		goto lock_fail;
	}

	dev_info(&pdev->dev, "sprd_efuse_probe end\n");
	return 0;

lock_fail:
	hwspin_lock_free(ap_efuse->hw_lock);
clk_fail:
	clk_unprepare(ap_efuse->clk);

	return ret;

}

static int sprd_ap_efuse_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &ap_efuse_attribute_group);
	clk_unprepare(ap_efuse->clk);
	return 0;
}

static struct platform_driver sprd_ap_efuse_driver = {
	.probe = sprd_ap_efuse_probe,
	.remove = sprd_ap_efuse_remove,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "ap_efuse",
		   .of_match_table = of_match_ptr(ap_efuse_of_match),
		   },
};

static int __init sprd_ap_efuse_init(void)
{
	return platform_driver_register(&sprd_ap_efuse_driver);
}

static void __exit sprd_ap_efuse_exit(void)
{
	platform_driver_unregister(&sprd_ap_efuse_driver);
}

subsys_initcall(sprd_ap_efuse_init);
module_exit(sprd_ap_efuse_exit);

MODULE_AUTHOR("Freeman Liu <freeman.liu@spreadtrum.com>");
MODULE_DESCRIPTION("Spreadtrum Otp Ap Efuse Driver");
MODULE_LICENSE("GPL");

/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
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
 * 1. check and clear blk prog/err flag if need
 * 1. only mutexlock and hwspinlock for efuse access sync with cp
 * 1. be care not do use efuse API in interrupt function
 * 1. no need soft reset after efuse read/prog and power on/off
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

#define EFUSE_ALL0_INDEX		0x8
#define EFUSE_MODE_CTRL			0xc
#define EFUSE_IP_VER			0x14
#define EFUSE_CFG0			0x18
#define EFUSE_NS_EN			0x20
#define EFUSE_NS_ERR_FLAG		0x24
#define EFUSE_NS_FLAG_CLR		0x28
#define EFUSE_NS_MAGIC_NUM		0x2c
#define EFUSE_FW_CFG			0x50
#define EFUSE_PW_SWT			0x54
#define EFUSE_MEM(val)			(0x1000 + (val << 2))

/* bits definitions for register EFUSE_MODE_CTRL */

#define BIT_EFUSE_ALL0_CHECK_START       BIT(0)

/* bits definitions for register EFUSE_NS_EN */

#define BIT_NS_VDD_EN			BIT(0)
#define BIT_NS_AUTO_CHECK_ENABLE		BIT(1)
#define BIT_DOUBLE_BIT_EN_NS            BIT(2)
#define BIT_NS_MARGIN_RD_ENABLE         BIT(3)
#define BIT_NS_LOCK_BIT_WR_EN           BIT(4)

/* bits definitions for register EFUSE_NS_ERR_FLAG */
#define BIT_NS_WORD0_ERR_FLAG		BIT(0)
#define BIT_NS_WORD1_ERR_FLAG		BIT(1)
#define BIT_NS_WORD0_PROT_FLAG		BIT(4)
#define BIT_NS_WORD1_PROT_FLAG		BIT(5)
#define BIT_NS_PG_EN_WR_FLAG		BIT(8)
#define BIT_NS_VDD_ON_RD_FLAG		BIT(9)
#define BIT_NS_BLOCK0_RD_FLAG		BIT(10)
#define BIT_NS_MAGNUM_WR_FLAG		BIT(11)
#define BIT_NS_ENK_ERR_FLAG		BIT(12)
#define BIT_NS_ALL0_CHECK_FLAG		BIT(13)

/* bits definitions for register EFUSE_NS_FLAG_CLR */
#define BIT_NS_WORD0_ERR_CLR		BIT(0)
#define BIT_NS_WORD1_ERR_CLR		BIT(1)
#define BIT_NS_WORD0_PROT_CLR		BIT(4)
#define BIT_NS_WORD1_PROT_CLR		BIT(5)
#define BIT_NS_PG_EN_WR_CLR		BIT(8)
#define BIT_NS_VDD_ON_RD_CLR		BIT(9)
#define BIT_NS_BLOCK0_RD_CLR		BIT(10)
#define BIT_NS_MAGNUM_WR_CLR		BIT(11)
#define BIT_NS_ENK_ERR_CLR		BIT(12)
#define BIT_NS_ALL0_CHECK_CLR		BIT(13)

/* bits definitions for register EFUSE_PW_SWT */

#define BIT_EFS_ENK1_ON				BIT(0)
#define BIT_EFS_ENK2_ON				BIT(1)
#define BIT_NS_S_PG_EN				BIT(2)

/* Magic number, only when this field is 0x8810, the efuse programming
 * command can be handle. So, if SW want to program efuse memory,
 * except open clocks and power, the follow conditions must be met:
 * 1. PGM_EN = 1;
 * 2. EFUSE_MAGIC_NUMBER = 0x8810
 */
#define BITS_NS_EFUSE_MAGIC_NUMBER(_x_)          ((_x_) << 0 & (BIT(0)|BIT(1)\
	|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)\
	|BIT(11)|BIT(12)|BIT(13)|BIT(14)|BIT(15)))

#define NS_AUTO_CHECK_FLAG (BIT_NS_WORD0_ERR_FLAG | BIT_NS_WORD1_ERR_FLAG)
#define UID_XY_MASK		GENMASK(6, 0)
#define UID_WAFER_MASK		GENMASK(4, 0)
#define UID_LOTID_MASK		GENMASK(5, 0)
#define ERR_CLR_MASK		GENMASK(13, 0)

#define EFUSE_MAGIC_NUMBER      (0x8810)
#define HWSPINLOCK_TIMEOUT		(5000)

enum sprd_public_efuse_type {
	UNKNOWN_AP_EFUSE,
	SPRD_R6P0_AP_EFUSE,
};

struct sprd_public_ap_efuse {
	enum sprd_public_efuse_type type;
	struct regmap *aon_apb_glb;
	struct device *dev;
	struct clk *clk;
	struct hwspinlock *hw_lock;
	unsigned long base;
	unsigned long efuse_hwlock_flag;
	unsigned int efuse_magic;
	unsigned int uid_start[2];
	unsigned int uid_end[2];
	unsigned int blk_start;
	unsigned int blk_num;
	unsigned int blk_width;
};

static unsigned long efuse_hwlock_flag;
static struct sprd_public_ap_efuse *ap_efuse;
static DEFINE_MUTEX(efuse_mtx);

static int ap_efuse_lock(void)
{
	int ret;

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

static void ap_efuse_prog_power_on(void)
{
	u32 cfg0;

	cfg0 = ap_efuse_reg_read(EFUSE_PW_SWT);
	cfg0 &= ~BIT_EFS_ENK2_ON;
	ap_efuse_reg_write(cfg0, EFUSE_PW_SWT);
	udelay(1000);
	cfg0 |= BIT_EFS_ENK1_ON;
	ap_efuse_reg_write(cfg0, EFUSE_PW_SWT);
	udelay(1000);
}

static void ap_efuse_prog_power_off(void)
{
	u32 cfg0;

	cfg0 = ap_efuse_reg_read(EFUSE_PW_SWT);
	cfg0 &= ~BIT_EFS_ENK1_ON;
	ap_efuse_reg_write(cfg0, EFUSE_PW_SWT);
	udelay(1000);
	cfg0 |= BIT_EFS_ENK2_ON;
	ap_efuse_reg_write(cfg0, EFUSE_PW_SWT);
	udelay(1000);
}

static void ap_efuse_read_power_on(void)
{
	u32 cfg0;

	cfg0 = ap_efuse_reg_read(EFUSE_NS_EN);
	cfg0 |= BIT_NS_VDD_EN;
	ap_efuse_reg_write(cfg0, EFUSE_NS_EN);
	udelay(1000);
}

static void ap_efuse_read_power_off(void)
{
	u32 cfg0;

	cfg0 = ap_efuse_reg_read(EFUSE_NS_EN);
	cfg0 &= ~BIT_NS_VDD_EN;
	ap_efuse_reg_write(cfg0, EFUSE_NS_EN);
	udelay(1000);
}

static void ap_efuse_prog_lock(bool en_lock)
{
	u32 cfg0;

	cfg0 = ap_efuse_reg_read(EFUSE_NS_EN);
	if (en_lock)
		cfg0 |= BIT_NS_LOCK_BIT_WR_EN;
	else
		cfg0 &= ~BIT_NS_LOCK_BIT_WR_EN;

	ap_efuse_reg_write(cfg0, EFUSE_NS_EN);
}

static void ap_efuse_auto_check(bool en_lock)
{
	u32 cfg0;

	cfg0 = ap_efuse_reg_read(EFUSE_NS_EN);
	if (en_lock)
		cfg0 |= BIT_NS_AUTO_CHECK_ENABLE;
	else
		cfg0 &= ~BIT_NS_AUTO_CHECK_ENABLE;

	ap_efuse_reg_write(cfg0, EFUSE_NS_EN);
}

static void ap_efuse_double(bool backup)
{
	u32 cfg0;

	cfg0 = ap_efuse_reg_read(EFUSE_NS_EN);
	if (backup)
		cfg0 |= BIT_DOUBLE_BIT_EN_NS;
	else
		cfg0 &= ~BIT_DOUBLE_BIT_EN_NS;

	ap_efuse_reg_write(cfg0, EFUSE_NS_EN);
}

static int ap_efuse_prog(u32 blk, bool backup, bool lock, u32 val)
{
	u32 cfg0;
	int ret;

	ap_efuse_reg_write(BITS_NS_EFUSE_MAGIC_NUMBER(ap_efuse->efuse_magic),
			   EFUSE_NS_MAGIC_NUM);
	ap_efuse_prog_power_on();
	ap_efuse_reg_write(ERR_CLR_MASK, EFUSE_NS_FLAG_CLR);
	cfg0 = ap_efuse_reg_read(EFUSE_PW_SWT);
	cfg0 |= BIT_NS_S_PG_EN;
	ap_efuse_reg_write(cfg0, EFUSE_PW_SWT);

	ap_efuse_double(backup);
	ap_efuse_auto_check(true);
	ap_efuse_reg_write(val, EFUSE_MEM(blk));
	ap_efuse_auto_check(false);
	ap_efuse_double(false);
	ret = ap_efuse_reg_read(EFUSE_NS_ERR_FLAG);
	if (!ret) {
		ap_efuse_prog_lock(lock);
		ap_efuse_reg_write(0x0, EFUSE_MEM(blk));
		ap_efuse_prog_lock(false);
	}
	ap_efuse_reg_write(ERR_CLR_MASK, EFUSE_NS_FLAG_CLR);

	cfg0 = ap_efuse_reg_read(EFUSE_PW_SWT);
	cfg0 &= ~BIT_NS_S_PG_EN;
	ap_efuse_reg_write(cfg0, EFUSE_PW_SWT);
	ap_efuse_prog_power_off();
	ap_efuse_reg_write(0, EFUSE_NS_MAGIC_NUM);

	return ret;
}

static u32 ap_efuse_read(int blk, bool backup)
{
	u32 val;

	ap_efuse_double(backup);
	val = ap_efuse_reg_read(EFUSE_MEM(blk));
	ap_efuse_double(false);
	return val;
}

u32 sprd_efuse_double_read(int blk, bool backup)
{
	u32 val;
	int ret;

	ret = ap_efuse_lock();
	if (ret)
		return 0;
	clk_enable(ap_efuse->clk);
	ap_efuse_reg_write(ERR_CLR_MASK, EFUSE_NS_FLAG_CLR);
	ap_efuse_read_power_on();
	val = ap_efuse_read(blk, backup);
	ap_efuse_read_power_off();
	ret = ap_efuse_reg_read(EFUSE_NS_ERR_FLAG);
	if (ret) {
		ap_efuse_reg_write(ERR_CLR_MASK, EFUSE_NS_FLAG_CLR);
		val = 0;
	}
	clk_disable(ap_efuse->clk);
	ap_efuse_unlock();
	pr_info("efuse read blk %d, ret 0x%x, val 0x%08x\n", blk, ret, val);

	return val;
}
EXPORT_SYMBOL_GPL(sprd_efuse_double_read);

u32 sprd_ap_efuse_read(int blk)
{
	return sprd_efuse_double_read(blk, 1);
}
EXPORT_SYMBOL_GPL(sprd_ap_efuse_read);

u32 sprd_ap_efuse_single_read(int blk)
{
	return sprd_efuse_double_read(blk, 0);
}
EXPORT_SYMBOL_GPL(sprd_ap_efuse_single_read);

int sprd_efuse_double_prog(int blk, bool backup, bool lock, u32 val)
{
	int ret;

	ret = ap_efuse_lock();
	if (ret)
		return ret;
	clk_enable(ap_efuse->clk);
	ret = ap_efuse_prog(blk, backup, lock, val);
	clk_disable(ap_efuse->clk);
	ap_efuse_unlock();
	pr_info("efuse prog blk %d, ret 0x%x, val 0x%08x\n", blk, ret, val);
	if (ret)
		ret = -EIO;
	return ret;
}
EXPORT_SYMBOL_GPL(sprd_efuse_double_prog);

int sprd_ap_efuse_prog(int blk_index, u32 val)
{
	pr_info("efuse prog %d 0x%08x\n", blk_index, val);
	return sprd_efuse_double_prog(blk_index, 0, 0, val);
}
EXPORT_SYMBOL_GPL(sprd_ap_efuse_prog);

static struct sprd_otp_operations efuse_ops = {
	.reset = sprd_ap_efuse_reset,
	.read = sprd_ap_efuse_single_read,
	.prog = sprd_ap_efuse_prog,
};

void sprd_get_chip_uid(char *buf)
{
	u32 block0, block1;
	u32 x, y, wafer_id;
	u32 LOTID_0, LOTID_1, LOTID_2, LOTID_3, LOTID_4, LOTID_5;
	char *p = buf;

	block1 = sprd_efuse_double_read(ap_efuse->uid_start[0],
					ap_efuse->uid_start[1]);
	block0 = sprd_efuse_double_read(ap_efuse->uid_end[0],
					ap_efuse->uid_end[1]);

	y = block1 & UID_XY_MASK;
	x = (block1 >> 7) & UID_XY_MASK;
	wafer_id = (block1 >> 14) & UID_WAFER_MASK;
	LOTID_0 = (block1 >> 19) & UID_LOTID_MASK;
	LOTID_1 = (block1 >> 25) & UID_LOTID_MASK;
	LOTID_2 = block0 & UID_LOTID_MASK;
	LOTID_3 = (block0 >> 6) & UID_LOTID_MASK;
	LOTID_4 = (block0 >> 12) & UID_LOTID_MASK;
	LOTID_5 = (block0 >> 18) & UID_LOTID_MASK;

	pr_info("uid is %x%x\n", block0, block1);
	p += sprintf(p, "%c%c%c%c%c%c_%d_%d_%d", LOTID_5 + 48, LOTID_4 + 48,
		     LOTID_3 + 48, LOTID_2 + 48, LOTID_1 + 48, LOTID_0 + 48,
		     wafer_id, x, y);
}
EXPORT_SYMBOL_GPL(sprd_get_chip_uid);

static ssize_t dump_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	int idx, blknum;
	char *p = buf;

	blknum = ap_efuse->blk_start + ap_efuse->blk_num;
	p += sprintf(p, "ap efuse blocks dump:\n");
	for (idx = ap_efuse->blk_start; idx <= blknum; idx++) {
		p += sprintf(p, "[%02d] %08x\n", idx,
			     sprd_efuse_double_read(idx, true));
	}

	p += sprintf(p, "[%02d] %08x\n", ap_efuse->uid_start[0],
			     sprd_efuse_double_read(ap_efuse->uid_start[0],
						    ap_efuse->uid_start[1]));
	p += sprintf(p, "[%02d] %08x\n", ap_efuse->uid_end[0],
			     sprd_efuse_double_read(ap_efuse->uid_end[0],
						    ap_efuse->uid_end[1]));
	return p - buf;
}
static DEVICE_ATTR_RO(dump);

static ssize_t uid_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	char *p = buf;
	char uid[25];

	sprd_get_chip_uid(uid);
	p += sprintf(p, "ap efuse uid dump:\n");
	p += sprintf(p, "%s\n", uid);
	p += sprintf(p, "\n");
	return p - buf;
}
static DEVICE_ATTR_RO(uid);

static ssize_t uidval_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	char *p = buf;
	char uid[25];

	sprd_get_chip_uid(uid);
	p += sprintf(p, "%s\n", uid);
	p += sprintf(p, "\n");
	return p - buf;
}
static DEVICE_ATTR_RO(uidval);

static ssize_t magic_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t size)
{
	int ret;

	ret = kstrtouint(buf, 16, &ap_efuse->efuse_magic);
	if (ret)
		return ret;
	return size;
}

static ssize_t magic_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%x\n", ap_efuse->efuse_magic);
}
static DEVICE_ATTR_RW(magic);

static struct attribute *ap_efuse_device[] = {
	&dev_attr_dump.attr,
	&dev_attr_uid.attr,
	&dev_attr_uidval.attr,
	&dev_attr_magic.attr,
	NULL,
};

static struct attribute_group ap_efuse_attribute_group = {
	.name = NULL,
	.attrs = ap_efuse_device,
};

static const struct of_device_id ap_efuse_of_match[] = {
	{.compatible = "sprd,ap_r6p0_efuse", .data =
	 (void *)SPRD_R6P0_AP_EFUSE,},
	{}
};

static int sprd_ap_efuse_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct regmap *efuse_glb;
	const struct of_device_id *of_id;
	void __iomem *efuse_base;
	int ret;
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
		return PTR_ERR(efuse_glb);
	}

	ap_efuse = devm_kzalloc(&pdev->dev,
				sizeof(struct sprd_public_ap_efuse),
				GFP_KERNEL);
	if (!ap_efuse)
		return -ENOMEM;

	ap_efuse->type = (enum sprd_public_efuse_type)of_id->data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	efuse_base = devm_ioremap_resource(&pdev->dev, res);
	if (!efuse_base) {
		dev_err(&pdev->dev, "remap ap efuse address failed!\n");
		return -ENOMEM;
	}

	ap_efuse->clk = devm_clk_get(&pdev->dev, "enable");
	if (IS_ERR(ap_efuse->clk)) {
		dev_err(&pdev->dev, "ap efuse can't get the clock dts\n");
		return PTR_ERR(ap_efuse->clk);
	}
	clk_prepare(ap_efuse->clk);

	ap_efuse->hw_lock =
	    of_hwspin_lock_request(pdev->dev.of_node, "ap_efuse");
	if (!ap_efuse->hw_lock) {
		dev_err(&pdev->dev,
			"ap_efuse not get the hardware spinlock.\n");
		ret = -ENXIO;
		goto clk_fail;
	}

	ret = of_property_read_u32_array(np, "sprd,uid-start",
					 ap_efuse->uid_start, 2);
	if (ret) {
		dev_err(&pdev->dev, "ap efuse get sprd,uid-start err\n");
		goto lock_fail;
	}

	ret = of_property_read_u32_array(np, "sprd,uid-end",
					 ap_efuse->uid_end, 2);
	if (ret) {
		dev_err(&pdev->dev, "ap efuse get sprd,uid-end err\n");
		goto lock_fail;
	}

	ret = of_property_read_u32(np, "sprd,block-start",
				   &ap_efuse->blk_start);
	if (ret) {
		dev_err(&pdev->dev, "ap efuse get sprd,block-start err\n");
		goto lock_fail;
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
		   .name = "ap-public-efuse",
		   .of_match_table = ap_efuse_of_match,
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

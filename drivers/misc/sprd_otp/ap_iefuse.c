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

/* IMPORTANT:
 * The electrical fuse is a type of non-volatile memory fabricated
 * in standard CMOS logic process. This electrical fuse macro is widely
 * used in chip ID, memory redundancy, security code, configuration
 * setting and feature selection, etc.
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
#define REG_EFUSE_BLOCK_INDEX			0x0008
#define REG_EFUSE_CFG0	                0x000C
#define REG_EFUSE_DC_CFG0               0x0010
#define REG_EFUSE_DC_CFG1               0x0014
#define REG_EFUSE_RD_CFG				0x0018
#define REG_EFUSE_OP_CFG				0x001C
#define REG_EFUSE_KAR					0x0020
#define REG_EFUSE_MOD_CTRL				0x0024
#define REG_EFUSE_STATUS                0x0028
#define REG_EFUSE_IP_REVISION           0x002C
#define REG_EFUSE_SECURE_STATUS         0x0030
#define REG_EFUSE_CMD_HI32				0x0034
#define REG_EFUSE_CMD_LO32				0x0038
#define REG_EFUSE_LDO_RAISE_TIME        0x003C
#define REG_EFUSE_LDO_DROP_TIME         0x0040

/* bits definitions for register REG_EFUSE_CFG0 */
#define BIT_PGM_EN                    (BIT(31))
#define BIT_EFS_VDD_ON                (BIT(30))
#define BIT_EFS_VCCRAM_ON             (BIT(29))
#define BIT_EFS_VCCFHV_ON             (BIT(28))

/* bits definitions for register REG_EFUSE_DC_CFG1 */
#define BIT_DC_NUM(_x_)		((_x_) << 0 & (BIT(0) | BIT(1) | BIT(2) | \
	BIT(3)))
#define BIT_DC_LEN(_x_)		((_x_) << 0 & (BIT(5) | BIT(6) | BIT(7) | \
	BIT(8) | BIT(9)))

/* bits definitions for register REG_EFUSE_RD_CFG */
#define BIT_ISENSELV(_x_)     ((_x_) << 0 & (BIT(0) | BIT(1) | BIT(2) | \
	BIT(3) | BIT(4) | BIT(5)))
#define BIT_IFAVOR_LEN(_x_)     ((_x_) << 0 & (BIT(6) | BIT(7) | BIT(8)))

/* bits definitions for register REG_EFUSE_OP_CFG */
#define BIT_OP_MODE(_x_)     ((_x_) << 0 & (BIT(0) | BIT(1) | BIT(2) | \
	BIT(3) | BIT(4)))

/* bits definitions for register REG_EFUSE_MOD_CTRL */
#define BIT_MARGIN_EN                 (BIT(3))
#define BIT_STANDBY_EN                (BIT(2))
#define BIT_CMD_DERECT_INPUT_SEL      (BIT(1))
#define BIT_CMD_INPUT                 (BIT(0))

/* bits definitions for register REG_EFUSE_STATUS */
#define BIT_PGM_BUSY                (BIT(0))
#define BIT_READ_BUSY               (BIT(1))
#define BIT_STANDY_BUSY             (BIT(2))
#define BIT_DATA_READY              (BIT(4))
#define BIT_DC_WRITE_BUSY           (BIT(20))
#define BIT_DC_READ_BUSY            (BIT(21))
#define BIT_CTL_BUSY                (BIT(24))

#define ISENSELV 0x0
#define IFAVOR 0x0
#define SHIFT_ISENSELV 0
#define READ_MODE 0x0015
#define PROG_MODE 0x0011
#define KEY_ACCESS_MODE 0x0018
#define KEY_ACCESS_VALUE             (0x55aa55)

#define DC_READ_MODE 0x0016
#define DC_WRITE_MODE 0x0013
#define HWSPINLOCK_TIMEOUT		(5000)
#define BIT_LOCK_EFUSE                (BIT(31))
#define BANK_INDEX(val)          (val << 7)

static unsigned long efuse_hwlock_flag;

enum ap_iefuse_ip_type {
	UNKNOWN_DMA,
	AP_IEFUSE_R1P0,
};

struct sprd_ap_efuse {
	enum ap_iefuse_ip_type iefuse_type;
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

static inline void ap_efuse_reg_write(u32 or_val, unsigned long reg)
{
	writel_relaxed(or_val, (void __iomem *)(reg + ap_efuse->base));
}

static inline void ap_efuse_bit_write(unsigned long reg, u32 bits,
				      u32 clear_msk)
{
	writel_relaxed(((readl_relaxed((void __iomem *)(reg + ap_efuse->base)) &
			 ~clear_msk) | bits),
		       ((void __iomem *)(reg + ap_efuse->base)));
}

static void sprd_ap_efuse_reset(void)
{
	unsigned int mask;

	mask = BIT_AON_APB_EFUSE_SOFT_RST;
	regmap_update_bits(ap_efuse->aon_apb_glb, REG_AON_APB_APB_RST0,
			   mask, mask);
	udelay(10);
	regmap_update_bits(ap_efuse->aon_apb_glb, REG_AON_APB_APB_RST0,
			   mask, ~mask);
}

static void ap_efuse_enable(void)
{
	clk_prepare_enable(ap_efuse->clk);
}

static void ap_efuse_disable(void)
{
	clk_disable_unprepare(ap_efuse->clk);
}

static void ap_efuse_margin_mode(void)
{
	ap_efuse_bit_write(REG_EFUSE_MOD_CTRL, 0, BIT_MARGIN_EN);
}

static int ap_efuse_wait_clear(u32 bits)
{
	int ret = 0, cnt = 50;

	/* wait for 10us for efuse status bit take effect */
	udelay(10);
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

static int ap_efuse_read(int bank_index, int blk_index, u32 *data)
{
	u32 val = 0;
	u32 cfg_temp = 0;
	int ret = 0;

	ap_efuse_reg_write(READ_MODE, REG_EFUSE_OP_CFG);
	cfg_temp = (ISENSELV << SHIFT_ISENSELV) | IFAVOR;
	ap_efuse_reg_write(cfg_temp, REG_EFUSE_RD_CFG);
	ap_efuse_reg_write(BANK_INDEX(bank_index) | blk_index, REG_EFUSE_BLOCK_INDEX);
	ap_efuse_bit_write(REG_EFUSE_MOD_CTRL, BIT_CMD_INPUT, 0);

	ret = ap_efuse_wait_clear(BIT_CTL_BUSY);
	if (ret) {
		pr_info("iefuse time out 0x%x\n",
			ap_efuse_reg_read(REG_EFUSE_STATUS));
		*data = 0;
		return ret;
	}
	val = ap_efuse_reg_read(REG_EFUSE_DATA_RD);
	*data = val;
	return ret;
}

static int ap_efuse_dc_read(u32 dc_num, u32 dc_len, u32 *data)
{
	u32 val = 0;
	int ret = 0;

	ap_efuse_reg_write(DC_READ_MODE, REG_EFUSE_OP_CFG);
	ap_efuse_bit_write(REG_EFUSE_DC_CFG1, dc_num, 0);
	ap_efuse_bit_write(REG_EFUSE_DC_CFG1, dc_len, 0);
	ap_efuse_bit_write(REG_EFUSE_MOD_CTRL, BIT_CMD_INPUT, 0);

	ret = ap_efuse_wait_clear(BIT_CTL_BUSY);
	if (ret) {
		pr_info("iefuse time out 0x%x\n",
			ap_efuse_reg_read(REG_EFUSE_STATUS));
		*data = 0;
		return ret;
	}
	val = ap_efuse_reg_read(REG_EFUSE_DATA_RD);
	*data = val;
	return ret;
}

static int ap_efuse_dc_write(u32 data, u32 dc_num, u32 dc_len)
{
	int ret = 0;

	ap_efuse_reg_write(DC_WRITE_MODE, REG_EFUSE_OP_CFG);
	ap_efuse_bit_write(REG_EFUSE_DC_CFG1, dc_num, 0);
	ap_efuse_bit_write(REG_EFUSE_DC_CFG1, dc_len, 0);
	ap_efuse_reg_write(data, REG_EFUSE_DC_CFG0);
	ap_efuse_bit_write(REG_EFUSE_MOD_CTRL, BIT_CMD_INPUT, 0);

	ret = ap_efuse_wait_clear(BIT_CTL_BUSY);
	if (ret)
		pr_info("iefuse time out 0x%x\n",
			ap_efuse_reg_read(REG_EFUSE_STATUS));

	return ret;
}

int sprd_ap_efuse_dc_read(u32 dc_num, u32 dc_len, u32 *data)
{
	u32 val = 0;
	int ret = 0;

	pr_info("iefuse dc read number=%d,len=%d\n", dc_len, dc_num);
	ret = ap_efuse_lock();
	if (ret)
		return ret;
	ap_efuse_enable();
	ap_efuse_margin_mode();
	ret = ap_efuse_dc_read(dc_num, dc_len, data);
	ap_efuse_disable();
	ap_efuse_unlock();
	pr_info("iefuse read 0x%08x\n", val);
	return ret;
}
EXPORT_SYMBOL_GPL(sprd_ap_efuse_dc_read);

int sprd_ap_efuse_dc_write(u32 data, u32 dc_len, u32 dc_num)
{
	int ret = 0;

	pr_info("iefuse dc write 0x%x num=%d,len=%d\n", data, dc_len, dc_num);
	ret = ap_efuse_lock();
	if (ret)
		return ret;
	ap_efuse_enable();
	ap_efuse_margin_mode();
	ret = ap_efuse_dc_write(data, dc_len, dc_num);
	ap_efuse_disable();
	ap_efuse_unlock();
	return ret;
}
EXPORT_SYMBOL_GPL(sprd_ap_efuse_dc_write);

u32 sprd_ap_efuse_read(int blk_index)
{
	u32 val, bank1_val;
	int bank_index = 0, bank1_index = 1;
	int ret, bank1_ret;

	ret = ap_efuse_lock();
	if (ret)
		return ret;
	ap_efuse_enable();
	ap_efuse_margin_mode();
	ret = ap_efuse_read(bank_index, blk_index, &val);
	bank1_ret = ap_efuse_read(bank1_index, 0, &bank1_val);
	ap_efuse_disable();
	ap_efuse_unlock();
	pr_info("iefuse read ret %d, bank1_ret %d,index %d, 0x%08x\n",
			ret, bank1_ret, blk_index, val);
	return val;
}
EXPORT_SYMBOL_GPL(sprd_ap_efuse_read);

int sprd_ap_efuse_prog(int blk_index, u32 val)
{
	pr_info("efuse prog %d 0x%08x\n", blk_index, val);
	return 0;
}
EXPORT_SYMBOL_GPL(sprd_ap_efuse_prog);

static struct sprd_otp_operations efuse_ops = {
	.reset = sprd_ap_efuse_reset,
	.read = sprd_ap_efuse_read,
	.prog = sprd_ap_efuse_prog,
};

void sprd_get_chip_uid(char *buf)
{
	u32 block0, block1, block5, block;
	u32 block0_int, block0_bit, block1_int, block1_bit;
	char *p = buf;

	block0 = sprd_ap_efuse_read(0);
	block1 = sprd_ap_efuse_read(1);
	block5 = sprd_ap_efuse_read(5);
	block0_int = (block0 >> 2) & 0xffffff;
	block0_bit = ((block0 & 0x3)<<2);
	block1_int = block1 & 0xfffffff;
	block1_bit = (block1 & 0x30000000) >> 28;
	block = block0_bit | block1_bit;
	block5 &= 0xfff;

	pr_info("uid is %x%x%x%x\n", block5, block0_int, block, block1_int);
	p += sprintf(p, "%03x%06x%01x%07x", block5, block0_int, block,
		block1_int);
}
EXPORT_SYMBOL_GPL(sprd_get_chip_uid);

void sprd_get_chip_sprduid(char *buf)
{
	u32 block62, block63;
	u32 product_flag, lot_number, wafer_number, x_locate, y_locate;
	int x_sign, y_sign;
	char *p = buf;
	char *product_name;

	block62 = sprd_ap_efuse_read(62);
	block63 = sprd_ap_efuse_read(63);
	product_flag = (block62 >> 24) & 0xf;
	lot_number = block62 & 0xffffff;
	wafer_number = (block63 >> 16) & 0x3ff;
	x_sign = (block63 >> 15) & 0x1;
	x_locate = (block63 >> 8) & 0x7f;
	y_sign = (block63 >> 7) & 0x1;
	y_locate = block63 & 0x7f;

	if (product_flag == 1)
		product_name = "BRS";
	else if (product_flag == 2)
		product_name = "BDS";
	else
		product_name = "UNKNOWN";

	p += sprintf(p, "%s%d_%d_%s%d_%s%d\n", product_name, lot_number,
		wafer_number, x_sign == 0 ? "":"-", x_locate,
		y_sign == 0 ? "":"-", y_locate);
}
EXPORT_SYMBOL_GPL(sprd_get_chip_sprduid);

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
	char *p = buf;
	char uid[20];

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
	char uid[30];

	sprd_get_chip_sprduid(uid);
	p += sprintf(p, "%s\n", uid);
	p += sprintf(p, "\n");
	return p - buf;
}
static DEVICE_ATTR_RO(uidval);

static struct attribute *ap_efuse_device[] = {
	&dev_attr_dump.attr,
	&dev_attr_uid.attr,
	&dev_attr_uidval.attr,
	NULL,
};

static struct attribute_group ap_efuse_attr_group = {
	.name = NULL,
	.attrs = ap_efuse_device,
};

static const struct of_device_id ap_efuse_of_match[] = {
	{.compatible = "sprd,ap_iefuse_r1p0", .data = (void *)AP_IEFUSE_R1P0,},
	{}
};

static int sprd_ap_efuse_probe(struct platform_device *pdev)
{
	struct resource *res = NULL;
	struct regmap *efuse_glb = NULL;
	const struct of_device_id *of_id;
	enum ap_iefuse_ip_type iefuse_type;
	void __iomem *efuse_base = NULL;
	int ret = 0;

	struct device_node *np = pdev->dev.of_node;

	if (!np) {
		dev_err(&pdev->dev, "device node not found\n");
		return -ENODEV;
	}
	dev_info(&pdev->dev, "sprd ap iefuse probe start\n");

	of_id = of_match_node(ap_efuse_of_match, pdev->dev.of_node);
	if (!of_id) {
		dev_err(&pdev->dev, "get id of ap iefuse failed!\n");
		return -ENODEV;
	}
	iefuse_type = (enum ap_iefuse_ip_type)of_id->data;

	efuse_glb = syscon_regmap_lookup_by_phandle(np, "sprd,syscon-enable");
	if (IS_ERR(efuse_glb)) {
		dev_err(&pdev->dev, "ap iefuse syscon failed!\n");
		return -ENODEV;
	}

	ap_efuse = devm_kzalloc(&pdev->dev,
				sizeof(struct sprd_ap_efuse), GFP_KERNEL);
	if (!ap_efuse)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	efuse_base = devm_ioremap_resource(&pdev->dev, res);
	if (!efuse_base) {
		dev_err(&pdev->dev, "remap ap iefuse address failed!\n");
		return -ENOMEM;
	}

	ap_efuse->clk = of_clk_get_by_name(pdev->dev.of_node, "enable");
	if (IS_ERR(ap_efuse->clk)) {
		dev_err(&pdev->dev, "ap iefuse not get clk enable\n");
		return PTR_ERR(ap_efuse->clk);
	}

	ap_efuse->hw_lock =
	    of_hwspin_lock_request(pdev->dev.of_node, "ap_efuse");
	if (!ap_efuse->hw_lock) {
		dev_err(&pdev->dev, "ap iefuse not get hardware spinlock.\n");
		ret = -ENXIO;
		goto clk_fail;
	}

	ret = of_property_read_u32(np, "sprd,block-num", &ap_efuse->blk_num);
	if (ret) {
		dev_err(&pdev->dev, "ap iefuse get sprd,block-num err\n");
		goto lock_fail;
	}

	ret =
	    of_property_read_u32(np, "sprd,block-width", &ap_efuse->blk_width);
	if (ret) {
		dev_err(&pdev->dev, "ap iefuse get sprd,block-width err\n");
		goto lock_fail;
	}

	ap_efuse->base = (unsigned long)efuse_base;
	ap_efuse->aon_apb_glb = efuse_glb;
	ap_efuse->iefuse_type = iefuse_type;
	ap_efuse->dev =
	    sprd_otp_register("sprd_otp_ap_efuse", &efuse_ops,
			      ap_efuse->blk_num, ap_efuse->blk_width / 8);
	if (IS_ERR_OR_NULL(ap_efuse->dev)) {
		dev_err(&pdev->dev, "ap iefuse register fail\n");
		ret = PTR_ERR(ap_efuse->dev);
		goto lock_fail;
	}

	ret = sysfs_create_group(&ap_efuse->dev->kobj, &ap_efuse_attr_group);
	if (ret) {
		dev_err(&pdev->dev,
			"failed to create sysfs device attributes\n");
		goto lock_fail;
	}
	dev_info(&pdev->dev, "sprd iefuse probe end\n");
	return ret;

lock_fail:
	hwspin_lock_free(ap_efuse->hw_lock);
clk_fail:
	clk_unprepare(ap_efuse->clk);

	return ret;
}

static int sprd_ap_efuse_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &ap_efuse_attr_group);
	return 0;
}

static struct platform_driver sprd_ap_efuse_driver = {
	.probe = sprd_ap_efuse_probe,
	.remove = sprd_ap_efuse_remove,
	.driver = {
		   .name = "ap_iefuse",
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
MODULE_DESCRIPTION("Spreadtrum Otp Ap Iefuse Driver");
MODULE_LICENSE("GPL");

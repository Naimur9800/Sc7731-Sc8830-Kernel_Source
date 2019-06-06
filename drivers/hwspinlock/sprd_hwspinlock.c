/*
 * sprd hardware spinlock driver
 * Copyright (C) 2015 Spreadtrum  - http://www.spreadtrum.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/hwspinlock.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/radix-tree.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include "hwspinlock_internal.h"

/*
 * HWLOCK_ADI				(0)
 * HWLOCK_GLB				(1)
 * HWLOCK_AGPIO				(2)
 * HWLOCK_AEIC				(3)
 * HWLOCK_ADC				(4)
 * HWLOCK_EFUSE				(8)
 */


/* hwspinlock registers definition */
#define HWSPINLOCK_RECCTRL			(0x4)
/* bits definition of RECCTRL reg */
#define	HWSPINLOCK_ID				(0x0)
#define	HWSPINLOCK_USER_BITS		(0x1)

#define HWSPINLOCK_TTLSTS			(0x8)
#define HWSPINLOCK_FLAG0			(0x10)
#define HWSPINLOCK_FLAG1			(0x14)
#define HWSPINLOCK_FLAG2			(0x18)
#define HWSPINLOCK_FLAG3			(0x1c)
#define HWSPINLOCK_MASTERID(_X_)	(0x80 + 0x4*(_X_))
#define HWSPINLOCK_TOKEN_V0(_X_)	(0x80 + 0x4*(_X_))
#define HWSPINLOCK_TOKEN_V1(_X_)	(0x800 + 0x4*(_X_))
/* untoken lock value */
#define HWSPINLOCK_NOTTAKEN_V1		(0x55aa10c5)
#define HWSPINLOCK_NOTTAKEN_V0		(0x524c534c)
#define HWSPINLOCK_VERID			(0xFFC)

/* hwspinlock number */
#define HWLOCK_ID_TOTAL_NUMS		(32)
/* processor specific write lock id */
#define HWSPINLOCK_WRITE_KEY		(0x1)

#ifdef CONFIG_HWSPINLOCK_SPRD_WA
#define HW_SPINLOCK_WA_ID_CHECK	5
static u32 ap_userid[] = {3};
#endif

struct sprd_lock_dev {
	unsigned int version_id;
	void __iomem *base;
	unsigned int cnt;
	struct hwspinlock_device bank;
	struct clk *clk;
	unsigned int record_id;
};

unsigned char local_locks_status[HWLOCK_ID_TOTAL_NUMS];
unsigned int local_locks_master_id[HWLOCK_ID_TOTAL_NUMS];

static const struct of_device_id sprd_lock_of_match[] = {
	{ .compatible = "sprd,hwspinlock-r2p0",},
	{ .compatible = "sprd,hwspinlock-r3p0",},
	{ /* sentinel */ }
};

static struct sprd_lock_dev *
sprd_lock_to_dev(struct hwspinlock *lock)
{
	struct hwspinlock_device *hwbank;
	int lock_id;

	if (!lock)
		return NULL;

	lock_id = hwlock_to_id(lock);
	hwbank = container_of(lock, struct hwspinlock_device, lock[lock_id]);
	if (hwbank)
		return container_of(hwbank, struct sprd_lock_dev, bank);
	else
		return NULL;
}

/* set the record reg */
static void
sprd_set_record(struct sprd_lock_dev *lock, unsigned int record_type)
{
	writel_relaxed(record_type, lock->base + HWSPINLOCK_RECCTRL);
}

/* enable the hardware spinlock */
static int sprd_lock_enable(struct sprd_lock_dev *lock)
{
	clk_prepare_enable(lock->clk);

	return 0;
}

/* reset the hardware spinlock */
static inline void sprd_reset_lock(struct sprd_lock_dev *lock)
{

}

/* get the hardware spinlock status */
static inline unsigned int
sprd_get_lock_sts(struct sprd_lock_dev *lock)
{
	unsigned int status;

	status = readl_relaxed(lock->base + HWSPINLOCK_TTLSTS);
	return status;
}

/* get the hardware spinlock master id */
static inline unsigned int
sprd_lock_master_id(struct sprd_lock_dev *lock,
					unsigned int lock_id)
{
	unsigned int master_id;

	if (lock_id >= HWLOCK_ID_TOTAL_NUMS) {
		pr_err("%s: lock id is out of the range!\n", __func__);
		return -ENXIO;
	}

	master_id = readl_relaxed(lock->base + HWSPINLOCK_MASTERID(lock_id));
	local_locks_master_id[lock_id] = master_id;
	return master_id;
}

/* get the hardware spinlock version id */
static inline unsigned int
sprd_get_lock_verid(struct sprd_lock_dev *lock)
{
	unsigned int version_id;

	version_id = readl_relaxed(lock->base + HWSPINLOCK_VERID);
	return version_id;
}

/* print the hardware spinlock debug information */
static int __maybe_unused sprd_dump_lock_info(struct hwspinlock *lock)
{
	struct sprd_lock_dev *sprd_lock;
	unsigned int lock_status = 0;
	unsigned int master_id = 0;

	sprd_lock = sprd_lock_to_dev(lock);
	if (!sprd_lock) {
		pr_err("%s: can't find hardware spinlock device!\n", __func__);
		return -ENODEV;
	}

	lock_status = sprd_get_lock_sts(sprd_lock);

	if (sprd_lock->record_id)
		master_id = sprd_lock_master_id(sprd_lock, hwlock_to_id(lock));

	pr_info("Hardware spinlock status = [0x%x], and the lock [%d]\n"
			"master id is [%d]\n", lock_status,
			hwlock_to_id(lock), master_id);
	return 0;
}

/* check hardware spinlock version id */
inline int sprd_check_lock_vid(unsigned int vid)
{
	/* for sharkl/sharkl64/whale project */
	if (vid >= 0x100)
		return 1;

	pr_err("Warning: hardware spinlock version id is 0x%x!\n", vid);
	return 0;
}

/* record the hardware spinlock lock status */
int sprd_record_lock_sts(unsigned int lock_id, unsigned int sts)
{
	if (lock_id >= HWLOCK_ID_TOTAL_NUMS) {
		pr_err("Hardware spinlock id is out of the range!\n");
		return -ENXIO;
	}

	local_locks_status[lock_id] = sts;
	return 0;
}

#ifdef CONFIG_HWSPINLOCK_SPRD_WA
static bool sprd_hw_id_is_self(u32 id)
{
	int cnt = ARRAY_SIZE(ap_userid);
	int i = 0;

	for (i = 0; i < cnt; i++)
		if (ap_userid[i] == id)
			return true;
	return false;
}

static int sprd_wait_get_lock(struct hwspinlock *lock)
{
	void __iomem *addr = lock->priv;
	int i = 0;
	u32 id;
	struct sprd_lock_dev *sprd_lock;

	sprd_lock = sprd_lock_to_dev(lock);
	while (1) {
		readl_relaxed(addr);
		id = sprd_lock_master_id(sprd_lock, hwlock_to_id(lock));
		for (i = 0; i < HW_SPINLOCK_WA_ID_CHECK; i++)
			if (unlikely(id != sprd_lock_master_id(sprd_lock,
				hwlock_to_id(lock))))
				break;
		if (likely(i == HW_SPINLOCK_WA_ID_CHECK &&
			sprd_hw_id_is_self(id)))
			return 0;
	}
}

/* try to lock the hardware spinlock  for workaround*/
static int sprd_lock_trylock_wa(struct hwspinlock *lock)
{
	if (sprd_wait_get_lock(lock))
		return 0;
	sprd_record_lock_sts(hwlock_to_id(lock), 1);

	return 1;
}
#endif

/* try to lock the hardware spinlock */
static int sprd_lock_trylock(struct hwspinlock *lock)
{
	void __iomem *addr = lock->priv;
	struct sprd_lock_dev *sprd_lock;
	unsigned int key;

	sprd_lock = sprd_lock_to_dev(lock);
	if (!sprd_lock) {
		pr_err("Trylock: can't find hardware spinlock device!\n");
		return -ENODEV;
	}

	if (sprd_check_lock_vid(sprd_lock->version_id)) {
		if (!readl_relaxed(addr))
			goto __locked;
	} else {
		key = HWSPINLOCK_WRITE_KEY;
		if (!(key ^ HWSPINLOCK_NOTTAKEN_V0))
			BUG_ON(1);
		if (readl_relaxed(addr) == HWSPINLOCK_NOTTAKEN_V0) {
			writel_relaxed(key, addr);
			if (key == readl_relaxed(addr))
				goto __locked;
		}
	}

	pr_err("Hardware spinlock [%d] lock failed!\n", hwlock_to_id(lock));
	sprd_dump_lock_info(lock);
	return 0;

__locked:
	sprd_record_lock_sts(hwlock_to_id(lock), 1);
	return 1;
}

/* unlock the hardware spinlock */
static void sprd_lock_unlock(struct hwspinlock *lock)
{
	void __iomem *lock_addr = lock->priv;
	struct sprd_lock_dev *sprd_lock;
	int unlock_key;

	sprd_lock = sprd_lock_to_dev(lock);
	if (!sprd_lock) {
		pr_err("Unlock: can't find hardware spinlock device!\n");
		return;
	}

	if (sprd_check_lock_vid(sprd_lock->version_id)) {
		unlock_key = HWSPINLOCK_NOTTAKEN_V1;
	} else {
		unlock_key = HWSPINLOCK_NOTTAKEN_V0;
		if (!(readl_relaxed(lock_addr) ^ unlock_key))
			BUG_ON(1);
	}

	writel_relaxed(unlock_key, lock_addr);
	sprd_record_lock_sts(hwlock_to_id(lock), 0);
}

/*
 * relax the  interconnect while spinning on it.
 *
 * The specs recommended that the retry delay time will be
 * just over half of the time that a requester would be
 * expected to hold the lock.
 *
 * The number below is taken from an hardware specs example,
 * obviously it is somewhat arbitrary.
 */
static void sprd_lock_relax(struct hwspinlock *lock)
{
	ndelay(10);
}

static const struct hwspinlock_ops sprd_lock_ops = {
	.trylock = sprd_lock_trylock,
	.unlock = sprd_lock_unlock,
	.relax = sprd_lock_relax,
};

#ifdef CONFIG_HWSPINLOCK_SPRD_WA
static const struct hwspinlock_ops sprd_lock_ops_wa = {
	.trylock = sprd_lock_trylock_wa,
	.unlock = sprd_lock_unlock,
	.relax = sprd_lock_relax,
};
#endif

static int sprd_lock_probe(struct platform_device *pdev)
{
	struct hwspinlock *lock;
	struct sprd_lock_dev *sprd_lock;
	void __iomem *lock_base = NULL;
	struct resource *res = NULL;
	static int lock_exist_cnt;
	const struct of_device_id *lock_of_id;
	const struct hwspinlock_ops *ops;
	int i, ret, lock_id;
	u32 num_locks, base_id;
#ifdef CONFIG_HWSPINLOCK_SPRD_WA
	static u32 verid_reg_addr;
#endif

	lock_of_id = of_match_node(sprd_lock_of_match,
		pdev->dev.of_node);
	if (!lock_of_id) {
		pr_err("Get the spinlock of device id failed!\n");
		return -ENODEV;
	}

	ret = of_alias_get_id(pdev->dev.of_node, "hwspinlock");
	if (IS_ERR_VALUE(ret)) {
		pr_err("Get hardware spinlock alias id failed!\n");
		return ret;
	}

	lock_id = ret;

	ret = of_property_read_u32(pdev->dev.of_node,
					"hwlocks-num", &num_locks);
	if (ret) {
		pr_err("Get hardware spinlock number failed!\n");
		return ret;
	}

	ret = of_property_read_u32(pdev->dev.of_node,
					"hwlocks-base", &base_id);
	if (ret) {
		pr_err("Get hardware spinlock base id failed!\n");
		return ret;
	}

	sprd_lock = devm_kzalloc(&pdev->dev,
					sizeof(struct sprd_lock_dev) +
					num_locks * sizeof(*lock),
					GFP_KERNEL);
	if (!sprd_lock) {
		pr_err("Hardware spinlock device allocates memory failed!\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	lock_base = devm_ioremap_resource(&pdev->dev, res);
	if (!lock_base) {
		pr_warn("Remap the hardware spinlock address failed!\n");
		return -ENOMEM;
	}

	sprd_lock->clk = of_clk_get_by_name(pdev->dev.of_node, "enable");
	if (IS_ERR(sprd_lock->clk)) {
		sprd_lock->clk = NULL;
		pr_err("hwspinlock can't get enable clock fail, switch to fpga mode\n");
	}
	/* initial the sprd_lock_dev structure */
	sprd_lock->cnt = num_locks;
	sprd_lock->base = lock_base;
	sprd_lock->record_id = 0;

	/* initial the hwspinlock register */
	sprd_lock_enable(sprd_lock);

	if (strcmp(lock_of_id->compatible, "sprd,hwspinlock-r2p0")) {
		if (of_property_read_bool(pdev->dev.of_node, "sprd,axi-id"))
			sprd_set_record(sprd_lock, HWSPINLOCK_ID);
		else
			sprd_set_record(sprd_lock, HWSPINLOCK_USER_BITS);
		sprd_lock->record_id = 1;
	}

	/* get the hwspinlock version id */
	sprd_lock->version_id = sprd_get_lock_verid(sprd_lock);

	for (i = 0, lock = &sprd_lock->bank.lock[0];
		 i < num_locks; i++, lock++) {
		if (sprd_check_lock_vid(sprd_lock->version_id))
			lock->priv = sprd_lock->base + HWSPINLOCK_TOKEN_V1(i);
		else
			lock->priv = sprd_lock->base + HWSPINLOCK_TOKEN_V0(i);
	}

	platform_set_drvdata(pdev, sprd_lock);
	/*
	 * runtime PM will make sure the clock of this module is
	 * enabled if at least one lock is requested
	 */
	pm_runtime_enable(&pdev->dev);

	ops = &sprd_lock_ops;
#ifdef CONFIG_HWSPINLOCK_SPRD_WA
	ops = &sprd_lock_ops_wa;
	if (!of_property_read_u32(pdev->dev.of_node, "sprd,ver_id_reg",
		&verid_reg_addr))
		if (verid_reg_addr != 0) {
			lock_base  = devm_ioremap_nocache(&pdev->dev,
				verid_reg_addr, sizeof(verid_reg_addr));
			if (readl_relaxed(lock_base) != 0)
				ops = &sprd_lock_ops;
		}
#endif
	ret = hwspin_lock_register(&sprd_lock->bank, &pdev->dev,
							   ops,
							   base_id,
							   num_locks);
	if (ret) {
		pr_err("Hardware spinlock register failed!\n");
		goto reg_fail;
	}

	lock_exist_cnt += num_locks;
	if (lock_exist_cnt > HWLOCK_ID_TOTAL_NUMS) {
		pr_err("Hardware spinlock number is out of range!\n");
		goto out_of_range;
	}

	pr_info("sprd lock probe ok: name = %s, version id = 0x%x,\n"
			"exist lock number = %d.\n", pdev->name,
			sprd_lock->version_id, lock_exist_cnt);

	return 0;

out_of_range:
	hwspin_lock_unregister(&sprd_lock->bank);
reg_fail:
	pm_runtime_disable(&pdev->dev);
	return ret;
}

static int sprd_lock_remove(struct platform_device *pdev)
{
	struct sprd_lock_dev *sprd_lock = platform_get_drvdata(pdev);
	int ret;

	ret = hwspin_lock_unregister(&sprd_lock->bank);
	if (ret)
		pr_err("Hardware spinlock unregister failed: %d\n", ret);

	pm_runtime_disable(&pdev->dev);
	return ret;
}

static struct platform_driver sprd_lock_driver = {
	.probe = sprd_lock_probe,
	.remove = sprd_lock_remove,
	.driver = {
		.name = "sprd-hwspinlock",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sprd_lock_of_match),
	},
};

static int __init sprd_lock_init(void)
{
	return platform_driver_register(&sprd_lock_driver);
}

static void __exit sprd_lock_exit(void)
{
	platform_driver_unregister(&sprd_lock_driver);
}

postcore_initcall(sprd_lock_init);
module_exit(sprd_lock_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Hardware spinlock driver for Spreadtrum");
MODULE_AUTHOR("Baolin Wang <baolin.wang@spreadtrum.com>");

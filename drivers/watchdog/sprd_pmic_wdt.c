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
#ifdef CONFIG_X86
#include "sprd_pmic_wdt.h"
#else
#include <asm/system_misc.h>
#endif
#include <linux/bitops.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/mfd/sprd/pmic_glb_reg.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/spi/spi-sprd-adi.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include "../base/regmap/internal.h"

#define WDG_INT_EN_BIT			BIT(0)
#define WDG_CNT_EN_BIT			BIT(1)
#define WDG_NEW_VER_EN			BIT(2)
#define WDG_INT_CLEAR_BIT		BIT(0)
#define WDG_RST_CLEAR_BIT		BIT(3)
#define WDG_LD_BUSY_BIT			BIT(4)
#define WDG_RST_EN_BIT			BIT(3)

#define ANA_RST_STATUS			ANA_REG_GLB_POR_RST_MONITOR
#define ANA_AGEN				ANA_REG_GLB_ARM_MODULE_EN
#define ANA_RTC_CLK_EN			ANA_REG_GLB_RTC_CLK_EN
#define AGEN_WDG_EN				BIT_ANA_WDG_EN
#define AGEN_RTC_WDG_EN			BIT_RTC_WDG_EN

#define WDG_CLK						32768
#define WDG_UNLOCK_KEY				0xE551
#define ANA_WDG_LOAD_TIMEOUT_NUM	(10000)

/*
* Definition of ANA_RST_STATUS(ANA_REG_GLB_POR_RST_MONITOR) register
* bit[0..7]: Reboot mode
* bit[9]: SYSDUMP enable/disable flag
*/

#define HWRST_STATUS_RECOVERY		(0x0020)
#define HWRST_STATUS_NORMAL		(0x0040)
#define HWRST_STATUS_ALARM		(0x0050)
#define HWRST_STATUS_SLEEP		(0x0060)
#define HWRST_STATUS_FASTBOOT		(0x0030)
#define HWRST_STATUS_SPECIAL		(0x0070)
#define HWRST_STATUS_PANIC		(0x0080)
#define HWRST_STATUS_CFTREBOOT		(0x0090)
#define HWRST_STATUS_AUTODLOADER	(0x00a0)
#define HWRST_STATUS_IQMODE		(0x00b0)
#define HWRST_STATUS_SPRDISK		(0x00c0)
#define HWRST_STATUS_SYSDUMP		(0x200)

#define HWRST_STATUS_SECURITY		(0x0002)

static struct spi_device *reboot_device;
static unsigned long sprd_wdt_base;
unsigned int reboot_mode_flag;
spinlock_t reboot_flag_lock;
int sysdump_status; /* used by sysdump.c */

#define WDG_LOAD_LOW		(sprd_wdt_base + 0x00)
#define WDG_LOAD_HIGH		(sprd_wdt_base + 0x04)
#define WDG_CTRL			(sprd_wdt_base + 0x08)
#define WDG_INT_CLR			(sprd_wdt_base + 0x0c)
#define WDG_INT_RAW			(sprd_wdt_base + 0x10)
#define WDG_INT_MSK			(sprd_wdt_base + 0x14)
#define WDG_CNT_LOW			(sprd_wdt_base + 0x18)
#define WDG_CNT_HIGH		(sprd_wdt_base + 0x1c)
#define WDG_LOCK			(sprd_wdt_base + 0x20)

void set_sysdump_enable(int on);
#ifdef CONFIG_SPRD_DMC_DRV
extern void sprd_ddr_force_light_sleep(void);
#endif
void sprd_set_reboot_mode(const char *cmd)
{
	pr_info("sprd_set_reboot_mode:cmd=%s\n", cmd);

	spin_lock(&reboot_flag_lock);
	if (cmd && !(strncmp(cmd, "recovery", 8)))
		reboot_mode_flag |= HWRST_STATUS_RECOVERY;
	else if (cmd && !strncmp(cmd, "alarm", 5))
		reboot_mode_flag |= HWRST_STATUS_ALARM;
	else if (cmd && !strncmp(cmd, "fastsleep", 9))
		reboot_mode_flag |= HWRST_STATUS_SLEEP;
	else if (cmd && !strncmp(cmd, "bootloader", 10))
		reboot_mode_flag |= HWRST_STATUS_FASTBOOT;
	else if (cmd && !strncmp(cmd, "panic", 5))
		reboot_mode_flag |= HWRST_STATUS_PANIC;
	else if (cmd && !strncmp(cmd, "special", 7))
		reboot_mode_flag |= HWRST_STATUS_SPECIAL;
	else if (cmd && !strncmp(cmd, "cftreboot", 9))
		reboot_mode_flag |= HWRST_STATUS_CFTREBOOT;
	else if (cmd && !strncmp(cmd, "autodloader", 11))
		reboot_mode_flag |= HWRST_STATUS_AUTODLOADER;
	else if (cmd && !strncmp(cmd, "iqmode", 6))
		reboot_mode_flag |= HWRST_STATUS_IQMODE;
	else if (cmd && !strncmp(cmd, "sprdisk", 7))
		reboot_mode_flag |= HWRST_STATUS_SPRDISK;
	else if (cmd && !strncmp(cmd, "dumpenable", 10)) {
		pr_emerg("sprd_set_reboot_mode: enable sysdump!\n");
		reboot_mode_flag |= HWRST_STATUS_SYSDUMP;
	} else if (cmd && !strncmp(cmd, "dumpdisable", 11)) {
		pr_emerg("sprd_set_reboot_mode: disable sysdump!\n");
		reboot_mode_flag &= ~(HWRST_STATUS_SYSDUMP);
	} else if (cmd && !strncmp(cmd, "tospanic", 8)) {
		reboot_mode_flag |= HWRST_STATUS_SECURITY;
	} else
		reboot_mode_flag = HWRST_STATUS_NORMAL;

	spin_unlock(&reboot_flag_lock);
}

void set_sysdump_enable(int on)
{
	unsigned int val = 0;

	val = sprd_adi_read(reboot_device, ANA_RST_STATUS);
	pr_emerg("set_sysdump_enable: get rst mode  value is = %x\n", val);

	if (on) {
		pr_emerg("sprd_set_reboot_mode: enable sysdump!\n");
		val |= HWRST_STATUS_SYSDUMP;
		sprd_adi_raw_write(reboot_device, ANA_RST_STATUS, val);
	} else {
		pr_emerg("sprd_set_reboot_mode: disable sysdump!\n");
		val &= ~(HWRST_STATUS_SYSDUMP);
		sprd_adi_raw_write(reboot_device, ANA_RST_STATUS, val);
	}
}

static inline void sprd_load_time_vale(unsigned int value)
{
	sprd_adi_raw_write(reboot_device, WDG_LOAD_HIGH,
			   (uint16_t) (((value) >> 16) & 0xffff));
	sprd_adi_raw_write(reboot_device, WDG_LOAD_LOW,
			   (uint16_t) ((value) & 0xffff));
}

static void sprd_turnon_watchdog(unsigned int ms)
{
	uint32_t cnt = (ms * 1000) / WDG_CLK;

	/*enable interface clk */
	sprd_adi_set(reboot_device, ANA_AGEN, AGEN_WDG_EN);

	/*enable work clk */
	sprd_adi_set(reboot_device, ANA_RTC_CLK_EN, AGEN_RTC_WDG_EN);
	sprd_adi_raw_write(reboot_device, WDG_LOCK, WDG_UNLOCK_KEY);
	sprd_adi_set(reboot_device, WDG_CTRL, WDG_NEW_VER_EN);
	sprd_load_time_vale(cnt);

	sprd_adi_set(reboot_device, WDG_CTRL,
		     (WDG_CNT_EN_BIT | WDG_RST_EN_BIT));
	sprd_adi_raw_write(reboot_device, WDG_LOCK,
			   (uint16_t) (~WDG_UNLOCK_KEY));
}

static void arch_reset(char mode, const char *cmd)
{
	sprd_adi_raw_write(reboot_device, ANA_RST_STATUS, reboot_mode_flag);
	if (mode == REBOOT_WDT)
		sprd_turnon_watchdog(400);
	else
		sprd_turnon_watchdog(50);
#ifdef CONFIG_SPRD_DMC_DRV
	sprd_ddr_force_light_sleep();
#endif
}

static void sprd_restart_handle(enum reboot_mode reboot_mode, const char *cmd)
{
	arch_reset(reboot_mode, cmd);

	mdelay(1000);

	pr_info("reboot failed!\n");

	while (1)
		;
}

static void sprd_restart_prepare(const char *cmd)
{
	sprd_set_reboot_mode(cmd);
}

static struct of_device_id sprd_reboot_of_match[] = {
	{.compatible = "sprd,sc2723t-wdt",},
	{.compatible = "sprd,sc2731-wdt",},
	{.compatible = "sprd,sc2721-wdt",},
	{.compatible = "sprd,sc2720-wdt",},
	{}
};

static int sprd_reboot_probe(struct platform_device *pdev)
{
	int ret = 0;
	u32 value = 0;
	struct device_node *node = pdev->dev.of_node;
	struct regmap *sprd_reboot;

	sprd_reboot = dev_get_regmap(pdev->dev.parent, NULL);
	if (!sprd_reboot) {
		dev_err(&pdev->dev, "sprd reboot probe failed!\n");
		return -EINVAL;
	}

	reboot_device = to_spi_device(sprd_reboot->dev);
	if (!reboot_device) {
		dev_err(&pdev->dev, "sprd reboot device probe failed!\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(node, "reg", &value);
	if (ret) {
		dev_err(&pdev->dev, "sprd reboot probe failed!\n");
		return -EINVAL;
	}

	sprd_wdt_base = (unsigned long)value;
#ifdef CONFIG_X86
	x86_pm_restart_prepare = sprd_restart_prepare;
	x86_pm_restart = sprd_restart_handle;
#else
	arm_pm_restart_prepare = sprd_restart_prepare;
	arm_pm_restart = sprd_restart_handle;
#endif
	dev_info(&pdev->dev, "PMIC Watchdog probe OK!\n");
	spin_lock_init(&reboot_flag_lock);
	reboot_mode_flag = 0;

#if defined(CONFIG_SPRD_DEBUG) && defined(CONFIG_SPRD_SYSDUMP)
	pr_emerg("userdebug enable sysdump in default !!!\n");
	sprd_set_reboot_mode("dumpenable");
	set_sysdump_enable(1);
	sysdump_status = 1;
#endif
	return ret;
}

static struct platform_driver sprd_reboot_driver = {
	.probe = sprd_reboot_probe,
	.driver = {
		.name = "sprd-pmic-wdt",
		.of_match_table = sprd_reboot_of_match,
	},
};

static int __init sprd_reboot_init(void)
{
	return platform_driver_register(&sprd_reboot_driver);
}

device_initcall(sprd_reboot_init);
MODULE_DESCRIPTION("SPREADTRUM PMIC Watchdog Driver");
MODULE_LICENSE("GPL v2");

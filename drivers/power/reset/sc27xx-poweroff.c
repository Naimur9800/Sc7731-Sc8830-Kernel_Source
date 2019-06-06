/*
 * Copyright (C) 2016 Spreadtrum Communications Inc.
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
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/mfd/sprd/pmic_glb_reg.h>
#include <linux/spi/spi-sprd-adi.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include "../../base/regmap/internal.h"

static struct regmap *sc27xx_power_off;
static struct spi_device *poweroff_device;


static void sc27xx_poweroff_prepare(void)
{
	int current_pid = 0;
	struct task_struct *g;

	rcu_read_lock();
	for_each_process(g) {
		if (!strcmp(g->comm, "spi5")) {
			current_pid = g->pid;
			break;
		}
	}
	rcu_read_unlock();
	sched_setaffinity(current_pid, cpumask_of(reboot_cpu));
}

static void sc27xx_poweroff(void)
{
	unsigned int reg_val;
	sprd_adi_raw_write(poweroff_device, ANA_REG_GLB_PWR_WR_PROT_VALUE,
			   BITS_PWR_WR_PROT_VALUE(0x6e7F));
	do {
		reg_val = sprd_adi_read(poweroff_device, ANA_REG_GLB_PWR_WR_PROT_VALUE);
		cpu_relax();
	} while ((reg_val & BIT_PWR_WR_PROT) == 0);

	sprd_adi_clr(poweroff_device, ANA_REG_GLB_SLP_CTRL, BIT_LDO_XTL_EN);
	sprd_adi_raw_write(poweroff_device, ANA_REG_GLB_POWER_PD_HW,
			   BIT_PWR_OFF_SEQ_EN);
}

static const struct of_device_id sc27xx_power_off_of_match_table[] = {
	{.compatible = "sprd,sc2731-poweroff"},
	{.compatible = "sprd,sc2721-poweroff"},
	{.compatible = "sprd,sc2720-poweroff"},
	{},
};

MODULE_DEVICE_TABLE(of, sprd_power_off_of_match_table);

static int sc27xx_power_off_probe(struct platform_device *pdev)
{
	sc27xx_power_off = dev_get_regmap(pdev->dev.parent, NULL);
	if (!sc27xx_power_off) {
		dev_err(&pdev->dev, "sc27xx power off probe failed!\n");
		return -EINVAL;
	}

	poweroff_device = to_spi_device(sc27xx_power_off->dev);
	if (!poweroff_device) {
		dev_err(&pdev->dev, "poweroff_device probe failed!\n");
		return -EINVAL;
	}

	pm_power_off_prepare = sc27xx_poweroff_prepare;
	pm_power_off = sc27xx_poweroff;

	dev_info(&pdev->dev, "sc27xx power off probe OK!\n");

	return 0;
}

static struct platform_driver sc27xx_power_off_driver = {
	.probe = sc27xx_power_off_probe,
	.driver = {
		   .name = "sprd_power_off",
		   .of_match_table =
		   of_match_ptr(sc27xx_power_off_of_match_table),
		   },
};

static int __init sprd_power_off_init(void)
{
	return platform_driver_register(&sc27xx_power_off_driver);
}

device_initcall(sprd_power_off_init);
MODULE_DESCRIPTION("PMIC SC27xx Power off Driver");
MODULE_LICENSE("GPL v2");

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
#include <asm/system_misc.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/mfd/sprd/pmic_glb_reg.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/pm.h>
#include <linux/regmap.h>

static struct regmap *sc2723_power_off;

static void sc2723_poweroff_prepare(void)
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

static void sc2723_poweroff(void)
{
	unsigned int reg_val;

	regmap_write(sc2723_power_off, ANA_REG_GLB_LDO_PD_CTRL, 0x1fff);
	regmap_write(sc2723_power_off, ANA_REG_GLB_PWR_WR_PROT_VALUE,
			   BITS_PWR_WR_PROT_VALUE(0x6e7f));

	do {
		regmap_read(sc2723_power_off, ANA_REG_GLB_PWR_WR_PROT_VALUE,
			    &reg_val);
		cpu_relax();
	} while ((reg_val & BIT_PWR_WR_PROT) == 0);

	regmap_write(sc2723_power_off, ANA_REG_GLB_LDO_PD_CTRL, 0xfff);
	regmap_write(sc2723_power_off, ANA_REG_GLB_LDO_DCDC_PD, 0x7fff);
}

static const struct of_device_id sc2723_power_off_of_match_table[] = {
	{.compatible = "sprd,sc2723-poweroff"},
	{},
};

MODULE_DEVICE_TABLE(of, sprd_power_off_of_match_table);

static int sc2723_power_off_probe(struct platform_device *pdev)
{
	sc2723_power_off = dev_get_regmap(pdev->dev.parent, NULL);
	if (!sc2723_power_off) {
		dev_err(&pdev->dev, "sc2723 power off probe failed!\n");
		return -EINVAL;
	}
	pm_power_off_prepare = sc2723_poweroff_prepare;
	pm_power_off = sc2723_poweroff;

	dev_info(&pdev->dev, "sc2723 power off probe OK!\n");

	return 0;
}

static struct platform_driver sc2723_power_off_driver = {
	.probe = sc2723_power_off_probe,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "sprd_power_off",
		   .of_match_table =
		   of_match_ptr(sc2723_power_off_of_match_table),
		   },
};

static int __init sprd_power_off_init(void)
{
	return platform_driver_register(&sc2723_power_off_driver);
}

device_initcall(sprd_power_off_init);
MODULE_DESCRIPTION("PMIC SC2723 Power off Driver");
MODULE_LICENSE("GPL v2");

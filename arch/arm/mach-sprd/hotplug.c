/* arch/arm/mach-sprd/hotplug.c
 *
 *  Copyright (C) 2015 SPREADTRUM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/smp.h>
#include <linux/sched.h>
#include <asm/smp_plat.h>
#include <linux/sprd_arm_pmu.h>


int sci_shark_enter_lowpower(void);


static int powerdown_cpus(int cpu)
{
	u32 poweron, val, i = 0;
	u32 pwon_reg_val = 0;

	struct device_node *np = of_get_cpu_node(cpu, NULL);
	struct regmap *cpu_syscon_apahb;
	struct regmap *cpu_syscon_pmuapb;

	cpu_syscon_apahb = syscon_regmap_lookup_by_phandle(np,
			"sprd,sys-ap-ahb");
	cpu_syscon_pmuapb = syscon_regmap_lookup_by_phandle(np,
			"sprd,sys-pmu-apb");

	if (IS_ERR(cpu_syscon_apahb) ||
		IS_ERR(cpu_syscon_pmuapb)) {
		pr_err("%s:failed to find cpu,syscon\n", __func__);
		return -EINVAL;
	}


	if (cpu == 1)
		poweron = REG_PMU_APB_PD_CA7_C1_CFG;
	else if (cpu == 2)
		poweron = REG_PMU_APB_PD_CA7_C2_CFG;
	else if (cpu == 3)
		poweron = REG_PMU_APB_PD_CA7_C3_CFG;
	else
		return -EINVAL;

	regmap_read(cpu_syscon_pmuapb, poweron, &val);

	pwon_reg_val = val | BIT_PMU_APB_PD_CA7_C3_FORCE_SHUTDOWN;
	pwon_reg_val = pwon_reg_val & ~(BIT_PMU_APB_PD_CA7_C3_AUTO_SHUTDOWN_EN);

	regmap_write(cpu_syscon_pmuapb, poweron, pwon_reg_val);

	/*
	* According to ASIC suggestion,
	* repeat 20 times for waiting cpu's SHUTDOWN status
	*/

	while (i < 20) {
		regmap_read(cpu_syscon_pmuapb,
			REG_PMU_APB_PWR_STATUS0_DBG, &val);

		/* check power down */
		if (((val >> (4 * (cpu_logical_map(cpu) + 1))) & 0x0f) == 0x07)
			break;

		/* delay 60us according to HW's requirement */
		udelay(60);
		i++;
	}
	pr_info("powerdown_cpus i=%d !!\n", i);
	BUG_ON(i >= 20);
	udelay(60);

	return 0;
}


int sprd_cpu_kill(unsigned int cpu)
{
	int i = 0;
	int val = 0;
	struct device_node *np = of_get_cpu_node(cpu, NULL);
	struct regmap *cpu_syscon_apahb;

	cpu_syscon_apahb = syscon_regmap_lookup_by_phandle(np,
			"sprd,sys-ap-ahb");
	if (IS_ERR(cpu_syscon_apahb)) {
		pr_err("%s:failed to find cpu,syscon\n", __func__);
		return 0;
	}

	pr_debug("!! %d  platform_cpu_kill %d !!\n", smp_processor_id(), cpu);


	/*
	* According to ASIC suggestion,
	* repeat 20 times for waiting cpu's WFI status
	*/
	while (i < 20) {
		regmap_read(cpu_syscon_apahb,
			REG_AP_AHB_CA7_STANDBY_STATUS, &val);
		if (val & (1 << cpu_logical_map(cpu))) {
			/*
			 * As the pmu irq of one core is still high level when
			 * this core was unpluged, we should disable the pmu
			 * irq of this core here. Also, in order to maintain
			 * consistency, we should enable this pmu irq when core
			 * was pluged in.
			 */
			sprd_pmu_disable_irq(cpu);
			powerdown_cpus(cpu);
			break;
		}
		/* delay 100us according to HW's requirement */
		udelay(100);
		i++;
	}
	pr_debug("platform_cpu_kill finished i=%d !!\n", i);



	return (i >= 20 ? 0 : 1);
}

void  sprd_cpu_die(unsigned int cpu)
{

	sci_shark_enter_lowpower();
	panic("shouldn't be here\n");
}

int sprd_cpu_disable(unsigned int cpu)
{
	/*
	* we don't allow CPU 0 to be shutdown (it is still too special
	* e.g. clock tick interrupts)
	*/
	return cpu == 0 ? -EPERM : 0;
}

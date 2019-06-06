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

#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/io.h>
#include "pm_debug.h"
#include <asm/irqflags.h>

static void sprd_pm_standby(void)
{
	cpu_do_idle();
}

static int sprd_pm_deepsleep(suspend_state_t state)
{
	int ret_val = 0;
	unsigned long flags;
	unsigned int cpu = smp_processor_id();

	/* add for debug & statisic*/
	clr_sleep_mode();
	time_statisic_begin();

	/*
	* when we get here, only boot cpu still alive
	*/
	if (cpu) {
		__WARN();
		goto enter_exit;
	}
	pr_err("cpu%d, enter %s\n", cpu, __func__);
	while (1) {
		local_fiq_disable();
		local_irq_save(flags);
		WARN_ONCE(!irqs_disabled(),
			"#####: Interrupts enabled in pm_enter()!\n");
		sprd_cpu_deep_sleep(cpu);
		if (!sprd_irq_pending())
			pr_emerg("empty interrupt detected\n");

		irq_wakeup_set();
		local_irq_restore(flags);
		local_fiq_enable();
		break;
	} /*end while*/
	time_statisic_end();

enter_exit:
	return ret_val;
}


static int sprd_pm_enter(suspend_state_t state)
{
	int rval = 0;

	switch (state) {
	case PM_SUSPEND_STANDBY:
		sprd_pm_standby();
		break;
	case PM_SUSPEND_MEM:
		rval = sprd_pm_deepsleep(state);
		break;
	default:
		break;
	}

	return rval;
}

static int sprd_pm_valid(suspend_state_t state)
{
	pr_debug("pm_valid: %d\n", state);
	switch (state) {
	case PM_SUSPEND_ON:
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		return 1;
	default:
		return 0;
	}
}

static int sprd_pm_prepare(void)
{
	pr_debug("enter %s\n", __func__);
	return 0;
}

static void sprd_pm_finish(void)
{
	pr_debug("enter %s\n", __func__);
	print_statisic();
}

const struct platform_suspend_ops sprd_pm_ops = {
	.valid		= sprd_pm_valid,
	.enter		= sprd_pm_enter,
	.prepare	= sprd_pm_prepare,
	.prepare_late	= NULL,
	.finish		= sprd_pm_finish,
};

static int __init sprd_pm_init(void)
{
	sc_pm_init();

#ifdef CONFIG_SUSPEND
	suspend_set_ops(&sprd_pm_ops);
#endif

	return 0;
}

subsys_initcall(sprd_pm_init);

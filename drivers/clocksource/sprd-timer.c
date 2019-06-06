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

#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/cpu.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>
#include <linux/slab.h>

#include <clocksource/sprd_timer.h>

#ifdef CONFIG_ARM
#include <linux/delay.h>
#endif

static struct sprd_clockevent *r3p0_evt_base;
static void __iomem *r3p0_source_base;

static int r3p0_set_next_event(unsigned long cycles,
			       struct clock_event_device *evt)
{
	struct sprd_clockevent *sevt = to_sprd_evt(evt);
	int val = 0;

	val = readl_relaxed(sevt->base + CTL_R3P0);
	val &= ~CTL_ENABLE_R3P0;
	writel_relaxed(val, sevt->base + CTL_R3P0);
	writel_relaxed(cycles, sevt->base + LOAD_R3P0);
	writel_relaxed(CTL_NEW_R3P0 | CTL_ENABLE_R3P0 | val,
		       sevt->base + CTL_R3P0);

	return 0;
}

static int r3p0_set_state_shutdown(struct clock_event_device *evt)
{
	struct sprd_clockevent *sevt = to_sprd_evt(evt);
	unsigned int val;

	val = readl_relaxed(sevt->base + CTL_R3P0);
	val &= ~CTL_ENABLE_R3P0;
	writel_relaxed(val, sevt->base + CTL_R3P0);
	return 0;
}

static irqreturn_t r3p0_interrupt(int irq, void *dev_id)
{
	unsigned int val;
	struct sprd_clockevent *sevt = (struct sprd_clockevent *)dev_id;

	val = readl_relaxed(sevt->base + CTL_R3P0);
	val &= ~CTL_ENABLE_R3P0;
	writel_relaxed(val, sevt->base + CTL_R3P0);

	val = readl_relaxed(sevt->base + INT_R3P0);
	val |= INT_CLR_R3P0;
	writel_relaxed(val, sevt->base + INT_R3P0);

	if (sevt->evt.event_handler && (val & INT_RAW_STS_R3P0))
		sevt->evt.event_handler(&sevt->evt);

	return IRQ_HANDLED;
}

static void r3p0_evt_common_init(struct sprd_clockevent *sevt)
{
	sevt->evt.features = CLOCK_EVT_FEAT_ONESHOT;
	sevt->evt.set_next_event = r3p0_set_next_event;
	sevt->evt.set_state_shutdown = r3p0_set_state_shutdown;

	/* enable timer, set new mode and oneshot mode */
	writel_relaxed(CTL_NEW_R3P0 | CTL_ENABLE_R3P0, sevt->base + CTL_R3P0);
	/* enable interrupt forever */
	writel_relaxed(INT_CLR_R3P0 | INT_EN_R3P0, sevt->base + INT_R3P0);
}

static void __init r3p0_bcevt_init(struct device_node *np)
{
	int ret = 0;
	struct sprd_clockevent *sevt = NULL;

	sevt = kzalloc(sizeof(struct sprd_clockevent), GFP_KERNEL);
	sevt->base = of_iomap(np, 0);
	if (!sevt->base)
		pr_err("Can't map bc evt reg!\n");

	sevt->evt.irq = irq_of_parse_and_map(np, 0);
	if (sevt->evt.irq <= 0)
		pr_err("Can't map bc irq");

	if (of_property_read_u32(np, "clock-frequency", &sevt->freq))
		pr_err("Can't get bc timer freq!\n");

	r3p0_evt_common_init(sevt);
	sevt->evt.features |= CLOCK_EVT_FEAT_DYNIRQ;
	sevt->evt.rating = 150;
	sevt->evt.name = "r3p0_bcevt";
	sevt->evt.cpumask = cpu_all_mask;

	ret = request_irq(sevt->evt.irq, r3p0_interrupt,
			  IRQF_TIMER, "r3p0_bcevt_timer", sevt);
	if (ret)
		pr_err("request irq for r3p0 bcevt fail!\n");

	clockevents_config_and_register(&sevt->evt, sevt->freq, 0xf,
					0x7fffffff);

	pr_notice("bc_irq is %d,clock-frequency is %d\n",
		  sevt->evt.irq, sevt->freq);
}

static notrace cycle_t r3p0_read_count(struct clocksource *cs)
{
	return ~(readl_relaxed(r3p0_source_base + CNT_RD_R3P0));
}

static void r3p0_suspend(struct clocksource *cs)
{
	int val = 0;

	val = readl_relaxed(r3p0_source_base + CTL_R3P0);
	val &= ~CTL_ENABLE_R3P0;
	writel_relaxed(val, r3p0_source_base + CTL_R3P0);
}

static void r3p0_resume(struct clocksource *cs)
{
	int val = 0;

	val = readl_relaxed(r3p0_source_base + CTL_R3P0);
	writel_relaxed(CTL_ENABLE_R3P0 | val, r3p0_source_base + CTL_R3P0);
}

static struct clocksource r3p0_clocksource = {
	.name = "sprd_r3p0_clksrc",
	.rating = 300,
	.read = r3p0_read_count,
	.mask = CLOCKSOURCE_MASK(32),
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
	.suspend = r3p0_suspend,
	.resume = r3p0_resume,
};

static int r3p0_evt_setup(struct sprd_clockevent *sevt)
{
	int cpu = smp_processor_id();
	int err;

	r3p0_evt_common_init(sevt);
	sevt->evt.name = "r3p0_evt";
	sevt->evt.rating = 300;
	sevt->evt.cpumask = cpumask_of(cpu);
	clockevents_config_and_register(&sevt->evt, sevt->freq, 0xf,
					0x7fffffff);

	err = request_irq(sevt->evt.irq, r3p0_interrupt, IRQF_TIMER,
			  "r3p0_evt_timer", sevt);
	if (err)
		pr_err("request irq for r3p0 evt failed\n");

	irq_force_affinity(sevt->evt.irq, cpumask_of(cpu));
	return 0;
}

static void r3p0_evt_stop(struct sprd_clockevent *sevt)
{
	struct clock_event_device *evt = &sevt->evt;

	evt->set_state_shutdown(evt);
	free_irq(sevt->evt.irq, sevt);
}

static int r3p0_cpu_notify(struct notifier_block *self,
			   unsigned long action, void *hcpu)
{
	/*
	 * Grab cpu pointer in each case to avoid spurious
	 * preemptible warnings
	 */
	int cpu = smp_processor_id();

	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_STARTING:
		r3p0_evt_setup(r3p0_evt_base + cpu);
		break;
	case CPU_DYING:
		r3p0_evt_stop(r3p0_evt_base + cpu);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block r3p0_cpu_nb = {
	.notifier_call = r3p0_cpu_notify,
};

static u64 notrace r3p0_sched_clock_read(void)
{
	return r3p0_clocksource.read(&r3p0_clocksource);
}

#ifdef CONFIG_ARM
static unsigned long r3p0_read_current_timer(void)
{
	return r3p0_clocksource.read(&r3p0_clocksource);
}

static struct delay_timer r3p0_delay_timer = {
	.read_current_timer = r3p0_read_current_timer,
};
#endif

static void r3p0_clksrc_register(u32 clksrc_freq)
{
	/* enable and register clocksource */
	writel_relaxed(0, r3p0_source_base + INT_R3P0);
	writel_relaxed(CTL_ENABLE_R3P0 | CTL_PERIOD_MODE_R3P0 | CTL_NEW_R3P0,
		       r3p0_source_base + CTL_R3P0);
	writel_relaxed(0xFFFFFFFF, r3p0_source_base + LOAD_R3P0);

	if (clocksource_register_hz(&r3p0_clocksource, clksrc_freq))
		pr_err("clocksource_register failed\n");
	sched_clock_register(r3p0_sched_clock_read, 32, clksrc_freq);
#ifdef CONFIG_ARM
	r3p0_delay_timer.freq = clksrc_freq;
	register_current_timer_delay(&r3p0_delay_timer);
#endif
}

static void r3p0_evt_register(void)
{
	if (register_cpu_notifier(&r3p0_cpu_nb))
		pr_err("register evt timer nb failed\n");

	/* Immediately configure the timer on the boot CPU */
	r3p0_evt_setup(r3p0_evt_base);
}

static void __init r3p0_evt_init(struct device_node *np)
{
	int i = 0;
	u32 freq = 0;
	int evt_num = 0;
	struct resource res;
	struct device_node *timer_node = NULL;
	struct sprd_clockevent *this_cpu_evt;

	evt_num = of_property_count_u32_elems(np, "sprd,evtlist");
	r3p0_evt_base = kcalloc(evt_num, sizeof(struct sprd_clockevent),
				GFP_KERNEL);
	if (!r3p0_evt_base)
		return;

	for (i = 0; i < evt_num; i++) {
		this_cpu_evt = r3p0_evt_base + i;
		timer_node = of_parse_phandle(np, "sprd,evtlist", i);
		if (!timer_node) {
			pr_err("Can not get evt%d node\n", i);
			return;
		}

		if (of_property_read_u32
		    (timer_node, "clock-frequency", &this_cpu_evt->freq)) {
			pr_err("Unknown evt frequency\n");
			return;
		}

		if (of_address_to_resource(timer_node, 0, &res)) {
			pr_err("Failed to parse evt resource\n");
			return;
		}

		this_cpu_evt->base = ioremap(res.start, resource_size(&res));
		if (!this_cpu_evt->base) {
			pr_err("Failed to map evt timer base\n");
			return;
		}

		this_cpu_evt->evt.irq = irq_of_parse_and_map(timer_node, 0);
		if (this_cpu_evt->evt.irq <= 0) {
			pr_err("Can't get evt irq\n");
			return;
		}
	}

	timer_node = of_parse_phandle(np, "sprd,clksrc", 0);
	if (!timer_node) {
		pr_err("Can not get clocksource node\n");
		return;
	}
	r3p0_source_base = of_iomap(timer_node, 0);
	if (!r3p0_source_base) {
		pr_err("Failed to map clksrc base\n");
		return;
	}

	if (of_property_read_u32(timer_node, "clock-frequency", &freq)) {
		pr_err("Unknown clksrc frequency\n");
		return;
	}

	r3p0_clksrc_register(freq);
	r3p0_evt_register();
}

CLOCKSOURCE_OF_DECLARE(r3p0_evt_timer, "sprd,evt-r3p0", r3p0_evt_init);
CLOCKSOURCE_OF_DECLARE(r3p0_bcevt_timer, "sprd,bcevt-r3p0", r3p0_bcevt_init);

static struct sprd_clockevent *r4p0_evt_base;
static void __iomem *r4p0_source_base;

static int r4p0_set_next_event(unsigned long cycles,
			       struct clock_event_device *evt)
{
	struct sprd_clockevent *sevt = to_sprd_evt(evt);
	int val = 0;

	val = readl_relaxed(sevt->base + CTL_R4P0);
	val &= ~CTL_ENABLE_R4P0;
	writel_relaxed(val, sevt->base + CTL_R4P0);

	writel_relaxed(cycles & 0xFFFFFFFF, sevt->base + LOAD_LO_R4P0);
#ifdef CONFIG_64BIT
	writel_relaxed(cycles >> 32, sevt->base + LOAD_HI_R4P0);
#else
	writel_relaxed(0, sevt->base + LOAD_HI_R4P0);
#endif
	val |= CTL_ENABLE_R4P0;
	writel_relaxed(val, sevt->base + CTL_R4P0);

	return 0;
}

static int r4p0_set_state_shutdown(struct clock_event_device *evt)
{
	struct sprd_clockevent *sevt = to_sprd_evt(evt);
	unsigned int val;

	val = readl_relaxed(sevt->base + CTL_R4P0);
	val &= ~CTL_ENABLE_R4P0;
	writel_relaxed(val, sevt->base + CTL_R4P0);

	return 0;
}

static irqreturn_t r4p0_interrupt(int irq, void *dev_id)
{
	unsigned int val;
	struct sprd_clockevent *sevt = (struct sprd_clockevent *)dev_id;

	val = readl_relaxed(sevt->base + CTL_R4P0);
	val &= ~CTL_ENABLE_R4P0;
	writel_relaxed(val, sevt->base + CTL_R4P0);

	val = readl_relaxed(sevt->base + INT_R4P0);
	val |= INT_CLR_R4P0;
	writel_relaxed(val, sevt->base + INT_R4P0);

	if (sevt->evt.event_handler && (val & INT_RAW_STS_R4P0))
		sevt->evt.event_handler(&sevt->evt);

	return IRQ_HANDLED;
}

static void r4p0_evt_common_init(struct sprd_clockevent *sevt)
{
	sevt->evt.features = CLOCK_EVT_FEAT_ONESHOT;
	sevt->evt.set_next_event = r4p0_set_next_event;
	sevt->evt.set_state_shutdown = r4p0_set_state_shutdown;

	/* enable timer, set new mode and oneshot mode */
	writel_relaxed(CTL_ENABLE_R4P0 | CTL_WIDTH_SEL_R4P0,
		       sevt->base + CTL_R4P0);
	/* enable interrupt forever */
	writel_relaxed(INT_CLR_R4P0 | INT_EN_R4P0, sevt->base + INT_R4P0);
}

static notrace cycle_t r4p0_read_count(struct clocksource *cs)
{
	unsigned int val_lo;
	unsigned int val_hi_old;
	unsigned int val_hi_new;

	val_hi_old = readl_relaxed(r4p0_source_base + VALUE_SHDW_HI_R4P0);
	val_lo = readl_relaxed(r4p0_source_base + VALUE_SHDW_LO_R4P0);
	val_hi_new = readl_relaxed(r4p0_source_base + VALUE_SHDW_HI_R4P0);
	while (val_hi_old != val_hi_new) {
		val_hi_old = val_hi_new;
		val_lo = readl_relaxed(r4p0_source_base + VALUE_SHDW_LO_R4P0);
		val_hi_new =
		    readl_relaxed(r4p0_source_base + VALUE_SHDW_HI_R4P0);
	}
	return ~(((u64) val_hi_new << 32) | val_lo);
}

static void r4p0_suspend(struct clocksource *cs)
{
	int val = 0;

	val = readl_relaxed(r4p0_source_base + CTL_R4P0);
	val &= ~CTL_ENABLE_R4P0;
	writel_relaxed(val, r4p0_source_base + CTL_R4P0);
}

static void r4p0_resume(struct clocksource *cs)
{
	int val = 0;

	val = readl_relaxed(r4p0_source_base + CTL_R4P0);
	writel_relaxed(CTL_ENABLE_R4P0 | val, r4p0_source_base + CTL_R4P0);
}

static struct clocksource r4p0_clocksource = {
	.name = "sprd_r4p0_clksrc",
	.rating = 300,
	.read = r4p0_read_count,
	.mask = CLOCKSOURCE_MASK(64),
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
	.suspend = r4p0_suspend,
	.resume = r4p0_resume,
};

static int r4p0_evt_setup(struct sprd_clockevent *sevt)
{
	int cpu = smp_processor_id();
	int err;

	r4p0_evt_common_init(sevt);
	sevt->evt.name = "r4p0_evt";
	sevt->evt.rating = 300;
	sevt->evt.cpumask = cpumask_of(cpu);
	clockevents_config_and_register(&sevt->evt, sevt->freq, 0xf,
					0x7fffffff);

	err = request_irq(sevt->evt.irq, r4p0_interrupt, IRQF_TIMER,
			  "r4p0_evt_timer", sevt);
	if (err)
		pr_err("request irq for r4p0 evt failed\n");

	irq_force_affinity(sevt->evt.irq, cpumask_of(cpu));
	return 0;
}

static void r4p0_evt_stop(struct sprd_clockevent *sevt)
{
	struct clock_event_device *evt = &sevt->evt;

	evt->set_state_shutdown(evt);
	free_irq(sevt->evt.irq, sevt);
}

static int r4p0_cpu_notify(struct notifier_block *self,
			   unsigned long action, void *hcpu)
{
	/*
	 * Grab cpu pointer in each case to avoid spurious
	 * preemptible warnings
	 */
	int cpu = smp_processor_id();

	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_STARTING:
		r4p0_evt_setup(r4p0_evt_base + cpu);
		break;
	case CPU_DYING:
		r4p0_evt_stop(r4p0_evt_base + cpu);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block r4p0_cpu_nb = {
	.notifier_call = r4p0_cpu_notify,
};

static u64 notrace r4p0_sched_clock_read(void)
{
	return r4p0_clocksource.read(&r4p0_clocksource);
}

#ifdef CONFIG_ARM
static unsigned long r4p0_read_current_timer(void)
{
	return r4p0_clocksource.read(&r4p0_clocksource);
}

static struct delay_timer r4p0_delay_timer = {
	.read_current_timer = r4p0_read_current_timer,
};
#endif

static void r4p0_clksrc_register(u32 clksrc_freq)
{
	/* enable and register clocksource */
	writel_relaxed(CTL_ENABLE_R4P0 | CTL_WIDTH_SEL_R4P0 |
		       CTL_PERIOD_MODE_R4P0, r4p0_source_base + CTL_R4P0);
	writel_relaxed(INT_CLR_R4P0, r4p0_source_base + INT_R4P0);
	writel_relaxed(0xFFFFFFFF, r4p0_source_base + LOAD_HI_R4P0);
	writel_relaxed(0xFFFFFFFF, r4p0_source_base + LOAD_LO_R4P0);

	if (clocksource_register_hz(&r4p0_clocksource, clksrc_freq))
		pr_err("clocksource_register failed\n");
	sched_clock_register(r4p0_sched_clock_read, 32, clksrc_freq);
#ifdef CONFIG_ARM
	r4p0_delay_timer.freq = clksrc_freq;
	register_current_timer_delay(&r4p0_delay_timer);
#endif
}

static void r4p0_evt_register(void)
{
	if (register_cpu_notifier(&r4p0_cpu_nb))
		pr_err("register evt timer nb failed\n");

	/* Immediately configure the timer on the boot CPU */
	r4p0_evt_setup(r4p0_evt_base);
}

static void __init r4p0_evt_init(struct device_node *np)
{
	int i = 0;
	u32 freq = 0;
	int evt_num = 0;
	struct resource res;
	struct device_node *timer_node = NULL;
	struct sprd_clockevent *this_cpu_evt;

	evt_num = of_property_count_u32_elems(np, "sprd,evtlist");
	r4p0_evt_base = kcalloc(evt_num, sizeof(struct sprd_clockevent),
				GFP_KERNEL);
	if (!r4p0_evt_base)
		return;

	for (i = 0; i < evt_num; i++) {
		this_cpu_evt = r4p0_evt_base + i;
		timer_node = of_parse_phandle(np, "sprd,evtlist", i);
		if (!timer_node) {
			pr_err("Can not get evt%d node\n", i);
			return;
		}

		if (of_property_read_u32
		    (timer_node, "clock-frequency", &this_cpu_evt->freq)) {
			pr_err("Unknown evt frequency\n");
			return;
		}

		if (of_address_to_resource(timer_node, 0, &res)) {
			pr_err("Failed to parse evt resource\n");
			return;
		}

		this_cpu_evt->base = ioremap(res.start, resource_size(&res));
		if (!this_cpu_evt->base) {
			pr_err("Failed to map evt timer base\n");
			return;
		}

		this_cpu_evt->evt.irq = irq_of_parse_and_map(timer_node, 0);
		if (this_cpu_evt->evt.irq <= 0) {
			pr_err("Can't get evt irq\n");
			return;
		}
	}

	timer_node = of_parse_phandle(np, "sprd,clksrc", 0);
	if (!timer_node) {
		pr_err("Can not get clocksource node\n");
		return;
	}
	r4p0_source_base = of_iomap(timer_node, 0);
	if (!r4p0_source_base) {
		pr_err("Failed to map clksrc base\n");
		return;
	}

	if (of_property_read_u32(timer_node, "clock-frequency", &freq)) {
		pr_err("Unknown clksrc frequency\n");
		return;
	}

	r4p0_clksrc_register(freq);
	r4p0_evt_register();
}

static void __init r4p0_bcevt_init(struct device_node *np)
{
	int ret = 0;
	struct sprd_clockevent *sevt = NULL;

	sevt = kzalloc(sizeof(struct sprd_clockevent), GFP_KERNEL);
	sevt->base = of_iomap(np, 0);
	if (!sevt->base)
		pr_err("Can't map bc evt reg!\n");

	sevt->evt.irq = irq_of_parse_and_map(np, 0);
	if (sevt->evt.irq <= 0)
		pr_err("Can't map bc irq");

	if (of_property_read_u32(np, "clock-frequency", &sevt->freq))
		pr_err("Can't get bc timer freq!\n");

	r4p0_evt_common_init(sevt);
	sevt->evt.features |= CLOCK_EVT_FEAT_DYNIRQ;
	sevt->evt.rating = 150;
	sevt->evt.name = "r4p0_bcevt_timer";
	sevt->evt.cpumask = cpu_all_mask;

	ret = request_irq(sevt->evt.irq, r4p0_interrupt, IRQF_TIMER,
			  "r4p0_bcevt_timer", sevt);
	if (ret)
		pr_err("request irq for r4p0 bcevt fail!\n");

	clockevents_config_and_register(&sevt->evt, sevt->freq, 0xf,
					0x7fffffff);

	pr_notice("bc_irq is %d,clock-frequency is %d\n",
		  sevt->evt.irq, sevt->freq);
}

CLOCKSOURCE_OF_DECLARE(r4p0_evt_timer, "sprd,evt-r4p0", r4p0_evt_init);
CLOCKSOURCE_OF_DECLARE(r4p0_bcevt_timer, "sprd,bcevt-r4p0", r4p0_bcevt_init);

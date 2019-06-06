/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
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
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/time.h>

#include <clocksource/sprd_timer.h>

#ifdef CONFIG_ARM
#include <asm/mach/time.h>
#endif

#ifdef CONFIG_X86
#include <asm/x86_init.h>
#endif

static void __iomem *sprd_perclock_base;
static u32 clock_freq;
static struct timespec persistent_ts;
static u64 persistent, last_persistent;

static notrace cycle_t sprd_perclock_read_count(void)
{
	unsigned int val_lo;
	unsigned int val_hi_old;
	unsigned int val_hi_new;

	val_hi_old = readl_relaxed(sprd_perclock_base + VALUE_SHDW_HI_R4P0);
	val_lo = readl_relaxed(sprd_perclock_base + VALUE_SHDW_LO_R4P0);
	val_hi_new = readl_relaxed(sprd_perclock_base + VALUE_SHDW_HI_R4P0);
	while (val_hi_old != val_hi_new) {
		val_hi_old = val_hi_new;
		val_lo = readl_relaxed(sprd_perclock_base + VALUE_SHDW_LO_R4P0);
		val_hi_new =
		    readl_relaxed(sprd_perclock_base + VALUE_SHDW_HI_R4P0);
	}
	return ~(((u64) val_hi_new << 32) | val_lo);
}

#if defined(CONFIG_X86)
static void sprd_read_persistent_clock(struct timespec *ts)
{
	u64 delta;

	last_persistent = persistent;
	persistent = sprd_perclock_read_count();
	delta = persistent - last_persistent;
	delta = delta * NSEC_PER_SEC;
	delta = div64_u64(delta, clock_freq);
	timespec_add_ns(&persistent_ts, delta);
	*ts = persistent_ts;
}
#elif defined(CONFIG_ARM)
static void sprd_read_persistent_clock(struct timespec64 *ts)
{
	u64 delta;

	last_persistent = persistent;
	persistent = sprd_perclock_read_count();
	delta = persistent - last_persistent;
	delta = delta * NSEC_PER_SEC;
	delta = div64_u64(delta, clock_freq);
	timespec_add_ns(&persistent_ts, delta);
	*ts = timespec_to_timespec64(persistent_ts);
}
#else
void read_persistent_clock(struct timespec *ts)
{
	u64 delta;

	if (sprd_perclock_base) {
		last_persistent = persistent;
		persistent = sprd_perclock_read_count();
		delta = persistent - last_persistent;
		delta = delta * NSEC_PER_SEC;
		delta = div64_u64(delta, clock_freq);
		timespec_add_ns(&persistent_ts, delta);
		*ts = persistent_ts;
	} else {
		ts->tv_sec = 0;
		ts->tv_nsec = 0;
	}
}
#endif

static void __init sprd_persistent_clock_init(struct device_node *np)
{
	if (of_property_read_u32(np, "clock-frequency", &clock_freq)) {
		pr_err("Can't get sprd persistent clock freq!\n");
		return;
	}

	sprd_perclock_base = of_iomap(np, 0);
	if (!sprd_perclock_base) {
		pr_err("Can't map sprd persistent clock reg!\n");
		return;
	}

	/* enable persistent clock */
	writel_relaxed(INT_CLR_R4P0, sprd_perclock_base + INT_R4P0);
	writel_relaxed(0xFFFFFFFF, sprd_perclock_base + LOAD_HI_R4P0);
	writel_relaxed(0xFFFFFFFF, sprd_perclock_base + LOAD_LO_R4P0);
	writel_relaxed(CTL_ENABLE_R4P0 | CTL_WIDTH_SEL_R4P0 |
		       CTL_PERIOD_MODE_R4P0, sprd_perclock_base + CTL_R4P0);

#ifdef CONFIG_ARM
	register_persistent_clock(NULL, sprd_read_persistent_clock);
#endif

#ifdef CONFIG_X86
	x86_platform.get_wallclock = sprd_read_persistent_clock;
#endif
	pr_notice("sprd persistent clock init, base:0x%p, freq:%d\n",
			sprd_perclock_base, clock_freq);
}

CLOCKSOURCE_OF_DECLARE(sprd_persistent_clock, "sprd,persistent-clock",
		       sprd_persistent_clock_init);

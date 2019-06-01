/*
 * Copyright (C) 2013 Spreadtrum Communications Inc.
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
#include <linux/input-hook.h>
#include <linux/delay.h>//add by liuwei
#include <soc/sprd/sys_reset.h>
#include <asm/system_misc.h>


#ifdef CONFIG_KEYBOARD_SYSDUMP_BYPASS
extern unsigned int powerkey_reset_enable;
#endif

static enum hrtimer_restart trigger_watch_event_timer_cb(struct hrtimer *timer)
{
    struct trigger_watch_event *event = container_of(timer, struct trigger_watch_event, timer);
    enum hrtimer_restart restart = HRTIMER_NORESTART;
    pr_info("< %s > trigger-watch event timer timeout %dms\n", event->name, event->period);
    if (event->trigger_watch_event_cb)
        event->trigger_watch_event_cb(event->private);
    if (event->repeated)   /* repeat timer */
    {
        hrtimer_forward_now(timer, ktime_set(event->period / 1000, (event->period % 1000) * 1000000));
        restart = HRTIMER_RESTART;
    }
    return restart;
}

int trigger_watch_event_stop(struct trigger_watch_event *event)
{
    if (!event->disable_timer)
    {
        hrtimer_cancel(&event->timer);
    }
    if (!event->disable_clk_source)
    {
        u64 delta = (local_clock() - event->last_ts);
        if (delta > ((u64)event->period) * 1000000)
        {
            pr_info("< %s > trigger-watch event clock timeout %d / %lld\n", event->name, event->period, delta);
            if (event->trigger_watch_event_cb)
                event->trigger_watch_event_cb(event->private);
        }
    }
    return 0;
}
EXPORT_SYMBOL(trigger_watch_event_stop);

void trigger_watch_event_start(struct trigger_watch_event *event, int ms)
{
    if (!event->initialized)
    {
        event->initialized = 1;
        if (!event->disable_timer)
        {
            hrtimer_init(&event->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
            event->timer.function = trigger_watch_event_timer_cb;
        }
    }
    event->repeated = !!(ms & TRIGGER_WATCH_EVENT_REPEATED);
    event->period = (ms & ~TRIGGER_WATCH_EVENT_REPEATED);
    event->last_ts = local_clock();
    trigger_watch_event_stop(event);
    if (!event->disable_timer)
    {
        hrtimer_start(&event->timer, ktime_set(event->period / 1000, (event->period % 1000) * 1000000), HRTIMER_MODE_REL);
    }
}
EXPORT_SYMBOL(trigger_watch_event_start);

static int hook_power = 1;

static struct ctl_table_header *hook_power_sysctl_hdr = NULL;
void input_hook_init(void)
{
    static ctl_table hook_power_sysctl_table[] =
    {
        {
            .procname       = "hook_power_enable",
            .data           = &hook_power,
            .maxlen         = sizeof(int),
            .mode           = 0644,
            .proc_handler   = proc_dointvec,
        },
        {}
    };

    static ctl_table hook_power_sysctl_root[] =
    {
        {
            .procname       = "kernel",
            .mode           = 0555,
            .child          = hook_power_sysctl_table,
        },
        {}
    };

    hook_power_sysctl_hdr = register_sysctl_table(hook_power_sysctl_root);
    if (!hook_power_sysctl_hdr)
    {
        pr_err("unable to create input_power entry\n");
        return -ENOMEM;
    }
    return 0;
}

void input_hook_exit(void)
{
    if (hook_power_sysctl_hdr)
        unregister_sysctl_table(hook_power_sysctl_hdr);
}

extern void sprd_pbint_7s_reset_disable(void);
extern void set_vibrator(int on);//add by liuwei
static void trigger_watch_powerkey(void *private)
{
#ifndef CONFIG_KEYBOARD_SYSDUMP_BYPASS
    unsigned long flags;
    local_irq_save(flags);
#ifdef CONFIG_MAGIC_SYSRQ
    handle_sysrq('m');
    handle_sysrq('w');
#endif
    sprd_pbint_7s_reset_disable();
    pr_warn("!!!! trigger_powerkey !!!! do panic\n");
    set_vibrator(1);//add by liuwei
    mdelay(300);//add by liuwei
    set_vibrator(0);//add by liuwei
    //sprd_set_reboot_mode("reboot") ;
    panic("!!!! trigger_powerkey !!!! do panic");
    pr_err("%s should never reach here!\n", __func__);
#else
    char cmd = 0;
    set_vibrator(1);//add by liuwei
    mdelay(300);//add by liuwei
    set_vibrator(0);//add by liuwei
    printk("!!!! trigger_powerkey !!!! do poweroff\n");
    //machine_restart(&cmd);
    //panic("!!!! trigger_powerkey !!!! do panic");
    //arm_pm_restart('h', "reboot");
    pm_power_off();
#endif
}

void input_report_key_hook(struct input_dev *dev, unsigned int code, int value)
{

    if (hook_power && code == KEY_POWER)
    {
        static struct trigger_watch_event twe =
        {
            .name = "power key",
            .private = NULL, /* like dev */
            .trigger_watch_event_cb = trigger_watch_powerkey,
        };
        static int pre_value = 0;
        if (value != pre_value)
        {
            if (value)
            {
#ifdef CONFIG_KEYBOARD_SYSDUMP_BYPASS
                if (powerkey_reset_enable)
#endif
                {
                    trigger_watch_event_start(&twe, 8 * 1000);
                }
            }
            else
            {
#ifdef CONFIG_KEYBOARD_SYSDUMP_BYPASS
                if (powerkey_reset_enable)
#endif
                {
                    trigger_watch_event_stop(&twe);
                }
            }
        }
        else
        {
            pr_warn("Key %d%c is dithering.", code, value ? 'D' : 'U');
        }
        pre_value = value;
    }
}

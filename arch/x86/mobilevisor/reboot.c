/* Copyright (C) 2013 Intel Mobile Communications GmbH
 * *
 * * This software is licensed under the terms of the GNU General Public
 * * License version 2, as published by the Free Software Foundation, and
 * * may be copied, distributed, and modified under those terms.
 * *
 * * This program is distributed in the hope that it will be useful,
 * * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * * GNU General Public License for more details.
 * */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/reboot.h>
#include <linux/notifier.h>
#include <linux/percpu.h>
#include <linux/smp.h>

#include <linux/io.h>
#include <asm/reboot.h>
#include <asm/mv/mobilevisor.h>
#include <asm/mv/mv_gal.h>


#define TRACE(x...) pr_notice("REBOOT driver: " x)
#define ETRACE(x...) pr_err("REBOOT driver: Error: " x)

#define HWRST_POWERON_MASK      (0xf0)
#define HWRST_RECOVERY          (0x20)
#define HWRST_FASTBOOT          (0x30)
#define HWRST_NORMAL            (0x40)
#define HWRST_ALARM             (0x50)
#define HWRST_SLEEP             (0x60)
#define HWRST_SPECIAL           (0x70)
#define HWRST_CALIBRATION       (0x90)
#define HWRST_PANIC             (0x80)
#define HWRST_AUTODLOADER       (0xa0)
#define HWRST_IQMODE            (0xb0)
#define HWRST_SPRDISK           (0xc0)
#define HWRST_NORMAL2           (0xf0)

static volatile unsigned int running_guest;
static bool is_linux_reboot = true;
static bool is_blocking;

extern void (*x86_pm_restart)(enum reboot_mode reboot_mode, const char *cmd);

extern void (*x86_pm_restart_prepare)(const char *cmd);

static void next_mode_transition(uint32_t *next_mode, char *reason)
{

	if (!reason)
		*next_mode = (HWRST_NORMAL << 16) | ENUM_BOOT_MODE_NORMAL;
	else if (strcmp(reason, "halt") == 0)
		*next_mode = ENUM_BOOT_MODE_SHUT_DOWN;
	else if (strcmp(reason, "poweroff") == 0)
		*next_mode = ENUM_BOOT_MODE_SHUT_DOWN;
	else if (strcmp(reason, "iqmode") == 0)
		*next_mode = (HWRST_IQMODE << 16) | ENUM_BOOT_MODE_IQMODE;
	else if (strcmp(reason, "alarm") == 0)
		*next_mode = (HWRST_ALARM << 16) | ENUM_BOOT_MODE_ALARM;
	else if (strcmp(reason, "recovery") == 0)
		*next_mode = (HWRST_RECOVERY << 16) | ENUM_BOOT_MODE_RECOVERY;
	else if (strcmp(reason, "fastsleep") == 0)
		*next_mode = (HWRST_SLEEP << 16) | ENUM_BOOT_MODE_SLEEP;
	else if (strcmp(reason, "bootloader") == 0)
		*next_mode = (HWRST_FASTBOOT << 16) | ENUM_BOOT_MODE_FASTBOOT;
	else if (strcmp(reason, "autodloader") == 0)
		*next_mode = (HWRST_AUTODLOADER << 16)
					| ENUM_BOOT_MODE_AUTODLOADER;
	else if (strcmp(reason, "cftreboot") == 0)
		*next_mode = (HWRST_CALIBRATION << 16)
					| ENUM_BOOT_MODE_CALIBRATION;
	else if (strcmp(reason, "special") == 0)
		*next_mode = (HWRST_SPECIAL << 16) | ENUM_BOOT_MODE_SPECIAL;
	else if (strcmp(reason, "sprdisk") == 0)
		*next_mode = (HWRST_SPRDISK << 16) | ENUM_BOOT_MODE_SPRDISK;
	else if (strcmp(reason, "panic") == 0)
		*next_mode = (HWRST_PANIC << 16) | ENUM_BOOT_MODE_SYS_CRASH;
	else
		*next_mode = (HWRST_NORMAL << 16) | ENUM_BOOT_MODE_NORMAL;
	TRACE("next_mode_transition 0x%x, reboot cmd: %s\n",
			*next_mode, reason);
}

static void deferred_reboot(struct work_struct *dummy)
{
	kernel_restart(NULL);
}

static DECLARE_WORK(reboot_work, deferred_reboot);
static irqreturn_t reboot_sysconf_hdl(int xirq, void *dev)
{
	struct mv_shared_data *data;
	data = mv_gal_get_shared_data();
	/* TRACE("system_reboot_action: %d\n", data->system_reboot_action); */
	if (data->is_platform_reboot_initiated > 0) {
		if (!is_blocking) {
			TRACE("This reboot triggered by other OS\n");
			is_linux_reboot = false;
			schedule_work(&reboot_work);
		}
	}

	return IRQ_HANDLED;
}

static int reboot(struct notifier_block *notifier,
		unsigned long event, void *data)
{
	uint32_t next_mode = 0;
	struct mv_shared_data *share_data;
	TRACE("Event code: %li!  reboot cmd: %s\n", event, (char *)data);
	if ((event == SYS_HALT) || (event == SYS_POWER_OFF))
		next_mode_transition(&next_mode, "poweroff");
	else
		next_mode_transition(&next_mode, (char *)data);
	is_blocking = true;
	if (is_linux_reboot) {
		enum boot_mode next_boot_mode;

		next_boot_mode = (enum boot_mode) (next_mode & 0xffff);
		if (next_boot_mode > ENUM_BOOT_MODE_POWER_OFF &&
			next_boot_mode < ENUM_BOOT_MODE_SHUT_DOWN) {
			if (x86_pm_restart_prepare)
				x86_pm_restart_prepare((char *)data);
			if (x86_pm_restart)
				x86_pm_restart(REBOOT_WDT, (char *)data);
		}
		mv_initiate_reboot(next_mode);
	}
	/*
	 * Block reboot process and waiting for other os finish the reboot
	 * since we have NVM depency
	 */
	while (1) {
		share_data = mv_gal_get_shared_data();
		running_guest = mv_get_running_guests();
		if (running_guest == (1 << share_data->os_id))
			break;
	}
	TRACE("all other guest os reboot successfully!\n");
	return NOTIFY_OK;
}

static struct notifier_block reboot_notifier = {
	.notifier_call = reboot
};

#ifdef CONFIG_KEXEC
static void vmm_machine_crash_shutdown(struct pt_regs *regs)
{
	machine_shutdown();
}
#endif

static void vmm_machine_emergency_restart(void)
{
	machine_shutdown();
}

static void vmm_machine_shutdown(void)
{
#ifdef CONFIG_SMP
	/*
	 * Stop all of the others. Also disable the local irq to
	 * not receive any interrupt which may trigger
	 * scheduler's load balance.
	 */
	local_irq_disable();
	stop_other_cpus();
#endif
	mv_vcpu_stop(smp_processor_id());
}

static void vmm_machine_restart(char *__unused)
{
	pr_notice("machine restart\n");
	machine_shutdown();
}

static void vmm_machine_halt(void)
{
	/* Stop other cpus and apics */
	machine_shutdown();

	stop_this_cpu(NULL);
}

static void vmm_machine_power_off(void)
{
	machine_shutdown();
}

static struct machine_ops vmm_machine_ops = {
	.power_off = vmm_machine_power_off,
	.shutdown = vmm_machine_shutdown,
	.emergency_restart = vmm_machine_emergency_restart,
	.restart = vmm_machine_restart,
	.halt = vmm_machine_halt,
#ifdef CONFIG_KEXEC
	.crash_shutdown = vmm_machine_crash_shutdown,
#endif
};

static int __init reboot_init(void)
{
	if (is_x86_mobilevisor()) {
		running_guest = mv_get_running_guests();
		mv_gal_register_hirq_callback(512,
					reboot_sysconf_hdl, NULL);
		register_reboot_notifier(&reboot_notifier);
		machine_ops = vmm_machine_ops;
	}
	return 0;
}

static void __exit reboot_exit(void)
{
	if (is_x86_mobilevisor())
		unregister_reboot_notifier(&reboot_notifier);
}

module_init(reboot_init);
module_exit(reboot_exit);

MODULE_LICENSE("GPL");



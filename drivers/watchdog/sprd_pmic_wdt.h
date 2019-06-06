#ifndef _SPRD_PMIC_WDT_H
#define _SPRD_PMIC_WDT_H

#ifdef CONFIG_X86
#include <linux/reboot.h>
extern void (*x86_pm_restart)(enum reboot_mode reboot_mode, const char *cmd);
extern void (*x86_pm_restart_prepare)(const char *cmd);
#endif

#endif

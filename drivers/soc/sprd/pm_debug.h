#ifndef _SPRD_PM_DEBUG_H
#define _SPRD_PM_DEBUG_H

/*print message every 30 second*/
#define PM_PRINT_ENABLE

/*sleep mode info*/
#define SLP_MODE_ARM  0
#define SLP_MODE_MCU  1
#define SLP_MODE_DEP  2
#define SLP_MODE_LIT  3
#define SLP_MODE_NON  4

extern void print_statisic(void);
extern void clr_sleep_mode(void);
extern void time_statisic_begin(void);
extern void time_statisic_end(void);
extern void irq_wakeup_set(void);
extern void sc_pm_init(void);
extern unsigned int sprd_irq_pending(void);
extern int sprd_cpu_deep_sleep(unsigned int cpu);

#endif/*_SPRD_PM_DEBUG_H*/

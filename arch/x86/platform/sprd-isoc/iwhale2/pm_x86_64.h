#ifndef _SPRD_PM_X86_64_H
#define _SPRD_PM_X86_64_H

/*print message every 30 second*/
#define PM_PRINT_ENABLE
extern int sprd_s3_deep_sleep(unsigned int cpu);
extern int sprd_s3_wake_up(void);
extern void sc_light_init(void);
extern void hard_irq_reset(void);
extern void (*sprd_soc_deepsleep_enter)(unsigned int cpu);
extern void (*sprd_soc_wakeup_enter)(void);
#endif/*_SPRD_PM_X86_64_H*/

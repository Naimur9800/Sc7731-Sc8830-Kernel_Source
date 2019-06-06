#ifndef _SPRD_PM_H
#define _SPRD_PM_H

/*externs should be avoided in .c files*/
extern void (*arm_pm_restart)(char str, const char *cmd);
extern int sc8830_get_clock_status(void);
extern void secondary_startup(void);
extern int sp_pm_collapse(unsigned int cpu, unsigned int save_state);
extern void sp_pm_collapse_exit(void);
extern void sc8830_standby_iram(void);
extern void sc8830_standby_iram_end(void);
extern void sc8830_standby_exit_iram(void);

extern void __iomem *reg_aon_apb_eb0_vir;
extern unsigned long reg_aon_apb_eb0_phy;
extern void __iomem *reg_uart1_enable_vir;
extern unsigned long reg_uart1_enable_phy;
extern void __iomem *reg_sprd_uart1_base;
#endif/*_SPRD_PM_H*/

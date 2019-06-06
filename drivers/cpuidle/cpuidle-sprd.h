#ifndef CPUIDLE_SPRD_HEADER
#define CPUIDLE_SPRD_HEADER
enum {
	STANDBY = 0,  /* WFI */
	L_SLEEP,      /* Light Sleep, WFI & DDR Self-refresh & MCU_SYS_SLEEP */
	H_SLEEP,        /* HEAVY/Doze Sleep */
	CORE_PD,	  /* Core power down & Lightsleep */
#ifdef CONFIG_ARM64
	CLUSTER_PD,   /* Cluster power down & Lightsleep */
	TOP_PD,	      /* Top Power Down & Lightsleep */
#endif
};

extern void light_sleep_en(struct device_node *np);
extern void light_sleep_dis(struct device_node *np);
extern bool panel_suspended;
#ifdef CONFIG_ARM
extern void light_doze_sleep_en(struct device_node *np);
extern void light_doze_sleep_dis(struct device_node *np);
#endif

#endif

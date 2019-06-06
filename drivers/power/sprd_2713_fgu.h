
#ifndef _SPRD_2713_FGU_H_
#define _SPRD_2713_FGU_H_
#include <linux/sprd_battery_common.h>
#include <linux/types.h>
#if defined(CONFIG_SPRD_2713_POWER)
#include "sprd_2713_charge.h"
#endif

#define AUTO_CNOM

#ifdef AUTO_CNOM
enum sprdfgu_cnom_record_type{
    END_CNOM_RECORD,
    START_CNOM_RECORD,
    ADJUST_CNOM_RECORD,
    DEBUG_CNOM_RECORD,
};
enum sprdfgu_cnom_opera_type{
    CNOM_OPERA_GET,
    CNOM_OPERA_SET,
    CNOM_OPERA_REC_ORGINAL,
};
#define MAX_CNOM 10000   /*invalid protection*/
#define REAL_CNOM_SAVE_PATH "/productinfo/.cnom"
#define CNOM_RECORD_END_CAP  1
#define CNOM_RECORD_LOW_VOLTAGE_ADJUST  3350
uint sprdfgu_cnom_record(uint capacity, uint vbat_ocv);
int sprdfgu_cnom_state_change(int on);
#endif

#define SPRDBAT_FGUADC_CAL_NO         0
#define SPRDBAT_FGUADC_CAL_NV         1
#define SPRDBAT_FGUADC_CAL_CHIP      2

int sprdfgu_init(struct sprd_battery_platform_data *pdata);
int sprdfgu_reset(void);
void sprdfgu_record_cap(u32 cap);
uint32_t sprdfgu_read_capacity(void);
uint32_t sprdfgu_poweron_capacity(void);
int sprdfgu_read_soc(void);
int sprdfgu_read_batcurrent(void);
uint32_t sprdfgu_read_vbat_vol(void);
uint32_t sprdfgu_read_vbat_ocv(void);
int sprdfgu_register_notifier(struct notifier_block *nb);
int sprdfgu_unregister_notifier(struct notifier_block *nb);
void sprdfgu_adp_status_set(int plugin);
void sprdfgu_pm_op(int is_suspend);

#endif

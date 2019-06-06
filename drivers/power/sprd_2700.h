#ifndef __LINUX_SPRD2700_H
#define __LINUX_SPRD2700_H

#include <linux/ioctl.h>
#include "sprd_battery.h"

typedef unsigned char BYTE;
#define SPRDBAT_CHG_CUR_LEVEL_MIN   100
#define SPRDBAT_CHG_CUR_LEVEL_MAX	1600

/******************************************************************************
* Register addresses start
******************************************************************************/
#define POWER_ON_CTL	    (0)
#define SYS_STATUS_REG	    (1)
#define CHG_CUR_CTL		    (2)
#define CHG_VOL_CTL		    (3)
#define CHG_PRO_CTL		    (4)

/******************************************************************************
* Register addresses end
******************************************************************************/

#define TIME_FAULT					(5)
#define SPRD_2700_TIME_FAULT		(0x38)
#define SPRD_2700_TIME_FAULT_SHIFT	(0x03)

#define CHG_CUR_BIT		            (0x0f)
#define CHG_TERMINA_CUR_BIT		    (0xE0)
#define CHG_TERMINA_CUR_SHIFT		(0x05)

#define USB_AIC_LIMIT_BIT           (0x18)
#define USB_AIC_LIMIT_SHIFT         (0x03)

#define USB_CVL_LIMIT_SHIFT         (0x00)
#define USB_CVL_LIMIT_BIT           (0x07)

#define CHG_EN_BIT					(0x40)
#define OTG_EN_BIT					(0x80)
#define OTG_EN_SHIFT				(0x07)

#define TERMINA_CUR_BIT				(0xe0)
#define CHG_STAT_BIT			    (0x0f)

#define FAULT_BIT                   (0x38)
#define FAULT_SHIFT				    (0x03)

#define CHG_CUR_SHIFT				(0x00)
#define CHG_OTG_EN_SHIFT            (0x07)

#define CHG_EN_SHIFT				(0x06)
#define TERMINA_CUR_SHIFT			(0x05)
#define CHG_STAT_SHIFT				(0x00)

#define CHG_OTG_VAL				    (0x01)
#define CHG_OTG_DISABLE_VAL	        (0x00)

#define CHG_VOL_LIMIT_BIT			(0x07)
#define CHG_VOL_LIMIT_SHIFT			(0x00)
#define OTGV_VOL_LIMIT_BIT          (0x3f)
#define OTGV_VOL_LIMIT_SHIFT        (0x06)

#define CHG_DISABLE_VAL			    (0x01)
#define CHG_BAT_VAL				    (0x00)

#define CHG_OTGVAL_5200				 5200
#define CHG_OTGVAL_5000				 5000
#define CHG_OTGVAL_4800				 4800


enum charge_status {
	CHG_SHUTDOWN = 0,
	CHG_RESET,
	CHG_PRECHARGE,
	CHG_CHGING,
	CHG_DONE,
	CHG_FAULT,
};

enum charge_fault {
	CHG_NO_FAULT = 0,
	CHG_VBUS_OVP,
	CHG_SLEEP_MODE,
	CHG_POOR_INPUT,
	CHG_VBAT_OVP,
	CHG_THM_SHUTDOWN,
	CHG_TIMER_FAULT,
	CHG_NO_BAT,
};

enum charge_config_type {
	CHG_DISABLE = 0,
	CHG_BATTERY,
	CHG_OTG,
};

struct sprd_2700 {
	struct i2c_client *client;
	struct work_struct chg_fault_irq_work;
};

struct sprd_2700_platform_data {
	uint16_t version;
	const struct sprd_ext_ic_operations *sprd_2700_ops;
	int irq_gpio_number;
};

extern void sprd_2700_init(void);
extern void sprd_2700_reset_timer(void);
extern void sprd_2700_sw_reset(void);
extern void sprd_2700_usb_aic_set(int usb_aic_vol);
extern void sprd_2700_sprdchg_cvl_set(unsigned int cvl_vol);
extern void sprd_2700_set_chg_current(unsigned int chg_cur);
extern void sprd_2700_termina_cur_set(unsigned int cur);
extern void sprd_2700_otg_enable(int enable);
extern void sprd_2700_stop_chg(void);
extern void sprd_2700_start_chg(void);
extern int sprd_2700_register_notifier(struct notifier_block *nb);
extern int sprd_2700_unregister_notifier(struct notifier_block *nb);
extern BYTE sprd_2700_get_sys_status(void);
extern BYTE sprd_2700_get_fault_val(void);
extern unsigned int sprd_2700_get_chg_current(void);
extern BYTE sprd_2700_get_chg_status(void);
const struct sprd_ext_ic_operations *sprd_get_ext_ic_ops(void);
#endif

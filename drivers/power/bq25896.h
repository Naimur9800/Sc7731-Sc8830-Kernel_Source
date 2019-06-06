#ifndef __LINUX_BQ25896_H
#define __LINUX_BQ25896_H

#include <linux/ioctl.h>
#include "sprd_battery.h"

/* Fault */
#define CHG_WATCHDOG_FAULT			(1)
#define BOOST_FAULT				(1)
#define INPUT_FAULT				(1)
#define CHG_SAFETY_TIMER_EXPIRE			(3)
#define CHG_VBAT_FAULT				(1)
#define CHG_COLD_FALUT				(5)
#define CHG_HOT_FALUT				(6)

/* Register definition */
#define REG00	(0)
#define REG01	(1)
#define REG02	(2)
#define REG03	(3)
#define REG04	(4)
#define REG05	(5)
#define REG06	(6)
#define REG07	(7)
#define REG08	(8)
#define REG09	(9)
#define REG0A	(10)
#define REG0B	(11)
#define REG0C	(12)
#define REG0D	(13)
#define REG0E	(14)
#define REG0F	(15)
#define REG10	(16)
#define REG11	(17)
#define REG12	(18)
#define REG13	(19)
#define REG14	(20)

/* BITS for REG00 */
#define EN_HIZ_BIT					(0x80)
#define EN_ILIM_BIT					(0x40)
#define IINLIM_BIT					(0x3f)
/* BITS for REG01 */
#define BHOT_BIT					(0xc0)
#define BCOLD_BIT					(0x20)
#define VINDPM_OS_BIT					(0x1f)
/* BITS for REG02 */
#define CONV_START_BIT					(0x80)
#define CONV_RATE_BIT				(0x40)
#define BOOST_FREQ_BIT				(0x20)
#define ICO_EN_BIT				(0x10)
#define FORCE_DPDM_BIT				(0x02)
#define AUTO_DPDM_EN_BIT			(0x01)
/* BITS for REG03 */
#define BAT_LOADEN_BIT			(0x80)
#define WD_RST_BIT				(0x40)
#define OTG_CONFIG_BIT				(0x20)
#define CHG_CONFIG_BIT			(0x10)
#define SYS_MIN_BIT			(0x0E)
#define MIN_VBAT_SEL_BIT		(0x01)
/* BITS for REG04 */
#define EN_PUMPX_BIT		(0x80)
#define ICHG_BIT			(0x7F)
/* BITS for REG05 */
#define IPRECHG_BIT		(0xF0)
#define ITERM_BIT			(0x0F)
/* BITS for REG06 */
#define VREG_BIT		(0xFC)
#define BATLOWV_BIT			(0x02)
#define VRECHG_BIT			(0x01)
/* BITS for REG07 */
#define EN_TERM_BIT		(0x80)
#define STAT_DS_BIT			(0x40)
#define WATCHDOG_BIT			(0x30)
#define EN_TIMER_BIT				(0X08)
#define CHG_TIMER_BIT				(0x06)
#define JEITA_ISET_BIT			(0x01)
/* BITS for REG08 TODO */
#define BAT_COMP_BIT			(0XE0)
#define VCLAMP_BIT				(0x1C)
#define TREG_BIT				(0x03)
/* BITS for REG09  */
#define BATFET_DIS_BIT			(0x20)
/* BITS for REG0A TODO */
/* BITS for REG0B */
#define VBUS_STAT_BIT			(0xe0)
#define CHRG_STAT_BIT			(0x18)
#define PG_STAT_BIT				(0x04)
#define VSYS_STAT_BIT					(0x01)
/* BITS for REG0C */
#define WATCHDOG_FAULT_BIT					(0x80)
#define BOOST_FAULT_BIT					(0x40)
#define CHRG_FAULT_BIT					(0x3F)
#define BAT_FAULT_BIT					(0x08)
#define NTC_FAULT_BIT					(0x07)
/* BITS for REG0D TODO */
/* BITS for REG0E TODO */
/* BITS for REG0F TODO */
/* BITS for REG10 TODO */
/* BITS for REG11 TODO */
/* BITS for REG12 TODO */
/* BITS for REG13 */
#define VDPM_STAT_BIT					(0x80)
#define IDPM_STAT_BIT					(0x40)
#define IDPM_LIM_BIT					(0x3F)
/* BITS for REG14 */
#define REG_RST_BIT					(0x80)

/* SHIFT for REG00 */
#define EN_HIZ_SHIFT					(0x07)
#define EN_ILIM_SHIFT					(0x06)
#define IINLIM_SHIFT					(0x00)
/* SHIFT for REG01 */
#define BHOT_SHIFT					(0x06)
#define BCOLD_SHIFT					(0x05)
#define VINDPM_OS_SHIFT					(0x00)
/* SHIFT for REG02 */
#define CONV_START_SHIFT					(0x07)
#define CONV_RATE_SHIFT				(0x06)
#define BOOST_FREQ_SHIFT				(0x05)
#define ICO_EN_SHIFT				(0x04)
#define FORCE_DPDM_SHIFT				(0x01)
#define AUTO_DPDM_EN_SHIFT			(0x00)
/* SHIFT for REG03 */
#define BAT_LOADEN_SHIFT			(0x07)
#define WD_RST_SHIFT				(0x06)
#define OTG_CONFIG_SHIFT				(0x05)
#define CHG_CONFIG_SHIFT			(0x04)
#define SYS_MIN_SHIFT			(0x01)
#define MIN_VBAT_SEL_SHIFT		(0x00)
/* SHIFT for REG04 */
#define EN_PUMPX_SHIFT		(0x07)
#define ICHG_SHIFT			(0x00)
/* SHIFT for REG05 */
#define IPRECHG_SHIFT		(0x04)
#define ITERM_SHIFT			(0x00)
/* SHIFT for REG06 */
#define VREG_SHIFT		(0x02)
#define BATLOWV_SHIFT			(0x01)
#define VRECHG_SHIFT			(0x00)
/* SHIFT for REG07 */
#define EN_TERM_SHIFT		(0x07)
#define STAT_DS_SHIFT			(0x06)
#define WATCHDOG_SHIFT			(0x04)
#define EN_TIMER_SHIFT				(0X03)
#define CHG_TIMER_SHIFT				(0x01)
#define JEITA_ISET_SHIFT			(0x00)
/* SHIFT for REG08 TODO */
#define BAT_COMP_SHIFT			(0X05)
#define VCLAMP_SHIFT			(0x02)
#define TREG_SHIFT				(0x00)
/* SHIFT for REG09  */
#define BATFET_DIS_SHIFT			(0x05)
/* SHIFT for REG0A TODO */
/* SHIFT for REG0B */
#define VBUS_STAT_SHIFT			(0x05)
#define CHRG_STAT_SHIFT			(0x03)
#define PG_STAT_SHIFT				(0x02)
#define VSYS_STAT_SHIFT					(0x00)
/* SHIFT for REG0C */
#define WATCHDOG_FAULT_SHIFT					(0x07)
#define BOOST_FAULT_SHIFT					(0x06)
#define CHRG_FAULT_SHIFT					(0x04)
#define BAT_FAULT_SHIFT					(0x03)
#define NTC_FAULT_SHIFT					(0x00)
/* SHIFT for REG0D TODO */
/* SHIFT for REG0E TODO */
/* SHIFT for REG0F TODO */
/* SHIFT for REG10 TODO */
/* SHIFT for REG11 TODO */
/* SHIFT for REG12 TODO */
/* SHIFT for REG13 */
#define VDPM_STAT_SHIFT					(0x07)
#define IDPM_STAT_SHIFT					(0x06)
#define IDPM_LIM_SHIFT					(0x00)
/* SHIFT for REG14 */
#define REG_RST_SHIFT					(0x07)

#define CHG_DISABLE_VAL			(0x00)
#define CHG_ENABLE_VAL			(0x01)
#define CHG_BAT_VAL				(0x01)
#define CHG_OTG_VAL				(0x01)

enum VBUS_STATUS {
	CHG_NO_INPUT = 0,
	CHG_SDP,
	CHG_ADAPTER,
	CHG_OTG = 0x111,
};

enum charge_status {
	CHG_NOT_CHGING = 0,
	CHG_PRE_CHGING,
	CHG_FAST_CHGING,
	CHG_TERMINA_DONE,
};

struct bq25896 {
	struct i2c_client *client;
	struct work_struct chg_fault_irq_work;
};

struct bq25896_platform_data {
	uint16_t version;
	struct sprd_ext_ic_operations *bq25896_ops;
	int irq_gpio_number;
	int vbat_detect;
};

extern void bq25896_reset_timer(void);
extern void  bq25896_sw_reset(void);
extern void bq25896_set_vindpm(unsigned char reg_val);
extern void bq25896_termina_cur_set(unsigned char reg_val);
extern void bq25896_termina_vol_set(unsigned char reg_val);
extern void bq25896_termina_time_set(unsigned char reg_val);
extern void bq25896_init(void);
extern void bq25896_otg_enable(int enable);
extern void bq25896_enable_flash(int enable);
extern void bq25896_set_flash_brightness(unsigned char  brightness);
extern void bq25896_enable_torch(int enable);
extern void bq25896_set_torch_brightness(unsigned char brightness);
extern void bq25896_stop_charging(unsigned int flag);
extern unsigned char bq25896_get_sys_status(void);
extern unsigned char bq25896_get_fault_val(void);
extern unsigned char bq25896_get_chgcur(void);
extern void bq25896_enable_chg(void);
extern void bq25896_set_chg_current_limit(uint32_t limit);
extern void bq25896_set_chg_current(unsigned char reg_val);
extern int bq25896_register_notifier(struct notifier_block *nb);
extern int bq25896_unregister_notifier(struct notifier_block *nb);
const struct sprd_ext_ic_operations *sprd_get_ext_ic_ops(void);
#endif

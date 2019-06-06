#ifndef __LINUX_SPRD2731_H
#define __LINUX_SPRD2731_H

#include "sprd_battery.h"

#define CHG_WDG_LOAD_LOW		(0x0000)
#define CHG_WDG_LOAD_HIGH		(0x0004)
#define CHG_WDG_CLR			(0x000C)
#define CHG_WDG_RAW_STATUS		(0x0010)
#define CHG_WDG_MSK_STATUS		(0x0014)
#define CHG_WDG_CNT_LOW			(0x0018)
#define CHG_WDG_CNT_HIGH		(0x001C)
#define CHG_WDG_LOCK			(0x0020)

#define SPRDBAT_CCCV_MAX		0x3F
#define ONE_CCCV_STEP_VOL		75

#define CHG_CFG0			(0x00)
#define CHG_CFG1			(0x04)
#define CHG_CFG2			(0x08)
#define CHG_CFG3			(0x0c)
#define CHG_CFG4			(0x10)
#define CHG_CFG5			(0x28)

#define SW_LED_EN			(0x248)
#define BST_INT_RAW			(0x240 + 0x2C)
#define BST_INT_CLR			(0x240 + 0x38)

/* RG_BST_CFG1 */
#define RG_BST_CFG1			0x25c
#define BST_CAL_MASK			GENMASK(4, 0)
#define BST_CAL_SHIFT			2
#define BITS_BST_CAL(x)			(((x) & BST_CAL_MASK) << BST_CAL_SHIFT)

/* BST_INT_RAW & BST_INT_CLR */
#define BIT_BST_UVI		(BIT(2))
#define BIT_OTG_SHTI		(BIT(1))
#define BIT_OTG_OCI		(BIT(0))

#define BIT_OTG_REG_EN			(BIT(4))
#define SWTCHG_APB_EN			BIT(5)

/* CHG_CFG0 */
#define BITS_PRECHG_RNG(x)		(((x) & 0x3) << 11)
#define BITS_TERM_VOL_CAL(x)		(((x) & 0x3f) << 3)
#define BITS_TERM_VOL_BIG(x)		(((x) & 0x3) << 1)
#define BITS_TERM_CUR(x)		((x) & 0x7)
#define SPRD2731_CC_EN			BIT(13)
#define SPRD2731_PD			BIT(0)

/* CHG_CFG1 */
#define BITS_INCUR_CAL(x)		(((x) & 0x1F) << 11)
#define BITS_OUTCUR_CAL(x)		(((x) & 0x1F) << 6)
#define SPRD2731_CUR_MASK		(0x1F)
#define SPRD2731_CUR_SHIFT		(0x0)
#define BITS_CHG_CUR(x)			((x) & 0x1f)

/* CHG_CFG2 */

#define SPRD2731_ITERM_MASK		(0x7)
#define SPRD2731_ITERM_SHIFT		(0x0)

#define CC_EN				BIT(15)

/* CHG_CFG5 */
#define BITS_ICRSET(x)			(((x) & 0x3) << 8)

struct sprdchg_2731_data {
	unsigned long base_addr;
	struct regmap *reg_map;
	unsigned int cccv_cal;
	struct delayed_work otg_work;
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

enum charge_status {
	CHG_READY = 0,
	CHG_CHGING,
	CHG_DONE,
	CHG_FAULT,
};

extern void sprdchg_reset_wdt(int sec);
extern void sprdchg_wdt_stop(void);
#endif

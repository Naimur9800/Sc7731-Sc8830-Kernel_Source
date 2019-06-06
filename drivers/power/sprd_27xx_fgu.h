#ifndef _SPRD_2723_FGU_H_
#define _SPRD_2723_FGU_H_

#include <linux/power_supply.h>
#include <linux/sprd_battery_common.h>
#include <linux/types.h>
#include <linux/wakelock.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
/* registers definitions for controller REGS_FGU */
#define REG_FGU_START                    (0x0000)
#define REG_FGU_CONFIG                  (0x0004)
#define REG_FGU_ADC_CONFIG       (0x0008)
#define REG_FGU_STATUS                 (0x000c)
#define REG_FGU_INT_EN                  (0x0010)
#define REG_FGU_INT_CLR                 (0x0014)
#define REG_FGU_INT_RAW                 (0x0018)
#define REG_FGU_INT_STS                (0x001c)
#define REG_FGU_VOLT_VAL                (0x0020)
#define REG_FGU_OCV_VAL            (0x0024)
#define REG_FGU_POCV_VAL                (0x0028)
#define REG_FGU_CURT_VAL                (0x002c)
#define REG_FGU_HIGH_OVER               (0x0030)
#define REG_FGU_LOW_OVER               (0x0034)
#define REG_FGU_VTHRE_HH               (0x0038)
#define REG_FGU_VTHRE_HL                (0x003c)
#define REG_FGU_VTHRE_LH               (0x0040)
#define REG_FGU_VTHRE_LL                (0x0044)
#define REG_FGU_OCV_LOCKLO              (0x0048)
#define REG_FGU_OCV_LOCKHI             (0x004c)
#define REG_FGU_CLBCNT_SETH            (0x0050)
#define REG_FGU_CLBCNT_SETL            (0x0054)
#define REG_FGU_CLBCNT_DELTH          (0x0058)
#define REG_FGU_CLBCNT_DELTL           (0x005c)
#define REG_FGU_CLBCNT_LASTOCVH       (0x0060)
#define REG_FGU_CLBCNT_LASTOCVL       (0x0064)
#define REG_FGU_CLBCNT_VALH            (0x0068)
#define REG_FGU_CLBCNT_VALL             (0x006c)
#define REG_FGU_CLBCNT_QMAXH            (0x0070)
#define REG_FGU_CLBCNT_QMAXL            (0x0074)
#define REG_FGU_QMAX_TOSET        (0x0078)
#define REG_FGU_QMAX_TIMER          (0x007c)
#define REG_FGU_RELAX_CURT_THRE         (0x0080)
#define REG_FGU_RELAX_CNT_THRE         (0x0084)
#define REG_FGU_RELAX_CNT               (0x0088)
#define REG_FGU_OCV_LAST_CNT            (0x008c)
#define REG_FGU_CURT_OFFSET            (0x0090)
#define REG_FGU_USER_AREA_SET		(0x00A0)
#define REG_FGU_USER_AREA_CLEAR		(0x00A4)
#define REG_FGU_USER_AREA_STATUS	(0x00A8)
#define REG_FGU_USER_AREA_SET1		(0x00C0)
#define REG_FGU_USER_AREA_CLEAR1	(0x00C4)
#define REG_FGU_USER_AREA_STATUS1	(0x00C8)
#define REG_FGU_VOLT_VALUE_BUF0		(0x00D0)
#define REG_FGU_CURT_VALUE_BUF0		(0x00F0)
#define BITS_POWERON_TYPE(_x_)           ((_x_) << 12 & (0xF000))
#define BITS_RTC_AREA(_x_)           ((_x_) << 0 & (0xFFF))
/* bits definitions for register REG_FGU_START */
#define BIT_QMAX_UPDATE_EN              (BIT(2))
#define BIT_FGU_RESET                   (BIT(1))
#define BIT_WRITE_SELCLB_EN             (BIT(0))
/* bits definitions for register REG_FGU_CONFIG */
#define BIT_VOLT_H_VALID                (BIT(12))
#define BIT_FGU_DISABLE_EN              (BIT(11))
#define BIT_CLBCNT_DELTA_MODE           (BIT(10))
#define BITS_ONEADC_DUTY(_x_)           ((_x_) << 8 & (BIT(8)|BIT(9)))
#define BIT_CURT_DUTY                   (BIT(7))
#define BITS_VOLT_DUTY(_x_)             ((_x_) << 5 & (BIT(5)|BIT(6)))
#define BIT_AD1_ENABLE                  (BIT(4))
#define BIT_SW_DIS_CURT                 (BIT(3))
#define BIT_FORCE_LOCK_EN               (BIT(2))
#define BIT_LOW_POWER_MODE              (BIT(1))
#define BIT_AUTO_LOW_POWER              (BIT(0))
/* bits definitions for register REG_FGU_ADC_CONFIG */
#define BIT_FORCE_AD1_VIN_EN            (BIT(7))
#define BIT_FORCE_AD0_VIN_EN            (BIT(6))
#define BIT_FORCE_AD0_IIN_EN            (BIT(5))
#define BIT_FORCE_AD_EN                 (BIT(4))
#define BIT_AD1_VOLT_REF                (BIT(3))
#define BIT_AD0_VOLT_REF                (BIT(2))
#define BIT_AD01_RESET                  (BIT(1))
#define BIT_AD01_PD                     (BIT(0))
/* bits definitions for register REG_FGU_STATUS */
#define BIT_POWER_LOW                   (BIT(5))
#define BIT_CURT_LOW                    (BIT(4))
#define BITS_OCV_LOCK_STS(_x_)          ((_x_) << 2 & (BIT(2)|BIT(3)))
#define BIT_QMAX_UPDATE_STS             (BIT(1))
#define BIT_WRITE_ACTIVE_STS            (BIT(0))
/* bits definitions for register REG_FGU_INT_EN */
#define BIT_CURT_RDEN_INT               (BIT(7))
#define BIT_VOLT_RDEN_INT               (BIT(6))
#define BIT_QMAX_UPD_TOUT               (BIT(5))
#define BIT_QMAX_UPD_DONE               (BIT(4))
#define BIT_RELX_CNT_INT                (BIT(3))
#define BIT_CLBCNT_DELTA_INT            (BIT(2))
#define BIT_VOLT_HIGH_INT               (BIT(1))
#define BIT_VOLT_LOW_INT                (BIT(0))
#define BITS_VOLT_VALUE(_x_) \
	((_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3) \
	BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|\
	BIT(10)|BIT(11)|BIT(12)))
/* bits definitions for register REG_FGU_CURT_VAL */
#define BITS_CURT_VALUE(_x_) \
	((_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|\
	BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|\
	BIT(10)|BIT(11)|BIT(12)|BIT(13)))
/* bits definitions for register REG_FGU_CLBCNT_SETH */
#define BITS_CLBCNT_SETH(_x_) \
	((_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|\
	BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|\
	BIT(10)|BIT(11)|BIT(12)|BIT(13)))
/* bits definitions for register REG_FGU_CLBCNT_SETL */
#define BITS_CLBCNT_SETL(_x_) \
	((_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|\
	BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|\
	BIT(10)|BIT(11)|BIT(12)|BIT(13)|BIT(14)|BIT(15)))
/* bits definitions for register REG_FGU_CLBCNT_DELTHAH */
#define BITS_CLBCNT_DELTHH(_x_) \
	((_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|\
	BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|\
	BIT(10)|BIT(11)|BIT(12)|BIT(13)))
/* bits definitions for register REG_FGU_CLBCNT_DELTAL */
#define BITS_CLBCNT_DELTHL(_x_) \
	((_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|\
	BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|\
	BIT(10)|BIT(11)|BIT(12)|BIT(13)|BIT(14)|BIT(15)))
/* bits definitions for register REG_FGU_RELAX_CURT_THRE */
#define BITS_RELAX_CUR_THRE(_x_) \
	((_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|\
	BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|\
	BIT(10)|BIT(11)|BIT(12)|BIT(13)))
/* bits definitions for register REG_FGU_RELAX_CNT_THRE */
#define BITS_RELAX_CNT_THRE(_x_) \
	((_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|\
	BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|\
	BIT(10)|BIT(11)|BIT(12)))
/* bits definitions for register REG_FGU_RELAX_CNT */
#define BITS_RELAX_CNT_VAL(_x_) \
	((_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|\
	BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|\
	BIT(10)|BIT(11)|BIT(12)))
/* bits definitions for register REG_FGU_OCV_LAST_CNT */
#define BITS_OCV_LAST_CNT(_x_) \
	((_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|\
	BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|\
	BIT(10)|BIT(11)|BIT(12)))
/* bits definitions for register REG_FGU_CURT_OFFSET */
#define BITS_CURT_OFFSET_VAL(_x_) \
	((_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|\
	BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|\
	BIT(10)|BIT(11)|BIT(12)|BIT(13)))

#define SPRDFGU_OCV_VALID_TIME    20
#define CUR_0ma_IDEA_ADC    8192
#define FGU_CUR_SAMPLE_HZ   2
#define FIRST_POWERTON  0xF
#define NORMAIL_POWERTON  0x5
#define WDG_POWERTON  0xA

#define CUR_VOL_BUFF_LEN 8
#define BUFF_LEN_2731 (CUR_VOL_BUFF_LEN)

enum fgu_type {
	FGU_UNKNOWN,
	FGU_2723,
	FGU_2731,
	FGU_2721,
	FGU_2720,
};

struct sprdfgu_platform_data {
	uint32_t fgu_irq;
	uint32_t ocv_type;
	uint32_t fgu_type;
	unsigned long base_addr;
};

struct sprdfgu_qmax {
	int state;
	int cur_cnom;
	int s_clbcnt;
	int s_soc;
	int s_vol;
	int s_cur;
	int e_vol;
	int e_cur;
	int force_s_flag;
	int force_s_soc;
	uint32_t timeout;
	__s64 s_time;
};

struct sprdfgu_tracking {
	__s64 r_time;
	__s64 query_time;
	int rint;
	int r_temp;
	int c_temp;
	int r_vol;
	int relax_vol;
	int relax_cur;
	int ave_vol;
	int ave_cur;
	int *cur_buff;
	int *vol_buff;
};

struct sprdfgu_drivier_data {
	struct sprd_battery_platform_data *pdata;
	struct sprdfgu_platform_data *fgu_pdata;
	struct device *dev;
	int adp_status;
	int warning_cap;
	int shutdown_vol;
	int bat_full_vol;
	int cur_rint;
	int cur_cnom;
	int poweron_rint;
	int init_cap;
	int init_clbcnt;
	int poweron_cap;
	unsigned int int_status;
	unsigned int is_track;
	struct sprdfgu_qmax qmax;
	struct sprdfgu_tracking track;
	struct delayed_work fgu_irq_work;
	struct power_supply *sprdfgu;
	struct mutex lock;
	struct mutex track_lock;
	struct wake_lock low_power_lock;
	struct workqueue_struct *fgu_wqueue;
	struct delayed_work fgu_qmax_work;
};

struct sprdfgu_cal {
	int cur_1000ma_adc;
	int vol_1000mv_adc;
	int cur_offset;
	int vol_offset;
	int cal_type;
};

#endif

/* Lite-On LTR-553ALS Android / Linux Driver
 *
 * Copyright (C) 2013 Lite-On Technology Corp (Singapore)
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */


#include <linux/delay.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <linux/version.h>
#include <ontim/light_prox/als_ps.h>
extern bool proximity_probe_ok;//bit4 add by liuwei
/* LTR-553 Registers */
#define LTR553_ALS_CONTR	0x80
#define LTR553_PS_CONTR		0x81
#define LTR553_PS_LED		0x82
#define LTR553_PS_N_PULSES	0x83
#define LTR553_PS_MEAS_RATE	0x84
#define LTR553_ALS_MEAS_RATE	0x85
#define LTR553_PART_ID		0x86
#define LTR553_MANUFACTURER_ID	0x87
#define LTR553_ALS_DATA_CH1_0	0x88
#define LTR553_ALS_DATA_CH1_1	0x89
#define LTR553_ALS_DATA_CH0_0	0x8A
#define LTR553_ALS_DATA_CH0_1	0x8B
#define LTR553_ALS_PS_STATUS	0x8C
#define LTR553_PS_DATA_0	0x8D
#define LTR553_PS_DATA_1	0x8E
#define LTR553_INTERRUPT	0x8F
#define LTR553_PS_THRES_UP_0	0x90
#define LTR553_PS_THRES_UP_1	0x91
#define LTR553_PS_THRES_LOW_0	0x92
#define LTR553_PS_THRES_LOW_1	0x93
#define LTR553_PS_OFFSET_1	0x94
#define LTR553_PS_OFFSET_0	0x95
#define LTR553_ALS_THRES_UP_0	0x97
#define LTR553_ALS_THRES_UP_1	0x98
#define LTR553_ALS_THRES_LOW_0	0x99
#define LTR553_ALS_THRES_LOW_1	0x9A
#define LTR553_INTERRUPT_PRST	0x9E
/* LTR-553 Registers */


#define SET_BIT 1
#define CLR_BIT 0

#define ALS 0
#define PS 1
#define ALSPS 2

//#define PS_W_SATURATION_BIT	3

/* address 0x80 */
#define ALS_MODE_ACTIVE	(1 << 0)
#define ALS_MODE_STDBY		(0 << 0)
#define ALS_SW_RESET		(1 << 1)
#define ALS_SW_NRESET		(0 << 1)
#define ALS_GAIN_1x		(0 << 2)
#define ALS_GAIN_2x		(1 << 2)
#define ALS_GAIN_4x		(2 << 2)
#define ALS_GAIN_8x		(3 << 2)
#define ALS_GAIN_48x	(6 << 2)
#define ALS_GAIN_96x	(7 << 2)
#define ALS_MODE_RDBCK			0
#define ALS_SWRT_RDBCK			1
#define ALS_GAIN_RDBCK			2
#define ALS_CONTR_RDBCK		3

/* address 0x81 */
#define PS_MODE_ACTIVE		(3 << 0)
#define PS_MODE_STDBY		(0 << 0)
#define PS_GAIN_16x			(0 << 2)
#define PS_GAIN_32x			(2 << 2)
#define PS_GAIN_64x			(3 << 2)
#define PS_SATUR_INDIC_EN	(1 << 5)
#define PS_SATU_INDIC_DIS	(0 << 5)
#define PS_MODE_RDBCK		0
#define PS_GAIN_RDBCK		1
#define PS_SATUR_RDBCK		2
#define PS_CONTR_RDBCK		3

/* address 0x82 */
#define LED_CURR_5MA		(0 << 0)
#define LED_CURR_10MA		(1 << 0)
#define LED_CURR_20MA		(2 << 0)
#define LED_CURR_50MA		(3 << 0)
#define LED_CURR_100MA		(4 << 0)
#define LED_CURR_DUTY_25PC		(0 << 3)
#define LED_CURR_DUTY_50PC		(1 << 3)
#define LED_CURR_DUTY_75PC		(2 << 3)
#define LED_CURR_DUTY_100PC	(3 << 3)
#define LED_PUL_FREQ_30KHZ		(0 << 5)
#define LED_PUL_FREQ_40KHZ		(1 << 5)
#define LED_PUL_FREQ_50KHZ		(2 << 5)
#define LED_PUL_FREQ_60KHZ		(3 << 5)
#define LED_PUL_FREQ_70KHZ		(4 << 5)
#define LED_PUL_FREQ_80KHZ		(5 << 5)
#define LED_PUL_FREQ_90KHZ		(6 << 5)
#define LED_PUL_FREQ_100KHZ	(7 << 5)
#define LED_CURR_RDBCK			0
#define LED_CURR_DUTY_RDBCK	1
#define LED_PUL_FREQ_RDBCK		2
#define PS_LED_RDBCK			3

/* address 0x84 */
#define PS_MEAS_RPT_RATE_50MS		(0 << 0)
#define PS_MEAS_RPT_RATE_70MS		(1 << 0)
#define PS_MEAS_RPT_RATE_100MS	(2 << 0)
#define PS_MEAS_RPT_RATE_200MS	(3 << 0)
#define PS_MEAS_RPT_RATE_500MS	(4 << 0)
#define PS_MEAS_RPT_RATE_1000MS	(5 << 0)
#define PS_MEAS_RPT_RATE_2000MS	(6 << 0)
#define PS_MEAS_RPT_RATE_10MS		(8 << 0)

/* address 0x85 */
#define ALS_MEAS_RPT_RATE_50MS	(0 << 0)
#define ALS_MEAS_RPT_RATE_100MS	(1 << 0)
#define ALS_MEAS_RPT_RATE_200MS	(2 << 0)
#define ALS_MEAS_RPT_RATE_500MS	(3 << 0)
#define ALS_MEAS_RPT_RATE_1000MS	(4 << 0)
#define ALS_MEAS_RPT_RATE_2000MS	(5 << 0)
#define ALS_INTEG_TM_100MS		(0 << 3)
#define ALS_INTEG_TM_50MS			(1 << 3)
#define ALS_INTEG_TM_200MS		(2 << 3)
#define ALS_INTEG_TM_400MS		(3 << 3)
#define ALS_INTEG_TM_150MS		(4 << 3)
#define ALS_INTEG_TM_250MS		(5 << 3)
#define ALS_INTEG_TM_300MS		(6 << 3)
#define ALS_INTEG_TM_350MS		(7 << 3)
#define ALS_MEAS_RPT_RATE_RDBCK	0
#define ALS_INTEG_TM_RDBCK			1
#define ALS_MEAS_RATE_RDBCK		2

/* address 0x86 */
#define PART_NUM_ID_RDBCK		0
#define REVISION_ID_RDBCK		1
#define PART_ID_REG_RDBCK		2

/* address 0x8C */
#define PS_DATA_STATUS_RDBCK		0
#define PS_INTERR_STATUS_RDBCK	1
#define ALS_DATA_STATUS_RDBCK		2
#define ALS_INTERR_STATUS_RDBCK	3
#define ALS_GAIN_STATUS_RDBCK		4
#define ALS_VALID_STATUS_RDBCK	5
#define ALS_PS_STATUS_RDBCK		6

/* address 0x8F */
#define INT_MODE_00					(0 << 0)
#define INT_MODE_PS_TRIG			(1 << 0)
#define INT_MODE_ALS_TRIG			(2 << 0)
#define INT_MODE_ALSPS_TRIG		(3 << 0)
#define INT_POLAR_ACT_LO			(0 << 2)
#define INT_POLAR_ACT_HI			(1 << 2)
#define INT_MODE_RDBCK				0
#define INT_POLAR_RDBCK			1
#define INT_INTERRUPT_RDBCK		2

/* address 0x9E */
#define ALS_PERSIST_SHIFT	0
#define PS_PERSIST_SHIFT	4
#define ALS_PRST_RDBCK		0
#define PS_PRST_RDBCK		1
#define ALSPS_PRST_RDBCK	2

#define PON_DELAY		600

#define ALS_MIN_MEASURE_VAL	0
#define ALS_MAX_MEASURE_VAL	65535
#define ALS_VALID_MEASURE_MASK	ALS_MAX_MEASURE_VAL
#define PS_MIN_MEASURE_VAL	0
#define PS_MAX_MEASURE_VAL	2047
#define PS_VALID_MEASURE_MASK   PS_MAX_MEASURE_VAL
#define LO_LIMIT			0
#define HI_LIMIT			1
#define LO_N_HI_LIMIT	2
#define PS_OFFSET_MIN_VAL		0
#define PS_OFFSET_MAX_VAL		1023
#define 	FAR_VAL		10
#define 	NEAR_VAL		0

#define DRIVER_VERSION "1.14"
#define PARTID 0x92
#define MANUID 0x05

#define I2C_RETRY 5

#define DEVICE_NAME "als_prox"

#define ACT_INTERRUPT 1
#define AGC		1
#define PS_Dyn_Calib 	0

/*
 * Magic Number
 * ============
 * Refer to file ioctl-number.txt for allocation
 */
#define LTR553_IOCTL_MAGIC      'c'

/* IOCTLs for LTR553 device */
#define LTR553_IOCTL_PS_ENABLE		_IOR(LTR553_IOCTL_MAGIC, 1, int *)
#define LTR553_IOCTL_PS_GET_ENABLED	_IOW(LTR553_IOCTL_MAGIC, 2, int *)
#define LTR553_IOCTL_ALS_ENABLE		_IOR(LTR553_IOCTL_MAGIC, 3, int *)
#define LTR553_IOCTL_ALS_GET_ENABLED	_IOW(LTR553_IOCTL_MAGIC, 4, int *)

//(Linux RTOS)>
#if 0
struct LTR553_platform_data {
	/* ALS */
	u16 pfd_levels[5];
	u16 pfd_als_lowthresh;
	u16 pfd_als_highthresh;
	int pfd_disable_als_on_suspend;

	/* PS */
	u16 pfd_ps_lowthresh;
	u16 pfd_ps_highthresh;
	int pfd_disable_ps_on_suspend;

	/* Interrupt */
	int pfd_gpio_int_no;
};
#endif
//(Linux RTOS)<


struct LTR553_data {
	/* Device */
	struct i2c_client *i2c_client;
	struct input_dev *als_input_dev;
	struct input_dev *ps_input_dev;
	struct workqueue_struct *workqueue;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	struct wake_lock ps_wake_lock;
	struct mutex bus_lock;

	/* Device mode
	 * 0 = ALS
	 * 1 = PS
	 */
	uint8_t mode;

	/* ALS */
	uint8_t als_enable_flag;
	uint8_t als_suspend_enable_flag;
	uint8_t als_irq_flag;
	uint8_t als_opened;
	u16 als_lowthresh;
	u16 als_highthresh;
	u16 default_als_lowthresh;
	u16 default_als_highthresh;
	u16 *adc_levels;
	/* Flag to suspend ALS on suspend or not */
	uint8_t disable_als_on_suspend;

	/* PS */
	uint8_t ps_enable_flag;
	uint8_t ps_suspend_enable_flag;
	uint8_t ps_irq_flag;
	uint8_t ps_opened;
	u16 ps_lowthresh;
	u16 ps_highthresh;
	u16 default_ps_lowthresh;
	u16 default_ps_highthresh;
	/* Flag to suspend PS on suspend or not */
	uint8_t disable_ps_on_suspend;

	/* LED */
	int led_pulse_freq;
	int led_duty_cyc;
	int led_peak_curr;
	int led_pulse_count;

	/* Interrupt */
	int irq;
	u8 gpio_int_no;
	int is_suspend;
//+add by hzb	
	int (*power_on)(int on);
//-add by hzb
};

static struct LTR553_data *sensor_info;

#define	PS_MAX_INIT_KEPT_DATA_COUNTER		8
#define	PS_MAX_MOV_AVG_KEPT_DATA_CTR		7

u16 winfac1 = 100;
u16 winfac2 = 80;
u16 winfac3 = 44;
uint8_t eqn_prev = 0;
uint8_t ratio_old = 0;
u16 ps_init_kept_data[PS_MAX_INIT_KEPT_DATA_COUNTER];
u16 ps_ct_avg;
uint8_t ps_grabData_stage = 0;
uint32_t ftn_init;
uint32_t ftn_final;
uint32_t ntf_final;
u16 lux_val_prev = 0;
uint8_t ps_kept_data_counter = 0;
u16 ps_movavg_data[PS_MAX_MOV_AVG_KEPT_DATA_CTR];
uint8_t ps_movavg_data_counter = 0;
u16 ps_movct_avg;
//u16 ps_thresh_hi, ps_thresh_lo;

#include <ontim/ontim_dev_dgb.h>
static char ltr553_prox_version[]="ltr553_1.0";
static char ltr553_prox_vendor_name[20]="ltr553";
u16 ltr553_prox_min = 0;
u16 ltr553_prox_max = 0;
u16 ltr553_prox_test = 0;
    DEV_ATTR_DECLARE(als_prox)
    DEV_ATTR_DEFINE("version",ltr553_prox_version)
    DEV_ATTR_DEFINE("vendor",ltr553_prox_vendor_name)
    DEV_ATTR_VAL_DEFINE("prox_min",&ltr553_prox_min,ONTIM_DEV_ARTTR_TYPE_VAL_16BIT)
    DEV_ATTR_VAL_DEFINE("prox_max",&ltr553_prox_max,ONTIM_DEV_ARTTR_TYPE_VAL_16BIT)
    DEV_ATTR_VAL_DEFINE("prox_test",&ltr553_prox_test,ONTIM_DEV_ARTTR_TYPE_VAL_16BIT)
    DEV_ATTR_DECLARE_END;
    ONTIM_DEBUG_DECLARE_AND_INIT(als_prox,als_prox,8);


/* I2C Read */
// take note --------------------------------------- 
// for i2c read, need to send the register address follwed by buffer over to register.
// There should not be a stop in between register address and buffer.  
// There should not be release of lock in between register address and buffer. 
// take note ---------------------------------------
static int8_t I2C_Read(uint8_t *rxData, uint8_t length)
{
	int8_t index;
	struct i2c_msg data[] = {
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (index = 0; index < I2C_RETRY; index++) {
		if (i2c_transfer(sensor_info->i2c_client->adapter, data, 2) > 0)
			break;

		mdelay(10);
	}

	if (index >= I2C_RETRY) {
		pr_alert("%s I2C Read Fail !!!!\n",__func__);
		return -EIO;
	}

	return 0;
}


/* I2C Write */
static int8_t I2C_Write(uint8_t *txData, uint8_t length)
{
	int8_t index;
	struct i2c_msg data[] = {
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	for (index = 0; index < I2C_RETRY; index++) {
		if (i2c_transfer(sensor_info->i2c_client->adapter, data, 1) > 0)
			break;

		mdelay(10);
	}

	if (index >= I2C_RETRY) {
		pr_alert("%s I2C Write Fail !!!!\n", __func__);
		return -EIO;
	}

	return 0;
}


/* Set register bit */
static int8_t _LTR553_set_bit(struct i2c_client *client, uint8_t set, uint8_t cmd, uint8_t data)
{
	uint8_t buffer[2];
	uint8_t value;
	int8_t ret = 0;

	buffer[0] = cmd;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (set)
		value |= data;
	else
		value &= ~data;

	buffer[0] = cmd;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return -EIO;
	}

	return ret;
}


static u16 lux_formula(u16 ch0_adc, u16 ch1_adc, uint8_t eqtn)
{
	uint32_t luxval = 0;
	uint32_t luxval_i = 0;
	uint32_t luxval_f = 0;
	u16 ch0_coeff_i = 0;
	u16 ch1_coeff_i = 0;
	u16 ch0_coeff_f = 0;
	u16 ch1_coeff_f = 0;
	int8_t ret; 
	uint8_t gain = 1, als_int_fac;
	uint8_t buffer[2];
	u16 win_fac = 0;
	int8_t fac = 1;

	buffer[0] = LTR553_ALS_PS_STATUS;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	gain = (buffer[0] & 0x70);
	gain >>= 4;

	if (gain == 0) {			//gain 1
		gain = 1;
	} else if (gain == 1) {		//gain 2
		gain = 2;
	} else if (gain == 2) {		//gain 4
		gain = 4;
	} else if (gain == 3) {		//gain 8
		gain = 8;
	} else if (gain == 6) {		//gain 48
		gain = 48;
	} else if (gain == 7) {		//gain 96
		gain = 96;
	}

	buffer[0] = LTR553_ALS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	als_int_fac = buffer[0] & 0x38;
	als_int_fac >>= 3;

	if (als_int_fac == 0) {
		als_int_fac = 10;
	} else if (als_int_fac == 1) {
		als_int_fac = 5;
	} else if (als_int_fac == 2) {
		als_int_fac = 20;
	} else if (als_int_fac == 3) {
		als_int_fac = 40;
	} else if (als_int_fac == 4) {
		als_int_fac = 15;
	} else if (als_int_fac == 5) {
		als_int_fac = 25;
	} else if (als_int_fac == 6) {
		als_int_fac = 30;
	} else if (als_int_fac == 7) {
		als_int_fac = 35;
	}

	if (eqtn == 1) {
		ch0_coeff_i = 1;
		ch1_coeff_i = 1;
		ch0_coeff_f = 7743;
		ch1_coeff_f = 1059;
		fac = 1;
		win_fac = winfac1;
		luxval_i = ((ch0_adc * ch0_coeff_i) + (ch1_adc * ch1_coeff_i)) * win_fac;
		luxval_f = (((ch0_adc * ch0_coeff_f) + (ch1_adc * ch1_coeff_f)) / 100) * win_fac;
		//luxval = ((17743 * ch0_calc) + (11059 * ch1_adc));
		//luxval = ((1.7743 * ch0_calc) + (1.1059 * ch1_adc)) / (gain * (als_int_fac / 10));
	} else if (eqtn == 2) {
		ch0_coeff_i = 4;
		ch1_coeff_i = 1;
		ch0_coeff_f = 2785;
		ch1_coeff_f = 696;
		win_fac = winfac2;
		if ((ch1_coeff_f * ch1_adc) < (ch0_adc * ch0_coeff_f)) {
			fac = 1;
			luxval_f = (((ch0_adc * ch0_coeff_f) - (ch1_adc * ch1_coeff_f)) / 100) * win_fac;
		} else {
			fac = -1;
			luxval_f = (((ch1_adc * ch1_coeff_f) - (ch0_adc * ch0_coeff_f)) / 100) * win_fac;
		}
		luxval_i = ((ch0_adc * ch0_coeff_i) - (ch1_adc * ch1_coeff_i)) * win_fac;
		//luxval = ((42785 * ch0_calc) - (10696 * ch1_adc));
		//luxval = ((4.2785 * ch0_calc) - (1.9548 * ch1_adc)) / (gain * (als_int_fac / 10));
	} else if (eqtn == 3) {
		ch0_coeff_i = 0;
		ch1_coeff_i = 0;
		ch0_coeff_f = 5926;
		ch1_coeff_f = 1300;
		fac = 1;
		win_fac = winfac3;
		luxval_i = ((ch0_adc * ch0_coeff_i) + (ch1_adc * ch1_coeff_i)) * win_fac;
		luxval_f = (((ch0_adc * ch0_coeff_f) + (ch1_adc * ch1_coeff_f)) / 100) * win_fac;
		//luxval = ((5926 * ch0_calc) + (1185 * ch1_adc));
		//luxval = ((0.5926 * ch0_calc) + (0.1185 * ch1_adc)) / (gain * (als_int_fac / 10));
	} else if (eqtn == 4) {
		ch0_coeff_i = 0;
		ch1_coeff_i = 0;
		ch0_coeff_f = 0;
		ch1_coeff_f = 0;
		fac = 1;
		luxval_i = 0;
		luxval_f = 0;
		//luxval = 0;
	}

    if (fac < 0) {
        	luxval = (luxval_i  - (luxval_f / 100)) / (gain * als_int_fac);                              
    } else {
        luxval = (luxval_i  + (luxval_f / 100)) / (gain * als_int_fac);
    }
    printk("[ltr553]<1><-asl_int_fac %d ,fac %d ,win_fac %d ,gain %d ,eqtn %d ,ch0_coeff_i %d ,ch0_coeff_f %d ->\n",
          als_int_fac, fac, win_fac ,gain ,eqtn ,ch0_coeff_i ,ch0_coeff_f);
    printk("[ltr553]<2><--ch1_coeff_i %d ,ch1_coeff_f %d ,luxval_i %d ,luxval_f %d -->\n",
             ch1_coeff_i ,ch1_coeff_f ,luxval_i ,luxval_f);
    return luxval;
}


static u16 ratioHysterisis (u16 ch0_adc, u16 ch1_adc)
{
    #define	RATIO_HYSVAL	10
    int ratio;
	uint8_t buffer[2], eqn_now;
	int8_t ret;
	u16 ch0_calc;
	uint32_t luxval = 0;
	int abs_ratio_now_old;

	buffer[0] = LTR553_ALS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	ch0_calc = ch0_adc;
	if ((buffer[0] & 0x20) == 0x20) {
		ch0_calc = ch0_adc - ch1_adc;
	}

	if ((ch1_adc + ch0_calc) == 0) {
		ratio = 100;
	} else {
		ratio = (ch1_adc*100) / (ch1_adc + ch0_calc);
	}

	if (ratio < 45) {
		eqn_now = 1;
	} else if ((ratio >= 45) && (ratio < 68)) {
		eqn_now = 2;
	} else if ((ratio >= 68) && (ratio < 99)) {
		eqn_now = 3;
	} else if (ratio >= 99) {
		eqn_now = 4;
	}

	if (eqn_prev == 0 || (ratio > 40 && ratio < 55 ) ) {
		luxval = lux_formula(ch0_calc, ch1_adc, eqn_now);
		ratio_old = ratio;
		eqn_prev = eqn_now;
	} else {
		if (eqn_now == eqn_prev) {
			luxval = lux_formula(ch0_calc, ch1_adc, eqn_now);
			ratio_old = ratio;
			eqn_prev = eqn_now;
		} else {
			abs_ratio_now_old = ratio - ratio_old;
			if (abs_ratio_now_old < 0) {
				abs_ratio_now_old *= (-1);
			}
			if (abs_ratio_now_old > RATIO_HYSVAL) {
				luxval = lux_formula(ch0_calc, ch1_adc, eqn_now);
				ratio_old = ratio;
				eqn_prev = eqn_now;
			} else {
				luxval = lux_formula(ch0_calc, ch1_adc, eqn_prev);
			}
		}
	}
    printk("[ltr553]<3><---ch0_adc %d ,ch1_adc %d ,(ratio %d%% eqn_now %d eqn_prev %d) ,luxval %d --->\n",
          ch0_adc, ch1_adc, ratio, eqn_now, eqn_prev, luxval);
	return luxval;
}


#if 0
void setWinFac (u16 *winfac, uint8_t *param_temp, size_t count)
{
	int winfacVal = 0;
	uint8_t u_ctr, deci_ctr = 0;
	
	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
	} else if (count == 2) { 				// 1 character
		if (param_temp[0] == '.') {
			param_temp[0] = 0;
		} else {
			param_temp[0] -= 48;
		}
		winfacVal = param_temp[0] * 10;
	} else if (count == 3) { 				// 2 characters
		for (u_ctr = 0; u_ctr < 2; u_ctr++) {
			if (param_temp[u_ctr] == '.')
				deci_ctr++;
		}

		if (deci_ctr <= 1) {
			if (param_temp[0] == '.') {
				winfacVal = param_temp[1] - 48;
			} else if (param_temp[1] == '.') {
				winfacVal = param_temp[0] - 48;
				winfacVal *= 10;
			} else {
				param_temp[0] -= 48;
				param_temp[1] -= 48;
				winfacVal = (param_temp[0] * 10) + param_temp[1];
				winfacVal *= 10;
			}
		}				
	} else if (count == 4) { 				// 3 characters
		for (u_ctr = 0; u_ctr < 3; u_ctr++) {
			if (param_temp[u_ctr] == '.')
				deci_ctr++;
		}

		if (deci_ctr <= 1) {
			if (param_temp[0] == '.') {
				param_temp[1] -= 48;
				param_temp[2] -= 48;
				if (param_temp[2] >= 5) {
					if ((param_temp[1] >=0) && (param_temp[1] <= 8)) {
						param_temp[1]++;
						winfacVal = param_temp[1];
					} else { 
						winfacVal = 10;
					}
				} else {
					winfacVal = param_temp[1];
				}				
			} else if (param_temp[1] == '.') {
				param_temp[0] -= 48;
				param_temp[2] -= 48;
				winfacVal = (param_temp[0] * 10) + param_temp[2];
			} else if (param_temp[2] == '.') {
				param_temp[0] -= 48;
				param_temp[1] -= 48;
				winfacVal = (param_temp[0] * 10) + param_temp[1];
				winfacVal *= 10;
			}
		}		
	} else if (count == 5) { 				// 4 characters
		for (u_ctr = 0; u_ctr < 3; u_ctr++) {
			if (param_temp[u_ctr] == '.')
				deci_ctr++;
		}

		if (deci_ctr <= 1) {
			if (param_temp[0] == '.') {
				param_temp[1] -= 48;
				param_temp[2] -= 48;
				if (param_temp[2] >= 5) {
					if ((param_temp[1] >=0) && (param_temp[1] <= 8)) {
						param_temp[1]++;
						winfacVal = param_temp[1];
					} else { 
						winfacVal = 10;
					}
				} else {
					winfacVal = param_temp[1];
				}
			} else if (param_temp[1] == '.') {
				param_temp[0] -= 48;
				param_temp[2] -= 48;
				winfacVal = (param_temp[0] * 10) + param_temp[2];
			} else if (param_temp[2] == '.') {
				param_temp[0] -= 48;
				param_temp[1] -= 48;
				param_temp[3] -= 48;
				winfacVal = (param_temp[0] * 100) + (param_temp[1] * 10) + param_temp[3];
			} else if (param_temp[3] == '.') {
				param_temp[0] -= 48;
				param_temp[1] -= 48;
				param_temp[2] -= 48;
				winfacVal = (param_temp[0] * 100) + (param_temp[1] * 10) + param_temp[2];
			}
		}		
	}

	*winfac = winfacVal;
}
#endif


/* Read ADC Value */
#if 0
static u16 read_adc_value(struct LTR553_data *LTR553)
{

	int8_t ret = -99;
	u16 value = -99;
	u16 ps_val;
	int ch0_val;
	int ch1_val;
	uint8_t gain, value_temp, gain_chg_req = 0;
	uint8_t buffer[4], temp;

#define AGC_UP_THRESHOLD		40000
#define AGC_DOWN_THRESHOLD  	5000
#define AGC_HYS					15

	switch (LTR553->mode) {
		case 0 :
			/* ALS */
			buffer[0] = LTR553_ALS_DATA_CH1_0;

			/* read data bytes from data regs */
			ret = I2C_Read(buffer, 4);
			break;

		case 1 :
		case 3 : /* PS with saturation bit */
			/* PS */
			buffer[0] = LTR553_PS_DATA_0;

			/* read data bytes from data regs */
			ret = I2C_Read(buffer, 2);
			break;
	}

	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}


	switch (LTR553->mode) {
		case 0 :
			/* ALS Ch0 */
		 	ch0_val = (u16)buffer[2] | ((u16)buffer[3] << 8);
				dev_dbg(&LTR553->i2c_client->dev, 
					"%s | als_ch0 value = 0x%04X\n", __func__, 
					ch0_val);

			if (ch0_val > ALS_MAX_MEASURE_VAL) {
				dev_err(&LTR553->i2c_client->dev,
				        "%s: ALS Value Error: 0x%X\n", __func__,
				        ch0_val);
			}
			ch0_val &= ALS_VALID_MEASURE_MASK;
			input_report_abs(LTR553->als_input_dev, ABS_MISC, ch0_val);
			input_sync(LTR553->als_input_dev);

			/* ALS Ch1 */
		 	ch1_val = (u16)buffer[0] | ((u16)buffer[1] << 8);
				dev_dbg(&LTR553->i2c_client->dev, 
					"%s | als_ch1 value = 0x%04X\n", __func__, 
					ch1_val);

			if (ch1_val > ALS_MAX_MEASURE_VAL) {
				dev_err(&LTR553->i2c_client->dev,
				        "%s: ALS Value Error: 0x%X\n", __func__,
				        ch1_val);
			}
			ch1_val &= ALS_VALID_MEASURE_MASK;
			input_report_abs(LTR553->als_input_dev, ABS_MISC, ch1_val);
			input_sync(LTR553->als_input_dev);

			buffer[0] = LTR553_ALS_PS_STATUS;
			ret = I2C_Read(buffer, 1);
			if (ret < 0) {
				dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
				return ret;
			}

			value_temp = buffer[0];
			temp = buffer[0];
			gain = (value_temp & 0x70);	
			gain >>= 4;

			if (gain == 0) {			//gain 1
				gain = 1;
			} else if (gain == 1) {		//gain 2
				gain = 2;
			} else if (gain == 2) {		//gain 4
				gain = 4;
			} else if (gain == 3) {		//gain 8
				gain = 8;
			} else if (gain == 6) {		//gain 48
				gain = 48;
			} else if (gain == 7) {		//gain 96
				gain = 96;
			}

			buffer[0] = LTR553_ALS_CONTR;
			ret = I2C_Read(buffer, 1);
			if (ret < 0) {
				dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
				return ret;
			}
			value_temp = buffer[0];
			value_temp &= 0xE3;
			

			if ((ch0_val == 0) && (ch1_val > 50 ))
			{
				value = lux_val_prev;
			} 
			else 
			{
				if(AGC==1)  //Default is AGC =1 (enable)
				{
    				if (gain == 1) {
    					if ((ch0_val + ch1_val) < ((AGC_DOWN_THRESHOLD * 10) / AGC_HYS)) {
    						value = ratioHysterisis(ch0_val, ch1_val);
    						value_temp |= ALS_GAIN_8x;
    						gain_chg_req = 1;
    					} else {
    						value = ratioHysterisis(ch0_val, ch1_val);
    					}
    				} else if (gain == 8) {
    					if ((ch0_val + ch1_val) > AGC_UP_THRESHOLD) {
    						value = ratioHysterisis(ch0_val, ch1_val);
    						value_temp |= ALS_GAIN_1x;
    						gain_chg_req = 1;
    					} else {
    						value = ratioHysterisis(ch0_val, ch1_val);
    					}
    				} else {
    					value = ratioHysterisis(ch0_val, ch1_val);
    				}
							
					if (gain_chg_req) {
    					buffer[0] = LTR553_ALS_CONTR;
    					buffer[1] = value_temp;
    					ret = I2C_Write(buffer, 2);
    					if (ret < 0) {
    						dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
    						return ret;
    					}
					}
				}
				else
				{
					value = ratioHysterisis(ch0_val, ch1_val);
				}

				/* ALS Lux Conversion */
				//value = lux_formula(ch0_val, ch1_val);
			}

			if ((value > 50000) || (((ch0_val + ch1_val) > 50000) && (temp & 0x80))) {
				value = 50000;
			}
			lux_val_prev = value;

			break;

		case 1 :	
		case 3 : /* PS with saturation bit */
			/* PS */
			ps_val = (u16)buffer[0] | ((u16)buffer[1] << 8);
				dev_dbg(&LTR553->i2c_client->dev, 
					"%s | ps value = 0x%04X\n", __func__, 
					ps_val);

			if (LTR553->mode == 1) {
				if (ps_val > PS_MAX_MEASURE_VAL) {
					dev_err(&LTR553->i2c_client->dev,
					        "%s: PS Value Error: 0x%X\n", __func__,
					        ps_val);
				}
				ps_val &= PS_VALID_MEASURE_MASK;				
			} else if (LTR553->mode == 3) {
				ps_val &= 0x87FF;
			}			
			
			value = ps_val;

			break;		
			
	}

	return value;
}
#endif

int ch0_raw_val = 0;

static u16 read_als_adc_value(struct LTR553_data *LTR553)
{

	int8_t ret = -99;
	u16 value = -99;
	int ch0_val;
	int ch1_val;
	uint8_t gain, value_temp, gain_chg_req = 0;
	uint8_t buffer[4], temp;

#define AGC_UP_THRESHOLD		40000
#define AGC_DOWN_THRESHOLD  	5000
#define AGC_HYS					15
#define MAX_VAL					50000

	/* ALS */
	buffer[0] = LTR553_ALS_DATA_CH1_0;

	/* read data bytes from data regs */
	ret = I2C_Read(buffer, 4);

	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}

	/* ALS Ch0 */
 	ch0_val = (u16)buffer[2] | ((u16)buffer[3] << 8);
		dev_dbg(&LTR553->i2c_client->dev, 
			"%s | als_ch0 value = 0x%04X\n", __func__, 
			ch0_val);

	if (ch0_val > ALS_MAX_MEASURE_VAL) {
		dev_err(&LTR553->i2c_client->dev,
		        "%s: ALS Value Error: 0x%X\n", __func__,
		        ch0_val);
	}
	ch0_val &= ALS_VALID_MEASURE_MASK;
    ch0_raw_val = ch0_val ;
	//input_report_abs(LTR553->als_input_dev, ABS_MISC, ch0_val);
	//input_sync(LTR553->als_input_dev);

	/* ALS Ch1 */
 	ch1_val = (u16)buffer[0] | ((u16)buffer[1] << 8);
		dev_dbg(&LTR553->i2c_client->dev, 
			"%s | als_ch1 value = 0x%04X\n", __func__, 
			ch1_val);

	if (ch1_val > ALS_MAX_MEASURE_VAL) {
		dev_err(&LTR553->i2c_client->dev,
		        "%s: ALS Value Error: 0x%X\n", __func__,
		        ch1_val);
	}
	ch1_val &= ALS_VALID_MEASURE_MASK;
	//input_report_abs(LTR553->als_input_dev, ABS_MISC, ch1_val);
	//input_sync(LTR553->als_input_dev);

	buffer[0] = LTR553_ALS_PS_STATUS;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}

	value_temp = buffer[0];
	temp = buffer[0];
	gain = (value_temp & 0x70);	
	gain >>= 4;

	if (gain == 0) {			//gain 1
		gain = 1;
	} else if (gain == 1) {		//gain 2
		gain = 2;
	} else if (gain == 2) {		//gain 4
		gain = 4;
	} else if (gain == 3) {		//gain 8
		gain = 8;
	} else if (gain == 6) {		//gain 48
		gain = 48;
	} else if (gain == 7) {		//gain 96
		gain = 96;
	}

	buffer[0] = LTR553_ALS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}
	value_temp = buffer[0];
	value_temp &= 0xE3;

	if ((ch0_val == 0) && (ch1_val > 50 )) {
		value = lux_val_prev;
	} else {
		if (gain == 1) {
			if ((ch0_val + ch1_val) < ((AGC_DOWN_THRESHOLD * 10) / AGC_HYS)) {
				value = ratioHysterisis(ch0_val, ch1_val);
				value_temp |= ALS_GAIN_8x;
				gain_chg_req = 1;
			} else {
				value = ratioHysterisis(ch0_val, ch1_val);
			}
		} else if (gain == 8) {
			if ((ch0_val + ch1_val) > AGC_UP_THRESHOLD) {
				value = ratioHysterisis(ch0_val, ch1_val);
				value_temp |= ALS_GAIN_1x;
				gain_chg_req = 1;
			} else {
				value = ratioHysterisis(ch0_val, ch1_val);
			}
		} else {
			value = ratioHysterisis(ch0_val, ch1_val);
		}
		if (gain_chg_req) {
			buffer[0] = LTR553_ALS_CONTR;
			buffer[1] = value_temp;
			ret = I2C_Write(buffer, 2);
			if (ret < 0) {
				dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);

				return ret;
			}
		}
		
	}

	if ((value > MAX_VAL) || (((ch0_val + ch1_val) > MAX_VAL) && (temp & 0x80))) {
		value = MAX_VAL;
	}
    value = (value * 8)/10 ;
	lux_val_prev = value;	
    printk("[ltr553]<4><----gain %d ,gain_chg_req %d ,lux_val %d ---->\n", gain, gain_chg_req, value);

	return value;
}


static u16 read_ps_adc_value(struct LTR553_data *LTR553)
{
	int8_t ret = -99;
	u16 value = -99;
	u16 ps_val;
	uint8_t buffer[4];

	buffer[0] = LTR553_PS_DATA_0;

	/* read data bytes from data regs */
	ret = I2C_Read(buffer, 2);

	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}

	ps_val = (u16)buffer[0] | ((u16)buffer[1] << 8);
    dev_dbg(&LTR553->i2c_client->dev, 
            "%s | ps value = 0x%04X\n", __func__, 
            ps_val);

    if (ps_val > PS_MAX_MEASURE_VAL) {
        dev_err(&LTR553->i2c_client->dev,
                "%s: PS Value Error: 0x%X\n", __func__,
                ps_val);
    }
    ps_val &= PS_VALID_MEASURE_MASK;				

    value = ps_val;

	return value;
}


static int8_t als_mode_setup (uint8_t alsMode_set_reset, struct LTR553_data *LTR553)
{
	int8_t ret = 0;

	ret = _LTR553_set_bit(LTR553->i2c_client, alsMode_set_reset, LTR553_ALS_CONTR, ALS_MODE_ACTIVE);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s ALS mode setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t als_sw_reset_setup(uint8_t alsSWReset_set_reset, struct LTR553_data *LTR553)
{
	int8_t ret = 0;

	ret = _LTR553_set_bit(LTR553->i2c_client, alsSWReset_set_reset, LTR553_ALS_CONTR, ALS_SW_RESET);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s ALS sw reset setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t als_gain_setup (uint8_t alsgain_range, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_ALS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xE3;

	if (alsgain_range == 1) {
		value |= ALS_GAIN_1x;
	} else if (alsgain_range == 2) {
		value |= ALS_GAIN_2x;
	} else if (alsgain_range == 4) {
		value |= ALS_GAIN_4x;
	} else if (alsgain_range == 8) {
		value |= ALS_GAIN_8x;
	} else if (alsgain_range == 48) {
		value |= ALS_GAIN_48x;
	} else if (alsgain_range == 96) {
		value |= ALS_GAIN_96x;
	}

	buffer[0] = LTR553_ALS_CONTR;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s ALS gain setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t als_contr_setup(uint8_t als_contr_val, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR553_ALS_CONTR;

	/* Default settings used for now. */
	buffer[1] = als_contr_val;
	buffer[1] &= 0x1F;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | ALS_CONTR (0x%02X) setup fail...", __func__, buffer[0]);
	}

	return ret;
}


static int8_t als_contr_readback (uint8_t rdbck_type, uint8_t *retVal, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_ALS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == ALS_MODE_RDBCK) {
		*retVal = value & 0x01;
	} else if (rdbck_type == ALS_SWRT_RDBCK) {
		*retVal = (value & 0x02) >> 1;
	} else if (rdbck_type == ALS_GAIN_RDBCK) {
		*retVal = (value & 0x1C) >> 2;
	} else if (rdbck_type == ALS_CONTR_RDBCK) {
		*retVal = value & 0x1F;
	}

	return ret;
}


static int8_t ps_mode_setup (uint8_t psMode_set_reset, struct LTR553_data *LTR553)
{
	int8_t ret = 0;

	ret = _LTR553_set_bit(LTR553->i2c_client, psMode_set_reset, LTR553_PS_CONTR, PS_MODE_ACTIVE);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s PS mode setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t ps_gain_setup (uint8_t psgain_range, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_PS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xF3;

	if (psgain_range == 16) {
		value |= PS_GAIN_16x;
	} else if (psgain_range == 32) {
		value |= PS_GAIN_32x;
	} else if (psgain_range == 64) {
		value |= PS_GAIN_64x;
	}

	buffer[0] = LTR553_PS_CONTR;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s PS gain setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t ps_satu_indica_setup(uint8_t pssatuindica_enable, struct LTR553_data *LTR553)
{
	int8_t ret = 0;

	ret = _LTR553_set_bit(LTR553->i2c_client, pssatuindica_enable, LTR553_PS_CONTR, PS_SATUR_INDIC_EN);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s PS saturation indicator setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t ps_contr_setup(uint8_t ps_contr_val, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR553_PS_CONTR;

	/* Default settings used for now. */
	buffer[1] = ps_contr_val;
	buffer[1] &= 0x2F;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | PS_CONTR (0x%02X) setup fail...", __func__, buffer[0]);
	}

	return ret;
}


static int8_t ps_contr_readback (uint8_t rdbck_type, uint8_t *retVal, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_PS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == PS_MODE_RDBCK) {
		*retVal = (value & 0x03);
	} else if (rdbck_type == PS_GAIN_RDBCK) {
		*retVal = (value & 0x0C) >> 2;
	} else if (rdbck_type == PS_SATUR_RDBCK) {
		*retVal = (value & 0x20) >> 5;
	} else if (rdbck_type == PS_CONTR_RDBCK) {
		*retVal = value & 0x2F;
	}

	return ret;
}


static int8_t ps_ledCurrent_setup (uint8_t psledcurr_val, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_PS_LED;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xF8;

	if (psledcurr_val == 5) {
		value |= LED_CURR_5MA;
	} else if (psledcurr_val == 10) {
		value |= LED_CURR_10MA;
	} else if (psledcurr_val == 20) {
		value |= LED_CURR_20MA;
	} else if (psledcurr_val == 50) {
		value |= LED_CURR_50MA;
	} else if (psledcurr_val == 100) {
		value |= LED_CURR_100MA;
	}

	buffer[0] = LTR553_PS_LED;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s PS LED current setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t ps_ledCurrDuty_setup (uint8_t psleddutycycle_val, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_PS_LED;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xE7;

	if (psleddutycycle_val == 25) {
		value |= LED_CURR_DUTY_25PC;
	} else if (psleddutycycle_val == 50) {
		value |= LED_CURR_DUTY_50PC;
	} else if (psleddutycycle_val == 75) {
		value |= LED_CURR_DUTY_75PC;
	} else if (psleddutycycle_val == 100) {
		value |= LED_CURR_DUTY_100PC;
	} 

	buffer[0] = LTR553_PS_LED;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s PS LED current duty setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t ps_ledPulseFreq_setup (uint8_t pspulreq_val, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_PS_LED;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0x1F;

	if (pspulreq_val == 30) {
		value |= LED_PUL_FREQ_30KHZ;
	} else if (pspulreq_val == 40) {
		value |= LED_PUL_FREQ_40KHZ;
	} else if (pspulreq_val == 50) {
		value |= LED_PUL_FREQ_50KHZ;
	} else if (pspulreq_val == 60) {
		value |= LED_PUL_FREQ_60KHZ;
	} else if (pspulreq_val == 70) {
		value |= LED_PUL_FREQ_70KHZ;
	} else if (pspulreq_val == 80) {
		value |= LED_PUL_FREQ_80KHZ;
	} else if (pspulreq_val == 90) {
		value |= LED_PUL_FREQ_90KHZ;
	} else if (pspulreq_val == 100) {
		value |= LED_PUL_FREQ_100KHZ;
	}

	buffer[0] = LTR553_PS_LED;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s PS LED pulse frequency setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


/* LED Setup */
static int8_t ps_led_setup(uint8_t ps_led_val, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR553_PS_LED;

	/* Default settings used for now. */
	buffer[1] = ps_led_val;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | PS_LED (0x%02X) setup fail...", __func__, buffer[0]);
	}

	return ret;
}


static int8_t ps_led_readback (uint8_t rdbck_type, uint8_t *retVal, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_PS_LED;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == LED_CURR_RDBCK) {
		*retVal = (value & 0x07);
	} else if (rdbck_type == LED_CURR_DUTY_RDBCK) {
		*retVal = (value & 0x18) >> 3;
	} else if (rdbck_type == LED_PUL_FREQ_RDBCK) {
		*retVal = (value & 0xE0) >> 5;
	} else if (rdbck_type == PS_LED_RDBCK) {
		*retVal = value;
	}

	return ret;
}


static int8_t ps_ledPulseCount_setup(uint8_t pspulsecount_val, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR553_PS_N_PULSES;

	/* Default settings used for now. */
	if (pspulsecount_val > 15) {
		pspulsecount_val = 15;
	}
	buffer[1] = pspulsecount_val;
	buffer[1] &= 0x0F;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | PS_LED_COUNT (0x%02X) setup fail...", __func__, buffer[0]);
	}

	return ret;
}


static int8_t ps_ledPulseCount_readback (uint8_t *retVal, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[3], value;

	buffer[0] = LTR553_PS_N_PULSES;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	*retVal = value;

	return ret;
}


static int8_t ps_meas_rate_setup(u16 meas_rate_val, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_PS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}

	value = buffer[0];
	value &= 0xF0;

	if (meas_rate_val == 50) {
		value |= PS_MEAS_RPT_RATE_50MS;
	} else if (meas_rate_val == 70) {
		value |= PS_MEAS_RPT_RATE_70MS;
	} else if (meas_rate_val == 100) {
		value |= PS_MEAS_RPT_RATE_100MS;
	} else if (meas_rate_val == 200) {
		value |= PS_MEAS_RPT_RATE_200MS;
	} else if (meas_rate_val == 500) {
		value |= PS_MEAS_RPT_RATE_500MS;
	} else if (meas_rate_val == 1000) {
		value |= PS_MEAS_RPT_RATE_1000MS;
	} else if (meas_rate_val == 2000) {
		value |= PS_MEAS_RPT_RATE_2000MS;
	} else if (meas_rate_val == 10) {
		value |= PS_MEAS_RPT_RATE_10MS;		
	}

	buffer[0] = LTR553_PS_MEAS_RATE;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s PS measurement rate setup fail...\n", __func__);

		return ret;
	}

	return ret;
}


static int8_t ps_meas_rate_readback (uint8_t *retVal, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[3], value;

	buffer[0] = LTR553_PS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	*retVal = (value & 0x0F);

	return ret;
}


static int8_t als_meas_rate_setup(u16 meas_rate_val, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_ALS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xF8;

	if (meas_rate_val == 50) {
		value |= ALS_MEAS_RPT_RATE_50MS;
	} else if (meas_rate_val == 100) {
		value |= ALS_MEAS_RPT_RATE_100MS;
	} else if (meas_rate_val == 200) {
		value |= ALS_MEAS_RPT_RATE_200MS;
	} else if (meas_rate_val == 500) {
		value |= ALS_MEAS_RPT_RATE_500MS;
	} else if (meas_rate_val == 1000) {
		value |= ALS_MEAS_RPT_RATE_1000MS;
	} else if (meas_rate_val == 2000) {
		value |= ALS_MEAS_RPT_RATE_2000MS;
	}

	buffer[0] = LTR553_ALS_MEAS_RATE;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s ALS measurement rate setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t als_integ_time_setup(u16 integ_time_val, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_ALS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xC7;

	if (integ_time_val == 100) {
		value |= ALS_INTEG_TM_100MS;
	} else if (integ_time_val == 50) {
		value |= ALS_INTEG_TM_50MS;
	} else if (integ_time_val == 200) {
		value |= ALS_INTEG_TM_200MS;
	} else if (integ_time_val == 400) {
		value |= ALS_INTEG_TM_400MS;
	} else if (integ_time_val == 150) {
		value |= ALS_INTEG_TM_150MS;
	} else if (integ_time_val == 250) {
		value |= ALS_INTEG_TM_250MS;
	} else if (integ_time_val == 300) {
		value |= ALS_INTEG_TM_300MS;
	} else if (integ_time_val == 350) {
		value |= ALS_INTEG_TM_350MS;
	}

	buffer[0] = LTR553_ALS_MEAS_RATE;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s ALS integration time setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t als_meas_rate_reg_setup(uint8_t als_meas_rate_reg_val, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR553_ALS_MEAS_RATE;

	buffer[1] = als_meas_rate_reg_val;
	buffer[1] &= 0x3F;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | ALS_MEAS_RATE (0x%02X) setup fail...", __func__, buffer[0]);
	}

	return ret;
}


static int8_t als_meas_rate_readback (uint8_t rdbck_type, uint8_t *retVal, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_ALS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == ALS_MEAS_RPT_RATE_RDBCK) {
		*retVal = (value & 0x07);
	} else if (rdbck_type == ALS_INTEG_TM_RDBCK) {
		*retVal = (value & 0x38) >> 3;
	} else if (rdbck_type == ALS_MEAS_RATE_RDBCK) {
		*retVal = (value & 0x3F);
	}

	return ret;
}


static int8_t part_ID_reg_readback (uint8_t rdbck_type, uint8_t *retVal, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[1], value;

	buffer[0] = LTR553_PART_ID;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == PART_NUM_ID_RDBCK) {
		*retVal = (value & 0xF0) >> 4;
	} else if (rdbck_type == REVISION_ID_RDBCK) {
		*retVal = value & 0x0F;
	} else if (rdbck_type == PART_ID_REG_RDBCK) {
		*retVal = value;
	}

	return ret;
}


static int8_t manu_ID_reg_readback (uint8_t *retVal, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[1], value;

	buffer[0] = LTR553_MANUFACTURER_ID;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	*retVal = value;

	return ret;
}


static int8_t als_ps_status_reg (uint8_t data_status_type, uint8_t *retVal, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_ALS_PS_STATUS;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (data_status_type == PS_DATA_STATUS_RDBCK) {
		*retVal = (value & 0x01);
	} else if (data_status_type == PS_INTERR_STATUS_RDBCK) {
		*retVal = (value & 0x02) >> 1;
	} else if (data_status_type == ALS_DATA_STATUS_RDBCK) {
		*retVal = (value & 0x04) >> 2;
	} else if (data_status_type == ALS_INTERR_STATUS_RDBCK) {
		*retVal = (value & 0x08) >> 3;
	} else if (data_status_type == ALS_GAIN_STATUS_RDBCK) {
		*retVal = (value & 0x70) >> 4;
	} else if (data_status_type == ALS_VALID_STATUS_RDBCK) {
		*retVal = (value & 0x80) >> 7;
	} else if (data_status_type == ALS_PS_STATUS_RDBCK) {
		*retVal = value;
	}

	return ret;
}


static int8_t als_ch0ch1raw_calc_readback (u16 *retVal1, u16 *retVal2, u16 *retVal3, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[11];
	u16 value1, value2, value3;

	buffer[0] = LTR553_ALS_DATA_CH1_0;
	ret = I2C_Read(buffer, 4);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value1 = ((int)buffer[2]) + ((int)buffer[3] << 8); // CH0
	value2 = ((int)buffer[0]) + ((int)buffer[1] << 8); // CH1

	value3 = ratioHysterisis(value1, value2);
	
	*retVal1 = value1;
	*retVal2 = value2;
	*retVal3 = value3;

	return ret;
}


static int8_t interrupt_mode_setup (uint8_t interr_mode_val, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_INTERRUPT;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xFC;

	if (interr_mode_val == 0) {
		value |= INT_MODE_00;
	} else if (interr_mode_val == 1) {
		value |= INT_MODE_PS_TRIG;
	} else if (interr_mode_val == 2) {
		value |= INT_MODE_ALS_TRIG;
	} else if (interr_mode_val == 3) {
		value |= INT_MODE_ALSPS_TRIG;
	} 

	buffer[0] = LTR553_INTERRUPT;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s Interrupt mode setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t interrupt_polarity_setup (uint8_t interr_polar_val, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_INTERRUPT;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xFB;

	if (interr_polar_val == 0) {
		value |= INT_POLAR_ACT_LO;
	} else if (interr_polar_val == 1) {
		value |= INT_POLAR_ACT_HI;
	}

	buffer[0] = LTR553_INTERRUPT;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s Interrupt polarity setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t interrupt_setup(uint8_t interrupt_val, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR553_INTERRUPT;

	/* Default settings used for now. */
	buffer[1] = interrupt_val;
	buffer[1] &= 0x07;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s |Interrupt (0x%02X) setup fail...", __func__, buffer[0]);
	}

	return ret;
}


static int8_t interrupt_readback (uint8_t rdbck_type, uint8_t *retVal, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_INTERRUPT;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == INT_MODE_RDBCK) {
		*retVal = (value & 0x03);
	} else if (rdbck_type == INT_POLAR_RDBCK) {
		*retVal = (value & 0x04) >> 2;
	} else if (rdbck_type == INT_INTERRUPT_RDBCK) {
		*retVal = (value & 0x07);
	}

	return ret;
}


static int8_t ps_offset_setup (u16 ps_offset_val, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[3];
	
	buffer[0] = LTR553_PS_OFFSET_1;
	buffer[1] = (ps_offset_val >> 8) & 0x03;
	buffer[2] = (ps_offset_val & 0xFF);

	ret = I2C_Write(buffer, 3);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s PS offset setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t ps_offset_readback (u16 *offsetval, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[2];
	u16 value;

	buffer[0] = LTR553_PS_OFFSET_1;
	ret = I2C_Read(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value <<= 8;
	value += buffer[1];

	*offsetval = value;

	return ret;
}


static int8_t interrupt_persist_setup (uint8_t interr_persist_val, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	value = interr_persist_val;

	buffer[0] = LTR553_INTERRUPT_PRST;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s Interrupt persist setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t interrupt_prst_readback (uint8_t *retVal, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_INTERRUPT_PRST;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	*retVal = value;

	return ret;
}


/* Set ALS range */
static int8_t set_als_range(u16 lt, u16 ht, uint8_t lo_hi)
{
	int8_t ret;
	uint8_t buffer[5], num_data = 0;

	if (lo_hi == LO_LIMIT) {
		buffer[0] = LTR553_ALS_THRES_LOW_0;
		buffer[1] = lt & 0xFF;
		buffer[2] = (lt >> 8) & 0xFF;
		num_data = 3;
	} else if (lo_hi == HI_LIMIT) {
		buffer[0] = LTR553_ALS_THRES_UP_0;
		buffer[1] = ht & 0xFF;
		buffer[2] = (ht >> 8) & 0xFF;
		num_data = 3;
	} else if (lo_hi == LO_N_HI_LIMIT) {
		buffer[0] = LTR553_ALS_THRES_UP_0;
		buffer[1] = ht & 0xFF;
		buffer[2] = (ht >> 8) & 0xFF;
		buffer[3] = lt & 0xFF;
		buffer[4] = (lt >> 8) & 0xFF;
		num_data = 5;
	}	

	ret = I2C_Write(buffer, num_data);
	if (ret <0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	dev_dbg(&sensor_info->i2c_client->dev, "%s Set als range:0x%04x"
	                                       " - 0x%04x\n", __func__, lt, ht);

	return ret;
}


static int8_t als_range_readback (u16 *lt, u16 *ht, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[5];
	u16 value_lo, value_hi;

	buffer[0] = LTR553_ALS_THRES_UP_0;
	ret = I2C_Read(buffer, 4);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value_lo = buffer[3];
	value_lo <<= 8;
	value_lo += buffer[2];
	*lt = value_lo;

	value_hi = buffer[1];
	value_hi <<= 8;
	value_hi += buffer[0];
	*ht = value_hi;

	return ret;
}


/* Set PS range */
static int8_t set_ps_range(u16 lt, u16 ht, uint8_t lo_hi, struct LTR553_data *LTR553)
{
	int8_t ret;
	uint8_t buffer[5], num_data = 0;

	if (lo_hi == LO_LIMIT) {
		buffer[0] = LTR553_PS_THRES_LOW_0;
		buffer[1] = lt & 0xFF;
		buffer[2] = (lt >> 8) & 0x07;
		num_data = 3;
	} else if (lo_hi == HI_LIMIT) {
		buffer[0] = LTR553_PS_THRES_UP_0;
		buffer[1] = ht & 0xFF;
		buffer[2] = (ht >> 8) & 0x07;
		num_data = 3;
	} else if (lo_hi == LO_N_HI_LIMIT) {
		buffer[0] = LTR553_PS_THRES_UP_0;
		buffer[1] = ht & 0xFF;
		buffer[2] = (ht >> 8) & 0x07;
		buffer[3] = lt & 0xFF;
		buffer[4] = (lt >> 8) & 0x07;
		num_data = 5;
	}	

	ret = I2C_Write(buffer, num_data);
	if (ret <0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}
	dev_dbg(&LTR553->i2c_client->dev, "%s Set ps range:0x%04x"
	                                       " - 0x%04x\n", __func__, lt, ht);

	return ret;
}


static int8_t ps_range_readback (u16 *lt, u16 *ht, struct LTR553_data *LTR553)
{
	int8_t ret = 0;
	uint8_t buffer[5];
	u16 value_lo, value_hi;

	buffer[0] = LTR553_PS_THRES_UP_0;
	ret = I2C_Read(buffer, 4);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}

	value_lo = buffer[3];
	value_lo <<= 8;
	value_lo += buffer[2];
	*lt = value_lo;

	value_hi = buffer[1];
	value_hi <<= 8;
	value_hi += buffer[0];
	*ht = value_hi;

	return ret;
}


static u16 discardMinMax_findCTMov_Avg (u16 *ps_val)
{
#define MAX_NUM_PS_DATA1		PS_MAX_MOV_AVG_KEPT_DATA_CTR
#define STARTING_PS_INDEX1		0
#define ENDING_PS_INDEX1		5
#define NUM_AVG_DATA1			5

	uint8_t i_ctr, i_ctr2, maxIndex, minIndex;
	u16 maxVal, minVal, _ps_val[MAX_NUM_PS_DATA1];
	u16 temp = 0;

	for (i_ctr = STARTING_PS_INDEX1; i_ctr < MAX_NUM_PS_DATA1; i_ctr++) {
		_ps_val[i_ctr] = ps_val[i_ctr];
	}

	maxVal = ps_val[STARTING_PS_INDEX1];
	maxIndex = STARTING_PS_INDEX1;
	minVal = ps_val[STARTING_PS_INDEX1];
	minIndex = STARTING_PS_INDEX1;

	for (i_ctr = STARTING_PS_INDEX1; i_ctr < MAX_NUM_PS_DATA1; i_ctr++) {
		if (ps_val[i_ctr] > maxVal) {
			maxVal = ps_val[i_ctr];
			maxIndex = i_ctr;
		}
	}

	for (i_ctr = STARTING_PS_INDEX1; i_ctr < MAX_NUM_PS_DATA1; i_ctr++) {
		if (ps_val[i_ctr] < minVal) {
			minVal = ps_val[i_ctr];
			minIndex = i_ctr;
		}
	}

	i_ctr2 = 0;

	if (minIndex != maxIndex) {
		for (i_ctr = STARTING_PS_INDEX1; i_ctr < MAX_NUM_PS_DATA1; i_ctr++) {
			if ((i_ctr != minIndex) && (i_ctr != maxIndex)) {
				ps_val[i_ctr2] = _ps_val[i_ctr];
				i_ctr2++;
			}
		}
	}
	ps_val[MAX_NUM_PS_DATA1 - 1] = 0;
	ps_val[MAX_NUM_PS_DATA1 - 2] = 0;

	for (i_ctr = STARTING_PS_INDEX1; i_ctr < ENDING_PS_INDEX1; i_ctr++) {
		temp += ps_val[i_ctr];
	}
	
	temp = (temp / NUM_AVG_DATA1);

	return temp;
}


static u16 findCT_Avg (u16 *ps_val)
{
#define MAX_NUM_PS_DATA2		PS_MAX_INIT_KEPT_DATA_COUNTER
#define STARTING_PS_INDEX2		3
#define NUM_AVG_DATA2			3

	uint8_t i_ctr, min_Index, max_Index;
	u16 max_val, min_val;
	u16 temp = 0;
	//struct LTR553_data *LTR553 = sensor_info;

	max_val = ps_val[STARTING_PS_INDEX2];
	max_Index = STARTING_PS_INDEX2;
	min_val = ps_val[STARTING_PS_INDEX2];
	min_Index = STARTING_PS_INDEX2;

	for (i_ctr = STARTING_PS_INDEX2; i_ctr < MAX_NUM_PS_DATA2; i_ctr++) {
		if (ps_val[i_ctr] > max_val) {
			max_val = ps_val[i_ctr];
			max_Index = i_ctr;
		}
	}

	for (i_ctr = STARTING_PS_INDEX2; i_ctr < MAX_NUM_PS_DATA2; i_ctr++) {
		if (ps_val[i_ctr] < min_val) {
			min_val = ps_val[i_ctr];
			min_Index = i_ctr;
		}
	}

	if (min_val == max_val) {
		// all values are the same
		temp = ps_val[STARTING_PS_INDEX2];
	} else {
		for (i_ctr = STARTING_PS_INDEX2; i_ctr < MAX_NUM_PS_DATA2; i_ctr++) {
			if ((i_ctr != min_Index) && (i_ctr != max_Index)) {
				temp += ps_val[i_ctr];
			}
		}
		temp = (temp / NUM_AVG_DATA2);
	}

	//temp = (temp / NUM_AVG_DATA2);

	return temp;
}


// take note ------------------------------------------
// This function should be called in the function which is called when the CALL button is pressed.
// take note ------------------------------------------
static void setThrDuringCall (void)
{
	int8_t ret;
	struct LTR553_data *LTR553 = sensor_info;

	// set ps measurement rate to 10ms
	ret = ps_meas_rate_setup(10, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS MeasRate Setup Fail...\n", __func__);
	}

	ps_grabData_stage = 0;
	ps_kept_data_counter = 0;
	ps_movavg_data_counter = 0;

	ret = set_ps_range(PS_MIN_MEASURE_VAL, PS_MIN_MEASURE_VAL, LO_N_HI_LIMIT, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s : PS thresholds setting Fail...\n", __func__);
	}

	ret = ps_contr_setup(0x03, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS Enable Fail...\n", __func__);
	}
}


//(Linux RTOS)>
/* Report PS input event */
static void report_ps_input_event(struct LTR553_data *LTR553)
{
	int8_t ret;
	u16 adc_value;

	adc_value = read_ps_adc_value (LTR553);
	printk(KERN_INFO "%s:adc_value = %d\n",__func__,adc_value);
#if 1	
	if(PS_Dyn_Calib==1) //Default is PS_Dyn_Calib=1 (enable)
	{
    	if (ps_grabData_stage == 0) {
    		if (ps_kept_data_counter < PS_MAX_INIT_KEPT_DATA_COUNTER) {
    			if (adc_value != 0) {
    				ps_init_kept_data[ps_kept_data_counter] = adc_value;
    				ps_kept_data_counter++;
    			}
    		} 
    
    		if (ps_kept_data_counter >= PS_MAX_INIT_KEPT_DATA_COUNTER) {
    			ps_ct_avg = findCT_Avg(ps_init_kept_data);
    			ftn_init = ps_ct_avg * 17;
    			ps_grabData_stage = 1;
    		}
    	}
    
    	if (ps_grabData_stage == 1) {
    		if ((ftn_init - (ps_ct_avg * 10)) < 1400) {
    			ftn_final = (ps_ct_avg * 10) + 1400;
    		} else {
    			if ((ftn_init - (ps_ct_avg * 10)) > 1800) {
    				ftn_final = (ps_ct_avg * 10) + 1800;
    			} else {
    				ftn_final = ftn_init;
    			}
    		}
    		ntf_final = (ftn_final - (ps_ct_avg * 10));
    		ntf_final *= 4;
    		ntf_final /= 100;
    		ntf_final += ps_ct_avg;
    		ftn_final /= 10;
    		if (ntf_final >= PS_MAX_MEASURE_VAL) {
    			ntf_final = PS_MAX_MEASURE_VAL;
    		}
    		if (ftn_final >= PS_MAX_MEASURE_VAL) {
    			ftn_final = PS_MAX_MEASURE_VAL;
    		}
    
    		ret = ps_meas_rate_setup(50, LTR553);
    		if (ret < 0) {
    			dev_err(&LTR553->i2c_client->dev, "%s: PS MeasRate Setup Fail...\n", __func__);
    		}
    		
    		ps_grabData_stage = 2;
    	}
	}
    else
    {
        ps_grabData_stage = 2;
        if (ltr553_prox_test)
        {
            LTR553->default_ps_highthresh = ltr553_prox_max;
            LTR553->default_ps_lowthresh = ltr553_prox_min;
        }
        ftn_final= LTR553->default_ps_highthresh;
        ntf_final= LTR553->default_ps_lowthresh;
    }

    if (ps_grabData_stage == 2) {
        //input_report_abs(LTR553->ps_input_dev, ABS_DISTANCE, adc_value);
        //input_sync(LTR553->ps_input_dev);
        if ( ltr553_prox_test > 1 )
        {
            input_report_abs(LTR553->ps_input_dev, ABS_DISTANCE, adc_value);
            input_sync(LTR553->ps_input_dev);
            set_ps_range(adc_value, adc_value, LO_N_HI_LIMIT, LTR553);
        }
        else
        {
            /* report NEAR or FAR to the user layer */
            if ((adc_value > ftn_final) || (adc_value < ntf_final)) {
                // FTN
                if (adc_value >= ftn_final) {
                    input_report_abs(LTR553->ps_input_dev, ABS_DISTANCE, NEAR_VAL);
                    input_sync(LTR553->ps_input_dev);
                    printk(KERN_INFO "%s: PS=%d\n",__func__,NEAR_VAL);
                    ntf_final = LTR553->default_ps_lowthresh ;
                    ftn_final = 0xFFFF ;
                }
                // FTN

                // NTF
                if (adc_value <= ntf_final) {
                    input_report_abs(LTR553->ps_input_dev, ABS_DISTANCE, FAR_VAL);
                    input_sync(LTR553->ps_input_dev);
                    printk(KERN_INFO "%s: PS=%d\n",__func__,FAR_VAL);
                    ntf_final = 0 ;
                    ftn_final = LTR553->default_ps_highthresh ;
                }
                // NTF
            }
            /* report NEAR or FAR to the user layer */
            set_ps_range(ntf_final, ftn_final, LO_N_HI_LIMIT, LTR553);
        }
        if (ps_movavg_data_counter < PS_MAX_MOV_AVG_KEPT_DATA_CTR) {
            if (adc_value != 0) {
                ps_movavg_data[ps_movavg_data_counter] = adc_value;
                ps_movavg_data_counter++;
            }
        } 

        if (ps_movavg_data_counter >= PS_MAX_MOV_AVG_KEPT_DATA_CTR) {
            ps_movct_avg = discardMinMax_findCTMov_Avg(ps_movavg_data);

            if (ps_movct_avg < ps_ct_avg) {
                ps_ct_avg = ps_movct_avg;
                ftn_init = ps_ct_avg * 17;
                ps_grabData_stage = 1;
            }
            ps_movavg_data_counter = 5;
        }

    }
    #endif

}


/* Report ALS input event */
static void report_als_input_event(struct LTR553_data *LTR553)
{
    //int8_t ret;
    u16 adc_value;
    //int thresh_hi, thresh_lo, thresh_delta;

    //LTR553->mode = ALS;
    //adc_value = read_adc_value (LTR553);
    adc_value = read_als_adc_value (LTR553);

    input_report_abs(LTR553->als_input_dev, ABS_MISC, adc_value);
    input_sync(LTR553->als_input_dev);
    set_als_range(ch0_raw_val*9/10, ch0_raw_val*11/10, LO_N_HI_LIMIT);


}


/* Work when interrupt */
static void LTR553_schedwork(struct work_struct *work)
{
    int8_t ret;
    uint8_t status;
    uint8_t	interrupt_stat, newdata;
    struct LTR553_data *LTR553 = sensor_info;
    uint8_t buffer[2];

    buffer[0] = LTR553_ALS_PS_STATUS;	
    ret = I2C_Read(buffer, 1);
    //printk(KERN_INFO "%s:status = 0x%x\n",__func__,buffer[0]);
    if (ret < 0) {
        dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
        return;
    }
    status = buffer[0];
    interrupt_stat = status & 0x0A;
    newdata = status & 0x05;
    // PS interrupt and PS with new data
    if ((interrupt_stat & 0x02) && (newdata & 0x01)) {
        LTR553->ps_irq_flag = 1;
        report_ps_input_event(LTR553);
        LTR553->ps_irq_flag = 0;
    }
    #if 1
    // ALS interrupt and ALS with new data
    if ((interrupt_stat & 0x08) && (newdata & 0x04)) {
        LTR553->als_irq_flag = 1;
        report_als_input_event(LTR553);
        LTR553->als_irq_flag = 0;
    }
    #endif
    printk(KERN_INFO "%s:exit\n",__func__);
    enable_irq(LTR553->irq);
}

static DECLARE_WORK(irq_workqueue, LTR553_schedwork);


/* IRQ Handler */
static irqreturn_t LTR553_irq_handler(int irq, void *data)
{
    struct LTR553_data *LTR553 = data;
    //printk(KERN_INFO "%s:\n",__func__);

    /* disable an irq without waiting */
    disable_irq_nosync(LTR553->irq);

    schedule_work(&irq_workqueue);

    return IRQ_HANDLED;
}


#if 1
static int LTR553_gpio_irq(struct LTR553_data *LTR553)
{
    int rc = 0;

    rc = gpio_request(LTR553->gpio_int_no, DEVICE_NAME);
    if (rc < 0) {
        dev_err(&LTR553->i2c_client->dev,"%s: GPIO %d Request Fail (%d)\n", __func__, LTR553->gpio_int_no, rc);
        return rc;
    }

    rc = gpio_direction_input(LTR553->gpio_int_no);
    if (rc < 0) {
        dev_err(&LTR553->i2c_client->dev, "%s: Set GPIO %d as Input Fail (%d)\n", __func__, LTR553->gpio_int_no, rc);
        goto out1;
    }

    /* Configure an active low trigger interrupt for the device */
    //rc = request_irq(LTR553->irq, LTR553_irq_handler, IRQF_TRIGGER_FALLING, DEVICE_NAME, LTR553);
    rc = request_irq(LTR553->irq, LTR553_irq_handler, IRQ_TYPE_EDGE_FALLING | IRQF_ONESHOT, DEVICE_NAME, LTR553);
    if (rc < 0) {
        dev_err(&LTR553->i2c_client->dev, "%s: Request IRQ (%d) for GPIO %d Fail (%d)\n", __func__, LTR553->irq,
                LTR553->gpio_int_no, rc);
        goto out1;
    }

    return rc;

    out1:
    gpio_free(LTR553->gpio_int_no);

    return rc;
}
#endif
//(Linux RTOS)<


/* PS Enable */
static int8_t ps_enable_init(struct LTR553_data *LTR553)
{
    int8_t rc = 0;
    uint8_t buffer[1]; // for dummy read

    if (ltr553_prox_test == 1)
    {
        LTR553->default_ps_highthresh = ltr553_prox_max;
        LTR553->default_ps_lowthresh = ltr553_prox_min;
    }
    else  if (ltr553_prox_test > 1)
    {
        LTR553->default_ps_highthresh = 50;
        LTR553->default_ps_lowthresh = 50;
    }

    setThrDuringCall();

    if (LTR553->ps_enable_flag) {
        dev_info(&LTR553->i2c_client->dev, "%s: already enabled\n", __func__);
        return 0;
    }

    /* Set thresholds where interrupt will *not* be generated */
    #if ACT_INTERRUPT
    //rc = set_ps_range(PS_MIN_MEASURE_VAL, PS_MIN_MEASURE_VAL, LO_N_HI_LIMIT);
    //rc = set_ps_range(PS_MIN_MEASURE_VAL, 400, LO_N_HI_LIMIT);
    rc = set_ps_range(LTR553->default_ps_lowthresh, LTR553->default_ps_highthresh, LO_N_HI_LIMIT, LTR553);
    #else
    rc = set_ps_range(PS_MIN_MEASURE_VAL, PS_MAX_MEASURE_VAL, LO_N_HI_LIMIT, LTR553);
    #endif
    if (rc < 0) {
        dev_err(&LTR553->i2c_client->dev, "%s : PS Thresholds Write Fail...\n", __func__);
        return rc;
    }

    //(Linux RTOS)>
    #if 0
    /* Allows this interrupt to wake the system */
	//rc = irq_set_irq_wake(LTR553->irq, 1);
	rc = set_irq_wake(LTR553->irq, 1);
	if (rc < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: IRQ-%d WakeUp Enable Fail...\n", __func__, LTR553->irq);
		return rc;
	}
#endif
	//(Linux RTOS)<

	rc = ps_led_setup(0x7F, LTR553);
	if (rc < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS LED Setup Fail...\n", __func__);
		return rc;
	}

	rc = ps_ledPulseCount_setup(0x04, LTR553); // pulses = 4
	if (rc < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS LED pulse count setup Fail...\n", __func__);
	}

	rc = ps_meas_rate_setup(10, LTR553);
	if (rc < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS MeasRate Setup Fail...\n", __func__);
		return rc;
	}

	rc = ps_contr_setup(0x03, LTR553);
	if (rc < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS Enable Fail...\n", __func__);
		return rc;
	}

	// dummy read
	buffer[0] = LTR553_PS_CONTR;
	I2C_Read(buffer, 1);
	// dumy read
	
	LTR553->ps_enable_flag = 1;
	

	return rc;
}


/* PS Disable */
static int8_t ps_disable(struct LTR553_data *LTR553)
{
	int8_t rc = 0;

	if (LTR553->ps_enable_flag == 0) {
		dev_info(&LTR553->i2c_client->dev, "%s: already disabled\n", __func__);
		return 0;
	}

	//(Linux RTOS)>
#if 0
	/* Don't allow this interrupt to wake the system anymore */
	//rc = irq_set_irq_wake(LTR553->irq, 0);
	rc = set_irq_wake(LTR553->irq, 0);
	if (rc < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: IRQ-%d WakeUp Disable Fail...\n", __func__, LTR553->irq);
		return rc;
	}
#endif
	//(Linux RTOS)<

	//rc = _LTR553_set_bit(LTR553->i2c_client, CLR_BIT, LTR553_PS_CONTR, PS_MODE);
	rc = ps_mode_setup(CLR_BIT, LTR553);
	if (rc < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS Disable Fail...\n", __func__);
		return rc;
	}

	LTR553->ps_enable_flag = 0;

	return rc;
}


/* PS open fops */
ssize_t ps_open(struct inode *inode, struct file *file)
{
	struct LTR553_data *LTR553 = sensor_info;

	if (LTR553->ps_opened)
		return -EBUSY;

	LTR553->ps_opened = 1;

	return 0;
}


/* PS release fops */
ssize_t ps_release(struct inode *inode, struct file *file)
{
	struct LTR553_data *LTR553 = sensor_info;

	LTR553->ps_opened = 0;

	return ps_disable(LTR553);	
}


/* PS IOCTL */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
static int ps_ioctl (struct inode *ino, struct file *file, unsigned int cmd, unsigned long arg)
#else
static long ps_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
	int rc = 0, val = 0;
	struct LTR553_data *LTR553 = sensor_info;

	pr_info("%s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
		case LTR553_IOCTL_PS_ENABLE:
			if (get_user(val, (unsigned long __user *)arg)) {
				rc = -EFAULT;
				break;
			}
			rc = val ? ps_enable_init(LTR553) : ps_disable(LTR553);

			break;
		case LTR553_IOCTL_PS_GET_ENABLED:
			rc = put_user(LTR553->ps_enable_flag, (unsigned long __user *)arg);

			break;
		default:
			pr_err("%s: INVALID COMMAND %d\n", __func__, _IOC_NR(cmd));
			rc = -EINVAL;
	}

	return rc;
}

static const struct file_operations ps_fops = {
	.owner = THIS_MODULE,
	.open = ps_open,
	.release = ps_release,
	#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
	.ioctl = ps_ioctl
	#else
	.unlocked_ioctl = ps_ioctl
	#endif
};

struct miscdevice ps_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "LTR553_ps",
	.fops = &ps_fops
};


static int8_t als_enable_init(struct LTR553_data *LTR553)
{
	int8_t rc = 0;
	uint8_t buffer[1]; // for dummy read

	/* if device not enabled, enable it */
	if (LTR553->als_enable_flag) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS already enabled...\n", __func__);
		return rc;
	}

	rc = als_meas_rate_reg_setup(0x03, LTR553);
	if (rc < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS_Meas_Rate register Setup Fail...\n", __func__);
		return rc;
	}

	/* Set minimummax thresholds where interrupt will *not* be generated */
#if ACT_INTERRUPT
	//rc = set_als_range(ALS_MIN_MEASURE_VAL, ALS_MIN_MEASURE_VAL, LO_N_HI_LIMIT);
	rc = set_als_range(5, 5, LO_N_HI_LIMIT);
#else
	rc = set_als_range(ALS_MIN_MEASURE_VAL, ALS_MAX_MEASURE_VAL, LO_N_HI_LIMIT);
#endif
	if (rc < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s : ALS Thresholds Write Fail...\n", __func__);
		return rc;
	}

	rc = als_contr_setup(0x0D, LTR553);
	if (rc < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS Enable Fail...\n", __func__);
		return rc;
	}

	// dummy read
	buffer[0] = LTR553_ALS_CONTR;
	I2C_Read(buffer, 1);
	// dumy read
	
	LTR553->als_enable_flag = 1;

	return rc;
}


static int8_t als_disable(struct LTR553_data *LTR553)
{
	int8_t rc = 0;

	if (LTR553->als_enable_flag == 0) {
		dev_err(&LTR553->i2c_client->dev, "%s : ALS already disabled...\n", __func__);
		return rc;
	}

	//rc = _LTR553_set_bit(LTR553->i2c_client, CLR_BIT, LTR553_ALS_CONTR, ALS_MODE);
	rc = als_mode_setup(CLR_BIT, LTR553);
	if (rc < 0) {
		dev_err(&LTR553->i2c_client->dev,"%s: ALS Disable Fail...\n", __func__);
		return rc;
	}
	LTR553->als_enable_flag = 0;

	return rc;
}


ssize_t als_open(struct inode *inode, struct file *file)
{
	struct LTR553_data *LTR553 = sensor_info;
	int8_t rc = 0;

	if (LTR553->als_opened) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS already Opened...\n", __func__);
		rc = -EBUSY;
	}
	LTR553->als_opened = 1;

	return rc;
}


ssize_t als_release(struct inode *inode, struct file *file)
{
	struct LTR553_data *LTR553 = sensor_info;

	LTR553->als_opened = 0;

	//return 0;
	return als_disable(LTR553);
}


#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
static int als_ioctl (struct inode *ino, struct file *file, unsigned int cmd, unsigned long arg)
#else
static long als_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
	int rc = 0, val = 0;
	struct LTR553_data *LTR553 = sensor_info;

	pr_debug("%s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
		case LTR553_IOCTL_ALS_ENABLE:
			if (get_user(val, (unsigned long __user *)arg)) {
				rc = -EFAULT;
				break;
			}
			/*pr_info("%s value = %d\n", __func__, val);*/
			rc = val ? als_enable_init(LTR553) : als_disable(LTR553);

			break;
		case LTR553_IOCTL_ALS_GET_ENABLED:
			val = LTR553->als_enable_flag;
			/*pr_info("%s enabled %d\n", __func__, val);*/
			rc = put_user(val, (unsigned long __user *)arg);

			break;
		default:
			pr_err("%s: INVALID COMMAND %d\n", __func__, _IOC_NR(cmd));
			rc = -EINVAL;
	}

	return rc;
}


static const struct file_operations als_fops = {
	.owner = THIS_MODULE,
	.open = als_open,
	.release = als_release,
	#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
	.ioctl = als_ioctl
	#else
	.unlocked_ioctl = als_ioctl
	#endif
};

static struct miscdevice als_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "LTR553_ls",
	.fops = &als_fops
};


static ssize_t als_adc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u16 value;
	int ret;
	struct LTR553_data *LTR553 = sensor_info;

	//LTR553->mode = ALS;
	//value = read_adc_value(LTR553);
	value = read_als_adc_value(LTR553);
	input_report_abs(LTR553->als_input_dev, ABS_MISC, value);
	input_sync(LTR553->als_input_dev);
	//ret = sprintf(buf, "%d\n", value);
	ret = sprintf(buf, "%d", value);

	return ret;
}

static DEVICE_ATTR(als_adc, 0666, als_adc_show, NULL);


static ssize_t ps_adc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u16 value;
	int ret;
	struct LTR553_data *LTR553 = sensor_info;

	//LTR553->mode = PS;
	//value = read_adc_value(LTR553);
	value = read_ps_adc_value(LTR553);
	//ret = sprintf(buf, "%d\n", value);
	ret = sprintf(buf, "%d", value);
	
	return ret;
}

static DEVICE_ATTR(ps_adc, 0666, ps_adc_show, NULL);


static ssize_t psadcsaturationBit_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u16 value;
	uint8_t saturation_bit;
	int ret;
	uint8_t buffer[3];
	struct LTR553_data *LTR553 = sensor_info;

	buffer[0] = LTR553_PS_DATA_0;
	ret = I2C_Read(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	value = (u16)buffer[0] | ((u16)buffer[1] << 8);
	//LTR553->mode = PS_W_SATURATION_BIT;	
	//value = read_adc_value(LTR553);
	saturation_bit = (value >> 15);
	value &= PS_VALID_MEASURE_MASK;
	ret = sprintf(buf, "%d %d\n", value, saturation_bit);
	
	return ret;
}

static DEVICE_ATTR(psadcsaturationBit, 0666, psadcsaturationBit_show, NULL);


static ssize_t LTR553help_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("To show ALS value : cat als_adc\n");
	printk("To show PS value : cat ps_adc\n");
	printk("To show PS value with saturation bit : cat psadcsaturationBit\n\n");
	
	// address 0x80
	printk("Address 0x80 (ALS_CONTR)\n");
	printk("ALS active mode : echo 1 > alsmodesetup\n");
	printk("ALS standby mode : echo 0 > alsmodesetup\n");
	printk("To read ALS mode : cat alsmodesetup\n\n");

	printk("ALS SW reset : echo 1 > alsswresetsetup\n");
	printk("ALS SW not reset : echo 0 > alsswresetsetup\n");
	printk("To read ALS SW reset bit : cat alsswresetsetup\n\n");

	printk("ALS gain 1x : echo 1 > alsgainsetup\n");
	printk("ALS gain 2x : echo 2 > alsgainsetup\n");
	printk("ALS gain 4x : echo 4 > alsgainsetup\n");
	printk("ALS gain 8x : echo 8 > alsgainsetup\n");
	printk("ALS gain 48x : echo 48 > alsgainsetup\n");
	printk("ALS gain 96x : echo 96 > alsgainsetup\n");
	printk("To read ALS gain : cat alsgainsetup\n\n");

	printk("Write value to ALS_CONTR register (0x80) : echo [hexcode value] > alscontrsetup\n");
	printk("Example...to write 0x0B : echo B > alscontrsetup or echo b > alscontrsetup\n");
	printk("Example...to write 0x13 : echo 13 > alscontrsetup\n");
	printk("To read register ALS_CONTR (0x80) : cat alscontrsetup\n\n");
	// address 0x80

	// address 0x81
	printk("Address 0x81 (PS_CONTR)\n");
	printk("PS active mode : echo 1 > psmodesetup\n");
	printk("PS standby mode : echo 0 > psmodesetup\n");
	printk("To read PS mode : cat psmodesetup\n\n");

	printk("PS gain x16 : echo 16 > psgainsetup\n");
	printk("PS gain x32 : echo 32 > psgainsetup\n");
	printk("PS gain x64 : echo 64 > psgainsetup\n");
	printk("To read PS gain : cat psgainsetup\n\n");

	printk("PS saturation indicator enable : echo 1 > pssatuindicasetup\n");
	printk("PS saturation indicator disable : echo 0 > pssatuindicasetup\n");
	printk("To read back PS saturation indicator : cat pssatuindicasetup\n\n");

	printk("Write value to PS_CONTR register (0x81) : echo [hexcode value] > pscontrsetup\n");
	printk("Example...to write 0x0B : echo B > pscontrsetup or echo b > pscontrsetup\n");
	printk("Example...to write 0x13 : echo 13 > pscontrsetup\n");
	printk("To read register PS_CONTR (0x81) : cat pscontrsetup\n\n");
	// address 0x81

	// address 0x82
	printk("Address 0x82 (PS_LED)\n");
	printk("LED current 5mA : echo 5 > psledcurrsetup\n");
	printk("LED current 10mA : echo 10 > psledcurrsetup\n");
	printk("LED current 20mA : echo 20 > psledcurrsetup\n");
	printk("LED current 50mA : echo 50 > psledcurrsetup\n");
	printk("LED current 100mA : echo 100 > psledcurrsetup\n");
	printk("To read LED current : cat psledcurrsetup\n\n");

	printk("LED current duty 25%% : echo 25 > psledcurrduty\n");
	printk("LED current duty 50%% : echo 50 > psledcurrduty\n");
	printk("LED current duty 75%% : echo 75 > psledcurrduty\n");
	printk("LED current duty 100%% : echo 100 > psledcurrduty\n");
	printk("To read LED current duty : cat psledcurrduty\n\n");

	printk("LED pulse freq 30kHz : echo 30 > psledpulsefreqsetup\n");
	printk("LED pulse freq 40kHz : echo 40 > psledpulsefreqsetup\n");
	printk("LED pulse freq 50kHz : echo 50 > psledpulsefreqsetup\n");
	printk("LED pulse freq 60kHz : echo 60 > psledpulsefreqsetup\n");
	printk("LED pulse freq 70kHz : echo 70 > psledpulsefreqsetup\n");
	printk("LED pulse freq 80kHz : echo 80 > psledpulsefreqsetup\n");
	printk("LED pulse freq 90kHz : echo 90 > psledpulsefreqsetup\n");
	printk("LED pulse freq 100kHz : echo 100 > psledpulsefreqsetup\n");
	printk("To read LED pulse freq : cat psledpulsefreqsetup\n\n");

	printk("Write value to PS_LED register (0x82) : echo [hexcode value] > psledsetup\n");
	printk("Example...to write 0x0B : echo B > psledsetup or echo b > psledsetup\n");
	printk("Example...to write 0x13 : echo 13 > psledsetup\n");
	printk("To read register PS_LED (0x82) : cat psledsetup\n\n");
	// address 0x82

	// address 0x83
	printk("Address 0x83 (PS_N_PULSES)\n");
	printk("To set PS LED pulse count (0x83) : echo [pulse count num] > psledpulsecountsetup\n");
	printk("[pulse count num] must be 0 to 15, inclusive\n");
	printk("Example...to set 0 count : echo 0 > psledpulsecountsetup\n");
	printk("Example...to set 13 counts : echo 13 > psledpulsecountsetup\n");
	printk("To read register PS_N_PULSES (0x83) : cat psledpulsecountsetup\n\n");
	// address 0x83

	// address 0x84
	printk("Address 0x84 (PS_MEAS_RATE)\n");
	printk("PS meas repeat rate 50ms : echo 50 > psmeasratesetup\n");
	printk("PS meas repeat rate 70ms : echo 70 > psmeasratesetup\n");
	printk("PS meas repeat rate 100ms : echo 100 > psmeasratesetup\n");
	printk("PS meas repeat rate 200ms : echo 200 > psmeasratesetup\n");
	printk("PS meas repeat rate 500ms : echo 500 > psmeasratesetup\n");
	printk("PS meas repeat rate 1000ms : echo 1000 > psmeasratesetup\n");
	printk("PS meas repeat rate 2000ms : echo 2000 > psmeasratesetup\n");
	printk("PS meas repeat rate 10ms : echo 10 > psmeasratesetup\n");
	printk("To read register PS_MEAS_RATE (0x84) : cat psmeasratesetup\n\n");
	// address 0x84

	// address 0x85
	printk("Address 0x85 (ALS_MEAS_RATE)\n");
	printk("ALS meas repeat rate 50ms : echo 50 > alsmeasratesetup\n");
	printk("ALS meas repeat rate 100ms : echo 100 > alsmeasratesetup\n");
	printk("ALS meas repeat rate 200ms : echo 200 > alsmeasratesetup\n");
	printk("ALS meas repeat rate 500ms : echo 500 > alsmeasratesetup\n");
	printk("ALS meas repeat rate 1000ms : echo 1000 > alsmeasratesetup\n");
	printk("ALS meas repeat rate 2000ms : echo 2000 > alsmeasratesetup\n");
	printk("To read ALS meas repeat rate : cat alsmeasratesetup\n\n");

	printk("ALS integration time 100ms : echo 100 > alsintegtimesetup\n");
	printk("ALS integration time 50ms : echo 50 > alsintegtimesetup\n");
	printk("ALS integration time 200ms : echo 200 > alsintegtimesetup\n");
	printk("ALS integration time 400ms : echo 400 > alsintegtimesetup\n");
	printk("ALS integration time 150ms : echo 150 > alsintegtimesetup\n");
	printk("ALS integration time 250ms : echo 250 > alsintegtimesetup\n");
	printk("ALS integration time 300ms : echo 300 > alsintegtimesetup\n");
	printk("ALS integration time 350ms : echo 350 > alsintegtimesetup\n");
	printk("To read ALS integration time : cat alsintegtimesetup\n\n");

	printk("Write value to ALS_MEAS register (0x85) : echo [hexcode value] > alsmeasrateregsetup\n");
	printk("Example...to write 0x0B : echo B > alsmeasrateregsetup or echo b > alsmeasrateregsetup\n");
	printk("Example...to write 0x13 : echo 13 > alsmeasrateregsetup\n");
	printk("To read register ALS_MEAS (0x85) : cat alsmeasrateregsetup\n\n");
	// address 0x85

	// address 0x86
	printk("To read part ID : cat partid\n");
	printk("To read revision ID : cat revid\n");
	printk("To read PART_ID register (0x86) : cat partidreg\n\n");
	// address 0x86

	// address 0x87
	printk("To read manufacturing ID : cat manuid\n\n");
	// address 0x87

	// address 0x8C
	printk("Address 0x8C (ALS_PS_STATUS)\n");
	printk("To read PS data status : cat psdatastatus\n");
	printk("To read PS interrupt status : cat psinterruptstatus\n");
	printk("To read ALS data status : cat alsdatastatus\n");
	printk("To read ALS interrupt status : cat alsinterruptstatus\n");
	printk("To read ALS gain status : cat alsgainstatus\n");
	printk("To read ALS validity status : cat alsdatavaliditystatus\n");
	printk("To read register ALS_PS_STATUS (0x8C) : cat alspsstatusreg\n\n");
	// address 0x8C

	// address 0x88, 0x89, 0x8A, 0x8B
	printk("ALS raw and calculated data, address 0x88, 0x89, 0x8A, 0x8B\n");
	printk("To read raw and calculated ALS data : cat alsch0ch1rawcalc\n\n");
	// address 0x88, 0x89, 0x8A, 0x8B

	// address 0x94, 0x95
	printk("To set PS offset (0 ~ 1023) : echo [decimal value] > setpsoffset\n");
	printk("Example...to write 55 : echo 55 > setpsoffset\n");
	printk("To read back the offset value : cat setpsoffset\n\n");
	// address 0x94, 0x95

	// address 0x8F
	printk("Address 0x8F (INTERRUPT)\n");
	printk("INT output pin inactive : echo 0 > interruptmodesetup\n");
	printk("Only PS triggers interrupt : echo 1 > interruptmodesetup\n");
	printk("Only ALS triggers interrupt : echo 2 > interruptmodesetup\n");
	printk("Both ALS PS trigger interrupt : echo 3 > interruptmodesetup\n");
	printk("To read interrupt mode : cat interruptmodesetup\n\n");

	printk("INT output pin active low : echo 0 > interruptpolarsetup\n");
	printk("INT output pin active high : echo 1 > interruptpolarsetup\n");
	printk("To read interrupt pin polarity : cat interruptpolarsetup\n\n");

	printk("Write value to INTERRUPT register (0x8F) : echo [hexcode value] > interruptsetup\n");
	printk("Example...to write 0x0B : echo B > interruptsetup or echo b > interruptsetup\n");
	printk("Example...to write 0x13 : echo 13 > interruptsetup\n");
	printk("To read register INTERRUPT (0x8F) : cat interruptsetup\n\n");
	// address 0x8F

	// address 0x9E
	printk("Address 0x9E (INTERRUPT PERSIST)\n");
	printk("Write value to INTERRUPT register (0x9E) : echo [hexcode value] > interruptpersistsetup\n");
	printk("Example...to write 0x0B : echo B > interruptpersistsetup or echo b > interruptpersistsetup\n");
	printk("Example...to write 0x13 : echo 13 > interruptpersistsetup\n");
	printk("To read register INTERRUPT PERSIST (0x9E) : cat interruptpersistsetup\n\n");
	// address 0x9E

	// ALS threshold setting
	printk("ALS threshold setting 0x97, 0x98, 0x99, 0x9A\n");
	printk("To set ALS lo threshold : echo [lo limit in decimal] > setalslothrerange\n");
	printk("Example...To set 20 to lo threshold : echo 20 > setalslothrerange\n");
	printk("To set ALS hi threshold : echo [hi limit in decimal] > setalshithrerange\n");
	printk("Example...To set 999 to hi threshold : echo 999 > setalshithrerange\n");
	printk("To read the threshold values : cat dispalsthrerange\n\n");	
	// ALS threshold setting

	// PS threshold setting
	printk("PS threshold setting 0x90, 0x91, 0x92, 0x93\n");
	printk("To set PS lo threshold : echo [lo limit in decimal] > setpslothrerange\n");
	printk("Example...To set 20 to lo threshold : echo 20 > setpslothrerange\n");
	printk("To set PS hi threshold : echo [hi limit in decimal] > setpshithrerange\n");
	printk("Example...To set 999 to hi threshold : echo 999 > setpshithrerange\n");
	printk("To read the threshold values : cat disppsthrerange\n\n");
	// PS threshold setting
	
	return 0;
}

static DEVICE_ATTR(LTR553help, 0666, LTR553help_show, NULL);


static ssize_t alsmodesetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = als_contr_readback(ALS_MODE_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS_MODE_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t alsmodesetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;
	struct LTR553_data *LTR553 = sensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = als_mode_setup((uint8_t)param, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS mode setup Fail...\n", __func__);
		return (-1);
	}

	return count;	
}

static DEVICE_ATTR(alsmodesetup, 0666, alsmodesetup_show, alsmodesetup_store);


static ssize_t alsswresetsetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = als_contr_readback(ALS_SWRT_RDBCK, &rdback_val, LTR553);

	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS_SWRT_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t alsswresetsetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;

	struct LTR553_data *LTR553 = sensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = als_sw_reset_setup((uint8_t)param, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS sw reset setup Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(alsswresetsetup, 0666, alsswresetsetup_show, alsswresetsetup_store);


static ssize_t alsgainsetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = als_contr_readback(ALS_GAIN_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS_GAIN_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t alsgainsetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	//int *param_temp = buf;
	int param_temp[2];

	struct LTR553_data *LTR553 = sensor_info;

	//sscanf(buf, "%d", param_temp);
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
	} else if (count == 2) {
		param_temp[0] -= 48;
		param_temp[1] = 0;

		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count >= 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
	}

	param = ((param_temp[0] * 10) + param_temp[1]);
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = als_gain_setup((uint8_t)param, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS gain setup Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(alsgainsetup, 0666, alsgainsetup_show, alsgainsetup_store);


static ssize_t alscontrsetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = als_contr_readback(ALS_CONTR_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS_CONTR_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

static ssize_t alscontrsetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	//int *param_temp = buf;
	int param_temp[2];

	struct LTR553_data *LTR553 = sensor_info;

	//sscanf(buf, "%d", param_temp);
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 48;
		param_temp[1] = 48;
	} else if (count == 2) {
		param_temp[1] = param_temp[0];
		param_temp[0] = 48;
	}


	if (param_temp[0] >= 65 && param_temp[0] <= 70) {
		param_temp[0] -= 55;
	} else if (param_temp[0] >= 97 && param_temp[0] <= 102) {
		param_temp[0] -= 87;
	} else if (param_temp[0] >= 48 && param_temp[0] <= 57) {
		param_temp[0] -= 48;
	} else {
		param_temp[0] = 0;
	}

	if (param_temp[1] >= 65 && param_temp[1] <= 70) {
		param_temp[1] -= 55;
	} else if (param_temp[1] >= 97 && param_temp[1] <= 102) {
		param_temp[1] -= 87;
	} else if (param_temp[1] >= 48 && param_temp[1] <= 57) {
		param_temp[1] -= 48;
	} else {
		param_temp[1] = 0;
	}

	param = ((param_temp[0] << 4) + (param_temp[1]));
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = als_contr_setup(param, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS contr setup Fail...\n", __func__);
		return (-1);
	}

	return count;	
}

static DEVICE_ATTR(alscontrsetup, 0666, alscontrsetup_show, alscontrsetup_store);


static ssize_t psmodesetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = ps_contr_readback(PS_MODE_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS_MODE_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t psmodesetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;
	struct LTR553_data *LTR553 = sensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = ps_mode_setup((uint8_t)param, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS mode setup Fail...\n", __func__);
		return (-1);
	}

	return count;
	
}

static DEVICE_ATTR(psmodesetup, 0666, psmodesetup_show, psmodesetup_store);


static ssize_t psgainsetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = ps_contr_readback(PS_GAIN_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS_GAIN_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t psgainsetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	uint8_t param;
	int8_t ret;
	int param_temp[2];

	struct LTR553_data *LTR553 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 2) {
		param_temp[0] = 0;
		param_temp[1] = 0;
	} else if (count >= 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;		
	}

	//for (i_ctr = 0; i_ctr < sizeof(param_temp); i_ctr++) {
	//	if ((param_temp[i_ctr] < 0) ||(param_temp[i_ctr] > 9)) {
	//		param_temp[i_ctr] = 0;
	//	}
	//}

	param = ((param_temp[0] * 10) + param_temp[1]);
	
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = ps_gain_setup(param, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS gain setup Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(psgainsetup, 0666, psgainsetup_show, psgainsetup_store);


static ssize_t pssatuindicasetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = ps_contr_readback(PS_SATUR_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS_SATUR_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t pssatuindicasetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;
	struct LTR553_data *LTR553 = sensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = ps_satu_indica_setup((uint8_t)param, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS saturation indicator setup Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(pssatuindicasetup, 0666, pssatuindicasetup_show, pssatuindicasetup_store);


static ssize_t pscontrsetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = ps_contr_readback(PS_CONTR_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS_CONTR_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t pscontrsetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	//int *param_temp = buf;
	int param_temp[2];

	struct LTR553_data *LTR553 = sensor_info;

	//sscanf(buf, "%d", param_temp);
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 48;
		param_temp[1] = 48;
	} else if (count == 2) {
		param_temp[1] = param_temp[0];
		param_temp[0] = 48;
	}

	if (param_temp[0] >= 65 && param_temp[0] <= 70) {
		param_temp[0] -= 55;
	} else if (param_temp[0] >= 97 && param_temp[0] <= 102) {
		param_temp[0] -= 87;
	} else if (param_temp[0] >= 48 && param_temp[0] <= 57) {
		param_temp[0] -= 48;
	} else {
		param_temp[0] = 0;
	}

	if (param_temp[1] >= 65 && param_temp[1] <= 70) {
		param_temp[1] -= 55;
	} else if (param_temp[1] >= 97 && param_temp[1] <= 102) {
		param_temp[1] -= 87;
	} else if (param_temp[1] >= 48 && param_temp[1] <= 57) {
		param_temp[1] -= 48;
	} else {
		param_temp[1] = 0;
	}

	param = ((param_temp[0] << 4) + (param_temp[1]));
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = ps_contr_setup(param, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS contr setup Fail...\n", __func__);
		return (-1);
	}

	return count;	
}

static DEVICE_ATTR(pscontrsetup, 0666, pscontrsetup_show, pscontrsetup_store);


static ssize_t psledcurrsetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = ps_led_readback(LED_CURR_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: LED_CURR_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;

}


static ssize_t psledcurrsetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	//int *param_temp = buf;
	int param_temp[3];
	
	struct LTR553_data *LTR553 = sensor_info;

	//sscanf(buf, "%d", param_temp);
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];

	if (count <=1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
	} else if (count == 2) {		
		param_temp[0] -= 48;
		param_temp[1] = 0;
		param_temp[2] = 0;

		param_temp[2] = param_temp[0];
		param_temp[0] = 0;
		param_temp[1] = 0;
	} else if (count == 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;

		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count > 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
	}

	//for (i_ctr = 0; i_ctr < sizeof(param_temp); i_ctr++) {
	//	if ((param_temp[i_ctr] < 0) ||(param_temp[i_ctr] > 9)) {
	//		param_temp[i_ctr] = 0;
	//	}
	//}

	param = ((param_temp[0] * 100) + (param_temp[1] * 10) + param_temp[2]);
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = ps_ledCurrent_setup(param, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS LED current setup Fail...\n", __func__);
		return (-1);
	}

	return count;
	
}

static DEVICE_ATTR(psledcurrsetup, 0666, psledcurrsetup_show, psledcurrsetup_store);


static ssize_t psledcurrduty_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = ps_led_readback(LED_CURR_DUTY_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: LED_CURR_DUTY_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
	
}


static ssize_t psledcurrduty_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	//int *param_temp = buf;
	int param_temp[3];

	struct LTR553_data *LTR553 = sensor_info;

	//sscanf(buf, "%d", param_temp);
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];

	if (count < 3) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
	} else if (count == 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;

		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count > 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
	}

	//for (i_ctr = 0; i_ctr < sizeof(param_temp); i_ctr++) {
	//	if ((param_temp[i_ctr] < 0) ||(param_temp[i_ctr] > 9)) {
	//		param_temp[i_ctr] = 0;
	//	}
	//}

	param = ((param_temp[0] * 100) + (param_temp[1] * 10) + param_temp[2]);
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = ps_ledCurrDuty_setup(param, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS LED curent duty setup Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(psledcurrduty, 0666, psledcurrduty_show, psledcurrduty_store);


static ssize_t psledpulsefreqsetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = ps_led_readback(LED_PUL_FREQ_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: LED_PUL_FREQ_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
	
}


static ssize_t psledpulsefreqsetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	//int *param_temp = buf;
	int param_temp[3];

	struct LTR553_data *LTR553 = sensor_info;

	//sscanf(buf, "%d", param_temp);
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];

	if (count < 3) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
	} else if (count == 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;

		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count > 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
	}

	//for (i_ctr = 0; i_ctr < sizeof(param_temp); i_ctr++) {
	//	if ((param_temp[i_ctr] < 0) ||(param_temp[i_ctr] > 9)) {
	//		param_temp[i_ctr] = 0;
	//	}
	//}

	param = ((param_temp[0] * 100) + (param_temp[1] * 10) + param_temp[2]);
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = ps_ledPulseFreq_setup(param, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS LED pulse frequency setup Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(psledpulsefreqsetup, 0666, psledpulsefreqsetup_show, psledpulsefreqsetup_store);


static ssize_t psledsetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = ps_led_readback(PS_LED_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS_LED_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t psledsetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	//int *param_temp = buf;
	int param_temp[2];

	struct LTR553_data *LTR553 = sensor_info;

	//sscanf(buf, "%d", param_temp);
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 48;
		param_temp[1] = 48;
	} else if (count == 2) {
		param_temp[1] = param_temp[0];
		param_temp[0] = 48;
	}

	if (param_temp[0] >= 65 && param_temp[0] <= 70) {
		param_temp[0] -= 55;
	} else if (param_temp[0] >= 97 && param_temp[0] <= 102) {
		param_temp[0] -= 87;
	} else if (param_temp[0] >= 48 && param_temp[0] <= 57) {
		param_temp[0] -= 48;
	} else {
		param_temp[0] = 0;
	}

	if (param_temp[1] >= 65 && param_temp[1] <= 70) {
		param_temp[1] -= 55;
	} else if (param_temp[1] >= 97 && param_temp[1] <= 102) {
		param_temp[1] -= 87;
	} else if (param_temp[1] >= 48 && param_temp[1] <= 57) {
		param_temp[1] -= 48;
	} else {
		param_temp[1] = 0;
	}

	param = ((param_temp[0] << 4) + (param_temp[1]));
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = ps_led_setup(param, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS LED setup Fail...\n", __func__);
		return (-1);
	}

	return count;	
}

static DEVICE_ATTR(psledsetup, 0666, psledsetup_show, psledsetup_store);


static ssize_t psledpulsecountsetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = ps_ledPulseCount_readback(&rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS LED pulse count readback Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t psledpulsecountsetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	//int *param_temp = buf;
	int param_temp[2];
	
	struct LTR553_data *LTR553 = sensor_info;

	//sscanf(buf, "%d", param_temp);
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if ((count <= 1) || (count > 3)){
		param_temp[0] = 0;
		param_temp[1] = 0;		
	} else if (count == 2) {		
		param_temp[0] -= 48;
		param_temp[1] = 0;

		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count == 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
	}

	//for (i_ctr = 0; i_ctr < sizeof(param_temp); i_ctr++) {
	//	if ((param_temp[i_ctr] < 0) ||(param_temp[i_ctr] > 9)) {
	//		param_temp[i_ctr] = 0;
	//	}
	//}

	param = ((param_temp[0] * 10) + param_temp[1]);
	if (param > 15) {
		param = 15;
	}
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = ps_ledPulseCount_setup(param, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS LED pulse count setup Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(psledpulsecountsetup, 0666, psledpulsecountsetup_show, psledpulsecountsetup_store);


static ssize_t psmeasratesetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = ps_meas_rate_readback(&rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS meas rate readback Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;

}


static ssize_t psmeasratesetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	u16 param;
	//int *param_temp = buf;
	int param_temp[4];
	
	struct LTR553_data *LTR553 = sensor_info;

	//sscanf(buf, "%d", param_temp);
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];

	if (count <= 2) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
	} else if (count == 3) {		
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;

		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count > 4) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;
	}

	//for (i_ctr = 0; i_ctr < sizeof(param_temp); i_ctr++) {
	//	if ((param_temp[i_ctr] < 0) ||(param_temp[i_ctr] > 9)) {
	//		param_temp[i_ctr] = 0;
	//	}
	//}

	param = ((param_temp[0] * 1000) + (param_temp[1] * 100) + (param_temp[2] * 10) + param_temp[3]);
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = ps_meas_rate_setup(param, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS measurement rate setup Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(psmeasratesetup, 0666, psmeasratesetup_show, psmeasratesetup_store);


static ssize_t alsmeasratesetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = als_meas_rate_readback(ALS_MEAS_RPT_RATE_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS_MEAS_RPT_RATE_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;

}


static ssize_t alsmeasratesetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	u16 param;
	//int *param_temp = buf;
	int param_temp[4];
	
	struct LTR553_data *LTR553 = sensor_info;

	//sscanf(buf, "%d", param_temp);
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];

	if (count <= 2) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
	} else if (count == 3) {		
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;

		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count > 4) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;
	}

	//for (i_ctr = 0; i_ctr < sizeof(param_temp); i_ctr++) {
	//	if ((param_temp[i_ctr] < 0) ||(param_temp[i_ctr] > 9)) {
	//		param_temp[i_ctr] = 0;
	//	}
	//}

	param = ((param_temp[0] * 1000) + (param_temp[1] * 100) + (param_temp[2] * 10) + param_temp[3]);
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = als_meas_rate_setup(param, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS measurement rate setup Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(alsmeasratesetup, 0666, alsmeasratesetup_show, alsmeasratesetup_store);


static ssize_t alsintegtimesetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = als_meas_rate_readback(ALS_INTEG_TM_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS_INTEG_TM_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
	
}


static ssize_t alsintegtimesetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	u16 param;
	//int *param_temp = buf;

	int param_temp[3];
	
	struct LTR553_data *LTR553 = sensor_info;

	//sscanf(buf, "%d", param_temp);
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];

	if (count <= 2) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
	} else if (count == 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;

		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count > 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
	}

	//for (i_ctr = 0; i_ctr < sizeof(param_temp); i_ctr++) {
	//	if ((param_temp[i_ctr] < 0) ||(param_temp[i_ctr] > 9)) {
	//		param_temp[i_ctr] = 0;
	//	}
	//}

	param = ((param_temp[0] * 100) + (param_temp[1] * 10) + param_temp[2]);
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = als_integ_time_setup(param, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS integration time setup Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(alsintegtimesetup, 0666, alsintegtimesetup_show, alsintegtimesetup_store);


static ssize_t alsmeasrateregsetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = als_meas_rate_readback(ALS_MEAS_RATE_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS_MEAS_RATE_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
	
}


static ssize_t alsmeasrateregsetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	//int *param_temp = buf;
	int param_temp[2];

	struct LTR553_data *LTR553 = sensor_info;

	//sscanf(buf, "%d", param_temp);
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 48;
		param_temp[1] = 48;
	} else if (count == 2) {
		param_temp[1] = param_temp[0];
		param_temp[0] = 48;
	}

	if (param_temp[0] >= 65 && param_temp[0] <= 70) {
		param_temp[0] -= 55;
	} else if (param_temp[0] >= 97 && param_temp[0] <= 102) {
		param_temp[0] -= 87;
	} else if (param_temp[0] >= 48 && param_temp[0] <= 57) {
		param_temp[0] -= 48;
	} else {
		param_temp[0] = 0;
	}

	if (param_temp[1] >= 65 && param_temp[1] <= 70) {
		param_temp[1] -= 55;
	} else if (param_temp[1] >= 97 && param_temp[1] <= 102) {
		param_temp[1] -= 87;
	} else if (param_temp[1] >= 48 && param_temp[1] <= 57) {
		param_temp[1] -= 48;
	} else {
		param_temp[1] = 0;
	}

	param = ((param_temp[0] << 4) + (param_temp[1]));
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = als_meas_rate_reg_setup(param, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS meas rate register setup Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(alsmeasrateregsetup, 0666, alsmeasrateregsetup_show, alsmeasrateregsetup_store);


static ssize_t partid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;

	ret = part_ID_reg_readback(PART_NUM_ID_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PART_NUM_ID_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(partid, 0666, partid_show, NULL);


static ssize_t revid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;

	ret = part_ID_reg_readback(REVISION_ID_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: REVISION_ID_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(revid, 0666, revid_show, NULL);


static ssize_t partidreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;

	ret = part_ID_reg_readback(PART_ID_REG_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PART_ID_REG_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(partidreg, 0666, partidreg_show, NULL);


static ssize_t manuid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;

	ret = manu_ID_reg_readback(&rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: Manufacturing ID readback Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(manuid, 0666, manuid_show, NULL);


static ssize_t psdatastatus_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = als_ps_status_reg(PS_DATA_STATUS_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS_DATA_STATUS_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;	
}

static DEVICE_ATTR(psdatastatus, 0666, psdatastatus_show, NULL);


static ssize_t psinterruptstatus_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = als_ps_status_reg(PS_INTERR_STATUS_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS_INTERR_STATUS_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(psinterruptstatus, 0666, psinterruptstatus_show, NULL);


static ssize_t alsdatastatus_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = als_ps_status_reg(ALS_DATA_STATUS_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS_DATA_STATUS_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(alsdatastatus, 0666, alsdatastatus_show, NULL);


static ssize_t alsinterruptstatus_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = als_ps_status_reg(ALS_INTERR_STATUS_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS_INTERR_STATUS_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(alsinterruptstatus, 0666, alsinterruptstatus_show, NULL);


static ssize_t alsgainstatus_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = als_ps_status_reg(ALS_GAIN_STATUS_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS_GAIN_STATUS_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(alsgainstatus, 0666, alsgainstatus_show, NULL);


static ssize_t alsdatavaliditystatus_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;

	ret = als_ps_status_reg(ALS_VALID_STATUS_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS_VALID_STATUS_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(alsdatavaliditystatus, 0666, alsdatavaliditystatus_show, NULL);


static ssize_t alspsstatusreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = als_ps_status_reg(ALS_PS_STATUS_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS_PS_STATUS_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
	
}

static DEVICE_ATTR(alspsstatusreg, 0666, alspsstatusreg_show, NULL);


static ssize_t alsch0ch1rawcalc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	u16 rdback_val1 = 0, rdback_val2 = 0, rdback_val3 = 0;
	struct LTR553_data *LTR553 = sensor_info;

	ret = als_ch0ch1raw_calc_readback(&rdback_val1, &rdback_val2, &rdback_val3, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS CH0 CH1 Calc reading readback Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d %d %d\n", rdback_val1, rdback_val2, rdback_val3);

	return ret;
		
}

static DEVICE_ATTR(alsch0ch1rawcalc, 0666, alsch0ch1rawcalc_show, NULL);


static ssize_t setpsoffset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	u16 rdback_val;
	struct LTR553_data *LTR553 = sensor_info;

	ret = ps_offset_readback(&rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS offset readback Fail...\n", __func__);
		return (-1);
	}
	
	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t setpsoffset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	u16 ps_offset = 0;
	uint8_t param_temp[4];
	struct LTR553_data *LTR553 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
	} else if (count == 2) { // 1 digit
		param_temp[0] -= 48;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[0];
		param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 3) { // 2 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) { // 3 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;

		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count >= 5) { // 4 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;		
	}

	//for (i_ctr = 0; i_ctr < sizeof(param_temp); i_ctr++) {
	//	if ((param_temp[i_ctr] < 0) ||(param_temp[i_ctr] > 9)) {
	//		param_temp[i_ctr] = 0;
	//	}
	//}

	ps_offset = ((param_temp[0] * 1000) + (param_temp[1] * 100) + (param_temp[2] * 10) + param_temp[3]);
	if (ps_offset > 1023) {
		ps_offset = 1023;
	}
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, ps_offset);

	ret = ps_offset_setup(ps_offset, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: set ps offset Fail...\n", __func__);
		return (-1);
	}
	
	return count;
}

static DEVICE_ATTR(setpsoffset, 0666, setpsoffset_show, setpsoffset_store);


static ssize_t interruptmodesetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = interrupt_readback(INT_MODE_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: INT_MODE_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t interruptmodesetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;

	struct LTR553_data *LTR553 = sensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = interrupt_mode_setup((uint8_t)param, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: interrupt mode setup Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(interruptmodesetup, 0666, interruptmodesetup_show, interruptmodesetup_store);


static ssize_t interruptpolarsetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = interrupt_readback(INT_POLAR_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: INT_POLAR_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;	
}


static ssize_t interruptpolarsetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;

	struct LTR553_data *LTR553 = sensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = interrupt_polarity_setup((uint8_t)param, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: interrupt polarity setup Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(interruptpolarsetup, 0666, interruptpolarsetup_show, interruptpolarsetup_store);


static ssize_t interruptsetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = interrupt_readback(INT_INTERRUPT_RDBCK, &rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: INT_INTERRUPT_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;	
}


static ssize_t interruptsetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	//int *param_temp = buf;
	int param_temp[2];

	struct LTR553_data *LTR553 = sensor_info;

	//sscanf(buf, "%d", param_temp);
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 48;
		param_temp[1] = 48;
	} else if (count == 2) {
		param_temp[1] = param_temp[0];
		param_temp[0] = 48;
	}

	if (param_temp[0] >= 65 && param_temp[0] <= 70) {
		param_temp[0] -= 55;
	} else if (param_temp[0] >= 97 && param_temp[0] <= 102) {
		param_temp[0] -= 87;
	} else if (param_temp[0] >= 48 && param_temp[0] <= 57) {
		param_temp[0] -= 48;
	} else {
		param_temp[0] = 0;
	}

	if (param_temp[1] >= 65 && param_temp[1] <= 70) {
		param_temp[1] -= 55;
	} else if (param_temp[1] >= 97 && param_temp[1] <= 102) {
		param_temp[1] -= 87;
	} else if (param_temp[1] >= 48 && param_temp[1] <= 57) {
		param_temp[1] -= 48;
	} else {
		param_temp[1] = 0;
	}

	param = ((param_temp[0] << 4) + (param_temp[1]));
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = interrupt_setup(param, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: interrupt setup Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(interruptsetup, 0666, interruptsetup_show, interruptsetup_store);


static ssize_t interruptpersistsetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR553_data *LTR553 = sensor_info;	

	ret = interrupt_prst_readback(&rdback_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: Interrupt persist readback Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t interruptpersistsetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret = 0;
	//uint8_t als_or_ps, prst_val;
	uint8_t prst_val;
	//int *param_temp = buf;
	int param_temp[2];

	struct LTR553_data *LTR553 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 48;
		param_temp[1] = 48;
	} else if (count == 2) {
		param_temp[1] = param_temp[0];
		param_temp[0] = 48;
	}

	if (param_temp[0] >= 65 && param_temp[0] <= 70) {
		param_temp[0] -= 55;
	} else if (param_temp[0] >= 97 && param_temp[0] <= 102) {
		param_temp[0] -= 87;
	} else if (param_temp[0] >= 48 && param_temp[0] <= 57) {
		param_temp[0] -= 48;
	} else {
		param_temp[0] = 0;
	}

	if (param_temp[1] >= 65 && param_temp[1] <= 70) {
		param_temp[1] -= 55;
	} else if (param_temp[1] >= 97 && param_temp[1] <= 102) {
		param_temp[1] -= 87;
	} else if (param_temp[1] >= 48 && param_temp[1] <= 57) {
		param_temp[1] -= 48;
	} else {
		param_temp[1] = 0;
	}

	prst_val = ((param_temp[0] << 4) + (param_temp[1]));
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, prst_val);

	//ret = interrupt_persist_setup(als_or_ps, prst_val, LTR553);
	ret = interrupt_persist_setup(prst_val, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: Interrupt persist setup Fail...\n", __func__);
		return (-1);
	}

	return count;
		
}

static DEVICE_ATTR(interruptpersistsetup, 0666, interruptpersistsetup_show, interruptpersistsetup_store);


static ssize_t setalslothrerange_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	int lo_thr = 0;
	uint8_t param_temp[5];
	struct LTR553_data *LTR553 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];
	param_temp[4] = buf[4];

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
		param_temp[4] = 0;
	} else if (count == 2) { // 1 digit
		param_temp[0] -= 48;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
		param_temp[4] = 0;

		param_temp[4] = param_temp[0];
		param_temp[3] = 0;
		param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 3) { // 2 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;
		param_temp[4] = 0;

		param_temp[4] = param_temp[1];
		param_temp[3] = param_temp[0];
		param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) { // 3 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;
		param_temp[4] = 0;

		param_temp[4] = param_temp[2];
		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 5) { // 4 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;
		param_temp[4] = 0;

		param_temp[4] = param_temp[3];
		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count >= 6) { // 5 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;
		param_temp[4] -= 48;
	}

	//for (i_ctr = 0; i_ctr < sizeof(param_temp); i_ctr++) {
	//	if ((param_temp[i_ctr] < 0) ||(param_temp[i_ctr] > 9)) {
	//		param_temp[i_ctr] = 0;
	//	}
	//}

	lo_thr = ((param_temp[0] * 10000) + (param_temp[1] * 1000) + (param_temp[2] * 100) + (param_temp[3] * 10) + param_temp[4]);
	if (lo_thr > 65535) {
		lo_thr = 65535;
	}
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, lo_thr);

	ret = set_als_range((u16)lo_thr, 0, LO_LIMIT);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: set ALS lo threshold Fail...\n", __func__);
		return (-1);
	}
	
	return count;
}

static DEVICE_ATTR(setalslothrerange, 0666, NULL, setalslothrerange_store);


static ssize_t setalshithrerange_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	int hi_thr = 0;
	uint8_t param_temp[5];
	struct LTR553_data *LTR553 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];
	param_temp[4] = buf[4];

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
		param_temp[4] = 0;
	} else if (count == 2) { // 1 digit
		param_temp[0] -= 48;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
		param_temp[4] = 0;

		param_temp[4] = param_temp[0];
		param_temp[3] = 0;
		param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 3) { // 2 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;
		param_temp[4] = 0;

		param_temp[4] = param_temp[1];
		param_temp[3] = param_temp[0];
		param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) { // 3 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;
		param_temp[4] = 0;

		param_temp[4] = param_temp[2];
		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 5) { // 4 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;
		param_temp[4] = 0;

		param_temp[4] = param_temp[3];
		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count >= 6) { // 5 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;
		param_temp[4] -= 48;
	}

	//for (i_ctr = 0; i_ctr < sizeof(param_temp); i_ctr++) {
	//	if ((param_temp[i_ctr] < 0) ||(param_temp[i_ctr] > 9)) {
	//		param_temp[i_ctr] = 0;
	//	}
	//}

	hi_thr = ((param_temp[0] * 10000) + (param_temp[1] * 1000) + (param_temp[2] * 100) + (param_temp[3] * 10) + param_temp[4]);
	if (hi_thr > 65535) {
		hi_thr = 65535;
	}
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, hi_thr);

	ret = set_als_range(0, (u16)hi_thr, HI_LIMIT);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: set ALS hi threshold Fail...\n", __func__);
		return (-1);
	}
	
	return count;
}

static DEVICE_ATTR(setalshithrerange, 0666, NULL, setalshithrerange_store);


#if 0
static ssize_t setwinfac1_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	uint8_t param_temp[4];
	struct LTR553_data *LTR553 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];

	setWinFac(&winfac1, param_temp, count);

	dev_dbg(&LTR553->i2c_client->dev, "%s: winfac1 value = %d\n", __func__, winfac1);
	
	if (winfac1 > 200) {
		winfac1 = 200;
	}

	return count;
}


static ssize_t setwinfac1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;

	//struct LTR553_data *LTR553 = sensor_info;

	ret = sprintf(buf, "%d\n", winfac1);

	return ret;
}

static DEVICE_ATTR(setwinfac1, 0666, setwinfac1_show, setwinfac1_store);


static ssize_t setwinfac2_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	uint8_t param_temp[4];
	struct LTR553_data *LTR553 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];

	setWinFac(&winfac2, param_temp, count);

	dev_dbg(&LTR553->i2c_client->dev, "%s: winfac2 value = %d\n", __func__, winfac2);
	
	if (winfac2 > 200) {
		winfac2 = 200;
	}

	return count;
}


static ssize_t setwinfac2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;

	//struct LTR553_data *LTR553 = sensor_info;

	ret = sprintf(buf, "%d\n", winfac2);

	return ret;
}

static DEVICE_ATTR(setwinfac2, 0666, setwinfac2_show, setwinfac2_store);


static ssize_t setwinfac3_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	uint8_t param_temp[4];
	struct LTR553_data *LTR553 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];

	setWinFac(&winfac3, param_temp, count);

	dev_dbg(&LTR553->i2c_client->dev, "%s: winfac1 value = %d\n", __func__, winfac3);
	
	if (winfac3 > 200) {
		winfac3 = 200;
	}

	return count;
}


static ssize_t setwinfac3_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;

	//struct LTR553_data *LTR553 = sensor_info;

	ret = sprintf(buf, "%d\n", winfac3);

	return ret;
}

static DEVICE_ATTR(setwinfac3, 0666, setwinfac3_show, setwinfac3_store);
#endif


static ssize_t dispalsthrerange_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	u16 rdback_lo, rdback_hi;
	struct LTR553_data *LTR553 = sensor_info;

	ret = als_range_readback(&rdback_lo, &rdback_hi, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS threshold range readback Fail...\n", __func__);
		return (-1);
	}
	
	ret = sprintf(buf, "%d %d\n", rdback_lo, rdback_hi);

	return ret;
}

static DEVICE_ATTR(dispalsthrerange, 0666, dispalsthrerange_show, NULL);


static ssize_t setpslothrerange_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	u16 lo_thr = 0;
	uint8_t param_temp[4];
	struct LTR553_data *LTR553 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
	} else if (count == 2) { // 1 digit
		param_temp[0] -= 48;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[0];
		param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 3) { // 2 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) { // 3 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;

		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count >= 5) { // 4 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;		
	}

	//for (i_ctr = 0; i_ctr < sizeof(param_temp); i_ctr++) {
	//	if ((param_temp[i_ctr] < 0) ||(param_temp[i_ctr] > 9)) {
	//		param_temp[i_ctr] = 0;
	//	}
	//}

	lo_thr = ((param_temp[0] * 1000) + (param_temp[1] * 100) + (param_temp[2] * 10) + param_temp[3]);
	if (lo_thr > 2047) {
		lo_thr = 2047;
	}
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, lo_thr);

	ret = set_ps_range(lo_thr, 0, LO_LIMIT, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: set PS lo threshold Fail...\n", __func__);
		return (-1);
	}
	
	return count;
}

static DEVICE_ATTR(setpslothrerange, 0666, NULL, setpslothrerange_store);


static ssize_t setpshithrerange_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	u16 hi_thr = 0;
	uint8_t param_temp[4];
	struct LTR553_data *LTR553 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
	} else if (count == 2) { // 1 digit
		param_temp[0] -= 48;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[0];
		param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 3) { // 2 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) { // 3 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;

		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count >= 5) { // 4 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;		
	}

	//for (i_ctr = 0; i_ctr < sizeof(param_temp); i_ctr++) {
	//	if ((param_temp[i_ctr] < 0) ||(param_temp[i_ctr] > 9)) {
	//		param_temp[i_ctr] = 0;
	//	}
	//}

	hi_thr = ((param_temp[0] * 1000) + (param_temp[1] * 100) + (param_temp[2] * 10) + param_temp[3]);
	if (hi_thr > 2047) {
		hi_thr = 2047;
	}
	dev_dbg(&LTR553->i2c_client->dev, "%s: store value = %d\n", __func__, hi_thr);

	ret = set_ps_range(0, hi_thr, HI_LIMIT, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: set PS hi threshold Fail...\n", __func__);
		return (-1);
	}
	
	return count;
}

static DEVICE_ATTR(setpshithrerange, 0666, NULL, setpshithrerange_store);


static ssize_t disppsthrerange_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	u16 rdback_lo, rdback_hi;
	struct LTR553_data *LTR553 = sensor_info;

	ret = ps_range_readback(&rdback_lo, &rdback_hi, LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS threshold range readback Fail...\n", __func__);
		return (-1);
	}
	
	ret = sprintf(buf, "%d %d\n", rdback_lo, rdback_hi);

	return ret;
}

static DEVICE_ATTR(disppsthrerange, 0666, disppsthrerange_show, NULL);


static void sysfs_register_device(struct i2c_client *client) 
{
	int rc = 0;

	rc += device_create_file(&client->dev, &dev_attr_als_adc);
	rc += device_create_file(&client->dev, &dev_attr_ps_adc);
	//rc += device_create_file(&client->dev, &dev_attr_setwinfac1);
	//rc += device_create_file(&client->dev, &dev_attr_setwinfac2);
	//rc += device_create_file(&client->dev, &dev_attr_setwinfac3);
	rc += device_create_file(&client->dev, &dev_attr_psadcsaturationBit);
	rc += device_create_file(&client->dev, &dev_attr_LTR553help);
	rc += device_create_file(&client->dev, &dev_attr_alsmodesetup);
	rc += device_create_file(&client->dev, &dev_attr_alsswresetsetup);
	rc += device_create_file(&client->dev, &dev_attr_alsgainsetup);
	rc += device_create_file(&client->dev, &dev_attr_alscontrsetup);
	rc += device_create_file(&client->dev, &dev_attr_psmodesetup);
	rc += device_create_file(&client->dev, &dev_attr_psgainsetup);
	rc += device_create_file(&client->dev, &dev_attr_pssatuindicasetup);	
	rc += device_create_file(&client->dev, &dev_attr_pscontrsetup);	
	rc += device_create_file(&client->dev, &dev_attr_psledcurrsetup);	
	rc += device_create_file(&client->dev, &dev_attr_psledcurrduty);	
	rc += device_create_file(&client->dev, &dev_attr_psledpulsefreqsetup);
	rc += device_create_file(&client->dev, &dev_attr_psledsetup);
	rc += device_create_file(&client->dev, &dev_attr_psledpulsecountsetup);
	rc += device_create_file(&client->dev, &dev_attr_psmeasratesetup);
	rc += device_create_file(&client->dev, &dev_attr_alsmeasratesetup);
	rc += device_create_file(&client->dev, &dev_attr_alsintegtimesetup);
	rc += device_create_file(&client->dev, &dev_attr_alsmeasrateregsetup);	
	rc += device_create_file(&client->dev, &dev_attr_partid);
	rc += device_create_file(&client->dev, &dev_attr_revid);
	rc += device_create_file(&client->dev, &dev_attr_partidreg);
	rc += device_create_file(&client->dev, &dev_attr_manuid);	
	rc += device_create_file(&client->dev, &dev_attr_psdatastatus);
	rc += device_create_file(&client->dev, &dev_attr_psinterruptstatus);
	rc += device_create_file(&client->dev, &dev_attr_alsdatastatus);
	rc += device_create_file(&client->dev, &dev_attr_alsinterruptstatus);
	rc += device_create_file(&client->dev, &dev_attr_alsgainstatus);
	rc += device_create_file(&client->dev, &dev_attr_alsdatavaliditystatus);
	rc += device_create_file(&client->dev, &dev_attr_alspsstatusreg);
	rc += device_create_file(&client->dev, &dev_attr_alsch0ch1rawcalc);
	rc += device_create_file(&client->dev, &dev_attr_setpsoffset);	
	rc += device_create_file(&client->dev, &dev_attr_interruptmodesetup);
	rc += device_create_file(&client->dev, &dev_attr_interruptpolarsetup);	
	rc += device_create_file(&client->dev, &dev_attr_interruptsetup);
	rc += device_create_file(&client->dev, &dev_attr_interruptpersistsetup);	
	rc += device_create_file(&client->dev, &dev_attr_setalslothrerange);
	rc += device_create_file(&client->dev, &dev_attr_setalshithrerange);
	rc += device_create_file(&client->dev, &dev_attr_dispalsthrerange);
	rc += device_create_file(&client->dev, &dev_attr_setpslothrerange);
	rc += device_create_file(&client->dev, &dev_attr_setpshithrerange);
	rc += device_create_file(&client->dev, &dev_attr_disppsthrerange);
	
	if (rc) {
		dev_err(&client->dev, "%s Unable to create sysfs files\n", __func__);
	} else {
		dev_dbg(&client->dev, "%s Created sysfs files\n", __func__);
	}
}

static ssize_t ltr553_show_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	
	return sprintf(buf, "%d\n", sensor_info->als_enable_flag ? 1:0);
}

static ssize_t ltr553_store_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct LTR553_data *LTR553 = sensor_info;
	int ret = 0;
	unsigned long val = simple_strtoul(buf, NULL, 10);

	printk(KERN_INFO "Ltr553 enable ALS sensor -> %ld\n", val);

	if ((val != 0) && (val != 1))
	{
		printk("%s: enable als sensor=%ld\n", __func__, val);
		return count;
	}

	if(val == 1)
	{
		//turn on light  sensor
		ret = als_enable_init(LTR553);
	}
	else
	{
		als_disable(LTR553);
	}

	return count;
}

static ssize_t ltr553_show_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	
	return sprintf(buf, "%d\n", sensor_info->ps_enable_flag ? 1:0);
}

static ssize_t ltr553_store_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct LTR553_data *LTR553 = sensor_info;
	int ret = 0;
	unsigned long val = simple_strtoul(buf, NULL, 10);

	printk(KERN_INFO "Ltr553 enable ALS sensor -> %ld\n", val);

	if ((val != 0) && (val != 1))
	{
		printk("%s: enable als sensor=%ld\n", __func__, val);
		return count;
	}

	if(val == 1)
	{
		//turn on light  sensor
		ret = ps_enable_init(LTR553);
	}
	else
	{
		ps_disable(LTR553);
	}

	return count;
}


static ssize_t ltr553_vendor_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	 return sprintf(buf, "ltr553\n");
}

static DEVICE_ATTR(vendor_name, S_IRUGO,
		ltr553_vendor_name_show, NULL);

static DEVICE_ATTR(als_onoff,  0664 ,
				  ltr553_show_enable_als_sensor, ltr553_store_enable_als_sensor);

static DEVICE_ATTR(proximity_onoff,  0664 ,
				   ltr553_show_enable_ps_sensor, ltr553_store_enable_ps_sensor);

#if 0
static DEVICE_ATTR(ps_thres_high,  0664 ,
				  ltr553_show_ps_thres_high, ltr553_store_ps_thres_high);

static DEVICE_ATTR(ps_thres_low,  0664 ,
				   ltr553_show_ps_thres_low, ltr553_store_ps_thres_low);

static DEVICE_ATTR(als_poll_delay,  0664,
				    ltr553_show_als_poll_delay, ltr553_store_als_poll_delay);

static DEVICE_ATTR(ps_calib,  0664 ,
				   ltr553_show_ps_calib, ltr553_store_ps_calib);
#endif

static struct attribute *ltr553_attributes[] = {
	&dev_attr_proximity_onoff.attr,
	&dev_attr_als_onoff.attr,
//	&dev_attr_als_poll_delay.attr,
//	&dev_attr_ps_thres_high.attr,
//	&dev_attr_ps_thres_low.attr,
//	&dev_attr_ps_calib.attr,
	&dev_attr_vendor_name.attr,
	NULL
};

static const struct attribute_group ltr553_attr_group = {
	.attrs = ltr553_attributes,
};

static int als_setup(struct LTR553_data *LTR553)
{
	int ret;

	LTR553->als_input_dev = input_allocate_device();
	if (!LTR553->als_input_dev) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS Input Allocate Device Fail...\n", __func__);
		return -ENOMEM;
	}
	LTR553->ps_input_dev = LTR553->als_input_dev;
	
	LTR553->als_input_dev->name = DEVICE_NAME;
	set_bit(EV_ABS, LTR553->als_input_dev->evbit);
	input_set_abs_params(LTR553->ps_input_dev, ABS_DISTANCE, PS_MIN_MEASURE_VAL, PS_MAX_MEASURE_VAL, 0, 0);
	input_set_abs_params(LTR553->als_input_dev, ABS_MISC, ALS_MIN_MEASURE_VAL, ALS_MAX_MEASURE_VAL, 0, 0);

	ret = input_register_device(LTR553->als_input_dev);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS Register Input Device Fail...\n", __func__);
		goto err_als_register_input_device;
	}
#if 0
	ret = misc_register(&als_misc);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS Register Misc Device Fail...\n", __func__);
		goto err_als_register_misc_device;
	}
#endif
	return ret;

err_als_register_misc_device:
	input_unregister_device(LTR553->als_input_dev);
err_als_register_input_device:
	input_free_device(LTR553->als_input_dev);

	return ret;
}


static int ps_setup(struct LTR553_data *LTR553)
{
	int ret;

	LTR553->ps_input_dev = input_allocate_device();
	if (!LTR553->ps_input_dev) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS Input Allocate Device Fail...\n", __func__);
		return -ENOMEM;
	}
	LTR553->ps_input_dev->name = "LTR553_ps";
	set_bit(EV_ABS, LTR553->ps_input_dev->evbit);
	input_set_abs_params(LTR553->ps_input_dev, ABS_DISTANCE, PS_MIN_MEASURE_VAL, PS_MAX_MEASURE_VAL, 0, 0);

	ret = input_register_device(LTR553->ps_input_dev);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS Register Input Device Fail...\n", __func__);
		goto err_ps_register_input_device;
	}

	ret = misc_register(&ps_misc);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS Register Misc Device Fail...\n", __func__);
		goto err_ps_register_misc_device;
	}

	return ret;

err_ps_register_misc_device:
	input_unregister_device(LTR553->ps_input_dev);
err_ps_register_input_device:
	input_free_device(LTR553->ps_input_dev);

	return ret;
}


static int8_t _check_part_id(struct LTR553_data *LTR553)
{
	int8_t ret;
	uint8_t buffer[2];

	buffer[0] = LTR553_PART_ID;
	ret = I2C_Read(buffer, 1);
    printk("[kernel] [%s %d][ID=0x%X] ............\n", __func__, __LINE__, buffer[0]) ;
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: Read failure :0x%02X",
		        __func__, buffer[0]);
		return -1;
	}

	if (buffer[0] != PARTID) {
		dev_err(&LTR553->i2c_client->dev, "%s: Part failure miscompare"
		        " act:0x%02x exp:0x%02x\n", __func__, buffer[0], PARTID);
		return -2;
	}

	return 0;
}


static int LTR553_setup(struct LTR553_data *LTR553)
{
	int ret = 0;

	/* Reset the devices */
	ret = _LTR553_set_bit(LTR553->i2c_client, SET_BIT, LTR553_ALS_CONTR, ALS_SW_RESET);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS reset fail...\n", __func__);
		goto err_out1;
	}

	ret = _LTR553_set_bit(LTR553->i2c_client, CLR_BIT, LTR553_PS_CONTR, PS_MODE_ACTIVE);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS reset fail...\n", __func__);
		goto err_out1;
	}

	msleep(PON_DELAY);
	dev_dbg(&LTR553->i2c_client->dev, "%s: Reset LTR553 device\n", __func__);

	/* Do another part read to ensure we have exited reset */
	if (_check_part_id(LTR553) < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: Part ID Read Fail after reset...\n", __func__);
		goto err_out1;
	}

	//(Linux RTOS)>
#if 1
	ret = LTR553_gpio_irq(LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: GPIO Request Fail...\n", __func__);
		goto err_out1;
	}
	dev_dbg(&LTR553->i2c_client->dev, "%s Requested interrupt\n", __func__);
#endif
	//(Linux RTOS)<

	/* Set count of measurements outside data range before interrupt is generated */
	ret = _LTR553_set_bit(LTR553->i2c_client, SET_BIT, LTR553_INTERRUPT_PRST, 0x01);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: ALS Set Persist Fail...\n", __func__);
		goto err_out2;
	}

	ret = _LTR553_set_bit(LTR553->i2c_client, SET_BIT, LTR553_INTERRUPT_PRST, 0x10);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev,"%s: PS Set Persist Fail...\n", __func__);
		goto err_out2;
	}
	dev_dbg(&LTR553->i2c_client->dev, "%s: Set LTR553 persists\n", __func__);

	/* Enable interrupts on the device and clear only when status is read */
#if ACT_INTERRUPT
	//ret = _LTR553_set_bit(LTR553->i2c_client, SET_BIT, LTR553_INTERRUPT, INT_MODE_ALSPS_TRIG);
	ret = _LTR553_set_bit(LTR553->i2c_client, SET_BIT, LTR553_INTERRUPT, INT_MODE_ALSPS_TRIG);
#else
	ret = _LTR553_set_bit(LTR553->i2c_client, SET_BIT, LTR553_INTERRUPT, INT_MODE_00);
#endif
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: Enabled interrupts failed...\n", __func__);
		goto err_out2;
	}
	dev_dbg(&LTR553->i2c_client->dev, "%s Enabled interrupt to device\n", __func__);
#if 0
	/* Turn on ALS and PS */
	ret = als_enable_init(LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s Unable to enable ALS", __func__);
		goto err_out2;
	}
	dev_info(&LTR553->i2c_client->dev, "%s Turned on ambient light sensor\n", __func__);

	ret = ps_enable_init(LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s Unable to enable PS", __func__);
		goto err_out2;
	}
	dev_info(&LTR553->i2c_client->dev, "%s Turned on proximity sensor\n", __func__);
#endif
	return ret;

err_out2:
	free_irq(LTR553->irq, LTR553);
	gpio_free(LTR553->gpio_int_no);

err_out1:
	dev_err(&LTR553->i2c_client->dev, "%s Unable to setup device\n", __func__);

	return ret;
}


//(Linux RTOS)>
#if 0
static void LTR553_early_suspend(struct early_suspend *h)
{
	int ret = 0;
	struct LTR553_data *LTR553 = sensor_info;

	if (LTR553->is_suspend != 0) {
		dev_err(&LTR553->i2c_client->dev, "%s Asked to suspend when already suspended\n", __func__);
		return;
	}
	LTR553->is_suspend = 1;

	/* Save away the state of the devices at suspend point */
	LTR553->als_suspend_enable_flag = LTR553->als_enable_flag;
	LTR553->ps_suspend_enable_flag = LTR553->ps_enable_flag;

	/* Disable the devices for suspend if configured */
	if (LTR553->disable_als_on_suspend && LTR553->als_enable_flag) {
		ret += als_disable(LTR553);
	}
	if (LTR553->disable_ps_on_suspend && LTR553->ps_enable_flag) {
		ret += ps_disable(LTR553);
	}

	if (ret) {
		dev_err(&LTR553->i2c_client->dev, "%s Unable to complete suspend\n", __func__);
	} else {
		dev_info(&LTR553->i2c_client->dev, "%s Suspend completed\n", __func__);
	}
}


static void LTR553_late_resume(struct early_suspend *h)
{
	struct LTR553_data *LTR553 = sensor_info;
	int ret = 0;

	if (LTR553->is_suspend != 1) {
		dev_err(&LTR553->i2c_client->dev, "%s Asked to resume when not suspended\n", __func__);
		return;
	}
	LTR553->is_suspend = 0;

	/* If ALS was enbled before suspend, enable during resume */
	if (LTR553->als_suspend_enable_flag) {
		ret += als_enable_init(LTR553);
		LTR553->als_suspend_enable_flag = 0;
	}

	/* If PS was enbled before suspend, enable during resume */
	if (LTR553->ps_suspend_enable_flag) {
		ret += ps_enable_init(LTR553);
		LTR553->ps_suspend_enable_flag = 0;
	}

	if (ret) {
		dev_err(&LTR553->i2c_client->dev, "%s Unable to complete resume\n", __func__);
	} else {
		dev_info(&LTR553->i2c_client->dev, "%s Resume completed\n", __func__);
	}
}
#endif
//(Linux RTOS)<


static int LTR553_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct LTR553_data *LTR553;
//(Linux RTOS)>
#if 1
	struct als_ps_platform_data *platdata;
#endif
//(Linux RTOS)<
	if(CHECK_THIS_DEV_DEBUG_AREADY_EXIT()==0)
	{ 
	   return -EIO;
	}
	LTR553 = kzalloc(sizeof(struct LTR553_data), GFP_KERNEL);
	if (!LTR553)
	{
		dev_err(&LTR553->i2c_client->dev, "%s: Mem Alloc Fail...\n", __func__);
		return -ENOMEM;
	}

	/* Global pointer for this device */
	sensor_info = LTR553;

	/* Set initial defaults */
	LTR553->als_enable_flag = 0;
	LTR553->ps_enable_flag = 0;

	LTR553->i2c_client = client;
	//LTR553->irq = client->irq;   //delete by hzb

	i2c_set_clientdata(client, LTR553);

	/* Parse the platform data */
	//(Linux RTOS)>
#if 1
	platdata = client->dev.platform_data;
	if (!platdata) {
		dev_err(&LTR553->i2c_client->dev, "%s: Platform Data assign Fail...\n", __func__);
		ret = -EBUSY;
		goto err_out;
	}

	LTR553->gpio_int_no = platdata->int_gpio; //wg
	//LTR553->adc_levels = platdata->pfd_levels;
	LTR553->default_ps_lowthresh = platdata->prox_threshold_lo;  //modify by wg
	LTR553->default_ps_highthresh = platdata->prox_threshold_hi;  //modify by wg

	/* Configuration to set or disable devices upon suspend */
	//LTR553->disable_als_on_suspend = platdata->pfd_disable_als_on_suspend;
	//LTR553->disable_ps_on_suspend = platdata->pfd_disable_ps_on_suspend;
#endif
	//(Linux RTOS)<
//+add by hzb
	ltr553_prox_max = LTR553->default_ps_highthresh;
	ltr553_prox_min = LTR553->default_ps_lowthresh;
	client->irq=gpio_to_irq(LTR553->gpio_int_no); 
	printk(KERN_INFO "%s: gpio_int_no=%d; client->irq=%d\n",__func__,LTR553->gpio_int_no,client->irq);
	LTR553->irq = client->irq;
	LTR553->power_on = platdata->power_on;  //wg
	
	if (LTR553->power_on) LTR553->power_on(1);
//-add by hzb

	if (_check_part_id(LTR553) < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: Part ID Read Fail...\n", __func__);
		goto err_out;
	}

	/* Setup the input subsystem for the ALS */
	ret = als_setup(LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev,"%s: ALS Setup Fail...\n", __func__);
		goto err_out;
	}
#if 0
	/* Setup the input subsystem for the PS */
	ret = ps_setup(LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: PS Setup Fail...\n", __func__);
		goto err_out;
	}
#endif
	/* Create the workqueue for the interrup handler */
	//(Linux RTOS)>
#if 1
	LTR553->workqueue = create_singlethread_workqueue("LTR553_wq");
	if (!LTR553->workqueue) {
		dev_err(&LTR553->i2c_client->dev, "%s: Create WorkQueue Fail...\n", __func__);
		ret = -ENOMEM;
		goto err_workqueue_out;
	}

	/* Wake lock option for promity sensor */
	wake_lock_init(&(LTR553->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");
#endif
	//(Linux RTOS)<

	/* Setup and configure both the ALS and PS on the LTR553 device */
	ret = LTR553_setup(LTR553);
	if (ret < 0) {
		dev_err(&LTR553->i2c_client->dev, "%s: Setup Fail...\n", __func__);
		goto err_LTR553_setup;
	}

	/* Setup the suspend and resume functionality */
	//(Linux RTOS)>
#if 0
	LTR553->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	LTR553->early_suspend.suspend = LTR553_early_suspend;
	LTR553->early_suspend.resume = LTR553_late_resume;
	register_early_suspend(&LTR553->early_suspend);
#endif
	//(Linux RTOS)<

	/* Register the sysfs files */
	//sysfs_register_device(client);
	//sysfs_register_als_device(client, &LTR553->als_input_dev->dev);
	//sysfs_register_ps_device(client, &LTR553->ps_input_dev->dev);
	/* Register sysfs hooks */
	if (sysfs_create_group(&LTR553->als_input_dev->dev.kobj, &ltr553_attr_group))
	{
		printk("%s sysfs_create_group error!!!\n", __func__);
	}

	printk(KERN_INFO "%s: probe complete\n", __func__);
    REGISTER_AND_INIT_ONTIM_DEBUG_FOR_THIS_DEV();

	if(!proximity_probe_ok)//add by liuwei
		proximity_probe_ok = 1;//add by liuwei

	return ret;

err_LTR553_setup:
	wake_lock_destroy(&(LTR553->ps_wake_lock));
	destroy_workqueue(LTR553->workqueue);
err_workqueue_out:
	input_unregister_device(LTR553->als_input_dev);
	input_free_device(LTR553->als_input_dev);
err_out:
	if (LTR553->power_on) LTR553->power_on(0);
	kfree(LTR553);
	return ret;
}


static const struct i2c_device_id LTR553_id[] = {
	{ "ltr553", 0 },
	{}
};

static struct i2c_driver LTR553_driver = {
	.probe = LTR553_probe,
	.id_table = LTR553_id,
	.driver = {
		.owner = THIS_MODULE,
		.name = "ltr553",
	},
};


static int __init LTR553_init(void)
{
	return i2c_add_driver(&LTR553_driver);
}

static void __exit LTR553_exit(void)
{
	i2c_del_driver(&LTR553_driver);
}


module_init(LTR553_init)
module_exit(LTR553_exit)

MODULE_AUTHOR("Lite-On Technology Corp");
MODULE_DESCRIPTION("LTR-553ALSPS Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRIVER_VERSION);



/* include/linux/i2c/rpr0400_driver.h - ROHM RPR0400 Linux kernel driver
 *
 * Copyright (C) 2012 
 * Written by Andy Mi <andy-mi@rohm.com.cn>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

 /*************** Definitions ******************/
/* GENERAL */
#include <ontim/light_prox/als_ps.h>
#define RPR400_DRV_NAME	"rpr400"
#define DRIVER_VERSION		"1.0"

#define THRES_TOLERANCE	15	//I think this is a proper value. It should not be too big.
#define THRES_DIFF		20		//I am not sure whether this is a proper value but I guess so. 
#define THRES_DEFAULT_DIFF		30

//#define RPR400_INT		81	//don't use because I use client->irq instead

/* RPR400 REGSTER */
#define REG_SYSTEMCONTROL         (0x40)
#define REG_MODECONTROL           (0x41)
#define REG_ALSPSCONTROL          (0x42)
#define REG_PERSISTENCE           (0x43)
#define REG_PSDATA                (0x44)
    #define REG_PSDATA_LSB            (0x44)
    #define REG_PSDATA_MBS            (0x45)
#define REG_ALSDATA0              (0x46)
    #define REG_ALSDATA0_LSB          (0x46)
    #define REG_ALSDATA0_MBS          (0x47)
#define REG_ALSDATA1              (0x48)
    #define REG_ALSDATA1_LSB          (0x48)
    #define REG_ALSDATA1_MBS          (0x49)
#define REG_INTERRUPT             (0x4A)
#define REG_PSTH                  (0x4B)
    #define REG_PSTH_LSB              (0x4B)
    #define REG_PSTH_MBS              (0x4C)
#define REG_PSTL                  (0x4D)
    #define REG_PSTL_LSB              (0x4D)
    #define REG_PSTL_MBS              (0x4E)
#define REG_ALSDATA0TH            (0x4F)
    #define REG_ALSDATA0TH_LSB       (0x4F)
    #define REG_ALSDATA0TH_MBS       (0x50)
#define REG_ALSDATA0TL            (0x51)
    #define REG_ALSDATA0TL_LSB        (0x51)
    #define REG_ALSDATA0TL_MBS        (0x52)

/* SETTINGS */
#define CUT_UNIT         (10)		//Andy 2012.6.6: it was (1000) previously. But I don't think the customer need als data with such accuracy.
#define CALC_ERROR        (0x80000000)
#define MASK_LONG         (0xFFFFFFFF)
#define MASK_CHAR         (0xFF)
#define BOTH_STANDBY	(0)
#define ALS100MS	(0x5)
#define PS100MS		(0x3)
#define BOTH100MS	(0x6)
#define LEDCURRENT_025MA    (0)
#define LEDCURRENT_050MA    (1)
#define LEDCURRENT_100MA    (2)
#define LEDCURRENT_200MA    (3)
#define ALSGAIN_X1X1        (0x0 << 2)
#define ALSGAIN_X1X2        (0x1 << 2)
#define ALSGAIN_X2X2        (0x5 << 2)
#define ALSGAIN_X64X64      (0xA << 2)
#define ALSGAIN_X128X64     (0xE << 2)
#define ALSGAIN_X128X128    (0xF << 2)
#define NORMAL_MODE         (0 << 4)
#define LOW_NOISE_MODE      (1 << 4)
#define PS_INT_MASK		(1 << 7)
#define ALS_INT_MASK	(1 << 6)
#define PS_THH_ONLY         (0 << 4)
#define PS_THH_BOTH_HYS     (1 << 4)
#define PS_THH_BOTH_OUTSIDE (2 << 4)
#define POLA_ACTIVEL        (0 << 3)
#define POLA_INACTIVEL      (1 << 3)
#define OUTPUT_ANYTIME      (0 << 2)
#define OUTPUT_LATCH        (1 << 2)
#define MODE_NONUSE         (0)
#define MODE_PROXIMITY      (1)
#define MODE_ILLUMINANCE    (2)
#define MODE_BOTH           (3)

/* RANGE */
#define REG_PSTH_MAX     (0xFFF)
#define REG_PSTL_MAX     (0xFFF)
#define PERSISTENCE_MAX     (0x0F)
#define GEN_READ_MAX 	(19)
#define REG_ALSPSCTL_MAX    (0x3F)
#define REG_INTERRUPT_MAX   (0x2F)

/* INIT PARAM */
#define PS_ALS_SET_MODE_CONTROL   (NORMAL_MODE)
#define PS_ALS_SET_ALSPS_CONTROL  (LEDCURRENT_100MA | ALSGAIN_X128X128)	//Set high gain value to acquire high accuracy
#define PS_ALS_SET_INTR_PERSIST   (3)
#define PS_ALS_SET_INTR           (PS_THH_BOTH_OUTSIDE| POLA_ACTIVEL | OUTPUT_LATCH | MODE_PROXIMITY)
#define PS_ALS_SET_PS_TH          (120)	//Customer should change the threshold value according to their mechanical design and measured data
#define PS_ALS_SET_PS_TL          (85)	//Changed from (0x000)
#define PS_ALS_SET_ALS_TH         (2000) 	//Compare with ALS_DATA0. ALS_Data equals 0.192*ALS_DATA0 roughly. Usually not used.
#define PS_ALS_SET_ALS_TL         (0x0000)	//Usually not used.
#define PS_ALS_SET_MIN_DELAY_TIME (100)	//Andy Mi: Changed from 125 to 100. I have no idea why it is 125 previously. 

/* OTHER */
#ifdef _ALS_BIG_ENDIAN_
#define CONVERT_TO_BE(value) ((((value) >> 8) & 0xFF) | (((value) << 8) & 0xFF00))
#else
#define CONVERT_TO_BE(value) (value)
#endif


#define _FUNCTION_USED_	(0)

/*************** Structs ******************/
struct  wake_lock ps_lock;
struct ALS_PS_DATA {
	struct i2c_client *client;
	struct als_ps_platform_data *pdata;
	struct mutex update_lock;
	struct delayed_work	dwork;	/* for PS interrupt */
	struct delayed_work    als_dwork; /* for ALS polling */
	struct input_dev *input_dev_als;
	struct input_dev *input_dev_ps;

	unsigned int enable;	//used to indicate working mode
	unsigned int als_time;	//als measurement time
	unsigned int ps_time;	//ps measurement time
	unsigned int ps_th_l;	//ps threshold low
	unsigned int ps_th_h;	//ps threshold high
	unsigned int ps_th_l_back; //ps threshold low backup
	unsigned int ps_th_h_back; //ps threshold high backup
	unsigned int als_th_l;	//als threshold low, not used in the program
	unsigned int als_th_h;	//als threshold high, not used in the program
	unsigned int persistence;	//persistence
	unsigned int control;	//als_ps_control

	/* register value */
	unsigned short als_data0_raw;	//register value of data0
	unsigned short als_data1_raw;	//register value of data1
	unsigned short ps_data_raw;	//register value of ps

	/* control flag from HAL */
	unsigned int enable_ps_sensor;
	unsigned int enable_als_sensor;

	/* PS parameters */

	unsigned int ps_direction;		/* 0 = near-to-far; 1 = far-to-near */
	unsigned int ps_data;			/* to store PS data */
	float ps_distance;
	unsigned int ledcurrent;	//led current

	/* ALS parameters */
	unsigned int als_data;			/* to store ALS data */
	unsigned int als_level;
	unsigned int gain0;	//als data0 gain
	unsigned int gain1;	//als data1 gain
	unsigned int als_poll_delay;	// the unit is ms I think. needed for als polling
	struct mutex als_ps_lock;
	int (*power_on)(int on);

};

typedef struct {
    unsigned long long data;
    unsigned long long data0;
    unsigned long long data1;
    unsigned char      gain_data0;
    unsigned char      gain_data1;
    unsigned long      dev_unit;
    unsigned char      als_time;
    unsigned short     als_data0;
    unsigned short     als_data1;
} CALC_DATA;

typedef struct {
    unsigned long positive;
    unsigned long decimal;
} CALC_ANS;


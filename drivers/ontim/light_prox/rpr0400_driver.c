/* drivers/i2c/chips/rpr0400_driver.c - ROHM RPR0400 Linux kernel driver
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
/*
 *  This is Linux kernel modules for ambient light + proximity sensor
 *  Revision History
 *  2012-7-19:	Ver. 1.0	New release together with a porting guide.
 *  2012-8-14:	Ver. 1.1	Added calibration and set thresholds methods. Besides, the thresholds are automatically changed if a ps int is triggered to avoid constant interrupts.
 */
#include <linux/debugfs.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/wakelock.h>

#include <soc/sprd/gpio.h>
#include <soc/sprd/irqs.h>
#include <linux/gpio.h>
#include <ontim/light_prox/rpr0400_driver.h>

#include <ontim/ontim_dev_dgb.h>
static char rpr400_prox_version[]="rpr400_1.0";
static char rpr400_prox_vendor_name[20]="rpr400";
u16 rpr400_prox_min = 0;
u16 rpr400_prox_max = 0;
u16 rpr400_prox_test = 0;
    DEV_ATTR_DECLARE(als_prox)
    DEV_ATTR_DEFINE("version",rpr400_prox_version)
    DEV_ATTR_DEFINE("vendor",rpr400_prox_vendor_name)
    DEV_ATTR_VAL_DEFINE("prox_min",&rpr400_prox_min,ONTIM_DEV_ARTTR_TYPE_VAL_16BIT)
    DEV_ATTR_VAL_DEFINE("prox_max",&rpr400_prox_max,ONTIM_DEV_ARTTR_TYPE_VAL_16BIT)
    DEV_ATTR_VAL_DEFINE("prox_test",&rpr400_prox_test,ONTIM_DEV_ARTTR_TYPE_VAL_16BIT)
    DEV_ATTR_DECLARE_END;
    ONTIM_DEBUG_DECLARE_AND_INIT(als_prox,als_prox,8);

/*************** Global Data ******************/
static struct ALS_PS_DATA *als_ps_data;


/* parameter for als calculation */
#define COEFFICIENT               (4)
const unsigned long data0_coefficient[COEFFICIENT] = {192, 141, 127, 117};
const unsigned long data1_coefficient[COEFFICIENT] = {316, 108,  86,  74};
const unsigned long judge_coefficient[COEFFICIENT] = { 29,  65,  85, 199};

/* mode control table */
#define MODE_CTL_FACTOR (16)
static const struct MCTL_TABLE {
    short ALS;
    short PS;
} mode_table[MODE_CTL_FACTOR] = {
    {  0,   0},   /*  0 */
    {  0,  10},   /*  1 */
    {  0,  40},   /*  2 */
    {  0, 100},   /*  3 */
    {  0, 400},   /*  4 */
    {100,   0},   /*  5 */
    {100, 100},   /*  6 */
    {100, 400},   /*  7 */
    {400,   0},   /*  8 */
    {400, 100},   /*  9 */
    {400,   0},   /* 10 */
    {400, 400},   /* 11 */
    {  0,   0},   /* 12 */
    {  0,   0},   /* 13 */
    {  0,   0},   /* 14 */
    {  0,   0}    /* 15 */
};

/* gain table */
#define GAIN_FACTOR (16)
static const struct GAIN_TABLE {
    char DATA0;
    char DATA1;
} gain_table[GAIN_FACTOR] = {
    {  1,   1},   /*  0 */
    {  2,   1},   /*  1 */
    {  0,   0},   /*  2 */
    {  0,   0},   /*  3 */
    {  0,   0},   /*  4 */
    {  2,   2},   /*  5 */
    {  0,   0},   /*  6 */
    {  0,   0},   /*  7 */
    {  0,   0},   /*  8 */
    {  0,   0},   /*  9 */
    { 64,  64},   /* 10 */
    {  0,   0},   /* 11 */
    {  0,   0},   /* 12 */
    {  0,   0},   /* 13 */
    {128,  64},   /* 14 */
    {128, 128}    /* 15 */
};

/*************** Functions ******************/
/******************************************************************************
 * NAME       : rpr400_set_enable
 * FUNCTION   : set measurement time according to enable
 * REMARKS    : this function will overwrite the work mode. if it is called improperly, 
 *			   you may shutdown some part unexpectedly. please check als_ps->enable first.
 *			   I assume it is run in normal mode. If you want low noise mode, the code should be modified.
 *****************************************************************************/
static int rpr400_set_enable(struct i2c_client *client, int enable)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;

	if(enable > 0x0B)
	{
		printk(KERN_ERR "%s: invalid measurement time setting.\n", __func__);
		return -EINVAL;
	}
	else
	{
		//mutex_lock(&als_ps->update_lock);
		ret = i2c_smbus_write_byte_data(client, REG_MODECONTROL, enable);
		//mutex_unlock(&als_ps->update_lock);

		als_ps->enable = enable;
		als_ps->als_time = mode_table[(enable & 0xF)].ALS;
		als_ps->ps_time = mode_table[(enable & 0xF)].PS;

		return ret;
	}
}

static int rpr400_set_ps_threshold_low(struct i2c_client *client, int threshold)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	unsigned short write_data;
	printk(KERN_INFO "%s: %d\n", __func__,threshold);

    /* check whether the parameter is valid */
	if(threshold > REG_PSTL_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	if(threshold > als_ps->ps_th_h)
	{
		printk(KERN_ERR "%s: higher than threshold high.\n", __func__);
		return -EINVAL;
	}
	
    /* write register to RPR400 via i2c */
	write_data = CONVERT_TO_BE(threshold);
	//mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_i2c_block_data(client, REG_PSTL, sizeof(write_data), (unsigned char *)&write_data);
	//mutex_unlock(&als_ps->update_lock);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c fail.\n", __func__);
		return ret;
	}
	//als_ps->ps_th_l = threshold;	//Update the value after successful i2c write to avoid difference. 
		
	return 0;
}

static int rpr400_set_ps_threshold_high(struct i2c_client *client, int threshold)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	unsigned short write_data;
	printk(KERN_INFO "%s: %d\n", __func__,threshold);

    /* check whether the parameter is valid */
	if(threshold > REG_PSTH_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	if(threshold < als_ps->ps_th_l)
	{
		printk(KERN_ERR "%s: lower than threshold low.\n", __func__);
		return -EINVAL;
	}
	
    /* write register to RPR400 via i2c */
	write_data = CONVERT_TO_BE(threshold);
	//mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_i2c_block_data(client, REG_PSTH, sizeof(write_data), (unsigned char *)&write_data);
	//mutex_unlock(&als_ps->update_lock);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c fail.\n", __func__);
		return ret;
	}
	//als_ps->ps_th_h = threshold;	//Update the value after successful i2c write to avoid difference. 
		
	return 0;
}

static int rpr400_calibrate(struct i2c_client *client)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int average;
 	unsigned int i, tmp, ps_th_h, ps_th_l;	

	average = 0;

	rpr400_set_enable(client, 0x01);	//PS 10ms

	for(i = 0; i < 20; i ++)
	{
		tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
		if(tmp < 0)
		{
			printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
			goto err_exit;
		}
		average += tmp & 0xFFF;	// 12 bit data
	}
	average /= 20;

//	ps_th_h = average + PS_ALS_SET_PS_TH;
//	ps_th_l = average + PS_ALS_SET_PS_TL;
	ps_th_h = average + THRES_TOLERANCE + THRES_DEFAULT_DIFF;
	ps_th_l = average + THRES_TOLERANCE;

	if(ps_th_h < 0)
	{
		printk(KERN_ERR "%s: high threshold is less than 0.\n", __func__);
		goto err_exit;
	}
	if(ps_th_h > REG_PSTH_MAX)
	{
		printk(KERN_ERR "%s: high threshold is greater than maximum allowed value.\n", __func__);
		goto err_exit;
	}
	if(ps_th_l < 0)
	{
		printk(KERN_ERR "%s: low threshold is less than 0.\n", __func__);
		goto err_exit;
	}
	if(ps_th_l > REG_PSTL_MAX)
	{
		printk(KERN_ERR "%s: low threshold is greater than maximum allowed value.\n", __func__);
		goto err_exit;
	}

	if(!(rpr400_set_ps_threshold_high(client, ps_th_h)))
	{
		als_ps->ps_th_h = ps_th_h;
		als_ps->ps_th_h_back = ps_th_h;
		rpr400_prox_max = als_ps->ps_th_h;
	}
	else 
		goto err_exit;
	if(!(rpr400_set_ps_threshold_low(client, ps_th_l)))
	{
		als_ps->ps_th_l = ps_th_l;
		als_ps->ps_th_l_back = ps_th_l;
		rpr400_prox_min = als_ps->ps_th_l;
	}
	else
		goto err_exit;

	rpr400_set_enable(client, 0);	//disable ps
	return 0;		

err_exit:
	rpr400_set_enable(client, 0);	//disable ps
	return -1;
	
}

#if _FUNCTION_USED_	//masked because they are claimed but not used, which may cause error when compilling if the warning level is high enough. These functions provides some methods.
static int rpr400_set_persist(struct i2c_client *client, int persist)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	
    /* check whether the parameter is valid */
	if(persist > PERSISTENCE_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	
    /* write register to RPR400 via i2c */
	//mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_byte_data(client, REG_PERSISTENCE, persist);
	//mutex_unlock(&als_ps->update_lock);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c fail.\n", __func__);
		return ret;
	}
	als_ps->persistence = persist;	//Update the value after successful i2c write to avoid difference. 

	return 0;
}

static int rpr400_set_control(struct i2c_client *client, int control)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	unsigned char gain, led_current;
	
	if(control > REG_ALSPSCTL_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	
	gain = (control & 0x3C) >> 2;	//gain setting values
	led_current = control & 0x03;		//led current setting value

	if(!((gain == ALSGAIN_X1X1) || (gain == ALSGAIN_X1X2) || (gain == ALSGAIN_X2X2) || (gain == ALSGAIN_X64X64)
		|| (gain == ALSGAIN_X128X64) || (gain == ALSGAIN_X128X128)))
	{
		printk(KERN_ERR "%s: invalid gain setting. \n", __func__);
		return -EINVAL;
	}
	
	//mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_byte_data(client, REG_ALSPSCONTROL, control);
	//mutex_unlock(&als_ps->update_lock);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c fail.\n", __func__);
		return ret;
	}
	als_ps->control = control;
	als_ps->gain0 = gain_table[gain].DATA0;
	als_ps->gain1 = gain_table[gain].DATA1;
	als_ps->ledcurrent = led_current;

	return ret;
}
#endif

/******************************************************************************
 * NAME       : long_long_divider
 * FUNCTION   : calc divider of unsigned long long int or unsgined long
 * REMARKS    :
 *****************************************************************************/
static void long_long_divider(unsigned long long data, unsigned long base_divier, unsigned long *answer, unsigned long long *overplus)
{
    volatile unsigned long long divier;
    volatile unsigned long      unit_sft;

    if ((long long)data < 0)	// . If data MSB is 1, it may go to endless loop. 
    	{
    	data /= 2;	//0xFFFFFFFFFFFFFFFF / 2 = 0x7FFFFFFFFFFFFFFF
	base_divier /= 2;
    	}
    divier = base_divier;
    if (data > MASK_LONG) {
        unit_sft = 0;
        while (data > divier) {
            unit_sft++;
            divier = divier << 1;
        }
        while (data > base_divier) {
            if (data > divier) {
                *answer += 1 << unit_sft;
                data    -= divier;
            }
            unit_sft--;
            divier = divier >> 1;
        }
        *overplus = data;
    } else {
        *answer = (unsigned long)(data & MASK_LONG) / base_divier;
        /* calculate over plus and shift 16bit */
        *overplus = (unsigned long long)(data - (*answer * base_divier));
    }
}

/******************************************************************************
 * NAME       : calc_rohm_als_data
 * FUNCTION   : calculate illuminance data for RPR400
 * REMARKS    : final_data is 1000 times, which is defined as CUT_UNIT, of the actual lux value
 *****************************************************************************/
static int calc_rohm_als_data(unsigned short data0, unsigned short data1, unsigned char gain0, unsigned char gain1, unsigned char time)
{
#define DECIMAL_BIT      (15)
#define JUDGE_FIXED_COEF (100)
#define MAX_OUTRANGE     (11357)
#define MAXRANGE_NMODE   (0xFFFF)
#define MAXSET_CASE      (4)

	int                final_data;
	CALC_DATA          calc_data;
	CALC_ANS           calc_ans;
	unsigned long      calc_judge;
	unsigned char      set_case;
	unsigned long      div_answer;
	unsigned long long div_overplus;
	unsigned long long overplus;
	unsigned long      max_range;

	/* set the value of measured als data */
	calc_data.als_data0  = data0;
	calc_data.als_data1  = data1;
	calc_data.gain_data0 = gain0;

	/* set max range */
	if (calc_data.gain_data0 == 0) 
	{
		/* issue error value when gain is 0 */
		return (CALC_ERROR);
	}
	else
	{
		max_range = MAX_OUTRANGE / calc_data.gain_data0;
	}
	
	/* calculate data */
	if (calc_data.als_data0 == MAXRANGE_NMODE) 
	{
		calc_ans.positive = max_range;
		calc_ans.decimal  = 0;
	} 
	else 
	{
		/* get the value which is measured from power table */
		calc_data.als_time = time;
		if (calc_data.als_time == 0) 
		{
			/* issue error value when time is 0 */
			return (CALC_ERROR);
		}

		calc_judge = calc_data.als_data1 * JUDGE_FIXED_COEF;
		if (calc_judge < (calc_data.als_data0 * judge_coefficient[0])) 
		{
			set_case = 0;
		} 
		else if (calc_judge < (calc_data.als_data0 * judge_coefficient[1]))
		{
			set_case = 1;
		} 
		else if (calc_judge < (calc_data.als_data0 * judge_coefficient[2])) 
		{
			set_case = 2;
		}
		else if (calc_judge < (calc_data.als_data0 * judge_coefficient[3])) 
		{
			 set_case = 3;
		} 
		else
		{
			set_case = MAXSET_CASE;
		}
		calc_ans.positive = 0;
		if (set_case >= MAXSET_CASE) 
		{
			calc_ans.decimal = 0;	//which means that lux output is 0
		}
		else
		{
			calc_data.gain_data1 = gain1;
			if (calc_data.gain_data1 == 0) 
			{
				/* issue error value when gain is 0 */
				return (CALC_ERROR);
			}
			calc_data.data0      = (unsigned long long )(data0_coefficient[set_case] * calc_data.als_data0) * calc_data.gain_data1;
			calc_data.data1      = (unsigned long long )(data1_coefficient[set_case] * calc_data.als_data1) * calc_data.gain_data0;
			if(calc_data.data0 < calc_data.data1)	//In this case, data will be less than 0. As data is unsigned long long, it will become extremely big.
			{
				return (CALC_ERROR);
			}
			else
			{
				calc_data.data       = (calc_data.data0 - calc_data.data1);
			}
			calc_data.dev_unit   = calc_data.gain_data0 * calc_data.gain_data1 * calc_data.als_time * 10;	//24 bit at max (128 * 128 * 100 * 10)
			if (calc_data.dev_unit == 0) 
			{
				/* issue error value when dev_unit is 0 */
				return (CALC_ERROR);
			}

			/* calculate a positive number */
			div_answer   = 0;
			div_overplus = 0;
			long_long_divider(calc_data.data, calc_data.dev_unit, &div_answer, &div_overplus);
			calc_ans.positive = div_answer;
			/* calculate a decimal number */
			calc_ans.decimal = 0;
			overplus         = div_overplus;
			if (calc_ans.positive < max_range)
			{
				if (overplus != 0)
				{
					overplus     = overplus << DECIMAL_BIT;
					div_answer   = 0;
					div_overplus = 0;
					long_long_divider(overplus, calc_data.dev_unit, &div_answer, &div_overplus);
					calc_ans.decimal = div_answer;
				}
			}

			else
			{
				calc_ans.positive = max_range;
			}
		}
	}
	
	final_data = calc_ans.positive * CUT_UNIT + ((calc_ans.decimal * CUT_UNIT) >> DECIMAL_BIT);
					
	return (final_data);

#undef DECIMAL_BIT
#undef JUDGE_FIXED_COEF
#undef MAX_OUTRANGE
#undef MAXRANGE_NMODE
#undef MAXSET_CASE
}

/******************************************************************************
 * NAME       : calculate_ps_data
 * FUNCTION   : calculate proximity data for RPR400
 * REMARKS    : 12 bit output
 *****************************************************************************/
static int calc_rohm_ps_data(unsigned short ps_reg_data)
{
    return (ps_reg_data & 0xFFF);
}

static unsigned int rpr400_als_data_to_level(unsigned int als_data)
{
#if 0
#define ALS_LEVEL_NUM 15
	int als_level[ALS_LEVEL_NUM]  = { 0, 50, 100, 150,  200,  250,  300, 350, 400,  450,  550, 650, 750, 900, 1100};
	int als_value[ALS_LEVEL_NUM]  = { 0, 50, 100, 150,  200,  250,  300, 350, 400,  450,  550, 650, 750, 900, 1100};
    	unsigned char idx;

	for(idx = 0; idx < ALS_LEVEL_NUM; idx ++)
	{
		if(als_data < als_value[idx])
		{
			break;
		}
	}
	if(idx >= ALS_LEVEL_NUM)
	{
		printk(KERN_ERR "RPR400 als data to level: exceed range.\n");
		idx = ALS_LEVEL_NUM - 1;
	}
	
	return als_level[idx];
#undef ALS_LEVEL_NUM
#else
	return als_data;
#endif
}

static void rpr400_reschedule_work(struct ALS_PS_DATA *als_ps,
					  unsigned long delay)
{
//	unsigned long flags;
	mutex_lock(&als_ps->als_ps_lock);
	//mutex_lock(&als_ps->als_ps_lock);

	//spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags);

	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	//__cancel_delayed_work(&als_ps->dwork);
	schedule_delayed_work(&als_ps->dwork, delay);

	//spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);
	mutex_unlock(&als_ps->als_ps_lock);
}

/* ALS polling routine */
static void rpr400_als_polling_work_handler(struct work_struct *work)
{
	struct ALS_PS_DATA *als_ps = container_of(work, struct ALS_PS_DATA, als_dwork.work);
	struct i2c_client *client=als_ps->client;
	int tmp = 0;
	
	
	mutex_lock(&als_ps->als_ps_lock);
	if (als_ps->enable_als_sensor == 0 )
	{
		printk(KERN_ERR "%s: enable_als_sensor is 0!!! \n", __func__);
		goto exit;
	}
	schedule_delayed_work(&als_ps->als_dwork, msecs_to_jiffies(als_ps->als_poll_delay));	// restart timer
	
	tmp = i2c_smbus_read_word_data(client, REG_ALSDATA0);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read data0 fail. \n", __func__);
		goto exit;
	}
	als_ps->als_data0_raw = (unsigned short)tmp;
	tmp = i2c_smbus_read_word_data(client, REG_ALSDATA1);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read data1 fail. \n", __func__);
		goto exit;
	}
	als_ps->als_data1_raw = (unsigned short)tmp;

// Theorically it is not necesssary to do so, but I just want to avoid any potential error.  -- Andy 2012.6.6
	tmp = i2c_smbus_read_byte_data(client, REG_ALSPSCONTROL);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read gain fail. \n", __func__);
		goto exit;
	}
	tmp = (tmp & 0x3C) >> 2;
	als_ps->gain0 = gain_table[tmp].DATA0;
	als_ps->gain1 = gain_table[tmp].DATA1;	
	
	tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read time fail. \n", __func__);
		goto exit;
	}
	tmp = tmp & 0xF;
	als_ps->als_time = mode_table[tmp].ALS;	
	
	als_ps->als_data = calc_rohm_als_data(als_ps->als_data0_raw, als_ps->als_data1_raw, als_ps->gain0, als_ps->gain1, als_ps->als_time);	
	if(als_ps->als_data == 0)
		als_ps->als_data ++;
	
	tmp=rpr400_als_data_to_level(als_ps->als_data);
	
	ontim_dev_dbg(1, "RPR400 als report: data0 = %d, data1 = %d, gain0 = %d, gain1 = %d, time = %d, lux = %d, level = %d.\n", als_ps->als_data0_raw, 
		als_ps->als_data1_raw, als_ps->gain0, als_ps->gain1, als_ps->als_time, als_ps->als_data, als_ps->als_level);

	
	if (als_ps->enable_als_sensor <= 2 )
	{
		als_ps->als_level=tmp + als_ps->enable_als_sensor;
		als_ps->enable_als_sensor++;
	}
	else
	{
		if ((tmp <= (als_ps->als_level * 12/10))&&(tmp >= (als_ps->als_level * 8/10)) ) goto exit;
		als_ps->als_level = tmp;
	}

	printk(KERN_INFO "RPR400 als report: level = %d.\n", als_ps->als_level);
	if(als_ps->als_data != CALC_ERROR)
	{
		input_report_abs(als_ps->input_dev_als, ABS_MISC, als_ps->als_level); // report als data. maybe necessary to convert to lux level
		input_sync(als_ps->input_dev_als);	
	}
exit:
	mutex_unlock(&als_ps->als_ps_lock);
}

//#define _AUTO_THRESHOLD_CHANGE_

/* PS interrupt routine */
static void rpr400_ps_int_work_handler(struct work_struct *work)
{
	struct ALS_PS_DATA *als_ps = container_of((struct delayed_work *)work, struct ALS_PS_DATA, dwork);
	struct i2c_client *client=als_ps->client;
	int tmp;
	
	mutex_lock(&als_ps->als_ps_lock);

	tmp =  i2c_smbus_read_byte_data(client, REG_INTERRUPT);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read interrupt status fail. \n", __func__);
		goto exit;
	}
	if (als_ps->enable_ps_sensor == 0 )
	{
		printk(KERN_ERR "%s: enable_ps_sensor is 0!!! \n", __func__);
		goto exit;
	}
	if(tmp & PS_INT_MASK)//Interrupt is caused by PS
	{
		tmp = i2c_smbus_read_byte_data(client, REG_ALSPSCONTROL);
		if(tmp < 0)
		{
			printk(KERN_ERR "%s: i2c read led current fail. \n", __func__);
			goto exit;
		}
		als_ps->ledcurrent = tmp & 0x3;
		
		tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
		if(tmp < 0)
		{
			printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
			goto exit;
		}
		als_ps->ps_data_raw = (unsigned short)tmp;

		als_ps->ps_data = calc_rohm_ps_data(als_ps->ps_data_raw);


		if (als_ps->enable_ps_sensor <= 2 )
		{
			als_ps->ps_direction = 7+als_ps->enable_ps_sensor;
			als_ps->enable_ps_sensor++;
		}
		else
		{
			if ( rpr400_prox_test )
			{
				if ( rpr400_prox_test == 1)
				{
					if(als_ps->ps_data > als_ps->ps_th_h)
					{		
						als_ps->ps_direction = 0;
					}
					else if(als_ps->ps_data < als_ps->ps_th_l)
					{
						als_ps->ps_direction = 10;
					}
				}
				else
				{
					als_ps->ps_direction = als_ps->ps_data;
				}
			}
			else
			{
				if (als_ps->enable_ps_sensor == 3 )
				{
					//mutex_lock(&als_ps->update_lock);
					i2c_smbus_write_byte_data(client, REG_PERSISTENCE, 0x01);
					//mutex_unlock(&als_ps->update_lock);
					als_ps->enable_ps_sensor++;
				}
				if(als_ps->ps_data > als_ps->ps_th_h)
				{		
					als_ps->ps_direction = 0;
					rpr400_set_ps_threshold_high(client, REG_PSTH_MAX);
					rpr400_set_ps_threshold_low(client, als_ps->ps_th_l);
				}
				else if(als_ps->ps_data < als_ps->ps_th_l)
				{
					als_ps->ps_direction = 10;
					rpr400_set_ps_threshold_high(client, als_ps->ps_th_h);
					rpr400_set_ps_threshold_low(client, 0);
				}
				if (als_ps->ps_direction)
				{
					als_ps->ps_direction=10;
				}
			}
		}
		printk(KERN_INFO "RPR400 ps report: raw_data = %d, data = %d, direction = %d. \n", 
				als_ps->ps_data_raw, als_ps->ps_data, als_ps->ps_direction);

		input_report_abs(als_ps->input_dev_ps, ABS_DISTANCE, als_ps->ps_direction); 
		input_sync(als_ps->input_dev_ps);	
	}
	else
	{
		printk(KERN_ERR "%s: unknown interrupt source.\n", __func__);
	}
exit:	
	mutex_unlock(&als_ps->als_ps_lock);
	enable_irq(client->irq);
}

/* assume this is ISR */
static irqreturn_t rpr400_interrupt(int vec, void *info)
{
	struct i2c_client *client=(struct i2c_client *)info;
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	disable_irq_nosync(client->irq);
	printk("%s\n", __func__);
	rpr400_reschedule_work(als_ps, 0);
	printk("%s:Finish\n", __func__);

	return IRQ_HANDLED;
}

/*************** SysFS Support ******************/
static ssize_t rpr400_show_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
//	struct i2c_client *client = als_ps_data->client;
	struct ALS_PS_DATA *als_ps = als_ps_data;

	
	return sprintf(buf, "%d\n", als_ps->enable_ps_sensor ? 1:0);
}

static int rpr400_init_client(struct i2c_client *client);
static ssize_t rpr400_store_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = als_ps_data->client;
	struct ALS_PS_DATA *als_ps = als_ps_data;
	unsigned long val = simple_strtoul(buf, NULL, 10);
// 	unsigned long flags;

	printk(KERN_INFO "RPR400 enable PS sensor -> %ld \n", val);
		
	if ((val != 0) && (val != 1)) 
	{
		printk("%s:store unvalid value=%ld\n", __func__, val);
		return count;
	}
	
	mutex_lock(&als_ps->als_ps_lock);
	if(val == 1) 
	{
		//turn on p sensor
		//wake_lock(&ps_lock);
		if (als_ps->enable_ps_sensor == 0) 
		{
			if (als_ps->power_on) als_ps->power_on(1);
			als_ps->enable_ps_sensor = 1;
		
			if (als_ps->enable_als_sensor == 0) 
			{
				rpr400_init_client(client);
				rpr400_set_enable(client, PS100MS);	//PS on and ALS off
			}
			else
			{
				rpr400_set_enable(client, BOTH100MS);	//PS on and ALS on
			}
			//mutex_lock(&als_ps->update_lock);
			i2c_smbus_write_byte_data(client, REG_PERSISTENCE, 0);
			//mutex_unlock(&als_ps->update_lock);
			if (rpr400_prox_test)
			{
				als_ps->ps_th_h = rpr400_prox_max;
				als_ps->ps_th_h_back = rpr400_prox_max;
				als_ps->ps_th_l = rpr400_prox_min;
				als_ps->ps_th_l_back = rpr400_prox_min;
			}
			rpr400_set_ps_threshold_high(client, als_ps->ps_th_h);
			rpr400_set_ps_threshold_low(client, als_ps->ps_th_l);
		}
	} 
	else 
	{
		if(als_ps->enable_ps_sensor )
		{
			als_ps->enable_ps_sensor = 0;
			if (als_ps->enable_als_sensor == 0) 
			{
				rpr400_set_enable(client, BOTH_STANDBY);	//PS off and ALS off
			}
			else 
			{
				rpr400_set_enable(client, ALS100MS);	//PS off and ALS on
			}
			//wake_unlock(&ps_lock);
			if (als_ps->power_on) als_ps->power_on(0);
		}
	}
	mutex_unlock(&als_ps->als_ps_lock);
		
	return count;
}

static ssize_t rpr400_show_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
//	struct i2c_client *client = als_ps_data->client;
	struct ALS_PS_DATA *als_ps = als_ps_data;
	
	return sprintf(buf, "%d\n", als_ps->enable_als_sensor ? 1:0);
}

static ssize_t rpr400_store_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
//	struct i2c_client *client = to_i2c_client(dev);
//	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	struct i2c_client *client = als_ps_data->client;
	struct ALS_PS_DATA *als_ps = als_ps_data;
	unsigned long val = simple_strtoul(buf, NULL, 10);
// 	unsigned long flags;
	
	printk(KERN_INFO "RPR400 enable ALS sensor -> %ld\n", val);

	if ((val != 0) && (val != 1))
	{
		printk("%s: enable als sensor=%ld\n", __func__, val);
		return count;
	}
	
	mutex_lock(&als_ps->als_ps_lock);
	if(val == 1)
	{
		//turn on light  sensor
		if (als_ps->enable_als_sensor==0)
		{
			if (als_ps->power_on) als_ps->power_on(1);
			als_ps->enable_als_sensor = 1;
			
			if(als_ps->enable_ps_sensor )
			{
				rpr400_set_enable(client, BOTH100MS);
			}
			else
			{
				rpr400_init_client(client);
				rpr400_set_enable(client, ALS100MS);
			}
		}
		
		//spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags); 
		//__cancel_delayed_work(&als_ps->als_dwork);
		schedule_delayed_work(&als_ps->als_dwork, msecs_to_jiffies(als_ps->als_poll_delay));	// 125ms
		//spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);	
	}
	else
	{
		if(als_ps->enable_als_sensor )
		{
			als_ps->enable_als_sensor = 0;

			if(als_ps->enable_ps_sensor )
			{
				rpr400_set_enable(client, PS100MS);
			}
			else
			{
				rpr400_set_enable(client, BOTH_STANDBY);
			}
			if (als_ps->power_on) als_ps->power_on(0);
		}
		
		//spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags); 
		//__cancel_delayed_work(&als_ps->als_dwork);
		//spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);	
	}
	
	mutex_unlock(&als_ps->als_ps_lock);
	return count;
}

static ssize_t rpr400_show_als_poll_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
//	struct i2c_client *client = als_ps_data->client;
	struct ALS_PS_DATA *als_ps = als_ps_data;
	
	return sprintf(buf, "%d\n", als_ps->als_poll_delay*1000);	// return in micro-second
}

static ssize_t rpr400_store_als_poll_delay(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
//	struct i2c_client *client = als_ps_data->client;
	struct ALS_PS_DATA *als_ps = als_ps_data;
	unsigned long val = simple_strtoul(buf, NULL, 10);
//	int ret;
//	int poll_delay = 0;
// 	unsigned long flags;
	
	if (val < PS_ALS_SET_MIN_DELAY_TIME * 1000)
		val = PS_ALS_SET_MIN_DELAY_TIME * 1000;	
	mutex_lock(&als_ps->als_ps_lock);
	
	als_ps->als_poll_delay = val /1000;	// convert us => ms
	
	/* we need this polling timer routine for sunlight canellation */
	//spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags); 
		
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	//__cancel_delayed_work(&als_ps->als_dwork);
	//schedule_delayed_work(&als_ps->als_dwork, msecs_to_jiffies(als_ps->als_poll_delay));	// 125ms
			
	//spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);	
	mutex_unlock(&als_ps->als_ps_lock);
	
	return count;
}

static ssize_t rpr400_show_ps_thres_high(struct device *dev,
				struct device_attribute *attr, char *buf)
{
//	struct i2c_client *client = als_ps_data->client;
	struct ALS_PS_DATA *als_ps = als_ps_data;
	
	return sprintf(buf, "%d\n", als_ps->ps_th_h);	
}

static ssize_t rpr400_store_ps_thres_high(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = als_ps_data->client;
	struct ALS_PS_DATA *als_ps = als_ps_data;
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	mutex_lock(&als_ps->als_ps_lock);
	if(!(rpr400_set_ps_threshold_high(client, val)))
	{
		als_ps->ps_th_h = val;
		als_ps->ps_th_h_back = als_ps->ps_th_h;
		rpr400_prox_max = als_ps->ps_th_h;
	}
	mutex_unlock(&als_ps->als_ps_lock);
	
	return count;
}

static ssize_t rpr400_show_ps_thres_low(struct device *dev,
				struct device_attribute *attr, char *buf)
{
//	struct i2c_client *client = als_ps_data->client;
	struct ALS_PS_DATA *als_ps = als_ps_data;
	
	return sprintf(buf, "%d\n", als_ps->ps_th_l);	
}

static ssize_t rpr400_store_ps_thres_low(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = als_ps_data->client;
	struct ALS_PS_DATA *als_ps = als_ps_data;
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	mutex_lock(&als_ps->als_ps_lock);
	if(!(rpr400_set_ps_threshold_low(client, val)))
	{
		als_ps->ps_th_l = val;
		als_ps->ps_th_l_back = als_ps->ps_th_l;
		rpr400_prox_min = als_ps->ps_th_l;
	}
	mutex_unlock(&als_ps->als_ps_lock);
	
	return count;
}

static ssize_t rpr400_show_ps_calib(struct device *dev,
				struct device_attribute *attr, char *buf)
{
//	struct i2c_client *client = als_ps_data->client;
	struct ALS_PS_DATA *als_ps = als_ps_data;
	
	return sprintf(buf, "%d\t%d\n", als_ps->ps_th_h, als_ps->ps_th_l);	
}

static ssize_t rpr400_store_ps_calib(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
#define SET_LOW_THRES	1
#define SET_HIGH_THRES	2
#define SET_BOTH_THRES	3

	struct i2c_client *client = als_ps_data->client;
	struct ALS_PS_DATA *als_ps = als_ps_data;
	unsigned long val = simple_strtoul(buf, NULL, 10);
 	unsigned int i, tmp, ps_th_h, ps_th_l;
	int average;	//This should be signed to avoid error.
	
	switch(val)
	{
		case SET_LOW_THRES:		
		//Take 20 average for noise. use noise + THRES_TOLERANCE as low threshold.
		//If high threshold is lower than the new low threshold + THRES_DIFF, make the high one equal low + THRES_DIFF
		//Please make sure that there is NO object above the sensor, otherwise it may cause the high threshold too high to trigger which make the LCD NEVER shutdown.
		//If the noise is too large, larger than 4065, it will return -1. If so, the mechanical design MUST be redo. It is quite unlikely. 
			average = 0;
			ps_th_h = als_ps->ps_th_h_back;
			ps_th_l = als_ps->ps_th_l_back;
			for(i = 0; i < 20; i ++)
			{
				tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
				if(tmp < 0)
				{
					printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
					return -1;

				}
				average += tmp & 0xFFF;	// 12 bit data
			}
			average /= 20;		//This is the average noise
			ps_th_l = average + THRES_TOLERANCE;	
			if(ps_th_l > REG_PSTL_MAX)
			{
				printk(KERN_ERR "%d in %s: low threshold is too high. \n", __LINE__, __func__);
				return -1;
			}
			if(ps_th_l < 0)
			{
				printk(KERN_ERR "%d in %s: low threshold is too low. \n", __LINE__, __func__);
				return -1;
			}
			if(ps_th_h < ps_th_l + THRES_DIFF)
			{
				ps_th_h = ps_th_l + THRES_DIFF;	//It will not be minus or an error should have occured earlier. 
				if(ps_th_h > REG_PSTH_MAX)
				{
					printk(KERN_ERR "%d in %s: high threshold is too high. \n", __LINE__, __func__);
					return -1;
				}
				if(!rpr400_set_ps_threshold_high(client, ps_th_h))
				{
					als_ps->ps_th_h = ps_th_h;
					als_ps->ps_th_h_back = ps_th_h;
					rpr400_prox_max = als_ps->ps_th_h;
				}
				else
					return -1;
			}
			if(!rpr400_set_ps_threshold_low(client, ps_th_l))
			{
				als_ps->ps_th_l = ps_th_l;
				als_ps->ps_th_l_back = ps_th_l;
				rpr400_prox_min = als_ps->ps_th_l;
			}
			else
				return -1;
			break;
		
		case SET_HIGH_THRES:	
		//Take 20 average for signal. use signal -THRES_TOLERANCE as high threshold. 
		//If low threshold is higher than the new high one - THRES_DIFF, make the low one equal high - THRES_DIFF
		//Please make sure that there IS AN object above the sensor, otherwise it will be a disaster. The high threshold will be too low which will cause the panel ALWAYS shutdown
		//Customer can use their own standard to set the test scenario. For example, a 18% grey card @ 2cm, or another example, a 90% white card 4cm, and etc. 
		//If the signal is too weak, less than 30, it will return -1. If so, the mechanical design MUST be redo. It shall not happen very frequently. 
			average = 0;
			ps_th_h = als_ps->ps_th_h_back;
			ps_th_l = als_ps->ps_th_l_back;
			for(i = 0; i < 20; i ++)
			{
				tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
				if(tmp < 0)
				{
					printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
					return -1;
				}
				average += tmp & 0xFFF;	// 12 bit data
			}
			average /= 20;		//This is the average signal
			ps_th_h = average - THRES_TOLERANCE;
			if(ps_th_h > REG_PSTH_MAX)
			{
				printk(KERN_ERR "%d in %s: high threshold is too high. \n", __LINE__, __func__);
				return -1;
			}
			if(ps_th_h < 0)
			{
				printk(KERN_ERR "%d in %s: high threshold is too low. \n", __LINE__, __func__);
				return -1;
			}
			if(ps_th_l > ps_th_h - THRES_DIFF)
			{
				ps_th_l = ps_th_h - THRES_DIFF;	//Given that REG_PSTH_MAX = REG_PSTL+MAX, it will not be greater than REG_PSTL_MAX or an error should have occured earlier.
				if(ps_th_l < 0)
				{
					printk(KERN_ERR "%d in %s: low threshold is too low. \n", __LINE__, __func__);
					return -1;
				}
				if(!rpr400_set_ps_threshold_low(client, ps_th_l))
				{
					als_ps->ps_th_l = ps_th_l;
					als_ps->ps_th_l_back = ps_th_l;
					rpr400_prox_min = als_ps->ps_th_l;
				}
				else
					return -1;
			}
			if(!rpr400_set_ps_threshold_high(client, ps_th_h))
			{
				als_ps->ps_th_h = ps_th_h;
				als_ps->ps_th_h_back = ps_th_h;
				rpr400_prox_max = als_ps->ps_th_h;
			}
			else
				return -1;
			break;
		
		case SET_BOTH_THRES:	//Take 20 average for noise. use noise + PS_ALS_SET_PS_TL as low threshold, noise + PS_ALS_SET_PS_TH as high threshold
			rpr400_calibrate(client);
			break;

		default:
			return -EINVAL;	//NOT supported!
	}
			
	return count;

#undef SET_BOTH_THRES
#undef SET_HIGH_THRES
#undef SET_LOW_THRES
}

static ssize_t rpr400_vendor_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	 return sprintf(buf, "rpr0400\n");
}

static DEVICE_ATTR(vendor_name, S_IRUGO,
		rpr400_vendor_name_show, NULL);

static DEVICE_ATTR(als_poll_delay,  0664,
				    rpr400_show_als_poll_delay, rpr400_store_als_poll_delay);

static DEVICE_ATTR(als_onoff,  0664 ,
				  rpr400_show_enable_als_sensor, rpr400_store_enable_als_sensor);

static DEVICE_ATTR(proximity_onoff,  0664 ,
				   rpr400_show_enable_ps_sensor, rpr400_store_enable_ps_sensor);

static DEVICE_ATTR(ps_thres_high,  0664 ,
				  rpr400_show_ps_thres_high, rpr400_store_ps_thres_high);

static DEVICE_ATTR(ps_thres_low,  0664 ,
				   rpr400_show_ps_thres_low, rpr400_store_ps_thres_low);

static DEVICE_ATTR(ps_calib,  0664 ,
				   rpr400_show_ps_calib, rpr400_store_ps_calib);


static struct attribute *rpr400_attributes[] = {
	&dev_attr_proximity_onoff.attr,
	&dev_attr_als_onoff.attr,
	&dev_attr_als_poll_delay.attr,
	&dev_attr_ps_thres_high.attr,
	&dev_attr_ps_thres_low.attr,
	&dev_attr_ps_calib.attr,
	&dev_attr_vendor_name.attr,
	NULL
};

static const struct attribute_group rpr400_attr_group = {
	.attrs = rpr400_attributes,
};

/*************** Initialze Functions ******************/
static int rpr400_init_client(struct i2c_client *client)
{
    struct init_func_write_data {
        unsigned char mode_ctl;
        unsigned char psals_ctl;
        unsigned char persist;
        unsigned char reserved0;
        unsigned char reserved1;
        unsigned char reserved2;
        unsigned char reserved3;
        unsigned char reserved4;
        unsigned char reserved5;
        unsigned char intr;
        unsigned char psth_hl;
        unsigned char psth_hh;
        unsigned char psth_ll;
        unsigned char psth_lh;
        unsigned char alsth_hl;
        unsigned char alsth_hh;
        unsigned char alsth_ll;
        unsigned char alsth_lh;
    } write_data;
    int result;
    unsigned char gain;

    unsigned char mode_ctl    = PS_ALS_SET_MODE_CONTROL;
    unsigned char psals_ctl   = PS_ALS_SET_ALSPS_CONTROL;
    unsigned char persist     = PS_ALS_SET_INTR_PERSIST;
    unsigned char intr        = PS_ALS_SET_INTR;
    unsigned short psth_upper  = PS_ALS_SET_PS_TH;
    unsigned short psth_low    = PS_ALS_SET_PS_TL;
    unsigned short alsth_upper = PS_ALS_SET_ALS_TH;
    unsigned short alsth_low   = PS_ALS_SET_ALS_TL;

    struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);

    if (als_ps->pdata)
    {
        psth_upper = als_ps->pdata->prox_threshold_hi;
        psth_low = als_ps->pdata->prox_threshold_lo;
    }
    

    result =  i2c_smbus_read_byte_data(client, REG_SYSTEMCONTROL);	//soft-reset
    printk(KERN_ERR "%s: i2c REG_SYSTEMCONTROL . result = 0x%x\n", __func__,result);
    if ((result < 0) ||((result & 0x3F) !=0x09 )){
    	 printk(KERN_ERR "%s: i2c REG_SYSTEMCONTROL fail. result = 0x%x\n", __func__,result);
        return (-1);
    }
    /* execute software reset */
    result =  i2c_smbus_write_byte_data(client, REG_SYSTEMCONTROL, 0xC0);	//soft-reset
    if (result != 0) {
        return (result);
    }
    if(i2c_smbus_read_word_data(client, REG_PSDATA) < 0)
    {
    	printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
    	return -1;
    }

    /* not check parameters are psth_upper, psth_low, alsth_upper, alsth_low */
    /* check the PS orerating mode */
    if ((NORMAL_MODE != mode_ctl) && (LOW_NOISE_MODE != mode_ctl)) {
	printk(KERN_ERR "%s: invalid parameter.\n", __func__);
        return (-EINVAL);
    }
    /* check the parameter of ps and als control */
    if (psals_ctl > REG_ALSPSCTL_MAX) {
	printk(KERN_ERR "%s: invalid parameter.\n", __func__);
        return (-EINVAL);
    }
    /* check the parameter of ps interrupt persistence */
    if (persist > PERSISTENCE_MAX) {
	printk(KERN_ERR "%s: invalid parameter.\n", __func__);
        return (-EINVAL);
    }
    /* check the parameter of interrupt */
    if (intr > REG_INTERRUPT_MAX) {
	printk(KERN_ERR "%s: invalid parameter.\n", __func__);
        return (-EINVAL);
    }
    /* check the parameter of proximity sensor threshold high */
    if (psth_upper > REG_PSTH_MAX) {
	printk(KERN_ERR "%s: invalid parameter.\n", __func__);
        return (-EINVAL);
    }
    /* check the parameter of proximity sensor threshold low */
    if (psth_low > REG_PSTL_MAX) {
	printk(KERN_ERR "%s: invalid parameter.\n", __func__);
        return (-EINVAL);
    }
    write_data.mode_ctl  = mode_ctl;
    write_data.psals_ctl = psals_ctl;
    write_data.persist   = persist;
    write_data.reserved0 = 0;
    write_data.reserved1 = 0;
    write_data.reserved2 = 0;
    write_data.reserved3 = 0;
    write_data.reserved4 = 0;
    write_data.reserved5 = 0;
    write_data.intr      = intr;
    write_data.psth_hl   = CONVERT_TO_BE(psth_upper) & MASK_CHAR;
    write_data.psth_hh   = CONVERT_TO_BE(psth_upper) >> 8;
    write_data.psth_ll   = CONVERT_TO_BE(psth_low) & MASK_CHAR;
    write_data.psth_lh   = CONVERT_TO_BE(psth_low) >> 8;
    write_data.alsth_hl  = CONVERT_TO_BE(alsth_upper) & MASK_CHAR;
    write_data.alsth_hh  = CONVERT_TO_BE(alsth_upper) >> 8;
    write_data.alsth_ll  = CONVERT_TO_BE(alsth_low) & MASK_CHAR;
    write_data.alsth_lh  = CONVERT_TO_BE(alsth_low) >> 8;
    result               = i2c_smbus_write_i2c_block_data(client, REG_MODECONTROL, sizeof(write_data), (unsigned char *)&write_data);

	if(result < 0)
	{
		printk(KERN_ERR "%s: i2c write fail. \n", __func__);
		return result;
	}

	gain = (psals_ctl & 0x3C) >> 2;	//gain setting values
	
	als_ps->enable = mode_ctl;
	als_ps->als_time = mode_table[(mode_ctl & 0xF)].ALS;
	als_ps->ps_time = mode_table[(mode_ctl & 0xF)].PS;
	als_ps->persistence = persist;
	als_ps->ps_th_l = psth_low;
	als_ps->ps_th_h = psth_upper;
	als_ps->als_th_l = alsth_low;
	als_ps->als_th_h = alsth_upper;
	als_ps->control = psals_ctl;
	als_ps->gain0 = gain_table[gain].DATA0;
	als_ps->gain1 = gain_table[gain].DATA1;
	als_ps->ledcurrent = psals_ctl & 0x03;
	rpr400_prox_max = als_ps->ps_th_h;
	rpr400_prox_min = als_ps->ps_th_l;
	
#ifdef _INIT_CALIB_
	rpr400_calibrate(client);
#else
	als_ps->ps_th_h_back = als_ps->ps_th_h;
	als_ps->ps_th_l_back = als_ps->ps_th_l;
#endif

    return (result);
}

/*********** I2C init/probing/exit functions ****************/

static struct i2c_driver rpr400_driver;

static int __init rpr400_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
#define ROHM_PSALS_ALSMAX (65535)
#define ROHM_PSALS_PSMAX  (4095)

	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct ALS_PS_DATA *als_ps;
	//unsigned long flags;
	
	int err = 0;
	printk("%s started.\n",__func__);
	
	if(CHECK_THIS_DEV_DEBUG_AREADY_EXIT()==0)
	{ 
	   return -EIO;
	}

	wake_lock_init(&ps_lock,WAKE_LOCK_SUSPEND,"ps wakelock");

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	als_ps = kzalloc(sizeof(struct ALS_PS_DATA), GFP_KERNEL);
	if (!als_ps) {
		err = -ENOMEM;
		goto exit;
	}
	als_ps_data = als_ps;
	als_ps->client = client;
	als_ps->pdata = client->dev.platform_data;
	als_ps->power_on=als_ps->pdata->power_on;

	i2c_set_clientdata(client, als_ps);

//	printk("enable = %x\n", als_ps->enable);

	mutex_init(&als_ps->update_lock);
	mutex_init(&als_ps->als_ps_lock);

	INIT_DELAYED_WORK(&als_ps->dwork, rpr400_ps_int_work_handler);
	INIT_DELAYED_WORK(&als_ps->als_dwork, rpr400_als_polling_work_handler); 
	
	//printk("%s :[1]\n", __func__);
	if (als_ps->power_on) als_ps->power_on(1);
	/* Initialize the RPR400 chip */
	err = rpr400_init_client(client);
	if (err)
		goto exit_kfree;

	//printk("%s :[2]\n", __func__);
	
	if (gpio_request(als_ps->pdata->int_gpio, "rpr0400 irq") < 0){
		printk("%s:Gpio request failed!\n", __func__);
		goto exit_kfree;
	}
	gpio_direction_input(als_ps->pdata->int_gpio);
	client->irq = gpio_to_irq(als_ps->pdata->int_gpio);

	als_ps->als_poll_delay = PS_ALS_SET_MIN_DELAY_TIME;	
//	err = rpr400_set_enable(client, BOTH100MS);
	/* Arrange als work after 125ms */
/*	
	als_ps->als_poll_delay = PS_ALS_SET_MIN_DELAY_TIME;
	spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags); 
	__cancel_delayed_work(&als_ps->als_dwork);
	schedule_delayed_work(&als_ps->als_dwork, msecs_to_jiffies(als_ps->als_poll_delay));	// 125ms
	spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);
*/

	/* Register to Input Device */
	als_ps->input_dev_als = input_allocate_device();
	if (!als_ps->input_dev_als) {
		err = -ENOMEM;
		printk("%s: Failed to allocate input device als\n", __func__);
		goto exit_free_irq;
	}
#if 0
	als_ps->input_dev_ps = input_allocate_device();
	if (!als_ps->input_dev_ps) {
		err = -ENOMEM;
		printk("%s: Failed to allocate input device ps\n", __func__);
		goto exit_free_dev_als;
	}
#else
	als_ps->input_dev_ps = als_ps->input_dev_als;
#endif	
	set_bit(EV_ABS, als_ps->input_dev_als->evbit);
	set_bit(EV_ABS, als_ps->input_dev_ps->evbit);

	input_set_abs_params(als_ps->input_dev_als, ABS_MISC, 0, ROHM_PSALS_ALSMAX, 0, 0);
	input_set_abs_params(als_ps->input_dev_ps, ABS_DISTANCE, 0, ROHM_PSALS_PSMAX, 0, 0);

	als_ps->input_dev_als->name = "als_prox";
#if 0
	als_ps->input_dev_ps->name = "proximity";

	err = input_register_device(als_ps->input_dev_als);
	if (err) {
		err = -ENOMEM;
		printk("%s: Unable to register input device als: %s\n", __func__, 
		       als_ps->input_dev_als->name);
		goto exit_free_dev_ps;
	}
#endif
	err = input_register_device(als_ps->input_dev_ps);
	if (err) {
		err = -ENOMEM;
		printk("%s: Unable to register input device ps: %s\n", __func__, 
		       als_ps->input_dev_ps->name);
		goto exit_unregister_dev_als;
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&als_ps->input_dev_ps->dev.kobj, &rpr400_attr_group);
	if (err)
	{
		printk("%s sysfs_create_groupX\n", __func__);
		goto exit_unregister_dev_ps;
	}
	
	printk("%s support ver. %s enabled\n", __func__, DRIVER_VERSION);
	if (request_threaded_irq(client->irq, NULL, rpr400_interrupt, IRQ_TYPE_EDGE_FALLING | IRQF_ONESHOT /*|IRQF_NO_SUSPEND*/,
		RPR400_DRV_NAME, (void *)client)) {
		printk("%s Could not allocate RPR400_INT !\n", __func__);
	
		goto exit_kfree;
	}

	//enable_irq_wake(client->irq);//add for wake sys during call suspend
	if (als_ps->power_on) als_ps->power_on(0);
	
	printk(KERN_INFO "%s: INT No. %d", __func__, client->irq);
    REGISTER_AND_INIT_ONTIM_DEBUG_FOR_THIS_DEV();
	return 0;


exit_unregister_dev_ps:
#if 0	
	input_unregister_device(als_ps->input_dev_ps);	
#endif
exit_unregister_dev_als:
	printk("%s exit_unregister_dev_als:\n", __func__);
	input_unregister_device(als_ps->input_dev_als);
//exit_free_dev_ps:
#if 0
	input_free_device(als_ps->input_dev_ps);
#endif
//exit_free_dev_als:
	input_free_device(als_ps->input_dev_als);
exit_free_irq:
	free_irq(client->irq, client);	
exit_kfree:
	if (als_ps->power_on) als_ps->power_on(0);
	als_ps_data = NULL;
	kfree(als_ps);
exit:
	return err;

#undef ROHM_PSALS_ALSMAX
#undef ROHM_PSALS_PSMAX
}

static int rpr400_remove(struct i2c_client *client)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	input_unregister_device(als_ps->input_dev_als);
	input_unregister_device(als_ps->input_dev_ps);
	
	input_free_device(als_ps->input_dev_als);
	input_free_device(als_ps->input_dev_ps);

	free_irq(client->irq, client);

	sysfs_remove_group(&client->dev.kobj, &rpr400_attr_group);

	/* Power down the device */
	rpr400_set_enable(client, 0);

	kfree(als_ps);

	return 0;
}

static int rpr400_suspend(struct i2c_client *client, pm_message_t mesg)
{
	unsigned long flags;
	//printk("%s\n", __func__);

	//disable_irq(client->irq);
	//wake_unlock(&ps_lock);
	//return rpr400_set_enable(client, 0);
	if (als_ps_data)
	{
		spin_lock_irqsave(&als_ps_data->update_lock.wait_lock, flags); 
		if (als_ps_data->enable_als_sensor)
		{
			__cancel_delayed_work(&als_ps_data->als_dwork);
			//if (als_ps_data->power_on) als_ps_data->power_on(0);

		}
		spin_unlock_irqrestore(&als_ps_data->update_lock.wait_lock, flags);	
	}
	//printk("%s: Exit \n", __func__);
	return 0;
}

static int rpr400_resume(struct i2c_client *client)
{
	unsigned long flags;
	//printk("%s \n", __func__);
	//wake_lock(&ps_lock);
	//enable_irq(client->irq);
	//return rpr400_set_enable(client, PS100MS);
	if (als_ps_data)
	{
		spin_lock_irqsave(&als_ps_data->update_lock.wait_lock, flags); 
		if (als_ps_data->enable_als_sensor)
		{
			//if (als_ps_data->power_on) als_ps_data->power_on(1);
			//__cancel_delayed_work(&als_ps_data->als_dwork);
			schedule_delayed_work(&als_ps_data->als_dwork, msecs_to_jiffies(als_ps_data->als_poll_delay));	// 125ms
		}
		spin_unlock_irqrestore(&als_ps_data->update_lock.wait_lock, flags);	
	}
	//printk("%s: Exit \n", __func__);
	return 0;
}


MODULE_DEVICE_TABLE(i2c, rpr400_id);

static const struct i2c_device_id rpr400_id[] = {
	{ "als_prox", 0 },
	{ }
};
 
static struct i2c_driver rpr400_driver = {
	.driver = {
		.name	= RPR400_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = rpr400_suspend,
	.resume	= rpr400_resume,
	.probe	= rpr400_probe,
	.remove	= rpr400_remove,
	.id_table = rpr400_id,
};

static int __init rpr400_init(void)
{
	return i2c_add_driver(&rpr400_driver);
}

static void __exit rpr400_exit(void)
{
	i2c_del_driver(&rpr400_driver);
}

MODULE_AUTHOR("Andy Mi @ ROHM");
MODULE_DESCRIPTION("RPR400 ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(rpr400_init);
module_exit(rpr400_exit);

/* drivers/input/touchscreen/ft6x06_ts.c
 *
 * FocalTech ft6x06 TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <ontim/touchscreen/focaltech/ft6x06_ts.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#else
#ifdef CONFIG_PM_RUNTIME
#include <linux/pm_runtime.h>
#endif
#endif
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <soc/sprd/irqs.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/input/mt.h>
#include <linux/pm_qos.h>

#include <linux/regulator/consumer.h>

#include <linux/notifier.h>  //add by hzb
#include <linux/platform_data/mv_usb.h> //add by hzb
//#include <linux/edge_wakeup_mmp.h>
#include <linux/pm_wakeup.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

extern int ft_psensor_drv_init(struct i2c_client *client);
extern void  ft_psensor_drv_exit(void);
extern int  fts_ctpm_get_fw_pid( struct i2c_client *client, unsigned char *vendor_id, unsigned char *product_name, int * panel_ic_id , unsigned char *force_updata);
//extern char * fts_ctpm_get_fw_vendor_name(unsigned char vendor_id);
extern void  fts_ctpm_set_fw_infor(unsigned int fw_len,unsigned char *fw);
void DrvGestureWakeupMode( char *pBuf );
extern bool tp_probe_ok;//add by liuwei
//#define FTS_CTL_FACE_DETECT
#define FTS_CTL_IIC
#define SYSFS_DEBUG
#define FTS_APK_DEBUG
#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif
#ifdef FTS_CTL_FACE_DETECT
#include "ft_psensor_drv.h"
#endif
#ifdef SYSFS_DEBUG
#include "ft6x06_ex_fun.h"
#endif
struct ts_event {
	u16 au16_x[CFG_MAX_TOUCH_POINTS];	/*x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];	/*y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];	/*touch event:
					0 -- down; 1-- contact; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];	/*touch ID */
	u16 pressure;
	u8 touch_point;

	u8 face_detect_enable;	/**/
	u8 is_face_detect;		/*check face detect*/
	u8 face_detect_value;	/*face detect value*/
};

struct ts_pinfo {
	u16 start_x[CFG_MAX_TOUCH_POINTS];	/*x coordinate */
	u16 start_y[CFG_MAX_TOUCH_POINTS];	/*y coordinate */
	u8 finger_id[CFG_MAX_TOUCH_POINTS];	/*touch ID */
	u8 is_key[CFG_MAX_TOUCH_POINTS];
	u8 touch_event[CFG_MAX_TOUCH_POINTS];
	u16 cur_x[CFG_MAX_TOUCH_POINTS];	/*x coordinate */
	u16 cur_y[CFG_MAX_TOUCH_POINTS];	/*y coordinate */
};

static struct ts_pinfo cur_info;

struct ft6x06_ts_data {
	unsigned int irq;
	unsigned int x_max;
	unsigned int y_max;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	struct touchpanel_platform_data *pdata;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
//+add by hzb
	unsigned int run_state;
	unsigned int suspend_state;
	unsigned int ps_onoff;
	unsigned int ps_state;
	struct input_dev *ps_input_dev;
	unsigned char version;
	unsigned char vendor_id;
	unsigned char product_name[20];
	struct touchpanel_panel_parm* panel_parm;
	int points ;
	int panel_ic_id;
	struct mutex ts_lock;
	struct mutex ts_suspend_lock;
	struct notifier_block chg_notif;
	int charger_state ;
	struct delayed_work work; /* for update charger state */
	int freq_hopping_mode;//frequency hopping
	int wakeup_enable;
	struct touchpanel_wakeup_event_data  *wakeup_event; 
	int wakeup_event_num;
	struct delayed_work wakeup_work; /* for update charger state */
	//int wakeup_state;
//-add by hzb
};

struct ft6x06_ts_data *g_ft6x06_ts;

//+add by hzb
#include <ontim/ontim_dev_dgb.h>

static char ft6x06_version[]="FT6x06 ontim ver 1.0";
static char ft6x06_vendor_name[50]="Ft6x06_vendor";
static u32 ft6x06_wakeup_support=0;
static u32 g_ft6x06_wakeup_support=0;
static char ft6x06_wakeup_enable=0x0;

DEV_ATTR_DECLARE(touch_screen)
DEV_ATTR_DEFINE("version",ft6x06_version)
DEV_ATTR_DEFINE("vendor",ft6x06_vendor_name)
DEV_ATTR_VAL_DEFINE("wakeup_support",&g_ft6x06_wakeup_support,ONTIM_DEV_ARTTR_TYPE_VAL_RO)
DEV_ATTR_EXEC_DEFINE_PARAM("wakeup_mode",DrvGestureWakeupMode)
DEV_ATTR_DECLARE_END;
ONTIM_DEBUG_DECLARE_AND_INIT(touch_screen,touch_screen,8);
//-add by hzb

extern uint32_t ontim_qucik_cit_boot_mode; //  factorycit

/*
*ft6x06_i2c_Read-read data and write data by i2c
*@client: handle of i2c
*@writebuf: Data that will be written to the slave
*@writelen: How many bytes to write
*@readbuf: Where to store data read from slave
*@readlen: How many bytes to read
*
*Returns negative errno, else the number of messages executed
*
*
*/
int ft6x06_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}
/*write data by i2c*/
int ft6x06_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};
	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);

	return ret;
}
#if 0
/*release the point*/
static void ft6x06_ts_release(struct ft6x06_ts_data *data)
{
	input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_sync(data->input_dev);
}
#endif
/*Read touch point information when the interrupt  is asserted.*/
static int ft6x06_read_Touchdata(struct ft6x06_ts_data *data)
{
	struct ts_event *event = &data->event;
	u8 buf[POINT_READ_BUF] = { 0 };
	u8 reg_addr = 0x00;
	u8 reg_value = 0x00;
	int ret = -1;
	int i = 0;
	int j = 0;
	u8 pointid = FT_MAX_ID;
	struct ts_pinfo pinfo;
	memset(&pinfo,0,sizeof(pinfo));
	
	ret = ft6x06_i2c_Read(data->client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n",
			__func__);
		return ret;
	}
	memset(event, 0, sizeof(struct ts_event));

	if (data->pdata->prox_enable)
	{
		/*get face detect information*/
		reg_addr = FT_FACE_DETECT_REG;
		event->face_detect_enable = FT_FACE_DETECT_DISABLE;
		event->is_face_detect = FT_FACE_DETECT_OFF;
		event->face_detect_value = 0;
		if( ft6x06_read_reg(data->client, reg_addr, &reg_value) < 0) {
			dev_err(&data->client->dev, "%s get face detect enable information failed.\n",
				__func__);
		} else {
			event->face_detect_enable = reg_value;
			//printk(KERN_INFO "%s: face_detect_enable = %d ; buf[1] = 0x%x\n",__func__,event->face_detect_enable,buf[FT_FACE_DETECT_POS]);
			if (FT_FACE_DETECT_ENABLE == (event->face_detect_enable & FT_FACE_DETECT_ENABLE)) {
				event->is_face_detect = buf[FT_FACE_DETECT_POS] & 0xE0;
				//event->face_detect_value = buf[FT_FACE_DETECT_POS] & 0x1F;
				event->face_detect_value = (event->face_detect_enable & 0x02) >> 1;
			}
		}
	}
	//event->touch_point = buf[2] & 0x0F;

	event->touch_point = 0;
	for (i = 0; i < data->points; i++) {
	//for (i = 0; i < event->touch_point; i++) {
		pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		if (pointid >= FT_MAX_ID)
			break;
		else
			event->touch_point++;
		event->au16_x[i] =
		    (s16) (buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i];
		event->au16_y[i] =
		    (s16) (buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i];
		event->au8_touch_event[i] =
		    buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
		event->au8_finger_id[i] =
		    (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
#if 0
		if (event->au8_touch_event[i]==0)
		{
			pinfo.start_x[i]=event->au16_x[i];
			pinfo.start_y[i]=event->au16_y[i];
			if (event->au16_y[i] > data->y_max)
			{
				pinfo.is_key[i]=1;
				if (event->au8_finger_id[i]>0)
				{
					event->au8_touch_event[i]=FTS_POINT_INVALID;   //skeep this point
				}
			}
			else
			{
				pinfo.is_key[i]=0;
			}
		}
		else 
		{
			for(j=0;j<data->points;j++)
			{
				if (cur_info.finger_id[j]==event->au8_finger_id[i]) 
				{
					pinfo.is_key[i]=cur_info.is_key[j];
					pinfo.start_x[i]=cur_info.start_x[j];
					pinfo.start_y[i]=cur_info.start_y[j];
					if (cur_info.touch_event[j] == FTS_POINT_INVALID)
					{
						event->au8_touch_event[i]=FTS_POINT_INVALID;   //skeep this point
					}
					break;
				}
			}
			{
				if (pinfo.is_key[i] == 0)
				{
					if ((pinfo.start_y[i] < data->y_max) && (event->au16_y[i] > data->y_max) && (j<data->points))
					{
						event->au16_y[i] = data->y_max-1;
						event->au16_x[i] = cur_info.cur_x[j];
					}
				}
			}
		}
		pinfo.finger_id[i]=event->au8_finger_id[i];
		pinfo.touch_event[i]=event->au8_touch_event[i];
		pinfo.cur_x[i]=event->au16_x[i];
		pinfo.cur_y[i]=event->au16_y[i];
#elif 0
		if ((event->au16_x[i] == 0) && (event->au16_y[i]== 0))
		{
			event->au8_touch_event[i]=FTS_POINT_INVALID;
			printk("%s: FTS_POINT_INVALID(x=0; y=%d)\n",__func__,data->y_max);
			continue;
		}
#endif
	}
	
	event->pressure = FT_PRESS;
	memcpy(&cur_info,&pinfo,sizeof(cur_info));
	return 0;
}

/*
*report the point information
*/
static void ft6x06_report_value(struct ft6x06_ts_data *data)
{
	struct ts_event *event = &data->event;
	int i = 0;
	int r=0; //add by hzb

   //if (FT_FACE_DETECT_ON != event->is_face_detect)
	if (0 == data->suspend_state)  //add by hzb
	{
		if (data->pdata->slot_enable)
		{
			for (i = 0; i < event->touch_point; i++) {
				/* LCD view area */
				//if (event->au8_touch_event[i] == FTS_POINT_INVALID ) continue;
				
				input_mt_slot(data->input_dev, event->au8_finger_id[i]);
				if ((event->au8_touch_event[i] == FTS_POINT_DOWN) ||(event->au8_touch_event[i] == FTS_POINT_CONTACT))
				{
					input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);
					
					input_report_abs(data->input_dev, ABS_MT_POSITION_X,event->au16_x[i]);
					input_report_abs(data->input_dev, ABS_MT_POSITION_Y,event->au16_y[i]);
					//input_report_abs(data->input_dev, ABS_MT_PRESSURE,event->pressure);
					input_report_abs(data->input_dev,ABS_MT_TOUCH_MAJOR,event->pressure);
				}
				else 
				{
					input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
					r++;
				}

				ontim_dev_dbg(2,"%s:x=%d;y= %d; p=%d; f=%d; e=%d; tp=%d; r =%d\n",__func__,event->au16_x[i],event->au16_y[i],event->pressure,event->au8_finger_id[i],event->au8_touch_event[i],event->touch_point,r);
			}
			if(event->touch_point == r)
			{
				for ( i=0;i<g_ft6x06_ts->points; i++)
				{
					input_mt_slot(data->input_dev, i);
					input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
				}
				input_report_key(data->input_dev, BTN_TOUCH, 0);
			}
			else
				input_report_key(data->input_dev, BTN_TOUCH, event->touch_point > 0);
			input_sync(data->input_dev);

		}
		else
		{
			for (i = 0; i < event->touch_point; i++) {
				/* LCD view area */
				if (event->au8_touch_event[i] == FTS_POINT_INVALID ) continue;
				input_report_abs(data->input_dev, ABS_MT_POSITION_X,event->au16_x[i]);
				input_report_abs(data->input_dev, ABS_MT_POSITION_Y,event->au16_y[i]);
				input_report_abs(data->input_dev, ABS_MT_PRESSURE,event->pressure);
				input_report_abs(data->input_dev, ABS_MT_TRACKING_ID,event->au8_finger_id[i]);
				if (event->au8_touch_event[i] == FTS_POINT_UP)
				{
					input_report_abs(data->input_dev,ABS_MT_TOUCH_MAJOR, 0);
					r++;
				}
				else 
				{
					input_report_abs(data->input_dev,ABS_MT_TOUCH_MAJOR,event->pressure);
				}

				input_mt_sync(data->input_dev);
				ontim_dev_dbg(2,"%s:x=%d;y= %d; p=%d; f=%d; e=%d; tp=%d; r =%d\n",__func__,event->au16_x[i],event->au16_y[i],event->pressure,event->au8_finger_id[i],event->au8_touch_event[i],event->touch_point,r);
			}

			input_report_key(data->input_dev, BTN_TOUCH, event->touch_point);
			input_sync(data->input_dev);
			
			if ((event->touch_point-r)  == 0)
			{
				input_mt_sync(data->input_dev); 
				input_sync(data->input_dev);
			}
		}
	}
	if (data->pdata->prox_enable)
	{
		if ((FT_FACE_DETECT_ENABLE == (event->face_detect_enable  & FT_FACE_DETECT_ENABLE)) && (FT_FACE_DETECT_ON == event->is_face_detect ||
				FT_FACE_DETECT_OFF == event->is_face_detect) && (data->ps_input_dev)) {
			static u8 detect=FT_FACE_DETECT_OFF;
			if ((detect != event->is_face_detect) || (data->ps_state == 255))
			{
				detect = event->is_face_detect;
				data->ps_state = (FT_FACE_DETECT_ON == event->is_face_detect)?0:10;
				input_report_abs(data->ps_input_dev, 
							ABS_DISTANCE, data->ps_state);
				input_sync(data->ps_input_dev);
				printk(KERN_INFO "%s: is_face_detect = 0x%x; face_detect_value = 0x%x\n",__func__,event->is_face_detect,data->ps_state);
			}
		}
	}
			
}
void DrvGestureWakeupMode( char *pBuf )
{

#define GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG     0x0001    //0000 0000 0000 0001
#define GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG        0x0002    //0000 0000 0000 0010
#define GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG      0x0004    //0000 0000 0000 0100
#define GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG      0x0008    //0000 0000 0000 1000
#define GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG     0x0010    //0000 0000 0001 0000

#define FTC_GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG      0x0001
#define FTC_GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG     0x0002 
#define FTC_GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG        0x0004  
#define FTC_GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG      0x0008   
#define FTC_GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG     0x0010    


    u32 nWakeupMode ;
  
    ft6x06_wakeup_enable = 0 ;

    if ( pBuf != NULL )
    {
        sscanf(pBuf, "%x", &nWakeupMode);  

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG) == GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG)
        {
            ft6x06_wakeup_enable = ft6x06_wakeup_enable | (FTC_GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG);
        }
        else
        {
            ft6x06_wakeup_enable = ft6x06_wakeup_enable & (~FTC_GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG);
        }
        
        if ((nWakeupMode & GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG)
        {
            ft6x06_wakeup_enable = ft6x06_wakeup_enable | (FTC_GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG);
        }
        else
        {
            ft6x06_wakeup_enable = ft6x06_wakeup_enable & (~(FTC_GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG));
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG)
        {
            ft6x06_wakeup_enable = ft6x06_wakeup_enable | (FTC_GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG);
        }
        else
        {
            ft6x06_wakeup_enable = ft6x06_wakeup_enable & (~(FTC_GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG));
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG)
        {
            ft6x06_wakeup_enable = ft6x06_wakeup_enable | (FTC_GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG);
        }
        else
        {
            ft6x06_wakeup_enable = ft6x06_wakeup_enable & (~(FTC_GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG));
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG)
        {
            ft6x06_wakeup_enable = ft6x06_wakeup_enable | (FTC_GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG);
        }
        else
        {
            ft6x06_wakeup_enable = ft6x06_wakeup_enable & (~(FTC_GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG));
        }

        printk("[kernel][%s]/-/-/-/-/-/-/wakeup_mode 0x%x/-/-/-/-/-/\n",__func__,ft6x06_wakeup_enable );
    }
}


static int ft6x06_wakeup_work_func(struct work_struct *work)
{
    struct ft6x06_ts_data *ft6x06_ts = 
    container_of(work, struct ft6x06_ts_data, wakeup_work);
    u8 val=0;
    u8 handle_id=0;
    mutex_lock(&(ft6x06_ts->ts_lock));
    //ft6x06_ts->wakeup_state=0;
    ft6x06_read_reg(ft6x06_ts->client,0xD3, &val);
    printk("%s:TP_wakeup---val = 0x%x\n",__func__,val);
    if ((val >=FTS_EVENT_LIFT) && (val <=FTS_EVENT_DOUBLE_CLICK ))
    {
        handle_id=val-FTS_EVENT_LIFT;

        if (ft6x06_ts->wakeup_enable && (1<<handle_id) )
        {
            int i;
            for(i=0; i<ft6x06_ts->wakeup_event_num; i++)
            {
                if (ft6x06_ts->wakeup_event[i].wakeup_event == (1<<handle_id) )
                {
                    input_report_key(ft6x06_ts->input_dev, ft6x06_ts->wakeup_event[i].key_code, 1);
                    input_report_key(ft6x06_ts->input_dev, ft6x06_ts->wakeup_event[i].key_code, 0);
                    input_sync(ft6x06_ts->input_dev);
                }
            }
        }
    }
    ft6x06_read_reg(ft6x06_ts->client,0xD3, &val);
    printk("%s:TP_wakeup[end]---val = 0x%x\n",__func__,val);
    mutex_unlock(&ft6x06_ts->ts_lock);
    return 0;
}

static void ft6x06_event_wakeup(int gpio, void *data)
{
    	struct ft6x06_ts_data *ft6x06_ts = (struct ft6x06_ts_data *)data;

   	printk("%s\n",__func__);
    	mutex_lock(&(ft6x06_ts->ts_lock));
	//ft6x06_ts->wakeup_state=2;
       schedule_delayed_work(&ft6x06_ts->wakeup_work, msecs_to_jiffies(200));
    	mutex_unlock(&ft6x06_ts->ts_lock);
	
}

/*The ft6x06 device will signal the host about TRIGGER_FALLING.
*Processed when the interrupt is asserted.
*/
static irqreturn_t ft6x06_ts_interrupt(int irq, void *dev_id)
{
	struct ft6x06_ts_data *ft6x06_ts = dev_id;
	int ret = 0;
    	u8 val=0;
    	u8 handle_id=0;
		
	//printk(KERN_INFO "%s: \n" ,__func__);
	if (ft6x06_ts->wakeup_enable)
	{
		if (delayed_work_pending(&ft6x06_ts->wakeup_work))
			cancel_delayed_work_sync(&ft6x06_ts->wakeup_work);
	}
	mutex_lock(&(ft6x06_ts->ts_lock));
	if ( ft6x06_ts->run_state )
	{
		disable_irq_nosync(ft6x06_ts->irq);

		if (ft6x06_ts->wakeup_enable)
		{

			ontim_dev_dbg(2, "%s:wakeup_enable. \n" ,__func__);
			pm_wakeup_event(&ft6x06_ts->client->dev, 3000);
			ft6x06_read_reg(ft6x06_ts->client,0xD3, &val);
			ontim_dev_dbg(2,"%s:TP_wakeup---val = 0x%x\n",__func__,val);
			if ((val >=FTS_EVENT_LIFT) && (val <=FTS_EVENT_DOUBLE_CLICK ))
			{
				handle_id=val-FTS_EVENT_LIFT;

				if (ft6x06_ts->wakeup_enable && (1<<handle_id) )
				{
					int i;
					for(i=0; i<ft6x06_ts->wakeup_event_num; i++)
					{
						if (ft6x06_ts->wakeup_event[i].wakeup_event == (1<<handle_id) )
						{
							//ft6x06_ts->wakeup_enable=0;  
							ft6x06_write_reg(ft6x06_ts->client,0xD0, 0);
							input_report_key(ft6x06_ts->input_dev, ft6x06_ts->wakeup_event[i].key_code, 1);
							input_sync(ft6x06_ts->input_dev);
							input_report_key(ft6x06_ts->input_dev, ft6x06_ts->wakeup_event[i].key_code, 0);
							input_sync(ft6x06_ts->input_dev);
							ontim_dev_dbg(2,"[kernel]:%s:@@@@@@@@@@@@@@gesture off in interrupt.@@@@@@@@@@@@@@@@@@\n",__func__);
						}
						else
						{
							printk("[kernel]:%s:not wake up system.TP_wakeup---val = 0x%x\n",__func__,val);
						}
					}
				}
			}
			ft6x06_read_reg(ft6x06_ts->client,0xD3, &val);
			//printk("%s:TP_wakeup[end]---val = 0x%x\n",__func__,val);

        }
        else
        {
	        ret = ft6x06_read_Touchdata(ft6x06_ts);
	        if (ret == 0)
	            ft6x06_report_value(ft6x06_ts);
            enable_irq(ft6x06_ts->irq);
        }
    }
    else
    {
        ft6x06_ts->run_state = 1; //add by hzb
    }
    //printk(KERN_INFO "%s: exit\n" ,__func__);
    mutex_unlock(&ft6x06_ts->ts_lock);
    return IRQ_HANDLED;
}

void ft6x06_reset_tp(int HighOrLow)
{
	if (g_ft6x06_ts->pdata->reset_gpio)
	{
		gpio_set_value(g_ft6x06_ts->pdata->reset_gpio, HighOrLow);
		printk(KERN_INFO "%s:Set tp reset pin to %d\n",__func__, HighOrLow);
	}
}

//+add by hzb
static ssize_t ft6x06_ts_virtual_keys_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
      #define CHAR_NB 40
      int i,virtual_key_number;
      struct touchpanel_virtual_key_data *vkey;

      char *buffer;
      int     char_num;
      int     key_code;
      int     logic_x;
      int     logic_y;
      int     x_correction;
      int     y_correction;

       if ( g_ft6x06_ts == NULL ) goto error;
	   
      	virtual_key_number = g_ft6x06_ts->panel_parm->virtual_key_num;
	vkey =  g_ft6x06_ts->panel_parm->virtual_key;
	if (( virtual_key_number == 0 ) || ( vkey == NULL )) goto error;
	
	buffer = kcalloc(virtual_key_number,CHAR_NB, GFP_KERNEL);	
       if ( buffer == NULL ) goto error;
	
      memset(buffer,0,CHAR_NB * virtual_key_number);

      for( i=0 ,char_num=0; i < virtual_key_number ; i++)
      {
          if ( vkey[i].key_code)
          {
            key_code = vkey[i].key_code;
            {
                logic_x = vkey[i].logic_x;
                logic_y = vkey[i].logic_y;
                x_correction = vkey[i].x_correction;
                y_correction = vkey[i].y_correction;
            }
			
            if ( char_num )
            {
                sprintf(buffer+char_num++,":");
            }
             char_num +=sprintf( buffer+char_num , __stringify(EV_KEY)":");
             char_num +=sprintf( buffer+char_num , "%d", key_code);
             char_num +=sprintf( buffer+char_num , ":%d", logic_x);
             char_num +=sprintf( buffer+char_num , ":%d", logic_y);
             char_num +=sprintf( buffer+char_num , ":%d", x_correction);
             char_num +=sprintf( buffer+char_num , ":%d", y_correction);
          }
      }
      if ( char_num )
      {
          sprintf(buffer+char_num++,"\n");
      }
      buffer[char_num]=0;
      printk(KERN_INFO "%s : %s   ;  len = %d\n",__func__,buffer,char_num);
       memcpy(buf,buffer,char_num);
	   kfree(buffer);
	 return char_num;
error:
      printk(KERN_INFO "%s : No virtualbutton\n",__func__);
	 return 0;
	   
}


static struct kobj_attribute ft6x06_ts_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.focaltech-ts",
		.mode = S_IRUGO,
	},
	.show = &ft6x06_ts_virtual_keys_show,
};

static struct attribute *ft6x06_ts_properties_attrs[] = {
	&ft6x06_ts_virtual_keys_attr.attr,
	NULL
};
static struct attribute_group ft6x06_ts_properties_attr_group = {
	.attrs = ft6x06_ts_properties_attrs,
};

extern int fts_ctpm_auto_upgrade(struct i2c_client *client, unsigned char *force_updata);

//+add by hzb


static ssize_t ft6x06_ps_onoff_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
        if(g_ft6x06_ts->ps_onoff)
	    return sprintf(buf, "%s\n","1");
        else
	    return sprintf(buf, "%s\n","0");
}

static ssize_t ft6x06_ps_onoff_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	printk( "%s : buf=%s enter \n",__func__,buf);

	if(strncmp(buf,"1",1)==0)
	{
		mutex_lock(&g_ft6x06_ts->ts_lock);
		//printk(KERN_INFO "%s: down -- sem = %d\n" ,__func__,--ft_sem);
		if((g_ft6x06_ts->ps_onoff==0) && (g_ft6x06_ts->ps_input_dev) )
		{
			g_ft6x06_ts->ps_onoff=1;
			if (g_ft6x06_ts->suspend_state == 0)
			{
				g_ft6x06_ts->ps_state=255;
				ft6x06_write_reg(g_ft6x06_ts->client, FT_FACE_DETECT_REG, 1);
				input_report_abs(g_ft6x06_ts->ps_input_dev, 
							ABS_DISTANCE, g_ft6x06_ts->ps_state);
				input_sync(g_ft6x06_ts->ps_input_dev);
			}
		}
		//printk(KERN_INFO "%s: up -- sem = %d\n" ,__func__,++ft_sem);
		mutex_unlock(&g_ft6x06_ts->ts_lock);
	}
	else if(strncmp(buf,"0",1)==0)
		{
			int need_disable_irq=0;
			mutex_lock(&g_ft6x06_ts->ts_suspend_lock);
			mutex_lock(&g_ft6x06_ts->ts_lock);
			//printk(KERN_INFO "%s: down -- sem = %d\n" ,__func__,--ft_sem);
			g_ft6x06_ts->ps_onoff=0;
			ft6x06_write_reg(g_ft6x06_ts->client, FT_FACE_DETECT_REG, 0);
			if (g_ft6x06_ts->suspend_state == 1)
			{
				printk("%s: [FTS]ft6x06 suspend [1]\n",__func__);
				g_ft6x06_ts->suspend_state=2;
				need_disable_irq = 1;
			}
			//printk(KERN_INFO "%s: up -- sem = %d\n" ,__func__,++ft_sem);
			mutex_unlock(&g_ft6x06_ts->ts_lock);
			
			if (need_disable_irq)
			{
				printk("%s: [FTS]ft6x06 suspend [2]\n",__func__);
				disable_irq(g_ft6x06_ts->irq);
				g_ft6x06_ts->run_state = 0;
				ft6x06_write_reg(g_ft6x06_ts->client, FT6x06_REG_PMODE, PMODE_HIBERNATE);
			}
			mutex_unlock(&g_ft6x06_ts->ts_suspend_lock);
		}
	return count;
}
static ssize_t ft6x06_ps_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
        if(g_ft6x06_ts->ps_state)
	    return sprintf(buf, "%s\n","1");
        else
	    return sprintf(buf, "%s\n","0");
}
static ssize_t ft6x06_ps_state_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
        return 0;
}

static ssize_t ft6x06_ps_vendor_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	 return sprintf(buf, "FT6x06\n");
}

static DEVICE_ATTR(proximity_onoff, S_IRUGO|S_IWUSR|S_IWGRP,
		ft6x06_ps_onoff_show, ft6x06_ps_onoff_store);

static DEVICE_ATTR(ps_state, S_IRUGO|S_IWUSR|S_IWGRP,
		ft6x06_ps_state_show, ft6x06_ps_state_store);

static DEVICE_ATTR(vendor_name, S_IRUGO,
		ft6x06_ps_vendor_name_show, NULL);

static struct attribute *ft6x06_ps_attributes[] = {
	&dev_attr_proximity_onoff.attr,
	&dev_attr_ps_state.attr,
	&dev_attr_vendor_name.attr,
	NULL
};

static struct attribute_group ft6x06_ps_attribute_group = {
	.attrs = ft6x06_ps_attributes
};
void ft6x06_ts_updata_version(void)
{
	if (g_ft6x06_ts)
	{
		ft6x06_read_reg(g_ft6x06_ts->client, FT6x06_REG_FW_VER, &g_ft6x06_ts->version);
		printk("%s: version = 0x%x\n",__func__, g_ft6x06_ts->version);
		sprintf(ft6x06_version,"0x%x",(g_ft6x06_ts->version));
	}
}

static int ft6x06_updata_freq_hopping(struct work_struct *work)
{
	struct ft6x06_ts_data *ft6x06_ts  = container_of(to_delayed_work(work),
						  struct ft6x06_ts_data,
						  work);
	int freq_hopping=0;

	char cmd[2]={FT6x06_REG_CHARGER_STATE,0};
	if (ft6x06_ts->freq_hopping_mode==TP_FREQ_HOPPING_MODE_AUTO)
	{
		return 0;
	}
	else if (ft6x06_ts->freq_hopping_mode==TP_FREQ_HOPPING_MODE_ON)
	{
		freq_hopping=1;
	}
	else
	{
		freq_hopping=ft6x06_ts->charger_state;
	}
	printk(KERN_INFO "%s: freq_hopping = %d\n",__func__,freq_hopping);
	mutex_lock(&(ft6x06_ts->ts_lock));
		cmd[1]=freq_hopping;
	ft6x06_i2c_Write(ft6x06_ts->client,cmd, 2);
	mutex_unlock(&(ft6x06_ts->ts_lock));
	return 0;
}

#ifdef CONFIG_USB_MV_UDC
static int ft6x06_charger_notifier(struct notifier_block *nb,
					 unsigned long type, void *chg_event)
{
	struct ft6x06_ts_data *ft6x06_ts =
	    container_of(nb, struct ft6x06_ts_data, chg_notif);
	char cmd[2]={FT6x06_REG_CHARGER_STATE,0};
	printk(KERN_INFO "%s:type = %d\n",__func__,(int)type);
	mutex_lock(&(ft6x06_ts->ts_lock));
	switch (type) {
	case NULL_CHARGER:
		cmd[1]=0;
		ft6x06_ts->charger_state=0;
		break;
	default:
		cmd[1]=1;
		ft6x06_ts->charger_state=1;
		break;
	}
	//ft6x06_i2c_Write(ft6x06_ts->client,cmd, 2);
	mutex_unlock(&(ft6x06_ts->ts_lock));
	if (delayed_work_pending(&ft6x06_ts->work))
		cancel_delayed_work_sync(&ft6x06_ts->work);
	schedule_delayed_work(&ft6x06_ts->work, msecs_to_jiffies(100));
	return 0;
}
#endif
//-add by hzb

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft6x06_ts_suspend(struct early_suspend *handler);
static void ft6x06_ts_resume(struct early_suspend *handler);
#endif
static int ft6x06_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
    struct touchpanel_platform_data *pdata =
        (struct touchpanel_platform_data *)client->dev.platform_data;
    struct ft6x06_ts_data *ft6x06_ts;
    struct input_dev *input_dev;
    struct input_dev *ps_input_dev;
    int err = 0;
    unsigned char uc_reg_value;
    unsigned char uc_reg_addr;
    unsigned char force_updata=0;

    extern struct touchpanel_platform_data ft6x06_ts_info;
    //+add by hzb
    struct ontim_debug ontim_debug_als_prox;
    unsigned char *vendor_name=NULL;
    int i;

    printk("[kernel][start probe %s + %d]......of_node= %p\n",__func__, __LINE__,client->dev.of_node);
    if (pdata==NULL)
    {
      printk("%s: pdata is NULL,  dev name is %s\n",__func__,client->dev.init_name);
      client->dev.platform_data=&ft6x06_ts_info;
      pdata=&ft6x06_ts_info;
    }
    if(CHECK_THIS_DEV_DEBUG_AREADY_EXIT()==0)
    { 
        printk("[kernel][check %s failed]......\n",__func__, __LINE__);
        return -EIO;
    }
    //-add by hzb
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        err = -ENODEV;
        goto exit_check_functionality_failed;
    }
    //+add by hzb
    if ( ( ft6x06_read_reg(client, FT6x06_REG_FW_VER, &uc_reg_value)) < 0 ) 
        goto exit_check_functionality_failed;
    //-add by hzb

    ft6x06_ts = kzalloc(sizeof(struct ft6x06_ts_data), GFP_KERNEL);

    if (!ft6x06_ts) {
        err = -ENOMEM;
        goto exit_alloc_data_failed;
    }
    g_ft6x06_ts = ft6x06_ts;
    mutex_init(&ft6x06_ts->ts_lock);
    mutex_init(&ft6x06_ts->ts_suspend_lock);

    if (pdata->pin_config)
    {
        pdata->pin_config();
    }

    if (pdata->irq_gpio)
    {
        gpio_request(pdata->irq_gpio, "ts_irq_pin");
        client->irq = gpio_to_irq(pdata->irq_gpio);
    }

    i2c_set_clientdata(client, ft6x06_ts);
    ft6x06_ts->irq = client->irq;
    ft6x06_ts->client = client;
    ft6x06_ts->pdata = pdata;

    if (pdata->reset_gpio)
    {
        err = gpio_request(pdata->reset_gpio, "ft6x06 reset");
        if (err < 0) {
            dev_err(&client->dev, "%s:failed to set gpio reset.\n",
                    __func__);
            goto exit_request_reset;
        }
    }
    #ifdef CONFIG_HAS_EARLYSUSPEND
    ft6x06_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 1 ;
    ft6x06_ts->early_suspend.suspend = ft6x06_ts_suspend;
    ft6x06_ts->early_suspend.resume	= ft6x06_ts_resume;
    register_early_suspend(&ft6x06_ts->early_suspend);
    #endif
    printk(KERN_ERR "%s: client->irq= %d, pdata->irqflags= 0x%lx, client->dev.driver->name is %s,ft6x06_ts = 0x%x\n",__func__,client->irq, pdata->irqflags, client->dev.driver->name,(int)ft6x06_ts);

    if (pdata->vdd_name)
    {
        struct regulator *reg_vdd;
        reg_vdd = regulator_get(&client->dev, pdata->vdd_name);
        //	regulator_set_voltage(reg_vdd, 2700000, 2800000);
        regulator_enable(reg_vdd);
        msleep(10);
    }
    if(pdata->power_ic)
        pdata->power_ic(&client->dev, 1);
    if(pdata->power_i2c)
        pdata->power_i2c(&client->dev, 1);

    if((pdata->power_ic) ||(pdata->power_i2c))
        msleep(10);

    //reset
    if (pdata->reset)
    {
        pdata->reset(&client->dev);
    }
    else if (pdata->reset_gpio)
    {
        gpio_direction_output(pdata->reset_gpio, 1);
        msleep(10);
        gpio_direction_output(pdata->reset_gpio, 0);
        msleep(20);
        gpio_direction_output(pdata->reset_gpio, 1);
        msleep(10);
    }
    msleep(150);
    //+add by hzb
    ft6x06_read_reg(client, FT6x06_REG_FW_VENDOR, &ft6x06_ts->vendor_id);
    printk("%s: vendor_id = 0x%x\n",__func__, ft6x06_ts->vendor_id);

    ft6x06_read_reg(client, FT6x06_REG_FW_VER, &ft6x06_ts->version);
    printk("%s: version = 0x%x\n",__func__, ft6x06_ts->version);

    fts_ctpm_get_fw_pid(client,&ft6x06_ts->vendor_id, &ft6x06_ts->product_name[0],&ft6x06_ts->panel_ic_id,&force_updata);
    printk(KERN_INFO "%s: panel_ic_id = 0x%x; vendor_id = 0x%x; product_name is %s\n",
           __func__,ft6x06_ts->panel_ic_id,ft6x06_ts->vendor_id,ft6x06_ts->product_name);

    ft6x06_ts->panel_parm= &pdata->panel_parm[0];
    sprintf(ft6x06_version,"0x%x",(ft6x06_ts->version));

    if ((ft6x06_ts->product_name[0] ==0) || (pdata->ignore_product_name))
    {
        printk(KERN_INFO "%s: Do not check product name!!!!\n",__func__);
        for(i=0 ; i <pdata->panel_parm_num ; i++)
        {
            if ((pdata->panel_parm[i].vendor_id == ft6x06_ts->vendor_id) && 
                ((pdata->panel_parm[i].panel_ic_id) ? (pdata->panel_parm[i].panel_ic_id == ft6x06_ts->panel_ic_id) : 1))
            {
                ft6x06_ts->panel_parm= &pdata->panel_parm[i];
                if (pdata->panel_parm[i].vendor_name[0] != 0)
                {
                    vendor_name=pdata->panel_parm[i].vendor_name;
                }
                memcpy(ft6x06_ts->product_name, pdata->panel_parm[i].product_name,strlen(pdata->panel_parm[i].product_name));
                ft6x06_ts->product_name[strlen(pdata->panel_parm[i].product_name)]=0;
                printk(KERN_INFO "%s: find panel Parameter;  i = %d ; vendor_id = 0x%x; product_name is %s; vendor_name is %s (0x%x)\n",__func__,i,ft6x06_ts->vendor_id,ft6x06_ts->product_name,vendor_name,(int)vendor_name);
                break;
            }
            printk(KERN_INFO "%s: [%d] vendor_id = 0x%x; product_name is %s\n",__func__,i,pdata->panel_parm[i].vendor_id,pdata->panel_parm[i].product_name);
        }
    }
    else
    {
        for(i=0 ; i <pdata->panel_parm_num ; i++)
        {
            if (((pdata->panel_parm[i].vendor_id == ft6x06_ts->vendor_id) && 
                 ((pdata->panel_parm[i].panel_ic_id) ? (pdata->panel_parm[i].panel_ic_id == ft6x06_ts->panel_ic_id) : 1)) && 
                (memcmp(ft6x06_ts->product_name, pdata->panel_parm[i].product_name,strlen(pdata->panel_parm[i].product_name) )==0))
            {
                ft6x06_ts->panel_parm= &pdata->panel_parm[i];
                if (pdata->panel_parm[i].vendor_name[0] != 0)
                {
                    vendor_name=pdata->panel_parm[i].vendor_name;
                }
                printk(KERN_INFO "%s: find panel Parameter;  i = %d ; vendor_id = 0x%x; product_name is %s\n",__func__,i,ft6x06_ts->vendor_id,ft6x06_ts->product_name);
                break;
            }
            printk(KERN_INFO "%s: [%d] vendor_id = 0x%x; product_name is %s\n",__func__,i,pdata->panel_parm[i].vendor_id,pdata->panel_parm[i].product_name);
        }
    }
    if ((ft6x06_ts->product_name[0] ==0)&&(ft6x06_ts->vendor_id ==0) && (pdata->panel_parm_num==1))
    {
        ft6x06_ts->vendor_id = ft6x06_ts->panel_parm->vendor_id;
        memcpy(ft6x06_ts->product_name, ft6x06_ts->panel_parm->product_name,strlen(ft6x06_ts->panel_parm->product_name));
        ft6x06_ts->product_name[strlen(ft6x06_ts->panel_parm->product_name)]=0;
    }
    #if 0
    if(vendor_name==NULL)
    {
        vendor_name=fts_ctpm_get_fw_vendor_name(ft6x06_ts->vendor_id);
        printk(KERN_INFO "%s: vendor_name is %s (0x%x)\n",__func__,vendor_name,(int)vendor_name);
    }
    #endif
    if(vendor_name!=NULL)
    {
        sprintf(ft6x06_vendor_name,"%s",vendor_name);
    }
    if ((memcmp(ft6x06_ts->product_name, ft6x06_ts->panel_parm->product_name,strlen(ft6x06_ts->panel_parm->product_name) )==0) &&
        (ft6x06_ts->panel_parm->panel_fw) && (ft6x06_ts->panel_parm->panel_fw_size) && (ft6x06_ts->vendor_id == ft6x06_ts->panel_parm->vendor_id))
	{
		fts_ctpm_set_fw_infor(ft6x06_ts->panel_parm->panel_fw_size, ft6x06_ts->panel_parm->panel_fw);
	}
	ft6x06_ts->x_max = ft6x06_ts->panel_parm->x_max_res - 1;
	ft6x06_ts->y_max = ft6x06_ts->panel_parm->y_max_res - 1;
	if ((ft6x06_ts->panel_parm->points > 0) && (ft6x06_ts->panel_parm->points <= CFG_MAX_TOUCH_POINTS))
	{
		ft6x06_ts->points = ft6x06_ts->panel_parm->points;
	}
	else
	{
		if ((ft6x06_ts->panel_ic_id == ((FT6x06_UPGRADE_ID_1 << 8)|FT6x06_UPGRADE_ID_2))
			||(ft6x06_ts->panel_ic_id == ((FT5x06_UPGRADE_ID_1 << 8)|FT5x06_UPGRADE_ID_2)))
			ft6x06_ts->points = 2;
		else 
			ft6x06_ts->points = CFG_MAX_TOUCH_POINTS;
	}
	if (ft6x06_ts->panel_ic_id == ((FT6x06_UPGRADE_ID_1 << 8)|FT6x06_UPGRADE_ID_2))
	{
		ft6x06_ts->freq_hopping_mode=TP_FREQ_HOPPING_MODE_AUTO;
	}
	else
	{
		ft6x06_ts->freq_hopping_mode=ft6x06_ts->panel_parm->freq_hopping_mode;
	}
	ft6x06_ts->charger_state=0;
	printk(KERN_INFO "%s:freq_hopping_mode %d\n",__func__,ft6x06_ts->freq_hopping_mode);
    ft6x06_ts->wakeup_enable = 0;
    ft6x06_ts->wakeup_event = ft6x06_ts->panel_parm->wakeup_event;
    ft6x06_ts->wakeup_event_num = ft6x06_ts->panel_parm->wakeup_event_num;
//-add by hzb

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ft6x06_ts->input_dev = input_dev;
	

	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

    if (ft6x06_ts->wakeup_event_num)
    {
    	for(i=0;i<ft6x06_ts->wakeup_event_num;i++)
    	{
    		ft6x06_wakeup_support |= ft6x06_ts->wakeup_event[i].wakeup_event;
            g_ft6x06_wakeup_support = ((ft6x06_wakeup_support >> 4) & 0x01) | \
                                      ((ft6x06_wakeup_support << 1) & 0x1E) ;

            set_bit(ft6x06_ts->wakeup_event[i].key_code, input_dev->keybit);
    	}
    }
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, ft6x06_ts->x_max, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, ft6x06_ts->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);  //modify by hzb
	if (ft6x06_ts->pdata->slot_enable)
	{
		input_mt_init_slots(input_dev, ft6x06_ts->points,(INPUT_MT_POINTER | INPUT_MT_DIRECT));
	}
	else
	{
		input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, PRESS_MAX, 0, 0);
		input_set_abs_params(input_dev,ABS_MT_TRACKING_ID, 0, ft6x06_ts->points, 0, 0);
	}

//+add by hzb
    if(ft6x06_ts->panel_parm->virtual_key)
    {
       int i;
       for(i=0;i<ft6x06_ts->panel_parm->virtual_key_num; i++)
       {
           if ( ft6x06_ts->panel_parm->virtual_key[i].key_code )
           {
           
//+ add by wjz, quickcit mode, home change to A key 
#if 0
				if( (ontim_qucik_cit_boot_mode ==1) && (KEY_HOME ==ft6x06_ts->panel_parm->virtual_key[i].key_code ) )
				{
					printk(KERN_INFO "%s: quickcit change home to %d\n",__func__,KEY_A);
					ft6x06_ts->panel_parm->virtual_key[i].key_code = KEY_A;
				}
#endif
//- add by wjz, quickcit mode, home change to A key 
		   
	        set_bit(ft6x06_ts->panel_parm->virtual_key[i].key_code, input_dev->evbit);
		 printk(KERN_INFO "%s:Set key %d\n",__func__,ft6x06_ts->panel_parm->virtual_key[i].key_code);
           }
           else
           {
               break;
           }
       }
    }
//-add by hzb
	/* Proximity */
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, Proximity_Max, 0, 0);

	input_dev->name = FT6X06_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
			"ft6x06_ts_probe: failed to register input device: %s\n",
			dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
	
//#ifdef FTS_CTL_FACE_DETECT      //add by hzb
	ft6x06_ts->ps_state=10;
	ft6x06_ts->ps_onoff=0;
	ft6x06_ts->ps_input_dev = NULL;
	
	ontim_debug_als_prox.dev_name="als_prox";
       if ((pdata->prox_enable) && (ontim_dev_debug_file_exist(&ontim_debug_als_prox)<0))
       { 
	 
		ps_input_dev = input_allocate_device();

		if (!ps_input_dev) {
			err = -ENOMEM;
			dev_err(&client->dev, "failed to allocate ps input device\n");
			goto exit_ps_input_dev_alloc_failed;
		}

		ft6x06_ts->ps_input_dev = ps_input_dev;

		set_bit(EV_ABS,ps_input_dev->evbit);
		/*proximity*/
		input_set_abs_params(ps_input_dev, ABS_DISTANCE, 0, 2048, 0, 0);


		ps_input_dev->name = "als_prox";

		err = input_register_device(ps_input_dev);

		if (err) {
		    printk(KERN_ERR "%s: Unable to register input device: %s\n", __func__,ps_input_dev->name);
		goto exit_ps_input_register_device_failed;
		}

		err = sysfs_create_group(&ps_input_dev->dev.kobj,
		          &ft6x06_ps_attribute_group);
		if (err < 0)
		{
		    printk(KERN_ERR "at %s line %d create sysfs have a error \n",__func__,__LINE__);
		goto exit_ps_sysfs_create_group_failed;
		}

      }
      else
      {
      		printk(KERN_INFO "%s:PS sensor exist\n",__func__);
      }
//#endif

	/*make sure CTP already finish startup process */

	if ( ft6x06_ts->panel_parm->virtual_key && ft6x06_ts->panel_parm->virtual_key_num)  //add by hzb
	{
	    int retval=0;
	    struct kobject *properties_kobj;
	    /*virtual key init here begin*/
	    properties_kobj = kobject_create_and_add("board_properties", NULL);
	    if (properties_kobj)
	         retval = sysfs_create_group(properties_kobj,
	                                   &ft6x06_ts_properties_attr_group);
	    if (!properties_kobj || retval)
	        printk(KERN_ERR "failed to create board_properties\n");
	    /*virtual key init end*/
	}

	/*get some register information */
	uc_reg_addr = FT6x06_REG_FW_VER;
	ft6x06_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	printk(KERN_INFO "[FTS] Firmware version = 0x%x\n", uc_reg_value);

	uc_reg_addr = FT6x06_REG_POINT_RATE;
	ft6x06_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	printk(KERN_INFO "[FTS] report rate is %dHz.\n",
		uc_reg_value * 10);

	uc_reg_addr = FT6x06_REG_THGROUP;
	ft6x06_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	printk(KERN_INFO "[FTS] touch threshold is %d.\n",
		uc_reg_value * 4);
#ifdef SYSFS_DEBUG
	ft6x06_create_sysfs(client);
#endif
#ifdef FTS_APK_DEBUG
	ft6x06_create_apk_debug_channel(client);
#endif

#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(client) < 0)
		dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",
				__func__);
#endif
#if 0 //def FTS_CTL_FACE_DETECT
		if (ft_psensor_drv_init(client) < 0)
			dev_err(&client->dev, "%s:[FTS] create fts control psensor driver failed\n",
					__func__);
#endif
	fts_ctpm_auto_upgrade(client,force_updata);
	ft6x06_ts_updata_version();
	//reset
	if (pdata->reset)
	{
		pdata->reset(&client->dev);
	}
	else if (pdata->reset_gpio)
	{
		gpio_direction_output(pdata->reset_gpio, 1);
		msleep(10);
		gpio_direction_output(pdata->reset_gpio, 0);
		msleep(20);
		gpio_direction_output(pdata->reset_gpio, 1);
		msleep(120);
	}
	
//	ft6x06_ts->run_state = 1; //add by hzb
	if (ft6x06_ts->freq_hopping_mode != TP_FREQ_HOPPING_MODE_AUTO)
	{
		INIT_DELAYED_WORK(&ft6x06_ts->work, ft6x06_updata_freq_hopping);
	}
	err = request_threaded_irq(client->irq, NULL, ft6x06_ts_interrupt,
				   pdata->irqflags, client->dev.driver->name,
				   ft6x06_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft6x06_probe: request irq failed\n");
		//goto exit_irq_request_failed;
		printk(KERN_INFO "%s:Request IRQ %d error %d\n",__func__,client->irq,err);
	}
    if (ft6x06_wakeup_support)
    {
      device_init_wakeup(&client->dev, 1);
      INIT_DELAYED_WORK(&ft6x06_ts->wakeup_work, ft6x06_wakeup_work_func);
	
    }
#ifndef CONFIG_HAS_EARLYSUSPEND
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_enable(&client->dev);
	pm_runtime_forbid(&client->dev);
#endif
#endif
#ifdef CONFIG_USB_MV_UDC
	if (ft6x06_ts->freq_hopping_mode == TP_FREQ_HOPPING_MODE_CHARGER)
	{
		ft6x06_ts->chg_notif.notifier_call=ft6x06_charger_notifier;
		if (mv_udc_register_client(&ft6x06_ts->chg_notif) < 0) {
			printk(KERN_ERR "%s:  mv_udc_register_client Error!!!\n",__func__);
		}
		else
		{
			printk(KERN_INFO "%s:mv_udc_register OK\n",__func__);
		}
	}
#endif
	//ft6x06_write_reg(client, FT_FACE_DETECT_REG, 1);
	REGISTER_AND_INIT_ONTIM_DEBUG_FOR_THIS_DEV();
	if(!tp_probe_ok)//add by liuwei
		tp_probe_ok = 1;//add by liuwei
    printk("[kernel][end %s %d] ....",__func__,__LINE__);
	
	return 0;

exit_ps_sysfs_create_group_failed:
	input_unregister_device(ps_input_dev);
exit_ps_input_register_device_failed:
	input_free_device(ps_input_dev);
exit_ps_input_dev_alloc_failed:
	input_unregister_device(input_dev);
	
exit_input_register_device_failed:
	input_free_device(input_dev);

exit_input_dev_alloc_failed:
       if (pdata->irq_gpio)
       {
		free_irq(client->irq, ft6x06_ts);
		gpio_free(pdata->irq_gpio);
       }
	if(pdata->power_ic)
		pdata->power_ic(&client->dev, 0);
	if(pdata->power_i2c)
		pdata->power_i2c(&client->dev, 0);
#if 1 //def CONFIG_PM
exit_request_reset:
       if (pdata->reset_gpio)
		gpio_free(pdata->reset_gpio);
#endif

//exit_irq_request_failed:
	i2c_set_clientdata(client, NULL);
	mutex_destroy(&ft6x06_ts->ts_lock);
	mutex_destroy(&ft6x06_ts->ts_suspend_lock);
	kfree(ft6x06_ts);

exit_alloc_data_failed:
exit_check_functionality_failed:
	g_ft6x06_ts = NULL;
	return err;
}

#ifdef CONFIG_EARLYSUSPEND
static void ft6x06_ts_suspend(struct early_suspend *handler)
{
	char sleep_cmd[2]={FT6x06_REG_PMODE,PMODE_HIBERNATE};
	struct ft6x06_ts_data *ts = container_of(handler, struct ft6x06_ts_data,
						early_suspend);
	int  suspend=0;
	
	if (ts->wakeup_enable)
	{
		if (delayed_work_pending(&ts->wakeup_work))
			cancel_delayed_work_sync(&ts->wakeup_work);
	}
	mutex_lock(&ts->ts_suspend_lock);
	mutex_lock(&ts->ts_lock);
	//printk(KERN_INFO "%s: down -- sem = %d\n" ,__func__,--ft_sem);
	ts->suspend_state=1;
	if (ts->ps_onoff == 0)
	{
		printk(KERN_INFO "%s: [FTS]ft6x06 suspend[1]\n",__func__);
		ts->suspend_state=2;
		suspend =1;
	}
	//printk(KERN_INFO "%s: up -- sem = %d\n" ,__func__,++ft_sem);
	mutex_unlock(&ts->ts_lock);
	if (suspend)
	{
		printk(KERN_INFO "%s: [FTS]ft6x06 suspend[2]\n",__func__);
		ts->wakeup_enable=(ft6x06_wakeup_support & ft6x06_wakeup_enable);
		if (ts->wakeup_enable && (ts->pdata->irq_gpio >= 0))
		{
			struct irq_desc * tp_irq_desc;
			printk("[kernel]:%s:edge_wakeup_gpio gpio %d\n",__func__,ts->pdata->irq_gpio);
			disable_irq(ts->irq);
			ft6x06_write_reg(ts->client,0xD1, ts->wakeup_enable);
			ft6x06_write_reg(ts->client,0xD0, 1);
			//ts->wakeup_state=1;
			tp_irq_desc = irq_to_desc(ts->irq);//
			tp_irq_desc->action->flags = IRQF_TRIGGER_LOW| IRQF_NO_SUSPEND;
			irq_set_irq_type(ts->irq, IRQF_TRIGGER_LOW| IRQF_NO_SUSPEND); 
			enable_irq(ts->irq);
			enable_irq_wake(ts->irq);
			printk("[kernel]:%s:@@@@@@@@@@@@@@gesture on.@@@@@@@@@@@@@@@@@@\n",__func__);

		}
        else
        {
            disable_irq(ts->irq);
            ts->run_state=0;
            ft6x06_i2c_Write(ts->client,sleep_cmd, 2);
            if(ts->pdata->power_ic)
            ts->pdata->power_ic(&ts->client->dev, 0);
            if(ts->pdata->power_i2c)
            ts->pdata->power_i2c(&ts->client->dev, 0);
        }
    }
    mutex_unlock(&ts->ts_suspend_lock);

}

static void ft6x06_ts_resume(struct early_suspend *handler)
{
    struct ft6x06_ts_data *ts = container_of(handler, struct ft6x06_ts_data,
                                             early_suspend);
    int  resume=0;
    //int ret;
	if (ts->wakeup_enable)
	{
		if (delayed_work_pending(&ts->wakeup_work))
			cancel_delayed_work_sync(&ts->wakeup_work);
	}
	mutex_lock(&ts->ts_suspend_lock);
	mutex_lock(&ts->ts_lock);
	//printk(KERN_INFO "%s: 1\n" ,__func__);
	if (ts->suspend_state == 2)
	{
		if ( ts->wakeup_enable )
		{
			struct irq_desc * tp_irq_desc;
			printk("[kernel]:%s:edge_wakeup_gpio gpio %d\n",__func__,ts->pdata->irq_gpio);
			disable_irq_wake(ts->irq);
			//disable_irq(ts->irq);
			tp_irq_desc = irq_to_desc(ts->client->irq);//
			tp_irq_desc->action->flags = ts->pdata->irqflags;
			irq_set_irq_type(ts->irq, ts->pdata->irqflags);
			ts->wakeup_enable=0;
			ft6x06_write_reg(ts->client,0xD0, 0);  
			enable_irq(ts->irq);
			printk("[kernel]:%s:@@@@@@@@@@@@@@gesture off in resume.@@@@@@@@@@@@@@@@@@\n",__func__);
			//ts->wakeup_state=0;

		}
		else
		{
			//printk(KERN_INFO "%s: 2\n" ,__func__);
			if (ts->pdata->power_on_when_sleep ==0)
			{
				if(ts->pdata->power_ic)
					ts->pdata->power_ic(&ts->client->dev, 1);
				if(ts->pdata->power_i2c)
					ts->pdata->power_i2c(&ts->client->dev, 1);

				if((ts->pdata->power_ic) ||(ts->pdata->power_i2c))
					msleep(10);
			}
			if (ts->pdata->reset)
			{
				ts->pdata->reset(&ts->client->dev);
			}
			else if (ts->pdata->reset_gpio)
			{
				//printk(KERN_INFO "%s: 3\n" ,__func__);
				gpio_direction_output(ts->pdata->reset_gpio, 1);
				msleep(10);
				gpio_direction_output(ts->pdata->reset_gpio, 0);
				msleep(20);
				gpio_direction_output(ts->pdata->reset_gpio, 1);
				msleep(10);
			}
			msleep(120);
			resume =1;
		}
	}
	ts->suspend_state=0;
	//printk(KERN_INFO "%s: 4\n" ,__func__);

	ft6x06_read_Touchdata(ts);
	if (ts->pdata->slot_enable)
	{
		int i;
		for (i = 0; i < ts->points; ++i) {
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
		}

	}
	else
	{
		input_report_abs(ts->input_dev,ABS_MT_TOUCH_MAJOR, 0);
		input_mt_sync(ts->input_dev); 
	}
	input_sync(ts->input_dev);
	mutex_unlock(&ts->ts_lock);
	if (ts->freq_hopping_mode != TP_FREQ_HOPPING_MODE_AUTO)
	{
		if (delayed_work_pending(&ts->work))
			cancel_delayed_work_sync(&ts->work);
		schedule_delayed_work(&ts->work, msecs_to_jiffies(500));
	}
	if (resume)
	{
		ts->run_state=1;
		enable_irq(ts->irq);
	}
	mutex_unlock(&ts->ts_suspend_lock);
	//printk(KERN_INFO "%s: exit\n" ,__func__);
}
#else
#ifdef CONFIG_PM_RUNTIME

static int ft6x06_runtime_suspend(struct device *dev)
{
	char sleep_cmd[2]={FT6x06_REG_PMODE,PMODE_HIBERNATE};
	struct i2c_client *client = container_of((struct device *)dev,
						 struct i2c_client, dev);

	struct ft6x06_ts_data *ts = i2c_get_clientdata(client);

	int  suspend=0;
	ontim_dev_dbg(0, "%s:\n",__func__);
	if (ts == NULL) return 0;
	
	if (delayed_work_pending(&ts->work))
		cancel_delayed_work_sync(&ts->work);
	//dump_stack();
	mutex_lock(&ts->ts_suspend_lock);
	mutex_lock(&ts->ts_lock);
	//printk(KERN_INFO "%s: down -- sem = %d\n" ,__func__,--ft_sem);
	ts->suspend_state=1;
	if (ts->ps_onoff == 0)
	{
		//printk(KERN_INFO "%s: [FTS]ft6x06 suspend[1]\n",__func__);
		ts->suspend_state=2;
		suspend =1;
	}
	//printk(KERN_INFO "%s: up -- sem = %d\n" ,__func__,++ft_sem);
	mutex_unlock(&ts->ts_lock);
	if (suspend)
	{
		//printk(KERN_INFO "%s: [FTS]ft6x06 suspend[2]\n",__func__);
	    ts->wakeup_enable=(ft6x06_wakeup_support & ft6x06_wakeup_enable);
        if (ts->wakeup_enable && (ts->pdata->irq_gpio >= 0))
        {
            struct irq_desc * tp_irq_desc;
            printk("%s:edge_wakeup_gpio gpio %d\n",__func__,ts->pdata->irq_gpio);
            if (delayed_work_pending(&ts->wakeup_work))
                cancel_delayed_work_sync(&ts->wakeup_work);
            if (request_mfp_edge_wakeup(ts->pdata->irq_gpio, ft6x06_event_wakeup,(void *)ts, &ts->client->dev))
                printk("%s:failed to request edge wakeup.\n",__func__);
            ft6x06_write_reg(ts->client,0xD1, ts->wakeup_enable);
            ft6x06_write_reg(ts->client,0xD0, 1);
            disable_irq(ts->irq);
            //ts->wakeup_state=1;
            tp_irq_desc = irq_to_desc(ts->client->irq);//
            tp_irq_desc->action->flags = IRQF_TRIGGER_FALLING |IRQF_TRIGGER_RISING| IRQF_ONESHOT | IRQF_DISABLED | IRQF_NO_SUSPEND;
            enable_irq(ts->irq);
            enable_irq_wake(ts->irq);
        }
        else
        {
		disable_irq(ts->irq);
		ts->run_state=0;
		ft6x06_i2c_Write(ts->client,sleep_cmd, 2);
		if (ts->pdata->power_on_when_sleep ==0)
		{
			if(ts->pdata->power_ic)
				ts->pdata->power_ic(&ts->client->dev, 0);
			if(ts->pdata->power_i2c)
				ts->pdata->power_i2c(&ts->client->dev, 0);
	        }
		}
	}
	mutex_unlock(&ts->ts_suspend_lock);
	ontim_dev_dbg(0, "%s:Exit\n",__func__);
	return 0;

}

static int ft6x06_runtime_resume(struct device *dev)
{
	struct i2c_client *client = container_of((struct device *)dev,
						 struct i2c_client, dev);
	struct ft6x06_ts_data *ts = i2c_get_clientdata(client);
	//int ret;
	int  resume=0;
	ontim_dev_dbg(0, "%s: \n",__func__);
	if (ts == NULL) return 0;
	//dump_stack();
	mutex_lock(&ts->ts_suspend_lock);
	mutex_lock(&ts->ts_lock);
	//printk(KERN_INFO "%s: down -- sem = %d\n" ,__func__,--ft_sem);
	if (ts->suspend_state == 2)
	{
		if (ts->wakeup_enable)
		{
			struct irq_desc * tp_irq_desc;
			printk("%s:edge_wakeup_gpio gpio %d\n",__func__,ts->pdata->irq_gpio);
			disable_irq_wake(ts->irq);
			disable_irq(ts->irq);
			if (delayed_work_pending(&ts->wakeup_work))
				cancel_delayed_work_sync(&ts->wakeup_work);
			tp_irq_desc = irq_to_desc(ts->client->irq);//
			tp_irq_desc->action->flags = ts->pdata->irqflags;
			enable_irq(ts->irq);
			ts->wakeup_enable=0;
			remove_mfp_edge_wakeup(ts->pdata->irq_gpio);
			ft6x06_write_reg(ts->client,0xD0, 0);
			//ts->wakeup_state=0;

		}
		else
		{
			//printk(KERN_INFO "%s: [FTS]ft6x06 resume.\n",__func__);
			if (ts->pdata->power_on_when_sleep ==0)
			{
				if(ts->pdata->power_ic)
					ts->pdata->power_ic(&ts->client->dev, 1);
				if(ts->pdata->power_i2c)
					ts->pdata->power_i2c(&ts->client->dev, 1);

				if((ts->pdata->power_ic) ||(ts->pdata->power_i2c))
					msleep(10);
			}
			if (ts->pdata->reset)
			{
				ts->pdata->reset(&ts->client->dev);
			}
			else if (ts->pdata->reset_gpio)
			{
				gpio_direction_output(ts->pdata->reset_gpio, 1);
				msleep(10);
				gpio_direction_output(ts->pdata->reset_gpio, 0);
				msleep(20);
				gpio_direction_output(ts->pdata->reset_gpio, 1);
				msleep(10);
			}
				//msleep(120);
			resume =1;
	       }
	}
	ts->suspend_state=0;
	//printk(KERN_INFO "%s: up -- sem = %d\n" ,__func__,++ft_sem);
	//ft6x06_read_Touchdata(ts);
	if (ts->pdata->slot_enable)
	{
		int i;
		for (i = 0; i < ts->points; ++i) {
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
		}

	}
	else
	{
		input_report_abs(ts->input_dev,ABS_MT_TOUCH_MAJOR, 0);
		input_mt_sync(ts->input_dev); 
	}
	input_sync(ts->input_dev);
	
	if ((ts->ps_onoff)&& (ts->ps_input_dev))
	{
		msleep(120);
		ts->ps_state=255;
		ft6x06_write_reg(ts->client, FT_FACE_DETECT_REG, 1);
		input_report_abs(ts->ps_input_dev, 
					ABS_DISTANCE, ts->ps_state);
		input_sync(ts->ps_input_dev);
	}
	
	mutex_unlock(&ts->ts_lock);
	if (ts->freq_hopping_mode != TP_FREQ_HOPPING_MODE_AUTO)
	{
		if (delayed_work_pending(&ts->work))
			cancel_delayed_work_sync(&ts->work);
		schedule_delayed_work(&ts->work, msecs_to_jiffies(500));
	}
	if (resume)
	{
//		ts->run_state=1;
		enable_irq(ts->irq);
	}
	
	mutex_unlock(&ts->ts_suspend_lock);
	ontim_dev_dbg(0, "%s:Exit\n",__func__);
	return 0;
}

static const struct dev_pm_ops ft6x06_ts_pmops = {
	SET_RUNTIME_PM_OPS(ft6x06_runtime_suspend,
			   ft6x06_runtime_resume, NULL)
};

#endif
#endif

static int ft6x06_ts_remove(struct i2c_client *client)
{
	struct ft6x06_ts_data *ft6x06_ts;
	ft6x06_ts = i2c_get_clientdata(client);

#ifdef CONFIG_USB_MV_UDC
	if (ft6x06_ts->freq_hopping_mode == TP_FREQ_HOPPING_MODE_CHARGER)
	{
		mv_udc_unregister_client(&ft6x06_ts->chg_notif);
	}
#endif
#ifndef CONFIG_EARLYSUSPEND
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_disable(&client->dev);
#endif
#endif

	if (ft6x06_ts->ps_input_dev)   //add by hzb
	{
		sysfs_remove_group(&client->dev.kobj, &ft6x06_ts_properties_attr_group);
		input_unregister_device(ft6x06_ts->ps_input_dev);
		input_free_device(ft6x06_ts->ps_input_dev);
	}
	
	input_unregister_device(ft6x06_ts->input_dev);
	input_free_device(ft6x06_ts->input_dev);
	
       if (ft6x06_ts->pdata->irq_gpio)
       {
		free_irq(client->irq, ft6x06_ts);
		gpio_free(ft6x06_ts->pdata->irq_gpio);
       }
	if(ft6x06_ts->pdata->power_ic)
		ft6x06_ts->pdata->power_ic(&client->dev, 0);
	if(ft6x06_ts->pdata->power_i2c)
		ft6x06_ts->pdata->power_i2c(&client->dev, 0);
	
	#if 1 //def CONFIG_PM
       if (ft6x06_ts->pdata->reset_gpio)
       {
		gpio_free(ft6x06_ts->pdata->reset_gpio);
       }
	#endif
	#ifdef FTS_APK_DEBUG
	ft6x06_release_apk_debug_channel();
	#endif
	#ifdef SYSFS_DEBUG
	ft6x06_release_sysfs(client);
	#endif
	#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
	#endif
	#if 0 //def FTS_CTL_FACE_DETECT
	ft_psensor_drv_exit();
	#endif
	kfree(ft6x06_ts);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id ft6x06_ts_id[] = {
	{FT6X06_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ft6x06_ts_id);

static struct of_device_id ft6x06_dt_ids[] = {
	{ .compatible = FT6X06_NAME, },
	{}
};

static struct i2c_driver ft6x06_ts_driver = {
	.probe = ft6x06_ts_probe,
	.remove = ft6x06_ts_remove,
	.id_table = ft6x06_ts_id,
	.driver = {
		.name = FT6X06_NAME,
		.owner = THIS_MODULE,
#ifndef CONFIG_HAS_EARLYSUSPEND
#ifdef CONFIG_PM_RUNTIME
		.pm = &ft6x06_ts_pmops,
#endif
#endif
	.of_match_table = of_match_ptr(ft6x06_dt_ids),
		   },
};

static int __init ft6x06_ts_init(void)
{
	int ret;
	ret = i2c_add_driver(&ft6x06_ts_driver);
	if (ret) {
		printk(KERN_WARNING "Adding ft6x06 driver failed "
		       "(errno = %d)\n", ret);
	} else {
		pr_info("Successfully added driver %s\n",
			ft6x06_ts_driver.driver.name);
	}
	return ret;
}

static void __exit ft6x06_ts_exit(void)
{
	i2c_del_driver(&ft6x06_ts_driver);
}

late_initcall(ft6x06_ts_init);
module_exit(ft6x06_ts_exit);

MODULE_AUTHOR("<luowj>");
MODULE_DESCRIPTION("FocalTech ft6x06 TouchScreen driver");
MODULE_LICENSE("GPL");

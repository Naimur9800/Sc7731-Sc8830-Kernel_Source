/* drivers/input/touchscreen/gt9xx.c
 * 
 * 2010 - 2012 Goodix Technology.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be a reference 
 * to you, when you are integrating the GOODiX's CTP IC into your system, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * General Public License for more details.
 * 
 * Version:1.4
 * Author:andrew@goodix.com
 * Release Date:2012/12/12
 * Revision record:
 *      V1.0:2012/08/31,first Release
 *      V1.2:2012/10/15,modify gtp_reset_guitar,slot report,tracking_id & 0x0F
 *      V1.4:2012/12/12,modify gt9xx_update.c
 *      
 */

#include <linux/irq.h>
#include "gt9xx.h"

int gtp_rst_port;
int gtp_int_port;

//#if GTP_ICS_SLOT_REPORT
#include <linux/input/mt.h>
//#endif

//+add by hzb
static struct goodix_ts_data *ts_data=NULL;
extern uint32_t ontim_qucik_cit_boot_mode; //  factorycit
extern s32 gtp_test_sysfs_init(void);
extern void gtp_test_sysfs_deinit(void);

#include <ontim/ontim_dev_dgb.h>

static char gt9xx_version[]="gt9xx ontim ver 1.0";
static char gt9xx_vendor_name[50]="no_vendor";
    DEV_ATTR_DECLARE(touch_screen)
    DEV_ATTR_DEFINE("version",gt9xx_version)
    DEV_ATTR_DEFINE("vendor",gt9xx_vendor_name)
    DEV_ATTR_DECLARE_END;
    ONTIM_DEBUG_DECLARE_AND_INIT(touch_screen,touch_screen,8);
//-add by hzb

static const char *goodix_ts_name = "goodix-ts";
static struct workqueue_struct *goodix_wq;
struct i2c_client * i2c_connect_client = NULL; 
static u8 config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH]
                = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};

#if GTP_HAVE_TOUCH_KEY
	static const u16 touch_key_array[] = GTP_KEY_TAB;
	#define GTP_MAX_KEY_NUM	 (sizeof(touch_key_array)/sizeof(touch_key_array[0]))
#endif

static s8 gtp_i2c_test(struct i2c_client *client);
void gtp_reset_guitar(struct i2c_client *client, s32 ms);
void gtp_int_sync(s32 ms);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h);
static void goodix_ts_late_resume(struct early_suspend *h);
#endif
 
#if GTP_CREATE_WR_NODE
extern s32 init_wr_node(struct i2c_client*);
extern void uninit_wr_node(void);
#endif

#if GTP_AUTO_UPDATE
extern u8 gup_init_update_proc(struct goodix_ts_data *);
#endif

#if GTP_ESD_PROTECT
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct * gtp_esd_check_workqueue = NULL;
static void gtp_esd_check_func(struct work_struct *);
#endif

/*******************************************************	
Function:
	Read data from the i2c slave device.

Input:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:read data buffer.
	len:operate length.
	
Output:
	numbers of i2c_msgs to transfer
*********************************************************/
s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
    struct i2c_msg msgs[2];
    s32 ret=-1;
    s32 retries = 0;

    GTP_DEBUG_FUNC();

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = client->addr;
    msgs[0].len   = GTP_ADDR_LENGTH;
    msgs[0].buf   = &buf[0];

    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = client->addr;
    msgs[1].len   = len - GTP_ADDR_LENGTH;
    msgs[1].buf   = &buf[GTP_ADDR_LENGTH];

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, msgs, 2);
        if(ret == 2)break;
        retries++;
    }
    if(retries >= 5)
    {
        GTP_DEBUG("I2C retry timeout, reset chip.");
        //gtp_reset_guitar(client, 10);
    }
    return ret;
}

/*******************************************************	
Function:
	write data to the i2c slave device.

Input:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:write data buffer.
	len:operate length.
	
Output:
	numbers of i2c_msgs to transfer.
*********************************************************/
s32 gtp_i2c_write(struct i2c_client *client,u8 *buf,s32 len)
{
    struct i2c_msg msg;
    s32 ret=-1;
    s32 retries = 0;

    GTP_DEBUG_FUNC();

    msg.flags = !I2C_M_RD;
    msg.addr  = client->addr;
    msg.len   = len;
    msg.buf   = buf;

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret == 1)break;
        retries++;
    }
    if(retries >= 5)
    {
        GTP_DEBUG("I2C retry timeout, reset chip.");
        //gtp_reset_guitar(client, 10);
    }
    return ret;
}

#if GTP_ESD_PROTECT
/*******************************************************
Function:
    switch on & off esd delayed work
Input:
    client:  i2c device
    on:      SWITCH_ON / SWITCH_OFF
Output:
    void
*********************************************************/
void gtp_esd_switch(struct i2c_client *client, s32 on)
{
    struct goodix_ts_data *ts;
    
    ts = i2c_get_clientdata(client);
    mutex_lock(&ts->esd_lock);
    
    if (SWITCH_ON == on)     // switch on esd 
    {
        if (!ts->esd_running)
        {
            ts->esd_running = 1;
             GTP_INFO("Esd enable");
        }
    }
    else    // switch off esd
    {
        if (ts->esd_running)
        {
            ts->esd_running = 0;
            GTP_INFO("Esd disable");
        }
    }
    mutex_unlock(&ts->esd_lock);
}

/*******************************************************
Function:
    Initialize external watchdog for esd protect
Input:
    client:  i2c device.
Output:
    result of i2c write operation. 
        1: succeed, otherwise: failed
*********************************************************/
static s32 gtp_init_ext_watchdog(struct i2c_client *client)
{
    u8 opr_buffer[4] = {0x80, 0x40, 0xAA, 0xAA};
    
    struct i2c_msg msg;         // in case of recursively reset by calling gtp_i2c_write
    s32 ret = -1;
    s32 retries = 0;

    GTP_DEBUG("Init external watchdog...");
    GTP_DEBUG_FUNC();

    msg.flags = !I2C_M_RD;
    msg.addr  = client->addr;
    msg.len   = 4;
    msg.buf   = opr_buffer;

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret == 1)
        {
            return 1;
        }
        retries++;
    }
    if (retries >= 5)
    {
        GTP_ERROR("init external watchdog failed!");
    }
    return 0;
}
#endif



/*******************************************************
Function:
	Send config Function.

Input:
	client:	i2c client.

Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
s32 gtp_send_cfg(struct i2c_client *client)
{
    s32 ret = 0;
    
#if GTP_DRIVER_SEND_CFG
    s32 retry = 0;

    for (retry = 0; retry < 5; retry++)
    {
        ret = gtp_i2c_write(client, config , GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
        if (ret > 0)
        {
            break;
        }
    }
#endif

    return ret;
}

/*******************************************************
Function:
	Enable IRQ Function.

Input:
	ts:	i2c client private struct.
	
Output:
	None.
*******************************************************/
void gtp_irq_disable(struct goodix_ts_data *ts)
{
    unsigned long irqflags;

    GTP_DEBUG_FUNC();

    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (!ts->irq_is_disable)
    {
        ts->irq_is_disable = 1; 
        disable_irq_nosync(ts->client->irq);
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
	Disable IRQ Function.

Input:
	ts:	i2c client private struct.
	
Output:
	None.
*******************************************************/
void gtp_irq_enable(struct goodix_ts_data *ts)
{
    unsigned long irqflags = 0;

    GTP_DEBUG_FUNC();
    
    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (ts->irq_is_disable) 
    {
        enable_irq(ts->client->irq);
        ts->irq_is_disable = 0;	
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
	Touch down report function.

Input:
	ts:private data.
	id:tracking id.
	x:input x.
	y:input y.
	w:input weight.
	
Output:
	None.
*******************************************************/
static void gtp_touch_down(struct goodix_ts_data* ts,s32 id,s32 x,s32 y,s32 w)
{
#if GTP_CHANGE_X2Y
    GTP_SWAP(x, y);
#endif

	if (ts->pdata->slot_enable)
	{
	    input_mt_slot(ts->input_dev, id);
	    input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
	    //input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
	    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	    input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
	    //input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
	}
	else
	{
	    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	    input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
	    input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
	    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
	    input_mt_sync(ts->input_dev);
	}

    ontim_dev_dbg(2,"ID:%d, X:%d, Y:%d, W:%d", id, x, y, w);
	GTP_DEBUG("ID:%d, X:%d, Y:%d, W:%d", id, x, y, w);
}

/*******************************************************
Function:
	Touch up report function.

Input:
	ts:private data.
	
Output:
	None.
*******************************************************/
static void gtp_touch_up(struct goodix_ts_data* ts, s32 id)
{
	if (ts->pdata->slot_enable)
	{
	    input_mt_slot(ts->input_dev, id);
	    //input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
	    input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
	}
	else
	{
	    input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	    input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
	    input_mt_sync(ts->input_dev);
	}
	GTP_DEBUG("Touch id[%2d] release!", id);
    ontim_dev_dbg(2,"Touch id[%2d] release!", id);
}

/*******************************************************
Function:
	Goodix touchscreen work function.

Input:
	work:	work_struct of goodix_wq.
	
Output:
	None.
*******************************************************/
static void goodix_ts_work_func(struct work_struct *work)
{
    u8  end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
    u8  point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1]={GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF};
    u8  touch_num = 0;
    u8  finger = 0;
    static u16 pre_touch = 0;
    static u8 pre_key = 0;
    u8  key_value = 0;
    u8* coor_data = NULL;
    s32 input_x = 0;
    s32 input_y = 0;
    s32 input_w = 0;
    s32 id = 0;
    s32 i  = 0;
    s32 ret = -1;
    struct goodix_ts_data *ts = NULL;

    GTP_DEBUG_FUNC();

    ts = container_of(work, struct goodix_ts_data, work);
    if (ts->enter_update)
    {
        return;
    }
#ifndef CONFIG_CORE_VOLT_FIX_1250 
    pm_qos_update_request_timeout(&ts->cpufreq_qos_req_min,LONG_MAX, 200000);
#endif
    ret = gtp_i2c_read(ts->client, point_data, 12);
    if (ret < 0)
    {
        GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
        goto exit_work_func;
    }

    finger = point_data[GTP_ADDR_LENGTH];    
    if((finger & 0x80) == 0)
    {
        goto exit_work_func;
    }

    touch_num = finger & 0x0f;
    if (touch_num > GTP_MAX_TOUCH)
    {
        goto exit_work_func;
    }

    if (touch_num > 1)
    {
        u8 buf[8 * GTP_MAX_TOUCH] = {(GTP_READ_COOR_ADDR + 10) >> 8, (GTP_READ_COOR_ADDR + 10) & 0xff};

        ret = gtp_i2c_read(ts->client, buf, 2 + 8 * (touch_num - 1)); 
        memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
    }
#if GTP_HAVE_ONTIM_TOUCH_KEY
    key_value = point_data[3 + 8 * touch_num];
    GTP_DEBUG( "%s: key_value=0x%x,pre_key=0x%x\n",__func__,key_value,pre_key);	
    
    if(key_value || pre_key)
    {
        int key_num=ts->panel_parm->virtual_key_num;
        u8 key=0;
        key_value &= ((0x01<<key_num) -1);
        key = key_value ^ pre_key;
        GTP_DEBUG( "%s: key_value=0x%x,pre_key=0x%x,key=0x%x\n",__func__,key_value,pre_key,key);	
        for (i = 0; i < ts->panel_parm->virtual_key_num; i++)
        {
            if (key & (0x01<<i))
            {
                if ((key & (0x01<<i)) & key_value)
                {
                    gtp_touch_down(ts, 0, ts->panel_parm->virtual_key[i].logic_x, ts->panel_parm->virtual_key[i].logic_y, 1);
                    GTP_DEBUG("%s: KEY DOWN ---- X=%d,Y=%d\n",__func__,ts->panel_parm->virtual_key[i].logic_x, ts->panel_parm->virtual_key[i].logic_y);	
                }
                else
                {
                    gtp_touch_up(ts, 0);
                    GTP_DEBUG("%s: KEY UP \n",__func__);	
                }
            }
        }
        touch_num = 0;
        pre_touch = 0;
    }

#elif GTP_HAVE_TOUCH_KEY
    key_value = point_data[3 + 8 * touch_num];
   
    if(key_value || pre_key)
    {
        for (i = 0; i < GTP_MAX_KEY_NUM; i++)
        {
            input_report_key(ts->input_dev, touch_key_array[i], key_value & (0x01<<i));   
        }
        touch_num = 0;
        pre_touch = 0;
    }
#endif
    pre_key = key_value;

    GTP_DEBUG("pre_touch:%02x, finger:%02x.", pre_touch, finger);

	if (ts->pdata->slot_enable)
	{
	    if (pre_touch || touch_num)
	    {
	        s32 pos = 0;
	        u16 touch_index = 0;

	        coor_data = &point_data[3];
	        if(touch_num)
	        {
	            id = coor_data[pos] & 0x0F;
	            touch_index |= (0x01<<id);
	        }

	        GTP_DEBUG("id=%d,touch_index=0x%x,pre_touch=0x%x\n",id, touch_index,pre_touch);
	        for (i = 0; i < GTP_MAX_TOUCH; i++)
	        {
	            if (touch_index & (0x01<<i))
	            {
	                input_x  = coor_data[pos + 1] | coor_data[pos + 2] << 8;
	                input_y  = coor_data[pos + 3] | coor_data[pos + 4] << 8;
	                input_w  = coor_data[pos + 5] | coor_data[pos + 6] << 8;

	                gtp_touch_down(ts, id, input_x, input_y, input_w);
	                pre_touch |= 0x01 << i;

	                pos += 8;
	                id = coor_data[pos] & 0x0F;
	                touch_index |= (0x01<<id);
	            }
	            else// if (pre_touch & (0x01 << i))
	            {
	                gtp_touch_up(ts, i);
	                pre_touch &= ~(0x01 << i);
	            }
	        }
	    }
	    input_report_key(ts->input_dev, BTN_TOUCH, (touch_num || key_value));
	}
	else
	{
	    if (touch_num )
	    {
	        for (i = 0; i < touch_num; i++)
	        {
	            coor_data = &point_data[i * 8 + 3];

	            id = coor_data[0] & 0x0F;
	            input_x  = coor_data[1] | coor_data[2] << 8;
	            input_y  = coor_data[3] | coor_data[4] << 8;
	            input_w  = coor_data[5] | coor_data[6] << 8;

	            gtp_touch_down(ts, id, input_x, input_y, input_w);
	        }
	    }
	    else if (pre_touch)
	    {
	        GTP_DEBUG("Touch Release!");
	        gtp_touch_up(ts, 0);
	    }

	    pre_touch = touch_num;
	    input_report_key(ts->input_dev, BTN_TOUCH, (touch_num || key_value));
	}

    input_sync(ts->input_dev);

exit_work_func:
    if(!ts->gtp_rawdiff_mode)
    {
        ret = gtp_i2c_write(ts->client, end_cmd, 3);
        if (ret < 0)
        {
            GTP_INFO("I2C write end_cmd  error!"); 
        }
    }

    if (ts->use_irq)
    {
        gtp_irq_enable(ts);
    }
}

/*******************************************************
Function:
	Timer interrupt service routine.

Input:
	timer:	timer struct pointer.
	
Output:
	Timer work mode. HRTIMER_NORESTART---not restart mode
*******************************************************/
static enum hrtimer_restart goodix_ts_timer_handler(struct hrtimer *timer)
{
    struct goodix_ts_data *ts = container_of(timer, struct goodix_ts_data, timer);

    GTP_DEBUG_FUNC();

    queue_work(goodix_wq, &ts->work);
    hrtimer_start(&ts->timer, ktime_set(0, (GTP_POLL_TIME+6)*1000000), HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}

/*******************************************************
Function:
	External interrupt service routine.

Input:
	irq:	interrupt number.
	dev_id: private data pointer.
	
Output:
	irq execute status.
*******************************************************/
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
    struct goodix_ts_data *ts = dev_id;

    GTP_DEBUG_FUNC();
    ts->check_esd_irq_state=0;

    if (!ts->irq_is_disable)
    {
	    gtp_irq_disable(ts);
	    queue_work(goodix_wq, &ts->work);
    }

    return IRQ_HANDLED;
}
/*******************************************************
Function:
	Int sync Function.

Input:
	ms:sync time.
	
Output:
	None.
*******************************************************/
void gtp_int_sync(s32 ms)
{
    GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
    msleep(ms);
    GTP_GPIO_AS_INT(GTP_INT_PORT);
}

/*******************************************************
Function:
	Reset chip Function.

Input:
	ms:reset time.
	
Output:
	None.
*******************************************************/
void gtp_reset_guitar(struct i2c_client *client, s32 ms)
{
    struct goodix_ts_data *ts;
    
    ts = i2c_get_clientdata(client);
    GTP_DEBUG_FUNC();
#if GTP_ESD_PROTECT
    mutex_lock(&ts->esd_lock);
#endif
    GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);   //begin select I2C slave addr
    msleep(ms);
    GTP_GPIO_OUTPUT(GTP_INT_PORT, client->addr == 0x14);

    msleep(2);
    GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);
    
    msleep(6);                          //must > 3ms
    GTP_GPIO_AS_INPUT(GTP_RST_PORT);    //end select I2C slave addr
    
    gtp_int_sync(50);
#if GTP_ESD_PROTECT
    gtp_init_ext_watchdog(client);
   mutex_unlock(&ts->esd_lock);
#endif
 
}

/*******************************************************
Function:
	Eter sleep function.

Input:
	ts:private data.
	
Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_enter_sleep(struct goodix_ts_data * ts)
{
    s8 ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 5};

    GTP_DEBUG_FUNC();

    GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
    GTP_GPIO_OUTPUT(GTP_RST_PORT, 1); //add by hzb
    msleep(5);
    while(retry++ < 5)
    {
        ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret > 0)
        {
            GTP_DEBUG("GTP enter sleep!");
            return ret;
        }
        msleep(10);
    }
    GTP_ERROR("GTP send sleep cmd failed.");
    return ret;
}

/*******************************************************
Function:
	Wakeup from sleep mode Function.

Input:
	ts:	private data.
	
Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_wakeup_sleep(struct goodix_ts_data * ts)
{
    u8 retry = 0;
    s8 ret = -1;

    GTP_DEBUG_FUNC();

#if GTP_POWER_CTRL_SLEEP
    while(retry++ < 5)
    {
        gtp_reset_guitar(ts->client, 20);
        ret = gtp_send_cfg(ts->client);
        if (ret > 0)
        {
            GTP_DEBUG("Wakeup sleep send config success.");
            return ret;
        }
    }
#else
    while(retry++ < 10)
    {
        GTP_GPIO_OUTPUT(GTP_INT_PORT, 1);
        msleep(5);
        
        ret = gtp_i2c_test(ts->client);
        if (ret > 0)
        {
            GTP_DEBUG("GTP wakeup sleep.");
            
            gtp_int_sync(25);
            return ret;
        }
        gtp_reset_guitar(ts->client, 20);
    }
#endif

    GTP_ERROR("GTP wakeup sleep failed.");
    return ret;
}

/*******************************************************
Function:
	GTP initialize function.

Input:
	ts:	i2c client private struct.
	
Output:
	Executive outcomes.0---succeed.
*******************************************************/
static s32 gtp_init_panel(struct goodix_ts_data *ts)
{
    s32 ret = -1;
  
#if GTP_DRIVER_SEND_CFG
    s32 i;
    u8 check_sum = 0;
    u8 rd_cfg_buf[16];
#if 0
    u8 cfg_info_group1[] = CTP_CFG_GROUP1;
    u8 cfg_info_group2[] = CTP_CFG_GROUP2;
    u8 cfg_info_group3[] = CTP_CFG_GROUP3;
    u8 *send_cfg_buf[3] = {cfg_info_group1, cfg_info_group2, cfg_info_group3};
    u8 cfg_info_len[3] = {sizeof(cfg_info_group1)/sizeof(cfg_info_group1[0]), 
                          sizeof(cfg_info_group2)/sizeof(cfg_info_group2[0]),
                          sizeof(cfg_info_group3)/sizeof(cfg_info_group3[0])};
    for(i=0; i<3; i++)
    {
        if(cfg_info_len[i] > ts->gtp_cfg_len)
        {
            ts->gtp_cfg_len = cfg_info_len[i];
        }
    }
    GTP_DEBUG("len1=%d,len2=%d,len3=%d,send_len:%d",cfg_info_len[0],cfg_info_len[1],cfg_info_len[2],ts->gtp_cfg_len);
    if ((!cfg_info_len[1]) && (!cfg_info_len[2]))
    {
        rd_cfg_buf[GTP_ADDR_LENGTH] = 0; 
    }
    else
    {
        rd_cfg_buf[0] = GTP_REG_SENSOR_ID >> 8;
        rd_cfg_buf[1] = GTP_REG_SENSOR_ID & 0xff;
        ret = gtp_i2c_read(ts->client, rd_cfg_buf, 3);
        if (ret < 0)
        {
            GTP_ERROR("Read SENSOR ID failed,default use group1 config!");
            rd_cfg_buf[GTP_ADDR_LENGTH] = 0;
        }
        rd_cfg_buf[GTP_ADDR_LENGTH] &= 0x07;
    }
    ts->gtp_sensor_id = rd_cfg_buf[GTP_ADDR_LENGTH];  //add by hzb
    GTP_DEBUG("SENSOR ID:%d", rd_cfg_buf[GTP_ADDR_LENGTH]);
    memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
    memcpy(&config[GTP_ADDR_LENGTH], send_cfg_buf[rd_cfg_buf[GTP_ADDR_LENGTH]], ts->gtp_cfg_len);
	
    ts->panel_parm = &(ts->pdata->panel_parm[0]);  //add by hzb
#else
    //u8 *send_cfg_buf;

    {
        rd_cfg_buf[0] = GTP_REG_SENSOR_ID >> 8;
        rd_cfg_buf[1] = GTP_REG_SENSOR_ID & 0xff;
        ret = gtp_i2c_read(ts->client, rd_cfg_buf, 3);
        if (ret < 0)
        {
            GTP_ERROR("Read SENSOR ID failed,default use group1 config!");
            rd_cfg_buf[GTP_ADDR_LENGTH] = 0;
        }
        rd_cfg_buf[GTP_ADDR_LENGTH] &= 0x07;
    }
    GTP_INFO("SENSOR ID:%d", rd_cfg_buf[GTP_ADDR_LENGTH]);
    ts->gtp_sensor_id = rd_cfg_buf[GTP_ADDR_LENGTH];
    for(i=0;i<ts->pdata->panel_parm_num;i++ )
    {
        if (ts->gtp_sensor_id == ts->pdata->panel_parm[i].vendor_id ) break;
    }
    if (i>= ts->pdata->panel_parm_num)
    {
        GTP_INFO("Sensor ID is 0x%x , Not find panel config, Use default config!!!",ts->gtp_sensor_id);
	 ts->panel_parm = &(ts->pdata->panel_parm[0]);
	 ts->panel_parm->panal_cfg_size=0;
	 ts->panel_parm->panal_cfg == NULL;
	 ts->panel_parm->panel_fw_size=0;
	 ts->panel_parm->panel_fw== NULL;
    }
    else
    {
	 ts->panel_parm = &(ts->pdata->panel_parm[i]);
    }
    if ((ts->panel_parm->panal_cfg_size == 0) ||
		(ts->panel_parm->panal_cfg == NULL))
    {
        GTP_INFO("Config is NULL, GTP read resolution & max_touch_num , use panel config!");
        ts->gtp_cfg_len = GTP_CONFIG_MAX_LENGTH;
        ret = gtp_i2c_read(ts->client, config, GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
        if (ret < 0)
        {
            GTP_ERROR("GTP read resolution & max_touch_num failed!");
            return -1;
        }
    }
    else
    {
        ts->gtp_cfg_len=ts->panel_parm->panal_cfg_size;
        memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
        memcpy(&config[GTP_ADDR_LENGTH], ts->panel_parm->panal_cfg, ts->gtp_cfg_len);
    }
#endif
//+add by hzb
#if GTP_AUTO_UPDATE
    gup_init_update_proc(ts);
#endif
//-add by hzb


#if GTP_CUSTOM_CFG
    config[RESOLUTION_LOC]     = (u8)GTP_MAX_WIDTH;
    config[RESOLUTION_LOC + 1] = (u8)(GTP_MAX_WIDTH>>8);
    config[RESOLUTION_LOC + 2] = (u8)GTP_MAX_HEIGHT;
    config[RESOLUTION_LOC + 3] = (u8)(GTP_MAX_HEIGHT>>8);
    
    if (GTP_INT_TRIGGER == 0)  //RISING
    {
        config[TRIGGER_LOC] &= 0xfe; 
    }
    else if (GTP_INT_TRIGGER == 1)  //FALLING
    {
        config[TRIGGER_LOC] |= 0x01;
    }
#endif  //endif GTP_CUSTOM_CFG
    
    check_sum = 0;
    for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
    {
        check_sum += config[i];
    }
    config[ts->gtp_cfg_len] = (~check_sum) + 1;
    
#else //else DRIVER NEED NOT SEND CONFIG

    if(ts->gtp_cfg_len == 0)
    {
        ts->gtp_cfg_len = GTP_CONFIG_MAX_LENGTH;
    }
    ret = gtp_i2c_read(ts->client, config, ts->gtp_cfg_len + GTP_ADDR_LENGTH);
    if (ret < 0)
    {
        GTP_ERROR("GTP read resolution & max_touch_num failed, use default value!");
        ts->abs_x_max = GTP_MAX_WIDTH;
        ts->abs_y_max = GTP_MAX_HEIGHT;
        ts->int_trigger_type = GTP_INT_TRIGGER;
    }
#endif //endif GTP_DRIVER_SEND_CFG

    GTP_DEBUG_FUNC();

    ts->abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
    ts->abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
    ts->int_trigger_type = (config[TRIGGER_LOC]) & 0x03;
    if ((!ts->abs_x_max)||(!ts->abs_y_max))
    {
        GTP_ERROR("GTP resolution & max_touch_num invalid, use default value!");
        ts->abs_x_max = GTP_MAX_WIDTH;
        ts->abs_y_max = GTP_MAX_HEIGHT;
    }

    ret = gtp_send_cfg(ts->client);
    if (ret < 0)
    {
        GTP_ERROR("Send config error.");
    }

    GTP_DEBUG("X_MAX = %d,Y_MAX = %d,TRIGGER = 0x%02x",
             ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);

    msleep(10);

    return 0;
}

/*******************************************************
Function:
	Read goodix touchscreen version function.

Input:
	client:	i2c client struct.
	version:address to store version info
	
Output:
	Executive outcomes.0---succeed.
*******************************************************/
s32 gtp_read_version(struct i2c_client *client, u16* version)
{
    s32 ret = -1;
    s32 i = 0;
    u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};

    GTP_DEBUG_FUNC();

    ret = gtp_i2c_read(client, buf, sizeof(buf));
    if (ret < 0)
    {
        GTP_ERROR("GTP read version failed");
        return ret;
    }

    if (version)
    {
        *version = (buf[7] << 8) | buf[6];
    }

    for(i=2; i<6; i++)
    {
        if(!buf[i])
        {
            buf[i] = 0x30;
        }
    }
    GTP_INFO("IC VERSION:%c%c%c%c_%02x%02x", 
              buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);

    return ret;
}

/*******************************************************
Function:
	I2c test Function.

Input:
	client:i2c client.
	
Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_i2c_test(struct i2c_client *client)
{
    u8 test[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
    u8 retry = 0;
    s8 ret = -1;
  
    GTP_DEBUG_FUNC();
  
    while(retry++ < 5)
    {
        ret = gtp_i2c_read(client, test, 3);
        if (ret > 0)
        {
            return ret;
        }
        GTP_ERROR("GTP i2c test failed time %d.",retry);
        msleep(10);
    }
    return ret;
}

/*******************************************************
Function:
	Request gpio Function.

Input:
	ts:private data.
	
Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_request_io_port(struct goodix_ts_data *ts)
{
    s32 ret = 0;

    ret = GTP_GPIO_REQUEST(GTP_INT_PORT, "GTP_INT_IRQ");
    if (ret < 0) 
    {
        GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32)GTP_INT_PORT, ret);
        ret = -ENODEV;
    }
    else
    {
        GTP_GPIO_AS_INT(GTP_INT_PORT);	
        ts->client->irq = GTP_INT_IRQ;
    }
    GTP_DEBUG("irq:%d", ts->client->irq);

    ret = GTP_GPIO_REQUEST(GTP_RST_PORT, "GTP_RST_PORT");
    if (ret < 0) 
    {
        GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d",(s32)GTP_RST_PORT,ret);
        ret = -ENODEV;
    }

    GTP_GPIO_AS_INPUT(GTP_RST_PORT);
    gtp_reset_guitar(ts->client, 20);
    
    if(ret < 0)
    {
        GTP_GPIO_FREE(GTP_RST_PORT);
        GTP_GPIO_FREE(GTP_INT_PORT);
    }

    return ret;
}

/*******************************************************
Function:
	Request irq Function.

Input:
	ts:private data.
	
Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_request_irq(struct goodix_ts_data *ts)
{
    s32 ret = -1;
    const u8 irq_table[] = GTP_IRQ_TAB;

    GTP_DEBUG("INT trigger type:%x", ts->int_trigger_type);

    ts->irq_is_disable=1;  //add by hzb
    ret  = request_irq(ts->client->irq, 
                       goodix_ts_irq_handler,
                       irq_table[ts->int_trigger_type],
                       ts->client->name,
                       ts);
    if (ret)
    {
        GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
        GTP_GPIO_AS_INPUT(GTP_INT_PORT);
        GTP_GPIO_FREE(GTP_INT_PORT);

        hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        ts->timer.function = goodix_ts_timer_handler;
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
        return -1;
    }
    else 
    {
        //gtp_irq_disable(ts);
        disable_irq_nosync(ts->client->irq);
        ts->use_irq = 1;
        return 0;
    }
}

#if GTP_HAVE_ONTIM_TOUCH_KEY
static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
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

       if ( ts_data == NULL ) goto error;
	   
      	virtual_key_number = ts_data->panel_parm->virtual_key_num;
	vkey =  ts_data->panel_parm->virtual_key;
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
                vkey[i].logic_x=logic_x *ts_data->abs_x_max/ ts_data->panel_parm->x_max_res ;
                logic_y = vkey[i].logic_y;
                vkey[i].logic_y=logic_y *ts_data->abs_y_max/ ts_data->panel_parm->y_max_res ;
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

static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.goodix-ts",
        .mode = S_IRUGO,
    },
    .show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] = {
    &virtual_keys_attr.attr,
    NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};
#endif



/*******************************************************
Function:
	Request input device Function.

Input:
	ts:private data.
	
Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_request_input_dev(struct goodix_ts_data *ts)
{
    s8 ret = -1;
    s8 phys[32];
#if GTP_HAVE_TOUCH_KEY
    u8 index = 0;
#endif
  
    GTP_DEBUG_FUNC();
  
    ts->input_dev = input_allocate_device();
    if (ts->input_dev == NULL)
    {
        GTP_ERROR("Failed to allocate input device.");
        return -ENOMEM;
    }

#if GTP_HAVE_ONTIM_TOUCH_KEY
	if (ts->panel_parm)
	{
		if(ts->panel_parm->virtual_key)
		{
			int i;
			for(i=0;i<ts->panel_parm->virtual_key_num; i++)
			{
				if ( ts->panel_parm->virtual_key[i].key_code )
				{

					//+ add by wjz, quickcit mode, home change to A key 
				#if 0
					if( (ontim_qucik_cit_boot_mode ==1) && (KEY_HOME ==ts->panel_parm->virtual_key[i].key_code ) )
					{
						printk(KERN_INFO "%s: quickcit change home to %d\n",__func__,KEY_A);
						ts->panel_parm->virtual_key[i].key_code = KEY_A;
					}
				#endif
					//- add by wjz, quickcit mode, home change to A key 

					set_bit(ts->panel_parm->virtual_key[i].key_code, ts->input_dev->evbit);
					printk(KERN_INFO "%s:Set key %d\n",__func__,ts->panel_parm->virtual_key[i].key_code);
				}
				else
				{
					break;
				}
			}
		}
	}

#elif GTP_HAVE_TOUCH_KEY
    for (index = 0; index < GTP_MAX_KEY_NUM; index++)
    {
        input_set_capability(ts->input_dev,EV_KEY,touch_key_array[index]);	
    }
#endif

#if GTP_CHANGE_X2Y
    GTP_SWAP(ts->abs_x_max, ts->abs_y_max);
#endif
    printk(KERN_INFO "%s:x_max= %d; y_max= %d\n",__func__,ts->abs_x_max,ts->abs_y_max);

    set_bit(EV_KEY, ts->input_dev->evbit);
    set_bit(EV_ABS, ts->input_dev->evbit);
    set_bit(BTN_TOUCH, ts->input_dev->keybit);
	
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);	
    if (ts->pdata->slot_enable)
    {
        input_mt_init_slots(ts->input_dev, GTP_MAX_TOUCH,(INPUT_MT_POINTER | INPUT_MT_DIRECT));
    }
    else
    {
	    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, GTP_MAX_TOUCH, 0, 0);
	    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
    }

    sprintf(phys, "input/ts");
    ts->input_dev->name = goodix_ts_name;
    ts->input_dev->phys = phys;
    ts->input_dev->id.bustype = BUS_I2C;
    ts->input_dev->id.vendor = 0xDEAD;
    ts->input_dev->id.product = 0xBEEF;
    ts->input_dev->id.version = 10427;
    ts->input_dev->dev.parent = &ts->client->dev;
	
    ret = input_register_device(ts->input_dev);
    if (ret)
    {
        GTP_ERROR("Register %s input device failed", ts->input_dev->name);
        return -ENODEV;
    }
	
#if GTP_HAVE_ONTIM_TOUCH_KEY
	if ( ts->panel_parm->virtual_key && ts->panel_parm->virtual_key_num)  //add by hzb
	{
	    int retval=0;
	    struct kobject *properties_kobj;
	    /*virtual key init here begin*/
	    properties_kobj = kobject_create_and_add("board_properties", NULL);
	    if (properties_kobj)
	         retval = sysfs_create_group(properties_kobj,
	                                   &properties_attr_group);
	    if (!properties_kobj || retval)
	        printk(KERN_ERR "failed to create board_properties\n");
	    /*virtual key init end*/
	}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    ts->early_suspend.suspend = goodix_ts_early_suspend;
    ts->early_suspend.resume = goodix_ts_late_resume;
    register_early_suspend(&ts->early_suspend);
#endif

    return 0;
}
//+add by hzb
static void goodix_ts_set_power(struct goodix_ts_data *ts, int on )
{
	on=!!on;
	if(ts->pdata->power_ic)
		ts->pdata->power_ic(&ts->client->dev, on);
	if(ts->pdata->power_i2c)
		ts->pdata->power_i2c(&ts->client->dev, on);
	if((ts->pdata->power_ic) ||(ts->pdata->power_i2c))
		msleep(10);
}
//-add by hzb
/*******************************************************
Function:
	Goodix touchscreen probe function.

Input:
	client:	i2c device struct.
	id:device id.
	
Output:
	Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    s32 ret = -1;
    struct goodix_ts_data *ts;
    u16 version_info;
//+add by hzb
    struct touchpanel_platform_data *pdata=NULL;
    pdata =(struct touchpanel_platform_data *)client->dev.platform_data;
//-add by hzb

    GTP_DEBUG_FUNC();
    if(CHECK_THIS_DEV_DEBUG_AREADY_EXIT()==0)
    { 
       return -EIO;
    }
    
    //do NOT remove these output log
    GTP_INFO("GTP Driver Version:%s",GTP_DRIVER_VERSION);
    GTP_INFO("GTP Driver build@%s,%s", __TIME__,__DATE__);
    GTP_INFO("GTP I2C Address:0x%02x", client->addr);

    i2c_connect_client = client;
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
    {
        GTP_ERROR("I2C check functionality failed.");
        return -ENODEV;
    }
//+add by hzb
    if (pdata == NULL)
    {
        GTP_ERROR("pdata failed.");
        return -EFAULT;
    }
//-add by hzb
    ts = kzalloc(sizeof(*ts), GFP_KERNEL);
    if (ts == NULL)
    {
        GTP_ERROR("Alloc GFP_KERNEL memory failed.");
        return -ENOMEM;
    }
   
    memset(ts, 0, sizeof(*ts));
    INIT_WORK(&ts->work, goodix_ts_work_func);
    ts->client = client;
    ts->pdata = pdata; //add by hzb
    spin_lock_init(&ts->irq_lock);

    i2c_set_clientdata(client, ts);
//    ts->irq_lock = SPIN_LOCK_UNLOCKED;
    ts->gtp_rawdiff_mode = 0;
//+add by hzb
#if GTP_ESD_PROTECT
    mutex_init(&ts->esd_lock);
#endif
    goodix_ts_set_power(ts,1);
    if (pdata->pin_config)
    {
	pdata->pin_config();
    }
    GTP_RST_PORT=pdata->reset_gpio;
    GTP_INT_PORT=pdata->irq_gpio;
//-add by hzb
    ret = gtp_request_io_port(ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP request IO port failed.");
        goto request_io_error;
    }

    ret = gtp_i2c_test(client);
    if (ret < 0)
    {
        GTP_ERROR("I2C communication ERROR!");
        goto i2c_test_error;
    }

#if 0 //GTP_AUTO_UPDATE  //delete by hzb
    ret = gup_init_update_proc(ts);
    if (ret < 0)
    {
        GTP_ERROR("Create update thread error.");
        goto update_error;
    }
#endif
    
    ret = gtp_init_panel(ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP init panel failed.");
        goto init_error;
    }

//+add by hzb
    ts_data = ts;
//-add by hzb
    ret = gtp_request_input_dev(ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP request input dev failed");
        goto request_input_dev_error;
    }
    
    ret = gtp_request_irq(ts); 
    if (ret < 0)
    {
        GTP_INFO("GTP works in polling mode.");
    }
    else
    {
        GTP_INFO("GTP works in interrupt mode.");
    }

    ret = gtp_read_version(client, &version_info);
    if (ret < 0)
    {
        GTP_ERROR("Read version failed.");
        goto read_version_error;
    }
 //   ts->irq_lock = SPIN_LOCK_UNLOCKED;

#ifndef CONFIG_CORE_VOLT_FIX_1250 
    ts->cpufreq_qos_req_min.name = "goodix-ts";
    pm_qos_add_request(&ts->cpufreq_qos_req_min,
			PM_QOS_CPUFREQ_MIN,
			PM_QOS_DEFAULT_VALUE);
#endif

#if GTP_CREATE_WR_NODE
    init_wr_node(client);
#endif

#if GTP_ESD_PROTECT
    ts->check_esd_irq_state=0;
    //spin_lock_init(&ts->esd_lock);
    INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
    gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
    gtp_esd_switch(client,SWITCH_ON);
    queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, GTP_ESD_CHECK_CIRCLE); 
#endif
	if (ts->panel_parm->vendor_name)
	{
		sprintf(gt9xx_vendor_name,"%s\0",ts->panel_parm->vendor_name);
	}
#ifndef CONFIG_HAS_EARLYSUSPEND
#ifdef CONFIG_PM_RUNTIME
    	//gtp_irq_disable(ts);
	goodix_ts_set_power(ts,0);
	pm_runtime_enable(&client->dev);
	pm_runtime_forbid(&client->dev);
#endif
#else
    gtp_irq_enable(ts);
#endif
    gtp_test_sysfs_init();

    REGISTER_AND_INIT_ONTIM_DEBUG_FOR_THIS_DEV();
    return 0;

read_version_error:
	if (ts->use_irq)
	{
		 free_irq(ts->client->irq,ts);
	        GTP_GPIO_AS_INPUT(GTP_INT_PORT);
	        GTP_GPIO_FREE(GTP_INT_PORT);
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&ts->early_suspend);
#endif
	input_unregister_device(ts->input_dev);
request_input_dev_error:
//update_error:
init_error:
i2c_test_error:
        GTP_GPIO_FREE(GTP_RST_PORT);
        GTP_GPIO_FREE(GTP_INT_PORT);
request_io_error:
	goodix_ts_set_power(ts,0);
       kfree(ts);
        return ret;
	
}


/*******************************************************
Function:
	Goodix touchscreen driver release function.

Input:
	client:	i2c device struct.
	
Output:
	Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_ts_remove(struct i2c_client *client)
{
    struct goodix_ts_data *ts = i2c_get_clientdata(client);
	
    GTP_DEBUG_FUNC();
	
    gtp_test_sysfs_deinit();
	
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&ts->early_suspend);
else
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_disable(&client->dev);
#endif
#endif

#if GTP_CREATE_WR_NODE
    uninit_wr_node();
#endif

#if GTP_ESD_PROTECT
    destroy_workqueue(gtp_esd_check_workqueue);
#endif

    if (ts) 
    {
        if (ts->use_irq)
        {
            GTP_GPIO_AS_INPUT(GTP_INT_PORT);
            GTP_GPIO_FREE(GTP_INT_PORT);
            free_irq(client->irq, ts);
        }
        else
        {
            hrtimer_cancel(&ts->timer);
        }
    }	
	
    GTP_INFO("GTP driver is removing...");
    i2c_set_clientdata(client, NULL);
    input_unregister_device(ts->input_dev);
    kfree(ts);

    return 0;
}

/*******************************************************
Function:
	Early suspend function.

Input:
	h:early_suspend struct.
	
Output:
	None.
*******************************************************/
#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h)
{
    struct goodix_ts_data *ts;
    s8 ret = -1;	
    ts = container_of(h, struct goodix_ts_data, early_suspend);
	
    GTP_DEBUG_FUNC();

#if GTP_ESD_PROTECT
    ts->gtp_is_suspend = 1;
    cancel_delayed_work_sync(&gtp_esd_check_work);
#endif

    if (ts->use_irq)
    {
        gtp_irq_disable(ts);
    }
    else
    {
        hrtimer_cancel(&ts->timer);
    }
    ret = gtp_enter_sleep(ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP early suspend failed.");
    }
}

/*******************************************************
Function:
	Late resume function.

Input:
	h:early_suspend struct.
	
Output:
	None.
*******************************************************/
static void goodix_ts_late_resume(struct early_suspend *h)
{
    struct goodix_ts_data *ts;
    s8 ret = -1;
    ts = container_of(h, struct goodix_ts_data, early_suspend);
	
    GTP_DEBUG_FUNC();
	
    ret = gtp_wakeup_sleep(ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP later resume failed.");
    }

    if (ts->use_irq)
    {
        gtp_irq_enable(ts);
    }
    else
    {
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
    }

#if GTP_ESD_PROTECT
    ts->check_esd_irq_state=0;
    ts->gtp_is_suspend = 0;
    queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, GTP_ESD_CHECK_CIRCLE);
#endif
}
#else
#ifdef CONFIG_PM_RUNTIME
static int goodix_ts_runtime_suspend(struct device *dev)
{
    struct goodix_ts_data *ts;
    s8 ret = -1;	
    ts = ts_data;
    if (ts==NULL) return 0;
    GTP_DEBUG_FUNC();
    printk(KERN_INFO "%s:\n",__func__);
#if GTP_ESD_PROTECT
    ts->gtp_is_suspend = 1;
    cancel_delayed_work_sync(&gtp_esd_check_work);
#endif

    if (ts->use_irq)
    {
        gtp_irq_disable(ts);
    }
    else
    {
        hrtimer_cancel(&ts->timer);
    }
    ret = gtp_enter_sleep(ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP early suspend failed.");
    }
    goodix_ts_set_power(ts,0);
    GTP_GPIO_AS_INPUT(GTP_RST_PORT);    //add by hzb

    return 0;
}

/*******************************************************
Function:
	Late resume function.

Input:
	h:early_suspend struct.
	
Output:
	None.
*******************************************************/
static int goodix_ts_runtime_resume(struct device *dev)
{
    struct goodix_ts_data *ts;
    s8 ret = -1;
    ts = ts_data;
    if (ts==NULL) return 0;
    printk(KERN_INFO "%s:\n",__func__);
	
    goodix_ts_set_power(ts,1);
    GTP_DEBUG_FUNC();
    ret = gtp_wakeup_sleep(ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP later resume failed.");
    }
    if (ts->pdata->slot_enable)
    {
    	int i;
    	for (i = 0; i < GTP_MAX_TOUCH; ++i) {
    		input_mt_slot(ts->input_dev, i);
    		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
    	}
    }

    if (ts->use_irq)
    {
        gtp_irq_enable(ts);
    }
    else
    {
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
    }

#if GTP_ESD_PROTECT
    ts->gtp_is_suspend = 0;
    queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, GTP_ESD_CHECK_CIRCLE);
#endif
    return 0;
}

static const struct dev_pm_ops goodix_ts_pmops = {
	SET_RUNTIME_PM_OPS(goodix_ts_runtime_suspend,
			   goodix_ts_runtime_resume, NULL)
};

#endif
#endif

#if GTP_ESD_PROTECT
static void gtp_esd_check_func(struct work_struct *work)
{
    s32 i;
    s32 ret = -1;
    struct goodix_ts_data *ts = NULL;
    //u8 test[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
    u8 test[5] = {0x80,0x40,0,0,0};

    GTP_DEBUG_FUNC();

    ts = i2c_get_clientdata(i2c_connect_client);

    if (ts->gtp_is_suspend)
    {
        return;
    }
    mutex_lock(&ts->esd_lock);
    if (!ts->esd_running)
    {
	    if(!ts->gtp_is_suspend)
	    {
	        queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, GTP_ESD_CHECK_CIRCLE);
	    }
    	    mutex_unlock(&ts->esd_lock);
	    return;
    }
    
    //printk(KERN_INFO "%s:\n",__func__);
    for (i = 0; i < 3; i++)
    {
        ret = gtp_i2c_read(i2c_connect_client, test, 4);
        if (ret < 0)
        {
            continue;
        }
	 if (( test[2]==0xAA)||( test[3]!=0xAA))
	 {
    		printk(KERN_INFO "%s:Read IIC error!!!\n",__func__);
	 	i=3;
		break;
	 }
	 else
	 {
    		//printk(KERN_INFO "%s:Read IIC OK\n",__func__);
	 	test[2]=0xAA;
		gtp_i2c_write(i2c_connect_client, test, 3);
		break;
	 }
    }
#if 0
    if (GTP_GPIO_GET_VALUE(GTP_INT_PORT) == 0)
    {
    	ts->check_esd_irq_state++;
    }
    else
    {
    	ts->check_esd_irq_state=0;
    }
#endif	
    if ((i >= 3)||( ts->check_esd_irq_state>=3))
    {
	printk(KERN_INFO "%s:Reset TP!!! i=%d; check_esd_irq_state=%d\n",__func__,i,ts->check_esd_irq_state);
	ts->check_esd_irq_state=0;
       test[0] = 0x42;
       test[1] = 0x26;
       test[2] = 1;
       test[3] = 1;
       test[4] = 1;
	
        ret = gtp_i2c_write(i2c_connect_client, test, 5);
        if (ret < 0)
        {
        	printk(KERN_INFO "%s:Write IIC OK! ret= %d\n",__func__,ret);
        }
        msleep(300);
	    GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);   //begin select I2C slave addr
	    msleep(300);
	    GTP_GPIO_OUTPUT(GTP_INT_PORT, i2c_connect_client->addr == 0x14);

	    msleep(2);
	    GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);
	    
	    msleep(6);                          //must > 3ms
	    GTP_GPIO_AS_INPUT(GTP_RST_PORT);    //end select I2C slave addr
	    
	    gtp_int_sync(50);
#if GTP_ESD_PROTECT
	    gtp_init_ext_watchdog(i2c_connect_client);
#endif
    }

    if(!ts->gtp_is_suspend)
    {
        queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, GTP_ESD_CHECK_CIRCLE);
    }
    mutex_unlock(&ts->esd_lock);

    return;
}
#endif

static const struct i2c_device_id goodix_ts_id[] = {
    { GTP_I2C_NAME, 0 },
    { }
};

static struct i2c_driver goodix_ts_driver = {
    .probe      = goodix_ts_probe,
    .remove     = goodix_ts_remove,
    .id_table   = goodix_ts_id,
    .driver = {
        .name     = GTP_I2C_NAME,
        .owner    = THIS_MODULE,
#ifndef CONFIG_HAS_EARLYSUSPEND
#ifdef CONFIG_PM_RUNTIME
		.pm = &goodix_ts_pmops,
#endif
#endif
    },
};

/*******************************************************	
Function:
	Driver Install function.
Input:
  None.
Output:
	Executive Outcomes. 0---succeed.
********************************************************/
static int __init goodix_ts_init(void)
{
    s32 ret;

    GTP_DEBUG_FUNC();	
    GTP_INFO("GTP driver install.");
    goodix_wq = create_singlethread_workqueue("goodix_wq");
    if (!goodix_wq)
    {
        GTP_ERROR("Creat workqueue failed.");
        return -ENOMEM;
    }
    ret = i2c_add_driver(&goodix_ts_driver);
    return ret; 
}

/*******************************************************	
Function:
	Driver uninstall function.
Input:
  None.
Output:
	Executive Outcomes. 0---succeed.
********************************************************/
static void __exit goodix_ts_exit(void)
{
    GTP_DEBUG_FUNC();
    GTP_INFO("GTP driver exited.");
    i2c_del_driver(&goodix_ts_driver);
    if (goodix_wq)
    {
        destroy_workqueue(goodix_wq);
    }
}

late_initcall(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");

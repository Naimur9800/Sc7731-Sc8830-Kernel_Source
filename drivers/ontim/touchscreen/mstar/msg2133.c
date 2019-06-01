#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/i2c.h>
#include <linux/suspend.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <soc/sprd/gpio.h>
#include <linux/gpio.h>
#include <linux/dma-mapping.h>
#include <ontim/touchscreen/touchpanel.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#else
#ifdef CONFIG_PM_RUNTIME
#include <linux/pm_runtime.h>
#endif
#endif
#include <linux/input/mt.h>

#include <soc/sprd/i2c-sprd.h>
//#include <linux/i2c/msg2133_ts.h>
//#include <linux/i2c/msg2133_firmware.h>

#define MAX_CMD_LEN       (255)
#define MSG2133_OK         (0)
#define MSG2133_ERROR    (-1)
#define FIND_KEY_MODE      (1)
#define BACK_KEY_MODE     (2)
#define MENU_KEY_MODE     (4)
#define HOME_KEY_MODE    (8)
#define ONE_FIGER_MODE    (10)
#define TWO_FIGER_MODE   (11)
#define FIGER_UP_MODE      (12)
#define ONTIM_KEY_MODE    (100)
#define MS_TS_MSG21XX_X_MAX   (480)
#define MS_TS_MSG21XX_Y_MAX   (854)
#define MAX_TOUCH_FINGER 2
#define REPORT_PACKET_LENGTH  8

#if 0
struct TpdTouchDataT
{
    unsigned char  packet_id;
    unsigned char  x_h_y_h;
    unsigned char  x_l;
    unsigned char  y_l;
    unsigned char  disx_h_disy_h;
    unsigned char  disx_l;
    unsigned char  disy_l;
    unsigned char  checksum;
};
static struct TpdTouchDataT TpdTouchData;
#endif
struct msg2133_touch
{
    struct input_dev *idev;
    struct i2c_client *i2c;
    struct workqueue_struct *msg2133_wq;
    struct work_struct work;
    struct touchpanel_platform_data *data;
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif
    int irq;
    int four_key_value;
    int is_run;
    int is_suspend;
    unsigned char  major_version ;
    unsigned char  minor_version ;
    int vendor_id;
    unsigned char *vendor_name;
    struct touchpanel_panel_parm * panel_parm;
};

static struct msg2133_touch *msg2133_dev;

#define __CHIP_MSG_2133A__
static u32 crc32_table[256];
static u8 g_dwiic_info_data[1024]= {0};
static int FwDataCnt;
static u8 temp[33][1024];
static  char fw_version[20];
struct class *firmware_class;
struct device *firmware_cmd_dev;

typedef struct
{
    u16 X;
    u16 Y;
} TouchPoint_t;

typedef struct
{
    u8 nTouchKeyMode;
    u8 nTouchKeyCode;
    u8 nFingerNum;
    TouchPoint_t Point[MAX_TOUCH_FINGER];
} TouchScreenInfo_t;

typedef enum
{
    EMEM_MAIN = 0,
    EMEM_INFO,
    EMEM_ALL,
} EMEM_TYPE_t;

/*************************************add firmware updata********************************************/
#define TP_DEBUG      printk
#define FW_ADDR_MSG21XX   (0xC4>>1)
#define FW_ADDR_MSG21XX_TP   (0x4C>>1)
#define FW_UPDATE_ADDR_MSG21XX   (0x92>>1)
#define PROC_FIRMWARE_UPDATE
extern bool tp_probe_ok;//add by liuwei
static int i2c_master_recv_ext(struct i2c_client *client, unsigned char *buf , int count);
//static int i2c_master_send_ext(struct i2c_client *client, const char *buf , int count);

//static u8 temp[94][1024];


//+add by hzb
#include <ontim/ontim_dev_dgb.h>

static char msg213x_version[25]="msg213x ontim ver 1.0";
static char msg213x_vendor_name[50]="msg213x_vendor";
static char msg213x_info_vendor_ID[10]="Null";
static char msg213x_main_vendor_ID[10]="Null";
DEV_ATTR_DECLARE(touch_screen)
DEV_ATTR_DEFINE("version",msg213x_version)
DEV_ATTR_DEFINE("vendor",msg213x_vendor_name)
DEV_ATTR_DEFINE("vendor_id",msg213x_main_vendor_ID)
DEV_ATTR_DEFINE("info_vendor_id",msg213x_info_vendor_ID)
DEV_ATTR_DECLARE_END;
ONTIM_DEBUG_DECLARE_AND_INIT(touch_screen,touch_screen,8);
//-add by hzb


static void _HalTscrHWReset(void )
{
    if (msg2133_dev->data->reset_gpio)
    {
        gpio_direction_output(msg2133_dev->data->reset_gpio, 1);
        msleep(50);
        gpio_direction_output(msg2133_dev->data->reset_gpio, 0);
        msleep(50);
        gpio_direction_output(msg2133_dev->data->reset_gpio, 1);
        msleep(100);
    }
}

static int HalTscrCReadI2CSeq(u8 addr, u8 *read_data, u8 size)
{
    int rc;
    struct i2c_msg msgs[] =
    {
        {
            .addr = addr,
            .flags = I2C_M_RD,
            .len = size,
            .buf = read_data,
        },
    };

    rc = i2c_transfer(msg2133_dev->i2c->adapter, msgs, 1);
    if( rc <=  0 )
    {
        printk("[HalTscrCReadI2CSeq] error %d\n", rc);
        rc=MSG2133_ERROR;
    }
    return rc;
}

static int HalTscrCDevWriteI2CSeq(u8 addr, u8 *data, u16 size)
{
    int rc;
    struct i2c_msg msgs[] =
    {
        {
            .addr = addr,
            .flags = 0,
            .len = size,
            .buf = data,
        },
    };
    rc = i2c_transfer(msg2133_dev->i2c->adapter, msgs, 1);
    if( rc <=  0 )
    {
        printk("[HalTscrCDevWriteI2CSeq] error %d,addr = %d\n", rc, addr);
        //dump_stack() ;
        rc=MSG2133_ERROR;
    }
    return rc;
}

static int i2c_master_recv_ext(struct i2c_client *client, unsigned char *buf , int count)
{
    int ret = 0;
    int retry = 3;
    int recv_ok = MSG2133_ERROR;
    if (count > MAX_CMD_LEN)
    {
        printk("[i2c_master_recv_ext] exceed the max write length \n");
        return MSG2133_ERROR;
    }
    do
    {
        ret = i2c_master_recv(client,  buf, count);
        retry--;
        if (ret != count)
        {
            printk("[i2c_master_recv_ext] Error sent I2C ret = %d\n", ret);

        }
        else
        {
            recv_ok = MSG2133_OK;

        }
    }
    while ((ret != count) && (retry > 0));

    return recv_ok;
}

static int get_fw_version(void )
{
    unsigned short major_version;
    unsigned short minor_version;
    unsigned char dbbus_tx_data[3];
    unsigned char dbbus_rx_data[4] ;
    unsigned int version = 0;
    int ret=0;

    //unsigned short major=0, minor=0;
    printk("%s()....\n",  __func__);
    //char *fw_version = kzalloc(sizeof(char), GFP_KERNEL);
    //SM-BUS GET FW VERSION
#ifdef __CHIP_MSG_2133A__
    dbbus_tx_data[0] = 0x53;
    dbbus_tx_data[1] = 0x00;
    dbbus_tx_data[2] = 0x2A;
#else
    dbbus_tx_data[0] = 0x53;
    dbbus_tx_data[1] = 0x00;
    dbbus_tx_data[2] = 0x74;
#endif
    ret = HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_tx_data[0], 3);
    if (ret<0)
    {
        printk(KERN_ERR "%s %d: Error!!\n",__func__,__LINE__);
        return MSG2133_ERROR;
    }
    ret = HalTscrCReadI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0], 4);
    if (ret<0)
    {
        printk(KERN_ERR "%s %d: Error!!\n",__func__,__LINE__);
        msg2133_dev->major_version = 0;
        msg2133_dev->minor_version = 0;
        return -2;
    }

    major_version = (dbbus_rx_data[1] << 8) + dbbus_rx_data[0];
    minor_version = (dbbus_rx_data[3] << 8) + dbbus_rx_data[2];

    version = major_version;
    version = (version<<16);
    version  =  version | minor_version;
    printk("tp_version:0x%x\n",  version);
    TP_DEBUG("***major = %d ***\n", major_version);
    TP_DEBUG("***minor = %d ***\n", minor_version);
    sprintf(fw_version, "%03d%03d", major_version, minor_version);
    TP_DEBUG("***fw_version = %s ***\n", fw_version);
    msg2133_dev->major_version = major_version;
    msg2133_dev->minor_version = minor_version;
//	if (major_version || minor_version ) return MSG2133_ERROR;
    return 0;

}

static int tpd_initialize(struct msg2133_touch  *dev)
{
    int retval = MSG2133_OK;
    char buf[3] = {0x10, 0x3C, 0xD6};
    printk("%s()...\n",  __func__);
    if (msg2133_dev->data->reset_gpio)
        gpio_direction_output(msg2133_dev->data->reset_gpio, 1);
    msleep(100);
    retval = i2c_master_send(dev->i2c, buf, 3);
    retval = i2c_master_recv(dev->i2c, buf, 2);
    if(retval > MSG2133_OK)
    {
        printk("msg2133 Version:0x%02x,0x%02x\n", buf[0], buf[1]);
        return MSG2133_OK;
    }
    else
    {
        printk("msg2133 Version error\n");
        return MSG2133_ERROR;
    }
    return retval;
}

static unsigned char calc_ck(unsigned char *msg,  int len)
{
    int checksum = 0;
    int i;
    for(i = 0; i < len;  i++)
    {
        checksum += msg[i];
    }
    return (unsigned char )((-checksum)& 0xff);
}

static u32 Distance(u16 X,u16 Y,u16 preX,u16 preY)
{
    u32 temp=0;
    temp=(((X-preX)*(X-preX))+((Y-preY)*(Y-preY)));
    return temp;
}

static void msg2133_touch_work(struct work_struct *work)
{

    u8 val[8] = {0};
    u8 Checksum = 0;
    u8 i;
    u32 delta_x = 0, delta_y = 0;
    u32 u32X = 0;
    u32 u32Y = 0;
    //u8 touchkeycode = 0;
    static TouchScreenInfo_t *touchData=NULL;
    static u32 preKeyStatus=0;
    u32	result = 0;
    static u8 preTouchStatus; //Previous Touch VA Status;
    u8 press[2] = {0,0};
    static u8 prepress[2] = {0,0};
    static u8 preTouchNum=0;
    static u16 preX[2]= {0xffff,0xffff},preY[2]= {0xffff,0xffff};
    u16 XX[2]= {0,0},YY[2]= {0,0};
    u16 temp;
    u8 changepoints=0;

    //printk(KERN_INFO"------------------------msg21xx_do_work-\n");
    if(touchData==NULL)
    {
        touchData = kzalloc(sizeof(TouchScreenInfo_t), GFP_KERNEL);
    }
    memset(touchData, 0, sizeof(TouchScreenInfo_t));
    if (msg2133_dev->is_suspend)
    {
        return;
    }

    disable_irq_nosync(msg2133_dev->irq);
    mutex_lock(&msg2133_dev->idev->mutex);
    //TP_DEBUG("msg21xx_data_msg21xx_do_work  val=%f,&val[0] = %f ,device addr = %f\n", val, &val[0], msg21xx_i2c_client->addr);
    //result = i2c_master_recv(msg21xx_i2c_client->adapter,&val[0],REPORT_PACKET_LENGTH);
    //result=msg21xx_i2c_rx_data( &val[0], REPORT_PACKET_LENGTH);
    result=i2c_master_recv_ext(msg2133_dev->i2c,  &val[0], REPORT_PACKET_LENGTH  );
    /*Added by liumx for tp error 2013.11.21 start*/
    if (result <0 )
    {
        enable_irq(msg2133_dev->irq);
        mutex_unlock(&msg2133_dev->idev->mutex);
        return;
    }
    /*Added by liumx for tp error 2013.11.21 end*/
    //Checksum = Calculate_8BitsChecksum(&val[0], (REPORT_PACKET_LENGTH-1)); //calculate checksum
    Checksum = calc_ck(&val[0], (REPORT_PACKET_LENGTH-1));


    ontim_dev_dbg(1,"primary:: val[0]=%d,val[1]=%d,val[2]=%d,val[3]=%d,val[4]=%d,val[5]=%d,val[6]=%d,val[7]=%d\n",val[0],val[1],val[2],val[3],val[4],val[5],val[6],val[7]);
    if ((Checksum == val[7]) && (val[0] == 0x52))   //check the checksum  of packet
    {
        u32X = (((val[1] & 0xF0) << 4) | val[2]);         //parse the packet to coordinates
        u32Y = (((val[1] & 0x0F) << 8) | val[3]);

        delta_x = (((val[4] & 0xF0) << 4) | val[5]);
        delta_y = (((val[4] & 0x0F) << 8) | val[6]);

        //TP_DEBUG("primary:: u32X=%d,u32Y=%d\n",u32X,u32Y);

        //DBG("[HAL] u32X = %x, u32Y = %x", u32X, u32Y);
        //DBG("[HAL] delta_x = %x, delta_y = %x", delta_x, delta_y);

        if ((val[1] == 0xFF) && (val[2] == 0xFF) && (val[3] == 0xFF) && (val[4] == 0xFF) && (val[6] == 0xFF))
        {
            touchData->Point[0].X = 0; // final X coordinate
            touchData->Point[0].Y = 0; // final Y coordinate

            if((val[5]==0x0)||(val[5]==0xFF)||(preTouchStatus==1))// if((val[5]==0x0)||(val[5]==0xFF))
            {
                touchData->nFingerNum = 0; //touch end
                touchData->nTouchKeyCode = 0; //TouchKeyMode
                touchData->nTouchKeyMode = 0; //TouchKeyMode
                preTouchStatus=0;
            }
            else
            {
                int i;
                unsigned char keycode=0;
                keycode=val[5]; //TpdTouchData.disx_l;
                for (i=0; i<8; i++,keycode>>=1)
                {
                    if (keycode == 1) break;
                }
                ontim_dev_dbg(0, "%s:Touch key num %d\n",__func__,i);
                if ((msg2133_dev->panel_parm->virtual_key) && (msg2133_dev->panel_parm->virtual_key_num > i))
                {
                    if ( msg2133_dev->panel_parm->virtual_key[i].key_code )
                    {
                        printk(KERN_INFO "%s:Touch key %d\n",__func__,msg2133_dev->panel_parm->virtual_key[i].key_code);
                        //touchData->nTouchKeyMode = 1;
                        touchData->nFingerNum = 1; //one touch
                        touchData->Point[0].X = msg2133_dev->panel_parm->virtual_key[i].logic_x; // final X coordinate
                        touchData->Point[0].Y = msg2133_dev->panel_parm->virtual_key[i].logic_y; // final Y coordinate

                        press[0]=1;
                        press[1]=0;
                        preTouchStatus=0;
                    }
                    else
                    {
                        preTouchStatus=0;
                        touchData->nFingerNum = 0; //touch end
                    }
                }
                else
                {
                    preTouchStatus=0;
                    touchData->nFingerNum = 0; //touch end
                }
            }
        }
        else
        {
            touchData->nTouchKeyMode = 0; //Touch on screen...
            if ((delta_x == 0) && (delta_y == 0))
            {
                touchData->nFingerNum = 1; //one touch
                touchData->Point[0].X = (u32X *  msg2133_dev->panel_parm->x_max_res) / 2048;
                touchData->Point[0].Y = (u32Y *  msg2133_dev->panel_parm->y_max_res) / 2048;//1781;

                press[0]=1;
                press[1]=0;

                /* Calibrate if the touch panel was reversed in Y */
            }
            else
            {
                u32 x2, y2;

                touchData->nFingerNum = 2; //two touch

                /* Finger 1 */
                touchData->Point[0].X = (u32X * msg2133_dev->panel_parm->x_max_res ) / 2048;
                touchData->Point[0].Y = (u32Y * msg2133_dev->panel_parm->y_max_res ) / 2048;//1781;

                /* Finger 2 */
                if (delta_x > 2048)     //transform the unsigh value to sign value
                {
                    delta_x -= 4096;
                }
                if (delta_y > 2048)
                {
                    delta_y -= 4096;
                }

                x2 = (u32)(u32X + delta_x);
                y2 = (u32)(u32Y + delta_y);

                touchData->Point[1].X = (x2 *  msg2133_dev->panel_parm->x_max_res ) / 2048;
                touchData->Point[1].Y = (y2 *  msg2133_dev->panel_parm->y_max_res ) /2048;// 1781;

                press[0]=1;
                press[1]=1;

                /* Calibrate if the touch panel was reversed in Y */
            }
#if 1//check swap two points by sam
            if(preTouchStatus==1)
            {
                for(i=0; i<2; i++)
                {
                    XX[i]=touchData->Point[i].X;
                    YY[i]=touchData->Point[i].Y;
                }
                if(/*(touchData->nFingerNum==1)&&*/(preTouchNum==2))
                {
                    if(Distance(XX[0],YY[0],preX[0],preY[0])>Distance(XX[0],YY[0],preX[1],preY[1]))
                        changepoints=1;
                }
                if((touchData->nFingerNum==2)&&(preTouchNum==1))
                {
                    if(prepress[0]==1)
                    {
                        if(Distance(XX[0],YY[0],preX[0],preY[0])>Distance(XX[1],YY[1],preX[0],preY[0]))
                            changepoints=1;
                    }
                    else
                    {
                        if(Distance(XX[0],YY[0],preX[1],preY[1])<Distance(XX[1],YY[1],preX[1],preY[1]))
                            changepoints=1;
                    }
                }
                if((touchData->nFingerNum==1)&&(preTouchNum==1))
                {
                    if(press[0]!=prepress[0])
                        changepoints=1;
                }
                if((touchData->nFingerNum==2)&&(preTouchNum==2))
                {
                    //
                }

                if(changepoints==1)
                {
                    temp=press[0];
                    press[0]=press[1];
                    press[1]=temp;

                    temp=touchData->Point[0].X;
                    touchData->Point[0].X=touchData->Point[1].X;
                    touchData->Point[1].X=temp;

                    temp=touchData->Point[0].Y;
                    touchData->Point[0].Y=touchData->Point[1].Y;
                    touchData->Point[1].Y=temp;
                }


            }
            //save current status
            for(i=0; i<2; i++)
            {
                prepress[i]=press[i];
                preX[i]=touchData->Point[i].X;
                preY[i]=touchData->Point[i].Y;
            }
            preTouchNum=touchData->nFingerNum;
            //end of save current status
#endif

            preTouchStatus=1;
        }
        //TP_DEBUG("primary:: touchData->nTouchKeyMode[%d]\n",touchData->nTouchKeyMode);
        //report...
        {
            //printk("%s:F[%d],p1[%d],p2[%d]\n",__func__,touchData->nFingerNum,press[0],press[1]);
            if((touchData->nFingerNum) == 0)   //touch end
            {
                {
                    for(i=0; i<2; i++)
                    {
                        input_mt_slot(msg2133_dev->idev, i);
                        input_mt_report_slot_state(msg2133_dev->idev, MT_TOOL_FINGER, false);
                        //input_report_abs(input, ABS_MT_TRACKING_ID, -1);
                    }
                    input_report_key(msg2133_dev->idev, BTN_TOUCH, 0);
                }
                input_sync(msg2133_dev->idev);

            }
            else //touch on screen
            {
                if(touchData->nFingerNum)
                {
                    int r=0;
                    for (i = 0; i < 2; i++)
                    {
                        /* LCD view area */
                        //if (event->au8_touch_event[i] == FTS_POINT_INVALID ) continue;

                        input_mt_slot(msg2133_dev->idev, i);
                        if (press[i])
                        {
                            ontim_dev_dbg(2,"report:: u32X=%d,u32Y=%d,i=%d\n",touchData->Point[i].X,touchData->Point[i].Y,i);
                            input_mt_report_slot_state(msg2133_dev->idev, MT_TOOL_FINGER, true);

                            input_report_abs(msg2133_dev->idev, ABS_MT_POSITION_X,touchData->Point[i].X);
                            input_report_abs(msg2133_dev->idev, ABS_MT_POSITION_Y,touchData->Point[i].Y);
                            //input_report_abs(msg2133_dev->idev, ABS_MT_PRESSURE,127);
                            input_report_abs(msg2133_dev->idev,ABS_MT_TOUCH_MAJOR,16);
                        }
                        else
                        {
                            input_mt_report_slot_state(msg2133_dev->idev, MT_TOOL_FINGER, false);
                            r++;
                        }

                        //ontim_dev_dbg(2,"%s:x=%d;y= %d; p=%d; f=%d; e=%d; tp=%d; r =%d\n",__func__,event->au16_x[i],event->au16_y[i],event->pressure,event->au8_finger_id[i],event->au8_touch_event[i],event->touch_point,r);
                    }
                    if(2 == r)
                    {
                        for ( i=0; i<2; i++)
                        {
                            input_mt_slot(msg2133_dev->idev, i);
                            input_mt_report_slot_state(msg2133_dev->idev, MT_TOOL_FINGER, false);
                        }
                        input_report_key(msg2133_dev->idev, BTN_TOUCH, 0);
                    }
                    else
                        input_report_key(msg2133_dev->idev, BTN_TOUCH, 2-r);
                    input_sync(msg2133_dev->idev);

                }
            }
            preKeyStatus=0; //clear key status..
        }
    }

    enable_irq(msg2133_dev->irq);
    mutex_unlock(&msg2133_dev->idev->mutex);
}


static irqreturn_t  msg2133_touch_irq_handler(int irq, void *dev_id)
{
    if (msg2133_dev->is_run)
    {
        queue_work(msg2133_dev->msg2133_wq, &msg2133_dev->work);
    }
    return IRQ_HANDLED;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void msg2133_touch_sleep_early_suspend(struct early_suspend *h)
{
    //printk("%s()....\n",  __func__);
    msg2133_dev->is_suspend=1;
    mutex_lock(&msg2133_dev->idev->mutex);
    //disable_irq_nosync(msg2133_dev->irq);
    disable_irq(msg2133_dev->irq);
    //cancel_work_sync(&msg2133_dev->work);
    mutex_unlock(&msg2133_dev->idev->mutex);

    gpio_direction_output(msg2133_dev->data->reset_gpio, 1);
    msleep(100);
    gpio_direction_output(msg2133_dev->data->reset_gpio, 0);
    if (msg2133_dev->data->power_on_when_sleep==0)
        msg2133_dev->data->power_ic(msg2133_dev,0);
}

static void msg2133_touch_normal_late_resume(struct early_suspend *h)
{
    printk("%s()....\n",  __func__);
    mutex_lock(&msg2133_dev->idev->mutex);
    if (msg2133_dev->data->power_on_when_sleep==0)
        msg2133_dev->data->power_ic(msg2133_dev,1);
    tpd_initialize(msg2133_dev);
    enable_irq(msg2133_dev->irq);
    mutex_unlock(&msg2133_dev->idev->mutex);
    msg2133_dev->is_suspend=0;
}

static struct early_suspend msg2133_touch_early_suspend_desc =
{
    .level =  EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 1,
    .suspend = msg2133_touch_sleep_early_suspend,
    .resume = msg2133_touch_normal_late_resume,
};
#else
#ifdef CONFIG_PM_RUNTIME
static int msg2133_runtime_suspend(struct device *dev)
{
    printk("%s()....\n",  __func__);
    msg2133_dev->is_suspend=1;
    mutex_lock(&msg2133_dev->idev->mutex);
    disable_irq_nosync(msg2133_dev->irq);
    //cancel_work_sync(&msg2133_dev->work);
    mutex_unlock(&msg2133_dev->idev->mutex);
    gpio_direction_output(msg2133_dev->data->reset_gpio, 1);
    msleep(100);
    gpio_direction_output(msg2133_dev->data->reset_gpio, 0);
    if (msg2133_dev->data->power_on_when_sleep==0)
        msg2133_dev->data->power_ic(dev,0);
    return 0;

}

static int msg2133_runtime_resume(struct device *dev)
{
    printk("%s()....\n",  __func__);
    mutex_lock(&msg2133_dev->idev->mutex);
    if (msg2133_dev->data->power_on_when_sleep==0)
        msg2133_dev->data->power_ic(dev,1);
    tpd_initialize(msg2133_dev);
    enable_irq(msg2133_dev->irq);
    mutex_unlock(&msg2133_dev->idev->mutex);
    msg2133_dev->is_suspend=0;
    return 0;
}

static const struct dev_pm_ops msg2133_ts_pmops =
{
    SET_RUNTIME_PM_OPS(msg2133_runtime_suspend,
    msg2133_runtime_resume, NULL)
};
#endif
#endif


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

    if ( msg2133_dev == NULL ) goto error;
    if ( msg2133_dev->data == NULL ) goto error;

    virtual_key_number = msg2133_dev->panel_parm->virtual_key_num;
    vkey =  msg2133_dev->panel_parm->virtual_key;
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

static struct kobj_attribute virtual_keys_attr =
{
    .attr = {
        .name = "virtualkeys.msg2133",
        .mode = S_IRUGO,
    },
    .show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] =
{
    &virtual_keys_attr.attr,
    NULL
};

static struct attribute_group properties_attr_group =
{
    .attrs = properties_attrs,
};



static void dbbusDWIICEnterSerialDebugMode(void)
{
    u8 data[5];

    // Enter the Serial Debug Mode
    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;
    mdelay(15);
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 5);
}
static void dbbusDWIICStopMCU(void)
{
    u8 data[1];

    // Stop the MCU
    data[0] = 0x37;
    mdelay(15);
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}
static void dbbusDWIICIICUseBus(void)
{
    u8 data[1];

    // IIC Use Bus
    data[0] = 0x35;
    mdelay(15);
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}
static void dbbusDWIICIICReshape(void)
{
    u8 data[1];

    // IIC Re-shape
    data[0] = 0x71;
    mdelay(15);
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

void drvDB_WriteReg(u8 bBank, u8 bAddr, u16 bData)
{
    u8 bWriteData[5];
    bWriteData[0]=0x10;
    bWriteData[1] = bBank;
    bWriteData[2] = bAddr;
    bWriteData[4] = bData>>8;
    bWriteData[3] = bData&0xFF;
    mdelay(15);
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, bWriteData, 5);
}




void drvDB_WriteReg8Bit(u8 bBank, u8 bAddr, u8 bData)
{
    u8 bWriteData[5];
    bWriteData[0] = 0x10;
    bWriteData[1] = bBank;
    bWriteData[2] = bAddr;
    bWriteData[3] = bData;
    mdelay(15);
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, bWriteData, 4);
}

u16 drvDB_ReadReg(u8 bBank,u8 bAddr)
{
    u16 val=0;
    u8 bWriteData[3]= {0x10,bBank,bAddr};
    u8 bReadData[2]= {0x00,0x00};
    mdelay(15);
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, bWriteData, 3);

    mdelay(15);
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &bReadData[0], 2);

    val=(bReadData[1]<<8)|(bReadData[0]);

    return val;
}


u32 Reflect(u32 ref, char ch)//unsigned int Reflect(unsigned int ref, char ch)
{
    u32 value=0;
    u32 i=0;

    for(i = 1; i < (ch + 1); i++)
    {
        if(ref & 1)
            value |= 1 << (ch - i);
        ref >>= 1;
    }
    return value;
}
void Init_CRC32_Table(void)
{
    u32 magicnumber = 0x04c11db7;
    u32 i=0,j;

    for(i = 0; i <= 0xFF; i++)
    {
        crc32_table[i]=Reflect(i, 8) << 24;
        for (j = 0; j < 8; j++)
        {
            crc32_table[i] = (crc32_table[i] << 1) ^ (crc32_table[i] & (0x80000000L) ? magicnumber : 0);
        }
        crc32_table[i] = Reflect(crc32_table[i], 32);
    }
}

u32 Get_CRC(u32 text,u32 prevCRC)
{
    u32  ulCRC = prevCRC;
    {
        ulCRC = (ulCRC >> 8) ^ crc32_table[(ulCRC & 0xFF) ^ text];
    }
    return ulCRC ;
}

static int drvTP_erase_emem_A ( EMEM_TYPE_t emem_type )
{
    // stop mcu
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0001 );

    //disable watch dog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    // set PROGRAM password
    drvDB_WriteReg8Bit ( 0x16, 0x1A, 0xBA );
    drvDB_WriteReg8Bit ( 0x16, 0x1B, 0xAB );

    //proto.MstarWriteReg(F1.loopDevice, 0x1618, 0x80);
    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    if ( emem_type == EMEM_ALL )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x08, 0x10 ); //mark
    }

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x40 );
    mdelay ( 10 );

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    // erase trigger
    if ( emem_type == EMEM_MAIN )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x04 ); //erase main
    }
    else
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x08 ); //erase all block
    }

    return ( 1 );
}

static int drvTP_read_info_dwiic_A ( void )
{
    u8  dwiic_tx_data[5];
    //u8  dwiic_rx_data[4];
    u16 reg_data=0;
    //unsigned char dbbus_rx_data[2] = {0};

    mdelay ( 300 );


    // Stop Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    drvDB_WriteReg ( 0x3C, 0xE4, 0xA4AB );

    drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );

    // TP SW reset
    drvDB_WriteReg ( 0x1E, 0x04, 0x829F );
    mdelay (1);

    dwiic_tx_data[0] = 0x10;
    dwiic_tx_data[1] = 0x0F;
    dwiic_tx_data[2] = 0xE6;
    dwiic_tx_data[3] = 0x00;
    mdelay(15);
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dwiic_tx_data, 4 );

    // stop mcu
    //  drvDB_WriteReg ( 0x1E, 0xE6, 0x0001 );

    mdelay ( 100 );
    TP_DEBUG ( "read infor 1\n");

    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x5B58 );
    TP_DEBUG ( "read infor +++2\n");
    dwiic_tx_data[0] = 0x72;

    // dwiic_tx_data[3] = 0x04;
    //  dwiic_tx_data[4] = 0x00;
    dwiic_tx_data[3] = 0x00;
    dwiic_tx_data[4] = 0x80;

    for(reg_data=0; reg_data<8; reg_data++)
    {
        dwiic_tx_data[1] = 0x80+(((reg_data*128)&0xff00)>>8);
        dwiic_tx_data[2] = (reg_data*128)&0x00ff;
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , dwiic_tx_data, 5 );

        mdelay (50 );

        // recive info data
        HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX_TP, &g_dwiic_info_data[reg_data*128], 128);
    }
    return ( 1 );
}

/*
static int drvTP_info_updata_A ( u16 start_index, u8 *data, u16 size )
{
// size != 0, start_index+size !> 1024
u16 i;
for ( i = 0; i < size; i++ )
{
g_dwiic_info_data[start_index] = * ( data + i );
start_index++;
}
return ( 1 );
}
*/
static ssize_t firmware_update_c33 ( EMEM_TYPE_t emem_type )
{
    u8  life_counter[2];
    u32 i, j;
    u32 crc_main, crc_main_tp,crc_temp;
    u32 crc_info, crc_info_tp;

    int update_pass = 1;
    u16 reg_data = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;

    //msg21xx_i2c_client->timing = 100;
    drvTP_read_info_dwiic_A();

    if ( g_dwiic_info_data[0] == 'M' && g_dwiic_info_data[1] == 'S' && g_dwiic_info_data[2] == 'T' && g_dwiic_info_data[3] == 'A' && g_dwiic_info_data[4] == 'R' && g_dwiic_info_data[5] == 'T' && g_dwiic_info_data[6] == 'P' && g_dwiic_info_data[7] == 'C' )
    {
        // updata FW Version
        //drvTP_info_updata_C33 ( 8, &temp[32][8], 5 );

        g_dwiic_info_data[8]=temp[32][8];
        g_dwiic_info_data[9]=temp[32][9];
        g_dwiic_info_data[10]=temp[32][10];
        g_dwiic_info_data[11]=temp[32][11];
        // updata life counter
        life_counter[1] = (( ( (g_dwiic_info_data[13] << 8 ) | g_dwiic_info_data[12]) + 1 ) >> 8 ) & 0xFF;
        life_counter[0] = ( ( (g_dwiic_info_data[13] << 8 ) | g_dwiic_info_data[12]) + 1 ) & 0xFF;
        g_dwiic_info_data[12]=life_counter[0];
        g_dwiic_info_data[13]=life_counter[1];
        //drvTP_info_updata_C33 ( 10, &life_counter[0], 3 );
        drvDB_WriteReg ( 0x3C, 0xE4, 0x78C5 );
        drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );
        // TP SW reset
        drvDB_WriteReg ( 0x1E, 0x04, 0x829F );

        mdelay ( 100 );
        //polling 0x3CE4 is 0x2F43
        do
        {
            reg_data = drvDB_ReadReg (0x3C, 0xE4);
        }
        while ( reg_data != 0x2F43 );

        // transmit lk info data
        for(i=0; i<8; i++)
        {
            HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , &g_dwiic_info_data[i*128], 128 );
        }
        //polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );

    }

    //erase main
    drvTP_erase_emem_A ( EMEM_MAIN );
    msleep ( 1000 );/*Added by liumx 2013.11.22*/
    //ResetSlave();
    _HalTscrHWReset();
    //	mdelay ( 300 );
    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );
    /////////////////////////
    // Program
    /////////////////////////

    //polling 0x3CE4 is 0x1C70
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0x1C70 );
    }

    switch ( emem_type )
    {
    case EMEM_ALL:
        drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks
        break;
    case EMEM_MAIN:
        drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for main block
        break;
    case EMEM_INFO:
        drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for info block

        drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x01 );

        drvDB_WriteReg8Bit ( 0x3C, 0xE4, 0xC5 ); //
        drvDB_WriteReg8Bit ( 0x3C, 0xE5, 0x78 ); //

        drvDB_WriteReg8Bit ( 0x1E, 0x04, 0x9F );
        drvDB_WriteReg8Bit ( 0x1E, 0x05, 0x82 );

        drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x00 );
        mdelay ( 100 );
        break;
    }
    mdelay ( 100 );
    // polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x2F43 );

    // calculate CRC 32
    Init_CRC32_Table ();

    for ( i = 0; i < 33; i++ ) // total  33 KB : 2 byte per R/W
    {
        if ( emem_type == EMEM_INFO )
            i = 32;

        if ( i < 32 )   //emem_main
        {
            if ( i == 31 )
            {
                temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
                temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

                for ( j = 0; j < 1016; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main);
                }
                crc_temp=crc_main;
                crc_temp=crc_temp ^ 0xffffffff;

                for(j=0; j<4; j++)
                {
                    /**************************modify by sam 20131224**************************/
                    temp[i][1023-j]=(crc_temp>>8*j)&0xFF;
                    /**************************end of modify***********************************/
                    TP_DEBUG("Upate crc32 into bin buffer temp[%d][%d]=%x\n",i,(1020+j),temp[i][1020+j]);
                }
            }
            else
            {
                for ( j = 0; j < 1024; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main);
                }
            }
        }
        else  //emem_info
        {
            for ( j = 0; j < 1024; j++ )
            {
                //crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
                crc_info = Get_CRC ( g_dwiic_info_data[j], crc_info);
            }
            if ( emem_type == EMEM_MAIN ) break;
        }

        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
        for(j=0; j<8; j++)
        {
            HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP, &temp[i][j*128], 128 );
        }
        mdelay ( 100 );
        // polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );

        drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // write file done and check crc
        drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );
    }
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // polling 0x3CE4 is 0x9432
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0x9432 );
    }

    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // CRC Main from TP
        crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
        crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );

        // CRC Info from TP
        crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
        crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );
    }
    TP_DEBUG ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
               crc_main, crc_info, crc_main_tp, crc_info_tp );

    //drvDB_ExitDBBUS();

    update_pass = 1;
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        if ( crc_main_tp != crc_main )
            update_pass = 0;

        if ( crc_info_tp != crc_info )
            update_pass = 0;
    }


    _HalTscrHWReset();
    get_fw_version();
    FwDataCnt = 0;

    if ( !update_pass )
    {
        printk ( "update FAILED\n" );
        return ( 0 );
    }
    else
    {
        printk ( "update OK\n" );
    }
    return ( 1 );
}

static int firmware_auto_update( EMEM_TYPE_t emem_type )
{
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};

    _HalTscrHWReset();

    // Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    /////////////////////////
    // Difference between C2 and C3
    /////////////////////////
    // c2:2133 c32:2133a(2) c33:2138
    //check id
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCC;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    if ( dbbus_rx_data[0] == 2 )
    {
        // check version
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3C;
        dbbus_tx_data[2] = 0xEA;
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
        HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
        TP_DEBUG ( "dbbus_rx version[0]=0x%x", dbbus_rx_data[0] );

        if ( dbbus_rx_data[0] == 3 )
        {
            return firmware_update_c33(emem_type );
        }
    }
    _HalTscrHWReset();
    return 0;
}

static ssize_t firmware_update_store( struct device *dev, struct device_attribute *attr,
                                      const char *buf, size_t size)
{
    printk("%s\n",__func__);
    disable_irq(msg2133_dev->irq);

    firmware_auto_update(EMEM_MAIN);

    enable_irq(msg2133_dev->irq);
    return size;
}


static ssize_t firmware_update_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    printk("%s\n",__func__);
    return sprintf(buf, "%s\n", fw_version);
}
static DEVICE_ATTR(update, 0664, firmware_update_show, firmware_update_store);

/*test=================*/
static ssize_t firmware_clear_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    printk("%s\n",__func__);
    return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_clear_store(struct device *dev,
                                    struct device_attribute *attr, const char *buf, size_t size)
{
    printk("%s\n",__func__);
    return size;
}

static DEVICE_ATTR(clear, 0664, firmware_clear_show, firmware_clear_store);

/*test=================*/
/*Add by Tracy.Lin for update touch panel firmware and get fw version*/

static ssize_t firmware_version_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    printk("%s\n",__func__);
    TP_DEBUG("*** firmware_version_show fw_version = %s***\n", fw_version);
    return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_version_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
    TP_DEBUG("***fw_version = %s ***\n", fw_version);
    printk("%s\n",__func__);


    return size;
}
static DEVICE_ATTR(version, 0664, firmware_version_show, firmware_version_store);

static ssize_t firmware_data_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
    printk("%s\n",__func__);
    return FwDataCnt;
}

static ssize_t firmware_data_store(struct device *dev,
                                   struct device_attribute *attr, const char *buf, size_t size)
{

    //int i;
    TP_DEBUG("***FwDataCnt = %d ***\n", FwDataCnt);
    //for (i = 0; i < 1024; i++)
    // {
    memcpy(temp[FwDataCnt], buf, 1024);
    // }
    FwDataCnt++;
    return size;
}
static DEVICE_ATTR(data, 0664, firmware_data_show, firmware_data_store);


//++++++++++++++++++++++++++++++

#ifdef PROC_FIRMWARE_UPDATE
static int proc_version_read(struct file *file, const char *buffer, unsigned long count, void *data)//(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int cnt= 0;
    char page[50];
    cnt=firmware_version_show(NULL,NULL, page);
    copy_to_user(buffer, page, cnt);
    return cnt;
}

static int proc_version_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
    firmware_version_store(NULL, NULL, NULL, 0);
    return count;
}

static int proc_update_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
    count = (unsigned long)firmware_update_store(NULL, NULL, NULL, (size_t)count);
    return count;
}

static int proc_data_read(struct file *file, const char *buffer, unsigned long count, void *data)//(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int cnt= 0;

    return count;
}

static int proc_data_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
    firmware_data_store(NULL, NULL, buffer, 0);
    return count;
}
#define CTP_AUTHORITY_PROC 0777


static const struct file_operations proc_version_fops =
{
    .owner = THIS_MODULE,
    .read = proc_version_read,
    .write = proc_version_write,
};

static const struct file_operations proc_data_fops =
{
    .owner = THIS_MODULE,
    .read = proc_data_read,
    .write = proc_data_write,
};

static const struct file_operations proc_update_fops =
{
    .owner = THIS_MODULE,
    .write = proc_update_write,
};

static void _msg_create_file_for_fwUpdate_proc(void)
{
    struct proc_dir_entry *msg_class_proc = NULL;
    struct proc_dir_entry *msg_msg20xx_proc = NULL;
    struct proc_dir_entry *msg_device_proc = NULL;
    struct proc_dir_entry *msg_version_proc = NULL;
    struct proc_dir_entry *msg_update_proc = NULL;
    struct proc_dir_entry *msg_data_proc = NULL;

    msg_class_proc = proc_mkdir("class", NULL);
    msg_msg20xx_proc = proc_mkdir("ms-touchscreen-msg20xx",msg_class_proc);
    msg_device_proc = proc_mkdir("device",msg_msg20xx_proc);

    msg_version_proc = proc_create_data("version", CTP_AUTHORITY_PROC, msg_device_proc,&proc_version_fops, NULL);
    if (msg_version_proc == NULL)
    {
        printk(KERN_ERR "create_proc_entry msg_version_proc failed\n");
    }
    else
    {
        printk(KERN_INFO "create_proc_entry msg_version_proc success\n");
    }
    msg_data_proc = proc_create_data("data", CTP_AUTHORITY_PROC, msg_device_proc,&proc_data_fops, NULL);
    if (msg_data_proc == NULL)
    {
        printk(KERN_ERR "create_proc_entry msg_data_proc failed\n");
    }
    else
    {
        printk(KERN_INFO "create_proc_entry msg_data_proc success\n");
    }
    msg_update_proc = proc_create_data("update", CTP_AUTHORITY_PROC, msg_device_proc,&proc_update_fops, NULL);
    if (msg_update_proc == NULL)
    {
        printk(KERN_ERR "create_proc_entry msg_update_proc failed\n");
    }
    else
    {
        printk(KERN_INFO "create_proc_entry msg_update_proc success\n");
    }
}
#endif

static int _CheckFwIntegrity(void)
{
    int ret=0;
    int cnt = 0 ;
    u8  info_vendorId=0;
    u8  main_vendorId=0;
    u8  info_vendorId_ok=1;
    u8  main_vendorId_ok=1;
    u32 cal_crc32, bin_conf_crc32;
    u16  reg_data=0;
    u32 i;
    u8  dbbus_tx_data[5]= {0};
    u8  dbbus_rx_data[4]= {0};

    _HalTscrHWReset();

    ret = get_fw_version();
    if (ret == MSG2133_ERROR)
    {
        printk(KERN_ERR "%s:get fw err!!!\n",__func__);
        return ret;
    }
    else if (ret == -2)
    {
      goto MsgReadErr ;
    }

    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 100 );

////////// CalMainCRC32 /////////////////////////////
    //Stop MCU
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0001 );

    // Stop Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    //cmd
    drvDB_WriteReg ( 0x3C, 0xE4, 0xDF4C );
    drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );
    // TP SW reset
    drvDB_WriteReg ( 0x1E, 0x04, 0x829F );

    //MCU run
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0000 );

    //polling 0x3CE4
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x9432 );

    // Cal CRC Main from TP
    cal_crc32 = drvDB_ReadReg ( 0x3C, 0x80 );
    cal_crc32 = ( cal_crc32 << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );

    TP_DEBUG("[21xxA]:Current main crc32=0x%x\n",cal_crc32);


////////// ReadBinConfig /////////////////////////////
    //Stop MCU
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0001 );

    // Stop Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    //cmd
    drvDB_WriteReg ( 0x3C, 0xE4, 0xA4AB );
    drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );

    // TP SW reset
    drvDB_WriteReg ( 0x1E, 0x04, 0x829F );

    //MCU run
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0000 );

    //polling 0x3CE4
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x5B58 );

    dbbus_tx_data[0] = 0x72;
    dbbus_tx_data[1] = 0x7F;
    dbbus_tx_data[2] = 0xFC;
    dbbus_tx_data[3] = 0x00;
    dbbus_tx_data[4] = 0x04;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , &dbbus_tx_data[0], 5 );
    // recive info data
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0], 4 );

    for(i=0; i<4; i++)
    {
        TP_DEBUG("[21xxA]:Bin Config dbbus_rx_data[%d]=%x\n",i,dbbus_rx_data[i]);
    }
    /**************************modify by sam 20131224**************************/
    bin_conf_crc32=dbbus_rx_data[0];
    bin_conf_crc32=(bin_conf_crc32<<8)|dbbus_rx_data[1];
    bin_conf_crc32=(bin_conf_crc32<<8)|dbbus_rx_data[2];
    bin_conf_crc32=(bin_conf_crc32<<8)|dbbus_rx_data[3];
    /**************************end of modify************************************/
    TP_DEBUG("[21xxA]:crc32 from bin config=%x\n",bin_conf_crc32);

////////// GetVendorID /////////////////////////////
MsgReadErr :
    // i = 0x8300; // i = 0x830D;

    dbbus_tx_data[0] = 0x72;
    dbbus_tx_data[1] = 0x83;
    dbbus_tx_data[2] = 0x00;  //0x0D
    dbbus_tx_data[3] = 0x00;
    dbbus_tx_data[4] = 0x04;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , &dbbus_tx_data[0], 5 );
    // recive info data
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0], 4 );
    info_vendorId_ok=1;
    for(i=0; i<4; i++)
    {
        TP_DEBUG("[21xxA]:Vendor id dbbus_rx_data[%d]=0x%x,%d\n",i,dbbus_rx_data[i],(dbbus_rx_data[i]-0x30));
        if (((dbbus_rx_data[i] < 0x30) ||(dbbus_rx_data[i] > 0x39)) && (i<3)) info_vendorId_ok=0;  //vendor id is error
    }
    if (info_vendorId_ok)
    {
        info_vendorId=(dbbus_rx_data[0]-0x30)*100+(dbbus_rx_data[1]-0x30)*10+(dbbus_rx_data[2]-0x30);
        sprintf(msg213x_info_vendor_ID,"%d",info_vendorId);
    }
    else
    {
        sprintf(msg213x_info_vendor_ID,"Null");
    }

    //if(info_vendorId_ok == 0)
    {
        //i = 0x7F55;

        dbbus_tx_data[0] = 0x72;
        dbbus_tx_data[1] = 0x7F;
        dbbus_tx_data[2] = 0x55;
        dbbus_tx_data[3] = 0x00;
        dbbus_tx_data[4] = 0x04;
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , &dbbus_tx_data[0], 5 );
        // recive info data
        HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0], 4 );
        main_vendorId_ok=1;
        for(i=0; i<4; i++)
        {
            TP_DEBUG("[21xxA]:Vendor id dbbus_rx_data[%d]=0x%x,%d\n",i,dbbus_rx_data[i],(dbbus_rx_data[i]-0x30));
            if (((dbbus_rx_data[i] < 0x30) ||(dbbus_rx_data[i] > 0x39)) && (i<3)) main_vendorId_ok=0;  //vendor id is error
        }

    }

    if (main_vendorId_ok)
    {
        main_vendorId=(dbbus_rx_data[0]-0x30)*100+(dbbus_rx_data[1]-0x30)*10+(dbbus_rx_data[2]-0x30);
        sprintf(msg213x_main_vendor_ID,"%d",main_vendorId);
    }
    else
    {
        sprintf(msg213x_main_vendor_ID,"Null");
    }

    if(info_vendorId_ok)
    {
        msg2133_dev->vendor_id =info_vendorId;
    }
    else if(main_vendorId_ok)
    {
        msg2133_dev->vendor_id =main_vendorId;
    }
    else
    {
        msg2133_dev->vendor_id = 0;
    }

    if ((msg2133_dev->major_version ==0) && (msg2133_dev->minor_version==0))
    {
        msg2133_dev->major_version = msg2133_dev->vendor_id;
    }
    if (cal_crc32!=bin_conf_crc32) msg2133_dev->minor_version=0;

    _HalTscrHWReset();

    return (0);
}

void Msg2133_AutoUpdata( EMEM_TYPE_t emem_type )
{
    u16 update_bin_major=0;
    u16 update_bin_minor=0;
    u16 update_vendor_id=0;
    int i;

    if((msg2133_dev->panel_parm->panel_fw) && (msg2133_dev->panel_parm->panel_fw_size==33*1024))
    {
        {
            update_bin_major = msg2133_dev->panel_parm->panel_fw[0x7f4f]<<8|msg2133_dev->panel_parm->panel_fw[0x7f4e];
            update_bin_minor = msg2133_dev->panel_parm->panel_fw[0x7f51]<<8|msg2133_dev->panel_parm->panel_fw[0x7f50];
            update_vendor_id =(msg2133_dev->panel_parm->panel_fw[0x7f55]-0x30)*100
                              +(msg2133_dev->panel_parm->panel_fw[0x7f56]-0x30)*10
                              +(msg2133_dev->panel_parm->panel_fw[0x7f57]-0x30);

        }
        /*
        printk("vendor_id=%d,update_vendor_id=%d,update_bin_major=%d, 
               update_bin_minor=%d,major_version=%d,minor_version=%d\n", 
               msg2133_dev->vendor_id,update_vendor_id,update_bin_major, 
               update_bin_minor,msg2133_dev->major_version,msg2133_dev->minor_version);
        */

        if((update_bin_minor > msg2133_dev->minor_version) && 
           (msg2133_dev->vendor_id == update_vendor_id )    && 
           (msg2133_dev->major_version == update_bin_major ))
        {
            printk("TP FW WILL UPGRADE TO V%x.%x\n",update_bin_major,update_bin_minor);
            for (i = 0; i < 33; i++)
            {
                firmware_data_store(NULL, NULL, &(msg2133_dev->panel_parm->panel_fw[i*1024]), 0);
            }
            firmware_auto_update(emem_type);
            _HalTscrHWReset();
        }
        else  //normal boot up process
        {
            printk("ENTER NORMAL BOOT-UP PROCESS!\n");
        }
    }
    else
    {
        printk("NOT FIND FW!\n");
    }
}

void msg2133_updata_version(void)
{
    get_fw_version();
    printk("%s: version = %d.%d\n",__func__, msg2133_dev->major_version,msg2133_dev->minor_version);
    sprintf(msg213x_version,"Ver %d.%02d",msg2133_dev->major_version,msg2133_dev->minor_version);
}


static int __init msg2133_touch_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i,ret=0,err = 0;

    printk(KERN_ERR "[kernel]:[%s]------------->\n",  __func__);
    //+add by hzb
    if(CHECK_THIS_DEV_DEBUG_AREADY_EXIT()==0)
    {
        return -EIO;
    }
    //-add by hzb
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        err = -ENODEV;
        return err ;
    }
    msg2133_dev = kzalloc(sizeof(struct msg2133_touch), GFP_KERNEL);
    if (msg2133_dev == NULL)
        return -ENOMEM;

    msg2133_dev->i2c = client;
    msg2133_dev->data = (struct touchpanel_platform_data *)client->dev.platform_data;
    if (msg2133_dev->data == NULL)
    {
        printk("%s(), error\n", __func__);
        goto err_free_mem;
    }
    if (msg2133_dev->data->irq_gpio)
    {
        err = gpio_request(msg2133_dev->data->irq_gpio, "ts_irq_pin");
        if (err < 0)
        {
            dev_err(&client->dev, "%s:failed to request irq_gpio.\n",
                    __func__);
            ret = err;
            goto err_free_mem;
        }
        client->irq = gpio_to_irq(msg2133_dev->data->irq_gpio);
    }
    msg2133_dev->irq = client->irq;
	//sprd_i2c_ctl_chg_clk(client->adapter->nr, 400000); //wanggang +
#ifdef CONFIG_HAS_EARLYSUSPEND
    msg2133_dev->early_suspend = msg2133_touch_early_suspend_desc ;
    register_early_suspend( &msg2133_dev->early_suspend );
#endif
    if (msg2133_dev->data->reset_gpio)
    {
        err = gpio_request(msg2133_dev->data->reset_gpio, "msg2133 reset");
        if (err < 0)
        {
            dev_err(&client->dev, "%s:failed to set gpio reset.\n",
                    __func__);
            ret = err;
            goto exit_request_reset;
        }
    }
    if(msg2133_dev->data->power_ic)
    {
        msg2133_dev->data->power_ic(&client->dev,1);
        msleep (10);
    }

    ret=_CheckFwIntegrity();
    if(MSG2133_ERROR == ret)
    {
        printk("get msg2133 error!\n");
        goto err_get_fw_version;
    }

    for(i=0; i<msg2133_dev->data->panel_parm_num; i++)
    {
        if ((msg2133_dev->data->panel_parm[i].vendor_id == msg2133_dev->vendor_id )
                && (msg2133_dev->major_version==msg2133_dev->data->panel_parm[i].major_version ))
        {
            msg2133_dev->panel_parm=&(msg2133_dev->data->panel_parm[i]);
            msg2133_dev->vendor_name=msg2133_dev->data->panel_parm[i].vendor_name;
            Msg2133_AutoUpdata(EMEM_MAIN);
            printk("[kernel] [%s %d] [vendor_id 0x%x major_version = 0x%x].....\n",__func__ ,__LINE__,
                                    msg2133_dev->vendor_id,msg2133_dev->major_version) ;
            break;
        }
    }
    if (i>=msg2133_dev->data->panel_parm_num)
    {
        msg2133_dev->panel_parm=&(msg2133_dev->data->panel_parm[0]);
    }

    printk("[kernel] [%s %d] [vendor_id 0x%x major_version = 0x%x].....\n",__func__ ,__LINE__,
                                    msg2133_dev->vendor_id,msg2133_dev->major_version) ;
    if (msg2133_dev->vendor_id == 0)  
    {
        msg2133_dev->vendor_id=msg2133_dev->panel_parm->vendor_id;
        msg2133_dev->major_version=msg2133_dev->panel_parm->major_version ;
        printk("[kernel] [%s %d] [vendor_id 0x%x major_version = 0x%x].....\n",__func__ ,__LINE__,
                                    msg2133_dev->vendor_id,msg2133_dev->major_version) ;
        Msg2133_AutoUpdata(EMEM_ALL);
    }
    msg2133_updata_version();

    msg2133_dev->idev = input_allocate_device();
    if (msg2133_dev->idev == NULL)
    {
        printk( "%s: failed to allocate input dev\n",  __FUNCTION__);
        ret = -ENOMEM;
        goto err_input_allocate_device;
    }
    msg2133_dev->idev->name = "msg2133";
    msg2133_dev->idev->id.bustype = BUS_I2C;
    msg2133_dev->idev->dev.parent = &client->dev;
    //	msg2133_dev->idev->phys =  "msg2133/input1";
    //msg2133_dev->idev->open = msg2133_touch_open;
    //msg2133_dev->idev->close = msg2133_touch_close;

    set_bit(EV_KEY, msg2133_dev->idev->evbit);
    set_bit(EV_ABS, msg2133_dev->idev->evbit);
    set_bit(BTN_TOUCH, msg2133_dev->idev->keybit);

    input_set_abs_params(msg2133_dev->idev,
                         ABS_MT_POSITION_X, 0, msg2133_dev->panel_parm->x_max_res-1, 0, 0);
    input_set_abs_params(msg2133_dev->idev,
                         ABS_MT_POSITION_Y, 0, msg2133_dev->panel_parm->y_max_res-1, 0, 0);
    input_set_abs_params(msg2133_dev->idev, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);  //modify by hzb
    if (1)
    {
        input_mt_init_slots(msg2133_dev->idev, 2,(INPUT_MT_POINTER | INPUT_MT_DIRECT));
    }
    else
    {
        input_set_abs_params(msg2133_dev->idev, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
        input_set_abs_params(msg2133_dev->idev,ABS_MT_TRACKING_ID, 0, 2, 0, 0);
    }
    if(msg2133_dev->panel_parm->virtual_key)
    {
        for(i=0; i<msg2133_dev->panel_parm->virtual_key_num; i++)
        {
            if ( msg2133_dev->panel_parm->virtual_key[i].key_code )
            {

                set_bit(msg2133_dev->panel_parm->virtual_key[i].key_code, msg2133_dev->idev->evbit);
                printk(KERN_INFO "%s:Set key %d\n",__func__,msg2133_dev->panel_parm->virtual_key[i].key_code);
            }
            else
            {
                break;
            }
        }
    }


    if ( msg2133_dev->panel_parm->virtual_key && msg2133_dev->panel_parm->virtual_key_num)
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

    ret = input_register_device(msg2133_dev->idev);
    if (ret)
    {
        printk( "%s: unabled to register input device, ret = %d\n",		       __FUNCTION__, ret);
        goto err_input_register_device;
    }

    firmware_class = class_create(THIS_MODULE, "ms-touchscreen-msg20xx");
    if (IS_ERR(firmware_class))
        pr_err("Failed to create class(firmware)!\n");

    firmware_cmd_dev = device_create(firmware_class,
                                     NULL, 0, NULL, "device");
    if (IS_ERR(firmware_cmd_dev))
        pr_err("Failed to create device(firmware_cmd_dev)!\n");

    // version
    if (device_create_file(firmware_cmd_dev, &dev_attr_version) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_version.attr.name);
    // update
    if (device_create_file(firmware_cmd_dev, &dev_attr_update) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_update.attr.name);
    // data
    if (device_create_file(firmware_cmd_dev, &dev_attr_data) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_data.attr.name);

    if (device_create_file(firmware_cmd_dev, &dev_attr_clear) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_clear.attr.name);

    dev_set_drvdata(firmware_cmd_dev, NULL);

#ifdef PROC_FIRMWARE_UPDATE
    _msg_create_file_for_fwUpdate_proc();
#endif

    printk("%s(), request irq:%d\n", __func__,  msg2133_dev->irq);
    msg2133_dev->is_run =0;
    ret = request_irq(msg2133_dev->irq, msg2133_touch_irq_handler,  IRQF_ONESHOT | IRQF_TRIGGER_FALLING,   "MSG2133",  msg2133_dev);
    if (ret)
    {
        printk("%s(), request irq error#######################\n",  __func__);
        goto err_request_irq;
    }
    disable_irq(msg2133_dev->irq);

    msg2133_dev->msg2133_wq = alloc_workqueue("msg2133_wq", WQ_HIGHPRI
                              | WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
    if (msg2133_dev->msg2133_wq==NULL)
    {
        printk("%s: alloc_workqueue error!!!!\n",  __func__);
        goto err_alloc_workqueue;
    }
    INIT_WORK(&msg2133_dev->work, msg2133_touch_work);

    msg2133_dev->is_run =1;

#ifndef CONFIG_HAS_EARLYSUSPEND
#ifdef CONFIG_PM_RUNTIME
    if((msg2133_dev->data->power_ic)&&(msg2133_dev->data->power_on_when_sleep==0))
    {
        msg2133_dev->data->power_ic(&client->dev,0);
    }
    pm_runtime_enable(&client->dev);
    pm_runtime_forbid(&client->dev);
#endif
#else
    enable_irq(msg2133_dev->irq);
#endif
    if (msg2133_dev->vendor_name)
        sprintf(msg213x_vendor_name,"%s",msg2133_dev->vendor_name);
    REGISTER_AND_INIT_ONTIM_DEBUG_FOR_THIS_DEV();

    if(!tp_probe_ok)//add by liuwei
        tp_probe_ok = 1;//add by liuwei
    return MSG2133_OK;

err_alloc_workqueue:
    free_irq(msg2133_dev->irq, msg2133_dev);
err_request_irq:
    input_unregister_device(msg2133_dev->idev);
err_input_register_device:
    input_free_device(msg2133_dev->idev);
err_input_allocate_device:
err_get_fw_version:
    if(msg2133_dev->data->power_ic)
    {
        msg2133_dev->data->power_ic(&client->dev,0);
    }
    if (msg2133_dev->data->reset_gpio)
        gpio_free(msg2133_dev->data->reset_gpio);
exit_request_reset:
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend( &msg2133_dev->early_suspend );
#endif

    if (msg2133_dev->data->irq_gpio)
        gpio_free(msg2133_dev->data->irq_gpio);
err_free_mem:
    kfree(msg2133_dev);
    msg2133_dev = NULL ;
    return ret;


}

static int msg2133_touch_remove(struct i2c_client *client)
{
    if (msg2133_dev->data->irq_gpio)
    {
        free_irq(msg2133_dev->irq, msg2133_dev);
        gpio_free(msg2133_dev->data->irq_gpio);
    }

#ifndef CONFIG_HAS_EARLYSUSPEND
#ifdef CONFIG_PM_RUNTIME
    pm_runtime_disable(&client->dev);
#endif
#endif

    input_unregister_device(msg2133_dev->idev);
    input_free_device(msg2133_dev->idev);

    if(msg2133_dev->data->power_ic)
    {
        msg2133_dev->data->power_ic(&client->dev,0);
    }
    if (msg2133_dev->data->reset_gpio)
        gpio_free(msg2133_dev->data->reset_gpio);

    kfree(msg2133_dev);
    return 0;
}

static const struct i2c_device_id msg2133_touch_id[] =
{
    {"MSG2XXX", 0},	{}
};

MODULE_DEVICE_TABLE(i2c, msg2133_touch_id);

static struct i2c_driver  msg2133_touch_driver =
{
    .probe = msg2133_touch_probe,
    .remove =  msg2133_touch_remove,
    .id_table = msg2133_touch_id,
    .driver =
    {
        .name = "MSG2XXX",
        .owner = THIS_MODULE,
#ifndef CONFIG_HAS_EARLYSUSPEND
#ifdef CONFIG_PM_RUNTIME
        .pm = &msg2133_ts_pmops,
#endif
#endif
    },
};

static int __init msg2133_touch_init(void)
{
    int ret;
    ret = i2c_add_driver(&msg2133_touch_driver);
    if (ret)
    {
        printk(KERN_ERR "Adding msg2133 driver failed "
               "(errno = %d)\n", ret);
    }
    else
    {
        printk("Successfully added driver %s\n",
               msg2133_touch_driver.driver.name);
    }
    return ret;
}
static void __exit msg2133_touch_exit(void)
{
    i2c_del_driver(&msg2133_touch_driver);
}

late_initcall(msg2133_touch_init);
module_exit(msg2133_touch_exit);
MODULE_DESCRIPTION("msg2133 touch Driver");
MODULE_LICENSE("GPL");

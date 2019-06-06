
/* 
 * drivers/input/touchscreen/gslX680.c
 *
 * Sileadinc gslX680 TouchScreen driver. 
 *
 * Copyright (c) 2012  Sileadinc
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
 * VERSION      	DATE			AUTHOR
 *   1.0		 2012-04-18		   leweihua
 *
 * note: only support mulititouch	Wenfs 2010-10-01
 */



#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/byteorder/generic.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/firmware.h>
//#include <mach/ldo.h>
#include <mach/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <linux/firmware.h>
#include <linux/platform_device.h>

#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <soc/sprd/regulator.h>

#ifdef CONFIG_I2C_SPRD
#include <soc/sprd/i2c-sprd.h>
#endif


#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/wakelock.h>
#include <linux/ioctl.h>
#include <linux/i2c.h>
#include <soc/sprd/i2c-sprd.h>

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <mach/gpio.h>
#include <linux/spinlock.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <asm/unistd.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/earlysuspend.h>
#include <linux/input/mt.h>
#include "sc_gslX680.h"
#include <soc/sprd/adc.h>

#define GSL_DEBUG
//#define GSL_MONITOR		/*Simon delete*/

#define MAX_FINGERS		10
#define MAX_CONTACTS	10
#define DMA_TRANS_LEN	0x20
#define GSL_PAGE_REG	0xf0

#define PRESS_MAX    		255
#define GSLX680_TS_NAME	"gslx680_ts"
#define GSLX680_TS_ADDR	0x40
#define FILTER_POINT
#ifdef FILTER_POINT
#define FILTER_MAX	9
#endif

#define TPD_PROC_DEBUG	/*Simon delete*/
#ifdef TPD_PROC_DEBUG
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
static struct proc_dir_entry *gsl_config_proc = NULL;
#define GSL_CONFIG_PROC_FILE "gsl_config"
#define CONFIG_LEN 31
static char gsl_read[CONFIG_LEN];
static u8 gsl_data_proc[8] = {0};
static u8 gsl_proc_flag = 0;
#endif

#define GSL_TIMER
#ifdef GSL_TIMER
#define GSL_TIMER_CHECK_CIRCLE        300
static struct delayed_work gsl_timer_check_work;
static struct workqueue_struct *gsl_timer_workqueue = NULL;
static char int_1st[4];
static char int_2nd[4];
static volatile char  i2c_lock_flag = 0;
static volatile char  err_count = 0;
#endif


//#define GSL_GESTURE
#ifdef GSL_GESTURE
#define GSL_IRQ_FALLING		1
#define GSL_IRQ_RISING		2
#define GSL_IRQ_LOW		3
#define GSL_IRQ_HIGH		4
static void gsl_irq_mode_change(struct i2c_client *client,u32 flag);
typedef enum{
	GE_DISABLE = 0,
	GE_ENABLE = 1,
	GE_WAKEUP = 2,
	GE_NOWORK = 3,
}GE_T;
static GE_T gsl_gesture_status = GE_DISABLE;
static unsigned int gsl_gesture_flag = 1;
#define GESTURE_SWITCH_FILE 		"/data/data/com.android.settings/shared_prefs/gesture_open.xml"  //总开关文件,获取第一个value的值,为1开,为0关
static char gsl_gesture_c = 0;
extern void gsl_FunIICRead(unsigned int (*fun) (unsigned int *,unsigned int,unsigned int));
extern int gsl_obtain_gesture(void);
static struct wake_lock gsl_wake_lock;
static unsigned int gsl_read_oneframe_data(unsigned int *data,
				unsigned int addr,unsigned int len);
#endif  //fan

static struct mutex gsl_i2c_lock;

struct gslX680_ts_data {
	struct input_dev	*input_dev;
	struct i2c_client	*client;
	u8 touch_data[44];	
	struct work_struct 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
	struct early_suspend	early_suspend;
	struct gslx680_ts_platform_data	*platform_data;
	struct gsl_touch_info		*cinfo;
};

static struct gslX680_ts_data *g_slX680_ts;
struct gslX680_ts_data *gslX680_ts;


//#define USE_TP_PSENSOR
#ifdef USE_TP_PSENSOR
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
//#include "tp_psensor.h"
#define PS_PHONE_STATE_FILE    "/data/data/com.android.phone/cache/state.txt"  //"/productinfo/state.txt" 
#define TP_PS_DEVICE		"ltr_558als"
#define TP_PS_INPUT_DEV			"alps_pxy"
#define FOCALTECH_IOCTL_MAGIC			0x1C
#define FOCALTECH_IOCTL_GET_PFLAG		_IOR(FOCALTECH_IOCTL_MAGIC, 1, int)
#define FOCALTECH_IOCTL_GET_LFLAG		_IOR(FOCALTECH_IOCTL_MAGIC, 2, int)
#define FOCALTECH_IOCTL_SET_PFLAG		_IOW(FOCALTECH_IOCTL_MAGIC, 3, int)
#define FOCALTECH_IOCTL_SET_LFLAG		_IOW(FOCALTECH_IOCTL_MAGIC, 4, int)
static u8 tpd_proximity_detect 		= 1;//0-->close ; 1--> far away
static struct input_dev *gsl_pls_input_dev;
static int gsl_ps_suspend_flag = 1;
//#define PS_DEBUG
#ifdef PS_DEBUG
#define PS_DBG(format, ...)	\
		printk(KERN_INFO "TP_PSENSOR " format "\n", ## __VA_ARGS__)
#else
#define PS_DBG(format, ...)
#endif 

static int tp_ps_opened = 0;
static atomic_t ps_flag;
//static tp_ps_t *tp_ps = 0;
static int ps_en = 0;
u8 gsl_psensor_data[8] = {0};

#endif

#ifdef GSL_DEBUG 
#define print_info(fmt, args...)   \
        do{                              \
                printk(fmt, ##args);     \
        }while(0)
#else
#define print_info(fmt, args...)   //
#endif

struct sprd_i2c_setup_data {
	unsigned i2c_bus;  //the same number as i2c->adap.nr in adapter probe function
	unsigned short i2c_address;
	int irq;
	char type[I2C_NAME_SIZE];
};

//static int sprd_3rdparty_gpio_tp_pwr;
//static int sprd_3rdparty_gpio_tp_rst;
//static int sprd_3rdparty_gpio_tp_irq;
static struct regulator *reg_vdd;
#define sprd_3rdparty_gpio_tp_rst	53
#define sprd_3rdparty_gpio_tp_irq	52

static u32 id_sign[MAX_CONTACTS+1] = {0};
static u8 id_state_flag[MAX_CONTACTS+1] = {0};
static u8 id_state_old_flag[MAX_CONTACTS+1] = {0};
static u16 x_old[MAX_CONTACTS+1] = {0};
static u16 y_old[MAX_CONTACTS+1] = {0};
static u16 x_new = 0;
static u16 y_new = 0;

static struct i2c_client *this_client = NULL;
static struct sprd_i2c_setup_data gslX680_ts_setup={0, GSLX680_TS_ADDR, 0, GSLX680_TS_NAME};

struct gslx680_ts_platform_data{
	int irq_gpio_number;
	int reset_gpio_number;
	const char *vdd_name;
	int virtualkeys[12];
	int TP_MAX_X;
	int TP_MAX_Y;
};

//#define HAVE_TOUCH_KEY
#define TOUCH_VIRTUAL_KEYS

#ifdef HAVE_TOUCH_KEY
static u16 key = 0;
static int key_state_flag = 0;
struct key_data {
	u16 key;
	u16 x_min;
	u16 x_max;
	u16 y_min;
	u16 y_max;	
};

const u16 key_array[]={
						KEY_MENU,
						KEY_HOME,
						KEY_BACK,
					}; 
#define MAX_KEY_NUM     (sizeof(key_array)/sizeof(key_array[0]))

struct key_data gsl_key_data[MAX_KEY_NUM] = {
	{KEY_MENU, 80, 120, 980, 1020},
	{KEY_HOME, 180, 220, 980, 1020},	
	{KEY_BACK, 380, 420, 980, 1020},
};
#endif

#if 0//def CONFIG_HAS_EARLYSUSPEND
static void gslX680_ts_suspend(struct early_suspend *handler);
static void gslX680_ts_resume(struct early_suspend *handler);
#endif

#ifdef TOUCH_VIRTUAL_KEYS
//#define PIXCIR_KEY_HOME	172
//#define PIXCIR_KEY_MENU	139
//#define PIXCIR_KEY_BACK	       158


static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
	 __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":100:1000:30:50"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":200:1000:30:30"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":400:1000:30:30" 
	);
}


static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.gslx680_ts",
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

static void pixcir_ts_virtual_keys_init(void)
{
    int ret;
    struct kobject *properties_kobj;	
	
    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (properties_kobj)
        ret = sysfs_create_group(properties_kobj,
                     &properties_attr_group);
    if (!properties_kobj || ret)
        pr_err("failed to create board_properties\n");    
}

#endif

static inline u16 join_bytes(u8 a, u8 b)
{
	u16 ab = 0;
	ab = ab | a;
	ab = ab << 8 | b;
	return ab;
}


static int gsl_ts_write(struct i2c_client *client, u8 addr, u8 *pdata, int datalen)
{
	int ret = 0;
	u8 tmp_buf[128];
	unsigned int bytelen = 0;
	if (datalen > 125)
	{
		////printk("%s too big datalen = %d!\n", __func__, datalen);
		return -1;
	}
	
	tmp_buf[0] = addr;
	bytelen++;
	
	if (datalen != 0 && pdata != NULL)
	{
		memcpy(&tmp_buf[bytelen], pdata, datalen);
		bytelen += datalen;
	}
	mutex_lock(&gsl_i2c_lock);	
	ret = i2c_master_send(client, tmp_buf, bytelen);
	mutex_unlock(&gsl_i2c_lock);
	return ret;
}

static int gsl_ts_read(struct i2c_client *client, u8 addr, u8 *pdata, unsigned int datalen)
{
	int ret = 0;

	if (datalen > 126)
	{
		//printk("%s too big datalen = %d!\n", __func__, datalen);
		return -1;
	}
	ret = gsl_ts_write(client, addr, NULL, 0);
	if (ret < 0)
	{
		//printk("%s set data address fail!\n", __func__);
		return ret;
	}
	mutex_lock(&gsl_i2c_lock);	
	ret = i2c_master_recv(client, pdata, datalen);
	mutex_unlock(&gsl_i2c_lock);
	return ret;
}


static int gsl_read_interface(struct i2c_client *client, u8 reg, u8 *buf, unsigned int num)
{
	struct i2c_msg xfer_msg[2];
	int err;
	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len =  1;
	xfer_msg[0].flags = 0;//client->flags & I2C_M_TEN;
	xfer_msg[0].buf = &reg;
	
	xfer_msg[1].addr = client->addr;
	xfer_msg[1].len = num;
	xfer_msg[1].flags = I2C_M_RD;//client->flags & I2C_M_TEN;
	xfer_msg[1].buf = buf;
	mutex_lock(&gsl_i2c_lock);
	err= i2c_transfer(client->adapter, xfer_msg, 2);
	mutex_unlock(&gsl_i2c_lock);

	return err;
}

static int gsl_write_interface(struct i2c_client *client,
        const u8 reg, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[1] = {0};
	int err;
	u8 tmp_buf[num + 1];

	tmp_buf[0] = reg;
	memcpy(tmp_buf + 1, buf, num);

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = num + 1;
	xfer_msg[0].flags = 0;//client->flags & I2C_M_TEN;
	xfer_msg[0].buf = tmp_buf;
	//xfer_msg[0].timing = 400;//I2C_TRANS_SPEED;
	mutex_lock(&gsl_i2c_lock);

	err = i2c_transfer(client->adapter, xfer_msg, 1);
	mutex_unlock(&gsl_i2c_lock);


	return err;
}

static __inline__ void fw2buf(u8 *buf, const u32 *fw)
{
	u32 *u32_buf = (int *)buf;
	*u32_buf = *fw;
}

static void gsl_load_fw(struct i2c_client *client,const struct fw_data *GSL_DOWNLOAD_DATA,int data_len)
{
	u8 buf[4] = {0};
	//u8 send_flag = 1;
	u8 addr=0;
	u32 source_line = 0;
	u32 source_len = data_len;//ARRAY_SIZE(GSL_DOWNLOAD_DATA);

	//printk("=============gsl_load_fw start==============\n");

	for (source_line = 0; source_line < source_len; source_line++) 
	{
		/* init page trans, set the page val */
    	addr = (u8)GSL_DOWNLOAD_DATA[source_line].offset;
		memcpy(buf,&GSL_DOWNLOAD_DATA[source_line].val,4);
    	gsl_ts_write(client, addr, buf, 4);
		//gsl_write_interface(client, addr, buf, 4);		
	}
}

static int test_i2c(struct i2c_client *client)
{
	u8 read_buf = 0;
	u8 buf[4] = {0};
	u8 write_buf = 0x12;
	int ret, rc = 1,i;

	gsl_ts_read( client, 0xfc, buf, 4);
	//printk("gsl::test_i2c:: 0xfc = 0x%02x%02x%02x%02x\n",buf[3],buf[2],buf[1],buf[0]);
		
	ret = gsl_ts_read( client, 0xf0, &read_buf, sizeof(read_buf) );
	if  (ret  < 0)  
    		rc --;
	else
		//printk("I read reg 0xf0 is %x\n", read_buf);
	
	msleep(2);
	ret = gsl_ts_write(client, 0xf0, &write_buf, sizeof(write_buf));
	if(ret  >=  0 )
		//printk("I write reg 0xf0 0x12\n");
	
	msleep(2);
	ret = gsl_ts_read( client, 0xf0, &read_buf, sizeof(read_buf) );
	if(ret <  0 )
		rc --;
	else
		//printk("I read reg 0xf0 is 0x%x\n", read_buf);

	return rc;
}

static void startup_chip(struct i2c_client *client)
{
	u8 tmp = 0x00;
	
#ifdef GSL_NOID_VERSION
		#ifdef GSL_GPIO_TP
		gsl_DataInit(gsl_cfg_table[gsl_cfg_index].data_id);	
		#else
		gsl_DataInit(gsl_config_data_id);
		#endif
#endif
	gsl_ts_write(client, 0xe0, &tmp, 1);
	msleep(10);	
}

static void reset_chip(struct i2c_client *client)
{
	u8 tmp = 0x88;
	u8 buf[4] = {0x00};
	
	gsl_ts_write(client, 0xe0, &tmp, sizeof(tmp));
	msleep(20);
	tmp = 0x04;
	gsl_ts_write(client, 0xe4, &tmp, sizeof(tmp));
	msleep(10);
	gsl_ts_write(client, 0xbc, buf, sizeof(buf));
	msleep(10);
}

static void clr_reg(struct i2c_client *client)
{
	u8 write_buf[4]	= {0};

	write_buf[0] = 0x88;
	gsl_ts_write(client, 0xe0, &write_buf[0], 1); 	
	msleep(20);
	write_buf[0] = 0x03;
	gsl_ts_write(client, 0x80, &write_buf[0], 1); 	
	msleep(5);
	write_buf[0] = 0x04;
	gsl_ts_write(client, 0xe4, &write_buf[0], 1); 	
	msleep(5);
	write_buf[0] = 0x00;
	gsl_ts_write(client, 0xe0, &write_buf[0], 1); 	
	msleep(20);
}

int gsl2336_Check_TP_ID(void)
{
		int data[4] = {0};
		int res = 0;
		int rawdata = 0;
		int tp_vol = 0;
#define AUXADC_TP_VOLTAGE_CHANNEL 1
#ifdef AUXADC_TP_VOLTAGE_CHANNEL
		extern int sci_adc_get_value(unsigned chan, int scale);
		rawdata = sci_adc_get_value(AUXADC_TP_VOLTAGE_CHANNEL,false);
		if(rawdata < 0)
		{ 
#ifdef BUILD_LK
				printf("[adc_uboot]: get data error\n");
#endif
				return 0;

		}
#endif
		//tp_vol = data[0]*1000+data[1]*10;
		//printk("wangcq327 --- data[0]:%d  data[1]:%d  tp_vol:%d\n",data[0],data[1],tp_vol);
		//printk("yedongyue ----------adc value= %d----------\n",rawdata);
		if(rawdata >2700 && rawdata < 3300){		
			return 0;//TP_VOL = 0;
		}
		else if(rawdata < 4400 && rawdata > 3800){
			return 1;//TP_VOL = 1;
		}
		else{
			return -1;//TP_VOL = -1;
		}
}

static void init_chip(struct i2c_client *client)
{
	int rc;
	int temp;
	struct gslx680_ts_platform_data *pdata = g_slX680_ts->platform_data;
	
	
	gpio_set_value(pdata->reset_gpio_number, 0);
	msleep(20); 	
	gpio_set_value(pdata->reset_gpio_number, 1);
	msleep(20); 		
	rc = test_i2c(client);
	if(rc < 0)
	{
		//printk("------gslX680 test_i2c error------\n");	
		return;
	}	
	clr_reg(client);
	reset_chip(client);
#ifdef GSL_GPIO_TP  
	temp = gsl_cfg_table[gsl_cfg_index].fw_size;
	gsl_load_fw(this_client,gsl_cfg_table[gsl_cfg_index].fw,
		temp);
#else
	temp = ARRAY_SIZE(GSLX680_FW);
	gsl_load_fw(client,GSLX680_FW,temp);
#endif 
	startup_chip(client);
}

static void check_mem_data(struct i2c_client *client)
{
	u8 read_buf[4]  = {0};
	
	msleep(30);
	gsl_ts_read(client,0xb0, read_buf, sizeof(read_buf));
	//printk("#########check mem read 0xb0 = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);	
	if (read_buf[3] != 0x5a || read_buf[2] != 0x5a || read_buf[1] != 0x5a || read_buf[0] != 0x5a)
	{
		//printk("#########check mem read 0xb0 = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		init_chip(client);
	}	
}

#ifdef TPD_PROC_DEBUG
#define GSL_APPLICATION
#ifdef GSL_APPLICATION
static int gsl_read_MorePage(struct i2c_client *client,u32 addr,u8 *buf,u32 num)
{
	int i;
	u8 tmp_buf[4] = {0};
	u8 tmp_addr;
	for(i=0;i<num/8;i++){
		tmp_buf[0]=(char)((addr+i*8)/0x80);
		tmp_buf[1]=(char)(((addr+i*8)/0x80)>>8);
		tmp_buf[2]=(char)(((addr+i*8)/0x80)>>16);
		tmp_buf[3]=(char)(((addr+i*8)/0x80)>>24);
		gsl_write_interface(client,0xf0,tmp_buf,4);
		tmp_addr = (char)((addr+i*8)%0x80);
		gsl_read_interface(client,tmp_addr,(buf+i*8),8);
	}
	if(i*8<num){
		tmp_buf[0]=(char)((addr+i*8)/0x80);
		tmp_buf[1]=(char)(((addr+i*8)/0x80)>>8);
		tmp_buf[2]=(char)(((addr+i*8)/0x80)>>16);
		tmp_buf[3]=(char)(((addr+i*8)/0x80)>>24);
		gsl_write_interface(client,0xf0,tmp_buf,4);
		tmp_addr = (char)((addr+i*8)%0x80);
		gsl_read_interface(client,tmp_addr,(buf+i*8),4);
	}
	return 0;
}
#endif
static int char_to_int(char ch)
{
	if(ch>='0' && ch<='9')
		return (ch-'0');
	else
		return (ch-'a'+10);
}

//static int gsl_config_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
static int gsl_config_read_proc(struct seq_file *m,void *v)
{
	char temp_data[5] = {0};
	//int i;
	unsigned int tmp=0;
	if('v'==gsl_read[0]&&'s'==gsl_read[1])
	{
#ifdef GSL_NOID_VERSION
		tmp=gsl_version_id();
#else
		tmp=0x20121215;
#endif
		seq_printf(m,"version:%x\n",tmp);
	}
	else if('r'==gsl_read[0]&&'e'==gsl_read[1])
	{
		if('i'==gsl_read[3])
		{
#ifdef GSL_NOID_VERSION
			tmp=(gsl_data_proc[5]<<8) | gsl_data_proc[4];
			seq_printf(m,"gsl_config_data_id[%d] = ",tmp);
			#ifdef GSL_GPIO_TP 
			if(tmp>=0&&tmp<gsl_cfg_table[gsl_cfg_index].data_size)
				seq_printf(m,"%d\n",gsl_cfg_table[gsl_cfg_index].data_id[tmp]);
			#else
			if(tmp>=0&&tmp<ARRAY_SIZE(gsl_config_data_id))
				seq_printf(m,"%d\n",gsl_config_data_id[tmp]); 	
			#endif
#endif
		}
		else
		{
			gsl_write_interface(this_client,0xf0,&gsl_data_proc[4],4);
			gsl_read_interface(this_client,gsl_data_proc[0],temp_data,4);
			seq_printf(m,"offset : {0x%02x,0x",gsl_data_proc[0]);
			seq_printf(m,"%02x",temp_data[3]);
			seq_printf(m,"%02x",temp_data[2]);
			seq_printf(m,"%02x",temp_data[1]);
			seq_printf(m,"%02x};\n",temp_data[0]);
		}
	}
#ifdef GSL_APPLICATION
	else if('a'==gsl_read[0]&&'p'==gsl_read[1]){
		char *buf;
		int temp1;
		tmp = (unsigned int)(((gsl_data_proc[2]<<8)|gsl_data_proc[1])&0xffff);
		buf=kzalloc(tmp,GFP_KERNEL);
		if(buf==NULL)
			return -1;
		if(3==gsl_data_proc[0]){
			gsl_read_interface(this_client,gsl_data_proc[3],buf,tmp);
			if(tmp < m->size){
				memcpy(m->buf,buf,tmp);
			}
		}else if(4==gsl_data_proc[0]){
			temp1=((gsl_data_proc[6]<<24)|(gsl_data_proc[5]<<16)|
				(gsl_data_proc[4]<<8)|gsl_data_proc[3]);
			gsl_read_MorePage(this_client,temp1,buf,tmp);
			if(tmp < m->size){
				memcpy(m->buf,buf,tmp);
			}
		}
		kfree(buf);
	}
#endif
	return 0;
}

static ssize_t  gsl_config_write_proc(struct file *file, const char __user  *buffer, size_t  count, loff_t *data)
{
	u8 buf[8] = {0};
	char temp_buf[CONFIG_LEN];
	char *path_buf;
	int tmp = 0;
	int tmp1 = 0;
	print_info("[tp-gsl][%s] \n",__func__);
	if(count > 512)
	{
		print_info("size not match [%d:%ld]\n", CONFIG_LEN, count);
        	return -EFAULT;
	}
	path_buf=kzalloc(count,GFP_KERNEL);
	if(!path_buf)
	{
		printk("alloc path_buf memory error \n");
		return -1;
	}
	if(copy_from_user(path_buf, buffer, count))
	{
		print_info("copy from user fail\n");
		goto exit_write_proc_out;
	}
	memcpy(temp_buf,path_buf,(count<CONFIG_LEN?count:CONFIG_LEN));
	print_info("[tp-gsl][%s][%s]\n",__func__,temp_buf);
#ifdef GSL_APPLICATION
	if('a'!=temp_buf[0]||'p'!=temp_buf[1]){
#endif
	buf[3]=char_to_int(temp_buf[14])<<4 | char_to_int(temp_buf[15]);	
	buf[2]=char_to_int(temp_buf[16])<<4 | char_to_int(temp_buf[17]);
	buf[1]=char_to_int(temp_buf[18])<<4 | char_to_int(temp_buf[19]);
	buf[0]=char_to_int(temp_buf[20])<<4 | char_to_int(temp_buf[21]);

	buf[7]=char_to_int(temp_buf[5])<<4 | char_to_int(temp_buf[6]);
	buf[6]=char_to_int(temp_buf[7])<<4 | char_to_int(temp_buf[8]);
	buf[5]=char_to_int(temp_buf[9])<<4 | char_to_int(temp_buf[10]);
	buf[4]=char_to_int(temp_buf[11])<<4 | char_to_int(temp_buf[12]);
#ifdef GSL_APPLICATION
	}
#endif
	if('v'==temp_buf[0]&& 's'==temp_buf[1])//version //vs
	{
		memcpy(gsl_read,temp_buf,4);
		printk("gsl version\n");
	}
	else if('s'==temp_buf[0]&& 't'==temp_buf[1])//start //st
	{
#ifdef GSL_TIMER
		cancel_delayed_work_sync(&gsl_timer_check_work);
#endif
		gsl_proc_flag = 1;
		reset_chip(this_client);
	}
	else if('e'==temp_buf[0]&&'n'==temp_buf[1])//end //en
	{
		msleep(20);
		reset_chip(this_client);
		startup_chip(this_client);
    #ifdef GSL_TIMER
	queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE);
    #endif	
		gsl_proc_flag = 0;
	}
	else if('r'==temp_buf[0]&&'e'==temp_buf[1])//read buf //
	{
		memcpy(gsl_read,temp_buf,4);
		memcpy(gsl_data_proc,buf,8);
	}
	else if('w'==temp_buf[0]&&'r'==temp_buf[1])//write buf
	{
		gsl_write_interface(this_client,buf[4],buf,4);
	}

#ifdef GSL_NOID_VERSION
	else if('i'==temp_buf[0]&&'d'==temp_buf[1])//write id config //
	{
		tmp1=(buf[7]<<24)|(buf[6]<<16)|(buf[5]<<8)|buf[4];
		tmp=(buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
		#ifdef GSL_GPIO_TP 
		if(tmp1>=0 && tmp1<gsl_cfg_table[gsl_cfg_index].data_size)
		{
			gsl_cfg_table[gsl_cfg_index].data_id[tmp1] = tmp;
		}
		#else
		if(tmp1>=0 && tmp1<ARRAY_SIZE(gsl_config_data_id))
		{
			gsl_config_data_id[tmp1] = tmp;
		}
		#endif
	}
#endif
#ifdef GSL_APPLICATION
	else if('a'==temp_buf[0]&&'p'==temp_buf[1]){
		if(1==path_buf[3]){
			tmp=((path_buf[5]<<8)|path_buf[4]);
			gsl_write_interface(this_client,path_buf[6],&path_buf[10],tmp);
		}else if(2==path_buf[3]){
			tmp = ((path_buf[5]<<8)|path_buf[4]);
			tmp1=((path_buf[9]<<24)|(path_buf[8]<<16)|(path_buf[7]<<8)
				|path_buf[6]);
			buf[0]=(char)((tmp1/0x80)&0xff);
			buf[1]=(char)(((tmp1/0x80)>>8)&0xff);
			buf[2]=(char)(((tmp1/0x80)>>16)&0xff);
			buf[3]=(char)(((tmp1/0x80)>>24)&0xff);
			buf[4]=(char)(tmp1%0x80);
			gsl_write_interface(this_client,0xf0,buf,4);
			gsl_write_interface(this_client,buf[4],&path_buf[10],tmp);
		}else if(3==path_buf[3]||4==path_buf[3]){
			memcpy(gsl_read,temp_buf,4);
			memcpy(gsl_data_proc,&path_buf[3],7);
		}
	}
#endif
exit_write_proc_out:
	kfree(path_buf);
	return count;
}
#endif
#ifdef TPD_PROC_DEBUG
static int gsl_server_list_open(struct inode *inode,struct file *file)
{
	return single_open(file,gsl_config_read_proc,NULL);
}
static const struct file_operations gsl_seq_fops = {
	.open = gsl_server_list_open,
	.read = seq_read,
	.release = single_release,
	.write = gsl_config_write_proc,
	.owner = THIS_MODULE,
};
#endif

#ifdef GSL_TIMER
static void gsl_timer_check_func(struct work_struct *work)
{
#if 1

	char read_buf[4]  = {0};
	char init_chip_flag = 0;
	int i,flag;
	//printk("----------------gsl_monitor_worker-----------------\n");	

	if(i2c_lock_flag != 0 )
		goto queue_monitor_work;
//		return;
	else
		i2c_lock_flag = 1;

	gsl_ts_read(this_client, 0xb4, read_buf, 4);
	memcpy(int_2nd,int_1st,4);
	memcpy(int_1st,read_buf,4);

	if(int_1st[3] == int_2nd[3] && int_1st[2] == int_2nd[2] &&
		int_1st[1] == int_2nd[1] && int_1st[0] == int_2nd[0])
	{
	//	printk("======int_1st: %x %x %x %x , int_2nd: %x %x %x %x ======\n",
	//		int_1st[3], int_1st[2], int_1st[1], int_1st[0], 
	//		int_2nd[3], int_2nd[2],int_2nd[1],int_2nd[0]);
		init_chip_flag = 1;
		goto queue_monitor_work;
	}
	/*check 0xb0 register,check firmware if ok*/
	for(i=0;i<5;i++){
		gsl_ts_read(this_client, 0xb0, read_buf, 4);
		if(read_buf[3] != 0x5a || read_buf[2] != 0x5a || 
			read_buf[1] != 0x5a || read_buf[0] != 0x5a){
	//		printk("gsl_monitor_worker 0xb0 = {0x%02x%02x%02x%02x};\n",
	//			read_buf[3],read_buf[2],read_buf[1],read_buf[0]);
			flag = 1;
		}else{
			flag = 0;
			break;
		}

	}
	if(flag == 1){
		init_chip_flag = 1;
		goto queue_monitor_work;
	}
	
	/*check 0xbc register,check dac if normal*/
//	for(i=0;i<5;i++){
//		gsl_ts_read(this_client, 0xbc, read_buf, 4);
//		if(read_buf[3] != 0 || read_buf[2] != 0 || 
//			read_buf[1] != 0 || read_buf[0] != 0){
//			flag = 1;
//		}else{
//			flag = 0;
//			break;
//		}
//	}
//	if(flag == 1){
//		//reset_chip(this_client);
//		//startup_chip(this_client);
//		init_chip_flag = 0;
//	}
queue_monitor_work:
	if(init_chip_flag){
		init_chip(this_client);
		memset(int_1st,0xff,sizeof(int_1st));
	}
	

		queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, 300);

	i2c_lock_flag = 0;
#else
	unsigned int data[48];
	gsl_read_oneframe_data(data,0x4*0x80+0x54,48);
	queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, 500);
#endif
}
#endif


#ifdef GSL_GESTURE
#if 0
static unsigned int gsl_read_oneframe_data(unsigned int *data,
				unsigned int addr,unsigned int len)
{
	u8 buf[4];
	int i;
	unsigned int page,offset;
	//printk("tp-gsl-gesture %s\n",__func__);
	//printk("gsl_read_oneframe_data:::addr=%x,len=%x\n",addr,len);
	
	for(i=0;i<len/2;i++)
	{
		page = (addr + i*8)/0x80;
		offset = (addr + i*8)%0x80;
		if(offset == 0x7c)// || offset == 0x78)
			page--;//page==0?
		buf[0] = (page>> 0) & 0xff;
		buf[1] = (page>> 8) & 0xff;
		buf[2] = (page>>16) & 0xff;
		buf[3] = (page>>24) & 0xff;
		gsl_write_interface(this_client,0xf0,buf,4);
		gsl_read_interface(this_client,offset,(char *)&data[i*2],8);
	}
	if(len%2)
	{
		page = (addr + len*4 - 4)/0x80;
		offset = (addr + len*4 - 4)%0x80;
		if(offset == 0x7c)
			page--;//page==0?
		buf[0] = (page>> 0) & 0xff;
		buf[1] = (page>> 8) & 0xff;
		buf[2] = (page>>16) & 0xff;
		buf[3] = (page>>24) & 0xff;
		gsl_write_interface(this_client,0xf0,buf,4);
		gsl_read_interface(this_client,offset,(char *)&data[len-1],4);
	}
	#if 1
	for(i=0;i<len;i++){
		//printk("gsl_read_oneframe_data =%08x\n",data[i]);	
	//print_info("gsl_read_oneframe_data =%x\n",data[len-1]);
	}
	#endif
	
	return len;
}
#else

static unsigned int gsl_read_oneframe_data(unsigned int *data,
				unsigned int addr,unsigned int len)
{
	u8 buf[4];
	int i;
	//printk("tp-gsl-gesture %s\n",__func__);
	//printk("gsl_read_oneframe_data:::addr=%x,len=%x\n",addr,len);
	buf[0] = ((addr)/0x80)&0xff;
	buf[1] = (((addr)/0x80)>>8)&0xff;
	buf[2] = (((addr)/0x80)>>16)&0xff;
	buf[3] = (((addr)/0x80)>>24)&0xff;
	gsl_ts_write(this_client, 0xf0, buf, 4);
	gsl_read_interface(this_client, (((addr)%0x80+8)&0x5f), (char *)&data[0], 4);
	gsl_read_interface(this_client, (addr)%0x80, (char *)&data[0], 4);

	for(i=0;i<len;i++)
	{
		/*buf[0] = ((addr+i*4)/0x80)&0xff;
		buf[1] = (((addr+i*4)/0x80)>>8)&0xff;
		buf[2] = (((addr+i*4)/0x80)>>16)&0xff;
		buf[3] = (((addr+i*4)/0x80)>>24)&0xff;
		gsl_ts_write(this_client, 0xf0, buf, 4);
		gsl_read_interface(this_client, (((addr+i*4)%0x80+8)&0x5f), (char *)&data[i], 4);
		gsl_read_interface(this_client, (addr+i*4)%0x80, (char *)&data[i], 4);
		*/
		gsl_read_interface(this_client, (addr+i*4)%0x80, (char *)&data[i], 4);
	////printk("gsl_read_oneframe_data =%x\n",data[len-1]);
	}
	return len;
}
#endif
static void gsl_enter_doze(struct i2c_client *client)
{
	//printk("%s : enter\n", __func__);
	u8 buf[4] = {0};
#if 0
	u32 tmp;
	gsl_reset_core(ts->client);
	temp = ARRAY_SIZE(GSLX68X_FW_GESTURE);
	gsl_load_fw(ts->client,GSLX68X_FW_GESTURE,temp);
	gsl_start_core(ts->client);
	msleep(1000);		
#endif

	buf[0] = 0xa;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;
	gsl_ts_write(client,0xf0,buf,4);
	buf[0] = 0;
	buf[1] = 0;
	buf[2] = 0x1;
	buf[3] = 0x5a;
	gsl_ts_write(client,0x8,buf,4);
	//gsl_gesture_status = GE_NOWORK;
	msleep(10);
	gsl_gesture_status = GE_ENABLE;
	//printk("%s : end\n", __func__);

}
static void gsl_quit_doze(struct i2c_client *client)
{
	//printk("%s : enter\n", __func__);

	struct gslx680_ts_platform_data *pdata = g_slX680_ts->platform_data;
	
	u8 buf[4] = {0};
	u32 tmp;

	gsl_gesture_status = GE_DISABLE;
	gpio_set_value(pdata->reset_gpio_number, 0);
	msleep(10);	
	gpio_set_value(pdata->reset_gpio_number, 1);
	msleep(20);
	
	buf[0] = 0xa;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;
	gsl_ts_write(client,0xf0,buf,4);
	buf[0] = 0;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0x5a;
	gsl_ts_write(client,0x8,buf,4);
	msleep(10);
	//printk("%s : end\n", __func__);
}


static void gsl_irq_mode_change(struct i2c_client *client,u32 flag)   //fan
{
	//printk("%s : enter\n", __func__);
	u8 buf[4]={0};

	buf[0] = 0x6;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;
	gsl_ts_write(client,0xf0,buf,4);

	//printk("%s :1 enter\n", __func__);

	
	if(GSL_IRQ_FALLING == flag){
		buf[0] = 1;
		buf[1] = 0;
		buf[2] = 0;
		buf[3] = 0;
		gsl_ts_write(client,0x18,buf,4);
		buf[0] = 1;
		buf[1] = 0;
		buf[2] = 0;
		buf[3] = 0;
		gsl_ts_write(client,0x1c,buf,4);
	}else if(GSL_IRQ_RISING == flag){
		buf[0] = 0;
		buf[1] = 0;
		buf[2] = 0;
		buf[3] = 0;
		gsl_ts_write(client,0x18,buf,4);
		buf[0] = 1;
		buf[1] = 0;
		buf[2] = 0;
		buf[3] = 0;
		gsl_ts_write(client,0x1c,buf,4);
	}else if(GSL_IRQ_LOW == flag){
		buf[0] = 1;
		buf[1] = 0;
		buf[2] = 0;
		buf[3] = 0;
		gsl_ts_write(client,0x18,buf,4);
		buf[0] = 0;
		buf[1] = 0;
		buf[2] = 0;
		buf[3] = 0;
		gsl_ts_write(client,0x1c,buf,4);
	}else if(GSL_IRQ_HIGH == flag){
		buf[0] = 0;
		buf[1] = 0;
		buf[2] = 0;
		buf[3] = 0;
		gsl_ts_write(client,0x18,buf,4);
		buf[0] = 0;
		buf[1] = 0;
		buf[2] = 0;
		buf[3] = 0;
		gsl_ts_write(client,0x1c,buf,4);
	}else{
		//printk("%s :2 enter\n", __func__);
		return;
	}
	//printk("%s : end\n", __func__);
}

static ssize_t gsl_sysfs_tpgesture_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u32 count = 0;
	count += scnprintf(buf,PAGE_SIZE,"tp gesture is on/off:\n");
	if(gsl_gesture_flag == 1){
		count += scnprintf(buf+count,PAGE_SIZE-count,
				" on \n");
	}else if(gsl_gesture_flag == 0){
		count += scnprintf(buf+count,PAGE_SIZE-count,
				" off \n");
	}
	count += scnprintf(buf+count,PAGE_SIZE-count,"tp gesture:");
	count += scnprintf(buf+count,PAGE_SIZE-count,
			"%c\n",gsl_gesture_c);
    	return count;
}
static ssize_t gsl_sysfs_tpgesturet_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
#if 1
	if(buf[0] == '0'){
		gsl_gesture_flag = 0;  
	}else if(buf[0] == '1'){
		gsl_gesture_flag = 1;
	}
#endif
	return count;
}   
static DEVICE_ATTR(gesture, 0666, gsl_sysfs_tpgesture_show, gsl_sysfs_tpgesturet_store);   //fan



static struct attribute *gsl_attrs[] = {
	&dev_attr_gesture.attr,
	NULL
};
static const struct attribute_group gsl_attr_group = {
	.attrs = gsl_attrs,
};
#endif  //define GSL_GESTURE  //fan


#ifdef  USE_TP_PSENSOR
/*customer implement: do something like read data from TP IC*/
static int tpd_get_phone_state(void)
{
	struct filp * fp=NULL;
	mm_segment_t fs;
	loff_t pos = 0;
	char buf[20]={0};
	char *p=NULL;
	int ret = 0;
	//printk("====liyan=== --- == %c\n",__func__);
	fp = filp_open(PS_PHONE_STATE_FILE ,O_RDONLY , 0);
	if(IS_ERR(fp)){
		ret = 0;
		//printk("====liyan=== ---open file error\n");
	}else{
		fs = get_fs();//get old fs;
		set_fs(KERNEL_DS);
		vfs_read(fp,buf,sizeof(buf),&pos);
		p = buf;
		//printk("====liyan=== --- *phone state == %c\n",*p);
		if(*p == '1'){
			ret = 1;
		}else{
			ret = 0;
		}

		filp_close(fp,NULL);
		set_fs(fs);
	}
	return ret;
}
static int tpd_get_ps_value(void)
{
	return tpd_proximity_detect;
}


static void gsl_gain_psensor_data(struct i2c_client *client)
{
	u8 buf[4]={0};
	/**************************/
	buf[0]=0x3;
	gsl_ts_write(client,0xf0,buf,4);
	gsl_ts_read(client,0,&gsl_psensor_data[0],4);
	/**************************/

	buf[0]=0x4;
	gsl_ts_write(client,0xf0,buf,4);
	gsl_ts_read(client,0,&gsl_psensor_data[4],4);
	/**************************/
}

static int tp_ps_enable(int enable)
{
	u8 state;
	int ret = -1;
	u8 buf[4]={0};
	
	//printk(" enter tp_ps_enable  :: enable = %d\n", enable);
	//printk("GSL::gsl_psensor_data=%d,%d,%d,%d,%d,%d,%d,%d\n",gsl_psensor_data[0],gsl_psensor_data[1],gsl_psensor_data[2],\
			gsl_psensor_data[3],gsl_psensor_data[4],gsl_psensor_data[5],gsl_psensor_data[6],gsl_psensor_data[7]);
	if (enable){
		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x3;
		gsl_ts_write(this_client, 0xf0, buf, 4);
		buf[3] = 0x5a;
		buf[2] = 0x5a;
		buf[1] = 0x5a;
		buf[0] = 0x5a;
		gsl_ts_write(this_client, 0, buf, 4);

		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x4;
		gsl_ts_write(this_client, 0xf0, buf, 4);
		buf[3] = 0x0;
		buf[2] = 0x0;
		buf[1] = 0x0;
		buf[0] = 0x2;
		gsl_ts_write(this_client, 0, buf, 4);
		ps_en = 1;
		//printk("GSL::ps function is on\n");	
		
	}else{
	//0x17190479
	#if 1
		ps_en = 0;
		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x3;
		gsl_ts_write(this_client, 0xf0, buf, 4);
		buf[3] = 0x17;
		buf[2] = 0x19;
		buf[1] = 0x04;
		buf[0] = 0x79;
		gsl_ts_write(this_client, 0, buf, 4);

		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x4;
		gsl_ts_write(this_client, 0xf0, buf, 4);
		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x00;
		gsl_ts_write(this_client, 0, buf, 4);
		//printk("GSL::ps function is off\n");	
	#else
		ps_en = 0;
		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x3;
		gsl_ts_write(this_client, 0xf0, buf, 4);
		buf[3] = gsl_psensor_data[3];
		buf[2] = gsl_psensor_data[2];
		buf[1] = gsl_psensor_data[1];
		buf[0] = gsl_psensor_data[0];
		gsl_ts_write(this_client, 0, buf, 4);

		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x4;
		gsl_ts_write(this_client, 0xf0, buf, 4);
		buf[3] = gsl_psensor_data[7];
		buf[2] = gsl_psensor_data[6];
		buf[1] = gsl_psensor_data[5];
		buf[0] = gsl_psensor_data[4];
		gsl_ts_write(this_client, 0, buf, 4);
		//printk("GSL::gsl_psensor_data=%d,%d,%d,%d,%d,%d,%d,%d\n",gsl_psensor_data[0],gsl_psensor_data[1],gsl_psensor_data[2],\
			gsl_psensor_data[3],gsl_psensor_data[4],gsl_psensor_data[5],gsl_psensor_data[6],gsl_psensor_data[7]);
		//printk("GSL::ps function is off\n");	
	#endif
	}
	return 0;
}

static int tp_ps_disable()
{
	//printk("%s\n", __func__);

	ps_en = 0;
	return 0;
}

static int tp_ps_open(struct inode *inode, struct file *file)
{
	//printk("%s\n", __func__);
	if (tp_ps_opened)
		return -EBUSY;
	tp_ps_opened = 1;
	return 0;
}

static int tp_ps_release(struct inode *inode, struct file *file)
{
	//printk("%s", __func__);
	tp_ps_opened = 0;
	return 0;//tp_ps_disable();
}

static long tp_ps_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int flag;
	int err = 0;

	//printk("%s: cmd %d", __func__, _IOC_NR(cmd));

	switch (cmd)
	{
		case FOCALTECH_IOCTL_SET_PFLAG:
			if (copy_from_user(&flag, argp, sizeof(flag)))
				return -EFAULT;
			//printk("FOCALTECH_IOCTL_SET_PFLAG");
			if(flag)
			{
				if((err=tp_ps_enable(1)) != 0)
				{
					//printk("enable ps fail: %d\n", err); 
					return -EFAULT;
				}
			}
			else
			{
				if((err=tp_ps_enable(0)) != 0)
				{
					//printk("disable ps fail: %d\n", err); 
					return -EFAULT;
				}
			}
			break;

		case FOCALTECH_IOCTL_SET_LFLAG:
			break;

		case FOCALTECH_IOCTL_GET_PFLAG:
			flag = ps_en;
			if (copy_to_user(argp, &flag, sizeof(flag)))
				return -EFAULT;
			break;

		case FOCALTECH_IOCTL_GET_LFLAG:
			flag = 0;
			if (copy_to_user(argp, &flag, sizeof(flag)))
				return -EFAULT;
			break;

		default:
			//printk("%s: invalid cmd %d", __func__, _IOC_NR(cmd));
			return -EINVAL;
	}

	return 0;
}

static struct file_operations tp_ps_fops = {
	.owner			= THIS_MODULE,
	.open			= tp_ps_open,
	.release		= tp_ps_release,
	.unlocked_ioctl		= tp_ps_ioctl,
};

static struct miscdevice tp_ps_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = TP_PS_DEVICE,
	.fops = &tp_ps_fops,
};


static int tp_ps_init(struct i2c_client *client)
{
	int err = 0;
	//struct input_dev *input_dev;

	ps_en = 0;
	
	//register device
	err = misc_register(&tp_ps_device);
	if (err) {
		PS_DBG("%s: tp_ps_device register failed\n", __func__);
		goto exit_misc_reg_fail;
	}

	// register input device 
	gsl_pls_input_dev = input_allocate_device();
	if (!gsl_pls_input_dev) 
	{
		PS_DBG("%s: input allocate device failed\n", __func__);
		err = -ENOMEM;
		goto exit_input_dev_allocate_failed;
	}

//	tp_ps->input = input_dev;

	gsl_pls_input_dev->name = TP_PS_INPUT_DEV;
	gsl_pls_input_dev->phys  = TP_PS_INPUT_DEV;
	gsl_pls_input_dev->id.bustype = BUS_I2C;
	gsl_pls_input_dev->dev.parent = &client->dev;
	gsl_pls_input_dev->id.vendor = 0x0001;
	gsl_pls_input_dev->id.product = 0x0001;
	gsl_pls_input_dev->id.version = 0x0010;

//	__set_bit(EV_ABS, input_dev->evbit);	
	//for proximity
	__set_bit(EV_ABS, gsl_pls_input_dev->evbit);
	input_set_abs_params(gsl_pls_input_dev, ABS_DISTANCE, 0, 1, 0, 0);
	input_set_abs_params(gsl_pls_input_dev, ABS_MISC, 0, 100001, 0, 0);
	err = input_register_device(gsl_pls_input_dev);
	if (err < 0)
	{
	    PS_DBG("%s: input device regist failed\n", __func__);
	    goto exit_input_register_failed;
	}

	PS_DBG("%s: Probe Success!\n",__func__);
	return 0;

exit_input_register_failed:
	input_free_device(gsl_pls_input_dev);
exit_input_dev_allocate_failed:
	misc_deregister(&tp_ps_device);
exit_misc_reg_fail:
	kfree(&tp_ps_device);
}

static int tp_ps_uninit()
{
	misc_deregister(&tp_ps_device);
	//free input
	input_unregister_device(gsl_pls_input_dev);
	//input_free_device(gsl_pls_input_dev);
	//free alloc
	kfree(&tp_ps_device);
	gsl_pls_input_dev = 0;
}

#endif

#ifdef HAVE_TOUCH_KEY
static void report_key(struct gslX680_ts_data *ts, u16 x, u16 y)
{
	u16 i = 0;

	for(i = 0; i < MAX_KEY_NUM; i++) 
	{
		if((gsl_key_data[i].x_min < x) && (x < gsl_key_data[i].x_max)&&(gsl_key_data[i].y_min < y) && (y < gsl_key_data[i].y_max))
		{
			key = gsl_key_data[i].key;	
			input_report_key(ts->input_dev, key, 1);
			input_sync(ts->input_dev); 		
			key_state_flag = 1;
			break;
		}
	}
}
#endif

static void gsl_report_point(struct input_dev *idev, struct gsl_touch_info *cinfo)
{
	int i; 	
	u32 temp=0;
	int gsl_up_flag = 0;
	if(cinfo->finger_num>0 && cinfo->finger_num<6)
	{
		gsl_up_flag = 0;

		for(i=0;i<cinfo->finger_num;i++)
		{
		//	gsl_point_state |= (0x1<<cinfo->id[i]);	
			//printk("chenmingming:id = %d, x = %d, y = %d \n",cinfo->id[i], cinfo->x[i],cinfo->y[i]);	
			input_report_key(idev, BTN_TOUCH, 1);
			input_report_abs(idev, ABS_MT_TRACKING_ID, cinfo->id[i]-1);
			//input_report_abs(idev, ABS_MT_TOUCH_MAJOR, GSL_PRESSURE);
			input_report_abs(idev, ABS_MT_POSITION_X, cinfo->x[i]);
			input_report_abs(idev, ABS_MT_POSITION_Y, cinfo->y[i]);	
			input_report_abs(idev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(idev);			
		}
	}
	else if(cinfo->finger_num == 0)
	{

		if(1 == gsl_up_flag)
		{
			return;
		}
		input_report_key(idev, BTN_TOUCH, 0);
		input_mt_sync(idev);
		gsl_up_flag = 1;
	}
	input_sync(idev);
}

static void gsl_report_work(struct work_struct *work)
{
	int rc,tmp;
	u8 buf[44] = {0};
	//u8 ps_buf[4] = {0};
	int tmp1=0;
	struct gsl_touch_info *cinfo = gslX680_ts->cinfo;
	struct i2c_client *client = gslX680_ts->client;
	struct input_dev *idev = gslX680_ts->input_dev;
#ifdef GSL_GESTURE
	unsigned int test_count = 0;
#endif
	
	struct gslX680_ts_data *ts = i2c_get_clientdata(this_client);	
	//if(1 == gsl_sw_flag)
		//goto schedule;
#ifdef GSL_TIMER
	if(i2c_lock_flag != 0 && err_count<10)
	{
		err_count ++;
		goto schedule;
	}
	i2c_lock_flag = 1;
	err_count = 0;
#endif
#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1){
		goto schedule;
	}
#endif

#ifdef USE_TP_PSENSOR
	u8 ps_buf[4] = {0};
	//printk("GSL_report_work::ps_en = %d\n",ps_en);
	if(ps_en)
	{
		gsl_ts_read(this_client, 0xac, ps_buf, 4);
//		//printk("GSL_report_work::ps_value = %02x\n",ps_buf[0]);
		if(ps_buf[0] == 0x1)
		{
			tpd_proximity_detect = 0;
		}else{
			tpd_proximity_detect = 1;
		}
		input_report_abs(gsl_pls_input_dev, ABS_DISTANCE, tpd_get_ps_value());
		input_sync(gsl_pls_input_dev);
	}
#endif
	
	/* read data from DATA_REG */
	rc = gsl_ts_read(client, 0x80, buf, 4);
	if (rc < 0) 
	{
		dev_err(&client->dev, "[gsl] I2C read failed\n");
		goto schedule;
	}

#ifdef GSL_GESTURE
	//if((buf[3]==0)&&(buf[2]==0)&&(buf[1]==0)&&(buf[0]==0))
	//{
		//msleep(10);
		gsl_ts_read(client, 0x80, buf, 4);
	//}
#endif
	if (buf[0] == 0xff) {
		goto schedule;
	}
	////printk("GSL:::0x80=%02x%02x%02x%02x\n",buf[3],buf[2],buf[1],buf[0]);
	////printk("GSL:::0x84=%02x%02x%02x%02x\n",buf[7],buf[6],buf[5],buf[4]);
	////printk("GSL:::0x88=%02x%02x%02x%02x\n",buf[11],buf[10],buf[9],buf[8]);
	cinfo->finger_num = buf[0]&0x0f;
	if(cinfo->finger_num > 0)
		gsl_ts_read(client, 0x84, &buf[4], 8);
	if(cinfo->finger_num > 2)
		gsl_ts_read(client, 0x8c, &buf[12], 8);
	if(cinfo->finger_num > 4)
		gsl_ts_read(client, 0x94, &buf[20], 8);
	if(cinfo->finger_num > 6)
		gsl_ts_read(client, 0x9c, &buf[28], 8);
	if(cinfo->finger_num > 8)
		gsl_ts_read(client, 0xa4, &buf[36], 8);
	//printk("GSL:::0x80=%02x%02x%02x%02x\n",buf[3],buf[2],buf[1],buf[0]);
	//printk("GSL:::0x84=%02x%02x%02x%02x\n",buf[7],buf[6],buf[5],buf[4]);
	//printk("GSL:::0x88=%02x%02x%02x%02x\n",buf[11],buf[10],buf[9],buf[8]);
	for(tmp=0;tmp<(cinfo->finger_num>10 ? 10:cinfo->finger_num);tmp++)
	{
		cinfo->y[tmp] = (buf[tmp*4+4] | ((buf[tmp*4+5])<<8));
		cinfo->x[tmp] = (buf[tmp*4+6] | ((buf[tmp*4+7] & 0x0f)<<8));
		cinfo->id[tmp] = buf[tmp*4+7] >> 4;
		//printk("tp-gsl  x = %d y = %d \n",cinfo->x[tmp],cinfo->y[tmp]);
	}
	
//	print_info("111 finger_num= %d\n",cinfo->finger_num);
#ifdef GSL_NOID_VERSION
	cinfo->finger_num = (buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|(buf[0]);
	gsl_alg_id_main(cinfo);
	tmp1=gsl_mask_tiaoping();
//	print_info("[tp-gsl] tmp1=%x\n",tmp1);
	if(tmp1>0&&tmp1<0xffffffff)
	{
		buf[0]=0xa;
		buf[1]=0;
		buf[2]=0;
		buf[3]=0;
		gsl_ts_write(client,0xf0,buf,4);
		buf[0]=(u8)(tmp1 & 0xff);
		buf[1]=(u8)((tmp1>>8) & 0xff);
		buf[2]=(u8)((tmp1>>16) & 0xff);
		buf[3]=(u8)((tmp1>>24) & 0xff);
//		print_info("tmp1=%08x,buf[0]=%02x,buf[1]=%02x,buf[2]=%02x,buf[3]=%02x\n",
//			tmp1,buf[0],buf[1],buf[2],buf[3]);
		gsl_ts_write(client,0x8,buf,4);
	}
#endif

/*	if(cinfo->finger_num == 0)
	{
		gsl_ts_read(this_client, 0xac, ps_buf, 4);
	printk("GSL:::0xac=%02x%02x%02x%02x\n",ps_buf[3],ps_buf[2],ps_buf[1],ps_buf[0]);
	}
*/

		#ifdef GSL_GESTURE
	//printk("gsl_gesture_status=%d,gsl_gesture_flag=%d\n",gsl_gesture_status,gsl_gesture_flag);
	
	int tmp_c;
	if(GE_ENABLE == gsl_gesture_status && gsl_gesture_flag == 1){
		u8 key_data = 0;
		
		tmp_c = gsl_obtain_gesture();
	
		//printk("gsl_obtain_gesture():tmp_c=%x\n",tmp_c);

		switch(tmp_c){
		case (int)'C':
			key_data = KEY_C;
			break;
		case (int)'E':
			key_data = KEY_E;
			break;
		case (int)'W':
			key_data = KEY_W;
			break;
		case (int)'O':
			key_data = KEY_O;
			break;
		case (int)'M':
			key_data = KEY_M;
			break;
		case (int)'Z':
			key_data = KEY_Z;
			break;
		case (int)'V':
			key_data = KEY_V;
			break;
		case (int)'S':
			key_data = KEY_S;
			break;
		case (int)'A':
			key_data = KEY_A;
			break;
		case (int)'*':		/* double click */
			key_data = KEY_U;
			break;
		case (int)0xa1fa:	/* right */
			key_data = KEY_RIGHT;
			break;
		case (int)0xa1fd:	/* down */
			key_data = KEY_DOWN;
			break;
		case (int)0xa1fc:	/* up */
			key_data = KEY_UP;
			break;
		case (int)0xa1fb:	/* left */
			key_data = KEY_LEFT;
			break;
		default:
			key_data = 0;
			break;
		}

		gsl_gesture_c = (char)(tmp_c & 0xff);
		//printk("gsl_gesture_c=%x\n",gsl_gesture_c);
		
		if(key_data != 0){
			//gsl_gesture_status = GE_WAKEUP;
			input_report_key(gslX680_ts->input_dev,key_data,1);
			input_sync(gslX680_ts->input_dev);			
			//msleep(100);
			input_report_key(gslX680_ts->input_dev,key_data,0);
			input_sync(gslX680_ts->input_dev);
			msleep(400);	
		}
	//	msleep(400);
		goto schedule;
	}
#endif   //fan


//	print_info("222 finger_num= %d\n",cinfo->finger_num);
	gsl_report_point(idev,cinfo);
//#ifdef GSL_TIMER
//	i2c_lock_flag = 0;
//#endif
schedule:
	enable_irq(this_client->irq);
#ifdef GSL_TIMER
	i2c_lock_flag = 0;
#endif

}


static irqreturn_t gslX680_ts_interrupt(int irq, void *dev_id)
{

	struct gslX680_ts_data *gslX680_ts = (struct gslX680_ts_data *)dev_id;

	//print_info("gslX680_ts_interrupt");

    	disable_irq_nosync(this_client->irq);
#ifdef GSL_GESTURE
	if(gsl_gesture_status==GE_ENABLE&&gsl_gesture_flag==1){
		wake_lock_timeout(&gsl_wake_lock, msecs_to_jiffies(2000));
		print_info("gsl-jeft\n");
	}
#endif
	
	if (!work_pending(&gslX680_ts->pen_event_work)) {
		queue_work(gslX680_ts->ts_workqueue, &gslX680_ts->pen_event_work);
	}

	return IRQ_HANDLED;
}


//#ifdef CONFIG_HAS_EARLYSUSPEND
static void gslX680_ts_suspend(struct early_suspend *handler)
{
	//printk("==gslX680_ts_suspend=ps_en=%d  phone_state=%d\n",ps_en,tpd_get_phone_state());
	struct gslx680_ts_platform_data *pdata = g_slX680_ts->platform_data;
#ifdef USE_TP_PSENSOR
	if (ps_en == 1)
	{
	//	ps_en = 0;
		gsl_ps_suspend_flag = 1;	
		if(!tpd_get_phone_state())
			tp_ps_enable(0);
		else
		return;
	}else if(tpd_get_phone_state()){
			tp_ps_enable(1);
			return;
	}
    	gsl_ps_suspend_flag = 0;
#endif
#ifdef GSL_GESTURE	
	//wangcq327 add start
	struct filp * fp=NULL;
	mm_segment_t fs;
	loff_t pos = 0;
	char buf[200]={0};
	char *p=NULL;
	fp = filp_open(GESTURE_SWITCH_FILE ,O_RDONLY , 0);
	if(IS_ERR(fp)){
		//printk("wangcq327 --- touchpanle Open file fail !!\n");
		gsl_gesture_flag = 1;
	}else{
		//read start
		fs = get_fs();//get old fs;
		set_fs(KERNEL_DS);
		vfs_read(fp,buf,sizeof(buf),&pos);
		////printk("wangcq327 --- read == %s\n",buf);
		//read end

		p = strstr(buf,"value=");//value="1"
		p += 7;

		////printk("wangcq327 --- *p == %c\n",*p);
		if(*p == '1'){
			gsl_gesture_flag = 1;
		}else{
			gsl_gesture_flag = 0;
		}
		////printk("wangcq327 --- TP_gesture_Switch == %d\n",(TP_gesture_Switch==true)?1:0);
		if(gsl_gesture_flag == 1)
			gsl_gesture_status = GE_ENABLE;
		else
			gsl_gesture_status = GE_DISABLE;
		filp_close(fp,NULL);
		set_fs(fs);
	}
	//wangcq327 add end
#endif	
#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1){
		return;
	}
#endif
	
	


#ifdef GSL_TIMER	
	cancel_delayed_work_sync(&gsl_timer_check_work);
#endif
	
#ifdef GSL_GESTURE
//	gsl_gesture_flag = 1;
    int ret;
	if(gsl_gesture_flag == 1&&gsl_ps_suspend_flag!=1){
		gsl_irq_mode_change(this_client,GSL_IRQ_HIGH);
		irq_set_irq_type(this_client->irq, IRQF_TRIGGER_HIGH|IRQF_NO_SUSPEND); //IRQ_TYPE_EDGE_BOTH IRQ_TYPE_LEVEL_LOW
		ret = enable_irq_wake(this_client->irq);		
		print_info("set_irq_wake(1) result = %d\n",ret);
		gsl_enter_doze(this_client);
		return;
	}
#endif//fan
    


    disable_irq_nosync(this_client->irq);
	gpio_set_value(pdata->reset_gpio_number, 0);
}

static void gslX680_ts_resume(struct early_suspend *handler)
{	
	//printk("==gslX680_ts_resume==ps_en=%d\n",ps_en);

	struct gslx680_ts_platform_data *pdata = g_slX680_ts->platform_data;
#ifdef USE_TP_PSENSOR
	//printk("ps_en = %d,gsl_ps_suspend_flag = %d phone state=%d\n",ps_en,gsl_ps_suspend_flag,tpd_get_phone_state());
	gsl_ps_suspend_flag=0;
	if (ps_en == 1) {    		
    		if(!tpd_get_phone_state())
				tp_ps_enable(0);
		else
			return;
	}else if(tpd_get_phone_state()){
		tp_ps_enable(1);
		return;
    }
#endif
#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1){
		return;
	}
#endif

#ifdef GSL_GESTURE
	int ret;
	if(gsl_gesture_flag == 1){
		gsl_gesture_status = GE_WAKEUP;
		ret =disable_irq_wake(this_client->irq);
		print_info("set_irq_wake(0) result = %d\n",ret);				
		gsl_quit_doze(this_client);
		irq_set_irq_type(this_client->irq,IRQF_TRIGGER_RISING);
		gsl_irq_mode_change(this_client,GSL_IRQ_FALLING);
	}
			//	gsl_gesture_flag = 0;
#endif  	//fan
    
	gpio_set_value(pdata->reset_gpio_number, 1);
	msleep(20);
	reset_chip(this_client);
	startup_chip(this_client);	
	msleep(20);	
	check_mem_data(this_client);
#ifdef GSL_TIMER
	queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE);
#endif
	
	enable_irq(this_client->irq);	
}

//#endif

#ifdef CONFIG_OF
static struct gslx680_ts_platform_data *gslx680_ts_parse_dt(struct device *dev)
{
	struct gslx680_ts_platform_data *pdata;
	struct device_node *np = dev->of_node;
	int ret;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "Could not allocate struct ft5x0x_ts_platform_data");
		return NULL;
	}
	pdata->reset_gpio_number = of_get_gpio(np, 0);
	if(pdata->reset_gpio_number < 0){
		dev_err(dev, "fail to get reset_gpio_number\n");
		goto fail;
	}
	pdata->irq_gpio_number = of_get_gpio(np, 1);
	if(pdata->reset_gpio_number < 0){
		dev_err(dev, "fail to get reset_gpio_number\n");
		goto fail;
	}
	ret = of_property_read_string(np, "vdd_name", &pdata->vdd_name);
	if(ret){
		dev_err(dev, "fail to get vdd_name\n");
		goto fail;
	}
	ret = of_property_read_u32_array(np, "virtualkeys", &pdata->virtualkeys,12);
	if(ret){
		dev_err(dev, "fail to get virtualkeys\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "TP_MAX_X", &pdata->TP_MAX_X);
	if(ret){
		dev_err(dev, "fail to get TP_MAX_X\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "TP_MAX_Y", &pdata->TP_MAX_Y);
	if(ret){
		dev_err(dev, "fail to get TP_MAX_Y\n");
		goto fail;
	}

	pr_info("[FST] %s  chenbowei [irq=%d];[rst=%d]\ [vdd = %s]n",__func__,pdata->irq_gpio_number,pdata->reset_gpio_number,pdata->vdd_name);
	
	return pdata;
fail:
	kfree(pdata);
	return NULL;
}
#endif

ssize_t DrvtpnameRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nLength;

	if (*pPos != 0)
    {
        return 0;
    }
	
    nLength = sprintf(pBuffer, "%s\n", GSLX680_TS_NAME_SPRD_NEED);

	*pPos += nLength;
	
    return nLength;
}	
			  
ssize_t DrvtpnameWrite(struct file *pFile, const char __user *pBuffer, size_t nCount, loff_t *pPos)  
{
    return nCount;
}

ssize_t DrvmytpnameRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nLength;

	if (*pPos != 0)
    {
        return 0;
    }
	
    nLength = sprintf(pBuffer, "%s\n", GSLX680_MY_TP_NAME);

	*pPos += nLength;
	
    return nLength;
}	
			  
ssize_t DrvmytpnameWrite(struct file *pFile, const char __user *pBuffer, size_t nCount, loff_t *pPos)  
{
    return nCount;
}


static const struct file_operations _gProctpname = { 
    .read = DrvtpnameRead,
    .write = DrvtpnameWrite,
};

static const struct file_operations my_gProctpname = { 
    .read = DrvmytpnameRead,
    .write = DrvmytpnameWrite,
};

static int gslX680_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	//struct gslX680_ts_data *gslX680_ts;
	struct input_dev *input_dev;
	struct regulator *reg_vdd;
	struct gslx680_ts_platform_data *pdata = client->dev.platform_data;
	int err = 0;
	unsigned char uc_reg_value=0; 
	u16 i = 0;

	print_info("%s\n",__func__);
	
#ifdef CONFIG_OF
	struct device_node *np = client->dev.of_node;
	if (np && !pdata){
		pdata = gslx680_ts_parse_dt(&client->dev);
		if(pdata){
			client->dev.platform_data = pdata;
		}
		else{
			err = -ENOMEM;
			goto exit_alloc_platform_data_failed;
		}
	}
#endif

	gslX680_ts = kzalloc(sizeof(*gslX680_ts), GFP_KERNEL);
	if (!gslX680_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	gslX680_ts->cinfo = kzalloc(sizeof(struct gsl_touch_info),GFP_KERNEL);
	if(gslX680_ts->cinfo == NULL)
	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	g_slX680_ts = gslX680_ts;
	this_client = client;
	gslX680_ts->platform_data = pdata;
	gslX680_ts->client = client;
	mutex_init(&gsl_i2c_lock);	
	//gslx680_ts_hw_init(gslX680_ts);
	//pdata->irq_gpio_number,pdata->reset_gpio_number);
	gpio_request(pdata->irq_gpio_number, "ts_irq_pin");
	gpio_request(pdata->reset_gpio_number, "ts_rst_pin");
	gpio_direction_output(pdata->reset_gpio_number, 1);
	gpio_direction_input(pdata->irq_gpio_number);

	reg_vdd = regulator_get(&client->dev, pdata->vdd_name);
	if (!WARN(IS_ERR(reg_vdd), "[FST] gslx680_ts_hw_init regulator: failed to get %s.\n", pdata->vdd_name)) {
		regulator_set_voltage(reg_vdd, 2800000, 2800000);
		regulator_enable(reg_vdd);
	}
	msleep(100);
	
	i2c_set_clientdata(client, gslX680_ts);
	//gpio_direction_input(sprd_3rdparty_gpio_tp_irq);	/*Simon:add the irq*/
	client->irq = gpio_to_irq(pdata->irq_gpio_number);
	
#ifdef GSL_GPIO_TP  
	    gsl_cfg_index = gsl2336_Check_TP_ID();
#endif 
	#ifdef CONFIG_I2C_SPRD
	//sprd_i2c_ctl_chg_clk(client->adapter->nr, 400000);
	#endif
	sprd_i2c_ctl_chg_clk(client->adapter->nr,100*1000);  //add by rl for tp i2c speed 100k 20150326
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}
	
	
	err = test_i2c(client);
	if(err < 0)
	{
		//printk("------gslX680 test_i2c error, no silead chip------\n");	
		goto exit_check_functionality_failed;
	}	

#ifdef USE_TP_PSENSOR
	tp_ps_init(client);
	gsl_gain_psensor_data(client);
#endif

	print_info("I2C addr=%x\r\n", client->addr);
#ifdef GSL_GESTURE
	wake_lock_init(&gsl_wake_lock, WAKE_LOCK_SUSPEND, "gsl_wake_lock");
#endif

	INIT_WORK(&gslX680_ts->pen_event_work, gsl_report_work);

	gslX680_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!gslX680_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
	
	//printk("%s: ==request_irq=\r\n",__func__);
	//printk("%s IRQ number is %d\r\n", client->name, client->irq);	/*Simon:why client's irq is 0,where is client value ? */
#ifdef GSL_GESTURE
	err = request_irq(client->irq,  gslX680_ts_interrupt, IRQF_TRIGGER_RISING | IRQF_ONESHOT | IRQF_NO_SUSPEND, client->name, gslX680_ts);
#else	
	err = request_irq(client->irq, gslX680_ts_interrupt, IRQF_TRIGGER_RISING, client->name, gslX680_ts);
#endif	
	if (err < 0) {
		dev_err(&client->dev, "gslX680_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	disable_irq(client->irq);

	//printk("==input_allocate_device=\n");
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	
	gslX680_ts->input_dev = input_dev;
	//gslX680_ts->idev = input_dev;

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);
#if 0
	__set_bit(EV_REP, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT,input_dev->propbit);
	input_mt_init_slots(input_dev,5);
#else
	__set_bit(BTN_TOUCH, input_dev->keybit);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID,0,5,0,0);
#endif
	__set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);

#ifdef TOUCH_VIRTUAL_KEYS   /*VIRTUAL_KEYS*/
	__set_bit(KEY_MENU,  input_dev->keybit);
	__set_bit(KEY_BACK,  input_dev->keybit);
	__set_bit(KEY_HOMEPAGE,  input_dev->keybit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	input_set_capability(input_dev, EV_KEY, KEY_HOMEPAGE);
	input_set_capability(input_dev, EV_KEY, KEY_MENU);
	input_set_capability(input_dev, EV_KEY, KEY_BACK);
	input_set_capability(input_dev, EV_KEY, KEY_SEARCH);
#endif

#ifdef GSL_GESTURE
	input_set_capability(input_dev, EV_KEY, KEY_UP);
	input_set_capability(input_dev, EV_KEY, KEY_DOWN);
	input_set_capability(input_dev, EV_KEY, KEY_LEFT);
	input_set_capability(input_dev, EV_KEY, KEY_RIGHT);
	input_set_capability(input_dev, EV_KEY, KEY_O);
	input_set_capability(input_dev, EV_KEY, KEY_W);
	input_set_capability(input_dev, EV_KEY, KEY_M);
	input_set_capability(input_dev, EV_KEY, KEY_E);
	input_set_capability(input_dev, EV_KEY, KEY_C);
	input_set_capability(input_dev, EV_KEY, KEY_Z);
	input_set_capability(input_dev, EV_KEY, KEY_V);
	input_set_capability(input_dev, EV_KEY, KEY_S);
	input_set_capability(input_dev, EV_KEY, KEY_POWER);
	input_set_capability(input_dev, EV_KEY, KEY_U);
	input_set_capability(input_dev, EV_KEY, KEY_A);
#endif	//fan
	
	input_set_abs_params(input_dev,
				ABS_MT_TRACKING_ID, 0, 255, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, pdata->TP_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, pdata->TP_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
#if MULTI_PROTOCOL_TYPE_B	
	input_mt_init_slots(input_dev, 11,0);
#endif

#ifdef HAVE_TOUCH_KEY
    for(i = 0; i < MAX_KEY_NUM; i++)
    {
        input_set_capability(input_dev, EV_KEY, key_array[i]);
    }
#endif
	input_dev->name = GSLX680_TS_NAME;		//dev_name(&client->dev)
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"gslX680_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
   

       	
	init_chip(this_client);
	check_mem_data(this_client);

#ifdef TOUCH_VIRTUAL_KEYS
	pixcir_ts_virtual_keys_init();
#endif

	//printk("==register_early_suspend =");
	gslX680_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	//gslX680_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	gslX680_ts->early_suspend.suspend = gslX680_ts_suspend;
	gslX680_ts->early_suspend.resume	= gslX680_ts_resume;
	register_early_suspend(&gslX680_ts->early_suspend);
#ifdef GSL_GESTURE
	sysfs_create_group(&client->dev.kobj,&gsl_attr_group);
#endif   //fan
	
   	enable_irq(client->irq);
#ifdef GSL_GESTURE
		gsl_FunIICRead(gsl_read_oneframe_data);
#endif

#ifdef GSL_TIMER
	INIT_DELAYED_WORK(&gsl_timer_check_work, gsl_timer_check_func);
	gsl_timer_workqueue = create_workqueue("gsl_timer_check");
	queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE);
#endif
		
#ifdef TPD_PROC_DEBUG
#if 0
	gsl_config_proc = create_proc_entry(GSL_CONFIG_PROC_FILE, 0666, NULL);
	if (gsl_config_proc == NULL)
	{
		print_info("create_proc_entry %s failed\n", GSL_CONFIG_PROC_FILE);
	}
	else
	{
		gsl_config_proc->read_proc = gsl_config_read_proc;
		gsl_config_proc->write_proc = gsl_config_write_proc;
	}
#else
	proc_create(GSL_CONFIG_PROC_FILE,0666,NULL,&gsl_seq_fops);
#endif
	gsl_proc_flag = 0;
#endif

	if (NULL == proc_create("cp_tpInfo", 0777, NULL, &_gProctpname))
    {
        printk("Failed to create _gProctpname!\n");
    }


	if (NULL == proc_create("my_tpInfo", 0777, NULL, &my_gProctpname))
    {
        printk("Failed to create my_gProctpname!\n");
    }

	print_info("%s: ==probe over =\n",__func__);
	return 0;

	

exit_input_register_device_failed:
	input_free_device(input_dev);
	//return err;
exit_input_dev_alloc_failed:
	free_irq(client->irq, gslX680_ts);
exit_irq_request_failed:
	cancel_work_sync(&gslX680_ts->pen_event_work);
	destroy_workqueue(gslX680_ts->ts_workqueue);
exit_create_singlethread:
	//printk("==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
	kfree(gslX680_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	gpio_free(pdata->irq_gpio_number);
	gpio_free(pdata->reset_gpio_number);
	//sprd_free_gpio_irq(gslX680_ts_setup.irq);		/*Simon delete*/
	return err;
exit_alloc_platform_data_failed:
	return err;
}

static int __exit gslX680_ts_remove(struct i2c_client *client)
{

	struct gslX680_ts_data *gslX680_ts = i2c_get_clientdata(client);

	//printk("==gslX680_ts_remove=\n");

#ifdef USE_TP_PSENSOR
	tp_ps_uninit();
#endif

#ifdef GSL_GESTURE
	sysfs_remove_group(&client->dev.kobj,&gsl_attr_group);	
#endif  
	unregister_early_suspend(&gslX680_ts->early_suspend);
	free_irq(client->irq, gslX680_ts);
	//sprd_free_gpio_irq(gslX680_ts_setup.irq);	/*Simon delete*/
	input_unregister_device(gslX680_ts->input_dev);
	kfree(gslX680_ts);
	cancel_work_sync(&gslX680_ts->pen_event_work);
	destroy_workqueue(gslX680_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL);

	//LDO_TurnOffLDO(LDO_LDO_SIM2);

	return 0;
}

static const struct i2c_device_id gslX680_ts_id[] = {
	{ GSLX680_TS_NAME, 0 },{ }
};

MODULE_DEVICE_TABLE(i2c, gslX680_ts_id);

static const struct of_device_id gslx680_of_match[] = {
       { .compatible = "gslx680,gslx680_ts", },
       { }
};

MODULE_DEVICE_TABLE(of, gslx680_of_match);

static struct i2c_driver gslX680_ts_driver = {
	.probe		= gslX680_ts_probe,
	.remove		= __exit_p(gslX680_ts_remove),
	.id_table	= gslX680_ts_id,
	.driver	= {
		.name	= GSLX680_TS_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = gslx680_of_match,
	},
	//.suspend = gslX680_ts_suspend,
	//.resume = gslX680_ts_resume,
};

static int __init gslx860_ts_init(void)
{
	print_info("gslx860_ts_init \n");
	return i2c_add_driver(&gslX680_ts_driver);
}

static void __exit gslx860_ts_exit(void)
{
	i2c_del_driver(&gslX680_ts_driver);
}

module_init(gslx860_ts_init);
module_exit(gslx860_ts_exit);

MODULE_AUTHOR("leweihua");
MODULE_DESCRIPTION("GSLX680 TouchScreen Driver");
MODULE_LICENSE("GPL");






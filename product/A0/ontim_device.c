/*
 *  linux/arch/arm/mach-mmp/ontim.c
 *
 *  Support for the ontim board on Marvell PXA1L88 Helan LTE DKB Development Platform.
 *
 *  Copyright (C) 2012 Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <asm/io.h>
#include <asm/setup.h>
#include <asm/mach/time.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <linux/irqchip/arm-gic.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/localtimer.h>
#ifdef CONFIG_OF
#include <linux/of_platform.h>
#include <linux/clocksource.h>
#include <linux/clk-provider.h>
#endif
#include <soc/sprd/hardware.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <soc/sprd/board.h>
#include <soc/sprd/serial_sprd.h>
#include <soc/sprd/adi.h>
#include <soc/sprd/adc.h>
#include <soc/sprd/pinmap.h>
#include <linux/irq.h>
#include <linux/input/matrix_keypad.h>

#include <soc/sprd/sci.h>
#include <soc/sprd/sci_glb_regs.h>
#include <soc/sprd/hardware.h>
#include <soc/sprd/kpd.h>

#include "devices.h"

#include <linux/regulator/consumer.h>
#include <soc/sprd/regulator.h>
#include <soc/sprd/i2s.h>
#include <linux/sprd_2351.h>


//+add by hzb
//#include <mach/mfp.h>
#include <linux/input.h>
#ifdef CONFIG_ONTIM_BACKLIGHT
#include <ontim/ontim_backlight.h>
#endif
#ifdef CONFIG_TOUCHSCREEN_FOCALTECH_ONTIM 
#include <ontim/touchscreen/focaltech/ft6x06_ts.h>
#endif

#include <ontim/touchscreen/touchpanel.h>

#if defined(CONFIG_LIGHT_PROXIMITY_TMD22713T) ||defined(CONFIG_LIGHT_PROXIMITY_RPR0400) \
   || defined(CONFIG_LIGHT_PROXIMITY_LTR553)
#include <ontim/light_prox/als_ps.h>
#endif

#if defined(CONFIG_LIGHT_PROXIMITY_STK3311)
#include <ontim/light_prox/stk3x1x.h>
#endif

#ifdef CONFIG_GSENSOR_LIS3DH_I2C
#include <ontim/gsensor/lis3dh.h>
#endif

#ifdef CONFIG_GSENSOR_MC3XXX_ONTIM
#include <ontim/gsensor/mc3xxx.h>
#endif

#ifdef CONFIG_GSENSOR_MIR3DA_ONTIM
#include <ontim/gsensor/mir3da_core.h>
#include <ontim/gsensor/mir3da_cust.h>
#endif

#ifdef CONFIG_SENSORS_BMA2X2
#include "ontim/gsensor/bma2x2_bst_sensor_common.h"
#endif

#ifdef CONFIG_GSENSOR_KIONIX
#include <ontim/gsensor/kionix_accel.h>
#endif 

#ifdef CONFIG_INPUT_LSM303D_I2C
#include <ontim/compass/lsm303d.h>
#endif

#ifdef CONFIG_BATTERY_STC3115
#include <ontim/gas-gauge/stc3115_battery.h>
#endif

#ifdef CONFIG_CHARGER_IC_FAN54015
#include <linux/power/ontim_fan54015_charger.h>
#endif

//+add by hzb

#ifndef mfp_to_gpio
#define mfp_to_gpio(m)	((m) % 256)
#endif

#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

#define ONTIM_GPIO_TP_RESET 81
#define ONTIM_GPIO_TP_INT 82
#define ONTIM_GPIO_PA_EN 217
#define ONTIM_GPIO_PROX_IRQ 216
#define ONTIM_GPIO_G_INT	215

//+add by hzb
struct pinmap_t{
    uint32_t reg;
    uint32_t val;
} ;
//-add by hzb

//+add by hzb for check cid
static int recovery_mode;
static int __init recovery_mode_setup(char *str)
{
	int n;
	if (!get_option(&str, &n))
		return 0;
	recovery_mode = n;
	return 1;
}
__setup("recovery=", recovery_mode_setup);

int is_recovery_mode(void)
{
	return recovery_mode;
}
//-add by hzb for check cid


static int boot_bsp=0;
static int __init boot_bsp_setup(char *str)
{
	int n;
	if (!get_option(&str, &n))
		return 0;
	boot_bsp = (u8)n;
	return 1;
}
__setup("androidboot.bsp=", boot_bsp_setup);

u8 is_bsp_mode(void)
{
	return boot_bsp;
}
//-add by hzb
/*+modify by hzb for hw ver detect*/
#define VER_P0 0x00
#define VER_P1 0x01
#define VER_P2 0x02
#define VER_P3 0x03
#define VER_P4 0x04
#define VER_P5 0x05
static u8 board_hw_ver=VER_P0;
#define VER_LTEV20 0xE20
static int board_id= VER_LTEV20;

static int __init board_id_setup(char *str)
{
	int n;
	if (!get_option(&str, &n))
		return 0;
	board_hw_ver = (u8)n;
	board_id = VER_LTEV20;
	return 1;
}
/*-modify by hzb for hw ver detect*/
__setup("board_id=", board_id_setup);
//add by liuwei start
u8 get_hardware_version(void)
{
	return board_hw_ver;
}
//add by liuwei end

int get_board_id(void)
{
	return board_id;
}

/*added by luhongjiang +++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
enum
{
	OTHERS_STATUS	=0x0,
	ONKEY_STATUS	=0x01,
	REBOOT_STATUS	=0x02,
	CHG_STATUS	=0x04,
	BAT_STATUS	=0X08,
	RTC_ALARM_STATUS=0x80,
};

static int bootcause=0;
static int __init bootcause_setup(char *str)
{
	int n;
	char *bootcause_str;
	if (!get_option(&str, &n))
		return 0;
	bootcause = n;

	if(bootcause==RTC_ALARM_STATUS)
		bootcause_str="rtc alarm";
	else
	if(bootcause==CHG_STATUS)
		bootcause_str="charger is plugged";
	else
	if(bootcause==ONKEY_STATUS)
		bootcause_str="onkey is pressed";
	else
		bootcause_str="unknow";

	printk(KERN_INFO "helanlte dkb bootcause: 0x%03x\n", bootcause);	
	printk(KERN_INFO "at %s helanlte dkb bootcause: %s\n",__func__, bootcause_str);		
	return 1;
}
__setup("bootcause=", bootcause_setup);
int is_charger_boot(void)
{
	if(bootcause == CHG_STATUS){
		return 1;
	}
	return 0;
}
EXPORT_SYMBOL(is_charger_boot);
/*------------------------------------------------------------------------------*/

void pinmap_config(struct pinmap_t *pin_cfgs, int num)
{
	int i;
	
	for (i = 0; i < num; i++, pin_cfgs++) 
    {
		pinmap_set(pin_cfgs->reg,pin_cfgs->val);

	}
	return ;
}

static int sensor_set_power(int on)
{
	int ret=0;
	static struct regulator *v_sensor_2v8;
	static struct regulator *v_sensor_1v8;
         printk(KERN_ERR  "at %s line %d enter \n",__func__,__LINE__);
	if (!v_sensor_1v8) {
		v_sensor_1v8 = regulator_get(NULL, "avdd18");
		if (IS_ERR(v_sensor_1v8)) {
			v_sensor_1v8 = NULL;
			return -EIO;
		}
		printk(KERN_INFO "%s:v_sensor_1v8 = %d\n",__func__,on);

		if (on) {
			regulator_set_voltage(v_sensor_1v8, 1800000, 1800000);
			ret |=regulator_enable(v_sensor_1v8);
                           regulator_set_mode(v_sensor_1v8, REGULATOR_MODE_NORMAL);
		} else {
			ret |=regulator_disable(v_sensor_1v8);
		}
		msleep(20);
	}
	if (!v_sensor_2v8) {
		v_sensor_2v8 = regulator_get(NULL, "vdd28");
		if (IS_ERR(v_sensor_2v8)) {
			v_sensor_2v8 = NULL;
			return -EIO;
		}
		printk(KERN_INFO "%s:v_sensor_2v8 =  %d\n",__func__,on);

		if (on) {
			regulator_set_voltage(v_sensor_2v8, 2800000, 2800000);
			ret |=regulator_enable(v_sensor_2v8);
		} else {
			ret |=regulator_disable(v_sensor_2v8);
		}
		msleep(20);
	}
	return 0;
}
	
static int tp_set_power(struct device *dev,int on)
{
	return sensor_set_power(on);
}

#ifdef CONFIG_TOUCHSCREEN_FOCALTECH_ONTIM
static struct pinmap_t ft6x06_pinmap_config[] = {
	{REG_PIN_SIMCLK2 , 	BIT_PIN_SLP_AP|BIT_PIN_NULL|BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE} /* TP Rst pin */
	,{REG_PIN_SIMDA2 ,	BIT_PIN_SLP_AP|BIT_PIN_NULL|BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPU|BIT_PIN_SLP_WPU|BIT_PIN_SLP_Z} /* TP Int pin */
};
static void ft6x06_pin_config( void )
{
	pinmap_config(ARRAY_AND_SIZE(ft6x06_pinmap_config));
	return ;
}
static int ft6x06_reset(struct device *dev)
{
	unsigned int touch_reset;

	touch_reset = mfp_to_gpio(ONTIM_GPIO_TP_RESET);

	if (gpio_request(touch_reset, "ft6x06_reset")) {
		pr_err("Failed to request GPIO for ft6x06_reset pin!\n");
		goto out;
	}

	gpio_direction_output(touch_reset, 1);
	msleep(1);
	gpio_direction_output(touch_reset, 0);
	msleep(5);
	gpio_direction_output(touch_reset, 1);
	msleep(200);
	printk(KERN_INFO "ft6x06_touch reset done.\n");
	gpio_free(touch_reset);
out:
	return 0;
}

static struct touchpanel_virtual_key_data virtual_key_array[]=
{
    {
        .key_code = KEY_APPSELECT,
        .logic_x = 60,
        .logic_y = 900,
        .x_correction = 60,
        .y_correction = 60,
    },
    {
        .key_code = KEY_HOMEPAGE,
        .logic_x = 180,
        .logic_y = 900,
        .x_correction = 60,
        .y_correction = 60,
    },
    {
        .key_code = KEY_BACK,
        .logic_x = 300,
        .logic_y = 900,
        .x_correction = 60,
        .y_correction = 60,
    },
};
#if 0
static unsigned char ft6436_fw_ofilm[]={
    #include <ontim/touchscreen/focaltech/ft6436_A368t_0x51.i>
};
static unsigned char ft6436_fw_yeji[]={
    #include <ontim/touchscreen/focaltech/ft6436_A368t_yeji_0x80.i>
};

static unsigned char ft6206_fw_dht[]={
    #include <ontim/touchscreen/focaltech/ft6206_w230_dht.i>
};

#endif
static struct touchpanel_panel_parm ft6x06_panel[]=
{
	{
		.points = 2,
		.x_max_res = 480,
		.y_max_res = 800,
		.virtual_key=virtual_key_array,
		.virtual_key_num=ARRAY_SIZE(virtual_key_array),
		.vendor_id = 0x11,
        .major_version = 0x11,
		.product_name="000",
		//.panel_fw = ft6206_fw_dht,
		//.panel_fw_size = sizeof(ft6206_fw_dht),
		.panel_ic_id = (FT6x06_UPGRADE_ID_1<<8) |FT6x06_UPGRADE_ID_2,
		.vendor_name="WEIQI-FT6206",
		.freq_hopping_mode=TP_FREQ_HOPPING_MODE_AUTO,
	},
	{
		.points = 2,
		.x_max_res = 480,
		.y_max_res = 800,
		.virtual_key=virtual_key_array,
		.virtual_key_num=ARRAY_SIZE(virtual_key_array),
		.vendor_id = 0xD2,
        .major_version = 0x1,
		.product_name="000",
		//.panel_fw = ft6206_fw_dht,
		//.panel_fw_size = sizeof(ft6206_fw_dht),
		.panel_ic_id = (FT6x36_UPGRADE_ID_1<<8) |FT6x36_UPGRADE_ID_2,
		.vendor_name="HS-FT6236",
		.freq_hopping_mode=TP_FREQ_HOPPING_MODE_AUTO,
	},

};


struct touchpanel_platform_data ft6x06_ts_info = {
    .irqflags=IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
    .panel_parm=&ft6x06_panel[0],
    .panel_parm_num=sizeof(ft6x06_panel)/sizeof(ft6x06_panel[0]),

    .reset_gpio=mfp_to_gpio(ONTIM_GPIO_TP_RESET),
    .irq_gpio = mfp_to_gpio(ONTIM_GPIO_TP_INT),

    .pin_config = ft6x06_pin_config,
    .power_ic = tp_set_power,
    .slot_enable = 1,
    .power_on_when_sleep=1,
//	.reset = ft6x06_reset,
//	.power_i2c = ft6x06_i2c_power,
//	.x_flip		= false,
//	.y_flip		= false,
//	.x_y_exchange   =false,
//	.vdd_name = "v_ldo8",
//	.ignore_product_name = true,
	.prox_enable =1,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_GOODIX
static struct pinmap_t gt9xx_pinmap_config[] = {
	{REG_PIN_SIMCLK2 , 	BIT_PIN_SLP_AP|BIT_PIN_NULL|BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPU|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE} /* TP Rst pin */
	,{REG_PIN_SIMDA2 ,	BIT_PIN_SLP_AP|BIT_PIN_NULL|BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE} /* TP Int pin */
};
static void gt9xx_pin_config( void )
{
	pinmap_config(ARRAY_AND_SIZE(gt9xx_pinmap_config));
	return ;
}

static struct touchpanel_virtual_key_data virtual_key_array_ofilm[]=
{
        {
	     .key_code = KEY_APPSELECT,
	     .logic_x = 56,
	     .logic_y = 900,
            .x_correction = 20,
            .y_correction = 20,
        },
        {
	     .key_code = KEY_HOMEPAGE,
	     .logic_x = 220,
	     .logic_y = 900,
            .x_correction = 20,
            .y_correction = 20,
        },
        {
	     .key_code = KEY_BACK,
	     .logic_x = 423,
	     .logic_y = 900,
            .x_correction = 20,
            .y_correction = 20,
        }
};
#if 0
static unsigned char gt915_cfg_ofilm[]={
	#include <ontim/touchscreen/goodix/gt915_A788t_ofilm.cfg>
};
static unsigned char gt915_fw_ofilm[]={
	#include <ontim/touchscreen/goodix/gt915_A788t_ofilm.i>
};
static unsigned char gt915_cfg_shenyue[]={
	#include <ontim/touchscreen/goodix/gt915_A788t_shenyue.cfg>
};
static unsigned char gt915_fw_shenyue[]={
	#include <ontim/touchscreen/goodix/gt915_A788t_shenyue.i>
};
#endif
static struct touchpanel_panel_parm gt9xx_panel[]=
{
	{
		.x_max_res = 480,
		.y_max_res = 854,
		.virtual_key=virtual_key_array_ofilm,
		.virtual_key_num=ARRAY_SIZE(virtual_key_array_ofilm),
		.vendor_id = 0,
		.product_name="A788",
		//.panal_cfg = gt915_cfg_ofilm,
		//.panal_cfg_size = sizeof(gt915_cfg_ofilm),
		//.panel_fw = gt915_fw_ofilm,
		//.panel_fw_size = sizeof(gt915_fw_ofilm),
	},
};


static struct touchpanel_platform_data gt9xx_ts_info = {
	.reset_gpio=mfp_to_gpio(ONTIM_GPIO_TP_RESET),
	.irq_gpio = mfp_to_gpio(ONTIM_GPIO_TP_INT),
	.panel_parm=&gt9xx_panel[0],
	.panel_parm_num=ARRAY_SIZE(gt9xx_panel),

	.pin_config = gt9xx_pin_config,
       .power_ic = tp_set_power,
       .slot_enable = 1,

};
#endif
#if defined (CONFIG_TOUCHSCREEN_MSTAR) || defined (CONFIG_TOUCHSCREEN_MSTAR2XXX) 
static struct pinmap_t msg2133_pinmap_config[] = {
	{REG_PIN_SIMCLK2 , 	BIT_PIN_SLP_AP|BIT_PIN_NULL|BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE} /* TP Rst pin */
	,{REG_PIN_SIMDA2 ,	BIT_PIN_SLP_AP|BIT_PIN_NULL|BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPU|BIT_PIN_SLP_WPU|BIT_PIN_SLP_Z} /* TP Int pin */
};
static void msg2133_pin_config( void )
{
	pinmap_config(ARRAY_AND_SIZE(msg2133_pinmap_config));
	return ;
}
static int msg2133_reset(struct device *dev)
{
	unsigned int touch_reset;

	touch_reset = mfp_to_gpio(ONTIM_GPIO_TP_RESET);

	if (gpio_request(touch_reset, "msg2133_reset")) {
		pr_err("Failed to request GPIO for msg2133_reset pin!\n");
		goto out;
	}

	gpio_direction_output(touch_reset, 1);
	msleep(1);
	gpio_direction_output(touch_reset, 0);
	msleep(5);
	gpio_direction_output(touch_reset, 1);
	msleep(200);
	printk(KERN_INFO "ft6x06_touch reset done.\n");
	gpio_free(touch_reset);
out:
	return 0;
}


static struct touchpanel_virtual_key_data msg2133_virtual_key_array[]=
{
    {
        .key_code = KEY_APPSELECT,
        .logic_x = 400,
        .logic_y = 860,
        .x_correction = 60,
        .y_correction = 60,
    },
    {
        .key_code = KEY_HOMEPAGE,
        .logic_x = 300,
        .logic_y = 860,
        .x_correction = 60,
        .y_correction = 60,
    },
    {
        .key_code = KEY_BACK,
        .logic_x = 200,
        .logic_y = 860,
        .x_correction = 60,
        .y_correction = 60,
    },
};
#if 0
static unsigned char msg2133_fw_holitech[]={
	#include <ontim/touchscreen/msg21xxA/msg2133A_W230_holitech.i>
};

#endif
static struct touchpanel_panel_parm msg2133_panel[]=
{
	{
		.points = 2,
		.x_max_res = 480,
		.y_max_res = 800,
		.virtual_key=msg2133_virtual_key_array,
		.virtual_key_num=ARRAY_SIZE(msg2133_virtual_key_array),
	    .major_version = 0x01 ,
		.vendor_id = 0x01,
		.product_name="W230",
	//	.panel_fw = msg2133_fw_holitech,
	//	.panel_fw_size = sizeof(msg2133_fw_holitech),
		.vendor_name="HOLITECH-MSG2133",
	},
};


static struct touchpanel_platform_data msg2133_ts_info = {
    .irqflags=IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
    .panel_parm=&msg2133_panel[0],
    .panel_parm_num=sizeof(msg2133_panel)/sizeof(msg2133_panel[0]),

    .reset_gpio=mfp_to_gpio(ONTIM_GPIO_TP_RESET),
    .irq_gpio = mfp_to_gpio(ONTIM_GPIO_TP_INT),

    .pin_config = msg2133_pin_config,
    .power_ic = tp_set_power,
    .slot_enable = 1,
    .power_on_when_sleep=0,
    //	.reset = ft6x06_reset,
    //	.power_i2c = ft6x06_i2c_power,
    //	.x_flip		= false,
    //	.y_flip		= false,
    //	.x_y_exchange   =false,
    //	.vdd_name = "v_ldo8",
    //	.ignore_product_name = true,
};
#endif

void Audio_PA_Enable(int v_Enable)
{
	gpio_request(mfp_to_gpio(ONTIM_GPIO_PA_EN), "Audio_PA_EN");

	printk(KERN_INFO "%s: v_Enable = %d\n",__func__,v_Enable);
	if (v_Enable)
	{
		gpio_direction_output(mfp_to_gpio(ONTIM_GPIO_PA_EN), 1);
	}
	else
	{
		gpio_direction_output(mfp_to_gpio(ONTIM_GPIO_PA_EN), 0);
	}
	gpio_free(mfp_to_gpio(ONTIM_GPIO_PA_EN));
	
	return ;
}

//-add by hzb

//+add by hzb
#if defined(CONFIG_LIGHT_PROXIMITY_TMD22713T)
static struct als_ps_platform_data comip_i2c_tmd22713_info = {
	.calibrate_target = 300000,
	.als_time = 50,//200,
	.scale_factor = 1,
	.gain_trim = 512,
	.filter_history = 3,
	.filter_count = 1,
	.gain = 1,//2,
	.prox_threshold_hi = 700,//500,//120,
	.prox_threshold_lo = 400,//400,//80,
	.als_threshold_hi = 3000,
	.als_threshold_lo = 10,
	.prox_int_time = 0xee, /* 50ms */
	.prox_adc_time = 0xff,
	.prox_wait_time = 0xee,
	.prox_intr_filter = 0x23,//0x11,//0x23,
	.prox_config = 0,
	.prox_pulse_cnt = 0x04,//0x08,
	.prox_gain = 0x21,//0x61,//0x62,
    .int_gpio = mfp_to_gpio(ONTIM_GPIO_PROX_IRQ),
	.power_on=sensor_set_power,
};
#endif

#if defined(CONFIG_LIGHT_PROXIMITY_RPR0400)
static struct als_ps_platform_data comip_i2c_rpr0400_info = {
	.prox_threshold_hi = 60,
	.prox_threshold_lo = 50,
    .int_gpio = mfp_to_gpio(ONTIM_GPIO_PROX_IRQ),
	.power_on=sensor_set_power,
};
#endif

#if defined(CONFIG_LIGHT_PROXIMITY_LTR553)
static struct als_ps_platform_data comip_i2c_ltr553_info = {
	.prox_threshold_hi = 75,
	.prox_threshold_lo = 40,
    .int_gpio = mfp_to_gpio(ONTIM_GPIO_PROX_IRQ),
	.power_on=sensor_set_power,
};
#endif

#if defined(CONFIG_LIGHT_PROXIMITY_STK3311)
static struct stk3x1x_platform_data comip_i2c_stk3311_info = {

         .state_reg = 0x0,/* disable all */
         .psctrl_reg = 0x31, //0x71/* ps_persistance=4, ps_gain=64X, PS_IT=0.391ms */
         .alsctrl_reg = 0x38,/* als_persistance=1, als_gain=64X, ALS_IT=50ms */
         .ledctrl_reg = 0xFF,/* 100mA IRDR, 64/64 LED duty */
         .wait_reg = 0x07,/* 50 ms */
         .ps_thd_h =1400,
         .ps_thd_l = 1200,
         .int_pin = mfp_to_gpio(ONTIM_GPIO_PROX_IRQ),//GPIO_PROX_INT,
         .transmittance = 500,
            
	.power_on=sensor_set_power,
	//.prox_threshold_hi = 75,
	//.prox_threshold_lo = 40,
         //.int_gpio = mfp_to_gpio(ONTIM_GPIO_PROX_IRQ),
};

#endif

#ifdef CONFIG_GSENSOR_LIS3DH_I2C
static struct lis3dh_acc_platform_data lis3dh_plat_data = {
	.power_on=sensor_set_power,
	.poll_interval = 10,
	.min_interval = 10,
	.g_range = LIS3DH_ACC_G_2G,
	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 1,
	.negate_z = 0
};
#endif

#ifdef CONFIG_GSENSOR_MC3XXX_ONTIM 
static struct mc3xxx_platform_data mc3xxx_plat_data = {
	.power_on=sensor_set_power,
	.current_placement=MC3XXX_TOP_LEFT_DOWN ,
};
#endif

#ifdef CONFIG_GSENSOR_MIR3DA_ONTIM 
static struct mir3da_platform_data mir3da_plat_data = {
	.power_on=sensor_set_power,
	.direction_placement=MIR3DA_DIRECTION_NUM ,
};
#endif

#ifdef CONFIG_SENSORS_BMA2X2
static int bma2x2_irq_gpio_cfg(void* data,int req)
{
	struct bosch_sensor_specific * pdata=data;
	
	//unsigned long gpio_cfg = MFP_PULL_LOW|MFP_LPM_DRIVE_LOW;
	
	if (pdata)
	{
		if (req)
		{
			if (pdata->irq_gpio_fst>=0)
			{
				if (gpio_request(pdata->irq_gpio_fst, "BMA2x2")) {
					pr_err("gpio %d request failed\n", pdata->irq_gpio_fst);
					return -1;
				}
				//gpio_cfg = pdata->irq_gpio_fst |MFP_PULL_LOW |MFP_LPM_DRIVE_LOW;
				//mfp_config(&gpio_cfg,1);
				gpio_direction_input(pdata->irq_gpio_fst);
				if (pdata->irq_use_num==1)
				{
					pdata->irq=gpio_to_irq(pdata->irq_gpio_fst);
				}
			}
			if (pdata->irq_gpio_sec>=0)
			{
				if (gpio_request(pdata->irq_gpio_fst, "BMA2x2")) {
					pr_err("gpio %d request failed\n", pdata->irq_gpio_fst);
					if (pdata->irq_gpio_fst>=0)
					{
						gpio_free(pdata->irq_gpio_fst);
					}
					return -1;
				}
				//gpio_cfg = pdata->irq_gpio_sec |MFP_PULL_LOW |MFP_LPM_DRIVE_LOW;
				//mfp_config(&gpio_cfg,1);
				gpio_direction_input(pdata->irq_gpio_sec);
				if (pdata->irq_use_num==2)
				{
					pdata->irq=gpio_to_irq(pdata->irq_gpio_sec);
				}
			}
		}
		else
		{
			if (pdata->irq_gpio_fst>=0)
			{
				gpio_free(pdata->irq_gpio_fst);
			}
			if (pdata->irq_gpio_sec>=0)
			{
				gpio_free(pdata->irq_gpio_sec);
			}
		}
	}
	return 0;
};

static struct bosch_sensor_specific bma2x2_plat_data = {
	.name = "BMA2x2",
	.place = 2,
	.irq_gpio_fst=mfp_to_gpio(ONTIM_GPIO_G_INT),
	.irq_gpio_sec=-1,
	.irq_use_num=-1,
	.irq=-1,
	.irq_gpio_cfg=bma2x2_irq_gpio_cfg,
	.power_on=sensor_set_power,
};
#endif

#ifdef CONFIG_INPUT_LSM303D_I2C

static const struct lsm303d_acc_platform_data lsm303d_acc_pdata = {
	.fs_range = LSM303D_ACC_FS_2G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
	.poll_interval = 100,
	.min_interval = LSM303D_ACC_MIN_POLL_PERIOD_MS,
	.aa_filter_bandwidth = ANTI_ALIASING_773,
	.gpio_int1 = DEFAULT_INT1_GPIO,
	.gpio_int2 = DEFAULT_INT2_GPIO,
};

static const struct lsm303d_mag_platform_data lsm303d_mag_pdata = {
	.poll_interval = 100,
	.min_interval = LSM303D_MAG_MIN_POLL_PERIOD_MS,
	.fs_range = LSM303D_MAG_FS_2G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
};

static const struct lsm303d_main_platform_data lsm303d_pdata = {
	.pdata_acc=&lsm303d_acc_pdata,
	.pdata_mag=&lsm303d_mag_pdata,
	.power_on=sensor_set_power,
};

#endif

#ifdef CONFIG_GSENSOR_KIONIX
static int kionix_irq_gpio_cfg(void* data,int req)
{
	struct kionix_accel_platform_data * pdata=data;
	
	//unsigned long gpio_cfg = MFP_PULL_LOW|MFP_LPM_DRIVE_LOW;
	
	if (pdata)
	{
		if (req)
		{
			if (gpio_request(ONTIM_GPIO_G_INT, "kionix")) {
				pr_err("gpio %d request failed\n", ONTIM_GPIO_G_INT);
				return -1;
			}
			//gpio_cfg = ONTIM_GPIO_G_INT |MFP_PULL_LOW |MFP_LPM_DRIVE_LOW;
			//mfp_config(&gpio_cfg,1);
			gpio_direction_input(ONTIM_GPIO_G_INT);
			pdata->irq=gpio_to_irq(ONTIM_GPIO_G_INT);
		}
		else
		{
			gpio_free(ONTIM_GPIO_G_INT);
		}
	}
	return 0;
};

struct kionix_accel_platform_data kionix_accel_pdata = {
	.min_interval = 5,
	.poll_interval = 200,
	.accel_direction = 2,
	.accel_irq_use_drdy = 0,
	.accel_res = KIONIX_ACCEL_RES_12BIT,
	.accel_g_range = KIONIX_ACCEL_G_2G,
	.power_on=sensor_set_power,
	.irq=-1,
	.irq_gpio_cfg=kionix_irq_gpio_cfg,
};
#endif
//-add by hzb

#if 0
static struct ontim_bl_platform_data ontim_lcd_backlight_data = {
		.name = "lcd-bl",
		.max_intensity = 255,
		.default_intensity = 100,
		.ctrl_type=ONTIM_BACKLIGHT_CTRL_LCDC,
		.bl_gpio=mfp_to_gpio(MFP_PIN_GPIO32),
};

static struct platform_device ontim_lcd_backlight_devices = {
	.name = "ontim_backlight",
	.id = 0,
	.dev = {
		.platform_data = &ontim_lcd_backlight_data,
	},
};

static struct ontim_bl_platform_data ontim_key_backlight_data = {
		.name = "key-bl",
		.max_intensity = 255,
		.default_intensity = 0,
		.ctrl_type=ONTIM_BACKLIGHT_CTRL_GPIO_ONOFF,
		.bl_gpio=mfp_to_gpio(MFP_PIN_GPIO127),
};

static struct platform_device ontim_key_backlight_devices = {
	.name = "ontim_backlight",
	.id = 1,
	.dev = {
		.platform_data = &ontim_key_backlight_data,
	},
};



static struct platform_device *ontim_backlight_devices[]=
{
	&ontim_lcd_backlight_devices,
	&ontim_key_backlight_devices,
};
#endif

//TP on i2c-1
static struct i2c_board_info ontim_i2c1_devs[] __initdata = {
#ifdef CONFIG_TOUCHSCREEN_GOODIX 
	{
		I2C_BOARD_INFO("Goodix-TS", 0x5D),
		.platform_data = &gt9xx_ts_info,
	},
#endif
#ifdef CONFIG_TOUCHSCREEN_FOCALTECH_ONTIM 
	{
		I2C_BOARD_INFO(FT6X06_NAME, FT6x06_TS_ADDR),
		.platform_data = &ft6x06_ts_info,
	},
#endif
#if defined (CONFIG_TOUCHSCREEN_MSTAR) || defined (CONFIG_TOUCHSCREEN_MSTAR2XXX) 
	{
		I2C_BOARD_INFO("MSG2XXX", 0x26),
		.platform_data = &msg2133_ts_info,
	},
#endif
};

//sensor on i2c-2
static struct i2c_board_info ontim_i2c2_devs[] __initdata = {

//+add by hzb
#if defined(CONFIG_LIGHT_PROXIMITY_TMD22713T) ||defined(CONFIG_LIGHT_PROXIMITY_RPR0400)
	{
		.type = "als_prox",
		.addr = 0x39,
	//	.irq = gpio_to_irq(ONTIM_GPIO_PROX_IRQ), // Replace with appropriate GPIO setup
#if defined(CONFIG_LIGHT_PROXIMITY_TMD22713T)
		.platform_data = &comip_i2c_tmd22713_info,
#elif defined(CONFIG_LIGHT_PROXIMITY_RPR0400)
		.platform_data = &comip_i2c_rpr0400_info,
#endif
	},
#endif
#if defined(CONFIG_LIGHT_PROXIMITY_LTR553)
	{ I2C_BOARD_INFO("ltr553", 0x23),
		.platform_data = &comip_i2c_ltr553_info,
	},
#endif
#if defined(CONFIG_LIGHT_PROXIMITY_STK3311)
	{ I2C_BOARD_INFO("stk_ps", 0x48),
		.platform_data = &comip_i2c_stk3311_info,
	},
#endif
#ifdef CONFIG_GSENSOR_LIS3DH_I2C
	{ I2C_BOARD_INFO(LIS3DH_ACC_I2C_NAME, LIS3DH_ACC_I2C_ADDR),
	  .platform_data = &lis3dh_plat_data,
	},
#endif
#ifdef CONFIG_INPUT_LSM303D_I2C
	{ I2C_BOARD_INFO(LSM303D_I2C_NAME, LSM303D_I2C_ADDR),
	  .platform_data = &lsm303d_pdata,
	},
#endif
#ifdef CONFIG_SENSORS_BMA2X2
	{ I2C_BOARD_INFO("bma2x2", 0x18),
	  .platform_data = &bma2x2_plat_data,
	},
#endif
#ifdef CONFIG_GSENSOR_KIONIX
	{ 
		I2C_BOARD_INFO(KIONIX_ACCEL_NAME, KIONIX_ACCEL_I2C_ADDR),
		.platform_data = &kionix_accel_pdata,
	},
#endif
#ifdef CONFIG_GSENSOR_MC3XXX_ONTIM
	{ I2C_BOARD_INFO(MC3XXX_ACC_I2C_NAME, MC3XXX_ACC_I2C_ADDR),
	  .platform_data = &mc3xxx_plat_data ,
	},
#endif
#ifdef CONFIG_GSENSOR_MIR3DA_ONTIM
	{ I2C_BOARD_INFO(MIR3DA_DRV_NAME, MIR3DA_I2C_ADDR),
	  .platform_data = &mir3da_plat_data ,
	},

#endif
//-add by hzb
};

//camera on i2c-0
static struct i2c_board_info ontim_i2c0_devs[] __initdata = {
};

void __init ontim_init_i2c_dev(void)
{
	if (ARRAY_SIZE(ontim_i2c0_devs))	
	{
		i2c_register_board_info(0, ontim_i2c0_devs,
				ARRAY_SIZE(ontim_i2c0_devs));
	}
	if (ARRAY_SIZE(ontim_i2c1_devs))	
	{
		i2c_register_board_info(1, ontim_i2c1_devs,
				ARRAY_SIZE(ontim_i2c1_devs));
	}
	if (ARRAY_SIZE(ontim_i2c2_devs))	
	{
		i2c_register_board_info(2, ontim_i2c2_devs,
				ARRAY_SIZE(ontim_i2c2_devs));
	}
}

void __init ontim_init_machine(void)
{
	printk(KERN_ERR "%s\n",__func__);	

	ontim_init_i2c_dev();
}



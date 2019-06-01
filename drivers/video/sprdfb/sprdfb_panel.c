/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/fb.h>

#include "sprdfb.h"
#include "sprdfb_panel.h"
#include "sprdfb_dispc_reg.h"

#include <linux/init.h>
#include <linux/io.h>
#include <soc/sprd/pinmap.h>
#include <soc/sprd/hardware.h>
#include <linux/gpio.h>
#include <linux/string.h>   //add by hzb

/*added by luhongjiang ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
enum 
{
   LCD_RSTN_MODE_RSTN,
   LCD_RSTN_MODE_GPIO,
};

typedef struct {
	uint32_t reg;
	uint32_t val;
} pinmap_t;  

 
static pinmap_t  LCD_RSTN_mode[] = {
    {REG_PIN_LCD_RSTN,            BITS_PIN_DS(1)|BITS_PIN_AF(0)|BIT_PIN_NUL|BIT_PIN_SLP_WPU|BIT_PIN_SLP_Z},
    {REG_PIN_LCD_RSTN,            BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_WPU|BIT_PIN_SLP_Z},
};
/*--------------------------------------------------------------------------------------------------------------------*/
struct panel_spec lcd_detect_mipi_info;//add by liuwei

static LIST_HEAD(panel_list_main);/* for main_lcd*/
static LIST_HEAD(panel_list_sub);/* for sub_lcd */
static DEFINE_MUTEX(panel_mutex);

static uint32_t lcd_id_from_uboot = 0;
unsigned long lcd_base_from_uboot = 0;
static uint32_t lcd_adc_from_uboot = 0;//+modify by liuwei
extern struct panel_if_ctrl sprdfb_mcu_ctrl;
extern struct panel_if_ctrl sprdfb_rgb_ctrl;
#ifndef CONFIG_FB_SCX15
extern struct panel_if_ctrl sprdfb_mipi_ctrl;
#endif
extern void sprdfb_panel_remove(struct sprdfb_device *dev);

int lcd_panel_cabc_pwm_backlight = 0;
static int __init lcd_id_get(char *str)
{
	if ((str != NULL) && (str[0] == 'I') && (str[1] == 'D')) {
		sscanf(&str[2], "%x", &lcd_id_from_uboot);
	}
#if 0
	if (lcd_id_from_uboot == 0x8369) {
		lcd_panel_cabc_pwm_backlight = 1;
	}
#endif
	pr_info("lcd_panel_cabc_pwm_backlight = %d\n", lcd_panel_cabc_pwm_backlight);
	printk(KERN_INFO "sprdfb: [%s]LCD Panel ID from uboot: 0x%x\n", __FUNCTION__, lcd_id_from_uboot);
	return 1;
}
__setup("lcd_id=", lcd_id_get);
static int __init lcd_base_get(char *str)
{
	if (str != NULL) {
		sscanf(&str[0], "%lx", &lcd_base_from_uboot);
	}
	printk(KERN_INFO "sprdfb: [%s]LCD Panel Base from uboot: 0x%lx\n", __FUNCTION__, lcd_base_from_uboot);
	return 1;
}
__setup("lcd_base=", lcd_base_get);
//+modify by liuwei
static int __init lcd_adc_get(char *str)
{
	if ((str != NULL) && (str[0] == 'A') && (str[1] == 'D')&& (str[2] == 'C')) {
		sscanf(&str[3], "%d", &lcd_adc_from_uboot);
        printk("[kernel][lcd_adc get cmd = %s] \n",str);
	}
	printk(KERN_INFO "LCD Panel ADC from uboot: 0x%x %s\n", lcd_adc_from_uboot,str);
	return 1;
}
__setup("lcd_adc=", lcd_adc_get);
//-modify by liuwei
//+modify by hzb
char lcd_name_from_uboot[50] = {'\0'};

static int __init lcd_name_get(char *str)
{
	sscanf(&str[0], "%s", &lcd_name_from_uboot);
	printk(KERN_INFO "LCD Panel name from uboot: 0x%s\n", lcd_name_from_uboot);
	return 1;
}
__setup("lcd_name=", lcd_name_get);
//-modify by hzb
int lcd_find_success;
static int __init lcd_find_flag_get(char *str)
{
	if ((str != NULL) && (str[0] == 'F') && (str[1] == 'L')&& (str[2] == 'A')&& (str[3] == 'G')) {
		sscanf(&str[4], "%d", &lcd_find_success);
        printk("[kernel][lcd_find_flag_get get cmd = %s \n",str);
	}
	printk(KERN_INFO "LCD Panel find flag from uboot: %d %s\n", lcd_find_success,str);
	return 1;
}
__setup("lcd_flag=", lcd_find_flag_get);
static int32_t panel_reset_dispc(struct panel_spec *self)
{
        uint16_t timing1, timing2, timing3;

        if((NULL != self) && (0 != self->reset_timing.time1) &&
            (0 != self->reset_timing.time2) && (0 != self->reset_timing.time3)) {
            timing1 = self->reset_timing.time1;
            timing2 = self->reset_timing.time2;
            timing3 = self->reset_timing.time3;
        }else {
            timing1 = 20;
            timing2 = 20;
            timing3 = 120;
        }

	dispc_write(1, DISPC_RSTN);
	usleep_range(timing1*1000, timing1*1000+500);
	dispc_write(0, DISPC_RSTN);
	usleep_range(timing2*1000, timing2*1000+500);
	dispc_write(1, DISPC_RSTN);

	/* wait 10ms util the lcd is stable */
	usleep_range(timing3*1000, timing3*1000+500);

	return 0;
}

static int32_t panel_set_resetpin_dispc( uint32_t status)
{
	if(0 == status){
		dispc_write(0, DISPC_RSTN);
	}else{
		dispc_write(1, DISPC_RSTN);
	}
	return 0;
}

static int panel_reset(struct sprdfb_device *dev)
{
	if((NULL == dev) || (NULL == dev->panel)){
		printk(KERN_ERR "sprdfb: [%s]: Invalid param\n", __FUNCTION__);
		return -1;
	}

	pr_debug("sprdfb: [%s], enter\n",__FUNCTION__);

	//clk/data lane enter LP
	if(NULL != dev->panel->if_ctrl->panel_if_before_panel_reset){
		dev->panel->if_ctrl->panel_if_before_panel_reset(dev);
	}
	usleep_range(5000, 5500);

	//reset panel
	dev->panel->ops->panel_reset(dev->panel);

	return 0;
}

static int panel_sleep(struct sprdfb_device *dev)
{
	if((NULL == dev) || (NULL == dev->panel)){
		printk(KERN_ERR "sprdfb: [%s]: Invalid param\n", __FUNCTION__);
		return -1;
	}

	pr_debug("sprdfb: [%s], enter\n",__FUNCTION__);

	//send sleep cmd to lcd
	if (dev->panel->ops->panel_enter_sleep != NULL) {
		dev->panel->ops->panel_enter_sleep(dev->panel,1);
	}
	msleep(100);
	//clk/data lane enter LP
	if((NULL != dev->panel->if_ctrl->panel_if_before_panel_reset)
		&&(SPRDFB_PANEL_TYPE_MIPI == dev->panel->type))
	{
		dev->panel->if_ctrl->panel_if_before_panel_reset(dev);
	}
	return 0;
}

static void panel_set_resetpin(uint16_t dev_id,  uint32_t status, struct panel_spec *panel )
{
	pr_debug("sprdfb: [%s].\n",__FUNCTION__);

	/*panel set reset pin status*/
	if(SPRDFB_MAINLCD_ID == dev_id){
		panel_set_resetpin_dispc(status);
	}
}


static int32_t panel_before_resume(struct sprdfb_device *dev)
{
	/*restore  the reset pin to high*/
	panel_set_resetpin(dev->dev_id, 1, dev->panel);
	return 0;
}

static int32_t panel_after_suspend(struct sprdfb_device *dev)
{
	/*set the reset pin to low*/
	panel_set_resetpin(dev->dev_id, 0, dev->panel);
	return 0;
}

static bool panel_check(struct panel_cfg *cfg)
{
	bool rval = true;

	if(NULL == cfg || NULL == cfg->panel){
		printk(KERN_ERR "sprdfb: [%s] :Invalid Param!\n", __FUNCTION__);
		return false;
	}

	pr_debug("sprdfb: [%s], dev_id = %d, lcd_id = 0x%x, type = %d\n",__FUNCTION__, cfg->dev_id, cfg->lcd_id, cfg->panel->type);

	switch(cfg->panel->type){
	case SPRDFB_PANEL_TYPE_MCU:
		cfg->panel->if_ctrl = &sprdfb_mcu_ctrl;
		break;
	case SPRDFB_PANEL_TYPE_RGB:
	case SPRDFB_PANEL_TYPE_LVDS:
		cfg->panel->if_ctrl = &sprdfb_rgb_ctrl;
		break;
#ifndef CONFIG_FB_SCX15
	case SPRDFB_PANEL_TYPE_MIPI:
		cfg->panel->if_ctrl = &sprdfb_mipi_ctrl;
		break;
#endif
	default:
		printk("sprdfb: [%s]: erro panel type.(%d,%d, %d)",__FUNCTION__, cfg->dev_id, cfg->lcd_id, cfg->panel->type);
		cfg->panel->if_ctrl = NULL;
		break;
	};

	if(cfg->panel->if_ctrl->panel_if_check){
		rval = cfg->panel->if_ctrl->panel_if_check(cfg->panel);
	}
	return rval;
}

static int panel_mount(struct sprdfb_device *dev, struct panel_spec *panel)
{
	printk("sprdfb: [%s], dev_id = %d\n",__FUNCTION__, dev->dev_id);

	/* TODO: check whether the mode/res are supported */
	dev->panel = panel;

	if(NULL == dev->panel->ops->panel_reset){
		if(SPRDFB_MAINLCD_ID == dev->dev_id){
			dev->panel->ops->panel_reset = panel_reset_dispc;
		}
	}

	panel->if_ctrl->panel_if_mount(dev);

	return 0;
}


int panel_init(struct sprdfb_device *dev)
{
	if((NULL == dev) || (NULL == dev->panel)){
		printk(KERN_ERR "sprdfb: [%s]: Invalid param\n", __FUNCTION__);
		return -1;
	}

	pr_debug("sprdfb: [%s], dev_id= %d, type = %d\n",__FUNCTION__, dev->dev_id, dev->panel->type);

	if(!dev->panel->if_ctrl->panel_if_init(dev)){
		printk(KERN_ERR "sprdfb: [%s]: panel_if_init fail!\n", __FUNCTION__);
		return -1;
	}

	return 0;
}

int panel_ready(struct sprdfb_device *dev)
{
	if((NULL == dev) || (NULL == dev->panel)){
		printk(KERN_ERR "sprdfb: [%s]: Invalid param\n", __FUNCTION__);
		return -1;
	}

	pr_debug("sprdfb: [%s], dev_id= %d, type = %d\n",__FUNCTION__, dev->dev_id, dev->panel->type);

	if(NULL != dev->panel->if_ctrl->panel_if_ready){
		dev->panel->if_ctrl->panel_if_ready(dev);
	}

	return 0;
}

#if 0
static struct panel_spec *adapt_panel_from_uboot(uint16_t dev_id)
{
	struct panel_cfg *cfg;
	struct list_head *panel_list;

	pr_debug("sprdfb: [%s], dev_id = %d\n",__FUNCTION__, dev_id);

	if (lcd_id_from_uboot == 0) {
		printk("sprdfb: [%s]: Not got lcd id from uboot\n", __FUNCTION__);
		return NULL;
	}

	if(SPRDFB_MAINLCD_ID == dev_id){
		panel_list = &panel_list_main;
	}else{
		panel_list = &panel_list_sub;
	}

	list_for_each_entry(cfg, panel_list, list) {
		if(lcd_id_from_uboot == cfg->lcd_id) {
			printk(KERN_INFO "sprdfb: [%s]: LCD Panel 0x%x is attached!\n", __FUNCTION__,cfg->lcd_id);
			return cfg->panel;
		}
	}
	printk(KERN_ERR "sprdfb: [%s]: Failed to match LCD Panel from uboot!\n", __FUNCTION__);

	return NULL;
}
#endif
static struct panel_spec *adapt_panel_from_uboot(struct sprdfb_device *dev)
{
    u8 i=0;
    struct panel_cfg *cfg;
    struct list_head *panel_list = &panel_list_main;

    pr_debug("sprdfb:entry  [%s]\n",__FUNCTION__);

    if (strlen(lcd_name_from_uboot) == 0) {
        printk("sprdfb: [%s]: Not got lcd name from uboot\n", __FUNCTION__);
    }
    else
    {
        dev->lcd_name = lcd_name_from_uboot;

        list_for_each_entry(cfg, panel_list, list) 
        {
            if((memcmp(lcd_name_from_uboot,cfg->lcd_name,strlen(lcd_name_from_uboot)) ==0)&&
               (strlen(lcd_name_from_uboot) == strlen( cfg->lcd_name)))
            {
                printk(KERN_INFO "kernel: [%s]: LCD Panel %s is attached!,lcd_name_from_uboot=%s\n", 
                       __FUNCTION__,cfg->lcd_name,lcd_name_from_uboot);
                return cfg->panel;
            }
        }
    }	

    list_for_each_entry(cfg, panel_list, list) 
    {
        printk("[kernel]%s min %d  max %d \n",cfg->lcd_name,cfg->lcd_voltage_min,cfg->lcd_voltage_max);
        if((lcd_adc_from_uboot>= cfg->lcd_voltage_min)&&(lcd_adc_from_uboot< cfg->lcd_voltage_max))
        {
            memcpy(lcd_name_from_uboot,cfg->lcd_name,strlen(cfg->lcd_name));
            printk(KERN_INFO "kernel: [%s]:** LCD Panel %s is attached!,lcd_adc_from_uboot=%d.\n", 
                   __FUNCTION__,cfg->lcd_name,lcd_adc_from_uboot);
            memcpy(&lcd_detect_mipi_info,cfg->panel,sizeof(struct panel_spec));
            return cfg->panel;
        }
    }

    cfg = list_entry( (panel_list)->next,typeof(*cfg),list) ;
    printk(KERN_ERR "kernel: [%s]: Failed to match LCD,use default,lcd_name=%s,adc=%d\n", 
           __FUNCTION__,cfg->lcd_name,lcd_adc_from_uboot);
    memcpy(lcd_name_from_uboot,cfg->lcd_name,strlen(cfg->lcd_name));
    memcpy(&lcd_detect_mipi_info,cfg->panel,sizeof(struct panel_spec));
    return cfg->panel;
}

static struct panel_spec *adapt_panel_from_readid(struct sprdfb_device *dev)
{
	struct panel_cfg *cfg;
	struct panel_cfg *dummy_cfg = NULL;
	struct list_head *panel_list;
	uint32_t id;

	printk("sprdfb: [%s], dev_id = %d\n",__FUNCTION__, dev->dev_id);

	if(SPRDFB_MAINLCD_ID == dev->dev_id){
		panel_list = &panel_list_main;
	}else{
		panel_list = &panel_list_sub;
	}

	list_for_each_entry(cfg, panel_list, list) {
		if(0xFFFFFFFF == cfg->lcd_id){
			dummy_cfg = cfg;
			continue;
		}
		printk("sprdfb: [%s]: try panel 0x%x\n", __FUNCTION__, cfg->lcd_id);
		panel_mount(dev, cfg->panel);
#ifndef CONFIG_SC_FPGA
		dev->ctrl->update_clk(dev);
#endif
		panel_init(dev);
		panel_reset(dev);
		id = dev->panel->ops->panel_readid(dev->panel);
		if(id == cfg->lcd_id) {
			pr_debug(KERN_INFO "sprdfb: [%s]: LCD Panel 0x%x is attached!\n", __FUNCTION__, cfg->lcd_id);
			dev->panel->ops->panel_init(dev->panel);
			panel_ready(dev);
			return cfg->panel;
		}
		sprdfb_panel_remove(dev);
	}
	if(dummy_cfg != NULL){
		printk("sprdfb: [%s]: Can't find read panel, Use dummy panel!\n", __FUNCTION__);
		panel_mount(dev, dummy_cfg->panel);
#ifndef CONFIG_SC_FPGA
		dev->ctrl->update_clk(dev);
#endif
		panel_init(dev);
		panel_reset(dev);
		id = dev->panel->ops->panel_readid(dev->panel);
		if(id == dummy_cfg->lcd_id) {
			pr_debug(KERN_INFO "sprdfb: [%s]: LCD Panel 0x%x is attached!\n", __FUNCTION__, dummy_cfg->lcd_id);
			dev->panel->ops->panel_init(dev->panel);
			panel_ready(dev);
			return dummy_cfg->panel;
		}
		sprdfb_panel_remove(dev);
	}
	printk(KERN_ERR "sprdfb:  [%s]: failed to attach LCD Panel!\n", __FUNCTION__);
	return NULL;
}

bool sprdfb_panel_get(struct sprdfb_device *dev)
{
	struct panel_spec *panel = NULL;

	if(NULL == dev){
		printk("sprdfb: [%s]: Invalid param\n", __FUNCTION__);
		return false;
	}

	printk("sprdfb: [%s], dev_id = %d\n",__FUNCTION__, dev->dev_id);

	//panel = adapt_panel_from_uboot(dev->dev_id);
	panel = adapt_panel_from_uboot(dev);//+modify by liuwei
	if (panel) {
		dev->panel_ready = true;
		panel_mount(dev, panel);
		panel_init(dev);
		printk("sprdfb: [%s] got panel\n", __FUNCTION__);
		return true;
	}

	printk("sprdfb: [%s] can not got panel\n", __FUNCTION__);

	return false;
}


bool sprdfb_panel_probe(struct sprdfb_device *dev)
{
	struct panel_spec *panel;

	if(NULL == dev){
		printk("sprdfb: [%s]: Invalid param\n", __FUNCTION__);
		return false;
	}

	printk("sprdfb: [%s], dev_id = %d\n",__FUNCTION__, dev->dev_id);

	/* can not be here in normal; we should get correct device id from uboot */
	panel = adapt_panel_from_readid(dev);

	if (panel) {
		printk("sprdfb: [%s] got panel\n", __FUNCTION__);
		return true;
	}

	printk("sprdfb: [%s] can not got panel\n", __FUNCTION__);

	return false;
}

void sprdfb_panel_invalidate_rect(struct panel_spec *self,
				uint16_t left, uint16_t top,
				uint16_t right, uint16_t bottom)
{
	/*Jessica TODO: */
	if(NULL != self->ops->panel_invalidate_rect){
		self->ops->panel_invalidate_rect(self, left, top, right, bottom);
	}
	/*Jessica TODO: Need set timing to GRAM timing*/
}

void sprdfb_panel_invalidate(struct panel_spec *self)
{
	/*Jessica TODO:*/
	if(NULL != self->ops->panel_invalidate){
		self->ops->panel_invalidate(self);
	}
	/*Jessica TODO: Need set timing to GRAM timing*/
}

void sprdfb_panel_before_refresh(struct sprdfb_device *dev)
{
	if(NULL != dev->panel->if_ctrl->panel_if_before_refresh){
		dev->panel->if_ctrl->panel_if_before_refresh(dev);
	}
}

void sprdfb_panel_after_refresh(struct sprdfb_device *dev)
{
	if(NULL != dev->panel->if_ctrl->panel_if_after_refresh){
		dev->panel->if_ctrl->panel_if_after_refresh(dev);
	}
}

#ifdef CONFIG_FB_DYNAMIC_FREQ_SCALING
void sprdfb_panel_change_fps(struct sprdfb_device *dev, int fps_level)
{
	if (dev->panel->ops->panel_change_fps!= NULL) {
		printk("sprdfb: [%s] fps_level= %d\n", __FUNCTION__,fps_level);
		dev->panel->ops->panel_change_fps(dev->panel,fps_level);
	}
}
#endif

#ifdef CONFIG_FB_ESD_SUPPORT
/*return value:  0--panel OK.1-panel has been reset*/
uint32_t sprdfb_panel_ESD_check(struct sprdfb_device *dev)
{
	int32_t result = 0;
	uint32_t if_status = 0;

//	printk("sprdfb: [%s] (%d, %d, %d)\n",__FUNCTION__, dev->check_esd_time, dev->panel_reset_time, dev->reset_dsi_time);

	dev->check_esd_time++;

	if(SPRDFB_PANEL_IF_EDPI == dev->panel_if_type){
		if (dev->panel->ops->panel_esd_check != NULL) {
			result = dev->panel->ops->panel_esd_check(dev->panel);
			pr_debug("sprdfb: [%s] panel check return %d\n", __FUNCTION__, result);
		}
	}else if(SPRDFB_PANEL_IF_DPI == dev->panel_if_type){
#ifdef FB_CHECK_ESD_BY_TE_SUPPORT
		dev->esd_te_waiter++;
		dev->esd_te_done = 0;
		dispc_set_bits(BIT(1), DISPC_INT_EN);
		result  = wait_event_interruptible_timeout(dev->esd_te_queue,
			          dev->esd_te_done, msecs_to_jiffies(600));
		pr_debug("sprdfb: after wait (%d)\n", result);
		dispc_clear_bits(BIT(1), DISPC_INT_EN);
		if(!result){ /*time out*/
			printk("sprdfb: [%s] esd check  not got te signal!!!!\n", __FUNCTION__);
			dev->esd_te_waiter = 0;
			result = 0;
		}else{
			pr_debug("sprdfb: [%s] esd check  got te signal!\n", __FUNCTION__);
			result = 1;
		}
#else
		if (dev->panel->ops->panel_esd_check != NULL) {
			result = dev->panel->ops->panel_esd_check(dev->panel);
//			pr_debug("sprdfb: [%s] panel check return %d\n", __FUNCTION__, result);
		}

#endif
	}


	if(0 == dev->enable){
		printk("sprdfb: [%s] leave (Invalid device status)!\n", __FUNCTION__);
		return 0;
	}

	if(result == 0){
		dev->panel_reset_time++;

		if(SPRDFB_PANEL_IF_EDPI == dev->panel_if_type){
			if(NULL != dev->panel->if_ctrl->panel_if_get_status){
				if_status = dev->panel->if_ctrl->panel_if_get_status(dev);
			}
		}else if(SPRDFB_PANEL_IF_DPI == dev->panel_if_type){
			if_status = 2; /*need reset dsi as default for dpi mode*/
		}

		if(0 == if_status){
			printk("sprdfb: [%s] fail! Need reset panel.(%d,%d,%d)\n",	__FUNCTION__,
                            dev->check_esd_time, dev->panel_reset_time, dev->reset_dsi_time);
			panel_reset(dev);

			if(0 == dev->enable){
				printk("sprdfb: [%s] leave (Invalid device status)!\n", __FUNCTION__);
				return 0;
			}

			dev->panel->ops->panel_init(dev->panel);
			panel_ready(dev);
		}else{
			printk("sprdfb: [%s] fail! Need reset panel and panel if!!!!(%d,%d,%d)\n",__FUNCTION__,
                            dev->check_esd_time, dev->panel_reset_time, dev->reset_dsi_time);
			dev->reset_dsi_time++;
			if(NULL != dev->panel->if_ctrl->panel_if_suspend){
				dev->panel->if_ctrl->panel_if_suspend(dev);
			}

			mdelay(10);

			if(0 == dev->enable){
				printk("sprdfb: [%s] leave (Invalid device status)!\n", __FUNCTION__);
				return 0;
			}

			panel_init(dev);
			panel_reset(dev);

			if(0 == dev->enable){
				printk("sprdfb: [%s] leave (Invalid device status)!\n", __FUNCTION__);
				return 0;
			}

			dev->panel->ops->panel_init(dev->panel);
			panel_ready(dev);
		}
		pr_debug("sprdfb: [%s]return 1\n",__FUNCTION__);
		return 1;
	}
//	pr_debug("sprdfb: [%s]return 0\n",__FUNCTION__);
	return 0;
}
#endif

void sprdfb_panel_suspend(struct sprdfb_device *dev)
{
	if(NULL == dev->panel){
		return;
	}

	printk("sprdfb: [%s], dev_id = %d\n",__FUNCTION__, dev->dev_id);
#if 0
	//step1-1 clk/data lane enter LP
	if(NULL != dev->panel->if_ctrl->panel_if_before_panel_reset){
		dev->panel->if_ctrl->panel_if_before_panel_reset(dev);
	}

	//step1-2 enter sleep  (another way : reset panel)
	/*Jessica TODO: Need do some I2c, SPI, mipi sleep here*/
	/* let lcdc sleep in */

	if (dev->panel->ops->panel_enter_sleep != NULL) {
		dev->panel->ops->panel_enter_sleep(dev->panel,1);
	}
	if(dev->panel->mipi_clk_level == 1)//add by liuwei 
		dev->panel->ops->panel_reset(dev->panel);//add by liuwei
	
	msleep(100);
#else
	//step1 send lcd sleep cmd or reset panel directly
	if(dev->panel->suspend_mode == SEND_SLEEP_CMD){
		panel_sleep(dev);
        printk("[kernel][%s %d]-------------\n",__func__,__LINE__);
	}else{
		panel_reset(dev);
        printk("[kernel][%s %d]-------------\n",__func__,__LINE__);
	}
#endif

	//step2 clk/data lane enter ulps
	if(NULL != dev->panel->if_ctrl->panel_if_enter_ulps){
		dev->panel->if_ctrl->panel_if_enter_ulps(dev);
	}

	//step3 turn off mipi
	if(NULL != dev->panel->if_ctrl->panel_if_suspend){
		dev->panel->if_ctrl->panel_if_suspend(dev);
	}

	//step4 reset pin to low
	if (dev->panel->ops->panel_after_suspend != NULL) {
		//himax mipi lcd may define empty function
		dev->panel->ops->panel_after_suspend(dev->panel);
	}
	else{
		panel_after_suspend(dev);
	}
}

void sprdfb_panel_resume(struct sprdfb_device *dev, bool from_deep_sleep)
{
	if(NULL == dev->panel){
		return;
	}

	printk(KERN_INFO "sprdfb: [%s], dev->enable= %d, from_deep_sleep = %d\n",__FUNCTION__, dev->enable, from_deep_sleep);
#if 0
	/*Jessica TODO: resume i2c, spi, mipi*/
	if(NULL != dev->panel->if_ctrl->panel_if_resume){
		dev->panel->if_ctrl->panel_if_resume(dev);
	}
	panel_ready(dev);
#endif
	//step1 reset pin to high
	if (dev->panel->ops->panel_before_resume != NULL) {
		//himax mipi lcd may define empty function
		dev->panel->ops->panel_before_resume(dev->panel);
	}
	else{
		panel_before_resume(dev);
	}

	if(from_deep_sleep){
		//step2 turn on mipi
		panel_init(dev);

		//step3 reset panel
		dev->panel->ops->panel_reset(dev->panel);//add by ontim
		panel_reset(dev);

        printk("[kernel][%s %d]-------------\n",__func__,__LINE__);
		//step4 panel init
		dev->panel->ops->panel_init(dev->panel);

		//step5 clk/data lane enter HS
		panel_ready(dev);
	}else{
		//step2 turn on mipi
		/*Jessica TODO: resume i2c, spi, mipi*/
		if(NULL != dev->panel->if_ctrl->panel_if_resume){
			dev->panel->if_ctrl->panel_if_resume(dev);
		}
        printk("[kernel][%s %d]-------------\n",__func__,__LINE__);
		//step3 sleep out
		if(NULL != dev->panel->ops->panel_enter_sleep){
			dev->panel->ops->panel_enter_sleep(dev->panel,0);
		}

		//step4 clk/data lane enter HS
		panel_ready(dev);
	}

}

void sprdfb_panel_remove(struct sprdfb_device *dev)
{
	if(NULL == dev->panel){
		return;
	}

	/*Jessica TODO:close panel, i2c, spi, mipi*/
	if(NULL != dev->panel->if_ctrl->panel_if_uninit){
		dev->panel->if_ctrl->panel_if_uninit(dev);
	}
	dev->panel = NULL;
}


int sprdfb_panel_register(struct panel_cfg *cfg)
{
	pr_debug("sprdfb: [%s], panel id = %d\n",__FUNCTION__, cfg->dev_id);

	if(!panel_check(cfg)){
		printk("sprdfb: [%s]: panel check fail!id = %d\n",__FUNCTION__,  cfg->dev_id);
		return -1;
	}

	mutex_lock(&panel_mutex);

	if (cfg->dev_id == SPRDFB_MAINLCD_ID) {
		list_add_tail(&cfg->list, &panel_list_main);
	} else if (cfg->dev_id == SPRDFB_SUBLCD_ID) {
		list_add_tail(&cfg->list, &panel_list_sub);
	} else {
		list_add_tail(&cfg->list, &panel_list_main);
		list_add_tail(&cfg->list, &panel_list_sub);
	}

	mutex_unlock(&panel_mutex);

	return 0;
}

 

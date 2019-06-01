/* drivers/video/sprdfb/lcd/lcd_nt35512s_mipi.c
 *
 * Support for nt35512s mipi LCD device
 *
 * Copyright (C) 2010 Spreadtrum
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
#include <linux/delay.h>
#include <linux/slab.h>
#include "../sprdfb_panel.h"
#include <linux/io.h>//add by liuwei
#include <soc/sprd/hardware.h>//add by liuwei
//#define LCD_Delay(ms)  uDelay(ms*1000)

#define MAX_DATA   48

typedef struct LCM_Init_Code_tag {
	unsigned int tag;
	unsigned char data[MAX_DATA];
}LCM_Init_Code;

typedef struct LCM_force_cmd_code_tag{
	unsigned int datatype;
	LCM_Init_Code real_cmd_code;
}LCM_Force_Cmd_Code;

#define LCM_TAG_SHIFT 24
#define LCM_TAG_MASK  ((1 << 24) -1)
#define LCM_SEND(len) ((1 << LCM_TAG_SHIFT)| len)
#define LCM_SLEEP(ms) ((2 << LCM_TAG_SHIFT)| ms)
//#define ARRAY_SIZE(array) ( sizeof(array) / sizeof(array[0]))

#define LCM_TAG_SEND  (1<< 0)
#define LCM_TAG_SLEEP (1 << 1)

static LCM_Force_Cmd_Code rd_prep_code[]={
	{0x37, {LCM_SEND(2), {0x5, 0}}},
};

static LCM_Force_Cmd_Code rd_prep_code_1[]={
	{0x37, {LCM_SEND(2), {0x1, 0}}},
};

static u8 lcd_nt35512s_oufei_cmd_1[]={0xFF,0xAA,0x55,0x69,0x10};
static u8 lcd_nt35512s_oufei_cmd_2[]={0x6F,0x10};
static u8 lcd_nt35512s_oufei_cmd_3[]={0xF7,0x01};
static u8 lcd_nt35512s_oufei_cmd_4[]={0xFF,0xAA,0x55,0x69,0x00};
static u8 lcd_nt35512s_oufei_cmd_5[]={0xF0,0x55,0xAA,0x52,0x08,0x02};
static u8 lcd_nt35512s_oufei_cmd_6[]={0xB7,0x01};
static u8 lcd_nt35512s_oufei_cmd_7[]={0xF0,0x55,0xAA,0x52,0x08,0x01};
static u8 lcd_nt35512s_oufei_cmd_8[]={0xB0,0x0D};
static u8 lcd_nt35512s_oufei_cmd_9[]={0xB6,0x44};
static u8 lcd_nt35512s_oufei_cmd_10[]={0xB1,0x0D};
static u8 lcd_nt35512s_oufei_cmd_11[]={0xB7,0x35};
static u8 lcd_nt35512s_oufei_cmd_12[]={0xB2,0x00};
static u8 lcd_nt35512s_oufei_cmd_13[]={0xB8,0x24};
static u8 lcd_nt35512s_oufei_cmd_14[]={0xBF,0x01};
static u8 lcd_nt35512s_oufei_cmd_15[]={0xB3,0x0F};
static u8 lcd_nt35512s_oufei_cmd_16[]={0xB9,0x35};
static u8 lcd_nt35512s_oufei_cmd_17[]={0xB5,0x08};
static u8 lcd_nt35512s_oufei_cmd_18[]={0xC2,0x03};
static u8 lcd_nt35512s_oufei_cmd_19[]={0xBA,0x14};
static u8 lcd_nt35512s_oufei_cmd_20[]={0xBC,0x00,0x78,0x00};
static u8 lcd_nt35512s_oufei_cmd_21[]={0xBD,0x00,0x78,0x00};
static u8 lcd_nt35512s_oufei_cmd_22[]={0xBE,0x00,0x56,0X00,0x56};
static u8 lcd_nt35512s_oufei_cmd_23[]={0xD1,0x00,0x00,0x00,0x03,0x00,0x0A,0x00,0x1D,0x00,0x37,0x00,0x6F,0x00,0x94,0x00,0xDF,0x01,0x0E,0x01,0x52,0x01,0x80,0x01,0xCA,0x01,0xFC,0x01,0xFF,0x02,0x2B,0x02,0x58,0x02,0x71,0x02,0x90,0x02,0xA5,0x02,0xBC,0x02,0xCC,0x02,0xE4,0x02,0xED,0x03,0x0E,0x03,0x49,0x03,0xEE};
static u8 lcd_nt35512s_oufei_cmd_24[]={0xF0,0x55,0xAA,0x52,0x08,0x00};
static u8 lcd_nt35512s_oufei_cmd_25[]={0xB2, 0x00};//improve esd
static u8 lcd_nt35512s_oufei_cmd_26[]={0xB5, 0x6B};
static u8 lcd_nt35512s_oufei_cmd_27[]={0xB1, 0xFC, 0x00};
static u8 lcd_nt35512s_oufei_cmd_28[]={0xB6,0x05};
static u8 lcd_nt35512s_oufei_cmd_29[]={0xB7,0x70,0x70};
static u8 lcd_nt35512s_oufei_cmd_30[]={0xB8,0x01,0x03,0x03,0x03};
static u8 lcd_nt35512s_oufei_cmd_31[]={0xBC,0x02};// 2 dot mode
static u8 lcd_nt35512s_oufei_cmd_32[]={0xC9,0xC0,0x02,0x50,0x50,0x50};
static u8 lcd_nt35512s_oufei_cmd_33[]={0x36,0x00};
          
static sprdfb_dev_dsi_cmds lcd_nt35512s_oufei_cmds_init[] =
{
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_1), .p_cmd=lcd_nt35512s_oufei_cmd_1,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_2), .p_cmd=lcd_nt35512s_oufei_cmd_2,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_3), .p_cmd=lcd_nt35512s_oufei_cmd_3,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_4), .p_cmd=lcd_nt35512s_oufei_cmd_4,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_5), .p_cmd=lcd_nt35512s_oufei_cmd_5,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_6), .p_cmd=lcd_nt35512s_oufei_cmd_6,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_7), .p_cmd=lcd_nt35512s_oufei_cmd_7,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_8), .p_cmd=lcd_nt35512s_oufei_cmd_8,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_9), .p_cmd=lcd_nt35512s_oufei_cmd_9,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_10), .p_cmd=lcd_nt35512s_oufei_cmd_10,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_11), .p_cmd=lcd_nt35512s_oufei_cmd_11,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_12), .p_cmd=lcd_nt35512s_oufei_cmd_12,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_13), .p_cmd=lcd_nt35512s_oufei_cmd_13,},
 	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_14), .p_cmd=lcd_nt35512s_oufei_cmd_14,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_15), .p_cmd=lcd_nt35512s_oufei_cmd_15,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_16), .p_cmd=lcd_nt35512s_oufei_cmd_16,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_17), .p_cmd=lcd_nt35512s_oufei_cmd_17,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_18), .p_cmd=lcd_nt35512s_oufei_cmd_18,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_19), .p_cmd=lcd_nt35512s_oufei_cmd_19,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_20), .p_cmd=lcd_nt35512s_oufei_cmd_20,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_21), .p_cmd=lcd_nt35512s_oufei_cmd_21,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_22), .p_cmd=lcd_nt35512s_oufei_cmd_22,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_23), .p_cmd=lcd_nt35512s_oufei_cmd_23,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_24), .p_cmd=lcd_nt35512s_oufei_cmd_24,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_25), .p_cmd=lcd_nt35512s_oufei_cmd_25,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_26), .p_cmd=lcd_nt35512s_oufei_cmd_26,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_27), .p_cmd=lcd_nt35512s_oufei_cmd_27,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_28), .p_cmd=lcd_nt35512s_oufei_cmd_28,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_29), .p_cmd=lcd_nt35512s_oufei_cmd_29,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_30), .p_cmd=lcd_nt35512s_oufei_cmd_30,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_31), .p_cmd=lcd_nt35512s_oufei_cmd_31,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_32), .p_cmd=lcd_nt35512s_oufei_cmd_32,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_oufei_cmd_33), .p_cmd=lcd_nt35512s_oufei_cmd_33,},
	
	{ .type = DSI_CMD_DCS,	.len = 1, .cmd={0x11},},
	{ .type = DSI_CMD_DEALY,	.len = 120, },
	{ .type = DSI_CMD_DCS,	.len = 1, .cmd={0x29},},
	{ .type = DSI_CMD_DEALY,	.len = 20, },
	//{ .type = DSI_CMD_DCS,	.len = 1, .cmd={0x2C},},
	{ .type = DSI_CMD_END,},
};

static sprdfb_dev_dsi_cmds lcd_nt35512s_oufei_entry_sleep_cmds_init[] =
{
 
	{ .type = DSI_CMD_DCS,	.len = 1, .cmd={0x28},},
	{ .type = DSI_CMD_DEALY,	.len = 10, },
	{ .type = DSI_CMD_DCS,	.len = 1, .cmd={0x10},},
	{ .type = DSI_CMD_DEALY,	.len = 120, },
	{ .type = DSI_CMD_END,},
};

static int32_t nt35512s_mipi_init(struct panel_spec *self)
{
	sprdfb_dev_dsi_cmds *init = lcd_nt35512s_oufei_cmds_init;
	mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
	mipi_common_cmd_t mipi_common_cmd = self->info.mipi->ops->mipi_common_cmd;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;


	mipi_set_cmd_mode();
	mipi_eotp_set(1,0);
	
	mipi_common_cmd(init);
	
	mipi_eotp_set(1,1);
	printk(KERN_INFO "[kernel]:%s.done\n",__func__);
	
    return 0;
}

static int32_t nt35512s_enter_sleep(struct panel_spec *self, uint8_t is_sleep)
{
	sprdfb_dev_dsi_cmds *init = lcd_nt35512s_oufei_entry_sleep_cmds_init;
	mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
	mipi_common_cmd_t mipi_common_cmd = self->info.mipi->ops->mipi_common_cmd;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;


	mipi_set_cmd_mode();
	mipi_eotp_set(1,0);
	
	mipi_common_cmd(init);
	
	mipi_eotp_set(1,1);
	printk(KERN_INFO "[kernel]:%s.done.is_sleep=%d.\n",__func__,is_sleep);
	
    return 0;
}
static uint32_t nt35512s_readid(struct panel_spec *self)
{
	/*Jessica TODO: need read id*/
	return 0x9806;
}

static uint32_t nt35512s_readpowermode(struct panel_spec *self)
{
	int32_t i = 0;
	uint32_t j =0;
	LCM_Force_Cmd_Code * rd_prepare = rd_prep_code_1;
	uint8_t read_data[1] = {0};
	int32_t read_rtn = 0;
	unsigned int tag = 0;

	mipi_force_write_t mipi_force_write = self->info.mipi->ops->mipi_force_write;
	mipi_force_read_t mipi_force_read = self->info.mipi->ops->mipi_force_read;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

	pr_debug("lcd_nt35512s_mipi read power mode!\n");
	mipi_eotp_set(0,1);
	for(j = 0; j < 4; j++){
		rd_prepare = rd_prep_code_1;
		for(i = 0; i < ARRAY_SIZE(rd_prep_code_1); i++){
			tag = (rd_prepare->real_cmd_code.tag >> 24);
			if(tag & LCM_TAG_SEND){
				mipi_force_write(rd_prepare->datatype, rd_prepare->real_cmd_code.data, (rd_prepare->real_cmd_code.tag & LCM_TAG_MASK));
			}else if(tag & LCM_TAG_SLEEP){
				msleep((rd_prepare->real_cmd_code.tag & LCM_TAG_MASK));
			}
			rd_prepare++;
		}
		read_rtn = mipi_force_read(0x0A, 1,(uint8_t *)read_data);
		pr_debug("lcd_nt35512s_mipi read power mode 0x0A value is 0x%x! , read result(%d)\n", read_data[0], read_rtn);
		if((0x9c == read_data[0])  && (0 == read_rtn)){
			pr_debug("lcd_nt35512sa_mipi read power mode success!\n");
			mipi_eotp_set(1,1);
			return 0x9c;
		}
	}

	printk("lcd_nt35512s_mipi read power mode fail!0x0A value is 0x%x! , read result(%d)\n", read_data[0], read_rtn);
	mipi_eotp_set(1,1);
	return 0x0;
}

static int32_t nt35512s_check_esd(struct panel_spec *self)
{
	uint32_t power_mode;

	mipi_set_lp_mode_t mipi_set_data_lp_mode = self->info.mipi->ops->mipi_set_data_lp_mode;
	mipi_set_hs_mode_t mipi_set_data_hs_mode = self->info.mipi->ops->mipi_set_data_hs_mode;
	mipi_set_lp_mode_t mipi_set_lp_mode = self->info.mipi->ops->mipi_set_lp_mode;
	mipi_set_hs_mode_t mipi_set_hs_mode = self->info.mipi->ops->mipi_set_hs_mode;
	uint16_t work_mode = self->info.mipi->work_mode;

	pr_debug("nt35512s_check_esd!\n");
	if(SPRDFB_MIPI_MODE_CMD==work_mode){
		mipi_set_lp_mode();
	}else{
		mipi_set_data_lp_mode();
	}
	power_mode = nt35512s_readpowermode(self);
	//power_mode = 0x0;
	if(SPRDFB_MIPI_MODE_CMD==work_mode){
		mipi_set_hs_mode();
	}else{
		mipi_set_data_hs_mode();
	}
	if(power_mode == 0x9c){
		pr_debug("nt35512s_check_esd OK!\n");
		return 1;
	}else{
		printk("nt35512s_check_esd fail!(0x%x)\n", power_mode);
		return 0;
	}
}



static u8 lcd_oufei_nt35512s_driving_mode_cmd_1[]={0xB2,0x82,0xFE,0x09,0x03,0x00,0x50,0x11,0x42,0x1D};
static sprdfb_dev_dsi_cmds lcd_driving_mode_control[] =
{
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_oufei_nt35512s_driving_mode_cmd_1), .p_cmd=lcd_oufei_nt35512s_driving_mode_cmd_1,},
};
static void nt35512s_oufei_driving_mode_set(struct panel_spec *self,bool enable)
{
    sprdfb_dev_dsi_cmds *init = lcd_driving_mode_control;
    mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
    mipi_common_cmd_t mipi_common_cmd = self->info.mipi->ops->mipi_common_cmd;
	
    if(enable)//dot mode 
    	lcd_driving_mode_control[0].p_cmd[1] = 0x82;
    else //column mode
	lcd_driving_mode_control[0].p_cmd[1] = 0x80;
	
    mipi_set_cmd_mode();

    mipi_common_cmd(init);
	
    printk(KERN_INFO "[kernel]:nt35512s_oufei_driving_mode_set ok.dot enable=%d.\n",enable);
}
static struct panel_operations lcd_nt35512s_mipi_operations = {
	.panel_init = nt35512s_mipi_init,
	//.panel_dot_init = nt35512s_mipi_dot_init,
	.panel_readid = nt35512s_readid,
	.panel_esd_check = nt35512s_check_esd,
	.panel_enter_sleep = nt35512s_enter_sleep,
	.lcd_set_driving_mode = nt35512s_oufei_driving_mode_set,
};

static struct timing_rgb lcd_nt35512s_mipi_timing =
{
	.hfp = 80,  /* unit: pixel */
	.hbp = 77,
	.hsync = 8,
	.vfp = 14, /*unit: line*/
	.vbp = 14,
	.vsync = 4,
};

static struct info_mipi lcd_nt35512s_mipi_info = {
	.work_mode  = SPRDFB_MIPI_MODE_VIDEO,
	.video_bus_width = 24, /*18,16*/
	.lan_number = 2,
	.phy_feq = 412*1000,
	.h_sync_pol = SPRDFB_POLARITY_POS,
	.v_sync_pol = SPRDFB_POLARITY_POS,
	.de_pol = SPRDFB_POLARITY_POS,
	.te_pol = SPRDFB_POLARITY_POS,
	.color_mode_pol = SPRDFB_POLARITY_NEG,
	.shut_down_pol = SPRDFB_POLARITY_NEG,
	.timing = &lcd_nt35512s_mipi_timing,
	.ops = NULL,
};

struct panel_spec lcd_nt35512s_oufei_mipi_spec = {
	.width = 480,
	.height = 854,
	.fps	= 60,
	.reset_timing = {5,40,120},
	.type = LCD_MODE_DSI,
	.direction = LCD_DIRECT_NORMAL,
	.is_clean_lcd = true,
	.suspend_mode = SEND_SLEEP_CMD ,
	.info = {
		.mipi = &lcd_nt35512s_mipi_info
	},
	.ops = &lcd_nt35512s_mipi_operations,
};

struct panel_cfg lcd_nt35512s_oufei_mipi = {
	/* this panel can only be main lcd */
	.dev_id = SPRDFB_MAINLCD_ID,
	.lcd_id = 0x9806,
	.lcd_name = "lcd_nt35512s_mipi",
    .lcd_voltage_max = 1699,
    .lcd_voltage_min = 1420,
	.panel = &lcd_nt35512s_oufei_mipi_spec,
};

static int __init lcd_nt35512s_mipi_init(void)
{
	return sprdfb_panel_register(&lcd_nt35512s_oufei_mipi);
}

subsys_initcall(lcd_nt35512s_mipi_init);

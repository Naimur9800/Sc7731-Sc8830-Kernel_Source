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

static u8 lcd_nt35512s_zgd102_cmd_1[]= {0xF0,0x55,0xAA,0x52,0x08,0x01};
static u8 lcd_nt35512s_zgd102_cmd_2[]= {0xB6,0x44,0x44,0x44};
static u8 lcd_nt35512s_zgd102_cmd_3[]= {0xB0,0x09,0x09,0x09};
static u8 lcd_nt35512s_zgd102_cmd_4[]= {0xB7,0x34,0x34,0x34};
static u8 lcd_nt35512s_zgd102_cmd_5[]= {0xB1,0x09,0x09,0x09};
static u8 lcd_nt35512s_zgd102_cmd_6[]= {0xB8,0x34,0x34,0x34};
static u8 lcd_nt35512s_zgd102_cmd_7[]= {0xB2,0x01,0x01,0x01};
static u8 lcd_nt35512s_zgd102_cmd_8[]= {0xB9,0x24,0x24,0x24};
static u8 lcd_nt35512s_zgd102_cmd_9[]= {0xB3,0x08,0x08,0x08};
static u8 lcd_nt35512s_zgd102_cmd_10[]={0xBA,0x24,0x24,0x24};
static u8 lcd_nt35512s_zgd102_cmd_11[]={0xB5,0x0B,0x0B,0x0B};
static u8 lcd_nt35512s_zgd102_cmd_12[]={0xBC,0x00,0xA3,0x00};
static u8 lcd_nt35512s_zgd102_cmd_13[]={0xBD,0x00,0xA3,0x00};
static u8 lcd_nt35512s_zgd102_cmd_14[]={0xBE,0x00,0x58};
static u8 lcd_nt35512s_zgd102_cmd_15[]={0xBF,0x01};
static u8 lcd_nt35512s_zgd102_cmd_16[]={0xD1,0x00,0x01,0x00,0x7f,0x00,0xaf,0x00,0xcc,0x00,0xe2,0x01,0x0a,0x01,0x28,0x01,0x57,0x01,0x7b,0x01,0xb7,0x01,0xe3,0x02,0x2c,0x02,0x65,0x02,0x67,0x02,0x9b,0x02,0xd6,0x02,0xf7,0x03,0x25,0x03,0x44,0x03,0x65,0x03,0x7b,0x03,0x97,0x03,0xb5,0x03,0xea,0x03,0xfd,0x03,0xFE};
static u8 lcd_nt35512s_zgd102_cmd_17[]={0xD2,0x00,0x01,0x00,0x7f,0x00,0xaf,0x00,0xcc,0x00,0xe2,0x01,0x0a,0x01,0x28,0x01,0x57,0x01,0x7b,0x01,0xb7,0x01,0xe3,0x02,0x2c,0x02,0x65,0x02,0x67,0x02,0x9b,0x02,0xd6,0x02,0xf7,0x03,0x25,0x03,0x44,0x03,0x65,0x03,0x7b,0x03,0x97,0x03,0xb5,0x03,0xea,0x03,0xfd,0x03,0xFE};
static u8 lcd_nt35512s_zgd102_cmd_18[]={0xD3,0x00,0x01,0x00,0x7f,0x00,0xaf,0x00,0xcc,0x00,0xe2,0x01,0x0a,0x01,0x28,0x01,0x57,0x01,0x7b,0x01,0xb7,0x01,0xe3,0x02,0x2c,0x02,0x65,0x02,0x67,0x02,0x9b,0x02,0xd6,0x02,0xf7,0x03,0x25,0x03,0x44,0x03,0x65,0x03,0x7b,0x03,0x97,0x03,0xb5,0x03,0xea,0x03,0xfd,0x03,0xFE};
static u8 lcd_nt35512s_zgd102_cmd_19[]={0xD4,0x00,0x01,0x00,0x7f,0x00,0xaf,0x00,0xcc,0x00,0xe2,0x01,0x0a,0x01,0x28,0x01,0x57,0x01,0x7b,0x01,0xb7,0x01,0xe3,0x02,0x2c,0x02,0x65,0x02,0x67,0x02,0x9b,0x02,0xd6,0x02,0xf7,0x03,0x25,0x03,0x44,0x03,0x65,0x03,0x7b,0x03,0x97,0x03,0xb5,0x03,0xea,0x03,0xfd,0x03,0xFE};
static u8 lcd_nt35512s_zgd102_cmd_20[]={0xD5,0x00,0x01,0x00,0x7f,0x00,0xaf,0x00,0xcc,0x00,0xe2,0x01,0x0a,0x01,0x28,0x01,0x57,0x01,0x7b,0x01,0xb7,0x01,0xe3,0x02,0x2c,0x02,0x65,0x02,0x67,0x02,0x9b,0x02,0xd6,0x02,0xf7,0x03,0x25,0x03,0x44,0x03,0x65,0x03,0x7b,0x03,0x97,0x03,0xb5,0x03,0xea,0x03,0xfd,0x03,0xFE};
static u8 lcd_nt35512s_zgd102_cmd_21[]={0xD6,0x00,0x01,0x00,0x7f,0x00,0xaf,0x00,0xcc,0x00,0xe2,0x01,0x0a,0x01,0x28,0x01,0x57,0x01,0x7b,0x01,0xb7,0x01,0xe3,0x02,0x2c,0x02,0x65,0x02,0x67,0x02,0x9b,0x02,0xd6,0x02,0xf7,0x03,0x25,0x03,0x44,0x03,0x65,0x03,0x7b,0x03,0x97,0x03,0xb5,0x03,0xea,0x03,0xfd,0x03,0xFE};
static u8 lcd_nt35512s_zgd102_cmd_22[]={0xF0,0x55,0xAA,0x52,0x08,0x00};
static u8 lcd_nt35512s_zgd102_cmd_23[]={0xB5,0x6B};
static u8 lcd_nt35512s_zgd102_cmd_24[]={0xB6,0x0A};
static u8 lcd_nt35512s_zgd102_cmd_25[]={0xB7,0x00,0x00};
static u8 lcd_nt35512s_zgd102_cmd_26[]={0xB8,0x01,0x05,0x05,0x05};
static u8 lcd_nt35512s_zgd102_cmd_27[]={0xBA,0x01};
static u8 lcd_nt35512s_zgd102_cmd_28[]={0xBC,0x02,0x02,0x02};
static u8 lcd_nt35512s_zgd102_cmd_29[]={0xBD,0x01,0x84,0x07,0x31,0x00};
static u8 lcd_nt35512s_zgd102_cmd_30[]={0xBE,0x01,0x84,0x07,0x31,0x00};
static u8 lcd_nt35512s_zgd102_cmd_31[]={0xBF,0x01,0x84,0x07,0x31,0x00};
static u8 lcd_nt35512s_zgd102_cmd_32[]={0xCC,0x03,0x00,0x00};
static u8 lcd_nt35512s_zgd102_cmd_33[]={0xB1,0x6c,0x00};
static u8 lcd_nt35512s_zgd102_cmd_34[]={0x36,0x00};
static u8 lcd_nt35512s_zgd102_cmd_35[]={0x3a,0x77};
static u8 lcd_nt35512s_zgd102_cmd_36[]={0x11};
static u8 lcd_nt35512s_zgd102_cmd_37[]={0x29};


static sprdfb_dev_dsi_cmds lcd_nt35512s_zgd102_cmds_init[] =
{
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_1), .p_cmd=lcd_nt35512s_zgd102_cmd_1,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_2), .p_cmd=lcd_nt35512s_zgd102_cmd_2,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_3), .p_cmd=lcd_nt35512s_zgd102_cmd_3,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_4), .p_cmd=lcd_nt35512s_zgd102_cmd_4,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_5), .p_cmd=lcd_nt35512s_zgd102_cmd_5,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_6), .p_cmd=lcd_nt35512s_zgd102_cmd_6,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_7), .p_cmd=lcd_nt35512s_zgd102_cmd_7,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_8), .p_cmd=lcd_nt35512s_zgd102_cmd_8,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_9), .p_cmd=lcd_nt35512s_zgd102_cmd_9,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_10), .p_cmd=lcd_nt35512s_zgd102_cmd_10,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_11), .p_cmd=lcd_nt35512s_zgd102_cmd_11,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_12), .p_cmd=lcd_nt35512s_zgd102_cmd_12,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_13), .p_cmd=lcd_nt35512s_zgd102_cmd_13,},
 	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_14), .p_cmd=lcd_nt35512s_zgd102_cmd_14,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_15), .p_cmd=lcd_nt35512s_zgd102_cmd_15,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_16), .p_cmd=lcd_nt35512s_zgd102_cmd_16,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_17), .p_cmd=lcd_nt35512s_zgd102_cmd_17,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_18), .p_cmd=lcd_nt35512s_zgd102_cmd_18,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_19), .p_cmd=lcd_nt35512s_zgd102_cmd_19,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_20), .p_cmd=lcd_nt35512s_zgd102_cmd_20,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_21), .p_cmd=lcd_nt35512s_zgd102_cmd_21,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_22), .p_cmd=lcd_nt35512s_zgd102_cmd_22,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_23), .p_cmd=lcd_nt35512s_zgd102_cmd_23,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_24), .p_cmd=lcd_nt35512s_zgd102_cmd_24,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_25), .p_cmd=lcd_nt35512s_zgd102_cmd_25,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_26), .p_cmd=lcd_nt35512s_zgd102_cmd_26,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_27), .p_cmd=lcd_nt35512s_zgd102_cmd_27,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_28), .p_cmd=lcd_nt35512s_zgd102_cmd_28,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_29), .p_cmd=lcd_nt35512s_zgd102_cmd_29,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_30), .p_cmd=lcd_nt35512s_zgd102_cmd_30,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_31), .p_cmd=lcd_nt35512s_zgd102_cmd_31,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_32), .p_cmd=lcd_nt35512s_zgd102_cmd_32,},
  { .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_33), .p_cmd=lcd_nt35512s_zgd102_cmd_33,},

	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_34), .p_cmd=lcd_nt35512s_zgd102_cmd_34,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_35), .p_cmd=lcd_nt35512s_zgd102_cmd_35,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_36), .p_cmd=lcd_nt35512s_zgd102_cmd_36,},
	{ .type = DSI_CMD_DEALY,	.len = 120, },
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_nt35512s_zgd102_cmd_37), .p_cmd=lcd_nt35512s_zgd102_cmd_37,},
  { .type = DSI_CMD_DEALY,	.len = 20, },
   	
	{ .type = DSI_CMD_END,},
};

static sprdfb_dev_dsi_cmds lcd_nt35512s_zgd102_entry_sleep_cmds_init[] =
{
 
	{ .type = DSI_CMD_DCS,	.len = 1, .cmd={0x28},},
	{ .type = DSI_CMD_DEALY,	.len = 10, },
	{ .type = DSI_CMD_DCS,	.len = 1, .cmd={0x10},},
	{ .type = DSI_CMD_DEALY,	.len = 120, },
	{ .type = DSI_CMD_END,},
};

extern unsigned int lcd_debug_panel_on_cmds( sprdfb_dev_dsi_cmds**cmds, unsigned char** buf) ;

static int32_t nt35512s_mipi_init(struct panel_spec *self)
{
    sprdfb_dev_dsi_cmds *dbg_cmds = NULL;
    unsigned char* dbg_data= NULL ;
    unsigned int size = 0;

	sprdfb_dev_dsi_cmds *init = lcd_nt35512s_zgd102_cmds_init;
	mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
	mipi_common_cmd_t mipi_common_cmd = self->info.mipi->ops->mipi_common_cmd;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;


    printk(KERN_DEBUG "[kernel]:nt35512s_zgd102_init\n");
	mipi_set_cmd_mode();
	mipi_eotp_set(1,0);
	
    if( ( size = lcd_debug_panel_on_cmds(&dbg_cmds, &dbg_data) ) > 0 )
    {
        mipi_common_cmd(dbg_cmds);
        kfree(dbg_cmds);
        kfree(dbg_data);
        printk(KERN_INFO "[kernel]:>>>>>>>load sdcard config param success<<<<<<<<< \n");
    }
    else
    {
        mipi_common_cmd(init);
        printk("[kernel]:>>>>>>>>load local config param success<<<<<<<<<<<<< \n");
    }
    mipi_eotp_set(1,1);
    
    return 0;
}

static int32_t nt35512s_enter_sleep(struct panel_spec *self, uint8_t is_sleep)
{
	sprdfb_dev_dsi_cmds *init = lcd_nt35512s_zgd102_entry_sleep_cmds_init;
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



static u8 lcd_zgd102_nt35512s_driving_mode_cmd_1[]={0xB2,0x82,0xFE,0x09,0x03,0x00,0x50,0x11,0x42,0x1D};
static sprdfb_dev_dsi_cmds lcd_driving_mode_control[] =
{
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_zgd102_nt35512s_driving_mode_cmd_1), .p_cmd=lcd_zgd102_nt35512s_driving_mode_cmd_1,},
};
static void nt35512s_zgd102_driving_mode_set(struct panel_spec *self,bool enable)
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
	
    printk(KERN_INFO "[kernel]:nt35512s_zgd102_driving_mode_set ok.dot enable=%d.\n",enable);
}
static struct panel_operations lcd_nt35512s_mipi_operations = {
	.panel_init = nt35512s_mipi_init,
	//.panel_dot_init = nt35512s_mipi_dot_init,
	.panel_readid = nt35512s_readid,
	.panel_esd_check = nt35512s_check_esd,
	.panel_enter_sleep = nt35512s_enter_sleep,
	.lcd_set_driving_mode = nt35512s_zgd102_driving_mode_set,
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

struct panel_spec lcd_nt35512s_zgd102_mipi_spec = {
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

struct panel_cfg lcd_nt35512s_zgd102_mipi = {
	/* this panel can only be main lcd */
	.dev_id = SPRDFB_MAINLCD_ID,
	.lcd_id = 0x35512,
	.lcd_name = "lcd_nt35512s_zgd102_mipi",
  .lcd_voltage_max = 2400,
  .lcd_voltage_min = 2150,
	.panel = &lcd_nt35512s_zgd102_mipi_spec,
};

static int __init lcd_nt35512s_mipi_init(void)
{
	return sprdfb_panel_register(&lcd_nt35512s_zgd102_mipi);
}

subsys_initcall(lcd_nt35512s_mipi_init);

/* drivers/video/sc8825/lcd_ili9805_yt40f139a_mipi.c
 *
 * Support for otm8019a mipi LCD device
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


static u8 lcd_ili9805_yt40f139a_cmd_1[]= {0xFF,0xFF,0x98,0x05};
static u8 lcd_ili9805_yt40f139a_cmd_2[]= {0xFD,0x00,0x10,0x44,0x33};
static u8 lcd_ili9805_yt40f139a_cmd_3[]= {0xF8,0x10,0x03,0x03,0xB4,0x03,0x03,0x11,0x03};
static u8 lcd_ili9805_yt40f139a_cmd_4[]= {0xF1,0xC1,0x08,0x00,0x2B,0x01};
static u8 lcd_ili9805_yt40f139a_cmd_5[]= {0xE0,0x00,0x07,0x11,0x0E,0x14,0x11,0x0A,0x08,0x03,0x03,0x0C,0x0B,0x0C,0x2B,0x28,0x00};
static u8 lcd_ili9805_yt40f139a_cmd_6[]= {0xE1,0x00,0x07,0x10,0x10,0x13,0x17,0x08,0x08,0x04,0x0C,0x03,0x0f,0x0e,0x32,0x2c,0x00};
static u8 lcd_ili9805_yt40f139a_cmd_7[]= {0xEA,0x02};
static u8 lcd_ili9805_yt40f139a_cmd_8[]= {0xE2,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00};
static u8 lcd_ili9805_yt40f139a_cmd_9[]= {0xE3,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static u8 lcd_ili9805_yt40f139a_cmd_10[]={0xE4,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static u8 lcd_ili9805_yt40f139a_cmd_11[]={0xC1,0x13,0x2E,0x0E,0x26};
static u8 lcd_ili9805_yt40f139a_cmd_12[]={0xC7,0xdb};
static u8 lcd_ili9805_yt40f139a_cmd_13[]={0xDF,0x23};
static u8 lcd_ili9805_yt40f139a_cmd_14[]={0xB0,0x01};
static u8 lcd_ili9805_yt40f139a_cmd_15[]={0xB9,0x08,0x02};
static u8 lcd_ili9805_yt40f139a_cmd_16[]={0xB4,0x02};
static u8 lcd_ili9805_yt40f139a_cmd_17[]={0x3A,0x70};
static u8 lcd_ili9805_yt40f139a_cmd_18[]={0x36,0x02};
static u8 lcd_ili9805_yt40f139a_cmd_19[]={0xC2,0x21};
static u8 lcd_ili9805_yt40f139a_cmd_20[]={0xF3,0x74};
static u8 lcd_ili9805_yt40f139a_cmd_21[]={0xBD,0x00,0x42,0x00};
static u8 lcd_ili9805_yt40f139a_cmd_22[]={0xEB,0x24};
static u8 lcd_ili9805_yt40f139a_cmd_23[]={0xC4,0xF8};
static u8 lcd_ili9805_yt40f139a_cmd_24[]={0x11};
static u8 lcd_ili9805_yt40f139a_cmd_25[]={0x29};


static sprdfb_dev_dsi_cmds lcd_ili9805_yt40f139a_cmds_init[] =
{
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_1), .p_cmd=lcd_ili9805_yt40f139a_cmd_1,},
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_2), .p_cmd=lcd_ili9805_yt40f139a_cmd_2,},
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_3), .p_cmd=lcd_ili9805_yt40f139a_cmd_3,},
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_4), .p_cmd=lcd_ili9805_yt40f139a_cmd_4,},
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_5), .p_cmd=lcd_ili9805_yt40f139a_cmd_5,},
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_6), .p_cmd=lcd_ili9805_yt40f139a_cmd_6,},
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_7), .p_cmd=lcd_ili9805_yt40f139a_cmd_7,},
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_8), .p_cmd=lcd_ili9805_yt40f139a_cmd_8,},
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_9), .p_cmd=lcd_ili9805_yt40f139a_cmd_9,},
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_10), .p_cmd=lcd_ili9805_yt40f139a_cmd_10,},
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_11), .p_cmd=lcd_ili9805_yt40f139a_cmd_11,},
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_12), .p_cmd=lcd_ili9805_yt40f139a_cmd_12,},
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_13), .p_cmd=lcd_ili9805_yt40f139a_cmd_13,},
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_14), .p_cmd=lcd_ili9805_yt40f139a_cmd_14,},
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_15), .p_cmd=lcd_ili9805_yt40f139a_cmd_15,},
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_16), .p_cmd=lcd_ili9805_yt40f139a_cmd_16,},
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_17), .p_cmd=lcd_ili9805_yt40f139a_cmd_17,},
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_18), .p_cmd=lcd_ili9805_yt40f139a_cmd_18,},
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_19), .p_cmd=lcd_ili9805_yt40f139a_cmd_19,},
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_20), .p_cmd=lcd_ili9805_yt40f139a_cmd_20,},
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_21), .p_cmd=lcd_ili9805_yt40f139a_cmd_21,},
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_22), .p_cmd=lcd_ili9805_yt40f139a_cmd_22,},
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_23), .p_cmd=lcd_ili9805_yt40f139a_cmd_23,},
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_24), .p_cmd=lcd_ili9805_yt40f139a_cmd_24,},
    { .type = DSI_CMD_DEALY,    .len = 120, },
    { .type = DSI_CMD_DCS,    .len = sizeof(lcd_ili9805_yt40f139a_cmd_25), .p_cmd=lcd_ili9805_yt40f139a_cmd_25,},
    { .type = DSI_CMD_DEALY,    .len = 20, },
    { .type = DSI_CMD_END,},
};

static sprdfb_dev_dsi_cmds disp_on = { .type = DSI_CMD_GEN,    .len = 1, .cmd={0x29},};

static sprdfb_dev_dsi_cmds sleep_in[] =
{
    { .type = DSI_CMD_GEN,    .len = 1, .cmd={0x22},},
    { .type = DSI_CMD_DEALY,    .len = 20, },
    { .type = DSI_CMD_GEN,    .len = 1, .cmd={0x28},},
    { .type = DSI_CMD_DEALY,    .len = 120, },
    { .type = DSI_CMD_GEN,    .len = 1, .cmd={0x10},},
    { .type = DSI_CMD_DEALY,    .len = 120, },
    { .type = DSI_CMD_END,},

};

static sprdfb_dev_dsi_cmds sleep_out[] =
{
    { .type = DSI_CMD_GEN,    .len = 1, .cmd={0x11},},
    { .type = DSI_CMD_DEALY,    .len = 120, },
    { .type = DSI_CMD_GEN,    .len = 1, .cmd={0x29},},
    { .type = DSI_CMD_DEALY,    .len = 20, },
    { .type = DSI_CMD_END,},
};

extern unsigned int lcd_debug_panel_on_cmds( sprdfb_dev_dsi_cmds**cmds, unsigned char** buf) ;
static uint32_t ili9805_yt40f139a_readreg(struct panel_spec *self) ;

static int32_t ili9805_yt40f139a_mipi_init(struct panel_spec *self)
{
    sprdfb_dev_dsi_cmds *dbg_cmds = NULL;
    unsigned char* dbg_data= NULL ;
    unsigned int size = 0;
    
    mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
    mipi_force_read_t mipi_force_read = self->info.mipi->ops->mipi_force_read;
    mipi_common_cmd_t mipi_common_cmd = self->info.mipi->ops->mipi_common_cmd;
    mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

    printk(KERN_DEBUG "[kernel]:ili9805_yt40f139a_init\n");

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
        mipi_common_cmd(lcd_ili9805_yt40f139a_cmds_init);
        printk("[kernel]:>>>>>>>>load local config param success<<<<<<<<<<<<< \n");
    }
    
    mipi_eotp_set(1,1);

    return 0;
}

static uint32_t ili9805_yt40f139a_readreg(struct panel_spec *self)
{
    /*Jessica TODO: need read id*/
    uint8_t read_data[3] = {0};
    int32_t read_rtn = 0;
    uint8_t read_addr_d9[2]= {0xd9,0x00} ;
    uint8_t read_addr_d8[2]= {0xd8,0x00} ;
    uint8_t read_addr[2]= {0x82,0xC5} ;

    mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
    mipi_force_write_t mipi_force_write = self->info.mipi->ops->mipi_force_write;
    mipi_gen_read_t mipi_gen_read = self->info.mipi->ops->mipi_gen_read;
    mipi_force_read_t mipi_force_read = self->info.mipi->ops->mipi_force_read;
    mipi_common_cmd_t mipi_common_cmd = self->info.mipi->ops->mipi_common_cmd;
    mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

    read_data[0] = 0 ;
    read_data[1] = 0 ;
    read_rtn = mipi_force_read(0xD9, 2,(uint8_t *)read_data);  
    printk("1**************************************************\n");
    printk("[kernel][%s %d]rtn %d read 0xD9 is 0x%x%x--------\n",
           __func__,__LINE__,read_rtn,read_data[0],read_data[1]); 

    read_data[0] = 0 ;
    read_data[1] = 0 ;
    read_rtn = mipi_gen_read( read_addr_d9, 2, 2, (uint8_t*)read_data ) ;
    printk("[kernel][%s %d]rtn %d read 0xD9 is 0x%x%x--------\n",
           __func__,__LINE__,read_rtn,read_data[0],read_data[1]); 

    read_data[0] = 0 ;
    read_data[1] = 0 ;
    read_rtn = mipi_gen_read( read_addr_d8, 2, 2, (uint8_t*)read_data ) ;
    printk("[kernel][%s %d]rtn %d read 0xD8 is 0x%x%x--------\n",
           __func__,__LINE__,read_rtn,read_data[0],read_data[1]); 

    read_data[0] = 0 ;
    read_data[1] = 0 ;
    read_rtn = mipi_gen_read( read_addr, 2, 2, (uint8_t*)read_data ) ;
    printk("[kernel][%s %d]rtn %d read 0xC5 is 0x%x%x--------\n",
           __func__,__LINE__,read_rtn,read_data[0],read_data[1]); 
    printk("2**************************************************\n");

    return 0;
}

static uint32_t ili9805_yt40f139a_readid(struct panel_spec *self)
{
    /*Jessica TODO: need read id*/

    return 0x18;
}

static int32_t ili9805_yt40f139a_enter_sleep(struct panel_spec *self, uint8_t is_sleep)
{
    sprdfb_dev_dsi_cmds *sleep_in_or_out = NULL;

    mipi_common_cmd_t mipi_common_cmd = self->info.mipi->ops->mipi_common_cmd;
      mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

    printk(KERN_DEBUG "ili9805_yt40f139a_enter_sleep, is_sleep = %d\n", is_sleep);

    if(is_sleep)
    {
        sleep_in_or_out = sleep_in;
    }
    else
    {
        sleep_in_or_out = sleep_out;
    }

    mipi_eotp_set(1,0);
    mipi_common_cmd(sleep_in_or_out);

    return 0;
}

#ifdef CONFIG_FB_ESD_SUPPORT
static uint32_t ili9805_yt40f139a_readpowermode(struct panel_spec *self)
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

	pr_debug("lcd_ili9805_yt40f139a_mipi read power mode!\n");
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
		pr_debug("lcd_ili9805_yt40f139a_mipi read power mode 0x0A value is 0x%x! , read result(%d)\n", read_data[0], read_rtn);
		if((0x9c == read_data[0])  && (0 == read_rtn)){
			pr_debug("lcd_ili9805_yt40f139aa_mipi read power mode success!\n");
			mipi_eotp_set(1,1);
			return 0x9c;
		}
	}

	printk("lcd_ili9805_yt40f139a_mipi read power mode fail!0x0A value is 0x%x! , read result(%d)\n", read_data[0], read_rtn);
	mipi_eotp_set(1,1);
	return 0x0;
}

static int32_t ili9805_yt40f139a_check_esd(struct panel_spec *self)
{
	uint32_t power_mode;

	mipi_set_lp_mode_t mipi_set_data_lp_mode = self->info.mipi->ops->mipi_set_data_lp_mode;
	mipi_set_hs_mode_t mipi_set_data_hs_mode = self->info.mipi->ops->mipi_set_data_hs_mode;
	mipi_set_lp_mode_t mipi_set_lp_mode = self->info.mipi->ops->mipi_set_lp_mode;
	mipi_set_hs_mode_t mipi_set_hs_mode = self->info.mipi->ops->mipi_set_hs_mode;
	uint16_t work_mode = self->info.mipi->work_mode;

	pr_debug("ili9805_yt40f139a_check_esd!\n");
	if(SPRDFB_MIPI_MODE_CMD==work_mode){
		mipi_set_lp_mode();
	}else{
		mipi_set_data_lp_mode();
	}
	power_mode = ili9805_yt40f139a_readpowermode(self);
	//power_mode = 0x0;
	if(SPRDFB_MIPI_MODE_CMD==work_mode){
		mipi_set_hs_mode();
	}else{
		mipi_set_data_hs_mode();
	}
	if(power_mode == 0x9c){
		pr_debug("ili9805_yt40f139a_check_esd OK!\n");
		return 1;
	}else{
		printk("ili9805_yt40f139a_check_esd fail!(0x%x)\n", power_mode);
		return 0;
	}
}
#endif//-modify by liuwei

static u8 lcd_yc3979_otm8019a_driving_mode_cmd_1[]={0xFF,0xFF,0x98,0x06,0x04,0x01};
static u8 lcd_yc3979_otm8019a_driving_mode_cmd_2[]={0x31,0x00};  
static u8 lcd_yc3979_otm8019a_driving_mode_cmd_3[]={0xFF,0xFF,0x98,0x06,0x04,0x00};
static sprdfb_dev_dsi_cmds lcd_driving_mode_control[] =
{
    { .type = DSI_CMD_GEN,    .len = sizeof(lcd_yc3979_otm8019a_driving_mode_cmd_1), .p_cmd=lcd_yc3979_otm8019a_driving_mode_cmd_1,},
    { .type = DSI_CMD_GEN,    .len = sizeof(lcd_yc3979_otm8019a_driving_mode_cmd_2), .p_cmd=lcd_yc3979_otm8019a_driving_mode_cmd_2,},
    { .type = DSI_CMD_GEN,    .len = sizeof(lcd_yc3979_otm8019a_driving_mode_cmd_3), .p_cmd=lcd_yc3979_otm8019a_driving_mode_cmd_3,},
};
static void ili9805_yt40f139a_driving_mode_set(struct panel_spec *self,bool enable)
{
    sprdfb_dev_dsi_cmds *init = lcd_driving_mode_control;
    mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
    mipi_common_cmd_t mipi_common_cmd = self->info.mipi->ops->mipi_common_cmd;
    mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;
    
    if(enable)//dot mode 
        lcd_driving_mode_control[1].p_cmd[1] = 0x02;
    else //column mode
    lcd_driving_mode_control[1].p_cmd[1] = 0x00;
    
    mipi_set_cmd_mode();
    mipi_eotp_set(0,1);

    mipi_common_cmd(init);
    mipi_eotp_set(1,1);
    
    printk(KERN_INFO "[kernel]:ili9805_yt40f139a_driving_mode_set ok.dot enable=%d.\n",enable);
}
static struct panel_operations lcd_ili9805_yt40f139a_mipi_operations =
{
    .panel_init = ili9805_yt40f139a_mipi_init,
    .panel_readid = ili9805_yt40f139a_readid,
    .panel_enter_sleep = ili9805_yt40f139a_enter_sleep,
#ifdef CONFIG_FB_ESD_SUPPORT
    .panel_esd_check = ili9805_yt40f139a_check_esd,
#endif
    .lcd_set_driving_mode = ili9805_yt40f139a_driving_mode_set,
};

static struct timing_rgb lcd_ili9805_yt40f139a_mipi_timing =
{   
    .hfp = 60,  /* unit: pixel */
    .hbp = 60,
    .hsync = 20,
    .vfp =10, /*unit: line*/
    .vbp = 10,
    .vsync = 6,
};

static struct info_mipi lcd_ili9805_yt40f139a_mipi_info =
{
    .work_mode  = SPRDFB_MIPI_MODE_VIDEO,
    .video_bus_width = 24, /*18,16*/
    .lan_number = 2,
    .phy_feq = 408*1000,
    .h_sync_pol = SPRDFB_POLARITY_POS,
    .v_sync_pol = SPRDFB_POLARITY_POS,
    .de_pol = SPRDFB_POLARITY_POS,
    .te_pol = SPRDFB_POLARITY_POS,
    .color_mode_pol = SPRDFB_POLARITY_NEG,
    .shut_down_pol = SPRDFB_POLARITY_NEG,
    .timing = &lcd_ili9805_yt40f139a_mipi_timing,
    .ops = NULL,
};

struct panel_spec lcd_ili9805_yt40f139a_mipi_spec =
{
    .width  = 480,
    .height = 800,
    .fps    = 60, 
    .reset_timing = {5,40,120},
    .type   = LCD_MODE_DSI,
    .direction    = LCD_DIRECT_NORMAL,
    .is_clean_lcd = true,
    .suspend_mode = SEND_SLEEP_CMD ,
    .info = {
        .mipi = &lcd_ili9805_yt40f139a_mipi_info
    },
    .ops = &lcd_ili9805_yt40f139a_mipi_operations,
    .mipi_clk_level = 0,
};

struct panel_cfg lcd_ili9805_yt40f139a_mipi =
{
    /* this panel can only be main lcd */
    .dev_id = SPRDFB_MAINLCD_ID,
    .lcd_id = 0x18,
    .lcd_name = "lcd_ili9805_yt40f139a_mipi",
    .lcd_voltage_max = 1200,
    .lcd_voltage_min = 1100,
    .panel = &lcd_ili9805_yt40f139a_mipi_spec,
};

static int __init lcd_ili9805_yt40f139a_mipi_init(void)
{
    return sprdfb_panel_register(&lcd_ili9805_yt40f139a_mipi);
}

subsys_initcall(lcd_ili9805_yt40f139a_mipi_init);

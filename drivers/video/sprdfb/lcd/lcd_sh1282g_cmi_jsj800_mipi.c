/* drivers/video/sc8825/lcd_sh1282g_cmi_jsj800_mipi.c
 *
 * Support for sh1282g_cmi_jsj800 mipi LCD device
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
//+modify by liuwei
typedef struct LCM_Init_Code_tag
{
    unsigned int tag;
    unsigned char data[MAX_DATA];
} LCM_Init_Code;
typedef struct LCM_force_cmd_code_tag
{
    unsigned int datatype;
    LCM_Init_Code real_cmd_code;
} LCM_Force_Cmd_Code;

#define LCM_TAG_SHIFT 24
#define LCM_TAG_MASK  ((1 << 24) -1)
#define LCM_SEND(len) ((1 << LCM_TAG_SHIFT)| len)
#define LCM_SLEEP(ms) ((2 << LCM_TAG_SHIFT)| ms)
//#define ARRAY_SIZE(array) ( sizeof(array) / sizeof(array[0]))
#define LCM_TAG_SEND  (1<< 0)
#define LCM_TAG_SLEEP (1 << 1)


static u8 lcd_sh1282g_cmi_jsj800_cmd_1[]= {0xf0,0x55,0xaa,0x52,0x08,0x00};
static u8 lcd_sh1282g_cmi_jsj800_cmd_2[]= {0xb0,0x00,0x1f,0x0a,0x5a,0x0a};
static u8 lcd_sh1282g_cmi_jsj800_cmd_3[]= {0xb1,0x78,0x00};
static u8 lcd_sh1282g_cmi_jsj800_cmd_4[]= {0xb5,0x50};//864*480
static u8 lcd_sh1282g_cmi_jsj800_cmd_5[]= {0xb6,0x14};//SourceOutputDataHoldTimeControl
static u8 lcd_sh1282g_cmi_jsj800_cmd_6[]= {0xB7,0x23,0x00,0x23,0x00};
static u8 lcd_sh1282g_cmi_jsj800_cmd_7[]= {0xB8,0x00,0x10,0x10,0x10};
static u8 lcd_sh1282g_cmi_jsj800_cmd_8[]= {0xbc,0x02,0x02,0x02};//InversionDrivingControl
static u8 lcd_sh1282g_cmi_jsj800_cmd_9[]= {0xC1,0x01,0x00,0x17,0x16,0x09,0x07,0x0D,0x0B,0x03,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x02,0x0A,0x0C,0x06,0x08,0x16,0x17,0x00,0x01};
static u8 lcd_sh1282g_cmi_jsj800_cmd_10[]={0xC2,0x86,0x33,0x28,0x85,0x28,0x84,0x33,0x28,0x83,0x28};
static u8 lcd_sh1282g_cmi_jsj800_cmd_11[]={0xC3,0x82,0x73,0x03,0x28,0x28,0x00,0x00,0x81,0x73,0x03,0x29,0x28,0x00,0x00,0x00,0x73,0x03,0x2A,0x28,0x00,0x00,0x01,0x73,0x03,0x2B,0x28,0x00,0x00,0x02,0x73,0x03,0x2C,0x28,0x00,0x00,0x03,0x73,0x03,0x2D,0x28,0x00,0x00};
static u8 lcd_sh1282g_cmi_jsj800_cmd_12[]={0xC4,0x04,0x73,0x03,0x2E,0x28,0x00,0x00,0x05,0x73,0x03,0x2F,0x28,0x00,0x00};
static u8 lcd_sh1282g_cmi_jsj800_cmd_13[]={0xC6,0x28,0x00,0x20,0x28,0x00,0x21,0x28,0x00,0x21,0x28,0x00,0x20};
static u8 lcd_sh1282g_cmi_jsj800_cmd_14[]={0xf0,0x55,0xaa,0x52,0x08,0x01};//SelectCommandPage1
static u8 lcd_sh1282g_cmi_jsj800_cmd_15[]={0xb0,0x0d};//SettingAVDDVoltage
static u8 lcd_sh1282g_cmi_jsj800_cmd_16[]={0xb6,0x34};//BT4PowerControlforVGH
static u8 lcd_sh1282g_cmi_jsj800_cmd_17[]={0xb1,0x0b};//SettingAVEEVoltage
static u8 lcd_sh1282g_cmi_jsj800_cmd_18[]={0xb7,0x34};
static u8 lcd_sh1282g_cmi_jsj800_cmd_19[]={0xb2,0x00};//SettingVCLVoltage
static u8 lcd_sh1282g_cmi_jsj800_cmd_20[]={0xbf,0x01};
static u8 lcd_sh1282g_cmi_jsj800_cmd_21[]={0xb3,0x16};//SettingVGHVoltage
static u8 lcd_sh1282g_cmi_jsj800_cmd_22[]={0xb9,0x35};//SettingVGL_REGVoltage/*
static u8 lcd_sh1282g_cmi_jsj800_cmd_23[]={0xba,0x25};//BT5PowerControlforVGLX
static u8 lcd_sh1282g_cmi_jsj800_cmd_24[]={0xbc,0x00,0x7b,0x00};//SettingVGMPandVGSPVoltage
static u8 lcd_sh1282g_cmi_jsj800_cmd_25[]={0xbd,0x00,0x7b,0x00};//SettingVGMNandVGSNVoltage
static u8 lcd_sh1282g_cmi_jsj800_cmd_26[]={0xbe,0x00,0x7c};//6a//SettingVCOMOffsetVoltage
static u8 lcd_sh1282g_cmi_jsj800_cmd_27[]={0xD1,0x00,0x00,0x00,0x12,0x00,0x30,0x00,0x4C,0x00,0x61,0x00,0x88,0x00,0xA7,0x00,0xD9,0x01,0x03,0x01,0x44,0x01,0x76,0x01,0xC7,0x02,0x09,0x02,0x0B,0x02,0x49,0x02,0x8F,0x02,0xBA,0x02,0xF5,0x03,0x1C,0x03,0x50,0x03,0x73,0x03,0xA3,0x03,0xBF,0x03,0xEA,0x03,0xF9,0x03,0xFA};
static u8 lcd_sh1282g_cmi_jsj800_cmd_28[]={0xD2,0x00,0x00,0x00,0x12,0x00,0x30,0x00,0x4C,0x00,0x61,0x00,0x88,0x00,0xA7,0x00,0xD9,0x01,0x03,0x01,0x44,0x01,0x76,0x01,0xC7,0x02,0x09,0x02,0x0B,0x02,0x49,0x02,0x8F,0x02,0xBA,0x02,0xF5,0x03,0x1C,0x03,0x50,0x03,0x73,0x03,0xA3,0x03,0xBF,0x03,0xEA,0x03,0xF9,0x03,0xFA};
static u8 lcd_sh1282g_cmi_jsj800_cmd_29[]={0x8A,0x13};
static u8 lcd_sh1282g_cmi_jsj800_cmd_30[]={0x85,0x11,0x44,0x44,0x11,0x44,0x44};
static u8 lcd_sh1282g_cmi_jsj800_cmd_31[]={0x3a,0x70};//24bit
static u8 lcd_sh1282g_cmi_jsj800_cmd_32[]={0x11};//Sleepout
static u8 lcd_sh1282g_cmi_jsj800_cmd_33[]={0x29};//DisplayOn

static sprdfb_dev_dsi_cmds lcd_sh1282g_cmi_jsj800_cmds_init[] =
{
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_1), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_1,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_2), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_2,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_3), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_3,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_4), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_4,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_5), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_5,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_6), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_6,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_7), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_7,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_8), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_8,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_9), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_9,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_10), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_10,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_11), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_11,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_12), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_12,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_13), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_13,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_14), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_14,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_15), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_15,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_16), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_16,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_17), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_17,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_18), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_18,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_19), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_19,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_20), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_20,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_21), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_21,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_22), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_22,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_23), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_23,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_24), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_24,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_25), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_25,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_26), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_26,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_27), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_27,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_28), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_28,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_29), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_29,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_30), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_30,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_31), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_31,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_32), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_32,},
	  { .type = DSI_CMD_DEALY,	.len = 120, },
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_cmd_33), .p_cmd=lcd_sh1282g_cmi_jsj800_cmd_33,},
	  { .type = DSI_CMD_DEALY,	.len = 20, },

    { .type = DSI_CMD_END,},

};

static sprdfb_dev_dsi_cmds disp_on = { .type = DSI_CMD_DCS,	.len = 1, .cmd={0x29},};

static sprdfb_dev_dsi_cmds sleep_in[] =
{
    { .type = DSI_CMD_DCS,	.len = 1, .cmd={0x28},},
    { .type = DSI_CMD_DEALY,	.len = 10, },
    { .type = DSI_CMD_DCS,	.len = 1, .cmd={0x10},},
    { .type = DSI_CMD_DEALY,	.len = 120, },
    { .type = DSI_CMD_END,},

};

static sprdfb_dev_dsi_cmds sleep_out[] =
{
    { .type = DSI_CMD_DCS,	.len = 1, .cmd={0x11},},
    { .type = DSI_CMD_DEALY,	.len = 120, },
    { .type = DSI_CMD_DCS,	.len = 1, .cmd={0x29},},
    { .type = DSI_CMD_DEALY,	.len = 20, },
    { .type = DSI_CMD_END,},
};

extern unsigned int lcd_debug_panel_on_cmds( sprdfb_dev_dsi_cmds**cmds, unsigned char** buf) ;

static int32_t sh1282g_cmi_jsj800_mipi_init(struct panel_spec *self)
{
    sprdfb_dev_dsi_cmds *dbg_cmds = NULL;
    unsigned char* dbg_data= NULL ;
    unsigned int size = 0;
    uint8_t read_data[3] = {0};
    int32_t read_rtn = 0;

    mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
    mipi_common_cmd_t mipi_common_cmd = self->info.mipi->ops->mipi_common_cmd;
    mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;
    mipi_force_read_t mipi_force_read = self->info.mipi->ops->mipi_force_read;

    printk(KERN_DEBUG "[kernel]:sh1282g_cmi_jsj800_init\n");

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
        mipi_common_cmd(lcd_sh1282g_cmi_jsj800_cmds_init);
        printk("[kernel]:>>>>>>>>load local config param success<<<<<<<<<<<<< \n");
    }
    #if 0
    read_rtn = mipi_force_read(0x0A, 1,(uint8_t *)read_data);  
    printk("**************************************************\n");
    printk("[kernel][%s %d]rtn %d read 0x0A is 0x%2.2x%2.2x--------\n",
           __func__,__LINE__,read_rtn,read_data[0],read_data[1]); 
    printk("**************************************************\n");
    #endif
    mipi_eotp_set(1,1);
    return 0;
}

static uint32_t sh1282g_cmi_jsj800_readid(struct panel_spec *self)
{
    return 0x18;
}

static int32_t sh1282g_cmi_jsj800_enter_sleep(struct panel_spec *self, uint8_t is_sleep)
{
    sprdfb_dev_dsi_cmds *sleep_in_or_out = NULL;

    mipi_common_cmd_t mipi_common_cmd = self->info.mipi->ops->mipi_common_cmd;
    mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

    printk(KERN_DEBUG "sh1282g_cmi_jsj800_enter_sleep, is_sleep = %d\n", is_sleep);

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

static uint32_t sh1282g_cmi_jsj800_readpowermode(struct panel_spec *self)
{
	  static LCM_Force_Cmd_Code rd_prep_code_1[]={
	    {0x39, {LCM_SEND(8), {0x6, 0, 0xF0,0x55,0xAA,0x52,0x08,0x01}}},
	    {0x37, {LCM_SEND(2), {0x5, 0}}},
    };

    int32_t i = 0;
    uint32_t j =0;
    LCM_Force_Cmd_Code * rd_prepare = rd_prep_code_1;
    uint8_t read_data[1] = {0};
    int32_t read_rtn = 0;
    unsigned int tag = 0;
    uint32_t reg_val_1 = 0;
    uint32_t reg_val_2 = 0;

    mipi_force_write_t mipi_force_write = self->info.mipi->ops->mipi_force_write;
    mipi_force_read_t mipi_force_read = self->info.mipi->ops->mipi_force_read;
    mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;
    mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;

    printk("lcd_sh1282g_cmi_jsj800_mipi read power mode!\n");
    mipi_set_cmd_mode();
    mipi_eotp_set(1,0);

    for(j = 0; j < 4; j++)
    {
        rd_prepare = rd_prep_code_1;
        for(i = 0; i < ARRAY_SIZE(rd_prep_code_1); i++)
        {
            tag = (rd_prepare->real_cmd_code.tag >> 24);
            if(tag & LCM_TAG_SEND)
            {
                mipi_force_write(rd_prepare->datatype, rd_prepare->real_cmd_code.data, 
                                 (rd_prepare->real_cmd_code.tag & LCM_TAG_MASK));
            }
            else if(tag & LCM_TAG_SLEEP)
            {
                msleep((rd_prepare->real_cmd_code.tag & LCM_TAG_MASK));
            }
            rd_prepare++;
        }

        read_rtn = mipi_force_read(0x0A, 1,(uint8_t *)read_data);
        printk("lcd_sh1282g_cmi_jsj800 mipi read power mode 0x0A value is 0x%x! , read result(%d)\n",
               read_data[0], read_rtn);
        mipi_eotp_set(1,1);
        if((0 == read_data[0])  && (0 == read_rtn))
        {
            printk("lcd_sh1282g_cmi_jsj800_mipi read power mode success!\n");
            return 0x9c;
        }
    }
    return 0x00;
}


static uint32_t sh1282g_cmi_jsj800_check_esd(struct panel_spec *self)
{
    uint32_t power_mode;

    printk("sh1282g_cmi_jsj800_check_esd!\n");
    mipi_set_lp_mode_t mipi_set_data_lp_mode = self->info.mipi->ops->mipi_set_data_lp_mode;
    mipi_set_hs_mode_t mipi_set_data_hs_mode = self->info.mipi->ops->mipi_set_data_hs_mode;

    mipi_set_lp_mode_t mipi_set_lp_mode = self->info.mipi->ops->mipi_set_lp_mode;
    mipi_set_hs_mode_t mipi_set_hs_mode = self->info.mipi->ops->mipi_set_hs_mode;
    uint16_t work_mode = self->info.mipi->work_mode;

    if(SPRDFB_MIPI_MODE_CMD==work_mode)
    {
        mipi_set_lp_mode();
    }
    else
    {
        mipi_set_data_lp_mode();
    }
    power_mode = sh1282g_cmi_jsj800_readpowermode(self);
    //power_mode = 0x0;
    if(SPRDFB_MIPI_MODE_CMD==work_mode)
    {
        mipi_set_hs_mode();
    }
    else
    {
        mipi_set_data_hs_mode();
    }
    if(power_mode == 0x9c)
    {
        printk("sh1282g_cmi_jsj800_check_esd OK!\n");
        return 1;
    }
    else
    {
        printk("sh1282g_cmi_jsj800_check_esd fail!(0x%x)\n", power_mode);
        return 0;
    }
}

#endif//-modify by liuwei

static u8 lcd_sh1282g_cmi_jsj800_driving_mode_cmd_1[]= {0xFF,0xFF,0x98,0x06,0x04,0x01};
static u8 lcd_sh1282g_cmi_jsj800_driving_mode_cmd_2[]= {0x31,0x00};
static u8 lcd_sh1282g_cmi_jsj800_driving_mode_cmd_3[]= {0xFF,0xFF,0x98,0x06,0x04,0x00};
static sprdfb_dev_dsi_cmds lcd_driving_mode_control[] =
{
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_driving_mode_cmd_1), .p_cmd=lcd_sh1282g_cmi_jsj800_driving_mode_cmd_1,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_driving_mode_cmd_2), .p_cmd=lcd_sh1282g_cmi_jsj800_driving_mode_cmd_2,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_sh1282g_cmi_jsj800_driving_mode_cmd_3), .p_cmd=lcd_sh1282g_cmi_jsj800_driving_mode_cmd_3,},
};
static void sh1282g_cmi_jsj800_driving_mode_set(struct panel_spec *self,bool enable)
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

    printk(KERN_INFO "[kernel]:sh1282g_cmi_jsj800_driving_mode_set ok.dot enable=%d.\n",enable);
}
static struct panel_operations lcd_sh1282g_cmi_jsj800_mipi_operations =
{
    .panel_init = sh1282g_cmi_jsj800_mipi_init,
    .panel_readid = sh1282g_cmi_jsj800_readid,
    .panel_enter_sleep = sh1282g_cmi_jsj800_enter_sleep,
#ifdef CONFIG_FB_ESD_SUPPORT
    .panel_esd_check = sh1282g_cmi_jsj800_check_esd,
#endif
    //.lcd_set_driving_mode = sh1282g_cmi_jsj800_driving_mode_set,
};

static struct timing_rgb lcd_sh1282g_cmi_jsj800_mipi_timing =
{   
	.hfp = 20,  /* unit: pixel */
    .hbp = 120,
    .hsync = 6,
    .vfp =20, /*unit: line*/
    .vbp = 26,
    .vsync = 4,
};

static struct info_mipi lcd_sh1282g_cmi_jsj800_mipi_info =
{
    .work_mode  = SPRDFB_MIPI_MODE_VIDEO,
    .video_bus_width = 24, /*18,16*/
    .lan_number = 2,
    .phy_feq = 350*1000,
    .h_sync_pol = SPRDFB_POLARITY_POS,
    .v_sync_pol = SPRDFB_POLARITY_POS,
    .de_pol = SPRDFB_POLARITY_POS,
    .te_pol = SPRDFB_POLARITY_POS,
    .color_mode_pol = SPRDFB_POLARITY_NEG,
    .shut_down_pol = SPRDFB_POLARITY_NEG,
    .timing = &lcd_sh1282g_cmi_jsj800_mipi_timing,
    .ops = NULL,
};

struct panel_spec lcd_sh1282g_cmi_jsj800_mipi_spec =
{
    .width  = 480,
    .height = 800,
    .fps    = 45,
    .type   = LCD_MODE_DSI,
    .direction    = LCD_DIRECT_NORMAL,
    .is_clean_lcd = true,
    .suspend_mode = SEND_SLEEP_CMD ,
    .info = {
        .mipi = &lcd_sh1282g_cmi_jsj800_mipi_info
    },
    .ops = &lcd_sh1282g_cmi_jsj800_mipi_operations,
    .mipi_clk_level = 0,
};

struct panel_cfg lcd_sh1282g_cmi_jsj800_mipi =
{
    /* this panel can only be main lcd */
    .dev_id = SPRDFB_MAINLCD_ID,
    .lcd_id = 0x18,
    .lcd_name = "lcd_sh1282g_cmi_jsj800_mipi",
    .lcd_voltage_max = 1600 ,
    .lcd_voltage_min = 1000 ,
    .panel = &lcd_sh1282g_cmi_jsj800_mipi_spec,
};

static int __init lcd_sh1282g_cmi_jsj800_mipi_init(void)
{
    return sprdfb_panel_register(&lcd_sh1282g_cmi_jsj800_mipi);
}

subsys_initcall(lcd_sh1282g_cmi_jsj800_mipi_init);

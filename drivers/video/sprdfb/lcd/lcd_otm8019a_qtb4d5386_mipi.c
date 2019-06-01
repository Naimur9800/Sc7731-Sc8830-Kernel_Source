/* drivers/video/sc8825/lcd_otm8019a_qtb4d5386_mipi.c
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
//+modify by liuwei
#if 0
typedef struct LCM_Init_Code_tag
{
    unsigned int tag;
    unsigned char data[MAX_DATA];
} LCM_Init_Code;

#define LCM_TAG_SHIFT 24
#define LCM_TAG_MASK  ((1 << 24) -1)
#define LCM_SEND(len) ((1 << LCM_TAG_SHIFT)| len)
#define LCM_SLEEP(ms) ((2 << LCM_TAG_SHIFT)| ms)
//#define ARRAY_SIZE(array) ( sizeof(array) / sizeof(array[0]))
#define LCM_TAG_SEND  (1<< 0)
#define LCM_TAG_SLEEP (1 << 1)
#endif


static u8 lcd_otm8019a_qtb4d5386_cmd_1[]= {0x00,0x00};
static u8 lcd_otm8019a_qtb4d5386_cmd_2[]= {0xFF,0x80,0x19,0x01};//
static u8 lcd_otm8019a_qtb4d5386_cmd_3[]= {0x00,0x80};
static u8 lcd_otm8019a_qtb4d5386_cmd_4[]= {0xFF,0x80,0x19};//
static u8 lcd_otm8019a_qtb4d5386_cmd_5[]= {0x00,0xB4};
static u8 lcd_otm8019a_qtb4d5386_cmd_6[]= {0xC0,0x00};//
static u8 lcd_otm8019a_qtb4d5386_cmd_7[]= {0x00,0x81};
static u8 lcd_otm8019a_qtb4d5386_cmd_8[]= {0xC5,0x66};
static u8 lcd_otm8019a_qtb4d5386_cmd_9[]= {0x00,0x82};
static u8 lcd_otm8019a_qtb4d5386_cmd_10[]={0xC5,0xB0};//
static u8 lcd_otm8019a_qtb4d5386_cmd_11[]={0x00,0x90};
static u8 lcd_otm8019a_qtb4d5386_cmd_12[]={0xC5,0x4E,0x9d};//
static u8 lcd_otm8019a_qtb4d5386_cmd_13[]={0x00,0x00};
static u8 lcd_otm8019a_qtb4d5386_cmd_14[]={0xD8,0x69,0x69};//
static u8 lcd_otm8019a_qtb4d5386_cmd_15[]={0x00,0x00};
static u8 lcd_otm8019a_qtb4d5386_cmd_16[]={0xD9,0x3a};
static u8 lcd_otm8019a_qtb4d5386_cmd_17[]={0x00,0x81};
static u8 lcd_otm8019a_qtb4d5386_cmd_18[]={0xC1,0x33};//
static u8 lcd_otm8019a_qtb4d5386_cmd_19[]={0x00,0xA1};
static u8 lcd_otm8019a_qtb4d5386_cmd_20[]={0xC1,0x08};
static u8 lcd_otm8019a_qtb4d5386_cmd_21[]={0x00,0x81};
static u8 lcd_otm8019a_qtb4d5386_cmd_22[]={0xC4,0x83};
static u8 lcd_otm8019a_qtb4d5386_cmd_23[]={0x00,0xB1};
static u8 lcd_otm8019a_qtb4d5386_cmd_24[]={0xC5,0xA9};
static u8 lcd_otm8019a_qtb4d5386_cmd_25[]={0x00,0x93};
static u8 lcd_otm8019a_qtb4d5386_cmd_26[]={0xC5,0x03};
static u8 lcd_otm8019a_qtb4d5386_cmd_27[]={0x00,0x92};
static u8 lcd_otm8019a_qtb4d5386_cmd_28[]={0xB3,0x40};//
static u8 lcd_otm8019a_qtb4d5386_cmd_29[]={0x00,0x90};
static u8 lcd_otm8019a_qtb4d5386_cmd_30[]={0xB3,0x02};
static u8 lcd_otm8019a_qtb4d5386_cmd_31[]={0x00,0x80};
static u8 lcd_otm8019a_qtb4d5386_cmd_32[]={0xC0,0x00,0x58,0x00,0x15,0x15,0x00,0x58,0x15,0x15};
static u8 lcd_otm8019a_qtb4d5386_cmd_33[]={0x00,0x90};
static u8 lcd_otm8019a_qtb4d5386_cmd_34[]={0xC0,0x00,0x15,0x00,0x00,0x00,0x03};//
static u8 lcd_otm8019a_qtb4d5386_cmd_35[]={0x00,0x80};
static u8 lcd_otm8019a_qtb4d5386_cmd_36[]={0xCE,0x8B,0x03,0x00,0x8A,0x03,0x00,0x89,0x03,0x00,0x88,0x03,0x00};
static u8 lcd_otm8019a_qtb4d5386_cmd_37[]={0x00,0xA0};
static u8 lcd_otm8019a_qtb4d5386_cmd_38[]={0xCE,0x38,0x07,0x03,0x54,0x00,0x02,0x00,0x38,0x06,0x03,0x55,0x00,0x02,0x00};
static u8 lcd_otm8019a_qtb4d5386_cmd_39[]={0x00,0xB0};
static u8 lcd_otm8019a_qtb4d5386_cmd_40[]={0xCE,0x38,0x05,0x03,0x56,0x00,0x02,0x00,0x38,0x04,0x03,0x57,0x00,0x02,0x00};
static u8 lcd_otm8019a_qtb4d5386_cmd_41[]={0x00,0xC0};
static u8 lcd_otm8019a_qtb4d5386_cmd_42[]={0xCE,0x38,0x03,0x03,0x58,0x00,0x02,0x00,0x38,0x02,0x03,0x59,0x00,0x02,0x00};
static u8 lcd_otm8019a_qtb4d5386_cmd_43[]={0x00,0xD0};
static u8 lcd_otm8019a_qtb4d5386_cmd_44[]={0xCE,0x38,0x01,0x03,0x5A,0x00,0x02,0x00,0x38,0x00,0x03,0x5B,0x00,0x02,0x00};
static u8 lcd_otm8019a_qtb4d5386_cmd_45[]={0x00,0xC0};
static u8 lcd_otm8019a_qtb4d5386_cmd_46[]={0xCF,0x02,0x02,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00};
static u8 lcd_otm8019a_qtb4d5386_cmd_47[]={0x00,0x90};
static u8 lcd_otm8019a_qtb4d5386_cmd_48[]={0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static u8 lcd_otm8019a_qtb4d5386_cmd_49[]={0x00,0xA0};
static u8 lcd_otm8019a_qtb4d5386_cmd_50[]={0xCB,0x00};
static u8 lcd_otm8019a_qtb4d5386_cmd_51[]={0x00,0xA5};
static u8 lcd_otm8019a_qtb4d5386_cmd_52[]={0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static u8 lcd_otm8019a_qtb4d5386_cmd_53[]={0x00,0xB0};
static u8 lcd_otm8019a_qtb4d5386_cmd_54[]={0xCB,0x00,0x00,0x00,0x00,0x00,0x00};
static u8 lcd_otm8019a_qtb4d5386_cmd_55[]={0x00,0xC0};
static u8 lcd_otm8019a_qtb4d5386_cmd_56[]={0xCB,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00};
static u8 lcd_otm8019a_qtb4d5386_cmd_57[]={0x00,0xD0};
static u8 lcd_otm8019a_qtb4d5386_cmd_58[]={0xCB,0x00};
static u8 lcd_otm8019a_qtb4d5386_cmd_59[]={0x00,0xD5};
static u8 lcd_otm8019a_qtb4d5386_cmd_60[]={0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01};
static u8 lcd_otm8019a_qtb4d5386_cmd_61[]={0x00,0xE0};
static u8 lcd_otm8019a_qtb4d5386_cmd_62[]={0xCB,0x01,0x01,0x01,0x01,0x01,0x01};
static u8 lcd_otm8019a_qtb4d5386_cmd_63[]={0x00,0x80};
static u8 lcd_otm8019a_qtb4d5386_cmd_64[]={0xCC,0x26,0x25,0x21,0x22,0x0C,0x0A,0x10,0x0E,0x02,0x04};
static u8 lcd_otm8019a_qtb4d5386_cmd_65[]={0x00,0x90};
static u8 lcd_otm8019a_qtb4d5386_cmd_66[]={0xCC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static u8 lcd_otm8019a_qtb4d5386_cmd_67[]={0x00,0xA0};
static u8 lcd_otm8019a_qtb4d5386_cmd_68[]={0xCC,0x00,0x03,0x01,0x0D,0x0F,0x09,0x0B,0x22,0x21,0x25,0x26};
static u8 lcd_otm8019a_qtb4d5386_cmd_69[]={0x00,0xB0};
static u8 lcd_otm8019a_qtb4d5386_cmd_70[]={0xCC,0x25,0x26,0x21,0x22,0x10,0x0A,0x0C,0x0E,0x04,0x02};
static u8 lcd_otm8019a_qtb4d5386_cmd_71[]={0x00,0xC0};
static u8 lcd_otm8019a_qtb4d5386_cmd_72[]={0xCC,0x00,0x00,0x00,0x00,0x00,0x00};
static u8 lcd_otm8019a_qtb4d5386_cmd_73[]={0x00,0xCA};
static u8 lcd_otm8019a_qtb4d5386_cmd_74[]={0xCC,0x00,0x00,0x00,0x00,0x00};
static u8 lcd_otm8019a_qtb4d5386_cmd_75[]={0x00,0xD0};
static u8 lcd_otm8019a_qtb4d5386_cmd_76[]={0xCC,0x00,0x01,0x03,0x0D,0x0B,0x09,0x0F,0x22,0x21,0x26,0x25};
static u8 lcd_otm8019a_qtb4d5386_cmd_77[]={0x00,0x00}; 
static u8 lcd_otm8019a_qtb4d5386_cmd_78[]={0xE1,0x00,0x02,0x06,0x11,0x24,0x38,0x41,0x75,0x67,0x7f,0x86,0x74,0x8b,0x75,0x79,0x71,0x69,0x58,0x4d,0x00}; 
static u8 lcd_otm8019a_qtb4d5386_cmd_79[]={0x00,0x00}; 
static u8 lcd_otm8019a_qtb4d5386_cmd_80[]={0xE2,0x00,0x02,0x06,0x11,0x24,0x38,0x41,0x75,0x68,0x7f,0x86,0x74,0x8b,0x74,0x79,0x70,0x68,0x58,0x4e,0x00}; 
static u8 lcd_otm8019a_qtb4d5386_cmd_81[]={0x00,0x80};
static u8 lcd_otm8019a_qtb4d5386_cmd_82[]={0xC4,0x30};	
static u8 lcd_otm8019a_qtb4d5386_cmd_83[]={0x00,0x98};
static u8 lcd_otm8019a_qtb4d5386_cmd_84[]={0xC0,0x00};	//
static u8 lcd_otm8019a_qtb4d5386_cmd_85[]={0x00,0xa9};
static u8 lcd_otm8019a_qtb4d5386_cmd_86[]={0xC0,0x0A};	//0x06
static u8 lcd_otm8019a_qtb4d5386_cmd_87[]={0x00,0xb0};
static u8 lcd_otm8019a_qtb4d5386_cmd_88[]={0xC1,0x20,0x00,0x00}; //
static u8 lcd_otm8019a_qtb4d5386_cmd_89[]={0x00,0xe1};
static u8 lcd_otm8019a_qtb4d5386_cmd_90[]={0xC0,0x40,0x30}; //0x40,0x18
static u8 lcd_otm8019a_qtb4d5386_cmd_91[]={0x00,0x80};
static u8 lcd_otm8019a_qtb4d5386_cmd_92[]={0xC1,0x03,0x33};
static u8 lcd_otm8019a_qtb4d5386_cmd_93[]={0x00,0xA0};
static u8 lcd_otm8019a_qtb4d5386_cmd_94[]={0xC1,0xe8};
static u8 lcd_otm8019a_qtb4d5386_cmd_95[]={0x00,0x90};
static u8 lcd_otm8019a_qtb4d5386_cmd_96[]={0xb6,0xb4};	//command fial
static u8 lcd_otm8019a_qtb4d5386_cmd_97[]={0x00,0x00};
static u8 lcd_otm8019a_qtb4d5386_cmd_98[]={0xfb,0x01};
static u8 lcd_otm8019a_qtb4d5386_cmd_99[]={0x00,0x00};
static u8 lcd_otm8019a_qtb4d5386_cmd_100[]={0xFF,0xFF,0xFF,0xFF};
static u8 lcd_otm8019a_qtb4d5386_cmd_101[]={0x11};
static u8 lcd_otm8019a_qtb4d5386_cmd_102[]={0x29};

static sprdfb_dev_dsi_cmds lcd_otm8019a_qtb4d5386_cmds_init[] =
{
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_1), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_1,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_2), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_2,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_3), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_3,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_4), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_4,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_5), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_5,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_6), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_6,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_7), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_7,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_8), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_8,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_9), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_9,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_10), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_10,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_11), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_11,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_12), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_12,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_13), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_13,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_14), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_14,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_15), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_15,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_16), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_16,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_17), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_17,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_18), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_18,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_19), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_19,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_20), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_20,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_21), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_21,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_22), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_22,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_23), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_23,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_24), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_24,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_25), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_25,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_26), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_26,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_27), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_27,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_28), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_28,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_29), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_29,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_30), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_30,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_31), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_31,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_32), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_32,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_33), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_33,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_34), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_34,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_35), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_35,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_36), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_36,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_37), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_37,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_38), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_38,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_39), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_39,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_40), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_40,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_41), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_41,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_42), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_42,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_43), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_43,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_44), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_44,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_45), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_45,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_46), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_46,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_47), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_47,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_48), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_48,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_49), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_49,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_50), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_50,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_51), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_51,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_52), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_52,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_53), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_53,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_54), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_54,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_55), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_55,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_56), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_56,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_57), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_57,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_58), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_58,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_59), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_59,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_60), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_60,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_61), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_61,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_62), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_62,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_63), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_63,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_64), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_64,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_65), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_65,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_66), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_66,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_67), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_67,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_68), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_68,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_69), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_69,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_70), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_70,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_71), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_71,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_72), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_72,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_73), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_73,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_74), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_74,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_75), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_75,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_76), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_76,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_77), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_77,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_78), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_78,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_79), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_79,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_80), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_80,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_81), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_81,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_82), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_82,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_83), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_83,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_84), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_84,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_85), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_85,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_86), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_86,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_87), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_87,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_88), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_88,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_89), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_89,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_90), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_90,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_91), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_91,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_92), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_92,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_93), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_93,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_94), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_94,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_95), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_95,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_96), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_96,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_97), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_97,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_98), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_98,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_99), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_99,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_100), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_100,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_101), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_101,},
    { .type = DSI_CMD_DEALY,	.len = 120, },
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_otm8019a_qtb4d5386_cmd_102), .p_cmd=lcd_otm8019a_qtb4d5386_cmd_102,},
    { .type = DSI_CMD_DEALY,	.len = 20, },
    { .type = DSI_CMD_END,},

};

static sprdfb_dev_dsi_cmds sleep_in[] =
{
	{ .type = DSI_CMD_GEN,	.len = 1, .cmd={0x22},},
    { .type = DSI_CMD_DEALY,	.len = 20, },
    { .type = DSI_CMD_GEN,	.len = 1, .cmd={0x28},},
    { .type = DSI_CMD_DEALY,	.len = 120, },
    { .type = DSI_CMD_GEN,	.len = 1, .cmd={0x10},},
    { .type = DSI_CMD_DEALY,	.len = 120, },
    { .type = DSI_CMD_END,},

};

static sprdfb_dev_dsi_cmds sleep_out[] =
{
    { .type = DSI_CMD_GEN,	.len = 1, .cmd={0x11},},
    { .type = DSI_CMD_DEALY,	.len = 120, },
    { .type = DSI_CMD_GEN,	.len = 1, .cmd={0x29},},
    { .type = DSI_CMD_DEALY,	.len = 20, },
    { .type = DSI_CMD_END,},
};

extern unsigned int lcd_debug_panel_on_cmds( sprdfb_dev_dsi_cmds**cmds, unsigned char** buf) ;
static uint32_t otm8019a_qtb4d5386_readreg(struct panel_spec *self) ;

static int32_t otm8019a_qtb4d5386_mipi_init(struct panel_spec *self)
{
    sprdfb_dev_dsi_cmds *dbg_cmds = NULL;
    unsigned char* dbg_data= NULL ;
    unsigned int size = 0;
    
    mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
    mipi_force_read_t mipi_force_read = self->info.mipi->ops->mipi_force_read;
    mipi_common_cmd_t mipi_common_cmd = self->info.mipi->ops->mipi_common_cmd;
    mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

    printk(KERN_DEBUG "[kernel]:otm8019a_qtb4d5386_init\n");

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
        mipi_common_cmd(lcd_otm8019a_qtb4d5386_cmds_init);
        printk("[kernel]:>>>>>>>>load local config param success<<<<<<<<<<<<< \n");
    }
    
    mipi_eotp_set(1,1);

    return 0;
}

static uint32_t otm8019a_qtb4d5386_readreg(struct panel_spec *self)
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

static uint32_t otm8019a_qtb4d5386_readid(struct panel_spec *self)
{
    /*Jessica TODO: need read id*/

    return 0x18;
}

static int32_t otm8019a_qtb4d5386_enter_sleep(struct panel_spec *self, uint8_t is_sleep)
{
    sprdfb_dev_dsi_cmds *sleep_in_or_out = NULL;

    mipi_common_cmd_t mipi_common_cmd = self->info.mipi->ops->mipi_common_cmd;
	  mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

    printk(KERN_DEBUG "otm8019a_qtb4d5386_enter_sleep, is_sleep = %d\n", is_sleep);

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

#define MAX_DATA   48
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

static LCM_Force_Cmd_Code rd_prep_code_1[]=
{
    {0x37, {LCM_SEND(2), {0x1, 0}}},
};
static uint32_t otm8019a_readpowermode(struct panel_spec *self)
{
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

    printk("lcd_otm8019a_mipi read power mode!\n");
    mipi_eotp_set(0,1);

    for(j = 0; j < 4; j++)
    {
        rd_prepare = rd_prep_code_1;
        for(i = 0; i < ARRAY_SIZE(rd_prep_code_1); i++)
        {
            tag = (rd_prepare->real_cmd_code.tag >> 24);
            if(tag & LCM_TAG_SEND)
            {
                mipi_force_write(rd_prepare->datatype, rd_prepare->real_cmd_code.data, (rd_prepare->real_cmd_code.tag & LCM_TAG_MASK));
            }
            else if(tag & LCM_TAG_SLEEP)
            {
                msleep((rd_prepare->real_cmd_code.tag & LCM_TAG_MASK));
            }
            rd_prepare++;
        }

        read_rtn = mipi_force_read(0x0A, 1,(uint8_t *)read_data);
        //printk("lcd_otm8019a mipi read power mode 0x0A value is 0x%x! , read result(%d)\n", read_data[0], read_rtn);

        if((0x9c == read_data[0])  && (0 == read_rtn))
        {
            printk("lcd_otm8019a_mipi read power mode success!\n");
            mipi_eotp_set(1,1);
            return 0x9c;
        }
    }

    mipi_eotp_set(1,1);
    return 0x0;
}


static uint32_t otm8019a_check_esd(struct panel_spec *self)
{
    uint32_t power_mode;

    printk("otm8019a_check_esd!\n");
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
    power_mode = otm8019a_readpowermode(self);
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
        printk("otm8019a_check_esd OK!\n");
        return 1;
    }
    else
    {
        printk("otm8019a_check_esd fail!(0x%x)\n", power_mode);
        return 0;
    }
}

#endif//-modify by liuwei

static u8 lcd_qtb4d5386_otm8019a_driving_mode_cmd_1[]={0xFF,0xFF,0x98,0x06,0x04,0x01};
static u8 lcd_qtb4d5386_otm8019a_driving_mode_cmd_2[]={0x31,0x00};  
static u8 lcd_qtb4d5386_otm8019a_driving_mode_cmd_3[]={0xFF,0xFF,0x98,0x06,0x04,0x00};
static sprdfb_dev_dsi_cmds lcd_driving_mode_control[] =
{
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_qtb4d5386_otm8019a_driving_mode_cmd_1), .p_cmd=lcd_qtb4d5386_otm8019a_driving_mode_cmd_1,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_qtb4d5386_otm8019a_driving_mode_cmd_2), .p_cmd=lcd_qtb4d5386_otm8019a_driving_mode_cmd_2,},
    { .type = DSI_CMD_GEN,	.len = sizeof(lcd_qtb4d5386_otm8019a_driving_mode_cmd_3), .p_cmd=lcd_qtb4d5386_otm8019a_driving_mode_cmd_3,},
};
static void otm8019a_qtb4d5386_driving_mode_set(struct panel_spec *self,bool enable)
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
	
    printk(KERN_INFO "[kernel]:otm8019a_qtb4d5386_driving_mode_set ok.dot enable=%d.\n",enable);
}
static struct panel_operations lcd_otm8019a_qtb4d5386_mipi_operations =
{
    .panel_init = otm8019a_qtb4d5386_mipi_init,
    .panel_readid = otm8019a_qtb4d5386_readid,
    .panel_enter_sleep = otm8019a_qtb4d5386_enter_sleep,
#ifdef CONFIG_FB_ESD_SUPPORT
    .panel_esd_check = otm8019a_check_esd,
#endif
    .lcd_set_driving_mode = otm8019a_qtb4d5386_driving_mode_set,
};

static struct timing_rgb lcd_otm8019a_qtb4d5386_mipi_timing =
{   
    .vfp = 15,  /* unit: line */
    .vbp = 20,
    .vsync = 5,

    .hfp = 55,  /* unit: pixel */
    .hbp = 50,
    .hsync = 5,
};

static struct info_mipi lcd_otm8019a_qtb4d5386_mipi_info =
{
    .work_mode  = SPRDFB_MIPI_MODE_VIDEO,
    .video_bus_width = 24, /*18,16*/
    .lan_number = 2,
    .phy_feq = 390*1000,
    .h_sync_pol = SPRDFB_POLARITY_POS,
    .v_sync_pol = SPRDFB_POLARITY_POS,
    .de_pol = SPRDFB_POLARITY_POS,
    .te_pol = SPRDFB_POLARITY_POS,
    .color_mode_pol = SPRDFB_POLARITY_NEG,
    .shut_down_pol = SPRDFB_POLARITY_NEG,
    .timing = &lcd_otm8019a_qtb4d5386_mipi_timing,
    .ops = NULL,
};

struct panel_spec lcd_otm8019a_qtb4d5386_mipi_spec =
{
    .width  = 480,
    .height = 854,
    .fps    = 60, 
    .type   = LCD_MODE_DSI,
    .direction    = LCD_DIRECT_NORMAL,
    .is_clean_lcd = true,
    .suspend_mode = SEND_SLEEP_CMD ,
    .info = {
        .mipi = &lcd_otm8019a_qtb4d5386_mipi_info
    },
    .ops = &lcd_otm8019a_qtb4d5386_mipi_operations,
    .mipi_clk_level = 0,
};

struct panel_cfg lcd_otm8019a_qtb4d5386_mipi =
{
    /* this panel can only be main lcd */
    .dev_id = SPRDFB_MAINLCD_ID,
    .lcd_id = 0x18,
    .lcd_name = "lcd_otm8019a_qtb4d5386_mipi",
    .lcd_voltage_max = 300,
    .lcd_voltage_min = 200,
    .panel = &lcd_otm8019a_qtb4d5386_mipi_spec,
};

static int __init lcd_otm8019a_qtb4d5386_mipi_init(void)
{
    return sprdfb_panel_register(&lcd_otm8019a_qtb4d5386_mipi);
}

subsys_initcall(lcd_otm8019a_qtb4d5386_mipi_init);

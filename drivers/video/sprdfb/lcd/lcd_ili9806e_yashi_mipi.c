/* drivers/video/sprdfb/lcd/lcd_ili9806e_mipi.c
 *
 * Support for ili9806e mipi LCD device
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

static u8 lcd_ili9806e_yashi_cmd_1[]={0xFF,0xFF,0x98,0x06,0x04,0x01};     // Change to Page 1
static u8 lcd_ili9806e_yashi_cmd_2[]={0x08,0x10};                 // output SDA	
static u8 lcd_ili9806e_yashi_cmd_3[]={0x21,0x01};                 // DE = 1 Active
static u8 lcd_ili9806e_yashi_cmd_4[]={0x30,0x01};                 // 480 X 854	
static u8 lcd_ili9806e_yashi_cmd_5[]={0x31,0x00};                 //  Inversion	
static u8 lcd_ili9806e_yashi_cmd_6[]={0x60,0x07};                 // SDTI	
static u8 lcd_ili9806e_yashi_cmd_7[]={0x61,0x00};                // CRTI	
static u8 lcd_ili9806e_yashi_cmd_8[]={0x62,0x07};                 // EQTI	
static u8 lcd_ili9806e_yashi_cmd_9[]={0x63,0x00};                // PCTI	
static u8 lcd_ili9806e_yashi_cmd_10[]={0x40,0x15};                // BT  +2.5/-2.5 pump for DDVDH-L	
static u8 lcd_ili9806e_yashi_cmd_11[]={0x41,0x44};                 // DVDDH DVDDL clamp  	
static u8 lcd_ili9806e_yashi_cmd_12[]={0x42,0x02};                 // VGH/VGL 	
static u8 lcd_ili9806e_yashi_cmd_13[]={0x43,0x09};                 // VGH/VGL 	
static u8 lcd_ili9806e_yashi_cmd_14[]={0x44,0x0C};                 // VGH/VGL 		
static u8 lcd_ili9806e_yashi_cmd_15[]={0x50,0x88};                 // VGMP	
static u8 lcd_ili9806e_yashi_cmd_16[]={0x51,0x88};                // VGMN	
static u8 lcd_ili9806e_yashi_cmd_17[]={0x52,0x00};                 //Flicker
static u8 lcd_ili9806e_yashi_cmd_18[]={0x53,0x2F};         	 //Flicker4F	
static u8 lcd_ili9806e_yashi_cmd_19[]={0x57,0x50};   	
static u8 lcd_ili9806e_yashi_cmd_20[]={0xA0,0x00};  // Gamma 0 /255
static u8 lcd_ili9806e_yashi_cmd_21[]={0xA1,0x09};  // Gamma 4 /251	
static u8 lcd_ili9806e_yashi_cmd_22[]={0xA2,0x14};  // Gamma 8 /247	
static u8 lcd_ili9806e_yashi_cmd_23[]={0xA3,0x14};  // Gamma 16	/239	
static u8 lcd_ili9806e_yashi_cmd_24[]={0xA4,0x0E};  // Gamma 24 /231	
static u8 lcd_ili9806e_yashi_cmd_25[]={0xA5,0x1C};  // Gamma 52 / 203	
static u8 lcd_ili9806e_yashi_cmd_26[]={0xA6,0x0A};  // Gamma 80 / 175	
static u8 lcd_ili9806e_yashi_cmd_27[]={0xA7,0x0A};  // Gamma 108 /147	
static u8 lcd_ili9806e_yashi_cmd_28[]={0xA8,0x00};  // Gamma 147 /108	
static u8 lcd_ili9806e_yashi_cmd_29[]={0xA9,0x06};  // Gamma 175 / 80
static u8 lcd_ili9806e_yashi_cmd_30[]={0xAA,0x09};  // Gamma 203 / 52
static u8 lcd_ili9806e_yashi_cmd_31[]={0xAB,0x0C};  // Gamma 231 / 24	
static u8 lcd_ili9806e_yashi_cmd_32[]={0xAC,0x17};  // Gamma 239 / 16	
static u8 lcd_ili9806e_yashi_cmd_33[]={0xAD,0x29};  // Gamma 247 / 8	
static u8 lcd_ili9806e_yashi_cmd_34[]={0xAE,0x2D};  // Gamma 251 / 4	
static u8 lcd_ili9806e_yashi_cmd_35[]={0xAF,0x00};  // Gamma 255 / 0
static u8 lcd_ili9806e_yashi_cmd_36[]={0xC0,0x00};  // Gamma 0 /255
static u8 lcd_ili9806e_yashi_cmd_37[]={0xC1,0x03};  // Gamma 4  /251
static u8 lcd_ili9806e_yashi_cmd_38[]={0xC2,0x14};  // Gamma 8  /247	
static u8 lcd_ili9806e_yashi_cmd_39[]={0xC3,0x14};  // Gamma 16  /239	
static u8 lcd_ili9806e_yashi_cmd_40[]={0xC4,0x0C};  // Gamma 24  /231	
static u8 lcd_ili9806e_yashi_cmd_41[]={0xC5,0x1C};  // Gamma 52  /203	
static u8 lcd_ili9806e_yashi_cmd_42[]={0xC6,0x08};  // Gamma 80  /175	
static u8 lcd_ili9806e_yashi_cmd_43[]={0xC7,0x05};  // Gamma 108  /147
static u8 lcd_ili9806e_yashi_cmd_44[]={0xC8,0x05};  // Gamma 147  /108	
static u8 lcd_ili9806e_yashi_cmd_45[]={0xC9,0x0B};  // Gamma 175  /80	
static u8 lcd_ili9806e_yashi_cmd_46[]={0xCA,0x02};  // Gamma 203  /52	
static u8 lcd_ili9806e_yashi_cmd_47[]={0xCB,0x00};  // Gamma 231  /24	
static u8 lcd_ili9806e_yashi_cmd_48[]={0xCC,0x00};  // Gamma 239  /16	
static u8 lcd_ili9806e_yashi_cmd_49[]={0xCD,0x29};  // Gamma 247  /8	
static u8 lcd_ili9806e_yashi_cmd_50[]={0xCE,0x1E};  // Gamma 251  /4
static u8 lcd_ili9806e_yashi_cmd_51[]={0xCF,0x00};  // Gamma 255
static u8 lcd_ili9806e_yashi_cmd_52[]={0xFF,0xFF,0x98,0x06,0x04,0x07};     // Change to Page 7	
static u8 lcd_ili9806e_yashi_cmd_53[]={0x17,0x22};                	              	
static u8 lcd_ili9806e_yashi_cmd_54[]={0x02,0x77};                		
static u8 lcd_ili9806e_yashi_cmd_55[]={0xE1,0x79};                   
static u8 lcd_ili9806e_yashi_cmd_56[]={0xFF,0xFF,0x98,0x06,0x04,0x06};     // Change to Page 6	
static u8 lcd_ili9806e_yashi_cmd_57[]={0x00,0x21};	
static u8 lcd_ili9806e_yashi_cmd_58[]={0x01,0x06};	
static u8 lcd_ili9806e_yashi_cmd_59[]={0x02,0x00};    	
static u8 lcd_ili9806e_yashi_cmd_60[]={0x03,0x00};	
static u8 lcd_ili9806e_yashi_cmd_61[]={0x04,0x16};	
static u8 lcd_ili9806e_yashi_cmd_62[]={0x05,0x16};	
static u8 lcd_ili9806e_yashi_cmd_63[]={0x06,0x80};    	
static u8 lcd_ili9806e_yashi_cmd_64[]={0x07,0x02};	
static u8 lcd_ili9806e_yashi_cmd_65[]={0x08,0x07};	
static u8 lcd_ili9806e_yashi_cmd_66[]={0x09,0x00};    	
static u8 lcd_ili9806e_yashi_cmd_67[]={0x0A,0x00};    	
static u8 lcd_ili9806e_yashi_cmd_68[]={0x0B,0x00};    
static u8 lcd_ili9806e_yashi_cmd_69[]={0x0C,0x16};
static u8 lcd_ili9806e_yashi_cmd_70[]={0x0D,0x16};
static u8 lcd_ili9806e_yashi_cmd_71[]={0x0E,0x00};	
static u8 lcd_ili9806e_yashi_cmd_72[]={0x0F,0x00};	
static u8 lcd_ili9806e_yashi_cmd_73[]={0x10,0x77};	
static u8 lcd_ili9806e_yashi_cmd_74[]={0x11,0xF0};	
static u8 lcd_ili9806e_yashi_cmd_75[]={0x12,0x00};
static u8 lcd_ili9806e_yashi_cmd_76[]={0x13,0x00};	
static u8 lcd_ili9806e_yashi_cmd_77[]={0x14,0x00};	
static u8 lcd_ili9806e_yashi_cmd_78[]={0x15,0xC0};	
static u8 lcd_ili9806e_yashi_cmd_79[]={0x16,0x08};
static u8 lcd_ili9806e_yashi_cmd_80[]={0x17,0x00};	
static u8 lcd_ili9806e_yashi_cmd_81[]={0x18,0x00};	
static u8 lcd_ili9806e_yashi_cmd_82[]={0x19,0x00};	
static u8 lcd_ili9806e_yashi_cmd_83[]={0x1A,0x00};	
static u8 lcd_ili9806e_yashi_cmd_84[]={0x1B,0x00};	
static u8 lcd_ili9806e_yashi_cmd_85[]={0x1C,0x00};	
static u8 lcd_ili9806e_yashi_cmd_86[]={0x1D,0x00};		
static u8 lcd_ili9806e_yashi_cmd_87[]={0x20,0x01};
static u8 lcd_ili9806e_yashi_cmd_88[]={0x21,0x23};	
static u8 lcd_ili9806e_yashi_cmd_89[]={0x22,0x45};	
static u8 lcd_ili9806e_yashi_cmd_90[]={0x23,0x67};	
static u8 lcd_ili9806e_yashi_cmd_91[]={0x24,0x01};	
static u8 lcd_ili9806e_yashi_cmd_92[]={0x25,0x23};	
static u8 lcd_ili9806e_yashi_cmd_93[]={0x26,0x45};	
static u8 lcd_ili9806e_yashi_cmd_94[]={0x27,0x67};		
static u8 lcd_ili9806e_yashi_cmd_95[]={0x30,0x11};	
static u8 lcd_ili9806e_yashi_cmd_96[]={0x31,0x22};	
static u8 lcd_ili9806e_yashi_cmd_97[]={0x32,0x11};	
static u8 lcd_ili9806e_yashi_cmd_98[]={0x33,0x00};	
static u8 lcd_ili9806e_yashi_cmd_99[]={0x34,0x66};	
static u8 lcd_ili9806e_yashi_cmd_100[]={0x35,0x88};
static u8 lcd_ili9806e_yashi_cmd_101[]={0x36,0x22};		
static u8 lcd_ili9806e_yashi_cmd_102[]={0x37,0x22};	
static u8 lcd_ili9806e_yashi_cmd_103[]={0x38,0xAA};	
static u8 lcd_ili9806e_yashi_cmd_104[]={0x39,0xCC};	
static u8 lcd_ili9806e_yashi_cmd_105[]={0x3A,0xBB};	
static u8 lcd_ili9806e_yashi_cmd_106[]={0x3B,0xDD};	
static u8 lcd_ili9806e_yashi_cmd_107[]={0x3C,0x22};	
static u8 lcd_ili9806e_yashi_cmd_108[]={0x3D,0x22};	
static u8 lcd_ili9806e_yashi_cmd_109[]={0x3E,0x22};
static u8 lcd_ili9806e_yashi_cmd_110[]={0x3F,0x22};
static u8 lcd_ili9806e_yashi_cmd_111[]={0x40,0x22};
static u8 lcd_ili9806e_yashi_cmd_112[]={0x52,0x10};
static u8 lcd_ili9806e_yashi_cmd_113[]={0x53,0x10};
static u8 lcd_ili9806e_yashi_cmd_114[]={0xFF,0xFF,0x98,0x06,0x04,0x00};  
static u8 lcd_ili9806e_yashi_cmd_115[]={0x51,0x00};
static u8 lcd_ili9806e_yashi_cmd_116[]={0x53,0x24}; 
static u8 lcd_ili9806e_yashi_cmd_117[]={0x55,0x01};       

static sprdfb_dev_dsi_cmds lcd_ili9806e_yashi_cmds_init[] =
{
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_1), .p_cmd=lcd_ili9806e_yashi_cmd_1,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_2), .p_cmd=lcd_ili9806e_yashi_cmd_2,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_3), .p_cmd=lcd_ili9806e_yashi_cmd_3,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_4), .p_cmd=lcd_ili9806e_yashi_cmd_4,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_5), .p_cmd=lcd_ili9806e_yashi_cmd_5,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_6), .p_cmd=lcd_ili9806e_yashi_cmd_6,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_7), .p_cmd=lcd_ili9806e_yashi_cmd_7,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_8), .p_cmd=lcd_ili9806e_yashi_cmd_8,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_9), .p_cmd=lcd_ili9806e_yashi_cmd_9,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_10), .p_cmd=lcd_ili9806e_yashi_cmd_10,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_11), .p_cmd=lcd_ili9806e_yashi_cmd_11,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_12), .p_cmd=lcd_ili9806e_yashi_cmd_12,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_13), .p_cmd=lcd_ili9806e_yashi_cmd_13,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_14), .p_cmd=lcd_ili9806e_yashi_cmd_14,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_15), .p_cmd=lcd_ili9806e_yashi_cmd_15,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_16), .p_cmd=lcd_ili9806e_yashi_cmd_16,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_17), .p_cmd=lcd_ili9806e_yashi_cmd_17,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_18), .p_cmd=lcd_ili9806e_yashi_cmd_18,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_19), .p_cmd=lcd_ili9806e_yashi_cmd_19,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_20), .p_cmd=lcd_ili9806e_yashi_cmd_20,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_21), .p_cmd=lcd_ili9806e_yashi_cmd_21,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_22), .p_cmd=lcd_ili9806e_yashi_cmd_22,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_23), .p_cmd=lcd_ili9806e_yashi_cmd_23,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_24), .p_cmd=lcd_ili9806e_yashi_cmd_24,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_25), .p_cmd=lcd_ili9806e_yashi_cmd_25,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_26), .p_cmd=lcd_ili9806e_yashi_cmd_26,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_27), .p_cmd=lcd_ili9806e_yashi_cmd_27,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_28), .p_cmd=lcd_ili9806e_yashi_cmd_28,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_29), .p_cmd=lcd_ili9806e_yashi_cmd_29,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_30), .p_cmd=lcd_ili9806e_yashi_cmd_30,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_31), .p_cmd=lcd_ili9806e_yashi_cmd_31,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_32), .p_cmd=lcd_ili9806e_yashi_cmd_32,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_33), .p_cmd=lcd_ili9806e_yashi_cmd_33,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_34), .p_cmd=lcd_ili9806e_yashi_cmd_34,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_35), .p_cmd=lcd_ili9806e_yashi_cmd_35,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_36), .p_cmd=lcd_ili9806e_yashi_cmd_36,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_37), .p_cmd=lcd_ili9806e_yashi_cmd_37,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_38), .p_cmd=lcd_ili9806e_yashi_cmd_38,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_39), .p_cmd=lcd_ili9806e_yashi_cmd_39,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_40), .p_cmd=lcd_ili9806e_yashi_cmd_40,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_41), .p_cmd=lcd_ili9806e_yashi_cmd_41,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_42), .p_cmd=lcd_ili9806e_yashi_cmd_42,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_43), .p_cmd=lcd_ili9806e_yashi_cmd_43,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_44), .p_cmd=lcd_ili9806e_yashi_cmd_44,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_45), .p_cmd=lcd_ili9806e_yashi_cmd_45,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_46), .p_cmd=lcd_ili9806e_yashi_cmd_46,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_47), .p_cmd=lcd_ili9806e_yashi_cmd_47,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_48), .p_cmd=lcd_ili9806e_yashi_cmd_48,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_49), .p_cmd=lcd_ili9806e_yashi_cmd_49,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_50), .p_cmd=lcd_ili9806e_yashi_cmd_50,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_51), .p_cmd=lcd_ili9806e_yashi_cmd_51,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_52), .p_cmd=lcd_ili9806e_yashi_cmd_52,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_53), .p_cmd=lcd_ili9806e_yashi_cmd_53,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_54), .p_cmd=lcd_ili9806e_yashi_cmd_54,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_55), .p_cmd=lcd_ili9806e_yashi_cmd_55,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_56), .p_cmd=lcd_ili9806e_yashi_cmd_56,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_57), .p_cmd=lcd_ili9806e_yashi_cmd_57,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_58), .p_cmd=lcd_ili9806e_yashi_cmd_58,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_59), .p_cmd=lcd_ili9806e_yashi_cmd_59,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_60), .p_cmd=lcd_ili9806e_yashi_cmd_60,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_61), .p_cmd=lcd_ili9806e_yashi_cmd_61,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_62), .p_cmd=lcd_ili9806e_yashi_cmd_62,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_63), .p_cmd=lcd_ili9806e_yashi_cmd_63,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_64), .p_cmd=lcd_ili9806e_yashi_cmd_64,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_65), .p_cmd=lcd_ili9806e_yashi_cmd_65,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_66), .p_cmd=lcd_ili9806e_yashi_cmd_66,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_67), .p_cmd=lcd_ili9806e_yashi_cmd_67,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_68), .p_cmd=lcd_ili9806e_yashi_cmd_68,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_69), .p_cmd=lcd_ili9806e_yashi_cmd_69,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_70), .p_cmd=lcd_ili9806e_yashi_cmd_70,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_71), .p_cmd=lcd_ili9806e_yashi_cmd_71,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_72), .p_cmd=lcd_ili9806e_yashi_cmd_72,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_73), .p_cmd=lcd_ili9806e_yashi_cmd_73,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_74), .p_cmd=lcd_ili9806e_yashi_cmd_74,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_75), .p_cmd=lcd_ili9806e_yashi_cmd_75,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_76), .p_cmd=lcd_ili9806e_yashi_cmd_76,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_77), .p_cmd=lcd_ili9806e_yashi_cmd_77,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_78), .p_cmd=lcd_ili9806e_yashi_cmd_78,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_79), .p_cmd=lcd_ili9806e_yashi_cmd_79,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_80), .p_cmd=lcd_ili9806e_yashi_cmd_80,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_81), .p_cmd=lcd_ili9806e_yashi_cmd_81,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_82), .p_cmd=lcd_ili9806e_yashi_cmd_82,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_83), .p_cmd=lcd_ili9806e_yashi_cmd_83,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_84), .p_cmd=lcd_ili9806e_yashi_cmd_84,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_85), .p_cmd=lcd_ili9806e_yashi_cmd_85,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_86), .p_cmd=lcd_ili9806e_yashi_cmd_86,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_87), .p_cmd=lcd_ili9806e_yashi_cmd_87,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_88), .p_cmd=lcd_ili9806e_yashi_cmd_88,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_89), .p_cmd=lcd_ili9806e_yashi_cmd_89,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_90), .p_cmd=lcd_ili9806e_yashi_cmd_90,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_91), .p_cmd=lcd_ili9806e_yashi_cmd_91,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_92), .p_cmd=lcd_ili9806e_yashi_cmd_92,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_93), .p_cmd=lcd_ili9806e_yashi_cmd_93,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_94), .p_cmd=lcd_ili9806e_yashi_cmd_94,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_95), .p_cmd=lcd_ili9806e_yashi_cmd_95,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_96), .p_cmd=lcd_ili9806e_yashi_cmd_96,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_97), .p_cmd=lcd_ili9806e_yashi_cmd_97,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_98), .p_cmd=lcd_ili9806e_yashi_cmd_98,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_99), .p_cmd=lcd_ili9806e_yashi_cmd_99,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_100), .p_cmd=lcd_ili9806e_yashi_cmd_100,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_101), .p_cmd=lcd_ili9806e_yashi_cmd_101,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_102), .p_cmd=lcd_ili9806e_yashi_cmd_102,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_103), .p_cmd=lcd_ili9806e_yashi_cmd_103,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_104), .p_cmd=lcd_ili9806e_yashi_cmd_104,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_105), .p_cmd=lcd_ili9806e_yashi_cmd_105,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_106), .p_cmd=lcd_ili9806e_yashi_cmd_106,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_107), .p_cmd=lcd_ili9806e_yashi_cmd_107,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_108), .p_cmd=lcd_ili9806e_yashi_cmd_108,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_109), .p_cmd=lcd_ili9806e_yashi_cmd_109,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_110), .p_cmd=lcd_ili9806e_yashi_cmd_110,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_111), .p_cmd=lcd_ili9806e_yashi_cmd_111,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_112), .p_cmd=lcd_ili9806e_yashi_cmd_112,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_113), .p_cmd=lcd_ili9806e_yashi_cmd_113,},
	{ .type = DSI_CMD_GEN,	.len = sizeof(lcd_ili9806e_yashi_cmd_114), .p_cmd=lcd_ili9806e_yashi_cmd_114,},
	{ .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_yashi_cmd_115), .p_cmd=lcd_ili9806e_yashi_cmd_115,},
	{ .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_yashi_cmd_116), .p_cmd=lcd_ili9806e_yashi_cmd_116,},
	{ .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_yashi_cmd_117), .p_cmd=lcd_ili9806e_yashi_cmd_117,},

    
	{ .type = DSI_CMD_DCS,	.len = 1, .cmd={0x11},},
	{ .type = DSI_CMD_DEALY,	.len = 120, },
	{ .type = DSI_CMD_DCS,	.len = 1, .cmd={0x29},},
	{ .type = DSI_CMD_DEALY,	.len = 20, },
	//{ .type = DSI_CMD_DCS,	.len = 1, .cmd={0x2C},},
	{ .type = DSI_CMD_END,},
};

static sprdfb_dev_dsi_cmds lcd_ili9806e_yashi_entry_sleep_cmds_init[] =
{
 
	{ .type = DSI_CMD_DCS,	.len = 1, .cmd={0x28},},
	{ .type = DSI_CMD_DEALY,	.len = 40, },
	{ .type = DSI_CMD_DCS,	.len = 1, .cmd={0x10},},
	{ .type = DSI_CMD_DEALY,	.len = 20, },
	{ .type = DSI_CMD_END,},
};

static int32_t ili9806e_mipi_init(struct panel_spec *self)
{
	sprdfb_dev_dsi_cmds *init = lcd_ili9806e_yashi_cmds_init;
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

static int32_t ili9806e_enter_sleep(struct panel_spec *self, uint8_t is_sleep)
{
	sprdfb_dev_dsi_cmds *init = lcd_ili9806e_yashi_entry_sleep_cmds_init;
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
static uint32_t ili9806e_readid(struct panel_spec *self)
{
	/*Jessica TODO: need read id*/
	return 0x9806;
}

static uint32_t ili9806e_readpowermode(struct panel_spec *self)
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

	pr_debug("lcd_ili9806e_mipi read power mode!\n");
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
		printk("lcd_ili9806e_mipi read power mode 0x0A value is 0x%x! , read result(%d)\n", read_data[0], read_rtn);
		if((0x9c == read_data[0])  && (0 == read_rtn)){
			printk("lcd_ili9806ea_mipi read power mode success!\n");
			mipi_eotp_set(1,1);
			return 0x9c;
		}
	}

	printk("lcd_ili9806e_mipi read power mode fail!0x0A value is 0x%x! , read result(%d)\n", read_data[0], read_rtn);
	mipi_eotp_set(1,1);
	return 0x0;
}

static int32_t ili9806e_check_esd(struct panel_spec *self)
{
	uint32_t power_mode;

	mipi_set_lp_mode_t mipi_set_data_lp_mode = self->info.mipi->ops->mipi_set_data_lp_mode;
	mipi_set_hs_mode_t mipi_set_data_hs_mode = self->info.mipi->ops->mipi_set_data_hs_mode;
	mipi_set_lp_mode_t mipi_set_lp_mode = self->info.mipi->ops->mipi_set_lp_mode;
	mipi_set_hs_mode_t mipi_set_hs_mode = self->info.mipi->ops->mipi_set_hs_mode;
	uint16_t work_mode = self->info.mipi->work_mode;

	pr_debug("ili9806e_check_esd!\n");
	if(SPRDFB_MIPI_MODE_CMD==work_mode){
		mipi_set_lp_mode();
	}else{
		mipi_set_data_lp_mode();
	}
	power_mode = ili9806e_readpowermode(self);
	//power_mode = 0x0;
	if(SPRDFB_MIPI_MODE_CMD==work_mode){
		mipi_set_hs_mode();
	}else{
		mipi_set_data_hs_mode();
	}
	if(power_mode == 0x9c){
		pr_debug("ili9806e_check_esd OK!\n");
		return 1;
	}else{
		printk("ili9806e_check_esd fail!(0x%x)\n", power_mode);
		return 0;
	}
}



static u8 lcd_yashi_ili9806e_driving_mode_cmd_1[]={0xB2,0x82,0xFE,0x09,0x03,0x00,0x50,0x11,0x42,0x1D};
static sprdfb_dev_dsi_cmds lcd_driving_mode_control[] =
{
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_yashi_ili9806e_driving_mode_cmd_1), .p_cmd=lcd_yashi_ili9806e_driving_mode_cmd_1,},
};
static void ili9806e_yashi_driving_mode_set(struct panel_spec *self,bool enable)
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
	
    printk(KERN_INFO "[kernel]:ili9806e_yashi_driving_mode_set ok.dot enable=%d.\n",enable);
}
static struct panel_operations lcd_ili9806e_mipi_operations = {
	.panel_init = ili9806e_mipi_init,
	//.panel_dot_init = ili9806e_mipi_dot_init,
	.panel_readid = ili9806e_readid,
	.panel_esd_check = ili9806e_check_esd,
	.panel_enter_sleep = ili9806e_enter_sleep,
	.lcd_set_driving_mode = ili9806e_yashi_driving_mode_set,
};

static struct timing_rgb lcd_ili9806e_mipi_timing =
{
	.hfp = 80,  /* unit: pixel */
	.hbp = 80,
	.hsync = 10,
	.vfp = 26, /*unit: line*/
	.vbp = 6,
	.vsync = 3,
};

static struct info_mipi lcd_ili9806e_mipi_info = {
	.work_mode  = SPRDFB_MIPI_MODE_VIDEO,
	.video_bus_width = 24, /*18,16*/
	.lan_number = 2,
	.phy_feq = 416*1000,
	.h_sync_pol = SPRDFB_POLARITY_POS,
	.v_sync_pol = SPRDFB_POLARITY_POS,
	.de_pol = SPRDFB_POLARITY_POS,
	.te_pol = SPRDFB_POLARITY_POS,
	.color_mode_pol = SPRDFB_POLARITY_NEG,
	.shut_down_pol = SPRDFB_POLARITY_NEG,
	.timing = &lcd_ili9806e_mipi_timing,
	.ops = NULL,
};

struct panel_spec lcd_ili9806e_yashi_mipi_spec = {
	.width = 480,
	.height = 854,
	.fps	= 60,
	.reset_timing = {5,40,120},
	.type = LCD_MODE_DSI,
	.direction = LCD_DIRECT_NORMAL,
	.is_clean_lcd = true,
	.suspend_mode = SEND_SLEEP_CMD ,
	.info = {
		.mipi = &lcd_ili9806e_mipi_info
	},
	.ops = &lcd_ili9806e_mipi_operations,
};

struct panel_cfg lcd_ili9806e_yashi_mipi = {
	/* this panel can only be main lcd */
	.dev_id = SPRDFB_MAINLCD_ID,
	.lcd_id = 0x9806,
	.lcd_name = "lcd_ili9806e_mipi",
    .lcd_voltage_max = 1325,
    .lcd_voltage_min = 725,
	.panel = &lcd_ili9806e_yashi_mipi_spec,
};

static int __init lcd_ili9806e_mipi_init(void)
{
	return sprdfb_panel_register(&lcd_ili9806e_yashi_mipi);
}

subsys_initcall(lcd_ili9806e_mipi_init);

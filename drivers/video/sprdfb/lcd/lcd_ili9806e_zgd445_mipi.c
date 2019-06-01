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

static u8 lcd_ili9806e_zgd445_cmd_001[]=   {0xFF,0xFF,0x98,0x06,0x04,0x01};     // Change to Page 1	
static u8 lcd_ili9806e_zgd445_cmd_002[]=   {0x08,0x10};
static u8 lcd_ili9806e_zgd445_cmd_003[]=   {0x21,0x01};
static u8 lcd_ili9806e_zgd445_cmd_004[]=   {0x30,0x01};
static u8 lcd_ili9806e_zgd445_cmd_005[]=   {0x31,0x02};
static u8 lcd_ili9806e_zgd445_cmd_006[]=   {0x40,0x16};  //10
static u8 lcd_ili9806e_zgd445_cmd_007[]=   {0x41,0x33};  //36
static u8 lcd_ili9806e_zgd445_cmd_008[]=   {0x42,0x03};  //02
static u8 lcd_ili9806e_zgd445_cmd_009[]=   {0x43,0x89};  //09
static u8 lcd_ili9806e_zgd445_cmd_010[]=   {0x44,0x86};  //06
static u8 lcd_ili9806e_zgd445_cmd_011[]=   {0x45,0xd6};
static u8 lcd_ili9806e_zgd445_cmd_012[]=   {0x52,0x00};
static u8 lcd_ili9806e_zgd445_cmd_013[]=   {0x53,0x6b};
static u8 lcd_ili9806e_zgd445_cmd_014[]=   {0x60,0x07};
static u8 lcd_ili9806e_zgd445_cmd_015[]=   {0x61,0x04};
static u8 lcd_ili9806e_zgd445_cmd_016[]=   {0x62,0x08};
static u8 lcd_ili9806e_zgd445_cmd_017[]=   {0x63,0x04};
static u8 lcd_ili9806e_zgd445_cmd_018[]=   {0xa0,0x00};
static u8 lcd_ili9806e_zgd445_cmd_019[]=   {0xa1,0x01};
static u8 lcd_ili9806e_zgd445_cmd_020[]=   {0xa2,0x0c};
static u8 lcd_ili9806e_zgd445_cmd_021[]=   {0xa3,0x12};
static u8 lcd_ili9806e_zgd445_cmd_022[]=   {0xa4,0x0c};
static u8 lcd_ili9806e_zgd445_cmd_023[]=   {0xa5,0x1d};
static u8 lcd_ili9806e_zgd445_cmd_024[]=   {0xa6,0x09};
static u8 lcd_ili9806e_zgd445_cmd_025[]=   {0xa7,0x07};
static u8 lcd_ili9806e_zgd445_cmd_026[]=   {0xa8,0x05};
static u8 lcd_ili9806e_zgd445_cmd_027[]=   {0xa9,0x0a};
static u8 lcd_ili9806e_zgd445_cmd_028[]=   {0xaa,0x00};
static u8 lcd_ili9806e_zgd445_cmd_029[]=   {0xab,0x01};
static u8 lcd_ili9806e_zgd445_cmd_030[]=   {0xac,0x08};
static u8 lcd_ili9806e_zgd445_cmd_031[]=   {0xad,0x30};
static u8 lcd_ili9806e_zgd445_cmd_032[]=   {0xae,0x2a};
static u8 lcd_ili9806e_zgd445_cmd_033[]=   {0xaf,0x00};
static u8 lcd_ili9806e_zgd445_cmd_034[]=   {0xc0,0x00};
static u8 lcd_ili9806e_zgd445_cmd_035[]=   {0xc1,0x05};
static u8 lcd_ili9806e_zgd445_cmd_036[]=   {0xc2,0x0b};
static u8 lcd_ili9806e_zgd445_cmd_037[]=   {0xc3,0x10};
static u8 lcd_ili9806e_zgd445_cmd_038[]=   {0xc4,0x0c};
static u8 lcd_ili9806e_zgd445_cmd_039[]=   {0xc5,0x1f};
static u8 lcd_ili9806e_zgd445_cmd_040[]=   {0xc6,0x0c};
static u8 lcd_ili9806e_zgd445_cmd_041[]=   {0xc7,0x0a};
static u8 lcd_ili9806e_zgd445_cmd_042[]=   {0xc8,0x03};
static u8 lcd_ili9806e_zgd445_cmd_043[]=   {0xc9,0x07};
static u8 lcd_ili9806e_zgd445_cmd_044[]=   {0xca,0x05};
static u8 lcd_ili9806e_zgd445_cmd_045[]=   {0xcb,0x02};
static u8 lcd_ili9806e_zgd445_cmd_046[]=   {0xcc,0x07};
static u8 lcd_ili9806e_zgd445_cmd_047[]=   {0xcd,0x2b};
static u8 lcd_ili9806e_zgd445_cmd_048[]=   {0xce,0x26};
static u8 lcd_ili9806e_zgd445_cmd_049[]=   {0xcf,0x00};
static u8 lcd_ili9806e_zgd445_cmd_050[]=   {0xFF,0xFF,0x98,0x06,0x04,0x06};  // Change to Page 6
static u8 lcd_ili9806e_zgd445_cmd_051[]=   {0x00,0x3c};
static u8 lcd_ili9806e_zgd445_cmd_052[]=   {0x01,0x06};
static u8 lcd_ili9806e_zgd445_cmd_053[]=   {0x02,0x00};
static u8 lcd_ili9806e_zgd445_cmd_054[]=   {0x03,0x00};
static u8 lcd_ili9806e_zgd445_cmd_055[]=   {0x04,0x1c};
static u8 lcd_ili9806e_zgd445_cmd_056[]=   {0x05,0x1c};
static u8 lcd_ili9806e_zgd445_cmd_057[]=   {0x06,0x98};
static u8 lcd_ili9806e_zgd445_cmd_058[]=   {0x07,0x04};
static u8 lcd_ili9806e_zgd445_cmd_059[]=   {0x08,0x08};
static u8 lcd_ili9806e_zgd445_cmd_060[]=   {0x09,0x00};
static u8 lcd_ili9806e_zgd445_cmd_061[]=   {0x0a,0x00};
static u8 lcd_ili9806e_zgd445_cmd_062[]=   {0x0b,0x00};
static u8 lcd_ili9806e_zgd445_cmd_063[]=   {0x0c,0x1c};
static u8 lcd_ili9806e_zgd445_cmd_064[]=   {0x0d,0x1c};
static u8 lcd_ili9806e_zgd445_cmd_065[]=   {0x0e,0x00};
static u8 lcd_ili9806e_zgd445_cmd_066[]=   {0x0f,0x00};
static u8 lcd_ili9806e_zgd445_cmd_067[]=   {0x10,0xd5};
static u8 lcd_ili9806e_zgd445_cmd_068[]=   {0x11,0xd0};
static u8 lcd_ili9806e_zgd445_cmd_069[]=   {0x12,0x00};
static u8 lcd_ili9806e_zgd445_cmd_070[]=   {0x13,0x00};
static u8 lcd_ili9806e_zgd445_cmd_071[]=   {0x14,0x00};
static u8 lcd_ili9806e_zgd445_cmd_072[]=   {0x15,0x43};
static u8 lcd_ili9806e_zgd445_cmd_073[]=   {0x16,0x0b};
static u8 lcd_ili9806e_zgd445_cmd_074[]=   {0x17,0x00};
static u8 lcd_ili9806e_zgd445_cmd_075[]=   {0x18,0x00};
static u8 lcd_ili9806e_zgd445_cmd_076[]=   {0x19,0x00};
static u8 lcd_ili9806e_zgd445_cmd_077[]=   {0x1a,0x00};
static u8 lcd_ili9806e_zgd445_cmd_078[]=   {0x1b,0x00};
static u8 lcd_ili9806e_zgd445_cmd_079[]=   {0x1c,0x48};
static u8 lcd_ili9806e_zgd445_cmd_080[]=   {0x1d,0x00};
static u8 lcd_ili9806e_zgd445_cmd_081[]=   {0x20,0x01};
static u8 lcd_ili9806e_zgd445_cmd_082[]=   {0x21,0x23};
static u8 lcd_ili9806e_zgd445_cmd_083[]=   {0x22,0x45};
static u8 lcd_ili9806e_zgd445_cmd_084[]=   {0x23,0x67};
static u8 lcd_ili9806e_zgd445_cmd_085[]=   {0x24,0x01};
static u8 lcd_ili9806e_zgd445_cmd_086[]=   {0x25,0x23};
static u8 lcd_ili9806e_zgd445_cmd_087[]=   {0x26,0x45};
static u8 lcd_ili9806e_zgd445_cmd_088[]=   {0x27,0x67};
static u8 lcd_ili9806e_zgd445_cmd_089[]=   {0x30,0x13};
static u8 lcd_ili9806e_zgd445_cmd_090[]=   {0x31,0x22};
static u8 lcd_ili9806e_zgd445_cmd_091[]=   {0x32,0xdd};
static u8 lcd_ili9806e_zgd445_cmd_092[]=   {0x33,0xcc};
static u8 lcd_ili9806e_zgd445_cmd_093[]=   {0x34,0xbb};
static u8 lcd_ili9806e_zgd445_cmd_094[]=   {0x35,0xaa};
static u8 lcd_ili9806e_zgd445_cmd_095[]=   {0x36,0x22};
static u8 lcd_ili9806e_zgd445_cmd_096[]=   {0x37,0x26};
static u8 lcd_ili9806e_zgd445_cmd_097[]=   {0x38,0x72};
static u8 lcd_ili9806e_zgd445_cmd_098[]=   {0x39,0xff};
static u8 lcd_ili9806e_zgd445_cmd_099[]=   {0x3a,0x22};
static u8 lcd_ili9806e_zgd445_cmd_100[]=   {0x3b,0xee};
static u8 lcd_ili9806e_zgd445_cmd_101[]=   {0x3c,0x22};
static u8 lcd_ili9806e_zgd445_cmd_102[]=   {0x3d,0x22};
static u8 lcd_ili9806e_zgd445_cmd_103[]=   {0x3e,0x22};
static u8 lcd_ili9806e_zgd445_cmd_104[]=   {0x3f,0x22};
static u8 lcd_ili9806e_zgd445_cmd_105[]=   {0x40,0x22};
static u8 lcd_ili9806e_zgd445_cmd_106[]=   {0x52,0x10};
static u8 lcd_ili9806e_zgd445_cmd_107[]=   {0x53,0x10};
static u8 lcd_ili9806e_zgd445_cmd_108[]=   {0xFF,0xFF,0x98,0x06,0x04,0x07};     // Change to Page 7
static u8 lcd_ili9806e_zgd445_cmd_109[]=   {0x18,0x1d};
static u8 lcd_ili9806e_zgd445_cmd_110[]=   {0x17,0x22};
static u8 lcd_ili9806e_zgd445_cmd_111[]=   {0x06,0x03};
static u8 lcd_ili9806e_zgd445_cmd_112[]=   {0x02,0x77};
static u8 lcd_ili9806e_zgd445_cmd_113[]=   {0xe1,0x79};
static u8 lcd_ili9806e_zgd445_cmd_114[]=   {0xFF,0xFF,0x98,0x06,0x04,0x00};      // Change to Page 0
static u8 lcd_ili9806e_zgd445_cmd_115[]=   {0x11};//Sleep-Out
static u8 lcd_ili9806e_zgd445_cmd_116[]=   {0x29};//DisplayOn

static sprdfb_dev_dsi_cmds lcd_ili9806e_zgd445_cmds_init[] =
{
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_001), .p_cmd=lcd_ili9806e_zgd445_cmd_001,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_002), .p_cmd=lcd_ili9806e_zgd445_cmd_002,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_003), .p_cmd=lcd_ili9806e_zgd445_cmd_003,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_004), .p_cmd=lcd_ili9806e_zgd445_cmd_004,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_005), .p_cmd=lcd_ili9806e_zgd445_cmd_005,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_006), .p_cmd=lcd_ili9806e_zgd445_cmd_006,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_007), .p_cmd=lcd_ili9806e_zgd445_cmd_007,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_008), .p_cmd=lcd_ili9806e_zgd445_cmd_008,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_009), .p_cmd=lcd_ili9806e_zgd445_cmd_009,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_010), .p_cmd=lcd_ili9806e_zgd445_cmd_010,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_011), .p_cmd=lcd_ili9806e_zgd445_cmd_011,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_012), .p_cmd=lcd_ili9806e_zgd445_cmd_012,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_013), .p_cmd=lcd_ili9806e_zgd445_cmd_013,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_014), .p_cmd=lcd_ili9806e_zgd445_cmd_014,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_015), .p_cmd=lcd_ili9806e_zgd445_cmd_015,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_016), .p_cmd=lcd_ili9806e_zgd445_cmd_016,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_017), .p_cmd=lcd_ili9806e_zgd445_cmd_017,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_018), .p_cmd=lcd_ili9806e_zgd445_cmd_018,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_019), .p_cmd=lcd_ili9806e_zgd445_cmd_019,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_020), .p_cmd=lcd_ili9806e_zgd445_cmd_020,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_021), .p_cmd=lcd_ili9806e_zgd445_cmd_021,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_022), .p_cmd=lcd_ili9806e_zgd445_cmd_022,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_023), .p_cmd=lcd_ili9806e_zgd445_cmd_023,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_024), .p_cmd=lcd_ili9806e_zgd445_cmd_024,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_025), .p_cmd=lcd_ili9806e_zgd445_cmd_025,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_026), .p_cmd=lcd_ili9806e_zgd445_cmd_026,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_027), .p_cmd=lcd_ili9806e_zgd445_cmd_027,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_028), .p_cmd=lcd_ili9806e_zgd445_cmd_028,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_029), .p_cmd=lcd_ili9806e_zgd445_cmd_029,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_030), .p_cmd=lcd_ili9806e_zgd445_cmd_030,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_031), .p_cmd=lcd_ili9806e_zgd445_cmd_031,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_032), .p_cmd=lcd_ili9806e_zgd445_cmd_032,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_033), .p_cmd=lcd_ili9806e_zgd445_cmd_033,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_034), .p_cmd=lcd_ili9806e_zgd445_cmd_034,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_035), .p_cmd=lcd_ili9806e_zgd445_cmd_035,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_036), .p_cmd=lcd_ili9806e_zgd445_cmd_036,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_037), .p_cmd=lcd_ili9806e_zgd445_cmd_037,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_038), .p_cmd=lcd_ili9806e_zgd445_cmd_038,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_039), .p_cmd=lcd_ili9806e_zgd445_cmd_039,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_040), .p_cmd=lcd_ili9806e_zgd445_cmd_040,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_041), .p_cmd=lcd_ili9806e_zgd445_cmd_041,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_042), .p_cmd=lcd_ili9806e_zgd445_cmd_042,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_043), .p_cmd=lcd_ili9806e_zgd445_cmd_043,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_044), .p_cmd=lcd_ili9806e_zgd445_cmd_044,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_045), .p_cmd=lcd_ili9806e_zgd445_cmd_045,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_046), .p_cmd=lcd_ili9806e_zgd445_cmd_046,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_047), .p_cmd=lcd_ili9806e_zgd445_cmd_047,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_048), .p_cmd=lcd_ili9806e_zgd445_cmd_048,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_049), .p_cmd=lcd_ili9806e_zgd445_cmd_049,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_050), .p_cmd=lcd_ili9806e_zgd445_cmd_050,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_051), .p_cmd=lcd_ili9806e_zgd445_cmd_051,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_052), .p_cmd=lcd_ili9806e_zgd445_cmd_052,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_053), .p_cmd=lcd_ili9806e_zgd445_cmd_053,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_054), .p_cmd=lcd_ili9806e_zgd445_cmd_054,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_055), .p_cmd=lcd_ili9806e_zgd445_cmd_055,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_056), .p_cmd=lcd_ili9806e_zgd445_cmd_056,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_057), .p_cmd=lcd_ili9806e_zgd445_cmd_057,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_058), .p_cmd=lcd_ili9806e_zgd445_cmd_058,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_059), .p_cmd=lcd_ili9806e_zgd445_cmd_059,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_060), .p_cmd=lcd_ili9806e_zgd445_cmd_060,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_061), .p_cmd=lcd_ili9806e_zgd445_cmd_061,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_062), .p_cmd=lcd_ili9806e_zgd445_cmd_062,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_063), .p_cmd=lcd_ili9806e_zgd445_cmd_063,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_064), .p_cmd=lcd_ili9806e_zgd445_cmd_064,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_065), .p_cmd=lcd_ili9806e_zgd445_cmd_065,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_066), .p_cmd=lcd_ili9806e_zgd445_cmd_066,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_067), .p_cmd=lcd_ili9806e_zgd445_cmd_067,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_068), .p_cmd=lcd_ili9806e_zgd445_cmd_068,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_069), .p_cmd=lcd_ili9806e_zgd445_cmd_069,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_070), .p_cmd=lcd_ili9806e_zgd445_cmd_070,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_071), .p_cmd=lcd_ili9806e_zgd445_cmd_071,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_072), .p_cmd=lcd_ili9806e_zgd445_cmd_072,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_073), .p_cmd=lcd_ili9806e_zgd445_cmd_073,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_074), .p_cmd=lcd_ili9806e_zgd445_cmd_074,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_075), .p_cmd=lcd_ili9806e_zgd445_cmd_075,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_076), .p_cmd=lcd_ili9806e_zgd445_cmd_076,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_077), .p_cmd=lcd_ili9806e_zgd445_cmd_077,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_078), .p_cmd=lcd_ili9806e_zgd445_cmd_078,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_079), .p_cmd=lcd_ili9806e_zgd445_cmd_079,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_080), .p_cmd=lcd_ili9806e_zgd445_cmd_080,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_081), .p_cmd=lcd_ili9806e_zgd445_cmd_081,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_082), .p_cmd=lcd_ili9806e_zgd445_cmd_082,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_083), .p_cmd=lcd_ili9806e_zgd445_cmd_083,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_084), .p_cmd=lcd_ili9806e_zgd445_cmd_084,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_085), .p_cmd=lcd_ili9806e_zgd445_cmd_085,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_086), .p_cmd=lcd_ili9806e_zgd445_cmd_086,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_087), .p_cmd=lcd_ili9806e_zgd445_cmd_087,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_088), .p_cmd=lcd_ili9806e_zgd445_cmd_088,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_089), .p_cmd=lcd_ili9806e_zgd445_cmd_089,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_090), .p_cmd=lcd_ili9806e_zgd445_cmd_090,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_091), .p_cmd=lcd_ili9806e_zgd445_cmd_091,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_092), .p_cmd=lcd_ili9806e_zgd445_cmd_092,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_093), .p_cmd=lcd_ili9806e_zgd445_cmd_093,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_094), .p_cmd=lcd_ili9806e_zgd445_cmd_094,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_095), .p_cmd=lcd_ili9806e_zgd445_cmd_095,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_096), .p_cmd=lcd_ili9806e_zgd445_cmd_096,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_097), .p_cmd=lcd_ili9806e_zgd445_cmd_097,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_098), .p_cmd=lcd_ili9806e_zgd445_cmd_098,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_099), .p_cmd=lcd_ili9806e_zgd445_cmd_099,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_100), .p_cmd=lcd_ili9806e_zgd445_cmd_100,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_101), .p_cmd=lcd_ili9806e_zgd445_cmd_101,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_102), .p_cmd=lcd_ili9806e_zgd445_cmd_102,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_103), .p_cmd=lcd_ili9806e_zgd445_cmd_103,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_104), .p_cmd=lcd_ili9806e_zgd445_cmd_104,},
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_105), .p_cmd=lcd_ili9806e_zgd445_cmd_105,},      
	
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_106), .p_cmd=lcd_ili9806e_zgd445_cmd_106,},      
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_107), .p_cmd=lcd_ili9806e_zgd445_cmd_107,},      
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_108), .p_cmd=lcd_ili9806e_zgd445_cmd_108,},      
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_109), .p_cmd=lcd_ili9806e_zgd445_cmd_109,},      
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_110), .p_cmd=lcd_ili9806e_zgd445_cmd_110,},      
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_111), .p_cmd=lcd_ili9806e_zgd445_cmd_111,},      
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_112), .p_cmd=lcd_ili9806e_zgd445_cmd_112,},      
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_113), .p_cmd=lcd_ili9806e_zgd445_cmd_113,},      
	{ .type = DSI_CMD_GEN,  .len = sizeof(lcd_ili9806e_zgd445_cmd_114), .p_cmd=lcd_ili9806e_zgd445_cmd_114,},      
	
	{ .type = DSI_CMD_DCS,  .len = sizeof(lcd_ili9806e_zgd445_cmd_115), .p_cmd=lcd_ili9806e_zgd445_cmd_115,},
	{ .type = DSI_CMD_DEALY, .len = 120, },                                                             
	{ .type = DSI_CMD_DCS,  .len = sizeof(lcd_ili9806e_zgd445_cmd_116), .p_cmd=lcd_ili9806e_zgd445_cmd_116,},
	{ .type = DSI_CMD_DEALY,    .len = 20, },                                                              
	{ .type = DSI_CMD_END,},                                       
};


static sprdfb_dev_dsi_cmds lcd_ili9806e_zgd445_entry_sleep_cmds_init[] =
{
 
	{ .type = DSI_CMD_DCS,	.len = 1, .cmd={0x28},},
	{ .type = DSI_CMD_DEALY,	.len = 40, },
	{ .type = DSI_CMD_DCS,	.len = 1, .cmd={0x10},},
	{ .type = DSI_CMD_DEALY,	.len = 20, },
	{ .type = DSI_CMD_END,},
};

extern unsigned int lcd_debug_panel_on_cmds( sprdfb_dev_dsi_cmds**cmds, unsigned char** buf) ;

static int32_t ili9806e_mipi_init(struct panel_spec *self)
{
    sprdfb_dev_dsi_cmds *dbg_cmds = NULL;
    unsigned char* dbg_data= NULL ;
    unsigned int size = 0;

	sprdfb_dev_dsi_cmds *init = lcd_ili9806e_zgd445_cmds_init;
	mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
	mipi_common_cmd_t mipi_common_cmd = self->info.mipi->ops->mipi_common_cmd;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

    printk(KERN_DEBUG "[kernel]:ili9806e_zgd445_init\n");

	mipi_set_cmd_mode();
	mipi_eotp_set(1,0);
	
    if( ( size = lcd_debug_panel_on_cmds(&dbg_cmds, &dbg_data) ) > 0 )
    {
        mipi_common_cmd(dbg_cmds);
        kfree(dbg_cmds);
        kfree(dbg_data);
        printk(KERN_INFO "[kernel]:>>>>>>>load sdcard config param success size = %d<<<<<<<<< \n",size);
    }
    else
    {
        mipi_common_cmd(init);
        printk("[kernel]:>>>>>>>>load local config param success<<<<<<<<<<<<< \n");
    }
    
    mipi_eotp_set(1,1);

    return 0;
}

static int32_t ili9806e_enter_sleep(struct panel_spec *self, uint8_t is_sleep)
{
	sprdfb_dev_dsi_cmds *init = lcd_ili9806e_zgd445_entry_sleep_cmds_init;
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



static u8 lcd_zgd367_ili9806e_driving_mode_cmd_1[]={0xB2,0x82,0xFE,0x09,0x03,0x00,0x50,0x11,0x42,0x1D};
static sprdfb_dev_dsi_cmds lcd_driving_mode_control[] =
{
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_zgd367_ili9806e_driving_mode_cmd_1), .p_cmd=lcd_zgd367_ili9806e_driving_mode_cmd_1,},
};
static void ili9806e_zgd445_driving_mode_set(struct panel_spec *self,bool enable)
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
	
    printk(KERN_INFO "[kernel]:ili9806e_zgd445_driving_mode_set ok.dot enable=%d.\n",enable);
}
static struct panel_operations lcd_ili9806e_mipi_operations = {
	.panel_init = ili9806e_mipi_init,
	//.panel_dot_init = ili9806e_mipi_dot_init,
	.panel_readid = ili9806e_readid,
	.panel_esd_check = ili9806e_check_esd,
	.panel_enter_sleep = ili9806e_enter_sleep,
	.lcd_set_driving_mode = ili9806e_zgd445_driving_mode_set,
};

static struct timing_rgb lcd_ili9806e_mipi_timing =
{
	.hfp = 80,  /* unit: pixel */
	.hbp = 77,
	.hsync = 8,
	.vfp = 14, /*unit: line*/
	.vbp = 14,
	.vsync = 4,
};

static struct info_mipi lcd_ili9806e_mipi_info = {
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
	.timing = &lcd_ili9806e_mipi_timing,
	.ops = NULL,
};

struct panel_spec lcd_ili9806e_zgd445_mipi_spec = {
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

struct panel_cfg lcd_ili9806e_zgd445_mipi = {
	/* this panel can only be main lcd */
    .dev_id = SPRDFB_MAINLCD_ID,
	.lcd_id = 0x9806,
	.lcd_name = "lcd_ili9806e_zgd445_mipi",
    .lcd_voltage_max = 100,
    .lcd_voltage_min = 0,
	.panel = &lcd_ili9806e_zgd445_mipi_spec,
};

static int __init lcd_ili9806e_mipi_init(void)
{
	return sprdfb_panel_register(&lcd_ili9806e_zgd445_mipi);
}

subsys_initcall(lcd_ili9806e_mipi_init);

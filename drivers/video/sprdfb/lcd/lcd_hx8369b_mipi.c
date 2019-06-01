/* drivers/video/sprdfb/lcd_hx8369b_mipi.c
 *
 * Support for hx8369b mipi LCD device
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
#include <linux/bug.h>
#include <linux/delay.h>
#include "../sprdfb_panel.h"
#include <mach/gpio.h>

#define  LCD_DEBUG
#ifdef LCD_DEBUG
#define LCD_PRINT printk
#else
#define LCD_PRINT(...)
#endif

#define MAX_DATA   150

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

static LCM_Init_Code init_data[] = {
	{LCM_SEND(6),{4,0,0xB9,0xFF,0x83,0x69}},
	{LCM_SLEEP(10)},
	{LCM_SEND(18),{16,0,0xBA,0x31,0x00,0x16,0xCA,0xB1,0x0A,0x00,0x10,0x28,0x02,0x21,0x21,0x9A,0x1A,0x8F}},

	//set GIP timing control
	{LCM_SEND(95),{93,0,0xD5,0x00,0x00,0x07,0x00,0x09,0x00,0x00,0x10,0x01,0x00,0x00,0x00,0x01,0x42,0x37,\
						0x00,0x00,0x08,0x2A,0x08,0x47,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
						0x15,0x00,0x00,0x98,0x98,0x44,0x66,0x00,0x22,0x51,0x00,0x00,0x89,0x98,0x33,0x11,0x77,0x55,0x04,0x00,\
						0x00,0x98,0x98,0x55,0x77,0x11,0x33,0x40,0x00,0x00,0x89,0x98,0x22,0x00,0x66,0x44,0x15,0x00,0x00,0x00,\
						0x01,0x00,0x00,0x00,0x03,0x00,0x0F,0xFF,0xFF,0x03,0x00,0x0F,0xFF,0xFF,0x00,0x8C,0x5A}},
	{LCM_SEND(2),{0x3A,0x70}},
	{LCM_SEND(2),{0x36,0x00}},

	{LCM_SEND(13),{11,0,0xB1,0x0B,0x83,0x77,0x00,0x0F,0x0F,0x16,0x16,0x0C,0x0A}},
	{LCM_SEND(2),{0xB2,0x03}},
//	{LCM_SEND(5),{3,0,0xB2,0x00,0x70}},
	{LCM_SEND(7),{5,0,0xB3,0x83,0x00,0x31,0x03}},
	{LCM_SEND(2),{0xB4,0x00}},
	{LCM_SEND(5),{3,0,0xB6,0xA3,0xA3}},
	{LCM_SEND(2),{0xCC,0x0C}},
	{LCM_SEND(9),{7,0,0xC0,0x73,0x50,0x00,0x34,0xc4,0x00}},
	{LCM_SEND(7),{5,0,0xE3,0x00,0x00,0x00,0x00}},
	{LCM_SEND(2),{0xEA,0x72}},
	{LCM_SEND(6),{4,0,0xC6,0x41,0xFF,0x7D}},


	//gamma setting
	{LCM_SEND(38),{36,0,0xE0,0x01,0x0D,0x18,0x14,0x13,0x2F,0x31,0x3B,0x09,0x10,0x12,0x16,0x17,0x16,0x16,0x11,0x14,0x01,\
						0x0D,0x18,0x14,0x13,0x2F,0x30,0x3B,0x09,0x10,0x11,0x16,0x17,0x15,0x16,0x11,0x14,0x01}},
	{LCM_SEND(130),{128,0,0xC1,0x03,0x00,0x06,0x0D,0x15,0x1D,0x25,0x2D,0x35,0x3D,\
				   0x45,0x4D,0x55,0x5D,0x65,0x6D,0x75,0x7C,0x84,0x8C,\
				   0x94,0x9C,0xA4,0xAC,0xB4,0xBC,0xC4,0xCC,0xD4,0xDC,\
				   0xE4,0xEC,0xF3,0xFD,0x0F,0xFA,0x95,0x40,0xFF,0xAA,\
				   0x54,0x03,0xC0,0x00,0x07,0x0F,0x17,0x1F,0x27,0x2F,\
				   0x37,0x3F,0x47,0x4F,0x57,0x5F,0x67,0x6F,0x77,0x7F,\
				   0x87,0x8F,0x97,0x9F,0xA7,0xAF,0xB7,0xBF,0xC7,0xCF,\
				   0xD7,0xDF,0xE7,0xEF,0xF7,0xFF,0x00,0x00,0x00,0x00,\
				   0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x0F,0x17,0x1F,\
				   0x27,0x2F,0x37,0x3F,0x47,0x4F,0x57,0x5F,0x67,0x6F,\
				   0x77,0x7F,0x87,0x8F,0x97,0x9F,0xA7,0xAF,0xB7,0xBF,\
				   0xC7,0xCF,0xD7,0xDF,0xE7,0xEF,0xF7,0xFF,0x00,0x00,\
				   0x00,0x00,0x00,0x00,0x00,0x00,0x00,}},

	{LCM_SEND(5),{3,0,0xC9,0x0F,0x00}},
	{LCM_SLEEP(5)},
	{LCM_SEND(2),{0x51,0x00}},
	{LCM_SLEEP(5)},
//	{LCM_SEND(2),{0x53,0x24}},
	{LCM_SEND(2),{0x53,0x24}},
	{LCM_SLEEP(5)},
	{LCM_SEND(1),{0x11}},
	{LCM_SLEEP(180)},
	{LCM_SEND(1),{0x29}},
	{LCM_SLEEP(10)}, //40

};

static LCM_Init_Code disp_on =  {LCM_SEND(1), {0x29}};

static LCM_Init_Code sleep_in[] =  {
	{LCM_SEND(1), {0x28}},
	{LCM_SLEEP(150)}, 	//>150ms
	{LCM_SEND(1), {0x10}},
	{LCM_SLEEP(150)},	//>150ms
};

static LCM_Init_Code sleep_out[] =  {
	{LCM_SEND(1), {0x11}},
	{LCM_SLEEP(120)},//>120ms
	{LCM_SEND(1), {0x29}},
	{LCM_SLEEP(20)}, //>20ms
};
static int32_t hx8369b_mipi_init(struct panel_spec *self)
{
	int32_t i = 0;
	LCM_Init_Code *init = init_data;
	unsigned int tag;

	mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
	mipi_dcs_write_t mipi_dcs_write = self->info.mipi->ops->mipi_dcs_write;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

	printk("kernel hx8369b_mipi_init\n");

	mipi_set_cmd_mode();
	mipi_eotp_set(0,0);

	for(i = 0; i < ARRAY_SIZE(init_data); i++){
		tag = (init->tag >>24);
		if(tag & LCM_TAG_SEND){
			mipi_dcs_write(init->data, (init->tag & LCM_TAG_MASK));
			udelay(20);
		}else if(tag & LCM_TAG_SLEEP){
			msleep(init->tag & LCM_TAG_MASK);
		}
		init++;
	}
	mipi_eotp_set(0,0);

	return 0;
}

static uint32_t hx8369b_readid(struct panel_spec *self)
	{
		int32_t j =0;
		uint8_t read_data[4] = {0};
		int32_t read_rtn = 0;
		uint8_t param[2] = {0};
		mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
		mipi_force_write_t mipi_force_write = self->info.mipi->ops->mipi_force_write;
		mipi_force_read_t mipi_force_read = self->info.mipi->ops->mipi_force_read;
		mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

		LCD_PRINT("kernel lcd_hx8369b_mipi read id!\n");

		mipi_set_cmd_mode();
		mipi_eotp_set(0,0);

		for(j = 0; j < 4; j++){
			param[0] = 0x01;
			param[1] = 0x00;
			mipi_force_write(0x37, param, 2);
			read_rtn = mipi_force_read(0xda,1,&read_data[0]);
			LCD_PRINT("lcd_hx8369b_mipi read id 0xda value is 0x%x!\n",read_data[0]);

			read_rtn = mipi_force_read(0xdb,1,&read_data[1]);
			LCD_PRINT("lcd_hx8369b_mipi read id 0xdb value is 0x%x!\n",read_data[1]);

			read_rtn = mipi_force_read(0xdc,1,&read_data[2]);
			LCD_PRINT("lcd_hx8369b_mipi read id 0xdc value is 0x%x!\n",read_data[2]);

			if((0x55 == read_data[0])&&(0x0c == read_data[1])&&(0x90 == read_data[2])){
					printk("lcd_hx8369b_mipi read id success!\n");
					return 0x8369;
				}
			}

		mipi_eotp_set(0,0);

		return 0;
}


static int32_t hx8369b_enter_sleep(struct panel_spec *self, uint8_t is_sleep)
{
	int32_t i = 0;
	LCM_Init_Code *sleep_in_out = NULL;
	unsigned int tag;
	int32_t size = 0;

	mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
	mipi_dcs_write_t mipi_dcs_write = self->info.mipi->ops->mipi_dcs_write;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

	printk("kernel hx8369b_enter_sleep, is_sleep = %d\n", is_sleep);

	if(is_sleep){
		sleep_in_out = sleep_in;
		size = ARRAY_SIZE(sleep_in);
	}else{
		sleep_in_out = sleep_out;
		size = ARRAY_SIZE(sleep_out);
	}

	mipi_set_cmd_mode();
	mipi_eotp_set(0,0);

	for(i = 0; i <size ; i++){
		tag = (sleep_in_out->tag >>24);
		if(tag & LCM_TAG_SEND){
			mipi_dcs_write(sleep_in_out->data, (sleep_in_out->tag & LCM_TAG_MASK));
		}else if(tag & LCM_TAG_SLEEP){
			msleep(sleep_in_out->tag & LCM_TAG_MASK);
		}
		sleep_in_out++;
	}
	mipi_eotp_set(0,0);

	return 0;
}

static struct panel_operations lcd_hx8369b_mipi_operations = {
	.panel_init = hx8369b_mipi_init,
	.panel_readid = hx8369b_readid,
	.panel_enter_sleep = hx8369b_enter_sleep,
};

static struct timing_rgb lcd_hx8369b_mipi_timing = {
	.hfp = 110,  /* unit: pixel */
	.hbp = 110,
	.hsync = 4,
	.vfp = 10, /*unit: line*/
	.vbp = 12,
	.vsync = 4,
};


static struct info_mipi lcd_hx8369b_mipi_info = {
	.work_mode  = SPRDFB_MIPI_MODE_VIDEO,
	.video_bus_width = 24, /*18,16*/
	.lan_number = 	2,
	.phy_feq =500*1000,
	.h_sync_pol = SPRDFB_POLARITY_POS,
	.v_sync_pol = SPRDFB_POLARITY_POS,
	.de_pol = SPRDFB_POLARITY_POS,
	.te_pol = SPRDFB_POLARITY_POS,
	.color_mode_pol = SPRDFB_POLARITY_NEG,
	.shut_down_pol = SPRDFB_POLARITY_NEG,
	.timing = &lcd_hx8369b_mipi_timing,
	.ops = NULL,
};

struct panel_spec lcd_hx8369b_mipi_spec = {
	.width = 480,
	.height = 800,
	.fps	= 60,
	.type = LCD_MODE_DSI,
	.direction = LCD_DIRECT_NORMAL,
	.info = {
		.mipi = &lcd_hx8369b_mipi_info
	},
	.ops = &lcd_hx8369b_mipi_operations,
};
struct panel_cfg lcd_hx8369b_mipi = {
	/* this panel can only be main lcd */
	.dev_id = SPRDFB_MAINLCD_ID,
	.lcd_id = 0x8369,
	.lcd_name = "lcd_hx8369b_mipi",
	.panel = &lcd_hx8369b_mipi_spec,
};

//temp code
static int __init lcd_hx8369b_mipi_init(void)
{
/*
	printk("hx8369b_mipi_init: set lcd backlight.\n");

		 if (gpio_request(214, "LCD_BL")) {
			   printk("Failed ro request LCD_BL GPIO_%d \n",
					   214);
			   return -ENODEV;
		 }
		 gpio_direction_output(214, 1);
		 gpio_set_value(214, 1);
*/

	return sprdfb_panel_register(&lcd_hx8369b_mipi);
}

subsys_initcall(lcd_hx8369b_mipi_init);

/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
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

#ifndef _SPRDFB_PANEL_H_
#define _SPRDFB_PANEL_H_

#include "sprdfb.h"

#define SPRD_MAINLCD_ID 0
#define SPRD_SUBLCD_ID 1
#define SPRD_UNDEFINELCD_ID -1

#define SPRD_I2C_WRITE_DELAY 0xffff

/* LCD mode */
#define LCD_MODE_MCU			0
#define LCD_MODE_RGB			1
#define LCD_MODE_DSI			2
#define LCD_MODE_LVDS			3

/* LCD supported FPS */
#define LCD_MAX_FPS 70
#define LCD_MIN_FPS 30

/* SEND_SLEEP_CMD refer to send enter sleep cmd to panel before
 * reset panel, ssd panel need hs clk before enter sleep or reset panel */
#define SEND_SLEEP_CMD		1

/* bus mode */
#define LCD_BUS_8080			0
#define LCD_BUS_6800			1
#define LCD_BUS_SPI			2

/* lcd directions */
#define LCD_DIRECT_NORMAL		0
#define LCD_DIRECT_ROT_90		1
#define LCD_DIRECT_ROT_180		2
#define LCD_DIRECT_ROT_270		3
#define LCD_DIRECT_MIR_H		4
#define LCD_DIRECT_MIR_V		5
#define LCD_DIRECT_MIR_HV		6

/* panel property */
/* lcdc refresh, TE on, double timing, partial update*/
#define PANEL_CAP_NORMAL               0x0

/* only not support partial update*/
#define PANEL_CAP_NOT_PARTIAL_UPDATE   0x1

/* write register, grame have the same timing */
#define PANEL_CAP_UNIQUE_TIMING        0x2

/* lcd not support TE */
#define PANEL_CAP_NOT_TEAR_SYNC        0x4

/* only do command/data register, such as some mono oled display device */
#define PANEL_CAP_MANUAL_REFRESH       0x8

enum {
	SPRD_PANEL_TYPE_MCU = 0,
	SPRD_PANEL_TYPE_RGB,
	SPRD_PANEL_TYPE_MIPI,
	SPRD_PANEL_TYPE_LVDS,
	SPRD_PANEL_TYPE_LIMIT
};

enum {
	SPRD_POLARITY_POS = 0,
	SPRD_POLARITY_NEG,
	SPRD_POLARITY_LIMIT
};

enum {
	SPRD_RGB_BUS_TYPE_I2C = 0,
	SPRD_RGB_BUS_TYPE_SPI,
	SPRD_RGB_BUS_TYPE_LVDS,
	SPRD_RGB_BUG_TYPE_LIMIT
};

enum {
	SPRD_MIPI_MODE_CMD = 0,
	SPRD_MIPI_MODE_VIDEO,
	SPRD_MIPI_MODE_LIMIT
};

struct panel_spec;

typedef int32_t(*send_cmd_t)(uint32_t data);
typedef int32_t(*send_data_t)(uint32_t data);
typedef int32_t(*send_cmd_data_t)(uint32_t cmd, uint32_t data);
typedef uint32_t(*read_data_t)(void);


typedef int32_t(*mipi_set_cmd_mode_t)(void);
typedef int32_t(*mipi_set_video_mode_t)(void);
typedef int32_t(*mipi_set_lp_mode_t)(void);
typedef int32_t(*mipi_set_hs_mode_t)(void);
typedef int32_t(*mipi_gen_write_t)(uint8_t *param, uint16_t param_length);
typedef int32_t(*mipi_gen_read_t)(uint8_t *param, uint16_t param_length,
			uint8_t bytes_to_read, uint8_t *read_buffer);
typedef int32_t(*mipi_dcs_write_t)(uint8_t *param, uint16_t param_length);
typedef int32_t(*mipi_dcs_read_t)(uint8_t command,
			uint8_t bytes_to_read, uint8_t *read_buffer);
typedef	int32_t (*mipi_force_write_t)(uint8_t data_type,
			uint8_t *param, uint16_t param_length);
typedef	int32_t (*mipi_force_read_t)(uint8_t command,
			uint8_t bytes_to_read, uint8_t *read_buffer);
typedef	int32_t (*mipi_eotp_set_t)(uint8_t rx_en, uint8_t tx_en);

typedef int32_t (*i2c_write_8bits_t)(uint8_t reg, uint8_t val);
typedef int32_t (*i2c_read_8bits_t)(uint8_t reg, uint8_t *val);
typedef int32_t (*i2c_write_16bits_t)(uint16_t reg, bool reg_is_8bit,
			uint16_t val, bool val_is_8bit);
typedef int32_t (*i2c_read_16bits_t)(uint16_t reg, bool reg_is_8bit,
			uint16_t *val, bool val_is_8bit);
typedef int32_t (*i2c_write_burst_t)(uint8_t *buf, int num);

typedef void (*spi_send_cmd_t)(uint32_t cmd);
typedef void (*spi_send_data_t)(uint32_t data);
typedef void (*spi_read_t)(uint32_t *data);

/* LCD operations */
struct panel_operations {
	int32_t (*panel_init)(struct panel_spec *self);
	int32_t (*panel_close)(struct panel_spec *self);
	int32_t (*sprd_panel_reset)(struct panel_spec *self);
	int32_t (*panel_enter_sleep)(struct panel_spec *self, uint8_t is_sleep);
	int32_t (*sprd_panel_after_suspend)(struct panel_spec *self);
	int32_t (*sprd_panel_before_resume)(struct panel_spec *self);
	int32_t (*panel_set_contrast)(struct panel_spec *self,
				uint16_t contrast);
	int32_t (*panel_set_brightness)(struct panel_spec *self,
				uint16_t brightness);
	int32_t (*panel_set_window)(struct panel_spec *self,
				uint16_t left, uint16_t top,
				uint16_t right, uint16_t bottom);
	int32_t (*panel_invalidate)(struct panel_spec *self);
	int32_t (*panel_invalidate_rect)(struct panel_spec *self,
				uint16_t left, uint16_t top,
				uint16_t right, uint16_t bottom);
	int32_t (*panel_rotate_invalidate_rect)(struct panel_spec *self,
				uint16_t left, uint16_t top,
				uint16_t right, uint16_t bottom,
				uint16_t angle);
	int32_t (*panel_set_direction)(struct panel_spec *self,
				 uint16_t direction);
	uint32_t (*panel_readid)(struct panel_spec *self);
	int32_t (*panel_esd_check)(struct panel_spec *self);
	int32_t (*panel_change_fps)(struct panel_spec *self, int fps_level);
	int32_t (*panel_change_epf)(struct panel_spec *self, bool is_default);
};

/* MCU LCD specific properties */
struct timing_mcu {
	uint16_t rcss; /*unit: ns*/
	uint16_t rlpw;
	uint16_t rhpw;
	uint16_t wcss;
	uint16_t wlpw;
	uint16_t whpw;
};

/* RGB LCD specific properties */
struct timing_rgb {
	uint16_t hfp;/*unit: pixel*/
	uint16_t hbp;
	uint16_t hsync;
	uint16_t vfp; /*unit: line*/
	uint16_t vbp;
	uint16_t vsync;
};

struct ops_mcu {
	int32_t (*send_cmd)(uint32_t cmd);
	int32_t (*send_cmd_data)(uint32_t cmd, uint32_t data);
	int32_t (*send_data)(uint32_t data);
	uint32_t (*read_data)(void);
};

struct ops_i2c {
	int32_t (*i2c_write_8bits)(uint8_t reg, uint8_t val);
	int32_t (*i2c_read_8bits)(uint8_t reg, uint8_t *val);
	int32_t (*i2c_write_16bits)(uint16_t reg, bool reg_is_8bit,
				uint16_t val, bool val_is_8bit);
	int32_t (*i2c_read_16bits)(uint16_t reg, bool reg_is_8bit,
				uint16_t *val, bool val_is_8bit);
	int32_t (*i2c_write_burst)(uint8_t *buf, int num);
};

struct ops_mipi {
	int32_t (*mipi_set_cmd_mode)(void);
	int32_t (*mipi_set_video_mode)(void);
	int32_t (*mipi_set_lp_mode)(void);
	int32_t (*mipi_set_hs_mode)(void);
	int32_t (*mipi_set_data_lp_mode)(void);
	int32_t (*mipi_set_data_hs_mode)(void);
	int32_t (*mipi_gen_write)(uint8_t *param, uint16_t param_length);
	int32_t (*mipi_gen_read)(uint8_t *param, uint16_t param_length,
			uint8_t bytes_to_read, uint8_t *read_buffer);
	int32_t (*mipi_dcs_write)(uint8_t *param, uint16_t param_length);
	int32_t (*mipi_dcs_read)(uint8_t command, uint8_t bytes_to_read,
			uint8_t *read_buffer);
	int32_t (*mipi_force_write)(uint8_t data_type, uint8_t *param,
			uint16_t param_length);
	int32_t (*mipi_force_read)(uint8_t command, uint8_t bytes_to_read,
			uint8_t *read_buffer);
	int32_t (*mipi_eotp_set)(uint8_t rx_en, uint8_t tx_en);
};

struct i2c_info {
	uint32_t i2c_addr;
	struct ops_i2c *ops;
};

struct spi_info {
	struct ops_spi *ops;
};

struct info_mipi {
	uint16_t work_mode;
	uint16_t video_bus_width;
	uint32_t lan_number;
	uint32_t phy_feq;
	uint16_t h_sync_pol;
	uint16_t v_sync_pol;
	uint16_t de_pol;
	uint16_t te_pol;
	uint16_t color_mode_pol;
	uint16_t shut_down_pol;
	struct timing_rgb *timing;
	struct ops_mipi *ops;
};

struct info_rgb {
	uint16_t cmd_bus_mode;
	uint16_t video_bus_width;
	uint16_t h_sync_pol;
	uint16_t v_sync_pol;
	uint16_t de_pol;
	struct timing_rgb *timing;
	union{
		struct i2c_info *i2c;
		struct spi_info *spi;
	} bus_info;
};

struct info_mcu {
	uint16_t bus_mode;
	uint16_t bus_width;
	uint16_t bpp;
	uint16_t te_pol;
	uint32_t te_sync_delay;
	struct timing_mcu *timing;
	struct ops_mcu *ops;
};

struct panel_if_ctrl {
	const char *if_name;

	int32_t (*panel_if_check)(struct panel_spec *self);
	void (*panel_if_mount)(struct sprd_fbdev *dev);
	bool (*panel_if_init)(struct sprd_fbdev *dev);
	void (*panel_if_ready)(struct sprd_fbdev *dev);
	void (*panel_if_uninit)(struct sprd_fbdev *dev);
	void (*panel_if_before_refresh)(struct sprd_fbdev *dev);
	void (*panel_if_after_refresh)(struct sprd_fbdev *dev);
	void (*panel_if_before_panel_reset)(struct sprd_fbdev *dev);
	void (*panel_if_enter_ulps)(struct sprd_fbdev *dev);
	void (*panel_if_suspend)(struct sprd_fbdev *dev);
	void (*panel_if_resume)(struct sprd_fbdev *dev);
	uint32_t  (*panel_if_get_status)(struct sprd_fbdev *dev);

};

struct reset_timing_s {
	uint32_t time[3];
};

struct dsi_cmd_header {
	char data_type;
	char wait;
	short len;
} __packed;

struct dsi_cmd_desc {
	struct dsi_cmd_header header;
	char *payload;
};

struct panel_cmds {
	struct dsi_cmd_desc *cmds;
	unsigned int cmd_total;
	unsigned char *cmd_base;
	unsigned int byte_len;
};

struct pair_cmds{
	char addr;
	char bytes;
};
struct panel_pair_cmds{
	short num_cmd;
	struct pair_cmds *pair_cmd;
};



/* LCD abstraction */
struct panel_spec {
	uint32_t cap;
	uint16_t width;
	uint16_t height;

	struct reset_timing_s reset_timing;
	uint32_t fps;
	uint16_t suspend_mode;
	/* mcu, rgb, mipi*/
	uint16_t type;
	uint16_t direction;
	bool	 is_clean_lcd;
	union {
		struct info_mcu *mcu;
		struct info_rgb *rgb;
		struct info_mipi *mipi;
	} info;
	struct panel_if_ctrl *if_ctrl;
	struct panel_operations *ops;

	/* generic panel driver support */
	short esd;
	short simu_height;
	short simu_width;
	short mipi_write_type;

	/* ops cmds */
	struct panel_cmds 		*init_cmds;
	struct panel_cmds 		*readid_cmds;
	struct panel_cmds 		*esd_cmds;
	struct panel_cmds 		*sleep_in_cmds;
	struct panel_cmds 		*sleep_out_cmds;
	struct panel_pair_cmds 	*readid_rcmds;
	struct panel_pair_cmds 	*readid_cmp;

};

struct panel_cfg {
	struct list_head list;
	/* main_lcd, sub_lcd, undefined*/
	uint32_t dev_id;
	uint32_t lcd_id;
	const char *lcd_name;
	struct panel_spec *panel;
};

int sprd_panel_register(struct panel_cfg *cfg);

void sprd_panel_change_fps(struct sprd_fbdev *dev, int fps_level);

static inline uint32_t rgb_calc_h_timing(struct timing_rgb *timing)
{
	return  (timing->hsync | (timing->hbp << 8) | (timing->hfp << 20));
}
static inline uint32_t rgb_calc_v_timing(struct timing_rgb *timing)
{
	return (timing->vsync | (timing->vbp << 8) | (timing->vfp << 20));
}

/* Current panel configuration */
extern struct panel_cfg *cur_panel_cfg;

/* Panel interface control */
extern struct panel_if_ctrl sprd_fb_mcu_ctrl;
extern struct panel_if_ctrl sprd_fb_rgb_ctrl;
extern struct panel_if_ctrl sprd_fb_mipi_ctrl;

/* Used by MIPI LCD */
extern struct ops_mipi sprd_fb_mipi_ops;

/* I2C Operations */
extern bool sprd_fb_i2c_init(struct sprd_fbdev *dev);
extern bool sprd_fb_i2c_uninit(struct sprd_fbdev *dev);

/* RGB timing operations */
extern uint32_t rgb_calc_h_timing(struct timing_rgb *timing);
extern uint32_t rgb_calc_v_timing(struct timing_rgb *timing);

extern void sprd_panel_invalidate(struct panel_spec *self);
extern void sprd_panel_invalidate_rect(struct panel_spec *self,
		uint16_t left, uint16_t top, uint16_t right, uint16_t bottom);

#ifdef CONFIG_FB_ESD_SUPPORT
extern uint32_t sprd_panel_ESD_check(struct sprd_fbdev *dev);
#endif

#endif

/*
 *Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 *This software is licensed under the terms of the GNU General Public
 *License version 2, as published by the Free Software Foundation, and
 *may be copied, distributed, and modified under those terms.
 *
 *This program is distributed in the hope that it will be useful,
 *but WITHOUT ANY WARRANTY; without even the implied warranty of
 *MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *GNU General Public License for more details.
 */

#ifndef _INTERFACE_H_
#define _INTERFACE_H_

#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/semaphore.h>
#include <video/display_timing.h>

#ifdef pr_fmt
#undef pr_fmt
#define pr_fmt(__fmt) "[sprd-adf][%20s] "__fmt, __func__
#endif

enum {
	SPRD_MAINLCD_ID = 0,
	SPRD_SUBLCD_ID,
	SPRD_MAX_LCD_ID,
};

/* LCD supported FPS */
#define LCD_MAX_FPS 70
#define LCD_MIN_FPS 30

enum {
	SPRD_PANEL_TYPE_MCU = 0,
	SPRD_PANEL_TYPE_RGB,
	SPRD_PANEL_TYPE_MIPI,
	SPRD_PANEL_TYPE_LVDS,
	SPRD_PANEL_TYPE_SPI,
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

enum {
	PANEL_VIDEO_NON_BURST_SYNC_PULSES = 0,
	PANEL_VIDEO_NON_BURST_SYNC_EVENTS,
	PANEL_VIDEO_BURST_MODE
};

enum {
	RGB_LCD_H_TIMING = 0,
	RGB_LCD_V_TIMING,
	RGB_LCD_TIMING_KIND_MAX
};

enum {
	CMD_CODE_INIT = 0,
	CMD_CODE_SLEEP_IN,
	CMD_CODE_SLEEP_OUT,
	CMD_OLED_BRIGHTNESS,
	CMD_OLED_REG_LOCK,
	CMD_OLED_REG_UNLOCK,
	CMD_SPI_ENABLE_TE,
	CMD_SPI_DISABLE_TE,
	CMD_CODE_RESERVED0,
	CMD_CODE_RESERVED1,
	CMD_CODE_RESERVED2,
	CMD_CODE_RESERVED3,
	CMD_CODE_RESERVED4,
	CMD_CODE_RESERVED5,
	CMD_CODE_MAX,
};

enum {
	ESD_MODE_NONE = 0,
	ESD_MODE_VIDEO,
	ESD_MODE_CMD_STOP_DPU,
	ESD_MODE_WAIT_TE,
	ESD_MODE_MAX
};

struct rgb_timing {
	uint16_t hfp;		/*unit: pixel */
	uint16_t hbp;
	uint16_t hsync;
	uint16_t vfp;		/*unit: line */
	uint16_t vbp;
	uint16_t vsync;
};

struct dsi_cmd_header {
	uint8_t data_type;
	uint8_t wait;
	uint16_t len;
} __packed;

struct dsi_cmd_desc {
	struct dsi_cmd_header header;
	char *payload;
};

struct panel_cmds {
	struct dsi_cmd_desc *cmds;
	uint32_t cmd_total;
	uint8_t *cmd_base;
	uint32_t byte_len;
};

struct gpio_timing {
	uint32_t gpio;
	uint32_t level;
	uint32_t delay;
};

struct power_sequence {
	uint32_t items;
	struct gpio_timing *timing;
};

/* LCD abstraction */
struct panel_info {
	/* common parameters */
	const char *lcd_name;
	uint8_t type;
	uint8_t bpp;
	uint8_t fps;
	uint16_t width;
	uint16_t height;
	uint16_t width_mm;
	uint16_t height_mm;
	uint16_t id_reg;
	uint16_t id_size;
	uint32_t id_val[4];
	uint16_t esd_reg;
	uint16_t esd_timeout;
	uint16_t esd_read_count;
	uint32_t esd_return_code[4];
	wait_queue_head_t wq_te_esd;
	bool te_esd_flag;
	bool te_esd_waiter;
	struct panel_cmds *cmd_codes[CMD_CODE_MAX];
	struct power_sequence pwr_on_seq;
	struct power_sequence pwr_off_seq;
	struct regulator *supply;

	/* DPI specific parameters */
	uint32_t pixel_clk;
	uint16_t h_sync_pol;
	uint16_t v_sync_pol;
	uint16_t de_pol;
	uint16_t te_pol;
	struct display_timing display_timing;
	struct rgb_timing rgb_timing;

	/* MIPI DSI specific parameters */
	uint32_t phy_freq;
	uint8_t lane_num;
	uint8_t dln_timer;
	uint8_t work_mode;	/*command_mode, video_mode */
	uint8_t burst_mode;	/*burst, non-burst */
	bool nc_clk_en;

	/* platform specific parameters */
	bool dsc_en;
	bool bv3_en;
	bool is_oled;
	int esd_check_en;
	uint8_t dev_id;	/*main_lcd, sub_lcd, undefined */
	bool low_res_simu;
	uint16_t simu_width;
	uint16_t simu_height;

	/* spi interface spec param*/
	uint16_t spi_bus_mode;
	uint16_t spi_te_pol;
	uint16_t spi_mode;
	uint16_t spi_cd_gpio;
	uint16_t spi_te_gpio;
	uint32_t spi_freq;
	uint32_t spi_sync_delay;
	uint8_t spi_bus_num;
	uint8_t spi_pol_mode;
	uint8_t spi_cs;
	uint8_t	spi_endian;
	struct device_node *of_node;
	struct panel_device *pd;
};

struct panel_device;

struct panel_ops {
	int32_t (*init)(struct panel_device *dev);
	int32_t (*close)(struct panel_device *dev);
	int32_t (*reset)(struct panel_device *dev);
	int32_t (*sleep_in)(struct panel_device *dev);
	int32_t (*set_contrast)(struct panel_device *dev,
				      uint16_t contrast);
	int32_t (*set_brightness)(struct panel_device *dev,
				int level);
	int32_t (*set_direction)(struct panel_device *dev,
				       uint16_t direction);
	uint32_t (*read_id)(struct panel_device *dev);
	int32_t (*esd_check)(struct panel_device *dev);
	int32_t (*enable_te)(struct panel_device *dev);
	int32_t (*disable_te)(struct panel_device *dev);
	int32_t (*change_fps)(struct panel_device *dev, int fps);
	int32_t (*send_cmd)(struct panel_device *dev, int cmd_id);
};

struct panel_device {
	struct device dev;
	struct device *intf;
	struct panel_ops *ops;
	struct panel_info *panel;
	struct notifier_block nb;
	struct delayed_work esd_work;
	bool esd_work_start;
};

struct sprd_oled_device {
	struct backlight_device *bd;
	struct panel_device *pd;
	int max_level;
};

extern struct panel_ops panel_ops_mipi;
extern struct panel_ops panel_ops_rgb;
extern struct panel_ops panel_ops_spi;

void panel_power_ctrl(struct power_sequence *seq);

#endif

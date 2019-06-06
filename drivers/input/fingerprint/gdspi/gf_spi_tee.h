/*
 * Copyright (C) 2013-2016, Shenzhen Huiding Technology Co., Ltd.
 * All Rights Reserved.
 */
#ifndef __GF_SPI_TEE_H
#define __GF_SPI_TEE_H

#include <linux/types.h>
#include <linux/netlink.h>
#include <linux/cdev.h>
#include <linux/platform_data/spi-s3c64xx.h>
#include <linux/wakelock.h>

/**************************debug******************************/
#define ERR_LOG  (0)
#define INFO_LOG (1)
#define DEBUG_LOG (2)


#define gf_debug(level, fmt, args...)  do { \
			if (g_debug_level >= level) {\
				pr_warn("[gf] " fmt, ##args); \
			} \
		} while (0)

#define FUNC_ENTRY()  gf_debug(DEBUG_LOG, "%s, %d, enter\n", __func__, __LINE__)
#define FUNC_EXIT()  gf_debug(DEBUG_LOG, "%s, %d, exit\n", __func__, __LINE__)

/**********************IO Magic**********************/
#define GF_IOC_MAGIC	'g'

enum gf_key_event {
	GF_KEY_NONE = 0,
	GF_KEY_HOME,
	GF_KEY_POWER,
	GF_KEY_MENU,
	GF_KEY_BACK,
	GF_KEY_CAPTURE,
	GF_KEY_UP,
	GF_KEY_DOWN,
	GF_KEY_RIGHT,
	GF_KEY_LEFT,
	GF_KEY_TAP,
	GF_KEY_HEAVY
};

struct gf_key {
	enum gf_key_event key;
	uint32_t value;   /* key down = 1, key up = 0 */
};

enum gf_netlink_cmd {
	GF_NETLINK_TEST = 0,
	GF_NETLINK_IRQ = 1,
	GF_NETLINK_SCREEN_OFF,
	GF_NETLINK_SCREEN_ON
};

struct gf_ioc_chip_info {
	u8 vendor_id;
	u8 mode;
	u8 operation;
	u8 reserved[5];
};

/* define commands */
#define GF_IOC_INIT				 _IOR(GF_IOC_MAGIC, 0, u8)
#define GF_IOC_EXIT				 _IO(GF_IOC_MAGIC, 1)
#define GF_IOC_RESET			 _IO(GF_IOC_MAGIC, 2)

#define GF_IOC_ENABLE_IRQ		 _IO(GF_IOC_MAGIC, 3)
#define GF_IOC_DISABLE_IRQ		 _IO(GF_IOC_MAGIC, 4)

#define GF_IOC_ENABLE_SPI_CLK		_IOW(GF_IOC_MAGIC, 5, u32)
#define GF_IOC_DISABLE_SPI_CLK		_IO(GF_IOC_MAGIC, 6)

#define GF_IOC_ENABLE_POWER			_IO(GF_IOC_MAGIC, 7)
#define GF_IOC_DISABLE_POWER		_IO(GF_IOC_MAGIC, 8)

#define GF_IOC_INPUT_KEY_EVENT		_IOW(GF_IOC_MAGIC, 9, struct gf_key)

/* fp sensor has change to sleep mode while screen off */
#define GF_IOC_ENTER_SLEEP_MODE		_IO(GF_IOC_MAGIC, 10)
#define GF_IOC_GET_FW_INFO			_IOR(GF_IOC_MAGIC, 11, u8)
#define GF_IOC_REMOVE				_IO(GF_IOC_MAGIC, 12)
#define GF_IOC_CHIP_INFO	_IOR(GF_IOC_MAGIC, 13, struct gf_ioc_chip_info)

/* for SPI REE transfer */
#define GF_IOC_TRANSFER_CMD	_IOWR(GF_IOC_MAGIC, 15, struct gf_ioc_transfer)

#define  GF_IOC_MAXNR    16  /* THIS MACRO IS NOT USED NOW... */

struct gf_device {
	dev_t devno;
	int device_count;
	u8 buf_status;
	struct cdev cdev;
	struct device *device;
	struct class *class;
	struct platform_device *pdev;

	struct list_head device_entry;
	struct input_dev *input;
	struct mutex buf_lock;
	struct mutex release_lock;
	struct sock *nl_sk;
	struct regulator *gx_power;
	struct notifier_block notifier;
	u8 probe_finish;
	u8 irq_count;
	/* bit24-bit32 of signal count */
	/* bit16-bit23 of event type */
	/*  1: key down; 2: key up; 3: fp data ready; 4: home key */
	/* bit0-bit15 of event type, buffer status register */
	u32 event_type;
	u8 sig_count;
	u8 is_sleep_mode;
	u8 system_status;
	int cs_gpio;
	int reset_gpio;
	int irq_gpio;
	int en3v_gpio;
	int en1v8_gpio;
	u32 irq_num;
	u8  need_update;
	u32 irq;

	struct wake_lock ttw_wl;

#ifdef CONFIG_OF
	struct pinctrl *pinctrl_gpios;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_miso_spi, *pins_miso_pullhigh;
	struct pinctrl_state *pins_miso_pulllow;
	struct pinctrl_state *pins_reset_high, *pins_reset_low;
#endif
};

#endif	/* __GF_SPI_TEE_H */

/*
 * Copyright (C) 2012-2022 fpsensor Technology (Beijing) Co., Ltd.
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

#ifndef __FPSENSOR_SPI_H
#define __FPSENSOR_SPI_H

#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/wait.h>

/**********************IO Magic**********************/
#define CONFIG_OF 1
#define FPSENSOR_IOC_MAGIC    0xf0

typedef enum fpsensor_key_event {
	FPSENSOR_KEY_NONE = 0,
	FPSENSOR_KEY_HOME,
	FPSENSOR_KEY_POWER,
	FPSENSOR_KEY_MENU,
	FPSENSOR_KEY_BACK,
	FPSENSOR_KEY_CAPTURE,
	FPSENSOR_KEY_UP,
	FPSENSOR_KEY_DOWN,
	FPSENSOR_KEY_RIGHT,
	FPSENSOR_KEY_LEFT,
	FPSENSOR_KEY_TAP,
	FPSENSOR_KEY_HEAVY
} fpsensor_key_event_t;

struct fpsensor_key {
	enum fpsensor_key_event key;
	uint32_t value;   /* key down = 1, key up = 0 */
};

/* define commands */
#define FPSENSOR_IOC_INIT                       _IOWR(FPSENSOR_IOC_MAGIC, 0, unsigned int)
#define FPSENSOR_IOC_EXIT                       _IOWR(FPSENSOR_IOC_MAGIC, 1, unsigned int)
#define FPSENSOR_IOC_RESET                      _IOWR(FPSENSOR_IOC_MAGIC, 2, unsigned int)
#define FPSENSOR_IOC_ENABLE_IRQ                 _IOWR(FPSENSOR_IOC_MAGIC, 3, unsigned int)
#define FPSENSOR_IOC_DISABLE_IRQ                _IOWR(FPSENSOR_IOC_MAGIC, 4, unsigned int)
#define FPSENSOR_IOC_GET_INT_VAL                _IOWR(FPSENSOR_IOC_MAGIC, 5, unsigned int)
#define FPSENSOR_IOC_DISABLE_SPI_CLK            _IOWR(FPSENSOR_IOC_MAGIC, 6, unsigned int)
#define FPSENSOR_IOC_ENABLE_SPI_CLK             _IOWR(FPSENSOR_IOC_MAGIC, 7, unsigned int)
#define FPSENSOR_IOC_ENABLE_POWER               _IOWR(FPSENSOR_IOC_MAGIC, 8, unsigned int)
#define FPSENSOR_IOC_DISABLE_POWER              _IOWR(FPSENSOR_IOC_MAGIC, 9, unsigned int)
#define FPSENSOR_IOC_INPUT_KEY_EVENT            _IOWR(FPSENSOR_IOC_MAGIC, 10, struct fpsensor_key)
/* fp sensor has change to sleep mode while screen off */
#define FPSENSOR_IOC_ENTER_SLEEP_MODE           _IOWR(FPSENSOR_IOC_MAGIC, 11, unsigned int)
#define FPSENSOR_IOC_REMOVE                     _IOWR(FPSENSOR_IOC_MAGIC, 12, unsigned int)
#define FPSENSOR_IOC_CANCEL_WAIT                _IOWR(FPSENSOR_IOC_MAGIC, 13, unsigned int)

#define FPSENSOR_IOC_MAXNR    14  /* THIS MACRO IS NOT USED NOW... */
static const char * const pctl_names[] = {
	"fpsensor_reset_reset",
	"fpsensor_reset_active",
	"fpsensor_irq_active",
};

typedef struct {
	dev_t devno;
	struct class           *class;
	struct device          *device;
	struct cdev            cdev;

	spinlock_t    spi_lock;
	struct platform_device *platform_device;
	struct list_head device_entry;
	struct input_dev *input;

	/* buffer is NULL unless this device is open (users > 0) */
	unsigned int users;
	u8 *buffer;
	struct mutex buf_lock;
	u8 buf_status;
	u8 device_available;    /* changed during fingerprint chip sleep and wakeup phase */
	int device_count;
	/* fasync support used */
	u8 probe_finish;
	u8 irq_count;
	/* bit24-bit32 of signal count */
	/* bit16-bit23 of event type, 1: key down; 2: key up; 3: fp data ready; 4: home key */
	/* bit0-bit15 of event type, buffer status register */
	u32 event_type;
	u8 sig_count;
	u8 is_sleep_mode;
	/* for netlink use */
	struct sock *nl_sk;
	int pid;
	volatile unsigned int RcvIRQ;
	int irq;
	int irq_gpio;
	int reset_gpio;
	int power_gpio;
	struct wake_lock ttw_wl;
	/*pinctrl*/
	struct pinctrl *fingerprint_pinctrl;
	struct pinctrl_state *pinctrl_state[ARRAY_SIZE(pctl_names)];
	wait_queue_head_t wq_irq_return;
	int cancel;
    u8 suspend_flag;
} fpsensor_data_t;
extern void setRcvIRQ(int  val);

#endif    /* __FPSENSOR_SPI_TEE_H */

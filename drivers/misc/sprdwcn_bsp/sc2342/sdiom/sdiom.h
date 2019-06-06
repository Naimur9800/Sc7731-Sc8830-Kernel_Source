#ifndef __SDIOM_H__
#define __SDIOM_H__

#include <linux/delay.h>
#include <linux/export.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/suspend.h>
#include <linux/syscalls.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/uaccess.h>

struct sdio_data {
	u32 wake_ack;
	u32 data_irq;
	u32 wake_out;
	u32 io_ready;
	u32 rfctl_off;
	const char *sdhci;
};

#define SDIOM_BLK_SIZE	512

#define SDIOM_MAX_FUNCS 2
#define FUNC_0  0
#define FUNC_1  1

#define SDIOM_DT_MODE_ADDR	0x0f
#define SDIOM_PK_MODE_ADDR	0x20

#define SDIOM_CCCR_ABORT	0x06
#define VAL_ABORT_TRANS	0x01

#define SDIOM_FBR_SYSADDR_0	0x15c
#define SDIOM_FBR_SYSADDR_1	0x15d
#define SDIOM_FBR_SYSADDR_2	0x15e
#define SDIOM_FBR_SYSADDR_3	0x15f

#define SDIOM_FBR_APBRW0_0	0x180
#define SDIOM_FBR_APBRW0_1	0x181
#define SDIOM_FBR_APBRW0_2	0x182
#define SDIOM_FBR_APBRW0_3	0x183

#define SDIOM_FBR_STBBA_0	0x1bc
#define SDIOM_FBR_STBBA_1	0x1bd
#define SDIOM_FBR_STBBA_2	0x1be
#define SDIOM_FBR_STBBA_3	0x1bf

#define SDIOM_FBR_DEINT_EN	0x1ca
#define VAL_DEINT_ENABLE	0x3

#define SDIOM_FBR_PUBINT_RAW4_0	0x1e8

#define DUMP_REG  (0x140)
#define DUMP_MIN_REG (0x7)
#define DUMP_REG_1a4  (0x1a4)
#define DUMP_REG_144  (0x144)
#define DUMP_REG_4 (0x4)

int sdiom_init(void);

/* for debugfs */
void sdiom_debug_init(void);
void sdiom_debug_deinit(void);
void sdiom_print_buf(unsigned char *buf, unsigned int len, const char *func);
unsigned int sdiom_get_debug_level(void);
void sdiom_set_debug_level(unsigned int level);

int sdiom_packer_init(void);
int sdiom_packer_deinit(void);
void sdiom_sdio_tx_status(void);
int sdiom_sdio_pt_write(void *src, unsigned datalen);
int sdiom_sdio_pt_read(void *src, unsigned datalen);
void sdiom_enable_rx_irq(void);
unsigned int sdiom_get_trans_pac_num(void);
int sdiom_aon_readb(unsigned int addr, unsigned char *val);
int sdiom_aon_writeb(unsigned int addr, unsigned char val);
unsigned int sdiom_get_carddump_status(void);
void sdiom_wakelock_init(void);
int sdiom_enable_slave_int(void);
void sdiom_unlock_rx_wakelock(void);
void sdiom_unlock_de0_wakelock(void);
int sdiom_driver_register(void);
void sdiom_driver_unregister(void);

#endif /* __SDIOM_H__ */

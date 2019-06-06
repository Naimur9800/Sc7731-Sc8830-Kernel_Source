#ifndef __SDIOM_DEPEND_H__
#define __SDIOM_DEPEND_H__

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "sdiom.h"

#define sdiom_debug(fmt, args...) \
	pr_debug("sdiom:" fmt, ## args)
#define sdiom_info(fmt, args...) \
	pr_info("sdiom:" fmt, ## args)
#define sdiom_err(fmt, args...) \
	pr_err("sdiom:" fmt, ## args)
#define sdiom_print(fmt, args...) ({ \
	if (sdiom_get_debug_level() == LEVEL_0) \
		sdiom_debug(fmt, ## args); \
	else if (sdiom_get_debug_level() == LEVEL_1) \
		sdiom_info(fmt, ## args); })

enum sdiom_debug {
	BASE_LEVEL = 0,
	LEVEL_0 = BASE_LEVEL, /* close debug info */
	LEVEL_1, /* for debug list and data */
	LEVEL_MAX
};

struct sdiom_sema_box_t {
	struct semaphore tx_core_sema;
	struct semaphore tx_trans_sema;
	struct semaphore rx_core_sema;
	struct semaphore rx_trans_sema;
};

extern void sdiom_sema_init(void);

extern void sdiom_tx_core_up(void);
extern void sdiom_tx_core_down(void);
extern void sdiom_tx_trans_up(void);
extern void sdiom_tx_trans_down(void);

extern void sdiom_rx_core_up(void);
extern void sdiom_rx_core_down(void);
extern void sdiom_rx_trans_up(void);
extern void sdiom_rx_trans_down(void);

extern void *sdiom_memset(void *dest, int c, unsigned int count);
extern void *sdiom_malloc(unsigned int size);
extern void sdiom_free(void *memblock);

extern void os_sleep(unsigned int i);
extern void sdiom_rx_cb_mutex_init(void);
extern void sdiom_rx_cb_lock(void);
extern void sdiom_rx_cb_unlock(void);
extern void sdiom_tx_mutex_init(void);
extern void sdiom_tx_lock(void);
extern void sdiom_tx_unlock(void);

#endif /* __SDIOM_DEPEND_H__ */

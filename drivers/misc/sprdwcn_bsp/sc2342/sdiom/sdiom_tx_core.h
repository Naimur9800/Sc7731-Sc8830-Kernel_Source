#ifndef __SDIOM_TX_CORE_H__
#define __SDIOM_TX_CORE_H__

#include <linux/marlin_platform.h>
#include <linux/wakelock.h>

#include "sdiom.h"

extern void sdiom_tx_core_up(void);
extern void sdiom_tx_core_down(void);

extern void sdiom_tx_core_task(void);

#endif /* __SDIOM_TX_CORE_H__ */

#ifndef __SDIOM_DEBUG_H__
#define __SDIOM_DEBUG_H__

#include <linux/debugfs.h>

#include "sdiom_depend.h"
#include "sdiom_rx_recvbuf.h"

#define SDIOM_WRITE_SIZE 64

#define REG_AP_INT_CP0	0x1b0
#define BIT_AP_INT_CP0_DEBUG	BIT(2)

#endif /* __SDIOM_DEBUG_H__ */

/*
 * File: OSInfo.h                                                          *
 * Description: Used to store Lens information                               *
 *                                                                           *
 * (C)Copyright altek Corporation 2014                                       *
 *                                                                           *
 * History:                                                                  *
 *   2015/11/9; CharlesHsu; Initial version                                *
 */
#ifndef _ALTEK_OS_INFO_H
#define _ALTEK_OS_INFO_H

#include "altek_platform_info.h"

#ifdef _SPRD_PLATFORM_G2V1
	#include <video/sprd_mm.h>
#endif

#define _ISP_LINUXOS_ENV_

#ifdef _ISP_LINUXOS_ENV_
	#ifdef _SPRD_PLATFORM_G2V1
		#define MAP_IVA_TO_KVA(x, y)     iva32_to_ka64(x, y)
		#define UNMAP_IVA_TO_KVA(x)      (x = NULL)
		#define MEMCPY(x, y, z)          memcpy(x, y, z)
		#define IOREAD32(a)              REG_RD(a)
		#define IOWRITE32(v, a)           REG_WR(a, v)
	#else
		#define MAP_IVA_TO_KVA(x, y)     ioremap_nocache(x, y)
		#define UNMAP_IVA_TO_KVA(x)      iounmap(x)
		#define MEMCPY(x, y, z)          memcpy(x, y, z)
		#define IOREAD32(x)              ioread32(x)
		#define IOWRITE32(x, y)             iowrite32(x, y)
	#endif

	#define SEMAINIT(x, y)         sema_init(x, y)
	#define SEMAPOST(x)            up(x)
	#define SEMAWAIT(x)            down_interruptible(x)
	#define SEMAERR                (-ERESTARTSYS)
	#define INTDISABLE(x)          disable_irq_nosync(x)
	#define INTENABLE(x)           enable_irq(x)
	#define SLEEPRANGE(x, y)       usleep_range(x, y)
	#elif defined _ISP_NONOS_ENV_
	#define IOREMAP_NOCACHE(x, y)   x
	#define SEMAINIT(x, y)
	#define SEMAPOST(x)
	#define SEMAWAIT(x)            0
	#define SEMAERR                0xffffffff
	#define INTDISABLE(x)
	#define INTENABLE(x)
	#define IOREAD32(x)            (*(x))
	#define SLEEPRANGE(x, y)       delay_cycles((x)*1000)
#endif

#endif

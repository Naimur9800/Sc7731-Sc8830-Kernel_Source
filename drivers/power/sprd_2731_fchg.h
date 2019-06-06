#ifndef _SPRD_2731_FCHG_H_
#define _SPRD_2731_FCHG_H_

#include <linux/types.h>
#include <linux/wakelock.h>

#define FCHG_TIME1		(0x0)
#define FCHG_TIME2		(0x4)
#define FCHG_DELAY		(0x8)
#define FCHG_CFG			(0xc)
#define FCHG_TO_SW		(0x10)
#define FCHG_INT_CTL		(0x14)

#define FCHG_TIME1_BITS(x)		((x) & 0x7ff)
#define FCHG_TIME2_BITS(x)		((x) & 0xfff)

#define FCHG_DET_VOL_BITS(x)		(((x) & 0x3) << 8)
#define FCHG_ENABLE_BIT		(BIT(0))

#define FCHG_ERR_REQ_BIT		(BIT(0))
#define FCHG_DET_OK_BIT		(BIT(1))
#define FCHG_INT_CLR_BIT			(BIT(2))
#define FCHG_INT_EN_BIT			(BIT(3))
#define FCHG_OUT_OK_BIT		(BIT(3))
#define FCHG_ERR0_BIT		(BIT(2))
#define FCHG_ERR1_BIT		(BIT(1))
#define FCHG_ERR2_BIT		(BIT(0))

struct sprd_fchg_platform_data {
	uint32_t fchg_irq;
	unsigned long base_addr;
};

struct sprd_fchg_data {
	int fchg_detect_ok;
	int fchg_check_flag;
	struct device *dev;
	struct mutex lock;
	struct sprd_fchg_platform_data *fchg_pdata;
};
#endif


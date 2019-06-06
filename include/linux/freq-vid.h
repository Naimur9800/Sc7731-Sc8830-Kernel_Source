#ifndef _LINUX_FREQ_VID_H
#define _LINUX_FREQ_VID_H

#include <linux/init.h>
#include <linux/types.h>

#ifdef ZEBU
#define NO_OF_PSTATES			0x3
#define LFM_INDEX			0x0
#define HFM_INDEX			0x2
#else
#define NO_OF_PSTATES			0x6
#define LFM_INDEX			0x0
#define HFM_INDEX			0x5
#endif

#define FSB_FREQ_MHZ			0x4E
#define VT_BASE_OFF			0x20
#define MODULE0				0x00
#define MODULE1				0x01

#define ADDR_DVFS_EN_MOD0		0xE44B0000
#define ADDR_DVFS_EN_MOD1		0xE44C0000
#define ADI_BASE_ADDR			0xE4030000

#define PMIC_VLT_AVAIL_BIT		31
#define PMIC_CTRL_BIT			5
#define PMIC_CAL_BIT			0

#define PMIC_VLT_INTERCEPT		400
#define PMIC_CTRL_MUL			100

/* FIXED POINT CAL PRECISION	3.125 * 256*/
#define PMIC_CAL_PRECISION		800

enum vid_encode {
	VID_V0 = 0x0,
	VID_V1,
	VID_V2,
	VID_V3,
	VID_V4,
	VID_V5,
	VID_V6,
	VID_V7,
	VID_INVALID,
};

struct pmic_vid {
	u32 cal:5;
	u32 ctrl:5;
	u32 res:21;
	u32 avail:1;
};

union ia_core_ratios {
	u32    raw;
	struct {
		u32	min_ratio:6;
		u32	reserved1:2;
		u32	lfm_ratio:6;
		u32	reserved2:2;
		u32	guar_ratio:6;
		u32	reserved3:10;
	} field;
}; /* ia core to bus frequency ratios */

#endif

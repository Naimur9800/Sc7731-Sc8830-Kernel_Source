#ifndef __AVOID_DISTURB_H__
#define __AVOID_DISTURB_H__

#include <linux/avoid_disturb_comm.h>

#define DBS_BIT_NUM		8
#define DBS_MSK			GENMASK(7, 0)
#define DBN_MSK			(DBS_MSK << DBS_BIT_NUM)

#define UNI_DELT_NUM		4
#define POS_SPREAD_MSK		GENMASK(3, 0)
#define NEG_SPREAD_MSK		GENMASK(8, 4)
#define POS_NORM_MSK		GENMASK(11, 8)
#define NEG_NORM_MSK		GENMASK(15, 12)

#define NORMAL_MODE		0
#define SPREAD_MODE		1

enum DISTURBED_ID_E {
	DISTURBED_ID_INVALID = 0,
	DISTURBED_ID_GSM,
	DISTURBED_ID_WCDMA,
	DISTURBED_ID_WIFI,
	DISTURBED_ID_BT,
	DISTURBED_ID_FM,
	DISTURBED_ID_GNSS,
	DISTURBED_ID_MAX
};

/* priority of disturbed part */
enum PRIORITY_LEVEL_E {
	PRIORITY_LEVEL_0 = 0,
	PRIORITY_LEVEL_1,
	PRIORITY_LEVEL_2,
	PRIORITY_LEVEL_3,
	PRIORITY_LEVEL_4,
	PRIORITY_LEVEL_5,
	PRIORITY_LEVEL_MAX
};

enum CLK_MODE_E {
	CLK_MODE_NORMAL = 0,
	CLK_MODE_SSC,
};

struct pll_chg_info {
	/* 0: norma 1:spread */
	u8 prefer;
	/* 0 stand for no requirement */
	u32 delt;
	u32 prio;
};

/*
 * The struct is used to store module current statue info.
 * time_stamp: time of last set
 * state: 0: Off 1: On
 * clk_mode: 0: Normal 1: SpreadSpectrum
 * delt_freq_index: lower 4bits: positive change
 *                  higher 4bits: negative change
 *                  0xf means no disturb
 * clk_freq_index: clock freq id
 * pll_id: pll id info
 */
struct disturbing_curstatus {
	u32 time_stamp;
	u8 state:4;
	u8 clk_mode:4;
	u8 delt_freq_index;
	u8 clk_freq_index;
	u8 pll_id;
} __aligned(1);

/*
 * The struct is used for store real request info.
 * delt_bitmap_spread: lower 4bit: positive chage,
 *                         higher 4bits: negative change
 *			   0xf means no disturb
 * delt_bitmap_normal: lower 4bit: positive chage,
 *                         higher 4bits: negative change
 *                         0xf menas no disturb
 * clk_freq_index: clock freq id.
 * reserved: for future use.
 */
struct delt_bitmap {
	u8 delt_bitmap_spread;
	u8 delt_bitmap_normal;
	u8 clk_freq_index;
	u8 pll_id;
} __aligned(1);

struct ads_sipc_message {
	u16 disturbed_id;
	u32 disturbing_id;
} __aligned(1);

/*
 * The struct is used for store request head info.
 * prio: the priority of the request
 * src_id: the disturbing id
 * prefer: the preferred solution
 * reserved: for future use
 */
struct avoidance_src_info {
	u8 prio;
	u8 src_id;
	u8 prefer;
	u8 reserved;
} __aligned(1);

struct request_info {
	struct avoidance_src_info src_info;
	struct delt_bitmap *delt;
} __aligned(1);

struct request_infos {
	u8 disturbing_cnt;
	u8 disturbed_cnt;
	u8 *clk_src_cnt;
	void __iomem *base_addr;
	void __iomem *cur_info_base;
	void __iomem *req_info_base;
	struct request_info *req_info;
};

struct clk_info {
	u8 *pll_id;
	unsigned int ads_pll_num;
	unsigned long *pll_old_rate;
	void __iomem *ccs_base_addr;
	struct clk **clk_array;
};

struct avoid_disturb {
	struct clk_info c_info;
	unsigned long sthread_num;
	struct task_struct *ads_mthread;
	struct ads_sipc_message *ads_smsg_array;
	struct task_struct **ads_sthread;
};

#endif

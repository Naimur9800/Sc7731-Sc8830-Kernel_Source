/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
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

#ifndef __SPRD_BM_PERF_H__
#define __SPRD_BM_PERF_H__
#include <linux/dmaengine.h>

/*
 *				BM  Define
 */
#ifdef CONFIG_X86
extern void ap_light_enable_disable(unsigned int req, const char *name);

static inline void close_smart_lightsleep(void)
{
	ap_light_enable_disable(0, "perf_bm");
}

static inline void open_smart_lightsleep(void)
{
	ap_light_enable_disable(1, "perf_bm");
}
#else
static inline void close_smart_lightsleep(void)
{
}

static inline void open_smart_lightsleep(void)
{
}
#endif

#define REG_PUB0_APB_BUSMON_CFG			(0x0004)
#define REG_PUB1_APB_BUSMON_CFG			(0x0004)

/* bm register CHN CFG bit map definition */
#define BM_INT_CLR				BIT(31)
#define BM_PERFOR_INT_MASK			BIT(30)
#define BM_F_DN_INT_MASK			BIT(29)
#define BM_F_UP_INT_MASK			BIT(28)

#define BM_PERFOR_INT_RAW			BIT(26)
#define BM_F_DN_INT_RAW				BIT(25)
#define BM_F_UP_INT_RAW				BIT(24)

#define BM_PERFOR_INT_EN			BIT(22)
#define BM_F_DN_INT_EN				BIT(21)
#define BM_F_UP_INT_EN				BIT(20)

#define BM_PERFOR_REQ_EN			BIT(8)
#define BM_F_DN_REQ_EN				BIT(7)
#define BM_F_UP_REQ_EN				BIT(6)
#define BM_RLATENCY_EN				BIT(5)
#define BM_RBW_EN				BIT(4)
#define BM_WLATENCY_EN				BIT(3)
#define BM_WBW_EN				BIT(2)
#define BM_AUTO_MODE_EN				BIT(1)
#define BM_CHN_EN				BIT(0)
#define BM_RESET_VALUE				(0x7FF)
#define BM_ENABLE_VALUE				(0x07FF0000)
#ifndef BIT_AON_APB_CGM_SP_TMR_EN
#define BIT_AON_APB_CGM_SP_TMR_EN		BIT(19)
#endif
#ifndef REG_AON_APB_EB2
#define REG_AON_APB_EB2				(0x00B0)
#endif
#ifndef BIT_REG_AON_APB_BST_TMR_EN
#define BIT_REG_AON_APB_BST_TMR_EN		BIT(14)
#endif
#ifndef REG_AON_APB_EB_AON_ADD1
#define REG_AON_APB_EB_AON_ADD1		0x01D0
#endif
#ifndef BIT_AON_APB_BSM_TMR_EB
#define BIT_AON_APB_BSM_TMR_EB			BIT(1)
#endif
#ifndef REG_AON_APB_CGM_REG1
#define REG_AON_APB_CGM_REG1                    0x0440
#endif
#define BM_CHN_BUF_MAX				11
#define BM_DATA_COUNT				(5)

#define SPRD_BM_CHN_REG(base, index)	(base + 0x10000 \
			* ((index < 10) ? index : (index + 3)))

#define BM_DMA_LOG_SIZE				(30 * MSEC_PER_SEC)
#define BM_REG_IN_WINDOW(base)			((base) + 0x28)
#define BM_DMA_FRAG_LEN				0x18
#define BM_DMA_LLINK_SIZE			50
#define BM_CHN_REG_OFFSET			0x10000
#define BM_CHN_DEF_WINLEN			80
#define BM_DMA_BLOCK_STEP			(BM_CHN_REG_OFFSET\
						- BM_DMA_FRAG_LEN)

#define BM_DMA_BUF_LEN(circle, bm_cnt)		((circle) * (bm_cnt)\
						* BM_DMA_FRAG_LEN)

enum bm_chip {
	WHALE_CHIP,
	WHALE2_CHIP,
	IWHALE2_CHIP,
	ISHARKL2_CHIP,
	SHARKL2_CHIP,
	PIKE2_CHIP
};

struct perf_data {
	const unsigned char **name_list;
	unsigned int channel_num;
	enum bm_chip chip;
};

/* bus monitor perf info */
struct bm_per_info {
	u32 count;
	u32 t_start;
	u32 t_stop;
	u32 tmp1;
	u32 tmp2;
	u32 perf_data[BM_CHN_BUF_MAX][6];
};

/* bus monitor timer info */
struct sprd_bm_tmr_info {
	u32 tmr1_len_h;
	u32 tmr1_len_l;
	u32 tmr2_len_h;
	u32 tmr2_len_l;
	u32 tmr_sel;
};


/*the buf can store 8 secondes data*/
#define BM_PER_CNT_RECORD_SIZE		800
#define BM_PER_CNT_BUF_SIZE		(sizeof(struct bm_per_info) \
					* BM_PER_CNT_RECORD_SIZE)
#define BM_LOG_FILE_PATH		"/mnt/obb/axi_per_log"
#define BM_LOG_FILE_SECONDS		(60  * 30)
#define BM_LOG_FILE_MAX_RECORDS		(BM_LOG_FILE_SECONDS * 100)
/* 0x6590 is 1ms */
#define BM_TMR_1_MS				0x6590
/* 0x6590 *10 = 0x3f7a0 = 10ms */
#define BM_TMR_H_LEN				0x3F7A0
#define BM_TMR_L_LEN				0x1

/* bm register definition */
struct sprd_bm_reg {
	u32 chn_cfg;
	u32 peak_win_len;
	u32 f_dn_rbw_set;
	u32 f_dn_rl_set;
	u32 f_dn_wbw_set;
	u32 f_dn_wl_set;
	u32 f_up_rbw_set;
	u32 f_up_rl_set;
	u32 f_up_wbw_set;
	u32 f_up_wl_set;
	u32 rtrns_in_win;
	u32 rbw_in_win;
	u32 rl_in_win;
	u32 wtrns_in_win;
	u32 wbw_in_win;
	u32 wl_in_win;
	u32 peakbm_in_win;
};

#define BM_TMR2_CNT_CLR				BIT(3)
#define BM_TMR2_EN					BIT(2)
#define BM_TMR1_CNT_CLR				BIT(1)
#define BM_TMR1_EN					BIT(0)

#define BM_CNT_STR2_ST				BIT(2)
#define BM_CNT_STR1_ST				BIT(1)
#define BM_TMR_SEL					BIT(0)

/* bm timer register definition */
struct sprd_bm_tmr_reg {
	u32 tmr_ctrl;
	u32 high_len_t1;
	u32 low_len_t1;
	u32 cnt_num_t1;
	u32 high_len_t2;
	u32 low_len_t2;
	u32 cnt_num_t2;
	u32 tmr_out_sel;
};

/* bm timer triger BM mode definition */
enum sprd_bm_tmr_sel {
	ALL_FROM_TIMER1 = 0x0,
	TRIGER_BY_EACH,
};

/* bm mode definition */
enum sprd_bm_mode {
	BM_CIRCLE_MODE = 0x0,
	BM_NORMAL_MODE,
	BM_FREQ_MODE,
};

/* bm cmd definetion, 0x16 compatible the old BM cmds */
enum sprd_bm_cmd {
	BM_DISABLE = 0x14,
	BM_ENABLE,
	BM_PROF_SET,
	BM_PROF_CLR,
	BM_CHN_CNT,
	BM_RD_CNT,
	BM_WR_CNT,
	BM_RD_BW,
	BM_WR_BW,
	BM_RD_LATENCY,
	BM_WR_LATENCY,
	BM_KERNEL_TIME,
	BM_TMR_CLR,
	BM_VER_GET,
	BM_CMD_MAX,
};

struct sprd_bm_dma_info {
	struct dma_chan				*dma;
	dma_cookie_t				cookie;
	u32					dma_id;
	unsigned long				cir;
	dma_addr_t				dst_buf_p;
	void					*dst_buf_v;
	dma_addr_t				src_buf_p;
	dma_addr_t				llist_p;
	void					*llist_v;
	struct hrtimer				timer;
};
/* bm device */
struct sprd_bm_dev {
	spinlock_t					bm_lock;
	struct device				dev;
	void __iomem				*bm_pub0_base;
	void __iomem				*bm_pub1_base;
	void __iomem				*bm_pub0_glb_base;
	void __iomem				*bm_pub1_glb_base;
	void __iomem				*bm_tmr_base;
	struct task_struct				*bm_thl;
	struct regmap				*aon_apb;
	struct sprd_bm_tmr_info			bm_tmr_info;
	struct bm_per_info				bm_perf_info;
	struct semaphore				bm_seam;
	int					bm_pub0_irq;
	int					bm_pub1_irq;
	int					bm_cnt;
	bool					bm_perf_st;
	bool					double_ddr;
	u32					bm_buf_write_cnt;
	int					*per_buf;
	const unsigned char			**name_list;
	const struct perf_data			*pdata;
	struct sprd_bm_dma_info			bm_dma_info;
};

static const unsigned char *sprd_whale_name_list[] = {
	"A53",
	"DISP",
	"GPU",
	"AP",
	"CAM",
	"VSP",
	"LWT-DSP",
	"LWT-ACC",
	"AUD-CP",
	"PUB CP",
};

static const unsigned char *sprd_whale2_name_list[] = {
	"A53",
	"DISP",
	"GPU",
	"AP",
	"CAM",
	"VSP",
	"LWT-DSP",
	"LWT-ACC",
	"PUB CP",
	"AUD-CP",
	"CAM-2",
};

static const unsigned char *sprd_iwhale2_name_list[] = {
	"BIA/AP",
	"VSP/GSP",
	"DISP/CAM",
	"GPU",
	"PUBCP",
	"WTLCP/AGCP",
};

static const unsigned char *sprd_isharkl2_name_list[] = {
	"BIA",
	"VSP/GSP/AP",
	"DISP/CAM",
	"GPU",
	"PUBCP",
	"WTLCP/AON",
};

static const unsigned char *sprd_sharkl2_name_list[] = {
	"MM",
	"GSP/GPU",
	"DISP",
	"CA7",
	"PUBCP",
	"VSP/JPEG",
	"WTLCP",
	"AON/AP",
};

static const unsigned char *sprd_pike2_name_list[] = {
	"WCN/AON",
	"GPU",
	"DISP/ISP",
	"CA7",
	"CP",
	"VSP",
	"AP",
};
static struct perf_data whale_perfdata = {sprd_whale_name_list,
	ARRAY_SIZE(sprd_whale_name_list), WHALE_CHIP};
static struct perf_data whale2_perfdata = {sprd_whale2_name_list,
	ARRAY_SIZE(sprd_whale2_name_list), WHALE2_CHIP};
static struct perf_data iwhale2_perfdata = {sprd_iwhale2_name_list,
	ARRAY_SIZE(sprd_iwhale2_name_list), IWHALE2_CHIP};
static struct perf_data isharkl2_perfdata = {sprd_isharkl2_name_list,
	ARRAY_SIZE(sprd_isharkl2_name_list), ISHARKL2_CHIP};
static struct perf_data sharkl2_perfdata = {sprd_sharkl2_name_list,
	ARRAY_SIZE(sprd_sharkl2_name_list), SHARKL2_CHIP};
static struct perf_data sharklj1_perfdata = {sprd_sharkl2_name_list,
	ARRAY_SIZE(sprd_sharkl2_name_list), SHARKL2_CHIP};
static struct perf_data pike2_perfdata = {sprd_pike2_name_list,
	ARRAY_SIZE(sprd_pike2_name_list), PIKE2_CHIP};
#endif

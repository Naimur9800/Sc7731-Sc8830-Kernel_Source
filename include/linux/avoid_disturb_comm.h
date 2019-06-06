#ifndef __AVOID_DISTURB_COMM_H__
#define __AVOID_DISTURB_COMM_H__

#include <linux/types.h>

enum DISTURBING_ID_E {
	DISTURBING_ID_DDR = 0,
	DISTURBING_ID_DSI,
	DISTURBING_ID_SDIO,
	DISTURBING_ID_EMMC,
	DISTURBING_ID_NAND,
	DISTURBING_ID_MAX = 16
};

enum PLL_ID_E {
	PLL_ID_MPLL = 0,
	PLL_ID_DPLL,
	PLL_ID_TWPLL,
	PLL_ID_CPLL,
	PLL_ID_GPLL,
	PLL_ID_MAX = 16
};

int sprd_ads_register_notifier(struct notifier_block *nb);
int sprd_ads_unregister_notifier(struct notifier_block *nb);
int sprd_ads_write_aon_cur_info(enum DISTURBING_ID_E id,
		enum PLL_ID_E pll_id, unsigned int state,
		uint8_t clk_src_id);

#endif

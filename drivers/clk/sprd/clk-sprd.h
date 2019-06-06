
#ifndef __CLK_SPRD_H__
#define __CLK_SPRD_H__

#define clk_debug(format, arg...) \
	pr_debug("clk: " "@@@%s: " format, __func__, ## arg)
#define clk_info(format, arg...) \
	pr_info("clk: " "@@@%s: " format, __func__, ## arg)
#define clk_err(format, arg...) \
	pr_err("clk: " "@@@%s: " format, __func__, ## arg)

struct clk_data {
	const char *clk_name;
	u32 msk;
};

#endif

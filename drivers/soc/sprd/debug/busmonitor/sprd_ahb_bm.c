/*copyright (C) 2015 Spreadtrum Communications Inc.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/timer.h>
#include <linux/timex.h>
#include <linux/rtc.h>

#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include "busmonitor.h"

#define SPRD_AXI_BM_OFFSET	 0x10000
#define SPRD_AHB_BM_OFFSET	 0x100000

#define AP_AHB_AHB_EB		REG_AP_AHB_AHB_EB
#define AP_AHB_AHB_RST		REG_AP_AHB_AHB_RST
#define AP_AHB_MISC_CFG		REG_AP_AHB_MISC_CFG

static const struct of_device_id bm_ahb_of_match[];
static struct sprd_bm_dev bm_dev;

static void __iomem *sprd_get_bm_base(u32 bm_index)
{
	return bm_dev.ahb_bm_base + (bm_index-AHB_BM0) * SPRD_AHB_BM_OFFSET;
}

static void sprd_ahb_bm_init(void)
{
	u32 bm_index;
	void __iomem *bm_reg;
	u32 offset = 0;

	for (bm_index = AHB_BM0; bm_index < BM_SIZE; bm_index++) {
		bm_reg = sprd_get_bm_base(bm_index);
		for (offset = 0; offset <= AXI_BM_MATCH_ID_REG; offset += 4)
				writel_relaxed(0, bm_reg + offset);
	}
}

static inline void sprd_ahb_bm_chn_int_clr(int chn)
{
	ulong val;
	void __iomem *base_reg;

	base_reg = sprd_get_bm_base(chn);
	val = readl_relaxed(base_reg + AXI_BM_INTC_REG);
	val |= BM_INT_CLR;
	writel_relaxed(val, base_reg + AXI_BM_INTC_REG);
}

static void sprd_ahb_bm_int_clr(void)
{
	int bm_index;

	for (bm_index = AHB_BM0; bm_index < BM_SIZE; bm_index++)
		sprd_ahb_bm_chn_int_clr(bm_index);
}

static int sprd_bm_glb_reset_and_enable(u32 bm_index, bool is_enable)
{
	ulong reg_en, reg_rst, bit_en, bit_rst;

	reg_en = AP_AHB_AHB_EB;
	reg_rst = AP_AHB_AHB_RST;
	bit_en = BIT(14 + bm_index - AHB_BM0);
	bit_rst = BIT(17 + bm_index - AHB_BM0);
	if (bm_index >= AHB_BM0) {
		regmap_update_bits(bm_dev.ap_ahb_gpr, reg_rst,
					bit_rst, bit_rst);
		regmap_update_bits(bm_dev.ap_ahb_gpr, reg_rst,
					bit_rst, 0);
	}
	if (is_enable)
		regmap_update_bits(bm_dev.ap_ahb_gpr,
			reg_en, bit_en, bit_en);
	else
		regmap_update_bits(bm_dev.ap_ahb_gpr,
				reg_en, bit_en, 0);

	return SPRD_BM_SUCCESS;
}

static u32 sprd_bm_get_chn_sel(u32 bm_index)
{
	u32 reg_val, chn_sel;

	regmap_read(bm_dev.ap_ahb_gpr, AP_AHB_MISC_CFG, &reg_val);
	switch (bm_index) {
	case AHB_BM0:
		reg_val &= (0x3 << 4);
		chn_sel = (reg_val >> 4) + AHB_BM0_CHN0;
		break;
	case AHB_BM1:
		reg_val &= (0x3 << 8);
		chn_sel = (reg_val >> 8) + AHB_BM1_CHN0;
		break;
	case AHB_BM2:
		reg_val &= (0x3 << 10);
		chn_sel = (reg_val >> 10) + AHB_BM2_CHN0;
		break;
	default:
		return bm_index;
	}

	return chn_sel;
}

static int sprd_bm_chn_sel(u32 bm_index, u32 chn_id)
{
	u32 reg_val;

	regmap_read(bm_dev.ap_ahb_gpr, AP_AHB_MISC_CFG, &reg_val);

	switch (bm_index) {
	case AHB_BM0:
		reg_val &= ~(0x3 << 4);
		reg_val |= (chn_id & 0x3) << 4;
		break;
	case AHB_BM1:
		reg_val &= ~(0x3 << 8);
		reg_val |= (chn_id & 0x3) << 8;
		break;
	case AHB_BM2:
		reg_val &= ~(0x3 << 10);
		reg_val |= (chn_id & 0x3) << 10;
		break;
	default:
		return -EINVAL;
	}
	regmap_write(bm_dev.ap_ahb_gpr, AP_AHB_MISC_CFG, reg_val);

	return SPRD_BM_SUCCESS;
}

static void sprd_bm_store_int_info(u32 bm_index)
{
	void __iomem *reg_addr;
	uint mask_id;

	reg_addr = sprd_get_bm_base(bm_index);
	if (bm_dev.bm_ctn_dbg.bm_continue_dbg == false) {
		bm_dev.debug_bm_int_info[bm_index].bm_index = bm_index;
		bm_dev.debug_bm_int_info[bm_index].msk_addr =
		readl_relaxed((void *)(reg_addr + AXI_BM_MATCH_ADDR_REG));
		bm_dev.debug_bm_int_info[bm_index].msk_cmd =
		readl_relaxed((void *)(reg_addr + AXI_BM_MATCH_CMD_REG));
		bm_dev.debug_bm_int_info[bm_index].msk_data_l =
		readl_relaxed((void *)(reg_addr + AXI_BM_MATCH_DATA_L_REG));
		bm_dev.debug_bm_int_info[bm_index].msk_data_h =
		readl_relaxed((void *)(reg_addr + AXI_BM_MATCH_DATA_H_REG));
		regmap_read(bm_dev.ap_ahb_gpr, REG_AP_AHB_MISC_CFG,
				&mask_id);
		bm_dev.debug_bm_int_info[bm_index].msk_id = 0;
		pr_cont("bm ahb int info:\nBM CHN:	%d\n"
			"Overlap ADDR:	0x%X\n"
			"Overlap CMD:	0x%X\n"
			"Overlap DATA:	0x%X0x%X\n"
			"Overlap ID:	%s--%u\n",
			bm_dev.debug_bm_int_info[bm_index].bm_index,
			bm_dev.debug_bm_int_info[bm_index].msk_addr,
			bm_dev.debug_bm_int_info[bm_index].msk_cmd,
			bm_dev.debug_bm_int_info[bm_index].msk_data_l,
			bm_dev.debug_bm_int_info[bm_index].msk_data_h,
			bm_dev.bm_chn_name[sprd_bm_get_chn_sel(bm_index)],
			mask_id);
	} else {
		bm_dev.bm_ctn_dbg.bm_ctn_info[bm_dev.bm_ctn_dbg.current_cnt]
		.bm_index = bm_index;
		bm_dev.bm_ctn_dbg.bm_ctn_info[bm_dev.bm_ctn_dbg.current_cnt]
		.msk_addr = readl_relaxed((void *)(reg_addr
				+ AXI_BM_MATCH_ADDR_REG));
		bm_dev.bm_ctn_dbg.bm_ctn_info[bm_dev.bm_ctn_dbg.current_cnt]
		.msk_cmd = readl_relaxed((void *)(reg_addr
				+ AXI_BM_MATCH_CMD_REG));
		bm_dev.bm_ctn_dbg.bm_ctn_info[bm_dev.bm_ctn_dbg.current_cnt]
		.msk_data_l = readl_relaxed((void *)(reg_addr
				+ AXI_BM_MATCH_DATA_L_REG));
		bm_dev.bm_ctn_dbg.bm_ctn_info[bm_dev.bm_ctn_dbg.current_cnt]
		.msk_data_h = readl_relaxed((void *)(reg_addr
				+ AXI_BM_MATCH_DATA_H_REG));
			regmap_read(bm_dev.ap_ahb_gpr,
				REG_AP_AHB_MISC_CFG, &mask_id);
		bm_dev.bm_ctn_dbg.bm_ctn_info[bm_dev
				.bm_ctn_dbg.current_cnt].msk_id = 0;
		pr_cont("bm ahb ctn info:\n"
		"BM CHN:	%d\n"
		"Overlap ADDR:	0x%X\n"
		"Overlap CMD:	0x%X\n"
		"Overlap DATA:	0x%X0x%X\n"
		"Overlap ID:	%s--%u\n",
			bm_dev.bm_ctn_dbg.bm_ctn_info[bm_dev
				.bm_ctn_dbg.current_cnt].bm_index,
			bm_dev.bm_ctn_dbg.bm_ctn_info[bm_dev
				.bm_ctn_dbg.current_cnt].msk_addr,
			bm_dev.bm_ctn_dbg.bm_ctn_info[bm_dev
				.bm_ctn_dbg.current_cnt].msk_cmd,
			bm_dev.bm_ctn_dbg.bm_ctn_info[bm_dev
				.bm_ctn_dbg.current_cnt].msk_data_l,
			bm_dev.bm_ctn_dbg.bm_ctn_info[bm_dev
				.bm_ctn_dbg.current_cnt].msk_data_h,
			bm_dev.bm_chn_name[sprd_bm_get_chn_sel(bm_index)],
			mask_id);
		bm_dev.bm_ctn_dbg.current_cnt++;
	}
}

static irqreturn_t sprd_bm_isr(int irq_num, void *dev)
{
	ulong bm_index, bm_int;
	struct bm_per_info *bm_info;
	void __iomem *bm_reg;

	bm_info = (struct bm_per_info *)bm_dev.per_buf;
	for (bm_index = AHB_BM0; bm_index < BM_SIZE; bm_index++) {
		bm_reg = sprd_get_bm_base(bm_index);
		bm_int = readl_relaxed(bm_reg
			+ AHB_BM_INTC_REG);
		if (bm_int & BM_INT_MSK_STS) {
			sprd_bm_store_int_info(bm_index);
			if ((bm_dev.bm_ctn_dbg.bm_continue_dbg == true)
				&& (bm_dev.bm_ctn_dbg.current_cnt
					<= BM_CONTINUE_DEBUG_SIZE)) {
				bm_int &= ~BM_INT_EN;
				bm_int |= (BM_INT_CLR | BM_INT_EN);
			} else {
				bm_int &= ~BM_INT_EN;
			}
			writel_relaxed(bm_int, bm_reg);
			if (bm_dev.bm_st_info.bm_stack_st == true) {
				pr_err("Bus Monitor output stack!\n");
				dump_stack();
			}
			if (true == bm_dev.bm_st_info.bm_panic_st) {
				pr_err("Bus Monitor enter panic!\n");
				BUG();
			}
		}
	}
	return IRQ_HANDLED;
}

static int sprd_bm_set_point(enum sci_bm_index bm_index, enum sci_bm_chn chn,
	const struct sci_bm_cfg *cfg, void(*call_back)(void *), void *data)
{
	int ret;
	u32 ddr_size = 0;
	struct device_node *np = NULL;
	struct resource res;
	void __iomem *bm_reg;

	bm_reg = sprd_get_bm_base(bm_index);
	/* clean the irq status */
	writel_relaxed(readl_relaxed(bm_reg + AXI_BM_INTC_REG)
				| BM_INT_CLR, bm_reg
				+ AXI_BM_INTC_REG);
	writel_relaxed(0, bm_reg + AXI_BM_INTC_REG);
	np = of_find_node_by_name(NULL, "memory");
	if (np) {
		ret = of_address_to_resource(np, 0, &res);
		if (!ret)
			ddr_size = res.end - res.start;
	}
	/* if monitor ddr addr, it needs addr wrap*/
	if (cfg->addr_min >= 0x80000000 &&
		(strcmp(bm_dev.bm_chn_name[bm_index], "ERROR_NAME"))) {
		if (ddr_size == 0x1fffffff) {/* 512M */
			bm_store_vale[bm_index].bm_mask = 0xE0000000;
		} else if (ddr_size == 0x3fffffff) {/* 1G */
			bm_store_vale[bm_index].bm_mask = 0xC0000000;
		} else {/* >= 4G, do not need mask */
			bm_store_vale[bm_index].bm_mask = 0x0;
		}
	} else {
		bm_store_vale[bm_index].bm_mask = 0x0;
	}
	if (strcmp(bm_dev.bm_chn_name[bm_index], "ERROR_NAME")) {
		if (bm_index >= AHB_BM0) {
			writel_relaxed(
				cfg->addr_min &
				(~bm_store_vale[bm_index].bm_mask), bm_reg +
				AXI_BM_ADDR_MIN_REG);
			writel_relaxed(
				cfg->addr_max &
				(~bm_store_vale[bm_index].bm_mask), bm_reg +
				AXI_BM_ADDR_MAX_REG);
			writel_relaxed(bm_store_vale[bm_index].bm_mask, bm_reg +
				AXI_BM_ADDR_MSK_REG);
			writel_relaxed(cfg->data_min_l, bm_reg +
				AXI_BM_DATA_MIN_L_REG);
			writel_relaxed(cfg->data_min_h, bm_reg +
				AXI_BM_DATA_MIN_H_REG);
			writel_relaxed(cfg->data_max_l, bm_reg +
				AXI_BM_DATA_MAX_L_REG);
			writel_relaxed(cfg->data_max_h, bm_reg +
				AXI_BM_DATA_MAX_H_REG);
			writel_relaxed(0x0, bm_reg +
				AXI_BM_DATA_MSK_L_REG);
			writel_relaxed(0x0, bm_reg +
				AXI_BM_DATA_MAX_H_REG);
			switch (cfg->bm_mode) {
			case R_MODE:
				writel_relaxed(0x1, bm_reg +
				AXI_BM_CFG_REG);
				break;
			case W_MODE:
				writel_relaxed(0x3, bm_reg +
				AXI_BM_CFG_REG);
				break;
			case RW_MODE:
				writel_relaxed(0x0, bm_reg +
				AXI_BM_CFG_REG);
				break;
			default:
				return -EINVAL;
			}
			ret = sprd_bm_chn_sel(bm_index, chn);
			if (ret < 0)
				return ret;
			writel_relaxed(readl_relaxed(bm_reg + AXI_BM_INTC_REG)
				| BM_INT_EN | BM_CHN_EN, bm_reg
				+ AXI_BM_INTC_REG);
		}
	}

	return SPRD_BM_SUCCESS;
}

static void sprd_bm_def_val_set(void)
{
	u32 bm_chn, ret;
	struct sci_bm_cfg bm_cfg;

	for (bm_chn = AHB_BM0; bm_chn < BM_SIZE; bm_chn++) {
		if ((0x0 == bm_def_value[bm_chn].str_addr)
			&& (0x0 == bm_def_value[bm_chn].end_addr))
			continue;
		bm_cfg.addr_min = bm_def_value[bm_chn].str_addr;
		bm_cfg.addr_max = bm_def_value[bm_chn].end_addr;
		bm_cfg.bm_mode = bm_def_value[bm_chn].mode;
		ret = sprd_bm_set_point(bm_chn, bm_def_value[bm_chn].chn_sel,
			&bm_cfg, NULL, NULL);
		if (SPRD_BM_SUCCESS != ret)
			return;
	}
}

static ssize_t state_show(struct device *dev,
		struct device_attribute *attr,  char *buf)
{
	return sprintf(buf, "Bus Monitor in debug mode!\n");
}

static DEVICE_ATTR_RO(state);

static ssize_t chn_show(struct device *dev,
		struct device_attribute *attr,  char *buf)
{
	u32 bm_index;
	char occ_info[30] = {};
	char chn_info[30*BM_CHANNEL_SIZE] = {};

	for (bm_index = AHB_BM0; bm_index < BM_CHANNEL_SIZE; bm_index++) {
		sprintf(occ_info, "%s\n", bm_dev.bm_chn_name[bm_index]);
		strcat(chn_info, occ_info);
	}

	return sprintf(buf, "%s\n", chn_info);
}

static DEVICE_ATTR_RO(chn);

static ssize_t ahb_dbg_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	u32 bm_index, chn_sel;
	ulong str_addr, end_addr, str_data, end_data;
	void __iomem *reg_addr;
	char chn_info[84] = {0};
	char info_buf[84 * 3] = {0};

	for (bm_index = AHB_BM0; bm_index < BM_SIZE; bm_index++) {
		reg_addr = sprd_get_bm_base(bm_index);
		str_addr = readl_relaxed(reg_addr
			+ AXI_BM_ADDR_MIN_REG)
			+ (0x80000000 & bm_store_vale[bm_index].bm_mask);
		end_addr = readl_relaxed(reg_addr
			+ AXI_BM_ADDR_MAX_REG)
			+ (0x80000000 & bm_store_vale[bm_index].bm_mask);
		str_data = readl_relaxed(reg_addr
			+ AXI_BM_DATA_MIN_L_REG);
		end_data = readl_relaxed(reg_addr
			+ AXI_BM_DATA_MAX_L_REG);
		chn_sel = sprd_bm_get_chn_sel(bm_index);
		if (chn_sel < 0)
			return -EINVAL;
		sprintf(chn_info, "%d	 0x%08lX  0x%08lX"
			"  0x%08lX  0x%08lX  %s\n",
			bm_index-10, str_addr, end_addr,
			str_data, end_data, bm_dev.bm_chn_name[chn_sel]);
		strcat(info_buf, chn_info);
	}
	pr_info("%s\n", info_buf);

	return sprintf(buf, "%s\n", info_buf);
}

static ssize_t ahb_dbg_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	unsigned char addr_start[12], data_min[12];
	unsigned char addr_end[12], data_max[12];
	unsigned long start_addr, end_addr, min_data,
			max_data, channel, chn_sel;
	struct sci_bm_cfg bm_cfg;
	int ret;

	ret = sscanf(buf, "%ld %ld %s %s %s %s", &channel, &chn_sel,
		addr_start, addr_end, data_min, data_max);
	if (ret < 6)
		return -EINVAL;
	ret = kstrtoul(addr_start, 0, &start_addr);
	if (ret)
		pr_err("start addr %s is not in hex or decimal form.\n", buf);
	ret = kstrtoul(addr_end, 0, &end_addr);
	if (ret)
		pr_err("end addr %s is not in hex or decimal form.\n", buf);
	ret = kstrtoul(data_min, 0, &min_data);
	if (ret)
		pr_err("start data %s is not in hex or decimal form.\n", buf);
	ret = kstrtoul(data_max, 0, &max_data);
	if (ret)
		pr_err("end data %s is not in hex or decimal form.\n", buf);
	pr_info("str addr 0x%lX end addr 0x%lX min data 0x%lX max data 0x%lX\n",
		start_addr, end_addr, min_data, max_data);
	if ((channel > 3) || (chn_sel > 4) || (start_addr > end_addr) ||
		(min_data > max_data))
		return -EINVAL;
	sprd_bm_glb_reset_and_enable(channel + AHB_BM0, true);
	bm_dev.bm_st_info.bm_dbg_st = true;
	bm_dev.bm_st_info.bm_dfs_off_st = true;
	bm_cfg.addr_min = (u32)start_addr;
	bm_cfg.addr_max = (u32)end_addr;
	bm_cfg.data_min_l = (u32)min_data;
	bm_cfg.data_min_h = 0x0;
	bm_cfg.data_max_l = (u32)max_data;
	bm_cfg.data_max_h = 0x0;
	bm_cfg.bm_mode = W_MODE;
	ret = sprd_bm_set_point(channel + AHB_BM0,
				chn_sel, &bm_cfg, NULL, NULL);
	if (SPRD_BM_SUCCESS != ret)
		return ret;

	return count;
}

static DEVICE_ATTR_RW(ahb_dbg);

static ssize_t occur_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	u32 bm_index;
	char occ_info[116] = {0};

	for (bm_index = AHB_BM0; bm_index < BM_SIZE; bm_index++) {
		if (bm_dev.debug_bm_int_info[bm_index].msk_addr != 0) {
			sprintf(occ_info, " %s\n addr: 0x%X\n CMD: 0x%X\n"
				"msk_data_l: 0x%X\n msk_data_h: 0x%X\n"
				"id 0x%X\n",
				bm_dev.
				bm_chn_name[sprd_bm_get_chn_sel(bm_index)],
				bm_dev.debug_bm_int_info[bm_index].msk_addr,
				bm_dev.debug_bm_int_info[bm_index].msk_cmd,
				bm_dev.debug_bm_int_info[bm_index].msk_data_l,
				bm_dev.debug_bm_int_info[bm_index].msk_data_h,
				bm_dev.debug_bm_int_info[bm_index].msk_id);
			strcat(buf, occ_info);
		}
	}
	if (occ_info[1] == 0)
		sprintf(buf, ":-) ! No action was monitored by BM!!!\n");

	return strlen(buf);
}

static DEVICE_ATTR_RO(occur);

static ssize_t continue_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	u32 bm_index;
	char occ_info[96] = {0};

	for (bm_index = 0; bm_index < bm_dev.bm_ctn_dbg.loop_cnt; bm_index++) {
		if (bm_dev.bm_ctn_dbg.bm_ctn_info[bm_index].msk_addr != 0) {
			sprintf(occ_info, " %d\n addr: 0x%X\n CMD: 0x%X\n\
				msk_data_l: 0x%X\n"
				"msk_data_h: 0x%X\n id 0x%X\n",
				bm_dev.bm_ctn_dbg.bm_ctn_info[bm_index]
					.bm_index,
				bm_dev.bm_ctn_dbg.bm_ctn_info[bm_index]
					.msk_addr,
				bm_dev.bm_ctn_dbg.bm_ctn_info[bm_index]
					.msk_cmd,
				bm_dev.bm_ctn_dbg.bm_ctn_info[bm_index]
					.msk_data_l,
				bm_dev.bm_ctn_dbg.bm_ctn_info[bm_index]
					.msk_data_h,
				bm_dev.bm_ctn_dbg.bm_ctn_info[bm_index]
					.msk_id);
			strcat(buf, occ_info);
		}
	}
	if (occ_info[1] == 0)
		sprintf(buf, ":-) ! No action was monitored by BM!!!\n");

	return strlen(buf);
}

static ssize_t continue_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	u32 ctn_num, ret;

	ret = kstrtou32(buf, 0, &ctn_num);
	if (ret)
		return -EINVAL;
	if (ctn_num) {
		pr_info("Set BM support continue debug!!!\n");
		bm_dev.bm_ctn_dbg.bm_continue_dbg = true;
		if (ctn_num > BM_CONTINUE_DEBUG_SIZE)
			bm_dev.bm_ctn_dbg.loop_cnt = BM_CONTINUE_DEBUG_SIZE;
		else
			bm_dev.bm_ctn_dbg.loop_cnt = ctn_num;
	} else {
		pr_info("Set BM do not support continue debug!!!\n");
		bm_dev.bm_ctn_dbg.bm_continue_dbg = false;
		bm_dev.bm_ctn_dbg.loop_cnt = 0;
	}
	return count;
}

static DEVICE_ATTR_RW(continue);

static ssize_t panic_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	if (bm_dev.bm_st_info.bm_panic_st == true)
		return sprintf(buf, "The BM panic is open.\n");
	else
		return sprintf(buf, "The BM panic is close.\n");
}

static ssize_t panic_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	u32 panic_flg, ret;

	ret = kstrtou32(buf, 0, &panic_flg);
	if (ret)
		return -EINVAL;
	if (panic_flg) {
		pr_info("Reopen BM panic.\n");
		bm_dev.bm_st_info.bm_panic_st = true;
	} else {
		pr_info("Disable BM panic.\n");
		bm_dev.bm_st_info.bm_panic_st = false;
	}
	return count;
}

static DEVICE_ATTR_RW(panic);

static ssize_t stack_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	if (bm_dev.bm_st_info.bm_stack_st == true)
		return sprintf(buf, "The BM stack is open.\n");
	else
		return sprintf(buf, "The BM stack is close.\n");
}

static ssize_t stack_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	u32 panic_flg, ret;

	ret = kstrtou32(buf, 0, &panic_flg);
	if (ret)
		return -EINVAL;
	if (panic_flg) {
		pr_info("Reopen BM DFS.\n");
		bm_dev.bm_st_info.bm_stack_st = true;
	} else {
		pr_info("Disable BM DFS.\n");
		bm_dev.bm_st_info.bm_stack_st = false;
	}
	return count;
}

static DEVICE_ATTR_RW(stack);

static struct attribute *bm_attrs[] = {
	&dev_attr_state.attr,
	&dev_attr_chn.attr,
	&dev_attr_ahb_dbg.attr,
	&dev_attr_occur.attr,
	&dev_attr_continue.attr,
	&dev_attr_panic.attr,
	&dev_attr_stack.attr,
	NULL,
};

ATTRIBUTE_GROUPS(bm);

static void sprd_bm_init_name_info(const struct of_device_id *lock_of_id)
{
	u32 n;

	for (n = 0; n < MEMLAYOUT_COUNT; n++) {
		if (!strncmp(lock_of_id->compatible, board_name_gat[n], 11)) {
			bm_dev.bm_chn_name = chn_name_gat[n];
			break;
		}
	}
}

static void sprd_bm_get_hw_info(const struct of_device_id *lock_of_id)
{
	struct bm_status *this_status = NULL;
	u32 n;

	for (n = 0; n < MEMLAYOUT_COUNT; n++) {
		if (!strncmp(lock_of_id->compatible, board_name_gat[n], 11)) {
			this_status = bmstatus_gat[n];
			break;
		}
	}
	if (n == MEMLAYOUT_COUNT)
		panic("busmonitor has no match hw_info");
	bm_dev.bm_st_info.bm_def_mode = this_status->bm_status;
	pr_info("Bus Monitor mode: 0 - Pref; 1 - Debug. Current mode %d\n",
			bm_dev.bm_st_info.bm_def_mode);
	bm_dev.bm_st_info.axi_bm_cnt = this_status->bm_count0;
	bm_dev.bm_st_info.ahb_bm_chn = this_status->bm_count1;
	sprd_bm_init_name_info(lock_of_id);
}

static int sprd_bm_ahb_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	u32 bm_chn, bm_mode;
	void __iomem *reg_addr;

	if (true == bm_dev.bm_st_info.bm_dbg_st) {
		for (bm_chn = AHB_BM0; bm_chn < BM_SIZE; bm_chn++) {
			reg_addr = sprd_get_bm_base(bm_chn);
			bm_store_vale[bm_chn].str_addr
				= readl_relaxed(reg_addr
				+ AXI_BM_ADDR_MIN_REG)
				+ (0x80000000 & bm_store_vale[bm_chn].bm_mask);
			bm_store_vale[bm_chn].end_addr
				= readl_relaxed(reg_addr
				+ AXI_BM_ADDR_MAX_REG)
				+ (0x80000000 & bm_store_vale[bm_chn].bm_mask);
			bm_mode = readl_relaxed(reg_addr
				+ AXI_BM_CFG_REG) & 0x3;
			if (0 == bm_mode)
				bm_store_vale[bm_chn].mode = RW_MODE;
			else if (1 == bm_mode)
				bm_store_vale[bm_chn].mode = R_MODE;
			else if (3 == bm_mode)
				bm_store_vale[bm_chn].mode = W_MODE;
			if (bm_chn >= AHB_BM0) {
				bm_store_vale[bm_chn].min_data
					= readl_relaxed(reg_addr
					+ AXI_BM_DATA_MIN_L_REG);
				bm_store_vale[bm_chn].max_data
					= readl_relaxed(reg_addr
					+ AXI_BM_DATA_MAX_L_REG);
				regmap_read(bm_dev.ap_ahb_gpr,
					REG_AP_AHB_MISC_CFG, &bm_mode);
				switch (bm_chn) {
				case AHB_BM0:
					bm_store_vale[bm_chn].chn_sel
						= (bm_mode >> 4) & 0x3;
					break;
				case AHB_BM1:
					bm_store_vale[bm_chn].chn_sel
						= (bm_mode >> 8) & 0x3;
					break;
				case AHB_BM2:
					bm_store_vale[bm_chn].chn_sel
						= (bm_mode >> 10) & 0x3;
					break;
				default:
					break;
				}
			}
		}
	}

	return 0;
}

static int sprd_bm_ahb_resume(struct platform_device *pdev)
{
	u32 bm_chn, ret;
	struct sci_bm_cfg bm_cfg;

	sprd_ahb_bm_init();
	sprd_ahb_bm_int_clr();
	if (true == bm_dev.bm_st_info.bm_dbg_st) {
		for (bm_chn = AHB_BM0; bm_chn < BM_SIZE; bm_chn++) {
			if (((0x0 == bm_store_vale[bm_chn].str_addr)
				&& (0x0 == bm_store_vale[bm_chn]
				.end_addr)) || !strcmp(bm_dev.
				bm_chn_name[bm_chn], "ERROR_NAME"))
				continue;
			bm_cfg.addr_min = bm_store_vale[bm_chn].str_addr;
			bm_cfg.addr_max = bm_store_vale[bm_chn].end_addr;
			bm_cfg.bm_mode = bm_store_vale[bm_chn].mode;
			if (bm_chn >= AHB_BM0) {
				bm_cfg.data_min_l = bm_store_vale[bm_chn]
							.min_data;
				bm_cfg.data_min_h = 0;
				bm_cfg.data_max_l = bm_store_vale[bm_chn]
							.max_data;
				bm_cfg.data_max_h = 0;
			}
			ret = sprd_bm_set_point(bm_chn,
				bm_store_vale[bm_chn].chn_sel, &bm_cfg, NULL,
					NULL);
			if (SPRD_BM_SUCCESS != ret)
				return ret;
		}
	}
	return 0;
}

static int sprd_bm_ahb_probe(struct platform_device *pdev)
{
	int ret, bm_index;
	u32 ahb_irq1, ahb_irq2, ahb_irq3;
	struct resource *res = NULL;
	const struct of_device_id *lock_of_id = NULL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "sprd_bm get io resource failed!\n");
		return -ENODEV;
	}
	bm_dev.ahb_bm_base = devm_ioremap_nocache(&pdev->dev,
		res->start, resource_size(res));
	if (!bm_dev.ahb_bm_base) {
		dev_err(&pdev->dev, "sprd_bm get base address failed!\n");
		return -ENODEV;
	}
	bm_dev.ap_ahb_gpr =  syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						"busmonitor,syscon-enable");
	if (IS_ERR(bm_dev.ap_ahb_gpr)) {
			dev_err(&pdev->dev,
				"%s:failed to find ap-ahb\n", __func__);
	}
	ahb_irq1 = platform_get_irq(pdev, 0);
	if (ahb_irq1 < 0) {
		pr_err("Error: Can't get the ahb irq number1!\n");
		return -EBUSY;
	}
	ret = devm_request_irq(&pdev->dev, ahb_irq1, sprd_bm_isr,
		IRQF_TRIGGER_NONE, pdev->name, pdev);
	if (ret)
		return ret;
	ahb_irq2 = platform_get_irq(pdev, 1);
	if (ahb_irq2 < 0) {
		pr_err("Error: Can't get the ahb irq number2!\n");
		return -EBUSY;
	}
	ret = devm_request_irq(&pdev->dev, ahb_irq2, sprd_bm_isr,
		IRQF_TRIGGER_NONE, pdev->name, pdev);
	if (ret)
		return ret;
	ahb_irq3 = platform_get_irq(pdev, 2);
	if (ahb_irq3 < 0) {
		pr_err("Error: Can't get the ahb irq number3!\n");
		return -EBUSY;
	}
	ret = devm_request_irq(&pdev->dev, ahb_irq3, sprd_bm_isr,
		IRQF_TRIGGER_NONE, pdev->name, pdev);
	if (ret)
		return ret;
	lock_of_id = of_match_node(bm_ahb_of_match, pdev->dev.of_node);
	sprd_bm_get_hw_info(lock_of_id);
	for (bm_index = AHB_BM0; bm_index < BM_SIZE; bm_index++)
		sprd_bm_glb_reset_and_enable(bm_index, true);
	sprd_ahb_bm_int_clr();
	sprd_ahb_bm_init();
	dev_set_name(&bm_dev.dev, "busmonitor");
	bm_dev.dev.parent = &pdev->dev;
	bm_dev.dev.groups = bm_groups;
	dev_set_drvdata(&bm_dev.dev, &bm_dev);
	ret = device_register(&bm_dev.dev);
	if (ret) {
		pr_err("Unable to device_register\n");
		return ret;
	}
	if (bm_dev.bm_st_info.bm_def_mode > 0) {
		sprd_bm_def_val_set();
		bm_dev.bm_st_info.bm_dbg_st = true;
		bm_dev.bm_st_info.bm_stack_st = true;
		bm_dev.bm_st_info.bm_panic_st = true;
		bm_dev.bm_st_info.bm_dfs_off_st = true;
		pr_info("Bus Monitor default config set success!!!\n");
	}
	pr_info("sprd_ahb_bm_probe done!\n");
	return SPRD_BM_SUCCESS;
}

static int sprd_bm_ahb_remove(struct platform_device *pdev)
{
	u32 bm_index;

	for (bm_index = AHB_BM0; bm_index < BM_SIZE; bm_index++)
		sprd_bm_glb_reset_and_enable(bm_index, false);
	device_unregister(&pdev->dev);
	return SPRD_BM_SUCCESS;
}


static int sprd_bm_open(struct inode *inode, struct file *filp)
{
	pr_info("%s!\n", __func__);
	return 0;
}

static int sprd_bm_release(struct inode *inode, struct file *filp)
{
	pr_info("%s!\n", __func__);
	return 0;
}

static long sprd_bm_ioctl(struct file *filp, unsigned int cmd,
	unsigned long args)
{
	static struct timespec64 now;
	void __iomem *bm_reg;
	unsigned long bm_user;
	unsigned char bm_name[18] = {0};
	u32 i, ret, bm_index = 0;

	if (cmd >= (BM_CHANNELS << BM_CHN_NAME_CMD_OFFSET)) {
		if (BM_CHANNELS == (cmd >> BM_CHN_NAME_CMD_OFFSET)) {
			bm_index = cmd & 0xf;
			cmd = cmd >> BM_CHN_NAME_CMD_OFFSET;
		}
	}
	if (_IOC_TYPE(cmd) >= BM_CMD_MAX)
		return -EINVAL;

	switch (cmd) {
	case BM_STATE:
		if (bm_dev.bm_st_info.bm_dbg_st)
			bm_user = 1;
		if (put_user(bm_user, (unsigned long __user *)args))
			return -EFAULT;
		break;
	case BM_CHANNELS:
		for (i = 0; i < strlen(bm_dev.bm_chn_name[bm_index]); i++)
			bm_name[i] = bm_dev.bm_chn_name[bm_index][i];
		bm_name[i] = '\0';
		if (copy_to_user((unsigned char __user *)args,
				&bm_name, strlen(bm_name)))
				return -EFAULT;
		break;
	case BM_OCCUR:/* read dbg info */
		if (copy_to_user((struct bm_debug_info __user *)args,
			bm_dev.debug_bm_int_info, sizeof(struct bm_debug_info)))
			return -EFAULT;
		break;
	case BM_CONTINUE_SET:/* set continue statue */
		if (copy_from_user(&bm_dev.bm_ctn_dbg,
			(struct bm_continue_debug __user *)args,
			sizeof(struct bm_continue_debug)))
			return -EFAULT;
		break;
	case BM_CONTINUE_UNSET:
		bm_dev.bm_ctn_dbg.bm_continue_dbg = false;
		break;
	case BM_PANIC_SET:/* set DFS statue */
		bm_dev.bm_st_info.bm_panic_st = true;
		break;
	case BM_PANIC_UNSET:
		bm_dev.bm_st_info.bm_panic_st = false;
		break;
	case BM_CHN_CNT:
		if (put_user(bm_dev.bm_st_info.axi_bm_cnt,
			(unsigned long __user *)args))
			return -EFAULT;
		break;
	case BM_RD_CNT:
		ret = copy_from_user(&bm_user,
			(unsigned long __user *)args, sizeof(unsigned long));
		if (bm_user > AHB_BM2 || bm_user < AHB_BM0)
			return -EFAULT;
		bm_reg = sprd_get_bm_base(bm_user);
		bm_user = readl_relaxed(bm_reg + AXI_BM_RTRANS_IN_WIN_REG);
		if (copy_to_user((unsigned long __user *)args,
			&bm_user, sizeof(unsigned long)))
			return -EFAULT;
		break;
	case BM_WR_CNT:
		ret = copy_from_user(&bm_user,
			(unsigned long __user *)args, sizeof(unsigned long));
		if (bm_user > AHB_BM2 || bm_user < AHB_BM0)
			return -EFAULT;
		bm_reg = sprd_get_bm_base(bm_user);
		bm_user = readl_relaxed(bm_reg + AXI_BM_WTRANS_IN_WIN_REG);
		if (copy_to_user((unsigned long __user *)args,
			&bm_user, sizeof(unsigned long)))
			return -EFAULT;
		break;
	case BM_RD_BW:
		ret = copy_from_user(&bm_user,
			(unsigned long __user *)args, sizeof(unsigned long));
		if (bm_user > AHB_BM2 || bm_user < AHB_BM0)
			return -EFAULT;
		bm_reg = sprd_get_bm_base(bm_user);
		bm_user = readl_relaxed(bm_reg + AXI_BM_RBW_IN_WIN_REG);
		if (copy_to_user((unsigned long __user *)args,
			&bm_user, sizeof(unsigned long)))
			return -EFAULT;
		break;
	case BM_WR_BW:
		ret = copy_from_user(&bm_user, (unsigned long __user *)args,
			sizeof(unsigned long));
		if (bm_user > AHB_BM2 || bm_user < AHB_BM0)
			return -EFAULT;
		bm_reg = sprd_get_bm_base(bm_user);
		bm_user = readl_relaxed(bm_reg + AXI_BM_WBW_IN_WIN_REG);
		if (copy_to_user((unsigned long __user *)args, &bm_user,
			sizeof(unsigned long)))
			return -EFAULT;
		break;
	case BM_RD_LATENCY:
		ret = copy_from_user(&bm_user, (unsigned long __user *)args,
			sizeof(unsigned long));
		if (bm_user > AHB_BM2 || bm_user < AHB_BM0)
			return -EFAULT;
		bm_reg = sprd_get_bm_base(bm_user);
		bm_user = readl_relaxed(bm_reg + AXI_BM_RLATENCY_IN_WIN_REG);
		if (copy_to_user((unsigned long __user *)args, &bm_user,
			sizeof(unsigned long)))
			return -EFAULT;
		break;
	case BM_WR_LATENCY:
		ret = copy_from_user(&bm_user, (unsigned long __user *)args,
			sizeof(unsigned long));
		if (bm_user > AHB_BM2 || bm_user < AHB_BM0)
			return -EFAULT;
		bm_reg = sprd_get_bm_base(bm_user);
		bm_user = readl_relaxed(bm_reg + AXI_BM_WLATENCY_IN_WIN_REG);
		if (copy_to_user((unsigned long __user *)args, &bm_user,
			sizeof(unsigned long)))
			return -EFAULT;
		break;
	case BM_KERNEL_TIME:
		getnstimeofday64(&now);
		if (copy_to_user((u64 __user *)args,
			&now.tv_sec, sizeof(u64)))
			return -EFAULT;
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static const struct file_operations bm_fops = {
	.owner = THIS_MODULE,
	.open = sprd_bm_open,
	.release = sprd_bm_release,
	.unlocked_ioctl = sprd_bm_ioctl,
};

static struct miscdevice bm_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sprd_ahb_bm",
	.fops = &bm_fops,
};

static const struct of_device_id bm_ahb_of_match[] = {
	{ .compatible = "sprd,sc9838-ap-busmon", },
	{ .compatible = "sprd,sc9830-ap-busmon", },
	{ }
};

static struct platform_driver sprd_bm_ahb_driver = {
	.probe    = sprd_bm_ahb_probe,
	.remove   = sprd_bm_ahb_remove,
	.suspend  = sprd_bm_ahb_suspend,
	.resume   = sprd_bm_ahb_resume,
	.driver = {
		.owner = THIS_MODULE,
		.name = "sprd_ahb_bm",
		.of_match_table = bm_ahb_of_match,
	},
};

static int __init sprd_busmonitor_init(void)
{
	misc_register(&bm_misc);
	return platform_driver_register(&sprd_bm_ahb_driver);
}

static void __exit sprd_busmonitor_exit(void)
{
	platform_driver_unregister(&sprd_bm_ahb_driver);
	misc_deregister(&bm_misc);
}

module_init(sprd_busmonitor_init);
module_exit(sprd_busmonitor_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aiden Cheng<aiden.cheng@spreadtrum.com>");
MODULE_DESCRIPTION("spreadtrum platform busmonitor driver");




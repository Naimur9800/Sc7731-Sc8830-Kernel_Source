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
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/hwspinlock.h>
#include <linux/sprd_hwspinlock.h>
#include <linux/regmap.h>
#include <linux/sipc.h>
#include <linux/compat.h>
#include "sprd_bm_djtag.h"

#define TEMP_BUF			(100)
#define TEMP_LITTLE_BUF			(30)

static const struct of_device_id sprd_djtag_match[];
static void sprd_djtag_bm_def_set(struct sprd_djtag_dev *sdev);
static struct bm_debug_info g_bm_info;
static void sprd_djtag_delay(void)
{
	udelay(100);
}

static int  sprd_djtag_init(struct sprd_djtag_dev *sdev)
{
	int ret;
	struct sprd_djtag_reg *djtag_reg = sdev->djtag_reg_base;

	disable_irq(sdev->djtag_irq);
	ret = hwspin_lock_timeout(sdev->hw_lock, 5000);
	if (ret) {
		pr_emerg("lock djtag hw spinlock failed.\n");
		return ret;
	}
	regmap_update_bits(sdev->aon_apb, REG_AON_APB_APB_EB1,
		BIT_AON_APB_DJTAG_EB, BIT_AON_APB_DJTAG_EB);
	if (sdev->pdata->chip == SHARKL2_CHIP ||
		sdev->pdata->chip == SHARKLJ1_CHIP) {
		regmap_update_bits(sdev->aon_apb, REG_AON_APB_APB_EB1,
		BIT_DJTAG_REG_ACCESS_EB, BIT_DJTAG_REG_ACCESS_EB);
	}
	writel_relaxed(DJTAG_IR_LEN, (void __iomem *)&djtag_reg->ir_len_cfg);
	writel_relaxed(DJTAG_DR_LEN, (void __iomem *)&djtag_reg->dr_len_cfg);
	if (sdev->pdata->chip == SHARKL2_CHIP ||
		sdev->pdata->chip == SHARKLJ1_CHIP) {
		regmap_update_bits(sdev->aon_apb, REG_AON_APB_APB_RST2,
		BIT_AON_APB_DJTAG_SOFT_RST, BIT_AON_APB_DJTAG_SOFT_RST);
		sprd_djtag_delay();
		regmap_update_bits(sdev->aon_apb, REG_AON_APB_APB_RST2,
		BIT_AON_APB_DJTAG_SOFT_RST, 0);
	} else {
		writel_relaxed(0x1, (void __iomem *)
			&djtag_reg->dap_mux_ctrl_rst);
		sprd_djtag_delay();
		writel_relaxed(0x0, (void __iomem *)
			&djtag_reg->dap_mux_ctrl_rst);
	}
	return 0;
}

static void sprd_djtag_mux_sel(struct sprd_djtag_dev *sdev,
	u32 mux, u32 dap)
{
	struct sprd_djtag_reg *djtag_reg = sdev->djtag_reg_base;
	u32 mux_value = 0;

	mux_value = mux << DJTAG_DAP_OFFSET;
	mux_value |= dap;
	writel_relaxed(DJTAG_DAP_MUX_RESET, (void __iomem *)&djtag_reg->ir_cfg);
	writel_relaxed(mux_value, (void __iomem *)&djtag_reg->dr_cfg);
	writel_relaxed(0x1, (void __iomem *)&djtag_reg->rnd_en_cfg);
	sprd_djtag_delay();
	writel_relaxed(0x0, (void __iomem *)&djtag_reg->rnd_en_cfg);
}

static void sprd_djtag_deinit(struct sprd_djtag_dev *sdev)
{
	struct sprd_djtag_reg *djtag_reg = sdev->djtag_reg_base;

	writel_relaxed(0x1, (void __iomem *)&djtag_reg->dap_mux_ctrl_rst);
	sprd_djtag_delay();
	writel_relaxed(0x0, (void __iomem *)&djtag_reg->dap_mux_ctrl_rst);
	if (sdev->pdata->chip == SHARKL2_CHIP) {
		regmap_update_bits(sdev->aon_apb, REG_AON_APB_APB_EB1,
		BIT_DJTAG_REG_ACCESS_EB, 0);
	}
	regmap_update_bits(sdev->aon_apb, REG_AON_APB_APB_EB1,
		BIT_AON_APB_DJTAG_EB, 0);
	hwspin_unlock(sdev->hw_lock);
	enable_irq(sdev->djtag_irq);
}

static u32 sprd_djtag_busmon_write(struct sprd_djtag_dev *sdev,
	u32 ir, u32 dr)
{
	struct sprd_djtag_reg *djtag_reg = sdev->djtag_reg_base;
	u32 ir_write = ir | BIT_DJTAG_CHAIN_UPDATE_T;

	writel_relaxed(ir_write, (void __iomem *)&djtag_reg->ir_cfg);
	writel_relaxed(dr, (void __iomem *)&djtag_reg->dr_cfg);
	writel_relaxed(0x1, (void __iomem *)&djtag_reg->rnd_en_cfg);
	sprd_djtag_delay();
	writel_relaxed(0x0, (void __iomem *)&djtag_reg->rnd_en_cfg);

	return readl_relaxed((void __iomem *)&djtag_reg->upd_dr_cfg);
}

static u32 sprd_djtag_busmon_read(struct sprd_djtag_dev *sdev, u32 ir)
{
	struct sprd_djtag_reg *djtag_reg = sdev->djtag_reg_base;
	u32 dr_value = 0;

	writel_relaxed(ir, (void __iomem *)&djtag_reg->ir_cfg);
	writel_relaxed(dr_value, (void __iomem *)&djtag_reg->dr_cfg);
	writel_relaxed(0x1, (void __iomem *)&djtag_reg->rnd_en_cfg);
	sprd_djtag_delay();
	writel_relaxed(0x0, (void __iomem *)&djtag_reg->rnd_en_cfg);
	return readl_relaxed((void __iomem *)&djtag_reg->upd_dr_cfg);
}

static void sprd_djtag_auto_scan_set(struct sprd_djtag_dev *sdev,
	u32 chain)
{
	struct sprd_djtag_reg *djtag_reg = sdev->djtag_reg_base;
	struct djtag_autoscan_set *autoscan_set =
		&sdev->autoscan_setting[chain];

	if (autoscan_set->scan) {
		u32 *addr = &djtag_reg->autoscan_chain_addr[chain];
		u32 *pattern = &djtag_reg->autoscan_chain_pattern[chain];
		u32 *mask = &djtag_reg->autoscan_chain_mask[chain];

		writel_relaxed(autoscan_set->autoscan_chain_addr,
			(void __iomem *)addr);
		writel_relaxed(autoscan_set->autoscan_chain_pattern,
			(void __iomem *)pattern);
		writel_relaxed(~(autoscan_set->autoscan_chain_mask),
			(void __iomem *)mask);
		writel_relaxed(1, (void __iomem *)&djtag_reg->autoscan_int_clr);
		writel_relaxed((1 << chain) |
			readl_relaxed((void __iomem *)&djtag_reg->autoscan_en),
			(void __iomem *)&djtag_reg->autoscan_en);
		writel_relaxed((1 << chain) |
			readl_relaxed((void __iomem *)
			&djtag_reg->autoscan_int_en),
			(void __iomem *)&djtag_reg->autoscan_int_en);
	}
}

static void sprd_djtag_auto_scan_en(struct sprd_djtag_dev *sdev)
{
	struct sprd_djtag_reg *djtag_reg = sdev->djtag_reg_base;

	writel_relaxed(DJTAG_AUTO_SCAN_IR,
		(void __iomem *)&djtag_reg->ir_cfg);
	writel_relaxed(0x0, (void __iomem *)&djtag_reg->dr_cfg);
	writel_relaxed(0x1, (void __iomem *)&djtag_reg->rnd_en_cfg);
	sprd_djtag_delay();
	writel_relaxed(0x0, (void __iomem *)&djtag_reg->rnd_en_cfg);
}

static void sprd_djtag_auto_scan_dis(struct sprd_djtag_dev *sdev)
{
	struct sprd_djtag_reg *djtag_reg = sdev->djtag_reg_base;

	writel_relaxed(0x0,
		(void __iomem *)&djtag_reg->ir_cfg);
	writel_relaxed(0x0, (void __iomem *)&djtag_reg->dr_cfg);
	writel_relaxed(0x1, (void __iomem *)&djtag_reg->rnd_en_cfg);
	sprd_djtag_delay();
	writel_relaxed(0x0, (void __iomem *)&djtag_reg->rnd_en_cfg);
}


static u32 sprd_djtag_autoscan_int_sts(struct sprd_djtag_dev *sdev)
{
	struct sprd_djtag_reg *djtag_reg = sdev->djtag_reg_base;
	u32 int_status = 0;

	int_status = readl_relaxed((void __iomem *)
		&djtag_reg->autoscan_int_raw);

	return int_status;
}

static void sprd_djtag_autoscan_clr_int(struct sprd_djtag_dev *sdev,
	int index)
{
	struct sprd_djtag_reg *djtag_reg = sdev->djtag_reg_base;

	writel_relaxed(0x0, (void __iomem *)&djtag_reg->autoscan_en);
	writel_relaxed(0x0, (void __iomem *)&djtag_reg->autoscan_int_en);
	writel_relaxed((1 << index) |
		readl_relaxed((void __iomem *)&djtag_reg->autoscan_int_clr),
		(void __iomem *)&djtag_reg->autoscan_int_clr);
	sprd_djtag_delay();
	writel_relaxed(~(1 << index) &
		readl_relaxed((void __iomem *)
		&djtag_reg->autoscan_int_clr),
		(void __iomem *)&djtag_reg->autoscan_int_clr);
}

static irqreturn_t sprd_djtag_isr(int irq_num, void *dev)
{
	struct sprd_djtag_dev *sdev = dev;
	struct sprd_djtag_reg *djtag_reg = sdev->djtag_reg_base;
	u32 int_status, i = 0;

	if (sprd_djtag_init(sdev))
		return IRQ_HANDLED;
	int_status = sprd_djtag_autoscan_int_sts(sdev);
	if (int_status) {
		pr_info("\r\n ***djtag interrupt status: 0x%08x ****\r\n",
			int_status);
	}
	while (int_status && (i < 16)) {
		if (int_status & 1) {
			sdev->autoscan_info[i].autoscan_chain_addr =
				readl_relaxed((void __iomem *)
				&djtag_reg->autoscan_chain_addr[i]);
			sdev->autoscan_info[i].autoscan_chain_pattern =
				readl_relaxed((void __iomem *)
				&djtag_reg->autoscan_chain_pattern[i]);
			sdev->autoscan_info[i].autoscan_chain_data =
				readl_relaxed((void __iomem *)
				&djtag_reg->autoscan_chain_data[i]);
			sdev->autoscan_info[i].autoscan_chain_mask =
				readl_relaxed((void __iomem *)
				&djtag_reg->autoscan_chain_mask[i]);
			sdev->autoscan_info[i].occurred = true;
			sprd_djtag_autoscan_clr_int(sdev, i);
		}
		i++;
		int_status >>= 1;
	}
	pr_info("\r\n ***interrupt status after clear: 0x%08x ****\r\n",
		sprd_djtag_autoscan_int_sts(sdev));
	sprd_djtag_auto_scan_dis(sdev);
	sprd_djtag_deinit(sdev);

	return IRQ_HANDLED;
}

static int sprd_djtag_enable(struct sprd_djtag_dev *sdev)
{
	return sprd_djtag_init(sdev);
}


static void sprd_djtag_scan_sel(struct sprd_djtag_dev *sdev,
	u32 mux, u32 dap)
{
	sprd_djtag_mux_sel(sdev, mux, dap);
}

static void sprd_djtag_disable(struct sprd_djtag_dev *sdev)
{
	sprd_djtag_deinit(sdev);
}

static u32 sprd_djtag_bm_write(struct sprd_djtag_dev *sdev,
	u32 ir, u32 dr)
{
	return sprd_djtag_busmon_write(sdev, ir, dr);
}

static u32 sprd_djtag_bm_read(struct sprd_djtag_dev *sdev, u32 ir)
{
	return sprd_djtag_busmon_read(sdev, ir);
}

static void sprd_djtag_bm_phy_open(struct sprd_djtag_dev *sdev,
	const u32 bm_index)
{
	if (sprd_djtag_enable(sdev))
		return;
	sprd_djtag_scan_sel(sdev, bm_def_monitor[bm_index].bm_arch,
		bm_def_monitor[bm_index].bm_dap);
	sprd_djtag_bm_write(sdev, AHB_CHN_INT,
		(BM_INT_CLR | BM_INT_EN | BM_CHN_EN));
	sprd_djtag_disable(sdev);
}

static void sprd_djtag_bm_phy_en(struct sprd_djtag_dev *sdev,
	const u32 bm_index)
{
	if (sprd_djtag_enable(sdev))
		return;
	sprd_djtag_scan_sel(sdev, bm_def_monitor[bm_index].bm_arch,
		bm_def_monitor[bm_index].bm_dap);
	sprd_djtag_bm_write(sdev, AHB_CHN_INT,
		BM_CHN_EN);
	sprd_djtag_disable(sdev);
}

static void sprd_djtag_bm_phy_close(struct sprd_djtag_dev *sdev,
	const u32 bm_index)
{
	u32 val = 0;

	/* Bus monitor disable and clear interrupt */
	if (sprd_djtag_enable(sdev))
		return;
	sprd_djtag_scan_sel(sdev, bm_def_monitor[bm_index].bm_arch,
		bm_def_monitor[bm_index].bm_dap);
	val &= ~BM_INT_EN;
	val |= BM_INT_CLR;
	sprd_djtag_bm_write(sdev, AHB_CHN_INT, val);
	sprd_djtag_disable(sdev);
}

static u32 sprd_djtag_get_bitmap(struct sprd_djtag_dev *sdev, u32 ir)
{
	struct sprd_djtag_reg *djtag_reg = sdev->djtag_reg_base;
	u32 value;

	djtag_reg->ir_cfg = ir;
	djtag_reg->dr_cfg = 0;
	djtag_reg->rnd_en_cfg = 0x1;
	sprd_djtag_delay();
	djtag_reg->rnd_en_cfg = 0x0;
	value = djtag_reg->upd_dr_cfg;

	return value;
}

static void sprd_djtag_bm_phy_reg_check
	(struct sprd_djtag_dev *sdev, const u32 bm_index)
{
	struct ahb_bm_reg ahb_reg;
	struct axi_bm_reg axi_reg;

	/* Bus monitor disable and clear interrupt */
	if (sprd_djtag_enable(sdev))
		return;
	sprd_djtag_scan_sel(sdev, bm_def_monitor[bm_index].bm_arch,
		bm_def_monitor[bm_index].bm_dap);
	if (bm_def_monitor[bm_index].bm_type == AHB_BM) {
		ahb_reg.ahb_bm_chn_int = sprd_djtag_bm_read(sdev, AHB_CHN_INT);
		ahb_reg.ahb_bm_match_set_reg.bm_chn_cfg =
			sprd_djtag_bm_read(sdev, AHB_CHN_CFG);
		ahb_reg.ahb_bm_match_set_reg.bm_addr_min =
			sprd_djtag_bm_read(sdev, AHB_ADDR_MIN);
		ahb_reg.ahb_bm_addr_minh =
			sprd_djtag_bm_read(sdev, AHB_ADDR_MIN_H32);
		ahb_reg.ahb_bm_match_set_reg.bm_addr_max =
			sprd_djtag_bm_read(sdev, AHB_ADDR_MAX);
		ahb_reg.ahb_bm_addr_maxh =
			sprd_djtag_bm_read(sdev, AHB_ADDR_MAX_H32);
		ahb_reg.ahb_bm_match_set_reg.bm_addr_mask =
			sprd_djtag_bm_read(sdev, AHB_ADDR_MASK);
		ahb_reg.ahb_bm_addr_maskh =
			sprd_djtag_bm_read(sdev, AHB_ADDR_MASK_H32);
		ahb_reg.ahb_bm_match_set_reg.bm_data_min_l32 =
			sprd_djtag_bm_read(sdev, AHB_DATA_MIN_L32);
		ahb_reg.ahb_bm_match_set_reg.bm_data_min_h32 =
			sprd_djtag_bm_read(sdev, AHB_DATA_MIN_H32);
		ahb_reg.ahb_bm_match_set_reg.bm_data_max_l32 =
			sprd_djtag_bm_read(sdev, AHB_DATA_MAX_L32);
		ahb_reg.ahb_bm_match_set_reg.bm_data_max_h32 =
			sprd_djtag_bm_read(sdev, AHB_DATA_MAX_H32);
		ahb_reg.ahb_bm_match_set_reg.bm_data_mask_l32 =
			sprd_djtag_bm_read(sdev, AHB_DATA_MASK_L32);
		ahb_reg.ahb_bm_match_set_reg.bm_data_mask_h32 =
			sprd_djtag_bm_read(sdev, AHB_DATA_MASK_H32);
		ahb_reg.ahb_bm_match_read_reg.bm_match_addr =
			sprd_djtag_bm_read(sdev, AHB_MATCH_ADDR);
		ahb_reg.ahb_bm_match_read_reg.bm_match_cmd =
			sprd_djtag_bm_read(sdev, AHB_MATCH_CMD);
		ahb_reg.ahb_bm_match_read_reg.bm_match_data_l32 =
			sprd_djtag_bm_read(sdev, AHB_MATCH_DATA_L32);
		ahb_reg.ahb_bm_match_read_reg.bm_match_data_h32 =
			sprd_djtag_bm_read(sdev, AHB_MATCH_DATA_H32);
		ahb_reg.ahb_bm_match_h = sprd_djtag_bm_read(sdev,
			AHB_MATCH_ADDR_H32);
		pr_cont("ahb bm index: %d\n", bm_index);
		pr_cont("int:0x%08x\ncfg:0x%08x\naddr_min:0x%08x\n",
			ahb_reg.ahb_bm_chn_int,
			ahb_reg.ahb_bm_match_set_reg.bm_chn_cfg,
			ahb_reg.ahb_bm_match_set_reg.bm_addr_min);
		pr_cont("add_max:0x%08x\nmatch_addr:0x%08x\n",
		ahb_reg.ahb_bm_match_set_reg.bm_addr_max,
		ahb_reg.ahb_bm_match_read_reg.bm_match_addr);
		pr_cont("match_data:0x%08x%08x\nmatch_cmd:0x%08x\n",
			ahb_reg.ahb_bm_match_read_reg.bm_match_data_h32,
			ahb_reg.ahb_bm_match_read_reg.bm_match_data_l32,
			ahb_reg.ahb_bm_match_read_reg.bm_match_cmd);
	} else {
		axi_reg.axi_bm_chn_int = sprd_djtag_bm_read(sdev, AXI_CHN_INT);
		axi_reg.axi_bm_match_set_reg.bm_chn_cfg =
			sprd_djtag_bm_read(sdev, AXI_CHN_CFG);
		axi_reg.axi_bm_match_set_reg.bm_id_cfg =
			sprd_djtag_bm_read(sdev, AXI_ID_CFG);
		axi_reg.axi_bm_match_set_reg.bm_addr_min_l32 =
			sprd_djtag_bm_read(sdev, AXI_ADDR_MIN);
		axi_reg.axi_bm_match_set_reg.bm_addr_min_h32 =
			sprd_djtag_bm_read(sdev, AXI_ADDR_MIN_H32);
		axi_reg.axi_bm_match_set_reg.bm_addr_max_l32 =
			sprd_djtag_bm_read(sdev, AXI_ADDR_MAX);
		axi_reg.axi_bm_match_set_reg.bm_addr_max_h32 =
			sprd_djtag_bm_read(sdev, AXI_ADDR_MAX_H32);
		axi_reg.axi_bm_match_set_reg.bm_addr_mask_l32 =
			sprd_djtag_bm_read(sdev, AXI_ADDR_MASK);
		axi_reg.axi_bm_match_set_reg.bm_addr_mask_h32 =
			sprd_djtag_bm_read(sdev, AXI_ADDR_MASK_H32);
		axi_reg.axi_bm_match_set_reg.bm_data_min_l32 =
			sprd_djtag_bm_read(sdev, AXI_DATA_MIN_L32);
		axi_reg.axi_bm_match_set_reg.bm_data_min_h32 =
			sprd_djtag_bm_read(sdev, AXI_DATA_MIN_H32);
		axi_reg.axi_bm_match_set_reg.bm_data_min_ext_l32 =
			sprd_djtag_bm_read(sdev, AXI_DATA_MIN_EXT_L32);
		axi_reg.axi_bm_match_set_reg.bm_data_min_ext_h32 =
			sprd_djtag_bm_read(sdev, AXI_DATA_MIN_EXT_H32);
		axi_reg.axi_bm_match_set_reg.bm_data_max_l32 =
			sprd_djtag_bm_read(sdev, AXI_DATA_MAX_L32);
		axi_reg.axi_bm_match_set_reg.bm_data_max_h32 =
			sprd_djtag_bm_read(sdev, AXI_DATA_MAX_H32);
		axi_reg.axi_bm_match_set_reg.bm_data_max_ext_l32 =
			sprd_djtag_bm_read(sdev, AXI_DATA_MAX_EXT_L32);
		axi_reg.axi_bm_match_set_reg.bm_data_max_ext_h32 =
			sprd_djtag_bm_read(sdev, AXI_DATA_MAX_EXT_H32);
		axi_reg.axi_bm_match_set_reg.bm_data_mask_l32 =
			sprd_djtag_bm_read(sdev, AXI_DATA_MASK_L32);
		axi_reg.axi_bm_match_set_reg.bm_data_mask_h32 =
			sprd_djtag_bm_read(sdev, AXI_DATA_MASK_H32);
		axi_reg.axi_bm_match_set_reg.bm_data_mask_ext_l32 =
			sprd_djtag_bm_read(sdev, AXI_DATA_MASK_EXT_L32);
		axi_reg.axi_bm_match_set_reg.bm_data_mask_ext_h32 =
			sprd_djtag_bm_read(sdev, AXI_DATA_MASK_EXT_H32);
		axi_reg.axi_bm_trans_len =
			sprd_djtag_bm_read(sdev, AXI_MON_TRANS_LEN);
		axi_reg.axi_bm_match_id =
			sprd_djtag_bm_read(sdev, AXI_MATCH_ID);
		axi_reg.axi_bm_match_addr_l32 =
			sprd_djtag_bm_read(sdev, AXI_MATCH_ADDR);
		axi_reg.axi_bm_match_addr_h32 =
			sprd_djtag_bm_read(sdev, AXI_MATCH_ADDR_H32);
		axi_reg.axi_bm_cmd = sprd_djtag_bm_read(sdev, AXI_MATCH_CMD);
		axi_reg.axi_bm_match_data_l32 =
			sprd_djtag_bm_read(sdev, AXI_MATCH_DATA_L32);
		axi_reg.axi_bm_match_data_h32 =
			sprd_djtag_bm_read(sdev, AXI_MATCH_DATA_H32);
		axi_reg.axi_bm_match_data_ext_l32 =
			sprd_djtag_bm_read(sdev, AXI_MATCH_DATA_EXT_L32);
		axi_reg.axi_bm_match_data_ext_h32 =
			sprd_djtag_bm_read(sdev, AXI_MATCH_DATA_EXT_H32);
		axi_reg.axi_bm_bus_status =
			sprd_djtag_bm_read(sdev, AXI_BUS_STATUS);
		pr_cont("axi bm index: %d\n", bm_index);
		pr_cont("int:0x%08x\ncfg:0x%08x\naddr_min:0x%08x%08x\n",
			axi_reg.axi_bm_chn_int,
			axi_reg.axi_bm_match_set_reg.bm_id_cfg,
			axi_reg.axi_bm_match_set_reg.bm_addr_min_h32,
			axi_reg.axi_bm_match_set_reg.bm_addr_min_l32);
		pr_cont("add_max:0x%08x%08x\nmatch_addr:0x%08x%08x\n",
			axi_reg.axi_bm_match_set_reg.bm_addr_max_h32,
			axi_reg.axi_bm_match_set_reg.bm_addr_max_l32,
			axi_reg.axi_bm_match_addr_h32,
			axi_reg.axi_bm_match_addr_l32);
		pr_cont("match_data:0x%08x%08x\nmatch_cmd:0x%08x\n",
			axi_reg.axi_bm_match_data_h32,
			axi_reg.axi_bm_match_data_l32,
			axi_reg.axi_bm_cmd);
		if (sdev->pdata->chip == IWHALE2_CHIP ||
			sdev->pdata->chip == SHARKL2_CHIP ||
			sdev->pdata->chip == ISHARKL2_CHIP ||
			sdev->pdata->chip == SHARKLJ1_CHIP) {
			axi_reg.axi_bm_match_userid =
			sprd_djtag_bm_read(sdev, AXI_MATCH_USERID);
			pr_cont("match_userid:0x%08x\n",
			axi_reg.axi_bm_match_userid);
		}
	}
	sprd_djtag_disable(sdev);
}

static void sprd_djtag_bm_phy_set(struct sprd_djtag_dev *sdev,
	const u32 bm_index, const struct bm_match_setting *bm_setting)
{
	if (sprd_djtag_enable(sdev))
		return;
	sprd_djtag_scan_sel(sdev, bm_def_monitor[bm_index].bm_arch,
		bm_def_monitor[bm_index].bm_dap);
	if (bm_setting->bm_type == AHB_BM) {
		sprd_djtag_bm_write(sdev, AHB_CHN_INT, BM_INT_CLR);
		if (bm_setting->rw_cfg == BM_WRITE)
			sprd_djtag_bm_write(sdev, AHB_CHN_CFG,
			(BM_WRITE_EN | BM_WRITE_CFG));
		else
			sprd_djtag_bm_write(sdev, AHB_CHN_CFG, BM_WRITE_EN);
		sprd_djtag_bm_write(sdev, AHB_ADDR_MIN,
			bm_setting->bm_addr_min_l32);
		sprd_djtag_bm_write(sdev, AHB_ADDR_MIN_H32,
			bm_setting->bm_addr_min_h32);
		sprd_djtag_bm_write(sdev, AHB_ADDR_MAX,
			bm_setting->bm_addr_max_l32);
		sprd_djtag_bm_write(sdev, AHB_ADDR_MAX_H32,
			bm_setting->bm_addr_max_h32);
		sprd_djtag_bm_write(sdev, AHB_ADDR_MASK,
			bm_setting->bm_addr_mask_l32);
		sprd_djtag_bm_write(sdev, AHB_ADDR_MASK_H32,
			bm_setting->bm_addr_mask_h32);
		sprd_djtag_bm_write(sdev, AHB_DATA_MIN_L32, 0x0);
		sprd_djtag_bm_write(sdev, AHB_DATA_MIN_H32, 0x0);
		sprd_djtag_bm_write(sdev, AHB_DATA_MAX_L32, 0xFFFFFFFF);
		sprd_djtag_bm_write(sdev, AHB_DATA_MAX_H32, 0xFFFFFFFF);
		sprd_djtag_bm_write(sdev, AHB_DATA_MASK_L32, 0x0);
		sprd_djtag_bm_write(sdev, AHB_DATA_MASK_H32, 0x0);
	} else {
		sprd_djtag_bm_write(sdev, AXI_CHN_INT, BM_INT_CLR);
		if (bm_setting->rw_cfg == BM_WRITE)
			sprd_djtag_bm_write(sdev, AXI_CHN_CFG,
			(BM_WRITE_EN | BM_WRITE_CFG));
		else
			sprd_djtag_bm_write(sdev, AXI_CHN_CFG, BM_WRITE_EN);
		sprd_djtag_bm_write(sdev, AXI_ADDR_MIN,
			bm_setting->bm_addr_min_l32);
		sprd_djtag_bm_write(sdev, AXI_ADDR_MIN_H32,
			bm_setting->bm_addr_min_h32);
		sprd_djtag_bm_write(sdev, AXI_ADDR_MAX,
			bm_setting->bm_addr_max_l32);
		sprd_djtag_bm_write(sdev, AXI_ADDR_MAX_H32,
			bm_setting->bm_addr_max_h32);
		sprd_djtag_bm_write(sdev, AXI_ADDR_MASK,
			bm_setting->bm_addr_mask_l32);
		sprd_djtag_bm_write(sdev, AXI_ADDR_MASK_H32,
			bm_setting->bm_addr_mask_h32);
		sprd_djtag_bm_write(sdev, AXI_DATA_MIN_L32, 0xFFFFFFFF);
		sprd_djtag_bm_write(sdev, AXI_DATA_MIN_H32, 0x0);
		sprd_djtag_bm_write(sdev, AXI_DATA_MIN_EXT_L32, 0xFFFFFFFF);
		sprd_djtag_bm_write(sdev, AXI_DATA_MIN_EXT_H32, 0x0);
		sprd_djtag_bm_write(sdev, AXI_DATA_MAX_L32, 0x0);
		sprd_djtag_bm_write(sdev, AXI_DATA_MAX_H32, 0x0);
		sprd_djtag_bm_write(sdev, AXI_DATA_MAX_EXT_L32, 0x0);
		sprd_djtag_bm_write(sdev, AXI_DATA_MAX_EXT_H32, 0x0);
		sprd_djtag_bm_write(sdev, AXI_DATA_MASK_L32, 0x0);
		sprd_djtag_bm_write(sdev, AXI_DATA_MASK_H32, 0x0);
		sprd_djtag_bm_write(sdev, AXI_DATA_MASK_EXT_L32, 0x0);
		sprd_djtag_bm_write(sdev, AXI_DATA_MASK_EXT_H32, 0x0);
		if (sdev->pdata->chip == IWHALE2_CHIP ||
			sdev->pdata->chip == SHARKL2_CHIP ||
			sdev->pdata->chip == ISHARKL2_CHIP ||
			sdev->pdata->chip == SHARKLJ1_CHIP) {
			if (bm_setting->bm_userid != 0xff)
				sprd_djtag_bm_write(sdev, AXI_USER_CFG,
					bm_setting->bm_userid |
					USER_CFG_ENUSERID);
			else
				sprd_djtag_bm_write(sdev, AXI_USER_CFG, 0);
		}
	}
	sprd_djtag_disable(sdev);
	sprd_djtag_bm_phy_open(sdev, bm_index);
}

static inline void sprd_djtag_bm_def_monitor_cfg
	(struct sprd_djtag_dev *sdev,
	unsigned long bm_index, unsigned long start,
	unsigned long end, unsigned int mode)
{
	/* whale ca53 ddr monitor after noc should subtracted 0x80000000 */
	if (sdev->pdata->chip == WHALE_CHIP) {
		if (bm_index == 15) {
			start &= ~(BIT(31));
			end &= ~(BIT(31));
		}
	}
	/* whale2 ca53 ddr monitor after noc should subtracted 0x80000000 */
	if (sdev->pdata->chip == WHALE2_CHIP) {
		if (bm_index == 12) {
			start &= ~(BIT(31));
			end &= ~(BIT(31));
		}
	}
	bm_def_monitor[bm_index].bm_addr_min_l32 = start & 0xffffffff;
	bm_def_monitor[bm_index].bm_addr_min_h32 = (u64)start >> 32;
	bm_def_monitor[bm_index].bm_addr_max_l32 = end & 0xffffffff;
	bm_def_monitor[bm_index].bm_addr_max_h32 = (u64)end >> 32;
	bm_def_monitor[bm_index].rw_cfg = mode;
}

static inline void sprd_djtag_bm_set_userid_cfg(struct sprd_djtag_dev *sdev,
	unsigned long bm_index, u32 userid)
{
	if (sdev->pdata->chip == IWHALE2_CHIP)
		iwhale2_userid_cfg[bm_index] = userid & USER_CFG_USERID_MASK;
	if (sdev->pdata->chip == SHARKL2_CHIP)
		sharkl2_userid_cfg[bm_index] = userid & USER_CFG_USERID_MASK;
	if (sdev->pdata->chip == ISHARKL2_CHIP)
		isharkl2_userid_cfg[bm_index] = userid & USER_CFG_USERID_MASK;
	if (sdev->pdata->chip == SHARKLJ1_CHIP)
		sharklj1_userid_cfg[bm_index] = userid & USER_CFG_USERID_MASK;
}

static void sprd_djtag_bm_set_point(struct sprd_djtag_dev *sdev,
	const u32 bm_index)
{
	struct bm_match_setting setting = {};
	struct smsg msend;

	if (sdev->pdata->chip == IWHALE2_CHIP) {
		if (bm_def_monitor[bm_index].bm_arch == MUX_AP_BM) {
			regmap_update_bits(sdev->ap_ahb, REG_AP_AHB_AHB_EB,
			BIT_AP_AHB_BUSMON_EB, BIT_AP_AHB_BUSMON_EB);
		}
	}
	if (sdev->pdata->chip == WHALE2_CHIP) {
		if (bm_def_monitor[bm_index].bm_arch == MUX_AP_BM) {
			regmap_update_bits(sdev->ap_ahb, REG_AP_AHB_AHB_EB,
			BIT_AP_AHB_BUSMON_EB, BIT_AP_AHB_BUSMON_EB);
		}
		if (bm_def_monitor[bm_index].bm_arch == MUX_CAM_BM) {
			u32 regval = BIT_CAM_AHB_BUSMON_M8_EB |
			BIT_CAM_AHB_BUSMON_M7_EB |
			BIT_CAM_AHB_BUSMON_M6_EB |
			BIT_CAM_AHB_BUSMON_M5_EB |
			BIT_CAM_AHB_BUSMON_M4_EB |
			BIT_CAM_AHB_BUSMON_M3_EB |
			BIT_CAM_AHB_BUSMON_M2_EB |
			BIT_CAM_AHB_BUSMON_M1_EB |
			BIT_CAM_AHB_BUSMON_M0_EB;
			regmap_update_bits(sdev->cam_ahb,
				REG_CAM_AHB_BUSMON_CK_EN, regval, regval);
		}
		if (bm_def_monitor[bm_index].bm_arch == MUX_DISP_BM) {
			u32 regval = BIT_DISP_AHB_GPU_GSP_BUSMON_EN |
				BIT_DISP_AHB_GSP1_BUSMON_EN |
				BIT_DISP_AHB_GSP0_BUSMON_EN |
				BIT_DISP_AHB_TMC_BUSMON_EN |
				BIT_DISP_AHB_DISPC1_BUSMON_EN |
				BIT_DISP_AHB_DISPC0_BUSMON_EN;
			regmap_update_bits(sdev->disp_ahb,
				REG_DISP_AHB_GEN_BUSMON_CFG,
			regval, regval);
		}
		if (bm_def_monitor[bm_index].bm_arch == MUX_VSP_BM) {
			u32 regval = BIT_VSP_AHB_VPP_BUSMON_EN |
				BIT_VSP_AHB_VSP_ENC_BUSMON_EN |
				BIT_VSP_AHB_VSP_BUSMON_EN;
			regmap_update_bits(sdev->vsp_ahb,
				REG_VSP_AHB_GEN_CKG_CFG, regval, regval);
		}
	}
	setting.bm_type = bm_def_monitor[bm_index].bm_type;
	setting.bm_dap = bm_def_monitor[bm_index].bm_dap;
	setting.rw_cfg = bm_def_monitor[bm_index].rw_cfg;
	setting.bm_addr_min_l32 = bm_def_monitor[bm_index].bm_addr_min_l32 &
		(~bm_def_monitor[bm_index].bm_addr_mask_l32);
	setting.bm_addr_min_h32 = bm_def_monitor[bm_index].bm_addr_min_h32 &
		(~bm_def_monitor[bm_index].bm_addr_mask_h32);
	setting.bm_addr_max_l32 = bm_def_monitor[bm_index].bm_addr_max_l32 &
		(~bm_def_monitor[bm_index].bm_addr_mask_l32);
	setting.bm_addr_max_h32 = bm_def_monitor[bm_index].bm_addr_max_h32 &
		(~bm_def_monitor[bm_index].bm_addr_mask_h32);
	setting.bm_addr_mask_l32 = bm_def_monitor[bm_index].bm_addr_mask_l32;
	setting.bm_addr_mask_h32 = bm_def_monitor[bm_index].bm_addr_mask_h32;
	if (sdev->pdata->chip == IWHALE2_CHIP)
		setting.bm_userid = iwhale2_userid_cfg[bm_index];
	if (sdev->pdata->chip == SHARKL2_CHIP)
		setting.bm_userid = sharkl2_userid_cfg[bm_index];
	if (sdev->pdata->chip == SHARKLJ1_CHIP)
		setting.bm_userid = sharklj1_userid_cfg[bm_index];
	if (sdev->pdata->chip == ISHARKL2_CHIP)
		setting.bm_userid = isharkl2_userid_cfg[bm_index];
	if (sdev->subsys_set) {
		if (sdev->subsys_set != 0xff) {
			smsg_set(&msend, SMSG_CH_PMSYS_DBG, 0, 0,
				sdev->subsys_set - MUX_DAP_MUX_BM);
			smsg_send(SIPC_ID_PM_SYS, &msend, -1);
			smsg_set(&msend, SMSG_CH_PMSYS_DBG, 0, 0,
				setting.rw_cfg);
			smsg_send(SIPC_ID_PM_SYS, &msend, -1);
			smsg_set(&msend, SMSG_CH_PMSYS_DBG, 0, 0,
				setting.bm_addr_max_h32);
			smsg_send(SIPC_ID_PM_SYS, &msend, -1);
			smsg_set(&msend, SMSG_CH_PMSYS_DBG, 0, 0,
				setting.bm_addr_max_l32);
			smsg_send(SIPC_ID_PM_SYS, &msend, -1);
			smsg_set(&msend, SMSG_CH_PMSYS_DBG, 0, 0,
				setting.bm_addr_min_h32);
			smsg_send(SIPC_ID_PM_SYS, &msend, -1);
			smsg_set(&msend, SMSG_CH_PMSYS_DBG, 0, 0,
				setting.bm_addr_min_l32);
			smsg_send(SIPC_ID_PM_SYS, &msend, -1);
			if (sdev->pdata->chip == IWHALE2_CHIP ||
				sdev->pdata->chip == SHARKL2_CHIP ||
				sdev->pdata->chip == ISHARKL2_CHIP ||
				sdev->pdata->chip == SHARKLJ1_CHIP) {
				smsg_set(&msend, SMSG_CH_PMSYS_DBG, 0, 0,
				setting.bm_userid);
				smsg_send(SIPC_ID_PM_SYS, &msend, -1);
			}
			sdev->subsys_set = 0xff;
		}
	} else {
		smsg_set(&msend, SMSG_CH_PMSYS_DBG, 0, 0,
			bm_index + MUX_DAP_MUX_BM);
		smsg_send(SIPC_ID_PM_SYS, &msend, -1);
		smsg_set(&msend, SMSG_CH_PMSYS_DBG, 0, 0,
			setting.rw_cfg);
		smsg_send(SIPC_ID_PM_SYS, &msend, -1);
		smsg_set(&msend, SMSG_CH_PMSYS_DBG, 0, 0,
			setting.bm_addr_max_h32);
		smsg_send(SIPC_ID_PM_SYS, &msend, -1);
		smsg_set(&msend, SMSG_CH_PMSYS_DBG, 0, 0,
			setting.bm_addr_max_l32);
		smsg_send(SIPC_ID_PM_SYS, &msend, -1);
		smsg_set(&msend, SMSG_CH_PMSYS_DBG, 0, 0,
			setting.bm_addr_min_h32);
		smsg_send(SIPC_ID_PM_SYS, &msend, -1);
		smsg_set(&msend, SMSG_CH_PMSYS_DBG, 0, 0,
			setting.bm_addr_min_l32);
		smsg_send(SIPC_ID_PM_SYS, &msend, -1);
		if (sdev->pdata->chip == IWHALE2_CHIP ||
			sdev->pdata->chip == SHARKL2_CHIP ||
			sdev->pdata->chip == ISHARKL2_CHIP ||
			sdev->pdata->chip == SHARKLJ1_CHIP) {
			smsg_set(&msend, SMSG_CH_PMSYS_DBG, 0, 0,
			setting.bm_userid);
			smsg_send(SIPC_ID_PM_SYS, &msend, -1);
		}
	}
	sprd_djtag_bm_phy_set(sdev, bm_index, &setting);
}

static void sprd_djtag_auto_scan_clr_all_int
	(struct sprd_djtag_dev *sdev)
{
	int i;

	for (i = 0; i < 16; i++)
		sprd_djtag_autoscan_clr_int(sdev, i);
}

static void sprd_djtag_auto_scan_enable
	(struct sprd_djtag_dev *sdev)
{
	sprd_djtag_auto_scan_clr_all_int(sdev);
	sprd_djtag_auto_scan_en(sdev);
}

static void sprd_djtag_auto_scan_disable
	(struct sprd_djtag_dev *sdev)
{
	sprd_djtag_auto_scan_dis(sdev);
}

static int sprd_djtag_dbg_thread(void *data)
{
	struct sprd_djtag_dev *sdev = (struct sprd_djtag_dev *)data;
	struct smsg mrecv;
	int rval;
	struct sched_param param = {.sched_priority = 91};

	rval = smsg_ch_open(SIPC_ID_PM_SYS, SMSG_CH_PMSYS_DBG, -1);
	if (rval) {
		pr_err("Unable to request PM sys Debug channel:");
		pr_err("dst %d chn %d ret %d !\n",
			SIPC_ID_PM_SYS, SMSG_CH_PMSYS_DBG, rval);
		return rval;
	}
	smsg_set(&mrecv, SMSG_CH_PMSYS_DBG, 0, 0, 0);
	rval = smsg_recv(SIPC_ID_PM_SYS, &mrecv, -1);
	if (rval) {
		pr_err("Unable to recv sipc message!\n");
		return rval;
	}
	sprd_djtag_bm_def_set(sdev);
	sched_setscheduler(current, SCHED_RR, &param);
	pr_info("Enter djtag debug thread!\n");
	while (!kthread_should_stop()) {
		smsg_set(&mrecv, SMSG_CH_PMSYS_DBG, 0, 0, 0);
		rval = smsg_recv(SIPC_ID_PM_SYS, &mrecv, -1);
		if (rval == -EIO) {
			msleep(500);
			continue;
		}
		g_bm_info.match_index =
			sdev->bm_dev_info.match_index = mrecv.value;
		smsg_set(&mrecv, SMSG_CH_PMSYS_DBG, 0, 0, 0);
		smsg_recv(SIPC_ID_PM_SYS, &mrecv, -1);
		g_bm_info.match_id =
				sdev->bm_dev_info.match_id = mrecv.value;
		smsg_set(&mrecv, SMSG_CH_PMSYS_DBG, 0, 0, 0);
		smsg_recv(SIPC_ID_PM_SYS, &mrecv, -1);
		g_bm_info.match_cmd =
				sdev->bm_dev_info.match_cmd = mrecv.value;
		smsg_set(&mrecv, SMSG_CH_PMSYS_DBG, 0, 0, 0);
		smsg_recv(SIPC_ID_PM_SYS, &mrecv, -1);
		g_bm_info.match_addr_h =
				sdev->bm_dev_info.match_addr_h = mrecv.value;
		smsg_set(&mrecv, SMSG_CH_PMSYS_DBG, 0, 0, 0);
		smsg_recv(SIPC_ID_PM_SYS, &mrecv, -1);
		g_bm_info.match_addr_l =
				sdev->bm_dev_info.match_addr_l = mrecv.value;
		smsg_set(&mrecv, SMSG_CH_PMSYS_DBG, 0, 0, 0);
		smsg_recv(SIPC_ID_PM_SYS, &mrecv, -1);
		g_bm_info.match_data_ext_h = sdev->bm_dev_info.match_data_ext_h
			= mrecv.value;
		smsg_set(&mrecv, SMSG_CH_PMSYS_DBG, 0, 0, 0);
		smsg_recv(SIPC_ID_PM_SYS, &mrecv, -1);
		g_bm_info.match_data_ext_l = sdev->bm_dev_info.match_data_ext_l
			= mrecv.value;
		smsg_set(&mrecv, SMSG_CH_PMSYS_DBG, 0, 0, 0);
		smsg_recv(SIPC_ID_PM_SYS, &mrecv, -1);
		g_bm_info.match_data_h =
				sdev->bm_dev_info.match_data_h = mrecv.value;
		smsg_set(&mrecv, SMSG_CH_PMSYS_DBG, 0, 0, 0);
		smsg_recv(SIPC_ID_PM_SYS, &mrecv, -1);
		g_bm_info.match_data_l =
				sdev->bm_dev_info.match_data_l = mrecv.value;
		g_bm_info.chn_name =
			sdev->bm_dev_info.chn_name =
			bm_def_monitor[g_bm_info.match_index].chn_name;
		if (sdev->pdata->chip == IWHALE2_CHIP ||
			sdev->pdata->chip == SHARKL2_CHIP ||
			sdev->pdata->chip == ISHARKL2_CHIP ||
			sdev->pdata->chip == SHARKLJ1_CHIP) {
			smsg_set(&mrecv, SMSG_CH_PMSYS_DBG, 0, 0, 0);
			smsg_recv(SIPC_ID_PM_SYS, &mrecv, -1);
			g_bm_info.match_userid =
				sdev->bm_dev_info.match_userid = mrecv.value;
		}
		/* print the overlap BM info */
		pr_emerg("djtag int info:\nBM CHN:%d\nBM name:%s\n",
			sdev->bm_dev_info.match_index,
			sdev->bm_dev_info.chn_name);
		pr_emerg("Overlap A:0x%08X%08X\nOverlap D:0x%08X%08X%08X%08X\n",
			sdev->bm_dev_info.match_addr_h,
			sdev->bm_dev_info.match_addr_l,
			sdev->bm_dev_info.match_data_ext_h,
			sdev->bm_dev_info.match_data_ext_l,
			sdev->bm_dev_info.match_data_h,
			sdev->bm_dev_info.match_data_l);
		pr_emerg("Overlap CMD:0x%X\nMatch ID:%d\n",
			sdev->bm_dev_info.match_cmd,
			sdev->bm_dev_info.match_id);
		if (sdev->pdata->chip == IWHALE2_CHIP ||
			sdev->pdata->chip == SHARKL2_CHIP ||
			sdev->pdata->chip == ISHARKL2_CHIP ||
			sdev->pdata->chip == SHARKLJ1_CHIP) {
			pr_emerg("Match USERID:0x%X\n",
				sdev->bm_dev_info.match_userid);
		}
		/* judge weather BUG on */
		if (sdev->bm_dev_info.bm_panic_st == true) {
			pr_cont("DJTAG Bus Monitor thread enter panic!\n");
			BUG();
		}
	}

	return 0;
}

static int sprd_whale2_busmon_poll(void *data)
{
	struct sprd_djtag_dev *sdev = (struct sprd_djtag_dev *)data;
	int i;
	u32 val;

	pr_info("Enter whale2_busmon_poll thread!\n");
	while (!kthread_should_stop()) {
		msleep(1000);
		for (i = 0; i < ARRAY_SIZE(whale2_poll); i++) {
			if (sprd_djtag_enable(sdev))
				return -ENOLCK;
			sprd_djtag_scan_sel(sdev, whale2_poll[i].bm_arch,
				whale2_poll[i].bm_dap);
			val = sprd_djtag_bm_read(sdev, whale2_poll[i].bm_chain);
			sprd_djtag_disable(sdev);
			if (val & whale2_poll[i].bm_mask) {
				val = __fls(val & whale2_poll[i].bm_mask)
					+ whale2_poll[i].channel;
				if (sprd_djtag_enable(sdev))
					return -ENOLCK;
				sprd_djtag_scan_sel(sdev,
					bm_def_monitor[val].bm_arch,
					bm_def_monitor[val].bm_dap);
				sdev->bm_dev_info.chn_name =
					bm_def_monitor[val].chn_name;
				if (bm_def_monitor[val].bm_type == AHB_BM) {
					g_bm_info.match_addr_l =
						sdev->bm_dev_info.match_addr_l =
						sprd_djtag_bm_read(sdev,
						AHB_MATCH_ADDR);
					g_bm_info.match_addr_h =
						sdev->bm_dev_info.match_addr_h =
						0;
					g_bm_info.match_data_l =
						sdev->bm_dev_info.match_data_l =
						sprd_djtag_bm_read(sdev,
						AHB_MATCH_DATA_L32);
					g_bm_info.match_data_h =
						sdev->bm_dev_info.match_data_h =
						sprd_djtag_bm_read(sdev,
						AHB_MATCH_DATA_H32);
					g_bm_info.match_data_ext_l =
						sdev->bm_dev_info.
						match_data_ext_l = 0;
					g_bm_info.match_data_ext_h =
						sdev->bm_dev_info.
						match_data_ext_h = 0;
					g_bm_info.match_cmd =
						sdev->bm_dev_info.match_cmd =
						sprd_djtag_bm_read(sdev,
						AHB_MATCH_CMD);
					g_bm_info.match_id =
						sdev->bm_dev_info.match_id = 0;
				} else {
					g_bm_info.match_addr_l =
						sdev->bm_dev_info.match_addr_l =
						sprd_djtag_bm_read(sdev,
						AXI_MATCH_ADDR);
					g_bm_info.match_addr_h =
						sdev->bm_dev_info.match_addr_h =
						sprd_djtag_bm_read(sdev,
						AXI_MATCH_ADDR_H32);
					g_bm_info.match_data_l =
						sdev->bm_dev_info.match_data_l =
						sprd_djtag_bm_read(sdev,
						AXI_MATCH_DATA_L32);
					g_bm_info.match_data_h =
						sdev->bm_dev_info.match_data_h =
						sprd_djtag_bm_read(sdev,
						AXI_MATCH_DATA_H32);
					g_bm_info.match_data_ext_l =
						sdev->bm_dev_info.
						match_data_ext_l =
						sprd_djtag_bm_read(sdev,
						AXI_MATCH_DATA_EXT_L32);
					g_bm_info.match_data_ext_h =
						sdev->bm_dev_info.
						match_data_ext_h =
						sprd_djtag_bm_read(sdev,
						AXI_MATCH_DATA_EXT_H32);
					g_bm_info.match_cmd =
						sdev->bm_dev_info.match_cmd =
						sprd_djtag_bm_read(sdev,
						AXI_MATCH_CMD);
					g_bm_info.match_id =
						sdev->bm_dev_info.match_id =
						sprd_djtag_bm_read(sdev,
						AXI_MATCH_ID);
				}
				sprd_djtag_disable(sdev);
				/* print the overlap BM info */
				pr_emerg("djtag int info:\nBM CHN:%d\n", val);
				pr_emerg("BM name:%s\n",
					sdev->bm_dev_info.chn_name);
				pr_emerg("Overlap A:0x%08X%08X\nOverlap",
					sdev->bm_dev_info.match_addr_h,
					sdev->bm_dev_info.match_addr_l);
				pr_emerg(":0x%08X%08X%08X%08X\n",
					sdev->bm_dev_info.match_data_ext_h,
					sdev->bm_dev_info.match_data_ext_l,
					sdev->bm_dev_info.match_data_h,
					sdev->bm_dev_info.match_data_l);
				pr_emerg("Overlap CMD:0x%X\nMatch ID:%d\n",
					sdev->bm_dev_info.match_cmd,
					sdev->bm_dev_info.match_id);
				/* judge weather BUG on */
				if (sdev->bm_dev_info.bm_panic_st == true) {
					pr_emerg("DJTAG Bus Monitor ");
					pr_emerg("thread enter panic!\n");
					BUG();
				}
			}
		}
	}
	return 0;
}

static int sprd_iwhale2_busmon_poll(void *data)
{
	struct sprd_djtag_dev *sdev = (struct sprd_djtag_dev *)data;
	int i;
	u32 val;

	pr_info("Enter iwhale2_busmon_poll thread!\n");
	while (!kthread_should_stop()) {
		msleep(1000);
		for (i = 0; i < ARRAY_SIZE(iwhale2_poll); i++) {
			if (sprd_djtag_enable(sdev))
				return -ENOLCK;
			sprd_djtag_scan_sel(sdev, iwhale2_poll[i].bm_arch,
				iwhale2_poll[i].bm_dap);
			val = sprd_djtag_bm_read(sdev,
				iwhale2_poll[i].bm_chain);
			sprd_djtag_disable(sdev);
			if (val & iwhale2_poll[i].bm_mask) {
				val = __fls(val & iwhale2_poll[i].bm_mask)
					+ iwhale2_poll[i].channel;
				if (sprd_djtag_enable(sdev))
					return -ENOLCK;
				sprd_djtag_scan_sel(sdev,
					bm_def_monitor[val].bm_arch,
					bm_def_monitor[val].bm_dap);
				sdev->bm_dev_info.chn_name =
					bm_def_monitor[val].chn_name;
				if (bm_def_monitor[val].bm_type == AHB_BM) {
					g_bm_info.match_addr_l =
						sdev->bm_dev_info.match_addr_l =
						sprd_djtag_bm_read(sdev,
						AHB_MATCH_ADDR);
					g_bm_info.match_addr_h =
						sdev->bm_dev_info.match_addr_h =
						0;
					g_bm_info.match_data_l =
						sdev->bm_dev_info.match_data_l =
						sprd_djtag_bm_read(sdev,
						AHB_MATCH_DATA_L32);
					g_bm_info.match_data_h =
						sdev->bm_dev_info.match_data_h =
						sprd_djtag_bm_read(sdev,
						AHB_MATCH_DATA_H32);
					g_bm_info.match_data_ext_l =
						sdev->bm_dev_info.
						match_data_ext_l = 0;
					g_bm_info.match_data_ext_h =
						sdev->bm_dev_info.
						match_data_ext_h = 0;
					g_bm_info.match_cmd =
						sdev->bm_dev_info.match_cmd =
						sprd_djtag_bm_read(sdev,
						AHB_MATCH_CMD);
					g_bm_info.match_id =
						sdev->bm_dev_info.match_id = 0;
				} else {
					g_bm_info.match_addr_l =
						sdev->bm_dev_info.match_addr_l =
						sprd_djtag_bm_read(sdev,
						AXI_MATCH_ADDR);
					g_bm_info.match_addr_h =
						sdev->bm_dev_info.match_addr_h =
						sprd_djtag_bm_read(sdev,
						AXI_MATCH_ADDR_H32);
					g_bm_info.match_data_l =
						sdev->bm_dev_info.match_data_l =
						sprd_djtag_bm_read(sdev,
						AXI_MATCH_DATA_L32);
					g_bm_info.match_data_h =
						sdev->bm_dev_info.match_data_h =
						sprd_djtag_bm_read(sdev,
						AXI_MATCH_DATA_H32);
					g_bm_info.match_data_ext_l =
						sdev->bm_dev_info.
						match_data_ext_l =
						sprd_djtag_bm_read(sdev,
						AXI_MATCH_DATA_EXT_L32);
					g_bm_info.match_data_ext_h =
						sdev->bm_dev_info.
						match_data_ext_h =
						sprd_djtag_bm_read(sdev,
						AXI_MATCH_DATA_EXT_H32);
					g_bm_info.match_cmd =
						sdev->bm_dev_info.match_cmd =
						sprd_djtag_bm_read(sdev,
						AXI_MATCH_CMD);
					g_bm_info.match_id =
						sdev->bm_dev_info.match_id =
						sprd_djtag_bm_read(sdev,
						AXI_MATCH_ID);
					g_bm_info.match_userid =
						sdev->bm_dev_info.match_userid =
						sprd_djtag_bm_read(sdev,
						AXI_MATCH_USERID);
				}
				sprd_djtag_disable(sdev);
				/* print the overlap BM info */
				pr_emerg("djtag int info:\nBM CHN:%d\n", val);
				pr_emerg("BM name:%s\n",
					sdev->bm_dev_info.chn_name);
				pr_emerg("Overlap A:0x%08X%08X\nOverlap",
					sdev->bm_dev_info.match_addr_h,
					sdev->bm_dev_info.match_addr_l);
				pr_emerg(":0x%08X%08X%08X%08X\n",
					sdev->bm_dev_info.match_data_ext_h,
					sdev->bm_dev_info.match_data_ext_l,
					sdev->bm_dev_info.match_data_h,
					sdev->bm_dev_info.match_data_l);
				pr_emerg("Overlap CMD:0x%X\nMatch ID:%d\n",
					sdev->bm_dev_info.match_cmd,
					sdev->bm_dev_info.match_id);
				pr_emerg("match_userid:0x%08x\n",
					sdev->bm_dev_info.match_userid);
				/* judge weather BUG on */
				if (sdev->bm_dev_info.bm_panic_st == true) {
					pr_emerg("DJTAG Bus Monitor ");
					pr_emerg("thread enter panic!\n");
					BUG();
				}
			}
		}
	}
	return 0;
}

static ssize_t autoscan_on_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sprd_djtag_dev *sdev = dev_get_drvdata(dev);

	if (sdev->djtag_auto_scan_st == true)
		return sprintf(buf, "auto scan is enable!\n");
	else
		return sprintf(buf, "auto scan is disable!\n");
}

static ssize_t autoscan_on_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct sprd_djtag_dev *sdev = dev_get_drvdata(dev);
	int flag;

	if (kstrtoint(buf, 0, &flag))
		return -EINVAL;
	if (flag) {
		sdev->djtag_auto_scan_st = true;
		sprd_djtag_auto_scan_enable(sdev);
		pr_info("auto scan is enable!\n");
	} else {
		sdev->djtag_auto_scan_st = false;
		sprd_djtag_auto_scan_disable(sdev);
		pr_info("auto scan is disable!\n");
	}
	return count;
}
static DEVICE_ATTR_RW(autoscan_on);

static ssize_t autoscan_occur_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sprd_djtag_dev *sdev = dev_get_drvdata(dev);
	char *chn_info = NULL;
	char *occ_buf = NULL;
	int i, cnt = -1;

	/* djtag info momery alloc */
	chn_info = kzalloc(TEMP_BUF, GFP_KERNEL);
	if (!chn_info)
		return -ENOMEM;
	occ_buf = kzalloc(TEMP_BUF * DJTAG_MAX_AUTOSCAN_CHANS,
		GFP_KERNEL);
	if (!occ_buf)
		goto mem_err;
	for (i = 0; i < DJTAG_MAX_AUTOSCAN_CHANS; i++) {
		if (sdev->autoscan_info[i].occurred) {
			sprintf(chn_info,
"chain%d:\n addr:0x%08X\n pattern:0x%08X\n data:0x%08X\n mask:0x%08X\n\n",
			i, sdev->autoscan_info[i].autoscan_chain_addr,
			sdev->autoscan_info[i].autoscan_chain_pattern,
			sdev->autoscan_info[i].autoscan_chain_data,
			sdev->autoscan_info[i].autoscan_chain_mask);
			strcat(occ_buf, chn_info);
		}
	}
	cnt = sprintf(buf, "%s", occ_buf);
	kfree(occ_buf);
mem_err:
	kfree(chn_info);

	return cnt;
}
static DEVICE_ATTR_RO(autoscan_occur);

static ssize_t autoscan_cfg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sprd_djtag_dev *sdev = dev_get_drvdata(dev);
	struct sprd_djtag_reg *djtag_reg =
		(struct sprd_djtag_reg *)sdev->djtag_reg_base;
	char *chn_buf = NULL;
	char *chn_info = NULL;
	int i, cnt = -1;

	/* djtag info momery alloc */
	chn_buf = kzalloc(TEMP_BUF, GFP_KERNEL);
	if (!chn_buf)
		return -ENOMEM;
	chn_info = kzalloc(TEMP_BUF * (DJTAG_MAX_AUTOSCAN_CHANS + 1),
		GFP_KERNEL);
	if (!chn_info) {
		kfree(chn_buf);
		return -ENOMEM;
	}
	if (sprd_djtag_init(sdev)) {
		kfree(chn_buf);
		kfree(chn_info);
		return -ENOLCK;
	}
	for (i = 0; i < DJTAG_MAX_AUTOSCAN_CHANS; i++) {
		sprintf(chn_buf,
	"autoscan_addr[%d] --> addr:0x%08X;pattern:0x%08X; mask:0x%08X\n",
		i, djtag_reg->autoscan_chain_addr[i],
		djtag_reg->autoscan_chain_pattern[i],
		djtag_reg->autoscan_chain_mask[i]);
		strcat(chn_info, chn_buf);
	}
	sprintf(chn_buf, "autoscan enable register: %08x\n",
		djtag_reg->autoscan_en);
	strcat(chn_info, chn_buf);
	sprd_djtag_deinit(sdev);
	cnt = sprintf(buf, "%s", chn_info);
	kfree(chn_info);
	kfree(chn_buf);
	return cnt;
}

static ssize_t autoscan_cfg_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	/*
	 *addr_index subsystem DAP chain	pattern
	 *echo	2	11	0	4	0x12345678 > autoscan_cfg
	 */
	struct sprd_djtag_dev *sdev = dev_get_drvdata(dev);
	unsigned char patt[11];
	u32 chn, sub_index, dap, chain;
	unsigned long pattern;
	int ret;

	ret = sscanf(buf, "%u %u %u %u %s", &chn, &sub_index, &dap,
		&chain, patt);
	if (ret < 5)
		return -EINVAL;
	ret = kstrtoul(patt, 0, &pattern);
	if (ret) {
		pr_err("Input pattern %s is not legal!\n", patt);
		return -EINVAL;
	}
	if (sprd_djtag_init(sdev))
		return -ENOLCK;
	sdev->autoscan_setting[chn].autoscan_chain_addr =
		AUTOSCAN_ADDRESS(sub_index, dap, chain);
	sdev->autoscan_setting[chn].autoscan_chain_pattern = pattern;
	sdev->autoscan_setting[chn].scan = true;
	pr_info("addr: %x, pattern: %08x\n",
		sdev->autoscan_setting[chn].autoscan_chain_addr,
		sdev->autoscan_setting[chn].autoscan_chain_pattern);
	sprd_djtag_auto_scan_set(sdev, chn);
	sprd_djtag_deinit(sdev);

	return count;
}
static DEVICE_ATTR_RW(autoscan_cfg);

static ssize_t chn_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u32 bm_index;
	char *occ_info = NULL;
	char *chn_info = NULL;
	int cnt = -1;

	/* djtag info momery alloc */
	occ_info = kzalloc(TEMP_LITTLE_BUF, GFP_KERNEL);
	if (!occ_info)
		return -ENOMEM;
	chn_info = kzalloc(TEMP_LITTLE_BUF * bm_dev_id_max,
		GFP_KERNEL);
	if (!chn_info) {
		kfree(occ_info);
		return -ENOMEM;
	}
	for (bm_index = 0; bm_index < bm_dev_id_max; bm_index++) {
		sprintf(occ_info, "%d:%s\n",
			bm_index,
			bm_def_monitor[bm_index].chn_name);
		strcat(chn_info, occ_info);
	}
	cnt = sprintf(buf, "%s", chn_info);
	kfree(chn_info);
	kfree(occ_info);
	return cnt;
}
static DEVICE_ATTR_RO(chn);

static ssize_t dbg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sprd_djtag_dev *sdev = dev_get_drvdata(dev);
	struct ahb_bm_reg ahb_reg = {0};
	struct axi_bm_reg axi_reg = {0};
	char *chn_info = NULL;
	char *info_buf = NULL;
	u32 bm_index;
	int cnt = -1;

	/* djtag info momery alloc */
	chn_info = kzalloc(TEMP_BUF, GFP_KERNEL);
	if (!chn_info)
		return -ENOMEM;
	info_buf = kzalloc(TEMP_BUF * bm_dev_id_max, GFP_KERNEL);
	if (!info_buf) {
		pr_err("djtag alloc dbg mem failed!\n");
		kfree(chn_info);
		return -ENOMEM;
	}

	for (bm_index = 0; bm_index < bm_dev_id_max; bm_index++) {
		if (sprd_djtag_enable(sdev)) {
			kfree(chn_info);
			kfree(info_buf);
			return -ENOLCK;
		}
		sprd_djtag_scan_sel(sdev, bm_def_monitor[bm_index].bm_arch,
			bm_def_monitor[bm_index].bm_dap);
		if (bm_def_monitor[bm_index].bm_type == AHB_BM) {
			ahb_reg.ahb_bm_match_set_reg.bm_addr_min =
				sprd_djtag_bm_read(sdev, AHB_ADDR_MIN);
			ahb_reg.ahb_bm_addr_minh =
				sprd_djtag_bm_read(sdev, AHB_ADDR_MIN_H32);
			ahb_reg.ahb_bm_match_set_reg.bm_addr_max =
				sprd_djtag_bm_read(sdev, AHB_ADDR_MAX);
			ahb_reg.ahb_bm_addr_maxh =
				sprd_djtag_bm_read(sdev, AHB_ADDR_MAX_H32);
			ahb_reg.ahb_bm_match_set_reg.bm_addr_mask =
				sprd_djtag_bm_read(sdev, AHB_ADDR_MASK);
			ahb_reg.ahb_bm_addr_maskh =
				sprd_djtag_bm_read(sdev, AHB_ADDR_MASK_H32);
			if ((ahb_reg.ahb_bm_match_set_reg.bm_addr_min == 0x0) &&
				(ahb_reg.ahb_bm_addr_minh == 0x0) &&
				(ahb_reg.ahb_bm_match_set_reg.
				bm_addr_max == 0x0) &&
				(ahb_reg.ahb_bm_addr_maxh == 0x0)) {
				sprd_djtag_disable(sdev);
				continue;
			}
			sprintf(chn_info,
				"%d:0x%08X%08X ~ 0x%08X%08X mask:0x%X%X %s\n",
				bm_index,
				ahb_reg.ahb_bm_addr_minh,
				ahb_reg.ahb_bm_match_set_reg.bm_addr_min,
				ahb_reg.ahb_bm_addr_maxh,
				ahb_reg.ahb_bm_match_set_reg.bm_addr_max,
				ahb_reg.ahb_bm_addr_maskh,
				ahb_reg.ahb_bm_match_set_reg.bm_addr_mask,
				bm_def_monitor[bm_index].chn_name);
			strcat(info_buf, chn_info);
		} else {
			axi_reg.axi_bm_match_set_reg.bm_addr_min_l32 =
				sprd_djtag_bm_read(sdev, AXI_ADDR_MIN);
			axi_reg.axi_bm_match_set_reg.bm_addr_min_h32 =
				sprd_djtag_bm_read(sdev, AXI_ADDR_MIN_H32);
			axi_reg.axi_bm_match_set_reg.bm_addr_max_l32 =
				sprd_djtag_bm_read(sdev, AXI_ADDR_MAX);
			axi_reg.axi_bm_match_set_reg.bm_addr_max_h32 =
				sprd_djtag_bm_read(sdev, AXI_ADDR_MAX_H32);
			axi_reg.axi_bm_match_set_reg.bm_addr_mask_l32 =
				sprd_djtag_bm_read(sdev, AXI_ADDR_MASK);
			axi_reg.axi_bm_match_set_reg.bm_addr_mask_h32 =
				sprd_djtag_bm_read(sdev, AXI_ADDR_MASK_H32);
			if (sdev->pdata->chip == IWHALE2_CHIP ||
				sdev->pdata->chip == SHARKL2_CHIP ||
				sdev->pdata->chip == ISHARKL2_CHIP ||
				sdev->pdata->chip == SHARKLJ1_CHIP)
				axi_reg.axi_bm_match_userid =
				sprd_djtag_bm_read(sdev, AXI_USER_CFG)
				& USER_CFG_USERID_MASK;
			if ((axi_reg.axi_bm_match_set_reg.
				bm_addr_min_l32 == 0) &&
				(axi_reg.axi_bm_match_set_reg.
				bm_addr_min_h32 == 0x0) &&
				(axi_reg.axi_bm_match_set_reg.
				bm_addr_max_l32 == 0x0) &&
				(axi_reg.axi_bm_match_set_reg.
				bm_addr_max_h32 == 0x0)) {
				sprd_djtag_disable(sdev);
				continue;
			}
			if (sdev->pdata->chip == IWHALE2_CHIP ||
				sdev->pdata->chip == SHARKL2_CHIP ||
				sdev->pdata->chip == ISHARKL2_CHIP ||
				sdev->pdata->chip == SHARKLJ1_CHIP) {
				sprintf(chn_info,
				"%d:0x%08X%08X~0x%08X%08X userid:0x%08X mask: 0x%X_%X%s\n",
				bm_index,
				axi_reg.axi_bm_match_set_reg.bm_addr_min_h32,
				axi_reg.axi_bm_match_set_reg.bm_addr_min_l32,
				axi_reg.axi_bm_match_set_reg.bm_addr_max_h32,
				axi_reg.axi_bm_match_set_reg.bm_addr_max_l32,
				axi_reg.axi_bm_match_userid,
				axi_reg.axi_bm_match_set_reg.bm_addr_mask_h32,
				axi_reg.axi_bm_match_set_reg.bm_addr_mask_l32,
				bm_def_monitor[bm_index].chn_name);
			} else {
				sprintf(chn_info,
				"%d:0x%08X%08X~0x%08X%08X mask: 0x%X_%X%s\n",
				bm_index,
				axi_reg.axi_bm_match_set_reg.bm_addr_min_h32,
				axi_reg.axi_bm_match_set_reg.bm_addr_min_l32,
				axi_reg.axi_bm_match_set_reg.bm_addr_max_h32,
				axi_reg.axi_bm_match_set_reg.bm_addr_max_l32,
				axi_reg.axi_bm_match_set_reg.bm_addr_mask_h32,
				axi_reg.axi_bm_match_set_reg.bm_addr_mask_l32,
				bm_def_monitor[bm_index].chn_name);
			}
			strcat(info_buf, chn_info);
		}
		sprd_djtag_disable(sdev);
	}
	if (info_buf[0] == 0) {
		kfree(chn_info);
		kfree(info_buf);
		return sprintf(buf, ":-) ! No action was monitored by BM!!!\n");
	}
	cnt = sprintf(buf, "%s", info_buf);
	kfree(info_buf);
	kfree(chn_info);
	return cnt;
}

static ssize_t dbg_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct sprd_djtag_dev *sdev = dev_get_drvdata(dev);
	unsigned char chn[8], start[20], end[20], mod[3], user[20];
	unsigned long start_addr, end_addr, userid, bm_index = 0;
	unsigned long channel = 0;
	unsigned long subsys;
	u32 rd_wt, ret;

	if (sdev->pdata->chip == IWHALE2_CHIP ||
		sdev->pdata->chip == SHARKL2_CHIP ||
		sdev->pdata->chip == ISHARKL2_CHIP ||
		sdev->pdata->chip == SHARKLJ1_CHIP) {
		ret = sscanf(buf, "%s %s %s %s %s",
			chn, start, end, mod, user);
		if (ret < 5) {
			pr_err("format: chn start end mod userid.\n");
			pr_err("example: ap or 0 0x1 0x2 w 0x2\n");
			pr_err("if userid is 0xff monitor all userid\n");
			return -EINVAL;
		}
		ret = kstrtoul(user, 0, &userid);
		if (ret) {
			pr_err("userid is not in hex or decimal form.\n");
			return -EINVAL;
		}
	} else {
		ret = sscanf(buf, "%s %s %s %s", chn, start, end, mod);
		if (ret < 4) {
			pr_err("format: chn, start, end, mod.\n");
			pr_err("example: ap or 0 0x1 0x2 w\n");
			return -EINVAL;
		}
	}
	subsys = 0xff;
	if (sdev->pdata->chip == SHARKL2_CHIP ||
		sdev->pdata->chip == SHARKLJ1_CHIP) {
		if (strcmp(chn, "ap") == 0)
			subsys = NMUX_AP_BM;
		if (strcmp(chn, "mm") == 0)
			subsys = NMUX_MM_BM;
		if (strcmp(chn, "gpu") == 0)
			subsys = NMUX_GPU_BM;
		if (strcmp(chn, "aon") == 0)
			subsys = NMUX_AON_BM;
		if (strcmp(chn, "pub") == 0)
			subsys = NMUX_PUB_BM;
		if (strcmp(chn, "wtlcp") == 0)
			subsys = NMUX_WTLCP_BM;
		if (strcmp(chn, "pubcp") == 0)
			subsys = NMUX_PUBCP_BM;
	} else {
		if (strcmp(chn, "ap") == 0)
			subsys = MUX_AP_BM;
		if (strcmp(chn, "cpu") == 0)
			subsys = MUX_CPU_BM;
		if (strcmp(chn, "gpu") == 0)
			subsys = MUX_GPU_BM;
		if (strcmp(chn, "vsp") == 0)
			subsys = MUX_VSP_BM;
		if (strcmp(chn, "cam") == 0)
			subsys = MUX_CAM_BM;
		if (strcmp(chn, "disp") == 0)
			subsys = MUX_DISP_BM;
		if (strcmp(chn, "pubcp") == 0)
			subsys = MUX_PUBCP_BM;
		if (strcmp(chn, "wtlcp") == 0)
			subsys = MUX_WTLCP_BM;
		if (strcmp(chn, "agcp") == 0)
			subsys = MUX_AGCP_BM;
		if (strcmp(chn, "pub0") == 0)
			subsys = MUX_PUB0_BM;
		if (strcmp(chn, "pub1") == 0)
			subsys = MUX_PUB1_BM;
		if (strcmp(chn, "aon") == 0)
			subsys = MUX_AON_BM;
	}
	if (subsys == 0xff) {
		if ((chn[0] >= '0') && (chn[0] <= '9')) {
			ret = kstrtoul(chn, 0, &channel);
			if (ret)
				pr_err("chn %s is not num.\n", chn);
			if (channel > bm_dev_id_max)
				pr_err("the BM channel is too big\n");
		} else {
			pr_err("please input a channel number! as 0-9,ap\n");
			return -EINVAL;
		}
	}
	/* get the monitor start and end address */
	ret = kstrtoul(start, 0, &start_addr);
	if (ret) {
		pr_err("start is not in hex or decimal form.\n");
		return -EINVAL;
	}
	ret = kstrtoul(end, 0, &end_addr);
	if (ret) {
		pr_err("end is not in hex or decimal form.\n");
		return -EINVAL;
	}
	/* get the monitor action */
	if ((strcmp(mod, "r") == 0) || (strcmp(mod, "R") == 0))
		rd_wt = BM_READ;
	else if ((strcmp(mod, "w") == 0) || (strcmp(mod, "W") == 0))
		rd_wt = BM_WRITE;
	else if ((strcmp(mod, "rw") == 0) || (strcmp(mod, "RW") == 0))
		rd_wt = BM_READWRITE;
	else {
		pr_err("please input a legal channel mode! e.g: r,w,rw\n");
		return -EINVAL;
	}
	pr_info("str addr 0x%lx; end addr 0x%lx; chn %s; rw %d\n",
		start_addr, end_addr, chn, rd_wt);
	if (((channel >= bm_dev_id_max) && (subsys == 0xff)) ||
		(rd_wt > BM_READWRITE) || (start_addr > end_addr))
		return -EINVAL;

	/* set subsys BM config */
	if (subsys != 0xff) {
		pr_info("subsys setting !!\n");
		sdev->subsys_set = subsys + MUX_DAP_MUX_BM;
		for (bm_index = 0; bm_index < bm_dev_id_max; bm_index++) {
			if (subsys != bm_def_monitor[bm_index].bm_arch)
				continue;
			sprd_djtag_bm_def_monitor_cfg(sdev, bm_index,
				start_addr, end_addr, rd_wt);
			if (sdev->pdata->chip == IWHALE2_CHIP ||
				sdev->pdata->chip == SHARKL2_CHIP ||
				sdev->pdata->chip == ISHARKL2_CHIP ||
				sdev->pdata->chip == SHARKLJ1_CHIP)
				sprd_djtag_bm_set_userid_cfg(sdev, bm_index,
				userid);
			sprd_djtag_bm_set_point(sdev, bm_index);
		}
	} else {
		sdev->subsys_set = 0;
		sprd_djtag_bm_def_monitor_cfg(sdev, channel,
			start_addr, end_addr, rd_wt);
		if (sdev->pdata->chip == IWHALE2_CHIP ||
			sdev->pdata->chip == SHARKL2_CHIP ||
			sdev->pdata->chip == ISHARKL2_CHIP ||
			sdev->pdata->chip == SHARKLJ1_CHIP)
			sprd_djtag_bm_set_userid_cfg(sdev, channel, userid);
		pr_info("single channel setting !!\n");
		sprd_djtag_bm_set_point(sdev, channel);
	}
	return strnlen(buf, count);
}
static DEVICE_ATTR_RW(dbg);

static ssize_t occur_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	struct sprd_djtag_dev *sdev = dev_get_drvdata(dev);
	ssize_t ret;

	if (!sdev->bm_dev_info.chn_name)
		return sprintf(buf, ":-) ! No action was monitored by BM!!!\n");
	ret = sprintf(buf, "%s\n addr: 0x%X 0x%X\n CMD: 0x%X\n"
		"data: 0x%X 0x%X\n data_ext: 0x%X 0x%X\n id 0x%X\n",
		sdev->bm_dev_info.chn_name,
		sdev->bm_dev_info.match_addr_h,
		sdev->bm_dev_info.match_addr_l,
		sdev->bm_dev_info.match_cmd,
		sdev->bm_dev_info.match_data_h,
		sdev->bm_dev_info.match_data_l,
		sdev->bm_dev_info.match_data_ext_h,
		sdev->bm_dev_info.match_data_ext_l,
		sdev->bm_dev_info.match_id);
	if (sdev->pdata->chip == IWHALE2_CHIP ||
		sdev->pdata->chip == SHARKL2_CHIP ||
		sdev->pdata->chip == ISHARKL2_CHIP ||
		sdev->pdata->chip == SHARKLJ1_CHIP) {
		ret += sprintf(buf + ret, "match userid:0x%X\n",
		sdev->bm_dev_info.match_userid);
	}
	return ret;
}
static DEVICE_ATTR_RO(occur);

static ssize_t subsys_status_show(struct device *dev,
		struct device_attribute *attr,  char *buf)
{
	struct sprd_djtag_dev *sdev = dev_get_drvdata(dev);
	u32 subsys_index, chain_index, tmp;
	char *st_buf = NULL;
	char *subsys_st = NULL;
	int cnt = -1;

	/* djtag info momery alloc */
	st_buf = kzalloc(200, GFP_KERNEL);
	if (!st_buf)
		return -ENOMEM;

	subsys_st = kzalloc(sdev->pdata->subsys_debug_cnt * 200, GFP_KERNEL);
	if (!subsys_st)
		goto mem_err;
	for (subsys_index = 0; subsys_index < sdev->pdata->subsys_debug_cnt;
		subsys_index++) {
		sprd_djtag_enable(sdev);
		sprd_djtag_scan_sel(sdev, sdev->pdata->subsys_debug_chains
			[subsys_index].subsys,
			sdev->pdata->subsys_debug_chains
			[subsys_index].dap_index);
		sprintf(st_buf, "%s DAP%lu-->",
				sdev->pdata->subsys_debug_chains
				[subsys_index].subsystem_name,
				sdev->pdata->subsys_debug_chains
				[subsys_index].dap_index);
		strcat(subsys_st, st_buf);
		for (chain_index = sdev->pdata->subsys_debug_chains
			[subsys_index].chain_start;
			chain_index < sdev->pdata->subsys_debug_chains
			[subsys_index].chain_start +
			sdev->pdata->subsys_debug_chains[subsys_index].chain_num
			; chain_index++) {
			tmp = sprd_djtag_get_bitmap(sdev, chain_index + 8);
			sprintf(st_buf, "chain%d:0x%08x  ", chain_index, tmp);
			strcat(subsys_st, st_buf);
		}
		strcat(subsys_st, "\n");
		sprd_djtag_disable(sdev);
	}
	cnt = sprintf(buf, "%s\n", subsys_st);
	kfree(subsys_st);
mem_err:
	kfree(st_buf);
	return cnt;
}
static DEVICE_ATTR_RO(subsys_status);

static ssize_t panic_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct sprd_djtag_dev *sdev = dev_get_drvdata(dev);

	if (sdev->bm_dev_info.bm_panic_st == true)
		return sprintf(buf, "The BM panic is open.\n");
	else
		return sprintf(buf, "The BM panic is close.\n");

	return 0;
}

static ssize_t panic_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct sprd_djtag_dev *sdev = dev_get_drvdata(dev);
	unsigned long panic_flg, ret;

	ret = kstrtoul(buf, 0, &panic_flg);
	if (ret)
		return -EINVAL;
	if (panic_flg) {
		pr_info("Reopen BM panic.\n");
		sdev->bm_dev_info.bm_panic_st = true;
	} else {
		pr_info("Disable BM panic.\n");
		sdev->bm_dev_info.bm_panic_st = false;
	}
	return strnlen(buf, count);
}
static DEVICE_ATTR_RW(panic);

static ssize_t chain_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "The selected chain value is %x.\n"
		, bm_chains_value);
}

static ssize_t chain_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct sprd_djtag_dev *sdev = dev_get_drvdata(dev);
	unsigned char sub[8], dap[8], chain[20], data[20], mod[3];
	unsigned long ddap, dchain, ddata, subsys;
	u32 rd_wt = BM_READ, ret;

	ret = sscanf(buf, "%s %s %s %s %s", sub, dap, chain, data, mod);
	if (ret < 5)
		return -EINVAL;
	subsys = 0xff;
	if (strcmp(sub, "ap") == 0)
		subsys = MUX_AP_BM;
	if (strcmp(sub, "cpu") == 0)
		subsys = MUX_CPU_BM;
	if (strcmp(sub, "gpu") == 0)
		subsys = MUX_GPU_BM;
	if (strcmp(sub, "vsp") == 0)
		subsys = MUX_VSP_BM;
	if (strcmp(sub, "cam") == 0)
		subsys = MUX_CAM_BM;
	if (strcmp(sub, "disp") == 0)
		subsys = MUX_DISP_BM;
	if (strcmp(sub, "pubcp") == 0)
		subsys = MUX_PUBCP_BM;
	if (strcmp(sub, "wtlcp") == 0)
		subsys = MUX_WTLCP_BM;
	if (strcmp(sub, "agcp") == 0)
		subsys = MUX_AGCP_BM;
	if (strcmp(sub, "pub0") == 0)
		subsys = MUX_PUB0_BM;
	if (strcmp(sub, "pub1") == 0)
		subsys = MUX_PUB1_BM;
	if (strcmp(sub, "aon") == 0)
		subsys = MUX_AON_BM;
	if (subsys == 0xff) {
		if ((sub[0] >= '0') && (sub[0] <= '9')) {
			ret = kstrtoul(sub, 0, &subsys);
			if (ret) {
				pr_err("dap number is error.");
				pr_err("please in hex or decimal form\n");
				return -EINVAL;
			}
			if (subsys > MUX_AON_BM) {
				pr_err("dap number is too big\n");
				return -EINVAL;
			}
		}
	}
	ret = kstrtoul(dap, 0, &ddap);
	if (ret) {
		pr_err("dap %s is not in hex or decimal form.\n", buf);
		return -EINVAL;
	}
	ret = kstrtoul(chain, 0, &dchain);
	if (ret) {
		pr_err("chain %s is not in hex or decimal form.\n", buf);
		return -EINVAL;
	}
	ret = kstrtoul(data, 0, &ddata);
	if (ret) {
		pr_err("data %s is not in hex or decimal form.\n", buf);
		return -EINVAL;
	}
	if ((strcmp(mod, "r") == 0) || (strcmp(mod, "R") == 0))
		rd_wt = BM_READ;
	else if ((strcmp(mod, "w") == 0) || (strcmp(mod, "W") == 0))
		rd_wt = BM_WRITE;
	if (sprd_djtag_enable(sdev))
		return -EINVAL;
	sprd_djtag_scan_sel(sdev, subsys,
		ddap);
	if (rd_wt == BM_WRITE)
		sprd_djtag_bm_write(sdev, dchain + 8, ddata);
	bm_chains_value = sprd_djtag_bm_read(sdev, dchain + 8);
	sprd_djtag_disable(sdev);
	return strnlen(buf, count);
}
static DEVICE_ATTR_RW(chain);

static ssize_t regs_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)

{
	struct sprd_djtag_dev *sdev = dev_get_drvdata(dev);
	unsigned char chn[8];
	unsigned long chn_int, ret, bm_index, subsys;

	ret = sscanf(buf, "%8s", chn);
	if (ret != 1)
		return -EINVAL;
	subsys = 0xff;
	if (sdev->pdata->chip == SHARKL2_CHIP ||
		sdev->pdata->chip == SHARKLJ1_CHIP) {
		if (strcmp(chn, "ap") == 0)
			subsys = NMUX_AP_BM;
		if (strcmp(chn, "mm") == 0)
			subsys = NMUX_MM_BM;
		if (strcmp(chn, "gpu") == 0)
			subsys = NMUX_GPU_BM;
		if (strcmp(chn, "aon") == 0)
			subsys = NMUX_AON_BM;
		if (strcmp(chn, "pub") == 0)
			subsys = NMUX_PUB_BM;
		if (strcmp(chn, "wtlcp") == 0)
			subsys = NMUX_WTLCP_BM;
		if (strcmp(chn, "pubcp") == 0)
			subsys = NMUX_PUBCP_BM;
	} else {
		if (strcmp(chn, "ap") == 0)
			subsys = MUX_AP_BM;
		if (strcmp(chn, "cpu") == 0)
			subsys = MUX_CPU_BM;
		if (strcmp(chn, "gpu") == 0)
			subsys = MUX_GPU_BM;
		if (strcmp(chn, "vsp") == 0)
			subsys = MUX_VSP_BM;
		if (strcmp(chn, "cam") == 0)
			subsys = MUX_CAM_BM;
		if (strcmp(chn, "disp") == 0)
			subsys = MUX_DISP_BM;
		if (strcmp(chn, "pubcp") == 0)
			subsys = MUX_PUBCP_BM;
		if (strcmp(chn, "wtlcp") == 0)
			subsys = MUX_WTLCP_BM;
		if (strcmp(chn, "agcp") == 0)
			subsys = MUX_AGCP_BM;
		if (strcmp(chn, "pub0") == 0)
			subsys = MUX_PUB0_BM;
		if (strcmp(chn, "pub1") == 0)
			subsys = MUX_PUB1_BM;
		if (strcmp(chn, "aon") == 0)
			subsys = MUX_AON_BM;
	}
	if (subsys == 0xff) {
		if ((chn[0] >= '0') && (chn[0] <= '9')) {
			ret = kstrtoul(chn, 0, &chn_int);
			if (ret)
				pr_err("chn %s is not num.\n", chn);
			if (chn_int >= bm_dev_id_max)
				pr_err("the BM channel is too big\n");
		} else {
			pr_err("please input a channel number! as 0-9,ap\n");
			return -EINVAL;
		}
	}
	if (subsys != 0xff) {
		pr_info("subsys setting !!\n");
		for (bm_index = 0; bm_index < bm_dev_id_max; bm_index++) {
			if (subsys != bm_def_monitor[bm_index].bm_arch)
				continue;
			sprd_djtag_bm_phy_reg_check(sdev, bm_index);
		}
	} else {
		sprd_djtag_bm_phy_reg_check(sdev, chn_int);
	}

	return strnlen(buf, count);
}
static DEVICE_ATTR_WO(regs);

static ssize_t whale2poll_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	struct sprd_djtag_dev *sdev = dev_get_drvdata(dev);

	if (sdev->bm_dev_info.whale2poll == true)
		return sprintf(buf, "The whale2_busmon_poll is open.\n");
	else
		return sprintf(buf, "The whale2_busmon_poll is close.\n");

	return 0;
}

static ssize_t whale2poll_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct sprd_djtag_dev *sdev = dev_get_drvdata(dev);
	static struct task_struct *bm_poll;
	unsigned long poll_flg, ret;

	ret = kstrtoul(buf, 0, &poll_flg);
	if (ret)
		return -EINVAL;
	if (poll_flg) {
		pr_info("Reopen BM whale2_busmon_poll.\n");
		if (!bm_poll && (sdev->pdata->chip == WHALE2_CHIP)) {
			bm_poll = kthread_create(sprd_whale2_busmon_poll, sdev,
			"busmon_poll");
			if (IS_ERR(bm_poll)) {
				dev_err(dev,
				"Failed to create kthread: busmon_poll\n");
				return PTR_ERR(bm_poll);
			}
			wake_up_process(bm_poll);
		}
		sdev->bm_dev_info.whale2poll = true;
	} else {
		pr_info("Disable whale2_busmon_poll.\n");
		if (bm_poll)
			kthread_stop(bm_poll);
		bm_poll = NULL;
		sdev->bm_dev_info.whale2poll = false;
	}
	return strnlen(buf, count);
}
static DEVICE_ATTR_RW(whale2poll);

static ssize_t iwhale2poll_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	struct sprd_djtag_dev *sdev = dev_get_drvdata(dev);

	if (sdev->bm_dev_info.iwhale2poll == true)
		return sprintf(buf, "The iwhale2_busmon_poll is open.\n");
	else
		return sprintf(buf, "The iwhale2_busmon_poll is close.\n");

	return 0;
}

static ssize_t iwhale2poll_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct sprd_djtag_dev *sdev = dev_get_drvdata(dev);
	static struct task_struct *bm_poll;
	unsigned long poll_flg, ret;

	ret = kstrtoul(buf, 0, &poll_flg);
	if (ret)
		return -EINVAL;
	if (poll_flg) {
		pr_info("Reopen BM iwhale2_busmon_poll.\n");
		if (!bm_poll && (sdev->pdata->chip == IWHALE2_CHIP)) {
			bm_poll = kthread_create(sprd_iwhale2_busmon_poll, sdev,
			"busmon_poll");
			if (IS_ERR(bm_poll)) {
				dev_err(dev,
				"Failed to create kthread: busmon_poll\n");
				return PTR_ERR(bm_poll);
			}
			wake_up_process(bm_poll);
		}
		sdev->bm_dev_info.iwhale2poll = true;
	} else {
		pr_info("Disable iwhale2_busmon_poll.\n");
		if (bm_poll)
			kthread_stop(bm_poll);
		bm_poll = NULL;
		sdev->bm_dev_info.iwhale2poll = false;
	}
	return strnlen(buf, count);
}
static DEVICE_ATTR_RW(iwhale2poll);

int sprd_bm_monitor_cp(long start_addr, long end_addr)
{
	struct device_node *np = NULL;
	struct platform_device *pdev = NULL;
	struct sprd_djtag_dev *sdev = NULL;
	int old_subsys = 0xff;
	int new_subsys;
	u32 bm_index;
	u32 rd_wt;
	int temp;

	np = of_find_compatible_node(NULL, NULL, compatible_str);
	if (!np)
		return 0;
	pdev = of_find_device_by_node(np);
	sdev = platform_get_drvdata(pdev);
	rd_wt = BM_WRITE;
	if (sdev == NULL)
		return -ENODEV;
	if (sdev->pdata->chip == SHARKL2_CHIP ||
		sdev->pdata->chip == SHARKLJ1_CHIP)
		temp = NMUX_AP_BM;
	else
		temp = MUX_CPU_BM;
	for (bm_index = 0; bm_index < bm_dev_id_max; bm_index++) {
		if (bm_def_monitor[bm_index].bm_arch <= temp) {
			new_subsys = bm_def_monitor[bm_index].bm_arch;
			if (new_subsys != old_subsys) {
				old_subsys = new_subsys;
				sdev->subsys_set = new_subsys + MUX_DAP_MUX_BM;
			}
			sprd_djtag_bm_def_monitor_cfg(sdev, bm_index,
				start_addr, end_addr, rd_wt);
			sprd_djtag_bm_set_userid_cfg(sdev, bm_index,
				0xff);
			sprd_djtag_bm_set_point(sdev, bm_index);
		}
	}
	pr_info("str addr 0x%lx; end addr 0x%lx; rw %d\n",
		start_addr, end_addr, rd_wt);

	return 0;
}
EXPORT_SYMBOL(sprd_bm_monitor_cp);

int sprd_bm_monitor_ap(long start_addr, long end_addr)
{
	struct device_node *np = NULL;
	struct platform_device *pdev = NULL;
	struct sprd_djtag_dev *sdev = NULL;
	int old_subsys = 0xff;
	int new_subsys;
	u32 bm_index;
	u32 rd_wt;
	int temp1, temp2;

	np = of_find_compatible_node(NULL, NULL, compatible_str);
	if (!np)
		return 0;
	pdev = of_find_device_by_node(np);
	sdev = platform_get_drvdata(pdev);
	rd_wt = BM_WRITE;
	if (sdev == NULL)
		return -ENODEV;
	if (sdev->pdata->chip == SHARKL2_CHIP ||
		sdev->pdata->chip == SHARKLJ1_CHIP) {
		temp1 = NMUX_WTLCP_BM;
		temp2 = NMUX_PUBCP_BM;
	} else {
		temp1 = MUX_PUBCP_BM;
		temp2 = MUX_AGCP_BM;
	}
	for (bm_index = 0; bm_index < bm_dev_id_max; bm_index++) {
		if (bm_def_monitor[bm_index].bm_arch >= temp1 &&
			bm_def_monitor[bm_index].bm_arch <= temp2) {
			new_subsys = bm_def_monitor[bm_index].bm_arch;
			if (new_subsys != old_subsys) {
				old_subsys = new_subsys;
				sdev->subsys_set = new_subsys + MUX_DAP_MUX_BM;
			}
			sprd_djtag_bm_def_monitor_cfg(sdev, bm_index,
				start_addr, end_addr, rd_wt);
			sprd_djtag_bm_set_userid_cfg(sdev, bm_index,
				0xff);
			sprd_djtag_bm_set_point(sdev, bm_index);
		}
	}
	pr_info("str addr 0x%lx; end addr 0x%lx; rw %d\n",
		start_addr, end_addr, rd_wt);

	return 0;
}
EXPORT_SYMBOL(sprd_bm_monitor_ap);

int sprd_bm_monitor_enable_ap(void)
{
	struct device_node *np = NULL;
	struct platform_device *pdev = NULL;
	struct sprd_djtag_dev *sdev = NULL;
	u32 bm_index;
	u32 rd_wt;
	int temp;

	np = of_find_compatible_node(NULL, NULL, compatible_str);
	if (!np)
		return 0;
	pdev = of_find_device_by_node(np);
	sdev = platform_get_drvdata(pdev);
	rd_wt = BM_WRITE;
	if (sdev == NULL)
		return -ENODEV;
	if (sdev->pdata->chip == SHARKL2_CHIP ||
		sdev->pdata->chip == SHARKLJ1_CHIP)
		temp = NMUX_AP_BM;
	else
		temp = MUX_CPU_BM;
	for (bm_index = 0; bm_index < bm_dev_id_max; bm_index++) {
		if (bm_def_monitor[bm_index].bm_arch <= temp)
			sprd_djtag_bm_phy_en(sdev, bm_index);
	}

	return 0;
}
EXPORT_SYMBOL(sprd_bm_monitor_enable_ap);

int sprd_bm_monitor_enable_aon(void)
{
	struct device_node *np = NULL;
	struct platform_device *pdev = NULL;
	struct sprd_djtag_dev *sdev = NULL;
	u32 bm_index;
	u32 rd_wt;
	int temp;

	np = of_find_compatible_node(NULL, NULL, compatible_str);
	if (!np)
		return 0;
	pdev = of_find_device_by_node(np);
	sdev = platform_get_drvdata(pdev);
	rd_wt = BM_WRITE;
	if (sdev == NULL)
		return -ENODEV;
	if (sdev->pdata->chip == SHARKL2_CHIP ||
		sdev->pdata->chip == SHARKLJ1_CHIP)
		temp = NMUX_AON_BM;
	else
		temp = MUX_AON_BM;
	for (bm_index = 0; bm_index < bm_dev_id_max; bm_index++) {
		if (bm_def_monitor[bm_index].bm_arch == temp)
			sprd_djtag_bm_phy_en(sdev, bm_index);
	}

	return 0;
}

int sprd_bm_monitor_enable_ca53(void)
{
	struct device_node *np = NULL;
	struct platform_device *pdev = NULL;
	struct sprd_djtag_dev *sdev = NULL;
	u32 bm_index;
	u32 rd_wt;
	int temp;

	np = of_find_compatible_node(NULL, NULL, compatible_str);
	if (!np)
		return 0;
	pdev = of_find_device_by_node(np);
	sdev = platform_get_drvdata(pdev);
	rd_wt = BM_WRITE;
	if (sdev == NULL)
		return -ENODEV;

	temp = NMUX_CPU_BM;
	for (bm_index = 0; bm_index < bm_dev_id_max; bm_index++) {
		if (bm_def_monitor[bm_index].bm_arch == temp)
			sprd_djtag_bm_phy_en(sdev, bm_index);
	}

	return 0;
}

static int __maybe_unused sprd_iwhale2_busmon_def(struct sprd_djtag_dev *sdev)
{
	struct device_node *np = NULL;
	struct resource res;
	long mem_rang[4] = {};
	int ret;

	/* get CP moden addr rang */
	np = of_find_node_by_name(NULL, "cp-modem");
	if (np) {
		ret = of_address_to_resource(np, 0, &res);
		if (!ret) {
			mem_rang[0] = res.start;
			mem_rang[1] = res.end;
		}
	} else {
		pr_err("sprd djtag&busmon get cp moden addr error!\n");
		return -EINVAL;
	}

	/* get Audio share memory addr rang, AGCP will access this addr rang */
	np = of_find_node_by_name(NULL, "sipc-mem");
	if (np) {
		ret = of_address_to_resource(np, 0, &res);
		if (!ret) {
			mem_rang[2] = res.start;
			mem_rang[3] = res.end;
		}
	} else {
		pr_err("sprd djtag&busmon get audio addr error!\n");
		return -EINVAL;
	}
	sprd_bm_monitor_ap(mem_rang[3] + 0x80000000 + 1, 0xffffffff);
	sprd_bm_monitor_cp(mem_rang[0], mem_rang[1]);

	return 0;
}

static struct attribute *sprd_djtag_bm_attrs[] = {
	&dev_attr_autoscan_on.attr,
	&dev_attr_autoscan_occur.attr,
	&dev_attr_autoscan_cfg.attr,
	&dev_attr_subsys_status.attr,
	&dev_attr_chn.attr,
	&dev_attr_dbg.attr,
	&dev_attr_chain.attr,
	&dev_attr_occur.attr,
	&dev_attr_panic.attr,
	&dev_attr_regs.attr,
	&dev_attr_whale2poll.attr,
	&dev_attr_iwhale2poll.attr,
	NULL,
};

static struct attribute_group djtag_attr_group = {
	.attrs	= sprd_djtag_bm_attrs,
	.name = "djtag",
};

static void sprd_djtag_bm_def_set(struct sprd_djtag_dev *sdev)
{
	u32 bm_index;
	int old_subsys = 0xff;
	int new_subsys;

	/*only enable ap busmon for bus status */
	if (sdev->pdata->chip == ISHARKL2_CHIP ||
		sdev->pdata->chip == SHARKLJ1_CHIP) {
		sprd_bm_monitor_enable_ap();
		sprd_bm_monitor_enable_aon();
	}

	if (sdev->pdata->chip == SHARKLJ1_CHIP)
		sprd_bm_monitor_enable_ca53();

	for (bm_index = 0; bm_index < bm_dev_id_max; bm_index++) {
		if ((bm_def_monitor[bm_index].bm_addr_max_h32 >
			bm_def_monitor[bm_index].bm_addr_min_h32) ||
			((bm_def_monitor[bm_index].bm_addr_max_h32 ==
			bm_def_monitor[bm_index].bm_addr_min_h32) &&
			(bm_def_monitor[bm_index].bm_addr_max_l32 >
			bm_def_monitor[bm_index].bm_addr_min_l32))){
			new_subsys = bm_def_monitor[bm_index].bm_arch;
			if (new_subsys != old_subsys) {
				old_subsys = new_subsys;
				sdev->subsys_set = new_subsys + MUX_DAP_MUX_BM;
			}
			sprd_djtag_bm_set_point(sdev, bm_index);
		}
	}
}

static int sprd_djtag_remove(struct platform_device *pdev)
{
	struct sprd_djtag_dev *sdev = platform_get_drvdata(pdev);

	misc_deregister(&sdev->misc);
	sysfs_remove_group(&pdev->dev.kobj, &djtag_attr_group);

	return 0;
}

static int sprd_djtag_open(struct inode *inode, struct file *filp)
{
	pr_info("%s!\n", __func__);
	return 0;
}

static int sprd_djtag_release(struct inode *inode, struct file *filp)
{
	pr_info("%s!\n", __func__);
	return 0;
}

static long sprd_djtag_ioctl(struct file *filp, unsigned int cmd,
	unsigned long args)
{
	struct device_node *np = NULL;
	struct platform_device *pdev = NULL;
	struct sprd_djtag_dev *sdev = NULL;
	u32 bm_index;

	np = of_find_compatible_node(NULL, NULL, compatible_str);
	if (!np)
		return -EINVAL;
	pdev = of_find_device_by_node(np);
	sdev = platform_get_drvdata(pdev);
	if (!sdev)
		return -EINVAL;

	switch (cmd) {
	case 0:
		for (bm_index = 0; bm_index < bm_dev_id_max; bm_index++)
			sprd_djtag_bm_phy_close(sdev, bm_index);
		break;
	case 1:
		 for (bm_index = 0; bm_index < bm_dev_id_max; bm_index++) {
			u32 temp;

			if (sprd_djtag_enable(sdev))
				return -EINVAL;
			sprd_djtag_scan_sel(sdev,
				bm_def_monitor[bm_index].bm_arch,
				bm_def_monitor[bm_index].bm_dap);
			if (bm_def_monitor[bm_index].bm_type == AHB_BM)
				temp = sprd_djtag_bm_read(sdev, AHB_ADDR_MAX);
			else
				temp = sprd_djtag_bm_read(sdev, AXI_ADDR_MAX);
			if (temp > 0)
				sprd_djtag_bm_write(sdev, AHB_CHN_INT,
				(BM_INT_CLR | BM_INT_EN | BM_CHN_EN));
			sprd_djtag_disable(sdev);
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

#ifdef CONFIG_COMPAT
static long sprd_djtag_compac_ioctl(struct file *filp, unsigned int cmd,
	unsigned long args)
{
	return sprd_djtag_ioctl(filp, cmd, (unsigned long)compat_ptr(args));
}
#endif

static const struct file_operations sprd_djtag_fops = {
	.owner = THIS_MODULE,
	.open = sprd_djtag_open,
	.release = sprd_djtag_release,
	.unlocked_ioctl = sprd_djtag_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = sprd_djtag_compac_ioctl,
#endif
};


static int sprd_djtag_probe(struct platform_device *pdev)
{
	struct sprd_djtag_dev *sdev = NULL;
	struct resource *res;
	const struct of_device_id *lock_of_id = NULL;
	void __iomem *djtag_base = NULL;
	u32 djtag_irq;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "sprd djtag get io resource failed!\n");
		return -ENODEV;
	}
	djtag_base = devm_ioremap_nocache(&pdev->dev,
		res->start, resource_size(res));
	if (!djtag_base) {
		dev_err(&pdev->dev, "sprd djtag get base address failed!\n");
		return -ENODEV;
	}

	djtag_irq = platform_get_irq(pdev, 0);
	if (djtag_irq == 0) {
		dev_err(&pdev->dev, "Can't get the djtag irq number!\n");
		return -EIO;
	}

	/* djtag device momery alloc */
	sdev = devm_kzalloc(&pdev->dev, sizeof(*sdev), GFP_KERNEL);
	if (!sdev)
		return -ENOMEM;
	sdev->dev.parent = &pdev->dev;
	sdev->djtag_irq = djtag_irq;
	sdev->djtag_reg_base = djtag_base;
	sdev->djtag_auto_scan_st = 0;
	sdev->bm_dev_info.bm_panic_st = true;
	sdev->bm_dev_info.whale2poll = false;
	sdev->djtag_auto_scan_st = false;
	sdev->subsys_set = 0;
	sdev->hw_lock = of_hwspin_lock_request(pdev->dev.of_node, "djtag");
	if (!sdev->hw_lock) {
		pr_err("DJATG can not get the hardware spinlock.\n");
		return -ENXIO;
	}
	lock_of_id = of_match_node(sprd_djtag_match,
				   pdev->dev.of_node);
	compatible_str = lock_of_id->compatible;
	sdev->pdata = lock_of_id->data;
	bm_def_monitor = sdev->pdata->bm_def_monitor;
	bm_dev_id_max = sdev->pdata->bm_dev_id_max;
	dev_set_name(&sdev->dev, "djtag");
	dev_set_drvdata(&pdev->dev, sdev);
	ret = sysfs_create_group(&pdev->dev.kobj, &djtag_attr_group);
	if (ret)
		return ret;
	ret = devm_request_irq(&pdev->dev, djtag_irq,
		sprd_djtag_isr, IRQF_TRIGGER_NONE, pdev->name, sdev);
	if (ret) {
		dev_err(&pdev->dev, "Unable to request DJTAG irq!\n");
		return ret;
	}
	sdev->aon_apb = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
				"sprd,syscon-aon-glb");
	sdev->ap_ahb = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
				"sprd,syscon-ap-glb");
	if (sdev->pdata->chip == WHALE2_CHIP) {
		sdev->cam_ahb =
			syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
				"sprd,syscon-cam-glb");
		sdev->vsp_ahb =
			syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
				"sprd,syscon-vsp-glb");
		sdev->disp_ahb =
			syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
				"sprd,syscon-disp-glb");
	}
	spin_lock_init(&sdev->djtag_lock);
	sdev->misc.parent = &pdev->dev;
	sdev->misc.fops = &sprd_djtag_fops;
	sdev->misc.name = "sprd-bm-djtag";
	sdev->misc.minor = MISC_DYNAMIC_MINOR;
	ret = misc_register(&sdev->misc);
	if (ret)
		return ret;
	sdev->djtag_thl = kthread_create(sprd_djtag_dbg_thread, sdev,
		"djtag_dbg-%d-%d",
		SIPC_ID_PM_SYS, SMSG_CH_PMSYS_DBG);
	if (IS_ERR(sdev->djtag_thl)) {
		dev_err(&pdev->dev, "Failed to create kthread: djtag_dbg\n");
		ret = PTR_ERR(sdev->djtag_thl);
		goto irq_err;
	}
	wake_up_process(sdev->djtag_thl);
	pr_info("sprd djtag probe done!\n");

	return 0;
irq_err:
	misc_deregister(&sdev->misc);
	hwspin_lock_free(sdev->hw_lock);
	return ret;
}

#ifdef CONFIG_PM
static int sprd_djtag_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	return 0;
}

static int sprd_djtag_resume(struct platform_device *pdev)
{
	struct sprd_djtag_dev *sdev = dev_get_drvdata(&pdev->dev);
	int bm_index;

	/*only enable ap busmon for bus status */
	if (sdev->pdata->chip == ISHARKL2_CHIP ||
		sdev->pdata->chip == SHARKLJ1_CHIP)
		sprd_bm_monitor_enable_ap();

	if (sdev->pdata->chip == SHARKLJ1_CHIP)
		sprd_bm_monitor_enable_ca53();
	/* sharkl2 and isharkl2 have no eb*/
	if (sdev->pdata->chip == SHARKL2_CHIP ||
		sdev->pdata->chip == ISHARKL2_CHIP ||
		sdev->pdata->chip == SHARKLJ1_CHIP)
		return 0;
	for (bm_index = 0; bm_index < bm_dev_id_max; bm_index++) {
		if ((bm_def_monitor[bm_index].bm_addr_max_h32 >
			bm_def_monitor[bm_index].bm_addr_min_h32) ||
			((bm_def_monitor[bm_index].bm_addr_max_h32 ==
			bm_def_monitor[bm_index].bm_addr_min_h32) &&
			(bm_def_monitor[bm_index].bm_addr_max_l32 >
			bm_def_monitor[bm_index].bm_addr_min_l32))){
			if (bm_def_monitor[bm_index].bm_arch == MUX_AP_BM) {
				regmap_update_bits(sdev->ap_ahb,
					REG_AP_AHB_AHB_EB,
				BIT_AP_AHB_BUSMON_EB, BIT_AP_AHB_BUSMON_EB);
				break;
			}
		}
	}

	return 0;
}
#endif

static const struct of_device_id sprd_djtag_match[] = {
	{ .compatible = "sprd,bm-djtag-iwhale2", .data = &iwhale2_data},
	{ .compatible = "sprd,bm-djtag-whale", .data = &whale_data},
	{ .compatible = "sprd,bm-djtag-whale2", .data = &whale2_data},
	{ .compatible = "sprd,bm-djtag-isharkl2", .data = &isharkl2_data},
	{ .compatible = "sprd,bm-djtag-sharkl2", .data = &sharkl2_data},
	{ .compatible = "sprd,bm-djtag-sharklj1", .data = &sharklj1_data},
	{ },
};

static struct platform_driver sprd_djtag_driver = {
	.probe = sprd_djtag_probe,
	.remove = sprd_djtag_remove,
#ifdef CONFIG_PM
	.suspend = sprd_djtag_suspend,
	.resume = sprd_djtag_resume,
#endif
	.driver = {
		.owner = THIS_MODULE,
		.name = "sprd_bm_djtag",
		.of_match_table = sprd_djtag_match,
	},
};
module_platform_driver(sprd_djtag_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aiden Cheng<aiden.cheng@spreadtrum.com>");
MODULE_DESCRIPTION("spreadtrum platform DJTAG driver");


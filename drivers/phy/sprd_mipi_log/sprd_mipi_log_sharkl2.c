/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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
#define pr_fmt(fmt)		"sprd-mipi-log: " fmt

#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include "sprd_mipi_log.h"

static unsigned long aon_apb;
static unsigned long serdes_apb;
static unsigned long dbg_apb;

static void mipi_log_error(const char *string)
{
	pr_err("%s", string);
}

static void reg_write(unsigned long reg, u32 val)
{
	writel(val, (void __iomem *)reg);
}

static u32 reg_read(unsigned long reg)
{
	return readl((void __iomem *)reg);
}

static void reg_bits_set(unsigned long reg, u32 bits)
{
	reg_write(reg, reg_read(reg) | bits);
}

static void reg_bits_clr(unsigned long reg, u32 bits)
{
	reg_write(reg, reg_read(reg) & ~bits);
}

static void phy_glb_enable(void)
{
	reg_bits_clr(aon_apb + REG_APB_RST2,
				BIT_SERDES_DPHY_APB_SORT_REST);
	reg_bits_set(aon_apb + REG_APB_EB1,
				BIT_SERDES_DPHY_EB);
	reg_bits_clr(aon_apb + REG_PWR_CTRL,
				BIT_MIPI_2P2L_PS_PD_L |
				BIT_MIPI_2P2L_PS_PD_M);
	reg_bits_set(aon_apb + REG_APB_EB2,
				BIT_SERDES_DPHY_CEG_EB |
				BIT_SERDES_DPHY_REF_EB);
	reg_bits_set(aon_apb + REG_2P2L_DBG_PHY_CTRL,
				BIT_2P2L_DBG_EN);
	reg_bits_set(aon_apb + REG_APB_EB2,
				BIT_BUGMON_DMA_EB);

	reg_bits_set(aon_apb + REG_APB_EB1,
				BIT_SERDES_DPHY_EB);

	reg_bits_set(aon_apb + REG_2P2P_S_PHY_CTRL,
				BIT_2P2L_TESTCLR_S_SET);
	reg_bits_set(aon_apb + REG_2P2P_M_PHY_CTRL,
				BIT_2P2L_TESTCLR_M_SET);
	reg_bits_set(aon_apb + REG_2P2P_S_PHY_CTRL,
				BIT_2P2L_TESTCLR_S);
	reg_bits_set(aon_apb + REG_2P2P_M_PHY_CTRL,
				BIT_2P2L_TESTCLR_M);
	reg_bits_clr(aon_apb + REG_2P2P_S_PHY_CTRL,
				BIT_2P2L_TESTCLR_S);
	reg_bits_clr(aon_apb + REG_2P2P_M_PHY_CTRL,
				BIT_2P2L_TESTCLR_M);

}

static uint32_t phy_core_read_function(unsigned long addr, uint32_t offset)
{
	return readl_relaxed((const void __iomem *)(addr + offset));
}

static void phy_core_write_function(unsigned long addr, uint32_t offset,
				    uint32_t data)
{
	writel_relaxed(data, (void __iomem *)(addr + offset));
}

static void mipi_select_channel(enum channel_en ch)
{
	reg_bits_clr(serdes_apb + REG_SERDES_APB_FUNC_EN, BIT(0));
	reg_bits_set(serdes_apb + REG_SERDES_APB_FUNC_EN, BIT(0));
	reg_write(serdes_apb + REG_SERDES_APB_FSM_CUT_OFF_LEN, 0x20);
	switch (ch) {
	case CH_EN_TRAINING:
		reg_write(serdes_apb + REG_SERDES_APB_CH_EN,
				BIT_SERDES_APB_TRAINING);
		break;
	case CH_EN_WTL:
		reg_write(serdes_apb + REG_SERDES_APB_CH_EN,
				BIT_SERDES_APB_WTL);
		break;
	case CH_EN_DBG_SYS:
		reg_write(serdes_apb + REG_SERDES_APB_CH_EN,
				BIT_SERDES_APB_DBG_SYS);
		break;
	case CH_EN_DBG_BUS:
		reg_write(serdes_apb + REG_SERDES_APB_CH_EN,
				BIT_SERDES_APB_DBG_BUS);
		break;
	case CH_EN_MDAR:
		reg_write(serdes_apb + REG_SERDES_APB_CH_EN,
				BIT_SERDES_APB_MDAR);
		break;
	default:
		break;
	}
	reg_write(serdes_apb + REG_SERDES_APB_FUNNEL_EN, 1);
}

int mipi_log_start(struct mipi_log_device *mipi_log, enum channel_en ch)
{
	dphy_t *dphy = &mipi_log->phy_instance;
	dsih_error_t result = OK;
	int wait_count = 0;

	phy_glb_enable();

	if (dphy->status == INITIALIZED) {
		mipi_select_channel(ch);
		return OK;
	}

	result = mipi_log_dphy_open(dphy);
	if (result != OK) {
		pr_err("mipi_dsih_dphy_open fail (%d)!\n", result);
		return result;
	}

	result = mipi_log_dphy_configure(dphy,
					mipi_log->max_lanes,
					mipi_log->phy_freq);
	if (result != OK) {
		pr_err("mipi_dsih_dphy_configure fail (%d)!\n", result);
		return result;
	}

	while (1) {
		if (mipi_log_dphy_is_locked(dphy))
			break;

		udelay(3);
		wait_count++;

		if (wait_count % 1000 == 0)
			pr_err("warning: busy waiting!\n");

		if (wait_count == 100000) {
			pr_err("Error: dsi phy can't be locked!!!\n");
			break;
		}
	}

	mipi_select_channel(ch);
	mipi_log->running = true;
	return OK;
}

int mipi_log_stop(struct mipi_log_device *mipi_log)
{
	dphy_t *dphy = &mipi_log->phy_instance;

	mipi_log_dphy_close(dphy);
	reg_bits_clr(aon_apb + REG_APB_EB2,
				BIT_BUGMON_DMA_EB);
	reg_bits_clr(dbg_apb + REG_DBG_PHY_CTRL_0,
				BIT_SHUTDOWN_DB);
	reg_bits_set(aon_apb + REG_PWR_CTRL,
				BIT_MIPI_2P2L_PS_PD_L |
				BIT_MIPI_2P2L_PS_PD_M);
	mipi_log->running = false;
	return OK;
}

static int phy_ctx_init(struct mipi_log_device *mipi_log)
{
	dphy_t *dphy = &mipi_log->phy_instance;

	dphy->reference_freq = 26*1000;
	dphy->address = (unsigned long)dbg_apb;
	dphy->log_info = NULL;
	dphy->log_error = mipi_log_error;
	dphy->core_read_function = phy_core_read_function;
	dphy->core_write_function = phy_core_write_function;

	return OK;
}

static int mipi_log_parse_dt(struct mipi_log_device *mipi_log_dev)
{
	struct device_node *node = mipi_log_dev->dev->of_node;
	struct resource r;
	void __iomem *addr;

	if (!node)
		return -ENODEV;

	if (of_address_to_resource(node, 0, &r) != 0) {
		pr_err("can't get aon apb register base address)\n");
		return -EINVAL;
	}

	addr = devm_ioremap_resource(mipi_log_dev->dev, &r);
	if (IS_ERR(addr)) {
		pr_err("aon_apb ioremap failed!\n");
		return PTR_ERR(addr);
	}
	aon_apb = (unsigned long)addr;

	if (of_address_to_resource(node, 1, &r) != 0) {
		pr_err("can't get serdes apb register base address)\n");
		return -EINVAL;
	}

	addr = devm_ioremap_resource(mipi_log_dev->dev, &r);
	if (IS_ERR(addr)) {
		pr_err("serdes_apb ioremap failed!\n");
		return PTR_ERR(addr);
	}
	serdes_apb = (unsigned long)addr;

	if (of_address_to_resource(node, 2, &r) != 0) {
		pr_err("can't get dbg apb register base address)\n");
		return -EINVAL;
	}

	addr = devm_ioremap_resource(mipi_log_dev->dev, &r);
	if (IS_ERR(addr)) {
		pr_err("dbg_apb ioremap failed!\n");
		return PTR_ERR(addr);
	}
	dbg_apb = (unsigned long)addr;

	of_property_read_u32(node,
			"phy-freq", &mipi_log_dev->phy_freq);
	pr_info("phy_freq = %d\n", mipi_log_dev->phy_freq);

	of_property_read_u32(node,
			"max-lanes", &mipi_log_dev->max_lanes);
	pr_info("lanes = %d\n", mipi_log_dev->max_lanes);

	return 0;
}


static int mipi_log_probe(struct platform_device *pdev)
{
	struct mipi_log_device *mipi_log_dev = NULL;
	int ret;

	mipi_log_dev = devm_kzalloc(&pdev->dev,
			sizeof(struct mipi_log_device), GFP_KERNEL);
	if (!mipi_log_dev)
		return -ENOMEM;

	mipi_log_dev->dev = &pdev->dev;

	ret = mipi_log_parse_dt(mipi_log_dev);
	if (ret) {
		dev_err(&pdev->dev, "fail to parse mipi log dt properties\n");
		devm_kfree(&pdev->dev, mipi_log_dev);
		return -EINVAL;
	}

	platform_set_drvdata(pdev, mipi_log_dev);
	phy_ctx_init(mipi_log_dev);
	mipi_log_create_sysfs(mipi_log_dev);

	return 0;
}

static const struct of_device_id dt_ids[] = {
	{
		.compatible = "sprd,mipi-log",
	},
	{},
};

static struct platform_driver mipi_log_driver = {
	.probe = mipi_log_probe,
	.driver = {
		   .name = "sprd_mipi_log",
		   .of_match_table = dt_ids,
		   },
};

static int __init mipi_log_init(void)
{
	return platform_driver_register(&mipi_log_driver);
}

static void __exit mipi_log_exit(void)
{
	return platform_driver_unregister(&mipi_log_driver);
}

module_init(mipi_log_init);
module_exit(mipi_log_exit);

MODULE_DESCRIPTION("Spreadtrum SoC MIPI log driver");
MODULE_AUTHOR("leon.he <leon.he@spreadtrum.com>");
MODULE_LICENSE("GPL v2");

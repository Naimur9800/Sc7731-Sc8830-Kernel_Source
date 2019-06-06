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
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/io.h>
#include "sprd_mipi_log.h"

struct regmap *aon_apb;
struct regmap *disp_ahb;
struct regmap *ana_apb;
void __iomem *serdes_base;

static void mipi_log_error(const char *string)
{
	pr_err("%s", string);
}

void phy_glb_enable(void)
{
	regmap_update_bits(disp_ahb,
		    REG_DISP_AHB_AHB_RST, BIT_DISP_AHB_DSI1_SOFT_RST,
		    BIT_DISP_AHB_DSI1_SOFT_RST);
	udelay(10);
	regmap_update_bits(disp_ahb,
		    REG_DISP_AHB_AHB_RST, BIT_DISP_AHB_DSI1_SOFT_RST,
		    (unsigned int)(~BIT_DISP_AHB_DSI1_SOFT_RST));

	regmap_update_bits(disp_ahb,
		    REG_DISP_AHB_GEN_CKG_CFG, BIT_DISP_AHB_DPHY1_CFG_CKG_EN,
		    BIT_DISP_AHB_DPHY1_CFG_CKG_EN);
	regmap_update_bits(disp_ahb,
		    REG_DISP_AHB_AHB_EB, BIT_DISP_AHB_DSI1_EB,
		    BIT_DISP_AHB_DSI1_EB);
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
	regmap_update_bits(aon_apb,
		    REG_AON_APB_CGM_REG1, BIT_AON_APB_CGM_DPHY_REF_EN,
		    BIT_AON_APB_CGM_DPHY_REF_EN);
	regmap_update_bits(ana_apb,
		    REG_ANA_APB_MIPI_PHY_CTRL2,
		    BIT_ANA_APB_DSI_ENABLECLK |
		    BIT_ANA_APB_DSI_TXREQUESTHSCLK_S |
		    BIT_ANA_APB_DSI_ENABLE_0_S |
		    BIT_ANA_APB_DSI_ENABLE_1_S |
		    BIT_ANA_APB_DSI_ENABLE_2_S |
		    BIT_ANA_APB_DSI_ENABLE_3_S,
		    BIT_ANA_APB_DSI_ENABLECLK |
		    BIT_ANA_APB_DSI_TXREQUESTHSCLK_S |
		    BIT_ANA_APB_DSI_ENABLE_0_S |
		    BIT_ANA_APB_DSI_ENABLE_1_S |
		    BIT_ANA_APB_DSI_ENABLE_2_S |
		    BIT_ANA_APB_DSI_ENABLE_3_S);
	regmap_update_bits(ana_apb,
		    REG_ANA_APB_MIPI_PHY_CTRL2,
		    BIT_ANA_APB_DSI_SHUTDOWNZ |
		    BIT_ANA_APB_DSI_RSTZ |
		    BIT_ANA_APB_DSI_FORCEPLL_S,
		    BIT_ANA_APB_DSI_SHUTDOWNZ |
		    BIT_ANA_APB_DSI_RSTZ |
		    BIT_ANA_APB_DSI_FORCEPLL_S);
	regmap_update_bits(ana_apb,
		    REG_ANA_APB_DEBUG_LOG_CTRL,
		    BIT_ANA_APB_DSI_IF_SEL_S |
		    BIT_ANA_APB_LOG_MODE_EN,
		    BIT_ANA_APB_DSI_IF_SEL_S |
		    BIT_ANA_APB_LOG_MODE_EN);
	regmap_write(aon_apb, REG_AON_APB_SUB_SYS_DBG_SIG6, (1 << 2));
	__raw_writel(0x20, serdes_base + REG_SERDES_APB_FSM_CUT_OFF_LEN);
	if (CH_EN_WTL == ch)
		__raw_writel(BIT_SERDES_APB_WTL,
				serdes_base + REG_SERDES_APB_CH_EN);
	else if (CH_EN_TRAINING == ch)
		__raw_writel(BIT_SERDES_APB_TRAINING,
				serdes_base + REG_SERDES_APB_CH_EN);
	__raw_writel(1, serdes_base + REG_SERDES_APB_FUNC_EN);
	regmap_update_bits(aon_apb,
		    REG_AON_APB_SUB_SYS_DBG_SIG6,
		    BIT_AON_APB_AP_DBG_MOD_SEL(4),
		    BIT_AON_APB_AP_DBG_MOD_SEL(4));
}

int mipi_log_start(struct mipi_log_device *mipi_log, enum channel_en ch)
{
	dphy_t *dphy = &mipi_log->phy_instance;
	dsih_error_t result = OK;
	int wait_count = 0;
	u32 status = 0;

	phy_glb_enable();

	if (INITIALIZED == dphy->status) {
		mipi_select_channel(ch);
		return OK;
	}

	result = mipi_dsih_dphy_open(dphy);
	if (OK != result) {
		pr_err("mipi_dsih_dphy_open fail (%d)!\n", result);
		return result;
	}

	result = mipi_dsih_dphy_configure(dphy,
					mipi_log->max_lanes,
					mipi_log->phy_freq);
	if (OK != result) {
		pr_err("mipi_dsih_dphy_configure fail (%d)!\n", result);
		return result;
	}

	while (1) {
		status = phy_core_read_function(dphy->address, R_DPHY_STATUS);

		if ((status & BIT_DSI_PHY_STATUS_PHY_LOCK) &&
			(status & BIT_DSI_PHY_STATUS_PHY_STOPSTATECLKLANE))
			break;

		udelay(3);
		wait_count++;

		if (0 == wait_count % 1000)
			pr_err("warning: busy waiting!\n");

		if (wait_count == 100000) {
			pr_err("Errior: dsi phy can't be locked!!!\n");
			break;
		}
	}

	mipi_dsih_dphy_enable_hs_clk(dphy, 1);
	mipi_dsih_dphy_enable_nc_clk(dphy, false);

	mipi_select_channel(ch);

	return OK;
}

int mipi_log_stop(struct mipi_log_device *mipi_log)
{
	dphy_t *dphy = &mipi_log->phy_instance;

	mipi_dsih_dphy_close(dphy);
	__raw_writel(0, serdes_base + REG_SERDES_APB_FUNC_EN);

	return OK;
}

static int phy_ctx_init(struct mipi_log_device *mipi_log)
{
	dphy_t *dphy = &mipi_log->phy_instance;

	dphy->reference_freq = 26*1000;
	dphy->address = (unsigned long)mipi_log->base;
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

	if (!node)
		return -ENODEV;

	if (0 != of_address_to_resource(node, 0, &r)) {
		pr_err("can't get dsi1 register base address)\n");
		return -EINVAL;
	}
	pr_info("DSI1 base addr = 0x%llx\n", r.start);

	mipi_log_dev->base = devm_ioremap_resource(mipi_log_dev->dev, &r);
	if (IS_ERR(mipi_log_dev->base)) {
		pr_err("mipi_log_dev->base ioremap failed!\n");
		return PTR_ERR(mipi_log_dev->base);
	}

	if (0 != of_address_to_resource(node, 1, &r)) {
		pr_err("can't get serdes register base address)\n");
		return -EINVAL;
	}
	pr_info("SERDES APB base addr = 0x%llx\n", r.start);

	serdes_base = devm_ioremap_resource(mipi_log_dev->dev, &r);
	if (IS_ERR(serdes_base)) {
		pr_err("serdes_base ioremap failed!\n");
		return PTR_ERR(serdes_base);
	}

	of_property_read_u32(node,
			"phy-freq", &mipi_log_dev->phy_freq);
	pr_info("phy_freq = %d\n", mipi_log_dev->phy_freq);

	of_property_read_u32(node,
			"max-lanes", &mipi_log_dev->max_lanes);
	pr_info("lanes = %d\n", mipi_log_dev->max_lanes);

	disp_ahb = syscon_regmap_lookup_by_compatible("sprd,sys-disp-ahb");
	if (IS_ERR(disp_ahb))
		return PTR_ERR(disp_ahb);
	aon_apb = syscon_regmap_lookup_by_compatible("sprd,sys-aon-apb");
	if (IS_ERR(aon_apb))
		return PTR_ERR(aon_apb);
	ana_apb = syscon_regmap_lookup_by_compatible("sprd,sys-ana-apb");
	if (IS_ERR(ana_apb))
		return PTR_ERR(ana_apb);

	return 0;
}


static int mipi_log_probe(struct platform_device *pdev)
{
	struct mipi_log_device *mipi_log_dev = NULL;
	int ret;

	mipi_log_dev = devm_kzalloc(&pdev->dev,
			sizeof(struct mipi_log_device), GFP_KERNEL);
	if (!mipi_log_dev) {
		pr_err("alloc mipi log device fail\n");
		return -ENOMEM;
	}

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
		   .owner = THIS_MODULE,
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

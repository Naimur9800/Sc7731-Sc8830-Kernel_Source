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

#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include "dsi_1_21a/mipi_dsih_api.h"
#include "dsi_1_21a/mipi_dsih_dphy.h"

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(__fmt) "sprd-adf: [%20s] <%d> "__fmt, __func__, __LINE__
#define ENTER() pr_err("enter\n")

static int first_power_on = true;

static int sprd_mipi_dphy_init(struct phy *phy) {

	if (first_power_on) {
		++phy->power_count;
		first_power_on = false;
	}
	return OK;
}

static int sprd_mipi_dphy_exit(struct phy *phy) {
	dsih_ctrl_t *dsi_instance = phy_get_drvdata(phy);

	ENTER();
	pr_info("get dsi drvdata : %p\n", dsi_instance);
	return OK;
}

static int sprd_mipi_video_phy_power_on(struct phy *phy)
{
	dsih_error_t result = OK;
	dsih_ctrl_t *dsi_instance = phy_get_drvdata(phy);
	dphy_t *dphy = &(dsi_instance->phy_instance);

	pr_info("lanes : %d\n", dsi_instance->max_lanes);
	pr_info("phy_freq : %d\n", dsi_instance->phy_feq);
	result = mipi_dsih_open(dsi_instance);
	if (OK != result) {
		pr_err("mipi_dsih_open fail (%d)!\n", result);
		return result;
	}

	result = mipi_dsih_dphy_configure(dphy,
					dsi_instance->max_lanes,
					dsi_instance->phy_feq);
	if (OK != result) {
		pr_err("mipi_dsih_dphy_configure fail (%d)!\n", result);
		return result;
	}

	return OK;
}

static int sprd_mipi_video_phy_power_off(struct phy *phy)
{
	dsih_error_t result = OK;
	dsih_ctrl_t *dsi_instance = phy_get_drvdata(phy);
	dphy_t *dphy = &(dsi_instance->phy_instance);

	/*set dsi_instance status*/
	mipi_dsih_dphy_enable_hs_clk(dphy, false);
	result = mipi_dsih_close(dsi_instance);
	if (OK != result)
		pr_err("sprd_dsi_uninit fail (%d)!\n", result);

	return result;
}

static struct phy_ops sprd_mipi_video_phy_ops = {
	.init		= sprd_mipi_dphy_init,
	.exit		= sprd_mipi_dphy_exit,
	.power_on	= sprd_mipi_video_phy_power_on,
	.power_off	= sprd_mipi_video_phy_power_off,
	.owner		= THIS_MODULE,
};

static const struct of_device_id sprd_mipi_phy_ids[] = {
	{
		.compatible = "sprd,mipi-video-phy",
		.data = &sprd_mipi_video_phy_ops,
	},
	{ },
};

static int sprd_mipi_video_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *phy_provider;
	struct phy *phy;
	const struct of_device_id *match;

	ENTER();
	pr_info("%p\n", &sprd_mipi_video_phy_ops);

	match = of_match_node(sprd_mipi_phy_ids, dev->of_node);
	pr_info("Got data in 0x%p\n", match->data);

	phy = devm_phy_create(dev, NULL, &sprd_mipi_video_phy_ops);
	if (IS_ERR(phy)) {
		dev_err(dev, "failed to create Display Port PHY\n");
		return PTR_ERR(phy);
	}

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static struct platform_driver sprd_mipi_video_phy_driver = {
	.probe	= sprd_mipi_video_phy_probe,
	.driver = {
		.owner = THIS_MODULE,
		.of_match_table	= of_match_ptr(sprd_mipi_phy_ids),
		.name  = "sprd-mipi-video-phy",
	}
};

module_platform_driver(sprd_mipi_video_phy_driver);

MODULE_DESCRIPTION("Spreadtrum SoC MIPI DSI PHY driver");
MODULE_AUTHOR("infi.chen <infi.chen@spreadtrum.com>");
MODULE_LICENSE("GPL v2");

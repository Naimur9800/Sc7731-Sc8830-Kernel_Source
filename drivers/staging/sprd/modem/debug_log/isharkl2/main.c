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
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd/isharkl2/isharkl2_glb.h>

#include "core.h"
#include "phy.h"

#define CHANNEL_MDAR			BIT(0)
#define CHANNEL_DBG_SYS			BIT(1)
#define CHANNEL_WTL			BIT(2)
#define CHANNEL_DBG_BUS			BIT(3)
#define CHANNEL_TRAINING		BIT(15)

static struct regmap *aon_apb;
static unsigned long serdes_apb;

static void dbg_log_init(struct dbg_log_device *dbg)
{
	int ret = 0;

	ret = dbg_phy_init(dbg->phy);

	if (!ret)
		regmap_update_bits(aon_apb, REG_AON_APB_APB_EB0,
				   BIT_AON_APB_DBG_SERDES_EB, BIT_AON_APB_DBG_SERDES_EB);
	else
		pr_err("dbg phy init fail!\n");
}

static void dbg_log_exit(struct dbg_log_device *dbg)
{
	dbg_phy_exit(dbg->phy);

	regmap_update_bits(aon_apb, REG_AON_APB_APB_EB0,
			   BIT_AON_APB_DBG_SERDES_EB, (u32)~BIT_AON_APB_DBG_SERDES_EB);
}

static void dbg_log_chn_sel(struct dbg_log_device *dbg)
{
	u32 ch = dbg->channel;

	reg_bits_clr(serdes_apb + REG_SERDES_APB_FUNC_EN,
		     BIT_SERDES_APB_FUNC_EN_FUNC_EN);
	reg_bits_set(serdes_apb + REG_SERDES_APB_FUNC_EN,
		     BIT_SERDES_APB_FUNC_EN_FUNC_EN);
	reg_write(serdes_apb + REG_SERDES_APB_FSM_CUT_OFF_LEN, 0x20);
	switch (ch) {
	case CH_EN_TRAINING:
		reg_write(serdes_apb + REG_SERDES_APB_CH_EN, CHANNEL_TRAINING);
		break;
	case CH_EN_WTL:
		reg_write(serdes_apb + REG_SERDES_APB_CH_EN, CHANNEL_WTL);
		break;
	case CH_EN_DBG_SYS:
		reg_write(serdes_apb + REG_SERDES_APB_CH_EN, CHANNEL_DBG_SYS);
		break;
	case CH_EN_DBG_BUS:
		reg_write(serdes_apb + REG_SERDES_APB_CH_EN, CHANNEL_DBG_BUS);
		break;
	case CH_EN_MDAR:
		reg_write(serdes_apb + REG_SERDES_APB_CH_EN, CHANNEL_MDAR);
		break;
	default:
		return;
	}
	reg_write(serdes_apb + REG_SERDES_APB_FUNNEL_EN, 1);
}

static struct dbg_log_ops ops = {
	.init = dbg_log_init,
	.exit = dbg_log_exit,
	.select = dbg_log_chn_sel,
};

static struct phy_ctx phy = {
	.freq = 1500000,
	.lanes = 4,
};

static int dbg_log_probe(struct platform_device *pdev)
{
	struct resource r;
	void __iomem *addr;

	if (of_address_to_resource(pdev->dev.of_node, 0, &r) != 0) {
		pr_err("can't get serdes register base address\n");
		return -EINVAL;
	}

	addr = ioremap_nocache(r.start, resource_size(&r));
	if (IS_ERR(addr)) {
		pr_err("serdes_apb ioremap failed!\n");
		return PTR_ERR(addr);
	}
	serdes_apb = (unsigned long)addr;

	if (of_address_to_resource(pdev->dev.of_node, 1, &r) != 0) {
		pr_err("can't get dbg apb ctrl base address\n");
		return -EINVAL;
	}

	addr = ioremap_nocache(r.start, resource_size(&r));
	if (IS_ERR(addr)) {
		pr_err("dbg_apb ioremap failed!\n");
		return PTR_ERR(addr);
	}
	phy.base = (unsigned long)addr;

	aon_apb = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						  "sprd,syscon-aon-apb");
	if (IS_ERR(aon_apb))
		return PTR_ERR(aon_apb);

	dbg_log_device_register(&pdev->dev, &ops, &phy);

	return 0;
}

static const struct of_device_id dt_ids[] = {
	{ .compatible = "sprd,dbg-log-isharkl2", },
	{},
};

static struct platform_driver dbg_log_driver = {
	.probe = dbg_log_probe,
	.driver = {
		.name = "modem-dbg-log",
		.of_match_table = dt_ids,
	},
};

module_platform_driver(dbg_log_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("yaping.long <yaping.long@spreadtrum.com>");
MODULE_DESCRIPTION("Spreadtrum SoC Modem Debug Log Driver");

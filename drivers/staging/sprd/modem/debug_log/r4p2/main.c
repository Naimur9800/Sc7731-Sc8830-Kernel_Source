/*
 * Copyright (C) 2018 Spreadtrum Communications Inc.
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

#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include "core.h"
#include "phy.h"

#define FREQ_OFFSET (50 * 1000)
#define MATCH_FREQ(f, src_f) (abs((f) - (src_f)) <= FREQ_OFFSET)

extern int sprd_cam_pw_on(void);
extern int sprd_cam_pw_off(void);

static void dbg_phy_ps_pd_l(struct dbg_log_device *dbg, int h)
{
	regmap_u_bits(dbg->aon_apb, REG_AON_APB_PWR_CTRL,
		      BIT_AON_APB_MIPI_CSI_2P2LANE_PS_PD_L, h);
}

static void dbg_phy_ps_pd_s(struct dbg_log_device *dbg, int h)
{
	regmap_u_bits(dbg->aon_apb, REG_AON_APB_PWR_CTRL,
		      BIT_AON_APB_MIPI_CSI_2P2LANE_PS_PD_S, h);
}

static void dbg_phy_iso_sw_en(struct dbg_log_device *dbg, int h)
{
	regmap_u_bits(dbg->aon_apb, REG_AON_APB_PWR_CTRL,
		      BIT_AON_APB_MIPI_CSI_2P2LANE_ISO_SW_EN, h);
}

static void inter_dbg_log_init(struct dbg_log_device *dbg)
{
	regmap_update_bits(dbg->aon_apb, REG_AON_APB_APB_EB2,
			   BIT_AON_APB_SERDES_DPHY_CFG_EB |
			   BIT_AON_APB_SERDES_DPHY_REF_EB,
			   BIT_AON_APB_SERDES_DPHY_CFG_EB |
			   BIT_AON_APB_SERDES_DPHY_REF_EB);

	clk_prepare_enable(dbg->clk_src[dbg->phy->clk_sel]);

	clk_prepare_enable(dbg->clk_serdes_eb);

	if (dbg->mm) {
		pr_info("MIPI LOG use MM Power Domain\n");
		sprd_cam_pw_on();
	}

	dbg_phy_ps_pd_l(dbg, 1);
	dbg_phy_ps_pd_s(dbg, 1);
	dbg_phy_iso_sw_en(dbg, 1);
	dbg_phy_init(dbg->phy);
}

static void inter_dbg_log_exit(struct dbg_log_device *dbg)
{
	dbg_phy_exit(dbg->phy);
	dbg_phy_iso_sw_en(dbg, 1);
	dbg_phy_ps_pd_s(dbg, 1);
	dbg_phy_ps_pd_l(dbg, 1);
	if (dbg->mm)
		sprd_cam_pw_off();

	clk_disable_unprepare(dbg->clk_src[dbg->phy->clk_sel]);

	clk_disable_unprepare(dbg->clk_serdes_eb);

	regmap_update_bits(dbg->aon_apb, REG_AON_APB_APB_EB2,
			   BIT_AON_APB_SERDES_DPHY_CFG_EB |
			   BIT_AON_APB_SERDES_DPHY_REF_EB,
			   (unsigned int)(~(BIT_AON_APB_SERDES_DPHY_CFG_EB |
					    BIT_AON_APB_SERDES_DPHY_REF_EB)));
}

static void dbg_phy_ext_ctl(void *ext_para)
{
	struct dbg_log_device *dbg = ext_para;

	dbg_phy_iso_sw_en(dbg, 0);
	mdelay(1);
	dbg_phy_ps_pd_s(dbg, 0);
	mdelay(1);
	dbg_phy_ps_pd_l(dbg, 0);
	mdelay(5);
}

static void inter_dbg_log_chn_sel(struct dbg_log_device *dbg)
{
	if (dbg->channel) {
		dbg_phy_enable(dbg->phy, 1, dbg_phy_ext_ctl, dbg);
		dbg->serdes.channel = dbg->serdes.ch_map[dbg->channel - 1];
		serdes_enable(&dbg->serdes, 1);
	} else {
		serdes_enable(&dbg->serdes, 0);
		dbg_phy_enable(dbg->phy, 0, 0, 0);
	}
}

static int inter_dbg_log_is_freq_valid(struct dbg_log_device *dbg, unsigned int freq)
{
	int i;
	unsigned long src_freq;

	pr_debug("input freq %d\n", freq);
	for (i = 0; i < CLK_SRC_MAX; i++) {
		src_freq = (clk_get_rate(dbg->clk_src[i]) / 1000);
		if (MATCH_FREQ(freq, src_freq)) {
			dbg->phy->clk_sel = i;
			return 0;
		}
	}
	pr_debug("input freq %d not match\n", freq);
	return -EINVAL;
}

static int inter_dbg_log_get_valid_channel(struct dbg_log_device *dbg,
				      const char *buf)
{
	int i;
	int cmp_len;

	pr_debug("input channel %s", buf);
	cmp_len = strlen(buf) - 1;
	if (!strncasecmp(STR_CH_DISABLE, buf, cmp_len))
		return 0;
	for (i = 0; i < dbg->serdes.ch_num; i++) {
		if (!strncasecmp(dbg->serdes.ch_str[i], buf, cmp_len))
			return i + 1;
	}
	pr_debug("not match input channel %s", buf);
	return -2;
}

static struct dbg_log_ops ops = {
	.init = inter_dbg_log_init,
	.exit = inter_dbg_log_exit,
	.select = inter_dbg_log_chn_sel,
	.is_freq_valid = inter_dbg_log_is_freq_valid,
	.get_valid_channel = inter_dbg_log_get_valid_channel,
};

static int dbg_log_probe(struct platform_device *pdev)
{
	struct resource r;
	void __iomem *addr;
	struct dbg_log_device *dbg;
	struct regmap *aon_apb;
	struct regmap *dsi_apb;
	struct regmap *pll_apb;
	phys_addr_t serdes_apb;
	int count;
	int val;
	int i;
	int rc;

	pr_debug("entry\n");

	if (of_address_to_resource(pdev->dev.of_node, 0, &r) != 0) {
		pr_err("can't get serdes register base address\n");
		return -EINVAL;
	}

	addr = ioremap_nocache(r.start, resource_size(&r));
	if (IS_ERR(addr)) {
		pr_err("serdes_apb ioremap failed!\n");
		return PTR_ERR(addr);
	}

	pr_debug("0x%lx 0x%lx, 0x%lx\n", (unsigned long)(r.start),
		 (unsigned long)(r.end), (unsigned long)addr);

	serdes_apb = (phys_addr_t)addr;

	aon_apb = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						  "sprd,syscon-aon-apb");
	if (IS_ERR(aon_apb)) {
		pr_err("aon apb get failed!\n");
		return PTR_ERR(aon_apb);
	}

	dsi_apb = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						  "sprd,syscon-dsi-apb");
	if (IS_ERR(dsi_apb)) {
		pr_err("dsi apb get failed!\n");
		return PTR_ERR(dsi_apb);
	}

	pll_apb = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						  "sprd,syscon-pll-apb");
	if (IS_ERR(pll_apb)) {
		pr_err("pll apb get failed!\n");
		return PTR_ERR(pll_apb);
	}

	dbg = dbg_log_device_register(&pdev->dev, &ops, NULL);

	if (NULL != dbg) {
		dbg->aon_apb = aon_apb;
		dbg->phy->freq = 1500000;
		dbg->phy->dsi_apb = dsi_apb;
		dbg->phy->pll_apb = pll_apb;
		dbg->serdes.base = (void *)serdes_apb;
		dbg->serdes.cut_off = 0x20;
		rc = of_property_read_u32(pdev->dev.of_node, "sprd,mm", &val);
		pr_debug("mm %d\n", val);
		if (!rc)
			dbg->mm = val;

		rc = of_property_read_u32(pdev->dev.of_node, "sprd,dcfix",
					  &val);
		pr_debug("dcfix %d\n", val);
		if (!rc)
			dbg->serdes.dc_blnc_fix = val;

		rc = of_property_read_u32(pdev->dev.of_node, "sprd,ch_num",
					  &count);
		pr_debug("ch_num %d\n", count);

		if ((count > 0) && (count < CH_MAX)) {
			dbg->serdes.ch_num = count;
			rc = of_property_read_u32_array(pdev->dev.of_node,
							"sprd,ch_sel",
							dbg->serdes.ch_map,
							count);
			pr_debug("sel count %d, rc %d\n", count, rc);
			if (rc) {
				pr_err("get channel map failed\n");
				dbg->serdes.ch_num = 0;
			}
			for (i = 0; i < count; i++) {
				pr_debug("sel %d = 0x%x\n", i,
					 dbg->serdes.ch_map[i]);
			}

			rc = of_property_read_string_array(pdev->dev.of_node,
							   "sprd,ch_str",
							   dbg->serdes.ch_str,
							   count);
			pr_debug("str count %d, rc %d\n", count, rc);
			if (rc != count) {
				pr_err("get channel string failed\n");
				dbg->serdes.ch_num = 0;
			}
			for (i = 0; i < count; i++)
				pr_debug("str %d = %s\n", i,
					 dbg->serdes.ch_str[i]);
		}

		dbg->clk_serdes_eb = devm_clk_get(&pdev->dev, "serdes_eb");
		if (IS_ERR(dbg->clk_serdes_eb)) {
			dev_warn(&pdev->dev,
				 "can't get the clock dts config: serdes_eb\n");
			dbg->clk_serdes_eb = NULL;
		}

		count = 0;
		for (i = 0; i < CLK_SRC_MAX; i++) {
			char src_str[8];

			snprintf(src_str, 8, "src%d", i);
			dbg->clk_src[i] = devm_clk_get(&pdev->dev, src_str);
			if (IS_ERR(dbg->clk_src[i])) {
				dev_warn(&pdev->dev,
					 "can't get the clock dts config: %s\n",
					 src_str);
				dbg->clk_src[i] = NULL;
			} else {
				count++;
			}
		}

		rc = of_property_read_u32_array(pdev->dev.of_node,
						"sprd,div1_map",
						dbg->phy->div1_map, count);
		pr_debug("div1 map count %d, rc %d\n", count, rc);
		if (rc)
			pr_err("get div1 map failed\n");

		inter_dbg_log_is_freq_valid(dbg, dbg->phy->freq);
	} else {
		return -ENOMEM;
	}

	return 0;
}

static const struct of_device_id dt_ids[] = {
	{.compatible = "sprd,dbg-log-r4p2",},
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
MODULE_AUTHOR("Ken Kuang <ken.kuang@spreadtrum.com>");
MODULE_DESCRIPTION("Spreadtrum SoC Modem Debug Log Driver");

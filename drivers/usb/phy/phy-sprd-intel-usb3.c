/**
 * phy-sprd-intel-usb3.c - Spreadtrum USB3 PHY(Intel) Glue layer
 *
 * Copyright (c) 2016 Spreadtrum Co., Ltd.
 *      http://www.spreadtrum.com
 *
 * Author: Cheng Zeng <cheng.zeng@spreadtrum.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/usb/phy.h>
#include <linux/usb/sprd_usb.h>

#define REG_SIDECLK_JITSET			(0x0004)
#define REG_SIDECLK_OFFSET			(0x00ac)
#define REG_SIDECLK_EYE1			(0x0400)
#define REG_SIDECLK_EYE2			(0x0498)
#define REG_SIDECLK_USBBW			(0xc020)
#define REG_SIDECLK_OSCCNT			(0xc050)
#define REG_SIDECLK_JITEYE			(0xc0b0)
#define REG_SIDECLK_TSHOLD			(0xfc10)
#define BIT_SIDECLK_GLITCH_FIX			BIT(23)

#define SIDECLK_JITSET_VALUE			(0xd3d8006)
#define SIDECLK_OFFSET_VALUE1			(0x2a7805)
#define SIDECLK_OFFSET_VALUE2			(0xea7805)
#define SIDECLK_EYE1_VALUE			(0x5e091)
#define SIDECLK_EYE2_VALUE			(0x1829248)
#define SIDECLK_USBBW_VALUE			(0xa22b1800)
#define SIDECLK_OSCCNT_VALUE			(0x59e92618)
#define SIDECLK_JITEYE_VALUE			(0x8ec200)
#define SIDECLK_TSHOLD_VALUE			(0x8480)

#define REG_PHYRESET_OFFSET			(0x0034)
#define BIT_PHYRESET_SELECT			BIT(13)
#define BIT_U2PHY_P1_SUSPENDM			BIT(22)
#define BIT_U2PHY_P0_SUSPENDM			BIT(23)

#define REG_PHYCLEAR_OFFSET			(0x000c)
#define BIT_PHYCLEAR_MAINR			BIT(1)

#define REG_PHYUTMI_OFFSET			(0x0004)
#define BIT_PHYRESET_HSDIS			BIT(7)

struct sprd_intelphy {
	struct usb_phy      phy;
	void __iomem        *base;
	void __iomem        *sideclk;
	void __iomem        *phy_rst;
	struct regmap       *axi_rst;
	struct regmap       *apb_u2ctrl;
	struct regmap       *anlg_phy_g4;
	struct regulator    *vdd;
	struct regulator    *ldo;
	uint32_t            vdd_vol;
	uint32_t            revision;
#define TSMC_REVISION_28NM_HPM      0x5533286e
#define TSMC_REVISION_16NM_FFPLL    0x5533166e

	atomic_t    reset;      /* Core reset flag */
	atomic_t    inited;     /* PHY init flag */
	atomic_t    susped;     /* PHY suspend flag */
};

/* Reset USB Core */
static inline void __reset_core(struct regmap *addr)
{
	regmap_update_bits(addr, REG_AP_AHB_AHB_RST,
			BIT_AP_AHB_USB3_SOFT_RST, BIT_AP_AHB_USB3_SOFT_RST);
	msleep(5);
	regmap_update_bits(addr, REG_AP_AHB_AHB_RST,
			BIT_AP_AHB_USB3_SOFT_RST, 0);
}

static int sprd_intelphy_init(struct usb_phy *x)
{
	struct sprd_intelphy *phy = container_of(x, struct sprd_intelphy, phy);
	uint32_t reg;
	int ret;

	if (atomic_read(&phy->inited)) {
		dev_dbg(x->dev, "%s is already inited!\n", __func__);
		return 0;
	}

	/* Power on USB2PHY */
	reg = readl_relaxed(phy->phy_rst + REG_PHYRESET_OFFSET);
	reg &= ~(BIT_U2PHY_P0_SUSPENDM | BIT_U2PHY_P1_SUSPENDM);
	writel_relaxed(reg, phy->phy_rst + REG_PHYRESET_OFFSET);
	regmap_update_bits(phy->apb_u2ctrl,
			REG_COM_PMU_APB_USB2PHY_PS_CTRL,
			BIT_COM_PMU_APB_USB2PHY_PWR_ON,
			BIT_COM_PMU_APB_USB2PHY_PWR_ON);

	/* Turn On VDD */
	regulator_set_voltage(phy->vdd, phy->vdd_vol, phy->vdd_vol);

	/* Due to chip design, some chips may turn on vddusb by default,
	* We MUST avoid turning it on twice
	*/
	if (!regulator_is_enabled(phy->vdd)) {
		ret = regulator_enable(phy->vdd);
		if (ret < 0)
			dev_dbg(x->dev, "regulator intelphy vdd fail!\n");
	}

	/*Enable PHY LDO*/
	if (!IS_ERR_OR_NULL(phy->ldo)) {
		ret = regulator_enable(phy->ldo);
		if (ret < 0)
			dev_dbg(x->dev, "regulator intelphy ldo fail!\n");
	} else {
		dev_err(x->dev, "intelphy phy->ldo is error!\n");
	}

	/* Reset Core and PHY */
	__reset_core(phy->axi_rst);
	atomic_set(&phy->reset, 1);

	/* Set PowerPresent */
	regmap_update_bits(phy->anlg_phy_g4,
			REG_ANLG_PHY_G4_ANALOG_USB3_WRAP_GLUE_CTRL,
			BIT_ANLG_PHY_G4_ANALOG_USB3_USB3_POWER_PRESENT,
			BIT_ANLG_PHY_G4_ANALOG_USB3_USB3_POWER_PRESENT);

	/* Set Sideclk Register */
	reg = readl_relaxed(phy->sideclk + REG_SIDECLK_OFFSET);
	reg |= BIT_SIDECLK_GLITCH_FIX;
	writel_relaxed(reg, phy->sideclk + REG_SIDECLK_OFFSET);

	/* 2.0 Jitter Configuration */
	writel_relaxed(SIDECLK_OFFSET_VALUE1,
			phy->sideclk + REG_SIDECLK_OFFSET);

	regmap_write(phy->anlg_phy_g4,
			REG_ANLG_PHY_G4_ANALOG_USB2_WRAP_GLUE_CTRL,
			BIT_ANLG_PHY_G4_ANALOG_USB2_USB2_REF_DIV_CGM_EN);
	regmap_write(phy->anlg_phy_g4,
			REG_ANLG_PHY_G4_ANALOG_USB2_WRAP_GLUE_CTRL,
			BIT_ANLG_PHY_G4_ANALOG_USB2_USB2_SOFT_RSTN |
			BIT_ANLG_PHY_G4_ANALOG_USB2_USB2_REF_DIV_CGM_EN);
	regmap_write(phy->anlg_phy_g4,
			REG_ANLG_PHY_G4_ANALOG_USB2_RESET_CTRL_0,
			BIT_ANLG_PHY_G4_ANALOG_USB2_IUSB2PHY_FTAP_TRST_B);
	regmap_write(phy->anlg_phy_g4,
			REG_ANLG_PHY_G4_ANALOG_USB2_REG_SEL_CFG_0,
			BIT_ANLG_PHY_G4_DBG_SEL_ANALOG_USB2_IUSB2PHY_APB_RST_B);
	regmap_write(phy->anlg_phy_g4,
			REG_ANLG_PHY_G4_ANALOG_USB2_RESET_CTRL_0,
			BIT_ANLG_PHY_G4_ANALOG_USB2_IUSB2PHY_FTAP_TRST_B |
			BIT_ANLG_PHY_G4_ANALOG_USB2_IUSB2PHY_APB_RST_B);
	regmap_write(phy->anlg_phy_g4,
			REG_ANLG_PHY_G4_ANALOG_USB2_REG_SEL_CFG_0, 0);

	writel_relaxed(SIDECLK_OSCCNT_VALUE,
			phy->sideclk + REG_SIDECLK_OSCCNT);
	writel_relaxed(SIDECLK_USBBW_VALUE,
			phy->sideclk + REG_SIDECLK_USBBW);
	writel_relaxed(SIDECLK_EYE2_VALUE,
			phy->sideclk + REG_SIDECLK_EYE2);
	writel_relaxed(SIDECLK_JITEYE_VALUE,
			phy->sideclk + REG_SIDECLK_JITEYE);
	writel_relaxed(SIDECLK_EYE1_VALUE,
			phy->sideclk + REG_SIDECLK_EYE1);
	writel_relaxed(SIDECLK_OFFSET_VALUE2,
			phy->sideclk + REG_SIDECLK_OFFSET);
	writel_relaxed(SIDECLK_JITSET_VALUE,
			phy->sideclk + REG_SIDECLK_JITSET);
	writel_relaxed(SIDECLK_TSHOLD_VALUE,
			phy->sideclk + REG_SIDECLK_TSHOLD);

	atomic_set(&phy->inited, 1);

	return 0;
}

/* PHY Main reset */
static int sprd_intelphy_reset(struct usb_phy *x)
{
	struct sprd_intelphy *phy = container_of(x, struct sprd_intelphy, phy);
	uint32_t reg;

	reg = readl_relaxed(phy->phy_rst + REG_PHYUTMI_OFFSET);
	if (reg & BIT_PHYRESET_HSDIS) {
		reg = readl_relaxed(phy->phy_rst + REG_PHYRESET_OFFSET);
		reg |= BIT_PHYRESET_SELECT;
		writel_relaxed(reg, phy->phy_rst + REG_PHYRESET_OFFSET);

		reg = readl_relaxed(phy->phy_rst + REG_PHYCLEAR_OFFSET);
		reg |= BIT_PHYCLEAR_MAINR;
		writel_relaxed(reg, phy->phy_rst + REG_PHYCLEAR_OFFSET);

		reg = readl_relaxed(phy->phy_rst + REG_PHYCLEAR_OFFSET);
		reg &= (~BIT_PHYCLEAR_MAINR);
		writel_relaxed(reg, phy->phy_rst + REG_PHYCLEAR_OFFSET);

		reg = readl_relaxed(phy->phy_rst + REG_PHYRESET_OFFSET);
		reg &= (~BIT_PHYRESET_SELECT);
		writel_relaxed(reg, phy->phy_rst + REG_PHYRESET_OFFSET);

		dev_dbg(x->dev, "%s\n", __func__);
	}
	return 0;
}


static int sprd_intelphy_post_init(struct usb_phy *x)
{
	struct sprd_intelphy *phy = container_of(x, struct sprd_intelphy, phy);

	/* clear PowerPresent */
	regmap_update_bits(phy->anlg_phy_g4,
			REG_ANLG_PHY_G4_ANALOG_USB3_WRAP_GLUE_CTRL,
			BIT_ANLG_PHY_G4_ANALOG_USB3_USB3_POWER_PRESENT, 0);

	/* Disable LDO of super-speed channel */
	typec_disable_ldo();

	return 0;
}

/* Turn off PHY and core */
static void sprd_intelphy_shutdown(struct usb_phy *x)
{
	struct sprd_intelphy *phy = container_of(x, struct sprd_intelphy, phy);
	uint32_t reg;

	if (!atomic_read(&phy->inited)) {
		dev_dbg(x->dev, "%s is already shut down\n", __func__);
		return;
	}

	/* Power off USB2PHY */
	reg = readl_relaxed(phy->phy_rst + REG_PHYRESET_OFFSET);
	reg |= (BIT_U2PHY_P0_SUSPENDM | BIT_U2PHY_P1_SUSPENDM);
	writel_relaxed(reg, phy->phy_rst + REG_PHYRESET_OFFSET);
	regmap_update_bits(phy->apb_u2ctrl, REG_COM_PMU_APB_USB2PHY_PS_CTRL,
			BIT_COM_PMU_APB_USB2PHY_PWR_ON, 0);

	/* clear PowerPresent */
	regmap_update_bits(phy->anlg_phy_g4,
			REG_ANLG_PHY_G4_ANALOG_USB3_WRAP_GLUE_CTRL,
			BIT_ANLG_PHY_G4_ANALOG_USB3_USB3_POWER_PRESENT, 0);

	/* Due to chip design, some chips may turn on vddusb by default,
	* We MUST avoid turning it off twice
	*/
	if (regulator_is_enabled(phy->vdd))
		regulator_disable(phy->vdd);

	if (!IS_ERR_OR_NULL(phy->ldo))
		regulator_disable(phy->ldo);
	else
		dev_warn(x->dev, "intelphy phy->ldo warn!\n");

	atomic_set(&phy->inited, 0);
	atomic_set(&phy->reset, 0);
}

static ssize_t vdd_voltage_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct usb_phy *x = dev_get_drvdata(dev);
	struct sprd_intelphy *phy;

	if (!x)
		return -EINVAL;

	phy = container_of(x, struct sprd_intelphy, phy);

	return sprintf(buf, "%d\n", phy->vdd_vol);
}

static ssize_t vdd_voltage_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct usb_phy *x = dev_get_drvdata(dev);
	struct sprd_intelphy *phy;
	unsigned int vol;

	if (!x)
		return -EINVAL;

	phy = container_of(x, struct sprd_intelphy, phy);
	if (kstrtouint(buf, 16, &vol) < 0)
		return -EINVAL;

	if (vol < 1200000 || vol > 3750000) {
		dev_err(dev, "Invalid voltage value %d, "
			"a valid voltage should be 1v2 ~ 3v75", vol);
		return -EINVAL;
	}
	phy->vdd_vol = vol;

	return size;
}
static DEVICE_ATTR_RW(vdd_voltage);

static struct attribute *usb_intelphy_attrs[] = {
	&dev_attr_vdd_voltage.attr,
	NULL
};
ATTRIBUTE_GROUPS(usb_intelphy);

static int sprd_intelphy_probe(struct platform_device *pdev)
{
	struct sprd_intelphy *phy;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret = 0;

	phy = devm_kzalloc(dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev,
			IORESOURCE_MEM, "phy_glb_regs");
	if (!res) {
		dev_err(dev, "missing USB PHY global registers resource\n");
		return -ENODEV;
	}

	phy->base = devm_ioremap_nocache(dev, res->start, resource_size(res));
	if (IS_ERR_OR_NULL(phy->base))
		return PTR_ERR(phy->base);

	res = platform_get_resource_byname(pdev,
			IORESOURCE_MEM, "phy_sideclk_regs");
	if (!res) {
		dev_err(dev, "missing USB PHY sideclk registers resource\n");
		return -ENODEV;
	}

	phy->sideclk = devm_ioremap_nocache(dev,
			res->start, resource_size(res));
	if (IS_ERR_OR_NULL(phy->sideclk))
		return PTR_ERR(phy->sideclk);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
		"phy_reset_regs");
	if (!res) {
		dev_err(dev, "missing USB PHY phy_rst registers resource\n");
		return -ENODEV;
	}

	phy->phy_rst = devm_ioremap_nocache(dev, res->start,
		resource_size(res));
	if (IS_ERR_OR_NULL(phy->phy_rst))
		return PTR_ERR(phy->phy_rst);

	phy->axi_rst = syscon_regmap_lookup_by_phandle(dev->of_node,
		"sprd,syscon-ap-ahb");
	if (IS_ERR_OR_NULL(phy->axi_rst)) {
		dev_err(dev, "failed to map PMU registers (via syscon)\n");
		return PTR_ERR(phy->axi_rst);
	}

	phy->apb_u2ctrl = syscon_regmap_lookup_by_phandle(dev->of_node,
		"sprd,syscon-aon-com-pmu-apb");
	if (IS_ERR_OR_NULL(phy->apb_u2ctrl)) {
		dev_err(dev, "failed to map com PMU registers (via syscon)\n");
		return PTR_ERR(phy->apb_u2ctrl);
	}

	phy->anlg_phy_g4 = syscon_regmap_lookup_by_phandle(dev->of_node,
		"sprd,syscon-anlg-phy-g4");
	if (IS_ERR_OR_NULL(phy->anlg_phy_g4)) {
		dev_err(dev, "failed to map analog PHY g4 registers (via syscon)\n");
		return PTR_ERR(phy->anlg_phy_g4);
	}

	ret = of_property_read_u32(dev->of_node, "revision", &phy->revision);
	if (ret < 0) {
		dev_err(dev, "unable to read platform data phy revision\n");
		return ret;
	}
	if (phy->revision != TSMC_REVISION_28NM_HPM &&
		phy->revision != TSMC_REVISION_16NM_FFPLL) {
		dev_err(dev, "unknown phy revision\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(dev->of_node, "sprd,vdd-voltage",
			&phy->vdd_vol);
	if (ret < 0) {
		dev_err(dev, "unable to read platform data intelphy vdd voltage\n");
		return ret;
	}
#ifndef CONFIG_USB_SPRD_FPGA
	phy->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR_OR_NULL(phy->vdd)) {
		dev_err(dev, "unable to get intelphy vdd supply\n");
		return PTR_ERR(phy->vdd);
	}

	ret = regulator_set_voltage(phy->vdd, phy->vdd_vol, phy->vdd_vol);
	if (ret < 0) {
		dev_err(dev, "fail to set intelphy vdd voltage at %dmV\n",
			phy->vdd_vol);
		return ret;
	}

	phy->ldo = devm_regulator_get(dev, "phy-ldo");
	if (IS_ERR_OR_NULL(phy->ldo))
		dev_warn(dev, "unable to get intelphy phy-ldo supply\n");
#endif

	platform_set_drvdata(pdev, phy);
	phy->phy.dev = dev;
	phy->phy.label = "sprd-intelphy";
	if (IS_ENABLED(CONFIG_USB_SPRD_FPGA)) {
		phy->phy.init = NULL;
		phy->phy.post_init = NULL;
		phy->phy.shutdown = NULL;
	} else {
		phy->phy.init = sprd_intelphy_init;
		phy->phy.post_init = sprd_intelphy_post_init;
		phy->phy.shutdown = sprd_intelphy_shutdown;
		phy->phy.reset_phy = sprd_intelphy_reset;
	}
	phy->phy.type = USB_PHY_TYPE_USB3;

	ret = usb_add_phy_dev(&phy->phy);
	if (ret) {
		dev_err(dev, "fail to add phy\n");
		return ret;
	}

	ret = sysfs_create_groups(&dev->kobj, usb_intelphy_groups);
	if (ret)
		dev_warn(dev, "failed to create usb intelphy attributes\n");

	return 0;
}

static int sprd_intelphy_remove(struct platform_device *pdev)
{
	struct sprd_intelphy *phy = platform_get_drvdata(pdev);

	usb_remove_phy(&phy->phy);
	regulator_disable(phy->vdd);

	return 0;
}

static const struct of_device_id sprd_intelphy_match[] = {
	{ .compatible = "sprd,intelphy" },
	{ },
};

MODULE_DEVICE_TABLE(of, sprd_intelphy_match);

static struct platform_driver sprd_intelphy_driver = {
	.probe      = sprd_intelphy_probe,
	.remove     = sprd_intelphy_remove,
	.driver     = {
		.name   = "sprd-intelphy",
		.of_match_table = sprd_intelphy_match,
	},
};

module_platform_driver(sprd_intelphy_driver);

MODULE_ALIAS("platform:sprd-intelphy");
MODULE_AUTHOR("Cheng Zeng <cheng.zeng@spreadtrum.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SPRD USB3 PHY(Intel) Glue Layer");


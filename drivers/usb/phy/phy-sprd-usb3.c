/**
 * phy-sprd-usb3.c - Spreadtrum USB3 PHY Glue layer
 *
 * Copyright (c) 2015 Spreadtrum Co., Ltd.
 *		http://www.spreadtrum.com
 *
 * Author: Miao Zhu <miao.zhu@spreadtrum.com>
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
#include <linux/usb/phy.h>

#define PHY_REG_BASE			(phy->base)

/* PHY registers offset */
#define RST_CTRL			(PHY_REG_BASE + 0x00)
#define PWR_CTRL			(PHY_REG_BASE + 0x08)
#define PHY_TUNE1			(PHY_REG_BASE + 0xe4)
#define PHY_TUNE2			(PHY_REG_BASE + 0xe8)
#define PHY_TEST			(PHY_REG_BASE + 0xec)
#define PHY_CTRL1			(PHY_REG_BASE + 0xf0)
#define PHY_CTRL2			(PHY_REG_BASE + 0xf4)
#define PHY_DBG1			(PHY_REG_BASE + 0xf8)
#define PHY_DBG2			(PHY_REG_BASE + 0xfc)
#define PHY_CTRL3			(PHY_REG_BASE + 0x214)

/* PHY registers bits */
#define CORE_SOFT_OFFSET		4
#define CORE_SOFT_RST			BIT(1)
#define PHY_SOFT_RST			BIT(1)
#define PHY_PS_PD			BIT(18)

/* USB3.0_PHY_TEST */
#define PIPEP_POWERPRESENT_CFG_SEL	BIT(15)
#define PIPEP_POWERPRESENT_REG		BIT(14)
#define POWERDOWN_HSP			BIT(2)
#define POWERDOWN_SSP			BIT(1)

/* USB3.0_PHY_CTRL1 */
#define FSEL(x)				(((x) & 0x3F) << 23)
#define MPLL_MULTIPLIER(x)		(((x) & 0x7F) << 16)
#define REF_CLKDIV2			BIT(15)
#define REF_SSP_EN			BIT(14)
#define SSC_REF_CLK_SEL(x)		((x) & 0x1FF)

/* USB3.0_PHY_CTRL3 */
#define DIGPWERENSSP			BIT(3)
#define DIGPWERENHSP0			BIT(2)
#define DIGOUTISOENSSP			BIT(1)
#define DIGOUTISOENHSP0			BIT(0)

struct sprd_ssphy {
	struct usb_phy		phy;
	void __iomem		*base;
	struct regmap		*axi_rst;
	struct regulator	*vdd;
	uint32_t		vdd_vol;
	uint32_t		phy_tune1;
	uint32_t		phy_tune2;
	uint32_t		revision;

#define TSMC_REVISION_28NM_HPM		0x5533286e
#define TSMC_REVISION_16NM_FFPLL	0x5533166e

	atomic_t		reset;
	atomic_t		inited;
	atomic_t		susped;
};

static inline void __reset_core(struct regmap *addr)
{
	uint32_t reg;

	regmap_read(addr, CORE_SOFT_OFFSET, &reg);
	reg |= CORE_SOFT_RST;
	regmap_write(addr, CORE_SOFT_OFFSET, reg);

	msleep(3);

	reg &= ~CORE_SOFT_RST;
	regmap_write(addr, CORE_SOFT_OFFSET, reg);
}

/* Reset USB Core */
static int sprd_ssphy_init(struct usb_phy *x)
{
	struct sprd_ssphy *phy = container_of(x, struct sprd_ssphy, phy);
	uint32_t reg, reg1;
	int ret;

	if (atomic_read(&phy->inited)) {
		dev_dbg(x->dev, "%s is already inited!\n", __func__);
		return 0;
	}

	/* Turn On VDD */
	regulator_set_voltage(phy->vdd, phy->vdd_vol, phy->vdd_vol);

	/* Reset PHY */
	reg = readl(RST_CTRL);
	reg |= PHY_SOFT_RST;
	writel(reg, RST_CTRL);

	/*
	 * Due to chip design, some chips may turn on vddusb by default,
	 * We MUST avoid turning it on twice.
	 */
	if (!regulator_is_enabled(phy->vdd))
		ret = regulator_enable(phy->vdd);

	/* Clear PHY PD */
	reg = readl(PWR_CTRL);
	reg &= ~PHY_PS_PD;
	writel(reg, PWR_CTRL);

	/* Restore PHY tunes */
	writel(phy->phy_tune1, PHY_TUNE1);
	writel(phy->phy_tune2, PHY_TUNE2);

	/* Set PipeP_PowerPresent */
	reg = readl(PHY_TEST);
	reg |= PIPEP_POWERPRESENT_CFG_SEL | PIPEP_POWERPRESENT_REG;
	writel(reg, PHY_TEST);

	/* Set PHY power on and isolation */
	if (phy->revision == TSMC_REVISION_16NM_FFPLL) {
		reg = readl(PHY_CTRL3);
		reg |= (DIGPWERENSSP | DIGPWERENHSP0);
		writel(reg, PHY_CTRL3);

		usleep_range(200, 300);

		reg1 = readl(PHY_TEST);
		reg1 |= POWERDOWN_HSP | POWERDOWN_SSP;
		writel(reg1, PHY_TEST);

		usleep_range(100, 200);

		reg &= ~(DIGOUTISOENSSP | DIGOUTISOENHSP0);
		writel(reg, PHY_CTRL3);

		usleep_range(100, 200);

		reg1 &= ~(POWERDOWN_HSP | POWERDOWN_SSP);
		writel(reg1, PHY_TEST);
	}
	msleep(5);

	/* Set SSP_EN and Clear CLK_DIV2 */
	reg = readl(PHY_CTRL1);
	reg |= REF_SSP_EN;
	reg &= ~REF_CLKDIV2;

	/*
	 * Switch 26MHz reference clock from the internal clock to the
	 * external one.
	 */
	if (phy->revision == TSMC_REVISION_16NM_FFPLL) {
		reg &= ~FSEL(0x3f);
		reg |= FSEL(0x2);
		reg |= MPLL_MULTIPLIER(0x60);
		reg |= SSC_REF_CLK_SEL(0x108);
	}
	writel(reg, PHY_CTRL1);

	/* Clear PHY Reset */
	reg = readl(RST_CTRL);
	reg &= ~PHY_SOFT_RST;
	writel(reg, RST_CTRL);

	if (!atomic_read(&phy->reset)) {
		msleep(5);
		__reset_core(phy->axi_rst);

		atomic_set(&phy->reset, 1);
	}

	atomic_set(&phy->inited, 1);

	return 0;
}

/* Turn off PHY and core */
static void sprd_ssphy_shutdown(struct usb_phy *x)
{
	struct sprd_ssphy *phy = container_of(x, struct sprd_ssphy, phy);
	uint32_t reg;

	if (!atomic_read(&phy->inited)) {
		dev_dbg(x->dev, "%s is already shut down\n", __func__);
		return;
	}

	/* Backup PHY Tune value */
	phy->phy_tune1 = readl(PHY_TUNE1);
	phy->phy_tune2 = readl(PHY_TUNE2);

	reg = readl(PWR_CTRL);
	reg |= PHY_PS_PD;
	writel(reg, PWR_CTRL);

	reg = readl(PHY_TEST);
	reg &= ~(PIPEP_POWERPRESENT_CFG_SEL | PIPEP_POWERPRESENT_REG);
	if (phy->revision == TSMC_REVISION_16NM_FFPLL)
		reg |= POWERDOWN_HSP | POWERDOWN_SSP;
	writel(reg, PHY_TEST);

	/*
	 * Due to chip design, some chips may turn on vddusb by default,
	 * We MUST avoid turning it off twice.
	 */
	if (regulator_is_enabled(phy->vdd))
		regulator_disable(phy->vdd);

	atomic_set(&phy->inited, 0);
	atomic_set(&phy->reset, 0);
}

static ssize_t phy_tune1_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct usb_phy *x = dev_get_drvdata(dev);
	struct sprd_ssphy *phy;

	if (!x)
		return -EINVAL;

	phy = container_of(x, struct sprd_ssphy, phy);

	return sprintf(buf, "0x%x\n", phy->phy_tune1);
}

static ssize_t phy_tune1_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct usb_phy *x = dev_get_drvdata(dev);
	struct sprd_ssphy *phy;

	if (!x)
		return -EINVAL;

	phy = container_of(x, struct sprd_ssphy, phy);
	if (kstrtouint(buf, 16, &phy->phy_tune1) < 0)
		return -EINVAL;

	return size;
}
static DEVICE_ATTR_RW(phy_tune1);

static ssize_t phy_tune2_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct usb_phy *x = dev_get_drvdata(dev);
	struct sprd_ssphy *phy;

	if (!x)
		return -EINVAL;

	phy = container_of(x, struct sprd_ssphy, phy);

	return sprintf(buf, "0x%x\n", phy->phy_tune2);
}

static ssize_t phy_tune2_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct usb_phy *x = dev_get_drvdata(dev);
	struct sprd_ssphy *phy;

	if (!x)
		return -EINVAL;

	phy = container_of(x, struct sprd_ssphy, phy);
	if (kstrtouint(buf, 16, &phy->phy_tune2) < 0)
		return -EINVAL;

	return size;
}
static DEVICE_ATTR_RW(phy_tune2);

static ssize_t vdd_voltage_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct usb_phy *x = dev_get_drvdata(dev);
	struct sprd_ssphy *phy;

	if (!x)
		return -EINVAL;

	phy = container_of(x, struct sprd_ssphy, phy);

	return sprintf(buf, "%d\n", phy->vdd_vol);
}

static ssize_t vdd_voltage_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct usb_phy *x = dev_get_drvdata(dev);
	struct sprd_ssphy *phy;
	unsigned int vol;

	if (!x)
		return -EINVAL;

	phy = container_of(x, struct sprd_ssphy, phy);
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

static struct attribute *usb_ssphy_attrs[] = {
	&dev_attr_phy_tune1.attr,
	&dev_attr_phy_tune2.attr,
	&dev_attr_vdd_voltage.attr,
	NULL
};
ATTRIBUTE_GROUPS(usb_ssphy);

static struct sprd_ssphy *__sprd_ssphy;

static int sprd_ssphy_probe(struct platform_device *pdev)
{
	struct sprd_ssphy *phy;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret = 0;

	phy = devm_kzalloc(dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	__sprd_ssphy = phy;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "phy_glb_regs");
	if (!res) {
		dev_err(dev, "missing USB PHY registers resource\n");
		return -ENODEV;
	}

	phy->base = devm_ioremap_nocache(dev, res->start, resource_size(res));
	if (IS_ERR(phy->base))
		return PTR_ERR(phy->base);

	phy->axi_rst = syscon_regmap_lookup_by_phandle(dev->of_node,
						       "sprd,syscon-ap-ahb");
	if (IS_ERR(phy->axi_rst)) {
		dev_err(dev, "failed to map PMU registers (via syscon)\n");
		return PTR_ERR(phy->axi_rst);
	}

	ret = of_property_read_u32(dev->of_node, "revision", &phy->revision);
	if (ret < 0) {
		dev_err(dev, "unable to read platform data phy revision\n");
		return ret;
	}
	if (phy->revision != TSMC_REVISION_28NM_HPM &&
		phy->revision != TSMC_REVISION_16NM_FFPLL) {
		dev_err(dev, "unknown phy revision\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "sprd,phy-tune1",
				   &phy->phy_tune1);
	if (ret < 0) {
		dev_err(dev, "unable to read ssphy usb phy tune1\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "sprd,phy-tune2",
				   &phy->phy_tune2);
	if (ret < 0) {
		dev_err(dev, "unable to read ssphy usb phy tune2\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "sprd,vdd-voltage",
				   &phy->vdd_vol);
	if (ret < 0) {
		dev_err(dev, "unable to read ssphy vdd voltage\n");
		return ret;
	}

	phy->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(phy->vdd)) {
		dev_err(dev, "unable to get ssphy vdd supply\n");
		return PTR_ERR(phy->vdd);
	}

	ret = regulator_set_voltage(phy->vdd, phy->vdd_vol, phy->vdd_vol);
	if (ret < 0) {
		dev_err(dev, "fail to set ssphy vdd voltage at %dmV\n",
			phy->vdd_vol);
		return ret;
	}

	platform_set_drvdata(pdev, phy);
	phy->phy.dev				= dev;
	phy->phy.label				= "sprd-ssphy";
	phy->phy.init				= sprd_ssphy_init;
	phy->phy.shutdown			= sprd_ssphy_shutdown;
	/*
	 * TODO:
	 * phy->phy.set_suspend			= sprd_ssphy_set_suspend;
	 * phy->phy.notify_connect		= sprd_ssphy_notify_connect;
	 * phy->phy.notify_disconnect		= sprd_ssphy_notify_disconnect;
	 * phy->phy.reset			= sprd_ssphy_reset;
	 */
	phy->phy.type				= USB_PHY_TYPE_USB3;

	ret = usb_add_phy_dev(&phy->phy);
	if (ret) {
		dev_err(dev, "fail to add phy\n");
		return ret;
	}

	ret = sysfs_create_groups(&dev->kobj, usb_ssphy_groups);
	if (ret)
		dev_err(dev, "failed to create usb ssphy attributes\n");

	return 0;
}

static int sprd_ssphy_remove(struct platform_device *pdev)
{
	struct sprd_ssphy *phy = platform_get_drvdata(pdev);

	sysfs_remove_groups(&pdev->dev.kobj, usb_ssphy_groups);
	usb_remove_phy(&phy->phy);
	regulator_disable(phy->vdd);
	kfree(phy);

	return 0;
}

static const struct of_device_id sprd_ssphy_match[] = {
	{ .compatible = "sprd,ssphy" },
	{},
};

MODULE_DEVICE_TABLE(of, sprd_ssphy_match);

static struct platform_driver sprd_ssphy_driver = {
	.probe		= sprd_ssphy_probe,
	.remove		= sprd_ssphy_remove,
	.driver		= {
		.name	= "sprd-ssphy",
		.of_match_table = sprd_ssphy_match,
	},
};

module_platform_driver(sprd_ssphy_driver);

MODULE_ALIAS("platform:sprd-ssphy	");
MODULE_AUTHOR("Miao Zhu <miao.zhu@spreadtrum.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 SPRD PHY Glue Layer");

/*
 * Copyright (C) 2016 Spreadtrum Communications Inc.
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
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/sprd_otp.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/usb/phy.h>
#include <linux/usb/otg.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/of_device.h>

#define PHY_REG_BASE	(phy->base)
#define OTG_PHY_TUNE_EFUSE_FLG BIT(30)

#ifndef REG_ANLG_PHY_TOP_ANALOG_TOP_REG_CTRL_2
#define REG_ANLG_PHY_TOP_ANALOG_TOP_REG_CTRL_2 0x10
#endif

#ifndef BIT_ANLG_PHY_TOP_USB20_ISO_SW_EN
#define BIT_ANLG_PHY_TOP_USB20_ISO_SW_EN BIT(0)
#endif

#ifndef BIT_OTG_PHY_SOFT_RST
#define BIT_OTG_PHY_SOFT_RST BIT_AP_AHB_OTG_PHY_SOFT_RST
#endif

#ifndef BIT_OTG_UTMI_SOFT_RST
#define BIT_OTG_UTMI_SOFT_RST BIT_AP_AHB_OTG_UTMI_SOFT_RST
#endif

#ifndef BIT_OTG_SOFT_RST
#define BIT_OTG_SOFT_RST BIT_AP_AHB_OTG_SOFT_RST
#endif

#define PIKE2_USB_PHY_ISOLATION	BIT(0)

struct phy_match_data {
	bool			iso_en;
	bool			iso_active_high;
	u32			iso_reg_bit;
};

struct sprd_hsphy {
	struct device		*dev;
	struct usb_phy		phy;
	void __iomem		*base;
	struct regulator	*vdd;
	struct regmap           *hsphy_glb;
	struct regmap           *hsphy_usbiso;
	uint32_t		vdd_vol;
	uint32_t		phy_tune;
	u32			efuse_blk_id;
	u32			efuse_backup;
	atomic_t		reset;
	atomic_t		inited;
	struct phy_match_data	*match_data;
};

static inline void __reset_core(void __iomem *addr)
{
	uint32_t reg;

	/* Reset PHY */
	reg = readl_relaxed(addr + REG_AP_AHB_AHB_RST);
	reg = BIT_OTG_PHY_SOFT_RST | BIT_OTG_UTMI_SOFT_RST |
			BIT_OTG_SOFT_RST;
	writel_relaxed(reg, addr + REG_AP_AHB_AHB_RST + 0x1000);
	msleep(20);
	writel_relaxed(reg, addr + REG_AP_AHB_AHB_RST + 0x2000);
}

static int sprd_hsphy_reset(struct usb_phy *x)
{
	struct sprd_hsphy *phy = container_of(x, struct sprd_hsphy, phy);

	__reset_core(PHY_REG_BASE);
	return 0;
}

static int sprd_hostphy_set(struct usb_phy *x, int on)
{
	struct sprd_hsphy *phy = container_of(x, struct sprd_hsphy, phy);
	uint32_t reg1;

	if (on) {
		reg1 = readl_relaxed(PHY_REG_BASE + REG_AP_AHB_OTG_PHY_CTRL);
		reg1 &= ~BIT_AP_AHB_USB2_PHY_IDDIG;
		reg1 |= BIT_AP_AHB_OTG_DPPULLDOWN | BIT_AP_AHB_OTG_DMPULLDOWN;
		writel_relaxed(reg1, PHY_REG_BASE + REG_AP_AHB_OTG_PHY_CTRL);

		reg1 = readl_relaxed(PHY_REG_BASE + REG_AP_AHB_OTG_CTRL0);
		reg1 |= BIT_AP_AHB_USB20_RESERVED(0x200);
		writel_relaxed(reg1, PHY_REG_BASE + REG_AP_AHB_OTG_CTRL0);
	} else {
		reg1 = readl_relaxed(PHY_REG_BASE + REG_AP_AHB_OTG_PHY_CTRL);
		reg1 |= BIT_AP_AHB_USB2_PHY_IDDIG;
		reg1 &= ~(BIT_AP_AHB_OTG_DPPULLDOWN |
			BIT_AP_AHB_OTG_DMPULLDOWN);
		writel_relaxed(reg1, PHY_REG_BASE + REG_AP_AHB_OTG_PHY_CTRL);

		reg1 = readl_relaxed(PHY_REG_BASE + REG_AP_AHB_OTG_CTRL0);
		reg1 &= ~(BIT_AP_AHB_USB20_RESERVED(0x200));
		writel_relaxed(reg1, PHY_REG_BASE + REG_AP_AHB_OTG_CTRL0);
	}
	return 0;
}

static void sprd_hsphy_emphasis_set(struct usb_phy *x, bool enabled)
{
	struct sprd_hsphy *phy = container_of(x, struct sprd_hsphy, phy);
	u32 reg;

	if (!phy)
		return;
	reg = readl_relaxed(PHY_REG_BASE + REG_AP_AHB_OTG_CTRL0);
	reg &= ~BIT_AP_AHB_USB20_TUNEHSAMP(3);
	if (enabled) {
		reg |= BIT_AP_AHB_USB20_TUNEHSAMP(2);
		writel_relaxed(reg, PHY_REG_BASE + REG_AP_AHB_OTG_CTRL0);
	} else
		writel_relaxed(reg, PHY_REG_BASE + REG_AP_AHB_OTG_CTRL0);
}

static int sprd_hsphy_init(struct usb_phy *x)
{
	struct sprd_hsphy *phy = container_of(x, struct sprd_hsphy, phy);
	struct phy_match_data *match_data = phy->match_data;
	uint32_t reg, reg1;
	int ret;

	if (atomic_read(&phy->inited)) {
		dev_dbg(x->dev, "%s is already inited!\n", __func__);
		return 0;
	}

	/* Turn On VDD */
	regulator_set_voltage(phy->vdd, phy->vdd_vol, phy->vdd_vol);

	if (!regulator_is_enabled(phy->vdd))
		ret = regulator_enable(phy->vdd);

	if (phy->hsphy_usbiso != NULL) {
		regmap_update_bits(phy->hsphy_usbiso, REG_ANLG_PHY_TOP_ANALOG_TOP_REG_CTRL_2,
			BIT_ANLG_PHY_TOP_USB20_ISO_SW_EN, 0);
	}

	if (match_data->iso_en == true) {
		if (match_data->iso_active_high == false) {
			reg = readl_relaxed(PHY_REG_BASE +
					REG_AP_AHB_OTG_CTRL0);
			reg |= match_data->iso_reg_bit;
			writel_relaxed(reg, PHY_REG_BASE +
					REG_AP_AHB_OTG_CTRL0);
		} else {
			reg = readl_relaxed(PHY_REG_BASE +
					REG_AP_AHB_OTG_CTRL0);
			reg &= ~match_data->iso_reg_bit;
			writel_relaxed(reg, PHY_REG_BASE +
					REG_AP_AHB_OTG_CTRL0);
		}
	}

	regmap_update_bits(phy->hsphy_glb, REG_AON_APB_PWR_CTRL,
		BIT_AON_APB_USB_PHY_PD_L, 0);
	regmap_update_bits(phy->hsphy_glb, REG_AON_APB_PWR_CTRL,
		BIT_AON_APB_USB_PHY_PD_S, 0);
	/* Restore PHY tunes */
	writel_relaxed(phy->phy_tune, PHY_REG_BASE + REG_AP_AHB_OTG_PHY_TUNE);

	mdelay(2);

	/* usb vbus valid */
	reg = readl_relaxed(PHY_REG_BASE + REG_AP_AHB_OTG_PHY_TEST);
	reg |= (BIT_AP_AHB_OTG_VBUS_VALID_EXT |
		 BIT_AP_AHB_OTG_VBUS_VALID_PHYREG);
	writel_relaxed(reg, PHY_REG_BASE + REG_AP_AHB_OTG_PHY_TEST);

	/* for SPRD phy utmi_width sel */
	reg1 = readl_relaxed(PHY_REG_BASE + REG_AP_AHB_OTG_PHY_CTRL);
	reg1 &= ~(BIT_AP_AHB_UTMI_WIDTH_SEL|BIT_AP_AHB_USB2_DATABUS16_8);
	writel_relaxed(reg1, PHY_REG_BASE + REG_AP_AHB_OTG_PHY_CTRL);

	reg1 = readl_relaxed(PHY_REG_BASE + REG_AP_AHB_OTG_CTRL0);
	reg1 &= ~BIT_AP_AHB_USB20_TUNEHSAMP(3);
	reg1 |= BIT_AP_AHB_USB20_TUNEHSAMP(2);
	writel_relaxed(reg1, PHY_REG_BASE + REG_AP_AHB_OTG_CTRL0);

	/* for SPRD phy sampler sel */
	reg1 = readl_relaxed(PHY_REG_BASE + REG_AP_AHB_OTG_CTRL1);
	reg1 |= BIT_AP_AHB_USB20_SAMPLER_SEL;
	writel_relaxed(reg1, PHY_REG_BASE + REG_AP_AHB_OTG_CTRL1);

	if (!atomic_read(&phy->reset)) {
		msleep(20);
		__reset_core(PHY_REG_BASE);
		atomic_set(&phy->reset, 1);
	}

	atomic_set(&phy->inited, 1);

	return 0;
}

static void sprd_hsphy_shutdown(struct usb_phy *x)
{
	struct sprd_hsphy *phy = container_of(x, struct sprd_hsphy, phy);
	struct phy_match_data *match_data = phy->match_data;
	uint32_t reg;

	if (!atomic_read(&phy->inited)) {
		dev_dbg(x->dev, "%s is already shut down\n", __func__);
		return;
	}

	reg = readl_relaxed(PHY_REG_BASE + REG_AP_AHB_OTG_PHY_TEST);
	reg &= ~(BIT_AP_AHB_OTG_VBUS_VALID_PHYREG |
		BIT_AP_AHB_OTG_VBUS_VALID_EXT);
	writel_relaxed(reg, PHY_REG_BASE + REG_AP_AHB_OTG_PHY_TEST);

	/* Backup PHY Tune value */
	phy->phy_tune = readl(PHY_REG_BASE + REG_AP_AHB_OTG_PHY_TUNE);

	regmap_update_bits(phy->hsphy_glb, REG_AON_APB_PWR_CTRL,
		BIT_AON_APB_USB_PHY_PD_L, BIT_AON_APB_USB_PHY_PD_L);
	regmap_update_bits(phy->hsphy_glb, REG_AON_APB_PWR_CTRL,
		BIT_AON_APB_USB_PHY_PD_S, BIT_AON_APB_USB_PHY_PD_S);

	if (phy->hsphy_usbiso != NULL) {
		regmap_update_bits(phy->hsphy_usbiso, REG_ANLG_PHY_TOP_ANALOG_TOP_REG_CTRL_2,
			BIT_ANLG_PHY_TOP_USB20_ISO_SW_EN, BIT_ANLG_PHY_TOP_USB20_ISO_SW_EN);
	}

	if (match_data->iso_en == true) {
		if (match_data->iso_active_high == false) {
			reg = readl_relaxed(PHY_REG_BASE +
					REG_AP_AHB_OTG_CTRL0);
			reg &= ~match_data->iso_reg_bit;
			writel_relaxed(reg, PHY_REG_BASE +
					REG_AP_AHB_OTG_CTRL0);
		} else {
			reg = readl_relaxed(PHY_REG_BASE +
					REG_AP_AHB_OTG_CTRL0);
			reg |= match_data->iso_reg_bit;
			writel_relaxed(reg, PHY_REG_BASE +
					REG_AP_AHB_OTG_CTRL0);
		}
	}

	if (regulator_is_enabled(phy->vdd))
		regulator_disable(phy->vdd);

	atomic_set(&phy->inited, 0);
	atomic_set(&phy->reset, 0);
}

static int sprd_hsphy_post_init(struct usb_phy *x)
{
	struct sprd_hsphy *phy = container_of(x, struct sprd_hsphy, phy);

	if (regulator_is_enabled(phy->vdd))
		regulator_disable(phy->vdd);
	return 0;
}

static ssize_t phy_tune_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct usb_phy *x = dev_get_drvdata(dev);
	struct sprd_hsphy *phy;

	if (!x)
		return -EINVAL;

	phy = container_of(x, struct sprd_hsphy, phy);

	return sprintf(buf, "0x%x\n", phy->phy_tune);
}

static ssize_t phy_tune_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct usb_phy *x = dev_get_drvdata(dev);
	struct sprd_hsphy *phy;

	if (!x)
		return -EINVAL;

	phy = container_of(x, struct sprd_hsphy, phy);
	if (kstrtouint(buf, 16, &phy->phy_tune) < 0)
		return -EINVAL;

	return size;
}
static DEVICE_ATTR_RW(phy_tune);

static ssize_t vdd_voltage_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct usb_phy *x = dev_get_drvdata(dev);
	struct sprd_hsphy *phy;

	if (!x)
		return -EINVAL;

	phy = container_of(x, struct sprd_hsphy, phy);

	return sprintf(buf, "%d\n", phy->vdd_vol);
}

static ssize_t vdd_voltage_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct usb_phy *x = dev_get_drvdata(dev);
	struct sprd_hsphy *phy;
	unsigned int vol;

	if (!x)
		return -EINVAL;

	phy = container_of(x, struct sprd_hsphy, phy);
	if (kstrtouint(buf, 16, &vol) < 0)
		return -EINVAL;

	if (vol < 1200000 || vol > 3750000) {
		dev_err(dev, "Invalid voltage value %d\n", vol);
		return -EINVAL;
	}
	phy->vdd_vol = vol;

	return size;
}
static DEVICE_ATTR_RW(vdd_voltage);

static struct attribute *usb_hsphy_attrs[] = {
	&dev_attr_phy_tune.attr,
	&dev_attr_vdd_voltage.attr,
	NULL
};
ATTRIBUTE_GROUPS(usb_hsphy);

static struct phy_match_data default_match_data = {
	.iso_en = false,
	.iso_active_high = true,
	.iso_reg_bit = 0,
};

static struct phy_match_data pike2_match_data = {
	.iso_en = true,
	.iso_active_high = false,
	.iso_reg_bit = PIKE2_USB_PHY_ISOLATION,
};

static const struct of_device_id sprd_hsphy_match[] = {
	{ .compatible = "sprd,usb-phy", .data = &default_match_data },
	{ .compatible = "sprd,pike2-usb-phy", .data = &pike2_match_data },
	{},
};

static int sprd_hsphy_probe(struct platform_device *pdev)
{
	struct sprd_hsphy *phy;
	struct device *dev = &pdev->dev;
	struct regmap *hsphy_glb = NULL;
	struct resource *res;
	int ret = 0;
	struct usb_otg *otg;
	const struct of_device_id *match;
	struct phy_match_data *match_data;

	match = of_match_device(sprd_hsphy_match, dev);
	match_data = (struct phy_match_data *)(match->data);

	phy = devm_kzalloc(dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "phy_glb_regs");
	if (!res) {
		dev_err(dev, "missing USB PHY registers resource\n");
		return -ENODEV;
	}

	phy->base = devm_ioremap_nocache(dev, res->start, resource_size(res));
	if (IS_ERR(phy->base))
		return PTR_ERR(phy->base);

	ret = of_property_read_u32(dev->of_node, "sprd,tune-value",
				   &phy->phy_tune);
	if (ret < 0) {
		dev_err(dev, "unable to read hsphy usb phy tune\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "sprd,vdd-voltage",
				   &phy->vdd_vol);
	if (ret < 0) {
		dev_err(dev, "unable to read ssphy vdd voltage\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "sprd,efuse-blk-id",
				   &phy->efuse_blk_id);
	if (ret < 0) {
#if defined(CONFIG_OTP_SPRD_AP_EFUSE)
		phy->efuse_blk_id = 7;
#elif defined(CONFIG_OTP_SPRD_AP_PUBLIC_EFUSE)
		phy->efuse_blk_id = 45;
#endif
		dev_info(dev, "default efuse blk id is %d\n",
				phy->efuse_blk_id);
	}

	ret = of_property_read_u32(dev->of_node, "sprd,efuse-backup",
				   &phy->efuse_backup);
	if (ret < 0) {
		phy->efuse_backup = 1;
		dev_info(dev, "default efuse backup is %d\n",
				phy->efuse_backup);
	}

	phy->match_data = match_data;

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

	otg = devm_kzalloc(&pdev->dev, sizeof(*otg), GFP_KERNEL);
	if (!otg)
		return -ENOMEM;

	hsphy_glb = syscon_regmap_lookup_by_phandle(dev->of_node,
				 "sprd,syscon-enable");
	if (IS_ERR(hsphy_glb)) {
		dev_err(&pdev->dev, "ap USB PHY syscon failed!\n");
		return -ENOMEM;
	}

	regmap_update_bits(hsphy_glb, REG_AON_APB_PWR_CTRL,
		BIT_AON_APB_USB_PHY_PD_L, BIT_AON_APB_USB_PHY_PD_L);
	regmap_update_bits(hsphy_glb, REG_AON_APB_PWR_CTRL,
		BIT_AON_APB_USB_PHY_PD_S, BIT_AON_APB_USB_PHY_PD_S);
	phy->hsphy_glb = hsphy_glb;

	hsphy_glb = syscon_regmap_lookup_by_phandle(dev->of_node,
				 "sprd,syscon-usb2iso");
	if (IS_ERR(hsphy_glb)) {
		phy->hsphy_usbiso = NULL;
	} else {
		regmap_update_bits(hsphy_glb, REG_ANLG_PHY_TOP_ANALOG_TOP_REG_CTRL_2,
			BIT_ANLG_PHY_TOP_USB20_ISO_SW_EN, BIT_ANLG_PHY_TOP_USB20_ISO_SW_EN);
		phy->hsphy_usbiso = hsphy_glb;
	}

	phy->phy.dev = dev;
	phy->phy.label = "sprd-hsphy";
	phy->phy.otg = otg;
	phy->phy.init = sprd_hsphy_init;
	phy->phy.shutdown = sprd_hsphy_shutdown;
	phy->phy.post_init = sprd_hsphy_post_init;
	phy->phy.reset_phy = sprd_hsphy_reset;
	phy->phy.set_vbus = sprd_hostphy_set;
	phy->phy.set_emphasis = sprd_hsphy_emphasis_set;
	phy->phy.type = USB_PHY_TYPE_USB2;
	otg->usb_phy = &phy->phy;

	platform_set_drvdata(pdev, phy);

	ret = usb_add_phy_dev(&phy->phy);
	if (ret) {
		dev_err(dev, "fail to add phy\n");
		return ret;
	}

	ret = sysfs_create_groups(&dev->kobj, usb_hsphy_groups);
	if (ret)
		dev_err(dev, "failed to create usb hsphy attributes\n");

	return 0;
}

static int sprd_hsphy_remove(struct platform_device *pdev)
{
	struct sprd_hsphy *phy = platform_get_drvdata(pdev);

	sysfs_remove_groups(&pdev->dev.kobj, usb_hsphy_groups);
	usb_remove_phy(&phy->phy);
	regulator_disable(phy->vdd);

	return 0;
}

MODULE_DEVICE_TABLE(of, sprd_ssphy_match);

static struct platform_driver sprd_hsphy_driver = {
	.probe	= sprd_hsphy_probe,
	.remove	= sprd_hsphy_remove,
	.driver	= {
		.name	= "sprd-hsphy",
		.of_match_table	= sprd_hsphy_match,
	},
};

module_platform_driver(sprd_hsphy_driver);

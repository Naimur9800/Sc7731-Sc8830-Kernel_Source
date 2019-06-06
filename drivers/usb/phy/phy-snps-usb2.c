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
#include <linux/mfd/syscon.h>
#include <linux/sprd_otp.h>
#include <linux/regmap.h>
#include <linux/usb/phy.h>
#include <linux/usb/otg.h>
#include <linux/mfd/syscon/sprd-glb.h>

#define OTG_PHY_EFUSE_BLOCK_ID 7
#define OTG_PHY_TUNE_EFUSE_FLG 0x40000000

struct sprd_hsphy {
	struct device		*dev;
	struct usb_phy		phy;
	struct regulator	*vdd;
	struct regmap           *hsphy_glb;
	struct regmap           *apahb;
	struct regmap           *ana_g2;
	uint32_t		vdd_vol;
	uint32_t		phy_tune;
	atomic_t		reset;
	atomic_t		inited;
};

static inline void __reset_core(struct sprd_hsphy *phy)
{
	uint32_t msk1, msk2;

	/* Reset PHY */
	msk1 = BIT_OTG_UTMI_SOFT_RST | BIT_OTG_SOFT_RST;
	msk2 = BIT_OTG_PHY_SOFT_RST;
	regmap_update_bits(phy->apahb, REG_AP_AHB_AHB_RST,
		msk1, msk1);
	regmap_update_bits(phy->hsphy_glb, REG_AON_APB_APB_RST2,
		msk2, msk2);
	msleep(20);
	regmap_update_bits(phy->apahb, REG_AP_AHB_AHB_RST,
		msk1, 0);
	regmap_update_bits(phy->hsphy_glb, REG_AON_APB_APB_RST2,
		msk2, 0);
}

static int sprd_hsphy_reset(struct usb_phy *x)
{
	struct sprd_hsphy *phy = container_of(x, struct sprd_hsphy, phy);

	__reset_core(phy);
	return 0;
}

static int sprd_hostphy_set(struct usb_phy *x, int on)
{
	struct sprd_hsphy *phy = container_of(x, struct sprd_hsphy, phy);
	uint32_t reg1 = 0, msk1 = 0;

	if (on) {
		msk1 = BIT_AP_AHB_USB2_PHY_IDDIG;
		regmap_hwlock_update_bits(phy->apahb,
			REG_AP_AHB_OTG_PHY_CTRL, msk1, 0);
	} else {
		reg1 = msk1 = BIT_AP_AHB_USB2_PHY_IDDIG;
		regmap_hwlock_update_bits(phy->apahb,
			REG_AP_AHB_OTG_PHY_CTRL, msk1, reg1);
	}
	return 0;
}

static int sprd_hsphy_init(struct usb_phy *x)
{
	struct sprd_hsphy *phy = container_of(x, struct sprd_hsphy, phy);
	uint32_t reg1 = 0, msk1;
	uint32_t efuse_val;
	int ret;

	if (atomic_read(&phy->inited)) {
		dev_dbg(x->dev, "%s is already inited!\n", __func__);
		return 0;
	}

	/* Turn On VDD */
	regulator_set_voltage(phy->vdd, phy->vdd_vol, phy->vdd_vol);

	if (!regulator_is_enabled(phy->vdd))
		ret = regulator_enable(phy->vdd);

	regmap_update_bits(phy->hsphy_glb, REG_AON_APB_APB_EB2,
		BIT_AON_APB_OTG_REF_EB, BIT_AON_APB_OTG_REF_EB);

	regmap_update_bits(phy->ana_g2,
		REG_ANLG_PHY_G2_ANALOG_USB20_USB20_RSTPD_CTRL0,
		BIT_ANLG_PHY_G2_ANALOG_USB20_SUSPENDM0,
		BIT_ANLG_PHY_G2_ANALOG_USB20_SUSPENDM0);
	regmap_update_bits(phy->ana_g2,
		REG_ANLG_PHY_G2_ANALOG_USB20_REG_SEL_CFG_0,
		BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_USB20_SUSPENDM0,
		0);

	/* Restore PHY tunes move to ana*/

	efuse_val =  sprd_ap_efuse_read(OTG_PHY_EFUSE_BLOCK_ID);
	if ((efuse_val & OTG_PHY_TUNE_EFUSE_FLG) == 0) {
		regmap_hwlock_update_bits(phy->ana_g2,
			REG_ANLG_PHY_G2_ANALOG_USB20_USB20_PARM_CTRL0,
			0xFFFFFFFF, phy->phy_tune);
		pr_info("Notify: USB PHY setting in efuse  is empty\n");
	}

	mdelay(2);

	/* usb vbus valid */
	reg1 = msk1 = BIT_AP_AHB_OTG_VBUS_VALID_PHYREG;
	regmap_hwlock_update_bits(phy->apahb, REG_AP_AHB_OTG_PHY_TEST,
		msk1, reg1);

	reg1 = msk1 = BIT_ANLG_PHY_G2_ANALOG_USB20_VBUSVLDEXT0;
	regmap_hwlock_update_bits(phy->ana_g2,
		REG_ANLG_PHY_G2_ANALOG_USB20_USB20_UART_CTRL0, msk1, reg1);

	reg1 = msk1 = BIT_ANLG_PHY_G2_ANALOG_USB20_VBUSVLDEXTSEL0;
	regmap_hwlock_update_bits(phy->ana_g2,
		REG_ANLG_PHY_G2_ANALOG_USB20_USB20_OTG_CTRL0, msk1, reg1);

	/* for SPRD phy utmi_width sel */
	reg1 = msk1 = BIT_AP_AHB_UTMI_WIDTH_SEL;
	regmap_hwlock_update_bits(phy->apahb, REG_AP_AHB_OTG_PHY_CTRL,
		msk1, reg1);

	reg1 = msk1 = BIT_ANLG_PHY_G2_ANALOG_USB20_WORDINTERFACE0;
	regmap_hwlock_update_bits(phy->ana_g2,
		REG_ANLG_PHY_G2_ANALOG_USB20_USB20_CTRL0,	msk1, reg1);

	if (!atomic_read(&phy->reset)) {
		msleep(20);
		__reset_core(phy);
		atomic_set(&phy->reset, 1);
	}

	atomic_set(&phy->inited, 1);

	return 0;
}

static void sprd_hsphy_shutdown(struct usb_phy *x)
{
	struct sprd_hsphy *phy = container_of(x, struct sprd_hsphy, phy);
	uint32_t reg1 = 0, msk1;

	if (!atomic_read(&phy->inited)) {
		dev_dbg(x->dev, "%s is already shut down\n", __func__);
		return;
	}
	/* Backup PHY Tune value move to ana*/

	/* usb vbus */
	msk1 = BIT_AP_AHB_OTG_VBUS_VALID_PHYREG;
	regmap_hwlock_update_bits(phy->apahb, REG_AP_AHB_OTG_PHY_TEST,
		msk1, reg1);

	msk1 = BIT_ANLG_PHY_G2_ANALOG_USB20_VBUSVLDEXT0;
	regmap_hwlock_update_bits(phy->ana_g2,
		REG_ANLG_PHY_G2_ANALOG_USB20_USB20_UART_CTRL0, msk1, reg1);

	msk1 = BIT_ANLG_PHY_G2_ANALOG_USB20_VBUSVLDEXTSEL0;
	regmap_hwlock_update_bits(phy->ana_g2,
		REG_ANLG_PHY_G2_ANALOG_USB20_USB20_OTG_CTRL0, msk1, reg1);

	regmap_update_bits(phy->ana_g2,
		REG_ANLG_PHY_G2_ANALOG_USB20_USB20_RSTPD_CTRL0,
		BIT_ANLG_PHY_G2_ANALOG_USB20_SUSPENDM0,
		0);
	regmap_update_bits(phy->ana_g2,
		REG_ANLG_PHY_G2_ANALOG_USB20_REG_SEL_CFG_0,
		BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_USB20_SUSPENDM0,
		BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_USB20_SUSPENDM0);

	regmap_update_bits(phy->hsphy_glb, REG_AON_APB_APB_EB2,
		BIT_AON_APB_OTG_REF_EB, reg1);

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

static int sprd_hsphy_probe(struct platform_device *pdev)
{
	struct sprd_hsphy *phy;
	struct device *dev = &pdev->dev;
	struct regmap *hsphy_glb = NULL, *base;
	int ret = 0;
	struct usb_otg *otg;

	phy = devm_kzalloc(dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

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

	base = syscon_regmap_lookup_by_phandle(dev->of_node,
				 "sprd,syscon-apahb");
	if (IS_ERR(base)) {
		dev_err(&pdev->dev, "ap USB apahb syscon failed!\n");
		return -ENOMEM;
	}
	phy->apahb = base;

	base = syscon_regmap_lookup_by_phandle(dev->of_node,
				 "sprd,syscon-anag2");
	if (IS_ERR(base)) {
		dev_err(&pdev->dev, "ap USB apahb syscon failed!\n");
		return -ENOMEM;
	}
	phy->ana_g2 = base;

	hsphy_glb = syscon_regmap_lookup_by_phandle(dev->of_node,
				 "sprd,syscon-enable");
	if (IS_ERR(hsphy_glb)) {
		dev_err(&pdev->dev, "ap USB PHY syscon failed!\n");
		return -ENOMEM;
	}
	phy->hsphy_glb = hsphy_glb;

	regmap_update_bits(phy->ana_g2,
		REG_ANLG_PHY_G2_ANALOG_USB20_USB20_RSTPD_CTRL0,
		BIT_ANLG_PHY_G2_ANALOG_USB20_SUSPENDM0,
		BIT_ANLG_PHY_G2_ANALOG_USB20_SUSPENDM0);
	regmap_update_bits(phy->ana_g2,
		REG_ANLG_PHY_G2_ANALOG_USB20_REG_SEL_CFG_0,
		BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_USB20_SUSPENDM0,
		0);

	regmap_update_bits(phy->ana_g2,
		REG_ANLG_PHY_G2_ANALOG_USB20_USB20_PLL_CTRL0,
		BIT_ANLG_PHY_G2_ANALOG_USB20_COMPDISTUNE0(0x7),
		BIT_ANLG_PHY_G2_ANALOG_USB20_COMPDISTUNE0(0x5));
	/* enable usb module */
	regmap_update_bits(phy->apahb, REG_AP_AHB_AHB_EB,
		BIT_AP_AHB_OTG_EB, BIT_AP_AHB_OTG_EB);
	regmap_update_bits(phy->hsphy_glb, REG_AON_APB_APB_EB2,
		BIT_AON_APB_ANLG_APB_EB | BIT_AON_APB_ANLG_EB |
		BIT_AON_APB_OTG_REF_EB,
		BIT_AON_APB_ANLG_APB_EB | BIT_AON_APB_ANLG_EB |
		BIT_AON_APB_OTG_REF_EB);

	phy->phy.dev = dev;
	phy->phy.label = "sprd-hsphy";
	phy->phy.otg = otg;
	phy->phy.init = sprd_hsphy_init;
	phy->phy.shutdown = sprd_hsphy_shutdown;
	phy->phy.post_init = sprd_hsphy_post_init;
	phy->phy.reset_phy = sprd_hsphy_reset;
	phy->phy.set_vbus = sprd_hostphy_set;
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

static const struct of_device_id sprd_hsphy_match[] = {
	{ .compatible = "sprd,usb-phy" },
	{},
};

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

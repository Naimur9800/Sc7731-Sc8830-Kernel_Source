/*
 * Copyright (C) 2016 Spreadtrum Communications Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/bug.h>
#include <linux/miscdevice.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/mfd/sprd/pmic_glb_reg.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>

struct sprd_7sreset {
	struct device *dev;
	struct regmap *reg_map;
	unsigned int reg_reset_ctrl;
	unsigned int reg_7s_ctrl;
	unsigned long chip_ver;
};

struct sprd_7sreset sprd_7sreset_dev;

static const struct of_device_id sprd_7sreset_of_match[] = {
	{.compatible = "sprd,sc2723-7sreset", .data = (void *)0x2723},
	{.compatible = "sprd,sc2731-7sreset", .data = (void *)0x2731},
	{.compatible = "sprd,sc2721-7sreset", .data = (void *)0x2721},
	{.compatible = "sprd,sc2720-7sreset", .data = (void *)0x2720},
	{}
};

static inline void sci_adi_write
	(unsigned int reg, unsigned int val, unsigned int msk)
{
	regmap_update_bits(sprd_7sreset_dev.reg_map, (reg), (msk), (val));
}

static inline void sci_adi_set(unsigned int reg, unsigned int bits)
{
	regmap_update_bits(sprd_7sreset_dev.reg_map, (reg), (bits), (bits));
}

static inline void sci_adi_clr(unsigned int reg, unsigned int bits)
{
	regmap_update_bits(sprd_7sreset_dev.reg_map, (reg), (bits), 0);
}

static inline unsigned int sci_adi_read(unsigned int reg)
{
	unsigned int r_val;

	regmap_read(sprd_7sreset_dev.reg_map, (reg), (&r_val));
	return r_val;
}

static inline int pbint_7s_rst_disable(int disable)
{
	if (disable)
		sci_adi_set(ANA_REG_GLB_POR_7S_CTRL, BIT_PBINT_7S_RST_DISABLE);
	 else
		sci_adi_clr(ANA_REG_GLB_POR_7S_CTRL, BIT_PBINT_7S_RST_DISABLE);

	return 0;
}

static inline int pbint_7s_rst_is_disable(void)
{
	return !!(sci_adi_read(ANA_REG_GLB_POR_7S_CTRL) &
		   BIT_PBINT_7S_RST_DISABLE);
}

static inline int pbint_7s_rst_set_keymode(int mode)
{
	unsigned int reg;

	if ((sprd_7sreset_dev.chip_ver == 0x2723) ||
	    (sprd_7sreset_dev.chip_ver == 0x2731)) {
		reg = sprd_7sreset_dev.reg_reset_ctrl;
	} else {
		reg = sprd_7sreset_dev.reg_7s_ctrl;
	}

	if (!mode)
		sci_adi_set(reg, BIT_KEY2_7S_RST_EN);
	else
		sci_adi_clr(reg, BIT_KEY2_7S_RST_EN);

	return 0;
}

static inline int pbint_7s_rst_get_keymode(void)
{
	unsigned int reg;

	if ((sprd_7sreset_dev.chip_ver == 0x2723) ||
	    (sprd_7sreset_dev.chip_ver == 0x2731)) {
		reg = sprd_7sreset_dev.reg_reset_ctrl;
	} else {
		reg = sprd_7sreset_dev.reg_7s_ctrl;
	}

	return !(sci_adi_read(reg) & BIT_KEY2_7S_RST_EN);
}

static inline int pbint_7s_rst_set_resetmode(int mode)
{
	if (mode) {
		sci_adi_set(sprd_7sreset_dev.reg_7s_ctrl,
			BIT_PBINT_7S_RST_MODE);
	} else {
		sci_adi_clr(sprd_7sreset_dev.reg_7s_ctrl,
			BIT_PBINT_7S_RST_MODE);
	}
	return 0;
}

static inline int pbint_7s_rst_get_resetmode(void)
{
	return !!(sci_adi_read(sprd_7sreset_dev.reg_7s_ctrl) &
		   BIT_PBINT_7S_RST_MODE);
}

static inline int pbint_7s_rst_set_shortmode(int mode)
{
	if (mode) {
		sci_adi_set(sprd_7sreset_dev.reg_7s_ctrl,
			BIT_PBINT_7S_RST_SWMODE);
	} else {
		sci_adi_clr(sprd_7sreset_dev.reg_7s_ctrl,
			BIT_PBINT_7S_RST_SWMODE);
	}
	return 0;
}

static inline int pbint_7s_rst_get_shortmode(void)
{
	return !!(sci_adi_read(sprd_7sreset_dev.reg_7s_ctrl)
		& BIT_PBINT_7S_RST_SWMODE);
}

static inline int pbint_7s_rst_set_threshold(unsigned int th)
{
	int shft = __ffs(BITS_PBINT_7S_RST_THRESHOLD(~0U));
	unsigned int max = BITS_PBINT_7S_RST_THRESHOLD(~0U) >> shft;

	if (th > max)
		th = max;

	sci_adi_write(sprd_7sreset_dev.reg_7s_ctrl,
		BITS_PBINT_7S_RST_THRESHOLD(th),
		BITS_PBINT_7S_RST_THRESHOLD(~0U));
	return 0;
}

static inline int pbint_7s_rst_get_threshold(void)
{
	int shft = __ffs(BITS_PBINT_7S_RST_THRESHOLD(~0U));

	return ((sci_adi_read(sprd_7sreset_dev.reg_7s_ctrl) &
		 BITS_PBINT_7S_RST_THRESHOLD(~0U)) >> shft);
}

static ssize_t module_disable_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long set_value;

	if (kstrtoul(buf, 10, &set_value))
		return -EINVAL;

	pr_info("%s,%lu\n", __func__, set_value);

	pbint_7s_rst_disable(set_value);

	return count;
}

static ssize_t module_disable_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int i = 0;
	int value;

	pr_info("%s\n", __func__);

	value = pbint_7s_rst_is_disable();

	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", value);
	return i;
}
static DEVICE_ATTR_RW(module_disable);

static ssize_t hard_mode_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long set_value;

	if (kstrtoul(buf, 10, &set_value))
		return -EINVAL;

	pr_info("%s,%lu\n", __func__, set_value);

	pbint_7s_rst_set_resetmode(set_value);

	return count;
}

static ssize_t hard_mode_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int i = 0;
	int value;

	pr_info("%s\n", __func__);

	value = pbint_7s_rst_get_resetmode();

	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", value);
	return i;
}
static DEVICE_ATTR_RW(hard_mode);

static ssize_t short_mode_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long set_value;

	if (kstrtoul(buf, 10, &set_value))
		return -EINVAL;

	pr_info("%s,%lu\n", __func__, set_value);

	pbint_7s_rst_set_shortmode(set_value);

	return count;
}

static ssize_t short_mode_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int i = 0;
	int value;

	pr_info("%s\n", __func__);

	value = pbint_7s_rst_get_shortmode();

	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", value);
	return i;
}
static DEVICE_ATTR_RW(short_mode);

static ssize_t keypad_mode_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long set_value;

	if (kstrtoul(buf, 10, &set_value))
		return -EINVAL;

	pr_info("%s,%lu\n", __func__, set_value);

	pbint_7s_rst_set_keymode(set_value);

	return count;
}

static ssize_t keypad_mode_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int i = 0;
	int value;

	pr_info("%s\n", __func__);

	value = pbint_7s_rst_get_keymode();

	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", value);
	return i;
}
static DEVICE_ATTR_RW(keypad_mode);

static ssize_t threshold_time_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long set_value;

	if (kstrtoul(buf, 10, &set_value))
		return -EINVAL;

	pr_info("%s,%lu\n", __func__, set_value);

	pbint_7s_rst_set_threshold(set_value);

	return count;
}

static ssize_t threshold_time_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int i = 0;
	int value;

	pr_info("%s\n", __func__);

	value = pbint_7s_rst_get_threshold();

	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", value);
	return i;
}
static DEVICE_ATTR_RW(threshold_time);

static struct attribute *sprd7sreset_attrs[] = {
	&dev_attr_module_disable.attr,
	&dev_attr_hard_mode.attr,
	&dev_attr_short_mode.attr,
	&dev_attr_keypad_mode.attr,
	&dev_attr_threshold_time.attr,
	NULL,
};

static struct attribute_group sprd7sreset_group = {
	.name = NULL,
	.attrs = sprd7sreset_attrs,
};

static struct miscdevice sprd_7sreset_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sprd_7sreset",
	.fops = NULL,
};

static int sprd_7sreset_probe(struct platform_device *pdev)
{
	int ret;
	u32 val;
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *of_id;

	sprd_7sreset_dev.reg_map = dev_get_regmap(pdev->dev.parent, NULL);
	if (!sprd_7sreset_dev.reg_map) {
		dev_err(&pdev->dev, "%s :NULL regmap for pmic\n",
			__func__);
		return -EINVAL;
	}

	sprd_7sreset_dev.dev = &pdev->dev;
	of_id = of_match_node(sprd_7sreset_of_match,
		pdev->dev.of_node);
	if (!of_id) {
		dev_err(&pdev->dev, "get 27xx 7sreset of device id failed!\n");
		return -ENODEV;
	}

	sprd_7sreset_dev.chip_ver = (unsigned long)(of_id->data);
	ret = of_property_read_u32_index(np, "reg", 0, &val);
	if (ret)
		return ret;

	sprd_7sreset_dev.reg_reset_ctrl = val;
	ret = of_property_read_u32_index(np, "reg", 1, &val);
	if (ret)
		return ret;

	sprd_7sreset_dev.reg_7s_ctrl = val;
	platform_set_drvdata(pdev, &sprd_7sreset_dev);
	ret = misc_register(&sprd_7sreset_miscdev);
	if (ret)
		return ret;

	ret = sysfs_create_group(&sprd_7sreset_miscdev.this_device->kobj,
			&sprd7sreset_group);
	if (ret) {
		pr_err("%s failed to create device attributes.\n", __func__);
		goto err_attr_failed;
	}
	return 0;

err_attr_failed:
	misc_deregister(&sprd_7sreset_miscdev);
	return ret;
}

static int sprd_7sreset_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&sprd_7sreset_miscdev.this_device->kobj,
				&sprd7sreset_group);

	misc_deregister(&sprd_7sreset_miscdev);
	return 0;
}

static struct platform_driver sprd_7sreset_drv = {
	.driver = {
		   .name = "sprd_7sreset",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(sprd_7sreset_of_match),
		   },
	.probe = sprd_7sreset_probe,
	.remove = sprd_7sreset_remove
};

static int __init sprd_7sreset_init(void)
{
	return platform_driver_register(&sprd_7sreset_drv);
}

static void __exit sprd_7sreset_exit(void)
{
	platform_driver_unregister(&sprd_7sreset_drv);
}

module_init(sprd_7sreset_init);
module_exit(sprd_7sreset_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Spreadtrum 7SReset Driver");
MODULE_AUTHOR("Mingwei <Mingwei.Zhang@spreadtrum.com>");

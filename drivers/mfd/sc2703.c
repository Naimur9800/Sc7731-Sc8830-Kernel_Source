/*
 * SPDX-License-Identifier: GPL-2.0
 * Core MFD(Charger, ADC, Flash and GPIO) driver for SC2703
 *
 * Copyright (c) 2018 Dialog Semiconductor.
 */

#include <linux/debugfs.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/of_platform.h>
#include <linux/mfd/core.h>
#include <linux/mfd/sc2703/registers.h>

#define DEBUG_ROOT_DIR	"sc2703_chg"
#define SC2703_NUM_IRQ_REGS	6

struct sc2703_pdata {
	int irq_base;
};

struct sc2703 {
	struct device *dev;
	struct regmap *regmap;
	struct regmap_irq_chip_data *regmap_irq_data;
	struct dentry *debugfs_root;
	int irq;
	int irq_base;
	u32 addr;
	u32 read_cnt;
};

enum sc2703_irq_defs {
	SC2703_IRQ_ADP_DET = 0,
	SC2703_IRQ_VIN2BAT,
	SC2703_IRQ_VIN_UV,
	SC2703_IRQ_VIN_DROP,
	SC2703_IRQ_VIN_OV,
	SC2703_IRQ_VBAT_UV,
	SC2703_IRQ_VBAT_OV,
	SC2703_IRQ_TJUNC_WARN,
	SC2703_IRQ_TJUNC_CRIT,
	SC2703_IRQ_TJUNC_POR,
	SC2703_IRQ_TBAT_T1,
	SC2703_IRQ_TBAT_T2,
	SC2703_IRQ_TBAT_T3,
	SC2703_IRQ_TBAT_T4,
	SC2703_IRQ_BATDET,
	SC2703_IRQ_VIN_REV_SHORT,
	SC2703_IRQ_VIN_REV_OV,
	SC2703_IRQ_DCDC_REV_BOOST_FAULT,
	SC2703_IRQ_IIN_REV_LIM,
	SC2703_IRQ_IIN_REV_LIM_MAX,
	SC2703_IRQ_LOWBAT,
	SC2703_IRQ_ADC_DONE,
	SC2703_IRQ_IIN_LIM,
	SC2703_IRQ_CHG_STAT,
	SC2703_IRQ_VSYS_POR,
	SC2703_IRQ_VSYS_OV,
	SC2703_IRQ_WD,
	SC2703_IRQ_TIMEOUT_PRE,
	SC2703_IRQ_TIMEOUT_CCCV,
	SC2703_IRQ_GPI0,
	SC2703_IRQ_GPI1,
	SC2703_IRQ_GPI2,
	SC2703_IRQ_GPI3,
	SC2703_IRQ_VSYS_UV,
	SC2703_IRQ_BOOST_STARTUP_OV,
	SC2703_IRQ_TORCH_CHG_OV,
	SC2703_IRQ_LOWBAT_BOOST,
	SC2703_NUM_IRQS,
};

static bool sc2703_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SC2703_STATUS_A:
	case SC2703_STATUS_B:
	case SC2703_STATUS_C:
	case SC2703_STATUS_D:
	case SC2703_STATUS_E:
	case SC2703_STATUS_F:
	case SC2703_STATUS_G:
	case SC2703_STATUS_H:
	case SC2703_STATUS_I:
	case SC2703_EVENT_A:
	case SC2703_EVENT_B:
	case SC2703_EVENT_C:
	case SC2703_EVENT_D:
	case SC2703_EVENT_E:
	case SC2703_EVENT_F:
	case SC2703_DCDC_CTRL_A:
	case SC2703_ADC_CTRL_A:
	case SC2703_ADC_RES_0:
	case SC2703_ADC_RES_1:
	case SC2703_ADC_RES_2:
	case SC2703_ADC_RES_3:
	case SC2703_ADC_RES_4:
	case SC2703_ADC_RES_5:
	case SC2703_CHG_TIMER_CTRL_B:
	case SC2703_CHG_TIMER_CTRL_C:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config sc2703_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = SC2703_MAX_REG,
	.cache_type = REGCACHE_RBTREE,
	.volatile_reg = sc2703_volatile_reg,
};

static const struct regmap_irq sc2703_irqs[] = {
	[SC2703_IRQ_ADP_DET] = {
		.reg_offset = 0,
		.mask = SC2703_E_ADP_DET_MASK,
	},
	[SC2703_IRQ_VIN2BAT] = {
		.reg_offset = 0,
		.mask = SC2703_E_VIN2BAT_MASK,
	},
	[SC2703_IRQ_VIN_UV] = {
		.reg_offset = 0,
		.mask = SC2703_E_VIN_UV_MASK,
	},
	[SC2703_IRQ_VIN_DROP] = {
		.reg_offset = 0,
		.mask = SC2703_E_VIN_DROP_MASK,
	},
	[SC2703_IRQ_VIN_OV] = {
		.reg_offset = 0,
		.mask = SC2703_E_VIN_OV_MASK,
	},
	[SC2703_IRQ_VBAT_UV] = {
		.reg_offset = 0,
		.mask = SC2703_E_VBAT_UV_MASK,
	},
	[SC2703_IRQ_VBAT_OV] = {
		.reg_offset = 0,
		.mask = SC2703_E_VBAT_OV_MASK,
	},
	[SC2703_IRQ_TJUNC_WARN] = {
		.reg_offset = 1,
		.mask = SC2703_E_TJUNC_WARN_MASK,
	},
	[SC2703_IRQ_TJUNC_CRIT] = {
		.reg_offset = 1,
		.mask = SC2703_E_TJUNC_CRIT_MASK,
	},
	[SC2703_IRQ_TJUNC_POR] = {
		.reg_offset = 1,
		.mask = SC2703_E_TJUNC_POR_MASK,
	},
	[SC2703_IRQ_TBAT_T1] = {
		.reg_offset = 1,
		.mask = SC2703_E_TBAT_T1_MASK,
	},
	[SC2703_IRQ_TBAT_T2] = {
		.reg_offset = 1,
		.mask = SC2703_E_TBAT_T2_MASK,
	},
	[SC2703_IRQ_TBAT_T3] = {
		.reg_offset = 1,
		.mask = SC2703_E_TBAT_T3_MASK,
	},
	[SC2703_IRQ_TBAT_T4] = {
		.reg_offset = 1,
		.mask = SC2703_E_TBAT_T4_MASK,
	},
	[SC2703_IRQ_BATDET] = {
		.reg_offset = 1,
		.mask = SC2703_E_BATDET_MASK,
	},
	[SC2703_IRQ_VIN_REV_SHORT] = {
		.reg_offset = 2,
		.mask = SC2703_E_VIN_REV_SHORT_MASK,
	},
	[SC2703_IRQ_VIN_REV_OV] = {
		.reg_offset = 2,
		.mask = SC2703_E_VIN_REV_OV_MASK,
	},
	[SC2703_IRQ_DCDC_REV_BOOST_FAULT] = {
		.reg_offset = 2,
		.mask = SC2703_E_DCDC_REV_BOOST_FAULT_MASK,
	},
	[SC2703_IRQ_IIN_REV_LIM] = {
		.reg_offset = 2,
		.mask = SC2703_E_IIN_REV_LIM_MASK,
	},
	[SC2703_IRQ_IIN_REV_LIM_MAX] = {
		.reg_offset = 2,
		.mask = SC2703_E_IIN_REV_LIM_MAX_MASK,
	},
	[SC2703_IRQ_LOWBAT] = {
		.reg_offset = 2,
		.mask = SC2703_E_LOWBAT_MASK,
	},
	[SC2703_IRQ_ADC_DONE] = {
		.reg_offset = 2,
		.mask = SC2703_E_ADC_DONE_MASK,
	},
	[SC2703_IRQ_IIN_LIM] = {
		.reg_offset = 2,
		.mask = SC2703_E_IIN_LIM_MASK,
	},
	[SC2703_IRQ_CHG_STAT] = {
		.reg_offset = 3,
		.mask = SC2703_E_CHG_STAT_MASK,
	},
	[SC2703_IRQ_VSYS_POR] = {
		.reg_offset = 3,
		.mask = SC2703_E_VSYS_POR_MASK,
	},
	[SC2703_IRQ_VSYS_OV] = {
		.reg_offset = 3,
		.mask = SC2703_E_VSYS_OV_MASK,
	},
	[SC2703_IRQ_WD] = {
		.reg_offset = 3,
		.mask = SC2703_E_WD_MASK,
	},
	[SC2703_IRQ_TIMEOUT_PRE] = {
		.reg_offset = 3,
		.mask = SC2703_E_TIMEOUT_PRE_MASK,
	},
	[SC2703_IRQ_TIMEOUT_CCCV] = {
		.reg_offset = 3,
		.mask = SC2703_E_TIMEOUT_CCCV_MASK,
	},
	[SC2703_IRQ_GPI0] = {
		.reg_offset = 4,
		.mask = SC2703_E_GPI0_MASK,
	},
	[SC2703_IRQ_GPI1] = {
		.reg_offset = 4,
		.mask = SC2703_E_GPI1_MASK,
	},
	[SC2703_IRQ_GPI2] = {
		.reg_offset = 4,
		.mask = SC2703_E_GPI2_MASK,
	},
	[SC2703_IRQ_GPI3] = {
		.reg_offset = 4,
		.mask = SC2703_E_GPI3_MASK,
	},
	[SC2703_IRQ_VSYS_UV] = {
		.reg_offset = 4,
		.mask = SC2703_E_VSYS_UV_MASK,
	},
	[SC2703_IRQ_BOOST_STARTUP_OV] = {
		.reg_offset = 4,
		.mask = SC2703_E_BOOST_STARTUP_OV_MASK,
	},
	[SC2703_IRQ_TORCH_CHG_OV] = {
		.reg_offset = 5,
		.mask = SC2703_E_TORCH_CHG_OV_MASK,
	},
	[SC2703_IRQ_LOWBAT_BOOST] = {
		.reg_offset = 5,
		.mask = SC2703_E_LOWBAT_BOOST_MASK,
	},
};

static const struct regmap_irq_chip sc2703_regmap_irq_chip = {
	.name = "sc2703_irq",
	.status_base = SC2703_EVENT_A,
	.mask_base = SC2703_MASK_A,
	.ack_base = SC2703_EVENT_A,
	.num_regs = SC2703_NUM_IRQ_REGS,
	.irqs = sc2703_irqs,
	.num_irqs = ARRAY_SIZE(sc2703_irqs),
};

/* Helper macro to automatically populate resource name */
#define SC2703_RES_IRQ_NAMED(_name)	\
	DEFINE_RES_IRQ_NAMED(SC2703_IRQ_##_name, #_name)

/* ADC IRQs */
static struct resource sc2703_adc_resources[] = {
	SC2703_RES_IRQ_NAMED(ADC_DONE),
};

/* Charger IRQs */
static struct resource sc2703_charger_resources[] = {
	SC2703_RES_IRQ_NAMED(ADP_DET),
	SC2703_RES_IRQ_NAMED(VIN2BAT),
	SC2703_RES_IRQ_NAMED(VIN_UV),
	SC2703_RES_IRQ_NAMED(VIN_DROP),
	SC2703_RES_IRQ_NAMED(VIN_OV),
	SC2703_RES_IRQ_NAMED(VBAT_UV),
	SC2703_RES_IRQ_NAMED(VBAT_OV),
	SC2703_RES_IRQ_NAMED(TJUNC_WARN),
	SC2703_RES_IRQ_NAMED(TJUNC_CRIT),
	SC2703_RES_IRQ_NAMED(TBAT_T1),
	SC2703_RES_IRQ_NAMED(TBAT_T2),
	SC2703_RES_IRQ_NAMED(TBAT_T3),
	SC2703_RES_IRQ_NAMED(TBAT_T4),
	SC2703_RES_IRQ_NAMED(BATDET),
	SC2703_RES_IRQ_NAMED(VIN_REV_SHORT),
	SC2703_RES_IRQ_NAMED(VIN_REV_OV),
	SC2703_RES_IRQ_NAMED(DCDC_REV_BOOST_FAULT),
	SC2703_RES_IRQ_NAMED(IIN_REV_LIM),
	SC2703_RES_IRQ_NAMED(IIN_REV_LIM_MAX),
	SC2703_RES_IRQ_NAMED(LOWBAT),
	SC2703_RES_IRQ_NAMED(IIN_LIM),
	SC2703_RES_IRQ_NAMED(CHG_STAT),
	SC2703_RES_IRQ_NAMED(VSYS_OV),
	SC2703_RES_IRQ_NAMED(WD),
	SC2703_RES_IRQ_NAMED(TIMEOUT_PRE),
	SC2703_RES_IRQ_NAMED(TIMEOUT_CCCV),
	SC2703_RES_IRQ_NAMED(VSYS_UV),
	SC2703_RES_IRQ_NAMED(BOOST_STARTUP_OV),
	SC2703_RES_IRQ_NAMED(TORCH_CHG_OV),
	SC2703_RES_IRQ_NAMED(LOWBAT_BOOST),
};

static struct resource sc2703_gpio_resources[] = {
	SC2703_RES_IRQ_NAMED(GPI0),
	SC2703_RES_IRQ_NAMED(GPI1),
	SC2703_RES_IRQ_NAMED(GPI2),
	SC2703_RES_IRQ_NAMED(GPI3),
};

enum sc2703_dev_idx {
	SC2703_ADC_IDX = 0,
	SC2703_CHARGER_IDX,
	SC2703_GPIO_IDX,
};

static struct mfd_cell sc2703_devs[] = {
	[SC2703_ADC_IDX] = {
		.name = "sc2703-adc",
		.of_compatible = "sprd,sc2703-adc",
		.resources = sc2703_adc_resources,
		.num_resources = ARRAY_SIZE(sc2703_adc_resources),
	},
	[SC2703_CHARGER_IDX] = {
		.name = "sc2703-charger",
		.of_compatible = "sprd,sc2703-charger",
		.resources = sc2703_charger_resources,
		.num_resources = ARRAY_SIZE(sc2703_charger_resources),
	},
	[SC2703_GPIO_IDX] = {
		.name = "sc2703-gpio",
		.of_compatible = "sprd,sc2703-gpio",
		.resources = sc2703_gpio_resources,
		.num_resources = ARRAY_SIZE(sc2703_gpio_resources),
	},
};

static int debugfs_addr_get(void *data, u64 *val)
{
	struct sc2703 *sc2703 = data;

	*val = sc2703->addr;
	return 0;
}

static int debugfs_addr_set(void *data, u64 val)
{
	struct sc2703 *sc2703 = data;

	sc2703->addr = (unsigned int)val;
	return 0;
}

static int debugfs_cnt_get(void *data, u64 *val)
{
	struct sc2703 *sc2703 = data;

	*val = sc2703->read_cnt;
	return 0;
}

static int debugfs_cnt_set(void *data, u64 val)
{
	struct sc2703 *sc2703 = data;

	sc2703->read_cnt = val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(sc2703_addr,
			debugfs_addr_get, debugfs_addr_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(sc2703_cnt,
			debugfs_cnt_get, debugfs_cnt_set, "%llu\n");

static int sc2703_debug_show(struct seq_file *m, void *private)
{
	struct sc2703 *sc2703 = m->private;
	unsigned int reg = sc2703->addr;
	unsigned int val, i;

	seq_puts(m, "*****************************\n");

	for (i = 0; i < sc2703->read_cnt; i++) {
		if (regmap_read(sc2703->regmap, reg, &val)) {
			seq_puts(m, "read i2c err!\n");
			return 0;
		}

		seq_printf(m, "reg:0x%x --- value:0x%x\n", reg, val);
		reg++;
	}

	seq_puts(m, "-----------------------------\n");

	return 0;
}

static int sc2703_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, sc2703_debug_show, inode->i_private);
}

static ssize_t sc2703_debug_write(struct file *filep,
				  const char __user *buf,
				  size_t len,
				  loff_t *ppos)
{
	struct seq_file *m = filep->private_data;
	struct sc2703 *sc2703 = m->private;
	u32 val;
	int ret;

	ret = kstrtouint_from_user(buf, len, 0, &val);
	if (ret)
		return ret;

	ret = regmap_write(sc2703->regmap, sc2703->addr, val);
	if (ret)
		return ret;

	return len;
}

static const struct file_operations sc2703_value_fops = {
	.open = sc2703_debug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write   = sc2703_debug_write,
	.release = single_release,
};

static int sc2703_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct sc2703 *sc2703;
	struct sc2703_pdata *pdata = dev_get_platdata(&client->dev);
	struct regmap *regmap;
	struct irq_data *irq_data = irq_get_irq_data(client->irq);
	unsigned int irq_flags, event_b, event_d;
	int ret;

	/* Determine IRQ level type */
	if (!irq_data) {
		dev_err(&client->dev, "Invalid IRQ given: %d\n", client->irq);
		return -EINVAL;
	}
	irq_flags = irqd_get_trigger_type(irq_data);
	sc2703 = devm_kzalloc(dev, sizeof(*sc2703), GFP_KERNEL);
	if (!sc2703)
		return -ENOMEM;

	sc2703->dev = dev;
	sc2703->irq = client->irq;
	i2c_set_clientdata(client, sc2703);

	if (pdata)
		sc2703->irq_base = pdata->irq_base;

	regmap = devm_regmap_init_i2c(client, &sc2703_regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}
	sc2703->regmap = regmap;
	sc2703->read_cnt = 1;

	/* Report and clear POR related fault events before enabling IRQs */
	ret = regmap_read(regmap, SC2703_EVENT_B, &event_b);
	if (ret)
		return ret;

	if (event_b & SC2703_E_TJUNC_POR_MASK) {
		dev_info(dev, "Reset due to TJUNC POR event\n");
		ret = regmap_update_bits(regmap, SC2703_EVENT_B,
					 SC2703_E_TJUNC_POR_MASK,
					 SC2703_E_TJUNC_POR_MASK);
		if (ret)
			return ret;
	}

	ret = regmap_read(regmap, SC2703_EVENT_D, &event_d);
	if (ret)
		return ret;

	if (event_d & SC2703_E_VSYS_POR_MASK) {
		dev_info(dev, "Reset due to VSYS POR event\n");
		ret = regmap_update_bits(regmap, SC2703_EVENT_D,
					 SC2703_E_VSYS_POR_MASK,
					 SC2703_E_VSYS_POR_MASK);
		if (ret)
			return ret;
	}

	ret = mfd_add_devices(dev, PLATFORM_DEVID_NONE, sc2703_devs,
			      ARRAY_SIZE(sc2703_devs), NULL,
			      sc2703->irq_base, NULL);
	if (ret) {
		dev_err(dev, "Failed to add child devices: %d\n", ret);
		return ret;
	}

	sc2703->debugfs_root = debugfs_create_dir(DEBUG_ROOT_DIR, NULL);
	if (!sc2703->debugfs_root) {
		dev_err(dev, "Failed to create sc2703 i2c debugfs directory\n");
		ret = -ENOENT;
		goto err_mfd;
	}

	debugfs_create_file("addr", S_IRUGO | S_IWUSR,
			    sc2703->debugfs_root, sc2703, &sc2703_addr);
	debugfs_create_file("read_cnt", S_IRUGO | S_IWUSR,
			    sc2703->debugfs_root, sc2703, &sc2703_cnt);
	debugfs_create_file("value", S_IRUGO | S_IWUSR,
			    sc2703->debugfs_root, sc2703, &sc2703_value_fops);

	return 0;

err_mfd:
	mfd_remove_devices(sc2703->dev);
	return ret;
}

static int sc2703_remove(struct i2c_client *client)
{
	struct sc2703 *sc2703 = i2c_get_clientdata(client);

	debugfs_remove_recursive(sc2703->debugfs_root);
	mfd_remove_devices(sc2703->dev);

	return 0;
}

static const struct of_device_id sc2703_of_match[] = {
	{ .compatible = "sprd,sc2703", },
	{ }
};

MODULE_DEVICE_TABLE(of, sc2703_of_match);

static const struct i2c_device_id sc2703_i2c_id[] = {
	{ "sc2703", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sc2703_i2c_id);

static struct i2c_driver sc2703_driver = {
	.driver	= {
		.name	= "sc2703",
		.of_match_table = sc2703_of_match,
	},
	.probe		= sc2703_probe,
	.remove		= sc2703_remove,
	.id_table	= sc2703_i2c_id,
};

static int __init sc2703_init(void)
{
	return i2c_add_driver(&sc2703_driver);
}
module_init(sc2703_init);

static void __exit sc2703_exit(void)
{
	i2c_del_driver(&sc2703_driver);
}
module_exit(sc2703_exit);

MODULE_DESCRIPTION("MFD Core Driver for SC2703");
MODULE_AUTHOR("Roy Im <Roy.Im.Opensource@diasemi.com>");
MODULE_LICENSE("GPL v2");

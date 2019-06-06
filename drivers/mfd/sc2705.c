/*
 * Core MFD(Charger, ADC, Flash and GPIO) driver for SC2705
 * Copyright (c) 2017 Dialog Semiconductor.
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
#include <linux/mfd/sc2705/core.h>
#include <linux/mfd/sc2705/registers.h>

#define DEBUG_ROOT_DIR	"sc2705_chg"

static bool sc2705_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SC2705_STATUS_A:
	case SC2705_STATUS_B:
	case SC2705_STATUS_C:
	case SC2705_STATUS_D:
	case SC2705_STATUS_E:
	case SC2705_STATUS_F:
	case SC2705_STATUS_G:
	case SC2705_STATUS_H:
	case SC2705_STATUS_I:
	case SC2705_EVENT_A:
	case SC2705_EVENT_B:
	case SC2705_EVENT_C:
	case SC2705_EVENT_D:
	case SC2705_EVENT_E:
	case SC2705_EVENT_F:
	case SC2705_EVENT_G:
	case SC2705_EVENT_H:
	case SC2705_DCDC_CTRL_A:
	case SC2705_ADC_CTRL_A:
	case SC2705_ADC_RES_0:
	case SC2705_ADC_RES_1:
	case SC2705_ADC_RES_2:
	case SC2705_ADC_RES_3:
	case SC2705_ADC_RES_4:
	case SC2705_ADC_RES_5:
	case SC2705_CHG_TIMER_CTRL_B:
	case SC2705_CHG_TIMER_CTRL_C:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config sc2705_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = SC2705_MAX_REG,
	.cache_type = REGCACHE_RBTREE,
	.volatile_reg = sc2705_volatile_reg,
};

static const struct regmap_irq sc2705_irqs[] = {
	[SC2705_IRQ_ADP_DET] = {
		.reg_offset = 0,
		.mask = SC2705_E_ADP_DET_MASK,
	},
	[SC2705_IRQ_VIN2BAT] = {
		.reg_offset = 0,
		.mask = SC2705_E_VIN2BAT_MASK,
	},
	[SC2705_IRQ_VIN_UV] = {
		.reg_offset = 0,
		.mask = SC2705_E_VIN_UV_MASK,
	},
	[SC2705_IRQ_VIN_DROP] = {
		.reg_offset = 0,
		.mask = SC2705_E_VIN_DROP_MASK,
	},
	[SC2705_IRQ_VIN_OV] = {
		.reg_offset = 0,
		.mask = SC2705_E_VIN_OV_MASK,
	},
	[SC2705_IRQ_VBAT_UV] = {
		.reg_offset = 0,
		.mask = SC2705_E_VBAT_UV_MASK,
	},
	[SC2705_IRQ_VBAT_OV] = {
		.reg_offset = 0,
		.mask = SC2705_E_VBAT_OV_MASK,
	},
	[SC2705_IRQ_IN_PWR_BLOCK] = {
		.reg_offset = 0,
		.mask = SC2705_E_IN_PWR_BLOCK_MASK,
	},
	[SC2705_IRQ_TJUNC_WARN] = {
		.reg_offset = 1,
		.mask = SC2705_E_TJUNC_WARN_MASK,
	},
	[SC2705_IRQ_TJUNC_CRIT] = {
		.reg_offset = 1,
		.mask = SC2705_E_TJUNC_CRIT_MASK,
	},
	[SC2705_IRQ_TJUNC_POR] = {
		.reg_offset = 1,
		.mask = SC2705_E_TJUNC_POR_MASK,
	},
	[SC2705_IRQ_TBAT_T1] = {
		.reg_offset = 1,
		.mask = SC2705_E_TBAT_T1_MASK,
	},
	[SC2705_IRQ_TBAT_T2] = {
		.reg_offset = 1,
		.mask = SC2705_E_TBAT_T2_MASK,
	},
	[SC2705_IRQ_TBAT_T3] = {
		.reg_offset = 1,
		.mask = SC2705_E_TBAT_T3_MASK,
	},
	[SC2705_IRQ_TBAT_T4] = {
		.reg_offset = 1,
		.mask = SC2705_E_TBAT_T4_MASK,
	},
	[SC2705_IRQ_BATDET] = {
		.reg_offset = 1,
		.mask = SC2705_E_BATDET_MASK,
	},
	[SC2705_IRQ_VIN_REV_SHORT] = {
		.reg_offset = 2,
		.mask = SC2705_E_VIN_REV_SHORT_MASK,
	},
	[SC2705_IRQ_VIN_REV_OV] = {
		.reg_offset = 2,
		.mask = SC2705_E_VIN_REV_OV_MASK,
	},
	[SC2705_IRQ_DCDC_REV_BOOST_FAULT] = {
		.reg_offset = 2,
		.mask = SC2705_E_DCDC_REV_BOOST_FAULT_MASK,
	},
	[SC2705_IRQ_IIN_REV_LIM] = {
		.reg_offset = 2,
		.mask = SC2705_E_IIN_REV_LIM_MASK,
	},
	[SC2705_IRQ_IIN_REV_LIM_MAX] = {
		.reg_offset = 2,
		.mask = SC2705_E_IIN_REV_LIM_MAX_MASK,
	},
	[SC2705_IRQ_LOWBAT] = {
		.reg_offset = 2,
		.mask = SC2705_E_LOWBAT_MASK,
	},
	[SC2705_IRQ_ADC_DONE] = {
		.reg_offset = 2,
		.mask = SC2705_E_ADC_DONE_MASK,
	},
	[SC2705_IRQ_IIN_LIM] = {
		.reg_offset = 2,
		.mask = SC2705_E_IIN_LIM_MASK,
	},
	[SC2705_IRQ_CHG_STAT] = {
		.reg_offset = 3,
		.mask = SC2705_E_CHG_STAT_MASK,
	},
	[SC2705_IRQ_VSYS_POR] = {
		.reg_offset = 3,
		.mask = SC2705_E_VSYS_POR_MASK,
	},
	[SC2705_IRQ_VSYS_OV] = {
		.reg_offset = 3,
		.mask = SC2705_E_VSYS_OV_MASK,
	},
	[SC2705_IRQ_WD] = {
		.reg_offset = 3,
		.mask = SC2705_E_WD_MASK,
	},
	[SC2705_IRQ_TIMEOUT_PRE] = {
		.reg_offset = 3,
		.mask = SC2705_E_TIMEOUT_PRE_MASK,
	},
	[SC2705_IRQ_TIMEOUT_CCCV] = {
		.reg_offset = 3,
		.mask = SC2705_E_TIMEOUT_CCCV_MASK,
	},
	[SC2705_IRQ_GPI0] = {
		.reg_offset = 4,
		.mask = SC2705_E_GPI0_MASK,
	},
	[SC2705_IRQ_GPI1] = {
		.reg_offset = 4,
		.mask = SC2705_E_GPI1_MASK,
	},
	[SC2705_IRQ_GPI2] = {
		.reg_offset = 4,
		.mask = SC2705_E_GPI2_MASK,
	},
	[SC2705_IRQ_GPI3] = {
		.reg_offset = 4,
		.mask = SC2705_E_GPI3_MASK,
	},
	[SC2705_IRQ_VSYS_UV] = {
		.reg_offset = 4,
		.mask = SC2705_E_VSYS_UV_MASK,
	},
	[SC2705_IRQ_BOOST_STARTUP_OV] = {
		.reg_offset = 4,
		.mask = SC2705_E_BOOST_STARTUP_OV_MASK,
	},
	[SC2705_IRQ_FLASH_LDO_SHORT_CKT] = {
		.reg_offset = 4,
		.mask = SC2705_E_FLASH_LDO_SHORT_CKT_MASK,
	},
	[SC2705_IRQ_TORCH_CHG_OV] = {
		.reg_offset = 5,
		.mask = SC2705_E_TORCH_CHG_OV_MASK,
	},
};

static const struct regmap_irq_chip sc2705_regmap_irq_chip = {
	.name = "sc2705_irq",
	.status_base = SC2705_EVENT_A,
	.mask_base = SC2705_MASK_A,
	.ack_base = SC2705_EVENT_A,
	.num_regs = SC2705_NUM_IRQ_REGS,
	.irqs = sc2705_irqs,
	.num_irqs = ARRAY_SIZE(sc2705_irqs),
};

/* Helper macro to automatically populate resource name */
#define SC2705_RES_IRQ_NAMED(_name)	\
	DEFINE_RES_IRQ_NAMED(SC2705_IRQ_##_name, #_name)

/* ADC IRQs */
static struct resource sc2705_adc_resources[] = {
	SC2705_RES_IRQ_NAMED(ADC_DONE),
};

/* Charger IRQs */
static struct resource sc2705_charger_resources[] = {
	SC2705_RES_IRQ_NAMED(ADP_DET),
	SC2705_RES_IRQ_NAMED(VIN2BAT),
	SC2705_RES_IRQ_NAMED(VIN_UV),
	SC2705_RES_IRQ_NAMED(VIN_DROP),
	SC2705_RES_IRQ_NAMED(VIN_OV),
	SC2705_RES_IRQ_NAMED(VBAT_UV),
	SC2705_RES_IRQ_NAMED(VBAT_OV),
	SC2705_RES_IRQ_NAMED(TJUNC_WARN),
	SC2705_RES_IRQ_NAMED(TJUNC_CRIT),
	SC2705_RES_IRQ_NAMED(TBAT_T1),
	SC2705_RES_IRQ_NAMED(TBAT_T4),
	SC2705_RES_IRQ_NAMED(BATDET),
	SC2705_RES_IRQ_NAMED(VIN_REV_SHORT),
	SC2705_RES_IRQ_NAMED(VIN_REV_OV),
	SC2705_RES_IRQ_NAMED(DCDC_REV_BOOST_FAULT),
	SC2705_RES_IRQ_NAMED(IIN_REV_LIM),
	SC2705_RES_IRQ_NAMED(IIN_REV_LIM_MAX),
	SC2705_RES_IRQ_NAMED(LOWBAT),
	SC2705_RES_IRQ_NAMED(IIN_LIM),
	SC2705_RES_IRQ_NAMED(CHG_STAT),
	SC2705_RES_IRQ_NAMED(VSYS_OV),
	SC2705_RES_IRQ_NAMED(WD),
	SC2705_RES_IRQ_NAMED(TIMEOUT_PRE),
	SC2705_RES_IRQ_NAMED(TIMEOUT_CCCV),
	SC2705_RES_IRQ_NAMED(VSYS_UV),
	SC2705_RES_IRQ_NAMED(BOOST_STARTUP_OV),
};

static struct resource sc2705_led_resources[] = {
	SC2705_RES_IRQ_NAMED(FLASH_LDO_SHORT_CKT),
	SC2705_RES_IRQ_NAMED(TORCH_CHG_OV),
};

static struct resource sc2705_gpio_resources[] = {
	SC2705_RES_IRQ_NAMED(GPI0),
	SC2705_RES_IRQ_NAMED(GPI1),
	SC2705_RES_IRQ_NAMED(GPI2),
	SC2705_RES_IRQ_NAMED(GPI3),
};

enum sc2705_dev_idx {
	SC2705_ADC_IDX = 0,
	SC2705_CHARGER_IDX,
	SC2705_LED_IDX,
	SC2705_GPIO_IDX,
};

static struct mfd_cell sc2705_devs[] = {
	[SC2705_ADC_IDX] = {
		.name = "sc2705-adc",
		.of_compatible = "sprd,sc2705-adc",
		.resources = sc2705_adc_resources,
		.num_resources = ARRAY_SIZE(sc2705_adc_resources),
	},
	[SC2705_CHARGER_IDX] = {
		.name = "sc2705-charger",
		.of_compatible = "sprd,sc2705-charger",
		.resources = sc2705_charger_resources,
		.num_resources = ARRAY_SIZE(sc2705_charger_resources),
	},
	[SC2705_LED_IDX] = {
		.name = "sc2705-led",
		.of_compatible = "sprd,sc2705-led",
		.resources = sc2705_led_resources,
		.num_resources = ARRAY_SIZE(sc2705_led_resources),
	},
	[SC2705_GPIO_IDX] = {
		.name = "sc2705-gpio",
		.of_compatible = "sprd,sc2705-gpio",
		.resources = sc2705_gpio_resources,
		.num_resources = ARRAY_SIZE(sc2705_gpio_resources),
	},
};

static int debugfs_addr_get(void *data, u64 *val)
{
	struct sc2705 *sc2705 = data;

	*val = sc2705->addr;
	return 0;
}

static int debugfs_addr_set(void *data, u64 val)
{
	struct sc2705 *sc2705 = data;

	sc2705->addr = (unsigned int)val;
	return 0;
}

static int debugfs_cnt_get(void *data, u64 *val)
{
	struct sc2705 *sc2705 = data;

	*val = sc2705->read_cnt;
	return 0;
}

static int debugfs_cnt_set(void *data, u64 val)
{
	struct sc2705 *sc2705 = data;

	sc2705->read_cnt = val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(sc2705_addr,
			debugfs_addr_get, debugfs_addr_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(sc2705_cnt,
			debugfs_cnt_get, debugfs_cnt_set, "%llu\n");

static int sc2705_debug_show(struct seq_file *m, void *private)
{
	struct sc2705 *sc2705 = m->private;
	unsigned int reg = sc2705->addr;
	unsigned int val, i;

	seq_puts(m, "*****************************\n");

	for (i = 0; i < sc2705->read_cnt; i++) {
		if (regmap_read(sc2705->regmap, reg, &val)) {
			seq_puts(m, "read i2c err!\n");
			return 0;
		}

		seq_printf(m, "reg:0x%x --- value:0x%x\n", reg, val);
		reg++;
	}

	seq_puts(m, "-----------------------------\n");

	return 0;
}

static int sc2705_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, sc2705_debug_show, inode->i_private);
}

static ssize_t sc2705_debug_write(struct file *filep,
					       const char __user *buf,
					       size_t len,
					       loff_t *ppos)
{
	struct seq_file *m = filep->private_data;
	struct sc2705 *sc2705 = m->private;
	unsigned long long val;
	int err;

	err = kstrtoull_from_user(buf, len, 0, &val);

	if (err)
		return -EINVAL;

	regmap_write(sc2705->regmap, sc2705->addr, (unsigned int)val);

	return len;
}

static const struct file_operations sc2705_value_fops = {
	.open = sc2705_debug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write   = sc2705_debug_write,
	.release = single_release,
};

static int sc2705_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct sc2705 *sc2705;
	struct sc2705_pdata *pdata = dev_get_platdata(&client->dev);
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

	sc2705 = devm_kzalloc(dev, sizeof(*sc2705), GFP_KERNEL);
	if (!sc2705)
		return -ENOMEM;

	sc2705->dev = dev;
	sc2705->irq = client->irq;
	i2c_set_clientdata(client, sc2705);

	if (pdata) {
		sc2705->irq_base = pdata->irq_base;

		sc2705_devs[SC2705_CHARGER_IDX].platform_data =
			pdata->charger_pdata;
		sc2705_devs[SC2705_CHARGER_IDX].pdata_size =
			sizeof(struct sc2705_charger_pdata);
	}

	regmap = devm_regmap_init_i2c(client, &sc2705_regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}
	sc2705->regmap = regmap;
	sc2705->read_cnt = 1;
	/* Restrict IDLE_LP mode */
	ret = regmap_update_bits(sc2705->regmap, SC2705_CONF_B,
				 SC2705_IDLE_LP_MODE_DIS_MASK,
				 0);
	if (ret)
		return ret;

	/* Report and clear POR related fault events before enabling IRQs */
	ret = regmap_read(regmap, SC2705_EVENT_B, &event_b);
	if (ret)
		return ret;

	if (event_b & SC2705_E_TJUNC_POR_MASK) {
		dev_info(dev, "Reset due to TJUNC POR event\n");
		ret = regmap_update_bits(regmap, SC2705_EVENT_B,
					 SC2705_E_TJUNC_POR_MASK,
					 SC2705_E_TJUNC_POR_MASK);
		if (ret)
			return ret;
	}

	ret = regmap_read(regmap, SC2705_EVENT_D, &event_d);
	if (ret)
		return ret;

	if (event_d & SC2705_E_VSYS_POR_MASK) {
		dev_info(dev, "Reset due to VSYS POR event\n");
		ret = regmap_update_bits(regmap, SC2705_EVENT_D,
					 SC2705_E_VSYS_POR_MASK,
					 SC2705_E_VSYS_POR_MASK);
		if (ret)
			return ret;
	}

	sc2705->debugfs_root = debugfs_create_dir(DEBUG_ROOT_DIR, NULL);
	if (IS_ERR_OR_NULL(sc2705->debugfs_root)) {
		WARN_ONCE(!sc2705->debugfs_root,
		     "%s: Failed to create sc2705 i2c debugfs directory\n",
		     DEBUG_ROOT_DIR);
		sc2705->debugfs_root = NULL;
	}

	debugfs_create_file("addr", S_IRUGO | S_IWUSR,
			    sc2705->debugfs_root, sc2705, &sc2705_addr);
	debugfs_create_file("read_cnt", S_IRUGO | S_IWUSR,
			    sc2705->debugfs_root, sc2705, &sc2705_cnt);
	debugfs_create_file("value", S_IRUGO | S_IWUSR,
			    sc2705->debugfs_root, sc2705, &sc2705_value_fops);

	of_platform_populate(client->dev.of_node, NULL, NULL, &client->dev);
	dev_info(dev, "sc2705 ok!!!\n");

	return 0;
}

static int sc2705_remove(struct i2c_client *client)
{
	struct sc2705 *sc2705 = i2c_get_clientdata(client);

	mfd_remove_devices(sc2705->dev);
	regmap_del_irq_chip(sc2705->irq, sc2705->regmap_irq_data);

	return 0;
}

static const struct of_device_id sc2705_of_match[] = {
	{ .compatible = "sprd,sc2705", },
	{ }
};
MODULE_DEVICE_TABLE(of, sc2705_of_match);

static const struct i2c_device_id sc2705_i2c_id[] = {
	{ "sc2705", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sc2705_i2c_id);

static struct i2c_driver sc2705_driver = {
	.driver	= {
		.name	= "sc2705",
		.of_match_table = of_match_ptr(sc2705_of_match),
	},
	.probe		= sc2705_probe,
	.remove		= sc2705_remove,
	.id_table	= sc2705_i2c_id,
};

static int __init sc2705_init(void)
{
	return i2c_add_driver(&sc2705_driver);
}
subsys_initcall_sync(sc2705_init);

static void __exit sc2705_exit(void)
{
	i2c_del_driver(&sc2705_driver);
}
module_exit(sc2705_exit);

MODULE_DESCRIPTION("MFD Core Driver for SC2705");
MODULE_AUTHOR("Roy Im <Roy.Im.Opensource@diasemi.com>");
MODULE_LICENSE("GPL");

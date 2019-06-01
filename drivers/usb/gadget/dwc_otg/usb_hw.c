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
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <asm/irq.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <soc/sprd/sci.h>
#include <soc/sprd/sci_glb_regs.h>
#include <soc/sprd/adi.h>
#include <linux/io.h>//__raw_writel
#include <asm/gpio.h>
#include <asm/hardirq.h>
#include "usb_hw.h"

#if (defined(CONFIG_ARCH_SCX35)||defined(CONFIG_ARCH_SCX35L64))
#define  USB_LDO_NAME	"vddusb"
#define  USB_CLK_NAME   	"clk_usb_ref"
#else
#define	 USB_LDO_NAME    "V_USB"
#define  USB_CLK_NAME    "clk_usb_ref"
#endif

static int    gpio_vbus =0xffffffff ;
static int    gpio_id=0xffffffff;
static uint32_t tune_from_uboot = 0x44073e33;

#define GPIO_INVALID 0xffffffff

extern int in_calibration(void);

static int __init usb_phy_tune_get(char *str)
{
	if (str != NULL) {
		sscanf(&str[0], "%x", &tune_from_uboot);
	}
	if(tune_from_uboot == 0)
		{
		tune_from_uboot = 0x44073e33;
	}
	pr_info("Usb_hw.c: [%s]usb phy tune from uboot: 0x%x\n", __FUNCTION__, tune_from_uboot);
	return 1;
}
__setup("usb_phy_tune=", usb_phy_tune_get);


static void usb_ldo_switch(int is_on)
{
	struct regulator *usb_regulator = NULL;

	if(usb_regulator == NULL){
		usb_regulator = regulator_get(NULL,USB_LDO_NAME);
	}
	if(!IS_ERR_OR_NULL(usb_regulator)){
		if(is_on){
			regulator_set_voltage(usb_regulator,3300000,3300000);
			regulator_enable(usb_regulator);
		}else{
			regulator_disable(usb_regulator);
		}
		regulator_put(usb_regulator);
	}
}
#if defined(CONFIG_ARCH_SC8825)
static int usb_clk_status = 0;
static int usb_clock_enable(int is_on)
{
	struct clk *usb_clock = NULL;

	usb_clock = clk_get(NULL,USB_CLK_NAME);
	if (usb_clock) {
		if (is_on) {
			if(usb_clk_status == 0){
				clk_enable(usb_clock);
				usb_clk_status = 1;
			}
		} else {
			if(usb_clk_status == 1){
				clk_disable(usb_clock);
				usb_clk_status = 0;
			}
		}
	}
	return 0;
}
#endif

#if (defined(CONFIG_ARCH_SCX35)||defined(CONFIG_ARCH_SCX35L64))
static void usb_enable_module(int en)
{
	if (en){
		sci_glb_clr(REG_AON_APB_PWR_CTRL,BIT(0));
		sci_glb_set(REG_AP_AHB_AHB_EB,BIT_USB_EB);
	}else {
		sci_glb_set(REG_AON_APB_PWR_CTRL,BIT(0));
		sci_glb_clr(REG_AP_AHB_AHB_EB,BIT_USB_EB);
	}
}

#else
static void usb_enable_module(int en)
{
	if (en){
		usb_clock_enable(1);
		sprd_greg_clear_bits(REG_TYPE_GLOBAL,BIT(9),GR_CLK_GEN5);
	}else {
		usb_clock_enable(0);
		sprd_greg_set_bits(REG_TYPE_GLOBAL,BIT(9),GR_CLK_GEN5);
		sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,AHB_CTL0_USBD_EN,AHB_CTL0);
	}
}
#endif
void usb_phy_init(struct platform_device *_dev)
{
#ifdef CONFIG_USB_CORE_IP_293A
#if (defined(CONFIG_ARCH_SCX35)||defined(CONFIG_ARCH_SCX35L64))
	/*shark and dolphin are the same value with SPRD ref phone*/
#ifdef CONFIG_OF
	struct device_node *np = _dev->dev.of_node;

	if (of_property_read_u32(np, "tune_value", &tune_from_uboot))
	{
		pr_info("read tune_value error\n");
		return -ENODEV;
	}
	pr_info("Usb_hw.c: [%s]usb phy tune from uboot: 0x%x\n", __FUNCTION__, tune_from_uboot);
#endif

#if defined(CONFIG_ARCH_SCX35L) 
	__raw_writel(tune_from_uboot,REG_AP_AHB_OTG_PHY_TUNE);
#else
	__raw_writel(tune_from_uboot,REG_AP_APB_USB_PHY_TUNE);
#endif

	//sci_glb_set(REG_AP_APB_USB_PHY_TUNE,BIT(9)|BIT(10)|BIT(11)|BIT(20));
#else
		/*
		* tiger PHY reg is different with previous ,
		*7710 has the same core IP with tiger,but PHY reg also diff
		*/
        sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(11), USB_PHY_CTRL);
        sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(10), USB_PHY_CTRL);
        sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(20), USB_PHY_CTRL);
        sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(9), USB_PHY_CTRL);
        sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,BIT(8), USB_PHY_CTRL);
        sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(13), USB_PHY_CTRL);
	sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,BIT(12), USB_PHY_CTRL);
        sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,BIT(15)|BIT(14), USB_PHY_CTRL);
#endif
#else
    if (sprd_greg_read(REG_TYPE_AHB_GLOBAL,CHIP_ID) == CHIP_ID_8810S){
                /*SMIC chip id == 0x88100001*/
                sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,BIT(3)|BIT(2), USB_PHY_CTRL);
                sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(1) | BIT(0), USB_PHY_CTRL);
                sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(9), USB_PHY_CTRL);
                sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(16), USB_PHY_CTRL);
                sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,BIT(17), USB_PHY_CTRL);
                sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,BIT(13), USB_PHY_CTRL);
                sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL, BIT(12), USB_PHY_CTRL);
                sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,BIT(15)|BIT(14), USB_PHY_CTRL);
                sprd_greg_write(REG_TYPE_AHB_GLOBAL,0x28,USB_SPR_REG);
        }else{
                /*
                 * config usb phy controller
                 */
                sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(8), USB_PHY_CTRL);
                sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(17), USB_PHY_CTRL);
                sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,BIT(16), USB_PHY_CTRL);
                sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,BIT(13)|BIT(12), USB_PHY_CTRL);
                sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(15)|BIT(14), USB_PHY_CTRL);
        }
#endif
}

void usb_phy_ahb_rst(void)
{
#if defined(CONFIG_ARCH_SCX35)
#if defined(CONFIG_ARCH_SCX35L)
	sci_glb_set(REG_AP_AHB_AHB_RST,BIT(6));
	mdelay(3);
	sci_glb_clr(REG_AP_AHB_AHB_RST,BIT(6));
	mdelay(3);
	pr_info("Usb_hw.c: [%s]usb phy tune : 0x%x\n", __FUNCTION__, tune_from_uboot);
#else
	sci_glb_set(REG_AP_AHB_AHB_RST,BIT(7));
	mdelay(3);
	sci_glb_clr(REG_AP_AHB_AHB_RST,BIT(7));
	mdelay(3);
#endif
#endif
}

static void usb_startup(void)
{
	usb_ldo_switch(1);
	mdelay(10);
	usb_enable_module(1);
	mdelay(2);
#if (defined(CONFIG_ARCH_SCX35)||defined(CONFIG_ARCH_SCX35L64))
#if defined(CONFIG_ARCH_SCX30G)
	sci_glb_set(REG_AP_AHB_AHB_RST,BIT(5)|BIT(6)|BIT(7));
	mdelay(5);
	sci_glb_clr(REG_AP_AHB_AHB_RST,BIT(5)|BIT(6)|BIT(7));
#else
	sci_glb_set(REG_AP_AHB_AHB_RST,BIT_OTG_SOFT_RST|BIT_OTG_UTMI_SOFT_RST|BIT_OTG_PHY_SOFT_RST);
	mdelay(5);
	sci_glb_clr(REG_AP_AHB_AHB_RST,BIT_OTG_SOFT_RST|BIT_OTG_UTMI_SOFT_RST|BIT_OTG_PHY_SOFT_RST);
	sci_glb_set(REG_AP_AHB_AHB_EB,BIT_USB_EB);
#endif
#else	
	sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,BIT_USB_S_HBIGEIDIAN|BIT_USB_M_HBIGENDIAN,REG_AHB_AHB_CTL3);
	sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT_CLK_USB_REF_EN,REG_AHB_AHB_CTL3);

	sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT_USBD_UTMI_SOFT_RST|BIT_USBPHY_SOFT_RST ,REG_AHB_SOFT_RST);
	mdelay(5);
	sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,BIT_USBD_UTMI_SOFT_RST|BIT_USBPHY_SOFT_RST,REG_AHB_SOFT_RST);
	sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT_USBD_EB,REG_AHB_AHB_CTL0);
#endif
	mdelay(3);
}

void udc_enable(void)
{
	pr_info("%s \n", __func__);
	usb_startup();
}
void udc_disable(void)
{
        pr_info("%s \n", __func__);
        usb_enable_module(0);
        usb_ldo_switch(0);
}


int usb_alloc_vbus_irq(int gpio)
{
	int irq;

	gpio_request(gpio,"sprd_otg");
	gpio_direction_input(gpio);
	irq = gpio_to_irq(gpio);
	gpio_vbus= gpio;

	return irq;
}

void usb_free_vbus_irq(int irq,int gpio)
{
	gpio_free(gpio);
}

int usb_get_vbus_irq(void)
{
	int value;

	value = gpio_to_irq(gpio_vbus);

	return value;
}
int usb_get_vbus_state(void)
{
	int value;
	value = gpio_get_value(gpio_vbus);
	return !!value;
}

void usb_set_vbus_irq_type(int irq, int irq_type)
{
	if (irq_type == VBUS_PLUG_IN)
		irq_set_irq_type(irq, IRQ_TYPE_LEVEL_HIGH);
	else if (irq_type == VBUS_PLUG_OUT)
		irq_set_irq_type(irq, IRQ_TYPE_LEVEL_LOW);
	else {
		pr_warning("error type for usb vbus\n");
	}

	return;
}

#ifndef DWC_DEVICE_ONLY
void charge_pump_set(int gpio,int state)
{
	struct regulator *usb_regulator = NULL;
#define  USB_CHG_PUMP_NAME	"chg_pump"

        if( GPIO_INVALID !=gpio)
        {
		gpio_request(gpio, "chg_ pump");
		gpio_direction_output(gpio,1);
		gpio_set_value(gpio, state);
        }
	else
	{
		if(usb_regulator == NULL){
			usb_regulator = regulator_get(NULL,USB_CHG_PUMP_NAME);
		}
		if(usb_regulator){
			if(state){
				regulator_enable(usb_regulator);
			}else{
				regulator_disable(usb_regulator);
			}
			regulator_put(usb_regulator);
		}
	}
}

int usb_alloc_id_irq(int gpio)
{
	int irq;

	gpio_request(gpio,"USB OTG CABLE");
	gpio_direction_input(gpio);
	irq = gpio_to_irq(gpio);
#if defined(CONFIG_MACH_SPX35EA)
	/**EA board H/W doesn't support OTG**/
	set_irq_flags(irq, IRQF_VALID | IRQF_NOAUTOEN);
#endif
	gpio_id=gpio;

	return irq;
}

void usb_free_id_irq(int irq,int gpio)
{
	gpio_free(gpio);
}

int usb_get_id_irq(void)
{
	int value;

	value = gpio_to_irq(gpio_id);

	return value;
}
int usb_get_id_state(void)
{
#if defined(CONFIG_MACH_SPX35EA)
	/**EA board H/W doesn't support OTG**/
	return 1;
#else
	int value;
	value = gpio_get_value(gpio_id);
	return !!value;
#endif
}

void usb_set_id_irq_type(int irq, int irq_type)
{
	if (irq_type == OTG_CABLE_PLUG_IN)
		irq_set_irq_type(irq, IRQ_TYPE_LEVEL_LOW);
	else if (irq_type == OTG_CABLE_PLUG_OUT)
		irq_set_irq_type(irq, IRQ_TYPE_LEVEL_HIGH);
	else {
		pr_warning("error type for usb vbus\n");
	}

	return;
}
#endif


EXPORT_SYMBOL(udc_disable);
EXPORT_SYMBOL(udc_enable);

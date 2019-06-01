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
#include <linux/kernel.h>
#include <linux/init.h>
#ifndef CONFIG_64BIT
#include <soc/sprd/hardware.h>
#include <soc/sprd/board.h>
#endif
#include <soc/sprd/adi.h>
#include "../common/parse_hwinfo.h"
static void sprd_leds_bltc_rgb_enable(void);
static void sprd_leds_bltc_rgb_disable(void);

int sprd_flash_on(void)
{
	printk("sprd_flash_on \n");
#ifdef CONFIG_ONTIM_FLASH_BY_RGB_DRV
	sprd_leds_bltc_rgb_enable();
#elif defined(CONFIG_ONTIM_LCD_BACKLIGHT_BY_FLASH_DRV)
#else
	sci_adi_clr(SPRD_ADISLAVE_BASE + SPRD_FLASH_OFST, SPRD_FLASH_HIGH_VAL);
	sci_adi_set(SPRD_ADISLAVE_BASE + SPRD_FLASH_OFST, SPRD_FLASH_CTRL_BIT | SPRD_FLASH_LOW_VAL); /*0x3 = 110ma*/
#endif
	return 0;
}

int sprd_flash_high_light(void)
{
	printk("sprd_flash_high_light \n");
#ifdef CONFIG_ONTIM_FLASH_BY_RGB_DRV
	sprd_leds_bltc_rgb_enable();
#elif defined(CONFIG_ONTIM_LCD_BACKLIGHT_BY_FLASH_DRV)
#else
	sci_adi_set(SPRD_ADISLAVE_BASE + SPRD_FLASH_OFST, SPRD_FLASH_CTRL_BIT | SPRD_FLASH_HIGH_VAL); /*0xf = 470ma*/
#endif
	return 0;
}

int sprd_flash_close(void)
{
	printk("sprd_flash_close \n");
#ifdef CONFIG_ONTIM_FLASH_BY_RGB_DRV
	sprd_leds_bltc_rgb_disable();
#elif defined(CONFIG_ONTIM_LCD_BACKLIGHT_BY_FLASH_DRV)
#else
	sci_adi_clr(SPRD_ADISLAVE_BASE + SPRD_FLASH_OFST, SPRD_FLASH_CTRL_BIT | SPRD_FLASH_HIGH_VAL);
#endif
	return 0;
}

/*
+                                 <0X480 0Xf53f8480 0x80>,
+                                 <0X3c0 0Xf53f83c0 0x40>,
+                                 <0X800 0Xf53f8800 0xf0>;
*/

#define ANA_REGS_GLB_BASE               0Xf53f8800 //SCI_IOMAP(0x40038800)
//#define ANA_REG_GLB_ARM_MODULE_EN       SCI_ADDR(ANA_REGS_GLB_BASE, 0x0000)

#define ANA_BTLC_BASE            (SPRD_MISC_BASE + 0x83c0)

#define BLTC_CTRL                               0x0000
#define BLTC_R_PRESCL                   0x0004
#define BLTC_G_PRESCL                   0x0014
#define BLTC_B_PRESCL                   0x0024
#define BLTC_PRESCL_OFFSET              0x0004
#define BLTC_DUTY_OFFSET                0x0004
#define BLTC_CURVE0_OFFSET              0x0008
#define BLTC_CURVE1_OFFSET              0x000C

#define PWM_MOD_COUNTER 0xFF

static inline uint32_t sprd_leds_bltc_rgb_read(uint32_t reg)
{
        return sci_adi_read(reg);
}

static void sprd_bltc_rgb_init(void)
{
        sci_adi_set(ANA_REGS_GLB_BASE, (0x1<<10));//ARM_MODULE_EN-enable pclk
        sci_adi_set(ANA_REGS_GLB_BASE + 0x08, (0x1<<8));//RTC_CLK_EN-enable rtc
        sci_adi_set(ANA_REGS_GLB_BASE + 0x0c,0x1);
        sci_adi_clr(ANA_REGS_GLB_BASE + 0x0c,0x1);
        sci_adi_clr(ANA_REGS_GLB_BASE + 0xa0, (0x1<<2));//SW POWERDOWN DISABLE
	sci_adi_set(ANA_REGS_GLB_BASE + 0xa0, sprd_leds_bltc_rgb_read(ANA_REGS_GLB_BASE + 0xa0)|(0x15<<8));
        //sci_adi_clr(ANA_REGS_GLB_BASE + 0xa0, (0x1f<<8));//CURRENT CONTROL DEFAULT
}

static void sprd_leds_bltc_rgb_enable(void)
{
        sprd_bltc_rgb_init();

                sci_adi_set(ANA_BTLC_BASE + BLTC_CTRL, (0x1<<0)|(0x1<<1));
                sci_adi_write((ANA_BTLC_BASE + BLTC_R_PRESCL + BLTC_DUTY_OFFSET), (0xff<<8)|PWM_MOD_COUNTER,0xffff);

                sci_adi_set(ANA_BTLC_BASE + BLTC_CTRL, (0x1<<4)|(0x1<<5));
                sci_adi_write((ANA_BTLC_BASE + BLTC_G_PRESCL + BLTC_DUTY_OFFSET), (0xff<<8)|PWM_MOD_COUNTER,0xffff);

                sci_adi_set(ANA_BTLC_BASE + BLTC_CTRL, (0x1<<8)|(0x1<<9));
                sci_adi_write((ANA_BTLC_BASE + BLTC_B_PRESCL + BLTC_DUTY_OFFSET), (0xff<<8)|PWM_MOD_COUNTER,0xffff);

        //printk("sprd_leds_bltc_rgb_enable\n");
}

static void sprd_leds_bltc_rgb_disable(void)
{
                sci_adi_clr(ANA_BTLC_BASE + BLTC_CTRL, (0x1<<0));

                sci_adi_clr(ANA_BTLC_BASE + BLTC_CTRL, (0x1<<4));

                sci_adi_clr(ANA_BTLC_BASE + BLTC_CTRL, (0x1<<8));

        sci_adi_set(ANA_REGS_GLB_BASE + 0xa0, sprd_leds_bltc_rgb_read(ANA_REGS_GLB_BASE + 0xa0)|(0x1<<2));//SW POWERDOWN ENABLE

        //printk("sprd_leds_bltc_rgb_disable\n");
        //printk("reg:0x%08X set_val:0x%08X brightness:%1d\n", brgb->sprd_arm_module_en_addr + 0xa0, \
        //           sprd_leds_bltc_rgb_read(brgb->sprd_arm_module_en_addr + 0xa0),brgb->value);
}




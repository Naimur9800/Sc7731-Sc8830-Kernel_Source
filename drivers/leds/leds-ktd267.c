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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/err.h>
/*wdm+++*/
#include <linux/hrtimer.h>
/*---*/
#include <linux/delay.h>
#include <linux/slab.h>
#include <soc/sprd/hardware.h>
#include <soc/sprd/adi.h>
#include <soc/sprd/adc.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <soc/sprd/board.h>
#include <linux/clk.h>
#include <soc/sprd/adi.h>
#include <linux/io.h>

//#define SPRD_TORCH_DBG
#define SPRD_TORCH_DBG_LVL	KERN_WARN
#ifdef SPRD_TORCH_DBG
#define ENTER printk(SPRD_TORCH_DBG_LVL "[SPRD_TORCH_DBG] func: %s  line: %04d\n", __func__, __LINE__);
#define PRINT_DBG(x...)  printk(SPRD_TORCH_DBG_LVL "[SPRD_TORCH_DBG] " x)
#define PRINT_INFO(x...)  printk(KERN_INFO "[SPRD_TORCH_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_WARN "[SPRD_TORCH_WARN] " x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[SPRD_TORCH_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#else
#define ENTER
#define PRINT_DBG(x...)
#define PRINT_INFO(x...)  printk(KERN_INFO "[SPRD_TORCH_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_WARN "[SPRD_TORCH_WARN] " x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[SPRD_TORCH_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#endif

#define SPRD_TORCH_ON		1
#define SPRD_TORCH_OFF		0    

//#define SPRD_TORCH_CONTROL_BY_PWM	// control torch by pwm
#define SPRD_TORCH_CONTROL_BY_GPIOS	// control torch by gpio(s)

/* sprd torch */
struct sprd_torch {
	struct platform_device *dev;
	struct mutex mutex;
	struct work_struct work;
	spinlock_t value_lock;
	u32 brightness;
	struct led_classdev cdev;
	int enabled;
#if defined(SPRD_TORCH_CONTROL_BY_GPIOS)
	unsigned gpio_enf;
	unsigned gpio_enm;
#elif defined(SPRD_TORCH_CONTROL_BY_PWM)
	struct clk      *clk;
	u32		pwm_index;
	int ctl_pin;
	int ctl_pin_level;
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	early_suspend;
#endif
	int (*sprd_torch_on)(struct sprd_torch *led);
	int (*sprd_torch_high_light)(struct sprd_torch *led);
	int (*sprd_torch_close)(struct sprd_torch *led);
	/*wdm+++*/
	struct hrtimer timer;
        ktime_t limit_time;
	/*---*/
};
static struct sprd_torch *led_info;
/*wdm+++*/
static struct workqueue_struct *g_timeout_work_queue;
static void timeout_work(struct work_struct *work);
static DECLARE_WORK(g_timeout_work, timeout_work);
/*---*/
#if defined(SPRD_TORCH_CONTROL_BY_GPIOS)

#define GPIO_CAM_FLASH_FLASHTORCH  235    
#define GPIO_CAM_FLASH_EN          234


#define SPRD_FLASH_ON		1
#define SPRD_FLASH_OFF		0
static int sprd_torch_on_by_2gpios(struct sprd_torch *led)
{
/*
	PRINT_DBG("<%s> en[%d] m[%d] val[%d] L%d.\n", __func__, led->gpio_enf, led->gpio_enm, led->brightness, __LINE__);
	gpio_request(led->gpio_enf,"torch_en");
	gpio_direction_output(led->gpio_enf, SPRD_TORCH_OFF);
	gpio_free(led->gpio_enf);

	gpio_request(led->gpio_enm,"torch_mode");
	gpio_direction_output(led->gpio_enm, SPRD_TORCH_ON);
	gpio_free(led->gpio_enm);
*/	
	
    gpio_request(GPIO_CAM_FLASH_FLASHTORCH,"cam_flash_torch");
	gpio_direction_output(GPIO_CAM_FLASH_FLASHTORCH, SPRD_FLASH_OFF);
	gpio_free(GPIO_CAM_FLASH_FLASHTORCH);
	gpio_request(GPIO_CAM_FLASH_EN,"cam_flash_en");
	gpio_direction_output(GPIO_CAM_FLASH_EN, SPRD_FLASH_ON);
	gpio_free(GPIO_CAM_FLASH_EN);
    return 0 ;
}

static int sprd_torch_high_light_by_2gpios(struct sprd_torch *led)
{
/*
	PRINT_DBG("<%s> en[%d] m[%d] val[%d] L%d.\n", __func__, led->gpio_enf, led->gpio_enm, led->brightness, __LINE__);
	gpio_request(led->gpio_enf,"torch_en");
	gpio_direction_output(led->gpio_enf, SPRD_TORCH_ON);
	gpio_free(led->gpio_enf);

	gpio_request(led->gpio_enm,"torch_mode");
	gpio_direction_output(led->gpio_enm, SPRD_TORCH_OFF);
	gpio_free(led->gpio_enm);
*/
	gpio_request(GPIO_CAM_FLASH_FLASHTORCH,"cam_flash_flash");
	gpio_direction_output(GPIO_CAM_FLASH_FLASHTORCH, SPRD_FLASH_ON);
	gpio_free(GPIO_CAM_FLASH_FLASHTORCH);

	gpio_request(GPIO_CAM_FLASH_EN,"cam_flash_en");
	gpio_direction_output(GPIO_CAM_FLASH_EN, SPRD_FLASH_ON);
	gpio_free(GPIO_CAM_FLASH_EN);
	return 0;
}

static int sprd_torch_close_by_2gpios(struct sprd_torch *led)
{
/*
	PRINT_DBG("<%s> en[%d] m[%d] val[%d] L%d.\n", __func__, led->gpio_enf, led->gpio_enm, led->brightness, __LINE__);
	gpio_request(led->gpio_enf,"torch_en");
	gpio_direction_output(led->gpio_enf, SPRD_TORCH_OFF);
	gpio_free(led->gpio_enf);

	gpio_request(led->gpio_enm,"torch_mode");
	gpio_direction_output(led->gpio_enm, SPRD_TORCH_OFF);
	gpio_free(led->gpio_enm);
*/
	gpio_request(GPIO_CAM_FLASH_EN,"cam_flash_en");
	gpio_direction_output(GPIO_CAM_FLASH_EN, SPRD_FLASH_OFF);
	gpio_free(GPIO_CAM_FLASH_EN);

	gpio_request(GPIO_CAM_FLASH_FLASHTORCH,"cam_flash_torch");
	gpio_direction_output(GPIO_CAM_FLASH_FLASHTORCH, SPRD_FLASH_OFF);
	gpio_free(GPIO_CAM_FLASH_FLASHTORCH);
	return 0;
}

#endif	/* SPRD_TORCH_CONTROL_BY_GPIOS */


static int sprd_torch_get_status_by_2gpio(struct sprd_torch *led)
{
    unsigned int v = 0 ;
	gpio_request(GPIO_CAM_FLASH_EN,"cam_flash_en");
    v  = __gpio_get_value(GPIO_CAM_FLASH_EN) ;
	gpio_free(GPIO_CAM_FLASH_EN);
    return v ;
}

static int sprd_torch_on_by_isink(struct sprd_torch *led)
{
	PRINT_DBG("<%s> en[%d] m[%d] val[%d] L%d.\n", __func__, led->gpio_enf, led->gpio_enm, led->brightness, __LINE__);
	sci_adi_clr(SPRD_ADISLAVE_BASE + SPRD_FLASH_OFST, SPRD_FLASH_HIGH_VAL);
	sci_adi_set(SPRD_ADISLAVE_BASE + SPRD_FLASH_OFST, SPRD_FLASH_CTRL_BIT | SPRD_FLASH_LOW_VAL); /*0x3 = 110ma*/
	return 0;
}

static int sprd_torch_high_light_by_isink(struct sprd_torch *led)
{
	PRINT_DBG("<%s> en[%d] m[%d] val[%d] L%d.\n", __func__, led->gpio_enf, led->gpio_enm, led->brightness, __LINE__);
	sci_adi_set(SPRD_ADISLAVE_BASE + SPRD_FLASH_OFST, SPRD_FLASH_CTRL_BIT | SPRD_FLASH_HIGH_VAL); /*0xf = 470ma*/
	return 0;
}

static int sprd_torch_close_by_isink(struct sprd_torch *led)
{
	PRINT_DBG("<%s> en[%d] m[%d] val[%d] L%d.\n", __func__, led->gpio_enf, led->gpio_enm, led->brightness, __LINE__);
	sci_adi_clr(SPRD_ADISLAVE_BASE + SPRD_FLASH_OFST, SPRD_FLASH_CTRL_BIT | SPRD_FLASH_HIGH_VAL);
	return 0;
}

#if defined(SPRD_TORCH_CONTROL_BY_PWM)
/* register definitions */
#define        PWM_PRESCALE    (0x0000)
#define        PWM_CNT         (0x0004)
#define        PWM_TONE_DIV    (0x0008)
#define        PWM_PAT_LOW     (0x000C)
#define        PWM_PAT_HIG     (0x0010)

#define        PWM_ENABLE      (1 << 8)
#define        PWM_DUTY            (1 << 8)
#define        PWM_MOD             0
#define        PWM_DIV             0x190
#define        PWM_LOW             0xffff
#define        PWM_HIG             0xffff
#ifdef         CONFIG_ARCH_SCX15
#define        PWM_SCALE           0xd
#else
#define        PWM_SCALE       1
#endif
#define        PWM2_SCALE           0x0
#define        PWM_REG_MSK     0xffff
#define        PWM_MOD_MAX     0xff
#define        PWM_DUTY_MAX     0x7f
static DEFINE_SPINLOCK(clock_en_lock);
static void sprd_torch_pwm_clk_en(struct sprd_torch *led, int enable)
{
        unsigned long spin_lock_flags;
        static int current_state = 0;

        spin_lock_irqsave(&clock_en_lock, spin_lock_flags);
        if (1 == enable) {
                if (0 == current_state) {
                        clk_prepare_enable(led->clk);
                        current_state = 1;
                }
        } else {
                if (1 == current_state) {
                        clk_disable_unprepare(led->clk);
                        current_state = 0;
                }
        }
        spin_unlock_irqrestore(&clock_en_lock, spin_lock_flags);

        return;
}

static inline uint32_t pwm_read(int index, uint32_t reg)
{
	//this is the D-die PWM controller, here we use PWM2(index=3) for external backlight control.
       return __raw_readl(SPRD_PWM_BASE + index * 0x20 + reg);
}

static void pwm_write(int index, uint32_t value, uint32_t reg)
{
	//this is the D-die PWM controller, here we use PWM2(index=3) for external backlight control.
       __raw_writel(value, SPRD_PWM_BASE + index * 0x20 + reg);
}

//sprd used PWM2(mapped to gpio190) for external backlight control.
static int sprd_torch_control_by_pwm(struct sprd_torch *led)
{
	u32 led_level;

	if (led->brightness == 0) {
		/* disable backlight */
		pwm_write(led->pwm_index, 0, PWM_PRESCALE);
		sprd_torch_pwm_clk_en(led, 0);
		PRINT_INFO("[pwm] disabled\n");
	} else {
		led_level = led->brightness & PWM_MOD_MAX;
		//led_level = (led_level * (PWM_DUTY_MAX+1) / (PWM_MOD_MAX+1)) + 10;
		//led_level = led_level * 67 / 100;
		if(led_level == 100)
			led_level = 35;
		if(led_level < 8)
			led_level = 8;
		PRINT_INFO("[pwm] brightness = %d, led_level = %d\n", led->brightness, led_level);
		sprd_torch_pwm_clk_en(led, 1);
		pwm_write(led->pwm_index, PWM2_SCALE, PWM_PRESCALE);
		pwm_write(led->pwm_index, (led_level << 8) | PWM_MOD_MAX, PWM_CNT);
		pwm_write(led->pwm_index, PWM_REG_MSK, PWM_PAT_LOW);
		pwm_write(led->pwm_index, PWM_REG_MSK, PWM_PAT_HIG);
		pwm_write(led->pwm_index, PWM_ENABLE, PWM_PRESCALE);
	}
	return 0;
}

int sprd_set_flash(u32 flash_level)
{
    struct sprd_torch *led = led_info;

	/*wdm+++*/
	if(185 == flash_level)
	{
		hrtimer_start(&led->timer, led->limit_time, HRTIMER_MODE_REL);
	}
	else if(0 == flash_level)
	{
		hrtimer_cancel(&led->timer);
                //cancel_work_sync(&g_timeout_work);
	}
	/*---*/
	
	led->brightness = flash_level;

	sprd_torch_control_by_pwm(led);

	return 0;
}
EXPORT_SYMBOL(sprd_set_flash);

#endif /* SPRD_TORCH_CONTROL_BY_PWM */

static void sprd_torch_controler_mach(struct sprd_torch *led)
{
	#ifdef SPRD_TORCH_CONTROL_BY_GPIOS
		led->sprd_torch_on = sprd_torch_on_by_2gpios;
		led->sprd_torch_high_light = sprd_torch_high_light_by_2gpios;
		led->sprd_torch_close = sprd_torch_close_by_2gpios;
	#elif defined(SPRD_TORCH_CONTROL_BY_PWM)
		led->sprd_torch_on = sprd_torch_control_by_pwm;
		led->sprd_torch_high_light = sprd_torch_control_by_pwm;
		led->sprd_torch_close = sprd_torch_control_by_pwm;
	#else	/* SPRD_TORCH_CONTROL_BY_GPIOS */
		// for sprd smart phone
		led->sprd_torch_on = sprd_torch_on_by_isink;
		led->sprd_torch_high_light = sprd_torch_high_light_by_isink;
		led->sprd_torch_close = sprd_torch_close_by_isink;
	#endif	/* SPRD_TORCH_CONTROL_BY_GPIOS */
}

static void sprd_torch_work(struct work_struct *work)
{
	struct sprd_torch *led = container_of(work, struct sprd_torch, work);
	unsigned long flags;

	mutex_lock(&led->mutex);
	spin_lock_irqsave(&led->value_lock, flags);
	if (led->brightness == (u32) LED_OFF) {
		led->sprd_torch_close(led);
	} else if (led->brightness <= (u32) LED_HALF) {
		led->sprd_torch_on(led);
	} else {	//(led->brightness > LED_HALF)
		led->sprd_torch_high_light(led);
	}
	spin_unlock_irqrestore(&led->value_lock, flags);
	mutex_unlock(&led->mutex);
}

static void sprd_torch_set(struct led_classdev *led_cdev,
			   enum led_brightness brightness)
{
	struct sprd_torch *led = container_of(led_cdev, struct sprd_torch, cdev);
	unsigned long flags;
	spin_lock_irqsave(&led->value_lock, flags);
	led->brightness = (u32) brightness;
	spin_unlock_irqrestore(&led->value_lock, flags);

	schedule_work(&led->work);	
}

static enum led_brightness sprd_torch_get(struct led_classdev *led_cdev)
{
	struct sprd_torch *led = container_of(led_cdev, struct sprd_torch, cdev);
    return sprd_torch_get_status_by_2gpio(led) == 0 ? LED_OFF : LED_FULL ;
}

static void sprd_torch_shutdown(struct platform_device *dev)
{
	struct sprd_torch *led = platform_get_drvdata(dev);

	mutex_lock(&led->mutex);
	led->sprd_torch_close(led);
	mutex_unlock(&led->mutex);
}

#ifdef CONFIG_EARLYSUSPEND
static void sprd_torch_early_suspend(struct early_suspend *es)
{
	PRINT_INFO("sprd_torch_early_suspend\n");
}

static void sprd_torch_late_resume(struct early_suspend *es)
{
	PRINT_INFO("sprd_torch_late_resume\n");
}
#endif

#ifdef CONFIG_OF
static int sprd_torch_parse_dt(struct platform_device *pdev, struct sprd_torch *led)
{
	int ret;
	struct device_node *np = pdev->dev.of_node;
	struct resource *pwm_res;
	
#if defined(SPRD_TORCH_CONTROL_BY_GPIOS)
	led->gpio_enf= of_get_gpio( pdev->dev.of_node,0);
	led->gpio_enm = of_get_gpio( pdev->dev.of_node,1);
	if (led->gpio_enf < 0 || led->gpio_enm < 0)
		return -ENODEV;
	else
		return 0;
#elif defined(SPRD_TORCH_CONTROL_BY_PWM)
	pwm_res = kzalloc(sizeof(*pwm_res), GFP_KERNEL);
	if (!pwm_res) {
		dev_err(pdev, "sprd_backlight Could not allocate struct resource");
		return -ENOMEM;
	}
	ret = of_property_read_u32(np, "start", &pwm_res->start);
	if(ret){
		dev_err(pdev, "fail to get resource.start\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "end", &pwm_res->end);
	if(ret){
		dev_err(pdev, "fail to get resource.end\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "ctl_pin", &led->ctl_pin);
	ret = of_property_read_u32(np, "ctl_pin_level", &led->ctl_pin_level);
	ret = of_property_read_u32(np, "flags", &pwm_res->flags);
	if(ret){
		dev_err(pdev, "fail to get resource.flags\n");
		goto fail;
	}
	led->pwm_index = pwm_res->start;
	return 0;
fail:
	kfree(pwm_res);
	return -ENODEV;
#else	// none
	return 0;
#endif
}
#endif	/* CONFIG_OF */

/*wdm+++*/
static void timeout_work(struct work_struct *work)
{
#if defined SPRD_TORCH_CONTROL_BY_PWM
  	sprd_set_flash(0);      
#elif defined SPRD_TORCH_CONTROL_BY_GPIOS
    sprd_torch_close_by_2gpios(NULL);
#endif
}

static enum hrtimer_restart timeout_timer_func(struct hrtimer *data)
{

        queue_work(g_timeout_work_queue, &g_timeout_work);
        return HRTIMER_NORESTART;
}
/*---*/
static int sprd_torch_probe(struct platform_device *pdev)
{
	struct sprd_torch *led;
	int ret;
	#ifdef SPRD_TORCH_CONTROL_BY_PWM
	struct clk* ext_26m = NULL;
	char pwm_clk_name[32];
	#endif	/* SPRD_TORCH_CONTROL_BY_PWM */
	
	led = kzalloc(sizeof(struct sprd_torch), GFP_KERNEL);
	if (led == NULL) {
		dev_err(&pdev->dev, "No memory for device\n");
		return -ENOMEM;
	}
	led_info = led;
#ifdef CONFIG_OF
	if(pdev->dev.of_node){
		ret = sprd_torch_parse_dt(pdev, led);
		if (ret){
			printk("Can't get resource");
			ret = -ENODEV;
			goto config_of_fail;
		}
	}else{
		printk("dev.of_node is NULL");
		ret = -ENODEV;
		goto config_of_fail;
	}
#endif	/* CONFIG_OF */
	led->cdev.brightness_set = sprd_torch_set;
	//led->cdev.default_trigger = "heartbeat";
	led->cdev.brightness_get = sprd_torch_get;
	led->cdev.default_trigger = "none";
	led->cdev.name = "torch-ktd267";
	led->cdev.flags |= LED_CORE_SUSPENDRESUME;
	led->enabled = 0;

#if defined(SPRD_TORCH_CONTROL_BY_PWM)
	/*fixme, the pwm's clk name must like this:clk_pwmx*/
	sprintf(pwm_clk_name, "%s%d", "clk_pwm", led->pwm_index);
	led->clk = clk_get(&pdev->dev, pwm_clk_name);
	if (IS_ERR(led->clk)) {
		printk("Can't get pwm's clk");
		ret = -ENODEV;
		goto get_clk_fail;
	}
	ext_26m = clk_get(NULL, "ext_26m");
	if (IS_ERR(ext_26m)) {
		printk("Can't get pwm's ext_26m");
		ret = -ENODEV;
		goto get_clk_fail;
	}
	clk_set_parent(led->clk,ext_26m);
#endif	/*SPRD_TORCH_CONTROL_BY_PWM*/

	spin_lock_init(&led->value_lock);
	mutex_init(&led->mutex);
	INIT_WORK(&led->work, sprd_torch_work);
	led->brightness = (u32) LED_OFF;
	platform_set_drvdata(pdev, led);

	/* register our new led device */

	ret = led_classdev_register(&pdev->dev, &led->cdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "led_classdev_register failed\n");
		goto get_clk_fail;
	}

	sprd_torch_controler_mach(led);
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	led->early_suspend.suspend = sprd_torch_early_suspend;
	led->early_suspend.resume  = sprd_torch_late_resume;
	led->early_suspend.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	register_early_suspend(&led->early_suspend);
#endif

	led->sprd_torch_close(led);//disabled by default
		
	/*wdm+++*/
	led->limit_time = ktime_set(0, 1000000000);  /* 1000 ms */
	g_timeout_work_queue = create_workqueue("timeout");
        if (g_timeout_work_queue == NULL) {
                ret = -ENOMEM;
                goto err_create_work_queue;
        }

	hrtimer_init(&led->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        led->timer.function = timeout_timer_func;
	/*---*/
	return 0;

/*wdm+++*/
err_create_work_queue:
/*---*/	
get_clk_fail:
config_of_fail:
	kfree(led);
	return ret;
}

static int sprd_torch_remove(struct platform_device *dev)
{
	struct sprd_torch *led = platform_get_drvdata(dev);

	led_classdev_unregister(&led->cdev);
	flush_scheduled_work();
	led->brightness = (u32) LED_OFF;
	led->enabled = 1;
	led->sprd_torch_close(led);
	kfree(led);
	/*wdm+++*/
	destroy_workqueue(g_timeout_work_queue);
	/*---*/

	return 0;
}

static const struct of_device_id torch_of_match[] = {
	{ .compatible = "sprd,torch-ktd267", },
	{ }
};

static struct platform_driver sprd_torch_driver = {
	.driver = {
		.name  = "torch_ktd267",
		.owner = THIS_MODULE,
		.of_match_table = torch_of_match,
	},
	.probe    = sprd_torch_probe,
	.remove   = sprd_torch_remove,
	.shutdown = sprd_torch_shutdown,
};

static int __init sprd_torch_init(void)
{
	return platform_driver_register(&sprd_torch_driver);
}

static void sprd_torch_exit(void)
{
	platform_driver_unregister(&sprd_torch_driver);
}

module_init(sprd_torch_init);
module_exit(sprd_torch_exit);

MODULE_AUTHOR("Chen wei <weicn.chen@spreadtrum.com>");
MODULE_DESCRIPTION("Sprd torch driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:torch");


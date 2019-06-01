/*
 *  Ontim Backlight Driver
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/fb.h>
#include <linux/ontim_backlight.h>
#include <linux/gpio.h> 
#ifdef CONFIG_PM_RUNTIME
#include <linux/pm_runtime.h>
#endif
struct ontim_backlight_device{
	struct backlight_device * bd;
	int bl_intensity;
	struct ontim_bl_platform_data * pdata;
	spinlock_t bl_lock;
};
#ifdef CONFIG_MMP_FB
extern struct mmp_mach_panel_info *get_lcd_detect_mipi_info(void);//add by liuwei
#else
extern struct pxa168fb_mach_info *get_lcd_detect_mipi_info(void);//add by liuwei
#endif

/* Flag to signal when the battery is low */
#define ONTIM_BL_BATTLOW       BL_CORE_DRIVER1
#define ONTIM_BL_RUNTIME_SUSPEND      BL_CORE_DRIVER2

static int ontim_bl_send_intensity(struct backlight_device *bd)
{
	int intensity = bd->props.brightness;
	unsigned long flags;
	#ifdef CONFIG_MMP_FB
	struct mmp_mach_panel_info *lcd_mipi_info= get_lcd_detect_mipi_info();//modify by liuwei
	#else
	struct pxa168fb_mach_info *lcd_mipi_info= get_lcd_detect_mipi_info();//modify by liuwei
	#endif
	struct ontim_backlight_device* ontim_bl_dev=(struct ontim_backlight_device*)dev_get_drvdata(&bd->dev);

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.state & BL_CORE_FBBLANK)
		intensity = 0;
	if (bd->props.state & BL_CORE_SUSPENDED)
		intensity = 0;
	if (bd->props.state & ONTIM_BL_RUNTIME_SUSPEND)
		intensity = 0;
	if ((ontim_bl_dev)&&(ontim_bl_dev->bl_intensity != intensity))
	{
		if (ontim_bl_dev->pdata)
		{
			if (bd->props.state & ONTIM_BL_BATTLOW)
				intensity &= ontim_bl_dev->pdata->limit_mask;
			//printk(KERN_INFO "%s: intensity = %d, ctrl_type = %d;bl_gpio = %d;pwm_id=%d\n",__func__
			//	,intensity,ontim_bl_dev->pdata->ctrl_type,ontim_bl_dev->pdata->bl_gpio,ontim_bl_dev->pdata->pwm_id);

			
			switch(ontim_bl_dev->pdata->ctrl_type)
			{
				case ONTIM_BACKLIGHT_CTRL_GPIO_ONOFF:
				{
					int bl_gpio=ontim_bl_dev->pdata->bl_gpio;
					if (bl_gpio < 0) break;
					
					spin_lock_irqsave(&ontim_bl_dev->bl_lock, flags);
					if (intensity)	
					{
						gpio_direction_output(bl_gpio, 1);
						printk(KERN_INFO "%s: Set %d to 1\n",__func__,bl_gpio);
					}
					else
					{
						gpio_direction_output(bl_gpio, 0);
						printk(KERN_INFO "%s: Set %d to 0\n",__func__,bl_gpio);
					}
					spin_unlock_irqrestore(&ontim_bl_dev->bl_lock, flags);
					break;
				}
				case ONTIM_BACKLIGHT_CTRL_PWM:
				{
					break;
				}
				case ONTIM_BACKLIGHT_CTRL_LCDC:
				{
					//modify by liuwei start
					#ifdef CONFIG_MMP_FB
						if(lcd_mipi_info->dsi_panel_backlight_config)						
							lcd_mipi_info->dsi_panel_backlight_config(intensity);
					#else
					if (gfx_info.fbi[0])							
						if (gfx_info.fbi[0]->bl.set_backlight)								
							gfx_info.fbi[0]->bl.set_backlight(gfx_info.fbi[0],intensity);
					#endif
					//modify by liuwei end
					break;
				}
				default :
				{
					break;
				}
			}

			if (ontim_bl_dev->pdata->set_bl_intensity)
				ontim_bl_dev->pdata->set_bl_intensity(intensity);

			if (ontim_bl_dev->pdata->kick_battery)
				ontim_bl_dev->pdata->kick_battery();
		}
		ontim_bl_dev->bl_intensity = intensity;
		
	}
	return 0;
}

static int ontim_bl_get_intensity(struct backlight_device *bd)
{
	struct ontim_backlight_device* ontim_bl_dev=(struct ontim_backlight_device*)dev_get_drvdata(&bd->dev);
	if (ontim_bl_dev)
		return ontim_bl_dev->bl_intensity;
	else
		return 0;
}

static const struct backlight_ops genericbl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.get_brightness = ontim_bl_get_intensity,
	.update_status  = ontim_bl_send_intensity,
};
#ifdef CONFIG_PM_RUNTIME
static int backlight_runtime_suspend(struct device *dev)
{
	struct backlight_device *bd = (struct backlight_device *)dev_get_drvdata(dev);

	mutex_lock(&bd->ops_lock);
	bd->props.state |= ONTIM_BL_RUNTIME_SUSPEND;
	backlight_update_status(bd);
	mutex_unlock(&bd->ops_lock);
	return 0;
}

static int backlight_runtime_resume(struct device *dev)
{
	struct backlight_device *bd = (struct backlight_device *)dev_get_drvdata(dev);

	mutex_lock(&bd->ops_lock);
	bd->props.state &= ~ONTIM_BL_RUNTIME_SUSPEND;
	backlight_update_status(bd);  
	mutex_unlock(&bd->ops_lock);
	printk(KERN_INFO "[kernel]:%s end.\n",__func__);
	return 0;
}

static const struct dev_pm_ops backlight_pm_ops = {
	SET_RUNTIME_PM_OPS(backlight_runtime_suspend, backlight_runtime_resume,
			   NULL)
};
#endif /* CONFIG_PM_RUNTIME */
static int ontim_bl_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct ontim_bl_platform_data * pdata = pdev->dev.platform_data;
	const char *name = "ontim-backlight";
	struct backlight_device *bd;
	struct ontim_backlight_device* ontim_bl_dev;
	
	ontim_bl_dev = kzalloc(sizeof(struct ontim_backlight_device), GFP_KERNEL);
	if (!ontim_bl_dev)
		return ERR_PTR(-ENOMEM);

	ontim_bl_dev->pdata = pdata;
	ontim_bl_dev->bl_lock= __SPIN_LOCK_UNLOCKED(pdata->name);
	switch(pdata->ctrl_type)
	{
		case ONTIM_BACKLIGHT_CTRL_GPIO_ONOFF:
		{
			int bl_gpio=ontim_bl_dev->pdata->bl_gpio;
			if (bl_gpio < 0) break;
			gpio_request(bl_gpio,pdata->name);	
			break;
		}
		case ONTIM_BACKLIGHT_CTRL_PWM:
		{
			break;
		}
		case ONTIM_BACKLIGHT_CTRL_LCDC:
		{
			break;
		}
		default :
		{
			break;
		}
	}
	
	if (!pdata->limit_mask)
		pdata->limit_mask = -1;

	if (pdata->name)
		name = pdata->name;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = pdata->max_intensity;
	bd = backlight_device_register(name, &pdev->dev, ontim_bl_dev, &genericbl_ops,
				       &props);
	if (IS_ERR (bd))
	{
		kfree(ontim_bl_dev);
		return PTR_ERR (bd);
	}

	platform_set_drvdata(pdev, bd);

	bd->props.power = FB_BLANK_UNBLANK;
	bd->props.brightness = pdata->default_intensity;
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_enable(&pdev->dev);
	pm_runtime_forbid(&pdev->dev);
#else
	backlight_update_status(bd);
#endif
	printk("Ontim Backlight Driver Initialized.\n");
	return 0;
}

static int ontim_bl_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);
	
	#ifdef CONFIG_PM_RUNTIME
	pm_runtime_disable(&pdev->dev);
	#endif
	bd->props.power = 0;
	bd->props.brightness = 0;
	backlight_update_status(bd);

	backlight_device_unregister(bd);

	printk("Ontim Backlight Driver Unloaded\n");
	return 0;
}

static struct platform_driver ontim_bl_driver = {
	.probe		= ontim_bl_probe,
	.remove		= ontim_bl_remove,
	.driver		= {
		.name	= "ontim_backlight",
		#ifdef CONFIG_PM_RUNTIME
		.pm	= &backlight_pm_ops,
		#endif
	},
};

module_platform_driver(ontim_bl_driver);

MODULE_AUTHOR("Richard Purdie <rpurdie@rpsys.net>");
MODULE_DESCRIPTION("Generic Backlight Driver");
MODULE_LICENSE("GPL");

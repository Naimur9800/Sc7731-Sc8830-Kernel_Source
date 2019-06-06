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
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/err.h>
#ifdef CONFIG_OF
#include <linux/of_device.h>
#endif

#include <linux/sipc.h>

#include <linux/dual_sim_plug.h>

#define DELAY_COUNT  1000

#define VALUE_SIM1_PLUGIN  0x11
#define VALUE_SIM1_PLUGOUT  0x10
#define VALUE_SIM2_PLUGIN  0x21
#define VALUE_SIM2_PLUGOUT  0x20


#define SMSG_SIM_PLUG 0xA5A5

int flag_low_count_sim1 = 0;
int flag_high_count_sim1 = 0;

int flag_low_count_sim2 = 0;
int flag_high_count_sim2 = 0;

struct dual_sim_plug_device{
  struct dual_sim_plug_init_data *init;
	struct task_struct *th1;
	int irq_cd1;
	int irq_cd2;
	unsigned int sim1_detect_gpio;
	unsigned int sim2_detect_gpio;
};

static irqreturn_t sim1_irq_cd(int irq, void *dev_id)
{
	struct dual_sim_plug_device *dual_sim_plug = (struct dual_sim_plug_device *)dev_id;

  struct smsg msend;
	int  flag = 0;

	//printk(KERN_INFO "Enter sim1_irq_cd\n");

	flag = gpio_get_value(dual_sim_plug->sim1_detect_gpio);

	//printk(KERN_INFO "Enter sim1_irq_cd,flag = %d\n",flag);

	if(flag == 0)
	{
		flag_low_count_sim1++;
		flag_high_count_sim1 = 0;
	}
	else
  {
		flag_high_count_sim1++;
		flag_low_count_sim1= 0;
  }

	if(flag_low_count_sim1 >= DELAY_COUNT)
	{
		flag_low_count_sim1 = 0;
		irq_set_irq_type(irq,IRQF_TRIGGER_HIGH);
		printk(KERN_INFO "Enter sim1_irq_cd,flag = low\n");
		smsg_set(&msend,dual_sim_plug->init->channel,SMSG_TYPE_CMD,SMSG_SIM_PLUG,VALUE_SIM1_PLUGIN);//plug in
		smsg_send(dual_sim_plug->init->dst,&msend,-1);
		printk(KERN_INFO "dual_sim_plug->init->dst = %d,msend.channel = %d,msend.type = %d,msend.flag = %d,msend.value = %d\n",dual_sim_plug->init->dst,msend.channel,msend.type,msend.flag,msend.value);

	}
  else if(flag_high_count_sim1 >= DELAY_COUNT)
  {
		flag_high_count_sim1 = 0;
	  irq_set_irq_type(irq,IRQF_TRIGGER_LOW);
		printk(KERN_INFO "Enter sim1_irq_cd,flag = high\n");
		smsg_set(&msend,dual_sim_plug->init->channel,SMSG_TYPE_CMD,SMSG_SIM_PLUG,VALUE_SIM1_PLUGOUT);//plug out
		smsg_send(dual_sim_plug->init->dst,&msend, -1);
		printk(KERN_INFO "dual_sim_plug->init->dst = %d,msend.channel = %d,msend.type = %d,msend.flag = %d,msend.value = %d\n",dual_sim_plug->init->dst,msend.channel,msend.type,msend.flag,msend.value);

  }
  else
   {
   }

   return IRQ_HANDLED;
     
}

static irqreturn_t sim2_irq_cd (int irq, void *dev_id)
{
	struct dual_sim_plug_device *dual_sim_plug = (struct dual_sim_plug_device *)dev_id;

	struct smsg msend;
	int  flag = 0;
	//printk(KERN_INFO "Enter sim2_irq_cd\n");
	flag = gpio_get_value(dual_sim_plug->sim2_detect_gpio);
	//printk(KERN_INFO "Enter sim2_irq_cd,flag = %d\n",flag);
  if(flag == 0)	
	{
		flag_low_count_sim2++;
		flag_high_count_sim2 = 0;
	}
	else
	{
		flag_high_count_sim2++;
		flag_low_count_sim2 = 0;
	}

	if(flag_low_count_sim2 >= DELAY_COUNT)	
	{
		flag_low_count_sim2 = 0;
		irq_set_irq_type(irq,IRQF_TRIGGER_HIGH); 
		printk(KERN_INFO "Enter sim2_irq_cd,flag = low\n");
		smsg_set(&msend, dual_sim_plug->init->channel,SMSG_TYPE_CMD,SMSG_SIM_PLUG,VALUE_SIM2_PLUGIN);//plug in
		smsg_send(dual_sim_plug->init->dst, &msend, -1);
		printk(KERN_INFO "dual_sim_plug->init->dst = %d,msend.channel = %d,msend.type = %d,msend.flag = %d,msend.value = %d\n",dual_sim_plug->init->dst,msend.channel,msend.type,msend.flag,msend.value);

	}
	else if(flag_high_count_sim2 >= DELAY_COUNT)
	{
		flag_high_count_sim2 = 0;
		irq_set_irq_type(irq,IRQF_TRIGGER_LOW); 
		printk(KERN_INFO "Enter sim2_irq_cd,flag = high\n");
		smsg_set(&msend, dual_sim_plug->init->channel,SMSG_TYPE_CMD,SMSG_SIM_PLUG,VALUE_SIM2_PLUGOUT);//plug out
		smsg_send(dual_sim_plug->init->dst,&msend, -1);
		printk(KERN_INFO "dual_sim_plug->init->dst = %d,msend.channel = %d,msend.type = %d,msend.flag = %d,msend.value = %d\n",dual_sim_plug->init->dst,msend.channel,msend.type,msend.flag,msend.value);
		
	}
	else
	{
	}

   return IRQ_HANDLED;
}
static int dual_sim_plug_thread(void *data)
{
		struct dual_sim_plug_device *dual_sim_plug = (struct dual_sim_plug_device *)data;
		struct smsg mrecv;
		int rval;
		struct sched_param param = {.sched_priority = 91};

		sched_setscheduler(current,SCHED_RR,&param);
		printk(KERN_INFO "Enter dual_sim_plug_thread\n");

		while (!kthread_should_stop()) 
		{
			smsg_set(&mrecv, dual_sim_plug->init->channel,0,0,0);
			rval = smsg_recv(dual_sim_plug->init->dst,&mrecv,-1);
			printk(KERN_INFO "Enter dual_sim_plug_thread rval = %d,mrecv.channel = %d,mrecv.type =%d,mrecv.flag =%d,mrecv.value =%d\n",rval,mrecv.channel,mrecv.type,mrecv.flag,mrecv.value);
		if (rval == -EIO)
		{
			msleep(5);
			continue;
		}
		switch(mrecv.type) 
		{
			case SMSG_TYPE_DONE:
				if(mrecv.flag == SMSG_SIM_PLUG)
					{
               if(mrecv.value == VALUE_SIM1_PLUGOUT)//sim1 plug out
               	{
               	    printk(KERN_INFO "Sim1 plug out done\n");
               	}
					     else if(mrecv.value == VALUE_SIM1_PLUGIN)//sim1 plug in
					     	{
					     	     printk(KERN_INFO "Sim1 plug in done\n");
					     	}
					     else if(mrecv.value == VALUE_SIM2_PLUGOUT)//sim2 plug out
					     	{
					     	     printk(KERN_INFO "Sim2 plug out done\n");
					     	}
					     else if(mrecv.value == VALUE_SIM2_PLUGIN)//sim2 plug in
					     	{
					     	     printk(KERN_INFO "Sim2 plug in done\n");
					     	}
					     else
					 	    {

					 	    }
					}
				break;
			case SMSG_TYPE_OPEN:
                                printk(KERN_INFO "SMSG_TYPE_OPEN!!!\n");
                                smsg_open_ack(dual_sim_plug->init->dst, dual_sim_plug->init->channel);
                                break;
			 default:
				break;
	 	}
	 }

		return 0;

}

static inline void dual_sim_plug_destroy_pdata(struct dual_sim_plug_init_data **init)
{
#ifdef CONFIG_OF
	struct dual_sim_plug_init_data *pdata = *init;

	if (pdata) {
		kfree(pdata);
	}

	*init = NULL;
#else
	return;
#endif
}

static int dual_sim_plug_parse_dt(struct dual_sim_plug_init_data **init, struct device *dev)
{
#ifdef CONFIG_OF
	struct device_node *np = dev->of_node;
	struct dual_sim_plug_init_data *pdata = NULL;
	int ret;
	uint32_t data;

	pdata = kzalloc(sizeof(struct dual_sim_plug_init_data),GFP_KERNEL);
	if (!pdata) {
		printk(KERN_ERR "Failed to allocate pdata memory\n");
		return -ENOMEM;
	}

	ret = of_property_read_string(np,"sprd,name",(const char**)&pdata->name);
	if (ret) {
		goto error;
	}

	ret = of_property_read_u32(np,"sprd,dst",(uint32_t *)&data);
	if (ret) {
		goto error;
	}
	pdata->dst = (uint8_t)data;

	ret = of_property_read_u32(np,"sprd,channel",(uint32_t *)&data);
	if (ret) {
		goto error;
	}
	pdata->channel = (uint8_t)data;
	
	ret = of_property_read_u32(np,"sprd,sim1_gpio",(uint32_t *)&data);
	if (ret) {
		goto error;
	}
	pdata->sim1_gpio = (uint32_t)data;
	
	ret = of_property_read_u32(np,"sprd,sim2_gpio",(uint32_t *)&data);
	if (ret) {
		goto error;
	}
	pdata->sim2_gpio = (uint32_t)data;

	*init = pdata;
	return ret;
error:
	kfree(pdata);
	*init = NULL;
	return ret;
#else
	return -ENODEV;
#endif
}

static int dual_sim_plug_probe(struct platform_device *pdev)//add by xiangru.bi
{
	struct dual_sim_plug_init_data *init = pdev->dev.platform_data;
	struct dual_sim_plug_device *dual_sim_plug;
	int rval = 0;
	int ret1,ret2;
	int result;
	int ret;

	//struct smsg msend;


	printk(KERN_INFO "Dual_sim_plug_probe start\n");

	if (pdev->dev.of_node && !init) {
		rval = dual_sim_plug_parse_dt(&init, &pdev->dev);
		if (rval) {
			printk(KERN_ERR "Failed to parse spipe device tree, ret=%d\n", rval);
			return rval;
		}
	}

	printk(KERN_INFO "Succed to parse spipe device tree, ret=%d,init->name=%s,init->dst=%d,init->channel=%d,init->sim1_gpio=%d,init->sim2_gpio=%d\n", rval,init->name,init->dst,init->channel,init->sim1_gpio,init->sim2_gpio);

	dual_sim_plug = kzalloc(sizeof(struct dual_sim_plug_device),GFP_KERNEL);
	if (!dual_sim_plug) {
		printk(KERN_ERR "Failed to allocate dual_sim_plug_device\n");
		ret = -ENOMEM;
		goto probe_out;
	}

	printk(KERN_INFO "Succed to allocate dual_sim_plug_device\n");

	dual_sim_plug->sim1_detect_gpio = init->sim1_gpio;
	dual_sim_plug->sim2_detect_gpio = init->sim2_gpio;

	dual_sim_plug->init = init;

	ret1 = gpio_request(dual_sim_plug->sim1_detect_gpio,"sim1_detect");
	if (ret1)
	{
		printk(KERN_ERR "failed to get sim1 detect gpio\n");
		ret = ret1;
		goto probe_free_dual_sim_plug;
	}

	gpio_direction_input(dual_sim_plug->sim1_detect_gpio);

       ret1 = gpio_get_value(dual_sim_plug->sim1_detect_gpio);
        if(ret1 == 0)
    	{
    	      printk(KERN_INFO "Sim1 detect gpio = low.\n");
    	}
	else
	{
	      printk(KERN_INFO "Sim1 detect gpio = high.\n");
	}


	printk(KERN_INFO "Succed to get sim1 detect gpio\n");
	
	dual_sim_plug ->irq_cd1 = gpio_to_irq(dual_sim_plug->sim1_detect_gpio);

	printk(KERN_INFO "Succed to get sim1 irq dual_sim_plug ->irq_cd1 =%d\n",dual_sim_plug ->irq_cd1);
	 if (dual_sim_plug ->irq_cd1 >= 0) 
	{
      if(ret1 == 0)
      {
			if (request_irq(dual_sim_plug ->irq_cd1,sim1_irq_cd,IRQF_TRIGGER_HIGH,"sim1_detect_hander",dual_sim_plug)) 
			{
				printk(KERN_ERR "can't get sim1 detect irq.\n");
				ret = -ENOENT;
				goto  probe_free_sim1_gpio;
			} 
			printk(KERN_INFO "Get sim1 detect irq.\n");
      }
		else
		{
			if (request_irq(dual_sim_plug ->irq_cd1,sim1_irq_cd,IRQF_TRIGGER_LOW,"sim1_detect_hander",dual_sim_plug)) 
			{
				printk(KERN_ERR "can't get sim1 detect irq.\n");
				ret = -ENOENT;
				goto  probe_free_sim1_gpio;
			} 
			printk(KERN_INFO "Get sim1 detect irq.\n");
		}
	}
    else
	{
		printk(KERN_INFO "host detect has no irq available\n");
	}  
	   
	ret2 = gpio_request(dual_sim_plug->sim2_detect_gpio, "sim2_detect");
  if (ret2)
	{
		printk(KERN_ERR "failed to get sim2 detect gpio\n");
		ret = ret2;
		goto probe_free_sim1_irq;
		
	}

	gpio_direction_input(dual_sim_plug->sim2_detect_gpio);

	ret2 = gpio_get_value(dual_sim_plug->sim2_detect_gpio);
  if(ret2 == 0)
  {
    	  printk(KERN_INFO "Sim2 detect gpio = low.\n");
  }
	else
	{
	      printk(KERN_INFO "Sim2 detect gpio = high.\n");
	}


	printk(KERN_INFO "Succed to get sim2 detect gpio\n");
	
	dual_sim_plug ->irq_cd2 = gpio_to_irq(dual_sim_plug->sim2_detect_gpio);	
	printk(KERN_INFO "Succed to get sim2 irq dual_sim_plug ->irq_cd2 =%d\n",dual_sim_plug ->irq_cd2);
	if (dual_sim_plug ->irq_cd2 >= 0) 
	{
      if(ret2 == 0)
      {
			if (request_irq(dual_sim_plug ->irq_cd2,sim2_irq_cd,IRQF_TRIGGER_HIGH,"sim2_detect_hander", dual_sim_plug)) 
			{
				printk(KERN_ERR "can't get sim2 detect irq.\n");
				ret = -ENOENT;
				goto probe_free_sim2_gpio;
			} 
			printk(KERN_INFO "Get sim2 detect irq.\n");
      }
		 else
	 	{
	 		if (request_irq(dual_sim_plug ->irq_cd2,sim2_irq_cd,IRQF_TRIGGER_LOW,"sim2_detect_hander", dual_sim_plug)) 
			{
				printk(KERN_ERR "can't get sim2 detect irq.\n");
				ret = -ENOENT;
				goto probe_free_sim2_gpio;
			} 
			printk(KERN_INFO "Get sim2 detect irq.\n");
	 	}
	}
   else 
	{
		printk(KERN_INFO "host detect has no irq available\n");
	}  

     rval = smsg_ch_open(init->dst,init->channel,-1);

    if (rval != 0) {
		printk(KERN_ERR "Failed to open channel: %d\n",init->channel);
		ret = rval;
		goto probe_free_sim2_irq;
	}   
	  
	printk(KERN_INFO "Succed to open channel: %d\n",init->channel);

  dual_sim_plug->th1 = kthread_create(dual_sim_plug_thread,dual_sim_plug,"dual_sim_plug-%d-%d",init->dst,init->channel);

	if (IS_ERR(dual_sim_plug->th1))
	{
		printk(KERN_ERR "Failed to create kthread: dual_sim_plug-%d-%d\n",init->dst,init->channel);
		result = PTR_ERR(dual_sim_plug->th1);
		ret = result;
		goto probe_close_ch;
	} 

	printk(KERN_INFO "Succed to create kthread: dual_sim_plug-%d-%d\n",init->dst,init->channel); 

	wake_up_process(dual_sim_plug->th1);

	platform_set_drvdata(pdev, dual_sim_plug);

	return 0;

probe_close_ch:
	smsg_ch_close(init->dst,init->channel,-1);
probe_free_sim2_irq:	
	free_irq(dual_sim_plug ->irq_cd2,dual_sim_plug);
probe_free_sim2_gpio:
	gpio_free(dual_sim_plug->sim2_detect_gpio);
 probe_free_sim1_irq:	
	free_irq(dual_sim_plug ->irq_cd1,dual_sim_plug);
 probe_free_sim1_gpio:
       gpio_free(dual_sim_plug->sim1_detect_gpio);
probe_free_dual_sim_plug:
	kfree(dual_sim_plug);	
probe_out:
	dual_sim_plug_destroy_pdata(&init);
	return ret;  
}

static int  dual_sim_plug_remove(struct platform_device *pdev)
{

   printk(KERN_INFO "Dual_sim_plug_remove start\n");
	struct dual_sim_plug_device *dual_sim_plug = platform_get_drvdata(pdev);

	printk(KERN_INFO "dual_sim_plug->init->dst = %d,dual_sim_plug->init->channel = %d,dual_sim_plug->init->sim1_gpio=%d,dual_sim_plug->init->sim2_gpio=%d\n",dual_sim_plug->init->dst,dual_sim_plug->init->channel,dual_sim_plug->init->sim1_gpio,dual_sim_plug->init->sim2_gpio); 

  if (!IS_ERR_OR_NULL(dual_sim_plug->th1)) {
		kthread_stop(dual_sim_plug->th1);
	}

	smsg_ch_close(dual_sim_plug->init->dst,dual_sim_plug->init->channel,-1);

  free_irq(dual_sim_plug ->irq_cd2,dual_sim_plug);

	gpio_free(dual_sim_plug->sim2_detect_gpio);
	   
	free_irq(dual_sim_plug ->irq_cd1,dual_sim_plug);

	gpio_free(dual_sim_plug->sim1_detect_gpio);

	dual_sim_plug_destroy_pdata(&dual_sim_plug->init);

	kfree(dual_sim_plug);

	platform_set_drvdata(pdev,NULL);

	return 0;
}

static const struct of_device_id dual_sim_plug_match_table[] = {
	{.compatible = "sprd,dual_sim_plug", },
	{ },
};

static struct platform_driver dual_sim_plug_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "dual_sim_plug",
		.of_match_table = dual_sim_plug_match_table,
	},
	.probe = dual_sim_plug_probe,
	.remove = dual_sim_plug_remove,
};

static int __init dual_sim_plug_init(void)
{
       printk(KERN_INFO "Dual_sim_plug_init start\n");
	return platform_driver_register(&dual_sim_plug_driver);
}

static void __exit dual_sim_plug_exit(void)
{
  printk(KERN_INFO "Dual_sim_plug_exit start\n");
	platform_driver_unregister(&dual_sim_plug_driver);
}

module_init(dual_sim_plug_init);
module_exit(dual_sim_plug_exit);

MODULE_AUTHOR("Bi Xiangru");
MODULE_DESCRIPTION("SIPC/DUAL_SIM_PLUG driver");
MODULE_LICENSE("GPL");


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/reboot.h>
#include <linux/string.h>
#include <video/mmp_disp.h>
#include "../../../drivers/video/sprdfb/sprdfb_panel.h"

extern u8 get_hardware_version();
extern char lcd_name_from_uboot[50];
extern int lcd_find_success;
extern struct panel_spec lcd_detect_mipi_info;//add by liuwei

static struct kobject *bootinfo_kobj = NULL;
static u8 lcd_array[]="lcd not found!";

bool tp_probe_ok;//bit0
bool camera_front_probe_ok;//bit1
bool camera_back_probe_ok;//bit2
bool gsensor_probe_ok;//bit3
bool proximity_probe_ok;//bit4
bool charger_probe_ok;//bit5
bool pmu_probe_ok;//bit6
void itoa(int value, char *str)
{
    if (value < 0) 

    {
        str[0] = '-';
        value = 0-value;
    }
    int i,j;
    for(i=1; value > 0; i++,value/=10) 

        str[i] = value%10+'0'; 

    for(j=i-1,i=1; j-i>=1; j--,i++) 

    {
        str[i] = str[i]^str[j];
        str[j] = str[i]^str[j];
        str[i] = str[i]^str[j];
    }
    if(str[0] != '-') 

    {
        for(i=0; str[i+1]!='\0'; i++)
            str[i] = str[i+1];
        str[i] = '\0';
    }
}

static ssize_t hardware_version_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;
	u8 hw_ver;
	hw_ver=get_hardware_version();
	s += sprintf(s, "%c\n", hw_ver+'0');

	return (s - buf);
}

static ssize_t hardware_version_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

static struct kobj_attribute hardware_version_attr = {
	.attr = {
		.name = "hardware_version",
		.mode = 0644,
	},
	.show =&hardware_version_show,
	.store= &hardware_version_store,
};
static ssize_t lcd_info_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;

	if(lcd_find_success)
		s += sprintf(s, "%s\n",lcd_name_from_uboot);
	else
		s += sprintf(s, "%s\n",lcd_array);
	
	return (s - buf);
}

static ssize_t lcd_info_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}
static struct kobj_attribute lcd_info_attr = {
	.attr = {
		.name = "lcd_info",
		.mode = 0644,
	},
	.show =&lcd_info_show,
	.store= &lcd_info_store,
};

//end
static ssize_t lcd_driving_mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
       unsigned int val;
       int res=0;
	   
	 res = kstrtouint(buf, 10, &val);

	if(lcd_detect_mipi_info.ops->lcd_set_driving_mode)
		lcd_detect_mipi_info.ops->lcd_set_driving_mode(&lcd_detect_mipi_info,val);
	else
		printk(KERN_ERR "[kernel]:lcd_set_driving_mode not found!.\n");
	return n;
}

static struct kobj_attribute lcd_driving_mode_set_attr = {
	.attr = {
		.name = "lcd_driving_mode_set_info",
		.mode = 0644,
	},
	.store = &lcd_driving_mode_store,
};

/*wdm+*/
static DEFINE_MUTEX(call_lock);
unsigned int call_state;
static ssize_t call_state_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	int res=0;

	mutex_lock(&call_lock);
        res = kstrtouint(buf, 10, &call_state);
        mutex_unlock(&call_lock);
       	
        return n;
}

static ssize_t call_state_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
       
	return sprintf(buf, "%u\n", call_state);
}

static struct kobj_attribute call_state_set_attr = {
        .attr = {
                .name = "call_state_info",
                .mode = 0644,
        },
	.show = &call_state_show,
        .store = &call_state_store,
};

unsigned int get_call_state(void)
{
	unsigned int temp;

	mutex_lock(&call_lock);
        temp = call_state;
	mutex_unlock(&call_lock);

	return temp;
}
EXPORT_SYMBOL_GPL(get_call_state);
/*wdm++*/

static ssize_t i2c_devices_info_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;
	u8 string[5]={'\0'};
	int tmp=0;
	tmp|= (tp_probe_ok<<0);
	tmp|= (camera_front_probe_ok<<1);
	tmp|= (camera_back_probe_ok<<2);
	tmp|= (gsensor_probe_ok<<3);
	tmp|= (proximity_probe_ok<<4);
	tmp|= (charger_probe_ok<<5);
	tmp|= (pmu_probe_ok<<6);

	itoa((int)tmp,string);
	s += sprintf(s, "%s\n",string);
	
	return (s - buf);
}
static struct kobj_attribute i2c_devices_info_attr = {
	.attr = {
		.name = "i2c_devices_probe_info",
		.mode = 0444,
	},
	.show =&i2c_devices_info_show,
};
static struct attribute * g[] = {
	&hardware_version_attr.attr,//+add by liuwei
	&lcd_info_attr.attr,//+add by liuwei
	&lcd_driving_mode_set_attr.attr,//+add by liuwei
	&i2c_devices_info_attr.attr,//+add by liuwei
	&call_state_set_attr.attr,//wdm
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};



static int __init bootinfo_init(void)
{
	int ret = -ENOMEM;
	
	printk("%s,line=%d\n",__func__,__LINE__);  

	bootinfo_kobj = kobject_create_and_add("bootinfo", NULL);

	if (bootinfo_kobj == NULL) {
		printk("bootinfo_init: kobject_create_and_add failed\n");
		goto fail;
	}

	ret = sysfs_create_group(bootinfo_kobj, &attr_group);
	if (ret) {
		printk("bootinfo_init: sysfs_create_group failed\n");
		goto sys_fail;
	}
	
	return ret;
sys_fail:
	kobject_del(bootinfo_kobj);
fail:
	return ret;

}

static void __exit bootinfo_exit(void)
{

	if (bootinfo_kobj) {
		sysfs_remove_group(bootinfo_kobj, &attr_group);
		kobject_del(bootinfo_kobj);
	}
}

arch_initcall(bootinfo_init);
module_exit(bootinfo_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Boot information collector");

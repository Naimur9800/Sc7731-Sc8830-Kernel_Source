#ifndef __LINUX_TOUCHPANEL_H__
#define __LINUX_TOUCHPANEL_H__

struct touchpanel_virtual_key_data {

    int     key_code;
    int     logic_x;
    int     logic_y;
    int     x_correction;
    int     y_correction;
};
struct touchpanel_wakeup_event_data {

    int     wakeup_event;
    int     key_code;
};
#define TP_FREQ_HOPPING_MODE_AUTO	0
#define TP_FREQ_HOPPING_MODE_CHARGER	1
#define TP_FREQ_HOPPING_MODE_ON	2
#define TP_WAKEUP_EVENT_NONE				(0)
#define TP_WAKEUP_EVENT_LIFT				(1<<0)
#define TP_WAKEUP_EVENT_RIGHT				(1<<1)
#define TP_WAKEUP_EVENT_UP				(1<<2)
#define TP_WAKEUP_EVENT_DOWN				(1<<3)
#define TP_WAKEUP_EVENT_DOUBLE_CLICK		(1<<4)

struct touchpanel_panel_parm {
	int points;
	int x_max_res;
	int y_max_res;
	struct touchpanel_virtual_key_data  *virtual_key; 
	int virtual_key_num;
	struct touchpanel_wakeup_event_data  *wakeup_event; 
	int wakeup_event_num;
	int vendor_id;
	int major_version;
       unsigned char product_name[16];
	unsigned char * panal_cfg;
	int panal_cfg_size;
	unsigned char * panel_fw;
	int panel_fw_size;
	unsigned char vendor_name[50];
	int panel_ic_id;
	int freq_hopping_mode;//frequency hopping
};

/* The platform data for the Focaltech ft5x0x touchscreen driver */
struct touchpanel_platform_data
{
	int (*reset)(struct device *);
	int (*power_i2c)(struct device *, int);
	int (*power_ic)(struct device *, int);
	void (*pin_config)( void );
	unsigned long irqflags;
	unsigned int irq_gpio;
	unsigned int reset_gpio;
	struct touchpanel_panel_parm * panel_parm;
	int panel_parm_num;
	int prox_enable;
	int slot_enable;
	bool power_on_when_sleep;
	
	bool x_flip;
	bool y_flip;
       bool x_y_exchange;
	bool ignore_product_name;
	const char *vdd_name;
};
#endif

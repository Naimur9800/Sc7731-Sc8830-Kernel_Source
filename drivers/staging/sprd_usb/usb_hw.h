#ifndef _USB_HW_H
#define _USB_HW_H
#include "dwc_otg_pcd.h"
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>

enum vbus_irq_type {
	        VBUS_PLUG_IN,
		        VBUS_PLUG_OUT
};

enum id_irq_type {
	        OTG_CABLE_PLUG_IN,
		        OTG_CABLE_PLUG_OUT
};

struct usb_priv_data {

	struct regmap			*ahb_syscon;
	struct usb_phy			*hs_phy;
	struct clk			*ref_clk;
	int				gpio_vbus;
	int				gpio_id;
	int				module_en;
	enum usb_dr_mode mode;
	uint32_t		g_intsts;
	uint32_t		g_intmsk;
	const char		*cable_detect;
};

struct gadget_wrapper {
	dwc_otg_pcd_t *pcd;

	struct usb_gadget gadget;
	struct usb_gadget_driver *driver;

	struct usb_ep ep0;
	struct usb_ep in_ep[16];
	struct usb_ep out_ep[16];
	/*
	 * this timer is used for checking cable type, usb or ac adapter.
	 */
	struct workqueue_struct *cable2pc_wq;
	struct workqueue_struct *detect_wq;
	struct work_struct detect_work;
	struct delayed_work cable2pc;

	int udc_startup;
	int enabled;
	bool vbus_active;
	enum usb_dr_mode		dr_mode;
	spinlock_t lock;
	dwc_mutex_t *udc_mutex;
};
#define HUB_PACK_SIZE 188
#if defined(CONFIG_ARCH_SCX20)
#define AP_TOP_USB_PHY_RST SPRD_PIN_BASE
extern unsigned int USB_GUSBCFG_REG;
#endif

extern struct gadget_wrapper *gadget_wrapper;
extern struct usb_priv_data *usb_priv;

int usb_alloc_vbus_irq(int gpio);
void usb_free_vbus_irq(int irq, int gpio);
int usb_get_vbus_irq(void);
int usb_get_vbus_state(void);
void charge_pump_set(int gpio, int state);
void udc_enable(enum usb_dr_mode mode);
enum usb_dr_mode usb_get_udc_mode(void);
void udc_disable(void);
int usb_alloc_id_irq(int gpio);
void usb_free_id_irq(int irq, int gpio);
int usb_get_id_irq(void);
int usb_get_id_state(void);
bool in_calibration(void);
bool in_charger(void);
bool in_autotest(void);
void usb_phy_ahb_rst(void);
void usb_set_vbus_irq_type(int irq, int irq_type);
void usb_set_id_irq_type(int irq, int irq_type);
enum usb_dr_mode usb_get_phy_mode(void);
void usb_phy_mode(enum usb_dr_mode mode);
int usb_get_module_enable(void);
int usb_get_module_en(void);

#endif

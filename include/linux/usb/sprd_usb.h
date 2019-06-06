#ifndef _SPRD_USB_H_
#define _SPRD_USB_H_

#include <linux/err.h>

#define CABLE_STATUS_ATTACHED	1
#define CABLE_STATUS_DETACHED	0

#define USB_CABLE_DEVICE	BIT(1)
#define USB_CABLE_HOST		BIT(2)
#define USB_CABLE_POWER		BIT(3)
#define USB_CABLE_AUDIO		BIT(4)
#define USB_CABLE_DEBUG		BIT(5)

#define CABLE_STATUS_UNKNOWN		0
#define CABLE_STATUS_DEV_CONN		(USB_CABLE_DEVICE | CABLE_STATUS_ATTACHED)
#define CABLE_STATUS_DEV_DISCONN	(USB_CABLE_DEVICE | CABLE_STATUS_DETACHED)
#define CABLE_STATUS_HOST_CONN		(USB_CABLE_HOST | CABLE_STATUS_ATTACHED)
#define CABLE_STATUS_HOST_DISCONN	(USB_CABLE_HOST | CABLE_STATUS_DETACHED)

/*
 * TODO: If these features would be supported in the future, unmask them.
 * #define CABLE_STATUS_POWER_CONN	(USB_CABLE_POWER | CABLE_STATUS_ATTACHED)
 * #define CABLE_STATUS_POWER_DISCONN	(USB_CABLE_POWER | CABLE_STATUS_DETACHED)
 * #define CABLE_STATUS_AUDIO_CONN	(USB_CABLE_AUDIO | CABLE_STATUS_ATTACHED)
 * #define CABLE_STATUS_AUDIO_DISCONN	(USB_CABLE_AUDIO | CABLE_STATUS_DETACHED)
 * #define CABLE_STATUS_DEBUG_CONN	(USB_CABLE_DEBUG | CABLE_STATUS_ATTACHED)
 * #define CABLE_STATUS_DEBUG_DISCONN	(USB_CABLE_DEBUG | CABLE_STATUS_DETACHED)
 */
#if IS_ENABLED(CONFIG_USB_SPRD_TYPEC)
int register_ext_notifier(struct notifier_block *nb);
int unregister_ext_notifier(struct notifier_block *nb);
void typec_disable_ldo(void);
#else
static inline int register_ext_notifier(struct notifier_block *nb)
{ return 0; }
static inline int unregister_ext_notifier(struct notifier_block *nb)
{ return 0; }
static inline void typec_disable_ldo(void) {}
#endif

#ifndef CONFIG_USB_SPRD_DWC
#if IS_ENABLED(CONFIG_BATTERY_SPRD)
void sprd_extic_otg_power(int enable);
#else
static inline void sprd_extic_otg_power(int enable) {}
#endif
#endif

enum usb_cable_status {
	USB_CABLE_PLUG_IN,
	USB_CABLE_PLUG_OUT,
};

#endif /* _SPRD_USB_H_ */

#ifndef __LINUX_USB_CHARGER_H__
#define __LINUX_USB_CHARGER_H__

#include <uapi/linux/usb/ch9.h>
#include <uapi/linux/usb/charger.h>

/* Current limitation by charger type */
struct usb_charger_cur_limit {
	unsigned int sdp_cur_limit;
	unsigned int dcp_cur_limit;
	unsigned int cdp_cur_limit;
	unsigned int aca_cur_limit;
};

struct usb_charger_nb {
	struct notifier_block	nb;
	struct usb_charger	*uchger;
};

struct usb_charger {
	struct device		dev;
	struct raw_notifier_head	uchger_nh;
	/* protect the notifier head and charger */
	struct mutex		lock;
	int			id;
	enum usb_charger_type	type;
	enum usb_charger_state	state;

	/* for supporting extcon usb gpio */
	struct extcon_dev	*extcon_dev;
	struct usb_charger_nb	extcon_nb;

	/* for supporting usb gadget */
	struct usb_gadget	*gadget;
	enum usb_device_state	old_gadget_state;

	/* for supporting power supply */
	struct power_supply	*psy;

	/* user can get charger type by implementing this callback */
	enum usb_charger_type	(*get_charger_type)(struct usb_charger *);
	/*
	 * charger detection method can be implemented if you need to
	 * manually detect the charger type.
	 */
	enum usb_charger_type	(*charger_detect)(struct usb_charger *);

	/* current limitation */
	struct usb_charger_cur_limit	cur_limit;
	/* to check if it is needed to change the SDP charger default current */
	unsigned int		sdp_default_cur_change;
};

#ifdef CONFIG_USB_CHARGER
extern struct usb_charger *usb_charger_find_by_name(const char *name);

extern struct usb_charger *usb_charger_get(struct usb_charger *uchger);
extern void usb_charger_put(struct usb_charger *uchger);

extern int usb_charger_register_notify(struct usb_charger *uchger,
				       struct notifier_block *nb);
extern int usb_charger_unregister_notify(struct usb_charger *uchger,
					 struct notifier_block *nb);

extern int usb_charger_set_cur_limit(struct usb_charger *uchger,
				struct usb_charger_cur_limit *cur_limit_set);
extern int usb_charger_set_cur_limit_by_type(struct usb_charger *uchger,
					     enum usb_charger_type type,
					     unsigned int cur_limit);
extern int usb_charger_set_current_by_type_lock(struct usb_charger *uchger,
						enum usb_charger_type type,
						unsigned int cur_limit);
extern unsigned int usb_charger_get_current(struct usb_charger *uchger);

extern int usb_charger_plug_by_gadget(struct usb_gadget *gadget,
				      unsigned long state);
extern enum usb_charger_type usb_charger_get_type(struct usb_charger *uchger);
extern int usb_charger_detect_type(struct usb_charger *uchger);
extern enum usb_charger_state usb_charger_get_state(struct usb_charger *uchger);

extern void devm_usb_charger_unregister(struct device *dev,
					struct usb_charger *uchger);
extern int devm_usb_charger_register(struct device *dev,
				     struct usb_charger *uchger);

extern int usb_charger_init(struct usb_gadget *ugadget);
extern int usb_charger_exit(struct usb_gadget *ugadget);
#else
static inline struct usb_charger *usb_charger_find_by_name(const char *name)
{
	return ERR_PTR(-ENODEV);
}

static inline struct usb_charger *usb_charger_get(struct usb_charger *uchger)
{
	return NULL;
}

static inline void usb_charger_put(struct usb_charger *uchger)
{
}

static inline int
usb_charger_register_notify(struct usb_charger *uchger,
			    struct notifier_block *nb)
{
	return 0;
}

static inline int
usb_charger_unregister_notify(struct usb_charger *uchger,
			      struct notifier_block *nb)
{
	return 0;
}

static inline int
usb_charger_set_cur_limit(struct usb_charger *uchger,
			  struct usb_charger_cur_limit *cur_limit_set)
{
	return 0;
}

static inline int
usb_charger_set_cur_limit_by_type(struct usb_charger *uchger,
				  enum usb_charger_type type,
				  unsigned int cur_limit)
{
	return 0;
}

static inline int
usb_charger_set_current_by_type_lock(struct usb_charger *uchger,
				     enum usb_charger_type type,
				     unsigned int cur_limit)
{
	return 0;
}

static inline unsigned int
usb_charger_get_current(struct usb_charger *uchger)
{
	return 0;
}

static inline enum usb_charger_type
usb_charger_get_type(struct usb_charger *uchger)
{
	return UNKNOWN_TYPE;
}

static inline int usb_charger_detect_type(struct usb_charger *uchger)
{
	return 0;
}

static inline enum usb_charger_state
usb_charger_get_state(struct usb_charger *uchger)
{
	return USB_CHARGER_REMOVE;
}

static inline int
usb_charger_plug_by_gadget(struct usb_gadget *gadget, unsigned long state)
{
	return 0;
}

static inline void devm_usb_charger_unregister(struct device *dev,
					       struct usb_charger *uchger)
{
}

static inline int devm_usb_charger_register(struct device *dev,
					    struct usb_charger *uchger)
{
	return 0;
}

static inline int usb_charger_init(struct usb_gadget *ugadget)
{
	return 0;
}

static inline int usb_charger_exit(struct usb_gadget *ugadget)
{
	return 0;
}
#endif

#endif /* __LINUX_USB_CHARGER_H__ */

/* sprd_mailbox.h */

#ifndef SPRD_MAILBOX_H
#define SPRD_MAILBOX_H

#include <linux/interrupt.h>

#ifdef CONFIG_SPRD_MAILBOX_FIFO
typedef irqreturn_t (*MBOX_FUNCALL)(void *ptr, void *private);
int mbox_register_irq_handle(u8 target_id,
			     MBOX_FUNCALL irq_handler,
			     void *priv_data);
#else
int mbox_register_irq_handle(u8 target_id,
			     irq_handler_t irq_handler,
			     void *priv_data);
#endif

int mbox_unregister_irq_handle(u8 target_id);
int mbox_raw_sent(u8 target_id, u64 msg);
void mbox_just_sent(u8 core_id, u64 msg);

#ifdef CONFIG_SPRD_MAILBOX_FIFO
u32 mbox_core_fifo_full(int core_id);
#endif

#ifdef CONFIG_SPRD_MAILBOX_SENSOR
#define RECV_MBOX_SENSOR_ID  (5)
#endif

#endif /* MAILBOX_H */

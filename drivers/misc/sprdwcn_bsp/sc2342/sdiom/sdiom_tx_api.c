
#include <linux/sdiom_tx_api.h>

#include "sdiom_depend.h"
#include "sdiom_tx_core.h"
#include "sdiom_tx_fifo.h"
#include "sdiom_tx_sendbuf.h"
#include "sdiom_type.h"

int sdiom_pt_write(void *buf, unsigned int len, unsigned int type,
			    unsigned int subtype)
{
	struct sdiom_tx_q_t *tx_q = NULL;
	struct sdiom_tx_msg_t tx_msg = { 0 };

	sdiom_tx_lock();
	tx_q = sdiom_tx_q_getptr(type, subtype);

	if (tx_q == NULL) {
		sdiom_tx_unlock();
		return ERROR;
	}

	tx_msg.buf = (unsigned char *)buf;
	tx_msg.len = len;
	tx_msg.skb_buf = NULL;
	tx_msg.type = type;
	tx_msg.subtype = subtype;

	if (sdiom_tx_msg_enq(tx_q, &tx_msg) != OK) {
		sdiom_tx_unlock();
		return ERROR;
	}

	sdiom_tx_fifo_cnt_inc();

	sdiom_tx_core_up();
	sdiom_tx_unlock();

	return OK;
}
EXPORT_SYMBOL_GPL(sdiom_pt_write);

int sdiom_pt_write_skb(void *buf, void *skb_buf, unsigned int len,
				unsigned int type, unsigned int subtype)
{
	struct sdiom_tx_q_t *tx_q = NULL;
	struct sdiom_tx_msg_t tx_msg = { 0 };

	sdiom_tx_lock();
	tx_q = sdiom_tx_q_getptr(type, subtype);

	if (tx_q == NULL) {
		sdiom_tx_unlock();
		return ERROR;
	}

	tx_msg.buf = buf;
	tx_msg.len = len;
	tx_msg.skb_buf = skb_buf;
	tx_msg.type = type;
	tx_msg.subtype = subtype;

	if (sdiom_tx_msg_enq(tx_q, &tx_msg) != OK) {
		sdiom_tx_unlock();
		return ERROR;
	}

	sdiom_tx_fifo_cnt_inc();

	sdiom_tx_core_up();
	sdiom_tx_unlock();

	return OK;
}
EXPORT_SYMBOL_GPL(sdiom_pt_write_skb);

int sdiom_register_pt_tx_release(unsigned int type,
					  unsigned int subtype, void *func)
{
	sdiom_tx_callback_set(type, subtype, func);

	return OK;
}
EXPORT_SYMBOL_GPL(sdiom_register_pt_tx_release);

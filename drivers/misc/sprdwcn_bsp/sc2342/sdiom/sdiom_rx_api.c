#include <linux/list.h>
#include <linux/sdiom_rx_api.h>

#include "sdiom_depend.h"
#include "sdiom_rx_recvbuf.h"
#include "sdiom_type.h"

int sdiom_register_pt_rx_process(unsigned int type,
					  unsigned int subtype, void *func)
{
	sdiom_rx_cb_lock();
	sdiom_rx_callback_set(type, subtype, func);
	sdiom_rx_cb_unlock();

	return 0;
}
EXPORT_SYMBOL_GPL(sdiom_register_pt_rx_process);

int sdiom_pt_read_release(unsigned int fifo_id)
{
	sdiom_rx_packet_release(fifo_id);

	return 0;
}
EXPORT_SYMBOL_GPL(sdiom_pt_read_release);

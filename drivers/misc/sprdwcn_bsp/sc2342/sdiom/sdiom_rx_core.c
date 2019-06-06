#include <linux/list.h>

#include "sdiom_depend.h"
#include "sdiom_rx_core.h"
#include "sdiom_rx_recvbuf.h"
#include "sdiom_rx_parser.h"
#include "sdiom_type.h"

static unsigned int s_tx_core_cnt;
static unsigned long long s_tx_total_packet;

void sdiom_rx_core_task(void)
{
	struct sdiom_rx_recvbuf_t *rbuf = NULL;
	unsigned int packet_cnt = 0;
	struct sched_param param;

	param.sched_priority = 0;
	sched_setscheduler(current, SCHED_FIFO, &param);

	while (1) {
		/* Wait the semaphore */
		sdiom_rx_core_down();

		/* process recv buf */
		rbuf = sdiom_rx_recvbuf_rd_getptr();

		packet_cnt = sdiom_rx_parser_total_packet(rbuf);

		s_tx_total_packet += packet_cnt;

		sdiom_print("rx core down [%d] buf_id[%d] total[%d]\n",
			    s_tx_core_cnt, rbuf->index,
			    atomic_read(&rbuf->total_packet));

		if (packet_cnt != 0)
			sdiom_rx_parser(rbuf);

		sdiom_rx_recvbuf_rd_switch();

		s_tx_core_cnt++;
	}
}

unsigned long long sdiom_get_rx_total_cnt(void)
{
	return s_tx_total_packet;
}
EXPORT_SYMBOL_GPL(sdiom_get_rx_total_cnt);

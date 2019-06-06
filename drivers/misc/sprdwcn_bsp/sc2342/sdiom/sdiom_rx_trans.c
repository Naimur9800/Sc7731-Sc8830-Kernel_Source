#include "sdiom_depend.h"
#include "sdiom_rx_recvbuf.h"
#include "sdiom_rx_trans.h"
#include "sdiom_sleep.h"
#include "sdiom_type.h"

void sdiom_rx_trans_task(void)
{
	static unsigned int s_rx_trans_cnt;

	struct sched_param param;

	param.sched_priority = 90;
	sched_setscheduler(current, SCHED_FIFO, &param);

	while (1) {
		/* Wait the semaphore */
		sdiom_rx_trans_down();

		sdiom_resume_wait();

		sdiom_cp_rx_wakeup();

		sdiom_print("rx trans down [%d]\n", s_rx_trans_cnt);
		s_rx_trans_cnt++;

		sdiom_rx_recvbuf_fill();
	}
}

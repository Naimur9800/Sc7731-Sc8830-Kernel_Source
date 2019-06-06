#include "sdiom.h"
#include "sdiom_depend.h"
#include "sdiom_sleep.h"
#include "sdiom_tx_core.h"
#include "sdiom_tx_fifo.h"
#include "sdiom_tx_sendbuf.h"
#include "sdiom_tx_trans.h"
#include "sdiom_type.h"

#define TX_RETRY_MAX 3

static int sdio_send(void *buf, unsigned int len)
{
	int ret = 0;

	sdiom_print
	    ("cmd53 sending addr[%p] len_ori[%d] len_align[%d]\n",
	     buf, len, SDIOM_ALIGN_512BYTE(len));

#ifdef TX_PRINT_BUF_SUPPORT
	sdiom_print_buf((unsigned char *)buf, len, __func__);
#endif

	ret = sdiom_sdio_pt_write(buf, SDIOM_ALIGN_512BYTE(len));

	if (ret == 0) {
#ifdef TX_STATUS_SUPPORT
		sdiom_sdio_tx_status();
#endif
		return ret;
	} else {
		return ret;
	}

	return OK;
}

void sdiom_tx_trans_task(void)
{

	static unsigned int s_tx_trans_cnt;
	static unsigned int s_tx_trans_retry_cnt;

	struct sdiom_tx_sendbuf_t *sbuf = NULL;

	unsigned int tx_pac_cnt = 0;

	struct sched_param param;

	param.sched_priority = 0;
	sched_setscheduler(current, SCHED_FIFO, &param);

	while (1) {
		/* Wait the semaphore */
		sdiom_tx_trans_down();

		sdiom_lock_tx_trans_wakelock();

		sdiom_resume_wait();

		/* wakeup cp */
		sdiom_cp_tx_wakeup();

		sdiom_print("tx trans down [%d] retry[%d]\n", s_tx_trans_cnt,
			    s_tx_trans_retry_cnt);
		s_tx_trans_cnt++;

		sbuf = sdiom_tx_sendbuf_rd_getptr();

#ifdef TX_PRINT_PACK_SUPPORT
		sdiom_tx_print_pack(sbuf);
#endif
		if (s_tx_trans_retry_cnt == 0) {
			/* normal process */
			if (sdio_send(sbuf->buf, sbuf->used_len) == OK) {
				/* set the sbuf idle */
				atomic_set(&sbuf->busy, 0);

				sdiom_tx_sendbuf_rd_switch();
			} else {
				sdiom_tx_sendbuf_rd_init_retrybuf();
				s_tx_trans_retry_cnt++;

				/* get package num already send */
				tx_pac_cnt = sdiom_get_trans_pac_num();

				/* get the buf ptr and length right now */
				sdiom_tx_sendbuf_rd_calc_retrybuf
				    (tx_pac_cnt);

				/* delay 5ms */
				usleep_range(4000, 6000);
				sdiom_tx_trans_up();
			}
		} else {
			/* retry process */

			/* resend OK */
			if (sdio_send(sbuf->retry_buf, sbuf->retry_len) == OK) {

				/* set the sbuf idle */
				s_tx_trans_retry_cnt = 0;
				atomic_set(&sbuf->busy, 0);
				sdiom_tx_sendbuf_rd_switch();

			} else {
				/* resend timeout,try again */
				s_tx_trans_retry_cnt++;

				if (s_tx_trans_retry_cnt < TX_RETRY_MAX) {

					/* get package num already send */
					tx_pac_cnt = sdiom_get_trans_pac_num();

					/* get buf ptr and length right now */
					sdiom_tx_sendbuf_rd_calc_retrybuf
					    (tx_pac_cnt);

					/* delay 5ms */
					usleep_range(4000, 6000);
					sdiom_tx_trans_up();
				} else {
					/* exceed max retry, discard buf */
					sdiom_info("tx retry max!\n");

					sdiom_tx_print_pack(sbuf);

					/* set the sbuf idle */
					s_tx_trans_retry_cnt = 0;
					atomic_set(&sbuf->busy, 0);
					sdiom_tx_sendbuf_rd_switch();
				}
			}
		}

		/* tx data send done allow cp sleep */
		if ((sdiom_is_tx_fifo_empty() == OK)
				&& (s_tx_trans_retry_cnt == 0))
			sdiom_cp_tx_sleep();

		sdiom_unlock_tx_trans_wakelock();
	}
}

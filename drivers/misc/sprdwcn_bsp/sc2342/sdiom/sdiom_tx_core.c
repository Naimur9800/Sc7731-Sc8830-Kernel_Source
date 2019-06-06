
#include "sdiom_depend.h"
#include "sdiom_sleep.h"
#include "sdiom_tx_core.h"
#include "sdiom_tx_fifo.h"
#include "sdiom_tx_sendbuf.h"
#include "sdiom_type.h"

struct sdiom_tx_q_t *ptq_last;

void sdiom_tx_cb_exec(struct sdiom_tx_msg_t *tx_msg)
{
	SDIOM_PT_TX_RELEASE_CALLBACK callback;

	callback = sdiom_tx_callback_get(tx_msg->type, tx_msg->subtype);

	if (callback != NULL) {
		if (tx_msg->skb_buf == NULL)
			callback(tx_msg->buf);
		else
			callback(tx_msg->skb_buf);
	} else {
		sdiom_err
		    ("caution callback[%d][%d] not registed\n",
		     tx_msg->type, tx_msg->subtype);
	}
}

void sdiom_tx_core_task(void)
{
	struct sdiom_tx_q_t *ptq = NULL;
	struct sdiom_tx_msg_t *tx_msg = NULL;

	while (1) {
		/* wait semphore */
		sdiom_tx_core_down();

		sdiom_lock_tx_core_wakelock();

		/* find the next q with data */
		ptq = sdiom_tx_q_find_data();

		if (ptq == NULL)
			sdiom_err("tx_q null\n");

		if (ptq_last != ptq) {
			if (ptq_last != NULL) {
				sdiom_print("tx q[%d][%d]->q[%d][%d]\n",
					ptq_last->type, ptq_last->subtype,
					ptq->type, ptq->subtype);
			}

			/* send the data */
			if (sdiom_tx_sendbuf_usedlen_now() != 0) {
				sdiom_tx_sendbuf_wt_switch();
				sdiom_tx_trans_up();
			}
		}

		ptq_last = ptq;

		if (ptq != NULL) {
			tx_msg = sdiom_tx_msg_get(ptq);

			sdiom_print("tx_msg get type[%d] subtype[%d] len[%d]\n",
				tx_msg->type, tx_msg->subtype, tx_msg->len);

			/* fill the sendbuf and deq the msg */

			/* delay for test */
			/* os_sleep(2); */

			sdiom_tx_sendbuf_fill(tx_msg);

			/*
			* After copy data to send_buf,
			* we call callback to free original buffer
			*/

			sdiom_tx_cb_exec(tx_msg);

			sdiom_tx_msg_deq(ptq);

			sdiom_tx_fifo_cnt_dec();
		}

		if (sdiom_is_tx_fifo_empty() == OK) {
			/* data is empty */
			if (sdiom_tx_sendbuf_usedlen_now() != 0) {
				sdiom_tx_sendbuf_wt_switch();
				sdiom_tx_trans_up();
			}
		}

		sdiom_unlock_tx_core_wakelock();
	}
}


#include "sdiom_depend.h"
#include "sdiom_tx_fifo.h"
#include "sdiom_tx_sendbuf.h"
#include "sdiom_tx_packer.h"
#include "sdiom_type.h"

#define SDIOM_TX_SENDBUF_NUM 40
#define SDIOM_TX_SENDBUF_LEN (10 << 10)


static struct sdiom_tx_sendbuf_t
s_tx_sendbuf_array[SDIOM_TX_SENDBUF_NUM] = { { {0} } };

static struct list_head *s_tx_sendbuf_wt_node;
static struct list_head *s_tx_sendbuf_rd_node;

int sdiom_tx_sendbuf_alloc(struct sdiom_tx_sendbuf_t *sendbuf, unsigned int len)
{
	sdiom_memset((char *)sendbuf, 0, sizeof(struct sdiom_tx_sendbuf_t));
	sendbuf->total_len = len;
	sendbuf->buf = sdiom_malloc(len);
	if (sendbuf->buf == NULL)
		return ERROR;

	return OK;
}

int sdiom_tx_sendbuf_init(void)
{
	unsigned int i = 0;
	static unsigned int first;

	struct list_head *first_pos = NULL;
	struct list_head *pos = NULL;

	struct sdiom_tx_sendbuf_t *ptsbuf = NULL;

	for (i = 0; i < SDIOM_TX_SENDBUF_NUM; i++) {
		sdiom_tx_sendbuf_alloc(&s_tx_sendbuf_array[i],
				       SDIOM_TX_SENDBUF_LEN);
		s_tx_sendbuf_array[i].index = i;

		if (first == 0) {
			INIT_LIST_HEAD(&s_tx_sendbuf_array[i].node);
			s_tx_sendbuf_wt_node =
			    (struct list_head *)(&s_tx_sendbuf_array[i].node.
						 next);
			first_pos = s_tx_sendbuf_wt_node;
			first = !0;
		} else {
			list_add(&s_tx_sendbuf_array[i].node,
				 s_tx_sendbuf_wt_node);
			s_tx_sendbuf_wt_node = &s_tx_sendbuf_array[i].node;
		}
	}

	s_tx_sendbuf_wt_node = first_pos;
	s_tx_sendbuf_rd_node = first_pos;

	sdiom_print("sdiom_tx_sendbuf_init after init\n");

	ptsbuf =
	    (struct sdiom_tx_sendbuf_t *)list_entry(first_pos,
						    struct sdiom_tx_sendbuf_t,
						    node);

	sdiom_print("first index[%d]\n", ptsbuf->index);

	for (pos = first_pos;;) {
		ptsbuf = (struct sdiom_tx_sendbuf_t *)list_entry(pos, struct
							 sdiom_tx_sendbuf_t,
								 node);

		sdiom_print("cur index[%d]\n", ptsbuf->index);

		pos = pos->next;
		if (pos == first_pos)
			break;
	}

	return 0;
}

int sdiom_tx_sendbuf_wt_switch(void)
{

	struct sdiom_tx_sendbuf_t *cur_tx_sbuf;
	unsigned int sbuf_busy_cnt = 0;

	/* fill eof before sending */
	cur_tx_sbuf =
	    list_entry(s_tx_sendbuf_wt_node, struct sdiom_tx_sendbuf_t, node);

	sdiom_tx_eof(cur_tx_sbuf);

	/* after eof the sbuf is busy */
	atomic_set(&cur_tx_sbuf->busy, 1);

	s_tx_sendbuf_wt_node = s_tx_sendbuf_wt_node->next;

	cur_tx_sbuf =
	    list_entry(s_tx_sendbuf_wt_node, struct sdiom_tx_sendbuf_t, node);

SDIOM_TX_SENDBUF_WT_SWITCH_RETRY:

	/* judge if the buffer is busy */

	if (atomic_read(&cur_tx_sbuf->busy) == 0) {
		/* the buffer is idle */

		/* clear used_len */
		cur_tx_sbuf->used_len = 0;	/* move to here for same task */

		sdiom_print("sdiom_tx_sendbuf_wt_switch switch to sbuf[%d]\n",
			    cur_tx_sbuf->index);
	} else {
		/* the buffer is busy */

		sbuf_busy_cnt++;

		sdiom_print
		    ("sdiom_tx_sendbuf_wt_switch sbuf[%d] is busy.[%d]\n",
		     cur_tx_sbuf->index, sbuf_busy_cnt);

		msleep(20);
		goto SDIOM_TX_SENDBUF_WT_SWITCH_RETRY;
	}

	return 0;
}

int sdiom_tx_sendbuf_rd_switch(void)
{

	struct sdiom_tx_sendbuf_t *cur_tx_sbuf;

	/* switch to next one */
	s_tx_sendbuf_rd_node = s_tx_sendbuf_rd_node->next;
	cur_tx_sbuf =
	    list_entry(s_tx_sendbuf_rd_node, struct sdiom_tx_sendbuf_t, node);

	sdiom_print("sdiom_tx_sendbuf_rd_switch switch to sbuf[%d]\n",
		    cur_tx_sbuf->index);

	return 0;
}

struct sdiom_tx_sendbuf_t *sdiom_tx_sendbuf_rd_getptr(void)
{
	struct sdiom_tx_sendbuf_t *cur_tx_sbuf;

	cur_tx_sbuf =
	    list_entry(s_tx_sendbuf_rd_node, struct sdiom_tx_sendbuf_t, node);

	return cur_tx_sbuf;
}

void sdiom_tx_sendbuf_rd_calc_retrybuf(unsigned int suc_pac_cnt)
{
	struct sdiom_tx_sendbuf_t *cur_tx_sbuf;

	unsigned int cnt = 0;
	struct sdiom_tx_puh_t *puh = NULL;
	unsigned char *p = NULL;

	if (suc_pac_cnt == 0)
		return;

	cur_tx_sbuf =
	    list_entry(s_tx_sendbuf_rd_node, struct sdiom_tx_sendbuf_t, node);

	puh = (struct sdiom_tx_puh_t *)cur_tx_sbuf->retry_buf;

	for (cnt = 0; cnt < suc_pac_cnt;) {
		if (puh->eof == 0) {
			p = (unsigned char *)puh;

			cnt++;

			cur_tx_sbuf->retry_len =
			    cur_tx_sbuf->retry_len -
			    sizeof(struct sdiom_tx_puh_t) -
			    SDIOM_ALIGN_32BIT(puh->len);

			/* pointer to next packet */
			p = p + sizeof(struct sdiom_tx_puh_t) +
			    SDIOM_ALIGN_32BIT(puh->len);

			puh = (struct sdiom_tx_puh_t *)p;

			cur_tx_sbuf->retry_buf = (unsigned char *)p;

		} else
			break;
	}

	sdiom_print("rd_calc_retrybuf exit retry_buf[%p] retry_len[%d]\n",
		    cur_tx_sbuf->retry_buf, cur_tx_sbuf->retry_len);
}

void sdiom_tx_sendbuf_rd_init_retrybuf(void)
{
	struct sdiom_tx_sendbuf_t *cur_tx_sbuf;

	cur_tx_sbuf =
	    list_entry(s_tx_sendbuf_rd_node, struct sdiom_tx_sendbuf_t, node);

	cur_tx_sbuf->retry_buf = cur_tx_sbuf->buf;
	cur_tx_sbuf->retry_len = cur_tx_sbuf->used_len;

	sdiom_print("rd_init_retrybuf retry_buf[%p] retry_len[%d]\n",
		    cur_tx_sbuf->retry_buf, cur_tx_sbuf->retry_len);
}

unsigned int sdiom_tx_sendbuf_get_free_len(struct sdiom_tx_sendbuf_t *sbuf)
{
	unsigned int free_len = 0;

	free_len = (sbuf->total_len - sbuf->used_len);

	return free_len;
}

unsigned int sdiom_tx_sendbuf_get_used_len(struct sdiom_tx_sendbuf_t *sbuf)
{
	return sbuf->used_len;
}

unsigned int sdiom_tx_sendbuf_usedlen_now(void)
{
	struct sdiom_tx_sendbuf_t *cur_tx_sbuf;
	unsigned int used_len;

	cur_tx_sbuf =
	    list_entry(s_tx_sendbuf_wt_node, struct sdiom_tx_sendbuf_t, node);

	used_len = sdiom_tx_sendbuf_get_used_len(cur_tx_sbuf);

	return used_len;
}

int sdiom_tx_sendbuf_fill(struct sdiom_tx_msg_t *tx_msg)
{
	struct sdiom_tx_sendbuf_t *cur_tx_sbuf;

SDIOM_TX_SENDBUF_FILL_RETRY:

	cur_tx_sbuf =
	    list_entry(s_tx_sendbuf_wt_node, struct sdiom_tx_sendbuf_t, node);

	sdiom_print("sdiom_tx_sendbuf_fill  using sbuf[%d]\n",
		    cur_tx_sbuf->index);

	if ((SDIOM_ALIGN_32BIT(tx_msg->len) +
	     2 * sizeof(struct sdiom_tx_puh_t)) >
	    sdiom_tx_sendbuf_get_free_len(cur_tx_sbuf)) {
		sdiom_tx_sendbuf_wt_switch();
		sdiom_tx_trans_up();
		/* next q */
		sdiom_tx_q_switch();
		goto SDIOM_TX_SENDBUF_FILL_RETRY;
	} else {
		sdiom_tx_packer(cur_tx_sbuf, tx_msg);
	}

	return 0;
}

void sdiom_tx_print_pack(struct sdiom_tx_sendbuf_t *tx_sbuf)
{
	struct sdiom_tx_sendbuf_t *cur_tx_sbuf;

	unsigned int cnt = 0;
	struct sdiom_tx_puh_t *puh = NULL;
	unsigned char *p = NULL;

	cur_tx_sbuf = tx_sbuf;

	puh = (struct sdiom_tx_puh_t *)cur_tx_sbuf->buf;

	for (cnt = 0; cnt < 0xffff;) {
		if (puh->eof == 0) {
			p = (unsigned char *)puh;

			sdiom_info("tx_pack[%d]type[%d]subtype[%d]len[%d]\n",
					cnt, puh->type, puh->subtype, puh->len);
			cnt++;

			/* pointer to next packet */

			p = p + sizeof(struct sdiom_tx_puh_t) +
			    SDIOM_ALIGN_32BIT(puh->len);

			puh = (struct sdiom_tx_puh_t *)p;
		} else {
			sdiom_info("tx_pack eof\n");
			break;
		}
	}
}


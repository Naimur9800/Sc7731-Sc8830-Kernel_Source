#include <linux/list.h>
#include <linux/vmalloc.h>

#include "sdiom.h"
#include "sdiom_depend.h"
#include "sdiom_rx_recvbuf.h"
#include "sdiom_rx_adapt.h"
#include "sdiom_sleep.h"
#include "sdiom_type.h"

static struct sdiom_rx_recvbuf_t
	s_rx_recvbuf_array[SDIOM_RX_RECVBUF_NUM] = { { { 0 } } };

static SDIOM_PT_RX_PROCESS_CALLBACK
	s_rx_callback_matrix[SDIOM_RX_TYPE_MAX][SDIOM_RX_TYPE_MAX] = { { 0 } };

static struct list_head *s_rx_recvbuf_wt_node;
static struct list_head *s_rx_recvbuf_rd_node;

int sdiom_rx_recvbuf_alloc(struct sdiom_rx_recvbuf_t *recvbuf, unsigned int len)
{
	sdiom_memset((char *)recvbuf, 0, sizeof(struct sdiom_rx_recvbuf_t));
	recvbuf->total_len = len;
	recvbuf->bufbk = NULL;
	recvbuf->buf = sdiom_malloc(len);
	if (recvbuf->buf == NULL)
		return ERROR;

	return OK;
}

static int sdiom_rx_recvbuf_vmalloc(struct sdiom_rx_recvbuf_t *recvbuf,
							unsigned int len)
{
	recvbuf->bufbk = vmalloc(len);
	if (recvbuf->bufbk == NULL)
		return ERROR;

	return OK;
}

int sdiom_rx_recvbufbk_alloc(void)
{
	unsigned int i, j;
	int ret;

	for (i = 0; i < SDIOM_RX_RECVBUF_NUM; i++) {
		ret = sdiom_rx_recvbuf_vmalloc(&s_rx_recvbuf_array[i],
			SDIOM_RX_RECVBUF_LEN);
		if (ret == ERROR) {
			for (j = 0; j < i; j++)
				vfree(s_rx_recvbuf_array[j].bufbk);
			sdiom_info("sdiom_rx_recvbufbk_alloc fail!\n");
			return ret;
		}
	}
	return OK;
}

int sdiom_rx_recvbuf_init(void)
{
	unsigned int i = 0;
	static unsigned int first;

	struct list_head *first_pos = NULL;
	struct list_head *pos = NULL;

	struct sdiom_rx_recvbuf_t *ptrbuf = NULL;

	for (i = 0; i < SDIOM_RX_RECVBUF_NUM; i++) {
		sdiom_rx_recvbuf_alloc(&s_rx_recvbuf_array[i],
				       SDIOM_RX_RECVBUF_LEN);
		s_rx_recvbuf_array[i].index = i;

		if (first == 0) {
			INIT_LIST_HEAD(&s_rx_recvbuf_array[i].node);
			s_rx_recvbuf_wt_node =
			(struct list_head *)(&s_rx_recvbuf_array[i].node.next);
			first_pos = s_rx_recvbuf_wt_node;
			first = !0;
		} else {
			list_add(&s_rx_recvbuf_array[i].node,
				 s_rx_recvbuf_wt_node);
			s_rx_recvbuf_wt_node = &s_rx_recvbuf_array[i].node;
		}
	}

	/* return cur_rx_recvbuf_node to head */
	s_rx_recvbuf_wt_node = first_pos;
	s_rx_recvbuf_rd_node = first_pos;

	sdiom_print("sdiom_rx_recvbuf_init after init\n");

	ptrbuf = (struct sdiom_rx_recvbuf_t *)list_entry(first_pos,
			struct sdiom_rx_recvbuf_t, node);

	for (pos = first_pos;;) {
		ptrbuf = (struct sdiom_rx_recvbuf_t *)list_entry(pos,
				struct sdiom_rx_recvbuf_t, node);

		pos = pos->next;

		if (pos == first_pos)
			break;
	}

	return 0;
}

void sdiom_rx_recvbuf_deinit(void)
{
	unsigned int i;

	if (sdiom_get_debug_level() == LEVEL_0)
		return;

	for (i = 0; i < SDIOM_RX_RECVBUF_NUM; i++)
		vfree(s_rx_recvbuf_array[i].bufbk);
}

void sdiom_rx_print_pack(struct sdiom_rx_recvbuf_t *rx_rbuf)
{
	struct sdiom_rx_recvbuf_t *cur_rx_rbuf;

	unsigned int cnt = 0;
	struct sdiom_rx_puh_t *puh = NULL;
	unsigned char *p = NULL;

	cur_rx_rbuf = rx_rbuf;

	puh = (struct sdiom_rx_puh_t *)cur_rx_rbuf->buf;

	for (cnt = 0; cnt < 0xffff;) {
		if (puh->eof == 0) {
			p = (unsigned char *)puh;

			/* parse info and send to callback */

			if (puh->type != 0xF) {

				sdiom_info("rx_pack[%d]type[%d]subtype[%d]\n",
						cnt, puh->type, puh->subtype);

				cnt++;
			}

			/* pointer to next packet */

			p = p + sizeof(struct sdiom_rx_puh_t) +
			    SDIOM_ALIGN_32BIT(puh->len);

			puh = (struct sdiom_rx_puh_t *)p;
		} else {
			sdiom_info("rx_pack eof total[%d] free[%d] id[%d]\n",
					atomic_read(&cur_rx_rbuf->total_packet),
					atomic_read(&cur_rx_rbuf->free_packet),
					cur_rx_rbuf->index);
			break;
		}
	}
}

int sdiom_rx_recvbuf_wt_switch(void)
{

	struct sdiom_rx_recvbuf_t *cur_rx_rbuf;
	unsigned int retry_cnt = 0;

	s_rx_recvbuf_wt_node = s_rx_recvbuf_wt_node->next;

	cur_rx_rbuf = list_entry(s_rx_recvbuf_wt_node,
				 struct sdiom_rx_recvbuf_t, node);

SDIOM_RX_RECVBUF_WT_RETRY:

	if (atomic_read(&cur_rx_rbuf->busy) == 0) {
		sdiom_print("%s switch to rbuf[%d]\n",
			    __func__, cur_rx_rbuf->index);
	} else {
		if (!retry_cnt) {
			sdiom_info("%s rbuf[%d] is busy, do retry\n",
				   __func__, cur_rx_rbuf->index);

			sdiom_rx_print_pack(cur_rx_rbuf);
		}
		usleep_range(700, 1000);
		retry_cnt++;
		goto SDIOM_RX_RECVBUF_WT_RETRY;
	}
	if (retry_cnt)
		sdiom_info("%s rbuf[%d] is busy.retry_cnt[%d]\n",
			   __func__, cur_rx_rbuf->index, retry_cnt);

	return 0;
}

int sdiom_rx_recvbuf_rd_switch(void)
{

	struct sdiom_rx_recvbuf_t *cur_rx_rbuf;

	s_rx_recvbuf_rd_node = s_rx_recvbuf_rd_node->next;
	cur_rx_rbuf =
	    list_entry(s_rx_recvbuf_rd_node, struct sdiom_rx_recvbuf_t, node);

	sdiom_print("sdiom_rx_recvbuf_rd_switch switch to rbuf[%d]\n",
		    cur_rx_rbuf->index);

	return 0;
}

struct sdiom_rx_recvbuf_t *sdiom_rx_recvbuf_rd_getptr(void)
{
	struct sdiom_rx_recvbuf_t *cur_tx_sbuf;

	cur_tx_sbuf =
	    list_entry(s_rx_recvbuf_rd_node, struct sdiom_rx_recvbuf_t, node);

	return cur_tx_sbuf;
}

int sdiom_rx_recvbuf_fill(void)
{
	/* should use adaptive algorithm, now no use */
	unsigned int read_len = 0;
	unsigned int rx_dtbs = 0;
	int ret = 0;

	struct sdiom_rx_recvbuf_t *cur_rx_rbuf;

	cur_rx_rbuf =
	    list_entry(s_rx_recvbuf_wt_node, struct sdiom_rx_recvbuf_t, node);

	sdiom_print("sdiom_rx_recvbuf_fill using rbuf[%d]\n",
		    cur_rx_rbuf->index);

	read_len = sdiom_rx_adapt_get();

	ret = sdiom_sdio_pt_read(cur_rx_rbuf->buf, read_len);
	if (ret != 0) {
		sdiom_err("pt read fail ret:%d\n", ret);
		rx_dtbs = 0;
		goto submit_list;
	}

	rx_dtbs = *((unsigned int *)(cur_rx_rbuf->buf + (read_len - 4)));
	cur_rx_rbuf->used_len =
		*((unsigned int *)(cur_rx_rbuf->buf + (read_len - 8)));

	if ((sdiom_get_debug_level() == LEVEL_1) && (cur_rx_rbuf->bufbk)) {
		memset(cur_rx_rbuf->bufbk, 0,
			sizeof(unsigned char)*SDIOM_RX_RECVBUF_LEN);
		memcpy(cur_rx_rbuf->bufbk, cur_rx_rbuf->buf, read_len);
		sdiom_print_buf(cur_rx_rbuf->bufbk, read_len, __func__);
	}

	sdiom_rx_adapt_set_dtbs(rx_dtbs);

	atomic_set(&cur_rx_rbuf->busy, 1);

	sdiom_rx_core_up();

	sdiom_rx_recvbuf_wt_switch();

submit_list:
	if (rx_dtbs > 0) {
		sdiom_rx_trans_up();
	} else {
		/* enable interrupt */
		sdiom_enable_rx_irq();
		sdiom_cp_rx_sleep();
		sdiom_unlock_rx_wakelock();
	}

	return 0;
}

void sdiom_rx_packet_release(unsigned int id)
{
	atomic_inc(&s_rx_recvbuf_array[id].free_packet);

	sdiom_print
	    ("sdiom_rx_packet_release free recvbuf[%d] free[%d] total[%d]\n",
	     id, atomic_read(&s_rx_recvbuf_array[id].free_packet),
	     atomic_read(&s_rx_recvbuf_array[id].total_packet));

	if (atomic_read(&s_rx_recvbuf_array[id].free_packet) ==
	    atomic_read(&s_rx_recvbuf_array[id].total_packet)) {
		sdiom_print("sdiom_rx_packet_release free recvbuf[%d]\n", id);
		/* free buf */
		atomic_set(&s_rx_recvbuf_array[id].total_packet, 0);
		atomic_set(&s_rx_recvbuf_array[id].free_packet, 0);
		atomic_set(&s_rx_recvbuf_array[id].busy, 0);
	}
}

void sdiom_rx_callback_set(unsigned int type, unsigned int subtype,
			   void *callback)
{
	if ((type < SDIOM_RX_TYPE_MAX) && (subtype < SDIOM_RX_SUB_MAX))
		s_rx_callback_matrix[type][subtype] =
			(SDIOM_PT_RX_PROCESS_CALLBACK) callback;
	else
		sdiom_print("%s, argument is err!\n", __func__);
}

SDIOM_PT_RX_PROCESS_CALLBACK sdiom_rx_callback_get(unsigned int type,
						   unsigned int subtype)
{
	if ((type < SDIOM_RX_TYPE_MAX) && (subtype < SDIOM_RX_SUB_MAX))
		return s_rx_callback_matrix[type][subtype];
	sdiom_print("%s, argument is err!\n", __func__);

	return NULL;
}

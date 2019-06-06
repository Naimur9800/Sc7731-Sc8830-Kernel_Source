#include "sdiom_depend.h"
#include "sdiom_tx_fifo.h"
#include "sdiom_type.h"

#define SDIOM_TX_TYPE_MAX	4
#define SDIOM_TX_SUBTYPE_MAX	4

static struct sdiom_tx_q_t
s_tx_q_matrix[SDIOM_TX_TYPE_MAX][SDIOM_TX_SUBTYPE_MAX];

static SDIOM_PT_TX_RELEASE_CALLBACK
s_tx_callback_matrix[SDIOM_TX_TYPE_MAX][SDIOM_TX_SUBTYPE_MAX] = { {0} };

static unsigned int
s_tx_q_num_table[SDIOM_TX_TYPE_MAX][SDIOM_TX_SUBTYPE_MAX] = {
	{10, 10, 10, 10},	/*[0][0]~[0][3] */
	{10, 10, 10, 10},	/*[1][0]~[1][3] */
	{10, 50, 10, 10},	/*[2][0]~[2][3] */
	{10, 10, 10, 10},	/*[3][0]~[3][3] */
};

static struct list_head *cur_tx_fifo_node;

static atomic_t s_tx_fifo_cnt = ATOMIC_INIT(0);

static unsigned int tx_q_init_indicator;

void sdiom_tx_fifo_cnt_inc(void)
{
	atomic_inc(&s_tx_fifo_cnt);
}

void sdiom_tx_fifo_cnt_dec(void)
{
	if (atomic_read(&s_tx_fifo_cnt) == 0)
		return;

	atomic_dec(&s_tx_fifo_cnt);
}

int sdiom_is_tx_fifo_empty(void)
{
	if (atomic_read(&s_tx_fifo_cnt) == 0)
		return OK;
	else
		return ERROR;
}

int sdiom_tx_msg_enq(struct sdiom_tx_q_t *sdiom_tx_q,
		     struct sdiom_tx_msg_t *sdiom_tx_msg)
{
	if ((sdiom_tx_q->wt + 1) % (sdiom_tx_q->total_num) == sdiom_tx_q->rd) {
		sdiom_print("sdiom_tx_q_in q_full wt[%d] rd[%d]\n",
			    sdiom_tx_q->wt, sdiom_tx_q->rd);
		return ERROR;
	}
	memcpy((sdiom_tx_q->mem + (sdiom_tx_q->size) * (sdiom_tx_q->wt)),
	       (unsigned char *)sdiom_tx_msg, sdiom_tx_q->size);
	sdiom_tx_q->wt =
	    INCR_RING_BUFF_INDX(sdiom_tx_q->wt, sdiom_tx_q->total_num);
	sdiom_tx_q->wt_cnt++;

	return OK;
}

struct sdiom_tx_msg_t *sdiom_tx_msg_get(struct sdiom_tx_q_t *sdiom_tx_q)
{
	struct sdiom_tx_msg_t *sdiom_tx_msg = NULL;

	if (sdiom_tx_q->wt == sdiom_tx_q->rd)
		return NULL;
	sdiom_tx_msg =
	    (struct sdiom_tx_msg_t *)(sdiom_tx_q->mem +
				      (sdiom_tx_q->size) * (sdiom_tx_q->rd));
	return sdiom_tx_msg;
}

int sdiom_tx_msg_deq(struct sdiom_tx_q_t *sdiom_tx_q)
{
	if (sdiom_tx_q->rd == sdiom_tx_q->wt)
		return ERROR;

	sdiom_tx_q->rd =
	    INCR_RING_BUFF_INDX(sdiom_tx_q->rd, sdiom_tx_q->total_num);

	sdiom_tx_q->rd_cnt++;

	return OK;
}

int sdiom_tx_msg_num(struct sdiom_tx_q_t *sdiom_tx_q)
{
	return sdiom_tx_q->wt_cnt - sdiom_tx_q->rd_cnt;
}

int sdiom_tx_q_alloc(struct sdiom_tx_q_t *sdiom_tx_q, unsigned short total_num)
{
	sdiom_memset((char *)sdiom_tx_q, 0, sizeof(struct sdiom_tx_q_t));

	sdiom_tx_q->mem =
	    sdiom_malloc(sizeof(struct sdiom_tx_msg_t) * total_num);

	if (sdiom_tx_q->mem == NULL)
		return ERROR;

	sdiom_tx_q->total_num = total_num;
	sdiom_tx_q->size = sizeof(struct sdiom_tx_msg_t);

	return OK;
}

int sdiom_tx_q_free(struct sdiom_tx_q_t *sdiom_tx_q)
{
	sdiom_free(sdiom_tx_q->mem);
	sdiom_memset((char *)sdiom_tx_q, 0, sizeof(struct sdiom_tx_q_t));

	return OK;
}

int sdiom_tx_q_init(void)
{
	unsigned int i = 0;
	unsigned int j = 0;
	static unsigned int first;

	struct list_head *first_pos = NULL;
	struct list_head *pos = NULL;

	struct sdiom_tx_q_t *ptq = NULL;

	sdiom_print("sdiom_tx_q_init enter\n");

	for (i = 0; i < SDIOM_TX_TYPE_MAX; i++)
		for (j = 0; j < SDIOM_TX_SUBTYPE_MAX; j++) {
			if (s_tx_q_num_table[i][j] != 0) {

				sdiom_tx_q_alloc(&s_tx_q_matrix[i][j],
						 s_tx_q_num_table[i][j]);

				s_tx_q_matrix[i][j].type = i;
				s_tx_q_matrix[i][j].subtype = j;

				if (first == 0) {
					INIT_LIST_HEAD(&s_tx_q_matrix[i]
						       [j].node);

					cur_tx_fifo_node =
					    s_tx_q_matrix[i][j].node.next;

					first_pos = cur_tx_fifo_node;
					first = !0;

				} else {
					list_add(&s_tx_q_matrix[i][j].node,
						 cur_tx_fifo_node);

					cur_tx_fifo_node =
					    &s_tx_q_matrix[i][j].node;
				}
			} else
				s_tx_q_matrix[i][j].total_num = 0;

		}

	cur_tx_fifo_node = first_pos;

	sdiom_print("sdiom_tx_q_init after init\n");

	ptq = (struct sdiom_tx_q_t *)list_entry(first_pos,
						struct sdiom_tx_q_t, node);

	sdiom_print("first type[%d] subtype[%d]\n", ptq->type, ptq->subtype);

	for (pos = first_pos;;) {
		ptq = (struct sdiom_tx_q_t *)list_entry(pos,
							struct sdiom_tx_q_t,
							node);

		sdiom_print("cur type[%d] subtype[%d]\n", ptq->type,
			    ptq->subtype);

		pos = pos->next;
		if (pos == first_pos)
			break;
	}

	tx_q_init_indicator = true;

	return 0;
}

struct sdiom_tx_q_t *sdiom_tx_q_getptr(unsigned int type, unsigned int subtype)
{
	if (unlikely(tx_q_init_indicator != true))
		return NULL;

	if ((type > SDIOM_TX_TYPE_MAX)
	    || (subtype > SDIOM_TX_SUBTYPE_MAX)) {
		return NULL;
	} else {
		return &s_tx_q_matrix[type][subtype];
	}
}

struct sdiom_tx_q_t *sdiom_tx_q_find_data(void)
{
	struct list_head *pos = NULL;
	struct sdiom_tx_q_t *ptq = NULL;

	for (pos = cur_tx_fifo_node;;) {

		ptq = (struct sdiom_tx_q_t *)list_entry(pos,
							struct sdiom_tx_q_t,
							node);

		if (ptq->wt != ptq->rd) {
			cur_tx_fifo_node = pos;
			return ptq;
		}
		pos = pos->next;
		if (pos == cur_tx_fifo_node)
			return NULL;
	}
}

void sdiom_tx_q_switch(void)
{
	struct list_head *pos = NULL;

	pos = cur_tx_fifo_node;
	pos = pos->next;
	cur_tx_fifo_node = pos;
}

void sdiom_tx_callback_set(unsigned int type, unsigned int subtype,
			   void *callback)
{
	if ((type < SDIOM_TX_TYPE_MAX) && (subtype < SDIOM_TX_SUBTYPE_MAX))
		s_tx_callback_matrix[type][subtype] =
	 (SDIOM_PT_TX_RELEASE_CALLBACK) callback;
	 else
		return;

}

SDIOM_PT_TX_RELEASE_CALLBACK sdiom_tx_callback_get(unsigned int type,
						   unsigned int subtype)
{
	if ((type < SDIOM_TX_TYPE_MAX) &&
		(subtype < SDIOM_TX_SUBTYPE_MAX))
		return s_tx_callback_matrix[type][subtype];
	else
		return NULL;
}

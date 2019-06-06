#ifndef __SDIOM_TX_FIFO_H__
#define __SDIOM_TX_FIFO_H__

#include "sdiom_type.h"

#define INCR_RING_BUFF_INDX(indx, max_num) \
	((((indx) + 1) < (max_num)) ? ((indx) + 1) : (0))

struct sdiom_tx_msg_t {
	unsigned int len;
	unsigned char *buf;
	unsigned char *skb_buf;
	unsigned char type;
	unsigned char subtype;
};

struct sdiom_tx_q_t {
	struct list_head node;
	unsigned int wt;
	unsigned int rd;
	unsigned char *mem;
	unsigned int total_num;
	unsigned int size;
	unsigned int wt_cnt;
	unsigned int rd_cnt;
	unsigned char type;
	unsigned char subtype;
	/* spinlock_t wt_lock; */
};

int sdiom_tx_q_init(void);

void sdiom_tx_q_switch(void);

int sdiom_tx_msg_enq(struct sdiom_tx_q_t *sdiom_tx_q,
	struct sdiom_tx_msg_t *sdiom_tx_msg);

int sdiom_tx_msg_deq(struct sdiom_tx_q_t *sdiom_tx_q);

struct sdiom_tx_q_t *sdiom_tx_q_getptr(unsigned int type, unsigned int subtype);

struct sdiom_tx_q_t *sdiom_tx_q_find_data(void);

struct sdiom_tx_msg_t *sdiom_tx_msg_get(struct sdiom_tx_q_t *sdiom_tx_q);

SDIOM_PT_TX_RELEASE_CALLBACK sdiom_tx_callback_get(unsigned int type,
						   unsigned int subtype);

void sdiom_tx_callback_set(unsigned int type, unsigned int subtype,
			   void *callback);

void sdiom_tx_fifo_cnt_inc(void);

void sdiom_tx_fifo_cnt_dec(void);

int sdiom_is_tx_fifo_empty(void);

#endif /* __SDIOM_TX_FIFO_H__ */

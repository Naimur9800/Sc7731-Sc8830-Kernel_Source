#ifndef __SDIOM_TX_SENDBUF_H__
#define __SDIOM_TX_SENDBUF_H__

#include <linux/list.h>

struct sdiom_tx_puh_t {
	unsigned int pad:7;
	unsigned int len:16;
	unsigned int eof:1;
	unsigned int subtype:4;
	unsigned int type:4;
};		/* 32bits public header */

struct sdiom_tx_sendbuf_t {
	struct list_head node;
	unsigned int index;
	unsigned int total_len;
	unsigned int used_len;
	unsigned char *buf;
	atomic_t busy;		/* 0:idle 1:busy */
	unsigned char *retry_buf;
	unsigned int retry_len;
	unsigned int helper;
};

int sdiom_tx_sendbuf_init(void);

int sdiom_tx_sendbuf_wt_switch(void);

int sdiom_tx_sendbuf_rd_switch(void);

int sdiom_tx_sendbuf_fill(struct sdiom_tx_msg_t *tx_msg);

struct sdiom_tx_sendbuf_t *sdiom_tx_sendbuf_rd_getptr(void);

void sdiom_tx_sendbuf_rd_init_retrybuf(void);

void sdiom_tx_sendbuf_rd_calc_retrybuf(unsigned int suc_byte);

unsigned int sdiom_tx_sendbuf_usedlen_now(void);

void sdiom_tx_print_pack(struct sdiom_tx_sendbuf_t *tx_sbuf);

#endif /* __SDIOM_TX_SENDBUF_H__ */


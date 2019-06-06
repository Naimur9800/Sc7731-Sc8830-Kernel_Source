#ifndef __SDIOM_RX_RECVBUF_H__
#define __SDIOM_RX_RECVBUF_H__

#include "sdiom_type.h"

#define SDIOM_RX_RECVBUF_NUM 40
#define SDIOM_RX_RECVBUF_LEN (10 << 10)

#define SDIOM_TYPE_BT 0
#define SDIOM_TYPE_FM 1
#define SDIOM_TYPE_WIFI 2
#define SDIOM_TYPE_BSP 3
#define SDIOM_RX_TYPE_MAX 4
#define SDIOM_RX_SUB_MAX 4

struct sdiom_rx_puh_t {
	unsigned int pad:7;
	unsigned int len:16;
	unsigned int eof:1;
	unsigned int subtype:4;
	unsigned int type:4;
};		/* 32bits public header */

struct sdiom_rx_recvbuf_t {
	struct list_head node;
	unsigned int index;
	unsigned int total_len;
	unsigned int used_len;
	unsigned char *buf;
	unsigned char *bufbk;
	atomic_t busy;		/* 0:idle 1:busy atomic for multi-task */
	atomic_t total_packet;	/* atomic for multi-task */
	atomic_t free_packet;	/* atomic for multi-task */
	unsigned int helper;
};

int sdiom_rx_recvbufbk_alloc(void);

int sdiom_rx_recvbuf_init(void);

void sdiom_rx_recvbuf_deinit(void);

int sdiom_rx_recvbuf_wt_switch(void);

int sdiom_rx_recvbuf_rd_switch(void);

struct sdiom_rx_recvbuf_t *sdiom_rx_recvbuf_rd_getptr(void);

int sdiom_rx_recvbuf_fill(void);

void sdiom_rx_callback_set(unsigned int type, unsigned int subtype,
			   void *callback);

extern SDIOM_PT_RX_PROCESS_CALLBACK sdiom_rx_callback_get(unsigned int
							  type,
							  unsigned int subtype);

void sdiom_rx_buf_cnt_inc(unsigned int *data);
void sdiom_rx_buf_cnt_dec(unsigned int *data);

void sdiom_rx_buf_cnt_set(unsigned int *data, unsigned int val);

void sdiom_rx_packet_release(unsigned int id);

#endif /* __SDIOM_RX_RECVBUF_H__ */

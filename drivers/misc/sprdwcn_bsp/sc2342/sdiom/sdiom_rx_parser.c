#include <linux/list.h>
#include <linux/sdiom_tx_api.h>

#include "sdiom.h"
#include "sdiom_depend.h"
#include "sdiom_rx_recvbuf.h"
#include "sdiom_rx_parser.h"
#include "sdiom_type.h"

#define RX_PUH_SIZE sizeof(struct sdiom_rx_puh_t)

#define RX_WRONG_DATA_DEBUG

#ifdef RX_WRONG_DATA_DEBUG
static unsigned int sdiom_rx_parser_type(struct sdiom_rx_recvbuf_t *rbuf,
	struct sdiom_rx_puh_t *puh, unsigned int cnt, unsigned int parse_len)
{
	unsigned int index = 0;
	char assert_str[64];
	char str[5][5] = {"BT", "FM", "WIFI", "BSP", "SDIO"};

	if ((puh->type >= SDIOM_RX_TYPE_MAX) ||
		(puh->subtype >= SDIOM_RX_SUB_MAX) ||
		(puh->len > SDIOM_RX_RECVBUF_LEN) ||
		(puh->len == 0)) {
		sdiom_err("rx rbuf[%d] err[%d]type[%d]sub[%d]len[%d]\n",
			rbuf->index, cnt,
			puh->type, puh->subtype,
			puh->len);
		sdiom_err("rx buf:%p puh:%p parse_len[%d] used_len[%d]\n",
			rbuf->buf, puh,
			parse_len, rbuf->used_len);

		if (puh->type <= SDIOM_TYPE_BSP)
			index = puh->type;
		else
			index = SDIOM_RX_TYPE_MAX;
		sprintf(assert_str,
			"[%s] RX ERROR! type[%d]sub[%d]len[%d]",
			str[index], puh->type, puh->subtype,
			puh->len);
		sdiom_print_buf(rbuf->buf, puh->len, __func__);
		mdbg_assert_interface(assert_str);
		if (sdiom_get_debug_level() == LEVEL_1)
			panic("sdiom_rx_parser_type error");

		return ERROR;
	}

	return OK;
}
#endif

unsigned int sdiom_rx_parser_total_packet(struct sdiom_rx_recvbuf_t *rbuf)
{
	unsigned int cnt = 0;
	unsigned int cnt_dummy = 0;
	unsigned int cnt_all = 0;
	unsigned int parse_len;
	struct sdiom_rx_puh_t *puh = NULL;
	unsigned char *p = NULL;

	/* If assert or dump memory happened, not parse data again. */
	if (sdiom_get_carddump_status() != 0)
		return 0;

	atomic_set(&rbuf->total_packet, 0);
	atomic_set(&rbuf->free_packet, 0);

	puh = (struct sdiom_rx_puh_t *)rbuf->buf;

	for (parse_len = 0; parse_len < rbuf->used_len;) {
		if (puh->eof != 0)
			break;

		/* consider dummy packet */
		cnt_all++;
		sdiom_print
			("rx_total rbuf[%d]cnt[%d]type[%d]sub[%d]len[%d]\n",
			rbuf->index, cnt_all, puh->type, puh->subtype,
			puh->len);

		if (puh->type != 0xF) {
#ifdef RX_WRONG_DATA_DEBUG
			if (sdiom_rx_parser_type(rbuf, puh, cnt, parse_len))
				break;
#endif
			parse_len += puh->len;
			cnt++;
		} else
			cnt_dummy++;

		p = (unsigned char *)puh;
		p = p + RX_PUH_SIZE + SDIOM_ALIGN_32BIT(puh->len);
		puh = (struct sdiom_rx_puh_t *)p;
	}

	sdiom_print("parse_len[%d] used_len[%d]\n",
		parse_len, rbuf->used_len);
	sdiom_print("rx_total rbuf[%d] cnt[%d] cnt_dummy[%d]\n",
		rbuf->index, cnt, cnt_dummy);

	atomic_set(&rbuf->total_packet, cnt);

	/* if pure dummy stream let the buf free */
	if (cnt == 0)
		atomic_set(&rbuf->busy, 0);

	return cnt;
}

void sdiom_rx_parser(struct sdiom_rx_recvbuf_t *rbuf)
{
	unsigned int cnt = 0;
	unsigned int cnt_dummy = 0;
	unsigned int parse_len, used_len;
	struct sdiom_rx_puh_t *puh = NULL;
	unsigned char *p = NULL;

	SDIOM_PT_RX_PROCESS_CALLBACK rx_process_cb = NULL;

	/* not deal with pure dummy stream */
	if (atomic_read(&rbuf->total_packet) == 0)
		return;

	puh = (struct sdiom_rx_puh_t *)rbuf->buf;
	used_len = rbuf->used_len;

	for (parse_len = 0; parse_len < used_len;) {
		if (puh->eof != 0)
			break;

		p = (unsigned char *)puh;

		/* parse info and send to callback */
		if (puh->type != 0xF) {
			if ((puh->type >= SDIOM_RX_TYPE_MAX) ||
				(puh->subtype >= SDIOM_RX_SUB_MAX) ||
				(puh->len > SDIOM_RX_RECVBUF_LEN) ||
				(puh->len == 0)) {

				sdiom_err("skip[%d]type[%d]sub[%d]len[%d]\n",
					cnt, puh->type, puh->subtype, puh->len);

				break;
			}
			parse_len += puh->len;
			cnt++;
			sdiom_rx_cb_lock();
			rx_process_cb =
				sdiom_rx_callback_get(puh->type,
					puh->subtype);

			if (rx_process_cb != NULL) {
				rx_process_cb((p + RX_PUH_SIZE),
					puh->len, rbuf->index);
			} else
				sdiom_rx_packet_release(rbuf->index);
			sdiom_rx_cb_unlock();

		} else
			cnt_dummy++;
		/* pointer to next packet */

		p = p + RX_PUH_SIZE + SDIOM_ALIGN_32BIT(puh->len);
		puh = (struct sdiom_rx_puh_t *)p;
	}
}

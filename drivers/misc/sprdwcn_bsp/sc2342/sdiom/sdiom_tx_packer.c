#include "sdiom_depend.h"
#include "sdiom_tx_fifo.h"
#include "sdiom_tx_sendbuf.h"
#include "sdiom_tx_packer.h"
#include "sdiom_type.h"

int sdiom_tx_packer(struct sdiom_tx_sendbuf_t *target_sbuf,
		    struct sdiom_tx_msg_t *tx_msg)
{
	struct sdiom_tx_puh_t *puh = NULL;

	puh =
	    (struct sdiom_tx_puh_t *)(target_sbuf->buf + target_sbuf->used_len);

	puh->type = tx_msg->type;
	puh->subtype = tx_msg->subtype;
	puh->len = tx_msg->len;
	puh->eof = 0;
	puh->pad = 0;

	memcpy(target_sbuf->buf + target_sbuf->used_len +
	       sizeof(struct sdiom_tx_puh_t), tx_msg->buf, tx_msg->len);

	target_sbuf->used_len =
	    target_sbuf->used_len + sizeof(struct sdiom_tx_puh_t) +
	    SDIOM_ALIGN_32BIT(tx_msg->len);

	return OK;
}

int sdiom_tx_eof(struct sdiom_tx_sendbuf_t *target_sbuf)
{
	struct sdiom_tx_puh_t *puh = NULL;

	puh =
	    (struct sdiom_tx_puh_t *)(target_sbuf->buf + target_sbuf->used_len);

	puh->type = 0;
	puh->subtype = 0;
	puh->len = 0;
	puh->eof = 1;
	puh->pad = 0;

	target_sbuf->used_len =
	    target_sbuf->used_len + sizeof(struct sdiom_tx_puh_t);

	return OK;
}

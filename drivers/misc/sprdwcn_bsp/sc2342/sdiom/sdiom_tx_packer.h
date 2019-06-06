#ifndef __SDIOM_TX_PACKER_H__
#define __SDIOM_TX_PACKER_H__

int sdiom_tx_packer(struct sdiom_tx_sendbuf_t *target_sbuf,
	struct sdiom_tx_msg_t *tx_msg);

int sdiom_tx_eof(struct sdiom_tx_sendbuf_t *target_sbuf);

#endif /* __SDIOM_TX_PACKER_H__ */

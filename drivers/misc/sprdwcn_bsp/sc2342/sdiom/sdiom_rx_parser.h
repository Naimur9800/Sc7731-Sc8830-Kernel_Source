#ifndef __SDIOM_RX_PARSER_H__
#define __SDIOM_RX_PARSER_H__

extern void mdbg_assert_interface(char *str);

unsigned int sdiom_rx_parser_total_packet(struct sdiom_rx_recvbuf_t *rbuf);

void sdiom_rx_parser(struct sdiom_rx_recvbuf_t *rbuf);

#endif /* __SDIOM_RX_PARSER_H__ */

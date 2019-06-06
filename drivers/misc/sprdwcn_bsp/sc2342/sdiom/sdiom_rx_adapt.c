#include "sdiom_type.h"
#include "sdiom_depend.h"
#include "sdiom_rx_adapt.h"
#include "sdiom_rx_recvbuf.h"

static unsigned int dtbs = 2048;

unsigned int sdiom_rx_adapt_get(void)
{
	/*
	 * should use adaptive algorithm,
	 * but now no use
	 */
	return dtbs;
}

void sdiom_rx_adapt_set_dtbs(unsigned int len)
{
	unsigned int off;

	if (len == 0) {
		dtbs = 2048;
		return;
	}

	off = (len >> 10) + 1;
	len = (len + 8 * off + 64 + 511) & (~511);
	dtbs = (len >= SDIOM_RX_RECVBUF_LEN) ? SDIOM_RX_RECVBUF_LEN : len;
}

#include <linux/list.h>
#include <linux/sdiom_rx_api.h>

#include "sdiom_depend.h"
#include "sdiom_rx_default.h"
#include "sdiom_rx_recvbuf.h"
#include "sdiom_type.h"

static unsigned int rx_0_0(void *addr, unsigned int len, unsigned int fifo_id)
{
	sdiom_print("rx_0_0 addr[%p] len[%d] fifo_id[%d]\n ",
			addr, len, fifo_id);
	sdiom_pt_read_release(fifo_id);

	return 0;
}

static unsigned int rx_0_1(void *addr, unsigned int len, unsigned int fifo_id)
{
	sdiom_print("rx_0_1 addr[%p] len[%d] fifo_id[%d]\n ",
			addr, len, fifo_id);
	sdiom_pt_read_release(fifo_id);

	return 0;
}

static unsigned int rx_0_2(void *addr, unsigned int len, unsigned int fifo_id)
{
	sdiom_print("rx_0_2 addr[%p] len[%d] fifo_id[%d]\n ",
			addr, len, fifo_id);
	sdiom_pt_read_release(fifo_id);

	return 0;
}

static unsigned int rx_0_3(void *addr, unsigned int len, unsigned int fifo_id)
{
	sdiom_print("rx_0_3 addr[%p] len[%d] fifo_id[%d]\n ",
			addr, len, fifo_id);
	sdiom_pt_read_release(fifo_id);

	return 0;
}

static unsigned int rx_1_0(void *addr, unsigned int len, unsigned int fifo_id)
{
	sdiom_print("rx_1_0 addr[%p] len[%d] fifo_id[%d]\n ",
			addr, len, fifo_id);
	sdiom_pt_read_release(fifo_id);

	return 0;
}

static unsigned int rx_1_1(void *addr, unsigned int len, unsigned int fifo_id)
{
	sdiom_print("rx_1_1 addr[%p] len[%d] fifo_id[%d]\n ",
			addr, len, fifo_id);
	sdiom_pt_read_release(fifo_id);

	return 0;
}

static unsigned int rx_1_2(void *addr, unsigned int len, unsigned int fifo_id)
{
	sdiom_print("rx_1_2 addr[%p] len[%d] fifo_id[%d]\n ",
			addr, len, fifo_id);
	sdiom_pt_read_release(fifo_id);

	return 0;
}

static unsigned int rx_1_3(void *addr, unsigned int len, unsigned int fifo_id)
{
	sdiom_print("rx_1_3 addr[%p] len[%d] fifo_id[%d]\n ",
			addr, len, fifo_id);
	sdiom_pt_read_release(fifo_id);

	return 0;
}

static unsigned int rx_2_0(void *addr, unsigned int len, unsigned int fifo_id)
{
	sdiom_print("rx_2_0 addr[%p] len[%d] fifo_id[%d]\n ",
			addr, len, fifo_id);
	sdiom_pt_read_release(fifo_id);

	return 0;
}

static unsigned int rx_2_1(void *addr, unsigned int len, unsigned int fifo_id)
{
	sdiom_print("rx_2_1 addr[%p] len[%d] fifo_id[%d]\n ",
			addr, len, fifo_id);
	sdiom_pt_read_release(fifo_id);

	return 0;
}

static unsigned int rx_2_2(void *addr, unsigned int len, unsigned int fifo_id)
{
	sdiom_print("rx_2_2 addr[%p] len[%d] fifo_id[%d]\n ",
			addr, len, fifo_id);
	sdiom_pt_read_release(fifo_id);

	return 0;
}

static unsigned int rx_2_3(void *addr, unsigned int len, unsigned int fifo_id)
{
	sdiom_print("rx_2_3 addr[%p] len[%d] fifo_id[%d]\n ",
			addr, len, fifo_id);
	sdiom_pt_read_release(fifo_id);

	return 0;
}

static unsigned int rx_3_0(void *addr, unsigned int len, unsigned int fifo_id)
{
	sdiom_print("rx_3_0 addr[%p] len[%d] fifo_id[%d]\n ",
			addr, len, fifo_id);
	sdiom_pt_read_release(fifo_id);

	return 0;
}

static unsigned int rx_3_1(void *addr, unsigned int len, unsigned int fifo_id)
{
	sdiom_print("rx_3_1 addr[%p] len[%d] fifo_id[%d]\n ",
			addr, len, fifo_id);
	sdiom_pt_read_release(fifo_id);

	return 0;
}

static unsigned int rx_3_2(void *addr, unsigned int len, unsigned int fifo_id)
{
	sdiom_print("rx_3_2 addr[%p] len[%d] fifo_id[%d]\n ",
			addr, len, fifo_id);
	sdiom_pt_read_release(fifo_id);

	return 0;
}

static unsigned int rx_3_3(void *addr, unsigned int len, unsigned int fifo_id)
{
	sdiom_print("rx_3_3 addr[%p] len[%d] fifo_id[%d]\n ",
			addr, len, fifo_id);
	sdiom_pt_read_release(fifo_id);

	return 0;
}

void sdiom_rx_default(void)
{
	sdiom_print("sdiom_rx_default\n");

	sdiom_rx_callback_set(0, 0, (void *)rx_0_0);
	sdiom_rx_callback_set(0, 1, (void *)rx_0_1);
	sdiom_rx_callback_set(0, 2, (void *)rx_0_2);
	sdiom_rx_callback_set(0, 3, (void *)rx_0_3);

	sdiom_rx_callback_set(1, 0, (void *)rx_1_0);
	sdiom_rx_callback_set(1, 1, (void *)rx_1_1);
	sdiom_rx_callback_set(1, 2, (void *)rx_1_2);
	sdiom_rx_callback_set(1, 3, (void *)rx_1_3);

	sdiom_rx_callback_set(2, 0, (void *)rx_2_0);
	sdiom_rx_callback_set(2, 1, (void *)rx_2_1);
	sdiom_rx_callback_set(2, 2, (void *)rx_2_2);
	sdiom_rx_callback_set(2, 3, (void *)rx_2_3);

	sdiom_rx_callback_set(3, 0, (void *)rx_3_0);
	sdiom_rx_callback_set(3, 1, (void *)rx_3_1);
	sdiom_rx_callback_set(3, 2, (void *)rx_3_2);
	sdiom_rx_callback_set(3, 3, (void *)rx_3_3);
}

#include "sdiom_depend.h"
#include "sdiom_tx_core.h"
#include "sdiom_tx_fifo.h"
#include "sdiom_tx_sendbuf.h"
#include "sdiom_tx_trans.h"
#include "sdiom_type.h"
#include "sdiom_rx_core.h"
#include "sdiom_rx_default.h"
#include "sdiom_rx_recvbuf.h"
#include "sdiom_rx_trans.h"

static int tx_core_thread(void *data)
{
	sdiom_tx_core_task();

	return 0;
}

static int tx_trans_thread(void *data)
{
	sdiom_tx_trans_task();

	return 0;
}

static int rx_core_thread(void *data)
{
	sdiom_rx_core_task();

	return 0;
}

static int rx_trans_thread(void *data)
{
	sdiom_rx_trans_task();

	return 0;
}

static void sdiom_launch_thread(void)
{
	struct task_struct *task_tx_core;
	struct task_struct *task_tx_trans;
	struct task_struct *task_rx_core;
	struct task_struct *task_rx_trans;

	task_tx_core = kthread_create(tx_core_thread, NULL, "tx_core_thread");
	if (task_tx_core != 0)
		wake_up_process(task_tx_core);
	else {
		sdiom_err("create tx_core_thread fail\n");
		return;
	}

	task_tx_trans =
	    kthread_create(tx_trans_thread, NULL, "tx_trans_thread");
	if (task_tx_trans != 0)
		wake_up_process(task_tx_trans);
	else {
		sdiom_err("tx_trans_thread create fail\n");
		return;
	}

	task_rx_core = kthread_create(rx_core_thread, NULL, "rx_core_thread");
	if (task_rx_core != 0)
		wake_up_process(task_rx_core);
	else {
		sdiom_err("rx_core_thread create thread fail\n");
		return;
	}

	task_rx_trans =
	    kthread_create(rx_trans_thread, NULL, "rx_trans_thread");
	if (task_rx_trans != 0)
		wake_up_process(task_rx_trans);
	else {
		sdiom_err("tx_core_trans create thread fail\n");
		return;
	}
}

int sdiom_packer_init(void)
{
	sdiom_tx_q_init();
	sdiom_tx_sendbuf_init();

	sdiom_rx_recvbuf_init();

	sdiom_sema_init();
	sdiom_launch_thread();

	return 0;
}

int sdiom_packer_deinit(void)
{
	sdiom_rx_recvbuf_deinit();

	return 0;
}

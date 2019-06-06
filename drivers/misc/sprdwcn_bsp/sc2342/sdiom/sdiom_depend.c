#include <linux/mutex.h>
#include "sdiom_depend.h"
#include "sdiom_type.h"


static struct sdiom_sema_box_t s_sdiom_sema_box;
static struct mutex sdiom_callback_mutex;
static struct mutex sdiom_tx_mutex;

void *sdiom_memset(void *dest, int c, unsigned int count)
{
	return memset(dest, c, count);
}

void *sdiom_malloc(unsigned int size)
{
	return kmalloc(size, GFP_KERNEL);
}

void sdiom_free(void *memblock)
{
	kfree(memblock);
}

void sdiom_sema_init(void)
{
	sema_init(&s_sdiom_sema_box.tx_core_sema, 0);
	sema_init(&s_sdiom_sema_box.tx_trans_sema, 0);
	sema_init(&s_sdiom_sema_box.rx_core_sema, 0);
	sema_init(&s_sdiom_sema_box.rx_trans_sema, 0);
}

void sdiom_tx_core_up(void)
{
	up(&s_sdiom_sema_box.tx_core_sema);
}

void sdiom_tx_core_down(void)
{
	down(&s_sdiom_sema_box.tx_core_sema);
}

void sdiom_tx_trans_up(void)
{
	up(&s_sdiom_sema_box.tx_trans_sema);
}

void sdiom_tx_trans_down(void)
{
	down(&s_sdiom_sema_box.tx_trans_sema);
}

void sdiom_rx_core_up(void)
{
	static int cnt_rx_core_up;

	cnt_rx_core_up++;
	sdiom_print("rx_core_up[%d]\n", cnt_rx_core_up);
	up(&s_sdiom_sema_box.rx_core_sema);
}

void sdiom_rx_core_down(void)
{
	down(&s_sdiom_sema_box.rx_core_sema);
}

void sdiom_rx_trans_up(void)
{
	static int cnt_rx_trans_up;

	cnt_rx_trans_up++;
	sdiom_print("rx_trans_up[%d]\n", cnt_rx_trans_up);
	up(&s_sdiom_sema_box.rx_trans_sema);
}

void sdiom_rx_trans_down(void)
{
	down(&s_sdiom_sema_box.rx_trans_sema);
}

void os_sleep(unsigned int i)
{
	msleep(i);
}

void sdiom_rx_cb_mutex_init(void)
{
	mutex_init(&sdiom_callback_mutex);
}
void sdiom_rx_cb_lock(void)
{
	mutex_lock(&sdiom_callback_mutex);
}

void sdiom_rx_cb_unlock(void)
{
	mutex_unlock(&sdiom_callback_mutex);
}

void sdiom_tx_mutex_init(void)
{
	mutex_init(&sdiom_tx_mutex);
}
void sdiom_tx_lock(void)
{
	mutex_lock(&sdiom_tx_mutex);
}

void sdiom_tx_unlock(void)
{
	mutex_unlock(&sdiom_tx_mutex);
}

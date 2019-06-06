#include <linux/wakelock.h>

#include "sdiom.h"
#include "sdiom_depend.h"
#include "linux/marlin_platform.h"
#include <linux/mutex.h>

atomic_t sdiom_wake;
atomic_t sdiom_tx_done;
atomic_t sdiom_resume_flag;

static struct wake_lock sdiom_tx_core_wakelock;
static struct wake_lock sdiom_tx_trans_wakelock;
static struct wake_lock sdiom_rx_wakelock;

static struct mutex sdiom_sleep_mutex;

void sdiom_cp_tx_sleep(void)
{
	sdiom_print("%s enter\n", __func__);
	marlin_set_sleep(MARLIN_SDIO_TX, TRUE);
}

void sdiom_cp_tx_wakeup(void)
{
	sdiom_print("%s enter\n", __func__);
	marlin_set_sleep(MARLIN_SDIO_TX, FALSE);
	marlin_set_wakeup(MARLIN_SDIO_TX);
}

void sdiom_cp_rx_sleep(void)
{
	sdiom_print("%s enter\n", __func__);
	marlin_set_sleep(MARLIN_SDIO_RX, TRUE);
}

void sdiom_cp_rx_wakeup(void)
{
	sdiom_print("%s enter\n", __func__);

	marlin_set_sleep(MARLIN_SDIO_RX, FALSE);
	marlin_set_wakeup(MARLIN_SDIO_RX);
}

void sdiom_wakelock_init(void)
{
	sdiom_print("%s enter\n", __func__);

	wake_lock_init(&sdiom_rx_wakelock, WAKE_LOCK_SUSPEND,
			"sdiom_rx_wakelock");

	wake_lock_init(&sdiom_tx_core_wakelock, WAKE_LOCK_SUSPEND,
			"sdiom_tx_core_wakelock");

	wake_lock_init(&sdiom_tx_trans_wakelock, WAKE_LOCK_SUSPEND,
			"sdiom_tx_trans_wakelock");
}

void sdiom_sleep_mutex_init(void)
{
	mutex_init(&sdiom_sleep_mutex);
}

void sdiom_sleep_flag_init(void)
{
	sdiom_print("%s enter\n", __func__);

	atomic_set(&sdiom_resume_flag, 1);
	atomic_set(&sdiom_wake, 0);
	atomic_set(&sdiom_tx_done, 0);
}

void sdiom_lock_rx_wakelock(void)
{
	sdiom_print("%s enter\n", __func__);
	wake_lock(&sdiom_rx_wakelock);
}


void sdiom_unlock_rx_wakelock(void)
{
	sdiom_print("%s enter\n", __func__);
	wake_unlock(&sdiom_rx_wakelock);
}

void sdiom_lock_tx_core_wakelock(void)
{
	sdiom_print("%s enter\n", __func__);
	wake_lock(&sdiom_tx_core_wakelock);
}

void sdiom_unlock_tx_core_wakelock(void)
{
	sdiom_print("%s enter\n", __func__);
	wake_unlock(&sdiom_tx_core_wakelock);
}

void sdiom_lock_tx_trans_wakelock(void)
{
	sdiom_print("%s enter\n", __func__);
	wake_lock(&sdiom_tx_trans_wakelock);
}

void sdiom_unlock_tx_trans_wakelock(void)
{
	sdiom_print("%s enter\n", __func__);
	wake_unlock(&sdiom_tx_trans_wakelock);
}

void sdiom_op_enter(void)
{
	mutex_lock(&sdiom_sleep_mutex);
}

void sdiom_op_leave(void)
{
	mutex_unlock(&sdiom_sleep_mutex);
}

void sdiom_resume_check(void)
{
	unsigned int cnt = 0;

	while (!atomic_read(&sdiom_resume_flag)) {
		if (cnt == 0) {
			sdiom_info("wait sdio resume %s\n", __func__);
			dump_stack();
		}
		usleep_range(4000, 6000);
		cnt++;
	}

}

void sdiom_resume_wait(void)
{
	while (!atomic_read(&sdiom_resume_flag)) {
		sdiom_print("sleep 5ms wait for sdio resume\n");
		usleep_range(4000, 6000);
	}
}

int sdiom_resume_wait_status(void)
{
	return atomic_read(&sdiom_resume_flag);
}
EXPORT_SYMBOL_GPL(sdiom_resume_wait_status);

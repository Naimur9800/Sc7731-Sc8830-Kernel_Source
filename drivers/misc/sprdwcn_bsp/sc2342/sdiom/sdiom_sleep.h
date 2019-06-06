#ifndef __SDIOM_SLEEP_H__
#define __SDIOM_SLEEP_H__

extern atomic_t sdiom_wake;
extern atomic_t sdiom_tx_done;
extern atomic_t sdiom_resume_flag;

void sdiom_cp_tx_sleep(void);
void sdiom_cp_tx_wakeup(void);

void sdiom_cp_rx_sleep(void);
void sdiom_cp_rx_wakeup(void);

void sdiom_wakelock_init(void);

void sdiom_sleep_flag_init(void);
void sdiom_sleep_mutex_init(void);

void sdiom_lock_de0_wakelock(void);
void sdiom_unlock_de0_wakelock(void);

void sdiom_lock_rx_wakelock(void);
void sdiom_unlock_rx_wakelock(void);

void sdiom_lock_tx_core_wakelock(void);
void sdiom_unlock_tx_core_wakelock(void);

void sdiom_lock_tx_trans_wakelock(void);
void sdiom_unlock_tx_trans_wakelock(void);

void sdiom_op_enter(void);
void sdiom_op_leave(void);

void sdiom_resume_check(void);
void sdiom_resume_wait(void);

#endif

#ifndef __SDIOM_TX_API_H__
#define __SDIOM_TX_API_H__

int sdiom_pt_write(void *buf, unsigned int len, unsigned int type,
			    unsigned int subtype);

int sdiom_pt_write_skb(void *buf, void *skb_buf, unsigned int len,
				unsigned int type, unsigned int subtype);

int sdiom_register_pt_tx_release(unsigned int type,
					  unsigned int subtype, void *func);

int sdiom_dt_write(unsigned int system_addr, void *buf,
	unsigned int len);

int sdiom_dt_write_bt(unsigned int system_addr, void *buf,
	unsigned int len);


int sdiom_aon_writeb(unsigned int addr, unsigned char val);

int sdiom_sdio_rescan(void);

void sdiom_register_rescan_cb(void *func);

int sdiom_init(void);

void sdiom_exit(void);

void sdiom_remove_card(void);

void sdiom_set_carddump_status(unsigned int flag);

unsigned int sdiom_get_carddump_status(void);

int sdiom_driver_register(void);

void sdiom_driver_unregister(void);

int sdiom_resume_wait_status(void);

int sdiom_sdio_pt_write_raw(unsigned char *src, unsigned int datalen, int retry);

#endif /*__SDIOM_TX_API_H__*/

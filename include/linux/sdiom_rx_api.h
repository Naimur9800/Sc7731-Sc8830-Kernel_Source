#ifndef __SDIOM_RX_API_H__
#define __SDIOM_RX_API_H__

int sdiom_register_pt_rx_process(unsigned int type,
					  unsigned int subtype, void *func);

int sdiom_pt_read_release(unsigned int fifo_id);

int sdiom_dt_read(unsigned int system_addr,
	void *buf, unsigned int len);

int sdiom_aon_readb(unsigned int addr, unsigned char *val);

unsigned long long sdiom_get_rx_total_cnt(void);

unsigned long long sdiom_get_status(void);

void sdiom_dump_aon_reg(void);

extern void mdbg_assert_interface(char *str);

#endif /* __SDIOM_RX_API_H__ */

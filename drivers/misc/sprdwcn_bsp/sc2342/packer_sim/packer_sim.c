/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>

#define lo4bit(x) ((x)&0x0F)
#define hi4bit(x) (((x)>>4)&0x0F)
#define hexformat "0x%x%x,0x%x%x,0x%x%x,0x%x%x"

#define RX_NUM 255

static void *buf_addr;
static bool can_use;
static char a[RX_NUM] = {1, 2, 3, 4, 5};
static unsigned int (*rx_cb)(void *addr, unsigned int len,
			unsigned int fifo_id);
static unsigned int (*tx_cb)(void *addr);
static struct timer_list wr_timer, rd_timer;

unsigned int sdiom_register_pt_rx_process(unsigned int type,
				unsigned int subtype, void *func)
{
	rx_cb = func;

	return 0;
}
EXPORT_SYMBOL(sdiom_register_pt_rx_process);

static unsigned int sdiom_register_pt_tx_release(unsigned int type,
					unsigned int subtype, void *func)
{
	tx_cb = func;

	return 0;
}
EXPORT_SYMBOL(sdiom_register_pt_tx_release);

unsigned int sdiom_pt_write(void *buf, unsigned int len, int type, int subtype)
{
	buf_addr = buf;
	mod_timer(&wr_timer, jiffies + msecs_to_jiffies(30));
	pr_info("%s write %d bytes\n", __func__, len);

	return len;
}
EXPORT_SYMBOL(sdiom_pt_write);

unsigned int sdiom_pt_read_release(unsigned int fifo_id)
{
	can_use = true;

	return 0;
}
EXPORT_SYMBOL(sdiom_pt_read_release);

unsigned int sdiom_dt_write(unsigned int system_addr,
		void *buf, unsigned int len)
{
	mod_timer(&rd_timer, jiffies + msecs_to_jiffies(100));
	pr_info("%s %d bytes\n", __func__, len);

	return len;
}
EXPORT_SYMBOL(sdiom_dt_write);

unsigned int sdiom_dt_read(unsigned int system_addr,
		void *buf, unsigned int len)
{
	pr_info("%s %d bytes\n", __func__, len);

	return len;
}
EXPORT_SYMBOL(sdiom_dt_read);

void wr_timer_cb(unsigned long data)
{
	tx_cb(buf_addr);
}

void rd_timer_cb(unsigned long data)
{
	if (true) {
		rx_cb(a, RX_NUM, 0);
		can_use = false;
	}
	mod_timer(&rd_timer, jiffies + msecs_to_jiffies(100));
}

void test_init(void)
{
	int i;

	setup_timer(&wr_timer, wr_timer_cb, 0);
	setup_timer(&rd_timer, rd_timer_cb, 0);
	can_use = true;
	for (i = 0; i < RX_NUM; i++)
		a[i] = '0' + i%10;
}
static int __init packer_sim_init(void)
{
	test_init();
	return 0;
}

static void __exit packer_sim_exit(void)
{
	del_timer(&wr_timer);
	del_timer(&rd_timer);
}

module_init(packer_sim_init);
module_exit(packer_sim_exit);

MODULE_AUTHOR("Spreadtrum BSP");
MODULE_DESCRIPTION("SPRD packer simulate driver");
MODULE_LICENSE("GPL");

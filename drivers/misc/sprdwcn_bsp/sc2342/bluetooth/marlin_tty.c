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

#include <linux/compat.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/kdev_t.h>
#include <linux/module.h>
#ifdef CONFIG_OF
#include <linux/of_device.h>
#endif
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/types.h>
#include <linux/vt_kern.h>
#include <linux/workqueue.h>
#ifndef MTTY_TEST
#include <linux/sdiom_rx_api.h>
#include <linux/sdiom_tx_api.h>
#include <linux/marlin_platform.h>
#endif
#include "marlin_tty.h"
#include "marlin_bt_lpm.h"
#include "marlin_rfkill.h"

#include "marlin2_sitm.h"

#define MTTY_DEV_MAX_NR		1

#define MTTY_STATE_OPEN		1
#define MTTY_STATE_CLOSE	0

#define lo4bit(x) ((x)&0x0F)
#define hi4bit(x) (((x)>>4)&0x0F)
#define hexformat "0x%x%x,0x%x%x,0x%x%x,0x%x%x"

#define SDIOM_WR_DIRECT_MOD
#ifdef SDIOM_WR_DIRECT_MOD
#define SDIOM_WR_DIRECT_MOD_ADDR 0x51004000
#endif

struct rx_data {
	unsigned char		*addr;
	unsigned int		len;
	unsigned int		fifo_id;
	struct list_head	entry;
};

struct mtty_device {
	struct mtty_init_data	*pdata;
	struct tty_port		*port0;
	struct tty_port		*port1;
	struct tty_struct	*tty;
	struct tty_driver	*driver;

	/* mtty state */
	uint32_t		state;
	struct mutex		stat_mutex;
	struct mutex		rw_mutex;
	struct list_head        rx_head;
	struct work_struct bt_rx_work;
	struct workqueue_struct *bt_rx_workqueue;
};

static struct mtty_device *mtty_dev;
static unsigned int que_task = 1;
static int que_sche = 1;

/* this is for test only */
#ifdef MTTY_TEST
#define RX_NUM 100
static void *buf_addr;
static char a[RX_NUM] = {1, 2, 3, 4, 5};
static unsigned int (*rx_cb)(void *addr, unsigned int len,
			unsigned int fifo_id);
static unsigned int (*tx_cb)(void *addr);
static struct timer_list test_timer;
static unsigned int sdiom_register_pt_rx_process(unsigned int type,
				unsigned int subtype, void *func)
{
	rx_cb = func;
	return 0;
}

static unsigned int sdiom_register_pt_tx_release(unsigned int type,
					unsigned int subtype, void *func)
{
	tx_cb = func;
	return 0;
}

unsigned int sdiom_pt_write(void *buf, unsigned int len, int type, int subtype)
{
	buf_addr = buf;
	mod_timer(&test_timer, jiffies + msecs_to_jiffies(30));
	return len;
}

unsigned int sdiom_dt_write(unsigned int system_addr, void *buf,
		unsigned int len)
{
	pr_info("%s %d bytes\n", __func__, len);
	mod_timer(&test_timer, jiffies + msecs_to_jiffies(30));
	return len;
}

unsigned int sdiom_pt_read_release(unsigned int fifo_id)
{
	return 0;
}

void timer_cb(unsigned long data)
{
	rx_cb(a, 10, 0);
	mod_timer(&test_timer, jiffies + msecs_to_jiffies(100));
}

void test_init(void)
{
	int i;

	for (i = 0; i < RX_NUM; i++)
		a[i] = '0' + i % 10;
}
#endif

/* static void mtty_rx_task(unsigned long data) */
static void mtty_rx_work_queue(struct work_struct *work)

{
	int i, ret = 0;
	/*struct mtty_device *mtty = (struct mtty_device *)data;*/
	struct mtty_device *mtty;
	struct rx_data *rx = NULL;

	 que_task = que_task + 1;
	 if (que_task > 65530)
		que_task = 0;
	pr_debug("mtty que_task= %d\n", que_task);
	que_sche = que_sche - 1;

	mtty = container_of(work, struct mtty_device, bt_rx_work);
	if (unlikely(!mtty)) {
		pr_err("mtty_rx_task mtty is NULL\n");
		return;
	}

	mutex_lock(&mtty->stat_mutex);
	if (mtty->state == MTTY_STATE_OPEN) {
		mutex_unlock(&mtty->stat_mutex);

		do {
			mutex_lock(&mtty->rw_mutex);
			if (list_empty_careful(&mtty->rx_head)) {
				pr_err("mtty over load queue done\n");
				mutex_unlock(&mtty->rw_mutex);
				break;
			}
			rx = list_first_entry_or_null(&mtty->rx_head,
				struct rx_data, entry);
			if (!rx) {
				pr_err("mtty over load queue abort\n");
				mutex_unlock(&mtty->rw_mutex);
				break;
			}
			list_del(&rx->entry);
			mutex_unlock(&mtty->rw_mutex);

			pr_err("mtty over load working at id: %d, len: %d\n",
				rx->fifo_id, rx->len);
			for (i = 0; i < rx->len; i++) {
				ret = tty_insert_flip_char(mtty->port0,
					*(rx->addr+i), TTY_NORMAL);
				if (ret != 1) {
					i--;
					continue;
				} else {
					tty_flip_buffer_push(mtty->port0);
				}
			}
			pr_err("mtty over load cut id: %d\n", rx->fifo_id);
			kfree(rx->addr);
			kfree(rx);

		} while (1);
	} else {
		pr_info("mtty status isn't open, status:%d\n", mtty->state);
		mutex_unlock(&mtty->stat_mutex);
	}
}

static unsigned int mtty_rx_cb(void *addr, unsigned int len,
		unsigned int fifo_id)
{
	int ret = 0;
	static unsigned int bit_count;
	struct rx_data *rx;
	unsigned char *sdio_buf = NULL;

	sdio_buf = (unsigned char *)addr;
	bt_wakeup_host();
	pr_debug("---mtty receive fifo_id= %d\n", fifo_id);

	if (len <= 4)
		pr_debug("mtty packer mode write count=%d,"hexformat"\n",
			len,
			hi4bit(sdio_buf[0]), lo4bit(sdio_buf[0]),
			hi4bit(sdio_buf[1]), lo4bit(sdio_buf[1]),
			hi4bit(sdio_buf[2]), lo4bit(sdio_buf[2]),
			hi4bit(sdio_buf[3]), lo4bit(sdio_buf[3]));

	else
		pr_debug("mtty callback count=%d,"hexformat"..."
			hexformat"\n", len,
			hi4bit(sdio_buf[0]), lo4bit(sdio_buf[0]),
			hi4bit(sdio_buf[1]), lo4bit(sdio_buf[1]),
			hi4bit(sdio_buf[2]), lo4bit(sdio_buf[2]),
			hi4bit(sdio_buf[3]), lo4bit(sdio_buf[3]),
			hi4bit(sdio_buf[len-4]),
			lo4bit(sdio_buf[len-4]),
			hi4bit(sdio_buf[len-3]),
			lo4bit(sdio_buf[len-3]),
			hi4bit(sdio_buf[len-2]),
			lo4bit(sdio_buf[len-2]),
			hi4bit(sdio_buf[len-1]),
			lo4bit(sdio_buf[len-1]));

	if (mtty_dev->state != MTTY_STATE_OPEN) {
		pr_err("mtty bt is closed abnormally\n");
		sdiom_pt_read_release(fifo_id);
		return -1;
	}

	if (mtty_dev != NULL) {
		if (!work_pending(&mtty_dev->bt_rx_work)) {
			ret = tty_insert_flip_string(mtty_dev->port0,
				(unsigned char *)addr,
					len);
			if (ret)
				tty_flip_buffer_push(mtty_dev->port0);
			if (ret == len) {
				sdiom_pt_read_release(fifo_id);
				return 0;
			}
		}

		rx = kmalloc(sizeof(struct rx_data), GFP_KERNEL);
		if (rx == NULL) {
			sdiom_pt_read_release(fifo_id);
			return -ENOMEM;
		}
		rx->len = len - ret;
		rx->addr = kmalloc(rx->len, GFP_KERNEL);
		if (rx->addr == NULL) {
			pr_err("mtty low memory!\n");
			sdiom_pt_read_release(fifo_id);
			kfree(rx);
			return -ENOMEM;
		}
		rx->fifo_id	= bit_count++;
		memcpy(rx->addr, addr, rx->len);
		sdiom_pt_read_release(fifo_id);
		mutex_lock(&mtty_dev->rw_mutex);
		pr_err("mtty over load push %d -> %d, id: %d len: %d\n",
			len, ret, rx->fifo_id, rx->len);
		list_add_tail(&rx->entry, &mtty_dev->rx_head);
		mutex_unlock(&mtty_dev->rw_mutex);
		if (!work_pending(&mtty_dev->bt_rx_work)) {
			queue_work(mtty_dev->bt_rx_workqueue,
				&mtty_dev->bt_rx_work);
		}

		return 0;
	}
	pr_err("mtty_rx_cb mtty_dev is NULL!!!\n");

	return -1;
}

static unsigned int mtty_tx_cb(void *addr)
{
	kfree(addr);

	return 1;
}

static int mtty_open(struct tty_struct *tty, struct file *filp)
{
	struct mtty_device *mtty = NULL;
	struct tty_driver *driver = NULL;

	if (tty == NULL) {
		pr_err("mtty open input tty is NULL!\n");
		return -ENOMEM;
	}
	driver = tty->driver;
	mtty = (struct mtty_device *)driver->driver_state;

	if (mtty == NULL) {
		pr_err("mtty open input mtty NULL!\n");
		return -ENOMEM;
	}

	mtty->tty = tty;
	tty->driver_data = (void *)mtty;

	mutex_lock(&mtty->stat_mutex);
	mtty->state = MTTY_STATE_OPEN;
	mutex_unlock(&mtty->stat_mutex);
	que_task = 0;
	que_sche = 0;
	sitm_ini();
	pr_info("mtty_open device success!\n");

	return 0;
}

static void mtty_close(struct tty_struct *tty, struct file *filp)
{
	struct mtty_device *mtty = NULL;

	if (tty == NULL) {
		pr_err("mtty close input tty is NULL!\n");
		return;
	}
	mtty = (struct mtty_device *) tty->driver_data;
	if (mtty == NULL) {
		pr_err("mtty close s tty is NULL!\n");
		return;
	}

	mutex_lock(&mtty->stat_mutex);
	if (mtty->state == MTTY_STATE_CLOSE) {
		mutex_unlock(&mtty->stat_mutex);
		pr_info("mtty_close device already closed !\n");
		return;
	}
	mtty->state = MTTY_STATE_CLOSE;
	mutex_unlock(&mtty->stat_mutex);
	sitm_cleanup();
	pr_info("mtty_close device success !\n");
}


static  int sdio_data_transmit(uint8_t *data, size_t count)
{
	int ret;

	if (*data == DATA_TYPE_SCO) {
		char *buf = kmalloc(count, GFP_KERNEL);

		memcpy(buf, data, count);
		ret = sdiom_pt_write(buf, count, 0, 1);
		if (ret != 0) {
			pr_err("mtty write PT data to sdiom fail Error number=%d\n",
						ret);
		}
	} else {
		ret = sdiom_dt_write_bt(SDIOM_WR_DIRECT_MOD_ADDR, (void *)data,
								count);
		if (ret != 0) {
			pr_err("mtty write DT cmd to sdiom fail Error number=%d\n",
						ret);
		}
	}

	return ret == 0 ? count : -EBUSY;
}

static int mtty_write_plus(struct tty_struct *tty,
	      const unsigned char *buf, int count)
{
	if (unlikely(marlin_get_download_status() != true))
		return -EIO;
	return sitm_write(buf, count, sdio_data_transmit);
}

#if 0
static int mtty_write(struct tty_struct *tty,
	      const unsigned char *buf, int count)
{
	int port_index = 0;
	int ret = 0;

	if (unlikely(marlin_get_download_status() != true))
		return -EIO;

	port_index = tty->index;

	if (port_index == 1) {
		unsigned char *sdio_buf = kmalloc(count, GFP_KERNEL);

		if (sdio_buf == NULL)
			return -ENOMEM;

		memcpy(sdio_buf, buf, count);
		if (count <= 4)
			pr_info("mtty packer mode write count=%d,"hexformat"\n",
				count,
				hi4bit(sdio_buf[0]), lo4bit(sdio_buf[0]),
				hi4bit(sdio_buf[1]), lo4bit(sdio_buf[1]),
				hi4bit(sdio_buf[2]), lo4bit(sdio_buf[2]),
				hi4bit(sdio_buf[3]), lo4bit(sdio_buf[3]));

		else
			pr_info("mtty packer mode write count=%d,"hexformat"..."
				hexformat"\n", count,
				hi4bit(sdio_buf[0]), lo4bit(sdio_buf[0]),
				hi4bit(sdio_buf[1]), lo4bit(sdio_buf[1]),
				hi4bit(sdio_buf[2]), lo4bit(sdio_buf[2]),
				hi4bit(sdio_buf[3]), lo4bit(sdio_buf[3]),
				hi4bit(sdio_buf[count-4]),
				lo4bit(sdio_buf[count-4]),
				hi4bit(sdio_buf[count-3]),
				lo4bit(sdio_buf[count-3]),
				hi4bit(sdio_buf[count-2]),
				lo4bit(sdio_buf[count-2]),
				hi4bit(sdio_buf[count-1]),
				lo4bit(sdio_buf[count-1]));
		/* packer type 0, subtype 0 */
		ret = sdiom_pt_write(sdio_buf, count, 0, 1);
		if (ret != 0) {
			pr_err("mtty write PT data to sdiom fail Error number=%d\n",
						ret);
			return -EBUSY;
		}

		return count;
	} else if (port_index == 0) {
		#if 0
		if (count <= 4)
			pr_info("mtty direct mode write count=%d,"hexformat"\n",
				count,
				hi4bit(buf[0]), lo4bit(buf[0]),
				hi4bit(buf[1]), lo4bit(buf[1]),
				hi4bit(buf[2]), lo4bit(buf[2]),
				hi4bit(buf[3]), lo4bit(buf[3]));
		else
			pr_info("mtty direct mode write count=%d,"hexformat"..."
				hexformat"\n", count,
				hi4bit(buf[0]), lo4bit(buf[0]),
				hi4bit(buf[1]), lo4bit(buf[1]),
				hi4bit(buf[2]), lo4bit(buf[2]),
				hi4bit(buf[3]), lo4bit(buf[3]),
				hi4bit(buf[count-4]), lo4bit(buf[count-4]),
				hi4bit(buf[count-3]), lo4bit(buf[count-3]),
				hi4bit(buf[count-2]), lo4bit(buf[count-2]),
				hi4bit(buf[count-1]), lo4bit(buf[count-1]));
		#endif
		ret = sdiom_dt_write_bt(SDIOM_WR_DIRECT_MOD_ADDR, (void *)buf,
								count);
		if (ret != 0) {
			pr_err("mtty write DT cmd to sdiom fail Error number=%d\n",
						ret);
			return -EBUSY;
		}
		return count;
	}

	 return count;

}
#endif

static void mtty_flush_chars(struct tty_struct *tty)
{
}

static int mtty_write_room(struct tty_struct *tty)
{
	return INT_MAX;
}

static const struct tty_operations mtty_ops = {
	.open  = mtty_open,
	.close = mtty_close,
	.write = mtty_write_plus,
	.flush_chars = mtty_flush_chars,
	.write_room  = mtty_write_room,
};

static struct tty_port *mtty_port_init(void)
{
	struct tty_port *port = NULL;

	port = kzalloc(sizeof(struct tty_port), GFP_KERNEL);
	if (port == NULL)
		return NULL;

	tty_port_init(port);

	return port;
}

static int mtty_tty_driver_init(struct mtty_device *device)
{
	struct tty_driver *driver;
	int ret = 0;

	device->port0 = mtty_port_init();
	if (!device->port0)
		return -ENOMEM;

	device->port1 = mtty_port_init();
	if (!device->port1)
		return -ENOMEM;

	driver = alloc_tty_driver(MTTY_DEV_MAX_NR * 2);
	if (!driver)
		return -ENOMEM;

	/*
	 * Initialize the tty_driver structure
	 * Entries in mtty_driver that are NOT initialized:
	 * proc_entry, set_termios, flush_buffer, set_ldisc, write_proc
	 */
	driver->owner = THIS_MODULE;
	driver->driver_name = device->pdata->name;
	driver->name = device->pdata->name;
	driver->major = 0;
	driver->type = TTY_DRIVER_TYPE_SYSTEM;
	driver->subtype = SYSTEM_TYPE_TTY;
	driver->init_termios = tty_std_termios;
	driver->driver_state = (void *)device;
	device->driver = driver;
	 /* initialize the tty driver */
	tty_set_operations(driver, &mtty_ops);
	tty_port_link_device(device->port0, driver, 0);
	tty_port_link_device(device->port1, driver, 1);
	ret = tty_register_driver(driver);
	if (ret) {
		put_tty_driver(driver);
		tty_port_destroy(device->port0);
		tty_port_destroy(device->port1);
		return ret;
	}
	return ret;
}

static void mtty_tty_driver_exit(struct mtty_device *device)
{
	struct tty_driver *driver = device->driver;

	tty_unregister_driver(driver);
	put_tty_driver(driver);
	tty_port_destroy(device->port0);
	tty_port_destroy(device->port1);
}

static int mtty_parse_dt(struct mtty_init_data **init, struct device *dev)
{
#ifdef CONFIG_OF
	struct device_node *np = dev->of_node;
	struct mtty_init_data *pdata = NULL;
	int ret;

	pdata = kzalloc(sizeof(struct mtty_init_data), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	ret = of_property_read_string(np,
			"sprd,name",
			(const char **)&pdata->name);
	if (ret)
		goto error;
	*init = pdata;

	return 0;
error:
	kfree(pdata);
	*init = NULL;
	return ret;
#else
	return -ENODEV;
#endif
}

static inline void mtty_destroy_pdata(struct mtty_init_data **init)
{
#ifdef CONFIG_OF
	struct mtty_init_data *pdata = *init;

	kfree(pdata);

	*init = NULL;
#else
	return;
#endif
}


static int  mtty_probe(struct platform_device *pdev)
{
	struct mtty_init_data *pdata = (struct mtty_init_data *)
					pdev->dev.platform_data;
	struct mtty_device *mtty;
	int rval = 0;

	if (pdev->dev.of_node && !pdata) {
		rval = mtty_parse_dt(&pdata, &pdev->dev);
		if (rval) {
			pr_err("failed to parse mtty device tree, ret=%d\n",
				rval);
			return rval;
		}
	}

	mtty = kzalloc(sizeof(struct mtty_device), GFP_KERNEL);
	if (mtty == NULL) {
		mtty_destroy_pdata(&pdata);
		pr_err("mtty Failed to allocate device!\n");
		return -ENOMEM;
	}

	mtty->pdata = pdata;
	rval = mtty_tty_driver_init(mtty);
	if (rval) {
		mtty_tty_driver_exit(mtty);
		kfree(mtty->port0);
		kfree(mtty->port1);
		kfree(mtty);
		mtty_destroy_pdata(&pdata);
		pr_err("regitster notifier failed (%d)\n", rval);
		return rval;
	}

	pr_info("mtty_probe init device addr: 0x%p\n", mtty);
	platform_set_drvdata(pdev, mtty);

	mutex_init(&mtty->stat_mutex);
	mutex_init(&mtty->rw_mutex);
	INIT_LIST_HEAD(&mtty->rx_head);
	mtty->bt_rx_workqueue =
		create_singlethread_workqueue("SPRDBT_RX_QUEUE");
	if (!mtty->bt_rx_workqueue) {
		pr_err("%s SPRDBT_RX_QUEUE create failed", __func__);
		return -ENOMEM;
	}
	INIT_WORK(&mtty->bt_rx_work, mtty_rx_work_queue);

	mtty_dev = mtty;

	rfkill_bluetooth_init(pdev);
	bluesleep_init();
	sdiom_register_pt_rx_process(0, 0, mtty_rx_cb);
	sdiom_register_pt_tx_release(0, 1, mtty_tx_cb);
	/* rx packer type:0, subtype 0 */
	sdiom_register_pt_tx_release(0, 0, mtty_tx_cb);
	/* tx packer type:0, subtype 0 */
#ifdef MTTY_TEST
	setup_timer(&test_timer, timer_cb, 0);
	test_init();
#endif
	return 0;
}

static int  mtty_remove(struct platform_device *pdev)
{
	struct mtty_device *mtty = platform_get_drvdata(pdev);

	mtty_tty_driver_exit(mtty);
	kfree(mtty->port0);
	kfree(mtty->port1);
	mtty_destroy_pdata(&mtty->pdata);
	flush_workqueue(mtty->bt_rx_workqueue);
	destroy_workqueue(mtty->bt_rx_workqueue);
	kfree(mtty);
	platform_set_drvdata(pdev, NULL);
	bluesleep_exit();

	return 0;
}

static const struct of_device_id mtty_match_table[] = {
	{ .compatible = "sprd,mtty", },
	{ },
};

static struct platform_driver mtty_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "mtty",
		.of_match_table = mtty_match_table,
	},
	.probe = mtty_probe,
	.remove = mtty_remove,
};

module_platform_driver(mtty_driver);

MODULE_AUTHOR("Spreadtrum BSP");
MODULE_DESCRIPTION("SPRD marlin tty driver");
MODULE_LICENSE("GPL");

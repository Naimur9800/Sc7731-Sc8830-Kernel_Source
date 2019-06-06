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

#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/resource.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/wait.h>

#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/jiffies.h>
#include <linux/sipc.h>
#include <linux/suspend.h>
#include <linux/kthread.h>

#ifdef CONFIG_SPRD_MAILBOX
#include "../include/sprd_mailbox.h"
#endif

#include "../include/sipc_priv.h"
#include "../sipc/sipc_debugfs.h"

#define BIT_0	0x1
#define BIT_1	0x2
#define BIT_2	0x4
#define BIT_3	0x8
#define BIT_4	0x10
#define BIT_5	0x20

/* reg offset define */
#define MBOX_ID			0x00
#define MBOX_MSG_L		0x04
#define MBOX_MSG_H		0x08
#define MBOX_TRI		0x0c
#define MBOX_FIFO_RST		0x10
#define MBOX_FIFO_STS		0x14
#define MBOX_IRQ_STS		0x18
#define MBOX_IRQ_MSK		0x1c
#define MBOX_LOCK		0x20
#define MBOX_FIFO_DEPTH		0x24
#define MBOX_VERSION		0x28

/* reg mask define */
#define MBOX_IRQ_CLR_BIT			(BIT_0)
#define MBOX_FIFO_NOT_EMPTY_BIT		(BIT_0)
#define MBOX_FIFO_EMPTY_BIT			(BIT_1)
#define MBOX_FIFO_FULL_BIT			(BIT_2)
#define MBOX_FIFO_OVERFLOW_BIT		(BIT_3)
#define MBOX_FIFO_CHANGE_BIT		(BIT_4)
#define MBOX_V1_FIFO_BLOCK_BIT		(BIT_5)

#define MBOX_INBOX_FIFO_BLOCK_BIT		(BIT_0)
#define MBOX_INBOX_FIFO_OVERFLOW_BIT	(BIT_1)
#define MBOX_INBOX_FIFO_CHANGE_BIT		(BIT_2)

#define MBOX_FIFO_FULL_STS_MASK			(0x4)
#define MBOX_FIFO_BLOCK_OVERFLOW_MASK	(0xFFFF)
#define MBOX_FIFO_DELIVER_OVERLOW_MASK  (0xFFFF00)
#define MBOX_FIFO_DELIVER_MASK          (0xFF0000)
#define MBOX_FIFO_BLOCK_MASK            (0xFF)

#define MBOX_FIFO_DELIVER_BIT       (16)
#define MBOX_FIFO_OUTBOX_RECV_BIT	(24)

#define MBOX_UNLOCK_KEY 0x5a5a5a5a

/* enable outbox not empty */
#define MBOX_OUTBOX_IRQ_CONFIG \
	(~(MBOX_FIFO_NOT_EMPTY_BIT))
	/* (~(MBOX_FIFO_CHANGE_BIT | MBOX_FIFO_NOT_EMPTY_BIT))  */

/* only enable outbox delivered now */
#define MBOX_OUTBOX_IRQ_CONFIG_FOR_IRQ_ONLY \
	(~MBOX_FIFO_CHANGE_BIT)

/* enable inbox block and overflow */
#define MBOX_INBOX_BLOCK_IRQ \
	(~(MBOX_INBOX_FIFO_OVERFLOW_BIT | MBOX_INBOX_FIFO_BLOCK_BIT))

/* if mailbox version is v1, the bits of inbox and outbox have the same means */
#define MBOX_INBOX_BLOCK_IRQ_V1 \
	(~(MBOX_FIFO_OVERFLOW_BIT | MBOX_V1_FIFO_BLOCK_BIT))

/* enable inbox send succ */
#define MBOX_INBOX_DELIVERED_IRQ_V2 \
	(~MBOX_INBOX_FIFO_CHANGE_BIT)
#define MBOX_INBOX_DELIVERED_IRQ_V1 \
		(~MBOX_FIFO_CHANGE_BIT)

#define MBOX_INBOX_DELIVERED_IRQ \
	((mbox_cfg.version > 1) ? \
	MBOX_INBOX_DELIVERED_IRQ_V2 : MBOX_INBOX_DELIVERED_IRQ_V1)

#define MBOX_IRQ_DISABLE_ALLIRQ (0xFFFFFFFF)

#define MBOX_FIFO_SIZE		256
#define MBOX_MAX_CORE_CNT	8
#define MBOX_MAX_CORE_MASK	7
#define MAX_SMSG_BAK		64

/*
 * mbox configs define: now we had two hardware version V1 and
 * V2, the below configs are diffrence between with two version
 */
#define MBOX_ENABLE_REG_OFFSET 0x4
#define MBOX_ENABLE_REG_MASK   0x200000
#define MBOX_ENABLE_REG_INFO_NR 2

/* mbox version 1 configs */
#define MBOX_V1_INBOX_FIFO_SIZE  0x8
#define MBOX_V1_OUTBOX_FIFO_SIZE 0x8

#define MBOX_V1_READ_PT_BIT  28
#define MBOX_V1_READ_PT_MASK 0x7

#define MBOX_V1_WRITE_PT_BIT 24
#define MBOX_V1_WRITE_PT_MASK 0x7

#define MBOX_V1_INBOX_CORE_SIZE 0x100
#define MBOX_V1_OUTBOX_CORE_SIZE 0x100

#define MBOX_V1_INBOX_IRQ_MASK   MBOX_INBOX_BLOCK_IRQ_V1
#define MBOX_V1_OUTBOX_IRQ_MASK  MBOX_OUTBOX_IRQ_CONFIG

/* mbox version 2 configs */
#define MBOX_V2_INBOX_FIFO_SIZE  0x1
#define MBOX_V2_OUTBOX_FIFO_SIZE 0x40

#define MBOX_V2_READ_PT_BIT  24
#define MBOX_V2_READ_PT_MASK 0xff

#define MBOX_V2_WRITE_PT_BIT 16
#define MBOX_V2_WRITE_PT_MASK 0xff

#define MBOX_V2_INBOX_CORE_SIZE 0x1000
#define MBOX_V2_OUTBOX_CORE_SIZE 0x1000

#define MBOX_V2_INBOX_IRQ_MASK   MBOX_INBOX_BLOCK_IRQ
#define MBOX_V2_OUTBOX_IRQ_MASK  MBOX_OUTBOX_IRQ_CONFIG

#define SEND_FIFO_LEN 64

/* mbox local feature define */
/* redefine debug function, only can open it in mbox bringup phase */
/*#define MBOX_REDEFINE_DEBUG_FUNCTION*/

/* remove pr_debug, because Mbox was used so frequently, remove
 * debug info can improve the system performance
 */
#define MBOX_REMOVE_PR_DEBUG

/* remove the same msg in fifo, It also can improve the system performance */
#define MBOX_REMOVE_THE_SAME_MSG

/* mbox test feature, will cteate a device "dev/sprd_mbox",
 * echo '1' to start,  echo '1' to stop, but we only can
 * open it in mbox bringup phase and don't care dsp state
 */

/* #define SPRD_MAILBOX_TEST */

#ifdef MBOX_REMOVE_PR_DEBUG
#undef pr_debug
#define pr_debug(fmt, ...)
#endif

/* mbox local staruct global var define */
static struct regmap *mailbox_gpr;
static unsigned long sprd_inbox_base;
static unsigned long sprd_outbox_base;

#define REGS_SEND_MBOX_BASE  (sprd_inbox_base)
#define REGS_RECV_MBOX_BASE  (sprd_outbox_base)
#define REGS_SEND_MBOX_RANGE (mbox_cfg.inbox_range)
#define REGS_RECV_MBOX_RANGE  (mbox_cfg.outbox_range)

#ifdef CONFIG_SPRD_MAILBOX_SENSOR
#define REGS_RECV_MBOX_SENSOR_BASE (sprd_outbox_base\
	+ (mbox_cfg.outbox_range * mbox_cfg.sensor_core))
#endif

#define MBOX_GET_FIFO_RD_PTR(val) ((val >> mbox_cfg.rd_bit) & mbox_cfg.rd_mask)
#define MBOX_GET_FIFO_WR_PTR(val) ((val >> mbox_cfg.wr_bit) & mbox_cfg.wr_mask)

#define MBOX_NR  (mbox_cfg.core_cnt)

struct mbox_cfg_tag {
	u32 inbox_irq;
	u32 inbox_base;
	u32 inbox_range;
	u32 inbox_fifo_size;
	u32 inbox_irq_mask;

	u32 outbox_irq;
#ifdef CONFIG_SPRD_MAILBOX_SENSOR
	u32 outbox_sensor_irq;
	u32 sensor_core;
#endif

	u32 outbox_base;
	u32 outbox_range;
	u32 outbox_fifo_size;
	u32 outbox_irq_mask;

	u32 rd_bit;
	u32 rd_mask;
	u32 wr_bit;
	u32 wr_mask;

	u32 enable_reg;
	u32 mask_bit;

	u32 core_cnt;
	u32 version;
};

struct mbox_chn_tag {
#ifdef CONFIG_SPRD_MAILBOX_FIFO
	MBOX_FUNCALL mbox_smsg_handler;
#else
	irq_handler_t mbox_smsg_handler;
#endif
	unsigned long max_irq_proc_time;
	unsigned long max_recv_flag_cnt;
	void *mbox_priv_data;
};

#ifdef CONFIG_SPRD_MAILBOX_FIFO
struct  mbox_fifo_data_tag {
	u8 core_id;
	u64 msg;
};

static struct  mbox_fifo_data_tag mbox_fifo[MBOX_FIFO_SIZE];
static struct  mbox_fifo_data_tag mbox_fifo_bak[MAX_SMSG_BAK];

static u8 mbox_read_all_fifo_msg(void);
static void mbox_raw_recv(struct  mbox_fifo_data_tag *fifo);
static void mbox_cfg_printk(void);

#ifdef CONFIG_SPRD_MAILBOX_SENSOR
static u8 mbox_sensor_read_all_fifo_msg(void);
static void mbox_sensor_raw_recv(struct  mbox_fifo_data_tag *fifo);
#endif

#endif

#ifdef CONFIG_SPRD_MAILBOX_FIFO
static int mbox_fifo_bak_len;
static u8 g_one_time_recv_cnt;
#endif
static u8 g_mbox_inited;
static u8 g_inbox_send;
static u32 g_inbox_irq_mask;

static unsigned long max_total_irq_proc_time;
static unsigned long max_total_irq_cnt;

static int g_restore_cnt;
static int g_inbox_block_cnt;
static int g_outbox_full_cnt;
static int g_skip_msg;
static unsigned int g_recv_cnt[MBOX_MAX_CORE_CNT];
static unsigned int g_send_cnt[MBOX_MAX_CORE_CNT];

static spinlock_t mbox_lock;

static struct mbox_chn_tag mbox_chns[MBOX_MAX_CORE_CNT];
static struct mbox_cfg_tag mbox_cfg;

static u64 g_send_fifo[MBOX_MAX_CORE_CNT][SEND_FIFO_LEN];
static u16 g_inptr[MBOX_MAX_CORE_CNT];
static u16 g_outptr[MBOX_MAX_CORE_CNT];

static struct task_struct *g_send_thread;
static wait_queue_head_t g_send_wait;

static u8 mbox_get_send_fifo_mask(u8 send_bit);
static void mbox_put1msg(u8 dst, u64 msg);
static int mbox_send_thread(void *pdata);
static void mbox_start_fifo_send(void);

#ifdef SPRD_MAILBOX_TEST
#include <linux/miscdevice.h>
#include <linux/delay.h>

#define MAX_SEND_CNT     64
#define MAX_SLEEP_TIME   1000 /* ms */
#define MIN_SLEEP_TIME   100 /* ms */

static unsigned int g_ap_self_rcv_cnt;

static void mbox_test_start(void);
static void mbox_test_stop(void);

#define MBOX_IRQ_ONLY    0x00010000 /* bit 16 */

enum {
	NORMAL_FIFO_MODE = 0,
	IRQ_ONLY_MODE,
};

static const u8 irq_mode[MBOX_MAX_CORE_CNT] = {
	NORMAL_FIFO_MODE, /* CORE 0, AP */
	NORMAL_FIFO_MODE, /* CORE 1, CM4 */
	NORMAL_FIFO_MODE, /* CORE 2, CR5 */
	NORMAL_FIFO_MODE, /* CORE 3, TGSP */
	NORMAL_FIFO_MODE, /* CORE 4, LDSP */
	NORMAL_FIFO_MODE, /* CORE 5, AGDSP */
	NORMAL_FIFO_MODE, /* CORE 6,  WCN */
	NORMAL_FIFO_MODE, /* CORE 7,  GPS */
};

static irqreturn_t mbox_ap_self_handle(void *ptr, void *private)
{
	g_ap_self_rcv_cnt++;

	return IRQ_HANDLED;
}

static void mbox_init_all_outboxmode(void)
{
	int i;
	unsigned long outbox;

	for (i = 0; i < mbox_cfg.core_cnt; i++) {
		outbox = REGS_RECV_MBOX_BASE + i * REGS_RECV_MBOX_RANGE;
		__raw_writel(((irq_mode[i] == IRQ_ONLY_MODE) ?
			      MBOX_IRQ_ONLY : 0),
			     (void __iomem *)(outbox + MBOX_FIFO_RST));
	}
}

static ssize_t sprd_mbox_write(struct file *file, const char __user *buf,
			       size_t count, loff_t *ppos)
{
	pr_info("mbox: %s\n", __func__);

	if (buf[0] == '0')
		mbox_test_stop();
	else
		mbox_test_start();

	return count;
}

static int sprd_mbox_open(struct inode *inode, struct file *filp)
{
	pr_info("mbox: %s\n", __func__);
	return 0;
}

static int sprd_mbox_release(struct inode *inode, struct file *filp)
{
	pr_info("mbox: %s\n", __func__);
	return 0;
}

static const struct file_operations sprd_mbox_fops = {
	.owner = THIS_MODULE,
	.write = sprd_mbox_write,
	.open = sprd_mbox_open,
	.release = sprd_mbox_release,
};

static struct miscdevice sprd_mbox_dev = {
	.minor   = MISC_DYNAMIC_MINOR,
	.name   = "sprd_mbox",
	.fops   = &sprd_mbox_fops,
};

struct task_struct *test_thread[MBOX_MAX_CORE_CNT];
static u8 g_b_test_start;

static void mbox_test_init(void)
{
	int ret;

	pr_info("mbox:%s!\n", __func__);

	mbox_register_irq_handle(0, mbox_ap_self_handle, NULL);

	mbox_init_all_outboxmode();

	ret = misc_register(&sprd_mbox_dev);
	if (ret)
		pr_err("mbox:register mbox dev ret = (%d)\n", ret);
}

static void mbox_test_sent(u8 core_id, u64 msg)
{
	mbox_raw_sent(core_id, msg);
}

static int mbox_test_thread(void *data)
{
	int i, cnt, time, dst;
	u64 msg;

	dst = (data - (void *)test_thread) / sizeof(struct task_struct *);

	pr_info("mbox: %s, %s\n", __func__, current->comm);

	while (!kthread_should_stop()) {
		cnt = jiffies % MAX_SEND_CNT + 1;
		time = jiffies % MAX_SLEEP_TIME + 1;
		pr_debug("mbox: %s, dst = %d, cnt = %d, time =%d!\n",
			 current->comm, dst, cnt, time);

		for (i = 0; i < cnt; i++) {
			if (g_b_test_start == 0)
				break;

			msg = g_send_cnt[dst];
			mbox_test_sent(dst, msg);

			msleep(time + MIN_SLEEP_TIME);
		}
		msleep(MIN_SLEEP_TIME);
	}

	return 0;
}

static void mbox_test_start(void)
{
	int i = 0;

	pr_info("mbox: %s\n", __func__);
	if (g_b_test_start)
		return;

	g_b_test_start = 1;

	for (i = 0; i < mbox_cfg.core_cnt; i++) {
		test_thread[i] = kthread_create(mbox_test_thread,
					&test_thread[i],
					"mbox-test-%d", i);
		if (IS_ERR(test_thread[i])) {
			pr_err("mbox-test-%d", i);
			continue;
		}
		msleep(1000);
		wake_up_process(test_thread[i]);
	}
}

static void mbox_test_stop(void)
{
	int i = 0;

	pr_info("mbox: %s\n", __func__);

	if (!g_b_test_start)
		return;

	g_b_test_start = 0;

	for (i = 0; i < mbox_cfg.core_cnt; i++) {
		/* stop sbuf thread if it's created successfully
		 * and still alive
		 */
		if (test_thread[i] &&
		    !IS_ERR_OR_NULL(test_thread[i])) {
			pr_info("mbox: kthread_stop %s\n",
				test_thread[i]->comm);

			kthread_stop(test_thread[i]);
			test_thread[i] = NULL;
		}
	}
}
#endif

static void mbox_restore_all_inbox(void)
{
	int i;
	unsigned long inbox;
	u32 status;

	for (i = 0; i < mbox_cfg.core_cnt; i++) {
		inbox = REGS_SEND_MBOX_BASE + i * REGS_SEND_MBOX_RANGE;
		status = __raw_readl((void __iomem *)(inbox + MBOX_FIFO_STS));

		/* find the block inbox*/
		if (status & MBOX_FIFO_BLOCK_OVERFLOW_MASK) {
			/* reset  inbox status */
			__raw_writel(
				(status & MBOX_FIFO_BLOCK_OVERFLOW_MASK),
				(void __iomem *)(inbox + MBOX_FIFO_RST));
			g_restore_cnt++;
			pr_info("mbox:%s, inbox %d, status = 0x%x, resCnt = %d\n",
				__func__, i, status, g_restore_cnt);
		}
	}
}

static irqreturn_t  mbox_src_irqhandle(int irq_num, void *dev)
{
	u32 fifo_sts;
	u32 block, succ;

	/*read inbox status */
	fifo_sts = __raw_readl(
		(void __iomem *)(REGS_SEND_MBOX_BASE + MBOX_FIFO_STS));
	pr_debug("mbox:%s,fifo_sts=%x\n", __func__, fifo_sts);

	/* clear inbox deliver and overlow status*/
	__raw_writel(fifo_sts & MBOX_FIFO_DELIVER_OVERLOW_MASK,
		     (void __iomem *)
		     (REGS_SEND_MBOX_BASE + MBOX_FIFO_RST));

	block = (fifo_sts & MBOX_FIFO_BLOCK_MASK);
	succ = (fifo_sts & MBOX_FIFO_DELIVER_MASK) >>
		MBOX_FIFO_DELIVER_BIT;

	/* if It's a send irq and have some send succ bits,
	 * and the corresponding send fifo is not empty
	 */
	if (g_inbox_irq_mask == MBOX_INBOX_DELIVERED_IRQ) {
		g_inbox_send = mbox_get_send_fifo_mask(succ);
		pr_debug("mbox: succ = 0x%x, 0x%x!\n", succ, g_inbox_send);

		if (g_inbox_send)
			mbox_start_fifo_send();
	}

	/* if have some bits block, we enable delivered irq,
	 * other wise we enable block irq
	 */
	if (block) {
		g_inbox_irq_mask = MBOX_INBOX_DELIVERED_IRQ;
		pr_debug("mbox: block irq succ = 0x%x!\n", succ);
	} else {
		g_inbox_irq_mask = mbox_cfg.inbox_irq_mask;
		pr_debug("mbox: nonblock irq succ = 0x%x!\n", succ);
	}
	__raw_writel(g_inbox_irq_mask,
		     (void __iomem *)
		     (REGS_SEND_MBOX_BASE + MBOX_IRQ_MSK));

	/* clear irq */
	__raw_writel(MBOX_IRQ_CLR_BIT,
		     (void __iomem *)(REGS_SEND_MBOX_BASE + MBOX_IRQ_STS));

	return IRQ_HANDLED;
}

static irqreturn_t mbox_recv_irqhandle(int irq_num, void *dev)
{
	u32 fifo_sts, irq_mask;
	int i = 0;
	void *priv_data;
#ifdef CONFIG_SPRD_MAILBOX_FIFO
	u8 target_id;
	u8 fifo_len;
#else
	struct smsg_ipc *ipc = NULL;
#endif
	unsigned long jiff, jiff_total;

	jiff_total = jiffies;

	/* get fifo status */
	fifo_sts = __raw_readl(
		(void __iomem *)(REGS_RECV_MBOX_BASE + MBOX_FIFO_STS));
	irq_mask = fifo_sts & 0x0000ff00;

	pr_debug("mbox:%s,fifo_sts=0x%08x\n", __func__, fifo_sts);

#ifdef CONFIG_SPRD_MAILBOX_FIFO
	fifo_len = mbox_read_all_fifo_msg();

	/* clear irq mask & irq after read all msg, if clear before read,
	 * it will produce a irq again
	 */
	__raw_writel(irq_mask | MBOX_IRQ_CLR_BIT,
		     (void __iomem *)(REGS_RECV_MBOX_BASE + MBOX_IRQ_STS));

	/* print the id of the fist mail to know who wake up ap */
	if (sipc_get_wakeup_flag())
		pr_info("mbox: wake up by id = %d\n",
			mbox_fifo[0].core_id);

	for (i = 0; i < fifo_len; i++) {
		target_id = mbox_fifo[i].core_id;

		if (target_id >= MBOX_NR) {
			pr_err("mbox:ERR on line %d, target_id >= MBOX_NR\n",
			       __LINE__);
			return IRQ_NONE;
		}

		if (mbox_chns[target_id].mbox_smsg_handler) {
			pr_debug("mbox: msg handle,index =%d, id = %d\n",
				 i, target_id);
			priv_data = mbox_chns[target_id].mbox_priv_data;
			/* get the jiffs before irq proc */
			jiff = jiffies;
			mbox_chns[target_id].mbox_smsg_handler(
				&mbox_fifo[i].msg,
				priv_data);
			/* update the max jiff time */
			jiff = jiffies - jiff;
			if (jiff > mbox_chns[target_id].max_irq_proc_time)
				mbox_chns[target_id].max_irq_proc_time = jiff;
		} else if (mbox_fifo_bak_len < MAX_SMSG_BAK) {
			pr_info("mbox: msg bak hear,index =%d, id = %d\n",
				i, target_id);
			memcpy(&mbox_fifo_bak[mbox_fifo_bak_len],
			       &mbox_fifo[i],
			       sizeof(struct  mbox_fifo_data_tag));
			mbox_fifo_bak_len++;
		} else {
			pr_err("mbox: msg drop hear,index =%d, id = %d\n",
			       i, target_id);
		}
	}

	if (sipc_get_wakeup_flag())
		sipc_clear_wakeup_flag();

#else
	/* clear irq mask & irq */
	__raw_writel(irq_mask | MBOX_IRQ_CLR_BIT,
		     (void __iomem *)(REGS_RECV_MBOX_BASE + MBOX_IRQ_STS));

	/* if many core send a mail to this core in the same time,
	 * it may lost some interrupts, so in hear used smsg wrptr
	 * to judge
	 */
	for (i = 0; i < MBOX_NR; i++) {
		if (mbox_chns[i].mbox_priv_data) {
			ipc = (struct smsg_ipc *)(mbox_chns[i].mbox_priv_data);
			if (readl((void __iomem *)ipc->rxbuf_wrptr)
				!= readl((void __iomem *)ipc->rxbuf_rdptr)) {
				pr_debug("mbox: msg handle,target_id = %d\n",
					 i);
				priv_data = mbox_chns[i].mbox_priv_data;
				mbox_chns[i].mbox_smsg_handler(irq_num,
					priv_data);
			}
		}
	}

#endif

	/* update the max total irq time */
	jiff_total = jiffies - jiff_total;
	if (jiff_total > max_total_irq_proc_time)
		max_total_irq_proc_time = jiff_total;

	max_total_irq_cnt++;

	return IRQ_HANDLED;
}

#ifdef CONFIG_SPRD_MAILBOX_SENSOR
static irqreturn_t mbox_sensor_recv_irqhandle(int irq_num, void *dev)
{
	u32 fifo_sts, irq_mask;
	int i = 0;
	void *priv_data;
	u8 target_id;
	u8 fifo_len;
	unsigned long jiff, jiff_total;

	jiff_total = jiffies;

	/* get fifo status */
	fifo_sts = __raw_readl(
		(void __iomem *)(REGS_RECV_MBOX_SENSOR_BASE + MBOX_FIFO_STS));
	irq_mask = fifo_sts & 0x0000ff00;

	pr_debug("mbox:%s,fifo_sts=0x%08x\n", __func__, fifo_sts);

	fifo_len = mbox_sensor_read_all_fifo_msg();

	/* clear irq mask & irq after read all msg, if clear before read,
	 * it will produce a irq again
	 */
	__raw_writel(irq_mask | MBOX_IRQ_CLR_BIT, (void __iomem *)
			(REGS_RECV_MBOX_SENSOR_BASE + MBOX_IRQ_STS));

	for (i = 0; i < fifo_len; i++) {
		target_id = mbox_fifo[i].core_id;

		if (target_id >= MBOX_NR) {
			pr_err("mbox:ERR on line %d, target_id >= MBOX_NR\n",
			       __LINE__);
			return IRQ_NONE;
		}

		if (mbox_chns[target_id].mbox_smsg_handler) {
			pr_debug("mbox: msg handle,index =%d, id = %d\n",
				 i, target_id);
			priv_data = mbox_chns[target_id].mbox_priv_data;
			/* get the jiffs before irq proc */
			jiff = jiffies;
			mbox_chns[target_id].mbox_smsg_handler(
				&mbox_fifo[i].msg,
				priv_data);
			/* update the max jiff time */
			jiff = jiffies - jiff;
			if (jiff > mbox_chns[target_id].max_irq_proc_time)
				mbox_chns[target_id].max_irq_proc_time = jiff;
		} else if (mbox_fifo_bak_len < MAX_SMSG_BAK) {
			pr_info("mbox: msg bak hear,index =%d, id = %d\n",
				i, target_id);
			memcpy(&mbox_fifo_bak[mbox_fifo_bak_len],
			       &mbox_fifo[i],
			       sizeof(struct  mbox_fifo_data_tag));
			mbox_fifo_bak_len++;
		} else {
			pr_err("mbox: msg drop hear,index =%d, id = %d\n",
			       i, target_id);
		}
	}

	/* update the max total irq time */
	jiff_total = jiffies - jiff_total;
	if (jiff_total > max_total_irq_proc_time)
		max_total_irq_proc_time = jiff_total;

	max_total_irq_cnt++;

	return IRQ_HANDLED;
}

static void mbox_sensor_raw_recv(struct  mbox_fifo_data_tag *fifo)
{
	u64 msg_l, msg_h;
	int target_id;

	msg_l = __raw_readl(
		(void __iomem *)(REGS_RECV_MBOX_SENSOR_BASE + MBOX_MSG_L));
	msg_h = __raw_readl(
		(void __iomem *)(REGS_RECV_MBOX_SENSOR_BASE + MBOX_MSG_H));
	target_id = __raw_readl(
		(void __iomem *)(REGS_RECV_MBOX_SENSOR_BASE + MBOX_ID));
	pr_debug("mbox:%s, id =%d, msg_l = 0x%x, msg_h = 0x%x\n",
		 __func__, target_id,
		 (unsigned int)msg_l, (unsigned int)msg_h);

	fifo->msg = (msg_h << 32) | msg_l;
	fifo->core_id = target_id & MBOX_MAX_CORE_MASK;
	g_recv_cnt[fifo->core_id]++;
	__raw_writel(0x1, (void __iomem *)
		(REGS_RECV_MBOX_SENSOR_BASE + MBOX_TRI));
}

static u8 mbox_sensor_read_all_fifo_msg(void)
{
	u32 fifo_sts;
	u8 fifo_depth;
	u8 rd, wt, cnt, i;
#ifdef MBOX_REMOVE_THE_SAME_MSG
	struct smsg *msg;
	u8 j, skip;
	struct  mbox_fifo_data_tag fifo_data;
#endif

	fifo_sts = __raw_readl(
		(void __iomem *)(REGS_RECV_MBOX_SENSOR_BASE + MBOX_FIFO_STS));
	wt = MBOX_GET_FIFO_WR_PTR(fifo_sts);
	rd = MBOX_GET_FIFO_RD_PTR(fifo_sts);

    /* if fifo is full or empty, when the read ptr == write ptr */
	if (rd == wt) {
		if (fifo_sts & MBOX_FIFO_FULL_STS_MASK) {
			g_outbox_full_cnt++;
			cnt = mbox_cfg.outbox_fifo_size;
		} else {
			cnt = 0;
		}
	} else {
		if (wt > rd)
			cnt = wt - rd;
		else
			cnt = mbox_cfg.outbox_fifo_size - rd + wt;
	}
	pr_debug("mbox: %s, rd = %d, wt = %d, cnt = %d\n",
		 __func__, rd, wt, cnt);

	fifo_depth = 0;
	for (i = 0; i < cnt; i++) {
#ifdef MBOX_REMOVE_THE_SAME_MSG
		mbox_sensor_raw_recv(&fifo_data);

		skip = 0;
		/* skip the same msg to reduce the irq proc time */
		for (j = 0; j < fifo_depth; j++) {
			if (fifo_data.core_id == mbox_fifo[j].core_id &&
			    fifo_data.msg == mbox_fifo[j].msg) {
				msg = (struct smsg *)&fifo_data.msg;
				/* only skip the event type msg */
				if (msg->type == SMSG_TYPE_EVENT) {
					skip = 1;
					g_skip_msg++;
					break;
				}
			}
		}

		if (skip)
			continue;

		/* copy to fifo */
		memcpy(&mbox_fifo[fifo_depth],
		       &fifo_data,
		       sizeof(struct  mbox_fifo_data_tag));
		fifo_depth++;
		pr_debug("mbox: %s, fifo_depth=%d\n", __func__, fifo_depth);
#else
		mbox_sensor_raw_recv(&mbox_fifo[i]);
		fifo_depth++;
#endif
	}

	/* the new mbox ip no need restall inbox */
	if (mbox_cfg.version <= 2) {
		/* if pre cnt + cur cnt >= mbox_cfg.outbox_fifo_size,
		 * may some inboxs have been blocked
		 */
		if (cnt + g_one_time_recv_cnt
			>= mbox_cfg.outbox_fifo_size)
			mbox_restore_all_inbox();
	}

	g_one_time_recv_cnt = cnt;

	return fifo_depth;
}
#endif

#ifdef CONFIG_SPRD_MAILBOX_FIFO
static void mailbox_process_bak_msg(void)
{
	int i;
	int cnt = 0;
	int target_id = 0;
	void *priv_data;

	for (i = 0; i < mbox_fifo_bak_len; i++) {
		target_id = mbox_fifo_bak[i].core_id;
		/* has been procced */
		if (target_id == MBOX_MAX_CORE_CNT) {
			cnt++;
			continue;
		}
		if (mbox_chns[target_id].mbox_smsg_handler) {
			pr_debug("mbox: bak msg pass to handler,index = %d, id = %d\n",
				 i, target_id);

			priv_data = mbox_chns[target_id].mbox_priv_data;
			mbox_chns[target_id].mbox_smsg_handler(
				&mbox_fifo_bak[i].msg,
				priv_data);
			/* set a mask indicate the bak msg is been procced*/
			mbox_fifo_bak[i].core_id = MBOX_MAX_CORE_CNT;
			cnt++;
		} else {
			pr_debug("mbox_smsg_handler is NULL,index = %d, id = %d\n",
				 i, target_id);
		}
	}

	/* reset  mbox_fifo_bak_len*/
	if (mbox_fifo_bak_len == cnt)
		mbox_fifo_bak_len = 0;
}

int mbox_register_irq_handle(u8 target_id,
			     MBOX_FUNCALL irq_handler,
			     void *priv_data)
#else
int mbox_register_irq_handle(u8 target_id,
			     irq_handler_t irq_handler,
			     void *priv_data)
#endif
{
	unsigned long flags;
#ifndef CONFIG_SPRD_MAILBOX_FIFO
	struct smsg_ipc *ipc;
#endif

	pr_debug("mbox:%s,target_id =%d\n", __func__, target_id);

	if (!g_mbox_inited) {
		pr_err("mbox:ERR on line %d!\n", __LINE__);
		return -EINVAL;
	}

	if (target_id >= MBOX_NR || mbox_chns[target_id].mbox_smsg_handler)
		return -EINVAL;

	spin_lock_irqsave(&mbox_lock, flags);

	mbox_chns[target_id].mbox_smsg_handler = irq_handler;
	mbox_chns[target_id].mbox_priv_data = priv_data;

	spin_unlock_irqrestore(&mbox_lock, flags);

#ifdef CONFIG_SPRD_MAILBOX_FIFO
	/* must do it, Ap may be have already revieved some msgs */
	mailbox_process_bak_msg();
#else
	ipc = (struct smsg_ipc *)(mbox_chns[target_id].mbox_priv_data);
	if (readl((void __iomem *)ipc->rxbuf_wrptr)
		!= readl((void __iomem *)ipc->rxbuf_rdptr)) {
		pr_debug("mbox:mbox msg handle,target_id = %d\n", target_id);
		priv_data = mbox_chns[target_id].mbox_priv_data;
		mbox_chns[target_id].mbox_smsg_handler(mbox_cfg.outbox_irq,
			priv_data);
	}
#endif

	return 0;
}
EXPORT_SYMBOL_GPL(mbox_register_irq_handle);

int mbox_unregister_irq_handle(u8 target_id)
{
	u32 reg_val;

	if (target_id >= MBOX_NR || !mbox_chns[target_id].mbox_smsg_handler)
		return -EINVAL;

	if (!g_mbox_inited) {
		pr_err("mbox:ERR on line %d!\n", __LINE__);
		return -EINVAL;
	}

	spin_lock(&mbox_lock);

	mbox_chns[target_id].mbox_smsg_handler = NULL;

	/*clean the the corresponding regist core  irq status*/
	reg_val = __raw_readl(
		(void __iomem *)(REGS_RECV_MBOX_BASE + MBOX_IRQ_STS));
	reg_val |= (0x1 << target_id) << 8;
	__raw_writel(reg_val,
		     (void __iomem *)(REGS_RECV_MBOX_BASE + MBOX_IRQ_STS));

	spin_unlock(&mbox_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(mbox_unregister_irq_handle);

#ifdef CONFIG_SPRD_MAILBOX_FIFO
static void mbox_raw_recv(struct  mbox_fifo_data_tag *fifo)
{
	u64 msg_l, msg_h;
	int target_id;

	msg_l = __raw_readl(
		(void __iomem *)(REGS_RECV_MBOX_BASE + MBOX_MSG_L));
	msg_h = __raw_readl(
		(void __iomem *)(REGS_RECV_MBOX_BASE + MBOX_MSG_H));
	target_id = __raw_readl(
		(void __iomem *)(REGS_RECV_MBOX_BASE + MBOX_ID));
	pr_debug("mbox:%s, id =%d, msg_l = 0x%x, msg_h = 0x%x\n",
		 __func__, target_id,
		 (unsigned int)msg_l, (unsigned int)msg_h);

	fifo->msg = (msg_h << 32) | msg_l;
	fifo->core_id = target_id & MBOX_MAX_CORE_MASK;
	g_recv_cnt[fifo->core_id]++;
	__raw_writel(0x1, (void __iomem *)(REGS_RECV_MBOX_BASE + MBOX_TRI));
}

static u8 mbox_read_all_fifo_msg(void)
{
	u32 fifo_sts;
	u8 fifo_depth;
	u8 rd, wt, cnt, i;
#ifdef MBOX_REMOVE_THE_SAME_MSG
	struct smsg *msg;
	u8 j, skip;
	struct  mbox_fifo_data_tag fifo_data;
#endif

	fifo_sts = __raw_readl(
		(void __iomem *)(REGS_RECV_MBOX_BASE + MBOX_FIFO_STS));
	wt = MBOX_GET_FIFO_WR_PTR(fifo_sts);
	rd = MBOX_GET_FIFO_RD_PTR(fifo_sts);

    /* if fifo is full or empty, when the read ptr == write ptr */
	if (rd == wt) {
		if (fifo_sts & MBOX_FIFO_FULL_STS_MASK) {
			g_outbox_full_cnt++;
			cnt = mbox_cfg.outbox_fifo_size;
		} else {
			cnt = 0;
		}
	} else {
		if (wt > rd)
			cnt = wt - rd;
		else
			cnt = mbox_cfg.outbox_fifo_size - rd + wt;
	}

	if (cnt == 0 || cnt > mbox_cfg.outbox_fifo_size)
		pr_err("mbox: %s, rd = %d, wt = %d, cnt = %d\n",
		       __func__, rd, wt, cnt);

	pr_debug("mbox: %s, rd = %d, wt = %d, cnt = %d\n",
		 __func__, rd, wt, cnt);

	fifo_depth = 0;
	for (i = 0; i < cnt; i++) {
#ifdef MBOX_REMOVE_THE_SAME_MSG
		mbox_raw_recv(&fifo_data);

		skip = 0;
		/* skip the same msg to reduce the irq proc time */
		for (j = 0; j < fifo_depth; j++) {
			if (fifo_data.core_id == mbox_fifo[j].core_id &&
			    fifo_data.msg == mbox_fifo[j].msg) {
				msg = (struct smsg *)&fifo_data.msg;
				/* only skip the event type msg */
				if (msg->type == SMSG_TYPE_EVENT) {
					skip = 1;
					g_skip_msg++;
					break;
				}
			}
		}

		if (skip)
			continue;

		/* copy to fifo */
		memcpy(&mbox_fifo[fifo_depth],
		       &fifo_data,
		       sizeof(struct  mbox_fifo_data_tag));
		fifo_depth++;
		pr_debug("mbox: %s, fifo_depth=%d\n", __func__, fifo_depth);
#else
		mbox_raw_recv(&mbox_fifo[i]);
		fifo_depth++;
#endif
	}

	/* the new mbox ip no need restall inbox */
	if (mbox_cfg.version <= 2) {
		/* if pre cnt + cur cnt >= mbox_cfg.outbox_fifo_size,
		 * may some inboxs have been blocked
		 */
		if (cnt + g_one_time_recv_cnt
			>= mbox_cfg.outbox_fifo_size)
			mbox_restore_all_inbox();
	}

	g_one_time_recv_cnt = cnt;

	return fifo_depth;
}

u32 mbox_core_fifo_full(int core_id)
{
	u32 fifo_sts = 0;
	u32 mail_box_offset;

	if (!g_mbox_inited) {
		pr_err("mbox:ERR on line %d! mbox not init\n", __LINE__);
		return 1;
	}

	mail_box_offset = REGS_RECV_MBOX_RANGE * core_id;

	fifo_sts = __raw_readl((void __iomem *)(
		REGS_RECV_MBOX_BASE + mail_box_offset + MBOX_FIFO_STS));

	if (fifo_sts & MBOX_FIFO_FULL_STS_MASK) {
		pr_info("mbox: is full,core_id = %d, fifo_sts=0x%x\n",
			core_id, fifo_sts);
		return 1;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(mbox_core_fifo_full);
#endif

static int mbox_phy_send(u8 core_id, u64 msg)
{
	u32 l_msg = (u32)msg;
	u32 h_msg = (u32)(msg >> 32);
	u32 fifo_sts, recv_flag;
	unsigned long recv_flag_cnt;

	pr_debug("mbox:%s, core_id=%d\n", __func__, (u32)core_id);

	/* if dst bit inbox block,  we dont't send it */
	fifo_sts = __raw_readl(
		(void __iomem *)(REGS_SEND_MBOX_BASE + MBOX_FIFO_STS));
	if (fifo_sts & (1 << core_id))
		goto block_exit;

	if (mbox_cfg.version > 2) {
		/* wait outbox recv flag, until flag is 0
		 * (mail be send to  outbox will clear it)
		 */
		recv_flag_cnt = 0;
		recv_flag = 1 << (core_id + MBOX_FIFO_OUTBOX_RECV_BIT);
		while (fifo_sts & recv_flag) {
			recv_flag_cnt++;
			fifo_sts =
			__raw_readl(
				    (void __iomem *)
				    (REGS_SEND_MBOX_BASE + MBOX_FIFO_STS));
			/* if block, outbox recv flag will always be 1,
			 * because mail cat't be send to outbox
			 */
			if (fifo_sts & (1 << core_id))
				goto block_exit;
		}

		if (mbox_chns[core_id].max_recv_flag_cnt < recv_flag_cnt)
			mbox_chns[core_id].max_recv_flag_cnt = recv_flag_cnt;
	}

	__raw_writel(l_msg,
		     (void __iomem *)
		     (REGS_SEND_MBOX_BASE + MBOX_MSG_L));
	__raw_writel(h_msg,
		     (void __iomem *)
		     (REGS_SEND_MBOX_BASE + MBOX_MSG_H));
	__raw_writel(core_id,
		     (void __iomem *)
		     (REGS_SEND_MBOX_BASE + MBOX_ID));
	__raw_writel(0x1,
		     (void __iomem *)
		     (REGS_SEND_MBOX_BASE + MBOX_TRI));

	g_send_cnt[core_id]++;

	return 0;

block_exit:

	pr_info("mbox: is block,core_id = %d, fifo_sts=0x%x",
		core_id, fifo_sts);
	g_inbox_block_cnt++;

	return -EBUSY;
}

int mbox_raw_sent(u8 core_id, u64 msg)
{
	unsigned long flag;

	if (!g_mbox_inited) {
		pr_err("mbox:ERR on line %d!\n", __LINE__);
		return -EINVAL;
	}

	if (core_id >= MBOX_MAX_CORE_CNT) {
		pr_err("mbox:ERR core_id = %d!\n", core_id);
		return -EINVAL;
	}

	spin_lock_irqsave(&mbox_lock, flag);

	/* if fifo is not empty, put it into the fifo,
	 * else if send failed , also put it into the fifo
	 */
	if (g_inptr[core_id] != g_outptr[core_id])
		mbox_put1msg(core_id, msg);
	else if (mbox_phy_send(core_id, msg) != 0)
		mbox_put1msg(core_id, msg);

	spin_unlock_irqrestore(&mbox_lock, flag);

	return 0;
}
EXPORT_SYMBOL_GPL(mbox_raw_sent);

void mbox_just_sent(u8 core_id, u64 msg)
{
	u32 l_msg = (u32)msg;
	u32 h_msg = (u32)(msg >> 32);

	__raw_writel(l_msg, (void __iomem *)(REGS_SEND_MBOX_BASE + MBOX_MSG_L));
	__raw_writel(h_msg, (void __iomem *)(REGS_SEND_MBOX_BASE + MBOX_MSG_H));
	__raw_writel(core_id, (void __iomem *)(REGS_SEND_MBOX_BASE + MBOX_ID));
	__raw_writel(0x1, (void __iomem *)(REGS_SEND_MBOX_BASE + MBOX_TRI));

	g_send_cnt[core_id]++;
}
EXPORT_SYMBOL_GPL(mbox_just_sent);

static int mbox_parse_dts(void)
{
	int ret;
	u32 enable_reg[MBOX_ENABLE_REG_INFO_NR];
	struct resource res;
	struct device_node *np;

	pr_info("mbox:%s!\n", __func__);

	np = of_find_compatible_node(NULL, NULL, "sprd,mailbox");
	if (!np) {
		pr_err("mbox: can't find %s!\n", "sprd,mailbox");
		return(-EINVAL);
	}

	/* init enable reg and mask bit */
	ret = of_property_read_u32_array(np,
					 "sprd,enable-reg",
					 enable_reg,
					 MBOX_ENABLE_REG_INFO_NR);

	if (!ret) {
		mbox_cfg.enable_reg = enable_reg[0];
		mbox_cfg.mask_bit = enable_reg[1];
	} else {
		mbox_cfg.enable_reg = MBOX_ENABLE_REG_OFFSET;
		mbox_cfg.mask_bit = MBOX_ENABLE_REG_MASK;
	}

	mailbox_gpr = syscon_regmap_lookup_by_phandle(np, "sprd,syscon-ap-apb");
	if (IS_ERR(mailbox_gpr)) {
		pr_err("mbox: failed to map mailbox_gpr\n");
		return PTR_ERR(mailbox_gpr);
	}

	/* parse inbox base */
	of_address_to_resource(np, 0, &res);
	mbox_cfg.inbox_base = res.start;
	sprd_inbox_base = (unsigned long)ioremap_nocache(res.start,
							 resource_size(&res));
	if (!sprd_inbox_base) {
		pr_err("mbox:ERR on line %d!\n", __LINE__);
		return -EINVAL;
	}

	/* parse out base */
	of_address_to_resource(np, 1, &res);
	mbox_cfg.outbox_base = res.start;
	sprd_outbox_base = (unsigned long)ioremap_nocache(res.start,
							  resource_size(&res));
	if (!sprd_outbox_base) {
		pr_err("mbox:ERR on line %d!\n", __LINE__);
		return -EINVAL;
	}

	/* parse inbox irq */
	ret = of_irq_get(np, 0);
	if (ret < 0) {
		pr_err("mbox:ERR on line %d!\n", __LINE__);
		return -EINVAL;
	}
	mbox_cfg.inbox_irq = ret;

	/* parse outbox irq */
	ret = of_irq_get(np, 1);
	if (ret < 0) {
		pr_err("mbox:ERR on line %d!\n", __LINE__);
		return -EINVAL;
	}
	mbox_cfg.outbox_irq = ret;

#ifdef CONFIG_SPRD_MAILBOX_SENSOR
	/* parse outbox sensor irq, in dt config */
	ret = of_irq_get(np, 2);
	if (ret < 0) {
		pr_err("mbox:ERR on line %d!\n", __LINE__);
		return -EINVAL;
	}
	mbox_cfg.outbox_sensor_irq = ret;

	/* parse sensor core */
	ret = of_property_read_u32(np, "sprd,sensor", &mbox_cfg.sensor_core);
	if (ret) {
		pr_info("mbox:no sprd,sensor!\n");
		mbox_cfg.sensor_core = RECV_MBOX_SENSOR_ID;
	}

	if (mbox_cfg.sensor_core >= MBOX_MAX_CORE_CNT) {
		pr_err("mbox:ERR on line %d!\n", __LINE__);
		return -EINVAL;
	}
#endif

	/* parse core cnt */
	ret = of_property_read_u32(np, "sprd,core-cnt", &mbox_cfg.core_cnt);
	if (ret) {
		pr_err("mbox:ERR on line %d!\n", __LINE__);
		return -EINVAL;
	}

	if (mbox_cfg.core_cnt > MBOX_MAX_CORE_CNT) {
		pr_err("mbox:ERR on line %d!\n", __LINE__);
		return -EINVAL;
	}

	/* parse mbox version */
	ret = of_property_read_u32(np, "sprd,version", &mbox_cfg.version);
	if (ret) {
		pr_err("mbox:ERR on line %d!\n", __LINE__);
		return -EINVAL;
	}

	return 0;
}

static void mbox_cfg_init(void)
{

	if (mbox_cfg.version > 1) {
		/* init fifo size */
		mbox_cfg.inbox_fifo_size  = MBOX_V2_INBOX_FIFO_SIZE;
		mbox_cfg.outbox_fifo_size = MBOX_V2_OUTBOX_FIFO_SIZE;

		/* init fifo read ptr */
		mbox_cfg.rd_bit  = MBOX_V2_READ_PT_BIT;
		mbox_cfg.rd_mask = MBOX_V2_READ_PT_MASK;

		/* init fifo write ptr */
		mbox_cfg.wr_bit  = MBOX_V2_WRITE_PT_BIT;
		mbox_cfg.wr_mask = MBOX_V2_WRITE_PT_MASK;

		/* init core range */
		mbox_cfg.inbox_range  = MBOX_V2_INBOX_CORE_SIZE;
		mbox_cfg.outbox_range = MBOX_V2_OUTBOX_CORE_SIZE;

		/* init irq mask */
		mbox_cfg.inbox_irq_mask  = MBOX_V2_INBOX_IRQ_MASK;
		mbox_cfg.outbox_irq_mask = MBOX_V2_OUTBOX_IRQ_MASK;
	} else {
		/* init fifo size */
		mbox_cfg.inbox_fifo_size  = MBOX_V1_INBOX_FIFO_SIZE;
		mbox_cfg.outbox_fifo_size = MBOX_V1_OUTBOX_FIFO_SIZE;

		/* init fifo read ptr */
		mbox_cfg.rd_bit  = MBOX_V1_READ_PT_BIT;
		mbox_cfg.rd_mask = MBOX_V1_READ_PT_MASK;

		/* init fifo write ptr */
		mbox_cfg.wr_bit  = MBOX_V1_WRITE_PT_BIT;
		mbox_cfg.wr_mask = MBOX_V1_WRITE_PT_MASK;

		/* init core range */
		mbox_cfg.inbox_range  = MBOX_V1_INBOX_CORE_SIZE;
		mbox_cfg.outbox_range = MBOX_V1_OUTBOX_CORE_SIZE;

		/* init irq mask */
		mbox_cfg.inbox_irq_mask  = MBOX_V1_INBOX_IRQ_MASK;
		mbox_cfg.outbox_irq_mask = MBOX_V1_OUTBOX_IRQ_MASK;
	}
}

static void mbox_cfg_printk(void)
{
	pr_info("mbox:inbox_base = 0x%x, outbox_base = 0x%x\n",
		mbox_cfg.inbox_base, mbox_cfg.outbox_base);

	pr_info("mbox:inbox_range = 0x%x, outbox_range = 0x%x\n",
		mbox_cfg.inbox_range, mbox_cfg.outbox_range);

	pr_info("mbox:inbox_fifo = %d, outbox_fifo = %d\n",
		mbox_cfg.inbox_fifo_size, mbox_cfg.outbox_fifo_size);

	pr_info("mbox:inbox_irq = %d, outbox_irq = %d\n",
		mbox_cfg.inbox_irq, mbox_cfg.outbox_irq);

	pr_info("mbox:inbox_irq_mask = 0x%x, outbox_irq_mask = 0x%x\n",
		mbox_cfg.inbox_irq_mask, mbox_cfg.outbox_irq_mask);

#ifdef CONFIG_SPRD_MAILBOX_SENSOR
	pr_info("mbox:outbox_sensor_irq = %d\n",
		mbox_cfg.outbox_sensor_irq);

	pr_info("mbox:sensor_core = %d\n",
		mbox_cfg.sensor_core);
#endif

	pr_info("mbox:rd_bit = %d, rd_mask = %d\n",
		mbox_cfg.rd_bit, mbox_cfg.rd_mask);

	pr_info("mbox:wr_bit = %d, wr_mask = %d\n",
		mbox_cfg.wr_bit, mbox_cfg.wr_mask);

	pr_info("mbox:enable_reg = 0x%x, mask_bit = 0x%x\n",
		mbox_cfg.enable_reg, mbox_cfg.mask_bit);

	pr_info("mbox:core_cnt = %d, version = %d\n",
		mbox_cfg.core_cnt, mbox_cfg.version);
}

#ifdef CONFIG_SPRD_MAILBOX_SENSOR
static int mbox_pm_event(struct notifier_block *notifier,
			 unsigned long pm_event, void *unused)
{
	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		pr_debug("mbox:PM_SUSPEND_PREPARE\n");
		/* disable all outbox irq mask */
		__raw_writel(MBOX_IRQ_DISABLE_ALLIRQ,
			     (void __iomem *)(REGS_RECV_MBOX_SENSOR_BASE
			     + MBOX_IRQ_MSK));

		break;

	case PM_POST_SUSPEND:
		pr_debug("mbox:PM_POST_SUSPEND\n");
		/* restore outbox irq mask */
		__raw_writel(mbox_cfg.outbox_irq_mask,
			     (void __iomem *)(REGS_RECV_MBOX_SENSOR_BASE
			     + MBOX_IRQ_MSK));
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block mailbox_pm_notifier_block = {
	.notifier_call = mbox_pm_event,
};
#endif

static int __init mbox_init(void)
{
	int ret;

	pr_info("mbox:%s!\n", __func__);

	ret = mbox_parse_dts();
	if (ret)
		return -EINVAL;

	mbox_cfg_init();

	/* 0x402e0004, bit 21 , enable mailbox clk */
	if (mailbox_gpr) {
		pr_info("mbox:enable mailbox clk!\n");

		regmap_update_bits(mailbox_gpr,
				   mbox_cfg.enable_reg,
				   mbox_cfg.mask_bit,
				   mbox_cfg.mask_bit);
	}

	/* read fifo depth and version form reg, must + 1 */
	if (mbox_cfg.version > 2) {
		mbox_cfg.outbox_fifo_size = __raw_readl(
		(void __iomem *)(REGS_RECV_MBOX_BASE + MBOX_FIFO_DEPTH)) + 1;
		mbox_cfg.version = __raw_readl(
		(void __iomem *)(REGS_RECV_MBOX_BASE + MBOX_VERSION));
	}

	mbox_cfg_printk();

	spin_lock_init(&mbox_lock);

#ifdef CONFIG_SPRD_MAILBOX_FIFO
	/* normal mode */
	__raw_writel(0x0,
		     (void __iomem *)(REGS_RECV_MBOX_BASE +  MBOX_FIFO_RST));
#else
	/* irq only mode */
	__raw_writel(0x10000,
		     (void __iomem *)(REGS_RECV_MBOX_BASE +  MBOX_FIFO_RST));
#endif

	/* set inbox irq mask */
	g_inbox_irq_mask = mbox_cfg.inbox_irq_mask;
	__raw_writel(mbox_cfg.inbox_irq_mask,
		     (void __iomem *)(REGS_SEND_MBOX_BASE + MBOX_IRQ_MSK));

	/* set outbox irq mask */
	__raw_writel(mbox_cfg.outbox_irq_mask,
		     (void __iomem *)(REGS_RECV_MBOX_BASE + MBOX_IRQ_MSK));

#ifdef CONFIG_SPRD_MAILBOX_SENSOR
	/* set outbox irq mask */
	__raw_writel(mbox_cfg.outbox_irq_mask,
		     (void __iomem *)(REGS_RECV_MBOX_SENSOR_BASE
		     + MBOX_IRQ_MSK));

	ret = register_pm_notifier(&mailbox_pm_notifier_block);
	if (ret) {
		pr_err("mbox:ERR on line %d!\n", __LINE__);
		return -EINVAL;
	}
#endif

	/* lock irq here until g_mbox_inited =1,
	 * it make sure that when irq handle come,
	 * g_mboX_inited is 1
	 */
	g_mbox_inited = 1;

	/* request inbox arm irq ,and enable wake up */
	ret = request_irq(mbox_cfg.inbox_irq,
			  mbox_src_irqhandle,
			  IRQF_NO_SUSPEND,
			  "sprd-mailbox_source",
			  NULL);
	if (ret) {
		pr_err("mbox:ERR on line %d!\n", __LINE__);
		return -EINVAL;
	}

	enable_irq_wake(mbox_cfg.inbox_irq);

	/* request outbox arm irq, and enable wake up */
	ret = request_irq(mbox_cfg.outbox_irq,
			  mbox_recv_irqhandle,
			  IRQF_NO_SUSPEND,
			  "sprd-mailbox_target",
			  NULL);
	if (ret) {
		pr_err("mbox:ERR on line %d!\n", __LINE__);
		return -EINVAL;
	}
	enable_irq_wake(mbox_cfg.outbox_irq);

#ifdef CONFIG_SPRD_MAILBOX_SENSOR
	if (mbox_cfg.outbox_sensor_irq) {
		ret = request_irq(mbox_cfg.outbox_sensor_irq,
				  mbox_sensor_recv_irqhandle,
				  IRQF_NO_SUSPEND,
				  "sprd-mailbox_target",
				  NULL);
		if (ret) {
			pr_err("mbox:ERR on line %d!\n", __LINE__);
			return -EINVAL;
		}

		enable_irq_wake(mbox_cfg.outbox_sensor_irq);
	}
#endif

#ifdef SPRD_MAILBOX_TEST
	mbox_test_init();
#endif

	init_waitqueue_head(&g_send_wait);
	g_send_thread = kthread_create(mbox_send_thread,
				       NULL,
				       "mbox-send-thread");
	wake_up_process(g_send_thread);

	return 0;
}

subsys_initcall(mbox_init);

static u8 mbox_get_send_fifo_mask(u8 send_bit)
{
	u8 dst_bit, dst, mask;

	spin_lock(&mbox_lock);

	mask = 0;
	for (dst = 0; dst < MBOX_MAX_CORE_CNT; dst++) {
		dst_bit = (1 << dst);
		if (send_bit & dst_bit) {
			if (g_outptr[dst] != g_inptr[dst])
				mask |= (dst_bit);
		}
	}

	spin_unlock(&mbox_lock);

	return mask;
}

static void mbox_put1msg(u8 dst, u64 msg)
{
	u16 rd, wt, pos;

	rd = g_outptr[dst];
	wt = g_inptr[dst];

	pos = wt % SEND_FIFO_LEN;
	/* if fifo full, return */
	if ((rd != wt) && (rd % SEND_FIFO_LEN == pos)) {
		pr_err("mbox:dst = %d, rd =%d, wt = %d\n", dst, rd, wt);
		return;
	}

	pr_debug("mbox:dst = %d, rd =%d, wt = %d, pos = %d\n",
		 dst, rd, wt, pos);

	g_send_fifo[dst][pos] = msg;
	g_inptr[dst] = wt + 1;
}

static int mbox_send_thread(void *pdata)
{
	unsigned long flag;
	u8 dst;
	u16 rd, pos;
	u64 msg;

	pr_info("mbox:%s!\n", __func__);

	while (!kthread_should_stop()) {
		wait_event(g_send_wait, g_inbox_send);

		pr_debug("mbox:%s, wait event!\n", __func__);

		/* Event triggered, process all FIFOs. */
		spin_lock_irqsave(&mbox_lock, flag);

		for (dst = 0; dst < MBOX_MAX_CORE_CNT; dst++) {
			if (!(g_inbox_send & (1 << dst)))
				continue;

			rd = g_outptr[dst];
			while (g_inptr[dst] != rd) {
				pos = rd % SEND_FIFO_LEN;
				msg = g_send_fifo[dst][pos];

				if (mbox_phy_send(dst, msg) != 0)
					/* one send failed, than next */
					break;

				rd += 1;
				g_outptr[dst] = rd;
			}
		}

		g_inbox_send = 0;
		spin_unlock_irqrestore(&mbox_lock, flag);
	}

	return 0;
}

static void mbox_start_fifo_send(void)
{
	pr_debug("mbox:%s!\n", __func__);

	wake_up(&g_send_wait);
}

#if defined(CONFIG_DEBUG_FS)
static void mbox_check_all_send_fifo(struct seq_file *m)
{
	u8 dst;
	u16 rd, wt, len;
	unsigned long flag;

	spin_lock_irqsave(&mbox_lock, flag);
	for (dst = 0; dst < MBOX_MAX_CORE_CNT; dst++) {
		rd = g_outptr[dst];
		wt = g_inptr[dst];

		if (rd <= wt)
			len = wt - rd;
		else
			len = (wt % SEND_FIFO_LEN)
				+ (SEND_FIFO_LEN - rd % SEND_FIFO_LEN);

		if (len > 0)
			seq_printf(m,
				   "	inbox fifo %d len is %d!\n",
				   dst, len);
	}
	spin_unlock_irqrestore(&mbox_lock, flag);
}

static void mbox_check_all_inbox(struct seq_file *m)
{
	int i;
	unsigned long inbox;
	u32 status;

	for (i = 0; i < mbox_cfg.core_cnt; i++) {
		inbox = REGS_SEND_MBOX_BASE + i * REGS_SEND_MBOX_RANGE;
		status = __raw_readl((void __iomem *)(inbox + MBOX_FIFO_STS));

		/* find the block inbox*/
		if (status & MBOX_FIFO_BLOCK_OVERFLOW_MASK)
			seq_printf(m, "    inbox %d is block!\n", i);
	}
}

static void mbox_check_all_outbox(struct seq_file *m)
{
	int i;
	unsigned long outbox;
	u32 status;

	for (i = 0; i < mbox_cfg.core_cnt; i++) {
		outbox = REGS_RECV_MBOX_BASE + i * REGS_RECV_MBOX_RANGE;
		status = __raw_readl((void __iomem *)(outbox + MBOX_FIFO_STS));

		/* find the full outbox */
		if (status & MBOX_FIFO_FULL_STS_MASK)
			seq_printf(m, "    outbox %d is full!\n", i);
	}
}

static int mbox_debug_show(struct seq_file *m, void *private)
{
	int i;
	unsigned long inbox;

	/* mbox */
	sipc_debug_putline(m, '*', 120);
	seq_puts(m, "1. mailbox config:\n");
	sipc_debug_putline(m, '-', 110);

	seq_printf(m, "\n    enable_reg:  0x%08x,     reg_mask:    0x%08x\n",
		   mbox_cfg.enable_reg, mbox_cfg.mask_bit);
	seq_printf(m, "    inbox_base:  0x%08x,     outbox_base:  0x%08x\n",
		   mbox_cfg.inbox_base, mbox_cfg.outbox_base);
	seq_printf(m, "    inbox_range: 0x%08x,     outbox_range: 0x%08x\n",
		   mbox_cfg.inbox_range, mbox_cfg.outbox_range);
	seq_printf(m, "    inbox_mask:  0x%08x,     outbox_mask:  0x%08x\n",
		   mbox_cfg.inbox_irq_mask, mbox_cfg.outbox_irq_mask);
	seq_printf(m, "\n    inbox_fifo: 0x%02x,     outbox_fifo: 0x%02x\n",
		   mbox_cfg.inbox_fifo_size, mbox_cfg.outbox_fifo_size);
	seq_printf(m, "    rd_pt_bit:  0x%02x,     wr_pt_bit:   0x%02x\n",
		   mbox_cfg.rd_bit, mbox_cfg.wr_bit);
	seq_printf(m, "    core_cnt:   0x%02x,     version:     0x%02x\n",
		   mbox_cfg.core_cnt, mbox_cfg.version);
	seq_printf(m, "    inbox_irq:  0x%02x,     outbox_irq:  0x%02x\n",
		   mbox_cfg.inbox_irq, mbox_cfg.outbox_irq);
#ifdef CONFIG_SPRD_MAILBOX_SENSOR
	seq_printf(m, "    sensor_id:  0x%02x,	   sensor_irq:	0x%02x\n",
		   mbox_cfg.sensor_core, mbox_cfg.outbox_sensor_irq);
#endif

	sipc_debug_putline(m, '-', 110);
	seq_puts(m, "\n2. mailbox outbox reg:\n");
	for (i = 0; i < mbox_cfg.core_cnt; i++) {
		inbox = REGS_RECV_MBOX_BASE + i * REGS_RECV_MBOX_RANGE;
		sipc_debug_putline(m, '-', 105);
		seq_printf(m, "outbox %d reg start:\n", i);
		seq_printf(m, "\n  core_id:     0x%08x",
			   __raw_readl((void __iomem *)(inbox + 0x0)));

		seq_printf(m, "  msg_low:     0x%08x  msg_high:    0x%08x\n",
			   __raw_readl((void __iomem *)(inbox + 0x4)),
			   __raw_readl((void __iomem *)(inbox + 0x8)));

		seq_printf(m, "  fifo_reset:  0x%08x  fifo_status: 0x%08x",
			   __raw_readl((void __iomem *)(inbox + 0x10)),
			   __raw_readl((void __iomem *)(inbox + 0x14)));

		seq_printf(m, "  irq_status:  0x%08x  irq_mask:    0x%08x\n",
			   __raw_readl((void __iomem *)(inbox + 0x18)),
			   __raw_readl((void __iomem *)(inbox + 0x1c)));

		if (mbox_cfg.version <= 2)
			continue;

		seq_printf(m, "  fifo_depth:  %d  version: 0x%08x\n",
			   __raw_readl((void __iomem *)(inbox + 0x24)) + 1,
			   __raw_readl((void __iomem *)(inbox + 0x28)));
	}

	sipc_debug_putline(m, '-', 110);
	seq_puts(m, "\n3. mailbox inbox reg:\n");

	for (i = 0; i < mbox_cfg.core_cnt; i++) {
		sipc_debug_putline(m, '-', 105);
		seq_printf(m, "inbox %d reg start:\n", i);

		inbox = REGS_SEND_MBOX_BASE + i * REGS_SEND_MBOX_RANGE;

		seq_printf(m, "\n  core_id:     0x%08x",
			   __raw_readl((void __iomem *)(inbox + 0x0)));

		seq_printf(m, "  msg_low:     0x%08x  msg_high:    0x%08x\n",
			   __raw_readl((void __iomem *)(inbox + 0x4)),
			   __raw_readl((void __iomem *)(inbox + 0x8)));

		seq_printf(m, "  fifo_reset:  0x%08x  fifo_status: 0x%08x",
			   __raw_readl((void __iomem *)(inbox + 0x10)),
			   __raw_readl((void __iomem *)(inbox + 0x14)));

		seq_printf(m, "  irq_status:  0x%08x  irq_mask:    0x%08x\n",
			   __raw_readl((void __iomem *)(inbox + 0x18)),
			   __raw_readl((void __iomem *)(inbox + 0x1c)));
	}

	sipc_debug_putline(m, '-', 110);

	seq_puts(m, "\n4. mailbox data:\n");

	seq_printf(m, "\n    restore_cnt: %d,     block_cnt: %d\n",
		   g_restore_cnt, g_inbox_block_cnt);
	seq_printf(m, "    oboxfull_cnt:%d,     skip_msg:  %d\n",
		   g_outbox_full_cnt, g_skip_msg);
	seq_printf(m, "    max_total_irq_time: %lu\n",
		   max_total_irq_proc_time);
	seq_printf(m, "    max_total_irq_cnt: %lu\n\n",
		   max_total_irq_cnt);

	for (i = 0; i < mbox_cfg.core_cnt; i++) {
		if (mbox_chns[i].mbox_smsg_handler)
			seq_printf(m, "    mbox core %d,        max_irq_time: %lu\n",
				   i, mbox_chns[i].max_irq_proc_time);

		seq_printf(m, "    mbox core %d,        max_recv_flag_cnt: %lu\n",
			   i, mbox_chns[i].max_recv_flag_cnt);
	}

	sipc_debug_putline(m, '-', 110);

	seq_puts(m, "\n5. mailbox total:\n");

	for (i = 0; i < mbox_cfg.core_cnt; i++) {
		seq_printf(m, "    mbox core %d,        recv_mail_cnt: %u\n",
			   i, g_recv_cnt[i]);
		seq_printf(m, "    mbox core %d,        send_mail_cnt: %u\n",
			   i, g_send_cnt[i]);
	}

	mbox_check_all_inbox(m);
	mbox_check_all_outbox(m);
	mbox_check_all_send_fifo(m);

	return 0;
}

static int mbox_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, mbox_debug_show, inode->i_private);
}

static const struct file_operations mbox_debug_fops = {
	.open = mbox_debug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

int  mbox_init_debugfs(void *root)
{
	if (!root)
		return -ENXIO;

	debugfs_create_file("mbox", S_IRUGO,
			    (struct dentry *)root,
			    NULL,
			    &mbox_debug_fops);
	return 0;
}
EXPORT_SYMBOL_GPL(mbox_init_debugfs);

#endif /* CONFIG_DEBUG_FS */

MODULE_AUTHOR("zhou wenping");
MODULE_DESCRIPTION("SIPC/mailbox driver");
MODULE_LICENSE("GPL");

/*
 * Copyright (C) 2012-2015 Spreadtrum Communications Inc.
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

#include <asm/cacheflush.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/file.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/regmap.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/spinlock_types.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>

#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

#include <uapi/video/isp_drv_kernel.h>

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_DRV: %d: %d " fmt, current->pid, __LINE__

#define ISP_QUEUE_LENGTH           16
#define SA_SHIRQ                   IRQF_SHARED

/* isp minor number */
#define ISP_MINOR                  MISC_DYNAMIC_MINOR
#define init_MUTEX(sem)            sema_init(sem, 1)
#define init_MUTEX_LOCKED(sem)     sema_init(sem, 0)
#define IO_PTR                     volatile void __iomem *
#define ISP_READL(a)               readl_relaxed((IO_PTR)a)
#define ISP_WRITEL(a,v)            writel_relaxed(v,(IO_PTR)a)

#define ISP_OWR(a,v)        \
		writel_relaxed((readl_relaxed((IO_PTR)a) | v), (IO_PTR)a)
#define ISP_AWR(a,v)        \
		writel_relaxed((readl_relaxed((IO_PTR)a) & v), (IO_PTR)a)
#define ISP_NAWR(a,v)       \
		writel_relaxed((readl_relaxed((IO_PTR)a) & ~v), (IO_PTR)a)
#define ISP_REG_RD(a)                      ISP_READL(((IO_PTR)a))

#define ISP_CHECK_ZERO(a)                                 \
	do {                                              \
		if (ISP_ADDR_INVALID(a)) {                \
			pr_err("zero pointer\n");  \
			return -EFAULT;                   \
		}                                         \
	} while(0)

#define ISP_CHECK_ZERO_VOID(a)                            \
	do {                                              \
		if (ISP_ADDR_INVALID(a)) {                \
			pr_err("zero pointer\n");  \
			return;                           \
		}                                         \
	} while(0)

#define ISP_ADDR_INVALID(addr)     \
			((unsigned long)(addr) < (unsigned long)ISP_LOWEST_ADDR)

#define ISP_LOWEST_ADDR            0x800
#define ISP_BUF_MAX_SIZE           ISP_TMP_BUF_SIZE_MAX_V0001
#define ISP_IRQ_NUM                ISP_IRQ_NUM_V0001
#define ISP_IRQ_HW_MASK            ISP_IRQ_HW_MASK_V0001
#define ISP_TIME_OUT_MAX           (500)

#define ISP_EB_BIT                 BIT(2)
#define ISP_RST_LOG_BIT            BIT(2)
#define ISP_RST_CFG_BIT            BIT(3)
#define ISP_CORE_CLK_EB_BIT        BIT(4)
#define ISP_AXI_MASTER_STOP_BIT    BIT(0)

unsigned long isp_regbase = 0;
unsigned long isp_reg_max_size = 0;
unsigned int isp_irq = 0;

#define ISP_BASE                   (isp_regbase)
#define ISP_BASE_ADDR              ISP_BASE
#define ISP_LNC_STATUS             (ISP_BASE_ADDR+0x0200UL)
#define ISP_LNC_LOAD               (ISP_BASE_ADDR+0x0220UL)
#define ISP_AXI_MASTER             (ISP_BASE_ADDR+0x2000UL)
#define ISP_INT_RAW                (ISP_BASE_ADDR+0x2080UL)
#define ISP_INT_STATUS             (ISP_BASE_ADDR+0x2084UL)
#define ISP_INT_EN                 (ISP_BASE_ADDR+0x2078UL)
#define ISP_INT_CLEAR              (ISP_BASE_ADDR+0x207cUL)
#define ISP_REG_MAX_SIZE           (isp_reg_max_size)
#define ISP_AXI_MASTER_STOP        (ISP_BASE_ADDR + 0X2054UL)

#define MM_AHB_RESET               0x0004UL

#define ISP_IRQ                    (isp_irq)

#define ISP_IRQ_HW_MASK_V0001      (0x1fffffff)
#define ISP_IRQ_NUM_V0001          (29)
#define ISP_TMP_BUF_SIZE_MAX_V0001 (32 * 1024)

struct isp_node {
	uint32_t isp_irq_val;
	uint32_t dcam_irq_val;
	uint64_t system_time;
};

struct isp_queue {
	struct isp_node node[ISP_QUEUE_LENGTH];
	struct isp_node *write;
	struct isp_node *read;
};

struct isp_lnc_load {
	uint8_t load_lnc_flag;
	uint8_t cap_eof_flag;
	uint32_t param_counts;
	unsigned long load_buf;
	struct semaphore load_done_sem;
};

struct isp_device_t {
	/* the pointer of isp register context */
	unsigned long reg_base_addr;
	/* struct size */
	uint32_t size;
	unsigned long buf_addr;
	uint32_t buf_len;
	struct isp_queue queue;
	/* for interrupts */
	struct semaphore sem_isr;
	/* for the isp device, protect the isp hardware;protect only one
	   caller use the oi controll/read/write functions */
	struct semaphore sem_isp;
	struct clk *s_isp_clk_mm_i;
	struct clk *s_isp_clk;
	struct semaphore kernel_sem;
	struct task_struct *kernel_thread;
	struct isp_lnc_load lnc_load;
	uint32_t is_kernel_thread_stop;
};

static struct isp_device_t *isp_dev_ptr = NULL;

uint32_t dcam_int_eb = 0x00;
uint32_t isp_alloc_order = 0x00;
uint32_t isp_alloc_len = 0x00;
unsigned long isp_alloc_addr = 0x00;
static atomic_t isp_users = ATOMIC_INIT(0);

/* for the isp driver, protect the isp module;
protect only one user open this module */
static struct mutex isp_lock;
static struct proc_dir_entry *isp_proc_file;
struct regmap *mmahb_gpr;
static struct miscdevice isp_dev;

static DEFINE_SPINLOCK(isp_spin_lock);
static int32_t isp_module_eb(void);
static int32_t isp_module_dis(void);
static int isp_registerirq(void);
static void isp_unregisterirq(void);
static irqreturn_t isp_irq_root(int irq, void *dev_id);
void dcam_isp_root(void);
static int isp_queue_init(struct isp_queue *queue);
static int isp_queue_write(struct isp_queue *queue, struct isp_node *node);
static int isp_queue_read(struct isp_queue *queue, struct isp_node *node);
static inline void isp_regread(char *dst, char *src, size_t n);
static inline void isp_regwrite(char *dst, char *src, size_t n);
static void read_reg(struct isp_reg_bits *reg_bits_ptr, uint32_t counts);
static void write_reg(struct isp_reg_bits *reg_bits_ptr, uint32_t counts);

static int32_t isp_lnc_param_load_ex(struct isp_reg_bits *reg_bits_ptr,
				     uint32_t counts);
static int isp_stop_kernel_thread(void *param);

static int32_t isp_kernel_open(struct inode *node, struct file *filp);
static int32_t isp_kernel_release(struct inode *node, struct file *filp);
static long isp_kernel_ioctl(struct file *fl,
			     unsigned int cmd, unsigned long param);
long compat_isp_kernel_ioctl(struct file *filp,
			     unsigned int cmd, unsigned long arg);

static int32_t isp_probe(struct platform_device *pdev);
static int32_t isp_remove(struct platform_device *dev);

extern int mm_pw_on(unsigned int client);
extern int mm_pw_off(unsigned int client);

static int32_t isp_get_systemtime(struct timeval *tv)
{
	struct timespec ts;

	ktime_get_ts(&ts);
	tv->tv_sec = ts.tv_sec;
	tv->tv_usec = ts.tv_nsec / NSEC_PER_USEC;
	return 0;
}

static int32_t isp_module_eb(void)
{
	int32_t ret = 0;
	struct clk *clk_mm_i = NULL;
	struct clk *clk_parent = NULL;
	struct clk *isp_axi_eb = NULL;
	struct clk *isp_eb = NULL;

	if (0x01 == atomic_inc_return(&isp_users)) {
		ret = mm_pw_on(1);
		if (ret)
			return ret;

		isp_axi_eb = devm_clk_get(isp_dev.this_device, "isp_axi_eb");
		if (NULL == isp_axi_eb) {
			pr_err("devm_clk_get failed\n");
			return -1;
		}
		ret = clk_prepare_enable(isp_axi_eb);
		if (unlikely(0 != ret)) {
			pr_err("clk_prepare_enable failed\n");
			return -1;
		}

		isp_eb = devm_clk_get(isp_dev.this_device, "isp_eb");
		if (NULL == isp_eb) {
			pr_err("devm_clk_get failed\n");
			return -1;
		}
		ret = clk_prepare_enable(isp_eb);
		if (unlikely(0 != ret)) {
			pr_err("clk_prepare_enable failed\n");
			return -1;
		}

		clk_mm_i = devm_clk_get(isp_dev.this_device, "clk_mm_i");
		if (NULL == clk_mm_i) {
			pr_err("devm_clk_get failed\n");
			return -1;
		}

		ret = clk_prepare_enable(clk_mm_i);
		if (unlikely(0 != ret)) {
			pr_err("clk_prepare_enable failed\n");
			return -1;
		}

		isp_dev_ptr->s_isp_clk = devm_clk_get(isp_dev.this_device,
						      "clk_isp");
		if (IS_ERR(isp_dev_ptr->s_isp_clk)) {
			pr_err("devm_clk_get fail, %p\n",
			       isp_dev_ptr->s_isp_clk);
			return -1;
		}

		clk_parent = devm_clk_get(isp_dev.this_device,
					  "clk_isp_parent");
		if (IS_ERR(clk_parent)) {
			pr_err("devm_clk_get fail, %p\n", clk_parent);
			return -1;
		}

		ret = clk_set_parent(isp_dev_ptr->s_isp_clk, clk_parent);
		if (ret) {
			pr_err("clk_set_parent fail, %d\n", ret);
		}

		ret = clk_prepare_enable(isp_dev_ptr->s_isp_clk);
		if (ret) {
			pr_err("clk_prepare_enable failed\n");
			return -1;
		}
	}

	pr_debug("end\n");
	return ret;
}

static int32_t isp_module_dis(void)
{
	int32_t ret = 0;
	struct clk *clk_mm_i = NULL;
	struct clk *isp_axi_eb = NULL;
	struct clk *isp_eb = NULL;

	if (0x00 == atomic_dec_return(&isp_users)) {
		if (isp_dev_ptr->s_isp_clk) {
			clk_disable_unprepare(isp_dev_ptr->s_isp_clk);
			devm_clk_put(isp_dev.this_device,
				     isp_dev_ptr->s_isp_clk);
			isp_dev_ptr->s_isp_clk = NULL;
		}

		clk_mm_i = devm_clk_get(isp_dev.this_device, "clk_mm_i");
		if (NULL == clk_mm_i) {
			pr_err("devm_clk_get failed\n");
			return -1;
		}

		clk_disable_unprepare(clk_mm_i);
		devm_clk_put(isp_dev.this_device, clk_mm_i);
		clk_mm_i = NULL;

		isp_axi_eb = devm_clk_get(isp_dev.this_device, "isp_axi_eb");
		clk_disable_unprepare(isp_axi_eb);
		isp_axi_eb = NULL;
		isp_eb = devm_clk_get(isp_dev.this_device, "isp_eb");
		clk_disable_unprepare(isp_eb);
		isp_eb = NULL;

		mm_pw_off(1);
	}
	return ret;
}

static int32_t isp_module_rst(void)
{
	int32_t ret = 0;
	uint32_t reg_value = 0x00;
	int32_t time_out_cnt = 0;

	if (0x00 != atomic_read(&isp_users)) {
		ISP_OWR(ISP_AXI_MASTER_STOP, BIT(0));
		reg_value = ISP_READL(ISP_AXI_MASTER);
		while ((0x00 == (reg_value & 0x08)) &&
		       (time_out_cnt < (ISP_TIME_OUT_MAX * 2))) {
			time_out_cnt++;
			udelay(50);
			reg_value = ISP_READL(ISP_AXI_MASTER);
		}

		if (time_out_cnt >= ISP_TIME_OUT_MAX) {
			ret = -1;
			pr_err("time out\n");
		}

		ISP_WRITEL(ISP_INT_CLEAR, ISP_IRQ_HW_MASK);

		regmap_update_bits(mmahb_gpr, MM_AHB_RESET, ISP_RST_LOG_BIT,
				   ISP_RST_LOG_BIT);
		regmap_update_bits(mmahb_gpr, MM_AHB_RESET, ISP_RST_CFG_BIT,
				   ISP_RST_CFG_BIT);
		regmap_update_bits(mmahb_gpr, MM_AHB_RESET, ISP_RST_LOG_BIT,
				   ISP_RST_LOG_BIT);
		regmap_update_bits(mmahb_gpr, MM_AHB_RESET, ISP_RST_CFG_BIT,
				   ISP_RST_CFG_BIT);
		regmap_update_bits(mmahb_gpr, MM_AHB_RESET, ISP_RST_LOG_BIT,
				   ISP_RST_LOG_BIT);
		regmap_update_bits(mmahb_gpr, MM_AHB_RESET, ISP_RST_CFG_BIT,
				   ISP_RST_CFG_BIT);

		regmap_update_bits(mmahb_gpr, MM_AHB_RESET, ISP_RST_CFG_BIT,
				(unsigned int)~ISP_RST_CFG_BIT);
		regmap_update_bits(mmahb_gpr, MM_AHB_RESET, ISP_RST_LOG_BIT,
				(unsigned int)~ISP_RST_LOG_BIT);
	}

	pr_debug("exit\n");
	return 0;
}

int isp_kernel_proc(void *arg)
{
	struct isp_device_t *dev = (struct isp_device_t *)arg;
	int ret = 0;
	struct isp_reg_bits *reg_bits_ptr;
	uint32_t counts;

	while (1) {
		if (0 == down_interruptible(&dev->kernel_sem)) {
			if (dev->is_kernel_thread_stop) {
				pr_debug("stop\n");
				break;
			}
			reg_bits_ptr = (struct isp_reg_bits *)
			    isp_dev_ptr->lnc_load.load_buf;

			counts = isp_dev_ptr->lnc_load.param_counts;
			ret = isp_lnc_param_load_ex(reg_bits_ptr, counts);
			if (ret) {
				pr_err("isp_lnc_param_load_ex failed\n");
				return ret;
			}
			memset((void *)isp_dev_ptr->lnc_load.load_buf,
			       0x00, ISP_BUF_MAX_SIZE);
		}
	}
	dev->is_kernel_thread_stop = 0;
	return ret;
}

int isp_kernel_thread(void *param)
{
	struct isp_device_t *dev = (struct isp_device_t *)param;

	if (dev == NULL) {
		pr_err("isp_kernel_thread, dev is NULL\n");
		return -1;
	}
	dev->lnc_load.load_lnc_flag = 0;
	dev->lnc_load.cap_eof_flag = 0;
	dev->is_kernel_thread_stop = 0;
	sema_init(&dev->kernel_sem, 0);
	sema_init(&dev->lnc_load.load_done_sem, 0);
	dev->kernel_thread = kthread_run(isp_kernel_proc,
					 param, "isp_kernel_proc_thread");
	if (IS_ERR(dev->kernel_thread)) {
		pr_err("isp_kernel_thread error\n");
		return -1;
	}
	return 0;
}

static int isp_stop_kernel_thread(void *param)
{
	struct isp_device_t *dev = (struct isp_device_t *)param;
	int cnt = 0;

	if (dev == NULL) {
		pr_err("dev is null\n");
		return -1;
	}
	if (dev->kernel_thread) {
		dev->is_kernel_thread_stop = 1;
		up(&dev->kernel_sem);
		if (0 != dev->is_kernel_thread_stop) {
			while (cnt < 500) {
				cnt++;
				if (0 == dev->is_kernel_thread_stop)
					break;
				msleep(1);
			}
		}
		dev->kernel_thread = NULL;
	}
	return 0;
}

static int32_t isp_lnc_param_load_ex(struct isp_reg_bits *reg_bits_ptr,
				     uint32_t counts)
{
	int32_t ret = 0;
	int32_t time_out_cnt = 0;
	volatile uint32_t reg_value = 0x00;

	if ((0x00 != isp_alloc_addr)
	    && (0x00 != isp_alloc_len)) {
#ifndef CONFIG_64BIT
		void *ptr = (void *)isp_alloc_addr;
		uint32_t len = isp_alloc_len;
		dmac_flush_range(ptr, ptr + len);
		outer_flush_range(__pa(ptr), __pa(ptr) + len);
#endif

		reg_bits_ptr->reg_value =
		    (uint32_t) __pa(reg_bits_ptr->reg_value);

		reg_value = ISP_READL(ISP_LNC_STATUS);
		while ((0x00 == (reg_value & ISP_LNC_STATUS_OK)) &&
		       (time_out_cnt < (ISP_TIME_OUT_MAX * 1000))) {
			udelay(1);
			reg_value = ISP_READL(ISP_LNC_STATUS);
			time_out_cnt++;
		}
		if (time_out_cnt >= (ISP_TIME_OUT_MAX * 1000)) {
			ret = -1;
			pr_err("isp lnc status time out\n");
		}
		ISP_OWR(ISP_INT_EN, ISP_INT_LENS_LOAD);
		write_reg(reg_bits_ptr, counts);
	} else {
		pr_err("isp load lnc param error\n");
	}
	return ret;
}

static int32_t isp_lnc_param_load(struct isp_reg_bits *reg_bits_ptr,
				  uint32_t counts)
{
	int32_t ret = 0;
	int32_t time_out_cnt = 0;
	uint32_t reg_value = 0x00;

	pr_debug("enter\n");
	if ((0x00 != isp_alloc_addr) && (0x00 != isp_alloc_len)) {
#ifndef CONFIG_64BIT
		void *ptr = (void *)isp_alloc_addr;
		uint32_t len = isp_alloc_len;
		dmac_flush_range(ptr, ptr + len);
		outer_flush_range(__pa(ptr), __pa(ptr) + len);
#endif

		reg_bits_ptr->reg_value =
		    (unsigned long)__pa(reg_bits_ptr->reg_value);

		{
			reg_value = ISP_READL(ISP_LNC_STATUS);
			while ((0x00 == (reg_value & ISP_LNC_STATUS_OK)) &&
			       (time_out_cnt < (ISP_TIME_OUT_MAX * 1000))) {
				udelay(1);
				reg_value = ISP_READL(ISP_LNC_STATUS);
				time_out_cnt++;
			}
			if (time_out_cnt >= (ISP_TIME_OUT_MAX * 1000)) {
				ret = -1;
				pr_err("lnc status time out\n");
			}
			write_reg(reg_bits_ptr, counts);
			pr_debug("load, %p: 0x%lx, 0x%lx, counts %d\n",
				 reg_bits_ptr, reg_bits_ptr->reg_addr,
				 reg_bits_ptr->reg_value, counts);
		}

		time_out_cnt = 0;
		reg_value = ISP_READL(ISP_INT_RAW);

		while ((0x00 == (reg_value & ISP_INT_LENS_LOAD)) &&
		       (time_out_cnt < ISP_TIME_OUT_MAX)) {
			udelay(1);
			reg_value = ISP_READL(ISP_INT_RAW);
			time_out_cnt++;
		}
		if (time_out_cnt >= ISP_TIME_OUT_MAX) {
			ret = -1;
			pr_err("load lnc param time out\n");
		}
		ISP_OWR(ISP_INT_CLEAR, ISP_INT_LENS_LOAD);
	} else {
		pr_err("load lnc param error\n");
	}

	return ret;
}

static int32_t isp_lnc_param_set(uint32_t * addr, uint32_t len)
{
	int32_t ret = 0;

	if ((0x00 != isp_alloc_addr) && (0x00 != addr)) {
		memcpy((void *)isp_alloc_addr, (void *)addr, len);
	}

	return ret;
}

static int32_t isp_free(void)
{
	int32_t ret = 0;

	if ((0x00 != isp_alloc_addr) && (0x00 != isp_alloc_order)) {
		free_pages(isp_alloc_addr, isp_alloc_order);
		isp_alloc_order = 0x00;
		isp_alloc_addr = 0x00;
		isp_alloc_len = 0x00;
	}

	return ret;
}

static int32_t isp_alloc(unsigned long *addr, uint32_t len)
{
	int32_t ret = 0x00;
	unsigned long buf = 0x00;
	void *ptr = 0x00;

	if ((0x00 != isp_alloc_addr) && (len <= isp_alloc_len)) {
		*addr = isp_alloc_addr;
	} else {
		isp_free();
	}

	if (0x00 == isp_alloc_addr) {
		isp_alloc_len = len;
		isp_alloc_order = get_order(len);
		isp_alloc_addr =
		    __get_free_pages(GFP_KERNEL | __GFP_COMP, isp_alloc_order);
		if (NULL == (void *)isp_alloc_addr) {
			pr_err("order:0x%x, addr:0x%lx, len:0x%x err\n",
			       isp_alloc_order, isp_alloc_addr, isp_alloc_len);
			return -1;
		}
		ptr = (void *)isp_alloc_addr;
		*addr = isp_alloc_addr;
		buf = virt_to_phys((volatile void *)isp_alloc_addr);

#ifndef CONFIG_64BIT
		dmac_flush_range(ptr, ptr + len);
		outer_flush_range(__pa(ptr), __pa(ptr) + len);
#endif
	}

	return ret;
}

static int isp_queue_init(struct isp_queue *queue)
{
	if (NULL == queue)
		return -EINVAL;

	memset(queue, 0x00, sizeof(*queue));
	queue->write = &queue->node[0];
	queue->read = &queue->node[0];

	return 0;
}

static int isp_queue_write(struct isp_queue *queue, struct isp_node *node)
{
	struct isp_node *ori_node;

	if (NULL == queue || NULL == node)
		return -EINVAL;

	ori_node = queue->write;

	*queue->write++ = *node;
	if (queue->write > &queue->node[ISP_QUEUE_LENGTH - 1]) {
		queue->write = &queue->node[0];
	}

	if (queue->write == queue->read) {
		queue->write = ori_node;
	}

	return 0;
}

static int isp_queue_read(struct isp_queue *queue, struct isp_node *node)
{
	int ret = 0;

	if (NULL == queue || NULL == node)
		return -EINVAL;
	if (queue->read != queue->write) {
		*node = *queue->read++;
		if (queue->read > &queue->node[ISP_QUEUE_LENGTH - 1]) {
			queue->read = &queue->node[0];
		}
	}
	return ret;
}

static inline void isp_regread(char *dst, char *src, size_t n)
{
	char tmp = 0;
	uint32_t tmp2 = 0;
	char *char_src = 0, *d = 0;
	uint32_t *d2 = (uint32_t *) dst;
	uint32_t *int_src = (uint32_t *) src;
	uint32_t counts = (n >> 2) << 2;
	uint32_t res_counts = n - counts;
	counts = counts >> 2;

	while (counts--) {
		tmp2 = ISP_READL(int_src);
		*d2++ = tmp2;
		int_src++;
	}

	if (res_counts) {
		d = (char *)d2;
		char_src = (char *)int_src;
		while (res_counts--) {
			tmp = __raw_readb(char_src);
			*d = tmp;
			char_src++;
		}
	}
}

static inline void isp_regwrite(char *dst, char *src, size_t n)
{
	uint32_t tmp2 = 0;
	char *char_src = 0, *d = 0;
	uint32_t *int_src = 0, *d2 = 0;
	uint32_t counts = 0, res_counts = 0;

	int_src = (uint32_t *) src;
	d2 = (uint32_t *) dst;
	counts = (n >> 2) << 2;
	res_counts = n - counts;
	counts = counts >> 2;

	while (counts--) {
		tmp2 = *int_src++;
		ISP_WRITEL(d2, tmp2);
		d2++;
	}

	if (res_counts) {
		d = (char *)d2;
		char_src = (char *)int_src;

		while (res_counts--) {
			tmp2 = *char_src++;
			writeb_relaxed(tmp2, d);
			d++;
		}
	}

}

static int32_t isp_get_ctlr(void *param)
{
	struct isp_device_t *dev_ptr = (struct isp_device_t *)param;

	down(&dev_ptr->sem_isp);

	return 0;
}

static int32_t isp_put_ctlr(void *param)
{
	struct isp_device_t *dev_ptr = (struct isp_device_t *)param;

	up(&dev_ptr->sem_isp);

	return 0;
}

static int isp_en_irq(unsigned long int_num)
{
	uint32_t ret = 0;

	ISP_WRITEL(ISP_INT_CLEAR, ISP_IRQ_HW_MASK);
	ISP_WRITEL(ISP_INT_EN, int_num);

	return ret;
}

static int isp_registerirq(void)
{
	uint32_t ret = 0;

	ret = request_irq(ISP_IRQ, isp_irq_root, IRQF_SHARED, "ISP", &isp_dev);

	return ret;
}

static void isp_unregisterirq(void)
{
	free_irq(ISP_IRQ, &isp_dev);
}

static int isp_cfg_dcam_int(uint32_t param)
{
	uint32_t ret = 0;

	dcam_int_eb = param;

	return ret;
}

static void read_reg(struct isp_reg_bits *reg_bits_ptr, uint32_t counts)
{
	uint32_t i = 0;
	unsigned long reg_val = 0;
	unsigned long reg_addr = 0;

	for (i = 0; i < counts; ++i) {
		reg_addr = ISP_BASE_ADDR + reg_bits_ptr[i].reg_addr;
		reg_val = ISP_READL(reg_addr);
		reg_bits_ptr[i].reg_value = reg_val;
	}
}

static void write_reg(struct isp_reg_bits *reg_bits_ptr, uint32_t counts)
{
	uint32_t i = 0;
	unsigned long reg_val = 0;
	unsigned long reg_addr = 0;

	for (i = 0; i < counts; ++i) {
		reg_addr = reg_bits_ptr[i].reg_addr + ISP_BASE_ADDR;
		reg_val = reg_bits_ptr[i].reg_value;
		ISP_WRITEL(reg_addr, reg_val);
	}
}

static int32_t isp_kernel_open(struct inode *node, struct file *pf)
{
	int32_t ret = 0;
	pr_debug("enter\n");

	ISP_CHECK_ZERO(isp_dev_ptr);

	ret = isp_get_ctlr(isp_dev_ptr);
	if (unlikely(ret)) {
		pr_err("get control error\n");
		ret = -EFAULT;
		return ret;
	}

	isp_dev_ptr->reg_base_addr = (unsigned long)ISP_BASE_ADDR;
	isp_dev_ptr->size = ISP_REG_MAX_SIZE;

	ret = isp_queue_init(&(isp_dev_ptr->queue));
	if (unlikely(0 != ret)) {
		pr_err("isp_queue_init failed\n");
		ret = -EIO;
		goto isp_open_err;
	}

	ret = isp_module_eb();
	if (unlikely(0 != ret)) {
		pr_err("isp_module_eb failed\n");
		ret = -EIO;
		goto isp_open_err;
	}

	ret = isp_module_rst();
	if (unlikely(0 != ret)) {
		pr_err("isp_module_rst failed\n");
		ret = -EIO;
		isp_module_dis();
		goto isp_open_err;
	}

	ret = isp_kernel_thread(isp_dev_ptr);
	if (unlikely(0 != ret)) {
		pr_err("isp_kernel_thread failed\n");
		ret = -EIO;
		isp_module_dis();
		goto isp_open_err;
	}

	ret = isp_registerirq();
	if (unlikely(0 != ret)) {
		pr_err("isp_registerirq failed\n");
		ret = -EIO;
		isp_stop_kernel_thread(isp_dev_ptr);
		isp_module_dis();
		goto isp_open_err;
	}

	pr_debug("exit\n");
	return ret;

isp_open_err:
	ret = isp_put_ctlr(isp_dev_ptr);
	if (unlikely(ret)) {
		pr_err("isp_put_ctlr failed\n");
		ret = -EFAULT;
	}
	pr_err("open error\n");
	return -EIO;
}

static irqreturn_t isp_irq_root(int irq, void *dev_id)
{
	int32_t ret = 0;
	uint32_t status = 0;
	uint32_t irq_line = 0;
	uint32_t irq_status = 0;
	unsigned long flag = 0;
	int32_t i = 0;
	struct isp_node node = { 0 };
	struct timeval system_time;

	status = ISP_REG_RD(ISP_INT_STATUS);
	irq_line = status & ISP_IRQ_HW_MASK;
	pr_debug("isp irq: 0x%x\n", irq_line);
	if (0 == irq_line) {
		return IRQ_NONE;
	}

	spin_lock_irqsave(&isp_spin_lock, flag);

	if (ISP_INT_FETCH_EOF & irq_line) {
		isp_dev_ptr->lnc_load.cap_eof_flag = 1;
		if (isp_dev_ptr->lnc_load.load_lnc_flag) {
			up(&isp_dev_ptr->kernel_sem);
			isp_dev_ptr->lnc_load.load_lnc_flag = 0;
		}
	}

	if ((ISP_INT_LENS_LOAD & irq_line) == ISP_INT_LENS_LOAD) {
		irq_line &= ~ISP_INT_LENS_LOAD;
		ISP_OWR(ISP_INT_CLEAR, ISP_INT_LENS_LOAD);
		up(&isp_dev_ptr->lnc_load.load_done_sem);
		if (!irq_line) {
			spin_unlock_irqrestore(&isp_spin_lock, flag);
			return IRQ_HANDLED;
		}
	}

	for (i = ISP_IRQ_NUM - 1; i >= 0; i--) {
		if (irq_line & (1 << (uint32_t) i)) {
			irq_status |= 1 << (uint32_t) i;
		}
		/* clear the interrupt flag */
		irq_line &= ~(uint32_t) (1 << (uint32_t) i);
		/* no interrupt source left */
		if (!irq_line)
			break;
	}

	ISP_WRITEL(ISP_INT_CLEAR, irq_status);
	node.isp_irq_val = irq_status;
	isp_get_systemtime(&system_time);
	system_time.tv_sec = system_time.tv_sec * 1000000;
	node.system_time =
	    (uint64_t) (system_time.tv_sec + system_time.tv_usec);
	ret =
	    isp_queue_write((struct isp_queue *)&isp_dev_ptr->queue,
			    (struct isp_node *)&node);
	spin_unlock_irqrestore(&isp_spin_lock, flag);
	up(&isp_dev_ptr->sem_isr);

	return IRQ_HANDLED;
}

void dcam_isp_root(void)
{
	int32_t ret = 0;
	unsigned long flag = 0;
	struct isp_node node = { 0 };
	struct timeval system_time;

	pr_debug("s_dcam_int_eb = %d\n", dcam_int_eb);
	if (0x00 != dcam_int_eb) {
		spin_lock_irqsave(&isp_spin_lock, flag);
		/* remove CONFIG_MACH_CORE3, see original isp_drv_kernel.cpp */
		node.dcam_irq_val = ISP_INT_FETCH_SOF;
		pr_debug("node.dcam_irq_val = 0x%x\n", node.dcam_irq_val);
		isp_get_systemtime(&system_time);
		system_time.tv_sec = system_time.tv_sec * 1000000;
		node.system_time =
		    (uint64_t) (system_time.tv_sec + system_time.tv_usec);
		ret =
		    isp_queue_write((struct isp_queue *)&isp_dev_ptr->queue,
				    (struct isp_node *)&node);
		spin_unlock_irqrestore(&isp_spin_lock, flag);
		up(&isp_dev_ptr->sem_isr);
	}
}

static int32_t isp_kernel_release(struct inode *node, struct file *pf)
{
	int ret = 0;

	pr_debug("enter\n");
	mutex_lock(&isp_lock);

	isp_unregisterirq();
	ret = isp_module_dis();

	mutex_unlock(&isp_lock);

	isp_stop_kernel_thread(isp_dev_ptr);
	ISP_CHECK_ZERO(isp_dev_ptr);
	ret = isp_put_ctlr(isp_dev_ptr);
	if (unlikely(ret)) {
		pr_err("isp_put_ctlr failed\n");
		return -EFAULT;
	}
	pr_debug("exit\n");
	return ret;
}

static long isp_kernel_ioctl(struct file *fl, unsigned int cmd,
			     unsigned long param)
{
	long ret = 0;
	unsigned long rtn = 0;
	uint32_t *addr = 0;
	unsigned long int_param;
	unsigned long flag = 0;
	unsigned long int_num;
	uint32_t buf_size = 0;
	uint32_t isp_irq_val, dcam_irq_val;
	struct isp_irq_param irq_param = { 0 };
	struct isp_node isp_node = { 0 };
	struct isp_reg_param reg_param = { 0 };
	struct isp_reg_bits *reg_bits_ptr = 0;
	struct isp_node node = { 0 };

	pr_debug("cmd = %x\n", cmd);

	if (!fl) {
		return -EINVAL;
	}

	ISP_CHECK_ZERO(isp_dev_ptr);

	if (ISP_IO_IRQ == cmd) {
		ret = down_interruptible(&isp_dev_ptr->sem_isr);
		if (ret) {
			pr_err("down_interruptible failed, ret = %ld\n", ret);
			memset(&irq_param, 0, sizeof(irq_param));
			irq_param.ret_val = ret;
			rtn = copy_to_user((void *)param,
					   (void *)&irq_param,
					   sizeof(irq_param));
			if (0 != rtn) {
				pr_err("rtn = %ld\n", rtn);
			}
			ret = -ERESTARTSYS;
			goto isp_ioctl_exit;
		}

		ret = isp_queue_read(&isp_dev_ptr->queue, &isp_node);
		if (0 != ret) {
			pr_err("isp_queue_read failed, ret = 0x%x\n",
			       (uint32_t) ret);
			ret = -EFAULT;
			goto isp_ioctl_exit;
		}

		irq_param.dcam_irq_val = isp_node.dcam_irq_val;
		irq_param.isp_irq_val = isp_node.isp_irq_val;
		isp_irq_val = isp_node.isp_irq_val;
		dcam_irq_val = isp_node.dcam_irq_val;
		irq_param.irq_val = dcam_irq_val | isp_irq_val;
		irq_param.system_time = isp_node.system_time;
		rtn = copy_to_user((void *)param,
				   (void *)&irq_param,
				   sizeof(struct isp_irq_param));
		if (0 != rtn) {
			pr_err("rtn = %ld\n", rtn);
			ret = -EFAULT;
		}
		return ret;
	}

	mutex_lock(&isp_lock);
	switch (cmd) {
	case ISP_IO_READ:
		rtn = copy_from_user((void *)&reg_param,
				     (void *)param,
				     sizeof(struct isp_reg_param));
		if (0 != rtn) {
			pr_err("rtn = %ld\n", rtn);
			ret = -EFAULT;
			goto io_read_exit;
		}

		buf_size = reg_param.counts * sizeof(struct isp_reg_bits);
		if (buf_size > isp_dev_ptr->buf_len) {
			ret = -EFAULT;
			goto io_read_exit;
		}

		reg_bits_ptr = (struct isp_reg_bits *)isp_dev_ptr->buf_addr;
		rtn = copy_from_user((void *)reg_bits_ptr,
				     (void *)reg_param.reg_param, buf_size);
		if (0 != rtn) {
			pr_err("rtn = %ld\n", rtn);
			ret = -EFAULT;
			goto io_read_exit;
		}
		read_reg(reg_bits_ptr, reg_param.counts);

		rtn = copy_to_user((void *)reg_param.reg_param,
				   (void *)reg_bits_ptr, buf_size);
		if (0 != rtn) {
			pr_err("rtn = %ld\n", rtn);
			ret = -EFAULT;
			goto io_read_exit;
		}

io_read_exit:
		if (reg_bits_ptr) {
			memset((void *)isp_dev_ptr->buf_addr, 0x00, buf_size);
			reg_bits_ptr = NULL;
		}
		break;

	case ISP_IO_WRITE:
		rtn = copy_from_user((void *)&reg_param,
				     (void *)param,
				     sizeof(struct isp_reg_param));
		if (0 != rtn) {
			pr_err("rtn = %ld\n", rtn);
			ret = -EFAULT;
			goto io_write_exit;
		}

		buf_size = reg_param.counts * sizeof(struct isp_reg_bits);
		if (buf_size > isp_dev_ptr->buf_len) {
			ret = -EFAULT;
			goto io_write_exit;
		}
		reg_bits_ptr = (struct isp_reg_bits *)isp_dev_ptr->buf_addr;

		rtn = copy_from_user((void *)reg_bits_ptr,
				     (void *)reg_param.reg_param, buf_size);
		if (0 != rtn) {
			pr_err("rtn = %ld\n", rtn);
			ret = -EFAULT;
			goto io_write_exit;
		}
		write_reg(reg_bits_ptr, reg_param.counts);

io_write_exit:
		if (reg_bits_ptr) {
			memset((void *)isp_dev_ptr->buf_addr, 0x00, buf_size);
			reg_bits_ptr = NULL;
		}
		break;

	case ISP_IO_RST:
		isp_dev_ptr->lnc_load.cap_eof_flag = 0;
		ret = isp_module_rst();
		if (ret) {
			pr_err("isp_module_rst failed\n");
			ret = -EFAULT;
		}
		break;

	case ISP_IO_SETCLK:
		pr_info("ISP_IO_SETCLK\n");
		break;

	case ISP_IO_STOP:
		/* dis-enable the interrupt */
		ret = isp_en_irq(0);
		spin_lock_irqsave(&isp_spin_lock, flag);
		node.dcam_irq_val = ISP_INT_STOP;
		ret = isp_queue_write((struct isp_queue *)
				      &isp_dev_ptr->queue,
				      (struct isp_node *)&node);
		if (ret) {
			pr_err("isp_queue_write failed, ret = %ld\n", ret);
			return ret;
		}
		spin_unlock_irqrestore(&isp_spin_lock, flag);
		up(&isp_dev_ptr->sem_isr);
		break;

	case ISP_IO_INT:
		rtn = copy_from_user((void *)&int_num, (void *)param, 0x04);
		if (rtn) {
			pr_err("rtn = %ld\n", rtn);
			ret = -EFAULT;
			goto isp_ioctl_locked_cmd_exit;
		}
		ret = isp_en_irq(int_num);
		if (unlikely(ret)) {
			pr_err("isp_en_irq failed\n");
			ret = -EFAULT;
		}
		break;

	case ISP_IO_DCAM_INT:
		rtn = copy_from_user((void *)&int_param, (void *)param, 0x04);
		if (rtn) {
			pr_err("rtn = %ld\n", rtn);
			ret = -EFAULT;
			goto isp_ioctl_locked_cmd_exit;
		}
		ret = isp_cfg_dcam_int(int_param);
		if (unlikely(ret)) {
			pr_err("ret =%ld\n", ret);
			ret = -EFAULT;
		}
		break;

	case ISP_IO_LNC_PARAM:
		rtn = copy_from_user((void *)&reg_param,
				     (void *)param,
				     sizeof(struct isp_reg_param));
		if (0 != rtn) {
			pr_err("rtn = %ld\n", rtn);
			ret = -EFAULT;
			goto io_lnc_param_exit;
		}

		buf_size = reg_param.counts;
		if (buf_size > isp_dev_ptr->buf_len) {
			ret = -EFAULT;
			goto io_lnc_param_exit;
		}

		addr = (uint32_t *) isp_dev_ptr->buf_addr;

		rtn = copy_from_user((void *)addr,
				     (void *)reg_param.reg_param, buf_size);
		if (0 != rtn) {
			pr_err("rtn = %ld\n", rtn);
			ret = -EFAULT;
			goto io_lnc_param_exit;
		}

		ret = isp_lnc_param_set(addr, buf_size);
		if (0 != ret) {
			pr_err("lnc_param_set, ret = %ld\n", ret);
			ret = -EFAULT;
			goto io_lnc_param_exit;
		}

io_lnc_param_exit:
		if (addr) {
			memset((void *)isp_dev_ptr->buf_addr, 0x00, buf_size);
			addr = NULL;
		}
		break;

	case ISP_IO_LNC:
		rtn = copy_from_user((void *)&reg_param,
				     (void *)param,
				     sizeof(struct isp_reg_param));
		if (0 != rtn) {
			pr_err("rtn = %ld\n", rtn);
			ret = -EFAULT;
			goto io_lnc_exit;
		}

		buf_size = reg_param.counts * sizeof(struct isp_reg_bits);
		if (buf_size > isp_dev_ptr->buf_len) {
			ret = -EFAULT;
			pr_err("isp_io_lnc: buf len failed\n");
			goto io_lnc_exit;
		}
		reg_bits_ptr = (struct isp_reg_bits *)isp_dev_ptr->buf_addr;

		rtn = copy_from_user((void *)reg_bits_ptr,
				     (void *)reg_param.reg_param, buf_size);
		if (0 != rtn) {
			pr_err("rtn = %ld\n", rtn);
			ret = -EFAULT;
			goto io_lnc_exit;
		}

		if (isp_dev_ptr->lnc_load.cap_eof_flag == 0) {
			ret = isp_lnc_param_load(reg_bits_ptr,
						 reg_param.counts);
		} else {
			memcpy((void *)isp_dev_ptr->lnc_load.load_buf,
			       (void *)reg_bits_ptr, buf_size);
			isp_dev_ptr->lnc_load.param_counts = reg_param.counts;
			isp_dev_ptr->lnc_load.load_lnc_flag = 1;
			ret =
			    down_timeout(&isp_dev_ptr->lnc_load.load_done_sem,
					 msecs_to_jiffies(500));
		}
		if (unlikely(ret)) {
			pr_err("load lnc error\n");
			ret = -EFAULT;
		}

io_lnc_exit:
		if (reg_bits_ptr) {
			memset((void *)isp_dev_ptr->buf_addr, 0x00, buf_size);
			reg_bits_ptr = NULL;
		}
		break;

	case ISP_IO_ALLOC:
		rtn = copy_from_user((void *)&reg_param,
				     (void *)param,
				     sizeof(struct isp_reg_param));
		if (rtn) {
			pr_err("rtn = %ld\n", rtn);
			ret = -EFAULT;
			break;
		}

		ret = isp_alloc(&reg_param.reg_param, reg_param.counts);
		if (ret) {
			pr_err("isp_alloc failed, ret = %ld\n", ret);
			return ret;
		}
		rtn = copy_to_user((void *)param,
				   (void *)&reg_param,
				   sizeof(struct isp_reg_param));
		if (rtn) {
			pr_err("rtn = %ld\n", rtn);
			ret = -EFAULT;
			return ret;
		}
		break;

	default:
		mutex_unlock(&isp_lock);
		pr_err("cmd is unsupported, cmd = %d\n", cmd);
		return -EFAULT;
	}

isp_ioctl_locked_cmd_exit:
	mutex_unlock(&isp_lock);
	pr_debug("ioctl finished\n");
isp_ioctl_exit:
	return ret;
}

#ifdef CONFIG_COMPAT
static int compat_get_rw_param(struct compat_isp_reg_param __user *data32,
			       struct isp_reg_param __user *data)
{
	int err = 0;
	compat_ulong_t addr;
	uint32_t cnt;

	err = get_user(addr, &data32->reg_param);
	err |= put_user(addr, &data->reg_param);
	err |= get_user(cnt, &data32->counts);
	err |= put_user(cnt, &data->counts);

	return err;
}

static int compat_get_rw_param_detail(struct isp_reg_param __user *data)
{
	struct compat_isp_reg_bits __user *data32;
	struct isp_reg_bits __user *reg_data;
	int err = 0;
	int i = 0;
	int cnt = data->counts;
	compat_ulong_t addr;
	compat_ulong_t val;
	struct isp_reg_bits __user *data_tmp;

	data32 = compat_ptr(data->reg_param);
	pr_debug("param: %p, cnt: %d\n", data32, cnt);
	reg_data = (struct isp_reg_bits __user *)((unsigned long)data +
						  sizeof(struct isp_reg_param));
	data->reg_param = (unsigned long)reg_data;
	if (reg_data == NULL) {
		pr_err("failed to compat_alloc_user_space\n");
		return -EFAULT;
	}
	pr_debug("param: data %p\n", reg_data);
	data_tmp = reg_data;
	for (i = 0; i < cnt; i++) {
		err = get_user(addr, &data32->reg_addr);
		err |= put_user(addr, &data_tmp->reg_addr);
		err |= get_user(val, &data32->reg_value);
		err |= put_user(val, &data_tmp->reg_value);
		if (err) {
			pr_err("failed cnt %d ret %d\n", cnt, err);
			break;
		}
		data32++;
		data_tmp++;
	}
	return err;
}

static int compat_put_read_param(struct compat_isp_reg_param __user *data32,
				 struct isp_reg_param __user *data)
{
	int err = 0;
	compat_ulong_t val;
	uint32_t cnt;
	int i = 0;
	struct compat_isp_reg_bits __user *reg_data32;
	struct isp_reg_bits __user *reg_data;

	cnt = data->counts;
	reg_data32 = (struct compat_isp_reg_bits __user *)((unsigned long)data32->reg_param);
	reg_data = (struct isp_reg_bits __user *)data->reg_param;
	for (i = 0; i < cnt; i++) {
		err = get_user(val, &reg_data->reg_value);
		err |= put_user(val, &reg_data32->reg_value);
		if (err) {
			pr_err("ret %d\n", err);
			break;
		}
		reg_data32++;
		reg_data++;
	}

	return err;
}

static int compat_get_alloc_param(struct compat_isp_reg_param __user *data32,
				  struct isp_reg_param __user *data)
{
	int err = 0;
	compat_ulong_t addr;
	uint32_t cnt;

	err = get_user(addr, &data32->reg_param);
	err |= put_user(addr, &data->reg_param);
	err |= get_user(cnt, &data32->counts);
	err |= put_user(cnt, &data->counts);

	return err;
}

static int compat_put_alloc(struct compat_isp_reg_param __user *data32,
			    struct isp_reg_param __user *data)
{
	int err = 0;
	compat_ulong_t addr;
	uint32_t cnt;

	err = get_user(addr, &data->reg_param);
	err |= put_user(addr, &data32->reg_param);
	err |= get_user(cnt, &data->counts);
	err |= put_user(cnt, &data32->counts);

	return err;
}

long compat_isp_kernel_ioctl(struct file *fl, unsigned int cmd,
			     unsigned long param)
{
	long ret = 0;
	int err = 0;
	void __user *up = compat_ptr(param);
	struct compat_isp_reg_param __user *data32;
	struct isp_reg_param __user *data;
	uint32_t cnt;

	if (ISP_IO_IRQ == cmd) {
		ret = fl->f_op->unlocked_ioctl(fl, cmd, (unsigned long)up);
		return ret;
	}

	switch (cmd) {
	case COMPAT_ISP_IO_READ:
		data32 = compat_ptr(param);
		err = get_user(cnt, &data32->counts);

		pr_debug("COMPAT_ISP_IO_READ param: %p\n", data32);
		data = compat_alloc_user_space(sizeof(struct isp_reg_param) +
					       cnt *
					       sizeof(struct isp_reg_bits));
		if (data == NULL) {
			pr_err("compat_alloc_user_space failed\n");
			return -EFAULT;
		}
		pr_debug("%p\n", data);
		err = compat_get_rw_param(data32, data);
		if (err) {
			return err;
		}
		err = compat_get_rw_param_detail(data);
		if (err) {
			pr_err("compat_get_rw_param_detail %d\n", err);
			return err;
		}
		err = fl->f_op->unlocked_ioctl(fl, ISP_IO_READ,
					       (unsigned long)data);
		err = compat_put_read_param(data32, data);
		ret = ret ? ret : err;
		break;

	case COMPAT_ISP_IO_WRITE:
		data32 = compat_ptr(param);
		err = get_user(cnt, &data32->counts);

		pr_debug("COMPAT_ISP_IO_WRITE param: %p\n", data32);
		data = compat_alloc_user_space(sizeof(struct isp_reg_param) +
					       cnt *
					       sizeof(struct isp_reg_bits));
		if (data == NULL) {
			pr_err("compat_alloc_user_space falied\n");
			return -EFAULT;
		}
		pr_debug("data = %p\n", data);
		err = compat_get_rw_param(data32, data);
		if (err) {
			pr_err("compat_get_rw_param failed\n");
			return err;
		}
		err = compat_get_rw_param_detail(data);
		if (err) {
			pr_err("compat_get_rw_param_detail failed %d\n", err);
			return err;
		}
		err = fl->f_op->unlocked_ioctl(fl, ISP_IO_WRITE,
					       (unsigned long)data);
		break;

	case COMPAT_ISP_IO_LNC_PARAM:
		data32 = compat_ptr(param);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL) {
			pr_err("compat_alloc_user_space failed\n");
			return -EFAULT;
		}
		err = compat_get_rw_param(data32, data);
		if (err) {
			pr_err("compat_get_rw_param failed\n");
			return err;
		}

		ret = fl->f_op->unlocked_ioctl(fl, ISP_IO_LNC_PARAM,
					       (unsigned long)
					       data);
		break;

	case COMPAT_ISP_IO_LNC:
		data32 = compat_ptr(param);
		err = get_user(cnt, &data32->counts);

		pr_debug("COMPAT_ISP_IO_LNC param: %p\n", data32);
		data = compat_alloc_user_space(sizeof
					       (struct isp_reg_param) +
					       cnt *
					       sizeof(struct isp_reg_bits));
		if (data == NULL) {
			pr_err("compat_alloc_user_space failed\n");
			return -EFAULT;
		}
		pr_debug("data = %p\n", data);
		err = compat_get_rw_param(data32, data);
		if (err) {
			return err;
		}
		err = compat_get_rw_param_detail(data);
		if (err) {
			pr_err("compat_get_rw_param_detail failed %d\n", err);
			return err;
		}
		err = fl->f_op->unlocked_ioctl(fl,
					       ISP_IO_LNC, (unsigned long)data);
		break;

	case COMPAT_ISP_IO_ALLOC:
		data32 = compat_ptr(param);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL) {
			pr_err("compat_alloc_user_space failed\n");
			return -EFAULT;
		}
		err = compat_get_alloc_param(data32, data);
		if (err) {
			pr_err("compat_get_alloc_param failed\n");
			return err;
		}
		ret = fl->f_op->unlocked_ioctl(fl,
					       ISP_IO_ALLOC,
					       (unsigned long)data);
		if (ret) {
			pr_err("unlocked_ioctl failed %ld\n", ret);
			goto compat_ioctl_exit;
		}
		err = compat_put_alloc(data32, data);
		ret = err;
		break;

	case ISP_IO_RST:
	case ISP_IO_SETCLK:
	case ISP_IO_STOP:
	case ISP_IO_INT:
	case ISP_IO_DCAM_INT:
		ret = fl->f_op->unlocked_ioctl(fl, cmd, (unsigned long)up);
		break;

	default:
		pr_info("don't support cmd 0x%x\n", cmd);
		break;
	}

compat_ioctl_exit:
	return ret;
}
#endif

static struct file_operations isp_fops = {
	.owner = THIS_MODULE,
	.open = isp_kernel_open,
	.unlocked_ioctl = isp_kernel_ioctl,
	.release = isp_kernel_release,
#ifdef CONFIG_COMPAT
	.compat_ioctl = compat_isp_kernel_ioctl,
#endif
};

static struct miscdevice isp_dev = {
	.minor = ISP_MINOR,
	.name = "sprd_isp",
	.fops = &isp_fops,
};

static int isp_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res = NULL;
	unsigned long addr = 0;

	pr_debug("enter\n");
	isp_dev_ptr = devm_kzalloc(&pdev->dev, sizeof(struct isp_device_t),
				   GFP_KERNEL);
	if (NULL == isp_dev_ptr) {
		pr_err("devm_kzalloc failed\n");
		return -ENOMEM;
	}
	init_MUTEX(&isp_dev_ptr->sem_isp);
	init_MUTEX_LOCKED(&isp_dev_ptr->sem_isr);
	isp_dev_ptr->s_isp_clk = NULL;
	isp_dev_ptr->s_isp_clk_mm_i = NULL;
	isp_dev_ptr->buf_addr = 0;
	isp_dev_ptr->buf_len = 0;
	isp_dev_ptr->buf_addr = (unsigned long)devm_kzalloc(&pdev->dev,
							    ISP_BUF_MAX_SIZE,
							    GFP_KERNEL);
	if (0 == isp_dev_ptr->buf_addr) {
		ret = -ENOMEM;
		pr_err("vzalloc failed\n");
		goto isp_probe_err;
	}
	isp_dev_ptr->buf_len = ISP_BUF_MAX_SIZE;
	isp_dev_ptr->lnc_load.load_buf =
	    (unsigned long)devm_kzalloc(&pdev->dev,
					ISP_BUF_MAX_SIZE, GFP_KERNEL);
	if (0 == isp_dev_ptr->lnc_load.load_buf) {
		ret = -ENOMEM;
		pr_err("vzalloc failed\n");
		goto isp_probe_err;
	}
	ret = isp_alloc(&addr, ISP_BUF_MAX_SIZE);
	if (ret) {
		ret = -ENOMEM;
		pr_err("isp_alloc failed\n");
		goto isp_probe_err;
	}

	ret = misc_register(&isp_dev);
	if (ret) {
		pr_err("cannot register miscdev on minor=%d, ret=%d\n",
		       (int32_t) ISP_MINOR, (int32_t) ret);
		return ret;
	}
	mutex_init(&isp_lock);

	/* get isp regbase */
	isp_dev.this_device->of_node = pdev->dev.of_node;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("platform_get_resource failed\n");
		return -1;
	}
	isp_reg_max_size = resource_size(res);
	isp_regbase = (unsigned long)devm_ioremap_resource(&pdev->dev, res);
	if (0 == isp_regbase) {
		pr_err("devm_ioremap_nocache failed\n");
		return -1;
	}
	pr_debug("isp register base addr: 0x%lx\n", isp_regbase);

	isp_irq = platform_get_irq(pdev, 0);;
	if (!isp_irq) {
		pr_err("irq_of_parse_and_map failed\n");
		return -1;
	}

	mmahb_gpr = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						    "sprd,sys-mm-ahb");
	if (IS_ERR(mmahb_gpr)) {
		pr_err("find mmahb failed\n");
	}

	pr_debug("exit\n");
	return 0;

isp_probe_err:
	isp_free();
	if (isp_dev_ptr->buf_addr) {
		devm_kfree(&pdev->dev, (void *)isp_dev_ptr->buf_addr);
		isp_dev_ptr->buf_addr = 0;
		isp_dev_ptr->buf_len = 0;
	}
	if (isp_dev_ptr->lnc_load.load_buf) {
		devm_kfree(&pdev->dev, (void *)isp_dev_ptr->lnc_load.load_buf);
		isp_dev_ptr->lnc_load.load_buf = 0;
	}
	if (isp_dev_ptr) {
		devm_kfree(&pdev->dev, isp_dev_ptr);
		isp_dev_ptr = NULL;
	}

	return ret;
}

static int isp_remove(struct platform_device *pdev)
{
	pr_debug("enter\n");
	misc_deregister(&isp_dev);
	if (isp_proc_file) {
		remove_proc_entry("driver/sprd_isp", NULL);
	}

	isp_free();
	if (isp_dev_ptr->buf_addr) {
		devm_kfree(&pdev->dev, (void *)isp_dev_ptr->buf_addr);
		isp_dev_ptr->buf_addr = 0;
		isp_dev_ptr->buf_len = 0;
	}
	if (isp_dev_ptr->lnc_load.load_buf) {
		devm_kfree(&pdev->dev, (void *)isp_dev_ptr->lnc_load.load_buf);
		isp_dev_ptr->lnc_load.load_buf = 0;
	}
	if (isp_dev_ptr) {
		devm_kfree(&pdev->dev, isp_dev_ptr);
		isp_dev_ptr = NULL;
	}

	mutex_destroy(&isp_lock);
	pr_debug("exit\n");
	return 0;
}

static const struct of_device_id of_match_table_isp[] = {
	{.compatible = "sprd,isp-r1p0",},
	{},
};

static struct platform_driver isp_driver = {
	.probe = isp_probe,
	.remove = isp_remove,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "sprd_isp",
		   .of_match_table = of_match_ptr(of_match_table_isp),
		   },
};

module_platform_driver(isp_driver);

MODULE_DESCRIPTION("Spreadtrum ISP Driver");
MODULE_LICENSE("GPL");

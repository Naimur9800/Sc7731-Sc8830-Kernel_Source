/*
 * Copyright (C) 2015-2016 Spreadtrum Communications Inc.
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
#define pr_fmt(fmt) "CPP,%s(%d): " fmt, __func__, __LINE__

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/sprd_iommu.h>
#include <linux/sprd_ion.h>
#include <linux/compat.h>

#include <video/sprd_cpp.h>

#include "../../common/cam_common.h"
#include "cam_pw_domain.h"

#include "cpp_reg.h"
#include "rot_drv.h"
#include "scale_drv.h"
#include "cpp_path2_drv.h"
#include "cpp_path3_drv.h"
#include "cpp_path0_path3_drv.h"
#include "cpp_path2_path3_drv.h"
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT

#include "../../dcam/isp_r6p10v2/isp_int.h"
#include "sprd_k_cpp.h"

#endif

#define cpp_reg_rd(iobase, reg) \
	readl_relaxed((iobase) + (reg))

#define cpp_reg_wr(iobase, reg, val) \
	writel_relaxed((val), ((iobase) + (reg)))

#define cpp_reg_and(iobase, reg, val) \
	writel_relaxed((readl_relaxed((iobase) + (reg)) & (val)), \
		       ((iobase) + (reg)))

#define cpp_reg_or(iobase, reg, val) \
	writel_relaxed((readl_relaxed((iobase) + (reg)) | (val)), \
		       ((iobase) + (reg)))

#define cpp_reg_mwr(io_base, reg, mask, val) \
	do { \
		unsigned int v = readl_relaxed((io_base) + (reg)); \
		v &= ~(mask); \
		writel_relaxed((v | ((mask) & (val))), \
			       ((io_base) + (reg))); \
	} while (0)

/* #define CPP_TEST_BY_RESERVED_MEMORY */ /* used for cpp FPGA test */

#define CPP_DEVICE_NAME                "sprd_cpp"
#define ROT_TIMEOUT                    200
#define SCALE_TIMEOUT                  200
#define PATH2_TIMEOUT                3000
#define PATH3_TIMEOUT               3000
#define ZSL_ZOOM_TIMEOUT               5000
#define RECORD_ZOOM_TIMEOUT            5000
#define CPP_IRQ_LINE_MASK              0xfffUL

#define CPP_PATH0_PROC_WIDTH           5120
#define CPP_PATH0_PROC_HEIGHT          4096
#define CPP_PATH2_PROC_WIDTH           1920
#define CPP_PATH2_PROC_HEIGHT          2160 /*2448*/
#define CPP_PATH3_PROC_WIDTH           1920 /*1920*/
#define CPP_PATH3_PROC_HEIGHT          1600 /*1080*/

#define CPP_TEST_SCALE_INPUT_WIDTH 640 /*4632 5312*/
#define CPP_TEST_SCALE_INPUT_HEIGHT 480 /*3480 3984*/
#define CPP_TEST_SCALE_OUTPUT_WIDTH 1280 /*2560*/
#define CPP_TEST_SCALE_OUTPUT_HEIGHT 960 /*1920*/

#define CPP_TEST_ROT_INPUT_WIDTH 640 /*640*/
#define CPP_TEST_ROT_INPUT_HEIGHT 480 /*480*/
#define CPP_TEST_ROT_OUTPUT_WIDTH CPP_TEST_ROT_INPUT_WIDTH
#define CPP_TEST_ROT_OUTPUT_HEIGHT CPP_TEST_ROT_INPUT_HEIGHT

#define CPP_TEST_PATH2_INPUT_WIDTH CPP_TEST_SCALE_INPUT_WIDTH /*640*/
#define CPP_TEST_PATH2_INPUT_HEIGHT CPP_TEST_SCALE_INPUT_HEIGHT /*480*/
#define CPP_TEST_PATH2_OUTPUT_WIDTH 1280 /*1280*/
#define CPP_TEST_PATH2_OUTPUT_HEIGHT 960 /*960*/

#define CPP_TEST_PATH3_INPUT_WIDTH  CPP_TEST_SCALE_INPUT_WIDTH /*1280*/
#define CPP_TEST_PATH3_INPUT_HEIGHT CPP_TEST_SCALE_INPUT_HEIGHT /*960*/
#define CPP_TEST_PATH3_OUTPUT_WIDTH 1920 /*CPP_TEST_SCALE_OUTPUT_WIDTH*/
#define CPP_TEST_PATH3_OUTPUT_HEIGHT 1080 /*CPP_TEST_SCALE_OUTPUT_HEIGHT*/

#ifdef CPP_TEST_BY_RESERVED_MEMORY
#define CPP_DUAL_CHANNEL_CAMERA_RESERVE_BASE 0x50000000 /*0xc64f0000*/
#define CPP_DUAL_CHANNEL_CAMERA_RESERVE_SIZE 0xFFFFFFF /*43MB, 0x380000 100M*/

#define CPP_DUAL_CHANNEL_CAMERA_RESERVE_IN CPP_DUAL_CHANNEL_CAMERA_RESERVE_BASE

#define CPP_DUAL_CHANNEL_CAMERA_RESERVE_OUT_SCALE \
	(CPP_DUAL_CHANNEL_CAMERA_RESERVE_IN + 0x2b00000)/*max 32M*/
#define CPP_DUAL_CHANNEL_CAMERA_RESERVE_OUT_ROT \
	(CPP_DUAL_CHANNEL_CAMERA_RESERVE_IN + 0x1000000)/*max 16M*/
#define CPP_DUAL_CHANNEL_CAMERA_RESERVE_OUT_PATH2 \
	(CPP_DUAL_CHANNEL_CAMERA_RESERVE_IN + 0x2b00000)/*max 32M*/
#define CPP_DUAL_CHANNEL_CAMERA_RESERVE_OUT_PATH3 \
	(CPP_DUAL_CHANNEL_CAMERA_RESERVE_OUT_SCALE + 0x2b00000)/*max 32M*/

#endif


enum cpp_irq_id {
	CPP_SCALE_DONE = 0,
	CPP_ROT_DONE,
	CPP_PATH2_DONE,
	CPP_PATH3_DONE,
	CPP_IRQ_NUMBER
};

struct rotif_device {
	atomic_t count;
	struct semaphore start_sem;
	struct completion done_com;
	struct rot_drv_private drv_priv;
};

struct scif_device {
	atomic_t count;
	struct semaphore start_sem;
	struct completion done_com;
	struct scale_drv_private drv_priv;
};

struct path2_if_device {
	atomic_t count;
	struct semaphore start_sem;
	struct completion done_com;
	struct cpp_path2_drv_private drv_priv;
};

struct path3_if_device {
	atomic_t count;
	struct semaphore start_sem;
	struct completion done_com;
	struct cpp_path3_drv_private drv_priv;
};

struct path0_path3_if_device {
	atomic_t count;
	struct scif_device *path0_if;
	struct path3_if_device *path3_if;
	struct cpp_path0_path3_drv_private drv_priv;
	/*int irq_collector;*/
	/*int irq_cur_path;*/
};

struct path2_path3_if_device {
	atomic_t count;
	struct path2_if_device *path2_if;
	struct path3_if_device *path3_if;
	struct cpp_path2_path3_drv_private drv_priv;
	/*int irq_collector;*/
	/*int irq_cur_path;*/
};


typedef void (*cpp_isr_func) (void *);

struct cpp_device {
	atomic_t pw_users;
	atomic_t open_users;
	struct mutex hw_lock;
	struct mutex lock;
	struct mutex dev_lock;
	spinlock_t slock;

	struct rotif_device *rotif;
	struct scif_device *scif;
	struct path2_if_device *path2_if;
	struct path3_if_device *path3_if;
	struct path0_path3_if_device *path0_path3_if;
	struct path2_path3_if_device *path2_path3_if;

	struct miscdevice md;
	int irq;
	cpp_isr_func isr_func[CPP_IRQ_NUMBER];
	void *isr_data[CPP_IRQ_NUMBER];

	void __iomem *io_base;

	struct platform_device *pdev;

	struct clk *cpp_clk;
	struct clk *cpp_clk_parent;
	struct clk *cpp_clk_default;
	struct clk *cpp_eb;
	struct clk *jpg_cpp_mtx_eb;
	struct clk *cpp_axi_eb;

	struct regmap *cam_ahb_gpr;
	enum sprd_cpp_current_senario cur_senario;
};

typedef void (*cpp_isr) (struct cpp_device *dev);

static struct cpp_device *sprd_cpp_dev;

#ifdef CPP_TEST_DRIVER
unsigned long cpp_test_image_in_alloc_virt_addr;
unsigned long cpp_test_image_out_alloc_virt_addr;
unsigned long cpp_test_image_out_alloc_virt_addr2;


unsigned long cpp_test_image_in_alloc_phy_addr;
unsigned long cpp_test_image_out_alloc_phy_addr;
unsigned long cpp_test_image_out_alloc_phy_addr2;


uint32_t cpp_test_image_in_alloc_order;
uint32_t cpp_test_image_out_alloc_order;

static unsigned long cpp_test_image_out_virt_path0;
static unsigned long cpp_test_image_out_virt_path2;
static unsigned long cpp_test_image_out_virt_path3;

#define CPP_TEST_IMAGE_IN_SIZE_ROT \
	(CPP_TEST_ROT_INPUT_WIDTH*CPP_TEST_ROT_INPUT_HEIGHT*2)
#define CPP_TEST_IMAGE_OUT_SIZE_ROT \
	(CPP_TEST_ROT_OUTPUT_WIDTH*CPP_TEST_ROT_OUTPUT_HEIGHT*2)

#define CPP_TEST_IMAGE_IN_SIZE_SCALE \
	(CPP_TEST_SCALE_INPUT_WIDTH*CPP_TEST_SCALE_INPUT_HEIGHT*2)
#define CPP_TEST_IMAGE_OUT_SIZE_SCALE \
	(CPP_TEST_SCALE_OUTPUT_WIDTH*CPP_TEST_SCALE_OUTPUT_HEIGHT*2)

#define CPP_TEST_IMAGE_IN_SIZE_PATH2 \
	(CPP_TEST_PATH2_INPUT_WIDTH*CPP_TEST_PATH2_INPUT_HEIGHT*2)
#define CPP_TEST_IMAGE_OUT_SIZE_PATH2 \
	(CPP_TEST_PATH2_OUTPUT_WIDTH*CPP_TEST_PATH2_OUTPUT_HEIGHT*2)

#define CPP_TEST_IMAGE_IN_SIZE_PATH3 \
	(CPP_TEST_PATH3_INPUT_WIDTH*CPP_TEST_PATH3_INPUT_HEIGHT*2)
#define CPP_TEST_IMAGE_OUT_SIZE_PATH3 \
	(CPP_TEST_PATH3_OUTPUT_WIDTH*CPP_TEST_PATH3_OUTPUT_HEIGHT*2)

#define CPP_TEST_IMAGE_IN_FILE_ROT_Y     "/system/xbin/ImageIn_y.yuv"
#define CPP_TEST_IMAGE_IN_FILE_ROT_UV	"/system/xbin/ImageIn_uv.yuv"
#define CPP_TEST_IMAGE_IN_FILE_SCALE   "/system/xbin/ImageIn_p0.yuv"
#define CPP_TEST_IMAGE_IN_FILE_PATH2   "/system/xbin/ImageIn_p2.yuv"
#define CPP_TEST_IMAGE_IN_FILE_PATH3   "/system/xbin/ImageIn_p3.yuv"


/*#define CPP_TEST_IMAGE_IN_FILE_SIZE     CPP_TEST_IMAGE_IN_SIZE*/
#define CPP_TEST_IMAGE_OUT_FILE_ROT     "/data/ImageOutP1.yuv"
#define CPP_TEST_IMAGE_OUT_FILE_SCALE     "/data/ImageOutP0.yuv"
#define CPP_TEST_IMAGE_OUT_FILE_PATH2     "/data/ImageOutP2.yuv"
#define CPP_TEST_IMAGE_OUT_FILE_PATH3     "/data/ImageOutP3.yuv"

#define CPP_TEST_IMAGE_OUT_FILE_BASE_ROT    "/data/ImageOutBaseP1.yuv"
#define CPP_TEST_IMAGE_OUT_FILE_BASE_SCALE     "/data/ImageOutBaseP0.yuv"
#define CPP_TEST_IMAGE_OUT_FILE_BASE_PATH2     "/data/ImageOutBaseP2.yuv"
#define CPP_TEST_IMAGE_OUT_FILE_BASE_PATH3     "/data/ImageOutBaseP3.yuv"
#endif

/* Internal Function Implementation */
void cpp_k_free_test_image(uint32_t *cpp_alloc_order, unsigned long *virt_addr)
{
/*	printk("cpp_k_free_test_image: order =0x%p, virt_addr = 0x%p\n",*/
		/**cpp_alloc_order, *virt_addr);*/

	if ((*virt_addr != 0x00) && (*cpp_alloc_order != 0x00)) {
		free_pages(*virt_addr, *cpp_alloc_order);

/*	printk("cpp_k_free_test_image succeed: */
/*order: 0x%p, virt_addr: 0x%p\n",*/
		/**cpp_alloc_order, *virt_addr);*/

		*cpp_alloc_order = 0x00;
		*virt_addr = 0x00;
	}

}

void cpp_k_alloc_test_image(uint32_t *cpp_alloc_order,
	unsigned long *virt_addr, unsigned long *phy_addr, uint32_t len)
{
	uint32_t cpp_alloc_len = 0x00;
	/*uint32_t cpp_alloc_order = 0x00;*/
	unsigned long cpp_alloc_addr = 0x00;
	void *ptr = 0x00;

	/* allocate physical memory for test image.*/
	cpp_alloc_len = len;
	*cpp_alloc_order = get_order(len);
	cpp_alloc_addr =
		__get_free_pages(GFP_KERNEL | __GFP_COMP, *cpp_alloc_order);
	if (NULL == (void *)cpp_alloc_addr) {
		/*pr_err("order:0x%p, addr:0x%p, len:0x%p err\n",*/
			/**cpp_alloc_order, cpp_alloc_addr, cpp_alloc_len);*/
		return;
	}
	ptr = (void *)cpp_alloc_addr;
	*virt_addr = cpp_alloc_addr;
	/*convert the virstual address to physical*/
	*phy_addr = virt_to_phys((void *)cpp_alloc_addr);

#ifndef CONFIG_64BIT
	/*dmac_flush_range(ptr, ptr + len);*/
	/*outer_flush_range(__pa(ptr), __pa(ptr) + len);*/
#endif

/*	printk("order: 0x%p, virt_addr: 0x%p, phy_addr: 0x%p, len: 0x%p\n",*/
		/**cpp_alloc_order, *virt_addr, *phy_addr, len);*/

}

#ifdef CPP_TEST_DRIVER
static inline void cpp_alloc_test_image_buffer(
		enum sprd_cpp_current_senario cur_senario,
		unsigned int sizeIn, unsigned int sizeOut1,
		unsigned int sizeOut2)
{
#ifdef CPP_TEST_BY_RESERVED_MEMORY
	cpp_test_image_in_alloc_phy_addr = CPP_DUAL_CHANNEL_CAMERA_RESERVE_IN;
	cpp_test_image_in_alloc_virt_addr =
		(unsigned long)phys_to_virt(cpp_test_image_in_alloc_phy_addr);
    /*(unsigned long)phys_to_virt(0x400000000|*/
			/*cpp_test_image_in_alloc_phy_addr);*/

	if (cur_senario == CPP_SENARIO_SCALE) {
		cpp_test_image_out_alloc_phy_addr =
			CPP_DUAL_CHANNEL_CAMERA_RESERVE_OUT_SCALE;
		cpp_test_image_out_alloc_virt_addr =
		(unsigned long)phys_to_virt(cpp_test_image_out_alloc_phy_addr);
	} else if (cur_senario == CPP_SENARIO_ROTATE) {
		cpp_test_image_out_alloc_phy_addr =
			CPP_DUAL_CHANNEL_CAMERA_RESERVE_OUT_ROT;
		cpp_test_image_out_alloc_virt_addr =
		(unsigned long)phys_to_virt(cpp_test_image_out_alloc_phy_addr);
	} else if (cur_senario == CPP_SENARIO_PATH2) {
		cpp_test_image_out_alloc_phy_addr =
			CPP_DUAL_CHANNEL_CAMERA_RESERVE_OUT_PATH2;
		cpp_test_image_out_alloc_virt_addr =
		(unsigned long)phys_to_virt(cpp_test_image_out_alloc_phy_addr);
	} else if (cur_senario == CPP_SENARIO_PATH3) {
		cpp_test_image_out_alloc_phy_addr =
			CPP_DUAL_CHANNEL_CAMERA_RESERVE_OUT_PATH3;
		cpp_test_image_out_alloc_virt_addr =
		(unsigned long)phys_to_virt(cpp_test_image_out_alloc_phy_addr);
	} else if (cur_senario == CPP_SENARIO_PATH0_PATH3) {
		cpp_test_image_out_alloc_phy_addr =
			CPP_DUAL_CHANNEL_CAMERA_RESERVE_OUT_SCALE;
		cpp_test_image_out_alloc_phy_addr2 =
			CPP_DUAL_CHANNEL_CAMERA_RESERVE_OUT_PATH3;

		cpp_test_image_out_alloc_virt_addr =
		(unsigned long)phys_to_virt(cpp_test_image_out_alloc_phy_addr);
		cpp_test_image_out_alloc_virt_addr2 =
		(unsigned long)phys_to_virt(cpp_test_image_out_alloc_phy_addr2);
	} else if (cur_senario == CPP_SENARIO_PATH2_PATH3) {
		cpp_test_image_out_alloc_phy_addr =
			CPP_DUAL_CHANNEL_CAMERA_RESERVE_OUT_PATH2;
		cpp_test_image_out_alloc_phy_addr2 =
			CPP_DUAL_CHANNEL_CAMERA_RESERVE_OUT_PATH3;

		cpp_test_image_out_alloc_virt_addr =
		(unsigned long)phys_to_virt(cpp_test_image_out_alloc_phy_addr);
		cpp_test_image_out_alloc_virt_addr2 =
		(unsigned long)phys_to_virt(cpp_test_image_out_alloc_phy_addr2);
	}
#else
	cpp_k_alloc_test_image(&cpp_test_image_in_alloc_order,
			&cpp_test_image_in_alloc_virt_addr,
			&cpp_test_image_in_alloc_phy_addr, sizeIn);

/*	printk("allocate test image: order: 0x%x, virt_addr: 0x%x.\n", \*/
		/*cpp_test_image_in_alloc_order,*/
		/*cpp_test_image_in_alloc_virt_addr);*/

	 cpp_k_alloc_test_image(&cpp_test_image_out_alloc_order,
		&cpp_test_image_out_alloc_virt_addr,
		&cpp_test_image_out_alloc_phy_addr, sizeOut1);

	 cpp_k_alloc_test_image(&cpp_test_image_out_alloc_order,
		&cpp_test_image_out_alloc_virt_addr2,
		&cpp_test_image_out_alloc_phy_addr2, sizeOut2);
#endif

	CPP_LOG("In buffer phy=0x%lx(0x%lx),size=%d(0x%x)",
		cpp_test_image_in_alloc_phy_addr,
		cpp_test_image_in_alloc_virt_addr,
		sizeIn, sizeIn);
	CPP_LOG("Out p=0x%lx(0x%lx), p2=0x%lx(0x%lx),Out1=%d,Out2=%d",
		cpp_test_image_out_alloc_phy_addr,
		cpp_test_image_out_alloc_virt_addr,
		cpp_test_image_out_alloc_phy_addr2,
		cpp_test_image_out_alloc_virt_addr2,
		sizeOut1, sizeOut2);
}

static inline void cpp_free_test_image_buffer(void)
{
#if !defined(CPP_TEST_BY_RESERVED_MEMORY)
	cpp_k_free_test_image(&cpp_test_image_in_alloc_order,
		&cpp_test_image_in_alloc_virt_addr);
	cpp_k_free_test_image(&cpp_test_image_out_alloc_order,
		&cpp_test_image_out_alloc_virt_addr);
	cpp_k_free_test_image(&cpp_test_image_out_alloc_order,
		&cpp_test_image_out_alloc_virt_addr2);
#endif
}

static inline void cpp_write_image_to_file(char  *buffer,
	unsigned int size, const char *file)
{
	struct file *wfp;
	/*mm_segment_t old_fs;*/
	int i = 0;
	int result = 0;
	/* Write imag to a file.*/
	CPP_LOG("write to image:%s, buf=0x%p, size=%d",
		file, buffer, size);
	wfp = filp_open(file, O_CREAT|O_RDWR, 0777);
	if (wfp != NULL) {
	do {
		i++;
		result = kernel_write(wfp, buffer, size, 0);
		CPP_LOG("cnt: %d, result: %d,size: %d\n",
			i, result, size);
		if (result > 0) {
			size -= result;
			buffer += result;
		}
	} while ((result > 0) && (size > 0));
	/*old_fs = get_fs();*/
	/*set_fs(get_ds());*/
	/*wfp->f_op->write(wfp, (char *)buffer, size, &wfp->f_pos);*/
	/*set_fs(old_fs);*/
	filp_close(wfp, NULL);
	} else {
		pr_err("can not open write file\n");
	}
}


static inline void cpp_load_image_and_copy(const char *fileIn1,
	const char *fileIn2, unsigned int sizeIn1, unsigned int sizeIn2,
	const char *fileOut1, const char *fileOut2,
	unsigned int sizeOut1, unsigned int sizeOut2)
{
	struct file *rfp;
	struct file *rfp1;
    /* mm_segment_t old_fs; */
    /* loff_t pos; */
	int result = 0;
	int i = 0;
	unsigned int size = 0;
	char *buffer = NULL;
	char *buffer_uv = NULL;

	buffer = (char *)cpp_test_image_in_alloc_virt_addr;
	buffer_uv = (char *)(cpp_test_image_in_alloc_virt_addr +
		CPP_TEST_ROT_INPUT_WIDTH*CPP_TEST_ROT_INPUT_HEIGHT);
{
	size = sizeIn1;
    /*Read an image file.*/
	rfp = filp_open(fileIn1, O_RDWR, 0777);
	if (IS_ERR(rfp))
		pr_err("read y failed: %ld\n", -PTR_ERR(rfp));
	if (!IS_ERR(rfp)) {
		CPP_LOG("load image:%s,In=%d,Out1=%d,Out2=%d",
				fileIn1, sizeIn1, sizeOut1, sizeOut2);
		do {
			i++;
			result = kernel_read(rfp, 0, buffer, size);
			CPP_LOG("y cnt: %d, result: %d, size: %d\n",
				i, result, size);
			if ((size-result) > 0) {
				size -= result;
				buffer += result;
			}
		} while ((size-result) > 0);
	}
}
	if (fileIn2 != NULL || sizeIn2 != 0) {
	size = sizeIn2;
	rfp1 = filp_open(fileIn2, O_RDWR, 0777);
	if (IS_ERR(rfp1))
		pr_err("read uv failed: %ld\n", -PTR_ERR(rfp1));
	if (!IS_ERR(rfp1)) {
		CPP_LOG("load image:%s,In=%d,Out1=%d,Out2=%d",
				fileIn2, sizeIn2, sizeOut1, sizeOut2);
		do {
			i++;
			result = kernel_read(rfp1, 0, buffer_uv, size);
			CPP_LOG("uv cnt: %d, result: %d, size: %d\n",
				i, result, size);
			if ((size-result) > 0) {
				size -= result;
				buffer_uv += result;
			}
		} while ((size-result) > 0);
	}
	}
#if 0
	result = kernel_read(rfp, 0,
			(char *)cpp_test_image_in_alloc_virt_addr, sizeIn);
	old_fs = get_fs();
		set_fs(get_ds());
		CPP_LOG("rfp:%p, virt_addr:%s, sizeIn:%d, rfp->pos:%p",
				rfp, (char *)cpp_test_image_in_alloc_virt_addr,
				sizeIn, &rfp->f_pos);
		rfp->f_op->read(rfp, (char *)cpp_test_image_in_alloc_virt_addr,
				sizeIn, &rfp->f_pos);
		pos = 0;
		vfs_read(rfp, (char *)cpp_test_image_in_alloc_virt_addr,
				sizeIn, &pos);
		set_fs(old_fs);
		filp_close(rfp, NULL);
		set_fs(old_fs);
#endif
	size = sizeIn1 + sizeIn2;
	if (fileOut1 && sizeOut1) {
		if (size <= sizeOut1) {
			CPP_LOG("copy image1 to %s", fileOut1);
			memcpy(
			(unsigned char *)cpp_test_image_out_alloc_virt_addr,
			(unsigned char *)cpp_test_image_in_alloc_virt_addr,
			size);
		} else {
			CPP_LOG("memset out buffer1 to 0xab!");
			memset((void *) cpp_test_image_out_alloc_virt_addr,
			0xab, sizeOut1);
		}
		/* Write imag to a file.*/
		cpp_write_image_to_file(
			(char *)cpp_test_image_out_alloc_virt_addr,
			sizeOut1, fileOut1);
	}

	if (fileOut2 && sizeOut2) {
		if (size <= sizeOut2) {
			CPP_LOG("copy image2 to %s", fileOut2);
			memcpy(
			(unsigned char *)cpp_test_image_out_alloc_virt_addr2,
			(unsigned char *)cpp_test_image_in_alloc_virt_addr,
			size);
		} else {
			CPP_LOG("memset out buffer2 to 0xab!");
			memset((void *) cpp_test_image_out_alloc_virt_addr2,
				0xab, sizeOut2);
	}
	/* Write imag to a file.*/
	cpp_write_image_to_file(
		(char *)cpp_test_image_out_alloc_virt_addr2,
		sizeOut2, fileOut2);
	}
}
#endif

static void cpp_scale_done(struct cpp_device *dev)
{
	cpp_isr_func user_func = dev->isr_func[CPP_SCALE_DONE];
	void *priv = dev->isr_data[CPP_SCALE_DONE];

/*	if (dev->cur_senario | CPP_SENARIO_PATH0_PATH3) {*/
		/*struct path0_path3_if_device *path0_path3_if =*/
			/*(struct path0_path3_if_device *) priv;*/
		/*path0_path3_if->irq_cur_path = CPP_SCALE_DONE;*/
	/*}*/
	if (user_func)
		(*user_func) (priv);
}

static void cpp_rot_done(struct cpp_device *dev)
{
	cpp_isr_func user_func = dev->isr_func[CPP_ROT_DONE];
	void *priv = dev->isr_data[CPP_ROT_DONE];

	if (user_func)
		(*user_func) (priv);
}

static void cpp_path2_done(struct cpp_device *dev)
{
	cpp_isr_func user_func = dev->isr_func[CPP_PATH2_DONE];
	void *priv = dev->isr_data[CPP_PATH2_DONE];

	if (user_func)
		(*user_func) (priv);
}

static void cpp_path3_done(struct cpp_device *dev)
{
	cpp_isr_func user_func = dev->isr_func[CPP_PATH3_DONE];
	void *priv = dev->isr_data[CPP_PATH3_DONE];

/*	if (dev->cur_senario | CPP_SENARIO_PATH0_PATH3) {*/
		/*struct path0_path3_if_device *path0_path3_if =*/
			/*(struct path0_path3_if_device *) priv;*/
		/*path0_path3_if->irq_cur_path = CPP_PATH3_DONE;*/
	/*}*/

	if (user_func)
		(*user_func) (priv);
}


static const cpp_isr cpp_isr_list[CPP_IRQ_NUMBER] = {
	cpp_scale_done,
	cpp_rot_done,
	cpp_path2_done,
	cpp_path3_done
};

static irqreturn_t cpp_isr_root(int irq, void *priv)
{

	int i;
	unsigned int irq_line, status;
	unsigned long flag;
	struct cpp_device *dev = (struct cpp_device *)priv;

	status = reg_rd(dev, CPP_INT_STS) & CPP_IRQ_LINE_MASK;

	/* CPP_LOG("enter, isr=0x%x", status); */

	if (unlikely(status == 0))
		return IRQ_NONE;

	irq_line = status;
	reg_wr(dev, CPP_INT_CLR, status);

	spin_lock_irqsave(&dev->slock, flag);
	for (i = 0; i < CPP_IRQ_NUMBER; i++) {
		if (irq_line & (1 << (unsigned int)i)) {
			if (cpp_isr_list[i])
				cpp_isr_list[i] (dev);
		}
		irq_line &= ~(unsigned int)(1 << (unsigned int)i);
		if (!irq_line)
			break;
	}
	spin_unlock_irqrestore(&dev->slock, flag);

	return IRQ_HANDLED;
}

static int cpp_module_enable(struct cpp_device *dev)
{
#ifndef CPP_HARDWARE_FPGA
	int ret = 0;

	CPP_LOG("enter");
	mutex_lock(&dev->lock);
	if (atomic_inc_return(&dev->pw_users) == 1) {
		/* enable clk */
		ret = sprd_cam_pw_on();
		if (ret != 0) {
			pr_err("%s : sprd cam_sys power on failed\n", __func__);
			goto fail;
		}
		CPP_LOG("uuuu enable cpp eb S");
		ret = clk_prepare_enable(dev->cpp_eb);
		if (ret)
			goto fail;

		CPP_LOG("uuuu enable jpg_cpp_mtx_eb S");
		ret = clk_prepare_enable(dev->jpg_cpp_mtx_eb);
		if (ret) {
			clk_disable_unprepare(dev->cpp_eb);
			goto fail;
		}

		CPP_LOG("uuuu enable cpp axi eb S");
		ret = clk_prepare_enable(dev->cpp_axi_eb);
		if (ret) {
			clk_disable_unprepare(dev->jpg_cpp_mtx_eb);
			clk_disable_unprepare(dev->cpp_eb);
			goto fail;
		}

		ret = clk_set_parent(dev->cpp_clk, dev->cpp_clk_parent);
		if (ret) {
			clk_disable_unprepare(dev->cpp_axi_eb);
			clk_disable_unprepare(dev->jpg_cpp_mtx_eb);
			clk_disable_unprepare(dev->cpp_eb);
			goto fail;
		}
		CPP_LOG("uuuu enable cpp clk S");
		ret = clk_prepare_enable(dev->cpp_clk);
		if (ret) {
			clk_disable_unprepare(dev->cpp_axi_eb);
			clk_disable_unprepare(dev->jpg_cpp_mtx_eb);
			clk_disable_unprepare(dev->cpp_eb);
			goto fail;
		}
		CPP_LOG("uuuu update ahb soft rst S");
		regmap_update_bits(dev->cam_ahb_gpr,
			   REG_CAM_AHB_AHB_RST,
			   BIT_CAM_AHB_CPP_SOFT_RST,
			   BIT_CAM_AHB_CPP_SOFT_RST);
		udelay(1);
		regmap_update_bits(dev->cam_ahb_gpr,
			   REG_CAM_AHB_AHB_RST,
			   BIT_CAM_AHB_CPP_SOFT_RST,
			   ~(unsigned int)BIT_CAM_AHB_CPP_SOFT_RST);
	} else
		CPP_LOG("cpp already powered on, do nothing\n");

	mutex_unlock(&dev->lock);

	return 0;
fail:
	mutex_unlock(&dev->lock);
	return ret;
#else
	int val = 0;

	pr_err("uuuu 0404 1107 enter FPGA test");
	mutex_lock(&dev->lock);
	{
		void __iomem *iomem_regs = NULL;

		iomem_regs = ioremap_nocache(0xe42b0234, 0x4);

		cpp_reg_or(iomem_regs, 0x00, BIT_1);
		val = cpp_reg_rd(iomem_regs, 0x00);
		pr_err("uuuu:<0xe42b0234>[0x%x]\n", val);

		iounmap(iomem_regs);
	}

	/*-------------cpp register init ------------------*/
	{
	  /* PMU APB */
	void __iomem *iomem_regs_pmu_base = NULL;

	iomem_regs_pmu_base = ioremap_nocache(0xe42b0000, 0x100);

	cpp_reg_and(iomem_regs_pmu_base, 0x88, ~(BIT_28));
	cpp_reg_and(iomem_regs_pmu_base, 0x88, ~(BIT_25));
	cpp_reg_and(iomem_regs_pmu_base, 0x88, ~(BIT_24));
	val = cpp_reg_rd(iomem_regs_pmu_base, 0x88);
	pr_err("uuuu:<0xe42b0088>[0x%x]\n", val);

	cpp_reg_and(iomem_regs_pmu_base, 0x94, ~(BIT_28));
	cpp_reg_and(iomem_regs_pmu_base, 0x94, ~(BIT_25));
	cpp_reg_and(iomem_regs_pmu_base, 0x94, ~(BIT_24));
	val = cpp_reg_rd(iomem_regs_pmu_base, 0x94);
	pr_err("uuuu:<0xe42b0094>[0x%x]\n", val);

	iounmap(iomem_regs_pmu_base);

	}
	{
		/* AP APB */
	void __iomem *iomem_regs_apb_base = NULL;

	iomem_regs_apb_base = ioremap_nocache(0xe7b00000, 0x4);

	/* cpp_reg_or(iomem_regs_apb_base, 0x00, BIT_19|BIT_8|BIT_12); */
	cpp_reg_or(iomem_regs_apb_base, 0x00, BIT_19);
	val = cpp_reg_rd(iomem_regs_apb_base, 0x00);
	pr_err("uuuu: <0xe7b00000>[0x%x]\n", val);

	iounmap(iomem_regs_apb_base);
	}
	{
		/* AON APB */
	void __iomem *iomem_regs = NULL;

	iomem_regs = ioremap_nocache(0xe42e0000, 0x4);

	cpp_reg_or(iomem_regs, 0x00,
			BIT_2|BIT_9|BIT_15|BIT_19|BIT_25|BIT_26|BIT_31);
	val = cpp_reg_rd(iomem_regs, 0x00);
	pr_err("uuuu:<0xe42e0000>[0x%x]\n", val);

	iounmap(iomem_regs);
	}
/*
*    {
*	void __iomem *iomem_regs= NULL;
*
*	iomem_regs= ioremap_nocache(0xe42e0000, 0x4);
*
*	cpp_reg_or(iomem_regs, 0x00,
*		BIT_25|BIT_26|BIT_31|BIT_3|BIT_2|BIT_9|BIT_15);
*	val = cpp_reg_rd(iomem_regs,0x00);
*	pr_err("vvv:<0xe42e0000>[0x%x]\n", val);
*
*	iounmap(iomem_regs);
*   }
*/
	{
	void __iomem *iomem_regs = NULL;

	iomem_regs = ioremap_nocache(0xe42e0000, 0x4);

	cpp_reg_or(iomem_regs, 0x04, BIT_29);
	val = cpp_reg_rd(iomem_regs, 0x00);
	pr_err("uuuu:<0xe42e0004>[0x%x]\n", val);

	iounmap(iomem_regs);
	}

	{
	void __iomem *iomem_regs = NULL;

	iomem_regs = ioremap_nocache(0xd3100000, 0x4);

	cpp_reg_or(iomem_regs, 0x00, BIT_0|BIT_1|BIT_2|BIT_3);
	val = cpp_reg_rd(iomem_regs, 0x00);
	pr_err("uuuu:<0xd3100000>[0x%x]\n", val);

	iounmap(iomem_regs);
	}

	{
	void __iomem *iomem_regs = NULL;

	iomem_regs = ioremap_nocache(0xd2100000, 0x4);

	cpp_reg_or(iomem_regs, 0x00, BIT_7|BIT_10|BIT_23|BIT_24);
	val = cpp_reg_rd(iomem_regs, 0x00);
	pr_err("uuuu:<0xd2100000>[0x%x]\n", val);

	iounmap(iomem_regs);
	}

	{
	void __iomem *iomem_regs = NULL;

	iomem_regs = ioremap_nocache(0xd2100008, 0x4);

	cpp_reg_or(iomem_regs, 0x00, BIT_13);
	val = cpp_reg_rd(iomem_regs, 0x00);
	pr_err("uuuu:<0xd2100008>[0x%x]\n", val);

	iounmap(iomem_regs);
	}

	{
	void __iomem *iomem_regs = NULL;

	iomem_regs = ioremap_nocache(0xd2100004, 0x4);
	cpp_reg_or(iomem_regs, 0x00, BIT_22);
	cpp_reg_and(iomem_regs, 0x00, ~BIT_22);
	val = cpp_reg_rd(iomem_regs, 0x00);
	pr_err("uuuu:<0xd2100004>[0x%x]\n", val);

	iounmap(iomem_regs);
	}

	{
	void __iomem *iomem_regs = NULL;

	iomem_regs = ioremap_nocache(0xe42b02cc, 0x4);

	cpp_reg_or(iomem_regs, 0x00, BIT_0);
	val = cpp_reg_rd(iomem_regs, 0x00);
	pr_err("uuuu:<0xe42b02cc>[0x%x]\n", val);

	iounmap(iomem_regs);
	}

	{
	/* d3000000 camera clk */
	void __iomem *iomem_regs = NULL;

	iomem_regs = ioremap_nocache(0xd3000068, 0x4);

	cpp_reg_or(iomem_regs, 0x00, BIT_0);
	val = cpp_reg_rd(iomem_regs, 0x00);
	pr_err("uuuu:<0xd3000068>[0x%x]\n", val);

	iounmap(iomem_regs);
	}

/* cpp register init end-- */
#ifdef CPP_HARDWARE_FPGA_CLK
	/* clk cpp_eb,0xd2100000:CAM_SYS AHB register */
	clk_base = ioremap_nocache(0xd2100000, 0x100);
	cpp_clk_cfg = clk_base + 0x0;
	val = readl_relaxed(cpp_clk_cfg);
	val |= (unsigned int)(1<<10);/* bit_10:cpp_clk */
	writel_relaxed(val, cpp_clk_cfg);

	/*clk cpp_axi_eb*/
		cpp_clk_cfg = clk_base + 0x8;
		val = readl_relaxed(cpp_clk_cfg);
		val |= (unsigned int)(1<<13);/* bit_13 CPP_AXI_CKG_EN */
		writel_relaxed(val, cpp_clk_cfg);

		iounmap((void *)clk_base);
#else

/*cpp soft reset
*
*		regmap_update_bits(dev->cam_ahb_gpr,
*				   0xd2100004,
*				   1<<22,
*				   1<<22);
*		udelay(1);
*		regmap_update_bits(dev->cam_ahb_gpr,
*				   0xd2100004,
*				   1<<22,
*				   0<<22);
*/
#endif
	mutex_unlock(&dev->lock);

	return 0;
#endif /* CPP_HARDWARE_FPGA */
}

static int cpp_module_disable(struct cpp_device *dev)
{
	int ret = 0;
#ifndef CPP_HARDWARE_FPGA

	CPP_LOG("enter");
	mutex_lock(&dev->lock);
	if (atomic_dec_return(&dev->pw_users) == 0) {
		/* disable clk */
		regmap_update_bits(dev->cam_ahb_gpr,
				   REG_CAM_AHB_AHB_RST,
				   BIT_CAM_AHB_CPP_SOFT_RST,
				   BIT_CAM_AHB_CPP_SOFT_RST);
		udelay(1);
		regmap_update_bits(dev->cam_ahb_gpr,
				   REG_CAM_AHB_AHB_RST,
				   BIT_CAM_AHB_CPP_SOFT_RST,
				   ~(unsigned int)BIT_CAM_AHB_CPP_SOFT_RST);

		clk_set_parent(dev->cpp_clk, dev->cpp_clk_default);
		clk_disable_unprepare(dev->cpp_clk);
		clk_disable_unprepare(dev->cpp_axi_eb);
		clk_disable_unprepare(dev->cpp_eb);
		clk_disable_unprepare(dev->jpg_cpp_mtx_eb);

		ret = sprd_cam_pw_off();
		if (ret != 0)
			pr_err("%s : sprd cam_sys power off failed\n",
					__func__);
		else
			pr_info("%s : sprd cam_sys power off!!\n",
					__func__);
	} else
		pr_info("cpp still has user, do not power off\n");

	mutex_unlock(&dev->lock);
#endif
	return ret;
}

static void cpp_register_isr(struct cpp_device *dev, enum cpp_irq_id id,
			     cpp_isr_func user_func, void *priv)
{
	unsigned long flag;

	if (id >= CPP_IRQ_NUMBER)
		return;

	spin_lock_irqsave(&dev->slock, flag);
	dev->isr_func[id] = user_func;
	dev->isr_data[id] = priv;
	if (user_func)
		reg_mwr(dev, CPP_INT_MASK, (1 << id), ~(1 << id));
	else
		reg_mwr(dev, CPP_INT_MASK, (1 << id), (1 << id));
	spin_unlock_irqrestore(&dev->slock, flag);
}

static void rot_isr(void *priv)
{
	struct rotif_device *rotif = (struct rotif_device *)priv;

	CPP_LOG("enter");

	if (!rotif)
		return;

	complete(&rotif->done_com);
}

static void scale_isr(void *priv)
{
	struct scif_device *scif = (struct scif_device *)priv;

	CPP_LOG("enter");

	if (!scif)
		return;

	complete(&scif->done_com);
}

static void path2_isr(void *priv)
{
	struct path2_if_device *path2_if = (struct path2_if_device *)priv;

	CPP_LOG("enter");

	if (!path2_if)
		return;

	complete(&path2_if->done_com);
}

static void path3_isr(void *priv)
{
	struct path3_if_device *path3_if =
		(struct path3_if_device *)priv;

	CPP_LOG("enter");

	if (!path3_if)
		return;

	complete(&path3_if->done_com);
}

static void cpp_read_reg(struct cpp_device *dev, unsigned int *reg_buf,
			 unsigned int *buf_len)
{

	unsigned int offset = 0;

	while (buf_len != 0 && offset < CPP_END) {
		*reg_buf++ = reg_rd(dev, offset);
		offset += 4;
		*buf_len -= 4;
	}

	*buf_len = offset;
}

void cpp_print_reg(void *priv)
{
	unsigned int *reg_buf = NULL;
	unsigned int reg_buf_len = 0x400;
	unsigned int print_len = 0, print_cnt = 0;
	struct cpp_device *dev = (struct cpp_device *)priv;

	reg_buf = vzalloc(reg_buf_len);
	if (!reg_buf)
		return;

	cpp_read_reg(dev, reg_buf, &reg_buf_len);

	CPP_LOG("scale registers\n");
	while (print_len < reg_buf_len) {
		CPP_LOG("offset 0x%03x : 0x%08x, 0x%08x, 0x%08x, 0x%08x\n",
			print_len,
			reg_buf[print_cnt],
			reg_buf[print_cnt + 1],
			reg_buf[print_cnt + 2], reg_buf[print_cnt + 3]);
		print_cnt += 4;
		print_len += 16;
	}
	vfree(reg_buf);
}

int cpp_get_sg_table(struct cpp_iommu_info *pfinfo)
{
	int i, ret;

	for (i = 0; i < 2; i++) {
		if (pfinfo->mfd[i] > 0) {
			ret = sprd_ion_get_buffer(pfinfo->mfd[i],
				NULL,
				&pfinfo->buf[i],
				&pfinfo->size[i]);
			if (ret) {
				pr_err("failed to get sg table\n");
				return -EFAULT;
			}
		}
	}
	return 0;
}

int cpp_get_addr(struct cpp_iommu_info *pfinfo, enum sprd_iommu_chtype ch_type)
{
	int i, ret;
	struct sprd_iommu_map_data iommu_data;

	for (i = 0; i < 2; i++) {
		if (pfinfo->size[i] <= 0)
			continue;

		if (sprd_iommu_attach_device(pfinfo->dev) == 0) {
			memset(&iommu_data, 0,
				sizeof(struct sprd_iommu_map_data));
			iommu_data.buf = pfinfo->buf[i];
			iommu_data.iova_size = pfinfo->size[i];
			iommu_data.ch_type = ch_type; /*SPRD_IOMMU_FM_CH_RW;*/
			iommu_data.sg_offset = pfinfo->offset[i];

			ret = sprd_iommu_map(pfinfo->dev, &iommu_data);
			if (ret) {
				pr_err("failed to get iommu kaddr %d\n", i);
				return -EFAULT;
			}

			if (ch_type == SPRD_IOMMU_FM_CH_RW)
				pfinfo->iova[i] = iommu_data.iova_addr
					+ pfinfo->offset[i];
			else
				pfinfo->iova[i] = iommu_data.iova_addr;
		} else {
			sprd_ion_get_phys_addr(pfinfo->mfd[i],
					NULL,
					&pfinfo->iova[i],
					&pfinfo->size[i]);
			pfinfo->iova[i] += pfinfo->offset[i];
			pr_info("cpp not use iommu,addr[%d]=0x%lx\n",
				i, pfinfo->iova[i]);
		}
	}
	return 0;
}

int cpp_free_addr(struct cpp_iommu_info *pfinfo, enum sprd_iommu_chtype ch_type)
{
	int i, ret;
	struct sprd_iommu_unmap_data iommu_data;

	for (i = 0; i < 2; i++) {
		if (pfinfo->size[i] <= 0)
			continue;

		if (sprd_iommu_attach_device(pfinfo->dev) == 0) {
			if (ch_type == SPRD_IOMMU_FM_CH_RW)
				iommu_data.iova_addr = pfinfo->iova[i]
							- pfinfo->offset[i];
			else
				iommu_data.iova_addr = pfinfo->iova[i];
			iommu_data.iova_size = pfinfo->size[i];
			iommu_data.ch_type = ch_type; /*SPRD_IOMMU_FM_CH_RW;*/
			iommu_data.buf = NULL;

			ret = sprd_iommu_unmap(pfinfo->dev, &iommu_data);
			if (ret) {
				pr_err("failed to free iommu %d\n", i);
				return -EFAULT;
			}
		}
	}
	return 0;
}

static inline struct scif_device *cpp_open_scale_if(struct cpp_device *dev)
{
	struct scif_device *scale_if = NULL;
	void *coeff_addr;

	CPP_LOG("dev->scif=%p", dev->scif);

	if (!dev->scif) {
		scale_if = vzalloc(sizeof(*scale_if));
		if (unlikely(!scale_if)) {
			pr_err("scale open: vzalloc fail!\n");
			return NULL;
		}

	coeff_addr = vzalloc(SC_COEFF_BUF_SIZE);
	if (unlikely(!coeff_addr)) {
		pr_err("scale create: vzalloc coeff_addr fail!\n");
		vfree(scale_if);
		return NULL;
	}

	init_completion(&scale_if->done_com);
	sema_init(&scale_if->start_sem, 1);

	scale_if->drv_priv.io_base = dev->io_base;
	scale_if->drv_priv.priv = (void *)scale_if;
	scale_if->drv_priv.coeff_addr = coeff_addr;
	scale_if->drv_priv.iommu_src.dev = &(dev->pdev->dev);
	scale_if->drv_priv.iommu_dst.dev = &(dev->pdev->dev);
	dev->scif = scale_if;
	} else {
		scale_if = dev->scif;
	}

	return scale_if;
}

static inline struct rotif_device *cpp_open_rot_if(struct cpp_device *dev)
{
	struct rotif_device *rotif = NULL;

	CPP_LOG("dev->rotif=%p", dev->rotif);
	if (!dev->rotif) {
		rotif = vzalloc(sizeof(*rotif));
		if (unlikely(!rotif)) {
			pr_err("rot open: vzalloc fail!\n");
			return NULL;
		}

	init_completion(&rotif->done_com);
	sema_init(&rotif->start_sem, 1);

	rotif->drv_priv.io_base = dev->io_base;
	rotif->drv_priv.priv = (void *)rotif;
	rotif->drv_priv.iommu_src.dev = &(dev->pdev->dev);
	rotif->drv_priv.iommu_dst.dev = &(dev->pdev->dev);
	dev->rotif = rotif;
	} else {
		rotif = dev->rotif;
	}

	return rotif;
}

static inline struct path2_if_device *cpp_open_path2_if(struct cpp_device *dev)
{
	struct path2_if_device *path2_if = NULL;
	void *coeff_addr;

	CPP_LOG("dev->path2_if=%p", dev->path2_if);
	if (!dev->path2_if) {
		path2_if = vzalloc(sizeof(*path2_if));
		if (unlikely(!path2_if)) {
			pr_err("path2 open: vzalloc fail!\n");
			return NULL;
		}

	coeff_addr = vzalloc(SC_COEFF_BUF_SIZE);
	if (unlikely(!coeff_addr)) {
		pr_err("path2 create: vzalloc coeff_addr fail!\n");
		vfree(path2_if);
		return NULL;
	}

	init_completion(&path2_if->done_com);
	sema_init(&path2_if->start_sem, 1);

	path2_if->drv_priv.io_base = dev->io_base;
	path2_if->drv_priv.priv = (void *)path2_if;
	path2_if->drv_priv.coeff_addr = coeff_addr;
	path2_if->drv_priv.iommu_src.dev = &(dev->pdev->dev);
	path2_if->drv_priv.iommu_dst.dev = &(dev->pdev->dev);
	dev->path2_if = path2_if;
	} else {
	path2_if = dev->path2_if;
	}

	return path2_if;
}

static inline struct path3_if_device *cpp_open_path3_if(struct cpp_device *dev)
{
	struct path3_if_device *path3_if = NULL;
	void *coeff_addr;

	CPP_LOG("dev->path3_if=%p", dev->path3_if);
	if (!dev->path3_if) {
		path3_if = vzalloc(sizeof(*path3_if));
		if (unlikely(!path3_if)) {
			pr_err("path3 open: vzalloc fail!\n");
			return NULL;
		}

	coeff_addr = vzalloc(SC_COEFF_BUF_SIZE);
	if (unlikely(!coeff_addr)) {
		pr_err("path3 create: vzalloc coeff_addr fail!\n");
		vfree(path3_if);
		return NULL;
	}

	init_completion(&path3_if->done_com);
	sema_init(&path3_if->start_sem, 1);

	path3_if->drv_priv.io_base = dev->io_base;
	path3_if->drv_priv.priv = (void *)path3_if;
	path3_if->drv_priv.coeff_addr = coeff_addr;
	path3_if->drv_priv.iommu_src.dev = &(dev->pdev->dev);
	path3_if->drv_priv.iommu_dst.dev = &(dev->pdev->dev);
	dev->path3_if = path3_if;
	} else {
	path3_if = dev->path3_if;
	}

	return path3_if;
}

static inline struct path0_path3_if_device *cpp_open_path0_path3_if(
	struct cpp_device *dev)
{
	struct path0_path3_if_device *path0_path3_if = NULL;

	CPP_LOG("dev->path0_path3_if=%p", dev->path0_path3_if);
	if (!dev->path0_path3_if) {
		path0_path3_if = vzalloc(sizeof(*path0_path3_if));
		if (unlikely(!path0_path3_if)) {
			pr_err("path0 path3 open: vzalloc fail!\n");
			return NULL;
	    }

	path0_path3_if->path0_if = dev->scif;
	path0_path3_if->path3_if = dev->path3_if;

	path0_path3_if->drv_priv.io_base = dev->io_base;
	path0_path3_if->drv_priv.priv = (void *)path0_path3_if;
	path0_path3_if->drv_priv.coeff_addr0 =
	    path0_path3_if->path0_if->drv_priv.coeff_addr;
	path0_path3_if->drv_priv.coeff_addr3 =
	    path0_path3_if->path3_if->drv_priv.coeff_addr;
	path0_path3_if->drv_priv.iommu_src.dev = &(dev->pdev->dev);
	path0_path3_if->drv_priv.iommu_dst_path0.dev = &(dev->pdev->dev);
	path0_path3_if->drv_priv.iommu_dst_path3.dev = &(dev->pdev->dev);

	dev->path0_path3_if = path0_path3_if;

	} else {
	    path0_path3_if = dev->path0_path3_if;
	}

	return path0_path3_if;
}

static inline struct path2_path3_if_device *cpp_open_path2_path3_if(
	struct cpp_device *dev)
{
	struct path2_path3_if_device *path2_path3_if = NULL;

	CPP_LOG("dev->path2_path3_if=%p", dev->path2_path3_if);
	if (!dev->path2_path3_if) {
		path2_path3_if = vzalloc(sizeof(*path2_path3_if));
		if (unlikely(!path2_path3_if)) {
			pr_err("path2 path3 open: vzalloc fail!\n");
			return NULL;
		}

	path2_path3_if->path2_if = dev->path2_if;
	path2_path3_if->path3_if = dev->path3_if;

	path2_path3_if->drv_priv.io_base = dev->io_base;
	path2_path3_if->drv_priv.priv = (void *)path2_path3_if;
	path2_path3_if->drv_priv.coeff_addr2 =
		path2_path3_if->path2_if->drv_priv.coeff_addr;
	path2_path3_if->drv_priv.coeff_addr3 =
		path2_path3_if->path3_if->drv_priv.coeff_addr;
	path2_path3_if->drv_priv.iommu_src.dev = &(dev->pdev->dev);
	path2_path3_if->drv_priv.iommu_dst_path2.dev = &(dev->pdev->dev);
	path2_path3_if->drv_priv.iommu_dst_path3.dev = &(dev->pdev->dev);

	dev->path2_path3_if = path2_path3_if;
	} else {
	    path2_path3_if = dev->path2_path3_if;
	}

	return path2_path3_if;
}

static inline void cpp_close_scale_if(struct cpp_device *dev)
{
	CPP_LOG("dev->scif=%p", dev->scif);

	if (dev->scif) {
		if (dev->scif->drv_priv.coeff_addr)
			vfree(dev->scif->drv_priv.coeff_addr);
		vfree(dev->scif);
		dev->scif = NULL;
	} else
	pr_info("close scale err: not opened yet");
}

static inline void cpp_close_rot_if(struct cpp_device *dev)
{
	CPP_LOG("dev->rotif=0x%p", dev->rotif);

	if (dev->rotif) {
		vfree(dev->rotif);
		dev->rotif = NULL;
	} else
	    pr_info("close rot err: not opened yet");
}

static inline void cpp_close_path2_if(struct cpp_device *dev)
{
	CPP_LOG("dev->path2_if=0x%p", dev->path2_if);

	if (dev->path2_if) {
		if (dev->path2_if->drv_priv.coeff_addr)
			vfree(dev->path2_if->drv_priv.coeff_addr);
		vfree(dev->path2_if);
		dev->path2_if = NULL;
	} else
	    pr_info("close path2 err: not opened yet");
}

static inline void cpp_close_path3_if(struct cpp_device *dev)
{
	CPP_LOG("dev->path3_if=0x%p", dev->path3_if);

	if (dev->path3_if) {
		if (dev->path3_if->drv_priv.coeff_addr)
			vfree(dev->path3_if->drv_priv.coeff_addr);
		vfree(dev->path3_if);
		dev->path3_if = NULL;
	} else
	    pr_info("close path2 err: not opened yet");
}

static inline void cpp_close_path0_path3_if(struct cpp_device *dev)
{
	CPP_LOG("dev->path0_path3_if=0x%p", dev->path0_path3_if);

	if (dev->path0_path3_if) {
		vfree(dev->path0_path3_if);
		dev->path0_path3_if = NULL;
	} else
	    pr_info("close path0 path3 err: not opened yet");
}

static inline void cpp_close_path2_path3_if(struct cpp_device *dev)
{
	CPP_LOG("dev->path2_path3_if=0x%p", dev->path2_path3_if);

	if (dev->path2_path3_if) {
		vfree(dev->path2_path3_if);
		dev->path2_path3_if = NULL;
	} else
	pr_info("close path2 path3 err: not opened yet");
}

static int cpp_open_path_all(struct cpp_device *dev)
{
	if (!cpp_open_scale_if(dev)) {
		pr_err("open path: fail to alloc scale_if!");
		return -1;
	}
	if (!cpp_open_rot_if(dev)) {
		pr_err("open path: fail to alloc rot_if!");
		cpp_close_scale_if(dev);
		return -1;
	}
	if (!cpp_open_path2_if(dev)) {
		pr_err("open path: fail to alloc path2_if!");
		cpp_close_scale_if(dev);
		cpp_close_rot_if(dev);
		return -1;
	}
	if (!cpp_open_path3_if(dev)) {
		pr_err("open path: fail to alloc path3_if!");
		cpp_close_scale_if(dev);
		cpp_close_rot_if(dev);
		cpp_close_path2_if(dev);
		return -1;
	}

	if (!cpp_open_path0_path3_if(dev)) {
		pr_err("open path: fail to alloc path0_path3_if!");
		cpp_close_scale_if(dev);
		cpp_close_rot_if(dev);
		cpp_close_path2_if(dev);
		cpp_close_path3_if(dev);
		return -1;
	}

	if (!cpp_open_path2_path3_if(dev)) {
		pr_err("open path: fail to alloc path2_path3_if!");
		cpp_close_scale_if(dev);
		cpp_close_rot_if(dev);
		cpp_close_path2_if(dev);
		cpp_close_path3_if(dev);
		cpp_close_path0_path3_if(dev);
		return -1;
	}
	return 0;
}

static inline void cpp_dump_parm(
	enum sprd_cpp_current_senario cur_senario, void *parm)
{
	struct sprd_cpp_scale_cfg_parm *parm_scale;
	struct sprd_cpp_rot_cfg_parm *parm_rot;
	struct sprd_cpp_path2_cfg_parm *parm_path2;
	struct sprd_cpp_path3_cfg_parm *parm_path3;
	struct sprd_cpp_path0_path3_cfg_parm *parm_path0_path3;
	struct sprd_cpp_path2_path3_cfg_parm *parm_path2_path3;

	switch (cur_senario) {
	case CPP_SENARIO_SCALE:
	parm_scale = (struct sprd_cpp_scale_cfg_parm *) parm;
	CPP_LOG("scale input size: %dX%d,rect=(%d,%d,%d,%d)",
		parm_scale->input_size.w, parm_scale->input_size.h,
		parm_scale->input_rect.x, parm_scale->input_rect.y,
		parm_scale->input_rect.w, parm_scale->input_rect.h);
	CPP_LOG("fmt=%d, endian=(%d,%d),addr=0x(%x,%x,%x),mode=%d",
		parm_scale->input_format,
		parm_scale->input_endian.y_endian,
		parm_scale->input_endian.uv_endian,
		parm_scale->input_addr.y, parm_scale->input_addr.u,
		parm_scale->input_addr.v, parm_scale->scale_mode);
	CPP_LOG("slice_height=%d,regualte_mode=%d",
		parm_scale->slice_height,
		parm_scale->regualte_mode);

	CPP_LOG("scale output size: %dX%d,fmt=%d",
		parm_scale->output_size.w, parm_scale->output_size.h,
		parm_scale->output_format);
	CPP_LOG("endian=(%d,%d),addr=0x(%x,%x,%x)",
		parm_scale->output_endian.y_endian,
		parm_scale->output_endian.uv_endian,
		parm_scale->output_addr.y, parm_scale->output_addr.u,
		parm_scale->output_addr.v);
	break;

	case CPP_SENARIO_ROTATE:
	parm_rot = (struct sprd_cpp_rot_cfg_parm *) parm;
	CPP_LOG("size: %dX%d, src=0x(%x,%x,%x), dst=0x(%x,%x,%x)",
		parm_rot->size.w, parm_rot->size.h, parm_rot->src_addr.y,
		parm_rot->src_addr.u, parm_rot->src_addr.v,
		parm_rot->dst_addr.y,
		parm_rot->dst_addr.u, parm_rot->dst_addr.v);
	CPP_LOG("format=%d, angle=%d, endian(%d,%d)",
		parm_rot->format, parm_rot->angle, parm_rot->src_endian,
		parm_rot->dst_endian);

	break;

	case CPP_SENARIO_PATH2:
	parm_path2 = (struct sprd_cpp_path2_cfg_parm *) parm;
	CPP_LOG("path2 input size: %dX%d,rect=(%d,%d,%d,%d)",
		parm_path2->input_size.w, parm_path2->input_size.h,
		parm_path2->input_rect.x, parm_path2->input_rect.y,
		parm_path2->input_rect.w, parm_path2->input_rect.h);
	CPP_LOG("fmt=%d, endian=(%d,%d),addr=0x(%x,%x,%x)",
		parm_path2->input_format,
		parm_path2->input_endian.y_endian,
		parm_path2->input_endian.uv_endian,
		parm_path2->input_addr.y, parm_path2->input_addr.u,
		parm_path2->input_addr.v); /*, parm_path2->scale_mode);*/
	/*CPP_LOG("slice_height=%d,regualte_mode=%d",*/
		/*parm_path2->slice_height,*/
		/*parm_path2->regualte_mode);*/

	CPP_LOG("path2 output size: %dX%d,fmt=%d",
		parm_path2->output_size.w, parm_path2->output_size.h,
		parm_path2->output_format);
	CPP_LOG("endian=(%d,%d),addr=0x(%x,%x,%x)",
		parm_path2->output_endian.y_endian,
		parm_path2->output_endian.uv_endian,
		parm_path2->output_addr.y, parm_path2->output_addr.u,
		parm_path2->output_addr.v);
	break;

	case CPP_SENARIO_PATH3:
	parm_path3 = (struct sprd_cpp_path3_cfg_parm *) parm;
	CPP_LOG("path3 input size: %dX%d,rect=(%d,%d,%d,%d)",
		parm_path3->input_size.w, parm_path3->input_size.h,
		parm_path3->input_rect.x, parm_path3->input_rect.y,
		parm_path3->input_rect.w, parm_path3->input_rect.h);
	CPP_LOG("fmt=%d, endian=(%d,%d),addr=0x(%x,%x,%x)",
		parm_path3->input_format,
		parm_path3->input_endian.y_endian,
		parm_path3->input_endian.uv_endian,
		parm_path3->input_addr.y, parm_path3->input_addr.u,
		parm_path3->input_addr.v); /*, parm_path3->scale_mode);*/
	/*CPP_LOG("slice_height=%d,regualte_mode=%d",*/
		/*parm_path3->slice_height,*/
		/*parm_path3->regualte_mode);*/

	CPP_LOG("path2 output size: %dX%d,fmt=%d",
		parm_path3->output_size.w, parm_path3->output_size.h,
		parm_path3->output_format);
	CPP_LOG("endian=(%d,%d),addr=0x(%x,%x,%x)",
		parm_path3->output_endian.y_endian,
		parm_path3->output_endian.uv_endian,
		parm_path3->output_addr.y, parm_path3->output_addr.u,
		parm_path3->output_addr.v);
	break;

	case CPP_SENARIO_PATH0_PATH3:
		parm_path0_path3 =
			(struct sprd_cpp_path0_path3_cfg_parm *) parm;
	    CPP_LOG("path03 input size: %dX%d,rect=(%d,%d,%d,%d)",
		parm_path0_path3->input_size.w, parm_path0_path3->input_size.h,
		parm_path0_path3->input_rect.x, parm_path0_path3->input_rect.y,
		parm_path0_path3->input_rect.w, parm_path0_path3->input_rect.h);
	    CPP_LOG("fmt=%d, endian=(%d,%d),addr=0x(%x,%x,%x)",
		parm_path0_path3->input_format,
		parm_path0_path3->input_endian.y_endian,
		parm_path0_path3->input_endian.uv_endian,
		parm_path0_path3->input_addr.y, parm_path0_path3->input_addr.u,
		parm_path0_path3->input_addr.v);
		/*path0*/
	    CPP_LOG("path0 output size: %dX%d,fmt=%d",
		parm_path0_path3->path0_output_size.w,
		parm_path0_path3->path0_output_size.h,
		parm_path0_path3->path0_output_format);
	    CPP_LOG("endian=(%d,%d),addr=0x(%x,%x,%x)",
		parm_path0_path3->path0_output_endian.y_endian,
		parm_path0_path3->path0_output_endian.uv_endian,
		parm_path0_path3->path0_output_addr.y,
		parm_path0_path3->path0_output_addr.u,
		parm_path0_path3->path0_output_addr.v);
		/*path3*/
		CPP_LOG("path3 output size: %dX%d,fmt=%d",
		parm_path0_path3->path3_output_size.w,
		parm_path0_path3->path3_output_size.h,
		parm_path0_path3->path3_output_format);
	    CPP_LOG("endian=(%d,%d),addr=0x(%x,%x,%x)",
		parm_path0_path3->path3_output_endian.y_endian,
		parm_path0_path3->path3_output_endian.uv_endian,
		parm_path0_path3->path3_output_addr.y,
		parm_path0_path3->path3_output_addr.u,
		parm_path0_path3->path3_output_addr.v);
	break;

	case CPP_SENARIO_PATH2_PATH3:
		parm_path2_path3 =
			(struct sprd_cpp_path2_path3_cfg_parm *) parm;
	    CPP_LOG("path23 input size: %dX%d,rect=(%d,%d,%d,%d)",
		parm_path2_path3->input_size.w, parm_path2_path3->input_size.h,
		parm_path2_path3->input_rect.x, parm_path2_path3->input_rect.y,
		parm_path2_path3->input_rect.w, parm_path2_path3->input_rect.h);
	    CPP_LOG("fmt=%d, endian=(%d,%d),addr=0x(%x,%x,%x)",
		parm_path2_path3->input_format,
		parm_path2_path3->input_endian.y_endian,
		parm_path2_path3->input_endian.uv_endian,
		parm_path2_path3->input_addr.y, parm_path2_path3->input_addr.u,
		parm_path2_path3->input_addr.v);
		/*path2*/
		CPP_LOG("path2 output size: %dX%d,fmt=%d",
		parm_path2_path3->path2_output_size.w,
		parm_path2_path3->path2_output_size.h,
		parm_path2_path3->path2_output_format);
	    CPP_LOG("endian=(%d,%d),addr=0x(%x,%x,%x)",
		parm_path2_path3->path2_output_endian.y_endian,
		parm_path2_path3->path2_output_endian.uv_endian,
		parm_path2_path3->path2_output_addr.y,
		parm_path2_path3->path2_output_addr.u,
		parm_path2_path3->path2_output_addr.v);
		/*path3*/
		CPP_LOG("path3 output size: %dX%d,fmt=%d",
		parm_path2_path3->path3_output_size.w,
		parm_path2_path3->path3_output_size.h,
		parm_path2_path3->path3_output_format);
	    CPP_LOG("endian=(%d,%d),addr=0x(%x,%x,%x)",
		parm_path2_path3->path3_output_endian.y_endian,
		parm_path2_path3->path3_output_endian.uv_endian,
		parm_path2_path3->path3_output_addr.y,
		parm_path2_path3->path3_output_addr.u,
		parm_path2_path3->path3_output_addr.v);
	break;

	default:
	CPP_LOG("cur_senario err: %d", cur_senario);
	}
}

#ifdef CPP_TEST_DRIVER
static void cpp_generate_parm(
	enum sprd_cpp_current_senario cur_senario, void *parm)
{
	struct sprd_cpp_scale_cfg_parm *parm_scale;
	struct sprd_cpp_rot_cfg_parm *parm_rot;
	struct sprd_cpp_path2_cfg_parm *parm_path2;
	struct sprd_cpp_path3_cfg_parm *parm_path3;
	struct sprd_cpp_path0_path3_cfg_parm *parm_path0_path3;
	struct sprd_cpp_path2_path3_cfg_parm *parm_path2_path3;

	switch (cur_senario) {
	case CPP_SENARIO_SCALE:
	parm_scale = (struct sprd_cpp_scale_cfg_parm *) parm;
	/*parm_scale->input_size.w = CPP_TEST_SCALE_INPUT_WIDTH;*/
	/*parm_scale->input_size.h = CPP_TEST_SCALE_INPUT_HEIGHT;*/
	/*parm_scale->input_rect.x = 0;*/
	/*parm_scale->input_rect.y = 0;*/
	/*parm_scale->input_rect.w = CPP_TEST_SCALE_INPUT_WIDTH;*/
	/*parm_scale->input_rect.h = CPP_TEST_SCALE_INPUT_HEIGHT;*/
	/*parm_scale->input_format = SCALE_YUV422;*/
	parm_scale->input_addr.y = cpp_test_image_in_alloc_phy_addr;
	parm_scale->input_addr.u = cpp_test_image_in_alloc_phy_addr
		+ (parm_scale->input_size.w * parm_scale->input_size.h);
	parm_scale->input_addr.v = 0;
	parm_scale->input_endian.y_endian = SCALE_ENDIAN_LITTLE;
	parm_scale->input_endian.uv_endian = SCALE_ENDIAN_HALFBIG;

	/*parm_scale->output_size.w = CPP_TEST_SCALE_OUTPUT_WIDTH;*/
	/*parm_scale->output_size.h = CPP_TEST_SCALE_OUTPUT_HEIGHT;*/
	parm_scale->output_addr.y = cpp_test_image_out_alloc_phy_addr;
	parm_scale->output_addr.u = cpp_test_image_out_alloc_phy_addr
		+ (parm_scale->output_size.w * parm_scale->output_size.h);
	parm_scale->output_addr.v = 0;
	/*parm_scale->output_format = SCALE_YUV420;*/
	parm_scale->output_endian.y_endian = SCALE_ENDIAN_LITTLE;
	parm_scale->output_endian.uv_endian = SCALE_ENDIAN_HALFBIG;
	parm_scale->slice_height = 0;
	parm_scale->regualte_mode = 0;
	parm_scale->scale_mode = 0;

	break;

	case CPP_SENARIO_ROTATE:
	parm_rot = (struct sprd_cpp_rot_cfg_parm *) parm;
	parm_rot->size.w = CPP_TEST_ROT_INPUT_WIDTH;
	parm_rot->size.h = CPP_TEST_ROT_INPUT_HEIGHT;
	parm_rot->format = ROT_YUV422;
	parm_rot->angle = ROT_180;
	/*((unsigned long long)&s8_test_rot_input[0]) & 0xffffffff;*/
	parm_rot->src_addr.y = cpp_test_image_in_alloc_phy_addr;
	/*((unsigned long long)&s8_test_rot_output[0]) & 0xffffffff;*/
	parm_rot->dst_addr.y = cpp_test_image_out_alloc_phy_addr;
	parm_rot->src_addr.u = 0;
	parm_rot->dst_addr.u = 0;
	parm_rot->src_addr.v = 0;
	parm_rot->dst_addr.v = 0;
	parm_rot->src_endian = 5;
	parm_rot->dst_endian = 5;
	/*memset(&s8_test_rot_input[0], 0x12, sizeof(s8_test_rot_input));*/
	/*memset(&s8_test_rot_output[0], 0x34, sizeof(s8_test_rot_output));*/
	break;

	case CPP_SENARIO_PATH2:
	parm_path2 = (struct sprd_cpp_path2_cfg_parm *) parm;
	parm_path2->input_size.w = CPP_TEST_PATH2_INPUT_WIDTH;
	parm_path2->input_size.h = CPP_TEST_PATH2_INPUT_HEIGHT;
	parm_path2->input_rect.x = 0;
	parm_path2->input_rect.y = 0;
	parm_path2->input_rect.w = CPP_TEST_PATH2_INPUT_WIDTH;
	parm_path2->input_rect.h = CPP_TEST_PATH2_INPUT_HEIGHT;
	parm_path2->input_format = SCALE_YUV422;
	parm_path2->input_addr.y = cpp_test_image_in_alloc_phy_addr;
	parm_path2->input_addr.u = cpp_test_image_in_alloc_phy_addr
		+ (CPP_TEST_PATH2_INPUT_WIDTH * CPP_TEST_PATH2_INPUT_HEIGHT);
	parm_path2->input_addr.v = 0;
	parm_path2->input_endian.y_endian = SCALE_ENDIAN_LITTLE;
	parm_path2->input_endian.uv_endian = SCALE_ENDIAN_HALFBIG;

	parm_path2->output_size.w = CPP_TEST_PATH2_OUTPUT_WIDTH;
	parm_path2->output_size.h = CPP_TEST_PATH2_OUTPUT_HEIGHT;
	parm_path2->output_addr.y = cpp_test_image_out_alloc_phy_addr;
	parm_path2->output_addr.u = cpp_test_image_out_alloc_phy_addr
		+ (CPP_TEST_PATH2_OUTPUT_WIDTH * CPP_TEST_PATH2_OUTPUT_HEIGHT);
	parm_path2->output_addr.v = 0;
	parm_path2->output_format = SCALE_YUV422;
	parm_path2->output_endian.y_endian = SCALE_ENDIAN_LITTLE;
	parm_path2->output_endian.uv_endian = SCALE_ENDIAN_HALFBIG;
	/*parm_path2->slice_height = 0;*/
	/*parm_path2->regualte_mode = 0;*/
	/*parm_path2->scale_mode = 0;*/
	break;

	case CPP_SENARIO_PATH3:
	parm_path3 = (struct sprd_cpp_path3_cfg_parm *) parm;
	parm_path3->input_size.w = CPP_TEST_PATH3_INPUT_WIDTH;
	parm_path3->input_size.h = CPP_TEST_PATH3_INPUT_HEIGHT;
	parm_path3->input_rect.x = 0;
	parm_path3->input_rect.y = 0;
	parm_path3->input_rect.w = CPP_TEST_PATH3_INPUT_WIDTH;
	parm_path3->input_rect.h = CPP_TEST_PATH3_INPUT_HEIGHT;
	parm_path3->input_format = SCALE_YUV422;
	parm_path3->input_addr.y = cpp_test_image_in_alloc_phy_addr;
	parm_path3->input_addr.u = cpp_test_image_in_alloc_phy_addr
		+ (CPP_TEST_PATH3_INPUT_WIDTH * CPP_TEST_PATH3_INPUT_HEIGHT);
	parm_path3->input_addr.v = 0;
	parm_path3->input_endian.y_endian = SCALE_ENDIAN_LITTLE;
	parm_path3->input_endian.uv_endian = SCALE_ENDIAN_HALFBIG;

	parm_path3->output_size.w = CPP_TEST_PATH3_OUTPUT_WIDTH;
	parm_path3->output_size.h = CPP_TEST_PATH3_OUTPUT_HEIGHT;
	parm_path3->output_addr.y = cpp_test_image_out_alloc_phy_addr;
	parm_path3->output_addr.u = cpp_test_image_out_alloc_phy_addr
		+ (CPP_TEST_PATH3_OUTPUT_WIDTH * CPP_TEST_PATH3_OUTPUT_HEIGHT);
	parm_path3->output_addr.v = 0;
	parm_path3->output_format = SCALE_YUV422;
	parm_path3->output_endian.y_endian = SCALE_ENDIAN_LITTLE;
	parm_path3->output_endian.uv_endian = SCALE_ENDIAN_HALFBIG;
	/*parm_path3->slice_height = 0;*/
	/*parm_path3->regualte_mode = 0;*/
	/*parm_path3->scale_mode = 0;*/
	break;

	case CPP_SENARIO_PATH0_PATH3:
	parm_path0_path3 = (struct sprd_cpp_path0_path3_cfg_parm *) parm;
    /*input param*/
	parm_path0_path3->input_size.w = CPP_TEST_SCALE_INPUT_WIDTH;
	parm_path0_path3->input_size.h = CPP_TEST_SCALE_INPUT_HEIGHT;
	parm_path0_path3->input_rect.x = 0;
	parm_path0_path3->input_rect.y = 0;
	parm_path0_path3->input_rect.w = CPP_TEST_SCALE_INPUT_WIDTH;
	parm_path0_path3->input_rect.h = CPP_TEST_SCALE_INPUT_HEIGHT;
	parm_path0_path3->input_format = SCALE_YUV422;
	parm_path0_path3->input_addr.y = cpp_test_image_in_alloc_phy_addr;
	parm_path0_path3->input_addr.u = cpp_test_image_in_alloc_phy_addr
		+ (CPP_TEST_SCALE_INPUT_WIDTH * CPP_TEST_SCALE_INPUT_HEIGHT);
	parm_path0_path3->input_addr.v = 0;
	parm_path0_path3->input_endian.y_endian = SCALE_ENDIAN_LITTLE;
	parm_path0_path3->input_endian.uv_endian = SCALE_ENDIAN_HALFBIG;
    /*path0 param*/
	parm_path0_path3->path0_output_size.w = CPP_TEST_SCALE_OUTPUT_WIDTH;
	parm_path0_path3->path0_output_size.h = CPP_TEST_SCALE_OUTPUT_HEIGHT;
	parm_path0_path3->path0_output_addr.y =
		cpp_test_image_out_alloc_phy_addr;
	parm_path0_path3->path0_output_addr.u =
		cpp_test_image_out_alloc_phy_addr
		+ (CPP_TEST_SCALE_OUTPUT_WIDTH * CPP_TEST_SCALE_OUTPUT_HEIGHT);
	parm_path0_path3->path0_output_addr.v = 0;
	parm_path0_path3->path0_output_format = SCALE_YUV422;
	parm_path0_path3->path0_output_endian.y_endian = SCALE_ENDIAN_LITTLE;
	parm_path0_path3->path0_output_endian.uv_endian = SCALE_ENDIAN_HALFBIG;
	/*parm_path0_path3->path0_slice_height = 0;*/
	/*parm_path0_path3->path0_regualte_mode = 0;*/
	/*parm_path0_path3->path0_scale_mode = 0;*/
    /*path3 param*/
	parm_path0_path3->path3_output_size.w = CPP_TEST_PATH3_OUTPUT_WIDTH;
	parm_path0_path3->path3_output_size.h = CPP_TEST_PATH3_OUTPUT_HEIGHT;
	parm_path0_path3->path3_output_addr.y =
		cpp_test_image_out_alloc_phy_addr2;
	parm_path0_path3->path3_output_addr.u =
		cpp_test_image_out_alloc_phy_addr2
		+ (CPP_TEST_PATH3_OUTPUT_WIDTH * CPP_TEST_PATH3_OUTPUT_HEIGHT);
	parm_path0_path3->path3_output_addr.v = 0;
	parm_path0_path3->path3_output_format = SCALE_YUV422;
	parm_path0_path3->path3_output_endian.y_endian = PATH3_ENDIAN_LITTLE;
	parm_path0_path3->path3_output_endian.uv_endian = PATH3_ENDIAN_HALFBIG;
	/*parm_path0_path3->path3_slice_height = 0;*/
	/*parm_path0_path3->path3_regualte_mode = 0;*/
	/*parm_path0_path3->path3_scale_mode = 0;*/
	break;

	case CPP_SENARIO_PATH2_PATH3:
	parm_path2_path3 = (struct sprd_cpp_path2_path3_cfg_parm *) parm;
    /*input param*/
	parm_path2_path3->input_size.w = CPP_TEST_PATH2_INPUT_WIDTH;
	parm_path2_path3->input_size.h = CPP_TEST_PATH2_INPUT_HEIGHT;
	parm_path2_path3->input_rect.x = 0;
	parm_path2_path3->input_rect.y = 0;
	parm_path2_path3->input_rect.w = CPP_TEST_PATH2_INPUT_WIDTH;
	parm_path2_path3->input_rect.h = CPP_TEST_PATH2_INPUT_HEIGHT;
	parm_path2_path3->input_format = SCALE_YUV422;
	parm_path2_path3->input_addr.y = cpp_test_image_in_alloc_phy_addr;
	parm_path2_path3->input_addr.u = cpp_test_image_in_alloc_phy_addr
		+ (CPP_TEST_PATH2_INPUT_WIDTH * CPP_TEST_PATH2_INPUT_HEIGHT);
	parm_path2_path3->input_addr.v = 0;
	parm_path2_path3->input_endian.y_endian = PATH2_ENDIAN_LITTLE;
	parm_path2_path3->input_endian.uv_endian = PATH2_ENDIAN_HALFBIG;
    /*path2 param*/
	parm_path2_path3->path2_output_size.w = CPP_TEST_PATH2_OUTPUT_WIDTH;
	parm_path2_path3->path2_output_size.h = CPP_TEST_PATH2_OUTPUT_HEIGHT;
	parm_path2_path3->path2_output_addr.y =
		cpp_test_image_out_alloc_phy_addr;
	parm_path2_path3->path2_output_addr.u =
		cpp_test_image_out_alloc_phy_addr
		+ (CPP_TEST_PATH2_OUTPUT_WIDTH * CPP_TEST_PATH2_OUTPUT_HEIGHT);
	parm_path2_path3->path2_output_addr.v = 0;
	parm_path2_path3->path2_output_format = SCALE_YUV420;
	parm_path2_path3->path2_output_endian.y_endian = PATH2_ENDIAN_LITTLE;
	parm_path2_path3->path2_output_endian.uv_endian = PATH2_ENDIAN_HALFBIG;
	/*parm_path2_path3->path2_slice_height = 0;*/
	/*parm_path2_path3->path2_regualte_mode = 0;*/
	/*parm_path2_path3->path2_scale_mode = 0;*/
    /*path3 param*/
	parm_path2_path3->path3_output_size.w = CPP_TEST_PATH3_OUTPUT_WIDTH;
	parm_path2_path3->path3_output_size.h = CPP_TEST_PATH3_OUTPUT_HEIGHT;
	parm_path2_path3->path3_output_addr.y =
		cpp_test_image_out_alloc_phy_addr2;
	parm_path2_path3->path3_output_addr.u =
		cpp_test_image_out_alloc_phy_addr2
		+ (CPP_TEST_PATH3_OUTPUT_WIDTH * CPP_TEST_PATH3_OUTPUT_HEIGHT);
	parm_path2_path3->path3_output_addr.v = 0;
	parm_path2_path3->path3_output_format = SCALE_YUV420;
	parm_path2_path3->path3_output_endian.y_endian = PATH3_ENDIAN_LITTLE;
	parm_path2_path3->path3_output_endian.uv_endian = PATH3_ENDIAN_HALFBIG;
	/*parm_path2_path3->path3_slice_height = 0;*/
	/*parm_path2_path3->path3_regualte_mode = 0;*/
	/*parm_path2_path3->path3_scale_mode = 0;*/
	break;

	default:
	CPP_LOG("cur_senario err: %d", cur_senario);
	}
}

static inline void cpp_test_alloc_and_load_image
	(enum sprd_cpp_current_senario cur_senario, void *parm)
{
	struct sprd_cpp_scale_cfg_parm *parm_scale;
	struct sprd_cpp_path2_cfg_parm *parm_path2;
	struct sprd_cpp_path3_cfg_parm *parm_path3;
	unsigned int test_image_in_size;
	unsigned int test_image_out1_size;

	switch (cur_senario) {
	case CPP_SENARIO_SCALE:
		parm_scale = (struct sprd_cpp_scale_cfg_parm *) parm;
		if (parm_scale->input_format == SCALE_YUV422)
			test_image_in_size =
				((parm_scale->input_size.w) *
				 (parm_scale->input_size.h)*2);
		else if (parm_scale->input_format == SCALE_YUV420)
			test_image_in_size =
				((parm_scale->input_size.w) *
				 (parm_scale->input_size.h)*3/2);
		else{
			test_image_in_size = 0;
			CPP_LOG("cur_senario err format: %d",
					cur_senario);
		}
		test_image_out1_size =
			((parm_scale->output_size.w) *
			 (parm_scale->output_size.h)*2);
		cpp_alloc_test_image_buffer(CPP_SENARIO_SCALE,
		    test_image_in_size,
		    test_image_out1_size, 0);
	    cpp_load_image_and_copy(CPP_TEST_IMAGE_IN_FILE_SCALE, NULL,
		    test_image_in_size, 0,
		    CPP_TEST_IMAGE_OUT_FILE_BASE_SCALE, NULL,
		    test_image_out1_size, 0);
		cpp_generate_parm(CPP_SENARIO_SCALE, parm);
		break;
	case CPP_SENARIO_ROTATE:
		cpp_alloc_test_image_buffer(CPP_SENARIO_ROTATE,
			CPP_TEST_IMAGE_IN_SIZE_ROT,
			CPP_TEST_IMAGE_OUT_SIZE_ROT, 0);
		cpp_load_image_and_copy(CPP_TEST_IMAGE_IN_FILE_ROT_Y,
			CPP_TEST_IMAGE_IN_FILE_ROT_UV,
			CPP_TEST_IMAGE_IN_SIZE_ROT/2,
			CPP_TEST_IMAGE_IN_SIZE_ROT/2,
			CPP_TEST_IMAGE_OUT_FILE_BASE_ROT, NULL,
			CPP_TEST_IMAGE_OUT_SIZE_ROT, 0);
		cpp_generate_parm(CPP_SENARIO_ROTATE, parm);
		break;
	case CPP_SENARIO_PATH2:
		parm_path2 = (struct sprd_cpp_path2_cfg_parm *) parm;
		if (parm_path2->input_format == SCALE_YUV422)
			test_image_in_size = ((parm_path2->input_size.w) *
					(parm_path2->input_size.h)*2);
		else if (parm_path2->input_format == SCALE_YUV420)
			test_image_in_size = ((parm_path2->input_size.w) *
					(parm_path2->input_size.h)*3/2);
		else{
			test_image_in_size = 0;
			CPP_LOG("cur_senario err format: %d",
					cur_senario);
		}
		test_image_out1_size = ((parm_path2->output_size.w) *
				(parm_path2->output_size.h)*2);
		cpp_alloc_test_image_buffer(CPP_SENARIO_PATH2,
			test_image_in_size,
			test_image_out1_size, 0);
		cpp_load_image_and_copy(CPP_TEST_IMAGE_IN_FILE_PATH2, NULL,
			test_image_in_size, 0,
			CPP_TEST_IMAGE_OUT_FILE_BASE_PATH2, NULL,
			test_image_out1_size, 0);
		cpp_generate_parm(CPP_SENARIO_PATH2, parm);
		break;
	case CPP_SENARIO_PATH3:
		parm_path3 = (struct sprd_cpp_path3_cfg_parm *) parm;
		if (parm_path3->input_format == SCALE_YUV422)
			test_image_in_size = ((parm_path3->input_size.w) *
					(parm_path3->input_size.h)*2);
		else if (parm_path3->input_format == SCALE_YUV420)
			test_image_in_size = ((parm_path3->input_size.w) *
					(parm_path3->input_size.h)*3/2);
		else{
			test_image_in_size = 0;
			CPP_LOG("cur_senario err format: %d",
					cur_senario);
		}
		test_image_out1_size = ((parm_path3->output_size.w) *
				(parm_path3->output_size.h)*2);
		cpp_alloc_test_image_buffer(CPP_SENARIO_PATH3,
			test_image_in_size,
			test_image_out1_size, 0);
		cpp_load_image_and_copy(CPP_TEST_IMAGE_IN_FILE_PATH3, NULL,
			test_image_in_size, 0,
			CPP_TEST_IMAGE_OUT_FILE_BASE_PATH3, NULL,
			test_image_out1_size, 0);
		cpp_generate_parm(CPP_SENARIO_PATH3, parm);
		break;
	case CPP_SENARIO_PATH0_PATH3:
	    cpp_alloc_test_image_buffer(CPP_SENARIO_PATH0_PATH3,
		    CPP_TEST_IMAGE_IN_SIZE_SCALE,
		    CPP_TEST_IMAGE_OUT_SIZE_SCALE,
			CPP_TEST_IMAGE_OUT_SIZE_PATH3);
	    cpp_load_image_and_copy(CPP_TEST_IMAGE_IN_FILE_SCALE, NULL,
		    CPP_TEST_IMAGE_IN_SIZE_SCALE, 0,
		    CPP_TEST_IMAGE_OUT_FILE_BASE_SCALE,
			CPP_TEST_IMAGE_OUT_FILE_BASE_PATH3,
		    CPP_TEST_IMAGE_OUT_SIZE_SCALE,
			CPP_TEST_IMAGE_OUT_SIZE_PATH3);
	    cpp_generate_parm(CPP_SENARIO_PATH0_PATH3, parm);
		break;
	case CPP_SENARIO_PATH2_PATH3:
	    cpp_alloc_test_image_buffer(CPP_SENARIO_PATH2_PATH3,
		    CPP_TEST_IMAGE_IN_SIZE_PATH2,
		    CPP_TEST_IMAGE_OUT_SIZE_PATH2,
			CPP_TEST_IMAGE_OUT_SIZE_PATH3);
	    cpp_load_image_and_copy(CPP_TEST_IMAGE_IN_FILE_PATH2, NULL,
		    CPP_TEST_IMAGE_IN_SIZE_PATH2, 0,
		    CPP_TEST_IMAGE_OUT_FILE_BASE_PATH2,
			CPP_TEST_IMAGE_OUT_FILE_BASE_PATH3,
			CPP_TEST_IMAGE_OUT_SIZE_PATH2,
			CPP_TEST_IMAGE_OUT_SIZE_PATH3);
	    cpp_generate_parm(CPP_SENARIO_PATH2_PATH3, parm);
		break;
	default:
	    CPP_LOG("cur_senario err: %d", cur_senario);
	}
}
#endif

static long sprd_cpp_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	int ret = 0;
	unsigned long timeleft = 0;
	struct cpp_device *dev = NULL;
	struct scif_device *scif = NULL;
	struct rotif_device *rotif = NULL;
	struct path2_if_device *path2_if = NULL;
	struct path3_if_device *path3_if = NULL;
	struct path0_path3_if_device *path0_path3_if = NULL;
	struct path2_path3_if_device *path2_path3_if = NULL;
	struct sprd_cpp_size s_sc_cap = {0, 0};
	int cpp_dimension = 0;

	if (!file)
		dev = sprd_cpp_dev;
	else
		dev = file->private_data;
	if (!dev)
		return -EFAULT;

	CPP_LOG("cmd=0x%x", cmd);
	switch (cmd) {
	case SPRD_CPP_IO_OPEN_SCALE:
	{
	mutex_lock(&dev->dev_lock);
	if (atomic_inc_return(&dev->open_users) == 1) {
		CPP_LOG("SPRD_CPP_IO_OPEN_PATH executed as a 1st run!");
		ret = cpp_open_path_all(dev);
		if (ret) {
			mutex_unlock(&dev->dev_lock);
			pr_err("open path: fail to open path!\n");
		    return -EFAULT;
		}
	} else
		pr_info("open path: already opened!\n");
	get_cpp_max_size(&s_sc_cap.w, &s_sc_cap.h);
	cpp_dimension = (s_sc_cap.h << 16) | s_sc_cap.w;
	ret = copy_to_user((int *)arg, &cpp_dimension,
				sizeof(cpp_dimension));
	if (ret) {
		mutex_unlock(&dev->dev_lock);
		return -EFAULT;
	}
	mutex_unlock(&dev->dev_lock);
	break;
	}
	case SPRD_CPP_IO_OPEN_ROT:
	case SPRD_CPP_IO_OPEN_PATH:
	{
	mutex_lock(&dev->dev_lock);
	if (atomic_inc_return(&dev->open_users) == 1) {
		CPP_LOG("SPRD_CPP_IO_OPEN_PATH executed as a 1st run!");
		ret = cpp_open_path_all(dev);
		if (ret) {
			mutex_unlock(&dev->dev_lock);
			pr_err("open path: fail to open path!\n");
		    return -EFAULT;
		}
	} else
		pr_info("open path: already opened!\n");
	mutex_unlock(&dev->dev_lock);
	break;
	}
	case SPRD_CPP_IO_CLOSE_ROT:
	case SPRD_CPP_IO_CLOSE_SCALE:
	case SPRD_CPP_IO_CLOSE_PATH:
	{
	mutex_lock(&dev->dev_lock);
	if (atomic_dec_return(&dev->open_users) == 0) {
		CPP_LOG(
			"SPRD_CPP_IO_CLOSE_PATH executed as no more open users!");
				cpp_close_scale_if(dev);
				cpp_close_rot_if(dev);
				cpp_close_path2_if(dev);
				cpp_close_path3_if(dev);
				cpp_close_path0_path3_if(dev);
				cpp_close_path2_path3_if(dev);
	}
	mutex_unlock(&dev->dev_lock);
	break;
	}
	case SPRD_CPP_IO_START_ROT:
	{

		struct sprd_cpp_rot_cfg_parm parm;

		CPP_LOG("SPRD_CPP_IO_START_ROT !");
		rotif = dev->rotif;
		if (!rotif) {
			pr_err("rot start: rotif is null!\n");
			return -EINVAL;
		}

		ret = copy_from_user(&parm,
				     (struct sprd_cpp_rot_cfg_parm *)arg,
					 sizeof(parm));
		if (ret)
			return -EFAULT;

#ifdef CPP_TEST_DRIVER
	cpp_test_alloc_and_load_image(CPP_SENARIO_ROTATE, (void *)&parm);
#endif

		ret = cpp_rot_check_parm(&parm);
		if (ret) {
			pr_err("failed to check rot parm\n");
			return -EINVAL;
		}

	cpp_dump_parm(CPP_SENARIO_ROTATE, (void *)&parm);

	down(&rotif->start_sem);

	dev->cur_senario |= CPP_SENARIO_ROTATE;
	cpp_register_isr(dev, CPP_ROT_DONE, rot_isr,
				 (void *)rotif);

	CPP_LOG("start to rot y");

	mutex_lock(&dev->hw_lock);
#ifdef CPP_TEST_DRIVER
	cpp_rot_set_y_parm_t32_load_image(&parm, &rotif->drv_priv);
#else
	cpp_rot_set_y_parm(&parm, &rotif->drv_priv);
#endif


	cpp_rot_start(&rotif->drv_priv);

	/*cpp_print_reg(dev);*/

	if (cpp_rot_is_end(&parm) == 0) {
		timeleft = wait_for_completion_timeout
			(&rotif->done_com,
			msecs_to_jiffies(ROT_TIMEOUT));
		if (timeleft == 0) {
			pr_err("failed to get rot done sem");
			cpp_rot_stop(&rotif->drv_priv);
			cpp_register_isr(dev, CPP_ROT_DONE, NULL, NULL);
			mutex_unlock(&dev->hw_lock);
			dev->cur_senario &= ~CPP_SENARIO_ROTATE;
			up(&rotif->start_sem);
			return -1;
		}

		CPP_LOG("start to rot uv");
#ifdef CPP_TEST_DRIVER
		cpp_rot_set_uv_parm_t32_load_image(&rotif->drv_priv);
		CPP_LOG("start to rot uv, src_addr=0x%x, dst_addr=0x%x",
			rotif->drv_priv.rot_src_addr,
			rotif->drv_priv.rot_dst_addr);
#else
		cpp_rot_set_uv_parm(&rotif->drv_priv);
#endif
		cpp_rot_start(&rotif->drv_priv);
		/*cpp_print_reg(dev);*/

		}
	timeleft = wait_for_completion_timeout
		(&rotif->done_com,
		msecs_to_jiffies(ROT_TIMEOUT));
	if (timeleft == 0) {
		pr_err("failed to get rot done sem2");
		cpp_rot_stop(&rotif->drv_priv);
		cpp_register_isr(dev, CPP_ROT_DONE, NULL, NULL);
		mutex_unlock(&dev->hw_lock);
		dev->cur_senario &= ~CPP_SENARIO_ROTATE;
		up(&rotif->start_sem);
		return -1;
		}
		cpp_rot_stop(&rotif->drv_priv);
		cpp_register_isr(dev, CPP_ROT_DONE, NULL, NULL);
		mutex_unlock(&dev->hw_lock);
		dev->cur_senario &= ~CPP_SENARIO_ROTATE;

		up(&rotif->start_sem);
#ifdef CPP_TEST_DRIVER
		/*Write imag to a file.*/
		cpp_write_image_to_file(
			(char *)cpp_test_image_out_alloc_virt_addr,
			CPP_TEST_IMAGE_OUT_SIZE_ROT,
			CPP_TEST_IMAGE_OUT_FILE_ROT);
		cpp_free_test_image_buffer();

#endif

	break;
	}
	case SPRD_CPP_IO_START_SCALE:
	{
		struct sprd_cpp_scale_cfg_parm parm;

		CPP_LOG("SPRD_CPP_IO_START_SCALE !");
		scif = dev->scif;

		if (!scif) {
			pr_err("scale start:scif is null!\n");
			return -EFAULT;
		}

		if (!file) {
			if ((void *)arg == NULL)
				return -EFAULT;
			parm = *((struct sprd_cpp_scale_cfg_parm *)arg);
		} else {
			ret = copy_from_user(&parm,
				     (struct sprd_cpp_scale_cfg_parm *)arg,
				     sizeof(struct sprd_cpp_scale_cfg_parm));
			if (ret)
				return -EFAULT;
		}
#ifdef CPP_TEST_DRIVER
	cpp_test_alloc_and_load_image(CPP_SENARIO_SCALE, (void *)&parm);
#endif
	cpp_dump_parm(CPP_SENARIO_SCALE, (void *)&parm);
	down(&scif->start_sem);
	dev->cur_senario |= CPP_SENARIO_SCALE;
	cpp_register_isr(dev, CPP_SCALE_DONE, scale_isr,
		(void *)scif);

	CPP_LOG("start to scale\n");

	mutex_lock(&dev->hw_lock);
	ret = cpp_scale_start(&parm, &scif->drv_priv);
	if (ret) {
		mutex_unlock(&dev->hw_lock);
		pr_err("failed to start scaler\n");
		dev->cur_senario &= ~CPP_SENARIO_SCALE;
		cpp_register_isr(dev, CPP_SCALE_DONE, NULL, NULL);
		up(&scif->start_sem);
		return -1;
	}
	mutex_unlock(&dev->hw_lock);
	break;
	}
	case SPRD_CPP_IO_CONTINUE_SCALE:
	{
	pr_info("not supported by current driver\n");
	break;
	}
	case SPRD_CPP_IO_STOP_SCALE:
	{
		scif = dev->scif;
		if (!scif) {
			pr_err("scale stop:scif is null!\n");
			return -EINVAL;
		}
		CPP_LOG("scale stop s\n");
		timeleft = wait_for_completion_timeout
			(&scif->done_com,
			 msecs_to_jiffies(SCALE_TIMEOUT));
		if (timeleft == 0) {
			pr_err("failed to get scaling done sem\n");
			ret = -1;
		}
		scif->drv_priv.iommu_src.dev = &(dev->pdev->dev);
		scif->drv_priv.iommu_dst.dev = &(dev->pdev->dev);

		mutex_lock(&dev->hw_lock);
		cpp_scale_stop(&scif->drv_priv);
		mutex_unlock(&dev->hw_lock);

		cpp_register_isr(dev, CPP_SCALE_DONE, NULL, NULL);
		dev->cur_senario &= ~CPP_SENARIO_SCALE;
		up(&scif->start_sem);
#ifdef CPP_TEST_DRIVER
		/* Write imag to a file.*/
		cpp_write_image_to_file((char *)
				cpp_test_image_out_alloc_virt_addr,
				CPP_TEST_IMAGE_OUT_SIZE_SCALE,
				CPP_TEST_IMAGE_OUT_FILE_SCALE);
		cpp_free_test_image_buffer();
#endif
		CPP_LOG("scale stop e\n");
		break;
	}
	case SPRD_CPP_IO_START_PATH2:
	{
		struct sprd_cpp_path2_cfg_parm parm;

		path2_if = dev->path2_if;

		if (!path2_if) {
			pr_err("path2 start:path2_if is null!\n");
			return -EFAULT;
		}

		if (!file) {
			if ((void *)arg == NULL)
				return -EFAULT;
			parm = *((struct sprd_cpp_path2_cfg_parm *)arg);
		} else {
			ret = copy_from_user(&parm,
				(struct sprd_cpp_path2_cfg_parm *)arg,
				sizeof(struct sprd_cpp_path2_cfg_parm));
			if (ret)
				return -EFAULT;
		}
#ifdef CPP_TEST_DRIVER
		cpp_test_alloc_and_load_image(CPP_SENARIO_PATH2,
				(void *)&parm);
#endif

		cpp_dump_parm(CPP_SENARIO_PATH2, (void *)&parm);

		down(&path2_if->start_sem);
		dev->cur_senario |= CPP_SENARIO_PATH2;
		cpp_register_isr(dev, CPP_PATH2_DONE, path2_isr,
				(void *)path2_if);

		CPP_LOG("start path2\n");

		mutex_lock(&dev->hw_lock);
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
		if (!file)
			path2_if->drv_priv.is_from_isp = 1;
#endif
		ret = cpp_path2_start(&parm, &path2_if->drv_priv);
		if (ret) {
			mutex_unlock(&dev->hw_lock);
			pr_err("failed to start path2\n");
			dev->cur_senario &= ~CPP_SENARIO_PATH2;
			up(&path2_if->start_sem);
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
			if (!file)
				path2_if->drv_priv.is_from_isp = 0;
#endif
			return -1;
		}

#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
		if (!file)
			path2_if->drv_priv.is_from_isp = 0;
#endif
		mutex_unlock(&dev->hw_lock);
		break;
	}
	case SPRD_CPP_IO_STOP_PATH2:
	{
		path2_if = dev->path2_if;
		if (!path2_if) {
			pr_err("path2 stop: path2_if is null!\n");
			return -EINVAL;
		}
		timeleft = wait_for_completion_timeout
			(&path2_if->done_com,
			 msecs_to_jiffies(PATH2_TIMEOUT));
		if (timeleft == 0)
			pr_err("failed to get path2 done sem\n");

		path2_if->drv_priv.iommu_src.dev = &dev->pdev->dev;
		path2_if->drv_priv.iommu_dst.dev = &dev->pdev->dev;

		mutex_lock(&dev->hw_lock);
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
		if (!file)
			path2_if->drv_priv.is_from_isp = 1;
#endif
		cpp_path2_stop(&path2_if->drv_priv);
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
		if (!file)
			path2_if->drv_priv.is_from_isp = 0;
#endif
		mutex_unlock(&dev->hw_lock);

		cpp_register_isr(dev, CPP_PATH2_DONE, NULL, NULL);
		dev->cur_senario &= ~CPP_SENARIO_PATH2;
		up(&path2_if->start_sem);
#ifdef CPP_TEST_DRIVER
		/* Write imag to a file.*/
		cpp_write_image_to_file((char *)
				cpp_test_image_out_alloc_virt_addr,
				CPP_TEST_IMAGE_OUT_SIZE_PATH2,
				CPP_TEST_IMAGE_OUT_FILE_PATH2);
		cpp_free_test_image_buffer();
#endif

		break;
	}
	case SPRD_CPP_IO_START_PATH3:
	{
		struct sprd_cpp_path3_cfg_parm parm;

		path3_if = dev->path3_if;

		if (!path3_if) {
			pr_err("path3 start:path3_if is null!\n");
			return -EFAULT;
		}

		if (!file) {
			if ((void *)arg == NULL)
				return -EFAULT;
			parm = *((struct sprd_cpp_path3_cfg_parm *)arg);
		} else {
			ret = copy_from_user(&parm,
				(struct sprd_cpp_path3_cfg_parm __user *)arg,
				sizeof(parm));
			if (ret)
				return -EFAULT;
		}

#ifdef CPP_TEST_DRIVER
		cpp_test_alloc_and_load_image(CPP_SENARIO_PATH3,
				(void *)&parm);
#endif

		cpp_dump_parm(CPP_SENARIO_PATH3, (void *)&parm);

		down(&path3_if->start_sem);
		dev->cur_senario |= CPP_SENARIO_PATH3;
		cpp_register_isr(dev, CPP_PATH3_DONE, path3_isr,
				(void *)path3_if);

		CPP_LOG("start path3\n");

		mutex_lock(&dev->hw_lock);
		ret = cpp_path3_start(&parm, &path3_if->drv_priv);
		if (ret) {
			mutex_unlock(&dev->hw_lock);
			pr_err("failed to start path3\n");
			dev->cur_senario &= ~CPP_SENARIO_PATH3;
			up(&path3_if->start_sem);
			return -1;
		}
		mutex_unlock(&dev->hw_lock);
		break;
	}
	case SPRD_CPP_IO_STOP_PATH3:
	{
		path3_if = dev->path3_if;
		if (!path3_if) {
			pr_err("path3 stop: path3_if is null!\n");
			return -EINVAL;
		}
		timeleft = wait_for_completion_timeout
			(&path3_if->done_com,
			 msecs_to_jiffies(PATH3_TIMEOUT));
		if (timeleft == 0)
			pr_err("failed to get path3 done sem\n");


		path3_if->drv_priv.iommu_src.dev = &dev->pdev->dev;
		path3_if->drv_priv.iommu_dst.dev = &dev->pdev->dev;

		mutex_lock(&dev->hw_lock);
		cpp_path3_stop(&path3_if->drv_priv);
		mutex_unlock(&dev->hw_lock);

		cpp_register_isr(dev, CPP_PATH3_DONE, NULL, NULL);
		dev->cur_senario &= ~CPP_SENARIO_PATH3;
		up(&path3_if->start_sem);
#ifdef CPP_TEST_DRIVER
		/* Write imag to a file.*/
		cpp_write_image_to_file((char *)
				cpp_test_image_out_alloc_virt_addr,
				CPP_TEST_IMAGE_OUT_SIZE_PATH3,
				CPP_TEST_IMAGE_OUT_FILE_PATH3);
		cpp_free_test_image_buffer();
#endif
		break;
	}
	case SPRD_CPP_IO_START_PATH0_PATH3:
	{
		struct sprd_cpp_path0_path3_cfg_parm parm;

		path0_path3_if = dev->path0_path3_if;
		if (!path0_path3_if) {
			pr_err("path0_path3 start: path0_path3_if is null!\n");
			return -EFAULT;
		}
		scif = path0_path3_if->path0_if;
		path3_if = path0_path3_if->path3_if;

		if (!scif || !path3_if) {
			pr_err("path0_path3 start: path0 or path3 interface is null!\n");
			return -EFAULT;
		}

		if (!file) {
			if ((void *)arg == NULL)
				return -EFAULT;
			parm = *((struct sprd_cpp_path0_path3_cfg_parm *)arg);
		} else {
			ret = copy_from_user(&parm,
				(struct sprd_cpp_path0_path3_cfg_parm __user *)
				arg,
				sizeof(parm));
			if (ret)
				return -EFAULT;
		}

#ifdef CPP_TEST_DRIVER
		cpp_test_alloc_and_load_image(CPP_SENARIO_PATH0_PATH3,
				(void *)&parm);
#endif
		cpp_dump_parm(CPP_SENARIO_PATH0_PATH3, (void *)&parm);
		down(&scif->start_sem);
		down(&path3_if->start_sem);

		dev->cur_senario |= CPP_SENARIO_PATH0_PATH3;

		cpp_register_isr(dev, CPP_SCALE_DONE, scale_isr,
				(void *)scif);
		/*cpp_register_isr(dev, CPP_PATH3_DONE, path3_isr,*/
		/*(void *)path3_if);*/

		CPP_LOG("start path0_path3\n");

		mutex_lock(&dev->hw_lock);
		ret = cpp_path0_path3_start(&parm, &path0_path3_if->drv_priv);
		if (ret) {
			mutex_unlock(&dev->hw_lock);
			pr_err("failed to start path0 path3\n");
			dev->cur_senario &= ~CPP_SENARIO_PATH0_PATH3;
			up(&path3_if->start_sem);
			up(&scif->start_sem);
			return -1;
		}
		mutex_unlock(&dev->hw_lock);
		break;
	}

	case SPRD_CPP_IO_STOP_PATH0_PATH3:
	{
		path0_path3_if = dev->path0_path3_if;


		if (!path0_path3_if) {
			pr_err("path0 path3 stop: path0_path3_if is null!\n");
			return -EINVAL;
		}
		scif = path0_path3_if->path0_if;
		path3_if = path0_path3_if->path3_if;

		timeleft = wait_for_completion_timeout
			(&scif->done_com,
			 msecs_to_jiffies(SCALE_TIMEOUT));
		if (timeleft == 0)
			pr_err("failed to get path0 done sem\n");

		path3_if->drv_priv.iommu_src.dev = &dev->pdev->dev;
		path3_if->drv_priv.iommu_dst.dev = &dev->pdev->dev;

		scif->drv_priv.iommu_src.dev = &dev->pdev->dev;
		scif->drv_priv.iommu_dst.dev = &dev->pdev->dev;

		mutex_lock(&dev->hw_lock);
		cpp_path0_path3_stop(&path0_path3_if->drv_priv);
		mutex_unlock(&dev->hw_lock);

		cpp_register_isr(dev, CPP_PATH3_DONE, NULL, NULL);
		cpp_register_isr(dev, CPP_SCALE_DONE, NULL, NULL);
		dev->cur_senario &= ~CPP_SENARIO_PATH0_PATH3;
		up(&path3_if->start_sem);
		up(&scif->start_sem);
#ifdef CPP_TEST_DRIVER
		/* Write imag to a file.*/
		cpp_write_image_to_file((char *)
				cpp_test_image_out_alloc_virt_addr,
			CPP_TEST_IMAGE_OUT_SIZE_SCALE,
			CPP_TEST_IMAGE_OUT_FILE_SCALE);
		cpp_write_image_to_file((char *)
				cpp_test_image_out_alloc_virt_addr2,
			CPP_TEST_IMAGE_OUT_SIZE_PATH3,
			CPP_TEST_IMAGE_OUT_FILE_PATH3);
		cpp_free_test_image_buffer();
#endif

		break;
	}
	case SPRD_CPP_IO_START_PATH2_PATH3:
	{
		struct sprd_cpp_path2_path3_cfg_parm parm;

		path2_path3_if = dev->path2_path3_if;
		if (!path2_path3_if) {
			pr_err("path2_path3 start: path2_path3_if is null!\n");
			return -EFAULT;
		}

		path2_if = path2_path3_if->path2_if;
		path3_if = path2_path3_if->path3_if;

		if (!path2_if || !path3_if) {
			pr_err("path2_path3 start: path2 or path3 interface is null!\n");
			return -EFAULT;
		}

		if (!file) {
			if ((void *)arg == NULL)
				return -EFAULT;
			parm = *((struct sprd_cpp_path2_path3_cfg_parm *)arg);
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
			path2_path3_if->drv_priv.is_from_isp = 1;
#endif
		} else {
			ret = copy_from_user(&parm,
				(struct sprd_cpp_path2_path3_cfg_parm __user *)
				arg,
				sizeof(parm));
			if (ret)
				return -EFAULT;
		}

#ifdef CPP_TEST_DRIVER
		cpp_test_alloc_and_load_image(CPP_SENARIO_PATH2_PATH3,
				(void *)&parm);
#endif
		cpp_dump_parm(CPP_SENARIO_PATH2_PATH3, (void *)&parm);
		down(&path2_if->start_sem);
		down(&path3_if->start_sem);

		dev->cur_senario |= CPP_SENARIO_PATH2_PATH3;

		cpp_register_isr(dev, CPP_PATH2_DONE, path2_isr,
				(void *)path2_if);
		/*cpp_register_isr(dev, CPP_PATH3_DONE, path3_isr,*/
		/*(void *)path3_if);*/

		mutex_lock(&dev->hw_lock);
		ret = cpp_path2_path3_start(&parm, &path2_path3_if->drv_priv);
		if (ret) {
			mutex_unlock(&dev->hw_lock);
			pr_err("failed to start path2 path3\n");
			dev->cur_senario &= ~CPP_SENARIO_PATH2_PATH3;
			up(&path3_if->start_sem);
			up(&path2_if->start_sem);
			return -1;
		}
		mutex_unlock(&dev->hw_lock);
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
		if (!file)
			path2_path3_if->drv_priv.is_from_isp = 0;
#endif
		break;
	}
	case SPRD_CPP_IO_STOP_PATH2_PATH3:
	{
		path2_path3_if = dev->path2_path3_if;

		if (!path2_path3_if) {
			pr_err("path2 path3 stop:path2_path3_if is null!\n");
			return -EINVAL;
		}

		path2_if = path2_path3_if->path2_if;
		path3_if = path2_path3_if->path3_if;

		timeleft = wait_for_completion_timeout
			(&path2_if->done_com,
			 msecs_to_jiffies(PATH2_TIMEOUT));
		if (timeleft == 0)
			pr_err("failed to get path2 done sem\n");

		path3_if->drv_priv.iommu_src.dev = &dev->pdev->dev;
		path3_if->drv_priv.iommu_dst.dev = &dev->pdev->dev;

		path2_if->drv_priv.iommu_src.dev = &dev->pdev->dev;
		path2_if->drv_priv.iommu_dst.dev = &dev->pdev->dev;

#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
		if (!file)
			path2_path3_if->drv_priv.is_from_isp = 1;
#endif
		mutex_lock(&dev->hw_lock);
		cpp_path2_path3_stop(&path2_path3_if->drv_priv);
		mutex_unlock(&dev->hw_lock);

		cpp_register_isr(dev, CPP_PATH3_DONE, NULL, NULL);
		cpp_register_isr(dev, CPP_PATH2_DONE, NULL, NULL);

		dev->cur_senario &= ~CPP_SENARIO_PATH2_PATH3;
		up(&path3_if->start_sem);
		up(&path2_if->start_sem);
#ifdef CPP_TEST_DRIVER
		/* Write imag to a file.*/
		cpp_write_image_to_file((char *)
				cpp_test_image_out_alloc_virt_addr,
				CPP_TEST_IMAGE_OUT_SIZE_PATH2,
				CPP_TEST_IMAGE_OUT_FILE_PATH2);
		cpp_write_image_to_file((char *)
				cpp_test_image_out_alloc_virt_addr2,
				CPP_TEST_IMAGE_OUT_SIZE_PATH3,
				CPP_TEST_IMAGE_OUT_FILE_PATH3);
		cpp_free_test_image_buffer();
#endif
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
		if (!file)
			path2_path3_if->drv_priv.is_from_isp = 0;
#endif

		break;
	}
	default:
	pr_err("cpp ioctl invalid cmd 0x%x\n", cmd);
	ret = -EINVAL;
	break;
	}

	return ret;
}


int cpp_k_open_device(void)
{
	int ret = -1;

	CPP_LOG("cpp_k_open_device\n");
	ret = cpp_module_enable(sprd_cpp_dev);
	if (ret)
		pr_err("cpp_k_open_device: fail to enable module\n");

	ret = sprd_cpp_ioctl(NULL, SPRD_CPP_IO_OPEN_PATH, 0);
	return ret;
}
EXPORT_SYMBOL(cpp_k_open_device);

int cpp_k_close_device(void)
{
	int ret = -1;

	CPP_LOG("cpp_k_close_device\n");
	if (sprd_cpp_dev)
		cpp_module_disable(sprd_cpp_dev);

	ret = sprd_cpp_ioctl(NULL, SPRD_CPP_IO_CLOSE_PATH, 0);
	return ret;
}
EXPORT_SYMBOL(cpp_k_close_device);

int cpp_k_start_scale(struct sprd_dcam_img_frm *src_img,
	struct sprd_dcam_img_frm *dst_img1,
	struct sprd_dcam_img_frm *dst_img2, int *path_id)
{
	int ret = -1;

	if ((!src_img) || (!dst_img1)) {
		pr_err("cpp start scale: src_img and dst_img1 should NOT be NULL");
		ret = -1;
		return ret;
	}

	if (!path_id) {
		pr_err("cpp start scale: path id should NOT be NULL");
		ret = -1;
		return ret;
	}

	/*TODO: add more detailed parameter checking here.*/

	if (!dst_img2) { /* 1 input 1 output.*/
		if ((*path_id != CPP_SENARIO_PATH3)
				&& (*path_id != CPP_SENARIO_PATH2)
				&& (*path_id != CPP_SENARIO_SCALE)) {
		/* step1: choose path id*/
		if (dst_img1->size.w <= CPP_PATH3_PROC_WIDTH)
			*path_id = CPP_SENARIO_PATH3;
		else if (dst_img1->size.w <= CPP_PATH2_PROC_WIDTH)
			*path_id = CPP_SENARIO_PATH2;
		else if (dst_img1->size.w <= CPP_PATH0_PROC_WIDTH)
			*path_id = CPP_SENARIO_SCALE;
		}
	/* step2: prepare parameters, and call corresponding path.*/

		switch (*path_id) {
		case CPP_SENARIO_SCALE:
		{
			struct sprd_cpp_scale_cfg_parm parm;

			/* input parameter;*/
			parm.input_size = src_img->size;
			parm.input_rect = src_img->rect;
			parm.input_format = src_img->fmt;
			parm.input_addr = src_img->addr_phy;
			parm.input_endian = src_img->data_end;

			/* output parameter;*/
			parm.output_size = dst_img1->size;
			parm.output_format = dst_img1->fmt;
			parm.output_addr = dst_img1->addr_phy;
			parm.output_endian = dst_img1->data_end;

			/* TODO: other parameters need data from SPRD.*/

			ret = sprd_cpp_ioctl(NULL, SPRD_CPP_IO_START_SCALE,
				(unsigned long)(&parm));
			break;
		}
		case CPP_SENARIO_PATH2:
		{
			struct sprd_cpp_path2_cfg_parm parm;

			/* input parameter;*/
			parm.input_size = src_img->size;
			parm.input_rect = src_img->rect;
			parm.input_format = src_img->fmt;
			parm.input_addr = src_img->addr_phy;
			parm.input_endian = src_img->data_end;

			/* output parameter;*/
			parm.output_size = dst_img1->size;
			parm.output_format = dst_img1->fmt;
			parm.output_addr = dst_img1->addr_phy;
			parm.output_endian = dst_img1->data_end;

			/* TODO: other parameters need data from SPRD.*/

			ret = sprd_cpp_ioctl(NULL, SPRD_CPP_IO_START_PATH2,
				(unsigned long)(&parm));
			break;
		}
		case CPP_SENARIO_PATH3:
		{
			struct sprd_cpp_path3_cfg_parm parm;

			/* input parameter;*/
			parm.input_size = src_img->size;
			parm.input_rect = src_img->rect;
			parm.input_format = src_img->fmt;
			parm.input_addr = src_img->addr_phy;
			parm.input_endian = src_img->data_end;

			/* output parameter;*/
			parm.output_size = dst_img1->size;
			parm.output_format = dst_img1->fmt;
			parm.output_addr = dst_img1->addr_phy;
			parm.output_endian = dst_img1->data_end;

			/* TODO: other parameters need data from SPRD.*/

			ret = sprd_cpp_ioctl(NULL, SPRD_CPP_IO_START_PATH3,
				(unsigned long)(&parm));

			break;
		}
		default:
			pr_err("invalid path id %d\n", *path_id);
		break;
	}

	} else {
	/* 1 input 2 outputs.*/
	int tmp_path_id1 = 0x0;
	int tmp_path_id2 = 0x0;

	if ((*path_id != CPP_SENARIO_PATH0_PATH3)
			&& (*path_id != CPP_SENARIO_PATH2_PATH3)) {
		/* step1: choose path id*/
		if (dst_img1->size.w <= CPP_PATH3_PROC_WIDTH)
			tmp_path_id1 = CPP_SENARIO_PATH3;
		else if (dst_img1->size.w <= CPP_PATH2_PROC_WIDTH)
			tmp_path_id1 = CPP_SENARIO_PATH2;
		else if (dst_img1->size.w <= CPP_PATH0_PROC_WIDTH)
			tmp_path_id1 = CPP_SENARIO_SCALE;

		if (dst_img2->size.w <= CPP_PATH3_PROC_WIDTH)
			tmp_path_id2 = CPP_SENARIO_PATH3;
		else if (dst_img2->size.w <= CPP_PATH2_PROC_WIDTH)
			tmp_path_id2 = CPP_SENARIO_PATH2;
		else if (dst_img2->size.w <= CPP_PATH0_PROC_WIDTH)
			tmp_path_id2 = CPP_SENARIO_SCALE;

		if ((tmp_path_id1 | tmp_path_id2) ==
			(CPP_SENARIO_PATH3 | CPP_SENARIO_SCALE))
			*path_id = CPP_SENARIO_PATH0_PATH3;
		else if ((tmp_path_id1 | tmp_path_id2) ==
			(CPP_SENARIO_PATH3 | CPP_SENARIO_PATH2))
			*path_id = CPP_SENARIO_PATH2_PATH3;
		else {
			pr_err("cpp start scale path=0x%x error.\n",
					(tmp_path_id1 | tmp_path_id2));
			ret = -1;
			return ret;
		}
	} else if (*path_id == CPP_SENARIO_PATH2_PATH3) {
		if (dst_img1->size.w <= dst_img2->size.w)
			tmp_path_id1 = CPP_SENARIO_PATH3;
		else
			tmp_path_id1 = CPP_SENARIO_PATH2;
	} else if (*path_id == CPP_SENARIO_PATH0_PATH3) {
		if (dst_img1->size.w <= dst_img2->size.w)
			tmp_path_id1 = CPP_SENARIO_PATH3;
		else
		tmp_path_id1 = CPP_SENARIO_SCALE;
	}
	/* step2: prepare parameters, and call corresponding path.*/

	switch (*path_id) {
	case CPP_SENARIO_PATH0_PATH3:
	{
		struct sprd_cpp_path0_path3_cfg_parm parm;
		struct sprd_dcam_img_frm *path0_img;
		struct sprd_dcam_img_frm *path3_img;

		/* input parameter;*/
		parm.input_size = src_img->size;
		parm.input_rect = src_img->rect;
		parm.input_format = src_img->fmt;
		parm.input_addr = src_img->addr_phy;
		parm.input_endian = src_img->data_end;

		/* output parameter;*/
		if (tmp_path_id1 == CPP_SENARIO_PATH3) {
			path0_img = dst_img2;
			path3_img = dst_img1;
		} else {
			path0_img = dst_img1;
			path3_img = dst_img2;
		}

		parm.path0_output_size = path0_img->size;
		parm.path0_output_format = path0_img->fmt;
		parm.path0_output_addr = path0_img->addr_phy;
		parm.path0_output_endian = path0_img->data_end;

		parm.path3_output_size = path3_img->size;
		parm.path3_output_format = path3_img->fmt;
		parm.path3_output_addr = path3_img->addr_phy;
		parm.path3_output_endian = path3_img->data_end;

		/* TODO: other parameters need data from SPRD.*/

		ret = sprd_cpp_ioctl(NULL, SPRD_CPP_IO_START_PATH0_PATH3,
			(unsigned long)(&parm));
		break;
	}
	case CPP_SENARIO_PATH2_PATH3:
	{
		struct sprd_cpp_path2_path3_cfg_parm parm;
		struct sprd_dcam_img_frm *path2_img;
		struct sprd_dcam_img_frm *path3_img;

		/* input parameter;*/
		parm.input_size = src_img->size;
		parm.input_rect = src_img->rect;
		parm.input_format = src_img->fmt;
		parm.input_addr = src_img->addr_phy;
		parm.input_endian = src_img->data_end;

		/* output parameter;*/
		if (tmp_path_id1 == CPP_SENARIO_PATH3) {
			path2_img = dst_img2;
			path3_img = dst_img1;
		} else{
			path2_img = dst_img1;
			path3_img = dst_img2;
		}

		parm.path2_output_size = path2_img->size;
		parm.path2_output_format = path2_img->fmt;
		parm.path2_output_addr = path2_img->addr_phy;
		parm.path2_output_endian = path2_img->data_end;

		parm.path3_output_size = path3_img->size;
		parm.path3_output_format = path3_img->fmt;
		parm.path3_output_addr = path3_img->addr_phy;
		parm.path3_output_endian = path3_img->data_end;

		/* TODO: other parameters need data from SPRD.*/

		ret = sprd_cpp_ioctl(NULL, SPRD_CPP_IO_START_PATH2_PATH3,
			(unsigned long)(&parm));
		break;
	}
	default:
		pr_err("invalid path id %d\n", *path_id);
	break;
	}

	}

	/*here put the logic to choose data path and invoke*/
	/*ioctl command to invokethe selected path with*/
	/*parameters converted.*/

	return ret;
}
EXPORT_SYMBOL(cpp_k_start_scale);

int cpp_k_stop_scale(int path_id)
{
	int ret = -1;

	switch (path_id) {
	case CPP_SENARIO_SCALE:
	{
		ret = sprd_cpp_ioctl(NULL, SPRD_CPP_IO_STOP_SCALE, 0x0);
		break;
	}
	case CPP_SENARIO_PATH2:
	{
		ret = sprd_cpp_ioctl(NULL, SPRD_CPP_IO_STOP_PATH2, 0x0);
		break;
	}
	case CPP_SENARIO_PATH3:
	{
		ret = sprd_cpp_ioctl(NULL, SPRD_CPP_IO_STOP_PATH3, 0x0);
		break;
	}
	case CPP_SENARIO_PATH0_PATH3:
	{
		ret = sprd_cpp_ioctl(NULL, SPRD_CPP_IO_STOP_PATH0_PATH3, 0x0);
		break;
	}
	case CPP_SENARIO_PATH2_PATH3:
	{
		ret = sprd_cpp_ioctl(NULL, SPRD_CPP_IO_STOP_PATH2_PATH3, 0x0);
		break;
	}
	default:
		pr_err("invalid path id %d\n", path_id);
	break;
	}

	return ret;
}
EXPORT_SYMBOL(cpp_k_stop_scale);

static int sprd_cpp_open(struct inode *node, struct file *file)
{
	struct cpp_device *dev = NULL;
	struct miscdevice *md = (struct miscdevice *)file->private_data;
	int ret = 0;

	if (!md)
		return -EFAULT;

	CPP_LOG("sprd_cpp_open enter");
	dev = md->this_device->platform_data;
	ret = cpp_module_enable(dev);
	if (ret)
		pr_err("sprd_cpp_open: fail to enable module\n");
	file->private_data = (void *)dev;

	ret = devm_request_irq(&dev->pdev->dev, dev->irq, cpp_isr_root,
				IRQF_SHARED, "CPP", (void *)dev);
	if (ret < 0) {
		pr_err("uuuu failed to install IRQ %d\n", ret);
		return -EFAULT;
	}
	return 0;
}

static int sprd_cpp_release(struct inode *node, struct file *file)
{
	struct cpp_device *dev = (struct cpp_device *) file->private_data;

	CPP_LOG("sprd_cpp_release\n");
	if (dev == NULL) {
		pr_err("failed to close cpp device\n");
		return -EFAULT;
	}
	devm_free_irq(&dev->pdev->dev, dev->irq, (void *)dev);
	cpp_module_disable(dev);

	file->private_data = NULL;
	return 0;
}

#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
int sprd_cpp_get_cpp_dev(struct device **dev)
{
	if (dev == NULL)
		return -EINVAL;
	*dev = &sprd_cpp_dev->pdev->dev;
	return 0;
}

void sprd_cpp_force_stop(int path_id)
{
	struct path2_path3_if_device *path2_path3_if = NULL;
	struct path2_if_device *path2_if = NULL;

	pr_info("cpp force stop path %d\n", path_id);
	switch (path_id) {
	case CPP_SENARIO_PATH2:
	    {
		path2_if = sprd_cpp_dev->path2_if;
		complete(&path2_if->done_com);
		break;
	    }
	case CPP_SENARIO_PATH2_PATH3:
	    {
		path2_path3_if = sprd_cpp_dev->path2_path3_if;
		path2_if = path2_path3_if->path2_if;
		complete(&path2_if->done_com);
		break;
	    }
	default:
	    {
		pr_debug("cpp force stop no valid path %d\n", path_id);
		break;
	    }
	}
}

#endif

static const struct file_operations cpp_fops = {
	.owner = THIS_MODULE,
	.open = sprd_cpp_open,
	.unlocked_ioctl = sprd_cpp_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = sprd_cpp_ioctl,
#endif
	.release = sprd_cpp_release,
};

static int sprd_cpp_probe(struct platform_device *pdev)
{
	int ret = 0;
	int irq;
	struct resource *res = NULL;
#ifndef CPP_HARDWARE_FPGA
	struct regmap *cam_ahb_gpr;
	struct regmap *pmu_apb_gpr;
#endif

	if (!pdev) {
		pr_err("uuuu Input pdev is NULL\n");
		return -EFAULT;
	}
	CPP_LOG("uuuu sprd_cpp_probe entry\n");
	sprd_cpp_dev = devm_kzalloc(&pdev->dev,
		sizeof(*sprd_cpp_dev), GFP_KERNEL);
	if (!sprd_cpp_dev)
		return -ENOMEM;

	mutex_init(&sprd_cpp_dev->lock);
	spin_lock_init(&sprd_cpp_dev->slock);
	mutex_init(&sprd_cpp_dev->hw_lock);
	mutex_init(&sprd_cpp_dev->dev_lock);

	CPP_LOG("uuuu sprd_cpp_probe get resource\n");
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("uuuu get resource failed ret:%d\n", ret);
		return -ENODEV;
	}
	/*sprd_cpp_dev->io_base = devm_ioremap_nocache(&pdev->dev, res->start,*/
	/*				    resource_size(res));*/
	sprd_cpp_dev->io_base = of_iomap(pdev->dev.of_node, 0);
	if (IS_ERR(sprd_cpp_dev->io_base))
		return PTR_ERR(sprd_cpp_dev->io_base);
#ifndef CPP_HARDWARE_FPGA
	sprd_cpp_dev->cpp_clk = devm_clk_get(&pdev->dev, "clk_cpp");
	if (IS_ERR(sprd_cpp_dev->cpp_clk))
		return PTR_ERR(sprd_cpp_dev->cpp_clk);

	sprd_cpp_dev->cpp_clk_default = clk_get_parent(sprd_cpp_dev->cpp_clk);
	if (IS_ERR(sprd_cpp_dev->cpp_clk_default))
		return PTR_ERR(sprd_cpp_dev->cpp_clk_default);

	sprd_cpp_dev->cpp_clk_parent = devm_clk_get(&pdev->dev,
		"clk_cpp_parent");
	if (IS_ERR(sprd_cpp_dev->cpp_clk_parent))
		return PTR_ERR(sprd_cpp_dev->cpp_clk_parent);

	sprd_cpp_dev->cpp_eb =
		of_clk_get_by_name(pdev->dev.of_node, "cpp_eb");
	if (IS_ERR(sprd_cpp_dev->cpp_eb))
		return PTR_ERR(sprd_cpp_dev->cpp_eb);
	CPP_LOG("uuuu get cpp_eb clk ok");

	sprd_cpp_dev->jpg_cpp_mtx_eb =
		of_clk_get_by_name(pdev->dev.of_node, "jpg_cpp_mtx_eb");
	if (IS_ERR(sprd_cpp_dev->jpg_cpp_mtx_eb))
		return PTR_ERR(sprd_cpp_dev->jpg_cpp_mtx_eb);
	CPP_LOG("uuuu get jpg_cpp_mtx_eb clk ok");

	sprd_cpp_dev->cpp_axi_eb =
	of_clk_get_by_name(pdev->dev.of_node, "cpp_axi_eb");
	if (IS_ERR(sprd_cpp_dev->cpp_axi_eb))
		return PTR_ERR(sprd_cpp_dev->cpp_axi_eb);
	CPP_LOG("uuuu get cpp_axi_eb ok");

	cam_ahb_gpr = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
			"sprd,syscon-cam-ahb");
	if (IS_ERR(cam_ahb_gpr))
		return PTR_ERR(cam_ahb_gpr);
	sprd_cpp_dev->cam_ahb_gpr = cam_ahb_gpr;

	pmu_apb_gpr = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
			"sprd,syscon-pmu-apb");
	if (IS_ERR(pmu_apb_gpr))
		return PTR_ERR(pmu_apb_gpr);
#endif
	CPP_LOG("uuuu start to register misc device");
	sprd_cpp_dev->md.minor = MISC_DYNAMIC_MINOR;
	sprd_cpp_dev->md.name = CPP_DEVICE_NAME;
	sprd_cpp_dev->md.fops = &cpp_fops;
	sprd_cpp_dev->md.parent = NULL;
	ret = misc_register(&sprd_cpp_dev->md);
	if (ret) {
		pr_err("failed to register misc devices\n");
		return ret;
	}
	CPP_LOG("uuuu register misc devices ok");
/*	irq = platform_get_irq(pdev, 0); */
	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (irq <= 0) {
		pr_err("uuuu failed to get IRQ\n");
		ret = -ENXIO;
		goto fail;
	}
	CPP_LOG("uuuu get irq %d success\n", irq);
	sprd_cpp_dev->irq = irq;

	sprd_cpp_dev->pdev = pdev;
	sprd_cpp_dev->md.this_device->platform_data = (void *)sprd_cpp_dev;
	platform_set_drvdata(pdev, (void *)sprd_cpp_dev);
	pr_info("uuuu probing successful\n");
#if 0
	/* workaround for power when start up */
	regmap_update_bits(pmu_apb_gpr,
			   REG_PMU_APB_PD_CAM_SYS_CFG,
			   BIT_PMU_APB_PD_CAM_SYS_AUTO_SHUTDOWN_EN,
			   ~(unsigned int)
			   BIT_PMU_APB_PD_CAM_SYS_AUTO_SHUTDOWN_EN);
	regmap_update_bits(pmu_apb_gpr,
			   REG_PMU_APB_PD_CAM_SYS_CFG,
			   BIT_PMU_APB_PD_CAM_SYS_FORCE_SHUTDOWN,
			   BIT_PMU_APB_PD_CAM_SYS_FORCE_SHUTDOWN);
#endif
	return 0;

fail:
	misc_deregister(&sprd_cpp_dev->md);
	return ret;
}

static int sprd_cpp_remove(struct platform_device *pdev)
{
	struct cpp_device *dev = platform_get_drvdata(pdev);

	CPP_LOG("sprd_cpp_remove\n");
	misc_deregister(&dev->md);

	return 0;
}

static const struct of_device_id of_match_table_rot[] = {
	{ .compatible = "sprd,cpp", },
	{},
};

static struct platform_driver sprd_cpp_driver = {
	.probe = sprd_cpp_probe,
	.remove = sprd_cpp_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = CPP_DEVICE_NAME,
		.of_match_table = of_match_ptr(of_match_table_rot),
	},
};

static int __init sprd_cpp_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&sprd_cpp_driver);
	CPP_LOG("uuuu init ret %d\n", ret);

	return ret;
}

static void sprd_cpp_exit(void)
{
	platform_driver_unregister(&sprd_cpp_driver);
}

module_init(sprd_cpp_init);
module_exit(sprd_cpp_exit);
MODULE_DESCRIPTION("Cpp Driver");
MODULE_LICENSE("GPL");

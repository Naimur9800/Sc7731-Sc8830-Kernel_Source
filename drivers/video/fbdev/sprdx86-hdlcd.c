/*
 * drivers/video/sprdx86-hdlcd.c
 *
 * Copyright (C) 2015 SPRD Limited
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 *
 *  SPRD DISPC Controller
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/memblock.h>
#include <linux/sprdx86-hdlcd.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/sprd_ion.h>

#ifdef HDLCD_COUNT_BUFFERUNDERRUNS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#endif

#include "edid.h"

#ifdef CONFIG_SERIAL_AMBA_PCU_UART
int get_edid(u8 *msgbuf);
#else
#endif

#define to_hdlcd_device(info)	container_of(info, struct hdlcd_device, fb)

#ifndef CONFIG_FB_FPGA_NODISPC
#define WRITE_HDLCD_REG(reg, value)	writel((value), hdlcd->base + (reg))
#define READ_HDLCD_REG(reg)		readl(hdlcd->base + (reg))
#else
unsigned long dummy_func(void)
{
    return 0;
}
#define WRITE_HDLCD_REG(reg, value)	dummy_func()
#define READ_HDLCD_REG(reg)		dummy_func()
#endif

static const struct of_device_id hdlcd_of_matches[] = {
	{.compatible = "sprd,x86-hdlcd-iwhale2"},
	{.compatible = "sprd,x86-hdlcd-isharkl2"},
	{},
};

MODULE_DEVICE_TABLE(of, hdlcd_of_matches);
/* Framebuffer size.  */
static unsigned long framebuffer_size;
const char *pro_name;

#ifdef HDLCD_COUNT_BUFFERUNDERRUNS
static unsigned long buffer_underrun_events;
static DEFINE_SPINLOCK(hdlcd_underrun_lock);

static void hdlcd_underrun_set(unsigned long val)
{
	spin_lock(&hdlcd_underrun_lock);
	buffer_underrun_events = val;
	spin_unlock(&hdlcd_underrun_lock);
}

static unsigned long hdlcd_underrun_get(void)
{
	unsigned long val;

	spin_lock(&hdlcd_underrun_lock);
	val = buffer_underrun_events;
	spin_unlock(&hdlcd_underrun_lock);
	return val;
}

#ifdef CONFIG_PROC_FS
static int hdlcd_underrun_show(struct seq_file *m, void *v)
{
	unsigned char underrun_string[32];

	snprintf(underrun_string, 32, "%lu\n", hdlcd_underrun_get());
	seq_puts(m, underrun_string);
	return 0;
}

static int proc_hdlcd_underrun_open(struct inode *inode, struct file *file)
{
	return single_open(file, hdlcd_underrun_show, NULL);
}

static const struct file_operations proc_hdlcd_underrun_operations = {
	.open = proc_hdlcd_underrun_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int hdlcd_underrun_init(void)
{
	hdlcd_underrun_set(0);
	proc_create("hdlcd_underrun", 0, NULL, &proc_hdlcd_underrun_operations);
	return 0;
}

static void hdlcd_underrun_close(void)
{
	remove_proc_entry("hdlcd_underrun", NULL);
}
#else
static int hdlcd_underrun_init(void)
{
	return 0;
}

static void hdlcd_underrun_close(void)
{
}
#endif
#endif


static struct fb_var_screeninfo cached_var_screeninfo;

static struct fb_videomode hdlcd_default_mode = {
	.refresh = 60,
	.xres = 1280,
	.yres = 720,
	.pixclock = 15189,
	.left_margin = 10,
	.right_margin = 3,
	.upper_margin = 5,
	.lower_margin = 4,
	.hsync_len = 10,
	.vsync_len = 5,
	.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	.vmode = FB_VMODE_NONINTERLACED
};

#ifndef CONFIG_FB_FPGA_NODISPC
void sci_glb_write(unsigned char *name, struct hdlcd_device *hdlcd,
	unsigned int reg, unsigned int value)
{
	struct regmap *reg_glb;
	struct device_node *np;
	np = hdlcd->dev->of_node;
	reg_glb = syscon_regmap_lookup_by_phandle(np, name);
	regmap_write(reg_glb, reg, value);
}

void sci_glb_read(unsigned char *name, struct hdlcd_device *hdlcd,
	unsigned int reg, unsigned int *value)
{
	struct regmap *reg_glb;
	struct device_node *np;
	np = hdlcd->dev->of_node;
	reg_glb =  syscon_regmap_lookup_by_phandle(np, name);
	regmap_read(reg_glb, reg, value);
}
#else
void sci_glb_write(unsigned char *name, struct hdlcd_device *hdlcd,
	unsigned int reg, unsigned int value)
{
}

void sci_glb_read(unsigned char *name, struct hdlcd_device *hdlcd,
	unsigned int reg, unsigned int *value)
{
}
#endif
static void hdlcd_glb_reg_config(struct hdlcd_device *hdlcd)
{
	static unsigned int reg_val;

	if (!strcmp(pro_name, "sprd,x86-hdlcd-isharkl2")) {
		/* 0xe42b0094 bit 24/25 -> 0 */
		sci_glb_read("sprd,syscon-aon-pwu-apb", hdlcd, 0x94, &reg_val);
		reg_val = reg_val & 0xfcffffff;
		sci_glb_write("sprd,syscon-aon-pwu-apb", hdlcd, 0x94, reg_val);

		udelay(5);

		/* 0xe42e0004 bit 29 */
		sci_glb_read("sprd,syscon-aon-apb", hdlcd, 0x04, &reg_val);
		reg_val = reg_val | 0x20000000;
		sci_glb_write("sprd,syscon-aon-apb", hdlcd, 0x04, reg_val);

		/* 0xe42b02cc bit 0 */
		sci_glb_read("sprd,syscon-aon-pwu-apb", hdlcd, 0x2cc, &reg_val);
		reg_val = reg_val | 0x1;
		sci_glb_write("sprd,syscon-aon-pwu-apb", hdlcd, 0x2cc, reg_val);

		/* 0xd3100000 bit 4/3/2/1/0 */
		sci_glb_read("sprd,syscon-dispc-ahb", hdlcd, 0x00, &reg_val);
		reg_val = reg_val | 0x1f;
		sci_glb_write("sprd,syscon-dispc-ahb", hdlcd, 0x00, reg_val);

		/* 0xd2100000 -> 0xffffffff */
		sci_glb_read("sprd,syscon-lpc-ahb", hdlcd, 0x00, &reg_val);
		reg_val = reg_val | 0xffffffff;
		sci_glb_write("sprd,syscon-lpc-ahb", hdlcd, 0x00, reg_val);

		/* 0xd2100008 -> 0xffffffff */
		sci_glb_read("sprd,syscon-lpc-ahb", hdlcd, 0x08, &reg_val);
		reg_val = reg_val | 0xffffffff;
		sci_glb_write("sprd,syscon-lpc-ahb", hdlcd, 0x08, reg_val);

		/* 0xd210002c -> 0xffffffff */
		sci_glb_read("sprd,syscon-lpc-ahb", hdlcd, 0x2c, &reg_val);
		reg_val = reg_val | 0xffffffff;
		sci_glb_write("sprd,syscon-lpc-ahb", hdlcd, 0x2c, reg_val);

		/* 0xd3000024 bit 0/1 */
		sci_glb_read("sprd,syscon-ap-cam-clk", hdlcd, 0x24, &reg_val);
		reg_val = reg_val | 0x3;
		sci_glb_write("sprd,syscon-ap-cam-clk", hdlcd, 0x24, reg_val);

		/* read global reg */
		sci_glb_read("sprd,syscon-aon-pwu-apb", hdlcd, 0x94, &reg_val);
		dev_err(hdlcd->dev, "0xe42b0094 = 0x%x\n", reg_val);

		sci_glb_read("sprd,syscon-aon-apb", hdlcd, 0x04, &reg_val);
		dev_err(hdlcd->dev, "0xe42e0004 = 0x%x\n", reg_val);

		sci_glb_read("sprd,syscon-aon-pwu-apb", hdlcd, 0x2cc, &reg_val);
		dev_err(hdlcd->dev, "0xe42b02cc = 0x%x\n", reg_val);

		sci_glb_read("sprd,syscon-dispc-ahb", hdlcd, 0x00, &reg_val);
		dev_err(hdlcd->dev, "0xd3100000 = 0x%x\n", reg_val);

		sci_glb_read("sprd,syscon-lpc-ahb", hdlcd, 0x00, &reg_val);
		dev_err(hdlcd->dev, "0xd2100000 = 0x%x\n", reg_val);

		sci_glb_read("sprd,syscon-lpc-ahb", hdlcd, 0x08, &reg_val);
		dev_err(hdlcd->dev, "0xd2100008 = 0x%x\n", reg_val);

		sci_glb_read("sprd,syscon-lpc-ahb", hdlcd, 0x2c, &reg_val);
		dev_err(hdlcd->dev, "0xd210002c = 0x%x\n", reg_val);

		sci_glb_read("sprd,syscon-ap-cam-clk", hdlcd, 0x24, &reg_val);
		dev_err(hdlcd->dev, "0xd3000024 = 0x%x\n", reg_val);
	} else if (!strcmp(pro_name, "sprd,x86-hdlcd-iwhale2")) {
		/* 0xe42b009c bit 24/25 -> 0 */
		sci_glb_read("sprd,syscon-aon-pwu-apb", hdlcd, 0x9c, &reg_val);
		reg_val = reg_val & 0xfcffffff;
		sci_glb_write("sprd,syscon-aon-pwu-apb", hdlcd, 0x9c, reg_val);

		/* 0xe42b0094 bit 24/25 -> 0 */
		sci_glb_read("sprd,syscon-aon-pwu-apb", hdlcd, 0x94, &reg_val);
		reg_val = reg_val & 0xfcffffff;
		sci_glb_write("sprd,syscon-aon-pwu-apb", hdlcd, 0x94, reg_val);

		udelay(5);

		/* 0xe42e0004 bit 29/30 */
		sci_glb_read("sprd,syscon-aon-apb", hdlcd, 0x04, &reg_val);
		reg_val = reg_val | BIT(29) | BIT(30);
		sci_glb_write("sprd,syscon-aon-apb", hdlcd, 0x04, reg_val);

		/* 0xd3100000 bit 0/2/9/16/17/18 */
		sci_glb_read("sprd,syscon-dispc-ahb", hdlcd, 0x00, &reg_val);
		reg_val = reg_val | BIT(0) | BIT(2) | BIT(7) | BIT(8) | BIT(9) | BIT(16) | BIT(17) | BIT(18);
		sci_glb_write("sprd,syscon-dispc-ahb", hdlcd, 0x00, reg_val);

		/* 0xd3100008 bit 7/9 */
		sci_glb_read("sprd,syscon-dispc-ahb", hdlcd, 0x08, &reg_val);
		reg_val = reg_val | BIT(7) | BIT(9);
		sci_glb_write("sprd,syscon-dispc-ahb", hdlcd, 0x08, reg_val);

		/* 0xd2100034 bit 10/13/19 */
		sci_glb_read("sprd,syscon-lpc-ahb", hdlcd, 0x34, &reg_val);
		reg_val = reg_val | BIT(10) | BIT(13) | BIT(16) | BIT(19);
		sci_glb_write("sprd,syscon-lpc-ahb", hdlcd, 0x34, reg_val);

		/* 0xe7b00000 bit 19 */
		sci_glb_read("sprd,syscon-ap-apb", hdlcd, 0x00, &reg_val);
		reg_val = reg_val | BIT(19);
		sci_glb_write("sprd,syscon-ap-apb", hdlcd, 0x00, reg_val);


		/* read global reg */
		sci_glb_read("sprd,syscon-aon-pwu-apb", hdlcd, 0x9c, &reg_val);
		dev_err(hdlcd->dev, "0xe42b009c = 0x%x\n", reg_val);

		sci_glb_read("sprd,syscon-aon-pwu-apb", hdlcd, 0x94, &reg_val);
		dev_err(hdlcd->dev, "0xe42b0094 = 0x%x\n", reg_val);

		sci_glb_read("sprd,syscon-aon-apb", hdlcd, 0x04, &reg_val);
		dev_err(hdlcd->dev, "0xe42e0004 = 0x%x\n", reg_val);

		sci_glb_read("sprd,syscon-dispc-ahb", hdlcd, 0x00, &reg_val);
		dev_err(hdlcd->dev, "0xd3100000 = 0x%x\n", reg_val);

		sci_glb_read("sprd,syscon-dispc-ahb", hdlcd, 0x08, &reg_val);
		dev_err(hdlcd->dev, "0xd3100008 = 0x%x\n", reg_val);

		sci_glb_read("sprd,syscon-lpc-ahb", hdlcd, 0x34, &reg_val);
		dev_err(hdlcd->dev, "0xd2100034 = 0x%x\n", reg_val);

		sci_glb_read("sprd,syscon-ap-apb", hdlcd, 0x00, &reg_val);
		dev_err(hdlcd->dev, "0xe7b00000 = 0x%x\n", reg_val);
	}
}

static inline void hdlcd_enable(struct hdlcd_device *hdlcd)
{
	unsigned int reg_value = 0;

	dev_dbg(hdlcd->dev, "HDLCD: output enabled\n");

	WRITE_HDLCD_REG(DISPC_CTRL, 0x1);

	/* clear interrupt */
	WRITE_HDLCD_REG(DISPC_INT_EN, 0x0);
	WRITE_HDLCD_REG(DISPC_INT_CLR, 0x3f);
	usleep_range(10*1000, 10*1000+500);

	reg_value = DISPC_INT_ERR | DISPC_INT_DPI_VSYNC;
	WRITE_HDLCD_REG(DISPC_INT_EN, reg_value);
}

static inline void hdlcd_disable(struct hdlcd_device *hdlcd)
{
	dev_dbg(hdlcd->dev, "HDLCD: output disabled\n");
	WRITE_HDLCD_REG(DISPC_CTRL, 0x0);

	/* do dispc reset */
	WRITE_HDLCD_REG(DISPC_CTRL, 0x1);
	usleep_range(20*1000, 20*1000+500);
	WRITE_HDLCD_REG(DISPC_CTRL, 0x0);
	usleep_range(10*1000, 10*1000+500);
	WRITE_HDLCD_REG(DISPC_CTRL, 0x1);

	/* wait 10ms util the lcd is stable */
	usleep_range(120*1000, 120*1000+500);
}

static void hdlcd_dispc_run(struct hdlcd_device *hdlcd)
{
	WRITE_HDLCD_REG(DISPC_DPI_CTRL, 0xb0);
	WRITE_HDLCD_REG(DISPC_CTRL, 0x11);
}

static int hdlcd_set_bitfields(struct hdlcd_device *hdlcd,
			       struct fb_var_screeninfo *var)
{
	int ret = 0;

	memset(&var->transp, 0, sizeof(var->transp));
	var->red.msb_right = 0;
	var->green.msb_right = 0;
	var->blue.msb_right = 0;
	var->blue.offset = 0;

	switch (var->bits_per_pixel) {
	case 8:
		/* pseudocolor */
		var->red.length = 8;
		var->green.length = 8;
		var->blue.length = 8;
		break;
	case 16:
		/* 565 format */
		var->red.length = 5;
		var->green.length = 6;
		var->blue.length = 5;
		break;
	case 32:
		var->transp.length = 8;
	case 24:
		var->red.length = 8;
		var->green.length = 8;
		var->blue.length = 8;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (!ret) {
		if (var->bits_per_pixel != 32) {
			var->green.offset = var->blue.length;
			var->red.offset = var->green.offset + var->green.length;
		} else {
			/* Previously, the byte ordering for 32-bit color was
			 * (msb)<alpha><red><green><blue>(lsb)
			 * but this does not match what android expects and
			 * the colors are odd. Instead, use
			 * <alpha><blue><green><red>
			 * Since we tell fb what we are doing, console
			 * , X and directfb access should work fine.
			 */
			var->green.offset = var->red.length;
			var->blue.offset =
			    var->green.offset + var->green.length;
			var->transp.offset =
			    var->blue.offset + var->blue.length;
			var->red.offset = 0;
		}
	}

	return ret;
}

static int hdlcd_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct hdlcd_device *hdlcd = to_hdlcd_device(info);
	int bytes_per_pixel = var->bits_per_pixel / 8;

#ifdef HDLCD_NO_VIRTUAL_SCREEN
	var->yres_virtual = var->yres;
#else
	var->yres_virtual = 3 * var->yres;
#endif

	if ((var->xres_virtual * bytes_per_pixel * var->yres_virtual) >
	    hdlcd->fb.fix.smem_len)
		return -ENOMEM;

	if (var->xres > HDLCD_MAX_XRES || var->yres > HDLCD_MAX_YRES)
		return -EINVAL;

	/* make sure the bitfields are set appropriately */
	return hdlcd_set_bitfields(hdlcd, var);
}

/* prototype */
static int hdlcd_pan_display(struct fb_var_screeninfo *var,
			     struct fb_info *info);

static int hdlcd_set_par(struct fb_info *info)
{
	struct hdlcd_device *hdlcd = to_hdlcd_device(info);
	int bytes_per_pixel = hdlcd->fb.var.bits_per_pixel / 8;
	int old_yoffset;
	static unsigned int reg_value;
	u32 version;

	dev_err(hdlcd->dev, "hdlcd: hdlcd_set_par\n");

	/* check for shortcuts */
	old_yoffset = cached_var_screeninfo.yoffset;
	cached_var_screeninfo.yoffset = info->var.yoffset;
	if (!memcmp(&info->var, &cached_var_screeninfo,
		    sizeof(struct fb_var_screeninfo))) {
		if (old_yoffset != info->var.yoffset) {
			/* we only changed yoffset, and we already
			 * already recorded it a couple lines up
			 */
			hdlcd_pan_display(&info->var, info);
		}
		/* or no change */
		return 0;
	}

	hdlcd->fb.fix.line_length = hdlcd->fb.var.xres * bytes_per_pixel;

	if (hdlcd->fb.var.bits_per_pixel >= 16)
		hdlcd->fb.fix.visual = FB_VISUAL_TRUECOLOR;
	else
		hdlcd->fb.fix.visual = FB_VISUAL_PSEUDOCOLOR;

	memcpy(&cached_var_screeninfo, &info->var,
	       sizeof(struct fb_var_screeninfo));

	if (hdlcd->is_set_par == 0) {
		dev_err(hdlcd->dev, "hdlcd: hdlcd_set_par first excute\n");

		/* global register config */
		hdlcd_glb_reg_config(hdlcd);

		hdlcd_disable(hdlcd);

		reg_value = 0x1;
		WRITE_HDLCD_REG(DISPC_CTRL, reg_value);

		reg_value = (hdlcd->fb.var.yres<<16) | hdlcd->fb.var.xres;
		WRITE_HDLCD_REG(DISPC_SIZE_XY, reg_value);
		WRITE_HDLCD_REG(DISPC_3D_CTRL, 0);
		WRITE_HDLCD_REG(DISPC_IMG_CTRL, 0);
		WRITE_HDLCD_REG(DISPC_BG_COLOR, 0xff0000);

		reg_value = 0x8035;
		WRITE_HDLCD_REG(DISPC_OSD_CTRL, reg_value);
		reg_value = (hdlcd->fb.var.yres<<16) | hdlcd->fb.var.xres;
		WRITE_HDLCD_REG(DISPC_OSD_SIZE_XY, reg_value);
		reg_value = hdlcd->fb.var.xres;
		WRITE_HDLCD_REG(DISPC_OSD_PITCH, reg_value);
		WRITE_HDLCD_REG(DISPC_OSD_DISP_XY, 0);
		WRITE_HDLCD_REG(DISPC_OSD_ALPHA, 0xff);
		WRITE_HDLCD_REG(DISPC_OSD_CK, 0);

		WRITE_HDLCD_REG(DISPC_DPI_CTRL, 0x90);

		reg_value = (hdlcd_default_mode.right_margin<<20) |
			(hdlcd_default_mode.left_margin<<8) | hdlcd_default_mode.hsync_len;
		WRITE_HDLCD_REG(DISPC_DPI_H_TIMING, reg_value);
		reg_value = (hdlcd_default_mode.upper_margin<<20) |
			(hdlcd_default_mode.lower_margin<<8) | hdlcd_default_mode.vsync_len;
		WRITE_HDLCD_REG(DISPC_DPI_V_TIMING, reg_value);
		WRITE_HDLCD_REG(DISPC_OSD_BASE_ADDR, 0x27500000);

		clk_set_rate(hdlcd->clk, (1000000000 / hdlcd->fb.var.pixclock) * 1000);
		clk_enable(hdlcd->clk);
		hdlcd_enable(hdlcd);

		hdlcd_dispc_run(hdlcd);

		hdlcd->is_set_par = 1;

		version = readl(hdlcd->base + DISPC_VERSION);
		dev_err(hdlcd->dev, "HDLCD: found SPRD DISPC HDLCD version 0x%x\n", version);
	}
	return 0;
}

static int hdlcd_setcolreg(unsigned int regno, unsigned int red,
			   unsigned int green, unsigned int blue,
			   unsigned int transp, struct fb_info *info)
{
	if (regno < 16) {
		u32 *pal = info->pseudo_palette;

		pal[regno] = ((red >> 8) << info->var.red.offset) |
		    ((green >> 8) << info->var.green.offset) |
		    ((blue >> 8) << info->var.blue.offset);
	}

	return 0;
}

static irqreturn_t hdlcd_irq(int irq, void *data)
{
	struct hdlcd_device *hdlcd = data;
	unsigned long irq_mask, irq_status;

	irq_mask = READ_HDLCD_REG(DISPC_INT_EN);
	irq_status = READ_HDLCD_REG(DISPC_INT_STATUS);

	if (irq_status & DISPC_INT_ERR) {
		dev_err(hdlcd->dev, "hdlcd: dispc underflow\n");
		WRITE_HDLCD_REG(DISPC_INT_CLR, DISPC_INT_ERR);
		WRITE_HDLCD_REG(DISPC_INT_EN,
				irq_mask & ~DISPC_INT_ERR);
	}

	if (irq_status & DISPC_INT_DPI_VSYNC) {
		dev_dbg(hdlcd->dev, "hdlcd: dispc vsync\n");
		WRITE_HDLCD_REG(DISPC_INT_CLR, DISPC_INT_DPI_VSYNC);
		complete(&hdlcd->vsync_completion);
	}

	if (irq_status & DISPC_INT_UPDATE_DONE) {
		dev_dbg(hdlcd->dev, "hdlcdb: dispc done\n");
		WRITE_HDLCD_REG(DISPC_INT_CLR, DISPC_INT_UPDATE_DONE);
	}
	return IRQ_HANDLED;
}

static int hdlcd_wait_for_vsync(struct fb_info *info)
{
	struct hdlcd_device *hdlcd = to_hdlcd_device(info);
	unsigned long irq_mask;
	int err;

	/* enable VSYNC interrupt */
	irq_mask = READ_HDLCD_REG(DISPC_INT_EN);
	WRITE_HDLCD_REG(DISPC_INT_EN, irq_mask | DISPC_INT_DPI_VSYNC);

	err =
	    wait_for_completion_interruptible_timeout(&hdlcd->vsync_completion,
						      msecs_to_jiffies(900));

	if (!err) {
		dev_err(hdlcd->dev, "hdlcd: dispc_wait_for_vsync time out\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int hdlcd_blank(int blank_mode, struct fb_info *info)
{
	struct hdlcd_device *hdlcd = to_hdlcd_device(info);

	switch (blank_mode) {
	case FB_BLANK_POWERDOWN:
		clk_disable(hdlcd->clk);
	case FB_BLANK_NORMAL:
		hdlcd_disable(hdlcd);
		break;
	case FB_BLANK_UNBLANK:
		clk_enable(hdlcd->clk);
		hdlcd_enable(hdlcd);
		break;
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	default:
		return 1;
	}

	return 0;
}

static void hdlcd_mmap_open(struct vm_area_struct *vma)
{
}

static void hdlcd_mmap_close(struct vm_area_struct *vma)
{
}

static struct vm_operations_struct hdlcd_mmap_ops = {
	.open = hdlcd_mmap_open,
	.close = hdlcd_mmap_close,
};

static int hdlcd_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	struct hdlcd_device *hdlcd = to_hdlcd_device(info);
	unsigned long off;
	unsigned long start;
	unsigned long len = hdlcd->fb.fix.smem_len;

	if (vma->vm_end - vma->vm_start == 0)
		return 0;
	if (vma->vm_pgoff > (~0UL >> PAGE_SHIFT))
		return -EINVAL;

	off = vma->vm_pgoff << PAGE_SHIFT;
	if ((off >= len) || (vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;

	start = hdlcd->fb.fix.smem_start;
	off += start;

	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	vma->vm_ops = &hdlcd_mmap_ops;
	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

#ifndef CONFIG_ION_SPRD
int sprd_ion_get_phys_addr(int fd_buffer, struct dma_buf *dmabuf, unsigned long *phys_addr, size_t *size)
{
    return 0;
}
#endif

static int hdlcd_pan_display(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
	struct hdlcd_device *hdlcd = to_hdlcd_device(info);
	unsigned int base;

	dev_err(hdlcd->dev, "hdlcd: pan_display\n");
	if ((hdlcd->probe_end == 0) || (hdlcd->is_set_par == 0))
		return 0;
	if (var->reserved[0] > 0) {
	/*GPU use fd mode*/
	    int ret = 0;
	    unsigned long iova = 0;
	    size_t size = 0;
	    int fd_buffer = -1;
	    struct dma_buf *dmabuf = dma_buf_get(var->reserved[0]);
	    if (dmabuf > 0) {
		    dev_info(hdlcd->dev, "hdlcd: get dma buffer success!\n");
		    ret = sprd_ion_get_phys_addr(fd_buffer, dmabuf, &iova, &size);
		    if (ret < 0) {
			    dev_err(hdlcd->dev, "hdlcd: sprd_ion_get_disp_addr failed:%d!\n", ret);

		    } else {
			    dev_err(hdlcd->dev,
				 "hdlcd: get_disp_addr success(iova:0x%lx size:%zu)\n",
				 iova, size);
			    WRITE_HDLCD_REG(DISPC_OSD_BASE_ADDR, iova);
		    }
	    } else {
		    dev_err(hdlcd->dev, "hdlcd: get dma buffer failed!\n");
	    }
	} else {
	/* normal mode */
	    hdlcd->fb.var.yoffset = var->yoffset;
	    base = hdlcd->fb.fix.smem_start + hdlcd->fb.fix.line_length * hdlcd->fb.var.yoffset;
	    WRITE_HDLCD_REG(DISPC_OSD_BASE_ADDR, base);
	}
	hdlcd_dispc_run(hdlcd);

	hdlcd_wait_for_vsync(info);

	return 0;
}

static int hdlcd_ioctl(struct fb_info *info, unsigned int cmd,
		       unsigned long arg)
{
	int err;

	switch (cmd) {
	case FBIO_WAITFORVSYNC:
		err = hdlcd_wait_for_vsync(info);
		break;
	default:
		err = -ENOIOCTLCMD;
		break;
	}

	return err;
}

static struct fb_ops hdlcd_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = hdlcd_check_var,
	.fb_set_par = hdlcd_set_par,
	.fb_setcolreg = hdlcd_setcolreg,
	.fb_blank = hdlcd_blank,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_mmap = hdlcd_mmap,
	.fb_pan_display = hdlcd_pan_display,
	.fb_ioctl = hdlcd_ioctl,
	.fb_compat_ioctl = hdlcd_ioctl
};

static int hdlcd_setup(struct hdlcd_device *hdlcd)
{
	int err = -EFAULT;

	hdlcd->fb.device = hdlcd->dev;

	hdlcd->clk = clk_get(hdlcd->dev, NULL);
	if (IS_ERR(hdlcd->clk)) {
		dev_err(hdlcd->dev, "HDLCD: unable to find clock data\n");
		return PTR_ERR(hdlcd->clk);
	}

	err = clk_prepare(hdlcd->clk);
	if (err)
		goto clk_prepare_err;

	hdlcd->base =
	    ioremap_nocache(hdlcd->fb.fix.mmio_start, hdlcd->fb.fix.mmio_len);
	if (!hdlcd->base) {
		dev_err(hdlcd->dev, "HDLCD: unable to map registers\n");
		goto remap_err;
	}

	hdlcd->fb.pseudo_palette = kmalloc(sizeof(u32) * 16, GFP_KERNEL);
	if (!hdlcd->fb.pseudo_palette) {
		dev_err(hdlcd->dev,
			"HDLCD: unable to allocate pseudo_palette memory\n");
		err = -ENOMEM;
		goto kmalloc_err;
	}

	strcpy(hdlcd->fb.fix.id, "hdlcd");
	hdlcd->fb.fbops = &hdlcd_ops;
	hdlcd->fb.flags = FBINFO_FLAG_DEFAULT;	/* | FBINFO_VIRTFB */

	hdlcd->fb.fix.type = FB_TYPE_PACKED_PIXELS;
	hdlcd->fb.fix.type_aux = 0;
	hdlcd->fb.fix.xpanstep = 0;
	hdlcd->fb.fix.ypanstep = 1;
	hdlcd->fb.fix.ywrapstep = 0;
	hdlcd->fb.fix.accel = FB_ACCEL_NONE;

	hdlcd->fb.var.nonstd = 0;
	hdlcd->fb.var.activate = FB_ACTIVATE_NOW;
	hdlcd->fb.var.height = -1;
	hdlcd->fb.var.width = -1;
	hdlcd->fb.var.accel_flags = 0;

	init_completion(&hdlcd->vsync_completion);

	if (hdlcd->edid) {
		/* build modedb from EDID */
		fb_edid_to_monspecs(hdlcd->edid, &hdlcd->fb.monspecs);
		fb_videomode_to_modelist(hdlcd->fb.monspecs.modedb,
					 hdlcd->fb.monspecs.modedb_len,
					 &hdlcd->fb.modelist);
		fb_find_mode(&hdlcd->fb.var, &hdlcd->fb, NULL,
			     hdlcd->fb.monspecs.modedb,
			     hdlcd->fb.monspecs.modedb_len,
			     &hdlcd_default_mode, 32);
	} else {
		hdlcd->fb.monspecs.hfmin = 0;
		hdlcd->fb.monspecs.hfmax = 100000;
		hdlcd->fb.monspecs.vfmin = 0;
		hdlcd->fb.monspecs.vfmax = 400;
		hdlcd->fb.monspecs.dclkmin = 1000000;
		hdlcd->fb.monspecs.dclkmax = 100000000;
		fb_find_mode(&hdlcd->fb.var, &hdlcd->fb, NULL, NULL, 0,
			     &hdlcd_default_mode, 32);
	}

	dev_err(hdlcd->dev, "using %dx%d-%d@%d mode\n", hdlcd->fb.var.xres,
		 hdlcd->fb.var.yres, hdlcd->fb.var.bits_per_pixel,
		 hdlcd->fb.mode ? hdlcd->fb.mode->refresh : 60);
	hdlcd->fb.var.xres_virtual = hdlcd->fb.var.xres;
#ifdef HDLCD_NO_VIRTUAL_SCREEN
	hdlcd->fb.var.yres_virtual = hdlcd->fb.var.yres;
#else
	hdlcd->fb.var.yres_virtual = hdlcd->fb.var.yres * 3;
#endif

	/* initialise and set the palette */
	if (fb_alloc_cmap(&hdlcd->fb.cmap, NR_PALETTE, 0)) {
		dev_err(hdlcd->dev, "failed to allocate cmap memory\n");
		err = -ENOMEM;
		goto setup_err;
	}
	fb_set_cmap(&hdlcd->fb.cmap, &hdlcd->fb);

	WRITE_HDLCD_REG(DISPC_OSD_BASE_ADDR, hdlcd->fb.fix.smem_start);

	/* Ensure interrupts are disabled */
	WRITE_HDLCD_REG(DISPC_INT_EN, 0);
	fb_set_var(&hdlcd->fb, &hdlcd->fb.var);

	if (!register_framebuffer(&hdlcd->fb))
		return 0;

	dev_err(hdlcd->dev, "HDLCD: cannot register framebuffer\n");

	fb_dealloc_cmap(&hdlcd->fb.cmap);
setup_err:
	iounmap(hdlcd->base);
kmalloc_err:
	kfree(hdlcd->fb.pseudo_palette);
remap_err:
	clk_unprepare(hdlcd->clk);
clk_prepare_err:
	clk_put(hdlcd->clk);
	return err;
}

static inline unsigned char atohex(u8 data)
{
	if (!isxdigit(data))
		return 0;
	/* truncate the upper nibble and add 9 to non-digit values */
	return (data > 0x39) ? ((data & 0xf) + 9) : (data & 0xf);
}

/* EDID data is passed from devicetree in a literal string that can contain
   spaces and the hexadecimal dump of the data */
static int parse_edid_data(struct hdlcd_device *hdlcd, const u8 *edid_data,
			   int data_len)
{
	int i, j;

	if (!edid_data)
		return -EINVAL;

	hdlcd->edid = kzalloc(EDID_LENGTH, GFP_KERNEL);
	if (!hdlcd->edid)
		return -ENOMEM;

	for (i = 0, j = 0; i < data_len; i++) {
		if (isspace(edid_data[i]))
			continue;
		hdlcd->edid[j++] = atohex(edid_data[i]);
		if (j >= EDID_LENGTH)
			break;
	}

	if (j < EDID_LENGTH) {
		kfree(hdlcd->edid);
		hdlcd->edid = NULL;
		return -EINVAL;
	}

	return 0;
}

static void *hdlcd_ram_vmap(const struct device *dev, phys_addr_t start,
			    size_t size, unsigned int memtype)
{
	struct page **pages;
	phys_addr_t page_start;
	unsigned int page_count;
	pgprot_t prot;
	unsigned int i;
	void *vaddr;

	page_start = start - offset_in_page(start);
	page_count = DIV_ROUND_UP(size + offset_in_page(start), PAGE_SIZE);

	if (memtype)
		prot = pgprot_noncached(PAGE_KERNEL);
	else
		prot = pgprot_writecombine(PAGE_KERNEL);

	pages = kmalloc_array(page_count, sizeof(struct page *), GFP_KERNEL);
	if (!pages) {
		dev_err(dev, "Failed to allocate array for %u pages\n",
			page_count);
		return NULL;
	}

	for (i = 0; i < page_count; i++) {
		phys_addr_t addr = page_start + i * PAGE_SIZE;
		pages[i] = pfn_to_page(addr >> PAGE_SHIFT);
	}
	vaddr = vmap(pages, page_count, VM_MAP, prot);
	kfree(pages);

	return vaddr;
}

static int hdlcd_probe(struct platform_device *pdev)
{
	int i = 0;
	int err = 0;
	int ret = 0;
	struct hdlcd_device *hdlcd;
	struct resource *mem;
	struct device_node *of_node;
	struct resource res = { 0 };
	struct device_node *memnp = NULL;

	memset(&cached_var_screeninfo, 0, sizeof(struct fb_var_screeninfo));

	dev_err(&pdev->dev, "HDLCD: probing\n");

	hdlcd = kzalloc(sizeof(*hdlcd), GFP_KERNEL);
	if (!hdlcd)
		return -ENOMEM;

	of_node = pdev->dev.of_node;
	if (of_node) {
		int len;
		const u8 *edid;

		const struct of_device_id *of_id;
		of_id = of_match_node(hdlcd_of_matches, of_node);
		pro_name = of_id->compatible;
		dev_err(&pdev->dev, "HDLCD: pro_name=%s\n", pro_name);
		memnp = of_parse_phandle(of_node, "memory-region", 0);
		if (!memnp)
			return -ENODEV;

		ret = of_address_to_resource(memnp, 0, &res);
		if (0 != ret) {
			pr_notice("of_address_to_resource failed!\n");
			return -EINVAL;
		}
		of_node_put(memnp);

		hdlcd->fb.fix.smem_start = res.start;
		framebuffer_size = resource_size(&res);
		if (framebuffer_size > HDLCD_MAX_FRAMEBUFFER_SIZE)
			framebuffer_size = HDLCD_MAX_FRAMEBUFFER_SIZE;
		dev_err(&pdev->dev,
			"HDLCD: phys_addr = 0x%lx, size = 0x%lx\n",
			hdlcd->fb.fix.smem_start, framebuffer_size);

		edid = of_get_property(of_node, "edid", &len);
		if (edid) {
			err = parse_edid_data(hdlcd, edid, len);
#ifdef CONFIG_SERIAL_AMBA_PCU_UART
		} else {
			/* ask the firmware to fetch the EDID */
			dev_dbg(&pdev->dev, "HDLCD: Requesting EDID data\n");
			hdlcd->edid = kzalloc(EDID_LENGTH, GFP_KERNEL);
			if (!hdlcd->edid)
				return -ENOMEM;
			err = get_edid(hdlcd->edid);
#endif /* CONFIG_SERIAL_AMBA_PCU_UART */
		}
		if (err)
			dev_info(&pdev->dev,
				 "HDLCD: Failed to parse EDID data\n");
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "HDLCD: cannot get platform resources\n");
		err = -EINVAL;
		goto resource_err;
	}

	i = platform_get_irq(pdev, 0);
	if (i < 0) {
		dev_err(&pdev->dev, "HDLCD: no irq defined for vsync\n");
		err = -ENOENT;
		goto resource_err;
	} else {
		err = request_irq(i, hdlcd_irq, 0, dev_name(&pdev->dev), hdlcd);
		if (err) {
			dev_err(&pdev->dev, "HDLCD: unable to request irq\n");
			goto resource_err;
		}
		hdlcd->irq = i;
	}

	if (!request_mem_region
	    (mem->start, resource_size(mem), dev_name(&pdev->dev))) {
		err = -ENXIO;
		goto request_err;
	}

	if (!hdlcd->fb.fix.smem_start) {
		dev_err(&pdev->dev,
			"platform did not allocate frame buffer memory\n");
		err = -ENOMEM;
		goto memalloc_err;
	}
	hdlcd->fb.screen_base =
	    hdlcd_ram_vmap(&pdev->dev, hdlcd->fb.fix.smem_start,
			   framebuffer_size, 1);
	if (!hdlcd->fb.screen_base) {
		dev_err(&pdev->dev, "unable to remap framebuffer\n");
		err = -ENOMEM;
		goto probe_err;
	}

	hdlcd->fb.screen_size = framebuffer_size;
	hdlcd->fb.fix.smem_len = framebuffer_size;
	hdlcd->fb.fix.mmio_start = mem->start;
	hdlcd->fb.fix.mmio_len = resource_size(mem);

	hdlcd->dev = &pdev->dev;

	dev_dbg(&pdev->dev,
		"HDLCD: framebuffer virt base %p, phys base 0x%lX\n",
		hdlcd->fb.screen_base, (unsigned long)hdlcd->fb.fix.smem_start);

	err = hdlcd_setup(hdlcd);

	if (err)
		goto probe_err;

	platform_set_drvdata(pdev, hdlcd);
	hdlcd->is_set_par = 0;
	hdlcd->probe_end = 1;
	dev_err(&pdev->dev, "hdlcd: hdlcd_probe end\n");
	return 0;

probe_err:
	iounmap(hdlcd->fb.screen_base);
	memblock_free(hdlcd->fb.fix.smem_start, hdlcd->fb.fix.smem_start);

memalloc_err:
	release_mem_region(mem->start, resource_size(mem));

request_err:
	free_irq(hdlcd->irq, hdlcd);

resource_err:
	kfree(hdlcd);

	return err;
}

static int hdlcd_remove(struct platform_device *pdev)
{
	struct hdlcd_device *hdlcd = platform_get_drvdata(pdev);

	clk_disable(hdlcd->clk);
	clk_unprepare(hdlcd->clk);
	clk_put(hdlcd->clk);

	/* unmap memory */
	iounmap(hdlcd->fb.screen_base);
	iounmap(hdlcd->base);

	/* deallocate fb memory */
	fb_dealloc_cmap(&hdlcd->fb.cmap);
	kfree(hdlcd->fb.pseudo_palette);
	memblock_free(hdlcd->fb.fix.smem_start, hdlcd->fb.fix.smem_start);
	release_mem_region(hdlcd->fb.fix.mmio_start, hdlcd->fb.fix.mmio_len);

	free_irq(hdlcd->irq, NULL);
	kfree(hdlcd);

	return 0;
}

#ifdef CONFIG_PM
static int hdlcd_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* not implemented yet */
	return 0;
}

static int hdlcd_resume(struct platform_device *pdev)
{
	/* not implemented yet */
	return 0;
}
#else
#define hdlcd_suspend	NULL
#define hdlcd_resume	NULL
#endif

static struct platform_driver hdlcd_driver = {
	.probe = hdlcd_probe,
	.remove = hdlcd_remove,
	.suspend = hdlcd_suspend,
	.resume = hdlcd_resume,
	.driver = {
		   .name = "hdlcd",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(hdlcd_of_matches),
		   },
};

static int __init hdlcd_init(void)
{
#ifdef HDLCD_COUNT_BUFFERUNDERRUNS
	int err = platform_driver_register(&hdlcd_driver);

	if (!err)
		hdlcd_underrun_init();
	return err;
#else
	return platform_driver_register(&hdlcd_driver);
#endif
}

void __exit hdlcd_exit(void)
{
#ifdef HDLCD_COUNT_BUFFERUNDERRUNS
	hdlcd_underrun_close();
#endif
	platform_driver_unregister(&hdlcd_driver);
}

module_init(hdlcd_init);
module_exit(hdlcd_exit);

MODULE_AUTHOR("Liviu Dudau");
MODULE_DESCRIPTION("SPRD DISPC HDLCD core driver");
MODULE_LICENSE("GPL v2");

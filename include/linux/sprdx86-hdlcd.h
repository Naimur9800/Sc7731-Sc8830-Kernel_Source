/*
 * include/linux/arm-hdlcd.h
 *
 * Copyright (C) 2011 ARM Limited
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 *
 *  ARM HDLCD Controller register definition
 */

#include <linux/fb.h>
#include <linux/completion.h>

/* register offsets */
/* DISPC regs offset */
#define DISPC_VERSION			(0x0000)
#define DISPC_CTRL			(0x0004)
#define DISPC_SIZE_XY			(0x0008)
#define DISPC_RSTN			(0x000C)
#define DISPC_STS			(0x0014)
#define DISPC_3D_CTRL			(0x0018)
#define DISPC_IMG_Y_BASE_ADDR		(0x0020)
#define DISPC_IMG_UV_BASE_ADDR		(0x0024)
#define DISPC_IMG_V_BASE_ADDR		(0x0028)
#define DISPC_IMG_CTRL			(0x0030)
#define DISPC_IMG_SIZE_XY		(0x0034)
#define DISPC_IMG_PITCH			(0x0038)
#define DISPC_IMG_DISP_XY		(0x003C)
#define DISPC_BG_COLOR			(0x0040)
#define DISPC_OSD_BASE_ADDR		(0x0050)
#define DISPC_OSD_CTRL			(0x0060)
#define DISPC_OSD_SIZE_XY		(0x0064)
#define DISPC_OSD_PITCH			(0x0068)
#define DISPC_OSD_DISP_XY		(0x006C)
#define DISPC_OSD_ALPHA			(0x0070)
#define DISPC_OSD_CK			(0x0074)
#define DISPC_Y2R_CTRL			(0x0080)
#define DISPC_Y2R_Y_PARAM		(0x0084)
#define DISPC_Y2R_U_PARAM		(0x0088)
#define DISPC_Y2R_V_PARAM		(0x008c)
#define DISPC_Y2R_COEF_00		(0x0090)
#define DISPC_Y2R_COEF_01		(0x0094)
#define DISPC_Y2R_COEF_10		(0x0098)
#define DISPC_Y2R_COEF_11		(0x009C)
#define DISPC_Y2R_COEF_20		(0x00A0)
#define DISPC_Y2R_COEF_21		(0x00A4)
#define DISPC_INT_EN			(0x00B0)
#define DISPC_INT_CLR			(0x00B4)
#define DISPC_INT_STATUS		(0x00B8)
#define DISPC_INT_RAW			(0x00BC)
#define DISPC_DPI_CTRL			(0x00C0)
#define DISPC_DPI_H_TIMING		(0x00C4)
#define DISPC_DPI_V_TIMING		(0x00C8)
#define DISPC_DPI_STS0			(0x00CC)
#define DISPC_DPI_STS1			(0x00D0)
#define DISPC_DBI_CTRL			(0x00E0)
#define DISPC_DBI_TIMING0		(0x00E4)
#define DISPC_DBI_TIMING1		(0x00E8)
#define DISPC_DBI_RDATA			(0x00EC)
#define DISPC_DBI_CMD			(0x00F0)
#define DISPC_DBI_DATA			(0x00F4)
#define DISPC_DBI_QUEUE			(0x00F8)
#define DISPC_TE_SYNC_DELAY		(0x00FC)

/* shadow register , read only */
#define SHDW_BUF_THRES			(0x0210)
#define SHDW_3D_CTRL			(0x0218)
#define SHDW_IMG_Y_BASE_ADDR		(0x0220)
#define SHDW_IMG_UV_BASE_ADDR		(0x0224)
#define SHDW_IMG_V_BASE_ADDR		(0x0228)
#define SHDW_IMG_CTRL			(0x0230)
#define SHDW_IMG_SIZE_XY		(0x0234)
#define SHDW_IMG_PITCH			(0x0238)
#define SHDW_IMG_DISP_XY		(0x023C)
#define SHDW_BG_COLOR			(0x0240)
#define SHDW_OSD_BASE_ADDR0		(0x0250)
#define SHDW_OSD_BASE_ADDR1		(0x0254)
#define SHDW_OSD_CTRL			(0x0260)
#define SHDW_OSD_SIZE_XY		(0x0264)
#define SHDW_OSD_PITCH			(0x0268)
#define SHDW_OSD_DISP_XY		(0x026C)
#define SHDW_OSD_ALPHA			(0x0270)
#define SHDW_OSD_CK			(0x0274)
#define SHDW_Y2R_CTRL			(0x0280)
#define SHDW_Y2R_CONTRAST		(0x0284)
#define SHDW_Y2R_SATURATION		(0x0288)
#define SHDW_Y2R_BRIGHTNESS		(0x028C)
#define SHDW_Y2R_COEF_00		(0x0290)
#define SHDW_Y2R_COEF_01		(0x0294)
#define SHDW_Y2R_COEF_10		(0x0298)
#define SHDW_Y2R_COEF_11		(0x029C)
#define SHDW_Y2R_COEF_20		(0x02A0)
#define SHDW_Y2R_COEF_21		(0x02A4)
#define SHDW_DPI_H_TIMING		(0x02C4)
#define SHDW_DPI_V_TIMING		(0x02C8)

/* interrupts */
/* interrupts */
#define DISPC_INT_DONE			(1<<0)
#define DISPC_INT_TE			(1<<1)
#define DISPC_INT_ERR			(1<<2)
#define DISPC_INT_EDPI_TE		(1<<3)
#define DISPC_INT_UPDATE_DONE		(1<<4)
#define DISPC_INT_DPI_VSYNC		(1<<5)

/* Max resolution supported is 4096x4096, 8 bit per color component,
   8 bit alpha, but we are going to choose the usual hardware default
   (2048x2048, 32 bpp) and enable double buffering */
#define HDLCD_MAX_XRES			2048
#define HDLCD_MAX_YRES			2048
#define HDLCD_MAX_FRAMEBUFFER_SIZE	(HDLCD_MAX_XRES * HDLCD_MAX_YRES << 2)

#define HDLCD_MEM_BASE			(CONFIG_PAGE_OFFSET - 0x1000000)

#define NR_PALETTE	256

/* OEMs using HDLCD may wish to enable these settings if
 * display disruption is apparent and you suspect HDLCD
 * access to RAM may be starved.
 */
/* Turn HDLCD default color red instead of black so
 * that it's easy to see pixel clock data underruns
 * (compared to other visual disruption)
 */
/* #define HDLCD_RED_DEFAULT_COLOUR */
/* Add a counter in the IRQ handler to count buffer underruns
 * and /proc/hdlcd_underrun to read the counter
 */
/* #define HDLCD_COUNT_BUFFERUNDERRUNS */
/* Restrict height to 1x screen size
 *
 */
/* #define HDLCD_NO_VIRTUAL_SCREEN */

#ifdef CONFIG_ANDROID
/* #define HDLCD_NO_VIRTUAL_SCREEN */
#endif

struct hdlcd_device {
	struct fb_info		fb;
	struct device		*dev;
	struct clk		*clk;
	void __iomem		*base;
	int			irq;
	struct completion	vsync_completion;
	unsigned char		*edid;
	unsigned char		probe_end;
	unsigned char           is_set_par;
};

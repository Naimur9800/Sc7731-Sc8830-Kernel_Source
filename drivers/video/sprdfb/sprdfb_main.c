/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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

#define pr_fmt(fmt)		"sprdfb: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <asm/uaccess.h>
#include <linux/compat.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_address.h>
#endif

#include "sprdfb.h"
#include "sprdfb_panel.h"
#include <soc/sprd/board.h>
#ifdef CONFIG_FB_MMAP_CACHED
#include <asm/pgtable.h>
#include <linux/mm.h>
#endif

#include "sprdfb_chip_common.h"
#include <ontim/ontim_dev_dgb.h>

enum{
	SPRD_IN_DATA_TYPE_ABGR888 = 0,
	SPRD_IN_DATA_TYPE_BGR565,
/*
	SPRD_IN_DATA_TYPE_RGB666,
	SPRD_IN_DATA_TYPE_RGB555,
	SPRD_IN_DATA_TYPE_PACKET,
*/ /*not support*/
	SPRD_IN_DATA_TYPE_LIMIT
};

#define SPRDFB_IN_DATA_TYPE SPRD_IN_DATA_TYPE_ABGR888

#ifdef CONFIG_FB_TRIPLE_FRAMEBUFFER
#define FRAMEBUFFER_NR		(3)
#else
#define FRAMEBUFFER_NR		(2)
#endif

#define SPRDFB_FRAMES_TO_SKIP 	(1)

#define SPRDFB_DEFAULT_FPS (60)

#define SPRDFB_ESD_TIME_OUT_CMD	(2000)

#define SPRDFB_ESD_TIME_OUT_VIDEO	(1000)

extern bool sprdfb_panel_get(struct sprdfb_device *dev);
extern int sprdfb_panel_probe(struct sprdfb_device *dev);
extern void sprdfb_panel_remove(struct sprdfb_device *dev);

extern struct display_ctrl sprdfb_dispc_ctrl ;

#ifdef CONFIG_OF
extern unsigned long g_dispc_base_addr;
#endif

static unsigned PP[16];
extern char lcd_name_from_uboot[50] ;

static int frame_count = 0;
static char sprdfb_version[]="sprdfb";
static char sprdfb_vendor_name[50]="sprdfb";
u_char sprdfb_esd_enable=0;
static void lcd_config_debug(unsigned char* srcbuf); //wg debug lcd

DEV_ATTR_DECLARE(sprdfb)
    DEV_ATTR_DEFINE("version",sprdfb_version)
    DEV_ATTR_DEFINE("vendor",sprdfb_vendor_name)
    DEV_ATTR_DEFINE("lcd_name",lcd_name_from_uboot)
    DEV_ATTR_EXEC_DEFINE_PARAM("lcd_config",lcd_config_debug)
    DEV_ATTR_VAL_DEFINE("esd_enable",&sprdfb_esd_enable,ONTIM_DEV_ARTTR_TYPE_VAL_8BIT)
    DEV_ATTR_DECLARE_END;
ONTIM_DEBUG_DECLARE_AND_INIT(sprdfb,sprdfb,8);

static int sprdfb_check_var(struct fb_var_screeninfo *var, struct fb_info *fb);
static int sprdfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *fb);
static int sprdfb_ioctl(struct fb_info *info, unsigned int cmd,
			unsigned long arg);
#ifdef CONFIG_COMPAT
static long sprdfb_compat_ioctl(struct fb_info *info, unsigned int cmd,
			    unsigned long arg);
#endif
#ifdef CONFIG_FB_MMAP_CACHED
static int sprdfb_mmap(struct fb_info *info, struct vm_area_struct *vma);
#endif

static struct fb_ops sprdfb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = sprdfb_check_var,
	.fb_pan_display = sprdfb_pan_display,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_ioctl = sprdfb_ioctl,
#ifdef CONFIG_COMPAT
	.fb_compat_ioctl = sprdfb_compat_ioctl,
#endif
#ifdef CONFIG_FB_MMAP_CACHED
	.fb_mmap = sprdfb_mmap,
#endif
};
//lcd config debug  add wg
static u8 convert_ascii2num(u8 v)
{
    if (v >= '0'&& v <='9')
        return v - '0';
    else if (v >= 'a' && v <='f')
        return v - 'a' + 0xa;
    else if (v >= 'A' && v <='F')
        return v - 'A' + 0xa;
    else
        return 0xff;
}
                                  
static unsigned int lcd_dbgcfg_tflash( sprdfb_dev_dsi_cmds* cfile, unsigned char* buf, unsigned char* src)
{
#define STATE_LINE_BEGIN  0
#define STATE_LINE_IGNORE 1
#define STATE_LEFT        2
#define STATE_ARRAY_0_0   3
#define STATE_ARRAY_0_X   4
#define STATE_ARRAY_NEXT_DATA 5
#define STATE_ARRAY_0_DATA0   6
#define STATE_ARRAY_0_9   7

    sprdfb_dev_dsi_cmds* file = cfile ;
    unsigned char* preg = buf ;
    int state = STATE_LINE_BEGIN;
    unsigned int i;
    char v = 0;
    unsigned char* pv = src ;
    unsigned char lsz = 0;
    unsigned int vsz = 0 ;
    unsigned char delay = 0 ;
    
    for (i = 0 ; *pv != '\0'; i++)
    {
        v = *pv++ ;
        if (' ' == v || '\t' == v)
            continue;
        if ('\n' == v )
        {
        	  //printk("-vsz %d lsz %d ",vsz,lsz);
            if(lsz)
            {
                int k = 0;
                
                file->type = *preg++ ;
                //printk("type 0x%x ",file->type);
                if( file->type == DSI_CMD_DEALY )
                {
                	file->len = preg++ ;
                	//printk("len %d \n",file->len);
                }
                else if(file->type == DSI_CMD_END)
                {
                	//printk("\n");
                }
                else
                {
                  file->len = lsz-1 ;
                  file->p_cmd = preg ;
                  preg += file->len ;
                  //printk(" len %d p_cmd 0x%x \n",file->len,file->p_cmd[0]);
                }
                
                lsz = 0 ;
                vsz ++ ;
                file ++ ;
            }
            state = STATE_LINE_BEGIN;
            continue;
        }

			if (STATE_LINE_BEGIN == state)
			{
				if ('{' == v)
		            state = STATE_LEFT;
				if ('/' == v)
				    state = STATE_LINE_IGNORE;
			}
			else if (STATE_LINE_IGNORE == state)
			{
				if ('\n' == v)
				{
				    state = STATE_LINE_BEGIN;
				}
			}
			else if (STATE_LEFT == state)
			{
				if (',' == v)
				    state = STATE_LEFT ;
				else if ('0' == v)
				    state = STATE_ARRAY_0_0;
				else if (v >= '1' && v <= '9')
				{
				    //delay = ((u32)(v-'0'));
				    preg[lsz] = ((u32)(v-'0'));
				    state = STATE_ARRAY_0_9;
				}
				else
		        state = STATE_LINE_IGNORE;
			}
			else if (STATE_ARRAY_0_0 == state)
			{
				if ('x' == v || 'X' == v)
				    state = STATE_ARRAY_0_X;
				else if (v >= '0' && v <= '9')
				{
				    preg[lsz] = ((u32)(v-'0'));
				    state = STATE_ARRAY_0_9;
				}
				else
				    state = STATE_LINE_IGNORE;
			}
			else if (STATE_ARRAY_0_X == state)
			{
				if (',' == v || '}' == v )
				    state = STATE_LEFT ;
				else
				{
				    v = convert_ascii2num(v);
				    if (v >= 0 && v <= 0xf)
				    {
		             preg[lsz] = ((u8)v);
		             state = STATE_ARRAY_0_DATA0;
				    }
				    else
		             state = STATE_LINE_IGNORE;
				}
			}
			else if (STATE_ARRAY_0_DATA0 == state)
			{
				if(',' == v || '}' == v )
				{
				    lsz++ ;
				    state = STATE_LEFT;
				}
				else
				{
				    v = convert_ascii2num(v);
				    if (v >= 0 && v <= 0xf)
				    {
					      preg[lsz] = preg[lsz] <<4;
		            preg[lsz++] |= ( ((u8)v) & 0xF );
		            state = STATE_LEFT;
				    }
				    else
		            state = STATE_LINE_IGNORE;
				}
			}
			else if (STATE_ARRAY_0_9 == state)
			{
				if(',' == v || '}' == v)
				{
				    lsz++ ;
				    state = STATE_LEFT;
				}
				else if (v >= '0' && v <= '9')
				{
				    preg[lsz] = preg[lsz]*10+((u32)(v-'0'));
				    state = STATE_ARRAY_0_9;
				}
				else
						state = STATE_LINE_IGNORE;
			}
    }
    return vsz;
}

static unsigned int lcd_dbgcmds_size = 0 ;
static unsigned char* lcd_dbgcmds_data = NULL ;
static sprdfb_dev_dsi_cmds* lcd_dbgcmds_desc = NULL ; 

unsigned int lcd_debug_panel_on_cmds(struct sprdfb_dev_dsi_cmds** cmds, unsigned char** buf )
{
    if(lcd_dbgcmds_data == NULL || lcd_dbgcmds_desc == NULL)
    return 0 ;

    *buf  = lcd_dbgcmds_data  ;
    *cmds = lcd_dbgcmds_desc ;
    lcd_dbgcmds_data  = NULL ;
    lcd_dbgcmds_desc = NULL ;

    return lcd_dbgcmds_size ;
}

static void lcd_config_debug(unsigned char* srcbuf)
{
    if(lcd_dbgcmds_desc == NULL)
        lcd_dbgcmds_data  = (unsigned char*) kzalloc(4*1024,GFP_KERNEL);
    
    if(lcd_dbgcmds_desc == NULL)
        lcd_dbgcmds_desc = ( sprdfb_dev_dsi_cmds*)(
                kzalloc(512*sizeof( sprdfb_dev_dsi_cmds),GFP_KERNEL) ) ;
    
    if(lcd_dbgcmds_data != NULL && lcd_dbgcmds_desc != NULL)
        lcd_dbgcmds_size = lcd_dbgcfg_tflash(lcd_dbgcmds_desc,lcd_dbgcmds_data, srcbuf);
}

//lcd debug add end by wg

#ifdef CONFIG_FB_MMAP_CACHED
static int sprdfb_mmap(struct fb_info *info,struct vm_area_struct *vma)
{
	struct sprdfb_device *dev = NULL;
	if(NULL == info){
			printk(KERN_ERR "sprdfb: sprdfb_ioctl error. (Invalid Parameter)");
			return -1;
	}

	dev = info->par;
	printk("sprdfb: sprdfb_mmap,vma=0x%x\n",vma);
	vma->vm_page_prot = pgprot_cached(vma->vm_page_prot);
	dev->ctrl->set_vma(vma);

	return vm_iomap_memory(vma, info->fix.smem_start, info->fix.smem_len);
}
#endif

static int setup_fb_mem(struct sprdfb_device *dev, struct platform_device *pdev)
{
	uint32_t len;
	void *addr;
	bool use_reserve_mem;
	uint32_t reserve_mem[2];
	int ret;

#ifdef CONFIG_FB_LOW_RES_SIMU
	if ((0!= dev->display_width) && (0 != dev->display_height))
		len = dev->display_width * dev->display_height * (dev->bpp / 8) * FRAMEBUFFER_NR;
	else
#endif
	len = dev->panel->width * dev->panel->height * (dev->bpp / 8) * FRAMEBUFFER_NR;

#ifdef CONFIG_OF
	use_reserve_mem = of_property_read_bool(pdev->dev.of_node, "sprd,fb_use_reservemem");
	if(use_reserve_mem) {
		ret = of_property_read_u32_array(pdev->dev.of_node, "sprd,fb_mem",
			reserve_mem, 2);
		if(0 != ret)
			printk(KERN_ERR "sprdfb: Failed to got framebuffer memory from dt file\n");
	}
#else
#ifdef	CONFIG_FB_LCD_RESERVE_MEM
	use_reserve_mem = true;
	reserve_mem[0] = SPRD_FB_MEM_BASE;
	reserve_mem[1] = SPRD_FB_MEM_SIZE;
#else
	use_reserve_mem = false;
#endif
#endif

	if(!use_reserve_mem) {
		addr = (void*)__get_free_pages(GFP_ATOMIC | __GFP_ZERO, get_order(len));
		if (NULL == addr) {
			printk(KERN_ERR "sprdfb: Failed to allocate framebuffer memory\n");
			return -ENOMEM;
		}
		printk(KERN_INFO "sprdfb: got %d bytes mem at 0x%p\n", len, addr);

		dev->fb->fix.smem_start = __pa(addr);
		dev->fb->fix.smem_len = len;
		dev->fb->screen_base = (char*)addr;
	}else{
		dev->fb->fix.smem_start = reserve_mem[0];
		printk("sprdfb: setup_fb_mem--smem_start:%lx,len:%d,reserved len:%d\n",
			dev->fb->fix.smem_start, len, reserve_mem[1]);
		addr = ioremap(dev->fb->fix.smem_start, len);
		if (NULL == addr) {
			printk(KERN_ERR "sprdfb: Unable to map framebuffer base: 0x%p\n", addr);
			return -ENOMEM;
		}
		dev->fb->fix.smem_len = len;
		dev->fb->screen_base = (char*)addr;
	}
	return 0;
}

static void setup_fb_info(struct sprdfb_device *dev)
{
	struct fb_info *fb = dev->fb;
	struct panel_spec *panel = dev->panel;
	int r;

	fb->fbops = &sprdfb_ops;
	fb->flags = FBINFO_DEFAULT;

	/* finish setting up the fb_info struct */
	strncpy(fb->fix.id, "sprdfb", 16);
	fb->fix.ypanstep = 1;
	fb->fix.type = FB_TYPE_PACKED_PIXELS;
	fb->fix.visual = FB_VISUAL_TRUECOLOR;
#ifdef CONFIG_FB_LOW_RES_SIMU
	if((0 != dev->display_width) && (0 != dev->display_height)){
		fb->fix.line_length = dev->display_width * dev->bpp / 8;

		fb->var.xres = dev->display_width;
		fb->var.yres = dev->display_height;
		fb->var.width = dev->display_width;
		fb->var.height = dev->display_height;

		fb->var.xres_virtual = dev->display_width;
		fb->var.yres_virtual = dev->display_height * FRAMEBUFFER_NR;
	}else
#endif
	{
		fb->fix.line_length = panel->width * dev->bpp / 8;

		fb->var.xres = panel->width;
		fb->var.yres = panel->height;
		fb->var.width = panel->width;
		fb->var.height = panel->height;

		fb->var.xres_virtual = panel->width;
		fb->var.yres_virtual = panel->height * FRAMEBUFFER_NR;
	}
	fb->var.bits_per_pixel = dev->bpp;
#ifdef CONFIG_FB_LOW_RES_SIMU
	if((0 != dev->display_width) && (0 != dev->display_height)){
		if(0 != dev->panel->fps){
			fb->var.pixclock = ((1000000000 /dev->display_width) * 1000) / (dev->panel->fps * dev->display_height);
		}else{
			fb->var.pixclock = ((1000000000 /dev->display_width) * 1000) / (SPRDFB_DEFAULT_FPS * dev->display_height);
		}
	}else
#endif
	{
		if(0 != dev->panel->fps){
			fb->var.pixclock = ((1000000000 /panel->width) * 1000) / (dev->panel->fps * panel->height);
		}else{
			fb->var.pixclock = ((1000000000 /panel->width) * 1000) / (SPRDFB_DEFAULT_FPS * panel->height);
		}
	}

	fb->var.accel_flags = 0;
	fb->var.yoffset = 0;

	/* only support two pixel format */
	if (dev->bpp == 32) { /* ABGR */
		fb->var.red.offset     = 16;
		fb->var.red.length     = 8;
		fb->var.red.msb_right  = 0;
		fb->var.green.offset   = 8;
		fb->var.green.length   = 8;
		fb->var.green.msb_right = 0;
		fb->var.blue.offset    = 0;
		fb->var.blue.length    = 8;
		fb->var.blue.msb_right = 0;
	} else { /*BGR*/
		fb->var.red.offset     = 11;
		fb->var.red.length     = 5;
		fb->var.red.msb_right  = 0;
		fb->var.green.offset   = 5;
		fb->var.green.length   = 6;
		fb->var.green.msb_right = 0;
		fb->var.blue.offset    = 0;
		fb->var.blue.length    = 5;
		fb->var.blue.msb_right = 0;
	}
	r = fb_alloc_cmap(&fb->cmap, 16, 0);
	fb->pseudo_palette = PP;

	PP[0] = 0;
	for (r = 1; r < 16; r++){
		PP[r] = 0xffffffff;
	}
}

static void fb_free_resources(struct sprdfb_device *dev)
{
	if (dev == NULL)
		return;

	if (&dev->fb->cmap != NULL) {
		fb_dealloc_cmap(&dev->fb->cmap);
	}
	if (dev->fb->screen_base) {
		free_pages((unsigned long)dev->fb->screen_base,
				get_order(dev->fb->fix.smem_len));
	}
	unregister_framebuffer(dev->fb);
	framebuffer_release(dev->fb);
}

static int sprdfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *fb)
{
	int32_t ret;
	struct sprdfb_device *dev = fb->par;

	pr_debug("sprdfb: [%s]\n", __FUNCTION__);
	
	if(frame_count < SPRDFB_FRAMES_TO_SKIP) {
		frame_count++;
		return 0;
    }

	if(0 == dev->enable){
		printk(KERN_ERR "sprdfb: [%s]: Invalid Device status %d", __FUNCTION__, dev->enable);
		return -1;
	}

	ret = dev->ctrl->refresh(dev);
	if (ret) {
		printk(KERN_ERR "sprdfb: failed to refresh !!!!\n");
		return -1;
	}

	return 0;
}

#include <video/sprd_fb.h>
static int sprdfb_ioctl(struct fb_info *info, unsigned int cmd,
			unsigned long arg)
{
	int result = -1;
	struct sprdfb_device *dev = NULL;
#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
	overlay_info local_overlay_info;
	overlay_display local_overlay_display;
#endif
	int power_mode;
	void __user *argp = (void __user *)arg;

	if(NULL == info){
		printk(KERN_ERR "sprdfb: sprdfb_ioctl error. (Invalid Parameter)");
		return -1;
	}

	dev = info->par;

	switch(cmd){
#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
	case SPRD_FB_SET_OVERLAY:
		pr_debug(KERN_INFO "sprdfb: [%s]: SPRD_FB_SET_OVERLAY\n", __FUNCTION__);
		memset(&local_overlay_info,0,sizeof(local_overlay_info));
		if (copy_from_user(&local_overlay_info, argp, sizeof(local_overlay_info))){
			printk("sprdfb: SET_OVERLAY copy failed!\n");
			return -EFAULT;
		}
		if(NULL != dev->ctrl->enable_overlay){
			result = dev->ctrl->enable_overlay(dev, &local_overlay_info, 1);
		}
		break;
	case SPRD_FB_DISPLAY_OVERLAY:
		pr_debug(KERN_INFO "sprdfb: [%s]: SPRD_FB_DISPLAY_OVERLAY\n", __FUNCTION__);
		memset(&local_overlay_display,0,sizeof(local_overlay_display));
		if (copy_from_user(&local_overlay_display, argp, sizeof(local_overlay_display))){
			printk("sprdfb: DISPLAY_OVERLAY copy failed!\n");
			return -EFAULT;
		}
		if(NULL != dev->ctrl->display_overlay){
			result = dev->ctrl->display_overlay(dev, &local_overlay_display);
		}
		break;
#endif
#ifdef CONFIG_FB_VSYNC_SUPPORT
	case FBIO_WAITFORVSYNC:
		pr_debug(KERN_INFO "sprdfb: [%s]: FBIO_WAITFORVSYNC\n", __FUNCTION__);
		if(NULL != dev->ctrl->wait_for_vsync){
			result = dev->ctrl->wait_for_vsync(dev);
		}
		break;
#endif

#ifdef CONFIG_FB_DYNAMIC_FREQ_SCALING
	case SPRD_FB_CHANGE_FPS:
		{
			int fps;
			result = copy_from_user(&fps, argp, sizeof(fps));
			if (result) {
				pr_err("%s: copy_from_user failed", __func__);
				return result;
			}
			pr_info("%s: fps will be changed to %d via ioctl\n",
					__func__, fps);
			result = sprdfb_chg_clk_intf(dev,
					SPRDFB_DYNAMIC_FPS, fps);
			if (result) {
				pr_err("%s: fps is set fail. fps=%d, ret=%d\n",
						__func__, fps, result);
				return result;
			}
			break;
		}
#endif

	case SPRD_FB_IS_REFRESH_DONE:
		pr_debug(KERN_INFO "sprdfb: [%s]: SPRD_FB_IS_REFRESH_DONE\n", __FUNCTION__);
		if(NULL != dev->ctrl->is_refresh_done){
			result = dev->ctrl->is_refresh_done(dev);
		}
		break;

	case SPRD_FB_SET_POWER_MODE:
		result = copy_from_user(&power_mode, argp, sizeof(power_mode));
		printk("sprdfb: [%s] : SPRD_FB_SET_POWER_MODE (%d)\n", __FUNCTION__, power_mode);
		break;

	default:
		printk(KERN_INFO "sprdfb: [%s]: unknown cmd(%d)\n", __FUNCTION__, cmd);
		break;
	}

	pr_debug(KERN_INFO "sprdfb: [%s]: return %d\n",__FUNCTION__, result);
	return result;
}


#ifdef CONFIG_COMPAT
#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT

static int sprdfb_set_overlayinfo(struct fb_info *info, unsigned int cmd,
			  unsigned long arg)
{
	overlay_info __user *local_overlay_info;
	overlay_info32 __user *local_overlay_info32;
	__u32 buffer;
	int err;

	local_overlay_info = compat_alloc_user_space(sizeof(overlay_info));
	local_overlay_info32 = compat_ptr(arg);

	if (copy_in_user(&(local_overlay_info->layer_index), &(local_overlay_info32->layer_index),
		4*sizeof(int)))
		return -EFAULT;

	if (copy_in_user(&(local_overlay_info->rb_switch), &(local_overlay_info32->rb_switch),
		sizeof(bool)))
		return -EFAULT;

	if (copy_in_user(&(local_overlay_info->rect), &(local_overlay_info32->rect),
		 sizeof(overlay_rect)))
		return -EFAULT;

	if (get_user(buffer, &local_overlay_info32->buffer)||
	    put_user(compat_ptr(buffer), &local_overlay_info->buffer))
		return -EFAULT;

	err = sprdfb_ioctl(info, cmd, (unsigned long) local_overlay_info);

	return err;
}
#endif
static long sprdfb_compat_ioctl(struct fb_info *info, unsigned int cmd,
			    unsigned long arg)
{
	int result = -1;

	pr_debug(KERN_INFO "sprdfb: [%s]: ++\n", __FUNCTION__);

	if(NULL == info){
		printk(KERN_ERR "sprdfb: sprdfb_ioctl error. (Invalid Parameter)");
		return -1;
	}

	switch(cmd){
#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
	case SPRD_FB_SET_OVERLAY:
		pr_debug(KERN_INFO "sprdfb: [%s]: SPRD_FB_SET_OVERLAY\n", __FUNCTION__);
		result = sprdfb_set_overlayinfo(info, cmd, arg);
		break;
	case SPRD_FB_DISPLAY_OVERLAY:
		pr_debug(KERN_INFO "sprdfb: [%s]: SPRD_FB_DISPLAY_OVERLAY\n", __FUNCTION__);
		arg = (unsigned long) compat_ptr(arg);
		result = sprdfb_ioctl(info, cmd, arg);
		break;
#endif
#ifdef CONFIG_FB_VSYNC_SUPPORT
	case FBIO_WAITFORVSYNC:
		pr_debug(KERN_INFO "sprdfb: [%s]: FBIO_WAITFORVSYNC\n", __FUNCTION__);
		result = sprdfb_ioctl(info, cmd, arg);
		break;
#endif
#ifdef CONFIG_FB_DYNAMIC_FREQ_SCALING
	case SPRD_FB_CHANGE_FPS:
#endif
	case SPRD_FB_SET_POWER_MODE:
		pr_debug(KERN_INFO "sprdfb: [%s]: (%d)\n", __FUNCTION__,cmd);
		arg = (unsigned long) compat_ptr(arg);
		result = sprdfb_ioctl(info, cmd, arg);
		break;
	case SPRD_FB_IS_REFRESH_DONE:
	default:
		pr_debug(KERN_INFO "sprdfb: [%s]: (%d)\n", __FUNCTION__,cmd);
		result = sprdfb_ioctl(info, cmd, arg);
		break;
	}
	pr_debug(KERN_INFO "sprdfb: [%s]: return %d\n",__FUNCTION__, result);
	return result;
}
#endif

static int sprdfb_check_var(struct fb_var_screeninfo *var, struct fb_info *fb)
{
	if ((var->xres != fb->var.xres) ||
		(var->yres != fb->var.yres) ||
		(var->xres_virtual != fb->var.xres_virtual) ||
		(var->yres_virtual != fb->var.yres_virtual) ||
		(var->xoffset != fb->var.xoffset) ||
#ifndef BIT_PER_PIXEL_SURPPORT
		(var->bits_per_pixel != fb->var.bits_per_pixel) ||
#endif
		(var->grayscale != fb->var.grayscale))
			return -EINVAL;
	return 0;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void sprdfb_early_suspend (struct early_suspend* es)
{
	struct sprdfb_device *dev = container_of(es, struct sprdfb_device, early_suspend);
	struct fb_info *fb = dev->fb;
	printk("sprdfb: [%s]\n",__FUNCTION__);

	fb_set_suspend(fb, FBINFO_STATE_SUSPENDED);

	if (!lock_fb_info(fb)) {
		return ;
	}
	dev->ctrl->suspend(dev);
	unlock_fb_info(fb);
}

static void sprdfb_late_resume (struct early_suspend* es)
{
	struct sprdfb_device *dev = container_of(es, struct sprdfb_device, early_suspend);
	struct fb_info *fb = dev->fb;
	pr_debug("sprdfb: [%s]\n",__FUNCTION__);

	if (!lock_fb_info(fb)) {
		return ;
	}
	dev->ctrl->resume(dev);
	unlock_fb_info(fb);

	fb_set_suspend(fb, FBINFO_STATE_RUNNING);
}

#else

static int sprdfb_suspend(struct platform_device *pdev,pm_message_t state)
{
	struct sprdfb_device *dev = platform_get_drvdata(pdev);
	printk("sprdfb: [%s]\n",__FUNCTION__);

	dev->ctrl->suspend(dev);

	return 0;
}

static int sprdfb_resume(struct platform_device *pdev)
{
	struct sprdfb_device *dev = platform_get_drvdata(pdev);
	printk("sprdfb: [%s]\n",__FUNCTION__);

	dev->ctrl->resume(dev);

	return 0;
}
#endif

#ifdef CONFIG_FB_ESD_SUPPORT
static void ESD_work_func(struct work_struct *work)
{
	struct sprdfb_device *dev = container_of(work, struct sprdfb_device, ESD_work.work);

	pr_debug("sprdfb: [%s] enter!\n", __FUNCTION__);

	//do real ESD check
	//mdelay(1000);
	if(NULL != dev->ctrl->ESD_check){
		dev->ctrl->ESD_check(dev);
	}

	if(0 != dev->enable){
		pr_debug("sprdfb: reschedule ESD workqueue!\n");
		schedule_delayed_work(&dev->ESD_work, msecs_to_jiffies(dev->ESD_timeout_val));
		dev->ESD_work_start = true;
	}else{
		printk("sprdfb: DON't reschedule ESD workqueue since device not avialbe!!\n");
	}

	pr_debug("sprdfb: [%s] leave!\n", __FUNCTION__);
}
#endif

static int sprdfb_probe(struct platform_device *pdev)
{
	struct fb_info *fb = NULL;
	struct sprdfb_device *dev = NULL;
	int ret = 0;
#ifdef CONFIG_OF
	struct resource r;

#ifdef CONFIG_FB_LOW_RES_SIMU
	uint32_t display_size_array[2];
#endif

	pr_debug(KERN_INFO "sprdfb: [%s]\n", __FUNCTION__);
#else
	pr_debug(KERN_INFO "sprdfb: [%s], id = %d\n", __FUNCTION__, pdev->id);
#endif

	fb = framebuffer_alloc(sizeof(struct sprdfb_device), &pdev->dev);
	if (!fb) {
		printk(KERN_ERR "sprdfb: sprdfb_probe allocate buffer fail.\n");
		ret = -ENOMEM;
		goto err0;
	}

	dev = fb->par;
	dev->fb = fb;
	dev->lcd_name = NULL;//add by liuwei
#ifdef CONFIG_OF
	dev->of_dev = &(pdev->dev);

	dev->dev_id = of_alias_get_id(pdev->dev.of_node, "lcd");
	printk("sprdfb: [%s] id = %d\n", __FUNCTION__, dev->dev_id);
#else
	dev->dev_id = pdev->id;
#endif
	if((SPRDFB_MAINLCD_ID != dev->dev_id) &&(SPRDFB_SUBLCD_ID != dev->dev_id)){
		printk(KERN_ERR "sprdfb: sprdfb_probe fail. (unsupported device id)\n");
		goto err0;
	}

	switch(SPRDFB_IN_DATA_TYPE){
	case SPRD_IN_DATA_TYPE_ABGR888:
		dev->bpp = 32;
		break;
	case SPRD_IN_DATA_TYPE_BGR565:
		dev->bpp = 16;
		break;
	default:
		dev->bpp = 32;
		break;
	}

	if(SPRDFB_MAINLCD_ID == dev->dev_id){
		dev->ctrl = &sprdfb_dispc_ctrl;
#ifdef CONFIG_OF
		if(0 != of_address_to_resource(pdev->dev.of_node, 0, &r)){
			printk(KERN_ERR "sprdfb: sprdfb_probe fail. (can't get register base address)\n");
			goto err0;
		}
		g_dispc_base_addr = (unsigned long)ioremap_nocache(r.start,
				resource_size(&r));
		if(!g_dispc_base_addr)
			BUG();
		printk("sprdfb: set g_dispc_base_addr = %ld\n", g_dispc_base_addr);
#endif
	}

	dev->frame_count = 0;
	dev->logo_buffer_addr_v = 0;

	if(sprdfb_panel_get(dev)){
		dev->panel_ready = true;

#ifdef CONFIG_OF
#ifdef CONFIG_FB_LOW_RES_SIMU
		ret = of_property_read_u32_array(pdev->dev.of_node, "sprd,fb_display_size", display_size_array, 2);
		if (0 != ret) {
			printk("sprdfb: read display_size from dts fail (%d)\n", ret);
			dev->display_width = dev->panel->width;
			dev->display_height = dev->panel->height;
		} else {
			dev->display_width = display_size_array[0];
			dev->display_height = display_size_array[1];
		}
#endif
#endif

		dev->ctrl->logo_proc(dev);
	}else{
		dev->panel_ready = false;
	}

	dev->ctrl->early_init(dev);
    /*modify by liuwei*/
    /*****
	if(!dev->panel_ready){
		if (!sprdfb_panel_probe(dev)) {
			ret = -EIO;
			goto cleanup;
		}
	}
    *****/

	ret = setup_fb_mem(dev, pdev);
	if (ret) {
		goto cleanup;
	}

	setup_fb_info(dev);
	/* register framebuffer device */
	ret = register_framebuffer(fb);
	if (ret) {
		printk(KERN_ERR "sprdfb: sprdfb_probe register framebuffer fail.\n");
		goto cleanup;
	}
	platform_set_drvdata(pdev, dev);
	sprdfb_create_sysfs(dev);
	dev->ctrl->init(dev);

#ifdef CONFIG_HAS_EARLYSUSPEND
	dev->early_suspend.suspend = sprdfb_early_suspend;
	dev->early_suspend.resume  = sprdfb_late_resume;
	dev->early_suspend.level   = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&dev->early_suspend);
#endif

#ifdef CONFIG_FB_ESD_SUPPORT
	pr_debug("sprdfb: Init ESD work queue!\n");
	INIT_DELAYED_WORK(&dev->ESD_work, ESD_work_func);
//	sema_init(&dev->ESD_lock, 1);

	if(SPRDFB_PANEL_IF_DPI == dev->panel_if_type){
		dev->ESD_timeout_val = SPRDFB_ESD_TIME_OUT_VIDEO;
	}else{
		dev->ESD_timeout_val = SPRDFB_ESD_TIME_OUT_CMD;
	}

	dev->ESD_work_start = false;
	dev->check_esd_time = 0;
	dev->reset_dsi_time = 0;
	dev->panel_reset_time = 0;
#endif
	if(dev->lcd_name != NULL)//add by liuwei
	{
		int len = strlen(dev->lcd_name);
		memcpy(sprdfb_vendor_name,dev->lcd_name,len);
		sprdfb_vendor_name[len]=0;
	}

	REGISTER_AND_INIT_ONTIM_DEBUG_FOR_THIS_DEV();
	return 0;

cleanup:
	sprdfb_panel_remove(dev);
	dev->ctrl->uninit(dev);
	fb_free_resources(dev);
err0:
	dev_err(&pdev->dev, "failed to probe sprdfb\n");
	return ret;
}

static int sprdfb_remove(struct platform_device *pdev)
{
	struct sprdfb_device *dev = platform_get_drvdata(pdev);

	printk("sprdfb: [%s]\n",__FUNCTION__);
	sprdfb_remove_sysfs(dev);
	sprdfb_panel_remove(dev);
	dev->ctrl->uninit(dev);
	fb_free_resources(dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sprdfb_dt_ids[] = {
	{ .compatible = "sprd,sprdfb", },
	{}
};
#endif

static struct platform_driver sprdfb_driver = {
	.probe = sprdfb_probe,

#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = sprdfb_suspend,
	.resume = sprdfb_resume,
#endif
	.remove = sprdfb_remove,
	.driver = {
		.name = "sprd_fb",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(sprdfb_dt_ids),
#endif
	},
};


static int __init sprdfb_init(void)
{
	return platform_driver_register(&sprdfb_driver);
}

static void __exit sprdfb_exit(void)
{
	return platform_driver_unregister(&sprdfb_driver);
}

module_init(sprdfb_init);
module_exit(sprdfb_exit);

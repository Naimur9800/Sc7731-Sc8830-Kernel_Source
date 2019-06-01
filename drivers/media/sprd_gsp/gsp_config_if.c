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

/**---------------------------------------------------------------------------*
**                         Dependencies                                      *
**---------------------------------------------------------------------------*/
//#include <linux/irq.h>




#include "gsp_config_if.h"
uint32_t testregsegment[0x190]= {0};
extern struct clk	*g_gsp_emc_clk;
extern struct clk	*g_gsp_clk;
#ifdef CONFIG_OF
extern ulong gsp_base_addr;
#if defined(CONFIG_ARCH_SCX15) || defined(CONFIG_ARCH_SCX30G) || defined(CONFIG_ARCH_SCX35L)
extern ulong gsp_mmu_ctrl_addr;
#endif
#endif

/**---------------------------------------------------------------------------*
 **                         Dependencies                                      *
 **---------------------------------------------------------------------------*/

/**---------------------------------------------------------------------------*
 **                         Macro Definition                                  *
 **---------------------------------------------------------------------------*/


/**---------------------------------------------------------------------------*
 **                         Function Define                                   *
 **---------------------------------------------------------------------------*/

extern GSP_CONFIG_INFO_T s_gsp_cfg;


LOCAL void GSP_SetLayer0Parameter(void)
{
    if(!s_gsp_cfg.layer0_info.layer_en)
    {
        GSP_L0_ENABLE_SET(0);
        return ;
    }

    GSP_L0_ADDR_SET(s_gsp_cfg.layer0_info.src_addr);
    GSP_L0_PITCH_SET(s_gsp_cfg.layer0_info.pitch);
    GSP_L0_CLIPRECT_SET(s_gsp_cfg.layer0_info.clip_rect);
    GSP_L0_DESRECT_SET(s_gsp_cfg.layer0_info.des_rect);
    GSP_L0_GREY_SET(s_gsp_cfg.layer0_info.grey);
    GSP_L0_ENDIAN_SET(s_gsp_cfg.layer0_info.endian_mode);
    GSP_L0_ALPHA_SET(s_gsp_cfg.layer0_info.alpha);
    GSP_L0_COLORKEY_SET(s_gsp_cfg.layer0_info.colorkey);
    GSP_L0_IMGFORMAT_SET(s_gsp_cfg.layer0_info.img_format);
    GSP_L0_ROTMODE_SET(s_gsp_cfg.layer0_info.rot_angle);
    GSP_L0_COLORKEYENABLE_SET(s_gsp_cfg.layer0_info.colorkey_en);
    GSP_L0_PALLETENABLE_SET(s_gsp_cfg.layer0_info.pallet_en);
    //GSP_L0_SCALETAPMODE_SET(s_gsp_cfg.layer0_info.row_tap_mode,s_gsp_cfg.layer0_info.col_tap_mode);

}


LOCAL void GSP_SetLayer1Parameter(void)
{
    if(!s_gsp_cfg.layer1_info.layer_en)
    {
        GSP_L1_ENABLE_SET(0);
        return ;
    }

    GSP_L1_ADDR_SET(s_gsp_cfg.layer1_info.src_addr);
    GSP_L1_PITCH_SET(s_gsp_cfg.layer1_info.pitch);
    GSP_L1_CLIPRECT_SET(s_gsp_cfg.layer1_info.clip_rect);
    GSP_L1_DESPOS_SET(s_gsp_cfg.layer1_info.des_pos);
    GSP_L1_GREY_SET(s_gsp_cfg.layer1_info.grey);
    GSP_L1_ENDIAN_SET(s_gsp_cfg.layer1_info.endian_mode);
    GSP_L1_ALPHA_SET(s_gsp_cfg.layer1_info.alpha);
    GSP_L1_COLORKEY_SET(s_gsp_cfg.layer1_info.colorkey);
    GSP_L1_IMGFORMAT_SET(s_gsp_cfg.layer1_info.img_format);
    GSP_L1_ROTMODE_SET(s_gsp_cfg.layer1_info.rot_angle);
    GSP_L1_COLORKEYENABLE_SET(s_gsp_cfg.layer1_info.colorkey_en);
    GSP_L1_PALLETENABLE_SET(s_gsp_cfg.layer1_info.pallet_en);

}


LOCAL void GSP_SetLayerDesParameter(void)
{
    if(!s_gsp_cfg.layer0_info.layer_en && !s_gsp_cfg.layer1_info.layer_en)
    {
        return ;
    }

    GSP_Ld_ADDR_SET(s_gsp_cfg.layer_des_info.src_addr);
    GSP_Ld_PITCH_SET(s_gsp_cfg.layer_des_info.pitch);
    GSP_Ld_ENDIAN_SET(s_gsp_cfg.layer_des_info.endian_mode);
    GSP_Ld_IMGFORMAT_SET(s_gsp_cfg.layer_des_info.img_format);
    GSP_Ld_COMPRESSRGB888_SET(s_gsp_cfg.layer_des_info.compress_r8_en);
}

LOCAL void GSP_SetMiscParameter(void)
{
    if(!s_gsp_cfg.layer0_info.layer_en && !s_gsp_cfg.layer1_info.layer_en)
    {
        return ;
    }

    GSP_L0_ENABLE_SET(s_gsp_cfg.layer0_info.layer_en);
    GSP_L1_ENABLE_SET(s_gsp_cfg.layer1_info.layer_en);

    if(s_gsp_cfg.layer0_info.scaling_en == 1)
    {
        GSP_SCALESTATUS_RESET();
    }
    GSP_SCALE_ENABLE_SET(s_gsp_cfg.layer0_info.scaling_en);


    GSP_PMARGB_ENABLE_SET(s_gsp_cfg.layer0_info.pmargb_en||s_gsp_cfg.layer1_info.pmargb_en);
    GSP_L0_PMARGBMODE_SET(s_gsp_cfg.layer0_info.pmargb_mod);
    GSP_L1_PMARGBMODE_SET(s_gsp_cfg.layer1_info.pmargb_mod);
    GSP_PAGES_BOARDER_SPLIT_SET(s_gsp_cfg.misc_info.split_pages);
    GSP_Y2R_OPT_SET(s_gsp_cfg.misc_info.y2r_opt);
    GSP_DITHER_ENABLE_SET(s_gsp_cfg.misc_info.dithering_en);
    //GSP_AHB_CLOCK_SET(s_gsp_cfg.misc_info.ahb_clock);
    GSP_CLOCK_SET(s_gsp_cfg.misc_info.gsp_clock);
    GSP_EMC_GAP_SET(s_gsp_cfg.misc_info.gsp_gap);
}

PUBLIC void GSP_module_enable(void)
{
	int ret = 0;
	//GSP_HWMODULE_ENABLE();
	if(g_gsp_clk != NULL){
            ret = clk_prepare_enable(g_gsp_clk);
	    if(ret) {
	        printk(KERN_ERR "%s: enable clock failed!\n",__FUNCTION__);
	        return;
	    } else {
	        pr_debug(KERN_INFO "%s: enable clock ok!\n",__FUNCTION__);
	    }
	} else {
		printk(KERN_ERR "%s: g_gsp_clk not init yet!\n",__FUNCTION__);
	}
}
PUBLIC void GSP_module_disable(void)
{
	if(g_gsp_clk != NULL){
	    //GSP_HWMODULE_DISABLE();//disable may not use the enable regiter
            clk_disable_unprepare(g_gsp_clk);

	} else {
		printk(KERN_ERR "%s: g_gsp_clk not init yet!\n",__FUNCTION__);
	}
}

/*
func:GSP_ClocksCheckPhase0
desc: check all clock except iommu
*/
PUBLIC int GSP_ClocksCheckPhase0(void)
{
    int ret = 0;

    //check GSP enable
    if(0==(GSP_REG_READ(GSP_MOD_EN)&GSP_MOD_EN_BIT)) {
        printk(KERN_ERR "%s: err: gsp enable is not set!%lx:%08x\n",__FUNCTION__,
               (ulong)GSP_MOD_EN,GSP_REG_READ(GSP_MOD_EN));
        ret++;
    } else {
        if(GSP_WORKSTATUS_GET() != 0) {
            printk(KERN_ERR "%s: err:busy is still on!!!!\n",__FUNCTION__);
            ret++;
        }
    }

    //check GSP clock select
    if(GSP_CLK_SEL_BIT_MASK!=(GSP_REG_READ(GSP_CLOCK_BASE)&GSP_CLK_SEL_BIT_MASK)) {
        printk(KERN_INFO "%s: info: gsp clock select is not set to hightest freq!%lx:%08x\n",__FUNCTION__,
               (ulong)GSP_CLOCK_BASE,GSP_REG_READ(GSP_CLOCK_BASE));
    }

    //check GSP AUTO_GATE clock
    if(0==(GSP_REG_READ(GSP_AUTO_GATE_ENABLE_BASE)&GSP_AUTO_GATE_ENABLE_BIT)) {
        printk(KERN_ERR "%s: err: gsp auto gate clock is not enable!%lx:%08x\n",__FUNCTION__,
               (ulong)GSP_AUTO_GATE_ENABLE_BASE,GSP_REG_READ(GSP_AUTO_GATE_ENABLE_BASE));
        ret++;
    }

    //check GSP EMC clock
    if(0==(GSP_REG_READ(GSP_EMC_MATRIX_BASE)&GSP_EMC_MATRIX_BIT)) {
        printk(KERN_ERR "%s: err: gsp emc clock is not enable!%lx:%08x\n",__FUNCTION__,
               (ulong)GSP_EMC_MATRIX_BASE,GSP_REG_READ(GSP_EMC_MATRIX_BASE));
        ret++;
    }
    return (ret>0)?GSP_KERNEL_CLOCK_ERR:GSP_NO_ERR;
}

/*
func:GSP_ClocksCheckPhase1
desc: check iommu cfg
*/
PUBLIC int GSP_ClocksCheckPhase1(void)
{
    int ret = 0;
    uint32_t ctl_val = GSP_REG_READ(GSP_MMU_CTRL_BASE);
    uint32_t addr_y0 = GSP_L0_ADDRY_GET();
    uint32_t addr_uv0 = GSP_L0_ADDRUV_GET();
    uint32_t addr_va0 = GSP_L0_ADDRVA_GET();
    uint32_t addr_y1 = GSP_L1_ADDRY_GET();
    uint32_t addr_uv1 = GSP_L1_ADDRUV_GET();
    uint32_t addr_va1 = GSP_L1_ADDRVA_GET();
    uint32_t addr_yd = GSP_Ld_ADDRY_GET();
    uint32_t addr_uvd = GSP_Ld_ADDRUV_GET();
    uint32_t addr_vad = GSP_Ld_ADDRVA_GET();

#define IOVA_CHECK(addr)    (0x10000000<= (addr) && (addr) < 0x80000000)

    //check GSP IOMMU ENABLE
    if(((ctl_val & 0x1)==0 || (ctl_val & 0xF0000000)==0)/*IOMMU be disabled*/
       &&(((GSP_L0_ENABLE_GET() == 1) && (IOVA_CHECK(addr_y0) || IOVA_CHECK(addr_uv0) || IOVA_CHECK(addr_va0)))/*L0 is enabled and use iova*/
          ||((GSP_L1_ENABLE_GET() == 1) && (IOVA_CHECK(addr_y1) || IOVA_CHECK(addr_uv1) || IOVA_CHECK(addr_va1)))/*L1 is enabled and use iova*/
          ||((GSP_L0_ENABLE_GET() == 1 ||GSP_L1_ENABLE_GET() == 1 ) && (IOVA_CHECK(addr_yd) || IOVA_CHECK(addr_uvd) || IOVA_CHECK(addr_vad))))) {
        printk(KERN_ERR "%s: err: gsp iommu is not enable or iova base is null!%lx:%08x\n",__FUNCTION__,
               (ulong)GSP_MMU_CTRL_BASE,GSP_REG_READ(GSP_MMU_CTRL_BASE));
        ret++;
    }
    return (ret>0)?GSP_KERNEL_CLOCK_ERR:GSP_NO_ERR;
}


PUBLIC int GSP_Init(void)
{
    int ret = 0;
    ret = clk_prepare_enable(g_gsp_emc_clk);
    /*gsp driver take charge of the auto_gate bit instead of pm*/
    /*GSP_AUTO_GATE_ENABLE();*/
    if(ret) {
        printk(KERN_ERR "%s: enable emc clock failed!\n",__FUNCTION__);
        return GSP_KERNEL_CLOCK_ERR;
    } else {
        pr_debug(KERN_INFO "%s: enable emc clock ok!\n",__FUNCTION__);
    }

#ifndef GSP_IOMMU_WORKAROUND1//workaround gsp-iommu bug
#ifndef CONFIG_ARCH_SCX35L//sharkL bug 350028
    GSP_HWMODULE_SOFTRESET();
#endif
#endif

    GSP_IRQMODE_SET(GSP_IRQ_MODE_LEVEL);

    ret = GSP_ClocksCheckPhase0();
    return ret;
}
PUBLIC void GSP_Deinit(void)
{
    clk_disable_unprepare(g_gsp_emc_clk);
    GSP_IRQSTATUS_CLEAR();
    GSP_IRQENABLE_SET(GSP_IRQ_TYPE_DISABLE);
}

PUBLIC void GSP_ConfigLayer(GSP_MODULE_ID_E layer_id)
{
    switch(layer_id)
    {
    case GSP_MODULE_LAYER0:
        GSP_SetLayer0Parameter();
        break;

    case GSP_MODULE_LAYER1:
        GSP_SetLayer1Parameter();
        break;

    case GSP_MODULE_DST:
        GSP_SetLayerDesParameter();
        break;

    default:
        GSP_SetMiscParameter();
        break;
    }
}

PUBLIC void GSP_Wait_Finish(void)
{
    if(GSP_WORKSTATUS_GET() != 0) {
        printk(KERN_ERR "%s: err:busy is still on!!!!\n",__FUNCTION__);
    }
}


PUBLIC uint32_t GSP_Trigger(void)
{
    int ret = GSP_ClocksCheckPhase1();
    if(ret) {
        return ret;
    }

    if(GSP_ERRFLAG_GET())
    {
        //GSP_ASSERT();
        return GSP_ERRCODE_GET();
    }

    GSP_IRQENABLE_SET(GSP_IRQ_TYPE_ENABLE);
    GSP_ENGINE_TRIGGER();
    return 0;
}


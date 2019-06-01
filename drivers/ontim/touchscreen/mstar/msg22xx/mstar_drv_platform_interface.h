////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2014 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// (??MStar Confidential Information??) by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
////////////////////////////////////////////////////////////////////////////////

/**
 *
 * @file    mstar_drv_platform_interface.h
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.3.0.0
 *
 */

#ifndef __MSTAR_DRV_PLATFORM_INTERFACE_H__
#define __MSTAR_DRV_PLATFORM_INTERFACE_H__

/*--------------------------------------------------------------------------*/
/* INCLUDE FILE                                                             */
/*--------------------------------------------------------------------------*/

#include "mstar_drv_common.h"
#include "mstar_drv_platform_porting_layer.h"
#include "mstar_drv_self_fw_control.h"
#include "mstar_drv_main.h"
#include <ontim/touchscreen/touchpanel.h>


struct msg2xxx_cfg_info
{
    struct input_dev *idev ;
    struct i2c_client *i2c ;
    struct touchpanel_platform_data *pdata;
    struct work_struct work;
	struct delayed_work ps_work;
    struct mutex ts_lock;
	struct wake_lock wlock ;
    struct mutex ts_suspend_lock;
    int  irq ;
    u16  major_version ;
    u16  minor_version ;
    u16  vendor_id   ;
    u16  info_vendor ;
    u16  main_vendor ;
    u8   *vendor_name;
    u8   i2c_id_dbbus ;
    u8   ChipType ;
    u32  i2c_update_clk ;
    u32  i2c_default_clk;
    struct touchpanel_panel_parm * panel_parm;
    s32 wakeup_enable;
    struct touchpanel_wakeup_event_data  *wakeup_event; 
    s32 wakeup_event_num ;
    s32 wakeup_support ;
    u8 suspend_state;
    //u8 run_state;
    u8 ps_state ;    
    u8 ps_onoff ;
    struct input_dev * ps_input_dev ;
    
};

/*--------------------------------------------------------------------------*/
/* GLOBAL FUNCTION DECLARATION                                              */
/*--------------------------------------------------------------------------*/
extern struct msg2xxx_cfg_info *msg2xxx_info;
extern s32 /*__devinit*/ MsDrvInterfaceTouchDeviceProbe(struct i2c_client *pClient, const struct i2c_device_id *pDeviceId);
extern s32 /*__devexit*/ MsDrvInterfaceTouchDeviceRemove(struct i2c_client *pClient);
#ifdef CONFIG_HAS_EARLYSUSPEND
extern void MsDrvInterfaceTouchDeviceResume(struct early_suspend *pSuspend);        
extern void MsDrvInterfaceTouchDeviceSuspend(struct early_suspend *pSuspend);
#endif
      
#endif  /* __MSTAR_DRV_PLATFORM_INTERFACE_H__ */

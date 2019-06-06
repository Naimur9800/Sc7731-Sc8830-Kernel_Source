/*==============================================================================
 *
 * Copyright (c) 2006-2012 MStar Semiconductor, Inc.
 * All rights reserved.
 *
 * Unless otherwise stipulated in writing, any and all information contained
 * herein regardless in any format shall remain the sole proprietary of
 * MStar Semiconductor Inc. and be kept in strict confidence
 * (??MStar Confidential Information??) by the recipient.
 * Any unauthorized act including without limitation unauthorized disclosure,
 * copying, use, reproduction, sale, distribution, modification, disassembling,
 * reverse engineering and compiling of the contents of MStar Confidential
 * Information is unlawful and strictly prohibited. MStar hereby reserves the
 * rights to any and all damages, losses, costs and expenses resulting
 therefrom.
 *
==============================================================================*/

/**
 *
 * @file    mstar_drv_utility_adaption.h
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.2.0.0
 *
 */

#ifndef __MSTAR_DRV_UTILITY_ADAPTION_H__
#define __MSTAR_DRV_UTILITY_ADAPTION_H__ (1)

/* ////////////////////////////////////////////////////////// */
/* / Included Files */
/* ////////////////////////////////////////////////////////// */

#include "mstar_drv_common.h"

/* ////////////////////////////////////////////////////////// */
/* / Constant */
/* ////////////////////////////////////////////////////////// */

/* ////////////////////////////////////////////////////////// */
/* / Data Types */
/* ////////////////////////////////////////////////////////// */

/* ////////////////////////////////////////////////////////// */
/* / Variables */
/* ////////////////////////////////////////////////////////// */

/* ////////////////////////////////////////////////////////// */
/* / Macro */
/* ////////////////////////////////////////////////////////// */
#define BK_REG8_WL(addr, val)    (reg_set_lowbyte(addr, val))
#define BK_REG8_WH(addr, val)    (reg_set_hbyte(addr, val))
#define BK_REG16_W(addr, val)    (reg_set16bit_value(addr, val))
#define BK_REG8_RL(addr)        (reg_get_lowbyte(addr))
#define BK_REG8_RH(addr)        (reg_get_hbyte(addr))
#define BK_REG16_R(addr)        (reg_get16bit_value(addr))
/* ////////////////////////////////////////////////////////// */
/* / Function Prototypes */
/* ////////////////////////////////////////////////////////// */

#ifdef CONFIG_ENABLE_DMA_IIC
extern void dma_alloc(void);
extern void dma_free(void);
#endif/* CONFIG_ENABLE_DMA_IIC */
extern u16 reg_get16bit_value(u16 n_addr);
extern u8 reg_get_lowbyte(u16 n_addr);
extern u8 reg_get_hbyte(u16 n_addr);
extern void reg_set16bit_value(u16 n_addr, u16 n_data);
extern void reg_set_lowbyte(u16 n_addr, u8 n_data);
extern void reg_set_hbyte(u16 n_addr, u8 n_data);
extern void reg_set16bit_valueon(u16 n_addr, u16 n_data);
extern void reg_set16bit_valueoff(u16 n_addr, u16 n_data);
extern void bus_enter_serial_debugmode(void);
extern void bus_exit_serial_debugmode(void);
extern void bus_i2c_use_bus(void);
extern void bus_i2c_notuse_buf(void);
extern void bus_i2c_reshape(void);
extern void bus_stop_mcu(void);
extern void buf_not_stop_mcu(void);
extern s32 i2c_write_data(u8 n_slave_id, u8 *p_buf, u16 n_size);
extern s32 i2c_read_data(u8 n_slave_id, u8 *p_buf, u16 n_size);
extern void mstp_memset(void *p_dst, s8 n_nval, u32 n_size);
extern void mstp_memcopy(void *p_dst, void *p_source, u32 n_size);
extern void mstp_delay(u32 n_time);


/* ////////////////////////////////////////////////////////// */
/* / Variables */
/* ////////////////////////////////////////////////////////// */
extern struct i2c_client *gi2c_client;

#endif/* __MSTAR_DRV_UTILITY_ADAPTION_H__ */

/*
 * File:Altek_PlatformInfo.h                                     *
 * Description: Platform dependent declaration and prototype     *
 *                                                               *
 * (C)Copyright altek Corporation 2014                           *
 *                                                               *
 * History:                                                      *
 *   2016/01/11; CaedmonLai; Initial version                     *
 */
#ifndef _ALTEK_PLATFORM_INFO_H
#define _ALTEK_PLATFORM_INFO_H

#include <linux/kernel.h>
#include <linux/unistd.h>
#define _SPRD_PLATFORM_G2V1

#ifdef _SPRD_PLATFORM_G2V1
extern u32 g_iva_isp_fw_buf_addr;
extern u64 g_kva_isp_fw_buff_addr;

void *iva32_to_ka64(u32 ivaddr, u32 size);
void dma_flush_range(const void *start, const void *end);
#endif

#endif


/*
 * File: altek_iq_info_func.h                                                *
 * Description: definition of iq info structure and api                      *
 *                                                                           *
 * (C)Copyright altek Corporation 2016                                       *
 *                                                                           *
 */
#ifndef _ALTEK_IQ_INFO_FUNC_H_
#define _ALTEK_IQ_INFO_FUNC_H_

/* [IQ_INFO_FUNC] set total buffer size, must be multiple of a page(4K) */
#define IQ_INFO_FUNC_PAGE_CNT_MAX  6
#define IQ_INFO_FUNC_BUFF_SIZE_MAX (0x1000 * IQ_INFO_FUNC_PAGE_CNT_MAX)

/* [IQ_INFO_FUNC] the limit of func id's count */
#define IQ_INFO_FUNC_ID_MAX (64)

/* [IQ_INFO_FUNC] copy each defined size into array g_iq_info_func_info_size */
/* #define IQ_INFO_FUNC_ID_1_SIZE (0x67A) */

u32 iq_info_func_tbl_size_of(u32 func_id);
u32 iq_info_set_func_tbl_size(u32 func_id, u32 tbl_size);

void iq_info_func_init(void *pri_data);
void iq_info_func_deinit(void *pri_data);
#endif /* _ALTEK_IQ_INFO_FUNC_H_ */

/**********************************************************
* File: altek_iq_info_func.c                              *
* Description: Implementaion of iq general API            *
*                                                         *
* (C)Copyright altek Corporation 2016                     *
*                                                         *
* History:                                                *
***********************************************************/
#include "altek_ahb_drv.h"
#include "altek_log_local.h"
#include "altek_iq_info_func.h"

/* [IQ_INFO_FUNC] required size of each func id */
static u32 g_iq_info_func_info_size[IQ_INFO_FUNC_ID_MAX] = {0};

/************************************************************
 *              private function                            *
 ************************************************************/

/************************************************************
*		local public function		            *
*************************************************************/
/*
 *\brief return required size of each func_id
 */
u32 iq_info_func_tbl_size_of(u32 func_id)
{
	u32 size = 0;

	if (func_id < IQ_INFO_FUNC_ID_MAX)
		size = g_iq_info_func_info_size[func_id];
	else
		isp_info("no func id defined: %d", func_id);
	isp_pr_debug("func_id[%d]: 0x%x", func_id, size);
	return size;
}

u32 iq_info_set_func_tbl_size(u32 func_id, u32 fid_size)
{
	u32 err = 0;

	if (func_id >= IQ_INFO_FUNC_ID_MAX)
		return -EINVAL;
	g_iq_info_func_info_size[func_id] = fid_size;
	return err;
}
void iq_info_func_init(void *pri_data)
{
}
void iq_info_func_deinit(void *pri_data)
{
	memset(g_iq_info_func_info_size, 0x0, sizeof(g_iq_info_func_info_size));
}

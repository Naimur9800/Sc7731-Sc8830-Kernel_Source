/*
 * File:Altek_PlatformInfo.c                             *
 * Description: Platform-dependent functions             *
 *                                                       *
 * (C)Copyright altek Corporation 2014                   *
 *                                                       *
 * History:                                              *
 *   2016/01/11; CaedmonLai; Initial version             *
 */
#include <linux/kernel.h>
#include "altek_platform_info.h"
#include "altek_ahb_drv.h"
#include "altek_log_local.h"

#ifdef _SPRD_PLATFORM_G2V1
void *iva32_to_ka64(u32 ivaddr, u32 size)
{
	void *map_addr = NULL;

	isp_debug_lo_start_tag();

	isp_debug_item_tag(
		"map addr 0x%x base_iommu_addr 0x%x base kaddr 0x%llx size 0x%x",
		ivaddr, g_iva_isp_fw_buf_addr, g_kva_isp_fw_buff_addr, size);
	if ((g_iva_isp_fw_buf_addr == 0) || (g_kva_isp_fw_buff_addr == 0)
		|| (ivaddr < g_iva_isp_fw_buf_addr)) {
		isp_err_lo_combo_desc_tag("invalid address\n");
	} else {
		map_addr = (void *)(g_kva_isp_fw_buff_addr +
		(ivaddr - g_iva_isp_fw_buf_addr));
	}

	isp_debug_lo_end_tag();

	return map_addr;
}

void dma_flush_range(const void *start, const void *end)
{
	/* Remove for iwhale platform */
	/*__dma_flush_range(start, end);*/
}
#endif

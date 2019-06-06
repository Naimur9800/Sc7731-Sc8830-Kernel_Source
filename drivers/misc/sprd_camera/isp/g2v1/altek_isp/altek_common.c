/*
 * File: altek_local.h                                                       *
 * Description: implementation of local shared api                            *
 *                                                                           *
 * (C)Copyright altek Corporation 2016                                       *
 *                                                                           *
 * History:                                                                  *
 *   2016/03/012; Caedmon Lai; Initial version                               *
 */

/************************************************************
*		local public function		            *
*************************************************************/
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/uaccess.h>
#include <linux/fcntl.h>


#include "altek_ahb_drv.h"
#include "altek_iq_info_func.h"
#include "altek_iq_info_local.h"
#include "altek_isp_drv.h"
#include "altek_isp_drv_local.h"
#include "altek_bist.h"
#include "altek_common.h"
#include "altek_log_local.h"

/* Protect writing ahb  reg*/
static struct semaphore command_sem;

static struct common_ispdrv_init_mod mod_list[] = {
	{"bist", bist_init, bist_deinit, NULL},
	{"iq_info", iq_info_init, iq_info_deinit, NULL},
	{"iq_info_func", iq_info_func_init, iq_info_func_deinit, NULL},
	{"isp_drv", isp_drv_init, isp_drv_deinit, NULL},
};

struct semaphore *get_command_semaphore(void)
{
	return &command_sem;
}

static void upload_linux_data_file(u64 Addr, char *data, unsigned int size)
{
	unsigned int *p_src;
	unsigned int i;
	u64 start_addr;
	unsigned int write_size;
	unsigned int data_size = size;

	for (i = 0; i < data_size; i += PAGE_SIZE_4K) {
		write_size =
		    ((data_size - i) >
		     PAGE_SIZE_4K) ? PAGE_SIZE_4K : (data_size - i);
		start_addr = Addr + i;
		p_src = (unsigned int *)(data + i);
		MEMCPY((void *)start_addr, (void *)p_src, write_size);
	}
}

void upload_data_file_mmu(unsigned int Addr, char *data,
				 unsigned int size)
{
	unsigned int *p_src;
	unsigned int *p_dword;
	unsigned int i;
	unsigned int start_addr;
	unsigned int write_size;
	unsigned int data_size = size;

	for (i = 0; i < data_size; i += PAGE_SIZE_4K) {
		write_size =
		    ((data_size - i) >
		     PAGE_SIZE_4K) ? PAGE_SIZE_4K : (data_size - i);
		start_addr = Addr + i;
		p_src = (unsigned int *)(data + i);

		p_dword = (unsigned int *)(MAP_IVA_TO_KVA(start_addr,
					 write_size));	/*  map 4K once*/

		if (!p_dword) {
			isp_err_lo_combo_desc_tag
			("** upload_data_file_mmu MAP_IVA_TO_KVA failed:\n");
			return;
		}
		MEMCPY((u8 *) p_dword, (u8 *) p_src, write_size);
		UNMAP_IVA_TO_KVA(p_dword);
	}

}


/* Accurate the checksum to compare two images for verifying  paths*/
void get_sum_xor_value(u8 *addr, u32 size, u32 *a_pud_sum,
			      u32 *a_pud_xor)
{
	u32 i, Shift[4] = { 0, 8, 16, 24 };
	u8 *addr1 = addr;

	*a_pud_sum = 0;
	*a_pud_xor = 0;

	for (i = 0; i < size; i++) {
		*a_pud_sum += *(addr1 + i) << (Shift[i % 4]);
		*a_pud_xor ^= *(addr1 + i) << (Shift[i % 4]);
	}
}


u32 load_golden_img(void *v4l2_device)
{

	const struct firmware *golden_ptr;

	u32 err = 0;
	u8 *data = 0;

	u64 direct_addr = 0,
		dram_lv_video_addr = 0,
		dram_normal_quality_still_addr = 0,
		dram_high_quality_still_addr = 0,
		front_addr = 0;

	isp_debug_lo_start_tag();

	bist_get_golden_addr(&direct_addr,
					       &dram_lv_video_addr,
					       &dram_normal_quality_still_addr,
					       &dram_high_quality_still_addr,
					       &front_addr);

	/*  Normal flow*/
	if (bist_get_self_test_addr_set() == 0) {
		isp_err_lo_combo_desc_tag("Not self test mode\n");
		goto load_golden_img_end;
	}

	/* Load Golden - Direct*/
	if (request_firmware
	(&golden_ptr, "Golden_Direct.raw",
	(struct device *)v4l2_device)) {
		isp_err_lo_combo_desc_tag(
			"ISP: Load golden direct img failed\n");
		goto load_golden_img_end;
	}

	if (golden_ptr->size > MAX_SELF_TEST_GOLDEN_SIZE_960x720) {
		isp_err_lo_combo_desc_tag(
			"ISP: Golden Direct IMG size is too big.\n");
		goto load_golden_img_end;
	}
	upload_linux_data_file(direct_addr, (void *)golden_ptr->data,
		golden_ptr->size);
	release_firmware(golden_ptr);

	data = (u8 *) direct_addr;
	isp_debug_item_tag("GoldenData:  %x %x %x %x %x %x %x %x\n",
		*(data), *(data + 1),
		*(data + 2), *(data + 3), *(data + 4), *(data + 5),
		*(data + 6), *(data + 7));

	/* Load Golden - Dram LV/VIDEO*/
	if (request_firmware
	(&golden_ptr, "Golden_Dram_LV_VIDEO.raw",
	(struct device *)v4l2_device)) {
		isp_err_lo_combo_desc_tag(
			"ISP: Load golden dram LV/VIDEO img failed\n");
		goto load_golden_img_end;
	}

	if (golden_ptr->size > MAX_SELF_TEST_GOLDEN_SIZE_640x360) {
		isp_err_lo_combo_desc_tag(
			"ISP: Golden dram LV/VIDEO IMG size is too big.\n");
		goto load_golden_img_end;
	}

	upload_linux_data_file(dram_lv_video_addr, (void *)golden_ptr->data,
		golden_ptr->size);
	release_firmware(golden_ptr);

	/* Load Golden - Dram NQ STILL*/
	if (request_firmware
	(&golden_ptr, "Golden_Dram_NQ_STILL.raw",
	(struct device *)v4l2_device)) {
		isp_err_lo_combo_desc_tag(
			"ISP: Load golden dram NQ STILL img failed\n");
		goto load_golden_img_end;
	}

	if (golden_ptr->size > MAX_SELF_TEST_GOLDEN_SIZE_1920x1080) {
		isp_err_lo_combo_desc_tag(
			"ISP: Golden Dram NQ STILL IMG size is too big.\n");
		goto load_golden_img_end;
	}

	upload_linux_data_file(dram_normal_quality_still_addr,
		(void *)golden_ptr->data, golden_ptr->size);
	release_firmware(golden_ptr);

	/* Load Golden - Dram HQ STILL*/
	if (request_firmware
	(&golden_ptr, "Golden_Dram_HQ_STILL.raw",
	(struct device *)v4l2_device)) {
		isp_err_lo_combo_desc_tag(
			"ISP: Load golden dram HQ STILL img failed\n");
		goto load_golden_img_end;
	}

	if (golden_ptr->size > MAX_SELF_TEST_GOLDEN_SIZE_1920x1080) {
		isp_err_lo_combo_desc_tag(
			"ISP: Golden Dram HQ STILL IMG size is too big.\n");
		goto load_golden_img_end;
	}
	upload_linux_data_file(dram_high_quality_still_addr,
				(void *)golden_ptr->data, golden_ptr->size);
	release_firmware(golden_ptr);

	/* Load Golden - front*/
	if (request_firmware
		(&golden_ptr, "Golden_Front.raw",
		(struct device *)v4l2_device)) {
		isp_err_lo_combo_desc_tag(
			"ISP: Load golden front img failed\n");
		goto load_golden_img_end;
	}

	if (golden_ptr->size > MAX_SELF_TEST_GOLDEN_SIZE_1920x1080) {
		isp_err_lo_combo_desc_tag(
			"ISP: Golden Front IMG size is too big.\n");
		goto load_golden_img_end;
	}

	upload_linux_data_file(front_addr, (void *)golden_ptr->data,
		golden_ptr->size);
	release_firmware(golden_ptr);

load_golden_img_end:

	isp_debug_lo_end_tag();
	return err;

}

void common_ispdrv_init(void)
{
	int i;
	int size;
	void *temp = NULL;

	size = sizeof(mod_list) / sizeof(struct common_ispdrv_init_mod);
	for (i = 0; i < size; i++) {
		if (mod_list[i].init_cb == NULL)
			continue;
		isp_pr_debug("init[%d]: %s", i, mod_list[i].name);
		/* init pri_data if needed */
		mod_list[i].init_cb(temp);
		if (temp)
			mod_list[i].pri_data = temp;
	}
}

void common_ispdrv_deinit(void)
{
	int i;
	int size;

	size = sizeof(mod_list) / sizeof(struct common_ispdrv_init_mod);
	for (i = 0; i < size; i++) {
		if (mod_list[i].deinit_cb == NULL)
			continue;
		isp_pr_debug("deinit[%d]: %s", i, mod_list[i].name);
		mod_list[i].deinit_cb(mod_list[i].pri_data);
		/* free mem if needed */
		mod_list[i].pri_data = NULL;
	}
}

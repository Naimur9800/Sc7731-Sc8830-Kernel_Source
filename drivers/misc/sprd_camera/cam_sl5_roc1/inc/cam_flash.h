/*
 * Copyright (C) 2017-2018 Spreadtrum Communications Inc.
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

#ifndef _CAM_FLASH_H_
#define _CAM_FLASH_H_

struct flash_led_task {
	struct sprd_img_set_flash set_flash;
	uint32_t frame_skipped;
	uint32_t after_af;
	struct timeval timestamp;
	uint32_t skip_number;	/*cap skip*/
	uint32_t cam_idx;

	struct completion flash_thread_com;
	struct task_struct *flash_thread;
	uint32_t is_flash_thread_stop;
};


int cam_create_flash_task(
	struct flash_led_task **param, uint32_t cam_idx);
int cam_destroy_flash_task(struct flash_led_task *param);
int cam_start_flash(void *param);


#endif /* _CAM_FLASH_H_ */

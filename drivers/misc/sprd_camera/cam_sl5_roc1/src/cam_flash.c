/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
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


#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <video/sprd_mm.h>
#include <video/sprd_img.h>

#include "cam_types.h"

#include "cam_flash.h"
#include "flash_drv.h"


static int cam_set_flash(uint32_t idx,
			     struct sprd_img_set_flash *set_flash)
{
	sprd_flash_ctrl(set_flash);
	pr_debug("cam %d set flash\n", idx);

	return 0;
}

static int img_opt_flash(void *param)
{
	struct flash_led_task *falsh_task = (struct flash_led_task *)param;
	uint32_t led0_ctrl;
	uint32_t led1_ctrl;
	uint32_t led0_status;
	uint32_t led1_status;

	led0_ctrl = falsh_task->set_flash.led0_ctrl;
	led1_ctrl = falsh_task->set_flash.led1_ctrl;
	led0_status = falsh_task->set_flash.led0_status;
	led1_status = falsh_task->set_flash.led1_status;

	if ((led0_ctrl && led0_status < FLASH_STATUS_MAX) ||
	    (led1_ctrl && led1_status < FLASH_STATUS_MAX)) {
		pr_debug("led0_status %d led1_status %d\n",
			   led0_status, led1_status);
		if (led0_status == FLASH_CLOSE_AFTER_AUTOFOCUS ||
		    led1_status == FLASH_CLOSE_AFTER_AUTOFOCUS) {
			/*cam_get_timestamp(&falsh_task->timestamp);*/
			falsh_task->after_af = 1;
			pr_debug("time, %d %d\n",
				   (int)falsh_task->timestamp.tv_sec,
				   (int)falsh_task->timestamp.tv_usec);
		}
		cam_set_flash(falsh_task->cam_idx, &falsh_task->set_flash);
		falsh_task->set_flash.led0_ctrl = 0;
		falsh_task->set_flash.led1_ctrl = 0;
		falsh_task->set_flash.led0_status = FLASH_STATUS_MAX;
		falsh_task->set_flash.led1_status = FLASH_STATUS_MAX;
	}

	return 0;
}

static int flash_thread_loop(void *arg)
{
	struct flash_led_task *falsh_task = (struct flash_led_task *)arg;
	struct sprd_img_set_flash set_flash;

	if (falsh_task == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	while (1) {
		if (wait_for_completion_interruptible(
			&falsh_task->flash_thread_com) == 0) {
			if (falsh_task->is_flash_thread_stop) {
				set_flash.led0_ctrl = 1;
				set_flash.led1_ctrl = 1;
				set_flash.led0_status = FLASH_CLOSE;
				set_flash.led1_status = FLASH_CLOSE;
				set_flash.flash_index = 0;
				cam_set_flash(falsh_task->cam_idx,
					&set_flash);
				set_flash.flash_index = 1;
				cam_set_flash(falsh_task->cam_idx,
					&set_flash);
				pr_debug("_flash_thread_loop stop\n");
				break;
			}
			img_opt_flash(arg);
		} else {
			pr_debug("flash int!");
			break;
		}
	}
	falsh_task->is_flash_thread_stop = 0;

	return 0;
}


int cam_start_flash(void *param)
{
	struct flash_led_task *falsh_task = (struct flash_led_task *)param;
	uint32_t need_light = 1;
	uint32_t led0_ctrl;
	uint32_t led1_ctrl;
	uint32_t led0_status;
	uint32_t led1_status;

	if (falsh_task == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	led0_ctrl = falsh_task->set_flash.led0_ctrl;
	led1_ctrl = falsh_task->set_flash.led1_ctrl;
	led0_status = falsh_task->set_flash.led0_status;
	led1_status = falsh_task->set_flash.led1_status;

	if ((led0_ctrl && led0_status < FLASH_STATUS_MAX) ||
		(led1_ctrl && led1_status < FLASH_STATUS_MAX)) {
		if ((led0_ctrl && FLASH_HIGH_LIGHT == led0_status) ||
			(led1_ctrl && FLASH_HIGH_LIGHT == led1_status)) {
			falsh_task->frame_skipped++;
			if (falsh_task->frame_skipped >=
				falsh_task->skip_number) {
				/* flash lighted at the last SOF before
				* the right capture frame
				*/
				pr_debug("waiting finished\n");
			} else {
				need_light = 0;
				pr_debug("wait for the next SOF, %d %d\n",
					falsh_task->frame_skipped,
					falsh_task->skip_number);
			}
		}
		if (need_light)
			complete(&falsh_task->flash_thread_com);
	}

	return 0;
}


int cam_create_flash_task(
		struct flash_led_task **param,
		uint32_t cam_idx)
{
	struct flash_led_task *flash_task = NULL;
	char thread_name[20] = { 0 };

	flash_task = kzalloc(sizeof(*flash_task), GFP_KERNEL);

	if (flash_task == NULL) {
		pr_err("fail to alloc flash task.\n");
		return -ENOMEM;
	}
	flash_task->cam_idx = cam_idx;
	flash_task->set_flash.led0_ctrl = 0;
	flash_task->set_flash.led1_ctrl = 0;
	flash_task->set_flash.led0_status = FLASH_STATUS_MAX;
	flash_task->set_flash.led1_status = FLASH_STATUS_MAX;
	flash_task->set_flash.flash_index = 0;

	flash_task->after_af = 0;

	flash_task->is_flash_thread_stop = 0;
	init_completion(&flash_task->flash_thread_com);
	sprintf(thread_name, "cam%d_flash_thread", flash_task->cam_idx);
	flash_task->flash_thread = kthread_run(flash_thread_loop,
							flash_task, thread_name);
	if (IS_ERR_OR_NULL(flash_task->flash_thread)) {
		pr_err("fail to create flash thread\n");
		kfree(flash_task);
		return -EINVAL;
	}
	*param = flash_task;
	return 0;
}


int cam_destroy_flash_task(struct flash_led_task *param)
{
	struct flash_led_task *falsh_task = (struct flash_led_task *)param;

	if (falsh_task == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	if (falsh_task->flash_thread) {
		falsh_task->is_flash_thread_stop = 1;
		complete(&falsh_task->flash_thread_com);
		if (falsh_task->is_flash_thread_stop != 0) {
			while (falsh_task->is_flash_thread_stop)
				udelay(1000);
		}
		falsh_task->flash_thread = NULL;
	}
	kfree(falsh_task);

	return 0;
}


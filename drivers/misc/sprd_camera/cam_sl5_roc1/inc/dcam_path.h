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

#ifndef _DCAM_PATH_H_
#define _DCAM_PATH_H_



#define DCAM_PATH_WMAX  5000
#define DCAM_PATH_HMAX  4000


int dcam_cfg_path(void *dcam_handle,
				struct dcam_path_desc *path,
				void *param);

int dcam_start_path(void *dcam_handle, struct dcam_path_desc *path);

int dcam_stop_path(void *dcam_handle, struct dcam_path_desc *path);

int dcam_path_set_store_frm(
			void *dcam_handle,
			struct dcam_path_desc *path);

int dcam_start_fetch(void *dcam_handle, struct dcam_fetch_info *fetch);
#endif

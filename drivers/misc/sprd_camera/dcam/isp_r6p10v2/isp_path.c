/*
 * Copyright (C) 2016 Spreadtrum Communications Inc.
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

#include <linux/kernel.h>
#include <linux/sprd_iommu.h>

#include "isp_path.h"
#include "isp_buf.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_PATH: %d " fmt, __LINE__

#define ISP_PATH_ALIGN_SIZE		2
#define ISP_ALIGNTO(size)	((size) & ~(ISP_PATH_ALIGN_SIZE - 1))

#define SHRINK_Y_UP_TH 235
#define SHRINK_Y_DN_TH 16
#define SHRINK_UV_UP_TH 240
#define SHRINK_UV_DN_TH 16
#define SHRINK_Y_OFFSET 16
#define SHRINK_Y_RANGE 2
#define SHRINK_C_OFFSET 16
#define SHRINK_C_RANGE 6

#define  MAX(_x, _y) (((_x) > (_y)) ? (_x) : (_y))
#define  MIN(_x, _y) (((_x) < (_y)) ? (_x) : (_y))

static int set_offline_scl_size(struct isp_path_desc *path,
	struct isp_store_cce_desc *store_cce)
{
	if (!path || !store_cce) {
		pr_err("Input ptr error!\n");
		return -EFAULT;
	}

	/* src size */
	path->src.w = store_cce->dst.w;
	path->src.h = store_cce->dst.h;
	/* trim0 size */
	path->trim0_info.start_x = path->in_rect.x -
		store_cce->border.left_border;
	path->trim0_info.start_y = path->in_rect.y -
		store_cce->border.up_border;
	path->trim0_info.size_x = path->in_rect.w;
	path->trim0_info.size_y = path->in_rect.h;
	/* dst size */
	path->dst.w = path->out_size.w;
	path->dst.h = path->out_size.h;
	/* trim1 size */
	path->trim1_info.start_x = 0;
	path->trim1_info.start_y = 0;
	path->trim1_info.size_x = path->dst.w;
	path->trim1_info.size_y = path->dst.h;
	path->path_sel = 1;

	return 0;
}

static int isp_path_offline_cowork_cap(struct isp_module *module,
		struct isp_path_desc *scl0,
		struct isp_path_desc *pre,
		struct isp_path_desc *vid,
		struct isp_path_desc *cap,
		struct isp_store_cce_desc *store_cce)
{
	int rtn = 0;
	struct isp_path_desc *max_path = NULL;
	unsigned int min_recty = 0;
	unsigned int tmph = 0;

	if (!module) {
		pr_err("input ptr is NULL!\n");
		return -EFAULT;
	}

	pre = &module->isp_path[ISP_SCL_PRE];
	vid = &module->isp_path[ISP_SCL_VID];
	cap = &module->isp_path[ISP_SCL_CAP];
	store_cce = &module->store_cce;

	if (pre->in_rect.w > vid->in_rect.w)
		max_path = pre;
	else
		max_path = vid;

	if (max_path->in_rect.w < cap->in_rect.w)
		max_path = cap;

	if (pre->in_rect.y < vid->in_rect.y)
		min_recty = pre->in_rect.y;
	else
		min_recty = vid->in_rect.y;

	if (min_recty > cap->in_rect.y)
		min_recty = cap->in_rect.y;

	store_cce->src.w = max_path->in_size.w;
	store_cce->src.h = max_path->in_size.h;
	store_cce->border.left_border = max_path->in_rect.x;
	store_cce->border.up_border = min_recty;
	store_cce->border.right_border = store_cce->src.w -
		max_path->in_rect.w - store_cce->border.left_border;
	tmph = max_path->in_rect.w * max_path->in_size.h /
		max_path->in_size.w;
	tmph = ISP_ALIGNTO(tmph);
	store_cce->border.down_border = store_cce->src.h - tmph -
		store_cce->border.up_border;
	store_cce->dst.w = max_path->in_rect.w;
	store_cce->dst.h = tmph;

	rtn = set_offline_scl_size(pre, store_cce);
	if (rtn) {
		pr_err("Set pre_path scl size failed\n");
		return rtn;
	}

	rtn = set_offline_scl_size(vid, store_cce);
	if (rtn) {
		pr_err("Set vid_path scl size failed\n");
		return rtn;
	}

	rtn = set_offline_scl_size(cap, store_cce);
	if (rtn) {
		pr_err("Set cap_path scl size failed\n");
		return rtn;
	}

	store_cce->valid = 1;

	return rtn;
}

static int isp_path_offline_cowork(struct isp_path_desc *pre,
	struct isp_path_desc *vid, struct isp_store_cce_desc *store_cce)
{
	int rtn = 0;
	struct isp_path_desc *max_path = NULL;
	struct isp_path_desc *min_path = NULL;
	unsigned int tmph = 0;

	if (!pre || !vid || !store_cce) {
		pr_err("Input ptr is NULL\n");
		return -EFAULT;
	}

	if (pre->in_rect.w > vid->in_rect.w) {
		max_path = pre;
		min_path = vid;
	} else {
		max_path = vid;
		min_path = pre;
	}

	store_cce->src.w = max_path->in_size.w;
	store_cce->src.h = max_path->in_size.h;
	store_cce->border.left_border = max_path->in_rect.x;
	store_cce->border.up_border = max_path->in_rect.y > min_path->in_rect.y
		? min_path->in_rect.y : max_path->in_rect.y;
	store_cce->border.right_border = store_cce->src.w -
		max_path->in_rect.w - store_cce->border.left_border;
	tmph = max_path->in_rect.w * max_path->in_size.h /
		max_path->in_size.w;
	tmph = ISP_ALIGNTO(tmph);
	store_cce->border.down_border = store_cce->src.h - tmph -
		store_cce->border.up_border;
	store_cce->dst.w = max_path->in_rect.w;
	store_cce->dst.h = tmph;

	rtn = set_offline_scl_size(max_path, store_cce);
	if (rtn) {
		pr_err("Set max_path scl size failed\n");
		return rtn;
	}

	rtn = set_offline_scl_size(min_path, store_cce);
	if (rtn) {
		pr_err("Set min_path scl size failed\n");
		return rtn;
	}

	store_cce->valid = 1;

	return rtn;
}

static int set_online_scl_size(struct isp_path_desc *path,
	struct isp_path_desc *scl0)
{
	if (!path || !scl0) {
		pr_err("Input ptr error!\n");
		return -EFAULT;
	}

	/* src size */
	path->src.w = scl0->trim1_info.size_x;
	path->src.h = scl0->trim1_info.size_y;
	/* trim0 size */
	path->trim0_info.size_x = path->in_rect.w;
	path->trim0_info.size_x = path->trim0_info.size_x * scl0->dst.w /
		scl0->trim0_info.size_x;
	path->trim0_info.size_x = ISP_ALIGNTO(path->trim0_info.size_x);
	path->trim0_info.size_x = MIN(path->trim0_info.size_x, path->src.w);
	path->trim0_info.size_y = path->in_rect.h;
	path->trim0_info.size_y = path->trim0_info.size_y * scl0->dst.h /
		scl0->trim0_info.size_y;
	path->trim0_info.size_y = ISP_ALIGNTO(path->trim0_info.size_y);
	path->trim0_info.size_y = MIN(path->trim0_info.size_y, path->src.h);
	path->trim0_info.start_x = abs(path->src.w -
		path->trim0_info.size_x) / 2;
	path->trim0_info.start_x = ISP_ALIGNTO(path->trim0_info.start_x);
	path->trim0_info.start_y = abs(path->src.h -
		path->trim0_info.size_y) / 2;
	path->trim0_info.start_y = ISP_ALIGNTO(path->trim0_info.start_y);

	/* dst size */
	path->dst.w = path->out_size.w;
	path->dst.h = path->out_size.h;
	/* trim1 size */
	path->trim1_info.start_x = 0;
	path->trim1_info.start_y = 0;
	path->trim1_info.size_x = path->dst.w;
	path->trim1_info.size_y = path->dst.h;
	path->path_sel = 0;

	pr_info("path isp_path_online_cowork src %d %d dst %d %d trim0 %d %d %d %d trim1 %d %d %d %d\n",
		path->src.w, path->src.h,
		path->dst.w, path->dst.h,
		path->trim0_info.start_x,
		path->trim0_info.start_y,
		path->trim0_info.size_x,
		path->trim0_info.size_y,
		path->trim1_info.start_x,
		path->trim1_info.start_y,
		path->trim1_info.size_x,
		path->trim1_info.size_y);

	return 0;
}

static int isp_path_online_cowork(enum isp_id idx, struct isp_path_desc *pre,
	struct isp_path_desc *vid, struct isp_path_desc *scl0)
{
	int rtn = 0;
	struct isp_path_desc *max_path = NULL;
	struct isp_path_desc *min_path = NULL;
	int scl0_max_width = ISP_SCL0_MAX_WIDTH;

	if (idx == ISP_ID_1)
		scl0_max_width = ISP1_SCL0_MAX_WIDTH;

	if (!pre || !vid || !scl0) {
		pr_err("Input ptr is NULL\n");
		return -EFAULT;
	}

	if (pre->in_rect.w > vid->in_rect.w) {
		max_path = pre;
		min_path = vid;
	} else {
		max_path = vid;
		min_path = pre;
	}

	/* scl0_max_width change here based on application scene
	 * eg: MAX & Default:with 2304; 3DNR with 720P; path cowork
	 * with 1080P; EIS on with 2304;
	 */

	if (scl0_max_width > max_path->in_size.w)
		scl0_max_width = max_path->in_size.w;

	scl0->src.w = max_path->in_size.w;
	scl0->src.h = max_path->in_size.h;
	scl0->dst.w = scl0_max_width;
	scl0->dst.h = max_path->out_size.h * scl0->dst.w /
		max_path->out_size.w;
	if (max_path->in_rect.h != min_path->in_rect.h) {
		scl0->dst.h = MAX(scl0->dst.h, max_path->in_rect.h);
		scl0->dst.h = MAX(scl0->dst.h, min_path->in_rect.h);
	}
	scl0->dst.h = ISP_ALIGNTO(scl0->dst.h);
	scl0->dst.h = MIN(scl0->dst.h, scl0->src.h);

	if (max_path->in_rect.w > scl0_max_width) {
		scl0->trim0_info.size_x = max_path->in_rect.w;
		scl0->trim0_info.size_x = MIN(scl0->trim0_info.size_x,
				scl0->src.w);
		scl0->trim0_info.size_x = MAX(scl0->trim0_info.size_x,
				scl0->dst.w);
		scl0->trim0_info.size_y = MAX(max_path->in_rect.h,
				min_path->in_rect.h);
		scl0->trim0_info.size_y = MIN(scl0->trim0_info.size_y,
				scl0->src.h);
		scl0->trim0_info.size_y = MAX(scl0->trim0_info.size_y,
				scl0->dst.h);
		scl0->trim0_info.start_x = abs(scl0->src.w -
				scl0->trim0_info.size_x) / 2;
		scl0->trim0_info.start_x =
			ISP_ALIGNTO(scl0->trim0_info.start_x);
		scl0->trim0_info.start_y = abs(scl0->src.h -
				scl0->trim0_info.size_y) / 2;
		scl0->trim0_info.start_y =
			ISP_ALIGNTO(scl0->trim0_info.start_y);

	} else {
		scl0->trim0_info.size_x = scl0_max_width;
		scl0->trim0_info.size_y = max_path->out_size.h *
			scl0_max_width / max_path->out_size.w;
		scl0->trim0_info.size_y =
			ISP_ALIGNTO(scl0->trim0_info.size_y);
		scl0->trim0_info.size_y = MIN(scl0->trim0_info.size_y,
			scl0->src.h);
		scl0->trim0_info.size_y = MAX(scl0->trim0_info.size_y,
			scl0->dst.h);
		scl0->trim0_info.start_x = abs(scl0->src.w -
			scl0->trim0_info.size_x) / 2;
		scl0->trim0_info.start_x =
			ISP_ALIGNTO(scl0->trim0_info.start_x);
		scl0->trim0_info.start_y = abs(scl0->src.h -
			scl0->trim0_info.size_y) / 2;
		scl0->trim0_info.start_y =
			ISP_ALIGNTO(scl0->trim0_info.start_y);
	}

	scl0->trim1_info.start_x = 0;
	scl0->trim1_info.start_y = 0;
	scl0->trim1_info.size_x = scl0->dst.w;
	scl0->trim1_info.size_y = scl0->dst.h;

	rtn = set_online_scl_size(max_path, scl0);
	if (rtn) {
		pr_err("Set max_path scl size failed\n");
		return rtn;
	}

	rtn = set_online_scl_size(min_path, scl0);
	if (rtn) {
		pr_err("Set min_path scl size failed\n");
		return rtn;
	}

	scl0->valid = 1;

	return rtn;
}

static int isp_path_offline(struct isp_path_desc *path,
		struct isp_path_desc *scl0,
		struct isp_store_cce_desc *store_cce,
		struct isp_module *module)
{
	int rtn = 0;
	enum isp_id idx = 0;
	int isp_max_width = ISP_SCL0_MAX_WIDTH;

	if (!path || !scl0 || !store_cce || !module) {
		pr_err("Input ptr is NULL\n");
		return -EFAULT;
	}

	idx = module->idx;

	if (idx == ISP_ID_1)
		isp_max_width = ISP1_SCL0_MAX_WIDTH;


	if (scl0->valid != 1) {
		scl0->valid = 1;
		scl0->src.w = path->in_size.w;
		scl0->src.h = path->in_size.h;
		scl0->trim0_info.start_x = path->in_rect.x;
		scl0->trim0_info.start_y = path->in_rect.y;
		scl0->trim0_info.size_x = path->in_rect.w;
		scl0->trim0_info.size_y = path->in_rect.h;
		if (path->in_rect.w > isp_max_width) {
			scl0->dst.w = isp_max_width;
			scl0->dst.h = path->in_rect.h * scl0->dst.w /
				path->in_rect.w;
			scl0->dst.h = ISP_ALIGNTO(scl0->dst.h);
		} else {
			scl0->dst.w = scl0->trim0_info.size_x;
			scl0->dst.h = scl0->trim0_info.size_y;
		}
		scl0->trim1_info.start_x = 0;
		scl0->trim1_info.start_y = 0;
		scl0->trim1_info.size_x = scl0->dst.w;
		scl0->trim1_info.size_y = scl0->dst.h;
	}

	store_cce->src.w = path->in_size.w;
	store_cce->src.h = path->in_size.h;
	store_cce->border.left_border = path->in_rect.x;
	store_cce->border.up_border = path->in_rect.y;
	store_cce->border.right_border = path->in_size.w - path->in_rect.w -
		store_cce->border.left_border;
	store_cce->border.down_border = path->in_size.h - path->in_rect.h -
		store_cce->border.up_border;
	store_cce->dst.w = path->in_rect.w;
	store_cce->dst.h = path->in_rect.h;
	path->src.w = store_cce->dst.w;
	path->src.h = store_cce->dst.h;
	path->trim0_info.start_x = 0;
	path->trim0_info.start_y = 0;
	path->trim0_info.size_x = path->src.w;
	path->trim0_info.size_y = path->src.h;
	path->dst.w = path->out_size.w;
	path->dst.h = path->out_size.h;
	path->trim1_info.start_x = 0;
	path->trim1_info.start_y = 0;
	path->trim1_info.size_x = path->dst.w;
	path->trim1_info.size_y = path->dst.h;
	path->path_sel = 1;
	store_cce->valid = 1;
#ifdef ISP_PW_SAVE
	if (store_cce->is_raw_cap == 1)
		store_cce->pw_save = 0;
	else
		store_cce->pw_save = 1;
#endif
	pr_debug("path offline src %d %d dst %d %d trim0 %d %d %d %d trim1 %d %d %d %d\n",
		path->src.w, path->src.h,
		path->dst.w, path->dst.h,
		path->trim0_info.start_x,
		path->trim0_info.start_y,
		path->trim0_info.size_x,
		path->trim0_info.size_y,
		path->trim1_info.start_x,
		path->trim1_info.start_y,
		path->trim1_info.size_x,
		path->trim1_info.size_y);

	return rtn;
}

static int isp_path_online(enum isp_id idx, struct isp_path_desc *path,
	struct isp_path_desc *scl0)
{
	int rtn = 0;
	int scl0_max_width = ISP_SCL0_MAX_WIDTH;

	if (idx == ISP_ID_1)
		scl0_max_width = ISP1_SCL0_MAX_WIDTH;

	if (!path || !scl0) {
		pr_err("Input ptr is NULL");
		return -EFAULT;
	}

	/* scl0_max_width change here based on application scene
	 * eg: MAX & Default:with 2304; 3DNR with 720P; path cowork
	 * with 1080P; EIS on with 2304;
	 */

	if (scl0_max_width > path->in_size.w)
		scl0_max_width = path->in_size.w;
	scl0->src.w = path->in_size.w;
	scl0->src.h = path->in_size.h;
	scl0->dst.w = scl0_max_width;
	scl0->dst.h = path->out_size.h * scl0->dst.w /
		path->out_size.w;
	scl0->dst.h = ISP_ALIGNTO(scl0->dst.h);
	scl0->dst.h = MIN(scl0->dst.h, scl0->src.h);
	scl0->trim1_info.start_x = 0;
	scl0->trim1_info.start_y = 0;
	scl0->trim1_info.size_x = scl0->dst.w;
	scl0->trim1_info.size_y = scl0->dst.h;

	path->src.w = scl0->trim1_info.size_x;
	path->src.h = scl0->trim1_info.size_y;

	if (path->in_rect.w > scl0_max_width) {
		scl0->trim0_info.size_x = path->in_rect.w;
		scl0->trim0_info.size_y = path->in_rect.h;
		scl0->trim0_info.size_y = MAX(scl0->trim0_info.size_y,
			scl0->dst.h);
		scl0->trim0_info.start_x = path->in_rect.x;
		scl0->trim0_info.start_y = abs(scl0->src.h -
			scl0->trim0_info.size_y) / 2;
		scl0->trim0_info.start_y =
			ISP_ALIGNTO(scl0->trim0_info.start_y);
		scl0->trim0_info.start_y = MIN(path->in_rect.y,
			scl0->trim0_info.start_y);
		path->trim0_info.start_x = 0;
		path->trim0_info.start_y = 0;
		path->trim0_info.size_x = path->src.w;
		path->trim0_info.size_y = path->src.h;
	} else {
		scl0->trim0_info.size_x = scl0_max_width;
		scl0->trim0_info.size_y = path->out_size.h *
			scl0_max_width / path->out_size.w;
		scl0->trim0_info.size_y =
			ISP_ALIGNTO(scl0->trim0_info.size_y);
		scl0->trim0_info.size_y = MIN(scl0->trim0_info.size_y,
			scl0->src.h);
		scl0->trim0_info.size_y = MAX(scl0->trim0_info.size_y,
			scl0->dst.h);
		scl0->trim0_info.start_x = abs(scl0->src.w -
			scl0->trim0_info.size_x) / 2;
		scl0->trim0_info.start_x =
			ISP_ALIGNTO(scl0->trim0_info.start_x);
		scl0->trim0_info.start_y = abs(scl0->src.h -
			scl0->trim0_info.size_y) / 2;
		scl0->trim0_info.start_y =
			ISP_ALIGNTO(scl0->trim0_info.start_y);
		path->trim0_info.size_x = path->in_rect.w;
		path->trim0_info.size_y = MIN(path->src.h,
			path->in_rect.h);
		path->trim0_info.start_x = abs(path->src.w -
			path->trim0_info.size_x) / 2;
		path->trim0_info.start_x =
			ISP_ALIGNTO(path->trim0_info.start_x);
		path->trim0_info.start_y = abs(path->src.h -
			path->trim0_info.size_y) / 2;
		path->trim0_info.start_y =
			ISP_ALIGNTO(path->trim0_info.start_y);
	}

#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
	if (path->path_mode == ISP_PRE_VID_ONLINE_CPP
	 || path->path_mode == ISP_PRE_ONLINE_CPP) {
		path->dst.w = path->trim0_info.size_x;
		path->dst.h = path->trim0_info.size_y;
	} else
#endif
	{
		path->dst.w = path->out_size.w;
		path->dst.h = path->out_size.h;
	}

	path->trim1_info.start_x = 0;
	path->trim1_info.start_y = 0;
	path->trim1_info.size_x = path->dst.w;
	path->trim1_info.size_y = path->dst.h;
	path->path_sel = 0;
	scl0->uv_sync_v = 1;
	scl0->valid = 1;
	return rtn;
}

#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
static int isp_pre_vid_cowork_cpp(enum isp_id idx,
	struct isp_path_desc *path,
	struct isp_path_desc *scl0)
{
	int rtn = 0;
	struct camera_size *in_size, *out_size;
	struct camera_rect *in_rect;
	unsigned int x, y, w, h, s_w, s_h, d_w, d_h;

	in_size = &path->in_size;
	in_rect = &path->in_rect;
	out_size = &path->out_size;

	x = in_rect->x;
	y = in_rect->y;
	w = in_rect->w;
	h = in_rect->h;

	s_w = in_size->w;
	s_h = in_size->h;
	d_w = out_size->w;
	d_h = out_size->h;

	w &= 0xfffffff8;
	h &= 0xfffffffc;

	x = (s_w - w)/2;
	y = (s_h - h)/2;

	x &= 0xfffffffe;
	y &= 0xfffffffe;

	in_rect->x = x;
	in_rect->y = y;
	in_rect->w = w;
	in_rect->h = h;

	rtn = isp_path_online(idx, path, scl0);
	if (rtn) {
		pr_err("Preview online config error\n");
		return rtn;
	}

	return rtn;
}
#endif
static enum isp_store_format isp_store_format(enum dcam_fmt in_format)
{
	enum isp_store_format format = ISP_STORE_FORMAT_MAX;

	switch (in_format) {
	case DCAM_YUV422:
		format = ISP_STORE_YUV422_2FRAME;
		break;
	case DCAM_YUV420:
		format = ISP_STORE_YVU420_2FRAME;
		break;
	case DCAM_YVU420:
		format = ISP_STORE_YUV420_2FRAME;
		break;
	case DCAM_YUV420_3FRAME:
		format = ISP_STORE_YUV420_3FRAME;
		break;
	case DCAM_RAWRGB:
		format = ISP_STORE_RAW10;
		break;
	case DCAM_RGB888:
		format = ISP_STORE_FULL_RGB8;
		break;
	default:
		format = ISP_STORE_FORMAT_MAX;
		pr_info("error, format not support!");
		break;
	}
	return format;
}

static enum isp_fetchYUV_format isp_fetchyuv_format(enum dcam_fmt in_format)
{
	enum isp_fetchYUV_format format = ISP_FETCHYUV_FORMAT_MAX;

	switch (in_format) {
	case DCAM_YUV422:
		format = ISP_FETCHYUV_YUV422_2FRAME;
		break;
	case DCAM_YUV420:
		format = ISP_FETCHYUV_YVU420_2FRAME;
		break;
	case DCAM_YVU420:
		format = ISP_FETCHYUV_YUV420_2FRAME;
		break;
	default:
		format = ISP_FETCHYUV_FORMAT_MAX;
		pr_info("error, format not support!");
		break;
	}
	return format;
}

static void get_fetchyuv_pitch(struct slice_pitch *pitch_ptr,
	enum isp_fetchYUV_format format, unsigned int width)
{
	switch (format) {
	case ISP_FETCHYUV_YUV422_2FRAME:
	case ISP_FETCHYUV_YVU422_2FRAME:
	case ISP_FETCHYUV_YUV420_2FRAME:
	case ISP_FETCHYUV_YVU420_2FRAME:
		pitch_ptr->chn0 = width;
		pitch_ptr->chn1 = width;
		break;
	default:
		break;
	}
}

static void get_store_pitch(struct slice_pitch *pitch_ptr,
	enum isp_store_format format, unsigned int width)
{
	switch (format) {
	case ISP_STORE_YUV422_3FRAME:
	case ISP_STORE_YUV420_3FRAME:
		pitch_ptr->chn0 = width;
		pitch_ptr->chn1 = width >> 1;
		pitch_ptr->chn2 = width >> 1;
		break;
	case ISP_STORE_YUV422_2FRAME:
	case ISP_STORE_YVU422_2FRAME:
	case ISP_STORE_YUV420_2FRAME:
	case ISP_STORE_YVU420_2FRAME:
		pitch_ptr->chn0 = width;
		pitch_ptr->chn1 = width;
		break;
	case ISP_STORE_UYVY:
		pitch_ptr->chn0 = width << 1;
		break;
	case ISP_STORE_RAW10:
		pitch_ptr->chn0 = width << 1;
		break;
	case ISP_STORE_FULL_RGB8:
		pitch_ptr->chn0 = width * 4;
		break;
	default:
		break;
	}

}

static int isp_get_store_param(struct isp_path_desc *path)
{
	struct isp_store_info *store_info = NULL;

	if (!path) {
		pr_err("input ptr is NULL!\n");
		return -EFAULT;
	}

	store_info = &path->store_info;
	store_info->bypass = 0;
	store_info->endian = path->data_endian.uv_endian;
	store_info->speed_2x = 1;
	store_info->mirror_en = 0;
	store_info->color_format = isp_store_format(path->output_format);
	store_info->max_len_sel = 0;
	store_info->shadow_clr_sel = 1;
	store_info->shadow_clr = 1;
	store_info->store_res = 1;
	store_info->rd_ctrl = 0;

	store_info->size.w = path->trim1_info.size_x;
	store_info->size.h = path->trim1_info.size_y;

	store_info->border.up_border = 0;
	store_info->border.down_border = 0;
	store_info->border.left_border = 0;
	store_info->border.right_border = 0;

	get_store_pitch((void *)&store_info->pitch,
			store_info->color_format, store_info->size.w);

	return 0;
}

static int isp_get_storecce_param(struct isp_store_cce_desc *store_cce)
{
	struct isp_store_info *store_info = NULL;
	struct isp_channel_info *ch_info = NULL;

	if (!store_cce) {
		pr_err("input ptr is NULL!\n");
		return -EFAULT;
	}

	store_info = &store_cce->store_info;
	store_info->bypass = 0;
	store_info->endian = store_cce->data_endian.uv_endian;
	store_info->speed_2x = 0;
	store_info->mirror_en = 0;
	store_info->color_format = isp_store_format(store_cce->output_format);
	store_info->max_len_sel = 0;
	store_info->shadow_clr_sel = 0;
	store_info->shadow_clr = 1;
	store_info->store_res = 1;
	store_info->rd_ctrl = 0;

	store_info->size.w = store_cce->dst.w;
	store_info->size.h = store_cce->dst.h;
	store_info->border.up_border = store_cce->border.up_border;
	store_info->border.down_border = store_cce->border.down_border;
	store_info->border.left_border = store_cce->border.left_border;
	store_info->border.right_border = store_cce->border.right_border;

	get_store_pitch((void *)&store_info->pitch,
			store_info->color_format, store_info->size.w);

	ch_info = &store_cce->ch_info;
	ch_info->srore_cce_en = 1;
	ch_info->store_cce_sdw_done_en = 1;
	ch_info->store_cce_all_done_ctrl = 0;
	if (store_info->color_format == ISP_STORE_RAW10)
		ch_info->store_cce_path_sel = 2;
	else if (store_info->color_format >= ISP_STORE_YUV420_2FRAME)
		ch_info->store_cce_path_sel = 1;
	else
		ch_info->store_cce_path_sel = 0;

	ch_info->fetchYUV_bypass = 0;
	ch_info->fetchYUV_format =
		isp_fetchyuv_format(store_cce->output_format);
	get_fetchyuv_pitch((void *)&ch_info->fetchYUV_pitch,
			ch_info->fetchYUV_format, store_info->size.w);

	return 0;
}

static int isp_path_store_cfg(struct isp_path_desc *scl0,
				struct isp_path_desc *pre,
				struct isp_path_desc *vid,
				struct isp_path_desc *cap,
				struct isp_store_cce_desc *store_cce)
{
	int rtn = 0;

	if (!scl0 || !pre || !vid || !cap || !store_cce) {
		pr_err("input ptr is NULL!\n");
		return -EFAULT;
	}

	if (pre->valid) {
		rtn = isp_get_store_param(pre);
		if (rtn) {
			pr_err("Get pre store param error\n");
			return rtn;
		}
	}

	if (vid->valid) {
		rtn = isp_get_store_param(vid);
		if (rtn) {
			pr_err("Get vid store param error\n");
			return rtn;
		}
	}

	if (cap->valid) {
		rtn = isp_get_store_param(cap);
		cap->store_info.shadow_clr_sel = 1;
		if (rtn) {
			pr_err("Get cap store param error\n");
			return rtn;
		}
	}

	if (store_cce->valid) {
		rtn = isp_get_storecce_param(store_cce);
		if (rtn) {
			pr_err("Get store cce param error\n");
			return rtn;
		}
	}

	return 0;
}

int isp_start_pre_proc(struct isp_module *module,
		struct isp_path_desc *scl0,
		struct isp_path_desc *pre,
		struct isp_path_desc *vid,
		struct isp_path_desc *cap,
		struct isp_store_cce_desc *store_cce)
{
	int rtn = 0;
	enum isp_id idx = ISP_ID_0;

	if (!module || !scl0 || !pre || !vid || !cap || !store_cce) {
		pr_err("input ptr is NULL!\n");
		return -EFAULT;
	}

	idx = module->idx;

	if (pre->valid) {
		if (pre->path_mode == ISP_PRE_ONLINE) {
			rtn = isp_path_online(idx, pre, scl0);
			if (rtn) {
				pr_err("Preview online config error\n");
				return rtn;
			}
		} else if (pre->path_mode == ISP_PRE_OFFLINE) {
			rtn = isp_path_offline(pre, scl0, store_cce, module);
			if (rtn) {
				pr_err("Preview offline config error\n");
				return rtn;
			}
		}
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
		else if (pre->path_mode == ISP_PRE_ONLINE_CPP) {
			/* preview will use online cpp path configuration */
			rtn = isp_pre_vid_cowork_cpp(idx, pre, scl0);
			if (rtn) {
				pr_err("Preview online config error\n");
				return rtn;
			}
			module->isp_cpp_dev.is_valid = 1;
		}  else if (pre->path_mode == ISP_PRE_VID_ONLINE_CPP) {
		    /* we will take care while configure video*/
		}
#endif
		else {
			pr_err("Isp preview path mode is error\n");
			return -EINVAL;
		}
	}

	if (vid->valid) {
		if (vid->path_mode == ISP_VID_ONLINE) {
			rtn = isp_path_online(idx, vid, scl0);
			if (rtn) {
				pr_err("Preview online config error\n");
				return rtn;
			}

			if (pre->valid && pre->path_mode == ISP_PRE_ONLINE) {
				rtn = isp_path_online_cowork(idx, pre,
							     vid, scl0);
				if (rtn) {
					pr_err("Onlie cowork config error\n");
					return rtn;
				}
			}
		} else if (vid->path_mode == ISP_VID_OFFLINE) {
			rtn = isp_path_offline(vid, scl0, store_cce, module);
			if (rtn) {
				pr_err("Video offline config error\n");
				return rtn;
			}
		}
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
		else if (vid->path_mode == ISP_PRE_VID_ONLINE_CPP &&
				pre->path_mode == ISP_PRE_VID_ONLINE_CPP) {

			rtn = isp_pre_vid_cowork_cpp(idx, pre, scl0);
			if (rtn) {
				pr_err("Video offline config error\n");
				return rtn;
			}
			module->isp_cpp_dev.is_valid = 1;
		}
#endif
		else {
			pr_err("Isp video path mode is error\n");
			return -EINVAL;
		}
	}

	if (cap->valid) {
		rtn = isp_path_offline(cap, scl0, store_cce, module);
		if (rtn) {
			pr_err("Cap offline config error\n");
			return rtn;
		}

		if (pre->valid && vid->valid &&
			pre->path_mode == ISP_PRE_OFFLINE &&
			vid->path_mode == ISP_VID_OFFLINE) {
			rtn = isp_path_offline_cowork_cap(module,
					scl0, pre, vid, cap, store_cce);
			if (rtn) {
				pr_err("Offline cowork config error\n");
				return rtn;
			}
		}
	} else if (pre->valid && vid->valid &&
			pre->path_mode == ISP_PRE_OFFLINE &&
			vid->path_mode == ISP_VID_OFFLINE) {
		rtn = isp_path_offline_cowork(pre, vid, store_cce);
		if (rtn) {
			pr_err("Preview&Video offline cowork config error\n");
			return rtn;
		}
	}

	rtn = isp_path_store_cfg(scl0, pre, vid, cap, store_cce);

	if (rtn) {
		pr_err("Get store param error\n");
		return rtn;
	}

	return rtn;
}

static int isp_set_store(enum isp_id idx, void *input_info, unsigned int addr)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_store_info *store_info = (struct isp_store_info *)input_info;

	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM,
			BIT_0, store_info->bypass);
	if (store_info->bypass)
		return 0;

	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM,
		BIT_1, (store_info->max_len_sel << 1));

	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM,
		BIT_2, (store_info->speed_2x << 2));

	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM,
		BIT_3, (store_info->mirror_en << 3));

	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM,
		0xF0, (store_info->color_format << 4));

	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM,
		0x300, (store_info->endian << 8));

	val = ((store_info->size.h & 0xFFFF) << 16) |
		   (store_info->size.w & 0xFFFF);
	ISP_REG_WR(idx, addr + ISP_STORE_SLICE_SIZE, val);

	val = ((store_info->border.right_border & 0xFF) << 24) |
		  ((store_info->border.left_border & 0xFF) << 16) |
		  ((store_info->border.down_border & 0xFF) << 8) |
		   (store_info->border.up_border & 0xFF);
	ISP_REG_WR(idx, addr+ISP_STORE_BORDER1, val);
	ISP_REG_WR(idx, addr+ISP_STORE_Y_PITCH, store_info->pitch.chn0);
	ISP_REG_WR(idx, addr+ISP_STORE_U_PITCH, store_info->pitch.chn1);
	ISP_REG_WR(idx, addr+ISP_STORE_V_PITCH, store_info->pitch.chn2);

	pr_debug("set_store size %d %d border %d %d %d %d\n",
		store_info->size.w, store_info->size.h,
		store_info->border.left_border,
		store_info->border.up_border,
		store_info->border.right_border,
		store_info->border.down_border);

	ISP_REG_MWR(idx, addr + ISP_STORE_READ_CONTROL,
		0x3, store_info->rd_ctrl);
	ISP_REG_MWR(idx, addr + ISP_STORE_READ_CONTROL,
		0xFFFFFFFC, store_info->store_res << 2);

	ISP_REG_MWR(idx, addr + ISP_STORE_SHADOW_CLR_SEL,
		BIT_1, store_info->shadow_clr_sel << 1);
	ISP_REG_MWR(idx, addr + ISP_STORE_SHADOW_CLR,
		BIT_0, store_info->shadow_clr);

	return ret;
}

static void cpp_print_coeff(enum isp_id idx,
			    unsigned long h_coeff_addr,
			    unsigned long v_chroma_coeff_addr,
			    unsigned long v_coeff_addr)
{
#ifdef SCALE_DRV_DEBUG
	int i = 0;
	int j = 0;
	unsigned long addr = 0;

	pr_info("SCAL: h_coeff_addr\n");
	for (i = 0; i < 16; i++) {
		pr_info("0x%lx: ", h_coeff_addr + 4 * (4 * i));
		for (j = 0; j < 4; j++)
			pr_info("0x%x ",
			ISP_REG_RD(idx, h_coeff_addr + 4 * (4 * i + j)));
		pr_info("\n");
	}

	pr_info("SCAL: v_chroma_coeff_addr\n");
	for (i = 0; i < 32; i++) {
		pr_info("0x%lx: ", v_chroma_coeff_addr + 4 * (4 * i));
		for (j = 0; j < 4; j++)
			pr_info("0x%x ",
				ISP_REG_RD(idx,
				       v_chroma_coeff_addr + 4 * (4 * i + j)));
		pr_info("\n");
	}

	pr_info("SCAL: v_coeff_addr\n");
	for (addr = v_coeff_addr; addr <= (v_coeff_addr + 0x6FC); addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_REG_RD(idx, addr), ISP_REG_RD(idx, addr + 4),
			ISP_REG_RD(idx, addr + 8), ISP_REG_RD(idx, addr + 12));
	}
#endif
}

int isp_set_sc_coeff_info(enum isp_id idx, unsigned int addr,
	unsigned int *coeff_buf)
{
	int i = 0;
	int rtn = 0;
	unsigned int h_coeff_addr = 0;
	unsigned int v_coeff_addr = 0;
	unsigned int v_chroma_coeff_addr = 0;
	unsigned int *h_coeff = NULL;
	unsigned int *v_coeff = NULL;
	unsigned int *v_chroma_coeff = NULL;
	unsigned int reg_val = 0;

	if (coeff_buf == NULL) {
		pr_info("zero pointer!");
		return -1;
	}

	h_coeff = coeff_buf;
	v_coeff = coeff_buf + (ISP_SC_COEFF_COEF_SIZE / 4);
	v_chroma_coeff = v_coeff + (ISP_SC_COEFF_COEF_SIZE / 4);

	h_coeff_addr = addr + ISP_SCALER_LUMA_HCOEFF;
	v_coeff_addr = addr + ISP_SCALER_LUMA_VCOEFF;
	v_chroma_coeff_addr = addr + ISP_SCALER_CHROMA_VCOEFF;

	if (addr == ISP_SCALER_CAP_BASE)
		ISP_REG_MWR(idx, addr+ISP_SCALER_CFG, BIT_30, 1 << 30);
	else
		reg_val = (ISP_REG_RD(idx, addr +
			ISP_SCALER_CFG) & BIT_30) >> 30;

	for (i = 0; i < ISP_SC_COEFF_H_NUM; i++) {
		ISP_REG_WR(idx, h_coeff_addr, *h_coeff);
		h_coeff_addr += 4;
		h_coeff++;
	}

	for (i = 0; i < ISP_SC_COEFF_V_NUM; i++) {
		ISP_REG_WR(idx, v_coeff_addr, *v_coeff);
		v_coeff_addr += 4;
		v_coeff++;
	}

	for (i = 0; i < ISP_SC_COEFF_V_CHROMA_NUM; i++) {
		ISP_REG_WR(idx, v_chroma_coeff_addr, *v_chroma_coeff);
		v_chroma_coeff_addr += 4;
		v_chroma_coeff++;
	}

	h_coeff_addr = addr + ISP_SCALER_LUMA_HCOEFF;
	v_coeff_addr = addr + ISP_SCALER_LUMA_VCOEFF;
	v_chroma_coeff_addr = addr + ISP_SCALER_CHROMA_VCOEFF;
	cpp_print_coeff(idx, h_coeff_addr, v_chroma_coeff_addr, v_coeff_addr);
	if (addr == ISP_SCALER_CAP_BASE)
		ISP_REG_MWR(idx, addr+ISP_SCALER_CFG, BIT_30, 0 << 30);
	else
		ISP_REG_MWR(idx, addr + ISP_SCALER_CFG,
			    BIT_30, (~reg_val) << 30);
	pr_debug("isp_set_sc_coeff_info\n");

	return rtn;
}


static void isp_set_shrink_info(void *input_info,
		      enum isp_id idx, unsigned int addr_base)
{
	unsigned long addr = 0;
	unsigned int reg_val = 0;
	struct isp_regular_info *regular_info = NULL;

	if (!input_info) {
		pr_err("Input info ptr is NULL\n");
		return;
	}

	regular_info = (struct isp_regular_info *)input_info;
	pr_debug("regular_mode %d\n", regular_info->regular_mode);
	addr = ISP_SCALER_CFG + addr_base;
	ISP_REG_MWR(idx, addr, (BIT_25 | BIT_26),
		    regular_info->regular_mode << 25);

	if (regular_info->regular_mode == DCAM_REGULAR_SHRINK) {
		regular_info->shrink_y_up_th = SHRINK_Y_UP_TH;
		regular_info->shrink_y_dn_th = SHRINK_Y_DN_TH;
		regular_info->shrink_uv_up_th = SHRINK_UV_UP_TH;
		regular_info->shrink_uv_dn_th = SHRINK_UV_DN_TH;
		addr = ISP_SCALER_SHRINK_CFG + addr_base;
		reg_val = ((regular_info->shrink_uv_dn_th & 0xFF) << 24) |
			((regular_info->shrink_uv_up_th & 0xFF) << 16);
		reg_val |= ((regular_info->shrink_y_dn_th  & 0xFF) << 8) |
			((regular_info->shrink_y_up_th & 0xFF));
		ISP_REG_WR(idx, addr, reg_val);

		regular_info->shrink_y_offset = SHRINK_Y_OFFSET;
		regular_info->shrink_y_range = SHRINK_Y_RANGE;
		regular_info->shrink_c_offset = SHRINK_C_OFFSET;
		regular_info->shrink_c_range = SHRINK_C_RANGE;
		addr = ISP_SCALER_REGULAR_CFG + addr_base;
		reg_val = ((regular_info->shrink_c_range & 0xF) << 24) |
			((regular_info->shrink_c_offset & 0x1F) << 16);
		reg_val |= ((regular_info->shrink_y_range & 0xF) << 8) |
			(regular_info->shrink_y_offset & 0x1F);
		ISP_REG_WR(idx, addr, reg_val);
	} else if (regular_info->regular_mode == DCAM_REGULAR_CUT) {
		addr = ISP_SCALER_SHRINK_CFG + addr_base;
		reg_val = ((regular_info->shrink_uv_dn_th & 0xFF) << 24) |
			((regular_info->shrink_uv_up_th & 0xFF) << 16);
		reg_val |= ((regular_info->shrink_y_dn_th  & 0xFF) << 8) |
			((regular_info->shrink_y_up_th & 0xFF));
		ISP_REG_WR(idx, addr, reg_val);
	} else if (regular_info->regular_mode == DCAM_REGULAR_EFFECT) {
		addr = ISP_SCALER_EFFECT_CFG + addr_base;
		reg_val = ((regular_info->effect_v_th & 0xFF) << 16) |
				((regular_info->effect_u_th & 0xFF) << 8);
		reg_val |= (regular_info->effect_y_th & 0xFF);
		ISP_REG_WR(idx, addr, reg_val);
	} else
		pr_debug("regular_mode %d\n", regular_info->regular_mode);
}

static void isp_set_scaler_info(void *input_info,
	enum isp_id idx, unsigned int addr_base)
{
	unsigned int reg_val;

	struct isp_scaler_info *scalerInfo =
		 (struct isp_scaler_info *)input_info;

	ISP_REG_MWR(idx, addr_base+ISP_SCALER_CFG, BIT_20,
			scalerInfo->scaler_bypass << 20);
	ISP_REG_MWR(idx, addr_base+ISP_SCALER_CFG, 0xF0000,
			scalerInfo->scaler_y_ver_tap << 16);
	ISP_REG_MWR(idx, addr_base+ISP_SCALER_CFG, 0xF800,
			scalerInfo->scaler_uv_ver_tap << 11);

	reg_val = ((scalerInfo->scaler_ip_int & 0xF) << 16) |
			(scalerInfo->scaler_ip_rmd & 0x3FFF);
	ISP_REG_MWR(idx, addr_base+ISP_SCALER_IP, 0xF3FFF, reg_val);
	reg_val = ((scalerInfo->scaler_cip_int & 0xF) << 16) |
			(scalerInfo->scaler_cip_rmd & 0x3FFF);
	ISP_REG_MWR(idx, addr_base+ISP_SCALER_CIP, 0xF3FFF, reg_val);
	reg_val = ((scalerInfo->scaler_factor_in & 0x3FFF) << 16) |
			(scalerInfo->scaler_factor_out & 0x3FFF);
	ISP_REG_MWR(idx, addr_base+ISP_SCALER_FACTOR, 0x3FFF3FFF, reg_val);

	reg_val = ((scalerInfo->scaler_ver_ip_int & 0xF) << 16) |
			 (scalerInfo->scaler_ver_ip_rmd & 0x3FFF);
	ISP_REG_MWR(idx, addr_base+ISP_SCALER_VER_IP, 0xF3FFF, reg_val);
	reg_val = ((scalerInfo->scaler_ver_cip_int & 0xF) << 16) |
			 (scalerInfo->scaler_ver_cip_rmd & 0x3FFF);
	ISP_REG_MWR(idx, addr_base+ISP_SCALER_VER_CIP, 0xF3FFF, reg_val);
	reg_val = ((scalerInfo->scaler_ver_factor_in & 0x3FFF) << 16) |
			 (scalerInfo->scaler_ver_factor_out & 0x3FFF);
	ISP_REG_MWR(idx, addr_base+ISP_SCALER_VER_FACTOR, 0x3FFF3FFF, reg_val);

	pr_debug("set_scale_info in %d %d out %d %d\n",
		scalerInfo->scaler_factor_in,
		scalerInfo->scaler_ver_factor_in,
		scalerInfo->scaler_factor_out,
		scalerInfo->scaler_ver_factor_out);

	isp_set_sc_coeff_info(idx, addr_base, scalerInfo->coeff_buf);
}

static void isp_set_deci_info(void *input_info, enum isp_id idx,
			   unsigned int addr_base)
{
	struct isp_deci_info *deciInfo = (struct isp_deci_info *)input_info;

	ISP_REG_MWR(idx, addr_base+ISP_SCALER_CFG, BIT_2,
		deciInfo->deci_x_eb << 2);
	ISP_REG_MWR(idx, addr_base+ISP_SCALER_CFG, (BIT_0 | BIT_1),
		deciInfo->deci_x);
	ISP_REG_MWR(idx, addr_base+ISP_SCALER_CFG, BIT_5,
		deciInfo->deci_y_eb << 5);
	ISP_REG_MWR(idx, addr_base+ISP_SCALER_CFG, (BIT_3 | BIT_4),
		deciInfo->deci_y << 3);
}

static int isp_set_storecce(enum isp_id idx, struct isp_store_info *store_info,
	unsigned int addr)
{
	int ret = 0;
	unsigned int val = 0;

	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM,
			BIT_0, store_info->bypass);
	if (store_info->bypass)
		return 0;

	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM,
		BIT_1, (store_info->max_len_sel << 1));

	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM,
		BIT_2, (store_info->speed_2x << 2));

	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM,
		BIT_3, (store_info->mirror_en << 3));

	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM,
		0xF0, (store_info->color_format << 4));

	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM,
		0x300, (store_info->endian << 8));

	val = ((store_info->size.h & 0xFFFF) << 16) |
		   (store_info->size.w & 0xFFFF);
	ISP_REG_WR(idx, addr + ISP_STORE_SLICE_SIZE, val);

	val = ((store_info->border.right_border & 0x1FFF) << 16) |
		((store_info->border.left_border & 0x1FFF));
	ISP_REG_WR(idx, addr+ISP_STORE_BORDER1, val);
	ISP_REG_WR(idx, addr+ISP_STORE_Y_PITCH, store_info->pitch.chn0);
	ISP_REG_WR(idx, addr+ISP_STORE_U_PITCH, store_info->pitch.chn1);
	ISP_REG_WR(idx, addr+ISP_STORE_V_PITCH, store_info->pitch.chn2);

	ISP_REG_MWR(idx, addr + ISP_STORE_READ_CONTROL,
		0x3, store_info->rd_ctrl);
	ISP_REG_MWR(idx, addr + ISP_STORE_READ_CONTROL,
		0xFFFFFFFC, store_info->store_res << 2);

	val = ((store_info->border.down_border & 0x1FFF) << 16) |
		((store_info->border.up_border & 0x1FFF));
	ISP_REG_WR(idx, addr+ISP_STORE_BORDER2, val);

	ISP_REG_MWR(idx, addr + ISP_STORE_SHADOW_CLR_SEL,
		BIT_1, store_info->shadow_clr_sel << 1);
	ISP_REG_MWR(idx, addr + ISP_STORE_SHADOW_CLR,
		BIT_0, store_info->shadow_clr);

	return ret;
}

static void isp_set_channel_info(enum isp_id idx,
	struct isp_channel_info *ch_info)
{
	ISP_REG_MWR(idx, ISP_FETCH2_PARAM, BIT_0, ch_info->fetchYUV_bypass);
	ISP_REG_MWR(idx, ISP_FETCH2_PARAM, 0xF0,
		(ch_info->fetchYUV_format << 4));
	ISP_REG_WR(idx, ISP_FETCH2_SLICE_Y_PITCH,
		ch_info->fetchYUV_pitch.chn0);
	ISP_REG_WR(idx, ISP_FETCH2_SLICE_U_PITCH,
		ch_info->fetchYUV_pitch.chn1);
	ISP_REG_WR(idx, ISP_FETCH2_SLICE_V_PITCH,
		ch_info->fetchYUV_pitch.chn2);
	ISP_REG_MWR(idx, ISP_COMMON_YUV_PATH_SEL_CH0, BIT_2,
		ch_info->srore_cce_en << 2);
	ISP_REG_MWR(idx, ISP_COMMON_YUV_PATH_SEL_CH0, (BIT_3 | BIT_4),
		ch_info->store_cce_path_sel << 3);
	ISP_REG_MWR(idx, ISP_INT_DONE_CTRL, BIT_31,
		ch_info->store_cce_sdw_done_en << 31);
	ISP_REG_MWR(idx, ISP_INT_DONE_CTRL, BIT_2,
		ch_info->store_cce_all_done_ctrl << 2);
}

int isp_set_storecce_info(enum isp_id idx, void *input_info, unsigned int addr)
{
	int ret = 0;
	struct isp_store_cce_desc *store_cce = NULL;
	struct isp_store_info *store_info = NULL;
	struct isp_channel_info *ch_info = NULL;

	store_cce = (struct isp_store_cce_desc *)input_info;
	store_info = &store_cce->store_info;
	ch_info = &store_cce->ch_info;

	isp_set_storecce(idx, store_info, addr);
	isp_set_channel_info(idx, ch_info);

	return ret;
}

void isp_path_set_scl(enum isp_id idx, struct isp_path_desc *path,
	unsigned int addr)
{
	unsigned int reg_val = 0;

	if (!path) {
		pr_err("Input path ptr is NULL\n");
		return;
	}

	/*CFG path_eb*/
	ISP_REG_MWR(idx, addr+ISP_SCALER_CFG, BIT_31, 1 << 31);
	if (addr == ISP_SCALER_CAP_BASE)
		ISP_REG_MWR(idx, addr+ISP_SCALER_CFG, BIT_29, 1 << 29);
	else
		ISP_REG_MWR(idx, addr+ISP_SCALER_CFG, BIT_29, 0 << 29);
	ISP_REG_MWR(idx, addr+ISP_SCALER_CFG, BIT_9, ~path->valid << 9);
	ISP_REG_MWR(idx, addr+ISP_SCALER_CFG, BIT_10, path->uv_sync_v << 10);
	/*CFG frame deci*/
	if (path->frm_deci >= 0 && path->frm_deci <= 3) {
		ISP_REG_MWR(idx, addr+ISP_SCALER_CFG, (BIT_23 | BIT_24),
			path->frm_deci << 23);
	} else {
		pr_info("invalid frame_deci %d\n", path->frm_deci);
	}
	/*CFG output format*/
	if (path->output_format == DCAM_YUV422)
		path->odata_mode = 0x00;
	else if (path->output_format == DCAM_YUV420)
		path->odata_mode = 0x01;
	else if (path->output_format == DCAM_YUV420_3FRAME)
		path->odata_mode = 0x03;
	else
		pr_info("invalid output format %d\n", path->output_format);

	ISP_REG_MWR(idx, addr+ISP_SCALER_CFG, BIT_6 | BIT_7,
			path->odata_mode << 6);

	/*CFG deci */
	isp_set_deci_info((void *)&path->deci_info, idx, addr);

	/*src size*/
	reg_val = ((path->src.h & 0x3FFF) << 16) | (path->src.w & 0x3FFF);
	ISP_REG_WR(idx, addr+ISP_SCALER_SRC_SIZE, reg_val);

	/* trim0 */
	reg_val = ((path->trim0_info.start_y & 0x3FFF) << 16) |
			(path->trim0_info.start_x & 0x3FFF);
	ISP_REG_WR(idx, addr+ISP_SCALER_TRIM0_START, reg_val);
	reg_val = ((path->trim0_info.size_y & 0x3FFF) << 16) |
			(path->trim0_info.size_x & 0x3FFF);
	ISP_REG_WR(idx, addr+ISP_SCALER_TRIM0_SIZE, reg_val);

	/* trim1 */
	reg_val = ((path->trim1_info.start_y & 0x3FFF) << 16) |
			(path->trim1_info.start_x & 0x3FFF);
	ISP_REG_WR(idx, addr+ISP_SCALER_TRIM1_START, reg_val);
	reg_val = ((path->trim1_info.size_y & 0x3FFF) << 16) |
			(path->trim1_info.size_x & 0x3FFF);
	ISP_REG_WR(idx, addr+ISP_SCALER_TRIM1_SIZE, reg_val);

	/* des size */
	reg_val = ((path->dst.h & 0x3FFF) << 16) | (path->dst.w & 0x3FFF);
	ISP_REG_WR(idx, addr+ISP_SCALER_DES_SIZE, reg_val);

	pr_debug("scl base addr %x\n", addr);
	pr_debug("idx %d set_scl src %d %d dst %d %d trim0 %d %d %d %d trim1 %d %d %d %d\n",
		idx,
		path->src.w, path->src.h,
		path->dst.w, path->dst.h,
		path->trim0_info.start_x,
		path->trim0_info.start_y,
		path->trim0_info.size_x,
		path->trim0_info.size_y,
		path->trim1_info.start_x,
		path->trim1_info.start_y,
		path->trim1_info.size_x,
		path->trim1_info.size_y);
	/* scaler info*/

	isp_set_scaler_info((void *)&path->scaler_info, idx, addr);

	/* scaler_vid shrink */
	if (addr == ISP_SCALER_VID_BASE)
		isp_set_shrink_info((void *)&path->regular_info, idx, addr);
}

void isp_path_set(struct isp_module *module,
	struct isp_path_desc *path, enum isp_path_index path_index)
{
	unsigned int scl_addr = 0, store_addr = 0;
	enum isp_id idx = 0;

	if (!module || !path) {
		pr_err("Input module ptr is NULL\n");
		return;
	}

	idx = module->idx;
	if (ISP_PATH_IDX_PRE & path_index) {
		scl_addr = ISP_SCALER_PRE_BASE;
		store_addr = ISP_STORE1_BASE;
	} else if (ISP_PATH_IDX_VID & path_index) {
		scl_addr = ISP_SCALER_VID_BASE;
		store_addr = ISP_STORE2_BASE;
	} else if (ISP_PATH_IDX_CAP & path_index) {
		scl_addr = ISP_SCALER_CAP_BASE;
		store_addr = ISP_STORE3_BASE;
	} else {
		pr_err("Not valid path index\n");
		return;
	}

	pr_debug("isp_path_set index %d\n", path_index);
	isp_path_set_scl(idx, path, scl_addr);
	isp_set_store(idx, (void *)&path->store_info, store_addr);
}

int isp_get_scl_index(unsigned int channel_id)
{
	enum isp_scl_id  path_index;

	switch (channel_id) {
	case ISP_PATH_IDX_0:
		path_index = ISP_SCL_0;
		break;
	case ISP_PATH_IDX_PRE:
		path_index = ISP_SCL_PRE;
		break;
	case ISP_PATH_IDX_VID:
		path_index = ISP_SCL_VID;
		break;
	case ISP_PATH_IDX_CAP:
		path_index = ISP_SCL_CAP;
		break;
	default:
		path_index = ISP_SCL_MAX;
		pr_info("failed to get path index, channel %d\n", channel_id);
	}
	return path_index;
}




int isp_path_set_next_frm(struct isp_module *module,
	enum isp_path_index path_index, struct slice_addr *addr)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct camera_frame *reserved_frame = NULL;
	struct camera_frame frame;
	struct isp_path_desc *path = NULL;
	struct isp_frm_queue *p_heap = NULL;
	struct isp_buf_queue *p_buf_queue = NULL;
	unsigned long yuv_reg[3] = {0};
	unsigned int yuv_addr[3] = {0};
	int use_reserve_frame = 0;
	enum isp_id idx = 0;
	int is_offline = 0;

	if (module == NULL) {
		pr_err("zero pointer\n");
		return -EFAULT;
	}
	memset((void *)&frame, 0, sizeof(frame));

	idx = module->idx;
	if (path_index == ISP_PATH_IDX_PRE) {
		reserved_frame = &module->path_reserved_frame[ISP_SCL_PRE];
		path = &module->isp_path[ISP_SCL_PRE];
		yuv_reg[0] = ISP_STORE_PREVIEW_Y_ADDR;
		yuv_reg[1] = ISP_STORE_PREVIEW_U_ADDR;
		yuv_reg[2] = ISP_STORE_PREVIEW_V_ADDR;
		p_heap = &path->frame_queue;
		p_buf_queue = &path->buf_queue;
		if (path->path_mode == ISP_PRE_OFFLINE)
			is_offline = 1;
	} else if (path_index == ISP_PATH_IDX_VID) {
		reserved_frame = &module->path_reserved_frame[ISP_SCL_VID];
		path = &module->isp_path[ISP_SCL_VID];
		yuv_reg[0] = ISP_STORE_VIDEO_Y_ADDR;
		yuv_reg[1] = ISP_STORE_VIDEO_U_ADDR;
		yuv_reg[2] = ISP_STORE_VIDEO_V_ADDR;
		p_heap = &path->frame_queue;
		p_buf_queue = &path->buf_queue;
		if (path->path_mode == ISP_VID_OFFLINE)
			is_offline = 1;
	} else if (path_index == ISP_PATH_IDX_CAP) {
		reserved_frame = &module->path_reserved_frame[ISP_SCL_CAP];
		path = &module->isp_path[ISP_SCL_CAP];
		yuv_reg[0] = ISP_STORE_CAPTURE_Y_ADDR;
		yuv_reg[1] = ISP_STORE_CAPTURE_U_ADDR;
		yuv_reg[2] = ISP_STORE_CAPTURE_V_ADDR;
		p_heap = &path->frame_queue;
		p_buf_queue = &path->buf_queue;
		if (path->path_mode == ISP_CAP_OFFLINE)
			is_offline = 1;
	}

	if (p_heap->valid_cnt >= ISP_FRM_QUEUE_LENGTH) {
		rtn = ISP_RTN_PATH_FRAME_LOCKED;
		pr_err("path id %d frame buf queue will overflow.\n",
				path_index);
		goto overflow;
	}

	path->frm_cnt++;
	if (path->frm_cnt <= path->skip_num && !is_offline) {
		use_reserve_frame = 1;
	} else if (isp_buf_queue_read(p_buf_queue, &frame) == 0 &&
	    (frame.pfinfo.mfd[0] != 0)) {
		path->output_frame_count--;
	} else {
		use_reserve_frame = 1;
	}

	if ((use_reserve_frame == 0) &&
		(frame.pfinfo.mfd[0] == reserved_frame->pfinfo.mfd[0])) {
		use_reserve_frame = 1;
		if (reserved_frame->pfinfo.iova[0] == 0) {
			rtn = pfiommu_get_addr(&reserved_frame->pfinfo);
			if (rtn) {
				pr_err("ISP%d: get reserved buffer failed!\n",
						idx);
				rtn = ISP_RTN_PATH_FRAME_LOCKED;
				goto iommu_err;
			}
		}
	}

	if (use_reserve_frame) {
		pr_debug("ISP%d: No freed frame id %d path_index %d\n",
				idx, frame.fid, path_index);
		if (reserved_frame->pfinfo.mfd[0] == 0) {
			pr_info("ISP%d: No need to cfg frame buffer", idx);
			return -1;
		}
		memcpy(&frame, reserved_frame, sizeof(struct camera_frame));
	} else {
		if (frame.pfinfo.dev == NULL)
			pr_info("ISP%d next dev NULL %p\n",
					idx, frame.pfinfo.dev);
		if (pfiommu_get_addr(&frame.pfinfo)) {
			pr_err("ISP%d: get frame iommu map address failed!\n",
					idx);
			rtn = ISP_RTN_PATH_FRAME_LOCKED;
			goto iommu_err;
		}
	}

	yuv_addr[0] = frame.pfinfo.iova[0] + frame.yaddr;
	yuv_addr[1] = frame.pfinfo.iova[0] + frame.uaddr;
	yuv_addr[2] = frame.pfinfo.iova[2];

	if (isp_frame_enqueue(p_heap, &frame) == 0)
		pr_debug("success to enq frame buf\n");
	else {
		rtn = ISP_RTN_PATH_FRAME_LOCKED;
		pr_err("fail to enq frame buf\n");
		goto queue_err;
	}

	ISP_REG_WR(idx, yuv_reg[0], yuv_addr[0]);
	if (path->output_format < DCAM_YUV400) {
		ISP_REG_WR(idx, yuv_reg[1], yuv_addr[1]);
		if (path->output_format == DCAM_YUV420_3FRAME)
			ISP_REG_WR(idx, yuv_reg[2], yuv_addr[2]);
	}

	if (addr != NULL) {
		addr->chn0 = yuv_addr[0];
		addr->chn1 = yuv_addr[1];
		addr->chn2 = yuv_addr[2];
	}

	return 0;

queue_err:
	if (use_reserve_frame)
		return -rtn;

	if (path_index == ISP_PATH_IDX_PRE) {
		pfiommu_free_addr_with_id(&frame.pfinfo, SPRD_IOMMU_EX_CH_WRITE,
			(BIT_15 | BIT_16 | BIT_17));
	} else if (path_index == ISP_PATH_IDX_VID) {
		pfiommu_free_addr_with_id(&frame.pfinfo, SPRD_IOMMU_EX_CH_WRITE,
			(BIT_18 | BIT_19 | BIT_20));
	} else if (path_index == ISP_PATH_IDX_CAP) {
		pfiommu_free_addr_with_id(&frame.pfinfo, SPRD_IOMMU_EX_CH_WRITE,
			(BIT_21 | BIT_22 | BIT_23));
	}

	memset(frame.pfinfo.iova, 0, sizeof(frame.pfinfo.iova));

iommu_err:
	if (!use_reserve_frame) {
		frame.pfinfo.offset[0] = 0;
		frame.pfinfo.offset[1] = 0;
		frame.pfinfo.offset[2] = 0;
		if (!isp_buf_queue_write(p_buf_queue, &frame))
			path->output_frame_count++;
	}

overflow:
	return -rtn;

}

int isp_storecce_set_next_frm(struct isp_module *module)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct camera_frame frame;
	struct isp_store_cce_desc *store_cce = NULL;
	unsigned long yuv_reg[3] = {0};
	unsigned int yuv_addr[3] = {0};
	struct isp_frm_queue *p_heap = NULL;
	struct isp_buf_queue *p_buf_queue = NULL;
	unsigned int output_frame_count = 0;
	enum isp_id idx = 0;
	unsigned int val = 0;

	if (!module) {
		pr_err("zero pointer\n");
		return -EFAULT;
	}
	memset((void *)&frame, 0, sizeof(frame));
	store_cce = &module->store_cce;
	idx = module->idx;
	yuv_reg[0] = ISP_STORE_CCE_SLICE_Y_ADDR;
	yuv_reg[1] = ISP_STORE_CCE_SLICE_U_ADDR;
	yuv_reg[2] = ISP_STORE_CCE_SLICE_V_ADDR;
	p_heap = &store_cce->frame_queue;
	p_buf_queue = &store_cce->tmp_buf_queue;
	output_frame_count = store_cce->output_frame_count;

	if (isp_buf_queue_read(p_buf_queue, &frame) == 0 &&
	    (frame.yaddr != 0)) {
		store_cce->output_frame_count--;
	} else {
		store_cce->read_buf_err = 1;
		pr_err("isp store cce buf read error\n");
		return -EFAULT;
	}

	yuv_addr[0] = frame.yaddr;
	yuv_addr[1] = frame.uaddr;
	yuv_addr[2] = frame.vaddr;

	pr_debug("store cce y_addr %x, uv_addr %x", yuv_addr[0], yuv_addr[1]);
	ISP_REG_WR(idx, yuv_reg[0], yuv_addr[0]);
	if (store_cce->output_format < DCAM_YUV400) {
		ISP_REG_WR(idx, yuv_reg[1], yuv_addr[1]);
		if (store_cce->output_format == DCAM_YUV420_3FRAME)
			ISP_REG_WR(idx, yuv_reg[2], yuv_addr[2]);
	}

	val = ISP_REG_RD(idx, ISP_STOREA_BASE + ISP_STORE_SLICE_SIZE);
	frame.width = val & 0xFFFF;
	frame.height = (val >> 16) & 0xFFFF;
	pr_debug("store cce w %d, h %d\n", frame.width, frame.height);
	if (isp_frame_enqueue(p_heap, &frame) == 0)
		pr_debug("success to enq frame buf\n");
	else
		rtn = ISP_RTN_PATH_FRAME_LOCKED;

	return -rtn;
}


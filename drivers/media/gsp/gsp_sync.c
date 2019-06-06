/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
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
#include <linux/export.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include "gsp_debug.h"
#include "gsp_layer.h"
#include "gsp_sync.h"
#include "sync.h"

#define GSP_FENCE_WAIT_TIMEOUT 2900/* ms */


static int gsp_sync_cmp(u32 a, u32 b)
{
	if (a == b)
		return 0;

	return ((s32) a - (s32) b) < 0 ? -1 : 1;
}

struct sync_pt *gsp_sync_pt_create(struct gsp_sync_timeline *obj, u32 value)
{
	struct gsp_sync_pt *pt;

	pt = (struct gsp_sync_pt *)
	    sync_pt_create(&obj->obj, sizeof(struct gsp_sync_pt));

	pt->value = value;

	return (struct sync_pt *)pt;
}

static struct sync_pt *gsp_sync_pt_dup(struct sync_pt *sync_pt)
{
	struct gsp_sync_pt *pt = (struct gsp_sync_pt *)sync_pt;
	struct gsp_sync_timeline *obj =
	    (struct gsp_sync_timeline *)sync_pt_parent(sync_pt);

	return (struct sync_pt *)gsp_sync_pt_create(obj, pt->value);
}

static int gsp_sync_pt_has_signaled(struct sync_pt *sync_pt)
{
	struct gsp_sync_pt *pt = (struct gsp_sync_pt *)sync_pt;
	struct gsp_sync_timeline *obj =
	    (struct gsp_sync_timeline *)sync_pt_parent(sync_pt);

	return gsp_sync_cmp(obj->value, pt->value) >= 0;
}

static int gsp_sync_pt_compare(struct sync_pt *a, struct sync_pt *b)
{
	struct gsp_sync_pt *pt_a = (struct gsp_sync_pt *)a;
	struct gsp_sync_pt *pt_b = (struct gsp_sync_pt *)b;

	return gsp_sync_cmp(pt_a->value, pt_b->value);
}

static int gsp_sync_fill_driver_data(struct sync_pt *sync_pt,
				     void *data, int size)
{
	struct gsp_sync_pt *pt = (struct gsp_sync_pt *)sync_pt;

	if (size < sizeof(pt->value))
		return -ENOMEM;

	memcpy(data, &pt->value, sizeof(pt->value));

	return sizeof(pt->value);
}

static void gsp_sync_timeline_value_str(struct sync_timeline *sync_timeline,
					char *str, int size)
{
	struct gsp_sync_timeline *timeline =
	    (struct gsp_sync_timeline *)sync_timeline;
	snprintf(str, size, "%d", timeline->value);
}

static void gsp_sync_pt_value_str(struct sync_pt *sync_pt, char *str, int size)
{
	struct gsp_sync_pt *pt = (struct gsp_sync_pt *)sync_pt;

	snprintf(str, size, "%d", pt->value);
}

static struct sync_timeline_ops gsp_sync_timeline_ops = {
	.driver_name = "gsp_sync",
	.dup = gsp_sync_pt_dup,
	.has_signaled = gsp_sync_pt_has_signaled,
	.compare = gsp_sync_pt_compare,
	.fill_driver_data = gsp_sync_fill_driver_data,
	.timeline_value_str = gsp_sync_timeline_value_str,
	.pt_value_str = gsp_sync_pt_value_str,
};

static void gsp_sync_record_inc(struct gsp_sync_timeline *tl)
{
	int v = 0;

	mutex_lock(&tl->lock);

	/* protect the record overflow */
	if (tl->record < UINT_MAX)
		tl->record++;
	else
		tl->record = 0;
	v = tl->record;
	mutex_unlock(&tl->lock);
}

static void gsp_sync_record_get(unsigned int *value,
				struct gsp_sync_timeline *tl)
{
	mutex_lock(&tl->lock);
	*value = tl->record;
	mutex_unlock(&tl->lock);
}

void gsp_sync_timeline_destroy(struct gsp_sync_timeline *obj)
{
	sync_timeline_destroy(&obj->obj);
}

struct gsp_sync_timeline *gsp_sync_timeline_create(const char *name)
{
	struct gsp_sync_timeline *obj = NULL;

	GSP_INFO("create timeline: %s\n", name);
	obj = (struct gsp_sync_timeline *)
		sync_timeline_create(&gsp_sync_timeline_ops,
				sizeof(struct gsp_sync_timeline), name);

	if (obj) {
		mutex_init(&obj->lock);
		obj->value = 0;
		obj->record = 1;
	}

	return obj;
}

void gsp_sync_timeline_inc(struct gsp_sync_timeline *obj, u32 inc)
{
	int r = -1;

	mutex_lock(&obj->lock);
	if (obj->value < UINT_MAX)
		obj->value += inc;
	else
		obj->value = 0;
	mutex_unlock(&obj->lock);
	sync_timeline_signal(&obj->obj);

	mutex_lock(&obj->lock);
	r = obj->value;
	mutex_unlock(&obj->lock);
	GSP_DEBUG("signal timeline value: %u\n", r);
}

void gsp_sync_fence_signal(struct gsp_sync_timeline *obj)
{
	if (obj == NULL)
		GSP_ERR("timeline is null\n");
	gsp_sync_timeline_inc(obj, 1);
}

static int gsp_sync_sig_fence_create(struct gsp_sync_timeline *obj,
				struct sync_fence **sig_fen,
				unsigned int record)
{
	int err;
	struct sync_pt *pt;
	struct sync_fence *fence;

	pt = gsp_sync_pt_create(obj, record);
	if (pt == NULL) {
		GSP_ERR("pt create failed\n");
		err = -ENOMEM;
		goto err;
	}

	fence = sync_fence_create("gsp", pt);

	if (fence == NULL) {
		GSP_ERR("fence create failed\n");
		sync_pt_free(pt);
		err = -ENOMEM;
		goto err;
	}

	/*store the new fence with the sig_fen pointer */
	*sig_fen = fence;

	return 0;

err:
	return err;
}

int gsp_sync_sig_fd_copy_to_user(struct sync_fence *fence,
				 int32_t __user *ufd)
{
	int fd = get_unused_fd_flags(0);

	if (fd < 0) {
		GSP_ERR("fd overflow, fd: %d\n", fd);
		return fd;
	}

	if (put_user(fd, ufd)) {
		GSP_ERR("signal fence fd copy to user failed\n");
		sync_fence_put(fence);
		goto err;
	}
	sync_fence_install(fence, fd);

	GSP_DEBUG("copy signal fd: %d to ufd: %p success\n", fd, ufd);
	return 0;

err:
	put_unused_fd(fd);
	return -1;
}

int gsp_sync_wait_fence_collect(struct sync_fence **wait_fen_arr,
				uint32_t *count, int fd)
{
	struct sync_fence *fence = NULL;
	int ret = -1;

	if (fd < 0) {
		GSP_DEBUG("wait fd < 0: indicate no need to wait\n");
		return 0;
	}

	if (*count >= GSP_WAIT_FENCE_MAX) {
		GSP_ERR("wait fence overflow, cnt: %d\n", *count);
		return ret;
	}

	fence = sync_fence_fdget(fd);
	if (fence != NULL) {

		/* store the wait fence at the wait_fen array */
		wait_fen_arr[*count] = fence;

		/* update the count of sig_fen */
		(*count)++;
		ret = 0;
	} else
		GSP_ERR("wait fence get from fd: %d error\n", fd);

	return ret;
}

int gsp_sync_fence_process(struct gsp_layer *layer,
			   struct gsp_fence_data *data,
			   bool last)
{
	int ret = 0;
	int wait_fd = -1;
	int share_fd = -1;
	enum gsp_layer_type type = GSP_INVAL_LAYER;

	if (IS_ERR_OR_NULL(layer)
	    || IS_ERR_OR_NULL(data)) {
		GSP_ERR("layer[%d] fence process params error\n",
			gsp_layer_to_type(layer));
		return -1;
	}

	wait_fd = gsp_layer_to_wait_fd(layer);
	type = gsp_layer_to_type(layer);
	share_fd = gsp_layer_to_share_fd(layer);

	/* first collect wait fence */
	if (layer->enable == 0 || wait_fd < 0) {
		GSP_DEBUG("layer[%d] no need to collect wait fence\n",
			  gsp_layer_to_type(layer));
	} else {
		ret = gsp_sync_wait_fence_collect(data->wait_fen_arr,
						  &data->wait_cnt, wait_fd);
		if (ret < 0) {
			GSP_ERR("collect layer[%d] wait fence failed\n",
				gsp_layer_to_type(layer));
			return ret;
		}
	}


	if (GSP_DES_LAYER != type) {
		GSP_DEBUG("no need to create sig fd for img and osd layer\n");
		return ret;
	}

	ret = gsp_sync_sig_fence_create(data->tl,
					&data->sig_fen,
					data->record);
	if (ret < 0) {
		GSP_ERR("create signal fence failed\n");
		return ret;
	}

	if (last != true) {
		GSP_DEBUG("no need to copy signal fd to user\n");
		return ret;
	}

	ret = gsp_sync_sig_fd_copy_to_user(data->sig_fen, data->ufd);
	if (ret < 0) {
		GSP_ERR("copy signal fd to user failed\n");
		return ret;
	}

	gsp_sync_record_inc(data->tl);

	return ret;
}

void gsp_sync_fence_data_setup(struct gsp_fence_data *data,
			       struct gsp_sync_timeline *tl,
			       int __user *ufd)
{
	int r = -1;

	if (IS_ERR_OR_NULL(data)
	    || IS_ERR_OR_NULL(tl)) {
		GSP_ERR("sync fence data set up params error\n");
		return;
	}

	gsp_sync_record_get(&r, tl);
	GSP_DEBUG("get timeline record: %u\n", r);

	data->record = r;
	data->tl = tl;
	data->ufd = ufd;
}

void gsp_sync_fence_free(struct gsp_fence_data *data)
{
	int i = 0;

	/* free acuqire fence array */
	for (i = 0; i < data->wait_cnt; i++) {
		if (!data->wait_fen_arr[i]) {
			GSP_WARN("free null acquire fen\n");
			continue;
		}

		sync_fence_put(data->wait_fen_arr[i]);
		data->wait_fen_arr[i] = NULL;
	}
	data->wait_cnt = 0;

	/* signal release fence */
	if (data->sig_fen)
		gsp_sync_fence_signal(data->tl);
}

int gsp_sync_fence_wait(struct gsp_fence_data *data)
{
	int ret = 0;
	int i = 0;

	/* wait acuqire fence array */
	for (i = 0; i < data->wait_cnt; i++) {
		if (!data->wait_fen_arr[i]) {
			GSP_WARN("wait null acquire fen\n");
			continue;
		}

		ret = sync_fence_wait(data->wait_fen_arr[i],
				      GSP_FENCE_WAIT_TIMEOUT);
		if (ret) {
			GSP_ERR("wait %d/%d fence failed\n",
				i + 1, data->wait_cnt);
			return ret;
		}
		sync_fence_put(data->wait_fen_arr[i]);
		data->wait_fen_arr[i] = NULL;
	}
	data->wait_cnt = 0;

	return ret;
}

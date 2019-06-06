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

#include "sprd_adf_device.h"
#include "sprd_adf_hw_information.h"
#include "sprd_adf_bridge.h"

#include <asm/div64.h>


#define dev_to_adf_device(p) \
	adf_obj_to_device(container_of(p, struct adf_obj, dev))

#define to_sprd_adf_device(x) container_of(x, struct sprd_adf_device, base)

/**
 * vsync_count_show - port vsync_count_show information to userspace
 *
 * @dev: device obj
 * @attr: device attr
 * @buf:
 *
 * vsync_count_show show the device vsync count.
 */
static ssize_t vsync_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct adf_device *adf_device = dev_to_adf_device(dev);
	struct sprd_adf_device *sprd_adf_device =
			to_sprd_adf_device(adf_device);
	return scnprintf(buf, PAGE_SIZE, "%llu\n",
			 sprd_adf_device->vsync_count);
}

/**
 * flip_count_show - port flip_count_show information to userspace
 *
 * @dev: device obj
 * @attr: device attr
 * @buf:
 *
 * flip_count_show show the device flip count.
 */
static ssize_t flip_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct adf_device *adf_device = dev_to_adf_device(dev);
	struct sprd_adf_device *sprd_adf_device =
			to_sprd_adf_device(adf_device);
	return scnprintf(buf, PAGE_SIZE, "%llu\n", sprd_adf_device->flip_count);
}

/**
 * fps_show - port fps information to userspace
 *
 * @dev: device obj
 * @attr: device attr
 * @buf:
 *
 * fps_show show the device fps.
 */
static ssize_t fps_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct adf_device *adf_device = dev_to_adf_device(dev);
	struct sprd_adf_device *sprd_adf_device =
			to_sprd_adf_device(adf_device);
	return scnprintf(buf, PAGE_SIZE, "%u\n", sprd_adf_device->fps);
}

static struct device_attribute sprd_adf_device_attrs[] = {
	__ATTR_RO(vsync_count),
	__ATTR_RO(flip_count),
	__ATTR_RO(fps),
};

/**
 * sprd_adf_restruct_post_config - restruct config depend on
 * display mode
 *
 * @dev: sprd adf device obj
 * @post: adf post cmd
 *
 * sprd_adf_restruct_post_config() is called in validate.
 */
static struct sprd_restruct_config *sprd_adf_restruct_post_config(
	struct sprd_adf_device *dev,
	struct adf_post *post)
{
	size_t custom_data_size;
	struct sprd_restruct_config *config = NULL;

	if (!dev) {
		ADF_DEBUG_WARN("parameter dev is ilegal\n");
		goto err_out;
	}

	if (!post) {
		ADF_DEBUG_WARN("parameter post is ilegal\n");
		goto err_out;
	}

	if (!post->custom_data) {
		ADF_DEBUG_WARN("parameter custom_data is ilegal\n");
		goto err_out;
	}

	custom_data_size = (size_t)post->custom_data_size;
	if (custom_data_size < FLIP_CUSTOM_DATA_MIN_SIZE) {
		ADF_DEBUG_WARN("data size is ilegal data_size = %zd;\n"
				, custom_data_size);
		goto err_out;
	}

	if (dev->ops && dev->ops->restruct_post_config)
		config = dev->ops->restruct_post_config(post);
	else
		ADF_DEBUG_WARN("ops or restruct_post_config is illegal\n");

err_out:
	return config;
}

/**
 * sprd_adf_free_restruct_config - free restruct resource
 *
 * @dev: sprd adf device obj
 * @config: restruct config information
 *
 * sprd_adf_free_restruct_config() is called in state_free.
 */
static void sprd_adf_free_restruct_config(struct sprd_adf_device *dev,
	struct sprd_restruct_config *config)
{
	ADF_DEBUG_GENERAL("entern\n");
	if (!dev) {
		ADF_DEBUG_WARN("parameter dev is ilegal\n");
		goto err_out;
	}

	if (!config) {
		ADF_DEBUG_WARN("parameter config is ilegal\n");
		goto err_out;
	}

	if (dev->ops && dev->ops->free_restruct_config)
		dev->ops->free_restruct_config(dev->intfs, config);
	else
		ADF_DEBUG_WARN("ops or free_restruct_config is illegal\n");

err_out:
	return;
}

/**
 * sprd_adf_flip_plane - flip post cmd to display hw
 *
 * @dev: sprd adf device obj
 * @config: restruct config information
 *
 * sprd_adf_flip_plane() flip all interfaces' post cmd.
 */
static void sprd_adf_flip_plane(struct sprd_adf_device *dev,
		struct sprd_restruct_config *config)
{
	size_t i, j;

	if (!dev) {
		ADF_DEBUG_WARN("parameter dev is ilegal\n");
		goto err_out;
	}

	if (!config) {
		ADF_DEBUG_WARN("parameter config is ilegal\n");
		goto err_out;
	}

	if (dev->ops && dev->ops->flip) {
		dev->ops->flip(dev->intfs, config);
	} else {
		ADF_DEBUG_WARN("ops or ops->flip is illegal\n");

		ADF_DEBUG_GENERAL("flip number_hwlayer = 0x%x\n",
				  config->number_hwlayer);
		for (i = 0; i < config->number_hwlayer; i++) {
			ADF_DEBUG_GENERAL("id = 0x%x;index = %zx;\n",
					  config->hwlayers[i].hwlayer_id, i);
			ADF_DEBUG_GENERAL("flip n_planes = 0x%x\n",
					  config->hwlayers[i].n_planes);
			ADF_DEBUG_GENERAL("flip alpha = 0x%x\n",
					  config->hwlayers[i].alpha);
			ADF_DEBUG_GENERAL("flip dst_x = 0x%x\n",
					  config->hwlayers[i].dst_x);
			ADF_DEBUG_GENERAL("flip dst_y = 0x%x\n",
					  config->hwlayers[i].dst_y);
			ADF_DEBUG_GENERAL("flip dst_w = 0x%x\n",
					  config->hwlayers[i].dst_w);
			ADF_DEBUG_GENERAL("flip dst_h = 0x%x\n",
					  config->hwlayers[i].dst_h);
			ADF_DEBUG_GENERAL("flip format = 0x%x\n",
					  config->hwlayers[i].format);
			ADF_DEBUG_GENERAL("flip blending = 0x%x\n",
					  config->hwlayers[i].blending);
			ADF_DEBUG_GENERAL("flip rotation = 0x%x\n",
					  config->hwlayers[i].rotation);
			ADF_DEBUG_GENERAL("flip scale = 0x%x\n",
					  config->hwlayers[i].scale);
			ADF_DEBUG_GENERAL("flip compression = 0x%x\n",
					  config->hwlayers[i].compression);

			for (j = 0; j < config->hwlayers[i].n_planes; j++) {
				ADF_DEBUG_GENERAL("pitch=0x%x;index= %zx;\n",
					config->hwlayers[i].pitch[j], j);
				ADF_DEBUG_GENERAL("iova = 0x%x\n",
					config->hwlayers[i].iova_plane[j]);
			}
		}
	}

err_out:
	return;
}

/**
 * sprd_adf_wait_flip_done - wait flip cmd take effect
 *
 * @dev: sprd adf device obj
 *
 * sprd_adf_wait_flip_done() wait flip done signal.
 */
static void sprd_adf_wait_flip_done(struct sprd_adf_device *dev)
{
	ADF_DEBUG_GENERAL("entern\n");
	if (!dev) {
		ADF_DEBUG_WARN("parameter dev is ilegal\n");
		goto err_out;
	}

	if (dev->ops && dev->ops->wait_flip_done)
		dev->ops->wait_flip_done(dev->intfs);
	else
		ADF_DEBUG_WARN("ops or ops->wait_flip_done is illegal\n");

err_out:
	return;
}

/**
 * sprd_adf_device_custom_data - get sprd adf device private data
 *
 * @obj: adf obj
 * @data: custom data buffer
 * @size: real custom data size
 *
 * sprd_adf_device_custom_data() will get the device capability.
 */
static int sprd_adf_device_custom_data(struct adf_obj *obj, void *data,
				       size_t *size)
{
	struct sprd_adf_device_capability *custome_data;
	struct sprd_adf_device_capability *capability;

	if (!obj) {
		ADF_DEBUG_WARN("parameter obj is ilegal\n");
		goto err_out;
	}

	if (!data) {
		ADF_DEBUG_WARN("parameter data is ilegal\n");
		goto err_out;
	}

	if (!size) {
		ADF_DEBUG_WARN("parameter size is ilegal\n");
		goto err_out;
	}

	custome_data = (struct sprd_adf_device_capability *)data;
	capability = sprd_adf_get_device_capability(0);
	if (!capability) {
		ADF_DEBUG_WARN("get device_capability error\n");
		goto err_out;
	}

	custome_data->device_id = capability->device_id;
	*size = sizeof(struct sprd_adf_device_capability);

	ADF_DEBUG_GENERAL("custome_data->device_id =%d:\n",
			  custome_data->device_id);
	ADF_DEBUG_GENERAL("*size= %zd:\n", *size);
	return 0;

err_out:
	return -1;
}

/**
 * sprd_adf_device_post - flip cmd and wait flip done
 *
 * @dev: adf device obj
 * @post: flip cmd
 * @driver_data: restruct custom data
 *
 * sprd_adf_device_post() will map buf to iommu and wait flip done.
 */
static void sprd_adf_device_post(struct adf_device *dev, struct adf_post *post,
				 void *driver_data)
{
	struct sprd_restruct_config *config;
	struct sprd_adf_device *sprd_adf_dev;
	static u64 last_time = 1;
	static u64 current_time = 1;
	static u8 flip_count = 1;
	u64 elapse_time;
	u64 temp_data;

	if (!dev) {
		ADF_DEBUG_WARN("parameter dev is ilegal\n");
		goto err_out;
	}

	if (!post) {
		ADF_DEBUG_WARN("parameter post is ilegal\n");
		goto err_out;
	}

	if (!driver_data) {
		ADF_DEBUG_WARN("parameter driver_data is ilegal\n");
		goto err_out;
	}

	sprd_adf_dev = to_sprd_adf_device(dev);
	config = (struct sprd_restruct_config *)driver_data;
	if (!config) {
		ADF_DEBUG_WARN("config state data error\n");
		goto err_out;
	}
	sprd_adf_flip_plane(sprd_adf_dev, config);
	sprd_adf_wait_flip_done(sprd_adf_dev);

	flip_count++;
	sprd_adf_dev->flip_count++;
	current_time = ktime_get_boot_ns();
	elapse_time = current_time - last_time;
	if (elapse_time >= (u64)1000000000L) {
		temp_data = (u64)1000000000L * (u64)(flip_count - 1);
		sprd_adf_dev->fps =
			do_div(temp_data, elapse_time);
		ADF_DEBUG_GENERAL("fps = %u;elapse = %lld;\n",
			sprd_adf_dev->fps, elapse_time);
		flip_count = 1;
		last_time = current_time;
	}

err_out:
	return;
}

/**
 * sprd_adf_device_validate - validate that the proposed
 * configuration
 *
 * @dev: adf device obj
 * @cfg: flip cmd
 * @driver_data: restruct custom data
 *
 * sprd_adf_device_validate() will allocate and return
 * some driver-private state.
 */
static int sprd_adf_device_validate(struct adf_device *dev,
				    struct adf_post *cfg, void **driver_data)
{
	struct sprd_restruct_config *config;
	struct sprd_adf_device *sprd_adf_dev;

	ADF_DEBUG_GENERAL("entern\n");
	if (!dev) {
		ADF_DEBUG_WARN("parameter dev is ilegal\n");
		goto err_out;
	}

	if (!cfg) {
		ADF_DEBUG_WARN("parameter cfg is ilegal\n");
		goto err_out;
	}

	if (!driver_data) {
		ADF_DEBUG_WARN("parameter driver_data is ilegal\n");
		goto err_out;
	}

	sprd_adf_dev = to_sprd_adf_device(dev);
	config =
		sprd_adf_restruct_post_config(sprd_adf_dev, cfg);
	if (!config) {
		ADF_DEBUG_WARN("call restruct_post_configs failed\n");
		goto err_out;
	}
	*driver_data = (void *)config;

	return 0;

err_out:
	return -1;
}

/**
 * sprd_adf_device_state_free - free driver state
 *
 * @dev: adf device obj
 * @driver_data: restruct custom data
 *
 * sprd_adf_device_state_free() free driver-private
 * state allocated during validate.
 */
static void sprd_adf_device_state_free(struct adf_device *dev,
				       void *driver_data)
{
	struct sprd_restruct_config *config;
	struct sprd_adf_device *sprd_adf_dev;

	ADF_DEBUG_GENERAL("entern\n");
	if (!dev) {
		ADF_DEBUG_WARN("parameter dev is ilegal\n");
		goto err_out;
	}

	if (!driver_data) {
		ADF_DEBUG_WARN("parameter driver_data is ilegal\n");
		goto err_out;
	}

	sprd_adf_dev = to_sprd_adf_device(dev);
	config = (struct sprd_restruct_config *)driver_data;
	if (!config) {
		ADF_DEBUG_WARN("config state data error\n");
		goto err_out;
	}

	sprd_adf_free_restruct_config(sprd_adf_dev, config);

err_out:
	return;
}

static const struct adf_device_ops sprd_adf_device_ops = {
	.owner = THIS_MODULE,
	.base = {
		 .custom_data = sprd_adf_device_custom_data,
		 },
	.validate = sprd_adf_device_validate,
	.post = sprd_adf_device_post,
	.state_free = sprd_adf_device_state_free,
};

static const char *adf_device_name[SPRD_ADF_MAX_DEVICE] = {
	"sprd-adf-dev",
};

/**
 * sprd_adf_create_devices - create sprd adf device objs
 *
 * @pdev: parent device of sprd adf dev
 * @n_devs: the number of sprd adf device obj
 * @ops: sprd adf device callback functions
 *
 * sprd_adf_create_devices() create adf device obj and set
 * sprd device callback functions.
 */
struct sprd_adf_device *sprd_adf_create_devices(struct platform_device *pdev,
				size_t n_devs)
{
	struct sprd_adf_device *devs;
	int ret;
	size_t i, j = 0;

	ADF_DEBUG_GENERAL("entern\n");
	if (!pdev) {
		ADF_DEBUG_WARN("parameter pdev is ilegal\n");
		goto error_out1;
	}

	if (n_devs > SPRD_ADF_MAX_DEVICE) {
		ADF_DEBUG_WARN("device number exceed SPRD_ADF_MAX_DEVICE\n");
		goto error_out1;
	}

	devs = kcalloc(n_devs, sizeof(struct sprd_adf_device), GFP_KERNEL);
	if (!devs) {
		ADF_DEBUG_WARN("kzalloc adf_device failed\n");
		goto error_out1;
	}

	for (i = 0; i < n_devs; i++) {
		ret =
		    adf_device_init(&devs[i].base, &pdev->dev,
				    &sprd_adf_device_ops, adf_device_name[i]);
		if (ret < 0) {
			ADF_DEBUG_WARN("adf_device_init failed\n");
			goto error_out0;
		}

		for (j = 0; j < ARRAY_SIZE(sprd_adf_device_attrs); j++) {
			ret = device_create_file(&devs[i].base.base.dev,
					&sprd_adf_device_attrs[j]);
			if (ret < 0) {
				ADF_DEBUG_WARN("create  attr %s failed: %d\n",
				sprd_adf_device_attrs[j].attr.name, ret);
				goto error_out0;
			}
		}
		devs[i].ops = sprd_adf_get_device_ops(i);
		devs[i].data = sprd_adf_get_device_private_data(pdev, i);
		devs[i].flip_count = 0;
		devs[i].vsync_count = 0;
	}

	return devs;

error_out0:
	for (; i >= 0; i--) {
		ADF_DEBUG_WARN("adf_device_destroy index =%zd;\n", i);
		adf_device_destroy(&devs[i].base);
		sprd_adf_destroy_device_private_data(devs[i].data);
		for (; j >= 0; j--)
			device_remove_file(&devs[i].base.base.dev,
				&sprd_adf_device_attrs[j]);
	}

	kfree(devs);
error_out1:
	return NULL;
}

/**
 * sprd_adf_destroy_devices - destroy sprd adf device objs
 *
 * @devs: sprd adf device obj
 * @n_devs: the number of sprd adf device obj
 *
 * sprd_adf_destroy_devices() destroy adf device obj and
 * free some resource
 */
int sprd_adf_destroy_devices(struct sprd_adf_device *devs, size_t n_devs)
{
	size_t i;

	ADF_DEBUG_GENERAL("entern\n");
	if (!devs) {
		ADF_DEBUG_WARN("par devs is null\n");
		goto err_out;
	}

	for (i = 0; i < n_devs; i++) {
		adf_device_destroy(&devs[i].base);
		sprd_adf_destroy_device_private_data(devs[i].data);
	}

	kfree(devs);

	return 0;

err_out:
	return -1;
}

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

#include <linux/compat.h>
#include <linux/fs.h>
#include <linux/sprd_ion.h>
#include <linux/uaccess.h>
#include "compat_sprd_ion.h"
#include "../ion.h"

struct compat_ion_phys_data {
	compat_int_t fd_buffer;
	compat_ulong_t phys;
	compat_size_t size;
};

struct compat_ion_msync_data {
	compat_int_t fd_buffer;
	compat_ulong_t vaddr;
	compat_ulong_t paddr;
	compat_size_t size;
};

struct compat_ion_fence_data {
	compat_uint_t device_type;
	compat_int_t life_value;
	compat_int_t release_fence_fd;
	compat_int_t retired_fence_fd;
};

struct compat_ion_kmap_data {
	compat_int_t fd_buffer;
	compat_u64 kaddr;
	compat_size_t size;
};

static int compat_get_ion_phys_data(
			struct compat_ion_phys_data __user *data32,
			struct ion_phys_data __user *data)
{
	compat_int_t i;
	int err;

	err = get_user(i, &data32->fd_buffer);
	err |= put_user(i, &data->fd_buffer);

	return err;
};

static int compat_put_ion_phys_data(
			struct compat_ion_phys_data __user *data32,
			struct ion_phys_data __user *data)
{
	compat_ulong_t ul;
	compat_size_t s;
	int err;

	err = get_user(ul, &data->phys);
	err |= put_user(ul, &data32->phys);
	err |= get_user(s, &data->size);
	err |= put_user(s, &data32->size);

	return err;
};

static int compat_get_ion_msync_data(
			struct compat_ion_msync_data __user *data32,
			struct ion_msync_data __user *data)
{
	compat_int_t i;
	compat_ulong_t ul;
	compat_size_t s;
	int err;

	err = get_user(i, &data32->fd_buffer);
	err |= put_user(i, &data->fd_buffer);
	err |= get_user(ul, &data32->vaddr);
	err |= put_user(ul, &data->vaddr);
	err |= get_user(ul, &data32->paddr);
	err |= put_user(ul, &data->paddr);
	err |= get_user(s, &data32->size);
	err |= put_user(s, &data->size);

	return err;
};

static int compat_get_ion_kmap_data(
			struct compat_ion_kmap_data __user *data32,
			struct ion_kmap_data __user *data)
{
	compat_int_t i;
	compat_u64 u64;
	compat_size_t s;
	int err;

	err = get_user(i, &data32->fd_buffer);
	err |= put_user(i, &data->fd_buffer);
	err |= get_user(u64, &data32->kaddr);
	err |= put_user(u64, &data->kaddr);
	err |= get_user(s, &data32->size);
	err |= put_user(s, &data->size);

	return err;
};

static int compat_put_ion_kmap_data(
			struct compat_ion_kmap_data __user *data32,
			struct ion_kmap_data __user *data)
{
	compat_int_t i;
	compat_u64 u64;
	compat_size_t s;
	int err;

	err = get_user(i, &data->fd_buffer);
	err |= put_user(i, &data32->fd_buffer);
	err |= get_user(u64, &data->kaddr);
	err |= put_user(u64, &data32->kaddr);
	err |= get_user(s, &data->size);
	err |= put_user(s, &data32->size);

	return err;
};

int compat_get_custom_data(unsigned int cmd,
			   void __user *data32,
			   void __user **data)
{
	int err = 0;

	pr_debug("%s, cmd: %u\n", __func__, cmd);
	switch (cmd) {
	case ION_SPRD_CUSTOM_PHYS:
	{
		struct compat_ion_phys_data __user *custom_data32;
		struct ion_phys_data __user *custom_data;

		custom_data32 = data32;
		custom_data = compat_alloc_user_space(
					sizeof(struct ion_custom_data __user)
					+ sizeof(*custom_data));
		if (!custom_data) {
			pr_err("%s, %d, alloc user space failed!\n",
				__func__, __LINE__);
			return -EFAULT;
		}

		err |= compat_get_ion_phys_data(custom_data32, custom_data);
		*data = (void __user *)custom_data;
		break;
	}
	case ION_SPRD_CUSTOM_MSYNC:
	{
		struct compat_ion_msync_data __user *custom_data32;
		struct ion_msync_data __user *custom_data;

		custom_data32 = data32;
		custom_data = compat_alloc_user_space(
					sizeof(struct ion_custom_data __user)
					+ sizeof(*custom_data));
		if (!custom_data) {
			pr_err("%s, %d, alloc user space failed!\n",
				__func__, __LINE__);
			return -EFAULT;
		}

		err |= compat_get_ion_msync_data(custom_data32, custom_data);
		*data = (void __user *)custom_data;
		break;
	}
	case ION_SPRD_CUSTOM_FENCE_CREATE:
	case ION_SPRD_CUSTOM_FENCE_SIGNAL:
	case ION_SPRD_CUSTOM_FENCE_DUP:
	case ION_SPRD_CUSTOM_INVALIDATE:
	case ION_SPRD_CUSTOM_UNMAP_KERNEL:
		*data = data32;
		break;
	case ION_SPRD_CUSTOM_MAP_KERNEL:
	{
		struct compat_ion_kmap_data __user *custom_data32;
		struct ion_kmap_data __user *custom_data;

		custom_data32 = data32;
		custom_data = compat_alloc_user_space(
					sizeof(struct ion_custom_data __user)
					+ sizeof(*custom_data));
		if (!custom_data) {
			pr_err("%s, %d, alloc user space failed!\n",
				__func__, __LINE__);
			return -EFAULT;
		}

		err |= compat_get_ion_kmap_data(custom_data32, custom_data);
		*data = (void __user *)custom_data;
		break;
	}
	default:
		return -ENOIOCTLCMD;
	}

	return err;
};

int compat_put_custom_data(unsigned int cmd,
			   void __user *data32,
			   void __user *data)
{
	int err = 0;

	pr_debug("%s, cmd: %u\n", __func__, cmd);
	switch (cmd) {
	case ION_SPRD_CUSTOM_PHYS:
	{
		struct compat_ion_phys_data __user *custom_data32;
		struct ion_phys_data __user *custom_data;

		custom_data32 = data32;
		custom_data = data;

		err |= compat_put_ion_phys_data(custom_data32, custom_data);
		break;
	}
	case ION_SPRD_CUSTOM_MSYNC:
	case ION_SPRD_CUSTOM_FENCE_CREATE:
	case ION_SPRD_CUSTOM_FENCE_SIGNAL:
	case ION_SPRD_CUSTOM_FENCE_DUP:
	case ION_SPRD_CUSTOM_INVALIDATE:
		break;
	case ION_SPRD_CUSTOM_MAP_KERNEL:
	{
		struct compat_ion_kmap_data __user *custom_data32;
		struct ion_kmap_data __user *custom_data;

		custom_data32 = data32;
		custom_data = data;

		err |= compat_put_ion_kmap_data(custom_data32, custom_data);
		break;
	}
	default:
		return -ENOIOCTLCMD;
	}

	return err;
};


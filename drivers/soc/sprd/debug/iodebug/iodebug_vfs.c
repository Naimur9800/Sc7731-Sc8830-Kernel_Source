/* linux/fs/iodebug/iodebug_vfs.c
 *
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 1, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *	VERSION			DATE			AUTHOR
 *	1.0			2015-11-23		Jack_wei.zhang
 *
 */
#include <linux/iodebug.h>
#include <linux/fs.h>
#include <linux/genhd.h>
#include <linux/mutex.h>

#define MAX_IO_BUFFER_LEN 64
#define MAX_CACHE_BUFFER_LEN 5


enum storage_device {
	DEV_EMMC,
	DEV_SD,
	DEV_FUSE,
	DEVICE_TYPE_MAX,

	NO_STORAGE = 0xFF,
};

enum io_size {
	IO_SIZE_0_1K,		/* <1K */
	IO_SIZE_1K_4K,		/* 1K,4K */
	IO_SIZE_4K_128K,	/* 4K,128K */
	IO_SIZE_128K_512K,	/* 128K,512K */
	IO_SIZE_512K_PLUS,	/* >512K */

	IO_SIZE_MAX,
};

struct io_info {
	unsigned int cnt[IO_SIZE_MAX];
	unsigned int bytes[IO_SIZE_MAX];
	unsigned long io_jiffies;
};

struct vfs_iodebug_info {
	pid_t pid;
	char name[TASK_COMM_LEN];
	struct io_info read_io_info;
	struct io_info write_io_info;
};

struct task_cache_info {
	char name[TASK_COMM_LEN];
	int idx;
};

static DEFINE_MUTEX(g_vfs_iodebug_lock);

struct vfs_iodebug_info
	g_vfs_io_buffer[DEVICE_TYPE_MAX][MAX_IO_BUFFER_LEN] = {};
unsigned int g_vfs_io_buffer_cnt;

struct vfs_iodebug_info
	g_vfs_io_buffer_temp[DEVICE_TYPE_MAX][MAX_IO_BUFFER_LEN] = {};
unsigned int g_vfs_io_buffer_cnt_temp;

struct task_cache_info
	g_task_cache_buffer[DEVICE_TYPE_MAX][MAX_CACHE_BUFFER_LEN] = {};
unsigned int g_task_cache_pos_arr[DEVICE_TYPE_MAX] = {0};

char *g_device_name[] = {"EMMC", "SD", "FUSE"};

/* err info */
unsigned int g_err_task_cache;
unsigned int g_err_io_buffer;


static enum storage_device iodebug_get_storage_device(struct file *filp)
{
	char *storage_dev_name = NULL;
	struct inode *inode = filp->f_mapping->host;

	if (inode) {
		if ((!inode->i_sb)
			|| (!inode->i_sb->s_bdev)
			|| (!inode->i_sb->s_bdev->bd_disk))
			return NO_STORAGE;
		storage_dev_name = inode->i_sb->s_bdev->bd_disk->disk_name;
	} else
		return NO_STORAGE;

	/* mmcblk0 is EMMC and mmcblk1 is SD */
	if (storage_dev_name[6] == '0')
		return DEV_EMMC;
	else
		return DEV_SD;
}


static int iodebug_update_vfs_io(struct vfs_iodebug_info *iodebug_info,
				int rw, size_t count,
				struct file *filp, unsigned long io_jiffies)
{
	enum io_size io_size;
	struct io_info *my_io_info = NULL;

	if (count <= 1024)
		io_size = IO_SIZE_0_1K;
	else if ((count > 1024) && (count <= 1024*4))
		io_size = IO_SIZE_1K_4K;
	else if ((count > 1024*4) && (count <= 1024*128))
		io_size = IO_SIZE_4K_128K;
	else if ((count > 1024*128) && (count <= 1024*512))
		io_size = IO_SIZE_128K_512K;
	else
		io_size = IO_SIZE_512K_PLUS;

	/* read */
	if (rw == 0)
		my_io_info = &iodebug_info->read_io_info;
	else
		my_io_info = &iodebug_info->write_io_info;

	/* new arrival task */
	if (iodebug_info->pid == 0) {
		memset(iodebug_info, 0, sizeof(struct vfs_iodebug_info));
		iodebug_info->pid = current->pid;
		strncpy(iodebug_info->name,
			current->group_leader->comm,
			sizeof(iodebug_info->name) - 1);
		g_vfs_io_buffer_cnt++;
	}

	my_io_info->cnt[io_size]++;
	my_io_info->bytes[io_size] += count;
	my_io_info->io_jiffies += io_jiffies;

	return 0;
}

static int iodebug_find_task_cache(enum storage_device device)
{
	int i;

	for (i = 0; i < MAX_CACHE_BUFFER_LEN; i++) {
		if (strcmp(g_task_cache_buffer[device][i].name,
				current->group_leader->comm) == 0) {
			int my_idx = g_task_cache_buffer[device][i].idx;

			if ((my_idx < 0) ||
			(strcmp(g_vfs_io_buffer[device][my_idx].name,
			current->group_leader->comm) != 0)) {
				pr_err("%s err,%s:%s,%s:%s,idx:%d,device:%d",
					__func__, "io_buffer",
					g_vfs_io_buffer[device][my_idx].name,
					"current", current->group_leader->comm,
					my_idx, device);
				return -1;
			}
			return g_task_cache_buffer[device][i].idx;
		}
	}

	return -1;
}

static void iodebug_update_task_cache(enum storage_device device, int i)
{
	int pos = g_task_cache_pos_arr[device];
	struct task_cache_info *cache_info = &g_task_cache_buffer[device][pos];

	if (i < MAX_IO_BUFFER_LEN) {
		strcpy(cache_info->name, current->group_leader->comm);
		cache_info->idx = i;
		g_task_cache_pos_arr[device] = (pos+1)%MAX_CACHE_BUFFER_LEN;
	}
}

int iodebug_save_vfs_io(int rw, size_t count, struct file *filp,
			char fuse_flag, unsigned long io_jiffies)
{
	int i;
	int ret = -1;
	enum storage_device storage_device;


	if (fuse_flag == 0) {
		storage_device = iodebug_get_storage_device(filp);
		if (storage_device >= DEVICE_TYPE_MAX)
			return ret;
	} else
		storage_device = DEV_FUSE;

	mutex_lock(&g_vfs_iodebug_lock);
	i = iodebug_find_task_cache(storage_device);
	if (i >= 0)
		goto update_buffer;

	for (i = 0; i < MAX_IO_BUFFER_LEN; i++) {
		if (strcmp(g_vfs_io_buffer[storage_device][i].name,
				current->group_leader->comm) == 0)
			/* has find */
			break;
		if (g_vfs_io_buffer[storage_device][i].pid == 0)
			/* not find and get an new place */
			break;
	}
	iodebug_update_task_cache(storage_device, i);

update_buffer:
	if (i < MAX_IO_BUFFER_LEN) {
		iodebug_update_vfs_io(&g_vfs_io_buffer[storage_device][i],
						rw, count, filp, io_jiffies);
		ret = 0;
	} else
		g_err_io_buffer++;

	mutex_unlock(&g_vfs_iodebug_lock);
	return ret;
}
EXPORT_SYMBOL(iodebug_save_vfs_io);

static void iodebug_print_vfs_io(int rw, int i, struct seq_file *seq_f, void *v)
{
	int j, k;
	unsigned int bytes_per_task = 0, cnt_per_task = 0;
	unsigned int bytes_per_size[IO_SIZE_MAX] = {0};
	unsigned int cnt_per_size[IO_SIZE_MAX] = {0};
	unsigned int bytes_total = 0, cnt_total = 0;
	char print_sum_flag = 0;
	struct io_info *my_io_info = NULL;
	char prf_buffer[256] = {0};

	memset(bytes_per_size, 0, sizeof(bytes_per_size));
	memset(cnt_per_size, 0, sizeof(cnt_per_size));
	bytes_total = 0;
	cnt_total = 0;
	print_sum_flag = 0;

	/* 1.head */
	/* read */
	if (rw == 0) {
		snprintf(prf_buffer, sizeof(prf_buffer),
		"\n\n*%-4s %-5s %5s %16s %13s %17s %19s %17s %15s %14s %16s",
		g_device_name[i], "READ", "PID", "NAME", "[0, 1KB]",
		"(1KB, 4KB]", "(4KB, 128KB]", "(128KB, 512KB]",
		"(512KB, ...]", "SUM", "TIME");
		seq_printf(seq_f, "%s\n", prf_buffer);
	} else {
		snprintf(prf_buffer, sizeof(prf_buffer),
		"\n %-4s %-5s %5s %16s %13s %17s %19s %17s %15s %14s %16s",
		" ", "WRITE", "PID", "NAME", "[0, 1KB]", "(1KB, 4KB]",
		"(4KB, 128KB]", "(128KB, 512KB]", "(512KB, ...]",
		"SUM", "TIME");
		seq_printf(seq_f, "%s\n", prf_buffer);
	}

	/* 2. each task info */
	for (j = 0; j < MAX_IO_BUFFER_LEN; j++) {
		/* not show empty device */
		if (g_vfs_io_buffer_temp[i][j].pid == 0)
			break;
		memset(prf_buffer, 0, sizeof(prf_buffer));
		bytes_per_task = 0;
		cnt_per_task = 0;
		/* read */
		if (rw == 0)
			my_io_info = &g_vfs_io_buffer_temp[i][j].read_io_info;
		else
			my_io_info = &g_vfs_io_buffer_temp[i][j].write_io_info;

		snprintf(prf_buffer, sizeof(prf_buffer), " %16d %16s",
			g_vfs_io_buffer_temp[i][j].pid,
			g_vfs_io_buffer_temp[i][j].name);
		for (k = 0; k < IO_SIZE_MAX; k++) {
			snprintf(prf_buffer, sizeof(prf_buffer),
				"%s %6d , %-7d ",
				prf_buffer,
				my_io_info->cnt[k],
				my_io_info->bytes[k]);
			bytes_per_task += my_io_info->bytes[k];
			cnt_per_task += my_io_info->cnt[k];
			cnt_per_size[k] += my_io_info->cnt[k];
			bytes_per_size[k] += my_io_info->bytes[k];
		}
		snprintf(prf_buffer, sizeof(prf_buffer),
				"%s %7d , %-7d %7dms",
				prf_buffer, cnt_per_task, bytes_per_task,
				jiffies_to_msecs(my_io_info->io_jiffies));

		prf_buffer[sizeof(prf_buffer)-1] = 0;
		if (cnt_per_task > 0) {
			/* print the task's info */
			seq_printf(seq_f, "%s\n", prf_buffer);
			print_sum_flag++;
		}

		cnt_total += cnt_per_task;
		bytes_total += bytes_per_task;
	}

	/* 3. summery */
	if (print_sum_flag > 1) {
		seq_printf(seq_f, "%37s%s%s%s\n",
				" ",
				"---------------------------------",
				"---------------------------------",
				"----------------------------------");
		seq_printf(seq_f, "%34s", " ");
		for (k = 0; k < IO_SIZE_MAX; k++) {
			seq_printf(seq_f, " %6d , %-7d ",
					cnt_per_size[k], bytes_per_size[k]);
		}
		seq_printf(seq_f, " %7d , %-7d\n", cnt_total, bytes_total);
	}
}

static int iodebug_show_vfs_io(struct seq_file *seq_f, void *v)
{
	int i;

	if ((g_err_io_buffer) || (g_err_task_cache)) {
		seq_printf(seq_f,
			"\nERROR: g_err_io_buffer:%d , g_err_task_cache:%d\n",
			g_err_io_buffer, g_err_task_cache);
		g_err_io_buffer = 0;
		g_err_task_cache = 0;
	}

	mutex_lock(&g_vfs_iodebug_lock);
	memcpy(g_vfs_io_buffer_temp, g_vfs_io_buffer, sizeof(g_vfs_io_buffer));
	g_vfs_io_buffer_cnt_temp = g_vfs_io_buffer_cnt;
	memset(g_vfs_io_buffer, 0, sizeof(g_vfs_io_buffer));
	g_vfs_io_buffer_cnt = 0;
	memset(g_task_cache_buffer, -1, sizeof(g_task_cache_buffer));
	memset(g_task_cache_pos_arr, 0, sizeof(g_task_cache_pos_arr));
	mutex_unlock(&g_vfs_iodebug_lock);

	seq_puts(seq_f, "     count , KB");
	for (i = 0; i < DEVICE_TYPE_MAX; i++) {
		if (g_vfs_io_buffer_temp[i][0].pid == 0)
			continue;
		/* not show FUSE */
		if (i == DEV_SD)
			if ((strcmp("sdcard",
				g_vfs_io_buffer_temp[i][0].name) == 0)
				&& (g_vfs_io_buffer_temp[i][1].pid == 0)) {
				g_device_name[DEV_FUSE] = "SD";
				continue;
			}

		iodebug_print_vfs_io(0, i, seq_f, v);
		iodebug_print_vfs_io(1, i, seq_f, v);
	}

	seq_puts(seq_f, "\n\n");

	return 0;
}

static int iodebug_open_vfs_io(struct inode *inode, struct file *file)
{
	return single_open(file, iodebug_show_vfs_io, NULL);
}
IODEBUG_FOPS(vfs_io);

int iodebug_vfs_trace_init(void)
{
	memset(g_vfs_io_buffer, 0, sizeof(g_vfs_io_buffer));
	memset(g_vfs_io_buffer_temp, 0, sizeof(g_vfs_io_buffer_temp));
	memset(g_task_cache_buffer, -1, sizeof(g_task_cache_buffer));
	memset(g_task_cache_pos_arr, 0, sizeof(g_task_cache_pos_arr));

	/*init debugfs entry*/
	IODEBUG_ADD_FILE("vfs_iodebug", S_IRUSR, iodebug_root_dir,
					NULL, iodebug_vfs_io_fops);

	return 0;
}


module_param_named(vfs_io_interval, g_vfs_io_buffer_cnt,
				int, S_IRUGO | S_IWUSR);

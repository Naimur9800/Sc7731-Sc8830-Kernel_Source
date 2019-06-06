/* linux/fs/iodebug/iodebug_bdi.c
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
#include <linux/timer.h>
#include <linux/writeback.h>

#define STORAGE_EMMC	"179:0"
#define STORAGE_SD	"179:32"
static spinlock_t iodebug_bdi_lock;
unsigned int g_bdi_reason_buffer[WB_REASON_MAX] = {0};
unsigned int g_bdi_write_pages_buffer[] = {0, 0, 0};
static const char * const reason_string[] = {
	"WB_REASON_BACKGROUND",
	"WB_REASON_TRY_TO_FREE_PAGES",
	"WB_REASON_SYNC",
	"WB_REASON_PERIODIC",
	"WB_REASON_LAPTOP_TIMER",
	"WB_REASON_FREE_MORE_MEM",
	"WB_REASON_FS_FREE_SPACE",
	"WB_REASON_FORKER_THREAD"
};
static const char * const device_name[] = {"emmc", "sd", "other"};

void iodebug_bdi_save_reason(enum wb_reason reason)
{
	unsigned long	flags;

	spin_lock_irqsave(&iodebug_bdi_lock, flags);
	g_bdi_reason_buffer[reason]++;
	spin_unlock_irqrestore(&iodebug_bdi_lock, flags);
}
EXPORT_SYMBOL(iodebug_bdi_save_reason);

void iodebug_bdi_save_pages(const char *dev_no, long nr_page)
{
	int device;
	unsigned long   flags;

	if (strcmp(STORAGE_EMMC, dev_no) == 0)
		device = 0;
	else if (strcmp(STORAGE_SD, dev_no) == 0)
		device = 1;
	else
		device = 2;

	spin_lock_irqsave(&iodebug_bdi_lock, flags);
	g_bdi_write_pages_buffer[device] += nr_page;
	spin_unlock_irqrestore(&iodebug_bdi_lock, flags);
}
EXPORT_SYMBOL(iodebug_bdi_save_pages);

static int iodebug_show_bdi_io(struct seq_file *seq_f, void *v)
{
	int i;
	unsigned int reason_buffer[WB_REASON_MAX] = {0};
	unsigned int write_pages_buffer[] = {0, 0, 0};
	unsigned long   flags;

	spin_lock_irqsave(&iodebug_bdi_lock, flags);
	memcpy(reason_buffer, g_bdi_reason_buffer,
				sizeof(g_bdi_reason_buffer));
	memcpy(write_pages_buffer, g_bdi_write_pages_buffer,
				sizeof(g_bdi_write_pages_buffer));
	memset(g_bdi_reason_buffer, 0,
				sizeof(g_bdi_reason_buffer));
	memset(g_bdi_write_pages_buffer, 0,
				sizeof(g_bdi_write_pages_buffer));
	spin_unlock_irqrestore(&iodebug_bdi_lock, flags);

	seq_puts(seq_f, "\n  *REASON:  count\n");
	for (i = 0; i < WB_REASON_MAX; i++) {
		seq_printf(seq_f, "%30s: %4d    ", reason_string[i],
							reason_buffer[i]);
		if (i%3 == 2)
			seq_puts(seq_f, "\n");
	}

	seq_puts(seq_f, "\n  *DEVICE:  KB\n");
	for (i = 0; i < 3; i++)
		seq_printf(seq_f, "%15s: %d KB", device_name[i],
						write_pages_buffer[i]*4);
	seq_puts(seq_f, "\n");

	return 0;
}

static int iodebug_open_bdi_io(struct inode *inode, struct file *file)
{
	return single_open(file, iodebug_show_bdi_io, NULL);
}
IODEBUG_FOPS(bdi_io);

int iodebug_bdi_trace_init(void)
{
	memset(g_bdi_reason_buffer, 0, sizeof(g_bdi_reason_buffer));

	spin_lock_init(&iodebug_bdi_lock);
	/*init debugfs entry*/
	IODEBUG_ADD_FILE("bdi_iodebug", S_IRUSR, iodebug_root_dir,
					NULL, iodebug_bdi_io_fops);

	return 0;
}


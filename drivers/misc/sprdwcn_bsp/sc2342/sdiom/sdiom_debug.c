#include "sdiom_debug.h"

static unsigned char buf[SDIOM_WRITE_SIZE];
static struct dentry *debug_root;

/* send sdio irq to cp, and trigger cp2 assert */
static int sdiom_dt_trigger_assert(void)
{
	unsigned char reg_int_cp0 = 0;

	sdiom_aon_readb(REG_AP_INT_CP0, &reg_int_cp0);
	reg_int_cp0 |= BIT_AP_INT_CP0_DEBUG;
	sdiom_aon_writeb(REG_AP_INT_CP0, reg_int_cp0);

	return 0;
}

static int at_cmd_init(void)
{
	return 0;
}

static int at_cmd_deinit(void)
{
	return 0;
}

static int at_cmd_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t at_cmd_read(struct file *filp,
	char __user *user_buf, size_t count, loff_t *pos)
{
	return count;
}

static ssize_t at_cmd_write(struct file *filp,
		const char __user *user_buf, size_t count, loff_t *pos)
{
	long int debug_level;
	int ret;

	if (count > SDIOM_WRITE_SIZE) {
		sdiom_err("%s write size > %d\n",
			__func__, SDIOM_WRITE_SIZE);
		return -ENOMEM;
	}

	memset(buf, 0, SDIOM_WRITE_SIZE);
	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	sdiom_info("%s write: %s\n", __func__, buf);

	if (strncmp(buf, "debug_level=", 12) == 0) {
		buf[SDIOM_WRITE_SIZE - 1] = 0;
		ret = kstrtol(&buf[strlen("debug_level=")],
			10, &debug_level);
		sdiom_info("%s debug_level: %ld\n",
			__func__, debug_level);
		sdiom_set_debug_level(debug_level);
		if (sdiom_get_debug_level() == LEVEL_1) {
			ret = sdiom_rx_recvbufbk_alloc();
			if (ret == ERROR)
				sdiom_set_debug_level(0);
		}
		return count;
	}

	if (strncmp(buf, "assert", 6) == 0) {
		sdiom_dt_trigger_assert();
		return count;
	}

	return count;
}

static int at_cmd_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static const struct file_operations at_cmd_fops = {
	.open = at_cmd_open,
	.read = at_cmd_read,
	.write = at_cmd_write,
	.release = at_cmd_release,
};

static int debug_help_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t debug_help_read(struct file *filp,
	char __user *user_buf, size_t count, loff_t *pos)
{
	return count;
}

static int debug_help_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static const struct file_operations debug_help = {
	.open = debug_help_open,
	.read = debug_help_read,
	.release = debug_help_release,
};

struct entry_file {
	const char *name;
	const struct file_operations *file_ops;
};

static struct entry_file entry_table[] = {
	{
		.name = "help",
		.file_ops = &debug_help,
	},
	{
		.name = "at_cmd",
		.file_ops = &at_cmd_fops,
	},
};

void sdiom_debug_init(void)
{
	int i;

	/*create debugfs */
	debug_root = debugfs_create_dir("sdiom_debug", NULL);
	for (i = 0; i < ARRAY_SIZE(entry_table); i++) {
		debugfs_create_file(entry_table[i].name, 0444,
			debug_root, NULL, entry_table[i].file_ops);
	}

	at_cmd_init();
}

void sdiom_debug_deinit(void)
{
	at_cmd_deinit();
}


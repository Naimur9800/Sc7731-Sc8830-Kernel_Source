/* linux/fs/iodebug/iodebug_common.c
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
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/netlink.h>
#include <net/netlink.h>
#include <net/net_namespace.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/oom.h>
#include <linux/buffer_head.h>
#include <linux/iodebug.h>

struct dentry *iodebug_root_dir;
static unsigned int iodebug_trace_enable;
asmlinkage int iodebug_trace(const char *fmt, ...)
{
	va_list args;
	int r = 0;

	if (iodebug_trace_enable) {
		va_start(args, fmt);
		r = vprintk_emit(0, -1, NULL, 0, fmt, args);
		va_end(args);
	}
	return r;
}
EXPORT_SYMBOL(iodebug_trace);

/*
 *for get io information periodically
 */
#if defined(CONFIG_IODEBUG_VFS) || defined(CONFIG_IODEBUG_BDI)
#define IODEBUG_PERIODIC	(5000)
static struct timer_list g_show_timer;
bool g_iodebug_vfs_timer_flag;
/* just send msg to slog, and then slog will cat the file */
static void show_timer_function(unsigned long arg)
{
	if (iodebug_nl_send_msg(NLMSGTYPE_K2U_SEND_EVENT,
				NLMSGEVENT_SHOW_IO_PERIOD))
		pr_err("iodebug: %s can't send to slog!\n", __func__);
	else
		mod_timer(&g_show_timer,
			jiffies + msecs_to_jiffies(IODEBUG_PERIODIC));
}

static int iodebug_periodic_func_timer_init(void)
{
	if (g_iodebug_vfs_timer_flag == false) {
		init_timer(&g_show_timer);
		g_show_timer.expires = jiffies
					+ msecs_to_jiffies(IODEBUG_PERIODIC);
		g_show_timer.data = 0UL;
		g_show_timer.function = show_timer_function;
		add_timer(&g_show_timer);
		g_iodebug_vfs_timer_flag = true;
	} else
		mod_timer(&g_show_timer,
				jiffies + msecs_to_jiffies(IODEBUG_PERIODIC));

	return 0;
}
#endif

/*
 *iodebug support netlink
 */
#define NLMSG_USER_PID_MAXLEN 4
static struct sock *iodebug_nlsock;
static int nlsock_user_data;
static int nlsock_user_pid;
static int iodebug_nl_rcv_pid(struct sk_buff *skb, struct nlmsghdr *nlh)
{
	int len = nlmsg_len(nlh);

	if (len > NLMSG_USER_PID_MAXLEN) {
		pr_err("iodebug: Invalid data len.\n");
		return -ERANGE;
	}
	nlsock_user_data = *(int *)nlmsg_data(nlh);
	nlsock_user_pid = nlh->nlmsg_pid;
	pr_info("%s, data:0x%x, pid:%d.\n",
		"iodebug: Establish connection with client",
		nlsock_user_data, nlsock_user_pid);
#if defined(CONFIG_IODEBUG_VFS) || defined(CONFIG_IODEBUG_BDI)
	iodebug_periodic_func_timer_init();
#endif
	return 0;
}

int iodebug_nl_send_msg(int msg_type, int event_key)
{
	int res;
	char *data;
	struct nlmsghdr *nlh;
	struct sk_buff *skb_out;
	int msg_size;

	if (!nlsock_user_pid) {
		pr_err("iodebug: Please establish connection firstly!\n");
		return -ENOENT;
	}
	msg_size = sizeof(event_key);
	/*alloc sk_buff, include nlmsg head and data*/
	skb_out = nlmsg_new(msg_size, 0);
	if (!skb_out) {
		pr_err("iodebug: Failed to allocate new skb.\n");
		return -ENOMEM;
	}

	nlh = nlmsg_put(skb_out, nlsock_user_pid, 0, msg_type, msg_size, 0);
	if (!nlh) {
		kfree_skb(skb_out);
		pr_err("iodebug: Failed to nlmsg_put skb.\n");
		return -EMSGSIZE;
	}
	/* not in mcast group */
	NETLINK_CB(skb_out).dst_group = 0;
	data = nlmsg_data(nlh);
	*(int *)data = event_key;

	res = nlmsg_unicast(iodebug_nlsock, skb_out, nlsock_user_pid);
	if (res < 0) {
		pr_err("iodebug: Error while sending event to user.\n");
		return res;
	}
	return 0;
}


static int iodebug_nl_rcv_msg(struct sk_buff *skb, struct nlmsghdr *nlh)
{
	unsigned short type;

	type = nlh->nlmsg_type;
	switch (type) {
	case NLMSGTYPE_U2K_GET_PID:
		return iodebug_nl_rcv_pid(skb, nlh);
	default:
		return -EINVAL;
	}
}

static void iodebug_nl_rcv(struct sk_buff *skb)
{
	netlink_rcv_skb(skb, &iodebug_nl_rcv_msg);
}

static int iodebug_nl_init(void)
{
	struct netlink_kernel_cfg cfg = {
		.input  = iodebug_nl_rcv,
	};

	iodebug_nlsock = netlink_kernel_create(&init_net,
						NETLINK_IODEBUG, &cfg);
	if (!iodebug_nlsock) {
		pr_err("iodebug: iodebug netlink init failed.\n");
		return -ENOMEM;
	}
	pr_info("iodebug: iodebug netlink init done.\n");
	return 0;
}

void iodebug_anr_get_io_info(void)
{
	if (iodebug_nl_send_msg(NLMSGTYPE_K2U_SEND_EVENT,
					NLMSGEVENT_SHOW_IO_ANR))
		pr_err("iodebug: %s fail to send!\n", __func__);
}
EXPORT_SYMBOL(iodebug_anr_get_io_info);

/*iodebug init*/
static int __init iodebug_init(void)
{
	int ret = 0;

	/*create io_debug directory*/
	iodebug_root_dir = debugfs_create_dir("iodebug", NULL);
	if (!iodebug_root_dir) {
		pr_err("Can't create debugfs dir for IO debug.\n");
		return -ENOENT;
	}
	/*netlink init*/
	ret = iodebug_nl_init();
	if (ret != 0)
		return ret;

#ifdef CONFIG_IODEBUG_HOTPOINT
	iodebug_point_monitor_init();
#endif
#ifdef CONFIG_IODEBUG_BDI
	iodebug_bdi_trace_init();
#endif
#ifdef CONFIG_IODEBUG_VFS
	iodebug_vfs_trace_init();
#endif

	return ret;
}


/*iodebug exit*/
static void __exit iodebug_exit(void)
{
	netlink_kernel_release(iodebug_nlsock);
}

module_param_named(trace_enable, iodebug_trace_enable, int, S_IRUGO | S_IWUSR);

module_init(iodebug_init);
module_exit(iodebug_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("IO debug feature.");

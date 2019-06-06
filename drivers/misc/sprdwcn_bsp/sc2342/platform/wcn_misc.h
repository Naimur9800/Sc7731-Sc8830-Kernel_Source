#ifndef __WCN_MISC_H__
#define __WCN_MISC_H__

#include <linux/mutex.h>
#include <linux/types.h>

/* Hours offset for GM and China-BeiJing */
#define WCN_BTWF_TIME_OFFSET (8)

#define ATCMD_FIFO_MAX	(16)

enum atcmd_owner {
	/* default AT CMD reply to WCND */
	WCN_ATCMD_WCND = 0x0,
	/* Kernel not deal response info from CP2. 20180515 */
	WCN_ATCMD_KERNEL,
	WCN_ATCMD_LOG,
};

/*
 * Until now, CP2 response every AT CMD to AP side
 * without owner-id.
 * AP side transfer every ATCMD response info to WCND.
 * If AP send AT CMD on kernel layer, and the response
 * info transfer to WCND and caused WCND deal error
 * response CMD.
 * We will save all of the owner-id to the fifo.
 * and dispatch the response ATCMD info to the matched owner.
 */
struct atcmd_fifo {
	enum atcmd_owner owner[ATCMD_FIFO_MAX];
	unsigned int head;
	unsigned int tail;
	struct mutex lock;
};

struct wcn_tm {
	int tm_msec;    /* mili seconds */
	int tm_sec;     /* seconds */
	int tm_min;     /* minutes */
	int tm_hour;    /* hours */
	int tm_mday;    /* day of the month */
	int tm_mon;     /* month */
	int tm_year;    /* year */
};

void mdbg_atcmd_owner_init(void);
void mdbg_atcmd_owner_deinit(void);
long int mdbg_send_atcmd(char *buf, long int len, enum atcmd_owner owner);
enum atcmd_owner mdbg_atcmd_owner_peek(void);
void mdbg_atcmd_clean(void);
/* AP notify BTWF time by at+aptime=... cmd */
long int wcn_ap_notify_btwf_time(void);

#endif

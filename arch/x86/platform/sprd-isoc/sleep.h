/*
 * Variables and functions used by the code in pm.c
 */

/* ACPI PM1 CONTROL OFFSET */
#define PM1_CNT                 0x4
#define SLEEP_TYPE_MASK         0xFFFFECFF
#define SLEEP_TYPE_S3           0x1800
#define SLEEP_ENABLE            0x2000

/* This must match data at wakeup_64.S */
#ifdef CONFIG_X86_64
struct wakeup_trampoline_header {
	u64 start;
	u64 efer;
	u64 misc;
	u32 cr4;
};

extern struct wakeup_trampoline_header wakeup_trampoline_header;
extern u64 wakeup_trampoline_pgd;
extern void wakeup_long64(void);
extern void wakeup_long64_direct(void);
#endif
extern void wakeup_start_32(void);

extern void do_suspend_lowlevel(void);
extern void resume_point(void);


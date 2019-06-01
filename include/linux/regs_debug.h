#ifndef _REGS_DEBUG_
#define _REGS_DEBUG_

#include <linux/smp.h>
#include <linux/threads.h>
#include <linux/jiffies.h>

struct sprd_debug_regs_access{
	u32 vaddr;
	u32 value;
	u32 stack;
	u32 pc;
	unsigned long time;
	unsigned int status;
};

#define sprd_debug_regs_read_start(a)	({u32 cpu_id, stack, lr;	\
		asm volatile(						\
			"	mrc	p15, 0, %0, c0, c0, 5\n"	\
			"	ands %0, %0, #0xf\n"			\
			"	mov %2, r13\n"				\
			"	mov %1, lr\n"				\
			: "=&r" (cpu_id), "=&r" (lr), "=&r" (stack)	\
			:						\
			: "memory");					\
		if(sprd_debug_last_regs_access){			\
		sprd_debug_last_regs_access[cpu_id].value = 0;		\
		sprd_debug_last_regs_access[cpu_id].vaddr = (u32)a;	\
		sprd_debug_last_regs_access[cpu_id].stack = stack;	\
		sprd_debug_last_regs_access[cpu_id].pc = lr;		\
		sprd_debug_last_regs_access[cpu_id].status = 0;}	\
		})

#define sprd_debug_regs_write_start(v, a)	({u32 cpu_id, stack, lr;	\
		asm volatile(						\
			"	mrc	p15, 0, %0, c0, c0, 5\n"	\
			"	ands %0, %0, #0xf\n"			\
			"	mov %2, r13\n"				\
			"	mov %1, lr\n"				\
			: "=&r" (cpu_id), "=&r" (lr), "=&r" (stack)	\
			:						\
			: "memory");					\
		if(sprd_debug_last_regs_access){			\
		sprd_debug_last_regs_access[cpu_id].value = (u32)(v);	\
		sprd_debug_last_regs_access[cpu_id].vaddr = (u32)(a);	\
		sprd_debug_last_regs_access[cpu_id].stack = stack;	\
		sprd_debug_last_regs_access[cpu_id].pc = lr;		\
		sprd_debug_last_regs_access[cpu_id].status = 0;}	\
		})

#define sprd_debug_regs_access_done()	({u32 cpu_id, lr;		\
		asm volatile(						\
			"	mrc	p15, 0, %0, c0, c0, 5\n"	\
			"	ands %0, %0, #0xf\n"			\
			"	mov %1, lr\n"				\
			: "=&r" (cpu_id), "=&r" (lr)			\
			:						\
			: "memory");					\
		if(sprd_debug_last_regs_access){			\
		sprd_debug_last_regs_access[cpu_id].time = jiffies;     \
		sprd_debug_last_regs_access[cpu_id].status = 1;}	\
		})

#endif

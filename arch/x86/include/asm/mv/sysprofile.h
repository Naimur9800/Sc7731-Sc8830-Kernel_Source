/* ----------------------------------------------------------------------------
   Copyright (C) 2015 Intel Corporation

 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.

  ---------------------------------------------------------------------------*/

#ifndef _sysprofile_h_
#define _sysprofile_h_

#include <linux/io.h>
#include <linux/smp.h>
#include <linux/types.h>

#if defined(CONFIG_SYSTEM_PROFILING) || defined(CONFIG_SYSTEM_PROFILING_V2)
#include <mv/vmcalls.h>
#endif

#define SYSPROF_NOF_LINUX_CORES         CONFIG_NR_CPUS

#ifdef CONFIG_SMP
#define SYSPROF_CORE_ID                 raw_smp_processor_id()
#else
#define SYSPROF_CORE_ID                 0
#endif

#if !defined(SYSTEM_PROFILING_CTX)
/* Type: Linux, Instance: 0 (default) */
#define SYSTEM_PROFILING_CTX            0x10
#endif

/* Event class definitions */
#define SYSPROF_EVENT_CLASS_RTOS        0
#define SYSPROF_EVENT_CLASS_RTOS_EX     1
#define SYSPROF_EVENT_CLASS_BW_COUNTS   4
#define SYSPROF_EVENT_CLASS_POWER       10

#if defined(CONFIG_SYSTEM_PROFILING)
/* Legacy System Profiling */

#define SYSPROF_LINUX_CTX_INST          0x10
#define SYSPROF_NOF_EVENT_CLASSES       11

#define SYS_PROF_IF(c)	sys_prof_if[SYSPROF_CORE_ID][SYSPROF_EVENT_CLASS_##c]

/* RTOS internal */
#define sysprof_sys_idle()		iowrite32((0x00000200 | \
						   SYSPROF_LINUX_CTX_INST), \
						  SYS_PROF_IF(RTOS))
#define sysprof_sys_active()		iowrite32((0x00000300 | \
						   SYSPROF_LINUX_CTX_INST), \
						  SYS_PROF_IF(RTOS))
#define sysprof_interrupt(line)		iowrite32((0x000E0000 | \
						   (line & 0xFFFF)), \
						  SYS_PROF_IF(RTOS))
#define sysprof_int_enter()		iowrite32((0x00000800), \
						  SYS_PROF_IF(RTOS))
#define sysprof_int_leave()		iowrite32((0x00000900), \
						  SYS_PROF_IF(RTOS))

/* RTOS extended */
#define sysprof_task_enter(tid)		iowrite32((0x00030000 | \
						   (tid  & 0xFFFF)), \
						  SYS_PROF_IF(RTOS_EX))
#define sysprof_task_leave(tid)		iowrite32((0x00040000 | \
						   (tid  & 0xFFFF)), \
						  SYS_PROF_IF(RTOS_EX))
#define sysprof_syscall_enter(scn)	iowrite32((0x00188000 | \
						   (scn  & 0x7FFF)), \
						  SYS_PROF_IF(RTOS_EX))
#define sysprof_syscall_leave(scn)	iowrite32((0x00198000 | \
						   (scn  & 0x7FFF)), \
						  SYS_PROF_IF(RTOS_EX))

/* Virtualization */
#define sysprof_xirq_to_mex(num)	iowrite32((0x00050000 | \
						   (num & 0xFFFF)), \
						  SYS_PROF_IF(RTOS))
#define sysprof_xirq_to_linux(num)	iowrite32((0x00060000 | \
						   (num & 0xFFFF)), \
						  SYS_PROF_IF(RTOS))
#define sysprof_hw_interrupt(line)	iowrite32((0x000F0000 | \
						   (line & 0xFFFF)), \
						  SYS_PROF_IF(RTOS))
#define sysprof_vmexit_start()		iowrite32((0x00000F01), \
						  SYS_PROF_IF(RTOS))
#define sysprof_vmentry_end()		iowrite32((0x00000F02), \
						  SYS_PROF_IF(RTOS))
#define sysprof_vmcall_entry(nr)	iowrite32((0x00001B00 | \
						   (nr & 0xFF)), \
						  SYS_PROF_IF(RTOS))
#define sysprof_platsvc_entry(svc, op)	iowrite32((0x003B0000 | \
						   ((svc & 0xFF) << 8) | \
						   (op & 0xFF)), \
						  SYS_PROF_IF(RTOS))

/* Bandwidth counters */

/*
 * Use the current mapping from Modem side.
 * Workaround till Linux is capable of dumping its own static metadata
 */
#define eSP_BWC_IPC_MEM_ul_total    52
#define eSP_BWC_IPC_MEM_dl_total    54

#define sysprof_bw( name, size )                                                                                                                        \
{                                                                                                                                                       \
            unsigned int sz = (unsigned int)size;                                                                                                       \
            if((sz >> 16) > 0)                                                                                                                          \
            {                                                                                                                                           \
                iowrite32(((0x04 << 24) | ( (eSP_##name & 0xFF) << 16 ) | (sz >> 16)),SYS_PROF_IF(BW_COUNTS));                                          \
            }                                                                                                                                           \
            iowrite32(((0x01 << 24) | ( (eSP_##name & 0xFF) << 16 ) | (sz & 0xFFFF)),SYS_PROF_IF(BW_COUNTS));                                           \
}

/* POWER */
#define sysprof_dvfs_vcore_load(data)       iowrite32((0x002C0000 | \
						   (data & 0xFFFF)), \
						  SYS_PROF_IF(POWER))
#define sysprof_dvfs_vcore_load_bal(data)   iowrite32((0x002D0000 | \
						   (data & 0xFFFF)), \
						  SYS_PROF_IF(POWER))
#define sysprof_dvfs_cpu_freq_notif(data)   iowrite32((0x002E0000 | \
						   (data & 0xFFFF)), \
						  SYS_PROF_IF(POWER))
#define sysprof_set_idle_state(data)        iowrite32((0x00002000 | \
						   (data & 0x00FF)), \
						  SYS_PROF_IF(POWER))
#define sysprof_idle_exit(data)             iowrite32((0x00150000 | \
						   (data & 0xFFFF)), \
						  SYS_PROF_IF(POWER))
#define sysprof_sleep_enter(data)           iowrite32((0x00160000 | \
						   (data & 0xFFFF)), \
						  SYS_PROF_IF(POWER))
#define sysprof_sleep_exit(data)            iowrite32((0x00170000 | \
						   (data & 0xFFFF)), \
						  SYS_PROF_IF(POWER))


#define mv_guest_trace_vmcall_entry(nr, svc, op)	{ \
		sysprof_platsvc_entry(svc, op); \
	}

extern uint32_t __iomem
	*sys_prof_if[CONFIG_NR_CPUS][SYSPROF_NOF_EVENT_CLASSES];

#elif defined(CONFIG_SYSTEM_PROFILING_V2)

#define SYS_PROF_IF                     sys_prof_if

#define SYS_PROF_IF_WP                  sys_prof_if.sys_prof_if_wp
#define SYS_PROF_IF_CLASS_MASK          sys_prof_if.sys_prof_if_class_mask
#define SYS_PROF_IF_SWT_TS_OFFSET       sys_prof_if.sys_prof_if_swt_ts_offset

/* RTOS internal */
#define sysprof_sys_idle()                                              \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_RTOS)) { \
		iowrite32((0x00000200 | SYSTEM_PROFILING_CTX),          \
			SYS_PROF_IF_WP[SYSPROF_CORE_ID]);               \
	}                                                               \
}
#define sysprof_sys_active()                                            \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_RTOS)) { \
		iowrite32((0x00000300 | SYSTEM_PROFILING_CTX),          \
			SYS_PROF_IF_WP[SYSPROF_CORE_ID]);               \
	}                                                               \
}
#define sysprof_interrupt(line)                                         \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_RTOS)) { \
		iowrite32((0x000E0000 | (line & 0xFFFF)),               \
			SYS_PROF_IF_WP[SYSPROF_CORE_ID]);               \
	}                                                               \
}
#define sysprof_int_enter()                                             \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_RTOS)) { \
		iowrite32((0x00000800), SYS_PROF_IF_WP[SYSPROF_CORE_ID]); \
	}                                                               \
}
#define sysprof_int_leave()                                             \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_RTOS)) { \
		iowrite32((0x00000900), SYS_PROF_IF_WP[SYSPROF_CORE_ID]); \
	}                                                               \
}

/* RTOS extended */
#define sysprof_task_enter(tid)                                         \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_RTOS_EX)) { \
		iowrite32((0x00030000 | (tid & 0xFFFF)),                \
			SYS_PROF_IF_WP[SYSPROF_CORE_ID]);               \
	}                                                               \
}
#define sysprof_task_leave(tid)                                         \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_RTOS_EX)) { \
		iowrite32((0x00040000 | (tid & 0xFFFF)),                \
			SYS_PROF_IF_WP[SYSPROF_CORE_ID]);               \
	}                                                               \
}
#define sysprof_syscall_enter(scn)                                      \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_RTOS_EX)) { \
		iowrite32((0x00188000 | (scn & 0x7FFF)),                \
			SYS_PROF_IF_WP[SYSPROF_CORE_ID]);               \
	}                                                               \
}
#define sysprof_syscall_leave(scn)                                      \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_RTOS_EX)) { \
		iowrite32((0x00198000 | (scn & 0x7FFF)),                \
			SYS_PROF_IF_WP[SYSPROF_CORE_ID]);               \
	}                                                               \
}
/* Virtualization */
#define sysprof_xirq_to_mex(num)                                        \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_RTOS)) { \
		iowrite32((0x00050000 | (num & 0xFFFF)),                \
			SYS_PROF_IF_WP[SYSPROF_CORE_ID]);               \
	}                                                               \
}
#define sysprof_xirq_to_linux(num)                                      \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_RTOS)) { \
		iowrite32((0x00060000 | (num & 0xFFFF)),                \
			SYS_PROF_IF_WP[SYSPROF_CORE_ID]);               \
	}                                                               \
}
#define sysprof_hw_interrupt(line)                                      \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_RTOS)) { \
		iowrite32((0x000F0000 | (line & 0xFFFF)),               \
			SYS_PROF_IF_WP[SYSPROF_CORE_ID]);               \
	}                                                               \
}
#define sysprof_vmexit_start()                                          \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_RTOS)) { \
		iowrite32((0x00000F01), SYS_PROF_IF_WP[SYSPROF_CORE_ID]); \
	}                                                               \
}
#define sysprof_vmentry_end()                                           \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_RTOS)) { \
		iowrite32((0x00000F02), SYS_PROF_IF_WP[SYSPROF_CORE_ID]); \
	}                                                               \
}
#define sysprof_vmcall_entry(nr)                                        \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_RTOS)) { \
		iowrite32((0x00001B00 | (nr & 0xFF)),                   \
			SYS_PROF_IF_WP[SYSPROF_CORE_ID]);               \
	}                                                               \
}
#define sysprof_platsvc_entry(svc, op)                                  \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_RTOS)) { \
		iowrite32((0x003B0000 | ((svc & 0xFF) << 8) | (op & 0xFF)), \
			SYS_PROF_IF_WP[SYSPROF_CORE_ID]);               \
	}                                                               \
}

/* Bandwidth counters */

/*
 * Use the current mapping from Modem side.
 * Workaround till Linux is capable of dumping its own static metadata
 */
#define eSP_BWC_IPC_MEM_ul_total        52
#define eSP_BWC_IPC_MEM_dl_total        54

#define sysprof_bw(name, size)                                          \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_BW_COUNTS)) { \
		unsigned int sz = (unsigned int)size;                   \
		if ((sz >> 16) > 0) {                                   \
			iowrite32(((0x04 << 24) | ((eSP_##name & 0xFF) << 16) \
			| (sz >> 16)), SYS_PROF_IF_WP[SYSPROF_CORE_ID]); \
		}                                                       \
		iowrite32(((0x01 << 24) | ((eSP_##name & 0xFF) << 16) | \
			(sz & 0xFFFF)), SYS_PROF_IF_WP[SYSPROF_CORE_ID]); \
	}                                                               \
}
/* POWER */
#define sysprof_dvfs_vcore_load(data)                                   \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_POWER)) { \
		iowrite32((0x002C0000 | (data & 0xFFFF)),               \
			SYS_PROF_IF_WP[SYSPROF_CORE_ID]);               \
	}                                                               \
}
#define sysprof_dvfs_vcore_load_bal(data)                               \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_POWER)) { \
		iowrite32((0x002D0000 | (data & 0xFFFF)),               \
			SYS_PROF_IF_WP[SYSPROF_CORE_ID]);               \
	}                                                               \
}
#define sysprof_dvfs_cpu_freq_notif(data)                               \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_POWER)) { \
		iowrite32((0x002E0000 | (data & 0xFFFF)),               \
			SYS_PROF_IF_WP[SYSPROF_CORE_ID]);               \
	}                                                               \
}

#define sysprof_set_idle_state(data)                                    \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_POWER)) { \
		iowrite32((0x00002000 | (data & 0x00FF)),               \
			SYS_PROF_IF_WP[SYSPROF_CORE_ID]);               \
	}                                                               \
}
#define sysprof_idle_exit(data)                                         \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_POWER)) { \
		iowrite32((0x00150000 | (data & 0xFFFF)),               \
			SYS_PROF_IF_WP[SYSPROF_CORE_ID]);               \
	}                                                               \
}
#define sysprof_sleep_enter(data)                                       \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_POWER)) { \
		iowrite32((0x00160000 | (data & 0xFFFF)),               \
			SYS_PROF_IF_WP[SYSPROF_CORE_ID]);               \
	}                                                               \
}
#define sysprof_sleep_exit(data)                                        \
{                                                                       \
	if (SYS_PROF_IF_CLASS_MASK & (1 << SYSPROF_EVENT_CLASS_POWER)) { \
		iowrite32((0x00170000 | (data & 0xFFFF)),               \
			SYS_PROF_IF_WP[SYSPROF_CORE_ID]);               \
	}                                                               \
}

#define mv_guest_trace_vmcall_entry(nr, svc, op)	\
{							\
	sysprof_platsvc_entry(svc, op);			\
}

/*
 * Profiling interface main data structure
 */
struct SysProfIf_t {
	/*
	 * Class mask variable used for enable/disable
	 * profiling events generation
	 */
	uint32_t sys_prof_if_class_mask;
	/* Swt ts_offset */
	uint32_t sys_prof_if_swt_ts_offset;
	/* Array with write pointers per core */
	uint32_t __iomem *sys_prof_if_wp[SYSPROF_NOF_LINUX_CORES];
};

extern struct SysProfIf_t sys_prof_if;

#else

#define sysprof_sys_idle()
#define sysprof_sys_active()
#define sysprof_interrupt(line)
#define sysprof_int_enter()
#define sysprof_int_leave()
#define sysprof_task_enter(tid)
#define sysprof_task_leave(tid)
#define sysprof_syscall_enter(scn)
#define sysprof_syscall_leave(scn)
#define sysprof_xirq_to_mex(num)
#define sysprof_xirq_to_linux(num)
#define sysprof_hw_interrupt(line)
#define sysprof_vmexit_start()
#define sysprof_vmentry_end()
#define sysprof_vmcall_entry(nr)
#define sysprof_platsvc_entry(svc, op)
#define sysprof_set_idle_state(data)
#define sysprof_idle_exit(data)
#define sysprof_sleep_enter(data)
#define sysprof_sleep_exit(data)
#define sysprof_bw(name, size)
#define sysprof_dvfs_vcore_load(data)
#define sysprof_dvfs_vcore_load_bal(data)
#define sysprof_dvfs_cpu_freq_notif(data)
#define sysprof_set_idle_state(data)
#define sysprof_idle_exit(data)
#define sysprof_sleep_enter(data)
#define sysprof_sleep_exit(data)
#define mv_guest_trace_vmcall_entry(nr, svc, op)

#endif

#define mv_guest_trace_vmcall_exit()            sysprof_vmentry_end()
#define mv_guest_trace_xirq_post(num)           sysprof_xirq_to_mex(num)
#define mv_guest_trace_ipi_post(num)            sysprof_xirq_to_linux(num)
#define mv_guest_trace_virq_mask(virq)
#define mv_guest_trace_virq_unmask(virq)

#endif

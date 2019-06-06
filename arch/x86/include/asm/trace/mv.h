#undef TRACE_SYSTEM
#define TRACE_SYSTEM mv

#if !defined(_TRACE_MV_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_MV_H

#include <linux/tracepoint.h>

TRACE_EVENT(vmcall_64_entry,

	TP_PROTO(uint64_t nr, uint64_t arg0, uint64_t arg1),

	TP_ARGS(nr, arg0, arg1),

	TP_STRUCT__entry(
		__field(uint64_t, nr)
		__field(uint64_t, arg0)
		__field(uint64_t, arg1)
	),

	TP_fast_assign(
		__entry->nr = nr;
		__entry->arg0 = arg0;
		__entry->arg1 = arg1;
	),

	TP_printk("nr=0x%llx arg0=0x%llx arg1=0x%llx",
		  __entry->nr, __entry->arg0, __entry->arg1)
);

TRACE_EVENT(vmcall_64_exit,

	TP_PROTO(uint64_t nr),

	TP_ARGS(nr),

	TP_STRUCT__entry(
		__field(uint64_t, nr)
	),

	TP_fast_assign(
		__entry->nr = nr;
	),

	TP_printk("nr=0x%llx", __entry->nr)
);

TRACE_EVENT(vmcall_entry,

	TP_PROTO(uint32_t nr, uint32_t arg0, uint32_t arg1),

	TP_ARGS(nr, arg0, arg1),

	TP_STRUCT__entry(
		__field(uint32_t, nr)
		__field(uint32_t, arg0)
		__field(uint32_t, arg1)
	),

	TP_fast_assign(
		__entry->nr = nr;
		__entry->arg0 = arg0;
		__entry->arg1 = arg1;
	),

	TP_printk("nr=0x%x arg0=0x%x arg1=0x%x",
		  __entry->nr, __entry->arg0, __entry->arg1)
);

TRACE_EVENT(vmcall_exit,

	TP_PROTO(uint32_t nr),

	TP_ARGS(nr),

	TP_STRUCT__entry(
		__field(uint32_t, nr)
	),

	TP_fast_assign(
		__entry->nr = nr;
	),

	TP_printk("nr=0x%x", __entry->nr)
);


#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH asm/trace/
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE mv
#endif /* _TRACE_MPX_H */

/* This part must be outside protection */
#include <trace/define_trace.h>

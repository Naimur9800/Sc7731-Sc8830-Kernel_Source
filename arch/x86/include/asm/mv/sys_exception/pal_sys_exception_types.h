/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
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

#ifndef _PAL_SYS_EXCEPTION_TYPES_H
#define _PAL_SYS_EXCEPTION_TYPES_H

#define TRAP_DUMP_DATE_SIZE      11
#define TRAP_DUMP_TIME_SIZE      9
#define TRAP_DUMP_FILENAME_SIZE  128
#define TRAP_MAX_LOG_DATA_SIZE   255

#define TRAP_SW_GENERATED        0xDDDD

#define MAX_VCPU_PER_VM    8
#define MAX_VM  3

enum sys_exception {
	SYS_EXCEPTION_MEX = 0,
	SYS_EXCEPTION_LINUX = 1,
	SYS_EXCEPTION_VMM = 2,
	SYS_EXCEPTION_SECURITY = 3
};

/**
 * Structure for holding internal X86 general purpose registers
 */
struct x86_gp_regs {
	uint64_t rax;
	uint64_t rbx;
	uint64_t rcx;
	uint64_t rdx;
	uint64_t rsi;
	uint64_t rdi;
	uint64_t rbp;
	uint64_t rsp;
	uint64_t r8;
	uint64_t r9;
	uint64_t r10;
	uint64_t r11;
	uint64_t r12;
	uint64_t r13;
	uint64_t r14;
	uint64_t r15;
};

/**
 * Structure for holding internal X86 control registers
 */
struct x86_ctrl_regs {
	uint64_t cr0;
	uint64_t cr2;
	uint64_t cr3;
	uint64_t cr4;
};

/**
 * Structure for holding internal X86 EFLAGS register
 */
struct x86_eflags_reg {
	uint32_t eflags;
};

/**
 * Structure for holding internal X86 segment registers
 */
struct x86_segment_regs {
	uint16_t cs;
	uint16_t ds;
	uint16_t ss;
	uint16_t es;
	uint16_t fs;
	uint16_t gs;
	uint64_t fs_base __aligned(8);
	uint64_t gs_base __aligned(8);
	uint64_t kernel_gs_base __aligned(8);
};

/**
 * Structure for holding internal X86 debug registers
 */
struct x86_debug_regs {
	uint64_t dr0;
	uint64_t dr1;
	uint64_t dr2;
	uint64_t dr3;
	uint64_t dr4;
	uint64_t dr5;
	uint64_t dr6;
	uint64_t dr7;
};

/**
 * Structure for holding internal X86 Memory manager registers
 */
struct x86_mm_regs {
	uint32_t gdtb;
	uint32_t idtb;
	uint32_t ldtb;
	uint32_t tssb;
	uint16_t gdtl;
	uint16_t idtl;
	uint16_t ldtl;
	uint16_t tssl;
	uint16_t tr;
	uint16_t ldtr;
};

/**
 * Structure for holding internal X86 EIP register
 */
struct x86_eip_reg {
	uint64_t rip;
};

struct x86_cpu_regs {
	struct x86_gp_regs       gp_regs __aligned(8);
	struct x86_segment_regs  segment_regs __aligned(8);
	struct x86_eflags_reg    eflags_reg __aligned(8);
	struct x86_eip_reg         eip_reg __aligned(8);
	struct x86_ctrl_regs     ctrl_regs __aligned(8);
	struct x86_mm_regs       mm_regs __aligned(8);
	struct x86_debug_regs    debug_regs __aligned(8);
};

struct sys_trap {
	uint32_t exception_type;
	uint16_t trap_vector;
	char date[TRAP_DUMP_DATE_SIZE];
	char time[TRAP_DUMP_TIME_SIZE];
	char filename[TRAP_DUMP_FILENAME_SIZE];
	uint32_t line;
	char log_data[TRAP_MAX_LOG_DATA_SIZE];
	struct x86_cpu_regs regs;

	/* OS dependent */
	union {
		struct {
			uint64_t kmsg;         /* kernel message */
			uint32_t kmsg_len;  /* kernel message length */
		} linux_log;
		/* other OS dependent log structure to be added here */
	} os;
};

struct sys_vm {
	uint32_t os_id;
	char no_of_vcpu;
	struct x86_cpu_regs vcpu_reg[MAX_VCPU_PER_VM];
};

struct sys_vm_dump {
	char no_of_vm;
	struct sys_vm  vm[MAX_VM];
};

#endif /* _PAL_SYS_EXCEPTION_TYPES_H */

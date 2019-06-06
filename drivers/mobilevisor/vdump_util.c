/*
 * Copyright (c) 2015, Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <asm/mv/mv_gal.h>
#include <asm/mv/mv_svc_hypercalls.h>
#include "asm/mv/mv_hypercalls.h"
#include <asm/mv/sys_exception/pal_sys_exception_types.h>
#include <asm/mv/coredump/pal_coredump_types.h>
#include "vdump_util.h"

static uint64_t convert_to_hex(unsigned char ascii)
{
	if (ascii >= 0x30 && ascii <= 0x39)
		return (ascii & 0x0F);
	if (ascii == 0x41 || ascii == 0x61)
		return 0x0A;
	if (ascii == 0x42 || ascii == 0x62)
		return 0x0B;
	if (ascii == 0x43 || ascii == 0x63)
		return 0x0C;
	if (ascii == 0x44 || ascii == 0x64)
		return 0x0D;
	if (ascii == 0x45 || ascii == 0x65)
		return 0x0E;
	if (ascii == 0x46 || ascii == 0x66)
		return 0x0F;

	return 0;
}

static uint64_t read_hex(const char *address, int bytes)
{
	int i;
	uint64_t hex_value;

	hex_value = 0;
	for (i = 0; i < bytes; i++) {
		hex_value = hex_value<<4;
		hex_value |= convert_to_hex(*((unsigned char *)(address + i)));
	}
	return hex_value;
}

static int search_backward(
	const char *search,
	int length,
	const char *buffer,
	int size)
{
	int i;

	if (search == NULL)
		return -1;

	if (buffer == NULL)
		return -1;

	if (length > size)
		return -1;

	for (i = size-length; i >= 0; i--) {
		if (memcmp(search, buffer+i, length) == 0)
			return i;
	}

	return -1;
}
#ifndef CONFIG_X86_64
static uint32_t get_cs_value(
	int bytes,
	const char *buffer,
	int buffer_size)
{
	int p;
	/* CS value obtain from
	*EIP: 0060:[<80e46ebd>] EFLAGS: 00010246 CPU: 0
	*/
	char cs_string[] = "EIP: ";
	char eip_string[] = "EIP: [<";


	p = search_backward(
			cs_string, sizeof(cs_string)-1, buffer, buffer_size);
	if (p == -1)
		return 0;
	/* continue to search if it is eip_string */
	if (memcmp(eip_string, buffer+p, sizeof(eip_string)-1) == 0) {
		p = search_backward(cs_string, sizeof(cs_string)-1, buffer, p);
		if (p != -1)
			return read_hex(buffer+p+sizeof(cs_string)-1, bytes);
		else
			return 0;

	} else {
		return read_hex(buffer+p+sizeof(cs_string)-1, bytes);
	}

}
#endif
static uint64_t get_reg_value(
	const char *tag,
	int tag_len,
	int offset,
	int bytes,
	const char *buffer,
	int buffer_size)
{
	int p;

	p = search_backward(tag, tag_len, buffer, buffer_size);

	if (p != -1)
		return read_hex(buffer+p+tag_len+offset, bytes);
	else
		return 0;
}

/*
*Search for CPU registers from Linux Kernel Log
*/
#ifdef CONFIG_X86_64
void vdump_save_linux_regs(
	char *kernel_log, int log_size, struct x86_cpu_regs *regs)
{
	int p;

	/* Skip the log by arch/x86/kernel/process_64.c:
	* show_extra_register_data
	*/
	p = search_backward("RIP: 0x", 7, kernel_log, log_size);
	if (p != -1)
		log_size = p;

	/* Refer arch/x86/kernel/process_64.c: __show_regs() */
	regs->gp_regs.rax
		= get_reg_value("RAX: ", 5, 0, 16, kernel_log, log_size);
	regs->gp_regs.rbx
		= get_reg_value("RBX: ", 5, 0, 16, kernel_log, log_size);
	regs->gp_regs.rcx
		= get_reg_value("RCX: ", 5, 0, 16, kernel_log, log_size);
	regs->gp_regs.rdx
		= get_reg_value("RDX: ", 5, 0, 16, kernel_log, log_size);
	regs->gp_regs.rsi
		= get_reg_value("RSI: ", 5, 0, 16, kernel_log, log_size);
	regs->gp_regs.rdi
		= get_reg_value("RDI: ", 5, 0, 16, kernel_log, log_size);
	regs->gp_regs.rbp
		= get_reg_value("RBP: ", 5, 0, 16, kernel_log, log_size);
	/* Kernel log RSP: 0018:ffff8800147c1e78, ss:RSP */
	regs->gp_regs.rsp
		= get_reg_value("RSP: ", 5, 5, 16, kernel_log, log_size);

	regs->gp_regs.r8
		= get_reg_value("R08: ", 5, 0, 16, kernel_log, log_size);
	regs->gp_regs.r9
		= get_reg_value("R09: ", 5, 0, 16, kernel_log, log_size);
	regs->gp_regs.r10
		= get_reg_value("R10: ", 5, 0, 16, kernel_log, log_size);
	regs->gp_regs.r11
		= get_reg_value("R11: ", 5, 0, 16, kernel_log, log_size);
	regs->gp_regs.r12
		= get_reg_value("R12: ", 5, 0, 16, kernel_log, log_size);
	regs->gp_regs.r13
		= get_reg_value("R13: ", 5, 0, 16, kernel_log, log_size);
	regs->gp_regs.r14
		= get_reg_value("R14: ", 5, 0, 16, kernel_log, log_size);
	regs->gp_regs.r15
		= get_reg_value("R15: ", 5, 0, 16, kernel_log, log_size);

	regs->ctrl_regs.cr0
		= get_reg_value("CR0: ", 5, 0, 16, kernel_log, log_size);
	regs->ctrl_regs.cr2
		= get_reg_value("CR2: ", 5, 0, 16, kernel_log, log_size);
	regs->ctrl_regs.cr3
		= get_reg_value("CR3: ", 5, 0, 16, kernel_log, log_size);
	regs->ctrl_regs.cr4
		= get_reg_value("CR4: ", 5, 0, 16, kernel_log, log_size);

	regs->segment_regs.cs
		= get_reg_value("CS:  ", 5, 0, 4, kernel_log, log_size);
	regs->segment_regs.ds
		= get_reg_value("DS: ", 4, 0, 4, kernel_log, log_size);
	/* Kernel log RSP: 0018:ffff8800147c1e78, ss:RSP */
	regs->segment_regs.ss
		= get_reg_value("RSP: ", 5, 0, 4, kernel_log, log_size);
	regs->segment_regs.es
		= get_reg_value("ES: ", 4, 0, 4, kernel_log, log_size);
	/* FS:  0000000000000000(0000) GS:ffff88003fc00000(0063)
	* knlGS:00000000f76f6508
	*/
	regs->segment_regs.fs_base
		= get_reg_value("FS:  ", 5, 0, 16, kernel_log, log_size);
	regs->segment_regs.fs
		= get_reg_value("FS:  ", 5, 17, 4, kernel_log, log_size);
	/* To distinguish knlGS, use " GS:" instead of "GS:" */
	regs->segment_regs.gs_base
		= get_reg_value(" GS:", 4, 0, 16, kernel_log, log_size);
	regs->segment_regs.gs
		= get_reg_value(" GS:", 4, 17, 4, kernel_log, log_size);
	regs->segment_regs.kernel_gs_base
		= get_reg_value("knlGS:", 6, 0, 16, kernel_log, log_size);
	/* Kernel log RIP: 0010:[<ffffffff829c1566>] */
	regs->eip_reg.rip
		= get_reg_value("RIP: ", 5, 7, 16, kernel_log, log_size);

	regs->eflags_reg.eflags
		= get_reg_value("EFLAGS: ", 8, 0, 8, kernel_log, log_size);
}
#else
void vdump_save_linux_regs(
	char *kernel_log, int log_size, struct x86_cpu_regs *regs)
{
	int p;

	/* Skip the log by arch/x86/kernel/process_32.c:
	* show_extra_register_data
	*/
	p = search_backward("EIP: 0x", 7, kernel_log, log_size);
	if (p != -1)
		log_size = p;

	regs->gp_regs.rax
		= get_reg_value("EAX: ", 5, 0, 8, kernel_log, log_size);
	regs->gp_regs.rbx
		= get_reg_value("EBX: ", 5, 0, 8, kernel_log, log_size);
	regs->gp_regs.rcx
		= get_reg_value("ECX: ", 5, 0, 8, kernel_log, log_size);
	regs->gp_regs.rdx
		= get_reg_value("EDX: ", 5, 0, 8, kernel_log, log_size);
	regs->gp_regs.rsi
		= get_reg_value("ESI: ", 5, 0, 8, kernel_log, log_size);
	regs->gp_regs.rdi
		= get_reg_value("EDI: ", 5, 0, 8, kernel_log, log_size);
	regs->gp_regs.rbp
		= get_reg_value("EBP: ", 5, 0, 8, kernel_log, log_size);
	regs->gp_regs.rsp
		= get_reg_value("ESP: ", 5, 0, 8, kernel_log, log_size);

	regs->ctrl_regs.cr0
		= get_reg_value("CR0: ", 5, 0, 8, kernel_log, log_size);
	/* Kernel log CR2 value length is 16 bytes: CR2: 0000000000000000 */
	regs->ctrl_regs.cr2
		= get_reg_value("CR2: ", 5, 8, 8, kernel_log, log_size);
	regs->ctrl_regs.cr3
		= get_reg_value("CR3: ", 5, 0, 8, kernel_log, log_size);
	regs->ctrl_regs.cr4
		= get_reg_value("CR4: ", 5, 0, 8, kernel_log, log_size);

	regs->segment_regs.cs
		= get_cs_value(4, kernel_log, log_size);
	regs->segment_regs.ds
		= get_reg_value("DS: ", 4, 0, 4, kernel_log, log_size);
	regs->segment_regs.ss
		= get_reg_value("SS: ", 4, 0, 4, kernel_log, log_size);
	regs->segment_regs.es
		= get_reg_value("ES: ", 4, 0, 4, kernel_log, log_size);
	regs->segment_regs.fs
		= get_reg_value("FS: ", 4, 0, 4, kernel_log, log_size);
	regs->segment_regs.gs
		= get_reg_value("GS: ", 4, 0, 4, kernel_log, log_size);

	regs->eip_reg.rip
		= get_reg_value("EIP: [<", 7, 0, 8, kernel_log, log_size);

	regs->eflags_reg.eflags
		= get_reg_value("EFLAGS: ", 8, 0, 8, kernel_log, log_size);
}
#endif

/* search for tag in buffer then copy the value after '=' to value */
bool vdump_get_value_by_tag(
	const char *config, const char *tag, char *value, int size)
{
	int i = 0;
	/* search for tag from buffer config */
	char *tv = strstr(config, tag);

	if (value == NULL || size == 0)
		return false;

	/* if tag is found */
	if (tv != NULL && strlen(tv) != 0) {
		bool b_equal_symbol = false;
		/* loop through the buffer before null or CR */
		while (*(++tv) != '\n' && *tv != '\0' && i < size) {
			if (!((*tv >= 'a' && *tv <= 'z') ||
				(*tv >= 'A' && *tv <= 'Z') ||
				(*tv >= '0' && *tv <= '9') ||
				*tv == '_' || *tv == '.' ||
				*tv == '=' || *tv == '/' ||
				*tv == '\\'))
				continue;
			/* copy the value from buffer to value
			*  after '=' is found
			*/
			if (b_equal_symbol)
				value[i++] = *tv;
			/* equal symbol is found */
			if (*tv == '=')
				b_equal_symbol = true;
			}
			value[i++] = '\0';
	}
	return true;
}

int str2int(char *str)
{
	int value = 0;
	char *s = str;

	if (str == NULL || strlen(str) == 0)
		return 0;
	do {
		if (*s > '9' || *s < '0')
			continue;
		value *= 10;
		value += *s - '0';
	} while (*(++s) != 0);
	return value;
}


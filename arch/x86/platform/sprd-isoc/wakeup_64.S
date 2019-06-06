#include <linux/linkage.h>
#include <asm/segment.h>
#include <asm/pgtable_types.h>
#include <asm/page_types.h>
#include <asm/page.h>
#include <asm/msr.h>
#include <asm/asm-offsets.h>
#include <asm/boot.h>
#include <asm/processor-flags.h>

# This file is a merge of arch/x86/kernel/acpi/wakeup_64.S
# and trampoline_64.S except that x86_acpi_enter_sleep_state
# is replaced by isoc_do_sleep. This is to be able to
# go from 32 to 64 after coming from SRAM.

.section ".text32","ax"
.code32
.balign 4

ENTRY(wakeup_start_32)
	# We are landing here, in 32 bit mode coming from
	# SRAM. In SRAM code we trust.

	# Let's be paranoic anyway
	pushl	$0
	popfl
	wbinvd

	xorl	%ebx, %ebx

#ifdef CONFIG_RELOCATABLE

/*
 * Calculate the delta between where we were compiled to run
 * at and where we were actually loaded at.  This can only be done
 * with a short local call on x86.  Nothing  else will tell us what
 * address we are running at.
 */
	call	1f
1:	popl	%ebp
	subl	$(1b - __START_KERNEL_map), %ebp

/*
 * %ebp contains the address we are loaded at by the boot loader and %ebx
 * would contain the  new  base address where the kernel image was relocated
 * if !CONFIG_RELOCATABLE ebx would be 0
 */

	movl	%ebp, %ebx
#endif
	leal	tr_gdt - __START_KERNEL_map(%ebx), %eax
	push	%eax
	pushl	$0x001f0000
	lgdtl	2 (%esp)
	addl	$8, %esp

	xorl	%eax, %eax
	push	%eax
	push	%eax
	lidtl	2 (%esp)
	addl	$8, %esp

	movw	$__KERNEL_DS, %ax
	movw	%ax, %ss
	movw	%ax, %ds
	movw	%ax, %es
	movw	%ax, %fs
	movw	%ax, %gs

	movl	wakeup_tr_misc - __START_KERNEL_map(%ebx), %eax
	movl	(wakeup_tr_misc - __START_KERNEL_map + 4)(%ebx), %edx
	movl	$MSR_IA32_MISC_ENABLE, %ecx
	wrmsr

	movl	%cr4, %eax
	orl	$(1 << 5), %eax
	movl	%eax, %cr4		# Enable PAE mode

	# Setup trampoline 4 level pagetables
	leal	wakeup_trampoline_pgd - __START_KERNEL_map(%ebx), %ecx
	movl	%ecx, %cr3

	movl	wakeup_tr_efer - __START_KERNEL_map(%ebx), %eax
	xor	%edx, %edx
	movl	$MSR_EFER, %ecx
	wrmsr

	# Enable paging and in turn activate Long Mode
	movl	%cr0, %eax
	orl	$X86_CR0_PG, %eax
	movl	%eax, %cr0

	/*
	 * At this point we're in long mode but in 32bit compatibility mode
	 * with EFER.LME = 1, CS.L = 0, CS.D = 1 (and in turn
	 * EFER.LMA = 1). Now we want to jump in 64bit mode, to do that we use
	 * the new gdt/idt that has __KERNEL_CS with CS.L = 1.
	 */
	leal	wakeup_long64 - __START_KERNEL_map(%ebx), %ecx
	push	$__KERNEL_CS
	pushl	%ecx
	retf

.section ".text64","ax"
.code64
.balign 4

	/*
	 * Hooray, we are in Long 64-bit mode (but still running in low memory)
	 */
ENTRY(wakeup_long64_direct)
	xorq	%rbx, %rbx

#ifdef CONFIG_RELOCATABLE

/*
 * Calculate the delta between where we were compiled to run
 * at and where we were actually loaded at.  This can only be done
 * with a short local call on x86.  Nothing  else will tell us what
 * address we are running at.
 */
	call	1f
1:	pop	%rbp
	subq	$(1b - __START_KERNEL_map), %rbp

/*
 * %ebp contains the address we are loaded at by the boot loader and %ebx
 * would contain the  new  base address where the kernel image was relocated
 * if !CONFIG_RELOCATABLE ebx would be 0
 */

	movq	%rbp, %rbx
#endif

	movq	wakeup_tr_misc - __START_KERNEL_map(%rbx), %rax
	movq	(wakeup_tr_misc - __START_KERNEL_map + 4)(%rbx), %rdx
	movq	$MSR_IA32_MISC_ENABLE, %rcx
	wrmsr

	movq	wakeup_tr_efer - __START_KERNEL_map(%rbx), %rax
	xorq	%rdx, %rdx
	movq	$MSR_EFER, %rcx
	wrmsr

	leaq	wakeup_long64 - __START_KERNEL_map(%rbx), %rcx
	push	$__KERNEL_CS
	push	%rcx
	retfq
ENDPROC(wakeup_long64_direct)

ENTRY(wakeup_long64)
	movw	$__KERNEL_DS, %ax
	movw	%ax, %ss
	movw	%ax, %ds
	movw	%ax, %es
	movw	%ax, %fs
	movw	%ax, %gs
	movq	saved_rsp, %rsp

	movq	saved_rbx, %rbx
	movq	saved_rdi, %rdi
	movq	saved_rsi, %rsi
	movq	saved_rbp, %rbp

	movq	saved_rip, %rax
	jmp	*%rax
ENDPROC(wakeup_long64)

ENTRY(do_suspend_lowlevel)
	subq	$8, %rsp
	xorl	%eax, %eax
	call	save_processor_state

	movq	$saved_context, %rax
	movq	%rsp, pt_regs_sp(%rax)
	movq	%rbp, pt_regs_bp(%rax)
	movq	%rsi, pt_regs_si(%rax)
	movq	%rdi, pt_regs_di(%rax)
	movq	%rbx, pt_regs_bx(%rax)
	movq	%rcx, pt_regs_cx(%rax)
	movq	%rdx, pt_regs_dx(%rax)
	movq	%r8, pt_regs_r8(%rax)
	movq	%r9, pt_regs_r9(%rax)
	movq	%r10, pt_regs_r10(%rax)
	movq	%r11, pt_regs_r11(%rax)
	movq	%r12, pt_regs_r12(%rax)
	movq	%r13, pt_regs_r13(%rax)
	movq	%r14, pt_regs_r14(%rax)
	movq	%r15, pt_regs_r15(%rax)
	pushfq
	popq	pt_regs_flags(%rax)

	movq	$resume_point, saved_rip(%rip)

	movq	%rsp, saved_rsp
	movq	%rbp, saved_rbp
	movq	%rbx, saved_rbx
	movq	%rdi, saved_rdi
	movq	%rsi, saved_rsi

	addq	$8, %rsp
	movl	$3, %edi /* PM_SUSPEND_MEM */
	xorl	%eax, %eax
	/* call	x86_acpi_enter_sleep_state */
	call	isoc_do_sleep
	/* in case something went wrong, restore the machine status and go on */
	jmp	resume_point
	ret
ENDPROC(do_suspend_lowlevel)

.align 4
ENTRY(resume_point)
	/* We don't restore %rax, it must be 0 anyway */
	movq	$saved_context, %rax
	movq	saved_context_cr4(%rax), %rbx
	movq	%rbx, %cr4
	movq	saved_context_cr3(%rax), %rbx
	movq	%rbx, %cr3
	movq	saved_context_cr2(%rax), %rbx
	movq	%rbx, %cr2
	movq	saved_context_cr0(%rax), %rbx
	movq	%rbx, %cr0

	lgdtq	saved_context_gdt_desc(%rax)

	pushq	pt_regs_flags(%rax)
	popfq
	movq	pt_regs_sp(%rax), %rsp
	movq	pt_regs_bp(%rax), %rbp
	movq	pt_regs_si(%rax), %rsi
	movq	pt_regs_di(%rax), %rdi
	movq	pt_regs_bx(%rax), %rbx
	movq	pt_regs_cx(%rax), %rcx
	movq	pt_regs_dx(%rax), %rdx
	movq	pt_regs_r8(%rax), %r8
	movq	pt_regs_r9(%rax), %r9
	movq	pt_regs_r10(%rax), %r10
	movq	pt_regs_r11(%rax), %r11
	movq	pt_regs_r12(%rax), %r12
	movq	pt_regs_r13(%rax), %r13
	movq	pt_regs_r14(%rax), %r14
	movq	pt_regs_r15(%rax), %r15

	xorl	%eax, %eax
	addq	$8, %rsp
	jmp	restore_processor_state
ENDPROC(resume_point)

.data
ALIGN

tr_gdt:
	.quad	0
	.quad	0x00cf9b000000ffff	# __KERNEL32_CS
	.quad	0x00af9b000000ffff	# __KERNEL_CS
	.quad	0x00cf93000000ffff	# __KERNEL_DS
tr_gdt_end:

idt_descr:
        .word 0           # idt contains 0 entries
        .long 0

ENTRY(saved_rbp)	.quad	0
ENTRY(saved_rsi)	.quad	0
ENTRY(saved_rdi)	.quad	0
ENTRY(saved_rbx)	.quad	0
ENTRY(saved_rip)	.quad	0
ENTRY(saved_rsp)	.quad	0

.balign	8
GLOBAL(wakeup_trampoline_header)
	wakeup_tr_start:	.space	8
	GLOBAL(wakeup_tr_efer)	.space	8
	GLOBAL(wakeup_tr_misc)	.space	8
	GLOBAL(wakeup_tr_cr4)	.space	4
END(wakeup_trampoline_header)

.balign	PAGE_SIZE
GLOBAL(wakeup_trampoline_pgd)	.space	PAGE_SIZE*6, 0

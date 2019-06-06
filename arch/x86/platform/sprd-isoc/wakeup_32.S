.text
#include <linux/linkage.h>
#include <asm/segment.h>
#include <asm/page_types.h>

# This file is a entire copy of arch/x86/kernel/acpi/wakeup_32.S except
# that x86_acpi_enter_sleep_state is replaced by isoc_do_sleep

.code32
ALIGN

ENTRY(wakeup_start_32)
	# We are landing here, in 32 bit mode coming from
	# SRAM. In SRAM code we trust.

	# Let's be paranoic anyway
	pushl	$0
	popfl
	cli
	wbinvd

	# Set up %cr3
	movl saved_cr3, %eax
	movl %eax, %cr3

	movl saved_cr4, %eax
	movl %eax, %cr4

        # Control registers were modified, pipeline resync is needed
        jmp 1f
1:
	movw	$__KERNEL_DS, %ax
	movw	%ax, %ss
	movw	%ax, %ds
	movw	%ax, %es
	movw	%ax, %fs
	movw	%ax, %gs

	# reload the gdt, as we need the full 32 bit address
	lidt	saved_idt
	lgdt	saved_gdt
	lldt    saved_ldt
	ljmp	$(__KERNEL_CS), $1f
1:
	movl	%cr3, %eax
	movl	%eax, %cr3

	movl saved_cr0, %eax
	movl %eax, %cr0

	wbinvd

	# and restore the stack ... but you need gdt for this to work
	movl	saved_context_esp, %esp

	# jump to place where we left off
	jmp	ret_point

save_registers:
	sidt	saved_idt
	sldt	saved_ldt
	str	saved_tss

	pushl %edx

	movl %cr0, %edx
	movl %edx, saved_cr0

	movl %cr3, %edx
	movl %edx, saved_cr3

	movl %cr4, %edx
	movl %edx, saved_cr4

	popl %edx

	leal	4(%esp), %eax
	movl	%eax, saved_context_esp
	movl	%ebx, saved_context_ebx
	movl	%ebp, saved_context_ebp
	movl	%esi, saved_context_esi
	movl	%edi, saved_context_edi
	pushfl
	popl	saved_context_eflags

	ret

restore_registers:
	movl	saved_context_ebp, %ebp
	movl	saved_context_ebx, %ebx
	movl	saved_context_esi, %esi
	movl	saved_context_edi, %edi
	pushl	saved_context_eflags
	popfl
	ret

ENTRY(do_suspend_lowlevel)
	call	save_processor_state
	call	save_registers
	pushl	$3
	call	isoc_do_sleep
	addl	$4, %esp

#	In case of S3 failure, we'll emerge here.  Jump
# 	to ret_point to recover
	jmp	ret_point
	.p2align 4,,7
ret_point:
	call	restore_registers
	call	restore_processor_state
	ret

.data
ALIGN

saved_gdt:             .long   0,0
saved_idt:             .long   0,0
saved_ldt:             .long   0
saved_tss:	       .long   0
saved_cr0:             .long   0
saved_cr3:             .long   0
saved_cr4:             .long   0
saved_context_esp:     .long   0
saved_context_edi:     .long   0
saved_context_esi:     .long   0
saved_context_ebx:     .long   0
saved_context_ebp:     .long   0
saved_context_eflags:  .long   0

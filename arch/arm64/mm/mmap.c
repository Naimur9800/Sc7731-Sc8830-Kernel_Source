/*
 * Based on arch/arm/mm/mmap.c
 *
 * Copyright (C) 2012 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/elf.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/export.h>
#include <linux/shm.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/personality.h>
#include <linux/random.h>
#include <linux/security.h>

#include <asm/cputype.h>

/*
 * Leave enough space between the mmap area and the stack to honour ulimit in
 * the face of randomisation.
 */
#define MIN_GAP (SZ_128M + ((STACK_RND_MASK << PAGE_SHIFT) + 1))
#define MAX_GAP	(STACK_TOP/6*5)

/* guest supposed to be AARCH32 */
#define STACK_TOP_GUEST 0xffff0000
#define MAX_GAP_GUEST (STACK_TOP_GUEST/6*5)

static int mmap_is_legacy(void)
{
	if (current->personality & ADDR_COMPAT_LAYOUT)
		return 1;

	if (rlimit(RLIMIT_STACK) == RLIM_INFINITY)
		return 1;

	return sysctl_legacy_va_layout;
}

unsigned long arch_mmap_rnd(void)
{
	unsigned long rnd;

#ifdef CONFIG_COMPAT
	if (test_thread_flag(TIF_32BIT))
		rnd = get_random_long() & ((1UL << mmap_rnd_compat_bits) - 1);
	else
#endif
		rnd = get_random_long() & ((1UL << mmap_rnd_bits) - 1);
	return rnd << PAGE_SHIFT;
}

static unsigned long mmap_base(unsigned long rnd)
{
	unsigned long gap = rlimit(RLIMIT_STACK);

	if (gap < MIN_GAP)
		gap = MIN_GAP;
	else if (gap > MAX_GAP)
		gap = MAX_GAP;

	return PAGE_ALIGN(STACK_TOP - gap - rnd);
}

static unsigned long mmap_base_guest(unsigned long rnd)
{
	unsigned long gap = rlimit(RLIMIT_STACK);

	if (gap < MIN_GAP)
		gap = MIN_GAP;
	else if (gap > MAX_GAP_GUEST)
		gap = MAX_GAP_GUEST;

	return PAGE_ALIGN(STACK_TOP_GUEST - gap - rnd);
}

unsigned long
arch_get_unmapped_area(struct file *filp, unsigned long addr,
		unsigned long len, unsigned long pgoff, unsigned long flags)
{
	struct mm_struct *mm = current->mm;
	struct vm_area_struct *vma;
	struct vm_unmapped_area_info info;

	if (len > TASK_SIZE - mmap_min_addr)
		return -ENOMEM;

	/* ensure that translated processes do not allocate the last page of
	 * the 32-bit address space
	 */
	if (addr || (flags & MAP_FIXED)) {
		if (current_thread_info()->dbt_syscall &&
		    addr + len > TASK_SIZE_32 - PAGE_SIZE)
			return -ENOMEM;
	}

	if (flags & MAP_FIXED)
		return addr;

	if (addr) {
		addr = PAGE_ALIGN(addr);
		vma = find_vma(mm, addr);
		if (TASK_SIZE - len >= addr && addr >= mmap_min_addr &&
		    (!vma || addr + len <= vma->vm_start))
			return addr;
	}

	info.flags = 0;
	info.length = len;
	info.low_limit = max(PAGE_SIZE, mmap_min_addr);
	if (current_thread_info()->dbt_syscall) {
		info.low_limit = mm->mmap_guest_base;
		info.high_limit = TASK_SIZE_32 - PAGE_SIZE;
	} else {
		info.low_limit = mm->mmap_base;
		info.high_limit = TASK_SIZE;
	}
	info.align_mask = 0;
	return vm_unmapped_area(&info);
}

unsigned long
arch_get_unmapped_area_topdown(struct file *filp, const unsigned long addr0,
			  const unsigned long len, const unsigned long pgoff,
			  const unsigned long flags)
{
	struct vm_area_struct *vma;
	struct mm_struct *mm = current->mm;
	unsigned long addr = addr0;
	struct vm_unmapped_area_info info;

	/* requested length too big for entire address space */
	if (len > TASK_SIZE - mmap_min_addr)
		return -ENOMEM;

	/* ensure that translated processes do not allocate the last page of
	 * the 32-bit address space
	 */
	if (addr || (flags & MAP_FIXED)) {
		if (current_thread_info()->dbt_syscall &&
		    addr + len > TASK_SIZE_32 - PAGE_SIZE)
			return -ENOMEM;
	}

	if (flags & MAP_FIXED)
		return addr;

	/* requesting a specific address */
	if (addr) {
		addr = PAGE_ALIGN(addr);
		vma = find_vma(mm, addr);
		if (TASK_SIZE - len >= addr && addr >= mmap_min_addr &&
		    (!vma || addr + len <= vma->vm_start))
			return addr;
	}

	info.flags = VM_UNMAPPED_AREA_TOPDOWN;
	info.length = len;
	info.low_limit = max(PAGE_SIZE, mmap_min_addr);
	if (current_thread_info()->dbt_syscall)
		info.high_limit = mm->mmap_guest_base;
	else
		info.high_limit = mm->mmap_base;
	info.align_mask = 0;
	addr = vm_unmapped_area(&info);

	/*
	 * A failed mmap() very likely causes application failure,
	 * so fall back to the bottom-up function here. This scenario
	 * can happen with large stack limits and large mmap()
	 * allocations.
	 */
	if (offset_in_page(addr)) {
		VM_BUG_ON(addr != -ENOMEM);
		info.flags = 0;
		if (current_thread_info()->dbt_syscall) {
			info.high_limit = TASK_SIZE_32 - PAGE_SIZE;
			info.low_limit = PAGE_ALIGN(TASK_SIZE_32/4);
		} else {
			info.low_limit = TASK_UNMAPPED_BASE;
			info.high_limit = TASK_SIZE;
		}
		addr = vm_unmapped_area(&info);
	}

	return addr;
}

/*
 * This function, called very early during the creation of a new process VM
 * image, sets up which VM layout function to use:
 */
void arch_pick_mmap_layout(struct mm_struct *mm)
{
	unsigned long random_factor = 0UL;
	unsigned long random_factor_guest = 0UL;

	if (current->flags & PF_RANDOMIZE) {
		random_factor = arch_mmap_rnd();
		random_factor_guest = (get_random_long() &
			((1UL << mmap_rnd_compat_bits) - 1)) << PAGE_SHIFT;
	}

	/*
	 * Fall back to the standard layout if the personality bit is set, or
	 * if the expected stack growth is unlimited:
	 */
	if (mmap_is_legacy()) {
		mm->mmap_base = TASK_UNMAPPED_BASE + random_factor;
		mm->mmap_guest_base = PAGE_ALIGN(TASK_SIZE_32/4) +
			random_factor_guest;
		mm->get_unmapped_area = arch_get_unmapped_area;
	} else {
		mm->mmap_base = mmap_base(random_factor);
		mm->mmap_guest_base = mmap_base_guest(random_factor_guest);
		mm->get_unmapped_area = arch_get_unmapped_area_topdown;
	}
}
EXPORT_SYMBOL_GPL(arch_pick_mmap_layout);


/*
 * You really shouldn't be using read() or write() on /dev/mem.  This might go
 * away in the future.
 */
int valid_phys_addr_range(phys_addr_t addr, size_t size)
{
	if (addr < PHYS_OFFSET)
		return 0;
	if (addr + size > __pa(high_memory - 1) + 1)
		return 0;

	return 1;
}

/*
 * Do not allow /dev/mem mappings beyond the supported physical range.
 */
int valid_mmap_phys_addr_range(unsigned long pfn, size_t size)
{
	return !(((pfn << PAGE_SHIFT) + size) & ~PHYS_MASK);
}

#ifdef CONFIG_STRICT_DEVMEM

#include <linux/ioport.h>

/*
 * devmem_is_allowed() checks to see if /dev/mem access to a certain address
 * is valid. The argument is a physical page number.  We mimic x86 here by
 * disallowing access to system RAM as well as device-exclusive MMIO regions.
 * This effectively disable read()/write() on /dev/mem.
 */
int devmem_is_allowed(unsigned long pfn)
{
	if (iomem_is_exclusive(pfn << PAGE_SHIFT))
		return 0;
	if (!page_is_ram(pfn))
		return 1;
	return 0;
}

#endif

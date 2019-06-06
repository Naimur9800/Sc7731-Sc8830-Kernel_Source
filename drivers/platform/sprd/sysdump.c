/*
 * Copyright (C) 2013 Spreadtrum Communications Inc.
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

#include <asm/cacheflush.h>
#include <asm/memory.h>
#include <linux/delay.h>
#include <linux/elf.h>
#include <linux/elfcore.h>
#include <linux/highuid.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/kallsyms.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/proc_fs.h>
#include <linux/reboot.h>
#include <linux/rtc.h>
#include <linux/sched.h>
#include <linux/sysctl.h>
#include <linux/sysrq.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/crc32.h>
#include "sysdump.h"

#define CORE_STR	"CORE"
#ifndef ELF_CORE_EFLAGS
#define ELF_CORE_EFLAGS	0
#endif

#define SYSDUMP_MAGIC	"SPRD_SYSDUMP_119"

#define SYSDUMP_NOTE_BYTES (ALIGN(sizeof(struct elf_note), 4) +   \
			    ALIGN(sizeof(CORE_STR), 4) + \
			    ALIGN(sizeof(struct elf_prstatus), 4))
#define MAX_NUM_DUMP_MEM 20

#define ALIGN_SIZE 0X100000
#define ROUND_UP(x)       ((x + ALIGN_SIZE - 1) & ~(ALIGN_SIZE - 1))

typedef char note_buf_t[SYSDUMP_NOTE_BYTES];

note_buf_t __percpu *crash_notes;

/* An ELF note in memory */
struct memelfnote {
	const char *name;
	int type;
	unsigned int datasz;
	void *data;
};

struct sysdump_info {
	char magic[16];
	char time[32];
	char dump_path[128];
	int elfhdr_size;
	int mem_num;
	unsigned long dump_mem_paddr;
	unsigned long last_kmsg_paddr;
	int crash_key;
	struct crash_info crash;
};

struct sysdump_extra {
	int enter_id;
	int enter_cpu;
	char reason[256];
	struct pt_regs cpu_context[CONFIG_NR_CPUS];
};

struct sysdump_config {
	int enable;
	int crashkey_only;
	int dump_modem;
	int reboot;
	char dump_path[128];
};

static struct sysdump_node sprd_last_kmsg[] = {
	{
		.name = "__log_buf",
		.size = 1 << CONFIG_LOG_BUF_SHIFT,
	},

	{
		.name = "log_next_idx",
		.size = 0,
	},

};

static struct sysdump_info *sprd_sysdump_info;
static unsigned long sysdump_magic_paddr;
static unsigned long sysdump_info_size;

/* must be global to let gdb know */
struct sysdump_extra sprd_sysdump_extra = {
	.enter_id = -1,
	.enter_cpu = -1,
	.reason = {0},
};

static struct sysdump_config sysdump_conf = {
	.enable = 1,
	.crashkey_only = 0,
	.dump_modem = 1,
	.reboot = 1,
	.dump_path = "",
};

static struct sysdump_mem sprd_dump_mem[MAX_NUM_DUMP_MEM];
static int sprd_dump_mem_num;
static int sprd_last_kmsg_num = ARRAY_SIZE(sprd_last_kmsg);

int sysdump_status;
extern void sysdump_enable_watchdog(int on);
extern void sprd_set_reboot_mode(const char *cmd);


void sprd_debug_check_crash_key(unsigned int code, int value)
{
	static bool volup_p;
	static bool voldown_p;
	static int loopcount;
	static int vol_pressed;

	if (value) {
		if (code == KEY_VOLUMEUP)
			volup_p = true;
		if (code == KEY_VOLUMEDOWN)
			voldown_p = true;
		if ((!vol_pressed) && volup_p && voldown_p)
			vol_pressed = jiffies_to_msecs(jiffies);
		if (volup_p && voldown_p) {
			static unsigned long stack_dump_jiffies = 0;

			if (!stack_dump_jiffies || jiffies > stack_dump_jiffies + HZ * 10) {
				pr_info("trigger sysrq w & m\n");

				handle_sysrq('w');

				handle_sysrq('m');
				stack_dump_jiffies = jiffies;
			}

			if (code == KEY_POWER) {
				pr_info("%s: Crash key count : %d\n", __func__, ++loopcount);

				if ((jiffies_to_msecs(jiffies) - vol_pressed) <= 5000) {
					if (loopcount == 2)
						panic("Crash Key");
					} else {
						volup_p = false;
						voldown_p = false;
						loopcount = 0;
						vol_pressed = 0;
					}
			}
		}
	} else {
		if (code == KEY_VOLUMEUP)
			volup_p = false;
		if (code == KEY_VOLUMEDOWN) {
			loopcount = 0;
			voldown_p = false;
		}
	}
}

static size_t get_elfhdr_size(int nphdr)
{
	size_t elfhdr_len;

	elfhdr_len = sizeof(struct elfhdr) +
	    (nphdr + 1) * sizeof(struct elf_phdr) +
	    ((sizeof(struct elf_note)) * 3 +
	     roundup(sizeof(CORE_STR), 4)) +
	    roundup(sizeof(struct elf_prstatus), 4) +
	    roundup(sizeof(struct elf_prpsinfo), 4) +
	    roundup(sizeof(struct task_struct), 4);
	elfhdr_len = PAGE_ALIGN(elfhdr_len);

	return elfhdr_len;
}

static char *storenote(struct memelfnote *men, char *bufp)
{
	struct elf_note en;

#define DUMP_WRITE(addr, nr) do {memcpy(bufp, addr, nr); bufp += nr; } while (0)

	en.n_namesz = strlen(men->name) + 1;
	en.n_descsz = men->datasz;
	en.n_type = men->type;

	DUMP_WRITE(&en, sizeof(en));
	DUMP_WRITE(men->name, en.n_namesz);

	/* XXX - cast from long long to long to avoid need for libgcc.a */
	bufp = (char *)roundup((unsigned long)bufp, 4);
	DUMP_WRITE(men->data, men->datasz);
	bufp = (char *)roundup((unsigned long)bufp, 4);

#undef DUMP_WRITE

	return bufp;
}

/*
 * fill up all the fields in prstatus from the given task struct, except
 * registers which need to be filled up separately.
 */
static void fill_prstatus(struct elf_prstatus *prstatus,
			  struct task_struct *p, long signr)
{
	prstatus->pr_info.si_signo = prstatus->pr_cursig = signr;
	prstatus->pr_sigpend = p->pending.signal.sig[0];
	prstatus->pr_sighold = p->blocked.sig[0];
	rcu_read_lock();
	prstatus->pr_ppid = task_pid_vnr(rcu_dereference(p->real_parent));
	rcu_read_unlock();
	prstatus->pr_pid = task_pid_vnr(p);
	prstatus->pr_pgrp = task_pgrp_vnr(p);
	prstatus->pr_sid = task_session_vnr(p);
	if (0 /* thread_group_leader(p) */ ) {
		struct task_cputime cputime;

		/*
		 * This is the record for the group leader.  It shows the
		 * group-wide total, not its individual thread total.
		 */
		/* thread_group_cputime(p, &cputime); */
		cputime_to_timeval(cputime.utime, &prstatus->pr_utime);
		cputime_to_timeval(cputime.stime, &prstatus->pr_stime);
	} else {
		cputime_to_timeval(p->utime, &prstatus->pr_utime);
		cputime_to_timeval(p->stime, &prstatus->pr_stime);
	}
	cputime_to_timeval(p->signal->cutime, &prstatus->pr_cutime);
	cputime_to_timeval(p->signal->cstime, &prstatus->pr_cstime);
}

void crash_note_save_cpu(struct pt_regs *regs, int cpu)
{
	struct elf_prstatus prstatus;
	struct memelfnote notes;

	notes.name = CORE_STR;
	notes.type = NT_PRSTATUS;
	notes.datasz = sizeof(struct elf_prstatus);
	notes.data = &prstatus;

	memset(&prstatus, 0, sizeof(struct elf_prstatus));
	fill_prstatus(&prstatus, current, 0);
	memcpy(&prstatus.pr_reg, regs, sizeof(struct pt_regs));
	storenote(&notes, (char *)per_cpu_ptr(crash_notes, cpu));
}

static void sysdump_fill_core_hdr(struct pt_regs *regs,
				  struct sysdump_mem *sysmem, int mem_num,
				  char *bufp, int nphdr, int dataoff)
{
	struct elf_phdr *nhdr, *phdr;
	struct elfhdr *elf;
	off_t offset = 0;
	int i;
	Elf_Off f_offset = dataoff;

	/* setup ELF header */
	elf = (struct elfhdr *)bufp;
	bufp += sizeof(struct elfhdr);
	offset += sizeof(struct elfhdr);
	memcpy(elf->e_ident, ELFMAG, SELFMAG);
	elf->e_ident[EI_CLASS] = ELF_CLASS;
	elf->e_ident[EI_DATA] = ELF_DATA;
	elf->e_ident[EI_VERSION] = EV_CURRENT;
	elf->e_ident[EI_OSABI] = ELF_OSABI;
	memset(elf->e_ident + EI_PAD, 0, EI_NIDENT - EI_PAD);
	elf->e_type = ET_CORE;
	elf->e_machine = ELF_ARCH;
	elf->e_version = EV_CURRENT;
	elf->e_entry = 0;
	elf->e_phoff = sizeof(struct elfhdr);
	elf->e_shoff = 0;
	elf->e_flags = ELF_CORE_EFLAGS;
	elf->e_ehsize = sizeof(struct elfhdr);
	elf->e_phentsize = sizeof(struct elf_phdr);
	elf->e_phnum = nphdr;
	elf->e_shentsize = 0;
	elf->e_shnum = 0;
	elf->e_shstrndx = 0;

	/* setup ELF PT_NOTE program header */
	nhdr = (struct elf_phdr *)bufp;
	bufp += sizeof(struct elf_phdr);
	offset += sizeof(struct elf_phdr);
	nhdr->p_type = PT_NOTE;
	nhdr->p_offset = 0;
	nhdr->p_vaddr = 0;
	nhdr->p_paddr = 0;
	nhdr->p_filesz = 0;
	nhdr->p_memsz = SYSDUMP_NOTE_BYTES * NR_CPUS;
	nhdr->p_flags = 0;
	nhdr->p_align = 0;

	/* setup ELF PT_LOAD program header for every area */
	for (i = 0; i < mem_num; i++) {
		phdr = (struct elf_phdr *)bufp;
		bufp += sizeof(struct elf_phdr);
		offset += sizeof(struct elf_phdr);

		phdr->p_type = PT_LOAD;
		phdr->p_flags = PF_R | PF_W | PF_X;
		phdr->p_offset = f_offset;
		phdr->p_vaddr = sysmem[i].vaddr;
		phdr->p_paddr = sysmem[i].paddr;
		phdr->p_filesz = phdr->p_memsz = sysmem[i].size;
		phdr->p_align = 0;	/*PAGE_SIZE; */
		f_offset += sysmem[i].size;
	}

	/*
	 * Set up the notes in similar form to SVR4 core dumps made
	 * with info from their /proc.
	 */
	nhdr->p_offset = offset;
	nhdr->p_filesz = 0;

	return;
}/* end elf_kcore_store_hdr() */

static void get_sprd_sysdump_info_paddr(void)
{
	struct device_node *node = NULL;
	struct device_node *node_mem = NULL;
	unsigned long *reg = NULL, *reg_end = NULL;
	unsigned long *magic_addr = NULL, *dump_version = NULL;
	int len, sw, aw, version;

	node = of_find_node_by_name(NULL, "sprd_sysdump");
	/*node = of_find_compatible_node(NULL, NULL, "sprd,sysdump"); */
	if (!node) {
		pr_emerg("Not find sprd_sysdump node from dts, no magic-addr property\n");
		sysdump_magic_paddr = SPRD_SYSDUMP_MAGIC;
		return;
	} else
		pr_emerg("node->name is %s\n", node->name);

	aw = of_n_addr_cells(node);
	sw = of_n_size_cells(node);

	dump_version = (unsigned long *)of_get_property(node, "version", &len);
	if (!dump_version) {
		pr_emerg("Not find version property from sprd_sysdump node!\n");
		version = 0x1;
	} else
		version = of_read_ulong((const __be32 *)dump_version, sw);

	pr_emerg("dump_version is 0x%x\n", version);

	magic_addr = (unsigned long *)of_get_property(node, "magic-addr", &len);
	if (!magic_addr) {
		pr_emerg("Not find magic-addr property from sprd_sysdump node!\n");
		if (version == 0x1)
			sysdump_magic_paddr = SPRD_SYSDUMP_MAGIC;
		else
			return;
	} else {
		if (version == 0x1)
			sysdump_magic_paddr = of_read_ulong((const __be32 *)magic_addr, aw);
		else {
			node_mem = of_find_node_by_path((const char *)magic_addr);
			reg = (unsigned long *)of_get_property(node_mem, "reg", &len);
			reg_end = reg + len / (sizeof(unsigned long));
			while (reg < reg_end) {
				sysdump_magic_paddr = of_read_ulong((const __be32 *)reg, aw);
				reg++;
				/*sysdump-reserved-memory size*/
				sysdump_info_size = of_read_ulong((const __be32 *)reg, sw);
				reg++;
			}
		}
	}
	pr_emerg("magic_addr is 0x%lx\n", sysdump_magic_paddr);
	pr_emerg("sysdump_info_size is 0x%lx\n", sysdump_info_size);
}

static void fill_sprd_sysdump_log(void)
{
	int i = 0;
	unsigned int crc32 = 0;

	for (i = 0; i < sprd_last_kmsg_num; i++) {
		sprd_last_kmsg[i].vaddr = kallsyms_lookup_name(sprd_last_kmsg[i].name);
		sprd_last_kmsg[i].paddr = (long)__pa(sprd_last_kmsg[i].vaddr);
#if 0
		crc32 = crc32(0, &sprd_last_kmsg[i].paddr, sizeof(sprd_last_kmsg[i].paddr));
		sprd_last_kmsg[i].crc32 = crc32(crc32, &sprd_last_kmsg[i].size,
										sizeof(sprd_last_kmsg[i].size));
#endif
	}
}

#if 0
static void sprd_sysdump_crc32(struct dump_collection_crc32 *collections)
{
	unsigned int crc32_log = 0;
	crc32_log = crc32(0, &collections->last_kmsg_paddr,
						sizeof(collections->last_kmsg_paddr));
	collections->crc32 = crc32_log;
}
#endif

static void fill_sprd_sysdump_mem(void)
{
	struct device_node *node = NULL;
	struct device_node *node_mem = NULL;
	unsigned long *reg = NULL, *reg_end = NULL;
	int len, sw, aw, i = 0;

	node = of_find_node_by_name(NULL, "sprd_sysdump");
	if (!node) {
		pr_emerg("not find sprd_sysdump node from dts\n");
		return;
	}
	else
		pr_emerg("node->name is %s\n", node->name);

	aw = of_n_addr_cells(node);
	sw = of_n_size_cells(node);

	reg = (unsigned long *)of_get_property(node, "ram", &len);
	if (reg == NULL)
		pr_emerg("no ram property.\n");
	else {
		node_mem = of_find_node_by_path((const char *)reg);
		reg = (unsigned long *)of_get_property(node_mem, "reg", &len);
		reg_end = reg + len / (sizeof(unsigned long));
		while (reg < reg_end) {
			sprd_dump_mem[i].paddr =
			    of_read_ulong((const __be32 *)reg, aw);
			reg++;
			sprd_dump_mem[i].size =
			    of_read_ulong((const __be32 *)reg, sw);
			sprd_dump_mem[i].size = ROUND_UP(sprd_dump_mem[i].size);
			reg++;
			sprd_dump_mem[i].soff = 0xffffffff;
			sprd_dump_mem[i].vaddr =
			    (long)__va(sprd_dump_mem[i].paddr);
			sprd_dump_mem[i].type = SYSDUMP_RAM;
			pr_emerg("sprd_dump_mem[%d].paddr is0x%lx\n", i,
				 sprd_dump_mem[i].paddr);
			pr_emerg("sprd_dump_mem[%d].size  is0x%lx\n", i,
				 sprd_dump_mem[i].size);
			i++;
		}
	}
	reg = (unsigned long *)of_get_property(node, "modem", &len);
	if (reg == NULL)
		pr_emerg("no modem property.\n");
	else {
		reg_end = reg + len / (sizeof(unsigned long));
		while (reg < reg_end) {
			sprd_dump_mem[i].paddr =
			    of_read_ulong((const __be32 *)reg, aw);
			reg++;
			sprd_dump_mem[i].size =
			    of_read_ulong((const __be32 *)reg, sw);
			reg++;
			sprd_dump_mem[i].soff = 0xffffffff;
			sprd_dump_mem[i].vaddr =
			    (long)__va(sprd_dump_mem[i].paddr);
			sprd_dump_mem[i].type = SYSDUMP_MODEM;
			pr_emerg("sprd_dump_mem[%d].paddr is0x%lx\n", i,
				 sprd_dump_mem[i].paddr);
			pr_emerg("sprd_dump_mem[%d].size  is0x%lx\n", i,
				 sprd_dump_mem[i].size);
			i++;
			if (!sysdump_conf.dump_modem)
				sprd_dump_mem[i].size = 0;
		}
	}

	reg = (unsigned long *)of_get_property(node, "iomem", &len);
	if (reg == NULL)
		pr_emerg("no iomem property.\n");
	else {
		reg_end = reg + len / (sizeof(unsigned long));
		while (reg < reg_end) {
			sprd_dump_mem[i].paddr =
			    of_read_ulong((const __be32 *)reg, aw);
			reg++;
			sprd_dump_mem[i].size =
			    of_read_ulong((const __be32 *)reg, sw);
			reg++;
			sprd_dump_mem[i].soff = 0;
			sprd_dump_mem[i].vaddr =
			    (unsigned long)ioremap(sprd_dump_mem[i].paddr,
						   sprd_dump_mem[i].size);
			sprd_dump_mem[i].type = SYSDUMP_IOMEM;
			pr_emerg("sprd_dump_mem[%d].paddr is0x%lx\n", i,
				 sprd_dump_mem[i].paddr);
			pr_emerg("sprd_dump_mem[%d].vaddr is0x%lx\n", i,
				 sprd_dump_mem[i].vaddr);
			pr_emerg("sprd_dump_mem[%d].size  is0x%lx\n", i,
				 sprd_dump_mem[i].size);
			i++;
		}
	}
	sprd_dump_mem_num = i;
}

static void sysdump_prepare_info(int enter_id, const char *reason,
				 struct pt_regs *regs)
{
	int iocnt, i;
	char *iomem;
	struct timex txc;
	struct rtc_time tm;

	strncpy(sprd_sysdump_extra.reason,
		reason, sizeof(sprd_sysdump_extra.reason));
	sprd_sysdump_extra.enter_id = enter_id;
	memcpy(sprd_sysdump_info->magic, SYSDUMP_MAGIC,
	       sizeof(sprd_sysdump_info->magic));

	if (reason != NULL && !strcmp(reason, "Crash Key"))
		sprd_sysdump_info->crash_key = 1;
	else
		sprd_sysdump_info->crash_key = 0;

	pr_emerg("reason: %s, sprd_sysdump_info->crash_key: %d\n",
		 reason, sprd_sysdump_info->crash_key);
	do_gettimeofday(&(txc.time));
	txc.time.tv_sec -= sys_tz.tz_minuteswest * 60;
	rtc_time_to_tm(txc.time.tv_sec, &tm);
	sprintf(sprd_sysdump_info->time, "%04d-%02d-%02d_%02d:%02d:%02d",
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
		tm.tm_min, tm.tm_sec);

	memcpy(sprd_sysdump_info->dump_path, sysdump_conf.dump_path,
	       sizeof(sprd_sysdump_info->dump_path));

	sprd_sysdump_info->dump_mem_paddr = virt_to_phys(sprd_dump_mem);
	sprd_sysdump_info->mem_num = sprd_dump_mem_num;
	sprd_sysdump_info->elfhdr_size =
	    get_elfhdr_size(sprd_sysdump_info->mem_num);

	sysdump_fill_core_hdr(regs,
			      sprd_dump_mem,
			      sprd_dump_mem_num,
			      (char *)sprd_sysdump_info +
			      sizeof(*sprd_sysdump_info),
			      sprd_sysdump_info->mem_num + 1,
			      sprd_sysdump_info->elfhdr_size);

	iocnt = 0;
	for (i = 0; i < sprd_dump_mem_num; i++) {
		/* save iomem(regiters) to ram,
		   cause they will change while rebooting */
		if (0xffffffff != sprd_dump_mem[i].soff) {
			sprd_dump_mem[i].soff = iocnt;
			iomem = (char *)sprd_sysdump_info +
			    sizeof(*sprd_sysdump_info) +
			    sprd_sysdump_info->elfhdr_size + iocnt;
			memcpy_fromio(iomem,
				      (void __iomem *)sprd_dump_mem[i].vaddr,
				      sprd_dump_mem[i].size);
			iocnt += sprd_dump_mem[i].size;
		}
	}

	return;
}

DEFINE_PER_CPU(struct sprd_debug_core_t, sprd_debug_core_reg);
DEFINE_PER_CPU(struct sprd_debug_mmu_reg_t, sprd_debug_mmu_reg);

static inline void sprd_debug_save_context(void)
{
	unsigned long flags;
	local_irq_save(flags);
	sprd_debug_save_mmu_reg(&per_cpu
				(sprd_debug_mmu_reg, smp_processor_id()));
	sprd_debug_save_core_reg(&per_cpu
				 (sprd_debug_core_reg, smp_processor_id()));

	pr_emerg("(%s) context saved(CPU:%d)\n", __func__, smp_processor_id());
	local_irq_restore(flags);

	flush_cache_all();
}

void sysdump_enter(int enter_id, const char *reason, struct pt_regs *regs)
{
	struct pt_regs *pregs;
	if (!sysdump_conf.enable)
		return;

	if ((sysdump_conf.crashkey_only) &&
	    !(reason != NULL && !strcmp(reason, "Crash Key")))
		return;

	/* this should before smp_send_stop() to make sysdump_ipi enable */
	sprd_sysdump_extra.enter_cpu = smp_processor_id();

	pregs = &sprd_sysdump_extra.cpu_context[sprd_sysdump_extra.enter_cpu];
	if (regs)
		memcpy(pregs, regs, sizeof(*regs));
	else
		crash_setup_regs((struct pt_regs *)pregs, NULL);

	dump_kernel_crash(reason, pregs);
	crash_note_save_cpu(pregs, sprd_sysdump_extra.enter_cpu);
	sprd_debug_save_context();

	smp_send_stop();
	mdelay(1000);

	pr_emerg("\n");
	pr_emerg("*****************************************************\n");
	pr_emerg("*                                                   *\n");
	pr_emerg("*  Sysdump enter, preparing debug info to dump ...  *\n");
	pr_emerg("*                                                   *\n");
	pr_emerg("*****************************************************\n");
	pr_emerg("\n");

	flush_cache_all();
	mdelay(1000);

	sysdump_prepare_info(enter_id, reason, regs);

	pr_emerg("\n");
	pr_emerg("*****************************************************\n");
	pr_emerg("*                                                   *\n");
	pr_emerg("*  Try to reboot system ...                         *\n");
	pr_emerg("*                                                   *\n");
	pr_emerg("*****************************************************\n");
	pr_emerg("\n");

	flush_cache_all();
	mdelay(1000);

	emergency_restart();

	return;
}

void sysdump_ipi(struct pt_regs *regs)
{
	int cpu = smp_processor_id();

	if (sprd_sysdump_extra.enter_cpu != -1) {
		memcpy((void *)&(sprd_sysdump_extra.cpu_context[cpu]),
		       (void *)regs, sizeof(struct pt_regs));
		crash_note_save_cpu(regs, cpu);
		sprd_debug_save_context();
	}
	return;
}

int sysdump_fill_crash_content(int type, char *magic, char *payload, int size)
{
	void *dest_addr;
	struct crash_info *info;
	int ret = size;
	unsigned long long t;
	unsigned long long nanosec_rem;

	if (size > (PAGE_SIZE << 1))
		return -1;

	dest_addr = (void *)PAGE_ALIGN((unsigned long)sprd_sysdump_info
		    + sizeof(*sprd_sysdump_info)
		    + get_elfhdr_size(sprd_dump_mem_num)
		    + PAGE_SIZE);

	info = &(sprd_sysdump_info->crash);

	t = sched_clock();
	nanosec_rem = do_div(t, NSEC_PER_SEC);
	snprintf(info->time, 32, "%llu.%06llu", (unsigned long)t, nanosec_rem);

	info->payload = (void *)virt_to_phys(dest_addr);
	info->size = size+1;
	memcpy(info->magic, magic, 16);

	if (type == FRAMEWORK_CRASH) {
		if (copy_from_user(dest_addr, payload, size))
			ret = -EFAULT;
	} else {
		memcpy(dest_addr, payload, size);
	}

	((char *)dest_addr)[size] = '\0';

	flush_cache_all();

	return ret;
}

static int sprd_sysdump_read(struct seq_file *s, void *v)
{
	seq_printf(s, "sysdump_status = %d\n", sysdump_status);
	return 0;
}

static int sprd_sysdump_open(struct inode *inode, struct file *file)
{
	return single_open(file, sprd_sysdump_read, NULL);
}

static ssize_t sprd_sysdump_write(struct file *file, const char __user *buf,
				  size_t count, loff_t *data)
{
	char sysdump_buf[5] = {0};

	pr_emerg("sprd_sysdump_write: start!!!\n");
	if (count) {
		if (copy_from_user(sysdump_buf, buf, count)) {
			pr_emerg("sprd_sysdump_write: copy_from_user failed!!!\n");
			return -1;
		}
		sysdump_buf[count] = '\0';

		if (!strncmp(sysdump_buf, "on", 2)) {
			pr_emerg("sprd_sysdump_write: enable user version sysdump!!!\n");
			sysdump_status = 1;
			sprd_set_reboot_mode("dumpenable");

		} else if (!strncmp(sysdump_buf, "off", 3)) {
			pr_emerg("sprd_sysdump_write: disable user version sysdump!!!\n");
			sysdump_status = 0;
			sprd_set_reboot_mode("dumpdisable");
		}
	}

	pr_emerg("sprd_sysdump_write: end!!!\n");
	return count;
}

static struct ctl_table sysdump_sysctl_table[] = {
	{
	 .procname = "sysdump_enable",
	 .data = &sysdump_conf.enable,
	 .maxlen = sizeof(int),
	 .mode = 0644,
	 .proc_handler = proc_dointvec,
	 },
	{
	 .procname = "sysdump_crashkey_only",
	 .data = &sysdump_conf.crashkey_only,
	 .maxlen = sizeof(int),
	 .mode = 0644,
	 .proc_handler = proc_dointvec,
	 },
	{
	 .procname = "sysdump_dump_modem",
	 .data = &sysdump_conf.dump_modem,
	 .maxlen = sizeof(int),
	 .mode = 0644,
	 .proc_handler = proc_dointvec,
	 },
	{
	 .procname = "sysdump_reboot",
	 .data = &sysdump_conf.reboot,
	 .maxlen = sizeof(int),
	 .mode = 0644,
	 .proc_handler = proc_dointvec,
	 },
	{
	 .procname = "sysdump_dump_path",
	 .data = sysdump_conf.dump_path,
	 .maxlen = sizeof(sysdump_conf.dump_path),
	 .mode = 0644,
	 .proc_handler = proc_dostring,
	 }
	,
	{}
};

static struct ctl_table sysdump_sysctl_root[] = {
	{
	 .procname = "kernel",
	 .mode = 0555,
	 .child = sysdump_sysctl_table,
	 },
	{}
};

static struct ctl_table_header *sysdump_sysctl_hdr;

static const struct file_operations sysdump_proc_fops = {
	.owner = THIS_MODULE,
	.open = sprd_sysdump_open,
	.read = seq_read,
	.write = sprd_sysdump_write,
	.llseek = seq_lseek,
	.release = single_release,
};

int sysdump_sysctl_init(void)
{
	struct proc_dir_entry *sysdump_proc;  

	get_sprd_sysdump_info_paddr();
	sprd_sysdump_info = (struct sysdump_info *)
	    phys_to_virt(sysdump_magic_paddr);

	fill_sprd_sysdump_mem();
	fill_sprd_sysdump_log();
	sprd_sysdump_info->last_kmsg_paddr = virt_to_phys(sprd_last_kmsg);
	pr_emerg("sprd_sysdump_info->last_kmsg_paddr is 0x%lx\n", virt_to_phys(sprd_last_kmsg));
	/*sprd_sysdump_crc32(&sprd_sysdump_info->collections);*/
	sysdump_sysctl_hdr =
	    register_sysctl_table((struct ctl_table *)sysdump_sysctl_root);
	if (!sysdump_sysctl_hdr)
		return -ENOMEM;

	crash_notes = alloc_percpu(note_buf_t);
	if (crash_notes == NULL)
		return -ENOMEM;

	dumpinfo_init();
	
	sysdump_proc = proc_create("sprd_sysdump", S_IWUSR | S_IRUSR, NULL, &sysdump_proc_fops);
	if (!sysdump_proc)
		return -ENOMEM;

#ifdef CONFIG_SPRD_DEBUG
	pr_emerg("userdebug enable sysdump in default !!!\n");
	sprd_set_reboot_mode("dumpenable");
	sysdump_status = 1;
#endif	

	return 0;
}

void sysdump_sysctl_exit(void)
{
	if (sysdump_sysctl_hdr)
		unregister_sysctl_table(sysdump_sysctl_hdr);

	dumpinfo_exit();
	
	remove_proc_entry("sprd_sysdump", NULL); 
}

module_init(sysdump_sysctl_init);
module_exit(sysdump_sysctl_exit);

MODULE_AUTHOR("Jianjun.He <jianjun.he@spreadtrum.com>");
MODULE_DESCRIPTION("kernel core dump for Spreadtrum");
MODULE_LICENSE("GPL");

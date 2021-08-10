/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/init.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/memblock.h>
#include <linux/elf.h>
#include <linux/kdebug.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/bug.h>
#include <linux/compiler.h>
#include <linux/sizes.h>
#include <linux/spinlock.h>
#include <linux/stacktrace.h>
#include <asm/pgtable.h>
#include <asm-generic/percpu.h>
#include <asm-generic/sections.h>
#include <asm/page.h>
#include <asm/irq.h>
#include <mrdump.h>
#include <mt-plat/aee.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_reserved_mem.h>
#include <linux/uaccess.h>
#include <linux/highmem.h>
#include "../../../../kernel/sched/sched.h"
#include "mrdump_mini.h"
#include "mrdump_private.h"
#include <mach/memory_layout.h>

#define LOG_DEBUG(fmt, ...)			\
	do {	\
		if (aee_in_nested_panic())			\
			aee_nested_printf(fmt, ##__VA_ARGS__);	\
		else						\
			pr_debug(fmt, ##__VA_ARGS__);	\
	} while (0)

#define LOG_NOTICE(fmt, ...)			\
	do {	\
		if (aee_in_nested_panic())			\
			aee_nested_printf(fmt, ##__VA_ARGS__);	\
		else						\
			pr_notice(fmt, ##__VA_ARGS__);	\
	} while (0)

#define LOGV(fmt, msg...)
#define LOGD LOG_DEBUG
#define LOGI LOG_DEBUG
#define LOGW LOG_NOTICE
#define LOGE LOG_NOTICE

static struct mrdump_mini_elf_header *mrdump_mini_ehdr;
#ifdef CONFIG_MODULES
static char modules_info_buf[MODULES_INFO_BUF_SIZE];
#endif

static bool dump_all_cpus;

__weak void get_android_log_buffer(unsigned long *addr, unsigned long *size,
		unsigned long *start, int type)
{
}

#if defined(CONFIG_GZ_LOG)
__weak void get_gz_log_buffer(unsigned long *addr, unsigned long *paddr,
			unsigned long *size, unsigned long *start)
{
}
#endif

__weak void get_disp_err_buffer(unsigned long *addr, unsigned long *size,
		unsigned long *start)
{
}


__weak void get_disp_fence_buffer(unsigned long *addr, unsigned long *size,
		unsigned long *start)
{
}

__weak void get_disp_dbg_buffer(unsigned long *addr, unsigned long *size,
		unsigned long *start)
{
}

__weak void get_disp_dump_buffer(unsigned long *addr, unsigned long *size,
		unsigned long *start)
{
}

__weak void get_kernel_log_buffer(unsigned long *addr, unsigned long *size,
		unsigned long *start)
{
}

__weak void aee_rr_get_desc_info(unsigned long *addr, unsigned long *size,
		unsigned long *start)
{
}

__weak void get_pidmap_aee_buffer(unsigned long *addr, unsigned long *size)
{
}

__weak struct vm_struct *find_vm_area(const void *addr)
{
	return NULL;
}

#ifdef __aarch64__
#define MIN_MARGIN KIMAGE_VADDR
#else
#define MIN_MARGIN PAGE_OFFSET
#endif

#undef mrdump_virt_addr_valid
#define mrdump_virt_addr_valid(kaddr) \
	pfn_valid(virt_2_pfn((unsigned long)(kaddr)))
#ifdef __aarch64__
static unsigned long virt_2_pfn(unsigned long addr)
{
	pgd_t *pgd = pgd_offset_k(addr), _pgd_val = {0};
	pud_t *pud, _pud_val = {{0} };
	pmd_t *pmd, _pmd_val = {0};
	pte_t *ptep, _pte_val = {0};
	unsigned long pfn = ~0UL;

#ifdef CONFIG_ARM64
	if (addr < VA_START)
		goto OUT;
#endif
	if (probe_kernel_address(pgd, _pgd_val) || pgd_none(_pgd_val))
		goto OUT;
	pud = pud_offset(pgd, addr);
	if (probe_kernel_address(pud, _pud_val) || pud_none(_pud_val))
		goto OUT;
	if (pud_sect(_pud_val)) {
		pfn = pud_pfn(_pud_val) + ((addr&~PUD_MASK) >> PAGE_SHIFT);
	} else if (pud_table(_pud_val)) {
		pmd = pmd_offset(pud, addr);
		if (probe_kernel_address(pmd, _pmd_val) || pmd_none(_pmd_val))
			goto OUT;
		if (pmd_sect(_pmd_val)) {
			pfn = pmd_pfn(_pmd_val) +
				((addr&~PMD_MASK) >> PAGE_SHIFT);
		} else if (pmd_table(_pmd_val)) {
			ptep = pte_offset_map(pmd, addr);
			if (probe_kernel_address(ptep, _pte_val)
				|| !pte_present(_pte_val)) {
				pte_unmap(ptep);
				goto OUT;
			}
			pfn = pte_pfn(_pte_val);
			pte_unmap(ptep);
		}
	}
OUT:
	return pfn;

}
#else
#ifndef pmd_sect
#define pmd_sect(pmd)	(pmd & PMD_TYPE_SECT)
#endif
#ifndef pmd_table
#define pmd_table(pmd)	(pmd & PMD_TYPE_TABLE)
#endif
#ifndef pmd_pfn
#define pmd_pfn(pmd)	(((pmd_val(pmd) & PMD_MASK) & PHYS_MASK) >> PAGE_SHIFT)
#endif
static unsigned long virt_2_pfn(unsigned long addr)
{
	pgd_t *pgd = pgd_offset_k(addr), _pgd_val = {0};
#ifdef CONFIG_ARM_LPAE
	pud_t *pud, _pud_val = {0};
#else
	pud_t *pud, _pud_val = {{0} };
#endif
	pmd_t *pmd, _pmd_val = 0;
	pte_t *ptep, _pte_val = 0;
	unsigned long pfn = ~0UL;

	if (probe_kernel_address(pgd, _pgd_val) || pgd_none(_pgd_val))
		goto OUT;
	pud = pud_offset(pgd, addr);
	if (probe_kernel_address(pud, _pud_val) || pud_none(_pud_val))
		goto OUT;
	pmd = pmd_offset(pud, addr);
	if (probe_kernel_address(pmd, _pmd_val) || pmd_none(_pmd_val))
		goto OUT;
	if (pmd_sect(_pmd_val)) {
		pfn = pmd_pfn(_pmd_val) + ((addr&~PMD_MASK) >> PAGE_SHIFT);
	} else if (pmd_table(_pmd_val)) {
		ptep = pte_offset_map(pmd, addr);
		if (probe_kernel_address(ptep, _pte_val)
			|| !pte_present(_pte_val)) {
			pte_unmap(ptep);
			goto OUT;
		}
		pfn = pte_pfn(_pte_val);
		pte_unmap(ptep);
	}
OUT:
	return pfn;
}
#endif

/* copy from fs/binfmt_elf.c */
static void fill_elf_header(struct elfhdr *elf, int segs)
{
	memcpy(elf->e_ident, ELFMAG, SELFMAG);
	elf->e_ident[EI_CLASS] = ELF_CLASS;
	elf->e_ident[EI_DATA] = ELF_DATA;
	elf->e_ident[EI_VERSION] = EV_CURRENT;
	elf->e_ident[EI_OSABI] = ELF_OSABI;

	elf->e_type = ET_CORE;
	elf->e_machine = ELF_ARCH;
	elf->e_version = EV_CURRENT;
	elf->e_phoff = sizeof(struct elfhdr);
#ifndef ELF_CORE_EFLAGS
#define ELF_CORE_EFLAGS	0
#endif
	elf->e_flags = ELF_CORE_EFLAGS;
	elf->e_ehsize = sizeof(struct elfhdr);
	elf->e_phentsize = sizeof(struct elf_phdr);
	elf->e_phnum = segs;

}

static void fill_elf_note_phdr(struct elf_phdr *phdr, int sz, loff_t offset)
{
	phdr->p_type = PT_NOTE;
	phdr->p_offset = offset;
	phdr->p_vaddr = 0;
	phdr->p_paddr = 0;
	phdr->p_filesz = sz;
	phdr->p_memsz = 0;
	phdr->p_flags = 0;
	phdr->p_align = 0;
}

static void fill_elf_load_phdr(struct elf_phdr *phdr, int sz,
			       unsigned long vaddr, unsigned long paddr)
{
	phdr->p_type = PT_LOAD;
	phdr->p_vaddr = vaddr;
	phdr->p_paddr = paddr;
	phdr->p_filesz = sz;
	phdr->p_memsz = 0;
	phdr->p_flags = 0;
	phdr->p_align = 0;
}

static noinline void fill_note(struct elf_note *note, const char *name,
		int type, unsigned int sz, unsigned int namesz)
{
	char *n_name = (char *)note + sizeof(struct elf_note);

	note->n_namesz = namesz;
	note->n_type = type;
	note->n_descsz = sz;
	strncpy(n_name, name, note->n_namesz);
}

static void fill_note_L(struct elf_note *note, const char *name, int type,
		unsigned int sz)
{
	fill_note(note, name, type, sz, NOTE_NAME_LONG);
}

static void fill_note_S(struct elf_note *note, const char *name, int type,
		unsigned int sz)
{
	fill_note(note, name, type, sz, NOTE_NAME_SHORT);
}

/*
 * fill up all the fields in prstatus from the given task struct, except
 * registers which need to be filled up separately.
 */
static void fill_prstatus(struct elf_prstatus *prstatus, struct pt_regs *regs,
			  struct task_struct *p, unsigned long pid)
{
	elf_core_copy_regs(&prstatus->pr_reg, regs);
	prstatus->pr_pid = pid;
	prstatus->pr_ppid = AEE_MTK_CPU_NUMS;
	prstatus->pr_sigpend = (uintptr_t)p;
}

static int fill_psinfo(struct elf_prpsinfo *psinfo)
{
	unsigned int i;

	strncpy(psinfo->pr_psargs, saved_command_line, ELF_PRARGSZ - 1);
	for (i = 0; i < ELF_PRARGSZ - 1; i++)
		if (psinfo->pr_psargs[i] == 0)
			psinfo->pr_psargs[i] = ' ';
	psinfo->pr_psargs[ELF_PRARGSZ - 1] = 0;
	strncpy(psinfo->pr_fname, "vmlinux", sizeof(psinfo->pr_fname));
	return 0;
}

void mrdump_mini_add_misc_pa(unsigned long va, unsigned long pa,
		unsigned long size, unsigned long start, char *name)
{
	int i;
	struct elf_note *note;

	for (i = 0; i < MRDUMP_MINI_NR_MISC; i++) {
		note = &mrdump_mini_ehdr->misc[i].note;
		if (note->n_type == NT_IPANIC_MISC) {
			if (strncmp(mrdump_mini_ehdr->misc[i].name, name, 16)
					!= 0)
				continue;
		}
		mrdump_mini_ehdr->misc[i].data.vaddr = va;
		mrdump_mini_ehdr->misc[i].data.paddr = pa;
		mrdump_mini_ehdr->misc[i].data.size = size;
		mrdump_mini_ehdr->misc[i].data.start =
		    mrdump_virt_addr_valid((void *)start) ? __pa(start) : 0;
		fill_note_L(note, name, NT_IPANIC_MISC,
				sizeof(struct mrdump_mini_elf_misc));
		break;
	}
}

void mrdump_mini_add_misc(unsigned long addr, unsigned long size,
		unsigned long start, char *name)
{
	if (!mrdump_virt_addr_valid((void *)addr))
		return;
	mrdump_mini_add_misc_pa(addr, __pa(addr), size, start, name);
}

int kernel_addr_valid(unsigned long addr)
{
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;

	if (addr < MIN_MARGIN)
		return 0;

	pgd = pgd_offset_k(addr);
	if (pgd_none(*pgd))
		return 0;
	pr_notice("[%08lx] *pgd=%08llx", addr, (long long)pgd_val(*pgd));

	pud = pud_offset(pgd, addr);
	if (pud_none(*pud))
		return 0;
	pr_notice("*pud=%08llx", (long long)pud_val(*pud));

	pmd = pmd_offset(pud, addr);
	if (pmd_none(*pmd))
		return 0;
	pr_notice("*pmd=%08llx", (long long)pmd_val(*pmd));

	pte = pte_offset_kernel(pmd, addr);
	if (pte_none(*pte))
		return 0;
	pr_notice("*pte=%08llx", (long long)pte_val(*pte));

	return pfn_valid(pte_pfn(*pte));
}

static void mrdump_mini_add_entry_ext(unsigned long start, unsigned long end,
		unsigned long pa)
{
	unsigned long laddr, haddr;
	struct elf_phdr *phdr;
	int i;

	for (i = 0; i < MRDUMP_MINI_NR_SECTION; i++) {
		phdr = &mrdump_mini_ehdr->phdrs[i];
		if (phdr->p_type == PT_NULL)
			break;
		if (phdr->p_type != PT_LOAD)
			continue;
		laddr = phdr->p_vaddr;
		haddr = laddr + phdr->p_filesz;
		if (start >= laddr && end <= haddr)
			return;
		if (start >= haddr || end <= laddr)
			continue;
		if (laddr < start) {
			start = laddr;
			pa = phdr->p_paddr;
		}
		if (haddr > end)
			end = haddr;
		break;
	}
	if (i < MRDUMP_MINI_NR_SECTION)
		fill_elf_load_phdr(phdr, end - start, start, pa);
	else
		LOGE("mrdump: MINI_NR_SECTION overflow!\n");
}

#ifdef __aarch64__
static bool addr_in_kimg(unsigned long addr)
{
	if ((void *)(addr) >= (void *)KIMAGE_VADDR &&
		(void *)(addr) < (void *)_end)
		return true;
	else
		return false;
}
#else
static bool addr_in_kimg(unsigned long addr)
{
	return false;
}
#endif

void mrdump_mini_add_entry(unsigned long addr, unsigned long size)
{
	unsigned long start = 0, __end, pa = 0, end_pfn = 0, _pfn;

	if (!pfn_valid(virt_2_pfn(addr)))
		return;
	if (((addr < VMALLOC_END && addr >= VMALLOC_START)
			|| addr < PAGE_OFFSET)
			&& !addr_in_kimg(addr)) {
		/* If addr belongs to non-linear mapping region and not
		 * in kernel image, we only dump 1 page to economize on
		 * the number of sections of minirdump
		 */
		start = addr & PAGE_MASK;
		mrdump_mini_add_entry_ext(start, start + PAGE_SIZE,
				__pfn_to_phys(virt_2_pfn(addr)));
		return;
	}
	for (__end = ALIGN(addr + size / 2, PAGE_SIZE),
			addr = __end - ALIGN(size, PAGE_SIZE);
			addr < __end; addr += PAGE_SIZE) {
		_pfn = virt_2_pfn(addr);
		if (pfn_valid(_pfn)) {
			if (!start || _pfn != end_pfn) {
				if (start)
					mrdump_mini_add_entry_ext(start,
						start + (__pfn_to_phys(end_pfn)
							- pa), pa);
				start = addr;
				pa = __pfn_to_phys(_pfn);
				end_pfn = _pfn + 1;
			} else {
				end_pfn++;
			}
		} else {
			if (start)
				mrdump_mini_add_entry_ext(start,
					start + (__pfn_to_phys(end_pfn) - pa),
					pa);
			start = 0;
		}
	}
	if (start)
		mrdump_mini_add_entry_ext(start,
				start + (__pfn_to_phys(end_pfn) - pa), pa);
}

static void mrdump_mini_add_tsk_ti(int cpu, struct pt_regs *regs,
		struct task_struct *tsk, int stack)
{
	struct thread_info *ti = NULL;
	unsigned long *bottom = NULL;
	unsigned long *top = NULL;
	unsigned long *p;

	if (!mrdump_virt_addr_valid(tsk)) {
		LOGE("mrdump: cpu:[%d] invalid task pointer:%p\n", cpu, tsk);
		if (cpu < num_possible_cpus())
			tsk = cpu_curr(cpu);
		else
			LOGE("mrdump: cpu:[%d] overflow with total:%d\n",
					cpu, num_possible_cpus());
	}
	if (!mrdump_virt_addr_valid(tsk))
		LOGE("mrdump: cpu:[%d] CAN'T get a valid task pointer:%p\n",
				cpu, tsk);
	else
		ti = (struct thread_info *)tsk->stack;

	bottom = (unsigned long *)regs->reg_sp;
	mrdump_mini_add_entry(regs->reg_sp, MRDUMP_MINI_SECTION_SIZE);
	mrdump_mini_add_entry((unsigned long)ti, MRDUMP_MINI_SECTION_SIZE);
	mrdump_mini_add_entry((unsigned long)tsk, MRDUMP_MINI_SECTION_SIZE);
	LOGE("mrdump: cpu[%d] tsk:%p ti:%p\n", cpu, tsk, ti);
	if (!stack)
		return;
#ifdef __aarch64__
	if (on_irq_stack((unsigned long)bottom, cpu))
		top = (unsigned long *)IRQ_STACK_PTR(cpu);
	else {
		top = (unsigned long *)ALIGN((unsigned long)bottom,
					THREAD_SIZE);
		p = (unsigned long *)((void *)ti + THREAD_SIZE);
		if (!mrdump_virt_addr_valid(top)
			|| !mrdump_virt_addr_valid(bottom)
			|| top != p || bottom > top) {
			LOGE(
				"mrdump: unexpected case bottom:%p top:%p ti + THREAD_SIZE:%p\n"
				, bottom, top, p);
			return;
		}
	}
#else
	top = (unsigned long *)((void *)ti + THREAD_SIZE);
	if (!mrdump_virt_addr_valid(ti) || !mrdump_virt_addr_valid(top)
		|| bottom < (unsigned long *)ti
		|| bottom > top)
		return;
#endif

	for (p = (unsigned long *)ALIGN((unsigned long)bottom,
				sizeof(unsigned long)); p < top; p++) {
		if (!mrdump_virt_addr_valid(*p))
			continue;
		if (*p >= (unsigned long)ti && *p <= (unsigned long)top)
			continue;
		if (*p >= (unsigned long)_stext && *p <= (unsigned long)_etext)
			continue;
		mrdump_mini_add_entry(*p, MRDUMP_MINI_SECTION_SIZE);
	}
}

static int mrdump_mini_cpu_regs(int cpu, struct pt_regs *regs,
		struct task_struct *tsk, int main)
{
	char name[NOTE_NAME_SHORT];
	int id;

	if (mrdump_mini_ehdr == NULL)
		mrdump_mini_init();
	if (cpu >= AEE_MTK_CPU_NUMS || mrdump_mini_ehdr == NULL)
		return -1;
	if (regs == NULL)
		return -1;
	id = main ? 0 : cpu + 1;
	if (strncmp(mrdump_mini_ehdr->prstatus[id].name, "NA", 2))
		return -1;
	snprintf(name, NOTE_NAME_SHORT - 1, main ? "ke%d" : "core%d", cpu);
	fill_prstatus(&mrdump_mini_ehdr->prstatus[id].data, regs, tsk,
			id ? id : (100 + cpu));
	fill_note_S(&mrdump_mini_ehdr->prstatus[id].note, name, NT_PRSTATUS,
		    sizeof(struct elf_prstatus));
	return 0;
}

void mrdump_mini_per_cpu_regs(int cpu, struct pt_regs *regs,
		struct task_struct *tsk)
{
	mrdump_mini_cpu_regs(cpu, regs, tsk, 0);
}
EXPORT_SYMBOL(mrdump_mini_per_cpu_regs);

void mrdump_mini_build_task_info(struct pt_regs *regs)
{
#define MAX_STACK_TRACE_DEPTH 64
	unsigned long ipanic_stack_entries[MAX_STACK_TRACE_DEPTH];
	char symbol[96] = {'\0'};
	int sz;
	int off, plen;
	struct stack_trace trace;
	int i;
	struct task_struct *tsk, *cur;
	struct task_struct *previous;
	struct aee_process_info *cur_proc;

	if (!mrdump_virt_addr_valid(current_thread_info())) {
		LOGE("current thread info invalid\n");
		return;
	}
	cur = current;
	tsk = cur;
	if (!mrdump_virt_addr_valid(tsk)) {
		LOGE("tsk invalid\n");
		return;
	}
	cur_proc = (struct aee_process_info *)((void *)mrdump_mini_ehdr +
			MRDUMP_MINI_HEADER_SIZE);
	/* Current panic user tasks */
	sz = 0;
	do {
		if (!tsk) {
			LOGE("No tsk info\n");
			memset_io(cur_proc, 0x0,
				sizeof(struct aee_process_info));
			break;
		}
		/* FIXME: Check overflow ? */
		sz += snprintf(symbol + sz, 96 - sz, "[%s, %d]", tsk->comm,
				tsk->pid);
		previous = tsk;
		tsk = tsk->real_parent;
		if (!mrdump_virt_addr_valid(tsk)) {
			LOGE("tsk(%p) invalid (previous: [%s, %d])\n", tsk,
					previous->comm, previous->pid);
			break;
		}
	} while (tsk && (tsk->pid != 0) && (tsk->pid != 1));
	if (strncmp(cur_proc->process_path, symbol, sz) == 0) {
		LOGE("same process path\n");
		return;
	}

	memset_io(cur_proc, 0, sizeof(struct aee_process_info));
	memcpy(cur_proc->process_path, symbol, sz);

	/* Grab kernel task stack trace */
	trace.nr_entries = 0;
	trace.max_entries = MAX_STACK_TRACE_DEPTH;
	trace.entries = ipanic_stack_entries;
	trace.skip = 8;
	save_stack_trace_tsk(cur, &trace);
	/* Skip the entries -
	 * ipanic_save_current_tsk_info/save_stack_trace_tsk
	 */
	for (i = 0; i < trace.nr_entries; i++) {
		off = strlen(cur_proc->backtrace);
		plen = AEE_BACKTRACE_LENGTH - ALIGN(off, 8);
		if (plen > 16) {
			sz = snprintf(symbol, 96, "[<%px>] %pS\n",
				      (void *)ipanic_stack_entries[i],
				      (void *)ipanic_stack_entries[i]);
			if (ALIGN(sz, 8) - sz) {
				memset_io(symbol + sz - 1, ' ',
						ALIGN(sz, 8) - sz);
				memset_io(symbol + ALIGN(sz, 8) - 1, '\n', 1);
			}
			if (ALIGN(sz, 8) <= plen)
				memcpy(cur_proc->backtrace + ALIGN(off, 8),
						symbol, ALIGN(sz, 8));
		}
	}
	if (regs) {
		cur_proc->ke_frame.pc = (__u64) regs->reg_pc;
		cur_proc->ke_frame.lr = (__u64) regs->reg_lr;
	} else {
		/* in case panic() is called without die */
		/* Todo: a UT for this */
		cur_proc->ke_frame.pc = ipanic_stack_entries[0];
		cur_proc->ke_frame.lr = ipanic_stack_entries[1];
	}
	snprintf(cur_proc->ke_frame.pc_symbol, AEE_SZ_SYMBOL_S, "[<%px>] %pS",
		 (void *)(unsigned long)cur_proc->ke_frame.pc,
		 (void *)(unsigned long)cur_proc->ke_frame.pc);
	snprintf(cur_proc->ke_frame.lr_symbol, AEE_SZ_SYMBOL_L, "[<%px>] %pS",
		 (void *)(unsigned long)cur_proc->ke_frame.lr,
		 (void *)(unsigned long)cur_proc->ke_frame.lr);
}

int mrdump_task_info(unsigned char *buffer, size_t sz_buf)
{
	if (sz_buf < sizeof(struct aee_process_info))
		return -1;
	memcpy(buffer, (void *)mrdump_mini_ehdr + MRDUMP_MINI_HEADER_SIZE,
	       sizeof(struct aee_process_info));
	return sizeof(struct aee_process_info);
}

__weak int save_modules(char *mbuf, int mbufsize)
{
	LOGE("%s weak function\n", __func__);
	return 0;
}

int mrdump_modules_info(unsigned char *buffer, size_t sz_buf)
{
#ifdef CONFIG_MODULES
	int sz;

	sz = save_modules(modules_info_buf, MODULES_INFO_BUF_SIZE);
	if (sz_buf < sz || buffer == NULL)
		return -1;
	memcpy(buffer, modules_info_buf, sz);
	return sz;
#else
	return -1;
#endif
}

static void mrdump_mini_clear_loads(void);
void mrdump_mini_add_hang_raw(unsigned long vaddr, unsigned long size)
{
	LOGE("mrdump: hang data 0x%lx size:0x%lx\n", vaddr, size);
	mrdump_mini_add_misc(vaddr, size, 0, "_HANG_DETECT_");
	/* hang only remove mini rdump loads info to save storage space */
	mrdump_mini_clear_loads();
}

#define EXTRA_MISC(func, name, max_size) \
	__weak void func(unsigned long *vaddr, unsigned long *size) \
	{ \
		if (size != NULL) \
			*size = 0; \
	}
#include "mrdump_mini_extra_misc.h"

#undef EXTRA_MISC
#define EXTRA_MISC(func, name, max_size) \
	{func, name, max_size},

static struct mrdump_mini_extra_misc extra_members[] = {
	#include "mrdump_mini_extra_misc.h"
};
#define EXTRA_TOTAL_NUM ((sizeof(extra_members)) / (sizeof(extra_members[0])))
static size_t __maybe_unused dummy_check(void)
{
	size_t dummy;

	dummy = BUILD_BUG_ON_ZERO(EXTRA_TOTAL_NUM > 10);
	return dummy;
}

static int _mrdump_mini_add_extra_misc(unsigned long vaddr, unsigned long size,
	const char *name)
{
	char name_buf[SZ_128];

	if (mrdump_mini_ehdr == NULL ||
		size == 0 ||
		size > SZ_512K ||
		name == NULL)
		return -1;
	snprintf(name_buf, SZ_128, "_EXTRA_%s_", name);
	mrdump_mini_add_misc(vaddr, size, 0, name_buf);
	return 0;
}

void mrdump_mini_add_extra_misc(void)
{
	static int once;
	int i;
	unsigned long vaddr = 0;
	unsigned long size = 0;
	int ret;

	if (once == 0) {
		once = 1;
		_mrdump_mini_add_extra_misc((unsigned long)extra_members,
			sizeof(extra_members), "ALL");
		for (i = 0; i < EXTRA_TOTAL_NUM; i++) {
			extra_members[i].dump_func(&vaddr, &size);
			if (size > extra_members[i].max_size)
				continue;
			ret = _mrdump_mini_add_extra_misc(vaddr, size,
					extra_members[i].dump_name);
			if (ret < 0)
				LOGE("mrdump: add %s:0x%lx sz:0x%lx failed\n",
					extra_members[i].dump_name,
					vaddr, size);
		}
	}
}

static void mrdump_mini_add_loads(void);
void mrdump_mini_ke_cpu_regs(struct pt_regs *regs)
{
	int cpu;
	struct pt_regs context;

	if (!regs) {
		regs = &context;
		mrdump_mini_save_regs(regs);
	}
	cpu = get_HW_cpuid();
	mrdump_mini_cpu_regs(cpu, regs, current, 1);
	mrdump_mini_add_loads();
	mrdump_mini_build_task_info(regs);
	mrdump_modules_info(NULL, -1);
	mrdump_mini_add_extra_misc();
}
EXPORT_SYMBOL(mrdump_mini_ke_cpu_regs);

static void mrdump_mini_fatal(const char *str)
{
	LOGE("minirdump: FATAL:%s\n", str);
	BUG();
}

static unsigned int mrdump_mini_addr;
static unsigned int mrdump_mini_size;
void mrdump_mini_set_addr_size(unsigned int addr, unsigned int size)
{
	mrdump_mini_addr = addr;
	mrdump_mini_size = size;
}

static void mrdump_mini_build_elf_misc(void)
{
	int i;
	struct mrdump_mini_elf_misc misc;
	char log_type[][16] = { "_MAIN_LOG_", "_EVENTS_LOG_", "_RADIO_LOG_",
		"_SYSTEM_LOG_" };
	unsigned long task_info_va =
	    (unsigned long)((void *)mrdump_mini_ehdr + MRDUMP_MINI_HEADER_SIZE);
	unsigned long task_info_pa = 0;
#if defined(CONFIG_GZ_LOG)
	unsigned long gz_log_pa = 0;

	memset_io(&misc, 0, sizeof(struct mrdump_mini_elf_misc));
	get_gz_log_buffer(&misc.vaddr, &gz_log_pa, &misc.size, &misc.start);
	if (gz_log_pa != 0)
		mrdump_mini_add_misc_pa(misc.vaddr, gz_log_pa, misc.size,
					misc.start, "_GZ_LOG_");
#endif

	if (mrdump_mini_addr != 0
		&& mrdump_mini_size != 0
		&& MRDUMP_MINI_HEADER_SIZE < mrdump_mini_size) {
		task_info_pa = (unsigned long)(mrdump_mini_addr +
				MRDUMP_MINI_HEADER_SIZE);
	} else {
		LOGE("minirdump: unexpected addr:0x%x, size:0x%x(0x%x)\n",
			mrdump_mini_addr, mrdump_mini_size,
			(unsigned int)MRDUMP_MINI_HEADER_SIZE);
		mrdump_mini_fatal("illegal addr size");
	}
	mrdump_mini_add_misc_pa(task_info_va, task_info_pa,
			sizeof(struct aee_process_info), 0, "PROC_CUR_TSK");
	memset_io(&misc, 0, sizeof(struct mrdump_mini_elf_misc));
	get_kernel_log_buffer(&misc.vaddr, &misc.size, &misc.start);
	mrdump_mini_add_misc(misc.vaddr, misc.size, misc.start, "_KERNEL_LOG_");
	memset_io(&misc, 0, sizeof(struct mrdump_mini_elf_misc));
	aee_rr_get_desc_info(&misc.vaddr, &misc.size, &misc.start);
	mrdump_mini_add_misc(misc.vaddr, misc.size, misc.start, "_RR_DESC_");
	memset_io(&misc, 0, sizeof(struct mrdump_mini_elf_misc));
	get_disp_err_buffer(&misc.vaddr, &misc.size, &misc.start);
	mrdump_mini_add_misc(misc.vaddr, misc.size, misc.start, "_DISP_ERR_");
	memset_io(&misc, 0, sizeof(struct mrdump_mini_elf_misc));
	get_disp_dump_buffer(&misc.vaddr, &misc.size, &misc.start);
	mrdump_mini_add_misc(misc.vaddr, misc.size, misc.start, "_DISP_DUMP_");
	memset_io(&misc, 0, sizeof(struct mrdump_mini_elf_misc));
	get_disp_fence_buffer(&misc.vaddr, &misc.size, &misc.start);
	mrdump_mini_add_misc(misc.vaddr, misc.size, misc.start, "_DISP_FENCE_");
	memset_io(&misc, 0, sizeof(struct mrdump_mini_elf_misc));
	get_disp_dbg_buffer(&misc.vaddr, &misc.size, &misc.start);
	mrdump_mini_add_misc(misc.vaddr, misc.size, misc.start, "_DISP_DBG_");
#ifdef CONFIG_MODULES
	mrdump_mini_add_misc_pa((unsigned long)modules_info_buf,
			(unsigned long)__pa(modules_info_buf),
			MODULES_INFO_BUF_SIZE, 0, "SYS_MODULES_INFO");
#endif
	for (i = 0; i < 4; i++) {
		memset_io(&misc, 0, sizeof(struct mrdump_mini_elf_misc));
		get_android_log_buffer(&misc.vaddr, &misc.size, &misc.start,
				i + 1);
		mrdump_mini_add_misc(misc.vaddr, misc.size, misc.start,
				log_type[i]);
	}

	memset_io(&misc, 0, sizeof(struct mrdump_mini_elf_misc));
	get_pidmap_aee_buffer(&misc.vaddr, &misc.size);
	misc.start = 0;
	mrdump_mini_add_misc(misc.vaddr, misc.size, misc.start, "_PIDMAP_");
}

static void mrdump_mini_add_loads(void)
{
	int cpu, i, id;
	struct pt_regs regs;
	struct elf_prstatus *prstatus;
	struct task_struct *tsk = NULL;
	struct thread_info *ti = NULL;

	if (mrdump_mini_ehdr == NULL)
		return;
	for (id = 0; id < AEE_MTK_CPU_NUMS + 1; id++) {
		if (!strncmp(mrdump_mini_ehdr->prstatus[id].name, "NA", 2))
			continue;
		prstatus = &mrdump_mini_ehdr->prstatus[id].data;
		tsk = (prstatus->pr_sigpend != 0) ?
			(struct task_struct *)prstatus->pr_sigpend : current;
		memcpy(&regs, &prstatus->pr_reg, sizeof(prstatus->pr_reg));
		if (prstatus->pr_pid >= 100) {
			for (i = 0; i < ELF_NGREG; i++)
				mrdump_mini_add_entry(
						((unsigned long *)&regs)[i],
						MRDUMP_MINI_SECTION_SIZE);
			cpu = prstatus->pr_pid - 100;
			mrdump_mini_add_tsk_ti(cpu, &regs, tsk, 1);
			mrdump_mini_add_entry((unsigned long)cpu_rq(cpu),
					MRDUMP_MINI_SECTION_SIZE);
		} else if (prstatus->pr_pid <= AEE_MTK_CPU_NUMS) {
			cpu = prstatus->pr_pid - 1;
			mrdump_mini_add_tsk_ti(cpu, &regs, tsk, 0);
			for (i = 0; i < ELF_NGREG; i++) {
				mrdump_mini_add_entry(
					((unsigned long *)&regs)[i],
					MRDUMP_MINI_SECTION_SIZE);
			}
		} else {
			LOGE("mrdump: wrong pr_pid: %d\n", prstatus->pr_pid);
		}
	}

	mrdump_mini_add_entry((unsigned long)__per_cpu_offset,
			MRDUMP_MINI_SECTION_SIZE);
	mrdump_mini_add_entry((unsigned long)&mem_map,
			MRDUMP_MINI_SECTION_SIZE);
	mrdump_mini_add_entry((unsigned long)mem_map, MRDUMP_MINI_SECTION_SIZE);
	if (dump_all_cpus) {
		for (cpu = 0; cpu < AEE_MTK_CPU_NUMS; cpu++) {
			tsk = cpu_curr(cpu);
			if (mrdump_virt_addr_valid(tsk))
				ti = (struct thread_info *)tsk->stack;
			else
				ti = NULL;
			mrdump_mini_add_entry((unsigned long)cpu_rq(cpu),
					MRDUMP_MINI_SECTION_SIZE);
			mrdump_mini_add_entry((unsigned long)tsk,
					MRDUMP_MINI_SECTION_SIZE);
			mrdump_mini_add_entry((unsigned long)ti,
					MRDUMP_MINI_SECTION_SIZE);
		}
	}
}

static void mrdump_mini_clear_loads(void)
{
	struct elf_phdr *phdr;
	int i;

	for (i = 0; i < MRDUMP_MINI_NR_SECTION; i++) {
		phdr = &mrdump_mini_ehdr->phdrs[i];
		if (phdr->p_type == PT_NULL)
			continue;
		if (phdr->p_type == PT_LOAD)
			phdr->p_type = PT_NULL;
	}
}

static void *remap_lowmem(phys_addr_t start, phys_addr_t size)
{
	struct page **pages;
	phys_addr_t page_start;
	unsigned int page_count;
	pgprot_t prot;
	unsigned int i;
	void *vaddr;

	page_start = start - offset_in_page(start);
	page_count = DIV_ROUND_UP(size + offset_in_page(start), PAGE_SIZE);

	prot = pgprot_noncached(PAGE_KERNEL);

	pages = kmalloc_array(page_count, sizeof(struct page *), GFP_KERNEL);
	if (!pages) {
		LOGE("%s: Failed to allocate array for %u pages\n",
				__func__, page_count);
		return NULL;
	}

	for (i = 0; i < page_count; i++) {
		phys_addr_t addr = page_start + i * PAGE_SIZE;

		pages[i] = pfn_to_page(addr >> PAGE_SHIFT);
	}
	vaddr = vmap(pages, page_count, VM_MAP, prot);
	kfree(pages);
	if (!vaddr) {
		LOGE("%s: Failed to map %u pages\n", __func__, page_count);
		return NULL;
	}

	return vaddr + offset_in_page(start);
}

#define TASK_INFO_SIZE PAGE_SIZE
#define PSTORE_SIZE 0x8000
static void __init mrdump_mini_elf_header_init(void)
{
	if (mrdump_mini_addr != 0
		&& mrdump_mini_size != 0) {
		mrdump_mini_ehdr =
		    remap_lowmem(mrdump_mini_addr,
				 mrdump_mini_size);
		LOGE("minirdump: [DT] reserved 0x%x+0x%lx->%p\n",
			mrdump_mini_addr,
			(unsigned long)mrdump_mini_size,
			mrdump_mini_ehdr);
	} else {
		LOGE("minirdump: [DT] illegal value 0x%x(0x%x)\n",
				mrdump_mini_addr,
				mrdump_mini_size);
		mrdump_mini_fatal("illegal addr size");
	}
	if (mrdump_mini_ehdr == NULL) {
		LOGE("mrdump mini reserve buffer fail");
		mrdump_mini_fatal("header null pointer");
		return;
	}
	memset_io(mrdump_mini_ehdr, 0, MRDUMP_MINI_HEADER_SIZE +
			sizeof(struct aee_process_info));
	fill_elf_header(&mrdump_mini_ehdr->ehdr, MRDUMP_MINI_NR_SECTION);
}

int mrdump_mini_init(void)
{
	int i;
	unsigned long size, offset;
	struct pt_regs regs;

	mrdump_mini_elf_header_init();

	fill_psinfo(&mrdump_mini_ehdr->psinfo.data);
	fill_note_S(&mrdump_mini_ehdr->psinfo.note, "vmlinux", NT_PRPSINFO,
		    sizeof(struct elf_prpsinfo));

	memset_io(&regs, 0, sizeof(struct pt_regs));
	for (i = 0; i < AEE_MTK_CPU_NUMS + 1; i++) {
		fill_prstatus(&mrdump_mini_ehdr->prstatus[i].data, &regs,
				NULL, i);
		fill_note_S(&mrdump_mini_ehdr->prstatus[i].note, "NA",
				NT_PRSTATUS, sizeof(struct elf_prstatus));
	}

	offset = offsetof(struct mrdump_mini_elf_header, psinfo);
	size = sizeof(mrdump_mini_ehdr->psinfo) +
		sizeof(mrdump_mini_ehdr->prstatus);
	fill_elf_note_phdr(&mrdump_mini_ehdr->phdrs[0], size, offset);

	for (i = 0; i < MRDUMP_MINI_NR_MISC; i++)
		fill_note_L(&mrdump_mini_ehdr->misc[i].note, "NA", 0,
			    sizeof(struct mrdump_mini_elf_misc));
	mrdump_mini_build_elf_misc();
	fill_elf_note_phdr(&mrdump_mini_ehdr->phdrs[1],
			sizeof(mrdump_mini_ehdr->misc),
			offsetof(struct mrdump_mini_elf_header, misc));

	if (mrdump_cblock) {

		mrdump_mini_add_entry_ext(
		  (unsigned long)mrdump_cblock,
		  (unsigned long)mrdump_cblock + mrdump_sram_cb.size,
		  mrdump_sram_cb.start_addr
		);

		mrdump_mini_add_entry(
		  (unsigned long)mrdump_cblock,
		  sizeof(struct mrdump_control_block) + 2 * PAGE_SIZE
		);

		mrdump_mini_add_entry(
		  ((unsigned long) &kallsyms_addresses +
		  (mrdump_cblock->machdesc.kallsyms.size / 2 - PAGE_SIZE)),
		  mrdump_cblock->machdesc.kallsyms.size + 2 * PAGE_SIZE);
	}

	return 0;
}

int mini_rdump_reserve_memory(struct reserved_mem *rmem)
{
	pr_info("[memblock]%s: 0x%llx - 0x%llx (0x%llx)\n",
		"mediatek,minirdump",
		 (unsigned long long)rmem->base,
		 (unsigned long long)rmem->base +
		 (unsigned long long)rmem->size,
		 (unsigned long long)rmem->size);
	return 0;
}

RESERVEDMEM_OF_DECLARE(reserve_memory_minirdump, "mediatek,minirdump",
		       mini_rdump_reserve_memory);

/* 0644: S_IRUGO | S_IWUSR */
module_param(dump_all_cpus, bool, 0644);

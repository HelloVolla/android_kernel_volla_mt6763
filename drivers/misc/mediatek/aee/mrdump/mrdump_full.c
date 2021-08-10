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

#include <stdarg.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <mt-plat/aee.h>
#include <linux/elf.h>
#include <linux/elfcore.h>
#include <linux/kallsyms.h>
#include <linux/miscdevice.h>
#include <mt-plat/aee.h>
#include <mt-plat/mtk_ram_console.h>
#include <linux/reboot.h>
#include <linux/stacktrace.h>
#include <linux/vmalloc.h>
#include <linux/elfcore.h>
#include <linux/kexec.h>
#include <asm/pgtable.h>
#include <asm/processor.h>
#include <mtk_wd_api.h>
#if defined(CONFIG_FIQ_GLUE)
#include <mt-plat/fiq_smp_call.h>
#endif
#include <mrdump.h>
#include <linux/kdebug.h>
#include "mrdump_private.h"
#ifdef CONFIG_MTK_WATCHDOG
#include <ext_wd_drv.h>
#endif

#define KEXEC_NOTE_HEAD_BYTES ALIGN(sizeof(struct elf_note), 4)
#define KEXEC_CORE_NOTE_NAME "CORE"
#define KEXEC_CORE_NOTE_NAME_BYTES ALIGN(sizeof(KEXEC_CORE_NOTE_NAME), 4)
#define KEXEC_CORE_NOTE_DESC_BYTES ALIGN(sizeof(struct elf_prstatus), 4)
#define KEXEC_NOTE_BYTES ((KEXEC_NOTE_HEAD_BYTES * 2) +		\
			  KEXEC_CORE_NOTE_NAME_BYTES +		\
			  KEXEC_CORE_NOTE_DESC_BYTES)
typedef u32 note_buf_t[KEXEC_NOTE_BYTES / 4];

static int crashing_cpu;

static note_buf_t __percpu *crash_notes;

static unsigned long mrdump_output_lbaooo;

static char mrdump_lk[12];

static u32 *append_elf_note(u32 *buf, char *name, unsigned int type,
				void *data, size_t data_len)
{
	struct elf_note note;

	note.n_namesz = strlen(name) + 1;
	note.n_descsz = data_len;
	note.n_type = type;
	memcpy(buf, &note, sizeof(note));
	buf += (sizeof(note) + 3) / 4;
	memcpy(buf, name, note.n_namesz);
	buf += (note.n_namesz + 3) / 4;
	memcpy(buf, data, note.n_descsz);
	buf += (note.n_descsz + 3) / 4;

	return buf;
}

static void final_note(u32 *buf)
{
	struct elf_note note;

	note.n_namesz = 0;
	note.n_descsz = 0;
	note.n_type = 0;
	memcpy(buf, &note, sizeof(note));
}

static void crash_save_cpu(struct pt_regs *regs, int cpu)
{
	struct elf_prstatus prstatus;
	u32 *buf;

	if ((cpu < 0) || (cpu >= nr_cpu_ids))
		return;
	if (!crash_notes)
		return;
	buf = (u32 *)per_cpu_ptr(crash_notes, cpu);
	if (!buf)
		return;
	memset(&prstatus, 0, sizeof(prstatus));
	prstatus.pr_pid = current->pid;
	elf_core_copy_kernel_regs((elf_gregset_t *)&prstatus.pr_reg, regs);
	buf = append_elf_note(buf, KEXEC_CORE_NOTE_NAME, NT_PRSTATUS,
			      &prstatus, sizeof(prstatus));
	final_note(buf);
}

#if defined(CONFIG_FIQ_GLUE)

static void aee_kdump_cpu_stop(void *arg, void *regs, void *svc_sp)
{
	struct mrdump_crash_record *crash_record = &mrdump_cblock->crash_record;
	int cpu = 0;

	register int sp asm("sp");
	struct pt_regs *ptregs = (struct pt_regs *)regs;
	void *creg;

	asm volatile("mov %0, %1\n\t"
		     "mov fp, %2\n\t"
		     : "=r" (sp)
		     : "r" (svc_sp), "r" (ptregs->ARM_fp)
		);
	cpu = get_HW_cpuid();

	if (cpu >= 0) {
		elf_core_copy_kernel_regs(
			(elf_gregset_t *)&crash_record->cpu_regs[cpu], ptregs);
		crash_save_cpu((struct pt_regs *)regs, cpu);

		creg = (void *)&crash_record->cpu_creg[cpu];

		mrdump_save_control_register(creg);
	}

	local_fiq_disable();
	local_irq_disable();

	dis_D_inner_flush_all();
	while (1)
		cpu_relax();
}

static void __mrdump_reboot_stop_all(struct mrdump_crash_record *crash_record)
{
	int timeout;

	fiq_smp_call_function(aee_kdump_cpu_stop, NULL, 0);

	/* Wait up to two second for other CPUs to stop */
	timeout = 2 * USEC_PER_SEC;
	while (num_online_cpus() > 1 && timeout--)
		udelay(1);
}

#else

/* Generic IPI support */
static atomic_t waiting_for_crash_ipi;

static void mrdump_stop_noncore_cpu(void *unused)
{
	struct mrdump_crash_record *crash_record = &mrdump_cblock->crash_record;
	struct pt_regs regs;
	void *creg;
	int cpu = get_HW_cpuid();

	if (cpu >= 0) {
		mrdump_save_current_backtrace(&regs);

		elf_core_copy_kernel_regs(
			(elf_gregset_t *)&crash_record->cpu_regs[cpu], &regs);
		crash_save_cpu((struct pt_regs *)&regs, cpu);

		creg = (void *)&crash_record->cpu_creg[cpu];

		mrdump_save_control_register(creg);
	}

	local_fiq_disable();
	local_irq_disable();

	dis_D_inner_flush_all();
	while (1)
		cpu_relax();
}

static void __mrdump_reboot_stop_all(struct mrdump_crash_record *crash_record)
{
	unsigned long msecs;

	atomic_set(&waiting_for_crash_ipi, num_online_cpus() - 1);
	smp_call_function(mrdump_stop_noncore_cpu, NULL, false);

	msecs = 1000; /* Wait at most a second for the other cpus to stop */
	while ((atomic_read(&waiting_for_crash_ipi) > 0) && msecs) {
		mdelay(1);
		msecs--;
	}
	if (atomic_read(&waiting_for_crash_ipi) > 0) {
		if (aee_in_nested_panic())
			aee_nested_printf(
				"Non-crashing CPUs did not react to IPI\n");
		else
			pr_notice("Non-crashing CPUs did not react to IPI\n");
	}
}

#endif

void mrdump_save_ctrlreg(int cpu)
{
	struct mrdump_crash_record *crash_record;
	void *creg;

	if (mrdump_cblock && cpu >= 0) {
		crash_record = &mrdump_cblock->crash_record;
		creg = (void *)&crash_record->cpu_creg[cpu];
		mrdump_save_control_register(creg);
	}
}

void mrdump_save_per_cpu_reg(int cpu, struct pt_regs *regs)
{
	struct mrdump_crash_record *crash_record;

	if (regs) {
		crash_save_cpu(regs, cpu);

		if (mrdump_cblock) {
			crash_record = &mrdump_cblock->crash_record;
			elf_core_copy_kernel_regs(
				(elf_gregset_t *)&crash_record->cpu_regs[cpu],
				regs
			);
		}
	}
}

void __mrdump_create_oops_dump(enum AEE_REBOOT_MODE reboot_mode,
		struct pt_regs *regs, const char *msg, ...)
{
	va_list ap;
	struct mrdump_crash_record *crash_record;
	void *creg;
	int cpu;

	if (mrdump_cblock) {
		crash_record = &mrdump_cblock->crash_record;

		local_irq_disable();
		local_fiq_disable();

#if defined(CONFIG_SMP)
		if ((reboot_mode != AEE_REBOOT_MODE_WDT) &&
		    (reboot_mode != AEE_REBOOT_MODE_GZ_WDT))
			__mrdump_reboot_stop_all(crash_record);
#endif

		cpu = get_HW_cpuid();
		if (cpu >= 0) {
			crashing_cpu = cpu;
			/* null regs, no register dump */
			if (regs) {
				crash_save_cpu(regs, cpu);
				elf_core_copy_kernel_regs(
					(elf_gregset_t *)
					&crash_record->cpu_regs[cpu],
					regs);
			}

			creg = (void *)&crash_record->cpu_creg[cpu];
			mrdump_save_control_register(creg);
		}

		va_start(ap, msg);
		vsnprintf(crash_record->msg, sizeof(crash_record->msg), msg,
				ap);
		va_end(ap);

		crash_record->fault_cpu = cpu;

		/* FIXME: Check reboot_mode is valid */
		crash_record->reboot_mode = reboot_mode;
	}
}

int __init mrdump_full_init(void)
{
	if (mrdump_cblock == NULL) {
		memset(mrdump_lk, 0, sizeof(mrdump_lk));
		pr_notice("%s: MT-RAMDUMP no control block\n", __func__);
		return -EINVAL;
	}

	/* Allocate memory for saving cpu registers. */
	crash_notes = alloc_percpu(note_buf_t);
	if (!crash_notes) {
		pr_notice("MT-RAMDUMP: Memory allocation for saving cpu register failed\n");
		return -ENOMEM;
	}

	if (strcmp(mrdump_lk, MRDUMP_GO_DUMP) != 0) {
		pr_notice("%s: MT-RAMDUMP init failed, lk version %s not matched.\n",
				__func__, mrdump_lk);
		return -EINVAL;
	}

	mrdump_cblock->enabled = MRDUMP_ENABLE_COOKIE;
	__inner_flush_dcache_all();
	pr_info("%s: MT-RAMDUMP enabled done\n", __func__);
	return 0;
}

#if CONFIG_SYSFS

static ssize_t mrdump_version_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", MRDUMP_GO_DUMP);
}

static struct kobj_attribute mrdump_version_attribute =
	__ATTR(version, 0600, mrdump_version_show, NULL);

static struct attribute *attrs[] = {
	&mrdump_version_attribute.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static int __init mrdump_sysfs_init(void)
{
	struct kobject *kobj;

	kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
	if (kobj) {
		if (sysfs_create_group(kobj, &attr_group)) {
			pr_notice("MT-RAMDUMP: sysfs create sysfs failed\n");
			return -ENOMEM;
		}
	} else {
		pr_notice("MT-RAMDUMP: Cannot find module %s object\n",
				KBUILD_MODNAME);
		return -EINVAL;
	}

	pr_info("%s: done.\n", __func__);
	return 0;
}

module_init(mrdump_sysfs_init);

#endif

static int param_set_mrdump_lbaooo(const char *val,
		const struct kernel_param *kp)
{
	int retval = 0;

	if (mrdump_cblock) {
		retval = param_set_ulong(val, kp);

		if (retval == 0) {
			mrdump_cblock->output_fs_lbaooo = mrdump_output_lbaooo;
			__inner_flush_dcache_all();
		}
	}

	return retval;
}

/* 0444: S_IRUGO */
module_param_string(lk, mrdump_lk, sizeof(mrdump_lk), 0444);

/* sys/modules/mrdump/parameter/lbaooo */
struct kernel_param_ops param_ops_mrdump_lbaooo = {
	.set = param_set_mrdump_lbaooo,
	.get = param_get_ulong,
};

param_check_ulong(lbaooo, &mrdump_output_lbaooo);
/* 0644: S_IRUGO | S_IWUSR */
module_param_cb(lbaooo, &param_ops_mrdump_lbaooo, &mrdump_output_lbaooo,
		0644);
__MODULE_PARM_TYPE(lbaooo, unsigned long);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MediaTek MRDUMP module");
MODULE_AUTHOR("MediaTek Inc.");


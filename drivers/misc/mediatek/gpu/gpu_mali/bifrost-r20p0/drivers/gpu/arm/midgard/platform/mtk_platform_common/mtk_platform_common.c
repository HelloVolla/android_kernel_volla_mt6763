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

#include <mali_kbase.h>
#include <mali_kbase_mem.h>

#include <linux/proc_fs.h>

#include <platform/mtk_platform_common.h>
#ifdef ENABLE_COMMON_DVFS
#include "mtk_gpufreq.h"
#endif /* ENABLE_COMMON_DVFS */
#include <mali_kbase_pm_internal.h>

#include <ged_log.h>
#include <ged_dvfs.h>

#include <linux/workqueue.h>

/* on:1, off:0 */
int g_vgpu_power_on_flag;
DEFINE_MUTEX(g_flag_lock);

int g_mtk_gpu_efuse_set_already;

#ifdef ENABLE_MTK_MEMINFO
int g_mtk_gpu_total_memory_usage_in_pages_debugfs;
static struct mtk_gpu_meminfo_type g_mtk_gpu_meminfo[MTK_MEMINFO_SIZE];

void mtk_kbase_gpu_memory_debug_init(void)
{
	mtk_dump_gpu_memory_usage_fp = mtk_kbase_dump_gpu_memory_usage;
	mtk_get_gpu_memory_usage_fp = mtk_kbase_report_gpu_memory_usage;
}

void mtk_kbase_gpu_memory_debug_remove(void)
{
	mtk_dump_gpu_memory_usage_fp = NULL;
	mtk_get_gpu_memory_usage_fp = NULL;
}

void mtk_kbase_reset_gpu_meminfo(void)
{
	int i = 0;

	for (i = 0; i < MTK_MEMINFO_SIZE; i++) {
		g_mtk_gpu_meminfo[i].pid = 0;
		g_mtk_gpu_meminfo[i].used_pages = 0;
	}
}

void mtk_kbase_set_gpu_meminfo(ssize_t index, int pid, int used_pages)
{
	g_mtk_gpu_meminfo[index].pid = pid;
	g_mtk_gpu_meminfo[index].used_pages = used_pages;
}

KBASE_EXPORT_TEST_API(mtk_kbase_dump_gpu_memory_usage)
bool mtk_kbase_dump_gpu_memory_usage(void)
{
	int i = 0;

	/* output the total memory usage and cap for this device */
	pr_warn("%10s\t%16s\n", "PID", "GPU Memory by Page");
	pr_warn("============================\n");

	for (i = 0; (i < MTK_MEMINFO_SIZE) && (g_mtk_gpu_meminfo[i].pid != 0); i++) {
		pr_warn("%10d\t%16d\n", g_mtk_gpu_meminfo[i].pid,
										g_mtk_gpu_meminfo[i].used_pages);
		}

	pr_warn("============================\n");
	pr_warn("%10s\t%16u\n",
			"Total",
			g_mtk_gpu_total_memory_usage_in_pages_debugfs);
	pr_warn("============================\n");
	return true;
}

KBASE_EXPORT_TEST_API(mtk_kbase_report_gpu_memory_usage)
unsigned int mtk_kbase_report_gpu_memory_usage(void)
{
	return (g_mtk_gpu_total_memory_usage_in_pages_debugfs * 4096);
}
#endif /* ENABLE_MTK_MEMINFO */

#ifdef CONFIG_PROC_FS

/* 0. For query the support command */
static int proc_gpu_help_show(struct seq_file *m, void *v)
{
	seq_puts(m, "======================================================================\n");
	seq_puts(m, "A.For Query GPU/CPU related Command:\n");
	seq_puts(m, "	 cat /proc/mali/utilization\n");
	seq_puts(m, "	 cat /proc/mali/frequency\n");
	seq_puts(m, "	 cat /proc/mali/memory_usage\n");
	seq_puts(m, "======================================================================\n");
	seq_puts(m, "B.For Fix GPU Frequency:\n");
	seq_puts(m, "	 echo > (450000, 280000) /proc/gpufreq/gpufreq_opp_freq\n");
	seq_puts(m, "	 echo 0 > /proc/gpufreq/gpufreq_opp_freq(re-enable GPU DVFS)\n");
	seq_puts(m, "C.For Turn On/Off CPU core number:\n");
	seq_puts(m, "	 echo (1, 0) > /sys/devices/system/cpu/cpu1/online\n");
	seq_puts(m, "	 echo (1, 0) > /sys/devices/system/cpu/cpu2/online\n");
	seq_puts(m, "	 echo (1, 0) > /sys/devices/system/cpu/cpuN/online\n");
	seq_puts(m, "D.For CPU Performance mode(Force CPU to run at highest speed:\n");
	seq_puts(m, " echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor\n");
	seq_puts(m, " echo interactive > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor(re-enable CPU DVFS)\n");
	seq_puts(m, "==============================================================================================\n");
	seq_puts(m, "E.For GPU advanced debugging command:\n");
	seq_puts(m, " echo [dvfs_freq(ms)] > /proc/mali/dvfs_freq\n");
	seq_puts(m, " echo [dvfs_thr_max] [dvfs_thr_min] > /proc/mali/dvfs_threshold\n");
	seq_puts(m, " echo [dvfs_deferred_count] > /proc/mali/dvfs_deferred_count\n");
	seq_puts(m, "==============================================================================================\n");

		return 0;
}

static int kbasep_gpu_help_debugfs_open(struct inode *in, struct file *file)
{
	return single_open(file, proc_gpu_help_show, NULL);
}

static const struct file_operations kbasep_gpu_help_debugfs_fops = {
	.open	 = kbasep_gpu_help_debugfs_open,
	.read	 = seq_read,
	.llseek	 = seq_lseek,
	.release = single_release,
};

/* 1. For GPU memory usage */
static int proc_gpu_memoryusage_show(struct seq_file *m, void *v)
{
	ssize_t ret = 0;

#ifdef ENABLE_MTK_MEMINFO
	int i = 0;
	int total_size_in_bytes;

	total_size_in_bytes = mtk_kbase_report_gpu_memory_usage();

	/* output the total memory usage and cap for this device */
	seq_printf(m, "%10s\t%16s\n", "PID", "GPU Memory by Page");
	seq_puts(m, "============================\n");

	for (i = 0; (i < MTK_MEMINFO_SIZE) && (g_mtk_gpu_meminfo[i].pid != 0); i++) {
		seq_printf(m, "%10d\t%16d\n", g_mtk_gpu_meminfo[i].pid,
		g_mtk_gpu_meminfo[i].used_pages);
	}

	seq_puts(m, "============================\n");
	seq_printf(m, "%10s\t%16u(%u bytes)\n", "Total",
			g_mtk_gpu_total_memory_usage_in_pages_debugfs, total_size_in_bytes);
	seq_puts(m, "============================\n");
#endif /* ENABLE_MTK_MEMINFO */

	return ret;
}

static int kbasep_gpu_memoryusage_debugfs_open(struct inode *in, struct file *file)
{
	return single_open(file, proc_gpu_memoryusage_show, NULL);
}

static const struct file_operations kbasep_gpu_memory_usage_debugfs_open = {
	.open	 = kbasep_gpu_memoryusage_debugfs_open,
	.read	 = seq_read,
	.llseek	 = seq_lseek,
	.release = single_release,
};

/* 2. For GL/CL utilization */
static int proc_gpu_utilization_show(struct seq_file *m, void *v)
{
#ifdef ENABLE_COMMON_DVFS
	unsigned long gl, cl0, cl1;
	unsigned int iCurrentFreq;

	iCurrentFreq = mt_gpufreq_get_cur_freq_index();

	gl = kbasep_get_gl_utilization();
	cl0 = kbasep_get_cl_js0_utilization();
	cl1 = kbasep_get_cl_js1_utilization();

	seq_printf(m, "gpu/cljs0/cljs1=%lu/%lu/%lu, frequency index=%d power(0:off, 1:0n):%d\n",
		gl, cl0, cl1, iCurrentFreq, mtk_get_vgpu_power_on_flag());
#else
	seq_puts(m, "GPU DVFS doesn't support\n");
#endif /* ENABLE_COMMON_DVFS */

	return 0;
}

static int kbasep_gpu_utilization_debugfs_open(struct inode *in, struct file *file)
{
	return single_open(file, proc_gpu_utilization_show, NULL);
}

static const struct file_operations kbasep_gpu_utilization_debugfs_fops = {
	.open	 = kbasep_gpu_utilization_debugfs_open,
	.read	 = seq_read,
	.llseek	 = seq_lseek,
	.release = single_release,
};

/* 3. For query GPU frequency index */
static int proc_gpu_frequency_show(struct seq_file *m, void *v)
{
#ifdef ENABLE_COMMON_DVFS
	unsigned int iCurrentFreq;

	iCurrentFreq = mt_gpufreq_get_cur_freq_index();

	seq_printf(m, "GPU Frequency Index: %u\n", iCurrentFreq);
#else
	seq_puts(m, "GPU DVFS doesn't support\n");
#endif /* ENABLE_COMMON_DVFS */

	return 0;
}

static int kbasep_gpu_frequency_debugfs_open(struct inode *in, struct file *file)
{
	return single_open(file, proc_gpu_frequency_show, NULL);
}

static const struct file_operations kbasep_gpu_frequency_debugfs_fops = {
	.open	 = kbasep_gpu_frequency_debugfs_open,
	.read	 = seq_read,
	.llseek	 = seq_lseek,
	.release = single_release,
};

/* 4. For query GPU dynamically enable DVFS */
static int g_dvfs_enabled = 1;

static int mtk_kbase_is_gpu_dvfs_enabled(void)
{
	return g_dvfs_enabled;
}

static int proc_gpu_dvfs_enabled_show(struct seq_file *m, void *v)
{
	int dvfs_enabled;

	dvfs_enabled = mtk_kbase_is_gpu_dvfs_enabled();
	seq_printf(m, "dvfs_enabled: %d\n", dvfs_enabled);
	return 0;
}

static int kbasep_gpu_dvfs_enable_debugfs_open(struct inode *in, struct file *file)
{
	return single_open(file, proc_gpu_dvfs_enabled_show, NULL);
}

static ssize_t kbasep_gpu_dvfs_enable_write(struct file *file, const char __user *buffer,
		size_t count, loff_t *data)
{
	char desc[32];
	int len = 0;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;

	desc[len] = '\0';

	if (!strncmp(desc, "1", 1))
		g_dvfs_enabled = 1;
	else if (!strncmp(desc, "0", 1))
		g_dvfs_enabled = 0;
	else if (!strncmp(desc, "2", 1))
		g_dvfs_enabled = 2;

	return count;
}

static const struct file_operations kbasep_gpu_dvfs_enable_debugfs_fops = {
	.open	 = kbasep_gpu_dvfs_enable_debugfs_open,
	.read	 = seq_read,
	.write	 = kbasep_gpu_dvfs_enable_write,
	.release = single_release,
};

/* 5. For GPU Always On */
static int g_always_on;

int mtk_kbase_is_gpu_always_on(void)
{
	return g_always_on;
}

static int proc_gpu_always_on_show(struct seq_file *m, void *v)
{
	int always_on;

	always_on = mtk_kbase_is_gpu_always_on();
	seq_printf(m, "always_on: %d\n", always_on);
	return 0;
}

static int kbasep_gpu_always_on_debugfs_open(struct inode *in, struct file *file)
{
	return single_open(file, proc_gpu_always_on_show, NULL);
}

static ssize_t kbasep_gpu_always_on_write(struct file *file, const char __user *buffer,
		size_t count, loff_t *data)
{
	char desc[32];
	int len = 0;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;

	desc[len] = '\0';

	if (!strncmp(desc, "1", 1))
		g_always_on = 1;
	else
		g_always_on = 0;

	return count;
}

static const struct file_operations kbasep_gpu_always_on_debugfs_fops = {
	.open	 = kbasep_gpu_always_on_debugfs_open,
	.read	 = seq_read,
	.write	 = kbasep_gpu_always_on_write,
	.release = single_release,
};

/* 6. For GPU Debug Log */
static int g_debug_log;

int mtk_kbase_gpu_debug_log(void)
{
	return g_debug_log;
}

static int proc_gpu_debug_log_show(struct seq_file *m, void *v)
{
	int debug_log;

	debug_log = mtk_kbase_gpu_debug_log();
	seq_printf(m, "debug_log: %d\n", debug_log);
	return 0;
}

static int kbasep_gpu_debug_log_debugfs_open(struct inode *in, struct file *file)
{
	return single_open(file, proc_gpu_debug_log_show, NULL);
}

static ssize_t kbasep_gpu_debug_log_write(struct file *file, const char __user *buffer,
		size_t count, loff_t *data)
{
	char desc[32];
	int len = 0;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;

	desc[len] = '\0';

	if (!strncmp(desc, "1", 1))
		g_debug_log = 1;
	else
		g_debug_log = 0;

	return count;
}

static const struct file_operations kbasep_gpu_debug_log_debugfs_fops = {
	.open	 = kbasep_gpu_debug_log_debugfs_open,
	.read	 = seq_read,
	.write	 = kbasep_gpu_debug_log_write,
	.release = single_release,
};


static struct proc_dir_entry *mali_pentry;

static struct workqueue_struct	   *g_aee_workqueue;
static struct work_struct		   g_aee_work;
static int g_aee_called;
static void aee_Handle(struct work_struct *_psWork)
{
}
void mtk_trigger_aee_report(const char *msg)
{
	pr_err("trigger aee: %s (aee warnning)", msg);
	if (g_aee_called == 0) {
		if (g_aee_workqueue) {
			g_aee_called = 1;
			queue_work(g_aee_workqueue, &g_aee_work);
		}
	}
}

static struct work_struct		   g_pa_work;
static u64 g_pa;
static void pa_Handle(struct work_struct *_psWork)
{
}
void mtk_trigger_emi_report(u64 pa)
{
	g_pa = pa;
	pr_err("emi mpu violation: pa: 0x%llx, dump registers and trigger check_pa", pa);
	queue_work(g_aee_workqueue, &g_pa_work);
}

void proc_mali_register(void)
{
	mali_pentry = proc_mkdir("mali", NULL);

	if (!mali_pentry)
		return;

	g_aee_workqueue = alloc_ordered_workqueue("mali_aeewp", WQ_FREEZABLE | WQ_MEM_RECLAIM);
	INIT_WORK(&g_aee_work, aee_Handle);
	INIT_WORK(&g_pa_work, pa_Handle);

	proc_create("help", 0, mali_pentry, &kbasep_gpu_help_debugfs_fops);
	proc_create("memory_usage", 0, mali_pentry, &kbasep_gpu_memory_usage_debugfs_open);
	proc_create("utilization", 0, mali_pentry, &kbasep_gpu_utilization_debugfs_fops);
	proc_create("frequency", 0, mali_pentry, &kbasep_gpu_frequency_debugfs_fops);
	proc_create("dvfs_enable", S_IRUGO | S_IWUSR, mali_pentry, &kbasep_gpu_dvfs_enable_debugfs_fops);
	proc_create("always_on", S_IRUGO | S_IWUSR, mali_pentry, &kbasep_gpu_always_on_debugfs_fops);
	proc_create("debug_log", S_IRUGO | S_IWUSR, mali_pentry, &kbasep_gpu_debug_log_debugfs_fops);
}

void proc_mali_unregister(void)
{
	if (!mali_pentry)
		return;

	remove_proc_entry("help", mali_pentry);
	remove_proc_entry("memory_usage", mali_pentry);
	/*	  remove_proc_entry("utilization", mali_pentry);*/
	/*	  remove_proc_entry("frequency", mali_pentry);*/
	remove_proc_entry("dvfs_enable", mali_pentry);
	/*	  remove_proc_entry("mali", NULL);*/
	mali_pentry = NULL;
}
#else
#define proc_mali_register() do {} while (0)
#define proc_mali_unregister() do {} while (0)
#endif /* CONFIG_PROC_FS */

int mtk_get_vgpu_power_on_flag(void)
{
	return g_vgpu_power_on_flag;
}

int mtk_set_vgpu_power_on_flag(int power_on_id)
{
	mutex_lock(&g_flag_lock);
	g_vgpu_power_on_flag = power_on_id;
	mutex_unlock(&g_flag_lock);

	return 0;
}

int mtk_set_mt_gpufreq_target(int freq_id)
{
#ifdef ENABLE_COMMON_DVFS
	int ret = 0;

	mutex_lock(&g_flag_lock);

	if (mtk_get_vgpu_power_on_flag() == MTK_VGPU_POWER_ON)
		ret = mt_gpufreq_target(freq_id);

	mutex_unlock(&g_flag_lock);

	return ret;
#else
	pr_alert("MALI: GPU DVFS doesn't support\n");
#endif /* ENABLE_COMMON_DVFS */

	return 0;
}

unsigned long mtk_get_ged_dvfs_last_commit_idx(void)
{
	return ged_dvfs_get_last_commit_idx();
}

int mtk_common_init(struct platform_device *pdev, struct kbase_device *kbdev)
{
	int ret = 0;

	mtk_mfg_counter_init();

	return ret;
}

int mtk_common_deinit(struct platform_device *pdev, struct kbase_device *kbdev)
{
	int ret = 0;

	mtk_mfg_counter_destroy();

	return ret;
}


/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/ring_buffer.h>
#include <linux/trace_events.h>
#include "mtk_ftrace.h"
#include "trace.h"

#ifdef CONFIG_MTK_PERF_TRACKER
#include <mt-plat/perf_tracker.h>
#endif

#ifdef CONFIG_MTK_KERNEL_MARKER
static unsigned long __read_mostly mark_addr;
static bool kernel_marker_on = true;

static inline void update_tracing_mark_write_addr(void)
{
	if (unlikely(mark_addr == 0))
		mark_addr = kallsyms_lookup_name("tracing_mark_write");
}

inline void trace_begin(char *name)
{
	if (unlikely(kernel_marker_on) && name) {
		preempt_disable();
		event_trace_printk(mark_addr, "B|%d|%s\n",
				   current->tgid, name);
		preempt_enable();
	}
}
EXPORT_SYMBOL(trace_begin);

inline void trace_counter(char *name, int count)
{
	if (unlikely(kernel_marker_on) && name) {
		preempt_disable();
		event_trace_printk(mark_addr, "C|%d|%s|%d\n",
				   current->tgid, name, count);
		preempt_enable();
	}
}
EXPORT_SYMBOL(trace_counter);

inline void trace_end(void)
{
	if (unlikely(kernel_marker_on)) {
		preempt_disable();
		event_trace_printk(mark_addr, "E\n");
		preempt_enable();
	}
}
EXPORT_SYMBOL(trace_end);

static ssize_t
kernel_marker_on_simple_read(struct file *filp, char __user *ubuf,
			     size_t cnt, loff_t *ppos)
{
	char buf[64];
	int r;

	r = sprintf(buf, "%d\n", kernel_marker_on);

	return simple_read_from_buffer(ubuf, cnt, ppos, buf, r);
}

static ssize_t
kernel_marker_on_simple_write(struct file *filp, const char __user *ubuf,
			      size_t cnt, loff_t *ppos)
{
	unsigned long val;
	int ret;

	ret = kstrtoul_from_user(ubuf, cnt, 10, &val);
	if (ret)
		return ret;

	kernel_marker_on = !!val;

	if (kernel_marker_on && !mark_addr)
		update_tracing_mark_write_addr();

	(*ppos)++;

	return cnt;
}

static const struct file_operations kernel_marker_on_simple_fops = {
	.open = tracing_open_generic,
	.read = kernel_marker_on_simple_read,
	.write = kernel_marker_on_simple_write,
	.llseek = default_llseek,
};

static __init int init_kernel_marker(void)
{
	struct trace_array *tr;
	struct dentry *d_tracer;

	tr  = top_trace_array();
	d_tracer = tracing_init_dentry();

	if (!tr || IS_ERR(d_tracer))
		return 0;
	trace_create_file("kernel_marker_on", 0644, d_tracer, tr,
			  &kernel_marker_on_simple_fops);
	update_tracing_mark_write_addr();
	return 0;
}

fs_initcall(init_kernel_marker);
#endif

#if defined(CONFIG_MTK_HIBERNATION) && defined(CONFIG_MTK_SCHED_TRACERS)
int resize_ring_buffer_for_hibernation(int enable)
{
	int ret = 0;
	struct trace_array *tr = NULL;

	if (enable) {
		ring_buffer_expanded = 0;
		ret = tracing_update_buffers();
	} else {
		tr = top_trace_array();
		if (!tr)
			return -ENODEV;
		ret = tracing_resize_ring_buffer(tr, 0, RING_BUFFER_ALL_CPUS);
	}

	return ret;
}
#endif

#ifdef CONFIG_MTK_SCHED_TRACERS
static unsigned long buf_size = 25165824UL;
static bool boot_trace;
static __init int boot_trace_cmdline(char *str)
{
	boot_trace = true;
	update_buf_size(buf_size);
	return 0;
}
__setup("androidboot.boot_trace", boot_trace_cmdline);

/* If boot tracing is on.Ignore tracing off command.*/
bool boot_ftrace_check(unsigned long trace_en)
{
	bool boot_complete = false;

	if (boot_trace != true || trace_en)
		return false;

#ifdef CONFIG_MTPROF
	boot_complete = boot_finish;
#endif
	if (!boot_complete) {
		pr_info("Capturing boot ftrace,Ignore tracing off.\n");
		return true;
	}
	return false;
}

#include <linux/rtc.h>

void print_enabled_events(struct trace_buffer *buf, struct seq_file *m)
{
	struct trace_event_call *call;
	struct trace_event_file *file;
	struct trace_array *tr;

	unsigned long usec_rem;
	unsigned long long t;
	struct rtc_time tm_utc, tm;
	struct timeval tv = { 0 };

	if (buf->tr)
		tr = buf->tr;
	else
		return;

	if (tr->name != NULL)
		seq_printf(m, "# instance: %s, enabled events:", tr->name);
	else
		seq_puts(m, "# enabled events:");

	list_for_each_entry(file, &tr->events, list) {
		call = file->event_call;
		if (file->flags & EVENT_FILE_FL_ENABLED)
			seq_printf(m, " %s:%s", call->class->system,
				   trace_event_name(call));
	}

	seq_puts(m, "\n");

	t = sched_clock();
	do_gettimeofday(&tv);
	t = ns2usecs(t);
	usec_rem = do_div(t, USEC_PER_SEC);
	rtc_time_to_tm(tv.tv_sec, &tm_utc);
	rtc_time_to_tm(tv.tv_sec - sys_tz.tz_minuteswest * 60, &tm);

	seq_printf(m, "# kernel time now: %5llu.%06lu\n",
		   t, usec_rem);
	seq_printf(m, "# UTC time:\t%d-%02d-%02d %02d:%02d:%02d.%03u\n",
			tm_utc.tm_year + 1900, tm_utc.tm_mon + 1,
			tm_utc.tm_mday, tm_utc.tm_hour,
			tm_utc.tm_min, tm_utc.tm_sec,
			(unsigned int)tv.tv_usec);
	seq_printf(m, "# android time:\t%d-%02d-%02d %02d:%02d:%02d.%03u\n",
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec,
			(unsigned int)tv.tv_usec);
}

/* ftrace's switch function for MTK solution */
static void ftrace_events_enable(int enable)
{
	if (enable) {
		trace_set_clr_event(NULL, "sched_switch", 1);
		trace_set_clr_event(NULL, "sched_wakeup", 1);
		trace_set_clr_event(NULL, "sched_wakeup_new", 1);
#ifdef CONFIG_SMP
		trace_set_clr_event(NULL, "sched_migrate_task", 1);
#endif
		trace_set_clr_event(NULL, "workqueue_execute_start", 1);
		trace_set_clr_event(NULL, "workqueue_execute_end", 1);
		trace_set_clr_event(NULL, "cpu_frequency", 1);

		trace_set_clr_event(NULL, "block_bio_frontmerge", 1);
		trace_set_clr_event(NULL, "block_bio_backmerge", 1);
		trace_set_clr_event(NULL, "block_rq_issue", 1);
		trace_set_clr_event(NULL, "block_rq_insert", 1);
		trace_set_clr_event(NULL, "block_rq_complete", 1);
		trace_set_clr_event(NULL, "block_rq_requeue", 1);
		trace_set_clr_event(NULL, "debug_allocate_large_pages", 1);
		trace_set_clr_event(NULL, "dump_allocate_large_pages", 1);
		trace_set_clr_event("mtk_events", NULL, 1);

		if (boot_trace) {
			trace_set_clr_event("android_fs", NULL, 1);
			trace_set_clr_event(NULL, "sched_blocked_reason", 1);
			/*trace_set_clr_event(NULL, "sched_waking", 1);*/
		} else {
			trace_set_clr_event("ipi", NULL, 1);
			trace_set_clr_event(NULL, "softirq_entry", 1);
			trace_set_clr_event(NULL, "softirq_exit", 1);
			trace_set_clr_event(NULL, "softirq_raise", 1);
			trace_set_clr_event(NULL, "irq_handler_entry", 1);
			trace_set_clr_event(NULL, "irq_handler_exit", 1);
#ifdef CONFIG_MTK_SCHED_MONITOR
			trace_set_clr_event(NULL, "sched_mon_msg", 1);
#endif
#ifdef CONFIG_LOCKDEP
			trace_set_clr_event(NULL, "lock_dbg", 1);
			trace_set_clr_event(NULL, "lock_monitor_msg", 1);
#endif
			trace_set_clr_event("met_bio", NULL, 1);
			trace_set_clr_event("met_fuse", NULL, 1);
		}

		tracing_on();
	} else {
		tracing_off();
		trace_set_clr_event(NULL, NULL, 0);
	}
}

static __init int boot_ftrace(void)
{
	struct trace_array *tr;
	int ret;

	if (boot_trace) {
#ifdef CONFIG_MTK_PERF_TRACKER
		perf_tracker_enable(1);
#endif
		tr = top_trace_array();
		ret = tracing_update_buffers();
		if (ret != 0)
			pr_debug("unable to expand buffer, ret=%d\n", ret);
#ifdef CONFIG_SCHEDSTATS
		force_schedstat_enabled();
#endif
		ftrace_events_enable(1);
		set_tracer_flag(tr, TRACE_ITER_OVERWRITE, 0);
		pr_debug("[ftrace]boot-time profiling...\n");
	}
	return 0;
}
core_initcall(boot_ftrace);

#ifdef CONFIG_MTK_FTRACE_DEFAULT_ENABLE
static __init int enable_ftrace(void)
{
	int ret;

	if (!boot_trace) {
		/* enable ftrace facilities */
		ftrace_events_enable(1);

		/*
		 * only update buffer eariler
		 * if we want to collect boot-time ftrace
		 * to avoid the boot time impacted by
		 * early-expanded ring buffer
		 */
		ret = tracing_update_buffers();
		if (ret != 0)
			pr_debug("fail to update buffer, ret=%d\n",
				 ret);
		else
			pr_debug("[ftrace]ftrace ready...\n");
	}
	return 0;
}
late_initcall(enable_ftrace);
#endif
#endif


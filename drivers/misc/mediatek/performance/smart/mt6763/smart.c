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

#include <linux/jiffies.h>
#include <linux/time.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/kallsyms.h>
#include <linux/utsname.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/uaccess.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/notifier.h>
#include <linux/suspend.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/kobject.h>
#include <linux/kthread.h>

#include <linux/ring_buffer.h>
#include <linux/trace_events.h>
#include <trace.h>

#include <linux/platform_device.h>
#include <mt-plat/met_drv.h>
#include <linux/sched.h>
#include "mtk_devinfo.h"
#include "smart.h"

#define SEQ_printf(m, x...)                                       \
	do {                                                          \
		if (m)                                                    \
			seq_printf(m, x);                                     \
		else                                                      \
			pr_debug(x);                                          \
	} while (0)
#undef TAG
#define TAG "[SMART]"

struct smart_det {
	spinlock_t smart_lock;
	wait_queue_head_t wq;
	struct task_struct *thread;
	int smart_event;
	struct timer_list tmr_list;
	atomic_t event;
};

/*--------------------------------------------*/

static struct smart_det tsmart;
#define SMART_TIMER_INTERVAL_MS (40)

#define CLUSTER_NUM (2)

static int turbo_support;
static int log_enable;
static int trace_enable;
static int uevent_enable;
/* Is foreground enter sports mode? set from perfservice */
static int app_is_sports;
static int app_is_running; /* Is app running */
static unsigned long app_load_thresh;
static unsigned long app_tlp_thresh;
static unsigned long app_btask_thresh;
static unsigned long app_up_times;
static unsigned long app_down_times;

static unsigned long app_up_count;
static unsigned long app_down_count;

static unsigned long turbo_util_thresh;
static int turbo_mode_enable;
static int force_isolate;
struct cpumask turbo_cpus;

struct smart_data {
	int is_hps_heavy;
	unsigned long check_duration;
	unsigned long valid_duration;
	int is_app_sports;
};

struct smart_context {
	struct input_dev *idev;
	struct miscdevice mdev;
	struct mutex s_op_mutex;
	struct smart_data s_data;
};

struct smart_context *smart_context_obj;


/******* FLIPER SETTING *********/

static ssize_t mt_app_is_sports_write(struct file *filp, const char *ubuf,
				      size_t cnt, loff_t *data)
{
	int ret;
	unsigned long arg;

	if (!kstrtoul_from_user(ubuf, cnt, 0, &arg)) {
		ret = 0;
		if (arg == 0)
			app_is_sports = 0;
		else if (arg == 1)
			app_is_sports = 1;
	} else
		ret = -EINVAL;

	return (ret < 0) ? ret : cnt;
}

static int mt_app_is_sports_show(struct seq_file *m, void *v)
{
	if (app_is_sports)
		SEQ_printf(m, "1\n");
	else
		SEQ_printf(m, "0\n");
	return 0;
}

static ssize_t mt_app_is_running_write(struct file *filp, const char *ubuf,
				       size_t cnt, loff_t *data)
{
	int ret;
	unsigned long arg;

	if (!kstrtoul_from_user(ubuf, cnt, 0, &arg)) {
		ret = 0;
		if (arg == 0)
			app_is_running = 0;
		else if (arg == 1)
			app_is_running = 1;
	} else
		ret = -EINVAL;

	return (ret < 0) ? ret : cnt;
}

static int mt_app_is_running_show(struct seq_file *m, void *v)
{
	if (app_is_running)
		SEQ_printf(m, "1\n");
	else
		SEQ_printf(m, "0\n");
	return 0;
}

static ssize_t mt_app_load_thresh_write(struct file *filp, const char *ubuf,
					size_t cnt, loff_t *data)
{
	int ret;
	unsigned long arg;

	if (!kstrtoul_from_user(ubuf, cnt, 0, &arg)) {
		ret = 0;
		app_load_thresh = arg;
	} else
		ret = -EINVAL;

	return (ret < 0) ? ret : cnt;
}

static int mt_app_load_thresh_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%lu\n", app_load_thresh);
	return 0;
}

static ssize_t mt_app_tlp_thresh_write(struct file *filp, const char *ubuf,
				       size_t cnt, loff_t *data)
{
	int ret;
	unsigned long arg;

	if (!kstrtoul_from_user(ubuf, cnt, 0, &arg)) {
		ret = 0;
		app_tlp_thresh = arg;
	} else
		ret = -EINVAL;

	return (ret < 0) ? ret : cnt;
}

static int mt_app_tlp_thresh_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%lu\n", app_tlp_thresh);
	return 0;
}

static ssize_t mt_app_btask_thresh_write(struct file *filp, const char *ubuf,
					 size_t cnt, loff_t *data)
{
	int ret;
	unsigned long arg;

	if (!kstrtoul_from_user(ubuf, cnt, 0, &arg)) {
		ret = 0;
		app_btask_thresh = arg;
	} else
		ret = -EINVAL;

	return (ret < 0) ? ret : cnt;
}

static int mt_app_btask_thresh_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%lu\n", app_btask_thresh);
	return 0;
}

static ssize_t mt_app_up_times_write(struct file *filp, const char *ubuf,
				     size_t cnt, loff_t *data)
{
	int ret;
	unsigned long arg;

	if (!kstrtoul_from_user(ubuf, cnt, 0, &arg)) {
		ret = 0;
		app_up_times = arg;
	} else
		ret = -EINVAL;

	return (ret < 0) ? ret : cnt;
}

static int mt_app_up_times_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%lu\n", app_up_times);
	return 0;
}

static ssize_t mt_app_down_times_write(struct file *filp, const char *ubuf,
				       size_t cnt, loff_t *data)
{
	int ret;
	unsigned long arg;

	if (!kstrtoul_from_user(ubuf, cnt, 0, &arg)) {
		ret = 0;
		app_down_times = arg;
	} else
		ret = -EINVAL;

	return (ret < 0) ? ret : cnt;
}

static int mt_app_down_times_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%lu\n", app_down_times);
	return 0;
}

static ssize_t mt_turbo_util_thresh_write(struct file *filp, const char *ubuf,
					  size_t cnt, loff_t *data)
{
	int ret;
	unsigned long arg;

	if (!kstrtoul_from_user(ubuf, cnt, 0, &arg)) {
		ret = 0;
		turbo_util_thresh = arg;
	} else
		ret = -EINVAL;

	return (ret < 0) ? ret : cnt;
}

static int mt_turbo_util_thresh_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%lu\n", turbo_util_thresh);
	return 0;
}

static ssize_t mt_smart_turbo_support_write(struct file *filp,
					    const char *ubuf, size_t cnt,
					    loff_t *data)
{
	int ret;
	unsigned long arg;

	if (!kstrtoul_from_user(ubuf, cnt, 0, &arg)) {
		ret = 0;
		if (arg == 0)
			turbo_support = 0;
		if (arg == 1)
			turbo_support = 1;
	} else
		ret = -EINVAL;

	return (ret < 0) ? ret : cnt;
}

static int mt_smart_turbo_support_show(struct seq_file *m, void *v)
{
	if (turbo_support)
		SEQ_printf(m, "1\n");
	else
		SEQ_printf(m, "0\n");
	return 0;
}

static ssize_t mt_smart_force_isolate_write(struct file *filp,
					    const char *ubuf, size_t cnt,
					    loff_t *data)
{
	int ret;
	unsigned long arg;

	if (!kstrtoul_from_user(ubuf, cnt, 0, &arg)) {
		ret = 0;
		if (arg == 0) {
			force_isolate = 0;
			unset_cpu_isolation(ISO_TURBO);
		}
		if (arg == 1) {
			force_isolate = 1;
			if (turbo_support)
				set_cpu_isolation(ISO_TURBO, &turbo_cpus);
		}
	} else
		ret = -EINVAL;

	return (ret < 0) ? ret : cnt;
}

static int mt_smart_force_isolate_show(struct seq_file *m, void *v)
{
	if (force_isolate)
		SEQ_printf(m, "1\n");
	else
		SEQ_printf(m, "0\n");
	return 0;
}

static ssize_t mt_smart_log_enable_write(struct file *filp, const char *ubuf,
					 size_t cnt, loff_t *data)
{
	int ret;
	unsigned long arg;

	if (!kstrtoul_from_user(ubuf, cnt, 0, &arg)) {
		ret = 0;
		if (arg == 0)
			log_enable = 0;
		if (arg == 1)
			log_enable = 1;
	} else
		ret = -EINVAL;

	return (ret < 0) ? ret : cnt;
}

static int mt_smart_log_enable_show(struct seq_file *m, void *v)
{
	if (log_enable)
		SEQ_printf(m, "1\n");
	else
		SEQ_printf(m, "0\n");
	return 0;
}

static ssize_t mt_smart_trace_enable_write(struct file *filp, const char *ubuf,
					   size_t cnt, loff_t *data)
{
	int ret;
	unsigned long arg;

	if (!kstrtoul_from_user(ubuf, cnt, 0, &arg)) {
		ret = 0;
		if (arg == 0)
			trace_enable = 0;
		if (arg == 1)
			trace_enable = 1;
	} else
		ret = -EINVAL;

	return (ret < 0) ? ret : cnt;
}

static int mt_smart_trace_enable_show(struct seq_file *m, void *v)
{
	if (trace_enable)
		SEQ_printf(m, "1\n");
	else
		SEQ_printf(m, "0\n");
	return 0;
}

static ssize_t mt_smart_uevent_enable_write(struct file *filp,
					    const char *ubuf, size_t cnt,
					    loff_t *data)
{
	int ret;
	unsigned long arg;

	if (!kstrtoul_from_user(ubuf, cnt, 0, &arg)) {
		ret = 0;
		if (arg == 0)
			uevent_enable = 0;
		if (arg == 1)
			uevent_enable = 1;
	} else
		ret = -EINVAL;

	return (ret < 0) ? ret : cnt;
}

static int mt_smart_uevent_enable_show(struct seq_file *m, void *v)
{
	if (uevent_enable)
		SEQ_printf(m, "1\n");
	else
		SEQ_printf(m, "0\n");
	return 0;
}

/*** Seq operation of mtprof ****/
static int mt_app_is_sports_open(struct inode *inode, struct file *file)
{
	return single_open(file, mt_app_is_sports_show, inode->i_private);
}

static const struct file_operations mt_app_is_sports_fops = {
	.open = mt_app_is_sports_open,
	.write = mt_app_is_sports_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int mt_app_is_running_open(struct inode *inode, struct file *file)
{
	return single_open(file, mt_app_is_running_show, inode->i_private);
}

static const struct file_operations mt_app_is_running_fops = {
	.open = mt_app_is_running_open,
	.write = mt_app_is_running_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int mt_app_load_thresh_open(struct inode *inode, struct file *file)
{
	return single_open(file, mt_app_load_thresh_show, inode->i_private);
}

static const struct file_operations mt_app_load_thresh_fops = {
	.open = mt_app_load_thresh_open,
	.write = mt_app_load_thresh_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int mt_app_tlp_thresh_open(struct inode *inode, struct file *file)
{
	return single_open(file, mt_app_tlp_thresh_show, inode->i_private);
}

static const struct file_operations mt_app_tlp_thresh_fops = {
	.open = mt_app_tlp_thresh_open,
	.write = mt_app_tlp_thresh_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int mt_app_btask_thresh_open(struct inode *inode, struct file *file)
{
	return single_open(file, mt_app_btask_thresh_show, inode->i_private);
}

static const struct file_operations mt_app_btask_thresh_fops = {
	.open = mt_app_btask_thresh_open,
	.write = mt_app_btask_thresh_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int mt_app_up_times_open(struct inode *inode, struct file *file)
{
	return single_open(file, mt_app_up_times_show, inode->i_private);
}

static const struct file_operations mt_app_up_times_fops = {
	.open = mt_app_up_times_open,
	.write = mt_app_up_times_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int mt_app_down_times_open(struct inode *inode, struct file *file)
{
	return single_open(file, mt_app_down_times_show, inode->i_private);
}

static const struct file_operations mt_app_down_times_fops = {
	.open = mt_app_down_times_open,
	.write = mt_app_down_times_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int mt_turbo_util_thresh_open(struct inode *inode, struct file *file)
{
	return single_open(file, mt_turbo_util_thresh_show, inode->i_private);
}

static const struct file_operations mt_turbo_util_thresh_fops = {
	.open = mt_turbo_util_thresh_open,
	.write = mt_turbo_util_thresh_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int mt_smart_turbo_support_open(struct inode *inode, struct file *file)
{
	return single_open(file, mt_smart_turbo_support_show,
			   inode->i_private);
}

static const struct file_operations mt_smart_turbo_support_fops = {
	.open = mt_smart_turbo_support_open,
	.write = mt_smart_turbo_support_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int mt_smart_force_isolate_open(struct inode *inode, struct file *file)
{
	return single_open(file, mt_smart_force_isolate_show,
			   inode->i_private);
}

static const struct file_operations mt_smart_force_isolate_fops = {
	.open = mt_smart_force_isolate_open,
	.write = mt_smart_force_isolate_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int mt_smart_log_enable_open(struct inode *inode, struct file *file)
{
	return single_open(file, mt_smart_log_enable_show, inode->i_private);
}

static const struct file_operations mt_smart_log_enable_fops = {
	.open = mt_smart_log_enable_open,
	.write = mt_smart_log_enable_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int mt_smart_trace_enable_open(struct inode *inode, struct file *file)
{
	return single_open(file, mt_smart_trace_enable_show, inode->i_private);
}

static const struct file_operations mt_smart_trace_enable_fops = {
	.open = mt_smart_trace_enable_open,
	.write = mt_smart_trace_enable_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int mt_smart_uevent_enable_open(struct inode *inode, struct file *file)
{
	return single_open(file, mt_smart_uevent_enable_show,
			   inode->i_private);
}

static const struct file_operations mt_smart_uevent_enable_fops = {
	.open = mt_smart_uevent_enable_open,
	.write = mt_smart_uevent_enable_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

#if 0
int smart_enter_turbo_mode(void)
{
#ifdef CONFIG_MTK_ACAO_SUPPORT
	return (turbo_support && turbo_mode_enable);
#else
	return 0;
#endif /* CONFIG_MTK_ACAO_SUPPORT */
}
#endif

void mt_smart_update_sysinfo(unsigned int cur_loads, unsigned int cur_tlp,
			     unsigned int btask, unsigned int htask)
{
	int ret, prev_enable;
	char event_running[9] = "DETECT=8";  /* running */
	char event_act_up[9] = "ACTION=1";   /* up	*/
	char event_act_down[9] = "ACTION=0"; /* down */
	char *envp_up[3] = {event_running, event_act_up, NULL};
	char *envp_down[3] = {event_running, event_act_down, NULL};
	unsigned long ll_util = 0, ll_cap = 0;

	if (!uevent_enable)
		return;

	if (log_enable)
		pr_debug(TAG
			"get_sysinfo sports:%d, cur_load:%d, cur_tlp:%d, btask:%d, htask:%d\n",
			app_is_sports, cur_loads, cur_tlp, btask, htask);

	if (app_is_sports == 1) { /* foreground app enters sports mode  */

		/* check turbo mode */
		if (turbo_support) {
			sched_get_cluster_util(0, &ll_util, &ll_cap);
			prev_enable = turbo_mode_enable;
			if (htask == 1 &&
			    (ll_util < ll_cap * turbo_util_thresh / 100)) {
				turbo_mode_enable = 1;
				if (turbo_mode_enable != prev_enable) {
					if (log_enable)
						pr_debug(TAG
						    "turbo_mode_enable:%d (htask:%d, ll_util:%lu, ll_cap:%lu, thresh:%lu)",
						    turbo_mode_enable, htask,
						    ll_util, ll_cap,
						    turbo_util_thresh);
					set_cpu_isolation(ISO_TURBO,
						&turbo_cpus);
				}
			} else {
				turbo_mode_enable = 0;
				if (turbo_mode_enable != prev_enable) {
					if (log_enable)
						pr_debug(TAG
						    "turbo_mode_enable:%d (htask:%d, ll_util:%lu, ll_cap:%lu, thresh:%lu)",
						    turbo_mode_enable, htask,
						    ll_util, ll_cap,
						    turbo_util_thresh);
					unset_cpu_isolation(ISO_TURBO);
				}
			}
		}

		/* APP is not running */
		if (app_is_running == 0) {
			if (cur_loads >= app_load_thresh &&
			    btask >= app_btask_thresh)
				app_up_count++;
			else
				app_up_count = 0;

			if (app_up_count >= app_up_times) {
				pr_debug(TAG
					 "get_sysinfo APP is running!!!\n");
				app_is_running = 1;
				if (smart_context_obj) {
					ret = kobject_uevent_env(
					    &smart_context_obj->mdev
						 .this_device->kobj,
					    KOBJ_CHANGE, envp_up);
					if (ret) {
						pr_debug(TAG
							"kobject_uevent error:%d\n",
							ret);
						return;
					}
				}
			}
		} else { /* app_is_running == 1 */
			if (cur_loads < app_load_thresh ||
			    btask < app_btask_thresh)
				app_down_count++;
			else
				app_down_count = 0;

			if (app_down_count >= app_down_times) {
				pr_debug(TAG
				    "get_sysinfo APP is not running!!!\n");
				app_is_running = 0;
				if (smart_context_obj) {
					ret = kobject_uevent_env(
					    &smart_context_obj->mdev
						 .this_device->kobj,
					    KOBJ_CHANGE, envp_down);
					if (ret)
						pr_debug(TAG
							"kobject_uevent error:%d\n",
							ret);
				}
			}
		}
	} else {
		if (turbo_mode_enable)
			unset_cpu_isolation(ISO_TURBO);
		turbo_mode_enable = 0; /* reset */
	}
}

static void smart_get_sysinfo(unsigned int *loads, unsigned int *tlp,
			      unsigned int *btask, unsigned int *htask)
{
	unsigned int cpu;
	unsigned int rel_load, abs_load, total_load = 0;
	unsigned int big_task_L, big_task_B;
	int lastpoll_htask1 = 0, lastpoll_htask2 = 0;
	int avg_htask = 0, avg_htask_scal = 0;
	int max;
	int heavy_task_threshold = get_heavy_task_threshold();
	int avg_heavy_task_threshold = get_avg_heavy_task_threshold();
	unsigned int total_htask = 0;
	int scaled_tlp = 0;
	unsigned int avg_tlp, iowait_avg;

/* TLP */

#ifdef CONFIG_MTK_SCHED_RQAVG_KS
	scaled_tlp =
	    sched_get_nr_running_avg((int *)&avg_tlp, (int *)&iowait_avg);
#else
	avg = 0;
	iowait_avg = 0;
#endif
	*tlp = max_t(int, scaled_tlp, (int)avg_tlp);

	/* loading */
	for_each_online_cpu(cpu) {
		if (cpu < 8) {
			sched_get_percpu_load2(cpu, 1, &rel_load, &abs_load);
			if (cpu < 4)
				total_load += rel_load;
			else
				total_load += abs_load;
		}
	}
	*loads = total_load;

	/* btask */
	sched_big_task_nr(&big_task_L, &big_task_B);
	*btask = big_task_L + big_task_B;

/* htask */
#ifdef CONFIG_MTK_SCHED_RQAVG_US
	/* cluster 0 */
	total_htask +=
	    sched_get_nr_heavy_task_by_threshold(0, heavy_task_threshold);

	/* in cluster L, heavy task by max of average(w/o remainder) and
	 * last_poll
	 */
	lastpoll_htask1 =
	    sched_get_nr_heavy_task_by_threshold(1, heavy_task_threshold);
	lastpoll_htask2 = sched_get_nr_heavy_running_avg(1, &avg_htask_scal);

	avg_htask = ((avg_htask_scal % 100) >= avg_heavy_task_threshold)
			? (avg_htask_scal / 100 + 1)
			: (avg_htask_scal / 100);

	max = max(max(lastpoll_htask1, lastpoll_htask2), avg_htask);
	total_htask += max;
	*htask = total_htask;
#else
	*htask = 0;
#endif
}

/*
 * smart timer callback
 */
static int _smart_timer_callback(unsigned long data)
{
	unsigned long flags;

	mod_timer(&tsmart.tmr_list,
		  (jiffies + msecs_to_jiffies(SMART_TIMER_INTERVAL_MS)));

	spin_lock_irqsave(&tsmart.smart_lock, flags);
	tsmart.smart_event = 0;
	spin_unlock_irqrestore(&tsmart.smart_lock, flags);

	atomic_inc(&tsmart.event);
	wake_up(&tsmart.wq);
	return HRTIMER_NORESTART;
}

/******* SMART THREAD *********/
static int tsmart_thread(void *ptr)
{
	unsigned long flags;
	int event;
	unsigned int cur_loads, cur_tlp, btask, htask;

	set_user_nice(current, -10);

	while (!kthread_should_stop()) {

		while (!atomic_read(&tsmart.event))
			wait_event(tsmart.wq, atomic_read(&tsmart.event));
		atomic_dec(&tsmart.event);

		spin_lock_irqsave(&tsmart.smart_lock, flags);
		event = tsmart.smart_event;
		spin_unlock_irqrestore(&tsmart.smart_lock, flags);

		/*pr_debug(TAG"tsmart_thread event:%d\n", event); */

		smart_get_sysinfo(&cur_loads, &cur_tlp, &btask, &htask);
		mt_smart_update_sysinfo(cur_loads, cur_tlp, btask, htask);

		/*mod_timer(&tsmart.tmr_list, (jiffies +
		 * msecs_to_jiffies(SMART_TIMER_INTERVAL_MS)));
		 */
	}
	return 0;
}

/*--------------------INIT------------------------*/

static int __init init_smart(void)
{
	struct proc_dir_entry *pe;
	struct proc_dir_entry *smart_dir = NULL;

	/* poting */
	smart_dir = proc_mkdir("perfmgr/smart", NULL);

	pr_debug(TAG "init smart driver start\n");
	pe = proc_create("smart_turbo_support", 0644, smart_dir,
			 &mt_smart_turbo_support_fops);
	if (!pe)
		return -ENOMEM;
	pe = proc_create("smart_force_isolate", 0644, smart_dir,
			 &mt_smart_force_isolate_fops);
	if (!pe)
		return -ENOMEM;
	pe = proc_create("smart_log_enable", 0644, smart_dir,
			 &mt_smart_log_enable_fops);
	if (!pe)
		return -ENOMEM;
	pe = proc_create("smart_trace_enable", 0644, smart_dir,
			 &mt_smart_trace_enable_fops);
	if (!pe)
		return -ENOMEM;
	pe = proc_create("smart_uevent_enable", 0644, smart_dir,
			 &mt_smart_uevent_enable_fops);
	if (!pe)
		return -ENOMEM;
	pe = proc_create("app_is_sports", 0644, smart_dir,
			 &mt_app_is_sports_fops);
	if (!pe)
		return -ENOMEM;
	pe = proc_create("app_is_running", 0644, smart_dir,
			 &mt_app_is_running_fops);
	if (!pe)
		return -ENOMEM;
	pe = proc_create("app_load_thresh", 0644, smart_dir,
			 &mt_app_load_thresh_fops);
	if (!pe)
		return -ENOMEM;
	pe = proc_create("app_tlp_thresh", 0644, smart_dir,
			 &mt_app_tlp_thresh_fops);
	if (!pe)
		return -ENOMEM;
	pe = proc_create("app_btask_thresh", 0644, smart_dir,
			 &mt_app_btask_thresh_fops);
	if (!pe)
		return -ENOMEM;
	pe = proc_create("app_up_times", 0644, smart_dir,
			 &mt_app_up_times_fops);
	if (!pe)
		return -ENOMEM;
	pe = proc_create("app_down_times", 0644, smart_dir,
			 &mt_app_down_times_fops);
	if (!pe)
		return -ENOMEM;
	pe = proc_create("turbo_util_thresh", 0644, smart_dir,
			 &mt_turbo_util_thresh_fops);
	if (!pe)
		return -ENOMEM;

	app_is_sports = 0;
	app_is_running = 0;
	app_load_thresh = 100; /* loading should > 100 */
	app_tlp_thresh = 100;  /* don't use */
	app_btask_thresh = 1;  /* btask should >= 1 */
	app_up_times = 10;     /* 40ms * 10 = 400ms */
	app_down_times = 100;  /* 40ms * 10 = 4000ms */

	turbo_util_thresh = 80; /* util <= 80% */

	log_enable = 0;    /* debug log */
	trace_enable = 0;  /* debug trace */
	uevent_enable = 1; /* smart.c will send uevent to user space */
	turbo_mode_enable = 0;
	force_isolate = 0; /* test mode: force turbo mode isolation on */

	/* check turbo mode: bit 3 of index 54 */
	if (get_devinfo_with_index(30) == 0x10) /* 63n */
		turbo_support = 0;
	else
		turbo_support = (get_devinfo_with_index(54) & (1 << 3)) ? 1 : 0;

	spin_lock_init(&tsmart.smart_lock);
	init_waitqueue_head(&tsmart.wq);
	atomic_set(&tsmart.event, 0);
	tsmart.thread = (struct task_struct *)kthread_run(
	    tsmart_thread, &tsmart, "smart_det");
	if (IS_ERR(tsmart.thread))
		return -EINVAL;

#if 0
	init_timer(&tsmart.tmr_list);
#else
	init_timer_deferrable(&tsmart.tmr_list);
#endif
	tsmart.tmr_list.function = (void *)&_smart_timer_callback;
	tsmart.tmr_list.data = (unsigned long)&tsmart;
	tsmart.tmr_list.expires =
	    jiffies + msecs_to_jiffies(SMART_TIMER_INTERVAL_MS);
	add_timer(&tsmart.tmr_list);

	/* cpu mask for L plus */
	cpumask_clear(&turbo_cpus);
	cpumask_set_cpu(0, &turbo_cpus);
	cpumask_set_cpu(1, &turbo_cpus);
	cpumask_set_cpu(2, &turbo_cpus);
	cpumask_set_cpu(3, &turbo_cpus);
	cpumask_set_cpu(7, &turbo_cpus);

	pr_debug(TAG "init smart driver done\n");

	return 0;
}
late_initcall(init_smart);


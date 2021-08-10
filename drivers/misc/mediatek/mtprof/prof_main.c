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

#define DEBUG 1

#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/kallsyms.h>
#include <linux/utsname.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/delay.h>

#include <linux/pid.h>
#include "internal.h"

#include <linux/signal.h>
#include <trace/events/signal.h>

#ifdef CONFIG_MTK_ENG_BUILD

MT_DEBUG_ENTRY(log);
static unsigned long print_num;
static unsigned long long second = 1;

static int mt_log_show(struct seq_file *m, void *v)
{
	SEQ_printf(m, "Print %ld lines log in %lld second in last time.\n",
		print_num, second);
	SEQ_printf(m,
		"show: Please echo m n > log again. m: second, n: level.\n");
	return 0;
}

static ssize_t mt_log_write(struct file *filp,
				const char *ubuf,
				size_t cnt,
				loff_t *data)
{
	char buf[64];
	unsigned long long t1 = 0, t2 = 0;
	int level = 0;

	if (cnt >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(&buf, ubuf, cnt))
		return -EFAULT;

	buf[cnt] = 0;

	if (sscanf(buf, "%lld %d ", &second, &level) == 2) {
		SEQ_printf(NULL, "will print log in level %d",
			level);
		SEQ_printf(NULL, " about %lld second.\n",
			second);
	} else {
		SEQ_printf(NULL, "Please echo m n > log;");
		SEQ_printf(NULL, "m: second, n: level.\n");
		return cnt;
	}
	t1 = sched_clock();
	pr_info("printk debug log: start time: %lld.\n", t1);
	print_num = 0;
	for (;;) {
		t2 = sched_clock();
		if (t2 - t1 > second * 1000000000)
			break;
		pr_info("printk debug log: the %ld line, time: %lld.\n",
			print_num++, t2);
		switch (level) {
		case 0:
			break;
		case 1:
			__delay(1);
			break;
		case 2:
			__delay(5);
			break;
		case 3:
			__delay(10);
			break;
		case 4:
			__delay(50);
			break;
		case 5:
			__delay(100);
			break;
		case 6:
			__delay(200);
			break;
		case 7:
			__delay(500);
			break;
		case 8:
			__delay(1000);
			break;
		case 9:
			msleep(20);
			break;
		default:
			msleep(20);
			break;
		}
	}

	pr_info("mt log total write %ld line in %lld second.\n",
		print_num, second);
	return cnt;
}
#endif

/* 6. reboot pid*/
MT_DEBUG_ENTRY(pid);

int reboot_pid;
static int mt_pid_show(struct seq_file *m, void *v)
{
	SEQ_printf(m, "reboot pid %d.\n", reboot_pid);
	return 0;
}

static ssize_t mt_pid_write(struct file *filp, const char *ubuf,
	   size_t cnt, loff_t *data)
{
	char buf[10];
	unsigned long val = 0;
	int ret;
	struct task_struct *tsk;

	if (cnt >= sizeof(buf)) {
		pr_debug("mt_pid input stream size to large.\n");
		return -EINVAL;
	}

	if (copy_from_user(&buf, ubuf, cnt))
		return -EFAULT;
	buf[cnt] = 0;
	ret = kstrtoul(buf, 10, &val);

	reboot_pid = val;
	if (reboot_pid > PID_MAX_DEFAULT) {
		pr_debug("get reboot pid error %d.\n", reboot_pid);
		reboot_pid = 0;
		return -EFAULT;
	}
	pr_debug("get reboot pid: %d.\n", reboot_pid);

	if (reboot_pid > 1) {
		tsk = find_task_by_vpid(reboot_pid);
		if (tsk != NULL)
			pr_info("Reboot Process(%s:%d).\n",
				tsk->comm, tsk->pid);
	}

	return cnt;

}

#define STORE_SIGINFO(_errno, _code, info)			\
	do {							\
		if (info == SEND_SIG_NOINFO ||			\
		    info == SEND_SIG_FORCED) {			\
			_errno	= 0;				\
			_code	= SI_USER;			\
		} else if (info == SEND_SIG_PRIV) {		\
			_errno	= 0;				\
			_code	= SI_KERNEL;			\
		} else {					\
			_errno	= info->si_errno;		\
			_code	= info->si_code;		\
		}						\
	} while (0)

static const char * const signal_deliver_results[] = {
	"delivered",
	"ignored",
	"already_pending",
	"overflow_fail",
	"lost_info",
};

/* 7. signal logs */
MT_DEBUG_ENTRY(signal_log);

enum {
	SI_GENERATE = (1 << 0),
	SI_DELIVER  = (1 << 1),
} SI_LOG_MASK;

static const char stat_nam[] = TASK_STATE_TO_CHAR_STR;
static unsigned int enabled_signal_log;

static void probe_signal_generate(void *ignore, int sig, struct siginfo *info,
		struct task_struct *task, int group, int result)
{
	unsigned int state = task->state ? __ffs(task->state) + 1 : 0;
	int errno, code;

	/*
	 * only log delivered signals
	 */
	STORE_SIGINFO(errno, code, info);
	pr_debug("[signal][%d:%s]generate sig %d",
		 current->pid, current->comm, sig);
	pr_debug(" to [%d:%s:%c] errno=%d code=%d grp=%d res=%s\n",
		task->pid, task->comm,
		state < sizeof(stat_nam) - 1 ? stat_nam[state] : '?',
		errno, code, group, signal_deliver_results[result]);
}

static void probe_signal_deliver(void *ignore, int sig, struct siginfo *info,
		struct k_sigaction *ka)
{
	int errno, code;

	STORE_SIGINFO(errno, code, info);
	pr_debug("[signal]sig %d delivered to [%d:%s]",
		sig,
		current->pid,
		current->comm);
	pr_debug(" errno=%d code=%d sa_handler=%lx sa_flags=%lx\n",
		errno,
		code,
		(unsigned long)ka->sa.sa_handler,
		ka->sa.sa_flags);

}

static void probe_death_signal(void *ignore, int sig, struct siginfo *info,
		struct task_struct *task, int _group, int result)
{
	struct signal_struct *signal = task->signal;
	unsigned int state;
	int group;

	/*
	 * all action will cause process coredump or terminate
	 * kernel log reduction: only print delivered signals
	 */
	if (sig_fatal(task, sig) && result == TRACE_SIGNAL_DELIVERED) {
		signal = task->signal;
		group = _group ||
		(signal->flags & (SIGNAL_GROUP_EXIT | SIGNAL_GROUP_COREDUMP));
		/*
		 * kernel log reduction
		 * skip SIGRTMIN because it's used as timer signal
		 * skip if the target thread is already dead
		 */
		if (sig == SIGRTMIN ||
		    (task->state & (TASK_DEAD | EXIT_DEAD | EXIT_ZOMBIE)))
			return;
		/*
		 * Global init gets no signals it doesn't want.
		 * Container-init gets no signals it doesn't want from same
		 * container.
		 *
		 * Note that if global/container-init sees a sig_kernel_only()
		 * signal here, the signal must have been generated internally
		 * or must have come from an ancestor namespace. In either
		 * case, the signal cannot be dropped.
		 */
		if (unlikely(signal->flags & SIGNAL_UNKILLABLE) &&
				!sig_kernel_only(sig))
			return;

		/*
		 * kernel log reduction
		 * only print process instead of all threads
		 */
		if (group && (task != task->group_leader))
			return;

		state = task->state ? __ffs(task->state) + 1 : 0;
		pr_debug("[signal][%d:%s]send death sig %d to[%d:%s:%c]\n",
			 current->pid, current->comm,
			 sig, task->pid, task->comm,
			 state < sizeof(stat_nam) - 1 ? stat_nam[state] : '?');
	} else if ((sig_kernel_stop(sig) && result == TRACE_SIGNAL_DELIVERED) ||
		   sig == SIGCONT) {

		/*
		 * kernel log reduction
		 * only print process instead of all threads
		 */
		if (_group && (task != task->group_leader))
			return;

		state = task->state ? __ffs(task->state) + 1 : 0;
		pr_debug("[signal][%d:%s]send %s sig %d to[%d:%s:%c]\n",
			 current->pid, current->comm,
			 (sig == SIGCONT) ? "continue" : "stop",
			 sig, task->pid, task->comm,
			 state < sizeof(stat_nam) - 1 ? stat_nam[state] : '?');
	}
}

static int mt_signal_log_show(struct seq_file *m, void *v)
{
	SEQ_printf(m, "%d: debug message for signal being generated\n",
		SI_GENERATE);
	SEQ_printf(m, "%d: debug message for signal being delivered\n",
		SI_DELIVER);
	SEQ_printf(m, "%d: enable all logs\n", SI_GENERATE | SI_DELIVER);
	SEQ_printf(m, "%d\n", enabled_signal_log);
	return 0;
}

static ssize_t mt_signal_log_write(struct file *filp, const char *ubuf,
	   size_t cnt, loff_t *data)
{
	unsigned long val = 0;
	unsigned long update;
	int ret;

	ret = kstrtoul_from_user(ubuf, cnt, 10, &val);
	if (ret)
		return ret;

	update = enabled_signal_log ^ val;
	if (update & SI_GENERATE) {
		if (val & SI_GENERATE)
			register_trace_signal_generate(probe_signal_generate,
				NULL);
		else
			unregister_trace_signal_generate(probe_signal_generate,
				NULL);
	}
	if (update & SI_DELIVER) {
		if (val & SI_DELIVER)
			register_trace_signal_deliver(probe_signal_deliver,
				NULL);
		else
			unregister_trace_signal_deliver(probe_signal_deliver,
				NULL);
	}
	enabled_signal_log = val;

	return cnt;
}

static void __init init_signal_log(void)
{
	if (enabled_signal_log & SI_GENERATE)
		register_trace_signal_generate(probe_signal_generate, NULL);
	if (enabled_signal_log & SI_DELIVER)
		register_trace_signal_deliver(probe_signal_deliver, NULL);
	register_trace_signal_generate(probe_death_signal, NULL);
}

/* 8. fork & exit logs */
#include <trace/events/sched.h>

MT_DEBUG_ENTRY(fork_exit_log);

enum {
	DO_FORK = (1 << 0),
	DO_EXIT  = (1 << 1),
} FORK_EXIT_LOG_MASK;

static unsigned int enabled_fork_exit_log;

static void probe_sched_fork_time(void *ignore,
	struct task_struct *parent,
	struct task_struct *child, unsigned long long dur)
{
	char parent_comm[TASK_COMM_LEN], child_comm[TASK_COMM_LEN];
	pid_t parent_pid, child_pid;
	unsigned long long fork_time;

	memcpy(parent_comm, parent->comm, TASK_COMM_LEN);
	parent_pid = parent->pid;
	memcpy(child_comm, child->comm, TASK_COMM_LEN);
	child_pid = child->pid;
	fork_time = dur;

	pr_debug("[fork]comm=%s pid=%d fork child_comm=%s child_pid=%d, total fork time=%llu us",
		parent_comm, parent_pid, child_comm, child_pid, fork_time);

}

static void probe_sched_process_exit(void *ignore, struct task_struct *p)
{
	char comm[TASK_COMM_LEN];
	pid_t pid;
	int prio;

	memcpy(comm, p->comm, TASK_COMM_LEN);
	pid = p->pid;
	prio = p->prio;

	pr_debug("[exit]comm=%s pid=%d prio=%d exited", comm, pid, prio);

}

static int mt_fork_exit_log_show(struct seq_file *m, void *v)
{
	SEQ_printf(m, "%d: debug message for fork\n", DO_FORK);
	SEQ_printf(m, "%d: debug message for exit\n", DO_EXIT);
	SEQ_printf(m, "%d: enable all logs\n", DO_FORK | DO_EXIT);
	SEQ_printf(m, "%d\n", enabled_fork_exit_log);
	return 0;
}

static ssize_t mt_fork_exit_log_write(struct file *filp, const char *ubuf,
	   size_t cnt, loff_t *data)
{
	unsigned long val = 0;
	unsigned long update;
	int ret;

	ret = kstrtoul_from_user(ubuf, cnt, 10, &val);
	if (ret)
		return ret;

	update = enabled_fork_exit_log ^ val;

	if (update & DO_FORK) {
		if (val & DO_FORK)
			register_trace_sched_fork_time(probe_sched_fork_time,
				NULL);
		else
			unregister_trace_sched_fork_time(probe_sched_fork_time,
				NULL);
	}

	if (update & DO_EXIT) {
		if (val & DO_EXIT)
			register_trace_sched_process_exit(
				probe_sched_process_exit,
				NULL);
		else
			unregister_trace_sched_process_exit(
				probe_sched_process_exit,
				NULL);
	}
	enabled_fork_exit_log = val;

	return cnt;
}

static void __init init_fork_exit_log(void)
{
	if (enabled_fork_exit_log & DO_FORK)
		register_trace_sched_fork_time(probe_sched_fork_time, NULL);
	if (enabled_fork_exit_log & DO_EXIT)
		register_trace_sched_process_exit(probe_sched_process_exit,
			NULL);
}

/*-------------------------------------------------------------------*/
static int __init init_mtsched_prof(void)
{
	struct proc_dir_entry *pe;

	if (!proc_mkdir("mtprof", NULL))
		return -1;
	pe = proc_create("mtprof/reboot_pid", 0660, NULL, &mt_pid_fops);
	if (!pe)
		return -ENOMEM;
	init_signal_log();
	pe = proc_create("mtprof/signal_log", 0664, NULL, &mt_signal_log_fops);
	if (!pe)
		return -ENOMEM;

	init_fork_exit_log();
	pe = proc_create("mtprof/fork_exit_log", 0664, NULL,
		&mt_fork_exit_log_fops);
	if (!pe)
		return -ENOMEM;

#ifdef CONFIG_MTK_ENG_BUILD
	pe = proc_create("mtprof/log", 0664, NULL, &mt_log_fops);
	if (!pe)
		return -ENOMEM;
#endif
	return 0;
}

device_initcall(init_mtsched_prof);

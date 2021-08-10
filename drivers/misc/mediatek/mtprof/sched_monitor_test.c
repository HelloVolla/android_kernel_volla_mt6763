/*
 * Copyright (C) 2018 MediaTek Inc.
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

#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>

/* TIMER_SOFTIRQ duration warning test */

static void delayed_timer(unsigned long arg)
{
	mdelay(600);
}

void sched_mon_test_TIMER_SOFTIRQ(void)
{
	struct timer_list timer;

	setup_timer(&timer, delayed_timer, 0);
	timer.expires = jiffies + msecs_to_jiffies(10);
	add_timer(&timer);
	mdelay(3000);
}

/* TASKLET_SOFTIRQ duration warning test */

static struct tasklet_struct sched_mon_tasklet;
static void sched_mon_tasklet_func(unsigned long data)
{
	mdelay(600);
}
void sched_mon_test_TASKLET_SOFTIRQ(void)
{
	tasklet_init(&sched_mon_tasklet, sched_mon_tasklet_func, 0);
	tasklet_schedule(&sched_mon_tasklet);
}

/* RCU_SOFTIRQ duration warning test */

struct sched_mon_rcu_t {
	int val;
};
static struct sched_mon_rcu_t __rcu *sched_mon_rcu_g;
static struct rcu_head sched_mon_rcu_head;
static void delayed_rcu_callback(struct rcu_head *r)
{
	/* RCU_SOFTIRQ handler */
	mdelay(600);
}
void sched_mon_test_RCU_SOFTIRQ(void)
{
	struct sched_mon_rcu_t *sched_mon_rcu;

	sched_mon_rcu = kmalloc(sizeof(*sched_mon_rcu), GFP_KERNEL);
	if (sched_mon_rcu == NULL)
		return;
	sched_mon_rcu->val = 100;

	RCU_INIT_POINTER(sched_mon_rcu_g, sched_mon_rcu);
	call_rcu(&sched_mon_rcu_head, delayed_rcu_callback);
}

/* irq_work monitor test */

static struct irq_work sched_mon_irqwork;
static void sched_mon_irq_work(struct irq_work *work)
{
	mdelay(600);
}
void sched_mon_test_irq_work(void)
{
	init_irq_work(&sched_mon_irqwork, sched_mon_irq_work);
	irq_work_queue(&sched_mon_irqwork);
}

/* IRQ disable monitor test */

void sched_mon_test_irq_disable(void)
{
	unsigned long flags;

	local_irq_save(flags);
	mdelay(600);
	local_irq_restore(flags);

	/* The test will trigger KernelAPI Dump */
}

struct sched_mon_test_func {
	char name[32];
	void (*func)(void);
};

struct sched_mon_test_func sched_mon_test_list[] = {
	{"sirq_timer_dur", sched_mon_test_TIMER_SOFTIRQ},
	{"sirq_tasklet_dur", sched_mon_test_TASKLET_SOFTIRQ},
	{"sirq_rcu_dur", sched_mon_test_RCU_SOFTIRQ},
	{"irq_work_dur", sched_mon_test_irq_work},
	{"hirq_disable", sched_mon_test_irq_disable},
};

static ssize_t sched_mon_test_write(struct file *file,
	const char *ubuf, size_t count, loff_t *ppos)
{
	int i;
	char buf[32];

	if (count >= sizeof(buf) || count == 0)
		return -EINVAL;

	if (copy_from_user(&buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	for (i = 0; i < ARRAY_SIZE(sched_mon_test_list); i++) {
		if (!strncmp(buf, sched_mon_test_list[i].name,
			strlen(sched_mon_test_list[i].name)))
			sched_mon_test_list[i].func();
	}

	return count;
}

static const struct file_operations proc_sched_monitor_test_fops = {
	.open  = simple_open,
	.write = sched_mon_test_write,
};

void mt_sched_monitor_test_init(struct proc_dir_entry *dir)
{
	proc_create("sched_mon_test", 0220, dir,
		&proc_sched_monitor_test_fops);
}


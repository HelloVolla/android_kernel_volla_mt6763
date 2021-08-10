/*
 * Copyright (C) 2018 MediaTek Inc.
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

#include <trace/events/sched.h>
#include <trace/events/mtk_events.h>
#ifdef CONFIG_MTK_DRAMC
#include <mtk_dramc.h>
#endif
#include <linux/mm.h>
#include <linux/swap.h>
#include <mt-plat/mtk_blocktag.h>
#include <helio-dvfsrc.h>
#include <linux/jiffies.h>
#ifdef CONFIG_MTK_QOS_FRAMEWORK
#include <mtk_qos_sram.h>
#else
#include <helio-dvfsrc.h>
#endif
#include <mt-plat/perf_tracker.h>
#include <linux/topology.h>

#include <linux/module.h>
#ifndef __CHECKER__
#define CREATE_TRACE_POINTS
#include "perf_tracker_trace.h"
#endif

#include "perf_tracker.h"

#ifdef CONFIG_MTK_GAUGE_VERSION
#include <mt-plat/mtk_battery.h>

static void fuel_gauge_handler(struct work_struct *work);

static int fuel_gauge_enable;
static int fuel_gauge_delay; /* ms */
static DECLARE_DELAYED_WORK(fuel_gauge, fuel_gauge_handler);
#endif

static int perf_tracker_on, perf_tracker_init;
static DEFINE_MUTEX(perf_ctl_mutex);
static int cluster_nr = -1;

#if !defined(CONFIG_MTK_BLOCK_TAG) || !defined(MTK_BTAG_FEATURE_MICTX_IOSTAT)
struct mtk_btag_mictx_iostat_struct {
	__u64 duration;  /* duration time for below performance data (ns) */
	__u32 tp_req_r;  /* throughput (per-request): read  (KB/s) */
	__u32 tp_req_w;  /* throughput (per-request): write (KB/s) */
	__u32 tp_all_r;  /* throughput (overlapped) : read  (KB/s) */
	__u32 tp_all_w;  /* throughput (overlapped) : write (KB/s) */
	__u32 reqsize_r; /* request size : read  (Bytes) */
	__u32 reqsize_w; /* request size : write (Bytes) */
	__u32 reqcnt_r;  /* request count: read */
	__u32 reqcnt_w;  /* request count: write */
	__u16 wl;        /* storage device workload (%) */
	__u16 q_depth;   /* storage cmdq queue depth */
};
#endif

#ifdef CONFIG_MTK_BLOCK_TAG
static struct mtk_btag_mictx_iostat_struct iostatptr;

void  __attribute__((weak)) mtk_btag_mictx_enable(int enable) {}

int __attribute__((weak)) mtk_btag_mictx_get_data(
	struct mtk_btag_mictx_iostat_struct *io)
{
	return -1;
}
#endif

int perf_tracker_enable(int val)
{
	mutex_lock(&perf_ctl_mutex);

	val = (val > 0) ? 1 : 0;

	perf_tracker_on = val;
#ifdef CONFIG_MTK_BLOCK_TAG
	mtk_btag_mictx_enable(val);
#endif

	mutex_unlock(&perf_ctl_mutex);

	return (perf_tracker_on == val) ? 0 : -1;
}

#ifdef CONFIG_MTK_GAUGE_VERSION
static void fuel_gauge_handler(struct work_struct *work)
{
	int curr, volt;

	if (!fuel_gauge_enable)
		return;

	/* read current(mA) and valtage(mV) from pmic */
	curr = battery_get_bat_current();
	volt = battery_get_bat_voltage();

	trace_fuel_gauge(curr, volt);

	queue_delayed_work(system_power_efficient_wq,
			&fuel_gauge, msecs_to_jiffies(fuel_gauge_delay));
}
#endif

u32 __attribute__((weak)) dvfsrc_sram_read(u32 offset)
{
	return 0;
}

unsigned int __attribute__((weak)) get_dram_data_rate(void)
{
	return 0;
}

u32 __attribute__((weak)) qos_sram_read(u32 offset)
{
	return 0;
}

static inline u32 cpu_stall_ratio(int cpu)
{
#ifdef CM_STALL_RATIO_OFFSET
	return qos_sram_read(CM_STALL_RATIO_OFFSET + cpu * 4);
#else
	return 0;
#endif
}

#define K(x) ((x) << (PAGE_SHIFT - 10))
#define max_cpus 8

void __perf_tracker(u64 wallclock,
		  long mm_available,
		  long mm_free)
{
	int dram_rate = 0;
#ifdef CONFIG_MTK_BLOCK_TAG
	struct mtk_btag_mictx_iostat_struct *iostat = &iostatptr;
#endif
	int bw_c = 0, bw_g = 0, bw_mm = 0, bw_total = 0;
	int i;
	int stall[max_cpus] = {0};
	unsigned int sched_freq[3] = {0};
	int cid;

	if (!perf_tracker_on || !perf_tracker_init)
		return;

	/* dram freq */
	dram_rate = get_dram_data_rate();

	/* emi */
	bw_c  = qos_sram_read(QOS_DEBUG_1);
	bw_g  = qos_sram_read(QOS_DEBUG_2);
	bw_mm = qos_sram_read(QOS_DEBUG_3);
	bw_total = qos_sram_read(QOS_DEBUG_0);

	/* sched: cpu freq */
	for (cid = 0; cid < cluster_nr; cid++)
		sched_freq[cid] =
			mt_cpufreq_get_cur_freq(cid);

	/* trace for short msg */
	trace_perf_index_s(
			sched_freq[0], sched_freq[1], sched_freq[2],
			dram_rate, bw_c, bw_g, bw_mm, bw_total
			);

	if (!hit_long_check())
		return;

	/* free mem */
	mm_free = global_page_state(NR_FREE_PAGES);
	mm_available = si_mem_available();

#ifdef CONFIG_MTK_BLOCK_TAG
	/* IO stat */
	if (mtk_btag_mictx_get_data(iostat))
		memset(iostat, 0, sizeof(struct mtk_btag_mictx_iostat_struct));
#endif

	/* cpu stall ratio */
	for (i = 0; i < nr_cpu_ids || i < max_cpus; i++)
		stall[i] = cpu_stall_ratio(i);

	/* trace for long msg */

	trace_perf_index_l(
			K(mm_free),
			K(mm_available),
#ifdef CONFIG_MTK_BLOCK_TAG
			iostat->wl,
			iostat->tp_req_r, iostat->tp_all_r,
			iostat->reqsize_r, iostat->reqcnt_r,
			iostat->tp_req_w, iostat->tp_all_w,
			iostat->reqsize_w, iostat->reqcnt_w,
			iostat->duration, iostat->q_depth,
#else
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
#endif
			stall
			);
}

#ifdef CONFIG_MTK_GAUGE_VERSION
static ssize_t show_fuel_gauge_enable(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	unsigned int len = 0, max_len = 4096;

	len += snprintf(buf, max_len, "fuel_gauge_enable = %u\n",
			fuel_gauge_enable);
	return len;
}

static ssize_t store_fuel_gauge_enable(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int tmp;

	mutex_lock(&perf_ctl_mutex);

	if (kstrtouint(buf, 10, &tmp) == 0)
		fuel_gauge_enable = (tmp > 0) ? 1 : 0;

	if (fuel_gauge_enable) {
		/* default delay 8ms */
		fuel_gauge_delay = (fuel_gauge_delay > 0) ?
				fuel_gauge_delay : 8;

		/* start fuel gauge tracking */
		queue_delayed_work(system_power_efficient_wq,
				&fuel_gauge,
				msecs_to_jiffies(fuel_gauge_delay));
	}

	mutex_unlock(&perf_ctl_mutex);

	return count;
}

static ssize_t show_fuel_gauge_period(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	unsigned int len = 0, max_len = 4096;

	len += snprintf(buf, max_len, "fuel_gauge_period = %u(ms)\n",
				fuel_gauge_delay);
	return len;
}

static ssize_t store_fuel_gauge_period(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int tmp;

	mutex_lock(&perf_ctl_mutex);

	if (kstrtouint(buf, 10, &tmp) == 0)
		if (tmp > 0) /* ms */
			fuel_gauge_delay = tmp;

	mutex_unlock(&perf_ctl_mutex);

	return count;
}
#endif

/*
 * make perf tracker on
 * /sys/devices/system/cpu/perf/enable
 * 1: on
 * 0: off
 */
static ssize_t show_perf_enable(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	unsigned int len = 0;
	unsigned int max_len = 4096;

	len += snprintf(buf, max_len, "enable = %d\n",
			perf_tracker_on);
	return len;
}

static ssize_t store_perf_enable(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val = 0;

	if (sscanf(buf, "%iu", &val) != 0)
		perf_tracker_enable(val);

	return count;
}

static struct kobj_attribute perf_enable_attr =
__ATTR(enable, 0600, show_perf_enable, store_perf_enable);

#ifdef CONFIG_MTK_GAUGE_VERSION
static struct kobj_attribute perf_fuel_gauge_enable_attr =
__ATTR(fuel_gauge_enable, 0600,
	show_fuel_gauge_enable, store_fuel_gauge_enable);
static struct kobj_attribute perf_fuel_gauge_period_attr =
__ATTR(fuel_gauge_period, 0600,
	show_fuel_gauge_period, store_fuel_gauge_period);
#endif

static struct attribute *perf_attrs[] = {
	&perf_enable_attr.attr,
#ifdef CONFIG_MTK_GAUGE_VERSION
	&perf_fuel_gauge_enable_attr.attr,
	&perf_fuel_gauge_period_attr.attr,
#endif
	NULL,
};

static struct attribute_group perf_attr_group = {
	.attrs = perf_attrs,
};

static int init_perf_tracker(void)
{
	int ret = 0;
	struct kobject *kobj = NULL;

	perf_tracker_init = 1;
	cluster_nr = arch_get_nr_clusters();
	if (unlikely(cluster_nr <= 0 || cluster_nr > 3))
		cluster_nr = 3;

	kobj = kobject_create_and_add("perf", &cpu_subsys.dev_root->kobj);

	if (kobj) {
		ret = sysfs_create_group(kobj, &perf_attr_group);
		if (ret)
			kobject_put(kobj);
		else
			kobject_uevent(kobj, KOBJ_ADD);
	}

	return 0;
}
late_initcall_sync(init_perf_tracker);

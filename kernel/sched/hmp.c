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

#include <linux/sched.h>
#include <linux/stat.h>
#include <linux/math64.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <trace/events/sched.h>
#include <linux/stop_machine.h>

#ifdef CONFIG_SCHED_HMP
static int is_heavy_task(struct task_struct *p)
{
	return p->se.avg.loadwop_avg >= 650 ? 1 : 0;
}

#ifdef CONFIG_CFS_BANDWIDTH
/* rq->task_clock normalized against any time this cfs_rq has spent throttled */
inline u64 cfs_rq_clock_task_no_lockdep(struct cfs_rq *cfs_rq)
{
	if (unlikely(cfs_rq->throttle_count))
		return cfs_rq->throttled_clock_task;

	return cfs_rq->rq->clock_task - cfs_rq->throttled_clock_task_time;
}
#else
inline u64 cfs_rq_clock_task_no_lockdep(struct cfs_rq *cfs_rq)
{
	return cfs_rq->rq->clock_task;
}
#endif

struct clb_env {
	struct clb_stats bstats;
	struct clb_stats lstats;
	int btarget, ltarget;

	struct cpumask bcpus;
	struct cpumask lcpus;

	unsigned int flags;
	struct mcheck {
		/* Details of this migration check */
		int status;
		/* Indicate whether we should perform this task migration */
		int result;
	} mcheck;
};

/*
 * move_task - move a task from one runqueue to another runqueue.
 * Both runqueues must be locked.
 */
void move_task(struct task_struct *p, struct lb_env *env)
{
	lockdep_assert_held(&env->src_rq->lock);
	lockdep_assert_held(&env->dst_rq->lock);

	p->on_rq = TASK_ON_RQ_MIGRATING;
	deactivate_task(env->src_rq, p, 0);
	set_task_cpu(p, env->dst_cpu);

	activate_task(env->dst_rq, p, 0);
	p->on_rq = TASK_ON_RQ_QUEUED;
	check_preempt_curr(env->dst_rq, p, 0);
	update_capacity_of(cpu_of(env->dst_rq), SCHE_VALID);
}

static void collect_cluster_stats(struct clb_stats *clbs,
		struct cpumask *cluster_cpus, int target)
{
	/* Update cluster informatics */
	int cpu;
	int loadwop;

	for_each_cpu(cpu, cluster_cpus) {
		if (cpu_online(cpu)) {
			clbs->ncpu++;
			clbs->ntask += cpu_rq(cpu)->cfs.h_nr_running;
			clbs->load_avg += cpu_rq(cpu)->cfs.avg.loadwop_avg;
#ifdef CONFIG_SCHED_HMP_PRIO_FILTER
			clbs->nr_normal_prio_task += cfs_nr_normal_prio(cpu);
			clbs->nr_dequeuing_low_prio +=
				cfs_nr_dequeuing_low_prio(cpu);
#endif
		}
	}

	if (!clbs->ncpu || target >= num_possible_cpus() ||
			!cpumask_test_cpu(target, cluster_cpus))
		return;

	/*
	 * Calculate available CPU capacity
	 * Calculate available task space
	 *
	 * Why load ratio should be multiplied by the number of task ?
	 * The task is the entity of scheduling unit so that we should consider
	 * it in scheduler. Only considering task load is not enough.
	 * Thus, multiplying the number of tasks can adjust load ratio to a more
	 * reasonable value.
	 */
	loadwop = cpu_rq(target)->cfs.avg.loadwop_avg;
	clbs->load_avg /= clbs->ncpu;
	clbs->acap = (clbs->cpu_capacity > loadwop) ?
		(clbs->cpu_capacity - loadwop) : 0;
	clbs->scaled_atask = (clbs->cpu_capacity > loadwop) ?
		(clbs->cpu_capacity - loadwop) : 0;

	trace_sched_cluster_stats(target,
			cpu_rq(target)->cfs.avg.loadwop_avg,
			cpu_rq(target)->cfs.h_nr_running,
			*cpumask_bits(cluster_cpus),
			clbs->ntask, clbs->load_avg,
			clbs->cpu_capacity, clbs->acap,
			clbs->scaled_atask, clbs->threshold);
}


/*
 * Task Dynamic Migration Threshold Adjustment.
 *
 * If the workload between clusters is not balanced, adjust migration
 * threshold in an attempt to move task precisely.
 *
 * Diff. = Max Threshold - Min Threshold
 *
 * Dynamic UP-Threshold =
 *                               B_nacap               B_natask
 * Max Threshold - Diff. x  -----------------  x  -------------------
 *                          B_nacap + L_nacap     B_natask + L_natask
 *
 *
 * Dynamic Down-Threshold =
 *                               L_nacap               L_natask
 * Min Threshold + Diff. x  -----------------  x  -------------------
 *                          B_nacap + L_nacap     B_natask + L_natask
 */
static void adj_threshold(struct clb_env *clbenv)
{
#define HMP_RESOLUTION_SCALING (4)
#define hmp_scale_down(w) ((w) >> HMP_RESOLUTION_SCALING)

	unsigned long b_cap = 0, l_cap = 0;
	int b_nacap, l_nacap, b_natask, l_natask;
	const int hmp_max_weight = scale_load_down(HMP_MAX_LOAD);

	b_cap = clbenv->bstats.cpu_power;
	l_cap = clbenv->lstats.cpu_power;
	b_nacap = clbenv->bstats.acap *	b_cap / (l_cap+1);
	b_natask = clbenv->bstats.scaled_atask *
			b_cap / (l_cap+1);
	l_nacap = clbenv->lstats.acap;
	l_natask = clbenv->lstats.scaled_atask;

	b_nacap = hmp_scale_down(b_nacap);
	l_nacap = hmp_scale_down(l_nacap);
	b_natask = hmp_scale_down(b_natask);
	l_natask = hmp_scale_down(l_natask);

	clbenv->bstats.threshold = hmp_max_weight -
		(hmp_max_weight * b_nacap * b_natask) /
		((b_nacap + l_nacap) * (b_natask + l_natask) + 1);
	clbenv->lstats.threshold = hmp_max_weight * l_nacap * l_natask /
		((b_nacap + l_nacap) * (b_natask + l_natask) + 1);

	trace_sched_adj_threshold(clbenv->bstats.threshold,
			clbenv->lstats.threshold, clbenv->ltarget,
			l_cap, clbenv->btarget, b_cap);
}

static void sched_update_clbstats(struct clb_env *clbenv)
{
	/* init cpu power and capacity */
	clbenv->bstats.cpu_power =
		(int) arch_scale_cpu_capacity(NULL, clbenv->btarget);
	clbenv->lstats.cpu_power =
		(int) arch_scale_cpu_capacity(NULL, clbenv->ltarget);
	clbenv->lstats.cpu_capacity = SCHED_CAPACITY_SCALE;
	clbenv->bstats.cpu_capacity = SCHED_CAPACITY_SCALE *
		clbenv->bstats.cpu_power / (clbenv->lstats.cpu_power+1);

	collect_cluster_stats(&clbenv->bstats, &clbenv->bcpus, clbenv->btarget);
	collect_cluster_stats(&clbenv->lstats, &clbenv->lcpus, clbenv->ltarget);
	adj_threshold(clbenv);
}


/*
 * Heterogenous multiprocessor (HMP) optimizations
 *
 * The cpu types are distinguished using a list of hmp_domains
 * which each represent one cpu type using a cpumask.
 * The list is assumed ordered by compute capacity with the
 * fastest domain first.
 */

DEFINE_PER_CPU(struct hmp_domain *, hmp_cpu_domain);

/* Setup hmp_domains */
static void __init hmp_cpu_mask_setup(void)
{
	struct hmp_domain *domain;
	struct list_head *pos;
	int cpu;

	pr_info("Initializing HMP scheduler:\n");

	/* Initialize hmp_domains using platform code */
	if (list_empty(&hmp_domains)) {
		pr_info("HMP domain list is empty!\n");
		return;
	}

	/* Print hmp_domains */
	list_for_each(pos, &hmp_domains) {
		domain = list_entry(pos, struct hmp_domain, hmp_domains);

		for_each_cpu(cpu, &domain->possible_cpus)
			per_cpu(hmp_cpu_domain, cpu) = domain;
	}
}

static struct hmp_domain *hmp_get_hmp_domain_for_cpu(int cpu)
{
	struct hmp_domain *domain;
	struct list_head *pos;

	list_for_each(pos, &hmp_domains) {
		domain = list_entry(pos, struct hmp_domain, hmp_domains);
		if (cpumask_test_cpu(cpu, &domain->possible_cpus))
			return domain;
	}
	return NULL;
}

static void hmp_online_cpu(int cpu)
{
	struct hmp_domain *domain = hmp_get_hmp_domain_for_cpu(cpu);

	if (domain)
		cpumask_set_cpu(cpu, &domain->cpus);
}

static void hmp_offline_cpu(int cpu)
{
	struct hmp_domain *domain = hmp_get_hmp_domain_for_cpu(cpu);

	if (domain)
		cpumask_clear_cpu(cpu, &domain->cpus);
}

unsigned int hmp_next_up_threshold = 4096;
unsigned int hmp_next_down_threshold = 4096;
#define hmp_last_up_migration(cpu) \
	cpu_rq(cpu)->cfs.avg.hmp_last_up_migration
#define hmp_last_down_migration(cpu) \
	cpu_rq(cpu)->cfs.avg.hmp_last_down_migration


static inline unsigned int hmp_domain_min_load(struct hmp_domain *hmpd,
		int *min_cpu);

/* Check if cpu is in fastest hmp_domain */
inline unsigned int hmp_cpu_is_fastest(int cpu)
{
	struct list_head *pos;

	pos = &hmp_cpu_domain(cpu)->hmp_domains;
	return pos == hmp_domains.next;
}

/* Check if cpu is in slowest hmp_domain */
inline unsigned int hmp_cpu_is_slowest(int cpu)
{
	struct list_head *pos;

	pos = &hmp_cpu_domain(cpu)->hmp_domains;
	return list_is_last(pos, &hmp_domains);
}

/* Next (slower) hmp_domain relative to cpu */
static inline struct hmp_domain *hmp_slower_domain(int cpu)
{
	struct list_head *pos;

	pos = &hmp_cpu_domain(cpu)->hmp_domains;
	if (list_is_last(pos, &hmp_domains))
		return list_entry(pos, struct hmp_domain, hmp_domains);

	return list_entry(pos->next, struct hmp_domain, hmp_domains);
}

/* Previous (faster) hmp_domain relative to cpu */
static inline struct hmp_domain *hmp_faster_domain(int cpu)
{
	struct list_head *pos;

	pos = &hmp_cpu_domain(cpu)->hmp_domains;
	if (pos->prev == &hmp_domains)
		return list_entry(pos, struct hmp_domain, hmp_domains);

	return list_entry(pos->prev, struct hmp_domain, hmp_domains);
}

/*
 * Selects a cpu in previous (faster) hmp_domain
 * Note that cpumask_any_and() returns the first cpu in the cpumask
 */
static inline unsigned int hmp_select_faster_cpu(struct task_struct *tsk,
		int cpu)
{
	int lowest_cpu = num_possible_cpus();
	__always_unused int lowest_ratio =
		hmp_domain_min_load(hmp_faster_domain(cpu), &lowest_cpu);
	/*
	 * If the lowest-loaded CPU in the domain is allowed by
	 * the task affinity.
	 * Select that one, otherwise select one which is allowed
	 */
	if (lowest_cpu < nr_cpu_ids &&
			cpumask_test_cpu(lowest_cpu, tsk_cpus_allowed(tsk)))
		return lowest_cpu;
	else
		return cpumask_any_and(&hmp_faster_domain(cpu)->cpus,
				tsk_cpus_allowed(tsk));
}

static inline void hmp_next_up_delay(struct sched_entity *se, int cpu)
{
	struct cfs_rq *cfs_rq = &cpu_rq(cpu)->cfs;

	hmp_last_up_migration(cpu) = cfs_rq_clock_task_no_lockdep(cfs_rq);
	hmp_last_down_migration(cpu) = 0;
}

static inline void hmp_next_down_delay(struct sched_entity *se, int cpu)
{
	struct cfs_rq *cfs_rq = &cpu_rq(cpu)->cfs;

	hmp_last_down_migration(cpu) = cfs_rq_clock_task_no_lockdep(cfs_rq);
	hmp_last_up_migration(cpu) = 0;
}

static inline unsigned int hmp_domain_min_load(struct hmp_domain *hmpd,
		int *min_cpu)
{
	int cpu;
	int min_cpu_runnable_temp = num_possible_cpus();
	unsigned long min_runnable_load = INT_MAX;
	unsigned long contrib;

	for_each_cpu(cpu, &hmpd->cpus) {
		struct cfs_rq *cfs_rq = &cpu_rq(cpu)->cfs;

		/* don't use the divisor in the loop, just at the end */
		contrib = cfs_rq->runnable_load_avg * scale_load_down(1024);
		if (contrib < min_runnable_load) {
			min_runnable_load = contrib;
			min_cpu_runnable_temp = cpu;
		}
	}

	if (min_cpu)
		*min_cpu = min_cpu_runnable_temp;

	/* domain will often have at least one empty CPU */
	return min_runnable_load ? min_runnable_load / (__LOAD_AVG_MAX + 1) : 0;
}


/* Function Declaration */
static int hmp_up_stable(int cpu);
static int hmp_down_stable(int cpu);
static unsigned int hmp_up_migration(int cpu,
		int *target_cpu, struct sched_entity *se,
		struct clb_env *clbenv);
static unsigned int hmp_down_migration(int cpu,
		int *target_cpu, struct sched_entity *se,
		struct clb_env *clbenv);
#ifdef CONFIG_SCHED_HMP_PLUS
static struct sched_entity *hmp_get_heaviest_task(
		struct sched_entity *se, int target_cpu);
static struct sched_entity *hmp_get_lightest_task(
		struct sched_entity *se, int migrate_down);
#endif

#define hmp_caller_is_gb(caller) ((caller == HMP_GB)?1:0)

#define hmp_cpu_stable(cpu, up) (up ? \
		hmp_up_stable(cpu) : hmp_down_stable(cpu))

#define hmp_inc(v) ((v) + 1)

#define task_created(f) ((SD_BALANCE_EXEC == f || SD_BALANCE_FORK == f)?1:0)

/*
 * Heterogenous Multi-Processor (HMP) - Utility Function
 */

/*
 * These functions add next up/down migration delay that prevents the task from
 * doing another migration in the same direction until the delay has expired.
 */
static int hmp_up_stable(int cpu)
{
	struct cfs_rq *cfs_rq = &cpu_rq(cpu)->cfs;
	u64 now = cfs_rq_clock_task_no_lockdep(cfs_rq);

	if (((now - hmp_last_up_migration(cpu)) >> 10) < hmp_next_up_threshold)
		return 0;
	return 1;
}

static int hmp_down_stable(int cpu)
{
	struct cfs_rq *cfs_rq = &cpu_rq(cpu)->cfs;
	u64 now = cfs_rq_clock_task_no_lockdep(cfs_rq);
	u64 duration = now - hmp_last_down_migration(cpu);

	if ((duration >> 10) < hmp_next_down_threshold)
		return 0;
	return 1;
}

/* Select the most appropriate CPU from hmp cluster */
static unsigned int hmp_select_cpu(unsigned int caller, struct task_struct *p,
		struct cpumask *mask, int prev, int up)
{
	int curr = 0;
	int target = num_possible_cpus();
	unsigned long curr_wload = 0;
	unsigned long target_wload = 0;
	struct cpumask srcp;
	struct cpumask *tsk_cpus_allow = tsk_cpus_allowed(p);

	cpumask_andnot(&srcp, cpu_online_mask, cpu_isolated_mask);
	cpumask_and(&srcp, &srcp, mask);
	target = cpumask_any_and(&srcp, tsk_cpus_allow);
	if (target >= num_possible_cpus())
		goto out;

	/*
	 * RT class is taken into account because CPU load is multiplied
	 * by the total number of CPU runnable tasks that includes RT tasks.
	 */
	target_wload = hmp_inc(cfs_load(target));
	target_wload += cfs_pending_load(target);
	target_wload *= rq_length(target);
	for_each_cpu(curr, mask) {
		/* Check CPU status and task affinity */
		if (!cpu_online(curr) ||
				!cpumask_test_cpu(curr, tsk_cpus_allow) ||
					cpu_isolated(curr))
			continue;

		/* For global load balancing, unstable CPU will be bypassed */
		if (hmp_caller_is_gb(caller) && !hmp_cpu_stable(curr, up))
			continue;

		curr_wload = hmp_inc(cfs_load(curr));
		curr_wload += cfs_pending_load(curr);
		curr_wload *= rq_length(curr);
		if (curr_wload < target_wload) {
			target_wload = curr_wload;
			target = curr;
		} else if (curr_wload == target_wload && curr == prev) {
			target = curr;
		}
	}

out:
	return target;
}

static int hmp_select_task_migration(int sd_flag,
		struct task_struct *p, int prev_cpu, int new_cpu,
		struct cpumask *fast_cpu_mask, struct cpumask *slow_cpu_mask)
{
	int step = 0;
	struct sched_entity *se = &p->se;
	int B_target = num_possible_cpus();
	int L_target = num_possible_cpus();
	struct clb_env clbenv;

	B_target = hmp_select_cpu(HMP_SELECT_RQ, p, fast_cpu_mask, prev_cpu, 0);
	L_target = hmp_select_cpu(HMP_SELECT_RQ, p, slow_cpu_mask, prev_cpu, 1);

	/*
	 * Only one cluster exists or only one cluster is allowed for this task
	 * Case 1: return the runqueue whose load is minimum
	 * Case 2: return original CFS runqueue selection result
	 */
	if (B_target >= num_possible_cpus() && L_target >= num_possible_cpus())
		goto out;
	if (B_target >= num_possible_cpus())
		goto select_slow;
	if (L_target >= num_possible_cpus())
		goto select_fast;

	/*
	 * Two clusters exist and both clusters are allowed for this task
	 * Step 1: Move newly created task to the cpu where no tasks are running
	 * Step 2: Migrate heavy-load task to big
	 * Step 3: Migrate light-load task to LITTLE
	 * Step 4: Make sure the task stays in its previous hmp domain
	 */
	step = 1;
	if (task_created(sd_flag) && !task_low_priority(p->prio)) {
		if (!rq_length(B_target))
			goto select_fast;
		if (!rq_length(L_target))
			goto select_slow;
	}
	memset(&clbenv, 0, sizeof(clbenv));
	clbenv.flags |= HMP_SELECT_RQ;
	cpumask_copy(&clbenv.lcpus, slow_cpu_mask);
	cpumask_copy(&clbenv.bcpus, fast_cpu_mask);
	clbenv.ltarget = L_target;
	clbenv.btarget = B_target;

	step = 2;
	sched_update_clbstats(&clbenv);
	if (hmp_up_migration(L_target, &B_target, se, &clbenv))
		goto select_fast;
	step = 3;
	if (hmp_down_migration(B_target, &L_target, se, &clbenv))
		goto select_slow;
	step = 4;
	if (hmp_cpu_is_slowest(prev_cpu))
		goto select_slow;
	goto select_fast;

select_fast:
	new_cpu = B_target;
	cpumask_clear(slow_cpu_mask);
	goto out;
select_slow:
	new_cpu = L_target;
	cpumask_copy(fast_cpu_mask, slow_cpu_mask);
	cpumask_clear(slow_cpu_mask);
	goto out;

out:
	/*
	 * Value of clbenb..load_avg only ready after step 2.
	 * Dump value after this step to avoid invalid stack value
	 */
	if (step > 1)
		trace_sched_hmp_load(step,
				clbenv.bstats.load_avg, clbenv.lstats.load_avg);
	return new_cpu;
}

/*
 * Heterogenous Multi-Processor (HMP) - Task Runqueue Selection
 */

/* This function enhances the original task selection function */
static int hmp_select_task_rq_fair(int sd_flag, struct task_struct *p,
		int prev_cpu, int new_cpu)
{
	struct list_head *pos;
	struct sched_entity *se = &p->se;
	struct cpumask fast_cpu_mask, slow_cpu_mask;

	if (idle_cpu(new_cpu) && hmp_cpu_is_fastest(new_cpu))
		return new_cpu;

	/* error handling */
	if (prev_cpu >= num_possible_cpus())
		return new_cpu;

	/*
	 * Skip all the checks if only one CPU is online.
	 * Otherwise, select the most appropriate CPU from cluster.
	 */
	if (num_online_cpus() == 1)
		goto out;

	cpumask_clear(&fast_cpu_mask);
	cpumask_clear(&slow_cpu_mask);
	/* order: fast to slow hmp domain */
	list_for_each(pos, &hmp_domains) {
		struct hmp_domain *domain;

		domain = list_entry(pos, struct hmp_domain, hmp_domains);
		if (cpumask_empty(&domain->cpus))
			continue;
		if (cpumask_empty(&fast_cpu_mask)) {
			cpumask_copy(&fast_cpu_mask, &domain->possible_cpus);
		} else {
			cpumask_copy(&slow_cpu_mask, &domain->possible_cpus);
			new_cpu = hmp_select_task_migration(sd_flag, p,
				prev_cpu, new_cpu, &fast_cpu_mask,
				&slow_cpu_mask);
		}
	}

out:
	/* it happens when num_online_cpus=1 */
	if (new_cpu >= nr_cpu_ids) {
		/* BUG_ON(1); */
		new_cpu = prev_cpu;
	}

	cfs_nr_pending(new_cpu)++;
	cfs_pending_load(new_cpu) += se_load(se);

	return new_cpu;

}

#define hmp_fast_cpu_has_spare_cycles(B, cpu_load) (cpu_load * 1280 < \
		(SCHED_CAPACITY_SCALE * 1024))

#define hmp_task_fast_cpu_afford(B, se, cpu) \
		(B->acap > 0 && hmp_fast_cpu_has_spare_cycles(B, \
		se_load(se) + cfs_load(cpu)))

#define hmp_fast_cpu_oversubscribed(caller, B, se, cpu) \
	(hmp_caller_is_gb(caller) ? \
	 !hmp_fast_cpu_has_spare_cycles(B, cfs_load(cpu)) : \
	 !hmp_task_fast_cpu_afford(B, se, cpu))

#define hmp_task_slow_cpu_afford(L, se) \
	(L->acap > 0 && L->acap >= se_load(se))

/* Macro used by low-priority task filter */
#define hmp_low_prio_task_up_rejected(p, B, L) \
	(task_low_priority(p->prio) && \
	 (B->ntask >= B->ncpu || 0 != L->nr_normal_prio_task) && \
	 (p->se.avg.loadwop_avg < 800))

#define hmp_low_prio_task_down_allowed(p, B, L) \
	(task_low_priority(p->prio) && !B->nr_dequeuing_low_prio && \
	 B->ntask >= B->ncpu && 0 != L->nr_normal_prio_task && \
	 (p->se.avg.loadwop_avg < 800))

/* Migration check result */
#define HMP_BIG_NOT_OVERSUBSCRIBED           (0x01)
#define HMP_BIG_CAPACITY_INSUFFICIENT        (0x02)
#define HMP_LITTLE_CAPACITY_INSUFFICIENT     (0x04)
#define HMP_LOW_PRIORITY_FILTER              (0x08)
#define HMP_BIG_BUSY_LITTLE_IDLE             (0x10)
#define HMP_BIG_IDLE                         (0x20)
#define HMP_MIGRATION_APPROVED              (0x100)
#define HMP_TASK_UP_MIGRATION               (0x200)
#define HMP_TASK_DOWN_MIGRATION             (0x400)

/* Migration statistics */
struct hmp_statisic hmp_stats;

/*
 * Check whether this task should be migrated to big
 * Briefly summarize the flow as below;
 * 1) Migration stabilizing
 * 2) Filter low-priority task
 * 2.5) Keep all cpu busy
 * 3) Check CPU capacity
 * 4) Check dynamic migration threshold
 */
static unsigned int hmp_up_migration(int cpu,
		int *target_cpu, struct sched_entity *se,
		struct clb_env *clbenv)
{
	struct task_struct *p = task_of(se);
	struct clb_stats *L, *B;
	struct mcheck *check;
	int curr_cpu = cpu;
	unsigned int caller = clbenv->flags;
	cpumask_t act_mask;

	L = &clbenv->lstats;
	B = &clbenv->bstats;
	check = &clbenv->mcheck;

	check->status = clbenv->flags;
	check->status |= HMP_TASK_UP_MIGRATION;
	check->result = 0;

	cpumask_andnot(&act_mask, cpu_active_mask, cpu_isolated_mask);

	/*
	 * No migration is needed if
	 * 1) There is only one cluster
	 * 2) Task is already in big cluster
	 * 3) It violates task affinity
	 */
	if (!L->ncpu || !B->ncpu
			|| cpumask_test_cpu(curr_cpu, &clbenv->bcpus)
			|| !cpumask_intersects(&clbenv->bcpus,
				tsk_cpus_allowed(p))
			|| !cpumask_intersects(&clbenv->bcpus, &act_mask))
		goto out;

	/*
	 * [1] Migration stabilizing
	 * Let the task load settle before doing another up migration.
	 * It can prevent a bunch of tasks from migrating to a unstable CPU.
	 */
	if (!hmp_up_stable(*target_cpu))
		goto out;

	/* [2] Filter low-priority task */
#ifdef CONFIG_SCHED_HMP_PRIO_FILTER
	if (hmp_low_prio_task_up_rejected(p, B, L)) {
		check->status |= HMP_LOW_PRIORITY_FILTER;
		goto trace;
	}
#endif

	/* [2.5]if big is idle, just go to big */
	if (rq_length(*target_cpu) == 0) {
		check->status |= HMP_BIG_IDLE;
		check->status |= HMP_MIGRATION_APPROVED;
		check->result = 1;
		goto trace;
	}

	/*
	 * [3] Check CPU capacity
	 * Forbid up-migration if big CPU can't handle this task
	 */
	if (!hmp_task_fast_cpu_afford(B, se, *target_cpu)) {
		check->status |= HMP_BIG_CAPACITY_INSUFFICIENT;
		goto trace;
	}

	/*
	 * [4] Check dynamic migration threshold
	 * Migrate task from LITTLE to big if load is greater than up-threshold
	 */
	if (se_load(se) > B->threshold) {
		check->status |= HMP_MIGRATION_APPROVED;
		check->result = 1;
	}

trace:
	if (check->result && hmp_caller_is_gb(caller))
		hmp_stats.nr_force_up++;
	trace_sched_hmp_stats(&hmp_stats);
	trace_sched_dynamic_threshold(task_of(se), B->threshold, check->status,
			curr_cpu, *target_cpu, se_load(se), B, L);
	trace_sched_dynamic_threshold_draw(B->threshold, L->threshold);
out:
	return check->result;
}

/*
 * Check whether this task should be migrated to LITTLE
 * Briefly summarize the flow as below;
 * 1) Migration stabilizing
 * 1.5) Keep all cpu busy
 * 2) Filter low-priority task
 * 3) Check CPU capacity
 * 4) Check dynamic migration threshold
 */
static unsigned int hmp_down_migration(int cpu,
		int *target_cpu, struct sched_entity *se,
		struct clb_env *clbenv)
{
	struct task_struct *p = task_of(se);
	struct clb_stats *L, *B;
	struct mcheck *check;
	int curr_cpu = cpu;
	unsigned int caller = clbenv->flags;
	cpumask_t act_mask;

	L = &clbenv->lstats;
	B = &clbenv->bstats;
	check = &clbenv->mcheck;

	check->status = caller;
	check->status |= HMP_TASK_DOWN_MIGRATION;
	check->result = 0;

	cpumask_andnot(&act_mask, cpu_active_mask, cpu_isolated_mask);

	/*
	 * No migration is needed if
	 * 1) There is only one cluster
	 * 2) Task is already in LITTLE cluster
	 * 3) It violates task affinity
	 */
	if (!L->ncpu || !B->ncpu
			|| cpumask_test_cpu(curr_cpu, &clbenv->lcpus)
			|| !cpumask_intersects(&clbenv->lcpus,
				tsk_cpus_allowed(p))
			|| !cpumask_intersects(&clbenv->lcpus, &act_mask))
		goto out;

	/*
	 * [1] Migration stabilizing
	 * Let the task load settle before doing another down migration.
	 * It can prevent a bunch of tasks from migrating to a unstable CPU.
	 */
	if (!hmp_down_stable(*target_cpu))
		goto out;

	/* [1.5]if big is busy and little is idle, just go to little */
	if (rq_length(*target_cpu) == 0 && caller == HMP_SELECT_RQ
			&& rq_length(curr_cpu) > 0) {
		struct rq *curr_rq = cpu_rq(curr_cpu);

		/*
		 * If current big core is not heavy task,
		 * and wake up task is heavy task.
		 *
		 * Dont go to little.
		 */
		if (!(!is_heavy_task(curr_rq->curr) && is_heavy_task(p))) {
			check->status |= HMP_BIG_BUSY_LITTLE_IDLE;
			check->status |= HMP_MIGRATION_APPROVED;
			check->result = 1;
			goto trace;
		}
	}

	/* [2] Filter low-priority task */
#ifdef CONFIG_SCHED_HMP_PRIO_FILTER
	if (hmp_low_prio_task_down_allowed(p, B, L)) {
		cfs_nr_dequeuing_low_prio(curr_cpu)++;
		check->status |= HMP_LOW_PRIORITY_FILTER;
		check->status |= HMP_MIGRATION_APPROVED;
		check->result = 1;
		goto trace;
	}
#endif

	/*
	 * [3] Check CPU capacity
	 * Forbid down-migration if either of the following conditions is true
	 * 1) big cpu is not oversubscribed (if big CPU seems to have spare
	 *    cycles, do not force this task to run on LITTLE CPU, but
	 *    keep it staying in its previous cluster instead)
	 * 2) LITTLE cpu doesn't have available capacity for this new task
	 */
	if (cpu_rq(curr_cpu)->cfs.h_nr_running > 1 &&
			!hmp_fast_cpu_oversubscribed(caller, B, se, curr_cpu)) {
		check->status |= HMP_BIG_NOT_OVERSUBSCRIBED;
		goto trace;
	}

	if (!hmp_task_slow_cpu_afford(L, se)) {
		check->status |= HMP_LITTLE_CAPACITY_INSUFFICIENT;
		goto trace;
	}

	/*
	 * [4] Check dynamic migration threshold
	 * Migrate task from big to LITTLE if load ratio is less than
	 * or equal to down-threshold
	 */
	if (L->threshold >= se_load(se)) {
		check->status |= HMP_MIGRATION_APPROVED;
		check->result = 1;
	}

trace:
	if (check->result && hmp_caller_is_gb(caller))
		hmp_stats.nr_force_down++;
	trace_sched_hmp_stats(&hmp_stats);
	trace_sched_dynamic_threshold(task_of(se), L->threshold, check->status,
			curr_cpu, *target_cpu, se_load(se), B, L);
	trace_sched_dynamic_threshold_draw(B->threshold, L->threshold);
out:
	return check->result;
}

static inline void
hmp_update_load_avg(unsigned int decayed, unsigned long weight,
		unsigned int scaled_delta_w, struct sched_avg *sa, u64 periods,
		u32 contrib, u64 scaled_delta, struct cfs_rq *cfs_rq)
{
	if (cfs_rq)
		weight = scale_load_down(NICE_0_LOAD) * cfs_rq->h_nr_running;
	else if (weight > 0)
		weight = scale_load_down(NICE_0_LOAD);

	if (decayed) {
		if (weight)
			sa->loadwop_sum += weight * scaled_delta_w;

		sa->loadwop_sum = decay_load(sa->loadwop_sum, periods + 1);

		if (weight)
			sa->loadwop_sum += weight * contrib;
	}

	if (weight)
		sa->loadwop_sum += weight * scaled_delta;

	if (decayed)
		sa->loadwop_avg = div_u64(sa->loadwop_sum, LOAD_AVG_MAX);

	/*
	 * if caller update cfs' loading, also dump cfs
	 * loading with systrace
	 */
	if (cfs_rq) {
		trace_sched_cfs_runnable_load(cpu_of(cfs_rq->rq),
			cfs_rq->avg.loadwop_avg, cfs_rq->h_nr_running);
	}
}

static int hmp_force_up_cpu_stop(void *data)
{
	return hmp_active_task_migration_cpu_stop(data);
}

static int hmp_force_down_cpu_stop(void *data)
{
	return hmp_active_task_migration_cpu_stop(data);
}

/*
 * According to Linaro's comment, we should only check the currently running
 * tasks because selecting other tasks for migration will require extensive
 * book keeping.
 */
static void hmp_force_down_migration(int this_cpu)
{
	int target_cpu;
	struct sched_entity *se;
	struct rq *target;
	unsigned long flags;
	unsigned int force = 0;
	struct task_struct *p;
	struct clb_env clbenv;
#ifdef CONFIG_SCHED_HMP_PLUS
	struct sched_entity *orig;
	int B_cpu;
#endif
	struct hmp_domain *hmp_domain = NULL;
	struct cpumask fast_cpu_mask, slow_cpu_mask;

	cpumask_clear(&fast_cpu_mask);
	cpumask_clear(&slow_cpu_mask);

	/* Migrate light task from big to LITTLE */
	if (!hmp_cpu_is_slowest(this_cpu)) {
		hmp_domain = hmp_cpu_domain(this_cpu);
		cpumask_copy(&fast_cpu_mask, &hmp_domain->possible_cpus);
		while (!list_is_last(&hmp_domain->hmp_domains, &hmp_domains)) {
			struct list_head *pos = &hmp_domain->hmp_domains;

			hmp_domain = list_entry(pos->next,
					struct hmp_domain, hmp_domains);

			if (!cpumask_empty(&hmp_domain->cpus)) {
				cpumask_copy(&slow_cpu_mask,
						&hmp_domain->possible_cpus);
				break;
			}
		}
	}
	if (!hmp_domain || hmp_domain == hmp_cpu_domain(this_cpu))
		return;

	if (cpumask_empty(&fast_cpu_mask) || cpumask_empty(&slow_cpu_mask))
		return;

	force = 0;
	target = cpu_rq(this_cpu);
	raw_spin_lock_irqsave(&target->lock, flags);
	se = target->cfs.curr;
	if (!se) {
		raw_spin_unlock_irqrestore(&target->lock, flags);
		return;
	}

	/* Find task entity */
	if (!entity_is_task(se)) {
		struct cfs_rq *cfs_rq;

		cfs_rq = group_cfs_rq(se);
		while (cfs_rq) {
			se = cfs_rq->curr;
			cfs_rq = group_cfs_rq(se);
		}
	}
#ifdef CONFIG_SCHED_HMP_PLUS
	orig = se;
	se = hmp_get_lightest_task(orig, 1);
	if (!se) {
		raw_spin_unlock_irqrestore(&target->lock, flags);
		return;
	}
	if (!entity_is_task(se))
		p = task_of(orig);
	else
#endif
		p = task_of(se);
#ifdef CONFIG_SCHED_HMP_PLUS
	/*
	 * Don't offload to little if there is one idle big,
	 * let load balance to do it's work.
	 * Also, to prevent idle_balance from leading to potential ping-pong
	 */
	B_cpu = hmp_select_cpu(HMP_GB, p, &fast_cpu_mask, this_cpu, 0);
	if (B_cpu < nr_cpu_ids && !rq_length(B_cpu)) {
		raw_spin_unlock_irqrestore(&target->lock, flags);
		return;
	}
#endif
	target_cpu = hmp_select_cpu(HMP_GB, p, &slow_cpu_mask, -1, 1);
	if (target_cpu >= num_possible_cpus()) {
		raw_spin_unlock_irqrestore(&target->lock, flags);
		return;
	}

	/* Collect cluster information */
	memset(&clbenv, 0, sizeof(clbenv));
	clbenv.flags |= HMP_GB;
	clbenv.btarget = this_cpu;
	clbenv.ltarget = target_cpu;
	cpumask_copy(&clbenv.lcpus, &slow_cpu_mask);
	cpumask_copy(&clbenv.bcpus, &fast_cpu_mask);
	sched_update_clbstats(&clbenv);

#ifdef CONFIG_SCHED_HMP_PLUS
	if (cpu_rq(this_cpu)->cfs.h_nr_running < 2) {
		raw_spin_unlock_irqrestore(&target->lock, flags);
		return;
	}
#endif

	/* Check migration threshold */
	if (!target->active_balance &&
			hmp_down_migration(this_cpu,
				&target_cpu, se, &clbenv) &&
			!cpu_park(cpu_of(target))) {
		if (p->state != TASK_DEAD) {
			get_task_struct(p);
			target->active_balance = 1; /* force down */
			target->push_cpu = target_cpu;
			target->migrate_task = p;
			force = 1;
			trace_sched_hmp_migrate(p, target->push_cpu, 1);
			hmp_next_down_delay(&p->se, target->push_cpu);
		}
	}
	raw_spin_unlock_irqrestore(&target->lock, flags);
	if (force) {
		if (stop_one_cpu_dispatch(cpu_of(target),
					hmp_force_down_cpu_stop,
					target, &target->active_balance_work)) {
			put_task_struct(p); /* out of rq->lock */
			raw_spin_lock_irqsave(&target->lock, flags);
			target->active_balance = 0;
			force = 0;
			raw_spin_unlock_irqrestore(&target->lock, flags);
		}
	}

}

/*
 * hmp_force_up_migration checks runqueues for tasks that need to
 * be actively migrated to a faster cpu.
 */
static void hmp_force_up_migration(int this_cpu)
{
	int curr_cpu, target_cpu;
	struct sched_entity *se;
	struct rq *target;
	unsigned long flags;
	unsigned int force = 0;
	struct task_struct *p;
	struct clb_env clbenv;
#ifdef CONFIG_SCHED_HMP_PLUS
	struct sched_entity *orig;
#endif

	if (!spin_trylock(&hmp_force_migration))
		return;

	/* Migrate heavy task from LITTLE to big */
	for_each_online_cpu(curr_cpu) {
		struct hmp_domain *hmp_domain = NULL;
		struct cpumask fast_cpu_mask, slow_cpu_mask;

		cpumask_clear(&fast_cpu_mask);
		cpumask_clear(&slow_cpu_mask);
		if (!hmp_cpu_is_fastest(curr_cpu)) {
			/* current cpu is slow_cpu_mask*/
			hmp_domain = hmp_cpu_domain(curr_cpu);
			cpumask_copy(&slow_cpu_mask,
					&hmp_domain->possible_cpus);

			while (&hmp_domain->hmp_domains != hmp_domains.next) {
				struct list_head *pos;

				pos = &hmp_domain->hmp_domains;
				hmp_domain = list_entry(pos->prev,
						struct hmp_domain, hmp_domains);
				if (cpumask_empty(&hmp_domain->cpus))
					continue;

				cpumask_copy(&fast_cpu_mask,
						&hmp_domain->possible_cpus);
				break;
			}
		} else {
			hmp_force_down_migration(curr_cpu);
			continue;
		}
		if (!hmp_domain || hmp_domain == hmp_cpu_domain(curr_cpu))
			continue;

		if (cpumask_empty(&fast_cpu_mask) ||
				cpumask_empty(&slow_cpu_mask))
			continue;

		force = 0;
		target = cpu_rq(curr_cpu);
		raw_spin_lock_irqsave(&target->lock, flags);
		se = target->cfs.curr;
		if (!se) {
			raw_spin_unlock_irqrestore(&target->lock, flags);
			continue;
		}

		/* Find task entity */
		if (!entity_is_task(se)) {
			struct cfs_rq *cfs_rq;

			cfs_rq = group_cfs_rq(se);
			while (cfs_rq) {
				se = cfs_rq->curr;
				cfs_rq = group_cfs_rq(se);
			}
		}
#ifdef CONFIG_SCHED_HMP_PLUS
		orig = se;
		se = hmp_get_heaviest_task(se, -1);
		if (!se) {
			raw_spin_unlock_irqrestore(&target->lock, flags);
			continue;
		}
		if (!entity_is_task(se))
			p = task_of(orig);
		else
#endif
			p = task_of(se);

		target_cpu = hmp_select_cpu(HMP_GB, p, &fast_cpu_mask, -1, 0);
		if (target_cpu >= num_possible_cpus()) {
			raw_spin_unlock_irqrestore(&target->lock, flags);
			continue;
		}

		/* Collect cluster information */
		memset(&clbenv, 0, sizeof(clbenv));
		clbenv.flags |= HMP_GB;
		clbenv.ltarget = curr_cpu;
		clbenv.btarget = target_cpu;
		cpumask_copy(&clbenv.lcpus, &slow_cpu_mask);
		cpumask_copy(&clbenv.bcpus, &fast_cpu_mask);
		sched_update_clbstats(&clbenv);

		/* Check migration threshold */
		if (!target->active_balance &&
				hmp_up_migration(curr_cpu,
					&target_cpu, se, &clbenv) &&
				!cpu_park(cpu_of(target))) {
			if (p->state != TASK_DEAD) {
				get_task_struct(p);
				target->active_balance = 1; /* force up */
				target->push_cpu = target_cpu;
				target->migrate_task = p;
				force = 1;
				trace_sched_hmp_migrate(p, target->push_cpu, 2);
				hmp_next_up_delay(&p->se, target->push_cpu);
			}
		}

		raw_spin_unlock_irqrestore(&target->lock, flags);
		if (force) {
			if (stop_one_cpu_dispatch(cpu_of(target),
					hmp_force_up_cpu_stop,
					target, &target->active_balance_work)) {
				put_task_struct(p); /* out of rq->lock */
				raw_spin_lock_irqsave(&target->lock, flags);
				target->active_balance = 0;
				force = 0;
				raw_spin_unlock_irqrestore(
						&target->lock, flags);
			}
		} else
			hmp_force_down_migration(this_cpu);
	}

	trace_sched_hmp_load(100,
			clbenv.bstats.load_avg, clbenv.lstats.load_avg);
	spin_unlock(&hmp_force_migration);

}

static inline void
hmp_enqueue_entity_load_avg(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	int cpu = cfs_rq->rq->cpu;

	cfs_nr_pending(cpu) = 0;
	cfs_pending_load(cpu) = 0;
	cfs_rq->avg.loadwop_avg += se->avg.loadwop_avg;
	cfs_rq->avg.loadwop_sum += se->avg.loadwop_sum;

#ifdef CONFIG_SCHED_HMP_PRIO_FILTER
	if (!task_low_priority(task_of(se)->prio))
		cfs_nr_normal_prio(cpu)++;
#endif
	trace_sched_cfs_enqueue_task(task_of(se), se_load(se), cpu);
}

static inline void
hmp_dequeue_entity_load_avg(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	int cpu = cfs_rq->rq->cpu;

	cfs_reset_nr_dequeuing_low_prio(cpu);
	if (!task_low_priority(task_of(se)->prio))
		cfs_nr_normal_prio(cpu)--;

	cfs_rq->avg.loadwop_avg = max_t(long,
			cfs_rq->avg.loadwop_avg - se->avg.loadwop_avg, 0);
	cfs_rq->avg.loadwop_sum = max_t(s64,
			cfs_rq->avg.loadwop_sum - se->avg.loadwop_sum, 0);

	trace_sched_cfs_dequeue_task(task_of(se), se_load(se), cfs_rq->rq->cpu);
}

/*
 * hmp_idle_pull looks at little domain runqueues to see
 * if a task should be pulled.
 *
 * Reuses hmp_force_migration spinlock.
 *
 */
static unsigned int hmp_idle_pull(int this_cpu)
{
	int cpu;
	struct sched_entity *curr, *orig;
	struct hmp_domain *hmp_domain = NULL;
	struct rq *target = NULL, *rq;
	unsigned long flags, ratio = 0;
	unsigned int moved = 0;
	struct task_struct *p = NULL;
	struct clb_env clbenv;
	struct task_struct *prev_selected = NULL;
	int selected = 0;

	if (!hmp_cpu_is_slowest(this_cpu))
		hmp_domain = hmp_slower_domain(this_cpu);
	if (!hmp_domain)
		return 0;

	if (!spin_trylock(&hmp_force_migration))
		return 0;

	memset(&clbenv, 0, sizeof(clbenv));
	clbenv.flags |= HMP_GB;
	clbenv.btarget = this_cpu;
	cpumask_copy(&clbenv.lcpus, &hmp_domain->possible_cpus);
	cpumask_copy(&clbenv.bcpus, &hmp_cpu_domain(this_cpu)->possible_cpus);

	/* first select a task */
	for_each_cpu(cpu, &hmp_domain->cpus) {
		rq = cpu_rq(cpu);
		raw_spin_lock_irqsave(&rq->lock, flags);
		curr = rq->cfs.curr;
		if (!curr) {
			raw_spin_unlock_irqrestore(&rq->lock, flags);
			continue;
		}
		if (!entity_is_task(curr)) {
			struct cfs_rq *cfs_rq;

			cfs_rq = group_cfs_rq(curr);
			while (cfs_rq) {
				curr = cfs_rq->curr;
				if (!entity_is_task(curr))
					cfs_rq = group_cfs_rq(curr);
				else
					cfs_rq = NULL;
			}
		}

		orig = curr;
		curr = hmp_get_heaviest_task(curr, this_cpu);
		/* check if heaviest eligible task on this
		 * CPU is heavier than previous task
		 */
		clbenv.ltarget = cpu;
		sched_update_clbstats(&clbenv);

		if (curr && entity_is_task(curr) &&
				(se_load(curr) > clbenv.bstats.threshold) &&
				(se_load(curr) > ratio) &&
				cpumask_test_cpu(this_cpu,
					tsk_cpus_allowed(task_of(curr)))) {
			selected = 1;
			/* get task and selection inside rq lock  */
			p = task_of(curr);
			get_task_struct(p);

			target = rq;
			ratio = curr->avg.loadwop_avg;
		}

		raw_spin_unlock_irqrestore(&rq->lock, flags);

		if (selected) {
			if (prev_selected) /* To put task out of rq lock */
				put_task_struct(prev_selected);
			prev_selected = p;
			selected = 0;
		}
	}
	if (!p)
		goto done;

	moved = migrate_running_task(this_cpu, p, target);
done:
	spin_unlock(&hmp_force_migration);
	if (p)
		put_task_struct(p);
	return moved;
}

/* must hold runqueue lock for queue se is currently on */
static const int hmp_max_tasks = 5;
static struct sched_entity *hmp_get_heaviest_task(
		struct sched_entity *se, int target_cpu)
{
	int num_tasks = hmp_max_tasks;
	struct sched_entity *max_se = 0;
	long int max_ratio = -1;
	const struct cpumask *hmp_target_mask = NULL;
	struct hmp_domain *hmp;

	if (hmp_cpu_is_fastest(cpu_of(se->cfs_rq->rq)))
		return max_se;

	if (!task_prefer_little(task_of(se))) {
		max_se = se;
		max_ratio = se->avg.loadwop_avg;
	}

	hmp = hmp_faster_domain(cpu_of(se->cfs_rq->rq));
	hmp_target_mask = &hmp->cpus;
	if (target_cpu >= 0) {
		/* idle_balance gets run on a CPU while
		 * it is in the middle of being hotplugged
		 * out. Bail early in that case.
		 */
		if (!cpumask_test_cpu(target_cpu, hmp_target_mask))
			return NULL;
		hmp_target_mask = cpumask_of(target_cpu);
	}
	/* The currently running task is not on the runqueue */
	se = __pick_first_entity(cfs_rq_of(se));
	while (num_tasks && se) {
		if (entity_is_task(se) && se->avg.loadwop_avg > max_ratio &&
				cpumask_intersects(hmp_target_mask,
					tsk_cpus_allowed(task_of(se)))) {
			max_se = se;
			max_ratio = se->avg.loadwop_avg;
		}
		se = __pick_next_entity(se);
		num_tasks--;
	}
	return max_se;
}

static struct sched_entity *hmp_get_lightest_task(
		struct sched_entity *se, int migrate_down)
{
	int num_tasks = hmp_max_tasks;
	struct sched_entity *min_se = 0;
	unsigned long int min_ratio = INT_MAX;
	const struct cpumask *hmp_target_mask = NULL;

	if (migrate_down) {
		struct hmp_domain *hmp;

		if (hmp_cpu_is_slowest(cpu_of(se->cfs_rq->rq)))
			return min_se;
		hmp = hmp_slower_domain(cpu_of(se->cfs_rq->rq));
		hmp_target_mask = &hmp->cpus;
	}

	if (!task_prefer_big(task_of(se))) {
		min_se = se;
		min_ratio = se->avg.loadwop_avg;
	}

	/* The currently running task is not on the runqueue */
	se = __pick_first_entity(cfs_rq_of(se));

	while (num_tasks && se) {
		if (entity_is_task(se) &&
				(se->avg.loadwop_avg < min_ratio
				 && hmp_target_mask &&
				 cpumask_intersects(hmp_target_mask,
					 tsk_cpus_allowed(task_of(se))))) {
			min_se = se;
			min_ratio = se->avg.loadwop_avg;
		}
		se = __pick_next_entity(se);
		num_tasks--;
	}
	return min_se;
}

inline int hmp_fork_balance(struct task_struct *p, int prev_cpu)
{
	int new_cpu = prev_cpu;
	int cpu = smp_processor_id();

	if (hmp_cpu_is_fastest(prev_cpu)) {
		/* prev_cpu is fastest domain */
		struct hmp_domain *hmpdom;
		__always_unused int lowest_ratio;

		hmpdom = list_entry(
				&hmp_cpu_domain(prev_cpu)->hmp_domains,
				struct hmp_domain, hmp_domains);

		lowest_ratio = hmp_domain_min_load(hmpdom, &new_cpu);

		if (new_cpu < nr_cpu_ids &&
				cpumask_test_cpu(new_cpu, tsk_cpus_allowed(p))
				&& !cpu_isolated(new_cpu))
			return new_cpu;

		new_cpu = cpumask_any_and(&hmp_faster_domain(cpu)->cpus,
				tsk_cpus_allowed(p));

		if (new_cpu < nr_cpu_ids)
			return new_cpu;
	} else {
		/* prev_cpu is not fastest domain */
		new_cpu = hmp_select_faster_cpu(p, prev_cpu);

		if (new_cpu < nr_cpu_ids)
			return new_cpu;
	}

	return new_cpu;
}
#endif

#ifdef CONFIG_HMP_FREQUENCY_INVARIANT_SCALE

#include <linux/cpufreq.h>

static int cpufreq_callback(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct cpufreq_freqs *freq = data;
	int cpu = freq->cpu;
	struct cpumask cls_cpus;
	int id;

	if (freq->flags & CPUFREQ_CONST_LOOPS)
		return NOTIFY_OK;

	if (val == CPUFREQ_PRECHANGE) {
		arch_get_cluster_cpus(&cls_cpus, arch_get_cluster_id(cpu));
		for_each_cpu(id, &cls_cpus)
			arch_scale_set_curr_freq(id, freq->new);
	}

	return NOTIFY_OK;
}

static struct notifier_block cpufreq_notifier = {
	.notifier_call = cpufreq_callback,
};

static int cpufreq_policy_callback(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct cpufreq_policy *policy = data;
	int i;

	if (val != CPUFREQ_NOTIFY)
		return NOTIFY_OK;

	for_each_cpu(i, policy->cpus) {
		arch_scale_set_curr_freq(i, policy->cur);
		arch_scale_set_max_freq(i, policy->max);
		arch_scale_set_min_freq(i, policy->min);
	}

	return NOTIFY_OK;
}

static struct notifier_block cpufreq_policy_notifier = {
	.notifier_call = cpufreq_policy_callback,
};

static int __init register_cpufreq_notifier(void)
{
	int ret;

	ret = cpufreq_register_notifier(&cpufreq_notifier,
			CPUFREQ_TRANSITION_NOTIFIER);
	if (ret)
		return ret;

	return cpufreq_register_notifier(&cpufreq_policy_notifier,
			CPUFREQ_POLICY_NOTIFIER);
}
core_initcall(register_cpufreq_notifier);
#endif /* CONFIG_HMP_FREQUENCY_INVARIANT_SCALE */

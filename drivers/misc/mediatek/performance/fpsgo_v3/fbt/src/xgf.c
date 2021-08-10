/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/rbtree.h>
#include <linux/preempt.h>
#include <linux/proc_fs.h>
#include <linux/debugfs.h>
#include <linux/vmalloc.h>
#include <linux/trace_events.h>
#include <linux/module.h>
#include <linux/cpumask.h>

#include <mt-plat/fpsgo_common.h>

#include <trace/events/fpsgo.h>
#include <trace/events/sched.h>
#include <trace/events/ipi.h>
#include <trace/events/irq.h>
#include <trace/events/mtk_events.h>

#include "xgf.h"
#include "fpsgo_base.h"

FPSFO_DECLARE_SYSTRACE(x, irq_handler_entry)
FPSFO_DECLARE_SYSTRACE(x, irq_handler_exit)
FPSFO_DECLARE_SYSTRACE(x, softirq_entry)
FPSFO_DECLARE_SYSTRACE(x, softirq_exit)
FPSFO_DECLARE_SYSTRACE(x, ipi_raise)
FPSFO_DECLARE_SYSTRACE(x, ipi_entry)
FPSFO_DECLARE_SYSTRACE(x, ipi_exit)
FPSFO_DECLARE_SYSTRACE(x, sched_wakeup)
FPSFO_DECLARE_SYSTRACE(x, sched_wakeup_new)
FPSFO_DECLARE_SYSTRACE(x, sched_switch)

static DEFINE_MUTEX(xgf_main_lock);
static int xgf_enable;
static int xgf_ko_ready;
static struct dentry *debugfs_xgf_dir;
static unsigned long long last_check2recycle_ts;
static atomic_t xgf_atomic_val_0 = ATOMIC_INIT(0);
static atomic_t xgf_atomic_val_1 = ATOMIC_INIT(0);
static unsigned long long last_update2spid_ts;
static char *xgf_sp_name = SP_ALLOW_NAME;
module_param(xgf_sp_name, charp, 0644);

HLIST_HEAD(xgf_renders);
HLIST_HEAD(xgf_hw_events);

int (*xgf_est_runtime_fp)(
	pid_t r_pid,
	struct xgf_render *render,
	unsigned long long *runtime,
	unsigned long long ts
	);
EXPORT_SYMBOL(xgf_est_runtime_fp);

int (*xgf_stat_xchg_fp)(int enable);
EXPORT_SYMBOL(xgf_stat_xchg_fp);

static inline void xgf_lock(const char *tag)
{
	mutex_lock(&xgf_main_lock);
}

static inline void xgf_unlock(const char *tag)
{
	mutex_unlock(&xgf_main_lock);
}

void xgf_lockprove(const char *tag)
{
	WARN_ON(!mutex_is_locked(&xgf_main_lock));
}
EXPORT_SYMBOL(xgf_lockprove);

void xgf_trace(const char *fmt, ...)
{
	char log[256];
	va_list args;
	int len;


	va_start(args, fmt);
	len = vsnprintf(log, sizeof(log), fmt, args);

	if (unlikely(len == 256))
		log[255] = '\0';
	va_end(args);
	trace_xgf_log(log);
}
EXPORT_SYMBOL(xgf_trace);

void *xgf_alloc(int i32Size)
{
	void *pvBuf;

	if (i32Size <= PAGE_SIZE)
		pvBuf = kzalloc(i32Size, GFP_ATOMIC);
	else
		pvBuf = vzalloc(i32Size);

	return pvBuf;
}
EXPORT_SYMBOL(xgf_alloc);

void xgf_free(void *pvBuf)
{
	kvfree(pvBuf);
}
EXPORT_SYMBOL(xgf_free);

void *xgf_atomic_val_assign(int select)
{
	if (!select)
		return (void *)(&xgf_atomic_val_0);
	else
		return (void *)(&xgf_atomic_val_1);
}
EXPORT_SYMBOL(xgf_atomic_val_assign);

int xgf_atomic_read(atomic_t *val)
{
	return atomic_read(val);
}
EXPORT_SYMBOL(xgf_atomic_read);

int xgf_atomic_inc_return(atomic_t *val)
{
	return atomic_inc_return(val);
}
EXPORT_SYMBOL(xgf_atomic_inc_return);

void xgf_atomic_set(atomic_t *val, int i)
{
	atomic_set(val, i);
}
EXPORT_SYMBOL(xgf_atomic_set);

unsigned int xgf_cpumask_next(int cpu,  const struct cpumask *srcp)
{
	return cpumask_next(cpu, srcp);
}
EXPORT_SYMBOL(xgf_cpumask_next);

int xgf_num_possible_cpus(void)
{
	return num_possible_cpus();
}
EXPORT_SYMBOL(xgf_num_possible_cpus);

int xgf_get_task_wake_cpu(struct task_struct *t)
{
	return t->wake_cpu;
}
EXPORT_SYMBOL(xgf_get_task_wake_cpu);

int xgf_get_task_pid(struct task_struct *t)
{
	return t->pid;
}
EXPORT_SYMBOL(xgf_get_task_pid);

long xgf_get_task_state(struct task_struct *t)
{
	return t->state;
}
EXPORT_SYMBOL(xgf_get_task_state);

unsigned long xgf_lookup_name(const char *name)
{
	return kallsyms_lookup_name(name);
}
EXPORT_SYMBOL(xgf_lookup_name);

static inline int xgf_ko_is_ready(void)
{
	xgf_lockprove(__func__);

	return xgf_ko_ready;
}

static inline int xgf_is_enable(void)
{
	xgf_lockprove(__func__);

	return xgf_enable;
}

unsigned long long xgf_get_time(void)
{
	unsigned long long temp;

	preempt_disable();
	temp = cpu_clock(smp_processor_id());
	preempt_enable();
	return temp;
}
EXPORT_SYMBOL(xgf_get_time);

static void xgf_reset_render_sector(struct xgf_render *render)
{
	struct hlist_node *s, *p;
	struct xgf_render_sector *sect_iter;
	struct xgf_pid_rec *pids_iter;

	xgf_lockprove(__func__);

	hlist_for_each_entry_safe(sect_iter, s, &render->sector_head, hlist) {
		hlist_for_each_entry_safe(pids_iter, p,
				&sect_iter->path_head, hlist) {
			hlist_del(&pids_iter->hlist);
			xgf_free(pids_iter);
		}
		INIT_HLIST_HEAD(&sect_iter->path_head);

		hlist_del(&sect_iter->hlist);
		xgf_free(sect_iter);
	}
	INIT_HLIST_HEAD(&render->sector_head);
	render->sector_nr = 0;
}

static void xgf_reset_render_hw_list(struct xgf_render *render)
{
	struct hlist_node *h;
	struct xgf_hw_rec *hw_iter;

	xgf_lockprove(__func__);

	hlist_for_each_entry_safe(hw_iter, h, &render->hw_head, hlist) {
		hlist_del(&hw_iter->hlist);
		xgf_free(hw_iter);
	}
	INIT_HLIST_HEAD(&render->hw_head);
}

static void xgf_clean_hw_events(void)
{
	struct xgf_hw_event *iter;
	struct hlist_node *t;

	xgf_lockprove(__func__);

	hlist_for_each_entry_safe(iter, t, &xgf_hw_events, hlist) {
		hlist_del(&iter->hlist);
		xgf_free(iter);
	}
}

void xgf_clean_deps_list(struct xgf_render *render, int pos)
{
	struct xgf_dep *iter;

	xgf_lockprove(__func__);

	if (pos == INNER_DEPS) {
		while (!RB_EMPTY_ROOT(&render->deps_list)) {
			iter = rb_entry(render->deps_list.rb_node,
						struct xgf_dep, rb_node);
			rb_erase(&iter->rb_node, &render->deps_list);
			xgf_free(iter);
		}
	}

	if (pos == OUTER_DEPS) {
		while (!RB_EMPTY_ROOT(&render->out_deps_list)) {
			iter = rb_entry(render->out_deps_list.rb_node,
						struct xgf_dep, rb_node);
			rb_erase(&iter->rb_node, &render->out_deps_list);
			xgf_free(iter);
		}
	}

	if (pos == PREVI_DEPS) {
		while (!RB_EMPTY_ROOT(&render->prev_deps_list)) {
			iter = rb_entry(render->prev_deps_list.rb_node,
						struct xgf_dep, rb_node);
			rb_erase(&iter->rb_node, &render->prev_deps_list);
			xgf_free(iter);
		}
	}
}
EXPORT_SYMBOL(xgf_clean_deps_list);

struct xgf_dep *xgf_get_dep(
	pid_t tid, struct xgf_render *render, int pos, int force)
{
	struct rb_root *r = NULL;
	struct rb_node **p = NULL;
	struct rb_node *parent = NULL;
	struct xgf_dep *xd = NULL;
	pid_t tp;

	xgf_lockprove(__func__);

	switch (pos) {
	case INNER_DEPS:
		p = &render->deps_list.rb_node;
		r = &render->deps_list;
		break;

	case OUTER_DEPS:
		p = &render->out_deps_list.rb_node;
		r = &render->out_deps_list;
		break;

	case PREVI_DEPS:
		p = &render->prev_deps_list.rb_node;
		r = &render->prev_deps_list;
		break;

	default:
		return NULL;
	}

	while (*p) {
		parent = *p;
		xd = rb_entry(parent, struct xgf_dep, rb_node);

		tp = xd->tid;
		if (tid < tp)
			p = &(*p)->rb_left;
		else if (tid > tp)
			p = &(*p)->rb_right;
		else
			return xd;
	}

	if (!force)
		return NULL;

	xd = xgf_alloc(sizeof(*xd));

	if (!xd)
		return NULL;

	xd->tid = tid;
	xd->render_dep = 1;

	rb_link_node(&xd->rb_node, parent, p);
	rb_insert_color(&xd->rb_node, r);

	return xd;
}
EXPORT_SYMBOL(xgf_get_dep);

static struct xgf_hw_rec *xgf_get_hw_rec(int event_type,
	unsigned long long mid, struct xgf_render *render, int force)
{
	struct xgf_hw_rec *iter;
	struct xgf_hw_rec *xhr = NULL;

	xgf_lockprove(__func__);

	hlist_for_each_entry(iter, &render->hw_head, hlist) {
		if (iter->event_type != event_type)
			continue;

		if (iter->mid != mid)
			continue;

		if (iter)
			xhr = iter;

		return xhr;
	}

	if (!force)
		return NULL;

	xhr = xgf_alloc(sizeof(*xhr));

	if (!xhr)
		return NULL;

	xhr->event_type = event_type;
	xhr->hw_type = PER_FRAME;
	xhr->mid = mid;

	hlist_add_head(&xhr->hlist, &render->hw_head);

	return xhr;
}

static void xgf_hw_event_update(unsigned long long max_ts,
	unsigned long long min_ts, int rpid, struct xgf_render *render)
{
	struct xgf_hw_event *iter;
	struct hlist_node *t;
	struct xgf_dep *xd;
	struct xgf_hw_rec *xhr;

	xgf_lockprove(__func__);

	hlist_for_each_entry_safe(iter, t, &xgf_hw_events, hlist) {
		if (!iter->tail_ts || iter->tail_ts > max_ts
				|| iter->tail_ts < min_ts)
			continue;

		xd = xgf_get_dep(iter->tid, render, INNER_DEPS, 0);
		if (!xd)
			continue;

		iter->render = rpid;
		iter->valid = 1;

		iter->event_index = render->frame_count;

		xhr = xgf_get_hw_rec(iter->event_type, iter->mid, render, 1);
	}
}

static int xgf_valid_hw_index(int check_index, int current_index)
{
	int ret = 0;
	int lower_bound = current_index - HW_MONITER_WINDOW + 1;
	int invert_bound;

	if (check_index < 0 || current_index < 0)
		return ret;

	if (unlikely(lower_bound < 0)) {
		if (check_index >= 0 && check_index <= current_index)
			ret = 1;

		invert_bound = INT_MAX + lower_bound;
		if (check_index > invert_bound)
			ret = 1;
	} else {
		if (check_index <= current_index && check_index >= lower_bound)
			ret = 1;
	}

	return ret;
}

static int xgf_is_valid_hw_event(int rpid, int event_type,
				int index, struct xgf_hw_event *event)
{
	if (event->event_index == -1 || event->event_type != event_type)
		return 0;

	if (event->render != rpid || !event->valid)
		return 0;

	if (!xgf_valid_hw_index(event->event_index, index))
		return 0;

	return 1;
}

static int xgf_hw_index_distance(int input_index, int current_index)
{
	int ret = 0;
	int lower_bound = current_index - HW_MONITER_WINDOW + 1;
	int invert_bound;

	if (input_index < 0 || current_index < 0)
		return ret;

	if (unlikely(lower_bound < 0)) {
		invert_bound = INT_MAX + lower_bound;

		if (input_index <= current_index)
			ret = current_index - input_index + 1;
		else if (input_index >= invert_bound)
			ret = (INT_MAX - input_index + 1) + (current_index + 1);
	} else
		ret = current_index - input_index + 1;

	return ret;
}

static int xgf_hw_type(int rpid, struct xgf_render *render)
{
	int event, curr_index, stored_count, valid_count, t_count;
	int level;
	struct xgf_hw_rec *hr_iter;
	struct hlist_node *hr;
	struct xgf_hw_event *he_iter;
	struct hlist_node *he;

	xgf_lockprove(__func__);

	for (event = 0; event < HW_EVENT_NUM; event++) {
		curr_index = render->frame_count;

		hlist_for_each_entry_safe(hr_iter, hr,
						&render->hw_head, hlist) {

			if (hr_iter->event_type != event)
				continue;

			valid_count = 0;
			stored_count = 0;
			level = 0;

			hlist_for_each_entry_safe(he_iter, he,
							&xgf_hw_events, hlist) {

				if (hr_iter->mid != he_iter->mid)
					continue;

				if (!xgf_is_valid_hw_event(rpid,
						event, curr_index, he_iter))
					continue;

				t_count = xgf_hw_index_distance(
					he_iter->event_index, curr_index);

				if (t_count > stored_count)
					stored_count = t_count;

				valid_count++;
			}
			xgf_trace("xgf hw type valid_count:%d stored_count:%d",
						valid_count, stored_count);

			if (valid_count > 0 && stored_count > 0) {
				level = (valid_count*10)/stored_count;
				if (level >= HW_MONITER_LEVEL)
					hr_iter->hw_type = PER_FRAME;
				else
					hr_iter->hw_type = CRO_FRAME;
			} else {
				if (stored_count > 0)
					hr_iter->hw_type = BACKGROUND;
				else {
					hlist_del(&hr_iter->hlist);
					xgf_free(hr_iter);
				}
			}
		}
	}

	return 1;
}

int xgf_hw_events_update(int rpid, struct xgf_render *render)
{
	int ret = 0;
	unsigned long long min_ts, max_ts;
	struct xgf_hw_event *iter;
	struct hlist_node *t, *p, *s;
	struct xgf_pid_rec *pids_iter;
	struct xgf_render_sector *xrs;
	struct xgf_hw_rec *xhr;

	xgf_lockprove(__func__);

	max_ts = render->curr_ts;
	min_ts = render->prev_ts;

	xgf_hw_event_update(max_ts, min_ts, rpid, render);

	ret = xgf_hw_type(rpid, render);

	/* ignore run time calculate when cross frame */
	hlist_for_each_entry_safe(iter, t, &xgf_hw_events, hlist) {
		if (!iter->tail_ts || !iter->valid
			|| iter->tail_ts > max_ts || iter->tail_ts < min_ts)
			continue;

		hlist_for_each_entry_safe(xrs, s, &render->sector_head, hlist) {
			if (iter->tail_ts > xrs->tail_ts
				|| iter->tail_ts < xrs->head_ts)
				continue;

			hlist_for_each_entry_safe(pids_iter, p,
				&xrs->path_head, hlist) {
				if (pids_iter->pid == iter->tid) {
					xhr = xgf_get_hw_rec(iter->event_type,
					iter->mid, render, 0);
					if (!xhr)
						continue;

					if (xhr->hw_type == CRO_FRAME)
						xrs->counted = 0;
				}
			}
		}
	}

	return ret;
}
EXPORT_SYMBOL(xgf_hw_events_update);

static void xgf_hw_event_begin(int event_type, int tid,
				unsigned long long mid)
{
	struct xgf_hw_event *event;
	unsigned long long now_ts = xgf_get_time();

	xgf_lockprove(__func__);

	event = xgf_alloc(sizeof(*event));

	if (event == NULL)
		return;

	event->event_index = -1;
	event->render = 0;
	event->valid = 0;
	event->event_type = event_type;
	event->head_ts = now_ts;
	event->tid = tid;
	event->mid = mid;
	event->tail_ts = 0;

	hlist_add_head(&event->hlist, &xgf_hw_events);
}

static int xgf_hw_event_end(int event_type, int tid,
				unsigned long long mid)
{
	struct xgf_hw_event *event;
	struct xgf_hw_rec *xhr;
	struct xgf_render *r_iter;
	struct hlist_node *tmp, *r_tmp;
	unsigned long long now_ts = xgf_get_time();
	int ret = PER_FRAME;

	xgf_lockprove(__func__);

	hlist_for_each_entry_safe(event, tmp, &xgf_hw_events, hlist) {
		if (event->tid != tid || event->tail_ts)
			continue;

		event->tail_ts = now_ts;
	}

	hlist_for_each_entry_safe(r_iter, r_tmp, &xgf_renders, hlist) {
		xhr = xgf_get_hw_rec(event_type, mid, r_iter, 0);

		if (xhr) {
			ret = xhr->hw_type;
			break;
		}
	}

	return ret;
}

static int xgf_hw_event_collect(int event_type, int tid,
				unsigned long long mid, int cmd)
{
	int ret = PER_FRAME;

	xgf_lock(__func__);

	if (!xgf_is_enable()) {
		xgf_unlock(__func__);
		return ret;
	}

	switch (cmd) {
	case 1:
		xgf_hw_event_begin(event_type, tid, mid);
		break;

	case 0:
	default:
		ret = xgf_hw_event_end(event_type, tid, mid);
		break;
	}

	xgf_unlock(__func__);

	return ret;
}

static int xgf_get_render(pid_t rpid, struct xgf_render **ret, int force)
{
	struct xgf_render *iter;

	xgf_lockprove(__func__);

	hlist_for_each_entry(iter, &xgf_renders, hlist) {
		if (iter->render != rpid)
			continue;

		if (ret)
			*ret = iter;
		return 0;
	}

	if (!force)
		return -EINVAL;

	iter = xgf_alloc(sizeof(*iter));

	if (!iter)
		return -ENOMEM;

	{
		struct task_struct *tsk;

		rcu_read_lock();
		tsk = find_task_by_vpid(rpid);
		if (tsk)
			get_task_struct(tsk);
		rcu_read_unlock();

		if (!tsk) {
			xgf_free(iter);
			return -EINVAL;
		}

		iter->parent = tsk->tgid;
		iter->render = rpid;
		put_task_struct(tsk);

		iter->sector_nr = 0;
		iter->curr_index = 0;
		iter->curr_ts = 0;
		iter->prev_index = 0;
		iter->prev_ts = 0;
		iter->event_count = 0;
		iter->frame_count = 0;
		iter->queue.start_ts = 0;
		iter->queue.end_ts = 0;
		iter->deque.start_ts = 0;
		iter->deque.end_ts = 0;
		iter->ema_runtime = 0;
		iter->spid = 0;
	}

	INIT_HLIST_HEAD(&iter->sector_head);
	INIT_HLIST_HEAD(&iter->hw_head);
	hlist_add_head(&iter->hlist, &xgf_renders);

	if (ret)
		*ret = iter;
	return 0;
}

static inline int xgf_ull_multi_overflow(
	unsigned long long multiplicand, int multiplier)
{
	int ret = 0;
	unsigned long long div_after_multi;

	div_after_multi = div_u64((multiplicand*multiplier), multiplier);
	if (multiplicand != div_after_multi)
		ret = 1;

	return ret;
}

static inline unsigned long long xgf_runtime_pro_rata(
	unsigned long long val, int dividend, int divisor)
{
	unsigned long long ret = 0;

	if (xgf_ull_multi_overflow(val, dividend))
		ret = div_u64(ULLONG_MAX, divisor);
	else
		ret = div_u64((val*dividend), divisor);

	return ret;
}

static inline unsigned long long xgf_ema_cal(
	unsigned long long curr, unsigned long long prev)
{
	unsigned long long ret = 0;
	unsigned long long curr_new, prev_new;

	if (!prev) {
		ret = curr;
		return ret;
	}

	curr_new = xgf_runtime_pro_rata(curr, EMA_DIVIDEND, EMA_DIVISOR);
	prev_new = xgf_runtime_pro_rata(prev, EMA_REST_DIVIDEND, EMA_DIVISOR);

	if (prev_new > (ULLONG_MAX - curr_new))
		ret = ULLONG_MAX; /* overflow */
	else
		ret = curr_new + prev_new;

	return ret;
}

void fpsgo_create_render_dep(void) {}

void fpsgo_update_render_dep(struct task_struct *p) {}

void xgf_igather_timer(const void * const timer, int fire) {}

void xgf_epoll_igather_timer(
	const void * const timer, ktime_t *expires, int fire)
{}

int has_xgf_dep(pid_t tid)
{
	struct xgf_dep *out_xd, *prev_xd;
	struct xgf_render *render_iter;
	struct hlist_node *n;
	pid_t query_tid;
	int ret = 0;

	xgf_lock(__func__);

	query_tid = tid;

	hlist_for_each_entry_safe(render_iter, n, &xgf_renders, hlist) {
		out_xd = xgf_get_dep(query_tid, render_iter, OUTER_DEPS, 0);
		prev_xd = xgf_get_dep(query_tid, render_iter, PREVI_DEPS, 0);

		if (!out_xd && !prev_xd)
			continue;

		if ((out_xd && out_xd->render_dep)
			|| (prev_xd && prev_xd->render_dep))
			ret = 1;

		if (ret)
			break;
	}

	xgf_unlock(__func__);
	return ret;
}

int fpsgo_fteh2xgf_get_dep_list_num(int pid)
{
	struct xgf_render *render_iter;
	struct hlist_node *n;
	struct rb_node *out_rbn;
	struct rb_node *pre_rbn;
	struct xgf_dep *out_iter;
	struct xgf_dep *pre_iter;
	int counts = 0;

	if (!pid)
		goto out;

	xgf_lock(__func__);

	hlist_for_each_entry_safe(render_iter, n, &xgf_renders, hlist) {
		if (render_iter->render != pid)
			continue;

		out_rbn = rb_first(&render_iter->out_deps_list);
		pre_rbn = rb_first(&render_iter->prev_deps_list);

		while (out_rbn != NULL && pre_rbn != NULL) {
			out_iter = rb_entry(out_rbn, struct xgf_dep, rb_node);
			pre_iter = rb_entry(pre_rbn, struct xgf_dep, rb_node);

			if (out_iter->tid < pre_iter->tid) {
				if (out_iter->render_dep)
					counts++;
				out_rbn = rb_next(out_rbn);
			} else if (out_iter->tid > pre_iter->tid) {
				if (pre_iter->render_dep)
					counts++;
				pre_rbn = rb_next(pre_rbn);
			} else {
				if (out_iter->render_dep
						|| pre_iter->render_dep)
					counts++;
				out_rbn = rb_next(out_rbn);
				pre_rbn = rb_next(pre_rbn);
			}
		}

		while (out_rbn != NULL) {
			out_iter = rb_entry(out_rbn, struct xgf_dep, rb_node);
			if (out_iter->render_dep)
				counts++;
			out_rbn = rb_next(out_rbn);
		}

		while (pre_rbn != NULL) {
			pre_iter = rb_entry(pre_rbn, struct xgf_dep, rb_node);
			if (pre_iter->render_dep)
				counts++;
			pre_rbn = rb_next(pre_rbn);
		}
	}

	xgf_unlock(__func__);

out:
	return counts;
}

int fpsgo_fteh2xgf_get_dep_list(int pid, int count, struct fpsgo_loading *arr)
{
	struct xgf_render *render_iter;
	struct hlist_node *n;
	struct rb_node *out_rbn;
	struct rb_node *pre_rbn;
	struct xgf_dep *out_iter;
	struct xgf_dep *pre_iter;
	int index = 0;

	if (!pid || !count)
		return 0;

	xgf_lock(__func__);

	hlist_for_each_entry_safe(render_iter, n, &xgf_renders, hlist) {
		if (render_iter->render != pid)
			continue;

		out_rbn = rb_first(&render_iter->out_deps_list);
		pre_rbn = rb_first(&render_iter->prev_deps_list);

		while (out_rbn != NULL && pre_rbn != NULL) {
			out_iter = rb_entry(out_rbn, struct xgf_dep, rb_node);
			pre_iter = rb_entry(pre_rbn, struct xgf_dep, rb_node);

			if (out_iter->tid < pre_iter->tid) {
				if (out_iter->render_dep && index < count) {
					arr[index].pid = out_iter->tid;
					index++;
				}
				out_rbn = rb_next(out_rbn);
			} else if (out_iter->tid > pre_iter->tid) {
				if (pre_iter->render_dep && index < count) {
					arr[index].pid = pre_iter->tid;
					index++;
				}
				pre_rbn = rb_next(pre_rbn);
			} else {
				if ((out_iter->render_dep
						|| pre_iter->render_dep)
						&& index < count) {
					arr[index].pid = out_iter->tid;
					index++;
				}
				out_rbn = rb_next(out_rbn);
				pre_rbn = rb_next(pre_rbn);
			}
		}

		while (out_rbn != NULL) {
			out_iter = rb_entry(out_rbn, struct xgf_dep, rb_node);
			if (out_iter->render_dep && index < count) {
				arr[index].pid = out_iter->tid;
				index++;
			}
			out_rbn = rb_next(out_rbn);
		}

		while (pre_rbn != NULL) {
			pre_iter = rb_entry(pre_rbn, struct xgf_dep, rb_node);
			if (pre_iter->render_dep && index < count) {
				arr[index].pid = pre_iter->tid;
				index++;
			}
			pre_rbn = rb_next(pre_rbn);
		}
	}

	xgf_unlock(__func__);

	return index;
}

void xgf_reset_renders(void)
{
	struct xgf_render *r_iter;
	struct hlist_node *r_tmp;

	xgf_lockprove(__func__);

	hlist_for_each_entry_safe(r_iter, r_tmp, &xgf_renders, hlist) {
		xgf_clean_deps_list(r_iter, INNER_DEPS);
		xgf_clean_deps_list(r_iter, OUTER_DEPS);
		xgf_clean_deps_list(r_iter, PREVI_DEPS);
		xgf_reset_render_sector(r_iter);
		xgf_reset_render_hw_list(r_iter);
		hlist_del(&r_iter->hlist);
		xgf_free(r_iter);
	}

	xgf_clean_hw_events();
}
EXPORT_SYMBOL(xgf_reset_renders);

void fpsgo_ctrl2xgf_nn_job_begin(unsigned int tid, unsigned long long mid)
{
	xgf_hw_event_collect(HW_AI, tid, mid, 1);
}

int fpsgo_ctrl2xgf_nn_job_end(unsigned int tid, unsigned long long mid)
{
	int hw_type = PER_FRAME;

	hw_type = xgf_hw_event_collect(HW_AI, tid, mid, 0);

	return hw_type;
}

static void xgf_enter_state_xchg(int enable)
{
	int ret = 0;

	if (enable != 0 && enable != 1) {
		ret = -1;
		goto out;
	}

	WARN_ON(!xgf_stat_xchg_fp);

	if (xgf_stat_xchg_fp)
		ret = xgf_stat_xchg_fp(enable);

out:
	xgf_trace("xgf k2ko xchg ret:%d enable:%d", ret, enable);
}

void notify_xgf_ko_ready(void)
{
	xgf_lock(__func__);

	xgf_ko_ready = 1;

	if (xgf_is_enable())
		xgf_enter_state_xchg(xgf_enable);

	xgf_unlock(__func__);
}
EXPORT_SYMBOL(notify_xgf_ko_ready);

void fpsgo_ctrl2xgf_switch_xgf(int val)
{
	xgf_lock(__func__);
	if (val != xgf_enable) {
		xgf_enable = val;

		xgf_reset_renders();

		if (xgf_ko_is_ready())
			xgf_enter_state_xchg(xgf_enable);
	}

	xgf_unlock(__func__);
}

void fpsgo_fstb2xgf_do_recycle(int fstb_active)
{
	unsigned long long now_ts = xgf_get_time();
	long long diff, check_period, recycle_period;
	struct xgf_hw_event *hw_iter;
	struct hlist_node *hw_t;

	/* over 1 seconds since last check2recycle */
	check_period = NSEC_PER_SEC;
	recycle_period = NSEC_PER_SEC >> 1;
	diff = (long long)now_ts - (long long)last_check2recycle_ts;

	xgf_trace("xgf_do_recycle now:%llu last_check:%llu fstb_act:%d",
		now_ts, last_check2recycle_ts, fstb_active);

	xgf_lock(__func__);

	if (!fstb_active) {
		xgf_reset_renders();
		goto out;
	}

	if (diff < 0LL || diff < check_period)
		goto done;

	hlist_for_each_entry_safe(hw_iter, hw_t, &xgf_hw_events, hlist) {
		diff = (long long)now_ts - (long long)hw_iter->head_ts;
		if (diff < check_period)
			continue;

		hlist_del(&hw_iter->hlist);
		xgf_free(hw_iter);
	}

out:
	last_check2recycle_ts = now_ts;
done:
	xgf_unlock(__func__);
}

static char *xgf_strcat(char *dest, const char *src,
	size_t buffersize, int *overflow)
{
	int i, j;
	int bufferbound = buffersize - 1;

	for (i = 0; dest[i] != '\0'; i++)
		;
	for (j = 0; src[j] != '\0'; j++) {
		if ((i+j) < bufferbound)
			dest[i+j] = src[j];

		if ((i+j) == bufferbound) {
			*overflow = 1;
			break;
		}
	}

	if (*overflow)
		dest[bufferbound] = '\0';
	else
		dest[i+j] = '\0';

	return dest;
}

static void xgf_log_trace(const char *fmt, ...)
{
	char log[1024];
	va_list args;
	int len;

	va_start(args, fmt);
	len = vsnprintf(log, sizeof(log), fmt, args);

	if (unlikely(len == 1024))
		log[1023] = '\0';
	va_end(args);
	trace_xgf_log(log);
}

static void xgf_print_debug_log(int rpid,
	struct xgf_render *render, unsigned long long runtime)
{
	char total_pid_list[1024] = {"\0"};
	char pid[20] = {"\0"};
	int overflow = 0;

	struct xgf_render_sector *xrs;
	struct hlist_node *s, *p;
	struct xgf_pid_rec *pids_iter;

	xgf_lockprove(__func__);

	hlist_for_each_entry_safe(xrs, s, &render->sector_head, hlist) {
		if (strlen(total_pid_list) == 0)
			snprintf(pid, sizeof(pid), "%d", xrs->sector_id);
		else
			snprintf(pid, sizeof(pid), "-%d", xrs->sector_id);

		overflow = 0;
		xgf_strcat(total_pid_list, pid,
			sizeof(total_pid_list), &overflow);

		if (overflow)
			goto out;

		hlist_for_each_entry_safe(pids_iter, p,
			&xrs->path_head, hlist) {
			snprintf(pid, sizeof(pid), ",%d", pids_iter->pid);

			overflow = 0;
			xgf_strcat(total_pid_list, pid,
				sizeof(total_pid_list), &overflow);
			if (overflow)
				goto out;
		}
	}

out:
	if (overflow)
		xgf_log_trace("xgf_debug_log(of) r:%d runtime:%llu pid_list:%s",
		rpid, runtime, total_pid_list);
	else
		xgf_log_trace("xgf_debug_log r:%d runtime:%llu pid_list:%s",
		rpid, runtime, total_pid_list);
}

static int xgf_enter_est_runtime(int rpid, struct xgf_render *render,
	unsigned long long *runtime, unsigned long long ts)
{
	int ret;

	WARN_ON(!xgf_est_runtime_fp);

	if (xgf_est_runtime_fp)
		ret = xgf_est_runtime_fp(rpid, render, runtime, ts);
	else
		ret = -ENOENT;

	return ret;
}

static int xgf_get_spid(struct xgf_render *render)
{
	struct rb_root *r;
	struct rb_node *rbn;
	struct xgf_dep *iter;
	int ret = 0;
	unsigned long long now_ts = xgf_get_time();
	long long diff, scan_period;

	xgf_lockprove(__func__);

	scan_period = NSEC_PER_SEC;
	diff = (long long)now_ts - (long long)last_update2spid_ts;

	if (diff < 0LL || diff < scan_period)
		return -1;

	if (!strlen(xgf_sp_name))
		return 0;

	r = &render->deps_list;
	for (rbn = rb_first(r); rbn != NULL; rbn = rb_next(rbn)) {
		struct task_struct *tsk;

		iter = rb_entry(rbn, struct xgf_dep, rb_node);

		rcu_read_lock();
		tsk = find_task_by_vpid(iter->tid);
		if (tsk)
			get_task_struct(tsk);
		rcu_read_unlock();

		if (tsk) {
			if ((tsk->tgid != render->parent)
				|| (task_pid_nr(tsk) == render->render))
				goto tsk_out;

			if (!strncmp(tsk->comm, xgf_sp_name,
				strlen(xgf_sp_name))) {
				ret = task_pid_nr(tsk);
				put_task_struct(tsk);
				goto out;
			}
tsk_out:
			put_task_struct(tsk);
		}
	}
out:
	last_update2spid_ts = now_ts;
	return ret;
}

int fpsgo_comp2xgf_qudeq_notify(int rpid, int cmd,
	unsigned long long *run_time, unsigned long long *mid,
	unsigned long long ts)
{
	int ret = XGF_NOTIFY_OK;
	struct xgf_render *r, **rrender;
	struct xgf_hw_rec *hr_iter;
	struct hlist_node *hr;
	unsigned long long raw_runtime = 0;
	int new_spid;

	xgf_lock(__func__);
	if (!xgf_is_enable()) {
		xgf_unlock(__func__);
		return XGF_DISABLE;
	}

	switch (cmd) {

	case XGF_QUEUE_START:
		rrender = &r;
		if (xgf_get_render(rpid, rrender, 0)) {
			ret = XGF_THREAD_NOT_FOUND;
			goto qudeq_notify_err;
		}
		r->queue.start_ts = ts;
		xgf_reset_render_sector(r);
		break;

	case XGF_QUEUE_END:
		rrender = &r;
		if (xgf_get_render(rpid, rrender, 1)) {
			ret = XGF_THREAD_NOT_FOUND;
			goto qudeq_notify_err;
		}
		r->queue.end_ts = ts;

		new_spid = xgf_get_spid(r);
		if (new_spid != -1) {
			xgf_trace("xgf spid:%d => %d", r->spid, new_spid);
			r->spid = new_spid;
		}
		ret = xgf_enter_est_runtime(rpid, r, &raw_runtime, ts);

		if (!raw_runtime)
			*run_time = raw_runtime;
		else {
			r->ema_runtime =
				xgf_ema_cal(raw_runtime, r->ema_runtime);

			*run_time = r->ema_runtime;
		}

		fpsgo_systrace_c_fbt(rpid, raw_runtime, "raw_t_cpu");

		/* hw event for fbt */
		hlist_for_each_entry_safe(hr_iter, hr, &r->hw_head, hlist) {
			if (hr_iter->hw_type == PER_FRAME)
				*mid = hr_iter->mid;
			xgf_trace("xgf hw mid:%llu event_type:%d hw_type:%d",
			hr_iter->mid, hr_iter->event_type, hr_iter->hw_type);
		}

		xgf_print_debug_log(rpid, r, raw_runtime);
		break;

	case XGF_DEQUEUE_START:
		rrender = &r;
		if (xgf_get_render(rpid, rrender, 0)) {
			ret = XGF_THREAD_NOT_FOUND;
			goto qudeq_notify_err;
		}
		r->deque.start_ts = ts;
		break;

	case XGF_DEQUEUE_END:
		rrender = &r;
		if (xgf_get_render(rpid, rrender, 0)) {
			ret = XGF_THREAD_NOT_FOUND;
			goto qudeq_notify_err;
		}
		r->deque.end_ts = ts;
		break;

	default:
		ret = XGF_PARAM_ERR;
		break;
	}

qudeq_notify_err:
	xgf_unlock(__func__);
	return ret;
}

#define FPSGO_DEBUGFS_ENTRY(name) \
static int fpsgo_##name##_open(struct inode *i, struct file *file) \
{ \
	return single_open(file, fpsgo_##name##_show, i->i_private); \
} \
\
static const struct file_operations fpsgo_##name##_fops = { \
	.owner = THIS_MODULE, \
	.open = fpsgo_##name##_open, \
	.read = seq_read, \
	.write = fpsgo_##name##_write, \
	.llseek = seq_lseek, \
	.release = single_release, \
}

static int fpsgo_deplist_show(struct seq_file *m, void *unused)
{
	struct xgf_render *r_iter;
	struct hlist_node *r_tmp;
	struct rb_root *r;
	struct xgf_dep *iter;
	struct rb_node *n;

	xgf_lock(__func__);

	hlist_for_each_entry_safe(r_iter, r_tmp, &xgf_renders, hlist) {
		r = &r_iter->deps_list;
		for (n = rb_first(r); n != NULL; n = rb_next(n)) {
			iter = rb_entry(n, struct xgf_dep, rb_node);
			seq_printf(m, "render tid:%d inner_deps_tid:%d\n",
				   r_iter->render, iter->tid);
		}

		r = &r_iter->out_deps_list;
		for (n = rb_first(r); n != NULL; n = rb_next(n)) {
			iter = rb_entry(n, struct xgf_dep, rb_node);
			seq_printf(m, "render tid:%d out_deps_tid:%d\n",
				   r_iter->render, iter->tid);
		}

		r = &r_iter->prev_deps_list;
		for (n = rb_first(r); n != NULL; n = rb_next(n)) {
			iter = rb_entry(n, struct xgf_dep, rb_node);
			seq_printf(m, "render tid:%d prev_deps_tid:%d\n",
				   r_iter->render, iter->tid);
		}
	}

	xgf_unlock(__func__);
	return 0;
}

static ssize_t fpsgo_deplist_write(struct file *flip,
			const char *ubuf, size_t cnt, loff_t *data)
{
	return cnt;
}

FPSGO_DEBUGFS_ENTRY(deplist);

int __init init_xgf(void)
{
	if (!fpsgo_debugfs_dir)
		return -ENODEV;

	debugfs_xgf_dir = debugfs_create_dir("xgf",
					     fpsgo_debugfs_dir);
	if (!debugfs_xgf_dir)
		return -ENODEV;

	debugfs_create_file("deplist",
			    0664,
			    debugfs_xgf_dir,
			    NULL,
			    &fpsgo_deplist_fops);

	return 0;
}

/* drivers/misc/lowmemorykiller.c
 *
 * The lowmemorykiller driver lets user-space specify a set of memory thresholds
 * where processes with a range of oom_score_adj values will get killed. Specify
 * the minimum oom_score_adj values in
 * /sys/module/lowmemorykiller/parameters/adj and the number of free pages in
 * /sys/module/lowmemorykiller/parameters/minfree. Both files take a comma
 * separated list of numbers in ascending order.
 *
 * For example, write "0,8" to /sys/module/lowmemorykiller/parameters/adj and
 * "1024,4096" to /sys/module/lowmemorykiller/parameters/minfree to kill
 * processes with a oom_score_adj value of 8 or higher when the free memory
 * drops below 4096 pages and kill processes with a oom_score_adj value of 0 or
 * higher when the free memory drops below 1024 pages.
 *
 * The driver considers memory used for caches to be free, but if a large
 * percentage of the cached memory is locked this can be very inaccurate
 * and processes may not get killed until the normal oom killer is triggered.
 *
 * Copyright (C) 2007-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/oom.h>
#include <linux/sched.h>
#include <linux/swap.h>
#include <linux/rcupdate.h>
#include <linux/profile.h>
#include <linux/notifier.h>
#include <linux/freezer.h>

#define MTK_LMK_USER_EVENT

#ifdef MTK_LMK_USER_EVENT
#include <linux/miscdevice.h>
#endif

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(CONFIG_MTK_ENG_BUILD)
#include <mt-plat/aee.h>
#include <disp_assert_layer.h>
static u32 in_lowmem;
#endif

#ifdef CONFIG_MTK_ION
#include "mtk/ion_drv.h"
#endif

#ifdef CONFIG_MTK_GPU_SUPPORT
#include <mt-plat/mtk_gpu_utility.h>
#endif

#ifdef CONFIG_64BIT
#define ENABLE_AMR_RAMSIZE	(0x80000)	/* > 2GB */
#else
#define ENABLE_AMR_RAMSIZE	(0x40000)	/* > 1GB */
#endif

static short lowmem_debug_adj;	/* default: 0 */
#ifdef CONFIG_MTK_ENG_BUILD
#ifdef CONFIG_MTK_AEE_FEATURE
static short lowmem_kernel_warn_adj;	/* default: 0 */
#endif
#define output_expect(x) likely(x)
static u32 enable_candidate_log = 1;
#define LMK_LOG_BUF_SIZE 500
static u8 lmk_log_buf[LMK_LOG_BUF_SIZE];
#else
#define output_expect(x) unlikely(x)
static u32 enable_candidate_log;
#endif
static DEFINE_SPINLOCK(lowmem_shrink_lock);

#define CREATE_TRACE_POINTS
#include "trace/lowmemorykiller.h"

#include "internal.h"

static u32 lowmem_debug_level = 1;
static short lowmem_adj[9] = {
	0,
	1,
	6,
	12,
};

static int lowmem_adj_size = 4;
static int lowmem_minfree[9] = {
	3 * 512,	/* 6MB */
	2 * 1024,	/* 8MB */
	4 * 1024,	/* 16MB */
	16 * 1024,	/* 64MB */
};

static int lowmem_minfree_size = 4;

#define LOWMEM_DEATHPENDING_TIMEOUT	(HZ)
static unsigned long lowmem_deathpending_timeout;

#define lowmem_print(level, x...)			\
	do {						\
		if (lowmem_debug_level >= (level))	\
			pr_info(x);			\
	} while (0)

static unsigned long lowmem_count(struct shrinker *s,
				  struct shrink_control *sc)
{
#ifdef CONFIG_FREEZER
	/* Do not allow LMK to work when system is freezing */
	if (pm_freezing)
		return 0;
#endif
	return global_page_state(NR_ACTIVE_ANON) +
		global_page_state(NR_ACTIVE_FILE) +
		global_page_state(NR_INACTIVE_ANON) +
		global_page_state(NR_INACTIVE_FILE);
}

#ifdef MTK_LMK_USER_EVENT
static const struct file_operations mtklmk_fops = {
	.owner = THIS_MODULE,
};

static struct miscdevice mtklmk_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mtklmk",
	.fops = &mtklmk_fops,
};

static struct work_struct mtklmk_work;
static int uevent_adj, uevent_minfree;
static void mtklmk_async_uevent(struct work_struct *work)
{
#define MTKLMK_EVENT_LENGTH	(24)
	char adj[MTKLMK_EVENT_LENGTH], free[MTKLMK_EVENT_LENGTH];
	char *envp[3] = { adj, free, NULL };

	snprintf(adj, MTKLMK_EVENT_LENGTH, "OOM_SCORE_ADJ=%d", uevent_adj);
	snprintf(free, MTKLMK_EVENT_LENGTH, "MINFREE=%d", uevent_minfree);
	kobject_uevent_env(&mtklmk_misc.this_device->kobj, KOBJ_CHANGE, envp);
#undef MTKLMK_EVENT_LENGTH
}

static unsigned int mtklmk_initialized;
static unsigned int mtklmk_uevent_timeout = 10000; /* ms */
module_param_named(uevent_timeout, mtklmk_uevent_timeout, uint, 0644);
static void mtklmk_uevent(int oom_score_adj, int minfree)
{
	static unsigned long last_time;
	unsigned long timeout;

	/* change to use jiffies */
	timeout = msecs_to_jiffies(mtklmk_uevent_timeout);

	if (!last_time)
		last_time = jiffies - timeout;

	if (time_before(jiffies, last_time + timeout))
		return;

	last_time = jiffies;

	uevent_adj = oom_score_adj;
	uevent_minfree = minfree;
	schedule_work(&mtklmk_work);
}
#endif

static void dump_memory_status(void)
{
	show_task_mem();
	show_free_areas(0);
#ifdef CONFIG_MTK_ION
	/* Show ION status */
	ion_mm_heap_memory_detail();
#endif
#ifdef CONFIG_MTK_GPU_SUPPORT
	if (mtk_dump_gpu_memory_usage() == false)
		lowmem_print(1, "mtk_dump_gpu_memory_usage not support\n");
#endif
}

static unsigned long lowmem_scan(struct shrinker *s, struct shrink_control *sc)
{
#define LOWMEM_P_STATE_D	(0x1)
#define LOWMEM_P_STATE_R	(0x2)
#define LOWMEM_P_STATE_OTHER	(0x4)

	struct task_struct *tsk;
	struct task_struct *selected = NULL;
	unsigned long rem = 0;
	int tasksize;
	int i;
	short min_score_adj = OOM_SCORE_ADJ_MAX + 1;
	int minfree = 0;
	int selected_tasksize = 0;
	short selected_oom_score_adj;
	int array_size = ARRAY_SIZE(lowmem_adj);
	int other_free = global_page_state(NR_FREE_PAGES) - totalreserve_pages;
	int other_file = global_page_state(NR_FILE_PAGES) -
						global_page_state(NR_SHMEM) -
						global_page_state(NR_UNEVICTABLE) -
						total_swapcache_pages();

	int print_extra_info = 0;
	static unsigned long lowmem_print_extra_info_timeout;
	enum zone_type high_zoneidx = gfp_zone(sc->gfp_mask);
	int p_state_is_found = 0;
	int unreclaimable_zones = 0;
#ifdef CONFIG_SWAP
	int to_be_aggressive = 0;
	unsigned long swap_pages = 0;
	short amr_adj = OOM_SCORE_ADJ_MAX + 1;
#endif
#ifdef CONFIG_MTK_ENG_BUILD
	int pid_dump = -1; /* process to be dump */
	int max_mem = 0;
	static int pid_flm_warn = -1;
	static unsigned long flm_warn_timeout;
	int log_offset = 0, log_ret;
#endif

	/* Subtract CMA free pages from other_free if this is an unmovable page allocation */
	if (IS_ENABLED(CONFIG_CMA))
		if (!(sc->gfp_mask & __GFP_MOVABLE))
			other_free -= global_page_state(NR_FREE_CMA_PAGES);

	if (!spin_trylock(&lowmem_shrink_lock)) {
		lowmem_print(4, "lowmem_shrink lock failed\n");
		return SHRINK_STOP;
	}

	/*
	 * Check whether it is caused by low memory in lower zone(s)!
	 * This will help solve over-reclaiming situation while total number
	 * of free pages is enough, but lower zone(s) is(are) under low memory.
	 */
	if (high_zoneidx < MAX_NR_ZONES - 1) {
		struct pglist_data *pgdat;
		struct zone *z;
		enum zone_type zoneidx;
		unsigned long accumulated_pages = 0, scale = totalram_pages;
		int new_other_free = 0, new_other_file = 0;
		int memory_pressure = 0;

		/* Go through all memory nodes */
		for_each_online_pgdat(pgdat) {
			for (zoneidx = 0; zoneidx <= high_zoneidx; zoneidx++) {
				z = pgdat->node_zones + zoneidx;
				accumulated_pages += z->managed_pages;
				new_other_free += zone_page_state(z, NR_FREE_PAGES);
				new_other_free -= high_wmark_pages(z);
				new_other_file += zone_page_state(z, NR_FILE_PAGES);
				new_other_file -= zone_page_state(z, NR_SHMEM);

				/* Compute memory pressure level */
				memory_pressure += zone_page_state(z, NR_ACTIVE_FILE) +
					zone_page_state(z, NR_INACTIVE_FILE) +
					zone_page_state(z, NR_ACTIVE_ANON) +
					zone_page_state(z, NR_INACTIVE_ANON) +
					new_other_free;

				/* Check whether there is any unreclaimable memory zone */
				if (populated_zone(z) && !zone_reclaimable(z))
					unreclaimable_zones++;
			}
		}

		/*
		 * Update if we go through ONLY lower zone(s) ACTUALLY
		 * and scale in totalram_pages
		 */
		if (totalram_pages > accumulated_pages) {
			do_div(scale, accumulated_pages);
			if (totalram_pages > accumulated_pages * scale)
				scale += 1;
			new_other_free *= scale;
			new_other_file *= scale;
		}

		/* Update if not kswapd or "being kswapd and high memory pressure" */
		if (!current_is_kswapd() || (current_is_kswapd() && memory_pressure < 0)) {
			other_free = new_other_free;
			other_file = new_other_file;
		}
	}

	/* Let other_free be positive or zero */
	if (other_free < 0) {
		/* lowmem_print(1, "Original other_free [%d] is too low!\n", other_free); */
		other_free = 0;
	}

#if defined(CONFIG_64BIT) && defined(CONFIG_SWAP)
	/* Halve other_free if there is less free swap */

	/* Disabled for Ubuntu Touch
	if (vm_swap_full()) {
		lowmem_print(3, "Halve other_free %d\n", other_free);
		other_free >>= 1;
	}*/
#endif

#ifdef CONFIG_SWAP
	swap_pages = atomic_long_read(&nr_swap_pages);
	/* More than 1/2 swap usage */
	/* Disabled for Ubuntu Touch
	if (swap_pages * 2 < total_swap_pages)
		to_be_aggressive++;*/
	/* More than 3/4 swap usage */
	/* Disabled for Ubuntu Touch
	if (swap_pages * 4 < total_swap_pages)
		to_be_aggressive++;*/

#ifndef CONFIG_MTK_GMO_RAM_OPTIMIZE
	/* Try to enable AMR when we have enough memory */
	if (totalram_pages < ENABLE_AMR_RAMSIZE) {
		to_be_aggressive = 0;
	} else {
		i = lowmem_adj_size - 1 - to_be_aggressive;
		if (to_be_aggressive > 0 && i >= 0)
			amr_adj = lowmem_adj[i];
	}
#endif
#endif

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;
	if (lowmem_minfree_size < array_size)
		array_size = lowmem_minfree_size;
	for (i = 0; i < array_size; i++) {
		minfree = lowmem_minfree[i];
		if (other_free < minfree && other_file < minfree) {
#ifdef CONFIG_SWAP
			if (to_be_aggressive != 0 && i > 3) {
				i -= to_be_aggressive;
				if (i < 3)
					i = 3;
			}
#endif
			min_score_adj = lowmem_adj[i];
			break;
		}
	}

#if defined(CONFIG_SWAP) && !defined(CONFIG_MTK_GMO_RAM_OPTIMIZE)
	min_score_adj = min(min_score_adj, amr_adj);
#endif

	/* Promote its priority */
	if (unreclaimable_zones > 0)
		min_score_adj = lowmem_adj[0];

	lowmem_print(3, "lowmem_scan %lu, %x, ofree %d %d, ma %hd\n",
			sc->nr_to_scan, sc->gfp_mask, other_free,
			other_file, min_score_adj);

	if (min_score_adj == OOM_SCORE_ADJ_MAX + 1) {
		lowmem_print(5, "lowmem_scan %lu, %x, return 0\n",
			     sc->nr_to_scan, sc->gfp_mask);

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(CONFIG_MTK_ENG_BUILD)
		/* Disable indication if low memory */
		if (in_lowmem) {
			in_lowmem = 0;
			lowmem_print(1, "LowMemoryOff\n");
		}
#endif
		spin_unlock(&lowmem_shrink_lock);
		return 0;
	}

	selected_oom_score_adj = min_score_adj;

	/* More debug log */
	if (output_expect(enable_candidate_log)) {
		if (min_score_adj <= lowmem_debug_adj) {
			if (time_after_eq(jiffies, lowmem_print_extra_info_timeout)) {
				lowmem_print_extra_info_timeout = jiffies + HZ * 2;
				print_extra_info = 1;
			}
		}

		if (print_extra_info) {
			lowmem_print(1, "Free memory other_free: %d, other_file:%d pages\n", other_free, other_file);
#ifdef CONFIG_MTK_ENG_BUILD
			log_offset = snprintf(lmk_log_buf, LMK_LOG_BUF_SIZE, "%s",
					      "<lmk>  pid  score_adj     rss   rswap name\n");
#else
			lowmem_print(1,
				     "<lmk>  pid  score_adj     rss   rswap name\n");
#endif
		}
	}

	rcu_read_lock();
	for_each_process(tsk) {
		struct task_struct *p;
		short oom_score_adj;

        // prize zengke 20180620 add for ddr test,ignore memtest thread-----begin
        if (strstr("memtester", tsk->comm) || strstr("pri.factorytest", tsk->comm))
                continue;
        // prize zengke 20180620 add for ddr test,ignore memtest thread-----end

		if (tsk->flags & PF_KTHREAD)
			continue;

		if (task_lmk_waiting(tsk) &&
		    time_before_eq(jiffies, lowmem_deathpending_timeout)) {
#ifdef CONFIG_MTK_ENG_BUILD
			static pid_t last_dying_pid;

			if (last_dying_pid != tsk->pid) {
				lowmem_print(1, "lowmem_shrink return directly, due to  %d (%s) is dying\n",
					     tsk->pid, tsk->comm);
				last_dying_pid = tsk->pid;
			}
#endif
			rcu_read_unlock();
			spin_unlock(&lowmem_shrink_lock);
			return SHRINK_STOP;
		}

		p = find_lock_task_mm(tsk);
		if (!p)
			continue;

#ifdef CONFIG_MTK_AEE_FEATURE
		if (p->signal->flags & SIGNAL_GROUP_COREDUMP) {
			task_unlock(p);
			continue;
		}
#endif

		/* Bypass process which has been selected */
		if (task_lmk_waiting(p)) {
#ifdef CONFIG_MTK_ENG_BUILD
			pr_info_ratelimited("%d (%s) is dying, find next candidate\n",
					    p->pid, p->comm);
#endif
			if (p->state == TASK_RUNNING)
				p_state_is_found |= LOWMEM_P_STATE_R;
			else
				p_state_is_found |= LOWMEM_P_STATE_OTHER;

			task_unlock(p);
			continue;
		}

		/* Bypass D-state process */
		if (p->state & TASK_UNINTERRUPTIBLE) {
			lowmem_print(2, "lowmem_scan filter D state process: %d (%s) state:0x%lx\n",
				     p->pid, p->comm, p->state);
			task_unlock(p);
			p_state_is_found |= LOWMEM_P_STATE_D;
			continue;
		}

		oom_score_adj = p->signal->oom_score_adj;

		if (output_expect(enable_candidate_log)) {
			if (print_extra_info) {
#ifdef CONFIG_MTK_ENG_BUILD
log_again:
				log_ret = snprintf(lmk_log_buf + log_offset, LMK_LOG_BUF_SIZE - log_offset,
						   "<lmk>%5d%11d%8lu%8lu %s\n", p->pid,
						   oom_score_adj, get_mm_rss(p->mm),
						   get_mm_counter(p->mm, MM_SWAPENTS), p->comm);

				if ((log_offset + log_ret) >= LMK_LOG_BUF_SIZE || log_ret < 0) {
					*(lmk_log_buf + log_offset) = '\0';
					lowmem_print(1, "\n%s", lmk_log_buf);
					log_offset = 0;
					memset(lmk_log_buf, 0x0, LMK_LOG_BUF_SIZE);
					goto log_again;
				} else {
					log_offset += log_ret;
				}
#else
				lowmem_print(1,	"<lmk>%5d%11d%8lu%8lu %s\n", p->pid,
					     oom_score_adj, get_mm_rss(p->mm),
					     get_mm_counter(p->mm, MM_SWAPENTS), p->comm);
#endif
			}
		}

#ifdef CONFIG_MTK_ENG_BUILD
		tasksize = get_mm_rss(p->mm) + get_mm_counter(p->mm, MM_SWAPENTS);

		/*
		* dump memory info when framework low memory:
		* record the first two pid which consumed most memory.
		*/
		if (tasksize > max_mem) {
			max_mem = tasksize;
			pid_dump = p->pid;
		}

		if (p->pid == pid_flm_warn &&
		    time_before_eq(jiffies, flm_warn_timeout)) {
			task_unlock(p);
			continue;
		}
#endif
		if (oom_score_adj < min_score_adj) {
			task_unlock(p);
			continue;
		}

#ifndef CONFIG_MTK_ENG_BUILD
		tasksize = get_mm_rss(p->mm) + get_mm_counter(p->mm, MM_SWAPENTS);
#endif
		task_unlock(p);
		if (tasksize <= 0)
			continue;
		if (selected) {
			if (oom_score_adj < selected_oom_score_adj)
				continue;
			if (oom_score_adj == selected_oom_score_adj &&
			    tasksize <= selected_tasksize)
				continue;
		}
		selected = p;
		selected_tasksize = tasksize;
		selected_oom_score_adj = oom_score_adj;
		lowmem_print(2, "select '%s' (%d), adj %hd, size %d, to kill\n",
			     p->comm, p->pid, oom_score_adj, tasksize);
	}

#ifdef CONFIG_MTK_ENG_BUILD
	if (log_offset > 0)
		lowmem_print(1, "\n%s", lmk_log_buf);
#endif

	if (selected) {
		long cache_size = other_file * (long)(PAGE_SIZE / 1024);
		long cache_limit = minfree * (long)(PAGE_SIZE / 1024);
		long free = other_free * (long)(PAGE_SIZE / 1024);

		task_lock(selected);
		send_sig(SIGKILL, selected, 0);
		if (selected->mm)
			task_set_lmk_waiting(selected);
		task_unlock(selected);
		trace_lowmemory_kill(selected, cache_size, cache_limit, free);
		lowmem_print(1, "Killing '%s' (%d), adj %hd, state(%ld)\n"
				"   to free %ldkB on behalf of '%s' (%d) because\n"
				"   cache %ldkB is below limit %ldkB for oom_score_adj %hd\n"
				"   Free memory is %ldkB above reserved\n"
#ifdef CONFIG_SWAP
				"   (decrease %d level, amr_adj %hd)\n"
#endif
				,
			     selected->comm, selected->pid,
			     selected_oom_score_adj, selected->state,
			     selected_tasksize * (long)(PAGE_SIZE / 1024),
			     current->comm, current->pid,
			     cache_size, cache_limit,
			     min_score_adj,
			     free
#ifdef CONFIG_SWAP
			     , to_be_aggressive, amr_adj
#endif
			     );

		lowmem_deathpending_timeout = jiffies + LOWMEM_DEATHPENDING_TIMEOUT;

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(CONFIG_MTK_ENG_BUILD)
		/*
		* when kill adj=0 process trigger kernel warning, only in MTK internal eng load
		*/
		if ((selected_oom_score_adj <= lowmem_kernel_warn_adj) && /*lowmem_kernel_warn_adj=16 for test*/
			time_after_eq(jiffies, flm_warn_timeout)) {
			if (pid_dump != pid_flm_warn) {
				#define MSG_SIZE_TO_AEE 70
				char msg_to_aee[MSG_SIZE_TO_AEE];

				lowmem_print(1, "low memory trigger kernel warning\n");
				snprintf(msg_to_aee, MSG_SIZE_TO_AEE,
					 "please contact AP/AF memory module owner[pid:%d]\n", pid_dump);
				aee_kernel_warning_api("LMK", 0, DB_OPT_DEFAULT |
					DB_OPT_DUMPSYS_ACTIVITY |
					DB_OPT_LOW_MEMORY_KILLER |
					DB_OPT_PID_MEMORY_INFO | /* smaps and hprof*/
					DB_OPT_PROCESS_COREDUMP |
					DB_OPT_DUMPSYS_SURFACEFLINGER |
					DB_OPT_DUMPSYS_GFXINFO |
					DB_OPT_DUMPSYS_PROCSTATS,
					"Framework low memory\nCRDISPATCH_KEY:FLM_APAF", msg_to_aee);

				if (pid_dump == selected->pid) {/*select 1st time, filter it*/
					pid_flm_warn = pid_dump;
					flm_warn_timeout = jiffies + 60 * HZ;
					lowmem_print(1, "'%s' (%d) max RSS, not kill\n",
						     selected->comm, selected->pid);
					send_sig(SIGSTOP, selected, 0);
					rcu_read_unlock();
					spin_unlock(&lowmem_shrink_lock);
					dump_memory_status();
					return rem;
				}
			} else {
				lowmem_print(1, "pid_flm_warn:%d, select '%s' (%d)\n",
					     pid_flm_warn, selected->comm, selected->pid);
				pid_flm_warn = -1; /*reset*/
			}
		}
#endif

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(CONFIG_MTK_ENG_BUILD)
		/* Show an indication if low memory */
		if (!in_lowmem && selected_oom_score_adj <= lowmem_debug_adj) {
			in_lowmem = 1;
			lowmem_print(1, "LowMemoryOn\n");
		}
#endif
		rem += selected_tasksize;
	} else {
		if (p_state_is_found & LOWMEM_P_STATE_D)
			lowmem_print(2, "No selected (full of D-state processes at %d)\n", (int)min_score_adj);
		if (p_state_is_found & LOWMEM_P_STATE_R)
			lowmem_print(2, "No selected (full of R-state processes at %d)\n", (int)min_score_adj);
		if (p_state_is_found & LOWMEM_P_STATE_OTHER)
			lowmem_print(2, "No selected (full of OTHER-state processes at %d)\n", (int)min_score_adj);
	}

	lowmem_print(4, "lowmem_scan %lu, %x, return %lu\n",
		     sc->nr_to_scan, sc->gfp_mask, rem);
	rcu_read_unlock();
	spin_unlock(&lowmem_shrink_lock);

	/* Dump HW memory outside the lock */
	if (selected && output_expect(enable_candidate_log))
		if (print_extra_info)
			dump_memory_status();

#ifdef MTK_LMK_USER_EVENT
	/* Send uevent if needed */
	if (mtklmk_initialized && current_is_kswapd() && mtklmk_uevent_timeout)
		mtklmk_uevent(min_score_adj, minfree);
#endif

	return rem;

#undef LOWMEM_P_STATE_D
#undef LOWMEM_P_STATE_R
#undef LOWMEM_P_STATE_OTHER
}

static struct shrinker lowmem_shrinker = {
	.scan_objects = lowmem_scan,
	.count_objects = lowmem_count,
	.seeks = DEFAULT_SEEKS * 16
};

static int __init lowmem_init(void)
{
#if defined(CONFIG_ZRAM) && defined(CONFIG_MTK_GMO_RAM_OPTIMIZE)
	vm_swappiness = 100;
#endif
	register_shrinker(&lowmem_shrinker);

#ifdef MTK_LMK_USER_EVENT
	/* initialize work for uevent */
	INIT_WORK(&mtklmk_work, mtklmk_async_uevent);

	/* register as misc device */
	if (!misc_register(&mtklmk_misc)) {
		pr_info("%s: successful to register misc device!\n", __func__);
		mtklmk_initialized = 1;
	}
#endif

	return 0;
}
device_initcall(lowmem_init);

#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
static short lowmem_oom_adj_to_oom_score_adj(short oom_adj)
{
	if (oom_adj == OOM_ADJUST_MAX)
		return OOM_SCORE_ADJ_MAX;
	else
		return (oom_adj * OOM_SCORE_ADJ_MAX) / -OOM_DISABLE;
}

static void lowmem_autodetect_oom_adj_values(void)
{
	int i;
	short oom_adj;
	short oom_score_adj;
	int array_size = ARRAY_SIZE(lowmem_adj);

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;

	if (array_size <= 0)
		return;

	oom_adj = lowmem_adj[array_size - 1];
	if (oom_adj > OOM_ADJUST_MAX)
		return;

	oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
	if (oom_score_adj <= OOM_ADJUST_MAX)
		return;

	lowmem_print(1, "lowmem_shrink: convert oom_adj to oom_score_adj:\n");
	for (i = 0; i < array_size; i++) {
		oom_adj = lowmem_adj[i];
		oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
		lowmem_adj[i] = oom_score_adj;
		lowmem_print(1, "oom_adj %d => oom_score_adj %d\n",
			     oom_adj, oom_score_adj);
	}
}

static int lowmem_adj_array_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	ret = param_array_ops.set(val, kp);

	/* HACK: Autodetect oom_adj values in lowmem_adj array */
	lowmem_autodetect_oom_adj_values();

	return ret;
}

static int lowmem_adj_array_get(char *buffer, const struct kernel_param *kp)
{
	return param_array_ops.get(buffer, kp);
}

static void lowmem_adj_array_free(void *arg)
{
	param_array_ops.free(arg);
}

static struct kernel_param_ops lowmem_adj_array_ops = {
	.set = lowmem_adj_array_set,
	.get = lowmem_adj_array_get,
	.free = lowmem_adj_array_free,
};

static const struct kparam_array __param_arr_adj = {
	.max = ARRAY_SIZE(lowmem_adj),
	.num = &lowmem_adj_size,
	.ops = &param_ops_short,
	.elemsize = sizeof(lowmem_adj[0]),
	.elem = lowmem_adj,
};
#endif

/*
 * get_min_free_pages
 * returns the low memory killer watermark of the given pid,
 * When the system free memory is lower than the watermark, the LMK (low memory
 * killer) may try to kill processes.
 */
int get_min_free_pages(pid_t pid)
{
	struct task_struct *p;
	int target_oom_adj = 0;
	int i = 0;
	int array_size = ARRAY_SIZE(lowmem_adj);

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;
	if (lowmem_minfree_size < array_size)
		array_size = lowmem_minfree_size;

	for_each_process(p) {
		/* search pid */
		if (p->pid == pid) {
			task_lock(p);
			target_oom_adj = p->signal->oom_score_adj;
			task_unlock(p);
			/* get min_free value of the pid */
			for (i = array_size - 1; i >= 0; i--) {
				if (target_oom_adj >= lowmem_adj[i]) {
					pr_debug("pid: %d, target_oom_adj = %d, lowmem_adj[%d] = %d, lowmem_minfree[%d] = %d\n",
						 pid, target_oom_adj, i, lowmem_adj[i], i, lowmem_minfree[i]);
					return lowmem_minfree[i];
				}
			}
			goto out;
		}
	}

out:
	lowmem_print(3, "[%s]pid: %d, adj: %d, lowmem_minfree = 0\n",
		     __func__, pid, p->signal->oom_score_adj);
	return 0;
}
EXPORT_SYMBOL(get_min_free_pages);

/* Query LMK minfree settings */
/* To query default value, you can input index with value -1. */
size_t query_lmk_minfree(int index)
{
	int which;

	/* Invalid input index, return default value */
	if (index < 0)
		return lowmem_minfree[2];

	/* Find a corresponding output */
	which = 5;
	do {
		if (lowmem_adj[which] <= index)
			break;
	} while (--which >= 0);

	/* Fix underflow bug */
	which = (which < 0) ? 0 : which;

	return lowmem_minfree[which];
}
EXPORT_SYMBOL(query_lmk_minfree);

/*
 * not really modular, but the easiest way to keep compat with existing
 * bootargs behaviour is to continue using module_param here.
 */
module_param_named(cost, lowmem_shrinker.seeks, int, S_IRUGO | S_IWUSR);
#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
module_param_cb(adj, &lowmem_adj_array_ops,
		.arr = &__param_arr_adj,
		S_IRUGO | S_IWUSR);
__MODULE_PARM_TYPE(adj, "array of short");
#else
module_param_array_named(adj, lowmem_adj, short, &lowmem_adj_size,
			 S_IRUGO | S_IWUSR);
#endif
module_param_array_named(minfree, lowmem_minfree, uint, &lowmem_minfree_size,
			 S_IRUGO | S_IWUSR);
module_param_named(debug_level, lowmem_debug_level, uint, S_IRUGO | S_IWUSR);
module_param_named(debug_adj, lowmem_debug_adj, short, S_IRUGO | S_IWUSR);
module_param_named(candidate_log, enable_candidate_log, uint, S_IRUGO | S_IWUSR);

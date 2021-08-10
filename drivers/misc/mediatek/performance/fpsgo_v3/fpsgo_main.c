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
#include <linux/workqueue.h>
#include <linux/unistd.h>
#include <linux/module.h>
#include <linux/sched.h>

#include "fpsgo_common.h"
#include "fpsgo_base.h"
#include "fpsgo_usedext.h"
#include "fbt_cpu.h"
#include "fstb.h"
#include "fps_composer.h"
#include "xgf.h"
#include "eara_job.h"

#define CREATE_TRACE_POINTS
#include <trace/events/fpsgo.h>

#define TARGET_UNLIMITED_FPS 60

enum FPSGO_NOTIFIER_PUSH_TYPE {
	FPSGO_NOTIFIER_SWITCH_FPSGO			= 0x00,
	FPSGO_NOTIFIER_QUEUE_DEQUEUE		= 0x01,
	FPSGO_NOTIFIER_CONNECT				= 0x02,
	FPSGO_NOTIFIER_DFRC_FPS				= 0x03,
	FPSGO_NOTIFIER_BQID				= 0x04,
	FPSGO_NOTIFIER_NN_JOB_BEGIN			= 0x05,
	FPSGO_NOTIFIER_NN_JOB_END			= 0x06,
	FPSGO_NOTIFIER_GPU_BLOCK			= 0x07,
	FPSGO_NOTIFIER_VSYNC				= 0x08,
};

/* TODO: use union*/
struct FPSGO_NOTIFIER_PUSH_TAG {
	enum FPSGO_NOTIFIER_PUSH_TYPE ePushType;

	int pid;
	unsigned long long cur_ts;

	int enable;

	int qudeq_cmd;
	unsigned int queue_arg;

	unsigned long long bufID;
	int connectedAPI;
	int queue_SF;
	unsigned long long identifier;
	int create;

	int dfrc_fps;

	int nn_pid;
	int nn_tid;
	unsigned long long nn_mid;
	int num_step;
	__s32 *device;
	__s32 *boost;
	__u64 *exec_time;


	int tid;
	int start;

	struct work_struct sWork;
};

static struct mutex notify_lock;
struct workqueue_struct *g_psNotifyWorkQueue;
static int fpsgo_enable;
static int fpsgo_force_onoff;
static int gpu_boost_enable_perf;
static int gpu_boost_enable_camera;

void (*rsu_cpufreq_notifier_fp)(int cluster_id, unsigned long freq);

/* TODO: event register & dispatch */
int fpsgo_is_enable(void)
{
	int enable;

	mutex_lock(&notify_lock);
	enable = fpsgo_enable;
	mutex_unlock(&notify_lock);

	FPSGO_LOGI("[FPSGO_CTRL] isenable %d\n", enable);
	return enable;
}

static void fpsgo_notifier_wq_cb_vsync(unsigned long long ts)
{
	FPSGO_LOGI("[FPSGO_CB] vsync: %llu\n", ts);

	if (!fpsgo_is_enable())
		return;

	fpsgo_ctrl2fbt_vsync(ts);
}

static void fpsgo_notifier_wq_cb_dfrc_fps(int dfrc_fps)
{
	FPSGO_LOGI("[FPSGO_CB] dfrc_fps %d\n", dfrc_fps);

	if (!fpsgo_is_enable())
		return;

	fpsgo_ctrl2fbt_dfrc_fps(dfrc_fps);
}

static void fpsgo_notifier_wq_cb_connect(int pid,
		int connectedAPI, unsigned long long id)
{
	FPSGO_LOGI(
		"[FPSGO_CB] connect: pid %d, API %d, id %llu\n",
		pid, connectedAPI, id);

	if (connectedAPI == WINDOW_DISCONNECT)
		fpsgo_ctrl2comp_disconnect_api(pid, connectedAPI, id);
	else
		fpsgo_ctrl2comp_connect_api(pid, connectedAPI, id);
}

static void fpsgo_notifier_wq_cb_nn_job_begin(unsigned int tid,
	unsigned long long mid)
{
	FPSGO_LOGI(
		"[FPSGO_CB] nn_job_begin: tid %d, mid %llu\n",
		tid, mid);

	if (!fpsgo_is_enable())
		return;

	fpsgo_ctrl2xgf_nn_job_begin(tid, mid);
}

static void fpsgo_notifier_wq_cb_nn_job_end(int pid, int tid,
	unsigned long long mid, int num_step,
	__s32 *boost, __s32 *device, __u64 *exec_time)
{
	int hw_type = BACKGROUND;

	FPSGO_LOGI(
		"[FPSGO_CB] nn_job_begin: tid %d, mid %llu\n",
		tid, mid);

	if (!fpsgo_is_enable())
		return;

	hw_type = fpsgo_ctrl2xgf_nn_job_end(tid, mid);

	if (boost && device && exec_time)
		fpsgo_ctrl2eara_nn_job_collect(pid, tid, mid,
			hw_type, num_step, boost, device, exec_time);
}

static void fpsgo_notifier_wq_cb_bqid(int pid, unsigned long long bufID,
	int queue_SF, unsigned long long id, int create)
{
	FPSGO_LOGI(
		"[FPSGO_CB] bqid: pid %d, bufID %llu, queue_SF %d, id %llu, create %d\n",
		pid, bufID, queue_SF, id, create);

	fpsgo_ctrl2comp_bqid(pid, bufID, queue_SF, id, create);
}

static void fpsgo_notifier_wq_cb_gblock(int tid, int start)
{
	FPSGO_LOGI(
		"[FPSGO_CB] gblock: tid %d, start %d\n",
		tid, start);

	if (!fpsgo_is_enable())
		return;

	fpsgo_ctrl2fstb_gblock(tid, start);
}

static void fpsgo_notifier_wq_cb_qudeq(int qudeq,
		unsigned int startend, int cur_pid,
		unsigned long long curr_ts, unsigned long long id)
{
	FPSGO_LOGI("[FPSGO_CB] qudeq: %d-%d, pid %d, ts %llu, id %llu\n",
		qudeq, startend, cur_pid, curr_ts, id);

	if (!fpsgo_is_enable())
		return;

	switch (qudeq) {
	case 1:
		if (startend) {
			FPSGO_LOGI("[FPSGO_CB] QUEUE Start: pid %d\n",
					cur_pid);
			fpsgo_ctrl2comp_enqueue_start(cur_pid,
					curr_ts, id);
		} else {
			FPSGO_LOGI("[FPSGO_CB] QUEUE End: pid %d\n",
					cur_pid);
			fpsgo_ctrl2comp_enqueue_end(cur_pid, curr_ts,
					id);
		}
		break;
	case 0:
		if (startend) {
			FPSGO_LOGI("[FPSGO_CB] DEQUEUE Start: pid %d\n",
					cur_pid);
			fpsgo_ctrl2comp_dequeue_start(cur_pid,
					curr_ts, id);
		} else {
			FPSGO_LOGI("[FPSGO_CB] DEQUEUE End: pid %d\n",
					cur_pid);
			fpsgo_ctrl2comp_dequeue_end(cur_pid,
					curr_ts, id);
		}
		break;
	default:
		break;
	}
}

static void fpsgo_notifier_wq_cb_enable(int enable)
{
	FPSGO_LOGI(
	"[FPSGO_CB] enable %d, fpsgo_enable %d, force_onoff %d\n",
	enable, fpsgo_enable, fpsgo_force_onoff);

	mutex_lock(&notify_lock);
	if (enable == fpsgo_enable) {
		mutex_unlock(&notify_lock);
		return;
	}

	if (fpsgo_force_onoff != FPSGO_FREE &&
			enable != fpsgo_force_onoff) {
		mutex_unlock(&notify_lock);
		return;
	}

	fpsgo_ctrl2fbt_switch_fbt(enable);
	fpsgo_ctrl2fstb_switch_fstb(enable);
	fpsgo_ctrl2xgf_switch_xgf(enable);

	fpsgo_enable = enable;

	if (!fpsgo_enable)
		fpsgo_clear();

	FPSGO_LOGI("[FPSGO_CB] fpsgo_enable %d\n",
			fpsgo_enable);
	mutex_unlock(&notify_lock);
}

static void fpsgo_notifier_wq_cb(struct work_struct *psWork)
{
	struct FPSGO_NOTIFIER_PUSH_TAG *vpPush =
		FPSGO_CONTAINER_OF(psWork,
				struct FPSGO_NOTIFIER_PUSH_TAG, sWork);

	if (!vpPush) {
		FPSGO_LOGE("[FPSGO_CTRL] ERROR\n");
		return;
	}

	FPSGO_LOGI("[FPSGO_CTRL] push type = %d\n",
			vpPush->ePushType);

	switch (vpPush->ePushType) {
	case FPSGO_NOTIFIER_SWITCH_FPSGO:
		fpsgo_notifier_wq_cb_enable(vpPush->enable);
		break;
	case FPSGO_NOTIFIER_QUEUE_DEQUEUE:
		fpsgo_notifier_wq_cb_qudeq(vpPush->qudeq_cmd,
				vpPush->queue_arg, vpPush->pid,
				vpPush->cur_ts, vpPush->identifier);
		break;
	case FPSGO_NOTIFIER_CONNECT:
		fpsgo_notifier_wq_cb_connect(vpPush->pid,
				vpPush->connectedAPI, vpPush->identifier);
		break;
	case FPSGO_NOTIFIER_DFRC_FPS:
		fpsgo_notifier_wq_cb_dfrc_fps(vpPush->dfrc_fps);
		break;
	case FPSGO_NOTIFIER_BQID:
		fpsgo_notifier_wq_cb_bqid(vpPush->pid, vpPush->bufID,
			vpPush->queue_SF, vpPush->identifier, vpPush->create);
		break;
	case FPSGO_NOTIFIER_GPU_BLOCK:
		fpsgo_notifier_wq_cb_gblock(vpPush->tid, vpPush->start);
		break;
	case FPSGO_NOTIFIER_NN_JOB_BEGIN:
		fpsgo_notifier_wq_cb_nn_job_begin(vpPush->nn_tid,
			vpPush->nn_mid);
		break;
	case FPSGO_NOTIFIER_NN_JOB_END:
		fpsgo_notifier_wq_cb_nn_job_end(vpPush->nn_pid, vpPush->nn_tid,
			vpPush->nn_mid, vpPush->num_step, vpPush->boost,
			vpPush->device, vpPush->exec_time);
		break;
	case FPSGO_NOTIFIER_VSYNC:
		fpsgo_notifier_wq_cb_vsync(vpPush->cur_ts);
		break;
	default:
		FPSGO_LOGE("[FPSGO_CTRL] unhandled push type = %d\n",
				vpPush->ePushType);
		break;
	}

	fpsgo_free(vpPush, sizeof(struct FPSGO_NOTIFIER_PUSH_TAG));
}

void fpsgo_notify_qudeq(int qudeq,
		unsigned int startend,
		int pid, unsigned long long id)
{
	unsigned long long cur_ts;
	struct FPSGO_NOTIFIER_PUSH_TAG *vpPush;

	FPSGO_LOGI("[FPSGO_CTRL] qudeq %d-%d, id %llu pid %d\n",
		qudeq, startend, id, pid);

	if (!fpsgo_is_enable())
		return;

	vpPush =
		(struct FPSGO_NOTIFIER_PUSH_TAG *)
		fpsgo_alloc_atomic(sizeof(struct FPSGO_NOTIFIER_PUSH_TAG));
	if (!vpPush) {
		FPSGO_LOGE("[FPSGO_CTRL] OOM\n");
		return;
	}

	if (!g_psNotifyWorkQueue) {
		FPSGO_LOGE("[FPSGO_CTRL] NULL WorkQueue\n");
		fpsgo_free(vpPush, sizeof(struct FPSGO_NOTIFIER_PUSH_TAG));
		return;
	}

	cur_ts = fpsgo_get_time();

	vpPush->ePushType = FPSGO_NOTIFIER_QUEUE_DEQUEUE;
	vpPush->pid = pid;
	vpPush->cur_ts = cur_ts;
	vpPush->qudeq_cmd = qudeq;
	vpPush->queue_arg = startend;
	vpPush->identifier = id;

	INIT_WORK(&vpPush->sWork, fpsgo_notifier_wq_cb);
	queue_work(g_psNotifyWorkQueue, &vpPush->sWork);
}
void fpsgo_notify_nn_job_begin(unsigned int tid, unsigned long long mid)
{
	struct FPSGO_NOTIFIER_PUSH_TAG *vpPush;

	FPSGO_LOGI(
		"[FPSGO_CTRL] nn_job_begin: tid %d, mid %llu\n",
		tid, mid);

	if (!fpsgo_is_enable())
		return;

	vpPush =
		(struct FPSGO_NOTIFIER_PUSH_TAG *)
		fpsgo_alloc_atomic(sizeof(struct FPSGO_NOTIFIER_PUSH_TAG));

	if (!vpPush) {
		FPSGO_LOGE("[FPSGO_CTRL] OOM\n");
		return;
	}

	if (!g_psNotifyWorkQueue) {
		FPSGO_LOGE("[FPSGO_CTRL] NULL WorkQueue\n");
		fpsgo_free(vpPush, sizeof(struct FPSGO_NOTIFIER_PUSH_TAG));
		return;
	}

	vpPush->ePushType = FPSGO_NOTIFIER_NN_JOB_BEGIN;
	vpPush->nn_tid = tid;
	vpPush->nn_mid = mid;

#if 1
	INIT_WORK(&vpPush->sWork, fpsgo_notifier_wq_cb);
	queue_work(g_psNotifyWorkQueue, &vpPush->sWork);
#endif
}

int fpsgo_get_nn_priority(unsigned int pid, unsigned long long mid)
{
	return fpsgo_ctrl2eara_get_nn_priority(pid, mid);
}

void fpsgo_get_nn_ttime(unsigned int pid, unsigned long long mid,
	int num_step, __u64 *time)
{
	fpsgo_ctrl2eara_get_nn_ttime(pid, mid, num_step, time);
}

void fpsgo_notify_nn_job_end(int pid, int tid, unsigned long long mid,
	int num_step, __s32 *boost, __s32 *device, __u64 *exec_time)
{
	struct FPSGO_NOTIFIER_PUSH_TAG *vpPush;
	int size;

	if (!fpsgo_is_enable())
		return;

	vpPush =
		(struct FPSGO_NOTIFIER_PUSH_TAG *)
		fpsgo_alloc_atomic(sizeof(struct FPSGO_NOTIFIER_PUSH_TAG));

	if (!vpPush) {
		FPSGO_LOGE("[FPSGO_CTRL] OOM\n");
		return;
	}

	if (!g_psNotifyWorkQueue) {
		FPSGO_LOGE("[FPSGO_CTRL] NULL WorkQueue\n");
		fpsgo_free(vpPush, sizeof(struct FPSGO_NOTIFIER_PUSH_TAG));
		return;
	}

	size = num_step * MAX_DEVICE;
	vpPush->ePushType = FPSGO_NOTIFIER_NN_JOB_END;
	vpPush->nn_pid = pid;
	vpPush->nn_tid = tid;
	vpPush->nn_mid = mid;
	vpPush->num_step = num_step;
	vpPush->boost = boost;
	vpPush->device = device;
	vpPush->exec_time = exec_time;

	INIT_WORK(&vpPush->sWork, fpsgo_notifier_wq_cb);
	queue_work(g_psNotifyWorkQueue, &vpPush->sWork);
}


void fpsgo_notify_connect(int pid,
		int connectedAPI, unsigned long long id)
{
	struct FPSGO_NOTIFIER_PUSH_TAG *vpPush;

	FPSGO_LOGI(
		"[FPSGO_CTRL] connect pid %d, id %llu, API %d\n",
		pid, id, connectedAPI);

	vpPush =
		(struct FPSGO_NOTIFIER_PUSH_TAG *)
		fpsgo_alloc_atomic(sizeof(struct FPSGO_NOTIFIER_PUSH_TAG));

	if (!vpPush) {
		FPSGO_LOGE("[FPSGO_CTRL] OOM\n");
		return;
	}

	if (!g_psNotifyWorkQueue) {
		FPSGO_LOGE("[FPSGO_CTRL] NULL WorkQueue\n");
		fpsgo_free(vpPush, sizeof(struct FPSGO_NOTIFIER_PUSH_TAG));
		return;
	}

	vpPush->ePushType = FPSGO_NOTIFIER_CONNECT;
	vpPush->pid = pid;
	vpPush->connectedAPI = connectedAPI;
	vpPush->identifier = id;

	INIT_WORK(&vpPush->sWork, fpsgo_notifier_wq_cb);
	queue_work(g_psNotifyWorkQueue, &vpPush->sWork);
}

void fpsgo_notify_bqid(int pid, unsigned long long bufID,
	int queue_SF, unsigned long long id, int create)
{
	struct FPSGO_NOTIFIER_PUSH_TAG *vpPush;

	FPSGO_LOGI("[FPSGO_CTRL] bqid pid %d, buf %llu, queue_SF %d, id %llu\n",
		pid, bufID, queue_SF, id);

	vpPush = (struct FPSGO_NOTIFIER_PUSH_TAG *)
		fpsgo_alloc_atomic(sizeof(struct FPSGO_NOTIFIER_PUSH_TAG));

	if (!vpPush) {
		FPSGO_LOGE("[FPSGO_CTRL] OOM\n");
		return;
	}

	if (!g_psNotifyWorkQueue) {
		FPSGO_LOGE("[FPSGO_CTRL] NULL WorkQueue\n");
		fpsgo_free(vpPush, sizeof(struct FPSGO_NOTIFIER_PUSH_TAG));
		return;
	}

	vpPush->ePushType = FPSGO_NOTIFIER_BQID;
	vpPush->pid = pid;
	vpPush->bufID = bufID;
	vpPush->queue_SF = queue_SF;
	vpPush->identifier = id;
	vpPush->create = create;

	INIT_WORK(&vpPush->sWork, fpsgo_notifier_wq_cb);
	queue_work(g_psNotifyWorkQueue, &vpPush->sWork);
}

int fpsgo_is_gpu_block_boost_enable(void)
{
	int enable;

	mutex_lock(&notify_lock);
	enable = max(gpu_boost_enable_camera,
		gpu_boost_enable_perf);
	mutex_unlock(&notify_lock);

	return enable;
}

int fpsgo_is_gpu_block_boost_perf_enable(void)
{
	int enable;

	mutex_lock(&notify_lock);
	enable = gpu_boost_enable_perf;
	mutex_unlock(&notify_lock);

	return enable;
}

int fpsgo_is_gpu_block_boost_camera_enable(void)
{
	int enable;

	mutex_lock(&notify_lock);
	enable = gpu_boost_enable_camera;
	mutex_unlock(&notify_lock);

	return enable;
}

void fpsgo_gpu_block_boost_enable_perf(int enable)
{
	mutex_lock(&notify_lock);
	gpu_boost_enable_perf = enable;
	mutex_unlock(&notify_lock);
}

void fpsgo_gpu_block_boost_enable_camera(int enable)
{
	mutex_lock(&notify_lock);
	gpu_boost_enable_camera = enable;
	mutex_unlock(&notify_lock);
}


int fpsgo_notify_gpu_block(int tid, unsigned long long mid, int start)
{
	struct FPSGO_NOTIFIER_PUSH_TAG *vpPush;
	int g_enable;

	FPSGO_LOGI("[FPSGO_CTRL] gblock pid %d, start %d\n",
		tid, start);

	g_enable = fpsgo_is_gpu_block_boost_enable();
	if (g_enable < 0 || g_enable > 100)
		return -1;

	vpPush = (struct FPSGO_NOTIFIER_PUSH_TAG *)
		fpsgo_alloc_atomic(sizeof(struct FPSGO_NOTIFIER_PUSH_TAG));

	if (!vpPush) {
		FPSGO_LOGE("[FPSGO_CTRL] OOM\n");
		return -1;
	}

	if (!g_psNotifyWorkQueue) {
		FPSGO_LOGE("[FPSGO_CTRL] NULL WorkQueue\n");
		fpsgo_free(vpPush, sizeof(struct FPSGO_NOTIFIER_PUSH_TAG));
		return -1;
	}

	vpPush->ePushType = FPSGO_NOTIFIER_GPU_BLOCK;
	vpPush->tid = tid;
	vpPush->start = start;

	INIT_WORK(&vpPush->sWork, fpsgo_notifier_wq_cb);
	queue_work(g_psNotifyWorkQueue, &vpPush->sWork);
	return g_enable;
}

void fpsgo_notify_vsync(void)
{
	struct FPSGO_NOTIFIER_PUSH_TAG *vpPush;

	FPSGO_LOGI("[FPSGO_CTRL] vsync\n");

	if (!fpsgo_is_enable())
		return;

	vpPush = (struct FPSGO_NOTIFIER_PUSH_TAG *)
		fpsgo_alloc_atomic(sizeof(struct FPSGO_NOTIFIER_PUSH_TAG));

	if (!vpPush) {
		FPSGO_LOGE("[FPSGO_CTRL] OOM\n");
		return;
	}

	if (!g_psNotifyWorkQueue) {
		FPSGO_LOGE("[FPSGO_CTRL] NULL WorkQueue\n");
		fpsgo_free(vpPush, sizeof(struct FPSGO_NOTIFIER_PUSH_TAG));
		return;
	}

	vpPush->ePushType = FPSGO_NOTIFIER_VSYNC;
	vpPush->cur_ts = fpsgo_get_time();

	INIT_WORK(&vpPush->sWork, fpsgo_notifier_wq_cb);
	queue_work(g_psNotifyWorkQueue, &vpPush->sWork);
}


void fpsgo_notify_cpufreq(int cid, unsigned long freq)
{
	FPSGO_LOGI("[FPSGO_CTRL] cid %d, cpufreq %lu\n", cid, freq);

	if (rsu_cpufreq_notifier_fp)
		rsu_cpufreq_notifier_fp(cid, freq);

	if (!fpsgo_enable)
		return;

	fpsgo_ctrl2fbt_cpufreq_cb(cid, freq);
}


/* FPSGO control */
void fpsgo_switch_enable(int enable)
{
	struct FPSGO_NOTIFIER_PUSH_TAG *vpPush = NULL;

	if (!g_psNotifyWorkQueue) {
		FPSGO_LOGE("[FPSGO_CTRL] NULL WorkQueue\n");
		return;
	}

	FPSGO_LOGI("[FPSGO_CTRL] switch enable %d\n", enable);

	if (fpsgo_is_force_enable() !=
			FPSGO_FREE && enable !=
			fpsgo_is_force_enable())
		return;

	vpPush =
		(struct FPSGO_NOTIFIER_PUSH_TAG *)
		fpsgo_alloc_atomic(sizeof(struct FPSGO_NOTIFIER_PUSH_TAG));

	if (!vpPush) {
		FPSGO_LOGE("[FPSGO_CTRL] OOM\n");
		return;
	}

	vpPush->ePushType = FPSGO_NOTIFIER_SWITCH_FPSGO;
	vpPush->enable = enable;

	INIT_WORK(&vpPush->sWork, fpsgo_notifier_wq_cb);
	queue_work(g_psNotifyWorkQueue, &vpPush->sWork);
}

int fpsgo_is_force_enable(void)
{
	int temp_onoff;

	mutex_lock(&notify_lock);
	temp_onoff = fpsgo_force_onoff;
	mutex_unlock(&notify_lock);

	return temp_onoff;
}

void fpsgo_force_switch_enable(int enable)
{
	mutex_lock(&notify_lock);
	fpsgo_force_onoff = enable;
	mutex_unlock(&notify_lock);

	fpsgo_switch_enable(enable?1:0);
}

/* FSTB control */
int fpsgo_is_fstb_enable(void)
{
	return is_fstb_enable();
}

int fpsgo_switch_fstb(int enable)
{
	return fpsgo_ctrl2fstb_switch_fstb(enable);
}

int fpsgo_fstb_sample_window(long long time_usec)
{
	return switch_sample_window(time_usec);
}

int fpsgo_fstb_fps_range(int nr_level,
		struct fps_level *level)
{
	return switch_fps_range(nr_level, level);
}

int fpsgo_fstb_process_fps_range(char *proc_name,
	int nr_level, struct fps_level *level)
{
	return switch_process_fps_range(proc_name, nr_level, level);
}

int fpsgo_fstb_thread_fps_range(pid_t pid,
	int nr_level, struct fps_level *level)
{
	return switch_thread_fps_range(pid, nr_level, level);
}

int fpsgo_fstb_dfps_ceiling(int fps)
{
	return switch_dfps_ceiling(fps);
}

int fpsgo_fstb_fps_error_threhosld(int threshold)
{
	return switch_fps_error_threhosld(threshold);
}

int fpsgo_fstb_percentile_frametime(int ratio)
{
	return switch_percentile_frametime(ratio);
}

static void __exit fpsgo_exit(void)
{
	fpsgo_notifier_wq_cb_enable(0);

	if (g_psNotifyWorkQueue) {
		flush_workqueue(g_psNotifyWorkQueue);
		destroy_workqueue(g_psNotifyWorkQueue);
		g_psNotifyWorkQueue = NULL;
	}

	fbt_cpu_exit();
	mtk_fstb_exit();
	fpsgo_composer_exit();
}

static int __init fpsgo_init(void)
{
	FPSGO_LOGI("[FPSGO_CTRL] init\n");

	g_psNotifyWorkQueue =
		create_singlethread_workqueue("fpsgo_notifier_wq");

	if (g_psNotifyWorkQueue == NULL)
		return -EFAULT;

	mutex_init(&notify_lock);

	fpsgo_force_onoff = FPSGO_FREE;
	gpu_boost_enable_perf = gpu_boost_enable_camera = -1;

	init_fpsgo_common();
	fbt_cpu_init();
	mtk_fstb_init();
	fpsgo_composer_init();

	fpsgo_switch_enable(1);

	cpufreq_notifier_fp = fpsgo_notify_cpufreq;

	fpsgo_notify_vsync_fp = fpsgo_notify_vsync;

	fpsgo_notify_qudeq_fp = fpsgo_notify_qudeq;
	fpsgo_notify_connect_fp = fpsgo_notify_connect;
	fpsgo_notify_bqid_fp = fpsgo_notify_bqid;

	fpsgo_notify_nn_job_begin_fp = fpsgo_notify_nn_job_begin;
	fpsgo_notify_nn_job_end_fp = fpsgo_notify_nn_job_end;
	fpsgo_get_nn_priority_fp = fpsgo_get_nn_priority;
	fpsgo_get_nn_ttime_fp = fpsgo_get_nn_ttime;

	return 0;
}

module_init(fpsgo_init);
module_exit(fpsgo_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek FPSGO");
MODULE_AUTHOR("MediaTek Inc.");

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
 */
#include "perf_ioctl.h"

#if defined(CONFIG_MTK_FPSGO) || defined(CONFIG_MTK_FPSGO_V3)
#include "tchbst.h"
#include "io_ctrl.h"
#endif

#if defined(LEGACY_TOUCH_BOOST_SUPPORT)
#include "touch_boost.h"
#endif

#include <mt-plat/mtk_perfobserver.h>

#define TAG "PERF_IOCTL"

void (*fpsgo_notify_qudeq_fp)(int qudeq,
		unsigned int startend,
		int pid, unsigned long long identifier);
void (*fpsgo_notify_connect_fp)(int pid,
		int connectedAPI, unsigned long long identifier);
void (*fpsgo_notify_bqid_fp)(int pid, unsigned long long bufID,
		int queue_SF, unsigned long long identifier, int create);
void (*fpsgo_notify_vsync_fp)(void);
void (*fpsgo_notify_nn_job_begin_fp)(unsigned int tid, unsigned long long mid);
void (*fpsgo_notify_nn_job_end_fp)(int pid, int tid, unsigned long long mid,
	int num_step, __s32 *boost, __s32 *device, __u64 *exec_time);
int (*fpsgo_get_nn_priority_fp)(unsigned int pid, unsigned long long mid);
void (*fpsgo_get_nn_ttime_fp)(unsigned int pid, unsigned long long mid,
	int num_step, __u64 *ttime);

void (*rsu_getusage_fp)(__s32 *devusage, __u32 *bwusage, __u32 pid);

static unsigned long perfctl_copy_from_user(void *pvTo,
		const void __user *pvFrom, unsigned long ulBytes)
{
	if (access_ok(VERIFY_READ, pvFrom, ulBytes))
		return __copy_from_user(pvTo, pvFrom, ulBytes);

	return ulBytes;
}

static unsigned long perfctl_copy_to_user(void __user *pvTo,
		const void *pvFrom, unsigned long ulBytes)
{
	if (access_ok(VERIFY_WRITE, pvTo, ulBytes))
		return __copy_to_user(pvTo, pvFrom, ulBytes);

	return ulBytes;
}

static void perfctl_notify_fpsgo_nn_begin(
	struct _EARA_NN_PACKAGE *msgKM,
	struct _EARA_NN_PACKAGE *msgUM)
{
	int arr_length;
	__u64 *__target_time = NULL;

	struct pob_nn_model_info pnmi = {msgKM->pid, msgKM->tid,
						msgKM->mid};
	pob_nn_update(POB_NN_BEGIN, &pnmi);

	if (!fpsgo_notify_nn_job_begin_fp || !fpsgo_get_nn_ttime_fp)
		return;

	fpsgo_notify_nn_job_begin_fp(msgKM->tid, msgKM->mid);

	arr_length = msgKM->num_step * MAX_DEVICE;
	__target_time =
		kmalloc(arr_length * sizeof(__u64), GFP_KERNEL);

	if (!__target_time)
		return;

	fpsgo_get_nn_ttime_fp(msgKM->pid,
		msgKM->mid, msgKM->num_step, __target_time);

	if (msgKM->target_time)
		perfctl_copy_to_user(msgKM->target_time,
			__target_time, arr_length * sizeof(__u64));

	kfree(__target_time);
}

static void perfctl_notify_fpsgo_nn_end(
	struct _EARA_NN_PACKAGE *msgKM,
	struct _EARA_NN_PACKAGE *msgUM)
{
	__s32 *boost, *device;
	__u64 *exec_time;
	int size;

	struct pob_nn_model_info pnmi = {msgKM->pid, msgKM->tid, msgKM->mid};

	pob_nn_update(POB_NN_END, &pnmi);

	if (!fpsgo_notify_nn_job_end_fp || !fpsgo_get_nn_priority_fp)
		return;

	size = msgKM->num_step * MAX_DEVICE;

	if (!msgKM->boost || !msgKM->device || !msgKM->exec_time)
		goto out_um_malloc_fail;

	boost = kmalloc_array(size, sizeof(__s32), GFP_KERNEL);
	if (!boost)
		goto out_boost_malloc_fail;
	device = kmalloc_array(size, sizeof(__s32), GFP_KERNEL);
	if (!device)
		goto out_device_malloc_fail;
	exec_time = kmalloc_array(size, sizeof(__u64), GFP_KERNEL);
	if (!exec_time)
		goto out_exec_time_malloc_fail;

	perfctl_copy_from_user(boost,
		msgKM->boost, size * sizeof(__s32));
	perfctl_copy_from_user(device,
		msgKM->device, size * sizeof(__s32));
	perfctl_copy_from_user(exec_time,
		msgKM->exec_time, size * sizeof(__u64));

	fpsgo_notify_nn_job_end_fp(msgKM->pid, msgKM->tid, msgKM->mid,
			msgKM->num_step, boost, device, exec_time);

	msgKM->priority = fpsgo_get_nn_priority_fp(msgKM->pid, msgKM->mid);

	perfctl_copy_to_user(msgUM, msgKM, sizeof(struct _EARA_NN_PACKAGE));
	return;

out_exec_time_malloc_fail:
	kfree(device);
out_device_malloc_fail:
	kfree(boost);
out_boost_malloc_fail:
out_um_malloc_fail:
	fpsgo_notify_nn_job_end_fp(msgKM->pid, msgKM->tid, msgKM->mid,
			msgKM->num_step, NULL, NULL, NULL);
}


/*--------------------DEV OP------------------------*/
static int eara_show(struct seq_file *m, void *v)
{
	return 0;
}

static int eara_open(struct inode *inode, struct file *file)
{
	return single_open(file, eara_show, inode->i_private);
}

static long eara_ioctl_impl(struct file *filp,
		unsigned int cmd, unsigned long arg, void *pKM)
{
	ssize_t ret = 0;
	struct _EARA_NN_PACKAGE *msgKM = NULL,
		*msgUM = (struct _EARA_NN_PACKAGE *)arg;
	struct _EARA_NN_PACKAGE smsgKM;

	msgKM = (struct _EARA_NN_PACKAGE *)pKM;
	if (!msgKM) {
		msgKM = &smsgKM;
		if (perfctl_copy_from_user(msgKM, msgUM,
				sizeof(struct _EARA_NN_PACKAGE))) {
			ret = -EFAULT;
			goto ret_ioctl;
		}
	}

	switch (cmd) {
	case EARA_NN_BEGIN:
		perfctl_notify_fpsgo_nn_begin(msgKM, msgUM);
		break;
	case EARA_NN_END:
		perfctl_notify_fpsgo_nn_end(msgKM, msgUM);
		break;
	case EARA_GETUSAGE:
		msgKM->bw_usage = 0;

		if (rsu_getusage_fp)
			rsu_getusage_fp(&msgKM->dev_usage, &msgKM->bw_usage,
					msgKM->pid);
		else
			msgKM->dev_usage = 0;

		perfctl_copy_to_user(msgUM, msgKM,
				sizeof(struct _EARA_NN_PACKAGE));

		break;
	default:
		pr_debug(TAG "%s %d: unknown cmd %x\n",
			__FILE__, __LINE__, cmd);
		ret = -1;
		goto ret_ioctl;
	}

ret_ioctl:
	return ret;
}

static long eara_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	return eara_ioctl_impl(filp, cmd, arg, NULL);
}

static long eara_compat_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	int ret = -EFAULT;
	struct _EARA_NN_PACKAGE_32 {
		__u32 pid;
		__u32 tid;
		__u64 mid;
		__s32 errorno;
		__s32 priority;
		__s32 num_step;

		__s32 dev_usage;
		__u32 bw_usage;

		union {
			__u32 device;
			__u64 p_dummy_device;
		};
		union {
			__u32 boost;
			__u64 p_dummy_boost;
		};
		union {
			__u32 exec_time;
			__u64 p_dummy_exec_time;
		};
		union {
			__u32 target_time;
			__u64 p_dummy_target_time;
		};
	};
	struct _EARA_NN_PACKAGE sEaraPackageKM64;
	struct _EARA_NN_PACKAGE_32 sEaraPackageKM32;
	struct _EARA_NN_PACKAGE_32 *psEaraPackageKM32 = &sEaraPackageKM32;
	struct _EARA_NN_PACKAGE_32 *psEaraPackageUM32 =
		(struct _EARA_NN_PACKAGE_32 *)arg;

	if (perfctl_copy_from_user(psEaraPackageKM32,
			psEaraPackageUM32, sizeof(struct _EARA_NN_PACKAGE_32)))
		goto unlock_and_return;

	sEaraPackageKM64 = *((struct _EARA_NN_PACKAGE *)psEaraPackageKM32);
	sEaraPackageKM64.device =
		(void *)((size_t) psEaraPackageKM32->device);
	sEaraPackageKM64.boost =
		(void *)((size_t) psEaraPackageKM32->boost);
	sEaraPackageKM64.exec_time =
		(void *)((size_t) psEaraPackageKM32->exec_time);
	sEaraPackageKM64.target_time =
		(void *)((size_t) psEaraPackageKM32->target_time);

	ret = eara_ioctl_impl(filp, cmd, arg, &sEaraPackageKM64);

unlock_and_return:
	return ret;
}

static const struct file_operations eara_Fops = {
	.unlocked_ioctl = eara_ioctl,
	.compat_ioctl = eara_compat_ioctl,
	.open = eara_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*--------------------PERFMGR OP------------------------*/
static int dev_perfmgr_show(struct seq_file *m, void *v)
{
	return 0;
}

static int dev_perfmgr_open(struct inode *inode, struct file *file)
{
	return single_open(file, dev_perfmgr_show, inode->i_private);
}

static long dev_perfmgr_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	ssize_t ret = 0;
	struct _PERFMGR_PACKAGE *msgKM = NULL,
		*msgUM = (struct _PERFMGR_PACKAGE *)arg;
	struct _PERFMGR_PACKAGE smsgKM;

	msgKM = &smsgKM;

	if (perfctl_copy_from_user(msgKM, msgUM,
		sizeof(struct _PERFMGR_PACKAGE))) {
		ret = -EFAULT;
		goto ret_ioctl;
	}

	switch (cmd) {
	case PERFMGR_CPU_PREFER:
		pr_debug(TAG " sched_set_cpuprefer:%d, %d\n",
			msgKM->tid, msgKM->prefer_type);
		sched_set_cpuprefer(msgKM->tid, msgKM->prefer_type);
		break;

	default:
		pr_debug(TAG "%s %d: unknown cmd %x\n",
			__FILE__, __LINE__, cmd);
		ret = -1;
		goto ret_ioctl;
	}

ret_ioctl:
	return ret;
}

static const struct file_operations perfmgr_Fops = {
	.unlocked_ioctl = dev_perfmgr_ioctl,
	.compat_ioctl = dev_perfmgr_ioctl,
	.open = dev_perfmgr_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


/*--------------------INIT------------------------*/

static int device_show(struct seq_file *m, void *v)
{
	return 0;
}

static int device_open(struct inode *inode, struct file *file)
{
	return single_open(file, device_show, inode->i_private);
}

static long device_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	ssize_t ret = 0;
	struct _FPSGO_PACKAGE *msgKM = NULL,
			*msgUM = (struct _FPSGO_PACKAGE *)arg;
	struct _FPSGO_PACKAGE smsgKM;

	msgKM = &smsgKM;

	if (perfctl_copy_from_user(msgKM, msgUM,
				sizeof(struct _FPSGO_PACKAGE))) {
		ret = -EFAULT;
		goto ret_ioctl;
	}

	switch (cmd) {
#if defined(CONFIG_MTK_FPSGO_V3)
	case FPSGO_QUEUE:
		if (fpsgo_notify_qudeq_fp)
			fpsgo_notify_qudeq_fp(1,
					msgKM->start, msgKM->tid,
					msgKM->identifier);
		break;
	case FPSGO_DEQUEUE:
		if (fpsgo_notify_qudeq_fp)
			fpsgo_notify_qudeq_fp(0,
					msgKM->start, msgKM->tid,
					msgKM->identifier);
		break;
	case FPSGO_QUEUE_CONNECT:
		if (fpsgo_notify_connect_fp)
			fpsgo_notify_connect_fp(msgKM->tid,
					msgKM->connectedAPI, msgKM->identifier);
		break;
	case FPSGO_BQID:
		if (fpsgo_notify_bqid_fp)
			fpsgo_notify_bqid_fp(msgKM->tid, msgKM->bufID,
				msgKM->queue_SF, msgKM->identifier,
				msgKM->start);
		break;
	case FPSGO_TOUCH:
		usrtch_ioctl(cmd, msgKM->frame_time);
		break;
	case FPSGO_VSYNC:
		if (fpsgo_notify_vsync_fp)
			fpsgo_notify_vsync_fp();
		break;
		break;

#else
	case FPSGO_TOUCH:
	case FPSGO_QUEUE:
#if defined(LEGACY_TOUCH_BOOST_SUPPORT)
		touch_boost_ioctl(cmd, msgKM->frame_time);
#endif /* LEGACY_TOUCH_BOOST_SUPPORT */
		break;
	case FPSGO_DEQUEUE:
		/* FALLTHROUGH */
	case FPSGO_QUEUE_CONNECT:
		/* FALLTHROUGH */
	case FPSGO_VSYNC:
		/* FALLTHROUGH */
	case FPSGO_BQID:
		break;
#endif

	default:
		pr_debug(TAG "%s %d: unknown cmd %x\n",
			__FILE__, __LINE__, cmd);
		ret = -1;
		goto ret_ioctl;
	}

ret_ioctl:
	return ret;
}

static const struct file_operations Fops = {
	.unlocked_ioctl = device_ioctl,
	.compat_ioctl = device_ioctl,
	.open = device_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*--------------------INIT------------------------*/
int init_perfctl(struct proc_dir_entry *parent)
{
	struct proc_dir_entry *pe;
	int ret_val = 0;


	pr_debug(TAG"Start to init perf_ioctl driver\n");

	pe = proc_create("perf_ioctl", 0664, parent, &Fops);
	if (!pe) {
		pr_debug(TAG"%s failed with %d\n",
				"Creating file node ",
				ret_val);
		ret_val = -ENOMEM;
		goto out_wq;
	}

	pe = proc_create("eara_ioctl", 0664, parent, &eara_Fops);
	if (!pe) {
		pr_debug(TAG"%s failed with %d\n",
				"Creating file node ",
				ret_val);
		ret_val = -ENOMEM;
		goto out_wq;
	}

	pe = proc_create("perfmgr_ioctl", 0664, parent, &perfmgr_Fops);
	if (!pe) {
		pr_debug(TAG"%s failed with %d\n",
				"Creating file node ",
				ret_val);
		ret_val = -ENOMEM;
		goto out_wq;
	}

#if defined(LEGACY_TOUCH_BOOST_SUPPORT)
	init_touch_boost();
#endif /* LEGACY_TOUCH_BOOST_SUPPORT */

	pr_debug(TAG"init perf_ioctl driver done\n");

	return 0;

out_wq:
	return ret_val;
}


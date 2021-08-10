/*
 * Copyright (C) 2016 MediaTek Inc.
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

#include <linux/types.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>

#include <linux/slab.h>

#include <linux/delay.h>
#include <linux/uaccess.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>

#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

#include <m4u.h>

#include <ion.h>
#include <mtk/ion_drv.h>
#include <mtk/mtk_ion.h>

#include "vpu_drv.h"
#include "vpu_cmn.h"
#include "vpu_dbg.h"

#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

#define VPU_DEV_NAME            "vpu"

static struct vpu_device *vpu_device;
static struct wakeup_source vpu_wake_lock;
static struct list_head device_debug_list;
static struct mutex debug_list_mutex;
struct ion_client *my_ion_client;
unsigned int efuse_data;

static int vpu_probe(struct platform_device *dev);

static int vpu_remove(struct platform_device *dev);

static int vpu_suspend(struct platform_device *dev, pm_message_t mesg);

static int vpu_resume(struct platform_device *dev);

/*---------------------------------------------------------------------------*/
/* VPU Driver: pm operations                                                 */
/*---------------------------------------------------------------------------*/
#ifdef CONFIG_PM
int vpu_pm_suspend(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);

	WARN_ON(pdev == NULL);

	return vpu_suspend(pdev, PMSG_SUSPEND);
}

int vpu_pm_resume(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);

	WARN_ON(pdev == NULL);

	return vpu_resume(pdev);
}

int vpu_pm_restore_noirq(struct device *device)
{
	return 0;
}
#else
#define vpu_pm_suspend NULL
#define vpu_pm_resume  NULL
#define vpu_pm_restore_noirq NULL
#endif

static const struct dev_pm_ops vpu_pm_ops = {
	.suspend = vpu_pm_suspend,
	.resume = vpu_pm_resume,
	.freeze = vpu_pm_suspend,
	.thaw = vpu_pm_resume,
	.poweroff = vpu_pm_suspend,
	.restore = vpu_pm_resume,
	.restore_noirq = vpu_pm_restore_noirq,
};


/*---------------------------------------------------------------------------*/
/* VPU Driver: Prototype                                                     */
/*---------------------------------------------------------------------------*/

static const struct of_device_id vpu_of_ids[] = {
	{.compatible = "mediatek,vpu_core0",},
	{.compatible = "mediatek,vpu_core1",},
	{.compatible = "mediatek,vpu_core2",},
	{}
};

static struct platform_driver vpu_driver = {
	.probe   = vpu_probe,
	.remove  = vpu_remove,
	.suspend = vpu_suspend,
	.resume  = vpu_resume,
	.driver  = {
		.name = VPU_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = vpu_of_ids,
#ifdef CONFIG_PM
		.pm = &vpu_pm_ops,
#endif
	}
};


/*---------------------------------------------------------------------------*/
/* VPU Driver: file operations                                               */
/*---------------------------------------------------------------------------*/
static int vpu_open(struct inode *inode, struct file *flip);

static int vpu_release(struct inode *inode, struct file *flip);

static int vpu_mmap(struct file *flip, struct vm_area_struct *vma);

#ifdef CONFIG_COMPAT
static long vpu_compat_ioctl(struct file *flip, unsigned int cmd,
	unsigned long arg);
#endif

static long vpu_ioctl(struct file *flip, unsigned int cmd, unsigned long arg);

static const struct file_operations vpu_fops = {
	.owner = THIS_MODULE,
	.open = vpu_open,
	.release = vpu_release,
	.mmap = vpu_mmap,
#ifdef CONFIG_COMPAT
	.compat_ioctl = vpu_compat_ioctl,
#endif
	.unlocked_ioctl = vpu_ioctl
};


/*---------------------------------------------------------------------------*/
/* M4U: fault callback                                                       */
/*---------------------------------------------------------------------------*/
enum m4u_callback_ret_t vpu_m4u_fault_callback(int port,
	unsigned int mva, void *data)
{
	LOG_DBG("[m4u] fault callback: port=%d, mva=0x%x", port, mva);
	return M4U_CALLBACK_HANDLED;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
static int vpu_num_users;

int vpu_create_user(struct vpu_user **user)
{
	struct vpu_user *u;
	int i = 0;

	u = kzalloc(sizeof(vlist_type(struct vpu_user)), GFP_KERNEL);
	if (!u)
		return -ENOMEM;

	mutex_init(&u->data_mutex);
	mutex_lock(&debug_list_mutex);
	vpu_num_users++;
	mutex_unlock(&debug_list_mutex);
	u->id = NULL;
	u->open_pid = current->pid;
	u->open_tgid = current->tgid;
	INIT_LIST_HEAD(&u->enque_list);
	INIT_LIST_HEAD(&u->deque_list);
	init_waitqueue_head(&u->deque_wait);
	init_waitqueue_head(&u->delete_wait);

	for (i = 0 ; i < MTK_VPU_CORE ; i++)
		u->running[i] = false;

	u->deleting = false;
	u->power_mode = VPU_POWER_MODE_DYNAMIC;
	u->power_opp = VPU_POWER_OPP_UNREQUEST;

	mutex_lock(&vpu_device->user_mutex);
	list_add_tail(vlist_link(u, struct vpu_user), &vpu_device->user_list);
	mutex_unlock(&vpu_device->user_mutex);

	*user = u;
	return 0;
}

static int vpu_write_register(struct vpu_reg_values *regs)
{
	return 0;
}

struct ion_handle *vpu_hw_ion_import_handle(struct ion_client *client, int fd)
{
	struct ion_handle *handle = NULL;

	if (!client) {
		LOG_WRN("[vpu] invalid ion client!\n");
		return handle;
	}
	if (fd == -1) {
		LOG_WRN("[vpu] invalid ion fd!\n");
		return handle;
	}
	LOG_DBG("[vpu] ion_import_handle +\n");
	handle = ion_import_dma_buf_fd(client, fd);

	if (IS_ERR(handle)) {
		LOG_WRN("[vpu] import ion handle failed!\n");
		return NULL;
	}

	if (g_vpu_log_level > Log_STATE_MACHINE)
		LOG_INF("[vpu] ion_import_handle(0x%p)\n", handle);

	return handle;
}

int vpu_push_request_to_queue(struct vpu_user *user, struct vpu_request *req)
{
	if (!user) {
		LOG_ERR("empty user\n");
		return -EINVAL;
	}

	if (user->deleting) {
		LOG_WRN("push a request while deleting the user\n");
		return -ENONET;
	}

	mutex_lock(&user->data_mutex);
	list_add_tail(vlist_link(req, struct vpu_request), &user->enque_list);
	mutex_unlock(&user->data_mutex);

	wake_up(&vpu_device->req_wait);

	return 0;
}

int vpu_put_request_to_pool(struct vpu_user *user, struct vpu_request *req)
{
	int i = 0, request_core_index = -1;
	int j = 0, cnt = 0;
	struct ion_handle *handle = NULL;

	if (!user) {
		LOG_ERR("empty user\n");
		return -EINVAL;
	}

	if (user->deleting) {
		LOG_WRN("push a request while deleting the user\n");
		return -ENONET;
	}
	if (!my_ion_client)
		my_ion_client = ion_client_create(g_ion_device, "vpu_drv");
	for (i = 0 ; i < req->buffer_count; i++) {
		for (j = 0 ; j < req->buffers[i].plane_count; j++) {
			handle = NULL;

			LOG_DBG("[vpu] (%d) FD.0x%lx\n", cnt,
			  (unsigned long)(uintptr_t)(req->buf_ion_infos[cnt]));

			handle = ion_import_dma_buf_fd(my_ion_client,
						req->buf_ion_infos[cnt]);
			if (IS_ERR(handle)) {
				LOG_WRN("[vpu_drv] %s=0x%p failed!\n",
					"import ion handle", handle);
			} else {
				if (g_vpu_log_level > Log_STATE_MACHINE)
					LOG_INF("[vpu_drv]cnt_%d,%s=0x%p\n",
						cnt,
						"ion_import_dma_buf handle",
						handle);
				/* import fd to handle for buffer ref count+1*/
				req->buf_ion_infos[cnt] =
						(uint64_t)(uintptr_t)handle;
			}
			cnt++;
		}
	}

	/* CHRISTODO, specific vpu */
	for (i = 0 ; i < MTK_VPU_CORE ; i++) {
		/*LOG_DBG("debug i(%d), (0x1 << i) (0x%x)", i, (0x1 << i));*/
		if (req->requested_core == (0x1 << i)) {
			request_core_index = i;
			if (!vpu_device->vpu_hw_support[request_core_index]) {
				LOG_ERR("[vpu_%d] not support. %s\n",
					request_core_index,
					"push to common queue");
				request_core_index = -1;
			}
			break;
		}
	}

	#if 0
	LOG_DBG("[vpu] push request to euque CORE_IDNEX (0x%x/0x%x)...\n",
		req->requested_core, request_core_index);
	#endif

	if (request_core_index >= MTK_VPU_CORE) {
		LOG_ERR("wrong core index (0x%x/%d/%d)",
				req->requested_core,
				request_core_index,
				MTK_VPU_CORE);
	}

	if (request_core_index > -1 && request_core_index < MTK_VPU_CORE) {
		/* LOG_DBG("[vpu] push self pool, index(%d/0x%x)\n",
		 * request_core_index, req->requested_core);
		 */
		mutex_lock(&vpu_device->servicepool_mutex[request_core_index]);

		list_add_tail(vlist_link(req, struct vpu_request),
			&vpu_device->pool_list[request_core_index]);

		vpu_device->servicepool_list_size[request_core_index] += 1;

		mutex_unlock(
			&vpu_device->servicepool_mutex[request_core_index]);
	} else {
		/* LOG_DBG("[vpu] push common pool, index(%d,0x%x)\n",
		 * request_core_index, req->requested_core);
		 */
		mutex_lock(&vpu_device->commonpool_mutex);

		list_add_tail(vlist_link(req, struct vpu_request),
				&vpu_device->cmnpool_list);

		vpu_device->commonpool_list_size += 1;
		/* LOG_DBG("[vpu] push common pool size (%d)\n",
		 * vpu_device->commonpool_list_size);
		 */
		mutex_unlock(&vpu_device->commonpool_mutex);
	}

	wake_up(&vpu_device->req_wait);
	/*LOG_DBG("[vpu] vpu_push_request_to_queue ---\n");*/

	return 0;
}

bool vpu_user_is_running(struct vpu_user *user)
{
	bool running = false;
	int i = 0;

	mutex_lock(&user->data_mutex);
	for (i = 0 ; i < MTK_VPU_CORE ; i++) {
		if (user->running[i]) {
			running = true;
			break;
		}
	}
	mutex_unlock(&user->data_mutex);

	return running;
}

int vpu_flush_requests_from_queue(struct vpu_user *user)
{
#if 0
	struct list_head *head, *temp;
	struct vpu_request *req;

	mutex_lock(&user->data_mutex);

	if (!user->running && list_empty(&user->enque_list)) {
		mutex_unlock(&user->data_mutex);
		return 0;
	}

	user->flushing = true;
	mutex_unlock(&user->data_mutex);

	/* the running request will add to the deque before interrupt */
	wait_event_interruptible(user->deque_wait, !user->running);

	while (user->running)
		ndelay(1000);

	mutex_lock(&user->data_mutex);
	/* push the remaining enque to the deque */
	list_for_each_safe(head, temp, &user->enque_list) {
		req = vlist_node_of(head, struct vpu_request);
		req->status = VPU_REQ_STATUS_FLUSH;
		list_del_init(head);
		list_add_tail(head, &user->deque_list);
	}

	user->flushing = false;
	LOG_DBG("flushed queue, user:%d\n", user->id);

	mutex_unlock(&user->data_mutex);
#endif
	return 0;
}

int vpu_pop_request_from_queue(struct vpu_user *user,
	struct vpu_request **rreq)
{
	int ret;
	struct vpu_request *req;

	/* wait until condition is true */
	ret = wait_event_interruptible(
		user->deque_wait,
		!list_empty(&user->deque_list));

	/* ret == -ERESTARTSYS, if signal interrupt */
	if (ret < 0) {
		LOG_ERR("interrupt by signal, %s, ret=%d\n",
			"while pop a request", ret);
		*rreq = NULL;
		return -EINTR;
	}

	mutex_lock(&user->data_mutex);
	/* This part should not be happened */
	if (list_empty(&user->deque_list)) {
		mutex_unlock(&user->data_mutex);
		LOG_ERR("pop a request from empty queue! ret=%d\n", ret);
		*rreq = NULL;
		return -ENODATA;
	};

	/* get first node from deque list */
	req = vlist_node_of(user->deque_list.next, struct vpu_request);

	list_del_init(vlist_link(req, struct vpu_request));
	mutex_unlock(&user->data_mutex);

	*rreq = req;
	return 0;
}

int vpu_get_request_from_queue(struct vpu_user *user,
	uint64_t request_id, struct vpu_request **rreq)
{
	int ret;
	struct list_head *head = NULL;
	struct vpu_request *req;
	bool get = false;
	int retry = 0;

	do {
		/* wait until condition is true */
		ret = wait_event_interruptible(
			user->deque_wait,
			!list_empty(&user->deque_list));

		/* ret == -ERESTARTSYS, if signal interrupt */
		if (ret < 0) {
			LOG_ERR("interrupt by signal, %s, ret=%d\n",
				"while pop a request", ret);
			if (retry < 5) {
				LOG_ERR("retry=%d\n", retry);
				retry += 1;
				get = false;
				continue;
			} else {
				LOG_ERR("retry %d times fail, return FAIL\n",
						retry);
				*rreq = NULL;
				return -EINTR;
			}
		}

		mutex_lock(&user->data_mutex);
		/* This part should not be happened */
		if (list_empty(&user->deque_list)) {
			mutex_unlock(&user->data_mutex);
			LOG_ERR("pop a request from empty queue! ret=%d\n",
					ret);
			*rreq = NULL;
			return -ENODATA;
		};

		/* get corresponding node from deque list */
		list_for_each(head, &user->deque_list)
		{
			req = vlist_node_of(head, struct vpu_request);
			LOG_DBG("[vpu] req->request_id = 0x%lx, 0x%lx\n",
					(unsigned long)req->request_id,
					(unsigned long)request_id);
			if ((unsigned long)req->request_id ==
					(unsigned long)request_id) {
				get = true;
				LOG_DBG("[vpu] get = true\n");
				break;
			}
		}

		if (g_vpu_log_level > VpuLogThre_PERFORMANCE)
			LOG_INF("[vpu] %s (%d)\n", __func__, get);
		if (get)
			list_del_init(vlist_link(req, struct vpu_request));

		mutex_unlock(&user->data_mutex);
	} while (!get);

	*rreq = req;
	return 0;
}

int vpu_get_core_status(struct vpu_status *status)
{
	int index = status->vpu_core_index; /* - 1;*/

	if (index > -1 && index < MTK_VPU_CORE) {
		LOG_DBG("vpu_%d, support(%d/0x%x)\n",
			index, vpu_device->vpu_hw_support[index], efuse_data);
		if (vpu_device->vpu_hw_support[index]) {
			mutex_lock(&vpu_device->servicepool_mutex[index]);

			status->vpu_core_available =
				vpu_device->service_core_available[index];

			status->pool_list_size =
				vpu_device->servicepool_list_size[index];
			mutex_unlock(&vpu_device->servicepool_mutex[index]);
		} else {
			LOG_ERR("core_%d not support (0x%x).\n",
					index, efuse_data);
			return -EINVAL;
		}
	} else {
		mutex_lock(&vpu_device->commonpool_mutex);
		status->vpu_core_available = true;
		status->pool_list_size = vpu_device->commonpool_list_size;
		mutex_unlock(&vpu_device->commonpool_mutex);
	}

	LOG_DBG("[vpu]%s idx(%d), available(%d), size(%d)\n", __func__,
			status->vpu_core_index,
			status->vpu_core_available,
			status->pool_list_size);

	return 0;
}

int vpu_delete_user(struct vpu_user *user)
{
	struct list_head *head, *temp;
	struct vpu_request *req;
	int ret = 0;

	if (!user) {
		LOG_ERR("delete empty user!\n");
		return -EINVAL;
	}
	mutex_lock(&user->data_mutex);
	user->deleting = true;
	mutex_unlock(&user->data_mutex);

	/*vpu_flush_requests_from_queue(user);*/

	ret = wait_event_interruptible(
			user->delete_wait,
			!vpu_user_is_running(user));
	if (ret < 0) {
		LOG_WRN("[vpu]%s, ret=%d, wait delete user again\n",
			"interrupt by signal", ret);
		wait_event_interruptible(user->delete_wait,
			!vpu_user_is_running(user));
	}

	/* clear the list of deque */
	mutex_lock(&user->data_mutex);
	list_for_each_safe(head, temp, &user->deque_list) {
		req = vlist_node_of(head, struct vpu_request);
		list_del(head);
		vpu_free_request(req);
	}
	mutex_unlock(&user->data_mutex);

	/* confirm the lock has released */
	if (user->locked)
		vpu_hw_unlock(user);

	mutex_lock(&vpu_device->user_mutex);
	LOG_INF("deleted user[0x%lx]\n", (unsigned long)(user->id));
	list_del(vlist_link(user, struct vpu_user));
	mutex_unlock(&vpu_device->user_mutex);

	kfree(user);

	return 0;
}

int vpu_dump_user(struct seq_file *s)
{
	struct vpu_user *user;
	struct list_head *head_user;
	struct list_head *head_req;
	uint32_t cnt_deq;

#define LINE_BAR "  +------------------+------+------+-------+-------+\n"
	vpu_print_seq(s, LINE_BAR);
	vpu_print_seq(s, "  |%-18s|%-6s|%-6s|%-7s|%-7s|\n",
			"Id", "Pid", "Tid", "Deque", "Locked");
	vpu_print_seq(s, LINE_BAR);

	mutex_lock(&vpu_device->user_mutex);
	list_for_each(head_user, &vpu_device->user_list)
	{
		user = vlist_node_of(head_user, struct vpu_user);
		cnt_deq = 0;

		list_for_each(head_req, &user->deque_list)
		{
			cnt_deq++;
		}

		vpu_print_seq(s, "  |0x%-16lx|%-6d|%-6d|%-7d|%7d|\n",
			      (unsigned long)(user->id),
			      user->open_pid,
			      user->open_tgid,
			      cnt_deq,
			      user->locked);
		vpu_print_seq(s, LINE_BAR);
	}
	mutex_unlock(&vpu_device->user_mutex);
	vpu_print_seq(s, "\n");

#undef LINE_BAR

	return 0;
}


int vpu_alloc_debug_info(struct vpu_dev_debug_info **rdbginfo)
{
	struct vpu_dev_debug_info *dbginfo;

	dbginfo = kzalloc(sizeof(vlist_type(struct vpu_dev_debug_info)),
				GFP_KERNEL);
	if (dbginfo == NULL) {
		LOG_ERR("%s, node=0x%p\n", __func__, dbginfo);
		return -ENOMEM;
	}

	*rdbginfo = dbginfo;

	return 0;
}

int vpu_free_debug_info(struct vpu_dev_debug_info *dbginfo)
{
	if (dbginfo != NULL)
		kfree(dbginfo);
	return 0;
}

int vpu_dump_device_dbg(struct seq_file *s)
{
	struct list_head *head = NULL;
	struct vpu_dev_debug_info *dbg_info;

#define LINE_BAR "  +-------+-------+-------+------------------------------+\n"

	vpu_print_seq(s, "========== vpu device debug info dump ==========\n");
	vpu_print_seq(s, LINE_BAR);
	vpu_print_seq(s, "  |%-7s|%-7s|%-7s|%-32s|\n",
				  "PID", "TGID", "OPENFD", "USER");
	vpu_print_seq(s, LINE_BAR);

	mutex_lock(&debug_list_mutex);
	list_for_each(head, &device_debug_list)
	{
		dbg_info = vlist_node_of(head, struct vpu_dev_debug_info);
		vpu_print_seq(s, "  |%-7d|%-7d|%-7d|%-32s|\n",
				  dbg_info->open_pid,
				  dbg_info->open_tgid,
				  dbg_info->dev_fd,
				  dbg_info->callername);
	}
	mutex_unlock(&debug_list_mutex);
	vpu_print_seq(s, LINE_BAR);
#undef LINE_BAR
	return 0;
}


/*---------------------------------------------------------------------------*/
/* IOCTL: implementation                                                     */
/*---------------------------------------------------------------------------*/

static int vpu_open(struct inode *inode, struct file *flip)
{
	int ret = 0, i = 0;
	bool not_support_vpu = true;
	struct vpu_user *user;

	for (i = 0 ; i < MTK_VPU_CORE ; i++) {
		if (vpu_device->vpu_hw_support[i]) {
			not_support_vpu = false;
			break;
		}
	}
	if (not_support_vpu) {
		LOG_ERR("not support vpu...(%d/0x%x)\n",
				not_support_vpu, efuse_data);
		return -ENODEV;
	}

	LOG_INF("vpu_support core : 0x%x\n", efuse_data);

	vpu_create_user(&user);
	if (IS_ERR_OR_NULL(user)) {
		LOG_ERR("fail to create user\n");
		return -ENOMEM;
	}

	user->id = (unsigned long *)user;
	LOG_INF("%s cnt(%d) user->id : 0x%lx, tids(%d/%d)\n", __func__,
		vpu_num_users,
		(unsigned long)(user->id),
		user->open_pid, user->open_tgid);
	flip->private_data = user;

	return ret;
}

#ifdef CONFIG_COMPAT
static long vpu_compat_ioctl(struct file *flip, unsigned int cmd,
	unsigned long arg)
{
	switch (cmd) {
	case VPU_IOCTL_SET_POWER:
	case VPU_IOCTL_ENQUE_REQUEST:
	case VPU_IOCTL_DEQUE_REQUEST:
	case VPU_IOCTL_GET_ALGO_INFO:
	case VPU_IOCTL_REG_WRITE:
	case VPU_IOCTL_REG_READ:
	case VPU_IOCTL_LOAD_ALG_TO_POOL:
	case VPU_IOCTL_GET_CORE_STATUS:
	case VPU_IOCTL_OPEN_DEV_NOTICE:
	case VPU_IOCTL_CLOSE_DEV_NOTICE:
	{
		/*void *ptr = compat_ptr(arg);*/

		/*return vpu_ioctl(flip, cmd, (unsigned long) ptr);*/
		return flip->f_op->unlocked_ioctl(flip, cmd,
					(unsigned long)compat_ptr(arg));
	}
	case VPU_IOCTL_LOCK:
	case VPU_IOCTL_UNLOCK:
	default:
		return -ENOIOCTLCMD;
		/*return vpu_ioctl(flip, cmd, arg);*/
	}
}
#endif

static long vpu_ioctl(struct file *flip, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct vpu_user *user = flip->private_data;
	int i = 0;

	switch (cmd) {
	case VPU_IOCTL_SET_POWER:
	{
		struct vpu_power power;

		ret = copy_from_user(&power, (void *) arg,
					sizeof(struct vpu_power));
		if (ret) {
			LOG_ERR("[SET_POWER] %s, ret=%d\n",
					"copy 'struct power' failed", ret);
			goto out;
		}

		ret = vpu_set_power(user, &power);
		if (ret) {
			LOG_ERR("[SET_POWER] set power failed, ret=%d\n", ret);
			goto out;
		}

		break;
	}
	case VPU_IOCTL_ENQUE_REQUEST:
	{
		struct vpu_request *req;
		struct vpu_request *u_req;

		/*if (g_vpu_log_level > VpuLogThre_PERFORMANCE)*/
		LOG_INF("[vpu] VPU_IOCTL_ENQUE_REQUEST +\n");

		ret = vpu_alloc_request(&req);
		if (ret) {
			LOG_ERR("[ENQUE alloc request failed, ret=%d\n", ret);
			goto out;
		}

		u_req = (struct vpu_request *) arg;
		ret = get_user(req->user_id, &u_req->user_id);
		ret |= get_user(req->request_id, &u_req->request_id);
		ret |= get_user(req->requested_core, &u_req->requested_core);
		ret |= copy_from_user(req->algo_id, u_req->algo_id,
				VPU_MAX_NUM_CORES * sizeof(vpu_id_t));
		ret |= get_user(req->frame_magic, &u_req->frame_magic);
		ret |= get_user(req->status, &u_req->status);
		ret |= get_user(req->buffer_count, &u_req->buffer_count);
		ret |= get_user(req->sett_ptr, &u_req->sett_ptr);
		ret |= get_user(req->sett_length, &u_req->sett_length);
		ret |= get_user(req->priv, &u_req->priv);
		ret |= get_user(req->power_param.bw,
					&u_req->power_param.bw);

		ret |= get_user(req->power_param.freq_step,
					&u_req->power_param.freq_step);

		ret |= get_user(req->power_param.opp_step,
					&u_req->power_param.opp_step);

		ret |= get_user(req->power_param.core,
					&u_req->power_param.core);
		req->user_id = (unsigned long *)user;

		if (req->request_id == 0x0) {
			LOG_ERR("wrong request_id (0x%lx)\n",
					(unsigned long)req->request_id);
			vpu_free_request(req);
			ret = -EFAULT;
			goto out;
		}

		if (ret)
			LOG_ERR("[ENQUE] get params failed, ret=%d\n", ret);
		else if (req->buffer_count > VPU_MAX_NUM_PORTS) {
			LOG_ERR("[ENQUE] %s, count=%d\n",
				"wrong buffer count", req->buffer_count);
		} else if (copy_from_user(req->buffers, u_req->buffers,
			    req->buffer_count * sizeof(struct vpu_buffer))) {
			LOG_ERR("[ENQUE] %s, ret=%d\n",
				"copy 'struct buffer' failed", ret);
		} else if (copy_from_user(req->buf_ion_infos,
				u_req->buf_ion_infos,
				req->buffer_count * 3 * sizeof(uint64_t))) {
			LOG_ERR("[ENQUE] %s, ret=%d\n",
				"copy 'buf_share_fds' failed", ret);
		} else if (vpu_put_request_to_pool(user, req)) {
			LOG_ERR("[ENQUE] %s, ret=%d\n",
				"push to user's queue failed", ret);
		} else
			break;

		/* free the request, error happened here*/
		vpu_free_request(req);
		if (g_vpu_log_level > VpuLogThre_PERFORMANCE)
			LOG_INF("[vpu] .VPU_IOCTL_ENQUE_REQUEST - ");
		ret = -EFAULT;
		break;
	}
	case VPU_IOCTL_DEQUE_REQUEST:
	{
		struct vpu_request *req;
		uint64_t kernel_request_id;
		struct vpu_request *u_req;

		if (g_vpu_log_level > VpuLogThre_PERFORMANCE)
			LOG_INF("[vpu] VPU_IOCTL_DEQUE_REQUEST + ");

		u_req = (struct vpu_request *) arg;
		#if 1
		ret = get_user(kernel_request_id, &u_req->request_id);
		if (ret) {
			LOG_ERR("[REG] get 'req id' failed,%d\n", ret);
			goto out;
		}

		LOG_DBG("[vpu] deque test: user_id_0x%lx, request_id_0x%lx",
			(unsigned long)user,
			(unsigned long)(kernel_request_id));

		ret = vpu_get_request_from_queue(user, kernel_request_id, &req);
		#else
		LOG_DBG("[vpu] dequee test: user_id_0x%lx, request_id_0x%lx",
				(unsigned long)user,
				(unsigned long)(u_req->request_id));

		ret = vpu_get_request_from_queue(user, u_req->request_id, &req);
		#endif
		if (ret) {
			LOG_ERR("[DEQUE] pop request failed, ret=%d\n", ret);
			goto out;
		}

		ret = put_user(req->status, &u_req->status);
		ret |= put_user(req->occupied_core, &u_req->occupied_core);
		ret |= put_user(req->frame_magic, &u_req->frame_magic);
		if (ret)
			LOG_ERR("[DEQUE] update status failed, ret=%d\n", ret);

		ret = vpu_free_request(req);
		if (ret) {
			LOG_ERR("[DEQUE] free request, ret=%d\n", ret);
			goto out;
		}
		if (g_vpu_log_level > VpuLogThre_PERFORMANCE)
			LOG_INF("[vpu] VPU_IOCTL_DEQUE_REQUEST - ");
		break;
	}
	case VPU_IOCTL_FLUSH_REQUEST:
	{
		ret = vpu_flush_requests_from_queue(user);
		if (ret) {
			LOG_ERR("[FLUSH] flush request failed, ret=%d\n", ret);
			goto out;
		}
		break;
	}
	case VPU_IOCTL_GET_ALGO_INFO:
	{
		LOG_WRN("DO NOT SUPPORT VPU_IOCTL_GET_ALGO_INFO");
		break;
	}
	case VPU_IOCTL_REG_WRITE:
	{
		struct vpu_reg_values regs;

		ret = copy_from_user(&regs, (void *) arg,
					sizeof(struct vpu_reg_values));
		if (ret) {
			LOG_ERR("[REG] copy 'struct reg' failed,%d\n", ret);
			goto out;
		}

		ret = vpu_write_register(&regs);
		if (ret) {
			LOG_ERR("[REG] write reg failed,%d\n", ret);
			goto out;
		}
		break;
	}
	case VPU_IOCTL_LOCK:
	{
		LOG_WRN("DO NOT SUPPORT VPU_IOCTL_LOCK\n");
		break;
	}
	case VPU_IOCTL_UNLOCK:
	{
		LOG_WRN("DO NOT SUPPORT VPU_IOCTL_LOCK\n");
		break;
	}
	case VPU_IOCTL_LOAD_ALG_TO_POOL:
	{
		char name[32];
		vpu_id_t algo_id[MTK_VPU_CORE];
		int temp_algo_id;
		struct vpu_algo *u_algo;

		u_algo = (struct vpu_algo *) arg;
		ret = copy_from_user(name, u_algo->name, (sizeof(char)*32));
		if (ret) {
			LOG_ERR("[GET_ALGO] copy 'name' failed, ret=%d\n", ret);
			goto out;
		}

		name[(sizeof(char)*32) - 1] = '\0';

		for (i = 0 ; i < MTK_VPU_CORE ; i++) {
			temp_algo_id = vpu_get_algo_id_by_name(i, name);
			if (temp_algo_id < 0) {
				LOG_ERR("[GET_ALGO] %s, name=%s, id:%d\n",
						"can not find algo",
						name, temp_algo_id);
				ret = -ESPIPE;
				goto out;
			} else {
				LOG_DBG("[GET_ALGO] core(%d) name=%s, id=%d\n",
						i, name, temp_algo_id);
			}
			algo_id[i] = (vpu_id_t)temp_algo_id;
		}

		ret = copy_to_user(u_algo->id, algo_id,
				MTK_VPU_CORE * sizeof(vpu_id_t));
		if (ret) {
			LOG_ERR("[GET_ALGO] update id failed, ret=%d\n", ret);
			goto out;
		}

		break;
	}
	case VPU_IOCTL_GET_CORE_STATUS:
	{
		struct vpu_status *u_status = (struct vpu_status *) arg;
		struct vpu_status status;

		ret = copy_from_user(&status, (void *) arg,
				sizeof(struct vpu_status));
		if (ret) {
			LOG_ERR("[%s]copy 'struct vpu_status' failed ret=%d\n",
					"GET_CORE_STATUS", ret);
			goto out;
		}

		ret = vpu_get_core_status(&status);
		if (ret) {
			LOG_ERR("[%s] vpu_get_core_status failed, ret=%d\n",
					"GET_CORE_STATUS", ret);
			goto out;
		}

		ret = put_user(status.vpu_core_available,
				(bool *)&(u_status->vpu_core_available));
		ret |= put_user(status.pool_list_size,
					(int *)&(u_status->pool_list_size));
		if (ret) {
			LOG_ERR("[%s] put to user failed, ret=%d\n",
					"GET_CORE_STATUS", ret);
			goto out;
		}

		break;
	}
	case VPU_IOCTL_OPEN_DEV_NOTICE:
	{
		struct vpu_dev_debug_info *dev_debug_info;
		struct vpu_dev_debug_info *u_dev_debug_info;

		ret = vpu_alloc_debug_info(&dev_debug_info);
		if (ret) {
			LOG_ERR("[%s] alloc debug_info failed, ret=%d\n",
					"OPEN_DEV_NOTICE", ret);
			goto out;
		}

		u_dev_debug_info = (struct vpu_dev_debug_info *) arg;
		ret = get_user(dev_debug_info->dev_fd,
					&u_dev_debug_info->dev_fd);
		if (ret) {
			LOG_ERR("[%s] copy 'dev_fd' failed, ret=%d\n",
				"VPU_IOCTL_OPEN_DEV_NOTICE", ret);
		}

		ret |= copy_from_user(dev_debug_info->callername,
			u_dev_debug_info->callername, (sizeof(char)*32));
		dev_debug_info->callername[(sizeof(char)*32) - 1] = '\0';
		if (ret) {
			LOG_ERR("[%s] copy 'callname' failed, ret=%d\n",
					"VPU_IOCTL_OPEN_DEV_NOTICE", ret);
		}

		dev_debug_info->open_pid = user->open_pid;
		dev_debug_info->open_tgid = user->open_tgid;

		if (g_vpu_log_level > Log_ALGO_OPP_INFO) {
			LOG_INF("[%s] user:%s/%d. pid(%d/%d)\n",
				"VPU_IOCTL_OPEN_DEV_NOTICE",
				dev_debug_info->callername,
				dev_debug_info->dev_fd,
				dev_debug_info->open_pid,
				dev_debug_info->open_tgid);
		}

		if (ret) {
			/* error handle, free memory */
			vpu_free_debug_info(dev_debug_info);
		} else {
			mutex_lock(&debug_list_mutex);
			list_add_tail(vlist_link(dev_debug_info,
					struct vpu_dev_debug_info),
					&device_debug_list);
			mutex_unlock(&debug_list_mutex);
		}

		break;
	}
	case VPU_IOCTL_CLOSE_DEV_NOTICE:
	{
		int dev_fd;
		struct list_head *head = NULL;
		struct vpu_dev_debug_info *dbg_info;
		bool get = false;

		ret = copy_from_user(&dev_fd, (void *) arg, sizeof(int));
		if (ret) {
			LOG_ERR("[%s] copy 'dev_fd' failed, ret=%d\n",
					"CLOSE_DEV_NOTICE", ret);
			goto out;
		}

		list_for_each(head, &device_debug_list)
		{
			dbg_info = vlist_node_of(head,
						struct vpu_dev_debug_info);
			if (g_vpu_log_level > Log_ALGO_OPP_INFO) {
				LOG_INF("[%s] req_user-> = %s/%d, %d/%d\n",
					"VPU_IOCTL_CLOSE_DEV_NOTICE",
					dbg_info->callername,
					dbg_info->dev_fd, dbg_info->open_pid,
					dbg_info->open_tgid);
			}

			if (dbg_info->dev_fd == dev_fd) {
				LOG_DBG("[%s] get fd(%d) to close\n",
					"VPU_IOCTL_CLOSE_DEV_NOTICE", dev_fd);
				get = true;
				break;
			}
		}

		if (g_vpu_log_level > Log_ALGO_OPP_INFO) {
			LOG_INF("[%s] user:%d. pid(%d/%d), get(%d)\n",
					"VPU_IOCTL_CLOSE_DEV_NOTICE",
					dev_fd, user->open_pid,
					user->open_tgid, get);
		}

		if (get) {
			list_del_init(vlist_link(dbg_info,
						struct vpu_dev_debug_info));
			vpu_free_debug_info(dbg_info);
		} else {
			LOG_ERR("[%s] want to close wrong fd(%d)\n",
				"VPU_IOCTL_CLOSE_DEV_NOTICE", dev_fd);
			ret = -ESPIPE;
			goto out;
		}

		break;
	}
	default:
		LOG_WRN("ioctl: no such command!\n");
		ret = -EINVAL;
		break;
	}

out:
	if (ret) {
		LOG_ERR("fail, cmd(%d), pid(%d), %s=%s, %s=%d, %s=%d\n",
				cmd, user->open_pid,
				"process", current->comm,
				"pid", current->pid,
				"tgid", current->tgid);
	}

	return ret;
}

static int vpu_release(struct inode *inode, struct file *flip)
{
	struct vpu_user *user = flip->private_data;

	LOG_INF("%s cnt(%d) user->id : 0x%lx, tids(%d/%d)\n", __func__,
		vpu_num_users,
		(unsigned long)(user->id),
		user->open_pid, user->open_tgid);

	vpu_delete_user(user);
	mutex_lock(&debug_list_mutex);
	vpu_num_users--;
	mutex_unlock(&debug_list_mutex);

	if (vpu_num_users > 10)
		vpu_dump_device_dbg(NULL);

	return 0;
}

static int vpu_mmap(struct file *flip, struct vm_area_struct *vma)
{
	unsigned long length = 0;
	unsigned int pfn = 0x0;

	length = (vma->vm_end - vma->vm_start);
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	pfn = vma->vm_pgoff << PAGE_SHIFT;

	LOG_INF("%s:%s=0x%lx,%s=0x%x,%s=0x%lx,%s=0x%lx,%s=0x%lx,%s=0x%lx\n",
			__func__,
			"vm_pgoff", vma->vm_pgoff,
			"pfn", pfn,
			"phy", vma->vm_pgoff << PAGE_SHIFT,
			"vm_start", vma->vm_start,
			"vm_end", vma->vm_end,
			"length", length);

	switch (pfn) {

	default:
		LOG_ERR("illegal hw addr for mmap!\n");
		return -EAGAIN;
	}
}

static dev_t vpu_devt;
static struct cdev *vpu_chardev;
static struct class *vpu_class;
static int vpu_num_devs;

static inline void vpu_unreg_chardev(void)
{
	/* Release char driver */
	if (vpu_chardev != NULL) {
		cdev_del(vpu_chardev);
		vpu_chardev = NULL;
	}
	unregister_chrdev_region(vpu_devt, 1);
}

static inline int vpu_reg_chardev(void)
{
	int ret = 0;

	ret = alloc_chrdev_region(&vpu_devt, 0, 1, VPU_DEV_NAME);
	if ((ret) < 0) {
		LOG_ERR("alloc_chrdev_region failed, %d\n", ret);
		return ret;
	}
	/* Allocate driver */
	vpu_chardev = cdev_alloc();
	if (vpu_chardev == NULL) {
		LOG_ERR("cdev_alloc failed\n");
		ret = -ENOMEM;
		goto out;
	}

	/* Attatch file operation. */
	cdev_init(vpu_chardev, &vpu_fops);

	vpu_chardev->owner = THIS_MODULE;

	/* Add to system */
	ret = cdev_add(vpu_chardev, vpu_devt, 1);
	if ((ret) < 0) {
		LOG_ERR("Attatch file operation failed, %d\n", ret);
		goto out;
	}

out:
	if (ret < 0)
		vpu_unreg_chardev();

	return ret;
}

/******************************************************************************
 * platform_driver
 *****************************************************************************/

static int vpu_probe(struct platform_device *pdev)
{
	int ret = 0;
	int core = 0;
	struct device *dev;
	struct device_node *node;
	unsigned int irq_info[3]; /* Record interrupts info from device tree */
	struct device_node *smi_node = NULL;
	struct device_node *ipu_conn_node = NULL;

	core = vpu_num_devs;

	smi_node = NULL;

	if (core == MTK_VPU_CORE) {
		LOG_INF("%s(%d), core(%d) = core(%d)+2 in FPGA, return\n",
			"vpu_num_devs", vpu_num_devs, core, MTK_VPU_CORE);
		return ret;
	}

	node = pdev->dev.of_node;
	vpu_device->dev[vpu_num_devs] = &pdev->dev;
	LOG_INF("probe 0, pdev id = %d name = %s, name = %s\n",
			pdev->id, pdev->name,
			pdev->dev.of_node->name);

#ifdef MTK_VPU_FPGA_PORTING
	if (vpu_num_devs == 0) {
		/* get register address */
#ifdef NEVER
		if (strncmp(node->name, "ipu_conn",
						strlen("ipu_conn"))) {
			vpu_device->vpu_syscfg_base =
					(unsigned long) of_iomap(node, 0);
		} else if (strncmp(node->name, "ipu_adl",
						strlen("ipu_adl"))) {
			vpu_device->vpu_adlctrl_base =
					(unsigned long) of_iomap(node, 0);
		} else if (strncmp(node->name, "ipu_vcore",
						strlen("ipu_vcore"))) {
			vpu_device->vpu_vcorecfg_base =
					(unsigned long) of_iomap(node, 0);
		}
#endif /* NEVER */
		struct device_node *tmp;

		tmp = of_find_compatible_node(NULL, NULL, "mediatek,ipu_conn");
		vpu_device->vpu_syscfg_base = (unsigned long) of_iomap(tmp, 0);

		tmp = of_find_compatible_node(NULL, NULL, "mediatek,ipu_adl");
		vpu_device->vpu_adlctrl_base = (unsigned long) of_iomap(tmp, 0);

		tmp = of_find_compatible_node(NULL, NULL, "mediatek,ipu_vcore");
		vpu_device->vpu_vcorecfg_base =
					(unsigned long) of_iomap(tmp, 0);

		LOG_DBG("probe 2, %s=0x%lx, %s=0x%lx, %s=0x%lx\n",
			 "vpu_syscfg_base", vpu_device->vpu_syscfg_base,
			 "vpu_adlctrl_base", vpu_device->vpu_adlctrl_base,
			 "vpu_vcorecfg_base", vpu_device->vpu_vcorecfg_base);
	}
#endif

#ifdef MTK_VPU_EMULATOR
	/* emulator will fill vpu_base and bin_base */
	vpu_init_emulator(vpu_device);
#else
	LOG_INF("[vpu] core/total : %d/%d\n", core, MTK_VPU_CORE);
	vpu_device->vpu_base[core] = (unsigned long) of_iomap(node, 0);
	/* get physical address of binary data loaded by LK */
	if (vpu_num_devs == 0) {
		uint32_t phy_addr;
		uint32_t phy_size;

		if (of_property_read_u32(node, "bin-phy-addr", &phy_addr) ||
			of_property_read_u32(node, "bin-size", &phy_size)) {
			LOG_ERR("fail to get phy address of vpu binary!\n");
			return -ENODEV;
		}

		/* bin_base for cpu read/write */
		vpu_device->bin_base =
			(unsigned long)ioremap_wc(phy_addr, phy_size);
		vpu_device->bin_pa = phy_addr;
		vpu_device->bin_size = phy_size;

		LOG_INF("probe core:%d, %s=0x%lx %s=0x%x, %s=0x%x\n",
			core,
			"bin_base", (unsigned long)vpu_device->bin_base,
			"phy_addr", phy_addr,
			"phy_size", phy_size);

		/* get smi common register */
		#ifdef MTK_VPU_SMI_DEBUG_ON
		smi_node = of_find_compatible_node(NULL, NULL,
						"mediatek,smi_common");
		vpu_device->smi_cmn_base =
				(unsigned long) of_iomap(smi_node, 0);
		#endif

		ipu_conn_node = of_find_compatible_node(NULL, NULL,
						"mediatek,ipu_conn");

		vpu_device->vpu_syscfg_base =
				(unsigned long) of_iomap(ipu_conn_node, 0);

		LOG_INF("probe, smi_cmn_base: 0x%lx, ipu_conn:0x%lx\n",
				vpu_device->smi_cmn_base,
				vpu_device->vpu_syscfg_base);
	}
#endif

	vpu_device->irq_num[core] = irq_of_parse_and_map(node, 0);
	LOG_DBG("probe 2, [%d/%d] %s=0x%lx, %s=0x%lx, %s=%d, %s:%p\n",
			 vpu_num_devs, core,
			 "vpu_base", vpu_device->vpu_base[core],
			 "bin_base", vpu_device->bin_base,
			 "irq_num", vpu_device->irq_num[core],
			 "pdev", vpu_device->dev[vpu_num_devs]);
	if (vpu_device->irq_num[core] > 0) {
		/* Get IRQ Flag from device node */
		if (of_property_read_u32_array(pdev->dev.of_node,
				"interrupts", irq_info, ARRAY_SIZE(irq_info))) {
			dev_err(&pdev->dev, "get irq flags from DTS fail!!\n");
			return -ENODEV;
		}
		vpu_device->irq_trig_level = irq_info[2];
		LOG_DBG("vpu_device->irq_trig_level (0x%x), %s(0x%x)\n",
			vpu_device->irq_trig_level,
			"IRQF_TRIGGER_NONE", IRQF_TRIGGER_NONE);
	}

	vpu_init_algo(vpu_device);
	LOG_DBG("[probe] [%d] init_algo done\n", core);
	vpu_init_hw(core, vpu_device);
	LOG_DBG("[probe] [%d] init_hw done\n", core);
	vpu_init_reg(core, vpu_device);
	LOG_DBG("[probe] [%d] init_reg done\n", core);
#ifdef MET_POLLING_MODE
	vpu_init_profile(core, vpu_device);
	LOG_DBG("[probe] [%d] vpu_init_profile done\n", core);
#endif
	if (!my_ion_client)
		my_ion_client = ion_client_create(g_ion_device, "vpu_drv");

	/* Only register char driver in the 1st time */
	if (++vpu_num_devs == 1) {
		vpu_init_debug(vpu_device);
		/* Register char driver */
		ret = vpu_reg_chardev();
		if (ret) {
			dev_err(vpu_device->dev[vpu_num_devs-1], "register char failed");
			return ret;
		}
		/* Create class register */
		vpu_class = class_create(THIS_MODULE, "vpudrv");
		if (IS_ERR(vpu_class)) {
			ret = PTR_ERR(vpu_class);
			LOG_ERR("Unable to create class, err = %d\n", ret);
			goto out;
		}

		dev = device_create(vpu_class, NULL, vpu_devt,
					NULL, VPU_DEV_NAME);
		if (IS_ERR(dev)) {
			ret = PTR_ERR(dev);
			dev_err(vpu_device->dev[vpu_num_devs-1],
				"Failed to create device: /dev/%s, err = %d",
				VPU_DEV_NAME, ret);
			goto out;
		}

		wakeup_source_init(&vpu_wake_lock, "vpu_lock_wakelock");

out:
		if (ret < 0)
			vpu_unreg_chardev();
	}

	LOG_DBG("probe vpu driver\n");

	return ret;
}


static int vpu_remove(struct platform_device *pDev)
{
	/*    struct resource *pRes;*/
	int irq_num, i;

	/*  */
	vpu_uninit_hw();
	/*  */
#ifdef MET_POLLING_MODE
	vpu_uninit_profile();
#endif
	if (my_ion_client) {
		ion_client_destroy(my_ion_client);
		my_ion_client = NULL;
	}
	/* */
	LOG_DBG("remove vpu driver");
	/* unregister char driver. */
	vpu_unreg_chardev();

	/* Release IRQ */
	for (i = 0 ; i < MTK_VPU_CORE ; i++)
		disable_irq(vpu_device->irq_num[i]);

	irq_num = platform_get_irq(pDev, 0);
	free_irq(irq_num, (void *) vpu_chardev);

	device_destroy(vpu_class, vpu_devt);
	class_destroy(vpu_class);
	vpu_class = NULL;
	return 0;
}

static int vpu_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	int i = 0;

	for (i = 0 ; i < MTK_VPU_CORE ; i++)
		vpu_quick_suspend(i);

	return 0;
}

static int vpu_resume(struct platform_device *pdev)
{
	return 0;
}

static int __init VPU_INIT(void)
{
	int ret = 0, i = 0;

	vpu_device = kzalloc(sizeof(struct vpu_device), GFP_KERNEL);

	INIT_LIST_HEAD(&vpu_device->user_list);
	mutex_init(&vpu_device->user_mutex);
	/*  */
	for (i = 0 ; i < MTK_VPU_CORE ; i++) {
		INIT_LIST_HEAD(&vpu_device->pool_list[i]);
		mutex_init(&vpu_device->servicepool_mutex[i]);
		vpu_device->servicepool_list_size[i] = 0;
		vpu_device->service_core_available[i] = true;
	}
	INIT_LIST_HEAD(&vpu_device->cmnpool_list);
	mutex_init(&vpu_device->commonpool_mutex);
	vpu_device->commonpool_list_size = 0;
	init_waitqueue_head(&vpu_device->req_wait);
	INIT_LIST_HEAD(&device_debug_list);
	mutex_init(&debug_list_mutex);

	/* Register M4U callback */
	LOG_DBG("register m4u callback");
	m4u_register_fault_callback(VPU_PORT_OF_IOMMU,
				vpu_m4u_fault_callback, NULL);

	LOG_DBG("platform_driver_register start\n");
	if (platform_driver_register(&vpu_driver)) {
		LOG_ERR("failed to register VPU driver");
		return -ENODEV;
	}
	LOG_DBG("platform_driver_register finsish\n");

	return ret;
}


static void __exit VPU_EXIT(void)
{
	platform_driver_unregister(&vpu_driver);

	kfree(vpu_device);
	/* Un-Register M4U callback */
	LOG_DBG("un-register m4u callback");
	m4u_unregister_fault_callback(VPU_PORT_OF_IOMMU);
}

module_init(VPU_INIT);
module_exit(VPU_EXIT);
MODULE_DESCRIPTION("MTK VPU Driver");
MODULE_AUTHOR("SW6");
MODULE_LICENSE("GPL");

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

#include "ccci_fsm_internal.h"

static char *fsm_monitor_name[MAX_MD_NUM] = {
	"ccci_monitor",
	"ccci2_monitor",
	"ccci3_monitor",
};

static int mdinit_killed;

static void set_mdinit_killed(int killed)
{
	mdinit_killed = killed;
}

int get_mdinit_killed(void)
{
	return mdinit_killed;
}

static int dev_char_open(struct inode *inode, struct file *file)
{
	struct ccci_fsm_ctl *ctl = NULL;
	struct ccci_fsm_monitor *monitor_ctl = NULL;

	ctl = fsm_get_entity_by_device_number(inode->i_rdev);
	if (!ctl)
		return -EINVAL;
	monitor_ctl = &ctl->monitor_ctl;
	if (atomic_read(&monitor_ctl->usage_cnt))
		return -EBUSY;

	monitor_ctl = &ctl->monitor_ctl;
	CCCI_NORMAL_LOG(monitor_ctl->md_id, FSM,
		"monitor node open by %s\n", current->comm);
	atomic_inc(&monitor_ctl->usage_cnt);
	file->private_data = monitor_ctl;
	nonseekable_open(inode, file);
	return 0;
}

static int dev_char_close(struct inode *inode, struct file *file)
{
	struct ccci_fsm_monitor *monitor_ctl = file->private_data;
	struct sk_buff *skb = NULL;
	int clear_cnt = 0, ret = 0;

	atomic_dec(&monitor_ctl->usage_cnt);
	while ((skb = ccci_skb_dequeue(&monitor_ctl->rx_skb_list)) != NULL) {
		ccci_free_skb(skb);
		clear_cnt++;
	}
	CCCI_NORMAL_LOG(monitor_ctl->md_id, FSM,
		"monitor close, clear_cnt=%d\n", clear_cnt);

	set_mdinit_killed(1);

	ret = force_md_stop(monitor_ctl);
	if (ret)
		CCCI_ERROR_LOG(monitor_ctl->md_id, FSM, "force stop MD fail\n");
	return 0;
}

static ssize_t dev_char_read(struct file *file, char *buf,
	size_t count, loff_t *ppos)
{
	struct ccci_fsm_monitor *monitor_ctl = file->private_data;
	struct sk_buff *skb = NULL;
	int ret = 0, read_len = 0;

	if (skb_queue_empty(&monitor_ctl->rx_skb_list.skb_list)) {
		ret = wait_event_interruptible(monitor_ctl->rx_wq,
		!skb_queue_empty(&monitor_ctl->rx_skb_list.skb_list));
		if (ret == -ERESTARTSYS) {
			ret = -EINTR;
			goto exit;
		}
	}
	skb = ccci_skb_dequeue(&monitor_ctl->rx_skb_list);
	if (skb) {
		read_len = skb->len;
		if (read_len > count
			|| copy_to_user(buf, skb->data, read_len)) {
			CCCI_ERROR_LOG(monitor_ctl->md_id, FSM,
				"read on monitor, copy to user failed, %d/%zu\n",
				     read_len, count);
			ret = -EFAULT;
		}
		ccci_free_skb(skb);
	}

 exit:
	return ret ? ret : read_len;
}

static ssize_t dev_char_write(struct file *file, const char __user *buf,
	size_t count, loff_t *ppos)
{
	return -EACCES;
}

static long dev_char_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	struct ccci_fsm_monitor *monitor_ctl = file->private_data;

	return ccci_fsm_ioctl(monitor_ctl->md_id, cmd, arg);
}

#ifdef CONFIG_COMPAT
static long dev_char_compat_ioctl(struct file *filp, unsigned int cmd,
	unsigned long arg)
{
	if (!filp->f_op || !filp->f_op->unlocked_ioctl)
		return -ENOTTY;
	return filp->f_op->unlocked_ioctl(filp, cmd,
				(unsigned long)compat_ptr(arg));
}
#endif

static const struct file_operations char_dev_fops = {
	.owner = THIS_MODULE,
	.open = &dev_char_open,
	.read = &dev_char_read,
	.write = &dev_char_write,
	.release = &dev_char_close,
	.unlocked_ioctl = &dev_char_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = &dev_char_compat_ioctl,
#endif
};

int fsm_monitor_send_message(int md_id, CCCI_MD_MSG msg, u32 resv)
{
	struct sk_buff *skb = NULL;
	struct ccci_header *ccci_h;
	struct ccci_fsm_ctl *ctl = fsm_get_entity_by_md_id(md_id);
	struct ccci_fsm_monitor *monitor_ctl = &ctl->monitor_ctl;

	if (!ctl)
		return -CCCI_ERR_INVALID_PARAM;

	if (unlikely(in_interrupt())) {
		CCCI_ERROR_LOG(monitor_ctl->md_id, FSM,
			"sending virtual msg from IRQ context %ps\n",
			__builtin_return_address(0));
		return -CCCI_ERR_ASSERT_ERR;
	}

	skb = ccci_alloc_skb(sizeof(struct ccci_header), 1, 1);
	ccci_h =
	(struct ccci_header *)skb_put(skb, sizeof(struct ccci_header));
	ccci_h->data[0] = CCCI_MAGIC_NUM;
	ccci_h->data[1] = msg;
	*(((u32 *) ccci_h) + 2) = CCCI_MONITOR_CH_ID;
	ccci_h->reserved = resv;
	ccci_skb_enqueue(&monitor_ctl->rx_skb_list, skb);
	wake_up_all(&monitor_ctl->rx_wq);
	return 0;
}

int fsm_monitor_init(struct ccci_fsm_monitor *monitor_ctl)
{
	struct ccci_fsm_ctl *ctl = container_of(monitor_ctl,
		struct ccci_fsm_ctl, monitor_ctl);
	int ret = 0;

	monitor_ctl->md_id = ctl->md_id;
	ccci_skb_queue_init(&monitor_ctl->rx_skb_list, 16, 1024, 0);
	init_waitqueue_head(&monitor_ctl->rx_wq);
	atomic_set(&monitor_ctl->usage_cnt, 0);

	monitor_ctl->char_dev = kmalloc(sizeof(struct cdev), GFP_KERNEL);

	if (unlikely(!monitor_ctl->char_dev)) {
		CCCI_ERROR_LOG(monitor_ctl->md_id, FSM,
			"alloc fsm monitor char dev fail!!\n");
		return -1;
	}

	cdev_init(monitor_ctl->char_dev, &char_dev_fops);
	monitor_ctl->char_dev->owner = THIS_MODULE;
	ret = alloc_chrdev_region(&monitor_ctl->dev_n,
			monitor_ctl->md_id, 1, FSM_NAME);
	if (ret)
		CCCI_ERROR_LOG(monitor_ctl->md_id, FSM,
			"alloc_chrdev_region fail, ret=%d\n", ret);
	ret = cdev_add(monitor_ctl->char_dev, monitor_ctl->dev_n, 1);
	if (ret)
		CCCI_ERROR_LOG(monitor_ctl->md_id, FSM,
			"cdev_add fail, ret=%d\n", ret);

	ret = ccci_register_dev_node(fsm_monitor_name[monitor_ctl->md_id],
			MAJOR(monitor_ctl->dev_n), MINOR(monitor_ctl->dev_n));
	if (ret)
		CCCI_ERROR_LOG(monitor_ctl->md_id, FSM,
			"device_create fail, ret=%d\n", ret);
	return ret;
}


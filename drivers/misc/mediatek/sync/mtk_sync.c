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


#include <linux/debugfs.h>
#include <linux/export.h>
#include <linux/seq_file.h>
#include <linux/file.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <linux/sync_file.h>

#include "mtk_sync.h"

/* ---------------------------------------------------------------- */

static const struct fence_ops timeline_fence_ops;

static inline struct sync_pt *fence_to_sync_pt(struct fence *fence)
{
	if (fence->ops != &timeline_fence_ops)
		return NULL;
	return container_of(fence, struct sync_pt, base);
}

static void sync_timeline_free(struct kref *kref)
{
	struct sync_timeline *obj =
		container_of(kref, struct sync_timeline, kref);

	sync_timeline_debug_remove(obj);

	kfree(obj);
}

static void sync_timeline_get(struct sync_timeline *obj)
{
	kref_get(&obj->kref);
}

static void sync_timeline_put(struct sync_timeline *obj)
{
	kref_put(&obj->kref, sync_timeline_free);
}

/**
 * sync_pt_create() - creates a sync pt
 * @parent:	fence's parent sync_timeline
 * @size:	size to allocate for this pt
 * @inc:	value of the fence
 *
 * Creates a new sync_pt as a child of @parent.  @size bytes will be
 * allocated allowing for implementation specific data to be kept after
 * the generic sync_timeline struct. Returns the sync_pt object or
 * NULL in case of error.
 */
static struct sync_pt *sync_pt_create(struct sync_timeline *obj,
					  unsigned int value)
{
	struct sync_pt *pt;

	pt = kzalloc(sizeof(*pt), GFP_KERNEL);
	if (!pt)
		return NULL;

	sync_timeline_get(obj);
	fence_init(&pt->base, &timeline_fence_ops, &obj->lock,
		   obj->context, value);
	INIT_LIST_HEAD(&pt->link);

	spin_lock_irq(&obj->lock);
	if (!fence_is_signaled_locked(&pt->base)) {
		struct rb_node **p = &obj->pt_tree.rb_node;
		struct rb_node *parent = NULL;

		while (*p) {
			struct sync_pt *other;
			int cmp;

			parent = *p;
			other = rb_entry(parent, typeof(*pt), node);
			cmp = value - other->base.seqno;
			if (cmp > 0) {
				p = &parent->rb_right;
			} else if (cmp < 0) {
				p = &parent->rb_left;
			} else {
				if (fence_get_rcu(&other->base)) {
					fence_put(&pt->base);
					pt = other;
					goto unlock;
				}
				p = &parent->rb_left;
			}
		}
		rb_link_node(&pt->node, parent, p);
		rb_insert_color(&pt->node, &obj->pt_tree);

		parent = rb_next(&pt->node);
		list_add_tail(&pt->link,
			parent ? &rb_entry(parent, typeof(*pt), node)->link :
			&obj->pt_list);
	}
unlock:
	spin_unlock_irq(&obj->lock);

	return pt;

}

static const char *timeline_fence_get_driver_name(struct fence *fence)
{
	return "mtk_sync";
}

static const char *timeline_fence_get_timeline_name(struct fence *fence)
{
	struct sync_timeline *parent = fence_parent(fence);

	return parent->name;
}

static void timeline_fence_release(struct fence *fence)
{
	struct sync_pt *pt = fence_to_sync_pt(fence);
	struct sync_timeline *parent = fence_parent(fence);

	if (!list_empty(&pt->link)) {
		unsigned long flags;

		spin_lock_irqsave(fence->lock, flags);
		if (!list_empty(&pt->link)) {
			list_del(&pt->link);
			rb_erase(&pt->node, &parent->pt_tree);
		}
		spin_unlock_irqrestore(fence->lock, flags);
	}

	sync_timeline_put(parent);
	fence_free(fence);
}

static bool timeline_fence_signaled(struct fence *fence)
{
	struct sync_timeline *parent = fence_parent(fence);

	return !__fence_is_later(fence->seqno, parent->value);
}

static bool timeline_fence_enable_signaling(struct fence *fence)
{
	return true;
}

static void timeline_fence_value_str(struct fence *fence,
				    char *str, int size)
{
	snprintf(str, size, "%d", fence->seqno);
}

static void timeline_fence_timeline_value_str(struct fence *fence,
					     char *str, int size)
{
	struct sync_timeline *parent = fence_parent(fence);

	snprintf(str, size, "%d", parent->value);
}

static const struct fence_ops timeline_fence_ops = {
	.get_driver_name = timeline_fence_get_driver_name,
	.get_timeline_name = timeline_fence_get_timeline_name,
	.enable_signaling = timeline_fence_enable_signaling,
	.signaled = timeline_fence_signaled,
	.wait = fence_default_wait,
	.release = timeline_fence_release,
	.fence_value_str = timeline_fence_value_str,
	.timeline_value_str = timeline_fence_timeline_value_str,
};

/* ---------------------------------------------------------------- */

/**
 * sync_timeline_signal() - signal a status change on a sync_timeline
 * @obj:	sync_timeline to signal
 * @inc:	num to increment on timeline->value
 *
 * A sync implementation should call this any time one of it's fences
 * has signaled or has an error condition.
 */
static void sync_timeline_signal(struct sync_timeline *obj, unsigned int inc)
{
	struct sync_pt *pt, *next;

	spin_lock_irq(&obj->lock);

	obj->value += inc;

	list_for_each_entry_safe(pt, next, &obj->pt_list, link) {
		if (!timeline_fence_signaled(&pt->base))
			break;

		list_del_init(&pt->link);
		rb_erase(&pt->node, &obj->pt_tree);

		/*
		 * A signal callback may release the last reference to this
		 * fence, causing it to be freed. That operation has to be
		 * last to avoid a use after free inside this loop, and must
		 * be after we remove the fence from the timeline in order to
		 * prevent deadlocking on timeline->lock inside
		 * timeline_fence_release().
		 */
		fence_signal_locked(&pt->base);
	}

	spin_unlock_irq(&obj->lock);
}

struct sync_timeline *timeline_create(const char *name)
{
	return sync_timeline_create(name);
}

void timeline_destroy(struct sync_timeline *obj)
{
	sync_timeline_put(obj);
}

void timeline_inc(struct sync_timeline *obj, u32 value)
{
	sync_timeline_signal(obj, value);
}

int fence_create(struct sync_timeline *obj, struct fence_data *data)
{
	int fd = get_unused_fd_flags(O_CLOEXEC);
	int err;
	struct sync_pt *pt;
	struct sync_file *sync_file;

	if (fd < 0)
		return fd;

	pt = sync_pt_create(obj, data->value);
	if (!pt) {
		err = -ENOMEM;
		goto err;
	}

	sync_file = sync_file_create(&pt->base);
	fence_put(&pt->base);
	if (!sync_file) {
		err = -ENOMEM;
		goto err;
	}

	data->fence = fd;

	fd_install(fd, sync_file->file);

	return 0;

err:
	put_unused_fd(fd);
	return err;

}

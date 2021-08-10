/*
 *
 * drivers/staging/android/ion/ion.c
 *
 * Copyright (C) 2011 Google, Inc.
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

#include <linux/device.h>
#include <linux/err.h>
#include <linux/file.h>
#include <linux/freezer.h>
#include <linux/fs.h>
#include <linux/anon_inodes.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/memblock.h>
#include <linux/miscdevice.h>
#include <linux/export.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/rbtree.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/debugfs.h>
#include <linux/dma-buf.h>
#include <linux/idr.h>

#include "ion.h"
#include "ion_priv.h"
#include "compat_ion.h"

#include "mtk/ion_profile.h"
#include "mtk/mtk_ion.h"
#include "mtk/ion_drv_priv.h"
atomic64_t page_sz_cnt = ATOMIC64_INIT(0);

bool ion_buffer_fault_user_mappings(struct ion_buffer *buffer)
{
	return (buffer->flags & ION_FLAG_CACHED) &&
		!(buffer->flags & ION_FLAG_CACHED_NEEDS_SYNC);
}

bool ion_buffer_cached(struct ion_buffer *buffer)
{
	return !!(buffer->flags & ION_FLAG_CACHED);
}

static inline struct page *ion_buffer_page(struct page *page)
{
	return (struct page *)((unsigned long)page & ~(1UL));
}

static inline bool ion_buffer_page_is_dirty(struct page *page)
{
	return !!((unsigned long)page & 1UL);
}

static inline void ion_buffer_page_dirty(struct page **page)
{
	*page = (struct page *)((unsigned long)(*page) | 1UL);
}

static inline void ion_buffer_page_clean(struct page **page)
{
	*page = (struct page *)((unsigned long)(*page) & ~(1UL));
}

/* this function should only be called while dev->lock is held */
static void ion_buffer_add(struct ion_device *dev,
			   struct ion_buffer *buffer)
{
	struct rb_node **p = &dev->buffers.rb_node;
	struct rb_node *parent = NULL;
	struct ion_buffer *entry;

	while (*p) {
		parent = *p;
		entry = rb_entry(parent, struct ion_buffer, node);

		if (buffer < entry) {
			p = &(*p)->rb_left;
		} else if (buffer > entry) {
			p = &(*p)->rb_right;
		} else {
			pr_err("%s: buffer already found.", __func__);
			BUG();
		}
	}

	rb_link_node(&buffer->node, parent, p);
	rb_insert_color(&buffer->node, &dev->buffers);
}

/* this function should only be called while dev->lock is held */
static struct ion_buffer *ion_buffer_create(struct ion_heap *heap,
					    struct ion_device *dev,
					    unsigned long len,
					    unsigned long align,
					    unsigned long flags)
{
	struct ion_buffer *buffer;
	struct sg_table *table;
	struct scatterlist *sg;
	int i, ret;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer) {
		IONMSG("%s kzalloc failed, buffer is null.\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	buffer->heap = heap;
	buffer->flags = flags;

	/* log task pid for debug +by k.zhang */
	{
		struct task_struct *task;

		task = current->group_leader;
		get_task_comm(buffer->task_comm, task);
		buffer->pid = task_pid_nr(task);
	}

	kref_init(&buffer->ref);

	ret = heap->ops->allocate(heap, buffer, len, align, flags);

	if (ret) {
		if (!(heap->flags & ION_HEAP_FLAG_DEFER_FREE))
			goto err2;

		ion_heap_freelist_drain(heap, 0);
		ret = heap->ops->allocate(heap, buffer, len, align,
					  flags);
		if (ret)
			goto err2;
	}

	if (buffer->sg_table == NULL) {
		WARN_ONCE(1, "This heap needs to set the sgtable");
		ret = -EINVAL;
		goto err1;
	}

	table = buffer->sg_table;
	buffer->dev = dev;
	buffer->size = len;

	if (ion_buffer_fault_user_mappings(buffer)) {
		int num_pages = PAGE_ALIGN(buffer->size) / PAGE_SIZE;
		struct scatterlist *sg;
		int i, j, k = 0;

		buffer->pages = vmalloc(sizeof(struct page *) * num_pages);
		if (!buffer->pages) {
			IONMSG("%s vamlloc failed pages is null.\n", __func__);
			ret = -ENOMEM;
			goto err1;
		}

		for_each_sg(table->sgl, sg, table->nents, i) {
			struct page *page = sg_page(sg);

			for (j = 0; j < sg->length / PAGE_SIZE; j++)
				buffer->pages[k++] = page++;
		}
	}

	buffer->dev = dev;
	buffer->size = len;
	INIT_LIST_HEAD(&buffer->vmas);
	mutex_init(&buffer->lock);
#ifdef CONFIG_MTK_PSEUDO_M4U_V2
	if (heap->id < ION_HEAP_TYPE_MULTIMEDIA) {
		for_each_sg(buffer->sg_table->sgl, sg, buffer->sg_table->nents,
			    i) {
			sg_dma_address(sg) = sg_phys(sg);
			sg_dma_len(sg) = sg->length;
		}
	}
#else
	/*
	 * this will set up dma addresses for the sglist -- it is not
	 * technically correct as per the dma api -- a specific
	 * device isn't really taking ownership here.  However, in practice on
	 * our systems the only dma_address space is physical addresses.
	 * Additionally, we can't afford the overhead of invalidating every
	 * allocation via dma_map_sg. The implicit contract here is that
	 * memory coming from the heaps is ready for dma, ie if it has a
	 * cached mapping that mapping has been invalidated
	 */
	for_each_sg(buffer->sg_table->sgl, sg, buffer->sg_table->nents, i) {
		sg_dma_address(sg) = sg_phys(sg);
		sg_dma_len(sg) = sg->length;
	}
#endif
	mutex_lock(&dev->buffer_lock);
	ion_buffer_add(dev, buffer);
	mutex_unlock(&dev->buffer_lock);
	return buffer;

err1:
	heap->ops->free(buffer);
err2:
	kfree(buffer);
	return ERR_PTR(ret);
}

void ion_buffer_destroy(struct ion_buffer *buffer)
{
	if (buffer->kmap_cnt > 0) {
		pr_warn_once("%s: buffer still mapped in the kernel\n",
			     __func__);
		buffer->heap->ops->unmap_kernel(buffer->heap, buffer);
	}
	buffer->heap->ops->free(buffer);
	vfree(buffer->pages);
	kfree(buffer);
}

static void _ion_buffer_destroy(struct kref *kref)
{
	struct ion_buffer *buffer = container_of(kref, struct ion_buffer, ref);
	struct ion_heap *heap = buffer->heap;
	struct ion_device *dev = buffer->dev;

	mutex_lock(&dev->buffer_lock);
	rb_erase(&buffer->node, &dev->buffers);
	mutex_unlock(&dev->buffer_lock);

	if (heap->flags & ION_HEAP_FLAG_DEFER_FREE)
		ion_heap_freelist_add(heap, buffer);
	else
		ion_buffer_destroy(buffer);
}

static void ion_buffer_get(struct ion_buffer *buffer)
{
	kref_get(&buffer->ref);
}

static int ion_buffer_put(struct ion_buffer *buffer)
{
	return kref_put(&buffer->ref, _ion_buffer_destroy);
}

static void ion_buffer_add_to_handle(struct ion_buffer *buffer)
{
	mutex_lock(&buffer->lock);
	buffer->handle_count++;
	mutex_unlock(&buffer->lock);
}

static void ion_buffer_remove_from_handle(struct ion_buffer *buffer)
{
	/*
	 * when a buffer is removed from a handle, if it is not in
	 * any other handles, copy the taskcomm and the pid of the
	 * process it's being removed from into the buffer.  At this
	 * point there will be no way to track what processes this buffer is
	 * being used by, it only exists as a dma_buf file descriptor.
	 * The taskcomm and pid can provide a debug hint as to where this fd
	 * is in the system
	 */
	mutex_lock(&buffer->lock);
	buffer->handle_count--;
	BUG_ON(buffer->handle_count < 0);
	if (!buffer->handle_count) {
		struct task_struct *task;

		task = current->group_leader;
		get_task_comm(buffer->task_comm, task);
		buffer->pid = task_pid_nr(task);
	}
	mutex_unlock(&buffer->lock);
}

static struct ion_handle *ion_handle_create(struct ion_client *client,
					    struct ion_buffer *buffer)
{
	struct ion_handle *handle;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle) {
		IONMSG("%s kzalloc failed handle is null.\n", __func__);
		return ERR_PTR(-ENOMEM);
	}
	kref_init(&handle->ref);
	RB_CLEAR_NODE(&handle->node);
	handle->client = client;
	ion_buffer_get(buffer);
	ion_buffer_add_to_handle(buffer);
	handle->buffer = buffer;

	return handle;
}

static void ion_handle_kmap_put(struct ion_handle *);

static void ion_handle_destroy(struct kref *kref)
{
	struct ion_handle *handle = container_of(kref, struct ion_handle, ref);
	struct ion_client *client = handle->client;
	struct ion_buffer *buffer = handle->buffer;

	mutex_lock(&buffer->lock);
	while (handle->kmap_cnt)
		ion_handle_kmap_put(handle);
	mutex_unlock(&buffer->lock);

	idr_remove(&client->idr, handle->id);
	if (!RB_EMPTY_NODE(&handle->node))
		rb_erase(&handle->node, &client->handles);

	ion_buffer_remove_from_handle(buffer);
	ion_buffer_put(buffer);

	handle->buffer = NULL;
	handle->client = NULL;

	kfree(handle);
}

static void ion_handle_get(struct ion_handle *handle)
{
	kref_get(&handle->ref);
}

/* Must hold the client lock */
static struct ion_handle *
ion_handle_get_check_overflow(struct ion_handle *handle)
{
	if (atomic_read(&handle->ref.refcount) + 1 == 0)
		return ERR_PTR(-EOVERFLOW);
	ion_handle_get(handle);
	return handle;
}

int ion_handle_put_nolock(struct ion_handle *handle)
{
	return kref_put(&handle->ref, ion_handle_destroy);
}

int ion_handle_put(struct ion_handle *handle)
{
	struct ion_client *client = handle->client;
	int ret;

	mutex_lock(&client->lock);
	ret = ion_handle_put_nolock(handle);
	mutex_unlock(&client->lock);

	return ret;
}

/* Must hold the client lock */
static void user_ion_handle_get(struct ion_handle *handle)
{
	if (handle->user_ref_count++ == 0)
		kref_get(&handle->ref);
}

/* Must hold the client lock */
static struct ion_handle *
user_ion_handle_get_check_overflow(struct ion_handle *handle)
{
	if (handle->user_ref_count + 1 == 0)
		return ERR_PTR(-EOVERFLOW);
	user_ion_handle_get(handle);
	return handle;
}

/* passes a kref to the user ref count.
 * We know we're holding a kref to the object before and
 * after this call, so no need to reverify handle.
 */
struct ion_handle *pass_to_user(struct ion_handle *handle)
{
	struct ion_client *client = handle->client;
	struct ion_handle *ret;

	mutex_lock(&client->lock);
	ret = user_ion_handle_get_check_overflow(handle);
	ion_handle_put_nolock(handle);
	mutex_unlock(&client->lock);
	return ret;
}

/* Must hold the client lock */
static int user_ion_handle_put_nolock(struct ion_handle *handle)
{
	int ret = 0;

	if (--handle->user_ref_count == 0)
		ret = ion_handle_put_nolock(handle);

	return ret;
}

static struct ion_handle *ion_handle_lookup(struct ion_client *client,
					    struct ion_buffer *buffer)
{
	struct rb_node *n = client->handles.rb_node;

	while (n) {
		struct ion_handle *entry = rb_entry(n, struct ion_handle, node);

		if (buffer < entry->buffer)
			n = n->rb_left;
		else if (buffer > entry->buffer)
			n = n->rb_right;
		else
			return entry;
	}
	return ERR_PTR(-EINVAL);
}

struct ion_handle *ion_handle_get_by_id_nolock(struct ion_client *client,
					       int id)
{
	struct ion_handle *handle;

	handle = idr_find(&client->idr, id);
	if (handle)
		return ion_handle_get_check_overflow(handle);

	IONDBG("%s: can't get handle by id:%d\n", __func__, id);
	return ERR_PTR(-EINVAL);
}

static bool ion_handle_validate(struct ion_client *client,
				struct ion_handle *handle)
{
	WARN_ON(!mutex_is_locked(&client->lock));
	return idr_find(&client->idr, handle->id) == handle;
}

static int ion_handle_add(struct ion_client *client, struct ion_handle *handle)
{
	int id;
	struct rb_node **p = &client->handles.rb_node;
	struct rb_node *parent = NULL;
	struct ion_handle *entry;

	id = idr_alloc(&client->idr, handle, 1, 0, GFP_KERNEL);
	if (id < 0) {
		IONMSG("%s idr_alloc failed id = %d.\n", __func__, id);
		return id;
	}

	handle->id = id;

	while (*p) {
		parent = *p;
		entry = rb_entry(parent, struct ion_handle, node);

		if (handle->buffer < entry->buffer)
			p = &(*p)->rb_left;
		else if (handle->buffer > entry->buffer)
			p = &(*p)->rb_right;
		else
			WARN(1, "%s: buffer already found.", __func__);
	}

	rb_link_node(&handle->node, parent, p);
	rb_insert_color(&handle->node, &client->handles);

	return 0;
}

struct ion_handle *ion_alloc(struct ion_client *client, size_t len,
			     size_t align, unsigned int heap_id_mask,
			     unsigned int flags)
{
	struct ion_handle *handle;
	struct ion_device *dev = client->dev;
	struct ion_buffer *buffer = NULL;
	struct ion_heap *heap;
	int ret;
	unsigned long long start, end;
	unsigned int heap_mask = ~0;

	pr_debug("%s: len %zu align %zu heap_id_mask %u flags %x\n", __func__,
		 len, align, heap_id_mask, flags);

	/* For some case(C2 audio decoder), it can not set heap id in AOSP,
	 * so mtk ion will set this heap to ion_mm_heap.
	 */
	if (heap_id_mask == heap_mask)
		heap_id_mask = ION_HEAP_MULTIMEDIA_MASK;

	/*
	 * traverse the list of heaps available in this system in priority
	 * order.  If the heap type is supported by the client, and matches the
	 * request of the caller allocate from it.  Repeat until allocate has
	 * succeeded or all heaps have been tried
	 */
	len = PAGE_ALIGN(len);
	if (!len) {
		IONMSG("%s len cannot be zero.\n", __func__);
		return ERR_PTR(-EINVAL);
	}

    /* add by k.zhang for sgtable_init KE bug */
	if ((len > 1024 * 1024 * 1024)) {
		IONMSG("%s error: size (%zu) is more than 1G !!\n",
		       __func__, len);
		return ERR_PTR(-EINVAL);
	}
	/*avoid camelcase, will modify in a letter*/
	mmprofile_log_ex(ion_mmp_events[PROFILE_ALLOC], MMPROFILE_FLAG_START,
			 (unsigned long)client, len);
	start = sched_clock();

	down_read(&dev->lock);
	plist_for_each_entry(heap, &dev->heaps, node) {
		/* if the caller didn't specify this heap id */
		if (!((1 << heap->id) & heap_id_mask))
			continue;
		buffer = ion_buffer_create(heap, dev, len, align, flags);
		if (!IS_ERR(buffer))
			break;
	}
	up_read(&dev->lock);

	if (!buffer) {
		IONMSG("%s buffer is null.\n", __func__);
		return ERR_PTR(-ENODEV);
	}

	if (IS_ERR(buffer)) {
		IONMSG("%s buffer is error 0x%p, heap_mask: 0x%x.\n",
		       __func__, buffer, heap_id_mask);
		return ERR_CAST(buffer);
	}

	handle = ion_handle_create(client, buffer);

	/*
	 * ion_buffer_create will create a buffer with a ref_cnt of 1,
	 * and ion_handle_create will take a second reference, drop one here
	 */
	ion_buffer_put(buffer);

	if (IS_ERR(handle)) {
		IONMSG("%s handle is error 0x%p.\n", __func__, handle);
		return handle;
	}

	mutex_lock(&client->lock);
	ret = ion_handle_add(client, handle);
	mutex_unlock(&client->lock);
	if (ret) {
		ion_handle_put(handle);
		handle = ERR_PTR(ret);
		IONMSG("%s ion handle add failed %d.\n", __func__, ret);
	}

	end = sched_clock();

	if (end - start > 100000000ULL) {/* unit is ns */
		IONMSG("warn: ion alloc buffer size: %zu time: %lld ns\n",
		       buffer->size, end - start);
	}
	/*avoid camelcase, will modify in a letter*/
	mmprofile_log_ex(ion_mmp_events[PROFILE_ALLOC], MMPROFILE_FLAG_END,
			 (unsigned long)client, (unsigned long)handle);

#ifdef CONFIG_MTK_ION
	ion_history_count_kick(true, len);
#endif

	handle->dbg.user_ts = end;
	do_div(handle->dbg.user_ts, 1000000);
	memcpy(buffer->alloc_dbg, client->dbg_name, ION_MM_DBG_NAME_LEN);

	return handle;
}
EXPORT_SYMBOL(ion_alloc);

void user_ion_free_nolock(struct ion_client *client, struct ion_handle *handle)
{
	bool valid_handle;

	WARN_ON(client != handle->client);
	valid_handle = ion_handle_validate(client, handle);
	if (!valid_handle) {
		WARN(1, "%s: invalid handle passed to free.\n", __func__);
		return;
	}
	if (handle->user_ref_count <= 0) {
		WARN(1, "%s: User does not have access!\n", __func__);
		return;
	}
	user_ion_handle_put_nolock(handle);
}

void ion_free_nolock(struct ion_client *client,
		     struct ion_handle *handle)
{
	WARN_ON(client != handle->client);

	if (!ion_handle_validate(client, handle)) {
		WARN(1, "%s: invalid handle passed to free.\n", __func__);
		return;
	}
	ion_handle_put_nolock(handle);
#ifdef CONFIG_MTK_ION
	ion_history_count_kick(false, 0);
#endif
}

void ion_free(struct ion_client *client, struct ion_handle *handle)
{
	BUG_ON(client != handle->client);

	mutex_lock(&client->lock);
	ion_free_nolock(client, handle);
	mutex_unlock(&client->lock);
	mmprofile_log_ex(ion_mmp_events[PROFILE_FREE], MMPROFILE_FLAG_PULSE,
			 (unsigned long)client, (unsigned long)handle);
}
EXPORT_SYMBOL(ion_free);

static void *ion_buffer_kmap_get(struct ion_buffer *buffer)
{
	void *vaddr;

	if (buffer->kmap_cnt) {
		buffer->kmap_cnt++;
		return buffer->vaddr;
	}
	vaddr = buffer->heap->ops->map_kernel(buffer->heap, buffer);
	if (WARN_ONCE(vaddr == NULL,
		      "heap->ops->map_kernel should return ERR_PTR on error"))
		return ERR_PTR(-EINVAL);
	if (IS_ERR(vaddr)) {
		IONMSG("%s map kernel is failed addr = 0x%p.\n",
		       __func__, vaddr);
		return vaddr;
	}
	buffer->vaddr = vaddr;
	buffer->kmap_cnt++;
	return vaddr;
}

static void *ion_handle_kmap_get(struct ion_handle *handle)
{
	struct ion_buffer *buffer = handle->buffer;
	void *vaddr;

	if (handle->kmap_cnt) {
		handle->kmap_cnt++;
		return buffer->vaddr;
	}
	vaddr = ion_buffer_kmap_get(buffer);
	if (IS_ERR(vaddr)) {
		IONMSG("%s vadd is error 0x%p.\n", __func__, vaddr);
		return vaddr;
	}
	handle->kmap_cnt++;
	return vaddr;
}

static void ion_buffer_kmap_put(struct ion_buffer *buffer)
{
	buffer->kmap_cnt--;
	if (!buffer->kmap_cnt) {
		/*avoid camelcase, will modify in a letter*/
		mmprofile_log_ex(ion_mmp_events[PROFILE_UNMAP_KERNEL],
				 MMPROFILE_FLAG_START,
				 buffer->size, buffer->kmap_cnt);
		buffer->heap->ops->unmap_kernel(buffer->heap, buffer);
		mmprofile_log_ex(ion_mmp_events[PROFILE_UNMAP_KERNEL],
				 MMPROFILE_FLAG_END,
				 buffer->size, buffer->kmap_cnt);
		buffer->vaddr = NULL;
	}
}

static void ion_handle_kmap_put(struct ion_handle *handle)
{
	struct ion_buffer *buffer = handle->buffer;

	if (!handle->kmap_cnt) {
		WARN(1, "%s: Double unmap detected! bailing...\n", __func__);
		return;
	}
	handle->kmap_cnt--;
	if (!handle->kmap_cnt)
		ion_buffer_kmap_put(buffer);
}

void *ion_map_kernel(struct ion_client *client, struct ion_handle *handle)
{
	struct ion_buffer *buffer;
	void *vaddr;

	mutex_lock(&client->lock);
	if (!ion_handle_validate(client, handle)) {
		pr_err("%s: invalid handle passed to map_kernel.\n",
		       __func__);
		mutex_unlock(&client->lock);
		return ERR_PTR(-EINVAL);
	}

	buffer = handle->buffer;

	if (!handle->buffer->heap->ops->map_kernel) {
		pr_err("%s: map_kernel is not implemented by this heap.\n",
		       __func__);
		mutex_unlock(&client->lock);
		return ERR_PTR(-ENODEV);
	}

	mutex_lock(&buffer->lock);
	vaddr = ion_handle_kmap_get(handle);
	mutex_unlock(&buffer->lock);
	mutex_unlock(&client->lock);
	return vaddr;
}
EXPORT_SYMBOL(ion_map_kernel);

void ion_unmap_kernel(struct ion_client *client, struct ion_handle *handle)
{
	struct ion_buffer *buffer;

	mutex_lock(&client->lock);
	buffer = handle->buffer;
	mutex_lock(&buffer->lock);
	ion_handle_kmap_put(handle);
	mutex_unlock(&buffer->lock);
	mutex_unlock(&client->lock);
}
EXPORT_SYMBOL(ion_unmap_kernel);

static int ion_client_validate(struct ion_device *dev,
			       struct ion_client *client)
{
	struct rb_node *n;

	for (n = rb_first(&dev->clients); n; n = rb_next(n)) {
		struct ion_client *valid_client = rb_entry(n,
							   struct ion_client,
							   node);

		if (client == valid_client)
			return 1;
	}

	return 0;
}

static int ion_debug_client_show(struct seq_file *s, void *unused)
{
	struct ion_client *client = s->private;
	struct ion_device *dev = g_ion_device;
	struct rb_node *n;
	/*size_t sizes[ION_NUM_HEAP_IDS] = {0};*/
	/*const char *names[ION_NUM_HEAP_IDS] = {NULL};*/
	size_t *sizes;
	const char **names;
	int i;

	sizes = kcalloc(ION_NUM_HEAP_IDS, sizeof(size_t), GFP_ATOMIC);

	if (!sizes)
		return -ENOMEM;

	names = kcalloc(ION_NUM_HEAP_IDS, sizeof(char *), GFP_ATOMIC);

	if (!names) {
		kfree(sizes);
		return -ENOMEM;
	}

	if (!down_read_trylock(&dev->lock)) {
		IONMSG("%s get lock fail\n", __func__);
		kfree(sizes);
		kfree(names);
		return 0;
	}
	if (!ion_client_validate(dev, client)) {
		seq_printf(s, "ion_client 0x%pK dead, can't dump its buffers\n",
			   client);
		IONMSG("%s:client invalid\n", __func__);
		up_read(&dev->lock);
		kfree(sizes);
		kfree(names);
		return 0;
	}

	seq_printf(s, "%16.s %8.s %8.s %8.s %8.s %8.s\n",
		   "heap_name", "pid", "size",
		   "handle_count", "handle", "buffer");

	mutex_lock(&client->lock);
	for (n = rb_first(&client->handles); n; n = rb_next(n)) {
		struct ion_handle *handle = rb_entry(n, struct ion_handle,
						     node);
		struct ion_buffer *buffer = handle->buffer;
		unsigned int id = buffer->heap->id;

		if (!names[id])
			names[id] = buffer->heap->name;
		sizes[id] += buffer->size;

		seq_printf(s, "%16.s %3d %8zu %3d %p %p.\n", buffer->heap->name,
			   client->pid, buffer->size,
			   buffer->handle_count, handle, buffer);
	}
	mutex_unlock(&client->lock);
	up_read(&dev->lock);

	seq_puts(s, "----------------------------------------------------\n");

	seq_printf(s, "%16.16s: %16.16s\n", "heap_name", "size_in_bytes");
	for (i = 0; i < ION_NUM_HEAP_IDS; i++) {
		if (!names[i])
			continue;
		seq_printf(s, "%16.16s: %16zu\n", names[i], sizes[i]);
	}

	kfree(sizes);
	kfree(names);

	return 0;
}

static int ion_debug_client_open(struct inode *inode, struct file *file)
{
	return single_open(file, ion_debug_client_show, inode->i_private);
}

static const struct file_operations debug_client_fops = {
	.open = ion_debug_client_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ion_get_client_serial(const struct rb_root *root,
				 const unsigned char *name)
{
	int serial = -1;
	struct rb_node *node;

	for (node = rb_first(root); node; node = rb_next(node)) {
		struct ion_client *client = rb_entry(node, struct ion_client,
						     node);

		if (strcmp(client->name, name))
			continue;
		serial = max(serial, client->display_serial);
	}
	return serial + 1;
}

struct ion_client *ion_client_create(struct ion_device *dev,
				     const char *name)
{
	struct ion_client *client;
	struct task_struct *task;
	struct rb_node **p;
	struct rb_node *parent = NULL;
	struct ion_client *entry;
	pid_t pid;

	if (!name) {
		pr_err("%s: Name cannot be null\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	get_task_struct(current->group_leader);
	task_lock(current->group_leader);
	pid = task_pid_nr(current->group_leader);
	/*
	 * don't bother to store task struct for kernel threads,
	 * they can't be killed anyway
	 */
	if (current->group_leader->flags & PF_KTHREAD) {
		put_task_struct(current->group_leader);
		task = NULL;
	} else {
		task = current->group_leader;
	}
	task_unlock(current->group_leader);

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		goto err_put_task_struct;

	client->dev = dev;
	client->handles = RB_ROOT;
	idr_init(&client->idr);
	mutex_init(&client->lock);
	client->task = task;
	client->pid = pid;
	client->name = kstrdup(name, GFP_KERNEL);
	if (!client->name)
		goto err_free_client;

	down_write(&dev->lock);
	client->display_serial = ion_get_client_serial(&dev->clients, name);
	client->display_name = kasprintf(
		GFP_KERNEL, "%s-%d", name, client->display_serial);
	if (!client->display_name) {
		up_write(&dev->lock);
		goto err_free_client_name;
	}
	p = &dev->clients.rb_node;
	while (*p) {
		parent = *p;
		entry = rb_entry(parent, struct ion_client, node);

		if (client < entry)
			p = &(*p)->rb_left;
		else if (client > entry)
			p = &(*p)->rb_right;
	}
	rb_link_node(&client->node, parent, p);
	rb_insert_color(&client->node, &dev->clients);

	client->debug_root = debugfs_create_file(client->display_name, 0664,
						 dev->clients_debug_root,
						 client, &debug_client_fops);
	if (!client->debug_root) {
		char buf[256], *path;

		path = dentry_path(dev->clients_debug_root, buf, 256);
		pr_err("Failed to create client debugfs at %s/%s\n",
		       path, client->display_name);
	}

	up_write(&dev->lock);

	return client;

err_free_client_name:
	kfree(client->name);
err_free_client:
	kfree(client);
err_put_task_struct:
	if (task)
		put_task_struct(current->group_leader);
	return ERR_PTR(-ENOMEM);
}
EXPORT_SYMBOL(ion_client_create);

void ion_client_destroy(struct ion_client *client)
{
	struct ion_device *dev = client->dev;
	struct rb_node *n;

	pr_debug("%s: %d\n", __func__, __LINE__);

	while ((n = rb_first(&client->handles))) {
		struct ion_handle *handle = rb_entry(n, struct ion_handle,
						     node);

		mutex_lock(&client->lock);
		IONMSG("%s:hdl=%p,buf=%p,ref=%d,kmp=%d,clnt=%s,dbg=%s\n",
		       __func__, handle, handle->buffer,
		       atomic_read(&handle->buffer->ref.refcount),
		       handle->buffer->kmap_cnt,
		       client->name ? client->name : NULL,
		       client->dbg_name);
		ion_handle_destroy(&handle->ref);
		mutex_unlock(&client->lock);
	}

	idr_destroy(&client->idr);

	down_write(&dev->lock);
	if (client->task)
		put_task_struct(client->task);
	rb_erase(&client->node, &dev->clients);
	debugfs_remove_recursive(client->debug_root);
	up_write(&dev->lock);

	kfree(client->display_name);
	kfree(client->name);
	kfree(client);
}
EXPORT_SYMBOL(ion_client_destroy);

static void ion_buffer_sync_for_device(struct ion_buffer *buffer,
				       struct device *dev,
				       enum dma_data_direction direction);

static struct sg_table *ion_map_dma_buf(struct dma_buf_attachment *attachment,
					enum dma_data_direction direction)
{
	struct dma_buf *dmabuf = attachment->dmabuf;
	struct ion_buffer *buffer = dmabuf->priv;

	ion_buffer_sync_for_device(buffer, attachment->dev, direction);
	return buffer->sg_table;
}

static void ion_unmap_dma_buf(struct dma_buf_attachment *attachment,
			      struct sg_table *table,
			      enum dma_data_direction direction)
{
}

void ion_pages_sync_for_device(struct device *dev, struct page *page,
			       size_t size, enum dma_data_direction dir)
{
	struct scatterlist sg;

	sg_init_table(&sg, 1);
	sg_set_page(&sg, page, size, 0);
	/*
	 * This is not correct - sg_dma_address need a dma_addr_t that is valid
	 * for the targeted device, but this works on the currently targeted
	 * hardware.
	 */
	sg_dma_address(&sg) = page_to_phys(page);
	dma_sync_sg_for_device(dev, &sg, 1, dir);
}

struct ion_vma_list {
	struct list_head list;
	struct vm_area_struct *vma;
};

static void ion_buffer_sync_for_device(struct ion_buffer *buffer,
				       struct device *dev,
				       enum dma_data_direction dir)
{
	struct ion_vma_list *vma_list;
	int pages = PAGE_ALIGN(buffer->size) / PAGE_SIZE;
	int i;

	if (!ion_buffer_fault_user_mappings(buffer))
		return;

	mutex_lock(&buffer->lock);
	for (i = 0; i < pages; i++) {
		struct page *page = buffer->pages[i];

		if (ion_buffer_page_is_dirty(page))
			ion_pages_sync_for_device(dev, ion_buffer_page(page),
						  PAGE_SIZE, dir);

		ion_buffer_page_clean(buffer->pages + i);
	}
	list_for_each_entry(vma_list, &buffer->vmas, list) {
		struct vm_area_struct *vma = vma_list->vma;

		zap_page_range(vma, vma->vm_start, vma->vm_end - vma->vm_start,
			       NULL);
	}
	mutex_unlock(&buffer->lock);
}

static int ion_vm_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct dma_buf *dmabuf = vma->vm_private_data;
	struct ion_buffer *buffer =  dmabuf->priv;
	unsigned long pfn;
	int ret;

	mutex_lock(&buffer->lock);
	ion_buffer_page_dirty(buffer->pages + vmf->pgoff);
	BUG_ON(!buffer->pages || !buffer->pages[vmf->pgoff]);

	pfn = page_to_pfn(ion_buffer_page(buffer->pages[vmf->pgoff]));
	ret = vm_insert_pfn(vma, (unsigned long)vmf->virtual_address, pfn);
	mutex_unlock(&buffer->lock);
	if (ret) {
		IONMSG("%s vm insert pfn fail,vma =0x%p,addr =0x%p,pfn =%lu\n",
		       __func__, vma, vmf->virtual_address, pfn);
		return VM_FAULT_ERROR;
	}

	return VM_FAULT_NOPAGE;
}

static void ion_vm_open(struct vm_area_struct *vma)
{
	struct dma_buf *dmabuf = vma->vm_private_data;
	struct ion_buffer *buffer =  dmabuf->priv;
	struct ion_vma_list *vma_list;

	vma_list = kmalloc(sizeof(*vma_list), GFP_KERNEL);
	if (!vma_list) {
		IONMSG("%s kmalloc failed, vma_list is null.\n", __func__);
		return;
	}
	vma_list->vma = vma;
	mutex_lock(&buffer->lock);
	list_add(&vma_list->list, &buffer->vmas);
	mutex_unlock(&buffer->lock);
}

static void ion_vm_close(struct vm_area_struct *vma)
{
	struct dma_buf *dmabuf = vma->vm_private_data;
	struct ion_buffer *buffer =  dmabuf->priv;
	struct ion_vma_list *vma_list, *tmp;

	mutex_lock(&buffer->lock);
	list_for_each_entry_safe(vma_list, tmp, &buffer->vmas, list) {
		if (vma_list->vma != vma)
			continue;
		list_del(&vma_list->list);
		kfree(vma_list);
		break;
	}
	mutex_unlock(&buffer->lock);
}

static const struct vm_operations_struct ion_vma_ops = {
	.open = ion_vm_open,
	.close = ion_vm_close,
	.fault = ion_vm_fault,
};

static int ion_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct ion_buffer *buffer = dmabuf->priv;
	int ret = 0;

	mmprofile_log_ex(ion_mmp_events[PROFILE_MAP_USER],
			 MMPROFILE_FLAG_START, buffer->size, vma->vm_start);

	if (!buffer->heap->ops->map_user) {
		pr_err("%s: heap does not define a method for map to native\n",
		       __func__);
		return -EINVAL;
	}

	vma->vm_private_data = dmabuf;
	if (ion_buffer_fault_user_mappings(buffer)) {
		vma->vm_flags |= VM_IO | VM_PFNMAP | VM_DONTEXPAND |
							VM_DONTDUMP;
		vma->vm_ops = &ion_vma_ops;
		ion_vm_open(vma);
		return 0;
	}

	if (!(buffer->flags & ION_FLAG_CACHED))
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	mutex_lock(&buffer->lock);
	/* now map it to userspace */
	ret = buffer->heap->ops->map_user(buffer->heap, buffer, vma);
	mutex_unlock(&buffer->lock);

	if (ret)
		pr_err("%s: failure mapping buffer to userspace\n",
		       __func__);
	mmprofile_log_ex(ion_mmp_events[PROFILE_MAP_USER],
			 MMPROFILE_FLAG_END, buffer->size, vma->vm_start);

	return ret;
}

static void ion_dma_buf_release(struct dma_buf *dmabuf)
{
	struct ion_buffer *buffer = dmabuf->priv;

	ion_buffer_put(buffer);
}

static void *ion_dma_buf_kmap(struct dma_buf *dmabuf, unsigned long offset)
{
	struct ion_buffer *buffer = dmabuf->priv;

	return buffer->vaddr + offset * PAGE_SIZE;
}

static void ion_dma_buf_kunmap(struct dma_buf *dmabuf, unsigned long offset,
			       void *ptr)
{
}

static int ion_dma_buf_begin_cpu_access(struct dma_buf *dmabuf,
					enum dma_data_direction direction)
{
	struct ion_buffer *buffer = dmabuf->priv;
	void *vaddr;

	if (!buffer->heap->ops->map_kernel) {
		pr_err("%s: map kernel is not implemented by this heap.\n",
		       __func__);
		return -ENODEV;
	}

	mutex_lock(&buffer->lock);
	vaddr = ion_buffer_kmap_get(buffer);
	mutex_unlock(&buffer->lock);
	return PTR_ERR_OR_ZERO(vaddr);
}

static int ion_dma_buf_end_cpu_access(struct dma_buf *dmabuf,
				      enum dma_data_direction direction)
{
	struct ion_buffer *buffer = dmabuf->priv;

	mutex_lock(&buffer->lock);
	ion_buffer_kmap_put(buffer);
	mutex_unlock(&buffer->lock);

	return 0;
}

static struct dma_buf_ops dma_buf_ops = {
	.map_dma_buf = ion_map_dma_buf,
	.unmap_dma_buf = ion_unmap_dma_buf,
	.mmap = ion_mmap,
	.release = ion_dma_buf_release,
	.begin_cpu_access = ion_dma_buf_begin_cpu_access,
	.end_cpu_access = ion_dma_buf_end_cpu_access,
	.kmap_atomic = ion_dma_buf_kmap,
	.kunmap_atomic = ion_dma_buf_kunmap,
	.kmap = ion_dma_buf_kmap,
	.kunmap = ion_dma_buf_kunmap,
};

static struct dma_buf *__ion_share_dma_buf(struct ion_client *client,
					   struct ion_handle *handle,
					   bool lock_client)
{
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct ion_buffer *buffer;
	struct dma_buf *dmabuf;
	bool valid_handle;

	if (lock_client)
		mutex_lock(&client->lock);
	valid_handle = ion_handle_validate(client, handle);
	if (!valid_handle) {
		WARN(1, "%s: invalid handle passed to share.\n", __func__);
		if (lock_client)
			mutex_unlock(&client->lock);
		return ERR_PTR(-EINVAL);
	}
	buffer = handle->buffer;
	ion_buffer_get(buffer);
	if (lock_client)
		mutex_unlock(&client->lock);

	exp_info.ops = &dma_buf_ops;
	exp_info.size = buffer->size;
	exp_info.flags = O_RDWR;
	exp_info.priv = buffer;

	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(dmabuf)) {
		IONMSG("%s dma buf export failed dmabuf is error 0x%p.\n",
		       __func__, dmabuf);
		ion_buffer_put(buffer);
		return dmabuf;
	}

	return dmabuf;
}

struct dma_buf *ion_share_dma_buf(struct ion_client *client,
				  struct ion_handle *handle)
{
	return __ion_share_dma_buf(client, handle, true);
}

EXPORT_SYMBOL(ion_share_dma_buf);

static int __ion_share_dma_buf_fd(struct ion_client *client,
				  struct ion_handle *handle, bool lock_client)
{
	struct dma_buf *dmabuf;
	int fd;

	mmprofile_log_ex(ion_mmp_events[PROFILE_SHARE], MMPROFILE_FLAG_START,
			 (unsigned long)client, (unsigned long)handle);
	dmabuf = __ion_share_dma_buf(client, handle, lock_client);
	if (IS_ERR(dmabuf)) {
		IONMSG("%s dmabuf is err 0x%p.\n", __func__, dmabuf);
		return PTR_ERR(dmabuf);
	}

	fd = dma_buf_fd(dmabuf, O_CLOEXEC);
	mmprofile_log_ex(ion_mmp_events[PROFILE_SHARE],
			 MMPROFILE_FLAG_END, (unsigned long)client, fd);
	if (fd < 0) {
		IONMSG("%s dma_buf_fd failed %d.\n", __func__, fd);
		dma_buf_put(dmabuf);
	}
	handle->dbg.fd = fd;
	return fd;
}

int ion_share_dma_buf_fd(struct ion_client *client, struct ion_handle *handle)
{
	return __ion_share_dma_buf_fd(client, handle, true);
}

EXPORT_SYMBOL(ion_share_dma_buf_fd);

int ion_share_dma_buf_fd_nolock(struct ion_client *client,
				struct ion_handle *handle)
{
	return __ion_share_dma_buf_fd(client, handle, false);
}

struct ion_handle *ion_import_dma_buf(struct ion_client *client,
				      struct dma_buf *dmabuf)
{
	struct ion_buffer *buffer;
	struct ion_handle *handle;
	int ret;

	/* if this memory came from ion */

	if (dmabuf->ops != &dma_buf_ops) {
		pr_err("%s: can not import dmabuf from another exporter\n",
		       __func__);
		return ERR_PTR(-EINVAL);
	}
	buffer = dmabuf->priv;

	mutex_lock(&client->lock);
	/* if a handle exists for this buffer just take a reference to it */
	handle = ion_handle_lookup(client, buffer);
	if (!IS_ERR(handle)) {
		handle = ion_handle_get_check_overflow(handle);
		mutex_unlock(&client->lock);
		goto end;
	}

	handle = ion_handle_create(client, buffer);
	if (IS_ERR(handle)) {
		mutex_unlock(&client->lock);
		IONMSG("%s handle is error 0x%p.\n", __func__, handle);
		goto end;
	}

	ret = ion_handle_add(client, handle);
	mutex_unlock(&client->lock);
	if (ret) {
		ion_handle_put(handle);
		handle = ERR_PTR(ret);
		IONMSG("ion_import: ion_handle_add fail %d\n", ret);
	}

end:
	return handle;
}
EXPORT_SYMBOL(ion_import_dma_buf);

struct ion_handle *ion_import_dma_buf_fd(struct ion_client *client, int fd)
{
	struct dma_buf *dmabuf;
	struct ion_handle *handle;

	mmprofile_log_ex(ion_mmp_events[PROFILE_IMPORT], MMPROFILE_FLAG_START,
			 (unsigned long)client, fd);

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf)) {
		IONMSG("%s dma_buf_get fail fd=%d ret=0x%p\n",
		       __func__, fd, dmabuf);
		return ERR_CAST(dmabuf);
	}

	handle = ion_import_dma_buf(client, dmabuf);
	dma_buf_put(dmabuf);

	mmprofile_log_ex(ion_mmp_events[PROFILE_IMPORT], MMPROFILE_FLAG_END,
			 (unsigned long)client,
			 (unsigned long)handle);
	if (!IS_ERR(handle)) {
		handle->dbg.fd = fd;
		handle->dbg.user_ts = sched_clock();
		do_div(handle->dbg.user_ts, 1000000);
	}
	return handle;
}
EXPORT_SYMBOL(ion_import_dma_buf_fd);

int ion_sync_for_device(struct ion_client *client, int fd)
{
	struct dma_buf *dmabuf;
	struct ion_buffer *buffer;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf)) {
		IONMSG("%s dma_buf_get failed dmabuf is err %d, 0x%p.\n",
		       __func__, fd, dmabuf);
		return PTR_ERR(dmabuf);
	}

	/* if this memory came from ion */
	if (dmabuf->ops != &dma_buf_ops) {
		pr_err("%s: can not sync dmabuf from another exporter\n",
		       __func__);
		dma_buf_put(dmabuf);
		return -EINVAL;
	}
	buffer = dmabuf->priv;

#ifdef CONFIG_MTK_ION
	if (buffer->heap->type != (int)ION_HEAP_TYPE_FB &&
	    buffer->heap->type != (int)ION_HEAP_TYPE_MULTIMEDIA_SEC)
		dma_sync_sg_for_device(g_ion_device->dev.this_device,
				       buffer->sg_table->sgl,
				       buffer->sg_table->nents,
				       DMA_BIDIRECTIONAL);
	else
		IONMSG("%s: can not support heap %s type %d to sync\n",
		       __func__, buffer->heap->name, buffer->heap->type);
#endif

	dma_buf_put(dmabuf);
	return 0;
}

int ion_query_heaps(struct ion_client *client, struct ion_heap_query *query)
{
	struct ion_device *dev = client->dev;
	struct ion_heap_data __user *buffer = u64_to_user_ptr(query->heaps);
	int ret = -EINVAL, cnt = 0, max_cnt;
	struct ion_heap *heap;
	struct ion_heap_data hdata;

	memset(&hdata, 0, sizeof(hdata));

	down_read(&dev->lock);
	if (!buffer) {
		query->cnt = dev->heap_cnt;
		ret = 0;
		goto out;
	}

	if (query->cnt <= 0)
		goto out;

	max_cnt = query->cnt;

	plist_for_each_entry(heap, &dev->heaps, node) {
		strncpy(hdata.name, heap->name, MAX_HEAP_NAME);
		hdata.name[sizeof(hdata.name) - 1] = '\0';
		hdata.type = heap->type;
		hdata.heap_id = heap->id;

		if (copy_to_user(&buffer[cnt], &hdata, sizeof(hdata))) {
			ret = -EFAULT;
			goto out;
		}

		cnt++;
		if (cnt >= max_cnt)
			break;
	}

	query->cnt = cnt;
out:
	up_read(&dev->lock);
	return ret;
}

static int ion_release(struct inode *inode, struct file *file)
{
	struct ion_client *client = file->private_data;

	ion_client_destroy(client);
	return 0;
}

static int ion_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct ion_device *dev = container_of(miscdev, struct ion_device, dev);
	struct ion_client *client;
	char debug_name[64];
	unsigned long long start, end;

	snprintf(debug_name, 64, "%u", task_pid_nr(current->group_leader));
	start = sched_clock();
	client = ion_client_create(dev, debug_name);
	if (IS_ERR(client)) {
		IONMSG("%s ion client create failed 0x%p.\n", __func__, client);
		return PTR_ERR(client);
	}
	file->private_data = client;
	end = sched_clock();

	if (end - start > 10000000ULL) {/* unit is ns */
		IONMSG("warn: ion open time: %lld ns\n", end - start);
	}

	return 0;
}

static const struct file_operations ion_fops = {
	.owner          = THIS_MODULE,
	.open           = ion_open,
	.release        = ion_release,
	.unlocked_ioctl = ion_ioctl,
	.compat_ioctl   = compat_ion_ioctl,
};

static size_t ion_debug_heap_total(struct ion_client *client,
				   unsigned int id)
{
	size_t size = 0;
	struct rb_node *n;
	unsigned int heapid;

	mutex_lock(&client->lock);
	for (n = rb_first(&client->handles); n; n = rb_next(n)) {
		struct ion_handle *handle = rb_entry(n,
						     struct ion_handle,
						     node);
		heapid = handle->buffer->heap->id;
		if ((heapid == id) ||
		    ((id == ION_HEAP_TYPE_MULTIMEDIA) &&
		     (heapid == ION_HEAP_TYPE_MULTIMEDIA_FOR_CAMERA)))
			size += handle->buffer->size;
	}
	mutex_unlock(&client->lock);
	return size;
}

static int ion_debug_heap_show(struct seq_file *s, void *unused)
{
	struct ion_heap *heap = s->private;
	struct ion_device *dev = heap->dev;
	struct rb_node *n;
	struct ion_heap *cam_heap = NULL;
	size_t total_size = 0;
	size_t camera_total_size = 0;
	size_t total_orphaned_size = 0;
	size_t cam_orphaned_size = 0;
	unsigned long long current_ts = 0;
	unsigned int cam_id = ION_HEAP_TYPE_MULTIMEDIA_FOR_CAMERA;

	seq_printf(s, "%16.s(%16.s) %16.s %16.s %s\n",
		   "client", "dbg_name", "pid", "size", "address");
	seq_puts(s, "----------------------------------------------------\n");

	down_read(&dev->lock);

	current_ts = sched_clock();
	do_div(current_ts, 1000000);
	seq_printf(s, "time 1 %lld ms\n", current_ts);

	for (n = rb_first(&dev->clients); n; n = rb_next(n)) {
		struct ion_client *client = rb_entry(n, struct ion_client,
						     node);
		size_t size = ion_debug_heap_total(client, heap->id);

		if (!size)
			continue;
		if (client->task) {
			char task_comm[TASK_COMM_LEN];

			get_task_comm(task_comm, client->task);
			seq_printf(s, "%16.s(%16.s) %16u %16zu 0x%p\n",
				   task_comm,
				   (*client->dbg_name) ? client->
					dbg_name : client->name,
				   client->pid, size, client);
		} else {
			seq_printf(s, "%16.s(%16.s) %16u %16zu 0x%p\n",
				   client->name, "from_kernel",
				   client->pid, size, client);
		}
	}
	up_read(&dev->lock);

	seq_puts(s, "----------------------------------------------------\n");
	seq_puts(s, "orphaned allocation (info is from last known client):\n");
	mutex_lock(&dev->buffer_lock);

	current_ts = sched_clock();
	do_div(current_ts, 1000000);
	seq_printf(s, "time 2 %lld ms\n", current_ts);

	for (n = rb_first(&dev->buffers); n; n = rb_next(n)) {
		struct ion_buffer *buffer = rb_entry(n, struct ion_buffer,
						     node);

		if (buffer->heap->id != heap->id) {
			if ((heap->id == ION_HEAP_TYPE_MULTIMEDIA) &&
			    (buffer->heap->id == cam_id)) {
				cam_heap = buffer->heap;
				camera_total_size += buffer->size;
			}
			else
				continue;
		}
		total_size += buffer->size;
		if (!buffer->handle_count) {
			seq_printf(s, "%16.s %16u %16zu\n",
				   buffer->task_comm, buffer->pid,
				   buffer->size);
			total_orphaned_size += buffer->size;
			if (buffer->heap->id == cam_id)
				cam_orphaned_size +=  buffer->size;
		}
	}
	mutex_unlock(&dev->buffer_lock);
	seq_puts(s, "----------------------------------------------------\n");
	seq_printf(s, "%16.s %16zu, %16zu\n", "total orphaned, cam orphaned",
		   total_orphaned_size, cam_orphaned_size);
	seq_printf(s, "%16.s %16zu\n", "total ", total_size);
	if (heap->id == ION_HEAP_TYPE_MULTIMEDIA)
		seq_printf(s, "%16.s %16zu\n", "cam total",
			   camera_total_size);
	if (heap->flags & ION_HEAP_FLAG_DEFER_FREE)
		seq_printf(s, "%16.s %u %16zu\n",
			   "deferred free heap_id",
				heap->id, heap->free_list_size);
	if (cam_heap) {
		if (cam_heap->flags & ION_HEAP_FLAG_DEFER_FREE)
		seq_printf(s, "%16.s %u %16zu\n",
			   "cam heap deferred free heap_id",
				cam_heap->id, cam_heap->free_list_size);
	}
	seq_puts(s, "----------------------------------------------------\n");

	if (heap->debug_show)
		heap->debug_show(heap, s, unused);

	return 0;
}

static int ion_debug_heap_open(struct inode *inode, struct file *file)
{
	return single_open(file, ion_debug_heap_show, inode->i_private);
}

static const struct file_operations debug_heap_fops = {
	.open = ion_debug_heap_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int debug_shrink_set(void *data, u64 val)
{
	struct ion_heap *heap = data;
	struct shrink_control sc;
	int objs;

	sc.gfp_mask = GFP_HIGHUSER;
	sc.nr_to_scan = val;

	if (!val) {
		objs = heap->shrinker.count_objects(&heap->shrinker, &sc);
		sc.nr_to_scan = objs;
	}

	heap->shrinker.scan_objects(&heap->shrinker, &sc);
	return 0;
}

static int debug_shrink_get(void *data, u64 *val)
{
	struct ion_heap *heap = data;
	struct shrink_control sc;
	int objs;

	sc.gfp_mask = GFP_HIGHUSER;
	sc.nr_to_scan = 0;

	objs = heap->shrinker.count_objects(&heap->shrinker, &sc);
	*val = objs;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_shrink_fops, debug_shrink_get,
			debug_shrink_set, "%llu\n");

void ion_device_add_heap(struct ion_device *dev, struct ion_heap *heap)
{
	struct dentry *debug_file;

	if (!heap->ops->allocate || !heap->ops->free)
		pr_err("%s: can not add heap with invalid ops struct.\n",
		       __func__);

	spin_lock_init(&heap->free_lock);
	heap->free_list_size = 0;

	if (heap->flags & ION_HEAP_FLAG_DEFER_FREE)
		ion_heap_init_deferred_free(heap);

	if ((heap->flags & ION_HEAP_FLAG_DEFER_FREE) || heap->ops->shrink)
		ion_heap_init_shrinker(heap);

	heap->dev = dev;
	down_write(&dev->lock);
	/*
	 * use negative heap->id to reverse the priority -- when traversing
	 * the list later attempt higher id numbers first
	 */
	plist_node_init(&heap->node, -heap->id);
	plist_add(&heap->node, &dev->heaps);
	debug_file = debugfs_create_file(heap->name, 0664,
					 dev->heaps_debug_root, heap,
					 &debug_heap_fops);

	if (!debug_file) {
		char buf[256], *path;

		path = dentry_path(dev->heaps_debug_root, buf, 256);
		pr_err("Failed to create heap debugfs at %s/%s\n",
		       path, heap->name);
	}

	if (heap->shrinker.count_objects && heap->shrinker.scan_objects) {
		char debug_name[64];

		snprintf(debug_name, 64, "%s_shrink", heap->name);
		debug_file = debugfs_create_file(
			debug_name, 0644, dev->heaps_debug_root, heap,
			&debug_shrink_fops);
		if (!debug_file) {
			char buf[256], *path;

			path = dentry_path(dev->heaps_debug_root, buf, 256);
			pr_err("Failed to create heap shrinker debugfs at %s/%s\n",
			       path, debug_name);
		}
	}

	dev->heap_cnt++;
	up_write(&dev->lock);
}
EXPORT_SYMBOL(ion_device_add_heap);

struct ion_device *ion_device_create(long (*custom_ioctl)
				     (struct ion_client *client,
				      unsigned int cmd,
				      unsigned long arg))
{
	struct ion_device *idev;
	int ret;

	idev = kzalloc(sizeof(*idev), GFP_KERNEL);
	if (!idev)
		return ERR_PTR(-ENOMEM);

	idev->dev.minor = MISC_DYNAMIC_MINOR;
	idev->dev.name = "ion";
	idev->dev.fops = &ion_fops;
	idev->dev.parent = NULL;
	ret = misc_register(&idev->dev);
	if (ret) {
		pr_err("ion: failed to register misc device.\n");
		kfree(idev);
		return ERR_PTR(ret);
	}

	idev->debug_root = debugfs_create_dir("ion", NULL);
	if (!idev->debug_root) {
		pr_err("ion: failed to create debugfs root directory.\n");
		goto debugfs_done;
	}
	idev->heaps_debug_root = debugfs_create_dir("heaps", idev->debug_root);
	if (!idev->heaps_debug_root) {
		pr_err("ion: failed to create debugfs heaps directory.\n");
		goto debugfs_done;
	}
	idev->clients_debug_root = debugfs_create_dir("clients",
						idev->debug_root);
	if (!idev->clients_debug_root)
		pr_err("ion: failed to create debugfs clients directory.\n");

debugfs_done:

	idev->custom_ioctl = custom_ioctl;
	idev->buffers = RB_ROOT;
	mutex_init(&idev->buffer_lock);
	init_rwsem(&idev->lock);
	plist_head_init(&idev->heaps);
	idev->clients = RB_ROOT;
	return idev;
}
EXPORT_SYMBOL(ion_device_create);

void ion_device_destroy(struct ion_device *dev)
{
	misc_deregister(&dev->dev);
	debugfs_remove_recursive(dev->debug_root);
	/* XXX need to free the heaps and clients ? */
	kfree(dev);
}
EXPORT_SYMBOL(ion_device_destroy);

int ion_phys(struct ion_client *client, struct ion_handle *handle,
	     ion_phys_addr_t *addr, size_t *len)
{
	struct ion_buffer *buffer;
	int ret;

	/*avoid camelcase, will modify in a letter*/
	mmprofile_log_ex(ion_mmp_events[PROFILE_GET_PHYS], MMPROFILE_FLAG_START,
			 (unsigned long)client, (unsigned long)handle);
	mutex_lock(&client->lock);
	if (!ion_handle_validate(client, handle)) {
		mutex_unlock(&client->lock);
		IONMSG("ion: %s invalid handle pass to phys.\n", __func__);
		return -EINVAL;
	}

	buffer = handle->buffer;

	if (!buffer->heap->ops->phys) {
		IONMSG("ion_phys is not impl by heap (name=%s, type=%d).\n",
		       buffer->heap->name, buffer->heap->type);
		mutex_unlock(&client->lock);
		return -ENODEV;
	}
	mutex_unlock(&client->lock);
	mutex_lock(&buffer->lock);
	ret = buffer->heap->ops->phys(buffer->heap, buffer, addr, len);
	mutex_unlock(&buffer->lock);
	/*avoid camelcase, will modify in a letter*/
	mmprofile_log_ex(ion_mmp_events[PROFILE_GET_PHYS], MMPROFILE_FLAG_END,
			 buffer->size, (unsigned long)*addr);
	return ret;
}
EXPORT_SYMBOL(ion_phys);

/* ===================================== */
/* helper functions */
/* ===================================== */
struct ion_buffer *ion_handle_buffer(struct ion_handle *handle)
{
	return handle->buffer;
}

struct ion_handle *ion_drv_get_handle(struct ion_client *client,
				      int user_handle,
				      struct ion_handle *kernel_handle,
				      int from_kernel)
{
	struct ion_handle *handle;

	mutex_lock(&client->lock);
	if (from_kernel) {
		handle = kernel_handle;

		if (IS_ERR_OR_NULL(handle) ||
		    !ion_handle_validate(client, handle))
			goto err;
		handle = ion_handle_get_check_overflow(handle);
	} else {
		handle = ion_handle_get_by_id_nolock(client, user_handle);
	}

	if (IS_ERR_OR_NULL(handle))
		goto err;

	mutex_unlock(&client->lock);
	return handle;
err:
	IONMSG("%s handle invalid, kernel:%d, handle=%p, handle_id=%d\n",
	       __func__, from_kernel, handle, user_handle);
	mutex_unlock(&client->lock);
	return ERR_PTR(-EINVAL);
}

int ion_drv_put_kernel_handle(void *kernel_handle)
{
	return ion_handle_put(kernel_handle);
}

struct ion_heap *ion_drv_get_heap(struct ion_device *dev,
				  int heap_id,
				  int need_lock)
{
	struct ion_heap *_heap, *heap = NULL, *tmp;

	if (need_lock) {
		if (!down_read_trylock(&dev->lock)) {
			IONMSG("%s get lock fail\n", __func__);
			return NULL;
		}
	}

	plist_for_each_entry_safe(_heap, tmp, &dev->heaps, node) {
		if (_heap->id == heap_id) {
			heap = _heap;
			break;
		}
	}

	if (need_lock)
		up_read(&dev->lock);

	return heap;
}

struct ion_buffer *ion_drv_file_to_buffer(struct file *file)
{
	struct dma_buf *dmabuf;
	struct ion_buffer *buffer = NULL;
	const char *pathname = NULL;

	if (!file)
		goto file2buf_exit;
	if (!(file->f_path.dentry))
		goto file2buf_exit;

	pathname = file->f_path.dentry->d_name.name;
	if (!pathname)
		goto file2buf_exit;

	if (strstr(pathname, "dmabuf")) {
		dmabuf = file->private_data;
		if (dmabuf->ops == &dma_buf_ops)
			buffer = dmabuf->priv;
	}

file2buf_exit:

	if (buffer)
		return buffer;
	else
		return ERR_PTR(-EINVAL);
}

/* ===================================== */

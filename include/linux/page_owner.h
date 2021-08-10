#ifndef __LINUX_PAGE_OWNER_H
#define __LINUX_PAGE_OWNER_H

#include <linux/jump_label.h>

#ifdef CONFIG_PAGE_OWNER
extern struct static_key_false page_owner_inited;
extern struct page_ext_operations page_owner_ops;

extern void __reset_page_owner(struct page *page, unsigned int order);
extern void __set_page_owner(struct page *page,
			unsigned int order, gfp_t gfp_mask);
extern void __split_page_owner(struct page *page, unsigned int order);
extern void __copy_page_owner(struct page *oldpage, struct page *newpage);
extern void __set_page_owner_migrate_reason(struct page *page, int reason);
extern void __dump_page_owner(struct page *page);
extern void pagetypeinfo_showmixedcount_print(struct seq_file *m,
					pg_data_t *pgdat, struct zone *zone);

extern int __dump_pfn_backtrace(unsigned long pfn);

static inline int dump_pfn_backtrace(unsigned long pfn)
{
	if (static_branch_unlikely(&page_owner_inited))
		return __dump_pfn_backtrace(pfn);
	else
		return -1;
}

static inline void reset_page_owner(struct page *page, unsigned int order)
{
	if (static_branch_unlikely(&page_owner_inited))
		__reset_page_owner(page, order);
}

static inline void set_page_owner(struct page *page,
			unsigned int order, gfp_t gfp_mask)
{
	if (static_branch_unlikely(&page_owner_inited))
		__set_page_owner(page, order, gfp_mask);
}

static inline void split_page_owner(struct page *page, unsigned int order)
{
	if (static_branch_unlikely(&page_owner_inited))
		__split_page_owner(page, order);
}
static inline void copy_page_owner(struct page *oldpage, struct page *newpage)
{
	if (static_branch_unlikely(&page_owner_inited))
		__copy_page_owner(oldpage, newpage);
}
static inline void set_page_owner_migrate_reason(struct page *page, int reason)
{
	if (static_branch_unlikely(&page_owner_inited))
		__set_page_owner_migrate_reason(page, reason);
}
static inline void dump_page_owner(struct page *page)
{
	if (static_branch_unlikely(&page_owner_inited))
		__dump_page_owner(page);
}

#ifdef CONFIG_PAGE_OWNER_SLIM

struct HandleCount {
	depot_stack_handle_t handle;
	size_t allocations;
	struct hlist_node node;
};

struct BtEntry {
	struct list_head list;
	size_t nr_entries;
	size_t allocations;
	unsigned long backtrace[8];
};

#endif
#else
static inline void reset_page_owner(struct page *page, unsigned int order)
{
}
static inline void set_page_owner(struct page *page,
			unsigned int order, gfp_t gfp_mask)
{
}
static inline void split_page_owner(struct page *page,
			unsigned int order)
{
}
static inline void copy_page_owner(struct page *oldpage, struct page *newpage)
{
}
static inline void set_page_owner_migrate_reason(struct page *page, int reason)
{
}

static inline int dump_pfn_backtrace(unsigned long pfn)
{
	return -1;
}

static inline void dump_page_owner(struct page *page)
{
}
#endif /* CONFIG_PAGE_OWNER */
#endif /* __LINUX_PAGE_OWNER_H */

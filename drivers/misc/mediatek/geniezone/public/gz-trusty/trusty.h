/*
 * Copyright (C) 2013 Google, Inc.
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
#ifndef __LINUX_TRUSTY_TRUSTY_H
#define __LINUX_TRUSTY_TRUSTY_H

#include <linux/kernel.h>
#include <gz-trusty/sm_err.h>
#include <linux/device.h>
#include <linux/pagemap.h>

#ifdef CONFIG_GZ_TRUSTY_INTERRUPT_MAP
extern void handle_trusty_ipi(int ipinr);
#endif	/* CONFIG_GZ_TRUSTY_INTERRUPT_MAP */
s32 trusty_std_call32(struct device *dev, u32 smcnr, u32 a0, u32 a1, u32 a2);
s32 trusty_fast_call32(struct device *dev, u32 smcnr, u32 a0, u32 a1, u32 a2);
#ifdef CONFIG_64BIT
s64 trusty_fast_call64(struct device *dev, u64 smcnr, u64 a0, u64 a1, u64 a2);
#endif	/* CONFIG_64BIT */

struct notifier_block;
enum {
	TRUSTY_CALL_PREPARE,
	TRUSTY_CALL_RETURNED,
};
int trusty_call_notifier_register(struct device *dev, struct notifier_block *n);
int trusty_call_notifier_unregister(struct device *dev,
				    struct notifier_block *n);
const char *trusty_version_str_get(struct device *dev);
u32 trusty_get_api_version(struct device *dev);

struct ns_mem_page_info {
	uint64_t attr;
};

int trusty_encode_page_info(struct ns_mem_page_info *inf,
			    struct page *page, pgprot_t pgprot);

int trusty_call32_mem_buf(struct device *dev, u32 smcnr,
			  struct page *page, u32 size, pgprot_t pgprot);

struct trusty_nop {
	struct list_head node;
	u32 args[3];
};

static inline void trusty_nop_init(struct trusty_nop *nop,
				   u32 arg0, u32 arg1, u32 arg2)
{
	INIT_LIST_HEAD(&nop->node);
	nop->args[0] = arg0;
	nop->args[1] = arg1;
	nop->args[2] = arg2;
}

/* For multiple TEEs */
enum tee_id_t {
	TEE_ID_TRUSTY = 0x0,	//MTEE1.x
	TEE_ID_NEBULA = 0x1,	//MTEE2.x
	TEE_ID_END
};

void trusty_enqueue_nop(struct device *dev, struct trusty_nop *nop);

void trusty_dequeue_nop(struct device *dev, struct trusty_nop *nop);

#define is_trusty_tee(tee_id) ((tee_id) == TEE_ID_TRUSTY)

#define is_nebula_tee(tee_id) ((tee_id) == TEE_ID_NEBULA)

#define is_tee_id(num) (is_trusty_tee(num) || is_nebula_tee(num))

#define get_tee_name(tee_id)	\
	((is_tee_id(tee_id)) ?		\
	((is_trusty_tee(tee_id)) ? "Trusty" : "Nebula") : "None")

#ifndef VIRTIO_ID_TRUSTY_IPC /*for kernel-4.19*/
#define VIRTIO_ID_TRUSTY_IPC   13
#endif
/* This define should be added into include/uapi/linux/virtio_ids.h
 * #define VIRTIO_ID_NEBULA_IPC   14
 */
#define VIRTIO_ID_NEBULA_IPC   13	/* virtio trusty ipc */

#define MAX_DEV_NAME_LEN 32
#define MAX_MINOR_NAME_LEN 16

struct tipc_dev_name {
	char cdev_name[MAX_MINOR_NAME_LEN];
	char tee_name[MAX_MINOR_NAME_LEN];
};

struct tipc_dev_config {
	u32 msg_buf_max_size;
	u32 msg_buf_alignment;
	struct tipc_dev_name dev_name;
} __packed;

struct trusty_work {
	struct trusty_state *ts;
	struct work_struct work;
};

struct trusty_state {
	struct mutex smc_lock;
	struct atomic_notifier_head notifier;
	struct completion cpu_idle_completion;
	char *version_str;
	u32 api_version;
	struct device *dev;
	struct workqueue_struct *nop_wq;
	struct trusty_work __percpu *nop_works;
	struct list_head nop_queue;
	spinlock_t nop_lock;	/* protects nop_queue */
	enum tee_id_t tee_id;
};

#ifdef CONFIG_MT_GZ_TRUSTY_DEBUGFS
void mtee_create_debugfs(struct trusty_state *s, struct device *dev);
#endif

#endif

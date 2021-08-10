/*
 * Copyright (C) 2018 MediaTek Inc.
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

#define PR_FMT_HEADER_MUST_BE_INCLUDED_BEFORE_ALL_HDRS
#include "private/tmem_pr_fmt.h" PR_FMT_HEADER_MUST_BE_INCLUDED_BEFORE_ALL_HDRS

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/unistd.h>
#include <linux/version.h>

#include "private/mld_helper.h"
#include "private/tmem_error.h"
#include "private/tmem_priv.h"
#include "private/tmem_utils.h"

#ifdef TCORE_UT_FWK_SUPPORT
#include "private/ut_tests.h"
DEFINE_UT_SERVER(trusted_mem_subsys);
#endif

struct trusted_mem_device_table {
	enum TRUSTED_MEM_TYPE mem_type;
	struct trusted_mem_device *device;
};

static struct trusted_mem_device_table tmem_dev[TRUSTED_MEM_MAX];

static inline void run_ut_with_memory_leak_check(u64 cmd, u64 param1,
						 u64 param2, u64 param3)
{
#ifdef TCORE_MEMORY_LEAK_DETECTION_SUPPORT
	size_t start_size = mld_stamp();
#endif

#ifdef TCORE_UT_FWK_SUPPORT
	invoke_ut_test_suite(cmd, param1, param2, param3);
#else
	pr_err("TCORE_UT_FWK_SUPPORT option is not enabled\n");
#endif

#ifdef TCORE_MEMORY_LEAK_DETECTION_SUPPORT
	if (mld_stamp_check(start_size) == MLD_CHECK_PASS)
		pr_info("memory leak check is passed!!!\n");
	else
		pr_err("trusted memory core exists memory leak!\n");
#endif
}

void trusted_mem_ut_cmd_invoke(u64 cmd, u64 param1, u64 param2, u64 param3)
{
	run_ut_with_memory_leak_check(cmd, param1, param2, param3);
}

#define VALID_MEM_TYPE(type) (type != TRUSTED_MEM_INVALID)
struct trusted_mem_device *
get_trusted_mem_device(enum TRUSTED_MEM_TYPE mem_type)
{
	if (VALID_MEM_TYPE(tmem_dev[mem_type].mem_type)
	    && VALID(tmem_dev[mem_type].device))
		return tmem_dev[mem_type].device;

	pr_err("trusted mem device:%d is not registered\n", mem_type);
	return NULL;
}

struct trusted_mem_device *
create_trusted_mem_device(enum TRUSTED_MEM_TYPE register_type,
			  struct trusted_mem_configs *cfg)
{
	struct trusted_mem_device *t_device;

	t_device = mld_kmalloc(sizeof(struct trusted_mem_device), GFP_KERNEL);
	if (INVALID(t_device)) {
		pr_err("%s:%d out of memory!\n", __func__, __LINE__);
		goto err_create_device;
	}

	memset(t_device, 0x0, sizeof(struct trusted_mem_device));
	t_device->ssmr_feature_id = SSMR_FEAT_INVALID_ID;
	if (VALID(cfg))
		memcpy(&t_device->configs, cfg,
		       sizeof(struct trusted_mem_configs));

	get_ssmr_ops(&t_device->ssmr_ops);

	t_device->peer_mgr = create_peer_mgr_desc();
	if (INVALID(t_device->peer_mgr))
		goto err_create_peer_mgr_desc;

	t_device->reg_mgr = create_reg_mgr_desc(register_type, t_device);
	if (INVALID(t_device->reg_mgr))
		goto err_create_reg_mgr_desc;

#ifdef TCORE_PROFILING_SUPPORT
	t_device->profile_mgr = create_profile_mgr_desc();
	if (INVALID(t_device->profile_mgr))
		goto err_create_profile_mgr_desc;
#endif

	pr_info("trusted mem device:%d created\n", register_type);
	return t_device;

#ifdef TCORE_PROFILING_SUPPORT
err_create_profile_mgr_desc:
	mld_kfree(t_device->reg_mgr);
#endif
err_create_reg_mgr_desc:
	mld_kfree(t_device->peer_mgr);
err_create_peer_mgr_desc:
	mld_kfree(t_device);
err_create_device:
	return NULL;
}

static bool is_virt_mem_type(enum TRUSTED_MEM_TYPE type)
{
	return (type >= TRUSTED_MEM_VIRT_START)
	       && (type <= TRUSTED_MEM_VIRT_END);
}

/* clang-format off */
#define FREE_IF_VALID(ptr) \
	do { \
		if (VALID(ptr)) \
			mld_kfree(ptr); \
	} while (0)
/* clang-format on */

void destroy_trusted_mem_device(struct trusted_mem_device *tmem_device)
{
	if (is_virt_mem_type(tmem_device->mem_type) && VALID(tmem_device)) {
		FREE_IF_VALID(tmem_device);
		return;
	}

	if (VALID(tmem_device)) {
#ifdef TCORE_PROFILING_SUPPORT
		FREE_IF_VALID(tmem_device->profile_mgr);
#endif
		FREE_IF_VALID(tmem_device->reg_mgr);
		FREE_IF_VALID(tmem_device->peer_mgr);
		FREE_IF_VALID(tmem_device);
	}
}

#ifdef TCORE_PROFILING_SUPPORT
/* clang-format off */
#define PTR_SWAP(a, b) \
	do { \
		typeof(a) tmp; \
		tmp = b; \
		b = a; \
		a = tmp; \
	} while (0)
/* clang-format on */

static void install_profiler(struct trusted_mem_device *tmem_device)
{
	PTR_SWAP(tmem_device->ssmr_ops,
		 tmem_device->profile_mgr->profiled_ssmr_ops);
	PTR_SWAP(tmem_device->peer_ops,
		 tmem_device->profile_mgr->profiled_peer_ops);
	PTR_SWAP(tmem_device->peer_priv,
		 tmem_device->profile_mgr->profiled_peer_priv);
}
#endif

static bool is_invalid_peer_ops(struct trusted_driver_operations *ops)
{
	if (INVALID(ops))
		return true;
	if (INVALID(ops->session_close) || INVALID(ops->session_open)
	    || INVALID(ops->memory_grant) || INVALID(ops->memory_reclaim)
	    || INVALID(ops->memory_alloc) || INVALID(ops->memory_free)
	    || INVALID(ops->invoke_cmd))
		return true;

	return false;
}

static bool is_invalid_ssmr_ops(struct ssmr_operations *ops)
{
	if (INVALID(ops))
		return true;
	if (INVALID(ops->offline) || INVALID(ops->online))
		return true;

	return false;
}

static bool is_invalid_ops_hooks(struct trusted_mem_device *mem_device)
{
	if (is_invalid_peer_ops(mem_device->peer_ops))
		return true;
	if (is_invalid_ssmr_ops(mem_device->ssmr_ops))
		return true;

	return false;
}

int register_trusted_mem_device(enum TRUSTED_MEM_TYPE register_type,
				struct trusted_mem_device *tmem_device)
{
	if (INVALID(tmem_device))
		return TMEM_PARAMETER_ERROR;

	if (!VALID_MEM_TYPE(register_type))
		return TMEM_INVALID_REGISTER_DEVICE;

	if (VALID_MEM_TYPE(tmem_dev[register_type].mem_type)
	    || VALID(tmem_dev[register_type].device))
		return TMEM_MEM_DEVICE_ALREADY_REGISTERED;

	if (is_invalid_ops_hooks(tmem_device))
		return TMEM_INVALID_OPS_HOOKS;

	tmem_dev[register_type].mem_type = register_type;
	tmem_dev[register_type].device = tmem_device;

	if (is_virt_mem_type(register_type))
		goto exit;

#ifdef TCORE_PROFILING_SUPPORT
	install_profiler(tmem_device);
#endif

exit:
	pr_info("trusted mem type '%s' %d registered!\n", tmem_device->name,
		register_type);
	return TMEM_OK;
}

struct trusted_mem_device *create_and_register_shared_trusted_mem_device(
	enum TRUSTED_MEM_TYPE mem_type, struct trusted_mem_device *tmem_device,
	char *dev_name)
{
	struct trusted_mem_device *t_shared_device = NULL;
	int ret;

	t_shared_device =
		mld_kmalloc(sizeof(struct trusted_mem_device), GFP_KERNEL);
	if (INVALID(t_shared_device)) {
		pr_err("%s:%d out of memory!\n", __func__, __LINE__);
		return NULL;
	}

	memcpy(t_shared_device, tmem_device, sizeof(struct trusted_mem_device));
	t_shared_device->ssmr_feature_id = SSMR_FEAT_INVALID_ID;
	snprintf(t_shared_device->name, MAX_DEVICE_NAME_LEN, "%s", dev_name);

	ret = register_trusted_mem_device(mem_type, t_shared_device);
	if (ret) {
		pr_err("register trusted mem device failed:%d\n", ret);
		mld_kfree(t_shared_device);
		return NULL;
	}

	tmem_device->shared_trusted_mem_device = t_shared_device;
	t_shared_device->shared_trusted_mem_device = tmem_device;

	return t_shared_device;
}

static int __init trusted_mem_subsys_init(void)
{
	int idx;

	pr_info("%s:%d\n", __func__, __LINE__);

#ifdef TCORE_MEMORY_LEAK_DETECTION_SUPPORT
	mld_init();
#endif

	for (idx = 0; idx < TRUSTED_MEM_MAX; idx++) {
		tmem_dev[idx].mem_type = TRUSTED_MEM_INVALID;
		tmem_dev[idx].device = NULL;
	}

	pr_info("%s:%d (end)\n", __func__, __LINE__);
	return TMEM_OK;
}

static void __exit trusted_mem_subsys_exit(void)
{
	int idx;

	pr_info("%s:%d\n", __func__, __LINE__);

	for (idx = 0; idx < TRUSTED_MEM_MAX; idx++) {
		if (VALID_MEM_TYPE(tmem_dev[idx].mem_type)
		    && VALID(tmem_dev[idx].device)) {
			destroy_trusted_mem_device(tmem_dev[idx].device);
			tmem_dev[idx].mem_type = TRUSTED_MEM_INVALID;
			tmem_dev[idx].device = NULL;
		}
	}
}

subsys_initcall(trusted_mem_subsys_init);
module_exit(trusted_mem_subsys_exit);

MODULE_AUTHOR("MediaTek Inc.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek Trusted Memory Subsystem Driver");

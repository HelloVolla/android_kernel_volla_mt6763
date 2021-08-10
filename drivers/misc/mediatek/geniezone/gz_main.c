/*
 * Copyright (C) 2015 MediaTek Inc.
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

#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <gz-trusty/smcall.h>
#include <gz-trusty/trusty.h>
#include <gz-trusty/trusty_ipc.h>
#include <kree/mem.h>
#include <kree/system.h>
#include <kree/tz_mod.h>
#include <tz_cross/ta_system.h>
#include <tz_cross/ta_test.h>
#include <tz_cross/trustzone.h>

#include "gz_main.h"
#include "gz_ut.h"
#include "unittest.h"
#include <mtk_mcdi_api.h>

/* FIXME: MTK_PPM_SUPPORT is disabled temporarily */
#ifdef MTK_PPM_SUPPORT
#ifdef CONFIG_MACH_MT6758
#include "legacy_controller.h"
#else
#include "mtk_ppm_platform.h"
#endif
#endif

#ifdef CONFIG_GZ_VPU_WITH_M4U
#include <m4u.h>
#include <m4u_port.h>
#include <kree/sdsp_m4u_mva.h>
#include <gz-trusty/smcall.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>

struct sg_table sg_sdsp_elf;
struct m4u_client_t *m4u_gz_client;
#endif

#if defined(CONFIG_GZ_VPU_WITH_M4U)
uint32_t sdsp_elf_size[2] = { 0, 0 };
uint64_t sdsp_elf_pa[2] = { 0, 0 };
#endif

#define MTEE_kernel_UT_RUN 1
#if MTEE_kernel_UT_RUN		//add tmp
#include "gz_shmem_ut.h"
#include "gz_chmem_ut.h"
#include "gz_sec_storage_ut.h"
#endif

#define KREE_DEBUG(fmt...) pr_debug("[KREE]" fmt)
#define KREE_INFO(fmt...) pr_info("[KREE]" fmt)
#define KREE_ERR(fmt...) pr_info("[KREE][ERR]" fmt)

static const struct file_operations fops = {.owner = THIS_MODULE,
	.open = gz_dev_open,
	.release = gz_dev_release,
	.unlocked_ioctl = gz_ioctl,
#if defined(CONFIG_COMPAT)
	.compat_ioctl = gz_compat_ioctl,
#endif
};

static struct miscdevice gz_device = {.minor = MISC_DYNAMIC_MINOR,
	.name = "gz_kree",
	.fops = &fops
};

static int get_gz_version(void *args);

static const char *cases =
	" 0: GZ Version\n 1: TIPC\n 2.General Function\n"
	" 3: Shared Memory\n 4: GZ abort 5: Chunk Memory\n"
	" C: Secure Storage\n";

/************* sysfs operations ****************/
static ssize_t gz_test_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n", cases);
}

static ssize_t gz_test_store(struct device *dev,
	struct device_attribute *attr, const char *buf,
	size_t n)
{
	char tmp[50];
	char c;
	struct task_struct *th;

	strncpy(tmp, buf, 1);
	tmp[n - 1] = '\0';
	c = tmp[0];

	KREE_DEBUG("GZ KREE test: %c\n", c);
	switch (c) {
	case '0':
		KREE_DEBUG("get gz version\n");
		th = kthread_run(get_gz_version, NULL, "GZ version");
		break;
	case '1':
		KREE_DEBUG("test tipc\n");
		th = kthread_run(gz_tipc_test, NULL, "GZ tipc test");
		break;
	case '2':
		KREE_DEBUG("test general functions\n");
		th = kthread_run(gz_test, NULL, "GZ KREE test");
		break;
	case '3':
		KREE_DEBUG("test shared memory functions\n");
		th = kthread_run(gz_test_shm, NULL, "GZ KREE test");
		break;
	case '4':
		KREE_DEBUG("test GenieZone abort\n");
		th = kthread_run(gz_abort_test, NULL, "GZ KREE test");
		break;
	case '5':
		KREE_DEBUG("test chunk memory\n");
		th = kthread_run(chunk_memory_ut, NULL, "MCM UT");
		break;
	case 'C':
		KREE_DEBUG("test GZ Secure Storage\n");
		th = kthread_run(test_SecureStorageBasic, NULL,
				 "sec_storage_ut");
		break;
	default:
		KREE_DEBUG("err: unknown test case\n");

		break;
	}
	return n;
}


DEVICE_ATTR_RW(gz_test);


static int create_files(void)
{
	int res;

	res = misc_register(&gz_device);

	if (res != 0) {
		KREE_DEBUG("ERR: misc register failed.");
		return res;
	}
	res = device_create_file(gz_device.this_device, &dev_attr_gz_test);
	if (res != 0) {
		KREE_DEBUG("ERR: create sysfs do_info failed.");
		return res;
	}
	return 0;
}

/*********** test case implementations *************/
static const char echo_srv_name[] = "com.mediatek.geniezone.srv.echo";
#define APP_NAME2 "com.mediatek.gz.srv.sync-ut"

static int get_gz_version(void *args)
{
	int ret;
	int i;
	int version_str_len;
	char *version_str;
	struct device *trusty_dev = tz_system_dev->dev.parent;

	ret = trusty_fast_call32(trusty_dev,
				MTEE_SMCNR(SMCF_FC_GET_VERSION_STR, trusty_dev),
				-1, 0, 0);
	if (ret <= 0) {
		KREE_ERR("failed to get version: %d\n", ret);
		return TZ_RESULT_ERROR_GENERIC;
	}

	version_str_len = ret;

	version_str = kmalloc(version_str_len + 1, GFP_KERNEL);
	if (!version_str)
		return TZ_RESULT_ERROR_OUT_OF_MEMORY;

	for (i = 0; i < version_str_len; i++) {
		ret = trusty_fast_call32(trusty_dev,
				MTEE_SMCNR(SMCF_FC_GET_VERSION_STR, trusty_dev),
				i, 0, 0);
		if (ret < 0)
			goto err_get_char;
		version_str[i] = ret;
	}
	version_str[i] = '\0';

	dev_info(gz_device.this_device, "GZ version: %s\n", version_str);
	KREE_DEBUG("GZ version is : %s.....\n", version_str);

err_get_char:
	kfree(version_str);
	version_str = NULL;

	return 0;
}

/************* kernel module file ops (dummy) ****************/
struct UREE_SHAREDMEM_PARAM_US {
	uint64_t buffer;	/* FIXME: userspace void* is 32-bit */
	uint32_t size;
	uint32_t region_id;
};

struct user_shm_param {
	uint32_t session;
	uint32_t shm_handle;
	struct UREE_SHAREDMEM_PARAM_US param;
};

/************ to close session while driver is cancelled *************/
struct session_info {
	int handle_num;		/*num of session handles */
	KREE_SESSION_HANDLE *handles;	/*session handles */
	struct mutex mux;
};

#define queue_SessionNum 32
#define non_SessionID (0xffffffff)

static int _init_session_info(struct file *fp)
{
	struct session_info *info;
	int i;

	info = kmalloc(sizeof(struct session_info), GFP_KERNEL);
	if (!info)
		return TZ_RESULT_ERROR_GENERIC;

	info->handles = kmalloc_array(queue_SessionNum,
				      sizeof(KREE_SESSION_HANDLE), GFP_KERNEL);
	if (!info->handles) {
		kfree(info);
		KREE_ERR("info->handles malloc fail. stop!\n");
		return TZ_RESULT_ERROR_GENERIC;
	}
	info->handle_num = queue_SessionNum;
	for (i = 0; i < info->handle_num; i++)
		info->handles[i] = non_SessionID;

	mutex_init(&info->mux);
	fp->private_data = info;
	return TZ_RESULT_SUCCESS;
}

static int _free_session_info(struct file *fp)
{
	struct session_info *info;
	int i, num;

	KREE_DEBUG("====> [%d] [start] %s is running.\n", __LINE__, __func__);

	info = (struct session_info *)fp->private_data;

	/* lock */
	mutex_lock(&info->mux);

	num = info->handle_num;
	for (i = 0; i < num; i++) {

		if (info->handles[i] == non_SessionID)
			continue;
		KREE_CloseSession(info->handles[i]);
		KREE_DEBUG("=====> session handle[%d] =%d is closed.\n", i,
			   info->handles[i]);

		info->handles[i] = (KREE_SESSION_HANDLE) non_SessionID;
	}

	/* unlock */
	fp->private_data = 0;
	mutex_unlock(&info->mux);

	kfree(info->handles);
	kfree(info);

	KREE_DEBUG("====> [%d] end of %s().\n", __LINE__, __func__);

	return TZ_RESULT_SUCCESS;
}


static int _register_session_info(struct file *fp, KREE_SESSION_HANDLE handle)
{
	struct session_info *info;
	int i, num, nspace, ret = -1;
	void *ptr;

	KREE_DEBUG("[%s][%d] in_handle=0x%x\n", __func__, __LINE__, handle);
	if (handle < 0)
		return TZ_RESULT_ERROR_BAD_PARAMETERS;

	info = (struct session_info *)fp->private_data;

	/* lock */
	mutex_lock(&info->mux);

	/* find empty space. */
	num = info->handle_num;
	for (i = 0; i < num; i++) {
		if (info->handles[i] == non_SessionID) {
			ret = i;
			break;
		}
	}

	/* Try grow the space */
	if (ret == -1) {

		nspace = num * 2;
		ptr = krealloc(info->handles,
			       nspace * sizeof(KREE_SESSION_HANDLE),
			       GFP_KERNEL);
		if (!ptr) {
			mutex_unlock(&info->mux);
			return TZ_RESULT_ERROR_GENERIC;
		}

		ret = num;
		info->handle_num = nspace;
		info->handles = (int *)ptr;
		memset(&info->handles[num], (KREE_SESSION_HANDLE) non_SessionID,
		       (nspace - num) * sizeof(KREE_SESSION_HANDLE));
	}

	if (ret >= 0)
		info->handles[ret] = handle;

	KREE_DEBUG("[%s][%d] reg. session handle[%d]=0x%x\n",
		   __func__, __LINE__, ret, handle);

	/* unlock */
	mutex_unlock(&info->mux);

	return TZ_RESULT_SUCCESS;
}


static int _unregister_session_info(struct file *fp,
	KREE_SESSION_HANDLE in_handleID)
{
	struct session_info *info;
	int ret = TZ_RESULT_ERROR_GENERIC;
	int i;

	KREE_DEBUG("[%s][%d] in_handleID=0x%x\n", __func__, __LINE__,
		   in_handleID);
	if (in_handleID < 0)
		return TZ_RESULT_ERROR_BAD_PARAMETERS;

	info = (struct session_info *)fp->private_data;

	/* lock */
	mutex_lock(&info->mux);

	for (i = 0; i < info->handle_num; i++) {
		if (info->handles[i] == in_handleID) {
			info->handles[i] = (KREE_SESSION_HANDLE) non_SessionID;
			KREE_DEBUG
			    ("[%s][%d]session handle[%d]=0x%x is unreg.\n",
			     __func__, __LINE__, i, in_handleID);
			ret = TZ_RESULT_SUCCESS;
			break;
		}
	}
	/* unlock */
	mutex_unlock(&info->mux);
	KREE_DEBUG("[%d][%s] done. ret=%d\n", __LINE__, __func__, ret);
	return ret;
}

int mtee_sdsp_enable(u32 on)
{
	return trusty_std_call32(tz_system_dev->dev.parent,
			MTEE_SMCNR(MT_SMCF_SC_VPU, tz_system_dev->dev.parent),
			on, 0, 0);
}

atomic_t get_gz_bind_cpu_allowed = ATOMIC_INIT(0);
void set_gz_bind_cpu(int state)
{
	atomic_set(&get_gz_bind_cpu_allowed, state);
}

int get_gz_bind_cpu(void)
{
	return atomic_read(&get_gz_bind_cpu_allowed);
}

int gz_get_cpuinfo_thread(void *data)
{
#ifdef MTK_PPM_SUPPORT
	struct cpufreq_policy curr_policy;
#endif
#ifdef CONFIG_GZ_VPU_WITH_M4U
	int ret;
	uint32_t sdsp_elf_buf_mva;
	uint32_t sdsp_elf_buf_size;
	struct sg_table *sg;
#endif

	if (platform_driver_register(&tz_system_driver))
		KREE_ERR("%s driver register fail\n", __func__);

	KREE_DEBUG("%s driver register done\n", __func__);

#ifdef CONFIG_MACH_MT6758
	msleep(3000);
#else
	msleep(1000);
#endif

#ifdef MTK_PPM_SUPPORT
	cpufreq_get_policy(&curr_policy, 0);
	cpus_cluster_freq[0].max_freq = curr_policy.cpuinfo.max_freq;
	cpus_cluster_freq[0].min_freq = curr_policy.cpuinfo.min_freq;
	cpufreq_get_policy(&curr_policy, 4);
	cpus_cluster_freq[1].max_freq = curr_policy.cpuinfo.max_freq;
	cpus_cluster_freq[1].min_freq = curr_policy.cpuinfo.min_freq;
	KREE_INFO("%s, cluster [0]=%u-%u, [1]=%u-%u\n", __func__,
		  cpus_cluster_freq[0].max_freq, cpus_cluster_freq[0].min_freq,
		  cpus_cluster_freq[1].max_freq, cpus_cluster_freq[1].min_freq);
#endif

	cpumask_clear(&trusty_all_cmask);
	cpumask_setall(&trusty_all_cmask);
	cpumask_clear(&trusty_big_cmask);
	cpumask_set_cpu(6, &trusty_big_cmask);

#ifdef CONFIG_GZ_VPU_WITH_M4U
	if (!m4u_gz_client)
		m4u_gz_client = m4u_create_client();
	KREE_DEBUG("m4u_gz_client(%p)\n", m4u_gz_client);

	sdsp_elf_buf_mva = SDSP_VPU0_ELF_MVA;
	if ((sdsp_elf_size[1]) &&
	    ((sdsp_elf_pa[0] + sdsp_elf_size[0]) == sdsp_elf_pa[1])) {
		sdsp_elf_buf_size = sdsp_elf_size[0] + sdsp_elf_size[1];
	} else {
		sdsp_elf_buf_size = sdsp_elf_size[0];
		KREE_ERR("vpu0,vpu1 pa/size(0x%llx/0x%x)(0x%llx/0x%x)\n",
			 sdsp_elf_pa[0], sdsp_elf_size[0],
			 sdsp_elf_pa[1], sdsp_elf_size[1]);
	}
	sg = &sg_sdsp_elf;
	ret = sg_alloc_table(sg, 1, GFP_KERNEL);
	KREE_DEBUG("%s elf sg_alloc_table %s(%d)\n",
		   __func__, ret == 0 ? "done" : "fail", ret);
	if (!ret) {
		sg_dma_address(sg->sgl) = sdsp_elf_pa[0];
		sg_dma_len(sg->sgl) = sdsp_elf_buf_size;
		ret = m4u_alloc_mva(m4u_gz_client,
				    M4U_PORT_VPU,
				    0, sg, sdsp_elf_buf_size,
				    M4U_PROT_READ | M4U_PROT_WRITE,
				    M4U_FLAGS_START_FROM, &sdsp_elf_buf_mva);
		KREE_INFO("%s elf m4u_alloc_mva(0x%x) %s(%d)\n",
			  __func__, sdsp_elf_buf_mva,
			  ret == 0 ? "done" : "fail", ret);
	}
#endif

	perf_boost_cnt = 0;
	mutex_init(&perf_boost_lock);

#ifdef CONFIG_PM_WAKELOCKS
	wakeup_source_init(&TeeServiceCall_wake_lock, "KREE_TeeServiceCall");
#else
	wake_lock_init(&TeeServiceCall_wake_lock, WAKE_LOCK_SUSPEND,
		"KREE_TeeServiceCall");
#endif

	return 0;
}

#if defined(CONFIG_GZ_VPU_WITH_M4U) && defined(CONFIG_OF_RESERVED_MEM)
static int __init store_sdsp_fw1_setup(struct reserved_mem *rmem)
{
	KREE_DEBUG("%s %s base(0x%llx) size(0x%llx)\n",
		   __func__, rmem->name, rmem->base, rmem->size);
	sdsp_elf_pa[0] = rmem->base;
	sdsp_elf_size[0] = rmem->size;
	return 0;
}

static int __init store_sdsp_fw2_setup(struct reserved_mem *rmem)
{
	KREE_DEBUG("%s %s base(%pa) size(%pa)\n",
		   __func__, rmem->name, &rmem->base, &rmem->size);
	sdsp_elf_pa[1] = rmem->base;
	sdsp_elf_size[1] = rmem->size;
	return 0;
}

RESERVEDMEM_OF_DECLARE(store_sdsp_fw1, "mediatek,gz-sdsp1-fw",
	store_sdsp_fw1_setup);
RESERVEDMEM_OF_DECLARE(store_sdsp_fw2, "mediatek,gz-sdsp2-fw",
	store_sdsp_fw2_setup);
#endif

#ifdef CONFIG_GZ_VPU_WITH_M4U
#include <mtee_regions.h>
struct gz_mva_map_table_t {
	const uint32_t region_id;
	const uint32_t mva;
	KREE_SHAREDMEM_HANDLE handle;
	uint32_t size;
	void *pa;
	struct sg_table sg;
};

#define MAX_GZ_MVA_MAP 1
struct gz_mva_map_table_t gz_mva_map_table[MAX_GZ_MVA_MAP] = {
	{
#if defined(CONFIG_MTK_SDSP_SHARED_PERM_VPU_TEE)
	 .region_id = MTEE_MCHUNKS_SDSP_SHARED_VPU_TEE,
#elif defined(CONFIG_MTK_SDSP_SHARED_PERM_MTEE_TEE)
	 .region_id = MTEE_MCHUNKS_SDSP_SHARED_MTEE_TEE,
#elif defined(CONFIG_MTK_SDSP_SHARED_PERM_VPU_MTEE_TEE)
	 .region_id = MTEE_MCHUNKS_SDSP_SHARED_VPU_MTEE_TEE,
#else
	 .region_id = 0xFFFFFFFF,
#endif
	 .mva = SDSP_VPU0_DTA_MVA,
	 .handle = 0,
	 .size = 0,
	 .pa = NULL}
};

int gz_do_m4u_map(KREE_SHAREDMEM_HANDLE handle, phys_addr_t pa, uint32_t size,
	uint32_t region_id)
{
	uint32_t i;
	uint32_t map_mva;
	int ret;
	struct sg_table *sg;

	if (!m4u_gz_client) {
		KREE_ERR("%s not create m4u_gz_client\n", __func__);
		return -1;
	}

	for (i = 0; i < MAX_GZ_MVA_MAP; i++) {
		if (gz_mva_map_table[i].region_id == region_id) {
			if (gz_mva_map_table[i].handle != 0) {
				KREE_ERR("%s region has been MAP\n", __func__);
				return -1;
			}
			map_mva = gz_mva_map_table[i].mva;
			sg = &(gz_mva_map_table[i].sg);
			ret = sg_alloc_table(sg, 1, GFP_KERNEL);
			if (ret) {
				KREE_ERR("%s region%u alloc sg fail\n",
				__func__, gz_mva_map_table[i].region_id);

				return ret;
			}
			sg_dma_address(sg->sgl) = (dma_addr_t) pa;
			sg_dma_len(sg->sgl) = size;
			ret = m4u_alloc_mva(m4u_gz_client, M4U_PORT_VPU,
					0, sg, size,
					M4U_PROT_READ | M4U_PROT_WRITE,
					M4U_FLAGS_START_FROM, &map_mva);
			if (ret || map_mva != gz_mva_map_table[i].mva) {
				KREE_ERR("%s mva alloc fail\n", __func__);
				return -1;
			}
			gz_mva_map_table[i].handle = handle;
			gz_mva_map_table[i].size = size;
			gz_mva_map_table[i].pa = pa;
			KREE_DEBUG("%s map pa(%p) size(%x) to mva(0x%x)\n",
				__func__, gz_mva_map_table[i].pa,
				gz_mva_map_table[i].size,
				gz_mva_map_table[i].mva);
			return 0;
		}
	}
	return 0;
}

int gz_do_m4u_umap(KREE_SHAREDMEM_HANDLE handle)
{
	uint32_t i;
	int ret;
	struct sg_table *sg;

	if (m4u_gz_client == NULL) {
		KREE_ERR("%s not create m4u_gz_client\n", __func__);
		return -1;
	}

	for (i = 0; i < MAX_GZ_MVA_MAP; i++) {
		if (gz_mva_map_table[i].handle == handle) {
			if (!gz_mva_map_table[i].handle) {
				KREE_ERR("%s region no any MAP\n", __func__);
				return -1;
			}
			ret = m4u_dealloc_mva(m4u_gz_client,
					M4U_PORT_VPU,
					gz_mva_map_table[i].mva);
			if (ret) {
				KREE_ERR("%s mva dealloc fail\n", __func__);
				return ret;
			}
			sg = &(gz_mva_map_table[i].sg);
			sg_free_table(sg);
			KREE_DEBUG("%s ummap mva(0x%x) for region(%u)\n",
				__func__, gz_mva_map_table[i].mva,
				gz_mva_map_table[i].region_id);
			gz_mva_map_table[i].handle = 0;
			gz_mva_map_table[i].size = 0;
			gz_mva_map_table[i].pa = 0;
			return 0;
		}
	}

	return 0;
}
#endif

static LIST_HEAD(fp_deepidle_list);

struct deepidle_counter_state {
	uint64_t fp;
	uint32_t fp_mcdi_counter;
	struct list_head node;
};

uint32_t gz_mcdi_pause_state;
struct mutex fp_mcdi_state_mux;

static int _init_deepidle_counter(struct file *fp)
{
	struct deepidle_counter_state *ddc_state;

	ddc_state = kzalloc(sizeof(struct deepidle_counter_state), GFP_KERNEL);
	if (!ddc_state) {
		KREE_ERR("kzalloc deepidle_counter_state failed!\n");
		return TZ_RESULT_ERROR_OUT_OF_MEMORY;
	}

	mutex_lock(&fp_mcdi_state_mux);
	ddc_state->fp = (uint64_t)fp;
	ddc_state->fp_mcdi_counter = 0;
	list_add_tail(&ddc_state->node, &fp_deepidle_list);
	mutex_unlock(&fp_mcdi_state_mux);

	return TZ_RESULT_SUCCESS;
}

static void _free_deepidle_counter(struct file *fp)
{
	struct deepidle_counter_state *ddc_state;

	mutex_lock(&fp_mcdi_state_mux);

	list_for_each_entry(ddc_state, &fp_deepidle_list, node) {
		if (ddc_state->fp == (uint64_t)fp) {
			list_del(&ddc_state->node);
			kfree(ddc_state);
			break;
		}
	}

	if (list_empty_careful(&fp_deepidle_list) && gz_mcdi_pause_state != 0) {
		mcdi_pause(MCDI_PAUSE_BY_GZ, false);
		gz_mcdi_pause_state = 0;
	}

	mutex_unlock(&fp_mcdi_state_mux);
}


static int gz_dev_open(struct inode *inode, struct file *filp)
{
	int rc = 0;

	rc = _init_deepidle_counter(filp);
	if (rc) {
		KREE_ERR("_init_deepidle_counter failed, rc (%d)\n", rc);
		return rc;
	}
	return _init_session_info(filp);
}

static int gz_dev_release(struct inode *inode, struct file *filp)
{
	_free_deepidle_counter(filp);
	return _free_session_info(filp);
}

static TZ_RESULT _get_US_PAMapAry(struct user_shm_param *shm_data,
	KREE_SHAREDMEM_PARAM *shm_param, int *numOfPA,
	struct MTIOMMU_PIN_RANGE_T *pin, uint64_t *map_p)
{
	unsigned long cret;
	struct page **page;
	int i;
	unsigned long *pfns;
	struct page **delpages;

	KREE_DEBUG("[%s][%d] runs.\n", __func__, __LINE__);
	KREE_DEBUG("session: %u, shm_handle: %u, size: %u, buffer: 0x%llx\n",
		(*shm_data).session, (*shm_data).shm_handle,
		(*shm_data).param.size, (*shm_data).param.buffer);

	if (((*shm_data).param.size <= 0) || (!(*shm_data).param.buffer)) {
		KREE_DEBUG("[%s] [fail] size <= 0 OR !buffer\n", __func__);
		return TZ_RESULT_ERROR_BAD_PARAMETERS;
	}

	/*init value */
	pin = NULL;
	map_p = NULL;
	(*shm_param).buffer = NULL;
	(*shm_param).size = 0;
	(*shm_param).mapAry = NULL;

	cret = TZ_RESULT_SUCCESS;
	/*
	 * map pages
	 */
	/*
	 * 1. get user pages
	 * note: 'pin' resource need to keep for unregister.
	 * It will be freed after unregisted.
	 */

	pin = kzalloc(sizeof(struct MTIOMMU_PIN_RANGE_T), GFP_KERNEL);
	if (!pin) {
		KREE_DEBUG("[%s]zalloc fail: pin is null.\n", __func__);
		cret = TZ_RESULT_ERROR_OUT_OF_MEMORY;
		goto us_map_fail;
	}
	pin->pageArray = NULL;
	cret = _map_user_pages(pin, (unsigned long)(*shm_data).param.buffer,
			(*shm_data).param.size, 0);

	if (cret) {
		pin->pageArray = NULL;
		KREE_DEBUG("[%s]_map_user_pages fail. map user pages = 0x%x\n",
			__func__, (uint32_t) cret);
		cret = TZ_RESULT_ERROR_INVALID_HANDLE;
		goto us_map_fail;
	}
	/* 2. build PA table */
	map_p = kzalloc(sizeof(uint64_t) * (pin->nrPages + 1), GFP_KERNEL);
	if (!map_p) {
		KREE_DEBUG("[%s]zalloc fail: map_p is null.\n", __func__);
		cret = TZ_RESULT_ERROR_OUT_OF_MEMORY;
		goto us_map_fail;
	}

	if (!pin->pageArray) {
		KREE_ERR("[%s]pin->pageArray is null. fail.\n", __func__);
		cret = TZ_RESULT_ERROR_GENERIC;
		goto us_map_fail;
	}

	map_p[0] = pin->nrPages;
	if (pin->isPage) {
		page = (struct page **)pin->pageArray;
		for (i = 0; i < pin->nrPages; i++) /* PA */
			map_p[1 + i] =
			(uint64_t) PFN_PHYS(page_to_pfn(page[i]));
	} else {		/* pfn */
		pfns = (unsigned long *)pin->pageArray;
		for (i = 0; i < pin->nrPages; i++) /* get PA */
			map_p[1 + i] = (uint64_t) PFN_PHYS(pfns[i]);
	}

	/* init register shared mem params */
	(*shm_param).buffer = NULL;
	(*shm_param).size = 0;
	(*shm_param).mapAry = (void *)map_p;

	*numOfPA = pin->nrPages;

us_map_fail:
	if (pin) {
		if (pin->pageArray) {
			delpages = (struct page **)pin->pageArray;
			if (pin->isPage) {
				for (i = 0; i < pin->nrPages; i++)
					put_page(delpages[i]);
			}
			kfree(pin->pageArray);
		}
		kfree(pin);
	}

	return cret;
}

/**************************************************************************
 *  DEV DRIVER IOCTL
 *  Ported from trustzone driver
 **************************************************************************/
static long tz_client_open_session(struct file *filep, unsigned long arg)
{
	struct kree_session_cmd_param param;
	unsigned long cret;
	char uuid[40];
	long len;
	TZ_RESULT ret;
	KREE_SESSION_HANDLE handle = 0;

	cret = copy_from_user(&param, (void *)arg, sizeof(param));
	if (cret)
		return -EFAULT;

	/* Check if can we access UUID string. 10 for min uuid len. */
	if (!access_ok(VERIFY_READ, (void *)param.data, 10))
		return -EFAULT;

	KREE_DEBUG("%s: uuid addr = 0x%llx\n", __func__, param.data);
	len = strncpy_from_user(uuid, (void *)(unsigned long)param.data,
		sizeof(uuid));
	if (len <= 0)
		return -EFAULT;

	uuid[sizeof(uuid) - 1] = 0;
	ret = KREE_CreateSession(uuid, &handle);
	if (ret != TZ_RESULT_SUCCESS) {
		KREE_DEBUG("%s: kree crt session fail(0x%x)\n", __func__, ret);
		goto tz_client_open_session_end;
	}

	KREE_DEBUG("===> %s: handle =%d\n", __func__, handle);

	_register_session_info(filep, handle);

tz_client_open_session_end:
	param.ret = ret;
	param.handle = handle;
	cret = copy_to_user((void *)arg, &param, sizeof(param));
	if (cret)
		return cret;

	return 0;
}

static long tz_client_close_session(struct file *filep, unsigned long arg)
{
	struct kree_session_cmd_param param;
	unsigned long cret;
	TZ_RESULT ret;

	cret = copy_from_user(&param, (void *)arg, sizeof(param));
	if (cret)
		return -EFAULT;

	if (param.handle < 0 || param.handle >= KREE_SESSION_HANDLE_MAX_SIZE)
		return TZ_RESULT_ERROR_INVALID_HANDLE;

	ret = KREE_CloseSession(param.handle);
	if (ret != TZ_RESULT_SUCCESS) {
		KREE_DEBUG("%s: fail(0x%x)\n", __func__, ret);
		goto _tz_client_close_session_end;
	}

	_unregister_session_info(filep, param.handle);

_tz_client_close_session_end:
	param.ret = ret;
	cret = copy_to_user((void *)arg, &param, sizeof(param));
	if (cret)
		return -EFAULT;

	return 0;
}

static long tz_client_tee_service(struct file *file, unsigned long arg,
	unsigned int compat)
{
	struct kree_tee_service_cmd_param cparam;
	unsigned long cret;
	uint32_t tmpTypes;
	union MTEEC_PARAM param[4], oparam[4];
	int i;
	TZ_RESULT ret;
	KREE_SESSION_HANDLE handle;
	void __user *ubuf;
	uint32_t ubuf_sz;

	cret = copy_from_user(&cparam, (void *)arg, sizeof(cparam));
	if (cret) {
		KREE_ERR("%s: copy_from_user(msg) failed\n", __func__);
		return -EFAULT;
	}

	if (cparam.paramTypes != TZPT_NONE || cparam.param) {
		if (!access_ok(VERIFY_READ, (void *)cparam.param,
			sizeof(oparam)))
			return -EFAULT;

		cret = copy_from_user(oparam,
				(void *)(unsigned long)cparam.param,
				sizeof(oparam));
		if (cret) {
			KREE_ERR("%s: copy_from_user(param) failed\n",
				__func__);
			return -EFAULT;
		}
	}

	handle = (KREE_SESSION_HANDLE) cparam.handle;
	KREE_DEBUG("%s: session handle = %u\n", __func__, handle);

	/* Parameter processing. */
	memset(param, 0, sizeof(param));
	tmpTypes = cparam.paramTypes;
	for (i = 0; tmpTypes; i++) {
		enum TZ_PARAM_TYPES type = tmpTypes & 0xff;

		tmpTypes >>= 8;
		switch (type) {
		case TZPT_VALUE_INPUT:
		case TZPT_VALUE_INOUT:
			param[i] = oparam[i];
			break;

		case TZPT_VALUE_OUTPUT:
		case TZPT_NONE:
			break;

		case TZPT_MEM_INPUT:
		case TZPT_MEM_OUTPUT:
		case TZPT_MEM_INOUT:
#ifdef CONFIG_COMPAT
			if (compat) {
				ubuf = compat_ptr(oparam[i].mem32.buffer);
				ubuf_sz = oparam[i].mem32.size;
			} else
#endif
			{
				ubuf = oparam[i].mem.buffer;
				ubuf_sz = oparam[i].mem.size;
			}

			KREE_DEBUG("%s: ubuf = %p, ubuf_sz = %u\n", __func__,
				ubuf, ubuf_sz);

			if (type != TZPT_MEM_OUTPUT) {
				if (!access_ok(VERIFY_READ, ubuf, ubuf_sz)) {
					KREE_ERR("%s: cannnot read mem\n",
						__func__);
					cret = -EFAULT;
					goto error;
				}
			}
			if (type != TZPT_MEM_INPUT) {
				if (!access_ok(VERIFY_WRITE, ubuf, ubuf_sz)) {
					KREE_ERR("%s: cannnot write mem\n",
						__func__);
					cret = -EFAULT;
					goto error;
				}
			}

			if (ubuf_sz > GP_MEM_MAX_LEN) {
				KREE_ERR("%s: ubuf_sz larger than max(%d)\n",
					__func__, GP_MEM_MAX_LEN);
				cret = -ENOMEM;
				goto error;
			}

			param[i].mem.size = ubuf_sz;
			param[i].mem.buffer =
			    kmalloc(param[i].mem.size, GFP_KERNEL);
			if (!param[i].mem.buffer) {
				KREE_ERR("%s: kmalloc failed\n", __func__);
				cret = -ENOMEM;
				goto error;
			}

			if (type != TZPT_MEM_OUTPUT) {
				cret = copy_from_user(param[i].mem.buffer, ubuf,
						param[i].mem.size);
				if (cret) {
					KREE_ERR("%s: copy_from_user failed\n",
						__func__);
					cret = -EFAULT;
					goto error;
				}
			}
			break;

		case TZPT_MEMREF_INPUT:
		case TZPT_MEMREF_OUTPUT:
		case TZPT_MEMREF_INOUT:
			param[i] = oparam[i];
			break;

		default:
			ret = TZ_RESULT_ERROR_BAD_FORMAT;
			goto error;
		}
	}

	KREE_SESSION_LOCK(handle);
	ret = KREE_TeeServiceCall(handle, cparam.command, cparam.paramTypes,
			param);
	KREE_SESSION_UNLOCK(handle);

	cparam.ret = ret;
	tmpTypes = cparam.paramTypes;
	for (i = 0; tmpTypes; i++) {
		enum TZ_PARAM_TYPES type = tmpTypes & 0xff;

		tmpTypes >>= 8;
		switch (type) {
		case TZPT_VALUE_OUTPUT:
		case TZPT_VALUE_INOUT:
			oparam[i] = param[i];
			break;

		default:
		case TZPT_MEMREF_INPUT:
		case TZPT_MEMREF_OUTPUT:
		case TZPT_MEMREF_INOUT:
		case TZPT_VALUE_INPUT:
		case TZPT_NONE:
			break;

		case TZPT_MEM_INPUT:
		case TZPT_MEM_OUTPUT:
		case TZPT_MEM_INOUT:
#ifdef CONFIG_COMPAT
			if (compat)
				ubuf = compat_ptr(oparam[i].mem32.buffer);
			else
#endif
				ubuf = oparam[i].mem.buffer;

			if (type != TZPT_MEM_INPUT) {
				cret = copy_to_user(ubuf, param[i].mem.buffer,
					param[i].mem.size);
				if (cret) {
					cret = -EFAULT;
					goto error;
				}
			}

			kfree(param[i].mem.buffer);
			break;
		}
	}

	/* Copy data back. */
	if (cparam.paramTypes != TZPT_NONE) {
		cret = copy_to_user((void *)(unsigned long)cparam.param, oparam,
				sizeof(oparam));
		if (cret) {
			KREE_ERR("%s: copy_to_user(param) failed\n", __func__);
			return -EFAULT;
		}
	}


	cret = copy_to_user((void *)arg, &cparam, sizeof(cparam));
	if (cret) {
		KREE_ERR("%s: copy_to_user(msg) failed\n", __func__);
		return -EFAULT;
	}
	return 0;

error:
	tmpTypes = cparam.paramTypes;
	for (i = 0; tmpTypes; i++) {
		enum TZ_PARAM_TYPES type = tmpTypes & 0xff;

		tmpTypes >>= 8;
		switch (type) {
		case TZPT_MEM_INPUT:
		case TZPT_MEM_OUTPUT:
		case TZPT_MEM_INOUT:
			kfree(param[i].mem.buffer);
			break;

		default:
			break;
		}
	}
	return cret;
}

static long _sc_test_cp_chm2shm(struct file *filep, unsigned long arg)
{

	struct kree_user_sc_param cparam;
	int ret;
	KREE_ION_HANDLE ION_Handle = 0;
	KREE_SECUREMEM_HANDLE shm_handle;
	KREE_SESSION_HANDLE cp_session;
	uint32_t size;

	/* copy param from user */
	ret = copy_from_user(&cparam, (void *)arg, sizeof(cparam));

	if (ret < 0) {
		KREE_ERR("%s: copy_from_user failed(%d)\n", __func__, ret);
		return ret;
	}

	/*input params */
	shm_handle = cparam.other_handle;
	ION_Handle = cparam.ION_handle;	/*need to transform to mem_handle */
	size = cparam.size;	/*alloc size */
	cp_session = cparam.chmp.alloc_chm_session;

	KREE_DEBUG("[%s] input: cp_session=0x%x, shm_handle=0x%x\n",
		__func__, cp_session, shm_handle);
	KREE_DEBUG("[%s] input: ION_Handle=0x%x, size=0x%x\n",
		__func__, ION_Handle, size);

	ret = KREE_ION_CP_Chm2Shm(cp_session, ION_Handle, shm_handle, size);

	if (ret != TZ_RESULT_SUCCESS)
		KREE_ERR("[%s] KREE_ION_CP_Chm2Shm Fail. ret=0x%x\n", __func__,
			ret);
	else
		KREE_DEBUG("[OK]KREE_ION_CP_Chm2Shm done\n");

	return ret;
}

static long _sc_test_upt_chmdata(struct file *filep, unsigned long arg)
{
	struct kree_user_sc_param cparam;
	int ret;
	KREE_ION_HANDLE ION_Handle = 0;
	KREE_SECUREMEM_HANDLE shm_handle;
	KREE_SESSION_HANDLE echo_session = 0;
	union MTEEC_PARAM param[4];
	uint32_t size;

	/* copy param from user */
	ret = copy_from_user(&cparam, (void *)arg, sizeof(cparam));

	if (ret < 0) {
		KREE_ERR("%s: copy_from_user failed(%d)\n", __func__, ret);
		return ret;
	}

	/*input params */
	shm_handle = cparam.other_handle;
	ION_Handle = cparam.ION_handle;
	size = cparam.size;

	/*create session for echo */
	ret = KREE_CreateSession(echo_srv_name, &echo_session);
	if (ret != TZ_RESULT_SUCCESS) {
		KREE_ERR
		("echo_srv CreateSession (echo_session:0x%x) Error: %d\n",
		(uint32_t) echo_session, ret);
		return ret;
	}
	KREE_DEBUG("[OK] create echo_session=0x%x.\n", (uint32_t) echo_session);

	param[0].value.a = ION_Handle; /*need to transform to mem_handle */
	param[1].value.a = size;       /*alloc size */

	KREE_DEBUG("[%s] input: shm_handle=0x%x. ION_Handle=0x%x\n", __func__,
		param[0].value.b, param[0].value.a);

	/*test: modify chm memory data */
	/*TZCMD_TEST_CHM_UPT_DATA */
	ret = KREE_ION_AccessChunkmem(echo_session, param, 0x9989);
	if (ret != TZ_RESULT_SUCCESS)
		KREE_ERR("[%s] modify chm memory data Fail. ret=0x%x\n",
			__func__, ret);
	else
		KREE_DEBUG("[OK]modify chm memory data done\n");

	ret = KREE_CloseSession(echo_session);
	if (ret != TZ_RESULT_SUCCESS)
		KREE_ERR("KREE_CloseSession Error:echo_session=0x%x, ret=%d\n",
			 (uint32_t) echo_session, ret);
	else
		KREE_DEBUG("[OK] close session OK. echo_session=0x%x\n",
			   (uint32_t) echo_session);

	return ret;
}

TZ_RESULT gz_deep_idle_mask(struct file *fp)
{
	struct deepidle_counter_state *ddc_state;

	mutex_lock(&fp_mcdi_state_mux);

	if (gz_mcdi_pause_state == 0)
		mcdi_pause(MCDI_PAUSE_BY_GZ, true);
	else
		KREE_INFO("mcdi already disable!!!!!!!\n");

	list_for_each_entry(ddc_state, &fp_deepidle_list, node) {
		if (ddc_state->fp == (uint64_t)fp) {
			ddc_state->fp_mcdi_counter++;
			gz_mcdi_pause_state++;
			break;
		}
	}
	mutex_unlock(&fp_mcdi_state_mux);

	return TZ_RESULT_SUCCESS;
}

TZ_RESULT gz_deep_idle_unmask(struct file *fp)
{
	TZ_RESULT rc = TZ_RESULT_SUCCESS;
	struct deepidle_counter_state *ddc_state;
	uint32_t tmp_counter = 0;

	mutex_lock(&fp_mcdi_state_mux);

	if (gz_mcdi_pause_state == 0) {
		KREE_INFO("mcdi already enable!!!!!!!\n");
		rc = TZ_RESULT_ERROR_BAD_STATE;
		goto _err_mcdi_state;
	}

	list_for_each_entry(ddc_state, &fp_deepidle_list, node) {
		if (ddc_state->fp == (uint64_t)fp) {
			ddc_state->fp_mcdi_counter--;
			gz_mcdi_pause_state--;
		}
		tmp_counter += ddc_state->fp_mcdi_counter;
	}

	if (tmp_counter == 0) {
		mcdi_pause(MCDI_PAUSE_BY_GZ, false);
		gz_mcdi_pause_state = 0;
	}

_err_mcdi_state:
	mutex_unlock(&fp_mcdi_state_mux);

	return rc;
}

static long _gz_ioctl(struct file *filep, unsigned int cmd, unsigned long arg,
	unsigned int compat)
{
	int err;
	TZ_RESULT ret = 0;
	char __user *user_req;
	struct user_shm_param shm_data;
	struct kree_user_sc_param cparam;
	KREE_SHAREDMEM_PARAM shm_param = {0};
	KREE_SHAREDMEM_HANDLE shm_handle = 0;
	struct MTIOMMU_PIN_RANGE_T *pin = NULL;
	uint64_t *map_p = NULL;
	int numOfPA = 0;

	if (_IOC_TYPE(cmd) != MTEE_IOC_MAGIC)
		return -EINVAL;

	if (compat)
		user_req = (char __user *)compat_ptr(arg);
	else
		user_req = (char __user *)arg;

	switch (cmd) {
	case MTEE_CMD_SHM_REG:
		KREE_DEBUG("[%s]cmd=MTEE_CMD_SHM_REG(0x%x)\n", __func__, cmd);
		/* copy param from user */
		err = copy_from_user(&shm_data, user_req, sizeof(shm_data));
		if (err < 0) {
			KREE_ERR("[%s]copy_from_user fail(0x%x)\n", __func__,
				err);
			return err;
		}

		if ((shm_data.param.size <= 0) || (!shm_data.param.buffer)) {
			KREE_DEBUG("[%s]bad param:size=%x or !param.buffer\n",
				__func__, shm_data.param.size);
			return TZ_RESULT_ERROR_BAD_PARAMETERS;
		}

		KREE_DEBUG("[%s]sizeof(shm_data):0x%x, session:%u, shm_hd:%u",
			__func__, (uint32_t) sizeof(shm_data), shm_data.session,
			shm_data.shm_handle);
		KREE_DEBUG("size:%u, &buffer:0x%llx\n", shm_data.param.size,
			shm_data.param.buffer);

		ret = _get_US_PAMapAry(&shm_data, &shm_param, &numOfPA, pin,
				map_p);
		if (ret != TZ_RESULT_SUCCESS) {
			KREE_ERR("[%s] _get_US_PAMapAry() Fail(ret=0x%x)\n",
				__func__, ret);

			if (shm_param.mapAry != NULL)
				kfree(shm_param.mapAry);

			shm_data.shm_handle = 0;

			/* copy result back to user */
			shm_data.session = ret;
			err = copy_to_user(user_req, &shm_data,
			sizeof(shm_data));
			if (err < 0) {
				KREE_ERR("[%s]copy_to_user fail(0x%x)\n",
					__func__, err);
				return err;
			}
			return ret;
		}

		shm_param.region_id = shm_data.param.region_id;
		ret = KREE_RegisterSharedmem(shm_data.session,
				&shm_handle, &shm_param);

		KREE_DEBUG("[%s] reg shmem ret hd=0x%x\n", __func__,
			shm_handle);
		if ((ret != TZ_RESULT_SUCCESS) || (shm_handle == 0)) {
			KREE_ERR("[%s]RegisterSharedmem Fail", __func__);
			KREE_ERR("ret=0x%x, shm_hd=0x%x)\n", ret, shm_handle);
			if (shm_param.mapAry != NULL)
				kfree(shm_param.mapAry);

			shm_data.shm_handle = 0;

			/* copy result back to user */
			shm_data.session = ret;
			err = copy_to_user(user_req, &shm_data,
			sizeof(shm_data));
			if (err < 0) {
				KREE_ERR("[%s]copy_to_user fail(0x%x)\n",
					__func__, err);
				return err;
			}
			return ret;
		}

		if (shm_param.mapAry != NULL)
			kfree(shm_param.mapAry);

		shm_data.shm_handle = shm_handle;

		/* copy result back to user */
		shm_data.session = ret;
		err = copy_to_user(user_req, &shm_data, sizeof(shm_data));
		if (err < 0) {
			KREE_ERR("[%s]copy_to_user fail(0x%x)\n", __func__,
				err);
			return err;
		}
		break;

	case MTEE_CMD_SHM_UNREG:
		/* do nothing */
		break;

	case MTEE_CMD_OPEN_SESSION:
		KREE_DEBUG("[%s]cmd=MTEE_CMD_OPEN_SESSION(0x%x)\n", __func__,
			cmd);
		ret = tz_client_open_session(filep, arg);
		if (ret != TZ_RESULT_SUCCESS) {
			KREE_ERR("[%s]tz_client_open_session() fail\n",
				__func__);
			return ret;
		}
		break;

	case MTEE_CMD_CLOSE_SESSION:
		KREE_DEBUG("[%s]cmd=MTEE_CMD_CLOSE_SESSION(0x%x)\n", __func__,
			cmd);
		ret = tz_client_close_session(filep, arg);
		if (ret != TZ_RESULT_SUCCESS) {
			KREE_ERR("[%s]tz_client_close_session() fail\n",
				__func__);
			return ret;
		}
		break;

	case MTEE_CMD_TEE_SERVICE:
		KREE_DEBUG("[%s]cmd=MTEE_CMD_TEE_SERVICE(0x%x)\n", __func__,
			cmd);
		return tz_client_tee_service(filep, arg, compat);

	case MTEE_CMD_SC_TEST_CP_CHM2SHM:	/*Secure Camera Test */
		KREE_DEBUG("[%s]cmd=MTEE_CMD_SC_TEST_CP_CHM2SHM(0x%x)\n",
			__func__, cmd);
		return _sc_test_cp_chm2shm(filep, arg);

	case MTEE_CMD_SC_TEST_UPT_CHMDATA:	/*Secure Camera Test */
		KREE_DEBUG("[%s]cmd=MTEE_CMD_SC_TEST_UPT_CHMDATA(0x%x)\n",
			__func__, cmd);
		return _sc_test_upt_chmdata(filep, arg);

	case MTEE_CMD_SC_CHMEM_HANDLE:
		KREE_DEBUG("[%s]cmd=MTEE_CMD_SC_CHMEM_HANDLE(0x%x)\n", __func__,
			cmd);
		err = copy_from_user(&cparam, user_req, sizeof(cparam));
		if (err < 0) {
			KREE_ERR("[%s]copy_from_user fail(0x%x)\n", __func__,
				err);
			return err;
		}
		ret =
			_IONHandle2MemHandle(cparam.ION_handle,
			&(cparam.other_handle));
		if (ret != TZ_RESULT_SUCCESS) {
			KREE_ERR("[%s]_IONHandle2MemHandle fail(0x%x)\n",
				__func__, ret);
			return ret;
		}
		err = copy_to_user(user_req, &cparam, sizeof(cparam));
		if (err < 0) {
			KREE_ERR("[%s]copy_to_user fail(0x%x)\n", __func__,
				err);
			return err;
		}
		break;

	case MTEE_CMD_FOD_TEE_SHM_ON:
		KREE_DEBUG("====> MTEE_CMD_FOD_TEE_SHM_ON ====\n");
		ret = mtee_sdsp_enable(1);
		break;

	case MTEE_CMD_FOD_TEE_SHM_OFF:
		KREE_DEBUG("====> MTEE_CMD_FOD_TEE_SHM_OFF ====\n");
		ret = mtee_sdsp_enable(0);
		break;

	case MTEE_CMD_DEEP_IDLE_MASK:
		ret = gz_deep_idle_mask(filep);
		break;

	case MTEE_CMD_DEEP_IDLE_UNMASK:
		ret = gz_deep_idle_unmask(filep);
		break;

	default:
		KREE_ERR("[%s] undefined ioctl cmd 0x%x\n", __func__, cmd);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static long gz_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	long ret;

	set_gz_bind_cpu(1);
	ret = _gz_ioctl(filep, cmd, arg, 0);
	set_gz_bind_cpu(0);
	return ret;
}

#if defined(CONFIG_COMPAT)
static long gz_compat_ioctl(struct file *filep, unsigned int cmd,
	unsigned long arg)
{
	long ret;

	set_gz_bind_cpu(1);
	ret = _gz_ioctl(filep, cmd, arg, 1);
	set_gz_bind_cpu(0);
	return ret;
}
#endif

/************ kernel module init entry ***************/
static int __init gz_init(void)
{
	int res;

	res = create_files();
	if (res) {
		KREE_DEBUG("create sysfs failed: %d\n", res);
	} else {
		struct task_struct *gz_get_cpuinfo_task;
		struct task_struct *ree_dummy_task;

		gz_get_cpuinfo_task =
		    kthread_create(gz_get_cpuinfo_thread, NULL,
				"gz_get_cpuinfo_task");
		if (IS_ERR(gz_get_cpuinfo_task)) {
			KREE_ERR("Unable to start kernel thread %s\n",
				__func__);
			res = PTR_ERR(gz_get_cpuinfo_task);
		} else
			wake_up_process(gz_get_cpuinfo_task);

		ree_dummy_task =
		kthread_create(ree_dummy_thread, NULL, "ree_dummy_task");
		if (IS_ERR(ree_dummy_task)) {
			KREE_ERR("Unable to start kernel thread %s\n",
				__func__);
			res = PTR_ERR(ree_dummy_task);
		} else {
			struct cpumask ree_dummy_cmask;

			cpumask_clear(&ree_dummy_cmask);
			cpumask_set_cpu(7, &ree_dummy_cmask);
			set_cpus_allowed_ptr(ree_dummy_task, &ree_dummy_cmask);
			set_user_nice(ree_dummy_task, -20);
			wake_up_process(ree_dummy_task);
		}
	}

	mutex_init(&fp_mcdi_state_mux);
	return res;
}

/************ kernel module exit entry ***************/
static void __exit gz_exit(void)
{
	KREE_DEBUG("gz driver exit\n");
}


module_init(gz_init);
module_exit(gz_exit);

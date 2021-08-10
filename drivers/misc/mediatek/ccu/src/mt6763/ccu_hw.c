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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/uaccess.h>

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>
#include <linux/spinlock.h>
#include <linux/iommu.h>

#include "mtk_ion.h"
#include "ion_drv.h"
#include <linux/iommu.h>

#ifdef CONFIG_MTK_IOMMU
#include "mtk_iommu.h"
#include <dt-bindings/memory/mt6763-larb-port.h>
#else
#include "m4u.h"
#endif

#include <linux/io.h> /*for mb();*/

#include "ccu_inc.h"
#include "ccu_hw.h"
#include "ccu_reg.h"
#include "ccu_cmn.h"
#include "ccu_kd_mailbox.h"
#include "ccu_i2c.h"

#include "kd_camera_feature.h" /*for sensorType in ccu_set_sensor_info*/

static uint64_t camsys_base;
static uint64_t bin_base;
static uint64_t dmem_base;

static struct ccu_device_s *ccu_dev;
static struct task_struct *enque_task;
static struct m4u_client_t *m4u_client;

uint32_t i2c_mva;

static struct mutex cmd_mutex;
static wait_queue_head_t cmd_wait;
static  bool cmd_done;
static int32_t g_ccu_sensor_current_fps = -1;

#define SENSOR_NAME_MAX_LEN 32
static struct ccu_sensor_info
	g_ccu_sensor_info[IMGSENSOR_SENSOR_IDX_MAX_NUM] = {0};
static char g_ccu_sensor_name
	[IMGSENSOR_SENSOR_IDX_MAX_NUM][SENSOR_NAME_MAX_LEN];

struct ccu_mailbox_t *pMailBox[MAX_MAILBOX_NUM];
static struct ccu_msg_t receivedCcuCmd;
static struct ccu_msg_t CcuAckCmd;
static uint32_t i2c_buffer_mva;

/*isr work management*/
struct ap_task_manage_t {
	struct workqueue_struct *ApTaskWorkQueue;
	struct mutex ApTaskMutex;
	struct list_head ApTskWorkList;
};

struct ap_task_manage_t ap_task_manage;


static struct CCU_INFO_STRUCT ccuInfo;
static  bool bWaitCond;
static bool AFbWaitCond[2];
static unsigned int g_LogBufIdx = 1;
static unsigned int AFg_LogBufIdx[2] = {1, 1};
static struct ccu_cmd_s *_fast_cmd_ack;

static int _ccu_powerdown(void);
static int _ccu_allocate_mva(uint32_t *mva, void *va);
static int _ccu_deallocate_mva(uint32_t *mva);

static int _ccu_config_m4u_port(void)
{
	int ret = 0;

#if defined(CONFIG_MTK_M4U)
	M4U_PORT_STRUCT port;

	port.ePortID = CCUG_OF_M4U_PORT;
	port.Virtuality = 1;
	port.Security = 0;
	port.domain = 3;
	port.Distance = 1;
	port.Direction = 0;

	ret = m4u_config_port(&port);
#endif
	return ret;
}

static int _ccu_deallocate_mva(uint32_t *mva)
{
	int ret = 0;

	LOG_DBG("X-:%s\n", __func__);

	if (*mva != 0 && m4u_client != NULL) {
		ret = m4u_dealloc_mva(m4u_client, CCUG_OF_M4U_PORT, *mva);
		if (ret)
			LOG_ERR("dealloc mva fail");
		m4u_destroy_client(m4u_client);
		m4u_client = NULL;
		*mva = 0;
	}

	return ret;
}

static int _ccu_allocate_mva(uint32_t *mva, void *va)
{
	int ret = 0;
	int buffer_size = 4096;
	struct sg_table *sg_table = NULL;
	unsigned int flag;

	if (!m4u_client)
		m4u_client = m4u_create_client();
	if (IS_ERR_OR_NULL(m4u_client)) {
		LOG_ERR("create client fail!\n");
		return -EINVAL;
	}

	ret = _ccu_config_m4u_port();
	if (ret) {
		LOG_ERR("fail to config m4u port!\n");
		return ret;
	}

	/* alloc mva */
	flag = M4U_FLAGS_START_FROM;
	ret = m4u_alloc_mva(m4u_client, CCUG_OF_M4U_PORT, (unsigned long)va,
		sg_table, buffer_size,
		M4U_PROT_READ | M4U_PROT_WRITE, flag, mva);
	if (ret)
		LOG_ERR("alloc mva fail");

	return ret;
}

static inline unsigned int CCU_MsToJiffies(unsigned int Ms)
{
	return ((Ms * HZ + 512) >> 10);
}

static inline void lock_command(void)
{
	mutex_lock(&cmd_mutex);
	cmd_done = false;
}

static inline int wait_command(void)
{
	return wait_event_interruptible_timeout(cmd_wait, cmd_done,
						msecs_to_jiffies(15));
}

static inline void unlock_command(void) { mutex_unlock(&cmd_mutex); }

static void isr_sp_task(void)
{
	MUINT32 sp_task = ccu_read_reg(ccu_base, CCU_STA_REG_SP_ISR_TASK);
	MUINT32 i2c_transac_len;
	MBOOL i2c_do_dma_en;
	unsigned long flags;

	switch (sp_task) {
	case APISR_SP_TASK_TRIGGER_I2C: {
		LOG_DBG("[%s]APISR_SP_TASK_TRIGGER_I2C +++++\n", __func__);

		i2c_transac_len =
		    ccu_read_reg(ccu_base, CCU_STA_REG_I2C_TRANSAC_LEN);
		i2c_do_dma_en =
		    ccu_read_reg(ccu_base, CCU_STA_REG_I2C_DO_DMA_EN);
		/*LOG_DBG("i2c_transac_len: %d\n", i2c_transac_len);*/
		/*LOG_DBG("i2c_do_dma_en: %d\n", i2c_do_dma_en);*/
		/*Use spinlock to avoid trigger i2c after i2c cg turned off*/
		spin_lock_irqsave(&ccuInfo.SpinLockI2cPower, flags);
		if (ccuInfo.IsI2cPoweredOn == 1 &&
		    ccuInfo.IsI2cPowerDisabling == 0)
			ccu_trigger_i2c(i2c_transac_len, i2c_do_dma_en);
		spin_unlock_irqrestore(&ccuInfo.SpinLockI2cPower, flags);

		ccu_write_reg(ccu_base, CCU_STA_REG_SP_ISR_TASK, 0);
		LOG_DBG("[%s]APISR_SP_TASK_TRIGGER_I2C -----\n", __func__);
		break;
	}

	case APISR_SP_TASK_RESET_I2C: {
		LOG_DBG("[%s]APISR_SP_TASK_RESET_I2C +++++\n", __func__);

		ccu_i2c_frame_reset();

		ccu_write_reg(ccu_base, CCU_STA_REG_SP_ISR_TASK, 0);

		LOG_DBG("[%s]APISR_SP_TASK_RESET_I2C -----\n", __func__);
		break;
	}

	default:
	{
		LOG_DBG("no %s %x\n", __func__, sp_task);
		break;
	}
	}
}

#define CCU_ISR_WORK_BUFFER_SZIE 16

irqreturn_t ccu_isr_handler(int irq, void *dev_id)
{
	enum mb_result mailboxRet;

	LOG_DBG("+++:%s\n", __func__);

	/*write clear mode*/
	LOG_DBG("write clear mode\n");
	ccu_write_reg(ccu_base, EINTC_CLR, 0xFF);
	LOG_DBG("read clear mode %d\n",
		ccu_read_reg(ccu_base, EINTC_ST));

	/**/

	isr_sp_task();

	while (1) {
		mailboxRet = mailbox_receive_cmd(&receivedCcuCmd);

		if (mailboxRet == MAILBOX_QUEUE_EMPTY) {
			LOG_DBG("MAIL_BOX IS EMPTY");
			goto ISR_EXIT;
		}

		LOG_DBG("receivedCcuCmd.msg_id : 0x%x\n",
			receivedCcuCmd.msg_id);

		switch (receivedCcuCmd.msg_id) {
		case MSG_TO_APMCU_FLUSH_LOG: {
			/*for ccu_waitirq();*/
			LOG_DBG(
				"got MSG_TO_APMCU_FLUSH_LOG:%d , wakeup ccuInfo.WaitQueueHead\n",
				receivedCcuCmd.in_data_ptr);
			bWaitCond = true;
			g_LogBufIdx = receivedCcuCmd.in_data_ptr;

			wake_up_interruptible(&ccuInfo.WaitQueueHead);
			LOG_DBG("wakeup ccuInfo.WaitQueueHead done\n");
			break;
		}
		case MSG_TO_APMCU_CCU_ASSERT: {
			LOG_ERR(
				"got MSG_TO_APMCU_CCU_ASSERT:%d, wakeup ccuInfo.WaitQueueHead\n",
				receivedCcuCmd.in_data_ptr);
			LOG_ERR("===== AP_ISR_CCU_ASSERT =====\n");
			bWaitCond = true;
			g_LogBufIdx = 0xFFFFFFFF; /* -1*/

			wake_up_interruptible(&ccuInfo.WaitQueueHead);
			ccu_i2c_dump_errr();
			LOG_ERR("wakeup ccuInfo.WaitQueueHead done\n");
			break;
		}
		case MSG_TO_APMCU_CCU_WARNING: {
			LOG_ERR(
				"got MSG_TO_APMCU_CCU_WARNING:%d, wakeup ccuInfo.WaitQueueHead\n",
				receivedCcuCmd.in_data_ptr);
				LOG_ERR("======AP_ISR_CCU_WARNING =====\n");
			bWaitCond = true;
			g_LogBufIdx = -2;

			wake_up_interruptible(&ccuInfo.WaitQueueHead);
			if ((receivedCcuCmd.in_data_ptr == 0xEE) ||
			    (receivedCcuCmd.in_data_ptr == 0xDD))
				ccu_i2c_dump_errr();
			LOG_ERR("wakeup ccuInfo.WaitQueueHead done\n");
			break;
		}
		case MSG_TO_APMCU_CAM_A_AFO_i: {
			LOG_DBG("AFWaitQueueHead:%d\n",
				receivedCcuCmd.in_data_ptr);
			LOG_DBG("======AFO_A_done_from_CCU =====\n");
			AFbWaitCond[0] = true;
			AFg_LogBufIdx[0] = 3;

			wake_up_interruptible(&ccuInfo.AFWaitQueueHead[0]);
			LOG_DBG("wakeup ccuInfo.AFWaitQueueHead done\n");
			break;
		}
		case MSG_TO_APMCU_CAM_B_AFO_i: {
			LOG_DBG("AFBWaitQueueHead:%d\n",
				receivedCcuCmd.in_data_ptr);
			LOG_DBG("===== AFO_B_done_from_CCU ======n");

			AFbWaitCond[1] = true;
			AFg_LogBufIdx[1] = 4;

			wake_up_interruptible(&ccuInfo.AFWaitQueueHead[1]);
			LOG_DBG("wakeup ccuInfo.AFBWaitQueueHead done\n");
			break;
		}
		default: {
			LOG_DBG("got msgId: %d, cmd_wait\n",
				receivedCcuCmd.msg_id);
			ccu_memcpy(&CcuAckCmd, &receivedCcuCmd,
				   sizeof(struct ccu_msg_t));

			cmd_done = true;
			wake_up_interruptible(&cmd_wait);
			break;
		}
		}
	}

ISR_EXIT:

	LOG_DBG("---:%s\n", __func__);

	/**/
	return IRQ_HANDLED;
}

static bool users_queue_is_empty(void)
{
	struct list_head *head;
	struct ccu_user_s *user;

	ccu_lock_user_mutex();

	list_for_each(head, &ccu_dev->user_list) {
		user = vlist_node_of(head, struct ccu_user_s);
		mutex_lock(&user->data_mutex);

		if (!list_empty(&user->enque_ccu_cmd_list)) {
			mutex_unlock(&user->data_mutex);
			ccu_unlock_user_mutex();
			return false;
		}
		mutex_unlock(&user->data_mutex);
	}

	ccu_unlock_user_mutex();

	return true;
}

int ccu_kenrel_fast_cmd_enque(struct ccu_cmd_s *cmd)
{
	ccu_lock_user_mutex();
	LOG_DBG("%s +:fast command %d\n",
		__func__, cmd->task.msg_id);

	ccu_send_command(cmd);
	_fast_cmd_ack = cmd;

	LOG_DBG("%s -:fast command %d\n",
		__func__, cmd->task.msg_id);
	ccu_unlock_user_mutex();

	return 0;
}

struct ccu_cmd_s *ccu_kenrel_fast_cmd_deque(void)
{
	struct ccu_cmd_s *deq_ptr;

	if (_fast_cmd_ack != NULL)
		LOG_DBG("%s +:fast command deq ok. %d\n",
			__func__, _fast_cmd_ack->task.msg_id);
	else
		LOG_DBG("%s +:fast command deq none.\n",
			__func__);

	deq_ptr = _fast_cmd_ack;
	_fast_cmd_ack = NULL;
	return deq_ptr;
}

static int ccu_enque_cmd_loop(void *arg)
{
	struct list_head *head;
	struct ccu_user_s *user;
	struct ccu_cmd_s *cmd;

	DEFINE_WAIT_FUNC(wait, woken_wake_function);

	/*set_current_state(TASK_INTERRUPTIBLE);*/
	for (; !kthread_should_stop();) {
		LOG_DBG("+:%s\n", __func__);

		/* wait commands if there is no one in user's queue */
		LOG_DBG("wait for ccu_dev->cmd_wait\n");
		add_wait_queue(&ccu_dev->cmd_wait, &wait);
		while (1) {
			if (!users_queue_is_empty()) {
				LOG_DBG("awake & condition pass\n");
				break;
			}

			wait_woken(&wait, TASK_INTERRUPTIBLE,
				   MAX_SCHEDULE_TIMEOUT);
			LOG_DBG("awake for ccu_dev->cmd_wait\n");
		}
		remove_wait_queue(&ccu_dev->cmd_wait, &wait);

		ccu_lock_user_mutex();

		/* consume the user's queue */
		list_for_each(head, &ccu_dev->user_list) {

			user = vlist_node_of(head, struct ccu_user_s);
			mutex_lock(&user->data_mutex);
			/* flush thread will handle the remaining queue if
			 * flush
			 */
			if (user->flush ||
			    list_empty(&user->enque_ccu_cmd_list)) {
				mutex_unlock(&user->data_mutex);
				continue;
			}

			/* get first node from enque list */
			cmd = vlist_node_of(user->enque_ccu_cmd_list.next,
					    struct ccu_cmd_s);

			list_del_init(vlist_link(cmd, struct ccu_cmd_s));
			user->running = true;
			mutex_unlock(&user->data_mutex);

			LOG_DBG("%s +:new command\n", __func__);
			ccu_send_command(cmd);

			mutex_lock(&user->data_mutex);
			list_add_tail(vlist_link(cmd, struct ccu_cmd_s),
				      &user->deque_ccu_cmd_list);
			user->running = false;

			LOG_DBG("list_empty(%d)\n",
				(int)list_empty(&user->deque_ccu_cmd_list));

			mutex_unlock(&user->data_mutex);

			wake_up_interruptible_all(&user->deque_wait);

			LOG_DBG("wake_up user->deque_wait done\n");
			LOG_DBG("%s -:new command\n", __func__);
		}
		ccu_unlock_user_mutex();

		/* release cpu for another operations */
		usleep_range(1, 10);
	}

	LOG_DBG("-:%s\n", __func__);
	return 0;
}

static void ccu_ap_task_mgr_init(void)
{
	mutex_init(&ap_task_manage.ApTaskMutex);
	/*
	 *if (!(ap_task_manage.ApTaskWorkQueue))
	 *{
	 *	LOG_ERR("creating ApTaskWorkQueue !!\n");
	 *	ap_task_manage.ApTaskWorkQueue =
	 *create_singlethread_workqueue("CCU_AP_WorkQueue");
	 *}
	 *if (!(ap_task_manage.ApTaskWorkQueue))
	 *{
	 *	LOG_ERR("create ApTaskWorkQueue error!!\n");
	 *}
	 */
	/*INIT_LIST_HEAD(&ap_task_manage.ApTskWorkList);*/
}

int ccu_init_hw(struct ccu_device_s *device)
{
	int ret = 0, n;

#ifdef CONFIG_MTK_CHIP
	init_check_sw_ver();
#endif

	/* init mutex */
	mutex_init(&cmd_mutex);
	/* init waitqueue */
	init_waitqueue_head(&cmd_wait);
	init_waitqueue_head(&ccuInfo.WaitQueueHead);
	init_waitqueue_head(&ccuInfo.AFWaitQueueHead[0]);
	init_waitqueue_head(&ccuInfo.AFWaitQueueHead[1]);
	/* init atomic task counter */
	/*ccuInfo.taskCount = ATOMIC_INIT(0);*/

	/* Init spinlocks */
	spin_lock_init(&(ccuInfo.SpinLockCcuRef));
	spin_lock_init(&(ccuInfo.SpinLockCcu));
	for (n = 0; n < CCU_IRQ_TYPE_AMOUNT; n++) {
		spin_lock_init(&(ccuInfo.SpinLockIrq[n]));
		spin_lock_init(&(ccuInfo.SpinLockIrqCnt[n]));
	}
	spin_lock_init(&(ccuInfo.SpinLockRTBC));
	spin_lock_init(&(ccuInfo.SpinLockClock));
	spin_lock_init(&(ccuInfo.SpinLockI2cPower));
	ccuInfo.IsI2cPoweredOn = 0;
	ccuInfo.IsI2cPowerDisabling = 0;
	/**/
	ccu_ap_task_mgr_init();

	ccu_base = device->ccu_base;
	camsys_base = device->camsys_base;
	bin_base = device->bin_base;
	dmem_base = device->dmem_base;

	ccu_dev = device;

	LOG_DBG("(0x%llx),(0x%llx),(0x%llx)\n", ccu_base, camsys_base,
		bin_base);

	if (request_irq(device->irq_num, ccu_isr_handler, IRQF_TRIGGER_NONE,
			"ccu", NULL)) {
		LOG_ERR("fail to request ccu irq!\n");
		ret = -ENODEV;
		goto out;
	}

	LOG_DBG("create ccu_enque_cmd_loop\n");
	enque_task = kthread_create(ccu_enque_cmd_loop, NULL, "ccu-enque");
	if (IS_ERR(enque_task)) {
		ret = PTR_ERR(enque_task);
		enque_task = NULL;
		goto out;
	}
	wake_up_process(enque_task);

out:
	return ret;
}

int ccu_uninit_hw(struct ccu_device_s *device)
{
	if (m4u_client != NULL)
		_ccu_deallocate_mva(&i2c_mva);

	if (enque_task) {
		kthread_stop(enque_task);
		enque_task = NULL;
	}

	flush_workqueue(ap_task_manage.ApTaskWorkQueue);
	destroy_workqueue(ap_task_manage.ApTaskWorkQueue);

	return 0;
}

int ccu_mmap_hw(struct file *filp, struct vm_area_struct *vma) { return 0; }

int ccu_get_i2c_dma_buf_addr(uint32_t *mva, uint32_t *pa_h, uint32_t *pa_l,
			     uint32_t *i2c_id)
{
	int ret = 0;
	void *va;

	ret = i2c_get_dma_buffer_addr(&va, pa_h, pa_l, i2c_id);
	LOG_DBG_MUST("got i2c buf pa: %d, %d\n", *pa_l, *pa_h);
	if (ret != 0)
		return ret;

	/*If there is existing i2c buffer mva allocated, deallocate it first*/
	_ccu_deallocate_mva(&i2c_mva);
	ret = _ccu_allocate_mva(&i2c_mva, va);
	*mva = i2c_mva;

	/*Record i2c_buffer_mva in kernel driver, thus can deallocate it at
	 * powerdown
	 */
	i2c_buffer_mva = *mva;

	return ret;
}

int ccu_memcpy(void *dest, void *src, int length)
{
	int i = 0;

	 char *destPtr = (char *)dest;
	 char *srcPtr = (char *)src;

	for (i = 0; i < length; i++)
		destPtr[i] = srcPtr[i];

	return length;
}

int ccu_memclr(void *dest, int length)
{
	int i = 0;

	 char *destPtr = (char *)dest;

	for (i = 0; i < length; i++)
		destPtr[i] = 0;

	return length;
}

int ccu_send_command(struct ccu_cmd_s *pCmd)
{
	int ret;
	/*unsigned int mva_buffers = 0;*/

	LOG_DBG("+:%s\n", __func__);

	lock_command();
	LOG_DBG("call ccu to do enque buffers\n");

	/* 1. push to mailbox_send */
	LOG_DBG("send command: id(%d), in(%x), out(%x)\n", pCmd->task.msg_id,
		pCmd->task.in_data_ptr, pCmd->task.out_data_ptr);
	mailbox_send_cmd(&(pCmd->task));

	/* 2. wait until done */
	LOG_DBG("wait ack command...\n");
	ret = wait_command();
	if (ret == 0) {
		pCmd->status = CCU_ENG_STATUS_TIMEOUT;
		LOG_ERR("timeout to wait ack command: %d\n",
			pCmd->task.msg_id);
		goto out;
	} else if (ret < 0) {
		LOG_ERR("interrupted by system signal: %d/%d\n",
			pCmd->task.msg_id, ret);

		if (ret == -ERESTARTSYS)
			LOG_ERR("interrupted as -ERESTARTSYS\n");

		pCmd->status = ret;
		goto out;
	}

	pCmd->status = CCU_ENG_STATUS_SUCCESS;

	/* 3. fill pCmd with received command */
	ccu_memcpy(&pCmd->task, &CcuAckCmd, sizeof(struct ccu_msg_t));

	LOG_DBG("got ack command: id(%d), in(%x), out(%x)\n",
		pCmd->task.msg_id, pCmd->task.in_data_ptr,
		pCmd->task.out_data_ptr);

out:

	unlock_command();

	LOG_DBG("-:%s\n", __func__);

	return ret;
}

void ccu_set_current_fps(int32_t current_fps)
{
	g_ccu_sensor_current_fps = current_fps;
	LOG_DBG_MUST("ccu catch current fps :%d\n", current_fps);
}

int32_t ccu_get_current_fps(void) { return g_ccu_sensor_current_fps; }

int ccu_power(struct ccu_power_s *power)
{
	int ret = 0;
	int32_t timeout = 10;

	LOG_DBG("+:%s,(0x%llx)(0x%llx)\n", __func__, ccu_base, camsys_base);
	LOG_DBG("power->bON: %d\n", power->bON);

	if (power->bON == 1) {
		/*CCU power on sequence*/

		/*0. Set CCU_A_RESET. CCU_HW_RST=1*/
		ccu_write_reg(ccu_base, RESET,
			      0xFF3FFCFF); /*TSF be affected.*/
		ccu_write_reg(ccu_base, RESET, 0x00010000); /*CCU_HW_RST.*/
		LOG_DBG("reset wrote\n");
		/*ccu_write_reg_bit(ccu_base, RESET, CCU_HW_RST, 1);*/

		/*1. Enable CCU CAMSYS_CG_CON bit12 CCU_CGPDN=0*/
		ccu_clock_enable();

		LOG_DBG("CG released\n");
/*mdelay(1);*/
/**/
/*ccu_write_reg_bit(ccu_base, RESET, CCU_HW_RST, 0);*/

#if 0
		/*CAMSYS_SW_RST,CCU_RST*/
		CCU_SET_BIT(camsys_base+0x0c, 24);
		CCU_SET_BIT(camsys_base+0x0c, 25);
		mdelay(1);
		CCU_CLR_BIT(camsys_base+0x0c, 24);
		CCU_CLR_BIT(camsys_base+0x0c, 25);
#endif

		/*use user space buffer*/
		ccu_write_reg(ccu_base, CCU_DATA_REG_LOG_BUF0,
			      power->workBuf.mva_log[0]);
		ccu_write_reg(ccu_base, CCU_DATA_REG_LOG_BUF1,
			      power->workBuf.mva_log[1]);

		LOG_DBG("LogBuf_mva[0](0x%x)\n", power->workBuf.mva_log[0]);
		LOG_DBG("LogBuf_mva[1](0x%x)\n", power->workBuf.mva_log[1]);

		ccuInfo.IsI2cPoweredOn = 1;
		ccuInfo.IsCcuPoweredOn = 1;
	} else if (power->bON == 0) {
		/*CCU Power off*/
		ret = _ccu_powerdown();
	} else if (power->bON == 2) {
		/*Restart CCU, no need to release CG*/

		/*0. Set CCU_A_RESET. CCU_HW_RST=1*/
		ccu_write_reg(ccu_base, RESET,
			      0xFF3FFCFF); /*TSF be affected.*/
		ccu_write_reg(ccu_base, RESET, 0x00010000); /*CCU_HW_RST.*/
		LOG_DBG("reset wrote\n");
		/*ccu_write_reg_bit(ccu_base, RESET, CCU_HW_RST, 1);*/

		/*use user space buffer*/
		ccu_write_reg(ccu_base, CCU_DATA_REG_LOG_BUF0,
			      power->workBuf.mva_log[0]);
		ccu_write_reg(ccu_base, CCU_DATA_REG_LOG_BUF1,
			      power->workBuf.mva_log[1]);

		LOG_DBG("LogBuf_mva[0](0x%x)\n", power->workBuf.mva_log[0]);
		LOG_DBG("LogBuf_mva[1](0x%x)\n", power->workBuf.mva_log[1]);
	} else if (power->bON == 3) {
		/*Pause CCU, but don't pullup CG*/

		/*Check CCU halt status*/
		while ((ccu_read_reg(ccu_base, CCU_STA_REG_SW_INIT_DONE) !=
			CCU_STATUS_INIT_DONE_2) &&
		       (timeout >= 0)) {
			mdelay(1);
			LOG_DBG("wait ccu halt done\n");
			LOG_DBG("ccu halt stat: %x\n",
				ccu_read_reg_bit(ccu_base, DONE_ST, CCU_HALT));
			timeout = timeout - 1;
		}

		if (timeout <= 0) {
			LOG_ERR("ccu_pause timeout\n");
			return -ETIMEDOUT;
		}

		/*Set CCU_A_RESET. CCU_HW_RST=1*/
		ccu_write_reg_bit(ccu_base, RESET, CCU_HW_RST, 1);

		ccuInfo.IsCcuPoweredOn = 0;

	if (m4u_client != NULL)
		_ccu_deallocate_mva(&i2c_mva);
	} else if (power->bON == 4) {
		/*CCU boot fail, just enable CG*/

		ccu_clock_disable();
		ccuInfo.IsCcuPoweredOn = 0;

	} else {
	}

	LOG_DBG("-:%s\n", __func__);
	return ret;
}

int ccu_force_powerdown(void)
{
	int ret = 0;

	if (ccuInfo.IsCcuPoweredOn == 1) {
		LOG_WARN(
		    "CCU kernel drv released on CCU running, try to force shutdown\n");
		/*Set special isr task to MSG_TO_CCU_SHUTDOWN*/
		ccu_write_reg(ccu_base, CCU_INFO29, MSG_TO_CCU_SHUTDOWN);
		/*Interrupt to CCU*/
		ccu_write_reg(ccu_base, CCU_INT, 1);

		ret = _ccu_powerdown();

		if (ret < 0)
			return ret;

		LOG_WARN("CCU force shutdown success\n");
	}

	return 0;
}

static int _ccu_powerdown(void)
{
	int32_t timeout = 10;
	unsigned long flags;

	g_ccu_sensor_current_fps = -1;

	if (ccu_read_reg_bit(ccu_base, RESET, CCU_HW_RST) == 1) {
		LOG_INF_MUST("ccu reset is up, skip halt checking.\n");
	} else {
		while ((ccu_read_reg_bit(ccu_base, DONE_ST, CCU_HALT) == 0) &&
		       timeout > 0) {
			mdelay(1);
			LOG_DBG("wait ccu shutdown done\n");
			LOG_DBG("ccu shutdown stat: %x\n",
				ccu_read_reg_bit(ccu_base, DONE_ST, CCU_HALT));
			timeout = timeout - 1;
		}

		if (timeout <= 0) {
			LOG_ERR("%s timeout\n", __func__);
			/*Even timed-out, clock disable is still necessary, DO
			 * NOT return here.
			 */
		}
	}

	/*Set CCU_A_RESET. CCU_HW_RST=1*/
	ccu_write_reg_bit(ccu_base, RESET, CCU_HW_RST, 1);
	/*CCF*/
	ccu_clock_disable();

	spin_lock_irqsave(&ccuInfo.SpinLockI2cPower, flags);
	ccuInfo.IsI2cPowerDisabling = 1;
	spin_unlock_irqrestore(&ccuInfo.SpinLockI2cPower, flags);

	ccu_i2c_buf_mode_en(0);
	ccuInfo.IsI2cPoweredOn = 0;
	ccuInfo.IsI2cPowerDisabling = 0;
	ccuInfo.IsCcuPoweredOn = 0;

	if (m4u_client != NULL)
		_ccu_deallocate_mva(&i2c_mva);

	return 0;
}

int ccu_run(void)
{
	int32_t timeout = 100;
	struct ccu_mailbox_t *ccuMbPtr = NULL;
	struct ccu_mailbox_t *apMbPtr = NULL;

	LOG_DBG("+:%s\n", __func__);

	/*smp_inner_dcache_flush_all();*/
	/*LOG_DBG("cache flushed 2\n");*/
	/*3. Set CCU_A_RESET. CCU_HW_RST=0*/
	ccu_write_reg_bit(ccu_base, RESET, CCU_HW_RST, 0);

	LOG_DBG("released CCU reset, wait for initial done, %x\n",
		ccu_read_reg(ccu_base, RESET));
	LOG_DBG("CCU reset: %x\n", ccu_read_reg(ccu_base, RESET));

	/*4. Pulling CCU init done spare register*/
	while ((ccu_read_reg(ccu_base, CCU_STA_REG_SW_INIT_DONE) !=
		CCU_STATUS_INIT_DONE) &&
	       (timeout >= 0)) {
		udelay(100);
		LOG_DBG("wait ccu initial done\n");
		LOG_DBG("ccu initial stat: %x\n",
			ccu_read_reg(ccu_base, CCU_STA_REG_SW_INIT_DONE));
		timeout = timeout - 1;
	}

	if (timeout <= 0) {
		LOG_ERR("CCU init timeout\n");
		LOG_ERR("ccu initial debug info: %x\n",
			ccu_read_reg(ccu_base, CCU_INFO28));
		return -ETIMEDOUT;
	}

	LOG_DBG("ccu initial done\n");
	LOG_DBG("ccu initial stat: %x\n",
		ccu_read_reg(ccu_base, CCU_STA_REG_SW_INIT_DONE));
	LOG_DBG("ccu initial debug info: %x\n",
		ccu_read_reg(ccu_base, CCU_INFO29));
	LOG_DBG("ccu initial debug info00: %x\n",
		ccu_read_reg(ccu_base, CCU_INFO00));
	LOG_DBG("ccu initial debug info01: %x\n",
		ccu_read_reg(ccu_base, CCU_INFO01));

	/*
	 * 20160930
	 * Due to AHB2GMC HW Bug, mailbox use SRAM
	 * Driver wait CCU main initialize done and
	 *     query INFO00 & INFO01 as mailbox address
	 */
	pMailBox[MAILBOX_SEND] =
		(struct ccu_mailbox_t *)(uintptr_t)(dmem_base +
			ccu_read_reg(ccu_base, CCU_DATA_REG_MAILBOX_CCU));
	pMailBox[MAILBOX_GET] =
		(struct ccu_mailbox_t *)(uintptr_t)(dmem_base +
			ccu_read_reg(ccu_base, CCU_DATA_REG_MAILBOX_APMCU));


	ccuMbPtr = (struct ccu_mailbox_t *) pMailBox[MAILBOX_SEND];
	apMbPtr = (struct ccu_mailbox_t *) pMailBox[MAILBOX_GET];

	mailbox_init(apMbPtr, ccuMbPtr);

	/*tell ccu that driver has initialized mailbox*/
	ccu_write_reg(ccu_base, CCU_STA_REG_SW_INIT_DONE, 0);

	timeout = 100;
	while ((ccu_read_reg(ccu_base, CCU_STA_REG_SW_INIT_DONE) !=
		CCU_STATUS_INIT_DONE_2) &&
	       (timeout >= 0)) {
		udelay(10);
		LOG_DBG("wait ccu 2nd initial done\n");
		timeout = timeout - 1;
	}

	if (timeout <= 0) {
		LOG_ERR("CCU init timeout 2\n");
		LOG_ERR("ccu initial debug info: %x\n",
			ccu_read_reg(ccu_base, CCU_INFO28));
		return -ETIMEDOUT;
	}

	LOG_DBG("ccu log test done\n");
	LOG_DBG("ccu log test stat: %x\n",
		ccu_read_reg(ccu_base, CCU_STA_REG_SW_INIT_DONE));
	LOG_DBG("ccu log test debug info: %x\n",
		ccu_read_reg(ccu_base, CCU_INFO29));

	LOG_DBG("-:%s\n", __func__);

	return 0;
}

int ccu_waitirq(struct CCU_WAIT_IRQ_STRUCT *WaitIrq)
{
	signed int ret = 0, Timeout = WaitIrq->EventInfo.Timeout;

	LOG_DBG("Clear(%d),bWaitCond(%d),Timeout(%d)\n",
		WaitIrq->EventInfo.Clear, bWaitCond, Timeout);
	LOG_DBG("arg is struct CCU_WAIT_IRQ_STRUCT, size:%zu\n",
		sizeof(struct CCU_WAIT_IRQ_STRUCT));

	if (Timeout != 0) {
		/* 2. start to wait signal */
		LOG_DBG("+:wait_event_interruptible_timeout\n");
		Timeout = wait_event_interruptible_timeout(
		    ccuInfo.WaitQueueHead, bWaitCond,
		    CCU_MsToJiffies(WaitIrq->EventInfo.Timeout));
		bWaitCond = false;
		LOG_DBG("-:wait_event_interruptible_timeout\n");
	} else {
		LOG_DBG("+:ccu wait_event_interruptible\n");
		/*task_count_temp = atomic_read(&(ccuInfo.taskCount))*/
		/*if(task_count_temp == 0)*/
		/*{*/

		mutex_unlock(&ap_task_manage.ApTaskMutex);
		LOG_DBG("unlock ApTaskMutex\n");
		wait_event_interruptible(ccuInfo.WaitQueueHead, bWaitCond);
		LOG_DBG("accuiring ApTaskMutex\n");
		mutex_lock(&ap_task_manage.ApTaskMutex);
		LOG_DBG("got ApTaskMutex\n");
		/*}*/
		/*else*/
		/*{*/
		/*LOG_DBG("ccuInfo.taskCount is not zero: %d\n",
		 * task_count_temp);
		 */
		/*}*/
		bWaitCond = false;
		LOG_DBG("-:ccu wait_event_interruptible\n");
	}

	if (Timeout > 0) {
		LOG_DBG("remain timeout:%d, task: %d\n", Timeout, g_LogBufIdx);
		/*send to user if not timeout*/
		WaitIrq->EventInfo.TimeInfo.passedbySigcnt = (int)g_LogBufIdx;
	}
	/*EXIT:*/

	return ret;
}

int ccu_AFwaitirq(struct CCU_WAIT_IRQ_STRUCT *WaitIrq, int tg_num)
{
	signed int ret = 0, Timeout = WaitIrq->EventInfo.Timeout;

	LOG_DBG("Clear(%d),AFbWaitCond(%d),Timeout(%d)\n",
		WaitIrq->EventInfo.Clear, AFbWaitCond[tg_num - 1], Timeout);
	LOG_DBG("arg is struct CCU_WAIT_IRQ_STRUCT, size:%zu\n",
		sizeof(struct CCU_WAIT_IRQ_STRUCT));

	if (Timeout != 0) {
		/* 2. start to wait signal */
		LOG_DBG("+:wait_event_interruptible_timeout\n");
		AFbWaitCond[tg_num - 1] = false;
		Timeout = wait_event_interruptible_timeout(
		    ccuInfo.AFWaitQueueHead[tg_num - 1],
		    AFbWaitCond[tg_num - 1],
		    CCU_MsToJiffies(WaitIrq->EventInfo.Timeout));

		LOG_DBG("-:wait_event_interruptible_timeout\n");
	} else {
		LOG_DBG("+:ccu wait_event_interruptible\n");
		/*task_count_temp = atomic_read(&(ccuInfo.taskCount))*/
		/*if(task_count_temp == 0)*/
		/*{*/

		mutex_unlock(&ap_task_manage.ApTaskMutex);
		LOG_DBG("unlock ApTaskMutex\n");
		wait_event_interruptible(ccuInfo.AFWaitQueueHead[tg_num - 1],
					 AFbWaitCond[tg_num - 1]);
		LOG_DBG("accuiring ApTaskMutex\n");
		mutex_lock(&ap_task_manage.ApTaskMutex);
		LOG_DBG("got ApTaskMutex\n");
		/*}*/
		/*else*/
		/*{*/
		/*LOG_DBG("ccuInfo.taskCount is not zero: %d\n",
		 * task_count_temp);
		 */
		/*}*/
		AFbWaitCond[tg_num - 1] = false;
		LOG_DBG("-:ccu wait_event_interruptible\n");
	}

	if (Timeout > 0) {
		LOG_DBG("remain timeout:%d, task: %d\n", Timeout,
			AFg_LogBufIdx[tg_num - 1]);
		/*send to user if not timeout*/
		WaitIrq->EventInfo.TimeInfo.passedbySigcnt =
		    (int)AFg_LogBufIdx[tg_num - 1];
	}
	/*EXIT:*/

	return ret;
}

int ccu_flushLog(int argc, int *argv)
{
	LOG_DBG("bWaitCond(%d)\n", bWaitCond);

	bWaitCond = true;

	wake_up_interruptible(&ccuInfo.WaitQueueHead);

	LOG_DBG("bWaitCond(%d)\n", bWaitCond);
	return 0;
}

int ccu_i2c_ctrl(unsigned char i2c_write_id, int transfer_len)
{

	LOG_DBG("+:%s\n", __func__);

	if (ccu_i2c_buf_mode_init(i2c_write_id, transfer_len) == -1) {
		LOG_DBG("ccu_i2c_buf_mode_init fail\n");
		return 0;
	}

	LOG_DBG("-:%s\n", __func__);

	return 0;
}

int ccu_read_info_reg(int regNo)
{
	int *offset = (int *)(ccu_base + 0x60 + regNo * 4);

	LOG_DBG("%s: %x\n", (unsigned int)(*offset), __func__);

	return *offset;
}

void ccu_set_sensor_info(int32_t sensorType, struct ccu_sensor_info *info)
{
	if (sensorType == IMGSENSOR_SENSOR_IDX_NONE) {
		/*Non-sensor*/
		LOG_ERR("No sensor been detected.\n");
	} else if ((sensorType >= IMGSENSOR_SENSOR_IDX_MIN_NUM) &&
		(sensorType < IMGSENSOR_SENSOR_IDX_MAX_NUM)) {
		g_ccu_sensor_info[sensorType].slave_addr  = info->slave_addr;
		g_ccu_sensor_info[sensorType].i2c_id  = info->i2c_id;
		if (info->sensor_name_string != NULL) {
			memcpy(g_ccu_sensor_name[sensorType],
			info->sensor_name_string,
			strlen(info->sensor_name_string)+1);
			g_ccu_sensor_info[sensorType].sensor_name_string =
			g_ccu_sensor_name[sensorType];
		}
		LOG_DBG_MUST("ccu catch sensor %d i2c slave address : 0x%x\n",
		sensorType, info->slave_addr);
		LOG_DBG_MUST("ccu catch sensor %d name : %s\n",
		sensorType, g_ccu_sensor_info[sensorType].sensor_name_string);
		LOG_DBG_MUST("ccu catch sensor %d i2c_id : %d\n",
		sensorType, g_ccu_sensor_info[sensorType].i2c_id);
	} else {
		LOG_DBG_MUST("ccu catch sensor i2c slave address fail!\n");
	}
}

void ccu_get_sensor_i2c_slave_addr(int32_t *sensorI2cSlaveAddr)
{
	sensorI2cSlaveAddr[0] =
		g_ccu_sensor_info[IMGSENSOR_SENSOR_IDX_MAIN].slave_addr;
	sensorI2cSlaveAddr[1] =
		g_ccu_sensor_info[IMGSENSOR_SENSOR_IDX_SUB].slave_addr;
	sensorI2cSlaveAddr[2] =
		g_ccu_sensor_info[IMGSENSOR_SENSOR_IDX_MAIN2].slave_addr;
}

void ccu_get_sensor_name(char **sensor_name)
{
	sensor_name[0] =
	g_ccu_sensor_info[IMGSENSOR_SENSOR_IDX_MAIN].sensor_name_string;
	sensor_name[1] =
	g_ccu_sensor_info[IMGSENSOR_SENSOR_IDX_SUB].sensor_name_string;
	sensor_name[2] =
	g_ccu_sensor_info[IMGSENSOR_SENSOR_IDX_MAIN2].sensor_name_string;
}


int ccu_query_power_status(void) { return ccuInfo.IsCcuPoweredOn; }

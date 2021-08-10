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

#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/ktime.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include "ion_drv.h"
#include "mtk_ion.h"
/* #include "mtk_idle.h" */
#include "mtk_spm_reg.h"
/* #include "pcm_def.h" */
/* #include "mtk_spm_idle.h" */
#include "mtk_smi.h"
#include "m4u.h"

#include "disp_drv_platform.h"
#include "debug.h"
#include "ddp_debug.h"
#include "disp_drv_log.h"
#include "disp_lcm.h"
#include "disp_utils.h"
#include "disp_session.h"
#include "primary_display.h"
#include "disp_helper.h"
#include "cmdq_def.h"
#include "cmdq_record.h"
#include "cmdq_reg.h"
#include "cmdq_core.h"
#include "ddp_manager.h"
#include "disp_lcm.h"
#include "ddp_clkmgr.h"
#include "mtk_smi.h"
/* #include "mmdvfs_mgr.h" */
#include "disp_drv_log.h"
#include "ddp_log.h"
#include "disp_lowpower.h"
/* device tree */
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/io.h>
/* #include "mach/eint.h" */
#if defined(CONFIG_MTK_LEGACY)
#include <mach/mtk_gpio.h>
#include <cust_gpio_usage.h>
#include <cust_eint.h>
#else
#include "disp_dts_gpio.h"
#include <linux/gpio.h>
#endif

#include "external_display.h"
#include "disp_recovery.h"
#include "disp_partial.h"
#include "ddp_dsi.h"


static struct task_struct *primary_display_check_task; /* For abnormal check */
static wait_queue_head_t _check_task_wq; /* used for blocking check task  */
static atomic_t _check_task_wakeup = ATOMIC_INIT(0); /* For  Check Task */

static wait_queue_head_t esd_ext_te_wq;		   /* For EXT TE EINT Check */
static atomic_t esd_ext_te_event = ATOMIC_INIT(0); /* For EXT TE EINT Check */
static unsigned int esd_check_mode;
static unsigned int esd_check_enable;

/***********external display dual LCM ESD check******************/
static struct task_struct
	*external_display_check_task; /* For abnormal check */
static wait_queue_head_t extd_check_task_wq; /* used for blocking check task */
static atomic_t extd_check_task_wakeup = ATOMIC_INIT(0); /* For  Check Task */

static wait_queue_head_t esd_ext_te_1_wq; /* For EXT TE EINT Check */
static atomic_t esd_ext_te_1_event =
	ATOMIC_INIT(0); /* For EXT TE EINT Check */
static unsigned int extd_esd_check_mode;
static unsigned int extd_esd_check_enable;

unsigned int get_esd_check_mode(void) { return esd_check_mode; }

void set_esd_check_mode(unsigned int mode) { esd_check_mode = mode; }

unsigned int _can_switch_check_mode(void)
{
	int ret = 0;

	if (primary_get_lcm()->params->dsi.customization_esd_check_enable ==
		0 &&
	    primary_get_lcm()->params->dsi.lcm_esd_check_table[0].cmd != 0)
		ret = 1;
	return ret;
}

static unsigned int _need_do_esd_check(void)
{
	int ret = 0;
#ifdef CONFIG_OF
	if ((primary_get_lcm()->params->dsi.esd_check_enable == 1) &&
	    (islcmconnected == 1))
		ret = 1;

#else
	if (primary_get_lcm()->params->dsi.esd_check_enable == 1)
		ret = 1;

#endif
	return ret;
}

/* For Cmd Mode Read LCM Check */
/* Config cmdq_handle_config_esd */
int _esd_check_config_handle_cmd(struct cmdqRecStruct *handle)
{
	int ret = 0; /* 0:success */

	/* 1.reset */
	cmdqRecReset(handle);

	primary_display_manual_lock();

	/* 2.write first instruction */
	/* cmd mode: wait CMDQ_SYNC_TOKEN_STREAM_EOF(wait trigger thread done)
	 */
	cmdqRecWaitNoClear(handle, CMDQ_SYNC_TOKEN_STREAM_EOF);

	/* 3.clear CMDQ_SYNC_TOKEN_ESD_EOF(trigger thread need wait this sync
	 * token)
	 */
	cmdqRecClearEventToken(handle, CMDQ_SYNC_TOKEN_ESD_EOF);

	/* 4.write instruction(read from lcm) */
	dpmgr_path_build_cmdq(primary_get_dpmgr_handle(), handle,
			      CMDQ_ESD_CHECK_READ, 0);

	/* 5.set CMDQ_SYNC_TOKE_ESD_EOF(trigger thread can work now) */
	cmdqRecSetEventToken(handle, CMDQ_SYNC_TOKEN_ESD_EOF);

	primary_display_manual_unlock();

	/* 6.flush instruction */
	dprec_logger_start(DPREC_LOGGER_ESD_CMDQ, 0, 0);
	ret = cmdqRecFlush(handle);
	dprec_logger_done(DPREC_LOGGER_ESD_CMDQ, 0, 0);

	DISPINFO("[ESD]%s ret=%d\n", __func__, ret);

	if (ret)
		ret = 1;
	return ret;
}

/* For Vdo Mode Read LCM Check */
/* Config cmdq_handle_config_esd */
int _esd_check_config_handle_vdo(struct cmdqRecStruct *handle)
{
	int ret = 0; /* 0:success , 1:fail */

	/* 1.reset */
	cmdqRecReset(handle);

	/* wait stream eof first */
	/*cmdqRecWait(handle, CMDQ_EVENT_DISP_RDMA0_EOF);*/
	cmdqRecWait(handle, CMDQ_EVENT_MUTEX0_STREAM_EOF);

	primary_display_manual_lock();
	/* 2.stop dsi vdo mode */
	dpmgr_path_build_cmdq(primary_get_dpmgr_handle(), handle,
			      CMDQ_STOP_VDO_MODE, 0);

	/* 3.write instruction(read from lcm) */
	dpmgr_path_build_cmdq(primary_get_dpmgr_handle(), handle,
			      CMDQ_ESD_CHECK_READ, 0);

	/* 4.start dsi vdo mode */
	dpmgr_path_build_cmdq(primary_get_dpmgr_handle(), handle,
			      CMDQ_START_VDO_MODE, 0);
	cmdqRecClearEventToken(handle, CMDQ_EVENT_MUTEX0_STREAM_EOF);
	/*cmdqRecClearEventToken(handle, CMDQ_EVENT_DISP_RDMA0_EOF);*/
	/* 5. trigger path */
	dpmgr_path_trigger(primary_get_dpmgr_handle(), handle, CMDQ_ENABLE);

	/*	mutex sof wait*/
	ddp_mutex_set_sof_wait(
	    dpmgr_path_get_mutex(primary_get_dpmgr_handle()), handle, 0);

	primary_display_manual_unlock();

	/* 6.flush instruction */
	dprec_logger_start(DPREC_LOGGER_ESD_CMDQ, 0, 0);
	ret = cmdqRecFlush(handle);
	dprec_logger_done(DPREC_LOGGER_ESD_CMDQ, 0, 0);

	DISPINFO("[ESD]%s ret=%d\n", __func__, ret);

	if (ret)
		ret = 1;
	return ret;
}

/* For EXT TE EINT Check */
static irqreturn_t _esd_check_ext_te_irq_handler(int irq, void *data)
{
	mmprofile_log_ex(ddp_mmp_get_events()->esd_vdo_eint,
			 MMPROFILE_FLAG_PULSE, 0, 0);
	atomic_set(&esd_ext_te_event, 1);
	wake_up_interruptible(&esd_ext_te_wq);
	return IRQ_HANDLED;
}

#if defined(CONFIG_MTK_LEGACY)
int primary_display_switch_esd_mode(int mode)
{
	int ret = 0;
	int gpio_mode = 0;
	struct device_node *node;
	int irq;
	u32 ints[2] = {0, 0};

#ifndef GPIO_DSI_TE_PIN
	return ret;
#endif

	gpio_mode = mt_get_gpio_mode(GPIO_DSI_TE_PIN);

	/* switch to eint mode */
	if (mode == GPIO_EINT_MODE) {
		/* 1. set mode0 */
		/* keep dsi te mode */
		/* mt_set_gpio_mode(GPIO_DSI_TE_PIN, GPIO_DSI_TE_PIN_M_GPIO);
		 */

		/* 2.register irq handler */
		node = of_find_compatible_node(NULL, NULL,
					       "mediatek, DSI_TE-eint");
		if (node) {
			of_property_read_u32_array(node, "debounce", ints,
						   ARRAY_SIZE(ints));
			mt_gpio_set_debounce(ints[0], ints[1]);
			irq = irq_of_parse_and_map(node, 0);
			if (primary_get_lcm()->params->dsi.ext_te_edge ==
			    LCM_POLARITY_FALLING) {
				if (request_irq(irq,
						_esd_check_ext_te_irq_handler,
						IRQF_TRIGGER_FALLING,
						"DSI_TE-eint", NULL))
					/*IRQF_TRIGGER_NONE*/
				pr_debug("[ESD]EINT IRQ LINE NOT AVAILABLE!!\n");
			} else {
				if (request_irq(irq,
						_esd_check_ext_te_irq_handler,
						IRQF_TRIGGER_RISING,
						"DSI_TE-eint", NULL))
					/*IRQF_TRIGGER_NONE*/
				pr_debug("[ESD]EINT IRQ LINE NOT AVAILABLE!!\n");
			}
		} else
			pr_debug(
				"[ESD][%s] can't find DSI_TE eint compatible node\n",
				__func__);
	}

	if (mode == GPIO_DSI_MODE) {

		/* 1. unregister irq handler */
		node = of_find_compatible_node(NULL, NULL,
					       "mediatek, DSI_TE-eint");
		if (node) {
			irq = irq_of_parse_and_map(node, 0);
			free_irq(irq, NULL);
		} else
			pr_debug(
				"[ESD][%s] can't find DSI_TE eint compatible node\n",
				__func__);

		/* 2. set dsi te mode */
		/* mt_set_gpio_mode(GPIO_DSI_TE_PIN, GPIO_DSI_TE_PIN_M_DSI_TE);
		 */
	}

	return ret;
}
#else
int primary_display_switch_esd_mode(int mode)
{
	int ret = 0;
	/* int gpio_mode = 0; */
	struct device_node *node;
	int irq;
	u32 ints[2] = {0, 0};

	if (mode == GPIO_EINT_MODE) {

		/* 1. set mode0 */
		/*disp_dts_gpio_select_state(DTS_GPIO_STATE_TE_MODE_GPIO);*/

		/* 2.register irq handler */
		node = of_find_compatible_node(NULL, NULL,
					       "mediatek, DSI_TE-eint");
		if (node) {
			if (of_property_read_u32_array(node, "debounce", ints,
						       ARRAY_SIZE(ints)) == 0)
				pr_debug("[%s]debounce:%d-%d\n", __func__,
					 ints[0], ints[1]);
			/*mt_gpio_set_debounce(ints[0], ints[1]); */
			else
				pr_info("[%s]debounce time not found\n",
					__func__);

			irq = irq_of_parse_and_map(node, 0);
			if (request_irq(irq, _esd_check_ext_te_irq_handler,
					IRQF_TRIGGER_RISING, "DSI_TE-eint",
					NULL))
				/*IRQF_TRIGGER_NONE*/
				pr_debug(
				    "[ESD]EINT IRQ LINE NOT AVAILABLE!!\n");

		} else
			pr_debug(
				"[ESD][%s] can't find DSI_TE eint compatible node\n",
				__func__);

	} else if (mode == GPIO_DSI_MODE) {
		/* 1. unregister irq handler */
		node = of_find_compatible_node(NULL, NULL,
					       "mediatek, DSI_TE-eint");

		if (node) {
			irq = irq_of_parse_and_map(node, 0);
			free_irq(irq, NULL);
		}

		/*disp_dts_gpio_select_state(DTS_GPIO_STATE_TE_MODE_TE);*/
	}

	return ret;
}
#endif

int do_esd_check_eint(void)
{
	int ret = 0;

	if (wait_event_interruptible_timeout(
		esd_ext_te_wq, atomic_read(&esd_ext_te_event), HZ / 2) > 0)
		ret = 0; /* esd check pass */
	else
		ret = 1; /* esd check fail */

	atomic_set(&esd_ext_te_event, 0);

	return ret;
}

int do_esd_check_dsi_te(void)
{
	int ret = 0;

	if (dpmgr_wait_event_timeout(primary_get_dpmgr_handle(),
				     DISP_PATH_EVENT_IF_VSYNC, HZ / 2) > 0)
		ret = 0; /* esd check pass */
	else
		ret = 1; /* esd check fail */

	return ret;
}

int do_esd_check_read(void)
{
	int ret = 0;
	struct cmdqRecStruct *handle;

	/* 0.create esd check cmdq */
	cmdqRecCreate(CMDQ_SCENARIO_DISP_ESD_CHECK, &handle);

	primary_display_manual_lock();
	dpmgr_path_build_cmdq(primary_get_dpmgr_handle(), handle,
			      CMDQ_ESD_ALLC_SLOT, 0);
	primary_display_manual_unlock();

	/* 1.use cmdq to read from lcm */
	if (primary_display_is_video_mode())
		ret = _esd_check_config_handle_vdo(handle);
	else
		ret = _esd_check_config_handle_cmd(handle);

	primary_display_manual_lock();

	if (ret == 1) { /* cmdq fail */
		if (need_wait_esd_eof()) {
			/* Need set esd check eof synctoken to let trigger loop
			 * go.
			 */
			cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_ESD_EOF);
		}
		/* do dsi reset */
		dpmgr_path_build_cmdq(primary_get_dpmgr_handle(), handle,
				      CMDQ_DSI_RESET, 0);
		goto destroy_cmdq;
	}

	/* 2.check data(*cpu check now) */
	ret = dpmgr_path_build_cmdq(primary_get_dpmgr_handle(), handle,
				    CMDQ_ESD_CHECK_CMP, 0);
	if (ret)
		ret = 1; /* esd check fail */

destroy_cmdq:
	dpmgr_path_build_cmdq(primary_get_dpmgr_handle(), handle,
			      CMDQ_ESD_FREE_SLOT, 0);
	primary_display_manual_unlock();

	/* 3.destroy esd config thread */
	cmdqRecDestroy(handle);

	return ret;
}
/* ESD CHECK FUNCTION */
/* return 1: esd check fail */
/* return 0: esd check pass */
int primary_display_esd_check(void)
{
	int ret = 0;
	unsigned int mode;

	dprec_logger_start(DPREC_LOGGER_ESD_CHECK, 0, 0);
	mmprofile_log_ex(ddp_mmp_get_events()->esd_check_t,
			 MMPROFILE_FLAG_START, 0, 0);
	DISPINFO("[ESD]ESD check begin\n");

	primary_display_manual_lock();
	if (primary_get_state() == DISP_SLEPT) {
		mmprofile_log_ex(ddp_mmp_get_events()->esd_check_t,
				 MMPROFILE_FLAG_PULSE, 1, 0);
		DISPINFO(
			"[ESD]primary display path is slept?? -- skip esd check\n"
			);
		primary_display_manual_unlock();
		goto done;
	}
	primary_display_manual_unlock();

	/*  Esd Check : EXT TE */
	if (primary_get_lcm()->params->dsi.customization_esd_check_enable ==
	    0) {
		/* use te for esd check */
		mmprofile_log_ex(ddp_mmp_get_events()->esd_extte,
				 MMPROFILE_FLAG_START, 0, 0);

		mode = get_esd_check_mode();
		if (mode == GPIO_EINT_MODE) {
			DISPINFO("[ESD]ESD check eint\n");
			mmprofile_log_ex(ddp_mmp_get_events()->esd_extte,
					 MMPROFILE_FLAG_PULSE,
					 primary_display_is_video_mode(),
					 mode);
			primary_display_switch_esd_mode(mode);
			ret = do_esd_check_eint();
			mode = GPIO_DSI_MODE; /* used for mode switch */
			primary_display_switch_esd_mode(mode);
		} else if (mode == GPIO_DSI_MODE) {
			mmprofile_log_ex(ddp_mmp_get_events()->esd_extte,
					 MMPROFILE_FLAG_PULSE,
					 primary_display_is_video_mode(),
					 mode);
			DISPCHECK("[ESD]ESD check read\n");
			ret = do_esd_check_read();

			mode = GPIO_EINT_MODE; /* used for mode switch */
		}
		if (disp_helper_get_option(DISP_OPT_ESD_CHECK_SWITCH))
			if (primary_display_is_video_mode()) {
				/* try eint & read switch on vdo mode */
				if (_can_switch_check_mode())
					set_esd_check_mode(mode);
			}

		mmprofile_log_ex(ddp_mmp_get_events()->esd_extte,
				 MMPROFILE_FLAG_END, 0, ret);

		goto done;
	}

	/*  Esd Check : Read from lcm */
	mmprofile_log_ex(ddp_mmp_get_events()->esd_rdlcm, MMPROFILE_FLAG_START,
			 0, primary_display_cmdq_enabled());

	if (primary_display_cmdq_enabled() == 0) {
		pr_debug("[ESD]not support cpu read do esd check\n");
		mmprofile_log_ex(ddp_mmp_get_events()->esd_rdlcm,
				 MMPROFILE_FLAG_END, 0, ret);
		goto done;
	}

	mmprofile_log_ex(ddp_mmp_get_events()->esd_rdlcm, MMPROFILE_FLAG_PULSE,
			 0, primary_display_is_video_mode());

	/* only cmd mode read & with disable mmsys clk will kick */
	if (disp_helper_get_option(DISP_OPT_IDLEMGR_ENTER_ULPS) &&
	    !primary_display_is_video_mode())
		primary_display_idlemgr_kick((char *)__func__, 1);
	ret = do_esd_check_read();

	mmprofile_log_ex(ddp_mmp_get_events()->esd_rdlcm, MMPROFILE_FLAG_END,
			 0, ret);

done:
	DISPINFO("[ESD]ESD check end, ret = %d\n", ret);
	mmprofile_log_ex(ddp_mmp_get_events()->esd_check_t, MMPROFILE_FLAG_END,
			 0, ret);
	dprec_logger_done(DPREC_LOGGER_ESD_CHECK, 0, 0);
	return ret;
}

static int primary_display_check_recovery_worker_kthread(void *data)
{
	struct sched_param param = {.sched_priority = 87};
	int ret = 0;
	int i = 0;
	int esd_try_cnt = 5; /* 20; */
	int recovery_done = 0;

	DISPFUNC();
	sched_setscheduler(current, SCHED_RR, &param);

	while (1) {
		msleep(2000); /*2s*/
		ret = wait_event_interruptible(
		    _check_task_wq, atomic_read(&_check_task_wakeup));
		if (ret < 0) {
			DISPINFO(
			    "[disp_check]check thread waked up accidently\n");
			continue;
		}

		_primary_path_switch_dst_lock();

		/* 1. esd check & recovery */
		if (esd_check_enable) {

			i = 0; /*repeat*/
			do {
				ret = primary_display_esd_check();

				if (ret == 1) {
					pr_debug(
						"[ESD]esd check fail, will do esd recovery. try=%d\n",
						i);
					primary_display_esd_recovery();
					recovery_done = 1;
				} else
					break;

			} while (++i < esd_try_cnt);

			if (ret == 1) {
			pr_debug(
				"[ESD]LCM recover fail. Try time:%d. Disable esd check\n",
				esd_try_cnt);
				primary_display_esd_check_enable(0);
			} else if (recovery_done == 1) {
				DISPCHECK("[ESD]esd recovery success\n");
				recovery_done = 0;
			}
		}

		_primary_path_switch_dst_unlock();

		/* 2. other check & recovery */

		if (kthread_should_stop())
			break;
	}
	return 0;
}

/* ESD RECOVERY */
int primary_display_esd_recovery(void)
{
	enum DISP_STATUS ret = DISP_STATUS_OK;
	struct LCM_PARAMS *lcm_param = NULL;

	DISPFUNC();
	dprec_logger_start(DPREC_LOGGER_ESD_RECOVERY, 0, 0);
	mmprofile_log_ex(ddp_mmp_get_events()->esd_recovery_t,
			 MMPROFILE_FLAG_START, 0, 0);
	DISPCHECK("[ESD]ESD recovery begin\n");
	primary_display_manual_lock();
	mmprofile_log_ex(ddp_mmp_get_events()->esd_recovery_t,
			 MMPROFILE_FLAG_PULSE, primary_display_is_video_mode(),
			 1);

	lcm_param = disp_lcm_get_params(primary_get_lcm());
	if (primary_get_state() == DISP_SLEPT) {
		DISPCHECK(
		    "[ESD]esd recovery but primary display path is slept??\n");
		goto done;
	}
	primary_display_idlemgr_kick((char *)__func__, 0);
	mmprofile_log_ex(ddp_mmp_get_events()->esd_recovery_t,
			 MMPROFILE_FLAG_PULSE, 0, 2);

	/* blocking flush before stop trigger loop */
	_blocking_flush();

	mmprofile_log_ex(ddp_mmp_get_events()->esd_recovery_t,
			 MMPROFILE_FLAG_PULSE, 0, 3);

	DISPINFO("[ESD]display cmdq trigger loop stop[begin]\n");
	_cmdq_stop_trigger_loop();
	DISPINFO("[ESD]display cmdq trigger loop stop[end]\n");

	mmprofile_log_ex(ddp_mmp_get_events()->esd_recovery_t,
			 MMPROFILE_FLAG_PULSE, 0, 4);

	DISPDBG("[ESD]stop dpmgr path[begin]\n");
	dpmgr_path_stop(primary_get_dpmgr_handle(), CMDQ_DISABLE);
	DISPCHECK("[ESD]stop dpmgr path[end]\n");
	mmprofile_log_ex(ddp_mmp_get_events()->esd_recovery_t,
			 MMPROFILE_FLAG_PULSE, 0, 0xff);

	if (dpmgr_path_is_busy(primary_get_dpmgr_handle())) {
		DISPCHECK("[ESD]primary display path is busy after stop\n");
		dpmgr_wait_event_timeout(primary_get_dpmgr_handle(),
					 DISP_PATH_EVENT_FRAME_DONE, HZ * 1);
		DISPCHECK("[ESD]wait frame done ret:%d\n", ret);
	}
	mmprofile_log_ex(ddp_mmp_get_events()->esd_recovery_t,
			 MMPROFILE_FLAG_PULSE, 0, 5);

	DISPDBG("[ESD]reset display path[begin]\n");
	dpmgr_path_reset(primary_get_dpmgr_handle(), CMDQ_DISABLE);
	DISPCHECK("[ESD]reset display path[end]\n");

	mmprofile_log_ex(ddp_mmp_get_events()->esd_recovery_t,
			 MMPROFILE_FLAG_PULSE, 0, 6);

	DISPDBG("[POWER]lcm suspend[begin]\n");
	/*after dsi_stop, we should enable the dsi basic irq.*/
	dsi_basic_irq_enable(DISP_MODULE_DSI0, NULL);
	disp_lcm_suspend(primary_get_lcm());
	DISPCHECK("[POWER]lcm suspend[end]\n");

	mmprofile_log_ex(ddp_mmp_get_events()->esd_recovery_t,
			 MMPROFILE_FLAG_PULSE, 0, 7);

	DISPDBG("[ESD]lcm force init[begin]\n");
	disp_lcm_init(primary_get_lcm(), 1);
	DISPCHECK("[ESD]lcm force init[end]\n");
	mmprofile_log_ex(ddp_mmp_get_events()->esd_recovery_t,
			 MMPROFILE_FLAG_PULSE, 0, 8);

	DISPDBG("[ESD]start dpmgr path[begin]\n");
	if (disp_partial_is_support()) {
		struct disp_ddp_path_config *data_config =
		    dpmgr_path_get_last_config(primary_get_dpmgr_handle());

		primary_display_config_full_roi(
		    data_config, primary_get_dpmgr_handle(), NULL);
	}
	dpmgr_path_start(primary_get_dpmgr_handle(), CMDQ_DISABLE);
	DISPCHECK("[ESD]start dpmgr path[end]\n");
	if (dpmgr_path_is_busy(primary_get_dpmgr_handle())) {
		pr_debug("[ESD]Main display busy before triggering SOF\n");
		ret = -1;
		/* goto done; */
	}

	mmprofile_log_ex(ddp_mmp_get_events()->esd_recovery_t,
			 MMPROFILE_FLAG_PULSE, 0, 9);
	DISPDBG("[ESD]start cmdq trigger loop[begin]\n");
	_cmdq_start_trigger_loop();
	DISPCHECK("[ESD]start cmdq trigger loop[end]\n");
	mmprofile_log_ex(ddp_mmp_get_events()->esd_recovery_t,
			 MMPROFILE_FLAG_PULSE, 0, 10);
	if (primary_display_is_video_mode()) {
		/*
		 * for video mode, we need to force trigger here
		 * for cmd mode, just set DPREC_EVENT_CMDQ_SET_EVENT_ALLOW
		 * when trigger loop start
		 */
		dpmgr_path_trigger(primary_get_dpmgr_handle(), NULL,
				   CMDQ_DISABLE);
	}
	mmprofile_log_ex(ddp_mmp_get_events()->esd_recovery_t,
			 MMPROFILE_FLAG_PULSE, 0, 11);

	/*
	 * (in suspend) when we stop trigger loop
	 * if no other thread is running, cmdq may disable its clock
	 * all cmdq event will be cleared after suspend
	 */
	cmdqCoreSetEvent(CMDQ_EVENT_DISP_WDMA0_EOF);

	/* set dirty to trigger one frame -- cmd mode */
	if (!primary_display_is_video_mode()) {
		cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
		mdelay(40);
	}
done:
	primary_display_manual_unlock();
	DISPCHECK("[ESD]ESD recovery end\n");
	mmprofile_log_ex(ddp_mmp_get_events()->esd_recovery_t,
			 MMPROFILE_FLAG_END, 0, 0);
	dprec_logger_done(DPREC_LOGGER_ESD_RECOVERY, 0, 0);
	return ret;
}

void primary_display_check_recovery_init(void)
{
	/* primary display check thread init */
	primary_display_check_task = kthread_create(
	    primary_display_check_recovery_worker_kthread, NULL, "disp_check");
	init_waitqueue_head(&_check_task_wq);

	if (disp_helper_get_option(DISP_OPT_ESD_CHECK_RECOVERY)) {
		wake_up_process(primary_display_check_task);
		if (_need_do_esd_check()) {
			/* esd check init */
			init_waitqueue_head(&esd_ext_te_wq);
			set_esd_check_mode(GPIO_EINT_MODE);
			primary_display_esd_check_enable(1);
		} else {
			atomic_set(&_check_task_wakeup, 1);
			wake_up_interruptible(&_check_task_wq);
		}
	}
}

void primary_display_esd_check_enable(int enable)
{
	if (_need_do_esd_check()) {

		if (enable) {
			esd_check_enable = 1;
			DISPCHECK("[ESD]enable esd check\n");
			atomic_set(&_check_task_wakeup, 1);
			wake_up_interruptible(&_check_task_wq);
		} else {
			esd_check_enable = 0;
			atomic_set(&_check_task_wakeup, 0);
			DISPCHECK("[ESD]disable esd check\n");
		}
	} else
		DISPCHECK("[ESD]do not support esd check\n");
}

unsigned int need_wait_esd_eof(void)
{
	int ret = 1;

	/* 1.esd check disable */
	/* 2.vdo mode */
	/* 3.cmd mode te */
	if (_need_do_esd_check() == 0)
		ret = 0;

	if (primary_display_is_video_mode())
		ret = 0;

	if (primary_get_lcm()->params->dsi.customization_esd_check_enable == 0)
		ret = 0;

	return ret;
}

/*********external display dual LCM feature************
 **********esd check*******************************
 */
static unsigned int extd_need_do_esd_check(void)
{
	int ret = 0;
	struct disp_lcm_handle *plcm = NULL;

	extd_disp_get_interface((struct disp_lcm_handle **)&plcm);

	if (plcm && plcm->params->dsi.esd_check_enable == 1)
		ret = 1;

	return ret;
}

/* For external display EXT TE EINT Check */
static irqreturn_t extd_esd_check_ext_te_irq_handler(int irq, void *data)
{
	mmprofile_log_ex(ddp_mmp_get_events()->esd_vdo_eint,
			 MMPROFILE_FLAG_PULSE, 0, 0);
	atomic_set(&esd_ext_te_1_event, 1);
	wake_up_interruptible(&esd_ext_te_1_wq);
	return IRQ_HANDLED;
}

static int extd_esd_check_eint(void)
{
	int ret = 0;

	if (wait_event_interruptible_timeout(
		esd_ext_te_1_wq, atomic_read(&esd_ext_te_1_event), HZ / 2) > 0)
		ret = 0; /* esd check pass */
	else
		ret = 1; /* esd check fail */

	atomic_set(&esd_ext_te_1_event, 0);

	return ret;
}

static unsigned int get_extd_esd_check_mode(void)
{
	return extd_esd_check_mode;
}

static void set_extd_esd_check_mode(unsigned int mode)
{
	extd_esd_check_mode = mode;
}

void external_display_esd_check_enable(int enable)
{
	if (extd_need_do_esd_check()) {

		if (enable) {
			extd_esd_check_enable = 1;
			DISPCHECK("[EXTD-ESD]enable esd check\n");
			atomic_set(&extd_check_task_wakeup, 1);
			wake_up_interruptible(&extd_check_task_wq);
		} else {
			extd_esd_check_enable = 0;
			atomic_set(&extd_check_task_wakeup, 0);
			DISPCHECK("[EXTD-ESD]disable esd check\n");
		}
	} else
		DISPCHECK("[EXTD-ESD]do not support esd check\n");
}

int external_display_switch_esd_mode(int mode)
{
	int ret = 0;
	struct device_node *node;
	int irq;
	u32 ints[2] = {0, 0};

	node = of_find_compatible_node(NULL, NULL, "mediatek, dsi_te_1-eint");
	if (node == NULL) {
		pr_debug(
		    "[EXTD-ESD][%s] can't find DSI_TE eint compatible node\n",
		    __func__);
		return ret;
	}

	if (mode == GPIO_EINT_MODE) {
		/* register irq handler */
		if (of_property_read_u32_array(node, "debounce", ints,
					       ARRAY_SIZE(ints)) == 0)
			pr_debug("[%s]debounce:%d-%d\n", __func__, ints[0],
				 ints[1]);
		/* mt_gpio_set_debounce(ints[0], ints[1]); */
		else
			pr_info("[%s]debounce time not found\n", __func__);
		irq = irq_of_parse_and_map(node, 0);
		if (request_irq(irq, extd_esd_check_ext_te_irq_handler,
				IRQF_TRIGGER_RISING, "dsi_te_1-eint", NULL))
			pr_debug("[EXTD-ESD]EINT IRQ LINE NOT AVAILABLE!!\n");
	} else if (mode == GPIO_DSI_MODE) {
		/* 1. unregister irq handler */
		irq = irq_of_parse_and_map(node, 0);
		free_irq(irq, NULL);
		/*disp_dts_gpio_select_state(DTS_GPIO_STATE_TE_MODE_TE);*/
	}

	return ret;
}

/* ESD CHECK FUNCTION */
/* return 1: esd check fail */
/* return 0: esd check pass */
int external_display_esd_check(void)
{
	int ret = 0;
	unsigned int mode;
	struct disp_lcm_handle *plcm = NULL;

	dprec_logger_start(DPREC_LOGGER_ESD_CHECK, 0, 0);
	mmprofile_log_ex(ddp_mmp_get_events()->esd_check_t,
			 MMPROFILE_FLAG_START, 0, 0);
	DISPCHECK("[EXTD-ESD]ESD check begin\n");

	if (ext_disp_is_alive() != EXTD_RESUME) {
		mmprofile_log_ex(ddp_mmp_get_events()->esd_check_t,
				 MMPROFILE_FLAG_PULSE, 1, 0);
		DISPCHECK("[EXTD-ESD]EXTD DISP is slept. skip esd check\n");
		goto done;
	}

	/*  Esd Check : EXT TE */
	extd_disp_get_interface((struct disp_lcm_handle **)&plcm);
	if (plcm && plcm->params->dsi.customization_esd_check_enable == 0) {
		/* use te for esd check */
		mmprofile_log_ex(ddp_mmp_get_events()->esd_extte,
				 MMPROFILE_FLAG_START, 0, 0);

		mode = get_extd_esd_check_mode();
		if (mode == GPIO_EINT_MODE) {
			DISPCHECK("[EXTD-ESD]ESD check eint\n");
			mmprofile_log_ex(ddp_mmp_get_events()->esd_extte,
					 MMPROFILE_FLAG_PULSE,
					 ext_disp_is_video_mode(), mode);
			external_display_switch_esd_mode(mode);
			DISPCHECK("[EXTD-ESD]ESD check begin ~\n");
			ret = extd_esd_check_eint();
			DISPCHECK("[EXTD-ESD]ESD check end, ret:%d\n", ret);
			mode = GPIO_DSI_MODE; /* used for mode switch */
			external_display_switch_esd_mode(mode);
		} else if (mode == GPIO_DSI_MODE) {
			mmprofile_log_ex(ddp_mmp_get_events()->esd_extte,
					 MMPROFILE_FLAG_PULSE,
					 ext_disp_is_video_mode(), mode);

			DISPCHECK("[EXTD-ESD]ESD check read\n");
			/*ret = do_esd_check_read();*/
			mode = GPIO_EINT_MODE; /* used for mode switch */
		}

		mmprofile_log_ex(ddp_mmp_get_events()->esd_extte,
				 MMPROFILE_FLAG_END, 0, ret);
	}

done:
	DISPCHECK("[EXTD-ESD]ESD check end, ret = %d\n", ret);
	mmprofile_log_ex(ddp_mmp_get_events()->esd_check_t, MMPROFILE_FLAG_END,
			 0, ret);
	dprec_logger_done(DPREC_LOGGER_ESD_CHECK, 0, 0);
	return ret;
}

static int external_display_check_recovery_worker_kthread(void *data)
{
	struct sched_param param = {.sched_priority = 87};
	int ret = 0;
	int i = 0;
	int esd_try_cnt = 5; /* 20; */
	int recovery_done = 0;

	DISPFUNC();
	sched_setscheduler(current, SCHED_RR, &param);

	while (1) {
		msleep(2000); /*2s*/
		ret = wait_event_interruptible(
		    extd_check_task_wq, atomic_read(&extd_check_task_wakeup));
		if (ret < 0) {
			DISPCHECK(
				"[ext_disp_check]check thread waked up accidently\n");
			continue;
		}
		pr_debug("[EXTD ext_disp_check]check thread waked up!\n");
		ext_disp_esd_check_lock();
		/* esd check and  recovery */
		if (extd_esd_check_enable) {
			i = 0; /*repeat*/
			do {
				ret = external_display_esd_check();
			if (ret != 1)
				break;
			pr_debug(
				"[EXTD-ESD]esd check fail, will do esd recovery. try=%d\n",
				i);
			ext_disp_esd_recovery();
			recovery_done = 1;

			} while (++i < esd_try_cnt);

		if (ret == 1) {
			pr_debug(
				"[EXTD-ESD]after esd recovery %d times, still fail, disable esd check\n",
				esd_try_cnt);
				external_display_esd_check_enable(0);
			} else if (recovery_done == 1) {
				DISPCHECK("[EXTD-ESD]esd recovery success\n");
				recovery_done = 0;
			}
		}
		ext_disp_esd_check_unlock();

		if (kthread_should_stop())
			break;
	}

	return 0;
}

void external_display_check_recovery_init(void)
{
	/* primary display check thread init */
	if (external_display_check_task == NULL) {
		external_display_check_task = kthread_create(
		    external_display_check_recovery_worker_kthread, NULL,
		    "extd_esd_check");
		init_waitqueue_head(&extd_check_task_wq);
		wake_up_process(external_display_check_task);
	}

	if (disp_helper_get_option(DISP_OPT_ESD_CHECK_RECOVERY) &&
	    extd_need_do_esd_check()) {
		/* esd check init */
		init_waitqueue_head(&esd_ext_te_1_wq);
		set_extd_esd_check_mode(GPIO_EINT_MODE);
		/*external_display_esd_check_enable(1);*/
	}
}

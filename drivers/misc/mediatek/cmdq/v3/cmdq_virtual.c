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

#include "cmdq_helper_ext.h"
#include "cmdq_reg.h"
#include "cmdq_device.h"
#include "cmdq_virtual.h"
#include <linux/seq_file.h>
#ifdef CMDQ_CG_M4U_LARB0
#include "m4u.h"
#endif
#ifdef CONFIG_MTK_SMI_EXT
#include "smi_public.h"
#endif

static struct cmdqCoreFuncStruct gFunctionPointer;

u64 cmdq_virtual_flag_from_scenario_default(enum CMDQ_SCENARIO_ENUM scn)
{
	u64 flag = 0;

	switch (scn) {
	case CMDQ_SCENARIO_JPEG_DEC:
		flag = (1LL << CMDQ_ENG_JPEG_DEC);
		break;

	case CMDQ_SCENARIO_SUB_MEMOUT:
		flag = ((1LL << CMDQ_ENG_DISP_OVL1) |
			(1LL << CMDQ_ENG_DISP_WDMA1));
		break;

	case CMDQ_SCENARIO_KERNEL_CONFIG_GENERAL:
		flag = 0LL;
		break;

	case CMDQ_SCENARIO_DISP_CONFIG_AAL:
	case CMDQ_SCENARIO_DISP_CONFIG_PRIMARY_GAMMA:
	case CMDQ_SCENARIO_DISP_CONFIG_PRIMARY_DITHER:
	case CMDQ_SCENARIO_DISP_CONFIG_PRIMARY_PWM:
	case CMDQ_SCENARIO_DISP_CONFIG_PRIMARY_PQ:
	case CMDQ_SCENARIO_DISP_CONFIG_SUB_GAMMA:
	case CMDQ_SCENARIO_DISP_CONFIG_SUB_DITHER:
	case CMDQ_SCENARIO_DISP_CONFIG_SUB_PWM:
	case CMDQ_SCENARIO_DISP_CONFIG_SUB_PQ:
	case CMDQ_SCENARIO_DISP_CONFIG_OD:
		flag = 0LL;
		break;
	case CMDQ_SCENARIO_TRIGGER_LOOP:
	case CMDQ_SCENARIO_HIGHP_TRIGGER_LOOP:
	case CMDQ_SCENARIO_LOWP_TRIGGER_LOOP:
		/* Trigger loop does not related to any HW by itself. */
		flag = 0LL;
		break;

	case CMDQ_SCENARIO_USER_SPACE:
		/* user space case, engine flag is passed seprately */
		flag = 0LL;
		break;

	case CMDQ_SCENARIO_DEBUG:
	case CMDQ_SCENARIO_DEBUG_PREFETCH:
		flag = 0LL;
		break;

	case CMDQ_SCENARIO_DISP_ESD_CHECK:
	case CMDQ_SCENARIO_DISP_SCREEN_CAPTURE:
		/* ESD check uses separate thread (not config, not trigger) */
		flag = 0LL;
		break;

	case CMDQ_SCENARIO_DISP_COLOR:
	case CMDQ_SCENARIO_USER_DISP_COLOR:
		/* color path */
		flag = 0LL;
		break;

	case CMDQ_SCENARIO_DISP_MIRROR_MODE:
		flag = 0LL;
		break;

	case CMDQ_SCENARIO_DISP_PRIMARY_DISABLE_SECURE_PATH:
	case CMDQ_SCENARIO_DISP_SUB_DISABLE_SECURE_PATH:
		/* secure path */
		flag = 0LL;
		break;

	default:
		if (scn < 0 || scn >= CMDQ_MAX_SCENARIO_COUNT) {
			/* Error status print */
			CMDQ_ERR("Unknown scenario type %d\n", scn);
		}
		flag = 0LL;
		break;
	}

	return flag;
}

const char *cmdq_virtual_parse_module_from_reg_addr_legacy(u32 reg_addr)
{
	const u32 addr_base_and_page = (reg_addr & 0xFFFFF000);

	/* for well-known base, we check them with 12-bit mask
	 * defined in mt_reg_base.h
	 * TODO: comfirm with SS if IO_VIRT_TO_PHYS workable when enable device
	 * tree?
	 */
	switch (addr_base_and_page) {
	case 0x14000000:
		return "MMSYS";
	case 0x14001000:
		return "MDP_RDMA0";
	case 0x14002000:
		return "MDP_RDMA1";
	case 0x14003000:
		return "MDP_RSZ0";
	case 0x14004000:
		return "MDP_RSZ1";
	case 0x14005000:
		return "MDP_RSZ2";
	case 0x14006000:
		return "MDP_WDMA";
	case 0x14007000:
		return "MDP_WROT0";
	case 0x14008000:
		return "MDP_WROT1";
	case 0x14009000:
		return "MDP_TDSHP0";
	case 0x1400A000:
		return "MDP_TDSHP1";
	case 0x1400B000:
		return "MDP_CROP";
	case 0x1400C000:
		return "DISP_OVL0";
	case 0x1400D000:
		return "DISP_OVL1";
	case 0x14013000:
		return "COLOR0";
	case 0x14014000:
		return "COLOR1";
	case 0x14015000:
		return "AAL";
	case 0x14016000:
		return "GAMA";
	case 0x14020000:
		return "MMSYS_MUTEX";
	case 0x18000000:
		return "VENC_GCON";
	case 0x18002000:
		return "VENC";
	case 0x18003000:
		return "JPGENC";
	case 0x18004000:
		return "JPGDEC";
	}

	/* for other register address we rely on GCE subsys to group them
	 * with 16-bit mask.
	 */
	return cmdq_core_parse_subsys_from_reg_addr(reg_addr);
}

/*
 * GCE capability
 */
u32 cmdq_virtual_get_subsys_LSB_in_arg_a(void)
{
	return 16;
}

/* HW thread related */
bool cmdq_virtual_is_a_secure_thread(const s32 thread)
{
#ifdef CMDQ_SECURE_PATH_SUPPORT
	if ((thread >= CMDQ_MIN_SECURE_THREAD_ID) &&
		(thread < CMDQ_MIN_SECURE_THREAD_ID +
		CMDQ_MAX_SECURE_THREAD_COUNT)) {
		return true;
	}
#endif
	return false;
}

/**
 * Scenario related
 *
 */
bool cmdq_virtual_is_disp_scenario(const enum CMDQ_SCENARIO_ENUM scenario)
{
	bool dispScenario = false;

	switch (scenario) {
	case CMDQ_SCENARIO_PRIMARY_DISP:
	case CMDQ_SCENARIO_PRIMARY_MEMOUT:
	case CMDQ_SCENARIO_PRIMARY_ALL:
	case CMDQ_SCENARIO_SUB_DISP:
	case CMDQ_SCENARIO_SUB_MEMOUT:
	case CMDQ_SCENARIO_SUB_ALL:
	case CMDQ_SCENARIO_MHL_DISP:
	case CMDQ_SCENARIO_RDMA0_DISP:
	case CMDQ_SCENARIO_RDMA1_DISP:
	case CMDQ_SCENARIO_RDMA2_DISP:
	case CMDQ_SCENARIO_RDMA0_COLOR0_DISP:
	case CMDQ_SCENARIO_TRIGGER_LOOP:
	case CMDQ_SCENARIO_HIGHP_TRIGGER_LOOP:
	case CMDQ_SCENARIO_LOWP_TRIGGER_LOOP:
	case CMDQ_SCENARIO_DISP_CONFIG_AAL:
	case CMDQ_SCENARIO_DISP_CONFIG_PRIMARY_GAMMA:
	case CMDQ_SCENARIO_DISP_CONFIG_SUB_GAMMA:
	case CMDQ_SCENARIO_DISP_CONFIG_PRIMARY_DITHER:
	case CMDQ_SCENARIO_DISP_CONFIG_SUB_DITHER:
	case CMDQ_SCENARIO_DISP_CONFIG_PRIMARY_PWM:
	case CMDQ_SCENARIO_DISP_CONFIG_SUB_PWM:
	case CMDQ_SCENARIO_DISP_CONFIG_PRIMARY_PQ:
	case CMDQ_SCENARIO_DISP_CONFIG_SUB_PQ:
	case CMDQ_SCENARIO_DISP_ESD_CHECK:
	case CMDQ_SCENARIO_DISP_SCREEN_CAPTURE:
	case CMDQ_SCENARIO_DISP_MIRROR_MODE:
	case CMDQ_SCENARIO_DISP_CONFIG_OD:
	case CMDQ_SCENARIO_DISP_VFP_CHANGE:
		/* color path */
	case CMDQ_SCENARIO_DISP_COLOR:
	case CMDQ_SCENARIO_USER_DISP_COLOR:
#ifdef CMDQ_SECURE_PATH_SUPPORT
		/* secure path */
	case CMDQ_SCENARIO_DISP_PRIMARY_DISABLE_SECURE_PATH:
	case CMDQ_SCENARIO_DISP_SUB_DISABLE_SECURE_PATH:
#endif
		dispScenario = true;
		break;
	default:
		break;
	}
	/* freely dispatch */
	return dispScenario;
}

bool cmdq_virtual_is_dynamic_scenario(
	const enum CMDQ_SCENARIO_ENUM scenario)
{
	bool dynamic_thread;

	switch (scenario) {
	case CMDQ_SCENARIO_USER_SPACE:
	case CMDQ_SCENARIO_USER_MDP:
	case CMDQ_SCENARIO_DEBUG_MDP:
		dynamic_thread = true;
		break;
	default:
		dynamic_thread = false;
		break;
	}

	return dynamic_thread;
}

bool cmdq_virtual_should_enable_prefetch(enum CMDQ_SCENARIO_ENUM scenario)
{
	bool shouldPrefetch = false;

	switch (scenario) {
	case CMDQ_SCENARIO_PRIMARY_DISP:
	case CMDQ_SCENARIO_PRIMARY_ALL:
	case CMDQ_SCENARIO_HIGHP_TRIGGER_LOOP:
	/* HACK: force debug into 0/1 thread */
	case CMDQ_SCENARIO_DEBUG_PREFETCH:
		/* any path that connects to Primary DISP HW
		 * should enable prefetch.
		 * MEMOUT scenarios does not.
		 * Also, since thread 0/1 shares one prefetch buffer,
		 * we allow only PRIMARY path to use prefetch.
		 */
		shouldPrefetch = true;
		break;
	default:
		break;
	}

	return shouldPrefetch;
}

int cmdq_virtual_disp_thread(enum CMDQ_SCENARIO_ENUM scenario)
{
	switch (scenario) {
	case CMDQ_SCENARIO_PRIMARY_DISP:
	case CMDQ_SCENARIO_PRIMARY_ALL:
	case CMDQ_SCENARIO_DISP_CONFIG_AAL:
	case CMDQ_SCENARIO_DISP_CONFIG_PRIMARY_GAMMA:
	case CMDQ_SCENARIO_DISP_CONFIG_PRIMARY_DITHER:
	case CMDQ_SCENARIO_DISP_CONFIG_PRIMARY_PWM:
	case CMDQ_SCENARIO_DISP_CONFIG_PRIMARY_PQ:
	case CMDQ_SCENARIO_DISP_CONFIG_OD:
	case CMDQ_SCENARIO_RDMA0_DISP:
	case CMDQ_SCENARIO_RDMA0_COLOR0_DISP:
	/* HACK: force debug into 0/1 thread */
	case CMDQ_SCENARIO_DEBUG_PREFETCH:
		/* primary config: thread 0 */
		return 0;

	case CMDQ_SCENARIO_SUB_DISP:
	case CMDQ_SCENARIO_SUB_ALL:
	case CMDQ_SCENARIO_RDMA1_DISP:
	case CMDQ_SCENARIO_RDMA2_DISP:
	case CMDQ_SCENARIO_DISP_CONFIG_SUB_GAMMA:
	case CMDQ_SCENARIO_DISP_CONFIG_SUB_DITHER:
	case CMDQ_SCENARIO_DISP_CONFIG_SUB_PQ:
	case CMDQ_SCENARIO_DISP_CONFIG_SUB_PWM:
	case CMDQ_SCENARIO_SUB_MEMOUT:
		return 1;

	case CMDQ_SCENARIO_MHL_DISP:
		return 5;

	case CMDQ_SCENARIO_HIGHP_TRIGGER_LOOP:
	case CMDQ_SCENARIO_DISP_VFP_CHANGE:
		return 2;

	case CMDQ_SCENARIO_DISP_ESD_CHECK:
		return 6;

	case CMDQ_SCENARIO_DISP_SCREEN_CAPTURE:
	case CMDQ_SCENARIO_DISP_MIRROR_MODE:
		return 3;

	case CMDQ_SCENARIO_DISP_COLOR:
	case CMDQ_SCENARIO_USER_DISP_COLOR:
	case CMDQ_SCENARIO_PRIMARY_MEMOUT:
		return 4;
	case CMDQ_SCENARIO_TRIGGER_LOOP:
		return 7;
	default:
		/* freely dispatch */
		return CMDQ_INVALID_THREAD;
	}
	/* freely dispatch */
	return CMDQ_INVALID_THREAD;
}

int cmdq_virtual_get_thread_index(enum CMDQ_SCENARIO_ENUM scenario,
	const bool secure)
{
#ifdef CMDQ_TIMER_ENABLE
	if (scenario == CMDQ_SCENARIO_TIMER_LOOP)
		return CMDQ_DELAY_THREAD_ID;
#endif

	if (!secure)
		return cmdq_get_func()->dispThread(scenario);

	/* dispatch secure thread according to scenario */
	switch (scenario) {
	case CMDQ_SCENARIO_DISP_PRIMARY_DISABLE_SECURE_PATH:
	case CMDQ_SCENARIO_PRIMARY_DISP:
	case CMDQ_SCENARIO_PRIMARY_ALL:
	case CMDQ_SCENARIO_RDMA0_DISP:
	case CMDQ_SCENARIO_DEBUG_PREFETCH:
		/* CMDQ_MIN_SECURE_THREAD_ID */
		return CMDQ_THREAD_SEC_PRIMARY_DISP;
	case CMDQ_SCENARIO_DISP_SUB_DISABLE_SECURE_PATH:
	case CMDQ_SCENARIO_SUB_DISP:
	case CMDQ_SCENARIO_SUB_ALL:
	case CMDQ_SCENARIO_MHL_DISP:
		/* because mirror mode and sub disp never use at the same time
		 * in secure path, dispatch to same HW thread
		 */
	case CMDQ_SCENARIO_DISP_MIRROR_MODE:
	case CMDQ_SCENARIO_DISP_COLOR:
	case CMDQ_SCENARIO_PRIMARY_MEMOUT:
		return CMDQ_THREAD_SEC_SUB_DISP;
	case CMDQ_SCENARIO_USER_MDP:
	case CMDQ_SCENARIO_USER_SPACE:
	case CMDQ_SCENARIO_DEBUG:
	case CMDQ_SCENARIO_DEBUG_MDP:
		/* because there is one input engine for MDP, reserve one
		 * secure thread is enough
		 */
		return CMDQ_THREAD_SEC_MDP;
	case CMDQ_SCENARIO_ISP_FDVT:
		return CMDQ_THREAD_SEC_ISP;
	default:
		CMDQ_ERR("no dedicated secure thread for senario:%d\n",
			scenario);
		return CMDQ_INVALID_THREAD;
	}
}

enum CMDQ_HW_THREAD_PRIORITY_ENUM cmdq_virtual_priority_from_scenario(
	enum CMDQ_SCENARIO_ENUM scenario)
{
	switch (scenario) {
	case CMDQ_SCENARIO_PRIMARY_DISP:
	case CMDQ_SCENARIO_PRIMARY_ALL:
	case CMDQ_SCENARIO_SUB_MEMOUT:
	case CMDQ_SCENARIO_SUB_DISP:
	case CMDQ_SCENARIO_SUB_ALL:
	case CMDQ_SCENARIO_RDMA1_DISP:
	case CMDQ_SCENARIO_RDMA2_DISP:
	case CMDQ_SCENARIO_MHL_DISP:
	case CMDQ_SCENARIO_RDMA0_DISP:
	case CMDQ_SCENARIO_RDMA0_COLOR0_DISP:
	case CMDQ_SCENARIO_DISP_MIRROR_MODE:
	case CMDQ_SCENARIO_PRIMARY_MEMOUT:
	case CMDQ_SCENARIO_DISP_CONFIG_AAL:
	case CMDQ_SCENARIO_DISP_CONFIG_PRIMARY_GAMMA:
	case CMDQ_SCENARIO_DISP_CONFIG_SUB_GAMMA:
	case CMDQ_SCENARIO_DISP_CONFIG_PRIMARY_DITHER:
	case CMDQ_SCENARIO_DISP_CONFIG_SUB_DITHER:
	case CMDQ_SCENARIO_DISP_CONFIG_PRIMARY_PWM:
	case CMDQ_SCENARIO_DISP_CONFIG_SUB_PWM:
	case CMDQ_SCENARIO_DISP_CONFIG_PRIMARY_PQ:
	case CMDQ_SCENARIO_DISP_CONFIG_SUB_PQ:
	case CMDQ_SCENARIO_DISP_CONFIG_OD:
	case CMDQ_SCENARIO_DISP_VFP_CHANGE:
		/* color path */
	case CMDQ_SCENARIO_DISP_COLOR:
	case CMDQ_SCENARIO_USER_DISP_COLOR:
		/* secure path */
	case CMDQ_SCENARIO_DISP_PRIMARY_DISABLE_SECURE_PATH:
	case CMDQ_SCENARIO_DISP_SUB_DISABLE_SECURE_PATH:
		/* currently, a prefetch thread is always in high priority. */
		return CMDQ_THR_PRIO_DISPLAY_CONFIG;

		/* HACK: force debug into 0/1 thread */
	case CMDQ_SCENARIO_DEBUG_PREFETCH:
		return CMDQ_THR_PRIO_DISPLAY_CONFIG;

	case CMDQ_SCENARIO_DISP_ESD_CHECK:
	case CMDQ_SCENARIO_DISP_SCREEN_CAPTURE:
		return CMDQ_THR_PRIO_DISPLAY_ESD;

	case CMDQ_SCENARIO_HIGHP_TRIGGER_LOOP:
		return CMDQ_THR_PRIO_SUPERHIGH;

	case CMDQ_SCENARIO_LOWP_TRIGGER_LOOP:
		return CMDQ_THR_PRIO_SUPERLOW;

	default:
		/* other cases need exta logic, see below. */
		break;
	}

	if (cmdq_get_func()->is_disp_loop(scenario))
		return CMDQ_THR_PRIO_DISPLAY_TRIGGER;
	else
		return CMDQ_THR_PRIO_NORMAL;
}

bool cmdq_virtual_force_loop_irq(enum CMDQ_SCENARIO_ENUM scenario)
{
	bool force_loop = false;

	if (scenario == CMDQ_SCENARIO_HIGHP_TRIGGER_LOOP ||
		scenario == CMDQ_SCENARIO_LOWP_TRIGGER_LOOP) {
		/* For monitor thread loop, we need IRQ to set callback
		 * function
		 */
		force_loop = true;
	}

	return force_loop;
}

bool cmdq_virtual_is_disp_loop(enum CMDQ_SCENARIO_ENUM scenario)
{
	bool is_disp_loop = false;

	if (scenario == CMDQ_SCENARIO_TRIGGER_LOOP)
		is_disp_loop = true;

	return is_disp_loop;
}

/**
 * Module dependent
 *
 */
void cmdq_virtual_get_reg_id_from_hwflag(u64 hwflag,
	enum cmdq_gpr_reg *valueRegId,
	enum cmdq_gpr_reg *destRegId,
	enum cmdq_event *regAccessToken)
{
	*regAccessToken = CMDQ_SYNC_TOKEN_INVALID;

	if (hwflag & (1LL << CMDQ_ENG_JPEG_ENC)) {
		*valueRegId = CMDQ_DATA_REG_JPEG;
		*destRegId = CMDQ_DATA_REG_JPEG_DST;
		*regAccessToken = CMDQ_SYNC_TOKEN_GPR_SET_0;
	} else if (hwflag & (1LL << CMDQ_ENG_MDP_TDSHP0)) {
		*valueRegId = CMDQ_DATA_REG_2D_SHARPNESS_0;
		*destRegId = CMDQ_DATA_REG_2D_SHARPNESS_0_DST;
		*regAccessToken = CMDQ_SYNC_TOKEN_GPR_SET_1;
	} else if (hwflag & (1LL << CMDQ_ENG_MDP_TDSHP1)) {
		*valueRegId = CMDQ_DATA_REG_2D_SHARPNESS_1;
		*destRegId = CMDQ_DATA_REG_2D_SHARPNESS_1_DST;
		*regAccessToken = CMDQ_SYNC_TOKEN_GPR_SET_2;
	} else if (hwflag & (1LL << CMDQ_ENG_DISP_COLOR0)) {
		*valueRegId = CMDQ_DATA_REG_PQ_COLOR;
		*destRegId = CMDQ_DATA_REG_PQ_COLOR_DST;
		*regAccessToken = CMDQ_SYNC_TOKEN_GPR_SET_3;
	} else {
		/* assume others are debug cases */
		*valueRegId = CMDQ_DATA_REG_DEBUG;
		*destRegId = CMDQ_DATA_REG_DEBUG_DST;
		*regAccessToken = CMDQ_SYNC_TOKEN_GPR_SET_4;
	}
}

const char *cmdq_virtual_module_from_event_id(const s32 event,
	struct CmdqCBkStruct *groupCallback, u64 engineFlag)
{
	const char *module = "CMDQ";
	enum CMDQ_GROUP_ENUM group = CMDQ_MAX_GROUP_COUNT;

	switch (event) {
	case CMDQ_EVENT_DISP_RDMA0_SOF:
	case CMDQ_EVENT_DISP_RDMA1_SOF:
	case CMDQ_EVENT_DISP_RDMA2_SOF:
	case CMDQ_EVENT_DISP_RDMA0_EOF:
	case CMDQ_EVENT_DISP_RDMA1_EOF:
	case CMDQ_EVENT_DISP_RDMA2_EOF:
	case CMDQ_EVENT_DISP_RDMA0_UNDERRUN:
	case CMDQ_EVENT_DISP_RDMA1_UNDERRUN:
	case CMDQ_EVENT_DISP_RDMA2_UNDERRUN:
		module = "DISP_RDMA";
		group = CMDQ_GROUP_DISP;
		break;

	case CMDQ_EVENT_DISP_WDMA0_SOF:
	case CMDQ_EVENT_DISP_WDMA1_SOF:
	case CMDQ_EVENT_DISP_WDMA0_EOF:
	case CMDQ_EVENT_DISP_WDMA1_EOF:
		module = "DISP_WDMA";
		group = CMDQ_GROUP_DISP;
		break;

	case CMDQ_EVENT_DISP_OVL0_SOF:
	case CMDQ_EVENT_DISP_OVL1_SOF:
	case CMDQ_EVENT_DISP_2L_OVL0_SOF:
	case CMDQ_EVENT_DISP_2L_OVL1_SOF:
	case CMDQ_EVENT_DISP_OVL0_EOF:
	case CMDQ_EVENT_DISP_OVL1_EOF:
	case CMDQ_EVENT_DISP_2L_OVL0_EOF:
	case CMDQ_EVENT_DISP_2L_OVL1_EOF:
		module = "DISP_OVL";
		group = CMDQ_GROUP_DISP;
		break;

	case CMDQ_EVENT_UFOD_RAMA0_L0_SOF ... CMDQ_EVENT_UFOD_RAMA1_L3_SOF:
	case CMDQ_EVENT_UFOD_RAMA0_L0_EOF ... CMDQ_EVENT_UFOD_RAMA1_L3_EOF:
		module = "DISP_UFOD";
		group = CMDQ_GROUP_DISP;
		break;

	case CMDQ_EVENT_DSI_TE:
	case CMDQ_EVENT_DSI0_TE:
	case CMDQ_EVENT_DSI1_TE:
	case CMDQ_EVENT_MDP_DSI0_TE_SOF:
	case CMDQ_EVENT_MDP_DSI1_TE_SOF:
	case CMDQ_EVENT_DISP_DSI0_EOF:
	case CMDQ_EVENT_DISP_DSI1_EOF:
	case CMDQ_EVENT_DISP_DPI0_EOF:
	case CMDQ_EVENT_DISP_COLOR_SOF ... CMDQ_EVENT_DISP_DSC_SOF:
	case CMDQ_EVENT_DISP_COLOR_EOF ... CMDQ_EVENT_DISP_DSC_EOF:
	case CMDQ_EVENT_MUTEX0_STREAM_EOF ... CMDQ_EVENT_MUTEX4_STREAM_EOF:
		module = "DISP";
		group = CMDQ_GROUP_DISP;
		break;
	case CMDQ_SYNC_TOKEN_CONFIG_DIRTY:
	case CMDQ_SYNC_TOKEN_STREAM_EOF:
		module = "DISP";
		group = CMDQ_GROUP_DISP;
		break;

	case CMDQ_EVENT_MDP_RDMA0_SOF ... CMDQ_EVENT_MDP_CROP_SOF:
	case CMDQ_EVENT_MDP_RDMA0_EOF ... CMDQ_EVENT_MDP_CROP_EOF:
	case CMDQ_EVENT_MUTEX5_STREAM_EOF ... CMDQ_EVENT_MUTEX9_STREAM_EOF:
		module = "MDP";
		group = CMDQ_GROUP_MDP;
		break;

	case CMDQ_EVENT_ISP_PASS2_2_EOF ... CMDQ_EVENT_ISP_PASS1_0_EOF:
	case CMDQ_EVENT_DIP_CQ_THREAD0_EOF ... CMDQ_EVENT_DIP_CQ_THREAD14_EOF:
	case CMDQ_EVENT_WMF_EOF:
	case CMDQ_EVENT_ISP_SENINF_CAM1_2_3_FULL:
	case CMDQ_EVENT_ISP_SENINF_CAM0_FULL:
	case CMDQ_EVENT_ISP_FRAME_DONE_A:
	case CMDQ_EVENT_ISP_FRAME_DONE_B:
	case CMDQ_EVENT_ISP_CAMSV_0_PASS1_DONE:
	case CMDQ_EVENT_ISP_CAMSV_1_PASS1_DONE:
	case CMDQ_EVENT_ISP_CAMSV_2_PASS1_DONE:
	case CMDQ_EVENT_SENINF_0_FIFO_FULL ... CMDQ_EVENT_SENINF_7_FIFO_FULL:
		module = "DIP";
		group = CMDQ_GROUP_ISP;
		break;

	case CMDQ_EVENT_DPE_EOF:
		module = "DPE";
		group = CMDQ_GROUP_ISP;
		break;

	case CMDQ_EVENT_RSC_EOF:
		module = "RSC";
		group = CMDQ_GROUP_ISP;
		break;

	case CMDQ_EVENT_WPE_A_EOF:
		module = "WPE";
		group = CMDQ_GROUP_ISP;
		break;

	case CMDQ_EVENT_MFB_DONE:
		module = "MFB";
		group = CMDQ_GROUP_ISP;
		break;

	case CMDQ_EVENT_FDVT_DONE:
		module = "FDVT";
		group = CMDQ_GROUP_ISP;
		break;

	case CMDQ_EVENT_GEPF_EOF:
	case CMDQ_EVENT_GEPF_TEMP_EOF:
	case CMDQ_EVENT_GEPF_BYPASS_EOF:
	case CMDQ_EVENT_EAF_EOF:
		module = "DIP";
		group = CMDQ_GROUP_ISP;
		break;

	case CMDQ_EVENT_JPEG_ENC_EOF:
	case CMDQ_EVENT_JPEG_ENC_PASS2_EOF:
	case CMDQ_EVENT_JPEG_ENC_PASS1_EOF:
	case CMDQ_EVENT_JPEG_DEC_EOF:
		module = "JPGE";
		group = CMDQ_GROUP_JPEG;
		break;

	case CMDQ_EVENT_VENC_EOF:
	case CMDQ_EVENT_VENC_MB_DONE:
	case CMDQ_EVENT_VENC_128BYTE_CNT_DONE:
		module = "VENC";
		group = CMDQ_GROUP_VENC;
		break;

	default:
		module = "CMDQ";
		group = CMDQ_MAX_GROUP_COUNT;
		break;
	}

	if (group < CMDQ_MAX_GROUP_COUNT && groupCallback[group].dispatchMod)
		module = groupCallback[group].dispatchMod(engineFlag);

	return module;
}

const char *cmdq_virtual_parse_module_from_reg_addr(u32 reg_addr)
{
	const u32 addr_base_and_page = (reg_addr & 0xFFFFF000);

#ifdef CMDQ_USE_LEGACY
	return cmdq_virtual_parse_module_from_reg_addr_legacy(reg_addr);
#else
	/* for well-known base, we check them with 12-bit mask
	 * defined in mt_reg_base.h
	 * TODO: comfirm with SS if IO_VIRT_TO_PHYS workable when enable
	 * device tree?
	 */
	switch (addr_base_and_page) {
	case 0x14001000: /* MDP_RDMA0 */
	case 0x14002000: /* MDP_RDMA1 */
	case 0x14003000: /* MDP_RSZ0 */
	case 0x14004000: /* MDP_RSZ1 */
	case 0x14005000: /* MDP_RSZ2 */
	case 0x14006000: /* MDP_WDMA */
	case 0x14007000: /* MDP_WROT0 */
	case 0x14008000: /* MDP_WROT1 */
	case 0x14009000: /* MDP_TDSHP */
		return "MDP";
	case 0x14014000: /* DISP_COLOR0 */
	case 0x14015000: /* DISP_COLOR1 */
		return "COLOR";
	case 0x14016000: /* DISP_COLOR0 */
	case 0x14017000: /* DISP_COLOR1 */
		return "CCORR";
	case 0x1400B000: /* DISP_OVL0 */
	case 0x1400D000: /* DISP_OVL0_2L */
		return "OVL0";
	case 0x1400C000: /* DISP_OVL1 */
	case 0x1400E000: /* DISP_OVL1_2L */
		return "OVL1";
	case 0x14018000: /* DISP_AAL0 */
	case 0x14019000: /* DISP_AAL1 */
	case 0x1401a000: /* DISP_GAMMA0 */
	case 0x1401b000: /* DISP_GAMMA1 */
		return "AAL";
	case 0x17020000: /* VENC */
		return "VENC";
	case 0x17030000: /* JPGENC */
		return "JPGENC";
	case 0x17040000: /* JPGDEC */
		return "JPGDEC";
	}

	/* for other register address we rely on GCE subsys to group them
	 * with 16-bit mask.
	 */
	return cmdq_core_parse_subsys_from_reg_addr(reg_addr);
#endif
}

s32 cmdq_virtual_can_module_entry_suspend(struct EngineStruct *engineList)
{
	s32 status = 0;
	int i;
	enum CMDQ_ENG_ENUM e = 0;

	enum CMDQ_ENG_ENUM mdpEngines[] = {
		CMDQ_ENG_ISP_IMGI,
		CMDQ_ENG_MDP_RDMA0,
		CMDQ_ENG_MDP_RDMA1,
		CMDQ_ENG_MDP_RSZ0,
		CMDQ_ENG_MDP_RSZ1,
		CMDQ_ENG_MDP_RSZ2,
		CMDQ_ENG_MDP_TDSHP0,
		CMDQ_ENG_MDP_TDSHP1,
		CMDQ_ENG_MDP_COLOR0,
		CMDQ_ENG_MDP_WROT0,
		CMDQ_ENG_MDP_WROT1,
		CMDQ_ENG_MDP_WDMA
	};

	for (i = 0; i < ARRAY_SIZE(mdpEngines); i++) {
		e = mdpEngines[i];
		if (engineList[e].userCount != 0) {
			CMDQ_ERR(
				"suspend but engine %d has userCount %d, owner=%d\n",
				e, engineList[e].userCount,
				engineList[e].currOwner);
			status = -EBUSY;
		}
	}

	return status;
}

ssize_t cmdq_virtual_print_status_clock(char *buf)
{
	s32 length = 0;
	char *pBuffer = buf;

#ifdef CMDQ_PWR_AWARE
	/* MT_CG_DISP0_MUTEX_32K is removed in this platform */
	pBuffer += sprintf(pBuffer, "MT_CG_INFRA_GCE: %d\n",
		cmdq_dev_gce_clock_is_enable());

	pBuffer += sprintf(pBuffer, "\n");
#endif

	length = pBuffer - buf;
	return length;
}

void cmdq_virtual_print_status_seq_clock(struct seq_file *m)
{
#ifdef CMDQ_PWR_AWARE
	/* MT_CG_DISP0_MUTEX_32K is removed in this platform */
	seq_printf(m, "MT_CG_INFRA_GCE: %d", cmdq_dev_gce_clock_is_enable());

	seq_puts(m, "\n");
#endif
}

void cmdq_virtual_enable_common_clock_locked(bool enable)
{
#ifdef CMDQ_PWR_AWARE
	if (enable) {
		CMDQ_VERBOSE("[CLOCK] Enable SMI & LARB0 Clock\n");
		/* Use SMI clock API */
#ifdef CONFIG_MTK_SMI_EXT
		smi_bus_prepare_enable(SMI_LARB0_REG_INDX, "CMDQ", true);
#endif
	} else {
		CMDQ_VERBOSE("[CLOCK] Disable SMI & LARB0 Clock\n");
		/* disable, reverse the sequence */
#ifdef CONFIG_MTK_SMI_EXT
		smi_bus_disable_unprepare(SMI_LARB0_REG_INDX, "CMDQ", true);
#endif
	}
#endif				/* CMDQ_PWR_AWARE */
}

void cmdq_virtual_enable_gce_clock_locked(bool enable)
{
#ifdef CMDQ_PWR_AWARE
	if (enable) {
		CMDQ_VERBOSE("[CLOCK] Enable CMDQ(GCE) Clock\n");
		cmdq_dev_enable_gce_clock(enable);
	} else {
		CMDQ_VERBOSE("[CLOCK] Disable CMDQ(GCE) Clock\n");
		cmdq_dev_enable_gce_clock(enable);
	}
#endif
}


const char *cmdq_virtual_parse_handle_error_module_by_hwflag_impl(
	const struct cmdqRecStruct *pHandle)
{
	const char *module = NULL;

	if (cmdq_get_func()->isDispScenario(pHandle->scenario))
		module = "DISP";
	else
		module = cmdq_mdp_parse_handle_error_module_by_hwflag(pHandle);

	/* other case, we need to analysis instruction for more detail */
	return module;
}

const char *cmdq_virtual_parse_error_module_by_hwflag_impl(
	const struct cmdqRecStruct *task)
{
	const char *module = NULL;

	/* TODO: fill in correct dispatch module */
	if (cmdq_get_func()->isDispScenario(task->scenario))
		module = "DISP";
	else
		module = cmdq_mdp_parse_handle_error_module_by_hwflag(task);
	/* other case, we need to analysis instruction for more detail */
	return module;
}

/**
 * Debug
 *
 */
int cmdq_virtual_dump_smi(const int showSmiDump)
{
	int isSMIHang = 0;

#if defined(CONFIG_MTK_SMI_EXT) && !defined(CONFIG_FPGA_EARLY_PORTING) && \
	!defined(CONFIG_MTK_SMI_VARIANT)
	isSMIHang = smi_debug_bus_hang_detect(SMI_PARAM_BUS_OPTIMIZATION,
		showSmiDump, showSmiDump, showSmiDump);
	CMDQ_ERR("SMI Hang? = %d\n", isSMIHang);
#else
	CMDQ_LOG("[WARNING]not enable SMI dump now\n");
#endif

	return isSMIHang;
}

void cmdq_virtual_dump_gpr(void)
{
	int i = 0;
	long offset = 0;
	u32 value = 0;

	CMDQ_LOG("========= GPR dump =========\n");
	for (i = 0; i < 16; i++) {
		offset = CMDQ_GPR_R32(i);
		value = CMDQ_REG_GET32(offset);
		CMDQ_LOG("[GPR %2d]+0x%lx = 0x%08x\n", i, offset, value);
	}
	CMDQ_LOG("========= GPR dump =========\n");
}


/**
 * Record usage
 *
 */

u64 cmdq_virtual_flag_from_scenario(enum CMDQ_SCENARIO_ENUM scn)
{
	u64 flag = 0;

#ifdef CMDQ_USE_LEGACY
	cmdq_virtual_flag_from_scenario_legacy(scn);
#else
	switch (scn) {
	case CMDQ_SCENARIO_PRIMARY_DISP:
		flag = (1LL << CMDQ_ENG_DISP_OVL0) |
		    (1LL << CMDQ_ENG_DISP_COLOR0) |
		    (1LL << CMDQ_ENG_DISP_AAL) |
		    (1LL << CMDQ_ENG_DISP_GAMMA) |
		    (1LL << CMDQ_ENG_DISP_RDMA0) |
		    (1LL << CMDQ_ENG_DISP_DSI0);
		break;
	case CMDQ_SCENARIO_PRIMARY_MEMOUT:
		flag = 0LL;
		break;
	case CMDQ_SCENARIO_PRIMARY_ALL:
		flag = ((1LL << CMDQ_ENG_DISP_OVL0) |
			(1LL << CMDQ_ENG_DISP_WDMA0) |
			(1LL << CMDQ_ENG_DISP_COLOR0) |
			(1LL << CMDQ_ENG_DISP_AAL) |
			(1LL << CMDQ_ENG_DISP_GAMMA) |
			(1LL << CMDQ_ENG_DISP_RDMA0) |
			(1LL << CMDQ_ENG_DISP_DSI0));
		break;
	case CMDQ_SCENARIO_SUB_DISP:
		flag = ((1LL << CMDQ_ENG_DISP_OVL1) |
			(1LL << CMDQ_ENG_DISP_RDMA1));
		break;
	case CMDQ_SCENARIO_SUB_ALL:
		flag = ((1LL << CMDQ_ENG_DISP_OVL1) |
			(1LL << CMDQ_ENG_DISP_WDMA1) |
			(1LL << CMDQ_ENG_DISP_RDMA1));
		break;
	case CMDQ_SCENARIO_RDMA0_DISP:
		flag = ((1LL << CMDQ_ENG_DISP_RDMA0) |
			(1LL << CMDQ_ENG_DISP_DSI0));
		break;
	case CMDQ_SCENARIO_RDMA0_COLOR0_DISP:
		flag = ((1LL << CMDQ_ENG_DISP_RDMA0) |
			(1LL << CMDQ_ENG_DISP_COLOR0) |
			(1LL << CMDQ_ENG_DISP_AAL) |
			(1LL << CMDQ_ENG_DISP_GAMMA) |
			(1LL << CMDQ_ENG_DISP_DSI0));
		break;
	case CMDQ_SCENARIO_MHL_DISP:
	case CMDQ_SCENARIO_RDMA1_DISP:
		flag = ((1LL << CMDQ_ENG_DISP_RDMA1));
		break;
	default:
		flag = 0LL;
		break;
	}
#endif
	if (flag == 0)
		cmdq_virtual_flag_from_scenario_default(scn);

	return flag;
}

/**
 * Event backup
 *
 */
struct cmdq_backup_event_struct {
	enum cmdq_event EventID;
	u32 BackupValue;
};

static struct cmdq_backup_event_struct g_cmdq_backup_event[] = {
#ifdef CMDQ_EVENT_NEED_BACKUP
	{CMDQ_SYNC_TOKEN_VENC_EOF, 0,},
	{CMDQ_SYNC_TOKEN_VENC_INPUT_READY, 0,},
#endif				/* CMDQ_EVENT_NEED_BACKUP */
#ifdef CMDQ_EVENT_SVP_BACKUP
	{CMDQ_SYNC_DISP_OVL0_2NONSEC_END, 0,},
	{CMDQ_SYNC_DISP_OVL1_2NONSEC_END, 0,},
	{CMDQ_SYNC_DISP_2LOVL0_2NONSEC_END, 0,},
	{CMDQ_SYNC_DISP_2LOVL1_2NONSEC_END, 0,},
	{CMDQ_SYNC_DISP_RDMA0_2NONSEC_END, 0,},
	{CMDQ_SYNC_DISP_RDMA1_2NONSEC_END, 0,},
	{CMDQ_SYNC_DISP_WDMA0_2NONSEC_END, 0,},
	{CMDQ_SYNC_DISP_WDMA1_2NONSEC_END, 0,},
	{CMDQ_SYNC_DISP_EXT_STREAM_EOF, 0,},
#endif				/* CMDQ_EVENT_SVP_BACKUP */
};


void cmdq_virtual_event_backup(void)
{
	int i;
	int array_size = (sizeof(g_cmdq_backup_event) /
		sizeof(struct cmdq_backup_event_struct));

	for (i = 0; i < array_size; i++) {
		if (g_cmdq_backup_event[i].EventID < 0 ||
			g_cmdq_backup_event[i].EventID >= CMDQ_SYNC_TOKEN_MAX)
			continue;

		g_cmdq_backup_event[i].BackupValue = cmdqCoreGetEvent(
			g_cmdq_backup_event[i].EventID);
		CMDQ_MSG("[backup event] event: %s, value: %d\n",
			cmdq_core_get_event_name_enum(
			g_cmdq_backup_event[i].EventID),
			g_cmdq_backup_event[i].BackupValue);
	}
}

void cmdq_virtual_event_restore(void)
{
	int i;
	int array_size = (sizeof(g_cmdq_backup_event) /
		sizeof(struct cmdq_backup_event_struct));

	for (i = 0; i < array_size; i++) {
		if (g_cmdq_backup_event[i].EventID < 0 ||
			g_cmdq_backup_event[i].EventID >= CMDQ_SYNC_TOKEN_MAX)
			continue;

		CMDQ_MSG("[restore event] event: %s, value: %d\n",
			cmdq_core_get_event_name_enum(
			g_cmdq_backup_event[i].EventID),
			g_cmdq_backup_event[i].BackupValue);

		if (g_cmdq_backup_event[i].BackupValue == 1)
			cmdqCoreSetEvent(g_cmdq_backup_event[i].EventID);
		else if (g_cmdq_backup_event[i].BackupValue == 0)
			cmdqCoreClearEvent(g_cmdq_backup_event[i].EventID);
	}
}

/**
 * Test
 *
 */
void cmdq_virtual_test_setup(void)
{
	/* unconditionally set CMDQ_SYNC_TOKEN_CONFIG_ALLOW and mutex
	 * STREAM_DONE so that DISPSYS scenarios may pass check.
	 */
	cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_STREAM_EOF);
	cmdqCoreSetEvent(CMDQ_EVENT_MUTEX0_STREAM_EOF);
	cmdqCoreSetEvent(CMDQ_EVENT_MUTEX1_STREAM_EOF);
	cmdqCoreSetEvent(CMDQ_EVENT_MUTEX2_STREAM_EOF);
	cmdqCoreSetEvent(CMDQ_EVENT_MUTEX3_STREAM_EOF);
}

void cmdq_virtual_test_cleanup(void)
{
	/* do nothing */
}

void cmdq_virtual_init_module_PA_stat(void)
{
}

void cmdq_virtual_function_setting(void)
{
	struct cmdqCoreFuncStruct *pFunc;

	pFunc = &(gFunctionPointer);

	/*
	 * GCE capability
	 */
	pFunc->getSubsysLSBArgA = cmdq_virtual_get_subsys_LSB_in_arg_a;

	/* HW thread related */
	pFunc->isSecureThread = cmdq_virtual_is_a_secure_thread;

	/**
	 * Scenario related
	 *
	 */
	pFunc->isDispScenario = cmdq_virtual_is_disp_scenario;
	pFunc->isDynamic = cmdq_virtual_is_dynamic_scenario;
	pFunc->shouldEnablePrefetch = cmdq_virtual_should_enable_prefetch;
	pFunc->dispThread = cmdq_virtual_disp_thread;
	pFunc->getThreadID = cmdq_virtual_get_thread_index;
	pFunc->priority = cmdq_virtual_priority_from_scenario;
	pFunc->force_loop_irq = cmdq_virtual_force_loop_irq;
	pFunc->is_disp_loop = cmdq_virtual_is_disp_loop;

	/**
	 * Module dependent
	 *
	 */
	pFunc->getRegID = cmdq_virtual_get_reg_id_from_hwflag;
	pFunc->moduleFromEvent = cmdq_virtual_module_from_event_id;
	pFunc->parseModule = cmdq_virtual_parse_module_from_reg_addr;
	pFunc->moduleEntrySuspend = cmdq_virtual_can_module_entry_suspend;
	pFunc->printStatusClock = cmdq_virtual_print_status_clock;
	pFunc->printStatusSeqClock = cmdq_virtual_print_status_seq_clock;
	pFunc->enableGCEClockLocked = cmdq_virtual_enable_gce_clock_locked;
	pFunc->parseErrorModule =
		cmdq_virtual_parse_error_module_by_hwflag_impl;
	pFunc->parseHandleErrorModule =
		cmdq_virtual_parse_handle_error_module_by_hwflag_impl;

	/**
	 * Debug
	 *
	 */
	pFunc->dumpSMI = cmdq_virtual_dump_smi;
	pFunc->dumpGPR = cmdq_virtual_dump_gpr;

	/**
	 * Record usage
	 *
	 */
	pFunc->flagFromScenario = cmdq_virtual_flag_from_scenario;

	/**
	 * Event backup
	 *
	 */
	pFunc->eventBackup = cmdq_virtual_event_backup;
	pFunc->eventRestore = cmdq_virtual_event_restore;

	/**
	 * Test
	 *
	 */
	pFunc->testSetup = cmdq_virtual_test_setup;
	pFunc->testCleanup = cmdq_virtual_test_cleanup;
	pFunc->initModulePAStat = cmdq_virtual_init_module_PA_stat;
}

struct cmdqCoreFuncStruct *cmdq_get_func(void)
{
	return &gFunctionPointer;
}

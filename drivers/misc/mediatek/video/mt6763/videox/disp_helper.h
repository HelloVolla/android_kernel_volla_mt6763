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

#ifndef _DISP_HELPER_H_
#define _DISP_HELPER_H_

enum DISP_HELPER_OPT {
	DISP_OPT_USE_CMDQ = 0,
	DISP_OPT_USE_M4U,
	DISP_OPT_MIPITX_ON_CHIP,
	DISP_OPT_USE_DEVICE_TREE,
	DISP_OPT_FAKE_LCM_X,
	DISP_OPT_FAKE_LCM_Y,
	DISP_OPT_FAKE_LCM_WIDTH,
	DISP_OPT_FAKE_LCM_HEIGHT,
	DISP_OPT_OVL_WARM_RESET,
	DISP_OPT_DYNAMIC_SWITCH_UNDERFLOW_EN,
	/* Begin: lowpower option*/
	DISP_OPT_SODI_SUPPORT,
	DISP_OPT_IDLE_MGR,
	DISP_OPT_IDLEMGR_SWTCH_DECOUPLE,
	DISP_OPT_IDLEMGR_ENTER_ULPS,
	DISP_OPT_SHARE_SRAM,
	DISP_OPT_DYNAMIC_SWITCH_MMSYSCLK,
	DISP_OPT_DYNAMIC_RDMA_GOLDEN_SETTING,
	DISP_OPT_IDLEMGR_DISABLE_ROUTINE_IRQ,
	DISP_OPT_MET_LOG, /* for met */
	/* End: lowpower option */
	DISP_OPT_DECOUPLE_MODE_USE_RGB565,
	DISP_OPT_NO_LCM_FOR_LOW_POWER_MEASUREMENT,
	DISP_OPT_NO_LK,
	DISP_OPT_BYPASS_PQ,
	DISP_OPT_ESD_CHECK_RECOVERY,
	DISP_OPT_ESD_CHECK_SWITCH,
	DISP_OPT_PRESENT_FENCE,
	DISP_OPT_PERFORMANCE_DEBUG,
	DISP_OPT_SWITCH_DST_MODE,
	DISP_OPT_MUTEX_EOF_EN_FOR_CMD_MODE,
	DISP_OPT_SCREEN_CAP_FROM_DITHER,
	DISP_OPT_BYPASS_OVL,
	DISP_OPT_FPS_CALC_WND,
	DISP_OPT_FPS_EXT,
	DISP_OPT_FPS_EXT_INTERVAL,
	DISP_OPT_SMART_OVL,
	DISP_OPT_DYNAMIC_DEBUG,
	DISP_OPT_SHOW_VISUAL_DEBUG_INFO,
	DISP_OPT_RDMA_UNDERFLOW_AEE,
	DISP_OPT_FENCE_TIMEOUT_AEE,
	DISP_OPT_HRT,
	DISP_OPT_PARTIAL_UPDATE,
	DISP_OPT_CV_BYSUSPEND,
	DISP_OPT_DELAYED_TRIGGER,
	/* TODO: Will be removed @{*/
	DISP_OPT_SHADOW_REGISTER, /* is chip has shadow register? */
	DISP_OPT_SHADOW_MODE,     /* 0:full; 1:force_commit; 2:bypass shadow */
	/* @} */
	DISP_OPT_OVL_EXT_LAYER, /* is ovl has ext layer support? */
	DISP_OPT_REG_PARSER_RAW_DUMP,
	DISP_OPT_AOD,
	/*ARR phase 1 option*/
	DISP_OPT_ARR_PHASE_1,
	DISP_OPT_RSZ,
	DISP_OPT_DUAL_PIPE,
	/* DISP_WDMA0 sharing internally for primary and external display */
	DISP_OPT_SHARE_WDMA0,
	DISP_OPT_ROUND_CORNER,
	DISP_OPT_DC_BY_HRT,
	DISP_OPT_GMO_OPTIMIZE,

	DISP_OPT_NUM
};

enum DISP_HELPER_STAGE {
	DISP_HELPER_STAGE_EARLY_PORTING,
	DISP_HELPER_STAGE_BRING_UP,
	DISP_HELPER_STAGE_NORMAL
};

struct DISP_OPT_INFO {
	enum DISP_HELPER_OPT option;
	int value;
	int backup;
};

void disp_helper_option_init(void);
int disp_helper_get_option(enum DISP_HELPER_OPT option);
int disp_helper_set_option(enum DISP_HELPER_OPT option, int value);
int disp_helper_set_option_by_name(const char *name, int value);
int disp_helper_get_option_list(char *stringbuf, int buf_len);

enum DISP_HELPER_STAGE disp_helper_get_stage(void);
const char *disp_helper_stage_spy(void);
int disp_helper_backup_reset(struct DISP_OPT_INFO info[], int n);
int disp_helper_restore(struct DISP_OPT_INFO info[], int n);

#endif

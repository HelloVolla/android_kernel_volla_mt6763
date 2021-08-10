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

#ifndef __GED_DVFS_H__
#define __GED_DVFS_H__

#include <linux/types.h>
#include "ged_type.h"

#define GED_DVFS_UM_CAL      1

#define GED_DVFS_PROBE_TO_UM 1
#define GED_DVFS_PROBE_IN_KM 0

#define GED_NO_UM_SERVICE    -1

#define GED_DVFS_VSYNC_OFFSET_SIGNAL_EVENT 44
#define GED_FPS_CHANGE_SIGNAL_EVENT        45
#define GED_SRV_SUICIDE_EVENT              46
#define GED_LOW_POWER_MODE_SIGNAL_EVENT    47
#define GED_MHL4K_VID_SIGNAL_EVENT         48
#define GED_GAS_SIGNAL_EVENT               49
#define GED_SIGNAL_BOOST_HOST_EVENT        50
#define GED_VILTE_VID_SIGNAL_EVENT         51
#define GED_LOW_LATENCY_MODE_SIGNAL_EVENT  52

/* GED_DVFS_DIFF_THRESHOLD (us) */
#define GED_DVFS_DIFF_THRESHOLD        500

#define GED_DVFS_TIMER_BACKUP          0x5566dead

typedef enum GED_DVFS_COMMIT_TAG
{
    GED_DVFS_DEFAULT_COMMIT,
    GED_DVFS_CUSTOM_CEIL_COMMIT,
    GED_DVFS_CUSTOM_BOOST_COMMIT,
    GED_DVFS_SET_BOTTOM_COMMIT,
    GED_DVFS_SET_LIMIT_COMMIT,
    GED_DVFS_INPUT_BOOST_COMMIT
} GED_DVFS_COMMIT_TYPE;

typedef enum GED_DVFS_TUNING_MODE_TAG
{
    GED_DVFS_DEFAULT,
    GED_DVFS_LP,
    GED_DVFS_JUST_MAKE,
    GED_DVFS_PERFORMANCE
} GED_DVFS_TUNING_MODE;

#define GED_EVENT_TOUCH            (1 << 0)
#define GED_EVENT_THERMAL          (1 << 1)
#define GED_EVENT_WFD              (1 << 2)
#define GED_EVENT_MHL              (1 << 3)
#define GED_EVENT_GAS              (1 << 4)
#define GED_EVENT_LOW_POWER_MODE   (1 << 5)
#define GED_EVENT_MHL4K_VID        (1 << 6)
#define GED_EVENT_BOOST_HOST       (1 << 7)
#define GED_EVENT_VR               (1 << 8)
#define GED_EVENT_VILTE_VID        (1 << 9)
#define GED_EVENT_LCD              (1 << 10)
#define GED_EVENT_LOW_LATENCY_MODE (1 << 13)
#define GED_EVENT_DHWC             (1 << 14)

typedef void (*ged_event_change_fp)(void *private_data, int events);

bool mtk_register_ged_event_change(const char *name, ged_event_change_fp callback, void *private_data);
bool mtk_unregister_ged_event_change(const char *name);
void mtk_ged_event_notify(int events);

#define GED_EVENT_FORCE_ON       (1 << 0)
#define GED_EVENT_FORCE_OFF      (1 << 1)
#define GED_EVENT_NOT_SYNC       (1 << 2)

#define GED_VSYNC_OFFSET_NOT_SYNC -2
#define GED_VSYNC_OFFSET_SYNC     -3

typedef struct GED_DVFS_FREQ_DATA_TAG {
	unsigned int ui32Idx;
	unsigned long ulFreq;
} GED_DVFS_FREQ_DATA;

struct GED_DVFS_BW_DATA {
	unsigned int ui32MaxBW;
	unsigned int ui32AvgBW;
};

#define MAX_BW_PROFILE 5

bool ged_dvfs_cal_gpu_utilization(unsigned int *pui32Loading,
	unsigned int *pui32Block, unsigned int *pui32Idle);
void ged_dvfs_cal_gpu_utilization_force(void);

void ged_dvfs_run(unsigned long t, long phase, unsigned long ul3DFenceDoneTime);

void ged_dvfs_set_tuning_mode(GED_DVFS_TUNING_MODE eMode);
GED_DVFS_TUNING_MODE ged_dvfs_get_tuning_mode(void);

GED_ERROR ged_dvfs_vsync_offset_event_switch(GED_DVFS_VSYNC_OFFSET_SWITCH_CMD eEvent, bool bSwitch);
void ged_dvfs_vsync_offset_level_set(int i32level);
int ged_dvfs_vsync_offset_level_get(void);

unsigned int ged_dvfs_get_gpu_loading(void);
unsigned int ged_dvfs_get_gpu_blocking(void);
unsigned int ged_dvfs_get_gpu_idle(void);

unsigned long ged_query_info( GED_INFO eType);

void ged_dvfs_get_gpu_cur_freq(GED_DVFS_FREQ_DATA *psData);
void ged_dvfs_get_gpu_pre_freq(GED_DVFS_FREQ_DATA *psData);

void ged_dvfs_sw_vsync_query_data(GED_DVFS_UM_QUERY_PACK *psQueryData);

void ged_dvfs_boost_gpu_freq(void);

GED_ERROR ged_dvfs_probe(int pid);
GED_ERROR ged_dvfs_um_commit( unsigned long gpu_tar_freq, bool bFallback);

GED_ERROR  ged_dvfs_probe_signal(int signo);

void ged_dvfs_gpu_clock_switch_notify(bool bSwitch);

void ged_dvfs_reset_opp_cost(int oppsize);
void ged_dvfs_update_opp_cost(unsigned int loading,
	unsigned int TSDiff_us, unsigned long long cur_us, unsigned int idx);
uint64_t *ged_dvfs_query_opp_cost(uint64_t reset_base_us, uint64_t curTs_us);

GED_ERROR ged_dvfs_system_init(void);
void ged_dvfs_system_exit(void);
unsigned long ged_dvfs_get_last_commit_idx(void);

extern void (*ged_kpi_set_gpu_dvfs_hint_fp)(int t_gpu_target, int boost_accum_gpu);

#ifdef GED_ENABLE_FB_DVFS
extern int (*ged_kpi_gpu_dvfs_fp)(int t_gpu, int t_gpu_target,
	int target_fps_margin, unsigned int force_fallback);
extern void (*ged_kpi_trigger_fb_dvfs_fp)(void);
extern int (*ged_kpi_check_if_fallback_mode_fp)(void);
#endif

extern void (*mtk_get_gpu_dvfs_cal_freq_fp)(unsigned long *pulGpu_tar_freq, int *pmode);

extern void mtk_gpu_ged_hint(int, int);
int ged_dvfs_boost_value(void);

#if (defined(GED_ENABLE_FB_DVFS) && defined(GED_ENABLE_DYNAMIC_DVFS_MARGIN))
extern void (*mtk_dvfs_margin_value_fp)(int i32MarginValue);
extern int (*mtk_get_dvfs_margin_value_fp)(void);
extern int gx_fb_dvfs_margin;
extern unsigned int dvfs_margin_mode;
#endif

#ifdef GED_CONFIGURE_LOADING_BASE_DVFS_STEP
extern void (*mtk_loading_base_dvfs_step_fp)(int i32MarginValue);
extern int (*mtk_get_loading_base_dvfs_step_fp)(void);
#endif

#ifdef GED_ENABLE_TIMER_BASED_DVFS_MARGIN
extern void (*mtk_timer_base_dvfs_margin_fp)(int i32MarginValue);
extern int (*mtk_get_timer_base_dvfs_margin_fp)(void);
#endif

#endif

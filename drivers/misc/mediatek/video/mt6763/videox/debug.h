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

#ifndef __MTKFB_DEBUG_H
#define __MTKFB_DEBUG_H

void DBG_Init(void);
void DBG_Deinit(void);

#include "mmprofile.h"
#include "mmprofile_function.h"
extern int bypass_blank;
extern int lcm_mode_status;
extern enum UNIFIED_COLOR_FMT force_dc_buf_fmt;
extern int layer_layout_allow_non_continuous;
extern unsigned long long idle_check_interval;

#include "disp_session.h"
#include "ddp_info.h"
#include "display_recorder.h"
extern int disp_layer_info_statistic(struct disp_ddp_path_config *last_config,
				     struct disp_frame_cfg_t *cfg);

/* show fps */
#define SHOW_LAYER_FPS (8)
#define SHOW_TOTAL_FPS (1)

struct dbg_disp_info {
	/* fps */
	int layer_show[SHOW_LAYER_FPS + SHOW_TOTAL_FPS];
	int show_fps_en;
	struct fps_debug fps_info_dbg;

	/* hrt */
	unsigned long long hrt_high, hrt_low;
	int show_hrt_en;

	/* layer_en */
	int layer_en_num;
	int layer_num_en;
	uint64_t layer_size_high;
	uint64_t layer_size_low;
	int layer_size_en;

	/* path mode */
	int path_mode_en;

	/* dsi mode */
	int dsi_mode_en;

	/* background set */
	int layer_off_dbg;
	unsigned int font_size;
	int fg_clo;
	int bg_clo;

	/* show layer buffer clean */
	int dbg_cmd_update_flg;
	struct disp_frame_cfg_t dbg_cfg;

	/* monitor thread */
	int create_thread_flg;

	/* others */
	int create_buf_flg;
	struct disp_internal_buffer_info *disp_buf;
	struct task_struct *fps_monitor_task;

	/* others module*/
	int thermal_en;
};

extern struct dbg_disp_info dbg_disp;

#ifdef MTKFB_DBG
#include "disp_drv_log.h"

#define DBG_BUF_SIZE 2048
#define MAX_DBG_INDENT_LEVEL 5
#define DBG_INDENT_SIZE 3
#define MAX_DBG_MESSAGES 0

static int dbg_indent;
static int dbg_cnt;
static char dbg_buf[DBG_BUF_SIZE];
static spinlock_t dbg_spinlock = SPIN_LOCK_UNLOCKED;
extern char *debug_buffer;
extern bool is_buffer_init;
extern unsigned int dump_output;
extern unsigned int dump_output_comp;
extern void *composed_buf;

static inline void dbg_print(int level, const char *fmt, ...)
{
	if (level <= MTKFB_DBG) {
		if (!MAX_DBG_MESSAGES || dbg_cnt < MAX_DBG_MESSAGES) {
			va_list args;
			int ind = dbg_indent;
			unsigned long flags;

			spin_lock_irqsave(&dbg_spinlock, flags);
			dbg_cnt++;
			if (ind > MAX_DBG_INDENT_LEVEL)
				ind = MAX_DBG_INDENT_LEVEL;

			DISP_LOG_PRINT(ANDROID_LOG_INFO, "DBG", "%*s",
				       ind * DBG_INDENT_SIZE, "");
			va_start(args, fmt);
			vsnprintf(dbg_buf, sizeof(dbg_buf), fmt, args);
			DISP_LOG_PRINT(ANDROID_LOG_INFO, "DBG", dbg_buf);
			va_end(args);
			spin_unlock_irqrestore(&dbg_spinlock, flags);
		}
	}
}

#define DBGPRINT dbg_print

#define DBGENTER(level)                                                       \
	do {                                                                  \
		dbg_print(level, "%s: Enter\n", __func__);                    \
		dbg_indent++;                                                 \
	} while (0)

#define DBGLEAVE(level)                                                       \
	do {                                                                  \
		dbg_indent--;                                                 \
		dbg_print(level, "%s: Leave\n", __func__);                    \
	} while (0)

/* Debug Macros */

#define MTKFB_DBG_EVT_NONE 0x00000000
#define MTKFB_DBG_EVT_FUNC 0x00000001 /* Function Entry	 */
#define MTKFB_DBG_EVT_ARGU 0x00000002 /* Function Arguments */
#define MTKFB_DBG_EVT_INFO 0x00000003 /* Information		*/

#define MTKFB_DBG_EVT_MASK (MTKFB_DBG_EVT_NONE)

#define MSG(evt, fmt, args...)                                                \
	do {                                                                  \
		if ((MTKFB_DBG_EVT_##evt) & MTKFB_DBG_EVT_MASK) {             \
			DISP_LOG_PRINT(ANDROID_LOG_INFO, "DBG", fmt, ##args); \
		}                                                             \
	} while (0)

#define MSG_FUNC_ENTER(f) MSG(FUNC, "<FB_ENTER>: %s\n", __func__)
#define MSG_FUNC_LEAVE(f) MSG(FUNC, "<FB_LEAVE>: %s\n", __func__)

#else /* MTKFB_DBG */

#define DBGPRINT(level, format, ...)
#define DBGENTER(level)
#define DBGLEAVE(level)

/* Debug Macros */

#define MSG(evt, fmt, args...)
#define MSG_FUNC_ENTER()
#define MSG_FUNC_LEAVE()

extern char *debug_buffer;
extern bool is_buffer_init;
void _debug_pattern(unsigned int mva, unsigned int va, unsigned int w,
		    unsigned int h, unsigned int linepitch, unsigned int color,
		    unsigned int layerid, unsigned int bufidx);

/* defined in mtkfb.c */
extern unsigned int mtkfb_fm_auto_test(void);
extern int pan_display_test(int frame_num, int bpp);
extern int mtkfb_get_debug_state(char *stringbuf, int buf_len);

int on_screen_en(void);
void thermal_en(int value);

#ifdef CONFIG_MTK_DISPLAY_120HZ_SUPPORT
extern void primary_display_od_bypass(int bypass);
#endif

#endif /* MTKFB_DBG */

#endif /* __MTKFB_DEBUG_H */

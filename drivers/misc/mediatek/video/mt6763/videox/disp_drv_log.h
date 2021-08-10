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

#ifndef __DISP_DRV_LOG_H__
#define __DISP_DRV_LOG_H__
#include "display_recorder.h"
#include "ddp_debug.h"
#include "mt-plat/aee.h"


#define DISP_LOG_PRINT(level, sub_module, fmt, args...)                       \
	dprec_logger_pr(DPREC_LOGGER_DEBUG, fmt, ##args)

#define DISPINFO(string, args...)                                              \
	do {                                                                   \
		dprec_logger_pr(DPREC_LOGGER_DEBUG, string, ##args);           \
		if (g_mobilelog)                                               \
			pr_info("[DISP]" string, ##args);                      \
	} while (0)

#define DISPMSG(string, args...)                                               \
	do {                                                                   \
		dprec_logger_pr(DPREC_LOGGER_DEBUG, string, ##args);           \
		pr_info("[DISP]" string, ##args);                              \
	} while (0)

#define DISPCHECK(string, args...)                                             \
	do {                                                                   \
		dprec_logger_pr(DPREC_LOGGER_DEBUG, string, ##args);           \
		pr_info("[DISP]" string, ##args);                              \
	} while (0)

#define DISPERR(string, args...)                                             \
	do {                                                                  \
		dprec_logger_pr(DPREC_LOGGER_ERROR, string, ##args);          \
		pr_info("[DISP][%s #%d]ERROR:" string,                         \
			__func__, __LINE__, ##args);                           \
	} while (0)

#define DISPWARN(string, args...)                                             \
	do {                                                                  \
		dprec_logger_pr(DPREC_LOGGER_ERROR, string, ##args);          \
		pr_info("[DISP][%s #%d]warn:" string,                          \
			__func__, __LINE__, ##args);                           \
	} while (0)

#define DISPPR_FENCE(string, args...)                                         \
	do {                                                                  \
		dprec_logger_pr(DPREC_LOGGER_FENCE, string, ##args);          \
		if (g_mobilelog)                                              \
			pr_info("fence/" string, ##args);                    \
	} while (0)

#define DISPDBG(string, args...)                                              \
	do {                                                                  \
		if (ddp_debug_dbg_log_level()) {                              \
			DISPMSG(string, ##args);                              \
		}                                                             \
	} while (0)

#define DISPFUNC()                                                            \
	do {                                                                  \
		dprec_logger_pr(DPREC_LOGGER_DEBUG, "func|%s\n", __func__);   \
		if (g_mobilelog)                                              \
			pr_info("[DISP]func|%s\n", __func__);                \
	} while (0)

#define DISPDBGFUNC() DISPFUNC()

#define DISPPR_HWOP(string, args...)

#define DISP_AEE_FLAG                                                         \
	(DB_OPT_DEFAULT | DB_OPT_MMPROFILE_BUFFER |                           \
	 DB_OPT_DISPLAY_HANG_DUMP | DB_OPT_DUMP_DISPLAY)

#define aee_print(flag, string, args...)                                      \
	do {                                                                  \
		char disp_name[100];                                          \
		snprintf(disp_name, 100, "[DISP]" string, ##args);            \
		aee_kernel_warning_api(__FILE__, __LINE__, flag, disp_name,   \
				       "[DISP] error" string, ##args);        \
		pr_info("DISP error: " string, ##args);                      \
	} while (0)

#define disp_aee_print(string, args...)                                       \
	aee_print(DISP_AEE_FLAG, string, ##args)

#define disp_aee_print_with_ftrace(string, args...)                           \
	aee_print(DISP_AEE_FLAG | DB_OPT_FTRACE, string, ##args)

#define disp_aee_db_print(string, args...)                                    \
	do {                                                                  \
		pr_info("DISP error:" string, ##args);                       \
		aee_kernel_exception("DISP", "[DISP]error:%s, %d\n",          \
				     __FILE__, __LINE__);                     \
	} while (0)

#define _DISP_PRINT_FENCE_OR_ERR(is_err, string, args...)                     \
	do {                                                                  \
		if (is_err)                                                   \
			DISPERR(string, ##args);                              \
		else                                                          \
			DISPPR_FENCE(string, ##args);                         \
	} while (0)


#endif /* __DISP_DRV_LOG_H__ */

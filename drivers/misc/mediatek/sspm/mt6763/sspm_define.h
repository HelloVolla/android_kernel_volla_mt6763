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
#ifndef __SSPM_DEFINE_H__
#define __SSPM_DEFINE_H__

#include <linux/param.h> /* for HZ */
#include <asm/arch_timer.h>

#define SSPM_MBOX_MAX 5
#define SSPM_CFG_OFS_SEMA 0x048
#define SLOT_PER_MBOX 32
#define MBOX_IN_IRQ_OFS 0x0
#define MBOX_OUT_IRQ_OFS 0x4
#define MBOX_SLOT_SIZE 0x4
#define SSPM_COREDUMP_SIZE (64 * 1024)
#define SSPM_DB_SIZE (8 * 1024)
#define TIMESYNC_TIMEOUT (60 * 60 * HZ)

#define SSPM_PLT_SERV_SUPPORT (1)
#define SSPM_LOGGER_SUPPORT (1)
#define SSPM_LASTK_SUPPORT (0)

/*
 * TimeSync v2
 *	- Enabled if CONFIG_MTK_TIMER_TIMESYNC is defined.
 *	- Timesync shall be trigered by timer module only, thus
 *	 SSPM_TIMESYNC_SUPPORT shall be disabled.
 */
#ifdef CONFIG_MTK_TIMER_TIMESYNC
#define SSPM_TIMESYNC_SUPPORT (0)
#else
#define SSPM_TIMESYNC_SUPPORT (1)
#endif

#define SSPM_COREDUMP_SUPPORT (0)
#define SSPM_EMI_PROTECTION_SUPPORT (1)

#ifndef DEBUG
#undef SSPM_COREDUMP_SUPPORT
#define SSPM_COREDUMP_SUPPORT (0)
#endif

#define PLT_INIT 0x504C5401
#define PLT_LOG_ENABLE 0x504C5402
#define PLT_LASTK_READY 0x504C5403
#define PLT_COREDUMP_READY 0x504C5404
#define PLT_TIMESYNC_SYNC 0x504C5405
#define PLT_TIMESYNC_SRAM_TEST 0x504C5406

#define SSPM_MPU_REGION_ID 4

#define mtk_timer_src_count(...) arch_counter_get_cntvct(__VA_ARGS__)

struct plt_ipi_data_s {
	unsigned int cmd;
	union {
		struct {
			unsigned int phys;
			unsigned int size;
		} ctrl;
		struct {
			unsigned int enable;
		} logger;
		struct {
			unsigned int exists;
		} lastk;
		struct {
			unsigned int exists;
		} coredump;
		struct {
			unsigned int mode;
		} ts;
	} u;
};

#endif

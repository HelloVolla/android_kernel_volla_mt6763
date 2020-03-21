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

#ifndef CONNECTIVITY_BUILD_IN_ADAPTER_H
#define CONNECTIVITY_BUILD_IN_ADAPTER_H

#include <linux/kernel.h>
#include <linux/sched.h>

/*******************************************************************************
 * Clock Buffer Control
 *
 * The Connsys adaptation layer must provide Clock Buffer Control support
 * should it be available from the platform.
 * Therefore CONNADP_HAS_CLOCK_BUF_CTRL is defined based on platform chip, and
 * is used to decide if adaptation has support on Clock Buffer Control.
 *
 * Each WMT platform file must still define its own CONSYS_CLOCK_BUF_CTRL to
 * decide if it is a co-clock'd platform.
 * It is possible that the Clock Buffer Control is available on the platform,
 * but is not used by Connsys.
 *
 * For Kernel-4,4, definition of CONNADP_HAS_CLOCK_BUF_CTRL must align with:
 *	drivers/misc/mediatek/base/power/include/mtk_clkbuf_ctl.h
 *
 * Platform that wishes to use Clock Buffer Control, please be sure to #include
 * the header file above.
 ******************************************************************************/
#if defined(CONFIG_MACH_MT6739) || \
	defined(CONFIG_MACH_MT6757) || \
	defined(CONFIG_MACH_MT6758) || \
	defined(CONFIG_MACH_MT6759) || \
	defined(CONFIG_MACH_MT6771) || \
	defined(CONFIG_MACH_MT6775) || \
	defined(CONFIG_MACH_MT6763)
#define CONNADP_HAS_CLOCK_BUF_CTRL
#define KERNEL_clk_buf_ctrl connectivity_export_clk_buf_ctrl
void connectivity_export_clk_buf_ctrl(/*enum clk_buf_id*/ int id, bool onoff);
#define KERNEL_is_clk_buf_from_pmic connectivity_export_is_clk_buf_from_pmic
bool connectivity_export_is_clk_buf_from_pmic(void);
#endif

/*******************************************************************************
 * PMIC
 * Caller please be sure to #include:
 *	drivers/misc/mediatek/include/mt-plat/upmu_common.h
 ******************************************************************************/
#if defined(CONFIG_MACH_MT6739) || \
	defined(CONFIG_MACH_MT6757) || \
	defined(CONFIG_MACH_MT6758) || \
	defined(CONFIG_MACH_MT6759) || \
	defined(CONFIG_MACH_MT6771) || \
	defined(CONFIG_MACH_MT6775) || \
	defined(CONFIG_MACH_MT6763)
#define CONNADP_HAS_PMIC_API
#define KERNEL_pmic_config_interface connectivity_export_pmic_config_interface
#define KERNEL_pmic_read_interface connectivity_export_pmic_read_interface
#define KERNEL_pmic_set_register_value connectivity_export_pmic_set_register_value
#define KERNEL_pmic_get_register_value connectivity_export_pmic_get_register_value
#define KERNEL_upmu_set_reg_value connectivity_export_upmu_set_reg_value
void connectivity_export_pmic_config_interface(unsigned int RegNum, unsigned int val,
						unsigned int MASK, unsigned int SHIFT);
void connectivity_export_pmic_read_interface(unsigned int RegNum, unsigned int *val,
						unsigned int MASK, unsigned int SHIFT);
void connectivity_export_pmic_set_register_value(/*PMU_FLAGS_LIST_ENUM*/ int flagname, unsigned int val);
unsigned short connectivity_export_pmic_get_register_value(/*PMU_FLAGS_LIST_ENUM*/ int flagname);
void connectivity_export_upmu_set_reg_value(unsigned int reg, unsigned int reg_val);
#endif
#if defined(CONFIG_MACH_MT8167)
#define CONNADP_HAS_UPMU_VCN_CTRL
#define KERNEL_upmu_set_vcn35_on_ctrl_bt connectivity_export_upmu_set_vcn35_on_ctrl_bt
#define KERNEL_upmu_set_vcn35_on_ctrl_wifi connectivity_export_upmu_set_vcn35_on_ctrl_wifi
void connectivity_export_upmu_set_vcn35_on_ctrl_bt(unsigned int val);
void connectivity_export_upmu_set_vcn35_on_ctrl_wifi(unsigned int val);
#endif

/*******************************************************************************
 * MMC
 * Caller please be sure to #include:
 *	<linux/mmc/host.h>
 *	<linux/mmc/card.h>
 *	drivers/mmc/core/sdio_ops.h
 ******************************************************************************/
#define KERNEL_mmc_io_rw_direct connectivity_export_mmc_io_rw_direct
struct mmc_card;
int connectivity_export_mmc_io_rw_direct(struct mmc_card *card, int write, unsigned fn,
						unsigned addr, u8 in, u8 *out);

/*******************************************************************************
 * MT6306 I2C-based GPIO Expander
 ******************************************************************************/
#ifdef CONFIG_MTK_MT6306_GPIO_SUPPORT
#define KERNEL_mt6306_set_gpio_out connectivity_export_mt6306_set_gpio_out
#define KERNEL_mt6306_set_gpio_dir connectivity_export_mt6306_set_gpio_dir
void connectivity_export_mt6306_set_gpio_out(unsigned long pin, unsigned long output);
void connectivity_export_mt6306_set_gpio_dir(unsigned long pin, unsigned long dir);
#endif


#ifdef CONFIG_MACH_MT6757
#define CPU_BOOST y
#endif
#ifdef CONFIG_MACH_MT6758
#define CPU_BOOST y
#endif
#ifdef CONFIG_MACH_MT6763
#define CPU_BOOST y
#endif
#ifdef CONFIG_MACH_MT6799
#define CPU_BOOST y
#endif
#ifdef CONFIG_MACH_MT6739
#define CPU_BOOST y
#endif
#ifdef CPU_BOOST
#include "mach/mtk_ppm_api.h"
#include "mtk_spm_resource_req.h"
#endif

#define KERNEL_slp_get_wake_reason connectivity_export_slp_get_wake_reason
#define KERNEL_spm_get_last_wakeup_src connectivity_export_spm_get_last_wakeup_src
#define KERNEL_show_stack connectivity_export_show_stack
#define KERNEL_tracing_record_cmdline connectivity_export_tracing_record_cmdline
#define KERNEL_dump_thread_state connectivity_export_dump_thread_state

#ifdef CPU_BOOST
#define KERNEL_mt_ppm_sysboost_freq connectivity_export_mt_ppm_sysboost_freq
#define KERNEL_mt_ppm_sysboost_core connectivity_export_mt_ppm_sysboost_core
#define KERNEL_mt_ppm_sysboost_set_core_limit connectivity_export_mt_ppm_sysboost_set_core_limit
#define KERNEL_mt_ppm_sysboost_set_freq_limit connectivity_export_mt_ppm_sysboost_set_freq_limit
#define KERNEL_spm_resource_req  connectivity_export_spm_resource_req
#else
#define KERNEL_mt_ppm_sysboost_freq
#define KERNEL_mt_ppm_sysboost_core
#define KERNEL_mt_ppm_sysboost_set_core_limit
#define KERNEL_mt_ppm_sysboost_set_freq_limit
#define KERNEL_spm_resource_req
#endif

unsigned int connectivity_export_slp_get_wake_reason(void);
unsigned int connectivity_export_spm_get_last_wakeup_src(void);
extern void tracing_record_cmdline(struct task_struct *tsk);
extern void show_stack(struct task_struct *tsk, unsigned long *sp);
#ifdef CPU_BOOST
extern void mt_ppm_sysboost_freq(enum ppm_sysboost_user user, unsigned int freq);
extern void mt_ppm_sysboost_core(enum ppm_sysboost_user user, unsigned int core_num);
extern void mt_ppm_sysboost_set_core_limit(enum ppm_sysboost_user user, unsigned int cluster,
					   int min_core, int max_core);
extern void mt_ppm_sysboost_set_freq_limit(enum ppm_sysboost_user user, unsigned int cluster,
					   int min_freq, int max_freq);
extern bool spm_resource_req(unsigned int user, unsigned int req_mask);
#endif

void connectivity_export_show_stack(struct task_struct *tsk, unsigned long *sp);
void connectivity_export_dump_thread_state(const char *name);
void connectivity_export_tracing_record_cmdline(struct task_struct *tsk);
#ifdef CPU_BOOST
void connectivity_export_mt_ppm_sysboost_freq(enum ppm_sysboost_user user, unsigned int freq);
void connectivity_export_mt_ppm_sysboost_core(enum ppm_sysboost_user user, unsigned int core_num);
void connectivity_export_mt_ppm_sysboost_set_core_limit(enum ppm_sysboost_user user, unsigned int cluster,
							int min_core, int max_core);
void connectivity_export_mt_ppm_sysboost_set_freq_limit(enum ppm_sysboost_user user, unsigned int cluster,
							int min_freq, int max_freq);
bool connectivity_export_spm_resource_req(unsigned int user, unsigned int req_mask);
#endif

/*********************************************
 * copy from
 * kernel-3.18/include/linux/ftrace_event.h
 * kernel-4.4/include/linux/trace_events.h
 *
 * event_trace_printk()
 *********************************************/

#ifndef KERNEL_event_trace_printk
#define KERNEL_event_trace_printk(ip, fmt, args...)               \
do {                                                              \
	__trace_printk_check_format(fmt, ##args);                 \
	KERNEL_tracing_record_cmdline(current);                   \
	if (__builtin_constant_p(fmt)) {                          \
		static const char *trace_printk_fmt               \
		__attribute__((section("__trace_printk_fmt"))) =  \
		__builtin_constant_p(fmt) ? fmt : NULL;           \
		__trace_bprintk(ip, trace_printk_fmt, ##args);    \
	} else                                                    \
		__trace_printk(ip, fmt, ##args);                  \
} while (0)
#endif /* KERNEL_event_trace_printk */

#endif /* CONNECTIVITY_BUILD_IN_ADAPTER_H */

/******************************************************************************
 * GPIO dump information
 ******************************************************************************/
#ifndef CONFIG_MTK_GPIO
#define KERNEL_gpio_dump_regs_range connectivity_export_dump_gpio_info
extern void gpio_dump_regs_range(int start, int end);
void connectivity_export_dump_gpio_info(int start, int end);
#endif

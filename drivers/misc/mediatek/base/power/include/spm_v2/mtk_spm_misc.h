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

#ifndef _MT_SPM_MISC_H
#define _MT_SPM_MISC_H

#include <linux/irqchip/mtk-gic.h>

/* AEE */
#ifdef CONFIG_MTK_RAM_CONSOLE
#define SPM_AEE_RR_REC 1
#else
#define SPM_AEE_RR_REC 0
#endif

/* IRQ */
extern int mt_irq_mask_all(struct mtk_irq_mask *mask);
extern int mt_irq_mask_restore(struct mtk_irq_mask *mask);
#if !defined(CONFIG_MACH_MT6757) && !defined(CONFIG_MACH_KIBOPLUS)
/* Dilapidate, needs to remove */
extern void mt_irq_unmask_for_sleep(unsigned int irq);
#endif

/* UART */
extern int request_uart_to_sleep(void);
extern int request_uart_to_wakeup(void);
extern void mtk_uart_restore(void);
extern void dump_uart_reg(void);

#if defined(CONFIG_MICROTRUST_TEE_SUPPORT)
extern int is_teei_ready(void);
#endif

/* SODI3 */
extern void soidle3_before_wfi(int cpu);
extern void soidle3_after_wfi(int cpu);

#if SPM_AEE_RR_REC
extern void aee_rr_rec_sodi3_val(u32 val);
extern u32 aee_rr_curr_sodi3_val(void);
#endif

#ifdef SPM_SODI_PROFILE_TIME
extern unsigned int soidle3_profile[4];
#endif

/* SODI */
extern void soidle_before_wfi(int cpu);
extern void soidle_after_wfi(int cpu);
#if SPM_AEE_RR_REC
extern void aee_rr_rec_sodi_val(u32 val);
extern u32 aee_rr_curr_sodi_val(void);
#endif

#ifdef SPM_SODI_PROFILE_TIME
extern unsigned int soidle_profile[4];
#endif

extern bool mtk_gpu_sodi_entry(void);
extern bool mtk_gpu_sodi_exit(void);
extern int hps_del_timer(void);
extern int hps_restart_timer(void);
extern int vcorefs_get_curr_ddr(void);

/* MCSODI */
#if SPM_AEE_RR_REC
extern void aee_rr_rec_mcsodi_val(u32 val);
extern u32 aee_rr_curr_mcsodi_val(void);
#endif

/* Deepidle */
#if SPM_AEE_RR_REC
extern void aee_rr_rec_deepidle_val(u32 val);
extern u32 aee_rr_curr_deepidle_val(void);
#endif

/* Suspend */
#if SPM_AEE_RR_REC
extern void aee_rr_rec_spm_suspend_val(u32 val);
extern u32 aee_rr_curr_spm_suspend_val(void);
#endif

/* MCDI */
extern void mcidle_before_wfi(int cpu);
extern void mcidle_after_wfi(int cpu);

#if SPM_AEE_RR_REC
extern unsigned int *aee_rr_rec_mcdi_wfi(void);
#endif

/* snapshot golden setting */
extern int snapshot_golden_setting(const char *func, const unsigned int line);
extern bool is_already_snap_shot;

/* power golden setting */
extern void mt_power_gs_dump_suspend(void);
extern void mt_power_gs_dump_dpidle(void);
extern void mt_power_gs_dump_sodi3(void);
extern bool slp_dump_golden_setting;
extern bool slp_chk_golden;
int __attribute__((weak)) snapshot_golden_setting(const char *func, const unsigned int line)
{
	return 0;
}

/* pasr */
extern void mtkpasr_phaseone_ops(void);
extern int configure_mrw_pasr(u32 segment_rank0, u32 segment_rank1);
extern int pasr_enter(u32 *sr, u32 *dpd);
extern int pasr_exit(void);
extern unsigned long mtkpasr_enable_sr;

/* eint */
extern void mt_eint_print_status(void);

#endif  /* _MT_SPM_MISC_H */

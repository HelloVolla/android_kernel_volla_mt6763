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

#ifndef __MT_FREQHOPPING_H__
#define __MT_FREQHOPPING_H__

/* Disable all FHCTL Common Interface for chip Bring-up */
#undef DISABLE_FREQ_HOPPING
/* Use HW semaphore to share REG_FHCTL_HP_EN with secure CPU DVFS */
#undef HP_EN_REG_SEMAPHORE_PROTECT

#define FHTAG "[FH]"
#define VERBOSE_DEBUG 0
#define DEBUG_MSG_ENABLE 0

#if VERBOSE_DEBUG
#define FH_MSG(fmt, args...)                                                   \
	pr_debug(FHTAG "" fmt " <- %s(): L<%d>  PID<%s><%d>\n", ##args,        \
		 __func__, __LINE__, current->comm, current->pid)

#define FH_MSG_DEBUG FH_MSG
#else

#define FH_MSG(fmt, args...) pr_debug(fmt, ##args)

#define FH_MSG_DEBUG(fmt, args...)                                             \
	do {                                                                   \
		if (DEBUG_MSG_ENABLE)                                          \
			pr_debug(FHTAG "" fmt "\n", ##args);                   \
	} while (0)
#endif

enum FH_FH_STATUS {
	FH_FH_DISABLE = 0,
	FH_FH_ENABLE_SSC,
};

enum FH_FH_PLL_SSC_DEF_STATUS {
	FH_SSC_DEF_DISABLE = 0,
	FH_SSC_DEF_ENABLE_SSC,
};

enum FH_PLL_STATUS { FH_PLL_DISABLE = 0, FH_PLL_ENABLE = 1 };

enum FH_CMD {
	FH_CMD_ENABLE = 1,
	FH_CMD_DISABLE,
	FH_CMD_ENABLE_USR_DEFINED,
	FH_CMD_DISABLE_USR_DEFINED,
	FH_CMD_INTERNAL_MAX_CMD
};

enum FH_PLL_STRUCT_FIELD {
	CURR_FREQ = 0,
	FH_STATUS,
	PLL_STATUS,
	SETTING_ID,
	SETTING_IDX_PATTERN,
	USER_DEFINED
};

enum FH_PLL_ID {
	FH_PLL0 = 0,   /* ARMPLL1 */
	FH_PLL1 = 1,   /* ARMPLL2 */
	FH_PLL2 = 2,   /* ARMPLL3 */
	FH_PLL3 = 3,   /* CCIPLL */
	FH_PLL4 = 4,   /* GPUPLL */
	FH_PLL5 = 5,   /* MPLL */
	FH_PLL6 = 6,   /* MEMPLL */
	FH_PLL7 = 7,   /* MAINPLL */
	FH_PLL8 = 8,   /* MSDCLL */
	FH_PLL9 = 9,   /* MMPLL */
	FH_PLL10 = 10, /* VDECPLL */
	FH_PLL11 = 11  /* TVDPLL */
};

#define FH_PLL_NUM 12

#define FH_GPU_PLLID FH_PLL4
#define FH_MEM_PLLID FH_PLL6
#define FH_MAIN_PLLID FH_PLL7
#define FH_MM_PLLID FH_PLL9
#define FH_VENC_PLLID FH_PLL10
#define isFHCTL(id) ((id >= FH_PLL0) ? true : false)

/* keep track the status of each PLL */
/* TODO: do we need another "uint mode" for Dynamic FH */
struct fh_pll_t {
	unsigned int curr_freq; /* Useless variable, just a legacy */
	unsigned int fh_status;
	unsigned int pll_status;
	unsigned int setting_id;
	unsigned int setting_idx_pattern;
	unsigned int user_defined;
};

struct freqhopping_ssc {
	unsigned int idx_pattern;
	unsigned int dt;
	unsigned int df;
	unsigned int upbnd;
	unsigned int lowbnd;
	unsigned int dds;
};

struct freqhopping_ioctl {
	unsigned int pll_id;
	struct freqhopping_ssc ssc_setting; /* used only when user-define */
	int result;
};

struct fhctl_ipi_data {
	unsigned int cmd;
	union {
		struct freqhopping_ioctl fh_ctl;
		unsigned int args[8];
	} u;
};

#if 0
/* FHCTL Legacy Interface. These API are useless in mt6797 */
int mt_fh_dram_overclock(int clk);
int mt_fh_get_dramc(void);
int mt_is_support_DFS_mode(void);
int mt_h2l_mempll(void);
int mt_l2h_mempll(void);
int mt_h2l_dvfs_mempll(void);
int mt_l2h_dvfs_mempll(void);
#endif

/* FHCTL HAL driver Interface. */
int mt_pause_armpll(unsigned int pll, unsigned int pause);

/* FHCTL Common driver Interface. */
int mt_dfs_mmpll(unsigned int dds);
int mt_dfs_armpll(unsigned int current_freq, unsigned int dds);
int mt_dfs_general_pll(unsigned int pll_id, unsigned int target_dds);
int freqhopping_config(unsigned int pll_id, unsigned long def_set_idx,
		       unsigned int enable);
int mt_freqhopping_devctl(unsigned int cmd, void *args);
void mt_freqhopping_pll_init(void);
/* void mt_freqhopping_init(void); */
void mt_fh_popod_save(void);
void mt_fh_popod_restore(void);
void mt_fh_unlock(void);
void mt_fh_lock(void);

struct mt_fh_hal_driver *mt_get_fh_hal_drv(void);

#endif /* !__MT_FREQHOPPING_H__ */

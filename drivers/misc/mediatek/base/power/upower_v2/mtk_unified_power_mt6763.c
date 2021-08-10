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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/rcupdate.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/ktime.h>
#include <linux/io.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <mt-plat/mtk_chip.h>

/* local include */
#include "mtk_upower.h"
#include "mtk_unified_power_data.h"
#include "mtk_devinfo.h"
#include "mtk_eem.h"
#include "upmu_common.h"

#ifndef EARLY_PORTING_SPOWER
#include "mtk_common_static_power.h"
#endif

#undef  BIT
#define BIT(bit)	(1U << (bit))

#define MSB(range)	(1 ? range)
#define LSB(range)	(0 ? range)
/**
 * Genearte a mask wher MSB to LSB are all 0b1
 * @r:	Range in the form of MSB:LSB
 */
#define BITMASK(r)	\
	(((unsigned int) -1 >> (31 - MSB(r))) & ~((1U << LSB(r)) - 1))

/**
 * Set value at MSB:LSB. For example, BITS(7:3, 0x5A)
 * will return a value where bit 3 to bit 7 is 0x5A
 * @r:	Range in the form of MSB:LSB
 */
/* BITS(MSB:LSB, value) => Set value at MSB:LSB  */
#define BITS(r, val)	((val << LSB(r)) & BITMASK(r))

#define GET_BITS_VAL(_bits_, _val_)  \
		(((_val_) & (BITMASK(_bits_))) >> ((0) ? _bits_))
/* #if (NR_UPOWER_TBL_LIST <= 1) */
struct upower_tbl final_upower_tbl[NR_UPOWER_BANK] = {};
/* #endif */

int degree_set[NR_UPOWER_DEGREE] = {
		UPOWER_DEGREE_0,
		UPOWER_DEGREE_1,
		UPOWER_DEGREE_2,
		UPOWER_DEGREE_3,
		UPOWER_DEGREE_4,
		UPOWER_DEGREE_5,
};

/* collect all the raw tables */
#define INIT_UPOWER_TBL_INFOS(name, tbl) {__stringify(name), &tbl}
struct upower_tbl_info
	upower_tbl_infos_list[NR_UPOWER_TBL_LIST][NR_UPOWER_BANK] = {
	/* MT6763 */
	[0] = {
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_LL, upower_tbl_ll_1_FY),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_L, upower_tbl_l_1_FY),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_LL,
			upower_tbl_cluster_ll_1_FY),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_L,
			upower_tbl_cluster_l_1_FY),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CCI, upower_tbl_cci_1_FY),
#ifdef UPOWER_L_PLUS
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_L_PLUS,
			upower_tbl_l_plus_1_FY),
#endif
	},
	/* MT6763T_FY */
	[1] = {
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_LL, upower_tbl_ll_2_FY),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_L, upower_tbl_l_2_FY),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_LL,
			upower_tbl_cluster_ll_2_FY),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_L,
			upower_tbl_cluster_l_2_FY),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CCI, upower_tbl_cci_2_FY),
#ifdef UPOWER_L_PLUS
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_L_PLUS,
			upower_tbl_l_plus_2_FY),
#endif
	},
	/* MT6763T_SB */
	[2] = {
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_LL, upower_tbl_ll_2_SB),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_L, upower_tbl_l_2_SB),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_LL,
			upower_tbl_cluster_ll_2_SB),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_L,
			upower_tbl_cluster_l_2_SB),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CCI, upower_tbl_cci_2_SB),
#ifdef UPOWER_L_PLUS
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_L_PLUS,
			upower_tbl_l_plus_2_SB),
#endif
	},
	/* MT6763TT */
	[3] = {
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_LL, upower_tbl_ll_3_FY),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_L, upower_tbl_l_3_FY),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_LL,
			upower_tbl_cluster_ll_3_FY),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_L,
			upower_tbl_cluster_l_3_FY),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CCI, upower_tbl_cci_3_FY),
#ifdef UPOWER_L_PLUS
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_L_PLUS,
			upower_tbl_l_plus_3_FY),
#endif
	},
	[4] = {
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_LL, upower_tbl_ll_3_SB),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_L, upower_tbl_l_3_SB),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_LL,
			upower_tbl_cluster_ll_3_SB),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_L,
			upower_tbl_cluster_l_3_SB),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CCI, upower_tbl_cci_3_SB),
#ifdef UPOWER_L_PLUS
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_L_PLUS,
			upower_tbl_l_plus_3_SB),
#endif
	},
};
/* Upower will know how to apply voltage that comes from EEM */
unsigned char upower_recognize_by_eem[NR_UPOWER_BANK] = {
	UPOWER_BANK_LL, /* LL EEM apply voltage to LL upower bank */
	UPOWER_BANK_L, /* L EEM apply voltage to L upower bank */
	UPOWER_BANK_LL, /* LL EEM apply voltage to CLS_LL upower bank */
	UPOWER_BANK_L, /* L EEM apply voltage to CLS_L upower bank */
	UPOWER_BANK_CCI, /* CCI EEM apply voltage to CCI upower bank */
#ifdef UPOWER_L_PLUS
	UPOWER_BANK_L, /* L EEM apply voltage to L_PLUS upower bank */
#endif
};

/* Used for rcu lock, points to all the raw tables list*/
struct upower_tbl_info *p_upower_tbl_infos = &upower_tbl_infos_list[0][0];

#ifndef EARLY_PORTING_SPOWER
int upower_bank_to_spower_bank(int upower_bank)
{
	int ret;

	switch (upower_bank) {
	case UPOWER_BANK_LL:
		ret = MTK_SPOWER_CPULL;
		break;
	case UPOWER_BANK_L:
		ret = MTK_SPOWER_CPUL;
		break;
	case UPOWER_BANK_CLS_LL:
		ret = MTK_SPOWER_CPULL_CLUSTER;
		break;
	case UPOWER_BANK_CLS_L:
		ret = MTK_SPOWER_CPUL_CLUSTER;
		break;
	case UPOWER_BANK_CCI:
		ret = MTK_SPOWER_CCI;
		break;
#ifdef UPOWER_L_PLUS
	case UPOWER_BANK_L_PLUS:
		ret = MTK_SPOWER_CPUL;
		break;
#endif
	default:
		ret = -1;
		break;
	}
	return ret;
}
#endif

static void upower_scale_l_cap(void)
{
	unsigned int ratio;
#ifdef CONFIG_ARM64
	unsigned long long temp;
#else
	unsigned int temp;
#endif
	unsigned int max_cap = 1024;
	int i, j;
	struct upower_tbl *tbl;

	/* get L opp0's cap and calculate scaling ratio */
	/* ratio = round_up(1024 * 1000 / opp0 cap) */
	/* new cap = orig cap * ratio / 1000 */
	tbl = upower_tbl_infos[UPOWER_BANK_L].p_upower_tbl;
	temp = tbl->row[UPOWER_OPP_NUM - 1].cap;
	ratio = ((max_cap * 1000) + (temp - 1)) / temp;
	upower_error("scale ratio = %d, orig cap = %d\n", ratio, temp);

	/* if L opp0's cap is 1024, no need to scale cap anymore */
	if (temp == 1024)
		return;

	/* scaling L and cluster L cap value */
	for (i = 0; i < NR_UPOWER_BANK; i++) {
		if ((i == UPOWER_BANK_L) || (i == UPOWER_BANK_CLS_L)) {
			tbl = upower_tbl_infos[i].p_upower_tbl;
			for (j = 0; j < UPOWER_OPP_NUM; j++) {
				temp = tbl->row[j].cap;
				tbl->row[j].cap = temp * ratio / 1000;
			}
		}
	}

	/* check opp0's cap after scaling */
	tbl = upower_tbl_infos[UPOWER_BANK_L].p_upower_tbl;
	temp = tbl->row[UPOWER_OPP_NUM - 1].cap;
	if (temp != 1024) {
		upower_error("Notice: new cap is not 1024 after scaling (%d)\n",
			ratio);
		tbl->row[UPOWER_OPP_NUM - 1].cap = 1024;
	}
}

/****************************************************
 * According to chip version get the raw upower tbl *
 * and let upower_tbl_infos points to it.           *
 * Choose a non used upower tbl location and let    *
 * upower_tbl_ref points to it to store target      *
 * power tbl.                                       *
 ***************************************************/

#define SEG_EFUSE 30
#define BIN_EFUSE 52 /* 588 */
void get_original_table(void)
{
	unsigned int bin = 0;
	unsigned short idx = 1; /* default use MT6763T_FY */
	unsigned int i, j;

	/* 0x588 bit[2:0] */
	bin = get_devinfo_with_index(BIN_EFUSE);
	bin = GET_BITS_VAL(2:0, bin);
	if (get_devinfo_with_index(SEG_EFUSE) == 0x10)
		idx = 0; /* MT6763 */
	else if (get_devinfo_with_index(SEG_EFUSE) == 0x20 ||
		get_devinfo_with_index(SEG_EFUSE) == 0) {
		if (bin == 1)
			idx = 2; /* MT6763T_SB */
		else
			idx = 1; /* MT6763T_FY */
	} else if (get_devinfo_with_index(SEG_EFUSE) == 0x30) {
		if (is_ext_buck_exist()) {
			if (bin == 1)
				idx = 4; /* MT6763TT_SB */
			else
				idx = 3; /* MT6763TT_FY */
		} else {
			if (bin == 1)
				idx = 2; /* MT6763T_SB */
			else
				idx = 1; /* MT6763T_FY */
		}
	}

	upower_error("bin=%d, idx=%d, is6311=%d, seg=%d\n", bin, idx,
		is_ext_buck_exist(), get_devinfo_with_index(SEG_EFUSE));

	/* get location of reference table */
	upower_tbl_infos = &upower_tbl_infos_list[idx][0];

	/* get location of target table */
/* #if (NR_UPOWER_TBL_LIST <= 1) */
	upower_tbl_ref = &final_upower_tbl[0];
/* #else */
/*	upower_tbl_ref = upower_tbl_infos_list[(idx+1) %
 *		NR_UPOWER_TBL_LIST][0].p_upower_tbl;
 */
/* #endif */

	upower_debug("idx %d dest:%p, src:%p\n",
		(idx+1)%NR_UPOWER_TBL_LIST, upower_tbl_ref, upower_tbl_infos);

	/* If disable upower, ptr will point to original upower table */
	p_upower_tbl_infos = upower_tbl_infos;

#if 0
	upower_debug("upower_tbl_ll_1_FY %p\n", &upower_tbl_ll_1_FY);
	upower_debug("upower_tbl_ll_1_SB %p\n", &upower_tbl_ll_1_SB);
	upower_debug("upower_tbl_ll_2_FY %p\n", &upower_tbl_ll_2_FY);
#endif

/*
 *  Clear volt fields before eem run.
 *  If eem is enabled, it will apply volt into it. If eem is disabled,
 *  the values of volt are 0 , and upower will apply orig volt into it
 */
	for (i = 0; i < NR_UPOWER_BANK; i++) {
		for (j = 0; j < UPOWER_OPP_NUM; j++)
			upower_tbl_ref[i].row[j].volt = 0;
	}
	for (i = 0; i < NR_UPOWER_BANK; i++)
		upower_debug("bank[%d] dest:%p dyn_pwr:%u, volt[0]%u\n",
			i, &upower_tbl_ref[i],
			upower_tbl_ref[i].row[0].dyn_pwr,
			upower_tbl_ref[i].row[0].volt);

	/* Not support L+ now, scale L and cluster L cap to 1024 */
	upower_scale_l_cap();
}

#ifdef UPOWER_L_PLUS
/* Copy L capacity to L plus  */
void upower_update_L_plus_cap(void)
{
	struct upower_tbl *tbl;
	int j;

	tbl = upower_tbl_infos[UPOWER_BANK_L].p_upower_tbl;
	for (j = 0; j < UPOWER_OPP_NUM; j++)
		upower_tbl_ref[UPOWER_BANK_L_PLUS].row[j].cap = tbl->row[j].cap;
}

/* Copy L capacity to L plus
 *  1. Get L plus opp15, 85c leakage
 *  2. Get L opp15, 85C leakage
 *  3. Calculate the ratio between 1 and 2
 *  4. Use ratio to calculate opp0-15, 6 degrees leakages
 */
void upower_update_L_plus_lkg_pwr(void)
{
	struct upower_tbl *target_tbl;
	struct upower_tbl *ref_tbl;
	enum upower_bank target_bank = UPOWER_BANK_L_PLUS;
	enum upower_bank ref_bank = UPOWER_BANK_L;
	unsigned int target_lkg, ref_lkg;
	unsigned int ratio;
	int j, i;

	/* MT6763 not support L+, so no need to calculate L+ leakage */
	/* In this case, L+ table is as same as L */
	if (get_devinfo_with_index(SEG_EFUSE) == 0x10)
		return;

	/* get L plus upower table */
	target_tbl = upower_tbl_infos[target_bank].p_upower_tbl;
	target_lkg = target_tbl->row[15].lkg_pwr[0];
	/* get L  upower table */
	ref_tbl = &upower_tbl_ref[ref_bank];
	ref_lkg = ref_tbl->row[15].lkg_pwr[0];
	/* calculate ratio */
	ratio = (target_lkg * 1000 + ref_lkg - 1) / ref_lkg;
	upower_error("target lkg: %d, ref_lkg: %d, ratio: %d\n",
					target_lkg, ref_lkg, ratio);

	for (j = 0; j < UPOWER_OPP_NUM; j++) {
		for (i = 0; i < NR_UPOWER_DEGREE; i++)
		upower_tbl_ref[target_bank].row[j].lkg_pwr[i] =
		upower_tbl_ref[ref_bank].row[j].lkg_pwr[i] * ratio / 1000;
	}
}
#endif
MODULE_DESCRIPTION("MediaTek Unified Power Driver v0.0");
MODULE_LICENSE("GPL");

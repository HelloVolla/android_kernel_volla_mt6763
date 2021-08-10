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

#ifndef _MT_PPM_IPI_
#define _MT_PPM_IPI_

#include "mach/mtk_ppm_api.h"
#include "mtk_ppm_platform.h"


#define PPM_D_LEN	(7) /* # of cmd + arg0 + arg1 + ... */

/* IPI Msg type */
enum {
	PPM_IPI_INIT,
	PPM_IPI_UPDATE_LIMIT,
	PPM_IPI_THERMAL_LIMIT_TEST,
	PPM_IPI_PTPOD_TEST,

	NR_PPM_IPI,
};

/* IPI Msg data structure */
struct ppm_ipi_data {
	/* 1 int for opcode */
	unsigned int cmd;
	/* 6 int for ipi data */
	union {
		struct {
			unsigned int efuse_val;
			unsigned int cobra_tbl_addr;
			unsigned int dvfs_tbl_type;
		} init;
		struct {
			struct {
				unsigned char min_cpufreq_idx;
				unsigned char max_cpufreq_idx;
				unsigned char max_cpu_core;
				unsigned char advise_cpufreq_idx;
			} cluster_limit[NR_PPM_CLUSTERS];
		} update_limit;
		struct {
			unsigned int budget;
		} thermal_limit_test;
		struct {
			unsigned int activate;
		} ptpod_test;
		/* padding to 6 int */
		struct {
			unsigned int ppm_data[6];
		};
	} u;
};

#ifdef PPM_SSPM_SUPPORT
extern void ppm_ipi_init(unsigned int efuse_val, unsigned int cobra_tbl_addr);
extern void ppm_ipi_update_limit(struct ppm_client_req req);
extern void ppm_ipi_thermal_limit_test(unsigned int budget);
extern void ppm_ipi_ptpod_test(unsigned int activate);
#endif

#endif


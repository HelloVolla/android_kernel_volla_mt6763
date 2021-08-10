/*
 * Copyright (C) 2017 MediaTek Inc.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef _MT_PMIC_INFO_H_
#define _MT_PMIC_INFO_H_

/*
 * The CHIP INFO
 */
#define PMIC6357_E1_CID_CODE    0x5710
#define PMIC6357_E2_CID_CODE    0x5720
#define PMIC6357_E3_CID_CODE    0x5730


#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
#define IPIMB
#endif

extern unsigned int pmic_ipi_test_code(void);

/*
 * Debugfs
 */
#define PMICTAG                "[PMIC] "
extern unsigned int gPMICDbgLvl;
extern unsigned int gPMICHKDbgLvl;
extern unsigned int gPMICCOMDbgLvl;
extern unsigned int gPMICIRQDbgLvl;
extern unsigned int gPMICREGDbgLvl;

#define PMIC_LOG_DBG     4
#define PMIC_LOG_INFO    3
#define PMIC_LOG_NOT     2
#define PMIC_LOG_WARN    1
#define PMIC_LOG_ERR     0

#define PMICLOG(fmt, arg...) do { \
	if (gPMICDbgLvl >= PMIC_LOG_DBG) \
		pr_notice(PMICTAG "%s: " fmt, __func__, ##arg); \
} while (0)

#define HKLOG(fmt, arg...) do { \
	if (gPMICHKDbgLvl >= PMIC_LOG_DBG) \
		pr_notice(PMICTAG "%s: " fmt, __func__, ##arg); \
} while (0)

#define COMLOG(fmt, arg...) do { \
	if (gPMICCOMDbgLvl >= PMIC_LOG_DBG) \
		pr_notice(PMICTAG "%s: " fmt, __func__, ##arg); \
} while (0)

#define IRQLOG(fmt, arg...) do { \
	if (gPMICIRQDbgLvl >= PMIC_LOG_DBG) \
		pr_notice(PMICTAG "%s: " fmt, __func__, ##arg); \
} while (0)

#define RGLTRLOG(fmt, arg...) do { \
	if (gPMICREGDbgLvl >= PMIC_LOG_DBG) \
		pr_notice(PMICTAG "%s: " fmt, __func__, ##arg); \
} while (0)

extern void wk_pmic_enable_sdn_delay(void);

#endif				/* _MT_PMIC_INFO_H_ */

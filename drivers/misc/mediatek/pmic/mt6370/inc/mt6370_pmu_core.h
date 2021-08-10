/*
 *  Copyright (C) 2017 MediaTek Inc.
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

#ifndef __LINUX_MFD_MT6370_PMU_CORE_H
#define __LINUX_MFD_MT6370_PMU_CORE_H

struct mt6370_pmu_core_platdata {
	uint8_t i2cstmr_rst_en:1;
	uint8_t i2cstmr_rst_tmr:2;
	uint8_t mrstb_en:1;
	uint8_t mrstb_tmr:3;
	uint8_t int_wdt:2;
	uint8_t int_deg:2;
};

#define MT6370_I2CRST_ENMASK	(0x80)
#define MT6370_I2CRST_ENSHFT	(7)
#define MT6370_I2CRST_TMRMASK	(0x60)
#define MT6370_I2CRST_TMRSHFT	(5)
#define MT6370_MRSTB_ENMASK	(0x10)
#define MT6370_MRSTB_ENSHFT	(4)
#define MT6370_MRSTB_TMRMASK	(0x0E)
#define MT6370_MRSTB_TMRSHFT	(1)

#define MT6370_INTWDT_TMRMASK	(0xC0)
#define MT6370_INTWDT_TMRSHFT	(6)
#define MT6370_INTDEG_TIMEMASK	(0x30)
#define MT6370_INTDEG_TIMESHFT	(4)

#endif /* #ifndef __LINUX_MFD_MT6370_PMU_CORE_H */

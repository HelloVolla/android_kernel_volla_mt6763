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
#ifndef __TMP_BTS_H__
#define __TMP_BTS_H__

#define AUX_IN0_NTC (0)
#define AUX_IN1_NTC (1)

#define BTS_RAP_PULL_UP_R 390000 /* 390K,pull up resister */
#define BTS_TAP_OVER_CRITICAL_LOW                                              \
	4397119 /* base on 100K NTC temp default value -40 deg */
#define BTS_RAP_PULL_UP_VOLTAGE 1800    /* 1.8V ,pull up voltage */
#define BTS_RAP_NTC_TABLE 7		/* default is NCP15WF104F03RC(100K) */
#define BTS_RAP_ADC_CHANNEL AUX_IN0_NTC /* default is 0 */

#define BTSMDPA_RAP_PULL_UP_R 390000 /* 390K,pull up resister */
#define BTSMDPA_TAP_OVER_CRITICAL_LOW                                          \
	4397119 /* base on 100K NTC temp default value -40 deg */
#define BTSMDPA_RAP_PULL_UP_VOLTAGE 1800 /* 1.8V ,pull up voltage */
#define BTSMDPA_RAP_NTC_TABLE 7		 /* default is NCP15WF104F03RC(100K) */
#define BTSMDPA_RAP_ADC_CHANNEL AUX_IN1_NTC /* default is 1 */

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int *rawdata);
extern int IMM_IsAdcInitReady(void);

#endif /* __TMP_BTS_H__ */

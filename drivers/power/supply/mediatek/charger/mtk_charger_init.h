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

#ifndef __MTK_CHARGER_INIT_H__
#define __MTK_CHARGER_INIT_H__

#define BATTERY_CV 4350000
#define V_CHARGER_MAX 6500000 /* 6.5 V */
#define V_CHARGER_MIN 4600000 /* 4.6 V */

#define USB_CHARGER_CURRENT_SUSPEND		0 /* def CONFIG_USB_IF */
#define USB_CHARGER_CURRENT_UNCONFIGURED	70000 /* 70mA */
#define USB_CHARGER_CURRENT_CONFIGURED		500000 /* 500mA */
#define USB_CHARGER_CURRENT			500000 /* 500mA */
#define AC_CHARGER_CURRENT			2050000
#define AC_CHARGER_INPUT_CURRENT		3200000
#define NON_STD_AC_CHARGER_CURRENT		500000
#define CHARGING_HOST_CHARGER_CURRENT		650000
#define APPLE_1_0A_CHARGER_CURRENT		650000
#define APPLE_2_1A_CHARGER_CURRENT		800000
#define TA_AC_CHARGING_CURRENT	3000000

/* dynamic mivr */
#define V_CHARGER_MIN_1 4400000 /* 4.4 V */
#define V_CHARGER_MIN_2 4200000 /* 4.2 V */
#define MAX_DMIVR_CHARGER_CURRENT 1400000 /* 1.4 A */

/* sw jeita */
#define JEITA_TEMP_ABOVE_T4_CV	4240000
#define JEITA_TEMP_T3_TO_T4_CV	4240000
#define JEITA_TEMP_T2_TO_T3_CV	4340000
#define JEITA_TEMP_T1_TO_T2_CV	4240000
#define JEITA_TEMP_T0_TO_T1_CV	4040000
#define JEITA_TEMP_BELOW_T0_CV	4040000
#define TEMP_T4_THRES  50
#define TEMP_T4_THRES_MINUS_X_DEGREE 47
#define TEMP_T3_THRES  45
#define TEMP_T3_THRES_MINUS_X_DEGREE 39
#define TEMP_T2_THRES  10
#define TEMP_T2_THRES_PLUS_X_DEGREE 16
#define TEMP_T1_THRES  0
#define TEMP_T1_THRES_PLUS_X_DEGREE 6
#define TEMP_T0_THRES  0
#define TEMP_T0_THRES_PLUS_X_DEGREE  0
#define TEMP_NEG_10_THRES 0

/* Battery Temperature Protection */
#define MIN_CHARGE_TEMP  0
#define MIN_CHARGE_TEMP_PLUS_X_DEGREE	6
#define MAX_CHARGE_TEMP  50
#define MAX_CHARGE_TEMP_MINUS_X_DEGREE	47

/* pe */
#define PE_ICHG_LEAVE_THRESHOLD 1000000 /* uA */
#define TA_AC_12V_INPUT_CURRENT 3200000
#define TA_AC_9V_INPUT_CURRENT	3200000
#define TA_AC_7V_INPUT_CURRENT	3200000
#define TA_9V_SUPPORT
#define TA_12V_SUPPORT

/* pe2.0 */
#define PE20_ICHG_LEAVE_THRESHOLD 1000000 /* uA */
#define TA_START_BATTERY_SOC	0
#define TA_STOP_BATTERY_SOC	85

/* dual charger */
#define TA_AC_MASTER_CHARGING_CURRENT 1500000
#define TA_AC_SLAVE_CHARGING_CURRENT 1500000
#define SLAVE_MIVR_DIFF 100000

/* slave charger */
#define CHG2_EFF 90

/* cable measurement impedance */
#define CABLE_IMP_THRESHOLD 699
#define VBAT_CABLE_IMP_THRESHOLD 3900000 /* uV */

/* bif */
#define BIF_THRESHOLD1 4250000	/* UV */
#define BIF_THRESHOLD2 4300000	/* UV */
#define BIF_CV_UNDER_THRESHOLD2 4450000	/* UV */
#define BIF_CV BATTERY_CV /* UV */

#define R_SENSE 56 /* mohm */

#define MAX_CHARGING_TIME (12 * 60 * 60) /* 12 hours */

#define DEFAULT_BC12_CHARGER 0 /* MAIN_CHARGER */

/* battery warning */
#define BATTERY_NOTIFY_CASE_0001_VCHARGER
#define BATTERY_NOTIFY_CASE_0002_VBATTEMP

/* pe4 */
#define PE40_MAX_VBUS 11000
#define PE40_MAX_IBUS 3000
#define HIGH_TEMP_TO_LEAVE_PE40 46
#define HIGH_TEMP_TO_ENTER_PE40 39
#define LOW_TEMP_TO_LEAVE_PE40 10
#define LOW_TEMP_TO_ENTER_PE40 16

/* pd */
#define PD_VBUS_UPPER_BOUND 10000000	/* uv */
#define PD_VBUS_LOW_BOUND 5000000	/* uv */
#define PD_ICHG_LEAVE_THRESHOLD 1000000 /* uA */
#define PD_STOP_BATTERY_SOC 80

#define VSYS_WATT 5000000
#define IBUS_ERR 14

#endif /*__MTK_CHARGER_INIT_H__*/

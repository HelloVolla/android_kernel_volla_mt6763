/* 
 * ICM426XX sensor driver
 * Copyright (C) 2018 Invensense, Inc.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef ICM426XX_ACCEL_H
#define ICM426XX_ACCEL_H

#define ICM426XX_USE_INTERRUPT        0     // Note if enable pedometer function , please enable this Macro defination 
//#define ICM426XX_ACCEL_LOW_POWER_MODE
#define ICM_ACCEL_AXES_NUM 3

/* 1 G = 9.80665f */
/* +/-2G = 4G(-2G ~ 2G) 0 ~ 65535 1G = 16384 0.06mg = 1 */
/* +/-4G = 8G(-4G ~ 4G) 0 ~ 65535 1G = 8192 0.12mg = 1 */
/* +/-8G = 8G(-8G ~ 8G) 0 ~ 65535 1G = 4096 0.24mg = 1 */
/* +/-16G = 8G(-16G ~ 16G) 0 ~ 65535 1G = 2046 0.49mg = 1 */

/* Min: ICM426XX_ACC_RANGE_16G */
#define ICM426XX_ACCEL_MIN_SENSITIVITY        2048
/* Max: ICM426XX_ACC_RANGE_2G */
#define ICM426XX_ACCEL_MAX_SENSITIVITY        16384
/* Default: ICM426XX_ACC_RANGE_4G */
#define ICM426XX_ACCEL_DEFAULT_SENSITIVITY    8192

/* GRAVITY_EARTH_1000 is defined as 9807 */
/* about (9.80665f)*1000, accel div 1000 */
#define GRAVITY_EARTH_1000 9807

/*
    Full scale select for accelerometer UI interface output 
    b'000: ¡¾16g (default) 
    b'001: ¡¾8g 
    b'010: ¡¾4g 
    b'011: ¡¾2g 
*/
#define ICM426XX_ACCEL_RANGE_2G 0x03
#define ICM426XX_ACCEL_RANGE_4G 0x02
#define ICM426XX_ACCEL_RANGE_8G 0x01
#define ICM426XX_ACCEL_RANGE_16G 0x00

#define SELF_TEST_ACC_PRECISION 1000
#define SELF_TEST_ACC_SHIFT_DELTA 500
#endif /* ICM426XX_ACCEL_H */


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

#ifndef _KD_CAMERA_FEATURE_H_
#define _KD_CAMERA_FEATURE_H_


#include "kd_camera_feature_id.h"
#include "kd_camera_feature_enum.h"

enum IMGSENSOR_SENSOR_IDX {
	IMGSENSOR_SENSOR_IDX_MIN_NUM = 0,
	IMGSENSOR_SENSOR_IDX_MAIN    = IMGSENSOR_SENSOR_IDX_MIN_NUM,
	IMGSENSOR_SENSOR_IDX_SUB,
	IMGSENSOR_SENSOR_IDX_MAIN2,
	IMGSENSOR_SENSOR_IDX_SUB2,
	IMGSENSOR_SENSOR_IDX_MAIN3,
	IMGSENSOR_SENSOR_IDX_MAX_NUM,
	IMGSENSOR_SENSOR_IDX_NONE,
};

enum CAMERA_DUAL_CAMERA_SENSOR_ENUM {
	DUAL_CAMERA_NONE_SENSOR        = 0,
	DUAL_CAMERA_MAIN_SENSOR        = 1,
	DUAL_CAMERA_SUB_SENSOR         = 2,
	DUAL_CAMERA_MAIN_2_SENSOR      = 4,
	DUAL_CAMERA_SUB_2_SENSOR       = 8,
	DUAL_CAMERA_MAIN_3_SENSOR      = 16,
	DUAL_CAMERA_SENSOR_MAX,
	/* for backward compatible */
	DUAL_CAMERA_MAIN_SECOND_SENSOR = DUAL_CAMERA_MAIN_2_SENSOR,

};

#define IMGSENSOR_SENSOR_DUAL2IDX(idx) ((ffs(idx) - 1))
#define IMGSENSOR_SENSOR_IDX2DUAL(idx) (1<<(idx))

#define IMGSENSOR_SENSOR_IDX_MAP(idx) \
	(((idx) > DUAL_CAMERA_NONE_SENSOR && (idx) < DUAL_CAMERA_SENSOR_MAX) \
	? (enum IMGSENSOR_SENSOR_IDX)IMGSENSOR_SENSOR_DUAL2IDX(idx) \
	: IMGSENSOR_SENSOR_IDX_NONE)

#endif              /* _KD_IMGSENSOR_DATA_H */


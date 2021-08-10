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

#ifndef _CCU_IMGSENSOR_IF_H_
#define _CCU_IMGSENSOR_IF_H_

struct ccu_sensor_info {
	int32_t slave_addr;
	int32_t i2c_id;
	char *sensor_name_string;
};


void ccu_set_current_fps(int32_t sensorType, int32_t current_fps);
void ccu_set_sensor_info(int32_t sensorType, struct ccu_sensor_info *info);

#endif

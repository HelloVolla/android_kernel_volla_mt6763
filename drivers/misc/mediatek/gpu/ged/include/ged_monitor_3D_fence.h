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

#ifndef __GED_MONITOR_3D_FENCE_H__
#define __GED_MONITOR_3D_FENCE_H__

#include "ged_type.h"

GED_ERROR ged_monitor_3D_fence_add(int fence_fd);
void ged_monitor_3D_fence_notify(void);
unsigned long ged_monitor_3D_fence_done_time(void);
int ged_monitor_3D_fence_get_count(void);

#endif

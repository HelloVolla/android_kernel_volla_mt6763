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

#ifndef ICM426XX_SHARE_INTERFACE_H
#define ICM426XX_SHARE_INTERFACE_H

#define ICM426XX_SUCCESS               0
#define ICM426XX_ERR_BUS              -1
#define ICM426XX_ERR_INVALID_PARAM    -2
#define ICM426XX_ERR_STATUS           -3
#define ICM426XX_ERR_SETUP_FAILURE    -4

extern int icm426xx_share_read_register(u8 addr, u8 *data, u8 len);
extern int icm426xx_share_write_register(u8 addr, u8 *data, u8 len);

#endif /* ICM426XX_SHARE_INTERFACE_H */

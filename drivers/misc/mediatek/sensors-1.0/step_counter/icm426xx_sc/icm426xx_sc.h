/* 
 * ICM206XX sensor driver
 * Copyright (C) 2016 Invensense, Inc.
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

#ifndef ICM426XX_SC_H
#define ICM426XX_SC_H

/*=======================================================================================*/
/* DMP Load Section					 			 	 */
/*=======================================================================================*/

/* To verify DMP firmware download, activate this feature	*/
//#define ICM426XX_DMP_DOWNLOAD_VERIFY

/* To tune DMP Step Counter, activate this feature 		*/
/* This feature is only for threshold tuning and debugging.	*/ 
/* For permanent change, dmpMemory should be changed 		*/
//#define ICM426XX_DMP_STEP_COUNTER_TUNE

/*----------------------------------------------------------------------------*/
#define ICM426XX_WOM_THRESHOLD_TUNE

/* 50Hz */
#define DEFAULT_SAMPLING_PERIOD_NS	(NSEC_PER_SEC / 50)

/* Wake-on-Motion/No-Motion */
#define ICM426XX_WOM_COUNT 200 // 200 samples
#define ICM426XX_WOM_COMPUTE(val_mg) ((256 * val_mg) / 1000)
#define ICM426XX_WOM_DEFAULT_THRESHOLD 25

#define SIG_ICM426XX    44
#define ICM426XX_IOCTL_GROUP    0x10
#define ICM426XX_WRITE_DAEMON_PID    _IO(ICM406XX_IOCTL_GROUP, 1)
#define ICM426XX_READ_SENSOR_DATA    _IO(ICM406XX_IOCTL_GROUP, 2)
#define ICM426XX_WRITE_SENSOR_DATA    _IO(ICM406XX_IOCTL_GROUP, 3)
#define ICM426XX_LOG    _IO(ICM406XX_IOCTL_GROUP, 15)

#define BIT_APEX_CONFIG0_DMP_POWER_SAVE_POS       7
#define BIT_APEX_CONFIG0_DMP_POWER_SAVE_MASK   (0x1 << BIT_APEX_CONFIG0_DMP_POWER_SAVE_POS)

#define	ICM426XX_APEX_CONFIG0_DMP_POWER_SAVE_EN   0x1 << BIT_APEX_CONFIG0_DMP_POWER_SAVE_POS
#define	ICM426XX_APEX_CONFIG0_DMP_POWER_SAVE_DIS  0x0 << BIT_APEX_CONFIG0_DMP_POWER_SAVE_POS

#define REQUEST_SIGNAL_PROCESS_DATA    0x01
void icm426xx_sc_NotifySensorData(void);
#endif /* ICM426XX_SC_H */


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

#ifndef _INV_REG_40604_H_
#define _INV_REG_40604_H_

/*register and associated bit definition*/
/* Bank 0 */
#define REG_CHIP_CONFIG_REG         0x11
#define REG_DRIVE_CONFIG			0x13
#define REG_INT_CONFIG_REG          0x14
#define REG_FIFO_CONFIG_REG         0x16
#define REG_TEMP_DATA0_UI           0x1D
#define REG_TEMP_DATA1_UI           0x1E
#define REG_ACCEL_DATA_X0_UI        0x1F
#define REG_ACCEL_DATA_X1_UI        0x20
#define REG_ACCEL_DATA_Y0_UI        0x21
#define REG_ACCEL_DATA_Y1_UI        0x22
#define REG_ACCEL_DATA_Z0_UI        0x23
#define REG_ACCEL_DATA_Z1_UI        0x24
#define REG_GYRO_DATA_X0_UI         0x25
#define REG_GYOR_DATA_X1_UI         0x26
#define REG_GYRO_DATA_Y0_UI         0x27
#define REG_GYRO_DATA_Y1_UI         0x28
#define REG_GYRO_DATA_Z0_UI         0x29
#define REG_GYRO_DATA_Z1_UI         0x2A
#define REG_TMST_FSYNC1             0x2B
#define REG_TMST_FSYNC2             0x2C
#define REG_INT_STATUS              0x2D
#define REG_FIFO_BYTE_COUNT1        0x2E
#define REG_FIFO_BYTE_COUNT2        0x2F
#define REG_FIFO_DATA_REG           0x30
#define REG_APEX_DATA0				0x31
#define REG_APEX_DATA1				0x32
#define REG_APEX_DATA2				0x33

#define REG_SIGNAL_PATH_RESET       0x4B
#define REG_INTF_CONFIG0            0x4C
#define REG_PWR_MGMT_0              0x4E
#define REG_GYRO_CONFIG0            0x4F
#define BIT_GYRO_FSR                0xE0
#define BIT_GYRO_ODR                0x0F
#define REG_ACCEL_CONFIG0           0x50
#define BIT_ACCEL_FSR               0xE0
#define BIT_ACCEL_ODR               0x0F
#define REG_GYRO_CONFIG1            0x51
#define REG_GYRO_ACCEL_CONFIG0      0x52
#define BIT_ACCEL_FILTER            0xF0
#define BIT_GYRO_FILTER             0x0F
#define REG_ACCEL_CONFIG1           0x53
#define REG_SMD_CONFIG              0x57
#define REG_INT_STATUS2             0x37
#define REG_INT_STATUS3             0x38
#define REG_TMST_CONFIG             0x54
#define REG_APEX_CONFIG0            0x56
#define REG_FIFO_CONFIG1            0x5F
#define REG_FIFO_CONFIG2            0x60
#define REG_FIFO_CONFIG3            0x61
#define REG_FSYNC_CONFIG            0x62
#define REG_INT_CONFIG0             0x63
#define REG_INT_CONFIG1             0x64
#define REG_INT_SOURCE0             0x65
#define REG_INT_SOURCE1             0x66
#define REG_INT_SOURCE2             0x67
#define REG_INT_SOURCE3             0x68
#define REG_INT_SOURCE4             0x69

#define REG_FIFO_LOST_PKT0          0x6C
#define REG_FIFO_LOST_PKT1          0x6D
#define REG_SELF_TEST_CONFIG        0x70
#define REG_WHO_AM_I                0x75
#define REG_REG_BANK_SEL            0x76
#define REG_GOS_USER0               0x77
#define REG_GOS_USER1               0x78
#define REG_GOS_USER2               0x79
#define REG_GOS_USER3               0x7A
#define REG_GOS_USER4               0x7B
#define REG_GOS_USER5               0x7C
#define REG_GOS_USER6               0x7D
#define REG_GOS_USER7               0x7E
#define REG_GOS_USER8               0x7F

/* Bank 1 */
#define REG_GYRO_CONFIG_STATIC2     0x0B
#define REG_XG_ST_DATA              0x5F
#define REG_YG_ST_DATA              0x60
#define REG_ZG_ST_DATA              0x61
#define REG_TMSTVAL0                0x62
#define REG_TMSTVAL1                0x63
#define REG_TMSTVAL2                0x64
#define REG_INTF_CONFIG4            0x7A
#define REG_INTF_CONFIG5            0x7B
#define REG_INTF_CONFIG6            0x7C

/* Bank 2 */
#define REG_XA_ST_DATA              0x3B
#define REG_YA_ST_DATA              0x3C
#define REG_ZA_ST_DATA              0x3D
#define REG_OIS1_CONFIG1            0x44
#define REG_OIS1_CONFIG2            0x45
#define REG_OIS1_CONFIG3            0x46
#define REG_TEMP_DATA0_OIS1         0x47
#define REG_TEMP_DATA1_OIS1         0x48
#define REG_ACCEL_DATA_X0_OIS1      0x49
#define REG_ACCEL_DATA_X1_OIS1      0x4A
#define REG_ACCEL_DATA_Y0_OIS1      0x4B
#define REG_ACCEL_DATA_Y1_OIS1      0x4C
#define REG_ACCEL_DATA_Z0_OIS1      0x4D
#define REG_ACCEL_DATA_Z1_OIS1      0x4E
#define REG_GYRO_DATA_X0_OIS1       0x4F
#define REG_GYRO_DATA_X1_OIS1       0x50
#define REG_GYRO_DATA_Y0_OIS1       0x51
#define REG_GYRO_DATA_Y1_OIS1       0x52
#define REG_GYRO_DATA_Z0_OIS1       0x53
#define REG_GYRO_DATA_Z1_OIS1       0x54
#define REG_TMSTVAL0_OIS1           0x55
#define REG_TMSTVAL1_OIS1           0x56
#define REG_INT_STATUS_OIS1         0x57

/* Bank 4 */
#define REG_APEX_CONFIG1            0X40
#define REG_APEX_CONFIG2            0X41
#define REG_APEX_CONFIG3            0X42
#define REG_APEX_CONFIG4            0X43
#define REG_APEX_CONFIG5            0X44
#define REG_APEX_CONFIG6            0X45
#define REG_APEX_CONFIG7            0X46
#define REG_APEX_CONFIG8            0X47
#define REG_APEX_CONFIG9            0X48
#define REG_ACCEL_WOM_X_THR 				0x4A
#define REG_ACCEL_WOM_Y_THR 				0x4B
#define REG_ACCEL_WOM_Z_THR 				0x4C
#define REG_INT_SOURCE6							0x4D

/* REG_WHO_AM_I */
#define WHO_AM_I_ICM426XX           0x42
#define WHO_AM_I_ICM40607			0x38
/* REG_REG_BANK_SEL */
#define BIT_BANK_SEL_0              0x00
#define BIT_BANK_SEL_1              0x01
#define BIT_BANK_SEL_2              0x02
#define BIT_BANK_SEL_3              0x03
#define BIT_BANK_SEL_4              0x04


/* REG_CHIP_CONFIG_REG */
#define BIT_SOFT_RESET              0x01

/* REG_GYRO_CONFIG0/REG_ACCEL_CONFIG0 */
#define SHIFT_GYRO_FS_SEL           5
#define SHIFT_ACCEL_FS_SEL          5
#define SHIFT_ODR_CONF              0

/* REG_INT_CONFIG_REG */
#define SHIFT_INT1_POLARITY         0
#define SHIFT_INT1_DRIVE_CIRCUIT    1
#define SHIFT_INT1_MODE             2

/* REG_PWR_MGMT_0 */
#define BIT_TEMP_DIS                0x20
#define BIT_IDLE                    0x10
#define BIT_GYRO_MODE_OFF           0x00
#define BIT_GYRO_MODE_STBY          0x04
#define BIT_GYRO_MODE_LPM           0x08
#define BIT_GYRO_MODE_LNM           0x0C
#define BIT_ACCEL_MODE_OFF          0x00
#define BIT_ACCEL_MODE_LPM          0x02
#define BIT_ACCEL_MODE_LNM          0x03

/* REG_SIGNAL_PATH_RESET */
#define BIT_FIFO_FLUSH              0x02

/* REG_INTF_CONFIG0 */
#define BIT_FIFO_COUNT_REC          0x40
#define BIT_COUNT_BIG_ENDIAN        0x20
#define BIT_SENS_DATA_BIG_ENDIAN    0x10
#define BIT_UI_SIFS_DISABLE_SPI     0x02
#define BIT_UI_SIFS_DISABLE_I2C     0x03

/* REG_FIFO_CONFIG1 */
#define BIT_FIFO_ACCEL_EN           0x01
#define BIT_FIFO_GYRO_EN            0x02
#define BIT_FIFO_TEMP_EN            0x04
#define BIT_FIFO_TMST_FSYNC_EN      0x08
#define BIT_FIFO_HIRES_EN           0x10
#define BIT_FIFO_WM_TH              0x20
#define BIT_FIFO_RESUME_PART_RD     0x40

/* REG_INT_CONFIG1 */
#define BIT_INT_ASY_RST_DISABLE     0x10

/* REG_INT_SOURCE0 */
#define BIT_INT_UI_AGC_RDY_INT1_EN  0x01
#define BIT_INT_FIFO_FULL_INT1_EN   0x02
#define BIT_INT_FIFO_THS_INT1_EN    0x04
#define BIT_INT_UI_DRDY_INT1_EN     0x08
#define BIT_INT_RESET_DONE_INT1_EN  0x10
#define BIT_INT_PLL_RDY_INT1_EN     0x20
#define BIT_INT_UI_FSYNC_INT1_EN    0x40

/* REG_INT_SOURCE1 */
#define BIT_INT_WOM_X_INT1_EN       0x01
#define BIT_INT_WOM_Y_INT1_EN       0x02
#define BIT_INT_WOM_Z_INT1_EN       0x04
#define BIT_INT_SMD_INT1_EN         0x08
#define BIT_INT_WOM_XYZ_INT1_EN     \
    (BIT_INT_WOM_X_INT1_EN | BIT_INT_WOM_Y_INT1_EN | BIT_INT_WOM_Z_INT1_EN)

/*INT_SOURCE6 */
#define BIT_STEP_DET_INT1_EN		0x20
#define BIT_STEP_CNT_OFL_INT1_EN	0x10

/* REG_SENSOR_SELFTEST_REG1 */
#define BIT_ACCEL_SELF_TEST_PASS    0x08
#define BIT_GYRO_SELF_TEST_PASS     0x04
#define BIT_ACCEL_SELF_TEST_DONE    0x02
#define BIT_GYRO_SELF_TEST_DONE     0x01

/* REG_SELF_TEST_CONFIG */
#define BIT_SELF_TEST_REGULATOR_EN  0x40
#define BIT_TEST_AZ_EN              0x20
#define BIT_TEST_AY_EN              0x10
#define BIT_TEST_AX_EN              0x08
#define BIT_TEST_GZ_EN              0x04
#define BIT_TEST_GY_EN              0x02
#define BIT_TEST_GX_EN              0x01

/* REG_INT_STATUS */
#define BIT_INT_STATUS_AGC_RDY      0x01
#define BIT_INT_STATUS_FIFO_FULL    0x02
#define BIT_INT_STATUS_FIFO_THS     0x04
#define BIT_INT_STATUS_DRDY         0x08
#define BIT_INT_STATUS_RESET_DONE   0x10
#define BIT_INT_STATUS_PLL_DRY      0x20
#define BIT_INT_STATUS_UI_FSYNC     0x40

/* REG_INT_STATUS2 */
#define BIT_INT_STATUS_WOM_X        0x01
#define BIT_INT_STATUS_WOM_Y        0x02
#define BIT_INT_STATUS_WOM_Z        0x04
#define BIT_INT_STATUS_SMD          0x08
#define BIT_INT_STATUS_WOM_XYZ      \
    (BIT_INT_STATUS_WOM_X | BIT_INT_STATUS_WOM_Y | BIT_INT_STATUS_WOM_Z)

/* REG_INT_STATUS3 */
#define BIT_STEP_DET_INT			0x20
#define BIT_STEP_CNT_OVF_INT		0x10

/* REG_FIFO_CONFIG_REG */
#define BIT_FIFO_MODE_BYPASS        0x00
#define BIT_FIFO_MODE_STREAM        0x40
#define BIT_FIFO_MODE_STOP_FULL     0x80

/* REG_GYRO_ACCEL_CONFIG0 */
#define BIT_ACCEL_UI_LNM_BW_2_FIR   0x00
#define BIT_ACCEL_UI_LNM_BW_4_IIR   0x10
#define BIT_ACCEL_UI_LNM_BW_5_IIR   0x20
#define BIT_ACCEL_UI_LNM_BW_8_IIR   0x30
#define BIT_ACCEL_UI_LNM_BW_10_IIR  0x40
#define BIT_ACCEL_UI_LNM_BW_16_IIR  0x50
#define BIT_ACCEL_UI_LNM_BW_20_IIR  0x60
#define BIT_ACCEL_UI_LNM_BW_40_IIR  0x70
#define BIT_ACCEL_UI_LNM_AVG_1      0xF0
#define BIT_ACCEL_UI_LPM_BW_2_FIR   0x00
#define BIT_ACCEL_UI_LPM_AVG_1      0x10
#define BIT_ACCEL_UI_LPM_AVG_2      0x20
#define BIT_ACCEL_UI_LPM_AVG_3      0x30
#define BIT_ACCEL_UI_LPM_AVG_4      0x40
#define BIT_ACCEL_UI_LPM_AVG_8      0x50
#define BIT_ACCEL_UI_LPM_AVG_16     0x60
#define BIT_ACCEL_UI_LPM_AVG_32     0x70
#define BIT_ACCEL_UI_LPM_AVG_64     0x80
#define BIT_ACCEL_UI_LPM_AVG_128    0x90
#define BIT_GYRO_UI_LNM_BW_2_FIR    0x00
#define BIT_GYRO_UI_LNM_BW_4_IIR    0x01
#define BIT_GYRO_UI_LNM_BW_5_IIR    0x02
#define BIT_GYRO_UI_LNM_BW_8_IIR    0x03
#define BIT_GYRO_UI_LNM_BW_10_IIR   0x04
#define BIT_GYRO_UI_LNM_BW_16_IIR   0x05
#define BIT_GYRO_UI_LNM_BW_20_IIR   0x06
#define BIT_GYRO_UI_LNM_BW_40_IIR   0x07
#define BIT_GYRO_UI_LNM_AVG_1       0xF0
#define BIT_GYRO_UI_LPM_BW_2_FIR    0x00
#define BIT_GYRO_UI_LPM_AVG_1       0x01
#define BIT_GYRO_UI_LPM_AVG_2       0x02
#define BIT_GYRO_UI_LPM_AVG_3       0x03
#define BIT_GYRO_UI_LPM_AVG_4       0x04
#define BIT_GYRO_UI_LPM_AVG_8       0x05
#define BIT_GYRO_UI_LPM_AVG_16      0x06
#define BIT_GYRO_UI_LPM_AVG_32      0x07
#define BIT_GYRO_UI_LPM_AVG_64      0x08
#define BIT_GYRO_UI_LPM_AVG_128     0x09

/* fifo data packet header */
#define BIT_FIFO_HEAD_MSG           0x80
#define BIT_FIFO_HEAD_ACCEL         0x40
#define BIT_FIFO_HEAD_GYRO          0x20
#define BIT_FIFO_HEAD_20            0x10
#define BIT_FIFO_HEAD_TMSP_ODR      0x08
#define BIT_FIFO_HEAD_TMSP_NO_ODR   0x04
#define BIT_FIFO_HEAD_TMSP_FSYNC    0x0C
#define BIT_FIFO_HEAD_ODR_ACCEL     0x02
#define BIT_FIFO_HEAD_ODR_GYRO      0x01

/* REG_SMD_CONFIG */
#define BIT_WOM_INT_MODE_OR         0x00
#define BIT_WOM_INT_MODE_AND        0x08
#define BIT_WOM_MODE_INITIAL        0x00
#define BIT_WOM_MODE_PREV           0x04
#define BIT_SMD_MODE_OFF            0x00
#define BIT_SMD_MODE_OLD            0x01
#define BIT_SMD_MODE_SHORT          0x02
#define BIT_SMD_MODE_LONG           0x03
#define BIT_SMD_MODE_MASK      		0x03

/* REG_TMST_CONFIG */
#define BIT_FIFO_RAM_ISO_ENA        0x40
#define BIT_EN_DREG_FIFO_D2A        0x20
#define BIT_TMST_TO_REGS_EN         0x10
#define BIT_TMST_RESOL              0x08
#define BIT_TMST_DELTA_EN           0x04
#define BIT_TMST_FSYNC_EN           0x02
#define BIT_TMST_EN                 0x01

/* REG_DMP_CONFIG_MASK  */
//#define DMP_ODR_100HZ               0x03
#define DMP_ODR_50HZ                0x02
#define DMP_ODR_15HZ                0x00

/* DMP_ODR */
#define BIT_APEX_CONFIG0_DMP_ODR_POS       0
#define BIT_APEX_CONFIG0_DMP_ODR_MASK   (0x3 << BIT_APEX_CONFIG0_DMP_ODR_POS)

#define DMP_INIT_EN                 0x40
#define DMP_MEM_RESET_EN            0x20

#define PED_ENABLE                  0x20

/* I2C BUS CONFIG_MASK in BLANK1 */   
#define BIT_I3C_BUS_MODE			0x40
#define BIT_I3C_EN					0x10
#define BIT_I3C_SDR_EN				0x01
#define BIT_I3C_DDR_EN				0x02

/* REG_DRV_GYR_CFG0_REG */
#define GYRO_DRV_TEST_FSMFORCE_D2A_LINEAR_START_MODE            0x0D
#define GYRO_DRV_TEST_FSMFORCE_D2A_STEADY_STATE_AGC_REG_MODE    0x2A

/* REG_DRV_GYR_CFG2_REG */
#define GYRO_DRV_SPARE2_D2A_EN      0x01

/* INT configurations */
// Polarity: 0 -> Active Low, 1 -> Active High
#define INT_POLARITY    1
// Drive circuit: 0 -> Open Drain, 1 -> Push-Pull
#define INT_DRIVE_CIRCUIT    1
// Mode: 0 -> Pulse, 1 -> Latch
#define INT_MODE    0


/* data definitions */
#define FIFO_PACKET_BYTE_SINGLE     8
#define FIFO_PACKET_BYTE_6X         16
#define FIFO_PACKET_BYTE_HIRES      20
#define FIFO_COUNT_BYTE             2

/* sensor startup time */
#define INV_ICM40600_GYRO_START_TIME    40
#define INV_ICM40600_ACCEL_START_TIME   10

/* temperature sensor */
/* scale by 100, 1LSB=1degC, 9447 */
#define TEMP_SCALE                  100
/* 25 degC */
#define TEMP_OFFSET                 (25 * TEMP_SCALE)

#ifndef ACC_LOG
#define ACC_LOG		pr_err
#endif

#ifndef ACC_PR_ERR
#define ACC_PR_ERR		pr_err
#endif

#ifndef STEP_C_PR_ERR
#define STEP_C_PR_ERR		pr_err
#endif

#ifndef STEP_C_LOG
#define STEP_C_LOG		pr_err
#endif

#endif /* _INV_REG_40604_H_ */


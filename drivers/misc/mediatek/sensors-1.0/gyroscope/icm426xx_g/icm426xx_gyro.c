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

#include <linux/ioctl.h>
#include <cust_gyro.h>
#include <gyroscope.h>
#include <sensors_io.h>
#include "gyroscope.h"
#include "../../accelerometer/icm426xx_a/icm426xx_register.h"
#include "../../accelerometer/icm426xx_a/icm426xx_share.h"
#include "icm426xx_gyro.h"

#define ICM426XX_GYRO_DEV_NAME "ICM426XX_GYRO"
#define ICM_AD0_LOW                 1              //define for select i2c address 
#define MISC_DEVICE_FACTORY 0
//#define DEBUG

static bool icm426xx_gyro_first_enable = false;
//static int icm206xx_gyro_current_highest_samplerate = 0;
static int icm426xx_gyro_discardcount = 0;

struct icm426xx_gyro_i2c_data {
    struct i2c_client *client;
    struct gyro_hw *hw;
    struct hwmsen_convert cvt;
    /*misc*/
    atomic_t trace;
    atomic_t suspend;
    atomic_t selftest;
    atomic_t is_enabled;
    /*data*/
    s16 cali_sw[ICM426XX_AXIS_NUM+1];
    s16 data[ICM426XX_AXIS_NUM+1];
};

static int icm426xx_gyro_init_flag =  -1;

static struct i2c_client *icm426xx_gyro_i2c_client;
static struct icm426xx_gyro_i2c_data *obj_i2c_data;

#ifdef ICM426XX_SELFTEST
static char selftestRes[8] = { 0 };
#define SELF_TEST_GYR_BW_IND        BIT_GYRO_UI_LNM_BW_10_IIR
#endif

/* +/-1000DPS as default */
static int g_icm426xx_gyro_sensitivity = ICM426XX_GYRO_DEFAULT_SENSITIVITY;

static struct gyro_hw gyro_cust;
static struct gyro_hw *hw = &gyro_cust;

static int icm426xx_gyro_local_init(struct platform_device *pdev);
static int icm426xx_gyro_remove(void);
static int icm426xx_gyro_get_data(int *x , int *y, int *z, int *status);
static int icm426xx_gyro_enable_nodata(int en);
static int icm426xx_gyro_set_delay(u64 ns);
static struct gyro_init_info icm426xx_gyro_init_info = {
    .name = ICM426XX_GYRO_DEV_NAME,
    .init = icm426xx_gyro_local_init,
    .uninit = icm426xx_gyro_remove,
};

static int icm426xx_gyro_SetFullScale(struct i2c_client *client, u8 gyro_fsr)
{
    u8 databuf[2] = {0};
    int res = 0;

    res = icm426xx_share_read_register(REG_GYRO_CONFIG0, databuf, 1);
    if (res < 0) {
        GYRO_PR_ERR("read fsr register err!\n");
        return ICM426XX_ERR_BUS;
    }
    /* clear FSR bit */
    databuf[0] &= ~BIT_GYRO_FSR;
    databuf[0] |= gyro_fsr << SHIFT_GYRO_FS_SEL;
    g_icm426xx_gyro_sensitivity = (ICM426XX_GYRO_MAX_SENSITIVITY >> (3 - gyro_fsr)) + 1;
    res = icm426xx_share_write_register(REG_GYRO_CONFIG0, databuf, 1);
    if (res < 0){
        GYRO_PR_ERR("write fsr register err!\n");
        return ICM426XX_ERR_BUS;
    }
    return ICM426XX_SUCCESS;
}

static int icm426xx_gyro_SetFilter(struct i2c_client *client, u8 gyro_filter)
{
    u8 databuf[2] = {0};
    int res = 0;

    res = icm426xx_share_read_register(REG_GYRO_ACCEL_CONFIG0, databuf, 1);
    if (res < 0) {
        GYRO_PR_ERR("read filter register err!\n");
        return ICM426XX_ERR_BUS;
    }
    /* clear filter bit */
    databuf[0] &= ~BIT_GYRO_FILTER;
    databuf[0] |= gyro_filter;
    res = icm426xx_share_write_register(REG_GYRO_ACCEL_CONFIG0, databuf, 1);
    if (res < 0) {
        GYRO_PR_ERR("write filter register err!\n");
        return ICM426XX_ERR_BUS;
    }
    return ICM426XX_SUCCESS;
}

#ifdef ICM426XX_SELFTEST 
static int icm426xx_gyro_ReadSensorDataDirect(struct i2c_client *client,
    s16 data[ICM426XX_AXIS_NUM])
{
    char databuf[6];
    int i;
    int res = 0;

    if (client == NULL)
        return ICM426XX_ERR_INVALID_PARAM;
    res = icm426xx_share_read_register(REG_GYRO_DATA_X0_UI, databuf,
        ICM426XX_DATA_LEN);
    if (res < 0) {
        GYRO_PR_ERR("read gyroscope data error\n");
        return ICM426XX_ERR_BUS ;
    }
    /* convert 8-bit to 16-bit */
    for (i = 0; i < ICM426XX_AXIS_NUM; i++)
        data[i] = be16_to_cpup((__be16 *) (&databuf[i*2]));
    return ICM426XX_SUCCESS;
}
#else

#if MISC_DEVICE_FACTORY

static int icm426xx_gyro_ReadSensorDataDirect(struct i2c_client *client,
    s16 data[ICM426XX_AXIS_NUM])
{
    char databuf[6];
    int i;
    int res = 0;

    if (client == NULL)
        return ICM426XX_ERR_INVALID_PARAM;
    res = icm426xx_share_read_register(REG_GYRO_DATA_X0_UI, databuf,
        ICM426XX_DATA_LEN);
    if (res < 0) {
        GYRO_PR_ERR("read gyroscope data error\n");
        return ICM426XX_ERR_BUS ;
    }
    /* convert 8-bit to 16-bit */
    for (i = 0; i < ICM426XX_AXIS_NUM; i++)
        data[i] = be16_to_cpup((__be16 *) (&databuf[i*2]));
    return ICM426XX_SUCCESS;
}
	#endif 
#endif 

static int icm426xx_gyro_ReadSensorData(struct i2c_client *client,
    char *buf, int bufsize)
{
    char databuf[6];
    int  data[3];
	int  data_buff[3];
    int  i = 0;
    int res = 0;
	static int discard_count = 0;
    struct icm426xx_gyro_i2c_data * obj = i2c_get_clientdata(client);

    if (client == NULL)
        return ICM426XX_ERR_INVALID_PARAM;
    res = icm426xx_share_read_register(REG_GYRO_DATA_X0_UI, databuf,
        ICM426XX_DATA_LEN);
    if (res < 0) {
        GYRO_PR_ERR("read gyroscope data error\n");
        return ICM426XX_ERR_BUS;
    }

	if (icm426xx_gyro_first_enable){
		if(discard_count < icm426xx_gyro_discardcount){ 
			discard_count ++;
			GYRO_PR_ERR("discard times %d\n",discard_count);
            return ICM426XX_ERR_INVALID_PARAM;
        }
        else{
            discard_count =0;
            icm426xx_gyro_first_enable = false;
        }
    }       
    /* convert 8-bit to 16-bit */
    for (i = 0; i < ICM426XX_AXIS_NUM; i++) {
        obj->data[i] = be16_to_cpup((__be16 *) (&databuf[i*2]));
        /* add calibration value */
        // 1. Apply SENSITIVITY_SCALE_FACTOR calculated from SetFullScale 	(DPS)
		// 2. Translated it to MAX SENSITIVITY_SCALE_FACTOR			(DPS)
		data_buff[i] = obj->data[i] * ICM426XX_GYRO_MAX_SENSITIVITY / g_icm426xx_gyro_sensitivity;
    }
  
	#ifdef DEBUG

		GYRO_LOG("Gyro Data 1- %d %d %d\n", 
					data_buff[ICM426XX_AXIS_X], 
					data_buff[ICM426XX_AXIS_Y], 
					data_buff[ICM426XX_AXIS_Z]);

		GYRO_LOG("offset - %d %d %d\n", 
					obj->cali_sw[ICM426XX_AXIS_X], 
					obj->cali_sw[ICM426XX_AXIS_Y], 
					obj->cali_sw[ICM426XX_AXIS_Z]);
		
	    GYRO_LOG("Gyro Data 2- %d %d %d\n", 
					data_buff[ICM426XX_AXIS_X], 
					data_buff[ICM426XX_AXIS_Y], 
					data_buff[ICM426XX_AXIS_Z]);

		GYRO_LOG("Gyro map x- %d %d \n", 
					obj->cvt.map[ICM426XX_AXIS_X], 
					obj->cvt.sign[ICM426XX_AXIS_X]);
		GYRO_LOG("Gyro map y- %d %d \n", 
					obj->cvt.map[ICM426XX_AXIS_Y], 
					obj->cvt.sign[ICM426XX_AXIS_Y]);	
		GYRO_LOG("Gyro map z- %d %d \n", 
					obj->cvt.map[ICM426XX_AXIS_Z], 
					obj->cvt.sign[ICM426XX_AXIS_Z]);	
	#endif
	/* Add Cali */
		//obj->data[i] += obj->cali_sw[i];
        data_buff[ICM426XX_AXIS_X]+= obj->cali_sw[ICM426XX_AXIS_X];
		data_buff[ICM426XX_AXIS_Y]+= obj->cali_sw[ICM426XX_AXIS_Y];
  		data_buff[ICM426XX_AXIS_Z]+= obj->cali_sw[ICM426XX_AXIS_Z];

		/* 3 . Orientation Translation Lower (sensor) --> Upper (Device) */
		data[obj->cvt.map[ICM426XX_AXIS_X]] = obj->cvt.sign[ICM426XX_AXIS_X] * data_buff[ICM426XX_AXIS_X];
		data[obj->cvt.map[ICM426XX_AXIS_Y]] = obj->cvt.sign[ICM426XX_AXIS_Y] * data_buff[ICM426XX_AXIS_Y];
		data[obj->cvt.map[ICM426XX_AXIS_Z]] = obj->cvt.sign[ICM426XX_AXIS_Z] * data_buff[ICM426XX_AXIS_Z];
    	sprintf(buf, "%04x %04x %04x",
        data[ICM426XX_AXIS_X],
        data[ICM426XX_AXIS_Y],
        data[ICM426XX_AXIS_Z]);
    if (atomic_read(&obj->trace)) {
        GYRO_LOG("Gyroscope data - %04x %04x %04x\n",
            data[ICM426XX_AXIS_X],
            data[ICM426XX_AXIS_Y],
            data[ICM426XX_AXIS_Z]);
    }
    return ICM426XX_SUCCESS;
}



static int icm426xx_gyro_ResetCalibration(struct i2c_client *client)
{
    struct icm426xx_gyro_i2c_data *obj = i2c_get_clientdata(client);

    memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
    return ICM426XX_SUCCESS;
}

static int icm426xx_gyro_ReadCalibration(struct i2c_client *client,
    struct SENSOR_DATA *sensor_data)
{
    struct icm426xx_gyro_i2c_data *obj = i2c_get_clientdata(client);
    int cali[ICM426XX_AXIS_NUM];
	
    cali[obj->cvt.map[ICM426XX_AXIS_X]] = obj->cvt.sign[ICM426XX_AXIS_X]*obj->cali_sw[ICM426XX_AXIS_X];
    cali[obj->cvt.map[ICM426XX_AXIS_Y]] = obj->cvt.sign[ICM426XX_AXIS_Y]*obj->cali_sw[ICM426XX_AXIS_Y];
    cali[obj->cvt.map[ICM426XX_AXIS_Z]] = obj->cvt.sign[ICM426XX_AXIS_Z]*obj->cali_sw[ICM426XX_AXIS_Z];
	sensor_data->x = cali[ICM426XX_AXIS_X] ;
	sensor_data->y = cali[ICM426XX_AXIS_Y] ;
	sensor_data->z = cali[ICM426XX_AXIS_Z] ;
    if (atomic_read(&obj->trace)) {
        GYRO_LOG("Gyro ReadCalibration:[sensor_data:%5d %5d %5d]\n",
            sensor_data->x, sensor_data->y, sensor_data->z);
        GYRO_LOG("Gyro ReadCalibration:[cali_sw:%5d %5d %5d]\n",
            obj->cali_sw[ICM426XX_AXIS_X],
            obj->cali_sw[ICM426XX_AXIS_Y],
            obj->cali_sw[ICM426XX_AXIS_Z]);
    }
    return ICM426XX_SUCCESS;
}

static int icm426xx_gyro_write_rel_calibration(struct icm426xx_gyro_i2c_data *obj, int dat[ICM_GYRO_AXES_NUM])
{
   	obj->cali_sw[ICM426XX_AXIS_X] = obj->cvt.sign[ICM426XX_AXIS_X]*dat[obj->cvt.map[ICM426XX_AXIS_X]];
    obj->cali_sw[ICM426XX_AXIS_Y] = obj->cvt.sign[ICM426XX_AXIS_Y]*dat[obj->cvt.map[ICM426XX_AXIS_Y]];
	obj->cali_sw[ICM426XX_AXIS_Z] = obj->cvt.sign[ICM426XX_AXIS_Z]*dat[obj->cvt.map[ICM426XX_AXIS_Z]];
#ifdef DEBUG
    GYRO_LOG("test (%5d, %5d, %5d))\n",obj->cali_sw[ICM426XX_AXIS_X],obj->cali_sw[ICM426XX_AXIS_Y],obj->cali_sw[ICM426XX_AXIS_Z]);		
#endif
    return 0;
}

static int icm426xx_gyro_WriteCalibration(struct i2c_client *client,
    struct SENSOR_DATA *sensor_data)
{
    struct icm426xx_gyro_i2c_data *obj = i2c_get_clientdata(client);
    int cali[ICM426XX_AXIS_NUM];
	
    cali[obj->cvt.map[ICM426XX_AXIS_X]] = obj->cvt.sign[ICM426XX_AXIS_X]*obj->cali_sw[ICM426XX_AXIS_X];
	cali[obj->cvt.map[ICM426XX_AXIS_Y]] = obj->cvt.sign[ICM426XX_AXIS_Y]*obj->cali_sw[ICM426XX_AXIS_Y];
	cali[obj->cvt.map[ICM426XX_AXIS_Z]] = obj->cvt.sign[ICM426XX_AXIS_Z]*obj->cali_sw[ICM426XX_AXIS_Z];
	cali[ICM426XX_AXIS_X] += sensor_data ->x;
	cali[ICM426XX_AXIS_Y] += sensor_data ->y;
	cali[ICM426XX_AXIS_Z] += sensor_data ->z;
#ifdef DEBUG
	GYRO_LOG("write gyro calibration data  (%5d, %5d, %5d)-->(%5d, %5d, %5d)\n",
			sensor_data ->x, sensor_data ->y, sensor_data ->z,
			cali[ICM426XX_AXIS_X],cali[ICM426XX_AXIS_Y],cali[ICM426XX_AXIS_Z]);
#endif
	return icm426xx_gyro_write_rel_calibration(obj, cali);
}

#ifdef ICM426XX_SELFTEST
/*static int icm426xx_gyro_ResetOffsetReg(void)
{
    int res = 0;
    char databuf[2] = {0};

    databuf[0] = 0x00;
    res = icm426xx_share_write_register(REG_GOS_USER0, databuf, 1);
    if (res < 0) {
        GYRO_PR_ERR("write bank select register err!\n");
        return ICM426XX_ERR_BUS;
    }
    res = icm426xx_share_write_register(REG_GOS_USER1, databuf, 1);
    if (res < 0) {
        GYRO_PR_ERR("write bank select register err!\n");
        return ICM426XX_ERR_BUS;
    }
    res = icm426xx_share_write_register(REG_GOS_USER2, databuf, 1);
    if (res < 0) {
        GYRO_PR_ERR("write bank select register err!\n");
        return ICM426XX_ERR_BUS;
    }
    res = icm426xx_share_write_register(REG_GOS_USER3, databuf, 1);
    if (res < 0) {
        GYRO_PR_ERR("write bank select register err!\n");
        return ICM426XX_ERR_BUS;
    }
    res = icm426xx_share_read_register(REG_GOS_USER4, 
        databuf, 1);
    if (res) {
        GYRO_PR_ERR("read offset fail: %d\n", res);
        return false;
    }
    databuf[0] &= 0xf0;
    res = icm426xx_share_write_register(REG_GOS_USER4, databuf, 1);
    if (res < 0) {
        GYRO_PR_ERR("write bank select register err!\n");
        return ICM426XX_ERR_BUS;
    }
    return ICM426XX_SUCCESS;
}*/

static int  icm426xx_gyro_InitSelfTest(struct i2c_client * client)
{
    int res = 0;

    /* softreset */
    res = icm426xx_share_ChipSoftReset();
    if (res != ICM426XX_SUCCESS)
        return res;
    /* set power mode */
    icm426xx_share_set_sensor_power_mode(ICM426XX_SENSOR_TYPE_GYRO,
        BIT_GYRO_MODE_LNM);
    /* reset offset
    res = icm426xx_gyro_ResetOffsetReg();
    if (res != ICM426XX_SUCCESS)
        return res;*/
    /* setpowermode(true) --> exit sleep */
    res = icm426xx_share_SetPowerMode(ICM426XX_SENSOR_TYPE_GYRO, true);
    if (res != ICM426XX_SUCCESS)
        return res;
	/* fsr : ICM426XX_GYRO_RANGE_250DPS */
    res = icm426xx_gyro_SetFullScale(client, ICM426XX_GYRO_RANGE_250DPS);
    if (res != ICM426XX_SUCCESS)
        return res;
    /* filter : SELF_TEST_GYR_BW_IND */
    res = icm426xx_gyro_SetFilter(client, SELF_TEST_GYR_BW_IND);
    if (res != ICM426XX_SUCCESS)
        return res;
    /* odr : 1000hz (1kHz) */
    res = icm426xx_share_SetSampleRate(ICM426XX_SENSOR_TYPE_GYRO,
        1000000, true);
    if (res != ICM426XX_SUCCESS)
        return res;
    /* set enable sensor */
    res = icm426xx_share_EnableSensor(ICM426XX_SENSOR_TYPE_GYRO, true,&icm426xx_gyro_first_enable);
    if (res != ICM426XX_SUCCESS)
        return res;
    /* delay for selftest */
    mdelay(SELF_TEST_READ_INTERVAL_MS);
    return res;
}

static int icm426xx_gyro_CalcAvgWithSamples(struct i2c_client *client,
    int avg[3], int count)
{
    int res = 0;
    int i, nAxis;
    s16 sensor_data[ICM426XX_AXIS_NUM];
    s32 sum[ICM426XX_AXIS_NUM] = {0,};

    for (i = 0; i < count; i++) {
        res = icm426xx_gyro_ReadSensorDataDirect(client, sensor_data);
        if (res) {
            GYRO_PR_ERR("read data fail: %d\n", res);
            return ICM426XX_ERR_STATUS;
        }
        for (nAxis = 0; nAxis < ICM426XX_AXIS_NUM; nAxis++)
            sum[nAxis] += sensor_data[nAxis];
        /* data register updated @1khz */
        mdelay(1);
    }
    for (nAxis = 0; nAxis < ICM426XX_AXIS_NUM; nAxis++)
        avg[nAxis] = (int)(sum[nAxis] / count) * SELF_TEST_PRECISION;
    return ICM426XX_SUCCESS;
}

static bool icm426xx_gyro_DoSelfTest(struct i2c_client *client)
{
    int res = 0;
    int i;
    int gyro_ST_on[ICM426XX_AXIS_NUM], gyro_ST_off[ICM426XX_AXIS_NUM];
    /* index of otp_lookup_tbl */
    u8  st_code[ICM426XX_AXIS_NUM];
    u16 st_otp[ICM426XX_AXIS_NUM];
    bool otp_value_has_zero = false;
    bool test_result = true;
    u8 databuf[2] = {0};
    int st_res;
    int retry;

    databuf[0] = BIT_BANK_SEL_1;
    res = icm426xx_share_write_register(REG_REG_BANK_SEL, databuf, 1);
    if (res < 0) {
        GYRO_PR_ERR("write bank select register err!\n");
        return ICM426XX_ERR_BUS;
    }
    /* acquire OTP value from OTP lookup table */
    res = icm426xx_share_read_register(REG_XG_ST_DATA, 
        &(st_code[0]), 3);
    if (res) {
        GYRO_PR_ERR("read data fail: %d\n", res);
        return false;
    }
    GYRO_LOG("st_code: %02x, %02x, %02x\n",
        st_code[0], st_code[1], st_code[2]);
    databuf[0] = BIT_BANK_SEL_0;
    res = icm426xx_share_write_register(REG_REG_BANK_SEL, databuf, 1);
    if (res < 0) {
        GYRO_PR_ERR("write bank select register err!\n");
        return ICM426XX_ERR_BUS;
    }
    /* lookup OTP value with st_code */
    for (i = 0; i < ICM426XX_AXIS_NUM; i++) {
        if (st_code[i] != 0)
            st_otp[i] = st_otp_lookup_tbl[ st_code[i] - 1];
        else {
            st_otp[i] = 0;
            otp_value_has_zero = true;
        }
    }
    /* read sensor data and calculate average values from it */
    for (retry = 0 ; retry < RETRY_CNT_SELF_TEST ; retry++ ) {
        /* read 200 samples with selftest off */
        res = icm426xx_gyro_CalcAvgWithSamples(client, gyro_ST_off,
            SELF_TEST_SAMPLE_NB);
        if (res) {
            GYRO_PR_ERR("read data fail: %d\n", res);
            return false;
        }
        /* set selftest on */
        databuf[0] = (BIT_TEST_GX_EN | BIT_TEST_GY_EN | BIT_TEST_GZ_EN);
        res = icm426xx_share_write_register(REG_SELF_TEST_CONFIG, databuf, 1);
        if (res) {
            GYRO_PR_ERR("enable st gyro fail: %d\n", res);
            return false;
        }
        /* wait 20ms for oscillations to stabilize */
        mdelay(20);
        /* Read 200 Samples with selftest on */
        res = icm426xx_gyro_CalcAvgWithSamples(client, gyro_ST_on,
            SELF_TEST_SAMPLE_NB);
        if (res) {
            GYRO_PR_ERR("read data fail: %d\n", res);
            return false;
        }
        /* set selftest off */
        databuf[0] = 0x00;
        res = icm426xx_share_write_register(REG_SELF_TEST_CONFIG, databuf, 1);
        if (res) {
            GYRO_PR_ERR("disable st gyro fail: %d\n", res);
            return false;
        }
        /* wait 20ms for oscillations to stabilize */
        mdelay(20);
        /* compare calculated value with OTP value to judge success or fail */
        if (!otp_value_has_zero) {
            /* criteria a */
            for (i = 0; i < ICM426XX_AXIS_NUM; i++) {
                st_res = gyro_ST_on[i] - gyro_ST_off[i];
                if (st_res <= st_otp[i] * SELF_TEST_GYR_SHIFT_DELTA) {
                    GYRO_LOG("error gyro[%d] : st_res = %d, st_otp = %d\n",
                        i, st_res, st_otp[i]);
                    test_result = false;
                }
            }
        } else {
            /* criteria b */
            for (i = 0; i < ICM426XX_AXIS_NUM; i++) {
                st_res = abs(gyro_ST_on[i] - gyro_ST_off[i]);
                if (st_res < SELF_TEST_MIN_GYR) {
                    GYRO_LOG("error gyro[%d] : st_res = %d, min = %d\n",
                        i, st_res, SELF_TEST_MIN_GYR);
                    test_result = false;
                }
            }
        }
        if (test_result) {
            /* criteria c */
            for (i = 0; i < ICM426XX_AXIS_NUM; i++) {
                if (abs(gyro_ST_off[i]) > SELF_TEST_MAX_GYR_OFFSET) {
                    GYRO_LOG("error gyro[%d] = %d, max = %d\n",
                        i, abs(gyro_ST_off[i]), SELF_TEST_MAX_GYR_OFFSET);
                    test_result = false;
                }
            }
        }
        if (test_result)
            break;
    }
    return test_result;
}
#endif

static int icm426xx_gyro_init_client(struct i2c_client *client, bool enable)
{
    int res = 0;
    u8 databuf[2] = {0};

	char strbuf[ICM426XX_BUFSIZE];
	icm426xx_share_ReadChipInfo(strbuf, ICM426XX_BUFSIZE);
	GYRO_LOG("%s \n",strbuf);
    /* xxit sleep mode */
    res = icm426xx_share_SetPowerMode(ICM426XX_SENSOR_TYPE_GYRO, true);
    if (res != ICM426XX_SUCCESS)
        return res;
    /* set fsr +/-1000 dps as default */
    res = icm426xx_gyro_SetFullScale(client, ICM426XX_GYRO_RANGE_1000DPS);
    if (res != ICM426XX_SUCCESS)
        return res;
#ifdef ICM426XX_ACCEL_LOW_POWER_MODE
    /* set power mode */
    icm426xx_share_set_sensor_power_mode(ICM426XX_SENSOR_TYPE_GYRO,
        BIT_GYRO_MODE_LPM);
    /* set filter BIT_ACCEL_UI_LPM_BW_2_FIR as default */
    res = icm426xx_gyro_SetFilter(client, BIT_GYRO_UI_LPM_AVG_1);
    if (res != ICM426XX_SUCCESS)
        return res;
#else
    /* set power mode */
    icm426xx_share_set_sensor_power_mode(ICM426XX_SENSOR_TYPE_GYRO,
        BIT_GYRO_MODE_LNM);
    /* set filter BIT_ACCEL_UI_LNM_BW_2_FIR as default */
    res = icm426xx_gyro_SetFilter(client, BIT_GYRO_UI_LNM_BW_2_FIR);
    if (res != ICM426XX_SUCCESS)
        return res;
#endif
    /* set 5ms(200hz) sample rate */
    res = icm426xx_share_SetSampleRate(ICM426XX_SENSOR_TYPE_GYRO,
        5000000, false);
    if (res != ICM426XX_SUCCESS)
        return res;
    /* set gyro config1 as default */
    databuf[0] = 0x1A;
    res = icm426xx_share_write_register(REG_GYRO_CONFIG1, databuf, 1);
    if (res != ICM426XX_SUCCESS)
        return res;
    /* disable sensor - standby mode for gyroscope */
    res = icm426xx_share_EnableSensor(ICM426XX_SENSOR_TYPE_GYRO, enable,&icm426xx_gyro_first_enable );
    if (res != ICM426XX_SUCCESS)
        return res;
    /* set power mode - sleep or normal */
    res = icm426xx_share_SetPowerMode(ICM426XX_SENSOR_TYPE_GYRO, enable);
    if (res != ICM426XX_SUCCESS)
        return res;
    GYRO_LOG("icm426xx_gyro_init_client OK!\n");
    return ICM426XX_SUCCESS;
}

struct gyro_hw *get_cust_gyro(void)
{
    return &gyro_cust;
}

static void icm426xx_gyro_power(struct gyro_hw *hw, unsigned int on)
{
    /* nothing to do here, because the power of sensor is always on */
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
    char strbuf[ICM426XX_BUFSIZE];

    icm426xx_share_ReadChipInfo(strbuf, ICM426XX_BUFSIZE);
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
    char strbuf[ICM426XX_BUFSIZE];
    struct i2c_client *client = icm426xx_gyro_i2c_client;

    if (NULL == client) {
        GYRO_PR_ERR("i2c client is null!!\n");
        return 0;
    }
    icm426xx_gyro_ReadSensorData(client, strbuf, ICM426XX_BUFSIZE);
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
    ssize_t res = 0;
    struct icm426xx_gyro_i2c_data *obj = obj_i2c_data;

    if (obj == NULL) {
        GYRO_PR_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
    return res;
}

static ssize_t store_trace_value(struct device_driver *ddri,
    const char *buf, size_t count)
{
    int trace;
    struct icm426xx_gyro_i2c_data *obj = obj_i2c_data;

    if (obj == NULL) {
        GYRO_PR_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    if (0 == kstrtoint(buf, 16, &trace))
        atomic_set(&obj->trace, trace);
    else
        GYRO_PR_ERR("invalid content: '%s', length = %zu\n", buf, count);
    return count;
}

static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct icm426xx_gyro_i2c_data *obj = obj_i2c_data;

    if (obj == NULL) {
        GYRO_PR_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    if (obj->hw)
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
            obj->hw->i2c_num,
            obj->hw->direction,
            obj->hw->power_id,
            obj->hw->power_vol);
    else
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
    return len;
}

static ssize_t show_chip_orientation(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct icm426xx_gyro_i2c_data *obj = obj_i2c_data;

    if (obj == NULL) {
        GYRO_PR_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    GYRO_LOG("[%s] default direction: %d\n", __func__, obj->hw->direction);
    len = snprintf(buf, PAGE_SIZE, "default direction = %d\n",
        obj->hw->direction);
    return len;
}

static ssize_t store_chip_orientation(struct device_driver *ddri,
    const char *buf, size_t tCount)
{
    int nDirection = 0;
    int res = 0;
    struct icm426xx_gyro_i2c_data   *obj = obj_i2c_data;

    if (obj == NULL) {
        GYRO_PR_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    res = kstrtoint(buf, 10, &nDirection);
    if (res != 0) {
        if (hwmsen_get_convert(nDirection, &obj->cvt))
            GYRO_PR_ERR("ERR: fail to set direction\n");
    }
    GYRO_LOG("[%s] set direction: %d\n", __func__, nDirection);
    return tCount;
}

#ifdef ICM426XX_SELFTEST
static ssize_t show_selftest_value(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = icm426xx_gyro_i2c_client;

    if (NULL == client) {
        GYRO_PR_ERR("i2c client is null!!\n");
        return 0;
    }
    return snprintf(buf, 8, "%s\n", selftestRes);
}

static ssize_t store_selftest_value(struct device_driver *ddri,
    const char *buf, size_t count)
{
    struct i2c_client *client = icm426xx_gyro_i2c_client;
    int num;
    int res = 0;

    /* check parameter values to run selftest */
    res = kstrtoint(buf, 10, &num);
    if (res != 0) {
        GYRO_PR_ERR("parse number fail\n");
        return count;
    } else if (num == 0) {
        GYRO_PR_ERR("invalid data count\n");
        return count;
    }
    /* run selftest */
    res = icm426xx_gyro_InitSelfTest(client);
    if (icm426xx_gyro_DoSelfTest(client) == true) {
        strcpy(selftestRes, "y");
        GYRO_LOG("GYRO SELFTEST : PASS\n");
    } else {
        strcpy(selftestRes, "n");
        GYRO_LOG("GYRO SELFTEST : FAIL\n");
    }
    /* selftest is considered to be called only in factory mode
    in general mode, the condition before selftest will not be recovered
    and sensor will not be in sleep mode */
    res = icm426xx_gyro_init_client(client, true);
    return count;
}
#endif

static DRIVER_ATTR(chipinfo, S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata,  S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(trace,  S_IWUSR | S_IRUGO, show_trace_value,
    store_trace_value);
static DRIVER_ATTR(status,  S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(orientation, S_IWUSR | S_IRUGO, show_chip_orientation,
    store_chip_orientation);
#ifdef ICM426XX_SELFTEST
static DRIVER_ATTR(selftest, S_IWUSR | S_IRUGO, show_selftest_value,
    store_selftest_value);
#endif

static struct driver_attribute *icm426xx_gyro_attr_list[] = {
    /* chip information - whoami */
    &driver_attr_chipinfo,
    /* dump sensor data */
    &driver_attr_sensordata,
    /* trace log */
    &driver_attr_trace,
    /* chip status */
    &driver_attr_status,
    /* chip orientation information */
    &driver_attr_orientation,
#ifdef ICM426XX_SELFTEST
    /* run selftest when store, report selftest result when show */
    &driver_attr_selftest,
#endif
};

static int icm426xx_gyro_create_attr(struct device_driver *driver)
{
    int idx;
    int res = 0;
    int num = (int)(sizeof(icm426xx_gyro_attr_list)/
        sizeof(icm426xx_gyro_attr_list[0]));

    if (driver == NULL)
        return -EINVAL;
    for (idx = 0; idx < num; idx++) {
        res = driver_create_file(driver, icm426xx_gyro_attr_list[idx]);
        if (0 != res) {
            GYRO_PR_ERR("driver_create_file (%s) = %d\n",
                icm426xx_gyro_attr_list[idx]->attr.name, res);
            break;
        }
    }
    return res;
}

static int icm426xx_gyro_delete_attr(struct device_driver *driver)
{
    int idx;
    int res = 0;
    int num = (int)(sizeof(icm426xx_gyro_attr_list)/
        sizeof(icm426xx_gyro_attr_list[0]));

    if (driver == NULL)
        return -EINVAL;
    for (idx = 0; idx < num; idx++)
        driver_remove_file(driver, icm426xx_gyro_attr_list[idx]);
    return res;
}

/*=======================================================================================*/
/* Misc - Factory Mode (IOCTL) Device Driver Section					 */
/*=======================================================================================*/
#if MISC_DEVICE_FACTORY

static int icm426xx_gyro_ReadRawData(struct i2c_client * client, char * buf)
{
    int res = 0;
    s16 data[ICM426XX_AXIS_NUM] = { 0, 0, 0 };

    res = icm426xx_gyro_ReadSensorDataDirect(client, data);
    if (res < 0) {
        GYRO_PR_ERR("read gyroscope raw data  error\n");
        return ICM426XX_ERR_BUS;
    }
    /* sensor raw data direct read from sensor register
    no orientation translation, no unit translation */
    sprintf(buf, "%04x %04x %04x",
        data[ICM426XX_AXIS_X],
        data[ICM426XX_AXIS_Y],
        data[ICM426XX_AXIS_Z]);
    return ICM426XX_SUCCESS;
}

static int icm426xx_gyro_open(struct inode *inode, struct file *file)
{
    file->private_data = icm426xx_gyro_i2c_client;

    if (file->private_data == NULL) {
        GYRO_PR_ERR("null pointer!!\n");
        return -EINVAL;
    }
    return nonseekable_open(inode, file);
}

static int icm426xx_gyro_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}

static long icm426xx_gyro_unlocked_ioctl(struct file *file,
    unsigned int cmd, unsigned long arg)
{
    struct i2c_client *client = (struct i2c_client *)file->private_data;
    char strbuf[ICM426XX_BUFSIZE] = {0};
    void __user *data;
    long res = 0;
    int copy_cnt = 0;
    struct SENSOR_DATA sensor_data;
    int smtRes = 0;

    if (_IOC_DIR(cmd) & _IOC_READ)
        res = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
        res = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    if (res) {
        GYRO_PR_ERR("access error: %08X, (%2d, %2d)\n",
            cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
        return -EFAULT;
    }
    switch (cmd) {
    case GYROSCOPE_IOCTL_INIT:
        icm426xx_gyro_init_client(client, true);
        break;
    case GYROSCOPE_IOCTL_SMT_DATA:
        data = (void __user *) arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        GYRO_LOG("ioctl smtRes: %d!\n", smtRes);
        copy_cnt = copy_to_user(data, &smtRes,  sizeof(smtRes));
        if (copy_cnt) {
            res = -EFAULT;
            GYRO_PR_ERR("copy gyro data to user failed!\n");
        }
        GYRO_LOG("copy gyro data to user OK: %d!\n", copy_cnt);
        break;
    case GYROSCOPE_IOCTL_READ_SENSORDATA:
        data = (void __user *) arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        icm426xx_gyro_ReadSensorData(client, strbuf, ICM426XX_BUFSIZE);
        if (copy_to_user(data, strbuf, sizeof(strbuf))) {
            res = -EFAULT;
            break;
        }
        break;
    case GYROSCOPE_IOCTL_SET_CALI:
        data = (void __user *)arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        if (copy_from_user(&sensor_data, data, sizeof(sensor_data)))
            res = -EFAULT;
        else {
            GYRO_LOG("gyro set cali:[%5d %5d %5d]\n",
                sensor_data.x, sensor_data.y, sensor_data.z);
            res = icm426xx_gyro_WriteCalibration(client, &sensor_data);
        }
        break;
    case GYROSCOPE_IOCTL_GET_CALI:
        data = (void __user *)arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        res = icm426xx_gyro_ReadCalibration(client, &sensor_data);
        if (copy_to_user(data, &sensor_data, sizeof(sensor_data))) {
            res = -EFAULT;
            break;
        }
        break;
    case GYROSCOPE_IOCTL_CLR_CALI:
        res = icm426xx_gyro_ResetCalibration(client);
        break;
    case GYROSCOPE_IOCTL_READ_SENSORDATA_RAW:
        data = (void __user *)arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        icm426xx_gyro_ReadRawData(client, strbuf);
        if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
            res = -EFAULT;
            break;
        }
        break;
    default:
        GYRO_PR_ERR("unknown IOCTL: 0x%08x\n", cmd);
        res = -ENOIOCTLCMD;
    }
    return res;
}

#ifdef CONFIG_COMPAT
static long icm426xx_gyro_compat_ioctl(struct file *file,
    unsigned int cmd, unsigned long arg)
{
    long res = 0;
    void __user *arg32 = compat_ptr(arg);

    if(!file->f_op || !file->f_op->unlocked_ioctl){
        GYRO_PR_ERR("compat_ion_ioctl file has no f_op\n");
        GYRO_PR_ERR("or no f_op->unlocked_ioctl.\n");
        return -ENOTTY;
    }
    switch (cmd) {
    case COMPAT_GYROSCOPE_IOCTL_INIT:
    case COMPAT_GYROSCOPE_IOCTL_SMT_DATA:
    case COMPAT_GYROSCOPE_IOCTL_READ_SENSORDATA:
    case COMPAT_GYROSCOPE_IOCTL_SET_CALI:
    case COMPAT_GYROSCOPE_IOCTL_GET_CALI:
    case COMPAT_GYROSCOPE_IOCTL_CLR_CALI:
    case COMPAT_GYROSCOPE_IOCTL_READ_SENSORDATA_RAW:
    case COMPAT_GYROSCOPE_IOCTL_READ_TEMPERATURE:
    case COMPAT_GYROSCOPE_IOCTL_GET_POWER_STATUS:
        if (arg32 == NULL) {
            GYRO_PR_ERR("invalid argument.");
            return -EINVAL;
        }
        res = file->f_op->unlocked_ioctl(file, cmd, (unsigned long)arg32);
        break;
    default:
        GYRO_PR_ERR("%s not supported = 0x%04x\n", __func__, cmd);
        res = -ENOIOCTLCMD;
        break;
    }
    return res;
}
#endif

static const struct file_operations icm426xx_gyro_fops = {
    .open = icm426xx_gyro_open,
    .release = icm426xx_gyro_release,
    .unlocked_ioctl = icm426xx_gyro_unlocked_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = icm426xx_gyro_compat_ioctl,
#endif
};

static struct miscdevice icm426xx_gyro_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "gyroscope",
    .fops = &icm426xx_gyro_fops,
};
#else 

/************************* For MTK New factory mode ************************************/
static int icm426xx_gyro_factory_do_self_test(void)
{
        return 0;
}

static int icm426xx_gyro_factory_get_cali(int32_t data[3])
{
        int err;
		struct icm426xx_gyro_i2c_data *priv = obj_i2c_data;
        struct SENSOR_DATA  sensor_data;
		GYRO_FUN();
	
        err = icm426xx_gyro_ReadCalibration(priv->client, &sensor_data);
        if (err) {
                GYRO_LOG("icm426xx_gyro_ReadCalibration failed!\n");
                return -1;
        }
        data[0] = sensor_data.x ;
        data[1] = sensor_data.y ;
        data[2] = sensor_data.z ;
        return 0;
}

static int icm426xx_gyro_factory_set_cali(int32_t data[3])
{
        int err = 0;
		struct icm426xx_gyro_i2c_data *priv = obj_i2c_data;
		struct SENSOR_DATA  sensor_data;
        GYRO_FUN();
        GYRO_LOG("gyro set cali:[%5d %5d %5d]\n", data[0], data[1], data[2]);
        sensor_data.x = data[0] ;
        sensor_data.y = data[1] ;
        sensor_data.z = data[2] ;	
        err = icm426xx_gyro_WriteCalibration(priv->client, &sensor_data);
        if (err) {
                GYRO_LOG("406xx_gyro_WriteCalibration failed!\n");
                return -1;
        }
        return 0;
}

static int icm426xx_gyro_factory_enable_calibration(void)
{
        return 0;
}
static int icm426xx_gyro_factory_clear_cali(void)
{
        int err = 0;
        struct icm426xx_gyro_i2c_data *priv = obj_i2c_data;
		GYRO_FUN();
        err = icm426xx_gyro_ResetCalibration(priv->client);
        if (err) {
                GYRO_LOG("icm426xx_gyro_factory_clear_cali failed!\n");
                return -1;
        }
        return 0;
}

static int icm426xx_gyro_factory_get_raw_data(int32_t data[3])
{
        
        GYRO_LOG("do not support raw data now!\n");
        return 0;
}

static int icm426xx_gyro_factory_get_data(int32_t data[3], int *status)
{
        int res;	
        res =  icm426xx_gyro_get_data(&data[0], &data[1], &data[2], status);
		GYRO_LOG("%s  %d %d %d!\n", __func__,data[0], data[1], data[2]);
		return res;

}

static int icm426xx_gyro_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
        int err;

        err = icm426xx_gyro_enable_nodata(enabledisable == true ? 1 : 0);
        if (err) {
                GYRO_LOG("%s enable gyro sensor failed!\n", __func__);
                return -1;
        }
        return 0;
}

static struct gyro_factory_fops icm426xx_gyro_factory_fops = {
        .enable_sensor =icm426xx_gyro_factory_enable_sensor,
        .get_data = icm426xx_gyro_factory_get_data,
        .get_raw_data =icm426xx_gyro_factory_get_raw_data,
        .enable_calibration = icm426xx_gyro_factory_enable_calibration,
        .clear_cali = icm426xx_gyro_factory_clear_cali,
        .set_cali = icm426xx_gyro_factory_set_cali,
        .get_cali = icm426xx_gyro_factory_get_cali,
        .do_self_test = icm426xx_gyro_factory_do_self_test,
};

static struct gyro_factory_public icm426xx_gyro_factory_device = {
        .gain = 1,
        .sensitivity = 1,
        .fops = &icm426xx_gyro_factory_fops,
};


#endif

static int icm426xx_gyro_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{

    return icm426xx_gyro_set_delay((u64)samplingPeriodNs);
}

static int icm426xx_gyro_flush(void)
{
    return gyro_flush_report();
}

static int icm426xx_gyro_open_report_data(int open)
{
    /* nothing to do here for 406xx */
    return 0;
}

static int icm426xx_gyro_enable_nodata(int en)
{
    /* if use this type of enable , 
    gsensor only enabled but not report inputEvent to HAL */
    int res = 0;
    bool power = false;
    struct icm426xx_gyro_i2c_data *obj = obj_i2c_data;

    if (1 == en) {
        power = true;
		res = icm426xx_share_EnableSensor(ICM426XX_SENSOR_TYPE_GYRO, power, &icm426xx_gyro_first_enable);
		res |= icm426xx_share_SetPowerMode(ICM426XX_SENSOR_TYPE_GYRO, power);
    } else {
        power = false;
        res = icm426xx_share_SetSampleRate(ICM426XX_SENSOR_TYPE_GYRO,
            0, false);
        res |= icm426xx_share_EnableSensor(ICM426XX_SENSOR_TYPE_GYRO, power,&icm426xx_gyro_first_enable);
        res |= icm426xx_share_SetPowerMode(ICM426XX_SENSOR_TYPE_GYRO, power);
    }
    if (res != ICM426XX_SUCCESS) {
        GYRO_PR_ERR("icm426xx_gyro_SetPowerMode fail!\n");
        return -1;
    }
    atomic_set(&obj->is_enabled, en);
    GYRO_LOG("icm426xx_gyro_enable_nodata OK!\n");
    return 0;
}

static int icm426xx_gyro_set_delay(u64 ns)
{
    /* power mode setting in case of set_delay
    is called before enable_nodata */
    if(icm426xx_share_get_sensor_power(ICM426XX_SENSOR_TYPE_GYRO) == false)
        icm426xx_share_SetPowerMode(ICM426XX_SENSOR_TYPE_GYRO, true);
    icm426xx_share_SetSampleRate(ICM426XX_SENSOR_TYPE_GYRO, ns, false);

	icm426xx_gyro_discardcount = 80 * 1000 *1000 / (unsigned int)(ns);   //80 * icm206xx_accel_current_highest_samplerate / 1000;
	if (icm426xx_gyro_discardcount <=2)
    	icm426xx_gyro_discardcount = 2;

    GYRO_LOG("%s  discardcount num %d\n", __func__, icm426xx_gyro_discardcount);
    return 0;
}

static int icm426xx_gyro_get_data(int *x , int *y, int *z, int *status)
{
    char buff[ICM426XX_BUFSIZE];
    int res = 0;

    /* dps */
    icm426xx_gyro_ReadSensorData(obj_i2c_data->client, buff, ICM426XX_BUFSIZE);

	 if (res < 0) {
     	GYRO_PR_ERR("read gyro data not ready\n");
        return ICM426XX_ERR_BUS;
    }

    res = sscanf(buff, "%x %x %x", x, y, z);
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    return 0;
}

static const struct i2c_device_id icm426xx_gyro_i2c_id[] =
    {{ICM426XX_GYRO_DEV_NAME, 0}, {} };

#ifdef CONFIG_OF
static const struct of_device_id gyro_of_match[] = {
    {.compatible = "mediatek,icm426xx_gyro"},
    {},
};
#endif

static int icm426xx_gyro_i2c_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
    struct i2c_client *new_client;
    struct icm426xx_gyro_i2c_data *obj;
    struct gyro_control_path ctl = {0};
    struct gyro_data_path data = {0};
    int res = 0;

    obj = kzalloc(sizeof(*obj), GFP_KERNEL);
    if (!obj) {
        res = -ENOMEM;
        goto exit;
    }
	res = get_gyro_dts_func(client->dev.of_node, hw);
        if (res < 0) {
                GYRO_PR_ERR("get gyro dts info fail\n");
                goto exit;
        }
    memset(obj, 0, sizeof(struct icm426xx_gyro_i2c_data));
    /* hwmsen_get_convert() depend on the direction value */
	
	hw->direction = 5;
	
    obj->hw = hw;
    res = hwmsen_get_convert(obj->hw->direction, &obj->cvt);
    if (res) {
        GYRO_PR_ERR("invalid direction: %d\n", obj->hw->direction);
        goto exit;
    }
    GYRO_LOG("direction: %d\n", obj->hw->direction);
	if (0 != obj->hw->addr) {
	#if ICM_AD0_LOW
		client->addr =  0xD0 >> 1;
	#else 
	    client->addr =  0xD2 >> 1;
	#endif
        GYRO_LOG("gyro_use_i2c_addr: %x\n", client->addr);
    }
    obj_i2c_data = obj;
	client->addr = 0x68;
    obj->client = client;
    new_client = obj->client;
    i2c_set_clientdata(new_client, obj);
    atomic_set(&obj->trace, 0);
    atomic_set(&obj->suspend, 0);
    atomic_set(&obj->is_enabled, 0);
    icm426xx_gyro_i2c_client = new_client;
    res = icm426xx_gyro_init_client(new_client, false);
    if (res)
        goto exit_init_failed;
    /* misc_register() for factory mode, engineer mode and so on */
#if  MISC_DEVICE_FACTORY

    res = misc_register(&icm426xx_gyro_device);
    if (res) {
        GYRO_PR_ERR("icm426xx_gyro_device misc register failed!\n");
        goto exit_misc_device_register_failed;
    }
#else
	res = gyro_factory_device_register(&icm426xx_gyro_factory_device);

        if (res) {
                GYRO_LOG("icm426xx_gyro_factory_device register failed!\n");
                goto exit_misc_device_register_failed;
        }

#endif
    /* create platform_driver attribute */
    res = icm426xx_gyro_create_attr(
        &(icm426xx_gyro_init_info.platform_diver_addr->driver));
    if (res) {
        GYRO_PR_ERR("icm426xx_g create attribute err = %d\n", res);
        goto exit_create_attr_failed;
    }
    /* fill the gyro_control_path */
    ctl.is_use_common_factory = false;
    ctl.is_report_input_direct = false;
    ctl.is_support_batch = obj->hw->is_batch_supported;
    ctl.open_report_data = icm426xx_gyro_open_report_data;
    ctl.enable_nodata = icm426xx_gyro_enable_nodata;
    ctl.set_delay  = icm426xx_gyro_set_delay;
	ctl.batch = icm426xx_gyro_batch;
    ctl.flush = icm426xx_gyro_flush;
    /* register the gyro_control_path */
    res = gyro_register_control_path(&ctl);
    if (res) {
        GYRO_PR_ERR("register gyro control path err\n");
        goto exit_kfree;
    }
    /* fill the gyro_data_path */
    data.get_data = icm426xx_gyro_get_data;
    data.vender_div = DEGREE_TO_RAD;
    /* register the gyro_data_path */
    res = gyro_register_data_path(&data);
    if (res) {
        GYRO_PR_ERR("register gyro_data_path fail = %d\n", res);
        goto exit_kfree;
    }
    icm426xx_gyro_init_flag = 0;
    GYRO_LOG("%s: OK\n", __func__);
    return 0;

exit_create_attr_failed:
#if MISC_DEVICE_FACTORY
    misc_deregister(&icm426xx_gyro_device);
#else
	gyro_factory_device_deregister(&icm426xx_gyro_factory_device);
#endif 
exit_misc_device_register_failed:
exit_init_failed:
exit_kfree:
exit:
    kfree(obj);
    obj = NULL;
    icm426xx_gyro_init_flag =  -1;
    GYRO_PR_ERR("%s: err = %d\n", __func__, res);
    return res;
}

static int icm426xx_gyro_i2c_remove(struct i2c_client *client)
{
    int res = 0;

    res = icm426xx_gyro_delete_attr(
        &(icm426xx_gyro_init_info.platform_diver_addr->driver));
		
#if MISC_DEVICE_FACTORY
	misc_deregister(&icm426xx_gyro_device);
#else
	gyro_factory_device_deregister(&icm426xx_gyro_factory_device);
#endif 

    icm426xx_gyro_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));
    return 0;
}

static int icm426xx_gyro_i2c_detect(struct i2c_client *client,
    struct i2c_board_info *info)
{
    strcpy(info->type, ICM426XX_GYRO_DEV_NAME);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int icm426xx_gyro_i2c_suspend(struct device *dev)
{
    int res = 0;
	struct i2c_client *client = to_i2c_client(dev);	
    struct icm426xx_gyro_i2c_data *obj = i2c_get_clientdata(client);


        if (obj == NULL) {
            GYRO_PR_ERR("null pointer!!\n");
            return -EINVAL;
        }
        atomic_set(&obj->suspend, 1);
        res = icm426xx_share_SetPowerMode(ICM426XX_SENSOR_TYPE_GYRO, false);
        if (res < 0) {
            GYRO_PR_ERR("write power control fail!\n");
            return res;
        }
    
    icm426xx_gyro_power(obj->hw, 0);
    GYRO_LOG("icm426xx_gyro suspend ok\n");
    return res;
}

static int icm426xx_gyro_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
    struct icm426xx_gyro_i2c_data *obj = i2c_get_clientdata(client);
    int res = 0;

    if (obj == NULL) {
        GYRO_PR_ERR("null pointer!!\n");
        return -EINVAL;
    }
    icm426xx_gyro_power(obj->hw, 1);
    if(atomic_read(&obj->is_enabled) == 1) {
        res = icm426xx_share_SetPowerMode(ICM426XX_SENSOR_TYPE_GYRO, true);
    }
    if (res) {
        GYRO_PR_ERR("resume client fail!!\n");
        return res;
    }
    atomic_set(&obj->suspend, 0);
    GYRO_LOG("icm426xx_gyro resume ok\n");
    return 0;
}

static const struct dev_pm_ops icm426xx_gyro_pm_ops = {
        SET_SYSTEM_SLEEP_PM_OPS(icm426xx_gyro_i2c_suspend, icm426xx_gyro_i2c_resume)
};
#endif

static struct i2c_driver icm426xx_gyro_i2c_driver = {
    .driver = {
        .name = ICM426XX_GYRO_DEV_NAME,
#ifdef CONFIG_OF
        .of_match_table = gyro_of_match,
#endif
#ifdef CONFIG_PM_SLEEP
    	.pm = &icm426xx_gyro_pm_ops,
#endif 
    },
    .probe = icm426xx_gyro_i2c_probe,
    .remove = icm426xx_gyro_i2c_remove,
    .detect = icm426xx_gyro_i2c_detect,
   
    .id_table = icm426xx_gyro_i2c_id,
};

static int icm426xx_gyro_remove(void)
{
    icm426xx_gyro_power(hw, 0);
    i2c_del_driver(&icm426xx_gyro_i2c_driver);
    return 0;
}

static int icm426xx_gyro_local_init(struct platform_device *pdev)
{
    icm426xx_gyro_power(hw, 1);
    if (i2c_add_driver(&icm426xx_gyro_i2c_driver)) {
        GYRO_PR_ERR("add driver error\n");
        return -1;
    }
    if (-1 == icm426xx_gyro_init_flag)
        return -1;
    return 0;
}

static int __init icm426xx_gyro_init(void)
{
    GYRO_LOG("%s: OK\n", __func__);
    gyro_driver_add(&icm426xx_gyro_init_info);
    return 0;
}

static void __exit icm426xx_gyro_exit(void)
{
    /* nothing to do here */
}

module_init(icm426xx_gyro_init);
module_exit(icm426xx_gyro_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("icm426xx gyroscope driver");

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


#include "cust_acc.h"
#include "accel.h"
#include "icm426xx_register.h"
#include "icm426xx_share.h"
#include "icm426xx_share_interface.h"

struct icm426xx_sensor_status_info {
    bool sensor_power;
    int sample_rate;
    u8  sensor_power_mode;
};
static struct icm426xx_sensor_status_info 
    icm426xx_all_sensor_status_info[ICM426XX_SENSOR_TYPE_MAX];

static void icm426xx_set_sensor_power_mode(int paramSensor,
    u8 sensor_power_mode)
{
    icm426xx_all_sensor_status_info[paramSensor].sensor_power_mode
        = sensor_power_mode;
}

static u8 icm426xx_get_sensor_power_mode(int paramSensor)
{
    return icm426xx_all_sensor_status_info[paramSensor].sensor_power_mode;
}

static bool icm426xx_get_sensor_power(int paramSensor)
{
    return icm426xx_all_sensor_status_info[paramSensor].sensor_power;
}

static bool icm426xx_any_accel_based_sensor_is_on(void)
{
/* return true if any other step counter sensors are enabled except itself
    step counter sensors are 
       : ICM426XX_SENSOR_TYPE_SC,
       : ICM426XX_SENSOR_TYPE_SD,
       : ICM426XX_SENSOR_TYPE_SMD 
*/
    if (icm426xx_all_sensor_status_info
        [ICM426XX_SENSOR_TYPE_SC].sensor_power)
        return true;
    if (icm426xx_all_sensor_status_info
        [ICM426XX_SENSOR_TYPE_SD].sensor_power)
        return true;
    if (icm426xx_all_sensor_status_info
        [ICM426XX_SENSOR_TYPE_SMD].sensor_power)
        return true;
    return false;
}


static int icm426xx_i2c_driver_setting(void)
{
	int res = 0;
    u8 databuf[2] = {0};
	u8 i2c_slew_rate =1;
	u8 spi_slew_rate =1;

	databuf[0] = BIT_BANK_SEL_1;
    res = icm426xx_share_write_register(REG_REG_BANK_SEL, databuf, 1);
   	if (res < 0) {
   		ACC_PR_ERR("write bank4 select register err!\n");
		return ICM426XX_ERR_BUS;
    }
   	res = icm426xx_share_read_register(REG_INTF_CONFIG4, databuf, 1);
   	if (res) {
    	ACC_PR_ERR("Read REG_INTF_CONFIG4: %d\n", res);
    }
	//clear bit 6 of REG_INTF_CONFIG4 to support I2C and I3C
    databuf[0] &= ~BIT_I3C_BUS_MODE;
     	
	res = icm426xx_share_write_register(REG_INTF_CONFIG4, databuf, 1);
    if (res) {
      	ACC_PR_ERR("write REG_INTF_CONFIG4: %d\n", res);
    }
	ACC_LOG("write REG_INTF_CONFIG4 %x!\n", databuf[0]);
	
	res = icm426xx_share_read_register(REG_INTF_CONFIG6, databuf, 1);
   	if (res) {
    	ACC_PR_ERR("Read REG_INTF_CONFIG6: %d\n", res);
       	
    }
	//clear bit 0 ,1 of REG_INTF_CONFIG6, set bit4 to support I2C and I3C
    databuf[0] &= ~(BIT_I3C_SDR_EN | BIT_I3C_DDR_EN);
	databuf[0] |= BIT_I3C_EN;
     	
	res = icm426xx_share_write_register(REG_INTF_CONFIG6, databuf, 1);
    if (res) {
      	ACC_PR_ERR("write REG_INTF_CONFIG6: %d\n", res);
    }
	ACC_LOG("write REG_INTF_CONFIG6 %x!\n", databuf[0]);

    databuf[0] = BIT_BANK_SEL_0;
   	res = icm426xx_share_write_register(REG_REG_BANK_SEL, databuf, 1);
   	if (res < 0) {
     	ACC_PR_ERR("write bank select register err!\n");
    }
     
	res = icm426xx_share_read_register(REG_DRIVE_CONFIG, databuf, 1);
   	if (res) {
    	ACC_PR_ERR("Read REG_DRIVE_CONFIG: %d\n", res);
       	
    }
	//clear bit 5[:0 ]of REG_INTF_CONFIG4 
    databuf[0] &= 0xC0;
	databuf[0] |= i2c_slew_rate <<3 | spi_slew_rate;
     	
	res = icm426xx_share_write_register(REG_DRIVE_CONFIG, databuf, 1);
    if (res) {
      	ACC_PR_ERR("write REG_DRIVE_CONFIG: %d\n", res);
      
    }
	ACC_LOG("write REG_DRIVE_CONFIG %x!\n", databuf[0]);

	msleep(50);
	return ICM426XX_SUCCESS;
}

static int icm426xx_ChipSoftReset(void)
{
    u8 databuf[10];
    int res = 0;
    int i;

    memset(databuf, 0, sizeof(u8) * 10);
    /* read */
    res = icm426xx_share_read_register(REG_CHIP_CONFIG_REG, databuf, 0x1);
    if (res < 0) {
        ACC_PR_ERR("read power ctl register err!\n");
        return ICM426XX_ERR_BUS;
    }
    /* set device_reset bit to do soft reset */
    databuf[0] |= BIT_SOFT_RESET;
    res = icm426xx_share_write_register(REG_CHIP_CONFIG_REG, databuf, 0x1);
    if (res < 0) {
        ACC_PR_ERR("write power ctl register err!\n");
        return ICM426XX_ERR_BUS;
    }
    mdelay(100);

	res = icm426xx_i2c_driver_setting();

	 if (res < 0) {
        ACC_PR_ERR("write i2c driver config err!\n");
        return ICM426XX_ERR_BUS;
    }
    /* sensor status reset */
    for (i = 0; i < ICM426XX_SENSOR_TYPE_MAX; i++) {
        icm426xx_all_sensor_status_info[i].sensor_power = false;
        icm426xx_all_sensor_status_info[i].sample_rate = 0;
    }
    return ICM426XX_SUCCESS;
}

static int icm426xx_SetPowerMode(int sensor_type, bool enable)
{
    //u8 databuf[2] = {0};
    //int res = 0;
    //int i;

    if(sensor_type >= ICM426XX_SENSOR_TYPE_MAX)
        return ICM426XX_ERR_INVALID_PARAM;
    icm426xx_all_sensor_status_info[sensor_type].sensor_power = enable;
#if 0
    res = icm426xx_share_read_register(REG_TMST_CONFIG, databuf, 1);
    if (res < 0) {
        ACC_PR_ERR("read tmst register err!\n");
        return ICM426XX_ERR_BUS;
    }
    if (enable == false) {
        /* check power status of all sensors */
        for(i = 0; i < ICM426XX_SENSOR_TYPE_MAX; i++)
            if(icm426xx_all_sensor_status_info[i].sensor_power == true)
                break;
        /* turn off RC oscillator when all sensors are disabled */
        if(i == ICM426XX_SENSOR_TYPE_MAX) {
            databuf[0] = BIT_EN_DREG_FIFO_D2A;
        }
    } else {
        databuf[0] = BIT_EN_DREG_FIFO_D2A |
            BIT_TMST_TO_REGS_EN |
            BIT_TMST_EN;
    }
    res = icm426xx_share_write_register(REG_TMST_CONFIG, databuf, 1);
    if (res < 0) {
        ACC_PR_ERR("write tmst register err!\n");
        return ICM426XX_ERR_BUS;
    }
#endif
    ACC_LOG("set power mode ok %d!\n", enable);
    return ICM426XX_SUCCESS;
}

static int icm426xx_InterruptConfig_INT1(void)
{
    u8 databuf[2] = {0};
    int res = 0;

	res = icm426xx_share_read_register(REG_INT_CONFIG_REG, databuf, 0x1);
    if (res < 0) {
        ACC_PR_ERR("read REG_INT_CONFIG_REG register err!\n");
        return ICM426XX_ERR_BUS;
    }
	ACC_LOG("read INT_CONFIG %x!\n", databuf[0]);
	// config interrupt as active high, push pull , pulsed mode
	databuf[0] &= ~0x07;
    databuf[0] |= (INT_POLARITY << SHIFT_INT1_POLARITY) |
   	    (INT_DRIVE_CIRCUIT << SHIFT_INT1_DRIVE_CIRCUIT) |
        (INT_MODE << SHIFT_INT1_MODE);
	
	res = icm426xx_share_write_register(REG_INT_CONFIG_REG, databuf, 1);
	ACC_LOG("write INT_CONFIG %x!\n", databuf[0]);
    if (res < 0) {
        ACC_PR_ERR("write interrupt config failed!\n");
        //return ICM426XX_ERR_BUS;
    }
    return ICM426XX_SUCCESS;
}


static int icm426xx_EnableInterrupt(u8 int_type, bool enable)
{
    int res = 0;
    u8 databuf[2] = {0};
	u8 intsource[2] = {0};
	ACC_LOG("icm426xx_EnableInterrupt type %d en %d!\n", int_type,enable);
    if(ICM426XX_INT_TYPE_STD == int_type){
		//switch to banck4
		databuf[0] = BIT_BANK_SEL_4;
    	res = icm426xx_share_write_register(REG_REG_BANK_SEL, databuf, 1);
    	if (res < 0) {
      		ACC_PR_ERR("write bank4 select register err!\n");
        	return ICM426XX_ERR_BUS;
    	}
    	res = icm426xx_share_read_register(REG_INT_SOURCE6, intsource, 1);
    	if (res) {
      		ACC_PR_ERR("Read REG_INT_SOURCE6: %d\n", res);
        	return false;
    	}

		if(enable == true)
    	    intsource[0] |= BIT_STEP_DET_INT1_EN;
     	else{
       	  	intsource[0] &= ~BIT_STEP_DET_INT1_EN;
     	}
		res = icm426xx_share_write_register(REG_INT_SOURCE6, intsource, 1);
    	if (res) {
      		ACC_PR_ERR("write REG_INT_SOURCE6: %d\n", res);
        	return false;
    	}

    	databuf[0] = BIT_BANK_SEL_0;
    	res = icm426xx_share_write_register(REG_REG_BANK_SEL, databuf, 1);
    	if (res < 0) {
      	 	ACC_PR_ERR("write bank select register err!\n");
        	return ICM426XX_ERR_BUS;
    	}
		return res;
    }else if(ICM426XX_INT_TYPE_STC_OFL == int_type){
		//switch to banck4
		databuf[0] = BIT_BANK_SEL_4;
    	res = icm426xx_share_write_register(REG_REG_BANK_SEL, databuf, 1);
    	if (res < 0) {
      		ACC_PR_ERR("write bank4 select register err!\n");
        	return ICM426XX_ERR_BUS;
    	}
    	res = icm426xx_share_read_register(REG_INT_SOURCE6, intsource, 1);
    	if (res) {
      		ACC_PR_ERR("Read REG_INT_SOURCE6: %d\n", res);
        	return false;
    	}

		if(enable == true)
    	    intsource[0] |= BIT_STEP_CNT_OFL_INT1_EN;
     	else{
       	  	intsource[0] &= ~BIT_STEP_CNT_OFL_INT1_EN;
     	}
		res = icm426xx_share_write_register(REG_INT_SOURCE6, intsource, 1);
    	if (res) {
      		ACC_PR_ERR("write REG_INT_SOURCE6: %d\n", res);
        	return false;
    	}

    	databuf[0] = BIT_BANK_SEL_0;
    	res = icm426xx_share_write_register(REG_REG_BANK_SEL, databuf, 1);
    	if (res < 0) {
      	 	ACC_PR_ERR("write bank select register err!\n");
        	return ICM426XX_ERR_BUS;
    	}

		msleep(2);
		return res;

    }else{
    	res = icm426xx_share_read_register(REG_INT_SOURCE1, databuf, 1);
   	  if (res < 0) {
      	  ACC_PR_ERR("read interrupt register err!\n");
      	  return ICM426XX_ERR_BUS;
   		}
    	switch (int_type) {
/*
    case ICM426XX_INT_TYPE_UI_AGC_RDY:
        break;
    case ICM426XX_INT_TYPE_FIFO_FULL:
        break;
    case ICM426XX_INT_TYPE_FIFO_THS:
        break;
    case ICM426XX_INT_TYPE_UI_DRDY:
        break;
    case ICM426XX_INT_TYPE_RESET_DONE:
        break;
    case ICM426XX_INT_TYPE_UI_FSYNC:
        break;
*/
#if(!ICM426xx_WOM_SMD)
    	case ICM426XX_INT_TYPE_SMD:
      	  if(enable == true)
      	     databuf[0] |= BIT_INT_SMD_INT1_EN;
     	   else
       	     databuf[0] &= ~BIT_INT_SMD_INT1_EN;
      	  break;
#else
		case ICM426XX_INT_TYPE_SMD:
#endif
    	case ICM426XX_INT_TYPE_WOM:
      	  if(enable == true)
        	    databuf[0] |= BIT_INT_WOM_XYZ_INT1_EN;
        	else
         	   databuf[0] &= ~BIT_INT_WOM_XYZ_INT1_EN;
       	  break;
    	default:
        	ACC_PR_ERR("interrupt tyep %x is not supported", int_type);
       	 return -ICM426XX_ERR_INVALID_PARAM;
    	}
		ACC_LOG("write REG_INT_SOURCE1 %x \n", databuf[0]);
    	res = icm426xx_share_write_register(REG_INT_SOURCE1, databuf, 1);
    	if (res < 0) {
       	 ACC_PR_ERR("read interrupt register err!\n");
       	 return ICM426XX_ERR_BUS;
    	}
    	return ICM426XX_SUCCESS;
    }
}

static int icm426xx_EnableSensor(int sensor_type, bool enable , bool *first_enable_flag)
{
    u8 databuf[2] = {0};
    int res = 0;

    if(sensor_type >= ICM426XX_SENSOR_TYPE_MAX)
        return ICM426XX_ERR_INVALID_PARAM;
    
    res = icm426xx_share_read_register(REG_PWR_MGMT_0, databuf, 1);
    if (res < 0) {
        ACC_PR_ERR("read power mgmt register err!\n");
        return ICM426XX_ERR_BUS;
    }
    /* gyro based sensors */
    if (sensor_type == ICM426XX_SENSOR_TYPE_GYRO) {
        /* clear gyro enable */
        databuf[0] &= ~0x0C;
        if (enable == true) {
			if (false == icm426xx_all_sensor_status_info[ICM426XX_SENSOR_TYPE_GYRO].sensor_power)
				*first_enable_flag = true;
			else	
				*first_enable_flag = false;
			
			if (icm426xx_all_sensor_status_info
                [ICM426XX_SENSOR_TYPE_GYRO].sample_rate > ICM426XX_LPM_MAX_RATE)
                databuf[0] |= BIT_GYRO_MODE_LNM;
            else
                databuf[0] |=
                    icm426xx_get_sensor_power_mode(ICM426XX_SENSOR_TYPE_GYRO);
        }
    }
    /* accel based sensors */
    if (sensor_type == ICM426XX_SENSOR_TYPE_ACC ||
        sensor_type == ICM426XX_SENSOR_TYPE_SC ||
        sensor_type == ICM426XX_SENSOR_TYPE_SD ||
        sensor_type == ICM426XX_SENSOR_TYPE_SMD
    ) {
        /* clear accel enable */
        databuf[0] &= ~0x03;
        if (enable == true || icm426xx_any_accel_based_sensor_is_on()) {
			if ((false == icm426xx_all_sensor_status_info[ICM426XX_SENSOR_TYPE_ACC].sensor_power)&&  
				(false == icm426xx_any_accel_based_sensor_is_on()))
				*first_enable_flag = true;
			else
				*first_enable_flag = false;
			
            if (icm426xx_all_sensor_status_info
                [ICM426XX_SENSOR_TYPE_ACC].sample_rate > ICM426XX_LPM_MAX_RATE)
                databuf[0] |= BIT_ACCEL_MODE_LNM;
            else
                databuf[0] |=
                    icm426xx_get_sensor_power_mode(ICM426XX_SENSOR_TYPE_ACC);
        }
    }
    res = icm426xx_share_write_register(REG_PWR_MGMT_0, databuf, 1);
    if (res < 0) {
        ACC_PR_ERR("set power mgmt failed!\n");
        return ICM426XX_ERR_BUS;
    }
	
	icm426xx_all_sensor_status_info[sensor_type].sensor_power = enable;
    if(enable == true)
        mdelay(1);
    return ICM426XX_SUCCESS;
}

static int icm426xx_ReadChipInfo(char *buf, int bufsize)
{
    u8 databuf[2] = {0};
    int res = 0;

    if ((NULL == buf) || (bufsize <= 30))
        return -1;
    res = icm426xx_share_read_register(REG_WHO_AM_I, databuf, 1);
    if (res < 0) {
        ACC_PR_ERR("read who_am_i register err!\n");
        return ICM426XX_ERR_BUS;
    }
	pr_err("--------------%s------id = 0x%x\n",__func__,databuf[0]);
    switch (databuf[0]) {
    case WHO_AM_I_ICM426XX:
	case WHO_AM_I_ICM40607:
        sprintf(buf, "ICM426XX [0x%x]", databuf[0]);
        break;
    default:
        sprintf(buf, "Unknown Sensor [0x%x]", databuf[0]);
        break;
    }
    return ICM426XX_SUCCESS;
}

static int icm426xx_SetSampleRate(int sensor_type,
    unsigned int delay_ns, bool force_1khz)
{
    u8 databuf[2] = {0};
    unsigned int sample_rate = 0;
    int res = 0;
    int i, highest_sample_rate = 0;

    /* ns to us */
    sample_rate = (delay_ns / 1000);
    /* us to ms */
    sample_rate = (sample_rate / 1000);
    /* ms to hz */
    if(sample_rate != 0)
        sample_rate = (int)(1000 / sample_rate);
        ACC_LOG("sample_rate odr %d\n",sample_rate);
	/* sample rate: 5hz to 500hz;  UP to android O , we need support up to 500 hz  */
    /* when force_1khz is true, it means self test mode is running at 1khz */
    if ((sample_rate > 500) && (force_1khz == false))
        sample_rate = 500;
    else if (sample_rate < 25)
        sample_rate = 25;
    if(icm426xx_all_sensor_status_info[sensor_type].sample_rate == sample_rate)
        return ICM426XX_SUCCESS;
    icm426xx_all_sensor_status_info[sensor_type].sample_rate = sample_rate;
    if (sensor_type == ICM426XX_SENSOR_TYPE_GYRO) {
        res = icm426xx_share_read_register(REG_GYRO_CONFIG0, databuf, 1);
        if (res < 0) {
            ACC_PR_ERR("read odr register err!\n");
            return ICM426XX_ERR_BUS;
        }
        /*
        b'0011: 8kHz 
        b'0100: 4kHz 
        b'0101: 2kHz 
        b'0110: 1kHz 
        b'0111: 200Hz
        b'1000: 100Hz 
        b'1001: 50Hz 
        b'1010: 25Hz 
        */
        databuf[0] &= ~BIT_GYRO_ODR;
        if (sample_rate > 200)
            databuf[0] |= 6;
        else if (sample_rate > 100)
            databuf[0] |= 7;
        else if (sample_rate > 50)
            databuf[0] |= 8;
        else if (sample_rate > 25)
            databuf[0] |= 9;
        else
            databuf[0] |= 10;
        res = icm426xx_share_write_register(REG_GYRO_CONFIG0, databuf, 1);
        if (res < 0) {
            ACC_PR_ERR("write odr register err!\n");
            return ICM426XX_ERR_BUS;
        }
    }
    if (sensor_type == ICM426XX_SENSOR_TYPE_ACC ||
        sensor_type == ICM426XX_SENSOR_TYPE_SC ||
        sensor_type == ICM426XX_SENSOR_TYPE_SD ||
        sensor_type == ICM426XX_SENSOR_TYPE_SMD
    ) {
        /* check sample rate of enabled sensors */
        for(i = 0; i < ICM426XX_SENSOR_TYPE_MAX; i++) {
            if (i == ICM426XX_SENSOR_TYPE_GYRO)
                continue;
            if(icm426xx_all_sensor_status_info[i].sensor_power == true) {
				ACC_LOG("highest_sample_rate %d,type %d rate %d\n",highest_sample_rate,i,icm426xx_all_sensor_status_info[i].sample_rate);
                if(highest_sample_rate <
                    icm426xx_all_sensor_status_info[i].sample_rate)
                    highest_sample_rate =
                        icm426xx_all_sensor_status_info[i].sample_rate;
            }
        }
        res = icm426xx_share_read_register(REG_ACCEL_CONFIG0, databuf, 1);
        if (res < 0) {
            ACC_PR_ERR("read odr register err!\n");
            return ICM426XX_ERR_BUS;
        }
        /*
        b'0011: 8kHz 
        b'0100: 4kHz 
        b'0101: 2kHz 
        b'0110: 1kHz 
        b'0111: 200Hz
        b'1000: 100Hz 
        b'1001: 50Hz 
        b'1010: 25Hz 
        */
        ACC_LOG("highest_sample_rate odr %d,REG_ACCEL_CONFIG0 buf %x\n",highest_sample_rate,databuf[0]);
        databuf[0] &= ~BIT_ACCEL_ODR;
        if (highest_sample_rate > 200)
            databuf[0] |= 6;
        else if (highest_sample_rate > 100)
            databuf[0] |= 7;
        else if (highest_sample_rate > 50)
            databuf[0] |= 8;
        else if (highest_sample_rate > 25)
            databuf[0] |= 9;
        else
            databuf[0] |= 10;
		databuf[0] |= 0x40;   //force to set 4g here ,always use 4g here ?
        res = icm426xx_share_write_register(REG_ACCEL_CONFIG0, databuf, 1);
        if (res < 0) {
            ACC_PR_ERR("write odr register err!\n");
            return ICM426XX_ERR_BUS;
        }
		ACC_LOG("REG_ACCEL_CONFIG0 write buf %x\n",databuf[0]);
#ifdef DEBUG
		res = icm426xx_share_read_register(REG_ACCEL_CONFIG0, databuf, 1);
        if (res < 0) {
            ACC_PR_ERR("read odr register err!\n");
            return ICM426XX_ERR_BUS;
        }
	    ACC_LOG("REG_ACCEL_CONFIG0 buf %x\n",databuf[0]);
#endif
    }
    return ICM426XX_SUCCESS;
}

/* wrappers */

int icm426xx_share_InterruptConfig_INT1(void){
	return icm426xx_InterruptConfig_INT1();
}
EXPORT_SYMBOL(icm426xx_share_InterruptConfig_INT1);

void icm426xx_share_set_sensor_power_mode(int paramSensor, u8 paramPower)
{
    icm426xx_set_sensor_power_mode(paramSensor, paramPower);
}
EXPORT_SYMBOL(icm426xx_share_set_sensor_power_mode);

u8 icm426xx_share_get_sensor_power_mode(int paramSensor)
{
    return icm426xx_get_sensor_power_mode(paramSensor);
}
EXPORT_SYMBOL(icm426xx_share_get_sensor_power_mode);

bool icm426xx_share_get_sensor_power(int paramSensor)
{
    return icm426xx_get_sensor_power(paramSensor);
}
EXPORT_SYMBOL(icm426xx_share_get_sensor_power);

bool icm426xx_share_any_accel_based_sensor_is_on(void)
{
    return icm426xx_any_accel_based_sensor_is_on();
}
EXPORT_SYMBOL(icm426xx_share_any_accel_based_sensor_is_on);

int icm426xx_share_ChipSoftReset(void)
{
    return icm426xx_ChipSoftReset();
}
EXPORT_SYMBOL(icm426xx_share_ChipSoftReset);

int icm426xx_share_SetPowerMode(int sensor_type, bool enable)
{
    return icm426xx_SetPowerMode(sensor_type, enable);
}
EXPORT_SYMBOL(icm426xx_share_SetPowerMode);

int icm426xx_share_EnableInterrupt(u8 int_type, bool enable)
{
    return icm426xx_EnableInterrupt(int_type, enable);
}
EXPORT_SYMBOL(icm426xx_share_EnableInterrupt);

int icm426xx_share_EnableSensor(int sensor_type, bool enable,bool *flag)
{
    return icm426xx_EnableSensor(sensor_type, enable , flag);
}
EXPORT_SYMBOL(icm426xx_share_EnableSensor);

int icm426xx_share_ReadChipInfo(char *buf, int bufsize)
{
    return icm426xx_ReadChipInfo(buf, bufsize);
}
EXPORT_SYMBOL(icm426xx_share_ReadChipInfo);

int icm426xx_share_SetSampleRate(int sensor_type,
    u64 delay_ns, bool force_1khz)
{
    return icm426xx_SetSampleRate(sensor_type, delay_ns, force_1khz);
}
EXPORT_SYMBOL(icm426xx_share_SetSampleRate);

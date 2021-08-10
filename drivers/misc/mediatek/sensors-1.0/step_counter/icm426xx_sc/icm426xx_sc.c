/* 
 * ICM206XX sensor driver
 * Copyright (C) 2016 Invensense, Inc.
 *STEP_C_PR_ERR
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
#include "../step_counter.h"
#include "../../accelerometer/icm426xx_a/icm426xx_register.h"
#include "../../accelerometer/icm426xx_a/icm426xx_share.h"
#include "icm426xx_sc.h"
#include <linux/miscdevice.h>

#ifdef ICM426XX_DMP_STEP_COUNTER_TUNE
static atomic_t icm426xx_sc_trace;
#endif

static int icm426xx_sc_init_flag =  -1;

static int icm426xx_sc_local_init(void);
static int icm426xx_sc_remove(void);

static int step_count_overflowvalue = 0;
static bool stepcount_need_reset = true;
static uint32_t previous_step = 0;
static struct step_c_init_info icm426xx_sc_init_info = {
	.name = "ICM426XX_SC",
	.init = icm426xx_sc_local_init,
	.uninit = icm426xx_sc_remove,
};

/*=======================================================================================*/
/* I2C Primitive Functions Section					 		 */
/*=======================================================================================*/

/* Share i2c function with Accelerometer 		*/
/* Function is defined in accelerometer/icm426xx_a 	*/

/*=======================================================================================*/
/* Vendor Specific Functions Section					  		 */
/*=======================================================================================*/

/*  adjust parameter in config to adjust the pedometer performance  */
static int icm426xx_sc_parameter_config(void)
{
	int res = 0;
	u8 databuf[2] = {0};

	int low_energy_amp_th = 10;
	//int pedo_amp_th_sel   = 8;
	int pedo_step_cnt_th_sel =5; 
	int pedo_sb_timer_th_sel = 4;
	int pedo_step_det_th_sel = 2;
	
	//switch to banck4
	databuf[0] = BIT_BANK_SEL_4;
    res = icm426xx_share_write_register(REG_REG_BANK_SEL, databuf, 1);
    if (res < 0) {
      	STEP_C_PR_ERR("write bank4 select register err!\n");
       	return ICM426XX_ERR_BUS;
    }
	// config CONFIG1
	res = icm426xx_share_read_register(REG_APEX_CONFIG1, databuf, 1);
   	if (res) {
      		STEP_C_PR_ERR("Read REG_APEX_CONFIG1: %d\n", res);
        	return false;
    	}

	STEP_C_LOG("read  REG_APEX_CONFIG1 %x\n", databuf[0]);

	databuf[0] = databuf[0] & 0x0F;
	databuf[0] |= low_energy_amp_th <<4;
	res = icm426xx_share_write_register(REG_APEX_CONFIG1, databuf, 1);
    	if (res) {
      		STEP_C_PR_ERR("write REG_APEX_CONFIG1: %d\n", res);
        	return false;
    	}	
	STEP_C_LOG("set  REG_APEX_CONFIG1 %x\n", databuf[0]);

	// config CONFIG2	
	res = icm426xx_share_read_register(REG_APEX_CONFIG2, databuf, 1);
   	if (res) {
      		STEP_C_PR_ERR("Read REG_APEX_CONFIG2: %d\n", res);
        	return false;
    	}
	
	STEP_C_LOG("read  REG_APEX_CONFIG2 %x\n", databuf[0]);
	// Should not set [7:4] bit 
	databuf[0] = databuf[0] & 0xF0;
	databuf[0] |= pedo_step_cnt_th_sel;
	res = icm426xx_share_write_register(REG_APEX_CONFIG2, databuf, 1);
    	if (res) {
      		STEP_C_PR_ERR("write REG_APEX_CONFIG2: %d\n", res);
        	return false;
    	}	
	STEP_C_LOG("set  REG_APEX_CONFIG2 %x\n", databuf[0]);

	// config CONFIG3	
	res = icm426xx_share_read_register(REG_APEX_CONFIG3, databuf, 1);
   	if (res) {
      		STEP_C_PR_ERR("Read REG_APEX_CONFIG2: %d\n", res);
        	return false;
    	}

	STEP_C_LOG("read  REG_APEX_CONFIG3 %x\n", databuf[0]);
	// Should not set [1:0] bit 
	databuf[0] = databuf[0] & 0x03;
	databuf[0] |= (pedo_step_det_th_sel << 5 ) | (pedo_sb_timer_th_sel <<2);
	res = icm426xx_share_write_register(REG_APEX_CONFIG3, databuf, 1);
    	if (res) {
      		STEP_C_PR_ERR("write REG_APEX_CONFIG3: %d\n", res);
        	return false;
    	}	
	STEP_C_LOG("set  REG_APEX_CONFIG3 %x\n", databuf[0]);

	// config CONFIG9 set sensitivity mode in bit 0 to 0 
	res = icm426xx_share_read_register(REG_APEX_CONFIG9, databuf, 1);
   	if (res) {
      		STEP_C_PR_ERR("Read REG_APEX_CONFIG9: %d\n", res);
        	return false;
    	}

	STEP_C_LOG("read  REG_APEX_CONFIG9 %x\n", databuf[0]);
	// Should set bit0 to 0 
	databuf[0] = databuf[0] & (~0x01);
	res = icm426xx_share_write_register(REG_APEX_CONFIG9, databuf, 1);
    	if (res) {
      		STEP_C_PR_ERR("write REG_APEX_CONFIG3: %d\n", res);
        	return false;
    	}	
	STEP_C_LOG("set  REG_APEX_CONFIG9 %x\n", databuf[0]);

	//switch back to BANK 0
	databuf[0] = BIT_BANK_SEL_0;
    res = icm426xx_share_write_register(REG_REG_BANK_SEL, databuf, 1);
   	if (res < 0) {
   	 	STEP_C_PR_ERR("write bank select register err!\n");
		return ICM426XX_ERR_BUS;
    }
	return res;
}

/*  DMP Power save mode */
static int icm426xx_sc_DMP_powersave(bool powersave)
{
	int res = 0;
	u8 databuf[2] = {0};

	res = icm426xx_share_read_register(REG_APEX_CONFIG0, databuf, 1);
   	if (res) {
      		STEP_C_PR_ERR("Read REG_APEX_CONFIG0: %d\n", res);
        	return false;
    	}

	if(true == powersave)
		databuf[0] |= ICM426XX_APEX_CONFIG0_DMP_POWER_SAVE_EN;
	else
		databuf[0] &= ~BIT_APEX_CONFIG0_DMP_POWER_SAVE_MASK;
		
	res = icm426xx_share_write_register(REG_APEX_CONFIG0, databuf, 1);
    	if (res) {
      		STEP_C_PR_ERR("write REG_APEX_CONFIG0: %d\n", res);
        	return false;
    	}	

	return res;
}



/*  Reset DMP_MEM that is reset all algorithm in DMP  */
static int icm426xx_sc_DMP_mem_reset(void)
{
	int res = 0;
	u8 databuf[2] = {0};
	int timeout_ms =5;
	
	res = icm426xx_share_read_register(REG_SIGNAL_PATH_RESET, databuf, 1);
   	if (res) {
      		STEP_C_PR_ERR("Read REG_APEX_CONFIG0: %d\n", res);
        	return false;
    	}

	databuf[0] = DMP_MEM_RESET_EN;

	res = icm426xx_share_write_register(REG_SIGNAL_PATH_RESET, databuf, 1);
    	if (res) {
      		STEP_C_PR_ERR("write REG_SIGNAL_PATH_RESET: %d\n", res);
        	return false;
    	}	
	msleep(1);

	do { 
		res = icm426xx_share_read_register(REG_SIGNAL_PATH_RESET, databuf, 1);
		msleep(1);
	} while ((databuf[0] != 0x00) && timeout_ms--);

	if(timeout_ms<=0)
		STEP_C_PR_ERR("DMP reset status error\n");
        return false;
		
	return res;
}


static int icm426xx_resume_dmp(void)
{
	int res = 0;
	u8 databuf[2] = {0};

	//const int ref_timeout = 5000; /*50 ms*/
	//int timeout = ref_timeout;

	//enable dmp_init_en 
	res = icm426xx_share_read_register(REG_SIGNAL_PATH_RESET, databuf, 1);
   	if (res) {
   		STEP_C_PR_ERR("Read REG_SIGNAL_PATH_RESET: %d\n", res);
       	return false;
   	}

	databuf[0] |= DMP_INIT_EN;

	res = icm426xx_share_write_register(REG_SIGNAL_PATH_RESET, databuf, 1);
   	if (res) {
    	STEP_C_PR_ERR("write REG_SIGNAL_PATH_RESET: %d\n", res);
       	return false;
    	}
	STEP_C_LOG("write DMP_INIT_EN %x\n",databuf[0]);

	#if 0
	// Wait for DMP busy  
	do { 
		result |= inv_icm426xx_rd_apex_data3_dmp_idle(s, &data);
		inv_icm426xx_sleep_us(10);
	} while ((data != ICM426XX_APEX_DATA3_DMP_IDLE_ON) && timeout--);
	
	if (timeout <= 0)
		return INV_ERROR_TIMEOUT;
	
	timeout = ref_timeout;
	
	// Wait for DMP idle
	do { 
		result |= inv_icm426xx_rd_apex_data3_dmp_idle(s, &data);
		inv_icm426xx_sleep_us(10);
	} while ((data != ICM426XX_APEX_DATA3_DMP_IDLE_OFF) && timeout--);
	
	if (timeout <= 0)
		return INV_ERROR_TIMEOUT;
	#endif
	
	msleep(100);
	return res;
}



/*  set DMP_ODR  to 50hz  */
static int icm426xx_sc_initDMP(void)
{
	int res = 0;
	u8 databuf[2] = {0};

	res = icm426xx_share_read_register(REG_APEX_CONFIG0, databuf, 1);
   	if (res) {
      		STEP_C_PR_ERR("Read REG_APEX_CONFIG0: %d\n", res);
        	return false;
    	}
	databuf[0] &= ~BIT_APEX_CONFIG0_DMP_ODR_MASK;
	databuf[0] |= DMP_ODR_50HZ;

	res = icm426xx_share_write_register(REG_APEX_CONFIG0, databuf, 1);
    	if (res) {
      		STEP_C_PR_ERR("write REG_APEX_CONFIG0: %d\n", res);
        	return false;
    	}	
	msleep(1);
	if(true == stepcount_need_reset)
	{ 
		icm426xx_sc_DMP_mem_reset();
		stepcount_need_reset = false;
		}
	icm426xx_resume_dmp();
	return res;	
}

/*static int icm426xx_sc_stopDMP(void)
{
	int res = 0;
	u8 databuf[2] = {0};

	databuf[0] = 0x00;
	res = icm426xx_share_write_register(ICM426XX_REG_USER_CTL, databuf, 1);
	if (res < 0) {
		STEP_C_PR_ERR("stop DMP by write USER_CTL register err!\n");
		return ICM426XX_ERR_BUS;
	}

	return res;
}*/

static int icm426xx_sc_sd_EnablePedometer(u8 en)
{
	int res = 0;
	u8 databuf[2] = {0};

	if(en){

		icm426xx_sc_initDMP();
		
		//enable pedo_en 
		res = icm426xx_share_read_register(REG_APEX_CONFIG0, databuf, 1);
   		if (res) {
      		STEP_C_PR_ERR("Read REG_APEX_CONFIG0: %d\n", res);
       		return false;
    	}

		databuf[0] |= PED_ENABLE;
		
		res = icm426xx_share_write_register(REG_APEX_CONFIG0, databuf, 1);
    	if (res) {
      		STEP_C_PR_ERR("write PED_ENABLE: %d\n", res);
        	return false;
    		}	
	}
	else{
			//disable pedo_en 
		res = icm426xx_share_read_register(REG_APEX_CONFIG0, databuf, 1);
   		if (res) {
      		STEP_C_PR_ERR("Read REG_APEX_CONFIG0: %d\n", res);
        	return false;
    		}
				
			databuf[0] &= (~PED_ENABLE);   //clear PED_ENABLE

			res = icm426xx_share_write_register(REG_APEX_CONFIG0, databuf, 1);
    		if (res) {
      			STEP_C_PR_ERR("write PED_ENABLE: %d\n", res);
        		return false;
    		}		
		 }
	return res;
}

static int icm426xx_sc_ThresholdWOM(void)
{
    u8 databuf[2] = {0};
    int res = 0;

	//switch to banck4
	databuf[0] = BIT_BANK_SEL_4;
    res = icm426xx_share_write_register(REG_REG_BANK_SEL, databuf, 1);
    if (res < 0) {
      	STEP_C_PR_ERR("write bank4 select register err!\n");
       	return ICM426XX_ERR_BUS;
    }
	
    databuf[0] = 30;  //98
    res = icm426xx_share_write_register(REG_ACCEL_WOM_X_THR, databuf, 1);
    if (res < 0) {
        STEP_C_PR_ERR("write wom config failed!\n");
        return ICM426XX_ERR_BUS;
    }
    res = icm426xx_share_write_register(REG_ACCEL_WOM_Y_THR, databuf, 1);
    if (res < 0) {
        STEP_C_PR_ERR("write wom config failed!\n");
        return ICM426XX_ERR_BUS;
    }
    res = icm426xx_share_write_register(REG_ACCEL_WOM_Z_THR, databuf, 1);
    if (res < 0) {
        STEP_C_PR_ERR("write wom config failed!\n");
        return ICM426XX_ERR_BUS;
    }

		//switch back to BANK 0
	databuf[0] = BIT_BANK_SEL_0;
    res = icm426xx_share_write_register(REG_REG_BANK_SEL, databuf, 1);
   	if (res < 0) {
   	 	STEP_C_PR_ERR("write bank select register err!\n");
		return ICM426XX_ERR_BUS;
    }
    return ICM426XX_SUCCESS;
}

static int icm426xx_sc_EnableSMD(u8 en)
{
    u8 databuf[2] = {0};
    int res = 0;
	int wom_int_mode = 0;
	int wom_mode     = 1;

	if(en){
		/*  set WOM threshold  */
		res = icm426xx_sc_ThresholdWOM();
 	 	if (res < 0) {
   	     STEP_C_PR_ERR("icm4206xx_sc_ThresholdWOM failed!\n");
   	     return ICM426XX_ERR_BUS;
  	    }
		msleep(1);
	}

    res = icm426xx_share_read_register(REG_SMD_CONFIG, databuf, 1);
    if (res < 0) {
        STEP_C_PR_ERR("read smd config failed!\n");
        return ICM426XX_ERR_BUS;
    }
    if (en) {
	#if ICM426xx_WOM_SMD
        databuf[0] |= (wom_int_mode << 3)|(wom_mode << 2)|(BIT_SMD_MODE_OLD);
	#else
		databuf[0] |= (wom_int_mode << 3)|(wom_mode << 2)|(BIT_SMD_MODE_LONG);
    #endif
	
    } else {
        databuf[0] &= ~(BIT_SMD_MODE_MASK);
    }

    res = icm426xx_share_write_register(REG_SMD_CONFIG, databuf, 1);
    if (res < 0) {
        STEP_C_PR_ERR("write smd config failed!\n");
        return ICM426XX_ERR_BUS;
    }

	msleep(1);
    return res;
}


static int icm426xx_sc_ReadSensorData(uint32_t *value)
{
	int res = 0;
	u8 databuf[4] = {0};
	u8 walk_status;

	res = icm426xx_share_read_register(REG_APEX_DATA0, databuf, 3);
	if (res < 0) {
		STEP_C_PR_ERR("read Step Counter err!\n");
		return ICM426XX_ERR_BUS;
	}

	*value = databuf[1] << 8 | databuf[0]  ;
	walk_status = databuf[2];
	STEP_C_LOG("Step Counter Data - %04x\n , satatus %x", (*value),walk_status);

	return res;
}

void icm426xx_sc_NotifySensorData(void)
{
	u8 databuf[2] = {0};
	u8 int_status3;
	u8 int_status2;
	int res;

	res = icm426xx_share_read_register(REG_INT_STATUS2, databuf, 1);
	int_status2 = databuf[0];
	res = icm426xx_share_read_register(REG_INT_STATUS3, databuf, 1);
	int_status3 = databuf[0];
	if(int_status3 & BIT_STEP_DET_INT)
	{
		step_notify(TYPE_STEP_DETECTOR);
		STEP_C_LOG("Step Detector Nofity - %x\n", int_status3);	
	}

	if(int_status3 & BIT_STEP_CNT_OVF_INT)
	{
	 	if(previous_step >= 32768)
			step_count_overflowvalue += 65536;              //prevent from false trigger to decide already count step value 
		STEP_C_LOG("Step Detector overflow- %x\n", int_status3);
	}

	//if(int_status2 & BIT_INT_STATUS_SMD)
    STEP_C_LOG("int_status2 %x, int_status3 %x\n", int_status2,int_status3);
#if ICM426xx_WOM_SMD		
	if(int_status2 & BIT_INT_STATUS_WOM_XYZ)
#else
	if(int_status2 & BIT_INT_STATUS_SMD)
#endif
	{
		step_notify(TYPE_SIGNIFICANT);
		STEP_C_LOG("Significant Motion Nofity - %d\n", int_status2);
	}
}


/*----------------------------------------------------------------------------*/

static int icm426xx_sc_init_client(bool enable)
{
	int res = 0;

	res = icm426xx_share_EnableInterrupt(ICM426XX_INT_TYPE_STC_OFL,true);	
	if(res < 0){
		STEP_C_PR_ERR("config interrupt err in icm426xx_sc_init_client !\n");
		return ICM426XX_ERR_BUS;
	}
	
	res = icm426xx_share_InterruptConfig_INT1();

	icm426xx_sc_parameter_config();
	icm426xx_sc_DMP_powersave(false);
	
	if(res < 0){
		STEP_C_PR_ERR("config interrupt1 mode err icm426xx_sc_init_client !\n");
		return ICM426XX_ERR_BUS;
	}

	STEP_C_LOG("icm426xx_sc_init_client OK!\n");
	return ICM426XX_SUCCESS;
}

/*=======================================================================================*/
/* Debug Only Functions Section					 		 	 */
/*=======================================================================================*/

#ifdef ICM426XX_DMP_DOWNLOAD_VERIFY
static char gstr_ReadDump[1024];

static void debug_ReadDump(void)
{
	int i;
	for (i = 0; i < 14; i++)
	{
		sprintf(&gstr_ReadDump[i * 48], "%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x\n",
				dmpVerify[i*16+0],
				dmpVerify[i*16+1],
				dmpVerify[i*16+2],
				dmpVerify[i*16+3],
				dmpVerify[i*16+4],
				dmpVerify[i*16+5],
				dmpVerify[i*16+6],
				dmpVerify[i*16+7],
				dmpVerify[i*16+8],
				dmpVerify[i*16+9],
				dmpVerify[i*16+10],
				dmpVerify[i*16+11],
				dmpVerify[i*16+12],
				dmpVerify[i*16+13],
				dmpVerify[i*16+14],
				dmpVerify[i*16+15]);
	}
}
#endif

/*=======================================================================================*/
/* Driver Attribute Section					 			 */
/*=======================================================================================*/
#ifdef ICM426XX_DMP_STEP_COUNTER_TUNE
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res = 0;

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&icm426xx_sc_trace));

	return res;
}

static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int trace;

	if (!kstrtoint(buf, 16, &trace))
		atomic_set(&icm426xx_sc_trace, trace);
	else
		STEP_C_PR_ERR("invalid content: '%s', length = %d\n", buf, (int)count);

	return count;
}

static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t res = 0;

#ifdef ICM426XX_DMP_DOWNLOAD_VERIFY
	debug_ReadDump();
	res = snprintf(buf, PAGE_SIZE, "%s\n", gstr_ReadDump);
#endif
	return res;
}


static ssize_t show_sb_threshold(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;

	int res = 0;
	u16 d = 0;
	uint16_t value;

	icm426xx_share_SetPowerMode(ICM426XX_SENSOR_TYPE_STEP_COUNTER, true);

	res = icm426xx_share_read_memory(D_PEDSTD_SB, 2, (u8 *)(&d));
	if (res < 0) {
		STEP_C_PR_ERR("read Step Buffer err!\n");
		return 0;
	}

	value = be16_to_cpup((__be16 *)(&d));
	len = snprintf(buf, PAGE_SIZE, "0x%02X\n", value);

	return len;
}

static ssize_t store_sb_threshold(struct device_driver *ddri, const char *buf, size_t count)
{
	int value;
	
	int res = 0;
	u8 databuf[3] = {0};

	icm426xx_share_SetPowerMode(ICM426XX_SENSOR_TYPE_STEP_COUNTER, true);

	if (kstrtoint(buf, 16, &value))
	{
		STEP_C_PR_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
		return count;
	}

	databuf[0] = (value >> 8);
	databuf[1] = (value & 0xFF);

	res = icm426xx_share_write_memory(D_PEDSTD_SB, 2, &databuf[0]);
	if (res < 0) {
		STEP_C_PR_ERR("write Step Buffer err!\n");
		return 0;
	}

	return count;
}

static ssize_t show_sbtime_threshold(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;

	int res = 0;
	u16 d = 0;
	uint16_t value;

	icm426xx_share_SetPowerMode(ICM426XX_SENSOR_TYPE_STEP_COUNTER, true);

	res = icm426xx_share_read_memory(D_PEDSTD_SB_TIME, 2, (u8 *)(&d));
	if (res < 0) {
		STEP_C_PR_ERR("read Step Buffer Time err!\n");
		return 0;
	}

	value = be16_to_cpup((__be16 *)(&d));
	len = snprintf(buf, PAGE_SIZE, "0x%02X\n", value);

	return len;
}

static ssize_t store_sbtime_threshold(struct device_driver *ddri, const char *buf, size_t count)
{
	int value;
	
	int res = 0;
	u8 databuf[3] = {0};

	icm426xx_share_SetPowerMode(ICM426XX_SENSOR_TYPE_STEP_COUNTER, true);

	if (kstrtoint(buf, 16, &value))
	{
		STEP_C_PR_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
		return count;
	}

	databuf[0] = (value >> 8);
	databuf[1] = (value & 0xFF);

	res = icm426xx_share_write_memory(D_PEDSTD_SB_TIME, 2, &databuf[0]);
	if (res < 0) {
		STEP_C_PR_ERR("write Step Buffer Time err!\n");
		return 0;
	}

	return count;
}

static ssize_t show_peak_threshold(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;

	int res = 0;
	u32 d = 0;
	uint32_t value;

	icm426xx_share_SetPowerMode(ICM426XX_SENSOR_TYPE_STEP_COUNTER, true);

	res = icm426xx_share_read_memory(D_PEDSTD_PEAKTHRSH, 4, (u8 *)(&d));
	if (res < 0) {
		STEP_C_PR_ERR("read Peak threshold err!\n");
		return 0;
	}

	value = be32_to_cpup((__be32 *)(&d));
	len = snprintf(buf, PAGE_SIZE, "0x%08X\n", value);

	return len;
}

static ssize_t store_peak_threshold(struct device_driver *ddri, const char *buf, size_t count)
{
	int value;
	
	int res = 0;
	u8 databuf[5] = {0};

	icm426xx_share_SetPowerMode(ICM426XX_SENSOR_TYPE_STEP_COUNTER, true);

	if (kstrtoint(buf, 16, &value))
	{
		STEP_C_PR_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
		return count;
	}

	databuf[0] = (value >> 24);
	databuf[1] = (value >> 16);
	databuf[2] = (value >> 8);
	databuf[3] = (value & 0xFF);

	res = icm426xx_share_write_memory(D_PEDSTD_PEAKTHRSH, 4, &databuf[0]);
	if (res < 0) {
		STEP_C_PR_ERR("write Peak threshold err!\n");
		return 0;
	}

	return count;
}
#endif

/*----------------------------------------------------------------------------*/
#ifdef ICM426XX_DMP_STEP_COUNTER_TUNE
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, show_trace_value,   store_trace_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value,   NULL);

static DRIVER_ATTR(sbth, S_IWUSR | S_IRUGO, show_sb_threshold, store_sb_threshold);
static DRIVER_ATTR(sbtimeth, S_IWUSR | S_IRUGO, show_sbtime_threshold, store_sbtime_threshold);
static DRIVER_ATTR(peakth, S_IWUSR | S_IRUGO, show_peak_threshold, store_peak_threshold);


static struct driver_attribute *icm426xx_sc_attr_list[] = {
	&driver_attr_trace,		/*trace log*/
	&driver_attr_status,

	&driver_attr_sbth,
	&driver_attr_sbtimeth,
	&driver_attr_peakth,

};
/*----------------------------------------------------------------------------*/
static int icm426xx_sc_create_attr(struct device_driver *driver)
{
	int idx;
	int res = 0;
	int num = (int)(sizeof(icm426xx_sc_attr_list)/sizeof(icm426xx_sc_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		res = driver_create_file(driver, icm426xx_sc_attr_list[idx]);
		if (0 != res) {
			STEP_C_PR_ERR("driver_create_file (%s) = %d\n", icm426xx_sc_attr_list[idx]->attr.name, res);
			break;
		}
	}
	return res;
}

static int icm426xx_sc_delete_attr(struct device_driver *driver)
{
	int idx;
	int res = 0;
	int num = (int)(sizeof(icm426xx_sc_attr_list)/sizeof(icm426xx_sc_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, icm426xx_sc_attr_list[idx]);

	return res;
}
#endif
/*=======================================================================================*/
/* Misc - Factory Mode (IOCTL) Device Driver Section					 */
/*=======================================================================================*/

static struct miscdevice icm426xx_sc_device = {
	.minor 		= MISC_DYNAMIC_MINOR,
	.name 		= "inv_step_counter",
};

/*=======================================================================================*/
/* Misc - I2C HAL Support Section				 			 */
/*=======================================================================================*/

/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int icm426xx_sc_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */
static int icm426xx_sc_enable_nodata(int en)
{
    bool flag; 

	STEP_C_LOG("icm426xx_sc_enable_nodata start!\n");
	if(1 == en)
	{
	
		if ((false == icm426xx_share_get_sensor_power(ICM426XX_SENSOR_TYPE_SD)) || 
			(false == icm426xx_share_get_sensor_power(ICM426XX_SENSOR_TYPE_SC)))
    	{
			icm426xx_share_EnableSensor(ICM426XX_SENSOR_TYPE_SC, true ,&flag);
			
			icm426xx_share_SetPowerMode(ICM426XX_SENSOR_TYPE_SC, true);
			icm426xx_share_SetSampleRate(ICM426XX_SENSOR_TYPE_SC, 10000000, false);           // always set pedometer run @ 100hz
			icm426xx_sc_sd_EnablePedometer(en);
		}
	}
	else{
		icm426xx_share_SetSampleRate(ICM426XX_SENSOR_TYPE_SC, 0, false);
		icm426xx_share_EnableSensor(ICM426XX_SENSOR_TYPE_SC, false,&flag);
		icm426xx_share_SetPowerMode(ICM426XX_SENSOR_TYPE_SC, false);
		if(false == icm426xx_share_get_sensor_power(ICM426XX_SENSOR_TYPE_SD))				// if both sc and sd are off turn off pedometer		
		{
			icm426xx_sc_sd_EnablePedometer(en);
		}
	}

	STEP_C_LOG("icm426xx_sc_enable_step_detect OK!\n");

	return 0;
}

static int icm426xx_sc_enable_step_detect(int en)
{
	int res;
    bool flag; 
	if(1 == en)
	{	

		if ((false == icm426xx_share_get_sensor_power(ICM426XX_SENSOR_TYPE_SD)) || 
			(false == icm426xx_share_get_sensor_power(ICM426XX_SENSOR_TYPE_SC)))
    	{
			icm426xx_share_EnableSensor(ICM426XX_SENSOR_TYPE_SD, true ,&flag);
			
			icm426xx_share_SetPowerMode(ICM426XX_SENSOR_TYPE_SD, true);
			icm426xx_share_SetSampleRate(ICM426XX_SENSOR_TYPE_SD, 10000000, false);           // always set pedometer run @ 100hz
			icm426xx_sc_sd_EnablePedometer(en);
		}
		// enable interrput function of sd and sofl
	
			res = icm426xx_share_EnableInterrupt(ICM426XX_INT_TYPE_STD,true);						// enable SD interrupt
	}
	else{
		icm426xx_share_SetSampleRate(ICM426XX_SENSOR_TYPE_SD, 0, false);
		icm426xx_share_SetPowerMode(ICM426XX_SENSOR_TYPE_SD, false);
		icm426xx_share_EnableSensor(ICM426XX_SENSOR_TYPE_SD, false,&flag);
		if(false == icm426xx_share_get_sensor_power(ICM426XX_SENSOR_TYPE_SC))				// if both sc and sd are off turn off pedometer		
		{
			icm426xx_sc_sd_EnablePedometer(en);
			res = icm426xx_share_EnableInterrupt(ICM426XX_INT_TYPE_STD,false);	
		}
	}

	STEP_C_LOG("icm426xx_sc_enable_step_detect OK!\n");

	return 0;
}

static int icm426xx_sc_enable_significant(int en)
{
	bool flag;
	
	if(1 == en)
	{
	 	if(false == icm426xx_share_get_sensor_power(ICM426XX_SENSOR_TYPE_SD)){
			icm426xx_share_EnableSensor(ICM426XX_SENSOR_TYPE_SMD, true ,&flag);	
			icm426xx_share_SetPowerMode(ICM426XX_SENSOR_TYPE_SMD, true);
			icm426xx_share_SetSampleRate(ICM426XX_SENSOR_TYPE_SMD, 20000000, false);           // always set SMD run @ 50 hz
			msleep(1);
			icm426xx_sc_EnableSMD(en);
			icm426xx_share_EnableInterrupt(ICM426XX_INT_TYPE_SMD, en);
			msleep(50);
	 	}
		
	}
	else
	{
		icm426xx_share_SetSampleRate(ICM426XX_SENSOR_TYPE_SD, 0, false);
		icm426xx_share_SetPowerMode(ICM426XX_SENSOR_TYPE_SD, false);
		icm426xx_share_EnableSensor(ICM426XX_SENSOR_TYPE_SMD, false,&flag);
		msleep(1);
		icm426xx_sc_EnableSMD(en);
		icm426xx_share_EnableInterrupt(ICM426XX_INT_TYPE_SMD, en);
		msleep(50);	
	}
	
	STEP_C_LOG("icm426xx_sc_enable_significant OK!\n");

	return 0;
}

static int icm426xx_sc_step_c_set_delay(u64 delay)
{
	return 0;
}

static int icm426xx_sc_step_d_set_delay(u64 delay)
{
	return 0;
}

static int icm426xx_sc_get_data_step_c(uint32_t *value, int *status)
{
	uint32_t step_count = 0;
	int res;
	res = icm426xx_sc_ReadSensorData(&step_count);
    // when res is erro , then report last time step counter value.
	if(res < 0)
	{
		*value = previous_step ;
	}
	else{
		*value = step_count + step_count_overflowvalue;
		previous_step = step_count + step_count_overflowvalue;
	}

	
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}

static int icm426xx_sc_get_data_step_d(uint32_t *value, int *status)
{
	return 0;
}

static int icm426xx_sc_get_data_significant(uint32_t *value, int *status)
{
	return 0;
}

static int icm426xx_sc_get_data_floor_c(uint32_t *value, int *status)
{
	return 0;
}

static int icm426xx_sc_enable_floor_c(int en)
{
	return 0;
}
/*=======================================================================================*/
/* HAL Attribute Registration Section							 */
/*=======================================================================================*/

static int icm426xx_sc_attr_create(void)
{
	struct step_c_control_path ctl = {0};
	struct step_c_data_path data = {0};
	int res = 0;

	STEP_C_LOG();

	res = icm426xx_sc_init_client(false);
	if (res)
		goto exit_init_failed;

	// misc_register() for factory mode, engineer mode and so on
	res = misc_register(&icm426xx_sc_device);
	if (res) {
		STEP_C_PR_ERR("icm426xx_a_device misc register failed!\n");
		goto exit_misc_device_register_failed;
	}
#ifdef ICM426XX_DMP_STEP_COUNTER_TUNE
	// Crate platform_driver attribute
	res = icm426xx_sc_create_attr(&(icm426xx_sc_init_info.platform_diver_addr->driver));
	if (res) {
		STEP_C_PR_ERR("icm426xx_sc create attribute err = %d\n", res);
		goto exit_create_attr_failed;
	}
#endif
	/*-----------------------------------------------------------*/
	// Fill and Register the step_c_control_path and step_c_data_path
	/*-----------------------------------------------------------*/

	// Fill the step_c_control_path
	ctl.is_report_input_direct = false;
	ctl.is_counter_support_batch = false;
	ctl.is_detector_support_batch = false;
	ctl.is_smd_support_batch = false;	
	ctl.open_report_data = icm426xx_sc_open_report_data;		// function pointer
	ctl.enable_nodata = icm426xx_sc_enable_nodata;			// function pointer
	ctl.step_c_set_delay  = icm426xx_sc_step_c_set_delay;		// function pointer
	ctl.step_d_set_delay  = icm426xx_sc_step_d_set_delay;		// function pointer
	ctl.enable_significant  = icm426xx_sc_enable_significant;	// function pointer
	ctl.enable_step_detect  = icm426xx_sc_enable_step_detect;	// function pointer
	ctl.enable_floor_c          =icm426xx_sc_enable_floor_c;

	// Register the step_c_control_path
	res = step_c_register_control_path(&ctl);
	if (res) {
		STEP_C_PR_ERR("register step_c control path err\n");
		goto exit_kfree;
	}

	// Fill the step_c_data_path
	data.get_data = icm426xx_sc_get_data_step_c;			// function pointer
	data.get_data_step_d = icm426xx_sc_get_data_step_d;
	data.get_data_significant = icm426xx_sc_get_data_significant;
	data.get_data_floor_c      = icm426xx_sc_get_data_floor_c;
	data.vender_div = 1000;					// Taly : Maybe not necessary for step counter! Check later!

	// Register the step_c_data_path
	res = step_c_register_data_path(&data);
	if (res) {
		STEP_C_PR_ERR("register step_c data path fail = %d\n", res);
		goto exit_kfree;
	}

	// Set init_flag = 0 and return
	icm426xx_sc_init_flag = 0;

	STEP_C_LOG("%s: OK\n", __func__);
	return 0;

exit_misc_device_register_failed:
	misc_deregister(&icm426xx_sc_device);
#ifdef ICM426XX_DMP_STEP_COUNTER_TUNE
exit_create_attr_failed:
#endif
exit_init_failed:
exit_kfree:
//exit:
	icm426xx_sc_init_flag =  -1;
	STEP_C_PR_ERR("%s: err = %d\n", __func__, res);
	return res;
}

static int icm426xx_sc_attr_remove(void)
{
	int res = 0;
#ifdef ICM426XX_DMP_STEP_COUNTER_TUNE
	res = icm426xx_sc_delete_attr(&(icm426xx_sc_init_info.platform_diver_addr->driver));
	if (res)
		STEP_C_PR_ERR("icm426xx_sc_delete_attr fail: %d\n", res);
	else
	  	res = 0;
#endif 
	misc_deregister(&icm426xx_sc_device);
	return res;
}

/*=======================================================================================*/
/* Kernel Module Section								 */
/*=======================================================================================*/

static int icm426xx_sc_remove(void)
{
	icm426xx_sc_attr_remove();

	return 0;
}

static int icm426xx_sc_local_init(void)
{
	icm426xx_sc_attr_create();

	if (-1 == icm426xx_sc_init_flag)		
		return -1;
	return 0;
}
/*----------------------------------------------------------------------------*/
static int __init icm426xx_sc_init(void)
{
	step_c_driver_add(&icm426xx_sc_init_info);

	return 0;
}

static void __exit icm426xx_sc_exit(void)
{

}

/*----------------------------------------------------------------------------*/
module_init(icm426xx_sc_init);
module_exit(icm426xx_sc_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("icm426xx step counter driver");

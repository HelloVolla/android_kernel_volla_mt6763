/* drivers/i2c/chips/AFX133.c - AFX133 compass driver
 *
 * Copyright (C) 2013 VTC Technology Inc.
 * Author: Gary Huang <gary.huang@voltafield.com>
 *         George Tseng <george.tseng@voltafield.com>
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
 * For Android 8.0
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
#include <asm/atomic.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/hrtimer.h>
#include <hwmsensor.h>
#include <sensors_io.h>
#include <linux/types.h>
#include <hwmsen_helper.h>
#include <cust_mag.h>
#include <mag.h>

#include "afx133.h"

/* Orthogonal Compensation:: george modify begin 20180202 */
static int sin_table[31]=
{
       0, 
      87,  174,  261,  348,  436,
     523,  610,  697,  784,  871, 
     958, 1045, 1132, 1218, 1305,
    1391, 1478, 1564, 1650, 1736,
    1822, 1908, 1993, 2079, 2164,
    2249, 2334, 2419, 2503, 2588
};
static int coeff[4];
/* Orthogonal Compensation:: george modify begin 20180202 */

/* Add for auto detect feature */
static int afx133_local_init(void);
static int afx133_remove(void);
static int afx133_open_report_data(int en);
static int afx133_set_delay(u64 delay);
static int afx133_enable(int en);
static int afx133_get_data(int *x,int *y, int *z,int *status);

static int afx133_init_flag = -1;  //0:ok,,-1:fail

static struct mag_init_info afx133_init_info = {
        .name   = "afx133",	
        .init   = afx133_local_init,
        .uninit = afx133_remove,	
};

/*----------------------------------------------------------------------------*/
#define DEBUG 1
#define AFX133_DEV_NAME         "afx133"
#define DRIVER_VERSION          "1.8.5"
#define DRIVER_RELEASE          "20180723"
/*----------------------------------------------------------------------------*/
#define AFX133_DEFAULT_DELAY    40
#define AFX133_DELAY_MIN     		5
#define AFX133_DELAY_MAX     		100
#define AFX133_DELAY_SW_GYRO    10
/*----------------------------------------------------------------------------*/
#define MSE_TAG                 "MSENSOR"
#define MSE_FUN(f)              printk(MSE_TAG" %s\r\n", __FUNCTION__)
#define MSE_ERR(fmt, args...)   printk(KERN_ERR MSE_TAG" %s %d : \r\n"fmt, __FUNCTION__, __LINE__, ##args)
#define MSE_LOG(fmt, args...)   printk(MSE_TAG fmt, ##args)
#define MSE_VER(fmt, args...)   ((void)0)
static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

static atomic_t open_flag = ATOMIC_INIT(0);
static atomic_t m_flag = ATOMIC_INIT(0);
static atomic_t o_flag = ATOMIC_INIT(0);

/* Maintain  cust info here */
struct mag_hw mag_cust;
/* For  driver get cust info */
static struct mag_hw *hw = &mag_cust;

static int mag_offset[3];
static int g_PID;
/*----------------------------------------------------------------------------*/
static struct i2c_client *afx133_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id afx133_i2c_id[] = {{AFX133_DEV_NAME,0},{}};
//static struct i2c_board_info __initdata i2c_afx133={ I2C_BOARD_INFO("afx133", AFX133_I2C_ADDRESS)};  //7-bit address
/*----------------------------------------------------------------------------*/
static int afx133_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int afx133_i2c_remove(struct i2c_client *client);
static int afx133_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#if defined(CONFIG_PRIZE_HARDWARE_INFO)
#include "../../../hardware_info/hardware_info.h"
extern struct hardware_info current_msensor_info;
#endif
//static int afx133_suspend(struct i2c_client *client, pm_message_t msg) ;
//static int afx133_resume(struct i2c_client *client);
//static int afx133_suspend(struct device *dev);
//static int afx133_resume(struct device *dev);
/*----------------------------------------------------------------------------*/
typedef enum {
    VTC_TRC_DEBUG  = 0x01,
} AMI_TRC;

/*----------------------------------------------------------------------------*/
struct afx133_i2c_data {
    struct i2c_client *client;
    struct mag_hw *hw;
    struct hwmsen_convert   cvt;
    atomic_t layout;   
    atomic_t trace;
};
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id mag_of_match[] = {
	{.compatible = "mediatek,msensor"},
	{.compatible = "mediatek,m_sensor"},
    {},
};
#endif
/*----------------------------------------------------------------------------*/
static struct i2c_driver afx133_i2c_driver = {
    .driver = {
        //.owner = THIS_MODULE, 
        .name  = AFX133_DEV_NAME,
#ifdef CONFIG_OF 
        .of_match_table = mag_of_match,
#endif
    },
	.probe      = afx133_i2c_probe,
	.remove     = afx133_i2c_remove,
	.detect     = afx133_i2c_detect,
	//.suspend    = afx133_suspend,
	//.resume     = afx133_resume,
	.id_table   = afx133_i2c_id,
};

static DEFINE_MUTEX(afx133_mutex);
#define I2C_FLAG_WRITE	0
#define I2C_FLAG_READ	1

/*----------------------------------------------------------------------------*/
static atomic_t dev_open_count;
/*----------------------------------------------------------------------------*/

//========================================================================
static void afx133_power(struct mag_hw *hw, unsigned int on)
{
	static unsigned int power_on = 0;
	
	power_on = on;
}
/*----------------------------------------------------------------------------*/
static int VTC_i2c_Rx(struct i2c_client *client, char *rxData, int length)
{
        uint8_t retry;
        struct i2c_msg msgs[] = 
        {
            {
                    .addr = client->addr,
                    .flags = 0,
                    .len = 1,
                    .buf = rxData,
            },
            {
                    .addr = client->addr,
                    .flags = I2C_M_RD,
                    .len = length,
                    .buf = rxData,
            },
        };

        for (retry = 0; retry < 3; retry++) 
        {
                if (i2c_transfer(client->adapter, msgs, 2) > 0)
                        break;
                else
                        mdelay(10);
        }

        if (retry >= 3) 
        {
                printk(KERN_ERR "%s: retry over 3\n", __func__);
                return -EIO;
        } 
        else
                return 0;
}

static int VTC_i2c_Tx(struct i2c_client *client, char *txData, int length)
{
        int retry;
        struct i2c_msg msg[] = 
        {
                {
                        .addr = client->addr,
                        .flags = 0,
                        .len = length,
                        .buf = txData,
                },
        };

        for (retry = 0; retry <= 3; retry++) 
        {
                if (i2c_transfer(client->adapter, msg, 1) > 0)
                        break;
                else
                        mdelay(10);
        }

        if (retry > 3) 
        {
                printk(KERN_ERR "%s: retry over 3\n", __func__);
                return -EIO;
        }
        else
                return 0;
}
/*----------------------------------------------------------------------------*/
int afx133_i2c_master_operate(struct i2c_client *client, char *buf, int count, int i2c_flag)
{

	if(i2c_flag == I2C_FLAG_READ)
	{
	  return (VTC_i2c_Rx(client, buf, count>>8));
	}
	else if(i2c_flag == I2C_FLAG_WRITE)
	{
	  return (VTC_i2c_Tx(client, buf, count));
  }
  
  return 0;
}

/*----------------------------------------------------------------------------*/
static int afx133_GetOffset(void)
{
  if(g_PID == AF9133_PID)
  {
    unsigned char databuf[10];
    int S_Data[3], SR_Data[3];
	
    int i,j,index;
    int loop=5;
    
 
    if(NULL == afx133_i2c_client)
    {
  	  return -1;
    }   
  	
  	for(i=0;i<3;i++)
      mag_offset[i] = 0;
  	
  	for(i=0;i<loop;i++)
  	{
  		// set data
      databuf[0] = REG_MODE;
      databuf[1] = 0x38; 
      afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);    	
  	
      databuf[0] = REG_MEASURE;
      databuf[1] = 0x01;
      afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  	
      mdelay(5);
  	
      databuf[0] = REG_DATA;
      afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
      S_Data[0] = ((int) databuf[1]) << 8 | ((int) databuf[0]);
      S_Data[1] = ((int) databuf[3]) << 8 | ((int) databuf[2]);
      S_Data[2] = ((int) databuf[5]) << 8 | ((int) databuf[4]);
  
      for(j=0;j<3;j++) S_Data[j] = (S_Data[j] > 32767) ? (S_Data[j] - 65536) : S_Data[j];
  
      // set/reset data
      databuf[0] = REG_MODE;
      databuf[1] = 0x3C; 
      afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);    	
  	
      databuf[0] = REG_MEASURE;
      databuf[1] = 0x01;
      afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  	
      mdelay(5);
  	
      databuf[0] = REG_DATA;
      afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
      SR_Data[0] = ((int) databuf[1]) << 8 | ((int) databuf[0]);
      SR_Data[1] = ((int) databuf[3]) << 8 | ((int) databuf[2]);
      SR_Data[2] = ((int) databuf[5]) << 8 | ((int) databuf[4]);
  
      for(j=0;j<3;j++) SR_Data[j] = (SR_Data[j] > 32767) ? (SR_Data[j] - 65536) : SR_Data[j];
      	
      // offset data
      for(j=0;j<3;j++)
        mag_offset[j] += (S_Data[j] - SR_Data[j]) / loop;
    }
    
    /* Orthogonal Compensation:: george modify begin 20180202 */
    databuf[0] = REG_OTP_ORTHO_X;
    afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x03, I2C_FLAG_READ);
    
    coeff[0]= ((databuf[0] & 0x08) ? ((databuf[0]&0x0F) - 0x10) : (databuf[0]&0x0F)) / 2;
    coeff[1]= ((databuf[0] & 0x80) ? (((databuf[0]&0xF0)>>4) - 0x10) : ((databuf[0]&0xF0)>>4)) / 2;
    coeff[2]= ((databuf[1] & 0x80) ? (databuf[1] - 0x100) : databuf[1]) / 2;
    coeff[3]= ((databuf[2] & 0x80) ? (databuf[2] - 0x100) : databuf[2]) / 2;
    
    for(i=0;i<4;i++)
    {
    	index = coeff[i] * 2;
    		if(index > 30 || index < -30)
    		{
    			index = 0;
    		}
    	coeff[i] = (((index < 0) ? (-1) : 1) * sin_table[(index < 0) ? -index : index]);		
    }
    /* Orthogonal Compensation:: george modify begin 20180202 */   
  	
  	// back to set mode
    databuf[0] = REG_MODE;
    databuf[1] = 0x38; 
    afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);    	
  	
    databuf[0] = REG_MEASURE;
    databuf[1] = 0x01;
    afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  	
    mdelay(5);
  }	
  
  return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_Chipset_Init(int mode)
{
  u8 databuf[2];
  
  databuf[0] = REG_SW_RESET;
  databuf[1] = 0x81; 
  afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  
  
  mdelay(15);
  
  databuf[0] = REG_PCODE;
  afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x101, I2C_FLAG_READ); 	
  
  if(AF8133J_PID != databuf[0] && AF9133_PID != databuf[0] && AF5133_PID != databuf[0])
  {
  	MSE_ERR("afx133 PCODE is incorrect: %d\n", databuf[0]);
#ifndef BYPASS_PCODE_CHECK_FOR_MTK_DRL
  	return -3;
#endif
  } 
  else
  {
    g_PID = databuf[0];  
  	printk("%s chip id:%#x\n",__func__,databuf[0]);
  }
  
  databuf[0] = REG_I2C_CHECK;
  databuf[1] = 0x55; 
  afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE); 

  if(g_PID != AF9133_PID)
  {
    databuf[0] = REG_MODE;
    databuf[1] = 0x3E; 
    afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  
  
    databuf[0] = REG_MEASURE;
    databuf[1] = 0x01;
    afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  
    mdelay(5);
  }
  
  databuf[0] = REG_MODE;
  databuf[1] = 0x38; 
  afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  
  
  databuf[0] = REG_AVG;
  if(g_PID == AF9133_PID)
  	databuf[1] = 0x36;
  else if(g_PID == AF5133_PID)
  	databuf[1] = 0x36;
  else if(g_PID == AF8133J_PID)
  	databuf[1] = 0x04;	
  else	 
  {
  	databuf[1] = 0x04;	
    MSE_ERR("afx133 PID is incorrect: %d\n", g_PID);
  }
  afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  
  
  databuf[0] = REG_OSR;
  databuf[1] = 0x15;
  afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  
  
  databuf[0] = REG_WAITING;
  databuf[1] = 0x66;     
  afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
  
  return 0;
}
/*----------------------------------------------------------------------------*/
__maybe_unused static int afx133_SetMode(int newmode)
{           
	return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_ReadSensorData(int *buf)
{
  unsigned char databuf[10];
  int output[3];
  int i;
  int temp[3];
  
  if(NULL == afx133_i2c_client)
  {
  	*buf = 0;
  	return -2;
  }   
  	
  databuf[0] = REG_DATA;
  afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
  output[0] = ((int) databuf[1]) << 8 | ((int) databuf[0]);
  output[1] = ((int) databuf[3]) << 8 | ((int) databuf[2]);
  output[2] = ((int) databuf[5]) << 8 | ((int) databuf[4]);
  
  for(i=0;i<3;i++) output[i] = (output[i] > 32767) ? (output[i] - 65536) : output[i];
  
  for(i=0;i<3;i++)
  {
    if(g_PID == AF9133_PID)
      buf[i] = output[i] - mag_offset[i];
    else
    	buf[i] = output[i];
  }
    	
  /* Orthogonal Compensation:: george modify begin 20180202 */
  if(g_PID == AF9133_PID || g_PID == AF8133J_PID)
  {
    temp[0] = buf[0];
    temp[1] = buf[1];
    temp[2] = buf[2];
    
    buf[2] = coeff[2] * temp[0] / 10000 +  // zx
             coeff[3] * temp[1] / 10000 +  // zy
                        temp[2];           // zz
    buf[1] = coeff[1] * temp[0] / 10000 +  // yx 
                        temp[1];           // yy
    buf[0] = coeff[0] * temp[1] / 10000 +  // xy 
                        temp[0];           // xx	
  }
  /* Orthogonal Compensation:: george modify end 20180202 */ 	
  
  databuf[0] = REG_MEASURE;
  databuf[1] = 0x01;
  afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
  
  return 0;
 }

/*----------------------------------------------------------------------------*/
static ssize_t show_daemon_name(struct device_driver *ddri, char *buf)
{
	char strbuf[10];
	sprintf(strbuf, "afx133d");
	return sprintf(buf, "%s", strbuf);		
}
/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char databuf[2];
	
	databuf[0] = REG_PCODE;
	afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x101, I2C_FLAG_READ); 	
  	
	if(AF8133J_PID != databuf[0] && AF9133_PID != databuf[0] && AF5133_PID != databuf[0])
	{
		printk("afx133 PCODE is incorrect: %d\n", databuf[0]);
	} 

	return sprintf(buf, "%X\n", databuf[0]);       
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	int databuf[3];
	afx133_ReadSensorData(databuf);
	return sprintf(buf, "%d %d %d\n", databuf[0],databuf[1],databuf[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_calidata_value(struct device_driver *ddri, char *buf)
{
	int tmp[3];
	int databuf[3] = {0};
	char strbuf[32];
	
	afx133_ReadSensorData(databuf);
	
	tmp[0] = databuf[0] / CONVERT_M_DIV;
	tmp[1] = databuf[1] / CONVERT_M_DIV;
	tmp[2] = databuf[2] / CONVERT_M_DIV; 
	
	sprintf(strbuf, "%d, %d, %d\n", tmp[0],tmp[1], tmp[2]);	
	
	return sprintf(buf, "%s\n", strbuf);            
}

/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
    struct afx133_i2c_data *data;

    if(NULL == afx133_i2c_client)
	{
		MSE_ERR("afx133_i2c_client IS NULL !\n");
		return -1;
	}

    data = i2c_get_clientdata(afx133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw->direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);            
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct afx133_i2c_data *data;
    int layout;

	if(NULL == afx133_i2c_client)
	{
		MSE_ERR("afx133_i2c_client IS NULL !\n");
		return -1;
	}

    data = i2c_get_clientdata(afx133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

	layout = 0;
	if(1 == sscanf(buf, "%d", &layout))
	{
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			MSE_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		}
		else if(!hwmsen_get_convert(data->hw->direction, &data->cvt))
		{
			MSE_ERR("invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		}
		else
		{
			MSE_ERR("invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	}
	else
	{
		MSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;            
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
    struct afx133_i2c_data *data;
    ssize_t len;

	if(NULL == afx133_i2c_client)
	{
		MSE_ERR("afx133_i2c_client IS NULL !\n");
		return -1;
	}
    
	data = i2c_get_clientdata(afx133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

	len = 0;
	if(data->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n", 
		data->hw->i2c_num, data->hw->direction, data->hw->power_id, data->hw->power_vol);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "OPEN: %d\n", atomic_read(&dev_open_count));
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_status_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct afx133_i2c_data *data;
    int value;

	if(NULL == afx133_i2c_client)
	{
		MSE_ERR("afx133_i2c_client IS NULL !\n");
		return -1;
	}

    data = i2c_get_clientdata(afx133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

    value = simple_strtol(buf, NULL, 10);
    
    data->hw->direction = value;
	if(hwmsen_get_convert(value, &data->cvt)<0)
	{
		MSE_ERR("invalid direction: %d\n", value);
	}

	atomic_set(&data->layout, value);
	return count;    

}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
    struct afx133_i2c_data *data;
	ssize_t res;

	if(NULL == afx133_i2c_client)
	{
		MSE_ERR("afx133_i2c_client IS NULL !\n");
		return -1;
	}

    data = i2c_get_clientdata(afx133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}	

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&data->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct afx133_i2c_data *data;
    int trace = 0;

	if(NULL == afx133_i2c_client)
	{
		MSE_ERR("afx133_i2c_client IS NULL !\n");
		return -1;
	}

    data = i2c_get_clientdata(afx133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}
    
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&data->trace, trace);
	}
	else 
	{
		MSE_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}

	return count;    
}

/*----------------------------shipment test------------------------------------------------*/
/*!
 @return If @a testdata is in the range of between @a lolimit and @a hilimit,
 the return value is 1, otherwise -1.
 @param[in] testno   A pointer to a text string.
 @param[in] testname A pointer to a text string.
 @param[in] testdata A data to be tested.
 @param[in] lolimit  The maximum allowable value of @a testdata.
 @param[in] hilimit  The minimum allowable value of @a testdata.
 @param[in,out] pf_total
 */
int AFX133_TEST_DATA(const char testno[], const char testname[], const int testdata,
	const int lolimit, const int hilimit, int *pf_total)
{
	int pf;			/* Pass;1, Fail;-1 */

	if ((testno == NULL) && (strncmp(testname, "START", 5) == 0)) {
		MSE_LOG("--------------------------------------------------------------------\n");
		MSE_LOG(" Test No. Test Name	Fail	Test Data	[	 Low	High]\n");
		MSE_LOG("--------------------------------------------------------------------\n");
		pf = 1;
	} else if ((testno == NULL) && (strncmp(testname, "END", 3) == 0)) {
		MSE_LOG("--------------------------------------------------------------------\n");
		if (*pf_total == 1)
			MSE_LOG("Factory shipment test was passed.\n\n");
		else
			MSE_LOG("Factory shipment test was failed.\n\n");

		pf = 1;
	} else {
		if ((lolimit <= testdata) && (testdata <= hilimit))
			pf = 1;
		else
			pf = -1;

	/* display result */
	MSE_LOG(" %7s  %-10s	 %c	%9d	[%9d	%9d]\n",
		testno, testname, ((pf == 1) ? ('.') : ('F')), testdata,
		lolimit, hilimit);
	}

	/* Pass/Fail check */
	if (*pf_total != 0) {
		if ((*pf_total == 1) && (pf == 1))
			*pf_total = 1;		/* Pass */
		else
			*pf_total = -1;		/* Fail */
	}
	return pf;
}
/*----------------------------------------------------------------------------*/
/*!
 Execute "Onboard Function Test" (NOT includes "START" and "END" command).
 @retval 1 The test is passed successfully.
 @retval -1 The test is failed.
 @retval 0 The test is aborted by kind of system error.
 */
int FST_AFX133(void)
{
	int  pf_total;  /* p/f flag for this subtest */
	u8   databuf[6];
	int  value[3];
	int  pos[3];
  int  neg[3];
  int  offset[3];
  int  i;
  int  pid_flag=0;

	/* *********************************************** */
	/* Reset Test Result */
	/* *********************************************** */
	pf_total = 1;

	/* *********************************************** */
	/* Step1 */
	/* *********************************************** */

	/* Reset device. */
	afx133_Chipset_Init(0);

	/* Read values from WIA. */
	databuf[0] = REG_PCODE;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x101, I2C_FLAG_READ) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

#ifndef BYPASS_PCODE_CHECK_FOR_MTK_DRL
  if(AF8133J_PID == databuf[0] || AF9133_PID == databuf[0] || AF5133_PID == databuf[0])
    pid_flag = 1;

	/* TEST */
	AFX133_TEST_DATA("T1", "AFX133 Product Code is correct", pid_flag, 1, 1, &pf_total);
#else
  AFX133_TEST_DATA("T1", "Bypass AFX133 Product Code", pid_flag, 0, 0, &pf_total);
#endif


	/* Find offset by SW set_reset */
	// neg data (reset)
  databuf[0] = REG_MODE;
	databuf[1] = 0x34;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

  databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}
	mdelay(5);

	databuf[0] = REG_DATA;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x601, I2C_FLAG_READ) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

  neg[0] = databuf[0] | (databuf[1] << 8);
  neg[1] = databuf[2] | (databuf[3] << 8);
  neg[2] = databuf[4] | (databuf[5] << 8);
  for(i=0;i<3;i++) neg[i] = (neg[i] > 32767) ? (neg[i] - 65536) : neg[i];

  // pos data (reset)
  databuf[0] = REG_MODE;
	databuf[1] = 0x38;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

  databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}
	mdelay(5);

	databuf[0] = REG_DATA;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x601, I2C_FLAG_READ) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

	pos[0] = databuf[0] | (databuf[1] << 8);
  pos[1] = databuf[2] | (databuf[3] << 8);
  pos[2] = databuf[4] | (databuf[5] << 8);
  for(i=0;i<3;i++) pos[i] = (pos[i] > 32767) ? (pos[i] - 65536) : pos[i];

  // offset
  for(i=0;i<3;i++) offset[i] = (pos[i] + neg[i]) / 2;

	/* TEST */
	AFX133_TEST_DATA("T2_1", "AFX133 x-axis offset", offset[0], AFX133_OFFSET_MIN, AFX133_OFFSET_MAX, &pf_total);
	AFX133_TEST_DATA("T2_2", "AFX133 y-axis offset", offset[1], AFX133_OFFSET_MIN, AFX133_OFFSET_MAX, &pf_total);
	AFX133_TEST_DATA("T2_3", "AFX133 z-axis offset", offset[2], AFX133_OFFSET_MIN, AFX133_OFFSET_MAX, &pf_total);

	/* *********************************************** */
	/* Step2 */
	/* *********************************************** */
	
  /* Set to Self-test mode */
  databuf[0] = REG_I2C_CHECK;
	databuf[1] = 0x55;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}	

  /* Set to Self-test mode */
  databuf[0] = REG_MODE;
	databuf[1] = 0x3C;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

  /************************************************************************************/
	// positive BIST function (X and Y)
  databuf[0] = REG_BIST;
	databuf[1] = 0xB1;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

  // open clock
  databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}
  mdelay(5);

  // positive measurement (X and Y)
  databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}
  mdelay(5);

	databuf[0] = REG_DATA;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x601, I2C_FLAG_READ) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

	pos[0] = databuf[0] | (databuf[1] << 8);
  pos[1] = databuf[2] | (databuf[3] << 8); 
  pos[0] = (pos[0] > 32767) ? (pos[0] - 65536) : pos[0];
  pos[1] = (pos[1] > 32767) ? (pos[1] - 65536) : pos[1];

	// negative BIST function  (X and Y)
  databuf[0] = REG_BIST;
	databuf[1] = 0xC1;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

  // open clock
  databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}
  mdelay(5);

  // negative measurement (X and Y)
  databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}
  mdelay(5);

	databuf[0] = REG_DATA;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x601, I2C_FLAG_READ) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

  neg[0] = databuf[0] | (databuf[1] << 8);
  neg[1] = databuf[2] | (databuf[3] << 8);
  neg[0] = (neg[0] > 32767) ? (neg[0] - 65536) : neg[0];
  neg[1] = (neg[1] > 32767) ? (neg[1] - 65536) : neg[1];

  /************************************************************************************/
	// positive BIST function (Z)
  databuf[0] = REG_BIST;
	databuf[1] = 0xD1;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

  // open clock
  databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}
  mdelay(5);

  // positive measurement (Z)
  databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}
  mdelay(5);

	databuf[0] = REG_DATA;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x601, I2C_FLAG_READ) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

	pos[2] = databuf[4] | (databuf[5] << 8);
  pos[2] = (pos[2] > 32767) ? (pos[2] - 65536) : pos[2];

	// negative BIST function (Z)
  databuf[0] = REG_BIST;
	databuf[1] = 0xE1;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

  // open clock
  databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}
  mdelay(5);

  // negative measurement (Z)
  databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}
  mdelay(5);

	databuf[0] = REG_DATA;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x601, I2C_FLAG_READ) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

  neg[2] = databuf[4] | (databuf[5] << 8);
  neg[2] = (neg[2] > 32767) ? (neg[2] - 65536) : neg[2];

  /************************************************************************************/
  
  // calculate sensitivity
  if(g_PID == AF8133J_PID)
  {
    value[0] = pos[0] - neg[0];
    value[1] = pos[1] - neg[1];
    value[2] = pos[2] - neg[2]; 
  }
  else if(g_PID == AF9133_PID)   
  {
    value[0] = neg[0] - pos[0];
    value[1] = neg[1] - pos[1];
    value[2] = pos[2] - neg[2];   	
  }
  else //AF5133
  {
    value[0] = neg[0] - pos[0];
    value[1] = neg[1] - pos[1];
    value[2] = pos[2] - neg[2];  	
  }

	/* TEST */
	AFX133_TEST_DATA("T3_1", "AFX133 BIST test 1", value[0], AFX133_SENS_MIN, AFX133_SENS_MAX, &pf_total);
	AFX133_TEST_DATA("T3_2", "AFX133 BIST test 2", value[1], AFX133_SENS_MIN, AFX133_SENS_MAX, &pf_total);
	AFX133_TEST_DATA("T3_2", "AFX133 BIST test 3", value[2], AFX133_SENS_MIN, AFX133_SENS_MAX, &pf_total);

  // disbale BIST function
  databuf[0] = REG_BIST;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

  // open clock
  databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}
  mdelay(5);

	return pf_total;
}
/*----------------------------------------------------------------------------*/
/*!
 Execute "Onboard Function Test" (includes "START" and "END" command).
 @retval 1 The test is passed successfully.
 @retval -1 The test is failed.
 @retval 0 The test is aborted by kind of system error.
 */
int AFX133_FctShipmntTestProcess_Body(void)
{
	int pf_total = 1;

	/* *********************************************** */
	/* Reset Test Result */
	/* *********************************************** */
	AFX133_TEST_DATA(NULL, "START", 0, 0, 0, &pf_total);

	/* *********************************************** */
	/* Step 1 to 2 */
	/* *********************************************** */
	pf_total = FST_AFX133();

	/* *********************************************** */
	/* Judge Test Result */
	/* *********************************************** */
	AFX133_TEST_DATA(NULL, "END", 0, 0, 0, &pf_total);

	return pf_total;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_shipment_test(struct device_driver *ddri, const char *buf, size_t count)
{
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_shipment_test(struct device_driver *ddri, char *buf)
{
	char result[10];
	int res = 0;

	res = AFX133_FctShipmntTestProcess_Body();
	if (1 == res) {
		MSE_LOG("shipment_test pass\n");
		strncpy(result, "y", sizeof(result));
	} else if (-1 == res) {
		MSE_LOG("shipment_test fail\n");
		strncpy(result, "n", sizeof(result));
	} else {
		MSE_LOG("shipment_test NaN\n");
		strncpy(result, "NaN", sizeof(result));
	}

	return sprintf(buf, "%s\n", result);
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(daemon,      S_IRUGO, show_daemon_name, NULL);
static DRIVER_ATTR(chipinfo,    S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata,  S_IRUGO, show_sensordata_value, NULL);
//static DRIVER_ATTR(posturedata, S_IRUGO, show_posturedata_value, NULL);
static DRIVER_ATTR(calidata,    S_IRUGO, show_calidata_value, NULL);
static DRIVER_ATTR(layout,      S_IRUGO | S_IWUSR, show_layout_value, store_layout_value );
static DRIVER_ATTR(status,      S_IRUGO | S_IWUSR, show_status_value, store_status_value);
static DRIVER_ATTR(trace,       S_IRUGO | S_IWUSR, show_trace_value, store_trace_value );
static DRIVER_ATTR(shipmenttest,S_IRUGO | S_IWUSR, show_shipment_test, store_shipment_test);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *afx133_attr_list[] = {
  &driver_attr_daemon,
	&driver_attr_chipinfo,
	&driver_attr_sensordata,
	//&driver_attr_posturedata,
	&driver_attr_calidata,
	&driver_attr_layout,
	&driver_attr_status,
	&driver_attr_trace,
	&driver_attr_shipmenttest,
};
/*----------------------------------------------------------------------------*/
static int afx133_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(afx133_attr_list)/sizeof(afx133_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, afx133_attr_list[idx])))
		{            
			MSE_ERR("driver_create_file (%s) = %d\n", afx133_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int afx133_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(afx133_attr_list)/sizeof(afx133_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}


	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, afx133_attr_list[idx]);
	}


	return err;
}

/*----------------------------------------------------------------------------*/
static int afx133_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;
  struct afx133_i2c_data *data;
	
	if(NULL == afx133_i2c_client)
	{
		MSE_ERR("afx133_i2c_client IS NULL !\n");
		return -1;
	}

	data = i2c_get_clientdata(afx133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}
  
	value = (int)samplingPeriodNs / 1000 / 1000;
	
	MSE_LOG("afx133 mag set delay = (%d) ok.\n", value);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int afx133_flush(void)
{
	return mag_flush_report();
}
/*----------------------------------------------------------------------------*/
static int afx133_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
	int err;

	err = afx133_enable(enabledisable == true ? 1 : 0);
	if (err) {
		MSE_ERR("%s enable sensor failed!\n", __func__);
		return -1;
	}
	err = afx133_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		MSE_ERR("%s enable set batch failed!\n", __func__);
		return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_factory_get_data(int32_t data[3], int *status)
{
	/* get raw data */
	return  afx133_get_data(&data[0], &data[1], &data[2], status);
}
/*----------------------------------------------------------------------------*/
static int afx133_factory_get_raw_data(int32_t data[3])
{
	MSE_LOG("do not support afx133_factory_get_raw_data!\n");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_factory_enable_calibration(void)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_factory_clear_cali(void)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_factory_set_cali(int32_t data[3])
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_factory_get_cali(int32_t data[3])
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_factory_do_self_test(void)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct mag_factory_fops afx133_factory_fops = {
	.enable_sensor      = afx133_factory_enable_sensor,
	.get_data           = afx133_factory_get_data,
	.get_raw_data       = afx133_factory_get_raw_data,
	.enable_calibration = afx133_factory_enable_calibration,
	.clear_cali         = afx133_factory_clear_cali,
	.set_cali           = afx133_factory_set_cali,
	.get_cali           = afx133_factory_get_cali,
	.do_self_test       = afx133_factory_do_self_test,
};
/*----------------------------------------------------------------------------*/
static struct mag_factory_public afx133_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &afx133_factory_fops,
};
/*----------------------------------------------------------------------------*/
static int afx133_open_report_data(int en)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_set_delay(u64 delay)
{
	int value = 0;
	struct afx133_i2c_data *data;
	
	if(NULL == afx133_i2c_client)
	{
		MSE_ERR("afx133_i2c_client IS NULL !\n");
		return -1;
	}

    data = i2c_get_clientdata(afx133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

	value = (int)(delay / 1000 / 1000);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, AFX133_DEV_NAME);
    return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_enable(int en)
{
	int value = 0;
    struct afx133_i2c_data *data ;
	if(NULL == afx133_i2c_client)
	{
		MSE_ERR("afx133_i2c_client IS NULL !\n");
		return -1;
	}

    data = i2c_get_clientdata(afx133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

	value = en;
	if(value == 1)
	{
		atomic_set(&m_flag, 1);
		atomic_set(&open_flag, 1);
	}
	else
	{
		atomic_set(&m_flag, 0);
		if(atomic_read(&o_flag) == 0)
		{
			atomic_set(&open_flag, 0);
		}	
	}
	wake_up(&open_wq);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_get_data(int *x,int *y, int *z,int *status)
{
  struct afx133_i2c_data *data ;
  int databuf[3]={0};
	
	if(NULL == afx133_i2c_client)
	{
		MSE_ERR("afx133_i2c_client IS NULL !\n");
		return -1;
	}

    data = i2c_get_clientdata(afx133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

	afx133_ReadSensorData(databuf); 
	
	*x = databuf[0];
	*y = databuf[1];
	*z = databuf[2];

	return 0;
}

/*----------------------------------------------------------------------------*/
/*
static int afx133_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct afx133_i2c_data *obj = i2c_get_clientdata(client);
	MSE_FUN();    
#if 1
	if(msg.event == PM_EVENT_SUSPEND)
	{   
		afx133_power(obj->hw, 0);   
			}
#endif
	return 0;
}
*/
/*----------------------------------------------------------------------------*/
/*
static int afx133_resume(struct i2c_client *client)
{
	int err;
	struct afx133_i2c_data *obj = i2c_get_clientdata(client);
	MSE_FUN();
#if 1
	afx133_power(obj->hw, 1);

	if((err = afx133_Chipset_Init(AFX133_MODE_SINGLE))!=0)
	{
		MSE_ERR("initialize client fail!!\n");
		return err;        
	}
#endif
	return 0;
}
*/
/*----------------------------------------------------------------------------*/
static int afx133_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct afx133_i2c_data *data;
	int err = 0;

    struct mag_control_path ctl_path ={0};
    struct mag_data_path dat_path = {0};

  printk("%s start\n",__func__);
  
  err = get_mag_dts_func(client->dev.of_node, hw);
	if (err < 0) 
	{
		MSE_ERR("get dts info fail\n");
		err = -EFAULT;
		goto exit;
	}

	if (!(data = kmalloc(sizeof(struct afx133_i2c_data), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct afx133_i2c_data));

   data->hw = hw;
	data->hw->direction = 3;
	err = hwmsen_get_convert(data->hw->direction, &data->cvt);
	if(err)
	{
		MSE_ERR("lsw_afx133 invalid direction: %d\n", data->hw->direction);
		goto exit;
	}
	//MSE_ERR("lsw_afx133  direction: %d\n", data->hw->direction);
	atomic_set(&data->layout, data->hw->direction);
	atomic_set(&data->trace, 0);

	init_waitqueue_head(&open_wq);

	data->client = client;
	new_client = data->client;
	i2c_set_clientdata(new_client, data);

	afx133_i2c_client = new_client;	

	if((err = afx133_Chipset_Init(AFX133_MODE_IDLE)))
	{
		MSE_ERR("afx133 register initial fail\n");
		goto exit_init_failed;
	}
	
	if((err = afx133_GetOffset()))
	{
		MSE_ERR("afx133 get offset initial fail\n");
		goto exit_init_failed;
	}

	/* Register sysfs attribute */
	if((err = afx133_create_attr(&afx133_init_info.platform_diver_addr->driver)))
	{
		MSE_ERR("create attribute err = %d\n", err);
		goto exit_sysfs_create_group_failed;
	}
	
	err = mag_factory_device_register(&afx133_factory_device);
	if (err)
	{
		MSE_ERR("factory device register failed, err = %d\n", err);
		goto exit_misc_device_register_failed;
	}
    
    ctl_path.is_use_common_factory = false;
		ctl_path.open_report_data = afx133_open_report_data;
		ctl_path.enable 					= afx133_enable;
		ctl_path.set_delay 				= afx133_set_delay;

		ctl_path.is_report_input_direct = false;
		ctl_path.is_support_batch 		= data->hw->is_batch_supported;
		ctl_path.batch = afx133_batch;
		ctl_path.flush = afx133_flush;
		//ctl_path.lib_name = "libvtclib";
		//strlcpy(ctl_path.libinfo.libname, "vtclib", sizeof(ctl_path.libinfo.libname));
		strcpy(ctl_path.libinfo.libname, "vtclib");
	  ctl_path.libinfo.layout = hw->direction;
	  ctl_path.libinfo.deviceid = AF8133J_PID;
    
    if(g_PID == AF9133_PID)
  	  ctl_path.libinfo.deviceid = AF9133_PID;
    else if(g_PID == AF5133_PID)
  	  ctl_path.libinfo.deviceid = AF5133_PID;
    else if(g_PID == AF8133J_PID)
  	  ctl_path.libinfo.deviceid = AF8133J_PID;
		
		err = mag_register_control_path(&ctl_path);

		if(err < 0)
		{
			MSE_ERR("mag_register_control_path failed!\n");
			goto exit_misc_device_register_failed;
		}


		dat_path.div = CONVERT_M_DIV;

		dat_path.get_data = afx133_get_data;

		err = mag_register_data_path(&dat_path);
		if(err < 0)
		{
			MSE_ERR("mag_register_control_path failed!\n");
			goto exit_misc_device_register_failed;
		}

	MSE_LOG("%s: OK\n", __func__);

	afx133_init_flag=0;
    #if defined(CONFIG_PRIZE_HARDWARE_INFO)
		strcpy(current_msensor_info.chip,"afx133");
		sprintf(current_msensor_info.id,"0x%04x",client->addr);
		strcpy(current_msensor_info.vendor,"senodia");
		strcpy(current_msensor_info.more,"magnetometer");
	#endif
	return 0;

	exit_sysfs_create_group_failed:   
	exit_init_failed:
	exit_misc_device_register_failed:
#ifdef SUPPORT_SOFTGYRO_FUNCTION
  exit_kfree:
#endif
  	kfree(data);
	exit:
	MSE_ERR("%s: err = %d\n", __func__, err);

	afx133_init_flag=-1;

	return err;
}
/*----------------------------------------------------------------------------*/
static int afx133_i2c_remove(struct i2c_client *client)
{
	int err;	

	if((err = afx133_delete_attr(&afx133_init_info.platform_diver_addr->driver)))
	{
		MSE_ERR("afx133_delete_attr fail: %d\n", err);
	}

	afx133_i2c_client = NULL;
	i2c_unregister_device(client);	
	mag_factory_device_deregister(&afx133_factory_device);
	kfree(i2c_get_clientdata(client));    
	return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_local_init(void)
{
	printk("afx133_local_init");  
	atomic_set(&dev_open_count, 0);
  
	if(i2c_add_driver(&afx133_i2c_driver))
	{
		MSE_ERR("add driver error\n");
		return -1;
	} 
	 
	if(-1 == afx133_init_flag)	
	{	   
		return -1;	
	}
	
    printk("%s done\n",__func__);
    
	return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_remove(void)
{
	MSE_FUN();
	afx133_power(hw, 0);    
	atomic_set(&dev_open_count, 0);  
	i2c_del_driver(&afx133_i2c_driver);
	afx133_init_flag = -1;
	return 0;
}
/*----------------------------------------------------------------------------*/
static int __init afx133_init(void)
{

	int err;
	MSE_FUN();
	err = mag_driver_add(&afx133_init_info);
	
	if(err < 0)
	{
		MSE_ERR("mag_driver_add failed!\n");
	}

	return 0;    
}
/*----------------------------------------------------------------------------*/
static void __exit afx133_exit(void)
{
	MSE_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(afx133_init);
module_exit(afx133_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Gary Huang, George Tseng");
MODULE_DESCRIPTION("AFX133 m-Sensor driver");
MODULE_LICENSE("VTC");
MODULE_VERSION(DRIVER_VERSION);


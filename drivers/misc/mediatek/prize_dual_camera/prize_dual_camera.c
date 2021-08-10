/**************************************************************************
*  double_camera.c
* 
*  Create Date : 
* 
*  Modify Date : 
*
*  Create by   : AWINIC Technology CO., LTD
*
*  Version     : 0.9, 2016/02/15
**************************************************************************/

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>

#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/init.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/mutex.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"


#define DCAM_NAME "dcam"
#define DCAM_R_NAME "dcam_r"
#define DCAM_F_NAME "dcam_f"
#define GC6133_WRITE_ID	0x80
#define GC6133_READ_ID	0x81
//prize  add  for gc6133/gc0310/gc2145 by zhuzhengjiang    20190116-begin
#define GC6133_SENSOR_ID	0xBA
//#define GC0310_SENSOR_ID	0xA310
#define GC2145_SENSOR_ID	0x2145
#define GC6153_SENSOR_ID	0x6153//yanrenjie added 6153 20190416

#define GC6133_SERIAL_ADDR	0x40
#define GC0310_SERIAL_ADDR	0x21
#define GC2145_SERIAL_ADDR	0x3C
#define GC6153_SERIAL_ADDR	0x40//yanrenjie added 6153 20190416
//prize  add  for gc6133/gc0310/gc2145 by zhuzhengjiang    20190116-end
#define CAMERA_DEBUG
#ifdef CAMERA_DEBUG

#define CAMERA_DBG(fmt,arg...) \
	do{\
		printk("<<DCAM-DBG>>[%s:%d]"fmt"\n",__func__, __LINE__, ##arg);\
    }while(0)
#else
#define CAMERA_DBG(fmt,arg...)
#endif

struct dcam_data_t{
	char name[5];
	char is_enabled;
	unsigned int sensor_type;
	unsigned int pdn_pin;
	unsigned int rst_pin;
};

static struct dcam_data_t dcam_r_devtype = {
	.name = "rear",
	.is_enabled = 0,
	.sensor_type = 0,
	.pdn_pin = 0,
	.rst_pin = 0,
};

static struct dcam_data_t dcam_f_devtype = {
	.name = "front",
	.is_enabled = 0,
	.sensor_type = 0,
	.pdn_pin = 0,
	.rst_pin = 0,
};

static DEFINE_MUTEX(dcam_r_mutex);
static DEFINE_MUTEX(dcam_f_mutex);

static struct i2c_client *g_dcam_r_client = NULL;
static struct i2c_client *g_dcam_f_client = NULL;
static struct kobject *dcam_kobj = NULL;

//static DEFINE_SPINLOCK(GC6133_drv_lock);
//extern int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, bool On, char *mode_name);
//prize  add  for gc6133/gc0310/gc2145 by zhuzhengjiang    20190116-begin
extern void set_avdd_regulator(int status);
extern void set_dvdd_regulator(int status);
kal_uint32 sensor_id=0;
int kdCISModulePowerOn(struct i2c_client* client,  bool On)
{
	struct dcam_data_t *dcam_data = i2c_get_clientdata(client);
	//printk("kdCISModulePowerOn  client.addr=0x%x  on=%d",client->addr,On);
	if(client->addr == 0x3c)
	{
	//gc2145
		if (On){
			gpio_direction_output(dcam_data->pdn_pin,1);
			mdelay(2);
			gpio_direction_output(dcam_data->rst_pin,0);
			mdelay(5);
			set_dvdd_regulator(1);
			set_avdd_regulator(1);
			mdelay(2);
			gpio_direction_output(dcam_data->pdn_pin,0);
			mdelay(1);
			gpio_direction_output(dcam_data->rst_pin,1);
		}else{
			gpio_direction_output(dcam_data->pdn_pin,1);
			gpio_direction_output(dcam_data->rst_pin,0);
			mdelay(2);
			set_avdd_regulator(0);
			set_dvdd_regulator(0);
		}
	}
	else if(client->addr == 0x40) 
	{
	//gc6133+gc6153
		if (On){
			set_avdd_regulator(1);
			mdelay(5);
			gpio_direction_output(dcam_data->pdn_pin,1);
			mdelay(4);
			gpio_direction_output(dcam_data->pdn_pin,0);
		}else{
			set_avdd_regulator(0);
			mdelay(5);
			gpio_direction_output(dcam_data->pdn_pin,1);
		}
	}
	else
	{
	//gc0310
		if (On){
			gpio_direction_output(dcam_data->pdn_pin,0);
			mdelay(3);
			set_avdd_regulator(1);
			mdelay(3);
			gpio_direction_output(dcam_data->pdn_pin,1);
			mdelay(1);
			gpio_direction_output(dcam_data->pdn_pin,0);
		}else{
			set_avdd_regulator(0);
			mdelay(5);
			gpio_direction_output(dcam_data->pdn_pin,1);
		}
	}	
	printk("DCAM sensorIdx(%s) level(%d)\n",dcam_data->name,On);
	return 0;
}
//prize  add  for gc6133/gc0310/gc2145 by zhuzhengjiang    20190116-end

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// i2c write and read
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static unsigned char I2C_write_reg(struct i2c_client *client, unsigned char addr, unsigned char reg_data)
{
	char ret;
	u8 wdbuf[512] = {0};

	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= wdbuf,
		},
	};

	wdbuf[0] = addr;
	wdbuf[1] = reg_data;

	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

    return ret;
}

static unsigned char I2C_read_reg(struct i2c_client *client, unsigned char addr)
{
	unsigned char ret;
	u8 rdbuf[512] = {0};

	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rdbuf,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= rdbuf,
		},
	};

	rdbuf[0] = addr;
	
	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

    return rdbuf[0];
}

static inline kal_uint16 write_cmos_sensor(struct i2c_client *client, kal_uint8 addr, kal_uint8 para)
{
	I2C_write_reg(client, addr, para);
	return 0;
}
static kal_uint16 read_cmos_sensor(struct i2c_client *client, kal_uint8 addr)
{
	kal_uint16 get_byte=0;
   
	get_byte=I2C_read_reg(client, addr);
    return get_byte;
}
//prize  add  for gc6133/gc0310/gc2145 by zhuzhengjiang    20190116-begin
static void CameraStreamOn(struct i2c_client *client)
{
	//msleep(150);
    if(sensor_id == GC6133_SENSOR_ID)
		client->addr = GC6133_SERIAL_ADDR;
	else if(sensor_id == GC2145_SENSOR_ID)
		client->addr = GC2145_SERIAL_ADDR;
	else if(sensor_id == GC0310_SENSOR_ID)
		client->addr = GC0310_SERIAL_ADDR;
	else if(sensor_id == GC6153_SENSOR_ID)//prize yanrenjie added 6153 20190416
		client->addr = GC6153_SERIAL_ADDR;	
	else
		client->addr = 0x3c;

    write_cmos_sensor(client,0xfe,0x03);
    write_cmos_sensor(client,0x10,0x94);
    write_cmos_sensor(client,0xfe,0x00);
    //msleep(50);
}

static void GC6133_Sensor_Init(struct i2c_client *client)
{
	/////////////////////////////////////////////////////
	//////////////////////	 SYS   //////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0xfe, 0xa0);
	write_cmos_sensor(client,0xfe, 0xa0);
	write_cmos_sensor(client,0xfe, 0xa0);

	write_cmos_sensor(client,0xf6, 0x00);
	write_cmos_sensor(client,0xfa, 0x11);
	write_cmos_sensor(client,0xfc, 0x12); //clock enable

	write_cmos_sensor(client,0xfe,0x00);
	write_cmos_sensor(client,0x49, 0x70);  //AWB r gain
	write_cmos_sensor(client,0x4a, 0x40);  //AWB g gain
	write_cmos_sensor(client,0x4b, 0x5d);  //AWB b gain
	/////////////////////////////////////////////////////
	////////////////   ANALOG & CISCTL	 ////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0x03, 0x00);
	write_cmos_sensor(client,0x04, 0xfa);
	write_cmos_sensor(client,0x01, 0x41); //hb
	write_cmos_sensor(client,0x02, 0x12); //vb
	write_cmos_sensor(client,0x0f, 0x01);
	write_cmos_sensor(client,0x0d, 0x30);
	write_cmos_sensor(client,0x12, 0xc8);
	write_cmos_sensor(client,0x14, 0x54); //dark CFA
	write_cmos_sensor(client,0x15, 0x32); //1:sdark 0:ndark
	write_cmos_sensor(client,0x16, 0x04);
	write_cmos_sensor(client,0x17, 0x19);
	write_cmos_sensor(client,0x1d, 0xb9);
	write_cmos_sensor(client,0x1f, 0x15); //PAD_drv
	write_cmos_sensor(client,0x7a, 0x00);
	write_cmos_sensor(client,0x7b, 0x14);
	write_cmos_sensor(client,0x7d, 0x36);
	write_cmos_sensor(client,0xfe, 0x10);  //add by 20160217 CISCTL rst [4]
	/////////////////////////////////////////////////////
	//////////////////////   ISP   //////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0x20, 0x7e);
	write_cmos_sensor(client,0x22, 0xb8);
	write_cmos_sensor(client,0x24, 0x54); //output_format
	write_cmos_sensor(client,0x26, 0x87); //[5]Y_switch [4]UV_switch [2]skin_en
	//write_cmos_sensor(client,0x29, 0x10);// disable isp quiet mode

	write_cmos_sensor(client,0x39, 0x00); //crop window
	write_cmos_sensor(client,0x3a, 0x80); 
	write_cmos_sensor(client,0x3b, 0x01);  //width
	write_cmos_sensor(client,0x3c, 0x40); 
	write_cmos_sensor(client,0x3e, 0xf0); //height
	/////////////////////////////////////////////////////
	//////////////////////   BLK   //////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0x2a, 0x2f);
	write_cmos_sensor(client,0x37, 0x46); //[4:0]blk_select_row

	/////////////////////////////////////////////////////
	//////////////////////   GAIN   /////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0x3f, 0x18); //global gain 20160901


	/////////////////////////////////////////////////////
	//////////////////////   DNDD   /////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0x50, 0x3c); 
	write_cmos_sensor(client,0x52, 0x4f); 
	write_cmos_sensor(client,0x53, 0x81);
	write_cmos_sensor(client,0x54, 0x43);
	write_cmos_sensor(client,0x56, 0x78); 
	write_cmos_sensor(client,0x57, 0xaa);//20160901 
	write_cmos_sensor(client,0x58, 0xff);//20160901

	/////////////////////////////////////////////////////
	//////////////////////   ASDE   /////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0x5b, 0x60); //dd&ee th
	write_cmos_sensor(client,0x5c, 0x80); //60/OT_th
	write_cmos_sensor(client,0xab, 0x28); 
	write_cmos_sensor(client,0xac, 0xb5);

	/////////////////////////////////////////////////////
	/////////////////////   INTPEE   ////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0x60, 0x45); 
	write_cmos_sensor(client,0x62, 0x68); //20160901
	write_cmos_sensor(client,0x63, 0x13); //edge effect
	write_cmos_sensor(client,0x64, 0x43);

	/////////////////////////////////////////////////////
	//////////////////////   CC   ///////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0x65, 0x13); //Y
	write_cmos_sensor(client,0x66, 0x26);
	write_cmos_sensor(client,0x67, 0x07);
	write_cmos_sensor(client,0x68, 0xf5); //Cb
	write_cmos_sensor(client,0x69, 0xea);
	write_cmos_sensor(client,0x6a, 0x21);
	write_cmos_sensor(client,0x6b, 0x21); //Cr
	write_cmos_sensor(client,0x6c, 0xe4);
	write_cmos_sensor(client,0x6d, 0xfb);

	/////////////////////////////////////////////////////
	//////////////////////   YCP   //////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0x81, 0x30); //cb
	write_cmos_sensor(client,0x82, 0x30); //cr
	write_cmos_sensor(client,0x83, 0x4a); //luma contrast
	write_cmos_sensor(client,0x85, 0x06);  //luma offset
	write_cmos_sensor(client,0x8d, 0x78); //edge dec sa
	write_cmos_sensor(client,0x8e, 0x25); //autogray

	/////////////////////////////////////////////////////
	//////////////////////   AEC   //////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0x90, 0x38);//20160901
	write_cmos_sensor(client,0x92, 0x50); //target
	write_cmos_sensor(client,0x9d, 0x32);//STEP
	write_cmos_sensor(client,0x9e, 0x61);//[7:4]margin  10fps
	write_cmos_sensor(client,0x9f, 0xf4);
	write_cmos_sensor(client,0xa3, 0x28); //pregain
	write_cmos_sensor(client,0xa4, 0x01); 

	/////////////////////////////////////////////////////
	//////////////////////   AWB   //////////////////////
	/////////////////////////////////////////////////////
#if 0//AWB2
	write_cmos_sensor(client,0xb0, 0xf2); //Y_to_C_diff
	write_cmos_sensor(client,0xb1, 0x10); //Y_to_C_diff
	write_cmos_sensor(client,0xb2, 0x08); //AWB_Y_to_C_diff_big
	write_cmos_sensor(client,0xb3, 0x30); //C_max   //20
	write_cmos_sensor(client,0xb4, 0x40); 
	write_cmos_sensor(client,0xb5, 0x20); //inter
	write_cmos_sensor(client,0xb6, 0x34); //inter2
	write_cmos_sensor(client,0xb7, 0x48); //AWB_C_inter3	 18
	write_cmos_sensor(client,0xba, 0x40);   // big c  20
	write_cmos_sensor(client,0xbb, 0x71); //62//AWB adjust   72
	write_cmos_sensor(client,0xbd, 0x7a); //R_limit  80
	write_cmos_sensor(client,0xbe, 0x40); //G_limit   58
	write_cmos_sensor(client,0xbf, 0x80); //B_limit    a0	
#else
	write_cmos_sensor(client,0xb1, 0x1e); //Y_to_C_diff
	write_cmos_sensor(client,0xb3, 0x20); //C_max
	write_cmos_sensor(client,0xbd, 0x70); //R_limit
	write_cmos_sensor(client,0xbe, 0x58); //G_limit
	write_cmos_sensor(client,0xbf, 0xa0); //B_limit

	write_cmos_sensor(client,0xfe, 0x00);	//20160901 update for AWB
	write_cmos_sensor(client,0x43, 0xa8); 
	write_cmos_sensor(client,0xb0, 0xf2); 
	write_cmos_sensor(client,0xb5, 0x40); 
	write_cmos_sensor(client,0xb8, 0x05); 
	write_cmos_sensor(client,0xba, 0x60); 	
#endif

	/////////////////////////////////////////////////////
	////////////////////   Banding   ////////////////////
	/////////////////////////////////////////////////////
	//write_cmos_sensor(client,0x01, 0x41); //hb
	//write_cmos_sensor(client,0x02, 0x12); //vb
	//write_cmos_sensor(client,0x0f, 0x01);
	//write_cmos_sensor(client,0x9d, 0x32); //step
	//write_cmos_sensor(client,0x9e, 0x61); //[7:4]margin  10fps
	//write_cmos_sensor(client,0x9f, 0xf4); 
	/////////////////////////////////////////////////////
	//////////////////////   SPI   //////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0xfe,0x02);
	write_cmos_sensor(client,0x01, 0x01); //spi enable
	write_cmos_sensor(client,0x02, 0x02);   //LSB & Falling edge sample; ddr disable
	write_cmos_sensor(client,0x03, 0x20);	//1-wire
	write_cmos_sensor(client,0x04, 0x20);   //[4] master_outformat
	write_cmos_sensor(client,0x0a, 0x00);   //Data ID, 0x00-YUV422, 0x01-RGB565
	write_cmos_sensor(client,0x13, 0x10);
	write_cmos_sensor(client,0x24, 0x00); //[1]sck_always [0]BT656
	write_cmos_sensor(client,0x28, 0x03); //clock_div_spi
	
	write_cmos_sensor(client,0xfe,0x00);
	////////////////////////////////////////////////////
	///////////////////////output//////////////////////
	///////////////////////////////////////////////////
	write_cmos_sensor(client,0x22, 0xf8); //open awb
	write_cmos_sensor(client,0xf1, 0x03); //output enable
	
}

static void GC2145_Sensor_Init(struct i2c_client *client)
{
	write_cmos_sensor(client,0xfe, 0xf0);
	write_cmos_sensor(client,0xfe, 0xf0);
	write_cmos_sensor(client,0xfe, 0xf0);
	write_cmos_sensor(client,0xfc, 0x06);
	write_cmos_sensor(client,0xf6, 0x00);
	write_cmos_sensor(client,0xf7, 0x1d);
	write_cmos_sensor(client,0xf8, 0x84);
	write_cmos_sensor(client,0xfa, 0x00);
	write_cmos_sensor(client,0xf9, 0x8e);
	write_cmos_sensor(client,0xf2, 0x00);
	/////////////////////////////////////////////////
	//////////////////ISP reg//////////////////////
	////////////////////////////////////////////////////
	write_cmos_sensor(client,0xfe , 0x00);
	write_cmos_sensor(client,0x03 , 0x04);
	write_cmos_sensor(client,0x04 , 0xe2);
	write_cmos_sensor(client,0x09 , 0x00);
	write_cmos_sensor(client,0x0a , 0x00);
	write_cmos_sensor(client,0x0b , 0x00);
	write_cmos_sensor(client,0x0c , 0x00);
	write_cmos_sensor(client,0x0d , 0x04);
	write_cmos_sensor(client,0x0e , 0xc0);
	write_cmos_sensor(client,0x0f , 0x06);
	write_cmos_sensor(client,0x10 , 0x52);
	write_cmos_sensor(client,0x12 , 0x2e);
	write_cmos_sensor(client,0x17 , 0x14); //mirror
	write_cmos_sensor(client,0x18 , 0x22);
	write_cmos_sensor(client,0x19 , 0x0e);
	write_cmos_sensor(client,0x1a , 0x01);
	write_cmos_sensor(client,0x1b , 0x4b);
	write_cmos_sensor(client,0x1c , 0x07);
	write_cmos_sensor(client,0x1d , 0x10);
	write_cmos_sensor(client,0x1e , 0x88);
	write_cmos_sensor(client,0x1f , 0x78);
	write_cmos_sensor(client,0x20 , 0x03);
	write_cmos_sensor(client,0x21 , 0x40);
	write_cmos_sensor(client,0x22 , 0xdc); 
	write_cmos_sensor(client,0x24 , 0x16);
	write_cmos_sensor(client,0x25 , 0x01);
	write_cmos_sensor(client,0x26 , 0x10);
	write_cmos_sensor(client,0x2d , 0x60);
	write_cmos_sensor(client,0x30 , 0x01);
	write_cmos_sensor(client,0x31 , 0x90);
	write_cmos_sensor(client,0x33 , 0x06);
	write_cmos_sensor(client,0x34 , 0x01);
	/////////////////////////////////////////////////
	//////////////////ISP reg////////////////////
	/////////////////////////////////////////////////
	write_cmos_sensor(client,0xfe , 0x00);
	write_cmos_sensor(client,0x80 , 0x7f);
	write_cmos_sensor(client,0x81 , 0x26);
	write_cmos_sensor(client,0x82 , 0xfa);
	write_cmos_sensor(client,0x83 , 0x00);
	write_cmos_sensor(client,0x84 , 0x03); 
	write_cmos_sensor(client,0x86 , 0x02);
	write_cmos_sensor(client,0x88 , 0x03);
	write_cmos_sensor(client,0x89 , 0x03);
	write_cmos_sensor(client,0x85 , 0x08); 
	write_cmos_sensor(client,0x8a , 0x00);
	write_cmos_sensor(client,0x8b , 0x00);
	write_cmos_sensor(client,0xb0 , 0x55);
	write_cmos_sensor(client,0xc3 , 0x00);
	write_cmos_sensor(client,0xc4 , 0x80);
	write_cmos_sensor(client,0xc5 , 0x90);
	write_cmos_sensor(client,0xc6 , 0x3b);
	write_cmos_sensor(client,0xc7 , 0x46);
	write_cmos_sensor(client,0xec , 0x06);
	write_cmos_sensor(client,0xed , 0x04);
	write_cmos_sensor(client,0xee , 0x60);
	write_cmos_sensor(client,0xef , 0x90);
	write_cmos_sensor(client,0xb6 , 0x01);
	write_cmos_sensor(client,0x90 , 0x01);
	write_cmos_sensor(client,0x91 , 0x00);
	write_cmos_sensor(client,0x92 , 0x00);
	write_cmos_sensor(client,0x93 , 0x00);
	write_cmos_sensor(client,0x94 , 0x00);
	write_cmos_sensor(client,0x95 , 0x04);
	write_cmos_sensor(client,0x96 , 0xb0);
	write_cmos_sensor(client,0x97 , 0x06);
	write_cmos_sensor(client,0x98 , 0x40);
	/////////////////////////////////////////
	/////////// BLK ////////////////////////
	/////////////////////////////////////////
	write_cmos_sensor(client,0xfe , 0x00);
	write_cmos_sensor(client,0x40 , 0x42);
	write_cmos_sensor(client,0x41 , 0x00);
	write_cmos_sensor(client,0x43 , 0x5b); 
	write_cmos_sensor(client,0x5e , 0x00); 
	write_cmos_sensor(client,0x5f , 0x00);
	write_cmos_sensor(client,0x60 , 0x00); 
	write_cmos_sensor(client,0x61 , 0x00); 
	write_cmos_sensor(client,0x62 , 0x00);
	write_cmos_sensor(client,0x63 , 0x00); 
	write_cmos_sensor(client,0x64 , 0x00); 
	write_cmos_sensor(client,0x65 , 0x00); 
	write_cmos_sensor(client,0x66 , 0x20);
	write_cmos_sensor(client,0x67 , 0x20); 
	write_cmos_sensor(client,0x68 , 0x20); 
	write_cmos_sensor(client,0x69 , 0x20); 
	write_cmos_sensor(client,0x76 , 0x00);                                  
	write_cmos_sensor(client,0x6a , 0x08); 
	write_cmos_sensor(client,0x6b , 0x08); 
	write_cmos_sensor(client,0x6c , 0x08); 
	write_cmos_sensor(client,0x6d , 0x08); 
	write_cmos_sensor(client,0x6e , 0x08); 
	write_cmos_sensor(client,0x6f , 0x08); 
	write_cmos_sensor(client,0x70 , 0x08); 
	write_cmos_sensor(client,0x71 , 0x08);   
	write_cmos_sensor(client,0x76 , 0x00);
	write_cmos_sensor(client,0x72 , 0xf0);
	write_cmos_sensor(client,0x7e , 0x3c);
	write_cmos_sensor(client,0x7f , 0x00);
	write_cmos_sensor(client,0xfe , 0x02);
	write_cmos_sensor(client,0x48 , 0x15);
	write_cmos_sensor(client,0x49 , 0x00);
	write_cmos_sensor(client,0x4b , 0x0b);
	write_cmos_sensor(client,0xfe , 0x00);
	////////////////////////////////////////
	/////////// AEC ////////////////////////
	////////////////////////////////////////
	write_cmos_sensor(client,0xfe , 0x01);
	write_cmos_sensor(client,0x01 , 0x04);
	write_cmos_sensor(client,0x02 , 0xc0);
	write_cmos_sensor(client,0x03 , 0x04);
	write_cmos_sensor(client,0x04 , 0x90);
	write_cmos_sensor(client,0x05 , 0x30);
	write_cmos_sensor(client,0x06 , 0x90);
	write_cmos_sensor(client,0x07 , 0x30);
	write_cmos_sensor(client,0x08 , 0x80);
	write_cmos_sensor(client,0x09 , 0x00);
	write_cmos_sensor(client,0x0a , 0x82);
	write_cmos_sensor(client,0x0b , 0x11);
	write_cmos_sensor(client,0x0c , 0x10);
	write_cmos_sensor(client,0x11 , 0x10);
	write_cmos_sensor(client,0x13 , 0x7b);
	write_cmos_sensor(client,0x17 , 0x00);
	write_cmos_sensor(client,0x1c , 0x11);
	write_cmos_sensor(client,0x1e , 0x61);
	write_cmos_sensor(client,0x1f , 0x35);
	write_cmos_sensor(client,0x20 , 0x40);
	write_cmos_sensor(client,0x22 , 0x40);
	write_cmos_sensor(client,0x23 , 0x20);
	write_cmos_sensor(client,0xfe , 0x02);
	write_cmos_sensor(client,0x0f , 0x04);
	write_cmos_sensor(client,0xfe , 0x01);
	write_cmos_sensor(client,0x12 , 0x35);
	write_cmos_sensor(client,0x15 , 0xb0);
	write_cmos_sensor(client,0x10 , 0x31);
	write_cmos_sensor(client,0x3e , 0x28);
	write_cmos_sensor(client,0x3f , 0xb0);
	write_cmos_sensor(client,0x40 , 0x90);
	write_cmos_sensor(client,0x41 , 0x0f);
	
	/////////////////////////////
	//////// INTPEE /////////////
	/////////////////////////////
	write_cmos_sensor(client,0xfe , 0x02);
	write_cmos_sensor(client,0x90 , 0x6c);
	write_cmos_sensor(client,0x91 , 0x03);
	write_cmos_sensor(client,0x92 , 0xcb);
	write_cmos_sensor(client,0x94 , 0x33);
	write_cmos_sensor(client,0x95 , 0x84);
	write_cmos_sensor(client,0x97 , 0x65);
	write_cmos_sensor(client,0xa2 , 0x11);
	write_cmos_sensor(client,0xfe , 0x00);
	/////////////////////////////
	//////// DNDD///////////////
	/////////////////////////////
	write_cmos_sensor(client,0xfe , 0x02);
	write_cmos_sensor(client,0x80 , 0xc1);
	write_cmos_sensor(client,0x81 , 0x08);
	write_cmos_sensor(client,0x82 , 0x05);
	write_cmos_sensor(client,0x83 , 0x08);
	write_cmos_sensor(client,0x84 , 0x0a);
	write_cmos_sensor(client,0x86 , 0xf0);
	write_cmos_sensor(client,0x87 , 0x50);
	write_cmos_sensor(client,0x88 , 0x15);
	write_cmos_sensor(client,0x89 , 0xb0);
	write_cmos_sensor(client,0x8a , 0x30);
	write_cmos_sensor(client,0x8b , 0x10);
	/////////////////////////////////////////
	/////////// ASDE ////////////////////////
	/////////////////////////////////////////
	write_cmos_sensor(client,0xfe , 0x01);
	write_cmos_sensor(client,0x21 , 0x04);
	write_cmos_sensor(client,0xfe , 0x02);
	write_cmos_sensor(client,0xa3 , 0x50);
	write_cmos_sensor(client,0xa4 , 0x20);
	write_cmos_sensor(client,0xa5 , 0x40);
	write_cmos_sensor(client,0xa6 , 0x80);
	write_cmos_sensor(client,0xab , 0x40);
	write_cmos_sensor(client,0xae , 0x0c);
	write_cmos_sensor(client,0xb3 , 0x46);
	write_cmos_sensor(client,0xb4 , 0x64);
	write_cmos_sensor(client,0xb6 , 0x38);
	write_cmos_sensor(client,0xb7 , 0x01);
	write_cmos_sensor(client,0xb9 , 0x2b);
	write_cmos_sensor(client,0x3c , 0x04);
	write_cmos_sensor(client,0x3d , 0x15);
	write_cmos_sensor(client,0x4b , 0x06);
	write_cmos_sensor(client,0x4c , 0x20);
	write_cmos_sensor(client,0xfe , 0x00);
	/////////////////////////////////////////
	/////////// GAMMA   ////////////////////////
	/////////////////////////////////////////
	
	///////////////////gamma1////////////////////
	#if 1
	write_cmos_sensor(client,0xfe , 0x02);
	write_cmos_sensor(client,0x10 , 0x09);
	write_cmos_sensor(client,0x11 , 0x0d);
	write_cmos_sensor(client,0x12 , 0x13);
	write_cmos_sensor(client,0x13 , 0x19);
	write_cmos_sensor(client,0x14 , 0x27);
	write_cmos_sensor(client,0x15 , 0x37);
	write_cmos_sensor(client,0x16 , 0x45);
	write_cmos_sensor(client,0x17 , 0x53);
	write_cmos_sensor(client,0x18 , 0x69);
	write_cmos_sensor(client,0x19 , 0x7d);
	write_cmos_sensor(client,0x1a , 0x8f);
	write_cmos_sensor(client,0x1b , 0x9d);
	write_cmos_sensor(client,0x1c , 0xa9);
	write_cmos_sensor(client,0x1d , 0xbd);
	write_cmos_sensor(client,0x1e , 0xcd);
	write_cmos_sensor(client,0x1f , 0xd9);
	write_cmos_sensor(client,0x20 , 0xe3);
	write_cmos_sensor(client,0x21 , 0xea);
	write_cmos_sensor(client,0x22 , 0xef);
	write_cmos_sensor(client,0x23 , 0xf5);
	write_cmos_sensor(client,0x24 , 0xf9);
	write_cmos_sensor(client,0x25 , 0xff);
	#else                               
	write_cmos_sensor(client,0xfe , 0x02);
	write_cmos_sensor(client,0x10 , 0x0a);
	write_cmos_sensor(client,0x11 , 0x12);
	write_cmos_sensor(client,0x12 , 0x19);
	write_cmos_sensor(client,0x13 , 0x1f);
	write_cmos_sensor(client,0x14 , 0x2c);
	write_cmos_sensor(client,0x15 , 0x38);
	write_cmos_sensor(client,0x16 , 0x42);
	write_cmos_sensor(client,0x17 , 0x4e);
	write_cmos_sensor(client,0x18 , 0x63);
	write_cmos_sensor(client,0x19 , 0x76);
	write_cmos_sensor(client,0x1a , 0x87);
	write_cmos_sensor(client,0x1b , 0x96);
	write_cmos_sensor(client,0x1c , 0xa2);
	write_cmos_sensor(client,0x1d , 0xb8);
	write_cmos_sensor(client,0x1e , 0xcb);
	write_cmos_sensor(client,0x1f , 0xd8);
	write_cmos_sensor(client,0x20 , 0xe2);
	write_cmos_sensor(client,0x21 , 0xe9);
	write_cmos_sensor(client,0x22 , 0xf0);
	write_cmos_sensor(client,0x23 , 0xf8);
	write_cmos_sensor(client,0x24 , 0xfd);
	write_cmos_sensor(client,0x25 , 0xff);
	write_cmos_sensor(client,0xfe , 0x00);     
	#endif 
	write_cmos_sensor(client,0xfe , 0x00);     
	write_cmos_sensor(client,0xc6 , 0x20);
	write_cmos_sensor(client,0xc7 , 0x2b);
	///////////////////gamma2////////////////////
	#if 1
	write_cmos_sensor(client,0xfe , 0x02);
	write_cmos_sensor(client,0x26 , 0x0f);
	write_cmos_sensor(client,0x27 , 0x14);
	write_cmos_sensor(client,0x28 , 0x19);
	write_cmos_sensor(client,0x29 , 0x1e);
	write_cmos_sensor(client,0x2a , 0x27);
	write_cmos_sensor(client,0x2b , 0x33);
	write_cmos_sensor(client,0x2c , 0x3b);
	write_cmos_sensor(client,0x2d , 0x45);
	write_cmos_sensor(client,0x2e , 0x59);
	write_cmos_sensor(client,0x2f , 0x69);
	write_cmos_sensor(client,0x30 , 0x7c);
	write_cmos_sensor(client,0x31 , 0x89);
	write_cmos_sensor(client,0x32 , 0x98);
	write_cmos_sensor(client,0x33 , 0xae);
	write_cmos_sensor(client,0x34 , 0xc0);
	write_cmos_sensor(client,0x35 , 0xcf);
	write_cmos_sensor(client,0x36 , 0xda);
	write_cmos_sensor(client,0x37 , 0xe2);
	write_cmos_sensor(client,0x38 , 0xe9);
	write_cmos_sensor(client,0x39 , 0xf3);
	write_cmos_sensor(client,0x3a , 0xf9);
	write_cmos_sensor(client,0x3b , 0xff);
	#else
	////Gamma outdoor
	write_cmos_sensor(client,0xfe , 0x02);
	write_cmos_sensor(client,0x26 , 0x17);
	write_cmos_sensor(client,0x27 , 0x18);
	write_cmos_sensor(client,0x28 , 0x1c);
	write_cmos_sensor(client,0x29 , 0x20);
	write_cmos_sensor(client,0x2a , 0x28);
	write_cmos_sensor(client,0x2b , 0x34);
	write_cmos_sensor(client,0x2c , 0x40);
	write_cmos_sensor(client,0x2d , 0x49);
	write_cmos_sensor(client,0x2e , 0x5b);
	write_cmos_sensor(client,0x2f , 0x6d);
	write_cmos_sensor(client,0x30 , 0x7d);
	write_cmos_sensor(client,0x31 , 0x89);
	write_cmos_sensor(client,0x32 , 0x97);
	write_cmos_sensor(client,0x33 , 0xac);
	write_cmos_sensor(client,0x34 , 0xc0);
	write_cmos_sensor(client,0x35 , 0xcf);
	write_cmos_sensor(client,0x36 , 0xda);
	write_cmos_sensor(client,0x37 , 0xe5);
	write_cmos_sensor(client,0x38 , 0xec);
	write_cmos_sensor(client,0x39 , 0xf8);
	write_cmos_sensor(client,0x3a , 0xfd);
	write_cmos_sensor(client,0x3b , 0xff);
	#endif
	/////////////////////////////////////////////// 
	///////////YCP /////////////////////// 
	/////////////////////////////////////////////// 
	write_cmos_sensor(client,0xfe , 0x02);
	write_cmos_sensor(client,0xd1 , 0x32);
	write_cmos_sensor(client,0xd2 , 0x32);
	write_cmos_sensor(client,0xd3 , 0x40);
	write_cmos_sensor(client,0xd6 , 0xf0);
	write_cmos_sensor(client,0xd7 , 0x10);
	write_cmos_sensor(client,0xd8 , 0xda);
	write_cmos_sensor(client,0xdd , 0x14);
	write_cmos_sensor(client,0xde , 0x86);
	write_cmos_sensor(client,0xed , 0x80);
	write_cmos_sensor(client,0xee , 0x00);
	write_cmos_sensor(client,0xef , 0x3f);
	write_cmos_sensor(client,0xd8 , 0xd8);
	///////////////////abs/////////////////
	write_cmos_sensor(client,0xfe , 0x01);
	write_cmos_sensor(client,0x9f , 0x40);
	/////////////////////////////////////////////
	//////////////////////// LSC ///////////////
	//////////////////////////////////////////
	write_cmos_sensor(client,0xfe , 0x01);
	write_cmos_sensor(client,0xc2 , 0x14);
	write_cmos_sensor(client,0xc3 , 0x0d);
	write_cmos_sensor(client,0xc4 , 0x0c);
	write_cmos_sensor(client,0xc8 , 0x15);
	write_cmos_sensor(client,0xc9 , 0x0d);
	write_cmos_sensor(client,0xca , 0x0a);
	write_cmos_sensor(client,0xbc , 0x24);
	write_cmos_sensor(client,0xbd , 0x10);
	write_cmos_sensor(client,0xbe , 0x0b);
	write_cmos_sensor(client,0xb6 , 0x25);
	write_cmos_sensor(client,0xb7 , 0x16);
	write_cmos_sensor(client,0xb8 , 0x15);
	write_cmos_sensor(client,0xc5 , 0x00);
	write_cmos_sensor(client,0xc6 , 0x00);
	write_cmos_sensor(client,0xc7 , 0x00);
	write_cmos_sensor(client,0xcb , 0x00);
	write_cmos_sensor(client,0xcc , 0x00);
	write_cmos_sensor(client,0xcd , 0x00);
	write_cmos_sensor(client,0xbf , 0x07);
	write_cmos_sensor(client,0xc0 , 0x00);
	write_cmos_sensor(client,0xc1 , 0x00);
	write_cmos_sensor(client,0xb9 , 0x00);
	write_cmos_sensor(client,0xba , 0x00);
	write_cmos_sensor(client,0xbb , 0x00);
	write_cmos_sensor(client,0xaa , 0x01);
	write_cmos_sensor(client,0xab , 0x01);
	write_cmos_sensor(client,0xac , 0x00);
	write_cmos_sensor(client,0xad , 0x05);
	write_cmos_sensor(client,0xae , 0x06);
	write_cmos_sensor(client,0xaf , 0x0e);
	write_cmos_sensor(client,0xb0 , 0x0b);
	write_cmos_sensor(client,0xb1 , 0x07);
	write_cmos_sensor(client,0xb2 , 0x06);
	write_cmos_sensor(client,0xb3 , 0x17);
	write_cmos_sensor(client,0xb4 , 0x0e);
	write_cmos_sensor(client,0xb5 , 0x0e);
	write_cmos_sensor(client,0xd0 , 0x09);
	write_cmos_sensor(client,0xd1 , 0x00);
	write_cmos_sensor(client,0xd2 , 0x00);
	write_cmos_sensor(client,0xd6 , 0x08);
	write_cmos_sensor(client,0xd7 , 0x00);
	write_cmos_sensor(client,0xd8 , 0x00);
	write_cmos_sensor(client,0xd9 , 0x00);
	write_cmos_sensor(client,0xda , 0x00);
	write_cmos_sensor(client,0xdb , 0x00);
	write_cmos_sensor(client,0xd3 , 0x0a);
	write_cmos_sensor(client,0xd4 , 0x00);
	write_cmos_sensor(client,0xd5 , 0x00);
	write_cmos_sensor(client,0xa4 , 0x00);
	write_cmos_sensor(client,0xa5 , 0x00);
	write_cmos_sensor(client,0xa6 , 0x77);
	write_cmos_sensor(client,0xa7 , 0x77);
	write_cmos_sensor(client,0xa8 , 0x77);
	write_cmos_sensor(client,0xa9 , 0x77);
	write_cmos_sensor(client,0xa1 , 0x80);
	write_cmos_sensor(client,0xa2 , 0x80);
	               
	write_cmos_sensor(client,0xfe , 0x01);
	write_cmos_sensor(client,0xdf , 0x0d);
	write_cmos_sensor(client,0xdc , 0x25);
	write_cmos_sensor(client,0xdd , 0x30);
	write_cmos_sensor(client,0xe0 , 0x77);
	write_cmos_sensor(client,0xe1 , 0x80);
	write_cmos_sensor(client,0xe2 , 0x77);
	write_cmos_sensor(client,0xe3 , 0x90);
	write_cmos_sensor(client,0xe6 , 0x90);
	write_cmos_sensor(client,0xe7 , 0xa0);
	write_cmos_sensor(client,0xe8 , 0x90);
	write_cmos_sensor(client,0xe9 , 0xa0);                                      
	write_cmos_sensor(client,0xfe , 0x00);
	///////////////////////////////////////////////
	/////////// AWB////////////////////////
	///////////////////////////////////////////////
	write_cmos_sensor(client,0xfe , 0x01);
	write_cmos_sensor(client,0x4f , 0x00);
	write_cmos_sensor(client,0x4f , 0x00);
	write_cmos_sensor(client,0x4b , 0x01);
	write_cmos_sensor(client,0x4f , 0x00);
	         
	write_cmos_sensor(client,0x4c , 0x01); // D75
	write_cmos_sensor(client,0x4d , 0x71);
	write_cmos_sensor(client,0x4e , 0x01);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0x91);
	write_cmos_sensor(client,0x4e , 0x01);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0x70);
	write_cmos_sensor(client,0x4e , 0x01);
	         
	write_cmos_sensor(client,0x4c , 0x01); // D65
	write_cmos_sensor(client,0x4d , 0x90);
	write_cmos_sensor(client,0x4e , 0x02);                                    
	         
	         
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0xb0);
	write_cmos_sensor(client,0x4e , 0x02);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0x8f);
	write_cmos_sensor(client,0x4e , 0x02);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0x6f);
	write_cmos_sensor(client,0x4e , 0x02);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0xaf);
	write_cmos_sensor(client,0x4e , 0x02);
	         
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0xd0);
	write_cmos_sensor(client,0x4e , 0x02);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0xf0);
	write_cmos_sensor(client,0x4e , 0x02);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0xcf);
	write_cmos_sensor(client,0x4e , 0x02);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0xef);
	write_cmos_sensor(client,0x4e , 0x02);
	         
	write_cmos_sensor(client,0x4c , 0x01);//D50
	write_cmos_sensor(client,0x4d , 0x6e);
	write_cmos_sensor(client,0x4e , 0x03);
	write_cmos_sensor(client,0x4c , 0x01); 
	write_cmos_sensor(client,0x4d , 0x8e);
	write_cmos_sensor(client,0x4e , 0x03);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0xae);
	write_cmos_sensor(client,0x4e , 0x03);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0xce);
	write_cmos_sensor(client,0x4e , 0x03);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0x4d);
	write_cmos_sensor(client,0x4e , 0x03);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0x6d);
	write_cmos_sensor(client,0x4e , 0x03);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0x8d);
	write_cmos_sensor(client,0x4e , 0x03);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0xad);
	write_cmos_sensor(client,0x4e , 0x03);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0xcd);
	write_cmos_sensor(client,0x4e , 0x03);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0x4c);
	write_cmos_sensor(client,0x4e , 0x03);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0x6c);
	write_cmos_sensor(client,0x4e , 0x03);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0x8c);
	write_cmos_sensor(client,0x4e , 0x03);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0xac);
	write_cmos_sensor(client,0x4e , 0x03);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0xcc);
	write_cmos_sensor(client,0x4e , 0x03);
	         
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0xcb);
	write_cmos_sensor(client,0x4e , 0x03);
	         
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0x4b);
	write_cmos_sensor(client,0x4e , 0x03);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0x6b);
	write_cmos_sensor(client,0x4e , 0x03);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0x8b);
	write_cmos_sensor(client,0x4e , 0x03);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0xab);
	write_cmos_sensor(client,0x4e , 0x03);
	         
	write_cmos_sensor(client,0x4c , 0x01);//CWF
	write_cmos_sensor(client,0x4d , 0x8a);
	write_cmos_sensor(client,0x4e , 0x04);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0xaa);
	write_cmos_sensor(client,0x4e , 0x04);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0xca);
	write_cmos_sensor(client,0x4e , 0x04);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0xca);
	write_cmos_sensor(client,0x4e , 0x04);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0xc9);
	write_cmos_sensor(client,0x4e , 0x04);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0x8a);
	write_cmos_sensor(client,0x4e , 0x04);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0x89);
	write_cmos_sensor(client,0x4e , 0x04);
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0xa9);
	write_cmos_sensor(client,0x4e , 0x04);
	         
	         
	         
	write_cmos_sensor(client,0x4c , 0x02);//tl84
	write_cmos_sensor(client,0x4d , 0x0b);
	write_cmos_sensor(client,0x4e , 0x05);
	write_cmos_sensor(client,0x4c , 0x02);
	write_cmos_sensor(client,0x4d , 0x0a);
	write_cmos_sensor(client,0x4e , 0x05);
	         
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0xeb);
	write_cmos_sensor(client,0x4e , 0x05);
	         
	write_cmos_sensor(client,0x4c , 0x01);
	write_cmos_sensor(client,0x4d , 0xea);
	write_cmos_sensor(client,0x4e , 0x05);
	                 
	write_cmos_sensor(client,0x4c , 0x02);
	write_cmos_sensor(client,0x4d , 0x09);
	write_cmos_sensor(client,0x4e , 0x05);
	write_cmos_sensor(client,0x4c , 0x02);
	write_cmos_sensor(client,0x4d , 0x29);
	write_cmos_sensor(client,0x4e , 0x05);
	                     
	write_cmos_sensor(client,0x4c , 0x02);
	write_cmos_sensor(client,0x4d , 0x2a);
	write_cmos_sensor(client,0x4e , 0x05);
	                      
	write_cmos_sensor(client,0x4c , 0x02);
	write_cmos_sensor(client,0x4d , 0x4a);
	write_cmos_sensor(client,0x4e , 0x05);
	
	//write_cmos_sensor(client,0x4c , 0x02); //A
	//write_cmos_sensor(client,0x4d , 0x6a);
	//write_cmos_sensor(client,0x4e , 0x06);
	
	write_cmos_sensor(client,0x4c , 0x02); 
	write_cmos_sensor(client,0x4d , 0x8a);
	write_cmos_sensor(client,0x4e , 0x06);
	                
	write_cmos_sensor(client,0x4c , 0x02);
	write_cmos_sensor(client,0x4d , 0x49);
	write_cmos_sensor(client,0x4e , 0x06);
	write_cmos_sensor(client,0x4c , 0x02);
	write_cmos_sensor(client,0x4d , 0x69);
	write_cmos_sensor(client,0x4e , 0x06);
	write_cmos_sensor(client,0x4c , 0x02);
	write_cmos_sensor(client,0x4d , 0x89);
	write_cmos_sensor(client,0x4e , 0x06);
	write_cmos_sensor(client,0x4c , 0x02);
	write_cmos_sensor(client,0x4d , 0xa9);
	write_cmos_sensor(client,0x4e , 0x06);
	               
	write_cmos_sensor(client,0x4c , 0x02);
	write_cmos_sensor(client,0x4d , 0x48);
	write_cmos_sensor(client,0x4e , 0x06);
	write_cmos_sensor(client,0x4c , 0x02);
	write_cmos_sensor(client,0x4d , 0x68);
	write_cmos_sensor(client,0x4e , 0x06);
	write_cmos_sensor(client,0x4c , 0x02);
	write_cmos_sensor(client,0x4d , 0x69);
	write_cmos_sensor(client,0x4e , 0x06);
	             
	write_cmos_sensor(client,0x4c , 0x02);//H
	write_cmos_sensor(client,0x4d , 0xca);
	write_cmos_sensor(client,0x4e , 0x07);
	write_cmos_sensor(client,0x4c , 0x02);
	write_cmos_sensor(client,0x4d , 0xc9);
	write_cmos_sensor(client,0x4e , 0x07);
	write_cmos_sensor(client,0x4c , 0x02);
	write_cmos_sensor(client,0x4d , 0xe9);
	write_cmos_sensor(client,0x4e , 0x07);
	write_cmos_sensor(client,0x4c , 0x03);
	write_cmos_sensor(client,0x4d , 0x09);
	write_cmos_sensor(client,0x4e , 0x07);
	write_cmos_sensor(client,0x4c , 0x02);
	write_cmos_sensor(client,0x4d , 0xc8);
	write_cmos_sensor(client,0x4e , 0x07);
	write_cmos_sensor(client,0x4c , 0x02);
	write_cmos_sensor(client,0x4d , 0xe8);
	write_cmos_sensor(client,0x4e , 0x07);
	write_cmos_sensor(client,0x4c , 0x02);
	write_cmos_sensor(client,0x4d , 0xa7);
	write_cmos_sensor(client,0x4e , 0x07);
	write_cmos_sensor(client,0x4c , 0x02);
	write_cmos_sensor(client,0x4d , 0xc7);
	write_cmos_sensor(client,0x4e , 0x07);
	write_cmos_sensor(client,0x4c , 0x02);
	write_cmos_sensor(client,0x4d , 0xe7);
	write_cmos_sensor(client,0x4e , 0x07);
	write_cmos_sensor(client,0x4c , 0x03);
	write_cmos_sensor(client,0x4d , 0x07);
	write_cmos_sensor(client,0x4e , 0x07);
	
	write_cmos_sensor(client,0x4f , 0x01);
	write_cmos_sensor(client,0x50 , 0x80);
	write_cmos_sensor(client,0x51 , 0xa8);
	write_cmos_sensor(client,0x52 , 0x47);
	write_cmos_sensor(client,0x53 , 0x38);
	write_cmos_sensor(client,0x54 , 0xc7);
	write_cmos_sensor(client,0x56 , 0x0e);
	write_cmos_sensor(client,0x58 , 0x08);
	write_cmos_sensor(client,0x5b , 0x00);
	write_cmos_sensor(client,0x5c , 0x74);
	write_cmos_sensor(client,0x5d , 0x8b);
	write_cmos_sensor(client,0x61 , 0xdb);
	write_cmos_sensor(client,0x62 , 0xb8);
	write_cmos_sensor(client,0x63 , 0x86);
	write_cmos_sensor(client,0x64 , 0xc0);
	write_cmos_sensor(client,0x65 , 0x04);
	
	write_cmos_sensor(client,0x67 , 0xa8);
	write_cmos_sensor(client,0x68 , 0xb0);
	write_cmos_sensor(client,0x69 , 0x00);
	write_cmos_sensor(client,0x6a , 0xa8);
	write_cmos_sensor(client,0x6b , 0xb0);
	write_cmos_sensor(client,0x6c , 0xaf);
	write_cmos_sensor(client,0x6d , 0x8b);
	write_cmos_sensor(client,0x6e , 0x50);
	write_cmos_sensor(client,0x6f , 0x18);
	write_cmos_sensor(client,0x73 , 0xf0);
	write_cmos_sensor(client,0x70 , 0x0d);
	write_cmos_sensor(client,0x71 , 0x60);
	write_cmos_sensor(client,0x72 , 0x80);
	write_cmos_sensor(client,0x74 , 0x01);
	write_cmos_sensor(client,0x75 , 0x01);
	write_cmos_sensor(client,0x7f , 0x0c);
	write_cmos_sensor(client,0x76 , 0x70);
	write_cmos_sensor(client,0x77 , 0x58);
	write_cmos_sensor(client,0x78 , 0xa0);
	write_cmos_sensor(client,0x79 , 0x5e);
	write_cmos_sensor(client,0x7a , 0x54);
	write_cmos_sensor(client,0x7b , 0x58);                                      
	write_cmos_sensor(client,0xfe , 0x00);
	//////////////////////////////////////////
	///////////CC////////////////////////
	//////////////////////////////////////////
	write_cmos_sensor(client,0xfe , 0x02);
	write_cmos_sensor(client,0xc0 , 0x01);                                   
	write_cmos_sensor(client,0xc1 , 0x44);
	write_cmos_sensor(client,0xc2 , 0xfd);
	write_cmos_sensor(client,0xc3 , 0x04);
	write_cmos_sensor(client,0xc4 , 0xf0);
	write_cmos_sensor(client,0xc5 , 0x48);
	write_cmos_sensor(client,0xc6 , 0xfd);
	write_cmos_sensor(client,0xc7 , 0x46);
	write_cmos_sensor(client,0xc8 , 0xfd);
	write_cmos_sensor(client,0xc9 , 0x02);
	write_cmos_sensor(client,0xca , 0xe0);
	write_cmos_sensor(client,0xcb , 0x45);
	write_cmos_sensor(client,0xcc , 0xec);                         
	write_cmos_sensor(client,0xcd , 0x48);
	write_cmos_sensor(client,0xce , 0xf0);
	write_cmos_sensor(client,0xcf , 0xf0);
	write_cmos_sensor(client,0xe3 , 0x0c);
	write_cmos_sensor(client,0xe4 , 0x4b);
	write_cmos_sensor(client,0xe5 , 0xe0);
	//////////////////////////////////////////
	///////////ABS ////////////////////
	//////////////////////////////////////////
	write_cmos_sensor(client,0xfe , 0x01);
	write_cmos_sensor(client,0x9f , 0x40);
	write_cmos_sensor(client,0xfe , 0x00); 
	//////////////////////////////////////
	///////////  OUTPUT   ////////////////
	//////////////////////////////////////
	write_cmos_sensor(client,0xfe, 0x00);
	write_cmos_sensor(client,0xf2, 0x00);
	
	//////////////frame rate 50Hz/////////
	write_cmos_sensor(client,0xfe , 0x00);
	write_cmos_sensor(client,0x05 , 0x01);
	write_cmos_sensor(client,0x06 , 0x56);
	write_cmos_sensor(client,0x07 , 0x00);
	write_cmos_sensor(client,0x08 , 0x32);
	write_cmos_sensor(client,0xfe , 0x01);
	write_cmos_sensor(client,0x25 , 0x00);
	write_cmos_sensor(client,0x26 , 0xfa); 
	write_cmos_sensor(client,0x27 , 0x04); 
	write_cmos_sensor(client,0x28 , 0xe2); //20fps 
	write_cmos_sensor(client,0x29 , 0x06); 
	write_cmos_sensor(client,0x2a , 0xd6); //14fps 
	write_cmos_sensor(client,0x2b , 0x07); 
	write_cmos_sensor(client,0x2c , 0xd0); //12fps
	write_cmos_sensor(client,0x2d , 0x0b); 
	write_cmos_sensor(client,0x2e , 0xb8); //8fps
	write_cmos_sensor(client,0xfe , 0x00);
	
	///////////////dark sun////////////////////
	write_cmos_sensor(client,0xfe , 0x02);
	write_cmos_sensor(client,0x40 , 0xbf);
	write_cmos_sensor(client,0x46 , 0xcf);
	write_cmos_sensor(client,0xfe , 0x00);
	/////////////////////////////////////////////////////
	//////////////////////   MIPI   /////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0xfe, 0x03);
	write_cmos_sensor(client,0x02, 0x22);
	write_cmos_sensor(client,0x03, 0x10); // 0x12 20140821
	write_cmos_sensor(client,0x04, 0x10); // 0x01 
	write_cmos_sensor(client,0x05, 0x00);
	write_cmos_sensor(client,0x06, 0x88);
	#ifdef GC2145MIPI_2Lane
		write_cmos_sensor(client,0x01, 0x87);
		write_cmos_sensor(client,0x10, 0x95);
	#else
		write_cmos_sensor(client,0x01, 0x83);
		write_cmos_sensor(client,0x10, 0x94);
	#endif
	write_cmos_sensor(client,0x11, 0x1e);
	write_cmos_sensor(client,0x12, 0x80);
	write_cmos_sensor(client,0x13, 0x0c);
	write_cmos_sensor(client,0x15, 0x10);
	write_cmos_sensor(client,0x17, 0xf0);
	
	write_cmos_sensor(client,0x21, 0x10);
	write_cmos_sensor(client,0x22, 0x04);
	write_cmos_sensor(client,0x23, 0x10);
	write_cmos_sensor(client,0x24, 0x10);
	write_cmos_sensor(client,0x25, 0x10);
	write_cmos_sensor(client,0x26, 0x05);
	write_cmos_sensor(client,0x29, 0x03);
	write_cmos_sensor(client,0x2a, 0x0a);
	write_cmos_sensor(client,0x2b, 0x06);
	write_cmos_sensor(client,0xfe, 0x00);
	
}

void GC0310_Sensor_Init(struct i2c_client *client)
{
	write_cmos_sensor(client,0xfe,0xf0);
	write_cmos_sensor(client,0xfe,0xf0);
	write_cmos_sensor(client,0xfe,0x00);
	write_cmos_sensor(client,0xfc,0x0e);
	write_cmos_sensor(client,0xfc,0x0e);
	write_cmos_sensor(client,0xf2,0x80);
	write_cmos_sensor(client,0xf3,0x00);
	write_cmos_sensor(client,0xf7,0x1b);
	write_cmos_sensor(client,0xf8,0x04);  // from 03 to 04
	write_cmos_sensor(client,0xf9,0x8e);
	write_cmos_sensor(client,0xfa,0x11);
	/////////////////////////////////////////////////      
	///////////////////   MIPI   ////////////////////      
	/////////////////////////////////////////////////      
	write_cmos_sensor(client,0xfe,0x03);
	write_cmos_sensor(client,0x40,0x08);
	write_cmos_sensor(client,0x42,0x00);
	write_cmos_sensor(client,0x43,0x00);
	write_cmos_sensor(client,0x01,0x03);
	write_cmos_sensor(client,0x10,0x84);
                                        
	write_cmos_sensor(client,0x01,0x03);             
	write_cmos_sensor(client,0x02,0x00);             
	write_cmos_sensor(client,0x03,0x94);             
	write_cmos_sensor(client,0x04,0x01);            
	write_cmos_sensor(client,0x05,0x40);  // 40      20     
	write_cmos_sensor(client,0x06,0x80);             
	write_cmos_sensor(client,0x11,0x1e);             
	write_cmos_sensor(client,0x12,0x00);      
	write_cmos_sensor(client,0x13,0x05);             
	write_cmos_sensor(client,0x15,0x10);                                                                    
	write_cmos_sensor(client,0x21,0x10);             
	write_cmos_sensor(client,0x22,0x01);             
	write_cmos_sensor(client,0x23,0x10);                                             
	write_cmos_sensor(client,0x24,0x02);                                             
	write_cmos_sensor(client,0x25,0x10);                                             
	write_cmos_sensor(client,0x26,0x03);                                             
	write_cmos_sensor(client,0x29,0x02); //02                                            
	write_cmos_sensor(client,0x2a,0x0a);   //0a                                          
	write_cmos_sensor(client,0x2b,0x04);                                             
	write_cmos_sensor(client,0xfe,0x00);
        /////////////////////////////////////////////////
        /////////////////   CISCTL reg  /////////////////
        /////////////////////////////////////////////////
	write_cmos_sensor(client,0x00,0x2f);
	write_cmos_sensor(client,0x01,0x0f);
	write_cmos_sensor(client,0x02,0x04);
	write_cmos_sensor(client,0x03,0x04);
	write_cmos_sensor(client,0x04,0xd0);
	write_cmos_sensor(client,0x09,0x00);
	write_cmos_sensor(client,0x0a,0x00);
	write_cmos_sensor(client,0x0b,0x00);
	write_cmos_sensor(client,0x0c,0x06);
	write_cmos_sensor(client,0x0d,0x01);
	write_cmos_sensor(client,0x0e,0xe8);
	write_cmos_sensor(client,0x0f,0x02);
	write_cmos_sensor(client,0x10,0x88);
	write_cmos_sensor(client,0x16,0x00);
	write_cmos_sensor(client,0x17,0x14);
	write_cmos_sensor(client,0x18,0x1a);
	write_cmos_sensor(client,0x19,0x14);
	write_cmos_sensor(client,0x1b,0x48);
	write_cmos_sensor(client,0x1e,0x6b);
	write_cmos_sensor(client,0x1f,0x28);
	write_cmos_sensor(client,0x20,0x8b);  // from 89 to 8b
	write_cmos_sensor(client,0x21,0x49);
	write_cmos_sensor(client,0x22,0xb0);
	write_cmos_sensor(client,0x23,0x04);
	write_cmos_sensor(client,0x24,0x16);
	write_cmos_sensor(client,0x34,0x20);
        
        /////////////////////////////////////////////////
        ////////////////////   BLK   ////////////////////
        /////////////////////////////////////////////////
	write_cmos_sensor(client,0x26,0x23); 
	write_cmos_sensor(client,0x28,0xff); 
	write_cmos_sensor(client,0x29,0x00); 
	write_cmos_sensor(client,0x33,0x10); 
	write_cmos_sensor(client,0x37,0x20); 
	write_cmos_sensor(client,0x38,0x10); 
	write_cmos_sensor(client,0x47,0x80); 
	write_cmos_sensor(client,0x4e,0x66); 
	write_cmos_sensor(client,0xa8,0x02); 
	write_cmos_sensor(client,0xa9,0x80);
        
        /////////////////////////////////////////////////
        //////////////////   ISP reg  ///////////////////
        /////////////////////////////////////////////////
	write_cmos_sensor(client,0x40,0xff); 
	write_cmos_sensor(client,0x41,0x21); 
	write_cmos_sensor(client,0x42,0xcf); 
	write_cmos_sensor(client,0x44,0x01); // 02 yuv 
	write_cmos_sensor(client,0x45,0xa0); // from a8 - a4 a4-a0
	write_cmos_sensor(client,0x46,0x03); 
	write_cmos_sensor(client,0x4a,0x11);
	write_cmos_sensor(client,0x4b,0x01);
	write_cmos_sensor(client,0x4c,0x20); 
	write_cmos_sensor(client,0x4d,0x05); 
	write_cmos_sensor(client,0x4f,0x01);
	write_cmos_sensor(client,0x50,0x01); 
	write_cmos_sensor(client,0x55,0x01); 
	write_cmos_sensor(client,0x56,0xe0);
	write_cmos_sensor(client,0x57,0x02); 
	write_cmos_sensor(client,0x58,0x80);
        
        /////////////////////////////////////////////////  
        ///////////////////   GAIN   ////////////////////
        /////////////////////////////////////////////////
	write_cmos_sensor(client,0x70,0x70); 
	write_cmos_sensor(client,0x5a,0x84); 
	write_cmos_sensor(client,0x5b,0xc9); 
	write_cmos_sensor(client,0x5c,0xed); 
	write_cmos_sensor(client,0x77,0x74); 
	write_cmos_sensor(client,0x78,0x40); 
	write_cmos_sensor(client,0x79,0x5f); 
        
        ///////////////////////////////////////////////// 
        ///////////////////   DNDD  /////////////////////
        ///////////////////////////////////////////////// 
	write_cmos_sensor(client,0x82,0x1f); 
	write_cmos_sensor(client,0x83,0x0b);
        
        
        ///////////////////////////////////////////////// 
        //////////////////   EEINTP  ////////////////////
        ///////////////////////////////////////////////// 
	write_cmos_sensor(client,0x8f,0xff); 
	write_cmos_sensor(client,0x90,0x9f); 
	write_cmos_sensor(client,0x91,0x90); 
	write_cmos_sensor(client,0x92,0x03); 
	write_cmos_sensor(client,0x93,0x03); 
	write_cmos_sensor(client,0x94,0x05);
	write_cmos_sensor(client,0x95,0x65); 
	write_cmos_sensor(client,0x96,0xf0); 
        
        ///////////////////////////////////////////////// 
        /////////////////////  ASDE  ////////////////////
        ///////////////////////////////////////////////// 
	write_cmos_sensor(client,0xfe,0x00);
	write_cmos_sensor(client,0x9a,0x20);
	write_cmos_sensor(client,0x9b,0x80);
	write_cmos_sensor(client,0x9c,0x40);
	write_cmos_sensor(client,0x9d,0x80);
	write_cmos_sensor(client,0xa1,0x30);
	write_cmos_sensor(client,0xa2,0x32);
	write_cmos_sensor(client,0xa4,0x30);
	write_cmos_sensor(client,0xa5,0x30);
	write_cmos_sensor(client,0xaa,0x50);
	write_cmos_sensor(client,0xac,0x22);
        
        /////////////////////////////////////////////////
        ///////////////////   GAMMA   ///////////////////
        /////////////////////////////////////////////////
	write_cmos_sensor(client,0xbf,0x08); 
	write_cmos_sensor(client,0xc0,0x16); 
	write_cmos_sensor(client,0xc1,0x28); 
	write_cmos_sensor(client,0xc2,0x41); 
	write_cmos_sensor(client,0xc3,0x5a); 
	write_cmos_sensor(client,0xc4,0x6c); 
	write_cmos_sensor(client,0xc5,0x7a); 
	write_cmos_sensor(client,0xc6,0x96); 
	write_cmos_sensor(client,0xc7,0xac); 
	write_cmos_sensor(client,0xc8,0xbc); 
	write_cmos_sensor(client,0xc9,0xc9); 
	write_cmos_sensor(client,0xca,0xd3); 
	write_cmos_sensor(client,0xcb,0xdd); 
	write_cmos_sensor(client,0xcc,0xe5); 
	write_cmos_sensor(client,0xcd,0xf1); 
	write_cmos_sensor(client,0xce,0xfa); 
	write_cmos_sensor(client,0xcf,0xff);
        
        /////////////////////////////////////////////////
        ///////////////////   YCP  //////////////////////
        /////////////////////////////////////////////////
	write_cmos_sensor(client,0xd0,0x40); 
	write_cmos_sensor(client,0xd1,0x34); 
	write_cmos_sensor(client,0xd2,0x34); 
	write_cmos_sensor(client,0xd3,0x3c); 
	write_cmos_sensor(client,0xd6,0xf2); 
	write_cmos_sensor(client,0xd7,0x1b); 
	write_cmos_sensor(client,0xd8,0x18); 
	write_cmos_sensor(client,0xdd,0x03); 
        /////////////////////////////////////////////////
        ////////////////////   AEC   ////////////////////
        /////////////////////////////////////////////////
	write_cmos_sensor(client,0xfe,0x01);
	write_cmos_sensor(client,0x05,0x30); 
	write_cmos_sensor(client,0x06,0x75); 
	write_cmos_sensor(client,0x07,0x40); 
	write_cmos_sensor(client,0x08,0xb0); 
	write_cmos_sensor(client,0x0a,0xc5); 
	write_cmos_sensor(client,0x0b,0x11);
	write_cmos_sensor(client,0x0c,0x00); 
	write_cmos_sensor(client,0x12,0x52);
	write_cmos_sensor(client,0x13,0x08);  //0x38 ---28
	write_cmos_sensor(client,0x18,0x95);
	write_cmos_sensor(client,0x19,0x96);
	write_cmos_sensor(client,0x1f,0x20);
	write_cmos_sensor(client,0x20,0xc0); 
	write_cmos_sensor(client,0x3e,0x40); 
	write_cmos_sensor(client,0x3f,0x57); 
	write_cmos_sensor(client,0x40,0x7d); 
	write_cmos_sensor(client,0x03,0x60); 
	write_cmos_sensor(client,0x44,0x02); 
	/////////////////////////////////////////////////
	////////////////////   AWB   ////////////////////
	/////////////////////////////////////////////////
	write_cmos_sensor(client,0x1c,0x91); 
	write_cmos_sensor(client,0x21,0x15); 
	write_cmos_sensor(client,0x50,0x80);
	write_cmos_sensor(client,0x56,0x04);
	write_cmos_sensor(client,0x58,0x08);    
	write_cmos_sensor(client,0x59,0x08); 
	write_cmos_sensor(client,0x5b,0x82);  // 02 to 82 to 02
	write_cmos_sensor(client,0x61,0x8d); 
	write_cmos_sensor(client,0x62,0xa7); 
	write_cmos_sensor(client,0x63,0x00);   // d0 to  00
	write_cmos_sensor(client,0x65,0x06);
	write_cmos_sensor(client,0x66,0x06);   // 06 to 03
	write_cmos_sensor(client,0x67,0x84); 
	write_cmos_sensor(client,0x69,0x08);   // 08 to 20
	write_cmos_sensor(client,0x6a,0x25); 
	write_cmos_sensor(client,0x6b,0x01); 
	write_cmos_sensor(client,0x6c,0x00);   // 00 to 0c
	write_cmos_sensor(client,0x6d,0x02); 
	write_cmos_sensor(client,0x6e,0x00);  // f0 to 00
	write_cmos_sensor(client,0x6f,0x80); 
	write_cmos_sensor(client,0x76,0x80); 
	write_cmos_sensor(client,0x78,0xaf); 
	write_cmos_sensor(client,0x79,0x75);
	write_cmos_sensor(client,0x7a,0x40);
	write_cmos_sensor(client,0x7b,0x50);
	write_cmos_sensor(client,0x7c,0x08); //0c to 08 8.11
         
	write_cmos_sensor(client,0xa4,0xb9); 
	write_cmos_sensor(client,0xa5,0xa0);
	write_cmos_sensor(client,0x90,0xc9); 
	write_cmos_sensor(client,0x91,0xbe);
	write_cmos_sensor(client,0xa6,0xb8); 
	write_cmos_sensor(client,0xa7,0x95); 
	write_cmos_sensor(client,0x92,0xe6); 
	write_cmos_sensor(client,0x93,0xca); 
	write_cmos_sensor(client,0xa9,0xb6); 
	write_cmos_sensor(client,0xaa,0x89); 
	write_cmos_sensor(client,0x95,0x23); 
	write_cmos_sensor(client,0x96,0xe7); 
	write_cmos_sensor(client,0xab,0x9d); 
	write_cmos_sensor(client,0xac,0x80);
	write_cmos_sensor(client,0x97,0x43); 
	write_cmos_sensor(client,0x98,0x24); 
	write_cmos_sensor(client,0xae,0xd0);   // b7 to d0
	write_cmos_sensor(client,0xaf,0x9e); 
	write_cmos_sensor(client,0x9a,0x43);
	write_cmos_sensor(client,0x9b,0x24); 
        
	write_cmos_sensor(client,0xb0,0xc0);  // c8 to c0
	write_cmos_sensor(client,0xb1,0xa8);   // 97 to a8
	write_cmos_sensor(client,0x9c,0xc4); 
	write_cmos_sensor(client,0x9d,0x44); 
	write_cmos_sensor(client,0xb3,0xb7); 
	write_cmos_sensor(client,0xb4,0x7f);
	write_cmos_sensor(client,0x9f,0xc7);
	write_cmos_sensor(client,0xa0,0xc8); 
	write_cmos_sensor(client,0xb5,0x00); 
	write_cmos_sensor(client,0xb6,0x00);
	write_cmos_sensor(client,0xa1,0x00);
	write_cmos_sensor(client,0xa2,0x00);
	write_cmos_sensor(client,0x86,0x60);
	write_cmos_sensor(client,0x87,0x08);
	write_cmos_sensor(client,0x88,0x00);
	write_cmos_sensor(client,0x89,0x00);
	write_cmos_sensor(client,0x8b,0xde);
	write_cmos_sensor(client,0x8c,0x80);
	write_cmos_sensor(client,0x8d,0x00);
	write_cmos_sensor(client,0x8e,0x00);
	write_cmos_sensor(client,0x94,0x55);
	write_cmos_sensor(client,0x99,0xa6);
	write_cmos_sensor(client,0x9e,0xaa);
	write_cmos_sensor(client,0xa3,0x0a);
	write_cmos_sensor(client,0x8a,0x0a);
	write_cmos_sensor(client,0xa8,0x55);
	write_cmos_sensor(client,0xad,0x55);
	write_cmos_sensor(client,0xb2,0x55);
	write_cmos_sensor(client,0xb7,0x05);
	write_cmos_sensor(client,0x8f,0x05);
	write_cmos_sensor(client,0xb8,0xcc);
	write_cmos_sensor(client,0xb9,0x9a);
        
        /////////////////////////////////////
        ////////////////////  CC ////////////
        /////////////////////////////////////
	write_cmos_sensor(client,0xfe,0x01);
	write_cmos_sensor(client,0xd0,0x38);
	write_cmos_sensor(client,0xd1,0xfd);
	write_cmos_sensor(client,0xd2,0x06);
	write_cmos_sensor(client,0xd3,0xf0);
	write_cmos_sensor(client,0xd4,0x40);
	write_cmos_sensor(client,0xd5,0x08);
	write_cmos_sensor(client,0xd6,0x30);
	write_cmos_sensor(client,0xd7,0x00);
	write_cmos_sensor(client,0xd8,0x0a);
	write_cmos_sensor(client,0xd9,0x16);
	write_cmos_sensor(client,0xda,0x39);
	write_cmos_sensor(client,0xdb,0xf8);
        
        /////////////////////////////////////////////////
        ////////////////////   LSC   ////////////////////
        /////////////////////////////////////////////////
	write_cmos_sensor(client,0xfe,0x01); 
	write_cmos_sensor(client,0xc1,0x3c); 
	write_cmos_sensor(client,0xc2,0x50); 
	write_cmos_sensor(client,0xc3,0x00); 
	write_cmos_sensor(client,0xc4,0x40); 
	write_cmos_sensor(client,0xc5,0x30); 
	write_cmos_sensor(client,0xc6,0x30); 
	write_cmos_sensor(client,0xc7,0x10); 
	write_cmos_sensor(client,0xc8,0x00); 
	write_cmos_sensor(client,0xc9,0x00); 
	write_cmos_sensor(client,0xdc,0x20); 
	write_cmos_sensor(client,0xdd,0x10); 
	write_cmos_sensor(client,0xdf,0x00); 
	write_cmos_sensor(client,0xde,0x00); 
        
        /////////////////////////////////////////////////
        ///////////////////  Histogram  /////////////////
        /////////////////////////////////////////////////
	write_cmos_sensor(client,0x01,0x10); 
	write_cmos_sensor(client,0x0b,0x31); 
	write_cmos_sensor(client,0x0e,0x50); 
	write_cmos_sensor(client,0x0f,0x0f); 
	write_cmos_sensor(client,0x10,0x6e); 
	write_cmos_sensor(client,0x12,0xa0); 
	write_cmos_sensor(client,0x15,0x60); 
	write_cmos_sensor(client,0x16,0x60); 
	write_cmos_sensor(client,0x17,0xe0); 
        
        /////////////////////////////////////////////////
        //////////////   Measure Window   ///////////////
        /////////////////////////////////////////////////
	write_cmos_sensor(client,0xcc,0x0c);  
	write_cmos_sensor(client,0xcd,0x10); 
	write_cmos_sensor(client,0xce,0xa0); 
	write_cmos_sensor(client,0xcf,0xe6); 
        
        /////////////////////////////////////////////////
        /////////////////   dark sun   //////////////////
        /////////////////////////////////////////////////
	write_cmos_sensor(client,0x45,0xf7);
	write_cmos_sensor(client,0x46,0xff); 
	write_cmos_sensor(client,0x47,0x15);
	write_cmos_sensor(client,0x48,0x03); 
	write_cmos_sensor(client,0x4f,0x60); 
        
        /////////////////////////////////////////////////
        ///////////////////  banding  ///////////////////
        /////////////////////////////////////////////////
	write_cmos_sensor(client,0xfe,0x00);
	write_cmos_sensor(client,0x05,0x01);
	write_cmos_sensor(client,0x06,0x18); //HB#if 1    
	write_cmos_sensor(client,0x07,0x00);
	write_cmos_sensor(client,0x08,0x10); //VB  from 10 to 50

	write_cmos_sensor(client,0xfe,0x01);
	write_cmos_sensor(client,0x25,0x00); //step 
	write_cmos_sensor(client,0x26,0x9a); 
	write_cmos_sensor(client,0x27,0x01); //30fps
	write_cmos_sensor(client,0x28,0xce);  
	write_cmos_sensor(client,0x29,0x04); //12.5fps
	write_cmos_sensor(client,0x2a,0x36); 
	write_cmos_sensor(client,0x2b,0x06); //10fps
	write_cmos_sensor(client,0x2c,0x04); 
	write_cmos_sensor(client,0x2d,0x0c); //5fps
	write_cmos_sensor(client,0x2e,0x08);
	write_cmos_sensor(client,0x3c,0x20);
        
        /////////////////////////////////////////////////
        ///////////////////   MIPI   ////////////////////
        /////////////////////////////////////////////////
	write_cmos_sensor(client,0xfe,0x03);
	write_cmos_sensor(client,0x10,0x94);  
	write_cmos_sensor(client,0xfe,0x00); 
	
}

//prize yanrenjie added gc6153 20190416
void GC6153_Sensor_Init(struct i2c_client *client)
{
	//CAMERA_DEBUG("GC6153_Sensor_Init");
	/////////////////////////////////////////////////////
	//////////////////////	 SYS   //////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0xfe, 0xa0);
	write_cmos_sensor(client,0xfe, 0xa0);
	write_cmos_sensor(client,0xfe, 0xa0);

	write_cmos_sensor(client,0xfa, 0x11);
	write_cmos_sensor(client,0xfc, 0x00);
	write_cmos_sensor(client,0xf6, 0x00); 
	write_cmos_sensor(client,0xfc, 0x12);
	/////////////////////////////////////////////////////
	////////////////   ANALOG & CISCTL	 ////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0xfe, 0x00);
	write_cmos_sensor(client,0x01, 0x40); 
	write_cmos_sensor(client,0x02, 0x12); 
	write_cmos_sensor(client,0x0d, 0x40); 
	write_cmos_sensor(client,0x14, 0x7e); 
	write_cmos_sensor(client,0x16, 0x05); 
	write_cmos_sensor(client,0x17, 0x18); 
	write_cmos_sensor(client,0x1c, 0x31); 
	write_cmos_sensor(client,0x1d, 0xb9); 
	write_cmos_sensor(client,0x1f, 0x1a); 
	write_cmos_sensor(client,0x73, 0x20); 
	write_cmos_sensor(client,0x74, 0x71); 
	write_cmos_sensor(client,0x77, 0x22); 
	write_cmos_sensor(client,0x7a, 0x08); 
	write_cmos_sensor(client,0x11, 0x18); 
	write_cmos_sensor(client,0x13, 0x48); 
	write_cmos_sensor(client,0x12, 0xc8); 
	write_cmos_sensor(client,0x70, 0xc8); 
	write_cmos_sensor(client,0x7b, 0x18); 
	write_cmos_sensor(client,0x7d, 0x30); 
	write_cmos_sensor(client,0x7e, 0x02); 

	write_cmos_sensor(client,0xfe, 0x10); 
	write_cmos_sensor(client,0xfe, 0x00);
	write_cmos_sensor(client,0xfe, 0x00);
	write_cmos_sensor(client,0xfe, 0x00);
	write_cmos_sensor(client,0xfe, 0x00);
	write_cmos_sensor(client,0xfe, 0x00);
	write_cmos_sensor(client,0xfe, 0x10);
	write_cmos_sensor(client,0xfe, 0x00);

	write_cmos_sensor(client,0x49, 0x61);
	write_cmos_sensor(client,0x4a, 0x40);
	write_cmos_sensor(client,0x4b, 0x58);

	/////////////////////////////////////////////////////
	//////////////////////	 ISP   //////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0xfe, 0x00);
	write_cmos_sensor(client,0x39, 0x02); 
	write_cmos_sensor(client,0x3a, 0x80); 
	write_cmos_sensor(client,0x20, 0x7e); 
	write_cmos_sensor(client,0x26, 0x87); 
	/////////////////////////////////////////////////////
	//////////////////////	 BLK   //////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0x33, 0x10); 
	write_cmos_sensor(client,0x37, 0x06); 
	write_cmos_sensor(client,0x2a, 0x21); 

	/////////////////////////////////////////////////////
	//////////////////////	 GAIN	/////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0x3f, 0x16); 


	/////////////////////////////////////////////////////
	//////////////////////	 DNDD	/////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0x52, 0xa6); 
	write_cmos_sensor(client,0x53, 0x81);
	write_cmos_sensor(client,0x54, 0x43);
	write_cmos_sensor(client,0x56, 0x78); 
	write_cmos_sensor(client,0x57, 0xaa);
	write_cmos_sensor(client,0x58, 0xff);

	/////////////////////////////////////////////////////
	//////////////////////	 ASDE	/////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0x5b, 0x60); //dd&ee th
	write_cmos_sensor(client,0x5c, 0x50); //60/OT_th
	write_cmos_sensor(client,0xab, 0x2a); 
	write_cmos_sensor(client,0xac, 0xb5);

	/////////////////////////////////////////////////////
	/////////////////////	INTPEE	 ////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0x5e, 0x06); 
	write_cmos_sensor(client,0x5f, 0x06);
	write_cmos_sensor(client,0x60, 0x44); 
	write_cmos_sensor(client,0x61, 0xff); //20160901
	write_cmos_sensor(client,0x62, 0x69); //edge effect
	write_cmos_sensor(client,0x63, 0x13);

	/////////////////////////////////////////////////////
	//////////////////////	 CC   ///////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0x65, 0x13); //Y
	write_cmos_sensor(client,0x66, 0x26);
	write_cmos_sensor(client,0x67, 0x07);
	write_cmos_sensor(client,0x68, 0xf5); //Cb
	write_cmos_sensor(client,0x69, 0xea);
	write_cmos_sensor(client,0x6a, 0x21);
	write_cmos_sensor(client,0x6b, 0x21); //Cr
	write_cmos_sensor(client,0x6c, 0xe4);
	write_cmos_sensor(client,0x6d, 0xfb);

	/////////////////////////////////////////////////////
	//////////////////////	 YCP   //////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0x81, 0x3b); 
	write_cmos_sensor(client,0x82, 0x3b); 
	write_cmos_sensor(client,0x83, 0x4b); 
	write_cmos_sensor(client,0x84, 0x90);  
	write_cmos_sensor(client,0x86, 0xf0); 
	write_cmos_sensor(client,0x87, 0x1d); 
	write_cmos_sensor(client,0x88, 0x16); 
	write_cmos_sensor(client,0x8d, 0x74); 
	write_cmos_sensor(client,0x8e, 0x25); 
	/////////////////////////////////////////////////////
	//////////////////////	 AEC   //////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0x90, 0x36);
	write_cmos_sensor(client,0x92, 0x43); 
	write_cmos_sensor(client,0x9d, 0x32);
	write_cmos_sensor(client,0x9e, 0x81);
	write_cmos_sensor(client,0x9f, 0xf4);
	write_cmos_sensor(client,0xa0, 0xa0); 
	write_cmos_sensor(client,0xa1, 0x04); 
	write_cmos_sensor(client,0xa3, 0x2d); 
	write_cmos_sensor(client,0xa4, 0x01);
	/////////////////////////////////////////////////////
	//////////////////////	 AWB   //////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0xb0, 0xc2);
	write_cmos_sensor(client,0xb1, 0x1e);
	write_cmos_sensor(client,0xb2, 0x10);
	write_cmos_sensor(client,0xb3, 0x20);
	write_cmos_sensor(client,0xb4, 0x2d);
	write_cmos_sensor(client,0xb5, 0x1b); 
	write_cmos_sensor(client,0xb6, 0x2e);
	write_cmos_sensor(client,0xb8, 0x13);
	write_cmos_sensor(client,0xba, 0x60);
	write_cmos_sensor(client,0xbb, 0x62);
	write_cmos_sensor(client,0xbd, 0x78); 
	write_cmos_sensor(client,0xbe, 0x55);
	write_cmos_sensor(client,0xbf, 0xa0); 
	write_cmos_sensor(client,0xc4, 0xe7);
	write_cmos_sensor(client,0xc5, 0x15);
	write_cmos_sensor(client,0xc6, 0x16);
	write_cmos_sensor(client,0xc7, 0xeb); 
	write_cmos_sensor(client,0xc8, 0xe4);
	write_cmos_sensor(client,0xc9, 0x16);
	write_cmos_sensor(client,0xca, 0x16);
	write_cmos_sensor(client,0xcb, 0xe9);
	write_cmos_sensor(client,0x22, 0xf8); 

	/////////////////////////////////////////////////////
	////////////////////   Banding	 ////////////////////
	/////////////////////////////////////////////////////
	//write_cmos_sensor(client,0x01, 0x41); //hb
	//write_cmos_sensor(client,0x02, 0x12); //vb
	//write_cmos_sensor(client,0x0f, 0x01);
	//write_cmos_sensor(client,0x9d, 0x32); //step
	//write_cmos_sensor(client,0x9e, 0x61); //[7:4]margin  10fps
	//write_cmos_sensor(client,0x9f, 0xf4); 
	/////////////////////////////////////////////////////
	//////////////////////	 SPI   //////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(client,0xfe, 0x02);
	write_cmos_sensor(client,0x01, 0x01); 
	write_cmos_sensor(client,0x02, 0x02); 
	write_cmos_sensor(client,0x03, 0x20); 
	write_cmos_sensor(client,0x04, 0x20); 
	write_cmos_sensor(client,0x0a, 0x00); 
	write_cmos_sensor(client,0x13, 0x10); 
	write_cmos_sensor(client,0x24, 0x00); 
	write_cmos_sensor(client,0x28, 0x03); 
	write_cmos_sensor(client,0xfe, 0x00);
	////////////////////////////////////////////////////
	///////////////////////output//////////////////////
	///////////////////////////////////////////////////
	write_cmos_sensor(client,0xf2, 0x03); //open awb
	write_cmos_sensor(client,0xfe, 0x00); //output enable
	
}

/*************************************************************************
 * FUNCTION
 *	CameraOpen
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static UINT32 CameraOpen(struct i2c_client *client)
{
	volatile signed char i;
	int id_status = 0;

	CAMERA_DBG("<Jet> Entry CameraOpen!!! \r\n");
	msleep(10);
	sensor_id = 0;
	//  Read sensor ID to adjust I2C is OK?
	for(i=0;i<3;i++)
	{
		client->addr = GC6133_SERIAL_ADDR;
		kdCISModulePowerOn(client,true);
		//sensor_id=(read_cmos_sensor(client,0xf0) << 8) |(read_cmos_sensor(client,0xf1));
		sensor_id=read_cmos_sensor(client,0xf0);
		CAMERA_DBG("gc6133 *sensorID=%x %s %d \n",sensor_id,__func__,__LINE__);
		if(sensor_id != GC6133_SENSOR_ID)  
		{
			CAMERA_DBG("GC6133 mipi Read Sensor ID Fail[open] = 0x%x\n", sensor_id); 
			//return ERROR_SENSOR_CONNECT_FAIL;
		}else{
		    g_dcam_r_client->addr = GC6133_SERIAL_ADDR;
			id_status = 1;
			break;
		}
	}
	if(!id_status)
	{
		for(i=0;i<3;i++)
		{
			client->addr = GC2145_SERIAL_ADDR;
			kdCISModulePowerOn(client,true);
			sensor_id=(read_cmos_sensor(client,0xf0) << 8) |(read_cmos_sensor(client,0xf1));
			//sensor_id=read_cmos_sensor(client,0xf0);
			CAMERA_DBG("gc2145 *sensorID=%x %s %d\n",sensor_id,__func__,__LINE__);
			if(sensor_id != GC2145_SENSOR_ID)  
			{
				CAMERA_DBG("GC2145 mipi Read Sensor ID Fail[open] = 0x%x\n", sensor_id); 
				//return ERROR_SENSOR_CONNECT_FAIL;
			}else{
				g_dcam_r_client->addr = GC2145_SERIAL_ADDR;
				id_status = 1;
				break;
			}
		}
	}
	if(!id_status)
	{
		for(i=0;i<3;i++)
		{
			client->addr = GC0310_SERIAL_ADDR;
			kdCISModulePowerOn(client,true);
			sensor_id=(read_cmos_sensor(client,0xf0) << 8) |(read_cmos_sensor(client,0xf1));
			//sensor_id=read_cmos_sensor(client,0xf0);
			CAMERA_DBG("0310 *sensorID=%x %s %d\n",sensor_id,__func__,__LINE__);
			if(sensor_id != GC0310_SENSOR_ID)  
			{
				CAMERA_DBG("GC2145 mipi Read Sensor ID Fail[open] = 0x%x\n", sensor_id); 
				//return ERROR_SENSOR_CONNECT_FAIL;
			}else{
				g_dcam_r_client->addr = GC0310_SERIAL_ADDR;							
				id_status = 1;
				break;
			}
		}
	}
	if(!id_status)//prize yanrenjie added gc6153 20190416
	{
		for(i=0;i<3;i++)
		{
			client->addr = GC6153_SERIAL_ADDR;//the same as 6133
			kdCISModulePowerOn(client,true);
			sensor_id=(read_cmos_sensor(client,0xf0) << 8) |(read_cmos_sensor(client,0xf1));
			CAMERA_DBG("GC6153 *sensorID=%x %s %d\n",sensor_id,__func__,__LINE__);
			if(sensor_id != GC6153_SENSOR_ID)  
			{
				CAMERA_DBG("GC6153 mipi Read Sensor ID Fail[open] = 0x%x\n", sensor_id); 
				//return ERROR_SENSOR_CONNECT_FAIL;
			}else{
				g_dcam_r_client->addr = GC6153_SERIAL_ADDR;							
				id_status = 1;
				break;
			}
		}
	}
	if (!id_status){
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	
	CAMERA_DBG("Cameramipi_ Sensor Read ID OK \r\n");
	if(sensor_id == GC6133_SENSOR_ID)  
		GC6133_Sensor_Init(client);
	else 	if(sensor_id == GC2145_SENSOR_ID)   
		GC2145_Sensor_Init(client);
	else    if(sensor_id == GC0310_SENSOR_ID)
		GC0310_Sensor_Init(client);
	else
		GC6153_Sensor_Init(client);

	return ERROR_NONE;
}
static kal_uint16 Camera_Read_Shutter(struct i2c_client *client)
{
    kal_uint8 temp_reg1, temp_reg2;
	int shutter;
	
	if(sensor_id == GC6133_SENSOR_ID)
		client->addr = GC6133_SERIAL_ADDR;
	else if(sensor_id == GC2145_SENSOR_ID)
		client->addr = GC2145_SERIAL_ADDR;
	else if(sensor_id == GC0310_SENSOR_ID)
		client->addr = GC0310_SERIAL_ADDR;
	else if(sensor_id == GC6153_SENSOR_ID)//prize yanrenjie added 6153 20190416
		client->addr = GC6153_SERIAL_ADDR;	
	else
		client->addr = 0x3c;
	temp_reg1 = read_cmos_sensor(client,0x04);
	temp_reg2 = read_cmos_sensor(client,0x03);

	shutter = (temp_reg1 & 0xFF) | (temp_reg2 << 8);
	CAMERA_DBG("camera_Read_Shutter real %d of %x\r\n",shutter,sensor_id);
	if(sensor_id == GC6133_SENSOR_ID)
		shutter = shutter * 4;  // prize add by zhuzhengjiang make shutter approximate, gc6133 max shutter is 500,but gc2145 max shutter is 2000
	if(sensor_id == GC0310_SENSOR_ID)
		shutter = shutter * 5/4;  // make shutter approximate, gc0310 max shutter is 1500,but gc2145 max shutter is 2000
	if(sensor_id == GC6153_SENSOR_ID)
		shutter = shutter * 4;	// prize add by yanrenjie make shutter approximate, gc6153 max shutter is 500

	CAMERA_DBG("camera_Read_Shutter %d\r\n",shutter);
	return shutter;
}

static ssize_t show_camera_front_shutter_value(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct i2c_client *client = g_dcam_f_client;
	struct dcam_data_t *dcam_data = i2c_get_clientdata(client);
	int shutter_value = 0;
	
	if (!dcam_data->is_enabled){
		return sprintf(buf, "%d\n", -1);
	}

	shutter_value=Camera_Read_Shutter(client);
	return sprintf(buf, "%d", shutter_value);
}
static ssize_t store_camera_front_shutter_value(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{ 
	return count;
}
static struct kobj_attribute camera_front_value_attr = {
		.attr = {
		.name = "dcam_f_value",
		.mode = 0644,
	},
	.show = show_camera_front_shutter_value,
	.store = store_camera_front_shutter_value,
};
//-----------------------------------------------------
static ssize_t show_camera_rear_shutter_value(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct i2c_client *client = g_dcam_r_client;
	struct dcam_data_t *dcam_data = i2c_get_clientdata(client);
	int shutter_value = 0;
	
	if (!dcam_data->is_enabled){
		return sprintf(buf, "%d\n", -1);
	}

	shutter_value=Camera_Read_Shutter(client);
	return sprintf(buf, "%d", shutter_value);
}
static ssize_t store_camera_rear_shutter_value(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	return count;
}
static struct kobj_attribute camera_rear_value_attr = {
	.attr = {
		 .name = "dcam_r_value",
		 .mode = 0644,
		 },
	.show = show_camera_rear_shutter_value,
	.store = store_camera_rear_shutter_value,
};
//--------------------------------------------------------------------------
static ssize_t show_camera_rear_open(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct i2c_client *client = g_dcam_r_client;
	struct dcam_data_t *dcam_data = i2c_get_clientdata(client);
	int ret = -1;

	//mdelay(100);
	mutex_lock(&dcam_r_mutex);
	if (!dcam_data->is_enabled){
		//kdCISModulePowerOn(client,"camerayuv",true,"kd_camera_hw");
		ret = CameraOpen(client);
		if (ret){
			printk(KERN_ERR"DCAM rear_open fail %d\n",ret);
			kdCISModulePowerOn(client,false);
			mutex_unlock(&dcam_r_mutex);
			return sprintf(buf, "%d\n", ret);
		}
		CameraStreamOn(client);
		dcam_data->is_enabled = 1;
	}
	mutex_unlock(&dcam_r_mutex);
	printk(KERN_INFO"DCAM rear_open %d\n",dcam_data->is_enabled);
	return sprintf(buf, "%d\n", 0);
}
static ssize_t store_camera_rear_open(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	return count;
}
static struct kobj_attribute camera_rear_open_attr = {
	.attr = {
		 .name = "dcam_r_open",
		 .mode = 0644,
		 },
	.show = show_camera_rear_open,
	.store = store_camera_rear_open,
};
//----------------------------------------------------------------------------
static ssize_t show_camera_front_open(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct i2c_client *client = g_dcam_f_client;
	struct dcam_data_t *dcam_data = i2c_get_clientdata(client);
	int ret = -1;
	
	//mdelay(100);
	mutex_lock(&dcam_f_mutex);
	if (!dcam_data->is_enabled){
		//kdCISModulePowerOn(client,"camerayuv",true,"kd_camera_hw");
		ret = CameraOpen(client);
		if (ret){
			printk(KERN_ERR"DCAM front_open fail %d\n",ret);
			kdCISModulePowerOn(client,false);
			mutex_unlock(&dcam_f_mutex);
			return sprintf(buf, "%d\n", ret);
		}
		CameraStreamOn(client);
		dcam_data->is_enabled = 1;
	}
	mutex_unlock(&dcam_f_mutex);
	printk(KERN_INFO"DCAM front_open %d\n",dcam_data->is_enabled);
	return sprintf(buf, "%d\n", 0);
}
static ssize_t store_camera_front_open(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	return count;
}
static struct kobj_attribute camera_front_open_attr = {
	.attr = {
		 .name = "dcam_f_open",
		 .mode = 0644,
		 },
	.show = show_camera_front_open,
	.store = store_camera_front_open,
};
//----------------------------------------------------------------
static ssize_t show_camera_rear_close(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct i2c_client *client = g_dcam_r_client;
	struct dcam_data_t *dcam_data = i2c_get_clientdata(client);

	mutex_lock(&dcam_r_mutex);
	if (dcam_data->is_enabled){
		//kdCISModulePowerOn(client,"gc6133yuv",false,"kd_camera_hw");
		kdCISModulePowerOn(client,false);
		dcam_data->is_enabled = 0;
	}

	mdelay(100);
	mutex_unlock(&dcam_r_mutex);
	printk(KERN_INFO"DCAM rear_close %d\n",dcam_data->is_enabled);
	return sprintf(buf, "%d\n", 0);
}
static ssize_t store_camera_rear_close(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	return count;
}
static struct kobj_attribute camera_rear_close_attr = {
	.attr = {
		 .name = "dcam_r_close",
		 .mode = 0644,
		 },
	.show = show_camera_rear_close,
	.store = store_camera_rear_close,
};
//----------------------------------------------------------------------------
static ssize_t show_camera_front_close(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct i2c_client *client = g_dcam_f_client;
	struct dcam_data_t *dcam_data = i2c_get_clientdata(client);

	mutex_lock(&dcam_f_mutex);
	if (dcam_data->is_enabled){
		//kdCISModulePowerOn(client,"gc6133yuv",false,"kd_camera_hw");
		kdCISModulePowerOn(client,false);
		dcam_data->is_enabled = 0;
	}
	mutex_unlock(&dcam_f_mutex);

	//mdelay(100);
	printk(KERN_INFO"DCAM rear_close %d\n",dcam_data->is_enabled);
	return sprintf(buf, "%d\n", 0);
}
static ssize_t store_camera_front_close(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	return count;
}
static struct kobj_attribute camera_front_close_attr = {
	.attr = {
		 .name = "dcam_f_close",
		 .mode = 0644,
		 },
	.show = show_camera_front_close,
	.store = store_camera_front_close,
};
//----------------------------------------------------------------


static ssize_t device_enable_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct dcam_data_t *dcam_data = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", dcam_data->is_enabled);
}
static ssize_t device_enable_store(struct device *dev, struct device_attribute *attr, 
               const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct dcam_data_t *dcam_data = i2c_get_clientdata(client);
	int cmd = -1;
	
	printk(KERN_INFO"DCAM %s:%s\n",__func__,client->name);
	if (dcam_data == NULL){
		printk(KERN_ERR"DCAM dcam_data is NULL'n");
		return count;
	}
	sscanf(buf, "%d", &cmd);
	if (cmd == 1){
		if (!dcam_data->is_enabled){
			//kdCISModulePowerOn(client,"gc6133yuv",true,"kd_camera_hw");
			CameraOpen(client);
			CameraStreamOn(client);
			dcam_data->is_enabled = 1;
		}
	}else if (cmd == 0){
		if (dcam_data->is_enabled){
			kdCISModulePowerOn(client,false);
			dcam_data->is_enabled = 0;
		}
	}else{
		printk(KERN_ERR"DCAM wrong cmd %d\n",cmd);
	}
	printk("DCAM 4\n");
	printk("DCAM %s cmd%d\n",__func__,cmd);

	return count;
}
//prize  add  for gc6133/gc0310/gc2145 by zhuzhengjiang    20190116-end
static DEVICE_ATTR(enable, 0644, device_enable_show, device_enable_store);

static ssize_t shutter_value_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct dcam_data_t *dcam_data = i2c_get_clientdata(client);
	int shutter_value = 0;
	
	if (unlikely(!dcam_data->is_enabled)){
		return sprintf(buf, "%d\n", -1);
	}

	shutter_value=Camera_Read_Shutter(client);
	return sprintf(buf, "%d\n", shutter_value);
}
static ssize_t __maybe_unused shutter_value_store(struct device *dev, struct device_attribute *attr, 
               const char *buf, size_t count)
{
	return count;
}
//static DEVICE_ATTR(value, 0644, shutter_value_show, shutter_value_store);
static DEVICE_ATTR(value, 0644, shutter_value_show, device_enable_store);


static const struct of_device_id __maybe_unused dcam_of_match[] = {
	{ .compatible = "prize,dcam_r", .data = &dcam_r_devtype },
	{ .compatible = "prize,dcam_f", .data = &dcam_f_devtype },
	{ }
};
MODULE_DEVICE_TABLE(of, dcam_of_match);

static const struct i2c_device_id dcam_id_table[] = {
	{ "DCAM_R",	(kernel_ulong_t)&dcam_r_devtype, },
	{ "DCAM_F",	(kernel_ulong_t)&dcam_f_devtype, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, dcam_id_table);

static int dcam_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id){
	
	int ret;
	struct dcam_data_t *dcam_data;
	const struct of_device_id *of_id = of_match_device(dcam_of_match,&client->dev);
	struct device_node *node;
	
	printk("DCAM %s %s\n",__func__,client->name);
	
	dcam_data = (struct dcam_data_t *)of_id->data;
	i2c_set_clientdata(client,dcam_data);
	
	if (!strcmp(client->name,"dcam_r")){
		g_dcam_r_client = client;
		node = of_find_compatible_node(NULL,NULL,"prize,dcam_r");
		dcam_data->pdn_pin = of_get_named_gpio(node,"pdn_pin",0);
		dcam_data->rst_pin = of_get_named_gpio(node,"rst_pin",0);

		printk("DCAM dcam_r_pdn_pin(%d)  dcam_data->pdn_pin(%d)\n",dcam_data->pdn_pin,dcam_data->pdn_pin);
		gpio_request(dcam_data->pdn_pin,"dcam_r");
		gpio_direction_output(dcam_data->pdn_pin,0);
        msleep(5);
		gpio_direction_output(dcam_data->pdn_pin,1); //prize-chj-20190528-Modify the timing problem to pull up and not work
		gpio_request(dcam_data->rst_pin,"dcam_r");
		gpio_direction_output(dcam_data->rst_pin,0);
		ret = of_property_read_u32(node,"sensor_type",&dcam_data->sensor_type);
		if (ret){
			printk("DCAM get sensor_type fail ret(%d)\n",ret);
		}

		if (NULL != g_dcam_r_client){
			ret =  sysfs_create_file(dcam_kobj, &camera_rear_value_attr.attr);
			if (ret)
				CAMERA_DBG(" kernel sysfs_create_file error \r\n");
			ret =  sysfs_create_file(dcam_kobj, &camera_rear_open_attr.attr);
			if (ret)
				CAMERA_DBG(" kernel sysfs_create_file error \r\n");
			ret =  sysfs_create_file(dcam_kobj, &camera_rear_close_attr.attr);
			if (ret)
				CAMERA_DBG(" kernel sysfs_create_file error \r\n");
		}
		
	}else if (!strcmp(client->name,"dcam_f")){
		g_dcam_f_client = client;
		node = of_find_compatible_node(NULL,NULL,"prize,dcam_f");
		dcam_data->pdn_pin = of_get_named_gpio(node,"pdn_pin",0);
		dcam_data->rst_pin = of_get_named_gpio(node,"rst_pin",0);
		printk("DCAM dcam_data->pdn_pin(%d)  dcam_data->rst_pin(%d) \n",dcam_data->pdn_pin,dcam_data->rst_pin);
		gpio_request(dcam_data->pdn_pin,"dcam_f");
		gpio_direction_output(dcam_data->pdn_pin,0);
		gpio_request(dcam_data->rst_pin,"dcam_f");
		gpio_direction_output(dcam_data->rst_pin,0);
		ret = of_property_read_u32(node,"sensor_type",&dcam_data->sensor_type);
		if (ret){
			printk("DCAM get sensor_type fail ret(%d)\n",ret);
		}
		
		if (NULL != g_dcam_f_client){	   
			ret =  sysfs_create_file(dcam_kobj, &camera_front_value_attr.attr);
			if (ret)
				CAMERA_DBG(" kernel sysfs_create_file error \r\n");
			ret =  sysfs_create_file(dcam_kobj, &camera_front_open_attr.attr);
			if (ret)
				CAMERA_DBG(" kernel sysfs_create_file error \r\n");
			ret =  sysfs_create_file(dcam_kobj, &camera_front_close_attr.attr);
			if (ret)
				CAMERA_DBG(" kernel sysfs_create_file error \r\n");
		}
	}
	
	//create sysfs
	ret = device_create_file(&client->dev, &dev_attr_enable);
	if (ret){
		printk(KERN_ERR"DCAM failed device_create_file(dev_attr_enable)\n");
	}
	ret = device_create_file(&client->dev, &dev_attr_value);
	if (ret){
		printk(KERN_ERR"DCAM failed device_create_file(dev_attr_value)\n");
	}
	return 0;
}

static int  dcam_i2c_remove(struct i2c_client *client)
{
	struct dcam_data_t *dcam_data = i2c_get_clientdata(client);
	
	device_remove_file(&client->dev, &dev_attr_enable);
	device_remove_file(&client->dev, &dev_attr_value);
	
	printk("DCAM %s\n",client->name);
	if (dcam_data == NULL){
		printk("dcam err ");
		return -1;
	}
	
	if (dcam_data->is_enabled){
		kdCISModulePowerOn(client,false);
	}

	return 0;
}

static int __maybe_unused dcam_i2c_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused dcam_i2c_resume(struct device *dev)
{
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops dcam_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dcam_i2c_suspend, dcam_i2c_resume)
};
#endif

static struct i2c_driver dcam_driver = {
	.driver = {
		.name		= DCAM_NAME,
		.owner		= THIS_MODULE,
		//.of_match_table	= of_match_ptr(sp_cam_of_match),
		.of_match_table	= dcam_of_match,
		.pm		= &dcam_pm_ops,
	},
	.probe		= dcam_i2c_probe,
	.remove		= dcam_i2c_remove,
	.id_table	= dcam_id_table,
};

//module_i2c_driver(dcam_driver);


static int __init dcam_init(void){
	int ret = -1;

	dcam_kobj = kobject_create_and_add("dcam", kernel_kobj);
	if (!dcam_kobj){
		CAMERA_DBG(" kernel kobject_create_and_add error \r\n"); 
		return -1;
	}

	ret = i2c_add_driver(&dcam_driver);
	if (ret != 0){
		i2c_del_driver(&dcam_driver);
	}
	
	return ret;
}
//prize  add  for gc6133/gc0310/gc2145 by zhuzhengjiang    20190116-begin
static void __exit dcam_exit(void){
	
	i2c_del_driver(&dcam_driver);
	
	if (!dcam_kobj){
		return;
	}
	
	if (NULL != g_dcam_r_client){
		sysfs_remove_file(dcam_kobj, &camera_rear_value_attr.attr);
		sysfs_remove_file(dcam_kobj, &camera_rear_open_attr.attr);
		sysfs_remove_file(dcam_kobj, &camera_rear_close_attr.attr);
	}
	if (NULL != g_dcam_f_client){	   
		sysfs_remove_file(dcam_kobj, &camera_front_value_attr.attr);
		sysfs_remove_file(dcam_kobj, &camera_front_open_attr.attr);
		sysfs_remove_file(dcam_kobj, &camera_front_close_attr.attr);
	}
	
	if (dcam_kobj){
		kobject_put(dcam_kobj);
	}
}
//prize  add  for gc6133/gc0310/gc2145 by zhuzhengjiang    20190116-end
module_init(dcam_init);
module_exit(dcam_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("<guoxiong@szprize.com>");
MODULE_DESCRIPTION("Prize double_camera  Driver");


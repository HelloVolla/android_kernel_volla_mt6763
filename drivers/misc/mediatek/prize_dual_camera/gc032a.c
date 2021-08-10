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
#include <linux/delay.h>

#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>


#include "prize_dual_cam.h"


#define GC032A_SENSOR_ID 0x232a


static char i2c_write_reg(struct i2c_client *client, char addr, char reg_data)
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

static char i2c_read_reg(struct i2c_client *client, char addr)
{
	char ret;
	u8 rdbuf[2] = {0};

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

static inline char gc032A_write_cmos_sensor(struct i2c_client *client,char addr, char para)
{
    return i2c_write_reg(client, addr, para);
}
static char gc032A_read_cmos_sensor(struct i2c_client *client,char addr)
{
	char get_byte=0;
   
	get_byte = i2c_read_reg(client, addr);
    return get_byte;
}

static void gc032a_sensor_init(struct i2c_client *client)
{
	/*System*/
	gc032A_write_cmos_sensor(client,0xf3,0xff);
	gc032A_write_cmos_sensor(client,0xf5,0x06);
	gc032A_write_cmos_sensor(client,0xf7,0x01);
	gc032A_write_cmos_sensor(client,0xf8,0x03);
	gc032A_write_cmos_sensor(client,0xf9,0xce); 
	gc032A_write_cmos_sensor(client,0xfa,0x00);
	gc032A_write_cmos_sensor(client,0xfc,0x02);
	gc032A_write_cmos_sensor(client,0xfe,0x02);
	gc032A_write_cmos_sensor(client,0x81,0x03); 

	gc032A_write_cmos_sensor(client,0xfe,0x00);
	gc032A_write_cmos_sensor(client,0x77,0x64);
	gc032A_write_cmos_sensor(client,0x78,0x40);
	gc032A_write_cmos_sensor(client,0x79,0x60);
	/*ANALOG & CISCTL*/
	gc032A_write_cmos_sensor(client,0xfe,0x00);
	gc032A_write_cmos_sensor(client,0x03,0x01);
	gc032A_write_cmos_sensor(client,0x04,0xce);
	gc032A_write_cmos_sensor(client,0x05,0x01);
	gc032A_write_cmos_sensor(client,0x06,0xad);
	gc032A_write_cmos_sensor(client,0x07,0x00);
	gc032A_write_cmos_sensor(client,0x08,0x10);
	gc032A_write_cmos_sensor(client,0x0a,0x00);
	gc032A_write_cmos_sensor(client,0x0c,0x00);
	gc032A_write_cmos_sensor(client,0x0d,0x01);
	gc032A_write_cmos_sensor(client,0x0e,0xe8);
	gc032A_write_cmos_sensor(client,0x0f,0x02);
	gc032A_write_cmos_sensor(client,0x10,0x88);
	gc032A_write_cmos_sensor(client,0x17,0x54);
	gc032A_write_cmos_sensor(client,0x19,0x08);
	gc032A_write_cmos_sensor(client,0x1a,0x0a);
	gc032A_write_cmos_sensor(client,0x1f,0x40);
	gc032A_write_cmos_sensor(client,0x20,0x30);
	gc032A_write_cmos_sensor(client,0x2e,0x80);
	gc032A_write_cmos_sensor(client,0x2f,0x2b);
	gc032A_write_cmos_sensor(client,0x30,0x1a);
	gc032A_write_cmos_sensor(client,0xfe,0x02);
	gc032A_write_cmos_sensor(client,0x03,0x02);
	gc032A_write_cmos_sensor(client,0x05,0xd7);
	gc032A_write_cmos_sensor(client,0x06,0x60);
	gc032A_write_cmos_sensor(client,0x08,0x80);
	gc032A_write_cmos_sensor(client,0x12,0x89);

	/*blk*/
	gc032A_write_cmos_sensor(client,0xfe,0x00);
	gc032A_write_cmos_sensor(client,0x18,0x02);
	gc032A_write_cmos_sensor(client,0xfe,0x02);
	gc032A_write_cmos_sensor(client,0x40,0x22);
	gc032A_write_cmos_sensor(client,0x45,0x00);
	gc032A_write_cmos_sensor(client,0x46,0x00);
	gc032A_write_cmos_sensor(client,0x49,0x20);
	gc032A_write_cmos_sensor(client,0x4b,0x3c);
	gc032A_write_cmos_sensor(client,0x50,0x20);
	gc032A_write_cmos_sensor(client,0x42,0x10);

	/*isp*/
	gc032A_write_cmos_sensor(client,0xfe,0x01);
	gc032A_write_cmos_sensor(client,0x0a,0xc5);
	gc032A_write_cmos_sensor(client,0x45,0x00);
	gc032A_write_cmos_sensor(client,0xfe,0x00);
	gc032A_write_cmos_sensor(client,0x40,0xff);
	gc032A_write_cmos_sensor(client,0x41,0x25);
	gc032A_write_cmos_sensor(client,0x42,0xcf);
	gc032A_write_cmos_sensor(client,0x43,0x10);
	gc032A_write_cmos_sensor(client,0x44,0x83);
	gc032A_write_cmos_sensor(client,0x46,0x22);
	gc032A_write_cmos_sensor(client,0x49,0x03);
	gc032A_write_cmos_sensor(client,0x52,0x02);
	gc032A_write_cmos_sensor(client,0x54,0x00);
	gc032A_write_cmos_sensor(client,0xfe,0x02);
	gc032A_write_cmos_sensor(client,0x22,0xf6);

	/*Shading*/
	gc032A_write_cmos_sensor(client,0xfe,0x01);	
	gc032A_write_cmos_sensor(client,0xc1,0x38);
	gc032A_write_cmos_sensor(client,0xc2,0x4c);
	gc032A_write_cmos_sensor(client,0xc3,0x00);
	gc032A_write_cmos_sensor(client,0xc4,0x32);
	gc032A_write_cmos_sensor(client,0xc5,0x24);
	gc032A_write_cmos_sensor(client,0xc6,0x16);
	gc032A_write_cmos_sensor(client,0xc7,0x08);
	gc032A_write_cmos_sensor(client,0xc8,0x08);
	gc032A_write_cmos_sensor(client,0xc9,0x00);
	gc032A_write_cmos_sensor(client,0xca,0x20);
	gc032A_write_cmos_sensor(client,0xdc,0x8a);
	gc032A_write_cmos_sensor(client,0xdd,0xa0);
	gc032A_write_cmos_sensor(client,0xde,0xa6);
	gc032A_write_cmos_sensor(client,0xdf,0x75);

	/*AWB*/
	gc032A_write_cmos_sensor(client,0xfe,0x01);
	gc032A_write_cmos_sensor(client,0x7c,0x09);
	gc032A_write_cmos_sensor(client,0x65,0x06);
	gc032A_write_cmos_sensor(client,0x7c,0x08);
	gc032A_write_cmos_sensor(client,0x56,0xf4); 
	gc032A_write_cmos_sensor(client,0x66,0x0f); 
	gc032A_write_cmos_sensor(client,0x67,0x84);
	gc032A_write_cmos_sensor(client,0x6b,0x80);
	gc032A_write_cmos_sensor(client,0x6d,0x12);
	gc032A_write_cmos_sensor(client,0x6e,0xb0); 
	gc032A_write_cmos_sensor(client,0x86,0x00);
	gc032A_write_cmos_sensor(client,0x87,0x00);
	gc032A_write_cmos_sensor(client,0x88,0x00);
	gc032A_write_cmos_sensor(client,0x89,0x00);
	gc032A_write_cmos_sensor(client,0x8a,0x00);
	gc032A_write_cmos_sensor(client,0x8b,0x00);
	gc032A_write_cmos_sensor(client,0x8c,0x00);
	gc032A_write_cmos_sensor(client,0x8d,0x00);
	gc032A_write_cmos_sensor(client,0x8e,0x00);
	gc032A_write_cmos_sensor(client,0x8f,0x00);
	gc032A_write_cmos_sensor(client,0x90,0x00);
	gc032A_write_cmos_sensor(client,0x91,0x00);
	gc032A_write_cmos_sensor(client,0x92,0xf4);
	gc032A_write_cmos_sensor(client,0x93,0xd5);
	gc032A_write_cmos_sensor(client,0x94,0x50);
	gc032A_write_cmos_sensor(client,0x95,0x0f);
	gc032A_write_cmos_sensor(client,0x96,0xf4);
	gc032A_write_cmos_sensor(client,0x97,0x2d);
	gc032A_write_cmos_sensor(client,0x98,0x0f);
	gc032A_write_cmos_sensor(client,0x99,0xa6);
	gc032A_write_cmos_sensor(client,0x9a,0x2d);
	gc032A_write_cmos_sensor(client,0x9b,0x0f);
	gc032A_write_cmos_sensor(client,0x9c,0x59);
	gc032A_write_cmos_sensor(client,0x9d,0x2d);
	gc032A_write_cmos_sensor(client,0x9e,0xaa);
	gc032A_write_cmos_sensor(client,0x9f,0x67);
	gc032A_write_cmos_sensor(client,0xa0,0x59);
	gc032A_write_cmos_sensor(client,0xa1,0x00);
	gc032A_write_cmos_sensor(client,0xa2,0x00);
	gc032A_write_cmos_sensor(client,0xa3,0x0a);
	gc032A_write_cmos_sensor(client,0xa4,0x00);
	gc032A_write_cmos_sensor(client,0xa5,0x00);
	gc032A_write_cmos_sensor(client,0xa6,0xd4);
	gc032A_write_cmos_sensor(client,0xa7,0x9f);
	gc032A_write_cmos_sensor(client,0xa8,0x55);
	gc032A_write_cmos_sensor(client,0xa9,0xd4);
	gc032A_write_cmos_sensor(client,0xaa,0x9f);
	gc032A_write_cmos_sensor(client,0xab,0xac);
	gc032A_write_cmos_sensor(client,0xac,0x9f);
	gc032A_write_cmos_sensor(client,0xad,0x55);
	gc032A_write_cmos_sensor(client,0xae,0xd4);
	gc032A_write_cmos_sensor(client,0xaf,0xac);
	gc032A_write_cmos_sensor(client,0xb0,0xd4);
	gc032A_write_cmos_sensor(client,0xb1,0xa3);
	gc032A_write_cmos_sensor(client,0xb2,0x55);
	gc032A_write_cmos_sensor(client,0xb3,0xd4);
	gc032A_write_cmos_sensor(client,0xb4,0xac);
	gc032A_write_cmos_sensor(client,0xb5,0x00);
	gc032A_write_cmos_sensor(client,0xb6,0x00);
	gc032A_write_cmos_sensor(client,0xb7,0x05);
	gc032A_write_cmos_sensor(client,0xb8,0xd6);
	gc032A_write_cmos_sensor(client,0xb9,0x8c);

	/*CC*/
	gc032A_write_cmos_sensor(client,0xfe,0x01);
	gc032A_write_cmos_sensor(client,0xd0,0x40);
	gc032A_write_cmos_sensor(client,0xd1,0xf8);
	gc032A_write_cmos_sensor(client,0xd2,0x00);
	gc032A_write_cmos_sensor(client,0xd3,0xfa);
	gc032A_write_cmos_sensor(client,0xd4,0x45);
	gc032A_write_cmos_sensor(client,0xd5,0x02);

	gc032A_write_cmos_sensor(client,0xd6,0x30);
	gc032A_write_cmos_sensor(client,0xd7,0xfa);
	gc032A_write_cmos_sensor(client,0xd8,0x08);
	gc032A_write_cmos_sensor(client,0xd9,0x08);
	gc032A_write_cmos_sensor(client,0xda,0x58);
	gc032A_write_cmos_sensor(client,0xdb,0x02);
	gc032A_write_cmos_sensor(client,0xfe,0x00);

	/*Gamma*/
	gc032A_write_cmos_sensor(client,0xfe,0x00);
	gc032A_write_cmos_sensor(client,0xba,0x00);
	gc032A_write_cmos_sensor(client,0xbb,0x04);
	gc032A_write_cmos_sensor(client,0xbc,0x0a);
	gc032A_write_cmos_sensor(client,0xbd,0x0e);
	gc032A_write_cmos_sensor(client,0xbe,0x22);
	gc032A_write_cmos_sensor(client,0xbf,0x30);
	gc032A_write_cmos_sensor(client,0xc0,0x3d);
	gc032A_write_cmos_sensor(client,0xc1,0x4a);
	gc032A_write_cmos_sensor(client,0xc2,0x5d);
	gc032A_write_cmos_sensor(client,0xc3,0x6b);
	gc032A_write_cmos_sensor(client,0xc4,0x7a);
	gc032A_write_cmos_sensor(client,0xc5,0x85);
	gc032A_write_cmos_sensor(client,0xc6,0x90);
	gc032A_write_cmos_sensor(client,0xc7,0xa5);
	gc032A_write_cmos_sensor(client,0xc8,0xb5);
	gc032A_write_cmos_sensor(client,0xc9,0xc2);
	gc032A_write_cmos_sensor(client,0xca,0xcc);
	gc032A_write_cmos_sensor(client,0xcb,0xd5);
	gc032A_write_cmos_sensor(client,0xcc,0xde);
	gc032A_write_cmos_sensor(client,0xcd,0xea);
	gc032A_write_cmos_sensor(client,0xce,0xf5);
	gc032A_write_cmos_sensor(client,0xcf,0xff);

	/*Auto Gamma*/          
	gc032A_write_cmos_sensor(client,0xfe,0x00);
	gc032A_write_cmos_sensor(client,0x5a,0x08);
	gc032A_write_cmos_sensor(client,0x5b,0x0f);
	gc032A_write_cmos_sensor(client,0x5c,0x15);
	gc032A_write_cmos_sensor(client,0x5d,0x1c);
	gc032A_write_cmos_sensor(client,0x5e,0x28);
	gc032A_write_cmos_sensor(client,0x5f,0x36);
	gc032A_write_cmos_sensor(client,0x60,0x45);
	gc032A_write_cmos_sensor(client,0x61,0x51);
	gc032A_write_cmos_sensor(client,0x62,0x6a);
	gc032A_write_cmos_sensor(client,0x63,0x7d);
	gc032A_write_cmos_sensor(client,0x64,0x8d);
	gc032A_write_cmos_sensor(client,0x65,0x98);
	gc032A_write_cmos_sensor(client,0x66,0xa2);
	gc032A_write_cmos_sensor(client,0x67,0xb5);
	gc032A_write_cmos_sensor(client,0x68,0xc3);
	gc032A_write_cmos_sensor(client,0x69,0xcd);
	gc032A_write_cmos_sensor(client,0x6a,0xd4);
	gc032A_write_cmos_sensor(client,0x6b,0xdc);
	gc032A_write_cmos_sensor(client,0x6c,0xe3);
	gc032A_write_cmos_sensor(client,0x6d,0xf0);
	gc032A_write_cmos_sensor(client,0x6e,0xf9);
	gc032A_write_cmos_sensor(client,0x6f,0xff);

	/*Gain*/
	gc032A_write_cmos_sensor(client,0xfe,0x00);
	gc032A_write_cmos_sensor(client,0x70,0x50);

	/*AEC*/
	gc032A_write_cmos_sensor(client,0xfe,0x00);
	gc032A_write_cmos_sensor(client,0x4f,0x01);
	gc032A_write_cmos_sensor(client,0xfe,0x01);
	gc032A_write_cmos_sensor(client,0x0d,0x00);
	gc032A_write_cmos_sensor(client,0x12,0xa0);
	gc032A_write_cmos_sensor(client,0x13,0x3a);	
	gc032A_write_cmos_sensor(client,0x44,0x04);
	gc032A_write_cmos_sensor(client,0x1f,0x30);
	gc032A_write_cmos_sensor(client,0x20,0x40);	
	gc032A_write_cmos_sensor(client,0x26,0x9a);
	gc032A_write_cmos_sensor(client,0x3e,0x20);
	gc032A_write_cmos_sensor(client,0x3f,0x2d);
	gc032A_write_cmos_sensor(client,0x40,0x40);
	gc032A_write_cmos_sensor(client,0x41,0x5b);
	gc032A_write_cmos_sensor(client,0x42,0x82);
	gc032A_write_cmos_sensor(client,0x43,0xb7);
	gc032A_write_cmos_sensor(client,0x04,0x0a);
	gc032A_write_cmos_sensor(client,0x02,0x79);
	gc032A_write_cmos_sensor(client,0x03,0xc0);

	/*measure window*/
	gc032A_write_cmos_sensor(client,0xfe,0x01);
	gc032A_write_cmos_sensor(client,0xcc,0x08);
	gc032A_write_cmos_sensor(client,0xcd,0x08);
	gc032A_write_cmos_sensor(client,0xce,0xa4);
	gc032A_write_cmos_sensor(client,0xcf,0xec);

	/*DNDD*/
	gc032A_write_cmos_sensor(client,0xfe,0x00);
	gc032A_write_cmos_sensor(client,0x81,0xb8);
	gc032A_write_cmos_sensor(client,0x82,0x12);
	gc032A_write_cmos_sensor(client,0x83,0x0a);
	gc032A_write_cmos_sensor(client,0x84,0x01);
	gc032A_write_cmos_sensor(client,0x86,0x50);
	gc032A_write_cmos_sensor(client,0x87,0x18);
	gc032A_write_cmos_sensor(client,0x88,0x10);
	gc032A_write_cmos_sensor(client,0x89,0x70);
	gc032A_write_cmos_sensor(client,0x8a,0x20);
	gc032A_write_cmos_sensor(client,0x8b,0x10);
	gc032A_write_cmos_sensor(client,0x8c,0x08);
	gc032A_write_cmos_sensor(client,0x8d,0x0a);

	/*Intpee*/
	gc032A_write_cmos_sensor(client,0xfe,0x00);
	gc032A_write_cmos_sensor(client,0x8f,0xaa);
	gc032A_write_cmos_sensor(client,0x90,0x9c);
	gc032A_write_cmos_sensor(client,0x91,0x52);
	gc032A_write_cmos_sensor(client,0x92,0x03);
	gc032A_write_cmos_sensor(client,0x93,0x03);
	gc032A_write_cmos_sensor(client,0x94,0x08);
	gc032A_write_cmos_sensor(client,0x95,0x44);
	gc032A_write_cmos_sensor(client,0x97,0x00);
	gc032A_write_cmos_sensor(client,0x98,0x00);

	/*ASDE*/
	gc032A_write_cmos_sensor(client,0xfe,0x00);
	gc032A_write_cmos_sensor(client,0xa1,0x30);
	gc032A_write_cmos_sensor(client,0xa2,0x41);
	gc032A_write_cmos_sensor(client,0xa4,0x30);
	gc032A_write_cmos_sensor(client,0xa5,0x20);
	gc032A_write_cmos_sensor(client,0xaa,0x30);
	gc032A_write_cmos_sensor(client,0xac,0x32);

	/*YCP*/
	gc032A_write_cmos_sensor(client,0xfe,0x00);
	gc032A_write_cmos_sensor(client,0xd1,0x3c);
	gc032A_write_cmos_sensor(client,0xd2,0x3c);
	gc032A_write_cmos_sensor(client,0xd3,0x38);
	gc032A_write_cmos_sensor(client,0xd6,0xf4);
	gc032A_write_cmos_sensor(client,0xd7,0x1d);
	gc032A_write_cmos_sensor(client,0xdd,0x73);
	gc032A_write_cmos_sensor(client,0xde,0x84);

	/*Banding*/
	gc032A_write_cmos_sensor(client,0xfe,0x00);
	gc032A_write_cmos_sensor(client,0x05,0x01);
	gc032A_write_cmos_sensor(client,0x06,0xad);
	gc032A_write_cmos_sensor(client,0x07,0x00);
	gc032A_write_cmos_sensor(client,0x08,0x10);

	gc032A_write_cmos_sensor(client,0xfe,0x01);
	gc032A_write_cmos_sensor(client,0x25,0x00);
	gc032A_write_cmos_sensor(client,0x26,0x9a);

	gc032A_write_cmos_sensor(client,0x27,0x01);
	gc032A_write_cmos_sensor(client,0x28,0xce);
	gc032A_write_cmos_sensor(client,0x29,0x03);
	gc032A_write_cmos_sensor(client,0x2a,0x02);
	gc032A_write_cmos_sensor(client,0x2b,0x04);
	gc032A_write_cmos_sensor(client,0x2c,0x36);
	gc032A_write_cmos_sensor(client,0x2d,0x07);
	gc032A_write_cmos_sensor(client,0x2e,0xd2);
	gc032A_write_cmos_sensor(client,0x2f,0x0b);
	gc032A_write_cmos_sensor(client,0x30,0x6e);
	gc032A_write_cmos_sensor(client,0x31,0x0e);
	gc032A_write_cmos_sensor(client,0x32,0x70);
	gc032A_write_cmos_sensor(client,0x33,0x12);
	gc032A_write_cmos_sensor(client,0x34,0x0c);
	gc032A_write_cmos_sensor(client,0x3c,0x30);
	gc032A_write_cmos_sensor(client,0xfe,0x00);

}

static void gc032a_write_more_registers(struct i2c_client *client)
{
	////////////////////for FAE to modify the necessary Init Regs.////////////////

}

static void gc032a_stream_on(struct i2c_client *client)
{

}

static unsigned short gc032a_read_shutter(struct i2c_client *client)
{
    unsigned char temp_reg1, temp_reg2;
	unsigned short shutter;

	temp_reg1 = gc032A_read_cmos_sensor(client,0x04);
	temp_reg2 = gc032A_read_cmos_sensor(client,0x03);

	shutter = (temp_reg1 & 0xFF) | (temp_reg2 << 8);

	return shutter;
} /* gc032a_read_shutter */

static unsigned int gc032a_get_sensor_id(struct i2c_client *client,unsigned int *sensorID)
{
    *sensorID=((gc032A_read_cmos_sensor(client,0xf0)<< 8)|gc032A_read_cmos_sensor(client,0xf1));
	CAMERA_DBG("GC032A Read ID %x",*sensorID);

    return 0;    
}

static int gc032a_set_power(struct i2c_client *client,unsigned int enable)
{
	struct spc_data_t *spc_data = i2c_get_clientdata(client);
	
    int  ret = 0;
	if (enable){
		gpio_direction_output(spc_data->pdn_pin,1);
		mdelay(4);
		gpio_direction_output(spc_data->pdn_pin,0);
	}else{
		gpio_direction_output(spc_data->pdn_pin,1);
	}
    return ret;    
}


static int gc032a_open(struct i2c_client *client)
{
	volatile signed char i;
	unsigned short sensor_id=0;

	CAMERA_DBG("<Jet> Entry Open!!!\r\n");

	//  Read sensor ID to adjust I2C is OK?
	for(i=0;i<3;i++)
	{
		sensor_id = ((gc032A_read_cmos_sensor(client,0xf0) << 8) | gc032A_read_cmos_sensor(client,0xf1));
		if(sensor_id != GC032A_SENSOR_ID)  
		{
			CAMERA_DBG("Read Sensor ID Fail[open] = 0x%x\n", sensor_id); 
			return -EINVAL;
		}
	}
	
	CAMERA_DBG("GC032A_ Sensor Read ID OK \r\n");
	gc032a_sensor_init(client);
	gc032a_write_more_registers(client);

	return 0;
} /* GC032AOpen */


const struct sensor_info_t gc032a_info = {
	.sensor_type = SENSOR_TYPE_032A,
	.sensor_id = GC032A_SENSOR_ID,
	.open = gc032a_open,
	.init = gc032a_sensor_init,
	.stream_on = gc032a_stream_on,
	.get_shutter = gc032a_read_shutter,
	.get_sensor_id = gc032a_get_sensor_id,
	.set_power = gc032a_set_power,
};
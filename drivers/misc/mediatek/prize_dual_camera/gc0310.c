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


#define GC0310_SENSOR_ID 0xa310


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

static inline char gc0310_write_cmos_sensor(struct i2c_client *client, char addr, char para)
{
    return i2c_write_reg(client, addr, para);
}
static char gc0310_read_cmos_sensor(struct i2c_client *client, char addr)
{
	char get_byte=0;
   
	get_byte = i2c_read_reg(client, addr);
    return get_byte;
}

static void gc0310_sensor_init(struct i2c_client *client)
{
    gc0310_write_cmos_sensor(client,0xfe,0xf0);
    gc0310_write_cmos_sensor(client,0xfe,0xf0);
    gc0310_write_cmos_sensor(client,0xfe,0x00);
    gc0310_write_cmos_sensor(client,0xfc,0x0e);
    gc0310_write_cmos_sensor(client,0xfc,0x0e);
    gc0310_write_cmos_sensor(client,0xf2,0x80);
    gc0310_write_cmos_sensor(client,0xf3,0x00);
    gc0310_write_cmos_sensor(client,0xf7,0x1b);
    gc0310_write_cmos_sensor(client,0xf8,0x04);  // from 03 to 04
    gc0310_write_cmos_sensor(client,0xf9,0x8e);
    gc0310_write_cmos_sensor(client,0xfa,0x11);
     /////////////////////////////////////////////////      
	///////////////////   MIPI   ////////////////////      
	/////////////////////////////////////////////////      
	gc0310_write_cmos_sensor(client,0xfe,0x03);
	gc0310_write_cmos_sensor(client,0x40,0x08);
	gc0310_write_cmos_sensor(client,0x42,0x00);
	gc0310_write_cmos_sensor(client,0x43,0x00);
	gc0310_write_cmos_sensor(client,0x01,0x03);
	gc0310_write_cmos_sensor(client,0x10,0x84);
                                        
	gc0310_write_cmos_sensor(client,0x01,0x03);             
	gc0310_write_cmos_sensor(client,0x02,0x00);             
	gc0310_write_cmos_sensor(client,0x03,0x94);             
	gc0310_write_cmos_sensor(client,0x04,0x01);            
	gc0310_write_cmos_sensor(client,0x05,0x40);  // 40      20     
	gc0310_write_cmos_sensor(client,0x06,0x80);             
	gc0310_write_cmos_sensor(client,0x11,0x1e);             
	gc0310_write_cmos_sensor(client,0x12,0x00);      
	gc0310_write_cmos_sensor(client,0x13,0x05);             
	gc0310_write_cmos_sensor(client,0x15,0x10);                                                                    
	gc0310_write_cmos_sensor(client,0x21,0x10);             
	gc0310_write_cmos_sensor(client,0x22,0x01);             
	gc0310_write_cmos_sensor(client,0x23,0x10);                                             
	gc0310_write_cmos_sensor(client,0x24,0x02);                                             
	gc0310_write_cmos_sensor(client,0x25,0x10);                                             
	gc0310_write_cmos_sensor(client,0x26,0x03);                                             
	gc0310_write_cmos_sensor(client,0x29,0x02); //02                                            
	gc0310_write_cmos_sensor(client,0x2a,0x0a);   //0a                                          
	gc0310_write_cmos_sensor(client,0x2b,0x04);                                             
	gc0310_write_cmos_sensor(client,0xfe,0x00);
        /////////////////////////////////////////////////
        /////////////////   CISCTL reg  /////////////////
        /////////////////////////////////////////////////
    gc0310_write_cmos_sensor(client,0x00,0x2f);
    gc0310_write_cmos_sensor(client,0x01,0x0f);
    gc0310_write_cmos_sensor(client,0x02,0x04);
    gc0310_write_cmos_sensor(client,0x03,0x04);
    gc0310_write_cmos_sensor(client,0x04,0xd0);
    gc0310_write_cmos_sensor(client,0x09,0x00);
    gc0310_write_cmos_sensor(client,0x0a,0x00);
    gc0310_write_cmos_sensor(client,0x0b,0x00);
    gc0310_write_cmos_sensor(client,0x0c,0x06);
    gc0310_write_cmos_sensor(client,0x0d,0x01);
    gc0310_write_cmos_sensor(client,0x0e,0xe8);
    gc0310_write_cmos_sensor(client,0x0f,0x02);
    gc0310_write_cmos_sensor(client,0x10,0x88);
    gc0310_write_cmos_sensor(client,0x16,0x00);
    gc0310_write_cmos_sensor(client,0x17,0x14);
    gc0310_write_cmos_sensor(client,0x18,0x1a);
    gc0310_write_cmos_sensor(client,0x19,0x14);
    gc0310_write_cmos_sensor(client,0x1b,0x48);
    gc0310_write_cmos_sensor(client,0x1e,0x6b);
    gc0310_write_cmos_sensor(client,0x1f,0x28);
    gc0310_write_cmos_sensor(client,0x20,0x8b);  // from 89 to 8b
    gc0310_write_cmos_sensor(client,0x21,0x49);
    gc0310_write_cmos_sensor(client,0x22,0xb0);
    gc0310_write_cmos_sensor(client,0x23,0x04);
    gc0310_write_cmos_sensor(client,0x24,0x16);
    gc0310_write_cmos_sensor(client,0x34,0x20);
        
        /////////////////////////////////////////////////
        ////////////////////   BLK   ////////////////////
        /////////////////////////////////////////////////
    gc0310_write_cmos_sensor(client,0x26,0x23); 
    gc0310_write_cmos_sensor(client,0x28,0xff); 
    gc0310_write_cmos_sensor(client,0x29,0x00); 
    gc0310_write_cmos_sensor(client,0x33,0x10); 
    gc0310_write_cmos_sensor(client,0x37,0x20); 
	gc0310_write_cmos_sensor(client,0x38,0x10); 
    gc0310_write_cmos_sensor(client,0x47,0x80); 
    gc0310_write_cmos_sensor(client,0x4e,0x66); 
    gc0310_write_cmos_sensor(client,0xa8,0x02); 
    gc0310_write_cmos_sensor(client,0xa9,0x80);
        
        /////////////////////////////////////////////////
        //////////////////   ISP reg  ///////////////////
        /////////////////////////////////////////////////
    gc0310_write_cmos_sensor(client,0x40,0xff); 
    gc0310_write_cmos_sensor(client,0x41,0x21); 
    gc0310_write_cmos_sensor(client,0x42,0xcf); 
    gc0310_write_cmos_sensor(client,0x44,0x01); // 02 yuv 
    gc0310_write_cmos_sensor(client,0x45,0xa0); // from a8 - a4 a4-a0
    gc0310_write_cmos_sensor(client,0x46,0x03); 
    gc0310_write_cmos_sensor(client,0x4a,0x11);
    gc0310_write_cmos_sensor(client,0x4b,0x01);
    gc0310_write_cmos_sensor(client,0x4c,0x20); 
    gc0310_write_cmos_sensor(client,0x4d,0x05); 
    gc0310_write_cmos_sensor(client,0x4f,0x01);
    gc0310_write_cmos_sensor(client,0x50,0x01); 
    gc0310_write_cmos_sensor(client,0x55,0x01); 
    gc0310_write_cmos_sensor(client,0x56,0xe0);
    gc0310_write_cmos_sensor(client,0x57,0x02); 
    gc0310_write_cmos_sensor(client,0x58,0x80);
        
        /////////////////////////////////////////////////  
        ///////////////////   GAIN   ////////////////////
        /////////////////////////////////////////////////
    gc0310_write_cmos_sensor(client,0x70,0x70); 
    gc0310_write_cmos_sensor(client,0x5a,0x84); 
    gc0310_write_cmos_sensor(client,0x5b,0xc9); 
    gc0310_write_cmos_sensor(client,0x5c,0xed); 
    gc0310_write_cmos_sensor(client,0x77,0x74); 
    gc0310_write_cmos_sensor(client,0x78,0x40); 
    gc0310_write_cmos_sensor(client,0x79,0x5f); 
        
        ///////////////////////////////////////////////// 
        ///////////////////   DNDD  /////////////////////
        ///////////////////////////////////////////////// 
    gc0310_write_cmos_sensor(client,0x82,0x1f); 
    gc0310_write_cmos_sensor(client,0x83,0x0b);
        
        
        ///////////////////////////////////////////////// 
        //////////////////   EEINTP  ////////////////////
        ///////////////////////////////////////////////// 
    gc0310_write_cmos_sensor(client,0x8f,0xff); 
    gc0310_write_cmos_sensor(client,0x90,0x9f); 
    gc0310_write_cmos_sensor(client,0x91,0x90); 
    gc0310_write_cmos_sensor(client,0x92,0x03); 
    gc0310_write_cmos_sensor(client,0x93,0x03); 
    gc0310_write_cmos_sensor(client,0x94,0x05);
    gc0310_write_cmos_sensor(client,0x95,0x65); 
    gc0310_write_cmos_sensor(client,0x96,0xf0); 
        
        ///////////////////////////////////////////////// 
        /////////////////////  ASDE  ////////////////////
        ///////////////////////////////////////////////// 
    gc0310_write_cmos_sensor(client,0xfe,0x00);
    gc0310_write_cmos_sensor(client,0x9a,0x20);
    gc0310_write_cmos_sensor(client,0x9b,0x80);
    gc0310_write_cmos_sensor(client,0x9c,0x40);
    gc0310_write_cmos_sensor(client,0x9d,0x80);
    gc0310_write_cmos_sensor(client,0xa1,0x30);
    gc0310_write_cmos_sensor(client,0xa2,0x32);
    gc0310_write_cmos_sensor(client,0xa4,0x30);
    gc0310_write_cmos_sensor(client,0xa5,0x30);
    gc0310_write_cmos_sensor(client,0xaa,0x50);
    gc0310_write_cmos_sensor(client,0xac,0x22);
        
        /////////////////////////////////////////////////
        ///////////////////   GAMMA   ///////////////////
        /////////////////////////////////////////////////
    gc0310_write_cmos_sensor(client,0xbf,0x08); 
    gc0310_write_cmos_sensor(client,0xc0,0x16); 
    gc0310_write_cmos_sensor(client,0xc1,0x28); 
    gc0310_write_cmos_sensor(client,0xc2,0x41); 
    gc0310_write_cmos_sensor(client,0xc3,0x5a); 
    gc0310_write_cmos_sensor(client,0xc4,0x6c); 
    gc0310_write_cmos_sensor(client,0xc5,0x7a); 
    gc0310_write_cmos_sensor(client,0xc6,0x96); 
    gc0310_write_cmos_sensor(client,0xc7,0xac); 
    gc0310_write_cmos_sensor(client,0xc8,0xbc); 
    gc0310_write_cmos_sensor(client,0xc9,0xc9); 
    gc0310_write_cmos_sensor(client,0xca,0xd3); 
    gc0310_write_cmos_sensor(client,0xcb,0xdd); 
    gc0310_write_cmos_sensor(client,0xcc,0xe5); 
    gc0310_write_cmos_sensor(client,0xcd,0xf1); 
    gc0310_write_cmos_sensor(client,0xce,0xfa); 
    gc0310_write_cmos_sensor(client,0xcf,0xff);
        
        /////////////////////////////////////////////////
        ///////////////////   YCP  //////////////////////
        /////////////////////////////////////////////////
    gc0310_write_cmos_sensor(client,0xd0,0x40); 
    gc0310_write_cmos_sensor(client,0xd1,0x34); 
    gc0310_write_cmos_sensor(client,0xd2,0x34); 
    gc0310_write_cmos_sensor(client,0xd3,0x3c); 
    gc0310_write_cmos_sensor(client,0xd6,0xf2); 
    gc0310_write_cmos_sensor(client,0xd7,0x1b); 
    gc0310_write_cmos_sensor(client,0xd8,0x18); 
    gc0310_write_cmos_sensor(client,0xdd,0x03); 
        /////////////////////////////////////////////////
        ////////////////////   AEC   ////////////////////
        /////////////////////////////////////////////////
    gc0310_write_cmos_sensor(client,0xfe,0x01);
    gc0310_write_cmos_sensor(client,0x05,0x30); 
    gc0310_write_cmos_sensor(client,0x06,0x75); 
    gc0310_write_cmos_sensor(client,0x07,0x40); 
    gc0310_write_cmos_sensor(client,0x08,0xb0); 
    gc0310_write_cmos_sensor(client,0x0a,0xc5); 
    gc0310_write_cmos_sensor(client,0x0b,0x11);
    gc0310_write_cmos_sensor(client,0x0c,0x00); 
    gc0310_write_cmos_sensor(client,0x12,0x52);
    gc0310_write_cmos_sensor(client,0x13,0x08);  //0x38 ---28
    gc0310_write_cmos_sensor(client,0x18,0x95);
    gc0310_write_cmos_sensor(client,0x19,0x96);
    gc0310_write_cmos_sensor(client,0x1f,0x20);
    gc0310_write_cmos_sensor(client,0x20,0xc0); 
    gc0310_write_cmos_sensor(client,0x3e,0x40); 
    gc0310_write_cmos_sensor(client,0x3f,0x57); 
    gc0310_write_cmos_sensor(client,0x40,0x7d); 
    gc0310_write_cmos_sensor(client,0x03,0x60); 
    gc0310_write_cmos_sensor(client,0x44,0x02); 
    /////////////////////////////////////////////////
    ////////////////////   AWB   ////////////////////
    /////////////////////////////////////////////////
    gc0310_write_cmos_sensor(client,0x1c,0x91); 
    gc0310_write_cmos_sensor(client,0x21,0x15); 
    gc0310_write_cmos_sensor(client,0x50,0x80);
    gc0310_write_cmos_sensor(client,0x56,0x04);
    gc0310_write_cmos_sensor(client,0x58,0x08);    
    gc0310_write_cmos_sensor(client,0x59,0x08); 
    gc0310_write_cmos_sensor(client,0x5b,0x82);  // 02 to 82 to 02
    gc0310_write_cmos_sensor(client,0x61,0x8d); 
    gc0310_write_cmos_sensor(client,0x62,0xa7); 
    gc0310_write_cmos_sensor(client,0x63,0x00);   // d0 to  00
    gc0310_write_cmos_sensor(client,0x65,0x06);
    gc0310_write_cmos_sensor(client,0x66,0x06);   // 06 to 03
    gc0310_write_cmos_sensor(client,0x67,0x84); 
    gc0310_write_cmos_sensor(client,0x69,0x08);   // 08 to 20
    gc0310_write_cmos_sensor(client,0x6a,0x25); 
    gc0310_write_cmos_sensor(client,0x6b,0x01); 
    gc0310_write_cmos_sensor(client,0x6c,0x00);   // 00 to 0c
    gc0310_write_cmos_sensor(client,0x6d,0x02); 
    gc0310_write_cmos_sensor(client,0x6e,0x00);  // f0 to 00
    gc0310_write_cmos_sensor(client,0x6f,0x80); 
    gc0310_write_cmos_sensor(client,0x76,0x80); 
    gc0310_write_cmos_sensor(client,0x78,0xaf); 
    gc0310_write_cmos_sensor(client,0x79,0x75);
    gc0310_write_cmos_sensor(client,0x7a,0x40);
    gc0310_write_cmos_sensor(client,0x7b,0x50);
    gc0310_write_cmos_sensor(client,0x7c,0x08); //0c to 08 8.11
         
    gc0310_write_cmos_sensor(client,0xa4,0xb9); 
    gc0310_write_cmos_sensor(client,0xa5,0xa0);
    gc0310_write_cmos_sensor(client,0x90,0xc9); 
    gc0310_write_cmos_sensor(client,0x91,0xbe);
    gc0310_write_cmos_sensor(client,0xa6,0xb8); 
    gc0310_write_cmos_sensor(client,0xa7,0x95); 
    gc0310_write_cmos_sensor(client,0x92,0xe6); 
    gc0310_write_cmos_sensor(client,0x93,0xca); 
    gc0310_write_cmos_sensor(client,0xa9,0xb6); 
    gc0310_write_cmos_sensor(client,0xaa,0x89); 
    gc0310_write_cmos_sensor(client,0x95,0x23); 
    gc0310_write_cmos_sensor(client,0x96,0xe7); 
    gc0310_write_cmos_sensor(client,0xab,0x9d); 
    gc0310_write_cmos_sensor(client,0xac,0x80);
    gc0310_write_cmos_sensor(client,0x97,0x43); 
    gc0310_write_cmos_sensor(client,0x98,0x24); 
    gc0310_write_cmos_sensor(client,0xae,0xd0);   // b7 to d0
    gc0310_write_cmos_sensor(client,0xaf,0x9e); 
    gc0310_write_cmos_sensor(client,0x9a,0x43);
    gc0310_write_cmos_sensor(client,0x9b,0x24); 
        
    gc0310_write_cmos_sensor(client,0xb0,0xc0);  // c8 to c0
    gc0310_write_cmos_sensor(client,0xb1,0xa8);   // 97 to a8
    gc0310_write_cmos_sensor(client,0x9c,0xc4); 
    gc0310_write_cmos_sensor(client,0x9d,0x44); 
    gc0310_write_cmos_sensor(client,0xb3,0xb7); 
    gc0310_write_cmos_sensor(client,0xb4,0x7f);
    gc0310_write_cmos_sensor(client,0x9f,0xc7);
    gc0310_write_cmos_sensor(client,0xa0,0xc8); 
    gc0310_write_cmos_sensor(client,0xb5,0x00); 
    gc0310_write_cmos_sensor(client,0xb6,0x00);
    gc0310_write_cmos_sensor(client,0xa1,0x00);
    gc0310_write_cmos_sensor(client,0xa2,0x00);
    gc0310_write_cmos_sensor(client,0x86,0x60);
    gc0310_write_cmos_sensor(client,0x87,0x08);
    gc0310_write_cmos_sensor(client,0x88,0x00);
    gc0310_write_cmos_sensor(client,0x89,0x00);
    gc0310_write_cmos_sensor(client,0x8b,0xde);
    gc0310_write_cmos_sensor(client,0x8c,0x80);
    gc0310_write_cmos_sensor(client,0x8d,0x00);
    gc0310_write_cmos_sensor(client,0x8e,0x00);
    gc0310_write_cmos_sensor(client,0x94,0x55);
    gc0310_write_cmos_sensor(client,0x99,0xa6);
    gc0310_write_cmos_sensor(client,0x9e,0xaa);
    gc0310_write_cmos_sensor(client,0xa3,0x0a);
    gc0310_write_cmos_sensor(client,0x8a,0x0a);
    gc0310_write_cmos_sensor(client,0xa8,0x55);
    gc0310_write_cmos_sensor(client,0xad,0x55);
    gc0310_write_cmos_sensor(client,0xb2,0x55);
    gc0310_write_cmos_sensor(client,0xb7,0x05);
    gc0310_write_cmos_sensor(client,0x8f,0x05);
    gc0310_write_cmos_sensor(client,0xb8,0xcc);
    gc0310_write_cmos_sensor(client,0xb9,0x9a);
        
        /////////////////////////////////////
        ////////////////////  CC ////////////
        /////////////////////////////////////
    gc0310_write_cmos_sensor(client,0xfe,0x01);
    gc0310_write_cmos_sensor(client,0xd0,0x38);
    gc0310_write_cmos_sensor(client,0xd1,0xfd);
    gc0310_write_cmos_sensor(client,0xd2,0x06);
    gc0310_write_cmos_sensor(client,0xd3,0xf0);
    gc0310_write_cmos_sensor(client,0xd4,0x40);
    gc0310_write_cmos_sensor(client,0xd5,0x08);
    gc0310_write_cmos_sensor(client,0xd6,0x30);
    gc0310_write_cmos_sensor(client,0xd7,0x00);
    gc0310_write_cmos_sensor(client,0xd8,0x0a);
    gc0310_write_cmos_sensor(client,0xd9,0x16);
    gc0310_write_cmos_sensor(client,0xda,0x39);
    gc0310_write_cmos_sensor(client,0xdb,0xf8);
        
        /////////////////////////////////////////////////
        ////////////////////   LSC   ////////////////////
        /////////////////////////////////////////////////
    gc0310_write_cmos_sensor(client,0xfe,0x01); 
    gc0310_write_cmos_sensor(client,0xc1,0x3c); 
    gc0310_write_cmos_sensor(client,0xc2,0x50); 
    gc0310_write_cmos_sensor(client,0xc3,0x00); 
    gc0310_write_cmos_sensor(client,0xc4,0x40); 
    gc0310_write_cmos_sensor(client,0xc5,0x30); 
    gc0310_write_cmos_sensor(client,0xc6,0x30); 
    gc0310_write_cmos_sensor(client,0xc7,0x10); 
    gc0310_write_cmos_sensor(client,0xc8,0x00); 
    gc0310_write_cmos_sensor(client,0xc9,0x00); 
    gc0310_write_cmos_sensor(client,0xdc,0x20); 
    gc0310_write_cmos_sensor(client,0xdd,0x10); 
    gc0310_write_cmos_sensor(client,0xdf,0x00); 
    gc0310_write_cmos_sensor(client,0xde,0x00); 
        
        /////////////////////////////////////////////////
        ///////////////////  Histogram  /////////////////
        /////////////////////////////////////////////////
    gc0310_write_cmos_sensor(client,0x01,0x10); 
    gc0310_write_cmos_sensor(client,0x0b,0x31); 
    gc0310_write_cmos_sensor(client,0x0e,0x50); 
    gc0310_write_cmos_sensor(client,0x0f,0x0f); 
    gc0310_write_cmos_sensor(client,0x10,0x6e); 
    gc0310_write_cmos_sensor(client,0x12,0xa0); 
    gc0310_write_cmos_sensor(client,0x15,0x60); 
    gc0310_write_cmos_sensor(client,0x16,0x60); 
    gc0310_write_cmos_sensor(client,0x17,0xe0); 
        
        /////////////////////////////////////////////////
        //////////////   Measure Window   ///////////////
        /////////////////////////////////////////////////
    gc0310_write_cmos_sensor(client,0xcc,0x0c);  
    gc0310_write_cmos_sensor(client,0xcd,0x10); 
    gc0310_write_cmos_sensor(client,0xce,0xa0); 
    gc0310_write_cmos_sensor(client,0xcf,0xe6); 
        
        /////////////////////////////////////////////////
        /////////////////   dark sun   //////////////////
        /////////////////////////////////////////////////
    gc0310_write_cmos_sensor(client,0x45,0xf7);
    gc0310_write_cmos_sensor(client,0x46,0xff); 
    gc0310_write_cmos_sensor(client,0x47,0x15);
    gc0310_write_cmos_sensor(client,0x48,0x03); 
    gc0310_write_cmos_sensor(client,0x4f,0x60); 
        
        /////////////////////////////////////////////////
        ///////////////////  banding  ///////////////////
        /////////////////////////////////////////////////
    gc0310_write_cmos_sensor(client,0xfe,0x00);
    gc0310_write_cmos_sensor(client,0x05,0x01);
    gc0310_write_cmos_sensor(client,0x06,0x18); //HB#if 1    
    gc0310_write_cmos_sensor(client,0x07,0x00);
    gc0310_write_cmos_sensor(client,0x08,0x10); //VB  from 10 to 50

    gc0310_write_cmos_sensor(client,0xfe,0x01);
    gc0310_write_cmos_sensor(client,0x25,0x00); //step 
    gc0310_write_cmos_sensor(client,0x26,0x9a); 
    gc0310_write_cmos_sensor(client,0x27,0x01); //30fps
    gc0310_write_cmos_sensor(client,0x28,0xce);  
    gc0310_write_cmos_sensor(client,0x29,0x04); //12.5fps
	gc0310_write_cmos_sensor(client,0x2a,0x36); 
	gc0310_write_cmos_sensor(client,0x2b,0x06); //10fps
	gc0310_write_cmos_sensor(client,0x2c,0x04); 
	gc0310_write_cmos_sensor(client,0x2d,0x0c); //5fps
	gc0310_write_cmos_sensor(client,0x2e,0x08);
    gc0310_write_cmos_sensor(client,0x3c,0x20);
        
        /////////////////////////////////////////////////
        ///////////////////   MIPI   ////////////////////
        /////////////////////////////////////////////////
   gc0310_write_cmos_sensor(client,0xfe,0x03);
   gc0310_write_cmos_sensor(client,0x10,0x94);  
   gc0310_write_cmos_sensor(client,0xfe,0x00); 
	
}

static void gc0310_stream_on(struct i2c_client *client)
{
	//msleep(150);
    gc0310_write_cmos_sensor(client,0xfe,0x03);
    gc0310_write_cmos_sensor(client,0x10,0x94);
    gc0310_write_cmos_sensor(client,0xfe,0x00);
    //msleep(50);
}

static unsigned short gc0310_read_shutter(struct i2c_client *client)
{
    unsigned char temp_reg1, temp_reg2;
	unsigned short shutter;
	
	temp_reg1 = gc0310_read_cmos_sensor(client,0x04);
	temp_reg2 = gc0310_read_cmos_sensor(client,0x03);

	shutter = (temp_reg1 & 0xFF) | (temp_reg2 << 8);
	//CAMERA_DBG("GC0310MIPI_Read_Shutter %d\r\n",shutter);
	return shutter;
}

static unsigned int gc0310_get_sensor_id(struct i2c_client *client,unsigned int *sensorID)
{
    // check if sensor ID correct
    *sensorID=((gc0310_read_cmos_sensor(client,0xf0)<< 8)|gc0310_read_cmos_sensor(client,0xf1));
	CAMERA_DBG("GC0310 Read ID %x",*sensorID);

    return 0;    
}

static int gc0310_set_power(struct i2c_client *client,unsigned int enable)
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


static int gc0310_open(struct i2c_client *client)
{
	volatile signed char i;
	unsigned short sensor_id=0;
	int id_status = 0;

	CAMERA_DBG("<Jet> Entry Open!!!");

	//  Read sensor ID to adjust I2C is OK?
	for(i=0;i<3;i++)
	{
		sensor_id = ((gc0310_read_cmos_sensor(client,0xf0) << 8) | gc0310_read_cmos_sensor(client,0xf1));
		CAMERA_DBG("*sensorID=%x %s",sensor_id,__func__);
		if(sensor_id != GC0310_SENSOR_ID)  
		{
			CAMERA_DBG("Read Sensor ID Fail[open] = 0x%x\n", sensor_id); 
			return -EINVAL;
		}else{
			id_status = 1;
			break;
		}
	}
	if (!id_status){
		return -EINVAL;
	}
	
	CAMERA_DBG("GC0310mipi_ Sensor Read ID OK \r\n");
	gc0310_sensor_init(client);

	return 0;
}

const struct sensor_info_t gc0310_info = {
	.sensor_type = SENSOR_TYPE_0310,
	.sensor_id = GC0310_SENSOR_ID,
	.open = gc0310_open,
	.init = gc0310_sensor_init,
	.stream_on = gc0310_stream_on,
	.get_shutter = gc0310_read_shutter,
	.get_sensor_id = gc0310_get_sensor_id,
	.set_power = gc0310_set_power,
};
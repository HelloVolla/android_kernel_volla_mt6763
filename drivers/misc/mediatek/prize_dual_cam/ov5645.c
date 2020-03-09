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


#define OV5645_SENSOR_ID 0x5645


static char i2c_write_reg(struct i2c_client *client, int addr, char reg_data)
{
	char ret;
	u8 wdbuf[512] = {0};

	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 3,
			.buf	= wdbuf,
		},
	};

	wdbuf[0] = ((addr >> 8)&0xff);
	wdbuf[1] = (addr&0xff);
	wdbuf[2] = reg_data;

	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

    return ret;
}

static char i2c_read_reg(struct i2c_client *client, int addr)
{
	char ret;
	u8 rdbuf[2] = {0};

	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= rdbuf,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= rdbuf,
		},
	};

	rdbuf[0] = ((addr >> 8)&0xff);
	rdbuf[1] = (addr&0xff);
	
	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

    return rdbuf[0];
}

static inline char ov5645_write_cmos_sensor(struct i2c_client *client, int addr, char para)
{
    return i2c_write_reg(client, addr, para);
}
static char ov5645_read_cmos_sensor(struct i2c_client *client, int addr)
{
	char get_byte=0;
   
	get_byte = i2c_read_reg(client, addr);
    return get_byte;
}

static void ov5645_sensor_init(struct i2c_client *client)
{
	ov5645_write_cmos_sensor(client,0x3103,0x11);
	ov5645_write_cmos_sensor(client,0x3008,0x82);
	ov5645_write_cmos_sensor(client,0x3008,0x42);
	ov5645_write_cmos_sensor(client,0x3103,0x03);
	ov5645_write_cmos_sensor(client,0x3503,0x07);
	ov5645_write_cmos_sensor(client,0x3002,0x1c);
	ov5645_write_cmos_sensor(client,0x3006,0xc3);
	ov5645_write_cmos_sensor(client,0x300e,0x45);
	ov5645_write_cmos_sensor(client,0x3017,0x40);
	ov5645_write_cmos_sensor(client,0x3018,0x00);
	ov5645_write_cmos_sensor(client,0x302e,0x0b);
	ov5645_write_cmos_sensor(client,0x3037,0x13);
	ov5645_write_cmos_sensor(client,0x3108,0x01);
	ov5645_write_cmos_sensor(client,0x3611,0x06);
	ov5645_write_cmos_sensor(client,0x3612,0xab);
	ov5645_write_cmos_sensor(client,0x3614,0x50);
	ov5645_write_cmos_sensor(client,0x3618,0x04);
	ov5645_write_cmos_sensor(client,0x3034,0x18);
	ov5645_write_cmos_sensor(client,0x3035,0x11);
	ov5645_write_cmos_sensor(client,0x3036,0x54);
	ov5645_write_cmos_sensor(client,0x3500,0x00);
	ov5645_write_cmos_sensor(client,0x3501,0x01);
	ov5645_write_cmos_sensor(client,0x3502,0x00);
	ov5645_write_cmos_sensor(client,0x350a,0x00);
	ov5645_write_cmos_sensor(client,0x350b,0x3f);
	ov5645_write_cmos_sensor(client,0x3600,0x08);
	ov5645_write_cmos_sensor(client,0x3601,0x33);
	ov5645_write_cmos_sensor(client,0x3620,0x33);
	ov5645_write_cmos_sensor(client,0x3621,0xe0);
	ov5645_write_cmos_sensor(client,0x3622,0x01);
	ov5645_write_cmos_sensor(client,0x3630,0x2d);
	ov5645_write_cmos_sensor(client,0x3631,0x00);
	ov5645_write_cmos_sensor(client,0x3632,0x32);
	ov5645_write_cmos_sensor(client,0x3633,0x52);
	ov5645_write_cmos_sensor(client,0x3634,0x70);
	ov5645_write_cmos_sensor(client,0x3635,0x13);
	ov5645_write_cmos_sensor(client,0x3636,0x03);
	ov5645_write_cmos_sensor(client,0x3702,0x6e);
	ov5645_write_cmos_sensor(client,0x3703,0x52);
	ov5645_write_cmos_sensor(client,0x3704,0xa0);
	ov5645_write_cmos_sensor(client,0x3705,0x33);
	ov5645_write_cmos_sensor(client,0x3708,0x63);
	ov5645_write_cmos_sensor(client,0x3709,0x12);
	ov5645_write_cmos_sensor(client,0x370b,0x61);
	ov5645_write_cmos_sensor(client,0x370c,0xc0);
	ov5645_write_cmos_sensor(client,0x370f,0x10);
	ov5645_write_cmos_sensor(client,0x3715,0x08);
	ov5645_write_cmos_sensor(client,0x3717,0x01);
	ov5645_write_cmos_sensor(client,0x371b,0x20);
	ov5645_write_cmos_sensor(client,0x3731,0x22);
	ov5645_write_cmos_sensor(client,0x3739,0x70);
	ov5645_write_cmos_sensor(client,0x3901,0x0a);
	ov5645_write_cmos_sensor(client,0x3905,0x02);
	ov5645_write_cmos_sensor(client,0x3906,0x10);
	ov5645_write_cmos_sensor(client,0x3719,0x86);
	ov5645_write_cmos_sensor(client,0x3800,0x00);
	ov5645_write_cmos_sensor(client,0x3801,0x00);
	ov5645_write_cmos_sensor(client,0x3802,0x00);
	ov5645_write_cmos_sensor(client,0x3803,0x00);
	ov5645_write_cmos_sensor(client,0x3804,0x0a);
	ov5645_write_cmos_sensor(client,0x3805,0x3f);
	ov5645_write_cmos_sensor(client,0x3806,0x07);
	ov5645_write_cmos_sensor(client,0x3807,0x9f);
	ov5645_write_cmos_sensor(client,0x3808,0x0a);
	ov5645_write_cmos_sensor(client,0x3809,0x20);
	ov5645_write_cmos_sensor(client,0x380a,0x07);
	ov5645_write_cmos_sensor(client,0x380b,0x98);
	ov5645_write_cmos_sensor(client,0x380c,0x0b);
	ov5645_write_cmos_sensor(client,0x380d,0x1c);
	ov5645_write_cmos_sensor(client,0x380e,0x07);
	ov5645_write_cmos_sensor(client,0x380f,0xb0);
	ov5645_write_cmos_sensor(client,0x3810,0x00);
	ov5645_write_cmos_sensor(client,0x3811,0x10);
	ov5645_write_cmos_sensor(client,0x3812,0x00);
	ov5645_write_cmos_sensor(client,0x3813,0x06);
	ov5645_write_cmos_sensor(client,0x3814,0x11);
	ov5645_write_cmos_sensor(client,0x3815,0x11);
	ov5645_write_cmos_sensor(client,0x3820,0x40);
	ov5645_write_cmos_sensor(client,0x3821,0x06);
	ov5645_write_cmos_sensor(client,0x3824,0x01);
	ov5645_write_cmos_sensor(client,0x3826,0x03);
	ov5645_write_cmos_sensor(client,0x3828,0x08);
	ov5645_write_cmos_sensor(client,0x3a02,0x07);
	ov5645_write_cmos_sensor(client,0x3a03,0xb0);
	ov5645_write_cmos_sensor(client,0x3a08,0x01);
	ov5645_write_cmos_sensor(client,0x3a09,0x27);
	ov5645_write_cmos_sensor(client,0x3a0a,0x00);
	ov5645_write_cmos_sensor(client,0x3a0b,0xf6);
	ov5645_write_cmos_sensor(client,0x3a0e,0x06);
	ov5645_write_cmos_sensor(client,0x3a0d,0x08);
	ov5645_write_cmos_sensor(client,0x3a14,0x07);
	ov5645_write_cmos_sensor(client,0x3a15,0xb0);
	ov5645_write_cmos_sensor(client,0x3a18,0x00);
	ov5645_write_cmos_sensor(client,0x3a19,0xf8);
	ov5645_write_cmos_sensor(client,0x3c01,0x34);
	ov5645_write_cmos_sensor(client,0x3c04,0x28);
	ov5645_write_cmos_sensor(client,0x3c05,0x98);
	ov5645_write_cmos_sensor(client,0x3c07,0x07);
	ov5645_write_cmos_sensor(client,0x3c09,0xc2);
	ov5645_write_cmos_sensor(client,0x3c0a,0x9c);
	ov5645_write_cmos_sensor(client,0x3c0b,0x40);
	ov5645_write_cmos_sensor(client,0x3c01,0x34);
	ov5645_write_cmos_sensor(client,0x4001,0x02);
	ov5645_write_cmos_sensor(client,0x4004,0x06);
	ov5645_write_cmos_sensor(client,0x4005,0x18);
	ov5645_write_cmos_sensor(client,0x4050,0x6e);
	ov5645_write_cmos_sensor(client,0x4051,0x8f);
	ov5645_write_cmos_sensor(client,0x4300,0x30);
	ov5645_write_cmos_sensor(client,0x4514,0x00);
	ov5645_write_cmos_sensor(client,0x4520,0xb0);
	ov5645_write_cmos_sensor(client,0x460b,0x37);
	ov5645_write_cmos_sensor(client,0x460c,0x20);
	ov5645_write_cmos_sensor(client,0x4818,0x01);
	ov5645_write_cmos_sensor(client,0x481d,0xf0);
	ov5645_write_cmos_sensor(client,0x481f,0x50);
	ov5645_write_cmos_sensor(client,0x4823,0x70);
	ov5645_write_cmos_sensor(client,0x4831,0x14);
	ov5645_write_cmos_sensor(client,0x4837,0x0b);
	ov5645_write_cmos_sensor(client,0x5000,0xa7);
	ov5645_write_cmos_sensor(client,0x5001,0x83);
	ov5645_write_cmos_sensor(client,0x501d,0x00);
	ov5645_write_cmos_sensor(client,0x501f,0x00);
	ov5645_write_cmos_sensor(client,0x503d,0x00);
	ov5645_write_cmos_sensor(client,0x505c,0x30);
	ov5645_write_cmos_sensor(client,0x5181,0x59);
	ov5645_write_cmos_sensor(client,0x5183,0x00);
	ov5645_write_cmos_sensor(client,0x5191,0xf0);
	ov5645_write_cmos_sensor(client,0x5192,0x03);
	ov5645_write_cmos_sensor(client,0x5684,0x10);
	ov5645_write_cmos_sensor(client,0x5685,0xa0);
	ov5645_write_cmos_sensor(client,0x5686,0x0c);
	ov5645_write_cmos_sensor(client,0x5687,0x78);
	ov5645_write_cmos_sensor(client,0x5a00,0x08);
	ov5645_write_cmos_sensor(client,0x5a21,0x00);
	ov5645_write_cmos_sensor(client,0x5a24,0x00);
	ov5645_write_cmos_sensor(client,0x3008,0x02);
	ov5645_write_cmos_sensor(client,0x3503,0x00);
	ov5645_write_cmos_sensor(client,0x5180,0xff);
	ov5645_write_cmos_sensor(client,0x5181,0xf2);
	ov5645_write_cmos_sensor(client,0x5182,0x00);
	ov5645_write_cmos_sensor(client,0x5183,0x14);
	ov5645_write_cmos_sensor(client,0x5184,0x25);
	ov5645_write_cmos_sensor(client,0x5185,0x24);
	ov5645_write_cmos_sensor(client,0x5186,0x09);
	ov5645_write_cmos_sensor(client,0x5187,0x09);
	ov5645_write_cmos_sensor(client,0x5188,0x0a);
	ov5645_write_cmos_sensor(client,0x5189,0x75);
	ov5645_write_cmos_sensor(client,0x518a,0x52);
	ov5645_write_cmos_sensor(client,0x518b,0xea);
	ov5645_write_cmos_sensor(client,0x518c,0xa8);
	ov5645_write_cmos_sensor(client,0x518d,0x42);
	ov5645_write_cmos_sensor(client,0x518e,0x38);
	ov5645_write_cmos_sensor(client,0x518f,0x56);
	ov5645_write_cmos_sensor(client,0x5190,0x42);
	ov5645_write_cmos_sensor(client,0x5191,0xf8);
	ov5645_write_cmos_sensor(client,0x5192,0x04);
	ov5645_write_cmos_sensor(client,0x5193,0x70);
	ov5645_write_cmos_sensor(client,0x5194,0xf0);
	ov5645_write_cmos_sensor(client,0x5195,0xf0);
	ov5645_write_cmos_sensor(client,0x5196,0x03);
	ov5645_write_cmos_sensor(client,0x5197,0x01);
	ov5645_write_cmos_sensor(client,0x5198,0x04);
	ov5645_write_cmos_sensor(client,0x5199,0x12);
	ov5645_write_cmos_sensor(client,0x519a,0x04);
	ov5645_write_cmos_sensor(client,0x519b,0x00);
	ov5645_write_cmos_sensor(client,0x519c,0x06);
	ov5645_write_cmos_sensor(client,0x519d,0x82);
	ov5645_write_cmos_sensor(client,0x519e,0x38);
	ov5645_write_cmos_sensor(client,0x5381,0x1e);
	ov5645_write_cmos_sensor(client,0x5382,0x5b);
	ov5645_write_cmos_sensor(client,0x5383,0x08);
	ov5645_write_cmos_sensor(client,0x5384,0x0b);
	ov5645_write_cmos_sensor(client,0x5385,0x84);
	ov5645_write_cmos_sensor(client,0x5386,0x8f);
	ov5645_write_cmos_sensor(client,0x5387,0x82);
	ov5645_write_cmos_sensor(client,0x5388,0x71);
	ov5645_write_cmos_sensor(client,0x5389,0x11);
	ov5645_write_cmos_sensor(client,0x538a,0x01);
	ov5645_write_cmos_sensor(client,0x538b,0x98);
	ov5645_write_cmos_sensor(client,0x5300,0x08);
	ov5645_write_cmos_sensor(client,0x5301,0x1e);
	ov5645_write_cmos_sensor(client,0x5302,0x10);
	ov5645_write_cmos_sensor(client,0x5303,0x00);
	ov5645_write_cmos_sensor(client,0x5304,0x08);
	ov5645_write_cmos_sensor(client,0x5305,0x1e);
	ov5645_write_cmos_sensor(client,0x5306,0x08);
	ov5645_write_cmos_sensor(client,0x5307,0x16);
	ov5645_write_cmos_sensor(client,0x5309,0x08);
	ov5645_write_cmos_sensor(client,0x530a,0x1e);
	ov5645_write_cmos_sensor(client,0x530b,0x04);
	ov5645_write_cmos_sensor(client,0x530c,0x06);
	ov5645_write_cmos_sensor(client,0x5480,0x01);
	ov5645_write_cmos_sensor(client,0x5481,0x0e);
	ov5645_write_cmos_sensor(client,0x5482,0x18);
	ov5645_write_cmos_sensor(client,0x5483,0x2b);
	ov5645_write_cmos_sensor(client,0x5484,0x52);
	ov5645_write_cmos_sensor(client,0x5485,0x65);
	ov5645_write_cmos_sensor(client,0x5486,0x71);
	ov5645_write_cmos_sensor(client,0x5487,0x7d);
	ov5645_write_cmos_sensor(client,0x5488,0x87);
	ov5645_write_cmos_sensor(client,0x5489,0x91);
	ov5645_write_cmos_sensor(client,0x548a,0x9a);
	ov5645_write_cmos_sensor(client,0x548b,0xaa);
	ov5645_write_cmos_sensor(client,0x548c,0xb8);
	ov5645_write_cmos_sensor(client,0x548d,0xcd);
	ov5645_write_cmos_sensor(client,0x548e,0xdd);
	ov5645_write_cmos_sensor(client,0x548f,0xea);
	ov5645_write_cmos_sensor(client,0x5490,0x1d);
	ov5645_write_cmos_sensor(client,0x5580,0x02);
	ov5645_write_cmos_sensor(client,0x5583,0x40);
	ov5645_write_cmos_sensor(client,0x5584,0x30);
	ov5645_write_cmos_sensor(client,0x5589,0x10);
	ov5645_write_cmos_sensor(client,0x558a,0x00);
	ov5645_write_cmos_sensor(client,0x558b,0xf8);
	ov5645_write_cmos_sensor(client,0x5780,0xfc);
	ov5645_write_cmos_sensor(client,0x5781,0x13);
	ov5645_write_cmos_sensor(client,0x5782,0x03);
	ov5645_write_cmos_sensor(client,0x5786,0x20);
	ov5645_write_cmos_sensor(client,0x5787,0x40);
	ov5645_write_cmos_sensor(client,0x5788,0x08);
	ov5645_write_cmos_sensor(client,0x5789,0x08);
	ov5645_write_cmos_sensor(client,0x578a,0x02);
	ov5645_write_cmos_sensor(client,0x578b,0x01);
	ov5645_write_cmos_sensor(client,0x578c,0x01);
	ov5645_write_cmos_sensor(client,0x578d,0x0c);
	ov5645_write_cmos_sensor(client,0x578e,0x02);
	ov5645_write_cmos_sensor(client,0x578f,0x01);
	ov5645_write_cmos_sensor(client,0x5790,0x01);
	ov5645_write_cmos_sensor(client,0x5800,0x3f);
	ov5645_write_cmos_sensor(client,0x5801,0x16);
	ov5645_write_cmos_sensor(client,0x5802,0x0e);
	ov5645_write_cmos_sensor(client,0x5803,0x0d);
	ov5645_write_cmos_sensor(client,0x5804,0x17);
	ov5645_write_cmos_sensor(client,0x5805,0x3f);
	ov5645_write_cmos_sensor(client,0x5806,0x0b);
	ov5645_write_cmos_sensor(client,0x5807,0x06);
	ov5645_write_cmos_sensor(client,0x5808,0x04);
	ov5645_write_cmos_sensor(client,0x5809,0x04);
	ov5645_write_cmos_sensor(client,0x580a,0x06);
	ov5645_write_cmos_sensor(client,0x580b,0x0b);
	ov5645_write_cmos_sensor(client,0x580c,0x09);
	ov5645_write_cmos_sensor(client,0x580d,0x03);
	ov5645_write_cmos_sensor(client,0x580e,0x00);
	ov5645_write_cmos_sensor(client,0x580f,0x00);
	ov5645_write_cmos_sensor(client,0x5810,0x03);
	ov5645_write_cmos_sensor(client,0x5811,0x08);
	ov5645_write_cmos_sensor(client,0x5812,0x0a);
	ov5645_write_cmos_sensor(client,0x5813,0x03);
	ov5645_write_cmos_sensor(client,0x5814,0x00);
	ov5645_write_cmos_sensor(client,0x5815,0x00);
	ov5645_write_cmos_sensor(client,0x5816,0x04);
	ov5645_write_cmos_sensor(client,0x5817,0x09);
	ov5645_write_cmos_sensor(client,0x5818,0x0f);
	ov5645_write_cmos_sensor(client,0x5819,0x08);
	ov5645_write_cmos_sensor(client,0x581a,0x06);
	ov5645_write_cmos_sensor(client,0x581b,0x06);
	ov5645_write_cmos_sensor(client,0x581c,0x08);
	ov5645_write_cmos_sensor(client,0x581d,0x0c);
	ov5645_write_cmos_sensor(client,0x581e,0x3f);
	ov5645_write_cmos_sensor(client,0x581f,0x1e);
	ov5645_write_cmos_sensor(client,0x5820,0x12);
	ov5645_write_cmos_sensor(client,0x5821,0x13);
	ov5645_write_cmos_sensor(client,0x5822,0x21);
	ov5645_write_cmos_sensor(client,0x5823,0x3f);
	ov5645_write_cmos_sensor(client,0x5824,0x68);
	ov5645_write_cmos_sensor(client,0x5825,0x28);
	ov5645_write_cmos_sensor(client,0x5826,0x2c);
	ov5645_write_cmos_sensor(client,0x5827,0x28);
	ov5645_write_cmos_sensor(client,0x5828,0x08);
	ov5645_write_cmos_sensor(client,0x5829,0x48);
	ov5645_write_cmos_sensor(client,0x582a,0x64);
	ov5645_write_cmos_sensor(client,0x582b,0x62);
	ov5645_write_cmos_sensor(client,0x582c,0x64);
	ov5645_write_cmos_sensor(client,0x582d,0x28);
	ov5645_write_cmos_sensor(client,0x582e,0x46);
	ov5645_write_cmos_sensor(client,0x582f,0x62);
	ov5645_write_cmos_sensor(client,0x5830,0x60);
	ov5645_write_cmos_sensor(client,0x5831,0x62);
	ov5645_write_cmos_sensor(client,0x5832,0x26);
	ov5645_write_cmos_sensor(client,0x5833,0x48);
	ov5645_write_cmos_sensor(client,0x5834,0x66);
	ov5645_write_cmos_sensor(client,0x5835,0x44);
	ov5645_write_cmos_sensor(client,0x5836,0x64);
	ov5645_write_cmos_sensor(client,0x5837,0x28);
	ov5645_write_cmos_sensor(client,0x5838,0x66);
	ov5645_write_cmos_sensor(client,0x5839,0x48);
	ov5645_write_cmos_sensor(client,0x583a,0x2c);
	ov5645_write_cmos_sensor(client,0x583b,0x28);
	ov5645_write_cmos_sensor(client,0x583c,0x26);
	ov5645_write_cmos_sensor(client,0x583d,0xae);
	ov5645_write_cmos_sensor(client,0x5025,0x00);
	ov5645_write_cmos_sensor(client,0x3a0f,0x38);
	ov5645_write_cmos_sensor(client,0x3a10,0x30);
	ov5645_write_cmos_sensor(client,0x3a11,0x70);
	ov5645_write_cmos_sensor(client,0x3a1b,0x38);
	ov5645_write_cmos_sensor(client,0x3a1e,0x30);
	ov5645_write_cmos_sensor(client,0x3a1f,0x18);
	ov5645_write_cmos_sensor(client,0x3008,0x02);
	
}

static void ov5645_stream_on(struct i2c_client *client)
{
	//msleep(150);
    //ov5645_write_cmos_sensor(client,0xfe,0x03);
    //ov5645_write_cmos_sensor(client,0x10,0x94);
    //ov5645_write_cmos_sensor(client,0xfe,0x00);
    //msleep(50);
}

static unsigned short ov5645_read_shutter(struct i2c_client *client)
{
/* prize modified by chenjiaxi, ov5645 read shutter, 20190305-start */
#if 0
    unsigned char temp_reg1,temp_reg2,temp_reg3;
	unsigned short shutter;
	
	//0x3a63 0x3a64 gain
	//0x3500 0x3501 0x3502 aec not good
	temp_reg1 = ov5645_read_cmos_sensor(client,0x3b01);
	temp_reg2 = ov5645_read_cmos_sensor(client,0x3b04);
	//temp_reg2 &= 0x01;
	temp_reg3 = ov5645_read_cmos_sensor(client,0x3b05);

	shutter = (temp_reg1 << 16)|(temp_reg2<<8)|temp_reg3;
	//shutter = shutter >>4;
#else
	unsigned char temp_reg;
	unsigned short shutter;

    temp_reg = ov5645_read_cmos_sensor(client,0x3507);
    shutter = temp_reg;
#endif
/* prize modified by chenjiaxi, ov5645 read shutter, 20190305-end */
	//CAMERA_DBG("OV5645MIPI_Read_Shutter %d\r\n",shutter);
	return shutter;
}

static unsigned int ov5645_get_sensor_id(struct i2c_client *client,unsigned int *sensorID)
{
    // check if sensor ID correct
    *sensorID=((ov5645_read_cmos_sensor(client,0x300A)<< 8)|ov5645_read_cmos_sensor(client,0x300B));
	CAMERA_DBG("OV5645 Read ID %x",*sensorID);

    return 0;
}

static int ov5645_set_power(struct i2c_client *client,unsigned int enable)
{
	struct spc_data_t *spc_data = i2c_get_clientdata(client);
	
    int  ret = 0;
	if (enable){
		gpio_direction_output(spc_data->pdn_pin,0);
		mdelay(4);
		gpio_direction_output(spc_data->pdn_pin,1);
	}else{
		gpio_direction_output(spc_data->pdn_pin,0);
		//gpio_direction_output(spc_data->rst_pin,0);
	}
    return ret;    
}

static int match(struct i2c_client *client){
	unsigned int sensor_id = 0;
	ov5645_set_power(client,1);
	sensor_id = ((ov5645_read_cmos_sensor(client,0x300A)<< 8)|ov5645_read_cmos_sensor(client,0x300B));
	CAMERA_DBG("%s OV5645 Read ID %x",__func__,sensor_id);
	ov5645_set_power(client,0);
	return (sensor_id == OV5645_SENSOR_ID)?sensor_id:0;
	
}

static int ov5645_open(struct i2c_client *client)
{
	unsigned char i;
	unsigned short sensor_id=0;
	int id_status = 0;

	int ret,addr = 0;
	CAMERA_DBG("<Jet> Entry Open!!!");

	//  Read sensor ID to adjust I2C is OK?
	addr = client->addr;
	for(i=0;i<128;i++){
		client->addr = i;
		ret = i2c_smbus_read_byte_data(client,0x30);
		CAMERA_DBG("i(%d) ret(%d)\n",i,ret);
	}
	client->addr = addr;
	for(i=0;i<3;i++)
	{
		sensor_id = ((ov5645_read_cmos_sensor(client,0x300A) << 8) | ov5645_read_cmos_sensor(client,0x300B));
		CAMERA_DBG("*sensorID=%x %s",sensor_id,__func__);
		if(sensor_id != OV5645_SENSOR_ID)  
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
	
	CAMERA_DBG("OV5645mipi_ Sensor Read ID OK \r\n");
	ov5645_sensor_init(client);

	return 0;
}

const struct sensor_info_t ov5645_info = {
	.sensor_type = SENSOR_TYPE_5645,
	.name = "OV5645",
	.sensor_id = OV5645_SENSOR_ID,
	.open = ov5645_open,
	.init = ov5645_sensor_init,
	.stream_on = ov5645_stream_on,
	.get_shutter = ov5645_read_shutter,
	.get_sensor_id = ov5645_get_sensor_id,
	.set_power = ov5645_set_power,
	.match = match,
};
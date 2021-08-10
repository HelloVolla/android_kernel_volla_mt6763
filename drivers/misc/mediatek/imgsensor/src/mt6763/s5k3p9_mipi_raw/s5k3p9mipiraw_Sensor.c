/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 S5K3p9mipiraw_sensor.c
 *
 * Project:
 * --------
 *	 ALPS MT6763
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k3p9mipiraw_Sensor.h"



#define PFX "S5K3p9"
#define LOG_INF_NEW(format, args...)    pr_debug(PFX "[%s] " format, __func__, ##args)
#define LOG_INF LOG_INF_NEW
#define LOG_1 LOG_INF("S5K3p9,MIPI 4LANE\n")
#define SENSORDB LOG_INF


static DEFINE_SPINLOCK(imgsensor_drv_lock);

static imgsensor_info_struct imgsensor_info = {
	.sensor_id = S5K3P9_SENSOR_ID,		/* record sensor id defined in Kd_imgsensor.h */

	.checksum_value = 0x52500dc0, /* checksum value for Camera Auto Test */

	.pre = {
		.pclk = 560000000,				/* record different mode's pclk */
		.linelength  = 5088,				/* record different mode's linelength */
		.framelength = 3668,			/* record different mode's framelength */
		.startx = 0,					
		.starty = 0,					/* record different mode's starty of grabwindow */
		.grabwindow_width  = 2304,		/* record different mode's width of grabwindow */
		.grabwindow_height = 1728,		/* record different mode's height of grabwindow */
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 560000000,				/* record different mode's pclk */
		.linelength  = 5088,				/* record different mode's linelength */
		.framelength = 3668,			/* record different mode's framelength */
		.startx = 0,					
		.starty = 0,					/* record different mode's starty of grabwindow */
		.grabwindow_width  = 2304,		/* record different mode's width of grabwindow */
		.grabwindow_height = 1728,		/* record different mode's height of grabwindow */
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
	},
	.cap1 = {							          /* capture for PIP 15ps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate */
		.pclk = 560000000,				/* record different mode's pclk */
		.linelength  = 5088,				/* record different mode's linelength */
		.framelength = 7336,			/* record different mode's framelength */
		.startx = 0,					
		.starty = 0,					/* record different mode's starty of grabwindow */
		.grabwindow_width  = 2304,		/* record different mode's width of grabwindow */
		.grabwindow_height = 1728,		/* record different mode's height of grabwindow */
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 150,
	},
	.normal_video = {
		.pclk = 560000000,				/* record different mode's pclk */
		.linelength  = 5088,				/* record different mode's linelength */
		.framelength = 3668,			/* record different mode's framelength */
		.startx = 0,					
		.starty = 0,					/* record different mode's starty of grabwindow */
		.grabwindow_width  = 2304,		/* record different mode's width of grabwindow */
		.grabwindow_height = 1728,		/* record different mode's height of grabwindow */
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 560000000,				/* record different mode's pclk */
		.linelength  = 5088,				/* record different mode's linelength */
		.framelength = 1834,			/* record different mode's framelength */
		.startx = 0,					
		.starty = 0,					/* record different mode's starty of grabwindow */
		.grabwindow_width  = 2304,		/* record different mode's width of grabwindow */
		.grabwindow_height = 1728,		/* record different mode's height of grabwindow */
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 600,
	},
	.slim_video = {
		.pclk = 560000000,				/* record different mode's pclk */
		.linelength  = 5088,				/* record different mode's linelength */
		.framelength = 3668,			/* record different mode's framelength */
		.startx = 0,					
		.starty = 0,					/* record different mode's starty of grabwindow */
		.grabwindow_width  = 2304,		/* record different mode's width of grabwindow */
		.grabwindow_height = 1728,		/* record different mode's height of grabwindow */
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
	},


	.margin = 5,			/* sensor framelength & shutter margin */
	.min_shutter = 4,		/* min shutter */
	.max_frame_length = 0xFFFF,/* REG0x0202 <=REG0x0340-5//max framelength by sensor register's limitation */
	.ae_shut_delay_frame = 0,	/* shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2 */
	.ae_sensor_gain_delay_frame = 0,/* sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2 */
	.ae_ispGain_delay_frame = 2,/* isp gain delay frame for AE cycle */
	.ihdr_support = 0,	  /* 1, support; 0,not support */
	.ihdr_le_firstline = 0,  /* 1,le first ; 0, se first */
	.sensor_mode_num = 5,	  /* support sensor mode num ,don't support Slow motion */

	.cap_delay_frame = 3,		/* enter capture delay frame num */
	.pre_delay_frame = 3,		/* enter preview delay frame num */
	.video_delay_frame = 3,		/* enter video delay frame num */
	.hs_video_delay_frame = 3,	/* enter high speed video  delay frame num */
	.slim_video_delay_frame = 3,/* enter slim video delay frame num */
    .custom1_delay_frame = 2,
    .custom2_delay_frame = 2,
    .custom3_delay_frame = 2,
    .custom4_delay_frame = 2,
    .custom5_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_8MA, /* mclk driving current */
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,/* sensor_interface_type */
    .mipi_sensor_type = MIPI_OPHY_NCSI2, /* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
    .mipi_settle_delay_mode = 1,/* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,/* sensor output first pixel color */
	.mclk = 24,/* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	.mipi_lane_num = SENSOR_MIPI_4_LANE,/* mipi lane num */
	.i2c_addr_table = {0x20, 0x21,0xff},
    .i2c_speed = 300, /* i2c read/write speed */
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				/* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT, /* IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video */
	.shutter = 0x200,					/* current shutter */
	.gain = 0x200,						/* current gain */
	.dummy_pixel = 0,					/* current dummypixel */
	.dummy_line = 0,					/* current dummyline */
	.current_fps = 0,  /* full size current fps : 24fps for PIP, 30fps for Normal or ZSD */
	.autoflicker_en = KAL_FALSE,  /* auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker */
	.test_pattern = KAL_FALSE,		/* test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output */
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,/* current scenario id */
	.ihdr_en = KAL_FALSE, /* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x5a,/* record current sensor's i2c write id */
};


/* Sensor output window information*/
/*prize  modify by zhuzhengjiang for cts 20190318-begin*/
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
 	{2304, 1728, 0, 0, 2304, 1728, 2304, 1728, 0000, 0000, 2304, 1728, 0, 0, 2304, 1728},	/* Preview */
	{2304, 1728, 0, 0, 2304, 1728, 2304, 1728, 0000, 0000, 2304, 1728, 0, 0, 2304, 1728},	/* capture */
	{2304, 1728, 0, 0, 2304, 1728, 2304, 1728, 0000, 0000, 2304, 1728, 0, 0, 2304, 1728},	/* video */
	{2304, 1728, 512, 504, 1280, 720, 1280, 720, 0000, 0000, 1280, 720, 0, 0, 1280, 720},	/* hight speed video */
	{2304, 1728, 0, 0, 2304, 1728, 2304, 1728, 0000, 0000, 2304, 1728, 0, 0, 2304, 1728},/* slim video */
};
/*prize  modify by zhuzhengjiang for cts 20190318-end*/
 static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info =

{

};

static kal_uint16 read_cmos_sensor_byte(kal_uint16 addr)
{
    kal_uint16 get_byte = 0;
    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };


    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);
    return get_byte;
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte = 0;
    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };


    iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
    return get_byte;
}

/*static void write_cmos_sensor_byte(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};


    iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}
*/
static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};


    iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
	write_cmos_sensor(0x0342, imgsensor.line_length & 0xFFFF);
}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;
	/* unsigned long flags; */

	LOG_INF("framerate = %d, min framelength should enable(%d)\n", framerate, min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */



/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
		/* Extend frame length */
		write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
	}

	/* Update Shutter */
	write_cmos_sensor(0X0202, shutter & 0xFFFF);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);

}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;
    reg_gain = gain/2;
	return (kal_uint16)reg_gain;
}

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	LOG_INF("set_gain %d\n", gain);
  /* gain = 64 = 1x real gain. */

	if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
		LOG_INF("Error gain setting");
		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 16 * BASEGAIN)
			gain = 16 * BASEGAIN;
	}

    reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor(0x0204, (reg_gain&0xFFFF));
	return gain;
}	/*	set_gain  */


static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	spin_lock(&imgsensor_drv_lock);
    imgsensor.mirror = image_mirror;
    spin_unlock(&imgsensor_drv_lock);
	switch (image_mirror) {
		case IMAGE_NORMAL:
			write_cmos_sensor(0x0101, 0X00); 
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor(0x0101, 0X01); 
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor(0x0101, 0X02); 
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor(0x0101, 0X03); 
			break;
		default:
			LOG_INF("Error image_mirror setting\n");
	}
}
static void check_output_stream_off(void)
{
	kal_uint16 read_count = 0, read_register0005_value = 0;
	
	for (read_count = 0; read_count <= 4; read_count++)
	{		
		read_register0005_value = read_cmos_sensor(0x0005);			

		if (read_register0005_value == 0xff)			
			break;		
		mdelay(50);
		
		if(read_count == 4)			
			LOG_INF("stream off error\n");	
	}
}

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}	/*	night_mode	*/
static void sensor_init(void)
{
  LOG_INF("E\n");
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6010, 0x0001);
	mdelay(3);
	write_cmos_sensor(0x6214, 0x7970);
	write_cmos_sensor(0x6218, 0x7150);

	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x3F4C);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0649);
	write_cmos_sensor(0x6F12, 0x0548);
	write_cmos_sensor(0x6F12, 0xC1F8);
	write_cmos_sensor(0x6F12, 0xC405);
	write_cmos_sensor(0x6F12, 0x0549);
	write_cmos_sensor(0x6F12, 0x081A);
	write_cmos_sensor(0x6F12, 0x0349);
	write_cmos_sensor(0x6F12, 0xA1F8);
	write_cmos_sensor(0x6F12, 0xC805);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xB9BF);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x51E8);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x2ED0);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x6C00);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x40BA);
	write_cmos_sensor(0x6F12, 0x7047);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xC0BA);
	write_cmos_sensor(0x6F12, 0x7047);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xF04F);
	write_cmos_sensor(0x6F12, 0x87B0);
	write_cmos_sensor(0x6F12, 0x0026);
	write_cmos_sensor(0x6F12, 0xDFF8);
	write_cmos_sensor(0x6F12, 0x3084);
	write_cmos_sensor(0x6F12, 0x0396);
	write_cmos_sensor(0x6F12, 0x0496);
	write_cmos_sensor(0x6F12, 0x0596);
	write_cmos_sensor(0x6F12, 0x0696);
	write_cmos_sensor(0x6F12, 0x4FF4);
	write_cmos_sensor(0x6F12, 0x806A);
	write_cmos_sensor(0x6F12, 0x5146);
	write_cmos_sensor(0x6F12, 0x48F6);
	write_cmos_sensor(0x6F12, 0x5C20);
	write_cmos_sensor(0x6F12, 0xD8F8);
	write_cmos_sensor(0x6F12, 0xB820);
	write_cmos_sensor(0x6F12, 0x01F0);
	write_cmos_sensor(0x6F12, 0x43F8);
	write_cmos_sensor(0x6F12, 0x0120);
	write_cmos_sensor(0x6F12, 0x0225);
	write_cmos_sensor(0x6F12, 0xCDE9);
	write_cmos_sensor(0x6F12, 0x0050);
	write_cmos_sensor(0x6F12, 0x06AB);
	write_cmos_sensor(0x6F12, 0x05AA);
	write_cmos_sensor(0x6F12, 0x04A9);
	write_cmos_sensor(0x6F12, 0x0296);
	write_cmos_sensor(0x6F12, 0x03A8);
	write_cmos_sensor(0x6F12, 0x01F0);
	write_cmos_sensor(0x6F12, 0x3DF8);
	write_cmos_sensor(0x6F12, 0xD8F8);
	write_cmos_sensor(0x6F12, 0xB800);
	write_cmos_sensor(0x6F12, 0x20F0);
	write_cmos_sensor(0x6F12, 0x2004);
	write_cmos_sensor(0x6F12, 0xFE48);
	write_cmos_sensor(0x6F12, 0x2246);
	write_cmos_sensor(0x6F12, 0x0078);
	write_cmos_sensor(0x6F12, 0x0102);
	write_cmos_sensor(0x6F12, 0x48F6);
	write_cmos_sensor(0x6F12, 0x6220);
	write_cmos_sensor(0x6F12, 0x01F0);
	write_cmos_sensor(0x6F12, 0x2CF8);
	write_cmos_sensor(0x6F12, 0x0095);
	write_cmos_sensor(0x6F12, 0xCDE9);
	write_cmos_sensor(0x6F12, 0x0156);
	write_cmos_sensor(0x6F12, 0x06AB);
	write_cmos_sensor(0x6F12, 0x05AA);
	write_cmos_sensor(0x6F12, 0x04A9);
	write_cmos_sensor(0x6F12, 0x03A8);
	write_cmos_sensor(0x6F12, 0x01F0);
	write_cmos_sensor(0x6F12, 0x28F8);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x40F2);
	write_cmos_sensor(0x6F12, 0x0C41);
	write_cmos_sensor(0x6F12, 0x48F6);
	write_cmos_sensor(0x6F12, 0x6820);
	write_cmos_sensor(0x6F12, 0x01F0);
	write_cmos_sensor(0x6F12, 0x1CF8);
	write_cmos_sensor(0x6F12, 0x0327);
	write_cmos_sensor(0x6F12, 0xCDE9);
	write_cmos_sensor(0x6F12, 0x0057);
	write_cmos_sensor(0x6F12, 0x06AB);
	write_cmos_sensor(0x6F12, 0x05AA);
	write_cmos_sensor(0x6F12, 0x04A9);
	write_cmos_sensor(0x6F12, 0x03A8);
	write_cmos_sensor(0x6F12, 0x0296);
	write_cmos_sensor(0x6F12, 0x01F0);
	write_cmos_sensor(0x6F12, 0x17F8);
	write_cmos_sensor(0x6F12, 0x4FF4);
	write_cmos_sensor(0x6F12, 0x816B);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x5946);
	write_cmos_sensor(0x6F12, 0x48F6);
	write_cmos_sensor(0x6F12, 0x6E20);
	write_cmos_sensor(0x6F12, 0x01F0);
	write_cmos_sensor(0x6F12, 0x0AF8);
	write_cmos_sensor(0x6F12, 0x4FF0);
	write_cmos_sensor(0x6F12, 0x0409);
	write_cmos_sensor(0x6F12, 0xCDE9);
	write_cmos_sensor(0x6F12, 0x0059);
	write_cmos_sensor(0x6F12, 0x06AB);
	write_cmos_sensor(0x6F12, 0x05AA);
	write_cmos_sensor(0x6F12, 0x04A9);
	write_cmos_sensor(0x6F12, 0x03A8);
	write_cmos_sensor(0x6F12, 0x0296);
	write_cmos_sensor(0x6F12, 0x01F0);
	write_cmos_sensor(0x6F12, 0x04F8);
	write_cmos_sensor(0x6F12, 0x44F0);
	write_cmos_sensor(0x6F12, 0x0102);
	write_cmos_sensor(0x6F12, 0x5146);
	write_cmos_sensor(0x6F12, 0x48F6);
	write_cmos_sensor(0x6F12, 0x7420);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xF8FF);
	write_cmos_sensor(0x6F12, 0x0520);
	write_cmos_sensor(0x6F12, 0xCDE9);
	write_cmos_sensor(0x6F12, 0x0050);
	write_cmos_sensor(0x6F12, 0x06AB);
	write_cmos_sensor(0x6F12, 0x05AA);
	write_cmos_sensor(0x6F12, 0x04A9);
	write_cmos_sensor(0x6F12, 0x0296);
	write_cmos_sensor(0x6F12, 0x03A8);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xF3FF);
	write_cmos_sensor(0x6F12, 0x98F8);
	write_cmos_sensor(0x6F12, 0x0021);
	write_cmos_sensor(0x6F12, 0x0626);
	write_cmos_sensor(0x6F12, 0x4FF4);
	write_cmos_sensor(0x6F12, 0x8361);
	write_cmos_sensor(0x6F12, 0x48F6);
	write_cmos_sensor(0x6F12, 0x7A20);
	write_cmos_sensor(0x6F12, 0x012A);
	write_cmos_sensor(0x6F12, 0x37D0);
	write_cmos_sensor(0x6F12, 0xA222);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xE2FF);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x0095);
	write_cmos_sensor(0x6F12, 0xCDE9);
	write_cmos_sensor(0x6F12, 0x0160);
	write_cmos_sensor(0x6F12, 0x06AB);
	write_cmos_sensor(0x6F12, 0x05AA);
	write_cmos_sensor(0x6F12, 0x04A9);
	write_cmos_sensor(0x6F12, 0x03A8);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xDDFF);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x40F2);
	write_cmos_sensor(0x6F12, 0x1441);
	write_cmos_sensor(0x6F12, 0x48F6);
	write_cmos_sensor(0x6F12, 0x8020);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xD1FF);
	write_cmos_sensor(0x6F12, 0x0026);
	write_cmos_sensor(0x6F12, 0x0720);
	write_cmos_sensor(0x6F12, 0xCDF8);
	write_cmos_sensor(0x6F12, 0x0090);
	write_cmos_sensor(0x6F12, 0xCDE9);
	write_cmos_sensor(0x6F12, 0x0106);
	write_cmos_sensor(0x6F12, 0x06AB);
	write_cmos_sensor(0x6F12, 0x05AA);
	write_cmos_sensor(0x6F12, 0x04A9);
	write_cmos_sensor(0x6F12, 0x03A8);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xCAFF);
	write_cmos_sensor(0x6F12, 0x4FF4);
	write_cmos_sensor(0x6F12, 0x0062);
	write_cmos_sensor(0x6F12, 0x5946);
	write_cmos_sensor(0x6F12, 0x48F6);
	write_cmos_sensor(0x6F12, 0x8620);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xBEFF);
	write_cmos_sensor(0x6F12, 0x0820);
	write_cmos_sensor(0x6F12, 0xCDE9);
	write_cmos_sensor(0x6F12, 0x0050);
	write_cmos_sensor(0x6F12, 0x06AB);
	write_cmos_sensor(0x6F12, 0x05AA);
	write_cmos_sensor(0x6F12, 0x04A9);
	write_cmos_sensor(0x6F12, 0x0296);
	write_cmos_sensor(0x6F12, 0x03A8);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xB9FF);
	write_cmos_sensor(0x6F12, 0xDFF8);
	write_cmos_sensor(0x6F12, 0xF4A2);
	write_cmos_sensor(0x6F12, 0x9AF8);
	write_cmos_sensor(0x6F12, 0x0201);
	write_cmos_sensor(0x6F12, 0x18B1);
	write_cmos_sensor(0x6F12, 0x0420);
	write_cmos_sensor(0x6F12, 0x02E0);
	write_cmos_sensor(0x6F12, 0xF422);
	write_cmos_sensor(0x6F12, 0xC6E7);
	write_cmos_sensor(0x6F12, 0x0620);
	write_cmos_sensor(0x6F12, 0x40F4);
	write_cmos_sensor(0x6F12, 0x8032);
	write_cmos_sensor(0x6F12, 0x4FF4);
	write_cmos_sensor(0x6F12, 0x8461);
	write_cmos_sensor(0x6F12, 0x48F6);
	write_cmos_sensor(0x6F12, 0x8C20);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xA2FF);
	write_cmos_sensor(0x6F12, 0x0920);
	write_cmos_sensor(0x6F12, 0xCDE9);
	write_cmos_sensor(0x6F12, 0x0050);
	write_cmos_sensor(0x6F12, 0xB046);
	write_cmos_sensor(0x6F12, 0x06AB);
	write_cmos_sensor(0x6F12, 0x05AA);
	write_cmos_sensor(0x6F12, 0x04A9);
	write_cmos_sensor(0x6F12, 0x0296);
	write_cmos_sensor(0x6F12, 0x03A8);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x9CFF);
	write_cmos_sensor(0x6F12, 0x44F0);
	write_cmos_sensor(0x6F12, 0x0202);
	write_cmos_sensor(0x6F12, 0x4FF4);
	write_cmos_sensor(0x6F12, 0x8061);
	write_cmos_sensor(0x6F12, 0x48F6);
	write_cmos_sensor(0x6F12, 0x9220);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x8FFF);
	write_cmos_sensor(0x6F12, 0x0A20);
	write_cmos_sensor(0x6F12, 0xCDE9);
	write_cmos_sensor(0x6F12, 0x0050);
	write_cmos_sensor(0x6F12, 0x06AB);
	write_cmos_sensor(0x6F12, 0x05AA);
	write_cmos_sensor(0x6F12, 0x04A9);
	write_cmos_sensor(0x6F12, 0x0296);
	write_cmos_sensor(0x6F12, 0x03A8);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x8AFF);
	write_cmos_sensor(0x6F12, 0x0222);
	write_cmos_sensor(0x6F12, 0x40F2);
	write_cmos_sensor(0x6F12, 0x1441);
	write_cmos_sensor(0x6F12, 0x48F6);
	write_cmos_sensor(0x6F12, 0x9820);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x7EFF);
	write_cmos_sensor(0x6F12, 0x9AF8);
	write_cmos_sensor(0x6F12, 0x0201);
	write_cmos_sensor(0x6F12, 0x5646);
	write_cmos_sensor(0x6F12, 0x08B1);
	write_cmos_sensor(0x6F12, 0x0120);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0x0320);
	write_cmos_sensor(0x6F12, 0x0B21);
	write_cmos_sensor(0x6F12, 0xCDF8);
	write_cmos_sensor(0x6F12, 0x0090);
	write_cmos_sensor(0x6F12, 0xCDE9);
	write_cmos_sensor(0x6F12, 0x0110);
	write_cmos_sensor(0x6F12, 0x06AB);
	write_cmos_sensor(0x6F12, 0x05AA);
	write_cmos_sensor(0x6F12, 0x04A9);
	write_cmos_sensor(0x6F12, 0x03A8);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x71FF);
	write_cmos_sensor(0x6F12, 0x96F8);
	write_cmos_sensor(0x6F12, 0x0201);
	write_cmos_sensor(0x6F12, 0xB146);
	write_cmos_sensor(0x6F12, 0x18B9);
	write_cmos_sensor(0x6F12, 0x9948);
	write_cmos_sensor(0x6F12, 0x0780);
	write_cmos_sensor(0x6F12, 0x801E);
	write_cmos_sensor(0x6F12, 0x0580);
	write_cmos_sensor(0x6F12, 0x9748);
	write_cmos_sensor(0x6F12, 0x40F2);
	write_cmos_sensor(0x6F12, 0x1C46);
	write_cmos_sensor(0x6F12, 0x4E30);
	write_cmos_sensor(0x6F12, 0x0680);
	write_cmos_sensor(0x6F12, 0x0C20);
	write_cmos_sensor(0x6F12, 0xCDE9);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0xBB46);
	write_cmos_sensor(0x6F12, 0x06AB);
	write_cmos_sensor(0x6F12, 0x05AA);
	write_cmos_sensor(0x6F12, 0x04A9);
	write_cmos_sensor(0x6F12, 0xCDF8);
	write_cmos_sensor(0x6F12, 0x0880);
	write_cmos_sensor(0x6F12, 0x03A8);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x58FF);
	write_cmos_sensor(0x6F12, 0x8F48);
	write_cmos_sensor(0x6F12, 0x5430);
	write_cmos_sensor(0x6F12, 0x0680);
	write_cmos_sensor(0x6F12, 0x0D20);
	write_cmos_sensor(0x6F12, 0xCDE9);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0x06AB);
	write_cmos_sensor(0x6F12, 0x05AA);
	write_cmos_sensor(0x6F12, 0x04A9);
	write_cmos_sensor(0x6F12, 0xCDF8);
	write_cmos_sensor(0x6F12, 0x0880);
	write_cmos_sensor(0x6F12, 0x03A8);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x4AFF);
	write_cmos_sensor(0x6F12, 0x8848);
	write_cmos_sensor(0x6F12, 0x5A30);
	write_cmos_sensor(0x6F12, 0x0680);
	write_cmos_sensor(0x6F12, 0x0E20);
	write_cmos_sensor(0x6F12, 0xCDE9);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0x06AB);
	write_cmos_sensor(0x6F12, 0x05AA);
	write_cmos_sensor(0x6F12, 0x04A9);
	write_cmos_sensor(0x6F12, 0xCDF8);
	write_cmos_sensor(0x6F12, 0x0880);
	write_cmos_sensor(0x6F12, 0x03A8);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x3CFF);
	write_cmos_sensor(0x6F12, 0x8148);
	write_cmos_sensor(0x6F12, 0x6030);
	write_cmos_sensor(0x6F12, 0x0680);
	write_cmos_sensor(0x6F12, 0x0F20);
	write_cmos_sensor(0x6F12, 0xCDE9);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0x06AB);
	write_cmos_sensor(0x6F12, 0x05AA);
	write_cmos_sensor(0x6F12, 0x04A9);
	write_cmos_sensor(0x6F12, 0xCDF8);
	write_cmos_sensor(0x6F12, 0x0880);
	write_cmos_sensor(0x6F12, 0x03A8);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x2EFF);
	write_cmos_sensor(0x6F12, 0x99F8);
	write_cmos_sensor(0x6F12, 0x0201);
	write_cmos_sensor(0x6F12, 0x1027);
	write_cmos_sensor(0x6F12, 0x4FF0);
	write_cmos_sensor(0x6F12, 0x1109);
	write_cmos_sensor(0x6F12, 0xF0B3);
	write_cmos_sensor(0x6F12, 0x2246);
	write_cmos_sensor(0x6F12, 0xB901);
	write_cmos_sensor(0x6F12, 0x48F6);
	write_cmos_sensor(0x6F12, 0xB620);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x1DFF);
	write_cmos_sensor(0x6F12, 0xCDE9);
	write_cmos_sensor(0x6F12, 0x0057);
	write_cmos_sensor(0x6F12, 0x4646);
	write_cmos_sensor(0x6F12, 0x06AB);
	write_cmos_sensor(0x6F12, 0x05AA);
	write_cmos_sensor(0x6F12, 0x04A9);
	write_cmos_sensor(0x6F12, 0x03A8);
	write_cmos_sensor(0x6F12, 0xCDF8);
	write_cmos_sensor(0x6F12, 0x0880);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x17FF);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0x40F2);
	write_cmos_sensor(0x6F12, 0x0C41);
	write_cmos_sensor(0x6F12, 0x48F6);
	write_cmos_sensor(0x6F12, 0xBC20);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x0BFF);
	write_cmos_sensor(0x6F12, 0xCDE9);
	write_cmos_sensor(0x6F12, 0x0059);
	write_cmos_sensor(0x6F12, 0x1224);
	write_cmos_sensor(0x6F12, 0x06AB);
	write_cmos_sensor(0x6F12, 0x05AA);
	write_cmos_sensor(0x6F12, 0x04A9);
	write_cmos_sensor(0x6F12, 0x03A8);
	write_cmos_sensor(0x6F12, 0x0296);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x06FF);
	write_cmos_sensor(0x6F12, 0x641E);
	write_cmos_sensor(0x6F12, 0x0120);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0x0410);
	write_cmos_sensor(0x6F12, 0x82B2);
	write_cmos_sensor(0x6F12, 0x40F2);
	write_cmos_sensor(0x6F12, 0xFF11);
	write_cmos_sensor(0x6F12, 0x48F6);
	write_cmos_sensor(0x6F12, 0x5420);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x00FF);
	write_cmos_sensor(0x6F12, 0x0398);
	write_cmos_sensor(0x6F12, 0x604A);
	write_cmos_sensor(0x6F12, 0x010C);
	write_cmos_sensor(0x6F12, 0x143A);
	write_cmos_sensor(0x6F12, 0x1180);
	write_cmos_sensor(0x6F12, 0x911E);
	write_cmos_sensor(0x6F12, 0x0880);
	write_cmos_sensor(0x6F12, 0x0498);
	write_cmos_sensor(0x6F12, 0x010C);
	write_cmos_sensor(0x6F12, 0x121D);
	write_cmos_sensor(0x6F12, 0x1180);
	write_cmos_sensor(0x6F12, 0x911E);
	write_cmos_sensor(0x6F12, 0x0880);
	write_cmos_sensor(0x6F12, 0x0598);
	write_cmos_sensor(0x6F12, 0x594A);
	write_cmos_sensor(0x6F12, 0x010C);
	write_cmos_sensor(0x6F12, 0x083A);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0x0BE0);
	write_cmos_sensor(0x6F12, 0x1180);
	write_cmos_sensor(0x6F12, 0x911E);
	write_cmos_sensor(0x6F12, 0x0880);
	write_cmos_sensor(0x6F12, 0x0698);
	write_cmos_sensor(0x6F12, 0x010C);
	write_cmos_sensor(0x6F12, 0x121F);
	write_cmos_sensor(0x6F12, 0x1180);
	write_cmos_sensor(0x6F12, 0x911E);
	write_cmos_sensor(0x6F12, 0x0880);
	write_cmos_sensor(0x6F12, 0x07B0);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF08F);
	write_cmos_sensor(0x6F12, 0x5149);
	write_cmos_sensor(0x6F12, 0xB246);
	write_cmos_sensor(0x6F12, 0x6631);
	write_cmos_sensor(0x6F12, 0x0E80);
	write_cmos_sensor(0x6F12, 0x4646);
	write_cmos_sensor(0x6F12, 0xCDF8);
	write_cmos_sensor(0x6F12, 0x00B0);
	write_cmos_sensor(0x6F12, 0xCDE9);
	write_cmos_sensor(0x6F12, 0x0176);
	write_cmos_sensor(0x6F12, 0xD846);
	write_cmos_sensor(0x6F12, 0x06AB);
	write_cmos_sensor(0x6F12, 0x05AA);
	write_cmos_sensor(0x6F12, 0x04A9);
	write_cmos_sensor(0x6F12, 0x03A8);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xCCFE);
	write_cmos_sensor(0x6F12, 0x4949);
	write_cmos_sensor(0x6F12, 0x6C31);
	write_cmos_sensor(0x6F12, 0xA1F8);
	write_cmos_sensor(0x6F12, 0x00A0);
	write_cmos_sensor(0x6F12, 0xCDF8);
	write_cmos_sensor(0x6F12, 0x0080);
	write_cmos_sensor(0x6F12, 0xCDE9);
	write_cmos_sensor(0x6F12, 0x0196);
	write_cmos_sensor(0x6F12, 0x06AB);
	write_cmos_sensor(0x6F12, 0x05AA);
	write_cmos_sensor(0x6F12, 0x04A9);
	write_cmos_sensor(0x6F12, 0x03A8);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xBEFE);
	write_cmos_sensor(0x6F12, 0x2246);
	write_cmos_sensor(0x6F12, 0x4FF4);
	write_cmos_sensor(0x6F12, 0x8061);
	write_cmos_sensor(0x6F12, 0x48F6);
	write_cmos_sensor(0x6F12, 0xC220);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xB2FE);
	write_cmos_sensor(0x6F12, 0x1220);
	write_cmos_sensor(0x6F12, 0xCDE9);
	write_cmos_sensor(0x6F12, 0x0050);
	write_cmos_sensor(0x6F12, 0x06AB);
	write_cmos_sensor(0x6F12, 0x05AA);
	write_cmos_sensor(0x6F12, 0x04A9);
	write_cmos_sensor(0x6F12, 0x0296);
	write_cmos_sensor(0x6F12, 0x03A8);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xADFE);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0x40F2);
	write_cmos_sensor(0x6F12, 0x0C41);
	write_cmos_sensor(0x6F12, 0x48F6);
	write_cmos_sensor(0x6F12, 0xC820);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xA1FE);
	write_cmos_sensor(0x6F12, 0x1320);
	write_cmos_sensor(0x6F12, 0xCDE9);
	write_cmos_sensor(0x6F12, 0x0050);
	write_cmos_sensor(0x6F12, 0x1424);
	write_cmos_sensor(0x6F12, 0x06AB);
	write_cmos_sensor(0x6F12, 0x05AA);
	write_cmos_sensor(0x6F12, 0x04A9);
	write_cmos_sensor(0x6F12, 0x0296);
	write_cmos_sensor(0x6F12, 0x03A8);
	write_cmos_sensor(0x6F12, 0x93E7);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xF047);
	write_cmos_sensor(0x6F12, 0x8046);
	write_cmos_sensor(0x6F12, 0x3048);
	write_cmos_sensor(0x6F12, 0x2F4E);
	write_cmos_sensor(0x6F12, 0x8946);
	write_cmos_sensor(0x6F12, 0xD0F8);
	write_cmos_sensor(0x6F12, 0x0C07);
	write_cmos_sensor(0x6F12, 0xB036);
	write_cmos_sensor(0x6F12, 0x40F0);
	write_cmos_sensor(0x6F12, 0x2001);
	write_cmos_sensor(0x6F12, 0x3160);
	write_cmos_sensor(0x6F12, 0x294F);
	write_cmos_sensor(0x6F12, 0x2A4D);
	write_cmos_sensor(0x6F12, 0xBC35);
	write_cmos_sensor(0x6F12, 0xD7F8);
	write_cmos_sensor(0x6F12, 0xC010);
	write_cmos_sensor(0x6F12, 0x2960);
	write_cmos_sensor(0x6F12, 0x284A);
	write_cmos_sensor(0x6F12, 0x0021);
	write_cmos_sensor(0x6F12, 0xD032);
	write_cmos_sensor(0x6F12, 0x1160);
	write_cmos_sensor(0x6F12, 0x2649);
	write_cmos_sensor(0x6F12, 0x1F22);
	write_cmos_sensor(0x6F12, 0xD431);
	write_cmos_sensor(0x6F12, 0x0A60);
	write_cmos_sensor(0x6F12, 0x254A);
	write_cmos_sensor(0x6F12, 0xD2F8);
	write_cmos_sensor(0x6F12, 0x2027);
	write_cmos_sensor(0x6F12, 0x0A60);
	write_cmos_sensor(0x6F12, 0x20F0);
	write_cmos_sensor(0x6F12, 0x2004);
	write_cmos_sensor(0x6F12, 0x3460);
	write_cmos_sensor(0x6F12, 0x1F48);
	write_cmos_sensor(0x6F12, 0x4078);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x81FE);
	write_cmos_sensor(0x6F12, 0x1F48);
	write_cmos_sensor(0x6F12, 0xD0F8);
	write_cmos_sensor(0x6F12, 0x1407);
	write_cmos_sensor(0x6F12, 0x8107);
	write_cmos_sensor(0x6F12, 0x02D4);
	write_cmos_sensor(0x6F12, 0x20F0);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x2860);
	write_cmos_sensor(0x6F12, 0x44F0);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x3060);
	write_cmos_sensor(0x6F12, 0x1948);
	write_cmos_sensor(0x6F12, 0xC830);
	write_cmos_sensor(0x6F12, 0xC0F8);
	write_cmos_sensor(0x6F12, 0x0080);
	write_cmos_sensor(0x6F12, 0xC0F8);
	write_cmos_sensor(0x6F12, 0x0090);
	write_cmos_sensor(0x6F12, 0x164A);
	write_cmos_sensor(0x6F12, 0xB7F8);
	write_cmos_sensor(0x6F12, 0xD800);
	write_cmos_sensor(0x6F12, 0xC432);
	write_cmos_sensor(0x6F12, 0x1168);
	write_cmos_sensor(0x6F12, 0x47F6);
	write_cmos_sensor(0x6F12, 0xC073);
	write_cmos_sensor(0x6F12, 0x03E0);
	write_cmos_sensor(0x6F12, 0x1168);
	write_cmos_sensor(0x6F12, 0xA0F1);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x80B2);
	write_cmos_sensor(0x6F12, 0x8F01);
	write_cmos_sensor(0x6F12, 0x01D5);
	write_cmos_sensor(0x6F12, 0x1942);
	write_cmos_sensor(0x6F12, 0x02D0);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0xF5D1);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0x60B9);
	write_cmos_sensor(0x6F12, 0x0021);
	write_cmos_sensor(0x6F12, 0x4FF4);
	write_cmos_sensor(0x6F12, 0x2A60);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x5CFE);
	write_cmos_sensor(0x6F12, 0x0A48);
	write_cmos_sensor(0x6F12, 0x00F2);
	write_cmos_sensor(0x6F12, 0x7271);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x7207);
	write_cmos_sensor(0x6F12, 0x40F0);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x0880);
	write_cmos_sensor(0x6F12, 0x3460);
	write_cmos_sensor(0x6F12, 0x0648);
	write_cmos_sensor(0x6F12, 0xD0F8);
	write_cmos_sensor(0x6F12, 0x1407);
	write_cmos_sensor(0x6F12, 0x8107);
	write_cmos_sensor(0x6F12, 0x0CD4);
	write_cmos_sensor(0x6F12, 0x08E0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x0E20);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x51A4);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0x8A50);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x2ED0);
	write_cmos_sensor(0x6F12, 0x40F0);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x2860);
	write_cmos_sensor(0x6F12, 0xFC48);
	write_cmos_sensor(0x6F12, 0xD0F8);
	write_cmos_sensor(0x6F12, 0x0C07);
	write_cmos_sensor(0x6F12, 0x40F0);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x3060);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF087);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xF05F);
	write_cmos_sensor(0x6F12, 0x0746);
	write_cmos_sensor(0x6F12, 0xF648);
	write_cmos_sensor(0x6F12, 0xF74C);
	write_cmos_sensor(0x6F12, 0x8B46);
	write_cmos_sensor(0x6F12, 0xD0F8);
	write_cmos_sensor(0x6F12, 0x0C07);
	write_cmos_sensor(0x6F12, 0x40F0);
	write_cmos_sensor(0x6F12, 0x2001);
	write_cmos_sensor(0x6F12, 0x2160);
	write_cmos_sensor(0x6F12, 0xF44E);
	write_cmos_sensor(0x6F12, 0xDFF8);
	write_cmos_sensor(0x6F12, 0xCC93);
	write_cmos_sensor(0x6F12, 0x09F1);
	write_cmos_sensor(0x6F12, 0x0C09);
	write_cmos_sensor(0x6F12, 0xD6F8);
	write_cmos_sensor(0x6F12, 0xC020);
	write_cmos_sensor(0x6F12, 0xC9F8);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0xEF4A);
	write_cmos_sensor(0x6F12, 0x0021);
	write_cmos_sensor(0x6F12, 0x2032);
	write_cmos_sensor(0x6F12, 0x1160);
	write_cmos_sensor(0x6F12, 0xED49);
	write_cmos_sensor(0x6F12, 0x1F22);
	write_cmos_sensor(0x6F12, 0x2431);
	write_cmos_sensor(0x6F12, 0x0A60);
	write_cmos_sensor(0x6F12, 0xEA4A);
	write_cmos_sensor(0x6F12, 0xD2F8);
	write_cmos_sensor(0x6F12, 0x2027);
	write_cmos_sensor(0x6F12, 0x0A60);
	write_cmos_sensor(0x6F12, 0x20F0);
	write_cmos_sensor(0x6F12, 0x2005);
	write_cmos_sensor(0x6F12, 0x2560);
	write_cmos_sensor(0x6F12, 0xE948);
	write_cmos_sensor(0x6F12, 0x4078);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x0DFE);
	write_cmos_sensor(0x6F12, 0xE448);
	write_cmos_sensor(0x6F12, 0xD0F8);
	write_cmos_sensor(0x6F12, 0x1407);
	write_cmos_sensor(0x6F12, 0x8107);
	write_cmos_sensor(0x6F12, 0x03D4);
	write_cmos_sensor(0x6F12, 0x20F0);
	write_cmos_sensor(0x6F12, 0x0101);
	write_cmos_sensor(0x6F12, 0xC9F8);
	write_cmos_sensor(0x6F12, 0x0010);
	write_cmos_sensor(0x6F12, 0xE148);
	write_cmos_sensor(0x6F12, 0x45F0);
	write_cmos_sensor(0x6F12, 0x0101);
	write_cmos_sensor(0x6F12, 0x0160);
	write_cmos_sensor(0x6F12, 0xDF48);
	write_cmos_sensor(0x6F12, 0x1830);
	write_cmos_sensor(0x6F12, 0x0760);
	write_cmos_sensor(0x6F12, 0xDE48);
	write_cmos_sensor(0x6F12, 0xDFF8);
	write_cmos_sensor(0x6F12, 0x7483);
	write_cmos_sensor(0x6F12, 0xDF4E);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0xD800);
	write_cmos_sensor(0x6F12, 0x47F6);
	write_cmos_sensor(0x6F12, 0xC072);
	write_cmos_sensor(0x6F12, 0x08F1);
	write_cmos_sensor(0x6F12, 0x1408);
	write_cmos_sensor(0x6F12, 0xD8F8);
	write_cmos_sensor(0x6F12, 0x0010);
	write_cmos_sensor(0x6F12, 0xA0F1);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x8B01);
	write_cmos_sensor(0x6F12, 0x01D5);
	write_cmos_sensor(0x6F12, 0x1142);
	write_cmos_sensor(0x6F12, 0x02D0);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0xF5D1);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0x50B9);
	write_cmos_sensor(0x6F12, 0x0021);
	write_cmos_sensor(0x6F12, 0x4FF4);
	write_cmos_sensor(0x6F12, 0x2A60);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xE7FD);
	write_cmos_sensor(0x6F12, 0xCF48);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x7207);
	write_cmos_sensor(0x6F12, 0x40F0);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x3080);
	write_cmos_sensor(0x6F12, 0xCC48);
	write_cmos_sensor(0x6F12, 0xD0F8);
	write_cmos_sensor(0x6F12, 0x1007);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xC040);
	write_cmos_sensor(0x6F12, 0x400F);
	write_cmos_sensor(0x6F12, 0x35D0);
	write_cmos_sensor(0x6F12, 0x0021);
	write_cmos_sensor(0x6F12, 0x01EB);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0x0BFB);
	write_cmos_sensor(0x6F12, 0x00F1);
	write_cmos_sensor(0x6F12, 0x8CB2);
	write_cmos_sensor(0x6F12, 0xA108);
	write_cmos_sensor(0x6F12, 0x4029);
	write_cmos_sensor(0x6F12, 0x04D3);
	write_cmos_sensor(0x6F12, 0x40F6);
	write_cmos_sensor(0x6F12, 0xA220);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xCDFD);
	write_cmos_sensor(0x6F12, 0x3F21);
	write_cmos_sensor(0x6F12, 0xC148);
	write_cmos_sensor(0x6F12, 0xD0F8);
	write_cmos_sensor(0x6F12, 0x1007);
	write_cmos_sensor(0x6F12, 0x20F4);
	write_cmos_sensor(0x6F12, 0xFC3A);
	write_cmos_sensor(0x6F12, 0x4AEA);
	write_cmos_sensor(0x6F12, 0xC120);
	write_cmos_sensor(0x6F12, 0xBF49);
	write_cmos_sensor(0x6F12, 0x0831);
	write_cmos_sensor(0x6F12, 0x0860);
	write_cmos_sensor(0x6F12, 0xBD49);
	write_cmos_sensor(0x6F12, 0x4BF4);
	write_cmos_sensor(0x6F12, 0x8030);
	write_cmos_sensor(0x6F12, 0x2031);
	write_cmos_sensor(0x6F12, 0x0860);
	write_cmos_sensor(0x6F12, 0xDFF8);
	write_cmos_sensor(0x6F12, 0xECB2);
	write_cmos_sensor(0x6F12, 0x25F0);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x40F0);
	write_cmos_sensor(0x6F12, 0x0207);
	write_cmos_sensor(0x6F12, 0xCBF8);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0xB848);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0xD600);
	write_cmos_sensor(0x6F12, 0x6043);
	write_cmos_sensor(0x6F12, 0xD8F8);
	write_cmos_sensor(0x6F12, 0x0010);
	write_cmos_sensor(0x6F12, 0xA0F1);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0xC1F3);
	write_cmos_sensor(0x6F12, 0xC835);
	write_cmos_sensor(0x6F12, 0x8907);
	write_cmos_sensor(0x6F12, 0x01D5);
	write_cmos_sensor(0x6F12, 0xA542);
	write_cmos_sensor(0x6F12, 0x04D0);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0xF3D1);
	write_cmos_sensor(0x6F12, 0x02E0);
	write_cmos_sensor(0x6F12, 0x0121);
	write_cmos_sensor(0x6F12, 0xC8E7);
	write_cmos_sensor(0x6F12, 0x50B9);
	write_cmos_sensor(0x6F12, 0x2946);
	write_cmos_sensor(0x6F12, 0x40F6);
	write_cmos_sensor(0x6F12, 0xA120);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x9CFD);
	write_cmos_sensor(0x6F12, 0xA948);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x7207);
	write_cmos_sensor(0x6F12, 0x40F0);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x6F12, 0x3080);
	write_cmos_sensor(0x6F12, 0x27F0);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x6F12, 0xCBF8);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xA54A);
	write_cmos_sensor(0x6F12, 0x0832);
	write_cmos_sensor(0x6F12, 0xC2F8);
	write_cmos_sensor(0x6F12, 0x00A0);
	write_cmos_sensor(0x6F12, 0xA34A);
	write_cmos_sensor(0x6F12, 0x0021);
	write_cmos_sensor(0x6F12, 0x2032);
	write_cmos_sensor(0x6F12, 0x1160);
	write_cmos_sensor(0x6F12, 0xA049);
	write_cmos_sensor(0x6F12, 0xD1F8);
	write_cmos_sensor(0x6F12, 0x1417);
	write_cmos_sensor(0x6F12, 0x8A07);
	write_cmos_sensor(0x6F12, 0x03D4);
	write_cmos_sensor(0x6F12, 0x41F0);
	write_cmos_sensor(0x6F12, 0x0102);
	write_cmos_sensor(0x6F12, 0xC9F8);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x9C49);
	write_cmos_sensor(0x6F12, 0xC1F8);
	write_cmos_sensor(0x6F12, 0x0C07);
	write_cmos_sensor(0x6F12, 0x2846);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF09F);
	write_cmos_sensor(0x6F12, 0x38B5);
	write_cmos_sensor(0x6F12, 0x9848);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x3607);
	write_cmos_sensor(0x6F12, 0x0328);
	write_cmos_sensor(0x6F12, 0x2ED1);
	write_cmos_sensor(0x6F12, 0x9648);
	write_cmos_sensor(0x6F12, 0xD0F8);
	write_cmos_sensor(0x6F12, 0x4407);
	write_cmos_sensor(0x6F12, 0x8047);
	write_cmos_sensor(0x6F12, 0x0220);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x73FD);
	write_cmos_sensor(0x6F12, 0x0120);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x75FD);
	write_cmos_sensor(0x6F12, 0x9649);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x4883);
	write_cmos_sensor(0x6F12, 0x914C);
	write_cmos_sensor(0x6F12, 0x94F8);
	write_cmos_sensor(0x6F12, 0x0121);
	write_cmos_sensor(0x6F12, 0x4A83);
	write_cmos_sensor(0x6F12, 0x8D49);
	write_cmos_sensor(0x6F12, 0x91F8);
	write_cmos_sensor(0x6F12, 0xA710);
	write_cmos_sensor(0x6F12, 0x0029);
	write_cmos_sensor(0x6F12, 0x18D0);
	write_cmos_sensor(0x6F12, 0x8B49);
	write_cmos_sensor(0x6F12, 0x81F8);
	write_cmos_sensor(0x6F12, 0xA807);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x69FD);
	write_cmos_sensor(0x6F12, 0x0546);
	write_cmos_sensor(0x6F12, 0x2846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x6AFD);
	write_cmos_sensor(0x6F12, 0xD4F8);
	write_cmos_sensor(0x6F12, 0x2411);
	write_cmos_sensor(0x6F12, 0x8142);
	write_cmos_sensor(0x6F12, 0x01D2);
	write_cmos_sensor(0x6F12, 0x0120);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x8349);
	write_cmos_sensor(0x6F12, 0x91F8);
	write_cmos_sensor(0x6F12, 0xA817);
	write_cmos_sensor(0x6F12, 0x8DF8);
	write_cmos_sensor(0x6F12, 0x0010);
	write_cmos_sensor(0x6F12, 0x9DF8);
	write_cmos_sensor(0x6F12, 0x0010);
	write_cmos_sensor(0x6F12, 0x0143);
	write_cmos_sensor(0x6F12, 0xECD0);
	write_cmos_sensor(0x6F12, 0x38BD);
	write_cmos_sensor(0x6F12, 0x70B5);
	write_cmos_sensor(0x6F12, 0x7F4B);
	write_cmos_sensor(0x6F12, 0x7D4C);
	write_cmos_sensor(0x6F12, 0xB3F8);
	write_cmos_sensor(0x6F12, 0x2C31);
	write_cmos_sensor(0x6F12, 0xA4F8);
	write_cmos_sensor(0x6F12, 0xAC37);
	write_cmos_sensor(0x6F12, 0x94F8);
	write_cmos_sensor(0x6F12, 0xA560);
	write_cmos_sensor(0x6F12, 0x04F2);
	write_cmos_sensor(0x6F12, 0xAC75);
	write_cmos_sensor(0x6F12, 0x56B1);
	write_cmos_sensor(0x6F12, 0x801A);
	write_cmos_sensor(0x6F12, 0xB4F8);
	write_cmos_sensor(0x6F12, 0xC820);
	write_cmos_sensor(0x6F12, 0x4A43);
	write_cmos_sensor(0x6F12, 0xA0EB);
	write_cmos_sensor(0x6F12, 0x2210);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0x02DD);
	write_cmos_sensor(0x6F12, 0x03FA);
	write_cmos_sensor(0x6F12, 0x80F0);
	write_cmos_sensor(0x6F12, 0x2880);
	write_cmos_sensor(0x6F12, 0x7248);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0xAC07);
	write_cmos_sensor(0x6F12, 0x70BD);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xF047);
	write_cmos_sensor(0x6F12, 0x6F48);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x3607);
	write_cmos_sensor(0x6F12, 0x0428);
	write_cmos_sensor(0x6F12, 0x7ED1);
	write_cmos_sensor(0x6F12, 0xDFF8);
	write_cmos_sensor(0x6F12, 0xBC81);
	write_cmos_sensor(0x6F12, 0x98F8);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0x78D0);
	write_cmos_sensor(0x6F12, 0x6A48);
	write_cmos_sensor(0x6F12, 0x6949);
	write_cmos_sensor(0x6F12, 0x3C22);
	write_cmos_sensor(0x6F12, 0xD0F8);
	write_cmos_sensor(0x6F12, 0x6007);
	write_cmos_sensor(0x6F12, 0x0B46);
	write_cmos_sensor(0x6F12, 0x3C30);
	write_cmos_sensor(0x6F12, 0xC1F8);
	write_cmos_sensor(0x6F12, 0x5C07);
	write_cmos_sensor(0x6F12, 0x0846);
	write_cmos_sensor(0x6F12, 0x00F2);
	write_cmos_sensor(0x6F12, 0x6477);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0x6407);
	write_cmos_sensor(0x6F12, 0xB3F8);
	write_cmos_sensor(0x6F12, 0x7837);
	write_cmos_sensor(0x6F12, 0x00FB);
	write_cmos_sensor(0x6F12, 0x0322);
	write_cmos_sensor(0x6F12, 0x0846);
	write_cmos_sensor(0x6F12, 0x00F2);
	write_cmos_sensor(0x6F12, 0x7A76);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x7A07);
	write_cmos_sensor(0x6F12, 0x1044);
	write_cmos_sensor(0x6F12, 0xA1F8);
	write_cmos_sensor(0x6F12, 0x7C07);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x1AFD);
	write_cmos_sensor(0x6F12, 0x604C);
	write_cmos_sensor(0x6F12, 0x40F6);
	write_cmos_sensor(0x6F12, 0xC329);
	write_cmos_sensor(0x6F12, 0x0228);
	write_cmos_sensor(0x6F12, 0x72D0);
	write_cmos_sensor(0x6F12, 0x0121);
	write_cmos_sensor(0x6F12, 0x0846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x16FD);
	write_cmos_sensor(0x6F12, 0x5749);
	write_cmos_sensor(0x6F12, 0x5D48);
	write_cmos_sensor(0x6F12, 0x01F2);
	write_cmos_sensor(0x6F12, 0x7472);
	write_cmos_sensor(0x6F12, 0xB1F8);
	write_cmos_sensor(0x6F12, 0x7437);
	write_cmos_sensor(0x6F12, 0xB1F8);
	write_cmos_sensor(0x6F12, 0x7217);
	write_cmos_sensor(0x6F12, 0x0B43);
	write_cmos_sensor(0x6F12, 0x1380);
	write_cmos_sensor(0x6F12, 0x524A);
	write_cmos_sensor(0x6F12, 0x0025);
	write_cmos_sensor(0x6F12, 0xB2F8);
	write_cmos_sensor(0x6F12, 0x7A27);
	write_cmos_sensor(0x6F12, 0x0282);
	write_cmos_sensor(0x6F12, 0x504A);
	write_cmos_sensor(0x6F12, 0x3580);
	write_cmos_sensor(0x6F12, 0x92F8);
	write_cmos_sensor(0x6F12, 0x6427);
	write_cmos_sensor(0x6F12, 0x4271);
	write_cmos_sensor(0x6F12, 0x4D4A);
	write_cmos_sensor(0x6F12, 0x3D70);
	write_cmos_sensor(0x6F12, 0x92F8);
	write_cmos_sensor(0x6F12, 0x6D27);
	write_cmos_sensor(0x6F12, 0x8271);
	write_cmos_sensor(0x6F12, 0x4B4A);
	write_cmos_sensor(0x6F12, 0x02F5);
	write_cmos_sensor(0x6F12, 0xF066);
	write_cmos_sensor(0x6F12, 0xB2F8);
	write_cmos_sensor(0x6F12, 0x8227);
	write_cmos_sensor(0x6F12, 0x8282);
	write_cmos_sensor(0x6F12, 0x484A);
	write_cmos_sensor(0x6F12, 0xD2F8);
	write_cmos_sensor(0x6F12, 0xC025);
	write_cmos_sensor(0x6F12, 0x0262);
	write_cmos_sensor(0x6F12, 0x8181);
	write_cmos_sensor(0x6F12, 0x4549);
	write_cmos_sensor(0x6F12, 0x4C4A);
	write_cmos_sensor(0x6F12, 0xD1F8);
	write_cmos_sensor(0x6F12, 0x2C17);
	write_cmos_sensor(0x6F12, 0xC161);
	write_cmos_sensor(0x6F12, 0x1288);
	write_cmos_sensor(0x6F12, 0xD8F8);
	write_cmos_sensor(0x6F12, 0x0C11);
	write_cmos_sensor(0x6F12, 0x484B);
	write_cmos_sensor(0x6F12, 0x9B1C);
	write_cmos_sensor(0x6F12, 0x1B88);
	write_cmos_sensor(0x6F12, 0x42EA);
	write_cmos_sensor(0x6F12, 0x0342);
	write_cmos_sensor(0x6F12, 0x0A44);
	write_cmos_sensor(0x6F12, 0x4262);
	write_cmos_sensor(0x6F12, 0x454A);
	write_cmos_sensor(0x6F12, 0x121D);
	write_cmos_sensor(0x6F12, 0x1288);
	write_cmos_sensor(0x6F12, 0x434B);
	write_cmos_sensor(0x6F12, 0x9B1D);
	write_cmos_sensor(0x6F12, 0x1B88);
	write_cmos_sensor(0x6F12, 0x42EA);
	write_cmos_sensor(0x6F12, 0x0342);
	write_cmos_sensor(0x6F12, 0x0A44);
	write_cmos_sensor(0x6F12, 0x8262);
	write_cmos_sensor(0x6F12, 0x404A);
	write_cmos_sensor(0x6F12, 0x0832);
	write_cmos_sensor(0x6F12, 0x1288);
	write_cmos_sensor(0x6F12, 0x3E4B);
	write_cmos_sensor(0x6F12, 0x0A33);
	write_cmos_sensor(0x6F12, 0x1B88);
	write_cmos_sensor(0x6F12, 0x42EA);
	write_cmos_sensor(0x6F12, 0x0342);
	write_cmos_sensor(0x6F12, 0x0A44);
	write_cmos_sensor(0x6F12, 0xC262);
	write_cmos_sensor(0x6F12, 0x3B4A);
	write_cmos_sensor(0x6F12, 0x0C32);
	write_cmos_sensor(0x6F12, 0x1288);
	write_cmos_sensor(0x6F12, 0x394B);
	write_cmos_sensor(0x6F12, 0x0E33);
	write_cmos_sensor(0x6F12, 0x1B88);
	write_cmos_sensor(0x6F12, 0x42EA);
	write_cmos_sensor(0x6F12, 0x0342);
	write_cmos_sensor(0x6F12, 0x0A44);
	write_cmos_sensor(0x6F12, 0x0263);
	write_cmos_sensor(0x6F12, 0x364A);
	write_cmos_sensor(0x6F12, 0x1032);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0x23E0);
	write_cmos_sensor(0x6F12, 0x1288);
	write_cmos_sensor(0x6F12, 0x334B);
	write_cmos_sensor(0x6F12, 0x1233);
	write_cmos_sensor(0x6F12, 0x1B88);
	write_cmos_sensor(0x6F12, 0x42EA);
	write_cmos_sensor(0x6F12, 0x0342);
	write_cmos_sensor(0x6F12, 0x1144);
	write_cmos_sensor(0x6F12, 0x4163);
	write_cmos_sensor(0x6F12, 0x3149);
	write_cmos_sensor(0x6F12, 0x8A88);
	write_cmos_sensor(0x6F12, 0xC282);
	write_cmos_sensor(0x6F12, 0xC979);
	write_cmos_sensor(0x6F12, 0xC172);
	write_cmos_sensor(0x6F12, 0x0121);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xB0FC);
	write_cmos_sensor(0x6F12, 0x0120);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xB2FC);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xA6FC);
	write_cmos_sensor(0x6F12, 0x0228);
	write_cmos_sensor(0x6F12, 0x0CD0);
	write_cmos_sensor(0x6F12, 0x15E0);
	write_cmos_sensor(0x6F12, 0xFFE7);
	write_cmos_sensor(0x6F12, 0x0221);
	write_cmos_sensor(0x6F12, 0x4846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x85FC);
	write_cmos_sensor(0x6F12, 0x1E48);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x7207);
	write_cmos_sensor(0x6F12, 0x40F4);
	write_cmos_sensor(0x6F12, 0x8040);
	write_cmos_sensor(0x6F12, 0x2080);
	write_cmos_sensor(0x6F12, 0x42E6);
	write_cmos_sensor(0x6F12, 0x0321);
	write_cmos_sensor(0x6F12, 0x4846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x7AFC);
	write_cmos_sensor(0x6F12, 0x1848);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x7207);
	write_cmos_sensor(0x6F12, 0x40F4);
	write_cmos_sensor(0x6F12, 0x8040);
	write_cmos_sensor(0x6F12, 0x2080);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x9AFC);
	write_cmos_sensor(0x6F12, 0x1B49);
	write_cmos_sensor(0x6F12, 0x1C31);
	write_cmos_sensor(0x6F12, 0x0880);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x891C);
	write_cmos_sensor(0x6F12, 0x0880);
	write_cmos_sensor(0x6F12, 0x1148);
	write_cmos_sensor(0x6F12, 0x1149);
	write_cmos_sensor(0x6F12, 0x134B);
	write_cmos_sensor(0x6F12, 0xD0F8);
	write_cmos_sensor(0x6F12, 0x8007);
	write_cmos_sensor(0x6F12, 0x134A);
	write_cmos_sensor(0x6F12, 0x401C);
	write_cmos_sensor(0x6F12, 0x3060);
	write_cmos_sensor(0x6F12, 0x91F8);
	write_cmos_sensor(0x6F12, 0x6D17);
	write_cmos_sensor(0x6F12, 0x9E78);
	write_cmos_sensor(0x6F12, 0xD21E);
	write_cmos_sensor(0x6F12, 0xB142);
	write_cmos_sensor(0x6F12, 0x02D3);
	write_cmos_sensor(0x6F12, 0xDB78);
	write_cmos_sensor(0x6F12, 0x9942);
	write_cmos_sensor(0x6F12, 0x24D9);
	write_cmos_sensor(0x6F12, 0x0128);
	write_cmos_sensor(0x6F12, 0x22D9);
	write_cmos_sensor(0x6F12, 0x0848);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0x6F07);
	write_cmos_sensor(0x6F12, 0x401C);
	write_cmos_sensor(0x6F12, 0x1070);
	write_cmos_sensor(0x6F12, 0x40F6);
	write_cmos_sensor(0x6F12, 0xC820);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x50FC);
	write_cmos_sensor(0x6F12, 0x0348);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x7207);
	write_cmos_sensor(0x6F12, 0x40F4);
	write_cmos_sensor(0x6F12, 0x8070);
	write_cmos_sensor(0x6F12, 0x2080);
	write_cmos_sensor(0x6F12, 0x15E0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x2ED0);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0x8B00);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x0E20);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x51A4);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x3642);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0x7000);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0xC000);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0x8A00);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x3680);
	write_cmos_sensor(0x6F12, 0xFE48);
	write_cmos_sensor(0x6F12, 0x80F8);
	write_cmos_sensor(0x6F12, 0x6F57);
	write_cmos_sensor(0x6F12, 0xFD49);
	write_cmos_sensor(0x6F12, 0x0120);
	write_cmos_sensor(0x6F12, 0xA1F8);
	write_cmos_sensor(0x6F12, 0x7607);
	write_cmos_sensor(0x6F12, 0x0846);
	write_cmos_sensor(0x6F12, 0xA0F8);
	write_cmos_sensor(0x6F12, 0x7057);
	write_cmos_sensor(0x6F12, 0xA0F8);
	write_cmos_sensor(0x6F12, 0x7257);
	write_cmos_sensor(0x6F12, 0x80F8);
	write_cmos_sensor(0x6F12, 0x6D57);
	write_cmos_sensor(0x6F12, 0xF848);
	write_cmos_sensor(0x6F12, 0x0560);
	write_cmos_sensor(0x6F12, 0xE9E5);
	write_cmos_sensor(0x6F12, 0xF648);
	write_cmos_sensor(0x6F12, 0x10B5);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x3607);
	write_cmos_sensor(0x6F12, 0x0428);
	write_cmos_sensor(0x6F12, 0x26D1);
	write_cmos_sensor(0x6F12, 0xF548);
	write_cmos_sensor(0x6F12, 0x0078);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0x22D0);
	write_cmos_sensor(0x6F12, 0xF148);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0xA500);
	write_cmos_sensor(0x6F12, 0xA8B1);
	write_cmos_sensor(0x6F12, 0xEE48);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0x6C07);
	write_cmos_sensor(0x6F12, 0x88B1);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x3DFC);
	write_cmos_sensor(0x6F12, 0x70B9);
	write_cmos_sensor(0x6F12, 0x0421);
	write_cmos_sensor(0x6F12, 0x40F6);
	write_cmos_sensor(0x6F12, 0xC320);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x0AFC);
	write_cmos_sensor(0x6F12, 0xEB48);
	write_cmos_sensor(0x6F12, 0x0480);
	write_cmos_sensor(0x6F12, 0xE748);
	write_cmos_sensor(0x6F12, 0x00F2);
	write_cmos_sensor(0x6F12, 0x7271);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x7207);
	write_cmos_sensor(0x6F12, 0x40F4);
	write_cmos_sensor(0x6F12, 0x8040);
	write_cmos_sensor(0x6F12, 0x0880);
	write_cmos_sensor(0x6F12, 0xE348);
	write_cmos_sensor(0x6F12, 0xE449);
	write_cmos_sensor(0x6F12, 0x80F8);
	write_cmos_sensor(0x6F12, 0x6C47);
	write_cmos_sensor(0x6F12, 0xA0F8);
	write_cmos_sensor(0x6F12, 0x7647);
	write_cmos_sensor(0x6F12, 0x0120);
	write_cmos_sensor(0x6F12, 0x0860);
	write_cmos_sensor(0x6F12, 0x10BD);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0x8046);
	write_cmos_sensor(0x6F12, 0xDD4E);
	write_cmos_sensor(0x6F12, 0xDF48);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0xB6F8);
	write_cmos_sensor(0x6F12, 0xAC52);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x0611);
	write_cmos_sensor(0x6F12, 0xB6F8);
	write_cmos_sensor(0x6F12, 0xBC02);
	write_cmos_sensor(0x6F12, 0x0144);
	write_cmos_sensor(0x6F12, 0x491E);
	write_cmos_sensor(0x6F12, 0x91FB);
	write_cmos_sensor(0x6F12, 0xF0F7);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x17FC);
	write_cmos_sensor(0x6F12, 0x96F8);
	write_cmos_sensor(0x6F12, 0x8D22);
	write_cmos_sensor(0x6F12, 0xE91B);
	write_cmos_sensor(0x6F12, 0x8A1A);
	write_cmos_sensor(0x6F12, 0xD549);
	write_cmos_sensor(0x6F12, 0x8242);
	write_cmos_sensor(0x6F12, 0x04D2);
	write_cmos_sensor(0x6F12, 0xA842);
	write_cmos_sensor(0x6F12, 0x02D8);
	write_cmos_sensor(0x6F12, 0x0868);
	write_cmos_sensor(0x6F12, 0x00B9);
	write_cmos_sensor(0x6F12, 0x0124);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x0860);
	write_cmos_sensor(0x6F12, 0xD249);
	write_cmos_sensor(0x6F12, 0xA1F8);
	write_cmos_sensor(0x6F12, 0x0080);
	write_cmos_sensor(0x6F12, 0x1CB9);
	write_cmos_sensor(0x6F12, 0xCD48);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x7607);
	write_cmos_sensor(0x6F12, 0xF0B1);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0xCE4D);
	write_cmos_sensor(0x6F12, 0x48B1);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xF7FB);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0x15D1);
	write_cmos_sensor(0x6F12, 0xC748);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x7207);
	write_cmos_sensor(0x6F12, 0x40F0);
	write_cmos_sensor(0x6F12, 0x8000);
	write_cmos_sensor(0x6F12, 0x2880);
	write_cmos_sensor(0x6F12, 0xC449);
	write_cmos_sensor(0x6F12, 0x0120);
	write_cmos_sensor(0x6F12, 0x81F8);
	write_cmos_sensor(0x6F12, 0x6C07);
	write_cmos_sensor(0x6F12, 0x0846);
	write_cmos_sensor(0x6F12, 0x91F8);
	write_cmos_sensor(0x6F12, 0x7717);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x7207);
	write_cmos_sensor(0x6F12, 0x40F4);
	write_cmos_sensor(0x6F12, 0x0060);
	write_cmos_sensor(0x6F12, 0x40EA);
	write_cmos_sensor(0x6F12, 0x0130);
	write_cmos_sensor(0x6F12, 0x40EA);
	write_cmos_sensor(0x6F12, 0x4430);
	write_cmos_sensor(0x6F12, 0x2880);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF081);
	write_cmos_sensor(0x6F12, 0x0120);
	write_cmos_sensor(0x6F12, 0xDFE7);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xF047);
	write_cmos_sensor(0x6F12, 0x0746);
	write_cmos_sensor(0x6F12, 0xBA48);
	write_cmos_sensor(0x6F12, 0x9046);
	write_cmos_sensor(0x6F12, 0x8946);
	write_cmos_sensor(0x6F12, 0x406A);
	write_cmos_sensor(0x6F12, 0x1C46);
	write_cmos_sensor(0x6F12, 0x86B2);
	write_cmos_sensor(0x6F12, 0x050C);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x3146);
	write_cmos_sensor(0x6F12, 0x2846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xD6FB);
	write_cmos_sensor(0x6F12, 0x2346);
	write_cmos_sensor(0x6F12, 0x4246);
	write_cmos_sensor(0x6F12, 0x4946);
	write_cmos_sensor(0x6F12, 0x3846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xD5FB);
	write_cmos_sensor(0x6F12, 0xB048);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0x8B02);
	write_cmos_sensor(0x6F12, 0x88B1);
	write_cmos_sensor(0x6F12, 0x788A);
	write_cmos_sensor(0x6F12, 0x04F1);
	write_cmos_sensor(0x6F12, 0x0054);
	write_cmos_sensor(0x6F12, 0x04EB);
	write_cmos_sensor(0x6F12, 0x8001);
	write_cmos_sensor(0x6F12, 0x09E0);
	write_cmos_sensor(0x6F12, 0x2268);
	write_cmos_sensor(0x6F12, 0xC2F3);
	write_cmos_sensor(0x6F12, 0xC360);
	write_cmos_sensor(0x6F12, 0x90FA);
	write_cmos_sensor(0x6F12, 0xA0F0);
	write_cmos_sensor(0x6F12, 0x22F0);
	write_cmos_sensor(0x6F12, 0x7842);
	write_cmos_sensor(0x6F12, 0x42EA);
	write_cmos_sensor(0x6F12, 0x5000);
	write_cmos_sensor(0x6F12, 0x01C4);
	write_cmos_sensor(0x6F12, 0x8C42);
	write_cmos_sensor(0x6F12, 0xF3D1);
	write_cmos_sensor(0x6F12, 0x3146);
	write_cmos_sensor(0x6F12, 0x2846);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF047);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xB3BB);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xFC5F);
	write_cmos_sensor(0x6F12, 0x8346);
	write_cmos_sensor(0x6F12, 0xA148);
	write_cmos_sensor(0x6F12, 0x8A46);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x806A);
	write_cmos_sensor(0x6F12, 0x010C);
	write_cmos_sensor(0x6F12, 0x80B2);
	write_cmos_sensor(0x6F12, 0xCDE9);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x0146);
	write_cmos_sensor(0x6F12, 0x0198);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xA4FB);
	write_cmos_sensor(0x6F12, 0xABFB);
	write_cmos_sensor(0x6F12, 0x0A10);
	write_cmos_sensor(0x6F12, 0x9B4B);
	write_cmos_sensor(0x6F12, 0x984D);
	write_cmos_sensor(0x6F12, 0x984A);
	write_cmos_sensor(0x6F12, 0x93F8);
	write_cmos_sensor(0x6F12, 0x9160);
	write_cmos_sensor(0x6F12, 0x05F5);
	write_cmos_sensor(0x6F12, 0xAA69);
	write_cmos_sensor(0x6F12, 0x06FB);
	write_cmos_sensor(0x6F12, 0x0BF6);
	write_cmos_sensor(0x6F12, 0x0023);
	write_cmos_sensor(0x6F12, 0x891B);
	write_cmos_sensor(0x6F12, 0x4D46);
	write_cmos_sensor(0x6F12, 0x60EB);
	write_cmos_sensor(0x6F12, 0x0300);
	write_cmos_sensor(0x6F12, 0x03C5);
	write_cmos_sensor(0x6F12, 0x1D46);
	write_cmos_sensor(0x6F12, 0xEBFB);
	write_cmos_sensor(0x6F12, 0x0A65);
	write_cmos_sensor(0x6F12, 0x02F5);
	write_cmos_sensor(0x6F12, 0xAB67);
	write_cmos_sensor(0x6F12, 0x3A46);
	write_cmos_sensor(0x6F12, 0x8E4C);
	write_cmos_sensor(0x6F12, 0x60C2);
	write_cmos_sensor(0x6F12, 0xA4F8);
	write_cmos_sensor(0x6F12, 0x4835);
	write_cmos_sensor(0x6F12, 0x04F5);
	write_cmos_sensor(0x6F12, 0xA962);
	write_cmos_sensor(0x6F12, 0x94F8);
	write_cmos_sensor(0x6F12, 0xA0C4);
	write_cmos_sensor(0x6F12, 0x94F8);
	write_cmos_sensor(0x6F12, 0xA144);
	write_cmos_sensor(0x6F12, 0x4FF4);
	write_cmos_sensor(0x6F12, 0xF858);
	write_cmos_sensor(0x6F12, 0xBCF1);
	write_cmos_sensor(0x6F12, 0x010F);
	write_cmos_sensor(0x6F12, 0x03D0);
	write_cmos_sensor(0x6F12, 0xBCF1);
	write_cmos_sensor(0x6F12, 0x020F);
	write_cmos_sensor(0x6F12, 0x14D0);
	write_cmos_sensor(0x6F12, 0x29E0);
	write_cmos_sensor(0x6F12, 0x08EA);
	write_cmos_sensor(0x6F12, 0x0423);
	write_cmos_sensor(0x6F12, 0x43F0);
	write_cmos_sensor(0x6F12, 0x1103);
	write_cmos_sensor(0x6F12, 0x1380);
	write_cmos_sensor(0x6F12, 0x2346);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x7BFB);
	write_cmos_sensor(0x6F12, 0xC9E9);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x2346);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x2846);
	write_cmos_sensor(0x6F12, 0x3146);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x73FB);
	write_cmos_sensor(0x6F12, 0xC7E9);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x15E0);
	write_cmos_sensor(0x6F12, 0x08EA);
	write_cmos_sensor(0x6F12, 0x042C);
	write_cmos_sensor(0x6F12, 0x4CF0);
	write_cmos_sensor(0x6F12, 0x010C);
	write_cmos_sensor(0x6F12, 0xA2F8);
	write_cmos_sensor(0x6F12, 0x00C0);
	write_cmos_sensor(0x6F12, 0xA1FB);
	write_cmos_sensor(0x6F12, 0x042C);
	write_cmos_sensor(0x6F12, 0x00FB);
	write_cmos_sensor(0x6F12, 0x04C0);
	write_cmos_sensor(0x6F12, 0x01FB);
	write_cmos_sensor(0x6F12, 0x0301);
	write_cmos_sensor(0x6F12, 0xC9E9);
	write_cmos_sensor(0x6F12, 0x0012);
	write_cmos_sensor(0x6F12, 0xA6FB);
	write_cmos_sensor(0x6F12, 0x0401);
	write_cmos_sensor(0x6F12, 0x05FB);
	write_cmos_sensor(0x6F12, 0x0411);
	write_cmos_sensor(0x6F12, 0x06FB);
	write_cmos_sensor(0x6F12, 0x0311);
	write_cmos_sensor(0x6F12, 0xC7E9);
	write_cmos_sensor(0x6F12, 0x0010);
	write_cmos_sensor(0x6F12, 0x7048);
	write_cmos_sensor(0x6F12, 0x7449);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x4805);
	write_cmos_sensor(0x6F12, 0x0880);
	write_cmos_sensor(0x6F12, 0x7348);
	write_cmos_sensor(0x6F12, 0x0CC8);
	write_cmos_sensor(0x6F12, 0x48F6);
	write_cmos_sensor(0x6F12, 0x2200);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x54FB);
	write_cmos_sensor(0x6F12, 0x7048);
	write_cmos_sensor(0x6F12, 0x0830);
	write_cmos_sensor(0x6F12, 0x0CC8);
	write_cmos_sensor(0x6F12, 0x48F6);
	write_cmos_sensor(0x6F12, 0x2A00);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x4DFB);
	write_cmos_sensor(0x6F12, 0x5846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x4FFB);
	write_cmos_sensor(0x6F12, 0x6549);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0xC1F8);
	write_cmos_sensor(0x6F12, 0x68A5);
	write_cmos_sensor(0x6F12, 0xDDE9);
	write_cmos_sensor(0x6F12, 0x0010);
	write_cmos_sensor(0x6F12, 0x02B0);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF05F);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x30BB);
	write_cmos_sensor(0x6F12, 0x604A);
	write_cmos_sensor(0x6F12, 0x92F8);
	write_cmos_sensor(0x6F12, 0xD525);
	write_cmos_sensor(0x6F12, 0x2AB1);
	write_cmos_sensor(0x6F12, 0x5E4A);
	write_cmos_sensor(0x6F12, 0x5D4B);
	write_cmos_sensor(0x6F12, 0xD2F8);
	write_cmos_sensor(0x6F12, 0x6825);
	write_cmos_sensor(0x6F12, 0xC3F8);
	write_cmos_sensor(0x6F12, 0x3024);
	write_cmos_sensor(0x6F12, 0x5B4A);
	write_cmos_sensor(0x6F12, 0xD2F8);
	write_cmos_sensor(0x6F12, 0x3024);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x3ABB);
	write_cmos_sensor(0x6F12, 0x10B5);
	write_cmos_sensor(0x6F12, 0x5849);
	write_cmos_sensor(0x6F12, 0x5E4A);
	write_cmos_sensor(0x6F12, 0x5F4B);
	write_cmos_sensor(0x6F12, 0xD1F8);
	write_cmos_sensor(0x6F12, 0x3C14);
	write_cmos_sensor(0x6F12, 0x947C);
	write_cmos_sensor(0x6F12, 0x0CB1);
	write_cmos_sensor(0x6F12, 0x908A);
	write_cmos_sensor(0x6F12, 0x1BE0);
	write_cmos_sensor(0x6F12, 0x534A);
	write_cmos_sensor(0x6F12, 0x92F8);
	write_cmos_sensor(0x6F12, 0xA220);
	write_cmos_sensor(0x6F12, 0xC2F1);
	write_cmos_sensor(0x6F12, 0x0C02);
	write_cmos_sensor(0x6F12, 0xD140);
	write_cmos_sensor(0x6F12, 0x4843);
	write_cmos_sensor(0x6F12, 0x010A);
	write_cmos_sensor(0x6F12, 0x5848);
	write_cmos_sensor(0x6F12, 0xD0F8);
	write_cmos_sensor(0x6F12, 0x8400);
	write_cmos_sensor(0x6F12, 0x0279);
	write_cmos_sensor(0x6F12, 0x4A43);
	write_cmos_sensor(0x6F12, 0x4179);
	write_cmos_sensor(0x6F12, 0xC088);
	write_cmos_sensor(0x6F12, 0xCA40);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0x1210);
	write_cmos_sensor(0x6F12, 0x4FF4);
	write_cmos_sensor(0x6F12, 0x8021);
	write_cmos_sensor(0x6F12, 0xB1FB);
	write_cmos_sensor(0x6F12, 0xF0F0);
	write_cmos_sensor(0x6F12, 0x0911);
	write_cmos_sensor(0x6F12, 0x8842);
	write_cmos_sensor(0x6F12, 0x04D2);
	write_cmos_sensor(0x6F12, 0x4028);
	write_cmos_sensor(0x6F12, 0x00D8);
	write_cmos_sensor(0x6F12, 0x4020);
	write_cmos_sensor(0x6F12, 0x5880);
	write_cmos_sensor(0x6F12, 0x10BD);
	write_cmos_sensor(0x6F12, 0x0846);
	write_cmos_sensor(0x6F12, 0xFBE7);
	write_cmos_sensor(0x6F12, 0x4168);
	write_cmos_sensor(0x6F12, 0x4A7B);
	write_cmos_sensor(0x6F12, 0x4C49);
	write_cmos_sensor(0x6F12, 0xA1F8);
	write_cmos_sensor(0x6F12, 0x8223);
	write_cmos_sensor(0x6F12, 0x4268);
	write_cmos_sensor(0x6F12, 0x537B);
	write_cmos_sensor(0x6F12, 0x002B);
	write_cmos_sensor(0x6F12, 0x15D0);
	write_cmos_sensor(0x6F12, 0x01F5);
	write_cmos_sensor(0x6F12, 0x6171);
	write_cmos_sensor(0x6F12, 0x927B);
	write_cmos_sensor(0x6F12, 0x0A80);
	write_cmos_sensor(0x6F12, 0x4068);
	write_cmos_sensor(0x6F12, 0xC07B);
	write_cmos_sensor(0x6F12, 0x4880);
	write_cmos_sensor(0x6F12, 0x4648);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0xC220);
	write_cmos_sensor(0x6F12, 0x8A80);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0xC420);
	write_cmos_sensor(0x6F12, 0xCA80);
	write_cmos_sensor(0x6F12, 0x10F8);
	write_cmos_sensor(0x6F12, 0xC72F);
	write_cmos_sensor(0x6F12, 0xC078);
	write_cmos_sensor(0x6F12, 0x5208);
	write_cmos_sensor(0x6F12, 0x4008);
	write_cmos_sensor(0x6F12, 0x42EA);
	write_cmos_sensor(0x6F12, 0x8000);
	write_cmos_sensor(0x6F12, 0x0881);
	write_cmos_sensor(0x6F12, 0x7047);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xFF4F);
	write_cmos_sensor(0x6F12, 0x3E48);
	write_cmos_sensor(0x6F12, 0x83B0);
	write_cmos_sensor(0x6F12, 0x1D46);
	write_cmos_sensor(0x6F12, 0xC079);
	write_cmos_sensor(0x6F12, 0xDDF8);
	write_cmos_sensor(0x6F12, 0x44B0);
	write_cmos_sensor(0x6F12, 0x1646);
	write_cmos_sensor(0x6F12, 0x0F46);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0x5AD0);
	write_cmos_sensor(0x6F12, 0xDFF8);
	write_cmos_sensor(0x6F12, 0xE0A0);
	write_cmos_sensor(0x6F12, 0x0AF1);
	write_cmos_sensor(0x6F12, 0xBA0A);
	write_cmos_sensor(0x6F12, 0xAAF1);
	write_cmos_sensor(0x6F12, 0x1C00);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x0090);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x0480);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xDDFA);
	write_cmos_sensor(0x6F12, 0x0399);
	write_cmos_sensor(0x6F12, 0x109C);
	write_cmos_sensor(0x6F12, 0x0843);
	write_cmos_sensor(0x6F12, 0x04F1);
	write_cmos_sensor(0x6F12, 0x8044);
	write_cmos_sensor(0x6F12, 0x07D0);
	write_cmos_sensor(0x6F12, 0xA780);
	write_cmos_sensor(0x6F12, 0xE680);
	write_cmos_sensor(0x6F12, 0xAAF1);
	write_cmos_sensor(0x6F12, 0x1C00);
	write_cmos_sensor(0x6F12, 0x0188);
	write_cmos_sensor(0x6F12, 0x2181);
	write_cmos_sensor(0x6F12, 0x8088);
	write_cmos_sensor(0x6F12, 0x20E0);
	write_cmos_sensor(0x6F12, 0x2048);
	write_cmos_sensor(0x6F12, 0x9AF8);
	write_cmos_sensor(0x6F12, 0x0C10);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0xD801);
	write_cmos_sensor(0x6F12, 0x4843);
	write_cmos_sensor(0x6F12, 0x0290);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xCBFA);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0x0298);
	write_cmos_sensor(0x6F12, 0x01D0);
	write_cmos_sensor(0x6F12, 0x361A);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0x0744);
	write_cmos_sensor(0x6F12, 0xA780);
	write_cmos_sensor(0x6F12, 0xE680);
	write_cmos_sensor(0x6F12, 0x1848);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0xDA61);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0x8A02);
	write_cmos_sensor(0x6F12, 0x4643);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xC0FA);
	write_cmos_sensor(0x6F12, 0x10B1);
	write_cmos_sensor(0x6F12, 0xA8EB);
	write_cmos_sensor(0x6F12, 0x0608);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0xB144);
	write_cmos_sensor(0x6F12, 0xA4F8);
	write_cmos_sensor(0x6F12, 0x0890);
	write_cmos_sensor(0x6F12, 0x4046);
	write_cmos_sensor(0x6F12, 0x6081);
	write_cmos_sensor(0x6F12, 0x0398);
	write_cmos_sensor(0x6F12, 0x28B1);
	write_cmos_sensor(0x6F12, 0x0E48);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0x4F11);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0x8902);
	write_cmos_sensor(0x6F12, 0x03E0);
	write_cmos_sensor(0x6F12, 0x9AF8);
	write_cmos_sensor(0x6F12, 0x0D10);
	write_cmos_sensor(0x6F12, 0x9AF8);
	write_cmos_sensor(0x6F12, 0x0C00);
	write_cmos_sensor(0x6F12, 0x0A01);
	write_cmos_sensor(0x6F12, 0x0949);
	write_cmos_sensor(0x6F12, 0x91F8);
	write_cmos_sensor(0x6F12, 0x4E11);
	write_cmos_sensor(0x6F12, 0x42EA);
	write_cmos_sensor(0x6F12, 0x8121);
	write_cmos_sensor(0x6F12, 0x41F0);
	write_cmos_sensor(0x6F12, 0x0301);
	write_cmos_sensor(0x6F12, 0xA181);
	write_cmos_sensor(0x6F12, 0x0121);
	write_cmos_sensor(0x6F12, 0xFF22);
	write_cmos_sensor(0x6F12, 0x02EB);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0x41EA);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0xE081);
	write_cmos_sensor(0x6F12, 0x01A9);
	write_cmos_sensor(0x6F12, 0x6846);
	write_cmos_sensor(0x6F12, 0x1AE0);
	write_cmos_sensor(0x6F12, 0x6EE0);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x2ED0);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x51A0);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x0E20);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0x8A2A);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x3642);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0x8832);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x3420);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x21A0);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x3F40);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x3E70);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0xA000);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x38C0);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x2210);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x80FA);
	write_cmos_sensor(0x6F12, 0x9DF8);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x9DF8);
	write_cmos_sensor(0x6F12, 0x0410);
	write_cmos_sensor(0x6F12, 0x40EA);
	write_cmos_sensor(0x6F12, 0x0120);
	write_cmos_sensor(0x6F12, 0x2082);
	write_cmos_sensor(0x6F12, 0x5F46);
	write_cmos_sensor(0x6F12, 0x3E46);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x70FA);
	write_cmos_sensor(0x6F12, 0x791E);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0xA889);
	write_cmos_sensor(0x6F12, 0x03D0);
	write_cmos_sensor(0x6F12, 0x4718);
	write_cmos_sensor(0x6F12, 0x46F6);
	write_cmos_sensor(0x6F12, 0xA410);
	write_cmos_sensor(0x6F12, 0x02E0);
	write_cmos_sensor(0x6F12, 0x4618);
	write_cmos_sensor(0x6F12, 0x46F6);
	write_cmos_sensor(0x6F12, 0x2410);
	write_cmos_sensor(0x6F12, 0xA880);
	write_cmos_sensor(0x6F12, 0x6782);
	write_cmos_sensor(0x6F12, 0xE682);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0xA082);
	write_cmos_sensor(0x6F12, 0xA888);
	write_cmos_sensor(0x6F12, 0x2080);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x66FA);
	write_cmos_sensor(0x6F12, 0x0128);
	write_cmos_sensor(0x6F12, 0x0CD1);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x67FA);
	write_cmos_sensor(0x6F12, 0x48B1);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x69FA);
	write_cmos_sensor(0x6F12, 0x30B1);
	write_cmos_sensor(0x6F12, 0x40F2);
	write_cmos_sensor(0x6F12, 0x1340);
	write_cmos_sensor(0x6F12, 0xA081);
	write_cmos_sensor(0x6F12, 0x40F2);
	write_cmos_sensor(0x6F12, 0x0110);
	write_cmos_sensor(0x6F12, 0xE081);
	write_cmos_sensor(0x6F12, 0x2082);
	write_cmos_sensor(0x6F12, 0x2B6A);
	write_cmos_sensor(0x6F12, 0x0021);
	write_cmos_sensor(0x6F12, 0x8320);
	write_cmos_sensor(0x6F12, 0x109A);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x60FA);
	write_cmos_sensor(0x6F12, 0xE881);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x4EFA);
	write_cmos_sensor(0x6F12, 0x0126);
	write_cmos_sensor(0x6F12, 0x0128);
	write_cmos_sensor(0x6F12, 0x12D1);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x4EFA);
	write_cmos_sensor(0x6F12, 0x78B1);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x50FA);
	write_cmos_sensor(0x6F12, 0x60B1);
	write_cmos_sensor(0x6F12, 0x2680);
	write_cmos_sensor(0x6F12, 0xCA48);
	write_cmos_sensor(0x6F12, 0x0021);
	write_cmos_sensor(0x6F12, 0x04E0);
	write_cmos_sensor(0x6F12, 0x0288);
	write_cmos_sensor(0x6F12, 0x5208);
	write_cmos_sensor(0x6F12, 0x20F8);
	write_cmos_sensor(0x6F12, 0x022B);
	write_cmos_sensor(0x6F12, 0x491C);
	write_cmos_sensor(0x6F12, 0xEA89);
	write_cmos_sensor(0x6F12, 0xB1EB);
	write_cmos_sensor(0x6F12, 0x420F);
	write_cmos_sensor(0x6F12, 0xF6DB);
	write_cmos_sensor(0x6F12, 0xE989);
	write_cmos_sensor(0x6F12, 0xA889);
	write_cmos_sensor(0x6F12, 0x8142);
	write_cmos_sensor(0x6F12, 0x00D9);
	write_cmos_sensor(0x6F12, 0xE881);
	write_cmos_sensor(0x6F12, 0x2680);
	write_cmos_sensor(0x6F12, 0x07B0);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF08F);
	write_cmos_sensor(0x6F12, 0x70B5);
	write_cmos_sensor(0x6F12, 0xC048);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x816B);
	write_cmos_sensor(0x6F12, 0x0C0C);
	write_cmos_sensor(0x6F12, 0x8DB2);
	write_cmos_sensor(0x6F12, 0x2946);
	write_cmos_sensor(0x6F12, 0x2046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xF3F9);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x37FA);
	write_cmos_sensor(0x6F12, 0xBB48);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0x7300);
	write_cmos_sensor(0x6F12, 0x30B1);
	write_cmos_sensor(0x6F12, 0xBA48);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0x8B02);
	write_cmos_sensor(0x6F12, 0x10B1);
	write_cmos_sensor(0x6F12, 0xB94A);
	write_cmos_sensor(0x6F12, 0x1B20);
	write_cmos_sensor(0x6F12, 0x1080);
	write_cmos_sensor(0x6F12, 0x2946);
	write_cmos_sensor(0x6F12, 0x2046);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0x7040);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xDFB9);
	write_cmos_sensor(0x6F12, 0x70B5);
	write_cmos_sensor(0x6F12, 0xB148);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xC16B);
	write_cmos_sensor(0x6F12, 0x0C0C);
	write_cmos_sensor(0x6F12, 0x8DB2);
	write_cmos_sensor(0x6F12, 0x2946);
	write_cmos_sensor(0x6F12, 0x2046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xD5F9);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x1EFA);
	write_cmos_sensor(0x6F12, 0xAC4A);
	write_cmos_sensor(0x6F12, 0x92F8);
	write_cmos_sensor(0x6F12, 0x7400);
	write_cmos_sensor(0x6F12, 0x10B1);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x82F8);
	write_cmos_sensor(0x6F12, 0x7000);
	write_cmos_sensor(0x6F12, 0x2946);
	write_cmos_sensor(0x6F12, 0x2046);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0x7040);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xC5B9);
	write_cmos_sensor(0x6F12, 0xA849);
	write_cmos_sensor(0x6F12, 0x0A8F);
	write_cmos_sensor(0x6F12, 0x498F);
	write_cmos_sensor(0x6F12, 0x801A);
	write_cmos_sensor(0x6F12, 0xA44A);
	write_cmos_sensor(0x6F12, 0x401E);
	write_cmos_sensor(0x6F12, 0xB2F8);
	write_cmos_sensor(0x6F12, 0xB032);
	write_cmos_sensor(0x6F12, 0x1944);
	write_cmos_sensor(0x6F12, 0x8142);
	write_cmos_sensor(0x6F12, 0x00D9);
	write_cmos_sensor(0x6F12, 0x0846);
	write_cmos_sensor(0x6F12, 0xA2F8);
	write_cmos_sensor(0x6F12, 0xB202);
	write_cmos_sensor(0x6F12, 0x7047);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0x0646);
	write_cmos_sensor(0x6F12, 0x9B48);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x406C);
	write_cmos_sensor(0x6F12, 0x85B2);
	write_cmos_sensor(0x6F12, 0x040C);
	write_cmos_sensor(0x6F12, 0x2946);
	write_cmos_sensor(0x6F12, 0x2046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xAAF9);
	write_cmos_sensor(0x6F12, 0x3046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xF7F9);
	write_cmos_sensor(0x6F12, 0x9A48);
	write_cmos_sensor(0x6F12, 0x9B4F);
	write_cmos_sensor(0x6F12, 0x0068);
	write_cmos_sensor(0x6F12, 0x3B68);
	write_cmos_sensor(0x6F12, 0x418B);
	write_cmos_sensor(0x6F12, 0x090A);
	write_cmos_sensor(0x6F12, 0x83F8);
	write_cmos_sensor(0x6F12, 0x3610);
	write_cmos_sensor(0x6F12, 0xC17E);
	write_cmos_sensor(0x6F12, 0x83F8);
	write_cmos_sensor(0x6F12, 0x3810);
	write_cmos_sensor(0x6F12, 0x9249);
	write_cmos_sensor(0x6F12, 0x91F8);
	write_cmos_sensor(0x6F12, 0x4C21);
	write_cmos_sensor(0x6F12, 0x002A);
	write_cmos_sensor(0x6F12, 0xD1F8);
	write_cmos_sensor(0x6F12, 0x3421);
	write_cmos_sensor(0x6F12, 0x01D0);
	write_cmos_sensor(0x6F12, 0x521C);
	write_cmos_sensor(0x6F12, 0x5208);
	write_cmos_sensor(0x6F12, 0xCE33);
	write_cmos_sensor(0x6F12, 0x160A);
	write_cmos_sensor(0x6F12, 0x1E71);
	write_cmos_sensor(0x6F12, 0x9A71);
	write_cmos_sensor(0x6F12, 0xB1F8);
	write_cmos_sensor(0x6F12, 0x3C21);
	write_cmos_sensor(0x6F12, 0xC2F3);
	write_cmos_sensor(0x6F12, 0x5712);
	write_cmos_sensor(0x6F12, 0x1A70);
	write_cmos_sensor(0x6F12, 0x91F8);
	write_cmos_sensor(0x6F12, 0x3D21);
	write_cmos_sensor(0x6F12, 0xD200);
	write_cmos_sensor(0x6F12, 0x9A70);
	write_cmos_sensor(0x6F12, 0x91F8);
	write_cmos_sensor(0x6F12, 0x4D21);
	write_cmos_sensor(0x6F12, 0xCE3B);
	write_cmos_sensor(0x6F12, 0x22B1);
	write_cmos_sensor(0x6F12, 0xD1F8);
	write_cmos_sensor(0x6F12, 0x3821);
	write_cmos_sensor(0x6F12, 0x521C);
	write_cmos_sensor(0x6F12, 0x5608);
	write_cmos_sensor(0x6F12, 0x01E0);
	write_cmos_sensor(0x6F12, 0xD1F8);
	write_cmos_sensor(0x6F12, 0x3861);
	write_cmos_sensor(0x6F12, 0x7A68);
	write_cmos_sensor(0x6F12, 0x4FEA);
	write_cmos_sensor(0x6F12, 0x162C);
	write_cmos_sensor(0x6F12, 0x01F5);
	write_cmos_sensor(0x6F12, 0x9071);
	write_cmos_sensor(0x6F12, 0x82F8);
	write_cmos_sensor(0x6F12, 0x16C0);
	write_cmos_sensor(0x6F12, 0x1676);
	write_cmos_sensor(0x6F12, 0xCE8B);
	write_cmos_sensor(0x6F12, 0x00F5);
	write_cmos_sensor(0x6F12, 0xBA70);
	write_cmos_sensor(0x6F12, 0xC6F3);
	write_cmos_sensor(0x6F12, 0x5716);
	write_cmos_sensor(0x6F12, 0x9674);
	write_cmos_sensor(0x6F12, 0xCE7F);
	write_cmos_sensor(0x6F12, 0xF600);
	write_cmos_sensor(0x6F12, 0x1675);
	write_cmos_sensor(0x6F12, 0x0E8C);
	write_cmos_sensor(0x6F12, 0xCF68);
	write_cmos_sensor(0x6F12, 0xF608);
	write_cmos_sensor(0x6F12, 0x7E43);
	write_cmos_sensor(0x6F12, 0x360B);
	write_cmos_sensor(0x6F12, 0x370A);
	write_cmos_sensor(0x6F12, 0x03F8);
	write_cmos_sensor(0x6F12, 0xD67F);
	write_cmos_sensor(0x6F12, 0x7732);
	write_cmos_sensor(0x6F12, 0x9E70);
	write_cmos_sensor(0x6F12, 0x0688);
	write_cmos_sensor(0x6F12, 0x360A);
	write_cmos_sensor(0x6F12, 0x02F8);
	write_cmos_sensor(0x6F12, 0x276C);
	write_cmos_sensor(0x6F12, 0x4678);
	write_cmos_sensor(0x6F12, 0x02F8);
	write_cmos_sensor(0x6F12, 0x256C);
	write_cmos_sensor(0x6F12, 0x8688);
	write_cmos_sensor(0x6F12, 0x360A);
	write_cmos_sensor(0x6F12, 0x02F8);
	write_cmos_sensor(0x6F12, 0x1F6C);
	write_cmos_sensor(0x6F12, 0x4679);
	write_cmos_sensor(0x6F12, 0x02F8);
	write_cmos_sensor(0x6F12, 0x1D6C);
	write_cmos_sensor(0x6F12, 0x6D4E);
	write_cmos_sensor(0x6F12, 0x96F8);
	write_cmos_sensor(0x6F12, 0x1064);
	write_cmos_sensor(0x6F12, 0xD671);
	write_cmos_sensor(0x6F12, 0x6B4E);
	write_cmos_sensor(0x6F12, 0x96F8);
	write_cmos_sensor(0x6F12, 0x1164);
	write_cmos_sensor(0x6F12, 0x5672);
	write_cmos_sensor(0x6F12, 0x694E);
	write_cmos_sensor(0x6F12, 0x96F8);
	write_cmos_sensor(0x6F12, 0x0B64);
	write_cmos_sensor(0x6F12, 0xD672);
	write_cmos_sensor(0x6F12, 0x674E);
	write_cmos_sensor(0x6F12, 0x96F8);
	write_cmos_sensor(0x6F12, 0x0964);
	write_cmos_sensor(0x6F12, 0x5673);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0x3060);
	write_cmos_sensor(0x6F12, 0xD673);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0xDE00);
	write_cmos_sensor(0x6F12, 0x02F8);
	write_cmos_sensor(0x6F12, 0x1F0F);
	write_cmos_sensor(0x6F12, 0x6148);
	write_cmos_sensor(0x6F12, 0x00F2);
	write_cmos_sensor(0x6F12, 0x7246);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0x7204);
	write_cmos_sensor(0x6F12, 0x9074);
	write_cmos_sensor(0x6F12, 0x3078);
	write_cmos_sensor(0x6F12, 0x1075);
	write_cmos_sensor(0x6F12, 0xA522);
	write_cmos_sensor(0x6F12, 0xDA70);
	write_cmos_sensor(0x6F12, 0x0E20);
	write_cmos_sensor(0x6F12, 0x1871);
	write_cmos_sensor(0x6F12, 0x11F8);
	write_cmos_sensor(0x6F12, 0x7E0C);
	write_cmos_sensor(0x6F12, 0xC0F1);
	write_cmos_sensor(0x6F12, 0x0C01);
	write_cmos_sensor(0x6F12, 0x5948);
	write_cmos_sensor(0x6F12, 0xD0F8);
	write_cmos_sensor(0x6F12, 0x3C04);
	write_cmos_sensor(0x6F12, 0xC840);
	write_cmos_sensor(0x6F12, 0x060A);
	write_cmos_sensor(0x6F12, 0x9E71);
	write_cmos_sensor(0x6F12, 0x1872);
	write_cmos_sensor(0x6F12, 0x0120);
	write_cmos_sensor(0x6F12, 0x03F8);
	write_cmos_sensor(0x6F12, 0x2C0C);
	write_cmos_sensor(0x6F12, 0x5448);
	write_cmos_sensor(0x6F12, 0xD0F8);
	write_cmos_sensor(0x6F12, 0x4C04);
	write_cmos_sensor(0x6F12, 0xC840);
	write_cmos_sensor(0x6F12, 0xAA21);
	write_cmos_sensor(0x6F12, 0x03F8);
	write_cmos_sensor(0x6F12, 0x571D);
	write_cmos_sensor(0x6F12, 0x0226);
	write_cmos_sensor(0x6F12, 0x5E70);
	write_cmos_sensor(0x6F12, 0x9A70);
	write_cmos_sensor(0x6F12, 0x3022);
	write_cmos_sensor(0x6F12, 0xDA70);
	write_cmos_sensor(0x6F12, 0x5A22);
	write_cmos_sensor(0x6F12, 0x1A71);
	write_cmos_sensor(0x6F12, 0x060A);
	write_cmos_sensor(0x6F12, 0x5E71);
	write_cmos_sensor(0x6F12, 0x9A71);
	write_cmos_sensor(0x6F12, 0xD871);
	write_cmos_sensor(0x6F12, 0x1972);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x5872);
	write_cmos_sensor(0x6F12, 0x2946);
	write_cmos_sensor(0x6F12, 0x2046);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x05B9);
	write_cmos_sensor(0x6F12, 0x70B5);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF6);
	write_cmos_sensor(0x6F12, 0xC131);
	write_cmos_sensor(0x6F12, 0x4948);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x53F9);
	write_cmos_sensor(0x6F12, 0x414C);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF6);
	write_cmos_sensor(0x6F12, 0xE921);
	write_cmos_sensor(0x6F12, 0xA060);
	write_cmos_sensor(0x6F12, 0x4648);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x4BF9);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF6);
	write_cmos_sensor(0x6F12, 0x7D11);
	write_cmos_sensor(0x6F12, 0xE060);
	write_cmos_sensor(0x6F12, 0x4448);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x44F9);
	write_cmos_sensor(0x6F12, 0x2061);
	write_cmos_sensor(0x6F12, 0xAFF6);
	write_cmos_sensor(0x6F12, 0x6770);
	write_cmos_sensor(0x6F12, 0x424D);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0xF751);
	write_cmos_sensor(0x6F12, 0xA862);
	write_cmos_sensor(0x6F12, 0x4048);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x39F9);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0xA351);
	write_cmos_sensor(0x6F12, 0x6062);
	write_cmos_sensor(0x6F12, 0x3E48);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x32F9);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF6);
	write_cmos_sensor(0x6F12, 0x4111);
	write_cmos_sensor(0x6F12, 0xA062);
	write_cmos_sensor(0x6F12, 0x3B48);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x2BF9);
	write_cmos_sensor(0x6F12, 0x6061);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0xB940);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x0B41);
	write_cmos_sensor(0x6F12, 0xE861);
	write_cmos_sensor(0x6F12, 0x3748);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x21F9);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x6721);
	write_cmos_sensor(0x6F12, 0x6063);
	write_cmos_sensor(0x6F12, 0x3548);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x1AF9);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x3721);
	write_cmos_sensor(0x6F12, 0xA063);
	write_cmos_sensor(0x6F12, 0x3248);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x13F9);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF6);
	write_cmos_sensor(0x6F12, 0x4B11);
	write_cmos_sensor(0x6F12, 0xE063);
	write_cmos_sensor(0x6F12, 0x3048);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x0CF9);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x4D71);
	write_cmos_sensor(0x6F12, 0x2062);
	write_cmos_sensor(0x6F12, 0x2D48);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x05F9);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x0171);
	write_cmos_sensor(0x6F12, 0xA061);
	write_cmos_sensor(0x6F12, 0x2B48);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xFEF8);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x3B21);
	write_cmos_sensor(0x6F12, 0xE061);
	write_cmos_sensor(0x6F12, 0x2848);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xF7F8);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x0351);
	write_cmos_sensor(0x6F12, 0x2064);
	write_cmos_sensor(0x6F12, 0x2648);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xF0F8);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0xBB41);
	write_cmos_sensor(0x6F12, 0xE062);
	write_cmos_sensor(0x6F12, 0x2348);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xE9F8);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x4921);
	write_cmos_sensor(0x6F12, 0x2063);
	write_cmos_sensor(0x6F12, 0x2148);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xE2F8);
	write_cmos_sensor(0x6F12, 0x3C22);
	write_cmos_sensor(0x6F12, 0x6064);
	write_cmos_sensor(0x6F12, 0x1146);
	write_cmos_sensor(0x6F12, 0x48F6);
	write_cmos_sensor(0x6F12, 0x0210);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x45F8);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xDEF8);
	write_cmos_sensor(0x6F12, 0x0949);
	write_cmos_sensor(0x6F12, 0x40F2);
	write_cmos_sensor(0x6F12, 0x4A40);
	write_cmos_sensor(0x6F12, 0x0968);
	write_cmos_sensor(0x6F12, 0x4883);
	write_cmos_sensor(0x6F12, 0x70BD);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x8000);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x51A0);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x2850);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x2ED0);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0xF47E);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x0E20);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x08D0);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x36E0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x7BAD);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x79A5);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xAB79);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x0850);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xDE1F);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x5F3B);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xB257);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xD719);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x27FF);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x39E3);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xA989);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xA739);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xA857);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x32CF);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x1E3B);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xEC45);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x67B9);
	write_cmos_sensor(0x6F12, 0x47F2);
	write_cmos_sensor(0x6F12, 0x0D2C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x47F2);
	write_cmos_sensor(0x6F12, 0x1B2C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4AF2);
	write_cmos_sensor(0x6F12, 0x471C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x49F6);
	write_cmos_sensor(0x6F12, 0xB17C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x45F6);
	write_cmos_sensor(0x6F12, 0xED2C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x47F6);
	write_cmos_sensor(0x6F12, 0x770C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4AF6);
	write_cmos_sensor(0x6F12, 0x733C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x49F6);
	write_cmos_sensor(0x6F12, 0x597C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x49F6);
	write_cmos_sensor(0x6F12, 0x857C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4FF2);
	write_cmos_sensor(0x6F12, 0x1D1C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4CF6);
	write_cmos_sensor(0x6F12, 0x594C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4CF6);
	write_cmos_sensor(0x6F12, 0x7B4C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x44F2);
	write_cmos_sensor(0x6F12, 0xAF1C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x47F2);
	write_cmos_sensor(0x6F12, 0xCF5C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x43F6);
	write_cmos_sensor(0x6F12, 0xA52C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4AF2);
	write_cmos_sensor(0x6F12, 0x2B1C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4DF6);
	write_cmos_sensor(0x6F12, 0x1F6C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x44F6);
	write_cmos_sensor(0x6F12, 0x655C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x010C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x45F6);
	write_cmos_sensor(0x6F12, 0x433C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x45F6);
	write_cmos_sensor(0x6F12, 0xE36C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x46F2);
	write_cmos_sensor(0x6F12, 0x7B1C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x44F6);
	write_cmos_sensor(0x6F12, 0xD90C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x44F6);
	write_cmos_sensor(0x6F12, 0x791C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x44F6);
	write_cmos_sensor(0x6F12, 0x811C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x44F2);
	write_cmos_sensor(0x6F12, 0xB50C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x44F6);
	write_cmos_sensor(0x6F12, 0xE90C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x43F6);
	write_cmos_sensor(0x6F12, 0x155C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x43F6);
	write_cmos_sensor(0x6F12, 0x1D5C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4DF2);
	write_cmos_sensor(0x6F12, 0xC95C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x42F2);
	write_cmos_sensor(0x6F12, 0xFF7C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x43F6);
	write_cmos_sensor(0x6F12, 0xE31C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x46F2);
	write_cmos_sensor(0x6F12, 0xB97C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4CF2);
	write_cmos_sensor(0x6F12, 0x2D1C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x46F2);
	write_cmos_sensor(0x6F12, 0x8D3C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);

	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x16F0);
	write_cmos_sensor(0x6F12, 0x2929);
	write_cmos_sensor(0x602A, 0x16F2);
	write_cmos_sensor(0x6F12, 0x2929);
	write_cmos_sensor(0x602A, 0x16FA);
	write_cmos_sensor(0x6F12, 0x0029);
	write_cmos_sensor(0x602A, 0x16FC);
	write_cmos_sensor(0x6F12, 0x0029);
	write_cmos_sensor(0x602A, 0x1708);
	write_cmos_sensor(0x6F12, 0x0029);
	write_cmos_sensor(0x602A, 0x170A);
	write_cmos_sensor(0x6F12, 0x0029);
	write_cmos_sensor(0x602A, 0x1712);
	write_cmos_sensor(0x6F12, 0x2929);
	write_cmos_sensor(0x602A, 0x1714);
	write_cmos_sensor(0x6F12, 0x2929);
	write_cmos_sensor(0x602A, 0x1716);
	write_cmos_sensor(0x6F12, 0x2929);
	write_cmos_sensor(0x602A, 0x1722);
	write_cmos_sensor(0x6F12, 0x152A);
	write_cmos_sensor(0x602A, 0x1724);
	write_cmos_sensor(0x6F12, 0x152A);
	write_cmos_sensor(0x602A, 0x172C);
	write_cmos_sensor(0x6F12, 0x002A);
	write_cmos_sensor(0x602A, 0x172E);
	write_cmos_sensor(0x6F12, 0x002A);
	write_cmos_sensor(0x602A, 0x1736);
	write_cmos_sensor(0x6F12, 0x1500);
	write_cmos_sensor(0x602A, 0x1738);
	write_cmos_sensor(0x6F12, 0x1500);
	write_cmos_sensor(0x602A, 0x1740);
	write_cmos_sensor(0x6F12, 0x152A);
	write_cmos_sensor(0x602A, 0x1742);
	write_cmos_sensor(0x6F12, 0x152A);
	write_cmos_sensor(0x602A, 0x16BE);
	write_cmos_sensor(0x6F12, 0x1515);
	write_cmos_sensor(0x6F12, 0x1515);
	write_cmos_sensor(0x602A, 0x16C8);
	write_cmos_sensor(0x6F12, 0x0029);
	write_cmos_sensor(0x6F12, 0x0029);
	write_cmos_sensor(0x602A, 0x16D6);
	write_cmos_sensor(0x6F12, 0x0015);
	write_cmos_sensor(0x6F12, 0x0015);
	write_cmos_sensor(0x6F12, 0x16E0);
	write_cmos_sensor(0x6F12, 0x2929);
	write_cmos_sensor(0x6F12, 0x2929);
	write_cmos_sensor(0x6F12, 0x2929);
	write_cmos_sensor(0x602A, 0x19B8);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x51A4);
	write_cmos_sensor(0x6F12, 0x2F04);
	write_cmos_sensor(0x6F12, 0x010F);
	write_cmos_sensor(0x602A, 0x2224);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x0DF8);
	write_cmos_sensor(0x6F12, 0x1001);
	write_cmos_sensor(0x602A, 0x1EDA);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x602A, 0x16A0);
	write_cmos_sensor(0x6F12, 0x3D09);
	write_cmos_sensor(0x602A, 0x10A8);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x602A, 0x1198);
	write_cmos_sensor(0x6F12, 0x002B);
	write_cmos_sensor(0x602A, 0x1758);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x602A, 0x1002);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x0F70);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x002F);
	write_cmos_sensor(0x602A, 0x0F76);
	write_cmos_sensor(0x6F12, 0x0030);
	write_cmos_sensor(0x602A, 0x0F7A);
	write_cmos_sensor(0x6F12, 0x000B);
	write_cmos_sensor(0x6F12, 0x0009);
	write_cmos_sensor(0x6F12, 0xF46E);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0FEA, 0x1440);
	write_cmos_sensor(0x0B06, 0x0101);
	write_cmos_sensor(0xF44A, 0x0006);
	write_cmos_sensor(0xF456, 0x0008);
	write_cmos_sensor(0xF46A, 0xBFA0);

	write_cmos_sensor(0x0100, 0x0000);

}	/*	sensor_init  */


static void preview_setting(void)
{
  LOG_INF("E\n");

  write_cmos_sensor(0x0100, 0x0000);
  check_output_stream_off();
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6214, 0x7970);
	write_cmos_sensor(0x6218, 0x7150);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x0ED6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CF0);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x602A, 0x0E58);
	write_cmos_sensor(0x6F12, 0x0023);
	write_cmos_sensor(0x602A, 0x1694);
	write_cmos_sensor(0x6F12, 0x1F0F);
	write_cmos_sensor(0x602A, 0x16AA);
	write_cmos_sensor(0x6F12, 0x009D);
	write_cmos_sensor(0x6F12, 0x000F);
	write_cmos_sensor(0x602A, 0x1098);
	write_cmos_sensor(0x6F12, 0x0012);
	write_cmos_sensor(0x602A, 0x2690);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x16A8);
	write_cmos_sensor(0x6F12, 0x38C0);
	write_cmos_sensor(0x602A, 0x108C);
	write_cmos_sensor(0x6F12, 0x0002);
	write_cmos_sensor(0x602A, 0x10CC);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x10D0);
	write_cmos_sensor(0x6F12, 0x000F);
	write_cmos_sensor(0x602A, 0x0F50);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0344, 0x0000);
	write_cmos_sensor(0x0346, 0x0000);
	write_cmos_sensor(0x0348, 0x122F);
	write_cmos_sensor(0x034A, 0x0DAF);
	write_cmos_sensor(0x034C, 0x0918);
	write_cmos_sensor(0x034E, 0x06D8);
	write_cmos_sensor(0x0900, 0x0122);
	write_cmos_sensor(0x0380, 0x0002);
	write_cmos_sensor(0x0382, 0x0002);
	write_cmos_sensor(0x0384, 0x0002);
	write_cmos_sensor(0x0386, 0x0002);
	write_cmos_sensor(0x0404, 0x1000);
	write_cmos_sensor(0x0402, 0x1010);
	write_cmos_sensor(0x0400, 0x1010);
	write_cmos_sensor(0x0114, 0x0300);
	write_cmos_sensor(0x0110, 0x1002);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x0300, 0x0007);
	write_cmos_sensor(0x0302, 0x0001);
	write_cmos_sensor(0x0304, 0x0006);
	write_cmos_sensor(0x0306, 0x00F5);
	write_cmos_sensor(0x0308, 0x0008);
	write_cmos_sensor(0x030A, 0x0001);
	write_cmos_sensor(0x030C, 0x0000);
	write_cmos_sensor(0x030E, 0x0004);
	write_cmos_sensor(0x0310, 0x0064);
	write_cmos_sensor(0x0312, 0x0000);
	write_cmos_sensor(0x0340, 0x0E54);
	write_cmos_sensor(0x0342, 0x13E0);
	write_cmos_sensor(0x0202, 0x0100);
	write_cmos_sensor(0x0200, 0x0100);
	write_cmos_sensor(0x021E, 0x0000);
	write_cmos_sensor(0x0D00, 0x0000);
	write_cmos_sensor(0x0D02, 0x0001);
  write_cmos_sensor(0x0100, 0x0100);

}	/*	preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);
	if (currefps == 300) {

  write_cmos_sensor(0x0100, 0x0000);
  check_output_stream_off();
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6214, 0x7970);
	write_cmos_sensor(0x6218, 0x7150);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x0ED6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CF0);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x602A, 0x0E58);
	write_cmos_sensor(0x6F12, 0x0023);
	write_cmos_sensor(0x602A, 0x1694);
	write_cmos_sensor(0x6F12, 0x1F0F);
	write_cmos_sensor(0x602A, 0x16AA);
	write_cmos_sensor(0x6F12, 0x009D);
	write_cmos_sensor(0x6F12, 0x000F);
	write_cmos_sensor(0x602A, 0x1098);
	write_cmos_sensor(0x6F12, 0x0012);
	write_cmos_sensor(0x602A, 0x2690);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x16A8);
	write_cmos_sensor(0x6F12, 0x38C0);
	write_cmos_sensor(0x602A, 0x108C);
	write_cmos_sensor(0x6F12, 0x0002);
	write_cmos_sensor(0x602A, 0x10CC);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x10D0);
	write_cmos_sensor(0x6F12, 0x000F);
	write_cmos_sensor(0x602A, 0x0F50);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0344, 0x0000);
	write_cmos_sensor(0x0346, 0x0000);
	write_cmos_sensor(0x0348, 0x122F);
	write_cmos_sensor(0x034A, 0x0DAF);
	write_cmos_sensor(0x034C, 0x0918);
	write_cmos_sensor(0x034E, 0x06D8);
	write_cmos_sensor(0x0900, 0x0122);
	write_cmos_sensor(0x0380, 0x0002);
	write_cmos_sensor(0x0382, 0x0002);
	write_cmos_sensor(0x0384, 0x0002);
	write_cmos_sensor(0x0386, 0x0002);
	write_cmos_sensor(0x0404, 0x1000);
	write_cmos_sensor(0x0402, 0x1010);
	write_cmos_sensor(0x0400, 0x1010);
	write_cmos_sensor(0x0114, 0x0300);
	write_cmos_sensor(0x0110, 0x1002);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x0300, 0x0007);
	write_cmos_sensor(0x0302, 0x0001);
	write_cmos_sensor(0x0304, 0x0006);
	write_cmos_sensor(0x0306, 0x00F5);
	write_cmos_sensor(0x0308, 0x0008);
	write_cmos_sensor(0x030A, 0x0001);
	write_cmos_sensor(0x030C, 0x0000);
	write_cmos_sensor(0x030E, 0x0004);
	write_cmos_sensor(0x0310, 0x0064);
	write_cmos_sensor(0x0312, 0x0000);
	write_cmos_sensor(0x0340, 0x0E54); 
	write_cmos_sensor(0x0342, 0x13E0);
	write_cmos_sensor(0x0202, 0x0100);
	write_cmos_sensor(0x0200, 0x0100);
	write_cmos_sensor(0x021E, 0x0000);
	write_cmos_sensor(0x0D00, 0x0000);
	write_cmos_sensor(0x0D02, 0x0001);
  write_cmos_sensor(0x0100, 0x0100);

	} else if (currefps == 240)
	{
		LOG_INF("else if (currefps == 240)\n");
  write_cmos_sensor(0x0100, 0x0000);
  check_output_stream_off();
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6214, 0x7970);
	write_cmos_sensor(0x6218, 0x7150);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x0ED6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CF0);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x602A, 0x0E58);
	write_cmos_sensor(0x6F12, 0x0023);
	write_cmos_sensor(0x602A, 0x1694);
	write_cmos_sensor(0x6F12, 0x1F0F);
	write_cmos_sensor(0x602A, 0x16AA);
	write_cmos_sensor(0x6F12, 0x009D);
	write_cmos_sensor(0x6F12, 0x000F);
	write_cmos_sensor(0x602A, 0x1098);
	write_cmos_sensor(0x6F12, 0x0012);
	write_cmos_sensor(0x602A, 0x2690);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x16A8);
	write_cmos_sensor(0x6F12, 0x38C0);
	write_cmos_sensor(0x602A, 0x108C);
	write_cmos_sensor(0x6F12, 0x0002);
	write_cmos_sensor(0x602A, 0x10CC);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x10D0);
	write_cmos_sensor(0x6F12, 0x000F);
	write_cmos_sensor(0x602A, 0x0F50);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0344, 0x0000);
	write_cmos_sensor(0x0346, 0x0000);
	write_cmos_sensor(0x0348, 0x122F);
	write_cmos_sensor(0x034A, 0x0DAF);
	write_cmos_sensor(0x034C, 0x0918);
	write_cmos_sensor(0x034E, 0x06D8);
	write_cmos_sensor(0x0900, 0x0122);
	write_cmos_sensor(0x0380, 0x0002);
	write_cmos_sensor(0x0382, 0x0002);
	write_cmos_sensor(0x0384, 0x0002);
	write_cmos_sensor(0x0386, 0x0002);
	write_cmos_sensor(0x0404, 0x1000);
	write_cmos_sensor(0x0402, 0x1010);
	write_cmos_sensor(0x0400, 0x1010);
	write_cmos_sensor(0x0114, 0x0300);
	write_cmos_sensor(0x0110, 0x1002);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x0300, 0x0007);
	write_cmos_sensor(0x0302, 0x0001);
	write_cmos_sensor(0x0304, 0x0006);
	write_cmos_sensor(0x0306, 0x00F5);
	write_cmos_sensor(0x0308, 0x0008);
	write_cmos_sensor(0x030A, 0x0001);
	write_cmos_sensor(0x030C, 0x0000);
	write_cmos_sensor(0x030E, 0x0004);
	write_cmos_sensor(0x0310, 0x0064);
	write_cmos_sensor(0x0312, 0x0000);
	write_cmos_sensor(0x0340, 0x11EA); 
	write_cmos_sensor(0x0342, 0x13E0);
	write_cmos_sensor(0x0202, 0x0100);
	write_cmos_sensor(0x0200, 0x0100);
	write_cmos_sensor(0x021E, 0x0000);
	write_cmos_sensor(0x0D00, 0x0000);
	write_cmos_sensor(0x0D02, 0x0001);
  write_cmos_sensor(0x0100, 0x0100);
	} else if (currefps == 150)
	{
		LOG_INF("else if (currefps == 150)\n");
  write_cmos_sensor(0x0100, 0x0000);
  check_output_stream_off();
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6214, 0x7970);
	write_cmos_sensor(0x6218, 0x7150);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x0ED6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CF0);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x602A, 0x0E58);
	write_cmos_sensor(0x6F12, 0x0023);
	write_cmos_sensor(0x602A, 0x1694);
	write_cmos_sensor(0x6F12, 0x1F0F);
	write_cmos_sensor(0x602A, 0x16AA);
	write_cmos_sensor(0x6F12, 0x009D);
	write_cmos_sensor(0x6F12, 0x000F);
	write_cmos_sensor(0x602A, 0x1098);
	write_cmos_sensor(0x6F12, 0x0012);
	write_cmos_sensor(0x602A, 0x2690);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x16A8);
	write_cmos_sensor(0x6F12, 0x38C0);
	write_cmos_sensor(0x602A, 0x108C);
	write_cmos_sensor(0x6F12, 0x0002);
	write_cmos_sensor(0x602A, 0x10CC);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x10D0);
	write_cmos_sensor(0x6F12, 0x000F);
	write_cmos_sensor(0x602A, 0x0F50);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0344, 0x0000);
	write_cmos_sensor(0x0346, 0x0000);
	write_cmos_sensor(0x0348, 0x122F);
	write_cmos_sensor(0x034A, 0x0DAF);
	write_cmos_sensor(0x034C, 0x0918);
	write_cmos_sensor(0x034E, 0x06D8);
	write_cmos_sensor(0x0900, 0x0122);
	write_cmos_sensor(0x0380, 0x0002);
	write_cmos_sensor(0x0382, 0x0002);
	write_cmos_sensor(0x0384, 0x0002);
	write_cmos_sensor(0x0386, 0x0002);
	write_cmos_sensor(0x0404, 0x1000);
	write_cmos_sensor(0x0402, 0x1010);
	write_cmos_sensor(0x0400, 0x1010);
	write_cmos_sensor(0x0114, 0x0300);
	write_cmos_sensor(0x0110, 0x1002);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x0300, 0x0007);
	write_cmos_sensor(0x0302, 0x0001);
	write_cmos_sensor(0x0304, 0x0006);
	write_cmos_sensor(0x0306, 0x00F5);
	write_cmos_sensor(0x0308, 0x0008);
	write_cmos_sensor(0x030A, 0x0001);
	write_cmos_sensor(0x030C, 0x0000);
	write_cmos_sensor(0x030E, 0x0004);
	write_cmos_sensor(0x0310, 0x0064);
	write_cmos_sensor(0x0312, 0x0000);
	write_cmos_sensor(0x0340, 0x1CA8); 
	write_cmos_sensor(0x0342, 0x13E0);
	write_cmos_sensor(0x0202, 0x0100);
	write_cmos_sensor(0x0200, 0x0100);
	write_cmos_sensor(0x021E, 0x0000);
	write_cmos_sensor(0x0D00, 0x0000);
	write_cmos_sensor(0x0D02, 0x0001);
  write_cmos_sensor(0x0100, 0x0100);
	} else { 

		LOG_INF("else  150fps\n");
  write_cmos_sensor(0x0100, 0x0000);
  check_output_stream_off();
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6214, 0x7970);
	write_cmos_sensor(0x6218, 0x7150);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x0ED6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CF0);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x602A, 0x0E58);
	write_cmos_sensor(0x6F12, 0x0023);
	write_cmos_sensor(0x602A, 0x1694);
	write_cmos_sensor(0x6F12, 0x1F0F);
	write_cmos_sensor(0x602A, 0x16AA);
	write_cmos_sensor(0x6F12, 0x009D);
	write_cmos_sensor(0x6F12, 0x000F);
	write_cmos_sensor(0x602A, 0x1098);
	write_cmos_sensor(0x6F12, 0x0012);
	write_cmos_sensor(0x602A, 0x2690);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x16A8);
	write_cmos_sensor(0x6F12, 0x38C0);
	write_cmos_sensor(0x602A, 0x108C);
	write_cmos_sensor(0x6F12, 0x0002);
	write_cmos_sensor(0x602A, 0x10CC);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x10D0);
	write_cmos_sensor(0x6F12, 0x000F);
	write_cmos_sensor(0x602A, 0x0F50);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0344, 0x0000);
	write_cmos_sensor(0x0346, 0x0000);
	write_cmos_sensor(0x0348, 0x122F);
	write_cmos_sensor(0x034A, 0x0DAF);
	write_cmos_sensor(0x034C, 0x0918);
	write_cmos_sensor(0x034E, 0x06D8);
	write_cmos_sensor(0x0900, 0x0122);
	write_cmos_sensor(0x0380, 0x0002);
	write_cmos_sensor(0x0382, 0x0002);
	write_cmos_sensor(0x0384, 0x0002);
	write_cmos_sensor(0x0386, 0x0002);
	write_cmos_sensor(0x0404, 0x1000);
	write_cmos_sensor(0x0402, 0x1010);
	write_cmos_sensor(0x0400, 0x1010);
	write_cmos_sensor(0x0114, 0x0300);
	write_cmos_sensor(0x0110, 0x1002);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x0300, 0x0007);
	write_cmos_sensor(0x0302, 0x0001);
	write_cmos_sensor(0x0304, 0x0006);
	write_cmos_sensor(0x0306, 0x00F5);
	write_cmos_sensor(0x0308, 0x0008);
	write_cmos_sensor(0x030A, 0x0001);
	write_cmos_sensor(0x030C, 0x0000);
	write_cmos_sensor(0x030E, 0x0004);
	write_cmos_sensor(0x0310, 0x0064);
	write_cmos_sensor(0x0312, 0x0000);
	write_cmos_sensor(0x0340, 0x1CA8); 
	write_cmos_sensor(0x0342, 0x13E0);
	write_cmos_sensor(0x0202, 0x0100);
	write_cmos_sensor(0x0200, 0x0100);
	write_cmos_sensor(0x021E, 0x0000);
	write_cmos_sensor(0x0D00, 0x0000);
	write_cmos_sensor(0x0D02, 0x0001);
  write_cmos_sensor(0x0100, 0x0100);
	}
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);
  preview_setting();
}

static void hs_video_setting(void)
{
	LOG_INF("E\n");

  write_cmos_sensor(0x0100, 0x0000);
  check_output_stream_off();
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6214, 0x7970);
	write_cmos_sensor(0x6218, 0x7150);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x0ED6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CF0);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x602A, 0x0E58);
	write_cmos_sensor(0x6F12, 0x0023);
	write_cmos_sensor(0x602A, 0x1694);
	write_cmos_sensor(0x6F12, 0x1F0F);
	write_cmos_sensor(0x602A, 0x16AA);
	write_cmos_sensor(0x6F12, 0x009D);
	write_cmos_sensor(0x6F12, 0x000F);
	write_cmos_sensor(0x602A, 0x1098);
	write_cmos_sensor(0x6F12, 0x0012);
	write_cmos_sensor(0x602A, 0x2690);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x16A8);
	write_cmos_sensor(0x6F12, 0x38C0);
	write_cmos_sensor(0x602A, 0x108C);
	write_cmos_sensor(0x6F12, 0x0002);
	write_cmos_sensor(0x602A, 0x10CC);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x10D0);
	write_cmos_sensor(0x6F12, 0x000F);
	write_cmos_sensor(0x602A, 0x0F50);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0344, 0x0000);
	write_cmos_sensor(0x0346, 0x0000);
	write_cmos_sensor(0x0348, 0x122F);
	write_cmos_sensor(0x034A, 0x0DAF);
	write_cmos_sensor(0x034C, 0x0918);
	write_cmos_sensor(0x034E, 0x06D8);
	write_cmos_sensor(0x0900, 0x0122);
	write_cmos_sensor(0x0380, 0x0002);
	write_cmos_sensor(0x0382, 0x0002);
	write_cmos_sensor(0x0384, 0x0002);
	write_cmos_sensor(0x0386, 0x0002);
	write_cmos_sensor(0x0404, 0x1000);
	write_cmos_sensor(0x0402, 0x1010);
	write_cmos_sensor(0x0400, 0x1010);
	write_cmos_sensor(0x0114, 0x0300);
	write_cmos_sensor(0x0110, 0x1002);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x0300, 0x0007);
	write_cmos_sensor(0x0302, 0x0001);
	write_cmos_sensor(0x0304, 0x0006);
	write_cmos_sensor(0x0306, 0x00F5);
	write_cmos_sensor(0x0308, 0x0008);
	write_cmos_sensor(0x030A, 0x0001);
	write_cmos_sensor(0x030C, 0x0000);
	write_cmos_sensor(0x030E, 0x0004);
	write_cmos_sensor(0x0310, 0x0064);
	write_cmos_sensor(0x0312, 0x0000);
	write_cmos_sensor(0x0340, 0x072A);
	write_cmos_sensor(0x0342, 0x13E0);
	write_cmos_sensor(0x0202, 0x0100);
	write_cmos_sensor(0x0200, 0x0100);
	write_cmos_sensor(0x021E, 0x0000);
	write_cmos_sensor(0x0D00, 0x0000);
	write_cmos_sensor(0x0D02, 0x0001);
  write_cmos_sensor(0x0100, 0x0100);
}

static void slim_video_setting(void)
{
	LOG_INF("E\n");

  write_cmos_sensor(0x0100, 0x0000);
  check_output_stream_off();
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6214, 0x7970);
	write_cmos_sensor(0x6218, 0x7150);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x0ED6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CF0);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x602A, 0x0E58);
	write_cmos_sensor(0x6F12, 0x0023);
	write_cmos_sensor(0x602A, 0x1694);
	write_cmos_sensor(0x6F12, 0x1F0F);
	write_cmos_sensor(0x602A, 0x16AA);
	write_cmos_sensor(0x6F12, 0x009D);
	write_cmos_sensor(0x6F12, 0x000F);
	write_cmos_sensor(0x602A, 0x1098);
	write_cmos_sensor(0x6F12, 0x0012);
	write_cmos_sensor(0x602A, 0x2690);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x16A8);
	write_cmos_sensor(0x6F12, 0x38C0);
	write_cmos_sensor(0x602A, 0x108C);
	write_cmos_sensor(0x6F12, 0x0002);
	write_cmos_sensor(0x602A, 0x10CC);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x10D0);
	write_cmos_sensor(0x6F12, 0x000F);
	write_cmos_sensor(0x602A, 0x0F50);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0344, 0x0000);
	write_cmos_sensor(0x0346, 0x0000);
	write_cmos_sensor(0x0348, 0x122F);
	write_cmos_sensor(0x034A, 0x0DAF);
	write_cmos_sensor(0x034C, 0x0918);
	write_cmos_sensor(0x034E, 0x06D8);
	write_cmos_sensor(0x0900, 0x0122);
	write_cmos_sensor(0x0380, 0x0002);
	write_cmos_sensor(0x0382, 0x0002);
	write_cmos_sensor(0x0384, 0x0002);
	write_cmos_sensor(0x0386, 0x0002);
	write_cmos_sensor(0x0404, 0x1000);
	write_cmos_sensor(0x0402, 0x1010);
	write_cmos_sensor(0x0400, 0x1010);
	write_cmos_sensor(0x0114, 0x0300);
	write_cmos_sensor(0x0110, 0x1002);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x0300, 0x0007);
	write_cmos_sensor(0x0302, 0x0001);
	write_cmos_sensor(0x0304, 0x0006);
	write_cmos_sensor(0x0306, 0x00F5);
	write_cmos_sensor(0x0308, 0x0008);
	write_cmos_sensor(0x030A, 0x0001);
	write_cmos_sensor(0x030C, 0x0000);
	write_cmos_sensor(0x030E, 0x0004);
	write_cmos_sensor(0x0310, 0x0064);
	write_cmos_sensor(0x0312, 0x0000);
	write_cmos_sensor(0x0340, 0x0E54);
	write_cmos_sensor(0x0342, 0x13E0);
	write_cmos_sensor(0x0202, 0x0100);
	write_cmos_sensor(0x0200, 0x0100);
	write_cmos_sensor(0x021E, 0x0000);
	write_cmos_sensor(0x0D00, 0x0000);
	write_cmos_sensor(0x0D02, 0x0001);
  write_cmos_sensor(0x0100, 0x0100);

}


static kal_uint32 return_sensor_id(void)
{
    return ((read_cmos_sensor_byte(0x0000) << 8) | read_cmos_sensor_byte(0x0001));
}

/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	*sensorID : return the sensor ID
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
	spin_lock(&imgsensor_drv_lock);
	imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
	spin_unlock(&imgsensor_drv_lock);
	printk("[get_imgsensor_id]s5k3p9mipi get sensor i2c_write_id=0x%x.\n",imgsensor.i2c_write_id);
	do {
	    *sensor_id = return_sensor_id();
		printk("[get_imgsensor_id]s5k3p9mipi get sensor *sensor_id=0x%x.\n",*sensor_id);
	    if (*sensor_id == imgsensor_info.sensor_id) {
#ifdef CONFIG_MTK_CAM_CAL

#endif
				LOG_INF("[get_imgsensor_id]s5k3p9mipi sensor get i2c write id: 0x%x, ReadOut sensor id: 0x%x, imgsensor_info.sensor_id:0x%x.\n", imgsensor.i2c_write_id, *sensor_id, imgsensor_info.sensor_id);
		return ERROR_NONE;
	    }
			LOG_INF("[get_imgsensor_id]s5k3p9mipi Read sensor id fail, i2c write id: 0x%x, ReadOut sensor id: 0x%x, imgsensor_info.sensor_id:0x%x.\n", imgsensor.i2c_write_id, *sensor_id, imgsensor_info.sensor_id);
	    retry--;
	} while (retry > 0);
	i++;
	retry = 1;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
	/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
	*sensor_id = 0xFFFFFFFF;
	return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
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
static kal_uint32 open(void)
{
    /* const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2}; */
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint32 sensor_id = 0;
	LOG_1;
    /* sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address */
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
	spin_lock(&imgsensor_drv_lock);
	imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
	spin_unlock(&imgsensor_drv_lock);
	do {
	    sensor_id = return_sensor_id();
	    if (sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
		break;
	    }
            LOG_INF("Read sensor id fail, id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
	    retry--;
	} while (retry > 0);
	i++;
	if (sensor_id == imgsensor_info.sensor_id)
	    break;
	retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id)
	return ERROR_SENSOR_CONNECT_FAIL;

    /* initail sequence write in  */
    sensor_init();

    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en = KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_en = KAL_FALSE;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}   /*  open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*
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
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/

	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	int i = 0;
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	/* imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	set_mirror_flip(IMAGE_NORMAL);
	mdelay(10);
	//#ifdef FANPENGTAO
	for (i = 0; i < 10; i++) {
		LOG_INF("delay time = %d, the frame no = %d\n", i*10, read_cmos_sensor(0x0005));
		mdelay(10);
	}
	//#endif
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap.max_framerate) {
		LOG_INF("capture30fps: use cap30FPS's setting: %d fps!\n", imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
		/* PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M */
		LOG_INF("cap115fps: use cap1's setting: %d fps!\n", imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else  { //PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		LOG_INF("Warning:=== current_fps %d fps is not support, so use cap1's setting\n", imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	set_mirror_flip(IMAGE_NORMAL);
	mdelay(10);

#if 0
	if (imgsensor.test_pattern == KAL_TRUE)
	{
		/* write_cmos_sensor(0x5002,0x00); */
  }
#endif

	return ERROR_NONE;
}	/* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	set_mirror_flip(IMAGE_NORMAL);


	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(IMAGE_NORMAL);

	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;
    sensor_resolution->SensorCustom1Width  = imgsensor_info.custom1.grabwindow_width;
    sensor_resolution->SensorCustom1Height     = imgsensor_info.custom1.grabwindow_height;

    sensor_resolution->SensorCustom2Width  = imgsensor_info.custom2.grabwindow_width;
    sensor_resolution->SensorCustom2Height     = imgsensor_info.custom2.grabwindow_height;

    sensor_resolution->SensorCustom3Width  = imgsensor_info.custom3.grabwindow_width;
    sensor_resolution->SensorCustom3Height     = imgsensor_info.custom3.grabwindow_height;

    sensor_resolution->SensorCustom4Width  = imgsensor_info.custom4.grabwindow_width;
    sensor_resolution->SensorCustom4Height     = imgsensor_info.custom4.grabwindow_height;

    sensor_resolution->SensorCustom5Width  = imgsensor_info.custom5.grabwindow_width;
    sensor_resolution->SensorCustom5Height     = imgsensor_info.custom5.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; /* inverse with datasheet */
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;


	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  /* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;
	#ifdef FPTPDAFSUPPORT
	sensor_info->PDAF_Support = 1;
	#else
	sensor_info->PDAF_Support = 0;
	#endif

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

			break;
		default:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}

	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) /* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else /* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if (framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			if (framerate == 300)
			{
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			} else
			{
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			}
			if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
			break;
		default:  /* coding with  preview scenario by default */
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			/* set_dummy(); */
			LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
			break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*framerate = imgsensor_info.pre.max_framerate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*framerate = imgsensor_info.cap.max_framerate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*framerate = imgsensor_info.hs_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*framerate = imgsensor_info.slim_video.max_framerate;
			break;
		default:
			break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {

		 write_cmos_sensor(0x0600, 0x0002);

	} else {

		 write_cmos_sensor(0x0600, 0x0000);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para, UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
    UINT16 *feature_data_16 = (UINT16 *) feature_para;
    UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
    UINT32 *feature_data_32 = (UINT32 *) feature_para;
    unsigned long long *feature_data = (unsigned long long *) feature_para;

    struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
	struct SET_PD_BLOCK_INFO_T *PDAFinfo;

    LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
	    *feature_return_para_16++ = imgsensor.line_length;
	    *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len = 4;
	    break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
	    *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len = 4;
	    break;
	case SENSOR_FEATURE_SET_ESHUTTER:
	    set_shutter(*feature_data);
	    break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) * feature_data);
	    break;
	case SENSOR_FEATURE_SET_GAIN:
	    set_gain((UINT16) *feature_data);
	    break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
	    break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
	    break;
	case SENSOR_FEATURE_SET_REGISTER:
	    write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
	    break;
	case SENSOR_FEATURE_GET_REGISTER:
	    sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
	    break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
	    /* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE */
	    /* if EEPROM does not exist in camera module. */
            *feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len = 4;
	    break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
	    set_video_mode(*feature_data);
	    break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
	    get_imgsensor_id(feature_return_para_32);
	    break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16, *(feature_data_16+1));
	    break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
	    set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
	    break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
	    get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
	    break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
	    set_test_pattern_mode((BOOL)*feature_data);
	    break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
	    *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len = 4;
	    break;
	case SENSOR_FEATURE_SET_FRAMERATE:
	    LOG_INF("current fps :%d\n", (UINT32)*feature_data);
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.current_fps = *feature_data;
	    spin_unlock(&imgsensor_drv_lock);
	    break;
	case SENSOR_FEATURE_SET_HDR:
	    LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.ihdr_en = (BOOL)*feature_data;
	    spin_unlock(&imgsensor_drv_lock);
	    break;
	case SENSOR_FEATURE_GET_CROP_INFO:
	    LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);

	    wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

	    switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		    break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		    break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		    break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		    break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
                    memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		    break;
	    }
			break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n", (UINT16)*feature_data, (UINT16)*(feature_data+1), (UINT16)*(feature_data+2));
	    break;
	/******************** PDAF START >>> *********/
		case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
			LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%llu\n", *feature_data);
			/* PDAF capacity enable or not, 2p8 only full size support PDAF */
			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				default:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
			}
			break;
		case SENSOR_FEATURE_GET_PDAF_INFO:
			LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%llu\n", *feature_data);
			PDAFinfo = (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));

			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info, sizeof(struct SET_PD_BLOCK_INFO_T));
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					break;
			}
			break;
		case SENSOR_FEATURE_GET_PDAF_DATA:
			LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
			/* S5K3p9_read_eeprom((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2))); */
			break;
	/******************** PDAF END   <<< *********/
	default:
	    break;
    }

    return ERROR_NONE;
}    /*    feature_control()  */


static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};


UINT32 S5K3p9_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc =  &sensor_func;
	return ERROR_NONE;
}	/*	S5K3p9_MIPI_RAW_SensorInit	*/




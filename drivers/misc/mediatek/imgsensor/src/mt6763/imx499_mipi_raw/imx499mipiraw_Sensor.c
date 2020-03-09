/*****************************************************************************
 *
 * Filename:
 * ---------
 *     IMX298mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
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
//#include <asm/system.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "imx499mipiraw_Sensor.h"

/****************************Modify following Strings for debug****************************/
#define PFX "IMX499_camera_sensor"

#define LOG_1 LOG_INF("IMX499,MIPI 4LANE\n")
#define LOG_2 LOG_INF("preview 2328*1748@30fps,864Mbps/lane; video 4196*3108@30fps,864Mbps/lane; capture 4656*3496 16M@30fps,864Mbps/lane\n")
/****************************   Modify end    *******************************************/
#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)

#define BYTE               unsigned char

//static BOOL read_lrc_flag = FALSE;


static DEFINE_SPINLOCK(imgsensor_drv_lock);

//static BYTE imx499_LRC_data[252] = { 0 };

static imgsensor_info_struct imgsensor_info = {
	.sensor_id = IMX499_SENSOR_ID,	/* record sensor id defined in Kd_imgsensor.h */

	.checksum_value = 0x11dcf259,	/* checksum value for Camera Auto Test */
	.pre = {
		.pclk = 280000000,	/* record different mode's pclk */
		.linelength = 5120,	/* record different mode's linelength */
		.framelength = 1822,	/* record different mode's framelength */
		.startx = 0,	/* record different mode's startx of grabwindow */
		.starty = 0,	/* record different mode's starty of grabwindow */
		.grabwindow_width = 2304,//2328,	/* 2096,        //record different mode's width of grabwindow */
		.grabwindow_height = 1728,//1746,	/* 1552,       //record different mode's height of grabwindow */
		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/* following for GetDefaultFramerateByScenario() */
		.max_framerate = 300,
	},
	.cap = {		/* 30  fps  capture */
		.pclk = 552000000,
		.linelength = 5120,
		.framelength = 3592,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4608,//4656,	/* 4192, */
		.grabwindow_height = 3456,//3496,	/* 3104, */
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
	},
	.cap1 = {		/* 15 fps  capture */
		.pclk = 276000000,
		.linelength = 5120,
		.framelength = 3592,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4608,//4656,	/* 4192, */
		.grabwindow_height = 3456,//3496,	/* 3104, */
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 150,
	},
	.normal_video = {	/* 30  fps  capture */
		.pclk = 552000000,
		.linelength = 5120,
		.framelength = 3592,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4608,//4656,	/* 4192, */
		.grabwindow_height = 3456,//2608,	/* 3104, */
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.hs_video = {		/* 120 fps */
		.pclk = 580000000,
		.linelength = 2560,
		.framelength = 1888,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 1200,
	},
	.slim_video = {
		.pclk = 240000000,
		.linelength = 5120,
		.framelength = 780,	/* 1640, */
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 30,
		.max_framerate = 600,
	},
	.custom1 = {//same as preview
		.pclk = 280000000,	/* record different mode's pclk */
		.linelength = 5120,	/* record different mode's linelength */
		.framelength = 1822,	/* record different mode's framelength */
		.startx = 0,	/* record different mode's startx of grabwindow */
		.starty = 0,	/* record different mode's starty of grabwindow */
		.grabwindow_width = 2304,//2328,	/* 2096,        //record different mode's width of grabwindow */
		.grabwindow_height = 1728,//1746,	/* 1552,       //record different mode's height of grabwindow */
		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/* following for GetDefaultFramerateByScenario() */
		.max_framerate = 300,
	},
	.custom2 = {//same as preview
		.pclk = 280000000,	/* record different mode's pclk */
		.linelength = 5120,	/* record different mode's linelength */
		.framelength = 1822,	/* record different mode's framelength */
		.startx = 0,	/* record different mode's startx of grabwindow */
		.starty = 0,	/* record different mode's starty of grabwindow */
		.grabwindow_width = 2304,//2328,	/* 2096,        //record different mode's width of grabwindow */
		.grabwindow_height = 1728,//1746,	/* 1552,       //record different mode's height of grabwindow */
		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/* following for GetDefaultFramerateByScenario() */
		.max_framerate = 300,
	},
	.custom3 = {//same as preview
		.pclk = 280000000,	/* record different mode's pclk */
		.linelength = 5120,	/* record different mode's linelength */
		.framelength = 1822,	/* record different mode's framelength */
		.startx = 0,	/* record different mode's startx of grabwindow */
		.starty = 0,	/* record different mode's starty of grabwindow */
		.grabwindow_width = 2304,//2328,	/* 2096,        //record different mode's width of grabwindow */
		.grabwindow_height = 1728,//1746,	/* 1552,       //record different mode's height of grabwindow */
		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/* following for GetDefaultFramerateByScenario() */
		.max_framerate = 300,
	},
	.custom4 = {//same as preview
		.pclk = 280000000,	/* record different mode's pclk */
		.linelength = 5120,	/* record different mode's linelength */
		.framelength = 1822,	/* record different mode's framelength */
		.startx = 0,	/* record different mode's startx of grabwindow */
		.starty = 0,	/* record different mode's starty of grabwindow */
		.grabwindow_width = 2304,//2328,	/* 2096,        //record different mode's width of grabwindow */
		.grabwindow_height = 1728,//1746,	/* 1552,       //record different mode's height of grabwindow */
		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/* following for GetDefaultFramerateByScenario() */
		.max_framerate = 300,
	},
	.custom5 = {//same as preview
		.pclk = 280000000,	/* record different mode's pclk */
		.linelength = 5120,	/* record different mode's linelength */
		.framelength = 1822,	/* record different mode's framelength */
		.startx = 0,	/* record different mode's startx of grabwindow */
		.starty = 0,	/* record different mode's starty of grabwindow */
		.grabwindow_width = 2304,//2328,	/* 2096,        //record different mode's width of grabwindow */
		.grabwindow_height = 1728,//1746,	/* 1552,       //record different mode's height of grabwindow */
		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/* following for GetDefaultFramerateByScenario() */
		.max_framerate = 300,
	},
	.margin = 4,		/* sensor framelength & shutter margin */
	.min_shutter = 1,	/* min shutter */
	.max_frame_length = 0x7fff,	/* max framelength by sensor register's limitation */
	.ae_shut_delay_frame = 0,	/* shutter delay frame for AE cycle,
					 * 2 frame with ispGain_delay-shut_delay=2-0=2
					 */
	.ae_sensor_gain_delay_frame = 0,	/* sensor gain delay frame for AE cycle,
						 * 2 frame with ispGain_delay-sensor_gain_delay=2-0=2
						 */
	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	//.frame_time_delay_frame = 2,/* The delay frame of setting frame length  */
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.sensor_mode_num = 10,	/* support sensor mode num */

	.cap_delay_frame = 3,	/* enter capture delay frame num */
	.pre_delay_frame = 3,	/* enter preview delay frame num */
	.video_delay_frame = 3,	/* enter video delay frame num */
	.hs_video_delay_frame = 3,	/* enter high speed video  delay frame num */
	.slim_video_delay_frame = 3,	/* enter slim video delay frame num */

	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,	/* sensor_interface_type */
	.mipi_sensor_type = MIPI_OPHY_NCSI2,	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = 1,	/* 0,MIPI_SETTLEDELAY_AUTO;
								 * 1,MIPI_SETTLEDELAY_MANNUAL
								 */
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,	/* sensor output first pixel color */
	.mclk = 24,	//24	/* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	.mipi_lane_num = SENSOR_MIPI_4_LANE,	/* mipi lane num */
	.i2c_addr_table = {0x34, 0x20, 0xff},	/* record sensor support all write id addr,
						 * only supprt 4must end with 0xff
						 */
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,	/* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT,	/* IMGSENSOR_MODE enum value,record current sensor mode,such as:
						 * INIT, Preview, Capture, Video,High Speed Video, Slim Video
						 */
	.shutter = 0x3D0,	/* current shutter */
	.gain = 0x100,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 300,	/* full size current fps : 24fps for PIP, 30fps for Normal or ZSD */
	.autoflicker_en = KAL_FALSE,	/* auto flicker enable: KAL_FALSE for disable auto flicker,
					 * KAL_TRUE for enable auto flicker
					 */
	.test_pattern = KAL_FALSE,	/* test pattern mode or not. KAL_FALSE for in test pattern mode,
					 * KAL_TRUE for normal output
					 */
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,	/* current scenario id */
	.ihdr_mode = 0,		/* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x34,	/* record current sensor's i2c write id */
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] = {
	{4656, 3496, 0024, 0020, 4608, 3456, 2304, 1728, 0000, 0000, 2304, 1728, 0000, 0000, 2304, 1728},/* Preview */
	{4656, 3496, 0024, 0020, 4608, 3456, 4608, 3456, 0000, 0000, 4608, 3456, 0000, 0000, 4608, 3456},/* capture */
    {4656, 3496, 0024, 0020, 4608, 3456, 4608, 3456, 0000, 0000, 4608, 3456, 0000, 0000, 4608, 3456},/* video */
	{4656, 3496, 1048, 1028, 2560, 1440, 1280, 720, 0000, 0000, 1280, 720, 0000, 0000, 1280, 720},/* hs video */
	{4656, 3496, 1048, 1028, 2560, 1440, 1280, 720, 0000, 0000, 1280, 720, 0000, 0000, 1280, 720},/* slim video */
	{4656, 3496, 0024, 0020, 4608, 3456, 2304, 1728, 0000, 0000, 2304, 1728, 0000, 0000, 2304, 1728},/* Custom1 */
	{4656, 3496, 0024, 0020, 4608, 3456, 2304, 1728, 0000, 0000, 2304, 1728, 0000, 0000, 2304, 1728},/* Custom2 */
	{4656, 3496, 0024, 0020, 4608, 3456, 2304, 1728, 0000, 0000, 2304, 1728, 0000, 0000, 2304, 1728},/* Custom3 */
	{4656, 3496, 0024, 0020, 4608, 3456, 2304, 1728, 0000, 0000, 2304, 1728, 0000, 0000, 2304, 1728},/* Custom4 */
	{4656, 3496, 0024, 0020, 4608, 3456, 2304, 1728, 0000, 0000, 2304, 1728, 0000, 0000, 2304, 1728},/* Custom5 */
};				/* slim video */

/*VC1 for HDR(DT=0X35) , VC2 for PDAF(DT=0X36), unit : 8bit */
/*pdaf winth=1grabwindow_width*10/8;pdaf support 8bit */

static SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3] = {

	/* Preview mode setting */
	{
		0x03, 0x0a, 0x00, 0x10, 0x40, 0x00,
		0x00, 0x2b, 0x0918, 0x06d4, 0x00, 0x35, 0x0280, 0x0001,
		0x00, 0x34, 0x2BC, 0x0340/*340*/, 0x03, 0x00, 0x0000, 0x0000
	},
	/* Capture mode setting */
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
		0x00, 0x2b, 0x1230, 0x0DA8, 0x00, 0x35, 0x0280, 0x0001,
		0x00, 0x34, 0x2BC/*0x2bc*/, 0x340, 0x03, 0x00, 0x0000, 0x0000
	},
	/* Video mode setting */
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
		0x00, 0x2b, 0x1230, 0x0DA8, 0x00, 0x35, 0x0280, 0x0001,
		0x00, 0x34, 0x2BC/*0x2bc*/, 0x340, 0x03, 0x00, 0x0000, 0x0000
	}
  
   
};
static SET_PD_BLOCK_INFO_T pd_block_info_cap = {
	.i4OffsetX = 86,
	.i4OffsetY = 96,
	.i4PitchX = 32,
	.i4PitchY = 32,
	.i4PairNum = 16,
	.i4SubBlkW = 8,
	.i4SubBlkH = 8,
	.i4PosL = {
	      {88, 96}, {96, 96}, {104, 96}, {112, 96}, {92, 104}, {100, 104}, {108, 104}, {116, 104},
		  {88, 112},
		  {96, 112}, {104, 112}, {112, 112}, {92, 120}, {100, 120}, {108, 120}, {116, 120} 
		},
	.i4PosR = {
		  {87, 96}, {95, 96}, {103, 96}, {111, 96}, {91, 104}, {99, 104}, {107, 104}, {115, 104},
		  {87, 112},
		  {95, 112}, {103, 112}, {111, 112}/*{111, 102} */, {91, 120}, {99, 120}, {107, 120}, {115, 120} 
		},
	.iMirrorFlip = 0,	/* 0:IMAGE_NORMAL,1:IMAGE_H_MIRROR,2:IMAGE_V_MIRROR,3:IMAGE_HV_MIRROR */
	.i4BlockNumX = 140,
	.i4BlockNumY = 104,
	.i4Crop = { {0, 0}, {0, 0}, {0, 444}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} },
};

static SET_PD_BLOCK_INFO_T pd_block_info_normal_video = {
	.i4OffsetX = 86,
	.i4OffsetY = 96,
	.i4PitchX = 32,
	.i4PitchY = 32,
	.i4PairNum = 16,
	.i4SubBlkW = 8,
	.i4SubBlkH = 8,
	.i4PosL = {
	      {88, 96}, {96, 96}, {104, 96}, {112, 96}, {92, 104}, {100, 104}, {108, 104}, {116, 104},
		  {88, 112},
		  {96, 112}, {104, 112}, {112, 112}, {92, 120}, {100, 120}, {108, 120}, {116, 120} 
		},
	.i4PosR = {
		  {87, 96}, {95, 96}, {103, 96}, {111, 96}, {91, 104}, {99, 104}, {107, 104}, {115, 104},
		  {87, 112},
		  {95, 112}, {103, 112}, {111, 112}/*{111, 102} */, {91, 120}, {99, 120}, {107, 120}, {115, 120} 
		},
	.iMirrorFlip = 0,	/* 0:IMAGE_NORMAL,1:IMAGE_H_MIRROR,2:IMAGE_V_MIRROR,3:IMAGE_HV_MIRROR */
	.i4BlockNumX = 140,
	.i4BlockNumY = 104,
	.i4Crop = { {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} },
};


typedef struct {
	MUINT16 DarkLimit_H;
	MUINT16 DarkLimit_L;
	MUINT16 OverExp_Min_H;
	MUINT16 OverExp_Min_L;
	MUINT16 OverExp_Max_H;
	MUINT16 OverExp_Max_L;
} SENSOR_ATR_INFO, *pSENSOR_ATR_INFO;
#if 0
static SENSOR_ATR_INFO sensorATR_Info[4] = {	/* Strength Range Min */
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	/* Strength Range Std */
	{0x00, 0x32, 0x00, 0x3c, 0x03, 0xff},
	/* Strength Range Max */
	{0x3f, 0xff, 0x3f, 0xff, 0x3f, 0xff},
	/* Strength Range Custom */
	{0x3F, 0xFF, 0x00, 0x0, 0x3F, 0xFF}
};
#endif

#define imx499MIPI_MaxGainIndex (101)
kal_uint16 imx499MIPI_sensorGainMapping[imx499MIPI_MaxGainIndex][2] = {
{64, 	0     },
{66, 	31   },
{67, 	46   },
{69, 	74   },
{71, 	101 },
{73, 	126 },
{75, 	150 },
{77, 	173 },
{79, 	194 },
{81, 	215 },
{86, 	262 },
{90, 	296 },
{93, 	319 },
{99, 	362 },
{102,	381 },
{109,	423 },
{115,	454 },
{120,	478 },
{126,	504 },
{130,	520 },
{133,	531 },
{136,	542 },
{139,	553 },
{142,	562 },
{147,	578 },
{150,	587 },
{158,	609 },
{163,	622 },
{170,	638 },
{176,	652 },
{181,	662 },
{185,	670 },
{189,	677 },
{196,	690 },
{202,	700 },
{207,	707 },
{214,	718 },
{221,	727 },
{224,	731 },
{231,	740 },
{237,	747 },
{246,	758 },
{252,	764 },
{256,	768 },
{262,	774 },
{275,	786 },
{280,	790 },
{290,	798 },
{312,	814 },
{321,	820 },
{331,	826 },
{345,	834 },
{352,	838 },
{364,	844 },
{372,	848 },
{386,	854 },
{400,	860 },
{410,	864 },
{420,	868 },
{437,	874 },
{449,	878 },
{455,	880 },
{461,	882 },
{468,	884 },
{475,	886 },
{482,	888 },
{489,	890 },
{496,	892 },
{504,	894 },
{512,	896 },
{520,	898 },
{529,	900 },
{537,	902 },
{546,	904 },
{555,	906 },
{565,	908 },
{575,	910 },
{585,	912 },
{596,	914 },
{607,	916 },
{618,	918 },
{630,	920 },
{642,	922 },
{655,	924 },
{669,	926 },
{683,	928 },
{697,	930 },
{713,	932 },
{728,	934 },
{745,	936 },
{762,	938 },
{780,	940 },
{799,	942 },
{840,	946 },
{862,	948 },
{886,	950 },
{910,	952 },
{936,	954 },
{964,	956 },
{993,	958 },
{1024,	960 },
};


static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;

	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *) &get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = { (char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF) };

	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

#if 0
static kal_uint32 imx499_ATR(UINT16 DarkLimit, UINT16 OverExp)
{
	/*
	 * write_cmos_sensor(0x6e50,sensorATR_Info[DarkLimit].DarkLimit_H);
	 * write_cmos_sensor(0x6e51,sensorATR_Info[DarkLimit].DarkLimit_L);
	 * write_cmos_sensor(0x9340,sensorATR_Info[OverExp].OverExp_Min_H);
	 * write_cmos_sensor(03x941,sensorATR_Info[OverExp].OverExp_Min_L);
	 * write_cmos_sensor(0x9342,sensorATR_Info[OverExp].OverExp_Max_H);
	 * write_cmos_sensor(0x9343,sensorATR_Info[OverExp].OverExp_Max_L);
	 * write_cmos_sensor(0x9706,0x10);
	 * write_cmos_sensor(0x9707,0x03);
	 * write_cmos_sensor(0x9708,0x03);
	 * write_cmos_sensor(0x9e24,0x00);
	 * write_cmos_sensor(0x9e25,0x8c);
	 * write_cmos_sensor(0x9e26,0x00);
	 * write_cmos_sensor(0x9e27,0x94);
	 * write_cmos_sensor(0x9e28,0x00);
	 * write_cmos_sensor(0x9e29,0x96);
	 * LOG_INF("DarkLimit 0x6e50(0x%x), 0x6e51(0x%x)\n",sensorATR_Info[DarkLimit].DarkLimit_H,
	 * sensorATR_Info[DarkLimit].DarkLimit_L);
	 * LOG_INF("OverExpMin 0x9340(0x%x), 0x9341(0x%x)\n",sensorATR_Info[OverExp].OverExp_Min_H,
	 * sensorATR_Info[OverExp].OverExp_Min_L);
	 * LOG_INF("OverExpMin 0x9342(0x%x), 0x9343(0x%x)\n",sensorATR_Info[OverExp].OverExp_Max_H,
	 * sensorATR_Info[OverExp].OverExp_Max_L);
	 */
	return ERROR_NONE;
}
#endif

extern bool read_imx499_eeprom_LRC( kal_uint16 addr, BYTE* data, kal_uint32 size);

static kal_uint8 IMX499MIPI_LRC_Data[140];
static kal_uint8 LRC_data_done = false;
static void load_imx499_LRC_Data(void)
{
	if (LRC_data_done == false) {
		if (!read_imx499_eeprom_LRC(0x0CEC, IMX499MIPI_LRC_Data, 140)) {
			LOG_INF("imx499 load lrc fail\n");
			return;
		}
		LRC_data_done = true;
	}
}
static void write_imx499_LRC_Data(void)
{
	kal_uint16 i;
	if ( LRC_data_done == false ) {
		if (!read_imx499_eeprom_LRC(0x0CEC,IMX499MIPI_LRC_Data,140)) {
			LOG_INF("imx499 load lrc fail\n");
			return;
		}
		LRC_data_done = true;
	}

	for(i=0; i<70; i++)
	{
		write_cmos_sensor(0x7520+i, IMX499MIPI_LRC_Data[i]);
		
	}
	for(i=0; i<70; i++)
	{
		write_cmos_sensor(0x7568+i, IMX499MIPI_LRC_Data[i+70]);
		
	}
}




static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/*
	 * you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel,
	 * or you can set dummy by imgsensor.frame_length and imgsensor.line_length
	 */
	write_cmos_sensor(0x0104, 0x01);

	write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
	write_cmos_sensor(0x0342, imgsensor.line_length >> 8);
	write_cmos_sensor(0x0343, imgsensor.line_length & 0xFF);

	write_cmos_sensor(0x0104, 0x00);
}				/*    set_dummy  */

static kal_uint32 return_sensor_id(void)
{
	kal_uint8 tmph, tmpl;

	LOG_INF("Enter\n");
	write_cmos_sensor(0x0A02, 0x17);
	write_cmos_sensor(0x0A00, 0x01);
	mdelay(10);
	tmph = 0x00;
	tmpl = 0x00;
	tmph = read_cmos_sensor(0x0A22);
	tmpl = read_cmos_sensor(0x0A23) >> 4;
	LOG_INF("[ylf]tmph = %d, tmpl= %d\n", tmph, tmpl);
	return ((read_cmos_sensor(0x0A22) << 4) | (read_cmos_sensor(0x0A23) >> 4));
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;
	/* unsigned long flags; */

	LOG_INF("framerate = %d, min framelength should enable %d\n", framerate, min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length)
		? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	/* dummy_line = frame_length - imgsensor.min_frame_length; */
	/* if (dummy_line < 0) */
	/* imgsensor.dummy_line = 0; */
	/* else */
	/* imgsensor.dummy_line = dummy_line; */
	/* imgsensor.frame_length = frame_length + imgsensor.dummy_line; */
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}				/*    set_max_framerate  */



/*************************************************************************
 * FUNCTION
 *    set_shutter
 *
 * DESCRIPTION
 *    This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *    iShutter : exposured lines
 *
 * RETURNS
 *    None
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

	/* write_shutter(shutter); */
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

	/* OV Recommend Solution */
	/* if shutter bigger than frame_length, should extend frame length first */
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor(0x0104, 0x01);
			write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
			write_cmos_sensor(0x0104, 0x00);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor(0x0104, 0x01);
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x0104, 0x00);
	}

	/* Update Shutter */
	write_cmos_sensor(0x0104, 0x01);
	write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor(0x0203, shutter & 0xFF);
	write_cmos_sensor(0x0104, 0x00);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);

}				/*    set_shutter */



static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint8 iI;

	LOG_INF("[imx499MIPI]enter imx499MIPIGain2Reg function\n");
	for (iI = 0; iI < imx499MIPI_MaxGainIndex; iI++) {
		if (gain <= imx499MIPI_sensorGainMapping[iI][0])
			return imx499MIPI_sensorGainMapping[iI][1];
	}

	LOG_INF("exit imx499MIPIGain2Reg function\n");
	return imx499MIPI_sensorGainMapping[iI - 1][1];
}

/*************************************************************************
 * FUNCTION
 *    set_gain
 *
 * DESCRIPTION
 *    This function is to set global gain to sensor.
 *
 * PARAMETERS
 *    iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *    the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	/* 0x350A[0:1], 0x350B[0:7] AGC real gain */
	/* [0:3] = N meams N /16 X    */
	/* [4:9] = M meams M X         */
	/* Total gain = M + N /16 X   */

	/*  */
	if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
		printk("Error gain setting");

	if (gain < BASEGAIN)
		gain = BASEGAIN;
	else if (gain > 16 * BASEGAIN)
		gain = 16 * BASEGAIN;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	printk("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor(0x0104, 0x01);
	write_cmos_sensor(0x0204, (reg_gain >> 8) & 0xFF);
	write_cmos_sensor(0x0205, reg_gain & 0xFF);
	write_cmos_sensor(0x0104, 0x00);

	return gain;
}				/*    set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{

	kal_uint16 realtime_fps = 0;
	kal_uint16 reg_gain;

	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n", le, se, gain);
	spin_lock(&imgsensor_drv_lock);
	if (le > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = le + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	if (le < imgsensor_info.min_shutter)
		le = imgsensor_info.min_shutter;
	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			write_cmos_sensor(0x0104, 0x01);
			write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
			write_cmos_sensor(0x0104, 0x00);
		}
	} else {
		write_cmos_sensor(0x0104, 0x01);
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x0104, 0x00);
	}
	write_cmos_sensor(0x0104, 0x01);
	/* Long exposure */
	write_cmos_sensor(0x0202, (le >> 8) & 0xFF);
	write_cmos_sensor(0x0203, le & 0xFF);
	/* Short exposure */
	write_cmos_sensor(0x0224, (se >> 8) & 0xFF);
	write_cmos_sensor(0x0225, se & 0xFF);
	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	/* Global analog Gain for Long expo */
	write_cmos_sensor(0x0204, (reg_gain >> 8) & 0xFF);
	write_cmos_sensor(0x0205, reg_gain & 0xFF);
	/* Global analog Gain for Short expo */
	write_cmos_sensor(0x0216, (reg_gain >> 8) & 0xFF);
	write_cmos_sensor(0x0217, reg_gain & 0xFF);
	write_cmos_sensor(0x0104, 0x00);

}

#if 0
static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	/********************************************************
	 *
	 *   0x3820[2] ISP Vertical flip
	 *   0x3820[1] Sensor Vertical flip
	 *
	 *   0x3821[2] ISP Horizontal mirror
	 *   0x3821[1] Sensor Horizontal mirror
	 *
	 *   ISP and Sensor flip or mirror register bit should be the same!!
	 *
	 ********************************************************/

	switch (image_mirror) {
	case IMAGE_NORMAL:
		write_cmos_sensor(0x0101, 0x00);
		break;
	case IMAGE_H_MIRROR:
		write_cmos_sensor(0x0101, 0x01);
		break;
	case IMAGE_V_MIRROR:
		write_cmos_sensor(0x0101, 0x02);
		break;
	case IMAGE_HV_MIRROR:
		write_cmos_sensor(0x0101, 0x03);
		break;
	default:
		LOG_INF("Error image_mirror setting\n");
	}

}
#endif
/*************************************************************************
 * FUNCTION
 *    night_mode
 *
 * DESCRIPTION
 *    This function night mode of sensor.
 *
 * PARAMETERS
 *    bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void night_mode(kal_bool enable)
{
	/*No Need to implement this function*/
}				/*    night_mode    */

static void sensor_init(void)
{
	//printk("fsd---sensor_init-start\n");
	/* external clock setting */
	write_cmos_sensor(0x0136, 0x18);
	write_cmos_sensor(0x0137, 0x00);
	/* Register version */
	write_cmos_sensor(0x3C7E, 0x01);
	write_cmos_sensor(0x3C7F, 0x01);

	/* Global setting */
	write_cmos_sensor(0x3F7F, 0x01);
	write_cmos_sensor(0x4D44, 0x00);
	write_cmos_sensor(0x4D45, 0x27);
	write_cmos_sensor(0x531C, 0x01);
	write_cmos_sensor(0x531D, 0x02);
	write_cmos_sensor(0x531E, 0x04);
	write_cmos_sensor(0x5928, 0x00);
	write_cmos_sensor(0x5929, 0x28);
	write_cmos_sensor(0x592A, 0x00);
	write_cmos_sensor(0x592B, 0x7E);
	write_cmos_sensor(0x592C, 0x00);
	write_cmos_sensor(0x592D, 0x3A);
	write_cmos_sensor(0x592E, 0x00);
	write_cmos_sensor(0x592F, 0x90);
	write_cmos_sensor(0x5930, 0x00);
	write_cmos_sensor(0x5931, 0x3F);
	write_cmos_sensor(0x5932, 0x00);
	write_cmos_sensor(0x5933, 0x95);
	write_cmos_sensor(0x5938, 0x00);
	write_cmos_sensor(0x5939, 0x20);
	write_cmos_sensor(0x593A, 0x00);
	write_cmos_sensor(0x593B, 0x76);
	write_cmos_sensor(0x5B38, 0x00);
	write_cmos_sensor(0x5B79, 0x02);
	write_cmos_sensor(0x5B7A, 0x07);
	write_cmos_sensor(0x5B88, 0x05);
	write_cmos_sensor(0x5B8D, 0x05);
	write_cmos_sensor(0x5C2E, 0x00);
	write_cmos_sensor(0x5C54, 0x00);
	write_cmos_sensor(0x6F6D, 0x01);
	write_cmos_sensor(0x79A0, 0x01);
	write_cmos_sensor(0x79A8, 0x00);
	write_cmos_sensor(0x79A9, 0x46);
	write_cmos_sensor(0x79AA, 0x01);
	write_cmos_sensor(0x79AD, 0x00);
	write_cmos_sensor(0x8169, 0x01);
	write_cmos_sensor(0x8359, 0x01);
	write_cmos_sensor(0x9004, 0x02);
	write_cmos_sensor(0x9200, 0x6A);
	write_cmos_sensor(0x9201, 0x22);
	write_cmos_sensor(0x9202, 0x6A);
	write_cmos_sensor(0x9203, 0x23);
	write_cmos_sensor(0x9302, 0x23);
	write_cmos_sensor(0x9312, 0x37);
	write_cmos_sensor(0x9316, 0x37);
	write_cmos_sensor(0xB046, 0x01);
	write_cmos_sensor(0xB048, 0x01);
	
    //prize added by fengshangdong 
	write_cmos_sensor(0xE186, 0x34);// 31 or 34
	write_cmos_sensor(0xE1A6, 0x34);
    //prize end
    
	write_cmos_sensor(0x0101, 0x03);// mirror/flip

	
	/* image quality */
	write_cmos_sensor(0xAA06, 0x3F);
	write_cmos_sensor(0xAA07, 0x05);
	write_cmos_sensor(0xAA08, 0x04);
	write_cmos_sensor(0xAA12, 0x3F);
	write_cmos_sensor(0xAA13, 0x04);
	write_cmos_sensor(0xAA14, 0x03);
	write_cmos_sensor(0xAB55, 0x02);
	write_cmos_sensor(0xAB57, 0x01);
	write_cmos_sensor(0xAB59, 0x01);
	write_cmos_sensor(0xABB4, 0x00);
	write_cmos_sensor(0xABB5, 0x01);
	write_cmos_sensor(0xABB6, 0x00);
	write_cmos_sensor(0xABB7, 0x01);
	write_cmos_sensor(0xABB8, 0x00);
	write_cmos_sensor(0xABB9, 0x01);
	write_imx499_LRC_Data();
	//printk("fsd--sensor_init-end\n");

}				/*    sensor_init  */


static void preview_setting(void)
{
	LOG_INF("fsd---E\n");

	/*************MIPI output setting************/
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x0114, 0x03);
	/*************line length PCK setting************/
	write_cmos_sensor(0x0342, 0x14);
	write_cmos_sensor(0x0343, 0x00);
	/*************frame length lines setting************/
	write_cmos_sensor(0x0340, 0x07);
	write_cmos_sensor(0x0341, 0x1E);
	/************ROI size setting**********/
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x00);
	write_cmos_sensor(0x0347, 0x02);
	write_cmos_sensor(0x0348, 0x12);
	write_cmos_sensor(0x0349, 0x2F);
	write_cmos_sensor(0x034A, 0x0D);
	write_cmos_sensor(0x034B, 0xA5);
	/***********mode setting************/
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x01);
	write_cmos_sensor(0x0901, 0x22);
	write_cmos_sensor(0x0902, 0x0A);
	write_cmos_sensor(0x3F4C, 0x05);
	write_cmos_sensor(0x3F4D, 0x03);
	/*************digital crop and scaling setting*********/
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x00);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x09);
	write_cmos_sensor(0x040D, 0x18);
	write_cmos_sensor(0x040E, 0x06);
	write_cmos_sensor(0x040F, 0xD2);
	/*************output size setting******/
	//write_cmos_sensor(0x034C, 0x09);
	//write_cmos_sensor(0x034D, 0x18);
	//write_cmos_sensor(0x034E, 0x06);
	//write_cmos_sensor(0x034F, 0xD2);
	write_cmos_sensor(0x034C, 0x09);
	write_cmos_sensor(0x034D, 0x00);
	write_cmos_sensor(0x034E, 0x06);
	write_cmos_sensor(0x034F, 0xC0);
	/************clock setting****************/
	write_cmos_sensor(0x0301, 0x06);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x02);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0x46);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x02);
	write_cmos_sensor(0x030E, 0x00);
	write_cmos_sensor(0x030F, 0x28);
	write_cmos_sensor(0x0310, 0x01);
	write_cmos_sensor(0x0820, 0x0E);
	write_cmos_sensor(0x0821, 0xD0);
	write_cmos_sensor(0x0822, 0x00);
	write_cmos_sensor(0x0823, 0x00);
	/*******PDAF Setting*****************/
	/*write_cmos_sensor(0x3E20, 0x02);
	write_cmos_sensor(0x3E3B, 0x01);
	write_cmos_sensor(0x4434, 0x02);
	write_cmos_sensor(0x4435, 0x30);//30
	write_cmos_sensor(0x8271, 0x00);*/
	write_cmos_sensor(0x3E20, 0x02);
	write_cmos_sensor(0x3E3B, 0x00);
	write_cmos_sensor(0x4434, 0x00);
	write_cmos_sensor(0x4435, 0x00);//30
	write_cmos_sensor(0x8271, 0x00);
	/*******other Setting*****************/
	write_cmos_sensor(0x0106, 0x00);
	write_cmos_sensor(0x0B00, 0x00);
	write_cmos_sensor(0x3230, 0x00);
	write_cmos_sensor(0x3C00, 0x00);
	write_cmos_sensor(0x3C01, 0x75);
	write_cmos_sensor(0x3F78, 0x00);
	write_cmos_sensor(0x3F79, 0xF9);
	/*******integration Setting*****************/
	write_cmos_sensor(0x0202, 0x07);
	write_cmos_sensor(0x0203, 0x0C);
	/*******gain Setting*****************/
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);

}				/*    preview_setting  */

// no HDR mode

static void capture_setting(kal_uint16 currefps)
{
	//LOG_INF("E! currefps:%d\n", currefps);
    //printk("fsd---capture_setting-start\n");
	/*************MIPI output setting************/
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x0114, 0x03);
	/*************line length PCK setting************/
	write_cmos_sensor(0x0342, 0x14);
	write_cmos_sensor(0x0343, 0x00);
	/*************frame length lines setting************/
	write_cmos_sensor(0x0340, 0x0E);
	write_cmos_sensor(0x0341, 0x08);
	/************ROI size setting**********/
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x00);
	write_cmos_sensor(0x0347, 0x00);
	write_cmos_sensor(0x0348, 0x12);
	write_cmos_sensor(0x0349, 0x2F);
	write_cmos_sensor(0x034A, 0x0D);
	write_cmos_sensor(0x034B, 0xA7);
	/***********mode setting************/
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x00);
	write_cmos_sensor(0x0901, 0x11);
	write_cmos_sensor(0x0902, 0x0A);
	write_cmos_sensor(0x3F4C, 0x01);
	write_cmos_sensor(0x3F4D, 0x01);
	/*************digital crop and scaling setting*********/
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x00);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x12);
	write_cmos_sensor(0x040D, 0x30);
	write_cmos_sensor(0x040E, 0x0D);
	write_cmos_sensor(0x040F, 0xA8);
	/*************output size setting******/
	//write_cmos_sensor(0x034C, 0x12);
	//write_cmos_sensor(0x034D, 0x30);
	//write_cmos_sensor(0x034E, 0x0D);
	//write_cmos_sensor(0x034F, 0xA8);
	write_cmos_sensor(0x034C, 0x12);
	write_cmos_sensor(0x034D, 0x00);
	write_cmos_sensor(0x034E, 0x0D);
	write_cmos_sensor(0x034F, 0x80);
	/************clock setting****************/
	write_cmos_sensor(0x0301, 0x06);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x02);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0x8A);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x02);
	write_cmos_sensor(0x030E, 0x00);
	write_cmos_sensor(0x030F, 0x82);
	write_cmos_sensor(0x0310, 0x01);
	write_cmos_sensor(0x0820, 0x18);
	write_cmos_sensor(0x0821, 0x60);
	write_cmos_sensor(0x0822, 0x00);
	write_cmos_sensor(0x0823, 0x00);
	/*******PDAF Setting*****************/
	write_cmos_sensor(0x3E20, 0x02);
	write_cmos_sensor(0x3E3B, 0x01);
	write_cmos_sensor(0x4434, 0x02);
	write_cmos_sensor(0x4435, 0x30);//30
	write_cmos_sensor(0x8271, 0x00);

	/*******other Setting*****************/
	write_cmos_sensor(0x0106, 0x00);
	write_cmos_sensor(0x0B00, 0x00);
	write_cmos_sensor(0x3230, 0x00);
	write_cmos_sensor(0x3C00, 0x00);
	write_cmos_sensor(0x3C01, 0x38);
	write_cmos_sensor(0x3F78, 0x01);
	write_cmos_sensor(0x3F79, 0x20);
	/*******integration Setting*****************/
	write_cmos_sensor(0x0202, 0x10);  // 0x0D 
	write_cmos_sensor(0x0203, 0x34);  // 0xF6 
	/*******gain Setting*****************/
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);


//	printk("fsd---capture_setting-end\n");

}

static void capture_setting_pdaf(kal_uint16 currefps)
{

 
	//printk("fsd---E! currefps:%d\n", currefps);
	if (currefps == 150) {//15fps
	/*************MIPI output setting************/
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x0114, 0x03);
	/*************line length PCK setting************/
	write_cmos_sensor(0x0342, 0x14);
	write_cmos_sensor(0x0343, 0x00);
	/*************frame length lines setting************/
	write_cmos_sensor(0x0340, 0x0E);
	write_cmos_sensor(0x0341, 0x08);
	/************ROI size setting**********/
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x00);
	write_cmos_sensor(0x0347, 0x00);
	write_cmos_sensor(0x0348, 0x12);
	write_cmos_sensor(0x0349, 0x2F);
	write_cmos_sensor(0x034A, 0x0D);
	write_cmos_sensor(0x034B, 0xA7);
	/***********mode setting************/
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x00);
	write_cmos_sensor(0x0901, 0x11);
	write_cmos_sensor(0x0902, 0x0A);
	write_cmos_sensor(0x3F4C, 0x01);
	write_cmos_sensor(0x3F4D, 0x01);
	/*************digital crop and scaling setting*********/
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x00);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x12);
	write_cmos_sensor(0x040D, 0x30);
	write_cmos_sensor(0x040E, 0x0D);
	write_cmos_sensor(0x040F, 0xA8);
	/*************output size setting******/
	//write_cmos_sensor(0x034C, 0x12);
	//write_cmos_sensor(0x034D, 0x30);
	//write_cmos_sensor(0x034E, 0x0D);
	//write_cmos_sensor(0x034F, 0xA8);
	write_cmos_sensor(0x034C, 0x12);
	write_cmos_sensor(0x034D, 0x00);
	write_cmos_sensor(0x034E, 0x0D);
	write_cmos_sensor(0x034F, 0x80);
	/************clock setting****************/
	write_cmos_sensor(0x0301, 0x06);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x02);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0x8A);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x02);
	write_cmos_sensor(0x030E, 0x00);
	write_cmos_sensor(0x030F, 0x41);
	write_cmos_sensor(0x0310, 0x01);
	write_cmos_sensor(0x0820, 0x18);
	write_cmos_sensor(0x0821, 0x60);
	write_cmos_sensor(0x0822, 0x00);
	write_cmos_sensor(0x0823, 0x00);
	/*******PDAF Setting*****************/
	write_cmos_sensor(0x3E20, 0x02);
	write_cmos_sensor(0x3E3B, 0x01);
	write_cmos_sensor(0x4434, 0x02);
	write_cmos_sensor(0x4435, 0x30);//30
	write_cmos_sensor(0x8271, 0x00);

	/*******other Setting*****************/
	write_cmos_sensor(0x0106, 0x00);
	write_cmos_sensor(0x0B00, 0x00);
	write_cmos_sensor(0x3230, 0x00);
	write_cmos_sensor(0x3C00, 0x00);
	write_cmos_sensor(0x3C01, 0x38);
	write_cmos_sensor(0x3F78, 0x01);
	write_cmos_sensor(0x3F79, 0x20);
	/*******integration Setting*****************/
	write_cmos_sensor(0x0202, 0x10);  // 0x0D 
	write_cmos_sensor(0x0203, 0x34);  // 0xF6 
	/*******gain Setting*****************/
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	} else { //30fps
	/*************MIPI output setting************/
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x0114, 0x03);
	/*************line length PCK setting************/
	write_cmos_sensor(0x0342, 0x14);
	write_cmos_sensor(0x0343, 0x00);
	/*************frame length lines setting************/
	write_cmos_sensor(0x0340, 0x0E);
	write_cmos_sensor(0x0341, 0x08);
	/************ROI size setting**********/
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x00);
	write_cmos_sensor(0x0347, 0x00);
	write_cmos_sensor(0x0348, 0x12);
	write_cmos_sensor(0x0349, 0x2F);
	write_cmos_sensor(0x034A, 0x0D);
	write_cmos_sensor(0x034B, 0xA7);
	/***********mode setting************/
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x00);
	write_cmos_sensor(0x0901, 0x11);
	write_cmos_sensor(0x0902, 0x0A);
	write_cmos_sensor(0x3F4C, 0x01);
	write_cmos_sensor(0x3F4D, 0x01);
	/*************digital crop and scaling setting*********/
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x00);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x12);
	write_cmos_sensor(0x040D, 0x30);
	write_cmos_sensor(0x040E, 0x0D);
	write_cmos_sensor(0x040F, 0xA8);
	/*************output size setting******/
	//write_cmos_sensor(0x034C, 0x12);
	//write_cmos_sensor(0x034D, 0x30);
	//write_cmos_sensor(0x034E, 0x0D);
	//write_cmos_sensor(0x034F, 0xA8);
	write_cmos_sensor(0x034C, 0x12);
	write_cmos_sensor(0x034D, 0x00);
	write_cmos_sensor(0x034E, 0x0D);
	write_cmos_sensor(0x034F, 0x80);
	/************clock setting****************/
	write_cmos_sensor(0x0301, 0x06);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x02);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0x8A);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x02);
	write_cmos_sensor(0x030E, 0x00);
	write_cmos_sensor(0x030F, 0x82);
	write_cmos_sensor(0x0310, 0x01);
	write_cmos_sensor(0x0820, 0x18);
	write_cmos_sensor(0x0821, 0x60);
	write_cmos_sensor(0x0822, 0x00);
	write_cmos_sensor(0x0823, 0x00);
	/*******PDAF Setting*****************/
	write_cmos_sensor(0x3E20, 0x02);
	write_cmos_sensor(0x3E3B, 0x01);
	write_cmos_sensor(0x4434, 0x02);
	write_cmos_sensor(0x4435, 0x30);//30
	write_cmos_sensor(0x8271, 0x00);

	/*******other Setting*****************/
	write_cmos_sensor(0x0106, 0x00);
	write_cmos_sensor(0x0B00, 0x00);
	write_cmos_sensor(0x3230, 0x00);
	write_cmos_sensor(0x3C00, 0x00);
	write_cmos_sensor(0x3C01, 0x38);
	write_cmos_sensor(0x3F78, 0x01);
	write_cmos_sensor(0x3F79, 0x20);
	/*******integration Setting*****************/
	write_cmos_sensor(0x0202, 0x10);  // 0x0D 
	write_cmos_sensor(0x0203, 0x34);  // 0xF6 
	/*******gain Setting*****************/
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);

	}
	
}


static void normal_video_setting(kal_uint16 currefps)
{
    #if 0
	/*************MIPI output setting************/
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x0114, 0x03);
	/*************line length PCK setting************/
	write_cmos_sensor(0x0342, 0x14);
	write_cmos_sensor(0x0343, 0x00);
	/*************frame length lines setting************/
	write_cmos_sensor(0x0340, 0x0E);
	write_cmos_sensor(0x0341, 0x08);
	/************ROI size setting**********/
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x01);
	write_cmos_sensor(0x0347, 0xBC);
	write_cmos_sensor(0x0348, 0x12);
	write_cmos_sensor(0x0349, 0x2F);
	write_cmos_sensor(0x034A, 0x0B);
	write_cmos_sensor(0x034B, 0xEB);
	/***********mode setting************/
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x00);
	write_cmos_sensor(0x0901, 0x11);
	write_cmos_sensor(0x0902, 0x0A);
	write_cmos_sensor(0x3F4C, 0x01);
	write_cmos_sensor(0x3F4D, 0x01);
	/*************digital crop and scaling setting*********/
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x00);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x12);
	write_cmos_sensor(0x040D, 0x30);
	write_cmos_sensor(0x040E, 0x0A);
	write_cmos_sensor(0x040F, 0x30);
	/*************output size setting******/
	//write_cmos_sensor(0x034C, 0x12);
	//write_cmos_sensor(0x034D, 0x30);
	//write_cmos_sensor(0x034E, 0x0A);
	//write_cmos_sensor(0x034F, 0x30);
	write_cmos_sensor(0x034C, 0x12);
	write_cmos_sensor(0x034D, 0x00);
	write_cmos_sensor(0x034E, 0x0D);
	write_cmos_sensor(0x034F, 0x80);
	/************clock setting****************/
	write_cmos_sensor(0x0301, 0x06);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x02);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0x8A);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x02);
	write_cmos_sensor(0x030E, 0x00);
	write_cmos_sensor(0x030F, 0x82);
	write_cmos_sensor(0x0310, 0x01);
	write_cmos_sensor(0x0820, 0x18);
	write_cmos_sensor(0x0821, 0x60);
	write_cmos_sensor(0x0822, 0x00);
	write_cmos_sensor(0x0823, 0x00);
	/*******PDAF Setting*****************/
	write_cmos_sensor(0x3E20, 0x02);
	write_cmos_sensor(0x3E3B, 0x01);
	write_cmos_sensor(0x4434, 0x02);
	write_cmos_sensor(0x4435, 0x30);//30
	write_cmos_sensor(0x8271, 0x00);
	/*******other Setting*****************/
	write_cmos_sensor(0x0106, 0x00);
	write_cmos_sensor(0x0B00, 0x00);
	write_cmos_sensor(0x3230, 0x00);
	write_cmos_sensor(0x3C00, 0x00);
	write_cmos_sensor(0x3C01, 0x38);
	write_cmos_sensor(0x3F78, 0x01);
	write_cmos_sensor(0x3F79, 0x20);
	/*******integration Setting*****************/
	write_cmos_sensor(0x0202, 0x0D);
	write_cmos_sensor(0x0203, 0xF6);
	/*******gain Setting*****************/
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
   #endif
   
   /*************MIPI output setting************/
	   write_cmos_sensor(0x0112, 0x0A);
	   write_cmos_sensor(0x0113, 0x0A);
	   write_cmos_sensor(0x0114, 0x03);
	   /*************line length PCK setting************/
	   write_cmos_sensor(0x0342, 0x14);
	   write_cmos_sensor(0x0343, 0x00);
	   /*************frame length lines setting************/
	   write_cmos_sensor(0x0340, 0x0E);
	   write_cmos_sensor(0x0341, 0x08);
	   /************ROI size setting**********/
	   write_cmos_sensor(0x0344, 0x00);
	   write_cmos_sensor(0x0345, 0x00);
	   write_cmos_sensor(0x0346, 0x00);
	   write_cmos_sensor(0x0347, 0x00);
	   write_cmos_sensor(0x0348, 0x12);
	   write_cmos_sensor(0x0349, 0x2F);
	   write_cmos_sensor(0x034A, 0x0D);
	   write_cmos_sensor(0x034B, 0xA7);
	   /***********mode setting************/
	   write_cmos_sensor(0x0381, 0x01);
	   write_cmos_sensor(0x0383, 0x01);
	   write_cmos_sensor(0x0385, 0x01);
	   write_cmos_sensor(0x0387, 0x01);
	   write_cmos_sensor(0x0900, 0x00);
	   write_cmos_sensor(0x0901, 0x11);
	   write_cmos_sensor(0x0902, 0x0A);
	   write_cmos_sensor(0x3F4C, 0x01);
	   write_cmos_sensor(0x3F4D, 0x01);
	   /*************digital crop and scaling setting*********/
	   write_cmos_sensor(0x0408, 0x00);
	   write_cmos_sensor(0x0409, 0x00);
	   write_cmos_sensor(0x040A, 0x00);
	   write_cmos_sensor(0x040B, 0x00);
	   write_cmos_sensor(0x040C, 0x12);
	   write_cmos_sensor(0x040D, 0x30);
	   write_cmos_sensor(0x040E, 0x0D);
	   write_cmos_sensor(0x040F, 0xA8);
	   /*************output size setting******/
	   //write_cmos_sensor(0x034C, 0x12);
	  //write_cmos_sensor(0x034D, 0x30);
	  //write_cmos_sensor(0x034E, 0x0D);
	  //write_cmos_sensor(0x034F, 0xA8);
	  write_cmos_sensor(0x034C, 0x12);
	  write_cmos_sensor(0x034D, 0x00);
	  write_cmos_sensor(0x034E, 0x0D);
	  write_cmos_sensor(0x034F, 0x80);
	   /************clock setting****************/
	   write_cmos_sensor(0x0301, 0x06);
	   write_cmos_sensor(0x0303, 0x02);
	   write_cmos_sensor(0x0305, 0x02);
	   write_cmos_sensor(0x0306, 0x00);
	   write_cmos_sensor(0x0307, 0x8A);
	   write_cmos_sensor(0x030B, 0x01);
	   write_cmos_sensor(0x030D, 0x02);
	   write_cmos_sensor(0x030E, 0x00);
	   write_cmos_sensor(0x030F, 0x82);
	   write_cmos_sensor(0x0310, 0x01);
	   write_cmos_sensor(0x0820, 0x18);
	   write_cmos_sensor(0x0821, 0x60);
	   write_cmos_sensor(0x0822, 0x00);
	   write_cmos_sensor(0x0823, 0x00);
	   /*******PDAF Setting*****************/
	   write_cmos_sensor(0x3E20, 0x02);
	   write_cmos_sensor(0x3E3B, 0x01);
	   write_cmos_sensor(0x4434, 0x02);
	   write_cmos_sensor(0x4435, 0x30);//30
	   write_cmos_sensor(0x8271, 0x00);
   
	   /*******other Setting*****************/
	   write_cmos_sensor(0x0106, 0x00);
	   write_cmos_sensor(0x0B00, 0x00);
	   write_cmos_sensor(0x3230, 0x00);
	   write_cmos_sensor(0x3C00, 0x00);
	   write_cmos_sensor(0x3C01, 0x38);
	   write_cmos_sensor(0x3F78, 0x01);
	   write_cmos_sensor(0x3F79, 0x20);
	   /*******integration Setting*****************/
	   write_cmos_sensor(0x0202, 0x10);  // 0x0D 
	   write_cmos_sensor(0x0203, 0x34);  // 0xF6 
	   /*******gain Setting*****************/
	   write_cmos_sensor(0x0204, 0x00);
	   write_cmos_sensor(0x0205, 0x00);
	   write_cmos_sensor(0x020E, 0x01);
	   write_cmos_sensor(0x020F, 0x00);
}

static void hs_video_setting(void)
{
	/*************MIPI output setting************/
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x0114, 0x03);
	/*************line length PCK setting************/
	write_cmos_sensor(0x0342, 0x0A);
	write_cmos_sensor(0x0343, 0x00);
	/*************frame length lines setting************/
	write_cmos_sensor(0x0340, 0x07);
	write_cmos_sensor(0x0341, 0x60);
	/************ROI size setting**********/
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x02);
	write_cmos_sensor(0x0347, 0x98);
	write_cmos_sensor(0x0348, 0x12);
	write_cmos_sensor(0x0349, 0x2F);
	write_cmos_sensor(0x034A, 0x0B);
	write_cmos_sensor(0x034B, 0x07);
	/***********mode setting************/
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x01);
	write_cmos_sensor(0x0901, 0x22);
	write_cmos_sensor(0x0902, 0x0B);
	write_cmos_sensor(0x3F4C, 0x01);
	write_cmos_sensor(0x3F4D, 0x01);
	/*************digital crop and scaling setting*********/
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0xCC);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x07);
	write_cmos_sensor(0x040D, 0x80);
	write_cmos_sensor(0x040E, 0x04);
	write_cmos_sensor(0x040F, 0x38);
	/*************output size setting******/
	write_cmos_sensor(0x034C, 0x07);
	write_cmos_sensor(0x034D, 0x80);
	write_cmos_sensor(0x034E, 0x04);
	write_cmos_sensor(0x034F, 0x38);
	/************clock setting****************/
	write_cmos_sensor(0x0301, 0x06);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x02);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0x91);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x02);
	write_cmos_sensor(0x030E, 0x00);
	write_cmos_sensor(0x030F, 0x72);
	write_cmos_sensor(0x0310, 0x01);
	write_cmos_sensor(0x0820, 0x15);
	write_cmos_sensor(0x0821, 0x60);
	write_cmos_sensor(0x0822, 0x00);
	write_cmos_sensor(0x0823, 0x00);
	/*******PDAF Setting*****************/
	write_cmos_sensor(0x3E20, 0x02);
	write_cmos_sensor(0x3E3B, 0x00);
	write_cmos_sensor(0x4434, 0x00);
	write_cmos_sensor(0x4435, 0x00);
	write_cmos_sensor(0x8271, 0x00);
	/*******other Setting*****************/
	write_cmos_sensor(0x0106, 0x00);
	write_cmos_sensor(0x0B00, 0x00);
	write_cmos_sensor(0x3230, 0x00);
	write_cmos_sensor(0x3C00, 0x00);
	write_cmos_sensor(0x3C01, 0x88);
	write_cmos_sensor(0x3F78, 0x01);
	write_cmos_sensor(0x3F79, 0x1E);
	/*******integration Setting*****************/
	write_cmos_sensor(0x0202, 0x07);
	write_cmos_sensor(0x0203, 0x4E);
	/*******gain Setting*****************/
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);

}

static void slim_video_setting(void)
{

	/*************MIPI output setting************/
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x0114, 0x03);
	/*************line length PCK setting************/
	write_cmos_sensor(0x0342, 0x14);
	write_cmos_sensor(0x0343, 0x00);
	/*************frame length lines setting************/
	write_cmos_sensor(0x0340, 0x03);
	write_cmos_sensor(0x0341, 0x0C);
	/************ROI size setting**********/
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x04);
	write_cmos_sensor(0x0347, 0x04);
	write_cmos_sensor(0x0348, 0x12);
	write_cmos_sensor(0x0349, 0x2F);
	write_cmos_sensor(0x034A, 0x09);
	write_cmos_sensor(0x034B, 0xA3);
	/***********mode setting************/
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x01);
	write_cmos_sensor(0x0901, 0x22);
	write_cmos_sensor(0x0902, 0x0B);
	write_cmos_sensor(0x3F4C, 0x01);
	write_cmos_sensor(0x3F4D, 0x03);
	/*************digital crop and scaling setting*********/
	write_cmos_sensor(0x0408, 0x02);
	write_cmos_sensor(0x0409, 0x0C);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x05);
	write_cmos_sensor(0x040D, 0x00);
	write_cmos_sensor(0x040E, 0x02);
	write_cmos_sensor(0x040F, 0xD0);
	/*************output size setting******/
	write_cmos_sensor(0x034C, 0x05);
	write_cmos_sensor(0x034D, 0x00);
	write_cmos_sensor(0x034E, 0x02);
	write_cmos_sensor(0x034F, 0xD0);
	/************clock setting****************/
	write_cmos_sensor(0x0301, 0x06);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x02);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0x3C);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x02);
	write_cmos_sensor(0x030E, 0x00);
	write_cmos_sensor(0x030F, 0x1D);
	write_cmos_sensor(0x0310, 0x01);
	write_cmos_sensor(0x0820, 0x05);
	write_cmos_sensor(0x0821, 0x70);
	write_cmos_sensor(0x0822, 0x00);
	write_cmos_sensor(0x0823, 0x00);
	/*******PDAF Setting*****************/
	write_cmos_sensor(0x3E20, 0x02);
	write_cmos_sensor(0x3E3B, 0x00);
	write_cmos_sensor(0x4434, 0x00);
	write_cmos_sensor(0x4435, 0x00);
	write_cmos_sensor(0x8271, 0x00);
	/*******other Setting*****************/
	write_cmos_sensor(0x0106, 0x00);
	write_cmos_sensor(0x0B00, 0x00);
	write_cmos_sensor(0x3230, 0x00);
	write_cmos_sensor(0x3C00, 0x00);
	write_cmos_sensor(0x3C01, 0x8C);
	write_cmos_sensor(0x3F78, 0x01);
	write_cmos_sensor(0x3F79, 0x60);
	/*******integration Setting*****************/
	write_cmos_sensor(0x0202, 0x02);
	write_cmos_sensor(0x0203, 0xFA);
	/*******gain Setting*****************/
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);

}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);
	if (enable)
		write_cmos_sensor(0x0601, 0x02);
	else
		write_cmos_sensor(0x0601, 0x00);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 *    get_imgsensor_id
 *
 * DESCRIPTION
 *    This function get the sensor ID
 *
 * PARAMETERS
 *    *sensorID : return the sensor ID
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
				load_imx499_LRC_Data();
				
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n",
				imgsensor.i2c_write_id, *sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
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
 *    open
 *
 * DESCRIPTION
 *    This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *    None
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
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
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n",
				imgsensor.i2c_write_id, sensor_id);
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
	imgsensor.ihdr_mode = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}				/*    open  */



/*************************************************************************
 * FUNCTION
 *    close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *    None
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function */

	return ERROR_NONE;
}				/*    close  */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *    This function start the sensor preview.
 *
 * PARAMETERS
 *    *image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
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
	
    preview_setting();	/*PDAF only */
	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */
	return ERROR_NONE;
}				/*    preview   */

/*************************************************************************
 * FUNCTION
 *    capture
 *
 * DESCRIPTION
 *    This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *    None
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
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
		/* PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M */
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
				imgsensor.current_fps, imgsensor_info.cap.max_framerate / 10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	if (imgsensor.pdaf_mode == 1)
		capture_setting_pdaf(imgsensor.current_fps);	/*PDAF only */
	else
		capture_setting(imgsensor.current_fps);	/*Full mode */

	return ERROR_NONE;
}				/* capture() */

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
	/* zhengjiang.zhu@Koobee.Camera.Driver  2018/06/05 add for video pdaf*/
	if (imgsensor.pdaf_mode == 1)
		capture_setting_pdaf(imgsensor.current_fps);	/*PDAF only */
	else
	normal_video_setting(imgsensor.current_fps);
	/* zhengjiang.zhu@Koobee.Camera.Driver  2018/06/05 end for video pdaf*/
	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */
	return ERROR_NONE;
}				/*    normal_video   */

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
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */
	return ERROR_NONE;
}				/*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			     MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */

	return ERROR_NONE;
}				/*    slim_video     */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}				/*    get_resolution    */

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);


	/* sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; // not use */
	/* sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; // not use */
	/* imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; // not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;	/* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;	/* inverse with datasheet */
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;	/* not use */
	sensor_info->SensorResetActiveHigh = FALSE;	/* not use */
	sensor_info->SensorResetDelayCount = 5;	/* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0;	/* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;	/* The frame of setting shutter
										 * default 0 for TG int
										 */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting
												 * sensor gain
												 */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = PDAF_SUPPORT_CAMSV;//PDAF_SUPPORT_RAW;	/*0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode */

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;	/* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;	/* not use */
	sensor_info->SensorPixelClockCount = 3;	/* not use */
	sensor_info->SensorDataLatchCount = 2;	/* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

		sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}				/*    get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
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
}				/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{				/* This Function not used after ROME */
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
	if (enable)		/* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else			/* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id,
						MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk / framerate * 10 /
			imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength)
			? (frame_length - imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength)
				? (frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
				LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
					framerate, imgsensor_info.cap.max_framerate / 10);
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength)
				? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength)
			? (frame_length - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength)
			? (frame_length - imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
			set_dummy();
		break;
	default:		/* coding with  preview scenario by default */
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
			set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id,
						    MUINT32 *framerate)
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


static kal_uint32 imx499_awb_gain(SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
	UINT32 rgain_32, grgain_32, gbgain_32, bgain_32;

	LOG_INF("imx499_awb_gain\n");
	grgain_32 = (pSetSensorAWB->ABS_GAIN_GR << 8) >> 9;
	rgain_32 = (pSetSensorAWB->ABS_GAIN_R << 8) >> 9;
	bgain_32 = (pSetSensorAWB->ABS_GAIN_B << 8) >> 9;
	gbgain_32 = (pSetSensorAWB->ABS_GAIN_GB << 8) >> 9;

	LOG_INF("[imx499_awb_gain] ABS_GAIN_GR:%d, grgain_32:%d\n", pSetSensorAWB->ABS_GAIN_GR,
		grgain_32);
	LOG_INF("[imx499_awb_gain] ABS_GAIN_R:%d, rgain_32:%d\n", pSetSensorAWB->ABS_GAIN_R,
		rgain_32);
	LOG_INF("[imx499_awb_gain] ABS_GAIN_B:%d, bgain_32:%d\n", pSetSensorAWB->ABS_GAIN_B,
		bgain_32);
	LOG_INF("[imx499_awb_gain] ABS_GAIN_GB:%d, gbgain_32:%d\n", pSetSensorAWB->ABS_GAIN_GB,
		gbgain_32);

	write_cmos_sensor(0x0b8e, (grgain_32 >> 8) & 0xFF);
	write_cmos_sensor(0x0b8f, grgain_32 & 0xFF);
	write_cmos_sensor(0x0b90, (rgain_32 >> 8) & 0xFF);
	write_cmos_sensor(0x0b91, rgain_32 & 0xFF);
	write_cmos_sensor(0x0b92, (bgain_32 >> 8) & 0xFF);
	write_cmos_sensor(0x0b93, bgain_32 & 0xFF);
	write_cmos_sensor(0x0b94, (gbgain_32 >> 8) & 0xFF);
	write_cmos_sensor(0x0b95, gbgain_32 & 0xFF);
	return ERROR_NONE;
}

static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable)
		write_cmos_sensor(0x0100, 0X01);
	else
		write_cmos_sensor(0x0100, 0x00);
	mdelay(10);
	return ERROR_NONE;
}


static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	/* unsigned long long *feature_return_data = (unsigned long long*)feature_para; */

	SET_PD_BLOCK_INFO_T *PDAFinfo;

	SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	SENSOR_VC_INFO_STRUCT *pvcinfo;
	SET_SENSOR_AWB_GAIN *pSetSensorAWB = (SET_SENSOR_AWB_GAIN *) feature_para;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

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
		night_mode((BOOL) *feature_data);
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
		set_video_mode(*feature_data_16);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL) *feature_data_16, *(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM) *feature_data,
					      *(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM) *feature_data,
						  (MUINT32 *) (uintptr_t) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL) *feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:	/* for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32) *feature_data);
		wininfo = (SENSOR_WINSIZE_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%llu\n", *feature_data);
		PDAFinfo = (SET_PD_BLOCK_INFO_T *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)PDAFinfo, (void *)&pd_block_info_cap,
			       sizeof(SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)PDAFinfo, (void *)&pd_block_info_normal_video,
			       sizeof(SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			break;
		}
		break;
		/*HDR CMD */
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_mode = (UINT8)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n", (UINT16) *feature_data,
			(UINT16) *(feature_data + 1), (UINT16) *(feature_data + 2));
		ihdr_write_shutter_gain(*feature_data, *(feature_data + 1), *(feature_data + 2));
		break;

	case SENSOR_FEATURE_GET_VC_INFO:
		LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16) *feature_data);
		pvcinfo = (SENSOR_VC_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],
			       sizeof(SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2],
			       sizeof(SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
			       sizeof(SENSOR_VC_INFO_STRUCT));
			break;
		}
		break;
 
	case SENSOR_FEATURE_SET_AWB_GAIN:
		imx499_awb_gain(pSetSensorAWB);
		break;
		/*END OF HDR CMD */
		/*PDAF CMD */
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%llu\n", *feature_data);
		/* PDAF capacity enable or not, 2p8 only full size support PDAF */
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 1;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 1;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		default:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_SET_PDAF:
		LOG_INF("PDAF mode :%d\n", *feature_data_16);
		imgsensor.pdaf_mode = *feature_data_16;
		break;
		/*End of PDAF */
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	default:
		break;
	}

	return ERROR_NONE;
}				/*    feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 IMX499_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}				/*    imx499_MIPI_RAW_SensorInit    */

 /*
 *
 * Filename:
 * ---------
 *     gc13023mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 * Driver Version:
 * ------------
 *     V0.6062.040
 *
 *-----------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 */

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "gc13023mipi_Sensor.h"

/************************** Modify Following Strings for Debug **************************/
#define PFX "GC13023MIPI"
#define LOG_1 LOG_INF("GC13023MIPI, 4LANE\n")
/****************************   Modify end    *******************************************/
#define GC13023_DEBUG    0
#if GC13023_DEBUG
#define LOG_INF(format, args...)    pr_err(PFX "[%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

static DEFINE_SPINLOCK(imgsensor_drv_lock);
static kal_uint8  BorF = 0;

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = GC13023_SENSOR_ID,
	.checksum_value = 0xf7375923,
#if BINNING_PREVIEW
	.pre = {
		.pclk = 100800000,
		.linelength = 4296,
		.framelength = 793,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2104,
		.grabwindow_height = 1560,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
#else
	.pre = {
		.pclk = 201600000,
		.linelength = 4296,
		.framelength = 1586,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
#endif
	.cap = {
		.pclk = 201600000,
		.linelength = 4296,
		.framelength = 1586,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.cap1 = {
		.pclk = 201600000,
		.linelength = 4296,
		.framelength = 1586,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.normal_video = {
		.pclk = 201600000,
		.linelength = 4296,
		.framelength = 1586,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 201600000,
		.linelength = 4296,
		.framelength = 1586,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.slim_video = {
		.pclk = 201600000,
		.linelength = 4296,
		.framelength = 1586,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},

	.margin = 16,
	.min_shutter = 1,
	.max_frame_length = 0x24aa,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,
	.ihdr_le_firstline = 0,
	.sensor_mode_num = 3,

	.cap_delay_frame = 2,
	.pre_delay_frame = 2,
	.video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
#if GC13023_MIRROR_FLIP_ENABLE
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,
#else
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
#endif
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x6e, 0x7e, 0xff},
};

static struct imgsensor_struct imgsensor = {
#if GC13023_MIRROR_FLIP_ENABLE
	.mirror = IMAGE_V_MIRROR,
#else
	.mirror = IMAGE_H_MIRROR,
#endif
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3ED,
	.gain = 0x100,
	.dummy_pixel = 0,
	.dummy_line = 0,
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = 0,
	.i2c_write_id = 0x6e,
};

/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
#if BINNING_PREVIEW
	{ 4208, 3120, 0, 0, 4208, 3120, 2104, 1560, 0, 0, 2104, 1560, 0, 0, 2104, 1560}, /* Preview */
#else
	{ 4208, 3120, 0, 0, 4208, 3120, 4208, 3120, 0, 0, 4208, 3120, 0, 0, 4208, 3120}, /* Preview */
#endif
	{ 4208, 3120, 0, 0, 4208, 3120, 4208, 3120, 0, 0, 4208, 3120, 0, 0, 4208, 3120}, /* capture */
	{ 4208, 3120, 0, 0, 4208, 3120, 4208, 3120, 0, 0, 4208, 3120, 0, 0, 4208, 3120}, /* video */
	{ 4208, 3120, 0, 0, 4208, 3120, 4208, 3120, 0, 0, 4208, 3120, 0, 0, 4208, 3120}, /* hs video */
	{ 4208, 3120, 0, 0, 4208, 3120, 4208, 3120, 0, 0, 4208, 3120, 0, 0, 4208, 3120}  /* slim video */
};

#if PDAFSUPPORT
static SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
#if GC13023_MIRROR_FLIP_ENABLE
	.i4OffsetX = 24,
	.i4OffsetY = 20,
	.i4PitchX = 64,
	.i4PitchY = 64,
	.i4PairNum = 16,
	.i4SubBlkW = 16,
	.i4SubBlkH = 16,
	.i4BlockNumX = 65,
	.i4BlockNumY = 48,
	.i4PosR = {
		{31, 24},
		{83, 24},
		{47, 28},
		{67, 28},
		{35, 44},
		{79, 44},
		{51, 48},
		{63, 48},
		{51, 56},
		{63, 56},
		{35, 60},
		{79, 60},
		{47, 76},
		{67, 76},
		{31, 80},
		{83, 80}
	},
	.i4PosL = {
		{31, 28},
		{83, 28},
		{47, 32},
		{67, 32},
		{35, 40},
		{79, 40},
		{51, 44},
		{63, 44},
		{51, 60},
		{63, 60},
		{35, 64},
		{79, 64},
		{47, 72},
		{67, 72},
		{31, 76},
		{83, 76}
	},
	.iMirrorFlip = 0,	/* 0:Normal; 1:H; 2:V; 3HV (imgsensor.mirror - Normal)*/
#else
	.i4OffsetX = 24,
	.i4OffsetY = 28,
	.i4PitchX  = 64,
	.i4PitchY  = 64,
	.i4PairNum = 16,
	.i4SubBlkW = 16,
	.i4SubBlkH = 16,
	.i4BlockNumX = 65,
	.i4BlockNumY = 48,
	.i4PosR = {
		{28, 35},
		{80, 35},
		{44, 39},
		{64, 39},
		{32, 47},
		{76, 47},
		{48, 51},
		{60, 51},
		{48, 67},
		{60, 67},
		{32, 71},
		{76, 71},
		{44, 79},
		{64, 79},
		{28, 83},
		{80, 83}
	},
	.i4PosL = {
		{28, 31},
		{80, 31},
		{44, 35},
		{64, 35},
		{32, 51},
		{76, 51},
		{48, 55},
		{60, 55},
		{48, 63},
		{60, 63},
		{32, 67},
		{76, 67},
		{44, 83},
		{64, 83},
		{28, 87},
		{80, 87}
	},
	.iMirrorFlip = 0,	/* 0:Normal; 1:H; 2:V; 3HV (imgsensor.mirror - Normal)*/

#endif
};
#endif

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[1] = { (char)(addr & 0xff) };

	iReadRegI2C(pu_send_cmd, 1, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[2] = { (char)(addr & 0xff), (char)(para & 0xff) };

	iWriteRegI2C(pu_send_cmd, 2, imgsensor.i2c_write_id);
}

static struct gc13023_otp gc13023_otp_info = {0};

static kal_uint8 gc13023_read_otp(kal_uint32 addr)
{
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x69, (addr >> 5) & 0x3f);
	write_cmos_sensor(0x6a, (addr << 3) & 0xff);
	mdelay(1);
	write_cmos_sensor(0xf3, 0x20);

	return  read_cmos_sensor(0x6c);
}

static void gc13023_gcore_read_otp_info(void)
{
	kal_uint16 i = 0, j = 0;
	kal_uint8  temp = 0;
	kal_uint8  flag_dd = 0;
	kal_uint8  flag_version = 0;

	memset(&gc13023_otp_info, 0, sizeof(gc13023_otp_info));

	/* PDAF & Package */
	gc13023_otp_info.pdaf_flag = gc13023_read_otp(0x00) & 0x30;
	gc13023_otp_info.package_flag = gc13023_read_otp(0x00) & 0x0f;
	LOG_INF("pdaf_flag = 0x%x, package_flag = 0x%x!\n",
		gc13023_otp_info.pdaf_flag, gc13023_otp_info.package_flag);

	switch (gc13023_otp_info.package_flag) {
	case 0x01:
		LOG_INF("COB!\n");
		break;
	case 0x02:
	case 0x03:
		LOG_INF("COM!\n");
		break;
	case 0x04:
	case 0x05:
		LOG_INF("TPLCC!\n");
		break;
	case 0x08:
	case 0x09:
		LOG_INF("CSP!\n");
		break;
	default:
		LOG_INF("Package Unkown!\n");
		break;
	}

	/* Sensor & Drive Version */
	flag_version = gc13023_read_otp(0x01);
	LOG_INF("flag_version = 0x%x\n", flag_version);
	switch (flag_version & 0x03) {
	case 0x00:
		LOG_INF("Version information is Empty!\n");
		gc13023_otp_info.version_flag = 0x00;
		break;
	case 0x01:
		LOG_INF("Version information is Valid!\n");
		gc13023_otp_info.sensor_version = gc13023_read_otp(0x02) & 0xf0;
		gc13023_otp_info.driver_version = gc13023_read_otp(0x02) & 0x0f;
		gc13023_otp_info.version_flag = 0x01;
		break;
	case 0x02:
	case 0x03:
		LOG_INF("Version information is Invalid!\n");
		gc13023_otp_info.version_flag = 0x02;
		break;
	default:
		break;
	}

	/* DPC */
	flag_dd = gc13023_read_otp(0x0d);
	LOG_INF("flag_dd = 0x%x\n", flag_dd);
	switch (flag_dd & 0x03) {
	case 0x00:
		LOG_INF("DD is Empty!\n");
		gc13023_otp_info.dd_flag = 0x00;
		break;
	case 0x01:
		LOG_INF("DD is Valid!\n");
		gc13023_otp_info.dd_cnt = gc13023_read_otp(0x0e) + gc13023_read_otp(0x0f);
		gc13023_otp_info.dd_flag = 0x01;
		LOG_INF("total_number = %d\n", gc13023_otp_info.dd_cnt);
		break;
	case 0x02:
	case 0x03:
		LOG_INF("DD is Invalid!\n");
		gc13023_otp_info.dd_flag = 0x02;
		break;
	default:
		break;
	}

	/* sensor registers */
	gc13023_otp_info.reg_flag = gc13023_read_otp(0x0150);

	if ((gc13023_otp_info.reg_flag & 0x03) == 1)
		for (i = 0; i < 5; i++) {
			temp = gc13023_read_otp(0x0151 + 5 * i);
			for (j = 0; j < 2; j++)
				if (((temp >> (4 * j + 3)) & 0x01) == 0x01) {
					gc13023_otp_info.reg_page[gc13023_otp_info.reg_num] = (temp >> (4 * j)) & 0x03;
					gc13023_otp_info.reg_addr[gc13023_otp_info.reg_num] =
						gc13023_read_otp(0x152 + 5 * i + 2 * j);
					gc13023_otp_info.reg_value[gc13023_otp_info.reg_num] =
						gc13023_read_otp(0x152 + 5 * i + 2 * j + 1);
					gc13023_otp_info.reg_num++;
				}
		}
}

static void gc13023_gcore_update_dd(void)
{
	kal_uint8 state = 0;
	kal_uint8 n = 0;

	if (gc13023_otp_info.dd_flag == 0x01) {
		LOG_INF("AUTO Load DD start!\n");
		write_cmos_sensor(0xfe, 0x01);
		write_cmos_sensor(0x60, 0x1c);
		write_cmos_sensor(0x63, 0x00);
		write_cmos_sensor(0x68, 0xf2);
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x6f, 0x2e);
		write_cmos_sensor(0xfe, 0x01);
		write_cmos_sensor(0x02, gc13023_otp_info.dd_cnt);
		write_cmos_sensor(0x05, 0x00);
		write_cmos_sensor(0x06, 0x80);
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x6e, 0x01);
		write_cmos_sensor(0xfe, 0x01);
		write_cmos_sensor(0x08, 0x10);
		write_cmos_sensor(0x09, 0x7e);
		write_cmos_sensor(0x0a, 0x0c);
		write_cmos_sensor(0x0b, 0x40);
		write_cmos_sensor(0xbe, 0x00);
		write_cmos_sensor(0xc0, 0x20);
		write_cmos_sensor(0xa9, 0x01);
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0xf2, 0x48);

		while (n < 3) {
			state = read_cmos_sensor(0x6e);
			if ((state | 0xef) == 0xff)
				mdelay(10);
			else
				n = 3;
			n++;
		}

		write_cmos_sensor(0xfe, 0x01);
		write_cmos_sensor(0xbe, 0x01);
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x6f, 0x00);
	}
}

static void gc13023_gcore_update_pdinfo(void)
{
#if PDAFSUPPORT
	if (gc13023_otp_info.pdaf_flag == 0x20)
		LOG_INF("This sensor supports PDAF!\n");
	else
		LOG_INF("PDAF flag error!\n");

#else
#if GC13023_MIRROR_FLIP_ENABLE
		write_cmos_sensor(0xfe, 0x01);
		write_cmos_sensor(0x90, 0x71);
		write_cmos_sensor(0x91, 0x1c);
		write_cmos_sensor(0x92, 0x00);
		write_cmos_sensor(0x93, 0x10);
		write_cmos_sensor(0x94, 0x18);
		write_cmos_sensor(0x95, 0x8c);
		write_cmos_sensor(0x96, 0x30);
		write_cmos_sensor(0xfe, 0x00);
#else
		write_cmos_sensor(0xfe, 0x01);
		write_cmos_sensor(0x90, 0x71);
		write_cmos_sensor(0x91, 0x24);
		write_cmos_sensor(0x92, 0x00);
		write_cmos_sensor(0x93, 0x10);
		write_cmos_sensor(0x94, 0x20);
		write_cmos_sensor(0x95, 0x8c);
		write_cmos_sensor(0x96, 0x30);
		write_cmos_sensor(0xfe, 0x00);
#endif
#endif
}

static void gc13023_gcore_update_register(void)
{
	kal_uint8 i = 0;

	LOG_INF("reg_num = %d\n", gc13023_otp_info.reg_num);

	if (gc13023_otp_info.reg_flag)
		for (i = 0; i < gc13023_otp_info.reg_num; i++) {
			write_cmos_sensor(0xfe, gc13023_otp_info.reg_page[i]);
			write_cmos_sensor(gc13023_otp_info.reg_addr[i], gc13023_otp_info.reg_value[i]);
			LOG_INF("P%d:0x%x -> 0x%x\n", gc13023_otp_info.reg_page[i],
				gc13023_otp_info.reg_addr[i], gc13023_otp_info.reg_value[i]);
		}
}

static void gc13023_gcore_update_otp(void)
{
	gc13023_gcore_update_dd();
	gc13023_gcore_update_pdinfo();
	gc13023_gcore_update_register();
}

static void gc13023_gcore_enable_otp(kal_uint8 state)
{
	if (state) {
		write_cmos_sensor(0xf6, 0xc4);/*[7]otp_clk*/
		write_cmos_sensor(0xfe, 0x01);
		write_cmos_sensor(0x67, 0x80);/*[7]otp_en*/
		write_cmos_sensor(0x60, 0x1c);
		write_cmos_sensor(0x61, 0x00);
		write_cmos_sensor(0x62, 0x00);
		write_cmos_sensor(0x63, 0x00);
		write_cmos_sensor(0xf2, 0x08);
		write_cmos_sensor(0x68, 0xf2);
		write_cmos_sensor(0xfe, 0x00);
		LOG_INF("Enable OTP!\n");
	} else {
		write_cmos_sensor(0xfe, 0x01);
		write_cmos_sensor(0x67, 0x00);
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0xf6, 0x44);
		LOG_INF("Disable OTP!\n");
	}
}

static void gc13023_gcore_identify_otp(void)
{
	gc13023_gcore_enable_otp(otp_open);
	gc13023_gcore_read_otp_info();
	gc13023_gcore_update_otp();
	gc13023_gcore_enable_otp(otp_close);
}

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor(0xf0) << 8) | read_cmos_sensor(0xf1));
}

static void set_dummy(void)
{
	kal_uint32 vb = 16;
	kal_uint32 basic_line = 3156;

	LOG_INF("frame_length = %d\n", imgsensor.frame_length);

	vb = (imgsensor.frame_length << (2 - BorF)) - basic_line;

	vb = (vb < 16) ? 16 : vb;
	vb = (vb > 16383) ? 16383 : vb;
	LOG_INF("BorF = %d(0:B 1:F), vb = %d\n", BorF, vb);

	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x07, (vb >> 8) & 0x3f);
	write_cmos_sensor(0x08, vb & 0xff);
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;

	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ?
		frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);

	set_dummy();
}

static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

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
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
		(imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
	LOG_INF("realtime_fps = %d\n", realtime_fps);

	write_cmos_sensor(0xfe, 0x00);
	if (imgsensor.autoflicker_en) {

		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else
			set_max_framerate(realtime_fps, 0);
	} else {
		set_max_framerate(realtime_fps, 0);
	}

	write_cmos_sensor(0x03, (shutter >> 8) & 0x7f);
	write_cmos_sensor(0x04, shutter & 0xff);

	LOG_INF("shutter = %d, framelength = %d\n", shutter, imgsensor.frame_length);
}

static void gain2reg(kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x100;
	kal_int16 gain_index = 0, temp_gain = 0;

	kal_uint16 gain_level[MAX_AG_INDEX] = {
		0x0100, /* 1.00*/
		0x016c, /* 1.42*/
		0x01f6, /* 1.96*/
		0x02d4, /* 2.84*/
		0x03fd, /* 3.99*/
		0x05a9, /* 5.66*/
		0x080f, /* 8.06*/
		0x0b80, /*11.50*/
		0x10b8, /*16.72*/
	};

	kal_uint8 agc_register[2 * MAX_AG_INDEX][AGC_REG_NUM] = {
		/*{ 0xfe, 0x3b, 0x20, 0x3a, 0xfe, 0xdf, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xfe },*/
		/* binning */
		{0x00, 0x06, 0xd4, 0x8d, 0x01, 0x00, 0x06, 0x1a, 0x0a, 0x16, 0x50, 0x60, 0xa0, 0xc0, 0x00},
		{0x00, 0x06, 0xd0, 0x90, 0x01, 0x00, 0x0e, 0x1e, 0x08, 0x14, 0x4e, 0x62, 0xa0, 0xd0, 0x00},
		{0x00, 0x06, 0xd0, 0x91, 0x01, 0x00, 0x0e, 0x1c, 0x08, 0x14, 0x4e, 0x62, 0x90, 0xd0, 0x00},
		{0x00, 0x06, 0xd4, 0x92, 0x01, 0x00, 0x08, 0x2a, 0x08, 0x1c, 0x58, 0x62, 0x90, 0xd0, 0x00},
		{0x00, 0x06, 0xd9, 0x96, 0x01, 0x00, 0x08, 0x2a, 0x08, 0x1c, 0x5c, 0x62, 0x90, 0xd0, 0x00},
		{0x00, 0x06, 0xda, 0x98, 0x01, 0x00, 0x0c, 0x36, 0x0a, 0x24, 0x5c, 0x64, 0x94, 0xd6, 0x00},
		{0x00, 0x06, 0xda, 0x9e, 0x01, 0x00, 0x0e, 0x3e, 0x0a, 0x38, 0x5c, 0x62, 0x98, 0xda, 0x00},
		{0x00, 0x06, 0xda, 0x9e, 0x01, 0x00, 0x0f, 0x3e, 0x0a, 0x38, 0x5e, 0x62, 0x98, 0xda, 0x00},
		{0x00, 0x06, 0xda, 0x9e, 0x01, 0x00, 0x10, 0x3e, 0x0a, 0x38, 0x5c, 0x62, 0x98, 0xda, 0x00},
		/* fullsize */
		{0x00, 0x06, 0xda, 0x95, 0x01, 0x00, 0x08, 0x20, 0x04, 0x1c, 0x50, 0x60, 0xa0, 0xc4, 0x00},
		{0x00, 0x06, 0xda, 0x95, 0x01, 0x00, 0x08, 0x24, 0x08, 0x1c, 0x50, 0x60, 0xa0, 0xc4, 0x00},
		{0x00, 0x06, 0xdb, 0x94, 0x01, 0x00, 0x08, 0x28, 0x08, 0x20, 0x50, 0x60, 0xa0, 0xc4, 0x00},
		{0x00, 0x06, 0xdc, 0x97, 0x01, 0x00, 0x0a, 0x2e, 0x08, 0x20, 0x56, 0x60, 0xa0, 0xc4, 0x00},
		{0x00, 0x06, 0xe0, 0x9f, 0x01, 0x00, 0x0a, 0x38, 0x08, 0x20, 0x58, 0x61, 0xa4, 0xc4, 0x00},
		{0x00, 0x06, 0xe0, 0x9f, 0x01, 0x00, 0x16, 0x3e, 0x10, 0x26, 0x58, 0x62, 0xa4, 0xc4, 0x00},
		{0x00, 0x06, 0xd6, 0x97, 0x01, 0x00, 0x2a, 0x3f, 0x16, 0x30, 0x52, 0x66, 0xa0, 0xc4, 0x00},
		{0x00, 0x06, 0xd7, 0x9a, 0x01, 0x00, 0x28, 0x3e, 0x16, 0x30, 0x56, 0x62, 0xa0, 0xc4, 0x00},
		{0x00, 0x06, 0xe0, 0x9f, 0x01, 0x00, 0x0a, 0x38, 0x08, 0x20, 0x58, 0x61, 0xa4, 0xc4, 0x00}
	};

	reg_gain = gain << 2;

	reg_gain = (reg_gain < SENSOR_BASE_GAIN) ? SENSOR_BASE_GAIN : reg_gain;
	reg_gain = (reg_gain > SENSOR_MAX_GAIN) ? SENSOR_MAX_GAIN : reg_gain;

	for (gain_index = MEAG_INDEX - 1; gain_index >= 0; gain_index--)
		if (reg_gain >= gain_level[gain_index]) {
			write_cmos_sensor(0xb6, gain_index);
			temp_gain = 256 * reg_gain / gain_level[gain_index];
			write_cmos_sensor(0xb1, temp_gain >> 8);
			write_cmos_sensor(0xb2, temp_gain & 0xff);

			write_cmos_sensor(0xfe, agc_register[BorF * MAX_AG_INDEX + gain_index][0]);
			write_cmos_sensor(0x3b, agc_register[BorF * MAX_AG_INDEX + gain_index][1]);
			write_cmos_sensor(0x20, agc_register[BorF * MAX_AG_INDEX + gain_index][2]);
			write_cmos_sensor(0x3a, agc_register[BorF * MAX_AG_INDEX + gain_index][3]);
			write_cmos_sensor(0xfe, agc_register[BorF * MAX_AG_INDEX + gain_index][4]);
			write_cmos_sensor(0xdf, agc_register[BorF * MAX_AG_INDEX + gain_index][5]);
			write_cmos_sensor(0xe7, agc_register[BorF * MAX_AG_INDEX + gain_index][6]);
			write_cmos_sensor(0xe8, agc_register[BorF * MAX_AG_INDEX + gain_index][7]);
			write_cmos_sensor(0xe9, agc_register[BorF * MAX_AG_INDEX + gain_index][8]);
			write_cmos_sensor(0xea, agc_register[BorF * MAX_AG_INDEX + gain_index][9]);
			write_cmos_sensor(0xeb, agc_register[BorF * MAX_AG_INDEX + gain_index][10]);
			write_cmos_sensor(0xec, agc_register[BorF * MAX_AG_INDEX + gain_index][11]);
			write_cmos_sensor(0xed, agc_register[BorF * MAX_AG_INDEX + gain_index][12]);
			write_cmos_sensor(0xee, agc_register[BorF * MAX_AG_INDEX + gain_index][13]);
			write_cmos_sensor(0xfe, agc_register[BorF * MAX_AG_INDEX + gain_index][14]);
			break;
		}
}

static kal_uint16 set_gain(kal_uint16 gain)
{
	gain2reg(gain);

	return gain;
}

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le: 0x%x, se: 0x%x, gain: 0x%x\n", le, se, gain);
}

/*
*static void set_mirror_flip(kal_uint8 image_mirror)
*{
*	LOG_INF("image_mirror = %d\n", image_mirror);
*	write_cmos_sensor(0xfe, 0x00);
*	switch (image_mirror) {
*	case IMAGE_NORMAL:
*		write_cmos_sensor(0x17, 0xd4);
*		break;
*	case IMAGE_H_MIRROR:
*		write_cmos_sensor(0x17, 0xd5);
*		break;
*	case IMAGE_V_MIRROR:
*		write_cmos_sensor(0x17, 0xd6);
*		break;
*	case IMAGE_HV_MIRROR:
*		write_cmos_sensor(0x17, 0xd7);
*		break;
*	}
*}
*/

static void night_mode(kal_bool enable)
{
	/* No Need to implement this function */
}

static void sensor_init(void)
{
	LOG_INF("E\n");

	/* System */
	write_cmos_sensor(0xfc, 0x01);
	write_cmos_sensor(0xf4, 0x00);
	write_cmos_sensor(0xf5, 0x80);
	write_cmos_sensor(0xf6, 0x44);
	write_cmos_sensor(0xf8, 0x37);
	write_cmos_sensor(0xfa, 0x4e);
	write_cmos_sensor(0xf9, 0x00);
	write_cmos_sensor(0xf7, 0x31);
	write_cmos_sensor(0xfc, 0x03);
	write_cmos_sensor(0xfc, 0x8a);
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x1a, 0x02);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);

	/* Analog */
	write_cmos_sensor(0x03, 0x08);
	write_cmos_sensor(0x04, 0xd0);
	write_cmos_sensor(0x05, 0x02);
	write_cmos_sensor(0x06, 0x19);
	write_cmos_sensor(0x07, 0x00);
	write_cmos_sensor(0x08, 0x10);
	write_cmos_sensor(0x0a, 0x18);
	write_cmos_sensor(0x0c, 0x00);
	write_cmos_sensor(0x12, 0x02);
	write_cmos_sensor(0x17, GC13023_MIRROR);
	write_cmos_sensor(0x18, 0x02);
	write_cmos_sensor(0x19, 0x08);
	write_cmos_sensor(0x1c, 0x20);
	write_cmos_sensor(0x21, 0x10);
	write_cmos_sensor(0x26, 0x00);
	write_cmos_sensor(0x27, 0x44);
	write_cmos_sensor(0x29, 0x58);
	write_cmos_sensor(0x2f, 0x34);
	write_cmos_sensor(0x32, 0xc8);
	write_cmos_sensor(0x36, 0x10);
	write_cmos_sensor(0xcd, 0xcc);
	write_cmos_sensor(0xce, 0x70);
	write_cmos_sensor(0xd0, 0xa2);
	write_cmos_sensor(0xd1, 0xbb);
	write_cmos_sensor(0xd2, 0xf4);
	write_cmos_sensor(0xd3, 0x3a);
	write_cmos_sensor(0xd8, 0x40);
	write_cmos_sensor(0xd9, 0xe0);
	write_cmos_sensor(0xda, 0x03);
	write_cmos_sensor(0xde, 0xcc);
	write_cmos_sensor(0xe1, 0x1a);
	write_cmos_sensor(0xe3, 0x44);
	write_cmos_sensor(0xe5, 0x0c);
	write_cmos_sensor(0xe6, 0xd0);
	write_cmos_sensor(0xe7, 0xc8);

	/* Stspd */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x89, 0x02);
	write_cmos_sensor(0x8a, 0x01);
	write_cmos_sensor(0x8b, 0x02);
	write_cmos_sensor(0x8c, 0x0a);
	write_cmos_sensor(0xfe, 0x00);

	/* ISP */
	write_cmos_sensor(0x80, 0xd0);
	write_cmos_sensor(0x83, 0x17);
	write_cmos_sensor(0x88, 0x23);
	write_cmos_sensor(0x8e, 0x09);
	write_cmos_sensor(0x8f, 0x14);

	/* DPC */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x08, 0x10);
	write_cmos_sensor(0x09, 0x7e);
	write_cmos_sensor(0xc5, 0x02);
	write_cmos_sensor(0xc1, 0x04);
	write_cmos_sensor(0xc2, 0x10);
	write_cmos_sensor(0xc3, 0x60);
	write_cmos_sensor(0xc4, 0x40);
	write_cmos_sensor(0xfe, 0x00);

	/* GAIN */
	write_cmos_sensor(0xb0, 0x4c);
	write_cmos_sensor(0xb3, 0x00);
	write_cmos_sensor(0xb1, 0x01);
	write_cmos_sensor(0xb2, 0x00);
	write_cmos_sensor(0xb6, 0x00);

	/* BLK */
	write_cmos_sensor(0x40, 0x22);
	write_cmos_sensor(0x43, 0x38);
	write_cmos_sensor(0x4e, 0x0f);
	write_cmos_sensor(0x4f, 0xf0);

	/* Gamma */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3b, 0x06);
	write_cmos_sensor(0x20, 0xda);
	write_cmos_sensor(0x3a, 0x95);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0xdf, 0x00);
	write_cmos_sensor(0xe7, 0x08);
	write_cmos_sensor(0xe8, 0x20);
	write_cmos_sensor(0xe9, 0x04);
	write_cmos_sensor(0xea, 0x1c);
	write_cmos_sensor(0xeb, 0x50);
	write_cmos_sensor(0xec, 0x60);
	write_cmos_sensor(0xed, 0xa0);
	write_cmos_sensor(0xee, 0xc4);
	write_cmos_sensor(0xfe, 0x00);

	/* Darksun */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x38, 0x00);
	write_cmos_sensor(0xfe, 0x00);

	/* Window 4208x3120 */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x90, 0x01);
	write_cmos_sensor(0x92, 0x08);
	write_cmos_sensor(0x94, 0x08);
	write_cmos_sensor(0x95, 0x0c);
	write_cmos_sensor(0x96, 0x30);
	write_cmos_sensor(0x97, 0x10);
	write_cmos_sensor(0x98, 0x70);

	/* MIPI */
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x01, 0x07);
	write_cmos_sensor(0x02, 0x03);
	write_cmos_sensor(0x03, 0x90);
	write_cmos_sensor(0x12, 0x8c);
	write_cmos_sensor(0x13, 0x14);
	write_cmos_sensor(0x15, 0x10);
	write_cmos_sensor(0x18, 0x07);
	write_cmos_sensor(0x19, 0xaa);
	write_cmos_sensor(0x21, 0x08);
	write_cmos_sensor(0x22, 0x08);
	write_cmos_sensor(0x23, 0x24);
	write_cmos_sensor(0x24, 0x02);
	write_cmos_sensor(0x25, 0x1a);
	write_cmos_sensor(0x26, 0x0c);
	write_cmos_sensor(0x29, 0x08);
	write_cmos_sensor(0x2a, 0x24);
	write_cmos_sensor(0x2b, 0x0c);
	write_cmos_sensor(0xfe, 0x00);

	/* Double Reset */
	write_cmos_sensor(0x1a, 0x08);
	write_cmos_sensor(0x1d, 0x13);
	write_cmos_sensor(0x33, 0x80);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x00);
}

#if BINNING_PREVIEW
static void binning_setting(void)
{
	LOG_INF("E\n");

	/* System */
	write_cmos_sensor(0xfc, 0x01);
	write_cmos_sensor(0xf4, 0x08);
	write_cmos_sensor(0xf5, 0x80);
	write_cmos_sensor(0xf6, 0x44);
	write_cmos_sensor(0xf8, 0x1b);
	write_cmos_sensor(0xfa, 0x27);
	write_cmos_sensor(0xf9, 0x00);
	write_cmos_sensor(0xf7, 0x39);
	write_cmos_sensor(0xfc, 0x03);
	write_cmos_sensor(0xfc, 0x8a);
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x1a, 0x02);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x00);

	/* Analog */
	write_cmos_sensor(0x1c, 0x14);
	write_cmos_sensor(0x2f, 0x32);
	write_cmos_sensor(0x33, 0x01);
	write_cmos_sensor(0x36, 0x20);
	write_cmos_sensor(0xce, 0x64);
	write_cmos_sensor(0xd0, 0xa3);
	write_cmos_sensor(0xd8, 0x50);
	write_cmos_sensor(0xde, 0x50);

	/* Stspd */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x08);

	/* DPC */
	write_cmos_sensor(0xc5, 0x05);
	write_cmos_sensor(0xc1, 0x02);
	write_cmos_sensor(0xc2, 0x08);
	write_cmos_sensor(0xc3, 0x30);
	write_cmos_sensor(0xc4, 0x20);
	write_cmos_sensor(0xfe, 0x00);

	/* BLK */
	write_cmos_sensor(0x4e, 0x00);
	write_cmos_sensor(0x4f, 0x3c);

	/* Window 2104x1560 */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x90, 0x01);
	write_cmos_sensor(0x92, 0x04);
	write_cmos_sensor(0x94, 0x04);
	write_cmos_sensor(0x95, 0x06);
	write_cmos_sensor(0x96, 0x18);
	write_cmos_sensor(0x97, 0x08);
	write_cmos_sensor(0x98, 0x38);

	/* MIPI */
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x01, 0x07);
	write_cmos_sensor(0x02, 0x03);
	write_cmos_sensor(0x03, 0x90);
	write_cmos_sensor(0x12, 0x46);
	write_cmos_sensor(0x13, 0x0a);
	write_cmos_sensor(0x15, 0x10);
	write_cmos_sensor(0x18, 0x07);
	write_cmos_sensor(0x19, 0xaa);
	write_cmos_sensor(0x21, 0x08);
	write_cmos_sensor(0x22, 0x02);
	write_cmos_sensor(0x23, 0x24);
	write_cmos_sensor(0x24, 0x02);
	write_cmos_sensor(0x25, 0x10);
	write_cmos_sensor(0x26, 0x04);
	write_cmos_sensor(0x29, 0x02);
	write_cmos_sensor(0x2a, 0x0f);
	write_cmos_sensor(0x2b, 0x04);
	write_cmos_sensor(0xfe, 0x00);

	/* Double Reset */
	write_cmos_sensor(0x1a, 0x08);
	write_cmos_sensor(0x1d, 0x13);
	write_cmos_sensor(0x33, 0x81);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0xd0);
	BorF = 0;
}
#endif

static void fullsize_setting(void)
{
	LOG_INF("E\n");

	/* System */
	write_cmos_sensor(0xfc, 0x01);
	write_cmos_sensor(0xf4, 0x00);
	write_cmos_sensor(0xf5, 0x80);
	write_cmos_sensor(0xf6, 0x44);
	write_cmos_sensor(0xf8, 0x37);
	write_cmos_sensor(0xfa, 0x4e);
	write_cmos_sensor(0xf9, 0x00);
	write_cmos_sensor(0xf7, 0x31);
	write_cmos_sensor(0xfc, 0x03);
	write_cmos_sensor(0xfc, 0x8a);
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x1a, 0x02);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x00);

	/* Analog */
	write_cmos_sensor(0x1c, 0x20);
	write_cmos_sensor(0x2f, 0x34);
	write_cmos_sensor(0x33, 0x00);
	write_cmos_sensor(0x36, 0x10);
	write_cmos_sensor(0xce, 0x70);
	write_cmos_sensor(0xd0, 0xa2);
	write_cmos_sensor(0xd8, 0x40);
	write_cmos_sensor(0xde, 0xcc);

	/* Stspd */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x0a);

	/* DPC */
	write_cmos_sensor(0xc5, 0x02);
	write_cmos_sensor(0xc1, 0x04);
	write_cmos_sensor(0xc2, 0x10);
	write_cmos_sensor(0xc3, 0x60);
	write_cmos_sensor(0xc4, 0x40);
	write_cmos_sensor(0xfe, 0x00);

	/* BLK */
	write_cmos_sensor(0x4e, 0x0f);
	write_cmos_sensor(0x4f, 0xf0);

	/* Window 4208x3120 */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x90, 0x01);
	write_cmos_sensor(0x92, 0x08);
	write_cmos_sensor(0x94, 0x08);
	write_cmos_sensor(0x95, 0x0c);
	write_cmos_sensor(0x96, 0x30);
	write_cmos_sensor(0x97, 0x10);
	write_cmos_sensor(0x98, 0x70);

	/* MIPI */
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x01, 0x07);
	write_cmos_sensor(0x02, 0x03);
	write_cmos_sensor(0x03, 0x90);
	write_cmos_sensor(0x12, 0x8c);
	write_cmos_sensor(0x13, 0x14);
	write_cmos_sensor(0x15, 0x10);
	write_cmos_sensor(0x18, 0x07);
	write_cmos_sensor(0x19, 0xaa);
	write_cmos_sensor(0x21, 0x08);
	write_cmos_sensor(0x22, 0x08);
	write_cmos_sensor(0x23, 0x24);
	write_cmos_sensor(0x24, 0x02);
	write_cmos_sensor(0x25, 0x1a);
	write_cmos_sensor(0x26, 0x0c);
	write_cmos_sensor(0x29, 0x08);
	write_cmos_sensor(0x2a, 0x24);
	write_cmos_sensor(0x2b, 0x0c);
	write_cmos_sensor(0xfe, 0x00);

	/* Double Reset */
	write_cmos_sensor(0x1a, 0x08);
	write_cmos_sensor(0x1d, 0x13);
	write_cmos_sensor(0x33, 0x80);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0xd0);
	BorF = 1;
}

static void preview_setting(void)
{
#if BINNING_PREVIEW
	binning_setting();
#else
	fullsize_setting();
#endif
}

static void capture_setting(void)
{
	fullsize_setting();
}

static void normal_video_setting(void)
{
	fullsize_setting();
}

static void hs_video_setting(void)
{
	fullsize_setting();
}

static void slim_video_setting(void)
{
	fullsize_setting();
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x8c, 0x11);
	} else {
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x8c, 0x10);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {
				pr_err("[GC13023MIPI]get_imgsensor_id: i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			pr_err("[GC13023MIPI]get_imgsensor_id: Read sensor id fail, write id: 0x%x, id: 0x%x\n",
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

static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;

	LOG_1;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);

		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				pr_err("[GC13023MIPI]open: i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			pr_err("[GC13023MIPI]open: Read sensor id fail, write id: 0x%x, id: 0x%x\n",
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

	gc13023_gcore_identify_otp();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}

static kal_uint32 close(void)
{
	LOG_INF("E\n");
	/* No Need to implement this function */
	return ERROR_NONE;
}

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
	preview_setting();
	return ERROR_NONE;
}

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
	capture_setting();
	return ERROR_NONE;
}

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
	normal_video_setting();
	return ERROR_NONE;
}

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
	return ERROR_NONE;
}

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
	sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;
	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
	MSDK_SENSOR_INFO_STRUCT *sensor_info,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;
	sensor_info->SensorResetActiveHigh = FALSE;
	sensor_info->SensorResetDelayCount = 5;

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	/* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
	/* The frame of setting sensor gain */
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
	sensor_info->SensorHightSampling = 0;  /* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;
#if PDAFSUPPORT
	sensor_info->PDAF_Support = 1;
#else
	sensor_info->PDAF_Support = 0;
#endif

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
}

static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
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
}

static kal_uint32 set_video_mode(UINT16 framerate)
{	/*This Function not used after ROME*/
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	/*
	 *if (framerate == 0)	 //Dynamic frame rate
	 *	return ERROR_NONE;
	 *spin_lock(&imgsensor_drv_lock);
	 *if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
	 *	imgsensor.current_fps = 296;
	 *else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
	 *	imgsensor.current_fps = 146;
	 *else
	 *	imgsensor.current_fps = framerate;
	 *spin_unlock(&imgsensor_drv_lock);
	 *set_max_framerate(imgsensor.current_fps, 1);
	*/
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

static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
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
		imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ?
			(frame_length - imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ?
				(frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
				LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
					framerate, imgsensor_info.cap.max_framerate / 10);
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ?
				(frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ?
			(frame_length - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ?
			(frame_length - imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	default:  /* coding with  preview scenario by default */
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
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

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
	UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *)feature_para;
	UINT16 *feature_data_16 = (UINT16 *)feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *)feature_para;
	UINT32 *feature_data_32 = (UINT32 *)feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;
	/* unsigned long long *feature_return_para=(unsigned long long *) feature_para; */

	SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *)feature_para;
#if PDAFSUPPORT
	SET_PD_BLOCK_INFO_T *PDAFinfo;
#endif
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
		night_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16)*feature_data);
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
		set_auto_flicker_mode((BOOL)*feature_data_16, *(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(MUINT32 *)(uintptr_t)(*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: /* for factory mode auto testing */
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
		wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data + 1));

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
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n", (UINT16)*feature_data,
			(UINT16)*(feature_data + 1), (UINT16)*(feature_data + 2));
		ihdr_write_shutter_gain((UINT16)*feature_data, (UINT16)*(feature_data + 1),
			(UINT16)*(feature_data + 2));
		break;
#if PDAFSUPPORT
		/******************** PDAF START **********************/
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%llu\n", *feature_data);
		/* PDAF capacity enable or not, 2p8 only full size support PDAF */
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1; /* video & capture use same setting */
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1;
			break;
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%llu\n", *feature_data);
		PDAFinfo = (SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data + 1));

		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info, sizeof(SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		default:
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
		gc13023_read_eeprom((kal_uint16)(*feature_data), (char *)(uintptr_t)(*(feature_data + 1)),
			(kal_uint32)(*(feature_data + 2)));
		break;
	/*
	*case SENSOR_FEATURE_SET_PDAF:
	*	LOG_INF("SENSOR_FEATURE_SET_PDAF PDAF mode : %d\n", *feature_data_16);
	*	imgsensor.pdaf_mode = *feature_data_16;
	*	break;
	*/
	/******************** PDAF END *********************/
#endif
	default:
		break;
	}

	return ERROR_NONE;
}

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 GC13023MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}

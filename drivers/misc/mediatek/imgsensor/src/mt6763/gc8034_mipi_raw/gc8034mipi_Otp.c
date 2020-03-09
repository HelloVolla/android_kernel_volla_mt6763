/*
*
* Filename:
* ---------
*     GC8034mipi_OTP.c
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
/*#include <asm/atomic.h>*/

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "gc8034mipi_Otp.h"

/****************************Modify Following Strings for Debug****************************/
#define PFX "GC8034_otp"
/****************************   Modify end    *******************************************/
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __func__, ##args)

static kal_uint8 LSC_ADDR[4] = { 0x0e, 0x20, 0x1a, 0x88 };
kal_uint8 otp_i2c_write_id = 0x6e;

static struct gc8034_otp_struct gc8034_otp_data;

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;

	char pu_send_cmd[1] = { (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 1, (u8 *)&get_byte, 1, otp_i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[2] = { (char)(addr & 0xFF), (char)(para & 0xFF) };
	iWriteRegI2C(pu_send_cmd, 2, otp_i2c_write_id);
}

static kal_uint8 gc8034_otp_read(kal_uint8 page, kal_uint8 addr)
{
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xd4, ((page << 2) & 0x3c) + ((addr >> 5) & 0x03));
	write_cmos_sensor(0xd5, (addr << 3) & 0xff);
	mdelay(1);
	write_cmos_sensor(0xf3, 0x20);

	return  read_cmos_sensor(0xd7);
}

static void gc8034_otp_read_group(kal_uint8 page, kal_uint8 addr, kal_uint8 *buff, int size)
{
	kal_uint8 i;
	kal_uint8 regf4 = read_cmos_sensor(0xf4);
	write_cmos_sensor(0xd4, ((page << 2) & 0x3c) + ((addr >> 5) & 0x03));
	write_cmos_sensor(0xd5, (addr << 3) & 0xff);
	mdelay(1);
	write_cmos_sensor(0xf3, 0x20);
	write_cmos_sensor(0xf4, regf4 | 0x02);
	write_cmos_sensor(0xf3, 0x80);

	for (i = 0; i < size; i++)
		buff[i] = read_cmos_sensor(0xd7);

	write_cmos_sensor(0xf3, 0x00);
	write_cmos_sensor(0xf4, regf4 & 0xfd);
}

static void gc8034_otp_format(void)
{
	kal_uint8  flag_dd = 0;
	kal_uint16 i = 0, j = 0;
	kal_uint8  temp = 0;
#ifdef GC8034OTP_FOR_CUSTOMER
	kal_uint32 check = 0;
	kal_uint8  flag_wb = 0, index = 0, flag_module = 0, flag_lsc = 0, flag_af = 0;
	kal_uint8  info[8] = { 0 };
	kal_uint8  wb[4] = { 0 };
	kal_uint8  golden[4] = { 0 };
	kal_uint8  af[4] = { 0 };
#endif
#ifdef GC8034OTP_DEBUG
	kal_uint8  debug[128] = { 0 };
#endif

	/* Static Defective Pixel */
	flag_dd = gc8034_otp_read(0, 0x0b);
	LOG_INF("GC8034_OTP_DD : flag_dd = 0x%x\n", flag_dd);

	switch (flag_dd & 0x03) {
	case 0x00:
		LOG_INF("Defective Pixel is Empty !!\n");
		gc8034_otp_data.dd_flag = 0x00;
		break;
	case 0x01:
		LOG_INF("Defective Pixel is Valid!!\n");
		gc8034_otp_data.dd_cnt = gc8034_otp_read(0, 0x0c) + gc8034_otp_read(0, 0x0d);
		gc8034_otp_data.dd_flag = 0x01;
		LOG_INF("Defective Pixel's total_number = %d\n", gc8034_otp_data.dd_cnt);
		break;
	case 0x02:
	case 0x03:
		LOG_INF("Defective Pixel is Invalid !!\n");
		gc8034_otp_data.dd_flag = 0x02;
		break;
	default:
		break;
	}

#ifdef GC8034OTP_FOR_CUSTOMER
	flag_module = gc8034_otp_read(9, 0x6f);
	flag_wb = gc8034_otp_read(9, 0x5e);
	flag_af = gc8034_otp_read(3, 0x3a);
	LOG_INF("GC8034_OTP : flag_module = 0x%x , flag_wb = 0x%x , flag_af = 0x%x\n", flag_module, flag_wb, flag_af);

	/* INFO & WB & AF */
	for (index = 0; index < 2; index++) {
		switch ((flag_module << (2 * index)) & 0x0c) {
		case 0x00:
			LOG_INF("module info group %d is Empty !!\n", index + 1);
			break;
		case 0x04:
			LOG_INF("module info group %d is Valid !!\n", index + 1);
			check = 0;
			gc8034_otp_read_group(9, INFO_ROM_START + index * INFO_WIDTH, &info[0], INFO_WIDTH);
			for (i = 0; i < INFO_WIDTH - 1; i++)
				check += info[i];

			if ((check % 255 + 1) == info[INFO_WIDTH - 1]) {
				gc8034_otp_data.module_id = info[0];
				gc8034_otp_data.lens_id = info[1];
				gc8034_otp_data.vcm_driver_id = info[2];
				gc8034_otp_data.vcm_id = info[3];
				gc8034_otp_data.year = info[4];
				gc8034_otp_data.month = info[5];
				gc8034_otp_data.day = info[6];
			} else
				LOG_INF("module info Check sum %d Error !!\n", index + 1);
			break;
		case 0x08:
		case 0x0c:
			LOG_INF("module info group %d is Invalid !!\n", index + 1);
			break;
		default:
			break;
		}

		switch ((flag_wb << (2 * index)) & 0x0c) {
		case 0x00:
			LOG_INF("wb info group %d is Empty !!\n", index + 1);
			gc8034_otp_data.wb_flag = gc8034_otp_data.wb_flag | 0x00;
			break;
		case 0x04:
			LOG_INF("wb info group %d is Valid !!\n", index + 1);
			check = 0;
			gc8034_otp_read_group(9, WB_ROM_START + index * WB_WIDTH, &wb[0], WB_WIDTH);
			for (i = 0; i < WB_WIDTH - 1; i++)
				check += wb[i];

			if ((check % 255 + 1) == wb[WB_WIDTH - 1]) {
				gc8034_otp_data.rg_gain = (wb[0] | ((wb[1] & 0xf0) << 4)) > 0 ?
					(wb[0] | ((wb[1] & 0xf0) << 4)) : 0x400;
				gc8034_otp_data.bg_gain = (((wb[1] & 0x0f) << 8) | wb[2]) > 0 ?
					(((wb[1] & 0x0f) << 8) | wb[2]) : 0x400;
				gc8034_otp_data.wb_flag = gc8034_otp_data.wb_flag | 0x01;
			} else
				LOG_INF("wb info check sum %d Error !!\n", index + 1);
			break;
		case 0x08:
		case 0x0c:
			LOG_INF("wb info group %d is Invalid !!\n", index + 1);
			gc8034_otp_data.wb_flag = gc8034_otp_data.wb_flag | 0x02;
			break;
		default:
			break;
		}

		switch ((flag_wb << (2 * index)) & 0xc0) {
		case 0x00:
			LOG_INF("wb golden info group %d is Empty !!\n", index + 1);
			gc8034_otp_data.golden_flag = gc8034_otp_data.golden_flag | 0x00;
			break;
		case 0x40:
			LOG_INF("wb golden info group %d is Valid !!\n", index + 1);
			check = 0;
			gc8034_otp_read_group(9, GOLDEN_ROM_START + index * GOLDEN_WIDTH, &golden[0], GOLDEN_WIDTH);
			for (i = 0; i < GOLDEN_WIDTH - 1; i++)
				check += golden[i];

			if ((check % 255 + 1) == golden[GOLDEN_WIDTH - 1]) {
				gc8034_otp_data.golden_rg = (golden[0] | ((golden[1] & 0xf0) << 4)) > 0 ?
					(golden[0] | ((golden[1] & 0xf0) << 4)) : RG_TYPICAL;
				gc8034_otp_data.golden_bg = (((golden[1] & 0x0f) << 8) | golden[2]) > 0 ?
					(((golden[1] & 0x0f) << 8) | golden[2]) : BG_TYPICAL;
				gc8034_otp_data.golden_flag = gc8034_otp_data.golden_flag | 0x01;
			} else
				LOG_INF("wb golden info check sum %d Error !!\n", index + 1);

			break;
		case 0x80:
		case 0xc0:
			LOG_INF("wb golden info group %d is Invalid !!\n", index + 1);
			gc8034_otp_data.golden_flag = gc8034_otp_data.golden_flag | 0x02;
			break;
		default:
			break;
		}

		switch ((flag_af << (2 * index)) & 0x0c) {
		case 0x00:
			LOG_INF("af group %d is Empty !!\n", index + 1);
			gc8034_otp_data.af_flag = gc8034_otp_data.af_flag | 0x00;
			break;
		case 0x04:
			LOG_INF("af group %d is Valid !!\n", index + 1);
			check = 0;
			gc8034_otp_read_group(3, AF_ROM_START + index * AF_WIDTH, &af[0], AF_WIDTH);
			for (i = 0; i < AF_WIDTH - 1; i++)
				check += af[i];

			if ((check % 255 + 1) == af[AF_WIDTH - 1]) {
				gc8034_otp_data.af_infinity = ((info[0] << 4) & 0x0f00) + info[1];
				gc8034_otp_data.af_macro = ((info[0] << 8) & 0x0f00) + info[2];
				gc8034_otp_data.af_flag = gc8034_otp_data.af_flag | 0x01;
			} else
				LOG_INF("af check sum %d Error !!\n", index + 1);
			break;
		case 0x08:
		case 0x0c:
			LOG_INF("af group %d is Invalid !!\n", index + 1);
			gc8034_otp_data.af_flag = gc8034_otp_data.af_flag | 0x02;
			break;
		default:
			break;
		}
	}

	/* LSC */
	flag_lsc = gc8034_otp_read(3, 0x43);
	gc8034_otp_data.lsc_flag = 2;
	for (index = 0; index < 2; index++) {
		switch ((flag_lsc << (2 * index)) & 0x0c) {
		case 0x00:
			LOG_INF("lens shading compensation group %d is Empty !\n", index + 1);
			break;
		case 0x04:
			LOG_INF("lens shading compensation group %d is Valid !\n", index + 1);
			gc8034_otp_data.lsc_flag = index;
			break;
		case 0x08:
		case 0x0c:
			LOG_INF("lens shading compensation group %d is Invalid !!\n", index + 1);
			break;
		default:
			break;
		}
	}

	/* print otp information */
	LOG_INF("custom info module_id=0x%x\n", gc8034_otp_data.module_id);
	LOG_INF("custom info lens_id=0x%x\n", gc8034_otp_data.lens_id);
	LOG_INF("custom info vcm_id=0x%x\n", gc8034_otp_data.vcm_id);
	LOG_INF("custom info vcm_driver_id=0x%x\n", gc8034_otp_data.vcm_driver_id);
	LOG_INF("custom info data=%d-%d-%d\n", gc8034_otp_data.year, gc8034_otp_data.month, gc8034_otp_data.day);
	LOG_INF("custom info r/g=0x%x\n", gc8034_otp_data.rg_gain);
	LOG_INF("custom info b/g=0x%x\n", gc8034_otp_data.bg_gain);
	LOG_INF("custom info golden_rg=0x%x\n", gc8034_otp_data.golden_rg);
	LOG_INF("custom info golden_bg=0x%x\n", gc8034_otp_data.golden_bg);
	LOG_INF("custom info infitiy=0x%x\n", gc8034_otp_data.af_infinity);
	LOG_INF("custom info macro=0x%x\n", gc8034_otp_data.af_macro);
#endif

	/* chip regs */
	gc8034_otp_data.reg_flag = gc8034_otp_read(2, 0x4e);

	if (gc8034_otp_data.reg_flag == 1)
		for (i = 0; i < 5; i++) {
			temp = gc8034_otp_read(2, 0x4f + 5 * i);
			for (j = 0; j < 2; j++)
				if (((temp >> (4 * j + 3)) & 0x01) == 0x01) {
					gc8034_otp_data.reg_update[gc8034_otp_data.reg_num].page =
						(temp >> (4 * j)) & 0x03;
					gc8034_otp_data.reg_update[gc8034_otp_data.reg_num].addr =
						gc8034_otp_read(2, 0x50 + 5 * i + 2 * j);
					gc8034_otp_data.reg_update[gc8034_otp_data.reg_num++].value =
						gc8034_otp_read(2, 0x50 + 5 * i + 2 * j + 1);
				}
		}

#ifdef GC8034OTP_DEBUG
	for (i = 0; i < 9; i++) {
		gc8034_otp_read_group(i, 0, &debug[0], 128);
		for (j = 0; j < 128; j++)
			LOG_INF("debug: Page%d: addr = 0x%x, value = 0x%x;\n", i, j, debug[j]);
	}
#endif
}

static void gc8034_otp_check_prsel(void)
{
	kal_uint8 product_level = 0;

	product_level = gc8034_otp_read(2, 0x68) & 0x07;

	LOG_INF("product level %d!\n", product_level);

	if ((product_level == 0x00) || (product_level == 0x01)) {
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0xd2, 0xcb);
		LOG_INF("pr select 0xcb !\n");
	} else {
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0xd2, 0xc3);
		LOG_INF("pr select 0xc3 !\n");
	}
}

static void gc8034_otp_update_dd(void)
{
	kal_uint8 state = 0;

	if (0x01 == gc8034_otp_data.dd_flag) {
		LOG_INF("defective pixel update started !\n");
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x79, 0x2e);
		write_cmos_sensor(0x7b, gc8034_otp_data.dd_cnt);
		write_cmos_sensor(0x7e, 0x00);
		write_cmos_sensor(0x7f, 0x70);
		write_cmos_sensor(0x6e, 0x01);
		write_cmos_sensor(0xbd, 0xd4);
		write_cmos_sensor(0xbe, 0x9c);
		write_cmos_sensor(0xbf, 0xa0);
		write_cmos_sensor(0xfe, 0x01);
		write_cmos_sensor(0xbe, 0x00);
		write_cmos_sensor(0xa9, 0x01);
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0xf2, 0x41);

		while (1) {
			state = read_cmos_sensor(0x6e);
			if ((state | 0xef) != 0xff)
				break;
			else
				mdelay(10);
		}

		write_cmos_sensor(0xfe, 0x01);
		write_cmos_sensor(0xbe, 0x01);
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x79, 0x00);
	} else {
		LOG_INF("defective pixel info is empty or invalid!\n");
	}
}

#ifdef GC8034OTP_FOR_CUSTOMER
static void gc8034_otp_update_wb(void)
{
	kal_uint16 r_gain_current = 0x400, g_gain_current = 0x400, b_gain_current = 0x400, base_gain = 2048;
	kal_uint16 r_gain = 1024, g_gain = 1024, b_gain = 1024;
	kal_uint16 rg_typical = RG_TYPICAL, bg_typical = BG_TYPICAL;

	if (0x01 == (gc8034_otp_data.golden_flag & 0x01)) {
		rg_typical = gc8034_otp_data.golden_rg;
		bg_typical = gc8034_otp_data.golden_bg;
		LOG_INF("rg_typical = 0x%x, bg_typical = 0x%x\n", rg_typical, bg_typical);
	} else {
		LOG_INF("golden value is emtpy or invalid, use typical value as rg_typical = 0x%x, bg_typical = 0x%x\n",
			rg_typical, bg_typical);
	}

	if (0x01 == (gc8034_otp_data.wb_flag & 0x01)) {
		r_gain_current = 2048 * rg_typical / gc8034_otp_data.rg_gain;
		b_gain_current = 2048 * bg_typical / gc8034_otp_data.bg_gain;
		g_gain_current = 2048;

		base_gain = (r_gain_current < b_gain_current) ? r_gain_current : b_gain_current;
		base_gain = (base_gain < g_gain_current) ? base_gain : g_gain_current;

		r_gain = 0x400 * r_gain_current / base_gain;
		g_gain = 0x400 * g_gain_current / base_gain;
		b_gain = 0x400 * b_gain_current / base_gain;
		LOG_INF("wb gain: r = 0x%x, g = 0x%x, b = 0x%x\n", r_gain, g_gain, b_gain);

		write_cmos_sensor(0xfe, 0x01);
		write_cmos_sensor(0x84, g_gain >> 3);
		write_cmos_sensor(0x85, r_gain >> 3);
		write_cmos_sensor(0x86, b_gain >> 3);
		write_cmos_sensor(0x87, g_gain >> 3);
		write_cmos_sensor(0x88, ((g_gain & 0x07) << 4) + (r_gain & 0x07));
		write_cmos_sensor(0x89, ((b_gain & 0x07) << 4) + (g_gain & 0x07));
		write_cmos_sensor(0xfe, 0x00);
	}
}

static void gc8034_otp_update_lsc(void)
{
	kal_uint8 state = 0;

	if (0x02 != gc8034_otp_data.lsc_flag) {
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x78, 0x9b);
		write_cmos_sensor(0x79, 0x0d);
		write_cmos_sensor(0x7a, LSC_NUM);
		write_cmos_sensor(0x7c, LSC_ADDR[gc8034_otp_data.lsc_flag * 2]);
		write_cmos_sensor(0x7d, LSC_ADDR[gc8034_otp_data.lsc_flag * 2 + 1]);
		write_cmos_sensor(0x6e, 0x01);
		write_cmos_sensor(0xfe, 0x01);
		write_cmos_sensor(0xcf, 0x00);
		write_cmos_sensor(0xc9, 0x01);
		write_cmos_sensor(0xf2, 0x41);
		write_cmos_sensor(0xfe, 0x00);

		while (1) {
			state = read_cmos_sensor(0x6e);
			if ((state | 0xdf) != 0xff)
				break;
			else
				mdelay(10);
		}

		write_cmos_sensor(0xfe, 0x01);
		write_cmos_sensor(0xcf, 0x01);
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x79, 0x00);
		write_cmos_sensor(0xfe, 0x01);
		write_cmos_sensor(0xa0, 0x13);
		write_cmos_sensor(0xfe, 0x00);
		LOG_INF("lsc group %d is valid\n", gc8034_otp_data.lsc_flag + 1);
	} else {
		LOG_INF("lsc is empty or invalid\n");
	}
}
#endif

static void gc8034_otp_update_chipversion(void)
{
	kal_uint8 i = 0;

	LOG_INF("GC8034_OTP_UPDATE_CHIPVERSION:reg_num = %d\n", gc8034_otp_data.reg_num);

	if (gc8034_otp_data.reg_flag)
		for (i = 0; i < gc8034_otp_data.reg_num; i++) {
			write_cmos_sensor(0xfe, gc8034_otp_data.reg_update[i].page);
			write_cmos_sensor(gc8034_otp_data.reg_update[i].addr, gc8034_otp_data.reg_update[i].value);
			LOG_INF("GC8034_OTP_UPDATE_CHIP_VERSION: P%d:0x%x -> 0x%x\n",
				gc8034_otp_data.reg_update[i].page,
				gc8034_otp_data.reg_update[i].addr,
				gc8034_otp_data.reg_update[i].value);
		}
}

static void gc8034_otp_update(void)
{
	gc8034_otp_update_dd();
	gc8034_otp_check_prsel();
#ifdef GC8034OTP_FOR_CUSTOMER
	gc8034_otp_update_wb();
	gc8034_otp_update_lsc();
#endif
	gc8034_otp_update_chipversion();
}

static void gc8034_otp_enable(int state)
{
	kal_uint8 otp_clk = 0, otp_en = 0;

	otp_clk = read_cmos_sensor(0xf2);
	otp_en = read_cmos_sensor(0xf4);
	if (state) {
		otp_clk = otp_clk | 0x01;
		otp_en = otp_en | 0x08;
		write_cmos_sensor(0xf2, otp_clk);
		write_cmos_sensor(0xf4, otp_en);
		LOG_INF("Enable OTP!\n");
	} else {
		otp_en = otp_en & 0xf7;
		otp_clk = otp_clk & 0xfe;
		write_cmos_sensor(0xf4, otp_en);
		write_cmos_sensor(0xf2, otp_clk);
		LOG_INF("Disable OTP!\n");
	}
}

void gc8034mipi_otp_control(void)
{
	memset(&gc8034_otp_data, 0, sizeof(struct gc8034_otp_struct));

	gc8034_otp_enable(1);
	gc8034_otp_format();
	gc8034_otp_update();
	gc8034_otp_enable(0);
}

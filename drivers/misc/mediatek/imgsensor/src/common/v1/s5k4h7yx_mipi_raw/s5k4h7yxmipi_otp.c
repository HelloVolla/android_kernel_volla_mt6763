/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */



#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>



#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "s5k4h7yxmipi_Sensor.h"

#define USHORT             unsigned short
#define BYTE               unsigned char
#define S5K4H7SUB_OTP_LSCFLAG_ADDR	0x0A3D
#define S5K4H7SUB_OTP_FLAGFLAG_ADDR	0x0A04
#define S5K4H7SUB_OTP_AWBFLAG_ADDR	0x0A0D
#define S5K4H7SUB_LSC_PAGE		0
#define S5K4H7SUB_FLAG_PAGE		21
#define S5K4H7SUB_AWB_PAGE		21

typedef struct {
	unsigned short	infoflag;
	unsigned short	lsc_infoflag;
	unsigned short	flag_infoflag;
	unsigned short	flag_module_integrator_id;
	int		awb_offset;
	int		flag_offset;
	int		lsc_offset;
	int 		lsc_group;
	int 		flag_group;
	int 		group;
	unsigned short	frgcur;
	unsigned short	fbgcur;
	unsigned int	nr_gain;
	unsigned int	ng_gain;
	unsigned int	nb_gain;
	unsigned int	ngrcur;
	unsigned int	ngbcur;
	unsigned int	ngcur;
	unsigned int	nrcur;
	unsigned int	nbcur;
	unsigned int	nggolden;
	unsigned int	nrgolden;
	unsigned int	nbgolden;
	unsigned int	ngrgolden;
	unsigned int	ngbgolden;
	unsigned int	frggolden;
	unsigned int	fbggolden;
	unsigned int	awb_flag_sum;
	unsigned int	lsc_sum;
	unsigned int	lsc_check_flag;
}OTP;

OTP otp_data_info = {0};

/**********************************************************
 * get_4h7_page_data
 * get page data
 * return true or false
 * *******************************************************/
void get_4h7_page_data(int pageidx, unsigned char *pdata)
{
	unsigned short get_byte=0;
	unsigned int addr = 0x0A04;
	int i = 0;
	otp_4h7_write_cmos_sensor_8(0x0A02,pageidx);
	otp_4h7_write_cmos_sensor_8(0x0A00,0x01);

	do
	{
		mdelay(1);
		get_byte = otp_4h7_read_cmos_sensor(0x0A01);
	}while((get_byte & 0x01) != 1);

	for(i = 0; i < 64; i++){
		pdata[i] = otp_4h7_read_cmos_sensor(addr);
		addr++;
	}

	otp_4h7_write_cmos_sensor_8(0x0A00,0x00);
}

unsigned short selective_read_region(int pageidx,unsigned int addr)
{
	unsigned short get_byte = 0;
	otp_4h7_write_cmos_sensor_8(0x0A02,pageidx);
	otp_4h7_write_cmos_sensor_8(0x0A00,0x01);
	do
	{
		mdelay(1);
		get_byte = otp_4h7_read_cmos_sensor(0x0A01);
	}while((get_byte & 0x01) != 1);

	get_byte = otp_4h7_read_cmos_sensor(addr);
	otp_4h7_write_cmos_sensor_8(0x0A00,0x00);

	return get_byte;
}

unsigned int selective_read_region_16(int pageidx,unsigned int addr)
{
	unsigned int get_byte = 0;
	static int old_pageidx = 0;
	if(pageidx != old_pageidx){
		otp_4h7_write_cmos_sensor_8(0x0A00,0x00);
		otp_4h7_write_cmos_sensor_8(0x0A02,pageidx);
		otp_4h7_write_cmos_sensor_8(0x0A00,0x01);
		do
		{
			mdelay(1);
			get_byte = otp_4h7_read_cmos_sensor(0x0A01);
		}while((get_byte & 0x01) != 1);
	}

	get_byte = ((otp_4h7_read_cmos_sensor(addr) << 8) | otp_4h7_read_cmos_sensor(addr+1));
	old_pageidx = pageidx;
	return get_byte;
}

/*****************************************************
 * cal_rgb_gain
 * **************************************************/
void cal_rgb_gain(int* r_gain, int* g_gain, int* b_gain, unsigned int r_ration, unsigned int b_ration)
{
	int gain_default = 0x0100;
	if(r_ration >= 1){
		if(b_ration >= 1){
			*g_gain = gain_default;
			*r_gain = (int)((gain_default*1000 * r_ration + 500)/1000);
			*b_gain = (int)((gain_default*1000 * b_ration + 500)/1000);
		}
		else{
			*b_gain = gain_default;
			*g_gain = (int)((gain_default * 1000 / b_ration + 500)/1000);
			*r_gain = (int)((gain_default * r_ration *1000 / b_ration + 500)/1000);
		}
	}
	else{
		if(b_ration >= 1){
			*r_gain = gain_default;
			*g_gain = (int)((gain_default * 1000 / r_ration + 500)/1000);
			*b_gain = (int)((gain_default * b_ration*1000 / r_ration + 500) / 1000);
		}
		else{
			if(r_ration >= b_ration){
				*b_gain = gain_default;
				*g_gain = (int)((gain_default * 1000 / b_ration + 500) / 1000);
				*r_gain = (int)((gain_default * r_ration * 1000 / b_ration + 500) / 1000);
			}
			else{
				*r_gain = gain_default;
				*g_gain = (int)((gain_default * 1000 / r_ration + 500)/1000);
				*b_gain = (int)((gain_default * b_ration * 1000 / r_ration + 500) / 1000);
			}
		}
	}
}

/**********************************************************
 * apply_4h7_otp_awb
 * apply otp
 * *******************************************************/
void apply_4h7_otp_awb(void)
{
	char r_gain_h, r_gain_l, g_gain_h, g_gain_l, b_gain_h, b_gain_l;
	unsigned int r_ratio, b_ratio;

	otp_data_info.ngcur = (unsigned int)((otp_data_info.ngrcur + otp_data_info.ngbcur)*1000/2 + 500);

	otp_data_info.frgcur = (unsigned int)(otp_data_info.nrcur*1000 / otp_data_info.ngcur + 500);
	otp_data_info.fbgcur = (unsigned int)(otp_data_info.nbcur*1000 / otp_data_info.ngcur + 500);

	otp_data_info.nggolden = (unsigned int)((otp_data_info.ngrgolden + otp_data_info.ngbgolden)*1000 / 2 + 500);

	otp_data_info.frggolden = (unsigned int)(otp_data_info.nrgolden*1000/ otp_data_info.nggolden +500);
	otp_data_info.fbggolden = (unsigned int)(otp_data_info.nbgolden*1000/ otp_data_info.nggolden +500);


	r_ratio = (unsigned int)((otp_data_info.frggolden * 1000 / otp_data_info.frgcur + 500)/1000);
	b_ratio = (unsigned int)((otp_data_info.fbggolden * 1000 / otp_data_info.fbgcur + 500)/1000);

	cal_rgb_gain(&otp_data_info.nr_gain, &otp_data_info.ng_gain, &otp_data_info.nb_gain, r_ratio, b_ratio);

	r_gain_h = (otp_data_info.nr_gain >> 8) & 0xff;
	r_gain_l = (otp_data_info.nr_gain >> 0) & 0xff;

	g_gain_h = (otp_data_info.ng_gain >> 8) & 0xff;
	g_gain_l = (otp_data_info.ng_gain >> 0) & 0xff;

	b_gain_h = (otp_data_info.nb_gain >> 8) & 0xff;
	b_gain_l = (otp_data_info.nb_gain >> 0) & 0xff;

	otp_4h7_write_cmos_sensor_8(0x0210,r_gain_h);
	otp_4h7_write_cmos_sensor_8(0x0211,r_gain_l);

	otp_4h7_write_cmos_sensor_8(0x020E,g_gain_h);
	otp_4h7_write_cmos_sensor_8(0x020F,g_gain_l);

	otp_4h7_write_cmos_sensor_8(0x0214,g_gain_h);
	otp_4h7_write_cmos_sensor_8(0x0215,g_gain_l);

	otp_4h7_write_cmos_sensor_8(0x0212,b_gain_h);
	otp_4h7_write_cmos_sensor_8(0x0213,b_gain_l);
	printk("OTP apply_4h7_otp_awb\n");
}

/*********************************************************
 *apply_4h7_otp_lsc
 * ******************************************************/

void apply_4h7_otp_enb_lsc(void)
{
	printk("OTP enable lsc\n");
	otp_4h7_write_cmos_sensor_8(0x0B00,0x01);
}

/*********************************************************
 * otp_group_info_4h7
 * *****************************************************/
int otp_group_info_4h7(void)
{
	memset(&otp_data_info,0,sizeof(OTP));

	otp_data_info.lsc_infoflag =
		selective_read_region(S5K4H7SUB_LSC_PAGE,S5K4H7SUB_OTP_LSCFLAG_ADDR);

	if( otp_data_info.lsc_infoflag == 0x01 ){
		otp_data_info.lsc_offset = 0;
		otp_data_info.lsc_group = 1;
		otp_data_info.lsc_sum = selective_read_region_16(S5K4H7SUB_AWB_PAGE,0x0A32);
	}
	else if ( otp_data_info.lsc_infoflag == 0x03 ){
		otp_data_info.lsc_offset = 1;
		otp_data_info.lsc_group = 2;
		otp_data_info.lsc_sum = selective_read_region_16(S5K4H7SUB_AWB_PAGE,0x0A34);
	}
	else{
		printk("4H7 OTP read data fail lsc empty!!!\n");
		goto error;
	}

	otp_data_info.flag_infoflag =
		selective_read_region(S5K4H7SUB_FLAG_PAGE,S5K4H7SUB_OTP_FLAGFLAG_ADDR);

	if( (otp_data_info.flag_infoflag>>4 & 0x0c) == 0x04 ){
		otp_data_info.flag_offset = 0;
		otp_data_info.flag_group = 1;
	}
	else if ( (otp_data_info.flag_infoflag>>4 & 0x03) == 0x01 ){
		otp_data_info.flag_offset = 4;
		otp_data_info.flag_group = 2;
	}
	else{
		printk("4h7 OTP read data fail flag empty!!!\n");
		goto error;
	}

	otp_data_info.flag_module_integrator_id =
		otp_4h7_read_cmos_sensor(S5K4H7SUB_OTP_FLAGFLAG_ADDR + otp_data_info.flag_offset + 1);

	otp_data_info.infoflag = selective_read_region(S5K4H7SUB_AWB_PAGE,S5K4H7SUB_OTP_AWBFLAG_ADDR);

	otp_data_info.nrcur = selective_read_region_16(S5K4H7SUB_AWB_PAGE,0x0A0E);
	otp_data_info.nbcur = selective_read_region_16(S5K4H7SUB_AWB_PAGE,0x0A10);
	otp_data_info.ngrcur = selective_read_region_16(S5K4H7SUB_AWB_PAGE,0x0A12);
	otp_data_info.ngbcur = selective_read_region_16(S5K4H7SUB_AWB_PAGE,0x0A14);

	otp_data_info.nrgolden = selective_read_region_16(S5K4H7SUB_AWB_PAGE,0x0A16);
	otp_data_info.nbgolden = selective_read_region_16(S5K4H7SUB_AWB_PAGE,0x0A18);
	otp_data_info.ngrgolden = selective_read_region_16(S5K4H7SUB_AWB_PAGE,0x0A1A);
	otp_data_info.ngbgolden = selective_read_region_16(S5K4H7SUB_AWB_PAGE,0x0A1c);

	if( (otp_data_info.infoflag>>4 & 0x0c) == 0x04 ){
		otp_data_info.awb_offset = 0;
		otp_data_info.group = 1;
		otp_data_info.awb_flag_sum = selective_read_region_16(S5K4H7SUB_AWB_PAGE,0x0A2E);
	}
	else if ( (otp_data_info.infoflag>>4 & 0x03) == 0x01 ){
		otp_data_info.awb_offset = 0X10;
		otp_data_info.group = 2;
		otp_data_info.awb_flag_sum = selective_read_region_16(S5K4H7SUB_AWB_PAGE,0x0A30);
	}
	else{
		printk("4h7 OTP read data fail otp_data_info.empty!!!\n");
		goto error;
	}

	return  0;
error:
	return  -1;
}
/*********************************************************
 * read_4h7_page
 * read_Page1~Page21 of data
 * return true or false
 ********************************************************/
bool read_4h7_page(int page_start,int page_end,unsigned char *pdata)
{
	bool bresult = true;
	int st_page_start = page_start;
	if (page_start <= 0 || page_end > 21){
		bresult = false;
		printk(" OTP page_end is large!");
		return bresult;
	}
	for(; st_page_start <= page_end; st_page_start++){
		get_4h7_page_data(st_page_start, pdata);
	}
	return bresult;
}

unsigned int sum_awb_flag_lsc(unsigned int sum_start, unsigned int sum_end, unsigned char *pdata)
{
	int i = 0;
	unsigned int start;
	unsigned int re_sum = 0;
	for(start = 0x0A04; i < 64; i++, start++){
		if((start >= sum_start) && (start <= sum_end)){
			re_sum += pdata[i];
		}
	}
	return  re_sum;
}

bool check_sum_flag_awb(void)
{
	int page_start = 21,page_end = 21;

	unsigned char data_p[21][64] = {};
	bool bresult = true;
	unsigned int  sum_awbfg = 0;

	bresult &= read_4h7_page(page_start, page_end, data_p[page_start-1]);

	if(otp_data_info.group == 1){
		sum_awbfg = sum_awb_flag_lsc(0x0A05, 0X0A08, data_p[page_start-1]);
		sum_awbfg += sum_awb_flag_lsc(0x0A0E, 0X0A1D, data_p[page_start-1]);
	}
	else if (otp_data_info.group == 2){
		sum_awbfg = sum_awb_flag_lsc(0x0A09, 0X0A0C, data_p[page_start-1]);
		sum_awbfg += sum_awb_flag_lsc(0x0A1E, 0X0A2D, data_p[page_start-1]);
	}
	if(sum_awbfg == otp_data_info.awb_flag_sum){
		apply_4h7_otp_awb();
	}
	else{
		printk("OTP 4h7 check awb flag sum fail!!!");
		bresult &= 0;
	}
	return  bresult;
}

bool  check_sum_flag_lsc(void)
{
	int page_start = 21,page_end = 21;

	unsigned char data_p[21][64] = {};
	bool bresult = true;
	unsigned int  sum_slc = 0;

	if(otp_data_info.lsc_group == 1){
		for(page_start = 1, page_end = 6; page_start <= page_end; page_start++){
			bresult &= read_4h7_page(page_start, page_start, data_p[page_start-1]);
			if(page_start == 6){
				sum_slc += sum_awb_flag_lsc(0x0A04, 0x0A2B, data_p[page_start-1]);
				continue;
			}
			sum_slc += sum_awb_flag_lsc(0x0A04, 0X0A43,data_p[page_start-1]);
		}
	}
	else if(otp_data_info.lsc_group == 2){
		for(page_start = 6, page_end = 12; page_start <= page_end; page_start++){
			bresult &= read_4h7_page(page_start, page_start, data_p[page_start-1]);
			if(page_start == 6){
				sum_slc += sum_awb_flag_lsc(0x0A2C, 0x0A43, data_p[page_start-1]);
				continue;
			}
			else if(page_start <12){
				sum_slc += sum_awb_flag_lsc(0x0A04, 0X0A43, data_p[page_start-1]);
			}
			else{
				sum_slc += sum_awb_flag_lsc(0x0A04, 0X0A13, data_p[page_start-1]);
			}
		}
	}

	if(sum_slc == otp_data_info.lsc_sum){
		apply_4h7_otp_enb_lsc();
		otp_data_info.lsc_check_flag = 1;
	}
	else{
		printk("OTP 4h7 check lsc sum fail!!!");
		bresult &= 0;
		otp_data_info.lsc_check_flag = 0;
	}
	return  bresult;
}

bool update_otp(void)
{
	int result = 1;
	if(otp_group_info_4h7() == -1){
		printk("OTP read data fail  empty!!!\n");
		result &= 0;
	}
	else{
		if(check_sum_flag_awb() == 0 && otp_data_info.lsc_check_flag){
			printk("OTP 4h7 check sum fail!!!\n");
			result &= 0;
		}
		else{
			printk("OTP 4h7 check ok\n");
		}
	}
	return  result;
}

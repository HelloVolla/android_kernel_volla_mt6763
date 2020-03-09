/*
*
* Filename:
* ---------
*     GC8034mipi_Otp.h
*
* Project:
* --------
*     ALPS
*
* Description:
* ------------
*     CMOS sensor header file
*
*/

#ifndef _GC8034MIPI_OTP_H
#define _GC8034MIPI_OTP_H

#define GC8034OTP_FOR_CUSTOMER
/* Please do not enable the following macro definition, if you are not debuging otp function*/
/*#define GC8034OTP_DEBUG*/

#ifdef GC8034OTP_FOR_CUSTOMER
#define RG_TYPICAL			0x0400  /* modify these two typical value as golden R/G & B/G ratio*/
#define BG_TYPICAL			0x0400
#define AF_ROM_START		0x3b
#define AF_WIDTH			0x04
#define INFO_ROM_START		0x70
#define INFO_WIDTH			0x08
#define WB_ROM_START		0x5f
#define WB_WIDTH			0x04
#define GOLDEN_ROM_START	0x67
#define GOLDEN_WIDTH		0x04
#define LSC_NUM				99		/* (7+2)*(9+2) */
#endif

struct gc_register {
	kal_uint8 page;
	kal_uint8 addr;
	kal_uint8 value;
};

struct gc8034_otp_struct {
	kal_uint8  dd_cnt;
	kal_uint8  dd_flag;
	kal_uint8  reg_flag;
	kal_uint8  reg_num;
	struct gc_register reg_update[10];
#ifdef GC8034OTP_FOR_CUSTOMER
	kal_uint8  module_id;
	kal_uint8  lens_id;
	kal_uint8  vcm_id;
	kal_uint8  vcm_driver_id;
	kal_uint8  year;
	kal_uint8  month;
	kal_uint8  day;
	kal_uint8  af_flag;
	kal_uint16 af_infinity;
	kal_uint16 af_macro;
	kal_uint8  wb_flag;
	kal_uint16 rg_gain;
	kal_uint16 bg_gain;
	kal_uint8  golden_flag;
	kal_uint16 golden_rg;
	kal_uint16 golden_bg;
	kal_uint8  lsc_flag;		/* 0:Empty 1:Success 2:Invalid */
#endif
};

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
extern int iWriteReg(u16 a_u2Addr, u32 a_u4Data, u32 a_u4Bytes, u16 i2cId);

#endif

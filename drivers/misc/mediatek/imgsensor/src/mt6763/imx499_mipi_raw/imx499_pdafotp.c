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

#define IMX499_PDAFOTP_DEBUG

#ifdef IMX499_PDAFOTP_DEBUG
#define PFX "IMX499_pdafotp"
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
#else
#define LOG_INF(format, args...)
#endif

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iMultiReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId, u8 number);


#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

#define IMX499_EEPROM_READ_ID  0xA0
#define IMX499_EEPROM_WRITE_ID   0xA1
#define IMX499_I2C_SPEED        100
#define IMX499_MAX_OFFSET		0xFFFF

#define DATA_SIZE 2048
BYTE imx499_eeprom_data[DATA_SIZE]= {0};
static bool get_done = false;
static int last_size = 0;
static int last_offset = 0;


static bool selective_read_eeprom(kal_uint16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    if(addr > IMX499_MAX_OFFSET){
        return false;
	}

	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, IMX499_EEPROM_READ_ID)<0)
	{
		return false;
	}
    return true;
}

static bool _read_imx499_eeprom(kal_uint16 addr, BYTE* data, kal_uint32 size ){
	int i = 0;
	int offset = addr;
	for(i = 0; i < size; i++) {
		if(!selective_read_eeprom(offset, &data[i])){
			return false;
		}
		LOG_INF("read_eeprom 0x%0x %d\n",offset, data[i]);
		offset++;
	}
	get_done = true;
	last_size = size;
	last_offset = addr;
    return true;
}

bool read_imx499_eeprom_LRC( kal_uint16 addr, BYTE* data, kal_uint32 size){
	int i = 0;

	addr = 0x0CEC;
	size = 140;
	

	//LOG_INF("read imx499 eeprom, size = %d\n", size);

	if(!get_done || last_size != size || last_offset != addr) {
		if(!_read_imx499_eeprom(addr, imx499_eeprom_data, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			return false;
		}
	}
    for(i=0;i<140;i++)
		printk("imx499_eeprom_data is 0x%x \n",imx499_eeprom_data[i]);
	//memset(imx258_eeprom_data, 0x01, size);
	memcpy(data, imx499_eeprom_data, size);
    return true;
}



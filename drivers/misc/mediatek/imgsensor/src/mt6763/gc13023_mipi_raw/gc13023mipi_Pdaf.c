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
#include "gc13023mipi_Pdaf.h"

/************************** Modify Following Strings for Debug **************************/
#define PFX "GC13023PDAF"
/****************************   Modify end    *******************************************/
#define GC13023_PDAF_DEBUG      0
#if GC13023_PDAF_DEBUG
#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

BYTE gc13023_eeprom_data[DATA_SIZE] = { 0 };

static kal_uint16 read_cmos_sensor_byte(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xff) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, EEPROM_WRITE_ID);
	return get_byte;
}

static bool _read_eeprom(kal_uint16 addr, kal_uint32 size)
{
	/* continue read reg by byte */
	int i = 0;

	for (; i < size; i++) {
		gc13023_eeprom_data[i] = read_cmos_sensor_byte(addr + i);
		LOG_INF("addr = 0x%x,\tvalue = 0x%x", i, gc13023_eeprom_data[i]);
	}

	return true;
}

bool gc13023_read_eeprom(kal_uint16 addr, BYTE *data, kal_uint32 size)
{
	addr = START_ADDR;
	size = DATA_SIZE;

	LOG_INF("Read EEPROM, addr = 0x%x, size = %d\n", addr, size);

	if (!_read_eeprom(addr, size)) {
		LOG_INF("error: read_eeprom fail!\n");
		return false;
	}

	memcpy(data, gc13023_eeprom_data, size);
	return true;
}

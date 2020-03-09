#ifndef __GC13023MIPI_PDAF_H__
#define __GC13023MIPI_PDAF_H__

#define EEPROM_WRITE_ID   0xA0
#define MAX_OFFSET		  0xFFFF
#define DATA_SIZE         1404
#define START_ADDR        0x1801

#define BYTE    unsigned char

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
extern int iMultiReadReg(u16 a_u2Addr, u8 *a_puBuff, u16 i2cId, u8 number);

#endif

/*
 *
 * FocalTech fts TouchScreen driver.
 *
 * Copyright (c) 2012-2018, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*****************************************************************************
*
* File Name: focaltech_upgrade_ft8739.c
*
* Author: Focaltech Driver Team
*
* Created: 2018-02-28
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "../focaltech_core.h"
#include "../focaltech_flash.h"

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
u8 pb_file_ft8739[] = {
#include "../include/pramboot/FT8739_Pramboot_V0.4_20180309.i"
};

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define LIC_FS_H_OFF                0
#define LIC_FS_L_OFF                1
#define LIC_CHECKSUM_H_OFF          2
#define LIC_CHECKSUM_L_OFF          3
#define LIC_LCD_LEN_H_OFF           4
#define LIC_LCD_LEN_L_OFF           5
#define LIC_ECC_H_OFF               6
#define LIC_ECC_L_OFF               7
#define LIC_ECC_START_OFF           12

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
u16 lcd_crc16(u8 *buf, u16 len)
{
    int i = 0;
    u16 crc_in = 0xFFFF;
    u16 c_poly = 0x8005;
    u8 ch = 0;

    while (len--) {
        ch = *(buf++);
        crc_in ^= (ch << 8);
        for (i = 0; i < 8; i++) {
            if (crc_in & 0x8000)
                crc_in = (crc_in << 1) ^ c_poly;
            else
                crc_in = crc_in << 1;
        }
    }

    return crc_in;
}

static int cal_lcdinitcode_ecc(u8 *buf, u16 *ecc_val)
{
    u8 bank_len = 0;
    u16 bank_pos = 0;
    u16 lcd_len = 0;
    u16 pos = 0;
    u16 i = 0;
    u16 idx = 0;
    u16 addr_h = 0;
    u16 addr_l = 0;
    u8 tmp = 0;
    u8 *ecc_buf = NULL;

    if ((NULL == buf) || (NULL == ecc_val)) {
        FTS_ERROR("buf/ecc_val is NULL");
        return -EINVAL;
    }

    lcd_len = ((u16)buf[LIC_LCD_LEN_H_OFF] << 8) + buf[LIC_LCD_LEN_L_OFF];
    if ((lcd_len >= FTS_MAX_LEN_SECTOR) || (lcd_len <= FTS_MIN_LEN)) {
        FTS_ERROR("host lcd len(0x%x) is too large", lcd_len);
        return -EINVAL;
    }

    ecc_buf = kmalloc(lcd_len, GFP_KERNEL);
    if (NULL == ecc_buf) {
        FTS_ERROR("initial code ecc buf malloc fail");
        return -EINVAL;
    }
    memset(ecc_buf, 0xFF, lcd_len);

    for (i = 0; i < lcd_len - 4; i++) {
        tmp = buf[i + LIC_ECC_START_OFF]; /* cal from bank0(offset 12) */

        if (idx == 0) {
            addr_h = tmp;
            ecc_buf[pos++] = tmp;
            idx = 1;
        } else if (idx == 1) {
            addr_l = tmp;
            idx = 2;
        } else if (idx == 2) {
            bank_len = tmp;
            bank_pos = 0;
            idx = 3;
        } else if (idx == 3) {
            ecc_buf[pos++] = tmp + addr_l + bank_pos;
            if (bank_pos < bank_len - 1) {
                bank_pos++;
            } else {
                idx = 0;
                addr_h = 0;
                addr_l = 0;
            }
        }
    }
    /* abnormal terminal */
    if ((idx == 1) || (idx == 2)) {
        pos--;
    }

    *ecc_val = lcd_crc16(ecc_buf, pos);
    if (NULL == ecc_buf) {
        kfree(ecc_buf);
        ecc_buf = NULL;
    }
    return 0;
}

/* calculate lcd init code checksum */
static u16 cal_lcdinitcode_checksum(u8 *ptr , int length)
{
    /* CRC16 */
    u16 cfcs = 0;
    int i = 0;
    int j = 0;

    if (length % 2) {
        return 0xFFFF;
    }

    for (i = 0; i < length; i += 2) {
        cfcs ^= (((u16)ptr[i] << 8) + ptr[i + 1]);
        for (j = 0; j < 16; j++) {
            if (cfcs & 1) {
                cfcs = (u16)((cfcs >> 1) ^ ((1 << 15) + (1 << 10) + (1 << 3)));
            } else {
                cfcs >>= 1;
            }
        }
    }
    return cfcs;
}

/*
 * check_initial_code_valid - check initial code valid or not
 */
static int check_initial_code_valid(struct i2c_client *client, u8 *buf)
{
    int ret = 0;
    u16 initcode_ecc = 0;
    u16 buf_ecc = 0;
    u16 initcode_checksum = 0;
    u16 buf_checksum = 0;
    u16 hlic_len = 0;

    hlic_len = (u16)(((u16)buf[LIC_FS_H_OFF]) << 8) + buf[LIC_FS_L_OFF];
    FTS_INFO("host lcd init code len:0x%x", hlic_len);
    if ((hlic_len >= FTS_MAX_LEN_SECTOR) || (hlic_len <= FTS_MIN_LEN)) {
        FTS_ERROR("host lcd init code len(0x%x) is too large", hlic_len);
        return -EINVAL;
    }
    initcode_checksum = cal_lcdinitcode_checksum(buf + 4, hlic_len - 4);
    buf_checksum =
        ((u16)((u16)buf[LIC_CHECKSUM_H_OFF] << 8) + buf[LIC_CHECKSUM_L_OFF]);
    FTS_INFO("lcd init code calc checksum:0x%04x,0x%04x", initcode_checksum, buf_checksum);
    if (initcode_checksum != buf_checksum) {
        FTS_ERROR("Initial Code checksum fail");
        return -EINVAL;
    }

    ret = cal_lcdinitcode_ecc(buf, &initcode_ecc);
    if (ret < 0) {
        FTS_ERROR("lcd init code ecc calculate fail");
        return ret;
    }
    buf_ecc = ((u16)((u16)buf[LIC_ECC_H_OFF] << 8) + buf[LIC_ECC_L_OFF]);
    FTS_INFO("lcd init code cal ecc:%04x, %04x", initcode_ecc, buf_ecc);
    if (initcode_ecc != buf_ecc) {
        FTS_ERROR("Initial Code ecc check fail");
        return -EINVAL;
    }

    return 0;
}

static int fts_ft8739_get_hlic_ver(u8 *initcode)
{
    u8 *hlic_buf = initcode;
    u16 hlic_len = 0;
    u8 hlic_ver[2] = { 0 };

    hlic_len =
        (u16)(((u16)hlic_buf[LIC_FS_H_OFF]) << 8) + hlic_buf[LIC_FS_L_OFF];
    FTS_INFO("host lcd init code len:0x%x", hlic_len);
    if ((hlic_len >= FTS_MAX_LEN_SECTOR) || (hlic_len <= FTS_MIN_LEN)) {
        FTS_ERROR("host lcd init code len(0x%x) is too large", hlic_len);
        return -EINVAL;
    }

    hlic_ver[0] = hlic_buf[hlic_len];
    hlic_ver[1] = hlic_buf[hlic_len + 1];

    FTS_INFO("host lcd init code ver:0x%x 0x%x", hlic_ver[0], hlic_ver[1]);
    if (0xFF != (hlic_ver[0] + hlic_ver[1])) {
        FTS_ERROR("host lcd init code version check fail");
        return -EINVAL;
    }

    return hlic_ver[0];
}

static int fts_ft8739_upgrade_mode(
    struct i2c_client *client,
    enum FW_FLASH_MODE mode,
    u8 *buf,
    u32 len)
{
    int ret = 0;
    u32 start_addr = 0;
    u8 cmd[4] = { 0 };
    u32 delay = 0;
    int ecc_in_host = 0;
    int ecc_in_tp = 0;

    if ((NULL == buf) || (len < FTS_MIN_LEN)) {
        FTS_ERROR("buffer/len(%x) is invalid", len);
        return -EINVAL;
    }

    /* enter into upgrade environment */
    ret = fts_fwupg_enter_into_boot(client);
    if (ret < 0) {
        FTS_ERROR("enter into pramboot/bootloader fail,ret=%d", ret);
        goto fw_reset;
    }

    cmd[0] = FTS_CMD_FLASH_MODE;
    cmd[1] = FLASH_MODE_UPGRADE_VALUE;
    start_addr = 0; /* offset handle in pramboot */
    if (FLASH_MODE_LIC == mode) {
        /* lcd initial code upgrade */
        /* read replace 3-gamma yet  */
        cmd[1] = FLASH_MODE_LIC_VALUE;
    } else if (FLASH_MODE_PARAM == mode) {
        cmd[1] = FLASH_MODE_PARAM_VALUE;
    }
    FTS_INFO("flash mode:0x%02x, start addr=0x%04x", cmd[1], start_addr);
    ret = fts_i2c_write(client, cmd, 2);
    if (ret < 0) {
        FTS_ERROR("upgrade mode(09) cmd write fail");
        goto fw_reset;
    }

    delay = FTS_ERASE_SECTOR_DELAY * (len / FTS_MAX_LEN_SECTOR);
    ret = fts_fwupg_erase(client, delay);
    if (ret < 0) {
        FTS_ERROR("erase cmd write fail");
        goto fw_reset;
    }

    /* write app */
    ecc_in_host = fts_flash_write_buf(client, start_addr, buf, len, 1);
    if (ecc_in_host < 0 ) {
        FTS_ERROR("lcd initial code write fail");
        goto fw_reset;
    }

    /* ecc */
    ecc_in_tp = fts_fwupg_ecc_cal(client, start_addr, len);
    if (ecc_in_tp < 0 ) {
        FTS_ERROR("ecc read fail");
        goto fw_reset;
    }

    FTS_INFO("ecc in tp:%x, host:%x", ecc_in_tp, ecc_in_host);
    if (ecc_in_tp != ecc_in_host) {
        FTS_ERROR("ecc check fail");
        goto fw_reset;
    }

    FTS_INFO("upgrade success, reset to normal boot");
    ret = fts_fwupg_reset_in_boot(client);
    if (ret < 0) {
        FTS_ERROR("reset to normal boot fail");
    }

    msleep(400);
    return 0;

fw_reset:
    return -EIO;
}

/************************************************************************
* Name: fts_ft8739_upgrade
* Brief:
* Input:
* Output:
* Return: return 0 if success, otherwise return error code
***********************************************************************/
static int fts_ft8739_upgrade(struct i2c_client *client, u8 *buf, u32 len)
{
    int ret = 0;
    u8 *tmpbuf = NULL;
    u32 app_len = 0;

    FTS_INFO("fw app upgrade...");
    if (NULL == buf) {
        FTS_ERROR("fw buf is null");
        return -EINVAL;
    }

    if ((len < FTS_MIN_LEN) || (len > FTS_MAX_LEN_FILE)) {
        FTS_ERROR("fw buffer len(%x) fail", len);
        return -EINVAL;
    }

    app_len = len - upgrade_func_ft8739.appoff;
    tmpbuf = buf + upgrade_func_ft8739.appoff;
    ret = fts_ft8739_upgrade_mode(client, FLASH_MODE_APP, tmpbuf, app_len);
    if (ret < 0) {
        FTS_INFO("fw upgrade fail,reset to normal boot");
        if (fts_fwupg_reset_in_boot(client) < 0) {
            FTS_ERROR("reset to normal boot fail");
        }
        return ret;
    }

    return 0;
}

/************************************************************************
* Name: fts_ft8739_lic_upgrade
* Brief: FT8739 initial code upgrade
* Input:
* Output:
* Return: return 0 if success, otherwise return error code
***********************************************************************/
static int fts_ft8739_lic_upgrade(struct i2c_client *client, u8 *buf, u32 len)
{
    int ret = 0;
    u8 *tmpbuf = NULL;
    u32 lic_len = 0;

    FTS_INFO("lcd initial code upgrade...");
    if (NULL == buf) {
        FTS_ERROR("lcd initial code buffer is null");
        return -EINVAL;
    }

    if ((len < FTS_MIN_LEN) || (len > FTS_MAX_LEN_FILE)) {
        FTS_ERROR("lcd initial code buffer len(%x) fail", len);
        return -EINVAL;
    }

    ret = check_initial_code_valid(client, buf);
    if (ret < 0) {
        FTS_ERROR("initial code invalid, not upgrade lcd init code");
        return -EINVAL;
    }

    /* remalloc memory for initcode, need change content of initcode afterwise */
    lic_len = FTS_MAX_LEN_SECTOR;
    tmpbuf = kmalloc(lic_len, GFP_KERNEL);
    if (NULL == tmpbuf) {
        FTS_ERROR("initial code buf malloc fail");
        return -EINVAL;
    }
    memcpy(tmpbuf, buf, lic_len);

    ret = fts_ft8739_upgrade_mode(client, FLASH_MODE_LIC, tmpbuf, lic_len);
    if (ret < 0) {
        FTS_INFO("lcd initial code upgrade fail,reset to normal boot");
        if (fts_fwupg_reset_in_boot(client) < 0) {
            FTS_ERROR("reset to normal boot fail");
        }
        if (tmpbuf) {
            kfree(tmpbuf);
            tmpbuf = NULL;
        }
        return ret;
    }

    if (tmpbuf) {
        kfree(tmpbuf);
        tmpbuf = NULL;
    }
    return 0;
}

/************************************************************************
 * Name: fts_ft8739_param_upgrade
 * Brief:
 * Input: buf - all.bin
 *        len - len of all.bin
 * Output:
 * Return: return 0 if success, otherwise return error code
 ***********************************************************************/
static int fts_ft8739_param_upgrade(struct i2c_client *client, u8 *buf, u32 len)
{
    int ret = 0;
    u8 *tmpbuf = NULL;
    u32 param_length = 0;

    FTS_INFO("parameter configure upgrade...");
    if (NULL == buf) {
        FTS_ERROR("fw file buffer is null");
        return -EINVAL;
    }

    if ((len < FTS_MIN_LEN) || (len > FTS_MAX_LEN_FILE)) {
        FTS_ERROR("fw file buffer len(%x) fail", len);
        return -EINVAL;
    }

    tmpbuf = buf + upgrade_func_ft8739.paramcfgoff;
    param_length = len - upgrade_func_ft8739.paramcfgoff;
    ret = fts_ft8739_upgrade_mode(client, FLASH_MODE_PARAM, tmpbuf, param_length);
    if (ret < 0) {
        FTS_INFO("fw upgrade fail,reset to normal boot");
        if (fts_fwupg_reset_in_boot(client) < 0) {
            FTS_ERROR("reset to normal boot fail");
        }
        return ret;
    }

    return 0;
}

struct upgrade_func upgrade_func_ft8739 = {
    .ctype = {0x0E},
    .newmode = true,
    .fwveroff = 0x110E,
    .fwcfgoff = 0x0F80,
    .appoff = 0x1000,
    .licoff = 0x0000,
    .paramcfgoff = 0x11000,
    .paramcfgveroff = 0x11004,
    .pramboot_supported = true,
    .pramboot = pb_file_ft8739,
    .pb_length = sizeof(pb_file_ft8739),
    .hid_supported = false,
    .upgrade = fts_ft8739_upgrade,
    .get_hlic_ver = fts_ft8739_get_hlic_ver,
    .lic_upgrade = fts_ft8739_lic_upgrade,
    .param_upgrade = fts_ft8739_param_upgrade,
};

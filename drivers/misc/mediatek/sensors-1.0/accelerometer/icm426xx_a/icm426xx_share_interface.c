/* 
 * ICM426XX sensor driver
 * Copyright (C) 2018 Invensense, Inc.
 *
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

#include <hwmsensor.h>
#include "cust_acc.h"
#include "accel.h"
#include "icm426xx_register.h"
#include "icm426xx_share_interface.h"

#define RETRY_CNT 2

extern struct i2c_client *icm426xx_accel_i2c_client;

static DEFINE_MUTEX(icm426xx_accel_i2c_mutex);

static int icm426xx_i2c_read_register(struct i2c_client *client,
    u8 addr, u8 *data, u8 len)
{
    u8 beg = addr;
    int res = 0;
    struct i2c_msg msgs[2] = {{0}, {0} };
    int retry = RETRY_CNT;

    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = &beg;
    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = len;
    msgs[1].buf = data;
    if (!client) {
        return -EINVAL;
    } else if (len > C_I2C_FIFO_SIZE) {
        ACC_PR_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        return -EINVAL;
    }
    do {
        res = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
        if (res != 2) {
            ACC_PR_ERR("i2c_transfer error: (%d %p %d) %d\n",
                addr, data, len, res);
            res = -EIO;
        } else
            res = 0;
    } while (res !=0 && --retry > 0);
    return res;
}

static int icm426xx_i2c_write_register(struct i2c_client *client,
    u8 addr, u8 *data, u8 len)
{
    /*because address also occupies one byte,
    the maximum length for write is 7 bytes*/
    int idx, num;
    int res = 0;
    char buf[C_I2C_FIFO_SIZE];
    int retry = RETRY_CNT;

    if (!client) {
        return -EINVAL;
    } else if (len >= C_I2C_FIFO_SIZE) {
        ACC_PR_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        return -EINVAL;
    }
    num = 0;
    buf[num++] = addr;
    for (idx = 0; idx < len; idx++)
        buf[num++] = data[idx];
    do {
        res = i2c_master_send(client, buf, num);
        if (res < 0) {
            ACC_PR_ERR("send command error!!\n");
            return -EFAULT;
        } else
            res = 0;
    } while (res !=0 && --retry > 0);
    return res;
}

int icm426xx_share_read_register(u8 addr, u8 *data, u8 len)
{
    int ret;

    mutex_lock(&icm426xx_accel_i2c_mutex);
    ret = icm426xx_i2c_read_register(icm426xx_accel_i2c_client,
        addr, data, len);
    mutex_unlock(&icm426xx_accel_i2c_mutex);
    return ret;
}
EXPORT_SYMBOL(icm426xx_share_read_register);

int icm426xx_share_write_register(u8 addr, u8 *data, u8 len)
{
    int ret;

    mutex_lock(&icm426xx_accel_i2c_mutex);
    ret = icm426xx_i2c_write_register(icm426xx_accel_i2c_client,
        addr, data, len);
    mutex_unlock(&icm426xx_accel_i2c_mutex);
    return ret;
}
EXPORT_SYMBOL(icm426xx_share_write_register);

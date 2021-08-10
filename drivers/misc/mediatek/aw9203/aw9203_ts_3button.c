/*
 * aw9203.c   aw9203 touch key module
 *
 * Version: v1.2.4
 *
 * Copyright (c) 2017 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
//#include <linux/wakelock.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/fb.h>
#include <linux/notifier.h>
#include "aw9203.h"
#include "aw9203_reg.h"
#include "aw9203_para.h"

/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW9203_I2C_NAME "aw9203_ts"

#define AW9203_VERSION "v1.2.4"

#define AW_I2C_RETRIES 2
#define AW_I2C_RETRY_DELAY 2
#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 2

#define AW9203_DEBUG_enable  0

#define AW9203_DEBUG(format, args...) do { \
	if (AW9203_DEBUG_enable) \
	{\
		printk(KERN_WARNING format, ##args);\
	} \
} while (0)
	
struct aw9203 *g_aw9203 = NULL;

/******************************************************
 *
 * variable
 *
 ******************************************************/

 /******************************************************
 *
 * aw9203 i2c write/read
 *
 ******************************************************/
static int i2c_write(struct aw9203 *aw9203,
		     unsigned char addr, unsigned int reg_data)
{
	int ret;
	u8 wbuf[512] = { 0 };

	struct i2c_msg msgs[] = {
		{
		 .addr = aw9203->i2c->addr,
		 .flags = 0,
		 .len = 3,
		 .buf = wbuf,
		 },
	};

	wbuf[0] = addr;
	wbuf[1] = (unsigned char)((reg_data & 0xff00) >> 8);
	wbuf[2] = (unsigned char)(reg_data & 0x00ff);

	ret = i2c_transfer(aw9203->i2c->adapter, msgs, 1);
	if (ret < 0)
		pr_err("%s: i2c write error: %d\n", __func__, ret);

	return ret;
}

static int i2c_read(struct aw9203 *aw9203,
		    unsigned char addr, unsigned int *reg_data)
{
	int ret;
	unsigned char rbuf[512] = { 0 };
	unsigned int get_data;

	struct i2c_msg msgs[] = {
		{
		 .addr = aw9203->i2c->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = &addr,
		 },
		{
		 .addr = aw9203->i2c->addr,
		 .flags = I2C_M_RD,
		 .len = 2,
		 .buf = rbuf,
		 },
	};

	ret = i2c_transfer(aw9203->i2c->adapter, msgs, 2);
	if (ret < 0) {
		pr_err("%s: i2c read error: %d\n", __func__, ret);
		return ret;
	}

	get_data = (unsigned int)(rbuf[0] & 0x00ff);
	get_data <<= 8;
	get_data |= (unsigned int)rbuf[1];

	*reg_data = get_data;

	return ret;
}

static int aw9203_i2c_write(struct aw9203 *aw9203,
			    unsigned char reg_addr, unsigned int reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_write(aw9203, reg_addr, reg_data);
		if (ret < 0) {
			pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt,
			       ret);
		} else {
			break;
		}
		cnt++;
	}

	return ret;
}

static int aw9203_i2c_read(struct aw9203 *aw9203,
			   unsigned char reg_addr, unsigned int *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_read(aw9203, reg_addr, reg_data);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt,
			       ret);
		} else {
			break;
		}
		cnt++;
	}
	return ret;
}

#if 0
static int aw9203_i2c_write_bits(struct aw9203 *aw9203,
				 unsigned char reg_addr, unsigned int mask,
				 unsigned int reg_data)
{
	unsigned int reg_val;

	aw9203_i2c_read(aw9203, reg_addr, &reg_val);
	reg_val &= mask;
	reg_val |= reg_data;
	aw9203_i2c_write(aw9203, reg_addr, reg_val);

	return 0;
}

static int aw9203_i2c_writes(struct aw9203 *aw9203,
			     unsigned char reg_addr, unsigned char *buf,
			     unsigned int len)
{
	int ret = -1;
	unsigned char *data;

	data = kmalloc(len + 1, GFP_KERNEL);
	if (data == NULL) {
		pr_err("%s: can not allocate memory\n", __func__);
		return -ENOMEM;
	}

	data[0] = reg_addr;
	memcpy(&data[1], buf, len);

	ret = i2c_master_send(aw9203->i2c, data, len + 1);
	if (ret < 0)
		pr_err("%s: i2c master send error\n", __func__);

	kfree(data);

	return ret;
}
#endif

/******************************************************
 *
 * auto cali
 *
 ******************************************************/
static int aw9203_auto_cali(struct aw9203 *aw9203)
{
	unsigned char i;
	unsigned char cali_dir[6];

	unsigned int buf[6];
	unsigned int ofr_cfg[6];
	unsigned int sen_num;
	unsigned int reg_val;

	if (aw9203->cali_num == 0) {
		aw9203_i2c_read(aw9203, AW9203_REG_OFR1, &reg_val);
		ofr_cfg[0] = reg_val;
		aw9203_i2c_read(aw9203, AW9203_REG_OFR2, &reg_val);
		ofr_cfg[1] = reg_val;
		aw9203_i2c_read(aw9203, AW9203_REG_OFR3, &reg_val);
		ofr_cfg[2] = reg_val;
	} else {
		for (i = 0; i < 3; i++)
			ofr_cfg[i] = aw9203->old_ofr_cfg[i];
	}

	aw9203_i2c_write(aw9203, AW9203_REG_MCR, 0x3);
	for (i = 0; i < 6; i++) {
		aw9203_i2c_read(aw9203, AW9203_REG_KDATA0 + i, &reg_val);
		buf[i] = reg_val;
	}
	aw9203_i2c_read(aw9203, AW9203_REG_SLPR, &sen_num);

	for (i = 0; i < 6; i++)
		aw9203->rawdata_sum[i] = (aw9203->cali_cnt == 0) ?
		    (0) : (aw9203->rawdata_sum[i] + buf[i]);

	if (aw9203->cali_cnt == 4) {
		for (i = 0; i < 6; i++) {
			if ((sen_num & (1 << i)) == 0) {	/* sensor used */
				if ((aw9203->rawdata_sum[i] >> 2) <
				    CALI_RAW_MIN) {
					if ((i % 2)
					    && ((ofr_cfg[i >> 1] & 0xFF00) ==
						0x1000)) {
						/* 0x10** -> 0x00** */
						ofr_cfg[i >> 1] =
						    ofr_cfg[i >> 1] & 0x00FF;
						cali_dir[i] = 2;
					} else if ((i % 2)
						   &&
						   ((ofr_cfg[i >> 1] & 0xFF00)
						    == 0x0000)) {
						/* 0x00**    no calibration */
						cali_dir[i] = 0;
					} else if (((i % 2) == 0)
						   &&
						   ((ofr_cfg[i >> 1] & 0x00FF)
						    == 0x0010)) {
						/* 0x**10 -> 0x**00 */
						ofr_cfg[i >> 1] =
						    ofr_cfg[i >> 1] & 0xFF00;
						cali_dir[i] = 2;
					} else if (((i % 2) == 0)
						   &&
						   ((ofr_cfg[i >> 1] & 0x00FF)
						    == 0x0000)) {
						/* 0x**00 no calibration */
						cali_dir[i] = 0;
					} else {
						ofr_cfg[i >> 1] =
						    ofr_cfg[i >> 1] -
						    ((i % 2) ? (1 << 8) : 1);
						cali_dir[i] = 2;
					}
				} else if ((aw9203->rawdata_sum[i] >> 2) >
					   CALI_RAW_MAX) {
					if ((i % 2)
					    && ((ofr_cfg[i >> 1] & 0xFF00) ==
						0x1F00)) {
						/* 0x1F** no calibration */
						cali_dir[i] = 0;
					} else if (((i % 2) == 0)
						   &&
						   ((ofr_cfg[i >> 1] & 0x00FF)
						    == 0x001F)) {
						/* 0x**1F no calibration */
						cali_dir[i] = 0;
					} else if ((i % 2)
						   &&
						   ((ofr_cfg[i >> 1] & 0xFF00)
						    == 0x0000)) {
						/* 0x00** -> 0x1000 */
						ofr_cfg[i >> 1] =
						    ofr_cfg[i >> 1] | 0x1000;
						cali_dir[i] = 1;
					} else if (((i % 2) == 0)
						   &&
						   ((ofr_cfg[i >> 1] & 0x00FF)
						    == 0x0000)) {
						/* 0x**00 -> 0x**10 */
						ofr_cfg[i >> 1] =
						    ofr_cfg[i >> 1] | 0x0010;
						cali_dir[i] = 1;
					} else {
						ofr_cfg[i >> 1] =
						    ofr_cfg[i >> 1] +
						    ((i % 2) ? (1 << 8) : 1);
						cali_dir[i] = 1;
					}
				} else {
					cali_dir[i] = 0;
				}

				if (aw9203->cali_num > 0) {
					if (cali_dir[i] !=
					    aw9203->old_cali_dir[i]) {
						cali_dir[i] = 0;
						ofr_cfg[i >> 1] =
						    aw9203->old_ofr_cfg[i >> 1];
					}
				}
			}
		}

		aw9203->cali_flag = 0;
		for (i = 0; i < 6; i++) {
			if ((sen_num & (1 << i)) == 0) {	/* sensor used */
				if (cali_dir[i] != 0)
					aw9203->cali_flag = 1;
			}
		}
		if ((aw9203->cali_flag == 0) && (aw9203->cali_num == 0))
			aw9203->cali_used = 0;
		else
			aw9203->cali_used = 1;

		if (aw9203->cali_flag == 0) {
			aw9203->cali_num = 0;
			aw9203->cali_cnt = 0;
			return 0;
		}
		/* touch disbale */
		aw9203_i2c_read(aw9203, AW9203_REG_GCR, &reg_val);
		reg_val &= 0xFFFD;
		aw9203_i2c_write(aw9203, AW9203_REG_GCR, reg_val);
		for (i = 0; i < 3; i++) {
			aw9203_i2c_write(aw9203, AW9203_REG_OFR1 + i,
					 ofr_cfg[i]);
		}
		aw9203_i2c_read(aw9203, AW9203_REG_GCR, &reg_val);
		reg_val |= 0x0002;
		aw9203_i2c_write(aw9203, AW9203_REG_GCR, reg_val);

		if (aw9203->cali_num == (CALI_NUM - 1)) {	/* no calibration */
			aw9203->cali_flag = 0;
			aw9203->cali_num = 0;
			aw9203->cali_cnt = 0;
			return 0;
		}

		for (i = 0; i < 6; i++)
			aw9203->old_cali_dir[i] = cali_dir[i];

		for (i = 0; i < 3; i++)
			aw9203->old_ofr_cfg[i] = ofr_cfg[i];

		aw9203->cali_num++;
	}

	if (aw9203->cali_cnt < 4)
		aw9203->cali_cnt++;
	else
		aw9203->cali_cnt = 0;

	return 0;
}

/******************************************************
 *
 * aw9203 cfg
 *
 ******************************************************/
static void aw9203_cali_init(struct aw9203 *aw9203)
{
	aw9203->cali_flag = 1;
	aw9203->cali_num = 0;
	aw9203->cali_cnt = 0;
}

static void aw9203_normal_mode_init(struct aw9203 *aw9203)
{
	unsigned int reg_val;

	aw9203_cali_init(aw9203);

	/* touch disbale */
	aw9203_i2c_read(aw9203, AW9203_REG_GCR, &reg_val);
	reg_val &= 0xFFFD;
	aw9203_i2c_write(aw9203, AW9203_REG_GCR, reg_val);	/* disable chip */

	/* cap-touch config */
	aw9203_i2c_write(aw9203, AW9203_REG_SLPR, AW9203_NORMAL_SLPR);	/* touch key enable */
	aw9203_i2c_write(aw9203, AW9203_REG_SCFG1, AW9203_NORMAL_SCFG1);	/* scan time setting */
	aw9203_i2c_write(aw9203, AW9203_REG_SCFG2, AW9203_NORMAL_SCFG2);	/* bit0~3 is sense seting */

	aw9203_i2c_write(aw9203, AW9203_REG_OFR1, AW9203_NORMAL_OFR1);	/* offset */
	aw9203_i2c_write(aw9203, AW9203_REG_OFR2, AW9203_NORMAL_OFR2);	/* offset */
	aw9203_i2c_write(aw9203, AW9203_REG_OFR3, AW9203_NORMAL_OFR3);	/* offset */

	aw9203_i2c_write(aw9203, AW9203_REG_THR2, AW9203_NORMAL_THR2);	/* s1 press thred setting */
	aw9203_i2c_write(aw9203, AW9203_REG_THR3, AW9203_NORMAL_THR3);	/* s2 press thred setting */
	aw9203_i2c_write(aw9203, AW9203_REG_THR4, AW9203_NORMAL_THR4);	/* s3 press thred setting */

	aw9203_i2c_write(aw9203, AW9203_REG_SETCNT, AW9203_NORMAL_SETCNT);	/* debounce */
	aw9203_i2c_write(aw9203, AW9203_REG_BLCTH, AW9203_NORMAL_BLCTH);	/* base trace rate */

	aw9203_i2c_write(aw9203, AW9203_REG_AKSR, AW9203_NORMAL_AKSR);	/* aks */
#ifndef AW_AUTO_CALI
	aw9203_i2c_write(aw9203, AW9203_REG_INTER, AW9203_NORMAL_INTER);	/* signel click interrupt */
#else
	aw9203_i2c_write(aw9203, AW9203_REG_INTER, 0x0080);	/* frame interrupt */
#endif

	aw9203_i2c_write(aw9203, AW9203_REG_MPTR, AW9203_NORMAL_MPTR);	/* long press time */
	aw9203_i2c_write(aw9203, AW9203_REG_GDTR, AW9203_NORMAL_GDTR);	/* gesture time setting */
	aw9203_i2c_write(aw9203, AW9203_REG_GDCFGR, AW9203_NORMAL_GDCFGR);	/* gesture key select */
	aw9203_i2c_write(aw9203, AW9203_REG_TAPR1, AW9203_NORMAL_TAPR1);	/* double click 1 */
	aw9203_i2c_write(aw9203, AW9203_REG_TAPR2, AW9203_NORMAL_TAPR2);	/* double click 2 */
	aw9203_i2c_write(aw9203, AW9203_REG_TDTR, AW9203_NORMAL_TDTR);	/* double click time */
	aw9203_i2c_write(aw9203, AW9203_REG_IDLECR, AW9203_NORMAL_IDLECR);	/* IDLE time setting */

#ifndef AW_AUTO_CALI
	aw9203_i2c_write(aw9203, AW9203_REG_GIER, AW9203_NORMAL_GIER);	/* gesture and double click enable */
#else
	aw9203_i2c_write(aw9203, AW9203_REG_GIER, 0x0000);	/* gesture and double click disable */
#endif

	/* chip enable */
	aw9203_i2c_read(aw9203, AW9203_REG_GCR, &reg_val);
	reg_val |= 0x0002;
	aw9203_i2c_write(aw9203, AW9203_REG_GCR, reg_val);	/* disable chip */
}

/******************************************************
 *
 * irq
 *
 ******************************************************/
static void aw9203_interrupt_clear(struct aw9203 *aw9203)
{
	unsigned int reg_val;

	AW9203_DEBUG("%s enter\n", __func__);
	aw9203_i2c_read(aw9203, AW9203_REG_ISR, &reg_val);
	AW9203_DEBUG("%s: reg ISR=0x%x\n", __func__, reg_val);
}

static void aw9203_interrupt_setup(struct aw9203 *aw9203)
{
	AW9203_DEBUG("%s enter\n", __func__);
	aw9203_interrupt_clear(aw9203);
}

static irqreturn_t aw9203_irq(int irq, void *data)
{
	struct aw9203 *aw9203 = data;
	unsigned int reg_val;
	unsigned int reg_kst;
	unsigned int reg_kisr;

	AW9203_DEBUG("%s enter\n", __func__);

#ifdef AW_AUTO_CALI
	if (aw9203->cali_flag) {
		aw9203_auto_cali(aw9203);
		if (aw9203->cali_flag == 0) {
			if (aw9203->cali_used) {
				aw9203_i2c_read(aw9203, AW9203_REG_GCR,
						&reg_val);
				reg_val &= 0xFFFD;
				aw9203_i2c_write(aw9203, AW9203_REG_GCR,
						 reg_val);
			}
			aw9203_i2c_write(aw9203, AW9203_REG_INTER,
					 AW9203_NORMAL_INTER);
			aw9203_i2c_write(aw9203, AW9203_REG_GIER,
					 AW9203_NORMAL_GIER);
			if (aw9203->cali_used) {
				aw9203_i2c_read(aw9203, AW9203_REG_GCR,
						&reg_val);
				reg_val |= 0x0002;
				aw9203_i2c_write(aw9203, AW9203_REG_GCR,
						 reg_val);
			}
		}
		aw9203_interrupt_clear(aw9203);
		return IRQ_HANDLED;
	}
#endif

	aw9203_i2c_read(aw9203, AW9203_REG_ISR, &reg_kisr);
	aw9203_i2c_read(aw9203, AW9203_REG_AKSST, &reg_kst);
	AW9203_DEBUG("%s: kisr=0x%04x, kst=0x%04x\n", __func__, reg_kisr, reg_kst);

	if (reg_kisr & 0x0010) 
	{
		if (reg_kst & 0x0010)
		{
			AW9203_DEBUG("aw9203 KEY_HOMEPAGE down...\n");
			input_report_key(aw9203->input, KEY_HOMEPAGE, 1);
		}
		else
		{
			AW9203_DEBUG("aw9203 KEY_HOMEPAGE up...\n");
			input_report_key(aw9203->input, KEY_HOMEPAGE, 0);
		}
		input_sync(aw9203->input);
	} 
	else if (reg_kisr & 0x0008) 
	{
		if (reg_kst & 0x0008)
		{
			AW9203_DEBUG("aw9203 KEY_MENU down...\n");
			input_report_key(aw9203->input, KEY_MENU, 1);
		}
		else
		{
			AW9203_DEBUG("aw9203 KEY_MENU up...\n");
			input_report_key(aw9203->input, KEY_MENU, 0);
		}
		input_sync(aw9203->input);
	} 
	else if (reg_kisr & 0x0004) 
	{
		if (reg_kst & 0x0004)
		{
			AW9203_DEBUG("aw9203 KEY_BACK down...\n");
			input_report_key(aw9203->input, KEY_BACK, 1);
		}
		else
		{
			AW9203_DEBUG("aw9203 KEY_BACK up...\n");
			input_report_key(aw9203->input, KEY_BACK, 0);
		}
		input_sync(aw9203->input);
	}

	aw9203_interrupt_clear(aw9203);
	AW9203_DEBUG("%s exit\n", __func__);

	return IRQ_HANDLED;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw9203_parse_dt(struct device *dev, struct aw9203 *aw9203,
			   struct device_node *np)
{
	aw9203->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw9203->reset_gpio < 0)
		dev_err(dev, "%s: no reset gpio provided.\n", __func__);
	else
		dev_info(dev, "%s: reset gpio provided ok.\n", __func__);

	aw9203->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (aw9203->irq_gpio < 0)
		dev_err(dev, "%s: no irq gpio provided.\n", __func__);
	else
		dev_info(dev, "%s: irq gpio provided ok.\n", __func__);

	return 0;
}

int aw9203_hw_reset(struct aw9203 *aw9203)
{
	AW9203_DEBUG("%s enter\n", __func__);

	if (aw9203 && gpio_is_valid(aw9203->reset_gpio)) {
		gpio_set_value_cansleep(aw9203->reset_gpio, 0);
		usleep_range(2000, 2500);
		gpio_set_value_cansleep(aw9203->reset_gpio, 1);
		usleep_range(5000, 5500);
	} else {
		dev_err(aw9203->dev, "%s:  failed\n", __func__);
	}
	return 0;
}

int aw9203_hw_off(struct aw9203 *aw9203)
{
	AW9203_DEBUG("%s enter\n", __func__);

	if (aw9203 && gpio_is_valid(aw9203->reset_gpio)) {
		gpio_set_value_cansleep(aw9203->reset_gpio, 0);
		usleep_range(1000, 1500);
	} else {
		dev_err(aw9203->dev, "%s:  failed\n", __func__);
	}
	return 0;
}

/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
int aw9203_read_chipid(struct aw9203 *aw9203)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned int reg_val = 0;

	while (cnt < AW_READ_CHIPID_RETRIES) {
		ret = aw9203_i2c_read(aw9203, AW9203_REG_RSTR, &reg_val);
		if (ret < 0) {
			dev_err(aw9203->dev,
				"%s: failed to read register AW9203_REG_RSTR: %d\n",
				__func__, ret);
		}
		switch (reg_val) {
		case 0xb223:
			AW9203_DEBUG("%s aw9203 detected\n", __func__);
			/*aw9203->flags |= AW9203_FLAG_SKIP_INTERRUPTS; */
			return 0;
		default:
			AW9203_DEBUG("%s unsupported device revision (0x%x)\n",
				__func__, reg_val);
			break;
		}
		cnt++;
		usleep_range(AW_READ_CHIPID_RETRY_DELAY * 1000,
			     AW_READ_CHIPID_RETRY_DELAY * 1000 + 500);
	}

	return -EINVAL;
}

/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t aw9203_i2c_reg_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct aw9203 *aw9203 = dev_get_drvdata(dev);

	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw9203_i2c_write(aw9203, (unsigned char)databuf[0],
				 (unsigned int)databuf[1]);
	}

	return count;
}

static ssize_t aw9203_i2c_reg_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct aw9203 *aw9203 = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned char i = 0;
	unsigned int reg_val = 0;

	for (i = 0; i < AW9203_REG_MAX; i++) {
		if (!(aw9203_reg_access[i] & REG_RD_ACCESS))
			continue;
		aw9203_i2c_read(aw9203, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%02x=0x%04x\n", i, reg_val);
	}
	return len;
}

static ssize_t aw9203_i2c_rawdata_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	/*struct aw9203 *aw9203 = dev_get_drvdata(dev); */
	return count;
}

static ssize_t aw9203_i2c_rawdata_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct aw9203 *aw9203 = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned int reg_val = 0;
	unsigned char i;

	mutex_lock(&aw9203->lock);
	aw9203_i2c_write(aw9203, AW9203_REG_MCR, 0x0003);
	len += snprintf(buf + len, PAGE_SIZE - len, "rawdata:\n");
	for (i = 0; i < 6; i++) {
		aw9203_i2c_read(aw9203, AW9203_REG_KDATA0 + i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len, "%d, ", reg_val);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	mutex_unlock(&aw9203->lock);

	return len;
}

static ssize_t aw9203_i2c_base_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	/*struct aw9203 *aw9203 = dev_get_drvdata(dev); */
	return count;
}

static ssize_t aw9203_i2c_base_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct aw9203 *aw9203 = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned int reg_val = 0;
	unsigned char i;

	mutex_lock(&aw9203->lock);
	aw9203_i2c_write(aw9203, AW9203_REG_MCR, 0x0002);
	len += snprintf(buf + len, PAGE_SIZE - len, "baseline:\n");
	for (i = 0; i < 6; i++) {
		aw9203_i2c_read(aw9203, AW9203_REG_KDATA0 + i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len, "%d, ", reg_val);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	mutex_unlock(&aw9203->lock);

	return len;
}

static ssize_t aw9203_i2c_delta_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	/*struct aw9203 *aw9203 = dev_get_drvdata(dev); */
	return count;
}

static ssize_t aw9203_i2c_delta_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct aw9203 *aw9203 = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned int reg_val = 0;
	int ireg_val;
	unsigned char i;

	mutex_lock(&aw9203->lock);
	aw9203_i2c_write(aw9203, AW9203_REG_MCR, 0x0001);
	len += snprintf(buf + len, PAGE_SIZE - len, "delta:\n");
	for (i = 0; i < 6; i++) {
		aw9203_i2c_read(aw9203, AW9203_REG_KDATA0 + i, &reg_val);
		if (reg_val & 0x8000) {
			ireg_val = reg_val | 0xffff0000;
			len += snprintf(buf + len, PAGE_SIZE - len,
					"%d, ", ireg_val);
			continue;
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "%d, ", reg_val);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	mutex_unlock(&aw9203->lock);

	return len;
}

static ssize_t aw9203_i2c_irqstate_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	/*struct aw9203 *aw9203 = dev_get_drvdata(dev); */
	return count;
}

static ssize_t aw9203_i2c_irqstate_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct aw9203 *aw9203 = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned int reg_val = 0;
	unsigned int tmp_val = 0;
	unsigned char i;

	len += snprintf(buf + len, PAGE_SIZE - len, "touch:\n");
	aw9203_i2c_read(aw9203, AW9203_REG_AKSST, &reg_val);
	for (i = 0; i < 6; i++) {
		tmp_val = (reg_val >> i) & 0x01;
		len += snprintf(buf + len, PAGE_SIZE - len, "%d, ", tmp_val);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "gesture:\n");
	aw9203_i2c_read(aw9203, AW9203_REG_GISR, &reg_val);
	for (i = 0; i < 5; i++) {
		tmp_val = (reg_val >> i) & 0x01;
		len += snprintf(buf + len, PAGE_SIZE - len, "%d, ", tmp_val);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");

	return len;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, aw9203_i2c_reg_show,
		   aw9203_i2c_reg_store);
static DEVICE_ATTR(rawdata, S_IWUSR | S_IRUGO, aw9203_i2c_rawdata_show,
		   aw9203_i2c_rawdata_store);
static DEVICE_ATTR(base, S_IWUSR | S_IRUGO, aw9203_i2c_base_show,
		   aw9203_i2c_base_store);
static DEVICE_ATTR(delta, S_IWUSR | S_IRUGO, aw9203_i2c_delta_show,
		   aw9203_i2c_delta_store);
static DEVICE_ATTR(irqstate, S_IWUSR | S_IRUGO, aw9203_i2c_irqstate_show,
		   aw9203_i2c_irqstate_store);

static struct attribute *aw9203_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_rawdata.attr,
	&dev_attr_base.attr,
	&dev_attr_delta.attr,
	&dev_attr_irqstate.attr,
	NULL
};

static struct attribute_group aw9203_attribute_group = {
	.attrs = aw9203_attributes
};

static int aw9203_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = NULL;
	int blank;
	//int err = 0;
	evdata = data;
	/* If we aren't interested in this event, skip it immediately ... */
	if (event != FB_EVENT_BLANK)
		return 0;

	blank = *(int *)evdata->data;
	switch (blank) {
		case FB_BLANK_UNBLANK:
			AW9203_DEBUG("[%s] screen on\n",__func__);
			if(g_aw9203 != NULL)
			{
				if(g_aw9203->irq_is_disable && (g_aw9203->aw9203_irq_t > 0))
				{
					g_aw9203->irq_is_disable = 0;
					enable_irq(g_aw9203->aw9203_irq_t);
					AW9203_DEBUG("[%s] screen on enable_irq\n",__func__);
				}
			}
			break;
		case FB_BLANK_POWERDOWN:
			AW9203_DEBUG("[%s] screen off\n",__func__);
			if(g_aw9203 != NULL)
			{
				if((g_aw9203->irq_is_disable == 0) && (g_aw9203->aw9203_irq_t > 0))
				{
					g_aw9203->irq_is_disable = 1;
					disable_irq(g_aw9203->aw9203_irq_t);
					AW9203_DEBUG("[%s] screen on disable_irq\n",__func__);
				}
			}
			break;
		default:
			break;
	}
	
	return 0;
}
static struct notifier_block aw9203_fb_notifier = {
	.notifier_call = aw9203_fb_notifier_callback,
};

/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw9203_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct aw9203 *aw9203;
	struct device_node *np = i2c->dev.of_node;
	struct input_dev *input_dev;
	int irq_flags;
	int ret = -1;

	AW9203_DEBUG("%s enter\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	aw9203 = devm_kzalloc(&i2c->dev, sizeof(struct aw9203), GFP_KERNEL);
	if (aw9203 == NULL)
		return -ENOMEM;
	
	g_aw9203 = aw9203;
	aw9203->dev = &i2c->dev;
	aw9203->i2c = i2c;

	i2c_set_clientdata(i2c, aw9203);

	mutex_init(&aw9203->lock);

	/* aw9203 int */
	if (np) {
		ret = aw9203_parse_dt(&i2c->dev, aw9203, np);
		if (ret) {
			dev_err(&i2c->dev,
				"%s: failed to parse device tree node\n",
				__func__);
			goto err;
		}
	} else {
		aw9203->reset_gpio = -1;
		aw9203->irq_gpio = -1;
	}

	if (gpio_is_valid(aw9203->reset_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw9203->reset_gpio,
					    GPIOF_OUT_INIT_LOW, "aw9203_rst");
		if (ret) {
			dev_err(&i2c->dev, "%s: rst request failed\n",
				__func__);
			goto err;
		}
	}

	if (gpio_is_valid(aw9203->irq_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw9203->irq_gpio,
					    GPIOF_DIR_IN, "aw9203_int");
		if (ret) {
			dev_err(&i2c->dev, "%s: int request failed\n",
				__func__);
			goto err;
		}
	}

	/* aw9203 hardware reset */
	aw9203_hw_reset(aw9203);

	/* aw9203 chip id */
	ret = aw9203_read_chipid(aw9203);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: aw9203_read_chipid failed ret=%d\n",
			__func__, ret);
		goto err_id;
	}

	/* aw9203 irq */
	if (gpio_is_valid(aw9203->irq_gpio) &&
	    !(aw9203->flags & AW9203_FLAG_SKIP_INTERRUPTS)) {
		/* register irq handler */
		aw9203_interrupt_setup(aw9203);
		
		aw9203->aw9203_irq_t = gpio_to_irq(aw9203->irq_gpio);
		
		AW9203_DEBUG("%s aw9203->aw9203_irq_t = %d\n", __func__,aw9203->aw9203_irq_t);
		
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev,
						aw9203->aw9203_irq_t,
						NULL, aw9203_irq, irq_flags,
						"aw9203", aw9203);
		if (ret != 0) {
			dev_err(&i2c->dev, "%s: failed to request IRQ %d: %d\n",
				__func__, gpio_to_irq(aw9203->irq_gpio), ret);
			goto err_irq;
		}
	} else {
		dev_info(&i2c->dev, "%s skipping IRQ registration\n", __func__);
		/* disable feature support if gpio was invalid */
		aw9203->flags |= AW9203_FLAG_SKIP_INTERRUPTS;
	}

	dev_set_drvdata(&i2c->dev, aw9203);

	/* input device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		ret = -ENOMEM;
		dev_err(&i2c->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(KEY_MENU, input_dev->keybit);
	__set_bit(KEY_HOMEPAGE, input_dev->keybit);
	__set_bit(KEY_BACK, input_dev->keybit);

	input_dev->name = AW9203_I2C_NAME;
	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&i2c->dev,
			"%s: failed to register input device: %s\n",
			__func__, dev_name(&i2c->dev));
		goto exit_input_register_device_failed;
	}
	aw9203->input = input_dev;

	/* attribute */
	ret = sysfs_create_group(&i2c->dev.kobj, &aw9203_attribute_group);
	if (ret < 0) {
		dev_info(&i2c->dev, "%s error creating sysfs attr files\n",
			 __func__);
		goto err_sysfs;
	}

	aw9203_normal_mode_init(aw9203);
	
	ret = fb_register_client(&aw9203_fb_notifier);
		if (ret)
			pr_debug("[%s] failed to register aw9203_fb_notifier %d\n", __func__, ret);

	AW9203_DEBUG("%s probe completed successfully!\n", __func__);

	return 0;

 err_sysfs:
	input_unregister_device(input_dev);
 exit_input_register_device_failed:
	input_free_device(input_dev);
 exit_input_dev_alloc_failed:
 err_irq:
 err_id:
	if (gpio_is_valid(aw9203->irq_gpio))
		devm_gpio_free(&i2c->dev, aw9203->irq_gpio);
 err:
	return ret;
}

static int aw9203_i2c_remove(struct i2c_client *i2c)
{
	struct aw9203 *aw9203 = i2c_get_clientdata(i2c);

	AW9203_DEBUG("%s enter\n", __func__);

	mutex_destroy(&aw9203->lock);

	sysfs_remove_group(&i2c->dev.kobj, &aw9203_attribute_group);

	if (gpio_is_valid(aw9203->irq_gpio))
		devm_gpio_free(&i2c->dev, aw9203->irq_gpio);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int aw9203_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct aw9203 *aw9203 = i2c_get_clientdata(client);

	aw9203_hw_off(aw9203);

	return 0;
}

static int aw9203_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct aw9203 *aw9203 = i2c_get_clientdata(client);

	aw9203_hw_reset(aw9203);
	aw9203_normal_mode_init(aw9203);

	return 0;
}

static SIMPLE_DEV_PM_OPS(aw9203_pm, aw9203_suspend, aw9203_resume);
#endif

static const struct i2c_device_id aw9203_i2c_id[] = {
	{AW9203_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, aw9203_i2c_id);

static const struct of_device_id aw9203_dt_match[] = {
	{.compatible = "awinic,aw9203_ts"},
	{},
};

static struct i2c_driver aw9203_i2c_driver = {
	.driver = {
		   .name = AW9203_I2C_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(aw9203_dt_match),
#ifdef CONFIG_PM_SLEEP
		   .pm = &aw9203_pm,
#endif
		   },
	.probe = aw9203_i2c_probe,
	.remove = aw9203_i2c_remove,
	.id_table = aw9203_i2c_id,
};

static int __init aw9203_i2c_init(void)
{
	int ret = 0;

	AW9203_DEBUG("aw9203 driver version %s\n", AW9203_VERSION);

	ret = i2c_add_driver(&aw9203_i2c_driver);
	if (ret) {
		pr_err("fail to add aw9203 device into i2c\n");
		return ret;
	}

	return 0;
}

module_init(aw9203_i2c_init);

static void __exit aw9203_i2c_exit(void)
{
	i2c_del_driver(&aw9203_i2c_driver);
}

module_exit(aw9203_i2c_exit);

MODULE_DESCRIPTION("AW9203 Touch Key Driver");
MODULE_LICENSE("GPL v2");

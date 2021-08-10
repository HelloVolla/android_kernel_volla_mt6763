/*
 * Copyright (C) 2017 MediaTek Inc.
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

#ifndef _ACCDET_H_
#define _ACCDET_H_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>

#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/input.h>
#include <linux/time.h>

#include <linux/string.h>


#define ACCDET_DEVNAME "accdet"

#define NO_KEY	(0x0)
#define UP_KEY	(0x01)
#define MD_KEY	(0x02)
#define DW_KEY	(0x04)
#define AS_KEY	(0x08)

#define HEADSET_MODE_1	(1)
#define HEADSET_MODE_2	(2)
#define HEADSET_MODE_6	(6)

/* IOCTL */
#define ACCDET_IOC_MAGIC 'A'
#define ACCDET_INIT _IO(ACCDET_IOC_MAGIC, 0)
#define SET_CALL_STATE _IO(ACCDET_IOC_MAGIC, 1)
#define GET_BUTTON_STATUS _IO(ACCDET_IOC_MAGIC, 2)

/* 400us, Accdet irq clear timeout  */
#define ACCDET_TIME_OUT 0x61A80

/* cable type  recognized by accdet, and report to WiredAccessoryManager */
enum accdet_report_state {
	NO_DEVICE = 0,
	HEADSET_MIC = 1,
	HEADSET_NO_MIC = 2,
	HEADSET_FIVE_POLE = 3,
	LINE_OUT_DEVICE = 4,
};

/* accdet status got from accdet FSM  */
enum accdet_status {
	PLUG_OUT = 0,
	MIC_BIAS = 1,
	HOOK_SWITCH = 2,
	BI_MIC_BIAS = 3,
	LINE_OUT = 4,
	STAND_BY = 5
};

struct three_key_threshold {
	unsigned int mid;
	unsigned int up;
	unsigned int down;
};
struct four_key_threshold {
	unsigned int mid;
	unsigned int voice;
	unsigned int up;
	unsigned int down;
};

struct pwm_deb_settings {
	unsigned int pwm_width;
	unsigned int pwm_thresh;
	unsigned int fall_delay;
	unsigned int rise_delay;
	/* state00, 3pole | hook switch */
	unsigned int debounce0;
	/* state01, mic bias debounce */
	unsigned int debounce1;
	/* state11, plug out debounce */
	unsigned int debounce3;
	/* auxadc debounce */
	unsigned int debounce4;
};

struct head_dts_data {
	/* set mic bias voltage set: 0x02,1.9V;0x07,2.7V */
	unsigned int mic_vol;
	/* set mic bias mode:1,ACC;2,DCC,without internal bias;
	 * 6,DCC,with internal bias
	 */
	unsigned int mic_mode;
	/* set the plugout debounce */
	unsigned int plugout_deb;
	/* eint0&eint1  polarity,LEVEL_HIGH(4); LEVEL_LOW(8);
	 * EDGE_FALLING(2); EDGE_RISING(1)
	 */
	unsigned int eint_pol;
	struct pwm_deb_settings pwm_deb;
	struct three_key_threshold three_key;
	struct four_key_threshold four_key;
};

enum {
	accdet_state000 = 0,
	accdet_state001,
	accdet_state010,
	accdet_state011,
	accdet_state100,
	accdet_state101,
	accdet_state110,
	accdet_state111,
	accdet_auxadc
};

extern const struct of_device_id accdet_of_match[];

extern s32 pwrap_read(u32 adr, u32 *rdata);
extern s32 pwrap_write(u32 adr, u32 wdata);
/* just be called by audio module */
int accdet_read_audio_res(unsigned int res_value);
/* just be called by audio module for DC trim */
void accdet_late_init(unsigned long data);
const struct file_operations *accdet_get_fops(void);
struct platform_driver accdet_driver_func(void);
void mt_accdet_remove(void);
void mt_accdet_suspend(void);
void mt_accdet_resume(void);
int mt_accdet_probe(struct platform_device *dev);
long mt_accdet_unlocked_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg);

#endif


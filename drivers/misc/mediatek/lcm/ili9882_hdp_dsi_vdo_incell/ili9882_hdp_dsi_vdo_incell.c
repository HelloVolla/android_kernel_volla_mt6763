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

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(ALWAYS, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(INFO, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_notice("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

#define LCM_DSI_CMD_MODE                0
#define FRAME_WIDTH			(720)
#define FRAME_HEIGHT			(1440)

//prize-tangcong modify LCD size-20200331-start
#define LCM_PHYSICAL_WIDTH                  				(63100)
#define LCM_PHYSICAL_HEIGHT                  				(123970)
//prize-tangcong modify LCD size-20200331-end

#define REGFLAG_PORT_SWAP               0xFFFA
#define REGFLAG_DELAY                   0xFFFC
#define REGFLAG_UDELAY                  0xFFFB
#define REGFLAG_END_OF_TABLE            0xFFFD

#ifndef GPIO_LCM_RST
#define GPIO_LCM_RST		(GPIO45 | 0x80000000)
#endif

// ---------------------------------------------------------------------------
//  Local Variable
// ---------------------------------------------------------------------------
static struct LCM_UTIL_FUNCS lcm_util;


#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))

#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
		lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#endif
struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[120];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	{0xFF, 03,{0x98,0x82,0x01}},
	{0x00, 01,{0x46}},
	{0x01, 01,{0x33}},
	{0x02, 01,{0x00}},
	{0x03, 01,{0x00}},
	{0x04, 01,{0x42}},
	{0x05, 01,{0x33}},
	{0x06, 01,{0x00}},
	{0x07, 01,{0x00}},
	{0x08, 01,{0x83}},
	{0x09, 01,{0x06}},
	{0x0A, 01,{0x72}},
	{0x0B, 01,{0x00}},
	{0x0C, 01,{0x00}},
	{0x0D, 01,{0x00}},
	{0x0E, 01,{0x00}},
	{0x0F, 01,{0x00}},
	{0x12, 01,{0x01}},
	{0x31, 01,{0x02}},
	{0x32, 01,{0x0E}},
	{0x33, 01,{0x0C}},
	{0x34, 01,{0x16}},
	{0x35, 01,{0x14}},
	{0x36, 01,{0x12}},
	{0x37, 01,{0x10}},
	{0x38, 01,{0x02}},
	{0x39, 01,{0x02}},
	{0x3A, 01,{0x02}},
	{0x3B, 01,{0x08}},
	{0x3C, 01,{0x0A}},
	{0x3D, 01,{0x07}},
	{0x3E, 01,{0x07}},
	{0x3F, 01,{0x07}},
	{0x40, 01,{0x07}},
	{0x41, 01,{0x07}},
	{0x42, 01,{0x07}},
	{0x43, 01,{0x07}},
	{0x44, 01,{0x07}},
	{0x45, 01,{0x07}},
	{0x46, 01,{0x07}},
	{0x47, 01,{0x0F}},
	{0x48, 01,{0x0D}},
	{0x49, 01,{0x17}},
	{0x4A, 01,{0x15}},
	{0x4B, 01,{0x13}},
	{0x4C, 01,{0x11}},
	{0x4D, 01,{0x02}},
	{0x4E, 01,{0x02}},
	{0x4F, 01,{0x02}},
	{0x50, 01,{0x09}},
	{0x51, 01,{0x0B}},
	{0x52, 01,{0x07}},
	{0x53, 01,{0x07}},
	{0x54, 01,{0x07}},
	{0x55, 01,{0x07}},
	{0x56, 01,{0x07}},
	{0x57, 01,{0x07}},
	{0x58, 01,{0x07}},
	{0x59, 01,{0x07}},
	{0x5A, 01,{0x07}},
	{0x5B, 01,{0x07}},
	{0x5C, 01,{0x07}},
	{0x61, 01,{0x02}},
	{0x62, 01,{0x09}},
	{0x63, 01,{0x0B}},
	{0x64, 01,{0x15}},
	{0x65, 01,{0x17}},
	{0x66, 01,{0x11}},
	{0x67, 01,{0x13}},
	{0x68, 01,{0x02}},
	{0x69, 01,{0x02}},
	{0x6A, 01,{0x02}},
	{0x6B, 01,{0x0F}},
	{0x6C, 01,{0x0D}},
	{0x6D, 01,{0x07}},
	{0x6E, 01,{0x07}},
	{0x6F, 01,{0x07}},
	{0x70, 01,{0x07}},
	{0x71, 01,{0x07}},
	{0x72, 01,{0x07}},
	{0x73, 01,{0x07}},
	{0x74, 01,{0x07}},
	{0x75, 01,{0x07}},
	{0x76, 01,{0x07}},
	{0x77, 01,{0x08}},
	{0x78, 01,{0x0A}},
	{0x79, 01,{0x14}},
	{0x7A, 01,{0x16}},
	{0x7B, 01,{0x10}},
	{0x7C, 01,{0x12}},
	{0x7D, 01,{0x02}},
	{0x7E, 01,{0x02}},
	{0x7F, 01,{0x02}},
	{0x80, 01,{0x0E}},
	{0x81, 01,{0x0C}},
	{0x82, 01,{0x07}},
	{0x83, 01,{0x07}},
	{0x84, 01,{0x07}},
	{0x85, 01,{0x07}},
	{0x86, 01,{0x07}},
	{0x87, 01,{0x07}},
	{0x88, 01,{0x07}},
	{0x89, 01,{0x07}},
	{0x8A, 01,{0x07}},
	{0x8B, 01,{0x07}},
	{0x8C, 01,{0x07}},
	{0xB0, 01,{0x22}},
	{0xE7, 01,{0x54}},
	{0xFF, 03,{0x98,0x82,0x02}},
	{0xF1, 01,{0x1C}},
	{0x4B, 01,{0x5A}},
	{0x50, 01,{0xCA}},
	{0x51, 01,{0x00}},
	{0x06, 01,{0x9B}},
	{0x0B, 01,{0xA0}},
	{0x0C, 01,{0x00}},
	{0x0D, 01,{0x20}},
	{0x0E, 01,{0xEA}},
	{0x4E, 01,{0x11}},
	{0xFF, 03,{0x98,0x82,0x05}},
	{0x03, 01,{0x01}},
	{0x04, 01,{0x06}},
	{0x63, 01,{0x7E}},
	{0x64, 01,{0x7E}},
	{0x68, 01,{0xA1}},
	{0x69, 01,{0xA7}},
	{0x6A, 01,{0x8D}},
	{0x6B, 01,{0x7F}},
	{0x85, 01,{0x37}},
	{0x46, 01,{0x00}},
	{0xFF, 03,{0x98,0x82,0x06}},
	{0xD9, 01,{0x1F}},
	{0xC0, 01,{0xA0}},
	{0xC1, 01,{0x15}},
	
	{0xD6, 01,{0x55}},
	{0xDD, 01,{0x61}},
	{0xDC, 01,{0x88}},
	

	{0xFF, 03,{0x98,0x82,0x08}},
	{0xE0, 27,{0x00,0x24,0x44,0x60,0x8B,0x50,0xB3,0xD7,0x03,0x29,0x55,0x67,0x9B,0xCA,0xF7,0xAA,0x27,0x62,0x87,0xB8,0xFE,0xE2,0x18,0x59,0x90,0x03,0xEC}},
	{0xE1, 27,{0x00,0x24,0x44,0x60,0x8B,0x50,0xB3,0xD7,0x03,0x29,0x55,0x67,0x9B,0xCA,0xF7,0xAA,0x27,0x62,0x87,0xB8,0xFE,0xE2,0x18,0x59,0x90,0x03,0xEC}},
	{0xFF, 03,{0x98,0x82,0x0B}},
	{0x9A, 01,{0x44}},
	{0x9B, 01,{0xF6}},
	{0x9C, 01,{0x04}},
	{0x9D, 01,{0x04}},
	{0x9E, 01,{0x7C}},
	{0x9F, 01,{0x7C}},
	{0xAB, 01,{0xE0}},
	{0xFF, 03,{0x98,0x82,0x0E}},
	{0x11, 01,{0x10}},
	{0x13, 01,{0x10}},
	{0x00, 01,{0xA0}},
	{0xFF, 03,{0x98,0x82,0x00}},
	{0x35, 01,{0x00}},
	{0x11,1,{0x00}},
	{REGFLAG_DELAY,120,{}},
	{0x29,1,{0x00}},
	{REGFLAG_DELAY,20,{}},

};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	{0x26, 1, {0x08}},
	// Display off sequence
	{0x28, 0, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	// Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}},
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	for(i = 0; i < count; i++) {
		unsigned int cmd;
		cmd = table[i].cmd;
		switch (cmd) {
			case REGFLAG_DELAY :
				MDELAY(table[i].count);
				break;
			case REGFLAG_END_OF_TABLE :
				break;
			default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

//	params->virtual_width = VIRTUAL_WIDTH;
//	params->virtual_height = VIRTUAL_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode            = CMD_MODE;
#else
	params->dsi.mode            = BURST_VDO_MODE;
#endif

	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding		= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability */
	/* video mode timing */
	params->dsi.PS                                  = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.vertical_sync_active                = 8;
	params->dsi.vertical_backporch                  = 20;
	params->dsi.vertical_frontporch                 = 220;
	params->dsi.vertical_active_line                = FRAME_HEIGHT;
	params->dsi.horizontal_sync_active              = 8;
	params->dsi.horizontal_backporch                = 40;
	params->dsi.horizontal_frontporch               = 40;
	params->dsi.horizontal_active_pixel             = FRAME_WIDTH;
	params->dsi.LANE_NUM                            = LCM_FOUR_LANE;
	/*prize-zhaopengge modify lcd fps-20201012-start*/
	params->dsi.PLL_CLOCK                           = 260;
	/*prize-zhaopengge modify lcd fps-20201012-end*/
     //prize-tangcong modify LCD size-20200331-start
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
	//prize-tangcong modify LCD size-20200331-end
	params->dsi.ssc_disable                         = 1;
	params->dsi.ssc_range                           = 4;

	params->dsi.HS_TRAIL                            = 15;
	params->dsi.noncont_clock                       = 1;
	params->dsi.noncont_clock_period                = 1;

	/* ESD check function */
	params->dsi.esd_check_enable                    = 1;
	params->dsi.customization_esd_check_enable      = 0;
	//params->dsi.clk_lp_per_line_enable              = 1;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 1;
	params->full_content = 0;
/*prize-zhaopengge modify round corner size-20200928-start*/
	params->corner_pattern_width = 720;
	params->corner_pattern_height = 16;
	params->corner_pattern_height_bot = 16;
/*prize-zhaopengge modify round corner size-20200928-end*/
#endif
}

static void lcm_init_power(void)
{
	display_bias_enable();
}

static void lcm_suspend_power(void)
{
	display_bias_disable();
}

static void lcm_resume_power(void)
{
	SET_RESET_PIN(0);
	display_bias_enable();
}

static void lcm_init(void)
{

	MDELAY(15);

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(35);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(5);

	SET_RESET_PIN(0);
	MDELAY(10);
}

static void lcm_resume(void)
{
	lcm_init();
}

static unsigned int lcm_compare_id(void)
{
    int array[4];
    char buffer[4]={0,0,0,0};
	char buffer1[4]={0,0,0,0};
	char buffer2[4]={0,0,0,0};
    unsigned int id=0;
	//int rawdata = 0;
	//int ret = 0;

	
	struct LCM_setting_table switch_table_page6[] = {
		{ 0xFF, 0x03, {0x98, 0x82, 0x06} }
	};
//	struct LCM_setting_table switch_table_page0[] = {
//		{ 0xFF, 0x03, {0x98, 0x81, 0x00} }
//	};
/*	
    display_ldo18_enable(1);
    display_bias_vpos_enable(1);
    display_bias_vneg_enable(1);
    MDELAY(10);
    display_bias_vpos_set(5800);
	display_bias_vneg_set(5800);
*/
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);

	push_table(switch_table_page6,sizeof(switch_table_page6) / sizeof(struct LCM_setting_table),1);

    MDELAY(5);
    array[0] = 0x00013700;
    dsi_set_cmdq(array, 1, 1);
    //MDELAY(10);
    read_reg_v2(0xF0, buffer, 1);//    NC 0x00  0x98 
	
	MDELAY(5);
    array[0] = 0x00013700;
    dsi_set_cmdq(array, 1, 1);
    //MDELAY(10);
    read_reg_v2(0xF1, buffer1, 1);//    NC 0x00  0x82 
	
	MDELAY(5);
    array[0] = 0x00013700;
    dsi_set_cmdq(array, 1, 1);
    //MDELAY(10);
    read_reg_v2(0xF2, buffer2, 1);//    NC 0x00  0x0d 

    id = (buffer[0]<<16) | (buffer1[0]<<8) |buffer2[0];
	
#ifdef BUILD_LK
	dprintf(CRITICAL, "%s, LK debug: ili9882 id = 0x%08x\n", __func__, id);
#else
	printk("%s: ili9882 id = 0x%08x \n", __func__, id);
#endif
  
    if(0x98820d == id)
	{
		//is_9882N_detect=1;
	    return 1;
	}
    else{
		return 0;
	}

}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
#if 1
	unsigned int ret = 0;
	unsigned int x0 = FRAME_WIDTH / 4;
	unsigned int x1 = FRAME_WIDTH * 3 / 4;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);

	unsigned int data_array[3];
	unsigned char read_buf[4];
	LCM_LOGI("ATA check size = 0x%x,0x%x,0x%x,0x%x\n", x0_MSB, x0_LSB, x1_MSB, x1_LSB);
	
	data_array[0] = 0x0003390A; /* HS packet */
	data_array[1] = 0x00595AF0;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x0003390A; /* HS packet */
	data_array[1] = 0x00A6A5F1;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x0004390A; /* HS packet */
	data_array[1] = (x1_LSB << 24) | (x1_MSB << 16) | (x0_LSB << 8) | 0xB9;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00033700; /* read id return two byte,version and id */
	dsi_set_cmdq(data_array, 1, 1);
	read_reg_v2(0x04, read_buf, 3);
	printk("%s read[0x04]= %x  %x %x %x\n", __func__, read_buf[0], read_buf[1],read_buf[2],read_buf[3]);

//	MDELAY(10);
	data_array[0] = 0x00033700; /* read id return two byte,version and id */
	dsi_set_cmdq(data_array, 1, 1);
	read_reg_v2(0xB9, read_buf, 3);
	printk("%s read[0xB9]= %x %x %x %x \n", __func__, read_buf[0],read_buf[1],read_buf[2],read_buf[3]);

	if ((read_buf[0] == x0_LSB) && (read_buf[1] == x1_MSB)
	        && (read_buf[2] == x1_LSB))
		ret = 1;
	else
		ret = 0;

	return ret;
#endif
	return 1;
#else
	return 0;
#endif
}

struct LCM_DRIVER ili9882_hdp_dsi_vdo_incell_lcm_drv =
{
	.name		= "ili9882_hdp_dsi_vdo_incell",
	#if defined(CONFIG_PRIZE_HARDWARE_INFO) && !defined (BUILD_LK)
	.lcm_info = {
		.chip	= "ili9882",
		.vendor	= "ilitek",
		.id = "0x98820d",
		.more	= "720*1440",
	},
   #endif
	.set_util_funcs	= lcm_set_util_funcs,
	.get_params	= lcm_get_params,
	.init		= lcm_init,
	.suspend	= lcm_suspend,
	.resume		= lcm_resume,
	.compare_id	= lcm_compare_id,
	.init_power	= lcm_init_power,
	.ata_check	= lcm_ata_check,
#ifndef BUILD_LK
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
#endif
};

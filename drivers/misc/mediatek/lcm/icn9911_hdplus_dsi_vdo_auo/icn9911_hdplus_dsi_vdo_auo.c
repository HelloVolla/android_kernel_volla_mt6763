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
#else
/*#include <mach/mt_pm_ldo.h>*/
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#endif
#endif
#ifdef CONFIG_MTK_LEGACY
#include <cust_gpio_usage.h>
#endif
#ifndef CONFIG_FPGA_EARLY_PORTING
#if defined(CONFIG_MTK_LEGACY)
#include <cust_i2c.h>
#endif
#endif
#include "tpd.h"
//#include <mach/gpio_const.h>
//#include "tps65132.h"

#ifdef BUILD_LK
#define LCM_LOGI(fmt, args...)  printk(KERN_INFO  " LCM file=%s: %s: line=%d: "fmt"\n", __FILE__,__func__,  __LINE__,##args)
#define LCM_LOGD(fmt, args...)  printk(KERN_DEBUG " LCM file=%s: %s: line=%d: "fmt"\n", __FILE__,__func__,  __LINE__,##args)
#define LCM_ENTER() printk(KERN_DEBUG " LCM file=%s: %s: line=%d: Enter------->\n", __FILE__,__func__, __LINE__)
#define LCM_EXIT()  printk(KERN_DEBUG " LCM file=%s: %s: line=%d: Exit<-------\n",  __FILE__,__func__, __LINE__)

#else
#define LCM_LOGI(fmt, args...)  printk(KERN_INFO " LCM :"fmt"\n", ##args)
#define LCM_LOGD(fmt, args...)  printk(KERN_DEBUG " LCM :"fmt"\n", ##args)
#define LCM_ENTER() 
#define LCM_EXIT()  

#endif


#define I2C_I2C_LCD_BIAS_CHANNEL 0
static struct LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)			(lcm_util.set_reset_pin((v)))
#define MDELAY(n)					(lcm_util.mdelay(n))

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) \
	lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
	lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
	lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
	lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

static const unsigned char LCD_MODULE_ID = 0x01;
/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
#define LCM_DSI_CMD_MODE	0
#define FRAME_WIDTH  										720
#define FRAME_HEIGHT 										1520
#define LCM_PHYSICAL_WIDTH                  				(67610)
#define LCM_PHYSICAL_HEIGHT                  				(142730)


#define REGFLAG_DELAY             							 0xFFFA
#define REGFLAG_UDELAY             							 0xFFFB
#define REGFLAG_PORT_SWAP									 0xFFFC
#define REGFLAG_END_OF_TABLE      							 0xFFFD   // END OF REGISTERS MARKER

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {

	{0xF0,2,{0x5A,0x5A}},
	{0xF1,2,{0xA5,0xA5}},
	{0xC2,1,{0x00}},
	{0xB0,16,{0x76,0x54,0x11,0x11,0x33,0x33,0x33,0x33,0x00,0x01,0x01,0x76,0x01,0x01,0x00,0x00}},
	{0xB1,13,{0x53,0xD4,0x02,0x85,0x00,0x01,0x01,0x76,0x01,0x01,0x53,0x5F,0x5F}},
	{0xB2,16,{0x37,0x20,0x05,0x80,0x65,0x02,0x08,0x20,0x30,0x91,0x22,0x33,0x44,0x00,0x18,0x01}},
	{0xB3,16,{0x0F,0x00,0x87,0x10,0x80,0x26,0x26,0xC0,0x3F,0xAA,0x33,0xC3,0xAA,0x30,0xC3,0xAA}},
	{0xB4,12,{0x03,0x23,0x14,0x05,0x07,0x1B,0x0F,0x0D,0x13,0x11,0x24,0x15}},
	{0xB6,12,{0x03,0x23,0x14,0x04,0x06,0x1B,0x0E,0x0C,0x12,0x10,0x24,0x15}},
	{0xBB,16,{0x00,0x00,0x00,0x00,0x02,0xFF,0xFD,0x0B,0x33,0x01,0x73,0x33,0x33,0x00,0x00,0x00}},
	{0xBC,10,{0x61,0x03,0xFF,0xDE,0x72,0xE0,0x2E,0x04,0x88,0x3E}},
	{0xBD,16,{0x8E,0x0E,0x78,0x78,0x15,0x15,0x46,0x5A,0x14,0x66,0x23,0x06,0x00,0x00,0x00,0x00}},
	{0xBE,5,{0x60,0x60,0x50,0x60,0x77}},
	{0xC1,14,{0x70,0x7C,0x0C,0x78,0x04,0x0C,0x10,0x04,0x2A,0x71,0x00,0x07,0x10,0x10}},
	{0xC3,9,{0x00,0x00,0xFF,0x00,0xFF,0x00,0x00,0x0D,0x1F}},
	{0xC4,8,{0xB4,0xA3,0xEE,0x41,0x04,0x2F,0x00,0x00}},
	{0xC5,16,{0x07,0x1F,0x42,0x26,0x51,0x44,0x14,0x1A,0x04,0x00,0x0A,0x08,0x00,0x00,0x00,0x00}},
	{0xC6,7,{0x85,0x01,0x67,0x01,0x33,0x00,0x03}},
	{0xC7,16,{0x7C,0x71,0x67,0x00,0x6E,0x5E,0x51,0x57,0x40,0x58,0x54,0x3C,0x52,0x4B,0x58,0x49}},
	{0xC8, 5,{0x34,0x00,0x36,0x24,0x0C}},
	{0xC9,16,{0x7C,0x71,0x67,0x00,0x6E,0x5E,0x51,0x57,0x40,0x58,0x54,0x3C,0x52,0x4B,0x58,0x49}},
	{0xCA, 5,{0x34,0x00,0x36,0x24,0x0C}},
	{0xCB,11,{0x00,0x00,0x00,0x01,0x6C,0x00,0x33,0x00,0x17,0xFF,0xEF}},
	{0xF0,2,{0xB4,0x4B}},
	{0xD0,8,{0x80,0x0D,0xFF,0x0F,0x63,0x2B,0x08,0x88}},
	{0xD2,10,{0x43,0x0C,0x00,0x01,0x80,0x26,0x04,0x00,0x16,0x01}},
	{0x35,1,{0x00}},
	{0xF0,2,{0xA5,0xA5}},
	{0xF1,2,{0x5A,0x5A}},
	{0x11,1,{0x00}},
	{REGFLAG_DELAY,60,{}},//120 sleep out
	{0x29,1,{0x00}},
	{REGFLAG_DELAY, 20, {}},//120
    {0x26,1,{0x01}},
	{REGFLAG_DELAY, 1, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}    
              
};



#if 1
static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x26, 1,{0x03} },
	{REGFLAG_DELAY, 1, {} },
	{0x28, 1,{0x00} },
	{REGFLAG_DELAY, 20, {} },
	{0x10, 1,{0x00} },
	{REGFLAG_DELAY, 120, {} },//sleep in
	{REGFLAG_END_OF_TABLE, 0x00, {}}  
};
#endif
//#if 0
static void push_table(struct LCM_setting_table *table, unsigned int count,
		       unsigned char force_update)
{
	unsigned int i;
    LCM_ENTER();
	for (i = 0; i < count; i++) {
		unsigned cmd;

		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
	LCM_EXIT();
}




/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */

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

		// enable tearing-free
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
    params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
        #else
	params->dsi.mode   =  BURST_VDO_MODE;//SYNC_EVENT_VDO_MODE;//BURST_VDO_MODE;////
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
	
	
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	
#if (LCM_DSI_CMD_MODE)
	params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
	params->dsi.word_count=FRAME_WIDTH*3;	//DSI CMD mode need set these two bellow params, different to 6577
#else
	params->dsi.intermediat_buffer_num = 0;	//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
#endif

	// Video mode setting
	params->dsi.packet_size=256;

	params->dsi.vertical_sync_active				=  4;
	params->dsi.vertical_backporch					= 12;//16 25 30 35 12 8
	params->dsi.vertical_frontporch					= 120;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active = 4;
	params->dsi.horizontal_backporch = 42;//32
	params->dsi.horizontal_frontporch = 80;//78
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	/* params->dsi.ssc_disable                                                       = 1; */

	params->dsi.PLL_CLOCK = 270;//244;
			params->dsi.ssc_disable = 1;
	params->dsi.ssc_range = 0;
	params->dsi.cont_clock = 0;
	params->dsi.clk_lp_per_line_enable = 0;
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
    params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
    params->physical_width_um = LCM_PHYSICAL_WIDTH;
    params->physical_height_um = LCM_PHYSICAL_HEIGHT;
    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable = 1;
    params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
    params->dsi.lcm_esd_check_table[0].count = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

}



static unsigned int lcm_compare_id(void)
{
	
	unsigned char buffer[2];
	unsigned int array[16];  
	
	SET_RESET_PIN(1);
	
	MDELAY(20);//100

	SET_RESET_PIN(0);
	MDELAY(20);//100
	//tps65132_avdd_en(TRUE);

	MDELAY(10);  

	SET_RESET_PIN(1);
	MDELAY(250);//250
	
   

	array[0]=0x00DE0500;
	dsi_set_cmdq(array, 1, 1);

	array[0]=0x32B41500; 
	dsi_set_cmdq(array, 1, 1);

	array[0]=0x00DF0500;
	dsi_set_cmdq(array, 1, 1);

	array[0] = 0x00013700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xFA, buffer, 1);
	
    #ifdef BUILD_LK
	LCM_LOGI("%s, LK TDDI id = 0x%08x\n", __func__, buffer[0]);
   #else
	LCM_LOGI("%s, Kernel TDDI id = 0x%08x\n", __func__, buffer[0]);
   #endif

   return ((0x99 == buffer[0]) || (0x00 == buffer[0]))?1:0; 
	//return 1;//(LCM_ID_NT35532 == id)?1:0;


}
#ifndef BUILD_LK
extern atomic_t ESDCheck_byCPU;
#endif
static unsigned int lcm_ata_check(unsigned char *bufferr)
{
	//prize-Solve ATA testing-pengzhipeng-20181127-start
	unsigned char buffer1[2]={0};
	unsigned char buffer2[2]={0};
	
	unsigned int data_array[6]; 
	 
	data_array[0]= 0x00023902;//LS packet 
	data_array[1]= 0x000050b8; 
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00013700;// read id return two byte,version and id
	dsi_set_cmdq(data_array, 1, 1);
	atomic_set(&ESDCheck_byCPU, 1);
	
	read_reg_v2(0xb8, buffer1, 1);
	atomic_set(&ESDCheck_byCPU, 0);
	
	
	data_array[0]= 0x0002390a;//HS packet 
	data_array[1]= 0x000031b8; 
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x00013700;// read id return two byte,version and id
	dsi_set_cmdq(data_array, 1, 1);
	atomic_set(&ESDCheck_byCPU, 1);
	
	read_reg_v2(0xb8, buffer2, 1);
	atomic_set(&ESDCheck_byCPU, 0);
	
	
	LCM_LOGI("%s, Kernel TDDI id buffer1= 0x%04x buffer2= 0x%04x\n", __func__, buffer1[0],buffer2[0]);
	return ((0x50 == buffer1[0])&&(0x31 == buffer2[0]))?1:0; 
	//prize-Solve ATA testing-pengzhipeng-20181127-end
}

static void lcm_init(void)
{

	tpd_gpio_output(0, 1);
	SET_RESET_PIN(1);
	
	MDELAY(10);//100

	SET_RESET_PIN(0);
	MDELAY(10);//100
	//tps65132_avdd_en(TRUE);

	//MDELAY(10);  

	SET_RESET_PIN(1);
	MDELAY(50);//250
	

	push_table(lcm_initialization_setting,
		   sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	LCM_EXIT();
}

static void lcm_suspend(void)
{
	LCM_ENTER();     
	push_table(lcm_suspend_setting,
		   sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(10);

	//mt_dsi_pinctrl_set(LCM_POWER_DM_NO, 0);
	//MDELAY(10);

	//mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 0);
	//MDELAY(10);

	LCM_EXIT();
}

static void lcm_resume(void)
{
	lcm_init();
	
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




struct LCM_DRIVER icn9911_hdplus_dsi_vdo_auo_drv= 
{
    .name			= "icn9911_hdplus_dsi_vdo_auo_auo",
	#if defined(CONFIG_PRIZE_HARDWARE_INFO) && !defined (BUILD_LK)
	.lcm_info = {
		.chip	= "icn9911",
		.vendor	= "himax",
		.id		= "0x9911",
		.more	= "720*1520",
	},
	#endif
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	.ata_check 		= lcm_ata_check,
   
};

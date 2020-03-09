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

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

static LCM_UTIL_FUNCS lcm_util;

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

/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
#define LCM_DSI_CMD_MODE	0
#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)

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

{0xB0,1,{0x04}},
{0xB3,3,{0x10,0x00,0x06}},
{0xB4,2,{0x00,0x03}},//3 lane
{0xB6,7,{0x32,0x33,0x80,0x00,0x00,0x07,0x86}},
{0xB8,6,{0x00,0x78,0x64,0x10,0x64,0xB4}},
{0xB9,6,{0x00,0x78,0x64,0x10,0x64,0xB4}},
{0xBA,6,{0x00,0x78,0x64,0x10,0x64,0xB4}},
{0xBB,3,{0x00,0xB4,0xA0}},
{0xBC,3,{0x00,0xB4,0xA0}},
{0xBD,3,{0x00,0xB4,0xA0}},
{0xBE,1,{0x04}},  //04
{0xC0,17,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
        ,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},  //03
{0xC1,34,{0x00,0x48,0x00,0x00,0x33,0x08,0x11,0x00,0x11,0x00
        ,0x73,0x23,0x23,0x11,0x00,0x00,0x00,0x00,0x00,0x00
		,0x00,0x00,0x00,0x00,0x00,0x00,0xDF,0x00,0x30,0x00
		,0x01,0x00,0x00,0x00}},  //03
{0xC2,19,{0x00,0xF0,0x05,0x00,0x0A,0x04,0x08,0x00,0x24,0x19
        ,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00}},
{0xC3,63,{0x51,0x15,0x11,0x51,0x10,0x00,0x00,0x00,0x00,0x00
        ,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
		,0x01,0x01,0x03,0x28,0x00,0x01,0x01,0x01,0x00,0x00
		,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
		,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x67,0x01
		,0x00,0x00,0x00,0x00,0x67,0x01,0x00,0x00,0x00,0x00
		,0x40,0x20,0x01}},
{0xC4,40,{0x70,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
        ,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
		,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
		,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01}},
{0xC5,10,{0x08,0x00,0x00,0x00,0x00,0x70,0x00,0x00,0x2D,0x41}},  //04
{0xC6,33,{0xC2,0x14,0x89,0x00,0x00,0x14,0x89,0x00,0x00,0x00
        ,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
		,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
		,0x00,0xC2,0xC2}},  //04
{0xC7,46,{0x00,0x0D,0x17,0x23,0x2E,0x38,0x4E,0x5D,0x6B,0x76
        ,0x28,0x34,0x42,0x57,0x63,0x71,0x84,0x90,0x97,0x00
		,0x0D,0x17,0x23,0x2E,0x38,0x4E,0x5D,0x6B,0x76,0x28
		,0x34,0x42,0x57,0x63,0x71,0x84,0x90,0x97,0x00,0x97
		,0x00,0x97,0x00,0x97,0x00,0x97}},
{0xC8,55,{0x01,0x00,0x00,0x00,0x00,0xFC,0x00,0x00,0x00,0x00,0x00,0xFC,
         0x00,0x00,0x00,0x00,0x00,0xFC,0x00,0x00,0x04,0xFF,0x03,0xFC,
		 0x00,0x00,0x04,0xFE,0x03,0xF8,0x00,0x00,0xFE,0x02,0xFB,0x00,
		 0x00,0x00,0x04,0xFE,0x03,0xFC,0x00,0x00,0x04,0xFE,0x02,0xF3,
		 0x00,0x00,0xFF,0x01,0xFA,0x84,0x00}},
{0xC9,19,{0x00,0x00,0x00,0x00,0x00,0xFC,0x00,0x00,0x00,0x00,0x00,0xFC,
         0x00,0x00,0x00,0x00,0x00,0xFC,0x00}},
{0xCA,43,{0x1D,0xFC,0xFC,0xFC,0xF0,0xF1,0xDA,0xD3,0xFA,0xD3
        ,0xFA,0xD3,0xE2,0xE2,0x00,0xE2,0x00,0x00,0x00,0xEC
		,0x00,0x00,0x00,0x30,0xC2,0x71,0x61,0x00,0x00,0x00
		,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
		,0x00,0x00,0x00}},
{0xCC,45,{0x00,0x00,0x00,0x00,0xD2,0x72,0x12,0x16,0x1A,0x1E
        ,0xD0,0x70,0x42,0x46,0x00,0x00,0x00,0x00,0x00,0x00
		,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
		,0x44,0x40,0x70,0xD0,0x1C,0x18,0x14,0x10,0x72,0xD2
		
		
		,0x00,0x00,0x00,0x00,0x00}},
{0xCD,19,{0x05,0x06,0x69,0x01,0x00,0x00,0x00,0x00,0x00,0x00
        ,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00}},
{0xCE,24,{0x5D,0x40,0x43,0x49,0x55,0x62,0x71,0x82,0x94,0xA8
        ,0xB9,0xCB,0xDB,0xE9,0xF5,0xFC,0xFF,0x04,0x00,0x04
		,0x04,0x00,0x00,0x00}},
{0xCF,2,{0x48,0x10}},
{0xD0,18,{0x11,0x04,0x59,0xD9,0x03,0x10,0x10,0x40,0x19,0x08
        ,0x99,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0xD1,1,{0x04}},
{0xD3,27,{0xBB,0x3B,0x33,0x3B,0x44,0x3B,0x44,0x3B,0x00,0x00
        ,0xEC,0x91,0x9A,0x23,0x22,0xD3,0xD3,0x3B,0xBB,0x4F
		,0xD0,0x3C,0x10,0x12,0x10,0x00,0x10}},
{0xD4,24,{0x80,0x04,0x04,0x33,0x00,0x04,0x00,0x00,0x00,0x00
        ,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0A,0x90
		,0x05,0x00,0x64,0x94}},

{0xD5,10,{0x06,0x00,0x00,0x01,0x4D,0x01,0x4D,0x01,0x00,0x00}},

{0xD6,1,{0x01}},
{0xD7,36,{0xF6,0xFF,0x03,0x05,0x43,0x24,0x80,0x1F,0xC7,0x1F,0x1B,0x00,0x0F,0x01
        ,0x20,0x08,0x80,0x3F,0x00,0x78,0x00,0x40,0x24,0x15,0x00,0x33,0x02,0xC0
		,0xAF,0xCB,0x60,0x30,0xFC,0x00,0x3F,0x00}},
{0xD8,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0xDD,4,{0x30,0x06,0x23,0x65}},
{0xDE,4,{0x00,0xFF,0xFF,0x90}},
{0xD6,1,{0x01}}, 
{0x51,1,{0xFF}},
{0x53,1,{0x0C}},
{0x55,1,{0x03}},
{0x35,1,{0x00}},

{0x11,1,{0x00}},
{REGFLAG_DELAY, 150, {}},  
{0x29,1,{0x00}},	
{REGFLAG_DELAY, 50, {}},  
{REGFLAG_END_OF_TABLE, 0x00, {}}       
              
};

static void push_table(struct LCM_setting_table *table, unsigned int count,
		       unsigned char force_update)
{
	unsigned int i;
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
}

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->physical_width = 68;//LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = 122;//LCM_PHYSICAL_HEIGHT/1000;
	params->physical_width_um = 68489;//LCM_PHYSICAL_WIDTH;	= sqrt((size*25.4)^2/(18^2+9^2))*9*1000
	params->physical_height_um = 121759;//LCM_PHYSICAL_HEIGHT; = sqrt((size*25.4)^2/(18^2+9^2))*18*1000
	params->density = 320;//LCM_DENSITY;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
#endif
	
	// DSI
	/* Command mode setting */
	//1 Three lane or Four lane
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;//LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Video mode setting		
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	
	params->dsi.vertical_sync_active = 126;
	params->dsi.vertical_backporch = 128;
	params->dsi.vertical_frontporch	= 127;
	params->dsi.vertical_active_line = FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active = 4;
	params->dsi.horizontal_backporch = 4;//32
	params->dsi.horizontal_frontporch = 78;//78
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	/* params->dsi.ssc_disable                                                       = 1; */
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 300;
#else
	params->dsi.PLL_CLOCK = 260;//300
#endif
	params->dsi.clk_lp_per_line_enable				= 0;//1;
	params->dsi.esd_check_enable					= 0;
	params->dsi.customization_esd_check_enable		= 0;
	params->dsi.lcm_esd_check_table[0].cmd			= 0x0a;
	params->dsi.lcm_esd_check_table[0].count		= 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
}

static void lcm_set_bias(int enable)
{
	
	if (enable){
		display_bias_vpos_set(5800);
		display_bias_vneg_set(5800);
		display_bias_enable();
	}else{
		display_bias_disable();
		
	}
}

#ifndef BUILD_LK
extern atomic_t ESDCheck_byCPU;
#endif
static unsigned int lcm_ata_check(unsigned char *buffer)
{
#if 1
#ifndef BUILD_LK 
unsigned int ret = 0 ,ret1=2; 
//unsigned int x0 = FRAME_WIDTH/4; 
//unsigned int x1 = FRAME_WIDTH*3/4; 
//unsigned int y0 = 0;
//unsigned int y1 = y0 + FRAME_HEIGHT - 1;
unsigned char x0_MSB = 0x5;//((x0>>8)&0xFF); 
unsigned char x0_LSB = 0x2;//(x0&0xFF); 
unsigned char x1_MSB = 0x1;//((x1>>8)&0xFF); 
unsigned char x1_LSB = 0x4;//(x1&0xFF); 
//	unsigned char y0_MSB = ((y0>>8)&0xFF);
//	unsigned char y0_LSB = (y0&0xFF);
//	unsigned char y1_MSB = ((y1>>8)&0xFF);
//	unsigned char y1_LSB = (y1&0xFF);
unsigned int data_array[6]; 
unsigned char read_buf[4]; 
unsigned char read_buf1[4]; 
unsigned char read_buf2[4]; 
unsigned char read_buf3[4]; 
#ifdef BUILD_LK 
printf("ATA check size = 0x%x,0x%x,0x%x,0x%x\n",x0_MSB,x0_LSB,x1_MSB,x1_LSB); 
#else 
printk("ATA check size = 0x%x,0x%x,0x%x,0x%x\n",x0_MSB,x0_LSB,x1_MSB,x1_LSB); 
#endif 
data_array[0]= 0x0002390A;//HS packet 
data_array[1]= 0x00002453; 
dsi_set_cmdq(data_array, 2, 1); 
 data_array[0]= 0x0002390A;//HS packet 
data_array[1]= 0x0000F05e; 
dsi_set_cmdq(data_array, 2, 1); 
data_array[0]= 0x0002390A;//HS packet 
data_array[1]= 0x00000355; 
dsi_set_cmdq(data_array, 2, 1); 
//data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x51; 
//data_array[2]= (x1_LSB); 
//dsi_set_cmdq(data_array, 3, 1); 
data_array[0] = 0x00013700; 
dsi_set_cmdq(data_array, 1, 1); 
atomic_set(&ESDCheck_byCPU, 1);
read_reg_v2(0X56, read_buf1, 1); 
read_reg_v2(0X54, read_buf2, 1); 
read_reg_v2(0X5F, read_buf3, 1);
atomic_set(&ESDCheck_byCPU, 0);
if((read_buf1[0] == 0x03)&& (read_buf2[0] == 0x24) && (read_buf3[0] == 0xf0)) 
    ret = 1; 
else 
    ret = 0; 
#ifdef BUILD_LK 
printf("ATA read buf size = 0x%x,0x%x,0x%x,0x%x,ret= %d\n",read_buf[0],read_buf[1],read_buf[2],read_buf[3],ret); 
#else 
printk("ATA read buf  size = 0x%x,0x%x,0x%x,0x%x,ret= %d ret1= %d\n",read_buf[0],read_buf1[0],read_buf2[0],read_buf3[0],ret,ret1); 
printk("ATA read buf new  size = 0x%x,0x%x,0x%x,0x%x,ret= %d ret1= %d\n",read_buf1[0],read_buf1[1],read_buf1[2],read_buf1[3],ret,ret1); 
#endif 
return ret; 
#endif //BUILD_LK
#endif
}

static void lcm_init(void)
{
	display_ldo18_enable(1);
	
	MDELAY(10);
	
	SET_RESET_PIN(0);
	MDELAY(20);//100

	SET_RESET_PIN(1);
	MDELAY(250);//250
	
	lcm_set_bias(1);

	MDELAY(10);
//	lcm_compare_id();
//    init_lcm_registers();
	push_table(lcm_initialization_setting,
		   sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	//push_table(lcm_suspend_setting,
		   //sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
    unsigned int data_array[16];
    
	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
    MDELAY(20);

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);

	SET_RESET_PIN(0);
	MDELAY(10);
	
	lcm_set_bias(0);
	display_ldo18_enable(0);
}

static void lcm_resume(void)
{
	lcm_init();
}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}
#endif

#define LCM_ID_NT35532 (0x32)

static unsigned int lcm_compare_id(void)
{
	unsigned char buffer[4];
	unsigned int array[16];  

	display_ldo18_enable(1);
	MDELAY(2);
	
	SET_RESET_PIN(0);
	MDELAY(20);//100

	SET_RESET_PIN(1);
	MDELAY(250);//250
	
	lcm_set_bias(1);

    MDELAY(10); 

	array[0]=0x04B01500; 
	dsi_set_cmdq(array, 1, 1);
	array[0] = 0x00043700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xBF, buffer, 4);
	
#ifdef BUILD_LK
	printf("%s, LK td4100 TDDI reg=(%02X)(%02X)(%02X)(%02X)\n", __func__, buffer[0],buffer[1],buffer[2],buffer[3]);
#else
	printk("lsw_test td4100_sh: %s, Kernel TDDI id = 0x%08x\n", __func__, buffer[0]);
#endif

	if (!(0x02 == buffer[0] && 0x3c == buffer[1])){
		lcm_set_bias(0);
		display_ldo18_enable(0);
	}

	return (0x02 == buffer[0] && 0x3c == buffer[1])?1:0; 	//0x02 0x3c is synaptics' ID, not for td4100

}



LCM_DRIVER td4100_hd720_dsi_vdo_jdi_incell_lcm_drv = 
{
    .name			= "td4100_hd720_dsi_vdo_jdi_incell",
	#if defined(CONFIG_PRIZE_HARDWARE_INFO) && !defined (BUILD_LK)
	.lcm_info = {
		.chip	= "td4100",
		.vendor	= "unknow",
		.id		= "0x7",
		.more	= "1280*720",
	},
	#endif
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,
	//.esd_check = lcm_esd_check,
    #if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
    #endif
    .ata_check	= lcm_ata_check,
    };

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

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#else
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#endif
#include "lcm_drv.h"

/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1440)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0
/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */

static struct LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
	
{0XFF,3,{0X98,0X81,0X05}},
{0XB2,1,{0X70}},       //panda timing clr delay 1 frame
{0X03,1,{0X00}},       //VCM1[8]
{0X04,1,{0X2c}},       //VCM1[7:0] (-0.324V @9881V)   

{0X4C,1,{0X11}},     //-3/3X (-3X/3X @9881V)
  
{0X1A,1,{0X50}},      //debounce 32us
{0X38,1,{0XA0}},     //IOVCC LVD (IOVCC 1.4V @9881V)
{0X4D,1,{0X22}},     //bypass VREG to FPC
{0X54,1,{0X28}},      //VGH CLP=12.03 
{0X55,1,{0X25}},     //VGL CLP=-12.03
  
{0X1B,1,{0X09}},      //11,0X keep LVD function
{0X26,1,{0X0E}},      //Auto 1/2 VCOM-new (9881V)
  
{0X78,1,{0X01}},
{0XA9,1,{0XC0}},
{0XB1,1,{0X70}},
{0X1E,1,{0X11}},

{0XFF,3,{0X98,0X81,0X02}},
{0X01,1,{0X50}},     //SDT=3us 
{0X15,1,{0X10}},    //timeout black
{0X42,1,{0X01}},   //2/3-power mode
{0X44,1,{0X01}},

{0X57,20,{0X00,0X1B,0X2B,0X13,0X16,0X29,0X1E,0X1F,0X90,0X1E,0X2B,0X79,0X18,0X12,0X41,0X1E,0X26,0X4E,0X5D,0X2C}},
{0X6B,20,{0X00,0X1B,0X2B,0X13,0X16,0X29,0X1E,0X1F,0X90,0X1E,0X2B,0X79,0X18,0X12,0X41,0X1E,0X26,0X4E,0X5D,0X2C}},

{0XFF,3,{0X98,0X81,0X01}},
{0X01,1,{0X00}},  //                                                                    
{0X02,1,{0X00}},                                                                        
{0X03,1,{0X56}},  //STV_A rise[9:8]=01, phase_stv_a=2, overlap_stv_a=6H                 
{0X04,1,{0X13}},  //phase_stv_b=2, overlap_stv_b=4H                                     
{0X05,1,{0X13}},  //phase_stv_c=2, overlap_stv_c=4H                                     
{0X06,1,{0X0a}},  //STV_A rise[7:0]                                                     
{0X07,1,{0X05}},  //STV_B rise[7:0]                                                     
{0X08,1,{0X05}},  //STV_C rise[7:0]                                                     
{0X09,1,{0X1D}},  //fti_1_rise:STV                                                      
{0X0a,1,{0X01}},  //fti_2_rise                                                          
{0X0b,1,{0X00}},  //fti_3_rise                                                          
{0X0c,1,{0X3F}},  //fti_1_fall                                                          
{0X0d,1,{0X29}},  //fti_2_fall                                                          
{0X0e,1,{0X29}},  //fti_3_fall                                                          
{0X0f,1,{0X1D}},  //clw_1_rise  29                                                      
{0X10,1,{0X1D}},  //clw_2_rise  29                                                      
{0X11,1,{0X00}},  //clwb_1_rise                                                         
{0X12,1,{0X00}},  //clwb_2_rise                                                         
{0X13,1,{0X08}},  //clw_x_fall                                                          
{0X14,1,{0X08}},  //clwb_x_fall                                                         
{0X15,1,{0X00}},                                                                        
{0X16,1,{0X00}},                                                                        
{0X17,1,{0X00}},                                                                        
{0X18,1,{0X00}},                                                                        
{0X19,1,{0X00}},                                                                        
{0X1a,1,{0X00}},                                                                        
{0X1b,1,{0X00}},                                                                        
{0X1c,1,{0X00}},                                                                        
{0X1d,1,{0X00}},                                                                        
{0X1e,1,{0X40}},  //clk_a_rise[10:8]                                                    
{0X1f,1,{0X88}},  //panda lvd_option, clk_b_rise[10:8], panda_back_vgh_option           
{0X20,1,{0X08}},  //clk_a_rise[7:0]                                                     
{0X21,1,{0X01}},  //clk_a_fall[7:0]                                                     
{0X22,1,{0X00}},  //clk_b_rise[7:0]                                                     
{0X23,1,{0X00}},  //clk_b_fall[7:0]                                                     
{0X24,1,{0X00}},  //clk_keep_pos1[7:0]                                                  
{0X25,1,{0X00}},  //clk_keep_pos2[7:0]                                                  
{0X26,1,{0X00}},  //clkb_keep_pos1[7:0]                                                 
{0X27,1,{0X00}},  //clkb_keep_pos2[7:0]                                                 
{0X28,1,{0X33}},  //clk_x_num[2:0]=8, phase_clk[2:0]=8                                  
{0X29,1,{0X03}},  //overlap_clk[3:0]=4H                                                 
{0X2a,1,{0X00}},                                                                        
{0X2b,1,{0X00}},                                                                        
{0X2c,1,{0X00}},                                                                        
{0X2d,1,{0X00}},                                                                        
{0X2e,1,{0X00}},                                                                        
{0X2f,1,{0X00}},                                                                        
{0X30,1,{0X00}},                                                                        
{0X31,1,{0X00}},                                                                        
{0X32,1,{0X00}},                                                                        
{0X33,1,{0X00}},                                                                        
{0X34,1,{0X00}},                                                                        
{0X35,1,{0X00}},                                                                        
{0X36,1,{0X00}},                                                                        
{0X37,1,{0X00}},                                                                        
{0X38,1,{0X00}},                                                                        
{0X39,1,{0X0f}},  //post_timing_en, gip_sr_vgl[0], gip_sr_vgh_[1:0]                     
{0X3a,1,{0X2a}},                                                                        
{0X3b,1,{0X00}},                                                                        
{0X3c,1,{0X00}},                                                                        
{0X3d,1,{0X00}},                                                                        
{0X3e,1,{0X00}},                                                                        
{0X3f,1,{0X00}},                                                                        
{0X40,1,{0X00}},                                                                        
{0X41,1,{0Xe0}},                                                                        
{0X42,1,{0X40}},                                                                        
{0X43,1,{0X0f}},                                                                        
{0X44,1,{0X11}},  //panda timing, default=15                                            
{0X45,1,{0Xa8}},                                                                        
{0X46,1,{0X00}},                                                                        
{0X47,1,{0X08}},                                                                        
{0X48,1,{0X00}},                                                                        
{0X49,1,{0X01}},                                                                        
{0X4a,1,{0X00}},                                                                        
{0X4b,1,{0X00}},

// ==== GOUT_BW_L[3:0] ====                                                                        
{0X4c,1,{0Xb2}},   //STVB2            
{0X4d,1,{0X22}},                      
{0X4e,1,{0X01}},   //VGL/VGH          
{0X4f,1,{0Xf7}},   //STVC2/STVA2      
{0X50,1,{0X29}},   //L9=19=CK8=CLK6   
{0X51,1,{0X72}},   //L12=17=CK6=CLK4  
{0X52,1,{0X25}},   //L13=15=CK4=CLK2  
{0X53,1,{0Xb2}},   //L16=1B=CK2=CLK8  
{0X54,1,{0X22}},
{0X55,1,{0X22}},
{0X56,1,{0X22}},

// ==== GOUT_BW_R[3:0] ====
{0X57,1,{0Xa2}},    //STVB1                
{0X58,1,{0X22}},                           
{0X59,1,{0X01}},                           
{0X5a,1,{0Xe6}},    //STVC1/STVA1          
{0X5b,1,{0X28}},    //R9=CK7=CLK5          
{0X5c,1,{0X62}},    //R12=CK5=CLK3         
{0X5d,1,{0X24}},    //R13=CK3=CLK1         
{0X5e,1,{0Xa2}},    //R16=CK1=CLK7         
{0X5f,1,{0X22}},                           
{0X60,1,{0X22}},                           
{0X61,1,{0X22}},  
                         
{0X62,1,{0Xee}},
// ==== GOUT_FW_L ==== 
{0X63,1,{0X02}},
{0X64,1,{0X0b}},
{0X65,1,{0X02}},
{0X66,1,{0X02}},
{0X67,1,{0X01}},
{0X68,1,{0X00}},
{0X69,1,{0X0f}},
{0X6a,1,{0X07}},
{0X6b,1,{0X55}},    //L9=CK8=CLK2      
{0X6c,1,{0X02}},                       
{0X6d,1,{0X02}},                       
{0X6e,1,{0X5b}},    //L12=CK6=CLK8     
{0X6f,1,{0X59}},    //L13=CK4=CLK6     
{0X70,1,{0X02}},                       
{0X71,1,{0X02}},                       
{0X72,1,{0X57}},    //L16=CK2=CLK4     
{0X73,1,{0X02}},
{0X74,1,{0X02}},
{0X75,1,{0X02}},
{0X76,1,{0X02}},
{0X77,1,{0X02}},
{0X78,1,{0X02}},
// ==== GOUT_FW_R ====
{0X79,1,{0X02}},
{0X7a,1,{0X0a}},
{0X7b,1,{0X02}},
{0X7c,1,{0X02}},
{0X7d,1,{0X01}},
{0X7e,1,{0X00}},
{0X7f,1,{0X0e}},
{0X80,1,{0X06}},
{0X81,1,{0X54}},    //R9=CK7=CLK1  
{0X82,1,{0X02}},                   
{0X83,1,{0X02}},                   
{0X84,1,{0X5a}},    //R12=CK5=CLK7 
{0X85,1,{0X58}},    //R13=CK3=CLK5 
{0X86,1,{0X02}},                   
{0X87,1,{0X02}},                   
{0X88,1,{0X56}},    //R16=CK1=CLK3 
{0X89,1,{0X02}},
{0X8a,1,{0X02}},
{0X8b,1,{0X02}},
{0X8c,1,{0X02}},
{0X8d,1,{0X02}},
{0X8e,1,{0X02}}, 
 
{0X8f,1,{0X44}},
{0X90,1,{0X44}},

{0XFF,3,{0X98,0X81,0X06}},
{0X01,1,{0X03}},         //LEDPWM/SDO hi-z  
{0X2B,1,{0X0A}},          //BGR_PANEL+SS_PANEL BW:09  
{0X04,1,{0X71}},          //70£º4lane, 71£º3lane,   73£º2lane, 

{0XC0,1,{0XCF}},       //720*1440
{0XC1,1,{0X2A}},       //720*1440

{0XFF,3,{0X98,0X81,0X00}},
{0X35,1,{0X00}},      
{0X36,1,{0X00}},      

		 //sleep out 
		{0x11,1,{0x00}},       
		{REGFLAG_DELAY, 150, {}},
		// Display ON            
		{0x29, 1, {0x00}},       
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
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



/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}


#ifndef BUILD_LK
__maybe_unused static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	
	{0xFF,0x03,{0x98,0x81,0x00}},
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},
    // Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
__maybe_unused static struct LCM_setting_table lcm_deep_sleep_mode_out_setting[] = {
	
	{0xFF,0x03,{0x98,0x81,0x00}},
	// Display off sequence
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},

    // Sleep Mode On
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif
static void lcm_get_params(struct LCM_PARAMS *params)
{

		memset(params, 0, sizeof(struct LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;
		params->density = 320;//LCM_DENSITY;


#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = BURST_VDO_MODE;
#endif
		params->dsi.vertical_sync_active = 10;
		params->dsi.vertical_backporch = 20;
		params->dsi.vertical_frontporch = 20;
		params->dsi.vertical_active_line = FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 40;	
		params->dsi.horizontal_backporch				= 110;//50
		params->dsi.horizontal_frontporch				= 110;  //30 
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		params->dsi.LANE_NUM = LCM_THREE_LANE;
		
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
		params->dsi.ssc_disable				=1;
		
		params->dsi.packet_size=256;
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		// Video mode setting		
		params->dsi.intermediat_buffer_num = 2;


		// params->dsi.HS_TRAIL=20; 
		params->dsi.PLL_CLOCK = 355; //320 20200424 //240=4db  230=1.5db
/***********************    esd  check   ***************************/
#ifndef BUILD_LK
		params->dsi.esd_check_enable = 1;
		params->dsi.customization_esd_check_enable = 1;
		params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
		params->dsi.lcm_esd_check_table[0].count        = 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
#endif

}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
#ifndef BUILD_LK
		//printk("[PRIZE KERNEL] %s, line =%d\n",__FUNCTION__,__LINE__);
		push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
		SET_RESET_PIN(0);
#else
		printf("[PRIZE LK] %s, line =%d\n",__FUNCTION__,__LINE__);
#endif
}


static void lcm_resume(void)
{
#ifndef BUILD_LK
		//printk("[PRIZE KERNEL] %s, line =%d\n",__FUNCTION__,__LINE__);
		lcm_init();
#else
		printf("[PRIZE LK] %s, line =%d\n",__FUNCTION__,__LINE__);
#endif
}

static unsigned int lcm_compare_id(void)
{
		return 0;
}


struct LCM_DRIVER ili9881c_hdplus_dsi_vdo_cmi_bx_lcm_drv = {
	.name		= "ili9881c_hdplus_dsi_vdo_cmi_bx",
	#if defined(CONFIG_PRIZE_HARDWARE_INFO)
	.lcm_info = {
		.chip	= "ili9881p",
		.vendor	= "baoxu",
		.id		= "0x9881",
		.more	= "lcm_1440*720",
	},
	#endif
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id 	= lcm_compare_id,
};


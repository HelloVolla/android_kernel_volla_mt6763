#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
//#include <mach/mt_gpio.h>
//#include <linux/xlog.h>
//#include <mach/mt_pm_ldo.h>
#endif


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1560)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif

extern int display_ldo18_enable(int enable);
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static struct LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
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
	
{0xFF,0x03,{0x98,0x81,0x03}},    
{0x01,0x01,{0x00}},
{0x02,0x01,{0x00}},
{0x03,0x01,{0x53}},
{0x04,0x01,{0x13}},
{0x05,0x01,{0x00}},
{0x06,0x01,{0x04}},
{0x07,0x01,{0x00}},
{0x08,0x01,{0x00}},
{0x09,0x01,{0x10}},
{0x0a,0x01,{0x21}},
{0x0b,0x01,{0x00}},
{0x0c,0x01,{0x01}},
{0x0d,0x01,{0x00}},
{0x0e,0x01,{0x00}},
{0x0f,0x01,{0x22}},
{0x10,0x01,{0x22}},
{0x11,0x01,{0x00}},
{0x12,0x01,{0x00}},
{0x13,0x01,{0x00}},
{0x14,0x01,{0x00}},
{0x15,0x01,{0x08}},
{0x16,0x01,{0x08}},
{0x17,0x01,{0x00}},
{0x18,0x01,{0x08}},
{0x19,0x01,{0x00}},
{0x1a,0x01,{0x00}},
{0x1b,0x01,{0x00}},
{0x1c,0x01,{0x00}},
{0x1d,0x01,{0x00}},
{0x1e,0x01,{0x44}},
{0x1f,0x01,{0x80}},
{0x20,0x01,{0x02}},
{0x21,0x01,{0x03}},
{0x22,0x01,{0x00}},
{0x23,0x01,{0x00}},
{0x24,0x01,{0x00}},
{0x25,0x01,{0x00}},
{0x26,0x01,{0x00}},
{0x27,0x01,{0x00}},
{0x28,0x01,{0x33}}, 
{0x29,0x01,{0x03}},
{0x2a,0x01,{0x00}},    
{0x2b,0x01,{0x00}},    
{0x2c,0x01,{0x00}},    
{0x2d,0x01,{0x00}},    
{0x2e,0x01,{0x00}},    
{0x2f,0x01,{0x00}},    
{0x30,0x01,{0x00}},    
{0x31,0x01,{0x00}},    
{0x32,0x01,{0x00}},    
{0x33,0x01,{0x00}},    
{0x34,0x01,{0x04}},    
{0x35,0x01,{0x00}},    
{0x36,0x01,{0x00}},    
{0x37,0x01,{0x00}},    
{0x38,0x01,{0x3C}},    
{0x39,0x01,{0x35}},
{0x3A,0x01,{0x01}},
{0x3B,0x01,{0x40}},
{0x3C,0x01,{0x00}},
{0x3D,0x01,{0x01}},
{0x3E,0x01,{0x00}},
{0x3F,0x01,{0x00}},
{0x40,0x01,{0x00}},
{0x41,0x01,{0x88}},
{0x42,0x01,{0x00}},
{0x43,0x01,{0x00}},
{0x44,0x01,{0x3F}},     
{0x45,0x01,{0x20}},     
{0x46,0x01,{0x00}},
{0x50,0x01,{0x01}},    
{0x51,0x01,{0x23}},    
{0x52,0x01,{0x45}},    
{0x53,0x01,{0x67}},    
{0x54,0x01,{0x89}},     
{0x55,0x01,{0xab}},    
{0x56,0x01,{0x01}},    
{0x57,0x01,{0x23}},    
{0x58,0x01,{0x45}},    
{0x59,0x01,{0x67}},    
{0x5a,0x01,{0x89}},    
{0x5b,0x01,{0xab}},    
{0x5c,0x01,{0xcd}},    
{0x5d,0x01,{0xef}},             
{0x5e,0x01,{0x11}},             
{0x5f,0x01,{0x01}},    
{0x60,0x01,{0x00}},    
{0x61,0x01,{0x15}},    
{0x62,0x01,{0x14}},    
{0x63,0x01,{0x0C}},    
{0x64,0x01,{0x0D}},    
{0x65,0x01,{0x0E}},    
{0x66,0x01,{0x0F}},    
{0x67,0x01,{0x06}},    
{0x68,0x01,{0x02}},    
{0x69,0x01,{0x02}},    
{0x6a,0x01,{0x02}},    
{0x6b,0x01,{0x02}},    
{0x6c,0x01,{0x02}},    
{0x6d,0x01,{0x02}},      
{0x6e,0x01,{0x08}},
{0x6f,0x01,{0x02}},    
{0x70,0x01,{0x02}},    
{0x71,0x01,{0x02}},    
{0x72,0x01,{0x02}},    
{0x73,0x01,{0x02}},    
{0x74,0x01,{0x02}},             
{0x75,0x01,{0x01}},    
{0x76,0x01,{0x00}},    
{0x77,0x01,{0x15}},    
{0x78,0x01,{0x14}},    
{0x79,0x01,{0x0C}},    
{0x7a,0x01,{0x0D}},
{0x7b,0x01,{0x0E}}, 
{0x7c,0x01,{0x0F}},    
{0x7D,0x01,{0x08}},  
{0x7E,0x01,{0x02}},    
{0x7F,0x01,{0x02}},    
{0x80,0x01,{0x02}},    
{0x81,0x01,{0x02}},    
{0x82,0x01,{0x02}},    
{0x83,0x01,{0x02}},    
{0x84,0x01,{0x06}},   
{0x85,0x01,{0x02}},    
{0x86,0x01,{0x02}},    
{0x87,0x01,{0x02}},    
{0x88,0x01,{0x02}},    
{0x89,0x01,{0x02}},    
{0x8A,0x01,{0x02}},              
{0xFF,0x03,{0x98,0x81,0x04}}, 
{0x68,0x01,{0xDB}},                
{0x66,0x01,{0xFE}},     
{0x3A,0x01,{0x24}},     
{0x82,0x01,{0x10}},     
{0x84,0x01,{0x10}},     
{0x85,0x01,{0x0D}},     
{0x32,0x01,{0xAC}},     
{0x8C,0x01,{0x80}},     
{0x3C,0x01,{0xF5}},     
{0x3A,0x01,{0x24}},     
{0xB5,0x01,{0x02}},     
{0x31,0x01,{0x25}},     
{0x88,0x01,{0x33}},     
{0x89,0x01,{0xBA}},     
{0x8D,0x01,{0x00}}, 
{0xFF,0x03,{0x98,0x81,0x01}},    
{0x22,0x01,{0x0A}},      
{0x31,0x01,{0x00}},     
{0x50,0x01,{0x7F}},     
{0x51,0x01,{0x7F}},     
{0x53,0x01,{0x60}}, //79  65  
{0x55,0x01,{0x60}}, //7c    
{0x60,0x01,{0x28}},     
{0x61,0x01,{0x00}},     
{0x62,0x01,{0x0D}},     
{0x63,0x01,{0x00}},     
{0x2E,0x01,{0x0E}},     
{0x2F,0x01,{0x00}},     
{0xF6,0x01,{0x01}},        
{0xA0,0x01,{0x00}},
{0xA1,0x01,{0x17}},
{0xA2,0x01,{0x26}},
{0xA3,0x01,{0x16}},
{0xA4,0x01,{0x1B}},
{0xA5,0x01,{0x2F}},
{0xA6,0x01,{0x23}},
{0xA7,0x01,{0x23}},
{0xA8,0x01,{0x88}},
{0xA9,0x01,{0x1B}},
{0xAA,0x01,{0x27}},
{0xAB,0x01,{0x6D}},
{0xAC,0x01,{0x1A}},
{0xAD,0x01,{0x19}},
{0xAE,0x01,{0x4E}},
{0xAF,0x01,{0x24}},
{0xB0,0x01,{0x29}},
{0xB1,0x01,{0x4B}},
{0xB2,0x01,{0x5A}},
{0xB3,0x01,{0x2B}},            
{0xC0,0x01,{0x00}},
{0xC1,0x01,{0x17}},
{0xC2,0x01,{0x26}},
{0xC3,0x01,{0x16}},
{0xC4,0x01,{0x1B}},
{0xC5,0x01,{0x2F}},
{0xC6,0x01,{0x23}},
{0xC7,0x01,{0x23}},
{0xC8,0x01,{0x88}},
{0xC9,0x01,{0x1B}},
{0xCA,0x01,{0x27}},
{0xCB,0x01,{0x6D}},
{0xCC,0x01,{0x1A}},
{0xCD,0x01,{0x19}},
{0xCE,0x01,{0x4E}},
{0xCF,0x01,{0x24}},
{0xD0,0x01,{0x29}},
{0xD1,0x01,{0x4B}},
{0xD2,0x01,{0x5A}},
{0xD3,0x01,{0x2B}},   
{0xFF,0x03,{0x98,0x81,0x00}},   
{0x35,0x01,{0x00}},  
{0x11,0x01,{0x00}},
{REGFLAG_DELAY, 120, {}},  
{0x29,0x01,{0x00}},
{REGFLAG_DELAY, 1, {}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};

/*

static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};



static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0xFF,0x03,{0x98,0x81,0x00}},
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 1, {}},

    // Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

/*

static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/

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

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	
	//5.5 18:9 62476 124951,16:9 68489 121759
//	params->physical_width = 68;//LCM_PHYSICAL_WIDTH/1000;
//	params->physical_height = 122;//LCM_PHYSICAL_HEIGHT/1000;
//	params->physical_width_um = 68489;//LCM_PHYSICAL_WIDTH;	= sqrt((size*25.4)^2/(18^2+9^2))*9*1000
//	params->physical_height_um = 121759;//LCM_PHYSICAL_HEIGHT; = sqrt((size*25.4)^2/(18^2+9^2))*18*1000
	params->density = 320;//LCM_DENSITY;

	// enable tearing-free
	params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM = LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size=256;

	// Video mode setting		
	params->dsi.intermediat_buffer_num = 2;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
                /*d.j modify 2015.02.13*/
		params->dsi.vertical_sync_active				 = 6;
		params->dsi.vertical_backporch					  = 16;
		params->dsi.vertical_frontporch 				   = 16;
		params->dsi.vertical_active_line				= FRAME_HEIGHT;

		params->dsi.horizontal_sync_active				  = 30;
		params->dsi.horizontal_backporch				= 60;
		params->dsi.horizontal_frontporch				 = 60;
		params->dsi.horizontal_active_pixel 			   = FRAME_WIDTH;

        params->dsi.PLL_CLOCK = 208; //this value must be in MTK suggested table  212

}


static void lcm_init(void)
{
	display_ldo18_enable(1);
	MDELAY(1);
    SET_RESET_PIN(1);
    MDELAY(6);
    SET_RESET_PIN(0);
    MDELAY(1);
    SET_RESET_PIN(1);
    MDELAY(12);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{

	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	//SET_RESET_PIN(1);
	//MDELAY(1);
	//SET_RESET_PIN(0);
	//MDELAY(1);
	//SET_RESET_PIN(1);
	//MDELAY(120);
	display_ldo18_enable(0);
}


static void lcm_resume(void)
{
	lcm_init();
	
//	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

/*
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	data_array[3]= 0x00053902;
	data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[5]= (y1_LSB);
	data_array[6]= 0x002c3909;

	dsi_set_cmdq(data_array, 7, 0);

}


static void lcm_setbacklight(unsigned int level)
{
	unsigned int default_level = 145;
	unsigned int mapped_level = 0;

	//for LGE backlight IC mapping table
	if(level > 255) 
			level = 255;

	if(level >0) 
			mapped_level = default_level+(level)*(255-default_level)/(255);
	else
			mapped_level=0;

	// Refresh value of backlight level.
	lcm_backlight_level_setting[0].para_list[0] = mapped_level;

	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}

*/

//prize-chenxiaowen-20161013-compatibility-(ili9881c_hd720_dsi_vdo_yin vs ili9881c_hd720_dsi_vdo_cmi)-start
#if defined(ILI9881C_HD720_DSI_VDO_YIN) && defined(ILI9881C_HD720_DSI_VDO_CMI)
#define AUXADC_LCM_VOLTAGE_CHANNEL 12
#define MIN_VOLTAGE (600)
#define MAX_VOLTAGE (1200)

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
#endif
//prize-chenxiaowen-20161013-compatibility-(ili9881c_hd720_dsi_vdo_yin vs ili9881c_hd720_dsi_vdo_cmi)-end

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
	//unsigned char y0_MSB = ((y0>>8)&0xFF);
	//unsigned char y0_LSB = (y0&0xFF);
	//unsigned char y1_MSB = ((y1>>8)&0xFF);
	//unsigned char y1_LSB = (y1&0xFF);
	
unsigned int data_array[6]; 
unsigned char read_buf[4]; 
unsigned char read_buf1[4]; 
unsigned char read_buf2[4]; 
unsigned char read_buf3[4]; 
#ifdef BUILD_LK 
printf("ATA check kernel size = 0x%x,0x%x,0x%x,0x%x\n",x0_MSB,x0_LSB,x1_MSB,x1_LSB); 
#else 
printk("ATA check kernel size = 0x%x,0x%x,0x%x,0x%x\n",x0_MSB,x0_LSB,x1_MSB,x1_LSB); 
#endif 
//write page frist lhr
//data_array[0]= 0x0002150A;//HS packet 
//data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x51; 
//data_array[2]= (x1_LSB); 
//dsi_set_cmdq(data_array, 3, 1); 
   
data_array[0]= 0x0002390A;//HS packet 
data_array[1]= 0x00002453; 
//data_array[2]= (x1_LSB); 
dsi_set_cmdq(data_array, 2, 1); 
    
 data_array[0]= 0x0002390A;//HS packet 
data_array[1]= 0x0000F05e; 
dsi_set_cmdq(data_array, 2, 1); 
data_array[0]= 0x0002390A;//HS packet 
data_array[1]= 0x00000151; 
dsi_set_cmdq(data_array, 2, 1); 
data_array[0]= 0x0002390A;//HS packet 
data_array[1]= 0x00000355; 
//data_array[2]= (x1_LSB); 
dsi_set_cmdq(data_array, 2, 1); 
 
//data_array[0]= 0x0002150A;//HS packet 
//data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x51; 
//data_array[2]= (x1_LSB); 
//dsi_set_cmdq(data_array, 3, 1); 
 
data_array[0] = 0x00013700; 
dsi_set_cmdq(data_array, 1, 1); 
atomic_set(&ESDCheck_byCPU, 1);
read_reg_v2(0X52, read_buf, 1); 
read_reg_v2(0X56, read_buf1, 1); 
read_reg_v2(0X54, read_buf2, 1); 
read_reg_v2(0X5F, read_buf3, 1);
atomic_set(&ESDCheck_byCPU, 0);
if((read_buf1[0] == 0x03) && (read_buf2[0] == 0x24) /*&& (read_buf3[0] == 0xf0)*/) 
   ret = 1; 
else 
    ret = 0; 
#ifdef BUILD_LK 
printf("ATA read buf kernel size = 0x%x,0x%x,0x%x,0x%x,ret= %d\n",read_buf[0],read_buf[1],read_buf[2],read_buf[3],ret); 
#else 
printk("ATA read buf kernel size = 0x%x,0x%x,0x%x,0x%x,ret= %d ret1= %d\n",read_buf[0],read_buf1[0],read_buf2[0],read_buf3[0],ret,ret1); 
printk("ATA read buf new kernel size = 0x%x,0x%x,0x%x,0x%x,ret= %d ret1= %d\n",read_buf1[0],read_buf1[1],read_buf1[2],read_buf1[3],ret,ret1); 
//printk("ATA read buf new kernel size = 0x%x,0x%x,0x%x,0x%x,ret= %d ret1= %d\n",read_buf1,read_buf1,read_buf1,read_buf1,ret,ret1); 
#endif 
return ret; 
#endif 
#endif
}

static unsigned int lcm_compare_id(void)
{
	#define LCM_ID 0x9881
   unsigned int array[4];
	unsigned char buffer[3];
	unsigned int id=0;	

	SET_RESET_PIN(1);
	MDELAY(10); 
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(120);

	array[0] = 0x00043902;
	array[1] = 0x068198FF;
	dsi_set_cmdq(array, 2, 1);

	array[0] = 0x00013700; //return byte number
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xf0, &buffer[0], 1);  //0x98

	array[0] = 0x00013700; //return byte number
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xf1, &buffer[1], 1);  //0x81

	id = (buffer[0]<<8) | buffer[1];
	printk("tangcong id is 0x%x %x %x\n",id,buffer[0],buffer[1]);
	return (id == LCM_ID) ? 1 : 0;
}

struct LCM_DRIVER ili9881d_hdplus1560_dsi_vdo_hsd_lcm_drv = {
	.name		= "ili9881d_hdplus1560_dsi_vdo_hsd",
    	//prize-lixuefeng-20150512-start
	#if defined(CONFIG_PRIZE_HARDWARE_INFO) && !defined (BUILD_LK)
	.lcm_info = {
		.chip	= "ili9881d",
		.vendor	= "unknow",
		.id		= "0x9881",
		.more	= "lcm_1560*720",
	},
	#endif
	//prize-lixuefeng-20150512-end	
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id 	= lcm_compare_id,
	
#if (LCM_DSI_CMD_MODE)
	.set_backlight	= lcm_setbacklight,
    .update         = lcm_update,
#endif
	.ata_check	= lcm_ata_check
};


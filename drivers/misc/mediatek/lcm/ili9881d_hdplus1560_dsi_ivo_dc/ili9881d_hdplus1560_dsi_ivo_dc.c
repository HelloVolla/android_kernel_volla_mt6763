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

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

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
{0xFF,3,{0x98,0x81,0x03}},

{0x01,1,{0x00}},
{0x02,1,{0x00}},
{0x03,1,{0x73}},
{0x04,1,{0x73}},
{0x05,1,{0x00}},
{0x06,1,{0x06}},
{0x07,1,{0x02}},
{0x08,1,{0x00}},
{0x09,1,{0x01}},
{0x0a,1,{0x01}},
{0x0b,1,{0x01}},
{0x0c,1,{0x01}},
{0x0d,1,{0x01}},
{0x0e,1,{0x01}},
{0x0f,1,{0x00}},
{0x10,1,{0x00}},
{0x11,1,{0x00}},
{0x12,1,{0x00}},
{0x13,1,{0x01}},
{0x14,1,{0x00}},
{0x15,1,{0x00}},
{0x16,1,{0x00}},
{0x17,1,{0x00}},
{0x18,1,{0x00}},
{0x19,1,{0x00}},
{0x1a,1,{0x00}},
{0x1b,1,{0x00}},
{0x1c,1,{0x00}},
{0x1d,1,{0x00}},
{0x1e,1,{0xC0}},
{0x1f,1,{0x80}},
{0x20,1,{0x03}},
{0x21,1,{0x04}},
{0x22,1,{0x00}},
{0x23,1,{0x00}},
{0x24,1,{0x00}},
{0x25,1,{0x00}},
{0x26,1,{0x00}},
{0x27,1,{0x00}},
{0x28,1,{0x33}},
{0x29,1,{0x02}},
{0x2a,1,{0x00}},
{0x2b,1,{0x00}},
{0x2c,1,{0x00}},
{0x2d,1,{0x00}},
{0x2e,1,{0x00}},
{0x2f,1,{0x00}},
{0x30,1,{0x00}},
{0x31,1,{0x00}},
{0x32,1,{0x00}},
{0x33,1,{0x00}},
{0x34,1,{0x03}},
{0x35,1,{0x00}},
{0x36,1,{0x03}},
{0x37,1,{0x00}},
{0x38,1,{0x00}},
{0x39,1,{0x35}},
{0x3A,1,{0x01}},
{0x3B,1,{0x40}},
{0x3C,1,{0x00}},
{0x3D,1,{0x00}},
{0x3E,1,{0x00}},
{0x3F,1,{0x00}},
{0x40,1,{0x00}},
{0x41,1,{0x88}},
{0x42,1,{0x00}},
{0x43,1,{0x00}},
{0x44,1,{0x3F}},
{0x45,1,{0x20}},
{0x46,1,{0x00}},

{0x50,1,{0x01}},
{0x51,1,{0x23}},
{0x52,1,{0x45}},
{0x53,1,{0x67}},
{0x54,1,{0x89}},
{0x55,1,{0xab}},
{0x56,1,{0x01}},
{0x57,1,{0x23}},
{0x58,1,{0x45}},
{0x59,1,{0x67}},
{0x5a,1,{0x89}},
{0x5b,1,{0xab}},
{0x5c,1,{0xcd}},
{0x5d,1,{0xef}},

{0x5e,1,{0x10}},
{0x5f,1,{0x09}},
{0x60,1,{0x08}},
{0x61,1,{0x02}},
{0x62,1,{0x02}},
{0x63,1,{0x02}},
{0x64,1,{0x0F}},
{0x65,1,{0x0E}},
{0x66,1,{0x0D}},
{0x67,1,{0x0C}},
{0x68,1,{0x02}},
{0x69,1,{0x02}},
{0x6a,1,{0x02}},
{0x6b,1,{0x02}},
{0x6c,1,{0x02}},
{0x6d,1,{0x02}},
{0x6e,1,{0x02}},
{0x6f,1,{0x02}},
{0x70,1,{0x02}},
{0x71,1,{0x06}},
{0x72,1,{0x07}},
{0x73,1,{0x02}},
{0x74,1,{0x02}},

{0x75,1,{0x06}},
{0x76,1,{0x07}},
{0x77,1,{0x02}},
{0x78,1,{0x02}},
{0x79,1,{0x02}},
{0x7a,1,{0x0E}},
{0x7b,1,{0x0F}},
{0x7c,1,{0x0C}},
{0x7d,1,{0x0D}},
{0x7e,1,{0x02}},
{0x7f,1,{0x02}},
{0x80,1,{0x02}},
{0x81,1,{0x02}},
{0x82,1,{0x02}},
{0x83,1,{0x02}},
{0x84,1,{0x02}},
{0x85,1,{0x02}},
{0x86,1,{0x02}},
{0x87,1,{0x09}},
{0x88,1,{0x08}},
{0x89,1,{0x02}},
{0x8A,1,{0x02}},

{0xFF,3,{0x98,0x81,0x04}},

{0x6D,1,{0x08}},
{0x6F,1,{0x05}},
{0x70,1,{0x00}},
{0x71,1,{0x00}},
{0x8D,1,{0x00}},
{0x66,1,{0xFE}},
{0x82,1,{0x17}},
{0x84,1,{0x17}},
{0x85,1,{0x10}},
{0x32,1,{0xAC}},
{0x8C,1,{0x80}},
{0x3C,1,{0xF5}},
{0x3A,1,{0x24}},
{0xB5,1,{0x07}},
{0x31,1,{0x45}},
{0x88,1,{0x33}},
{0x89,1,{0xBA}},
{0x38,1,{0x01}},
{0x39,1,{0x00}},

{0xFF,3,{0x98,0x81,0x01}},

{0x22,1,{0x0A}},
{0x31,1,{0x00}},
{0x41,1,{0x24}},
{0x53,1,{0x33}},
{0x55,1,{0x33}},
{0x50,1,{0x6B}},
{0x51,1,{0x6B}},
{0x60,1,{0x1D}},
{0x61,1,{0x00}},
{0x62,1,{0x0D}},
{0x63,1,{0x00}},
{0x2E,1,{0x0E}},
{0xF6,1,{0x01}},
{0x2F,1,{0x00}},

{0xA0,1,{0x00}},
{0xA1,1,{0x1C}},
{0xA2,1,{0x2E}},
{0xA3,1,{0x18}},
{0xA4,1,{0x1E}},
{0xA5,1,{0x33}},
{0xA6,1,{0x27}},
{0xA7,1,{0x26}},
{0xA8,1,{0xAA}},
{0xA9,1,{0x1D}},
{0xAA,1,{0x28}},
{0xAB,1,{0x8C}},
{0xAC,1,{0x1C}},
{0xAD,1,{0x1D}},
{0xAE,1,{0x53}},
{0xAF,1,{0x27}},
{0xB0,1,{0x2B}},
{0xB1,1,{0x51}},
{0xB2,1,{0x5C}},
{0xB3,1,{0x23}},

{0xC0,1,{0x00}},
{0xC1,1,{0x1C}},
{0xC2,1,{0x2E}},
{0xC3,1,{0x18}},
{0xC4,1,{0x1E}},
{0xC5,1,{0x33}},
{0xC6,1,{0x27}},
{0xC7,1,{0x26}},
{0xC8,1,{0xAA}},
{0xC9,1,{0x1D}},
{0xCA,1,{0x28}},
{0xCB,1,{0x8C}},
{0xCC,1,{0x1C}},
{0xCD,1,{0x1D}},
{0xCE,1,{0x53}},
{0xCF,1,{0x27}},
{0xD0,1,{0x2B}},
{0xD1,1,{0x51}},
{0xD2,1,{0x5C}},
{0xD3,1,{0x23}},

{0xFF,3,{0x98,0x81,0x00}},
{0x35,1,{0x00}},
	
{0x11, 1, {0x00}},
{REGFLAG_DELAY, 120, {}},
{0x29, 1, {0x00}},
{REGFLAG_DELAY, 20, {}},
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

	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},

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

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

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
    params->dbi.te_mode                  = LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
    params->dsi.switch_mode              = SYNC_PULSE_VDO_MODE;
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
	params->dsi.vertical_sync_active				= 20;//2;
	params->dsi.vertical_backporch					= 20;//8;
	params->dsi.vertical_frontporch					= 20;  // rom Q driver
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 40;//10;
	params->dsi.horizontal_backporch				= 50;//20;
	params->dsi.horizontal_frontporch				= 40;//40;
	params->dsi.horizontal_active_pixel			= FRAME_WIDTH;

    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable = 1;
    params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
    params->dsi.lcm_esd_check_table[0].count        = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
	params->dsi.PLL_CLOCK = 220;//236;//240; //this value must be in MTK suggested table 224 235 241 245 231 251 238 243 237
		//params->dsi.HS_TRAIL = 15;
}
static void lcm_set_bias(int enable)
{
	
	if (enable){
		display_bias_vpos_set(5500);
		display_bias_vneg_set(5500);
		display_bias_enable();
	}else{
		display_bias_disable();
		
	}
}
#define LCM_ID_ILI9881D 0x0098810d
static unsigned int lcm_compare_id(void)
{
    int array[4];
    char buffer[4]={0,0,0,0};
    char id_high=0;
    char id_mid=0;
    char id_low=0;
    int id=0;

    SET_RESET_PIN(1);
    MDELAY(1);
    SET_RESET_PIN(0);
    MDELAY(1);
    SET_RESET_PIN(1);
    MDELAY(120);

    array[0]=0x00043902;
    array[1]=0x068198ff;
    dsi_set_cmdq(array, 2, 1);

    MDELAY(10);
    array[0] = 0x00083700;
    dsi_set_cmdq(array, 1, 1);

    MDELAY(10);
    read_reg_v2(0xf0, &buffer[0], 1);//    NC 0x00  0x98 0x16
    MDELAY(10);
    read_reg_v2(0xf1, &buffer[1], 1);//    NC 0x00  0x98 0x16
    MDELAY(10);
    read_reg_v2(0xf2, &buffer[2], 1);//    NC 0x00  0x98 0x16

    id_high = buffer[0];
    id_mid = buffer[1];
    id_low = buffer[2];
    id = (id_high<<16) | (id_mid<<8) |id_low;


	#ifdef BUILD_LK
	printf(CRITICAL, "%s, LK debug: ili9881d id = 0x%08x\n", __func__, buffer);
	#else
	printk("%s: ili9881D id = 0x%08x \n", __func__, id);
	#endif

    return (LCM_ID_ILI9881D == id)?1:0;
}


static void lcm_init(void)
{
	display_ldo18_enable(1);
	lcm_set_bias(1);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);//250
	
	SET_RESET_PIN(0);
	MDELAY(10);//100
    SET_RESET_PIN(1);
    MDELAY(120);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{

	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	SET_RESET_PIN(0);
	MDELAY(10);
	
	lcm_set_bias(0);
	display_ldo18_enable(0);
}


static void lcm_resume(void)
{
	lcm_init();
	
//	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
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




LCM_DRIVER ili9881d_hdplus1560_dsi_ivo_dc_lcm_drv = {
	.name		= "ili9881d_hdplus1560_dsi_ivo_dc",
    	//prize-lixuefeng-20150512-start
	#if defined(CONFIG_PRIZE_HARDWARE_INFO) && !defined (BUILD_LK)
	.lcm_info = {
		.chip	= "ili9881d",
		.vendor	= "unknow",
		.id		= "0x98810d",
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
    .update         = lcm_update,
#endif
};


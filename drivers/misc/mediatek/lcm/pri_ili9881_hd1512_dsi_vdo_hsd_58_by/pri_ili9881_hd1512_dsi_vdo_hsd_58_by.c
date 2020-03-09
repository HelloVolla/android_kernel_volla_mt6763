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
//#include "pmic_drv.h"
//#include "mt6353_sw.h"

#ifdef CONFIG_MTK_PMIC_CHIP_MT6353
#include <mt-plat/upmu_common.h>
#endif
#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_notice("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

#define I2C_I2C_LCD_BIAS_CHANNEL 0
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
#define FRAME_HEIGHT 										(1512)



//#ifndef MACH_FPGA
//#define GPIO_65132_EN GPIO_LCD_BIAS_ENP_PIN
//#endif

#define REGFLAG_DELAY		0xFFFC
#define REGFLAG_UDELAY	0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW	    0xFFFE
#define REGFLAG_RESET_HIGH	0xFFFF


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
{0xFF,03,{0x98,0x81,0x01}},
{0x01,01,{0x00}},
{0x02,01,{0x00}},
{0x03,01,{0x53}},
{0x04,01,{0x13}},
{0x05,01,{0x00}},
{0x06,01,{0x04}},
{0x07,01,{0x00}},
{0x08,01,{0x00}},
{0x09,01,{0x20}},  //00
{0x0a,01,{0x20}},  //00
{0x0b,01,{0x20}},   //00
{0x0c,01,{0x00}},  //21
{0x0d,01,{0x00}},  //21
{0x0e,01,{0x00}},   //21
{0x0f,01,{0x20}},  //00
{0x10,01,{0x20}},  //00
{0x11,01,{0x00}},
{0x12,01,{0x00}},
{0x13,01,{0x00}},  //21
{0x14,01,{0x00}},
{0x15,01,{0x00}},
{0x16,01,{0x00}},
{0x17,01,{0x00}},
{0x18,01,{0x00}},
{0x19,01,{0x00}},
{0x1a,01,{0x00}},
{0x1b,01,{0x00}},
{0x1c,01,{0x00}},
{0x1d,01,{0x00}},
{0x1e,01,{0x44}},
{0x1f,01,{0x00}},
{0x20,01,{0x02}},
{0x21,01,{0x03}},
{0x22,01,{0x00}},
{0x23,01,{0x00}},
{0x24,01,{0x00}},
{0x25,01,{0x00}},
{0x26,01,{0x00}},
{0x27,01,{0x00}},
{0x28,01,{0x33}},
{0x29,01,{0x03}},
{0x2a,01,{0x00}},
{0x2b,01,{0x00}},
{0x2c,01,{0x00}},
{0x2d,01,{0x00}},
{0x2e,01,{0x00}},
{0x2f,01,{0x00}},
{0x30,01,{0x00}},
{0x31,01,{0x00}},
{0x32,01,{0x00}},
{0x33,01,{0x00}},
{0x34,01,{0x04}},
{0x35,01,{0x00}},
{0x36,01,{0x00}},
{0x37,01,{0x00}},
{0x38,01,{0x3C}},  //78
{0x39,01,{0x03}},
{0x3a,01,{0x00}},
{0x3b,01,{0x00}},
{0x3c,01,{0x00}},
{0x3d,01,{0x00}},
{0x3e,01,{0x00}},
{0x3f,01,{0x00}},
{0x40,01,{0x00}},
{0x41,01,{0x20}},
{0x42,01,{0x00}},
{0x43,01,{0x00}},
{0x44,01,{0x0f}},
{0x45,01,{0x00}},
{0x46,01,{0x00}},
{0x47,01,{0x08}},
{0x48,01,{0x00}},
{0x49,01,{0x00}},
{0x4a,01,{0x00}},
{0x4b,01,{0x00}},
                                       
// ==== GOUT_BW_L[3:0] ====
{0x4c,01,{0x01}},
{0x4d,01,{0x54}},
{0x4e,01,{0x64}},
{0x4f,01,{0xa8}},
{0x50,01,{0x2a}},
{0x51,01,{0x22}},
{0x52,01,{0x22}},
{0x53,01,{0x62}},
{0x54,01,{0x22}},
{0x55,01,{0x22}},
{0x56,01,{0x22}},
                                       
// ==== GOUT_BW_R[3:0] ====
{0x57,01,{0x01}},
{0x58,01,{0x54}},
{0x59,01,{0x75}},
{0x5a,01,{0xb9}},
{0x5b,01,{0x2b}},
{0x5c,01,{0x22}},
{0x5d,01,{0x22}},
{0x5e,01,{0x72}},
{0x5f,01,{0x22}},
{0x60,01,{0x22}},
{0x61,01,{0x22}},

{0x62,01,{0x06}},
                                       
// ==== GOUT_BW_L ====
{0x63,01,{0x01}},
{0x64,01,{0x00}},
{0x65,01,{0xa4}},
{0x66,01,{0xa5}},
{0x67,01,{0x54}},
{0x68,01,{0x56}},
{0x69,01,{0x58}},
{0x6a,01,{0x5a}},
{0x6b,01,{0x06}},
{0x6c,01,{0x02}},
{0x6d,01,{0x02}},
{0x6e,01,{0x02}},
{0x6f,01,{0x02}},
{0x70,01,{0x02}},
{0x71,01,{0x02}},
{0x72,01,{0x0a}},
{0x73,01,{0x02}},
{0x74,01,{0x02}},
{0x75,01,{0x02}},
{0x76,01,{0x02}},
{0x77,01,{0x02}},
{0x78,01,{0x02}},
                                       
// ==== GOUT_BW_R ====
{0x79,01,{0x01}},
{0x7a,01,{0x00}},
{0x7b,01,{0xa4}},
{0x7c,01,{0xa5}},
{0x7d,01,{0x55}},
{0x7e,01,{0x57}},
{0x7f,01,{0x59}},
{0x80,01,{0x5b}},
{0x81,01,{0x07}},
{0x82,01,{0x02}},
{0x83,01,{0x02}},
{0x84,01,{0x02}},
{0x85,01,{0x02}},
{0x86,01,{0x02}},
{0x87,01,{0x02}},
{0x88,01,{0x0b}},
{0x89,01,{0x02}},
{0x8a,01,{0x02}},
{0x8b,01,{0x02}},
{0x8c,01,{0x02}},
{0x8d,01,{0x02}},
{0x8e,01,{0x02}},
{0x8f,01,{0x00}},
{0x90,01,{0x00}},


{0xFF,03,{0x98,0x81,0x02}},
{0x42,01,{0x30}},    //20
{0x08,01,{0x22}},
{0x15,01,{0x10}},    //2-powermode  50


{0x57,01,{0x00}},  //Gamma
{0x58,01,{0x22}},
{0x59,01,{0x31}},
{0x5A,01,{0x13}},
{0x5B,01,{0x16}},
{0x5C,01,{0x2A}},
{0x5D,01,{0x1D}},
{0x5E,01,{0x1F}},
{0x5F,01,{0x90}},
{0x60,01,{0x1B}},
{0x61,01,{0x27}},
{0x62,01,{0x7C}},
{0x63,01,{0x1A}},
{0x64,01,{0x18}},
{0x65,01,{0x4D}},
{0x66,01,{0x23}},
{0x67,01,{0x28}},
{0x68,01,{0x55}},
{0x69,01,{0x64}},
{0x6A,01,{0x30}},

{0x6B,01,{0x00}},
{0x6C,01,{0x22}},
{0x6D,01,{0x31}},
{0x6E,01,{0x13}},
{0x6F,01,{0x16}},
{0x70,01,{0x2A}},
{0x71,01,{0x1D}},
{0x72,01,{0x1F}},
{0x73,01,{0x90}},
{0x74,01,{0x1B}},
{0x75,01,{0x27}},
{0x76,01,{0x7C}},
{0x77,01,{0x1A}},
{0x78,01,{0x18}},
{0x79,01,{0x4D}},
{0x7A,01,{0x23}},
{0x7B,01,{0x28}},
{0x7C,01,{0x55}},
{0x7D,01,{0x64}},
{0x7E,01,{0x30}},



{0xFF,03,{0x98,0x81,0x05}},
{0x30,01,{0x77}},       //VGH&VGLSeting
{0x54,01,{0x39}},        //VGH=15V
{0x55,01,{0x1A}},       //VGL=-10V
{0x56,01,{0x7B}},        //VREG1=5.0V
{0x57,01,{0x7B}},        //VREG2=-5.0V
{0x04,01,{0x57}},        //VCOM
{0x06,01,{0x4A}},         //VCOM

{0xFF,03,{0x98,0x81,0x06}},
{0xc0,01,{0xf3}},           //
{0xc1,01,{0x2a}},   //0a           //
 
{0xFF,03,{0x98,0x81,0x00}},
{0x36,01,{0x0A}},


{0x11,01,{0x00}},
{REGFLAG_DELAY,120,{}},

{0x29,01,{0x00}},
{REGFLAG_DELAY,20,{}},



{REGFLAG_END_OF_TABLE, 0x00, {}}    
              
};
//#endif
//#if 0
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
//#endif



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

		// enable tearing-free
	//	params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
//		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
	//5.2 18:9 59068 118136, 16:9 64754 115118; 5.7 18:9 64748 129495
	params->physical_width = 64;//LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = 134;//LCM_PHYSICAL_HEIGHT/1000;
	params->physical_width_um = 63883;//LCM_PHYSICAL_WIDTH;	= sqrt((size*25.4)^2/(18^2+9^2))*9*1000
	params->physical_height_um = 134156;//LCM_PHYSICAL_HEIGHT; = sqrt((size*25.4)^2/(18^2+9^2))*18*1000
	params->density = 320;//LCM_DENSITY;
#if (LCM_DSI_CMD_MODE)
    params->dsi.mode                     = CMD_MODE;
    params->dsi.switch_mode              = SYNC_PULSE_VDO_MODE;
#else
    params->dsi.mode                     = SYNC_PULSE_VDO_MODE;//SYNC_EVENT_VDO_MODE;//BURST_VDO_MODE;////
#endif

    // DSI
    /* Command mode setting */
    //1 Three lane or Four lane
    params->dsi.LANE_NUM                 = LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order  = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq    = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding      = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format       = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;


	params->dsi.vertical_sync_active				=  10;
	params->dsi.vertical_backporch					= 16;
	params->dsi.vertical_frontporch					= 24;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active = 40;
	params->dsi.horizontal_backporch = 50;
	params->dsi.horizontal_frontporch = 60;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	/* params->dsi.ssc_disable                                                       = 1; */

	params->dsi.PLL_CLOCK = 260;//500,228
	//params->dsi.ssc_disable =1;
//	params->dsi.CLK_HS_POST = 36;
	//params->dsi.clk_lp_per_line_enable = 0;
	//params->dsi.esd_check_enable = 1;
	//params->dsi.customization_esd_check_enable = 1;
	//params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	//params->dsi.lcm_esd_check_table[0].count = 1;
	//params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

}





//#define LCM_ID_JD9365  (0x9365)
static unsigned int lcm_compare_id(void)
{
	return 0;

}

static void lcm_init(void)
{
	display_ldo18_enable(1);
	
	MDELAY(10);
	
	SET_RESET_PIN(0);
	MDELAY(20);//100

	SET_RESET_PIN(1);
	MDELAY(250);//250
	
//	lcm_set_bias(1);

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
	
//	lcm_set_bias(0);
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




LCM_DRIVER pri_ili9881_hd1512_dsi_vdo_hsd_58_by = 
{
    .name			= "pri_ili9881_hd1512_dsi_vdo_hsd_58_by",
  	//prize-lixuefeng-20150512-start
	#if defined(CONFIG_PRIZE_HARDWARE_INFO) && !defined (BUILD_LK)
	.lcm_info = {
		.chip	= "ILI9881",
		.vendor	= "boyi",
		.id		= "0x32",
		.more	= "720*1512",
	},
	#endif
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,
    #if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
    #endif

    };

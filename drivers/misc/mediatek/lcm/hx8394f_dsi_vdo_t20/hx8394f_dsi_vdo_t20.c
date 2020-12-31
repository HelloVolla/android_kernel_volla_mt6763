#ifdef BUILD_LK
#include <debug.h>
#else
#include <linux/kernel.h>
#include <linux/string.h>
#if defined(BUILD_UBOOT)
#include <asm/arch/mt6577_gpio.h>
#else
//#include <mach/mt_gpio.h>
#endif
#endif
#include "lcm_drv.h"
//#include "tps65132.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1560)
#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER
#define HX8394F_HD720_ID  (0x8394)
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
static unsigned int lcm_compare_id(void);

static struct LCM_setting_table lcm_initialization_setting[] = {
	
{0xB9,3,{0xFF,0x83,0x94}},
{0xB1,10,{0x48,0x0F,0x6F,0x09,0x32,0x44,0x71,0x31,0x4D,0x30}},
{0xBA,6,{0x63,0x03,0x68,0x6B,0xB2,0xC0}},
{0xB2,5,{0x00,0x80,0x87,0x0C,0x07}},
{0xB4,21,{0x17,0x60,0x17,0x60,0x17,0x60,0x01,0x0C,0x70,0x55,0x00,0x3F,0x01,0x6B,0x01,0x6B,0x01,0x6B,0x01,0x0C,0x7C}},
{0xD3,34,{0x00,0x00,0x00,0x00,0x3C,0x1C,0x00,0x00,0x32,0x10,0x09,0x00,0x09,0x32,0x16,0x25,0x06,0x25,0x32,0x00,0x00,0x00,0x00,0x37,0x03,0x0B,0x0B,0x37,0x00,0x00,0x00,0x0C,0x40}},

{0xD5,45,{0x19,0x19,0x18,0x18,0x1B,0x1B,0x1A,0x1A,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x20,0x21,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x24,0x25,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},

{0xD6,45,{0x18,0x18,0x19,0x19,0x1B,0x1B,0x1A,0x1A,0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00,0x25,0x24,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x21,0x20,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},

{0xE0,59,{0x00,0x02,0x07,0x0D,0x0D,0x10,0x12,0x10,0x22,0x32,0x42,0x41,0x4B,0x60,0x66,0x6D,0x7B,0x7F,0x7C,0x8C,0x9A,0x4C,0x49,0x4B,0x4D,0x4E,0x6D,0x7F,0x7F,0x00,0x02,0x07,0x0C,0x0C,0x0F,0x11,0x10,0x22,0x32,0x42,0x41,0x4B,0x5F,0x65,0x6C,0x7A,0x7F,0x7C,0x8C,0x9A,0x4C,0x49,0x4B,0x4D,0x4E,0x6D,0x7F,0x7F}},

{0xCC,1,{0x03}},
{0xC0,2,{0x1F,0x31}},
{0xB6,2,{0x75,0x75}},
{0xD4,1,{0x02}},
{0xBD,1,{0x01}},
{0xB1,1,{0x00}},
{0xBD,1,{0x00}},
{0xC6,1,{0xED}},
{0x36,1, {0x02}},
 {0x11,0, {0x00}},
 {REGFLAG_DELAY, 120, {}},
 {0x29,0, {0x00}},
 {REGFLAG_DELAY, 20, {}},
 
	
 {REGFLAG_END_OF_TABLE, 0x00, {}}
};
#if 0
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
 {0x11, 1, {0x00}},
 {REGFLAG_DELAY, 120, {}},

 // Display ON
 {0x29, 1, {0x00}},
 {REGFLAG_DELAY, 20, {}},
 {REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

static struct LCM_setting_table lcm_sleep_in_setting[] = {
 // Display off sequence
 {0x28, 1, {0x00}},
 {REGFLAG_DELAY, 50, {}},

 // Sleep Mode On
 {0x10, 1, {0x00}},
 {REGFLAG_DELAY, 120, {}},
 //{0xB1, 1, {0x01}},
 //{REGFLAG_DELAY, 120, {}},
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
	
	params->density = 320;//LCM_DENSITY;
	
#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = SYNC_PULSE_VDO_MODE;//SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
#endif
		
	// DSI
	/* Command mode setting */
	//1 Three lane or Four lane
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;
	params->dsi.packet_size = 256;
	
	// Video mode setting		
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
			
	params->dsi.vertical_sync_active				= 2;
	params->dsi.vertical_backporch					= 16;
	params->dsi.vertical_frontporch 				= 9;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 
	
	params->dsi.horizontal_sync_active				= 30;
	params->dsi.horizontal_backporch				= 30;
	params->dsi.horizontal_frontporch				= 30;
	params->dsi.horizontal_active_pixel 			= FRAME_WIDTH;
	
	// Bit rate calculation
	//params->dsi.PLL_CLOCK = 220;
	params->dsi.PLL_CLOCK = 245;
		
    params->dsi.ssc_disable = 0;
 	params->dsi.clk_lp_per_line_enable = 0;
		
		
		
#if 1
	params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable = 1;

                              
    params->dsi.lcm_esd_check_table[0].cmd              = 0xD9;
    params->dsi.lcm_esd_check_table[0].count            = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0]     = 0x80;

                
	params->dsi.lcm_esd_check_table[1].cmd          = 0x09;
	params->dsi.lcm_esd_check_table[1].count        = 4;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x81;
	params->dsi.lcm_esd_check_table[1].para_list[1] = 0x73;
	params->dsi.lcm_esd_check_table[1].para_list[2] = 0x04;
  params->dsi.lcm_esd_check_table[1].para_list[3] = 0x00;	
	
		
	params->dsi.lcm_esd_check_table[2].cmd          = 0x45;
	params->dsi.lcm_esd_check_table[2].count        = 2;
	params->dsi.lcm_esd_check_table[2].para_list[0] = 0x06;
	params->dsi.lcm_esd_check_table[2].para_list[1] = 0x31;
	
	params->dsi.lcm_esd_check_table[3].cmd          = 0xE0;
	params->dsi.lcm_esd_check_table[3].count        = 10;
	params->dsi.lcm_esd_check_table[3].para_list[0] = 0x00;
	params->dsi.lcm_esd_check_table[3].para_list[1] = 0x02;
	params->dsi.lcm_esd_check_table[3].para_list[2] = 0x07;
	params->dsi.lcm_esd_check_table[3].para_list[3] = 0x0d;
	params->dsi.lcm_esd_check_table[3].para_list[4] = 0x0d;
	params->dsi.lcm_esd_check_table[3].para_list[5] = 0x10;
	params->dsi.lcm_esd_check_table[3].para_list[6] = 0x12;
	params->dsi.lcm_esd_check_table[3].para_list[7] = 0x10;
	params->dsi.lcm_esd_check_table[3].para_list[8] = 0x22;
	params->dsi.lcm_esd_check_table[3].para_list[9] = 0x32;		
//{0xE0,59,{0x00,0x02,0x07,0x0D,0x0D,0x10,0x12,0x10,0x22,0x32,0x42,0x41,0x4B,0x60,0x66,0x6D,0x7B,0x7F,0x7C,0x8C,0x9A,0x4C,0x49,0x4B,0x4D,0x4E,0x6D,0x7F,0x7F,0x00,0x02,0x07,0x0C,0x0C,0x0F,0x11,0x10,0x22,0x32,0x42,0x41,0x4B,0x5F,0x65,0x6C,0x7A,0x7F,0x7C,0x8C,0x9A,0x4C,0x49,0x4B,0x4D,0x4E,0x6D,0x7F,0x7F}},

#endif
   
}

static void lcm_power_sequence_on(void)
{
	//tps65132_avdd_en(TRUE);
	//display_ldo28_enable(1);
	display_bias_enable();
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(5);
    SET_RESET_PIN(0);
    MDELAY(5);
    SET_RESET_PIN(1);
    MDELAY(120);
	
}
#if 1
static void lcm_power_sequence_off(void)
{
	//tps65132_avdd_en(FALSE);
	display_bias_disable();
	MDELAY(5);
	 SET_RESET_PIN(0);
    MDELAY(5);
}
#endif

static void lcm_init(void)
{
   
    lcm_power_sequence_on();
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
		
}
static void lcm_suspend(void)
{
	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
	lcm_power_sequence_off();
}
static void lcm_resume(void)
{
//	lcm_power_sequence_on();
//	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
    lcm_init();
}


static unsigned int lcm_compare_id(void)
{	
	unsigned char buffer[3];
	unsigned int id = 0;
	unsigned int data_array[2];
	
	lcm_power_sequence_on();
	
		
	data_array[0]= 0x00043902;
	data_array[1]= (0x94<<24)|(0x83<<16)|(0xff<<8)|0xb9;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0]= 0x00023902;
	data_array[1]= (0x33<<8)|0xba;
	dsi_set_cmdq(data_array, 2, 1);
		
	data_array[0]= 0x00043902;
	data_array[1]= (0x94<<24)|(0x83<<16)|(0xff<<8)|0xb9;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x00033700;
	dsi_set_cmdq(data_array, 1, 1);
	
	read_reg_v2(0x04, buffer, 3);
		
	id = (buffer[0] << 16) | (buffer [1] << 8) | buffer [2] ; 
	
#ifdef BUILD_LK
		dprintf(0,"hx8394f id: buf:0x%02x ,0x%02x,0x%02x, id=0x%x\n", buffer[0], buffer[1], buffer[2], id);
#else
		printk("%s, kernel debug: hx8394f id = 0x%x\n", __func__, id);
#endif
	return (id == HX8394F_HD720_ID ? 1 : 0);
}

#ifndef BUILD_LK
extern atomic_t ESDCheck_byCPU;
#endif
static unsigned int lcm_ata_check(unsigned char *bufferr)
{
	unsigned char buffer0[2]={0};
	unsigned char buffer1[2]={0};
	//unsigned char buffer2[2]={0};
	//unsigned int id = 0;
	unsigned int data_array[2];
		//

	data_array[0]= 0x00023902;
	data_array[1]= 0x000050b6;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0]= 0x00023902;
	data_array[1]= 0x000008b7;
	dsi_set_cmdq(data_array, 2, 1);
	//MDELAY(120);
	
	data_array[0] = 0x00023700;
	dsi_set_cmdq(data_array, 1, 1);
	atomic_set(&ESDCheck_byCPU, 1);
	read_reg_v2(0xb6, buffer0, 1);
	read_reg_v2(0xb7, buffer1, 1);
	atomic_set(&ESDCheck_byCPU, 0);
	//printk("hx8394f id: buf:0x%02x 0x%02x \n", buffer0[0], buffer1[0]);
	return ((0x50 == buffer0[0])&&(0x08 == buffer1[0]))?1:0;
	//return 1;
	
}

struct LCM_DRIVER hx8394f_dsi_vdo_t20_lcm_drv = 
{
	.name			= "hx8394f_hd720_dsi_vdo",
  
	#if defined(CONFIG_PRIZE_HARDWARE_INFO)  && !defined (BUILD_LK)
	.lcm_info = {
		.chip	= "hx8394f",
		.vendor	= "unknow",
		.id		= "0x0094",
		.more	= "1440*720",
	},
	#endif
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.ata_check 		= lcm_ata_check,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	
};

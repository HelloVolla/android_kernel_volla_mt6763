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
/*********************************************************************************************************************/

static struct LCM_setting_table lcm_initialization_setting[] = {
{0xDF, 3,{0x93,0x65,0xF8}},
{0xB0, 7,{0x01,0x03,0x02,0x00,0x64,0x06,0x01}},
{0xB2, 2,{0x00,0x40}},
{0xB3, 2,{0x00,0x4C}},
{0xB7, 6,{0x00,0xD7,0x00,0x00,0xD7,0x00}},
{0xB9, 4,{0x00,0x04,0x13,0x07}},
{0xBB,11,{0x03,0x01,0x24,0x00,0x2C,0x0F,0x28,0x04,0xBB,0xBB,0xBB}},
{0xBE, 2,{0x1E,0xF2}},
{0xC0, 2,{0x29,0x04}},
{0xC1, 2,{0x00,0x12}},
{0xC3, 6,{0x04,0x02,0x08,0x63,0x01,0x64}},
{0xC4, 6,{0x34,0x0C,0xB4,0x81,0x12,0x0F}},
{0xC8,38,{0x7F,0x68,0x59,0x4B,0x47,0x38,0x3E,0x29,0x46,0x47,0x49,0x68,0x56,0x5D,0x4F,0x48,0x3B,0x29,0x0C,0x7F,0x68,0x59,0x4B,0x47,0x38,0x3E,0x29,0x46,0x47,0x49,0x68,0x56,0x5D,0x4F,0x48,0x3B,0x29,0x0C}},
{0xD0,22,{0x1E,0x1F,0x57,0x58,0x44,0x46,0x48,0x4A,0x40,0x1D,0x1D,0x1D,0x1D,0x1D,0x1D,0x50,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}},
{0xD1,22,{0x1E,0x1F,0x57,0x58,0x45,0x47,0x49,0x4B,0x41,0x1D,0x1D,0x1D,0x1D,0x1D,0x1D,0x51,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}},
{0xD2,22,{0x1F,0x1E,0x17,0x18,0x0B,0x09,0x07,0x05,0x11,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x01,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}},
{0xD3,22,{0x1F,0x1E,0x17,0x18,0x0A,0x08,0x06,0x04,0x10,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x00,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}},
{0xD4,22,{0x00,0x00,0x00,0x04,0x0B,0x30,0x01,0x02,0x00,0x50,0x56,0x27,0x30,0x03,0x04,0x0F,0x58,0x73,0x0D,0x0F,0x4D,0x00}},
{0xD5,17,{0x00,0x08,0x3F,0x00,0x30,0x00,0x00,0x00,0x00,0x00,0xBC,0x50,0x00,0x06,0x39,0x00,0x51}},
{0xDE, 1,{0x02}},
{0xB7, 4,{0x1B,0x00,0x00,0x04}},
{0xBB, 6,{0x21,0x25,0x23,0x24,0x34,0x35}},
{0xDE, 1,{0x00}},
{0x35, 1,{0x00}},
{0x11, 1,{0x00}},
{REGFLAG_DELAY,120,{}},
{0x29, 1,{0x00}},
{REGFLAG_DELAY, 50, {0}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#ifndef BUILD_LK
__maybe_unused static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},
    // Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
__maybe_unused static struct LCM_setting_table lcm_deep_sleep_mode_out_setting[] = {
	
	// Display off sequence
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},

    // Sleep Mode On
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

#if LCM_DSI_CMD_MODE
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = BURST_VDO_MODE;
#endif

		params->dsi.LANE_NUM = LCM_FOUR_LANE;
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
		
		
		params->dsi.packet_size=262;
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		// Video mode setting		
		params->dsi.intermediat_buffer_num = 2;
		
		params->dsi.vertical_sync_active				 = 4;
		params->dsi.vertical_backporch					  = 12;
		params->dsi.vertical_frontporch 				   = 18;
		params->dsi.vertical_active_line				= FRAME_HEIGHT;

		params->dsi.horizontal_sync_active				  = 8;
		params->dsi.horizontal_backporch				= 32;
		params->dsi.horizontal_frontporch				 = 32;
		params->dsi.horizontal_active_pixel 			   = FRAME_WIDTH;

		params->dsi.PLL_CLOCK = 232;
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
//	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	SET_RESET_PIN(1);
	MDELAY(120);
}


static void lcm_resume(void)
{
	lcm_init();
//	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}


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
printf("ATA check lk size = 0x%x,0x%x,0x%x,0x%x\n",x0_MSB,x0_LSB,x1_MSB,x1_LSB); 
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
printf("ATA read buf lk size = 0x%x,0x%x,0x%x,0x%x,ret= %d\n",read_buf[0],read_buf[1],read_buf[2],read_buf[3],ret); 
#else 
printk("ATA read buf kernel size = 0x%x,0x%x,0x%x,0x%x,ret= %d ret1= %d\n",read_buf[0],read_buf1[0],read_buf2[0],read_buf3[0],ret,ret1); 
printk("ATA read buf new kernel size = 0x%x,0x%x,0x%x,0x%x,ret= %d ret1= %d\n",read_buf1[0],read_buf1[1],read_buf1[2],read_buf1[3],ret,ret1); 

//printk("ATA read buf lk size = 0x%x,0x%x,0x%x,0x%x,ret= %d ret1= %d\n",read_buf,read_buf1,read_buf2,read_buf3,ret,ret1); 
#endif 

return ret; 
#endif 
#endif	

}

static unsigned int lcm_compare_id(void)
{
    #define LCM_ID 0x9365
    
    int array[4];
    char buffer[5];
    int id = 0;

    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);

    array[0] = 0x00023700; // read id return two byte,version and id
    dsi_set_cmdq(array, 1, 1);
    read_reg_v2(0x04, buffer, 2);

    id = (buffer[0] << 8) | buffer[1]; 

#ifdef BUILD_LK
    printf("lsw: jd9365d %s %d, id = 0x%08x\n", __func__,__LINE__, id);
#else
    printk("lsw: jd9365d %s %d, id = 0x%08x\n", __func__,__LINE__, id);
#endif

    return (id == LCM_ID) ? 1 : 0;
}

#define AUXADC_COMPARE_ID          0
#define AUXADC_LCM_VOLTAGE_CHANNEL (2)
#define MIN_VOLTAGE (0)
#define MAX_VOLTAGE (800)
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
static unsigned int ili_lcm_compare_id(void)
{
    int data[4] = {0,0,0,0};
    int res = 0;
    int rawdata = 0;
    int lcm_vol = 0;

#ifdef AUXADC_LCM_VOLTAGE_CHANNEL
    res = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL,data,&rawdata);
    if(res < 0)
    {
#ifdef BUILD_LK
        dprintf(0,"lixf lcd [adc_uboot]: get data error\n");
#endif
        return 0;
    }
#endif

    lcm_vol = data[0]*1000+data[1]*10;

#ifdef BUILD_LK
    dprintf(0,"lsw lk: lcm_vol= %d , file : %s, line : %d\n",lcm_vol, __FILE__, __LINE__);
#else
    printk("lsw kernel: lcm_vol= %d , file : %s, line : %d\n",lcm_vol, __FILE__, __LINE__);
#endif

    if (lcm_vol >= MIN_VOLTAGE && lcm_vol <= MAX_VOLTAGE && lcm_compare_id())
    {
        return 1;
    }

    return 0;
}

LCM_DRIVER jd9365z_hdplus1560_dsi_vdo_hsd_lcm_drv = {
	.name		= "jd9365z_hdplus1560_dsi_vdo_hsd",
    	//prize-lixuefeng-20150512-start
	#if defined(CONFIG_PRIZE_HARDWARE_INFO) && !defined (BUILD_LK)
	.lcm_info = {
		.chip	= "jd9365z_hsd",
		.vendor	= "hongzhan",
		.id		= "0x9365",
		.more	= "lcm_1560x720",
	},
	#endif
	//prize-lixuefeng-20150512-end	
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id 	= ili_lcm_compare_id,
	
#if (LCM_DSI_CMD_MODE)
	.set_backlight	= lcm_setbacklight,
    .update         = lcm_update,
#endif
	.ata_check	= lcm_ata_check
};


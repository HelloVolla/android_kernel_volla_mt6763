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
#define FRAME_HEIGHT 										(1280)

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
		
	//{0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},//////////page 1    For 2 POWER to 3 power IC  NT50198B control
	//{0xC0,1,{0x00}},   //For 2 POWER to 3 power IC  NT50198B control
	//{0xB5,2,{0x08,0x08}},
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},///page 0
	{0xC0,9,{0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x02}},  
	{0xC8,1,{0x80}},
	{0xc6,2,{0x41,0x18}},
	{0xB1,2,{0xE8,0x21}},
	{0xB5,2,{0x05,0x00}},
	{0xBB,2,{0x93,0x93}},
	{0xBC,2,{0x0F,0x00}},
	{0xBD,4,{0x11,0x30,0x10,0x10}},
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},/////page 1
	{0xD7,2,{0x00,0xFF}},
	{0XC0,1,{0X00}},
	{0XB5,2,{0x08,0x08}},
	{0xB7,2,{0x00,0x6C}},
	{0xCE,1,{0x00}},
	{0xCA,1,{0x03}},
	{0xB3,2,{0x23,0x23}},
	{0xB4,2,{0x23,0x23}},
	{0xC3,2,{0x5A,0x5A}},
	{0xC4,2,{0x5A,0x5A}},
	{0xC2,2,{0x5A,0x5A}},
	{0xB9,2,{0x34,0x34}},
	{0xBA,2,{0x34,0x34}},
	{0xBC,2,{0x50,0x00}},
	{0xBD,2,{0x50,0x00}},
	{0xBE,2,{0x00,0x71}},
	{0xBF,2,{0x00,0x71}},
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x03}}, ///page 3
	{0xB0,4,{0x00,0x00,0x00,0x00}},
	{0xB1,4,{0x00,0x00,0x00,0x00}},
	{0xB2,7,{0x00,0x00,0x0A,0x06,0x00,0xF0,0x5B}},
	{0xB3,7,{0x00,0x00,0x09,0x06,0x00,0xF0,0x5B}},
	{0xB6,10,{0xF0,0x05,0x06,0x03,0x00,0x00,0x00,0x00,0x10,0x10}},
	{0xB7,10,{0xF0,0x05,0x07,0x03,0x00,0x00,0x00,0x00,0x10,0x10}},
	{0xBC,7,{0xC5,0x03,0x00,0x08,0x00,0xF0,0x7f}},
	{0xC4,2,{0x00,0x00}},
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x05}},  ////page 5
	{0xB0,4,{0x33,0x04,0x00,0x01}},
	{0xB1,2,{0x40,0x00}},
	{0xB2,3,{0x03,0x02,0x02}},
	{0xB3,4,{0x83,0x23,0x42,0x97}},
	{0xB4,4,{0xC5,0x35,0x77,0x53}},
	{0xB5,7,{0x4C,0xE5,0x31,0x33,0x33,0xA3,0x0A}},
	{0xB6,6,{0x00,0x00,0xD5,0x31,0x77,0x53}},
	{0xB9,5,{0x00,0x00,0x00,0x05,0x00}},
	{0xC0,5,{0x35,0x33,0x33,0x50,0x05}},
	{0xC6,4,{0x00,0x00,0x00,0x00}},
	{0xCE,2,{0xF0,0x1F}},
	{0xD2,5,{0x00,0x25,0x02,0x00,0x00}},
	{0xE7,2,{0xE8,0xFF}},
	{0xE8,2,{0xFF,0xFF}},
	{0xE9,1,{0x00}},
	{0xEA,1,{0xAA}},
	{0xEB,1,{0xAA}},
	{0xEC,1,{0xAA}},
	{0xEE,1,{0xAA}},
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x06}},  ////page 6
	{0xB0,5,{0x7D,0x4A,0x7D,0x7D,0x7D}},
	{0xB1,5,{0x7D,0x7D,0x42,0x5d,0x7D}},
	{0xB2,5,{0x7D,0x63,0x61,0x7D,0x7D}},
	{0xB3,5,{0x5f,0x72,0x7d,0x7D,0x7D}},
	{0xB4,2,{0x7D,0x7D}},
	{0xB5,5,{0x7D,0x48,0x7D,0x7D,0x7D}},
	{0xB6,5,{0x7D,0x7D,0x40,0x5c,0x7D}},
	{0xB7,5,{0x7D,0x62,0x60,0x7D,0x7D}},
	{0xB8,5,{0x5e,0x72,0x7d,0x7D,0x7D}},
	{0xB9,2,{0x7D,0x7D}},
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x02}},////page 2
	{0xB0,1,{0x42}},
	{0xD1,16,{0x00,0x00,0x00,0x1B,0x00,0x3F,0x00,0x5B,0x00,0x71,0x00,0x97,0x00,0xB5,0x00,0xE6}},
	{0xD2,16,{0x01,0x0D,0x01,0x4A,0x01,0x7B,0x01,0xC7,0x02,0x03,0x02,0x05,0x02,0x3A,0x02,0x73}},
	{0xD3,16,{0x02,0x97,0x02,0xCB,0x02,0xEE,0x03,0x20,0x03,0x41,0x03,0x6D,0x03,0x8C,0x03,0xAB}},
	{0xD4,4,{0x03,0xBC,0x03,0xBE}},
	{0xFF,4,{0xAA,0x55,0xA5,0x80}},//command 3

	{0x6F,1,{0x0C}},//add
	{0xF7,1,{0x10}},//add


	{0xF3,1,{0xC0}},
	//{0x62,1,{0x01}},
	{0x35,1,{0x00}},
	{0x11,0,{}},
	{REGFLAG_DELAY, 200, {}},
	{0x29,0,{}},// Display on
	{REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}},     
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
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

#if LCM_DSI_CMD_MODE
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = BURST_VDO_MODE;
#endif

		params->dsi.LANE_NUM = LCM_THREE_LANE;
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
		
		
		params->dsi.packet_size=256;
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		// Video mode setting		
		params->dsi.intermediat_buffer_num = 2;
		
		params->dsi.vertical_sync_active				 = 2;
		params->dsi.vertical_backporch					  = 14;
		params->dsi.vertical_frontporch 				   = 14;
		params->dsi.vertical_active_line				= FRAME_HEIGHT;

		params->dsi.horizontal_sync_active				  = 4;
		params->dsi.horizontal_backporch				= 60;
		params->dsi.horizontal_frontporch				 = 60;
		params->dsi.horizontal_active_pixel 			   = FRAME_WIDTH;

		params->dsi.PLL_CLOCK = 265;

		params->dsi.esd_check_enable = 1;
		params->dsi.customization_esd_check_enable = 1;
		params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
		params->dsi.lcm_esd_check_table[0].count        = 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
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

struct LCM_DRIVER nt35532_hd720_dsi_vdo_xm_lcm_drv = {
	.name		= "nt35532_hd720_dsi_vdo_xm",
    //prize-lixuefeng-20150512-start
	#if defined(CONFIG_PRIZE_HARDWARE_INFO) && !defined (BUILD_LK)
		.lcm_info = {
		.chip	= "nt35532",
		.vendor	= "xiongmao",
		.id		= "0x5532",
		.more	= "lcm_1280x720",
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


/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */ 
/* MediaTek Inc. (C) 2010. All rights reserved.
 * 
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE") 
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */
 /*
  * Version: V2.4
  * Release Date: 2014/11/28
  */

#ifndef TPD_CUSTOM_GT9XX_H__
#define TPD_CUSTOM_GT9XX_H__

#include <linux/hrtimer.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
//#include <linux/io.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/time.h>
//#include <linux/rtpm_prio.h>

#include <linux/proc_fs.h>
#include <asm/uaccess.h>

//#include <mach/mt_pm_ldo.h>
//#include <mach/mt_typedefs.h>
//#include <mt-plat/mt_boot.h>
//#include <mach/gpio_const.h>
//#include <mt-plat/mt_gpio.h>

//#include <cust_eint.h>
#include <linux/jiffies.h>
//#include <pmic_drv.h>
//#include <cust_i2c.h>

/* Pre-defined definition */

#define TPD_KEY_COUNT   3
#define key_1           130,2380              //auto define  
#define key_2           360,2380
#define key_3           590,2380
#define key_4           420,954

#define TPD_KEYS        {KEY_MENU,KEY_HOMEPAGE,KEY_BACK}
#define TPD_KEYS_DIM    {{key_1,50,30},{key_2,50,30},{key_3,50,30}}
#define GTP_KEY_MAP_ARRAY {key_1,key_2,key_3}


//***************************PART1:ON/OFF define*******************************
#define GTP_CUSTOM_CFG        0
#define GTP_DRIVER_SEND_CFG   0       //driver send config to TP 
#define GTP_HAVE_TOUCH_KEY    0
#define GTP_POWER_CTRL_SLEEP  0       //power off  when suspend
#define GTP_AUTO_UPDATE       0       //auto update firmware
#define GTP_CHANGE_X2Y        0		  //swap x  y
#define GTP_HEADER_FW_UPDATE  0       // updated fw by gt9xx_firmware.h,function together with GTP_AUTO_UPDATE
#define GTP_AUTO_UPDATE_CFG   0       // auto update config by .cfg file, function together with GTP_AUTO_UPDATE

#define GTP_SUPPORT_I2C_DMA   0       
#define GTP_COMPATIBLE_MODE   1       // must be 1, compatible with GT9XXF

#define GTP_CREATE_WR_NODE    1       //supports GTP Tools
#define GTP_ESD_PROTECT       0       // esd protection

#define GTP_CONFIG_MIN_LENGTH       186
#define GTP_CONFIG_MAX_LENGTH       240
#define GTP_CHARGER_SWITCH    0       // charger plugin & plugout detect
#define GTP_WITH_PEN          0       
#define HOTKNOT_BLOCK_RW      0        // hotknot blcoking read and write

#define GTP_GESTURE_WAKEUP    0

//#define TPD_PROXIMITY

//#define TPD_WARP_X
//#define TPD_WARP_Y
#define TPD_HAVE_BUTTON             // report key as coordinate,Vibration feedback

#define GTP_DEBUG_ON          0
#define GTP_DEBUG_ARRAY_ON    0
#define GTP_DEBUG_FUNC_ON     0

#define CFG_GROUP_LEN(p_cfg_grp)  (sizeof(p_cfg_grp) / sizeof(p_cfg_grp[0]))
#define FLASHLESS_FLASH_WORKROUND  0

#if GTP_COMPATIBLE_MODE
extern void force_reset_guitar(void);
#endif
//***************************PART2:TODO define**********************************
//STEP_1(REQUIRED):Change config table.
/*TODO: puts the config info corresponded to your TP here, the following is just
a sample config, send this config should cause the chip cannot work normally*/
/*
 old:
0x42,0xD0,0x02,0x00,0x05,0x05,0x34,0x03,\
0x01,0x08,0x28,0x08,0x5A,0x40,0x03,0x03,\
0x00,0x00,0xFF,0x7F,0x00,0x00,0x85,0x17,\
0x1A,0x1D,0x14,0x89,0x29,0x0A,0x35,0x37,\
0xB8,0x08,0x03,0x00,0x00,0x98,0x33,0x1D,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x09,\
0x10,0x00,0x28,0x1E,0x57,0x94,0x45,0x42,\
0x00,0x14,0x00,0x04,0xC3,0x21,0x00,0xA0,\
0x29,0x00,0x84,0x33,0x00,0x6E,0x3F,0x00,\
0x5C,0x4E,0x00,0x5C,0x10,0x28,0x48,0x00,\
0xF0,0x4A,0x3A,0xAA,0xAF,0x27,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x0F,0x0F,0x03,0x06,0x10,\
0x42,0x86,0x0E,0x19,0x00,0x00,0x55,0x41,\
0x08,0x0A,0x0C,0x0E,0x10,0x12,0x14,0x16,\
0x18,0x1A,0x00,0x00,0x00,0x00,0x1F,0xFF,\
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,\
0x00,0x00,0x00,0x00,0xFF,0x00,0x0E,0x0C,\
0x0A,0x08,0x06,0x05,0x04,0x02,0x00,0x1D,\
0x1E,0x1F,0x20,0x22,0x24,0x28,0x29,0x2A,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,\
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,\
0x40,0x01,\
 */

/* E9C hotknot*/
/*debug charging jump ponit*/
#define CTP_CFG_GROUP0 {\
0x45,0xD0,0x02,0x00,0x05,0x05,0x34,0x33,\
0x01,0x08,0x28,0x08,0x5A,0x3C,0x03,0x03,\
0x00,0x00,0xFF,0x7F,0x00,0x00,0x85,0x17,\
0x1A,0x1D,0x14,0x89,0x29,0x0A,0x21,0x23,\
0xD3,0x07,0x03,0x00,0x00,0x9A,0x33,0x1D,\
0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x09,\
0x10,0x00,0x2A,0x1E,0x5A,0x94,0xC5,0x42,\
0x00,0x14,0x00,0x04,0x80,0x21,0x00,0x69,\
0x29,0x00,0x54,0x34,0x00,0x45,0x41,0x00,\
0x3A,0x51,0x00,0x3A,0x10,0x28,0x48,0x00,\
0xF0,0x4A,0x3A,0xAA,0xAF,0x27,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x0F,0x0F,0x03,0x06,0x10,\
0x42,0x86,0x0E,0x19,0x00,0x00,0x55,0x41,\
0x08,0x0A,0x0C,0x0E,0x10,0x12,0x14,0x16,\
0x18,0x1A,0x00,0x00,0x00,0x00,0x1F,0xFF,\
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,\
0x00,0x00,0x00,0x00,0xFF,0x00,0x0E,0x0C,\
0x0A,0x08,0x06,0x05,0x04,0x02,0x00,0x1D,\
0x1E,0x1F,0x20,0x22,0x24,0x28,0x29,0x2A,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,\
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,\
0xA7,0x01,\
}

#define GTP_CFG_GROUP0_CHARGER {\
}

//TODO puts your group2 config info here,if need.
#define CTP_CFG_GROUP1 {\
}

//TODO puts your group2 config info here,if need.
#define GTP_CFG_GROUP1_CHARGER {\
    }

//TODO puts your group3 config info here,if need.
#define CTP_CFG_GROUP2 {\
    }

//TODO puts your group3 config info here,if need.
#define GTP_CFG_GROUP2_CHARGER {\
    }

// TODO: define your config for Sensor_ID == 3 here, if needed
#define CTP_CFG_GROUP3 {\
    }
    
//TODO puts your group3 config info here,if need.
#define GTP_CFG_GROUP3_CHARGER {\
    }
    
// TODO: define your config for Sensor_ID == 4 here, if needed
#define CTP_CFG_GROUP4 {\
    }
    
//TODO puts your group4 config info here,if need.
#define GTP_CFG_GROUP4_CHARGER {\
    }
    
// TODO: define your config for Sensor_ID == 5 here, if needed
#define CTP_CFG_GROUP5 {\
    }
    
//TODO puts your group5 config info here,if need.
#define GTP_CFG_GROUP5_CHARGER {\
    }

//STEP_2(REQUIRED):Change I/O define & I/O operation mode.

//#define GTP_RST_PORT    0//GPIO_CTP_RST_PIN
//#define GTP_INT_PORT    1//GPIO_CTP_EINT_PIN

/*#define GTP_GPIO_AS_INPUT(pin)          do{\
                                            if(pin == GTP_INT_PORT)\
                                                mt_set_gpio_mode(pin, GPIO_MODE_00);\
                                            else\
                                                mt_set_gpio_mode(pin, GPIO_MODE_00);\
                                            mt_set_gpio_dir(pin, GPIO_DIR_IN);\
                                            mt_set_gpio_pull_enable(pin, GPIO_PULL_DISABLE);\
                                        }while(0)*/
/*#define GTP_GPIO_AS_INT(pin)            do{\
                                            mt_set_gpio_mode(pin, GPIO_MODE_00);\
                                            mt_set_gpio_dir(pin, GPIO_DIR_IN);\
                                            mt_set_gpio_pull_enable(pin, GPIO_PULL_DISABLE);\
                                        }while(0)*/
#define GTP_GPIO_GET_VALUE(pin)         mt_get_gpio_in(pin)
/*#define GTP_GPIO_OUTPUT(pin,level)      do{\
                                            if(pin == GTP_INT_PORT)\
                                                mt_set_gpio_mode(pin, 0);\
                                            else\
                                                mt_set_gpio_mode(pin, GPIO_MODE_00);\
                                            mt_set_gpio_dir(pin, GPIO_DIR_OUT);\
                                            mt_set_gpio_out(pin, level);\
                                        }while(0)*/
#define GTP_GPIO_REQUEST(pin, label)    gpio_request(pin, label)
#define GTP_GPIO_FREE(pin)              gpio_free(pin)
#define GTP_IRQ_TAB                     {IRQ_TYPE_EDGE_RISING, IRQ_TYPE_EDGE_FALLING, IRQ_TYPE_LEVEL_LOW, IRQ_TYPE_LEVEL_HIGH}
#define GTP_GPIO_AS_INT(pin) 			tpd_gpio_as_int(pin)
#define GTP_GPIO_OUTPUT(pin, level) 	tpd_gpio_output(pin, level)
//#define GTP_GPIO_OUTPUT(pin, level)		gpio_direction_output(pin, level)

//STEP_3(optional):Custom set some config by themself,if need.
#if GTP_CUSTOM_CFG
  #define GTP_MAX_HEIGHT   1280           
  #define GTP_MAX_WIDTH    720
#define GTP_INT_TRIGGER  0    //0:Rising 1:Falling
#else
#define GTP_MAX_HEIGHT   1280
#define GTP_MAX_WIDTH    720
#define GTP_INT_TRIGGER  1
#endif
#define GTP_MAX_TOUCH      5

#ifdef MT6589
    extern void mt65xx_eint_unmask(unsigned int line);
    extern void mt65xx_eint_mask(unsigned int line);
    #define mt_eint_mask mt65xx_eint_mask
    #define mt_eint_unmask mt65xx_eint_unmask
#endif
#ifndef CONFIG_ARCH_MT6755
#define TPD_POWER_SOURCE_CUSTOM     PMIC_APP_CAP_TOUCH_VDD      // define your power source for tp if needed
#endif
#define VELOCITY_CUSTOM
#define TPD_VELOCITY_CUSTOM_X 15
#define TPD_VELOCITY_CUSTOM_Y 15

//STEP_4(optional):If this project have touch key,Set touch key config.
#if GTP_HAVE_TOUCH_KEY
    #define GTP_KEY_TAB  {KEY_MENU, KEY_HOMEPAGE, KEY_BACK}
#endif

//***************************PART3:OTHER define*********************************
#define GTP_DRIVER_VERSION          "V2.4<2014/11/28>"
#define GTP_I2C_NAME                "Goodix-TS"
#define GT91XX_CONFIG_PROC_FILE     "gt9xx_config"
#define GTP_POLL_TIME               10
#define GTP_ADDR_LENGTH             2
#define GTP_CONFIG_MIN_LENGTH       186
#define GTP_CONFIG_MAX_LENGTH       240
#define FAIL                        0
#define SUCCESS                     1
#define SWITCH_OFF                  0
#define SWITCH_ON                   1

//******************** For GT9XXF Start **********************//
#if GTP_COMPATIBLE_MODE
typedef enum
{
    CHIP_TYPE_GT9  = 0,
    CHIP_TYPE_GT9F = 1,
} CHIP_TYPE_T;
#endif

#define GTP_REG_MATRIX_DRVNUM           0x8069
#define GTP_REG_MATRIX_SENNUM           0x806A
#define GTP_REG_RQST                    0x8043
#define GTP_REG_BAK_REF                 0x99D0
#define GTP_REG_MAIN_CLK                0x8020
#define GTP_REG_CHIP_TYPE               0x8000
#define GTP_REG_HAVE_KEY                0x804E
#define GTP_REG_HN_STATE                0xAB10

#define GTP_FL_FW_BURN                  0x00
#define GTP_FL_ESD_RECOVERY             0x01
#define GTP_FL_READ_REPAIR              0x02

#define GTP_BAK_REF_SEND                0
#define GTP_BAK_REF_STORE               1
#define CFG_LOC_DRVA_NUM                29
#define CFG_LOC_DRVB_NUM                30
#define CFG_LOC_SENS_NUM                31

#define GTP_CHK_FW_MAX                  1000
#define GTP_CHK_FS_MNT_MAX              300
#define GTP_BAK_REF_PATH                "/data/gtp_ref.bin"
#define GTP_MAIN_CLK_PATH               "/data/gtp_clk.bin"
#define GTP_RQST_CONFIG                 0x01
#define GTP_RQST_BAK_REF                0x02
#define GTP_RQST_RESET                  0x03
#define GTP_RQST_MAIN_CLOCK             0x04
#define GTP_RQST_HOTKNOT_CODE           0x20
#define GTP_RQST_RESPONDED              0x00
#define GTP_RQST_IDLE                   0xFF

#define HN_DEVICE_PAIRED                0x80
#define HN_MASTER_DEPARTED              0x40
#define HN_SLAVE_DEPARTED               0x20
#define HN_MASTER_SEND                  0x10
#define HN_SLAVE_RECEIVED               0x08


//******************** For GT9XXF End **********************//

//Register define
#define GTP_READ_COOR_ADDR          0x814E
#define GTP_REG_SLEEP               0x8040
#define GTP_REG_SENSOR_ID           0x814A
#define GTP_REG_CONFIG_DATA         0x8047
#define GTP_REG_VERSION             0x8140
#define GTP_REG_HW_INFO             0x4220
#define GTP_REG_REFRESH_RATE		0x8056

#define RESOLUTION_LOC              3
#define TRIGGER_LOC                 8


#define GTP_DMA_MAX_TRANSACTION_LENGTH  255   // for DMA mode
#define GTP_DMA_MAX_I2C_TRANSFER_SIZE   (GTP_DMA_MAX_TRANSACTION_LENGTH - GTP_ADDR_LENGTH)
#define MAX_TRANSACTION_LENGTH        8
#define TPD_I2C_NUMBER					1 //I2C_CAP_TOUCH_CHANNEL
#define I2C_BUS_NUMBER              	1 //    I2C_CAP_TOUCH_CHANNEL // 1     // I2C Bus for TP, mt6572
#define I2C_MASTER_CLOCK              300
#define MAX_I2C_TRANSFER_SIZE         (MAX_TRANSACTION_LENGTH - GTP_ADDR_LENGTH)
#define TPD_MAX_RESET_COUNT           1
#define TPD_CALIBRATION_MATRIX        {962,0,0,0,1600,0,0,0};


#define TPD_RESET_ISSUE_WORKAROUND
#define TPD_HAVE_CALIBRATION
#define TPD_NO_GPIO
#define TPD_RESET_ISSUE_WORKAROUND

#ifdef TPD_WARP_X
#undef TPD_WARP_X
#define TPD_WARP_X(x_max, x) ( x_max - 1 - x )
#else
#define TPD_WARP_X(x_max, x) x
#endif

#ifdef TPD_WARP_Y
#undef TPD_WARP_Y
#define TPD_WARP_Y(y_max, y) ( y_max - 1 - y )
#else
#define TPD_WARP_Y(y_max, y) y
#endif

//Log define
#define GTP_INFO(fmt,arg...)           printk("<<-GTP-INFO->> "fmt"\n",##arg)
#define GTP_ERROR(fmt,arg...)          printk("<<-GTP-ERROR->> "fmt"\n",##arg)
#define GTP_DEBUG(fmt,arg...)          do{\
                                         if(GTP_DEBUG_ON)\
                                         printk("<<-GTP-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                       }while(0)
#define GTP_DEBUG_ARRAY(array, num)    do{\
                                         s32 i;\
                                         u8* a = array;\
                                         if(GTP_DEBUG_ARRAY_ON)\
                                         {\
                                            printk("<<-GTP-DEBUG-ARRAY->>\n");\
                                            for (i = 0; i < (num); i++)\
                                            {\
                                                printk("%02x   ", (a)[i]);\
                                                if ((i + 1 ) %10 == 0)\
                                                {\
                                                    printk("\n");\
                                                }\
                                            }\
                                            printk("\n");\
                                        }\
                                       }while(0)
#define GTP_DEBUG_FUNC()               do{\
                                         if(GTP_DEBUG_FUNC_ON)\
                                         printk("<<-GTP-FUNC->> Func:%s@Line:%d\n",__func__,__LINE__);\
                                       }while(0)
#define GTP_SWAP(x, y)                 do{\
                                         typeof(x) z = x;\
                                         x = y;\
                                         y = z;\
                                       }while (0)

//****************************PART4:UPDATE define*******************************
//Error no
#define ERROR_NO_FILE           2   //ENOENT
#define ERROR_FILE_READ         23  //ENFILE
#define ERROR_FILE_TYPE         21  //EISDIR
#define ERROR_GPIO_REQUEST      4   //EINTR
#define ERROR_I2C_TRANSFER      5   //EIO
#define ERROR_NO_RESPONSE       16  //EBUSY
#define ERROR_TIMEOUT           110 //ETIMEDOUT

struct goodix_ts_data
{
    spinlock_t irq_lock;
    struct i2c_client *client;
    struct input_dev  *input_dev;
    struct hrtimer timer;
    struct work_struct  work;
#ifdef USE_EARLY_SUSPEND	
    struct early_suspend early_suspend;
#endif
    s32 irq_is_disable;
    s32 use_irq;
    u16 abs_x_max;
    u16 abs_y_max;
    u8  max_touch_num;
    u8  int_trigger_type;
    u8  green_wake_mode;
    u8  chip_type;
    u8  enter_update;
    u8  gtp_is_suspend;
    u8  gtp_rawdiff_mode;
};

extern u16 show_len;
extern u16 total_len;
extern u8 gtp_rawdiff_mode;
extern u8 gtp_loading_fw;

extern int tpd_halt;
extern s32 gtp_send_cfg(struct i2c_client *client);
extern void gtp_reset_guitar(struct i2c_client *client, s32 ms);
extern void gtp_int_sync(s32 ms);
extern u8 gup_init_update_proc(struct i2c_client *client);
extern u8 gup_init_fw_proc(struct i2c_client *client);
extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);
extern s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len);
extern s32 gtp_i2c_write(struct i2c_client *client, u8 *buf, s32 len);
extern int i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *txbuf, int len);
extern int i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *rxbuf, int len);
extern s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len);
extern u8 gtp_hotknot_enabled;
extern u8 wait_hotknot_state;
extern u8 got_hotknot_state;
extern u8 got_hotknot_extra_state;
extern u8 hotknot_paired_flag;
extern wait_queue_head_t bp_waiter;
extern s32 gup_load_calibration1(void);

extern unsigned char gtp_hotknot_calibration_section0[];


//*****************************End of Part III********************************

#endif /* TPD_CUSTOM_GT9XX_H__ */

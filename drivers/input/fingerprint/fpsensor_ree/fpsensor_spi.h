#ifndef __FPSENSOR_SPI_H
#define __FPSENSOR_SPI_H

#include <linux/types.h>
#include <linux/netlink.h>
#include <linux/cdev.h>
#include <linux/wait.h>
//#include <mt_spi.h>

#define FP_NOTIFY         1
#if FP_NOTIFY
#include <linux/fb.h>
#include <linux/notifier.h>
#endif

// #define  USE_PLATFORM_BUS     1
#define  USE_SPI_BUS    1
/**************************feature control******************************/
#define FPSENSOR_IOCTL    1
#define FPSENSOR_SYSFS    0
#define FPSENSOR_INPUT    0
#define FPSENSOR_MANUAL_CS 1
#define SLEEP_WAKEUP_HIGH_LEV 0
/**********************IO Magic**********************/
#define FPSENSOR_IOC_MAGIC    0xf0    //CHIP

#if defined(CONFIG_PRIZE_HARDWARE_INFO)
#include "../../hardware_info/hardware_info.h"
extern struct hardware_info current_fingerprint_info;
#endif

typedef enum fpsensor_key_event {
    FPSENSOR_KEY_NONE = 0,
    FPSENSOR_KEY_HOME,
    FPSENSOR_KEY_POWER,
    FPSENSOR_KEY_MENU,
    FPSENSOR_KEY_BACK,
    FPSENSOR_KEY_CAPTURE,
    FPSENSOR_KEY_UP,
    FPSENSOR_KEY_DOWN,
    FPSENSOR_KEY_RIGHT,
    FPSENSOR_KEY_LEFT,
    FPSENSOR_KEY_TAP,
    FPSENSOR_KEY_HEAVY
} fpsensor_key_event_t;

struct fpsensor_key {
    enum fpsensor_key_event key;
    uint32_t value;   /* key down = 1, key up = 0 */
};

/* define commands */
#define FPSENSOR_IOC_INIT                       _IOWR(FPSENSOR_IOC_MAGIC,0,unsigned int)
#define FPSENSOR_IOC_EXIT                       _IOWR(FPSENSOR_IOC_MAGIC,1,unsigned int)
#define FPSENSOR_IOC_RESET                      _IOWR(FPSENSOR_IOC_MAGIC,2,unsigned int)
#define FPSENSOR_IOC_ENABLE_IRQ                 _IOWR(FPSENSOR_IOC_MAGIC,3,unsigned int)
#define FPSENSOR_IOC_DISABLE_IRQ                _IOWR(FPSENSOR_IOC_MAGIC,4,unsigned int)
#define FPSENSOR_IOC_GET_INT_VAL                _IOWR(FPSENSOR_IOC_MAGIC,5,unsigned int)
#define FPSENSOR_IOC_DISABLE_SPI_CLK            _IOWR(FPSENSOR_IOC_MAGIC,6,unsigned int)
#define FPSENSOR_IOC_ENABLE_SPI_CLK             _IOWR(FPSENSOR_IOC_MAGIC,7,unsigned int)
#define FPSENSOR_IOC_ENABLE_POWER               _IOWR(FPSENSOR_IOC_MAGIC,8,unsigned int)
#define FPSENSOR_IOC_DISABLE_POWER              _IOWR(FPSENSOR_IOC_MAGIC,9,unsigned int)
#define FPSENSOR_IOC_INPUT_KEY_EVENT            _IOWR(FPSENSOR_IOC_MAGIC,10,struct fpsensor_key)
/* fp sensor has change to sleep mode while screen off */
#define FPSENSOR_IOC_ENTER_SLEEP_MODE           _IOWR(FPSENSOR_IOC_MAGIC,11,unsigned int)
#define FPSENSOR_IOC_REMOVE                     _IOWR(FPSENSOR_IOC_MAGIC,12,unsigned int)
#define FPSENSOR_IOC_CANCEL_WAIT                _IOWR(FPSENSOR_IOC_MAGIC,13,unsigned int)

#define FPSENSOR_IOCTL_R_SENSOR_REG                _IOWR(FPSENSOR_IOC_MAGIC,14,unsigned char [17])
#define FPSENSOR_IOCTL_W_SENSOR_REG                _IOWR(FPSENSOR_IOC_MAGIC,15,unsigned char [17])
#define FPSENSOR_IOCTL_SEND_CMD                   _IOWR(FPSENSOR_IOC_MAGIC,16,unsigned char *)
#define FPSENSOR_IOCTL_SET_SPI_CLK               _IOWR(FPSENSOR_IOC_MAGIC,17,unsigned int*)
#define FPSENSOR_IOCTL_FETCH_IMAGE               _IOWR(FPSENSOR_IOC_MAGIC,18,unsigned char [8])

#define FPSENSOR_IOC_GET_FP_STATUS              _IOWR(FPSENSOR_IOC_MAGIC,19,unsigned int*)
#define FPSENSOR_IOC_ENABLE_REPORT_BLANKON      _IOWR(FPSENSOR_IOC_MAGIC,21,unsigned int*)
#define FPSENSOR_IOC_UPDATE_DRIVER_SN           _IOWR(FPSENSOR_IOC_MAGIC,22,unsigned int*)
#define FPSENSOR_IOC_MAXNR    64  /* THIS MACRO IS NOT USED NOW... */
#define FPSENSOR_MAX_VER_BUF_LEN  64
#define FPSENSOR_MAX_CHIP_NAME_LEN 64

#define SUPPORT_REE_SPI   0
typedef struct {
    dev_t devno;
    struct class           *class;
    struct device          *device;
    struct cdev            cdev;

    spinlock_t    spi_lock;
#if defined(USE_SPI_BUS)
    struct spi_device *spi;
#elif defined(USE_PLATFORM_BUS)
    struct platform_device *spi;
#endif

    u8 device_available;    /* changed during fingerprint chip sleep and wakeup phase */
    int device_count;
    u8 irq_count;
    /* bit24-bit32 of signal count */
    volatile unsigned int RcvIRQ;
    //irq
    int irq;
    u32 irq_gpio;
    //wait queue
    wait_queue_head_t wq_irq_return;
    int cancel;
    u8 suspend_flag;
    struct pinctrl *pinctrl1;
    struct pinctrl_state *eint_as_int, *eint_in_low, *eint_in_high, *eint_in_float, *fp_rst_low, *fp_rst_high,
            *fp_spi_miso, *fp_spi_mosi, *fp_spi_clk, *fp_cs_low, *fp_cs_high;
    unsigned char *huge_buffer;
    size_t                 huge_buffer_size;
    u16                    spi_freq_khz;
    unsigned char          fetch_image_cmd;
    unsigned char          fetch_image_cmd_len;
    int                    enable_report_blankon;
    int free_flag;
#if  FP_NOTIFY
    struct notifier_block notifier;
    u8 fb_status;
#endif
} fpsensor_data_t;
#define     FPSENSOR_RST_PIN      1  // not gpio, only macro,not need modified!!
#define     FPSENSOR_SPI_CS_PIN   2  // not gpio, only macro,not need modified!!     
#define     FPSENSOR_SPI_MO_PIN   3  // not gpio, only macro,not need modified!!   
#define     FPSENSOR_SPI_MI_PIN   4  // not gpio, only macro,not need modified!!   
#define     FPSENSOR_SPI_CK_PIN   5  // not gpio, only macro,not need modified!!   


/**************************debug******************************/
#define ERR_LOG  (0)
#define INFO_LOG (1)
#define DEBUG_LOG (2)

/* debug log setting */

extern u8 fpsensor_debug_level;
extern uint32_t g_cmd_sn;
#define fpsensor_debug(level, fmt, args...) do { \
        if (fpsensor_debug_level >= level) {\
            printk("[fpsensor][SN=%d] " fmt, g_cmd_sn, ##args); \
        } \
    } while (0)

#define FUNC_ENTRY()  fpsensor_debug(DEBUG_LOG, "%s, %d, entry\n", __func__, __LINE__)
#define FUNC_EXIT()  fpsensor_debug(DEBUG_LOG, "%s, %d, exit\n", __func__, __LINE__)


#endif    /* __FPSENSOR_SPI_H */




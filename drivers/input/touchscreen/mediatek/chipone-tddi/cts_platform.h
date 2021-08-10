#ifndef CTS_PLATFORM_H
#define CTS_PLATFORM_H

#include <linux/types.h>
#include <asm/byteorder.h>
#include <linux/bitops.h>
#include <linux/ctype.h>
#include <linux/unaligned/access_ok.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/spinlock.h>
#include <linux/rtmutex.h>
#include <linux/byteorder/generic.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/string.h>
#include <linux/suspend.h>
//#include <linux/wakelock.h>
#include <linux/firmware.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#endif /* CONFIG_OF */

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include "cts_config.h"
#include "cts_core.h"

#ifdef TPD_SUPPORT_I2C_DMA
#include <linux/dma-mapping.h>
#endif /* TPD_SUPPORT_I2C_DMA */

#ifdef CONFIG_MTK_BOOT
#include "mtk_boot_common.h"
#endif /* CONFIG_MTK_BOOT */

#include "tpd.h"
#include "tpd_debug.h"
#include "upmu_common.h"

extern bool cts_show_debug_log;
extern struct chipone_ts_data *chipone_ts_data;

#ifndef LOG_TAG
#define LOG_TAG         ""
#endif /* LOG_TAG */

#define cts_err(fmt, ...)   \
    printk("<E>CTS-" LOG_TAG " " fmt"\n", ##__VA_ARGS__)
#define cts_warn(fmt, ...)  \
    printk("<W>CTS-" LOG_TAG " " fmt"\n", ##__VA_ARGS__)
#define cts_info(fmt, ...)  \
    printk("<I>CTS-" LOG_TAG " " fmt"\n", ##__VA_ARGS__)
#define cts_dbg(fmt, ...)   \
    do {                                                    \
        if (cts_show_debug_log)                                     \
            printk("<D>CTS-" LOG_TAG " "fmt"\n", ##__VA_ARGS__);   \
    } while(0)

struct cts_device;
struct cts_device_touch_msg;
struct cts_device_gesture_info;

struct cts_platform_data {
    int irq;

#ifdef CONFIG_CTS_VIRTUALKEY
    u8  vkey_num;
    u8  vkey_state;
#endif /* CONFIG_CTS_VIRTUALKEY */

    struct cts_device *cts_dev;

    struct input_dev *ts_input_dev;

#ifndef CONFIG_GENERIC_HARDIRQS
    struct work_struct ts_irq_work;
#endif /* CONFIG_GENERIC_HARDIRQS */

    struct rt_mutex dev_lock;
    struct spinlock irq_lock;
    bool            irq_is_disable;

#ifdef CFG_CTS_GESTURE
    u8  gesture_num;
    u8  gesture_keymap[CFG_CTS_NUM_GESTURE][2];
    bool irq_wake_enabled;
#endif /* CFG_CTS_GESTURE */

#ifdef TPD_SUPPORT_I2C_DMA
    u8 *i2c_dma_buff_va;
    dma_addr_t i2c_dma_buff_pa;
#endif /* TPD_SUPPORT_I2C_DMA */

#ifdef CFG_CTS_FORCE_UP
	struct timer_list touch_event_timeout_timer;
#endif

#ifdef CFG_CTS_FW_LOG_REDIRECT
    u8 fw_log_buf[CTS_FW_LOG_BUF_LEN];  
#endif      

#ifdef CONFIG_CTS_I2C_HOST
    struct i2c_client *i2c_client;
    u8 i2c_fifo_buf[CFG_CTS_MAX_I2C_XFER_SIZE];
#else
	struct spi_device *spi_client;
	u8 spi_cache_buf[ALIGN(CFG_CTS_MAX_SPI_XFER_SIZE+10,4)];
	u8 spi_rx_buf[ALIGN(CFG_CTS_MAX_SPI_XFER_SIZE+10,4)];
	u8 spi_tx_buf[ALIGN(CFG_CTS_MAX_SPI_XFER_SIZE+10,4)];
	u32 spi_speed;
#ifdef CFG_CTS_MANUAL_CS
    struct pinctrl *pinctrl1;
    struct pinctrl_state *spi_cs_low, *spi_cs_high;
#endif
#endif /* CONFIG_CTS_I2C_HOST */

    struct cts_device_touch_info touch_info;
    bool dma_available;
};

#ifdef CONFIG_CTS_I2C_HOST
extern size_t cts_plat_get_max_i2c_xfer_size(struct cts_platform_data *pdata);
extern u8 *cts_plat_get_i2c_xfer_buf(struct cts_platform_data *pdata, 
        size_t xfer_size);
extern int cts_plat_i2c_write(struct cts_platform_data *pdata, u8 i2c_addr,
        const void *src, size_t len, int retry, int delay);
extern int cts_plat_i2c_read(struct cts_platform_data *pdata, u8 i2c_addr,
        const u8 *wbuf, size_t wlen, void *rbuf, size_t rlen,
        int retry, int delay);
extern int cts_plat_is_i2c_online(struct cts_platform_data *pdata, u8 i2c_addr);
#else /* CONFIG_CTS_I2C_HOST */
extern size_t cts_plat_get_max_spi_xfer_size(struct cts_platform_data *pdata);
extern u8 *cts_plat_get_spi_xfer_buf(struct cts_platform_data *pdata, size_t xfer_size);
extern int cts_plat_spi_write(struct cts_platform_data *pdata, u8 i2c_addr, const void *src, 
		size_t len, int retry, int delay);
extern int cts_plat_spi_read(struct cts_platform_data *pdata, u8 i2c_addr,
        const u8 *wbuf, size_t wlen, void *rbuf, size_t rlen,
        int retry, int delay);
extern int cts_plat_spi_read_delay_idle(struct cts_platform_data *pdata, u8 dev_addr,
        const u8 *wbuf, size_t wlen, void *rbuf, size_t rlen,
        int retry, int delay, int idle);
extern int cts_spi_send_recv(struct cts_platform_data *pdata, size_t len , u8 *tx_buffer, u8 *rx_buffer);
#endif /* CONFIG_CTS_I2C_HOST */

#ifdef CONFIG_CTS_I2C_HOST
extern int cts_init_platform_data(struct cts_platform_data *pdata,
        struct i2c_client *i2c_client);
#else
extern int cts_plat_is_normal_mode(struct cts_platform_data *pdata);
extern int cts_init_platform_data(struct cts_platform_data *pdata,
        struct spi_device *spi);
extern int cts_plat_spi_set_mode(struct cts_platform_data *pdata);
#endif

extern int cts_plat_request_resource(struct cts_platform_data *pdata);
extern void cts_plat_free_resource(struct cts_platform_data *pdata);

extern int cts_plat_request_irq(struct cts_platform_data *pdata);
extern void cts_plat_free_irq(struct cts_platform_data *pdata);
extern int cts_plat_enable_irq(struct cts_platform_data *pdata);
extern int cts_plat_disable_irq(struct cts_platform_data *pdata);

#ifdef CFG_CTS_HAS_RESET_PIN
extern int cts_plat_reset_device(struct cts_platform_data *pdata);
#else /* CFG_CTS_HAS_RESET_PIN */
static inline int cts_plat_reset_device(struct cts_platform_data *pdata) {return 0;}
#endif /* CFG_CTS_HAS_RESET_PIN */

extern int cts_plat_init_touch_device(struct cts_platform_data *pdata);
extern void cts_plat_deinit_touch_device(struct cts_platform_data *pdata);
extern int cts_plat_process_touch_msg(struct cts_platform_data *pdata,
        struct cts_device_touch_msg *msgs, int num);
extern int cts_plat_release_all_touch(struct cts_platform_data *pdata);

#ifdef CONFIG_CTS_VIRTUALKEY
extern int cts_plat_init_vkey_device(struct cts_platform_data *pdata);
extern void cts_plat_deinit_vkey_device(struct cts_platform_data *pdata);
extern int cts_plat_process_vkey(struct cts_platform_data *pdata, u8 vkey_state);
extern int cts_plat_release_all_vkey(struct cts_platform_data *pdata);
#else /* CONFIG_CTS_VIRTUALKEY */
static inline int cts_plat_init_vkey_device(struct cts_platform_data *pdata) {return 0;}
static inline void cts_plat_deinit_vkey_device(struct cts_platform_data *pdata) {}
static inline int cts_plat_process_vkey(struct cts_platform_data *pdata, u8 vkey_state) {return 0;}
static inline int cts_plat_release_all_vkey(struct cts_platform_data *pdata) {return 0;}
#endif /* CONFIG_CTS_VIRTUALKEY */

#ifdef CFG_CTS_GESTURE
extern int cts_plat_enable_irq_wake(struct cts_platform_data *pdata);
extern int cts_plat_disable_irq_wake(struct cts_platform_data *pdata);

extern int cts_plat_init_gesture(struct cts_platform_data *pdata);
extern void cts_plat_deinit_gesture(struct cts_platform_data *pdata);
extern int cts_plat_process_gesture_info(struct cts_platform_data *pdata,
        struct cts_device_gesture_info *gesture_info);
#else /* CFG_CTS_GESTURE */
static inline int cts_plat_init_gesture(struct cts_platform_data *pdata) {return 0;}
static inline void cts_plat_deinit_gesture(struct cts_platform_data *pdata)  {}
#endif /* CFG_CTS_GESTURE */

#endif /* CTS_PLATFORM_H */

#ifdef CFG_CTS_FW_LOG_REDIRECT
extern size_t cts_plat_get_max_fw_log_size(struct cts_platform_data *pdata);
extern u8 *cts_plat_get_fw_log_buf(struct cts_platform_data *pdata, size_t size);
#endif

extern int cts_plat_set_reset(struct cts_platform_data *pdata, int val);

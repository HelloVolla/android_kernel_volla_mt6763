/* Copyright (C) MicroArray
 * MicroArray Fprint Driver Code
 * mtk-settings.h
 * Date: 2017-3-15
 * Version: v4.0.06
 * Author: guq
 * Contact: guq@microarray.com.cn
 */

#ifndef __MTK_SETTINGS_H_
#define __MTK_SETTINGS_H_



#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include "madev.h"
#include <linux/spi/spi.h>
//#include <linux/wakelock.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
////lude "mt_spi.h"
//#include <linux/wakelock.h>
////lude <mt-plat/mt_gpio.h>
//macro settings
#include <linux/clk.h>

#define MA_DRV_NAME             "madev"

#define MA_DTS_NAME            "mediatek,mt6765-fpc"

#define MA_EINT_DTS_NAME        "mediatek,mt6765-fpc"

#define MA_INT_PIN_LABEL		"fpc_eint_as_int"
//macro settings end



//call madev function
extern int mas_plat_probe(struct platform_device *pdev);
extern int mas_plat_remove(struct platform_device *pdev);

extern int mas_probe(struct spi_device *spi);
extern int mas_remove(struct spi_device *spi);

/* add for spi cls ctl start */
struct mt_spi_t {
        struct platform_device *pdev;
        void __iomem *regs;
        int irq;
        int running;
        //struct wake_lock wk_lock;
        struct mt_chip_conf *config;
        struct spi_master *master;

        struct spi_transfer *cur_transfer;
        struct spi_transfer *next_transfer;

        spinlock_t lock;
        struct list_head queue;
#if !defined(CONFIG_MTK_CLKMGR)
        struct clk *clk_main;
#endif
};
void mt_spi_enable_clk(struct mt_spi_t *ms);
void mt_spi_disable_clk(struct mt_spi_t *ms);
void mt_spi_enable_master_clk(struct spi_device *spidev);
void mt_spi_disable_master_clk(struct spi_device *spidev);

/* add for spi cls ctl end this func only used in tee enviroment*/
//packaging
//void mas_enable_spi_clock(struct spi_device *spi);
//void mas_diasble_spi_clock(struct spi_device *spi);
//packaging end

//the interface called by madev
void mas_select_transfer(struct spi_device *spi, int len);
int mas_finger_get_gpio_info(struct platform_device *pdev);
int mas_finger_set_gpio_info(int cmd);
void mas_enable_spi_clock(struct spi_device *spi);
void mas_disable_spi_clock(struct spi_device *spi);

unsigned int mas_get_irq(struct device *dev);
int mas_get_platform(void);
int mas_remove_platform(void);
int mas_power(int cmd);
int get_screen(void);
void ma_spi_change(struct spi_device *spi, unsigned int speed, int flag);
int mas_get_interrupt_gpio(unsigned int index);
int mas_switch_power(unsigned int on_off);
int mas_do_some_for_probe(struct spi_device *spi);
#endif

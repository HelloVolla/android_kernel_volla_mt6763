#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/fb.h>
#include <linux/ioctl.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/completion.h>
#include <linux/gpio.h>

#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/pm_qos.h>
#include <linux/cpufreq.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#else
#include <linux/notifier.h>
#endif

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#endif

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#ifdef CONFIG_MTK_CLKMGR
#include "mach/mt_clkmgr.h"
#else
#include <linux/clk.h>
#endif

#include <net/sock.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/wakelock.h>

/* MTK header */
//#include "mt_spi.h"
//#include "mt_spi_hal.h"
//#include "mt_gpio.h"
//#include "mach/gpio_const.h"

#include "fpsensor_spi.h"
#include "fpsensor_platform.h"

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>


/*device name*/
#define FPSENSOR_DEV_NAME       "fpsensor"
#define FPSENSOR_CLASS_NAME     "fpsensor"
#define FPSENSOR_MAJOR          0
#define N_SPI_MINORS            32    /* ... up to 256 */

#define FPSENSOR_SPI_VERSION    "fpsensor_spi_ree_mtk_v1.22.1"

/***********************GPIO setting port layer*************************/
/* customer hardware port layer, please change according to customer's hardware */
#define GPIO_PIN_IRQ   86

/*************************************************************/
static struct wake_lock fpsensor_timeout_wakelock;
/* debug log setting */
u8 fpsensor_debug_level = DEBUG_LOG;

fpsensor_data_t *g_fpsensor = NULL;
uint32_t g_cmd_sn = 0;


#define ROUND_UP(x, align)        ((x+(align-1))&~(align-1))


#define FPSENSOR_SPI_BUS_DYNAMIC 1
#if FPSENSOR_SPI_BUS_DYNAMIC
static struct spi_board_info spi_board_devs[] __initdata = {
    [0] = {
        .modalias = FPSENSOR_DEV_NAME,
        .bus_num = 0,
        .chip_select = 0,
        .mode = SPI_MODE_0,
        //.controller_data = &fpsensor_spi_conf_mt65xx, //&spi_conf
    },
};
#endif



/* -------------------------------------------------------------------- */
/* fingerprint chip hardware configuration                              */
/* -------------------------------------------------------------------- */
static DEFINE_MUTEX(spidev_set_gpio_mutex);
static void spidev_gpio_as_int(fpsensor_data_t *fpsensor)
{
    FUNC_ENTRY();
    mutex_lock(&spidev_set_gpio_mutex);
    pinctrl_select_state(fpsensor->pinctrl1, fpsensor->eint_as_int);
    mutex_unlock(&spidev_set_gpio_mutex);
    FUNC_EXIT();
}
void setRcvIRQ(int val)
{
    fpsensor_data_t *fpsensor_dev = g_fpsensor;
    // fpsensor_debug(INFO_LOG, "[rickon]: %s befor val :  %d ; set val : %d   \n", __func__, fpsensor_dev-> RcvIRQ, val);
    fpsensor_dev-> RcvIRQ = val;
}
void fpsensor_gpio_output_dts(int gpio, int level)
{
    FUNC_ENTRY();
    mutex_lock(&spidev_set_gpio_mutex);
    if (gpio == FPSENSOR_RST_PIN) {
        if (level) {
            pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_rst_high);
        } else {
            pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_rst_low);
        }
    } else if (gpio == FPSENSOR_SPI_CS_PIN) {
        if (level) {
            pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_cs_high);
        } else {
            pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_cs_low);
        }
    }

    mutex_unlock(&spidev_set_gpio_mutex);
    FUNC_EXIT();
}


int fpsensor_gpio_wirte(int gpio, int value)
{
    fpsensor_gpio_output_dts(gpio, value);
    return 0;
}
int fpsensor_gpio_read(int gpio)
{
    return gpio_get_value(gpio);
}

int fpsensor_spidev_dts_init(fpsensor_data_t *fpsensor)
{
    struct device_node *node = NULL;
    struct platform_device *pdev = NULL;
    int ret = 0;
    FUNC_ENTRY();
    node = of_find_compatible_node(NULL, NULL, "mediatek,mt6765-fpc");
    if (node) {
        pdev = of_find_device_by_node(node);
        if(pdev) {
            fpsensor->pinctrl1 = devm_pinctrl_get(&pdev->dev);
            if (IS_ERR(fpsensor->pinctrl1)) {
                ret = PTR_ERR(fpsensor->pinctrl1);
                fpsensor_debug(ERR_LOG,"fpsensor Cannot find fp pinctrl1.\n");
                return ret;
            }
        } else {
            fpsensor_debug(ERR_LOG,"fpsensor Cannot find device.\n");
            return -ENODEV;
        }
        fpsensor->eint_as_int = pinctrl_lookup_state(fpsensor->pinctrl1, "finger_eint");
        if (IS_ERR(fpsensor->eint_as_int)) {
            ret = PTR_ERR(fpsensor->eint_as_int);
            fpsensor_debug(ERR_LOG, "fpsensor Cannot find fp pinctrl eint_as_int!\n");
            return ret;
        }
        fpsensor->fp_rst_low = pinctrl_lookup_state(fpsensor->pinctrl1, "finger_rst_low");
        if (IS_ERR(fpsensor->fp_rst_low)) {
            ret = PTR_ERR(fpsensor->fp_rst_low);
            fpsensor_debug(ERR_LOG, "fpsensor Cannot find fp pinctrl fpsensor_finger_rst_low!\n");
            return ret;
        }
        fpsensor->fp_rst_high = pinctrl_lookup_state(fpsensor->pinctrl1, "finger_rst_high");
        if (IS_ERR(fpsensor->fp_rst_high)) {
            ret = PTR_ERR(fpsensor->fp_rst_high);
            fpsensor_debug(ERR_LOG, "fpsensor Cannot find fp pinctrl fpsensor_finger_rst_high!\n");
            return ret;
        }

//------------------------------ree
        fpsensor->fp_spi_miso = pinctrl_lookup_state(fpsensor->pinctrl1, "spi_mi_low");
        if (IS_ERR(fpsensor->fp_spi_miso)) {
            ret = PTR_ERR(fpsensor->fp_spi_miso);
            fpsensor_debug(ERR_LOG, " Cannot find fp pinctrl fpsensor_spi_mi_low!\n");
            return ret;
        }
        fpsensor->fp_spi_mosi = pinctrl_lookup_state(fpsensor->pinctrl1, "spi_mo_low");
        if (IS_ERR(fpsensor->fp_spi_mosi)) {
            ret = PTR_ERR(fpsensor->fp_spi_mosi);
            fpsensor_debug(ERR_LOG, " Cannot find fp pinctrl fpsensor_spi_mo_low!\n");
            return ret;
        }
        fpsensor->fp_cs_high = pinctrl_lookup_state(fpsensor->pinctrl1, "spi_cs_high");
        if (IS_ERR(fpsensor->fp_cs_high)) {
            ret = PTR_ERR(fpsensor->fp_cs_high);
            fpsensor_debug(ERR_LOG, " Cannot find fp pinctrl fpsensor_finger_cs_high!\n");
            return ret;
        }
        fpsensor->fp_cs_low = pinctrl_lookup_state(fpsensor->pinctrl1, "spi_cs_low");
        if (IS_ERR(fpsensor->fp_cs_low)) {
            ret = PTR_ERR(fpsensor->fp_cs_low);
            fpsensor_debug(ERR_LOG, " Cannot find fp pinctrl fpsensor_finger_cs_low!\n");
            return ret;
        }
        fpsensor->fp_spi_clk = pinctrl_lookup_state(fpsensor->pinctrl1, "spi_mclk_low");
        if (IS_ERR(fpsensor->fp_spi_clk)) {
            ret = PTR_ERR(fpsensor->fp_spi_clk);
            fpsensor_debug(ERR_LOG, " Cannot find fp pinctrl fpsensor_spi_mclk_low!\n");
            return ret;
        }
        pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_spi_miso);
        pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_spi_mosi);
        pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_spi_clk);
    } else {
        fpsensor_debug(ERR_LOG, "compatible_node Cannot find node!\n");
        return -ENODEV;
    }
    FUNC_EXIT();
    return 0;
}
/* delay us after reset */
static void fpsensor_hw_reset(int delay)
{
    FUNC_ENTRY();

    fpsensor_gpio_wirte(FPSENSOR_RST_PIN,    1);
    udelay(100);
    fpsensor_gpio_wirte(FPSENSOR_RST_PIN,  0);
    udelay(1000);
    fpsensor_gpio_wirte(FPSENSOR_RST_PIN,  1);
    if (delay) {
        /* delay is configurable */
        udelay(delay);
    }

    FUNC_EXIT();
    return;
}

static void fpsensor_spi_clk_enable(u8 bonoff)
{

}


static int fpsensor_irq_gpio_cfg(void)
{
    struct device_node *node;
    fpsensor_data_t *fpsensor;
    u32 ints[2] = {0, 0};
    FUNC_ENTRY();

    fpsensor = g_fpsensor;

    spidev_gpio_as_int(fpsensor);

    node = of_find_compatible_node(NULL, NULL, "mediatek,mt6765-fpc");
    if ( node) {
        of_property_read_u32_array( node, "debounce", ints, ARRAY_SIZE(ints));
        // gpio_request(ints[0], "fpsensor-irq");
        //gpio_set_debounce(ints[0], ints[1]);
        fpsensor_debug(INFO_LOG, "[fpsensor]ints[0] = %d,is irq_gpio , ints[1] = %d!!\n", ints[0], ints[1]);
        fpsensor->irq_gpio = ints[0];
        fpsensor->irq = irq_of_parse_and_map(node, 0);  // get irq number
        if (!fpsensor->irq) {
            fpsensor_debug(ERR_LOG, "fpsensor irq_of_parse_and_map fail!!\n");
            return -EINVAL;
        }
        fpsensor_debug(INFO_LOG, " [fpsensor]fpsensor->irq= %d,fpsensor>irq_gpio = %d\n", fpsensor->irq,
                       fpsensor->irq_gpio);
    } else {
        fpsensor_debug(ERR_LOG, "fpsensor null irq node!!\n");
        return -EINVAL;
    }
    FUNC_EXIT();
    return 0;

}
static void fpsensor_enable_irq(fpsensor_data_t *fpsensor_dev)
{
    FUNC_ENTRY();
    setRcvIRQ(0);
    if (0 == fpsensor_dev->device_available) {
        fpsensor_debug(ERR_LOG, "%s, devices not available\n", __func__);
    } else {
        if (1 == fpsensor_dev->irq_count) {
            fpsensor_debug(ERR_LOG, "%s, irq already enabled\n", __func__);
        } else {
            enable_irq(fpsensor_dev->irq);
            fpsensor_dev->irq_count = 1;
            fpsensor_debug(INFO_LOG, "%s enable interrupt!\n", __func__);
        }
    }
    FUNC_EXIT();
    return;
}

static void fpsensor_disable_irq(fpsensor_data_t *fpsensor_dev)
{
    FUNC_ENTRY();

    if (0 == fpsensor_dev->device_available) {
        fpsensor_debug(ERR_LOG, "%s, devices not available\n", __func__);
    } else {
        if (0 == fpsensor_dev->irq_count) {
            fpsensor_debug(ERR_LOG, "%s, irq already disabled\n", __func__);
        } else {
            disable_irq(fpsensor_dev->irq);
            fpsensor_dev->irq_count = 0;
            fpsensor_debug(DEBUG_LOG, "%s disable interrupt!\n", __func__);
        }
    }
    setRcvIRQ(0);
    FUNC_EXIT();
    return;
}

/* -------------------------------------------------------------------- */
/* file operation function                                              */
/* -------------------------------------------------------------------- */

static ssize_t fpsensor_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    int error;
    FUNC_ENTRY();
    error =  fpsensor_spi_dma_fetch_image(g_fpsensor, count);
    if (error) {
        return -EFAULT;
    }
    error = copy_to_user(buf, &g_fpsensor->huge_buffer[0], count);
    if (error) {
        return -EFAULT;
    }
    FUNC_EXIT();
    return count;

}

static ssize_t fpsensor_write(struct file *filp, const char __user *buf,
                              size_t count, loff_t *f_pos)
{
    fpsensor_debug(ERR_LOG, "Not support write opertion in TEE version\n");
    return -EFAULT;
}

static irqreturn_t fpsensor_irq(int irq, void *handle)
{
    fpsensor_data_t *fpsensor_dev = (fpsensor_data_t *)handle;
#if SLEEP_WAKEUP_HIGH_LEV
    int irqf = 0;
    irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT | IRQF_NO_SUSPEND;
    if (fpsensor_dev->suspend_flag == 1) {
        irq_set_irq_type(fpsensor_dev->irq, irqf);
        fpsensor_dev->suspend_flag = 0;
    }
#endif
    wake_lock_timeout(&fpsensor_timeout_wakelock, msecs_to_jiffies(1000));
#if FPSENSOR_IOCTL
    setRcvIRQ(1);
#endif
    wake_up_interruptible(&fpsensor_dev->wq_irq_return);
    return IRQ_HANDLED;
}


static long fpsensor_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    fpsensor_data_t *fpsensor_dev = NULL;
    u8 reg_data[17];
    u8 reg_rx[16];
    int retval = 0;
    unsigned int val = 0;

    FUNC_ENTRY();
    fpsensor_debug(INFO_LOG, "[rickon]: fpsensor ioctl cmd : 0x%x \n", cmd );
    fpsensor_dev = (fpsensor_data_t *)filp->private_data;
    //clear cancel flag
    fpsensor_dev->cancel = 0 ;
    switch (cmd) {
    case FPSENSOR_IOC_INIT:
        fpsensor_debug(INFO_LOG, "%s: fpsensor init started======\n", __func__);
        wake_lock_init(&fpsensor_timeout_wakelock, WAKE_LOCK_SUSPEND, "fpsensor timeout wakelock");
        if(fpsensor_irq_gpio_cfg() != 0) {
            break;
        }
        retval = request_threaded_irq(fpsensor_dev->irq, fpsensor_irq, NULL,
                                      IRQF_TRIGGER_RISING | IRQF_ONESHOT, FPSENSOR_DEV_NAME, fpsensor_dev);
        if (retval == 0) {
            fpsensor_debug(ERR_LOG, " irq thread reqquest success!\n");
        } else {
            fpsensor_debug(ERR_LOG, " irq thread request failed , retval =%d \n", retval);
            break;
        }
        fpsensor_dev->device_available = 1;
        fpsensor_dev->irq_count = 1;
        fpsensor_disable_irq(fpsensor_dev);
        fpsensor_debug(INFO_LOG, "%s: fpsensor init finished======\n", __func__);
        break;

    case FPSENSOR_IOC_EXIT:
        fpsensor_disable_irq(fpsensor_dev);
        fpsensor_dev->irq_count = 0;
        if (fpsensor_dev->irq) {
            free_irq(fpsensor_dev->irq, fpsensor_dev);
            fpsensor_dev->irq = 0;
        }
        fpsensor_dev->device_available = 0;
        fpsensor_debug(INFO_LOG, "%s: fpsensor exit finished======\n", __func__);
        break;

    case FPSENSOR_IOC_RESET:
        fpsensor_debug(INFO_LOG, "%s: chip reset command\n", __func__);
        fpsensor_hw_reset(1250);
        break;

    case FPSENSOR_IOC_ENABLE_IRQ:
        fpsensor_debug(INFO_LOG, "%s: chip ENable IRQ command\n", __func__);
        fpsensor_enable_irq(fpsensor_dev);
        break;

    case FPSENSOR_IOC_DISABLE_IRQ:
        fpsensor_debug(INFO_LOG, "%s: chip disable IRQ command\n", __func__);
        fpsensor_disable_irq(fpsensor_dev);
        break;
    case FPSENSOR_IOC_GET_INT_VAL:
        val = __gpio_get_value(GPIO_PIN_IRQ);
        if (copy_to_user((void __user *)arg, (void *)&val, sizeof(unsigned int))) {
            fpsensor_debug(ERR_LOG, "Failed to copy data to user\n");
            retval = -EFAULT;
            break;
        }
        retval = 0;
        break;
    case FPSENSOR_IOC_ENABLE_SPI_CLK:
        fpsensor_debug(INFO_LOG, "%s: ENABLE_SPI_CLK ======\n", __func__);
        fpsensor_spi_clk_enable(1);
        break;
    case FPSENSOR_IOC_DISABLE_SPI_CLK:
        fpsensor_debug(INFO_LOG, "%s: DISABLE_SPI_CLK ======\n", __func__);
        fpsensor_spi_clk_enable(0);
        break;

    case FPSENSOR_IOC_ENABLE_POWER:
        fpsensor_debug(INFO_LOG, "%s: FPSENSOR_IOC_ENABLE_POWER ======\n", __func__);
        // fpsensor_hw_power_enable(1);
        break;

    case FPSENSOR_IOC_DISABLE_POWER:
        fpsensor_debug(INFO_LOG, "%s: FPSENSOR_IOC_DISABLE_POWER ======\n", __func__);
        // fpsensor_hw_power_enable(0);
        break;

    case FPSENSOR_IOC_REMOVE:
        fpsensor_spi_clk_enable(0);
        unregister_chrdev_region(fpsensor_dev->devno, 1);
        if (fpsensor_dev->device != NULL) {
            device_destroy(fpsensor_dev->class, fpsensor_dev->devno);
        }
        if (fpsensor_dev->class != NULL ) {
            class_destroy(fpsensor_dev->class);
        }
#if FP_NOTIFY
        fb_unregister_client(&fpsensor_dev->notifier);
#endif
        fpsensor_dev->free_flag = 1;
        fpsensor_debug(INFO_LOG, "%s remove finished\n", __func__);
        break;

    case FPSENSOR_IOC_CANCEL_WAIT:
        fpsensor_debug(INFO_LOG, "%s: FPSENSOR CANCEL WAIT\n", __func__);
        fpsensor_dev->cancel = 1;
        wake_up_interruptible(&fpsensor_dev->wq_irq_return);
        break;
    case FPSENSOR_IOCTL_W_SENSOR_REG:
        if (copy_from_user(reg_data, (void __user *)arg, 17)) {
            // error = -EFAULT;
            break;
        }
        fpsensor_spi_send_recv(fpsensor_dev, reg_data[0], reg_data + 1, reg_rx);
        break;

    case FPSENSOR_IOCTL_R_SENSOR_REG:
        if (copy_from_user(reg_data, (void __user *)arg, 17)) {
            break;
        }
        fpsensor_spi_send_recv(fpsensor_dev, reg_data[0], reg_data + 1, reg_rx);
        fpsensor_debug(INFO_LOG, "%s: reg_rx : %x   %x   %x  %x\n", __func__, reg_rx[0], reg_rx[1],
                       reg_rx[2], reg_rx[3]);
        if (copy_to_user((void __user *)arg, reg_rx, 16) != 0) {
            break;
        }
        break;
    case FPSENSOR_IOCTL_SEND_CMD:
        if (copy_from_user(reg_data, (void __user *)arg, 1)) {
            break;
        }
        fpsensor_spi_send_recv(fpsensor_dev, 1 , reg_data, NULL);
        break;
    case FPSENSOR_IOCTL_SET_SPI_CLK:
        if (copy_from_user(&val, (void __user *)arg, 4)) {
            retval = -EFAULT;
            break;
        }
        fpsensor_dev->spi_freq_khz = val;
        break;
    case FPSENSOR_IOCTL_FETCH_IMAGE:
        if (copy_from_user(reg_data, (void __user *)arg, 8)) {
            retval = -EFAULT;
            break;
        }
        fpsensor_dev->fetch_image_cmd_len = reg_data[1];
        fpsensor_dev->fetch_image_cmd = reg_data[0];
        break;
#if FP_NOTIFY
    case FPSENSOR_IOC_GET_FP_STATUS :
        val = fpsensor_dev->fb_status;
        fpsensor_debug(INFO_LOG, "%s: FPSENSOR_IOC_GET_FP_STATUS  %d \n",__func__, fpsensor_dev->fb_status);
        if (copy_to_user((void __user *)arg, (void *)&val, sizeof(unsigned int))) {
            fpsensor_debug(ERR_LOG, "Failed to copy data to user\n");
            retval = -EFAULT;
            break;
        }
        retval = 0;
        break;
#endif
    case FPSENSOR_IOC_ENABLE_REPORT_BLANKON:
        if (copy_from_user(&val, (void __user *)arg, 4)) {
            retval = -EFAULT;
            break;
        }
        fpsensor_dev->enable_report_blankon = val;
        break;
    case FPSENSOR_IOC_UPDATE_DRIVER_SN:
        if (copy_from_user(&g_cmd_sn, (unsigned int *)arg, sizeof(unsigned int))) {
            fpsensor_debug(ERR_LOG, "Failed to copy g_cmd_sn from user to kernel\n");
            retval = -EFAULT;
            break;
        }
        break;
    default:
        fpsensor_debug(ERR_LOG, "fpsensor doesn't support this command(%d)\n", cmd);
        break;
    }

    FUNC_EXIT();
    return retval;
}
static long fpsensor_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return fpsensor_ioctl(filp, cmd, (unsigned long)(arg));
}

static unsigned int fpsensor_poll(struct file *filp, struct poll_table_struct *wait)
{
    unsigned int ret = 0;
    fpsensor_debug(ERR_LOG, " support poll opertion  in   version\n");
    ret |= POLLIN;
    poll_wait(filp, &g_fpsensor->wq_irq_return, wait);
    if (g_fpsensor->cancel == 1 ) {
        fpsensor_debug(ERR_LOG, " cancle\n");
        ret =  POLLERR;
        g_fpsensor->cancel = 0;
        return ret;
    }
    if ( g_fpsensor->RcvIRQ) {
        if (g_fpsensor->RcvIRQ == 2) {
            fpsensor_debug(ERR_LOG, " get fp on notify\n");
            ret |= POLLHUP;
        } else {
            fpsensor_debug(ERR_LOG, " get irq\n");
            ret |= POLLRDNORM;
        }
    } else {
        ret = 0;
    }
    return ret;
}


/* -------------------------------------------------------------------- */
/* device function                                                      */
/* -------------------------------------------------------------------- */
static int fpsensor_open(struct inode *inode, struct file *filp)
{
    fpsensor_data_t *fpsensor_dev;

    FUNC_ENTRY();
    fpsensor_dev = container_of(inode->i_cdev, fpsensor_data_t, cdev);
    fpsensor_dev->device_available = 1;
    filp->private_data = fpsensor_dev;
    FUNC_EXIT();
    return 0;
}

static int fpsensor_release(struct inode *inode, struct file *filp)
{
    fpsensor_data_t *fpsensor_dev;

    FUNC_ENTRY();
    fpsensor_dev = filp->private_data;
    filp->private_data = NULL;

    /*last close??*/
    fpsensor_debug(INFO_LOG, "%s, disble_irq. irq = %d\n", __func__, fpsensor_dev->irq);
    fpsensor_disable_irq(fpsensor_dev);
    fpsensor_dev->device_available = 0;
    if (fpsensor_dev->free_flag == 1) {
        kfree(fpsensor_dev);
    }
    FUNC_EXIT();
    return 0;
}



static const struct file_operations fpsensor_fops = {
    .owner =    THIS_MODULE,

    .write =    fpsensor_write,
    .read =        fpsensor_read,
    .unlocked_ioctl = fpsensor_ioctl,
    .compat_ioctl   = fpsensor_compat_ioctl,
    .open =        fpsensor_open,
    .release =    fpsensor_release,
    .poll    = fpsensor_poll,
};

static int fpsensor_create_class(fpsensor_data_t *fpsensor)
{
    int error = 0;

    fpsensor->class = class_create(THIS_MODULE, FPSENSOR_CLASS_NAME);
    if (IS_ERR(fpsensor->class)) {
        fpsensor_debug(ERR_LOG, "%s, Failed to create class.\n", __func__);
        error = PTR_ERR(fpsensor->class);
    }

    return error;
}

static int fpsensor_create_device(fpsensor_data_t *fpsensor)
{
    int error = 0;


    if (FPSENSOR_MAJOR > 0) {
        //fpsensor->devno = MKDEV(FPSENSOR_MAJOR, fpsensor_device_count++);
        //error = register_chrdev_region(fpsensor->devno,
        //                 1,
        //                 FPSENSOR_DEV_NAME);
    } else {
        error = alloc_chrdev_region(&fpsensor->devno,
                                    fpsensor->device_count++,
                                    1,
                                    FPSENSOR_DEV_NAME);
    }

    if (error < 0) {
        fpsensor_debug(ERR_LOG,
                       "%s: FAILED %d.\n", __func__, error);
        goto out;

    } else {
        fpsensor_debug(INFO_LOG, "%s: major=%d, minor=%d\n",
                       __func__,
                       MAJOR(fpsensor->devno),
                       MINOR(fpsensor->devno));
    }

    fpsensor->device = device_create(fpsensor->class, &(fpsensor->spi->dev), fpsensor->devno,
                                     fpsensor, FPSENSOR_DEV_NAME);

    if (IS_ERR(fpsensor->device)) {
        fpsensor_debug(ERR_LOG, "device_create failed.\n");
        error = PTR_ERR(fpsensor->device);
    }
out:
    return error;
}

extern int get_fp_vendor(void);
enum {
    FP_VENDOR_INVALID = 0,
    FPC_VENDOR,
    ELAN_VENDOR,
    GOODIX_VENDOR,
    CHIPONE_VENDOR
};

#if FP_NOTIFY
static int fpsensor_fb_notifier_callback(struct notifier_block* self,
        unsigned long event, void* data)
{
    static char screen_status[64] = {'\0'};
    struct fb_event* evdata = data;
    unsigned int blank;
    fpsensor_data_t *fpsensor_dev = g_fpsensor;
    fpsensor_debug(INFO_LOG,"%s enter.  event : 0x%lx\n", __func__,event);

    if (event != FB_EVENT_BLANK /* FB_EARLY_EVENT_BLANK */) {
        return 0;
    }

    blank = *(int*)evdata->data;
    fpsensor_debug(INFO_LOG,"%s enter, blank=0x%x\n", __func__, blank);

    switch (blank) {
    case FB_BLANK_UNBLANK:
        fpsensor_debug(INFO_LOG,"%s: lcd on notify\n", __func__);
        sprintf(screen_status, "SCREEN_STATUS=%s", "ON");
        fpsensor_dev->fb_status = 1;
        if( fpsensor_dev->enable_report_blankon) {
            fpsensor_dev->RcvIRQ = 2;
            wake_up_interruptible(&fpsensor_dev->wq_irq_return);
        }
        break;

    case FB_BLANK_POWERDOWN:
        fpsensor_debug(INFO_LOG,"%s: lcd off notify\n", __func__);
        sprintf(screen_status, "SCREEN_STATUS=%s", "OFF");
        fpsensor_dev->fb_status = 0;
        break;

    default:
        fpsensor_debug(INFO_LOG,"%s: other notifier, ignore\n", __func__);
        break;
    }

    fpsensor_debug(INFO_LOG,"%s %s leave.\n", screen_status, __func__);
    return 0;
}
#endif

static int fpsensor_probe(struct spi_device *spi)
{
    struct device *dev = &spi->dev;
    fpsensor_data_t *fpsensor_dev = NULL;
    int error = 0;
    int status = -EINVAL;

    FUNC_ENTRY();
    /* Allocate driver data */
    fpsensor_dev = kzalloc(sizeof(*fpsensor_dev), GFP_KERNEL);
    if (!fpsensor_dev) {
        fpsensor_debug(ERR_LOG, "%s, Failed to alloc memory for fpsensor device.\n", __func__);
        FUNC_EXIT();
        return -ENOMEM;
    }
    fpsensor_dev->device = dev ;

    g_fpsensor = fpsensor_dev;
    /* Initialize the driver data */
    spi_set_drvdata(spi, fpsensor_dev);
    fpsensor_dev->spi = spi;
    fpsensor_dev->device_available = 0;
    fpsensor_dev->irq = 0;
    fpsensor_dev->device_count     = 0;
    fpsensor_dev->spi_freq_khz = 6000u;
    fpsensor_dev->fetch_image_cmd = 0x2c;
    fpsensor_dev->fetch_image_cmd_len = 2;
    fpsensor_dev->free_flag         = 0;
    /*setup fpsensor configurations.*/
    fpsensor_debug(INFO_LOG, "%s, Setting fpsensor device configuration.\n", __func__);

    // dts read
    if(fpsensor_spidev_dts_init(fpsensor_dev) != 0) {
        goto err2;
    }
    fpsensor_spi_setup(fpsensor_dev);
    fpsensor_manage_image_buffer(fpsensor_dev, 160 * 160 * 2);
    fpsensor_hw_reset(1250);
    if (fpsensor_check_HWID(fpsensor_dev) == 0) {
        fpsensor_debug(ERR_LOG, "get chip id error .\n");
        goto err2;
    }
    error = fpsensor_create_class(fpsensor_dev);
    if (error) {
        goto err2;
    }
    error = fpsensor_create_device(fpsensor_dev);
    if (error) {
        goto err1;
    }
    cdev_init(&fpsensor_dev->cdev, &fpsensor_fops);
    fpsensor_dev->cdev.owner = THIS_MODULE;
    error = cdev_add(&fpsensor_dev->cdev, fpsensor_dev->devno, 1);
    if (error) {
        goto err1;
    }
    fpsensor_dev->device_available = 1;
    fpsensor_dev->irq_count = 0;
    // mt_eint_unmask(fpsensor_dev->irq);
    fpsensor_debug(INFO_LOG, "%s: fpsensor init finished======\n", __func__);
    fpsensor_spi_clk_enable(1);
    //init wait queue
    init_waitqueue_head(&fpsensor_dev->wq_irq_return);

    fpsensor_debug(INFO_LOG, "%s probe finished, normal driver version: %s\n", __func__,
                   FPSENSOR_SPI_VERSION);
#if FP_NOTIFY
    fpsensor_dev->notifier.notifier_call = fpsensor_fb_notifier_callback;
    fb_register_client(&fpsensor_dev->notifier);
#endif

#if defined(CONFIG_PRIZE_HARDWARE_INFO)
	sprintf(current_fingerprint_info.chip,"icnfpsensor");
	strcpy(current_fingerprint_info.vendor,"chiponeic");
	strcpy(current_fingerprint_info.more,"fingerprint");
#endif
    FUNC_EXIT();
    return 0;
err1:
    if (fpsensor_dev->device != NULL) {
        device_destroy(fpsensor_dev->class, fpsensor_dev->devno);
    }
    if (fpsensor_dev->class != NULL ) {

        class_destroy(fpsensor_dev->class);
    }
err2:
    kfree(fpsensor_dev);
    FUNC_EXIT();
    return status;
}

static int fpsensor_remove(struct spi_device *spi)
{

    fpsensor_data_t *fpsensor_dev = spi_get_drvdata(spi);

    FUNC_ENTRY();

    /* make sure ops on existing fds can abort cleanly */
    if (fpsensor_dev->irq) {
        free_irq(fpsensor_dev->irq, fpsensor_dev);
        fpsensor_dev->irq = 0;
    }

    fpsensor_dev->spi = NULL;
    spi_set_drvdata(spi, NULL);
    unregister_chrdev_region(fpsensor_dev->devno, 1);
    if (fpsensor_dev->device != NULL) {
        device_destroy(fpsensor_dev->class, fpsensor_dev->devno);
    }
    if (fpsensor_dev->class != NULL ) {

        class_destroy(fpsensor_dev->class);
    }
#if FP_NOTIFY
    fb_unregister_client(&fpsensor_dev->notifier);
#endif
    wake_lock_destroy(&fpsensor_timeout_wakelock);
    fpsensor_debug(INFO_LOG, "%s remove finished\n", __func__);
    kfree(fpsensor_dev);
#if 0
    if (s_DEVINFO_fpsensor != NULL) {
        kfree(s_DEVINFO_fpsensor);
    }
#endif
    FUNC_EXIT();
    return 0;
}
#ifdef CONFIG_PM
static int fpsensor_suspend(struct device *pdev)
{
#if SLEEP_WAKEUP_HIGH_LEV
    int irqf = 0;
#endif

    fpsensor_debug(INFO_LOG, "%s\n", __func__);

#if SLEEP_WAKEUP_HIGH_LEV
    irqf = IRQF_TRIGGER_HIGH | IRQF_ONESHOT | IRQF_NO_SUSPEND;
    fpsensor_disable_irq(g_fpsensor);
    irq_set_irq_type(g_fpsensor->irq, irqf);
    g_fpsensor->suspend_flag = 1;
    fpsensor_enable_irq(g_fpsensor);
#endif
    enable_irq_wake(g_fpsensor->irq);
    fpsensor_debug(INFO_LOG, "%s exit\n", __func__);
    //fpsensor_debug(INFO_LOG, "%s, exit irq is %d\n", __func__, gpio_get_value(g_fpsensor->irq_gpio));
    return 0;
}


static int fpsensor_resume(struct device *pdev)
{
    fpsensor_debug(INFO_LOG, "%s\n", __func__);
    disable_irq_wake(g_fpsensor->irq);
    return 0;
}
#endif

#ifdef CONFIG_OF
static struct of_device_id fpsensor_of_match[] = {
    { .compatible = "mediatek,icnfpsensor", },
    {}
};
MODULE_DEVICE_TABLE(of, fpsensor_of_match);
#endif
struct spi_device_id fpsensor_spi_id_table = {FPSENSOR_DEV_NAME, 0};

static const struct dev_pm_ops fpsensor_pm = {
    .suspend = fpsensor_suspend,
    .resume = fpsensor_resume
};

static struct spi_driver fpsensor_spi_driver = {
    .driver = {
        .name = FPSENSOR_DEV_NAME,
        .bus = &spi_bus_type,
        .owner = THIS_MODULE,
        // .mode = SPI_MODE_0,
        // .controller_data = &fpsensor_spi_conf_mt65xx, //&spi_conf
#ifdef CONFIG_PM
        .pm = &fpsensor_pm,
#endif
#ifdef CONFIG_OF
        .of_match_table = fpsensor_of_match,
#endif
    },
    .id_table = &fpsensor_spi_id_table,
    .probe = fpsensor_probe,
    .remove = fpsensor_remove,
};


static int __init fpsensor_init(void)
{
    int status;

    FUNC_ENTRY();


#if FPSENSOR_SPI_BUS_DYNAMIC
    spi_register_board_info(spi_board_devs, ARRAY_SIZE(spi_board_devs));
#endif
    status = spi_register_driver(&fpsensor_spi_driver);

    if (status < 0) {
        fpsensor_debug(ERR_LOG, "%s, Failed to register SPI driver.\n", __func__);
    }

    FUNC_EXIT();
    return status;
}
module_init(fpsensor_init);

static void __exit fpsensor_exit(void)
{
    FUNC_ENTRY();

    spi_unregister_driver(&fpsensor_spi_driver);

    FUNC_EXIT();
}
module_exit(fpsensor_exit);

MODULE_AUTHOR("xhli");
MODULE_DESCRIPTION(" Fingerprint chip driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:fpsensor_spi");

/**
 * plat-mt6795.c
 *
**/

#include <linux/delay.h>

#include <mach/mt_gpio.h>
#include <mach/eint.h>
#include <mach/mt_clkmgr.h>
#include <mach/mt_pm_ldo.h>

#include "ff_log.h"
#include "ff_ctl.h"

# undef LOG_TAG
#define LOG_TAG "mt6795"

/*
 * Driver configuration. See ff_ctl.c
 */
extern ff_driver_config_t *g_config;

int ff_ctl_init_pins(int *irq_num)
{
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);

    if (unlikely(!g_config)) {
        return (-ENOSYS);
    }

    /* Initialize SPI pins. */
    if (g_config && g_config->enable_spidev) {
        g_config->gpio_miso_pin |= 0x80000000;
        g_config->gpio_mosi_pin |= 0x80000000;
        g_config->gpio_ck_pin   |= 0x80000000;
        g_config->gpio_cs_pin   |= 0x80000000;

        err = mt_set_gpio_mode(g_config->gpio_miso_pin, GPIO_MODE_01);
        err+= mt_set_gpio_mode(g_config->gpio_mosi_pin, GPIO_MODE_01);
        err+= mt_set_gpio_mode(g_config->gpio_ck_pin  , GPIO_MODE_01);
        err+= mt_set_gpio_mode(g_config->gpio_cs_pin  , GPIO_MODE_01);

        err+= mt_set_gpio_dir (g_config->gpio_miso_pin, GPIO_DIR_IN );
        err+= mt_set_gpio_dir (g_config->gpio_mosi_pin, GPIO_DIR_OUT);
        err+= mt_set_gpio_dir (g_config->gpio_ck_pin  , GPIO_DIR_OUT);
        err+= mt_set_gpio_dir (g_config->gpio_cs_pin  , GPIO_DIR_OUT);

        if (err) {
            FF_LOGE("mt_set_gpio_mode/dir(..) error for SPI pins.");
            return (-EIO);
        }
    }

    /* Initialize RST pin. */
    err = mt_set_gpio_mode(g_config->gpio_rst_pin, GPIO_MODE_GPIO);
    err+= mt_set_gpio_dir (g_config->gpio_rst_pin, GPIO_DIR_OUT);
    err+= mt_set_gpio_out (g_config->gpio_rst_pin, GPIO_OUT_ONE);
    if (err) {
        FF_LOGE("mt_set_gpio_mode/dir(..) error for RST pin.");
        return (-EIO);
    }

    /* Initialize INT pin. */
    err = mt_set_gpio_mode(g_config->gpio_int_pin, GPIO_MODE_GPIO);
    err+= mt_set_gpio_dir (g_config->gpio_int_pin, GPIO_DIR_IN);
    err+= mt_set_gpio_out (g_config->gpio_int_pin, GPIO_OUT_ZERO);
    if (err) {
        FF_LOGE("mt_set_gpio_mode/dir(..) error for INT pin.");
        return (-EIO);
    }

    /* Retrieve the IRQ number. */
    *irq_num = mt_gpio_to_irq(g_config->gpio_int_pin);
    if (*irq_num < 0) {
        FF_LOGE("mt_gpio_to_irq(%d) failed.", g_config->gpio_int_pin);
        return (-EIO);
    } else {
        FF_LOGD("mt_gpio_to_irq(%d) = %d.", g_config->gpio_int_pin, *irq_num);
    }

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

int ff_ctl_free_pins(void)
{
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);

    // TODO:

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

int ff_ctl_enable_spiclk(bool on)
{
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);
    FF_LOGD("clock: '%s'.", on ? "enable" : "disabled");

    if (on) {
        enable_clock(MT_CG_PERI_SPI0, "spi");
    } else {
        disable_clock(MT_CG_PERI_SPI0, "spi");
    }

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

int ff_ctl_enable_power(bool on)
{
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);
    FF_LOGD("power: '%s'.", on ? "on" : "off");

    if (on) {
        hwPowerOn(MT6331_POWER_LDO_VMCH, VOL_3300, "fingerprint");
    } else {
        hwPowerDown(MT6331_POWER_LDO_VMCH, "fingerprint");
    }

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

int ff_ctl_reset_device(void)
{
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);

    if (unlikely(!g_config)) {
        return (-ENOSYS);
    }

    /* 3-1: Pull down RST pin. */
    err = mt_set_gpio_out(g_config->gpio_rst_pin, 0);

    /* 3-2: Delay for 10ms. */
    mdelay(10);

    /* Pull up RST pin. */
    err = mt_set_gpio_out(g_config->gpio_rst_pin, 1);

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

const char *ff_ctl_arch_str(void)
{
    return CONFIG_MTK_PLATFORM;
}

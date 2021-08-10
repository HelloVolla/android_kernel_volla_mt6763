/**
 * plat-mt6762.c
 *
**/

#include <linux/stddef.h>
#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

#if !defined(CONFIG_MTK_CLKMGR)
# include <linux/clk.h>
#else
# include <mach/mt_clkmgr.h>
#endif

#include "ff_log.h"

# undef LOG_TAG
#define LOG_TAG "mt6762"

int ff_ctl_enable_power(bool on);

/* TODO: */
#define FF_COMPATIBLE_NODE_1 "mediatek,mt6765-fpc"
#define FF_COMPATIBLE_NODE_2 "mediatek,mt6765-fpc"
//#define FF_COMPATIBLE_NODE_1 "mediatek,focal-fp"
//#define FF_COMPATIBLE_NODE_2 "mediatek,fpc1145"
#define FF_COMPATIBLE_NODE_3 "mediatek,mt6765-spi"

/* Define pinctrl state types. */
#if 0
typedef enum {
    FF_PINCTRL_STATE_SPI_CS_ACT,
    FF_PINCTRL_STATE_SPI_CK_ACT,
    FF_PINCTRL_STATE_SPI_MOSI_ACT,
    FF_PINCTRL_STATE_SPI_MISO_ACT,
    FF_PINCTRL_STATE_PWR_ACT,
    FF_PINCTRL_STATE_PWR_CLR,
    FF_PINCTRL_STATE_RST_ACT,
    FF_PINCTRL_STATE_RST_CLR,
    FF_PINCTRL_STATE_INT_ACT,
    FF_PINCTRL_STATE_MAXIMUM /* Array size */
} ff_pinctrl_state_t;

typedef enum {
    FF_PINCTRL_STATE_PWR_ACT,
    FF_PINCTRL_STATE_PWR_CLR,
    FF_PINCTRL_STATE_RST_CLR,
    FF_PINCTRL_STATE_RST_ACT,
    FF_PINCTRL_STATE_INT_ACT,
    FF_PINCTRL_STATE_CS_SET,
    FF_PINCTRL_STATE_CLK_SET,
    FF_PINCTRL_STATE_MI_SET,
    FF_PINCTRL_STATE_MO_SET,
    FF_PINCTRL_STATE_MI_ACT,
    FF_PINCTRL_STATE_MI_CLR,
    FF_PINCTRL_STATE_MO_ACT,
    FF_PINCTRL_STATE_MO_CLR,
    FF_PINCTRL_STATE_MAXIMUM /* Array size */
} ff_pinctrl_state_t;

#else
typedef enum {
    FF_PINCTRL_STATE_PWR_ACT,
    FF_PINCTRL_STATE_PWR_CLR,
    FF_PINCTRL_STATE_RST_CLR,
    FF_PINCTRL_STATE_RST_ACT,
    FF_PINCTRL_STATE_INT_ACT,
    FF_PINCTRL_STATE_MAXIMUM /* Array size */
} ff_pinctrl_state_t;
#endif
/* Define pinctrl state names. */
#if 0
static const char *g_pinctrl_state_names[FF_PINCTRL_STATE_MAXIMUM] = {
    "csb_spi", "clk_spi", "mosi_spi", "miso_spi",
    "power_on", "power_off", "reset_low", "reset_high", "irq_gpio",
};

static const char *g_pinctrl_state_names[FF_PINCTRL_STATE_MAXIMUM] = {
    "fpc_pins_pwr_high", "fpc_pins_pwr_low", "fpc_pins_rst_low", "fpc_pins_rst_high",
    "fpc_eint_as_int", "fpc_mode_as_cs", "fpc_mode_as_ck", "fpc_mode_as_mi",
    "fpc_mode_as_mo", "fpc_miso_pull_up", "fpc_miso_pull_down",
    "fpc_mosi_pull_up", "fpc_mosi_pull_down",
};

#else
static const char *g_pinctrl_state_names[FF_PINCTRL_STATE_MAXIMUM] = {
    "fpc_pins_pwr_high", "fpc_pins_pwr_low", "fpc_pins_rst_low", "fpc_pins_rst_high",
    "fpc_eint_as_int", 
};
#endif

/* Native context and its singleton instance. */
typedef struct {
    struct pinctrl *pinctrl;
    struct pinctrl_state *pin_states[FF_PINCTRL_STATE_MAXIMUM];
#if !defined(CONFIG_MTK_CLKMGR)
    struct clk *spiclk;
#endif
    bool b_spiclk_enabled;
} ff_mt6762_context_t;
static ff_mt6762_context_t ff_mt6762_context, *g_context = &ff_mt6762_context;

int ff_ctl_init_pins(int *irq_num)
{
    int err = 0, i;
	int irq_num1 = 0;
    struct device_node *dev_node = NULL;
    struct platform_device *pdev = NULL;

    printk("'%s' enter.", __func__);

    /* Find device tree node. */
    dev_node = of_find_compatible_node(NULL, NULL, FF_COMPATIBLE_NODE_1);
    if (!dev_node) {
        FF_LOGE("of_find_compatible_node(.., '%s') failed.", FF_COMPATIBLE_NODE_1);
        return (-ENODEV);
    }

	irq_num1 = irq_of_parse_and_map(dev_node, 0);
	*irq_num = irq_num1;
    printk("pzp irq number is %d.", irq_num1);
    /* Convert to platform device. */
    pdev = of_find_device_by_node(dev_node);
    if (!pdev) {
        FF_LOGE("of_find_device_by_node(..) failed.");
        return (-ENODEV);
    }

    /* Retrieve the pinctrl handler. */
    g_context->pinctrl = devm_pinctrl_get(&pdev->dev);
    if (!g_context->pinctrl) {
        FF_LOGE("devm_pinctrl_get(..) failed.");
        return (-ENODEV);
    }

    /* Register all pins. */
    for (i = 0; i < FF_PINCTRL_STATE_MAXIMUM; ++i) {
        g_context->pin_states[i] = pinctrl_lookup_state(g_context->pinctrl, g_pinctrl_state_names[i]);
        if (!g_context->pin_states[i]) {
            FF_LOGE("can't find pinctrl state for '%s'.", g_pinctrl_state_names[i]);
            err = (-ENODEV);
            break;
        }
    }
    if (i < FF_PINCTRL_STATE_MAXIMUM) {
        return (-ENODEV);
    }

    /* init spi,sunch as cs clck miso mosi mode, gpio pullup pulldown */
	/*
        for (i = FF_PINCTRL_STATE_INT_ACT + 1; i < FF_PINCTRL_STATE_MAXIMUM; ++i) {
            err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[i]);

            if (err) {
                printk("%s() pinctrl_select_state(%s) failed.\n", __FUNCTION__, g_pinctrl_state_names[i]);
                break;
            }

            printk("pinctrl_select_state(%s) ok.\n", g_pinctrl_state_names[i]);
        }
        */
    /* Initialize the INT pin. */
    err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_INT_ACT]);

    /* Retrieve the irq number. 
    dev_node = of_find_compatible_node(NULL, NULL, FF_COMPATIBLE_NODE_2);
    if (!dev_node) {
        printk("of_find_compatible_node(.., '%s') failed.", FF_COMPATIBLE_NODE_2);
        return (-ENODEV);
    }
    *irq_num = irq_of_parse_and_map(dev_node, 0);
    printk("irq number is %d.", *irq_num);*/

    pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_RST_ACT]);
	
#if 1
//#if !defined(CONFIG_MTK_CLKMGR)
    //
    // Retrieve the clock source of the SPI controller.
    //

    /* 3-1: Find device tree node. */
    dev_node = of_find_compatible_node(NULL, NULL, FF_COMPATIBLE_NODE_3);
    if (!dev_node) {
        FF_LOGE("of_find_compatible_node(.., '%s') failed.", FF_COMPATIBLE_NODE_3);
        return (-ENODEV);
    }

    /* 3-2: Convert to platform device. */
    pdev = of_find_device_by_node(dev_node);
    if (!pdev) {
        FF_LOGE("of_find_device_by_node(..) failed.");
        return (-ENODEV);
    } else {
        //u32 frequency, div;
        //err = of_property_read_u32(pdev->dev.of_node, "clock-frequency", &frequency);
        //err = of_property_read_u32(pdev->dev.of_node, "clock-div", &div);
        FF_LOGE("spi controller(#%d) name: %s.", pdev->id, pdev->name);
        //FF_LOGD("spi controller(#%d) clk : %dHz.", pdev->id, frequency / div);
    }

    /* 3-3: Retrieve the SPI clk handler. */
    g_context->spiclk = devm_clk_get(&pdev->dev, "spi-clk");
    if (!g_context->spiclk) {
        FF_LOGE("devm_clk_get(..) failed.");
        return (-ENODEV);
    }
#endif

    ff_ctl_enable_power(true);

    printk("'%s' leave.", __func__);
    return err;
}

int ff_ctl_free_pins(void)
{
    int err = 0;
    printk("'%s' enter.", __func__);

    // TODO:
	if (g_context->pinctrl) {
        pinctrl_put(g_context->pinctrl);
        g_context->pinctrl = NULL;
    }
    printk("'%s' leave.", __func__);
    return err;
}

int ff_ctl_enable_spiclk(bool on)
{
    int err = 0;
    printk("'%s' enter.", __func__);
    FF_LOGD("clock: '%s'.", on ? "enable" : "disabled");

    if (unlikely(!g_context->spiclk)) {
        return (-ENOSYS);
    }
	
	printk("focal '%s' b_spiclk_enabled = %d. \n", __func__, g_context->b_spiclk_enabled);

#if !defined(CONFIG_MTK_CLKMGR)
    /* Prepare the clock source. */
    //err = clk_prepare(g_context->spiclk);
#endif

    /* Control the clock source. */
    if (on && !g_context->b_spiclk_enabled) {
//#if !defined(CONFIG_MTK_CLKMGR)
	//err = clk_prepare(g_context->spiclk);
        err = clk_prepare_enable(g_context->spiclk);;
        if (err) {
            FF_LOGE("clk_prepare_enable(..) = %d.", err);
        }
//#else
        //enable_clock(MT_CG_PERI_SPI0, "spi");
//#endif
        g_context->b_spiclk_enabled = true;
    } else if (!on && g_context->b_spiclk_enabled) {
//#if !defined(CONFIG_MTK_CLKMGR)
        //clk_disable(g_context->spiclk);
//#else
        //disable_clock(MT_CG_PERI_SPI0, "spi");
//#endif
		clk_disable_unprepare(g_context->spiclk);
        g_context->b_spiclk_enabled = false;
    }

    printk("'%s' leave.", __func__);
    return err;
}

int ff_ctl_enable_power(bool on)
{
    int err = 0;
    printk("'%s' enter.", __func__);
    FF_LOGD("power: '%s'.", on ? "on" : "off");

    if (unlikely(!g_context->pinctrl)) {
        return (-ENOSYS);
    }

    if (on) {
        err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_PWR_ACT]);
    } else {
        err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_PWR_CLR]);
    }

    printk("'%s' leave.", __func__);
    return err;
}


int ff_ctl_reset_device(void)
{
    int err = 0;
    printk("'%s' enter.", __func__);

    if (unlikely(!g_context->pinctrl)) {
        return (-ENOSYS);
    }

	err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_RST_ACT]);
	mdelay(1);
    /* 3-1: Pull down RST pin. */
	err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_RST_CLR]);

    /* 3-2: Delay for 10ms. */
    mdelay(10);

    /* Pull up RST pin. */
    err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_RST_ACT]);

    printk("'%s' leave.", __func__);
    return err;
}

const char *ff_ctl_arch_str(void)
{
    //return ("CONFIG_MTK_PLATFORM");
	return (CONFIG_MTK_PLATFORM);
}


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
extern struct spi_device *g_spidev;


extern void mt_spi_enable_master_clk(struct spi_device *spidev);
extern void mt_spi_disable_master_clk(struct spi_device *spidev);

/* TODO: */
#define FF_COMPATIBLE_NODE_1 "mediatek,mt6765-fpc"
#define FF_COMPATIBLE_NODE_2 "mediatek,mt6765-fpc"
//#define FF_COMPATIBLE_NODE_1 "mediatek,focal-fp"
//#define FF_COMPATIBLE_NODE_2 "mediatek,fpc1145"
//#define FF_COMPATIBLE_NODE_3 "mediatek,mt6762-spi"

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
	FF_PINCTRL_STATE_INT_ACT,
	FF_PINCTRL_STATE_RST_ACT,
    FF_PINCTRL_STATE_RST_CLR,
    FF_PINCTRL_STATE_PWR_ACT,
    FF_PINCTRL_STATE_PWR_CLR,
    FF_PINCTRL_STATE_SPI_CS_ACT,
    FF_PINCTRL_STATE_SPI_CK_ACT,
    FF_PINCTRL_STATE_SPI_MOSI_ACT,
    FF_PINCTRL_STATE_SPI_MISO_ACT,
    FF_PINCTRL_STATE_MAXIMUM /* Array size */
} ff_pinctrl_state_t;
#endif
/* Define pinctrl state names. */
#if 1

static const char *g_pinctrl_state_names[FF_PINCTRL_STATE_MAXIMUM] = {
    "fpc_eint_as_int", "fpc_pins_rst_high","fpc_pins_rst_low", "fpc_pins_pwr_high", "fpc_pins_pwr_low",  
    "fpc_mode_as_cs",  "fpc_mode_as_ck",   "fpc_mode_as_mi",   "fpc_mode_as_mo", 
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
        printk("of_find_compatible_node(.., '%s') failed.", FF_COMPATIBLE_NODE_1);
        return (-ENODEV);
    }

	irq_num1 = irq_of_parse_and_map(dev_node, 0);
	*irq_num = irq_num1;
    printk("pzp irq number is %d.", irq_num1);
    /* Convert to platform device. */
    pdev = of_find_device_by_node(dev_node);
    if (!pdev) {
        printk("of_find_device_by_node(..) failed.");
        return (-ENODEV);
    }

    /* Retrieve the pinctrl handler. */
    g_context->pinctrl = devm_pinctrl_get(&pdev->dev);
    if (!g_context->pinctrl) {
        printk("devm_pinctrl_get(..) failed.");
        return (-ENODEV);
    }

    /* Register all pins. */
    for (i = 0; i < FF_PINCTRL_STATE_MAXIMUM; ++i) {
        g_context->pin_states[i] = pinctrl_lookup_state(g_context->pinctrl, g_pinctrl_state_names[i]);
        if (!g_context->pin_states[i]) {
            printk("can't find pinctrl state for '%s'.", g_pinctrl_state_names[i]);
            err = (-ENODEV);
            break;
        }
    }
    if (i < FF_PINCTRL_STATE_MAXIMUM) {
        return (-ENODEV);
    }
  
	 /* Initialize the SPI pins. */
    err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_SPI_CS_ACT]);
    err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_SPI_CK_ACT]);
    err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_SPI_MOSI_ACT]);
    err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_SPI_MISO_ACT]);

    /* Initialize the INT pin. */
    err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_INT_ACT]);
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
	//static int count;
    printk("'%s' enter.", __func__);
    FF_LOGD("clock: '%s'.", on ? "enable" : "disabled");
	
	printk("focal '%s' b_spiclk_enabled = %d. \n", __func__, g_context->b_spiclk_enabled);
#if 0
    /* Control the clock source. */
    if (on && !g_context->b_spiclk_enabled) {
        err = clk_prepare_enable(g_context->spiclk);;
        if (err) {
            FF_LOGE("clk_prepare_enable(..) = %d.", err);
        }
        g_context->b_spiclk_enabled = true;
    } else if (!on && g_context->b_spiclk_enabled) {
		clk_disable_unprepare(g_context->spiclk);
        g_context->b_spiclk_enabled = false;
    }

#endif

    if (on && !g_context->b_spiclk_enabled) {
		printk("'%s' pzp enter.mt_spi_enable_master_clk", __func__);
        mt_spi_enable_master_clk(g_spidev);
        g_context->b_spiclk_enabled = true;
    }
    else if (!on && g_context->b_spiclk_enabled) {
        mt_spi_disable_master_clk(g_spidev);
		printk("'%s' pzp enter.mt_spi_disable_master_clk", __func__);
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
    return ("CONFIG_MTK_PLATFORM");
}


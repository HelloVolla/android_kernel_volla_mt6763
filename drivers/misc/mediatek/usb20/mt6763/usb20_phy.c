/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifdef CONFIG_MTK_CLKMGR
#include <mach/mt_clkmgr.h>
#else
#include <linux/clk.h>
#endif
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <mtk_musb.h>
#include <musb_core.h>
#include "usb20.h"
#include "mtk_devinfo.h"

#ifdef CONFIG_OF
#include <linux/of_address.h>
#endif

#include <mt-plat/mtk_boot_common.h>

#define FRA (48)
#define PARA (28)

#ifdef FPGA_PLATFORM
bool usb_enable_clock(bool enable) { return true; }

void usb_phy_poweron(void) {}

void usb_phy_savecurrent(void) {}

void usb_phy_recover(void) {}

/* BC1.2 */
void Charger_Detect_Init(void) {}

void Charger_Detect_Release(void) {}

void usb_phy_context_save(void) {}

void usb_phy_context_restore(void) {}

#ifdef CONFIG_MTK_UART_USB_SWITCH
bool usb_phy_check_in_uart_mode(void) { return false; }

void usb_phy_switch_to_uart(void) {}

void usb_phy_switch_to_usb(void) {}
#endif

#else
#include <linux/of_address.h>
#include <linux/of_irq.h>
#define VAL_MAX_WIDTH_2 0x3
#define VAL_MAX_WIDTH_3 0x7
#define OFFSET_RG_USB20_VRT_VREF_SEL 0x4
#define SHFT_RG_USB20_VRT_VREF_SEL 12
#define OFFSET_RG_USB20_TERM_VREF_SEL 0x4
#define SHFT_RG_USB20_TERM_VREF_SEL 8
#define OFFSET_RG_USB20_PHY_REV6 0x18
#define SHFT_RG_USB20_PHY_REV6 30
void usb_phy_tuning(void)
{
	static bool inited;
	static s32 u2_vrt_ref, u2_term_ref, u2_enhance;
	static struct device_node *of_node;

	if (!inited) {
		u2_vrt_ref = u2_term_ref = u2_enhance = -1;
		of_node =
		    of_find_compatible_node(NULL, NULL, "mediatek,phy_tuning");
		if (of_node) {
			/* value won't be updated if property not being found
			 */
			of_property_read_u32(of_node, "u2_vrt_ref",
					     (u32 *)&u2_vrt_ref);
			of_property_read_u32(of_node, "u2_term_ref",
					     (u32 *)&u2_term_ref);
			of_property_read_u32(of_node, "u2_enhance",
					     (u32 *)&u2_enhance);
		}
		inited = true;
	} else if (!of_node)
		return;

	if (u2_vrt_ref != -1) {
		if (u2_vrt_ref <= VAL_MAX_WIDTH_3) {
			USBPHY_CLR32(OFFSET_RG_USB20_VRT_VREF_SEL,
				     VAL_MAX_WIDTH_3
					 << SHFT_RG_USB20_VRT_VREF_SEL);
			USBPHY_SET32(OFFSET_RG_USB20_VRT_VREF_SEL,
				     u2_vrt_ref << SHFT_RG_USB20_VRT_VREF_SEL);
		}
	}
	if (u2_term_ref != -1) {
		if (u2_term_ref <= VAL_MAX_WIDTH_3) {
			USBPHY_CLR32(OFFSET_RG_USB20_TERM_VREF_SEL,
				     VAL_MAX_WIDTH_3
					 << SHFT_RG_USB20_TERM_VREF_SEL);
			USBPHY_SET32(OFFSET_RG_USB20_TERM_VREF_SEL,
				     u2_term_ref
					 << SHFT_RG_USB20_TERM_VREF_SEL);
		}
	}
	if (u2_enhance != -1) {
		if (u2_enhance <= VAL_MAX_WIDTH_2) {
			USBPHY_CLR32(OFFSET_RG_USB20_PHY_REV6,
				     VAL_MAX_WIDTH_2
					 << SHFT_RG_USB20_PHY_REV6);
			USBPHY_SET32(OFFSET_RG_USB20_PHY_REV6,
				     u2_enhance << SHFT_RG_USB20_PHY_REV6);
		}
	}
}

#ifdef CONFIG_MTK_USB2JTAG_SUPPORT
int usb2jtag_usb_init(void)
{
	struct device_node *node = NULL;
	void __iomem *usb_phy_base;
	u32 temp;

	node = of_find_compatible_node(NULL, NULL, "mediatek,mt6763-usb20");
	if (!node) {
		pr_debug("[USB2JTAG] map node @ mediatek,USB0 failed\n");
		return -1;
	}

	usb_phy_base = of_iomap(node, 1);
	if (!usb_phy_base) {
		pr_debug("[USB2JTAG] iomap usb_phy_base failed\n");
		return -1;
	}

	/* rg_usb20_gpio_ctl: bit[9] = 1 */
	temp = readl(usb_phy_base + 0x820);
	writel(temp | (1 << 9), usb_phy_base + 0x820);

	/* RG_USB20_BGR_EN: bit[0] = 1 */
	temp = readl(usb_phy_base + 0x800);
	writel(temp | (1 << 0), usb_phy_base + 0x800);

	/* RG_USB20_BC11_SW_EN: bit[23] = 0 */
	temp = readl(usb_phy_base + 0x818);
	writel(temp & ~(1 << 23), usb_phy_base + 0x818);

	/* wait stable */
	mdelay(1);

	iounmap(usb_phy_base);

	return 0;
}
#endif

#ifdef CONFIG_MTK_UART_USB_SWITCH
bool in_uart_mode;
#endif

static DEFINE_SPINLOCK(musb_reg_clock_lock);

bool usb_enable_clock(bool enable)
{
	static int count;
	static int real_enable = 0, real_disable;
	static int virt_enable = 0, virt_disable;
	int res = -1;
	unsigned long flags;

	DBG(1, "enable(%d),count(%d),<%d,%d,%d,%d>\n", enable, count,
	    virt_enable, virt_disable, real_enable, real_disable);

	spin_lock_irqsave(&musb_reg_clock_lock, flags);

	if (enable && count == 0) {
		usb_hal_dpidle_request(USB_DPIDLE_FORBIDDEN);
		real_enable++;

#ifdef CONFIG_MTK_CLKMGR
		res = enable_clock(MT_CG_PERI_USB0, "PERI_USB");
#else
		res = clk_enable(musb_clk);
#endif
	} else if (!enable && count == 1) {
		real_disable++;
#ifdef CONFIG_MTK_CLKMGR
		res = disable_clock(MT_CG_PERI_USB0, "PERI_USB");
#else
		res = 0;
		clk_disable(musb_clk);
#endif
		usb_hal_dpidle_request(USB_DPIDLE_ALLOWED);
	}

	if (enable) {
		virt_enable++;
		count++;
	} else {
		virt_disable++;
		count = (count == 0) ? 0 : (count - 1);
	}

	spin_unlock_irqrestore(&musb_reg_clock_lock, flags);

	DBG(1, "enable(%d),count(%d),res(%d),<%d,%d,%d,%d>\n", enable, count,
	    res, virt_enable, virt_disable, real_enable, real_disable);
	return 1;
}

static void hs_slew_rate_cal(void)
{
	unsigned long data;
	unsigned long x;
	unsigned char value;
	unsigned long start_time, timeout;
	unsigned int timeout_flag = 0;
	/* enable usb ring oscillator. */
	USBPHY_SET32(0x14, (0x1 << 15));

	/* wait 1us. */
	udelay(1);

	/* enable free run clock */
	USBPHY_SET32(0xF10 - 0x800, (0x01 << 8));
	/* setting cyclecnt. */
	USBPHY_SET32(0xF00 - 0x800, (0x04 << 8));
	/* enable frequency meter */
	USBPHY_SET32(0xF00 - 0x800, (0x01 << 24));

	/* wait for frequency valid. */
	start_time = jiffies;
	timeout = jiffies + 3 * HZ;

	while (!((USBPHY_READ32(0xF10 - 0x800) & 0xFF) == 0x1)) {
		if (time_after(jiffies, timeout)) {
			timeout_flag = 1;
			break;
		}
	}

	/* read result. */
	if (timeout_flag) {
		DBG(0, "[USBPHY] Slew Rate Calibration: Timeout\n");
		value = 0x4;
	} else {
		data = USBPHY_READ32(0xF0C - 0x800);
		x = ((1024 * FRA * PARA) / data);
		value = (unsigned char)(x / 1000);
		if ((x - value * 1000) / 100 >= 5)
			value += 1;
		DBG(1, "[USBPHY]slew calibration:FM_OUT =%lu,x=%lu,value=%d\n",
		    data, x, value);
	}

	/* disable Frequency and disable free run clock. */
	USBPHY_CLR32(0xF00 - 0x800, (0x01 << 24));
	USBPHY_CLR32(0xF10 - 0x800, (0x01 << 8));

#define MSK_RG_USB20_HSTX_SRCTRL 0x7
	/* all clr first then set */
	USBPHY_CLR32(0x14, (MSK_RG_USB20_HSTX_SRCTRL << 12));
	USBPHY_SET32(0x14, ((value & MSK_RG_USB20_HSTX_SRCTRL) << 12));

	/* disable usb ring oscillator. */
	USBPHY_CLR32(0x14, (0x1 << 15));
}

#ifdef CONFIG_MTK_UART_USB_SWITCH
bool usb_phy_check_in_uart_mode(void)
{
	u32 usb_port_mode;

	usb_enable_clock(true);
	udelay(50);
	usb_port_mode = USBPHY_READ32(0x68);
	usb_enable_clock(false);

	if (((usb_port_mode >> 30) & 0x3) == 1) {
		DBG(0, "%s:%d - IN UART MODE : 0x%x\n", __func__, __LINE__,
		    usb_port_mode);
		in_uart_mode = true;
	} else {
		DBG(0, "%s:%d - NOT IN UART MODE : 0x%x\n", __func__, __LINE__,
		    usb_port_mode);
		in_uart_mode = false;
	}
	return in_uart_mode;
}

void usb_phy_switch_to_uart(void)
{
	unsigned int val = 0;

	if (usb_phy_check_in_uart_mode()) {
		DBG(0, "Already in UART mode.\n");
		return;
	}

	usb_enable_clock(true);
	udelay(50);

	/* RG_USB20_BC11_SW_EN 0x11F4_0818[23] = 1'b0 */
	USBPHY_CLR32(0x18, (0x1 << 23));

	/* Set RG_SUSPENDM 0x11F4_0868[3] to 1 */
	USBPHY_SET32(0x68, (0x1 << 3));

	/* force suspendm 0x11F4_0868[18] = 1 */
	USBPHY_SET32(0x68, (0x1 << 18));

	/* Set rg_uart_mode 0x11F4_0868[31:30] to 2'b01 */
	USBPHY_CLR32(0x68, (0x3 << 30));
	USBPHY_SET32(0x68, (0x1 << 30));

	/* force_uart_i 0x11F4_0868[29] = 0*/
	USBPHY_CLR32(0x68, (0x1 << 29));

	/* force_uart_bias_en 0x11F4_0868[28] = 1 */
	USBPHY_SET32(0x68, (0x1 << 28));

	/* force_uart_tx_oe 0x11F4_0868[27] = 1 */
	USBPHY_SET32(0x68, (0x1 << 27));

	/* force_uart_en 0x11F4_0868[26] = 1 */
	USBPHY_SET32(0x68, (0x1 << 26));

	/* RG_UART_BIAS_EN 0x11F4_086c[18] = 1 */
	USBPHY_SET32(0x6C, (0x1 << 18));

	/* RG_UART_TX_OE 0x11F4_086c[17] = 1 */
	USBPHY_SET32(0x6C, (0x1 << 17));

	/* Set RG_UART_EN to 1 */
	USBPHY_SET32(0x6C, (0x1 << 16));

	/* Set RG_USB20_DM_100K_EN to 1 */
	USBPHY_SET32(0x20, (0x1 << 17));

	usb_enable_clock(false);

	/* GPIO Selection */
	val = readl(ap_gpio_base);
	writel(val & (~(GPIO_SEL_MASK)), ap_gpio_base);

	val = readl(ap_gpio_base);
	writel(val | (GPIO_SEL_UART0), ap_gpio_base);

	in_uart_mode = true;
}

void usb_phy_switch_to_usb(void)
{
	unsigned int val = 0;

	/* GPIO Selection */
	val = readl(ap_gpio_base);
	writel(val & (~(GPIO_SEL_MASK)), ap_gpio_base);

	usb_enable_clock(true);
	udelay(50);
	/* clear force_uart_en */
	USBPHY_CLR32(0x68, (0x1 << 26));

	/* Set rg_uart_mode 0x11F4_0868[31:30] to 2'b00 */
	USBPHY_CLR32(0x68, (0x3 << 30));

	in_uart_mode = false;

	usb_enable_clock(false);

	usb_phy_poweron();
	/* disable the USB clock turned on in usb_phy_poweron() */
	usb_enable_clock(false);
}
#endif

void set_usb_phy_mode(int mode)
{
	switch (mode) {
	case PHY_DEV_ACTIVE:
		/* VBUSVALID=1, AVALID=1, BVALID=1, SESSEND=0, IDDIG=1,
		 * IDPULLUP=1
		 */
		USBPHY_CLR32(0x6C, (0x10 << 0));
		USBPHY_SET32(0x6C, (0x2F << 0));
		USBPHY_SET32(0x6C, (0x3F << 8));
		break;
	case PHY_HOST_ACTIVE:
		/* VBUSVALID=1, AVALID=1, BVALID=1, SESSEND=0, IDDIG=0,
		 * IDPULLUP=1
		 */
		USBPHY_CLR32(0x6c, (0x12 << 0));
		USBPHY_SET32(0x6c, (0x2d << 0));
		USBPHY_SET32(0x6c, (0x3f << 8));
		break;
	case PHY_IDLE_MODE:
		/* VBUSVALID=0, AVALID=0, BVALID=0, SESSEND=1, IDDIG=0,
		 * IDPULLUP=1
		 */
		USBPHY_SET32(0x6c, (0x11 << 0));
		USBPHY_CLR32(0x6c, (0x2e << 0));
		USBPHY_SET32(0x6c, (0x3f << 8));
		break;
	default:
		DBG(0, "mode error %d\n", mode);
	}
	DBG(0, "force PHY to mode %d, 0x6c=%x\n", mode, USBPHY_READ32(0x6c));
}

void usb_rev6_setting(int value)
{
	static int direct_return;

	if (direct_return)
		return;

	/* RG_USB20_PHY_REV[7:0] = 8'b01000000 */
	USBPHY_CLR32(0x18, (0xFF << 24));

	if (value)
		USBPHY_SET32(0x18, (value << 24));
	else
		direct_return = 1;
}

/* M17_USB_PWR Sequence 20160603.xls */
void usb_phy_poweron(void)
{
#ifdef CONFIG_MTK_UART_USB_SWITCH
	if (in_uart_mode) {
		DBG(0, "At UART mode. No %s\n", __func__);
		return;
	}
#endif
	/* enable USB MAC clock. */
	usb_enable_clock(true);

	/* wait 50 usec for PHY3.3v/1.8v stable. */
	udelay(50);

	/*
	 * force_uart_en	1'b0		0x68 26
	 * RG_UART_EN		1'b0		0x6c 16
	 * rg_usb20_gpio_ctl	1'b0		0x20 09
	 * usb20_gpio_mode	1'b0		0x20 08
	 * RG_USB20_BC11_SW_EN	1'b0		0x18 23
	 * rg_usb20_dp_100k_mode 1'b1		0x20 18
	 * USB20_DP_100K_EN	1'b0		0x20 16
	 * RG_USB20_DM_100K_EN	1'b0		0x20 17
	 * RG_USB20_OTG_VBUSCMP_EN	1'b1	0x18 20
	 * force_suspendm		1'b0	0x68 18
	 */

	/* force_uart_en, 1'b0 */
	USBPHY_CLR32(0x68, (0x1 << 26));
	/* RG_UART_EN, 1'b0 */
	USBPHY_CLR32(0x6c, (0x1 << 16));
	/* rg_usb20_gpio_ctl, 1'b0, usb20_gpio_mode, 1'b0 */
	USBPHY_CLR32(0x20, ((0x1 << 9) | (0x1 << 8)));

	/* RG_USB20_BC11_SW_EN, 1'b0 */
	USBPHY_CLR32(0x18, (0x1 << 23));

	/* rg_usb20_dp_100k_mode, 1'b1 */
	USBPHY_SET32(0x20, (0x1 << 18));
	/* USB20_DP_100K_EN 1'b0, RG_USB20_DM_100K_EN, 1'b0 */
	USBPHY_CLR32(0x20, ((0x1 << 16) | (0x1 << 17)));

	/* RG_USB20_OTG_VBUSCMP_EN, 1'b1 */
	USBPHY_SET32(0x18, (0x1 << 20));

	/* force_suspendm, 1'b0 */
	USBPHY_CLR32(0x68, (0x1 << 18));

	/* RG_USB20_PHY_REV[7:0] = 8'b01000000 */
	USBPHY_CLR32(0x18, (0xFF << 24));
	USBPHY_SET32(0x18, (0x40 << 24));

	/* wait for 800 usec. */
	udelay(800);

	DBG(0, "usb power on success\n");
}

/* M17_USB_PWR Sequence 20160603.xls */
static void usb_phy_savecurrent_internal(void)
{
#ifdef CONFIG_MTK_UART_USB_SWITCH
	if (in_uart_mode) {
		DBG(0, "At UART mode. No %s\n", __func__);
		return;
	}
#endif
	/*
	 * force_uart_en	1'b0		0x68 26
	 * RG_UART_EN		1'b0		0x6c 16
	 * rg_usb20_gpio_ctl	1'b0		0x20 09
	 * usb20_gpio_mode	1'b0		0x20 08

	 * RG_USB20_BC11_SW_EN	1'b0		0x18 23
	 * RG_USB20_OTG_VBUSCMP_EN	1'b0	0x18 20
	 * RG_SUSPENDM		1'b1		0x68 03
	 * force_suspendm	1'b1		0x68 18

	 * RG_DPPULLDOWN	1'b1		0x68 06
	 * RG_DMPULLDOWN	1'b1		0x68 07
	 * RG_XCVRSEL[1:0]	2'b01		0x68 [04-05]
	 * RG_TERMSEL		1'b1		0x68 02
	 * RG_DATAIN[3:0]	4'b0000		0x68 [10-13]
	 * force_dp_pulldown	1'b1		0x68 20
	 * force_dm_pulldown	1'b1		0x68 21
	 * force_xcversel	1'b1		0x68 19
	 * force_termsel	1'b1		0x68 17
	 * force_datain		1'b1		0x68 23

	 * RG_SUSPENDM		1'b0		0x68 03
	 */
	/* force_uart_en, 1'b0 */
	USBPHY_CLR32(0x68, (0x1 << 26));
	/* RG_UART_EN, 1'b0 */
	USBPHY_CLR32(0x6c, (0x1 << 16));
	/* rg_usb20_gpio_ctl, 1'b0, usb20_gpio_mode, 1'b0 */
	USBPHY_CLR32(0x20, (0x1 << 9));
	USBPHY_CLR32(0x20, (0x1 << 8));

	/* RG_USB20_BC11_SW_EN, 1'b0 */
	USBPHY_CLR32(0x18, (0x1 << 23));
	/* RG_USB20_OTG_VBUSCMP_EN, 1'b0 */
	USBPHY_CLR32(0x18, (0x1 << 20));

	/* RG_SUSPENDM, 1'b1 */
	USBPHY_SET32(0x68, (0x1 << 3));
	/* force_suspendm, 1'b1 */
	USBPHY_SET32(0x68, (0x1 << 18));

	/* RG_DPPULLDOWN, 1'b1, RG_DMPULLDOWN, 1'b1 */
	USBPHY_SET32(0x68, ((0x1 << 6) | (0x1 << 7)));

	/* RG_XCVRSEL[1:0], 2'b01. */
	USBPHY_CLR32(0x68, (0x3 << 4));
	USBPHY_SET32(0x68, (0x1 << 4));
	/* RG_TERMSEL, 1'b1 */
	USBPHY_SET32(0x68, (0x1 << 2));
	/* RG_DATAIN[3:0], 4'b0000 */
	USBPHY_CLR32(0x68, (0xF << 10));

	/* force_dp_pulldown, 1'b1, force_dm_pulldown, 1'b1,
	 * force_xcversel, 1'b1, force_termsel, 1'b1, force_datain, 1'b1
	 */
	USBPHY_SET32(0x68, ((0x1 << 20) | (0x1 << 21) | (0x1 << 19) |
			    (0x1 << 17) | (0x1 << 23)));

	udelay(800);

	/* RG_SUSPENDM, 1'b0 */
	USBPHY_CLR32(0x68, (0x1 << 3));

	udelay(1);

	set_usb_phy_mode(PHY_IDLE_MODE);
}

void usb_phy_savecurrent(void)
{

	usb_phy_savecurrent_internal();

	/* 4 14. turn off internal 48Mhz PLL. */
	usb_enable_clock(false);

	DBG(0, "usb save current success\n");
}

/* M17_USB_PWR Sequence 20160603.xls */
void usb_phy_recover(void)
{
	unsigned int efuse_val = 0;

#ifdef CONFIG_MTK_UART_USB_SWITCH
	if (in_uart_mode) {
		DBG(0, "At UART mode. No %s\n", __func__);
		return;
	}
#endif
	/* turn on USB reference clock. */
	usb_enable_clock(true);

	/* wait 50 usec. */
	udelay(50);

	/*
	 * 04.force_uart_en	1'b0 0x68 26
	 * 04.RG_UART_EN		1'b0 0x6C 16
	 * 04.rg_usb20_gpio_ctl	1'b0 0x20 09
	 * 04.usb20_gpio_mode	1'b0 0x20 08

	 * 05.force_suspendm	1'b0 0x68 18

	 * 06.RG_DPPULLDOWN	1'b0 0x68 06
	 * 07.RG_DMPULLDOWN	1'b0 0x68 07
	 * 08.RG_XCVRSEL[1:0]	2'b00 0x68 [04:05]
	 * 09.RG_TERMSEL		1'b0 0x68 02
	 * 10.RG_DATAIN[3:0]	4'b0000 0x68 [10:13]
	 * 11.force_dp_pulldown	1'b0 0x68 20
	 * 12.force_dm_pulldown	1'b0 0x68 21
	 * 13.force_xcversel	1'b0 0x68 19
	 * 14.force_termsel	1'b0 0x68 17
	 * 15.force_datain	1'b0 0x68 23
	 * 16.RG_USB20_BC11_SW_EN	1'b0 0x18 23
	 * 17.RG_USB20_OTG_VBUSCMP_EN	1'b1 0x18 20
	 */

	/* clean PUPD_BIST_EN */
	/* PUPD_BIST_EN = 1'b0 */
	/* PMIC will use it to detect charger type */
	/* NEED?? USBPHY_CLR8(0x1d, 0x10);*/
	USBPHY_CLR32(0x1c, (0x1 << 12));

	/* force_uart_en, 1'b0 */
	USBPHY_CLR32(0x68, (0x1 << 26));
	/* RG_UART_EN, 1'b0 */
	USBPHY_CLR32(0x6C, (0x1 << 16));
	/* rg_usb20_gpio_ctl, 1'b0, usb20_gpio_mode, 1'b0 */
	USBPHY_CLR32(0x20, (0x1 << 9));
	USBPHY_CLR32(0x20, (0x1 << 8));

	/* force_suspendm, 1'b0 */
	USBPHY_CLR32(0x68, (0x1 << 18));

	/* RG_DPPULLDOWN, 1'b0, RG_DMPULLDOWN, 1'b0 */
	USBPHY_CLR32(0x68, ((0x1 << 6) | (0x1 << 7)));

	/* RG_XCVRSEL[1:0], 2'b00. */
	USBPHY_CLR32(0x68, (0x3 << 4));

	/* RG_TERMSEL, 1'b0 */
	USBPHY_CLR32(0x68, (0x1 << 2));
	/* RG_DATAIN[3:0], 4'b0000 */
	USBPHY_CLR32(0x68, (0xF << 10));

	/* force_dp_pulldown, 1'b0, force_dm_pulldown, 1'b0,
	 * force_xcversel, 1'b0, force_termsel, 1'b0, force_datain, 1'b0
	 */
	USBPHY_CLR32(0x68, ((0x1 << 20) | (0x1 << 21) | (0x1 << 19) |
			    (0x1 << 17) | (0x1 << 23)));

	/* RG_USB20_BC11_SW_EN, 1'b0 */
	USBPHY_CLR32(0x18, (0x1 << 23));
	/* RG_USB20_OTG_VBUSCMP_EN, 1'b1 */
	USBPHY_SET32(0x18, (0x1 << 20));

	/* RG_USB20_PHY_REV[7:0] = 8'b01000000 */
	usb_rev6_setting(0x40);

	/* wait 800 usec. */
	udelay(800);

	/* force enter device mode */
	set_usb_phy_mode(PHY_DEV_ACTIVE);

	hs_slew_rate_cal();

	/* M_ANALOG8[4:0] => RG_USB20_INTR_CAL[4:0] */
	efuse_val = (get_devinfo_with_index(108) & (0x1f << 0)) >> 0;
	if (efuse_val) {
		DBG(0, "apply efuse setting, RG_USB20_INTR_CAL=0x%x\n",
		    efuse_val);
		USBPHY_CLR32(0x04, (0x1F << 19));
		USBPHY_SET32(0x04, (efuse_val << 19));
	}

	/* disc threshold to max, RG_USB20_DISCTH[7:4], dft:1000, MAX:1111 */
	USBPHY_SET32(0x18, (0xf0 << 0));

	usb_phy_tuning();

	DBG(0, "usb recovery success\n");
}

/* BC1.2 */
void Charger_Detect_Init(void)
{
	unsigned long flags;
	int do_lock = 0;

	if ((get_boot_mode() == META_BOOT) ||
	    (get_boot_mode() == ADVMETA_BOOT)) {
		DBG(0, "%s Skip\n", __func__);
		return;
	}

	if (mtk_musb) {
		spin_lock_irqsave(&mtk_musb->lock, flags);
		do_lock = 1;
	} else
		DBG(0, "mtk_musb is NULL\n");

	/* turn on USB reference clock. */
	usb_enable_clock(true);

	/* wait 50 usec. */
	udelay(50);

	/* RG_USB20_BC11_SW_EN = 1'b1 */
	USBPHY_SET32(0x18, (0x1 << 23));

	/* turn off USB reference clock. */
	usb_enable_clock(false);

	if (do_lock)
		spin_unlock_irqrestore(&mtk_musb->lock, flags);
	DBG(0, "%s\n", __func__);
}

void Charger_Detect_Release(void)
{
	unsigned long flags;
	int do_lock = 0;

	if ((get_boot_mode() == META_BOOT) ||
	    (get_boot_mode() == ADVMETA_BOOT)) {
		DBG(0, "%s Skip\n", __func__);
		return;
	}

	if (mtk_musb) {
		spin_lock_irqsave(&mtk_musb->lock, flags);
		do_lock = 1;
	} else
		DBG(0, "mtk_musb is NULL\n");

	/* turn on USB reference clock. */
	usb_enable_clock(true);

	/* RG_USB20_BC11_SW_EN = 1'b0 */
	USBPHY_CLR32(0x18, (0x1 << 23));

	udelay(1);

	/* 4 14. turn off internal 48Mhz PLL. */
	usb_enable_clock(false);

	if (do_lock)
		spin_unlock_irqrestore(&mtk_musb->lock, flags);

	DBG(0, "%s\n", __func__);
}

void usb_phy_context_save(void)
{
#ifdef CONFIG_MTK_UART_USB_SWITCH
	in_uart_mode = usb_phy_check_in_uart_mode();
#endif
}

void usb_phy_context_restore(void)
{
#ifdef CONFIG_MTK_UART_USB_SWITCH
	if (in_uart_mode)
		usb_phy_switch_to_uart();
#endif
}

#endif

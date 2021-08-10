/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>

#include <mt-plat/charger_type.h>
#include <mt-plat/mtk_boot.h>
#include <mt-plat/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>

/* #define __FORCE_USB_TYPE__ */
#define __SW_CHRDET_IN_PROBE_PHASE__

static enum charger_type g_chr_type;
#ifdef __SW_CHRDET_IN_PROBE_PHASE__
static struct work_struct chr_work;
#endif
static DEFINE_MUTEX(chrdet_lock);
static struct power_supply *chrdet_psy;


bool __attribute__((weak)) mt_usb_is_device(void)
{
	pr_notice("%s is not defined, return true\n", __func__);
	return true;
}

static int chrdet_inform_psy_changed(enum charger_type chg_type,
				bool chg_online)
{
	int ret = 0;
	union power_supply_propval propval;

	pr_info("charger type: %s: online = %d, type = %d\n", __func__,
		chg_online, chg_type);

	/* Inform chg det power supply */
	if (chg_online) {
		propval.intval = chg_online;
		ret = power_supply_set_property(chrdet_psy,
				POWER_SUPPLY_PROP_ONLINE, &propval);
		if (ret < 0)
			pr_notice("%s: psy online failed, ret = %d\n",
				__func__, ret);

		propval.intval = chg_type;
		ret = power_supply_set_property(chrdet_psy,
				POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
		if (ret < 0)
			pr_notice("%s: psy type failed, ret = %d\n",
				__func__, ret);

		return ret;
	}

	propval.intval = chg_type;
	ret = power_supply_set_property(chrdet_psy,
				POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
	if (ret < 0)
		pr_notice("%s: psy type failed, ret(%d)\n", __func__, ret);

	propval.intval = chg_online;
	ret = power_supply_set_property(chrdet_psy, POWER_SUPPLY_PROP_ONLINE,
				&propval);
	if (ret < 0)
		pr_notice("%s: psy online failed, ret(%d)\n", __func__, ret);
	return ret;
}

int hw_charging_get_charger_type(void)
{
	/* Force Standard USB Host */
	g_chr_type = STANDARD_HOST;
	chrdet_inform_psy_changed(g_chr_type, 1);
	return g_chr_type;
}

/* Charger Detection */
void do_charger_detect(void)
{
	if (!mt_usb_is_device()) {
		g_chr_type = CHARGER_UNKNOWN;
		pr_info("charger type: Now is usb host mode. Skip detection\n");
		return;
	}

	if (is_meta_mode()) {
		/* Skip charger type detection to speed up meta boot */
		pr_notice("charger type: force Standard USB Host in meta\n");
		g_chr_type = STANDARD_HOST;
		chrdet_inform_psy_changed(g_chr_type, 1);
		return;
	}

	mutex_lock(&chrdet_lock);

	if (pmic_get_register_value(PMIC_RGS_CHRDET)) {
		pr_info("charger type: charger IN\n");
		g_chr_type = hw_charging_get_charger_type();
		chrdet_inform_psy_changed(g_chr_type, 1);
	} else {
		pr_info("charger type: charger OUT\n");
		g_chr_type = CHARGER_UNKNOWN;
		chrdet_inform_psy_changed(g_chr_type, 0);
	}

	mutex_unlock(&chrdet_lock);
}



/* PMIC Int Handler */
void chrdet_int_handler(void)
{
	/*
	 * pr_notice("[chrdet_int_handler]CHRDET status = %d....\n",
	 *	pmic_get_register_value(PMIC_RGS_CHRDET));
	 */
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	if (!pmic_get_register_value(PMIC_RGS_CHRDET)) {
		int boot_mode = 0;

		boot_mode = get_boot_mode();

		if (boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT
		    || boot_mode == LOW_POWER_OFF_CHARGING_BOOT) {
			pr_info("[%s] Unplug Charger/USB\n", __func__);
#ifndef CONFIG_TCPC_CLASS
			mt_power_off();
#else
			return;
#endif
		}
	}
#endif
	do_charger_detect();
}


/* Charger Probe Related */
#ifdef __SW_CHRDET_IN_PROBE_PHASE__
static void do_charger_detection_work(struct work_struct *data)
{
	if (pmic_get_register_value(PMIC_RGS_CHRDET))
		do_charger_detect();
}
#endif

static int __init pmic_chrdet_init(void)
{
	mutex_init(&chrdet_lock);
	chrdet_psy = power_supply_get_by_name("charger");
	if (!chrdet_psy) {
		pr_notice("%s: get power supply failed\n", __func__);
		return -EINVAL;
	}

#ifdef __SW_CHRDET_IN_PROBE_PHASE__
	/* do charger detect here to prevent HW miss interrupt*/
	INIT_WORK(&chr_work, do_charger_detection_work);
	schedule_work(&chr_work);
#endif

	pmic_register_interrupt_callback(INT_CHRDET_EDGE, chrdet_int_handler);
	pmic_enable_interrupt(INT_CHRDET_EDGE, 1, "PMIC");

	return 0;
}

late_initcall(pmic_chrdet_init);

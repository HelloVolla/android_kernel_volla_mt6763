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

#include <generated/autoconf.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/pm_wakeup.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/iio/consumer.h>
#include <mt-plat/upmu_common.h>
#include <mt-plat/mtk_auxadc_intf.h>
#if defined(CONFIG_MTK_SELINUX_AEE_WARNING)
#include <mt-plat/aee.h>
#endif
#include <mach/mtk_pmic.h>
#include "include/pmic.h"
#include "include/pmic_throttling_dlpt.h"
#include "include/pmic_auxadc.h"
#include <pmic_lbat_service.h>
#include <mtk_idle.h>
#include <mt-plat/mtk_charger.h>


#if (CONFIG_MTK_GAUGE_VERSION == 30)
#include <mt-plat/mtk_battery.h>
#include <mach/mtk_battery_property.h>
#include <linux/reboot.h>
#include <mtk_battery_internal.h>
#else
#include <mt-plat/battery_meter.h>
#include <mt-plat/battery_common.h>
#include <mach/mtk_battery_meter.h>
#endif

/*****************************************************************************
 * PMIC related define
 ******************************************************************************/
#define PMIC_THROTTLING_DLPT_UT	0
#define UNIT_FGCURRENT     (314331)
/*****************************************************************************
 * PMIC PT and DLPT UT
 ******************************************************************************/
#if PMIC_THROTTLING_DLPT_UT
/* UT test code TBD */
void low_bat_test(enum LOW_BATTERY_LEVEL_TAG level_val)
{
	pr_info("[%s] get %d\n", __func__, level_val);
}

void bat_oc_test(enum BATTERY_OC_LEVEL_TAG level_val)
{
	pr_info("[%s] get %d\n", __func__, level_val);
}

void bat_per_test(enum BATTERY_PERCENT_LEVEL_TAG level_val)
{
	pr_info("[%s] get %d\n", __func__, level_val);
}

#if defined(POWER_BAT_OC_CURRENT_H)

#undef POWER_BAT_OC_CURRENT_H
#undef POWER_BAT_OC_CURRENT_L

#define POWER_BAT_OC_CURRENT_H	500
#define POWER_BAT_OC_CURRENT_L	600
#endif

#endif

bool __attribute__ ((weak)) is_power_path_supported(void)
{
	pr_notice_once("%s: check mtk_charger\n", __func__);
	return 0;
}

/***************************************************************************
 * Low battery call back function
 ****************************************************************************/
#ifndef DISABLE_LOW_BATTERY_PROTECT
#define LOW_BATTERY_PROTECT
#endif

#ifdef LOW_BATTERY_PROTECT

#if PMIC_THROTTLING_DLPT_UT
static struct lbat_user lbat_test1;
static struct lbat_user lbat_test2;
static struct lbat_user lbat_test3;

void lbat_test_callback(unsigned int thd)
{
	pr_notice("[%s] thd=%d\n", __func__, thd);
}
#endif

static struct lbat_user lbat_pt;
int g_low_battery_level;
int g_low_battery_stop;
/* give one change to ignore DLPT power off. battery voltage
 * may return to 3.25 or higher because loading become light.
 */
int g_low_battery_if_power_off;

struct low_battery_callback_table {
	void (*lbcb)(enum LOW_BATTERY_LEVEL_TAG);
};

struct low_battery_callback_table lbcb_tb[] = {
	{NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL},
	{NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}
};

void register_low_battery_notify(
	void (*low_battery_callback)(enum LOW_BATTERY_LEVEL_TAG),
	enum LOW_BATTERY_PRIO_TAG prio_val)
{
	PMICLOG("[%s] start\n", __func__);

	lbcb_tb[prio_val].lbcb = low_battery_callback;

	pr_info("[%s] prio_val=%d\n", __func__, prio_val);
}

void exec_low_battery_callback(unsigned int thd)
{
	int i = 0;
	enum LOW_BATTERY_LEVEL_TAG low_battery_level = 0;

	if (g_low_battery_stop == 1) {
		pr_info("[%s] g_low_battery_stop=%d\n"
			, __func__, g_low_battery_stop);
	} else {
		if (thd == POWER_INT0_VOLT)
			low_battery_level = LOW_BATTERY_LEVEL_0;
		else if (thd == POWER_INT1_VOLT)
			low_battery_level = LOW_BATTERY_LEVEL_1;
		else if (thd == POWER_INT2_VOLT)
			low_battery_level = LOW_BATTERY_LEVEL_2;
		g_low_battery_level = low_battery_level;
		for (i = 0; i < ARRAY_SIZE(lbcb_tb); i++) {
			if (lbcb_tb[i].lbcb != NULL)
				lbcb_tb[i].lbcb(low_battery_level);
		}
	}
#if PMIC_THROTTLING_DLPT_UT
	pr_info("[%s] prio_val=%d,low_battery=%d\n"
			, __func__, i, low_battery_level);
	lbat_dump_reg();
#else
	pr_info("[%s] prio_val=%d,low_battery=%d\n"
			, __func__, i, low_battery_level);
#endif
}

void low_battery_protect_init(void)
{
	int ret = 0;

	ret = lbat_user_register(&lbat_pt, "power throttling"
			, POWER_INT0_VOLT, POWER_INT1_VOLT
			, POWER_INT2_VOLT, exec_low_battery_callback);
#if PMIC_THROTTLING_DLPT_UT
	ret = lbat_user_register(&lbat_test1, "test1",
		3450, 3200, 3000, lbat_test_callback);
	ret = lbat_user_register(&lbat_test2, "test2",
		POWER_INT0_VOLT, 2900, 2800, lbat_test_callback);
	ret = lbat_user_register(&lbat_test3, "test3",
		3450, 3200, 3000, NULL);
#endif
	if (ret)
		pr_notice("[%s] error ret=%d\n", __func__, ret);

	lbat_dump_reg();
	pr_info("[%s] %d mV, %d mV, %d mV Done\n"
		, __func__, POWER_INT0_VOLT
		, POWER_INT1_VOLT, POWER_INT2_VOLT);
}

int dlpt_check_power_off(void)
{
	int ret = 0;

	ret = 0;
	if (g_low_battery_stop == 1) {
		PMICLOG("[%s] not start\n", __func__);
		return ret;
	}
	if (g_low_battery_level == 2) {
		/*1st time receive battery voltage < 3.1V, record it */
		if (g_low_battery_if_power_off == 0) {
			g_low_battery_if_power_off++;
			pr_info("[%s] %d\n", __func__,
				g_low_battery_if_power_off);
		} else {
			/* 2nd time receive battery voltage < 3.1V */
			/* Wait FG to call power off */
			ret = 1;
			pr_info("[%s] %d %d\n", __func__,
				ret, g_low_battery_if_power_off);
		}
	} else {
		ret = 0;
		/* battery voltage > 3.1V, ignore it */
		g_low_battery_if_power_off = 0;
	}
	PMICLOG("[%s]g_low_battery_level=%d, ret=%d\n", __func__,
		g_low_battery_level, ret);

	return ret;
}

#else
void __attribute__ ((weak)) register_low_battery_notify(
	void (*low_battery_callback)(enum LOW_BATTERY_LEVEL_TAG),
	enum LOW_BATTERY_PRIO_TAG prio_val)
{
}

int __attribute__ ((weak)) dlpt_check_power_off(void)
{
	return 0;
}
#endif /*#ifdef LOW_BATTERY_PROTECT*/

/*******************************************************************
 * Battery OC call back function
 *******************************************************************/
#define OCCB_NUM 16

#ifndef DISABLE_BATTERY_OC_PROTECT
#define BATTERY_OC_PROTECT
#endif

#ifdef BATTERY_OC_PROTECT

#if (CONFIG_MTK_GAUGE_VERSION == 30)
/*
 * I(0.1mA)=reg *UNIT_FGCURRENT /100000 * 100 / fg_cust_data.r_fg_value *
 * fg_cust_data.car_tune_value /1000
 *
 * I(1mA)=reg *UNIT_FGCURRENT /100000 *100 /fg_cust_data.r_fg_value *
 * fg_cust_data.car_tune_value /1000 /10
 *
 * Reg=I /UNIT_FGCURRENT *100000 /100 *fg_cust_data.r_fg_value /
 * fg_cust_data.car_tune_value *10000 *95 /100
 *
 * Ricky:need *0.95
 * 65535-reg
 * (65535-(I *fg_cust_data.r_fg_value *1000 /UNIT_FGCURRENT *95 *100
 *  /fg_cust_data.car_tune_value))
 */
#define bat_oc_h_thd(cur)   \
(65535-(cur*fg_cust_data.r_fg_value*1000/UNIT_FGCURRENT* \
	95*100/fg_cust_data.car_tune_value))

#else /* CONFIG_MTK_GAUGE_VERSION != 30 */
/* ex. Ireg = 65535 - (I * 950000uA / 2 / 158.122 / CAR_TUNE_VALUE * 100)*/
/* (950000/2/158.122)*100~=300400*/
#define bat_oc_h_thd(cur)   \
(65535-((300400*cur/1000)/batt_meter_cust_data.car_tune_value))	/*ex: 4670mA*/

#endif /* end of #if CONFIG_MTK_GAUGE_VERSION == 30 */

#define bat_oc_l_thd(cur) bat_oc_h_thd(cur)

int g_battery_oc_level;
int g_battery_oc_stop;
int g_battery_oc_h_thd;
int g_battery_oc_l_thd;

struct battery_oc_callback_table {
	void (*occb)(enum BATTERY_OC_LEVEL_TAG);
};

struct battery_oc_callback_table occb_tb[] = {
	{NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL},
	{NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}
};

void register_battery_oc_notify(
	void (*battery_oc_callback)(enum BATTERY_OC_LEVEL_TAG),
	enum BATTERY_OC_PRIO_TAG prio_val)
{
	PMICLOG("[%s] start\n", __func__);

	occb_tb[prio_val].occb = battery_oc_callback;

	pr_info("[%s] prio_val=%d\n", __func__, prio_val);
}

void exec_battery_oc_callback(enum BATTERY_OC_LEVEL_TAG battery_oc_level)
{
	int i = 0;

	if (g_battery_oc_stop == 1) {
		pr_info("[%s] g_battery_oc_stop=%d\n"
			, __func__, g_battery_oc_stop);
	} else {
		for (i = 0; i < OCCB_NUM; i++) {
			if (occb_tb[i].occb != NULL) {
				occb_tb[i].occb(battery_oc_level);
				pr_info("[%s] prio_val=%d,battery_oc_level=%d\n"
					, __func__, i, battery_oc_level);
			}
		}
	}
}

void bat_oc_h_en_setting(int en_val)
{
	pmic_enable_interrupt(INT_FG_CUR_H, en_val, "pmic_throttling_dlpt");
}

void bat_oc_l_en_setting(int en_val)
{
	pmic_enable_interrupt(INT_FG_CUR_L, en_val, "pmic_throttling_dlpt");
}


void fg_cur_h_int_handler(void)
{
#if PMIC_THROTTLING_DLPT_UT
	pr_info("[%s]\n", __func__);
#else
	PMICLOG("[%s]\n", __func__);
#endif

	g_battery_oc_level = 0;
	exec_battery_oc_callback(BATTERY_OC_LEVEL_0);
	bat_oc_h_en_setting(0);
	bat_oc_l_en_setting(0);
	mdelay(1);
	bat_oc_l_en_setting(1);

#if PMIC_THROTTLING_DLPT_UT
	pr_info("FG_CUR_HTH = 0x%x, FG_CUR_LTH = 0x%x, RG_INT_EN_FG_CUR_H = %d, RG_INT_EN_FG_CUR_L = %d\n"
		, pmic_get_register_value(PMIC_FG_CUR_HTH)
		, pmic_get_register_value(PMIC_FG_CUR_LTH)
		, pmic_get_register_value(PMIC_RG_INT_EN_FG_CUR_H)
		, pmic_get_register_value(PMIC_RG_INT_EN_FG_CUR_L));
#else
	PMICLOG("Reg[0x%x]=0x%x, Reg[0x%x]=0x%x, Reg[0x%x]=0x%x\n",
		PMIC_FG_CUR_HTH_ADDR,
		upmu_get_reg_value(PMIC_FG_CUR_HTH_ADDR),
		PMIC_FG_CUR_LTH_ADDR,
		upmu_get_reg_value(PMIC_FG_CUR_LTH_ADDR),
		PMIC_RG_INT_EN_FG_BAT0_H_ADDR,
		upmu_get_reg_value(PMIC_RG_INT_EN_FG_BAT0_H_ADDR));
#endif
}

void fg_cur_l_int_handler(void)
{
#if PMIC_THROTTLING_DLPT_UT
	pr_info("[%s]\n", __func__);
#else
	PMICLOG("[%s]\n", __func__);
#endif

	g_battery_oc_level = 1;
	exec_battery_oc_callback(BATTERY_OC_LEVEL_1);
	bat_oc_h_en_setting(0);
	bat_oc_l_en_setting(0);
	mdelay(1);
	bat_oc_h_en_setting(1);

#if PMIC_THROTTLING_DLPT_UT
	pr_info("FG_CUR_HTH = 0x%x, FG_CUR_LTH = 0x%x, RG_INT_EN_FG_CUR_H = %d,	RG_INT_EN_FG_CUR_L = %d\n"
		, pmic_get_register_value(PMIC_FG_CUR_HTH)
		, pmic_get_register_value(PMIC_FG_CUR_LTH)
		, pmic_get_register_value(PMIC_RG_INT_EN_FG_CUR_H)
		, pmic_get_register_value(PMIC_RG_INT_EN_FG_CUR_L));
#else
	PMICLOG("Reg[0x%x]=0x%x, Reg[0x%x]=0x%x, Reg[0x%x]=0x%x\n",
		PMIC_FG_CUR_HTH_ADDR,
		upmu_get_reg_value(PMIC_FG_CUR_HTH_ADDR),
		PMIC_FG_CUR_LTH_ADDR,
		upmu_get_reg_value(PMIC_FG_CUR_LTH_ADDR),
		PMIC_RG_INT_EN_FG_BAT0_H_ADDR,
		upmu_get_reg_value(PMIC_RG_INT_EN_FG_BAT0_H_ADDR));
#endif
}

void battery_oc_protect_init(void)
{
	pmic_register_interrupt_callback(INT_FG_CUR_H, fg_cur_h_int_handler);
	pmic_register_interrupt_callback(INT_FG_CUR_L, fg_cur_l_int_handler);

	pmic_set_register_value(PMIC_FG_CUR_HTH
				, bat_oc_h_thd(POWER_BAT_OC_CURRENT_H));
	pmic_set_register_value(PMIC_FG_CUR_LTH
				, bat_oc_l_thd(POWER_BAT_OC_CURRENT_L));

	bat_oc_h_en_setting(0);
	bat_oc_l_en_setting(1);

	pr_info("FG_CUR_HTH = 0x%x, FG_CUR_LTH = 0x%x, RG_INT_EN_FG_CUR_H = %d, RG_INT_EN_FG_CUR_L = %d\n"
		, pmic_get_register_value(PMIC_FG_CUR_HTH)
		, pmic_get_register_value(PMIC_FG_CUR_LTH)
		, pmic_get_register_value(PMIC_RG_INT_EN_FG_CUR_H)
		, pmic_get_register_value(PMIC_RG_INT_EN_FG_CUR_L));

	pr_info("[%s] %d mA, %d mA Done\n", __func__,
		POWER_BAT_OC_CURRENT_H, POWER_BAT_OC_CURRENT_L);
}
#else
void __attribute__ ((weak)) register_battery_oc_notify(
	void (*battery_oc_callback)(enum BATTERY_OC_LEVEL_TAG),
	enum BATTERY_OC_PRIO_TAG prio_val)
{
}
#endif /* #ifdef BATTERY_OC_PROTECT */


/*******************************************************************
 * 15% notify service
 *******************************************************************/
#ifndef DISABLE_BATTERY_PERCENT_PROTECT
#define BATTERY_PERCENT_PROTECT
#endif

#ifdef BATTERY_PERCENT_PROTECT
static struct hrtimer bat_percent_notify_timer;
static struct task_struct *bat_percent_notify_thread;
static bool bat_percent_notify_flag;
static DECLARE_WAIT_QUEUE_HEAD(bat_percent_notify_waiter);

struct wakeup_source bat_percent_notify_lock;
static DEFINE_MUTEX(bat_percent_notify_mutex);

#define BPCB_NUM 16

int g_battery_percent_level;
int g_battery_percent_stop;

#define BAT_PERCENT_LINIT 15

struct battery_percent_callback_table {
	void (*bpcb)(enum BATTERY_PERCENT_LEVEL_TAG);
};

struct battery_percent_callback_table bpcb_tb[] = {
	{NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL},
	{NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}
};

void register_battery_percent_notify(
	void (*battery_percent_callback)(enum BATTERY_PERCENT_LEVEL_TAG),
	BATTERY_PERCENT_PRIO prio_val)
{
	PMICLOG("[%s] start\n", __func__);

	bpcb_tb[prio_val].bpcb = battery_percent_callback;

	pr_info("[%s] prio_val=%d\n", __func__, prio_val);

	if ((g_battery_percent_stop == 0) && (g_battery_percent_level == 1)) {
#ifdef DISABLE_DLPT_FEATURE
		pr_info("[%s] level l happen\n", __func__);
		if (battery_percent_callback != NULL)
			battery_percent_callback(BATTERY_PERCENT_LEVEL_1);
#else
		if (prio_val == BATTERY_PERCENT_PRIO_FLASHLIGHT) {
			pr_info("[%s at DLPT] level l happen\n", __func__);
			if (battery_percent_callback != NULL)
				battery_percent_callback(
						BATTERY_PERCENT_LEVEL_1);
		}
#endif
	}
}

void exec_battery_percent_callback(
	enum BATTERY_PERCENT_LEVEL_TAG battery_percent_level)
{				/*0:no limit */
#ifdef DISABLE_DLPT_FEATURE
	int i = 0;
#endif

	if (g_battery_percent_stop == 1) {
		pr_info("[%s] g_battery_percent_stop=%d\n"
			, __func__, g_battery_percent_stop);
	} else {
#ifdef DISABLE_DLPT_FEATURE
		for (i = 0; i < BPCB_NUM; i++) {
			if (bpcb_tb[i].bpcb != NULL) {
				bpcb_tb[i].bpcb(battery_percent_level);
				pr_info("[%s] prio_val=%d,battery_percent_level=%d\n"
					, __func__, i, battery_percent_level);
			}
		}
#else
		if (bpcb_tb[BATTERY_PERCENT_PRIO_FLASHLIGHT].bpcb != NULL) {
			bpcb_tb[BATTERY_PERCENT_PRIO_FLASHLIGHT].bpcb(
				battery_percent_level);
		} else
			pr_notice("[%s]BATTERY_PERCENT_PRIO_FLASHLIGHT is null\n"
			       , __func__);
		pr_info
			("[%s at DLPT] prio_val=%d,battery_percent_level=%d\n"
			 , __func__
			 , BATTERY_PERCENT_PRIO_FLASHLIGHT
			 , battery_percent_level);
#endif
	}
}

int bat_percent_notify_handler(void *unused)
{
	ktime_t ktime;
	int bat_per_val = 0;

	do {
		ktime = ktime_set(10, 0);

		wait_event_interruptible(bat_percent_notify_waiter,
					 (bat_percent_notify_flag == true));

		__pm_stay_awake(&bat_percent_notify_lock);
		mutex_lock(&bat_percent_notify_mutex);

#if (CONFIG_MTK_GAUGE_VERSION == 30)
		bat_per_val = battery_get_uisoc();
#endif
		if ((upmu_get_rgs_chrdet() == 0) &&
		    (g_battery_percent_level == 0) &&
		    (bat_per_val <= BAT_PERCENT_LINIT)) {
			g_battery_percent_level = 1;
			exec_battery_percent_callback(BATTERY_PERCENT_LEVEL_1);
		} else if ((g_battery_percent_level == 1) &&
			   (bat_per_val > BAT_PERCENT_LINIT)) {
			g_battery_percent_level = 0;
			exec_battery_percent_callback(BATTERY_PERCENT_LEVEL_0);
		} else {
		}
		bat_percent_notify_flag = false;

		PMICLOG("bat_per_level=%d,bat_per_val=%d\n"
			, g_battery_percent_level, bat_per_val);

		mutex_unlock(&bat_percent_notify_mutex);
		__pm_relax(&bat_percent_notify_lock);

		hrtimer_start(&bat_percent_notify_timer
			      , ktime, HRTIMER_MODE_REL);

	} while (!kthread_should_stop());

	return 0;
}

enum hrtimer_restart bat_percent_notify_task(struct hrtimer *timer)
{
	bat_percent_notify_flag = true;
	wake_up_interruptible(&bat_percent_notify_waiter);
	PMICLOG("%s is called\n", __func__);

	return HRTIMER_NORESTART;
}

void bat_percent_notify_init(void)
{
	ktime_t ktime;

	ktime = ktime_set(20, 0);
	hrtimer_init(&bat_percent_notify_timer
		     , CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	bat_percent_notify_timer.function = bat_percent_notify_task;
	hrtimer_start(&bat_percent_notify_timer, ktime, HRTIMER_MODE_REL);

	wakeup_source_init(&bat_percent_notify_lock,
		"bat_percent_notify_lock wakelock");

	bat_percent_notify_thread =
	    kthread_run(bat_percent_notify_handler, 0
			, "bat_percent_notify_thread");
	if (IS_ERR(bat_percent_notify_thread))
		pr_notice("Failed to create bat_percent_notify_thread\n");
	else
		pr_info("Create bat_percent_notify_thread : done\n");
}
#else
void __attribute__ ((weak)) register_battery_percent_notify(
	void (*battery_percent_callback)(enum BATTERY_PERCENT_LEVEL_TAG),
	BATTERY_PERCENT_PRIO prio_val)
{
}
#endif /* #ifdef BATTERY_PERCENT_PROTECT */

/*******************************************************************
 * AuxADC Impedence Measurement
 *******************************************************************/
static unsigned int count_time_out_adc_imp = 36;
static struct wakeup_source ptim_wake_lock;
static struct mutex ptim_mutex;

static void ptim_lock(void)
{
	__pm_stay_awake(&ptim_wake_lock);
	mutex_lock(&ptim_mutex);
}

static void ptim_unlock(void)
{
	mutex_unlock(&ptim_mutex);
	__pm_relax(&ptim_wake_lock);
}

int do_ptim_internal(bool isSuspend, unsigned int *bat,
		     signed int *cur, bool *is_charging)
{
	unsigned int vbat_reg;
	unsigned int count_adc_imp = 0;
	int ret = 0;

	/* selection setting */
	if (is_isense_supported() && is_power_path_supported())
		pmic_set_hk_reg_value(PMIC_AUXADC_IMPEDANCE_CHSEL, 1);
	else
		pmic_set_hk_reg_value(PMIC_AUXADC_IMPEDANCE_CHSEL, 0);
	/* enable setting */
	pmic_set_hk_reg_value(PMIC_RG_AUXADC_IMP_CK_SW_MODE, 1);
	pmic_set_hk_reg_value(PMIC_RG_AUXADC_IMP_CK_SW_EN, 1);

	/* start setting */
	/*Peter-SW:55,56*/
	pmic_set_hk_reg_value(PMIC_AUXADC_IMP_AUTORPT_EN, 1);
	/*set issue interrupt */
	/*pmic_set_hk_reg_value(PMIC_RG_INT_EN_AUXADC_IMP,1); */

	while (pmic_get_register_value(PMIC_AUXADC_IMPEDANCE_IRQ_STATUS) == 0) {
		if ((count_adc_imp++) > count_time_out_adc_imp) {
			pr_notice("do_ptim over %d times/ms\n", count_adc_imp);
			ret = 1;
			break;
		}
		mdelay(1);
	}
	vbat_reg = pmic_get_register_value(PMIC_AUXADC_ADC_OUT_IMP);

	/*clear irq */
	pmic_set_hk_reg_value(PMIC_AUXADC_CLR_IMP_CNT_STOP, 1);
	pmic_set_hk_reg_value(PMIC_AUXADC_IMPEDANCE_IRQ_CLR, 1);
	pmic_set_hk_reg_value(PMIC_AUXADC_CLR_IMP_CNT_STOP, 0);
	pmic_set_hk_reg_value(PMIC_AUXADC_IMPEDANCE_IRQ_CLR, 0);

	/* stop setting */
	/*Peter-SW:55,56*/
	pmic_set_hk_reg_value(PMIC_AUXADC_IMP_AUTORPT_EN, 0);

	/* disable setting */
	pmic_set_hk_reg_value(PMIC_RG_AUXADC_IMP_CK_SW_MODE, 0);
	pmic_set_hk_reg_value(PMIC_RG_AUXADC_IMP_CK_SW_EN, 1);

	*bat = (vbat_reg * 3 * 18000) / 32768;
#if (CONFIG_MTK_GAUGE_VERSION == 30)
	gauge_get_ptim_current(cur, is_charging);
#else
	*cur = 0;
#endif
	pr_info("%s : bat %d cur %d\n", __func__, *bat, *cur);

	return ret;
}

int do_ptim_gauge(bool isSuspend, unsigned int *bat,
		  signed int *cur, bool *is_charging)
{
	int ret;

	if (isSuspend == false)
		ptim_lock();

	ret = do_ptim_internal(isSuspend, bat, cur, is_charging);

	if (isSuspend == false)
		ptim_unlock();
	return ret;
}

/*******************************************************************
 * DLPT service
 *******************************************************************/
#ifndef DISABLE_DLPT_FEATURE
#define DLPT_FEATURE_SUPPORT
#endif

#ifdef DLPT_FEATURE_SUPPORT

static unsigned int ptim_bat_vol;
static signed int ptim_R_curr;
static int ptim_rac_val_avg;
static int g_imix_val;

static unsigned int g_dlpt_val;

int g_dlpt_stop;
int g_lbatInt1 = DLPT_VOLT_MIN * 10;

void get_ptim_value(bool isSuspend, unsigned int *bat, signed int *cur)
{
	if (isSuspend == false)
		ptim_lock();
	*bat = ptim_bat_vol;
	*cur = ptim_R_curr;
	if (isSuspend == false)
		ptim_unlock();
}

int get_rac(void)
{
	return ptim_rac_val_avg;
}

int get_imix(void)
{
	return g_imix_val;
}

int do_ptim(bool isSuspend)
{
	int ret;
	bool is_charging;

	if (isSuspend == false)
		ptim_lock();

	ret = do_ptim_internal(isSuspend,
		&ptim_bat_vol, &ptim_R_curr, &is_charging);

	if (isSuspend == false)
		ptim_unlock();
	return ret;
}

void enable_dummy_load(unsigned int en)
{
	if (en == 1) {
#if 0
		/* enable isink step */
		pmic_set_register_value(PMIC_ISINK_CH2_STEP, 0x7);
		pmic_set_register_value(PMIC_ISINK_CH3_STEP, 0x7);
#endif
		/* double function */
		/*CH3 double on  */
		pmic_set_register_value(PMIC_RG_ISINK3_DOUBLE, 0x1);
		/*CH2 double on  */
		pmic_set_register_value(PMIC_RG_ISINK2_DOUBLE, 0x1);
		/*enable isink */
		pmic_set_register_value(PMIC_ISINK_CH3_BIAS_EN, 0x1);
		pmic_set_register_value(PMIC_ISINK_CH2_BIAS_EN, 0x1);
		pmic_set_register_value(PMIC_ISINK_CH3_EN, 0x1);
		pmic_set_register_value(PMIC_ISINK_CH2_EN, 0x1);
		/*PMICLOG("[enable dummy load]\n"); */
	} else {
		pmic_set_register_value(PMIC_ISINK_CH3_EN, 0);
		pmic_set_register_value(PMIC_ISINK_CH2_EN, 0);
		pmic_set_register_value(PMIC_ISINK_CH3_BIAS_EN, 0);
		pmic_set_register_value(PMIC_ISINK_CH2_BIAS_EN, 0);
		/*PMICLOG("[disable dummy load]\n"); */
	}
}

static struct timer_list dlpt_notify_timer;
static struct task_struct *dlpt_notify_thread;
static bool dlpt_notify_flag;
static DECLARE_WAIT_QUEUE_HEAD(dlpt_notify_waiter);
struct wakeup_source dlpt_notify_lock;
static DEFINE_MUTEX(dlpt_notify_mutex);

#define DLPT_NUM 16

struct dlpt_callback_table {
	void (*dlpt_cb)(unsigned int dlpt_val);
};

struct dlpt_callback_table dlpt_cb_tb[] = {
	{NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL},
	{NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}
};

void register_dlpt_notify(
	void (*dlpt_callback)(unsigned int dlpt_val),
	enum DLPT_PRIO_TAG prio_val)
{
	PMICLOG("[%s] start\n", __func__);

	dlpt_cb_tb[prio_val].dlpt_cb = dlpt_callback;

	pr_info("[%s] prio_val=%d\n", __func__, prio_val);

	if ((g_dlpt_stop == 0) && (g_dlpt_val != 0)) {
		pr_notice("[%s] dlpt happen\n", __func__);
		if (dlpt_callback != NULL)
			dlpt_callback(g_dlpt_val);
	}
}

static void exec_dlpt_callback(unsigned int dlpt_val)
{
	int i = 0;

	g_dlpt_val = dlpt_val;

	if (g_dlpt_stop == 1) {
		pr_notice("[%s] g_dlpt_stop=%d\n", __func__, g_dlpt_stop);
	} else {
		for (i = 0; i < DLPT_NUM; i++) {
			if (dlpt_cb_tb[i].dlpt_cb != NULL) {
				dlpt_cb_tb[i].dlpt_cb(g_dlpt_val);
				PMICLOG("[%s] g_dlpt_val=%d\n",
					__func__, g_dlpt_val);
			}
		}
	}
}

#if (CONFIG_MTK_GAUGE_VERSION == 30)
static int get_rac_val(void)
{
	int volt_1 = 0;
	int volt_2 = 0;
	int curr_1 = 0;
	int curr_2 = 0;
	int rac_cal = 0;
	int ret = 0;
	bool retry_state = false;
	int retry_count = 0;

	do {
		/* Trigger ADC PTIM mode to get VBAT and current */
		do_ptim(true);
		volt_1 = ptim_bat_vol;
		curr_1 = ptim_R_curr;

		/* enable dummy load */
		enable_dummy_load(1);
		mdelay(50);
		/*Wait ----------------------------------------------- */

		/* Trigger ADC PTIM mode again to get new VBAT and current */
		do_ptim(true);
		volt_2 = ptim_bat_vol;
		curr_2 = ptim_R_curr;

		/*Disable dummy load----------------------------------- */
		enable_dummy_load(0);

		/*Calculate Rac---------------------------------------- */
		/* 70mA <= c_diff <= 120mA, 4mV <= v_diff <= 200mV */
		if ((curr_2 - curr_1) >= 700 && (curr_2 - curr_1) <= 1200
		    && (volt_1 - volt_2) >= 40 && (volt_1 - volt_2) <= 2000) {
			/*m-ohm */
			rac_cal = ((volt_1 - volt_2) * 1000) /
				(curr_2 - curr_1);

			if (rac_cal < 0)
				ret = (rac_cal - (rac_cal * 2)) * 1;
			else
				ret = rac_cal * 1;
			if (ret < 50) {
				ret = -1;
				pmic_spm_crit2("bypass due to Rac < 50mOhm\n");
			}
		} else {
			ret = -1;
			pmic_spm_crit2("[4, Cal.Rac] bypass c_diff < 70mA\n");
		}

		pmic_spm_crit2("v1=%d,v2=%d,c1=%d,c2=%d,v_diff=%d,c_diff=%d\n"
			, volt_1, volt_2, curr_1, curr_2,
			(volt_1 - volt_2), (curr_2 - curr_1));

		pmic_spm_crit2("rac_cal=%d,ret=%d,retry=%d\n"
			, rac_cal, ret, retry_count);

		/*------------------------*/
		retry_count++;

		if ((retry_count < 3) && (ret == -1))
			retry_state = true;
		else
			retry_state = false;

	} while (retry_state == true);

	return ret;
}
#endif /* end of #if CONFIG_MTK_GAUGE_VERSION == 30 */

int get_dlpt_imix_spm(void)
{
#if (CONFIG_MTK_GAUGE_VERSION == 30)
	int rac_val[5], rac_val_avg;
	int i;
	static unsigned int pre_ui_soc = 101;
	unsigned int ui_soc;

	ui_soc = battery_get_uisoc();

	if (ui_soc != pre_ui_soc)
		pre_ui_soc = ui_soc;
	else {
		pmic_spm_crit2("[dlpt_R] pre_SOC=%d SOC=%d skip\n",
			pre_ui_soc, ui_soc);
		return 0;
	}

	for (i = 0; i < 2; i++) {
		rac_val[i] = get_rac_val();
		if (rac_val[i] == -1)
			return -1;
	}

	rac_val_avg = rac_val[0] + rac_val[1];
	rac_val_avg = rac_val_avg / 2;
	pr_info("[dlpt_R] %d,%d,%d\n", rac_val[0], rac_val[1], rac_val_avg);

	if (rac_val_avg > 100)
		ptim_rac_val_avg = rac_val_avg;

#endif /* end of #if CONFIG_MTK_GAUGE_VERSION == 30 */
	return 0;

}

void pmic_swap(int *a, int *b)
{
	int temp = *a;
	*a = *b;
	*b = temp;
}

void pmic_quicksort(int *data, int left, int right)
{
	int pivot, i, j;

	if (left >= right)
		return;

	pivot = data[left];

	i = left + 1;
	j = right;

	while (1) {
		while (i <= right) {
			if (data[i] > pivot)
				break;
			i = i + 1;
		}

		while (j > left) {
			if (data[j] < pivot)
				break;
			j = j - 1;
		}

		if (i > j)
			break;

		pmic_swap(&data[i], &data[j]);
	}

	pmic_swap(&data[left], &data[j]);

	pmic_quicksort(data, left, j - 1);
	pmic_quicksort(data, j + 1, right);
}

int get_dlpt_imix(void)
{
	int volt[5], curr[5], volt_avg = 0, curr_avg = 0;
	int imix;
	int i, count_do_ptim = 0;

	for (i = 0; i < 5; i++) {
		/*adc and fg---------------------------------------------- */
		while (do_ptim(false)) {
			if ((count_do_ptim >= 2) && (count_do_ptim < 4))
				pr_notice("do_ptim more than twice times\n");
			else if (count_do_ptim > 3) {
				pr_notice("do_ptim more than five times\n");
				ptim_lock();
				pmic_set_hk_reg_value(PMIC_RG_AUXADC_RST, 1);
				pmic_set_hk_reg_value(PMIC_RG_AUXADC_RST, 0);
				ptim_unlock();
#if defined(CONFIG_MTK_SELINUX_AEE_WARNING)
				aee_kernel_warning("PTIM timeout", "PTIM");
#endif
				break;
			}
			count_do_ptim++;
		}

		volt[i] = ptim_bat_vol;
		curr[i] = ptim_R_curr;
		PMICLOG("[%s:%d] %d,%d,%d,%d\n", __func__,
			i, volt[i], curr[i], volt_avg, curr_avg);
	}

	pmic_quicksort(volt, 0, 4);
	pmic_quicksort(curr, 0, 4);
	volt_avg = volt[1] + volt[2] + volt[3];
	curr_avg = curr[1] + curr[2] + curr[3];
	volt_avg = volt_avg / 3;
	curr_avg = curr_avg / 3;
	imix = (curr_avg + (volt_avg - g_lbatInt1) * 1000 / ptim_rac_val_avg)
						/ 10;

#if (CONFIG_MTK_GAUGE_VERSION == 30)
	pr_info("[%s] %d,%d,%d,%d,%d,%d,%d\n", __func__,
		volt_avg, curr_avg, g_lbatInt1, ptim_rac_val_avg, imix,
		battery_get_soc(), battery_get_uisoc());
#else
	pr_info("[%s] %d,%d,%d,%d,%d,NA,NA\n", __func__,
		volt_avg, curr_avg, g_lbatInt1, ptim_rac_val_avg, imix);
#endif

	if (imix < 0) {
		pr_notice("[%s] imix = %d < 1\n", __func__, imix);
		return g_imix_val;
	}
	return imix;

}

static int get_dlpt_imix_charging(void)
{
	int zcv_val = 0;
	int vsys_min_1_val = DLPT_VOLT_MIN;
	int imix = 0;
	static struct iio_channel *chan;

	if (chan == NULL) {
		if (is_isense_supported() && is_power_path_supported())
			chan = iio_channel_get(NULL, "AUXADC_ISENSE");
		else
			chan = iio_channel_get(NULL, "AUXADC_BATADC");
	}
	if (IS_ERR(chan)) {
		pr_notice("[%s] iio channel consumer error, (%d, %d)\n",
			__func__, is_isense_supported(),
			is_power_path_supported());
		return 0;
	}
	iio_read_channel_processed(chan, &zcv_val);
	imix = (zcv_val - vsys_min_1_val) * 1000 / ptim_rac_val_avg * 9 / 10;
	PMICLOG("[dlpt] %s %d %d %d %d\n", __func__,
		imix, zcv_val, vsys_min_1_val, ptim_rac_val_avg);

	return imix;
}

/* for dlpt_notify_handler */
static int g_low_per_timer;
static int g_low_per_timeout_val = 60;

bool __attribute__ ((weak)) mtk_dpidle_is_active(void)
{
	return false;
}

int dlpt_notify_handler(void *unused)
{
#if (CONFIG_MTK_GAUGE_VERSION == 30)
	unsigned long dlpt_notify_interval;
	int pre_ui_soc = 0;
	int cur_ui_soc = 0;
	int diff_ui_soc = 1;
	unsigned short power_off_cnt = 0;

	pre_ui_soc = battery_get_uisoc();
	cur_ui_soc = pre_ui_soc;

	do {
		if (dpidle_active_status())
			dlpt_notify_interval = HZ * 20; /* light-loading mode */
		else
			dlpt_notify_interval = HZ * 10; /* normal mode */

		wait_event_interruptible(dlpt_notify_waiter,
			(dlpt_notify_flag == true));

		__pm_stay_awake(&dlpt_notify_lock);
		mutex_lock(&dlpt_notify_mutex);

		cur_ui_soc = battery_get_uisoc();
		if (cur_ui_soc <= 1) {
			g_low_per_timer += 10;
			if (g_low_per_timer > g_low_per_timeout_val)
				g_low_per_timer = 0;
			PMICLOG("[DLPT] g_low_per_timer=%d\n", g_low_per_timer);
			PMICLOG("[DLPT] g_low_per_timeout_val%d\n",
				g_low_per_timeout_val);
		} else
			g_low_per_timer = 0;

		PMICLOG("[%s] %d %d %d %d %d\n", __func__,
			pre_ui_soc, cur_ui_soc, g_imix_val,
			g_low_per_timer, g_low_per_timeout_val);

		PMICLOG("[DLPT] is running\n");
		if (ptim_rac_val_avg == 0)
			pr_info("[DLPT] ptim_rac_val_avg=0, skip\n");
		else {
			if (upmu_get_rgs_chrdet())
				g_imix_val = get_dlpt_imix_charging();
			else
				g_imix_val = get_dlpt_imix();

			/*Notify */
			if (g_imix_val > IMAX_MAX_VALUE)
				g_imix_val = IMAX_MAX_VALUE;
			exec_dlpt_callback(g_imix_val);
			pre_ui_soc = cur_ui_soc;

			pr_info("[DLPT_final] %d,%d,%d,%d,%d\n",
				g_imix_val, pre_ui_soc, cur_ui_soc,
				diff_ui_soc, IMAX_MAX_VALUE);
		}

		dlpt_notify_flag = false;
		/* notify battery driver to power off by SOC=0 */
#if PMIC_THROTTLING_DLPT_UT
		/* LBAT UT should not power off */
		power_off_cnt = 0;
#else
		/* notify battery driver to power off by SOC=0*/
		if (cur_ui_soc <= DLPT_POWER_OFF_THD) {
			if (dlpt_check_power_off() == 1) {
				set_shutdown_cond(DLPT_SHUTDOWN);
				power_off_cnt++;
				pr_info("[DLPT_POWER_OFF_EN] notify SOC=0 to power off, power_off_cnt=%d\n",
					power_off_cnt);
				if (power_off_cnt >= 4)
					kernel_restart(
						"DLPT reboot system");
			} else
				power_off_cnt = 0;
		}
#endif
		mutex_unlock(&dlpt_notify_mutex);
		__pm_relax(&dlpt_notify_lock);

		mod_timer(&dlpt_notify_timer, jiffies + dlpt_notify_interval);

	} while (!kthread_should_stop());
#endif /* end of #if CONFIG_MTK_GAUGE_VERSION == 30 */

	return 0;
}

void dlpt_notify_task(unsigned long data)
{
	dlpt_notify_flag = true;
	wake_up_interruptible(&dlpt_notify_waiter);
	PMICLOG("%s is called\n", __func__);
}

void dlpt_notify_init(void)
{
	unsigned long dlpt_notify_interval;

	dlpt_notify_interval = HZ * 30;
	init_timer_deferrable(&dlpt_notify_timer);
	dlpt_notify_timer.function = dlpt_notify_task;
	dlpt_notify_timer.data = (unsigned long)&dlpt_notify_timer;
	mod_timer(&dlpt_notify_timer, jiffies + dlpt_notify_interval);

	wakeup_source_init(&dlpt_notify_lock, "dlpt_notify_lock wakelock");

	dlpt_notify_thread = kthread_run(
		dlpt_notify_handler, 0, "dlpt_notify_thread");
	if (IS_ERR(dlpt_notify_thread))
		pr_notice("Failed to create dlpt_notify_thread\n");
	else
		pr_info("Create dlpt_notify_thread : done\n");
}

#else
int get_dlpt_imix_spm(void)
{
	return 1;
}

int get_rac(void)
{
	return 0;
}

int get_imix(void)
{
	return 0;
}
#endif				/*#ifdef DLPT_FEATURE_SUPPORT */

#ifdef LOW_BATTERY_PROTECT
/*****************************************************************************
 * low battery protect UT
 ******************************************************************************/
static ssize_t show_low_battery_protect_ut(
		struct device *dev, struct device_attribute *attr,
		char *buf)
{
	PMICLOG("[%s] g_low_battery_level=%d\n", __func__, g_low_battery_level);
	return sprintf(buf, "%u\n", g_low_battery_level);
}

static ssize_t store_low_battery_protect_ut(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL;
	unsigned int val = 0;
	unsigned int thd;

	pr_info("[%s]\n", __func__);

	if (buf != NULL && size != 0) {
		pr_info("[%s] buf is %s and size is %zu\n",
			__func__, buf, size);
		pvalue = (char *)buf;
		ret = kstrtou32(pvalue, 16, (unsigned int *)&val);
		if (val <= 2) {
			if (val == LOW_BATTERY_LEVEL_0)
				thd = POWER_INT0_VOLT;
			else if (val == LOW_BATTERY_LEVEL_1)
				thd = POWER_INT1_VOLT;
			else if (val == LOW_BATTERY_LEVEL_2)
				thd = POWER_INT2_VOLT;
			pr_info("[%s] your input is %d(%d)\n",
				__func__, val, thd);
			exec_low_battery_callback(thd);
		} else {
			pr_info("[%s] wrong number (%d)\n", __func__, val);
		}
	}
	return size;
}

static DEVICE_ATTR(low_battery_protect_ut, 0664
		, show_low_battery_protect_ut, store_low_battery_protect_ut);

/*****************************************************************************
 * low battery protect stop
 ******************************************************************************/
static ssize_t show_low_battery_protect_stop(
		struct device *dev, struct device_attribute *attr,
		char *buf)
{
	PMICLOG("[%s] g_low_battery_stop=%d\n", __func__, g_low_battery_stop);
	return sprintf(buf, "%u\n", g_low_battery_stop);
}

static ssize_t store_low_battery_protect_stop(struct device *dev
		, struct device_attribute *attr
		, const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL;
	unsigned int val = 0;

	pr_info("[%s]\n", __func__);

	if (buf != NULL && size != 0) {
		pr_info("[%s] buf is %s and size is %zu\n",
			__func__, buf, size);
		pvalue = (char *)buf;
		ret = kstrtou32(pvalue, 16, (unsigned int *)&val);
		if ((val != 0) && (val != 1))
			val = 0;
		g_low_battery_stop = val;
		pr_info("[%s] g_low_battery_stop=%d\n",
			__func__, g_low_battery_stop);
	}
	return size;
}

static DEVICE_ATTR(low_battery_protect_stop, 0664,
	show_low_battery_protect_stop, store_low_battery_protect_stop);

/*
 * low battery protect level
 */
static ssize_t show_low_battery_protect_level(struct device *dev
				, struct device_attribute *attr, char *buf)
{
	PMICLOG("[%s] g_low_battery_level=%d\n",
		__func__, g_low_battery_level);
	return sprintf(buf, "%u\n", g_low_battery_level);
}

static ssize_t store_low_battery_protect_level(struct device *dev
				, struct device_attribute *attr
				, const char *buf, size_t size)
{
	PMICLOG("[%s] g_low_battery_level=%d\n",
		__func__, g_low_battery_level);
	return size;
}

static DEVICE_ATTR(low_battery_protect_level, 0664,
	show_low_battery_protect_level, store_low_battery_protect_level);
#endif /* end of #ifdef LOW_BATTERY_PROTECT */

#ifdef BATTERY_OC_PROTECT
/*****************************************************************************
 * battery OC protect UT
 ******************************************************************************/
static ssize_t show_battery_oc_protect_ut(
		struct device *dev, struct device_attribute *attr,
		char *buf)
{
	PMICLOG("[%s] g_battery_oc_level=%d\n",
		__func__, g_battery_oc_level);
	return sprintf(buf, "%u\n", g_battery_oc_level);
}

static ssize_t store_battery_oc_protect_ut(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL;
	unsigned int val = 0;

	pr_info("[%s]\n", __func__);

	if (buf != NULL && size != 0) {
		pr_info("[%s] buf is %s and size is %zu\n",
			__func__, buf, size);
		pvalue = (char *)buf;
		ret = kstrtou32(pvalue, 16, (unsigned int *)&val);
		if (val <= 1) {
			pr_info("[%s] your input is %d\n", __func__, val);
			exec_battery_oc_callback(val);
		} else {
			pr_info("[%s] wrong number (%d)\n", __func__, val);
		}
	}
	return size;
}

static DEVICE_ATTR(battery_oc_protect_ut, 0664,
	show_battery_oc_protect_ut, store_battery_oc_protect_ut);

/*****************************************************************************
 * battery OC protect stop
 ******************************************************************************/
static ssize_t show_battery_oc_protect_stop(
		struct device *dev, struct device_attribute *attr,
		char *buf)
{
	PMICLOG("[%s] g_battery_oc_stop=%d\n"
		, __func__, g_battery_oc_stop);
	return sprintf(buf, "%u\n", g_battery_oc_stop);
}

static ssize_t store_battery_oc_protect_stop(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL;
	unsigned int val = 0;

	pr_info("[_protect_stop]\n");

	if (buf != NULL && size != 0) {
		pr_info("[%s] buf is %s and size is %zu\n"
			, __func__, buf, size);
		pvalue = (char *)buf;
		ret = kstrtou32(pvalue, 16, (unsigned int *)&val);
		if ((val != 0) && (val != 1))
			val = 0;
		g_battery_oc_stop = val;
		pr_info("[%s] g_battery_oc_stop=%d\n"
			, __func__, g_battery_oc_stop);
	}
	return size;
}

static DEVICE_ATTR(battery_oc_protect_stop, 0664,
	show_battery_oc_protect_stop, store_battery_oc_protect_stop);

/*****************************************************************************
 * battery OC protect level
 ******************************************************************************/
static ssize_t show_battery_oc_protect_level(
		struct device *dev, struct device_attribute *attr,
		char *buf)
{
	PMICLOG("[%s] g_battery_oc_level=%d\n",
		__func__, g_battery_oc_level);
	return sprintf(buf, "%u\n", g_battery_oc_level);
}

static ssize_t store_battery_oc_protect_level(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	PMICLOG("[%s] g_battery_oc_level=%d\n",
		__func__, g_battery_oc_level);
	return size;
}

static DEVICE_ATTR(battery_oc_protect_level, 0664,
	show_battery_oc_protect_level, store_battery_oc_protect_level);

/*****************************************************************************
 * battery OC protect threshold
 ******************************************************************************/
static ssize_t show_battery_oc_protect_thd(
		struct device *dev, struct device_attribute *attr,
		char *buf)
{
	PMICLOG("[%s] g_battery_oc_level = %d\n",
		__func__, g_battery_oc_level);
	return snprintf(buf, PAGE_SIZE,
		"[%s] g_battery_oc_l_thd = %x(%d), g_battery_oc_h_thd = %x(%d)\n"
		, __func__
		, bat_oc_l_thd(g_battery_oc_l_thd), g_battery_oc_l_thd
		, bat_oc_h_thd(g_battery_oc_h_thd), g_battery_oc_h_thd);
}

static ssize_t store_battery_oc_protect_thd(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int battery_oc_l_thd, battery_oc_h_thd;
	int num = sscanf(buf, "%d %d", &battery_oc_l_thd, &battery_oc_h_thd);

	if ((num != 2) || (battery_oc_l_thd >= battery_oc_h_thd))
		pr_notice("Invalid parameter : %s\n", buf);
	else {
		g_battery_oc_l_thd = battery_oc_l_thd;
		g_battery_oc_h_thd = battery_oc_h_thd;
		pr_info("[%s] g_battery_oc_l_thd = %x(%d), g_battery_oc_h_thd = %x(%d)\n"
			, __func__,
			bat_oc_l_thd(g_battery_oc_l_thd), g_battery_oc_l_thd,
			bat_oc_h_thd(g_battery_oc_h_thd), g_battery_oc_h_thd);
		pmic_set_register_value(PMIC_FG_CUR_HTH,
			bat_oc_h_thd(g_battery_oc_h_thd));
		pmic_set_register_value(PMIC_FG_CUR_LTH,
			bat_oc_l_thd(g_battery_oc_l_thd));
	}

	return size;
}

static DEVICE_ATTR(battery_oc_protect_thd, 0664,
	show_battery_oc_protect_thd, store_battery_oc_protect_thd);
#endif /* end of #ifdef BATTERY_OC_PROTECT */

#ifdef BATTERY_PERCENT_PROTECT
/*****************************************************************************
 * battery percent protect UT
 ******************************************************************************/
static ssize_t show_battery_percent_ut(
		struct device *dev, struct device_attribute *attr, char *buf)
{
	/*show_battery_percent_protect_ut */
	PMICLOG("[%s] g_battery_percent_level=%d\n"
		, __func__, g_battery_percent_level);
	return sprintf(buf, "%u\n", g_battery_percent_level);
}

static ssize_t store_battery_percent_ut(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL;
	unsigned int val = 0;
	/*store_battery_percent_protect_ut */
	pr_info("[%s]\n", __func__);

	if (buf != NULL && size != 0) {
		pr_info("[%s] buf is %s and size is %zu\n"
			, __func__, buf, size);
		pvalue = (char *)buf;
		ret = kstrtou32(pvalue, 16, (unsigned int *)&val);
		if (val <= 1) {
			pr_info("[%s] your input is %d\n", __func__, val);
			exec_battery_percent_callback(val);
		} else {
			pr_info("[%s] wrong number (%d)\n", __func__, val);
		}
	}
	return size;
}

static DEVICE_ATTR(battery_percent_protect_ut, 0664,
	show_battery_percent_ut, store_battery_percent_ut);

/*****************************************************************************
 * battery percent protect stop
 ******************************************************************************/
static ssize_t show_battery_percent_stop(
		struct device *dev, struct device_attribute *attr,
		char *buf)
{
	/*show_battery_percent_protect_stop */
	PMICLOG("[%s] g_battery_percent_stop=%d\n"
		, __func__, g_battery_percent_stop);
	return sprintf(buf, "%u\n", g_battery_percent_stop);
}

static ssize_t store_battery_percent_stop(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL;
	unsigned int val = 0;
	/*store_battery_percent_protect_stop */
	pr_info("[%s]\n", __func__);

	if (buf != NULL && size != 0) {
		pr_info("[%s] buf is %s and size is %zu\n"
			, __func__, buf, size);
		pvalue = (char *)buf;
		ret = kstrtou32(pvalue, 16, (unsigned int *)&val);
		if ((val != 0) && (val != 1))
			val = 0;
		g_battery_percent_stop = val;
		pr_info("[%s] g_battery_percent_stop=%d\n"
			, __func__, g_battery_percent_stop);
	}
	return size;
}

static DEVICE_ATTR(battery_percent_protect_stop, 0664,
	show_battery_percent_stop, store_battery_percent_stop);

/*
 * battery percent protect level
 ******************************************************************************/
static ssize_t show_battery_percent_level(
		struct device *dev, struct device_attribute *attr, char *buf)
{
	/*show_battery_percent_protect_level */
	PMICLOG("[%s] g_battery_percent_level=%d\n"
			, __func__, g_battery_percent_level);
	return sprintf(buf, "%u\n", g_battery_percent_level);
}

static ssize_t store_battery_percent_level(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t size)
{
	/*store_battery_percent_protect_level */
	PMICLOG("[%s] g_battery_percent_level=%d\n"
		, __func__, g_battery_percent_level);

	return size;
}

static DEVICE_ATTR(battery_percent_protect_level, 0664,
	show_battery_percent_level, store_battery_percent_level);
#endif /*end of #ifdef BATTERY_PERCENT_PROTECT */

#ifdef DLPT_FEATURE_SUPPORT
/*****************************************************************************
 * DLPT UT
 ******************************************************************************/
static ssize_t show_dlpt_ut(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	PMICLOG("[%s] g_dlpt_val=%d\n", __func__, g_dlpt_val);
	return sprintf(buf, "%u\n", g_dlpt_val);
}

static ssize_t store_dlpt_ut(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t size)
{
	char *pvalue = NULL;
	unsigned int val = 0;
	int ret = 0;

	pr_info("[%s]\n", __func__);

	if (buf != NULL && size != 0) {
		pr_info("[%s] buf is %s and size is %zu\n",
			__func__, buf, size);
		pvalue = (char *)buf;
		ret = kstrtou32(pvalue, 10, (unsigned int *)&val);

		pr_info("[%s] your input is %d\n", __func__, val);
		exec_dlpt_callback(val);
	}
	return size;
}

static DEVICE_ATTR(dlpt_ut, 0664, show_dlpt_ut, store_dlpt_ut);

/*****************************************************************************
 * DLPT stop
 ******************************************************************************/
static ssize_t show_dlpt_stop(
		struct device *dev, struct device_attribute *attr, char *buf)
{
	PMICLOG("[%s] g_dlpt_stop=%d\n", __func__, g_dlpt_stop);
	return sprintf(buf, "%u\n", g_dlpt_stop);
}

static ssize_t store_dlpt_stop(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	char *pvalue = NULL;
	unsigned int val = 0;
	int ret = 0;

	pr_info("[%s]\n", __func__);

	if (buf != NULL && size != 0) {
		pr_info("[%s] buf is %s and size is %zu\n",
			__func__, buf, size);
		pvalue = (char *)buf;
		ret = kstrtou32(pvalue, 16, (unsigned int *)&val);
		if ((val != 0) && (val != 1))
			val = 0;
		g_dlpt_stop = val;
		pr_info("[%s] g_dlpt_stop=%d\n", __func__, g_dlpt_stop);
	}
	return size;
}

static DEVICE_ATTR(dlpt_stop, 0664, show_dlpt_stop, store_dlpt_stop);

/*****************************************************************************
 * DLPT level
 ******************************************************************************/
static ssize_t show_dlpt_level(
		struct device *dev, struct device_attribute *attr,
		char *buf)
{
	PMICLOG("[%s] g_dlpt_val=%d\n", __func__, g_dlpt_val);
	return sprintf(buf, "%u\n", g_dlpt_val);
}

static ssize_t store_dlpt_level(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	PMICLOG("[%s] g_dlpt_val=%d\n", __func__, g_dlpt_val);

	return size;
}

static DEVICE_ATTR(dlpt_level, 0664, show_dlpt_level, store_dlpt_level);
#endif /*end of #ifdef DLPT_FEATURE_SUPPORT */

/*****************************************************************************
 * system function
 ******************************************************************************/
#ifdef DLPT_FEATURE_SUPPORT
static unsigned long pmic_node;

static int fb_early_init_dt_get_chosen(
		unsigned long node, const char *uname, int depth, void *data)
{
	if (depth != 1 ||
	   (strcmp(uname, "chosen") != 0 && strcmp(uname, "chosen@0") != 0))
		return 0;
	pmic_node = node;
	return 1;
}
#endif /*end of #ifdef DLPT_FEATURE_SUPPORT */

void pmic_throttling_dlpt_suspend(void)
{
	lbat_suspend();
#ifdef BATTERY_OC_PROTECT
	bat_oc_h_en_setting(0);
	bat_oc_l_en_setting(0);

	PMICLOG("Reg[0x%x]=0x%x, Reg[0x%x]=0x%x, Reg[0x%x]=0x%x\n",
		PMIC_FG_CUR_HTH_ADDR, upmu_get_reg_value(PMIC_FG_CUR_HTH_ADDR),
		PMIC_FG_CUR_LTH_ADDR, upmu_get_reg_value(PMIC_FG_CUR_LTH_ADDR),
		PMIC_RG_INT_EN_FG_BAT0_H_ADDR,
		upmu_get_reg_value(PMIC_RG_INT_EN_FG_BAT0_H_ADDR));
#endif
}

void pmic_throttling_dlpt_resume(void)
{
	lbat_resume();
#ifdef BATTERY_OC_PROTECT
	bat_oc_h_en_setting(0);
	bat_oc_l_en_setting(0);
	mdelay(1);

	if (g_battery_oc_level == 1)
		bat_oc_h_en_setting(1);
	else
		bat_oc_l_en_setting(1);

	PMICLOG("Reg[0x%x]=0x%x, Reg[0x%x]=0x%x, Reg[0x%x]=0x%x\n"
		, PMIC_FG_CUR_HTH_ADDR
		, upmu_get_reg_value(PMIC_FG_CUR_HTH_ADDR)
		, PMIC_FG_CUR_LTH_ADDR
		, upmu_get_reg_value(PMIC_FG_CUR_LTH_ADDR)
		, PMIC_RG_INT_EN_FG_BAT0_H_ADDR
		, upmu_get_reg_value(PMIC_RG_INT_EN_FG_BAT0_H_ADDR));
#endif
}

void pmic_throttling_dlpt_debug_init(struct platform_device *dev
				     , struct dentry *debug_dir)
{
	int ret = 0;

#ifdef LOW_BATTERY_PROTECT
	ret = device_create_file(&(dev->dev)
				, &dev_attr_low_battery_protect_ut);
	ret = device_create_file(&(dev->dev)
				, &dev_attr_low_battery_protect_stop);
	ret = device_create_file(&(dev->dev)
				, &dev_attr_low_battery_protect_level);
#endif

#ifdef BATTERY_OC_PROTECT
	ret = device_create_file(&(dev->dev)
				, &dev_attr_battery_oc_protect_ut);
	ret = device_create_file(&(dev->dev)
				, &dev_attr_battery_oc_protect_stop);
	ret = device_create_file(&(dev->dev)
				, &dev_attr_battery_oc_protect_level);
	ret = device_create_file(&(dev->dev)
				, &dev_attr_battery_oc_protect_thd);
#endif

#ifdef BATTERY_PERCENT_PROTECT
	ret = device_create_file(&(dev->dev)
				, &dev_attr_battery_percent_protect_ut);
	ret = device_create_file(&(dev->dev)
				, &dev_attr_battery_percent_protect_stop);
	ret = device_create_file(&(dev->dev)
				, &dev_attr_battery_percent_protect_level);
#endif

#ifdef DLPT_FEATURE_SUPPORT
	ret = device_create_file(&(dev->dev), &dev_attr_dlpt_ut);
	ret = device_create_file(&(dev->dev), &dev_attr_dlpt_stop);
	ret = device_create_file(&(dev->dev), &dev_attr_dlpt_level);
#endif

	if (ret)
		pr_notice("[%s] error ret=%d\n", __func__, ret);
}

static void pmic_uvlo_init(void)
{
	pmic_set_register_value(PMIC_RG_UVLO_VTHL, 0);

	/*re-init UVLO volt */
	switch (POWER_UVLO_VOLT_LEVEL) {
	case 2500:
		pmic_set_register_value(PMIC_RG_UVLO_VTHL, 0);
		break;
	case 2550:
		pmic_set_register_value(PMIC_RG_UVLO_VTHL, 1);
		break;
	case 2600:
		pmic_set_register_value(PMIC_RG_UVLO_VTHL, 2);
		break;
	case 2650:
		pmic_set_register_value(PMIC_RG_UVLO_VTHL, 3);
		break;
	case 2700:
		pmic_set_register_value(PMIC_RG_UVLO_VTHL, 4);
		break;
	case 2750:
		pmic_set_register_value(PMIC_RG_UVLO_VTHL, 5);
		break;
	case 2800:
		pmic_set_register_value(PMIC_RG_UVLO_VTHL, 6);
		break;
	case 2850:
		pmic_set_register_value(PMIC_RG_UVLO_VTHL, 7);
		break;
	case 2900:
		pmic_set_register_value(PMIC_RG_UVLO_VTHL, 8);
		break;
	default:
		pr_notice("Invalid value(%d)\n", POWER_UVLO_VOLT_LEVEL);
		break;
	}
	pr_info("POWER_UVLO_VOLT_LEVEL = %d, RG_UVLO_VTHL = 0x%x\n"
		, POWER_UVLO_VOLT_LEVEL
		, pmic_get_register_value(PMIC_RG_UVLO_VTHL));
}

int pmic_throttling_dlpt_init(void)
{
#ifdef CONFIG_MTK_GAUGE_VERSION
	struct device_node *np;
	u32 val;
	char *path;

#if (CONFIG_MTK_GAUGE_VERSION == 30)
	path = "/bat_gm30";
	np = of_find_node_by_path(path);
	if (of_property_read_u32(np, "CAR_TUNE_VALUE", &val) == 0) {
		fg_cust_data.car_tune_value = (int)val*10;
		pr_info("Get car_tune_value from DT: %d\n"
			, fg_cust_data.car_tune_value);
	} else {
		fg_cust_data.car_tune_value = CAR_TUNE_VALUE*10;
		pr_info("Get default car_tune_value= %d\n"
			, fg_cust_data.car_tune_value);
	}
	if (of_property_read_u32(np, "R_FG_VALUE", &val) == 0) {
		fg_cust_data.r_fg_value = (int)val*10;
		pr_info("Get r_fg_value from DT: %d\n"
			, fg_cust_data.r_fg_value);
	} else {
		fg_cust_data.r_fg_value = R_FG_VALUE*10;
		pr_info("Get default r_fg_value= %d\n"
			, fg_cust_data.r_fg_value);
	}
	pr_info("Get default UNIT_FGCURRENT= %d\n", UNIT_FGCURRENT);
#elif (CONFIG_MTK_GAUGE_VERSION == 20)
	path = "/bus/BAT_METTER";
	np = of_find_node_by_path(path);
	if (of_property_read_u32(np, "car_tune_value", &val) == 0) {
		batt_meter_cust_data.car_tune_value = (int)val;
		PMICLOG("Get car_tune_value from DT: %d\n"
			, batt_meter_cust_data.car_tune_value);
	} else {
		batt_meter_cust_data.car_tune_value = CAR_TUNE_VALUE;
		PMICLOG("Get car_tune_value from cust header\n");
	}
#endif /* end of #if CONFIG_MTK_GAUGE_VERSION == 30 */
#endif /* end of #ifdef CONFIG_MTK_GAUGE_VERSION */

	wakeup_source_init(&ptim_wake_lock, "PTIM_wakelock");
	mutex_init(&ptim_mutex);
	/* IMPEDANCE initial setting */
	pmic_set_hk_reg_value(PMIC_AUXADC_IMPEDANCE_CNT, 1);
	pmic_set_hk_reg_value(PMIC_AUXADC_IMPEDANCE_MODE, 1);
	pmic_set_hk_reg_value(PMIC_AUXADC_IMP_AUTORPT_PRD, 6);

	/* no need to depend on LOW_BATTERY_PROTECT */
	lbat_service_init();
#ifdef LOW_BATTERY_PROTECT
	low_battery_protect_init();
#else
	pr_info("[PMIC] no define LOW_BATTERY_PROTECT\n");
#endif

#ifdef BATTERY_OC_PROTECT
	battery_oc_protect_init();
#else
	pr_info("[PMIC] no define BATTERY_OC_PROTECT\n");
#endif

#ifdef BATTERY_PERCENT_PROTECT
	bat_percent_notify_init();
#else
	pr_info("[PMIC] no define BATTERY_PERCENT_PROTECT\n");
#endif

#ifdef DLPT_FEATURE_SUPPORT
	dlpt_notify_init();
#else
	pr_info("[PMIC] no define DLPT_FEATURE_SUPPORT\n");
#endif
	pmic_uvlo_init();

#if PMIC_THROTTLING_DLPT_UT
	register_low_battery_notify(&low_bat_test, LOW_BATTERY_PRIO_CPU_B);
	PMICLOG("register_low_battery_notify:done\n");

	register_battery_oc_notify(&bat_oc_test, BATTERY_OC_PRIO_CPU_B);
	PMICLOG("register_battery_oc_notify:done\n");

	register_battery_percent_notify(&bat_per_test
					, BATTERY_PERCENT_PRIO_CPU_B);
	PMICLOG("register_battery_percent_notify:done\n");
#endif
	return 0;
}

static int __init pmic_throttling_dlpt_rac_init(void)
{
#ifdef DLPT_FEATURE_SUPPORT
	const int *pimix = NULL;
	int len = 0;

	if (of_scan_flat_dt(fb_early_init_dt_get_chosen, NULL) > 0)
		pimix = of_get_flat_dt_prop(pmic_node, "atag,imix_r", &len);
	if (pimix == NULL) {
		pr_notice(" pimix == NULL len = %d\n", len);
	} else {
		pr_info(" pimix = %d\n", *pimix);
		ptim_rac_val_avg = *pimix;
	}

	PMICLOG("******** MT pmic driver probe!! ********%d\n"
		, ptim_rac_val_avg);
#endif /* #ifdef DLPT_FEATURE_SUPPORT */
	return 0;
}

fs_initcall(pmic_throttling_dlpt_rac_init);

MODULE_AUTHOR("Argus Lin");
MODULE_DESCRIPTION("MT PMIC Device Driver");
MODULE_LICENSE("GPL");

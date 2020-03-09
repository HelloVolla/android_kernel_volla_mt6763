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

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"

/* define device tree */
/* TODO: modify temp device tree name */
#ifndef WD3100D_DTNAME
#define WD3100D_DTNAME "mediatek,flashlights_wd3100d_gpio"
#endif

/* TODO: define driver name */
#define WD3100D_NAME "flashlights-wd3100d-gpio"

/* define registers */
/* TODO: define register */

/* define mutex and work queue */
static DEFINE_MUTEX(wd3100d_mutex);
static struct work_struct wd3100d_work;

typedef enum {
	HWEN_PIN = 0,
	FLASH_PIN,
}DEV_PIN;
/* define pinctrl */
/* TODO: define pinctrl */
#define WD3100D_PINCTRL_PIN_XXX 0
#define WD3100D_PINCTRL_PINSTATE_LOW 0
#define WD3100D_PINCTRL_PINSTATE_HIGH 1
static struct pinctrl *wd3100d_pinctrl;
static struct pinctrl_state *wd3100d_hwen_high;
static struct pinctrl_state *wd3100d_hwen_low;

/* define usage count */
static int use_count;

/* platform data */
struct wd3100d_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int wd3100d_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	/* get pinctrl */
	wd3100d_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(wd3100d_pinctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(wd3100d_pinctrl);
	}

	/* TODO: Flashlight XXX pin initialization */
	wd3100d_hwen_high = pinctrl_lookup_state(wd3100d_pinctrl, "hwen_high");
	if (IS_ERR(wd3100d_hwen_high)) {
		pr_err("Failed to init (hwen_high)\n");
		ret = PTR_ERR(wd3100d_hwen_high);
	}
	wd3100d_hwen_low = pinctrl_lookup_state(wd3100d_pinctrl, "hwen_low");
	if (IS_ERR(wd3100d_hwen_low)) {
		pr_err("Failed to init (hwen_low)\n");
		ret = PTR_ERR(wd3100d_hwen_low);
	}

	return ret;
}

static int wd3100d_pinctrl_set(DEV_PIN pin, int state)
{
	int ret = 0;

	if (IS_ERR(wd3100d_pinctrl)) {
		pr_err("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case HWEN_PIN:
		if (state == WD3100D_PINCTRL_PINSTATE_LOW && !IS_ERR(wd3100d_hwen_low))
			pinctrl_select_state(wd3100d_pinctrl, wd3100d_hwen_low);
		else if (state == WD3100D_PINCTRL_PINSTATE_HIGH && !IS_ERR(wd3100d_hwen_high))
			pinctrl_select_state(wd3100d_pinctrl, wd3100d_hwen_high);
		else
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	case FLASH_PIN:
		pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	pr_debug("pin(%d) state(%d)\n", pin, state);

	return ret;
}


/******************************************************************************
 * wd3100d operations
 *****************************************************************************/
/* flashlight enable function */
static int wd3100d_enable(void)
{
	//int state = 1;
	printk("FLASHLIGHT %s\n",__func__);
	/* TODO: wrap enable function */

	return wd3100d_pinctrl_set(HWEN_PIN, 1);
}

/* flashlight disable function */
static int wd3100d_disable(void)
{

	/* TODO: wrap disable function */

	return wd3100d_pinctrl_set(HWEN_PIN, 0);;
}

/* set flashlight level */
static int wd3100d_set_level(int level)
{
	//int state = 0;

	/* TODO: wrap set level function */
	printk("%s flashlight duty = %d\n",__func__,level);

	return 0;//wd3100d_pinctrl_set(HWEN_PIN, state);
}

/* flashlight init */
static int wd3100d_init(void)
{

	/* TODO: wrap init function */

	
	return wd3100d_pinctrl_set(HWEN_PIN, 0);
}

/* flashlight uninit */
static int wd3100d_uninit(void)
{

	/* TODO: wrap uninit function */
	
	return wd3100d_pinctrl_set(HWEN_PIN, 0);
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer wd3100d_timer;
static unsigned int wd3100d_timeout_ms;

static void wd3100d_work_disable(struct work_struct *data)
{
	pr_debug("work queue callback\n");
	wd3100d_disable();
}

static enum hrtimer_restart wd3100d_timer_func(struct hrtimer *timer)
{
	schedule_work(&wd3100d_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int wd3100d_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_debug("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		wd3100d_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		wd3100d_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_debug("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (wd3100d_timeout_ms) {
				ktime = ktime_set(wd3100d_timeout_ms / 1000,
						(wd3100d_timeout_ms % 1000) * 1000000);
				hrtimer_start(&wd3100d_timer, ktime, HRTIMER_MODE_REL);
			}
			wd3100d_enable();
		} else {
			wd3100d_disable();
			hrtimer_cancel(&wd3100d_timer);
		}
		break;
	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int wd3100d_open(void)
{
	/* Actual behavior move to set driver function since power saving issue */
	return 0;
}

static int wd3100d_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int wd3100d_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&wd3100d_mutex);
	if (set) {
		if (!use_count)
			ret = wd3100d_init();
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = wd3100d_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&wd3100d_mutex);

	return ret;
}

static ssize_t wd3100d_strobe_store(struct flashlight_arg arg)
{
	wd3100d_set_driver(1);
	wd3100d_set_level(arg.level);
	wd3100d_timeout_ms = 0;
	wd3100d_enable();
	msleep(arg.dur);
	wd3100d_disable();
	wd3100d_set_driver(0);

	return 0;
}

static struct flashlight_operations wd3100d_ops = {
	wd3100d_open,
	wd3100d_release,
	wd3100d_ioctl,
	wd3100d_strobe_store,
	wd3100d_set_driver
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int wd3100d_chip_init(void)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * wd3100d_init();
	 */

	return 0;
}

static int wd3100d_parse_dt(struct device *dev,
		struct wd3100d_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		pr_info("Parse no dt, node.\n");
		return 0;
	}
	pr_info("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		pr_info("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num * sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE, WD3100D_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel, pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int wd3100d_probe(struct platform_device *pdev)
{
	struct wd3100d_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int err;
	int i;

	pr_debug("Probe start.\n");

	/* init pinctrl */
	if (wd3100d_pinctrl_init(pdev)) {
		pr_debug("Failed to init pinctrl.\n");
		err = -EFAULT;
		goto err;
	}

	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err;
		}
		pdev->dev.platform_data = pdata;
		err = wd3100d_parse_dt(&pdev->dev, pdata);
		if (err)
		goto err;
	}

	/* init work queue */
	INIT_WORK(&wd3100d_work, wd3100d_work_disable);

	/* init timer */
	hrtimer_init(&wd3100d_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	wd3100d_timer.function = wd3100d_timer_func;
	wd3100d_timeout_ms = 100;

	/* init chip hw */
	wd3100d_chip_init();

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(&pdata->dev_id[i], &wd3100d_ops)) {
				err = -EFAULT;
				goto err;
			}
	} else {
		if (flashlight_dev_register(WD3100D_NAME, &wd3100d_ops)) {
			err = -EFAULT;
			goto err;
		}
	}

	pr_debug("Probe done.\n");

	return 0;
err:
	return err;
}

static int wd3100d_remove(struct platform_device *pdev)
{
	struct wd3100d_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	pr_debug("Remove start.\n");

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(WD3100D_NAME);

	/* flush work queue */
	flush_work(&wd3100d_work);

	pr_debug("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id wd3100d_gpio_of_match[] = {
	{.compatible = WD3100D_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, wd3100d_gpio_of_match);
#else
static struct platform_device wd3100d_gpio_platform_device[] = {
	{
		.name = WD3100D_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, wd3100d_gpio_platform_device);
#endif

static struct platform_driver wd3100d_platform_driver = {
	.probe = wd3100d_probe,
	.remove = wd3100d_remove,
	.driver = {
		.name = WD3100D_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = wd3100d_gpio_of_match,
#endif
	},
};

static int __init flashlight_wd3100d_init(void)
{
	int ret;

	pr_debug("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&wd3100d_gpio_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&wd3100d_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_wd3100d_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&wd3100d_platform_driver);

	pr_debug("Exit done.\n");
}

module_init(flashlight_wd3100d_init);
module_exit(flashlight_wd3100d_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight WD3100D GPIO Driver");


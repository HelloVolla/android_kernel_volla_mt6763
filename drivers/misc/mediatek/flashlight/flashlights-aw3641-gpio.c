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
#ifndef AW3641_DTNAME
#define AW3641_DTNAME "mediatek,flashlights_aw3641_gpio"
#endif

/* TODO: define driver name */
#define AW3641_NAME "flashlights-aw3641-gpio"

//prize lsw add for only two duty mode to adapt prize flash mode start
#define PPRIZE_TWO_DUTY_MODE
//prize lsw add for only two duty mode to adapt prize flash mode end

/* define registers */
/* TODO: define register */

/* define mutex and work queue */
static DEFINE_MUTEX(aw3641_mutex);
static struct work_struct aw3641_work;

typedef enum {
	HWEN_PIN = 0,
	FLASH_PIN,
}DEV_PIN;
/* define pinctrl */
/* TODO: define pinctrl */
#define AW3641_PINCTRL_PIN_XXX 0
#define AW3641_PINCTRL_PINSTATE_LOW 0
#define AW3641_PINCTRL_PINSTATE_HIGH 1
#define AW3641_PINCTRL_STATE_XXX_HIGH "xxx_high"
#define AW3641_PINCTRL_STATE_XXX_LOW  "xxx_low"
static struct pinctrl *aw3641_pinctrl;
static struct pinctrl_state *aw3641_hwen_high;
static struct pinctrl_state *aw3641_hwen_low;
static struct pinctrl_state *aw3641_flash_high;
static struct pinctrl_state *aw3641_flash_low;

/* define usage count */
static int use_count;

/* platform data */
struct aw3641_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int aw3641_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	/* get pinctrl */
	aw3641_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(aw3641_pinctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(aw3641_pinctrl);
	}

	/* TODO: Flashlight XXX pin initialization */
	aw3641_hwen_high = pinctrl_lookup_state(aw3641_pinctrl, "hwen_high");
	if (IS_ERR(aw3641_hwen_high)) {
		pr_err("Failed to init (hwen_high)\n");
		ret = PTR_ERR(aw3641_hwen_high);
	}
	aw3641_hwen_low = pinctrl_lookup_state(aw3641_pinctrl, "hwen_low");
	if (IS_ERR(aw3641_hwen_low)) {
		pr_err("Failed to init (hwen_low)\n");
		ret = PTR_ERR(aw3641_hwen_low);
	}
	
	aw3641_flash_high = pinctrl_lookup_state(aw3641_pinctrl, "flash_high");
	if (IS_ERR(aw3641_flash_high)) {
		pr_err("Failed to init (flash_high)\n");
		ret = PTR_ERR(aw3641_flash_high);
	}
	aw3641_flash_low = pinctrl_lookup_state(aw3641_pinctrl, "flash_low");
	if (IS_ERR(aw3641_flash_low)) {
		pr_err("Failed to init (flash_low)\n");
		ret = PTR_ERR(aw3641_flash_low);
	}

	return ret;
}

static int aw3641_pinctrl_set(DEV_PIN pin, int state)
{
	int ret = 0;

	if (IS_ERR(aw3641_pinctrl)) {
		pr_err("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case HWEN_PIN:
		if (state == AW3641_PINCTRL_PINSTATE_LOW && !IS_ERR(aw3641_hwen_low))
			pinctrl_select_state(aw3641_pinctrl, aw3641_hwen_low);
		else if (state == AW3641_PINCTRL_PINSTATE_HIGH && !IS_ERR(aw3641_hwen_high))
			pinctrl_select_state(aw3641_pinctrl, aw3641_hwen_high);
		else
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	case FLASH_PIN:
		if (state == AW3641_PINCTRL_PINSTATE_LOW && !IS_ERR(aw3641_flash_low))
			pinctrl_select_state(aw3641_pinctrl, aw3641_flash_low);
		else if (state == AW3641_PINCTRL_PINSTATE_HIGH && !IS_ERR(aw3641_flash_high))
			pinctrl_select_state(aw3641_pinctrl, aw3641_flash_high);
		else
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
 * aw3641 operations
 *****************************************************************************/
/* flashlight enable function */
static int aw3641_enable(void)
{
	//int state = 1;
	printk("FLASHLIGHT %s\n",__func__);
	/* TODO: wrap enable function */

	return aw3641_pinctrl_set(HWEN_PIN, 1);
}

/* flashlight disable function */
static int aw3641_disable(void)
{
	int ret;
	printk("FLASHLIGHT %s\n",__func__);
	/* TODO: wrap disable function */
	aw3641_pinctrl_set(HWEN_PIN, 0);
	ret = aw3641_pinctrl_set(FLASH_PIN, 0);

	return ret;
}

/* set flashlight level */
static int aw3641_set_level(int level)
{
	int state = 0;
	printk("FLASHLIGHT %s duty = %d\n",__func__,level);
	/* TODO: wrap set level function */
	if (level == 0){
		state = 0;
	}else{
		state = 1;
	}

	return aw3641_pinctrl_set(FLASH_PIN, state);
}

/* flashlight init */
static int aw3641_init(void)
{
	int ret;
	/* TODO: wrap init function */

	aw3641_pinctrl_set(HWEN_PIN, 0);
	ret = aw3641_pinctrl_set(FLASH_PIN, 0);
	return ret;
}

/* flashlight uninit */
static int aw3641_uninit(void)
{
	int ret;
printk("FLASHLIGHT %s\n",__func__);
	/* TODO: wrap uninit function */
	aw3641_pinctrl_set(HWEN_PIN, 0);
	ret = aw3641_pinctrl_set(FLASH_PIN, 0);
	return ret;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer aw3641_timer;
static unsigned int aw3641_timeout_ms;

static void aw3641_work_disable(struct work_struct *data)
{
	pr_debug("work queue callback\n");
	aw3641_disable();
}

static enum hrtimer_restart aw3641_timer_func(struct hrtimer *timer)
{
	schedule_work(&aw3641_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int aw3641_ioctl(unsigned int cmd, unsigned long arg)
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
		aw3641_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
//prize lsw add for only two duty mode to adapt prize flash mode start
#ifdef	PPRIZE_TWO_DUTY_MODE
		if(fl_arg->arg >= 2)
			aw3641_set_level(0);
		else
#endif
//prize lsw add for only two duty mode to adapt prize flash mode end		
			aw3641_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_debug("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (aw3641_timeout_ms) {
				ktime = ktime_set(aw3641_timeout_ms / 1000,
						(aw3641_timeout_ms % 1000) * 1000000);
				hrtimer_start(&aw3641_timer, ktime, HRTIMER_MODE_REL);
			}
			aw3641_enable();
		} else {
			aw3641_disable();
			hrtimer_cancel(&aw3641_timer);
		}
		break;
	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int aw3641_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int aw3641_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int aw3641_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&aw3641_mutex);
	if (set) {
		if (!use_count)
			ret = aw3641_init();
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = aw3641_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&aw3641_mutex);

	return ret;
}

static ssize_t aw3641_strobe_store(struct flashlight_arg arg)
{
	aw3641_set_driver(1);
	aw3641_set_level(arg.level);
	aw3641_timeout_ms = 0;
	aw3641_enable();
	msleep(arg.dur);
	aw3641_disable();
	aw3641_set_driver(0);

	return 0;
}

static struct flashlight_operations aw3641_ops = {
	aw3641_open,
	aw3641_release,
	aw3641_ioctl,
	aw3641_strobe_store,
	aw3641_set_driver
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int aw3641_chip_init(void)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * aw3641_init();
	 */

	return 0;
}

static int aw3641_parse_dt(struct device *dev,
		struct aw3641_platform_data *pdata)
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
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE, AW3641_NAME);
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

static int aw3641_probe(struct platform_device *pdev)
{
	struct aw3641_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int err;
	int i;

	pr_debug("Probe start.\n");

	/* init pinctrl */
	if (aw3641_pinctrl_init(pdev)) {
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
		err = aw3641_parse_dt(&pdev->dev, pdata);
		if (err)
		goto err;
	}

	/* init work queue */
	INIT_WORK(&aw3641_work, aw3641_work_disable);

	/* init timer */
	hrtimer_init(&aw3641_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw3641_timer.function = aw3641_timer_func;
	aw3641_timeout_ms = 100;

	/* init chip hw */
	aw3641_chip_init();

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(&pdata->dev_id[i], &aw3641_ops)) {
				err = -EFAULT;
				goto err;
			}
	} else {
		if (flashlight_dev_register(AW3641_NAME, &aw3641_ops)) {
			err = -EFAULT;
			goto err;
		}
	}

	pr_debug("Probe done.\n");

	return 0;
err:
	return err;
}

static int aw3641_remove(struct platform_device *pdev)
{
	struct aw3641_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	pr_debug("Remove start.\n");

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(AW3641_NAME);

	/* flush work queue */
	flush_work(&aw3641_work);

	pr_debug("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id aw3641_gpio_of_match[] = {
	{.compatible = AW3641_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, aw3641_gpio_of_match);
#else
static struct platform_device aw3641_gpio_platform_device[] = {
	{
		.name = AW3641_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, aw3641_gpio_platform_device);
#endif

static struct platform_driver aw3641_platform_driver = {
	.probe = aw3641_probe,
	.remove = aw3641_remove,
	.driver = {
		.name = AW3641_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = aw3641_gpio_of_match,
#endif
	},
};

static int __init flashlight_aw3641_init(void)
{
	int ret;

	pr_debug("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&aw3641_gpio_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&aw3641_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_aw3641_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&aw3641_platform_driver);

	pr_debug("Exit done.\n");
}

module_init(flashlight_aw3641_init);
module_exit(flashlight_aw3641_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight aw3641 GPIO Driver");


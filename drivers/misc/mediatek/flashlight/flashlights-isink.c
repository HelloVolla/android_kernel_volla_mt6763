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
#include <mach/upmu_hw.h>
#include <mach/upmu_sw.h>
#include <mt-plat/upmu_common.h>

/* define device tree */
/* TODO: modify temp device tree name */
#ifndef ISINK_DTNAME
#define ISINK_DTNAME "mediatek,flashlights_isink"
#endif

/* TODO: define driver name */
#define ISINK_NAME "flashlights-isink"

/* define registers */
/* TODO: define register */

/* define mutex and work queue */
static DEFINE_MUTEX(isink_mutex);
static struct work_struct isink_work;

/* define pinctrl */

/* define usage count */
static int use_count;

/* platform data */
struct isink_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

typedef enum{   
    ISINK_DISABLE = 0,
	ISINK_ENABLE = 1, 
    ISINK_UNKNOW
} MT65XX_PMIC_CTRL_SUB;

typedef enum{  
    ISINK0,  
    ISINK1,  
    ISINK_NUM
} MT65XX_PMIC_ISINK_NO_SUB;

enum MT65XX_PMIC_ISINK_STEP {
	ISINK_0 = 0,		/* 4mA */
	ISINK_1 = 1,		/* 8mA */
	ISINK_2 = 2,		/* 12mA */
	ISINK_3 = 3,		/* 16mA */
	ISINK_4 = 4,		/* 20mA */
	ISINK_5 = 5		/* 24mA */
};

enum MT65XX_PMIC_ISINK_MODE {
	ISINK_PWM_MODE = 0,
	ISINK_BREATH_MODE = 1,
	ISINK_REGISTER_MODE = 2
};

enum MT65XX_PMIC_ISINK_FSEL {
	/* 32K clock */
	ISINK_1KHZ = 0,
	ISINK_200HZ = 4,
	ISINK_5HZ = 199,
	ISINK_2HZ = 499,
	ISINK_1HZ = 999,
	ISINK_05HZ = 1999,
	ISINK_02HZ = 4999,
	ISINK_01HZ = 9999,
	/* 2M clock */
	ISINK_2M_20KHZ = 2,
	ISINK_2M_1KHZ = 61,
	ISINK_2M_200HZ = 311,
	ISINK_2M_5HZ = 12499,
	ISINK_2M_2HZ = 31249,
	ISINK_2M_1HZ = 62499,

	/* 128K clock */
	ISINK_128K_500HZ = 0,
	ISINK_128K_256HZ = 1,
	ISINK_128K_167HZ = 2,
	ISINK_128K_128HZ = 3,
	ISINK_128K_100HZ = 4,
	ISINK_128K_83HZ = 5,
	ISINK_128K_50HZ = 9,
	ISINK_128K_17HZ = 28,
};

//kernel-4.4/drivers/misc/mediatek/pmic/mt6357/v1/pmic_common.c     [pmu_flags_table]
#define PMIC_RG_DRV_128K_CK_PDN 3573

#ifdef CONFIG_MTK_PMIC_CHIP_MT6356
#define PMIC_ISINK_CH0_MODE 4761
#define PMIC_ISINK_CH0_STEP 4722
#define PMIC_ISINK_DIM0_DUTY 4721
#define PMIC_ISINK_DIM0_FSEL 4719
#define PMIC_ISINK_CH0_EN 4753
#define PMIC_RG_DRV_ISINK0_CK_PDN 4656
#define PMIC_ISINK_CH0_BIAS_EN 4757
#define PMIC_ISINK_CHOP0_EN 4755
#endif

#define PMIC_ISINK_CH1_EN 3621
#define	PMIC_RG_DRV_ISINK1_CK_PDN 3570
#define	PMIC_ISINK_CH1_MODE 3626
#define	PMIC_ISINK_CH1_STEP 3606
#define	PMIC_ISINK_DIM1_DUTY 3605
#define	PMIC_ISINK_DIM1_FSEL 3599
#define	PMIC_ISINK_CH1_BIAS_EN 3623
#define	PMIC_ISINK_CHOP1_EN 3622


/******************************************************************************
 * isink operations
 *****************************************************************************/
/* flashlight enable function */
static int isink_enable(void)
{
	pmic_set_register_value(PMIC_ISINK_CH1_BIAS_EN, ISINK_ENABLE);
	pmic_set_register_value(PMIC_ISINK_CHOP1_EN, ISINK_ENABLE);
	pmic_set_register_value(PMIC_ISINK_CH1_EN, ISINK_ENABLE);
	return 0;
}

/* flashlight disable function */
static int isink_disable(void)
{
	pmic_set_register_value(PMIC_ISINK_CH1_BIAS_EN, ISINK_DISABLE);
	pmic_set_register_value(PMIC_ISINK_CHOP1_EN, ISINK_DISABLE);
	pmic_set_register_value(PMIC_ISINK_CH1_EN, ISINK_DISABLE);
	return 0;
}

/* set flashlight level */
static int isink_set_level(int level)
{
	int state = 0;
	
	/* TODO: wrap set level function */
	printk("flashlight duty = %d\n",level);
	if (level == 0){
		state = ISINK_3;
	}else{
		state = ISINK_3;
	}

	pmic_set_register_value(PMIC_ISINK_CH1_STEP, state);
	return 0;
}

/* flashlight init */
static int isink_init(void)
{
	pmic_set_register_value(PMIC_RG_DRV_128K_CK_PDN, 0x0);	/* Disable power down */
	pmic_set_register_value(PMIC_RG_DRV_ISINK1_CK_PDN, 0);
	pmic_set_register_value(PMIC_ISINK_CH1_MODE,ISINK_REGISTER_MODE);
	pmic_set_register_value(PMIC_ISINK_DIM1_DUTY, 255);
	pmic_set_register_value(PMIC_ISINK_DIM1_FSEL, ISINK_1KHZ);

	return 0;
}

/* flashlight uninit */
static int isink_uninit(void)
{
	isink_disable();
	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer isink_timer;
static unsigned int isink_timeout_ms;

static void isink_work_disable(struct work_struct *data)
{
	pr_debug("work queue callback\n");
	isink_disable();
}

static enum hrtimer_restart isink_timer_func(struct hrtimer *timer)
{
	schedule_work(&isink_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int isink_ioctl(unsigned int cmd, unsigned long arg)
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
		isink_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		isink_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_debug("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (isink_timeout_ms) {
				ktime = ktime_set(isink_timeout_ms / 1000,
						(isink_timeout_ms % 1000) * 1000000);
				hrtimer_start(&isink_timer, ktime, HRTIMER_MODE_REL);
			}
			isink_enable();
		} else {
			isink_disable();
			hrtimer_cancel(&isink_timer);
		}
		break;
	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int isink_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int isink_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int isink_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&isink_mutex);
	if (set) {
		if (!use_count)
			ret = isink_init();
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = isink_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&isink_mutex);

	return ret;
}

static ssize_t isink_strobe_store(struct flashlight_arg arg)
{
	isink_set_driver(1);
	isink_set_level(arg.level);
	isink_timeout_ms = 0;
	isink_enable();
	msleep(arg.dur);
	isink_disable();
	isink_set_driver(0);

	return 0;
}

static struct flashlight_operations isink_ops = {
	isink_open,
	isink_release,
	isink_ioctl,
	isink_strobe_store,
	isink_set_driver
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int isink_chip_init(void)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * isink_init();
	 */

	return 0;
}

static int isink_parse_dt(struct device *dev,
		struct isink_platform_data *pdata)
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
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE, ISINK_NAME);
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

static int isink_probe(struct platform_device *pdev)
{
	struct isink_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int err;
	int i;

	printk("[flashlight-isink.c][isink_probe]Probe start.\n");


	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err;
		}
		pdev->dev.platform_data = pdata;
		err = isink_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err;
	}

	/* init work queue */
	INIT_WORK(&isink_work, isink_work_disable);

	/* init timer */
	hrtimer_init(&isink_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	isink_timer.function = isink_timer_func;
	isink_timeout_ms = 100;

	/* init chip hw */
	isink_chip_init();

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(&pdata->dev_id[i], &isink_ops)) {
				err = -EFAULT;
				goto err;
			}
	} else {
		if (flashlight_dev_register(ISINK_NAME, &isink_ops)) {
			err = -EFAULT;
			goto err;
		}
	}

	pr_debug("Probe done.\n");

	return 0;
err:
	return err;
}

static int isink_remove(struct platform_device *pdev)
{
	struct isink_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	pr_debug("Remove start.\n");

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(ISINK_NAME);

	/* flush work queue */
	flush_work(&isink_work);

	pr_debug("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id isink_of_match[] = {
	{.compatible = ISINK_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, isink_of_match);
#else
static struct platform_device isink_platform_device[] = {
	{
		.name = ISINK_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, isink_platform_device);
#endif

static struct platform_driver isink_platform_driver = {
	.probe = isink_probe,
	.remove = isink_remove,
	.driver = {
		.name = ISINK_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = isink_of_match,
#endif
	},
};

static int __init flashlight_isink_init(void)
{
	int ret;

	pr_debug("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&isink_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&isink_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_isink_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&isink_platform_driver);

	pr_debug("Exit done.\n");
}

module_init(flashlight_isink_init);
module_exit(flashlight_isink_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight ISINK GPIO Driver");


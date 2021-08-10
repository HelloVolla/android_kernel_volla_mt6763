/*
 *  Copyright (C) 2016 Richtek Technology Corp.
 *  cy_huang <cy_huang@richtek.com>
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/leds.h>

#include "../../flashlight/richtek/rtfled.h"
#include "inc/rt5081_pmu.h"
#include "inc/rt5081_pmu_bled.h"

struct rt5081_pmu_bled_data {
	struct rt_fled_dev base;
	struct rt5081_pmu_chip *chip;
	struct device *dev;
	struct platform_device *rt_flash_dev;
};

static uint8_t bled_init_data[] = {
	0x42, /* RT5081_PMU_REG_BLEN */
	0x89, /* RT5081_PMU_REG_BLBSTCTRL */
	0x00, /* RT5081_PMU_REG_BLPWM */
	0x00, /* RT5081_PMU_REG_BLCTRL */
	0x00, /* RT5081_PMU_REG_BLDIM2 */
	0x00, /* RT5081_PMU_REG_BLDIM1 */
	0x00, /* RT5081_PMU_REG_BLAFH */
	0x00, /* RT5081_PMU_REG_BLFL */
	0x8c, /* RT5081_PMU_REG_BLFLTO */
	0x80, /* RT5081_PMU_REG_BLTORCTRL */
	0xff, /* RT5081_PMU_REG_BLSTRBCTRL */
	0x00, /* RT5081_PMU_REG_BLAVG */
};

static int rt5081_bled_fled_set_mode(struct rt_fled_dev *fled_dev,
	enum flashlight_mode mode)
{
	struct rt5081_pmu_bled_data *bled_data =
			(struct rt5081_pmu_bled_data *)fled_dev;
	int ret = 0;

	switch (mode) {
	case FLASHLIGHT_MODE_OFF:
		ret = rt5081_pmu_reg_update_bits(bled_data->chip,
				RT5081_PMU_REG_BLFL, RT5081_BLFLMODE_MASK,
				0 << RT5081_BLFLMODE_SHFT);
		break;
	case FLASHLIGHT_MODE_TORCH:
		ret = rt5081_pmu_reg_update_bits(bled_data->chip,
				RT5081_PMU_REG_BLFL, RT5081_BLFLMODE_MASK,
				2 << RT5081_BLFLMODE_SHFT);
		break;
	case FLASHLIGHT_MODE_FLASH:
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int rt5081_bled_fled_get_mode(struct rt_fled_dev *fled_dev)
{
	struct rt5081_pmu_bled_data *bled_data =
			(struct rt5081_pmu_bled_data *)fled_dev;
	int ret = 0;

	ret = rt5081_pmu_reg_read(bled_data->chip, RT5081_PMU_REG_BLFL);
	if (ret < 0)
		return ret;
	ret &= RT5081_BLFLMODE_MASK;
	ret >>= RT5081_BLFLMODE_SHFT;
	switch (ret) {
	case 0:
		ret = FLASHLIGHT_MODE_OFF;
		break;
	case 1:
		ret = FLASHLIGHT_MODE_FLASH;
		break;
	case 2:
	case 3:
		ret = FLASHLIGHT_MODE_TORCH;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int rt5081_bled_fled_strobe(struct rt_fled_dev *fled_dev)
{
	struct rt5081_pmu_bled_data *bled_data =
			(struct rt5081_pmu_bled_data *)fled_dev;
	int ret = 0;

	ret = rt5081_bled_fled_set_mode(fled_dev, FLASHLIGHT_MODE_OFF);
	if (ret < 0)
		return ret;
	return rt5081_pmu_reg_update_bits(bled_data->chip,
			RT5081_PMU_REG_BLFL, RT5081_BLFLMODE_MASK,
			1 << RT5081_BLFLMODE_SHFT);
}

static int rt5081_bled_fled_torch_current_list(struct rt_fled_dev *fled_dev,
	int selector)
{
	if (selector >= 256)
		return -EINVAL;
	return 117 * selector;
}

static int rt5081_bled_fled_strobe_current_list(struct rt_fled_dev *fled_dev,
	int selector)
{
	if (selector >= 256)
		return -EINVAL;
	return 117 * selector;
}

static int rt5081_bled_fled_timeout_level_list(struct rt_fled_dev *fled_dev,
	int selector)
{
	return -EINVAL;
}

static int rt5081_bled_fled_lv_protection_list(struct rt_fled_dev *fled_dev,
	int selector)
{
	return -EINVAL;
}

static int rt5081_bled_fled_strobe_timeout_list(struct rt_fled_dev *fled_dev,
	int selector)
{
	if (selector >= 128)
		return -EINVAL;
	return 16 * selector;
}

static int rt5081_bled_fled_set_torch_current_sel(struct rt_fled_dev *fled_dev,
	int selector)
{
	struct rt5081_pmu_bled_data *bled_data =
			(struct rt5081_pmu_bled_data *)fled_dev;

	if (selector >= 256)
		return -EINVAL;
	return rt5081_pmu_reg_write(bled_data->chip,
			RT5081_PMU_REG_BLTORCTRL, selector);
}

static int rt5081_bled_fled_set_strobe_current_sel(struct rt_fled_dev *fled_dev,
	int selector)
{
	struct rt5081_pmu_bled_data *bled_data =
			(struct rt5081_pmu_bled_data *)fled_dev;

	if (selector >= 256)
		return -EINVAL;
	return rt5081_pmu_reg_write(bled_data->chip,
			RT5081_PMU_REG_BLSTRBCTRL, selector);
}

static int rt5081_bled_fled_set_timeout_level_sel(struct rt_fled_dev *fled_dev,
	int selector)
{
	return -EINVAL;
}

static int rt5081_bled_fled_set_lv_protection_sel(struct rt_fled_dev *fled_dev,
	int selector)
{
	return -EINVAL;
}

static int rt5081_bled_fled_set_strobe_timeout_sel(struct rt_fled_dev *fled_dev,
	int selector)
{
	struct rt5081_pmu_bled_data *bled_data =
			(struct rt5081_pmu_bled_data *)fled_dev;

	if (selector >= 128)
		return -EINVAL;
	return rt5081_pmu_reg_update_bits(bled_data->chip,
			RT5081_PMU_REG_BLFLTO, RT5081_BLSTRB_TOMASK, selector);
}

static int rt5081_bled_fled_get_torch_current_sel(struct rt_fled_dev *fled_dev)
{
	struct rt5081_pmu_bled_data *bled_data =
			(struct rt5081_pmu_bled_data *)fled_dev;

	return rt5081_pmu_reg_read(bled_data->chip, RT5081_PMU_REG_BLTORCTRL);
}

static int rt5081_bled_fled_get_strobe_current_sel(struct rt_fled_dev *fled_dev)
{
	struct rt5081_pmu_bled_data *bled_data =
			(struct rt5081_pmu_bled_data *)fled_dev;

	return rt5081_pmu_reg_read(bled_data->chip, RT5081_PMU_REG_BLSTRBCTRL);
}

static int rt5081_bled_fled_get_timeout_level_sel(struct rt_fled_dev *fled_dev)
{
	return -EINVAL;
}

static int rt5081_bled_fled_get_lv_protection_sel(struct rt_fled_dev *fled_dev)
{
	return -EINVAL;
}

static int rt5081_bled_fled_get_strobe_timeout_sel(struct rt_fled_dev *fled_dev)
{
	struct rt5081_pmu_bled_data *bled_data =
			(struct rt5081_pmu_bled_data *)fled_dev;
	int ret = 0;

	ret = rt5081_pmu_reg_read(bled_data->chip, RT5081_PMU_REG_BLFLTO);
	return (ret < 0 ? ret : ret & RT5081_BLSTRB_TOMASK);
}

static void rt5081_bled_fled_shutdown(struct rt_fled_dev *fled_dev)
{
	rt5081_bled_fled_set_mode(fled_dev, FLASHLIGHT_MODE_OFF);
}

static struct rt_fled_hal rt5081_bledfl_hal = {
	.rt_hal_fled_set_mode = rt5081_bled_fled_set_mode,
	.rt_hal_fled_get_mode = rt5081_bled_fled_get_mode,
	.rt_hal_fled_strobe = rt5081_bled_fled_strobe,
	.rt_hal_fled_torch_current_list = rt5081_bled_fled_torch_current_list,
	.rt_hal_fled_strobe_current_list = rt5081_bled_fled_strobe_current_list,
	.rt_hal_fled_timeout_level_list = rt5081_bled_fled_timeout_level_list,
	.rt_hal_fled_lv_protection_list = rt5081_bled_fled_lv_protection_list,
	.rt_hal_fled_strobe_timeout_list = rt5081_bled_fled_strobe_timeout_list,
	.rt_hal_fled_set_torch_current_sel =
					rt5081_bled_fled_set_torch_current_sel,
	.rt_hal_fled_set_strobe_current_sel =
					rt5081_bled_fled_set_strobe_current_sel,
	.rt_hal_fled_set_timeout_level_sel =
					rt5081_bled_fled_set_timeout_level_sel,
	.rt_hal_fled_set_lv_protection_sel =
					rt5081_bled_fled_set_lv_protection_sel,
	.rt_hal_fled_set_strobe_timeout_sel =
					rt5081_bled_fled_set_strobe_timeout_sel,
	.rt_hal_fled_get_torch_current_sel =
					rt5081_bled_fled_get_torch_current_sel,
	.rt_hal_fled_get_strobe_current_sel =
					rt5081_bled_fled_get_strobe_current_sel,
	.rt_hal_fled_get_timeout_level_sel =
					rt5081_bled_fled_get_timeout_level_sel,
	.rt_hal_fled_get_lv_protection_sel =
					rt5081_bled_fled_get_lv_protection_sel,
	.rt_hal_fled_get_strobe_timeout_sel =
					rt5081_bled_fled_get_strobe_timeout_sel,
	.rt_hal_fled_shutdown = rt5081_bled_fled_shutdown,
};

static const struct flashlight_properties rt5081_bledfl_props = {
	.type = FLASHLIGHT_TYPE_LED,
	.torch_brightness = 128, /* default torch brightness 15mA */
	.torch_max_brightness = 255,
	.strobe_brightness = 255, /* default strobe brightness 30mA */
	.strobe_max_brightness = 255,
	.strobe_timeout = 208, /* default strobe timeout 208mS */
	.alias_name = "rt5081_pmu_bled",
};

static void rt5081_pmu_bled_bright_set(struct led_classdev *led_cdev,
	enum led_brightness brightness)
{
	struct rt5081_pmu_bled_data *bled_data =
				dev_get_drvdata(led_cdev->dev->parent);
	struct rt5081_pmu_bled_platdata *pdata =
				dev_get_platdata(bled_data->dev);
	uint32_t bright = (pdata->max_bled_brightness << 8) / 255;
	int ret = 0;

	dev_dbg(led_cdev->dev, "%s: %d\n", __func__, brightness);
	bright = (bright * brightness) >> 8;
	ret = rt5081_pmu_reg_update_bits(bled_data->chip, RT5081_PMU_REG_BLDIM2,
					 RT5081_DIM2_MASK, bright & 0x7);
	if (ret < 0)
		goto out_bright_set;
	ret = rt5081_pmu_reg_write(bled_data->chip, RT5081_PMU_REG_BLDIM1,
				   (bright >> 3) & RT5081_DIM_MASK);
	if (ret < 0)
		goto out_bright_set;
	/* if choose external enable pin, no effect even config this bit */
	ret = rt5081_pmu_reg_update_bits(bled_data->chip, RT5081_PMU_REG_BLEN,
					 RT5081_BLED_EN,
					 brightness > 0 ?
					 RT5081_BLED_EN : ~RT5081_BLED_EN);
	if (ret < 0)
		goto out_bright_set;
	return;
out_bright_set:
	dev_dbg(led_cdev->dev, "%s error %d\n", __func__, ret);
}

static struct led_classdev rt5081_pmu_bled_dev = {
	.name = "rt5081_pmu_bled",
	.brightness_set = rt5081_pmu_bled_bright_set,
};

static irqreturn_t rt5081_pmu_bled_ocp_irq_handler(int irq, void *data)
{
	struct rt5081_pmu_bled_data *bled_data = data;

	dev_dbg(bled_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5081_pmu_bled_ovp_irq_handler(int irq, void *data)
{
	struct rt5081_pmu_bled_data *bled_data = data;

	dev_dbg(bled_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static struct rt5081_pmu_irq_desc rt5081_bled_irq_desc[] = {
	RT5081_PMU_IRQDESC(bled_ocp),
	RT5081_PMU_IRQDESC(bled_ovp),
};

static void rt5081_pmu_bled_irq_register(struct platform_device *pdev)
{
	struct resource *res;
	int i, ret = 0;

	for (i = 0; i < ARRAY_SIZE(rt5081_bled_irq_desc); i++) {
		if (!rt5081_bled_irq_desc[i].name)
			continue;
		res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
					rt5081_bled_irq_desc[i].name);
		if (!res)
			continue;
		ret = devm_request_threaded_irq(&pdev->dev, res->start, NULL,
					rt5081_bled_irq_desc[i].irq_handler,
					IRQF_TRIGGER_FALLING,
					rt5081_bled_irq_desc[i].name,
					platform_get_drvdata(pdev));
		if (ret < 0) {
			dev_dbg(&pdev->dev, "request %s irq fail\n", res->name);
			continue;
		}
		rt5081_bled_irq_desc[i].irq = res->start;
	}
}

static inline int rt5081_pmu_bled_init_register(
	struct rt5081_pmu_bled_data *bled_data)
{
	if (bled_data->chip->chip_rev <= 1)
		bled_init_data[1] |= RT5081_BLED_OVOCSHDNDIS;
	return rt5081_pmu_reg_block_write(bled_data->chip, RT5081_PMU_REG_BLEN,
			ARRAY_SIZE(bled_init_data), bled_init_data);
}

static inline int rt5081_pmu_bled_parse_initdata(
	struct rt5081_pmu_bled_data *bled_data)
{
	struct rt5081_pmu_bled_platdata *pdata = dev_get_platdata(bled_data->dev);
	uint32_t bright = (pdata->max_bled_brightness << 8) / 255;

	if (pdata->ext_en_pin) {
		bled_init_data[0] &= ~(RT5081_BLED_EN | RT5081_BLED_EXTEN);
		bled_init_data[0] |= RT5081_BLED_EXTEN;
	}
	bled_init_data[0] |= (pdata->chan_en << RT5081_BLED_CHANENSHFT);
	if (!pdata->map_linear)
		bled_init_data[0] &= ~(RT5081_BLED_MAPLINEAR);
	bled_init_data[1] |= (pdata->bl_ovp_level << RT5081_BLED_OVPSHFT);
	bled_init_data[1] |= (pdata->bl_ocp_level << RT5081_BLED_OCPSHFT);
	bled_init_data[2] |= (pdata->use_pwm << RT5081_BLED_PWMSHIFT);
	bled_init_data[2] |= (pdata->pwm_fsample << RT5081_BLED_PWMFSHFT);
	bled_init_data[2] |= (pdata->pwm_deglitch << RT5081_BLED_PWMDSHFT);
	bled_init_data[3] |= (pdata->bled_ramptime << RT5081_BLED_RAMPTSHFT);
	bright = (bright * 255) >> 8;
	bled_init_data[4] |= (bright & 0x7);
	bled_init_data[5] |= ((bright >> 3) & 0xff);
	bled_init_data[7] |= (pdata->bled_flash_ramp << RT5081_BLFLRAMP_SHFT);
	bled_init_data[10] |= (pdata->pwm_avg_cycle);
	if (pdata->bled_name)
		rt5081_pmu_bled_dev.name = pdata->bled_name;
	return 0;
}

static inline int rt_parse_dt(struct device *dev)
{
	struct rt5081_pmu_bled_platdata *pdata = dev_get_platdata(dev);
	struct device_node *np = dev->of_node;
	u32 tmp = 0;

	if (of_property_read_bool(np, "rt,ext_en_pin"))
		pdata->ext_en_pin = 1;
	if (of_property_read_u32(np, "rt,chan_en", &tmp) < 0)
		pdata->chan_en = 0x0; /* default 4 channel disable */
	else
		pdata->chan_en = tmp;
	if (of_property_read_bool(np, "rt,map_linear"))
		pdata->map_linear = 1;
	if (of_property_read_u32(np, "rt,bl_ovp_level", &tmp) < 0)
		pdata->bl_ovp_level = 0x3; /* default 29V */
	else
		pdata->bl_ovp_level = tmp;
	if (of_property_read_u32(np, "rt,bl_ocp_level", &tmp) < 0)
		pdata->bl_ocp_level = 0x2; /* default 1500mA */
	else
		pdata->bl_ocp_level = tmp;
	if (of_property_read_bool(np, "rt,use_pwm"))
		pdata->use_pwm = 1;
	if (of_property_read_u32(np, "rt,pwm_fsample", &tmp) < 0)
		pdata->pwm_fsample = 0x1; /* 4MHz */
	else
		pdata->pwm_fsample = tmp;
	if (of_property_read_u32(np, "rt,pwm_deglitch", &tmp) < 0)
		pdata->pwm_deglitch = 0x1;
	else
		pdata->pwm_deglitch = tmp;
	if (of_property_read_u32(np, "rt,pwm_hys_en", &tmp) < 0)
		pdata->pwm_hys_en = 0x1;
	else
		pdata->pwm_hys_en = tmp;
	if (of_property_read_u32(np, "rt,pwm_hys", &tmp) < 0)
		pdata->pwm_hys = 0x0;	/* 1 bit */
	else
		pdata->pwm_hys = tmp;
	if (of_property_read_u32(np, "rt,pwm_avg_cycle", &tmp) < 0)
		pdata->pwm_avg_cycle = 0;
	else
		pdata->pwm_avg_cycle = tmp;
	if (of_property_read_u32(np, "rt,bled_ramptime", &tmp) < 0)
		pdata->bled_ramptime = 0x3; /* default 1ms */
	else
		pdata->bled_ramptime = tmp;
	if (of_property_read_u32(np, "rt,bled_flash_ramp", &tmp) < 0)
		pdata->bled_flash_ramp = 0x1;
	else
		pdata->bled_flash_ramp = tmp;
	if (of_property_read_u32(np, "rt,max_bled_brightness", &tmp) < 0)
		pdata->max_bled_brightness = 1024;
	else
		pdata->max_bled_brightness = tmp;
	if (of_property_read_string(np, "rt,bled_name", &(pdata->bled_name)) < 0)
		pdata->bled_name = "rt5081_pmu_bled";
	return 0;
}

static int rt5081_pmu_bled_probe(struct platform_device *pdev)
{
	struct rt5081_pmu_bled_platdata *pdata = dev_get_platdata(&pdev->dev);
	struct rt5081_pmu_bled_data *bled_data;
	bool use_dt = pdev->dev.of_node;
	int ret = 0;

	bled_data = devm_kzalloc(&pdev->dev, sizeof(*bled_data), GFP_KERNEL);
	if (!bled_data)
		return -ENOMEM;
	if (use_dt) {
		/* DTS used */
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			ret = -ENOMEM;
			goto out_pdata;
		}
		pdev->dev.platform_data = pdata;
		ret = rt_parse_dt(&pdev->dev);
		if (ret < 0) {
			devm_kfree(&pdev->dev, pdata);
			goto out_pdata;
		}
	} else {
		if (!pdata) {
			ret = -EINVAL;
			goto out_pdata;
		}
	}
	bled_data->chip = dev_get_drvdata(pdev->dev.parent);
	bled_data->dev = &pdev->dev;
	platform_set_drvdata(pdev, bled_data);

	ret = rt5081_pmu_bled_parse_initdata(bled_data);
	if (ret < 0)
		goto out_init_data;

	ret = rt5081_pmu_bled_init_register(bled_data);
	if (ret < 0)
		goto out_init_reg;

	if (pdata->use_pwm)
		rt5081_pmu_bled_dev.default_trigger = "backlight";
	rt5081_pmu_bled_dev.brightness = LED_FULL;
	ret = led_classdev_register(bled_data->dev, &rt5081_pmu_bled_dev);
	if (ret < 0) {
		dev_dbg(&pdev->dev, "register leddev fail\n");
		goto out_led_register;
	}

	bled_data->base.init_props = &rt5081_bledfl_props;
	bled_data->base.hal = &rt5081_bledfl_hal;
	bled_data->base.name = kstrndup(pdata->bled_name,
					strlen(pdata->bled_name), GFP_KERNEL);
	bled_data->base.chip_name = "rt5081_pmu_bled";
	bled_data->rt_flash_dev =
		platform_device_register_resndata(bled_data->dev,
						  "rt-flash-led", 2, NULL, 0,
						  NULL, 0);

	if (IS_ERR(bled_data->rt_flash_dev)) {
		dev_dbg(&pdev->dev, "register rt_flash_dev fail\n");
		goto out_rt_flash_register;
	}

	rt5081_pmu_bled_irq_register(pdev);
	dev_info(&pdev->dev, "%s successfully\n", __func__);
	return 0;
out_rt_flash_register:
	led_classdev_unregister(&rt5081_pmu_bled_dev);
out_led_register:
out_init_reg:
out_init_data:
out_pdata:
	devm_kfree(&pdev->dev, bled_data);
	return ret;
}

static int rt5081_pmu_bled_remove(struct platform_device *pdev)
{
	struct rt5081_pmu_bled_data *bled_data = platform_get_drvdata(pdev);

	platform_device_unregister(bled_data->rt_flash_dev);
	led_classdev_unregister(&rt5081_pmu_bled_dev);
	dev_info(bled_data->dev, "%s successfully\n", __func__);
	return 0;
}

static const struct of_device_id rt_ofid_table[] = {
	{ .compatible = "richtek,rt5081_pmu_bled", },
	{ },
};
MODULE_DEVICE_TABLE(of, rt_ofid_table);

static const struct platform_device_id rt_id_table[] = {
	{ "rt5081_pmu_bled", 0},
	{ },
};
MODULE_DEVICE_TABLE(platform, rt_id_table);

static struct platform_driver rt5081_pmu_bled = {
	.driver = {
		.name = "rt5081_pmu_bled",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rt_ofid_table),
	},
	.probe = rt5081_pmu_bled_probe,
	.remove = rt5081_pmu_bled_remove,
	.id_table = rt_id_table,
};
module_platform_driver(rt5081_pmu_bled);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("cy_huang <cy_huang@richtek.com>");
MODULE_DESCRIPTION("Richtek RT5081 PMU Bled");
MODULE_VERSION("1.0.1_MTK");

/*
 * Version Note
 * 1.0.1_MTK
 * (1) Remove typedef
 *
 * 1.0.0_MTK
 * (1) Initial Release
 */

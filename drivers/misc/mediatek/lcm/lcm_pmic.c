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

#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/kernel.h>

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
static struct regulator *disp_bias_pos;
static struct regulator *disp_bias_neg;
static int regulator_inited;

int display_bias_regulator_init(void)
{
	int ret = 0;

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	disp_bias_pos = regulator_get(NULL, "dsv_pos");
	if (IS_ERR(disp_bias_pos)) { /* handle return value */
		ret = PTR_ERR(disp_bias_pos);
		pr_info("get dsv_pos fail, error: %d\n", ret);
		return ret;
	}

	disp_bias_neg = regulator_get(NULL, "dsv_neg");
	if (IS_ERR(disp_bias_neg)) { /* handle return value */
		ret = PTR_ERR(disp_bias_neg);
		pr_info("get dsv_neg fail, error: %d\n", ret);
		return ret;
	}

	regulator_inited = 1;
	return ret; /* must be 0 */

}
EXPORT_SYMBOL(display_bias_regulator_init);

int display_bias_enable(void)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(disp_bias_pos, 5900000, 5900000);
	if (ret < 0)
		pr_info("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_set_voltage(disp_bias_neg, 5900000, 5900000);
	if (ret < 0)
		pr_info("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

#if 0
	/* get voltage */
	ret = mtk_regulator_get_voltage(&disp_bias_pos);
	if (ret < 0)
		pr_info("get voltage disp_bias_pos fail\n");
	pr_debug("pos voltage = %d\n", ret);

	ret = mtk_regulator_get_voltage(&disp_bias_neg);
	if (ret < 0)
		pr_info("get voltage disp_bias_neg fail\n");
	pr_debug("neg voltage = %d\n", ret);
#endif
	/* enable regulator */
	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_info("enable regulator disp_bias_pos fail, ret = %d\n",
			ret);
	retval |= ret;

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_info("enable regulator disp_bias_neg fail, ret = %d\n",
			ret);
	retval |= ret;

	return retval;
}
EXPORT_SYMBOL(display_bias_enable);

int display_bias_disable(void)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	ret = regulator_disable(disp_bias_neg);
	if (ret < 0)
		pr_info("disable regulator disp_bias_neg fail, ret = %d\n",
			ret);
	retval |= ret;

	ret = regulator_disable(disp_bias_pos);
	if (ret < 0)
		pr_info("disable regulator disp_bias_pos fail, ret = %d\n",
			ret);
	retval |= ret;

	return retval;
}
EXPORT_SYMBOL(display_bias_disable);
int display_bias_vpos_enable(int enable)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	if (enable){
		/* enable regulator */
		ret = regulator_enable(disp_bias_pos);
		if (ret < 0)
			pr_err("enable regulator disp_bias_pos fail, ret = %d\n", ret);
	}else{
		ret = regulator_disable(disp_bias_pos);
		if (ret < 0)
			pr_err("disable regulator disp_bias_pos fail, ret = %d\n", ret);
	}
	retval |= ret;

	return retval;
}
EXPORT_SYMBOL(display_bias_vpos_enable);

int display_bias_vneg_enable(int enable)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	if (enable){
		/* enable regulator */
		ret = regulator_enable(disp_bias_neg);
		if (ret < 0)
			pr_err("enable regulator disp_bias_neg fail, ret = %d\n", ret);
	}else{
		ret = regulator_disable(disp_bias_neg);
		if (ret < 0)
			pr_err("disable regulator disp_bias_neg fail, ret = %d\n", ret);
	}
	retval |= ret;

	return retval;
}
EXPORT_SYMBOL(display_bias_vneg_enable);

int display_bias_vpos_set(int mv)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(disp_bias_pos, mv*1000, mv*1000);
	if (ret < 0)
		pr_err("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

#if 0
	/* get voltage */
	ret = mtk_regulator_get_voltage(&disp_bias_pos);
	if (ret < 0)
		pr_err("get voltage disp_bias_pos fail\n");
	pr_debug("pos voltage = %d\n", ret);

#endif

	return retval;
}
EXPORT_SYMBOL(display_bias_vpos_set);

int display_bias_vneg_set(int mv)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(disp_bias_neg, mv*1000, mv*1000);
	if (ret < 0)
		pr_err("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

#if 0
	/* get voltage */
	ret = mtk_regulator_get_voltage(&disp_bias_neg);
	if (ret < 0)
		pr_err("get voltage disp_bias_neg fail\n");
	pr_debug("neg voltage = %d\n", ret);

#endif

	return retval;
}
EXPORT_SYMBOL(display_bias_vneg_set);

#else
int display_bias_regulator_init(void)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_regulator_init);

int display_bias_enable(void)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_enable);

int display_bias_disable(void)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_disable);
int display_bias_vpos_enable(int enable)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_vpos_enable);

int display_bias_vneg_enable(int enable)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_vneg_enable);

int display_bias_vpos_set(int mv)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_vpos_set);

int display_bias_vneg_set(int mv)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_vneg_set);
#endif

#include <linux/of_gpio.h>
static char is_init_lcm_ldo_gpio = 0;
static int gpio_lcd_ldo18_pin = -1;
static int gpio_lcd_ldo28_pin = -1;
static char init_lcm_ldo_gpio_form_dtb(void){
	struct device_node *node;
	node = of_find_compatible_node(NULL,NULL,"prize,lcm_power_gpio");
	if (node){
		gpio_lcd_ldo18_pin = of_get_named_gpio(node,"gpio_lcd_ldo18_gpio",0);
		if (gpio_lcd_ldo18_pin < 0){
			printk(KERN_ERR"lcm_pmic get gpio_lcd_ldo18_pin fail %d\n",gpio_lcd_ldo18_pin);
		}else{
			gpio_request(gpio_lcd_ldo18_pin,"lcd_ldo18");
		}
		
		gpio_lcd_ldo28_pin = of_get_named_gpio(node,"gpio_lcd_ldo28_gpio",0);
		if (gpio_lcd_ldo28_pin < 0){
			printk(KERN_INFO"lcm_pmic get gpio_lcd_ldo28_pin fail %d\n",gpio_lcd_ldo28_pin);
		}else{
			gpio_request(gpio_lcd_ldo28_pin,"lcd_ldo28");
		}
		
		is_init_lcm_ldo_gpio = 1;
	}else{
		printk(KERN_ERR"lcm_pmic get of_node prize,lcm_power_gpio fail\n");
	}
	return -1;
}
#if 1
int display_ldo18_enable(int enable){
	int ret = 0;
	
	if (unlikely(!is_init_lcm_ldo_gpio)){
		init_lcm_ldo_gpio_form_dtb();
	}
	if (likely(gpio_lcd_ldo18_pin > 0)){
		if (enable){
			gpio_direction_output(gpio_lcd_ldo18_pin,1);
		}else{
			gpio_direction_output(gpio_lcd_ldo18_pin,0);
		}
	}
	return ret;
}
int display_ldo28_enable(int enable){
	int ret = 0;
	
	if (unlikely(!is_init_lcm_ldo_gpio)){
		init_lcm_ldo_gpio_form_dtb();
	}
	if (likely(gpio_lcd_ldo28_pin > 0)){
		if (enable){
			gpio_direction_output(gpio_lcd_ldo28_pin,1);
		}else{
			gpio_direction_output(gpio_lcd_ldo28_pin,0);
		}
	}
	return ret;
}
#else
int display_ldo18_enable(int enable){
	return 0;
}
int display_ldo28_enable(int enable){
	return 0;
}
#endif

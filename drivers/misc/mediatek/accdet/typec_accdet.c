/*
 * Copyright (C) 2016 MediaTek Inc.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/pm_wakeup.h>
#include <linux/reboot.h>
#include <linux/pm.h>
#include <linux/delay.h>

#include "tcpm.h"

#include <mt-plat/mtk_boot.h>

#include <mtk_auxadc_intf.h>
#include <mach/mtk_pmic.h>

#define EINT_PIN_PLUG_OUT       (0)
#define EINT_PIN_PLUG_IN        (1)

extern void accdet_eint_func_extern(int state);


static struct pinctrl *accdet_pinctrl;
static struct pinctrl_state *alp_state_h;
static struct pinctrl_state *alp_state_l;

static int mic_select_pin = 0;		/*SGM3798 0:FET2 1:FET1*/
static unsigned int mic_detect_thr = 0;

static struct tcpc_device *tcpc_dev;
static struct notifier_block audio_nb;
static char typec_eint_pending = 0;


int typec_accdet_mic_detect(void){
	//static int sel_pin_state = 0;
	unsigned int accdet_val;
	printk(KERN_INFO"typec_accdet typec_eint_pending(%d)\n",typec_eint_pending);
	if (!typec_eint_pending){	//check if typec headset event
		return 0;
	}
	typec_eint_pending = 0;
	
	if (mic_select_pin > 0){
		
		mdelay(2);
		accdet_val = pmic_get_auxadc_value(AUXADC_LIST_ACCDET);
		
		if (accdet_val <= mic_detect_thr){
			if (gpio_get_value(mic_select_pin)){
				gpio_direction_output(mic_select_pin,0);
			}else{
				gpio_direction_output(mic_select_pin,1);
			}
		}
		printk(KERN_INFO"typec_accdet AccdetVolt(%d) mic_pin reverse thr(%d)\n",accdet_val,mic_detect_thr);
	}
	return 0;
}
EXPORT_SYMBOL(typec_accdet_mic_detect);

static int audio_tcp_notifier_call(struct notifier_block *nb,
					unsigned long event, void *data)
{
	struct tcp_notify *noti = data;

	switch (event) {

	case TCP_NOTIFY_TYPEC_STATE:
		if (noti->typec_state.old_state == TYPEC_UNATTACHED && noti->typec_state.new_state == TYPEC_ATTACHED_AUDIO){
			pr_info("%s audio accessory Plug in, pol = %d\n", __func__,	noti->typec_state.polarity);
			typec_eint_pending = 1;
			accdet_eint_func_extern(EINT_PIN_PLUG_IN);
		}else if(noti->typec_state.old_state == TYPEC_ATTACHED_AUDIO && noti->typec_state.new_state == TYPEC_UNATTACHED){
			pr_info("%s audio accessory Plug out\n", __func__);
			accdet_eint_func_extern(EINT_PIN_PLUG_OUT);
		}
		break;
	default:
		//printk("HH event %u,oldstate %d,newstate %d\n",event,noti->typec_state.old_state,noti->typec_state.new_state);
		break;
	};
	return NOTIFY_OK;
}

static int typec_accdet_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct pinctrl_state *pin_default;
	struct device_node *node;
	
	//get pin
	accdet_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (!IS_ERR(accdet_pinctrl)){
		pin_default = pinctrl_lookup_state(accdet_pinctrl,"typec_accdet_default");
		if (IS_ERR(pin_default)){
			printk(KERN_ERR"typec_accdet get pinctrl state typec_accdet_default fail %d\n",PTR_ERR(pin_default));
		}
		alp_state_h = pinctrl_lookup_state(accdet_pinctrl,"typec_accdet_alp_h");
		if (IS_ERR(alp_state_h)){
			printk(KERN_ERR"typec_accdet get pinctrl state alp_h fail %d\n",PTR_ERR(alp_state_h));
		}
		alp_state_l = pinctrl_lookup_state(accdet_pinctrl,"typec_accdet_alp_l");
		if (IS_ERR(alp_state_l)){
			printk(KERN_ERR"typec_accdet get pinctrl state alp_l fail %d\n",PTR_ERR(alp_state_l));
		}
	}else{
		printk(KERN_ERR"typec_accdet get pinctrl fail %d\n",PTR_ERR(accdet_pinctrl));
		return -EINVAL;
	}
	if (!IS_ERR(pin_default)){
		pinctrl_select_state(accdet_pinctrl,pin_default);
	}
	

	//get mic settings
	node = pdev->dev.of_node;
	if (!IS_ERR(node)){
		//thr
		ret = of_property_read_u32(node,"mic_detect_thr",&mic_detect_thr);
		if (ret){
			mic_detect_thr = 230;
			printk(KERN_ERR"typec_accdet get mic_detect_thr fail %d, user default 300\n",ret);
		}
		
		//mic_select_pin
		mic_select_pin = of_get_named_gpio(node,"mic_select_pin",0);
		if (mic_select_pin < 0){
			printk(KERN_ERR"typec_accdet get mic_select_pin fail %d\n",mic_select_pin);
		}else{
			ret = gpio_request(mic_select_pin,"mic_select_pin");
			if (ret < 0){
				printk(KERN_ERR"typec_accdet gpio_request fail %d\n",ret);
				mic_select_pin = -1;
			}else{
				if (gpio_get_value(mic_select_pin)){
					gpio_direction_output(mic_select_pin,1);
				}else{
					gpio_direction_output(mic_select_pin,0);
				}
			}
		}
	}
	
	tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	if (!tcpc_dev) {
		pr_err("%s get tcpc device type_c_port0 fail\n", __func__);
		return -ENODEV;
	}

	audio_nb.notifier_call = audio_tcp_notifier_call;
	ret = register_tcp_dev_notifier(tcpc_dev, &audio_nb, TCP_NOTIFY_TYPEC_STATE);
	if (ret < 0) {
		pr_err("%s: register tcpc notifer fail\n", __func__);
		return -EINVAL;
	}

	pr_info("%s Done!!\n", __func__);
	return ret;
}

static int typec_accdet_remove(struct platform_device *pdev)
{
	if (mic_select_pin > 0){
		gpio_free(mic_select_pin);
	}
	return 0;
}

static const struct of_device_id typec_accdet_of_match[] = {
	{ .compatible = "prize,typec_accdet" },
	{ }
};
MODULE_DEVICE_TABLE(of, typec_accdet_of_match);

static struct platform_driver typec_accdet_driver = {
	.driver = {
		.name = "typec_accdet",
		.of_match_table = of_match_ptr(typec_accdet_of_match),
	},
	.probe = typec_accdet_probe,
	.remove = typec_accdet_remove,
};

static int __init typec_accdet_init(void)
{
	return platform_driver_register(&typec_accdet_driver);
}

static void __exit typec_accdet_exit(void)
{
	platform_driver_unregister(&typec_accdet_driver);
}

late_initcall(typec_accdet_init);
module_exit(typec_accdet_exit);

MODULE_AUTHOR("HH");
MODULE_LICENSE("GPL");

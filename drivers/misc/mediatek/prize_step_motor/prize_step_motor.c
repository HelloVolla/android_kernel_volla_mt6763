#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>

#include "prize_step_motor.h"

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#endif
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <mt-plat/mtk_pwm.h>
//prize  add  for proc/motor by zhuzhengjiang    20190612-start
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
//prize  add  for proc/motor by zhuzhengjiang    20190612-end
static struct pinctrl *dsi_pinctrl1;
static struct pinctrl_state *pins_default, *steppermotor_step_low, *steppermotor_step_high,*steppermotor_dir_low, *steppermotor_dir_high,*steppermotor_sleep_low, *steppermotor_sleep_high,*steppermotor_steppwm_mode,*steppermotor_enable_low,*steppermotor_enable_high;
static struct pinctrl_state *steppermotor_curren_m1_high,*steppermotor_curren_m1_low,*steppermotor_curren_m0_high,*steppermotor_curren_m0_low;
//prize  modify by zhuzhengjiang  for Opt k6309 hall parameters  20190110-begin
extern short  m1120_up_get_value(void);
extern short  m1120_down_get_value(void);
extern int m1120_down_state(void);
extern int m1120_up_state(void);
extern int calidata_flag;
static int state_down =-1;
static int  state_up =-1;
//prize  add  for proc/motor by zhuzhengjiang    20190612-start
static int  motor_enable =0;
static int  motor_state =0;
enum {
    MOTOR_STATE_START,
    MOTOR_STATE_UP_MOVING,
    MOTOR_STATE_UP_DONE,
    MOTOR_STATE_DOWN_MOVING,
    MOTOR_STATE_DOWN_DONE,
    MOTOR_STATE_MAX,
};
//prize  add  for proc/motor by zhuzhengjiang    20190612-end
//prize  modify by zhuzhengjiang  for Opt k6309 hall parameters  20190110-end
int stepper_motor_get_gpio_info(struct platform_device *pdev)
{

	int ret;

	printk("stepper_motor_get_gpio_info+++++++++++++++++\n");
	dsi_pinctrl1 = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(dsi_pinctrl1)) {
		ret = PTR_ERR(dsi_pinctrl1);
		dev_err(&pdev->dev, "hushilun Cannot find stepper_motor pinctrl1!\n");
             return -1;
	}
	pins_default = pinctrl_lookup_state(dsi_pinctrl1, "default");
	if (IS_ERR(pins_default)) {
		ret = PTR_ERR(pins_default);
		dev_err(&pdev->dev, "hushilun Cannot find stepper_motor pinctrl default %d!\n", ret);
	}
	steppermotor_step_low = pinctrl_lookup_state(dsi_pinctrl1, "step_low");
	if (IS_ERR(steppermotor_step_low)) {
		ret = PTR_ERR(steppermotor_step_low);
		dev_err(&pdev->dev, "hushilun Cannot find stepper_motor pinctrl step_low!\n");
	}
	steppermotor_step_high = pinctrl_lookup_state(dsi_pinctrl1, "step_high");
	if (IS_ERR(steppermotor_step_high)) {
		ret = PTR_ERR(steppermotor_step_high);
		dev_err(&pdev->dev, "hushilun Cannot find stepper_motor pinctrl step_high!\n");
	}

  steppermotor_dir_low = pinctrl_lookup_state(dsi_pinctrl1, "dir_low");
  if (IS_ERR(steppermotor_dir_low)) {
      ret = PTR_ERR(steppermotor_dir_low);
      dev_err(&pdev->dev, "hushilun Cannot find stepper_motor pinctrl dir_low!\n");
  }
  steppermotor_dir_high = pinctrl_lookup_state(dsi_pinctrl1, "dir_high");
  if (IS_ERR(steppermotor_dir_high)) {
      ret = PTR_ERR(steppermotor_dir_high);
      dev_err(&pdev->dev, "hushilun Cannot find stepper_motor pinctrl dir_high!\n");
  }
  
  steppermotor_sleep_low = pinctrl_lookup_state(dsi_pinctrl1, "sleep_low");
  if (IS_ERR(steppermotor_sleep_low)) {
      ret = PTR_ERR(steppermotor_sleep_low);
      dev_err(&pdev->dev, "hushilun Cannot find stepper_motor pinctrl sleep_low!\n");
  }
  steppermotor_sleep_high = pinctrl_lookup_state(dsi_pinctrl1, "sleep_high");
  if (IS_ERR(steppermotor_sleep_high)) {
      ret = PTR_ERR(steppermotor_sleep_high);
      dev_err(&pdev->dev, "hushilun Cannot find stepper_motor pinctrl sleep_high!\n");
  }
  steppermotor_steppwm_mode= pinctrl_lookup_state(dsi_pinctrl1, "step_pwm");
  if (IS_ERR(steppermotor_steppwm_mode)) {
      ret = PTR_ERR(steppermotor_steppwm_mode);
      dev_err(&pdev->dev, "hushilun Cannot find stepper_motor pinctrl steppermotor_steppwm_mode!\n");
  }

   steppermotor_enable_low = pinctrl_lookup_state(dsi_pinctrl1, "enable_low");
  if (IS_ERR(steppermotor_enable_low)) {
      ret = PTR_ERR(steppermotor_enable_low);
      dev_err(&pdev->dev, "hushilun Cannot find stepper_motor pinctrl sleep_low!\n");
  }

  steppermotor_enable_high = pinctrl_lookup_state(dsi_pinctrl1, "enable_high");
  if (IS_ERR(steppermotor_enable_high)) {
      ret = PTR_ERR(steppermotor_enable_high);
      dev_err(&pdev->dev, "hushilun Cannot find stepper_motor pinctrl sleep_high!\n");
  }

  steppermotor_curren_m1_high = pinctrl_lookup_state(dsi_pinctrl1, "curren_m1_high");
  if (IS_ERR(steppermotor_curren_m1_high)) {
      ret = PTR_ERR(steppermotor_curren_m1_high);
      dev_err(&pdev->dev, "hushilun Cannot find curren_m1 pinctrl dir_high!\n");
  }

    steppermotor_curren_m1_low = pinctrl_lookup_state(dsi_pinctrl1, "curren_m1_low");
  if (IS_ERR(steppermotor_curren_m1_low)) {
      ret = PTR_ERR(steppermotor_curren_m1_low);
      dev_err(&pdev->dev, "hushilun Cannot find curren_m1 pinctrl dir_low!\n");
  }

    steppermotor_curren_m0_high = pinctrl_lookup_state(dsi_pinctrl1, "curren_m0_high");
  if (IS_ERR(steppermotor_curren_m0_high)) {
      ret = PTR_ERR(steppermotor_curren_m0_high);
      dev_err(&pdev->dev, "hushilun Cannot find curren_m0 pinctrl dir_high!\n");
  }

    steppermotor_curren_m0_low = pinctrl_lookup_state(dsi_pinctrl1, "curren_m0_low");
  if (IS_ERR(steppermotor_curren_m0_low)) {
      ret = PTR_ERR(steppermotor_curren_m0_low);
      dev_err(&pdev->dev, "hushilun Cannot find curren_m0 pinctrl dir_low!\n");
  }
  
  pinctrl_select_state(dsi_pinctrl1, steppermotor_step_low); 
  pinctrl_select_state(dsi_pinctrl1, steppermotor_dir_low); 
  pinctrl_select_state(dsi_pinctrl1, steppermotor_sleep_low); 
  pinctrl_select_state(dsi_pinctrl1, steppermotor_steppwm_mode); 
  
 pinctrl_select_state(dsi_pinctrl1, steppermotor_enable_high); 
  
   pinctrl_select_state(dsi_pinctrl1, steppermotor_curren_m1_high); 
   pinctrl_select_state(dsi_pinctrl1, steppermotor_curren_m0_low); 
      return 0;

}

int stepper_motor_pinctrl_set(unsigned int pin , unsigned int level)
{
    int ret =-1;

	switch(pin)
	{
	case STEPPER_MOTOR_STEP_NO: 	
           if(IS_ERR(steppermotor_step_low)||IS_ERR(steppermotor_step_high))
           {
                pr_err( "err: steppermotor_step_low or steppermotor_step_high is error!!!");
                return ret;
           }   
           else
            break;
	case STEPPER_MOTOR_DIR_NO: 	
        if( IS_ERR(steppermotor_dir_low)|| IS_ERR(steppermotor_dir_high))
        {
             pr_err("err: steppermotor_step_low or steppermotor_step_low is error!!!");
             return ret;
        }   
        else
         break;
	case STEPPER_MOTOR_SLEEP_NO: 	
        if( IS_ERR(steppermotor_sleep_low)|| IS_ERR(steppermotor_sleep_high))
        {
             pr_err("err: steppermotor_sleep_low or steppermotor_sleep_high is error!!!");
             return ret;
        }   
        else
         break;
		
	case STEPPER_MOTOR_ENABLE_NO: 	
        if( IS_ERR(steppermotor_enable_low)|| IS_ERR(steppermotor_enable_high))
        {
             pr_err("err: steppermotor_sleep_low or steppermotor_sleep_high is error!!!");
             return ret;
        }   
        else
         break;
	default:
          {
              pr_err("err: stepper_motor_pinctrl_set pin[%d] is error!!, drv need to check!", pin);
              return -1;
           }
	}
      

	switch(pin)
	{
	case STEPPER_MOTOR_STEP_NO: 	pinctrl_select_state(dsi_pinctrl1, level ? steppermotor_step_high      : steppermotor_step_low);      break; //pinctrl_select_state(dsi_pinctrl1, steppermotor_step_high);      break; //
	case STEPPER_MOTOR_DIR_NO: 	pinctrl_select_state(dsi_pinctrl1, level ? steppermotor_dir_high  : steppermotor_dir_low);  break; 
	//case STEPPER_MOTOR_SLEEP_NO: 	pinctrl_select_state(dsi_pinctrl1, steppermotor_sleep_high);  break; //pinctrl_select_state(dsi_pinctrl1, level ? steppermotor_sleep_high  : steppermotor_sleep_low);  break;
	case STEPPER_MOTOR_SLEEP_NO: 	pinctrl_select_state(dsi_pinctrl1, level ? steppermotor_sleep_high  : steppermotor_sleep_low);  break;
	case STEPPER_MOTOR_ENABLE_NO: 	pinctrl_select_state(dsi_pinctrl1, level ? steppermotor_enable_high  : steppermotor_enable_low);  break; 
	default: ;
	}
      return 0;
}
static struct hrtimer stepper_motor_timer;
int stepper_motor_onoff(unsigned int on_off);
#if 1
static int stepper_motor_timer_start(ktime_t ktime)
{
	hrtimer_start(&stepper_motor_timer, ktime, HRTIMER_MODE_REL);

	return 0;
}

static int stepper_motor_timer_cancel(void)
{

	hrtimer_cancel(&stepper_motor_timer);
	return 0;
}
#endif
static enum hrtimer_restart stepper_motor_timer_func(struct hrtimer *timer)
{
//	schedule_work(&stepper_motor_timer_func);
	stepper_motor_pinctrl_set(STEPPER_MOTOR_SLEEP_NO,1);
	printk("hushilun: stepper_motor_timer_func test\n");
//	stepper_motor_timer_cancel();
	return HRTIMER_NORESTART;
}




int mt_stepper_motor_set_pwm(int pwm_num,int on_off)
{
	struct pwm_spec_config pwm_setting;
//	int time_index = 0;

	memset(&pwm_setting, 0, sizeof(struct pwm_spec_config));
	pwm_setting.pwm_no = pwm_num;
	pwm_setting.mode = PWM_MODE_OLD;

	printk("mt_stepper_motor_set_pwm: on_off=%d,pwm_no=%d\n", on_off,
		   pwm_num);
	/* We won't choose 32K to be the clock src of old mode because of system performance. */
	/* The setting here will be clock src = 26MHz, CLKSEL = 26M/1625 (i.e. 16K) */
	pwm_setting.clk_src = PWM_CLK_OLD_MODE_32K;
	pwm_setting.pmic_pad = 0;

	switch (on_off) {
	/* Actually, the setting still can not to turn off NLED. We should disable PWM to turn off NLED. */
	case 0:
		pwm_setting.PWM_MODE_OLD_REGS.THRESH = 0;
		pwm_setting.clk_div = CLK_DIV1;
		pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = 100 / 2;
		break;
	case 1:
		pwm_setting.PWM_MODE_OLD_REGS.THRESH = 5; //4
		pwm_setting.clk_div = CLK_DIV1;
		pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = 6; //8
		break;

	default:
		printk("Invalid nled mode\n");
		return -1;
	}

	pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.GDURATION = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM = 0;
	pwm_set_spec_config(&pwm_setting);

	return 0;
}

static int dir_in_out=0;
//prize  modify by zhuzhengjiang  for Opt k6309 hall parameters  20190110-begin
int stepper_motor_onoff(unsigned int on_off)
{
	int step_count=0;
	ktime_t ktime;
	//short hall_up_data =0;
	//short hall_down_data =0;
	int time_out_index = 0;
	stepper_motor_pinctrl_set(STEPPER_MOTOR_STEP_NO,1);
	pinctrl_select_state(dsi_pinctrl1, steppermotor_steppwm_mode);

	pinctrl_select_state(dsi_pinctrl1, steppermotor_curren_m1_low);
	pinctrl_select_state(dsi_pinctrl1, steppermotor_curren_m0_low);
	pinctrl_select_state(dsi_pinctrl1, steppermotor_enable_low);

	printk("stepper_motor_onoff test0  on_off=%d \n",on_off);
	motor_state = MOTOR_STATE_START;
	if(1==on_off)
	{
	    printk("hushilun: stepper_motor_pinctrl_set on \n");
	    dir_in_out=!dir_in_out;
	    stepper_motor_pinctrl_set(STEPPER_MOTOR_ENABLE_NO,0);
	    stepper_motor_pinctrl_set(STEPPER_MOTOR_SLEEP_NO,1);
	    stepper_motor_pinctrl_set(STEPPER_MOTOR_DIR_NO,1);
	    stepper_motor_timer_cancel();
	    mt_stepper_motor_set_pwm(0,1);

	    ktime = ktime_set(1,700*1000*1000);
	    stepper_motor_timer_start(ktime);
	    time_out_index = 0;
	    while(1)  {
	         time_out_index++;
	         //hall_up_data =m1120_up_get_value();
	         //mdelay(5);
	         state_up = m1120_up_state();
	         //printk("motor up  hall_up_data=%d time_out_index=%d state_up=%d \n",hall_up_data,time_out_index,state_up);
	         if((state_up == 1 &&  calidata_flag == 0) ||time_out_index > 55 )
	         {
	             motor_state =MOTOR_STATE_UP_DONE;
	             break;
	         }
	         else {
	             motor_state =MOTOR_STATE_UP_MOVING;
	         }
	    }

	    //mdelay(2000);
	    stepper_motor_pinctrl_set(STEPPER_MOTOR_ENABLE_NO,1);
	    stepper_motor_pinctrl_set(STEPPER_MOTOR_SLEEP_NO,0);
	         printk("motor down  up end  \n");
	}
	else
	{
	    stepper_motor_pinctrl_set(STEPPER_MOTOR_ENABLE_NO,0);
	    stepper_motor_pinctrl_set(STEPPER_MOTOR_SLEEP_NO,1);
	    stepper_motor_pinctrl_set(STEPPER_MOTOR_DIR_NO,0);

	    stepper_motor_timer_cancel();
	    mt_stepper_motor_set_pwm(0,1);
	    time_out_index = 0;
	    ktime = ktime_set(1,700*1000*1000);
	    stepper_motor_timer_start(ktime);
	    while(1)  {
	        time_out_index ++;
	       //hall_down_data =m1120_down_get_value();
	        //mdelay(5);
	        state_down = m1120_down_state();
	        //printk("motor down hall_down_data=%d  time_out_index =%d  state_down=%d\n",hall_down_data,time_out_index,state_down);
	        if((state_down == 1 &&  calidata_flag == 0) ||time_out_index > 55 )
	        {
	             motor_state =MOTOR_STATE_DOWN_DONE;
	             break;
	         }
	         else {
	             motor_state =MOTOR_STATE_DOWN_MOVING;
	         }
	    }

	    stepper_motor_pinctrl_set(STEPPER_MOTOR_ENABLE_NO,1);
	    stepper_motor_pinctrl_set(STEPPER_MOTOR_SLEEP_NO,0);
	}
	step_count=0;

	stepper_motor_pinctrl_set(STEPPER_MOTOR_STEP_NO,1);

	return 0;
}
//prize  modify by zhuzhengjiang  for Opt k6309 hall parameters 20190110-end
EXPORT_SYMBOL_GPL(stepper_motor_onoff);
//prize  add  for proc/motor by zhuzhengjiang    20190612-start
static struct proc_dir_entry *step_motor_proc_entry;

static int step_motor_enable_read_proc(struct seq_file *m, void *v)
{
    //char value[2] = {0};
    seq_printf(m, "%d", motor_enable);

    //printk("step_motor_read_proc step_motor:%d\n", raw);
    return 0;
}

static int step_motor_enable_open(struct inode *inode, struct file *file)
{
    return single_open(file, step_motor_enable_read_proc, inode->i_private);
}

static ssize_t step_motor_enable_write_proc(struct file *filp, const char __user *buff, size_t count, loff_t *data)
{
	char temp[8] = {0};
	int len = 0;
	//int ret = 0;

	if (step_motor_proc_entry == NULL) {
		printk("step_motor_proc_entry pointer null \n");
		return -1;
	}

	len = (count < (sizeof(temp) - 1)) ? count : (sizeof(temp) - 1);
	if (copy_from_user(temp, buff, len))
		return 0;
	
       motor_enable =temp[0]-'0';
      if(motor_enable==1)
	  	stepper_motor_onoff(1);
     else 
	 	stepper_motor_onoff(0);
	return count;
}

static const struct file_operations step_motor_enable = {
	.owner = THIS_MODULE,
	.open = step_motor_enable_open,
	.write = step_motor_enable_write_proc,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
//prize  add  for proc/motor/step_motor_state by zhuzhengjiang    20190612-start
static int step_motor_state_read_proc(struct seq_file *m, void *v)
{
    printk("step_motor_state_read_proc motor_state%d \n",motor_state);
    seq_printf(m, "%d", motor_state);

    return 0;
}

static int step_motor_state_open(struct inode *inode, struct file *file)
{
    return single_open(file, step_motor_state_read_proc, inode->i_private);
}

static ssize_t step_motor_state_write_proc(struct file *filp, const char __user *buff, size_t count, loff_t *data)
{
	return count;
}
static const struct file_operations step_motor_state = {
	.owner = THIS_MODULE,
	.open = step_motor_state_open,
	.write = step_motor_state_write_proc,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
//prize  add  for proc/motor/step_motor_enable by zhuzhengjiang    20190612-end
int step_motor_node_create(void)
{
	step_motor_proc_entry = proc_mkdir("motor", NULL);
	if (step_motor_proc_entry == NULL) {
		printk("create motor_proc   FAILED!");
		return -1;
	}

	proc_create("step_motor_enable", 0664, step_motor_proc_entry, &step_motor_enable);

	printk("create_proc_entry  step_motor SUCCESS.");
	proc_create("step_motor_state", 0664, step_motor_proc_entry, &step_motor_state);
	return 0;
}
//prize  add  for proc/motor by zhuzhengjiang    20190612-end
static int stepper_motor_probe(struct platform_device *dev)
{
	printk("panf stepper_motor_gpio_probe begin\n");
	stepper_motor_get_gpio_info(dev);
	hrtimer_init(&stepper_motor_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stepper_motor_timer.function = stepper_motor_timer_func;
	return 0;
}
static const struct of_device_id stepper_motor_gpio_of_ids[] = {
	{.compatible = "mediatek,prize_stepper_motor",},
	{}
};

static struct platform_driver stepper_motor_gpio_driver = {
	.driver = {
		   .name = "prize_stepper_motor",
	#ifdef CONFIG_OF
		   .of_match_table = stepper_motor_gpio_of_ids,
	#endif
		   },
	.probe = stepper_motor_probe,
};


static int __init stepper_motor_gpio_init(void)
{

	printk("panf stepper_motor GPIO driver init\n");
	if (platform_driver_register(&stepper_motor_gpio_driver) != 0) {
		pr_err("unable to register stepper_motor GPIO driver.\n");
		return -1;
	}
	//prize  add  for proc/motor by zhuzhengjiang    20190612-start
	step_motor_node_create();
	//prize  add  for proc/motor by zhuzhengjiang    20190612-end
	return 0;
}

static void __exit stepper_motor_gpio_exit(void)
{
	printk("panf stepper_motor GPIO driver exit\n");
	platform_driver_unregister(&stepper_motor_gpio_driver);

}
module_init(stepper_motor_gpio_init);
module_exit(stepper_motor_gpio_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("prize stepper_motor GPIO driver");
MODULE_AUTHOR("Pan fei<panfei@szprize.com>");

/**************************************************************
dual hall dirver
by - gezi 2020/08/25
***************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#if defined(CONFIG_PM_WAKELOCKS)
#include <linux/pm_wakeup.h>
#else
#include <linux/wakelock.h>
#endif


#define HALL_NEAR        (1)
#define HALL_FAR       	 (0)

#define HALL_DEVNAME    "hall_dev"

static unsigned int g_cur_state_hall_1 = HALL_FAR;
static unsigned int g_cur_state_hall_2 = HALL_FAR;

unsigned int eint_hall_1 = 0;
unsigned int eint_hall_2 = 0;

int irq_hall_1;
int irq_hall_2;

unsigned int eint_type_hall_1;
unsigned int eint_type_hall_2;

static struct work_struct eint_work_hall_1;
static struct workqueue_struct *eint_workqueue_hall_1;

static struct work_struct eint_work_hall_2;
static struct workqueue_struct *eint_workqueue_hall_2;


//static dev_t g_hall_devno;
//static struct cdev *g_hall_cdev = NULL;
//static struct class *hall_class = NULL;
//static struct device *hall_nor_device = NULL;
static struct input_dev *hall_input_dev;
static struct kobject *g_hall_sys_device;
static int hall_key_event = 0;

//prize modified by huarui, update with kernel version 20200408 start
#if defined(CONFIG_PM_WAKELOCKS)
struct wakeup_source hall_key_lock;
#else
struct wake_lock hall_key_lock;
#endif
//prize modified by huarui, update with kernel version 20200408 end

static atomic_t send_event_flag = ATOMIC_INIT(0);
static DECLARE_WAIT_QUEUE_HEAD(send_event_wq);

static int HALL_DEBUG_enable = 1;
#define HALL_DEBUG(format, args...) do { \
	if (HALL_DEBUG_enable) \
	{\
		printk(KERN_WARNING format, ##args);\
	} \
} while (0)
	
static const struct of_device_id hall_id[] = {
	{.compatible = "mediatek,hall"},
	{},
};
MODULE_DEVICE_TABLE(of, hall_id);


static ssize_t notify_sendKeyEvent(int event)
{
    hall_key_event = event;
    atomic_set(&send_event_flag, 1);
    wake_up(&send_event_wq);
    HALL_DEBUG("[hall_dev]:notify_sendKeyEvent !\n");
    return 0;
}


static void work_callback_hall_1(struct work_struct *work)
{
	if(g_cur_state_hall_1 ==  HALL_NEAR) 
	{
		HALL_DEBUG("1 HALL_NEAR\n");
		
		if( __gpio_get_value(eint_hall_1) == 0)
		{
			HALL_DEBUG("1 gezi HALL_NEAR...\n");
			notify_sendKeyEvent(HALL_NEAR);
		}
	}	
	else
	{
		HALL_DEBUG("1 HALL_FAR\n");
	
		if( __gpio_get_value(eint_hall_1) == 1)
		{
			HALL_DEBUG("1 gezi HALL_FAR...\n");
			notify_sendKeyEvent(HALL_FAR);
		}
	}	

	enable_irq(irq_hall_1);
}

static irqreturn_t eint_func_hall_1(int irq,void *data)
{

	HALL_DEBUG("[eint_func_hall_1] enter,g_cur_state_hall_1=%d,eint_type_hall_1=%d\n",g_cur_state_hall_1,eint_type_hall_1);

	disable_irq_nosync(irq_hall_1);

	g_cur_state_hall_1 = !g_cur_state_hall_1;
	
	if (eint_type_hall_1 == IRQ_TYPE_LEVEL_LOW)
	{
		HALL_DEBUG("eint_func_hall_1 set IRQ_TYPE_LEVEL_HIGH!!!\n");
		eint_type_hall_1 = IRQ_TYPE_LEVEL_HIGH;
	}
	else
	{
		HALL_DEBUG("eint_func_hall_1 set IRQ_TYPE_LEVEL_LOW!!!\n");
		eint_type_hall_1 = IRQ_TYPE_LEVEL_LOW;
	}

	irq_set_irq_type(irq_hall_1, eint_type_hall_1);
	
	queue_work(eint_workqueue_hall_1, &eint_work_hall_1);
				
	return IRQ_HANDLED;	
	
}


static void work_callback_hall_2(struct work_struct *work)
{
	if(g_cur_state_hall_2 ==  HALL_NEAR) 
	{
		HALL_DEBUG("2 HALL_NEAR\n");
		
		if( __gpio_get_value(eint_hall_2) == 0)
		{
			HALL_DEBUG(" 2 gezi HALL_NEAR...\n");
			notify_sendKeyEvent(HALL_NEAR);
		}
	}	
	else
	{
		HALL_DEBUG("2 HALL_FAR\n");
	
		if( __gpio_get_value(eint_hall_2) == 1)
		{
			HALL_DEBUG("2 gezi HALL_FAR...\n");
			notify_sendKeyEvent(HALL_FAR);
		}
	}	

	enable_irq(irq_hall_2);
}

static irqreturn_t eint_func_hall_2(int irq,void *data)
{

	HALL_DEBUG("[eint_func_hall_2] enter,g_cur_state_hall_1=%d,eint_type_hall_1=%d\n",g_cur_state_hall_1,eint_type_hall_1);

	disable_irq_nosync(irq_hall_2);

	g_cur_state_hall_2 = !g_cur_state_hall_2;
	
	if (eint_type_hall_2 == IRQ_TYPE_LEVEL_LOW)
	{
		HALL_DEBUG("eint_func_hall_2 set IRQ_TYPE_LEVEL_HIGH!!!\n");
		eint_type_hall_2 = IRQ_TYPE_LEVEL_HIGH;
	}
	else
	{
		HALL_DEBUG("eint_func_hall_2 set IRQ_TYPE_LEVEL_LOW!!!\n");
		eint_type_hall_2 = IRQ_TYPE_LEVEL_LOW;
	}

	irq_set_irq_type(irq_hall_2, eint_type_hall_2);
	
	queue_work(eint_workqueue_hall_2, &eint_work_hall_2);
				
	return IRQ_HANDLED;	
	
}

static inline int hall_setup_eint(struct platform_device *dev) 
{
#if 1
	int ret;
	struct device_node *node;

	eint_type_hall_1 = IRQ_TYPE_LEVEL_LOW;
	eint_type_hall_2 = IRQ_TYPE_LEVEL_LOW;
	
	node = of_find_compatible_node(NULL, NULL, "mediatek,hall");

	eint_hall_1 = of_get_named_gpio(node, "eint_hall_1", 0);
	
	eint_hall_2 = of_get_named_gpio(node, "eint_hall_2", 0);
	
	HALL_DEBUG("eint_hall1 = %d,eint_hall_2 = %d\n",eint_hall_1,eint_hall_2);
	
	ret = gpio_request(eint_hall_1,"eint_hall_1");
	if (ret < 0) {
			HALL_DEBUG("Unable to request gpio eint_hall1\n");
	}
	
	
	ret = gpio_request(eint_hall_2,"eint_hall_2");
	if (ret < 0) {
			HALL_DEBUG("Unable to request gpio irq_hall_2\n");
	}
	
	gpio_direction_input(eint_hall_1);	
	gpio_direction_input(eint_hall_2);	
	
	irq_hall_1 = gpio_to_irq(eint_hall_1);
	irq_hall_2 = gpio_to_irq(eint_hall_2);
		
	if (node) 
	{
		//irq_hall_1 = irq_of_parse_and_map(node, 0);
		HALL_DEBUG("irq_hall_1=%d ,irq_hall_2 = %d\n", irq_hall_1,irq_hall_2);
		ret = request_irq(irq_hall_1, eint_func_hall_1,IRQ_TYPE_LEVEL_LOW, "irq_hall_1", NULL);	
		if (ret > 0)
			HALL_DEBUG("irq_hall_1 EINT IRQ LINE NOT AVAILABLE\n");
		else {
			HALL_DEBUG("irq_hall_1 eint set EINT finished, irq_hall_1=%d\n",irq_hall_1);
		}
		
		ret = request_irq(irq_hall_2, eint_func_hall_2,IRQ_TYPE_LEVEL_LOW, "irq_hall_2", NULL);	
		
		if (ret > 0)
			HALL_DEBUG("irq_hall_2 EINT IRQ LINE NOT AVAILABLE\n");
		else {
			HALL_DEBUG("irq_hall_2 eint set EINT finished, irq_hall_2=%d\n",irq_hall_2);
		}
		
		enable_irq_wake(irq_hall_1);
		enable_irq_wake(irq_hall_2);	

	}
#else

#endif
	return 0;
}

static ssize_t hall1_status_info_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	HALL_DEBUG("[hall_dev] g_cur_state_hall_1=%d\n", g_cur_state_hall_1);
	return sprintf(buf, "%u\n", g_cur_state_hall_1);
}

static ssize_t hall1_status_info_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
	HALL_DEBUG("[hall_dev] %s ON/OFF value = %d:\n ", __func__, g_cur_state_hall_1);

	/*if(sscanf(buf, "%u", &g_cur_state_hall_1) != 1)
	{
		HALL_DEBUG("[hall_dev]: Invalid values\n");
		return -EINVAL;
	}*/
	return size;
}

static DEVICE_ATTR(hall_status1, 0644, hall1_status_info_show,  hall1_status_info_store);

static ssize_t hall2_status_info_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	HALL_DEBUG("[hall_dev] g_cur_state_hall_2=%d\n", g_cur_state_hall_2);
	return sprintf(buf, "%u\n", g_cur_state_hall_2);
}

static ssize_t hall2_status_info_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
	HALL_DEBUG("[hall_dev] %s ON/OFF value = %d:\n ", __func__, g_cur_state_hall_1);

	/*if(sscanf(buf, "%u", &g_cur_state_hall_1) != 1)
	{
		HALL_DEBUG("[hall_dev]: Invalid values\n");
		return -EINVAL;
	}*/
	return size;
}

static DEVICE_ATTR(hall_status2, 0644, hall2_status_info_show,  hall2_status_info_store);


static int sendKeyEvent(void *unuse)
{
    while(1)
    {
        HALL_DEBUG("[hall_dev]:sendKeyEvent wait\n");
        //wait for signal
        wait_event_interruptible(send_event_wq, (atomic_read(&send_event_flag) != 0));

//prize modified by huarui, update with kernel version 20200408 start
	#if defined(CONFIG_PM_WAKELOCKS)
		__pm_wakeup_event(&hall_key_lock, 2*HZ);
	#else
        wake_lock_timeout(&hall_key_lock, 2*HZ);    //set the wake lock.
	#endif
//prize modified by huarui, update with kernel version 20200408 end
        HALL_DEBUG("[hall_dev]:going to send event %d\n", hall_key_event);

        //send key event
        if(HALL_NEAR == hall_key_event)
        {
            HALL_DEBUG("[hall_dev]:HALL_OPEN!\n");
		/*prize-lixuefeng-20150602-start*/
            input_report_key(hall_input_dev, KEY_F8, 1);
            input_report_key(hall_input_dev, KEY_F8, 0);
		/*prize-lixuefeng-20150602-end*/
            input_sync(hall_input_dev);
        }
	    else if(HALL_FAR == hall_key_event)
        {
            HALL_DEBUG("[hall_dev]:HALL_CLOSE!\n");
		/*prize-lixuefeng-20150602-start*/
            input_report_key(hall_input_dev, KEY_F7, 1);
            input_report_key(hall_input_dev, KEY_F7, 0);
		/*prize-lixuefeng-20150602-end*/
            input_sync(hall_input_dev);
        }
        atomic_set(&send_event_flag, 0);
    }
    return 0;
}

static int hall_input_dev_register(void)
{
	int ret = 0;
	struct task_struct *keyEvent_thread = NULL;
	
	hall_input_dev = input_allocate_device();
	
    if (!hall_input_dev)
    {
		HALL_DEBUG("[hall_dev]:hall_input_dev : fail!\n");
        return -ENOMEM;
    }
	
	__set_bit(EV_KEY, hall_input_dev->evbit);
    __set_bit(KEY_F8, hall_input_dev->keybit);
    __set_bit(KEY_F7, hall_input_dev->keybit);

	hall_input_dev->id.bustype = BUS_HOST;
	hall_input_dev->name = HALL_DEVNAME;
	if(input_register_device(hall_input_dev))
	{
		HALL_DEBUG("[hall_dev]:hall_input_dev register : fail!\n");
	}
	else
	{
		HALL_DEBUG("[hall_dev]:hall_input_dev register : success!!\n");
	}
	//prize modified by huarui, update with kernel version 20200408 start
	#if defined(CONFIG_PM_WAKELOCKS)
		wakeup_source_init(&hall_key_lock, "hall key wakelock");
	#else
		wake_lock_init(&hall_key_lock, WAKE_LOCK_SUSPEND, "hall key wakelock");
	#endif
	//prize modified by huarui, update with kernel version 20200408 end
	
	init_waitqueue_head(&send_event_wq);
    //start send key event thread
    keyEvent_thread = kthread_run(sendKeyEvent, 0, "keyEvent_send");
    if (IS_ERR(keyEvent_thread)) 
    { 
       ret = PTR_ERR(keyEvent_thread);
       HALL_DEBUG("[hall_dev]:failed to create kernel thread: %d\n", ret);
	   return ret;
    }
	
	return 0;
}

static int hall_add_8804(void)
{
	int ret = 0;

    g_hall_sys_device = kobject_create_and_add("hall_state", NULL);
    if (g_hall_sys_device == NULL)
	{
        HALL_DEBUG("[hall_dev]:%s: subsystem_register failed\n", __func__);
        ret = -ENXIO;
        return ret ;
	}
	
	ret = sysfs_create_file(g_hall_sys_device, &dev_attr_hall_status1.attr);
	if (ret) 
	{
      	HALL_DEBUG("[hall_dev]:%s: sysfs_create_file failed\n", __func__);
       	goto err;
   	}
	
	ret = sysfs_create_file(g_hall_sys_device, &dev_attr_hall_status2.attr);
	if (ret) 
	{
      	HALL_DEBUG("[hall_dev]:%s: sysfs_create_file failed\n", __func__);
       	goto err;
   	}
	
    return 0;

err: 
	kobject_del(g_hall_sys_device);
	return -1;
}

static int hall_probe(struct platform_device *dev)
{
	/*int r=0;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_int;*/
	
	HALL_DEBUG("[hall_probe]hall_probe!\n");
	
	hall_setup_eint(dev);
	
	eint_workqueue_hall_1 = create_singlethread_workqueue("eint_hall_1");
	INIT_WORK(&eint_work_hall_1, work_callback_hall_1);
	
	eint_workqueue_hall_2 = create_singlethread_workqueue("eint_hall_2");
	INIT_WORK(&eint_work_hall_2, work_callback_hall_2);
	
	hall_input_dev_register();
	
	hall_add_8804();
	
	device_init_wakeup(&dev->dev, 1);	

	
	return 0;
}

static int hall_remove(struct platform_device *dev)
{
	//touch_wakeup_remove();
	destroy_workqueue(eint_workqueue_hall_1);
	destroy_workqueue(eint_workqueue_hall_2);
	//wake_unlock(&usb_lock);
	return 0;
}

static int hall_suspend(struct platform_device *pdev, pm_message_t state)
{
	pr_debug("Enter %s\n",__FUNCTION__);

        if (device_may_wakeup(&pdev->dev))
		{
                enable_irq_wake(irq_hall_1);
				enable_irq_wake(irq_hall_2);
		}

        /*
         * IRQ must be disabled during suspend because if it happens
         * while suspended it will be handled before resuming I2C.
         *
         * When device is woken up from suspend (e.g. by RTC wake alarm),
         * an interrupt occurs before resuming I2C bus controller.
         * Interrupt handler tries to read registers but this read
         * will fail because I2C is still suspended.
         */
        disable_irq(irq_hall_1);
		disable_irq(irq_hall_2);
		
	return 0;
}

static int hall_resume(struct platform_device *pdev)
{
	pr_debug("Enter %s\n",__FUNCTION__);

        if (device_may_wakeup(&pdev->dev))
		{
            disable_irq_wake(irq_hall_1);
			disable_irq_wake(irq_hall_2);
		}

        enable_irq(irq_hall_1);
		enable_irq(irq_hall_2);

	return 0;
}

static struct platform_driver hall_driver = {
	.probe = hall_probe,
	.suspend = hall_suspend, 
	.resume = hall_resume,
	.remove = hall_remove,
	.driver = {
		   .name = "hall-as1800",
		   .of_match_table = of_match_ptr(hall_id),	
	},
};

static int hall_mod_init(void)
{
	int ret = 0;

	HALL_DEBUG("hall_mod_init\n");

	ret = platform_driver_register(&hall_driver);
	if (ret) {
		HALL_DEBUG("hall_mod_init driver register error:(%d)\n", ret);
		return ret;
	} else {
		HALL_DEBUG("usb eint platform driver register done!\n");
	}

	HALL_DEBUG("[usb eint]hall_mod_init done!\n");
	return 0;

}

static void hall_mod_exit(void)
{
	HALL_DEBUG("hall_mod_exit\n");
	platform_driver_unregister(&hall_driver);

}

late_initcall_sync(hall_mod_init);
module_exit(hall_mod_exit);

MODULE_DESCRIPTION("prize hall driver");
MODULE_AUTHOR("zhaopengge <zhaopengge@szprize.com>");
MODULE_LICENSE("GPL");

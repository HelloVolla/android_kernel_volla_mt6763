/**************************************************************************
*  dts example
*	spc@21 {
*		compatible = "prize,spc_r_1";
*		reg = <0x21>;
*		pdn_pin = <&pio 20 0>;
*		sensor_id = <0x232a>;
*	};
**************************************************************************/

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include "prize_dual_cam.h"


static struct spc_data_t spc_r_devtype = {
	.name = "rear",
	.is_enabled = 0,
	.sensor_type = SENSOR_TYPE_UNKNOWN,
	.pdn_pin = 0,
	.rst_pin = 0,
};

static struct spc_data_t spc_r_1_devtype = {
	.name = "rear_1",
	.is_enabled = 0,
	.sensor_type = SENSOR_TYPE_UNKNOWN,
	.pdn_pin = 0,
	.rst_pin = 0,
};

static struct spc_data_t spc_f_devtype = {
	.name = "front",
	.is_enabled = 0,
	.sensor_type = SENSOR_TYPE_UNKNOWN,
	.pdn_pin = 0,
	.rst_pin = 0,
};

extern struct sensor_info_t gc032a_info;
extern struct sensor_info_t gc0310_info;
extern struct sensor_info_t ov5645_info;
static struct sensor_info_t *sensor_list[] = {&gc0310_info,&gc032a_info,&ov5645_info};

static struct kobject *spc_kobj = NULL;

static int detect_sensor_type(struct i2c_client *client){
	int i;
	struct spc_data_t *spc_data = i2c_get_clientdata(client);
	
	for(i = 0;i < sizeof(sensor_list)/sizeof(sensor_list[0]);i++){
		if (sensor_list[i]->match(client)){
			spc_data->sensor_type = sensor_list[i]->sensor_type;
			spc_data->ops = sensor_list[i];
			break;
		}
	}
	
	if (spc_data->sensor_type == SENSOR_TYPE_AUTO){
		spc_data->sensor_type = SENSOR_TYPE_UNKNOWN;
		spc_data->ops = sensor_list[0];
		return -EINVAL;
	}
	return 0;
}

static ssize_t device_open_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct spc_data_t *spc_data = i2c_get_clientdata(client);
	
	int ret = -1;

	//mdelay(100);
	mutex_lock(&spc_data->ops_mutex);
	if (!spc_data->is_enabled){
		
		if (spc_data->sensor_type == SENSOR_TYPE_AUTO){
			detect_sensor_type(client);
		}
		
		spc_data->ops->set_power(client,1);
		ret = spc_data->ops->open(client);
		if (ret){
			spc_data->ops->set_power(client,0);
			mutex_unlock(&spc_data->ops_mutex);
			printk(KERN_ERR"SPC rear_open fail %d\n",ret);
			return sprintf(buf, "%d\n", ret);
		}
		spc_data->is_enabled = 1;
	}
	mutex_unlock(&spc_data->ops_mutex);
	printk(KERN_INFO"SPC rear_open %d\n",spc_data->is_enabled);
	return sprintf(buf, "%d\n", 0);
}
static DEVICE_ATTR(open, 0644, device_open_show, NULL);

static ssize_t shutter_value_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct spc_data_t *spc_data = i2c_get_clientdata(client);
	int shutter_value = 0;
	
	if (unlikely(!spc_data->is_enabled)){
		return sprintf(buf, "%d\n", -1);
	}

	shutter_value=spc_data->ops->get_shutter(client);
	return sprintf(buf, "%d\n", shutter_value);
}
static DEVICE_ATTR(value, 0644, shutter_value_show, NULL);


static ssize_t device_close_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct spc_data_t *spc_data = i2c_get_clientdata(client);

	mutex_lock(&spc_data->ops_mutex);
	if (spc_data->is_enabled){
		spc_data->ops->set_power(client,0);
		spc_data->is_enabled = 0;
	}
	mdelay(10);
	mutex_unlock(&spc_data->ops_mutex);
	printk(KERN_INFO"SPC rear_close %d\n",spc_data->is_enabled);
	return sprintf(buf, "%d\n", 0);
}
static DEVICE_ATTR(close, 0644, device_close_show, NULL);

static ssize_t device_type_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct spc_data_t *spc_data = i2c_get_clientdata(client);
	int ret = 0;

	switch(spc_data->sensor_type){
		case SENSOR_TYPE_UNKNOWN:
		case SENSOR_TYPE_AUTO: ret = sprintf(buf,"%s\n","UNKNOWN"); break;
		default : ret = sprintf(buf,"%s\n",spc_data->ops->name); break;
	}
	return ret;
}
static DEVICE_ATTR(type, 0644, device_type_show, NULL);

static const struct of_device_id __maybe_unused spc_of_match[] = {
	{ .compatible = "prize,spc_r", .data = &spc_r_devtype },
	{ .compatible = "prize,spc_f", .data = &spc_f_devtype },
	{ .compatible = "prize,spc_r_1", .data = &spc_r_1_devtype },
};
MODULE_DEVICE_TABLE(of, spc_of_match);

static const struct i2c_device_id spc_id_table[] = {
	{ "SPC_R",	(kernel_ulong_t)&spc_r_devtype, },
	{ "SPC_F",	(kernel_ulong_t)&spc_f_devtype, },
	{ "SPC_R_1",	(kernel_ulong_t)&spc_r_1_devtype, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, spc_id_table);

static int spc_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id){
	
	int ret,i;
	struct spc_data_t *spc_data;
	const struct of_device_id *of_id = of_match_device(spc_of_match,&client->dev);
	struct device_node *node;
	int sensor_id;
	struct sensor_info_t *sensor;
	
	printk("SPC %s %s\n",__func__,client->name);
	
	spc_data = (struct spc_data_t *)of_id->data;
	i2c_set_clientdata(client,spc_data);
	
	mutex_init(&spc_data->ops_mutex);
	
	node = client->dev.of_node;
	if (!IS_ERR(node)){
		spc_data->pdn_pin = of_get_named_gpio(node,"pdn_pin",0);
		printk("SPC spc_r_pdn_pin(%d)\n",spc_data->pdn_pin);
		gpio_request(spc_data->pdn_pin,client->name);
		gpio_direction_output(spc_data->pdn_pin,0);
		
		spc_data->rst_pin = of_get_named_gpio(node,"rst_pin",0);
		printk("SPC spc_r_rst_pin(%d)\n",spc_data->rst_pin);
		//gpio_request(spc_data->rst_pin,client->name);
		//gpio_direction_output(spc_data->rst_pin,0);
	}else{
		printk("SPC get device node fail %s\n",client->name);
	}
	
	//set sensor type
	if (!IS_ERR(node)){
		ret = of_property_read_u32(node,"sensor_type",&sensor_id);
		if (ret){
			printk("SPC get sensor_id fail ret(%d)\n",ret);
			spc_data->sensor_type = SENSOR_TYPE_AUTO;
			spc_data->ops = sensor_list[0];
		}else{
			if (sensor_id == 0x00){
				spc_data->sensor_type = SENSOR_TYPE_AUTO;
				spc_data->ops = sensor_list[0];
			}else{
				for(i = 0;i < sizeof(sensor_list)/sizeof(sensor_list[0]); i++){
					sensor = sensor_list[i];
					if (sensor->sensor_id == sensor_id){
						spc_data->sensor_type = sensor->sensor_type;
						spc_data->ops = sensor;
						break;
					}
				}
				
				if (spc_data->sensor_type == SENSOR_TYPE_UNKNOWN){
					spc_data->sensor_type = SENSOR_TYPE_AUTO;
					spc_data->ops = sensor_list[0];
				}
			}
			printk(KERN_INFO"SPC %s sensor_id(%x)\n",client->name,sensor_id);
		}
	}
	
	//create sysfs
	ret = sysfs_create_link(spc_kobj,&client->dev.kobj,client->name);
	if (ret){
		printk(KERN_ERR"SPC sysfs_create_link fail\n");
	}
	ret = device_create_file(&client->dev, &dev_attr_open);
	if (ret){
		printk(KERN_ERR"SPC failed device_create_file(dev_attr_open)\n");
	}
	ret = device_create_file(&client->dev, &dev_attr_value);
	if (ret){
		printk(KERN_ERR"SPC failed device_create_file(dev_attr_value)\n");
	}
	ret = device_create_file(&client->dev, &dev_attr_close);
	if (ret){
		printk(KERN_ERR"SPC failed device_create_file(dev_attr_close)\n");
	}
	ret = device_create_file(&client->dev, &dev_attr_type);
	if (ret){
		printk(KERN_ERR"SPC failed device_create_file(dev_attr_type)\n");
	}
	
	return 0;
}

static int  spc_i2c_remove(struct i2c_client *client)
{
	struct spc_data_t *spc_data = i2c_get_clientdata(client);
	
	device_remove_file(&client->dev, &dev_attr_open);
	device_remove_file(&client->dev, &dev_attr_value);
	device_remove_file(&client->dev, &dev_attr_close);
	device_remove_file(&client->dev, &dev_attr_type);
	
	sysfs_remove_link(spc_kobj,client->name);
	
	printk("SPC %s\n",client->name);
	if (spc_data == NULL){
		printk("spc err ");
		return -1;
	}
	
	if (spc_data->is_enabled){
		spc_data->ops->set_power(client,0);
	}
	
	return 0;
}

static int __maybe_unused spc_i2c_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused spc_i2c_resume(struct device *dev)
{
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops spc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(spc_i2c_suspend, spc_i2c_resume)
};
#endif

static struct i2c_driver spc_driver = {
	.driver = {
		.name		= "spc",
		.owner		= THIS_MODULE,
		//.of_match_table	= of_match_ptr(sp_cam_of_match),
		.of_match_table	= spc_of_match,
		.pm		= &spc_pm_ops,
	},
	.probe		= spc_i2c_probe,
	.remove		= spc_i2c_remove,
	.id_table	= spc_id_table,
};

//module_i2c_driver(spc_driver);


static int __init spc_init(void){
	int ret = -1;

	spc_kobj = kobject_create_and_add("spc", kernel_kobj);
	if (!spc_kobj){
		CAMERA_DBG(" kernel kobject_create_and_add error \r\n"); 
		return -1;
	}

	ret = i2c_add_driver(&spc_driver);
	if (ret != 0){
		i2c_del_driver(&spc_driver);
	}
	
	return ret;
}

static void __exit spc_exit(void){
	
	i2c_del_driver(&spc_driver);
	
	if (!spc_kobj){
		return;
	}
	
	if (spc_kobj){
		kobject_put(spc_kobj);
	}
}
module_init(spc_init);
module_exit(spc_exit);

MODULE_LICENSE("GPL");


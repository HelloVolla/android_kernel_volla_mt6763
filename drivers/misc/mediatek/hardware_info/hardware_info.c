

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <linux/poll.h>
#include <linux/fb.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>

#include<linux/timer.h> 
#include<linux/jiffies.h>

//#include "lcm_drv.h"
#include "hardware_info.h"
#include "prize_custom_memory.h"
#include <linux/fs.h>


#define DEBUG_ON		0
#define HW_PRINT(fmt,arg...)           printk("[HW_INFO] "fmt"\n",##arg)
#define HW_ERROR(fmt,arg...)          printk("[HW_INFO] ERROR:"fmt"\n",##arg)
#define HW_DEBUG(fmt,arg...)          do{\
	if(DEBUG_ON)\
		printk("[HW_INFO] [%d]"fmt"\n",__LINE__, ##arg);\
		}while(0)

	

int len = 0;

struct class *hardware_info_class;
// prize add for tof info by zhuzhengjiang  20200521 start
struct hardware_info current_tof_info =
{
	"unknow","unknow","unknow","unknow",
};
// prize add for tof info by zhuzhengjiang  20200521 end
struct hardware_info current_lcm_info =
{
	"unknow","unknow","unknow","unknow",
};
struct hardware_info current_camera_info[5] = 
{
	{"unknow","unknow","unknow","unknow"},
	{"unknow","unknow","unknow","unknow"},
	{"unknow","unknow","unknow","unknow"},
	{"unknow","unknow","unknow","unknow"},
	{"unknow","unknow","unknow","unknow"},
};
struct hardware_info current_tp_info =
{
	"unknow","unknow","unknow","unknow",
};

struct hardware_info current_alsps_info =
{
	"unknow","unknow","unknow","unknow",
};

struct hardware_info current_gsensor_info =
{
	"unknow","unknow","unknow","unknow",
};
struct hardware_info current_msensor_info =
{
	"unknow","unknow","unknow","unknow",
};
struct hardware_info current_fingerprint_info =
{
	"unknow","unknow","unknow","unknow",
};
#if defined(CONFIG_PRIZE_HARDWARE_INFO_BAT)
struct hardware_info current_battery_info =
{
	"unknow","unknow","unknow","unknow","unknow",
};
EXPORT_SYMBOL_GPL(current_battery_info);
#endif
struct hardware_info current_coulo_info =
{
	"unknow","unknow","unknow","unknow",
};

EXPORT_SYMBOL_GPL(current_lcm_info);
EXPORT_SYMBOL_GPL(current_camera_info);
EXPORT_SYMBOL_GPL(current_tp_info);
EXPORT_SYMBOL_GPL(current_alsps_info);
EXPORT_SYMBOL_GPL(current_gsensor_info);
EXPORT_SYMBOL_GPL(current_msensor_info);
EXPORT_SYMBOL_GPL(current_fingerprint_info);
 // prize add for tof info by zhuzhengjiang  20200521 start
EXPORT_SYMBOL_GPL(current_tof_info); // prize add for tof info by zhuzhengjiang  20200521 start
//mt_battery_meter.h
static void dev_get_current_tof_info(char *buf)
{
    char *p = buf;	   
	HW_PRINT("hardware_info_tof");

	if(strcmp(current_tof_info.chip,"unknow") == 0)
	 	return ;

	 
	 p += sprintf(p, "[TOF]:\n");	
	 p += sprintf(p, "  chip:%s\n", current_tof_info.chip);	
	 p += sprintf(p, "  id:%s\n", current_tof_info.id);	
	 p += sprintf(p, "  vendor:%s\n",current_tof_info.vendor);		
	 p += sprintf(p, "  more:%s\n", current_tof_info.more);

	 len += (p - buf);  
	 HW_PRINT("%s",buf);
}
 // prize add for tof info by zhuzhengjiang  20200521 end
static void dev_get_current_lcm_info(char *buf)
{
    char *p = buf;	   
	HW_PRINT("hardware_info_lcm");

	if(strcmp(current_lcm_info.chip,"unknow") == 0)
	 	return ;

	 
	 p += sprintf(p, "[LCM]:\n");	
	 p += sprintf(p, "  chip:%s\n", current_lcm_info.chip);	
	 p += sprintf(p, "  id:%s\n", current_lcm_info.id);	
	 p += sprintf(p, "  vendor:%s\n",current_lcm_info.vendor);		
	 p += sprintf(p, "  more:%s\n", current_lcm_info.more);

	 len += (p - buf);  
	 HW_PRINT("%s",buf);
}
static void dev_get_current_camera_info(char *buf)
{
     char *p = buf;
	 HW_PRINT("dev_get_current_camera_info");
	 
	 
	 if(strcmp(current_camera_info[0].chip,"unknow") != 0)
	 {	 	   
	 	p += sprintf(p, "\n[Main Camera]:\n");	
	 	p += sprintf(p, "  chip:%s\n", current_camera_info[0].chip);	
	 	p += sprintf(p, "  id:%s\n", current_camera_info[0].id);	
	 	p += sprintf(p, "  vendor:%s\n",current_camera_info[0].vendor);		
	 	p += sprintf(p, "  more:%s\n", current_camera_info[0].more);
	 }

	 if(strcmp(current_camera_info[1].chip,"unknow") != 0)
	 {
		p += sprintf(p, "\n[Sub Camera]:\n");	
	 	p += sprintf(p, "  chip:%s\n", current_camera_info[1].chip);	
	 	p += sprintf(p, "  id:%s\n", current_camera_info[1].id);	
	 	p += sprintf(p, "  vendor:%s\n",current_camera_info[1].vendor);		
	 	p += sprintf(p, "  more:%s\n", current_camera_info[1].more);
	 }

	 if(strcmp(current_camera_info[2].chip,"unknow") != 0)
	 {
		p += sprintf(p, "\n[Main2 Camera]:\n");	
	 	p += sprintf(p, "  chip:%s\n", current_camera_info[2].chip);	
	 	p += sprintf(p, "  id:%s\n", current_camera_info[2].id);	
	 	p += sprintf(p, "  vendor:%s\n",current_camera_info[2].vendor);		
	 	p += sprintf(p, "  more:%s\n", current_camera_info[2].more);
	 }

	 if(strcmp(current_camera_info[3].chip,"unknow") != 0)
	 {
		p += sprintf(p, "\n[Main3 Camera]:\n");	
	 	p += sprintf(p, "  chip:%s\n", current_camera_info[3].chip);	
	 	p += sprintf(p, "  id:%s\n", current_camera_info[3].id);	
	 	p += sprintf(p, "  vendor:%s\n",current_camera_info[3].vendor);		
	 	p += sprintf(p, "  more:%s\n", current_camera_info[3].more);
	 }
	 
	 if(strcmp(current_camera_info[4].chip,"unknow") != 0)
	 {
		p += sprintf(p, "\n[Main4 Camera]:\n");	
	 	p += sprintf(p, "  chip:%s\n", current_camera_info[4].chip);	
	 	p += sprintf(p, "  id:%s\n", current_camera_info[4].id);	
	 	p += sprintf(p, "  vendor:%s\n",current_camera_info[4].vendor);		
	 	p += sprintf(p, "  more:%s\n", current_camera_info[4].more);
	 }
	 len += (p - buf);  
	 HW_PRINT("%s",buf);
}
static void  dev_get_current_tp_info(char *buf)
{	

    char *p = buf;	
	HW_PRINT("dev_get_current_tp_info");
	if(strcmp(current_tp_info.chip,"unknow") == 0)
	 	return ;
 
	    
	 p += sprintf(p, "\n[Touch Panel]:\n");	
	 p += sprintf(p, "  chip:%s\n", current_tp_info.chip);	
	 p += sprintf(p, "  id:%s\n", current_tp_info.id);	
	 p += sprintf(p, "  vendor:%s\n",current_tp_info.vendor);		
	 p += sprintf(p, "  more:%s\n", current_tp_info.more);

	 len += (p - buf);  
	 HW_PRINT("%s",buf);
	
}
static void dev_get_current_alsps_info(char *buf)
{
    
	 char *p = buf;	  
	HW_PRINT("dev_get_current_alsps_info");
	if(strcmp(current_alsps_info.chip,"unknow") == 0)
	 	return ;	
 
	 p += sprintf(p, "\n[ALS/PS]:\n");	
	 p += sprintf(p, "  chip:%s\n", current_alsps_info.chip);	
	 p += sprintf(p, "  id:%s\n", current_alsps_info.id);	
	 p += sprintf(p, "  vendor:%s\n",current_alsps_info.vendor);		
	 p += sprintf(p, "  more:%s\n", current_alsps_info.more);

	 len += (p - buf);  
	 HW_PRINT("%s",buf);
}

static void dev_get_current_gsensor_info(char *buf)
{
    char *p = buf;	  
	HW_PRINT("dev_get_current_gsensor_info");
	if(strcmp(current_gsensor_info.chip,"unknow") == 0)
	 	return ;		

	  
	 p += sprintf(p, "\n[G-sensor]:\n");	
	 p += sprintf(p, "  chip:%s\n", current_gsensor_info.chip);	
	 p += sprintf(p, "  id:%s\n", current_gsensor_info.id);	
	 p += sprintf(p, "  vendor:%s\n",current_gsensor_info.vendor);		
	 p += sprintf(p, "  more:%s\n", current_gsensor_info.more);

	 len += (p - buf);  
	 HW_PRINT("%s",buf);
}

static void dev_get_current_msensor_info(char *buf)
{
      
	 char *p = buf;	
	HW_PRINT("dev_get_current_msensor_info");
	if(strcmp(current_msensor_info.chip,"unknow") == 0)
	 	return ;		
   
	 p += sprintf(p, "\n[M-sensor]:\n");	
	 p += sprintf(p, "  chip:%s\n", current_msensor_info.chip);	
	 p += sprintf(p, "  id:%s\n", current_msensor_info.id);	
	 p += sprintf(p, "  vendor:%s\n",current_msensor_info.vendor);		
	 p += sprintf(p, "  more:%s\n", current_msensor_info.more);

	 len += (p - buf);  
	 HW_PRINT("%s",buf);
}

static void dev_get_current_fingerprint_info(char *buf)
{
     char *p = buf;
	HW_PRINT("dev_get_current_fingerprint_info");
	if(strcmp(current_fingerprint_info.chip,"unknow") == 0)
		return ;

	
	 p += sprintf(p, "\n[Fingerprint]:\n");
	 p += sprintf(p, "  chip:%s\n", current_fingerprint_info.chip);
	 p += sprintf(p, "  id:%s\n", current_fingerprint_info.id);
	 p += sprintf(p, "  vendor:%s\n",current_fingerprint_info.vendor);
	 p += sprintf(p, "  more:%s\n", current_fingerprint_info.more);

	 len += (p - buf);
	 HW_PRINT("%s",buf);
}
#if defined(CONFIG_PRIZE_HARDWARE_INFO_BAT)
static void dev_get_current_battery_info(char *buf)
{
     char *p = buf;
	HW_PRINT("dev_get_current_battery_info");
	//if(strcmp(current_battery_info.chip,"unknow") == 0)
	//	return ;

	
	 p += sprintf(p, "\n[battery]:\n");
	 p += sprintf(p, " batt_vendor:%s\n",current_battery_info.batt_versions);
	 p += sprintf(p, " Q_MAX_POS_50:%s\n",current_battery_info.Q_MAX_50);
	 p += sprintf(p, " Q_MAX_POS_25:%s\n",current_battery_info.Q_MAX_25);
	 p += sprintf(p, " Q_MAX_POS_0:%s\n",current_battery_info.Q_MAX_0);
	 p += sprintf(p, " Q_MAX_NEG_10:%s\n",current_battery_info.Q_MAX_10);
	

	 len += (p - buf);
	 HW_PRINT("%s",buf);
}
#endif

static void dev_get_current_coulo_info(char *buf)
{
     char *p = buf;
	HW_PRINT("dev_get_current_coulo_info");
	if(strcmp(current_coulo_info.chip,"unknow") == 0)
		return ;

	
	 p += sprintf(p, "\n[coulo]:\n");
	 p += sprintf(p, "  chip:%s\n", current_coulo_info.chip);
	 p += sprintf(p, "  id:%s\n", current_coulo_info.id);
	 p += sprintf(p, "  vendor:%s\n",current_coulo_info.vendor);
	 p += sprintf(p, "  more:%s\n", current_coulo_info.more);

	 len += (p - buf);
	 HW_PRINT("%s",buf);
}

#ifdef CONFIG_PRIZE_CTS
extern int sec_schip_enabled(void);
static void dev_get_efuse_status(char *buf)
{
     char *p = buf;
	HW_PRINT("dev_get_efuse_status");
	//if(strcmp(current_battery_info.chip,"unknow") == 0)
	//	return ;
	
	 p += sprintf(p, "\n[EFUSE]:\n");

	if(sec_schip_enabled())
		p += sprintf(p, " Status: eFuse blown!\n");
	else
		p += sprintf(p, " Status: eFuse not blown!\n");

	 len += (p - buf);
	 HW_PRINT("%s",buf);
}
#endif

static ssize_t hardware_info_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
	HW_PRINT("hardware_info_store buf:%s,size:%d=======",buf,(int)size);
	
	return size;
	
}

extern char *saved_command_line;
int get_lpddr_emmc_used_index(void)
{
     char *ptr;
     int lpddr_index=0;
     ptr=strstr(saved_command_line,"lpddr_used_index=");
     if(ptr==NULL)
	     return -1;
     ptr+=strlen("lpddr_used_index=");
     lpddr_index=simple_strtol(ptr,NULL,10);
     return lpddr_index;
} 

static void dev_get_current_flash_lpddr_index_info(char *buf)
{
    
	 char *p = buf;	
	 int flash_lpddr_index =-1;  
	HW_PRINT("dev_get_flash_info");
	
//	if(strcmp(current_alsps_info.chip,"unknow") == 0)
	 //	return ;	
     flash_lpddr_index=get_lpddr_emmc_used_index();
	 p += sprintf(p, "\n[flash]:\n");	
	 p += sprintf(p, " %s\n",Cust_emmc_support[flash_lpddr_index]);
	 	
	 len += (p - buf);  
	 HW_PRINT("%s",buf);
}

static void dev_get_AudioParam_version_info(char *buf)
{

	   char *p = buf;
	   struct file *fp = NULL;
	   mm_segment_t fs;
	   loff_t pos;
	   char databuf[100]={0};
	   int ret = -1;
	   
	   HW_PRINT("hardware_info_store hello enter\n");
	   fp = filp_open("/vendor/etc/audio_param/AudioParamVersionInfo.txt", O_RDONLY, 0664);
	   if (IS_ERR(fp)){
			HW_ERROR("open AudioParamVersionInfo.txt file error\n");
			return;
	   }
	   fs = get_fs();
	   set_fs(KERNEL_DS);
	   pos =0;
	   ret = vfs_read(fp, databuf, sizeof(databuf), &pos);
	   HW_PRINT("hardware_info_store read ret: %d\n",ret);
	   filp_close(fp,NULL);
	   set_fs(fs);
	   
	   p += sprintf(p, "\n[AudioParamVersionInfo]:\n");
	   p += sprintf(p, "%s\n", databuf);
	   
	   len += (p - buf);
	   HW_PRINT("%s",buf);
}

static ssize_t hardware_info_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	len = 0;
	HW_PRINT("hardware_info_show=======");
	dev_get_current_lcm_info(buf + len);

	dev_get_current_camera_info(buf + len);

	dev_get_current_tp_info(buf + len);

	dev_get_current_alsps_info(buf + len);

	dev_get_current_gsensor_info(buf + len);

	dev_get_current_msensor_info(buf + len);

	dev_get_current_fingerprint_info(buf + len);
	dev_get_current_coulo_info(buf + len);
#if defined(CONFIG_PRIZE_HARDWARE_INFO_BAT)
	dev_get_current_battery_info(buf + len);
#endif
#ifdef CONFIG_PRIZE_CTS	
	dev_get_efuse_status(buf + len);
#endif
   
	dev_get_current_flash_lpddr_index_info(buf + len);

	dev_get_AudioParam_version_info(buf + len);
	dev_get_current_tof_info(buf + len);// prize add for tof info by zhuzhengjiang  20200521 start
	return len;

}

static DEVICE_ATTR(hw_info_read, 0664, hardware_info_show, hardware_info_store);

static int __init hardware_info_dev_init(void) {	


	struct device *hardware_info_dev;
	hardware_info_class = class_create(THIS_MODULE, "hw_info");
	
	if (IS_ERR(hardware_info_class)) {
		HW_ERROR("Failed to create class(hardware_info)!");
		return PTR_ERR(hardware_info_class);
	}

	hardware_info_dev = device_create(hardware_info_class, NULL, 0, NULL, "hw_info_data");
	if (IS_ERR(hardware_info_dev))
		HW_ERROR("Failed to create hardware_info_dev device");
	
	if (device_create_file(hardware_info_dev, &dev_attr_hw_info_read) < 0)
		HW_ERROR("Failed to create device file(%s)!",dev_attr_hw_info_read.attr.name);	
	

	HW_PRINT("hardware_info initialized ok ");
	return 0;


}

static void __exit hardware_info_dev_exit(void) 
{
	class_destroy(hardware_info_class);
}

module_init (hardware_info_dev_init);
module_exit(hardware_info_dev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("lixuefeng <lixuefeng@boruizhiheng.com>");
MODULE_DESCRIPTION("show hardware info Driver");


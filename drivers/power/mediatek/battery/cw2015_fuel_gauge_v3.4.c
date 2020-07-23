#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <mt-plat/mtk_battery.h>

#if defined(CONFIG_PRIZE_HARDWARE_INFO)
#include "../../../misc/mediatek/hardware_info/hardware_info.h"
extern struct hardware_info current_coulo_info;
#endif

#define CWFG_DISABLE_LOG 1 //CHANGE   Customer need to change this for enable/disable log
//prize-wyq 20190603 diff i2c busnum-start
#if defined(CONFIG_PROJECT_KOOBEE_K6073) || defined(CONFIG_PROJECT_KOOBEE_P7H)
#define CWFG_I2C_BUSNUM 1 //CHANGE   Customer need to change this number according to the principle of hardware
#else
#define CWFG_I2C_BUSNUM 0 //CHANGE   Customer need to change this number according to the principle of hardware
#endif
//prize-wyq 20190603 diff i2c busnum-end

#define DOUBLE_SERIES_BATTERY 0
/*
#define USB_CHARGING_FILE "/sys/class/power_supply/usb/online" // Chaman
#define DC_CHARGING_FILE "/sys/class/power_supply/ac/online"
*/
#define queue_delayed_work_time  8000
#define CW_PROPERTIES "cw-bat"

#define REG_VERSION             0x0
#define REG_VCELL               0x2
#define REG_SOC                 0x4
#define REG_RRT_ALERT           0x6
#define REG_CONFIG              0x8
#define REG_MODE                0xA
#define REG_VTEMPL              0xC
#define REG_VTEMPH              0xD
#define REG_BATINFO             0x10

#define MODE_SLEEP_MASK         (0x3<<6)
#define MODE_SLEEP              (0x3<<6)
#define MODE_NORMAL             (0x0<<6)
#define MODE_QUICK_START        (0x3<<4)
#define MODE_RESTART            (0xf<<0)

#define CONFIG_UPDATE_FLG       (0x1<<1)
#define ATHD                    (0x0<<3)        // ATHD = 0%

#define BATTERY_UP_MAX_CHANGE   300*1000            // The time for add capacity when charging //420s
#define BATTERY_DOWN_MAX_CHANGE 120*1000
#define BATTERY_JUMP_TO_ZERO    30*1000
#define BATTERY_CAPACITY_ERROR  40*1000
#define BATTERY_CHARGING_ZERO   1800*1000

#define CHARGING_ON 1
#define NO_CHARGING 0


#define cw_printk(flg, fmt, arg...)        \
	({                                    \
		if(flg >= CWFG_DISABLE_LOG){\
			printk("FG_CW2015 : %s : " fmt, __FUNCTION__ ,##arg);\
		}else{}                           \
	})     //need check by Chaman


//#define cw_printk(fmt, arg...)  printk("FG_CW2015 : %s : " fmt, __FUNCTION__ ,##arg);

#define CWFG_NAME "cw2015"
#define SIZE_BATINFO    64
#if defined (CONFIG_PROJECT_KOOBEE_K5803)
static unsigned char config_info[SIZE_BATINFO] = {
0x17,0xF9,0x68,0x6B,0x6C,0x65,0x66,0x63,0x5B,0x6C,0x70,0x4B,0x5C,0x58,0x49,0x3D,0x38,0x31,0x2B,0x24,
0x2A,0x32,0x41,0x4E,0x20,0x47,0x0B,0x85,0x1E,0x3C,0x54,0x74,0x8B,0x8E,0x91,0x94,0x3E,0x1A,0x69,0x42,
0x10,0x31,0x52,0x87,0x8F,0x91,0x94,0x52,0x82,0x8C,0x92,0x96,0x62,0xB1,0xAB,0xCB,0x2F,0x7D,0x64,0xA5,
0xB5,0x0E,0x08,0x59,
};
#elif defined (CONFIG_PROJECT_KOOBEE_K605)
/*prize-wyq k605 6000mAh battery curve-start*/
static unsigned char config_info[SIZE_BATINFO] = {
    #include "profile_BRZH150X_K605Q_6000mAh_20190428.i"
};
/*prize-wyq k605 6000mAh battery curve-end*/
#elif defined (CONFIG_PROJECT_KOOBEE_K6305)
/*prize-pzp k6305 4600mAh battery curve-start*/
static unsigned char config_info[SIZE_BATINFO] = {
    #include "profile_BRZH301_K6305Q_4600mAhLT_20190521.i"
};
#elif defined (CONFIG_PROJECT_KOOBEE_K6073)
/*prize-chj k6073 3450mAh battery curve-start*/
static unsigned char config_info[SIZE_BATINFO] = {
    #include "profile_BRZH301_K6073Q_3450mAhLT_180117.i"
};
#elif defined (CONFIG_PROJECT_KOOBEE_K6303)
/*prize-chj k6303 3950mAh battery curve-start*/
static unsigned char config_info[SIZE_BATINFO] = {
    #include "profile_BRZH301_K6306Q_3950mAhLT_181121.i"
};

/*prize-chj k6073 3450mAh battery curve-end*/
#else
/*start modify prize-sunshuai-2019-01-24 Update 6309 battery curve*/
static unsigned char config_info[SIZE_BATINFO] = {
    #include "profile_BRZH1401_K6309Q_3500LT_190114.i"
};
/*end modify prize-sunshuai-2019-01-24 Update 6309 battery curve*/
#endif
static struct power_supply *chrg_usb_psy;
static struct power_supply *chrg_ac_psy;

#ifdef CONFIG_PM
static struct timespec suspend_time_before;
static struct timespec after;
static int suspend_resume_mark = 0;
#endif

struct cw_battery {
    struct i2c_client *client;

    struct workqueue_struct *cwfg_workqueue;
	struct delayed_work battery_delay_work;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
	struct power_supply cw_bat;
#else
	struct power_supply *cw_bat;
#endif

    int charger_mode;
    int capacity;
    int voltage;
    int status;
    int time_to_empty;
	int change;
    //int alt;
};

int g_cw2015_capacity = 0;
int g_cw2015_vol = 0;
int cw2015_exit_flag=0;
int charger_status = 0;


/*Define CW2015 iic read function*/
int cw_read(struct i2c_client *client, unsigned char reg, unsigned char buf[])
{
	int ret = 0;
	//msleep(10);	
	ret = i2c_smbus_read_i2c_block_data( client, reg, 1, buf);
	cw_printk(0,"%2x = %2x\n", reg, buf[0]);
	return ret;
}
/*Define CW2015 iic write function*/		
int cw_write(struct i2c_client *client, unsigned char reg, unsigned char const buf[])
{
	int ret = 0;
	msleep(10);	
	ret = i2c_smbus_write_i2c_block_data( client, reg, 1, &buf[0] );
	cw_printk(0,"%2x = %2x\n", reg, buf[0]);
	return ret;
}
/*Define CW2015 iic read word function*/	
int cw_read_word(struct i2c_client *client, unsigned char reg, unsigned char buf[])
{
	int ret = 0;
	msleep(10);	
	ret = i2c_smbus_read_i2c_block_data( client, reg, 2, buf );
	cw_printk(0,"%2x = %2x %2x\n", reg, buf[0], buf[1]);
	return ret;
}

/*CW2015 update profile function, Often called during initialization*/
int cw_update_config_info(struct cw_battery *cw_bat)
{
    int ret;
    unsigned char reg_val;
    int i;
    unsigned char reset_val;

    cw_printk(0,"[FGADC] test config_info = 0x%x\n",config_info[0]);

    
    // make sure no in sleep mode
    ret = cw_read(cw_bat->client, REG_MODE, &reg_val);
    if(ret < 0) {
        return ret;
    }

    reset_val = reg_val;
    if((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) {
        return -1;
    }

    // update new battery info
    for (i = 0; i < SIZE_BATINFO; i++) {
		cw_printk(0,"%X\n", config_info[i]);
        ret = cw_write(cw_bat->client, REG_BATINFO + i, &config_info[i]);
        if(ret < 0) 
			return ret;
    }

    reg_val |= CONFIG_UPDATE_FLG;   // set UPDATE_FLAG
    //reg_val &= 0x07;                // clear ATHD
   // reg_val |= ATHD;                // set ATHD
    ret = cw_write(cw_bat->client, REG_CONFIG, &reg_val);
    if(ret < 0) 
		return ret;
    // read back and check
    ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
    if(ret < 0) {
        return ret;
    }

    if (!(reg_val & CONFIG_UPDATE_FLG)) {
		cw_printk(1,"Error: The new config set fail\n");
		//return -1;
    }

    if ((reg_val & 0xf8) != ATHD) {
		cw_printk(1,"Error: The new ATHD set fail\n");
		//return -1;
    }

    // reset
    reset_val &= ~(MODE_RESTART);
    reg_val = reset_val | MODE_RESTART;
    ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
    if(ret < 0) return ret;

    msleep(10);
    
    ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
    if(ret < 0) return ret;
	
	cw_printk(0,"cw2015 update config success!\n");
	
    return 0;
}
/*CW2015 init function, Often called during initialization*/
static int cw_init(struct cw_battery *cw_bat)
{
    int ret;
    int i;
    unsigned char reg_val = MODE_SLEEP;
	
    if ((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) {
        reg_val = MODE_NORMAL;
        ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
        if (ret < 0) 
            return ret;
    }

    ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
    if (ret < 0)
    	return ret;
#if 0
    if ((reg_val & 0xf8) != ATHD) {
        reg_val &= 0x07;    /* clear ATHD */
        reg_val |= ATHD;    /* set ATHD */
        ret = cw_write(cw_bat->client, REG_CONFIG, &reg_val);
        if (ret < 0)
            return ret;
    }
#endif 
    ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
    if (ret < 0) 
        return ret;

    if (!(reg_val & CONFIG_UPDATE_FLG)) {
		cw_printk(0,"update config flg is true, need update config\n");
        ret = cw_update_config_info(cw_bat);
        if (ret < 0) {
			cw_printk(1,"update config fail\n");
            return ret;
        }
    } else {
    	for(i = 0; i < SIZE_BATINFO; i++) { 
	        ret = cw_read(cw_bat->client, (REG_BATINFO + i), &reg_val);
	        if (ret < 0)
	        	return ret;
	        
			cw_printk(0,"%X\n", reg_val);
	        if (config_info[i] != reg_val)
	            break;
        }
        if (i != SIZE_BATINFO) {
			cw_printk(0,"config didn't match, need update config\n");
        	ret = cw_update_config_info(cw_bat);
            if (ret < 0){
                return ret;
            }
        }
    }
	
	msleep(10);
    for (i = 0; i < 30; i++) {
        ret = cw_read(cw_bat->client, REG_SOC, &reg_val);
        if (ret < 0)
            return ret;
        else if (reg_val <= 0x64) 
            break;
        msleep(120);
    }
	
    if (i >= 30 ){
    	 reg_val = MODE_SLEEP;
         ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
         cw_printk(0,"cw2015 input unvalid power error, cw2015 join sleep mode\n");
         return -1;
    } 

	cw_printk(1,"cw2015 init success!\n");	
    return 0;
}

/*Functions:< check_chrg_usb_psy check_chrg_ac_psy get_chrg_psy get_charge_state > for Get Charger Status from outside*/
static int check_chrg_usb_psy(struct device *dev, void *data)
{
        struct power_supply *psy = dev_get_drvdata(dev);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
        if (psy->type == POWER_SUPPLY_TYPE_USB) {
#else
		if (psy->desc->type == POWER_SUPPLY_TYPE_USB) {
#endif
                chrg_usb_psy = psy;
                return 1;
        }
        return 0;
}

static int check_chrg_ac_psy(struct device *dev, void *data)
{
        struct power_supply *psy = dev_get_drvdata(dev);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
        if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
#else
		if (psy->desc->type == POWER_SUPPLY_TYPE_MAINS) {
#endif
                chrg_ac_psy = psy;
                return 1;
        }
        return 0;
}

static void get_chrg_psy(void)
{
	if(!chrg_usb_psy)
		class_for_each_device(power_supply_class, NULL, NULL, check_chrg_usb_psy);
	if(!chrg_ac_psy)
		class_for_each_device(power_supply_class, NULL, NULL, check_chrg_ac_psy);
}

static int get_charge_state(void)
{
        union power_supply_propval val;
        int ret = -ENODEV;
		int usb_online = 0;
		int ac_online = 0;

        if (!chrg_usb_psy || !chrg_ac_psy)
                get_chrg_psy();
			
        if(chrg_usb_psy) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
            ret = chrg_usb_psy->get_property(chrg_usb_psy, POWER_SUPPLY_PROP_ONLINE, &val);
#else
			ret = chrg_usb_psy->desc->get_property(chrg_usb_psy, POWER_SUPPLY_PROP_ONLINE, &val);
#endif
            if (!ret)
                usb_online = val.intval;
        }
		if(chrg_ac_psy) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
            ret = chrg_ac_psy->get_property(chrg_ac_psy, POWER_SUPPLY_PROP_ONLINE, &val);
#else
			ret = chrg_ac_psy->desc->get_property(chrg_ac_psy, POWER_SUPPLY_PROP_ONLINE, &val);
#endif
            if (!ret)
                ac_online = val.intval;			
		}
		if(!chrg_usb_psy){
			cw_printk(0,"Usb online didn't find\n");
		}
		if(!chrg_ac_psy){
			cw_printk(0,"Ac online didn't find\n");
		}
		cw_printk(0,"ac_online = %d    usb_online = %d\n", ac_online, usb_online);
		if(ac_online || usb_online){
			return 1;
		}
        return 0;
}


static int cw_por(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reset_val;
	
	reset_val = MODE_SLEEP; 			  
	ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
	if (ret < 0)
		return ret;
	reset_val = MODE_NORMAL;
	msleep(10);
	ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
	if (ret < 0)
		return ret;
	ret = cw_init(cw_bat);
	if (ret) 
		return ret;
	return 0;
}
static int cw_quickstart(struct cw_battery *cw_bat)
{
        int ret = 0;
        u8 reg_val = MODE_QUICK_START;

        ret = cw_write(cw_bat->client, REG_MODE, &reg_val);     //(MODE_QUICK_START | MODE_NORMAL));  // 0x30
        if(ret < 0) {
                dev_err(&cw_bat->client->dev, "Error quick start1\n");
                return ret;
        }
        
        reg_val = MODE_NORMAL;
        ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
        if(ret < 0) {
                dev_err(&cw_bat->client->dev, "Error quick start2\n");
                return ret;
        }
        return 1;
}
void set_charger_state(int state)
{
	
	charger_status = state;
	//printk("set_charger_state =%d\n", charger_status);
}
#define  LOW_TEMPERATURE_POWER_OFF 0xf0
extern int read_tbat_value(void);
static int cw_get_capacity(struct cw_battery *cw_bat)
{
	int cw_capacity;
	int ret;
	unsigned char reg_val[2];
	unsigned char temp_val;
	int i;
	unsigned char temp_val1;
	static int temperature = -100;
	static int reset_loop = 0;
	static int charging_loop = 0;
	static int discharging_loop = 0;
	static int jump_flag = 0;
	static int charging_5_loop = 0;
	int sleep_cap = 0;
	int remainder = 0;
	int real_SOC = 0;
	int digit_SOC = 0;
	
	ret = cw_read_word(cw_bat->client, REG_SOC, reg_val);
	if (ret < 0){
		cw_printk(0,"pzp cw_bat->capacity=%d\n", cw_bat->capacity);
		return ret;
	}
	
	

	cw_capacity = reg_val[0];
	
	real_SOC = reg_val[0];
	digit_SOC = reg_val[1];
	
	temperature = battery_get_bat_temperature();//read_tbat_value();

	if ((temperature < -1) && (cw_capacity == 0))
	{
		ret = cw_read(cw_bat->client, REG_CONFIG, &temp_val);
		if (ret < 0) 
			return ret;
		
		temp_val &= 0x07;    /* clear ATHD */
        temp_val |= LOW_TEMPERATURE_POWER_OFF;    /* set ATHD */
        ret = cw_write(cw_bat->client, REG_CONFIG, &temp_val);
		if (ret < 0) 
			return ret;
		cw_printk(0,"pzp low temper  set ATHD  cw_bat->capacity=%d\n", cw_bat->capacity);

	}
	
	
	if ((temperature > 8) && (cw_capacity == 0))
	{
		ret = cw_read(cw_bat->client, REG_CONFIG, &temp_val1);
		if (ret < 0) 
		  return ret;
	  
		if ((temp_val1 & 0xf0) == LOW_TEMPERATURE_POWER_OFF)
		{
			ret = cw_read(cw_bat->client, REG_MODE, &temp_val);
			if (ret < 0) 
				return ret;
		
			cw_quickstart(cw_bat);
			for (i = 0; i < 30; i++) {
				ret = cw_read_word(cw_bat->client, REG_SOC, reg_val);
				if (ret < 0)
					return ret;
				else if (reg_val[0] <= 0x64) 
					break;
				msleep(120);
			}
			
			temp_val1 &= 0x07;    /* clear ATHD */
			ret = cw_write(cw_bat->client, REG_CONFIG, &temp_val1);
			if (ret < 0) 
				return ret;
			cw_capacity = reg_val[0];
			cw_printk(0,"pzp low temper cw_bat->capacity=%d\n", cw_bat->capacity);
		}
		
	}
	
	if ((cw_bat->capacity > 5) && (cw_capacity == 0))
		return cw_bat->capacity;
	//printk("pzp cw_capacity = %d temperature=%d\n", cw_capacity, temperature);
	if ((cw_capacity < 0) || (cw_capacity > 100)) {
		cw_printk(0,"Error:  cw_capacity = %d\n", cw_capacity);
		reset_loop++;			
		if (reset_loop > (BATTERY_CAPACITY_ERROR / queue_delayed_work_time)){ 
			cw_por(cw_bat);
			reset_loop =0;							 
		}
								 
		return cw_bat->capacity; //cw_capacity Chaman change because I think customer didn't want to get error capacity.
	}else {
		reset_loop =0;
	}

	/*if ((charger_status == 0x0002) && (cw_capacity < 100))
	{
		cw_capacity = 100;
		cw_bat->capacity = 100;
	}*/
	
	/* case 1 : aviod swing */
	remainder = (((real_SOC * 256 + digit_SOC) * 100) / 256) % 100;
	if((remainder > 70 || remainder < 30) && (cw_capacity >= (cw_bat->capacity - 1)) && (cw_capacity <= (cw_bat->capacity + 1)) && (cw_capacity != 100))
	{
		cw_capacity = cw_bat->capacity;
	}
	
	
	/* case 2 : aviod no charge full */
	if ((cw_bat->charger_mode > 0) && (cw_capacity >= 95) && (cw_capacity <= cw_bat->capacity)) {
		cw_printk(0,"Chaman join no charge full\n");
		charging_loop++;	
		if (charging_loop > (BATTERY_UP_MAX_CHANGE / queue_delayed_work_time) ){
			cw_capacity = (cw_bat->capacity + 1) <= 100 ? (cw_bat->capacity + 1) : 100; 
			charging_loop = 0;
			jump_flag = 1;
		}else{
			cw_capacity = cw_bat->capacity; 
		}
	}

	/*case 3 : avoid battery level jump to CW_BAT */
	if ((cw_bat->charger_mode == 0) && (cw_capacity <= cw_bat->capacity ) && (cw_capacity >= 90) && (jump_flag == 1)) {
		cw_printk(0,"Chaman join no charge full discharging\n");
		#ifdef CONFIG_PM
		if(suspend_resume_mark == 1){
			suspend_resume_mark = 0;
			sleep_cap = (after.tv_sec + discharging_loop * (queue_delayed_work_time / 1000))/ (BATTERY_DOWN_MAX_CHANGE/1000) ;
			cw_printk(0,"sleep_cap = %d\n", sleep_cap);
			
			if(cw_capacity >= cw_bat->capacity - sleep_cap) {
				return cw_capacity;
			}else{
				if(!sleep_cap)
					discharging_loop = discharging_loop + 1 + after.tv_sec / (queue_delayed_work_time/1000);
				else
					discharging_loop = 0;
				cw_printk(0,"discharging_loop = %d\n", discharging_loop);
				return cw_bat->capacity - sleep_cap;
			}
		}
		#endif
		discharging_loop++;
		if (discharging_loop > (BATTERY_DOWN_MAX_CHANGE / queue_delayed_work_time) ){
			if (cw_capacity >= cw_bat->capacity - 1){
				jump_flag = 0;
			} else {
				cw_capacity = cw_bat->capacity - 1;
			}
			discharging_loop = 0;
		}else{
			cw_capacity = cw_bat->capacity;
		}
	}
	
	/*case 4 : avoid battery level is 0% when long time charging*/
	if((cw_bat->charger_mode > 0) &&(cw_capacity == 0))
	{
		charging_5_loop++;
		if (charging_5_loop > BATTERY_CHARGING_ZERO / queue_delayed_work_time) {
			cw_por(cw_bat);
			charging_5_loop = 0;
		}
		cw_capacity=1;//fucehou keep 1% charge status
		cw_printk(0,"keep soc 1 charging\n");
	}else if(charging_5_loop != 0){
		charging_5_loop = 0;
	}
	#ifdef CONFIG_PM
	if(suspend_resume_mark == 1)
		suspend_resume_mark = 0;
	#endif
	return cw_capacity;
}

/*This function called when get voltage from cw2015*/
static int cw_get_voltage(struct cw_battery *cw_bat)
{    
    int ret;
    unsigned char reg_val[2];
    u16 value16, value16_1, value16_2, value16_3;
    int voltage;
    
    ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
    if(ret < 0) {
        return ret;
    }
    value16 = (reg_val[0] << 8) + reg_val[1];

    ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
    if(ret < 0) {
          return ret;
    }
    value16_1 = (reg_val[0] << 8) + reg_val[1];

    ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
    if(ret < 0) {
        return ret;
    }
    value16_2 = (reg_val[0] << 8) + reg_val[1];

    if(value16 > value16_1) {     
        value16_3 = value16;
        value16 = value16_1;
        value16_1 = value16_3;
    }

    if(value16_1 > value16_2) {
    value16_3 =value16_1;
    value16_1 =value16_2;
    value16_2 =value16_3;
    }

    if(value16 >value16_1) {     
    value16_3 =value16;
    value16 =value16_1;
    value16_1 =value16_3;
    }            

    voltage = value16_1 * 312 / 1024;

	if(DOUBLE_SERIES_BATTERY)
		voltage = voltage * 2;
    return voltage;
}

/*This function called when get RRT from cw2015*/
static int cw_get_time_to_empty(struct cw_battery *cw_bat)
{
    int ret;
    unsigned char reg_val;
    u16 value16;

    ret = cw_read(cw_bat->client, REG_RRT_ALERT, &reg_val);
    if (ret < 0)
            return ret;

    value16 = reg_val;

    ret = cw_read(cw_bat->client, REG_RRT_ALERT + 1, &reg_val);
    if (ret < 0)
            return ret;

    value16 = ((value16 << 8) + reg_val) & 0x1fff;
    return value16;
}
/*
int check_charging_state(const char *filename)
{
	struct file *fp;
	mm_segment_t fs;
	loff_t pos;
	int read_size = 8;
	int state = 0;
	char buf[read_size];
	int ret;

	cw_printk("\n");
	fp = filp_open(filename, O_RDONLY, 0644);
	if (IS_ERR(fp))
		return -1;
	fs = get_fs();
	set_fs(KERNEL_DS);	
	pos = 0;
	ret = vfs_read(fp, buf, read_size, &pos);
	if(ret < 0)
		return -1;
	
	filp_close(fp,NULL);
	set_fs(fs);
	
	state = buf[0] - '0';
	cw_printk(1," filename = %s  state = %d \n", filename, state);
	return state;
}
*/ //Old function of get charger status

static void cw_update_charge_status(struct cw_battery *cw_bat)
{
/*
	int if_charging = 0;
	if(check_charging_state(USB_CHARGING_FILE) == 1 
		|| check_charging_state(DC_CHARGING_FILE) == 1)
	{
		if_charging = CHARGING_ON;
	}else{
		if_charging = NO_CHARGING;
	}
	if(if_charging != cw_bat->charger_mode){
		cw_bat->charger_mode = if_charging;
	}
*/ //Old function of get charger status
	int cw_charger_mode;
	cw_charger_mode = get_charge_state();
	if(cw_bat->charger_mode != cw_charger_mode){
        cw_bat->charger_mode = cw_charger_mode;
		cw_bat->change = 1;		
	}
}


static void cw_update_capacity(struct cw_battery *cw_bat)
{
    int cw_capacity;
    cw_capacity = cw_get_capacity(cw_bat);

    if ((cw_capacity >= 0) && (cw_capacity <= 100) && (cw_bat->capacity != cw_capacity)) {				
        cw_bat->capacity = cw_capacity;
		cw_bat->change = 1;
    }
}



static void cw_update_vol(struct cw_battery *cw_bat)
{
    int ret;
    ret = cw_get_voltage(cw_bat);
    if ((ret >= 0) && (cw_bat->voltage != ret)) {
        cw_bat->voltage = ret;
		cw_bat->change = 1;
    }
}

static void cw_update_status(struct cw_battery *cw_bat)
{
    int status;

    if (cw_bat->charger_mode > 0) {
        if (cw_bat->capacity >= 100) 
            status = POWER_SUPPLY_STATUS_FULL;
        else
            status = POWER_SUPPLY_STATUS_CHARGING;
    } else {
        status = POWER_SUPPLY_STATUS_DISCHARGING;
    }

    if (cw_bat->status != status) {
        cw_bat->status = status;
		cw_bat->change = 1;
    } 
}

static void cw_update_time_to_empty(struct cw_battery *cw_bat)
{
    int ret;
    ret = cw_get_time_to_empty(cw_bat);
    if ((ret >= 0) && (cw_bat->time_to_empty != ret)) {
        cw_bat->time_to_empty = ret;
		cw_bat->change = 1;
    }
}


static void cw_bat_work(struct work_struct *work)
{
    struct delayed_work *delay_work;
    struct cw_battery *cw_bat;

    delay_work = container_of(work, struct delayed_work, work);
    cw_bat = container_of(delay_work, struct cw_battery, battery_delay_work);

		cw_update_capacity(cw_bat);
		cw_update_vol(cw_bat);
		cw_update_charge_status(cw_bat);
		cw_update_status(cw_bat);
		cw_update_time_to_empty(cw_bat);
	cw_printk(0,"charger_mod = %d, status = %d, capacity = %d, voltage = %d\n", cw_bat->charger_mode, cw_bat->status, cw_bat->capacity, cw_bat->voltage);

	#ifdef CONFIG_PM
	if(suspend_resume_mark == 1)
		suspend_resume_mark = 0;
	#endif
	
	#ifdef CW_PROPERTIES
	if (cw_bat->change == 1){
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
		power_supply_changed(&cw_bat->cw_bat); 
#else
		power_supply_changed(cw_bat->cw_bat); 
#endif
		cw_bat->change = 0;
	}
	#endif
	g_cw2015_capacity = cw_bat->capacity;
    g_cw2015_vol = cw_bat->voltage;
	
	queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(queue_delayed_work_time));
}

#ifdef CW_PROPERTIES
static int cw_battery_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
    int ret = 0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
    struct cw_battery *cw_bat;
    cw_bat = container_of(psy, struct cw_battery, cw_bat); 
#else
	struct cw_battery *cw_bat = power_supply_get_drvdata(psy); 
#endif

    switch (psp) {
    case POWER_SUPPLY_PROP_CAPACITY:
            val->intval = cw_bat->capacity;
            break;
	/*
    case POWER_SUPPLY_PROP_STATUS:   //Chaman charger ic will give a real value
            val->intval = cw_bat->status; 
            break;                 
    */        
    case POWER_SUPPLY_PROP_HEALTH:   //Chaman charger ic will give a real value
            val->intval= POWER_SUPPLY_HEALTH_GOOD;
            break;
    case POWER_SUPPLY_PROP_PRESENT:
            val->intval = cw_bat->voltage <= 0 ? 0 : 1;
            break;
            
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
            val->intval = g_cw2015_vol * 1000;	//prize-cw2015-wyq-20190418 sync 0.001mA unit with sys/class/power_supply/battery/voltage_now
            break;
            
    case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
            val->intval = cw_bat->time_to_empty;			
            break;
        
    case POWER_SUPPLY_PROP_TECHNOLOGY:  //Chaman this value no need
            val->intval = POWER_SUPPLY_TECHNOLOGY_LION;	
            break;

    default:
            break;
    }
    return ret;
}

static enum power_supply_property cw_battery_properties[] = {
    POWER_SUPPLY_PROP_CAPACITY,
    //POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
    POWER_SUPPLY_PROP_TECHNOLOGY,
};
#endif 

static int cw2015_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;
    int loop = 0;
	struct cw_battery *cw_bat;
	
#ifdef CW_PROPERTIES
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)	
	struct power_supply_desc *psy_desc;
	struct power_supply_config psy_cfg = {0};
#endif
#endif
    //struct device *dev;

    cw_bat = devm_kzalloc(&client->dev, sizeof(*cw_bat), GFP_KERNEL);
    if (!cw_bat) {
		cw_printk(0,"cw_bat create fail!\n");
        return -ENOMEM;
    }

    i2c_set_clientdata(client, cw_bat);

    cw_bat->client = client;
    cw_bat->capacity = 1;
    cw_bat->voltage = 0;
    cw_bat->status = 0;
	cw_bat->charger_mode = NO_CHARGING;
	cw_bat->change = 0;
	
    ret = cw_init(cw_bat);
    while ((loop++ < 3) && (ret != 0)) {
		msleep(200);
        ret = cw_init(cw_bat);
    }
    if (ret) {
		cw_printk(1,"cw2015 init fail!\n");
        return ret;	
    }

	#ifdef CW_PROPERTIES
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
	cw_bat->cw_bat.name = CW_PROPERTIES;
	cw_bat->cw_bat.type = POWER_SUPPLY_TYPE_BATTERY;
	cw_bat->cw_bat.properties = cw_battery_properties;
	cw_bat->cw_bat.num_properties = ARRAY_SIZE(cw_battery_properties);
	cw_bat->cw_bat.get_property = cw_battery_get_property;
	ret = power_supply_register(&client->dev, &cw_bat->cw_bat);
	if(ret < 0) {
	    power_supply_unregister(&cw_bat->cw_bat);
	    return ret;
	}
#else
	psy_desc = devm_kzalloc(&client->dev, sizeof(*psy_desc), GFP_KERNEL);
	if (!psy_desc)
		return -ENOMEM;
	
	psy_cfg.drv_data = cw_bat;
	psy_desc->name = CW_PROPERTIES;
	psy_desc->type = POWER_SUPPLY_TYPE_BATTERY;
	psy_desc->properties = cw_battery_properties;
	psy_desc->num_properties = ARRAY_SIZE(cw_battery_properties);
	psy_desc->get_property = cw_battery_get_property;
	cw_bat->cw_bat = power_supply_register(&client->dev, psy_desc, &psy_cfg);
	if(IS_ERR(cw_bat->cw_bat)) {
		ret = PTR_ERR(cw_bat->cw_bat);
	    cw_printk(1, "failed to register battery: %d\n", ret);
	    return ret;
	}
#endif
	#endif

	cw_bat->cwfg_workqueue = create_singlethread_workqueue("cwfg_gauge");
	INIT_DELAYED_WORK(&cw_bat->battery_delay_work, cw_bat_work);
	queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work , msecs_to_jiffies(800)); //50 [prize]?a??1??¨²3?¦Ì?3?¡ä???¨º?1%¦Ì?¨¢?¦Ì??¨º¨¬a hjian 20171019
    #if defined(CONFIG_PRIZE_HARDWARE_INFO)
		strcpy(current_coulo_info.chip,"cw2015");
		sprintf(current_coulo_info.id,"0x%04x",client->addr);
		strcpy(current_coulo_info.vendor,"weike");
		strcpy(current_coulo_info.more,"coulombmeter");
	#endif
	cw2015_exit_flag = 1;
	cw_printk(0,"cw2015 driver probe success!\n");
    return 0;
}

/*
static int cw2015_detect(struct i2c_client *client, struct i2c_board_info *info) 
{	 
	cw_printk("\n");
	strcpy(info->type, CWFG_NAME);
	return 0;
}
*/

#ifdef CONFIG_PM
static int cw_bat_suspend(struct device *dev)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct cw_battery *cw_bat = i2c_get_clientdata(client);
		read_persistent_clock(&suspend_time_before);
        cancel_delayed_work(&cw_bat->battery_delay_work);
        return 0;
}

static int cw_bat_resume(struct device *dev)
{	
        struct i2c_client *client = to_i2c_client(dev);
        struct cw_battery *cw_bat = i2c_get_clientdata(client);
		suspend_resume_mark = 1;
		read_persistent_clock(&after);
		after = timespec_sub(after, suspend_time_before);
        queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(2));
        return 0;
}

static const struct dev_pm_ops cw_bat_pm_ops = {
        .suspend  = cw_bat_suspend,
        .resume   = cw_bat_resume,
};
#endif

static int cw2015_remove(struct i2c_client *client)	 
{
	return 0;
}

static const struct i2c_device_id cw2015_id_table[] = {
	{CWFG_NAME, 0},
	{}
};

static struct i2c_driver cw2015_driver = {
	.driver 	  = {
		.name = CWFG_NAME,
#ifdef CONFIG_PM
        .pm     = &cw_bat_pm_ops,
#endif
		.owner	= THIS_MODULE,
	},
	.probe		  = cw2015_probe,
	.remove 	  = cw2015_remove,
	//.detect 	  = cw2015_detect,
	.id_table = cw2015_id_table,
};


static struct i2c_board_info __initdata fgadc_dev = { 
	I2C_BOARD_INFO(CWFG_NAME, 0x62) 
};

static int __init cw215_init(void)
{
	struct i2c_client *client;
	struct i2c_adapter *i2c_adp;

    //i2c_register_board_info(CWFG_I2C_BUSNUM, &fgadc_dev, 1);
	i2c_adp = i2c_get_adapter(CWFG_I2C_BUSNUM);
	client = i2c_new_device(i2c_adp, &fgadc_dev);
	
    i2c_add_driver(&cw2015_driver);
    return 0; 
}


static void __exit cw215_exit(void)
{
    i2c_del_driver(&cw2015_driver);
}

module_init(cw215_init);
module_exit(cw215_exit);

MODULE_AUTHOR("Chaman Qi");
MODULE_DESCRIPTION("CW2015 FGADC Device Driver V3.0");
MODULE_LICENSE("GPL");

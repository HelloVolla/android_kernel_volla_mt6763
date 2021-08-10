/******************************************************************************
* function: linux test platform driver
*        
* file  MT5715 wireless driver .c
*        
* author  Yangwl@maxictech.com  11/5/2018
* 
* interrupt and ldo control by author lifenfen@szprize.com  1/2/2019 
******************************************************************************/
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/timer.h>

#include <linux/delay.h>
#include <linux/kernel.h>

#include <linux/poll.h>

#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>

#include "MT5715_wireless.h"

#define DEVICE_NAME  "mt5715_iic"

enum REG_INDEX
{
	CHIPID = 0,
	VOUT,
	INT_FLAG,
	INTCTLR,
	VOUTSET,
	VFC,
	CMD,
	INDEX_MAX,
};

/* REG RW permission */
#define REG_NONE_ACCESS 0
#define REG_RD_ACCESS  (1 << 0)
#define REG_WR_ACCESS  (1 << 1)
#define REG_BIT_ACCESS  (1 << 2)
#define REG_MAX         0x0F
/* REG RW permission */

struct reg_attr {
	const char *name;
	u16 addr;
	u8 flag;
};

static struct reg_attr reg_access[INDEX_MAX] = {
    [CHIPID] = {"CHIPID", REG_CHIPID, REG_RD_ACCESS},
    [VOUT] = {"VOUT", REG_VOUT, REG_RD_ACCESS},
    [INT_FLAG] = {"INT_FLAG", REG_INT_FLAG,REG_RD_ACCESS},
    [INTCTLR] = {"INTCLR", REG_INTCLR,REG_WR_ACCESS},
    [VOUTSET] = {"VOUTSET", REG_VOUTSET, REG_RD_ACCESS|REG_WR_ACCESS},
    [VFC] = {"VFC", REG_VFC, REG_RD_ACCESS|REG_WR_ACCESS},
    [CMD] = {"CMD", REG_CMD, REG_RD_ACCESS|REG_WR_ACCESS|REG_BIT_ACCESS},
};

/* INT status reg 0x08 */
typedef enum {
    INT_BC_RECV   = 0x0001,
    INT_LDO_ON    = 0x0002,
    INT_LDO_OFF   = 0x0004,
    INT_OVP_FLAG  = 0x0008,
    INT_OCP_FLAG  = 0x0010,
    INT_PLDO_FLAG = 0x0020,
    INT_AVDD5V_ON = 0x0040,
    INT_AVDD5V_OFF = 0x0080,
    INT_AFC_ON    = 0x0100,
} IntType;
/* INT status reg 0x08 */

/* 
	notice:	little endian;
	value 9000 = 0x2328, ptr[1] = 0x23, ptr[0] = 0x28;
	i2c buffer byte is from index 0, big endian
*/
typedef union{
    u16 value;
    u8  ptr[2];
}vuc;

struct MT5715_dev *mte;
int is_5715_probe_done = 0;

struct MT5715_func {
    int (*read)(struct MT5715_dev *di, u16 reg, u8 *val);
    int (*write)(struct MT5715_dev *di, u16 reg, u8 val);
    int (*read_buf)(struct MT5715_dev *di,
                    u16 reg, u8 *buf, u32 size);
    int (*write_buf)(struct MT5715_dev *di,
                     u16 reg, u8 *buf, u32 size);
};

struct otg_en_6370 {
	struct pinctrl *pinctrl_gpios;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_otg_high, *pins_otg_low;
	bool gpio_otg_prepare;
};
struct MT5715_dev {
    char                *name;
    struct i2c_client    *client;
    struct device       *dev;
    struct regmap       *regmap;
    struct MT5715_func  bus;
    int dc_gpio;
    int irq_gpio;
//#if defined (CONFIG_PROJECT_KOOBEE_K605)
	int VrectFC;
	struct delayed_work  check_afc_work;
//#endif
    struct mutex slock;
    //struct wake_lock wakelock;
    struct delayed_work  eint_work;
	struct delayed_work check_work;
    int ldo_status;
	int is_samsung_charge;
	int afc_count;
	struct otg_en_6370 otg_en;
	bool full_bat_disable_wireless;
};

int  fast_sv(int temp);
int  ldo_disable(void);


bool MT5715_good_status(void)
{
    pr_err("%s: gpio_get_value(mte->dc_gpio) =%d\n", __func__,gpio_get_value(mte->dc_gpio));
	return !!(gpio_get_value(mte->dc_gpio));
}
EXPORT_SYMBOL(MT5715_good_status);


/* ic power status */
static bool MT5715_power_status(void)
{
   	return !!(gpio_get_value(mte->dc_gpio));
}
/* ic power status */

/* platform i2c operation */
int MT5715_read(struct MT5715_dev *di, u16 reg, u8 *val)
{
    unsigned int temp;
    int rc;

    if (MT5715_power_status()) {
	    rc = regmap_read(di->regmap, reg, &temp);
	    if (rc >= 0)
	        *val = (u8)temp;
    } else {
	    pr_err("%s: charger is off state\n", __func__);
	    return -1;
    }
    return rc;
}

int MT5715_write(struct MT5715_dev *di, u16 reg, u8 val)
{
    int rc = 0;

    if (MT5715_power_status()) {
	    rc = regmap_write(di->regmap, reg, val);
	    if (rc < 0)
	        dev_err(di->dev, "NE6153 write error: %d\n", rc);
    } else {
	    pr_err("%s: charger is off state\n", __func__);
	    return -1;
    }

    return rc;
}

int MT5715_read_buffer(struct MT5715_dev *di, u16 reg, u8 *buf, u32 size)
{
	if (MT5715_power_status()) {
		return regmap_bulk_read(di->regmap, reg, buf, size);
	} else {
		pr_err("%s: charger is off state\n", __func__);
		return -1;
	}
}

int MT5715_write_buffer(struct MT5715_dev *di, u16 reg, u8 *buf, u32 size)
{
    int rc = 0;

	#if 0
	/* debug */
	if (size == 2)
		pr_err("%s: 0x%02x%02x\n", __func__, buf[0], buf[1]);
	#endif

    if (MT5715_power_status()) {
    	while (size--) {
    		rc = di->bus.write(di, reg++, *buf++);
    		if (rc < 0) {
    			dev_err(di->dev, "write error: %d\n", rc);
    			return rc;
    		}
    	}
    } else {
    	pr_err("%s: charger is off state\n", __func__);
    	return -1;
    }

    return rc;
}

int set_otg_gpio(int en){

	int ret =0;
	
	if(mte->full_bat_disable_wireless == true)
		en = 1;
	
    if (mte->otg_en.gpio_otg_prepare) {
		if (en) {
			pinctrl_select_state(mte->otg_en.pinctrl_gpios, mte->otg_en.pins_otg_high);
			printk("%s: set w_otg_en PIN to high\n", __func__);
			ret =0;
		}
		else {
			pinctrl_select_state(mte->otg_en.pinctrl_gpios, mte->otg_en.pins_otg_low);
			printk("%s: set w_otg_en PIN to low\n", __func__);
			ret =0;
		}
	}
	else {
		printk("%s:, error, gpio otg not prepared\n", __func__);
		ret =-1;
	}
	return ret;
}

/* platform i2c operation */

/* ic irq read/ write clear */
static int MT5715_irq_status(int read)
{
	vuc irq_flag;
	vuc irq_flag_temp;
	int rc = 0;
	int count = 3;//i2c err

	pr_err("enter %s: \n", __func__);


    irq_flag_temp.value = 0;
	
	if (MT5715_power_status() == 0) {
		return -1;//charger off
	}

	//mutex_lock(&mte->slock);
	do {
		if (read) {//read
			rc = MT5715_read_buffer(mte, reg_access[INT_FLAG].addr, irq_flag.ptr, 2);			
			if (rc){
				pr_err("%s: MT5715_read_buffer rc =%d \n", __func__,rc);
				continue;
			}
			else {
				pr_err("%s: read irq status successful, 0x%02x%02x, 0x%04x\n", __func__, irq_flag.ptr[0], irq_flag.ptr[1], irq_flag.value);
				return (irq_flag.ptr[0] << 8 | irq_flag.ptr[1]);
			}
		} else {//write clear
			rc = MT5715_read_buffer(mte, reg_access[INT_FLAG].addr, irq_flag.ptr, 2);
			if (rc){ 
				pr_err("%s: MT5715_read_buffer rc =%d  step1\n", __func__,rc);
				continue;
			}

			pr_err("%s: read irq, 0x%02x%02x, 0x%04x\n", __func__, irq_flag.ptr[0], irq_flag.ptr[1], irq_flag.value);

            irq_flag_temp.ptr[0] = irq_flag.ptr[0];
			irq_flag.ptr[0] = irq_flag_temp.ptr[0] & 0xFE;
			
			pr_err("%s: INTCTLR, 0x%02x%02x, 0x%04x\n", __func__, irq_flag.ptr[0], irq_flag.ptr[1], irq_flag.value);
			rc = MT5715_write_buffer(mte, reg_access[INTCTLR].addr, irq_flag.ptr, 2);
			if (rc){
				pr_err("%s: MT5715_read_buffer rc =%d  step2\n", __func__,rc);
				continue;
			}

			irq_flag.ptr[0] = 0x00;
			irq_flag.ptr[1] = CLRINT;
			pr_err("%s: cmd, 0x%02x%02x, 0x%04x\n", __func__, irq_flag.ptr[0], irq_flag.ptr[1], irq_flag.value);
			rc = MT5715_write_buffer(mte, reg_access[CMD].addr, irq_flag.ptr, 2);
			if (rc){
				pr_err("%s: MT5715_read_buffer rc =%d  step3\n", __func__,rc);
				continue;
			}

			rc = MT5715_read_buffer(mte, reg_access[INT_FLAG].addr, irq_flag.ptr, 2);
			if (rc){ 
				pr_err("%s: MT5715_read_buffer rc =%d  step4\n", __func__,rc);
				continue;
			}
			else {
				pr_err("%s: read irq after, 0x%02x%02x, 0x%04x\n", __func__, irq_flag.ptr[0], irq_flag.ptr[1], irq_flag.value);
				if (irq_flag.value == 0) {
					pr_err("%s: clear irq successful\n", __func__);
					return irq_flag.value;
				}
				else {				
					pr_err("%s: clear irq fail 0x%04x\n", __func__, irq_flag.value);
					msleep(5);
					rc = MT5715_read_buffer(mte, reg_access[INT_FLAG].addr, irq_flag.ptr, 2);
					if (rc){
						pr_err("%s: MT5715_read_buffer rc =%d  step5\n", __func__,rc);
						continue;
					}
					pr_err("%s: clear irq second time 0x%02x%02x 0x%04x\n", __func__, irq_flag.ptr[0], irq_flag.ptr[1], irq_flag.value);
					return (irq_flag.ptr[0] << 8 | irq_flag.ptr[1]);
				}
			}
		}
	} while (count --);
	//mutex_unlock(&mte->slock);

	pr_err("%s: rc = %d\n", __func__, rc);
	return rc;
}
/* ic irq read/ write clear */

#if 0
/* LDO voltage */
static int MT5715_ldo_status(bool read, int *vol)
{
    int ret = 0;
    vuc val;
    int vout_value  = 0;
    u8  tmp = 0;

    if (read) {
    	ret = MT5715_read_buffer(mte, reg_access[VOUT].addr, val.ptr, 2);
    	if (ret)
    		return -1;
    	else {
    		vout_value = val.ptr[0] << 8 |  val.ptr[1];
    		pr_err("chip Vout : %d\n",vout_value);
    		*vol = vout_value;
    		return ret;
    	}
    } else {
    	val.value = *vol;
    	if((vout_value < 0) || (vout_value > 20000)) {
    		pr_err("REG_VOUTSET Parameter error\n");
    		return -1;
    	}

    	tmp = val.ptr[0];
    	val.ptr[0] = val.ptr[1];
    	val.ptr[1] = tmp;
    	pr_err("set chip Vout: %02x%02x\n", val.ptr[0], val.ptr[1]);

    	ret = MT5715_write_buffer(mte, REG_VOUTSET, val.ptr,2);
    	if (ret) {
    		pr_err("set chip Vout: %d fail\n",vout_value);
    		return -1;
    	} else {
    		ret = MT5715_read_buffer(mte, reg_access[VOUT].addr, val.ptr, 2);
    		if (ret) {
    			pr_err("read chip Vout: %d fail\n",vout_value);
    			return -1;
    		} else {
    			vout_value = val.ptr[0] << 8 |  val.ptr[1];
    			pr_err("read chip Vout: %d\n",vout_value);
    			return vout_value;
    		}
    	}
    }
}
/* LDO voltage */
#endif

//prize modify by sunshuai, wireless charge MT5715  soft get ldo status , 20190302-start
int get_lod_status(void){
    if((mte->ldo_status ==1)&&(MT5715_power_status() == true)){
		return 1;
    }
	else{
		return 0;
	}
}
EXPORT_SYMBOL(get_lod_status);
//prize modify by sunshuai, wireless charge MT5715  soft get ldo status , 20190302-end

//prize modify by sunshuai, wireless charge MT5715 get  good status  , 20190302-start
int get_MT5715on_status(void){
    if(MT5715_power_status() == true)
		return 1;
	else
		return 0;

}
EXPORT_SYMBOL(get_MT5715on_status);
//prize modify by sunshuai, wireless charge MT5715 get  good status  , 20190302-end


void wireless_power_charge_complete(void){
    u8 ask_buff[2] = {0x02,0x01};
    u8 cmd_buff[2] = {0x00,0x08};
	int ret;
    ret = MT5715_write_buffer(mte,0x0040,ask_buff,2);
    pr_err("%s: MT5715_write_buffer  0x0040 ret =%d  \n", __func__,ret);
 
    ret = MT5715_write_buffer(mte,REG_CMD,cmd_buff,2);
	pr_err("%s: MT5715_write_buffer  REG_CMD ret =%d  \n", __func__,ret);
}
EXPORT_SYMBOL(wireless_power_charge_complete);

void verify_iic_write(u16 reg_addr,u8 *ptr,u8 len){
	u8 vytr[20];
	int ir,jr,ret;
	if(len > 19){
		pr_err("%s:len error!  \n",__func__);
		return;
	}
	for(ir = 0;ir < 3;ir++){
		ret = 0;
		MT5715_write_buffer(mte,reg_addr,ptr,len);
		msleep(10);
		MT5715_read_buffer(mte,reg_addr,vytr,len);
		for(jr = 0;jr < len;jr++){
			if(vytr[jr]!=ptr[jr]) {
				ret = 1;
				pr_err("%s: rror!  Data written and read do not correspond retry time:%d\n",__func__,ir);
			}
		}
		if(ret == 0) return;
	}
	
}

//prize modify by sunshuai, wireless charge MT5715  soft Compatible reporting up 9 volts interrupt handling, 20190302-start
static int MT5715_handle(u16 flag)
{
	//size_t eol;
	vuc irq_flag;
	//int val;
	int ret = 0;

	//eol = find_next_bit((unsigned long *)&flag, sizeof(flag), 0);
	pr_err("%s:flag = 0x%x, \n", __func__,flag);
    irq_flag.value = flag;

	if(irq_flag.ptr[0] & 0x02){//bit 1
		vuc ttm;
		vuc val;
		ttm.value = 0;		
		verify_iic_write( 0xB580, ttm.ptr, 1);
		msleep(5);
		verify_iic_write( 0xB581, ttm.ptr, 1);
		msleep(5);
		verify_iic_write( 0xB582, ttm.ptr, 1);
		msleep(5);
		verify_iic_write( 0xB583, ttm.ptr, 1);
		schedule_delayed_work(&mte->check_work,500);
//#if defined (CONFIG_PROJECT_KOOBEE_K605)
	val.ptr[0] = 0x34;
    val.ptr[1] = 0x28;
	verify_iic_write( 0x0074, val.ptr, 2);//Increase the received power increase reported by RX to TX 750ma--1.2a is 2W.
	msleep(5);
	pr_err("%s:write 0x0074 addr  value: 0x%02x,0x%02x \n",__func__, val.ptr[0],val.ptr[1]);
	
	val.ptr[0] = 0x33;
    val.ptr[1] = 0x28;
	verify_iic_write(0x0076, val.ptr, 2);//Increase the received power increase reported by RX to TX 750ma--1.2a is 2W.
	pr_err("%s:write 0x0076 addr  value: 0x%02x,0x%02x \n",__func__, val.ptr[0],val.ptr[1]);
	msleep(5);
	val.ptr[0] = 0x31;
    val.ptr[1] = 0x28;
	verify_iic_write( 0x0078, val.ptr, 2);//Increase the received power increase reported by RX to TX 750ma--1.2a is 2W.
	pr_err("%s:write 0x0076 addr  value: 0x%02x,0x%02x \n",__func__, val.ptr[0],val.ptr[1]);
	msleep(5);
    val.ptr[0] = 0x32;
    val.ptr[1] = 0x5E;
	verify_iic_write( 0x007A, val.ptr, 2);//Increase the received power increase reported by RX to TX 750ma--1.2a is 2W.
	pr_err("%s:write 0x007A addr  value: 0x%02x,0x%02x \n",__func__, val.ptr[0],val.ptr[1]);
//#endif
        mte->ldo_status = 1;
		mte->is_samsung_charge =0;
		mte->afc_count = 0;
		pr_err("%s: INT_LDO_ON mte->ldo_status =%d mte->is_samsung_charge =%d\n" , __func__,mte->ldo_status,mte->is_samsung_charge);
	}
	
    if(irq_flag.ptr[0] & 0x04){// bit 2
		mte->ldo_status = 0;
		mte->is_samsung_charge =0;
		pr_err("%s: INT_LDO_OFF mte->ldo_status =%d mte->is_samsung_charge=%d\n" , __func__,mte->ldo_status,mte->is_samsung_charge);
	}


	if(irq_flag.ptr[1] & 0x01){//bit 0

		vuc val;

		pr_err("%s: before step up 9V \n" , __func__);
		schedule_delayed_work(&mte->check_work,300);
		mte->is_samsung_charge = 1;

		MT5715_read_buffer(mte,REG_VRECT,val.ptr,2);
		mte->VrectFC = val.ptr[0] << 8 |  val.ptr[1];
		schedule_delayed_work(&mte->check_afc_work,400);

		pr_err("%s: after step up 9V \n" , __func__);
	}
	return ret;
}
//prize modify by sunshuai, wireless charge MT5715  soft Compatible reporting up 9 volts interrupt handling, 20190302-end


/* ldo status */
int MT5715_ldo_on(bool on)
{
	vuc value;
	int rc = 0;
	int count = 3;//i2c err
	pr_err("%s: mte->ldo_status = %d on = %d\n", __func__, mte->ldo_status, on);

	if (mte->ldo_status ^ on) {
		do {
			value.ptr[0] = 0x00;
			value.ptr[1] = LDOTGL;
			pr_err("%s: cmd, 0x%02x%02x, 0x%04x\n", __func__, value.ptr[0], value.ptr[1], value.value);
			rc = MT5715_write_buffer(mte, reg_access[CMD].addr, value.ptr, 2);
			if (rc) 
				continue;
		} while (count --);
	}
	return rc;
}
EXPORT_SYMBOL(MT5715_ldo_on);
/* ldo status */

void MT5715_irq_handle(void)
{
	int flag = 0;
	int flag_c = 0;
	int count = 5;//new irq count, add error handling later
	int ret = 0;

	pr_err("enter %s:\n", __func__);

	while (count) {
		flag = MT5715_irq_status(true);
		if (flag > 0)
		        pr_err("%s: read irq successful, flag = %x\n", __func__, flag);
		else if (flag == 0) {
		        pr_err("%s: no irq\n", __func__);
		        return;
		} else {
		        pr_err("%s: charger off\n", __func__);
		        return;
		}

		ret = MT5715_handle((u16)flag);

#if 1
		flag_c = MT5715_irq_status(false);
		if (flag_c == 0) {
		        pr_err("%s: clear irq successful, flag = %x\n", __func__, flag_c);
		        return;
		}
		else if(flag_c > 0) {//new irq
		        pr_err("%s: new irq occur, flag_c = %x\n", __func__, flag_c);
		        if (flag ^ flag_c) {
		                MT5715_handle(flag_c);
		                count --;
		                pr_err("%s: new irq occur, count = %d\n", __func__, count);
		        }
		        else {
		                pr_err("%s: flag_c = %x, count = %d. same irq occur, clear fail!!!\n", __func__, flag_c, count);
		                break;
		        }

		} else {
		        pr_err("%s: ic error, flag_c = %x\n", __func__, flag_c);
		        return;
		}
#endif
	}
	return;
}
EXPORT_SYMBOL(MT5715_irq_handle);

static void MT5715_eint_work(struct work_struct *work)
{
	MT5715_irq_handle();
}
static void MT5715_check_work(struct work_struct *work)
{
	vuc val;
	vuc irq_flag;
	int rc;
	u8  fcflag;
	u8 ret = 0;
	
	if(mte->is_samsung_charge == 1){
		MT5715_read_buffer(mte,REG_VOUT,val.ptr,2);
		rc = val.ptr[0] << 8 |  val.ptr[1];
		pr_err("%s: vol read  vol=%d\n", __func__,rc);
		if(rc < 7500){
			val.value = 9000;  // 9000  0x2328
			fcflag = val.ptr[0]; //swap
			val.ptr[0] = val.ptr[1];
			val.ptr[1] = fcflag;  //0x2823
			MT5715_write_buffer(mte, REG_VFC, val.ptr, 2);
			pr_err("FC send data step up 9V : 0x%02x,0x%02x \n", val.ptr[0],val.ptr[1]);
			irq_flag.ptr[0] = 0x00;
			irq_flag.ptr[1] = 0x10;
			MT5715_write_buffer(mte, REG_CMD, irq_flag.ptr, 2);
			ret = 0;
		}else{
			ret = 1;
		}
	}else{
		MT5715_read_buffer(mte,REG_INT_FLAG,val.ptr,2);
		pr_err("%s: read irq status successful, 0x%02x%02x, 0x%04x is_samsung_charge=%d\n", __func__, val.ptr[0], val.ptr[1], val.value,mte->is_samsung_charge);
		if(val.ptr[0] & 0x01) {
			//Clear interrupt
			mte->is_samsung_charge =1;
			pr_err("%s: set is_samsung_charge =%d\n", __func__,mte->is_samsung_charge);
			irq_flag.ptr[0] = 0x01;
			irq_flag.ptr[1] = 0x00;
			rc = MT5715_write_buffer(mte, reg_access[INTCTLR].addr, irq_flag.ptr, 2); 
			if (rc){
				pr_err("%s: clean irq fail rc =%d  step1\n", __func__,rc);
				return;
			}

			val.value = 9000;  // 9000  0x2328
			fcflag = val.ptr[0]; //swap
			val.ptr[0] = val.ptr[1];
			val.ptr[1] = fcflag;  //0x2823
			MT5715_write_buffer(mte, REG_VFC, val.ptr, 2);
			pr_err("FC send data step up 9V : 0x%02x,0x%02x \n", val.ptr[0],val.ptr[1]);
			irq_flag.ptr[0] = 0x00;
			irq_flag.ptr[1] = 0x10 |CLRINT ;//prize modify by sunshuai  Put the 9V interrupt flag bit together with the 9V command   201900621
			MT5715_write_buffer(mte, REG_CMD, irq_flag.ptr, 2);
			MT5715_read_buffer(mte,REG_VRECT,val.ptr,2);
			mte->VrectFC = val.ptr[0] << 8 |  val.ptr[1];
			schedule_delayed_work(&mte->check_afc_work,400);
		}
		ret = 0;
	}
	if(ret == 0) schedule_delayed_work(&mte->check_work,1000);
	/*
	else{
    	if(mte->is_samsung_charge == 1){//If the Samsung protocol plate boosts 9V fails, try again multiple boost 9V 201900621
			MT5715_read_buffer(mte,REG_VOUT,val.ptr,2);
		    rc = val.ptr[0] << 8 |  val.ptr[1];
			pr_err("%s: vol read  vol=%d\n", __func__,rc);
			if(rc < 7500){
				val.value = temp;  // 9000  0x2328
		        fcflag = val.ptr[0]; //swap
		        val.ptr[0] = val.ptr[1];
		        val.ptr[1] = fcflag;  //0x2823
		        MT5715_write_buffer(mte, REG_VFC, val.ptr, 2);
		        pr_err("FC send data step up 9V : 0x%02x,0x%02x \n", val.ptr[0],val.ptr[1]);
				irq_flag.ptr[0] = 0x00;
				irq_flag.ptr[1] = 0x10  ;
		        MT5715_write_buffer(mte, REG_CMD, irq_flag.ptr, 2);
			}
    	}
    
    }
	*/
}
//#if defined (CONFIG_PROJECT_KOOBEE_K605)
static void MT5715_check_afc_work(struct work_struct *work)
{
	vuc val;
	unsigned short Vrect_value,Vout_value;
	MT5715_read_buffer(mte,REG_VOUT,val.ptr,2);
	Vout_value = val.ptr[0] << 8 |  val.ptr[1];
	if(Vout_value > 8000){
		pr_err("%s: successful boosting\n", __func__);
		return;
	}
	MT5715_read_buffer(mte,REG_VRECT,val.ptr,2);
	Vrect_value = val.ptr[0] << 8 |  val.ptr[1];
	if((Vrect_value - mte->VrectFC) > 1200){ 
		val.ptr[0] = 0x23;
		val.ptr[1] = 0x28;
		MT5715_write_buffer(mte, REG_VOUTSET, val.ptr, 2);
		pr_err("%s: Ap write LDO Vout,value: 0x%02x,0x%02x\n", __func__,val.ptr[0],val.ptr[1]);
		mte->afc_count = 0;
		schedule_delayed_work(&mte->check_afc_work,600);
	}else{
		mte->afc_count++;
		if(mte->afc_count > 10){
			mte->afc_count = 0;
			return;
		}else{
			schedule_delayed_work(&mte->check_afc_work,200);
		}
	}
	/*
	 unsigned char   fwver[2];
    unsigned short  vout_value;
    ssize_t len = 0;

    MT5715_read_buffer(mte, REG_VOUT, fwver,2);
    vout_value = fwver[0] << 8 |  fwver[1];
	*/
	
	
}
//#endif
static irqreturn_t MT5715_irq(int irq, void *data)
{
    struct MT5715_dev *mt5715 = data;
	pr_err("enter %s: \n", __func__);
	schedule_delayed_work(&mt5715->eint_work, 0);
    return IRQ_HANDLED;
}

static int MT5715_parse_dt(struct i2c_client *client, struct MT5715_dev *mt5715)
{
	int ret =0;
	mt5715->dc_gpio = of_get_named_gpio(client->dev.of_node, "dc-gpio", 0);
	if (mt5715->dc_gpio < 0) {
		pr_err("%s: no dc gpio provided\n", __func__);
		return -1;
	} else {
		pr_info("%s: dc gpio provided ok. mt5715->dc_gpio = %d\n", __func__, mt5715->dc_gpio);
	}

	mt5715->irq_gpio = of_get_named_gpio(client->dev.of_node, "irq-gpio", 0);
	if (mt5715->irq_gpio < 0) {
		pr_err("%s: no irq gpio provided.\n", __func__);
		return -1;
	} else {
		pr_info("%s: irq gpio provided ok. mt5715->irq_gpio = %d\n", __func__, mt5715->irq_gpio);
	}
//prize add by sunshuai 20200111 start
   mt5715->otg_en.pinctrl_gpios = devm_pinctrl_get(&client->dev);
   if (IS_ERR(mt5715->otg_en.pinctrl_gpios)) {
		ret = PTR_ERR(mt5715->otg_en.pinctrl_gpios);
		printk("%s can't find chg_data pinctrl\n", __func__);
		return ret;
   }
   
	mt5715->otg_en.pins_default = pinctrl_lookup_state(mt5715->otg_en.pinctrl_gpios, "default");
	if (IS_ERR(mt5715->otg_en.pins_default)) {
		ret = PTR_ERR(mt5715->otg_en.pins_default);
		printk("%s can't find chg_data pinctrl default\n", __func__);
		/* return ret; */
	}

	mt5715->otg_en.pins_otg_high = pinctrl_lookup_state(mt5715->otg_en.pinctrl_gpios, "charger_otg_on");
	if (IS_ERR(mt5715->otg_en.pins_otg_high)) {
		ret = PTR_ERR(mt5715->otg_en.pins_otg_high);
		printk("%s  can't find chg_data pinctrl otg high\n", __func__);
		return ret;
	}

	mt5715->otg_en.pins_otg_low = pinctrl_lookup_state(mt5715->otg_en.pinctrl_gpios, "charger_otg_off");
	if (IS_ERR(mt5715->otg_en.pins_otg_low)) {
		ret = PTR_ERR(mt5715->otg_en.pins_otg_low);
		printk("%s  can't find chg_data pinctrl otg low\n", __func__);
		return ret;
	}

	mt5715->otg_en.gpio_otg_prepare = true;
//prize add by sunshuai 20200111 end

	return 0;
}

/* attr for debug */
static ssize_t get_reg(struct device* cd,struct device_attribute *attr, char* buf)
{
    u8 val[2];
    ssize_t len = 0;
    int i = 0;
	
    for(i = 0; i< INDEX_MAX ;i++) {
        if(reg_access[i].flag & REG_RD_ACCESS) {
            MT5715_read_buffer(mte,reg_access[i].addr, val, 2);
            len += snprintf(buf+len, PAGE_SIZE-len, "reg:%s 0x%02x=0x%02x%02x\n", reg_access[i].name, reg_access[i].addr,val[0],val[1]);
        }
    }

    return len;
}

#if 0
void get_int_flag(void){
	vuc irq_flag;
	int rc = 0;
	u8  ptr_temp;
	bool flag= 0;
	int val = 0;
	
	 rc = MT5715_read_buffer(mte, reg_access[INT_FLAG].addr, irq_flag.ptr, 2);
	 if (rc){
			pr_err("%s: MT5715_read_buffer rc =%d \n", __func__,rc);
			val = 0;
			return;
	 }
	 else {
			pr_err("%s: read irq status successful, 0x%02x%02x, 0x%04x\n", __func__, irq_flag.ptr[0], irq_flag.ptr[1], irq_flag.value);
			//val = irq_flag.ptr[0] << 8 | irq_flag.ptr[1];
			ptr_temp = irq_flag.ptr[1];
			
            flag = (ptr_temp & 0x04) == 0x00 ? true:false;
			
			if (irq_flag.ptr[0]== 0x01 && flag== true) {
					rc = MT5715_ldo_status(true, &val);
					if (!rc)
						pr_err("%s: ldo = %d\n", __func__, val);

					val = 9000;
					rc = MT5715_ldo_status(false, &val);
					pr_err("%s: set ldo:%d, act ldo:%d\n", __func__, val, rc);
			}
			return;
	 }
			
}
EXPORT_SYMBOL(get_int_flag);
#endif

static ssize_t set_reg(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
    unsigned int databuf[2];
    vuc val;	
    u8  tmp[2];
    u16 regdata;
    int i = 0;
	int ret = 0;

	ret = sscanf(buf,"%x %x",&databuf[0], &databuf[1]);

    if(2 == ret) {
		for(i = 0; i< INDEX_MAX ;i++) {
			if(databuf[0] == reg_access[i].addr) {
				if (reg_access[i].flag & REG_WR_ACCESS) {
					val.ptr[0] = (databuf[1] & 0xff00) >> 8;
					val.ptr[1] = databuf[1] & 0x00ff;   //big endian
					if (reg_access[i].flag & REG_BIT_ACCESS) {
						MT5715_read_buffer(mte, databuf[0], tmp,2);	
						regdata = tmp[0] << 8 | tmp[1];
						val.value |= regdata;
						printk("get reg: 0x%04x  set reg: 0x%04x \n", regdata, val.value);
						MT5715_write_buffer(mte, databuf[0], val.ptr, 2);
					}
					else {
						printk("Set reg : [0x%04x]  0x%x 0x%x \n",databuf[1], val.ptr[0], val.ptr[1]);
						MT5715_write_buffer(mte, databuf[0], val.ptr, 2);
					}
				}
				break;
			}
		}
    }
    return len;
}


static ssize_t chip_version_show(struct device* dev, struct device_attribute* attr, char* buf)
{
    u8 fwver[2];
    ssize_t len = 0;// must to set 0
    MT5715_read_buffer(mte, REG_FW_VER, fwver, 2);
    len += snprintf(buf+len, PAGE_SIZE-len, "chip_version : %02x,%02x\n", fwver[0],fwver[1]);
    return len;
}

/* voltage limit attrs */
static ssize_t chip_vout_show(struct device* dev, struct device_attribute* attr, char* buf)
{
    unsigned char   fwver[2];
    unsigned short  vout_value;
    ssize_t len = 0;

    MT5715_read_buffer(mte, REG_VOUT, fwver,2);
    vout_value = fwver[0] << 8 |  fwver[1];
    pr_debug("chip Vout : %d\n", vout_value);
    len += snprintf(buf+len, PAGE_SIZE-len, "chip Vout : %d mV\n", vout_value);
    return len;
}

static ssize_t chip_vout_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t count)
{
    vuc val;
    int error;
    unsigned int temp;
    u8 vptemp;
    error = kstrtouint(buf, 10, &temp);
    if (error)
        return error;
    if( (temp < 0) ||( temp > 20000)){
        pr_debug(" Parameter error\n");
        return count;
    }
    val.value = temp;
    vptemp = val.ptr[0];
    val.ptr[0] = val.ptr[1];
    val.ptr[1] = vptemp;
    MT5715_write_buffer(mte, REG_VOUTSET, val.ptr,2);
    pr_err("Set Vout : %d \n", val.value);

    return count;
}


int  fast_sv_no_samsung(int temp){
	vuc val;
	int rc;
	u8  fcflag;
	//u8  count = 3;
    if( (temp < 0) ||( temp > 20000)){
        pr_debug(" Parameter error\n");
        return 0;
    }
	
#if 0
	//Raise the voltage to 9V
	val.value = temp;  // 9000  0x2328
    fcflag = val.ptr[0]; //swap
    val.ptr[0] = val.ptr[1];
    val.ptr[1] = fcflag;  //0x2823
    MT5715_write_buffer(mte, REG_VOUTSET, val.ptr, 2);
#endif

			//Raise the voltage to 9V
//#if defined (CONFIG_PROJECT_KOOBEE_K605)
	msleep(500);
//#endif
	MT5715_read_buffer(mte,REG_VOUT,val.ptr,2);
    rc = val.ptr[0] << 8 |  val.ptr[1];
	pr_err("%s: vol read  vol=%d\n", __func__,rc);
	
	if(rc > 8000) {
		return 1;
	}else{
		val.ptr[0] = 0x00;
	    val.ptr[1] = 0x05;
		MT5715_write_buffer(mte, 0x00F8, val.ptr, 2); //close chip autoflow,then. use AP cmd
	    val.value = temp;  // 9000  0x2328
	    fcflag = val.ptr[0]; //swap
	    val.ptr[0] = val.ptr[1];
	    val.ptr[1] = fcflag;  //0x2823
	    MT5715_write_buffer(mte, REG_VFC, val.ptr, 2);
	    pr_err("%s:FC send data step up 9V : 0x%02x,0x%02x \n",__func__, val.ptr[0],val.ptr[1]);
		val.ptr[0] = 0x00;
		val.ptr[1] = 0x10;
	    MT5715_write_buffer(mte, REG_CMD, val.ptr, 2);
		
	    pr_err("%s:fast_sv_no_samsung  step up REG_VFC 9V : 0x%02x,0x%02x \n", __func__,val.ptr[0],val.ptr[1]);
		return 0;
	}   
    return 1;    
	
}

EXPORT_SYMBOL(fast_sv_no_samsung);


int  ldo_disable(void){
	int voutval = 0;
	int ret = 0;
	vuc val;
	u8 fcflag;
	ret = MT5715_read_buffer(mte,REG_VOUT,val.ptr,2);
	if(ret){
		pr_err("%s: MT5715_read_buffer REG_VOUT ret  =%d  \n", __func__,ret);
		return (-1);
	}
    voutval = val.ptr[0] << 8 |  val.ptr[1];
	if(voutval > 2000){
		 val.ptr[0] = 0x00;
		 val.ptr[1] = LDOTGL;
		 ret = MT5715_write_buffer(mte, reg_access[CMD].addr, val.ptr, 2);
		 if (ret){
			pr_err("%s: MT5715_write_buffer ret =%d  step3\n", __func__,ret);
			return (-1);
		 }
		 val.value = 3200;  // 
         fcflag = val.ptr[0]; //
         val.ptr[0] = val.ptr[1]; 

		 
         val.ptr[1] = fcflag;  //
		 ret = MT5715_write_buffer(mte, REG_VOUTSET,val.ptr, 2);// 
		 if (ret){
			pr_err("%s: MT5715_write_buffer ret =%d  step4\n", __func__,ret);
			return (-1);
		 }
		
		 pr_err("%s: sucess  ret =%d \n", __func__,ret);
		 return 1;
	}else{
	    pr_err("%s: voutval < 2000  ret =%d \n", __func__,ret);
		return 0; 
	}
}
EXPORT_SYMBOL(ldo_disable);

//prize added by sunshuai, wireless charge MT5715  soft Raise the voltage to 9V, 20190307-start

int  get_mt5715_9V_charge_status(void){
	vuc val;
	int rc = 0;
	int lod_on_status = 0;
	
	lod_on_status = get_lod_status();
    MT5715_read_buffer(mte,REG_VOUT,val.ptr,2);
    rc = val.ptr[0] << 8 |  val.ptr[1];
	pr_err("%s: vol read  vol=%d\n", __func__,rc);	
	if((rc > 8000) && (lod_on_status == 1))
		return 1;
	else
		return 0;	
}

EXPORT_SYMBOL(get_mt5715_9V_charge_status);


//prize added by sunshuai, wireless charge MT5715  soft Raise the voltage to 9V, 20190307-end

int get_is_samsung_charge (void){
   pr_err("%s: is_samsung_charge %d\n", __func__, mte->is_samsung_charge);
   return  mte->is_samsung_charge;
}
EXPORT_SYMBOL(get_is_samsung_charge);

int set_is_samsung_charge(int temp){
    mte->is_samsung_charge =temp;
	return 1;
}
EXPORT_SYMBOL(set_is_samsung_charge);


void set_wireless_disable_flag(bool flag){
	mte->full_bat_disable_wireless = flag;
	if(flag == true){
		disable_irq(gpio_to_irq(mte->irq_gpio));
		pr_err("%s:  disable_irq\n", __func__);
	}
	else{
		enable_irq(gpio_to_irq(mte->irq_gpio));
		pr_err("%s:  enable_irq\n", __func__);
	}
	printk("%s:  flag =%s\n", __func__,flag == true ?"true":"false");
}
EXPORT_SYMBOL(set_wireless_disable_flag);


//prize added by sunshuai, wireless charge MT5715  soft Raise the voltage to 9V, 20190223-start
int  fast_sv(int temp){
	vuc val;
	vuc irq_flag;
	int rc;
	//u8  fcflag;
	MT5715_read_buffer(mte,REG_INT_FLAG,val.ptr,2);
	pr_err("%s: read irq status successful, 0x%02x%02x, 0x%04x is_samsung_charge=%d\n", __func__, val.ptr[0], val.ptr[1], val.value,mte->is_samsung_charge);

	
    if(val.ptr[0] & 0x01) {
		//Clear interrupt
		mte->is_samsung_charge =1;
		pr_err("%s: set is_samsung_charge =%d\n", __func__,mte->is_samsung_charge);
		irq_flag.ptr[0] = 0x01;
		irq_flag.ptr[1] = 0x00;
		rc = MT5715_write_buffer(mte, reg_access[INTCTLR].addr, irq_flag.ptr, 2); 
		if (rc){
			pr_err("%s: clean irq fail rc =%d  step1\n", __func__,rc);
			return (-1);
		}
		irq_flag.ptr[1] = CLRINT ;//prize modify by sunshuai  Put the 9V interrupt flag bit together with the 9V command   201900621
		MT5715_read_buffer(mte,REG_VRECT,val.ptr,2);
		mte->VrectFC = val.ptr[0] << 8 |  val.ptr[1];
        MT5715_write_buffer(mte, REG_CMD, irq_flag.ptr, 2);
		schedule_delayed_work(&mte->check_work, 50);
		schedule_delayed_work(&mte->check_afc_work,300);
    }

    MT5715_read_buffer(mte,REG_VOUT,val.ptr,2);
    rc = val.ptr[0] << 8 |  val.ptr[1];
	pr_err("%s: vol read  vol=%d\n", __func__,rc);
	
	if(rc > 8000) return 1;
	else          return 0;
    
	
}

EXPORT_SYMBOL(fast_sv);
//prize added by sunshuai, wireless charge MT5715  soft Raise the voltage to 9V, 20190223-end


static ssize_t fast_charging_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t count)
{
   // vuc val;
    int error;
    unsigned int temp;
    //u8  fcflag;
	pr_err("enter fast_charging_store \n");
	
    error = kstrtouint(buf, 10, &temp);//"9000"
    if (error)
        return error;
    if( (temp < 0) ||( temp > 20000)) {
        pr_err(" Parameter error\n");
        return count;
    }
	fast_sv(temp);
   // MT5715_read(mte, 9, &fcflag);
 /*   MT5715_read_buffer(mte,REG_INT_FLAG,val.ptr,2);
    if(val.ptr[0] & 0x01) {
		val.value = temp;  // 9000  0x2328
        fcflag = val.ptr[0]; //swap
        val.ptr[0] = val.ptr[1];
        val.ptr[1] = fcflag;  //0x2823
        MT5715_write_buffer(mte, REG_VFC, val.ptr, 2);
        pr_err("FC send data : 0x%02x,0x%02x \n", val.ptr[0],val.ptr[1]);
		val.ptr[0] = 0x00;
		val.ptr[1] = 0x10;
        MT5715_write_buffer(mte, REG_CMD, val.ptr, 2);
       // pr_debug("FC : %d \n", val.value);
       // MT5715_read_buffer(mte,REG_VFC,val.ptr,2);
       // pr_debug("FC read data : 0x%02x,0x%02x \n", val.ptr[0],val.ptr[1]);
    } else {
        pr_err("Fast charging is not supported \n");
    }*/
    return count;
}

static ssize_t fast_charging_show(struct device* dev, struct device_attribute* attr, char* buf)
{
    unsigned char   fwver[2];
    unsigned short  fc_value;
    ssize_t len = 0;

    MT5715_read_buffer(mte, REG_VFC, fwver,2);
    fc_value = fwver[0] << 8 |  fwver[1];
    pr_debug("FC read data : %d\n", fc_value);
    len += snprintf(buf+len, PAGE_SIZE-len, "FC read data : %d mV\n", fc_value);
    return len;
}

static DEVICE_ATTR(chip_version, S_IRUGO | S_IWUSR, chip_version_show, NULL);
static DEVICE_ATTR(chip_vout, S_IRUGO | S_IWUSR, chip_vout_show, chip_vout_store);
static DEVICE_ATTR(fast_charging,S_IRUGO | S_IWUSR, fast_charging_show, fast_charging_store);
static DEVICE_ATTR(reg,S_IRUGO | S_IWUSR, get_reg, set_reg);

static struct attribute* mt5715_sysfs_attrs[] = {
    &dev_attr_chip_version.attr,
    &dev_attr_chip_vout.attr,
    &dev_attr_fast_charging.attr,
    &dev_attr_reg.attr,//for debug
    NULL,
};
/* attr for debug */

static const struct attribute_group mt5715_sysfs_group = {
    .name = "mt5715group",
    .attrs = mt5715_sysfs_attrs,
};

static const struct of_device_id match_table[] = {
    {.compatible = "MT5715,MT5715_receiver",},
    { },
};

static const struct regmap_config MT5715_regmap_config = {
    .reg_bits = 16,
    .val_bits = 8,
    .max_register = 0xFFFF,  
};

//prize added by sunshuai, wireless charge MT5715  soft Used to detect whether the wireless charging chip 5715 is in working state, 20190621-start
int confirm_MT5715_works(void)
{
	vuc chips;
	int rc;
	int count = 3;
	while(count--){
		rc = MT5715_read_buffer(mte, REG_CHIPID, chips.ptr,2);
		if(rc){
			pr_err("%s: IIC failed  \n",__func__);
		}else{
			if((chips.ptr[0] == 0x57)&&(chips.ptr[1] == 0x15)){
				pr_err("%s: chipID : %02x%02x \n",__func__,chips.ptr[0],chips.ptr[1]);
				return 1;
			}
		}
	}
	return 0;
}
EXPORT_SYMBOL(confirm_MT5715_works);
//prize added by sunshuai, wireless charge MT5715  soft Used to detect whether the wireless charging chip 5715 is in working state, 20190621-end



static int MT5715_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    struct MT5715_dev *chip;
    int irq_flags = 0;
    int rc = 0;

    pr_err("MT5715 probe.\n");
    chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
    if (!chip)
        return -ENOMEM;

    //wake_lock_init(&chip->wakelock, WAKE_LOCK_SUSPEND, "wireless charger suspend wakelock");
    mutex_init(&chip->slock);

    pr_err("MT5715 chip.\n");
    chip->regmap = regmap_init_i2c(client, &MT5715_regmap_config);
    if (!chip->regmap) {
        pr_err("parent regmap is missing\n");
        return -EINVAL;
    }
    pr_err("MT5715 regmap.\n");
	
    chip->client = client;
    chip->dev = &client->dev;

    chip->bus.read = MT5715_read;
    chip->bus.write = MT5715_write;
    chip->bus.read_buf = MT5715_read_buffer;
    chip->bus.write_buf = MT5715_write_buffer;

    device_init_wakeup(chip->dev, true);
    
    sysfs_create_group(&client->dev.kobj, &mt5715_sysfs_group);
 
    pr_err("MT5715 probed successfully\n");

    mte = chip;
	mte->is_samsung_charge =0;
	mte->full_bat_disable_wireless = false;

    INIT_DELAYED_WORK(&chip->eint_work, MT5715_eint_work);
	INIT_DELAYED_WORK(&chip->check_work, MT5715_check_work);
//#if defined (CONFIG_PROJECT_KOOBEE_K605)
	INIT_DELAYED_WORK(&chip->check_afc_work, MT5715_check_afc_work);
//#endif
    rc = MT5715_parse_dt(client, chip);
    if (rc ) {
    	pr_err("%s: failed to parse device tree node\n", __func__);
        chip->dc_gpio = -1;
        chip->irq_gpio = -1;
    }

    if (gpio_is_valid(chip->dc_gpio)) {
        rc = devm_gpio_request_one(&client->dev, chip->dc_gpio,
              GPIOF_DIR_IN, "mt5715_dc");
        if (rc){
              pr_err("%s: dc_gpio request failed\n", __func__);
              goto err;
        }
    } else {
		pr_err("%s: dc_gpio %d is invalid\n", __func__, chip->dc_gpio);
    }

    if (gpio_is_valid(chip->irq_gpio)) {
        rc = devm_gpio_request_one(&client->dev, chip->irq_gpio,
              GPIOF_DIR_IN, "mt5715_int");
        if (rc) {
              pr_err("%s: irq_gpio request failed\n", __func__);
              goto err;
        }

        irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
        rc = devm_request_threaded_irq(&client->dev,
                          gpio_to_irq(chip->irq_gpio),
                          NULL, MT5715_irq, irq_flags,
                          "mt5715", chip);
        if (rc != 0) {
              pr_err("failed to request IRQ %d: %d\n",
                          gpio_to_irq(chip->irq_gpio), rc);
              goto err;
        }
		pr_err("sucess to request IRQ %d: %d\n",
                          gpio_to_irq(chip->irq_gpio), rc);

		//start add by sunshuai
		if(!(gpio_get_value(mte->irq_gpio))){
			pr_err("%s The interruption has come \n", __func__);
			MT5715_irq_handle();			
		}
		//end   add by sunshuai
    } else {
		pr_info("%s skipping IRQ registration\n", __func__);
    }

    //MT5715_read_buffer(mte, REG_CHIPID, chipid.ptr,2);
    //if(chipid.value == MT5715ID){
	//	pr_err("ID Correct query\n");
	//} else {
	//	pr_err("ID error :%d\n ", chipid.value);
	//}

	is_5715_probe_done = 1;

	err:
    return rc;
}

static int MT5715_remove(struct i2c_client *client) {

    
    sysfs_remove_group(&client->dev.kobj, &mt5715_sysfs_group);

    if (gpio_is_valid(mte->irq_gpio))
        devm_gpio_free(&client->dev, mte->irq_gpio);
    if (gpio_is_valid(mte->dc_gpio))
        devm_gpio_free(&client->dev, mte->dc_gpio);

    return 0;
}


static const struct i2c_device_id MT5715_dev_id[] = {
    {"MT5715_receiver", 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, MT5715_dev_id);

static struct i2c_driver MT5715_driver = {
    .driver   = {
        .name           = DEVICE_NAME,
        .owner          = THIS_MODULE,
        .of_match_table = match_table,
    },
    .probe    = MT5715_probe,
    .remove   = MT5715_remove,
    .id_table = MT5715_dev_id,
};
module_i2c_driver(MT5715_driver);

MODULE_AUTHOR("Yangwl@maxictech.com");
MODULE_DESCRIPTION("MT5715 Wireless Power Receiver");
MODULE_LICENSE("GPL v2");

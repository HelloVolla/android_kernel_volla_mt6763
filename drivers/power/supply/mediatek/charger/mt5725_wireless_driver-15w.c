/******************************************************************************
* file  MT5725 15W wireless charge  driver 
* Copyright (C) 2020 prize.
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
#include "mtk_charger_intf.h" //add by sunshuai

//#include <linux/wakelock.h>
//#include <extcon_usb.h>

#include "mt5725_wireless_15w.h"

#define DEVICE_NAME  "mt5725_iic"

#define DRIVER_FIRMWARE_VERSION "0.0.3"

#define OTP_PAGE_SIZE        128
#define EEPROM_PAGE_SIZE     32
#define SDRAM_PAGE_SIZE      128
#define OTP_BLOCK_SIZE       128
#define MAX_MSG_SIZE         20
#define FLASH_PAGE_SIZE      512
#define TRUE 1
#define FALSE 0

#define OTP_WRITE_FLAG_ADDR  0x3E00    //In the middle of code area and trim area

typedef union {
    u16 value;
    u8 ptr[2];
} vuc;

//The boot loader will update the Status in the SRAM as follows:
#define PGM_STATUS_READY     0x00 // reset value (from AP)
#define PGM_STATUS_WRITE     0x01 // buffer validated / busy (from AP)
#define PGM_STATUS_PROGOK    0x02 // finish OK (from the boot loader)
#define PGM_STATUS_ERRCS     0x04 // wrong check sum (from the boot loader)
#define PGM_STATUS_READ      0x08 // read flash
#define PGM_STATUS_WNVR      0x10 // write NVR
#define PGM_STATUS_ERRPGM    0x20
#define PGM_STATUS_DONE      0x80

#define PGM_STATUS_ADDR      0x0000
#define PGM_ADDR_ADDR        0x0002
#define PGM_LENGTH_ADDR      0x0004
#define PGM_CHECKSUM_ADDR    0x0006
#define PGM_DATA_ADDR        0x0008
extern void mtk_xhci_enable_vbus(void);
extern void mtk_xhci_disable_vbus(void);
extern int wireless_charge_chage_current(void);
void fast_vfc(int vol);

struct MT5725_dev *mte, *chip;
//struct delayed_work MT5725_int_delayed_work;
typedef struct Pgm_Type {
    u16 status;
    u16 addr;
    u16 length;
    u16 cs;
    u8 data[256];
} Pgm_Type_t;

struct pinctrl* mt5725_pinctrl;
struct pinctrl_state *mt5725_rsv0_low, *mt5725_rsv0_high;

struct MT5725_func {
    int (*read)(struct MT5725_dev* di, u16 reg, u8* val);
    int (*write)(struct MT5725_dev* di, u16 reg, u8 val);
    int (*read_buf)(struct MT5725_dev* di, u16 reg, u8* buf, u32 size);
    int (*write_buf)(struct MT5725_dev* di, u16 reg, u8* buf, u32 size);
};

struct otg_wireless_ctl {
	struct pinctrl *pinctrl_gpios;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *charger_otg_off, *charger_otg_on,*wireless_5725_off,*wireless_5725_on;
	bool gpio_otg_prepare;
};

enum wireless_charge_protocol {
	PROTOCOL_UNKNOWN = 0,
	BPP,
	EPP,
	AFC,
};

struct MT5725_dev {
    char  *name;
    struct i2c_client* client;
    struct device* dev;
    struct regmap* regmap;
    struct MT5725_func bus;
    struct device_node	*irq_nd;     /* node */
    struct delayed_work eint_work;
	struct delayed_work add_current_work;
    int    irq_gpio;
	int    statu_gpio;
    int fsk_status;
	enum wireless_charge_protocol charge_protocol;
	int wireless_max_power;
	int input_current;
	int charge_current;
    int   tx_count;   //Enable TX function, write 0 for this value
    struct otg_wireless_ctl otg_5725_ctl;
};

#define REG_NONE_ACCESS 0
#define REG_RD_ACCESS (1 << 0)
#define REG_WR_ACCESS (1 << 1)
#define REG_BIT_ACCESS (1 << 2)

#define REG_MAX 0x0F

struct reg_attr {
    const char* name;
    u16 addr;
    u8 flag;
};

enum REG_INDEX {
    CHIPID = 0,
    VOUT,
    INT_FLAG,
    INTCTLR,
    VOUTSET,
    VFC,
    CMD,
    INDEX_MAX,
};

static struct reg_attr reg_access[INDEX_MAX]={
    [CHIPID] = { "CHIPID", REG_CHIPID, REG_RD_ACCESS },
    [VOUT] = { "VOUT", REG_VOUT, REG_RD_ACCESS },
    [INT_FLAG] = { "INT_FLAG", REG_INTFLAG, REG_RD_ACCESS },
    [INTCTLR] = { "INTCLR", REG_INTCLR, REG_WR_ACCESS },
    [VOUTSET] = { "VOUTSET", REG_VOUTSET, REG_RD_ACCESS | REG_WR_ACCESS },
    [VFC] = { "VFC", REG_VFC, REG_RD_ACCESS | REG_WR_ACCESS },
    [CMD] = { "CMD", REG_CMD, REG_RD_ACCESS | REG_WR_ACCESS | REG_BIT_ACCESS },
};

static u32 SizeofPkt(u8 hdr) {
    if (hdr < 0x20)
        return 1;

    if (hdr < 0x80)
        return (2 + ((hdr - 0x20) >> 4));

    if (hdr < 0xe0)
        return (8 + ((hdr - 0x80) >> 3));

    return (20 + ((hdr - 0xe0) >> 2));
}



static int MT5725_read(struct MT5725_dev* di, u16 reg, u8* val) {
    unsigned int temp;
    int rc;

    rc = regmap_read(di->regmap, reg, &temp);
    if (rc >= 0)
        *val = (u8)temp;

    return rc;
}

static int MT5725_write(struct MT5725_dev* di, u16 reg, u8 val) {
    int rc = 0;

    rc = regmap_write(di->regmap, reg, val);
    if (rc < 0)
        dev_err(di->dev, "MT5725 write error: %d\n", rc);

    return rc;
}

static int MT5725_read_buffer(struct MT5725_dev* di, u16 reg, u8* buf, u32 size) {
  
    return regmap_bulk_read(di->regmap, reg, buf, size);
}

static int MT5725_write_buffer(struct MT5725_dev* di, u16 reg, u8* buf, u32 size) {
    int rc = 0;
    
    while (size--) {
        rc = di->bus.write(di, reg++, *buf++);
        if (rc < 0) {
            dev_err(di->dev, "MT5725 write error: %d\n", rc);
            return rc;
        }
    }
  

    return rc;
}

static void MT5725_sram_write(u32 addr, u8 *data,u32 len) {
    u32 offset,length,size;
    //vuc val;
    offset = 0;
    length = 0;
    size = len;
    pr_info("[%s] Length to write:%d\n",__func__, len);
    while(size > 0) {
        if(size > SRAM_PAGE_SIZE) {
            length = SRAM_PAGE_SIZE;
        } else {
            length = size;
        }
        pr_info("[%s] Length of this write :%d\n",__func__,length);
		MT5725_write_buffer(mte, addr + offset ,data+offset, length);
        size -= length;
        offset += length;
        msleep(2);
    }
    pr_info("[%s] Write completion\n",__func__);
}


static void MT5725_run_pgm_fw(void) {
    vuc val;
    //wdg_disable
    val.value  = MT5725_WDG_DISABLE;
    MT5725_write_buffer(mte, REG_PMU_WDGEN, val.ptr, 2);
    MT5725_write_buffer(mte, REG_PMU_WDGEN, val.ptr, 2);
	MT5725_write_buffer(mte, REG_PMU_WDGEN, val.ptr, 2);
    val.value  = MT5725_WDT_INTFALG;
    MT5725_write_buffer(mte, REG_PMU_FLAG, val.ptr, 2);
    //sys_remap
    val.value = 0X57;
    MT5725_write_buffer(mte, REG_SYS_KEY, val.ptr, 2);

	val.value = 0x0FFF;
	MT5725_write_buffer(mte, REG_SRAM_REMAP, val.ptr, 2);
	val.value = 0x08;
    MT5725_write_buffer(mte, REG_CODE_REMAP, val.ptr, 2);
    msleep(50);
    //sram_write
    MT5725_sram_write(0x1800,(u8 *)mt5725_pgm_bin,sizeof(mt5725_pgm_bin));
    //sys_run
    msleep(50);
    val.value = MT5725_M0_RESET;
    MT5725_write_buffer(mte, REG_M0_CTRL, val.ptr, 2);
    msleep(50);
	pr_info("[%s]  finish  \n",__func__);
}


static u8 MT5725_otp_read(u32 addr, u8 * buf , u32 size, u8 mode) {
    u32 length ,i,status,times;
	//u32 j;
    vuc val;
    Pgm_Type_t pgm;
    pr_info("[%s] parameter size :%d\n",__func__,size);
    length = (size+(OTP_BLOCK_SIZE-1))/OTP_BLOCK_SIZE*OTP_BLOCK_SIZE;
    MT5725_run_pgm_fw();
	pr_info("[%s] Calculate the length to read:%d\n",__func__,length);
    for (i = 0; i < length/OTP_BLOCK_SIZE; ++i) {
        pgm.length = OTP_BLOCK_SIZE;
        pgm.addr = addr+i*OTP_BLOCK_SIZE;
        pgm.status = PGM_STATUS_READY;
        val.value = pgm.status;
        MT5725_write_buffer(mte,PGM_STATUS_ADDR,val.ptr,2);
        val.value = pgm.addr;
        MT5725_write_buffer(mte,PGM_ADDR_ADDR,val.ptr,2);
        val.value = pgm.length;
        MT5725_write_buffer(mte,PGM_LENGTH_ADDR,val.ptr,2);

        val.value = PGM_STATUS_READ;
        MT5725_write_buffer(mte,PGM_STATUS_ADDR,val.ptr,2);
        msleep(50);
        MT5725_read_buffer(mte, PGM_STATUS_ADDR, val.ptr, 2);
        status = val.ptr[0];
        times = 0;
        while(status == PGM_STATUS_READ) {
            msleep(50);
            MT5725_read_buffer(mte, PGM_STATUS_ADDR, val.ptr, 2);
			pr_info("[%s] Program reading",__func__);
            status = val.ptr[0];
            times+=1;
            if (times>100) {
                pr_err("[%s] error! Read OTP TImeout\n",__func__);
                return FALSE;
            }
        }
        if (status == PGM_STATUS_PROGOK) {
            pr_info("[%s] PGM_STATUS_PROGOK\n",__func__);
			MT5725_read_buffer(mte, PGM_DATA_ADDR, &buf[OTP_BLOCK_SIZE*i], OTP_BLOCK_SIZE);
        } else {
		    pr_err("[%s] OtpRead error , status = 0x%02x\n",__func__,status);
            return FALSE;
        }
        if (mode == TRUE) {
            /* code */
        }
    }
	return TRUE;
}

static u8 MT5725_otp_write(u32 addr, u8 * buf , u32 len) {
    u32 offset;
    u32 write_size , size ,status,times,zero_cnt;
    u32 write_retrycnt;
    int i;
    vuc val;
    Pgm_Type_t pgm;
    size = len;
    offset = 0;
    write_size = 0;
    pr_info("[%s] Size to write:%d\n",__func__,size);
    MT5725_run_pgm_fw();
    write_retrycnt = 8;
    while(size>0) {
        if (size>OTP_BLOCK_SIZE) {
            pgm.length = OTP_BLOCK_SIZE;
            write_size = OTP_BLOCK_SIZE;
        } else {
            pgm.length = size;
            write_size = size;
        }
        pgm.addr = addr+offset;
        pgm.cs = pgm.addr;
        pgm.status = PGM_STATUS_READY;
        for (i = 0; i < pgm.length; ++i) {
            pgm.data[i] = buf[offset + i];
            pgm.cs += pgm.data[i];
        }
        zero_cnt = 0;
        for (i = (pgm.length -1); i >= 0; --i) {
            if (pgm.data[i] != 0x00) {
                break;
            }
            zero_cnt+=1;
        }
        if(zero_cnt == pgm.length) {
            size-=write_size;
            offset+=write_size;
            continue;
        }
        pgm.length-=zero_cnt;
        pgm.cs+=pgm.length;
        val.value = pgm.status;
        MT5725_write_buffer(mte,PGM_STATUS_ADDR,val.ptr,2);
        val.value = pgm.addr;
        MT5725_write_buffer(mte,PGM_ADDR_ADDR,val.ptr,2);
        val.value = pgm.length;
        MT5725_write_buffer(mte,PGM_LENGTH_ADDR,val.ptr,2);
        val.value = pgm.cs;
        MT5725_write_buffer(mte,PGM_CHECKSUM_ADDR,val.ptr,2);
        MT5725_write_buffer(mte, PGM_DATA_ADDR, pgm.data, pgm.length);
        val.value = PGM_STATUS_WRITE;
        MT5725_write_buffer(mte,PGM_STATUS_ADDR,val.ptr,2);
        msleep(50);
        MT5725_read_buffer(mte, PGM_STATUS_ADDR, val.ptr, 2);
        status = val.ptr[0];
        times = 0;
        while(status == PGM_STATUS_WRITE) {
            msleep(50);
            MT5725_read_buffer(mte, PGM_STATUS_ADDR, val.ptr, 2);
            status = val.ptr[0];
            pr_info("[%s] Program writing\n",__func__);
            times+=1;
            if (times > 100) {
                pr_err("[%s] Program write timeout\n",__func__);
                return FALSE;
            }

        }
        if (status == PGM_STATUS_PROGOK) {
            size-=write_size;
            offset+=write_size;
            pr_info("[%s] PGM_STATUS_PROGOK\n",__func__);
        } else if (status == PGM_STATUS_ERRCS) {
            if (write_retrycnt > 0) {
                write_retrycnt--;
                pr_err("[%s]  ERRCS write_retrycnt:%d\n",__func__,write_retrycnt);
                continue;
            } else {
            pr_err("[%s] PGM_STATUS_ERRCS\n",__func__);
            return FALSE;
            }
        } else if (status == PGM_STATUS_ERRPGM) {
            if (write_retrycnt > 0) {
                write_retrycnt--;
                pr_err("[%s] ERRPGM write_retrycnt:%d\n",__func__,write_retrycnt);
                continue;
            } else {
            pr_err("[%s] PGM_STATUS_ERRPGM\n",__func__);
            return FALSE;
            }
        } else {
            if (write_retrycnt > 0) {
                write_retrycnt--;
                pr_err("[%s] NUKNOWN write_retrycnt:%d\n",__func__,write_retrycnt);
                continue;
            } else {
                pr_err("[%s] PGM_STATUS_NUKNOWN \n",__func__);
                return FALSE;
            }
        }
    }
	return TRUE;
}

static u8 MT5725_otp_write_check(u32 flagaddr) {
    u8 i;
    u8 otpwrite_flagdata[4] = {0x5a,0xa5,0x5a,0xa5};
    u8 *otpwrite_flagread;
	otpwrite_flagread = kmalloc(1024, GFP_KERNEL);

    MT5725_otp_read(flagaddr, otpwrite_flagread,4,1);
    for (i = 0; i < 4; ++i) {
        if (otpwrite_flagread[i] != otpwrite_flagdata[i]) {
            pr_info("[%s] MT5725 OTP  Flag not written: %d\n",__func__,i);
            return FALSE;
        }
    }
    printk("[%s] MT5725 OTP Flag has been written",__func__);
	kfree(otpwrite_flagread);
    return TRUE;
}

void MT5725_write_otpok_flag(u32 flagaddr) {
    u8 otpwrite_flagdata[4] = {0x5a,0xa5,0x5a,0xa5};
    if (flagaddr < 0x3700) {
        pr_err("[%s] Address out of range\n",__func__);
        return;
    }
    MT5725_otp_write(flagaddr,otpwrite_flagdata,4);
}

static u8 MT5725_otp_verify(u32 addr,u8 * data,u32 len) {
    u8 *otp_read_temp;
    u32 i;
	otp_read_temp = kmalloc(1024*17, GFP_KERNEL);
	if(otp_read_temp == NULL){
		pr_err("[%s] devm_kzalloc Error\n",__func__);
		return FALSE;
	}
    if (len > 1024*17) {
        pr_err("[%s] Wrong parameter length\n",__func__);
        return FALSE;
    }
    MT5725_otp_read(addr, otp_read_temp,len, 1);
    for (i = 0; i < len; ++i) {
        if (data[i] != otp_read_temp[i]) {
            pr_err("[%s]  FALSE Raw_data:0x%02x,read_data:0x%02x,addr:0x%02x",__func__,data[i],otp_read_temp[i],i);
            return FALSE;
        }
    }/**/
    kfree(otp_read_temp);
    printk("[%s]  success",__func__);
    return TRUE;
}


/*
  Send proprietary packet to Tx
  Step 1: write data into REG_PPP
  Step 2: write REG_CMD
*/
void MT5725_send_ppp(PktType *pkt) {
    vuc val;
    
    MT5725_write_buffer(mte, REG_PPP, (u8 *)pkt, SizeofPkt(pkt->header)+1);
    mte->fsk_status = FSK_WAITTING;
    val.value = SEND_PPP;
    MT5725_write_buffer(mte, REG_CMD, val.ptr, 2);
}
EXPORT_SYMBOL(MT5725_send_ppp);



int Get_adaptertype(void){
    PktType eptpkt;
    int count = 0;
    u8 fsk_msg[10];
    eptpkt.header  = PP18;
    eptpkt.msg_pakt.cmd     = CMD_ADAPTER_TYPE;
    MT5725_send_ppp(&eptpkt);
    while(mte->fsk_status == FSK_WAITTING){
        msleep(20);
        if((count++) > 50 ){
            pr_err("[%s] AP system judgement:FSK receive timeout \n",__func__);
            return (-1);
        }
    }
    if(mte->fsk_status == FSK_FAILED){
        pr_err("[%s] Wireless charging system judgement:FSK receive timeout \n",__func__);
        return (-1);
    }
    if(mte->fsk_status == FSK_SUCCESS){
        MT5725_read_buffer(mte,REG_BC,fsk_msg,10);
        pr_info("[%s] Information received : 0x%02x 0x%02x 0x%02x \n",__func__,fsk_msg[0],fsk_msg[1],fsk_msg[2]);
    }
    return fsk_msg[2];
}

static ssize_t get_reg(struct device* cd, struct device_attribute* attr, char* buf) {
    vuc val;
    ssize_t len = 0;
    int i = 0;

    for (i = 0; i < INDEX_MAX; i++) {
        if (reg_access[i].flag & REG_RD_ACCESS) {
            MT5725_read_buffer(mte, reg_access[i].addr, val.ptr, 2);
            len += snprintf(buf + len, PAGE_SIZE - len, "reg:%s 0x%04x=0x%04x,%d\n", reg_access[i].name, reg_access[i].addr, val.value,val.value);
        }
    }

    return len;
}

static ssize_t set_reg(struct device* cd, struct device_attribute* attr, const char* buf, size_t len) {
    unsigned int databuf[2];
    vuc val;
    u8 tmp[2];
    u16 regdata;
    int i = 0;
    int ret = 0;

    ret = sscanf(buf, "%x %x", &databuf[0], &databuf[1]);

    if (2 == ret) {
        for (i = 0; i < INDEX_MAX; i++) {
            if (databuf[0] == reg_access[i].addr) {
                if (reg_access[i].flag & REG_WR_ACCESS) {
                    // val.ptr[0] = (databuf[1] & 0xff00) >> 8;
                    val.value = databuf[1];
                    // val.ptr[1] = databuf[1] & 0x00ff; //big endian
                    if (reg_access[i].flag & REG_BIT_ACCESS) {
                        MT5725_read_buffer(mte, databuf[0], tmp, 2);
                        regdata = tmp[0] << 8 | tmp[1];
                        val.value |= regdata;
                        pr_info("get reg: 0x%04x  set reg: 0x%04x \n", regdata, val.value);
                        MT5725_write_buffer(mte, databuf[0], val.ptr, 2);
                    } else {
                        pr_info("Set reg : [0x%04x]  0x%x \n", databuf[0], val.value);
                        MT5725_write_buffer(mte, databuf[0], val.ptr, 2);
                    }
                }
                break;
            }
        }
    }else{
        pr_info("Error \n");
    }
    return len;
}

static ssize_t fast_charging_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t count) {
    vuc val;
    int error;
    unsigned int a;
    error = kstrtouint(buf, 10, &a);
    val.value = (unsigned short)a;

    if (error)
        return error;
    if ((val.value < 0) || (val.value > 20000)) {
        pr_info("[%s] MT5725 Parameter error\n",__func__);
        return count;
    }
    fast_vfc(val.value);
    return count;
}


static ssize_t get_adapter(struct device* cd, struct device_attribute* attr, char* buf) {
    ssize_t len = 0;
    int rc;
    rc = Get_adaptertype();
    if(rc == (-1)){
        len += snprintf(buf + len, PAGE_SIZE - len, "[%s] Failed to read adapter type\n", __func__);
    }else if(rc == ADAPTER_NONE ){
        len += snprintf(buf + len, PAGE_SIZE - len, "[%s] Adapter type : Unknown\n", __func__);
    }else if(rc == ADAPTER_SDP ){
        len += snprintf(buf + len, PAGE_SIZE - len, "[%s] Adapter type : SDP\n", __func__);
    }else if(rc == ADAPTER_CDP ){
        len += snprintf(buf + len, PAGE_SIZE - len, "[%s] Adapter type : CDP\n", __func__);
    }else if(rc == ADAPTER_DCP ){
        len += snprintf(buf + len, PAGE_SIZE - len, "[%s] Adapter type : DCP\n", __func__);
    }else if(rc == ADAPTER_QC20 ){
        len += snprintf(buf + len, PAGE_SIZE - len, "[%s] Adapter type : QC2.0\n", __func__);
    }else if(rc == ADAPTER_QC30 ){
        len += snprintf(buf + len, PAGE_SIZE - len, "[%s] Adapter type : QC3.0\n", __func__);
    }else if(rc == ADAPTER_PD ){
        len += snprintf(buf + len, PAGE_SIZE - len, "[%s] Adapter type : PD\n", __func__);
    }else if(rc == ADAPTER_FCP ){
        len += snprintf(buf + len, PAGE_SIZE - len, "[%s] Adapter type : FCP\n", __func__);
    }else if(rc == ADAPTER_SCP ){
        len += snprintf(buf + len, PAGE_SIZE - len, "[%s] Adapter type : SCP\n", __func__);
    }else if(rc == ADAPTER_DCS ){
        len += snprintf(buf + len, PAGE_SIZE - len, "[%s] Adapter type : DC source\n", __func__);
    }else if(rc == ADAPTER_AFC ){
        len += snprintf(buf + len, PAGE_SIZE - len, "[%s] Adapter type : AFC\n", __func__);
    }else if(rc == ADAPTER_PDPPS ){
        len += snprintf(buf + len, PAGE_SIZE - len, "[%s] Adapter type : PD PPS\n", __func__);
    }
    return len;
}


static ssize_t brushfirmware(struct device* dev, struct device_attribute* attr, const char* buf, size_t count) {
    int error;
    unsigned int pter;
    error = kstrtouint(buf, 10, &pter);
    if(pter == 1){
        pr_info("[%s] Only brush BOOT program\n",__func__);
		pr_info("[%s] FW BOOT ERROR \n",__func__);
       /* if(MT5725_otp_write(0x0000,(u8 *)MT5725_onlg_boot_bin,sizeof(MT5725_onlg_boot_bin))){
            pr_info("[%s] Write complete, start verification \n",__func__);
            if (MT5725_otp_verify(0x0000,(u8 *)MT5725_onlg_boot_bin,sizeof (MT5725_onlg_boot_bin))) {
                pr_info("[%s] MMT5725_otp_verify OK \n",__func__);
            } else {
                pr_err("[%s] MT5725_otp_verify check program failed \n",__func__);
            }
        }*/
    }else if(pter == 2){
        pr_info("[%s] brush FORCEOTP program\n",__func__);
        if (MT5725_otp_write_check(OTP_WRITE_FLAG_ADDR) == TRUE) {
            pr_info("[%s] MT5725_otp_write_check Done\n",__func__);
        } else {
            if(MT5725_otp_write(0x0000,(u8 *)MT5725_forceotp_bin,sizeof(MT5725_forceotp_bin))){
                pr_info("[%s] Write complete, start verification \n",__func__);
                if (MT5725_otp_verify(0x0000,(u8 *)MT5725_forceotp_bin,sizeof (MT5725_forceotp_bin))) {
                    pr_info("[%s] MMT5725_otp_verify OK \n",__func__);
                    MT5725_write_otpok_flag(OTP_WRITE_FLAG_ADDR);
                    pr_info("[%s] MT5725_write_otp_flag \n",__func__);
                } else {
                    pr_err("[%s] MT5725_otp_verify check program failed \n",__func__);
                }
            }
        }
        
    }
    pr_info("[%s] Exit this operation \n",__func__);
    return count;
}


static DEVICE_ATTR(fast_charging, S_IRUGO | S_IWUSR, NULL, fast_charging_store);
static DEVICE_ATTR(reg, S_IRUGO | S_IWUSR, get_reg, set_reg);
static DEVICE_ATTR(adapter_type,S_IRUGO | S_IWUSR,get_adapter,NULL);
static DEVICE_ATTR(brushFW,S_IRUGO | S_IWUSR,NULL,brushfirmware);

void fast_vfc(int vol) {
    vuc val;
    val.value = vol;
    MT5725_write_buffer(mte, REG_VFC, val.ptr, 2);
    val.value = FAST_CHARGE;
    MT5725_write_buffer(mte, REG_CMD, val.ptr, 2);
    pr_info("%s,write reg_cmd : 0x%04x,\n", __func__, val.value);
}

void fastcharge_afc(void) {
    vuc val;
    vuc temp, fclr, scmd;
    scmd.value = 0;

    MT5725_read_buffer(mte, REG_INTFLAG, val.ptr, 2);
    fclr.value = FAST_CHARGE;
    if (val.value & INT_AFC) {
        pr_info("MT5725 %s ,version 0.1 Tx support samsung_afc\n", __func__);
        temp.value = 9000;
        MT5725_write_buffer(mte, REG_VFC, temp.ptr, 2);
        scmd.value |= FAST_CHARGE;
        scmd.value |= CLEAR_INT;
        MT5725_write_buffer(mte, REG_INTCLR, fclr.ptr, 2);
        pr_info("%s,version 0.1 write reg_clr : 0x%04x,\n", __func__, fclr.value);
        MT5725_write_buffer(mte, REG_CMD, scmd.ptr, 2);
        pr_info("%s,version 0.1 write reg_cmd : 0x%04x,\n", __func__, scmd.value);
    }
}

void download_code(void) {
    //?
}

static struct attribute* mt5725_sysfs_attrs[] = {
    &dev_attr_fast_charging.attr,
    &dev_attr_reg.attr,
    &dev_attr_adapter_type.attr,
    &dev_attr_brushFW.attr,
    NULL,
};

static const struct attribute_group mt5725_sysfs_group = {
    .name  = "mt5725group",
    .attrs = mt5725_sysfs_attrs,
};

static const struct regmap_config MT5725_regmap_config = {
    .reg_bits     = 16,
    .val_bits     = 8,
    .max_register = 0xFFFF,
};





/**
 * [MT5725_send_EPT End Power Transfer Packet]
 * @param endreson [end powr Reson]
 */
void MT5725_send_EPT(u8 endreson) {
    PktType eptpkt;
    eptpkt.header  = PP18;
    eptpkt.msg_pakt.cmd     = ENDPOWERXFERPACKET;
    eptpkt.msg_pakt.data[0] = endreson;
    MT5725_send_ppp(&eptpkt);
}

void Set_staystate_current(void)
{	
	pr_info("[%s],call\n", __func__);
	
	wireless_charge_chage_current();
	return;
}
EXPORT_SYMBOL(Set_staystate_current);

int get_mt5725_voltage(void){
	vuc val;
	if(gpio_get_value(mte->statu_gpio)){
		MT5725_read_buffer(mte,REG_VOUT,val.ptr,2);
		pr_err("%s: vol read  vol=%d\n", __func__,val.value);
	}
	return val.value;
}
EXPORT_SYMBOL(get_mt5725_voltage);

int get_mt5725_Iout(void){
	vuc val;
	if(gpio_get_value(mte->statu_gpio)){
		MT5725_read_buffer(mte,REG_IOUT,val.ptr,2);
		pr_err("%s: vol read  vol=%d\n", __func__,val.value);
	}
	return val.value;
}


void En_Dis_add_current(int i){
	vuc protocol;
	protocol.ptr[0] = i; 
	protocol.ptr[1] = 6;//default value in Firmware IIC address REG_CURFUNC+1
	printk(" En_Dis_add_current i=%d  protocol.ptr[0]=%02x\n",i,protocol.ptr[0]);
	MT5725_write_buffer(mte, REG_CURFUNC, protocol.ptr,1);
}
EXPORT_SYMBOL(En_Dis_add_current);

static void MT5725_add_current(void){
	vuc protocol;
	vuc epp;
	long voltage = 9000;
	long tcurrent =0;
	long tvoltage =0;
	long maxpower =0;
	long powertemp =0;
	long maxchargecurrent = 1000000;
	epp.value =0;
    protocol.value =0;
	//mte->charge_current = 1000000;
	tvoltage = get_mt5725_voltage();
	pr_err("[%s] Rx Vout:%d\n", __func__,tvoltage);
	tcurrent = get_mt5725_Iout();
	pr_err("[%s] Rx Iout:%d\n", __func__,tcurrent);
	if(gpio_get_value(mte->statu_gpio)){
		 MT5725_read_buffer(mte, 0x0098, protocol.ptr,1);
		 pr_err("%s: protocol read 1 : %02x%02x  protocol.value =0x%04x  protocol.value =%d\n",__func__,protocol.ptr[0],protocol.ptr[1],protocol.value,protocol.value);
		 if(protocol.ptr[0]== 1){
		 	mte->charge_protocol = BPP;
			mte->wireless_max_power = 5;
			voltage = 5000;
			maxchargecurrent = 1000000;
			pr_info("[%s] BPP Load power is recommended to be less than %dW\n", __func__,mte->wireless_max_power);
		 } else if(protocol.ptr[0] == 2){
		    mte->charge_protocol = AFC;
			mte->wireless_max_power = 10;
			voltage = 9000;
			maxchargecurrent = 1100000;
			pr_info("[%s] AFC Load power is recommended to be less than %dW\n", __func__,mte->wireless_max_power);
		 } else if(protocol.ptr[0] == 3){
		    mte->charge_protocol = EPP;
			MT5725_read_buffer(mte,REG_MAXPOWER,epp.ptr,1);
		    epp.ptr[0] = epp.ptr[0]/2;
			voltage = 9000;
		    pr_info("[%s]  EPP Load power is recommended to be less than %dW\n", __func__,epp.ptr[0]);
		    mte->wireless_max_power = epp.ptr[0];
			if(mte->wireless_max_power > 15) mte->wireless_max_power = 15;
			maxchargecurrent = mte->wireless_max_power * 1000 / 9 * 92 *10;
		 } else {
			chr_err("%s read 0x0098 info no BPP EPP AFC protocol\n", __func__);
		 }
     }
	// if(mte->wireless_max_power ==  5)  maxpower = mte->wireless_max_power * 1000;
     //else                               maxpower = mte->wireless_max_power * 1000;    
     maxpower = mte->wireless_max_power * 1000 * 98 / 100;    
	 pr_err("[%s]: wireless system support power  = %d W\n",__func__,mte->wireless_max_power);
	 pr_err("[%s]: Max charge maxchargecurrent = %d mA\n",__func__,(maxchargecurrent/1000));
	 pr_err("[%s]: Get wireless output voltage = %d mV\n",__func__,voltage);
	 tcurrent = mte->input_current / 1000;
     pr_err("[%s]: last time,Set input tcurrent = %d mA\n",__func__,tcurrent);	 
	 if(tcurrent == 0) tcurrent = 100;
	 powertemp = voltage * tcurrent / 1000 ;
	 pr_err("[%s]: powertemp = %d , maxpower = %d\n",__func__,powertemp,maxpower);
	 if(powertemp < maxpower ){
	 	powertemp = powertemp + 2000;
		if(powertemp > maxpower) {
			powertemp = maxpower;
			pr_info("[%s] DISABLE_ADD_CURRENT INT!\n", __func__);
			En_Dis_add_current(DISABLE_ADD_CURRENT);
		}
		pr_info("[%s] add power  = %d mW\n", __func__,powertemp);
	 	mte->input_current = powertemp * 1000 / voltage * 1000;
		if(mte->charge_protocol == BPP){
			if(mte->input_current > 1000000) mte->input_current = 1000000;
		}
		if(mte->charge_protocol == AFC){
			if(mte->input_current > 1100000) mte->input_current = 1100000;
		}
		if(mte->input_current > maxchargecurrent)  mte->input_current= maxchargecurrent;
		mte->charge_current = powertemp / 4 * 1000;
		pr_info("[%s] Set input_current =  %d, Set charge_current = %d\n", __func__,mte->input_current,mte->charge_current);
		Set_staystate_current();
		schedule_delayed_work(&mte->add_current_work,100);
	 }else{
	 	pr_info("[%s] return!\n", __func__);
		En_Dis_add_current(DISABLE_ADD_CURRENT);
	 	return;
	 }
	
}

void print_curfunc_info(void){    
	vuc protocol;
    protocol.value =0;
    if(gpio_get_value(mte->statu_gpio)){
		MT5725_read_buffer(mte, REG_CURFUNC, protocol.ptr,1);
		pr_err("%s: protocol read 1 : %02x%02x  protocol.value =0x%04x  protocol.value =%d\n",__func__,protocol.ptr[0],protocol.ptr[1],protocol.value,protocol.value);
    }else{
        pr_err("%s: statu_gpio is low\n",__func__);
	}
}

void MT5725_irq_handle(void) {
    vuc val;
    vuc temp, fclr, scmd;
    int iic_rf;
    scmd.value = 0;
    pr_info("----------------MT5725_delayed_work-----------------------\n");
    temp.value = MT5725ID;
    iic_rf = MT5725_write_buffer(mte,REG_CHIPID,temp.ptr,2);
    if(iic_rf < 0){
        pr_err("[%s] Chip may not be working\n",__func__);
        return;
    }
    MT5725_read_buffer(mte,REG_FW_VER,temp.ptr,2);
    if(temp.ptr[1] & RXMODE){
        pr_info("[%s] The chip works in Rx mode\n",__func__);
        MT5725_read_buffer(mte, REG_INTFLAG, val.ptr, 2);
        fclr.value = val.value;
        pr_info("[%s] REG_INTFLAG value:0x%04x\n", __func__, val.value);
        if(val.value == 0){
            pr_info("[%s] There's no interruption here\n", __func__);
            return;
        }
        print_curfunc_info();	
        if(val.value & INT_OCP){
            pr_info("[%s] Interrupt signal: INT_OCP,Load current too high\n", __func__);
        }
        if(val.value & INT_PLDO){
            pr_info("[%s] Interrupt signal: INT_PLDO,LDO over power protection\n", __func__);
        }
        if (val.value & INT_POWER_ON) {
			pr_info("[%s] Interrupt signal: PowerON 01\n", __func__);
			mte->wireless_max_power = 0;			
			mte->charge_protocol = PROTOCOL_UNKNOWN;
			mte->input_current = 500000;
	        mte->charge_current = 500000;
			//Set_staystate_current();
            pr_info("[%s] Interrupt signal: PowerON 02\n", __func__);
        }
        if (val.value & INT_LDO_ON) {			
            pr_info("[%s] Interrupt signal:LDO ON\n", __func__);
        }
        if (val.value & INT_RX_READY) {
            pr_info("[%s] Interrupt signal:MT5725 is Ready\n", __func__);
        }
        if (val.value & INT_LDO_OFF) {
            pr_info("[%s] Interrupt signal:MT5725 LDO_OFF\n", __func__);
        }
        if (val.value & INT_FSK_RECV) {
            pr_info("[%s] Interrupt signal:FSK received successfully\n", __func__);
            mte->fsk_status = FSK_SUCCESS;
            //read REG_BC
        }
        if (val.value & INT_FSK_SUCCESS) {
            pr_info("[%s] Interrupt signal:FSK received successfully\n", __func__);
            mte->fsk_status = FSK_SUCCESS;
            //read REG_BC
        }
        if (val.value & INT_FSK_TIMEOUT) {
            pr_info("[%s] Interrupt signal:Failed to receive FSK\n", __func__);
            mte->fsk_status = FSK_FAILED;
            //read REG_BC
        }
        if(val.value & INT_BPP){
            pr_info("[%s] Interrupt signal:Tx BPP\n", __func__);
            pr_info("[%s] Load power is recommended to be less than 5W\n", __func__);
			mte->wireless_max_power = 5;
			mte->charge_protocol = BPP;
			mte->input_current = 500000;
	        mte->charge_current = 500000;
			//Set_staystate_current();
			//schedule_delayed_work(&mte->add_current_work,400);
        }
        if(val.value & INT_EPP){
            vuc epp;
            pr_info("[%s] Interrupt signal:Tx EPP\n", __func__);
            MT5725_read_buffer(mte,REG_MAXPOWER,epp.ptr,1);
            epp.ptr[0] = epp.ptr[0]/2;
            pr_info("[%s] Load power is recommended to be less than %dW\n", __func__,epp.ptr[0]);
			mte->wireless_max_power = epp.ptr[0];
			mte->charge_protocol = EPP;
			mte->input_current = 500000;
	        mte->charge_current = 500000;
			//Set_staystate_current();
			//schedule_delayed_work(&mte->add_current_work,400);
        }
        if (val.value & INT_AFC) {
            pr_info("[%s] Interrupt signal:Tx support samsung_afc\n", __func__);
            pr_info("[%s] Load power recommended to 9W\n", __func__);
			mte->wireless_max_power = 10;
			mte->charge_protocol = AFC;
			mte->input_current = 500000;
	        mte->charge_current = 500000;
			//Set_staystate_current();
        }
		if(val.value & INT_ADDCURRENT){
			 pr_info("[%s] Add current\n", __func__);
			 MT5725_add_current();
		}
		pr_info("[%s] Get power wireless charging chip %dW\n", __func__,mte->wireless_max_power);
    }
    if(temp.ptr[1] & TXMODE){
        pr_info("[%s] The chip works in Tx mode\n",__func__);
        MT5725_read_buffer(mte, REG_INTFLAG, val.ptr, 2);
        fclr.value = val.value;
        pr_info("[%s] REG_INTFLAG value:0x%04x\n", __func__, val.value);
        if(val.value == 0){
            pr_info("[%s] There's no interruption here\n", __func__);
            return;
        }
        if(val.value & INT_DETECT_RX){
            pr_info("[%s] Found RX close\n", __func__);
        }
        if(val.value & INT_OCPFRX){
            pr_info("[%s] TX OCP protection\n", __func__);
        }
        if(val.value & INT_FODDE){
            vuc fod;
            pr_info("[%s] TX FOD\n", __func__);
            MT5725_read_buffer(mte,REG_F_TXPOWER,fod.ptr,2);
            pr_info("[%s] TX_power:%d mW\n", __func__,fod.value);
            MT5725_read_buffer(mte,REG_F_RXPOWER,fod.ptr,2);
            pr_info("[%s] RX_power:%d mW\n", __func__,fod.value);
            MT5725_read_buffer(mte,REG_F_ISTAY,fod.ptr,2);
            pr_info("[%s] Stay current:%d mA\n", __func__,fod.value);
            pr_info("[%s] Clear this flag and the system will ping again\n", __func__,fod.value);
            pr_info("[%s] In case of FOD, there may be metal in the middle, and TX function needs to be turned off\n", __func__,fod.value);
        }
        if(val.value & INT_POWER_TRANS){
            pr_info("[%s] Charge RX normally\n", __func__);
        }
        if(val.value & INT_REMOVE_POEWR){
            pr_info("[%s] Off charge, RX may be removed, or for other reasons\n", __func__);
        }
        if(val.value & INT_CHARGE_STATUS){
            vuc eleq;
            pr_info("[%s] RX reported the current power\n", __func__);
            MT5725_read_buffer(mte,REG_RXCHARGESTATUS,eleq.ptr,2);
            if(eleq.value >= 100){
                pr_info("[%s] Off charge,Disconnect wireless charging,now!!!\n", __func__);
            }
        }
        if(val.value & INT_TXINIT){
            mte->tx_count++;			
            if(mte->tx_count == 1){
                vuc txinit;
                MT5725_read_buffer(mte,REG_STABILITY,txinit.ptr,2);
                if(txinit.value == 0x5555){
                    txinit.value = 0x6666;
                    MT5725_write_buffer(mte,REG_STABILITY,txinit.ptr,2);
                    pr_info("[%s] TX starts working\n", __func__);
                }
            }
            if(mte->tx_count == 2){
                pr_err("[%s] The chip is reset. It may be put on the TX of another home when it is charged reversely\n",__func__);
                pr_err("[%s] Turn off TX function\n",__func__);
            }
        }
    }
    scmd.value |= CLEAR_INT;
    //---clrintflag
    //MT5725_write_buffer(mte, REG_INTCLR, fclr.ptr, 2);
    MT5725_write_buffer(mte, REG_INTCLR, fclr.ptr, 2);
    pr_info("[%s] write REG_INTCLR : 0x%04x,\n", __func__, fclr.value);

    MT5725_write_buffer(mte, REG_CMD, scmd.ptr, 2);
    pr_info("[%s] write REG_CMD : 0x%04x,\n", __func__, scmd.value);

    schedule_delayed_work(&mte->eint_work,100);  //Callback check if the interrupt is cleared
}

EXPORT_SYMBOL(MT5725_irq_handle);





static void MT5725_eint_work(struct work_struct* work) {
     MT5725_irq_handle();
}
static void MT5725_add_current_work(struct work_struct* work) {
	 vuc tcur;
     vuc temp;
	 MT5725_read_buffer(mte, REG_CURFUNC, temp.ptr,1);
	 pr_err("[%s] Call back\n",__func__);
	 if(temp.ptr[0] == 0){
	 	MT5725_read_buffer(mte, REG_IOUT, tcur.ptr,2);
		pr_err("[%s] Rx:Iout:%d\n",__func__,tcur.value);
		pr_err("[%s] wireless max power :%d\n",__func__,mte->wireless_max_power);
		if(mte->wireless_max_power >=10){
			int ifit =  mte->wireless_max_power *1000 / 9 * 98 / 100;
			if(ifit < tcur.value) {
				En_Dis_add_current(0xFF);
				pr_err("[%s] En_Dis_add_current : DISABLE\n",__func__);
			}
		}
	 }else{
	 	pr_err("[%s]  It has been disabled\n",__func__);
	 }
	 return;
}


static irqreturn_t MT5725_irq(int irq,void * data){
    struct  MT5725_dev * mt5725 = data;
    schedule_delayed_work(&mt5725->eint_work,100);
    return IRQ_HANDLED;
}

static const struct of_device_id match_table[] = {
    {.compatible = "maxictech,mt5725-15w",},
    { },
};


int get_MT5725_status(void){
   vuc chipid;
   chipid.value =0;

   if(gpio_get_value(mte->statu_gpio)){
      if(MT5725_read_buffer(mte, REG_CHIPID, chipid.ptr,2) == 0){
	     if(chipid.value == MT5725ID){
	        pr_err("%s: chipID : %02x%02x  chipid.value =0x%04x\n",__func__,chipid.ptr[0],chipid.ptr[1],chipid.value);
			return 0;
	     } else {
		    pr_err("ID error :%d\n ", chipid.value);
			return -3;
	     }
       }
	   pr_err("MT5725_read_buffer read fail \n ");
	   return -2;
	}else{
	   pr_err("%s: statu_gpio is low\n",__func__);
	   return -1;
	}
   
}
EXPORT_SYMBOL(get_MT5725_status);

int reset_mt5725_info(void){
	mte->wireless_max_power =0;
	mte->charge_protocol = PROTOCOL_UNKNOWN;
	mte->input_current = 0;
	mte->charge_current = 0;
	pr_err("%s: \n",__func__);
	return 0;
}
EXPORT_SYMBOL(reset_mt5725_info);



int get_wireless_charge_current(struct charger_data *pdata){
	pdata->input_current_limit = mte->input_current;
	pdata->charging_current_limit = mte->charge_current;
	pr_info("[%s] input_current =  %d,charge_current = %d\n", __func__,mte->input_current,mte->charge_current);
	print_curfunc_info();
	//vuc protocol;
	//vuc epp;
	//epp.value =0;
    //protocol.value =0;
/*	
	if(gpio_get_value(mte->statu_gpio)){
		 MT5725_read_buffer(mte, 0x0098, protocol.ptr,1);
		 pr_err("%s: protocol read 1 : %02x%02x  protocol.value =0x%04x  protocol.value =%d\n",__func__,protocol.ptr[0],protocol.ptr[1],protocol.value,protocol.value);
		 if(protocol.ptr[0]== 1){
		 	mte->charge_protocol = BPP;
			mte->wireless_max_power = 5;
			pr_info("[%s] BPP Load power is recommended to be less than %dW\n", __func__,mte->wireless_max_power);
		 } else if(protocol.ptr[0] == 2){
		    mte->charge_protocol = AFC;
			mte->wireless_max_power = 9;
			pr_info("[%s] AFC Load power is recommended to be less than %dW\n", __func__,mte->wireless_max_power);
		 } else if(protocol.ptr[0] == 3){
		    mte->charge_protocol = EPP;
			MT5725_read_buffer(mte,REG_MAXPOWER,epp.ptr,1);
		    epp.ptr[0] = epp.ptr[0]/2;
		    pr_info("[%s]  EPP Load power is recommended to be less than %dW\n", __func__,epp.ptr[0]);
		    mte->wireless_max_power = epp.ptr[0];
		 } else {
			chr_err("%s read 0x0098 info no BPP EPP AFC protocol \n", __func__);
		 }
     }
	
	if(mte->charge_protocol == BPP){
	   pdata->input_current_limit = 900000;
	   pdata->charging_current_limit = 1000000;
	} else if ((mte->charge_protocol == EPP) ||( mte->charge_protocol== AFC)){
         if(get_mt5725_voltage() < 8000){
		 	pdata->input_current_limit = 1000000;
			pdata->charging_current_limit = 1000000;
         } else {
            if(mte->input_current == 0)
			   mte->input_current = (mte->wireless_max_power*1000000.0)/(9/1.0);

			if(mte->charge_current == 0)
	           mte->charge_current = (mte->wireless_max_power*1000000.0)/(4/1.0);
			
            pdata->input_current_limit = mte->input_current;
            pdata->charging_current_limit = mte->charge_current;
         }
	} else {
	   chr_err("%s no BPP EPP AFC protocol \n", __func__);
	}
	*/
    return 0;
}
EXPORT_SYMBOL(get_wireless_charge_current);

static int MT5725_parse_dt(struct i2c_client *client, struct MT5725_dev *mt5725)
{
    int ret =0;
	mt5725->statu_gpio = of_get_named_gpio(client->dev.of_node, "statu_gpio", 0);
	if (mt5725->statu_gpio < 0) {
		pr_err("%s: no dc gpio provided\n", __func__);
		return -1;
	} else {
		pr_info("%s: dc gpio provided ok. mt5715->statu_gpio = %d\n", __func__, mt5725->statu_gpio);
	}

	mt5725->irq_gpio = of_get_named_gpio(client->dev.of_node, "irq-gpio", 0);
	if (mt5725->irq_gpio < 0) {
		pr_err("%s: no irq gpio provided.\n", __func__);
		return -1;
	} else {
		pr_info("%s: irq gpio provided ok. mt5715->irq_gpio = %d\n", __func__, mt5725->irq_gpio);
	}

	mt5725->otg_5725_ctl.pinctrl_gpios = devm_pinctrl_get(&client->dev);
   if (IS_ERR(mt5725->otg_5725_ctl.pinctrl_gpios)) {
		ret = PTR_ERR(mt5725->otg_5725_ctl.pinctrl_gpios);
		pr_err("%s can't find chg_data pinctrl\n", __func__);
		return ret;
   }
   
	mt5725->otg_5725_ctl.pins_default = pinctrl_lookup_state(mt5725->otg_5725_ctl.pinctrl_gpios, "default");
	if (IS_ERR(mt5725->otg_5725_ctl.pins_default)) {
		ret = PTR_ERR(mt5725->otg_5725_ctl.pins_default);
		pr_err("%s can't find chg_data pinctrl default\n", __func__);
		/* return ret; */
	}

	mt5725->otg_5725_ctl.charger_otg_off = pinctrl_lookup_state(mt5725->otg_5725_ctl.pinctrl_gpios, "charger_otg_off");
	if (IS_ERR(mt5725->otg_5725_ctl.charger_otg_off)) {
		ret = PTR_ERR(mt5725->otg_5725_ctl.charger_otg_off);
		pr_err("%s  can't find chg_data pinctrl otg high\n", __func__);
		return ret;
	}

	mt5725->otg_5725_ctl.charger_otg_on = pinctrl_lookup_state(mt5725->otg_5725_ctl.pinctrl_gpios, "charger_otg_on");
	if (IS_ERR(mt5725->otg_5725_ctl.charger_otg_on)) {
		ret = PTR_ERR(mt5725->otg_5725_ctl.charger_otg_on);
		pr_err("%s  can't find chg_data pinctrl otg low\n", __func__);
		return ret;
	}

	mt5725->otg_5725_ctl.wireless_5725_off = pinctrl_lookup_state(mt5725->otg_5725_ctl.pinctrl_gpios, "wireless_5725_off");
	if (IS_ERR(mt5725->otg_5725_ctl.wireless_5725_off)) {
		ret = PTR_ERR(mt5725->otg_5725_ctl.wireless_5725_off);
		pr_err("%s  can't find chg_data pinctrl wireless_5725_off\n", __func__);
		return ret;
	}

	mt5725->otg_5725_ctl.wireless_5725_on = pinctrl_lookup_state(mt5725->otg_5725_ctl.pinctrl_gpios, "wireless_5725_on");
	if (IS_ERR(mt5725->otg_5725_ctl.wireless_5725_on)) {
		ret = PTR_ERR(mt5725->otg_5725_ctl.wireless_5725_on);
		pr_err("%s  can't find chg_data pinctrl wireless_5725_on\n", __func__);
		return ret;
	}

	mt5725->otg_5725_ctl.gpio_otg_prepare = true;
	return 0;
}

int turn_off_5725(int en){
	int ret =0;
	
    if (mte->otg_5725_ctl.gpio_otg_prepare) {
		if (en) {
			pinctrl_select_state(mte->otg_5725_ctl.pinctrl_gpios, mte->otg_5725_ctl.wireless_5725_off);
			printk("%s: set W_OTG_EN2 to hight\n", __func__);
			ret =0;
		}
		else {
			pinctrl_select_state(mte->otg_5725_ctl.pinctrl_gpios, mte->otg_5725_ctl.wireless_5725_on);
			printk("%s: set W_OTG_EN2 to low\n", __func__);
			ret =0;
		}
	}
	else {
		printk("%s:, error, gpio otg not prepared\n", __func__);
		ret =-1;
	}
	return ret;
}
EXPORT_SYMBOL(turn_off_5725);

int set_otg_gpio(int en){

	int ret =0;
	
    if (mte->otg_5725_ctl.gpio_otg_prepare) {
		if (en) {
			pinctrl_select_state(mte->otg_5725_ctl.pinctrl_gpios, mte->otg_5725_ctl.charger_otg_on);
			printk("%s: set w_otg_en PIN to high\n", __func__);
			ret =0;
		}
		else {
			pinctrl_select_state(mte->otg_5725_ctl.pinctrl_gpios, mte->otg_5725_ctl.charger_otg_off);
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
EXPORT_SYMBOL(set_otg_gpio);


static int MT5725_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    struct MT5725_dev *chip;
    int irq_flags = 0;
    int rc = 0;
	vuc protocol;

    pr_err("MT5725 probe.\n");
    chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
    if (!chip)
        return -ENOMEM;

    pr_err("MT5725 chip.\n");
    chip->regmap = regmap_init_i2c(client, &MT5725_regmap_config);
    if (!chip->regmap) {
        pr_err("parent regmap is missing\n");
        return -EINVAL;
    }
    pr_err("MT5725 regmap.\n");
	
    chip->client = client;
    chip->dev = &client->dev;

    chip->bus.read = MT5725_read;
    chip->bus.write = MT5725_write;
    chip->bus.read_buf = MT5725_read_buffer;
    chip->bus.write_buf = MT5725_write_buffer;

    device_init_wakeup(chip->dev, true);
    
    sysfs_create_group(&client->dev.kobj, &mt5725_sysfs_group);
 
    pr_err("MT5725 probed successfully\n");

    mte = chip;

	mte->wireless_max_power = 0;
	mte->otg_5725_ctl.gpio_otg_prepare = false;
	mte->charge_protocol = PROTOCOL_UNKNOWN;
	mte->input_current = 0;
	mte->charge_current = 0;

    INIT_DELAYED_WORK(&chip->eint_work, MT5725_eint_work);
	INIT_DELAYED_WORK(&chip->add_current_work, MT5725_add_current_work);
    rc = MT5725_parse_dt(client, chip);
    if (rc ) {
    	pr_err("%s: failed to parse device tree node\n", __func__);
        chip->statu_gpio = -1;
        chip->irq_gpio = -1;
    }
	
    protocol.value =0;
    if(gpio_get_value(mte->statu_gpio)){
		MT5725_read_buffer(mte, REG_CURFUNC, protocol.ptr,1);
		pr_err("%s: protocol read 1 : %02x%02x  protocol.value =0x%04x  protocol.value =%d\n",__func__,protocol.ptr[0],protocol.ptr[1],protocol.value,protocol.value);
    }

    if (gpio_is_valid(chip->irq_gpio)) {
        rc = devm_gpio_request_one(&client->dev, chip->irq_gpio,
			GPIOF_DIR_IN, "mt5725_int");
        if (rc) {
			pr_err("%s: irq_gpio request failed\n", __func__);
			goto err;
        }

        irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
        rc = devm_request_threaded_irq(&client->dev, gpio_to_irq(chip->irq_gpio),
						NULL, MT5725_irq, irq_flags, "mt5725", chip);
        if (rc != 0) {
			pr_err("failed to request IRQ %d: %d\n", gpio_to_irq(chip->irq_gpio), rc);
			goto err;
        }
		pr_err("sucess to request IRQ %d: %d\n", gpio_to_irq(chip->irq_gpio), rc);

		//start add by sunshuai
		if(!(gpio_get_value(mte->irq_gpio))){
			pr_err("%s The interruption has come \n", __func__);
			MT5725_irq_handle();
		}
		//end   add by sunshuai
    } else {
		pr_info("%s skipping IRQ registration\n", __func__);
    }
	i2c_set_clientdata(client,chip);
	pr_info("%s probe done\n", __func__);
err:
    return rc;
}

static int MT5725_remove(struct i2c_client *client) {
    sysfs_remove_group(&client->dev.kobj, &mt5725_sysfs_group);
    if (gpio_is_valid(mte->irq_gpio))
        devm_gpio_free(&client->dev, mte->irq_gpio);

	if (gpio_is_valid(mte->statu_gpio))
        devm_gpio_free(&client->dev, mte->statu_gpio);
    return 0;
}


static const struct i2c_device_id MT5725_dev_id[] = {
    {"MT5715_receiver", 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, MT5725_dev_id);

static struct i2c_driver MT5725_driver = {
    .driver   = {
        .name           = DEVICE_NAME,
        .owner          = THIS_MODULE,
        .of_match_table = match_table,
    },
    .probe    = MT5725_probe,
    .remove   = MT5725_remove,
    .id_table = MT5725_dev_id,
};
module_i2c_driver(MT5725_driver);

MODULE_AUTHOR("sunshuai@szprize.com");
MODULE_DESCRIPTION("MT5725 Wireless Power Receiver");
MODULE_LICENSE("GPL v2");

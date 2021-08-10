#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iMultiReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId, u8 number);

#define BYTE               unsigned char

#define PFX "S5KGM1SP_STEREO_OTP"
#undef LOG_INF
#define LOG_INF(format, args...)    pr_err(PFX "[%s] " format, __FUNCTION__, ##args)

#define EEPROM_WRITE_ID   0xB0
#define I2C_SPEED        300
// dual camera by westalgo
#define CALI_DATA_OFFESET     0xD7A
#define CALI_DATA_FLAG_OFFSET 0x157B 
#define CALI_DATA_CHECKSUM_OFFSET 0x157A 
#define CALI_DATA_FLAG_VALUE  0x01



#define PATH_CALI   "/mnt/vendor/ArcSoftCali/mcal.bin"
//#define PATH_CALI_FLAG   "/mnt/vendor/ArcSoftCali/dualcam_flag.bin"
#define PATH_CALI_READ_RESULT_FLAG   "/mnt/vendor/ArcSoftCali/read_result_flag.bin"


#define PATH_CALI_SD   "/sdcard/.ArcSoftCali/mcal.bin"
//#define PATH_CALI_FLAG_SD   "/sdcard/.ArcSoftCali/dualcam_flag.bin"
#define PATH_CALI_WRITE_RESULT_FLAG_SD   "/sdcard/.ArcSoftCali/write_result_flag.bin"

extern int CALI_DATA_NUM;

static BYTE *bin_buffer = NULL;
static BYTE *cali_buffer = NULL;
//static BYTE cali_flag = 0;

static void write_cmos_sensor_byte(kal_uint32 addr, kal_uint32 para);
static bool selective_read_eeprom(kal_uint16 addr, BYTE *data);
// zhuzhengjiang add 
static void disable_write_protection(void)
{
   //char i2c_send_cmd_1[3] = {(char)0x1F/*addr >> 8*/, (char)0x40/*addr & 0xff*/, (char)0x00/*value*/};
   // printk(PFX "disable software write protection\n");
    //iWriteRegI2C(i2c_send_cmd_1, 3, 0xA2);
    write_cmos_sensor_byte(0x8000,0x00);
    msleep(10);

}

// Start+0xFO
static void enable_write_protection(void)
{
    //char i2c_send_cmd_1[3] = {(char)0x1F/*addr >> 8*/, (char)0x40/*addr & 0xff*/, (char)0x0e/*value*/};
    //printk(PFX "enable software write protection\n");
   // iWriteRegI2C(i2c_send_cmd_1, 3, 0xA2);
   write_cmos_sensor_byte(0x8000,0x0E);
    msleep(10);
}
//zhuzhengjiang end
static uint32_t calcCheckSum(BYTE *data, uint32_t num)
{
    uint32_t i, checkSum = 0;
    for (i = 0; i < num; i++) {
        checkSum += *data;
        data++;
    }

    checkSum = checkSum % 256;

    return checkSum;
}

static bool write_eeprom(kal_uint16 addr, kal_uint16 size, BYTE *data)
{
    int i = 0;
    int count = 0;
    int offset = addr;
    BYTE data_R;
    u8 retry = 0, delay = 0;
    disable_write_protection();
    for(i = 0; i < size; i++) {
        delay = 5;
        retry = 3;
        write_cmos_sensor_byte((u16)offset, (u32) * (data + i));
        mdelay(delay);
        
        do {
            if(selective_read_eeprom(offset, &data_R) != true) {
                printk("read data error, offset: %x\n", offset);
                return false;
            } else {
//                printk("data-R %x data: %x\n",(u32)data_R, (u32)*(data + i));
                if (data_R == *(data + i)) {
//                    printk("write addr 0x%0x data 0x%0x",offset,(u32)*(data + i));
                    break;
                } else {
                    printk("try to write offset(%x) retry: %d\n", offset, retry);
                    write_cmos_sensor_byte((u16)offset, (u32) * (data + i));
                    delay += 5;
                    mdelay(delay);
                }
            }
            retry--;
        } while(retry != 0);
        count++;
        offset++;
    }
    enable_write_protection();
    if (count == size)
        return true;

    printk("Eeprom write failed");

    return false;
}

static bool read_eeprom_cali( kal_uint16 addr, BYTE *data, kal_uint32 size)
{

    int i = 0;
    int offset = addr;

    printk("Read eeprom calibration data, size = %d\n", size);

    for(i = 0; i < size; i++) {
        if(!selective_read_eeprom(offset, &data[i]))
            return false;
//        printk("read_eeprom 0x%0x 0x%0x\n",offset, data[i]);
        offset++;
    }
    return true;
}

static uint32_t read_dualcam_cali_data(void)
{
    struct file *fp;
    uint32_t data_size = 0;
    loff_t pos = 0;
    mm_segment_t fs = get_fs();

    fp = filp_open(PATH_CALI_SD, O_RDWR, 0666);
    if (IS_ERR(fp)) {
        printk("faile to open file cali data error\n");
        return -1;
    }

    data_size = vfs_llseek(fp, 0, SEEK_END);
    printk("Binary data size is %d bytes \n", data_size);

    if(data_size > 0) {
        if(bin_buffer == NULL) {
            bin_buffer = kzalloc(data_size, GFP_KERNEL);
            if (bin_buffer == NULL) {
                printk("[Error]malloc memery failed \n");
                goto close;
            }
        }
        set_fs(KERNEL_DS);
        pos = 0;
        vfs_read(fp, bin_buffer, data_size, &pos);
        printk("Read new calibration data done!\n");

        filp_close(fp, NULL);
        set_fs(fs);
        return data_size;
    } else
        printk("[Error] Get calibration data failed\n");

close:
    filp_close(fp, NULL);
    set_fs(fs);
    printk("read dualcam cali data exit\n");
    return 0;
}

static int write_eeprom_memory(BYTE *data, uint32_t size, BYTE checkSum)
{
    bool rc = false;

    rc = write_eeprom(CALI_DATA_OFFESET, size, data);
    if(!rc) {
        printk("%s: write eeprom failed\n", __func__);
        return -1;
    }

    rc = write_eeprom(CALI_DATA_CHECKSUM_OFFSET, 1, &checkSum);
    if(!rc) {
        printk("%s: write dual cali checkSum failed\n", __func__);
        return -1;
    }

    return 0;
}


#if 0
static int get_verify_flag(void)
{

    BYTE flag = 0;

    if(!selective_read_eeprom(CALI_DATA_FLAG_OFFSET, &flag))
        return -1;

    printk("Get verify flag data 0x%x\n", flag);

    return flag;
}


static int set_verify_flag(void)
{
    int rc = -1;
    BYTE flag = 0;

    if(!selective_read_eeprom(CALI_DATA_FLAG_OFFSET, &flag))
        return rc;

    printk("Get verify flag data 0x%x\n", flag);

    if ( flag != CALI_DATA_FLAG_VALUE){
	        write_cmos_sensor_byte(CALI_DATA_FLAG_OFFSET, CALI_DATA_FLAG_VALUE);
			mdelay(5);
		} 
	else 
		{
        	printk("Verify flag is already set\n");
		}

    return 0;
}


int write_dualcam_cali_flag_file_s5kgm1sp(void)
{
    struct file *eeprom_file = NULL;
    int err = 0;
    mm_segment_t fs = get_fs();
    loff_t pos;

    printk("Need to read cali flag to flag file, %s", __func__);
    cali_flag = get_verify_flag();
    printk("cali_flag:%d to flag dir, %s", cali_flag, __func__);
    eeprom_file = filp_open(PATH_CALI_FLAG_SD, O_RDWR | O_CREAT, 0644);
    if (IS_ERR(eeprom_file)) {
        printk("open epprom file error");
        return -1;
    }
    set_fs(KERNEL_DS);
    pos = 0;
    printk("eeprom_file: %p, cali_flag: %d, %s\n", eeprom_file, cali_flag, __func__);
    err = vfs_write(eeprom_file, &cali_flag, sizeof(cali_flag), &pos);
    if (err < 0)
        printk("write epprom data error");
    filp_close(eeprom_file, NULL);
    set_fs(fs);

    return 0;
}
#endif

int write_dualcam_cali_result_flag_file_s5kgm1sp(int result)
{
    struct file *result_file = NULL;
    int err = 0;
    mm_segment_t fs = get_fs();
    loff_t pos;

    printk("cali_result_flag:%d to flag file, %s", result, __func__);
    result_file = filp_open(PATH_CALI_WRITE_RESULT_FLAG_SD, O_RDWR | O_CREAT, 0644);
    if (IS_ERR(result_file)) {
        printk("open epprom file error");
        return -1;
    }
    set_fs(KERNEL_DS);
    pos = 0;
    printk("write_calibraion_result_file: %p, result_flag: %d, %s\n", result_file, result, __func__);
    err = vfs_write(result_file, (char *)&result, sizeof(result), &pos);
    if (err < 0)
        printk("write calibration result file error");
    filp_close(result_file, NULL);
    set_fs(fs);

    return 0;
}


#if 0
int dump_dualcam_cali_flag_file_s5kgm1sp(void)
{
    struct file *eeprom_file = NULL;
    int err = 0;
    mm_segment_t fs = get_fs();
    loff_t pos;

    printk("Need to read cali flag to flag dir, %s", __func__);
    cali_flag = get_verify_flag();
    printk("cali_flag:%d to flag dir, %s", cali_flag, __func__);
    eeprom_file = filp_open(PATH_CALI_FLAG, O_RDWR | O_CREAT, 0644);

    if (IS_ERR(eeprom_file)) {
        printk("open epprom file error");
        return -1;
    }

    set_fs(KERNEL_DS);
    pos = 0;
    printk("eeprom_file: %p, cali_flag: %d, %s\n", eeprom_file, cali_flag, __func__);
    err = vfs_write(eeprom_file, &cali_flag, sizeof(cali_flag), &pos);
    if (err < 0)
        printk("write epprom data error");
    filp_close(eeprom_file, NULL);
    set_fs(fs);

    return 0;
}
#endif


int dump_dualcam_cali_result_flag_file_s5kgm1sp(int result)
{
    struct file *result_file = NULL;
    int err = 0;
    mm_segment_t fs = get_fs();
    loff_t pos;

    printk("dump_result_flag:%d to flag dir, %s", result, __func__);
    result_file = filp_open(PATH_CALI_READ_RESULT_FLAG, O_RDWR | O_CREAT, 0644);

    if (IS_ERR(result_file)) {
        printk("open epprom file error");
        return -1;
    }

    set_fs(KERNEL_DS);
    pos = 0;
    printk("write dump_result_file: %p, result_flag: %d, %s\n", result_file, result, __func__);
    err = vfs_write(result_file, (char *)&result, sizeof(result), &pos);
    if (err < 0)
        printk("write dump result file error");
    filp_close(result_file, NULL);
    set_fs(fs);

    return 0;
}


int store_dualcam_cali_data_s5kgm1sp(void)
{
    int size = -1, rc = -1;
    BYTE checkSum = 0;
	int ret = 1;
	#if 1
	int i = 0;
	#endif

    printk("enter store_dualcam_cali_data, %s\n", __func__);

    size = read_dualcam_cali_data();

    if (size < 0) {
        printk("Fail to get new calibration data, /sdcard/.ArcSoftCali/mcal.bin file need %s\n", __func__);
        //return -1;
        ret = 0;
    }

	#if 1
	for(i=0;i<CALI_DATA_NUM;i++)
	{
				
		//printf("read dual camera cali_buffer data[%d]=0x%x\r\n",i,dual_camera[i]);
		//printf("read dual camera cali_buffer data[%d]=0x%x\r\n",i,dual_camera[i]);
		//bin_buffer[i] = i % 256;
		printk("write dual camera cali_buffer data[%d]=0x%x\r\n",i,bin_buffer[i]);
	}
	#endif
    checkSum = calcCheckSum(bin_buffer, CALI_DATA_NUM);
    rc = write_eeprom_memory(bin_buffer, size, checkSum);
    if (rc < 0) {
        printk("%s: failed to write_eeprom_memory ,need camera opend and /sdcard/.ArcSoftCali/mcal.bin file \n", __func__);
        //return -1;
        ret = 0;
    }

#if 0
    rc = set_verify_flag();
    if (rc < 0) {
        printk("%s: failed to set_verify_flag\n", __func__);
        return -1;
    }
    rc = write_dualcam_cali_flag_file_s5kgm1sp();
    if (rc < 0) {
        printk("%s: failed to write_dualcam_cali_flag_file\n", __func__);
        return -1;
    }
#endif

 	printk(" store_dualcam_cali_data %s, %s\n", (ret == 1 ? "sucess":"failed"), __func__);

    rc = write_dualcam_cali_result_flag_file_s5kgm1sp(ret);
    if (rc < 0) {
        printk("%s: failed to write_dualcam_cali_result_flag_file\n", __func__);
        return -1;
    }

    return ret;
}

int dump_dualcam_cali_data_s5kgm1sp(void)
{
    struct file *eeprom_file = NULL;
    int err = 0, ret = -1, ret2 = 1;
    mm_segment_t fs;
    loff_t pos;
    BYTE checkSum = 0, checkSum_R = 0;
	#if 1
	int i = 0;
	#endif
    printk("Need to read cali data to data dir, %s\n", __func__);

    if(cali_buffer == NULL) {
        cali_buffer = kzalloc(CALI_DATA_NUM, GFP_KERNEL);
        if (cali_buffer == NULL) {
            printk("[Error]malloc memery failed \n");
            //return -1;
            ret2 = 0;
        }
    }

    printk("Need to read cali data to data dir, %s\n", __func__);
    ret = read_eeprom_cali(CALI_DATA_OFFESET, cali_buffer, CALI_DATA_NUM);
    if (!ret) {
        printk("fail to read calibration data from eeprom\n");
        //return -1;
        ret2 = 0;
    }

    fs = get_fs();
    if(!selective_read_eeprom((u16)CALI_DATA_CHECKSUM_OFFSET, (BYTE *)&checkSum)) {
        printk("%s: read check sum failed!\n", __func__);
        //return -1;
        ret2 = 0;
    }
    checkSum_R = calcCheckSum(cali_buffer, CALI_DATA_NUM);
    printk("[%s] checkSum: %x, checkSum_R: %x\n", __func__, checkSum, checkSum_R);
    if(checkSum != checkSum_R) {
        printk("%s: fail to check sum, contact the  vendor to check\n", __func__);
        //return -1;
        ret2 = 0;
    }
	#if 1	
	for(i=0;i<CALI_DATA_NUM;i++)
	{
				
		//printf("read dual camera cali_buffer data[%d]=0x%x\r\n",i,dual_camera[i]);
		//printf("read dual camera cali_buffer data[%d]=0x%x\r\n",i,dual_camera[i]);
		//dual_camera[i] = i % 256;
		printk("read dual camera cali_buffer data[%d]=0x%x\r\n",i,cali_buffer[i]);
	}
	#endif
    eeprom_file = filp_open(PATH_CALI, O_RDWR | O_CREAT, 0644);
    if (IS_ERR(eeprom_file)) {
        printk("open epprom file error, /sdcard/.ArcSoftCali/ dir need %s", __func__);
        //return -1;
        ret2 = 0;
    }


    set_fs(KERNEL_DS);
    pos = 0;
    printk("eeprom_file: %p, cali_buffer: %p\n", eeprom_file, cali_buffer);
    err = vfs_write(eeprom_file, cali_buffer, CALI_DATA_NUM, &pos);
    if (err < 0){
        printk("write epprom data error, , %s", __func__);

		ret2 = 0;
    }
    filp_close(eeprom_file, NULL);
    set_fs(fs);

#if 0
    err = dump_dualcam_cali_flag_file_s5kgm1sp();

    if (err < 0) {
        printk("%s: failed to write_dualcam_cali_flag_file\n", __func__);
        return -1;
    }
#endif

 	printk(" dump_dualcam_cali_data %s, %s\n", (ret2 == 1 ? "sucess":"failed"), __func__);

    err = dump_dualcam_cali_result_flag_file_s5kgm1sp(ret2);
    if (err < 0) {
        printk("%s: failed to write_dualcam_cali_flag_file\n", __func__);
        //return -1;
        ret2 = -1;
    }

    kfree(cali_buffer);
    cali_buffer = NULL;

    return ret2;
}

static bool selective_read_eeprom(kal_uint16 addr, BYTE *data)
{
    char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };

    if(iReadRegI2C(pu_send_cmd, 2, (u8 *)data, 1, EEPROM_WRITE_ID) < 0)
        return false;

    return true;
}

static void write_cmos_sensor_byte(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

    iWriteRegI2C(pu_send_cmd, 3, EEPROM_WRITE_ID);
}

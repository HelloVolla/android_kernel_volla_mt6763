/*
 * Driver for CAM_CAL
 *
 *
 */

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>

#include <asm/atomic.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/types.h>
#include <sync_write.h>
#include <sync_write.h>
#include <linux/proc_fs.h>
#include <linux/module.h>
#include "kd_camera_feature.h"
#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "kd_camera_typedef.h"



#include "s5k3P3_otp_mcnex.h"
//#include <asm/system.h>  // for SMP
#include <linux/dma-mapping.h>



//#include "read.h"
#define CAM_CALGETDLT_DEBUG
#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
#define CAM_CALDB printk
#else
#define CAM_CALDB(x,...)
#endif

static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP
#define CAM_CAL_I2C_BUSNUM 0 //fucehou
extern u8 OTPData[];//FCH
//u8 OTPData[];
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif //fch

#ifdef CONFIG_OF
/* device tree */
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#endif

/* prize-lifenfen-1205 */
#ifdef CAM_CAL_DEBUG
#include <linux/proc_fs.h>
#endif
/* prize-lifenfen-1205 */

/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_ICS_REVISION 1 //seanlin111208
/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "S5K3P3_CAM_CAL_DRV"
#define CAM_CAL_I2C_GROUP_ID 0
/*******************************************************************************
*
********************************************************************************/
//static struct i2c_board_info __initdata kd_cam_cal_dev={ I2C_BOARD_INFO(CAM_CAL_DRVNAME, 0xa0>>1)};

static struct i2c_client * g_pstI2Cclient = NULL;

//81 is used for V4L driver
static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;
//static spinlock_t g_CAM_CALLock;
//spin_lock(&g_CAM_CALLock);
//spin_unlock(&g_CAM_CALLock);

static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;
//static DEFINE_SPINLOCK(kdcam_cal_drv_lock);
//spin_lock(&kdcam_cal_drv_lock);
//spin_unlock(&kdcam_cal_drv_lock);

#define LSCOTPDATASIZE 512
#define S5K3L8_EEPROM_READ_ID  0xA0
#define S5K3L8_EEPROM_WRITE_ID   0xA1
#define S5K3L8_I2C_SPEED        100  
#define S5K3L8_MAX_OFFSET		0xFFFF
static bool get_done = false;
static int last_size = 0;
static int last_offset = 0;

//int otp_flag=1;

//static kal_uint8 lscotpdata[LSCOTPDATASIZE];
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);

#if 1
ssize_t zc533_read_register(kal_uint8 page,kal_uint8 page_add, kal_uint8 *returnData)
{

	int 	ret=0;
	char	cmd_buf[2]={0x00,0x00};
	char	readData = 0;
	if(!g_pstI2Cclient)
	{	
		printk("I2C client not initialized!!");
		return -1;
	}	
	
    cmd_buf[0] = (page>>2&0x3f);
	cmd_buf[1] = ((page_add&0x3f)+((page&0x03)<<6));
	
	ret = i2c_master_send(g_pstI2Cclient, &cmd_buf[0], 2);
	//printk("fch page=%x page_add=%x cmd_buf=[%x %x]\n" ,page,page_add,cmd_buf[0],cmd_buf[1]);
	if (ret < 0) {
		printk("read sends command error!! ret=%d addr=0x%x%x \n",ret,cmd_buf[0],cmd_buf[1]);
		return -1;
	}
	ret = i2c_master_recv(g_pstI2Cclient, &readData, 1);
	
	if (ret < 0) {
		printk("reads recv data error!!");
		return -1;
	} else{

		//printk("=cs43l36========reg:0x%x  readData:0x%x\n",addr,readData);
	}
	*returnData = readData;
	
	return ret;
}
#endif



/*******************************************************************************
*
********************************************************************************/
//Address: 2Byte, Data: 1Byte
/*
static int iWriteCAM_CAL(u16 a_u2Addr  , u32 a_u4Bytes, u8 puDataInBytes)
{   
    #if 0
    int  i4RetValue = 0;
	int retry = 3;
    char puSendCmd[3] = {(char)(a_u2Addr >> 8) , (char)(a_u2Addr & 0xFF) ,puDataInBytes};
	g_pstI2Cclient->addr = (S5K3L8OTP_DEVICE_ID>> 1);

	do {
        CAM_CALDB("[CAM_CAL][iWriteCAM_CAL] Write 0x%x=0x%x \n",a_u2Addr, puDataInBytes);
		i4RetValue = i2c_master_send(g_pstI2Cclient, puSendCmd, 3);
        if (i4RetValue != 3) {
            CAM_CALDB("[CAM_CAL] I2C send failed!!\n");
        }
        else {
            break;
    	}
        mdelay(10);
    } while ((retry--) > 0);    
   //CAM_CALDB("[CAM_CAL] iWriteCAM_CAL done!! \n");
   #else
   CAM_CALDB("[CAM_CAL][iWriteCAM_CAL] Write 0x%x=0x%x \n",a_u2Addr, puDataInBytes);
   iWriteReg(a_u2Addr,puDataInBytes,1,S5K3L8OTP_DEVICE_ID);
   #endif
   return 0;
}
*/

//Address: 2Byte, Data: 1Byte
static int iReadCAM_CAL(u16 a_u2Addr, u32 ui4_length, u8 * a_puBuff)
{
    #if 1
    int  i4RetValue = 0;
    char puReadCmd[2] = {(char)(a_u2Addr >> 8) , (char)(a_u2Addr & 0xFF)};
	g_pstI2Cclient->addr = (S5K3L8OTP_DEVICE_ID>> 1);

    //CAM_CALDB("[CAM_CAL] iReadCAM_CAL!! \n");   
    //CAM_CALDB("[CAM_CAL] i2c_master_send \n");
    i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 2);
	
    if (i4RetValue != 2)
    {
        CAM_CALDB("[CAM_CAL] I2C send read address failed!! \n");
        return -1;
    }

    //CAM_CALDB("[CAM_CAL] i2c_master_recv \n");
    i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, ui4_length);
	//CAM_CALDB("[CAM_CAL][iReadCAM_CAL] Read 0x%x=0x%x \n", a_u2Addr, a_puBuff[0]);
    if (i4RetValue != ui4_length)
    {
        CAM_CALDB("[CAM_CAL] I2C read data failed!! \n");
        return -1;
    } 

    //CAM_CALDB("[CAM_CAL] iReadCAM_CAL done!! \n");
	#else
	int  i4RetValue = 0;

	i4RetValue=iReadReg(a_u2Addr,a_puBuff,S5K3L8OTP_DEVICE_ID);
	
	if (i4RetValue !=0)
		{
			CAM_CALDB("[CAM_CAL] I2C read data failed!! \n");
			return -1;
		} 	
	CAM_CALDB("[CAM_CAL][iReadCAM_CAL] Read 0x%x=0x%x \n", a_u2Addr, a_puBuff[0]);
	#endif
    return 0;
}

int iReadCAM_CAL_8(u8 a_u2Addr, u8 * a_puBuff, u16 i2c_id)
{
    int  i4RetValue = 0;
    char puReadCmd[1] = {(char)(a_u2Addr)};

    spin_lock(&g_CAM_CALLock); //for SMP
    g_pstI2Cclient->addr = i2c_id>>1;
	g_pstI2Cclient->timing=300;
	g_pstI2Cclient->addr = g_pstI2Cclient->addr & (I2C_MASK_FLAG | I2C_WR_FLAG);
    spin_unlock(&g_CAM_CALLock); // for SMP    
    
    i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 1);
	
    if (i4RetValue != 1)
    {
        CAM_CALDB("[CAM_CAL] I2C send read address failed!! \n");
	    CAM_CALDB("[CAMERA SENSOR] I2C send failed, addr = 0x%x, data = 0x%x !! \n", a_u2Addr,  *a_puBuff );
        return -1;
    }

    i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, 1);
	//CAM_CALDB("[CAM_CAL][iReadCAM_CAL] Read 0x%x=0x%x \n", a_u2Addr, a_puBuff[0]);
    if (i4RetValue != 1)
    {
        CAM_CALDB("[CAM_CAL] I2C read data failed!! \n");
        return -1;
    } 

    return 0;
}



/* prize-lifenfen-1205 */
#ifdef CAM_CAL_DEBUG
static struct proc_dir_entry *s5k3l8_otp_proc = NULL;
char temptest_data[0xCF3] = {0};

static ssize_t s5k3l8_otp_read_proc(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
    int len, err = -1;
	u16 i= 0;
	//u8 readbuff = 0;
	char *page = NULL;
	 char *ptr = NULL;
       int checksum = 0;

   page = kmalloc(16*PAGE_SIZE, GFP_KERNEL);	 
   if (!page) 
   {	 
	 kfree(page);		 
	 return -ENOMEM; 
   }
 
	 ptr = page; 
	 ptr += sprintf(ptr, "==== otp aata value====\n");
 
    for (i = 0 ; i < 0x0CF3 ; i++)
    {
        ptr += sprintf(ptr, "0x  %x,0x  %x\n", i, temptest_data[i]);

    }

ptr += sprintf(ptr, "==== otp data end====\n");

//	checksum
	checksum = 0;
	for (i = 0x1 ; i <= 0x7 ; i++)
	checksum += temptest_data[i];
	ptr += sprintf(ptr, "info valid %d, check: %x , 0x08 value:%x\n",temptest_data[0], checksum%256, temptest_data[8]);
	

		checksum =  0;
        for (i = 0xd ; i <= 0x14 ; i++)
        checksum += temptest_data[i];
        ptr += sprintf(ptr, "awb valid %d, check: %x , 0x15 value:%x\n",temptest_data[0xc], checksum%256, temptest_data[0x15]);

        checksum =  0;
        for (i = 0x17 ; i <= 0x762 ; i++)
        checksum += temptest_data[i]  ;                  
        ptr += sprintf(ptr, "lsc valid %d, check: %x , 0x763 value:%x\n",temptest_data[0x16], checksum%256, temptest_data[0x763]);


        checksum =  0;
        for (i = 0x765 ; i <= 0x954 ; i++)
        checksum += temptest_data[i];
        ptr += sprintf(ptr, "pd1 valid %d, check: %x , 0x955 value:%x\n",temptest_data[0x764], checksum%256, temptest_data[0x955]);



        checksum =  0;
        for (i = 0x957 ; i <= 0xc7c ; i++)
        checksum += temptest_data[i];
        ptr += sprintf(ptr, "pd2 valid %d, check: %x , 0xc7d value:%x\n",temptest_data[0x956], checksum%256, temptest_data[0xc7d]);
        checksum =  0;
        for (i = 0xc7f ; i <= 0xce4; i++)
        checksum += temptest_data[i];
        ptr += sprintf(ptr, "pd3 valid %d, check: %x , 0xce5 value:%x\n",temptest_data[0xc7e], checksum%256, temptest_data[0xce5]);

        checksum =  0;
        for (i = 0xce7 ; i <= 0xcec; i++)
        checksum += temptest_data[i];
        ptr += sprintf(ptr, "af valid %d, check: %x , 0xced value:%x\n",temptest_data[0xce6], checksum%256, temptest_data[0xced]);

	  len = ptr - page; 			 	
	  if(*ppos >= len)
	  {		
		  kfree(page); 		
		  return 0; 	
	  }	
	  err = copy_to_user(buffer,(char *)page,len); 			
	  *ppos += len; 	
	  if(err) 
	  {		
	    kfree(page); 		
		  return err; 	
	  }	
	  kfree(page); 	
	  return len;	

    //return (ptr - page);
}

/*add by prize, eileen*/
static ssize_t s5k3l8_otp_write_proc(struct file *file, const char *buffer, size_t count, loff_t *ppos)
/*add by prize, eileen*/
{
    return count;
}


static const struct file_operations s5k3l8_otp_proc_fops = { 
    .write = s5k3l8_otp_write_proc,
    .read = s5k3l8_otp_read_proc
};



kal_bool check_S5K3L8_otp_data(kal_uint8 page)
{

	u8 readbuff = 0;
	u16 i= 0;

    for (i = 0 ; i < 0x0CF3 ; i++)
    {
    	iReadCAM_CAL(i,1,&readbuff);
	temptest_data[i] = readbuff;
	 
    }

	
	return KAL_TRUE;
}
#endif
/* prize-lifenfen-1205 */

#if 0

kal_bool check_S5K3L8_otp_valid_LSC_Page(kal_uint8 page)
{
	kal_uint8 LSC_OK = 0x07;
	u8 readbuff, i;

	iReadCAM_CAL_8(0x04,&readbuff,0xA0);
	
	LSC_OK = readbuff;
	
	if (LSC_OK==7)
	{
		CAM_CALDB("can read LSC otp from page0~page5\n");
		CAM_CALDB("page number %d is valid\n",LSC_OK);	 
	}
	else
	    return KAL_FALSE;
	
	return KAL_TRUE;
}


 kal_bool S5K3L8_Read_LSC_Otp(kal_uint16 ui4_offset,u16 Outdatalen,unsigned char * pOutputdata)
 {
 
 //kal_uint8 page = 0;
 //kal_uint16 byteperpage = 256;
 kal_uint16 number = 0;
 //kal_uint16 LSCOTPaddress = 0x00 ; 
 //u8 readbuff=0;
 u16 i = 0;
 int checksum=0;
 int a6_checksum=0;
 //int check_i=0;
 unsigned returnData=0;
 //int add;
 int page=0,page_add=0;
 //int INF_L=0,INL_H=0,MAC_L=0,MAC_H=0,CHECKSUM=0,INF=0,MAC=0;
 

 
	if(Outdatalen==1868)
	{
		
   
			
			 for(i=1;i<=3;i++)
			 {
			     number=0;
			 checksum=0;
			 for(page=2;page<32;page++)
	   	     {
			 
			 if(page==31)
			 {
			     for(page_add=0x00;page_add<0x0d;page_add++)//0x763 ((0x17~0x762) %256)
			     {
				zc533_read_register(page,page_add,&returnData);
				
			
				pOutputdata[number]=returnData;
				
				checksum=checksum+pOutputdata[number];	  
			  	//printk("pOutputdata[%d]=%x\n",number,pOutputdata[number]);
			  
			  	number+=1;
			     }
				// printk("for number=%d\n",number);
			 }else if(page==2)
			 {
			    for(page_add=0x01;page_add<=0x3f;page_add++)//0x763 ((0x17~0x762) %256)
			     {
				zc533_read_register(page,page_add,&returnData);
				
				pOutputdata[number]=returnData;
					
				
				checksum=checksum+pOutputdata[number];	  
			  	//printk("pOutputdata[%d]=%x\n",number,pOutputdata[number]);
			  
			  	number+=1;
			     }
			 
			 }else
			 {
			     for(page_add=0x00;page_add<=0x3f;page_add++)//0x763 ((0x17~0x762) %256)
			     {
				zc533_read_register(page,page_add,&returnData);
				
				pOutputdata[number]=returnData;
					
				
				checksum=checksum+pOutputdata[number];	  
			  	//printk("pOutputdata[%d]=%x\n",number,pOutputdata[number]);
			  
			  	number+=1;
			     }
	          }
			  
	         }
		zc533_read_register(31,0x0d,&returnData);
		a6_checksum =returnData;
		  //   printk("for number=%d\n",number);
		 //printk("a6_checksum=%x checksum=%x value=%d\n",a6_checksum,(checksum%255+1),checksum);
		 if(a6_checksum==(checksum%255+1))
		   {
			   printk("camera 3l8 otp lsc ok =%d i=%d!!! \n",number,i);
			   break;
		   }
		      

		}
          return KAL_TRUE;
			 
		 
	}
	else
	{
		printk("camera 3l8 otp lsc Length donot include");
		 return KAL_FALSE;
	}

 
 }
#endif


 void S5K3L8_ReadOtp(kal_uint8 page,kal_uint16 address,unsigned char *iBuffer,unsigned int buffersize)
{
		kal_uint16 i = 0;
		u8 readbuff;
		int ret ;
		//base_add=(address/10)*16+(address%10);
			
		CAM_CALDB("[CAM_CAL]ENTER page:0x%x address:0x%x buffersize:%d\n ",page, address, buffersize);
			for(i = 0; i<buffersize; i++)
			{				
				ret= iReadCAM_CAL(address+i, 1, &readbuff);
				//CAM_CALDB("[CAM_CAL]address+%d = 0x%x,readbuff = 0x%x\n",i,(address+i),readbuff );
				*(iBuffer+i) =(unsigned char)readbuff;
			}
}


//Burst Write Data
static int iWriteData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
	return 1;
}


static bool selective_read_eeprom(kal_uint16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    if(addr > S5K3L8_MAX_OFFSET)
        return false;
	kdSetI2CSpeed(S5K3L8_I2C_SPEED);

	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, S5K3L8_EEPROM_READ_ID)<0)
		return false;
    return true;
}

 bool _read_s5k3l8_eeprom(kal_uint16 addr, BYTE* data, kal_uint32 size ){
	int i = 0;
	int offset = addr;
	for(i = 0; i < size; i++) {
		if(!selective_read_eeprom(offset, &data[i])){
			return false;
		}
		printk("###########   _read_s5k3l8_eeprom 0x%0x %d\n",offset, data[i]);
		offset++;
	}
	get_done = true;
	last_size = size;
	last_offset = addr;
    return true;
}

//Burst Read Data
 int iReadData(kal_uint16 ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
   //int  i4RetValue = 0;
    //kal_uint8 page = 0, pageS=0,pageE=1;
	//1. check which page is valid
	//int INF_L=0,INL_H=0,MAC_L=0,MAC_H=0,CHECKSUM=0,INF=0,MAC=0,af_sum=0;
	//int INF_L=0,INL_H=0,MAC_L=0,MAC_H=0;
	//kal_uint8 check_mid[1];
	static kal_uint8 S5K3L8MIPI_LSC_Data[1868];
	static kal_uint8 S5K3L8MIPI_LSC_Data_checksum=0;
	unsigned int check_sum=0;
	int i;
	//int j;
	printk("[CAM_CAL] iReadData offset=%d length=%d \n", ui4_offset,ui4_length);
    if(ui4_offset==0)//id
    	{
		 //zc533_read_register(0x00,0x01,&check_mid);
		   //S5K3L8_ReadOtp();
		   //_read_s5k3l8_eeprom();
		   //_read_s5k3l8_eeprom(0x0001,check_mid,1);
		   //_read_s5k3l8_eeprom(0x003A,check_mid,1);
		//pinputdata[0]=check_mid[0];
		pinputdata[0]=1;
		printk("[CAM_CAL] iReadData ui4_offset==0  pinputdata[0]=0x%x\n \n", pinputdata[0]);
		
	
    	}
	 else if(ui4_offset==44)
		{
					printk("[CAM_CAL]LSC OTP Data Read!\n");
			_read_s5k3l8_eeprom(0x002C,S5K3L8MIPI_LSC_Data,1868);
		
			for(i=0;i<1868;i++)
			{
				check_sum=check_sum+S5K3L8MIPI_LSC_Data[i];
				pinputdata[i] = S5K3L8MIPI_LSC_Data[i];
			}
		
			_read_s5k3l8_eeprom(0x0778,&S5K3L8MIPI_LSC_Data_checksum,1);	
			check_sum=check_sum%256;
			
			
				//pinputdata=S5K3L8MIPI_LSC_Data;
			
		printk("[CAM_CAL] iReadData ui4_offset==59  check_sum=0x%x,S5K3L8MIPI_LSC_Data_checksum =0x%x \n",check_sum,S5K3L8MIPI_LSC_Data_checksum);
		}
    else if(ui4_offset==33)
    	{
    		
		    //
		//   zc533_read_register(32,0x01,&INF_L);
		//   zc533_read_register(32,0x02,&INL_H);
		   //INF=(INF_L+(INL_H<<8));
		   //pinputdata[0]=INF_L;
		   //pinputdata[1]=INL_H;
		   	//printk("af INF_L=%x INF_L=%x pinputdata=(%x %x)\n",INF_L,INL_H,pinputdata[0], pinputdata[1]);
    	}
    else if(ui4_offset==35)
		{
	    //   zc533_read_register(32,0x03,&MAC_L);
		  // zc533_read_register(32,0x04,&MAC_H);
		   //MAC=(MAC_L+(MAC_H<<8));
		  //pinputdata[0]=MAC_L;
		   //pinputdata[1]=MAC_H;
		 // printk("af MAC_L=%x MAC_H=%x pinputdata=(%x %x)\n",MAC_L,MAC_H,pinputdata[0], pinputdata[1]);
		//  zc533_read_register(32,0x05,&af_sum);
		  	//printk("af MAC=%d pinputdata=%d\n",MAC,pinputdata[0]);
           //  CHECKSUM=INF_L+INL_H+MAC_L+MAC_H;
			// printk("[%x:%x:%x:%x]\n [%d:%d:%d:%d] %d",INF_L,INL_H,MAC_L,MAC_H,INF_L,INL_H,MAC_L,MAC_H,CHECKSUM);
    		//CHECKSUM=(CHECKSUM%255+1);
			//printk("CHECKSUM=%d af_sum=%d\n",CHECKSUM,af_sum);
			
    }
	
	else if(ui4_offset==59)//Lsc
		{
			printk("[CAM_CAL]LSC OTP Data Read!\n");
			_read_s5k3l8_eeprom(0x0016,S5K3L8MIPI_LSC_Data,1868);
		
			for(i=0;i<1868;i++)
			{
				check_sum=check_sum+S5K3L8MIPI_LSC_Data[i];
				pinputdata[i] = S5K3L8MIPI_LSC_Data[i];
			}
		
			_read_s5k3l8_eeprom(0x0762,&S5K3L8MIPI_LSC_Data_checksum,1);	
			check_sum=check_sum%255+1;
			
			
				//pinputdata=S5K3L8MIPI_LSC_Data;
			
		printk("[CAM_CAL] iReadData ui4_offset==59  check_sum=0x%x,S5K3L8MIPI_LSC_Data_checksum =0x%x \n",check_sum,S5K3L8MIPI_LSC_Data_checksum);
	  }
	  
	else
		{
		printk("[CAM_CAL] ui4_offset not include\n");
		}
  

  
   return 0;
}
///////////////////////////////////fch

#ifdef CONFIG_COMPAT
static int compat_put_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    //compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data->u4Offset);
    err |= put_user(i, &data32->u4Offset);
    err |= get_user(i, &data->u4Length);
    err |= put_user(i, &data32->u4Length);
 
#if 0
    err |= get_user(p, &data->pu1Params);
    err |= put_user(p, &data32->pu1Params);
#endif
    return err;
}

static int compat_get_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data32->u4Offset);
    err |= put_user(i, &data->u4Offset);
    err |= get_user(i, &data32->u4Length);
    err |= put_user(i, &data->u4Length);
    err |= get_user(p, &data32->pu1Params);
    err |= put_user(compat_ptr(p), &data->pu1Params);

    return err;
}


static long s5k3l8_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret;

	COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
	stCAM_CAL_INFO_STRUCT __user *data;
	int err;

	CAM_CALDB("[CAMERA SENSOR] IMX135_OTP_DEVICE_ID,%p %p %x ioc size %d\n",
	filp->f_op , filp->f_op->unlocked_ioctl, cmd, _IOC_SIZE(cmd));

	if (!filp->f_op || !filp->f_op->unlocked_ioctl)
		return -ENOTTY;

	printk("lixf otp COMPAT_CAM_CALIOC_G_READ:%lu,CAM_CALIOC_G_READ:%lu,cmd:%x\n",COMPAT_CAM_CALIOC_G_READ,CAM_CALIOC_G_READ,cmd);
	printk("lixf otp 000\n");
	switch (cmd) {
	printk("lee\n");
	case COMPAT_CAM_CALIOC_G_READ: {
	//case CAM_CALIOC_G_READ: {
		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_cal_info_struct(data32, data);
		if (err)
			return err;

		ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ, (unsigned long)data);
		err = compat_put_cal_info_struct(data32, data);


		if (err != 0)
			printk("[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
		return ret;
	}
	default:
	{
		printk("lixf otp 222\n");
		return -ENOIOCTLCMD;
	}
	}
}
#endif
///////////////////////////////



/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode * a_pstInode,
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
#else 
static long CAM_CAL_Ioctl(
    struct file *file, 
    unsigned int a_u4Command, 
    unsigned long a_u4Param
)
#endif
{
    int i4RetValue = 0;
    u8 * pBuff = NULL;
    u8 * pWorkingBuff = NULL;
    stCAM_CAL_INFO_STRUCT *ptempbuf;
	
#ifdef CAM_CALGETDLT_DEBUG
    struct timeval ktv1, ktv2;
    unsigned long TimeIntervalUS;
#endif
CAM_CALDB("fucehou----ioctl\n");
printk("lixf otp CAM_CAL_Ioctl\n");
    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);
        //pu1Params = kmalloc(ptempbuf->u4Length, GFP_KERNEL);

        if(NULL == pBuff)
        {
            CAM_CALDB("[CAM_CAL] ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                CAM_CALDB("[CAM_CAL] ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pWorkingBuff = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL); 
    if(NULL == pWorkingBuff)
    {
        kfree(pBuff);
        CAM_CALDB("[CAM_CAL] ioctl allocate mem failed\n");
        return -ENOMEM;
    }
     CAM_CALDB("[CAM_CAL] init Working buffer address 0x%p  command is 0x%8x\n", pWorkingBuff, (u32)a_u4Command);//fch

 
    if(copy_from_user((u8*)pWorkingBuff ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pWorkingBuff);
        CAM_CALDB("[CAM_CAL] ioctl copy from user failed\n");
        return -EFAULT;
    } 
    
    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:    
            CAM_CALDB("[CAM_CAL] Write CMD \n");
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif            
            i4RetValue = iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            printk("Write data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif            
            break;
        case CAM_CALIOC_G_READ:
            CAM_CALDB("[CAM_CAL] Read CMD \n");
#ifdef CAM_CALGETDLT_DEBUG            
            do_gettimeofday(&ktv1);
#endif     
            CAM_CALDB("[CAM_CAL] offset %d \n", ptempbuf->u4Offset);
            CAM_CALDB("[CAM_CAL] length %d \n", ptempbuf->u4Length);
            CAM_CALDB("[CAM_CAL] Before read Working buffer address 0x%p \n", pWorkingBuff);//fch
			//otp_flag=1;
            i4RetValue = iReadData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
			
            CAM_CALDB("[CAM_CAL] After read Working buffer data  0x%4x i4RetValue=%d\n", *pWorkingBuff,i4RetValue);


#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            printk("Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif            

            break;
        default :
      	     CAM_CALDB("[CAM_CAL] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        CAM_CALDB("[CAM_CAL] to user length %d \n", ptempbuf->u4Length);
        //CAM_CALDB("[CAM_CAL] to user  Working buffer address 0x%8x \n", (u32)pWorkingBuff);//fch
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pWorkingBuff , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pWorkingBuff);
            CAM_CALDB("[CAM_CAL] ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pWorkingBuff);
    return i4RetValue;
}


static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    CAM_CALDB("[CAM_CAL] CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
		CAM_CALDB("[CAM_CAL] Opened, return -EBUSY\n");
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_CAM_CALatomic,0);
    }
    spin_unlock(&g_CAM_CALLock);

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_CAM_CALLock);

    g_u4Opened = 0;

    atomic_set(&g_CAM_CALatomic,0);

    spin_unlock(&g_CAM_CALLock);

    return 0;
}

static const struct file_operations g_stCAM_CAL_fops =
{
    .owner = THIS_MODULE,
    .open = CAM_CAL_Open,
    .release = CAM_CAL_Release,
    //.ioctl = CAM_CAL_Ioctl
    #ifdef CONFIG_COMPAT
    .compat_ioctl = s5k3l8_Ioctl_Compat,
   // .compat_ioctl = CAM_CAL_Ioctl,
    #endif
    .unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
inline static int RegisterCAM_CALCharDrv(void)
{
    struct device* CAM_CAL_device = NULL;

	CAM_CALDB("[CAM_CAL] RegisterCAM_CALCharDrv\n");
#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_CAM_CALdevno, 0, 1,CAM_CAL_DRVNAME) )
    {
        CAM_CALDB("[CAM_CAL] Allocate device no failed\n");

        return -EAGAIN;
    }
#else
    if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
    {
        CAM_CALDB("[CAM_CAL] Register device no failed\n");

        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);

        CAM_CALDB("[CAM_CAL] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        CAM_CALDB("[CAM_CAL] Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }

    CAM_CAL_class = class_create(THIS_MODULE, "CAM_CALdrv");
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        CAM_CALDB("Unable to create class, err = %d\n", ret);
        return ret;
    }
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

    return 0;
}

inline static void UnregisterCAM_CALCharDrv(void)
{
    //Release char driver
    cdev_del(g_pCAM_CAL_CharDrv);

    unregister_chrdev_region(g_CAM_CALdevno, 1);

    device_destroy(CAM_CAL_class, g_CAM_CALdevno);
    class_destroy(CAM_CAL_class);
}


//////////////////////////////////////////////////////////////////////
#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
#elif 0
static int CAM_CAL_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#else
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int CAM_CAL_i2c_remove(struct i2c_client *);

static const struct i2c_device_id CAM_CAL_i2c_id[] = {{CAM_CAL_DRVNAME,0},{}};   
#if 0 //test110314 Please use the same I2C Group ID as Sensor
static unsigned short force[] = {CAM_CAL_I2C_GROUP_ID, S5K3L8OTP_DEVICE_ID, I2C_CLIENT_END, I2C_CLIENT_END};   
#else
//static unsigned short force[] = {IMG_SENSOR_I2C_GROUP_ID, OV5647OTP_DEVICE_ID, I2C_CLIENT_END, I2C_CLIENT_END};   
#endif
//static const unsigned short * const forces[] = { force, NULL };              
//static struct i2c_client_address_data addr_data = { .forces = forces,}; 

static const struct of_device_id otp_of_match[] = {
	{.compatible = "mediatek,main_camera_otp"},
	{},
};

static struct i2c_driver CAM_CAL_i2c_driver = {
    .probe = CAM_CAL_i2c_probe,                                   
    .remove = CAM_CAL_i2c_remove,                           
//   .detect = CAM_CAL_i2c_detect,
		 .driver.of_match_table = otp_of_match,                           
    .driver.name = CAM_CAL_DRVNAME,
    .id_table = CAM_CAL_i2c_id,                             
};

#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {         
    strcpy(info->type, CAM_CAL_DRVNAME);
    return 0;
}
#endif

static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {             
int i4RetValue = 0;
    CAM_CALDB("[CAM_CAL] CAM_CAL_i2c_probe  \n");
//    spin_lock_init(&g_CAM_CALLock);

    //get sensor i2c client
    printk("lixf CAM_CAL_i2c_probe\n");
    spin_lock(&g_CAM_CALLock); //for SMP
    g_pstI2Cclient = client;
    g_pstI2Cclient->addr = S5K3L8OTP_DEVICE_ID>>1;
    spin_unlock(&g_CAM_CALLock); // for SMP    
    
    CAM_CALDB("[CAM_CAL] g_pstI2Cclient->addr = 0x%8x \n",g_pstI2Cclient->addr);
    //Register char driver
    /* Register char driver */
		i4RetValue = RegisterCAM_CALCharDrv();
		if (i4RetValue) {
			CAM_CALDB(" register char device failed!\n");
			return i4RetValue;
		}

    if(i4RetValue){
        CAM_CALDB("[CAM_CAL] register char device failed!\n");
        return i4RetValue;
    }


    CAM_CALDB("[CAM_CAL] Attached!! \n");
    return 0;                                                                                       
} 

static int CAM_CAL_i2c_remove(struct i2c_client *client)
{
    return 0;
}

static int CAM_CAL_probe(struct platform_device *pdev)
{
/* prize-lifenfen-1205 */
CAM_CALDB("[CAM_CAL] CAM_CAL_probe platform \n");

#ifdef CAM_CAL_DEBUG
    // Create proc file system
    s5k3l8_otp_proc = proc_create("otp", 0660, NULL, &s5k3l8_otp_proc_fops);
    if (s5k3l8_otp_proc == NULL)
    {
        printk("create_proc_entry otp failed\n");
    }
#endif
/* prize-lifenfen-1205 */

    return i2c_add_driver(&CAM_CAL_i2c_driver);
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
    i2c_del_driver(&CAM_CAL_i2c_driver);
    return 0;
}

// platform structure
static struct platform_driver g_stCAM_CAL_Driver = {
    .probe		= CAM_CAL_probe,
    .remove	= CAM_CAL_remove,
    .driver		= {
        .name	= CAM_CAL_DRVNAME,
        .owner	= THIS_MODULE,
    }
};


static struct platform_device g_stCAM_CAL_Device = {
    .name = CAM_CAL_DRVNAME,
    .id = 0,
    .dev = {
    }
};

static int __init CAM_CAL_i2C_init(void)
{

	
		CAM_CALDB("CAM_CAL_i2C_init\n");
	
	CAM_CALDB(" Attached!!\n");
	  printk("CAM_CAL __init\n");
    //i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1);
    if(platform_driver_register(&g_stCAM_CAL_Driver)){
        CAM_CALDB("failed to register CAM_CAL driver\n");
        return -ENODEV;
    }

    if (platform_device_register(&g_stCAM_CAL_Device))
    {
        CAM_CALDB("failed to register CAM_CAL driver, 2nd time\n");
        return -ENODEV;
    }	

    return 0;
}

static void __exit CAM_CAL_i2C_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_i2C_init);
module_exit(CAM_CAL_i2C_exit);

//MODULE_DESCRIPTION("CAM_CAL driver");
//MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
//MODULE_LICENSE("GPL");



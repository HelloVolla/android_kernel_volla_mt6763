1.选择平台
平台是必选项，根据项目平台选择对应选项，填入 sf_user.h 的 SF_PLATFORM_SEL 之后，Android 8.0 后，REE 与 TEE 共用一套代码
【REE】
SF_REE_MTK             // REE MTK 平台
SF_REE_QUALCOMM        // REE 高通平台
SF_REE_SPREAD          // REE 展讯平台
SF_REE_HIKEY9600       // REE 麒麟960
SF_REE_MTK_L5_X        // REE MTK 5.x 平台

【TEE】
SF_TEE_BEANPOD         // TEE 豆荚平台
SF_TEE_TRUSTKERNEL     // TEE 瓶钵平台
SF_TEE_QSEE            // TEE 高通平台
SF_TEE_TRUSTONIC       // TEE trustonic 平台
SF_TEE_RONGCARD        // TEE 融卡平台
SF_TEE_TRUSTY          // TEE 展讯平台


2.多指纹兼容选择
兼容选择是必选项，默认不做兼容，根据项目是否需要与其他指纹厂商兼容而现在对应选项，填入 sf_user.h 的 SF_COMPATIBLE_SEL
之后
SF_COMPATIBLE_NOF           // 不做兼容
SF_COMPATIBLE_NOF_BP_V2_7   // beanpod pay ISEE2.7.0 不做兼容
SF_COMPATIBLE_REE           // 所有【REE】兼容，包括 SF_REE_MTK、SF_REE_QUALCOMM、SF_REE_SPREAD 三种平台
SF_COMPATIBLE_BEANPOD_V1    // TEE 豆荚兼容 V1，使用 V1 或者 V2 或者 V2_7 要询问项目的豆荚工程师
SF_COMPATIBLE_BEANPOD_V2    // TEE 豆荚兼容 V2，使用 V1 或者 V2 或者 V2_7 要询问项目的豆荚工程师
SF_COMPATIBLE_BEANPOD_V2_7  // TEE 豆荚兼容 V2，使用 V1 或者 V2 或者 V2_7 要询问项目的豆荚工程师
SF_COMPATIBLE_TRUSTKERNEL   // TEE 瓶钵兼容
SF_COMPATIBLE_QSEE          // TEE 高通兼容
SF_COMPATIBLE_TRUSTY        // TEE 展讯兼容
SF_COMPATIBLE_RONGCARD      // TEE 融卡兼容
SF_COMPATIBLE_TRUSTONIC     // TEE trustonic 兼容


3.指纹 IC 供电控制方式选择
IC 供电控制方式选择是可选项，为保证指纹 IC 在 HAL 初始化时已上电，请保证选择正确供电控制方式，填入 sf_user.h 的
SF_POWER_MODE_SEL 之后，否则开机读 ID 可能失败
PWR_MODE_NOF                // 无需控制指纹供电
PWR_MODE_GPIO               // GPIO 方式控制指纹供电
PWR_MODE_REGULATOR          // regulator 方式控制指纹供电


4.填写 DTS 信息
4.1 填写信炜指纹 DTS 节点，填入 sf_user.h 的 COMPATIBLE_SW_FP 之后
4.2 填写复位、中断和电源 DTS 信息
针对 MTK 平台，请填写 FINGER_POWER_ON、FINGER_POWER_OFF、FINGER_RESET_LOW、FINGER_RESET_HIGH 和 FINGER_INT_SET 的 pinctl
信息
针对高通和展讯平台，请填写 COMPATIBLE_RESET_GPIO、COMPATIBLE_IRQ_GPIO 和 COMPATIBLE_PWR_GPIO 等 DTS 信息
针对 regulator 方式供电（3.指纹 IC 供电控制方式选择 中选择 PWR_MODE_REGULATOR 的），必须填写 SF_VDD_NAME，即 vdd 在 DTS
中的信息


5.其余宏定义请保持默认


//-----------------------------------------------------------------------------------------------------------
以下是关于多指纹 IC 兼容的旧资料，用于旧驱动，仅供参考

一、豆荚兼容 V1:
做豆荚兼容时，如果豆荚sboot里面添加了读id功能，会通过get_t_device_id函数传出我们的id值，
现在默认我们是传出0x02，
get_t_device_id在nt_smc_call.h头文件里面；

瓶钵兼容:
sf_read_sensor_id里面包括了我们所有读id功能，这里需要注意的是，
由于屏蔽默认做兼容时只能读写8个字节，并且只能读写8次, 但8201、8211读id会读10个字节，
如果打开串口log看会有报错，所以默认把8201、8211读id功能屏蔽，
如果需要做8201、8211兼容，需先跟瓶钵沟通好放宽读id字节数；

二、考虑到一次完成兼容现在 IC ID, 统一在 reset 脚拉高 200 ms 之后读 ID，调试时，请向 TEE 方案商强调这个时间；

三、瓶钵兼容跟ree下面兼容差不多，会有选择不同的cs片选，所以兼容时注意spi_board_devs中的chip_select以及修改平台num_chipselect；

四、由于不同平台对应enable clk的函数不同，所以这部分需要按平台来修改，现在默认是用mt_spi_enable_clk来使能clk；

五、针对不同客户，dts、供电这部分会有不同，这部分需按不同客户现场修改；

六、豆荚sboot里面读id参考代码：

spi：spi速度配置为500k
chip_config->setuptime = 10;
chip_config->holdtime = 10;
chip_config->high_time = 120; // 120->500k
chip_config->low_time = 120;
chip_config->cs_idletime = 10;
chip_config->ulthgh_thrsh = 0;

#define VENDOR_ID_SUNWAVE 0x02

static int get_fp_sensor_id()
{
    int tlRet = 0;
    int trytimes = 0;
    unsigned char spi_buffer[64];
    unsigned char id_buffer[64];
    memset(spi_buffer, 0, sizeof(spi_buffer));
    memset(id_buffer, 0, sizeof(id_buffer));
    printf("---xinan sunwave wake_up 2017-05-04 start!--\n");

    do {
        延时5ms
        spi_buffer[0] = 0x60;
        spi_buffer[1] = 0x9f;
        spi_buffer[2] = 0x28;
        spi_buffer[3] = 0x02;
        spi_buffer[4] = 0x00;
        tlRet = drSpiSend(spi_buffer, id_buffer, 5, 7, &spi_chip_config, 1);
        printf("xinan get vendor_id = %02x\n", id_buffer[5]);

        if (tlRet < 0) {
            printf("%s:%d xinan for sunwave spi_write failed , tlRet = %d", __FUNCTION__, __LINE__, tlRet);
            return 0;
        }

        trytimes++;

        if (id_buffer[5] == 0x82) {
            printf("xinan entry vendor_sunwave\n");
            return VENDOR_ID_SUNWAVE;
        }
    }
    while (trytimes <= 3)
        do {
            延时5ms
            spi_buffer[0] = 0x60;
            spi_buffer[1] = 0x28;
            spi_buffer[2] = 0x02;
            spi_buffer[3] = 0x00;
            tlRet = drSpiSend(spi_buffer, id_buffer, 4, 6, &spi_chip_config, 1);
            printf("xinan get vendor_id = %02x\n", id_buffer[4]);

            if (tlRet < 0) {
                printf("%s:%d xinan for sunwave spi_write failed , tlRet = %d", __FUNCTION__, __LINE__, tlRet);
                return 0;
            }

            trytimes++;

            if (id_buffer[4] == 0x82) {
                printf("xinan entry vendor_sunwave\n");
                return VENDOR_ID_SUNWAVE;
            }
        }
        while (trytimes <= 3)
            延时100ms
            printf("---xinan sunwave wake_up start!--\n");

    spi_buffer[0] = 0x1c;
    spi_buffer[1] = 0x1c;
    spi_buffer[2] = 0x1c;
    tlRet = drSpiSend(spi_buffer, id_buffer, 3, 3, &spi_chip_config, 1);

    if (tlRet < 0) {
        printf("%s:%d xinan for sunwave spi_write failed , tlRet = %d\n", __FUNCTION__, __LINE__, tlRet);
        //return -2;
    }

    延时5ms
    trytimes = 0;
    printf("xinan  sunwave write cmd start!\n");

    do {
        spi_buffer[0] = 0x96;
        spi_buffer[1] = 0x69;
        spi_buffer[2] = 0x00;
        spi_buffer[3] = 0x00;
        spi_buffer[4] = 0x1e;
        spi_buffer[5] = 0x00;
        spi_buffer[6] = 0x02;
        spi_buffer[7] = 0x00;
        tlRet = drSpiSend(spi_buffer, id_buffer, 8, 10, &spi_chip_config, 1);
        printf("xinan get vendor_id = %02x, %02x\n", id_buffer[8], id_buffer[9]);
        trytimes++;

        if ((id_buffer[8] == 0xFA) || (id_buffer[9] == 0xFA)) {
            return VENDOR_ID_SUNWAVE;
        }
    }
    while (trytimes <= 3)
        return 0;
}

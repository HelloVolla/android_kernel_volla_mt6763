1.ѡ��ƽ̨
ƽ̨�Ǳ�ѡ�������Ŀƽ̨ѡ���Ӧѡ����� sf_user.h �� SF_PLATFORM_SEL ֮��Android 8.0 ��REE �� TEE ����һ�״���
��REE��
SF_REE_MTK             // REE MTK ƽ̨
SF_REE_QUALCOMM        // REE ��ͨƽ̨
SF_REE_SPREAD          // REE չѶƽ̨
SF_REE_HIKEY9600       // REE ����960
SF_REE_MTK_L5_X        // REE MTK 5.x ƽ̨

��TEE��
SF_TEE_BEANPOD         // TEE ����ƽ̨
SF_TEE_TRUSTKERNEL     // TEE ƿ��ƽ̨
SF_TEE_QSEE            // TEE ��ͨƽ̨
SF_TEE_TRUSTONIC       // TEE trustonic ƽ̨
SF_TEE_RONGCARD        // TEE �ڿ�ƽ̨
SF_TEE_TRUSTY          // TEE չѶƽ̨


2.��ָ�Ƽ���ѡ��
����ѡ���Ǳ�ѡ�Ĭ�ϲ������ݣ�������Ŀ�Ƿ���Ҫ������ָ�Ƴ��̼��ݶ����ڶ�Ӧѡ����� sf_user.h �� SF_COMPATIBLE_SEL
֮��
SF_COMPATIBLE_NOF           // ��������
SF_COMPATIBLE_NOF_BP_V2_7   // beanpod pay ISEE2.7.0 ��������
SF_COMPATIBLE_REE           // ���С�REE�����ݣ����� SF_REE_MTK��SF_REE_QUALCOMM��SF_REE_SPREAD ����ƽ̨
SF_COMPATIBLE_BEANPOD_V1    // TEE ���Լ��� V1��ʹ�� V1 ���� V2 ���� V2_7 Ҫѯ����Ŀ�Ķ��Թ���ʦ
SF_COMPATIBLE_BEANPOD_V2    // TEE ���Լ��� V2��ʹ�� V1 ���� V2 ���� V2_7 Ҫѯ����Ŀ�Ķ��Թ���ʦ
SF_COMPATIBLE_BEANPOD_V2_7  // TEE ���Լ��� V2��ʹ�� V1 ���� V2 ���� V2_7 Ҫѯ����Ŀ�Ķ��Թ���ʦ
SF_COMPATIBLE_TRUSTKERNEL   // TEE ƿ������
SF_COMPATIBLE_QSEE          // TEE ��ͨ����
SF_COMPATIBLE_TRUSTY        // TEE չѶ����
SF_COMPATIBLE_RONGCARD      // TEE �ڿ�����
SF_COMPATIBLE_TRUSTONIC     // TEE trustonic ����


3.ָ�� IC ������Ʒ�ʽѡ��
IC ������Ʒ�ʽѡ���ǿ�ѡ�Ϊ��ָ֤�� IC �� HAL ��ʼ��ʱ���ϵ磬�뱣֤ѡ����ȷ������Ʒ�ʽ������ sf_user.h ��
SF_POWER_MODE_SEL ֮�󣬷��򿪻��� ID ����ʧ��
PWR_MODE_NOF                // �������ָ�ƹ���
PWR_MODE_GPIO               // GPIO ��ʽ����ָ�ƹ���
PWR_MODE_REGULATOR          // regulator ��ʽ����ָ�ƹ���


4.��д DTS ��Ϣ
4.1 ��д���ָ�� DTS �ڵ㣬���� sf_user.h �� COMPATIBLE_SW_FP ֮��
4.2 ��д��λ���жϺ͵�Դ DTS ��Ϣ
��� MTK ƽ̨������д FINGER_POWER_ON��FINGER_POWER_OFF��FINGER_RESET_LOW��FINGER_RESET_HIGH �� FINGER_INT_SET �� pinctl
��Ϣ
��Ը�ͨ��չѶƽ̨������д COMPATIBLE_RESET_GPIO��COMPATIBLE_IRQ_GPIO �� COMPATIBLE_PWR_GPIO �� DTS ��Ϣ
��� regulator ��ʽ���磨3.ָ�� IC ������Ʒ�ʽѡ�� ��ѡ�� PWR_MODE_REGULATOR �ģ���������д SF_VDD_NAME���� vdd �� DTS
�е���Ϣ


5.����궨���뱣��Ĭ��


//-----------------------------------------------------------------------------------------------------------
�����ǹ��ڶ�ָ�� IC ���ݵľ����ϣ����ھ������������ο�

һ�����Լ��� V1:
�����Լ���ʱ���������sboot��������˶�id���ܣ���ͨ��get_t_device_id�����������ǵ�idֵ��
����Ĭ�������Ǵ���0x02��
get_t_device_id��nt_smc_call.hͷ�ļ����棻

ƿ������:
sf_read_sensor_id����������������ж�id���ܣ�������Ҫע����ǣ�
��������Ĭ��������ʱֻ�ܶ�д8���ֽڣ�����ֻ�ܶ�д8��, ��8201��8211��id���10���ֽڣ�
����򿪴���log�����б�������Ĭ�ϰ�8201��8211��id�������Σ�
�����Ҫ��8201��8211���ݣ����ȸ�ƿ����ͨ�÷ſ��id�ֽ�����

�������ǵ�һ����ɼ������� IC ID, ͳһ�� reset ������ 200 ms ֮��� ID������ʱ������ TEE ������ǿ�����ʱ�䣻

����ƿ�����ݸ�ree������ݲ�࣬����ѡ��ͬ��csƬѡ�����Լ���ʱע��spi_board_devs�е�chip_select�Լ��޸�ƽ̨num_chipselect��

�ġ����ڲ�ͬƽ̨��Ӧenable clk�ĺ�����ͬ�������ⲿ����Ҫ��ƽ̨���޸ģ�����Ĭ������mt_spi_enable_clk��ʹ��clk��

�塢��Բ�ͬ�ͻ���dts�������ⲿ�ֻ��в�ͬ���ⲿ���谴��ͬ�ͻ��ֳ��޸ģ�

��������sboot�����id�ο����룺

spi��spi�ٶ�����Ϊ500k
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
        ��ʱ5ms
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
            ��ʱ5ms
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
            ��ʱ100ms
            printf("---xinan sunwave wake_up start!--\n");

    spi_buffer[0] = 0x1c;
    spi_buffer[1] = 0x1c;
    spi_buffer[2] = 0x1c;
    tlRet = drSpiSend(spi_buffer, id_buffer, 3, 3, &spi_chip_config, 1);

    if (tlRet < 0) {
        printf("%s:%d xinan for sunwave spi_write failed , tlRet = %d\n", __FUNCTION__, __LINE__, tlRet);
        //return -2;
    }

    ��ʱ5ms
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

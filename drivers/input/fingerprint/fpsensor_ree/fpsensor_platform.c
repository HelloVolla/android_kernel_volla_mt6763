#include "fpsensor_platform.h"
#include "fpsensor_spi.h"
#include "fpsensor_spi_mt65xx.h"

struct mt_chip_conf fpsensor_spi_conf_mt65xx = {
    .setuptime = 15,
    .holdtime = 15,
    .high_time = 21, //for mt6582, 104000khz/(4+4) = 130000khz
    .low_time = 21,
    .cs_idletime = 20,
    .ulthgh_thrsh = 0,

    .cpol = 0,
    .cpha = 0,

    .rx_mlsb = 1,
    .tx_mlsb = 1,

    .tx_endian = 0,
    .rx_endian = 0,

    .com_mod = FIFO_TRANSFER,
    .pause = 1,
    .finish_intr = 1,
    .deassert = 0,
    .ulthigh = 0,
    .tckdly = 0,
};


typedef enum {
    SPEED_500KHZ = 500,
    SPEED_1MHZ = 1000,
    SPEED_2MHZ = 2000,
    SPEED_3MHZ = 3000,
    SPEED_4MHZ = 4000,
    SPEED_6MHZ = 6000,
    SPEED_8MHZ = 8000,
    SPEED_KEEP,
    SPEED_UNSUPPORTED
} SPI_SPEED;

void fpsensor_spi_set_mode(struct spi_device *spi, SPI_SPEED speed, int flag)
{

    struct mt_chip_conf *mcc = &fpsensor_spi_conf_mt65xx;
    if (flag == 0) {
        mcc->com_mod = FIFO_TRANSFER;
    } else {
        mcc->com_mod = DMA_TRANSFER;
    }

    switch (speed) {
    case SPEED_500KHZ:
        mcc->high_time = 120;
        mcc->low_time = 120;
        break;
    case SPEED_1MHZ:
        mcc->high_time = 60;
        mcc->low_time = 60;
        break;
    case SPEED_2MHZ:
        mcc->high_time = 30;
        mcc->low_time = 30;
        break;
    case SPEED_3MHZ:
        mcc->high_time = 20;
        mcc->low_time = 20;
        break;
    case SPEED_4MHZ:
        mcc->high_time = 15;
        mcc->low_time = 15;
        break;
    case SPEED_6MHZ:
        mcc->high_time = 10;
        mcc->low_time = 10;
        break;
    case SPEED_8MHZ:
        mcc->high_time = 8;
        mcc->low_time = 8;
        break;
    case SPEED_KEEP:
    case SPEED_UNSUPPORTED:
        break;
    }
    if (spi_setup(spi) < 0) {
        fpsensor_debug(ERR_LOG, "fpsensor:Failed to set spi.\n");
    }
}

/* -------------------------------------------------------------------- */
int fpsensor_spi_setup(fpsensor_data_t *fpsensor)
{
    int error = 0;

    FUNC_ENTRY();
    fpsensor->spi->mode = SPI_MODE_0;
    fpsensor->spi->bits_per_word = 8;
//    fpsensor->spi->chip_select = 0;
     fpsensor->spi->controller_data = (void *)&fpsensor_spi_conf_mt65xx;
    spi_setup(fpsensor->spi);
    if (error) {
        fpsensor_debug(ERR_LOG, "spi_setup failed\n");
        goto out_err;
    }
    fpsensor_spi_set_mode(fpsensor->spi, fpsensor->spi_freq_khz, 0);

out_err:
    FUNC_EXIT() ;
    return error;
}


int fpsensor_spi_send_recv(fpsensor_data_t *fpsensor, size_t len , u8 *tx_buffer, u8 *rx_buffer)
{
    struct spi_message msg;
    struct spi_transfer cmd = {
        .cs_change = 0,
        .delay_usecs = 0,
        .speed_hz = (u32)fpsensor->spi_freq_khz * 1000u,
        .tx_buf = tx_buffer,
        .rx_buf = rx_buffer,
        .len    = len,
        .tx_dma = 0,
        .rx_dma = 0,
        .bits_per_word = 0,
    };
    int error = 0 ;
    FUNC_ENTRY();
#if FPSENSOR_MANUAL_CS
    fpsensor_gpio_wirte(FPSENSOR_SPI_CS_PIN, 0);
#endif
    fpsensor_spi_set_mode(fpsensor->spi, (u32)fpsensor->spi_freq_khz, 0);
    spi_message_init(&msg);
    spi_message_add_tail(&cmd,  &msg);
    spi_sync(fpsensor->spi, &msg);
    if (error) {
        fpsensor_debug(ERR_LOG, "spi_sync failed.\n");
    }
    fpsensor_debug(DEBUG_LOG, "tx_len : %d \n", (int)len);
    fpsensor_debug(DEBUG_LOG, "tx_buf : %x  %x  %x  %x  %x  %x\n",
                   (len > 0) ? tx_buffer[0] : 0,
                   (len > 1) ? tx_buffer[1] : 0,
                   (len > 2) ? tx_buffer[2] : 0,
                   (len > 3) ? tx_buffer[3] : 0,
                   (len > 4) ? tx_buffer[4] : 0,
                   (len > 5) ? tx_buffer[5] : 0);
    fpsensor_debug(DEBUG_LOG, "rx_buf : %x  %x  %x  %x  %x  %x\n",
                   (len > 0) ? rx_buffer[0] : 0,
                   (len > 1) ? rx_buffer[1] : 0,
                   (len > 2) ? rx_buffer[2] : 0,
                   (len > 3) ? rx_buffer[3] : 0,
                   (len > 4) ? rx_buffer[4] : 0,
                   (len > 5) ? rx_buffer[5] : 0);
#if FPSENSOR_MANUAL_CS
    fpsensor_gpio_wirte(FPSENSOR_SPI_CS_PIN, 1);
#endif
    FUNC_EXIT() ;
    return error;
}

#define FPSENSOR_SPI_BLOCK_SIZE      1024
#define FPSENSOR_REG_MAX_SIZE    16
static struct spi_transfer fpsensor_xfer[100];
static u8 rx_buf0[FPSENSOR_REG_MAX_SIZE] = {0};
static u8 tx_buf0[FPSENSOR_REG_MAX_SIZE] = {0};
int fpsensor_spi_dma_fetch_image(fpsensor_data_t *fpsensor , size_t image_size_bytes)
{
    int error = 0;
    struct spi_message msg;
    unsigned char *buffer = fpsensor->huge_buffer;
    int spiBlockCount = image_size_bytes / FPSENSOR_SPI_BLOCK_SIZE;
    int spiBlocktLastNum = image_size_bytes % FPSENSOR_SPI_BLOCK_SIZE;
    int i;
#define FPSENSOR_CMD_READ_IMAGE         0x2C
    memset(rx_buf0, 0x00, FPSENSOR_REG_MAX_SIZE);
    memset(tx_buf0, 0x00, FPSENSOR_REG_MAX_SIZE);
#if FPSENSOR_MANUAL_CS
    fpsensor_gpio_wirte(FPSENSOR_SPI_CS_PIN, 0);
#endif
    fpsensor_spi_set_mode(fpsensor->spi, fpsensor->spi_freq_khz, 1);  //DMA
    spi_message_init(&msg);

    tx_buf0[0] = fpsensor->fetch_image_cmd;
    tx_buf0[1] = 0;
    fpsensor_xfer[0].tx_buf = tx_buf0;
    fpsensor_xfer[0].rx_buf = rx_buf0;
    fpsensor_xfer[0].len = fpsensor->fetch_image_cmd_len;
    fpsensor_xfer[0].cs_change = 0;
    spi_message_add_tail(&fpsensor_xfer[0], &msg);

    for (i = 0; i < spiBlockCount; i++) {
        fpsensor_xfer[i + 1].rx_buf = &buffer[i * FPSENSOR_SPI_BLOCK_SIZE];
        fpsensor_xfer[i + 1].tx_buf = &buffer[(i + spiBlockCount + 2) * FPSENSOR_SPI_BLOCK_SIZE];
        fpsensor_xfer[i + 1].len = FPSENSOR_SPI_BLOCK_SIZE;
        fpsensor_xfer[i + 1].cs_change = 0;
        spi_message_add_tail(&fpsensor_xfer[i + 1], &msg);
    }
    if (spiBlocktLastNum > 0) {
        fpsensor_xfer[i + 1].rx_buf = &buffer[i * FPSENSOR_SPI_BLOCK_SIZE];
        fpsensor_xfer[i + 1].tx_buf = &buffer[(i + spiBlockCount + 2) * FPSENSOR_SPI_BLOCK_SIZE];
        fpsensor_xfer[i + 1].len = spiBlocktLastNum;
        fpsensor_xfer[i + 1].cs_change = 0;
        spi_message_add_tail(&fpsensor_xfer[i + 1], &msg);
    }
    error =  spi_sync(fpsensor->spi, &msg);
#if FPSENSOR_MANUAL_CS
    fpsensor_gpio_wirte(FPSENSOR_SPI_CS_PIN, 1);
#endif
    if (error) {
        fpsensor_debug(ERR_LOG, "spi_sync failed.\n");
    }
    return error;
}

int fpsensor_check_HWID(fpsensor_data_t *fpsensor)
{
    unsigned int hwid = 0;
    unsigned char  tx[8] = {0};
    unsigned char  rx[8] = {0};
    int match = 0;
    tx[0] = 0x08;
    tx[1] = 0x55;
    fpsensor_spi_send_recv(fpsensor, 2, tx, rx);

    tx[0] = 0x00;
    rx[1] = 0x00;
    rx[2] = 0x00;
    fpsensor_spi_send_recv(fpsensor, 3, tx, rx);
    hwid = ((rx[1] << 8) | (rx[2]));
    fpsensor_debug(ERR_LOG,"HWID 0x%x .\n",hwid);
#if defined(CONFIG_PRIZE_HARDWARE_INFO)
	sprintf(current_fingerprint_info.id,"0x%x", hwid);
#endif
    if ((hwid == 0x7153) || (hwid == 0x7230) ||(hwid == 0x7222)||(hwid == 0x7332)) {
        match = 1;
    }
    return match;
}

int fpsensor_manage_image_buffer(fpsensor_data_t *fpsensor , size_t new_size)
{
    int error = 0;
    int buffer_order_new, buffer_order_curr;

    buffer_order_curr = get_order(fpsensor->huge_buffer_size);
    buffer_order_new  = get_order(new_size);

    if (new_size == 0) {
        if (fpsensor->huge_buffer) {
            free_pages((unsigned long)fpsensor->huge_buffer,
                       buffer_order_curr);

            fpsensor->huge_buffer = NULL;
        }
        fpsensor->huge_buffer_size = 0;
        error = 0;

    } else {
        if (fpsensor->huge_buffer &&
            (buffer_order_curr != buffer_order_new)) {

            free_pages((unsigned long)fpsensor->huge_buffer,
                       buffer_order_curr);

            fpsensor->huge_buffer = NULL;
        }

        if (fpsensor->huge_buffer == NULL) {
            fpsensor->huge_buffer =
                (u8 *)__get_free_pages(GFP_KERNEL,
                                       buffer_order_new);

            fpsensor->huge_buffer_size = (fpsensor->huge_buffer) ?
                                         (size_t)PAGE_SIZE << buffer_order_new : 0;

            error = (fpsensor->huge_buffer_size == 0) ? -ENOMEM : 0;
        }
    }


    if (error) {
        fpsensor_debug(ERR_LOG, "%s, failed %d\n",
                       __func__, error);
    } else {
        fpsensor_debug(DEBUG_LOG, "%s, size=%d bytes\n",
                       __func__, (int)fpsensor->huge_buffer_size);
    }

    return error;
}







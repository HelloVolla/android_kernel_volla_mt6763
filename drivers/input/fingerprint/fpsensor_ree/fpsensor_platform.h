//#include "mt_spi.h"
//#include "mt_spi_hal.h"
//#include "mt_gpio.h"
//#include "mach/gpio_const.h"
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/spi/spidev.h>
#include <linux/spi/spi.h>
#include "fpsensor_spi.h"

extern struct mt_chip_conf fpsensor_spi_conf_mt65xx;

extern int fpsensor_manage_image_buffer(fpsensor_data_t *fpsensor , size_t new_size);
extern int fpsensor_spi_send_recv(fpsensor_data_t *fpsensor, size_t len , u8 *tx_buffer, u8 *rx_buffer);
extern int fpsensor_spi_setup(fpsensor_data_t *fpsensor);
extern int fpsensor_spi_dma_fetch_image(fpsensor_data_t *fpsensor , size_t image_size_bytes);
extern void fpsensor_spi_pins_config(void);

extern int fpsensor_check_HWID(fpsensor_data_t *fpensor);
extern int fpsensor_gpio_wirte(int gpio, int value);
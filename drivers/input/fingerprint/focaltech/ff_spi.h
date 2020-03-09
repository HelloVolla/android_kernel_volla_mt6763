/**
 * ${ANDROID_BUILD_TOP}/vendor/focaltech/src/base/focaltech/ff_spi.h
 *
 * Copyright (C) 2014-2017 FocalTech Systems Co., Ltd. All Rights Reserved.
 *
**/

#ifndef __FF_SPI_H__
#define __FF_SPI_H__

enum spi_sample_sel {
    POSEDGE,
    NEGEDGE
};
enum spi_cs_pol {
    ACTIVE_LOW,
    ACTIVE_HIGH
};

enum spi_cpol {
    SPI_CPOL_0,
    SPI_CPOL_1
};

enum spi_cpha {
    SPI_CPHA_0,
    SPI_CPHA_1
};

enum spi_mlsb {
    SPI_LSB,
    SPI_MSB
};

enum spi_endian {
    SPI_LENDIAN,
    SPI_BENDIAN
};

enum spi_transfer_mode {
    FIFO_TRANSFER,
    DMA_TRANSFER,
    OTHER1,
    OTHER2,
};

enum spi_pause_mode {
    PAUSE_MODE_DISABLE,
    PAUSE_MODE_ENABLE
};
enum spi_finish_intr {
    FINISH_INTR_DIS,
    FINISH_INTR_EN,
};

enum spi_deassert_mode {
    DEASSERT_DISABLE,
    DEASSERT_ENABLE
};

enum spi_ulthigh {
    ULTRA_HIGH_DISABLE,
    ULTRA_HIGH_ENABLE
};

enum spi_tckdly {
    TICK_DLY0,
    TICK_DLY1,
    TICK_DLY2,
    TICK_DLY3
};

struct mt_chip_conf {
    u32 setuptime;
    u32 holdtime;
    u32 high_time;
    u32 low_time;
    u32 cs_idletime;
    u32 ulthgh_thrsh;
    enum spi_sample_sel sample_sel;
    enum spi_cs_pol cs_pol;
    enum spi_cpol cpol;
    enum spi_cpha cpha;
    enum spi_mlsb tx_mlsb;
    enum spi_mlsb rx_mlsb;
    enum spi_endian tx_endian;
    enum spi_endian rx_endian;
    enum spi_transfer_mode com_mod;
    enum spi_pause_mode pause;
    enum spi_finish_intr finish_intr;
    enum spi_deassert_mode deassert;
    enum spi_ulthigh ulthigh;
    enum spi_tckdly tckdly;
};

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Init ff_spi component.
 *
 * @return
 *  ff_err_t code.
 */
int ff_spi_init(void);

/*
 * De-init ff_spi component.
 */
int ff_spi_free(void);

/*
 * Configurate the SPI speed.
 *
 * @params
 *  bps: bits per second.
 *
 * @return
 *  ff_err_t code.
 */
int ff_spi_config_speed(int bps);

/*
 * Writes data to SPI bus.
 *
 * @params
 *  tx_buf: TX data buffer to be written.
 *  tx_len: TX data length.
 *
 * @return
 *  ff_err_t code.
 */
int ff_spi_write_buf(const void *tx_buf, int tx_len);

/*
 * Writes command data to SPI bus and then reads data from SPI bus.
 *
 * @params
 *  tx_buf: TX data buffer to be written.
 *  tx_len: TX data length.
 *  rx_buf: RX data buffer to be read.
 *  rx_len: RX data length.
 *
 * @return
 *  ff_err_t code.
 */
int ff_spi_write_then_read_buf(const void *tx_buf, int tx_len, void *rx_buf, int rx_len);

#ifdef __cplusplus
}
#endif

#endif /* __FF_SPI_H__ */

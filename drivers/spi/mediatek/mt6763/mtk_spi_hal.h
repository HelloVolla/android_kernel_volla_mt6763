/*
 *  Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef __MTK_SPI_HAL_H__
#define __MTK_SPI_HAL_H__

/*****************************************************************************
 * define macro for spi register
 ******************************************************************************/
#define SPI_CFG0_REG (0x0000)
#define SPI_CFG1_REG (0x0004)
#define SPI_TX_SRC_REG (0x0008)
#define SPI_RX_DST_REG (0x000c)
#define SPI_TX_DATA_REG (0x0010)
#define SPI_RX_DATA_REG (0x0014)
#define SPI_CMD_REG (0x0018)
#define SPI_STATUS0_REG (0x001c)
#define SPI_STATUS1_REG (0x0020)
#define SPI_PAD_SEL_REG (0x0024)
#define SPI_CFG2_REG (0x0028)
#define SPI_TX_SRC_REG_64 (0x002C)
#define SPI_RX_DST_REG_64 (0x0030)

#define MIPI_TX_PU_TDP1_OFFSET 4
#define MIPI_TX_PU_TDP1_MASK 0x10
#define MIPI_TX_PD_TDP1_OFFSET 12
#define MIPI_TX_PD_TDP1_MASK 0x1000

#define SPI_CFG0_SCK_HIGH_OFFSET 0
#define SPI_CFG0_SCK_LOW_OFFSET 16
#define SPI_CFG0_CS_HOLD_OFFSET 0
#define SPI_CFG0_CS_SETUP_OFFSET 16

#define SPI_CFG0_SCK_HIGH_MASK 0xffff
#define SPI_CFG0_SCK_LOW_MASK 0xffff0000
#define SPI_CFG0_CS_HOLD_MASK 0xffff
#define SPI_CFG0_CS_SETUP_MASK 0xffff0000

#define SPI_CFG1_CS_IDLE_OFFSET 0
#define SPI_CFG1_PACKET_LOOP_OFFSET 8
#define SPI_CFG1_PACKET_LENGTH_OFFSET 16
#define SPI_CFG1_GET_TICK_DLY_OFFSET 29

#define SPI_CFG1_CS_IDLE_MASK 0xff
#define SPI_CFG1_PACKET_LOOP_MASK 0xff00
#define SPI_CFG1_PACKET_LENGTH_MASK 0x3ff0000
#define SPI_CFG1_GET_TICK_DLY_MASK 0xe0000000

#define SPI_CMD_ACT_OFFSET 0
#define SPI_CMD_RESUME_OFFSET 1
#define SPI_CMD_RST_OFFSET 2
#define SPI_CMD_PAUSE_EN_OFFSET 4
#define SPI_CMD_DEASSERT_OFFSET 5
#define SPI_CMD_SAMPLE_SEL_OFFSET 6
#define SPI_CMD_CS_POL_OFFSET 7
#define SPI_CMD_CPHA_OFFSET 8
#define SPI_CMD_CPOL_OFFSET 9
#define SPI_CMD_RX_DMA_OFFSET 10
#define SPI_CMD_TX_DMA_OFFSET 11
#define SPI_CMD_TXMSBF_OFFSET 12
#define SPI_CMD_RXMSBF_OFFSET 13
#define SPI_CMD_RX_ENDIAN_OFFSET 14
#define SPI_CMD_TX_ENDIAN_OFFSET 15
#define SPI_CMD_FINISH_IE_OFFSET 16
#define SPI_CMD_PAUSE_IE_OFFSET 17

#define SPI_CMD_ACT_MASK 0x1
#define SPI_CMD_RESUME_MASK 0x2
#define SPI_CMD_RST_MASK 0x4
#define SPI_CMD_PAUSE_EN_MASK 0x10
#define SPI_CMD_DEASSERT_MASK 0x20
#define SPI_CMD_SAMPLE_SEL_MASK 0x40
#define SPI_CMD_CS_POL_MASK 0x80
#define SPI_CMD_CPHA_MASK 0x100
#define SPI_CMD_CPOL_MASK 0x200
#define SPI_CMD_RX_DMA_MASK 0x400
#define SPI_CMD_TX_DMA_MASK 0x800
#define SPI_CMD_TXMSBF_MASK 0x1000
#define SPI_CMD_RXMSBF_MASK 0x2000
#define SPI_CMD_RX_ENDIAN_MASK 0x4000
#define SPI_CMD_TX_ENDIAN_MASK 0x8000
#define SPI_CMD_FINISH_IE_MASK 0x10000
#define SPI_CMD_PAUSE_IE_MASK 0x20000

#define SPI_ULTRA_HIGH_EN_OFFSET 0
#define SPI_ULTRA_HIGH_THRESH_OFFSET 16

#define SPI_ULTRA_HIGH_EN_MASK 0x1
#define SPI_ULTRA_HIGH_THRESH_MASK 0xffff0000

#include <sync_write.h>
#define spi_readl(port, offset) __raw_readl((port)->regs + (offset))
#if 0
#define spi_writel(port, offset, value)                                       \
	__raw_writel((value), (port)->regs + (offset))
#endif

#define spi_writel(port, offset, value)                                       \
	mt_reg_sync_writel((value), (port)->regs + (offset))

extern void mt_spi_enable_master_clk(struct spi_device *ms);
extern void mt_spi_disable_master_clk(struct spi_device *ms);

#endif

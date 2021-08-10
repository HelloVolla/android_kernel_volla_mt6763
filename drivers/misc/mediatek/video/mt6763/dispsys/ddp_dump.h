/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _DDP_DUMP_H_
#define _DDP_DUMP_H_

#include "ddp_info.h"
#include "ddp_path.h"

enum DISP_ENGINE_SIGNAL0 {
	DDP_SIGNAL_DPI0_SEL__DPI0 = 31,
	DDP_SIGNAL_DIS0_SEL__DSI0 = 30,
	DDP_SIGNAL_RDMA1_SOUT1__DPI0_SIN2 = 29,
	DDP_SIGNAL_RDMA1_SOUT0__DSI0_SIN2 = 28,
	DDP_SIGNAL_RDMA1__RDMA1_SOUT = 27,
	DDP_SIGNAL_OVL1_MOUT2__OVL0 = 26,
	DDP_SIGNAL_OVL1_MOUT1__WDMA1 = 25,
	DDP_SIGNAL_OVL1_MOUT0__RDMA1 = 24,
	DDP_SIGNAL_OVL1__OVL1_MOUT = 23,
	DDP_SIGNAL_WDMA0_SEL__WDMA0 = 22,
	DDP_SIGNAL_UFOE_MOUT2__WDMA0_SIN2 = 21,
	DDP_SIGNAL_UFOE_MOUT1__DPI0_SIN0 = 20,
	DDP_SIGNAL_UFOE_MOUT0__DSI0_SIN0 = 19,
	DDP_SIGNAL_UFOE__UFOE_MOUT = 18,
	DDP_SIGNAL_UFOE_SEL__UFOE = 17,
	DDP_SIGNAL_RDMA0_SOUT3__DPI0_SIN1 = 16,
	DDP_SIGNAL_RDMA0_SOUT2__DSI0_SIN1 = 15,
	DDP_SIGNAL_RDMA0_SOUT1__COLOR_SIN0 = 14,
	DDP_SIGNAL_RDMA0_SOUT0__UFOE_SIN0 = 13,
	DDP_SIGNAL_RDMA0__RDMA0_SOUT = 12,
	DDP_SIGNAL_DITHER_MOUT2__WDMA0_SIN1 = 11,
	DDP_SIGNAL_DITHER_MOUT1__UFOE_SIN1 = 10,
	DDP_SIGNAL_DITHER_MOUT0__RDMA0 = 9,
	DDP_SIGNAL_DITHER__DITHER_MOUT = 8,
	DDP_SIGNAL_GAMMA__DITHER = 7,
	DDP_SIGNAL_AAL__GAMMA = 6,
	DDP_SIGNAL_CCORR__AAL = 5,
	DDP_SIGNAL_COLOR__CCORR = 4,
	DDP_SIGNAL_COLOR_SEL__COLOR = 3,
	DDP_SIGNAL_OVL0_MOUT1__WDMA0_SIN0 = 2,
	DDP_SIGNAL_OVL0_MOUT0__COLOR_SIN1 = 1,
	DDP_SIGNAL_OVL0__OVL0_MOUT = 0,
};

char *ddp_get_fmt_name(enum DISP_MODULE_ENUM module, unsigned int fmt);
int ddp_dump_analysis(enum DISP_MODULE_ENUM module);
int ddp_dump_reg(enum DISP_MODULE_ENUM module);
#endif

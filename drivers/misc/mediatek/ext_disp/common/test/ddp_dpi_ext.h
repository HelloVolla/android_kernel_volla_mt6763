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

#ifndef __DDP_DPI_EXT_H__
#define __DDP_DPI_EXT_H__


#if defined(RDMA_DPI_PATH_SUPPORT) || defined(DPI_DVT_TEST_SUPPORT)

#include "lcm_drv.h"
#include "ddp_info.h"
#include "cmdq_record.h"
/*#include "dpi_dvt_test.h"*/
#include "ddp_dpi.h"

#ifdef __cplusplus
extern "C" {
#endif


enum DPI_EXT_STATUS {
	DPI_EXT_STATUS_OK = 0,
	DPI_EXT_STATUS_ERROR,
};


/*************************for DPI DVT***************************/

enum DPI_EXT_STATUS DPI_EnableColorBar(unsigned int pattern);
enum DPI_EXT_STATUS DPI_DisableColorBar(void);
enum DPI_EXT_STATUS ddp_dpi_EnableColorBar_0(void);
enum DPI_EXT_STATUS ddp_dpi_EnableColorBar_16(void);

int configInterlaceMode(unsigned int resolution);
int config3DMode(unsigned int resolution);
int config3DInterlaceMode(unsigned int resolution);
unsigned int readDPIStatus(void);
unsigned int readDPITDLRStatus(void);
unsigned int clearDPIStatus(void);
unsigned int clearDPIIntrStatus(void);
unsigned int readDPIIntrStatus(void);
unsigned int ClearDPIIntrStatus(void);
unsigned int enableRGB2YUV(enum AviColorSpace_e format);
unsigned int enableSingleEdge(void);
int enableAndGetChecksum(void);
unsigned int configDpiRepetition(void);
unsigned int configDpiEmbsync(void);
/****************************************/

#ifdef __cplusplus
}
#endif
#endif
#endif /* __DPI_DRV_H__ */

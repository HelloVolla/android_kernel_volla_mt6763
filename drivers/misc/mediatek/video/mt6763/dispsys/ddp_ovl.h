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

#ifndef _DDP_OVL_H_
#define _DDP_OVL_H_

#include "ddp_hal.h"
#include "ddp_info.h"

#define OVL_MAX_WIDTH (4095)
#define OVL_MAX_HEIGHT (4095)

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#define TOTAL_OVL_LAYER_NUM (4 + 3 + 1 + 2)
#else
#define TOTAL_OVL_LAYER_NUM (4 + 3 + 2 + 3)
#endif
#define TOTAL_REAL_OVL_LAYER_NUM (4 + 3 + 2 + 3)
#define OVL_NUM (3)
#define PRIMARY_THREE_OVL_CASCADE

/* start overlay module */
int ovl_start(enum DISP_MODULE_ENUM module, void *handle);

/* stop overlay module */
int ovl_stop(enum DISP_MODULE_ENUM module, void *handle);

/* reset overlay module */
int ovl_reset(enum DISP_MODULE_ENUM module, void *handle);

/* set region of interest */
int ovl_roi(enum DISP_MODULE_ENUM module, unsigned int bgW,
	    unsigned int bgH, /* region size */
	    unsigned int bgColor, /* border color */ void *handle);

/* switch layer on/off */
int ovl_layer_switch(enum DISP_MODULE_ENUM module, unsigned int layer,
	unsigned int en, void *handle);
/* get ovl input address */
void ovl_get_address(enum DISP_MODULE_ENUM module, unsigned long *add);

int ovl_3d_config(enum DISP_MODULE_ENUM module, unsigned int layer_id,
		  unsigned int en_3d, unsigned int landscape,
		  unsigned int r_first, void *handle);

void ovl_dump_analysis(enum DISP_MODULE_ENUM module);
void ovl_dump_reg(enum DISP_MODULE_ENUM module);
unsigned long ovl_base_addr(enum DISP_MODULE_ENUM module);
unsigned long ovl_to_index(enum DISP_MODULE_ENUM module);

void ovl_get_info(enum DISP_MODULE_ENUM module, void *data);
int _ovl_get_info(enum DISP_MODULE_ENUM module, void *data, int *layer_idx);
void __ovl_get_info(enum DISP_MODULE_ENUM module, void *p, int ext_en,
		    int ext_layer, int layer);
unsigned int ddp_ovl_get_cur_addr(bool rdma_mode, int layerid);
enum DISP_MODULE_ENUM _ovl_index_to_mod(int index);
unsigned long _ovl_layer_num(enum DISP_MODULE_ENUM module);

#endif

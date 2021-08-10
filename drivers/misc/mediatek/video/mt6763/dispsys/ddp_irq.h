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

#ifndef _H_DDP_IRQ_H
#define _H_DDP_IRQ_H

#include "ddp_info.h"
#include <linux/interrupt.h>

typedef void (*DDP_IRQ_CALLBACK)(enum DISP_MODULE_ENUM module,
				 unsigned int reg_value);

int disp_register_module_irq_callback(enum DISP_MODULE_ENUM module,
				      DDP_IRQ_CALLBACK cb);
int disp_unregister_module_irq_callback(enum DISP_MODULE_ENUM module,
					DDP_IRQ_CALLBACK cb);

int disp_register_irq_callback(DDP_IRQ_CALLBACK cb);
int disp_unregister_irq_callback(DDP_IRQ_CALLBACK cb);

void disp_register_irq(unsigned int irq_num, char *device_name);
int disp_init_irq(void);
irqreturn_t disp_irq_handler(int irq, void *dev_id);

extern atomic_t ESDCheck_byCPU;

int disp_irq_esd_cust_get(void);
void disp_irq_esd_cust_bycmdq(int enable);
#endif

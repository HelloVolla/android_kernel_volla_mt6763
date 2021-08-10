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

#ifndef __CMDQ_DEVICE_H__
#define __CMDQ_DEVICE_H__

#include <linux/platform_device.h>
#include <linux/device.h>
#include "cmdq_def.h"
#include "cmdq_mdp_common.h"
#include <linux/clk.h>

#define DECLARE_ENABLE_HW_CLOCK(HW_NAME) \
	u32 cmdq_dev_enable_clock_##HW_NAME(bool enable)
DECLARE_ENABLE_HW_CLOCK(SMI_COMMON);
DECLARE_ENABLE_HW_CLOCK(SMI_LARB0);
#ifdef CMDQ_USE_LEGACY
DECLARE_ENABLE_HW_CLOCK(MUTEX_32K);
#endif
#undef DECLARE_ENABLE_HW_CLOCK

void cmdq_dev_get_module_clock_by_name(const char *ref_name,
	const char *clkName, struct clk **clk_module);
u32 cmdq_dev_enable_device_clock(bool enable, struct clk *clk_module,
	const char *clkName);
bool cmdq_dev_device_clock_is_enable(struct clk *clk_module);
/* For test case used */
void testcase_clkmgr_impl(enum CMDQ_ENG_ENUM engine,
	char *name, const unsigned long testWriteReg,
	const u32 testWriteValue,
	const unsigned long testReadReg, const bool verifyWriteResult);

struct device *cmdq_dev_get(void);
/* interrupt index */
u32 cmdq_dev_get_irq_id(void);
u32 cmdq_dev_get_irq_secure_id(void);
/* GCE clock */
void cmdq_dev_enable_gce_clock(bool enable);
bool cmdq_dev_gce_clock_is_enable(void);
/* virtual address */
long cmdq_dev_get_module_base_VA_GCE(void);
unsigned long cmdq_dev_alloc_reference_VA_by_name(const char *ref_name);
/* Other modules information */
void cmdq_dev_free_module_base_VA(const long VA);
u32 cmdq_dev_get_mmsys_dummy_reg_offset(void);
/* physical address */
phys_addr_t cmdq_dev_get_reference_PA(const char *ref_name, int index);
phys_addr_t cmdq_dev_get_module_base_PA_GCE(void);
/* GCE event */
void cmdq_dev_init_event_table(struct device_node *node);
void cmdq_dev_test_dts_correctness(void);
/* device initialization / deinitialization */
void cmdq_dev_init(struct platform_device *pDevice);
void cmdq_dev_deinit(void);
/* dma_set_mask result, to show in status */
s32 cmdq_dev_get_dma_mask_result(void);
u32 cmdq_dev_get_thread_count(void);

struct cmdq_dts_setting {
	u32 prefetch_thread_count;
	u32 *prefetch_size;
	u32 ctl_int0;
	u32 cpr_size;
};

/* callback when read resource from device tree */
typedef void(*CMDQ_DEV_INIT_RESOURCE_CB) (u32 engineFlag,
	enum cmdq_event resourceEvent);

void cmdq_dev_get_dts_setting(struct cmdq_dts_setting *dts_setting);
void cmdq_dev_init_resource(CMDQ_DEV_INIT_RESOURCE_CB init_cb);





#endif				/* __CMDQ_DEVICE_H__ */

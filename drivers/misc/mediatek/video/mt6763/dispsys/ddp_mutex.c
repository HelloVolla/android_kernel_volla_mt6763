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

#include "ddp_reg.h"
#include "ddp_mutex.h"
#include "ddp_log.h"
#include "ddp_info.h"
#include "disp_helper.h"

static struct module_map_t module_mutex_map[DISP_MODULE_NUM] = {
	{DISP_MODULE_OVL0, 9, 0},
	{DISP_MODULE_OVL1, -1, 0},
	{DISP_MODULE_OVL0_2L, 10, 0},
	{DISP_MODULE_OVL1_2L, 11, 0},
	{DISP_MODULE_OVL0_VIRTUAL, -1, 0},
	{DISP_MODULE_OVL0_2L_VIRTUAL, -1, 0},
	{DISP_MODULE_OVL1_2L_VIRTUAL, -1, 0},

	{DISP_MODULE_RDMA0, 0, 0},
	{DISP_MODULE_RDMA1, 1, 0},
	{DISP_MODULE_RDMA2, -1, 0},

	{DISP_MODULE_WDMA0, 12, 0},
	{DISP_MODULE_WDMA0, -1, 0},
	{DISP_MODULE_WDMA_VIRTUAL0, -1, 0},
	{DISP_MODULE_WDMA_VIRTUAL1, -1, 0},

	{DISP_MODULE_COLOR0, 13, 0},
	{DISP_MODULE_COLOR1, -1, 0},
	{DISP_MODULE_CCORR0, 14, 0},
	{DISP_MODULE_CCORR1, -1, 0},
	{DISP_MODULE_AAL0, 15, 0},
	{DISP_MODULE_AAL1, -1, 0},
	{DISP_MODULE_GAMMA0, 16, 0},
	{DISP_MODULE_GAMMA1, -1, 0},
	{DISP_MODULE_OD, -1, 0},
	{DISP_MODULE_DITHER0, 17, 0},
	{DISP_MODULE_DITHER1, -1, 0},

	{DISP_MODULE_PATH0, -1, 0},
	{DISP_MODULE_PATH1, -1, 0},

	{DISP_MODULE_UFOE, -1, 0},
	{DISP_MODULE_DSC, -1, 0},
	{DISP_MODULE_DSC_2ND, -1, 0},
	{DISP_MODULE_SPLIT0, -1, 0},

	{DISP_MODULE_DPI, 20, 0},

	{DISP_MODULE_DSI0, 19, 0},
	{DISP_MODULE_DSI1, -1, 0},
	{DISP_MODULE_DSIDUAL, -1, 0},

	{DISP_MODULE_PWM0, 18, 0},
	{DISP_MODULE_PWM1, -1, 0},

	{DISP_MODULE_CONFIG, -1, 0},
	{DISP_MODULE_MUTEX, -1, 0},
	{DISP_MODULE_SMI_COMMON, -1, 0},
	{DISP_MODULE_SMI_LARB0, -1, 0},
	{DISP_MODULE_SMI_LARB1, -1, 0},
	{DISP_MODULE_MIPI0, -1, 0},
	{DISP_MODULE_MIPI1, -1, 0},

	{DISP_MODULE_RSZ0, -1, 0},
	{DISP_MODULE_RSZ1, -1, 0},

	{DISP_MODULE_UNKNOWN, -1, 0},
};

static int ddp_get_mutex_src(enum DISP_MODULE_ENUM dest_module,
			     enum DDP_MODE ddp_mode, unsigned int *SOF_src,
			     unsigned int *EOF_src)
{
	unsigned int src_from_dst_module = 0;

	if (dest_module == DISP_MODULE_WDMA0) {
		if (ddp_mode == DDP_VIDEO_MODE)
			DISP_LOG_W("%s: dst_mode=%s, but is video mode !!\n",
				   __func__, ddp_get_module_name(dest_module));

		*SOF_src = SOF_VAL_MUTEX0_SOF_SINGLE_MODE;
		*EOF_src = SOF_VAL_MUTEX0_EOF_DISABLE;
		return 0;
	}

	if (dest_module == DISP_MODULE_DSI0 ||
	    dest_module == DISP_MODULE_DSIDUAL) {
		src_from_dst_module = SOF_VAL_MUTEX0_SOF_FROM_DSI0;
	} else if (dest_module == DISP_MODULE_DSI1) {
		src_from_dst_module = SOF_VAL_MUTEX0_SOF_FROM_DSI1;
	} else if (dest_module == DISP_MODULE_DPI) {
		src_from_dst_module = SOF_VAL_MUTEX0_SOF_FROM_DPI;
	} else {
		DDPERR("%s, invalid param dst module = %s(%d), dsi mode %s\n",
			   __func__,
		       ddp_get_module_name(dest_module), dest_module,
		       ddp_get_mode_name(ddp_mode));
		WARN_ON(1);
	}

	if (ddp_mode == DDP_CMD_MODE) {
		*SOF_src = SOF_VAL_MUTEX0_SOF_SINGLE_MODE;

		if (disp_helper_get_option(DISP_OPT_MUTEX_EOF_EN_FOR_CMD_MODE))
			*EOF_src = src_from_dst_module;
		else
			*EOF_src = SOF_VAL_MUTEX0_EOF_DISABLE;

	} else {
		*SOF_src = *EOF_src = src_from_dst_module;
	}

	return 0;
}

static int ddp_mutex_add_module(int mutex_id, enum DISP_MODULE_ENUM module,
				void *handle)
{
	int value = 0;

	if (is_ddp_module(module)) {
		if (module_mutex_map[module].bit != -1) {
			DDPDBG("module %s added to mutex %d\n",
			       ddp_get_module_name(module), mutex_id);
			value = (1 << module_mutex_map[module].bit);
			if (module_mutex_map[module].mod_num == 0)
				DISP_REG_MASK(
				    handle,
				    DISP_REG_CONFIG_MUTEX_MOD0(mutex_id),
				    value, value);
		} else if (module == DISP_MODULE_DSIDUAL) {
			DDPDBG("module %s added to mutex %d\n",
			       ddp_get_module_name(module), mutex_id);
			value = (1 << module_mutex_map[DISP_MODULE_DSI0].bit);
			value |= (1 << module_mutex_map[DISP_MODULE_DSI1].bit);
			if (module_mutex_map[module].mod_num == 0)
				DISP_REG_MASK(
				    handle,
				    DISP_REG_CONFIG_MUTEX_MOD0(mutex_id),
				    value, value);
		}
	}

	return value;
}

int ddp_mutex_remove_module(int mutex_id, enum DISP_MODULE_ENUM module,
			    void *handle)
{
	int ret = 0;
	int value = 0;

	if (is_ddp_module(module)) {
		if (module_mutex_map[module].bit != -1) {
			DDPDBG("module %s added to mutex %d\n",
			       ddp_get_module_name(module), mutex_id);
			value = (1 << module_mutex_map[module].bit);
			if (module_mutex_map[module].mod_num == 0) {
				DISP_REG_MASK(
				    handle,
				    DISP_REG_CONFIG_MUTEX_MOD0(mutex_id), 0,
				    value);
				ret = 1;
			}
		} else if (module == DISP_MODULE_DSIDUAL) {
			value = (1 << module_mutex_map[DISP_MODULE_DSI0].bit);
			value |= (1 << module_mutex_map[DISP_MODULE_DSI1].bit);
			if (module_mutex_map[module].mod_num == 0) {
				DISP_REG_MASK(
				    handle,
				    DISP_REG_CONFIG_MUTEX_MOD0(mutex_id), 0,
				    value);
				ret = 1;
			}
		}
	}
	return ret;
}

/* id: mutex ID, 0~3 */
static int ddp_mutex_set_l(int mutex_id, int *module_list,
			   enum DDP_MODE ddp_mode, void *handle)
{
	int i = 0;
	unsigned int value = 0;
	unsigned int sof_val;
	unsigned int sof_src = 0, eof_src = 0;
	int module_num = ddp_get_module_num_l(module_list);

	if (mutex_id < DISP_MUTEX_DDP_FIRST ||
	    mutex_id > DISP_MUTEX_DDP_LAST) {
		DDPERR("exceed mutex max (0 ~ %d)\n", DISP_MUTEX_DDP_LAST);
		return -1;
	}

	/* important */
	DISP_REG_SET(handle, DISP_REG_CONFIG_MUTEX_MOD0(mutex_id), 0);

	for (i = 0; i < module_num; i++)
		value |=
		    ddp_mutex_add_module(mutex_id, module_list[i], handle);

	ddp_get_mutex_src(module_list[module_num - 1], ddp_mode, &sof_src,
			  &eof_src);
	sof_val = REG_FLD_VAL(SOF_FLD_MUTEX0_SOF, sof_src);
	sof_val |= REG_FLD_VAL(SOF_FLD_MUTEX0_EOF, eof_src);
	DISP_REG_SET(handle, DISP_REG_CONFIG_MUTEX_SOF(mutex_id),
		     sof_val); /* use default timing */

	DDPDBG("mutex %d value=0x%x, sof=%s, eof=%s\n", mutex_id, value,
	       ddp_get_mutex_sof_name(sof_src),
	       ddp_get_mutex_sof_name(eof_src));
	return 0;
}

static void ddp_check_mutex_l(int mutex_id, int *module_list,
			      enum DDP_MODE ddp_mode)
{
	int i = 0;
	uint32_t real_value0 = 0;
	uint32_t expect_value0 = 0;
	unsigned int real_sof, real_eof, val;
	unsigned int expect_sof = 0, expect_eof = 0;
	int module_num = ddp_get_module_num_l(module_list);

	if (mutex_id < DISP_MUTEX_DDP_FIRST ||
	    mutex_id > DISP_MUTEX_DDP_LAST) {
		DDPDUMP("error:check mutex fail:exceed mutex max (0 ~ %d)\n",
			DISP_MUTEX_DDP_LAST);
		return;
	}
	/* check mod */
	real_value0 = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_MOD0(mutex_id));
	for (i = 0; i < module_num; i++) {
		if (module_mutex_map[module_list[i]].bit != -1) {
			if (module_mutex_map[module_list[i]].mod_num == 0)
				expect_value0 |=
				    (1
				     << module_mutex_map[module_list[i]].bit);
		} else if (module_list[i] == DISP_MODULE_DSIDUAL) {
			if (module_mutex_map[module_list[i]].mod_num == 0) {
				expect_value0 |=
				    (1 << module_mutex_map[DISP_MODULE_DSI0]
					      .bit);
				expect_value0 |=
				    (1 << module_mutex_map[DISP_MODULE_DSI1]
					      .bit);
			}
		}
	}
	if (expect_value0 != real_value0)
		DDPDUMP("error:mutex %d error: expect0 0x%x, real0 0x%x\n",
			mutex_id, expect_value0, real_value0);

	/* check sof */
	val = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_SOF(mutex_id));
	real_sof = REG_FLD_VAL_GET(SOF_FLD_MUTEX0_SOF, val);
	real_eof = REG_FLD_VAL_GET(SOF_FLD_MUTEX0_EOF, val);
	ddp_get_mutex_src(module_list[module_num - 1], ddp_mode, &expect_sof,
			  &expect_eof);
	if (expect_sof != real_sof)
		DDPDUMP("error:mutex %d sof error: expect %s, real %s\n",
			mutex_id, ddp_get_mutex_sof_name(expect_sof),
			ddp_get_mutex_sof_name(real_sof));
	if (expect_eof != real_eof)
		DDPDUMP("error:mutex %d eof error: expect %s, real %s\n",
			mutex_id, ddp_get_mutex_sof_name(expect_eof),
			ddp_get_mutex_sof_name(real_eof));
}


void ddp_mutex_interrupt_enable(int mutex_id, void *handle)
{
#if 0
	DDPDBG("mutex %d interrupt enable\n", mutex_id);
	DISP_REG_MASK(handle, DISP_REG_CONFIG_MUTEX_INTEN,
		0x1 << mutex_id, 0x1 << mutex_id); /* sof irq */
	DISP_REG_MASK(handle, DISP_REG_CONFIG_MUTEX_INTEN,
		0x1 << (mutex_id + DISP_MUTEX_TOTAL),
		0x1 << (mutex_id + DISP_MUTEX_TOTAL)); /* stream done irq */

	DISP_REG_MASK(handle, DISP_REG_CONFIG_MUTEX_INTEN_1,
		0x1 << mutex_id, 0x1 << mutex_id); /* regs update irq */
	DISP_REG_MASK(handle, DISP_REG_CONFIG_MUTEX_INTEN_1,
		0x1 << (mutex_id + DISP_MUTEX_TOTAL),
		0x1 << (mutex_id + DISP_MUTEX_TOTAL)); /* update timeout irq */
#endif
}

void ddp_mutex_interrupt_disable(int mutex_id, void *handle)
{
	DDPDBG("mutex %d interrupt disenable\n", mutex_id);
	DISP_REG_MASK(handle, DISP_REG_CONFIG_MUTEX_INTEN, 0,
		      0x1 << mutex_id); /* sof irq */
	DISP_REG_MASK(handle, DISP_REG_CONFIG_MUTEX_INTEN, 0,
		      0x1 << (mutex_id + DISP_MUTEX_TOTAL));

	DISP_REG_MASK(handle, DISP_REG_CONFIG_MUTEX_INTEN_1, 0,
		      0x1 << mutex_id); /* regs update irq */
	DISP_REG_MASK(handle, DISP_REG_CONFIG_MUTEX_INTEN_1, 0,
		      0x1 << (mutex_id + DISP_MUTEX_TOTAL));
}

void ddp_mutex_reset(int mutex_id, void *handle)
{
	DDPDBG("mutex %d reset\n", mutex_id);
	DISP_REG_SET(handle, DISP_REG_CONFIG_MUTEX_RST(mutex_id), 1);
	DISP_REG_SET(handle, DISP_REG_CONFIG_MUTEX_RST(mutex_id), 0);
	/* DCM will be enabled after reset, so disable it */
	DISP_REG_SET(handle, DISP_REG_CONFIG_MUTEX_CFG, 0);
}

int ddp_is_moudule_in_mutex(int mutex_id, enum DISP_MODULE_ENUM module)
{
	int ret = 0;
	uint32_t real_value = 0;

	if (mutex_id < DISP_MUTEX_DDP_FIRST ||
		mutex_id > DISP_MUTEX_DDP_LAST) {
		DDPDUMP("err:check_module_in_mute fail:exceed max (0~%d)\n",
			DISP_MUTEX_DDP_LAST);
		return ret;
	}

	if (module_mutex_map[module].mod_num == 0)
		real_value =
		    DISP_REG_GET(DISP_REG_CONFIG_MUTEX_MOD0(mutex_id));

	if (module == DISP_MODULE_DSIDUAL) {
		if (1 == ((real_value >>
			   module_mutex_map[DISP_MODULE_DSI0].bit) &
			  0x01) &&
		    1 == ((real_value >>
			   module_mutex_map[DISP_MODULE_DSI1].bit) &
			  0x01))
			ret = 1;
	} else {
		if (1 == ((real_value >> module_mutex_map[module].bit) & 0x01))
			ret = 1;
	}

	return ret;
}

void ddp_mutex_clear(int mutex_id, void *handle)
{
	DDPDBG("mutex %d clear\n", mutex_id);
	/*reset mutex */
	ddp_mutex_reset(mutex_id, handle); /* disable mutex */
	DISP_REG_SET(handle, DISP_REG_CONFIG_MUTEX_MOD0(mutex_id), 0);
	DISP_REG_SET(handle, DISP_REG_CONFIG_MUTEX_SOF(mutex_id), 0);
}

int ddp_mutex_set_sof_wait(int mutex_id, struct cmdqRecStruct *handle,
			   int wait)
{
	if (mutex_id < DISP_MUTEX_DDP_FIRST ||
	    mutex_id > DISP_MUTEX_DDP_LAST) {
		DDPERR("exceed mutex max (0 ~ %d)\n", DISP_MUTEX_DDP_LAST);
		return -1;
	}

	DISP_REG_SET_FIELD(handle, SOF_FLD_MUTEX0_SOF_WAIT,
			   DISP_REG_CONFIG_MUTEX_SOF(mutex_id), wait);
	return 0;
}

int ddp_mutex_enable(int mutex_id, enum DDP_SCENARIO_ENUM scenario,
		     enum DDP_MODE mode, void *handle)
{
	DDPDBG("mutex %d enable\n", mutex_id);
	/* disable mutex dcm */
	DISP_REG_SET(handle, DISP_REG_CONFIG_MUTEX_CFG, 0);
	DISP_REG_SET(handle, DISP_REG_CONFIG_MUTEX_EN(mutex_id), 1);
	return 0;
}

int ddp_mutex_set(int mutex_id, enum DDP_SCENARIO_ENUM scenario,
		  enum DDP_MODE mode, void *handle)
{
	if (scenario < DDP_SCENARIO_MAX)
		return ddp_mutex_set_l(
		    mutex_id, module_list_scenario[scenario], mode, handle);

	DDPERR("Invalid scenario %d when setting mutex\n", scenario);
	return -1;
}

void ddp_check_mutex(int mutex_id, enum DDP_SCENARIO_ENUM scenario,
		     enum DDP_MODE mode)
{
	DDPDBG("check mutex %d on scenario %s\n", mutex_id,
	       ddp_get_scenario_name(scenario));
	ddp_check_mutex_l(mutex_id, module_list_scenario[scenario], mode);
}

char *ddp_get_mutex_sof_name(unsigned int regval)
{
	switch (regval) {
	case SOF_VAL_MUTEX0_SOF_SINGLE_MODE:
		return "single";
	case SOF_VAL_MUTEX0_SOF_FROM_DSI0:
		return "dsi0";
	case SOF_VAL_MUTEX0_SOF_FROM_DPI:
		return "dpi";
	default:
		DDPDUMP("%s, unknown reg=%d\n", __func__, regval);
		return "unknown";
	}
}

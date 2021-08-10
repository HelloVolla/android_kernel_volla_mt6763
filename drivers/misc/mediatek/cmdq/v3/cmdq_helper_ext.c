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

#include <linux/kernel.h>
#include <linux/memblock.h>
#include <linux/memory.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/uaccess.h>
#include <linux/notifier.h>
#include <linux/irqchip/mtk-gic-extend.h>
#include <linux/arm-smccc.h>
#include <mt-plat/mtk_secure_api.h>

#include "cmdq_helper_ext.h"
#include "cmdq_record.h"
#include "cmdq_device.h"
#include "cmdq_virtual.h"
#include "cmdq_reg.h"
#include "cmdq_mdp_common.h"
#ifdef CMDQ_SECURE_PATH_SUPPORT
#include "cmdq_sec.h"
#endif
#ifdef CMDQ_PROFILE_MMP
#include "cmdq_mmp.h"
#endif

#define CMDQ_GET_COOKIE_CNT(thread) \
	(CMDQ_REG_GET32(CMDQ_THR_EXEC_CNT(thread)) & CMDQ_MAX_COOKIE_VALUE)
#define CMDQ_SYNC_TOKEN_APPEND_THR(id)     (CMDQ_SYNC_TOKEN_APPEND_THR0 + id)

#define CMDQ_PROFILE_LIMIT_0	3000000
#define CMDQ_PROFILE_LIMIT_1	10000000
#define CMDQ_PROFILE_LIMIT_2	20000000

struct cmdq_cmd_struct {
	void *p_va_base;	/* VA: denote CMD virtual address space */
	dma_addr_t mva_base;	/* PA: denote the PA for CMD */
	u32 sram_base;		/* SRAM base offset for CMD start */
	u32 buffer_size;	/* size of allocated command buffer */
};

/* mutex and spinlock in core, first define first lock */
static DEFINE_MUTEX(cmdq_clock_mutex);
static DEFINE_MUTEX(cmdq_res_mutex);
static DEFINE_MUTEX(cmdq_err_mutex);
static DEFINE_MUTEX(cmdq_handle_list_mutex);
static DEFINE_MUTEX(cmdq_thread_mutex);
static DEFINE_MUTEX(cmdq_inst_check_mutex);

static DEFINE_SPINLOCK(cmdq_write_addr_lock);
static DEFINE_SPINLOCK(cmdq_record_lock);
#ifdef CMDQ_TIMER_ENABLE
static DEFINE_SPINLOCK(cmdq_delay_thd_lock);
#endif
static DEFINE_SPINLOCK(cmdq_first_err_lock);

/* callbacks */
static BLOCKING_NOTIFIER_HEAD(cmdq_status_dump_notifier);

static struct cmdq_client *cmdq_clients[CMDQ_MAX_THREAD_COUNT];
static struct cmdq_base *cmdq_client_base;
static atomic_t cmdq_thread_usage;

static wait_queue_head_t *cmdq_wait_queue; /* task done notify */
static struct ContextStruct cmdq_ctx; /* cmdq driver context */
static struct DumpFirstErrorStruct cmdq_first_err;
static const char *cmdq_first_err_mod;
static struct DumpCommandBufferStruct cmdq_command_dump;
static struct CmdqCBkStruct cmdq_group_cb[CMDQ_MAX_GROUP_COUNT];
static struct CmdqDebugCBkStruct cmdq_debug_cb;
static struct cmdq_dts_setting cmdq_dts;
static struct cmdqDTSDataStruct cmdq_dts_data;
static struct SubsysStruct cmdq_adds_subsys = {
	.msb = 0,
	.subsysID = -1,
	.mask = 0,
	.grpName = "AddOn"
};

#ifdef CMDQ_TIMER_ENABLE
static struct cmdq_cmd_struct cmdq_delay_thd_cmd;
static bool cmdq_delay_thd_started;
static bool cmdq_delay_thd_inited;
#endif

/* debug for alloc hw buffer count */
static atomic_t cmdq_alloc_cnt[CMDQ_CLT_MAX + 1];

/* CMDQ core feature functions */

/* #define CMDQ_DEBUG_ADDR */

enum xpr_rt_state {
	XPR_UNLOCK,
	XPR_UNKNOWN,
	XPR_REG,
	XPR_DMA
};

struct cmdq_check {
	union {
		struct {
			u16 ci:16;
			u16 bi:16;
		};
		u32 val;
	};
	u16 ai:16;
	u8 sop:5;
	u8 opt:3;
	u8 op:8;
};

enum xpr_id {
	xpr_r0 = CMDQ_DATA_REG_JPEG,
	xpr_p1 = CMDQ_DATA_REG_JPEG_DST,
	xpr_r5 = CMDQ_DATA_REG_2D_SHARPNESS_0,
	xpr_p4 = CMDQ_DATA_REG_2D_SHARPNESS_0_DST,
	xpr_r10 = CMDQ_DATA_REG_2D_SHARPNESS_1,
	xpr_p6 = CMDQ_DATA_REG_2D_SHARPNESS_1_DST,
	xpr_r4 = CMDQ_DATA_REG_PQ_COLOR,
	xpr_p3 = CMDQ_DATA_REG_PQ_COLOR,
	xpr_r11 = CMDQ_DATA_REG_DEBUG,
	xpr_p7 = CMDQ_DATA_REG_DEBUG_DST,
	xpr_spr0 = 0,
	xpr_total = 32,
};

static u32 mdp_base[1] = {0};
static u32 mdp_sub_base[1] = {0};

static bool cmdq_mdp_is_reg_valid(const unsigned long pa)
{
	u32 base = (u32)(pa & 0xFFFFF000);
	u32 i;
	static u32 last_idx;

	if (base == mdp_base[last_idx])
		return true;

	for (i = 0; i < ARRAY_SIZE(mdp_base); i++)
		if (base == mdp_base[i]) {
			last_idx = i;
			return true;
		}

#ifdef CMDQ_DEBUG_ADDR
	CMDQ_LOG("[note]blocking pa:%#010lx\n", pa);
	return true;
#else
	return false;
#endif
}

static bool cmdq_mdp_is_sub_valid(u8 subsys, u16 offset)
{
	u32 base = (offset & 0xF000) | subsys;
	u32 i;
	static u32 last_idx;

	if (base == mdp_sub_base[last_idx])
		return true;

	for (i = 0; i < ARRAY_SIZE(mdp_sub_base); i++)
		if (base == mdp_sub_base[i]) {
			last_idx = i;
			return true;
		}
#ifdef CMDQ_DEBUG_ADDR
	CMDQ_LOG("[note]blocking subsys:%#04x %#06x\n",
		(u32)subsys, (u32)offset);
	return true;
#else
	return false;
#endif
}

static bool cmdq_core_check_dma_valid(const unsigned long pa)
{
	struct WriteAddrStruct *waddr = NULL;
	unsigned long flags = 0L;
	bool ret = false;

	spin_lock_irqsave(&cmdq_write_addr_lock, flags);
	list_for_each_entry(waddr, &cmdq_ctx.writeAddrList, list_node)
		if (pa - (unsigned long)waddr->pa < waddr->count << 2) {
			ret = true;
			break;
		}
	spin_unlock_irqrestore(&cmdq_write_addr_lock, flags);
	return ret;
}

static bool cmdq_core_check_addr_valid(const unsigned long pa, bool *dma)
{
	static phys_addr_t start;

	if (!start)
		start = memblock_start_of_DRAM();

	if (pa < start) {
		*dma = false;
		return cmdq_mdp_is_reg_valid((u32)pa);
	}

	*dma = true;
	return cmdq_core_check_dma_valid(pa);
}

static bool cmdq_core_check_move(const struct cmdq_check *check,
	enum xpr_rt_state *xpr)
{
	bool dma;

	if (((u32 *)check)[1] == 0x2000000)
		return true;

	if (unlikely(check->opt != 0x4))
		return false;

	if (unlikely(!cmdq_core_check_addr_valid(check->val, &dma)))
		return false;

	if (unlikely(xpr[check->sop] == XPR_UNLOCK))
		return false;
	xpr[check->sop] = dma ? XPR_DMA : XPR_REG;
	return true;
}

#ifdef CMDQ_MDP_ENABLE_SPR
static bool cmdq_core_check_logic(const struct cmdq_check *check,
	enum xpr_rt_state *xpr, u32 *spr)
{
	if (unlikely(check->opt != 0x4 || check->ai || check->sop ||
		check->bi))
		return false;
	*spr = check->val;
	xpr[xpr_spr0] = XPR_UNKNOWN;
	return true;
}

static bool cmdq_core_check_write_s(const struct cmdq_check *check,
	enum xpr_rt_state *xpr, u32 *spr)
{
	u8 base_type = check->ai & 0x2;
	bool dma;

	switch (check->opt) {
	case 0:
		break;
	case 0x2:
		if (unlikely(!base_type || check->bi != 1))
			return false;
		break;
	default:
		return false;
	}

	if (base_type) { /* base + offset case */
		if (unlikely(check->sop)) /* must spr0 */
			return false;
		if (unlikely(xpr[check->sop] != XPR_UNKNOWN))
			return false;
		return cmdq_core_check_addr_valid(
			(*spr << 16) | (check->ai & ~0x3), &dma);
	}
	return cmdq_mdp_is_sub_valid(check->sop, check->ai);
}

static bool cmdq_core_check_read_s(const struct cmdq_check *check,
	enum xpr_rt_state *xpr, u32 *spr)
{
	if (unlikely(check->opt != 0x4 || check->ai != 1))
		return false;

	if (check->bi & 0x2) {
		if (unlikely(check->sop)) /* must spr0 */
			return false;
		if (unlikely(xpr[check->sop] != XPR_UNKNOWN))
			return false;
		return cmdq_mdp_is_reg_valid(
			(*spr << 16) | (check->bi & ~0x3));
	}

	return cmdq_mdp_is_sub_valid(check->sop, check->bi);
}

#endif

static bool cmdq_core_valid_gpr_token(u16 event, bool lock,
	enum xpr_rt_state *xpr)
{
	static u16 gpr_idx_r[] = {
		xpr_r0,
		xpr_r5,
		xpr_r10,
		xpr_r4,
		xpr_r11,
	};

	static u16 gpr_idx_p[] = {
		xpr_p1,
		xpr_p4,
		xpr_p6,
		xpr_p3,
		xpr_p7,
	};

	u16 idx = event - CMDQ_SYNC_TOKEN_GPR_SET_0;

	if (idx >= ARRAY_SIZE(gpr_idx_r))
		return true;

	if (lock) {
		if (unlikely(xpr[gpr_idx_r[idx]] != XPR_UNLOCK ||
			xpr[gpr_idx_p[idx]] != XPR_UNLOCK))
			return false;
		xpr[gpr_idx_r[idx]] = XPR_UNKNOWN;
		xpr[gpr_idx_p[idx]] = XPR_UNKNOWN;
	} else {
		if (unlikely(xpr[gpr_idx_r[idx]] == XPR_UNLOCK ||
			xpr[gpr_idx_p[idx]] == XPR_UNLOCK))
			return false;
		xpr[gpr_idx_r[idx]] = XPR_UNLOCK;
		xpr[gpr_idx_p[idx]] = XPR_UNLOCK;
	}

	return true;
}

static bool cmdq_core_check_event(const struct cmdq_check *check,
	enum xpr_rt_state *xpr)
{
	u64 pattern = *(u64 *)check & 0xfffffc00ffffffff;

	switch (pattern) {
	case 0x2000000080008001: /* wait and clear event */
		return cmdq_core_valid_gpr_token(check->ai, true, xpr);
	case 0x2000000080010000: /* set */
		return cmdq_core_valid_gpr_token(check->ai, false, xpr);
	case 0x2000000080018000: /* acquire */
	case 0x2000000080000000: /* clear event */
	case 0x2000000000008001: /* wait no clear */
		if (unlikely(check->ai == CMDQ_SYNC_TOKEN_GPR_SET_0 ||
			check->ai == CMDQ_SYNC_TOKEN_GPR_SET_1 ||
			check->ai == CMDQ_SYNC_TOKEN_GPR_SET_2 ||
			check->ai == CMDQ_SYNC_TOKEN_GPR_SET_3 ||
			check->ai == CMDQ_SYNC_TOKEN_GPR_SET_4))
			return false;
		break;
	default:
		return false;
	}

	return true;
}

static bool cmdq_core_check_write(const struct cmdq_check *check,
	enum xpr_rt_state *xpr)
{
	switch (check->opt) {
	case 0:
		return cmdq_mdp_is_sub_valid(check->sop, check->ai);
	case 0x2:
		return check->val < xpr_total &&
			xpr[check->val] != XPR_UNLOCK &&
			cmdq_mdp_is_sub_valid(check->sop, check->ai);
	case 0x4:
		if (unlikely(xpr[check->sop] != XPR_DMA &&
			xpr[check->sop] != XPR_REG))
			return false;
		break;
	case 0x6:
		if (unlikely((xpr[check->sop] != XPR_DMA &&
			xpr[check->sop] != XPR_REG) ||
			check->val >= xpr_total ||
			xpr[check->val] != XPR_UNKNOWN)) {
			return false;
		}
		break;
	default:
		return false;
	}

	return true;
}

static bool cmdq_core_check_read(const struct cmdq_check *check,
	enum xpr_rt_state *xpr)
{
	switch (check->opt) {
	case 0x2:
		if (unlikely(check->val >= xpr_total ||
			xpr[check->val] == XPR_UNLOCK) ||
			!cmdq_mdp_is_sub_valid(check->sop, check->ai))
			return false;
		break;
	case 0x6:
		if (unlikely(check->sop >= xpr_total ||
			check->val >= xpr_total ||
			xpr[check->sop] != XPR_REG ||
			xpr[check->val] == XPR_UNLOCK))
			return false;
		break;
	default:
		return false;
	}

	/* state to unknown */
	xpr[check->val] = XPR_UNKNOWN;
	return true;
}

static bool cmdq_core_check_instr_valid(const u64 instr,
	enum xpr_rt_state *gpr, enum xpr_rt_state *spr, u32 *spr_val)
{
	const struct cmdq_check *check = (void *)&instr;

	switch (check->op) {
	case CMDQ_CODE_WFE:
		return cmdq_core_check_event(check, gpr);
#ifdef CMDQ_MDP_ENABLE_SPR
	case CMDQ_CODE_WRITE_S:
	case CMDQ_CODE_WRITE_S_W_MASK:
		return cmdq_core_check_write_s(check, spr, spr_val);
	case CMDQ_CODE_LOGIC:
		return cmdq_core_check_logic(check, spr, spr_val);
	case CMDQ_CODE_READ_S:
		return cmdq_core_check_read_s(check, spr, spr_val);
#endif
	case CMDQ_CODE_WRITE:
		return cmdq_core_check_write(check, gpr);
	case CMDQ_CODE_READ:
		return cmdq_core_check_read(check, gpr);
	case CMDQ_CODE_MOVE:
		return cmdq_core_check_move(check, gpr);
	case CMDQ_CODE_JUMP:
#ifdef CMDQ_MDP_ENABLE_SPR
		return instr == 0x1000000000000001;
#else
		return instr == 0x1000000000000008;
#endif
	case CMDQ_CODE_EOC:
	case CMDQ_CODE_POLL:
		return true;

	default:
		return false;
	}
}

bool cmdq_core_check_user_valid(void *src, u32 size,
	struct cmdqRecStruct *handle)
{
	enum xpr_rt_state gpr[xpr_total] = {0};
	enum xpr_rt_state spr[xpr_total] = {0};
	u32 spr0 = 0;
	void *buffer;
	u64 *va;
	bool ret = true;
	u32 copy_size;
	u32 remain_size = size;
	void *cur_src = src;
	CMDQ_TIME cost = sched_clock();

	mutex_lock(&cmdq_inst_check_mutex);
	if (!cmdq_ctx.inst_check_buffer) {
		cmdq_ctx.inst_check_buffer = kmalloc(CMDQ_CMD_BUFFER_SIZE,
			GFP_KERNEL);
		if (!cmdq_ctx.inst_check_buffer) {
			CMDQ_ERR("fail to alloc check buffer\n");
			mutex_unlock(&cmdq_inst_check_mutex);
			return false;
		}
	}

	buffer = cmdq_ctx.inst_check_buffer;

	while (remain_size > 0 && ret) {
		copy_size = remain_size > CMDQ_CMD_BUFFER_SIZE ?
			CMDQ_CMD_BUFFER_SIZE : remain_size;
		if (copy_from_user(buffer, cur_src, copy_size)) {
			CMDQ_ERR("copy from user fail size:%u\n", size);
			ret = false;
			break;
		}

		for (va = (u64 *)buffer;
			va < (u64 *)(buffer + copy_size); va++) {
			ret = cmdq_core_check_instr_valid(*va, gpr, spr,
				&spr0);
			if (unlikely(!ret)) {
				CMDQ_ERR("instr:%#llx\n", *va);
				break;
			}
		}

		remain_size -= copy_size;
		cur_src += copy_size;

		cmdq_pkt_copy_cmd(handle, buffer, copy_size, false);
	}

	mutex_unlock(&cmdq_inst_check_mutex);

	cost = sched_clock() - cost;
	do_div(cost, 1000);

	CMDQ_MSG("%s size:%u cost:%lluus ret:%s\n", __func__, size, (u64)cost,
		ret ? "true" : "false");

	return ret;
}

static void cmdq_core_config_prefetch_gsize(void)
{
	u32 i = 0, prefetch_gsize = 0, total_size = 0;

	if (!cmdq_dts.prefetch_thread_count)
		return;

	for (i = 0; i < cmdq_dts.prefetch_thread_count; i++) {
		total_size += cmdq_dts.prefetch_size[i];
		prefetch_gsize |= (cmdq_dts.prefetch_size[i] /
			32 - 1) << (i * 4);
	}

	CMDQ_REG_SET32(CMDQ_PREFETCH_GSIZE, prefetch_gsize);
	CMDQ_MSG("prefetch gsize configure:0x%08x total size:%u\n",
		prefetch_gsize, total_size);
}

static void cmdq_core_init_thread_work_queue(void)
{
	int index;
	const u32 max_thread_count = cmdq_dev_get_thread_count();

	cmdq_ctx.taskThreadAutoReleaseWQ = kcalloc(max_thread_count,
		sizeof(*cmdq_ctx.taskThreadAutoReleaseWQ), GFP_KERNEL);

	/* Initialize work queue per thread */
	for (index = 0; index < max_thread_count; index++) {
		cmdq_ctx.taskThreadAutoReleaseWQ[index] =
		    create_singlethread_workqueue("cmdq_auto_release_thread");
	}
}

static void cmdq_core_destroy_thread_work_queue(void)
{
	int index;
	const u32 max_thread_count = cmdq_dev_get_thread_count();

	/* destroy work queue per thread */
	for (index = 0; index < max_thread_count; index++) {
		destroy_workqueue(
			cmdq_ctx.taskThreadAutoReleaseWQ[index]);
		cmdq_ctx.taskThreadAutoReleaseWQ[index] = NULL;
	}

	kfree(cmdq_ctx.taskThreadAutoReleaseWQ);
}

void cmdq_core_deinit_group_cb(void)
{
	memset(&cmdq_group_cb, 0x0, sizeof(cmdq_group_cb));
	memset(&cmdq_debug_cb, 0x0, sizeof(cmdq_debug_cb));
}

static bool cmdq_core_is_valid_group(enum CMDQ_GROUP_ENUM engGroup)
{
	/* check range */
	if (engGroup < 0 || engGroup >= CMDQ_MAX_GROUP_COUNT)
		return false;

	return true;
}

s32 cmdqCoreRegisterCB(enum CMDQ_GROUP_ENUM engGroup,
	CmdqClockOnCB clockOn, CmdqDumpInfoCB dumpInfo,
	CmdqResetEngCB resetEng, CmdqClockOffCB clockOff)
{
	struct CmdqCBkStruct *callback;

	if (!cmdq_core_is_valid_group(engGroup))
		return -EFAULT;

	CMDQ_MSG("Register %d group engines' callback\n", engGroup);
	CMDQ_MSG("clockOn:%pf dumpInfo:%pf resetEng:%pf clockOff:%pf\n",
		clockOn, dumpInfo, resetEng, clockOff);

	callback = &cmdq_group_cb[engGroup];
	callback->clockOn = clockOn;
	callback->dumpInfo = dumpInfo;
	callback->resetEng = resetEng;
	callback->clockOff = clockOff;

	return 0;
}

s32 cmdqCoreRegisterDispatchModCB(
	enum CMDQ_GROUP_ENUM engGroup, CmdqDispatchModuleCB dispatchMod)
{
	struct CmdqCBkStruct *callback;

	if (!cmdq_core_is_valid_group(engGroup))
		return -EFAULT;

	CMDQ_MSG("Register %d group engines' dispatch callback\n",
		engGroup);
	callback = &cmdq_group_cb[engGroup];
	callback->dispatchMod = dispatchMod;

	return 0;
}

s32 cmdqCoreRegisterDebugRegDumpCB(
	CmdqDebugRegDumpBeginCB beginCB, CmdqDebugRegDumpEndCB endCB)
{
	CMDQ_VERBOSE("Register reg dump: begin:%p end:%p\n",
		beginCB, endCB);
	cmdq_debug_cb.beginDebugRegDump = beginCB;
	cmdq_debug_cb.endDebugRegDump = endCB;
	return 0;
}

s32 cmdqCoreRegisterTrackTaskCB(enum CMDQ_GROUP_ENUM engGroup,
	CmdqTrackTaskCB trackTask)
{
	struct CmdqCBkStruct *callback;

	if (!cmdq_core_is_valid_group(engGroup))
		return -EFAULT;

	CMDQ_MSG("Register %d group engines' callback\n", engGroup);
	CMDQ_MSG("trackTask:%pf\n", trackTask);

	callback = &cmdq_group_cb[engGroup];
	callback->trackTask = trackTask;

	return 0;
}

s32 cmdqCoreRegisterErrorResetCB(enum CMDQ_GROUP_ENUM engGroup,
	CmdqErrorResetCB errorReset)
{
	struct CmdqCBkStruct *callback;

	if (!cmdq_core_is_valid_group(engGroup))
		return -EFAULT;

	CMDQ_MSG("Register %d group engines' callback\n", engGroup);
	CMDQ_MSG("errorReset:%pf\n", errorReset);

	callback = &cmdq_group_cb[engGroup];
	callback->errorReset = errorReset;

	return 0;
}

void cmdq_core_register_status_dump(struct notifier_block *notifier)
{
	s32 ret;

	CMDQ_LOG("%s notifier:0x%p\n", __func__, notifier);
	ret = blocking_notifier_chain_register(&cmdq_status_dump_notifier,
		notifier);
	if (ret < 0)
		CMDQ_ERR("fail to register chanin:0x%p\n", notifier);
}

void cmdq_core_remove_status_dump(struct notifier_block *notifier)
{
	s32 ret;

	ret = blocking_notifier_chain_unregister(&cmdq_status_dump_notifier,
		notifier);
	if (ret < 0)
		CMDQ_ERR("fail to unregister chanin:0x%p\n", notifier);
}

/* for PMQOS task begin/end */
s32 cmdq_core_register_task_cycle_cb(enum CMDQ_GROUP_ENUM group,
	CmdqBeginTaskCB beginTask, CmdqEndTaskCB endTask)
{
	struct CmdqCBkStruct *callback;

	if (!cmdq_core_is_valid_group(group))
		return -EFAULT;

	CMDQ_MSG("Register %d group engines' callback begin:%pf end:%pf\n",
		group, beginTask, endTask);

	callback = &cmdq_group_cb[group];

	callback->beginTask = beginTask;
	callback->endTask = endTask;
	return 0;
}

const char *cmdq_core_parse_op(u32 op_code)
{
	switch (op_code) {
	case CMDQ_CODE_POLL:
		return "POLL";
	case CMDQ_CODE_WRITE:
		return "WRIT";
	case CMDQ_CODE_WFE:
		return "SYNC";
	case CMDQ_CODE_READ:
		return "READ";
	case CMDQ_CODE_MOVE:
		return "MASK";
	case CMDQ_CODE_JUMP:
		return "JUMP";
	case CMDQ_CODE_EOC:
		return "MARK";
	case CMDQ_CODE_READ_S:
		return "READ_S";
	case CMDQ_CODE_WRITE_S:
		return "WRITE_S";
	case CMDQ_CODE_WRITE_S_W_MASK:
		return "WRITE_S with mask";
	case CMDQ_CODE_LOGIC:
		return "LOGIC";
	case CMDQ_CODE_JUMP_C_RELATIVE:
		return "JUMP_C related";
	case CMDQ_CODE_JUMP_C_ABSOLUTE:
		return "JUMP_C absolute";
	}
	return NULL;
}

static const char *cmdq_core_parse_logic_sop(u32 s_op)
{
	switch (s_op) {
	case CMDQ_LOGIC_ASSIGN:
		return "=";
	case CMDQ_LOGIC_ADD:
		return "+";
	case CMDQ_LOGIC_SUBTRACT:
		return "-";
	case CMDQ_LOGIC_MULTIPLY:
		return "*";
	case CMDQ_LOGIC_XOR:
		return "^";
	case CMDQ_LOGIC_NOT:
		return "~";
	case CMDQ_LOGIC_OR:
		return "|";
	case CMDQ_LOGIC_AND:
		return "&";
	case CMDQ_LOGIC_LEFT_SHIFT:
		return "<<";
	case CMDQ_LOGIC_RIGHT_SHIFT:
		return ">>";
	}
	return NULL;
}

static const char *cmdq_core_parse_jump_c_sop(u32 s_op)
{
	switch (s_op) {
	case CMDQ_EQUAL:
		return "==";
	case CMDQ_NOT_EQUAL:
		return "!=";
	case CMDQ_GREATER_THAN_AND_EQUAL:
		return ">=";
	case CMDQ_LESS_THAN_AND_EQUAL:
		return "<=";
	case CMDQ_GREATER_THAN:
		return ">";
	case CMDQ_LESS_THAN:
		return "<";
	}
	return NULL;
}

static u32 cmdq_core_interpret_wpr_value_data_register(
	const u32 op, u32 arg_value)
{
	u32 reg_id = arg_value;

	if (op == CMDQ_CODE_WRITE_S || op == CMDQ_CODE_WRITE_S_W_MASK)
		reg_id = ((arg_value >> 16) & 0xFFFF);
	else if (op == CMDQ_CODE_READ_S)
		reg_id = (arg_value & 0xFFFF);
	else
		reg_id += CMDQ_GPR_V3_OFFSET;
	return reg_id;
}

static u32 cmdq_core_interpret_wpr_address_data_register(
	const u32 op, u32 arg_addr)
{
	u32 reg_id = (arg_addr >> 16) & 0x1F;

	if (op == CMDQ_CODE_WRITE_S || op == CMDQ_CODE_WRITE_S_W_MASK ||
		op == CMDQ_CODE_READ_S)
		reg_id = (arg_addr & 0xFFFF);
	else
		reg_id += CMDQ_GPR_V3_OFFSET;
	return reg_id;
}

s32 cmdq_core_interpret_instruction(char *textBuf, s32 bufLen,
	const u32 op, const u32 arg_a, const u32 arg_b)
{
	int reqLen = 0;
	u32 arg_addr, arg_value;
	u32 arg_addr_type, arg_value_type;
	u32 reg_addr;
	u32 reg_id, use_mask;
	const u32 addr_mask = 0xFFFFFFFE;

	switch (op) {
	case CMDQ_CODE_MOVE:
		if (1 & (arg_a >> 23)) {
			reg_id = ((arg_a >> 16) & 0x1f) + CMDQ_GPR_V3_OFFSET;
			reqLen = snprintf(textBuf, bufLen,
				"MOVE:0x%08x to Reg%d\n", arg_b, reg_id);
		} else {
			reqLen = snprintf(textBuf, bufLen,
				"Set MASK:0x%08x\n", arg_b);
		}
		break;
	case CMDQ_CODE_READ:
	case CMDQ_CODE_WRITE:
	case CMDQ_CODE_POLL:
	case CMDQ_CODE_READ_S:
	case CMDQ_CODE_WRITE_S:
	case CMDQ_CODE_WRITE_S_W_MASK:
		reqLen = snprintf(textBuf, bufLen, "%s: ",
			cmdq_core_parse_op(op));
		bufLen -= reqLen;
		textBuf += reqLen;

		if (op == CMDQ_CODE_READ_S) {
			arg_addr = (arg_a & CMDQ_ARG_A_SUBSYS_MASK) |
				((arg_b >> 16) & 0xFFFF);
			arg_addr_type = arg_a & (1 << 22);
			arg_value = arg_a & 0xFFFF;
			arg_value_type = arg_a & (1 << 23);
		} else {
			arg_addr = arg_a;
			arg_addr_type = arg_a & (1 << 23);
			arg_value = arg_b;
			arg_value_type = arg_a & (1 << 22);
		}

		/* data (value) */
		if (arg_value_type != 0) {
			reg_id = cmdq_core_interpret_wpr_value_data_register(
				op, arg_value);
			reqLen = snprintf(textBuf, bufLen, "Reg%d, ", reg_id);
			bufLen -= reqLen;
			textBuf += reqLen;
		} else {
			reqLen = snprintf(textBuf, bufLen, "0x%08x, ",
				arg_value);
			bufLen -= reqLen;
			textBuf += reqLen;
		}

		/* address */
		if (arg_addr_type != 0) {
			reg_id = cmdq_core_interpret_wpr_address_data_register(
				op, arg_addr);
			reqLen = snprintf(textBuf, bufLen, "Reg%d, ", reg_id);
			bufLen -= reqLen;
			textBuf += reqLen;
		} else {
			reg_addr = cmdq_core_subsys_to_reg_addr(arg_addr);

			reqLen = snprintf(textBuf, bufLen,
				"addr:0x%08x [%s], ", reg_addr & addr_mask,
				cmdq_get_func()->parseModule(reg_addr));
			bufLen -= reqLen;
			textBuf += reqLen;
		}

		use_mask = (arg_addr & 0x1);
		if (op == CMDQ_CODE_WRITE_S_W_MASK)
			use_mask = 1;
		else if (op == CMDQ_CODE_READ_S || op == CMDQ_CODE_WRITE_S)
			use_mask = 0;
		reqLen = snprintf(textBuf, bufLen, "use_mask:%d\n", use_mask);
		bufLen -= reqLen;
		textBuf += reqLen;
		break;
	case CMDQ_CODE_JUMP:
		if (arg_a) {
			if (arg_a & (1 << 22)) {
				/* jump by register */
				reqLen = snprintf(textBuf, bufLen,
					"JUMP(register): Reg%d\n", arg_b);
			} else {
				/* absolute */
				reqLen = snprintf(textBuf, bufLen,
					"JUMP(absolute):0x%08x\n", arg_b);
			}
		} else {
			/* relative */
			if ((s32) arg_b >= 0) {
				reqLen = snprintf(textBuf, bufLen,
					"JUMP(relative):+%d\n",
					(s32)CMDQ_REG_REVERT_ADDR(arg_b));
			} else {
				reqLen = snprintf(textBuf, bufLen,
					"JUMP(relative):%d\n",
					(s32)CMDQ_REG_REVERT_ADDR(arg_b));
			}
		}
		break;
	case CMDQ_CODE_WFE:
		if (arg_b == 0x80008001) {
			reqLen = snprintf(textBuf, bufLen,
				"Wait And Clear Event:%s\n",
				cmdq_core_get_event_name(arg_a));
		} else if (arg_b == 0x80000000) {
			reqLen = snprintf(textBuf, bufLen, "Clear Event:%s\n",
				cmdq_core_get_event_name(arg_a));
		} else if (arg_b == 0x80010000) {
			reqLen = snprintf(textBuf, bufLen, "Set Event:%s\n",
				cmdq_core_get_event_name(arg_a));
		} else if (arg_b == 0x00008001) {
			reqLen = snprintf(textBuf, bufLen,
				"Wait No Clear Event:%s\n",
				cmdq_core_get_event_name(arg_a));
		} else {
			reqLen = snprintf(textBuf, bufLen,
				"SYNC:%s, upd:%d op:%d val:%d wait:%d wop:%d val:%d\n",
				cmdq_core_get_event_name(arg_a),
				(arg_b >> 31) & 0x1,
				(arg_b >> 28) & 0x7,
				(arg_b >> 16) & 0xFFF,
				(arg_b >> 15) & 0x1,
				(arg_b >> 12) & 0x7, (arg_b >> 0) & 0xFFF);
		}
		break;
	case CMDQ_CODE_EOC:
		if (arg_a == 0 && arg_b == 0x00000001) {
			reqLen = snprintf(textBuf, bufLen, "EOC\n");
		} else {
			reqLen = snprintf(textBuf, bufLen, "MARKER:");
			bufLen -= reqLen;
			textBuf += reqLen;
			if (arg_b == 0x00100000) {
				reqLen = snprintf(textBuf, bufLen, " Disable");
			} else if (arg_b == 0x00130000) {
				reqLen = snprintf(textBuf, bufLen, " Enable");
			} else {
				reqLen = snprintf(textBuf, bufLen,
					"no_suspnd:%d no_inc:%d m:%d m_en:%d prefetch:%d irq:%d\n",
						(arg_a & (1 << 21)) > 0,
						(arg_a & (1 << 16)) > 0,
						(arg_b & (1 << 20)) > 0,
						(arg_b & (1 << 17)) > 0,
						(arg_b & (1 << 16)) > 0,
						(arg_b & (1 << 0)) > 0);
			}
		}
		break;
	case CMDQ_CODE_LOGIC:
		{
			const u32 subsys_bit =
				cmdq_get_func()->getSubsysLSBArgA();
			const u32 s_op =
				(arg_a & CMDQ_ARG_A_SUBSYS_MASK) >> subsys_bit;

			reqLen = snprintf(textBuf, bufLen, "%s: ",
				cmdq_core_parse_op(op));
			bufLen -= reqLen;
			textBuf += reqLen;

			reqLen = snprintf(textBuf, bufLen, "Reg%d = ",
				(arg_a & 0xFFFF));
			bufLen -= reqLen;
			textBuf += reqLen;

			if (s_op == CMDQ_LOGIC_ASSIGN) {
				reqLen = snprintf(textBuf, bufLen, "0x%08x\n",
					arg_b);
				bufLen -= reqLen;
				textBuf += reqLen;
			} else if (s_op == CMDQ_LOGIC_NOT) {
				const u32 arg_b_type = arg_a & (1 << 22);
				const u32 arg_b_i = (arg_b >> 16) & 0xFFFF;

				if (arg_b_type != 0)
					reqLen = snprintf(textBuf, bufLen,
						"~Reg%d\n", arg_b_i);
				else
					reqLen = snprintf(textBuf, bufLen,
						"~%d\n", arg_b_i);

				bufLen -= reqLen;
				textBuf += reqLen;
			} else {
				const u32 arg_b_i = (arg_b >> 16) & 0xFFFF;
				const u32 arg_c_i = arg_b & 0xFFFF;
				const u32 arg_b_type = arg_a & (1 << 22);
				const u32 arg_c_type = arg_a & (1 << 21);

				/* arg_b_i */
				if (arg_b_type != 0) {
					reqLen = snprintf(textBuf, bufLen,
						"Reg%d ", arg_b_i);
					bufLen -= reqLen;
					textBuf += reqLen;
				} else {
					reqLen = snprintf(textBuf, bufLen,
						"%d ", arg_b_i);
					bufLen -= reqLen;
					textBuf += reqLen;
				}
				/* operator */
				reqLen = snprintf(textBuf, bufLen, "%s ",
					cmdq_core_parse_logic_sop(s_op));
				bufLen -= reqLen;
				textBuf += reqLen;
				/* arg_c_i */
				if (arg_c_type != 0) {
					reqLen = snprintf(textBuf, bufLen,
						"Reg%d\n", arg_c_i);
					bufLen -= reqLen;
					textBuf += reqLen;
				} else {
					reqLen = snprintf(textBuf, bufLen,
						"%d\n", CMDQ_REG_REVERT_ADDR(
						arg_c_i));
					bufLen -= reqLen;
					textBuf += reqLen;
				}
			}
		}
		break;
	case CMDQ_CODE_JUMP_C_RELATIVE:
	case CMDQ_CODE_JUMP_C_ABSOLUTE:
		{
			const u32 subsys_bit =
				cmdq_get_func()->getSubsysLSBArgA();
			const u32 s_op =
				(arg_a & CMDQ_ARG_A_SUBSYS_MASK) >> subsys_bit;
			const u32 arg_a_i = arg_a & 0xFFFF;
			const u32 arg_b_i = (arg_b >> 16) & 0xFFFF;
			const u32 arg_c_i = arg_b & 0xFFFF;
			const u32 arg_a_type = arg_a & (1 << 23);
			const u32 arg_b_type = arg_a & (1 << 22);
			const u32 arg_c_type = arg_a & (1 << 21);

			reqLen = snprintf(textBuf, bufLen, "%s: if (",
				cmdq_core_parse_op(op));
			bufLen -= reqLen;
			textBuf += reqLen;

			/* arg_b_i */
			if (arg_b_type != 0) {
				reqLen = snprintf(textBuf, bufLen, "Reg%d ",
					arg_b_i);
				bufLen -= reqLen;
				textBuf += reqLen;
			} else {
				reqLen = snprintf(textBuf, bufLen, "%d ",
					arg_b_i);
				bufLen -= reqLen;
				textBuf += reqLen;
			}
			/* operator */
			reqLen = snprintf(textBuf, bufLen, "%s ",
				cmdq_core_parse_jump_c_sop(s_op));
			bufLen -= reqLen;
			textBuf += reqLen;
			/* arg_c_i */
			if (arg_c_type != 0) {
				reqLen = snprintf(textBuf, bufLen,
					"Reg%d) jump ", arg_c_i);
				bufLen -= reqLen;
				textBuf += reqLen;
			} else {
				reqLen = snprintf(textBuf, bufLen,
					"%d) jump ", CMDQ_REG_REVERT_ADDR(
					arg_c_i));
				bufLen -= reqLen;
				textBuf += reqLen;
			}
			/* jump to */
			if (arg_a_type != 0) {
				reqLen = snprintf(textBuf, bufLen,
					"Reg%d\n", arg_a_i);
				bufLen -= reqLen;
				textBuf += reqLen;
			} else {
				reqLen = snprintf(textBuf, bufLen,
					"+%d\n", arg_a_i);
				bufLen -= reqLen;
				textBuf += reqLen;
			}
		}
		break;
	default:
		reqLen = snprintf(textBuf, bufLen,
			"UNDEFINED (0x%02x 0x%08x%08x)\n",
			op, arg_a, arg_b);
		break;
	}

	return reqLen;
}

s32 cmdq_core_parse_instruction(const u32 *pCmd, char *textBuf, int bufLen)
{
	int reqLen = 0;

	const u32 op = (pCmd[1] & 0xFF000000) >> 24;
	const u32 arg_a = pCmd[1] & (~0xFF000000);
	const u32 arg_b = pCmd[0];

	reqLen = cmdq_core_interpret_instruction(
		textBuf, bufLen, op, arg_a, arg_b);

	return reqLen;
}

bool cmdq_core_should_print_msg(void)
{
	bool logLevel = (cmdq_ctx.logLevel & (1 << CMDQ_LOG_LEVEL_MSG)) ?
		(1) : (0);
	return logLevel;
}

bool cmdq_core_should_full_error(void)
{
	bool logLevel = (cmdq_ctx.logLevel &
		(1 << CMDQ_LOG_LEVEL_FULL_ERROR)) ? (1) : (0);
	return logLevel;
}

bool cmdq_core_should_pmqos_log(void)
{
	return cmdq_ctx.logLevel & (1 << CMDQ_LOG_LEVEL_PMQOS);
}

bool cmdq_core_should_secure_log(void)
{
	return cmdq_ctx.logLevel & (1 << CMDQ_LOG_LEVEL_SECURE);
}

bool cmdq_core_aee_enable(void)
{
	return cmdq_ctx.aee;
}

void cmdq_core_set_aee(bool enable)
{
	cmdq_ctx.aee = enable;
}

bool cmdq_core_met_enabled(void)
{
	return cmdq_ctx.enableProfile & (1 << CMDQ_PROFILE_MET);
}

bool cmdq_core_ftrace_enabled(void)
{
	return cmdq_ctx.enableProfile & (1 << CMDQ_PROFILE_FTRACE);
}

void cmdq_long_string_init(bool force, char *buf, u32 *offset, s32 *max_size)
{
	buf[0] = '\0';
	*offset = 0;
	if (force || cmdq_core_should_print_msg())
		*max_size = CMDQ_LONGSTRING_MAX - 1;
	else
		*max_size = 0;
}

void cmdq_long_string(char *buf, u32 *offset, s32 *max_size,
	const char *string, ...)
{
	int msg_len;
	va_list arg_ptr;
	char *buffer;

	if (*max_size <= 0)
		return;

	va_start(arg_ptr, string);
	buffer = buf + (*offset);
	msg_len = vsnprintf(buffer, *max_size, string, arg_ptr);
	*max_size -= msg_len;
	if (*max_size < 0)
		*max_size = 0;
	*offset += msg_len;
	va_end(arg_ptr);
}

s32 cmdq_core_reg_dump_begin(u32 taskID, u32 *regCount, u32 **regAddress)
{
	if (!cmdq_debug_cb.beginDebugRegDump) {
		CMDQ_ERR("beginDebugRegDump not registered\n");
		return -EFAULT;
	}

	return cmdq_debug_cb.beginDebugRegDump(taskID, regCount,
		regAddress);
}

s32 cmdq_core_reg_dump_end(u32 taskID, u32 regCount, u32 *regValues)
{
	if (!cmdq_debug_cb.endDebugRegDump) {
		CMDQ_ERR("endDebugRegDump not registered\n");
		return -EFAULT;
	}

	return cmdq_debug_cb.endDebugRegDump(taskID, regCount, regValues);
}

int cmdq_core_print_record_title(char *_buf, int bufLen)
{
	int length = 0;
	char *buf;

	buf = _buf;

	length = snprintf(buf, bufLen,
		"index,pid,scn,flag,task_pri,is_sec,size,thr#,thr_pri,");
	bufLen -= length;
	buf += length;

	length = snprintf(buf, bufLen,
		"submit,acq_thr,irq_time,begin_wait,exec_time,buf_alloc,buf_rec,buf_rel,total_time,start,end,jump\n");
	bufLen -= length;
	buf += length;

	length = (buf - _buf);

	return length;
}

static int cmdq_core_print_record(const struct RecordStruct *pRecord,
	int index, char *_buf, int bufLen)
{
	int length = 0;
	char *unit[5] = { "ms", "ms", "ms", "ms", "ms" };
	s32 IRQTime;
	s32 execTime;
	s32 beginWaitTime;
	s32 totalTime;
	s32 acquireThreadTime;
	unsigned long rem_nsec;
	CMDQ_TIME submitTimeSec;
	char *buf;
	s32 profileMarkerCount;
	s32 i;

	rem_nsec = 0;
	submitTimeSec = pRecord->submit;
	rem_nsec = do_div(submitTimeSec, 1000000000);
	buf = _buf;

	CMDQ_GET_TIME_IN_MS(pRecord->submit, pRecord->done, totalTime);
	CMDQ_GET_TIME_IN_MS(pRecord->submit, pRecord->trigger,
		acquireThreadTime);
	CMDQ_GET_TIME_IN_MS(pRecord->submit, pRecord->beginWait,
		beginWaitTime);
	CMDQ_GET_TIME_IN_MS(pRecord->trigger, pRecord->gotIRQ, IRQTime);
	CMDQ_GET_TIME_IN_MS(pRecord->trigger, pRecord->wakedUp, execTime);

	/* detect us interval */
	if (acquireThreadTime == 0) {
		CMDQ_GET_TIME_IN_US_PART(pRecord->submit, pRecord->trigger,
			acquireThreadTime);
		unit[0] = "us";
	}
	if (IRQTime == 0) {
		CMDQ_GET_TIME_IN_US_PART(pRecord->trigger, pRecord->gotIRQ,
			IRQTime);
		unit[1] = "us";
	}
	if (beginWaitTime == 0) {
		CMDQ_GET_TIME_IN_US_PART(pRecord->submit, pRecord->beginWait,
			beginWaitTime);
		unit[2] = "us";
	}
	if (execTime == 0) {
		CMDQ_GET_TIME_IN_US_PART(pRecord->trigger, pRecord->wakedUp,
			execTime);
		unit[3] = "us";
	}
	if (totalTime == 0) {
		CMDQ_GET_TIME_IN_US_PART(pRecord->submit, pRecord->done,
			totalTime);
		unit[4] = "us";
	}

	/* pRecord->priority for task priority */
	/* when pRecord->is_secure is 0 for secure task */
	length = snprintf(buf, bufLen,
		"%4d,%5d,%2d,0x%012llx,%2d,%d,%d,%02d,%02d,",
		index, pRecord->user, pRecord->scenario, pRecord->engineFlag,
		pRecord->priority, pRecord->is_secure, pRecord->size,
		pRecord->thread,
		cmdq_get_func()->priority(pRecord->scenario));
	bufLen -= length;
	buf += length;

	length = snprintf(buf, bufLen,
		"%5llu.%06lu,%4d%s,%4d%s,%4d%s,%4d%s,%dus,%dus,%dus,%4d%s,",
		submitTimeSec, rem_nsec / 1000, acquireThreadTime, unit[0],
		IRQTime, unit[1], beginWaitTime, unit[2], execTime, unit[3],
		pRecord->durAlloc, pRecord->durReclaim, pRecord->durRelease,
		totalTime, unit[4]);
	bufLen -= length;
	buf += length;

	/* record address */
	length = snprintf(buf, bufLen,
		"0x%08x,0x%08x,0x%08x",
		pRecord->start, pRecord->end, pRecord->jump);
	bufLen -= length;
	buf += length;

	profileMarkerCount = pRecord->profileMarkerCount;
	if (profileMarkerCount > CMDQ_MAX_PROFILE_MARKER_IN_TASK)
		profileMarkerCount = CMDQ_MAX_PROFILE_MARKER_IN_TASK;

	for (i = 0; i < profileMarkerCount; i++) {
		length = snprintf(buf, bufLen, ",%s,%lld",
			pRecord->profileMarkerTag[i],
			pRecord->profileMarkerTimeNS[i]);
		bufLen -= length;
		buf += length;
	}

	length = snprintf(buf, bufLen, "\n");
	bufLen -= length;
	buf += length;

	length = (buf - _buf);
	return length;
}

int cmdq_core_print_record_seq(struct seq_file *m, void *v)
{
	unsigned long flags;
	s32 index;
	s32 numRec;
	struct RecordStruct record;
	char msg[160] = { 0 };

	/* we try to minimize time spent in spin lock */
	/* since record is an array so it is okay to */
	/* allow displaying an out-of-date entry. */
	spin_lock_irqsave(&cmdq_record_lock, flags);
	numRec = cmdq_ctx.recNum;
	index = cmdq_ctx.lastID - 1;
	spin_unlock_irqrestore(&cmdq_record_lock, flags);

	/* we print record in reverse order. */
	cmdq_core_print_record_title(msg, sizeof(msg));
	seq_printf(m, "%s", msg);
	for (; numRec > 0; --numRec, --index) {
		if (index >= CMDQ_MAX_RECORD_COUNT)
			index = 0;
		else if (index < 0)
			index = CMDQ_MAX_RECORD_COUNT - 1;

		spin_lock_irqsave(&cmdq_record_lock, flags);
		record = cmdq_ctx.record[index];
		spin_unlock_irqrestore(&cmdq_record_lock, flags);

		cmdq_core_print_record(&record, index, msg, sizeof(msg));
		seq_printf(m, "%s", msg);
	}

	return 0;
}

static void cmdq_core_print_thd_usage(struct seq_file *m, void *v,
	u32 thread_idx)
{
	void *va = NULL;
	dma_addr_t pa = 0;
	u32 *pcva, pcpa;
	u32 insts[2] = {0};
	char parsed_inst[128] = { 0 };
	struct cmdq_core_thread *thread = &cmdq_ctx.thread[thread_idx];
	struct cmdqRecStruct *task;
	u32 counter = 0;

	seq_printf(m, "====== Thread %d Usage =======\n", thread_idx);
#if 0
	seq_printf(m, "Wait Cookie:%d Next Cookie:%d\n",
		thread->waitCookie, thread->nextCookie);
#endif
	seq_printf(m, "acquire:%u scenario:%d used:%s\n",
		thread->acquire, thread->scenario,
		thread->used ? "true" : "false");

	mutex_lock(&cmdq_handle_list_mutex);
	mutex_lock(&cmdq_thread_mutex);

	list_for_each_entry(task, &cmdq_ctx.handle_active, list_entry) {
		if (task->thread != thread_idx)
			continue;

		/* dump task basic info */
		seq_printf(m,
			   "Index:%u handle:0x%p Pid:%d Name:%s Scn:%d",
			   counter, task, task->caller_pid,
			   task->caller_name, task->scenario);

		/* here only print first buffer to reduce log */
		cmdq_pkt_get_first_buffer(task, &va, &pa);
		seq_printf(m,
			" va:0x%p pa:%pa size:%zu",
			va, &pa, task->pkt->cmd_buf_size);

		if (task->use_sram_buffer)
			seq_printf(m, " SRAM Base:%u",
				task->sram_base);

		if (task->cmd_end) {
			seq_printf(m,
				" Last Command:0x%08x:0x%08x",
				task->cmd_end[0], task->cmd_end[1]);
		}

		seq_puts(m, "\n");

		/* dump PC info */
		pcva = cmdq_core_get_pc_inst(task, thread_idx, insts, &pcpa);
		if (pcva) {
			cmdq_core_parse_instruction(
				pcva, parsed_inst, sizeof(parsed_inst));
			seq_printf(m,
				   "PC:0x%p(0x%08x) 0x%08x:0x%08x => %s",
				   pcva, pcpa, insts[0], insts[1],
				   parsed_inst);
		} else {
			seq_puts(m, "PC(VA): Not available\n");
		}

		counter++;
	}

	mutex_unlock(&cmdq_thread_mutex);
	mutex_unlock(&cmdq_handle_list_mutex);
}

/* TODO: move all engine to cmdq_mdp_common.c */
struct EngineStruct *cmdq_mdp_get_engines(void);

int cmdq_core_print_status_seq(struct seq_file *m, void *v)
{
	s32 index = 0;
	struct SRAMChunk *p_sram_chunk;
	struct cmdqRecStruct *handle = NULL;
	struct cmdq_client *client = NULL;
	struct cmdq_pkt_buffer *buf;
	const u32 max_thread_count = cmdq_dev_get_thread_count();

#ifdef CMDQ_DUMP_FIRSTERROR
	if (cmdq_first_err.cmdqCount > 0) {
		unsigned long long saveTimeSec = cmdq_first_err.savetime;
		unsigned long rem_nsec = do_div(saveTimeSec, 1000000000);
		struct tm nowTM;

		time_to_tm(cmdq_first_err.savetv.tv_sec,
			sys_tz.tz_minuteswest * 60, &nowTM);
		seq_puts(m, "================= [CMDQ] Dump first error ================\n");
		seq_printf(m, "kernel time:[%5llu.%06lu],",
			saveTimeSec, rem_nsec / 1000);
		seq_printf(m, " UTC time:[%04ld-%02d-%02d %02d:%02d:%02d.%06ld],",
			   (nowTM.tm_year + 1900), (nowTM.tm_mon + 1),
			   nowTM.tm_mday, nowTM.tm_hour, nowTM.tm_min,
			   nowTM.tm_sec, cmdq_first_err.savetv.tv_usec);
		seq_printf(m, " Pid:%d Name:%s\n", cmdq_first_err.callerPid,
			cmdq_first_err.callerName);
		if (cmdq_first_err.cmdqString)
			seq_printf(m, "%s", cmdq_first_err.cmdqString);
		else
			seq_puts(m, "\nWARNING: first error unavailable\n");
		if (cmdq_first_err.cmdqMaxSize <= 0)
			seq_printf(m, "\nWARNING: MAX size:%d is full\n",
			CMDQ_MAX_FIRSTERROR);
		seq_puts(m, "\n\n");
	}
#endif


	/* Save command buffer dump */
	if (cmdq_command_dump.count > 0) {
		s32 buffer_id;

		seq_puts(m, "================= [CMDQ] Dump Command Buffer =================\n");
		/* use which mutex? */
		for (buffer_id = 0; buffer_id < cmdq_command_dump.bufferSize;
			buffer_id++)
			seq_printf(m, "%c",
				cmdq_command_dump.cmdqString[buffer_id]);
		seq_puts(m, "\n=============== [CMDQ] Dump Command Buffer END ===============\n\n\n");
	}


#ifdef CMDQ_PWR_AWARE
	/* note for constatnt format (without a % substitution),
	 * use seq_puts to speed up outputs
	 */
	seq_puts(m, "====== Clock Status =======\n");
	cmdq_get_func()->printStatusSeqClock(m);
#endif

	seq_puts(m, "====== DMA Mask Status =======\n");
	seq_printf(m, "dma_set_mask result:%d\n",
		cmdq_dev_get_dma_mask_result());

	seq_puts(m, "====== SRAM Usage Status =======\n");
	index = 0;
	list_for_each_entry(p_sram_chunk, &cmdq_ctx.sram_allocated_list,
		list_node) {
		seq_printf(m, "SRAM Chunk(%d)-32bit unit: start:0x%x count:%zu Name:%s\n",
			index, p_sram_chunk->start_offset, p_sram_chunk->count,
			p_sram_chunk->owner);
		index++;
	}
#ifdef CMDQ_TIMER_ENABLE
	cmdq_dev_enable_gce_clock(true);
	seq_printf(m, "==Delay Task TPR_MASK:0x%08x size:%u started:%d pa:%pa va:0x%p sram:%u\n",
		CMDQ_REG_GET32(CMDQ_TPR_MASK), cmdq_delay_thd_cmd.buffer_size,
		cmdq_delay_thd_started, &cmdq_delay_thd_cmd.mva_base,
		cmdq_delay_thd_cmd.p_va_base, cmdq_delay_thd_cmd.sram_base);
	cmdq_dev_enable_gce_clock(false);
#endif

	/* call to dump other infos */
	blocking_notifier_call_chain(&cmdq_status_dump_notifier, 0, m);

	/* finally dump all active handles */
	mutex_lock(&cmdq_handle_list_mutex);
	index = 0;

	list_for_each_entry(handle, &cmdq_ctx.handle_active, list_entry) {
		u32 irq = 0;

		seq_printf(m,
			"====== Handle(%d) 0x%p pkt 0x%p Usage =======\n",
			index, handle, handle->pkt);

		seq_printf(m, "State:%d Size:%zu\n",
			handle->state, handle->pkt->cmd_buf_size);

		list_for_each_entry(buf, &handle->pkt->buf, list_entry) {
			seq_printf(m, "va:0x%p pa:%pa\n",
				buf->va_base, &buf->pa_base);
		}

		if (handle->use_sram_buffer)
			seq_printf(m, "USE SRAM BUFFER sram base address:%u\n",
			handle->sram_base);

		client = cmdq_clients[handle->thread];
		cmdq_task_get_thread_irq(client->chan, &irq);

		seq_printf(m,
			"Scenario:%d Priority:%d Flag:0x%llx va end:0x%p IRQ:0x%x\n",
			handle->scenario, 0, handle->engineFlag,
			handle->cmd_end, irq);

		seq_printf(m,
			"Reorder:%d Trigger:%lld IRQ:0x%llx Wait:%lld Wake Up:%lld\n",
			handle->reorder,
			handle->trigger, handle->gotIRQ,
			handle->beginWait, handle->wakedUp);

		++index;
	}
	seq_printf(m, "====== Total %d Handle =======\n", index);
	mutex_unlock(&cmdq_handle_list_mutex);

	for (index = 0; index < max_thread_count; index++)
		cmdq_core_print_thd_usage(m, v, index);

	return 0;
}

#if 0
static s32 cmdq_core_thread_exec_counter(const s32 thread)
{
#ifdef CMDQ_SECURE_PATH_SUPPORT
	return (!cmdq_get_func()->isSecureThread(thread)) ?
	    (CMDQ_GET_COOKIE_CNT(thread)) :
	    (cmdq_sec_get_secure_thread_exec_counter(thread));
#else
	return CMDQ_GET_COOKIE_CNT(thread);
#endif
}
#endif

static s32 cmdq_core_get_thread_id(s32 scenario)
{
	return cmdq_get_func()->getThreadID(scenario, false);
}

struct cmdqSecSharedMemoryStruct *cmdq_core_get_secure_shared_memory(void)
{
	return cmdq_ctx.hSecSharedMem;
}

static void cmdq_core_dump_thread(const struct cmdqRecStruct *handle,
	s32 thread, bool dump_irq, const char *tag)
{
	u32 value[15] = { 0 };

	if (thread == CMDQ_INVALID_THREAD)
		return;

	if (handle && handle->timeout_info) {
		value[0] = (u32)handle->timeout_info->curr_pc;
		value[1] = (u32)handle->timeout_info->end_addr;
		value[2] = handle->timeout_info->irq;
	} else {
		/* normal thread */
		value[0] = cmdq_core_get_pc(thread);
		value[1] = cmdq_core_get_end(thread);
		value[2] = CMDQ_REG_GET32(CMDQ_THR_IRQ_STATUS(thread));
	}

	value[3] = CMDQ_REG_GET32(CMDQ_THR_WAIT_TOKEN(thread));
	value[4] = CMDQ_GET_COOKIE_CNT(thread);
	value[5] = CMDQ_REG_GET32(CMDQ_THR_INST_CYCLES(thread));
	value[6] = CMDQ_REG_GET32(CMDQ_THR_CURR_STATUS(thread));
	value[7] = CMDQ_REG_GET32(CMDQ_THR_IRQ_ENABLE(thread));
	value[8] = CMDQ_REG_GET32(CMDQ_THR_ENABLE_TASK(thread));
	value[9] = CMDQ_REG_GET32(CMDQ_THR_WARM_RESET(thread));
	value[10] = CMDQ_REG_GET32(CMDQ_THR_SUSPEND_TASK(thread));
	value[11] = CMDQ_REG_GET32(CMDQ_THR_SECURITY(thread));
	value[12] = CMDQ_REG_GET32(CMDQ_THR_CFG(thread));
	value[13] = CMDQ_REG_GET32(CMDQ_THR_PREFETCH(thread));
	value[14] = CMDQ_REG_GET32(CMDQ_THR_INST_THRESX(thread));

	CMDQ_LOG(
		"[%s]===== Error Thread Status index:%d enabled:%d scenario:%d =====\n",
		tag, thread, value[8], cmdq_ctx.thread[thread].scenario);

	CMDQ_LOG(
		"[%s]PC:0x%08x End:0x%08x Wait Token:0x%08x IRQ:0x%x IRQ_EN:0x%x cookie:%u\n",
		tag, value[0], value[1], value[3], value[2], value[7],
		value[4]);
	/* TODO: support cookie */
#if 0
	CMDQ_LOG(
		"[%s]Curr Cookie:%d Wait Cookie:%d Next Cookie:%d Task Count:%d engineFlag:0x%llx\n",
		tag, value[4], pThread->waitCookie, pThread->nextCookie,
		pThread->taskCount, pThread->engineFlag);
#endif

#ifdef CMDQ_SECURE_PATH_SUPPORT
	cmdq_sec_dump_secure_thread_cookie(thread);
#endif

	CMDQ_LOG(
		"[%s]Timeout Cycle:%d Status:0x%x reset:0x%x Suspend:%d sec:%d cfg:%d prefetch:%d thrsex:%d\n",
		tag, value[5], value[6], value[9], value[10],
		value[11], value[12], value[13], value[14]);

	/* if pc match end and irq flag on, dump irq status */
	if (dump_irq && value[0] == value[1] && value[2] == 1)
		mt_irq_dump_status(cmdq_dev_get_irq_id());
}

void cmdq_core_dump_trigger_loop_thread(const char *tag)
{
	u32 i;
	const u32 max_thread_count = cmdq_dev_get_thread_count();
	u32 val;
	const u32 evt_rdma = CMDQ_EVENT_DISP_RDMA0_EOF;
	s32 static_id = cmdq_core_get_thread_id(CMDQ_SCENARIO_TRIGGER_LOOP);

	/* dump trigger loop */
	for (i = 0; i < max_thread_count; i++) {
		if (cmdq_ctx.thread[i].scenario !=
			CMDQ_SCENARIO_TRIGGER_LOOP &&
			i != static_id)
			continue;
		cmdq_core_dump_thread(NULL, i, false, tag);
		cmdq_core_dump_pc(NULL, i, tag);
		val = cmdqCoreGetEvent(evt_rdma);
		CMDQ_LOG("[%s]CMDQ_EVENT_DISP_RDMA0_EOF is %d\n", tag, val);
		break;
	}
}

static void cmdq_core_dump_cmd_mem(const u32 *cmd, s32 size)
{
	static char buf[128];
	int i;

	print_hex_dump(KERN_ERR, "[CMDQ]", DUMP_PREFIX_ADDRESS, 16, 4, cmd,
		size, false);
	CMDQ_LOG("==== TASK command buffer END\n");

	for (i = 0; i < size; i += CMDQ_INST_SIZE, cmd += 2) {
		cmdq_core_parse_instruction(cmd, buf, 128);
		CMDQ_LOG("%p: %s", cmd, buf);
	}
	CMDQ_LOG("TASK command buffer TRANSLATED END\n");
}

s32 cmdq_core_dump_sram_mem(u32 sram_base, u32 command_size)
{
	void *p_va_dest = NULL;
	dma_addr_t pa_dest = 0;
	s32 status = 0;
	u32 cpr_offset = CMDQ_INVALID_CPR_OFFSET;

	do {
		/* copy SRAM command to DRAM */
		cpr_offset = CMDQ_CPR_OFFSET(sram_base);
		CMDQ_LOG(
			"==Dump SRAM: size (%d) CPR OFFSET(0x%x), ADDR(0x%x)\n",
			command_size, cpr_offset, sram_base);
		p_va_dest = cmdq_core_alloc_hw_buffer_clt(cmdq_dev_get(),
			command_size, &pa_dest, GFP_KERNEL, CMDQ_CLT_CMDQ);
		if (!p_va_dest)
			break;
		status = cmdq_task_copy_from_sram(pa_dest, cpr_offset,
			command_size);
		if (status < 0) {
			CMDQ_ERR("%s copy from sram API failed:%d\n",
				__func__, status);
			break;
		}
		CMDQ_LOG("======Dump SRAM: size (%d) SRAM command START\n",
			command_size);
		cmdq_core_dump_cmd_mem(p_va_dest, command_size);
		CMDQ_LOG("======Dump SRAM command END\n");
	} while (0);

	if (p_va_dest)
		cmdq_core_free_hw_buffer_clt(cmdq_dev_get(),
			command_size, p_va_dest, pa_dest, CMDQ_CLT_CMDQ);

	return status;
}

void cmdq_core_turnon_first_dump(const struct cmdqRecStruct *task)
{
	if (cmdq_first_err.cmdqCount || !task)
		return;

	cmdq_first_err.flag = true;
	/* save kernel time, pid, and caller name */
	cmdq_first_err.callerPid = task->caller_pid;
	snprintf(cmdq_first_err.callerName, TASK_COMM_LEN, "%s",
		task->caller_name);
	cmdq_first_err.savetime = sched_clock();
	do_gettimeofday(&cmdq_first_err.savetv);
}

void cmdq_core_turnoff_first_dump(void)
{
	cmdq_first_err.flag = false;
}

void cmdq_core_reset_first_dump(void)
{
	memset(&cmdq_first_err, 0, sizeof(cmdq_first_err));
	cmdq_first_err.cmdqMaxSize = CMDQ_MAX_FIRSTERROR;
	cmdq_ctx.errNum = 0;
}

s32 cmdq_core_save_first_dump(const char *string, ...)
{
	int logLen;
	va_list argptr;
	char *buffer;
	unsigned long flags;

	if (!cmdq_first_err.flag)
		return -EFAULT;

	spin_lock_irqsave(&cmdq_first_err_lock, flags);

	if (!cmdq_first_err.cmdqString) {
		cmdq_first_err.cmdqString = kmalloc(CMDQ_MAX_FIRSTERROR,
			GFP_ATOMIC);
		if (!cmdq_first_err.cmdqString) {
			spin_unlock_irqrestore(&cmdq_first_err_lock, flags);
			cmdq_first_err.flag = false;
			CMDQ_LOG("[ERR] Error0 dump buffer allocate fail\n");
			return -ENOMEM;
		}
	}

	va_start(argptr, string);
	buffer = cmdq_first_err.cmdqString + cmdq_first_err.cmdqCount;
	logLen = vsnprintf(buffer, cmdq_first_err.cmdqMaxSize, string, argptr);
	cmdq_first_err.cmdqMaxSize -= logLen;
	cmdq_first_err.cmdqCount += logLen;

	spin_unlock_irqrestore(&cmdq_first_err_lock, flags);

	if (cmdq_first_err.cmdqMaxSize <= 0) {
		cmdq_first_err.flag = false;
		CMDQ_LOG("[ERR] Error0 dump saving buffer is full\n");
	}
	va_end(argptr);
	return 0;
}

const char *cmdq_core_query_first_err_mod(void)
{
	return cmdq_first_err_mod;
}

void cmdq_core_hex_dump_to_buffer(const void *buf, size_t len, int rowsize,
	int groupsize, char *linebuf, size_t linebuflen)
{
	const u8 *ptr = buf;
	u8 ch;
	int j, lx = 0;

	if (rowsize != 16 && rowsize != 32)
		rowsize = 16;

	if (!len)
		goto nil;
	if (len > rowsize)	/* limit to one line at a time */
		len = rowsize;
	if ((len % groupsize) != 0)	/* no mixed size output */
		groupsize = 1;

	switch (groupsize) {
	case 8:{
			const u64 *ptr8 = buf;
			int ngroups = len / groupsize;

			for (j = 0; j < ngroups; j++)
				lx += scnprintf(linebuf + lx, linebuflen - lx,
				"%s%16.16llx", j ? " " : "",
				(unsigned long long)*(ptr8 + j));
			break;
		}

	case 4:{
			const u32 *ptr4 = buf;
			int ngroups = len / groupsize;

			for (j = 0; j < ngroups; j++)
				lx += scnprintf(linebuf + lx, linebuflen - lx,
				"%s%8.8x", j ? " " : "", *(ptr4 + j));
			break;
		}

	case 2:{
			const u16 *ptr2 = buf;
			int ngroups = len / groupsize;

			for (j = 0; j < ngroups; j++)
				lx += scnprintf(linebuf + lx, linebuflen - lx,
				"%s%4.4x", j ? " " : "", *(ptr2 + j));
			break;
		}

	default:
		for (j = 0; (j < len) && (lx + 3) <= linebuflen; j++) {
			ch = ptr[j];
			linebuf[lx++] = hex_asc_hi(ch);
			linebuf[lx++] = hex_asc_lo(ch);
			linebuf[lx++] = ' ';
		}
		if (j)
			lx--;
		break;
	}
nil:
	linebuf[lx++] = '\0';
}

void cmdq_core_save_hex_first_dump(const char *prefix_str,
	int rowsize, int groupsize, const void *buf, size_t len)
{
	const u8 *ptr = buf;
	int i, linelen, remaining = len;
	unsigned char linebuf[32 * 3 + 2 + 32 + 1];
	int logLen;
	char *pBuffer;

	if (cmdq_first_err.flag == false)
		return;

	if (rowsize != 16 && rowsize != 32)
		rowsize = 16;

	for (i = 0; i < len; i += rowsize) {
		linelen = min(remaining, rowsize);
		remaining -= rowsize;

		cmdq_core_hex_dump_to_buffer(ptr + i, linelen, rowsize,
			groupsize, linebuf, sizeof(linebuf));

		pBuffer = cmdq_first_err.cmdqString +
			cmdq_first_err.cmdqCount;
		logLen = snprintf(pBuffer, cmdq_first_err.cmdqMaxSize,
			"%s%p:%s\n", prefix_str, ptr + i, linebuf);
		cmdq_first_err.cmdqMaxSize -= logLen;
		cmdq_first_err.cmdqCount += logLen;

		if (cmdq_first_err.cmdqMaxSize <= 0) {
			cmdq_first_err.flag = false;
			CMDQ_LOG("[ERR] Error0 dump saving buffer is full\n");
		}
	}
}

void *cmdq_core_alloc_hw_buffer_clt(struct device *dev, size_t size,
	dma_addr_t *dma_handle, const gfp_t flag, enum CMDQ_CLT_ENUM clt)
{
	s32 alloc_cnt, alloc_max = 1 << 10;
	void *ret = cmdq_core_alloc_hw_buffer(dev, size, dma_handle, flag);

	if (!ret)
		return NULL;

	alloc_cnt = atomic_inc_return(&cmdq_alloc_cnt[CMDQ_CLT_MAX]);
	alloc_cnt = atomic_inc_return(&cmdq_alloc_cnt[clt]);
	if (alloc_cnt > alloc_max)
		CMDQ_ERR(
			"clt:%u MDP(1):%u CMDQ(2):%u GNRL(3):%u DISP(4):%u TTL:%u\n",
			clt, atomic_read(&cmdq_alloc_cnt[1]),
			atomic_read(&cmdq_alloc_cnt[2]),
			atomic_read(&cmdq_alloc_cnt[3]),
			atomic_read(&cmdq_alloc_cnt[4]),
			atomic_read(&cmdq_alloc_cnt[5]));
	return ret;
}

void *cmdq_core_alloc_hw_buffer(struct device *dev, size_t size,
	dma_addr_t *dma_handle, const gfp_t flag)
{
	void *pVA;
	dma_addr_t PA;
	CMDQ_TIME alloc_cost = 0;

	do {
		PA = 0;
		pVA = NULL;

		CMDQ_PROF_START(current->pid, __func__);
		CMDQ_PROF_MMP(cmdq_mmp_get_event()->alloc_buffer,
			MMPROFILE_FLAG_START, current->pid, size);
		alloc_cost = sched_clock();

		pVA = dma_alloc_coherent(dev, size, &PA, flag);

		alloc_cost = sched_clock() - alloc_cost;
		CMDQ_PROF_MMP(cmdq_mmp_get_event()->alloc_buffer,
			MMPROFILE_FLAG_END, current->pid, alloc_cost);
		CMDQ_PROF_END(current->pid, __func__);

		if (alloc_cost > CMDQ_PROFILE_LIMIT_1) {
#if defined(__LP64__) || defined(_LP64)
			CMDQ_LOG(
				"[warn] alloc buffer (size:%zu) cost %llu us > %ums\n",
				size, alloc_cost / 1000,
				CMDQ_PROFILE_LIMIT_1 / 1000000);
#else
			CMDQ_LOG(
				"[warn] alloc buffer (size:%zu) cost %llu us > %llums\n",
				size, div_s64(alloc_cost, 1000),
				div_s64(CMDQ_PROFILE_LIMIT_1, 1000000));
#endif
		}

	} while (0);

	*dma_handle = PA;

	CMDQ_VERBOSE("%s va:0x%p pa:%pa paout:%pa\n",
		__func__, pVA, &PA, &(*dma_handle));

	return pVA;
}

void cmdq_core_free_hw_buffer_clt(struct device *dev, size_t size,
	void *cpu_addr, dma_addr_t dma_handle, enum CMDQ_CLT_ENUM clt)
{
	atomic_dec(&cmdq_alloc_cnt[CMDQ_CLT_MAX]);
	atomic_dec(&cmdq_alloc_cnt[clt]);
	cmdq_core_free_hw_buffer(dev, size, cpu_addr, dma_handle);
}

void cmdq_core_free_hw_buffer(struct device *dev, size_t size,
	void *cpu_addr, dma_addr_t dma_handle)
{
	CMDQ_TIME free_cost = 0;

	CMDQ_VERBOSE("%s va:0x%p pa:%pa\n",
		__func__, cpu_addr, &dma_handle);

	free_cost = sched_clock();
	dma_free_coherent(dev, size, cpu_addr, dma_handle);
	free_cost = sched_clock() - free_cost;

	if (free_cost > CMDQ_PROFILE_LIMIT_1) {
#if defined(__LP64__) || defined(_LP64)
		CMDQ_LOG(
			"[warn] free buffer (size:%zu) cost %llu us > %ums\n",
			size, free_cost/1000, CMDQ_PROFILE_LIMIT_1 / 1000000);
#else
		CMDQ_LOG(
			"[warn] free buffer (size:%zu) cost %llu us > %llums\n",
			size, div_s64(free_cost, 1000),
			div_s64(CMDQ_PROFILE_LIMIT_1, 1000000));
#endif
	}
}

void cmdq_core_dump_sram(void)
{
	struct SRAMChunk *p_sram_chunk;
	s32 index = 0;

	list_for_each_entry(p_sram_chunk, &cmdq_ctx.sram_allocated_list,
		list_node) {
		CMDQ_LOG(
			"SRAM Chunk(%d)-32bit unit: start:0x%x count:%zu Name:%s\n",
			index, p_sram_chunk->start_offset,
			p_sram_chunk->count, p_sram_chunk->owner);
		index++;
	}
}

s32 cmdq_core_alloc_sram_buffer(size_t size,
	const char *owner_name, u32 *out_cpr_offset)
{
	u32 cpr_offset = 0;
	struct SRAMChunk *p_sram_chunk, *p_last_chunk;
	/* Normalize from byte unit to 32bit unit */
	size_t normalized_count = size / sizeof(u32);

	/* Align allocated buffer to 64bit due to instruction alignment */
	if (normalized_count % 2 != 0)
		normalized_count++;

	/* Get last entry to calculate new SRAM start address */
	if (!list_empty(&cmdq_ctx.sram_allocated_list)) {
		p_last_chunk = list_last_entry(&cmdq_ctx.sram_allocated_list,
			struct SRAMChunk, list_node);
		cpr_offset = p_last_chunk->start_offset + p_last_chunk->count;
	}

	if (cpr_offset + normalized_count > cmdq_dts.cpr_size) {
		CMDQ_LOG(
			"[WARN]SRAM count is out of memory, start:%u want:%zu owner:%s\n",
			cpr_offset, normalized_count, owner_name);
		cmdq_core_dump_sram();
		return -ENOMEM;
	}

	p_sram_chunk = kzalloc(sizeof(struct SRAMChunk), GFP_KERNEL);
	if (p_sram_chunk) {
		p_sram_chunk->start_offset = cpr_offset;
		p_sram_chunk->count = normalized_count;
		strncpy(p_sram_chunk->owner, owner_name,
			sizeof(p_sram_chunk->owner) - 1);
		list_add_tail(&(p_sram_chunk->list_node),
			&cmdq_ctx.sram_allocated_list);
		cmdq_ctx.allocated_sram_count += normalized_count;
		CMDQ_LOG(
			"SRAM Chunk New-32bit unit: start:0x%x count:%zu Name:%s\n",
			p_sram_chunk->start_offset, p_sram_chunk->count,
			p_sram_chunk->owner);
	}

	*out_cpr_offset = cpr_offset;
	return 0;
}

void cmdq_core_free_sram_buffer(u32 cpr_offset, size_t size)
{
	struct SRAMChunk *p_sram_chunk;
	bool released = false;
	/* Normalize from byte unit to 32bit unit */
	size_t normalized_count = size / sizeof(u32);

	/* Find and remove */
	list_for_each_entry(p_sram_chunk, &cmdq_ctx.sram_allocated_list,
		list_node) {
		if (p_sram_chunk->start_offset == cpr_offset &&
			p_sram_chunk->count == normalized_count) {
			CMDQ_MSG(
				"SRAM Chunk Free-32bit unit: start:0x%x count:%zu Name:%s\n",
				p_sram_chunk->start_offset,
				p_sram_chunk->count, p_sram_chunk->owner);
			list_del_init(&(p_sram_chunk->list_node));
			cmdq_ctx.allocated_sram_count -= normalized_count;
			released = true;
			break;
		}
	}

	if (!released) {
		CMDQ_ERR(
			"SRAM Chunk Free-32bit unit: start:0x%x count:%zu failed\n",
			cpr_offset, normalized_count);
		cmdq_core_dump_sram();
	}
}

size_t cmdq_core_get_free_sram_size(void)
{
	return (cmdq_dts.cpr_size - cmdq_ctx.allocated_sram_count) *
		sizeof(u32);
}

size_t cmdq_core_get_cpr_cnt(void)
{
	return cmdq_dts.cpr_size;
}

#ifdef CMDQ_TIMER_ENABLE
static s32 cmdq_copy_delay_to_sram(void)
{
	u32 cpr_offset = 0;
	dma_addr_t pa = cmdq_delay_thd_cmd.mva_base;
	u32 buffer_size = cmdq_delay_thd_cmd.buffer_size;

	if (cmdq_delay_thd_cmd.sram_base == 0)
		return 0;

	cpr_offset = CMDQ_CPR_OFFSET(cmdq_delay_thd_cmd.sram_base);
	if (cmdq_task_copy_to_sram(pa, cpr_offset, buffer_size) < 0) {
		CMDQ_ERR("DELAY: copy delay thread to SRAM failed!\n");
		return -EFAULT;
	}
	CMDQ_MSG("Copy delay thread:%pa size:%u cpr_offset:0x%x\n",
		&pa, buffer_size, cpr_offset);
	return 0;
}

static s32 cmdq_delay_thread_init(void)
{
	static u32 sram_task_size;
	static bool use_sram = true;

	void *p_delay_thread_buffer = NULL;
	void *p_va = NULL;
	dma_addr_t pa = 0;
	u32 buffer_size = 0;
	u32 cpr_offset = 0;
	u32 free_sram_size = cmdq_core_get_free_sram_size();

	memset(&cmdq_delay_thd_cmd, 0x0, sizeof(cmdq_delay_thd_cmd));

	if (!sram_task_size) {
		if (cmdq_task_create_delay_thread_sram(&p_delay_thread_buffer,
			&sram_task_size, &cpr_offset) < 0) {
			CMDQ_LOG(
				"[DelayThread]create delay thread in sram failed free:%u sram size:%u\n",
				free_sram_size, sram_task_size);
			use_sram = false;
		} else {
			CMDQ_LOG("[DelayThread]sram task size:%u free:%u\n",
				sram_task_size, free_sram_size);
			if (sram_task_size > free_sram_size)
				use_sram = false;
			else
				buffer_size = sram_task_size;
		}
	}

	if (!use_sram) {
		if (cmdq_task_create_delay_thread_dram(&p_delay_thread_buffer,
			&buffer_size) < 0) {
			CMDQ_ERR(
				"[DelayThread]create delay thread in dram failed!\n");
			return -EINVAL;
		}
	} else if (!p_delay_thread_buffer) {
		if (cmdq_task_create_delay_thread_sram(&p_delay_thread_buffer,
			&buffer_size, &cpr_offset) < 0) {
			CMDQ_ERR(
				"[DelayThread]create delay thread in sram failed, free:%u sram size:%u\n",
				free_sram_size, sram_task_size);
			return -EINVAL;
		}
	}

	CMDQ_LOG(
		"[DelayThread]create delay thread task in %s task size:%u sram size:%u\n",
		use_sram ? "SRAM" : "DRAM", buffer_size, free_sram_size);

	p_va = cmdq_core_alloc_hw_buffer(cmdq_dev_get(), buffer_size, &pa,
		GFP_KERNEL);

	memcpy(p_va, p_delay_thread_buffer, buffer_size);

	CMDQ_MSG("Set delay thread CMD START:%pa size:%u cpr_offset:0x%x\n",
		&pa, buffer_size, cpr_offset);

	cmdq_delay_thd_cmd.mva_base = pa;
	cmdq_delay_thd_cmd.p_va_base = p_va;
	cmdq_delay_thd_cmd.buffer_size = buffer_size;
	if (cpr_offset > 0) {
		cmdq_delay_thd_cmd.sram_base = CMDQ_SRAM_ADDR(cpr_offset);
		cmdq_copy_delay_to_sram();
	}

	kfree(p_delay_thread_buffer);
	cmdq_delay_thd_inited = true;
	cmdq_delay_thd_started = false;
	return 0;
}

static s32 cmdq_delay_thread_start(void)
{
	bool enable_prefetch;
	int thread_priority;
	u32 end_address;
	const s32 thread = CMDQ_DELAY_THREAD_ID;
	unsigned long flags;
	struct cmdq_client *client = cmdq_clients[thread];

	spin_lock_irqsave(&cmdq_delay_thd_lock, flags);
	if (!cmdq_delay_thd_inited || cmdq_delay_thd_started ||
		atomic_read(&cmdq_thread_usage) < 1) {
		spin_unlock_irqrestore(&cmdq_delay_thd_lock, flags);
		return 0;
	}

	CMDQ_MSG("EXEC: new delay thread:%d\n", thread);
	if (cmdq_mbox_thread_reset((void *)client->chan) < 0) {
		spin_unlock_irqrestore(&cmdq_delay_thd_lock, flags);
		return -EFAULT;
	}

	CMDQ_REG_SET32(CMDQ_THR_INST_CYCLES(thread), 0);

	enable_prefetch = cmdq_core_get_thread_prefetch_size(thread) > 0;
	if (enable_prefetch) {
		CMDQ_MSG("EXEC: set delay thread:%d enable prefetch size:%d\n",
			thread, cmdq_core_get_thread_prefetch_size(thread));
		CMDQ_REG_SET32(CMDQ_THR_PREFETCH(thread), 0x1);
	}

	thread_priority = cmdq_get_func()->priority(CMDQ_SCENARIO_TIMER_LOOP);

	if (cmdq_delay_thd_cmd.sram_base > 0) {
		CMDQ_MSG(
			"EXEC: set delay thread(%d) in SRAM sram_addr:%u qos:%d\n",
			 thread, cmdq_delay_thd_cmd.sram_base, thread_priority);

		end_address = cmdq_delay_thd_cmd.sram_base +
			cmdq_delay_thd_cmd.buffer_size;
		CMDQ_REG_SET32(CMDQ_THR_END_ADDR(thread), end_address);
		CMDQ_REG_SET32(CMDQ_THR_CURR_ADDR(thread),
			cmdq_delay_thd_cmd.sram_base);
	} else {
		CMDQ_MSG("EXEC: set delay thread:%d pc:%pa qos:%d\n",
			 thread, &cmdq_delay_thd_cmd.mva_base,
			 thread_priority);

		end_address = CMDQ_PHYS_TO_AREG(cmdq_delay_thd_cmd.mva_base +
			cmdq_delay_thd_cmd.buffer_size);
		CMDQ_REG_SET32(CMDQ_THR_END_ADDR(thread), end_address);
		CMDQ_REG_SET32(CMDQ_THR_CURR_ADDR(thread),
			CMDQ_PHYS_TO_AREG(cmdq_delay_thd_cmd.mva_base));
	}

	/* set thread priority, bit 0-2 for priority level; */
	CMDQ_REG_SET32(CMDQ_THR_CFG(thread), thread_priority & 0x7);

	/* For loop thread, do not enable timeout */
	CMDQ_REG_SET32(CMDQ_THR_IRQ_ENABLE(thread), 0x011);

	/* enable thread */
	CMDQ_REG_SET32(CMDQ_THR_ENABLE_TASK(thread), 0x01);

	cmdq_delay_thd_started = true;

	spin_unlock_irqrestore(&cmdq_delay_thd_lock, flags);

	return 0;
}

static s32 cmdq_delay_thread_stop(void)
{
	unsigned long flags;
	const s32 thread = CMDQ_DELAY_THREAD_ID;
	struct cmdq_client *client = cmdq_clients[thread];

	spin_lock_irqsave(&cmdq_delay_thd_lock, flags);
	if (!cmdq_delay_thd_started ||
		atomic_read(&cmdq_thread_usage) > 1) {
		spin_unlock_irqrestore(&cmdq_delay_thd_lock, flags);
		return 0;
	}

	if (cmdq_mbox_thread_suspend((void *)client->chan) < 0) {
		spin_unlock_irqrestore(&cmdq_delay_thd_lock, flags);
		return -EFAULT;
	}

	cmdq_mbox_thread_disable((void *)client->chan);

	cmdq_delay_thd_started = false;
	spin_unlock_irqrestore(&cmdq_delay_thd_lock, flags);
	CMDQ_MSG("EXEC: stop delay thread:%d\n", thread);

	return 0;
}

static void cmdq_delay_thread_deinit(void)
{
	cmdq_core_free_hw_buffer(cmdq_dev_get(),
		cmdq_delay_thd_cmd.buffer_size,
		cmdq_delay_thd_cmd.p_va_base,
		cmdq_delay_thd_cmd.mva_base);
}

void cmdq_delay_dump_thread(bool dump_sram)
{
	cmdq_core_dump_thread(NULL, CMDQ_DELAY_THREAD_ID, false, "INFO");
	CMDQ_LOG(
		"==Delay Thread Task, size:%u started:%d pa:%pa va:0x%p sram:%u\n",
		cmdq_delay_thd_cmd.buffer_size, cmdq_delay_thd_started,
		&cmdq_delay_thd_cmd.mva_base, cmdq_delay_thd_cmd.p_va_base,
		cmdq_delay_thd_cmd.sram_base);
	CMDQ_LOG("Dump TPR_MASK:0x%08x\n", CMDQ_REG_GET32(CMDQ_TPR_MASK));
	cmdq_core_dump_cmd_mem(cmdq_delay_thd_cmd.p_va_base,
		cmdq_delay_thd_cmd.buffer_size);
	CMDQ_LOG("==Delay Thread Task command END\n");
	if (dump_sram)
		cmdq_core_dump_sram_mem(cmdq_delay_thd_cmd.sram_base,
			cmdq_delay_thd_cmd.buffer_size);
}

u32 cmdq_core_get_delay_start_cpr(void)
{
	return cmdq_ctx.delay_cpr_start;
}

s32 cmdq_delay_get_id_by_scenario(enum CMDQ_SCENARIO_ENUM scenario)
{
	s32 delay_id = -1;

	switch (scenario) {
	case CMDQ_SCENARIO_PRIMARY_DISP:
	/* HACK: force debug into 0/1 thread */
	case CMDQ_SCENARIO_DEBUG_PREFETCH:
		delay_id = 0;
		break;
	case CMDQ_SCENARIO_TRIGGER_LOOP:
		delay_id = 1;
		break;
	case CMDQ_SCENARIO_DEBUG:
	case CMDQ_SCENARIO_DEBUG_MDP:
		delay_id = 2;
		break;
	default:
		delay_id = -1;
		break;
	}

	return delay_id;
}
#endif

int cmdqCoreAllocWriteAddress(u32 count, dma_addr_t *paStart,
	enum CMDQ_CLT_ENUM clt)
{
	unsigned long flags;
	struct WriteAddrStruct *pWriteAddr = NULL;
	int status = 0;

	do {
		if (!paStart) {
			CMDQ_ERR("invalid output argument\n");
			status = -EINVAL;
			break;
		}
		*paStart = 0;

		if (!count || count > CMDQ_MAX_WRITE_ADDR_COUNT) {
			CMDQ_ERR("invalid alloc write addr count:%u max:%u\n",
				count, (u32)CMDQ_MAX_WRITE_ADDR_COUNT);
			status = -EINVAL;
			break;
		}

		pWriteAddr = kzalloc(sizeof(struct WriteAddrStruct),
			GFP_KERNEL);
		if (!pWriteAddr) {
			CMDQ_ERR("failed to alloc WriteAddrStruct\n");
			status = -ENOMEM;
			break;
		}
		memset(pWriteAddr, 0, sizeof(struct WriteAddrStruct));

		pWriteAddr->count = count;
		pWriteAddr->va = cmdq_core_alloc_hw_buffer_clt(cmdq_dev_get(),
			count * sizeof(u32), &(pWriteAddr->pa), GFP_KERNEL,
			clt);
		if (current)
			pWriteAddr->user = current->pid;

		if (!pWriteAddr->va) {
			CMDQ_ERR("failed to alloc write buffer\n");
			status = -ENOMEM;
			break;
		}

		/* clear buffer content */
		do {
			u32 *pInt = (u32 *) pWriteAddr->va;
			int i = 0;

			for (i = 0; i < count; ++i) {
				*(pInt + i) = 0xcdcdabab;
				/* make sure instructions are really in DRAM */
				mb();
				/* make sure instructions are really in DRAM */
				smp_mb();
			}
		} while (0);

		/* assign output pa */
		*paStart = pWriteAddr->pa;

		spin_lock_irqsave(&cmdq_write_addr_lock, flags);
		list_add_tail(&(pWriteAddr->list_node),
			&cmdq_ctx.writeAddrList);
		spin_unlock_irqrestore(&cmdq_write_addr_lock, flags);

		status = 0;

		atomic_inc(&cmdq_ctx.write_addr_cnt);

	} while (0);

	if (status != 0) {
		/* release resources */
		if (pWriteAddr && pWriteAddr->va) {
			cmdq_core_free_hw_buffer_clt(cmdq_dev_get(),
				sizeof(u32) * pWriteAddr->count,
				pWriteAddr->va, pWriteAddr->pa, clt);
			memset(pWriteAddr, 0, sizeof(struct WriteAddrStruct));
		}

		kfree(pWriteAddr);
		pWriteAddr = NULL;
	}

	return status;
}

u32 cmdqCoreReadWriteAddress(dma_addr_t pa)
{
	struct list_head *p = NULL;
	struct WriteAddrStruct *pWriteAddr = NULL;
	s32 offset = 0;
	u32 value = 0;
	unsigned long flags;

	/* search for the entry */
	spin_lock_irqsave(&cmdq_write_addr_lock, flags);
	list_for_each(p, &cmdq_ctx.writeAddrList) {
		pWriteAddr = list_entry(p, struct WriteAddrStruct, list_node);
		if (!pWriteAddr)
			continue;

		offset = pa - pWriteAddr->pa;

		if (offset >= 0 && (offset / sizeof(u32)) <
			pWriteAddr->count) {
			CMDQ_VERBOSE(
				"%s input:%pa got offset:%d va:%p pa_start:%pa\n",
				__func__, &pa, offset,
				pWriteAddr->va + offset, &pWriteAddr->pa);
			value = *((u32 *)(pWriteAddr->va + offset));
			CMDQ_VERBOSE(
				"%s found offset:%d va:%p value:0x%08x\n",
				__func__, offset, pWriteAddr->va + offset,
				value);
			break;
		}
	}
	spin_unlock_irqrestore(&cmdq_write_addr_lock, flags);

	return value;
}

void cmdqCoreReadWriteAddressBatch(u32 *addrs, u32 count, u32 *val_out)
{
	struct WriteAddrStruct *waddr, *cur_waddr = NULL;
	unsigned long flags = 0L;
	u32 i;
	dma_addr_t pa;

	/* search for the entry */
	spin_lock_irqsave(&cmdq_write_addr_lock, flags);

	CMDQ_PROF_MMP(cmdq_mmp_get_event()->read_reg,
		MMPROFILE_FLAG_START, ((unsigned long)addrs),
		(u32)atomic_read(&cmdq_ctx.write_addr_cnt));

	for (i = 0; i < count; i++) {
		pa = addrs[i];

		if (!cur_waddr || pa < cur_waddr->pa ||
			pa >= cur_waddr->pa + cur_waddr->count * sizeof(u32)) {
			cur_waddr = NULL;
			list_for_each_entry(waddr, &cmdq_ctx.writeAddrList,
				list_node) {

				if (pa < waddr->pa || pa >= waddr->pa +
					waddr->count * sizeof(u32))
					continue;
				cur_waddr = waddr;
				break;
			}
		}

		if (cur_waddr)
			val_out[i] = *((u32 *)(cur_waddr->va +
				(pa - cur_waddr->pa)));
		else
			val_out[i] = 0;
	}

	CMDQ_PROF_MMP(cmdq_mmp_get_event()->read_reg,
		MMPROFILE_FLAG_END, ((unsigned long)addrs), count);

	spin_unlock_irqrestore(&cmdq_write_addr_lock, flags);

}

u32 cmdqCoreWriteWriteAddress(dma_addr_t pa, u32 value)
{
	struct list_head *p = NULL;
	struct WriteAddrStruct *pWriteAddr = NULL;
	s32 offset = 0;
	unsigned long flags;
	char long_msg[CMDQ_LONGSTRING_MAX];
	u32 msg_offset;
	s32 msg_max_size;

	/* search for the entry */
	spin_lock_irqsave(&cmdq_write_addr_lock, flags);
	list_for_each(p, &cmdq_ctx.writeAddrList) {
		pWriteAddr = list_entry(p, struct WriteAddrStruct, list_node);
		if (!pWriteAddr)
			continue;

		offset = pa - pWriteAddr->pa;

		/* note it is 64 bit length for u32 variable in 64 bit kernel
		 * use sizeof(u_log) to check valid offset range
		 */
		if (offset >= 0 && (offset / sizeof(u32)) <
			pWriteAddr->count) {
			cmdq_long_string_init(false, long_msg, &msg_offset,
				&msg_max_size);
			cmdq_long_string(long_msg, &msg_offset, &msg_max_size,
				"%s input:%pa", __func__, &pa);
			cmdq_long_string(long_msg, &msg_offset, &msg_max_size,
				" got offset:%d va:%p pa_start:%pa value:0x%08x\n",
				offset, pWriteAddr->va + offset,
				&pWriteAddr->pa, value);
			CMDQ_VERBOSE("%s", long_msg);

			*((u32 *)(pWriteAddr->va + offset)) = value;
			break;
		}
	}
	spin_unlock_irqrestore(&cmdq_write_addr_lock, flags);

	return value;
}

int cmdqCoreFreeWriteAddress(dma_addr_t paStart, enum CMDQ_CLT_ENUM clt)
{
	struct list_head *p, *n = NULL;
	struct WriteAddrStruct *pWriteAddr = NULL;
	bool foundEntry;
	unsigned long flags;

	foundEntry = false;

	/* search for the entry */
	spin_lock_irqsave(&cmdq_write_addr_lock, flags);
	list_for_each_safe(p, n, &cmdq_ctx.writeAddrList) {
		pWriteAddr = list_entry(p, struct WriteAddrStruct, list_node);
		if (pWriteAddr && pWriteAddr->pa == paStart) {
			list_del(&(pWriteAddr->list_node));
			foundEntry = true;
			break;
		}
	}
	spin_unlock_irqrestore(&cmdq_write_addr_lock, flags);

	/* when list is not empty, we always get a entry
	 * even we don't found a valid entry
	 * use foundEntry to confirm search result
	 */
	if (!foundEntry) {
		CMDQ_ERR("%s no matching entry, paStart:%pa\n",
			__func__, &paStart);
		return -EINVAL;
	}

	/* release resources */
	if (pWriteAddr->va) {
		cmdq_core_free_hw_buffer_clt(cmdq_dev_get(),
			sizeof(u32) * pWriteAddr->count,
			pWriteAddr->va, pWriteAddr->pa, clt);
		memset(pWriteAddr, 0xda, sizeof(struct WriteAddrStruct));
	}

	kfree(pWriteAddr);
	pWriteAddr = NULL;

	atomic_dec(&cmdq_ctx.write_addr_cnt);

	return 0;
}

void cmdq_core_init_dts_data(void)
{
	u32 i;

	memset(&cmdq_dts_data, 0x0, sizeof(cmdq_dts_data));

	for (i = 0; i < CMDQ_SYNC_TOKEN_MAX; i++) {
		if (i <= CMDQ_MAX_HW_EVENT_COUNT) {
			/* GCE HW evevt */
			cmdq_dts_data.eventTable[i] =
				CMDQ_SYNC_TOKEN_INVALID - 1 - i;
		} else {
			/* GCE SW evevt */
			cmdq_dts_data.eventTable[i] = i;
		}
	}
}

struct cmdqDTSDataStruct *cmdq_core_get_dts_data(void)
{
	return &cmdq_dts_data;
}

u32 cmdq_core_get_thread_prefetch_size(const s32 thread)
{
	if (thread >= 0 && thread < cmdq_dev_get_thread_count())
		return cmdq_dts.prefetch_size[thread];
	return 0;
}

void cmdq_core_dump_dts_setting(void)
{
	u32 index;
	const u32 max_thread_count = cmdq_dev_get_thread_count();

	CMDQ_LOG("[DTS] Prefetch Thread Count:%d/%u\n",
		cmdq_dts.prefetch_thread_count, max_thread_count);
	CMDQ_LOG("[DTS] Prefetch Size of Thread:\n");
	for (index = 0; index < cmdq_dts.prefetch_thread_count &&
		index < max_thread_count; index++)
		CMDQ_LOG("	Thread[%d]:%d\n",
			index, cmdq_dts.prefetch_size[index]);
}

void cmdq_core_set_event_table(enum cmdq_event event, const s32 value)
{
	if (event >= 0 && event < CMDQ_SYNC_TOKEN_MAX)
		cmdq_dts_data.eventTable[event] = value;
}

s32 cmdq_core_get_event_value(enum cmdq_event event)
{
	if (event < 0 || event >= CMDQ_SYNC_TOKEN_MAX)
		return -EINVAL;

	return cmdq_dts_data.eventTable[event];
}

static s32 cmdq_core_reverse_event_enum(const u32 value)
{
	u32 evt;

	for (evt = 0; evt < CMDQ_SYNC_TOKEN_MAX; evt++)
		if (value == cmdq_dts_data.eventTable[evt])
			return evt;

	return CMDQ_SYNC_TOKEN_INVALID;
}

const char *cmdq_core_get_event_name_enum(enum cmdq_event event)
{
	const char *eventName = "CMDQ_EVENT_UNKNOWN";
	struct cmdq_event_table *events = cmdq_event_get_table();
	u32 table_size = cmdq_event_get_table_size();
	u32 i = 0;

	for (i = 0; i < table_size; i++) {
		if (events[i].event == event)
			return events[i].event_name;
	}

	return eventName;
}

const char *cmdq_core_get_event_name(u32 hw_event)
{
	const s32 event_enum = cmdq_core_reverse_event_enum(hw_event);

	return cmdq_core_get_event_name_enum(event_enum);
}

void cmdqCoreClearEvent(enum cmdq_event event)
{
	s32 eventValue = cmdq_core_get_event_value(event);

	CMDQ_MSG("clear event %d\n", eventValue);
	CMDQ_REG_SET32(CMDQ_SYNC_TOKEN_UPD, eventValue);
}

void cmdqCoreSetEvent(enum cmdq_event event)
{
	s32 eventValue = cmdq_core_get_event_value(event);

	if (eventValue < 0)
		return;

	CMDQ_REG_SET32(CMDQ_SYNC_TOKEN_UPD, (1L << 16) | (u32)eventValue);
}

u32 cmdqCoreGetEvent(enum cmdq_event event)
{
	u32 regValue = 0;
	s32 eventValue = cmdq_core_get_event_value(event);

	if (eventValue < 0)
		return -EINVAL;

	CMDQ_REG_SET32(CMDQ_SYNC_TOKEN_ID, (0x3FF & (u32)eventValue));
	regValue = CMDQ_REG_GET32(CMDQ_SYNC_TOKEN_VAL);
	return regValue;
}

static void cmdq_core_reset_hw_events_impl(enum cmdq_event event)
{
	s32 value = cmdq_core_get_event_value(event);

	if (value > 0) {
		/* Reset GCE event */
		CMDQ_REG_SET32(CMDQ_SYNC_TOKEN_UPD,
			CMDQ_SYNC_TOKEN_MAX & (u32)value);
	}
}

static void cmdq_core_reset_hw_events(void)
{
	int index;
	const u32 max_thread_count = cmdq_dev_get_thread_count();
	struct cmdq_event_table *events = cmdq_event_get_table();
	u32 table_size = cmdq_event_get_table_size();

	/* set all defined events to 0 */
	CMDQ_MSG("%s\n", __func__);

	for (index = 0; index < table_size; index++)
		cmdq_core_reset_hw_events_impl(events[index].event);

	/* However, GRP_SET are resource flags, */
	/* by default they should be 1. */
	cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_GPR_SET_0);
	cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_GPR_SET_1);
	cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_GPR_SET_2);
	cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_GPR_SET_3);
	cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_GPR_SET_4);

	/* CMDQ_SYNC_RESOURCE are resource flags, */
	/* by default they should be 1. */
	cmdqCoreSetEvent(CMDQ_SYNC_RESOURCE_WROT0);
	cmdqCoreSetEvent(CMDQ_SYNC_RESOURCE_WROT1);
	/* TODO: register reset callback from cmdq_mdp_common.c */
	cmdq_mdp_reset_resource();

	/* However, CMDQ_SYNC_RESOURCE are WSM lock flags, */
	/* by default they should be 1. */
	cmdqCoreSetEvent(CMDQ_SYNC_SECURE_WSM_LOCK);

	/* However, APPEND_THR are resource flags, */
	/* by default they should be 1. */
	for (index = 0; index < max_thread_count; index++)
		cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_APPEND_THR(index));
}

void cmdq_core_reset_gce(void)
{
	/* CMDQ init flow:
	 * 1. clock-on
	 * 2. reset all events
	 */
	cmdq_get_func()->enableGCEClockLocked(true);
	cmdq_core_reset_hw_events();
	cmdq_core_config_prefetch_gsize();
#ifdef CMDQ_ENABLE_BUS_ULTRA
	CMDQ_LOG("Enable GCE Ultra ability");
	CMDQ_REG_SET32(CMDQ_BUS_CONTROL_TYPE, 0x3);
#endif
	if (cmdq_dts.ctl_int0 > 0) {
		CMDQ_REG_SET32(CMDQ_CTL_INT0, cmdq_dts.ctl_int0);
		CMDQ_MSG("[CTL_INT0] set %d\n",
			cmdq_dts.ctl_int0);
	}
	/* Restore event */
	cmdq_get_func()->eventRestore();
}

void cmdq_core_set_addon_subsys(u32 msb, s32 subsys_id, u32 mask)
{
	cmdq_adds_subsys.msb = msb;
	cmdq_adds_subsys.subsysID = subsys_id;
	cmdq_adds_subsys.mask = mask;
	CMDQ_LOG("Set AddOn Subsys:msb:0x%08x mask:0x%08x id:%d\n",
		msb, mask, subsys_id);
}

u32 cmdq_core_subsys_to_reg_addr(u32 arg_a)
{
	const u32 subsysBit = cmdq_get_func()->getSubsysLSBArgA();
	const s32 subsys_id = (arg_a & CMDQ_ARG_A_SUBSYS_MASK) >> subsysBit;
	u32 offset = 0;
	u32 base_addr = 0;
	u32 i;

	for (i = 0; i < CMDQ_SUBSYS_MAX_COUNT; i++) {
		if (cmdq_dts_data.subsys[i].subsysID == subsys_id) {
			base_addr = cmdq_dts_data.subsys[i].msb;
			offset = arg_a & ~cmdq_dts_data.subsys[i].mask;
			break;
		}
	}

	if (!base_addr && cmdq_adds_subsys.subsysID > 0 &&
		subsys_id == cmdq_adds_subsys.subsysID) {
		base_addr = cmdq_adds_subsys.msb;
		offset = arg_a & ~cmdq_adds_subsys.mask;
	}

	return base_addr | offset;
}

const char *cmdq_core_parse_subsys_from_reg_addr(u32 reg_addr)
{
	u32 addr_base_shifted;
	const char *module = "CMDQ";
	u32 i;

	for (i = 0; i < CMDQ_SUBSYS_MAX_COUNT; i++) {
		if (cmdq_dts_data.subsys[i].subsysID == -1)
			continue;

		addr_base_shifted = reg_addr & cmdq_dts_data.subsys[i].mask;
		if (cmdq_dts_data.subsys[i].msb == addr_base_shifted) {
			module = cmdq_dts_data.subsys[i].grpName;
			break;
		}
	}

	return module;
}

s32 cmdq_core_subsys_from_phys_addr(u32 physAddr)
{
	s32 msb;
	s32 subsysID = CMDQ_SPECIAL_SUBSYS_ADDR;
	u32 i;

	for (i = 0; i < CMDQ_SUBSYS_MAX_COUNT; i++) {
		if (cmdq_dts_data.subsys[i].subsysID == -1)
			continue;

		msb = physAddr & cmdq_dts_data.subsys[i].mask;
		if (msb == cmdq_dts_data.subsys[i].msb) {
			subsysID = cmdq_dts_data.subsys[i].subsysID;
			break;
		}
	}

	return subsysID;
}

ssize_t cmdq_core_print_error(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i;
	int length = 0;

	for (i = 0; i < cmdq_ctx.errNum && i < CMDQ_MAX_ERROR_COUNT;
		i++) {
		struct ErrorStruct *pError = &cmdq_ctx.error[i];
		u64 ts = pError->ts_nsec;
		unsigned long rem_nsec = do_div(ts, 1000000000);

		length += snprintf(buf + length,
			PAGE_SIZE - length, "[%5lu.%06lu] ",
			(unsigned long)ts, rem_nsec / 1000);
		length += cmdq_core_print_record(&pError->errorRec,
			i, buf + length, PAGE_SIZE - length);
		if (length >= PAGE_SIZE)
			break;
	}

	return length;
}

void cmdq_core_set_log_level(const s32 value)
{
	if (value == CMDQ_LOG_LEVEL_NORMAL) {
		/* Only print CMDQ ERR and CMDQ LOG */
		cmdq_ctx.logLevel = CMDQ_LOG_LEVEL_NORMAL;
	} else if (value < CMDQ_LOG_LEVEL_MAX) {
		/* Modify log level */
		cmdq_ctx.logLevel = (1 << value);
	}
}

ssize_t cmdq_core_print_log_level(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int len = 0;

	if (buf)
		len = sprintf(buf, "%d\n", cmdq_ctx.logLevel);

	return len;
}

ssize_t cmdq_core_write_log_level(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int len = 0;
	int value = 0;
	int status = 0;

	char textBuf[10] = { 0 };

	do {
		if (size >= 10) {
			status = -EFAULT;
			break;
		}

		len = size;
		memcpy(textBuf, buf, len);

		textBuf[len] = '\0';
		if (kstrtoint(textBuf, 10, &value) < 0) {
			status = -EFAULT;
			break;
		}

		status = len;
		if (value < 0 || value > CMDQ_LOG_LEVEL_MAX)
			value = 0;

		cmdq_core_set_log_level(value);
	} while (0);

	return status;
}

ssize_t cmdq_core_print_profile_enable(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int len = 0;

	if (buf)
		len = sprintf(buf, "0x%x\n", cmdq_ctx.enableProfile);

	return len;

}

ssize_t cmdq_core_write_profile_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int len = 0;
	int value = 0;
	int status = 0;

	char textBuf[10] = { 0 };

	do {
		if (size >= 10) {
			status = -EFAULT;
			break;
		}

		len = size;
		memcpy(textBuf, buf, len);

		textBuf[len] = '\0';
		if (kstrtoint(textBuf, 10, &value) < 0) {
			status = -EFAULT;
			break;
		}

		status = len;
		if (value < 0 || value > CMDQ_PROFILE_MAX)
			value = 0;

		if (value == CMDQ_PROFILE_OFF)
			cmdq_ctx.enableProfile = CMDQ_PROFILE_OFF;
		else
			cmdq_ctx.enableProfile |= (1 << value);
	} while (0);

	return status;
}

static void cmdq_core_fill_handle_record(
	struct RecordStruct *record, const struct cmdqRecStruct *handle,
	u32 thread)
{

	if (record && handle) {
		/* Record scenario */
		record->user = handle->caller_pid;
		record->scenario = handle->scenario;
		record->priority = handle->pkt ? handle->pkt->priority : 0;
		record->thread = thread;
		record->reorder = handle->reorder;
		record->engineFlag = handle->engineFlag;
		record->size = handle->pkt ? handle->pkt->cmd_buf_size : 0;
		record->is_secure = handle->secData.is_secure;

		/* TODO: extend Record time field*/
		record->submit = handle->submit;
		record->trigger = handle->trigger;
		record->gotIRQ = handle->gotIRQ;
		record->beginWait = handle->beginWait;
		record->wakedUp = handle->wakedUp;
		record->durAlloc = handle->durAlloc;
		record->durReclaim = handle->durReclaim;
		record->durRelease = handle->durRelease;
		/* Record address */
		if (handle->pkt && !list_empty(&handle->pkt->buf)) {
			struct cmdq_pkt_buffer *first = list_first_entry(
				&handle->pkt->buf, typeof(*first), list_entry);
			struct cmdq_pkt_buffer *last = list_last_entry(
				&handle->pkt->buf, typeof(*first), list_entry);

			record->start = (u32)first->pa_base;
			record->end = (u32)last->pa_base +
				CMDQ_CMD_BUFFER_SIZE -
				handle->pkt->avail_buf_size;
			record->jump = handle->cmd_end ?
				handle->cmd_end[0] : 0;
		} else {
			record->start = 0;
			record->end = 0;
			record->jump = 0;
		}
	}

}

static void cmdq_core_track_handle_record(struct cmdqRecStruct *handle,
	u32 thread)
{
	struct RecordStruct *pRecord;
	unsigned long flags;
	CMDQ_TIME done;
	int lastID;
	char buf[256];

	done = sched_clock();

	spin_lock_irqsave(&cmdq_record_lock, flags);

	pRecord = &(cmdq_ctx.record[cmdq_ctx.lastID]);
	lastID = cmdq_ctx.lastID;

	cmdq_core_fill_handle_record(pRecord, handle, thread);

	pRecord->done = done;

	cmdq_ctx.lastID++;
	if (cmdq_ctx.lastID >= CMDQ_MAX_RECORD_COUNT)
		cmdq_ctx.lastID = 0;

	cmdq_ctx.recNum++;
	if (cmdq_ctx.recNum >= CMDQ_MAX_RECORD_COUNT)
		cmdq_ctx.recNum = CMDQ_MAX_RECORD_COUNT;

	spin_unlock_irqrestore(&cmdq_record_lock, flags);

	if (handle->dumpAllocTime) {
		cmdq_core_print_record(pRecord, lastID, buf,
			ARRAY_SIZE(buf));
		CMDQ_LOG("Record:%s\n", buf);
	}

}

void cmdq_core_turnon_first_dump_by_handle(
	const struct cmdqRecStruct *handle)
{
	if (cmdq_first_err.cmdqCount != 0 || !handle)
		return;

	cmdq_first_err.flag = true;
	/* save kernel time, pid, and caller name */

	cmdq_first_err.callerPid = handle->caller_pid;
	snprintf(cmdq_first_err.callerName, TASK_COMM_LEN, "%s",
		handle->caller_name);
	cmdq_first_err.savetime = sched_clock();
	do_gettimeofday(&cmdq_first_err.savetv);
}

void cmdq_core_dump_tasks_info(void)
{
	/* TODO: dump mailbox cmdq tasks */
#if 0
	struct TaskStruct *pTask = NULL;
	struct list_head *p = NULL;
	s32 index = 0;

	CMDQ_ERR("========= Active List Task Dump =========\n");
	index = 0;
	list_for_each(p, &cmdq_ctx.taskActiveList) {
		pTask = list_entry(p, struct TaskStruct, listEntry);
		CMDQ_ERR(
			"Task(%d) 0x%p Pid:%d Name:%s Scenario:%d engineFlag:0x%llx\n",
			index, pTask, pTask->callerPid, pTask->callerName,
			pTask->scenario, pTask->engineFlag);
		++index;
	}
	CMDQ_ERR("====== Total %d in Active Task =======\n", index);
#endif
}

static void *cmdq_core_get_pc_va(dma_addr_t pc,
	const struct cmdqRecStruct *handle)
{
	struct cmdq_pkt_buffer *buf;

	if (!pc)
		return NULL;

	list_for_each_entry(buf, &handle->pkt->buf, list_entry) {
		if (!(pc >= buf->pa_base &&
			pc < buf->pa_base + CMDQ_CMD_BUFFER_SIZE))
			continue;
		return buf->va_base + (pc - buf->pa_base);
	}

	return NULL;
}

struct cmdqRecStruct *cmdq_core_get_valid_handle(unsigned long job)
{
	struct cmdqRecStruct *handle = NULL, *entry;

	mutex_lock(&cmdq_handle_list_mutex);
	list_for_each_entry(entry, &cmdq_ctx.handle_active, list_entry) {
		if (entry == (void *)job) {
			handle = entry;
			break;
		}
	}
	mutex_unlock(&cmdq_handle_list_mutex);

	return handle;
}

u32 *cmdq_core_get_pc_inst(const struct cmdqRecStruct *handle,
	s32 thread, u32 insts[2], u32 *pa_out)
{
	u32 curr_pc = 0L;
	void *inst_ptr = NULL;
	s32 ret = 0;

	if (unlikely(!handle || list_empty(&handle->pkt->buf) ||
		thread == CMDQ_INVALID_THREAD)) {
		CMDQ_ERR(
			"get pc failed since invalid param handle:0x%p thread:%d\n",
			handle, thread);
		return NULL;
	}

	curr_pc = CMDQ_AREG_TO_PHYS(cmdq_core_get_pc(thread));
	if (unlikely(handle->use_sram_buffer)) {
		void *va = cmdq_pkt_get_first_va(handle);

		inst_ptr = va + (curr_pc - handle->sram_base);
	} else if (curr_pc) {
		inst_ptr = cmdq_core_get_pc_va(curr_pc, handle);
	}

	if (inst_ptr) {
		ret = cmdq_pkt_get_cmd_by_pc(handle, curr_pc, insts, 2);

		if (!ret) {
			insts[0] = CMDQ_REG_GET32(inst_ptr + 0);
			insts[1] = CMDQ_REG_GET32(inst_ptr + 4);
		} else {
			insts[0] = 0;
			insts[1] = 0;
		}
	} else {
		insts[0] = 0;
		insts[1] = 0;
	}

	if (pa_out)
		*pa_out = curr_pc;

	return (u32 *)inst_ptr;
}


static void cmdq_core_parse_handle_error(const struct cmdqRecStruct *handle,
	u32 thread, const char **moduleName, s32 *flag,
	u32 *insts, u32 size, u32 **pc_va)
{
	u32 op, arg_a, arg_b;
	s32 eventENUM;
	u32 addr = 0;
	const char *module = NULL;
	int isSMIHang = 0;
	dma_addr_t curr_pc = 0;
	u32 tmp_instr[2] = { 0 };
	struct cmdq_client *client;

	if (unlikely(!handle)) {
		CMDQ_ERR("No handle to parse error\n");
		return;
	}

	client = cmdq_clients[handle->thread];

	do {
		/* confirm if SMI is hang */
		isSMIHang = cmdq_get_func()->dumpSMI(0);
		if (isSMIHang) {
			module = "SMI";
			break;
		}

		/* other cases, use instruction to judge
		 * because scenario / HW flag are not sufficient
		 a* e.g. ISP pass 2 involves both MDP and ISP
		 * so we need to check which instruction timeout-ed.
		 */
		if (!insts || !size)
			break;

		cmdq_task_get_thread_pc(client->chan, &curr_pc);
		if (cmdq_pkt_get_cmd_by_pc(handle, curr_pc, tmp_instr, 2) <
			0) {
			CMDQ_ERR("%s get handle cmd fail\n", __func__);
			break;
		}

		insts[0] = tmp_instr[0];
		insts[1] = tmp_instr[1];
		if (!insts[0] && !insts[1])
			break;

		op = (insts[1] & 0xFF000000) >> 24;
		arg_a = insts[1] & (~0xFF000000);
		arg_b = insts[0];

		/* quick exam by hwflag first */
		module = cmdq_get_func()->parseHandleErrorModule(handle);
		if (module != NULL)
			break;

		switch (op) {
		case CMDQ_CODE_POLL:
		case CMDQ_CODE_WRITE:
			addr = cmdq_core_subsys_to_reg_addr(arg_a);
			module = cmdq_get_func()->parseModule(addr);
			break;
		case CMDQ_CODE_WFE:
			/* arg_a is the event ID */
			eventENUM = cmdq_core_reverse_event_enum(arg_a);
			module = cmdq_get_func()->moduleFromEvent(eventENUM,
				cmdq_group_cb, handle->engineFlag);
			break;
		case CMDQ_CODE_READ:
		case CMDQ_CODE_MOVE:
		case CMDQ_CODE_JUMP:
		case CMDQ_CODE_EOC:
		default:
			module = "CMDQ";
			break;
		}
	} while (0);

	/* fill output parameter */
	*moduleName = module ? module : "CMDQ";
	cmdq_task_get_thread_irq(client->chan, flag);
	if (pc_va)
		*pc_va = cmdq_core_get_pc_va(curr_pc, handle);
}

void cmdq_core_dump_handle_error_instruction(const u32 *pcVA, const long pcPA,
	u32 *insts, int thread, u32 lineNum)
{
	char parsedInstruction[128] = { 0 };
	const u32 op = (insts[1] & 0xFF000000) >> 24;

	if (CMDQ_IS_END_ADDR(pcPA)) {
		/* in end address case instruction may not correct */
		CMDQ_ERR("PC stay at GCE end address, line:%u\n", lineNum);
		return;
	} else if (pcVA == NULL) {
		CMDQ_ERR("Dump error instruction with null va, line:%u\n",
			lineNum);
		return;
	}

	cmdq_core_parse_instruction(pcVA, parsedInstruction,
		sizeof(parsedInstruction));
	CMDQ_ERR("Thread %d error instruction:0x%p 0x%08x:0x%08x => %s",
		 thread, pcVA, insts[0], insts[1], parsedInstruction);

	/* for WFE, we specifically dump the event value */
	if (op == CMDQ_CODE_WFE) {
		u32 regValue = 0;
		const u32 eventID = 0x3FF & insts[1];

		CMDQ_REG_SET32(CMDQ_SYNC_TOKEN_ID, eventID);
		regValue = CMDQ_REG_GET32(CMDQ_SYNC_TOKEN_VAL);
		CMDQ_ERR("CMDQ_SYNC_TOKEN_VAL(idx:%u) of %s is %d\n",
			eventID, cmdq_core_get_event_name(eventID), regValue);
	}
}

static void cmdq_core_create_nghandleinfo(const struct cmdqRecStruct *nghandle,
	void *va_pc, struct cmdq_ng_handle_info *nginfo_out)
{
	void *buffer = NULL;
	struct cmdq_pkt_buffer *buf;

	if (!nginfo_out)
		return;

	/* copy ngtask info */
	buf = list_first_entry(&nghandle->pkt->buf, typeof(*buf), list_entry);
	nginfo_out->va_start = buf->va_base;
	nginfo_out->va_pc = (u32 *)va_pc;
	nginfo_out->buffer = NULL;
	nginfo_out->engine_flag = nghandle->engineFlag;
	nginfo_out->scenario = nghandle->scenario;
	nginfo_out->nghandle = nghandle;
	nginfo_out->buffer_size = nghandle->pkt->cmd_buf_size;

	buffer = kzalloc(nghandle->pkt->buf_size, GFP_ATOMIC);
	if (!buffer) {
		CMDQ_ERR("fail to allocate NG buffer\n");
		return;
	}

	nginfo_out->buffer = (u32 *)buffer;
	list_for_each_entry(buf, &nghandle->pkt->buf, list_entry) {
		u32 buf_size;

		if (list_is_last(&buf->list_entry, &nghandle->pkt->buf))
			buf_size = CMDQ_CMD_BUFFER_SIZE -
				nghandle->pkt->avail_buf_size;
		else
			buf_size = CMDQ_CMD_BUFFER_SIZE;

		memcpy(buffer, buf->va_base, buf_size);
		if (va_pc >= buf->va_base &&
			va_pc < buf->va_base + CMDQ_CMD_BUFFER_SIZE) {
			buffer += va_pc - buf->va_base;
			break;
		}

		buffer += buf_size;
	}

	/* only dump to pc */
	nginfo_out->dump_size = buffer - (void *)nginfo_out->buffer;
}

static void cmdq_core_dump_handle_summary(const struct cmdqRecStruct *handle,
	s32 thread, const struct cmdqRecStruct **nghandle_out,
	struct cmdq_ng_handle_info *nginfo_out)
{
	const char *module = NULL;
	s32 irqFlag = 0;
	u32 insts[2] = { 0 };
	u32 *pcVA = NULL;
	dma_addr_t curr_pc = 0;
	struct cmdq_client *client;

	if (!handle || !handle->pkt || list_empty(&handle->pkt->buf) ||
		thread == CMDQ_INVALID_THREAD) {
		CMDQ_ERR(
			"%s invalid param handle:0x%p pkt:0x%p thread:%d\n",
			__func__, handle, handle ? handle->pkt : NULL, thread);
		return;
	}

	client = cmdq_clients[handle->thread];

	/* Do summary ! */
	cmdq_core_parse_handle_error(handle, thread, &module, &irqFlag,
		insts, ARRAY_SIZE(insts), &pcVA);
	CMDQ_ERR("** [Module] %s **\n", module);

	if (!cmdq_first_err_mod)
		cmdq_first_err_mod = module;

	CMDQ_ERR(
		"** [Error Info] Refer to instruction and check engine dump for debug**\n");

	cmdq_task_get_thread_pc(client->chan, &curr_pc);
	cmdq_core_dump_handle_error_instruction(pcVA,
		(long)curr_pc, insts, thread, __LINE__);

	cmdq_core_dump_trigger_loop_thread("ERR");

	*nghandle_out = handle;
	if (nginfo_out) {
		cmdq_core_create_nghandleinfo(handle, pcVA, nginfo_out);
		nginfo_out->module = module;
		nginfo_out->irq_flag = irqFlag;
		nginfo_out->inst[0] = insts[0];
		nginfo_out->inst[1] = insts[1];
	}

}

static void cmdq_core_dump_task_in_thread_by_handle(const s32 thread,
	const bool fullTatskDump, const bool dumpCookie,
	const bool dumpCmd, const struct cmdqRecStruct *handle)
{
#if 0
	struct cmdq_client *client;

	if (thread == CMDQ_INVALID_THREAD || !handle)
		return;

	CMDQ_ERR("========= [CMDQ] All Task in Error Thread %d =========\n",
			thread);

	client = cmdq_clients[handle->thread];
#endif
#if 0
	struct cmdq_timeout_info *timeout_info = NULL;
	struct cmdq_thread_task_info *task_info = NULL;
	u32 index = 0;

	if (thread == CMDQ_INVALID_THREAD || !handle || !handle->timeout_info)
		return;

	CMDQ_ERR("========= [CMDQ] All Task in Error Thread %d =========\n",
		thread);

	timeout_info = handle->timeout_info;

	list_for_each_entry(task_info, &timeout_info->task_list, list_entry) {
		struct cmdq_pkt_buffer *buf;
		u32 *va;

		buf = list_last_entry(&handle->pkt->buf, typeof(*buf),
			list_entry);
		va = (u32 *)(buf->va_base + CMDQ_CMD_BUFFER_SIZE -
			handle->pkt->avail_buf_size);
		va -= 4;

		CMDQ_ERR(
			"Slot %d task:0x%p va:0x%p pa:%pa size:%zu priority:%d\n",
			index, task_info, buf->va_base, &buf->pa_base,
			handle->pkt->cmd_buf_size, 0);
		CMDQ_ERR("cont'd: Last Inst 0x%08x:0x%08x 0x%08x:0x%08x\n",
			va[1], va[0], va[3], va[2]);

		if (dumpCmd && buf->va_base) {
			u32 dump_size = CMDQ_CMD_BUFFER_SIZE -
				handle->pkt->avail_buf_size;

			CMDQ_ERR("dump last buffer content size:%u ...\n",
				dump_size);
			print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS,
				16, 4, buf->va_base, dump_size, true);
		}
		index++;
	}
#endif
}

static void cmdq_core_dump_handle_with_engine_flag(
	const struct cmdqRecStruct *handle, u64 engineFlag, s32 current_thread)
{
	/* TODO: move to cmdq_mdp_common.c */
#if 0
	const u32 max_thread_count = cmdq_dev_get_thread_count();
	s32 thread_idx = 0;

	if (current_thread == CMDQ_INVALID_THREAD)
		return;

	CMDQ_ERR(
		"=============== [CMDQ] All task in thread sharing same engine flag 0x%016llx ===============\n",
		engineFlag);

	/* TODO: revise thread with mdp_common.c */
	for (thread_idx = 0; thread_idx < max_thread_count; thread_idx++) {
		struct ThreadStruct *thread = &cmdq_ctx.thread[thread_idx];

		if (thread_idx == current_thread || thread->taskCount <= 0 ||
			!(thread->engineFlag & engineFlag))
			continue;

		cmdq_core_dump_task_in_thread_by_handle(current_thread,
			false, false, false, handle);
	}
#endif
}

static atomic_t cmdq_sec_dbg_ctrl = ATOMIC_INIT(0);

static void cmdq_core_dump_dbg(const char *tag)
{
	u32 dbg0[3], dbg2[6], i;

	if (atomic_cmpxchg(&cmdq_sec_dbg_ctrl, 0, 1) == 0) {
		struct arm_smccc_res res;

		arm_smccc_smc(MTK_SIP_CMDQ_CONTROL, CMDQ_ENABLE_DEBUG,
			0, 0, 0, 0, 0, 0, &res);
	}

	/* debug select */
	for (i = 0; i < 6; i++) {
		if (i < 3) {
			CMDQ_REG_SET32(GCE_DBG_CTL, (i << 8) | i);
			dbg0[i] = CMDQ_REG_GET32(GCE_DBG0);
		} else {
			/* only other part */
			CMDQ_REG_SET32(GCE_DBG_CTL, i << 8);
		}
		dbg2[i] = CMDQ_REG_GET32(GCE_DBG2);
	}

	CMDQ_LOG("[%s]dbg0:%#x %#x %#x dbg2:%#x %#x %#x %#x %#x %#x\n",
		tag, dbg0[0], dbg0[1], dbg0[2],
		dbg2[0], dbg2[1], dbg2[2], dbg2[3], dbg2[4], dbg2[5]);
}

static void cmdq_core_dump_status(const char *tag)
{
	s32 coreExecThread = CMDQ_INVALID_THREAD;
	u32 value[4] = { 0 };

	value[0] = CMDQ_REG_GET32(CMDQ_CURR_LOADED_THR);
	value[1] = CMDQ_REG_GET32(CMDQ_THR_EXEC_CYCLES);
	value[2] = CMDQ_REG_GET32(CMDQ_THR_TIMEOUT_TIMER);
	value[3] = CMDQ_REG_GET32(CMDQ_CURR_IRQ_STATUS);

	/* this returns (1 + index of least bit set) or 0 if input is 0. */
	coreExecThread = __builtin_ffs(value[0]) - 1;

	CMDQ_LOG(
		"[%s]IRQ:0x%08x Execing:%d Thread:%d CURR_LOADED_THR:0x%08x THR_EXEC_CYCLES:0x%08x\n",
		tag, value[3], (0x80000000 & value[0]) ? 1 : 0,
		 coreExecThread, value[0], value[1]);

	cmdq_core_dump_dbg(tag);
}

void cmdq_core_dump_handle_buffer(const struct cmdq_pkt *pkt,
	const char *tag)
{
	struct cmdq_pkt_buffer *buf;

	list_for_each_entry(buf, &pkt->buf, list_entry) {
		u32 *va;

		if (list_is_last(&buf->list_entry, &pkt->buf))
			va = (u32 *)(buf->va_base + CMDQ_CMD_BUFFER_SIZE -
				pkt->avail_buf_size);
		else
			va = (u32 *)(buf->va_base + CMDQ_CMD_BUFFER_SIZE);
		va -= 4;

		CMDQ_LOG(
			"[%s]va:0x%p pa:%pa last inst (0x%p) 0x%08x:%08x 0x%08x:%08x\n",
			tag, buf->va_base, &buf->pa_base,
			va, va[1], va[0], va[3], va[2]);
	}
}

static void cmdq_core_dump_handle(const struct cmdqRecStruct *handle,
	const char *tag)
{
	CMDQ_LOG(
		"[%s]handle:0x%p pkt:0x%p scenario:%d pri:%u state:%d flag:0x%llx\n",
		tag, handle, handle->pkt, handle->scenario,
		handle->pkt->priority, handle->state, handle->engineFlag);
	cmdq_core_dump_handle_buffer(handle->pkt, tag);
	CMDQ_LOG("[%s]cmd size:%zu buffer size:%zu available size:%zu\n",
		tag, handle->pkt->cmd_buf_size, handle->pkt->buf_size,
		handle->pkt->avail_buf_size);
	if (handle->reg_values || handle->reg_count)
		CMDQ_LOG("[%s]Result buffer va:0x%p pa:%pa count:%u\n",
			tag, handle->reg_values, &handle->reg_values_pa,
			handle->reg_count);
	if (handle->sram_base)
		CMDQ_LOG("[%s]** This is SRAM task, sram base:0x%08x\n",
			tag, handle->sram_base);
	CMDQ_LOG(
		"[%s]Reorder:%d Trigger:%lld Got IRQ:0x%llx Wait:%lld Finish:%lld\n",
		tag, handle->reorder, handle->trigger, handle->gotIRQ,
		handle->beginWait, handle->wakedUp);
	CMDQ_LOG("[%s]Caller pid:%d name:%s\n",
		tag, handle->caller_pid, handle->caller_name);
}

u32 *cmdq_core_dump_pc(const struct cmdqRecStruct *handle,
	int thread, const char *tag)
{
	u32 *pcVA = NULL;
	u32 insts[2] = { 0 };
	char parsedInstruction[128] = { 0 };
	dma_addr_t curr_pc = 0;
	u32 tmp_insts[2] = { 0 };
	struct cmdq_client *client;

	if (!handle)
		return NULL;

	client = cmdq_clients[handle->thread];
	cmdq_task_get_thread_pc(client->chan, &curr_pc);

	pcVA = cmdq_core_get_pc_va(curr_pc, handle);

	if (cmdq_pkt_get_cmd_by_pc(handle, curr_pc, tmp_insts, 2) < 0) {
		CMDQ_ERR("%s get handle cmd fail\n", __func__);
		return NULL;
	}

	insts[0] = tmp_insts[0];
	insts[1] = tmp_insts[1];

	if (pcVA) {
		const u32 op = (insts[1] & 0xFF000000) >> 24;

		cmdq_core_parse_instruction(pcVA,
			parsedInstruction, sizeof(parsedInstruction));

		/* for WFE, we specifically dump the event value */
		if (op == CMDQ_CODE_WFE) {
			u32 regValue = 0;
			const u32 eventID = 0x3FF & insts[1];

			CMDQ_REG_SET32(CMDQ_SYNC_TOKEN_ID, eventID);
			regValue = CMDQ_REG_GET32(CMDQ_SYNC_TOKEN_VAL);
			CMDQ_LOG(
				"[%s]Thread %d PC:0x%p(%pa) 0x%08x:0x%08x => %s value:%d",
				tag, thread, pcVA, &curr_pc,
				insts[0], insts[1],
				parsedInstruction, regValue);
		} else {
			CMDQ_LOG(
				"[%s]Thread %d PC:0x%p(%pa), 0x%08x:0x%08x => %s",
				tag, thread, pcVA, &curr_pc,
				insts[0], insts[1], parsedInstruction);
		}
	} else {
		CMDQ_LOG("[%s]Thread %d PC:%s\n", tag, thread,
			handle->secData.is_secure ?
			"HIDDEN INFO since is it's secure thread" :
			"Not available");
	}

	return pcVA;
}

static void cmdq_core_dump_error_handle(const struct cmdqRecStruct *handle,
	u32 thread, u32 **pc_out)
{
	u32 *hwPC = NULL;
	u64 printEngineFlag = 0;

	cmdq_core_dump_thread(handle, thread, true, "ERR");

	if (handle) {
		CMDQ_ERR("============ [CMDQ] Error Thread PC ============\n");
		hwPC = cmdq_core_dump_pc(handle, thread, "ERR");

		CMDQ_ERR("========= [CMDQ] Error Task Status =========\n");
		cmdq_core_dump_handle(handle, "ERR");

		printEngineFlag |= handle->engineFlag;
	}

	if (cmdq_ctx.errNum > 1)
		return;

	/* dump tasks in error thread */
	cmdq_core_dump_task_in_thread_by_handle(
		thread, false, false, false, handle);
	cmdq_core_dump_handle_with_engine_flag(
		handle, printEngineFlag, thread);

	CMDQ_ERR("============ [CMDQ] CMDQ Status ============\n");
	cmdq_core_dump_status("ERR");

	CMDQ_ERR("============ [CMDQ] Clock Gating Status ============\n");
	CMDQ_ERR("[CLOCK] common clock ref:%d\n",
		atomic_read(&cmdq_thread_usage));

	if (pc_out)
		*pc_out = hwPC;
}

static void cmdq_core_attach_cmdq_error(
	const struct cmdqRecStruct *handle, s32 thread,
	u32 **pc_out, struct cmdq_ng_handle_info *nginfo_out)
{
	const struct cmdqRecStruct *nghandle = NULL;
	u64 eng_flag = 0;
	s32 index = 0;
	struct EngineStruct *engines = cmdq_mdp_get_engines();

	CMDQ_PROF_MMP(cmdq_mmp_get_event()->warning, MMPROFILE_FLAG_PULSE,
		((unsigned long)handle), thread);

	/* Update engine fail count */
	eng_flag = handle->engineFlag;
	for (index = 0; index < CMDQ_MAX_ENGINE_COUNT; index++) {
		if (eng_flag & (1LL << index))
			engines[index].failCount++;
	}

	/* register error record */
	if (cmdq_ctx.errNum < CMDQ_MAX_ERROR_COUNT) {
		struct ErrorStruct *error = &cmdq_ctx.error[cmdq_ctx.errNum];

		cmdq_core_fill_handle_record(&error->errorRec, handle, thread);
		error->ts_nsec = local_clock();
	}

	/* Turn on first CMDQ error dump */
	cmdq_core_turnon_first_dump_by_handle(handle);

	/* Then we just print out info */
	CMDQ_ERR("============== [CMDQ] Begin of Error %d =============\n",
		cmdq_ctx.errNum);

	cmdq_core_dump_handle_summary(handle, thread, &nghandle, nginfo_out);
	cmdq_core_dump_error_handle(handle, thread, pc_out);


	CMDQ_ERR("============== [CMDQ] End of Error %d =============\n",
		 cmdq_ctx.errNum);
	cmdq_ctx.errNum++;
}

static void cmdq_core_release_nghandleinfo(struct cmdq_ng_handle_info *nginfo)
{
	kfree(nginfo->buffer);
}

static void cmdq_core_dump_error_buffer(const struct cmdqRecStruct *handle,
	void *pc, u32 pc_pa)
{
	struct cmdq_pkt_buffer *buf;
	const struct cmdq_pkt *pkt = handle->pkt;
	u32 cmd_size = 0, dump_size = 0;
	bool dump = false;
	u32 dump_buff_count = 0;
	u32 cnt = 0;

	if (list_empty(&pkt->buf))
		return;

	list_for_each_entry(buf, &pkt->buf, list_entry) {
		if (list_is_last(&buf->list_entry, &pkt->buf))
			/* remain buffer always print */
			cmd_size = CMDQ_CMD_BUFFER_SIZE - pkt->avail_buf_size;
		else
			cmd_size = CMDQ_CMD_BUFFER_SIZE;
		if (pc && pc >= buf->va_base && pc < buf->va_base + cmd_size) {
			/* because hwPC points to "start" of the
			 * instruction, add offset 1
			 */
			dump_size = pc - buf->va_base + CMDQ_INST_SIZE;
			dump = true;
		} else if (pc_pa && pc_pa >= buf->pa_base &&
			pc_pa < buf->pa_base + cmd_size) {
			dump_size = pc_pa - buf->pa_base + CMDQ_INST_SIZE;
			dump = true;
		} else {
			dump_size = cmd_size;
		}
		CMDQ_LOG("buffer %u va:0x%p pa:%pa\n",
			cnt, buf->va_base, &buf->pa_base);
		print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS,
			16, 4, buf->va_base, dump_size, true);
		cmdq_core_save_hex_first_dump("", 16, 4,
			buf->va_base, dump_size);
		if (dump)
			break;
		if (dump_buff_count++ >= 2) {
			CMDQ_LOG("only dump 2 buffer...\n");
			break;
		}
		cnt++;
	}
	if (pc_pa && !dump)
		CMDQ_LOG("PC is not in region, dump all\n");
}

static void cmdq_core_dump_handle_command(const struct cmdqRecStruct *handle,
	u32 *pc)
{
	CMDQ_ERR("============ [CMDQ] Error Command Buffer ============\n");

	cmdq_core_dump_error_buffer(handle, pc, 0);
}

s32 cmdq_core_is_group_flag(enum CMDQ_GROUP_ENUM engGroup, u64 engineFlag)
{
	if (!cmdq_core_is_valid_group(engGroup))
		return false;

	if (cmdq_mdp_get_func()->getEngineGroupBits(engGroup) & engineFlag)
		return true;

	return false;
}

static void cmdq_core_attach_engine_error(
	const struct cmdqRecStruct *handle, s32 thread,
	const struct cmdq_ng_handle_info *nginfo, bool short_log)
{
	s32 index = 0;
	bool disp_scn = false;
	u64 print_eng_flag = 0;
	u64 engine_bit = 0;
	CmdqMdpGetEngineGroupBits get_engine_group_bit = NULL;
	CmdqIsDispScenario is_disp_scen = NULL;
	struct CmdqCBkStruct *callback = NULL;
	static const char *const engineGroupName[] = {
		CMDQ_FOREACH_GROUP(GENERATE_STRING)
	};

#ifndef CONFIG_FPGA_EARLY_PORTING
	CMDQ_ERR("============ [CMDQ] SMI Status ============\n");
	cmdq_get_func()->dumpSMI(1);
#endif

	if (short_log) {
		CMDQ_ERR("============ skip detail error dump ============\n");
		return;
	}

	print_eng_flag = handle->engineFlag;
	get_engine_group_bit = cmdq_mdp_get_func()->getEngineGroupBits;
	is_disp_scen = cmdq_get_func()->isDispScenario;

	if (nginfo)
		print_eng_flag |= nginfo->engine_flag;

	/* Dump MMSYS configuration */
	CMDQ_ERR("============ [CMDQ] MMSYS_CONFIG ============\n");
	cmdq_mdp_get_func()->dumpMMSYSConfig();

	/* ask each module to print their status */
	CMDQ_ERR("============ [CMDQ] Engine Status ============\n");
	callback = cmdq_group_cb;
	for (index = 0; index < CMDQ_MAX_GROUP_COUNT; index++) {
		if (!cmdq_core_is_group_flag(
			(enum CMDQ_GROUP_ENUM) index, print_eng_flag))
			continue;

		CMDQ_ERR("====== engine group %s status =======\n",
			engineGroupName[index]);

		if (callback[index].dumpInfo == NULL) {
			CMDQ_ERR("(no dump function)\n");
			continue;
		}

		callback[index].dumpInfo(
			(get_engine_group_bit(index) & print_eng_flag),
			cmdq_ctx.logLevel);
	}

	/* force dump DISP for DISP scenario with 0x0 engine flag */
	disp_scn = is_disp_scen(handle->scenario);

	if (nginfo)
		disp_scn = disp_scn | is_disp_scen(nginfo->scenario);

	if (disp_scn) {
		index = CMDQ_GROUP_DISP;
		if (callback[index].dumpInfo) {
			engine_bit =
				get_engine_group_bit(index) & print_eng_flag;

			callback[index].dumpInfo(engine_bit,
				cmdq_ctx.logLevel);
		}
	}

	for (index = 0; index < CMDQ_MAX_GROUP_COUNT; ++index) {
		if (!cmdq_core_is_group_flag((enum CMDQ_GROUP_ENUM) index,
			print_eng_flag))
			continue;

		if (callback[index].errorReset) {
			callback[index].errorReset(
				get_engine_group_bit(index) & print_eng_flag);
			CMDQ_LOG(
				"engine group (%s): function is called\n",
				engineGroupName[index]);
		}

	}
}

static void cmdq_core_attach_error_handle_detail(
	const struct cmdqRecStruct *handle, s32 thread,
	const struct cmdqRecStruct **nghandle_out, const char **module_out,
	u32 *irq_flag_out, u32 *inst_a_out, u32 *inst_b_out, bool *aee_out)
{
	s32 error_num = cmdq_ctx.errNum;
	u32 *pc = NULL;
	struct cmdq_ng_handle_info nginfo = {0};
	bool detail_log = false;

	if (!handle) {
		CMDQ_ERR("attach error failed since handle is NULL");
		if (aee_out)
			*aee_out = false;
		return;
	}

	/* prevent errors dump at same time */
	mutex_lock(&cmdq_err_mutex);

	cmdq_core_attach_cmdq_error(handle, thread, &pc, &nginfo);

	if (nghandle_out && nginfo.nghandle)
		*nghandle_out = nginfo.nghandle;

	if (module_out && irq_flag_out && inst_a_out && inst_b_out) {
		*module_out = nginfo.module;
		*irq_flag_out = nginfo.irq_flag;
		*inst_a_out = nginfo.inst[1];
		*inst_b_out = nginfo.inst[0];
	}

	if (((nginfo.inst[1] & 0xFF000000) >> 24) == CMDQ_CODE_WFE) {
		const u32 event = nginfo.inst[1] & ~0xFF000000;

		if (event >= CMDQ_SYNC_RESOURCE_WROT0)
			cmdq_mdp_dump_resource(event);
	}

	if (handle->scenario == CMDQ_SCENARIO_DISP_ESD_CHECK) {
		cmdq_ctx.errNum--;
		if (!cmdq_ctx.errNum)
			cmdq_core_reset_first_dump();
		mutex_unlock(&cmdq_err_mutex);
		cmdq_core_release_nghandleinfo(&nginfo);
		/* don't aee for esd case */
		if (aee_out)
			*aee_out = false;
		return;
	}

	detail_log = error_num <= 2 || error_num % 16 == 0 ||
		cmdq_core_should_full_error();
	cmdq_core_attach_engine_error(handle, thread,
		&nginfo, !detail_log);
	if (detail_log)
		cmdq_core_dump_handle_command(handle, pc);

	CMDQ_ERR("=========== [CMDQ] End of Full Error %d ==========\n",
		error_num);

	mutex_unlock(&cmdq_err_mutex);

	cmdq_core_release_nghandleinfo(&nginfo);
	if (aee_out)
		*aee_out = true;
}

void cmdq_core_attach_error_handle(const struct cmdqRecStruct *handle,
	s32 thread)
{
	cmdq_core_attach_error_handle_detail(handle, thread, NULL, NULL, NULL,
		NULL, NULL, NULL);
}

static void cmdq_core_attach_error_handle_by_state(
	const struct cmdqRecStruct *handle, enum TASK_STATE_ENUM state)
{
	const struct cmdqRecStruct *nghandle = NULL;
	const char *module = NULL;
	u32 irq_flag = 0, inst_a = 0, inst_b = 0;
	u32 op = 0;
	bool aee = true;

	if (unlikely(!handle)) {
		CMDQ_ERR("attach error without handle\n");
		return;
	}

	if (unlikely((state != TASK_STATE_TIMEOUT))) {
		CMDQ_ERR("attach error handle:0x%p state:%d\n",
			handle, state);
		return;
	}

	cmdq_core_attach_error_handle_detail(handle, handle->thread,
		&nghandle, &module, &irq_flag, &inst_a, &inst_b, &aee);
	op = (inst_a & 0xFF000000) >> 24;

	if (!aee) {
		CMDQ_ERR(
			"Skip AEE for %s in CMDQ IRQ:0x%02x, INST:(0x%08x, 0x%08x), OP:%s\n",
			module, irq_flag, inst_a, inst_b,
			cmdq_core_parse_op(op));
		return;
	}

	switch (op) {
	case CMDQ_CODE_WFE:
		CMDQ_AEE(module,
			"%s in CMDQ IRQ:0x%02x, INST:(0x%08x, 0x%08x), OP:WAIT EVENT:%s\n",
			module, irq_flag, inst_a, inst_b,
			cmdq_core_get_event_name(inst_a & (~0xFF000000)));
		break;
	default:
		CMDQ_AEE(module,
			"%s in CMDQ IRQ:0x%02x, INST:(0x%08x, 0x%08x), OP:%s\n",
			module, irq_flag, inst_a, inst_b,
			cmdq_core_parse_op(op));
		break;
	}
}

static void cmdq_core_dump_thread_usage(void)
{
	u32 idx;

	for (idx = CMDQ_DYNAMIC_THREAD_ID_START;
		idx < cmdq_dev_get_thread_count(); idx++) {
		CMDQ_LOG(
			"thread:%d used:%s acquire:%u scenario:%d\n",
			idx,
			cmdq_ctx.thread[idx].used ? "true" : "false",
			cmdq_ctx.thread[idx].acquire,
			cmdq_ctx.thread[idx].scenario);
	}
}

s32 cmdq_core_acquire_thread(enum CMDQ_SCENARIO_ENUM scenario, bool exclusive)
{
	s32 idx, thread_id = CMDQ_INVALID_THREAD;

	mutex_lock(&cmdq_thread_mutex);
	for (idx = CMDQ_DYNAMIC_THREAD_ID_START;
		idx < cmdq_dev_get_thread_count(); idx++) {
		/* ignore all static thread */
		if (cmdq_ctx.thread[idx].used)
			continue;

		/* acquire for same scenario */
		if (cmdq_ctx.thread[idx].acquire &&
			cmdq_ctx.thread[idx].scenario != scenario)
			continue;

		/* for exclusvie acquire case, do not match scenario */
		if (exclusive && cmdq_ctx.thread[idx].acquire)
			continue;

		cmdq_ctx.thread[idx].acquire++;
		cmdq_ctx.thread[idx].scenario = scenario;
		thread_id = idx;
		CMDQ_MSG("dispatch scenario:%d to thread:%d acquired:%u\n",
			scenario, idx, cmdq_ctx.thread[idx].acquire);
		break;
	}
	mutex_unlock(&cmdq_thread_mutex);

	if (thread_id == CMDQ_INVALID_THREAD) {
		CMDQ_ERR("unable dispatch scenario:%d dump usage\n", scenario);
		cmdq_core_dump_thread_usage();
	}

	return thread_id;
}

void cmdq_core_release_thread(s32 scenario, s32 thread)
{
	if (thread == CMDQ_INVALID_THREAD) {
		CMDQ_ERR("release invalid thread by scenario:%d\n",
			scenario);
		return;
	}

	mutex_lock(&cmdq_thread_mutex);
	if (!cmdq_ctx.thread[thread].acquire)
		CMDQ_ERR("counter fatal error thread:%d scenario:%d\n",
			thread, scenario);
	else
		cmdq_ctx.thread[thread].acquire--;
	if (scenario != cmdq_ctx.thread[thread].scenario) {
		CMDQ_ERR(
			"use diff scenario to release thread:%d to %d acquire:%d thread:%d\n",
			cmdq_ctx.thread[thread].scenario,
			scenario,
			cmdq_ctx.thread[thread].acquire,
			thread);
		dump_stack();
	}
	if (!cmdq_ctx.thread[thread].acquire)
		CMDQ_MSG("thread:%d released\n", thread);
	mutex_unlock(&cmdq_thread_mutex);
}

static void cmdq_core_group_reset_hw(u64 engine_flag)
{
	struct CmdqCBkStruct *callback = cmdq_group_cb;
	s32 status;
	u32 i;

	CMDQ_LOG("%s engine:0x%llx\n", __func__, engine_flag);

	for (i = 0; i < CMDQ_MAX_GROUP_COUNT; i++) {
		if (cmdq_core_is_group_flag((enum CMDQ_GROUP_ENUM)i,
			engine_flag)) {
			if (!callback[i].resetEng) {
				CMDQ_ERR(
					"no reset func to reset engine group:%u\n",
					i);
				continue;
			}
			status = callback[i].resetEng(
				cmdq_mdp_get_func()->getEngineGroupBits(i) &
				engine_flag);
			if (status < 0) {
				/* Error status print */
				CMDQ_ERR("Reset engine group %d failed:%d\n",
					i, status);
			}
		}
	}
}

static void cmdq_core_group_clk_on(enum CMDQ_GROUP_ENUM group,
	u64 engine_flag)
{
	struct CmdqCBkStruct *callback = cmdq_group_cb;
	s32 status;

	if (!callback[group].clockOn) {
		CMDQ_MSG("[CLOCK][WARN]enable group %d clockOn func NULL\n",
			group);
		return;
	}

	status = callback[group].clockOn(
		cmdq_mdp_get_func()->getEngineGroupBits(group) & engine_flag);
	if (status < 0)
		CMDQ_ERR("[CLOCK]enable group %d clockOn failed\n", group);
}

static void cmdq_core_group_clk_off(enum CMDQ_GROUP_ENUM group,
	u64 engine_flag)
{
	struct CmdqCBkStruct *callback = cmdq_group_cb;
	s32 status;

	if (!callback[group].clockOff) {
		CMDQ_MSG("[CLOCK][WARN]enable group %d clockOff func NULL\n",
			group);
		return;
	}

	status = callback[group].clockOff(
		cmdq_mdp_get_func()->getEngineGroupBits(group) & engine_flag);
	if (status < 0)
		CMDQ_ERR("[CLOCK]enable group %d clockOn failed\n", group);
}

static void cmdq_core_group_clk_cb(bool enable,
	u64 engine_flag, u64 engine_clk)
{
	s32 index;

	/* ISP special check: Always call ISP on/off if this task
	 * involves ISP. Ignore the ISP HW flags.
	 */
	if (cmdq_core_is_group_flag(CMDQ_GROUP_ISP, engine_flag)) {
		if (enable)
			cmdq_core_group_clk_on(CMDQ_GROUP_ISP, engine_flag);
		else
			cmdq_core_group_clk_off(CMDQ_GROUP_ISP, engine_flag);
	}

	for (index = CMDQ_MAX_GROUP_COUNT - 1; index >= 0; index--) {
		/* note that DISPSYS controls their own clock on/off */
		if (index == CMDQ_GROUP_DISP)
			continue;

		/* note that ISP is per-task on/off, not per HW flag */
		if (index == CMDQ_GROUP_ISP)
			continue;

		if (cmdq_core_is_group_flag(index, engine_flag)) {
			if (enable)
				cmdq_core_group_clk_on(index, engine_clk);
			else
				cmdq_core_group_clk_off(index, engine_clk);
		}
	}
}

bool cmdq_thread_in_use(void)
{
	return (bool)(atomic_read(&cmdq_thread_usage) > 0);
}

static void cmdq_core_clk_enable(struct cmdqRecStruct *handle)
{
	s32 clock_count;

	clock_count = atomic_inc_return(&cmdq_thread_usage);

	CMDQ_MSG("[CLOCK]enable usage:%d scenario:%d\n",
		clock_count, handle->scenario);

	if (clock_count == 1)
		cmdq_core_reset_gce();

	cmdq_core_group_clk_cb(true, handle->engineFlag, handle->engine_clk);
}

static void cmdq_core_clk_disable(struct cmdqRecStruct *handle)
{
	s32 clock_count;

	cmdq_core_group_clk_cb(false, handle->engineFlag, handle->engine_clk);

	clock_count = atomic_dec_return(&cmdq_thread_usage);

	CMDQ_MSG("[CLOCK]disable usage:%d\n", clock_count);

	if (clock_count == 0) {
		/* Backup event */
		cmdq_get_func()->eventBackup();
		/* clock-off */
		cmdq_get_func()->enableGCEClockLocked(false);
	} else if (clock_count < 0) {
		CMDQ_ERR(
			"enable clock %s error usage:%d smi use:%d\n",
			__func__, clock_count,
			(s32)atomic_read(&cmdq_thread_usage));
	}
}

s32 cmdq_core_suspend_hw_thread(s32 thread)
{
	struct cmdq_client *client;

	if (thread == CMDQ_INVALID_THREAD) {
		CMDQ_ERR("invalid thread to suspend\n");
		return -EINVAL;
	}

	client = cmdq_clients[thread];
	if (!client) {
		CMDQ_ERR("mbox client not ready for thread:%d suspend\n",
			thread);
		dump_stack();
		return -EINVAL;
	}

	return cmdq_mbox_thread_suspend(client->chan);
}

u64 cmdq_core_get_gpr64(const enum cmdq_gpr_reg regID)
{
	u64 value, value1, value2;
	const u32 x = regID & 0x0F;

	if ((regID & 0x10) > 0) {
		/* query address GPR(64bit), Px */
		value1 = 0LL | CMDQ_REG_GET32(CMDQ_GPR_R32((2 * x)));
		value2 = 0LL | CMDQ_REG_GET32(CMDQ_GPR_R32((2 * x + 1)));
	} else {
		/* query data GPR(32bit), Rx */
		value1 = 0LL | CMDQ_REG_GET32(CMDQ_GPR_R32(x));
		value2 = 0LL;
	}

	value = (0LL) | (value2 << 32) | (value1);
	CMDQ_VERBOSE("get_GPR64(%x):0x%llx(0x%llx, 0x%llx)\n",
		regID, value, value2, value1);

	return value;
}

void cmdq_core_set_gpr64(const enum cmdq_gpr_reg reg_id, const u64 value)
{
	const u64 value1 = 0x00000000FFFFFFFF & value;
	const u64 value2 = 0LL | value >> 32;
	const u32 x = reg_id & 0x0F;
	u64 result;

	if ((reg_id & 0x10) > 0) {
		/* set address GPR(64bit), Px */
		CMDQ_REG_SET32(CMDQ_GPR_R32((2 * x)), value1);
		CMDQ_REG_SET32(CMDQ_GPR_R32((2 * x + 1)), value2);
	} else {
		/* set data GPR(32bit), Rx */
		CMDQ_REG_SET32(CMDQ_GPR_R32((2 * x)), value1);
	}

	result = 0LL | cmdq_core_get_gpr64(reg_id);
	if (value != result) {
		CMDQ_ERR(
			"set GPR64 0x%08x failed value:0x%llx is not value:0x%llx\n",
			reg_id, result, value);
	}
}

static bool cmdq_core_is_thread_cpr(u32 argument)
{
	if (argument >= CMDQ_THR_SPR_MAX && argument < CMDQ_THR_VAR_MAX)
		return true;
	else
		return false;
}

static void cmdq_core_replace_arg_cpr(u32 *cmd, u32 cpr_offset)
{
	u32 arg_header = (cmd[1] >> 16) & 0xFFFF;
	u32 arg_a_i = cmd[1] & 0xFFFF;
	u32 arg_b_i = (cmd[0] >> 16) & 0xFFFF;
	u32 arg_c_i = cmd[0] & 0xFFFF;
	u32 arg_a_type = cmd[1] & (1 << 23);
	u32 arg_b_type = cmd[1] & (1 << 22);
	u32 arg_c_type = cmd[1] & (1 << 21);

	CMDQ_MSG("replace instruction 0x%p: 0x%08x 0x%08x\n",
		cmd, cmd[0], cmd[1]);

	if (arg_a_type != 0 && cmdq_core_is_thread_cpr(arg_a_i)) {
		arg_a_i = cpr_offset + (arg_a_i - CMDQ_THR_SPR_MAX);
		cmd[1] = (arg_header<<16) | (arg_a_i & 0xFFFF);
	}

	if (arg_b_type != 0 && cmdq_core_is_thread_cpr(arg_b_i))
		arg_b_i = cpr_offset + (arg_b_i - CMDQ_THR_SPR_MAX);

	if (arg_c_type != 0 && cmdq_core_is_thread_cpr(arg_c_i))
		arg_c_i = cpr_offset + (arg_c_i - CMDQ_THR_SPR_MAX);

	cmd[0] = (arg_b_i<<16) | (arg_c_i & 0xFFFF);

	CMDQ_MSG("replace result: 0x%p: 0x%08x 0x%08x\n",
		cmd, cmd[0], cmd[1]);
}

static inline u32 cmdq_core_get_subsys_id(u32 arg_a)
{
	return (arg_a & CMDQ_ARG_A_SUBSYS_MASK) >> subsys_lsb_bit;
}

static void cmdq_core_replace_overwrite_instr(struct cmdqRecStruct *handle,
	u32 index)
{
	/* check if this is overwrite instruction */
	u32 *p_cmd_assign = (u32 *)cmdq_pkt_get_va_by_offset(handle->pkt,
		(index - 1) * CMDQ_INST_SIZE);

	if (!p_cmd_assign) {
		CMDQ_ERR("cannot find overwrite instr in %s\n", __func__);
		return;
	}

	if ((p_cmd_assign[1] >> 24) == CMDQ_CODE_LOGIC &&
		((p_cmd_assign[1] >> 23) & 1) == 1 &&
		cmdq_core_get_subsys_id(p_cmd_assign[1]) ==
		CMDQ_LOGIC_ASSIGN &&
		(p_cmd_assign[1] & 0xFFFF) == CMDQ_SPR_FOR_TEMP) {
		u32 overwrite_index = p_cmd_assign[0];
		dma_addr_t overwrite_pa = cmdq_pkt_get_pa_by_offset(
			handle->pkt, overwrite_index * CMDQ_INST_SIZE);

		p_cmd_assign[0] = (u32) overwrite_pa;
		CMDQ_LOG("overwrite: original index:%d, change to PA:%pa\n",
			overwrite_index, &overwrite_pa);
	}
}

static void cmdq_core_v3_adjust_jump(struct cmdqRecStruct *handle,
	u32 instr_pos, u32 *va)
{
	u32 offset;
	dma_addr_t pa;

	if (va[1] & 0x1)
		return;

	offset = instr_pos * CMDQ_INST_SIZE + CMDQ_REG_REVERT_ADDR(va[0]);
	if (offset == handle->pkt->buf_size) {
		/* offset to last instruction may not adjust,
		 * return back 1 instruction
		 */
		offset = offset - CMDQ_INST_SIZE;
	}

	/* change to absolute to avoid jump cross buffer */
	pa = cmdq_pkt_get_pa_by_offset(handle->pkt, offset);
	CMDQ_MSG("Replace jump from:0x%08x to offset:%u\n", va[0], offset);
	va[1] = va[1] | 0x1;
	va[0] = CMDQ_PHYS_TO_AREG(CMDQ_REG_SHIFT_ADDR(pa));
}

static void cmdq_core_v3_replace_arg(struct cmdqRecStruct *handle, u32 *va,
	u32 thread_offset, u32 arg_op_code, u32 inst_idx)
{
	u32 arg_a_i;
	u32 arg_a_type;

	cmdq_core_replace_arg_cpr(va, thread_offset);
	arg_a_type = va[1] & (1 << 23);
	arg_a_i = va[1] & 0xFFFF;

	if (arg_op_code == CMDQ_CODE_WRITE_S && inst_idx > 0 &&
		arg_a_type != 0 && arg_a_i == CMDQ_SPR_FOR_TEMP)
		cmdq_core_replace_overwrite_instr(handle, inst_idx);
}

static void cmdq_core_v3_replace_jumpc(struct cmdqRecStruct *handle,
	u32 *va, u32 inst_idx, u32 inst_pos)
{
	u32 *p_cmd_logic = (u32 *)cmdq_pkt_get_va_by_offset(handle->pkt,
		(inst_idx - 1) * CMDQ_INST_SIZE);
	bool revise_offset = false;

	if (!p_cmd_logic) {
		CMDQ_ERR("cannot find jump cmd in %s\n", __func__);
		return;
	}

	/* logic and jump relative maybe separate by jump cross buffer */
	if (p_cmd_logic[1] == ((CMDQ_CODE_JUMP << 24) | 1)) {
		CMDQ_MSG(
			"Re-adjust new logic instruction due to jump cross buffer:%u\n",
			inst_idx);
		p_cmd_logic = (u32 *)cmdq_pkt_get_va_by_offset(
			handle->pkt, (inst_idx - 2) * CMDQ_INST_SIZE);
		revise_offset = true;
	}

	if ((p_cmd_logic[1] >> 24) == CMDQ_CODE_LOGIC &&
		((p_cmd_logic[1] >> 16) & 0x1F) == CMDQ_LOGIC_ASSIGN) {
		u32 jump_op = CMDQ_CODE_JUMP_C_ABSOLUTE << 24;
		u32 jump_op_header = va[1] & 0xFFFFFF;
		u32 jump_offset = CMDQ_REG_REVERT_ADDR(p_cmd_logic[0]);
		u32 offset_target = (revise_offset) ?
			(inst_pos - 1) * CMDQ_INST_SIZE + jump_offset :
			inst_pos * CMDQ_INST_SIZE + jump_offset;
		dma_addr_t dest_addr_pa;

		dest_addr_pa = cmdq_pkt_get_pa_by_offset(
			handle->pkt, offset_target);

		if (dest_addr_pa == 0) {
			cmdq_core_dump_handle(handle, "ERR");
			CMDQ_AEE("CMDQ",
				"Wrong PA offset, task:0x%p, inst idx:0x%08x cmd:0x%08x:%08x addr:0x%p\n",
				handle, inst_idx, p_cmd_logic[1],
				p_cmd_logic[0], p_cmd_logic);
			return;
		}

		p_cmd_logic[0] = CMDQ_PHYS_TO_AREG(
			CMDQ_REG_SHIFT_ADDR(dest_addr_pa));
		va[1] = jump_op | jump_op_header;

		CMDQ_MSG(
			"Replace jump_c inst idx:%u(%u) cmd:0x%p %#016llx logic:0x%p %#016llx\n",
			inst_idx, inst_pos,
			va, *(u64 *)va,
			p_cmd_logic, *(u64 *)p_cmd_logic);
	} else {
		/* unable to jump correctly since relative offset
		 * may cross page
		 */
		CMDQ_ERR(
			"No logic before jump, handle:0x%p, inst idx:0x%08x cmd:0x%p 0x%08x:%08x logic:0x%p 0x%08x:0x%08x\n",
			handle, inst_idx,
			va, va[1], va[0],
			p_cmd_logic, p_cmd_logic[1], p_cmd_logic[0]);
	}
}

void cmdq_core_replace_v3_instr(struct cmdqRecStruct *handle, s32 thread)
{
	u32 i;
	u32 arg_op_code;
	u32 *p_cmd_va;
	u32 thread_offset;
	u32 *p_instr_position = CMDQ_U32_PTR(handle->replace_instr.position);
	u32 inst_idx = 0;
	const u32 boundary_idx = (handle->pkt->buf_size / CMDQ_INST_SIZE);

	if (handle->replace_instr.number == 0)
		return;

	CMDQ_MSG("replace_instr.number:%u\n", handle->replace_instr.number);
	thread_offset = CMDQ_CPR_STRAT_ID + CMDQ_THR_CPR_MAX * thread;

	for (i = 0; i < handle->replace_instr.number; i++) {
		if ((p_instr_position[i] + 1) * CMDQ_INST_SIZE >
			handle->pkt->cmd_buf_size) {
			CMDQ_ERR(
				"Incorrect replace instruction position, index (%d), size (%zu)\n",
				p_instr_position[i],
				handle->pkt->cmd_buf_size);
			break;
		}

		inst_idx = p_instr_position[i];
		if (inst_idx == boundary_idx) {
			/* last jump instruction may not adjust,
			 * return back 1 instruction
			 */
			inst_idx--;
		}

		p_cmd_va = (u32 *)cmdq_pkt_get_va_by_offset(handle->pkt,
			inst_idx * CMDQ_INST_SIZE);

		if (!p_cmd_va) {
			CMDQ_AEE("CMDQ",
				"Cannot find va, task:0x%p idx:%u/%u instruction idx:%u(%u) size:%zu+%zu\n",
				handle, i, handle->replace_instr.number,
				inst_idx, p_instr_position[i],
				handle->pkt->buf_size,
				handle->pkt->avail_buf_size);
			cmdq_core_dump_handle_buffer(handle->pkt, "ERR");
			continue;
		}

		arg_op_code = p_cmd_va[1] >> 24;
		if (arg_op_code == CMDQ_CODE_JUMP) {
			if (!handle->ctrl->change_jump)
				continue;
			cmdq_core_v3_adjust_jump(handle,
				(u32)p_instr_position[i], p_cmd_va);
		} else {
			CMDQ_MSG(
				"replace instruction: (%d)0x%p:0x%08x 0x%08x\n",
				i, p_cmd_va, p_cmd_va[0], p_cmd_va[1]);
			cmdq_core_v3_replace_arg(handle, p_cmd_va,
				thread_offset, arg_op_code, inst_idx);
		}

		if (arg_op_code == CMDQ_CODE_JUMP_C_RELATIVE) {
			if (!handle->ctrl->change_jump)
				continue;
			cmdq_core_v3_replace_jumpc(handle, p_cmd_va, inst_idx,
				p_instr_position[i]);
		}
	}
}

void cmdq_core_release_handle_by_file_node(void *file_node)
{
	struct cmdqRecStruct *handle;
	struct cmdq_client *client;

	mutex_lock(&cmdq_handle_list_mutex);
	list_for_each_entry(handle, &cmdq_ctx.handle_active, list_entry) {
		if (!(handle->state != TASK_STATE_IDLE &&
			handle->node_private == file_node))
			continue;
		CMDQ_LOG(
			"[warn]running handle 0x%p auto release because file node 0x%p closed\n",
			handle, file_node);

		/* since we already inside mutex, do not release directly,
		 * instead we change state to "KILLED"
		 * and arrange a auto-release.
		 * Note that these tasks may already issued to HW
		 * so there is a chance that following MPU/M4U
		 * violation may occur, if the user space process has
		 * destroyed.
		 * The ideal solution is to stop / cancel HW operation
		 * immediately, but we cannot do so due to SMI hang risk.
		 */
		client = cmdq_clients[handle->thread];
		cmdq_mbox_thread_remove_task(client->chan, handle->pkt);
		cmdq_pkt_auto_release_task(handle);
	}
	mutex_unlock(&cmdq_handle_list_mutex);
}

s32 cmdq_core_suspend(void)
{
	s32 ref_count;
	u32 exec_thread = 0;
	bool kill_task = false;

	ref_count = atomic_read(&cmdq_thread_usage);
	if (ref_count)
		exec_thread = CMDQ_REG_GET32(CMDQ_CURR_LOADED_THR);
	CMDQ_LOG("%s usage:%d exec thread:0x%x\n",
		__func__, ref_count, exec_thread);

	if (cmdq_get_func()->moduleEntrySuspend(cmdq_mdp_get_engines()) < 0) {
		CMDQ_ERR(
			"[SUSPEND] MDP running, kill tasks. threads:0x%08x ref:%d\n",
			exec_thread, ref_count);
		kill_task = true;
	} else if ((ref_count > 0) || (0x80000000 & exec_thread)) {
		CMDQ_ERR(
			"[SUSPEND] other running, kill tasks. threads:0x%08x ref:%d\n",
			exec_thread, ref_count);
		kill_task = true;
	}

	atomic_set(&cmdq_sec_dbg_ctrl, 0);

	/* TODO:
	 * We need to ensure the system is ready to suspend,
	 * so kill all running CMDQ tasks
	 * and release HW engines.
	 */
	if (kill_task) {
#if 0
		CMDQ_PROF_MUTEX_LOCK(gCmdqTaskMutex, suspend_kill);

		/* print active tasks */
		CMDQ_ERR("[SUSPEND] active tasks during suspend:\n");
		list_for_each(p, &cmdq_ctx.taskActiveList) {
			pTask = list_entry(p, struct TaskStruct, listEntry);
			if (cmdq_core_is_valid_in_active_list(pTask))
				cmdq_core_dump_task(pTask);
		}

		/* remove all active task from thread */
		CMDQ_ERR("[SUSPEND] remove all active tasks\n");
		list_for_each(p, &cmdq_ctx.taskActiveList) {
			pTask = list_entry(p, struct TaskStruct, listEntry);

			if (pTask->thread != CMDQ_INVALID_THREAD) {
				spin_lock_irqsave(&gCmdqExecLock, flags,
					suspend_loop);

				CMDQ_ERR("[SUSPEND] release task:0x%p\n",
					pTask);

				cmdq_core_force_remove_task_from_thread(pTask,
					pTask->thread);
				pTask->taskState = TASK_STATE_KILLED;

				spin_unlock_irqrestore(&gCmdqExecLock, flags,
					suspend_loop);

				/* release all thread and mark active tasks
				 * as "KILLED" (so that thread won't release
				 * again)
				 */
				CMDQ_ERR(
					"[SUSPEND] release all threads and HW clocks\n");
				cmdq_core_release_thread(pTask);
			}
		}

		CMDQ_PROF_MUTEX_UNLOCK(gCmdqTaskMutex, suspend_kill);

		/* TODO: skip secure path thread... */
		/* disable all HW thread */
		CMDQ_ERR("[SUSPEND] disable all HW threads\n");
		for (i = 0; i < max_thread_count; i++)
			cmdq_core_disable_HW_thread(i);

		/* reset all threadStruct */
		cmdq_core_reset_thread_struct();

		/* reset all engineStruct */
		memset(&cmdq_ctx.engine[0], 0,
			sizeof(cmdq_ctx.engine));
		cmdq_core_reset_engine_struct();
#endif
	}

	CMDQ_MSG("CMDQ is suspended\n");
#ifdef CMDQ_TIMER_ENABLE
	/* set to true to disable delay thread when boot */
	cmdq_delay_thd_inited = false;
	cmdq_helper_mbox_clear_pools();
#endif

	/* ALWAYS allow suspend */
	return 0;
}

s32 cmdq_core_resume(void)
{
	CMDQ_VERBOSE("[RESUME] do nothing\n");
	/* do nothing */
	return 0;
}

s32 cmdq_core_resume_notifier(void)
{
	s32 ref_count;

	CMDQ_LOG("%s\n", __func__);

	/* TEE project limitation:
	 * .t-base daemon process is available after process-unfreeze
	 * .need t-base daemon for communication to secure world
	 * .M4U port security setting backup/resore needs to entry sec world
	 * .M4U port security setting is access normal PA
	 *
	 * Delay resume timing until process-unfreeze done in order to
	 * ensure M4U driver had restore M4U port security setting
	 */

	ref_count = atomic_read(&cmdq_thread_usage);

#ifdef CMDQ_TIMER_ENABLE
	if (!cmdq_delay_thd_inited) {
		cmdq_copy_delay_to_sram();
		CMDQ_MSG("resume, after copy delay to SRAM\n");
		cmdq_delay_thd_inited = true;
	}
#endif

	cmdq_mdp_resume();

	return 0;
}

void cmdq_core_set_spm_mode(enum CMDQ_SPM_MODE mode)
{
	CMDQ_MSG("before setting, reg:0x%x\n",
		CMDQ_REG_GET32(CMDQ_H_SPEED_BUSY));

	switch (mode) {
	case CMDQ_CG_MODE:
		CMDQ_REG_SET32(CMDQ_H_SPEED_BUSY, 0x20002);
		break;
	case CMDQ_PD_MODE:
		CMDQ_REG_SET32(CMDQ_H_SPEED_BUSY, 0x3);
		break;
	default:
		break;
	}

	CMDQ_MSG("after setting, reg:0x%x\n",
		CMDQ_REG_GET32(CMDQ_H_SPEED_BUSY));
}

struct cmdq_dts_setting *cmdq_core_get_dts_setting(void)
{
	return &cmdq_dts;
}

struct ContextStruct *cmdq_core_get_context(void)
{
	return &cmdq_ctx;
}

struct CmdqCBkStruct *cmdq_core_get_group_cb(void)
{
	return cmdq_group_cb;
}

dma_addr_t cmdq_core_get_pc(s32 thread)
{
	dma_addr_t pc = CMDQ_REG_GET32(CMDQ_THR_CURR_ADDR(thread));

	if (CMDQ_IS_SRAM_ADDR(pc))
		return pc;
	return CMDQ_REG_REVERT_ADDR(pc);
}

dma_addr_t cmdq_core_get_end(s32 thread)
{
	dma_addr_t end = CMDQ_REG_GET32(CMDQ_THR_END_ADDR(thread));

	if (CMDQ_IS_SRAM_ADDR(end))
		return end;
	return CMDQ_REG_REVERT_ADDR(end);
}

unsigned long cmdq_get_tracing_mark(void)
{
	static unsigned long __read_mostly tracing_mark_write_addr;

	if (unlikely(tracing_mark_write_addr == 0))
		tracing_mark_write_addr =
			kallsyms_lookup_name("tracing_mark_write");

	return tracing_mark_write_addr;
}

static s32 cmdq_pkt_handle_wait_result(struct cmdqRecStruct *handle, s32 thread)
{
	s32 ret = 0;

	/* set to timeout if state change to */
	if (handle->state == TASK_STATE_TIMEOUT) {
		/* timeout info already dump during callback */
		ret = -ETIMEDOUT;
	} else if (handle->state == TASK_STATE_ERR_IRQ) {
		/* dump current buffer for trace */
		ret = -EINVAL;
		CMDQ_ERR("dump error irq handle:0x%p pc:0x%08x\n",
			handle, handle->error_irq_pc);
		cmdq_core_dump_error_buffer(handle, NULL,
			handle->error_irq_pc);
		CMDQ_AEE("CMDQ", "Error IRQ\n");

	}

	return ret;
}

/* core controller function */
static const struct cmdq_controller g_cmdq_core_ctrl = {
#if 0
	.compose = cmdq_core_task_compose,
	.copy_command = cmdq_core_task_copy_command,
#endif
	.get_thread_id = cmdq_core_get_thread_id,
#if 0
	.execute_prepare = cmdq_core_exec_task_prepare,
	.execute = cmdq_core_exec_task_async_impl,
	.handle_wait_result = cmdq_core_handle_wait_task_result_impl,
	.free_buffer = cmdq_core_task_free_buffer_impl,
	.append_command = cmdq_core_append_command,
	.dump_err_buffer = cmdq_core_dump_err_buffer,
	.dump_summary = cmdq_core_dump_summary,
#endif
	.handle_wait_result = cmdq_pkt_handle_wait_result,
	.change_jump = true,
};

const struct cmdq_controller *cmdq_core_get_controller(void)
{
	return &g_cmdq_core_ctrl;
}

static bool cmdq_pkt_is_buffer_size_valid(const struct cmdqRecStruct *handle)
{
	return (handle->pkt->cmd_buf_size % CMDQ_CMD_BUFFER_SIZE ==
		(handle->pkt->avail_buf_size > 0 ?
		CMDQ_CMD_BUFFER_SIZE - handle->pkt->avail_buf_size : 0));
}

s32 cmdq_pkt_get_cmd_by_pc(const struct cmdqRecStruct *handle, u32 pc,
	u32 *inst_out, u32 size)
{
	void *va = NULL;
	u32 i = 0;
	struct cmdq_pkt_buffer *buf;
	bool found = false;

	if (!inst_out)
		return -EINVAL;

	CMDQ_MSG("%s handle:0x%p pc:0x%08x inst_out:0x%p size:%d\n",
		__func__, handle, pc, inst_out, size);

	list_for_each_entry(buf, &handle->pkt->buf, list_entry) {
		if (!found && !(pc >= buf->pa_base &&
			pc < buf->pa_base + CMDQ_CMD_BUFFER_SIZE))
			continue;

		if (!found)
			va = buf->va_base + (pc - buf->pa_base);
		else
			va = buf->va_base;
		found = true;

		for (i = 0; i < size; i++, va += 4) {
			if (va >= buf->va_base + CMDQ_CMD_BUFFER_SIZE)
				break;
			inst_out[i] = *((u32 *)va);
		}

		/* maybe next buffer */
		size -= i;
		if (!size)
			break;
	}

	return 0;
}

void cmdq_pkt_get_first_buffer(struct cmdqRecStruct *handle,
	void **va_out, dma_addr_t *pa_out)
{
	struct cmdq_pkt_buffer *buf;

	if (list_empty(&handle->pkt->buf))
		return;
	buf = list_first_entry(&handle->pkt->buf, typeof(*buf), list_entry);
	*va_out = buf->va_base;
	*pa_out = buf->pa_base;
}

void *cmdq_pkt_get_first_va(const struct cmdqRecStruct *handle)
{
	struct cmdq_pkt_buffer *buf;

	if (list_empty(&handle->pkt->buf))
		return NULL;
	buf = list_first_entry(&handle->pkt->buf, typeof(*buf), list_entry);
	return buf->va_base;
}

static s32 cmdq_pkt_copy_buffer(void *dst, void *src, const u32 size,
	const bool is_copy_from_user)
{
	s32 status = 0;

	if (!is_copy_from_user) {
		CMDQ_VERBOSE("copy kernel to 0x%p\n", dst);
		memcpy(dst, src, size);
	} else {
		CMDQ_VERBOSE("copy user to 0x%p\n", dst);
		if (copy_from_user(dst, src, size)) {
			CMDQ_AEE("CMDQ",
				"CRDISPATCH_KEY:CMDQ Fail to copy from user 0x%p, size:%d\n",
				src, size);
			status = -ENOMEM;
		}
	}

	return status;
}

s32 cmdq_pkt_copy_cmd(struct cmdqRecStruct *handle, void *src, const u32 size,
	const bool is_copy_from_user)
{
	s32 status = 0;
	u32 remaind_cmd_size = size;
	u32 copy_size = 0;
	struct cmdq_pkt *pkt = NULL;
	void *va;
	struct cmdq_pkt_buffer *buf;


	if (!handle) {
		CMDQ_ERR("%s invalid param: handle is NULL\n", __func__);
		return -EINVAL;
	}
	pkt = handle->pkt;

	while (remaind_cmd_size > 0) {
		/* extend buffer to copy more instruction */
		if (!handle->pkt->avail_buf_size) {
			if (cmdq_pkt_add_cmd_buffer(handle->pkt) < 0)
				return -ENOMEM;
		}

		copy_size = pkt->avail_buf_size > remaind_cmd_size ?
			remaind_cmd_size : pkt->avail_buf_size;
		buf = list_last_entry(&pkt->buf, typeof(*buf), list_entry);
		va = buf->va_base + CMDQ_CMD_BUFFER_SIZE - pkt->avail_buf_size;
		status = cmdq_pkt_copy_buffer(va,
			src + size - remaind_cmd_size,
			copy_size, is_copy_from_user);
		if (status < 0)
			return status;

		/* update last instruction position */
		pkt->avail_buf_size -= copy_size;
		pkt->cmd_buf_size += copy_size;
		remaind_cmd_size -= copy_size;
		handle->cmd_end = buf->va_base + CMDQ_CMD_BUFFER_SIZE -
			pkt->avail_buf_size - CMDQ_INST_SIZE;

		if (unlikely(!cmdq_pkt_is_buffer_size_valid(handle))) {
			/* buffer size is total size and should sync
			 * with available space
			 */
			CMDQ_AEE("CMDQ",
				"buf size:%zu size:%zu available size:%zu of %u end cmd:0x%p first va:0x%p out of sync!\n",
				pkt->buf_size, pkt->cmd_buf_size,
				pkt->avail_buf_size,
				(u32)CMDQ_CMD_BUFFER_SIZE,
				handle->cmd_end,
				cmdq_pkt_get_first_va(handle));
			cmdq_core_dump_handle(handle, "ERR");
		}

		CMDQ_MSG(
			"buf size:%zu size:%zu available:%zu end:0x%p va:0x%p\n",
			pkt->buf_size, pkt->cmd_buf_size, pkt->avail_buf_size,
			handle->cmd_end, va);
	}

	return status;
}

static s32 cmdq_pkt_lock_handle(struct cmdqRecStruct *handle,
	struct cmdq_client **client_out)
{
	struct cmdq_client *client;
	s32 ref;

	if (handle->thread == CMDQ_INVALID_THREAD) {
		CMDQ_ERR("invalid thread handle:0x%p scenario:%d\n",
			handle, handle->scenario);
		return -EINVAL;
	}

	client = cmdq_clients[handle->thread];
	if (!client) {
		CMDQ_ERR("mbox client not ready for thread:%d\n",
			handle->thread);
		dump_stack();
		return -EINVAL;
	}

	ref = atomic_inc_return(&handle->exec);
	if (ref > 1) {
		atomic_dec(&handle->exec);
		CMDQ_LOG(
			"[warn]handle already executed:0x%p ref:%d scenario:%u\n",
			handle, ref, handle->scenario);
		return -EBUSY;
	}

	CMDQ_MSG("lock handle:0x%p thread:%d engine:0x%llx scenario:%u\n",
		handle, handle->thread, handle->res_flag_acquire,
		handle->scenario);

	/* change state to busy */
	handle->state = TASK_STATE_BUSY;
	handle->gotIRQ = 0;

	*client_out = client;

	/* protect multi-thread lock/unlock same time */
	mutex_lock(&cmdq_clock_mutex);

	/* callback clients we are about to start handle in gce */
	handle->prepare(handle);

	/* task and thread dispatched, increase usage */
	cmdq_core_clk_enable(handle);

	/* Start delay thread before first task */
#ifdef CMDQ_TIMER_ENABLE
	cmdq_delay_thread_start();
#endif
	mutex_unlock(&cmdq_clock_mutex);

	mutex_lock(&cmdq_handle_list_mutex);
	list_add_tail(&handle->list_entry, &cmdq_ctx.handle_active);
	handle->check_list_del = 1;
	mutex_unlock(&cmdq_handle_list_mutex);

	return 0;
}

static void cmdq_core_group_begin_task(struct cmdqRecStruct *handle,
	struct cmdqRecStruct **handle_list, u32 size)
{
	enum CMDQ_GROUP_ENUM group = 0;

	for (group = 0; group < CMDQ_MAX_GROUP_COUNT; group++) {
		if (!cmdq_group_cb[group].beginTask ||
			!cmdq_core_is_group_flag(group, handle->engineFlag))
			continue;
		cmdq_group_cb[group].beginTask(handle, handle_list, size);
	}
}

static void cmdq_core_group_end_task(struct cmdqRecStruct *handle,
	struct cmdqRecStruct **handle_list, u32 size)
{
	enum CMDQ_GROUP_ENUM group = 0;

	for (group = 0; group < CMDQ_MAX_GROUP_COUNT; group++) {
		if (!cmdq_group_cb[group].endTask ||
			!cmdq_core_is_group_flag(group, handle->engineFlag))
			continue;
		cmdq_group_cb[group].endTask(handle, handle_list, size);
	}
}

static s32 cmdq_core_get_pmqos_handle_list(struct cmdqRecStruct *handle,
	struct cmdqRecStruct **handle_out, u32 handle_list_size)
{
	struct cmdq_client *client;
	struct cmdq_pkt **pkt_list = NULL;
	u32 i;
	u32 pkt_count;

	if (!handle || !handle_out || !handle_list_size)
		return -EINVAL;

	pkt_list = kcalloc(handle_list_size, sizeof(*pkt_list), GFP_KERNEL);
	if (!pkt_list)
		return -ENOMEM;

	client = cmdq_clients[handle->thread];

	cmdq_task_get_pkt_from_thread(client->chan, pkt_list,
		handle_list_size, &pkt_count);

	/* get handle from user_data */
	for (i = 0; i < pkt_count; i++) {
		if (!pkt_list[i])
			continue;
		handle_out[i] = pkt_list[i]->user_data;
	}

	kfree(pkt_list);
	return 0;
}

void cmdq_pkt_release_handle(struct cmdqRecStruct *handle)
{
	s32 ref;
	struct ContextStruct *ctx;
	struct cmdqRecStruct **pmqos_handle_list = NULL;
	u32 handle_count;

	CMDQ_MSG("release handle:0x%p pkt:0x%p thread:%d engine:0x%llx\n",
		handle, handle->pkt, handle->thread,
		handle->res_flag_release);

	ref = atomic_dec_return(&handle->exec);
	if (ref != 0) {
		CMDQ_ERR("handle state not right:0x%p ref:%d thread:%d\n",
			handle, ref, handle->thread);
		mutex_lock(&cmdq_handle_list_mutex);
		list_del_init(&handle->list_entry);
		mutex_unlock(&cmdq_handle_list_mutex);
		dump_stack();
		return;
	}

	cmdq_core_track_handle_record(handle, handle->thread);

	/* TODO: remove is_secure flag */
	if (handle->thread != CMDQ_INVALID_THREAD &&
		!handle->secData.is_secure) {
		/* PMQoS Implement */
		mutex_lock(&cmdq_thread_mutex);
		ctx = cmdq_core_get_context();
		handle_count = --ctx->thread[handle->thread].handle_count;

		if (handle_count) {
			pmqos_handle_list = kcalloc(handle_count + 1,
				sizeof(*pmqos_handle_list), GFP_KERNEL);

			if (pmqos_handle_list)
				cmdq_core_get_pmqos_handle_list(handle,
					pmqos_handle_list, handle_count);
		}

		cmdq_core_group_end_task(handle, pmqos_handle_list,
			handle_count);

		kfree(pmqos_handle_list);
		mutex_unlock(&cmdq_thread_mutex);
	}

#ifdef CMDQ_TIMER_ENABLE
	/* Stop delay thread after last task is done */
	cmdq_delay_thread_stop();
#endif

	/* protect multi-thread lock/unlock same time */
	mutex_lock(&cmdq_clock_mutex);

	/* callback clients this thread about to clean */
	handle->unprepare(handle);

	/* before stop job, decrease usage */
	cmdq_core_clk_disable(handle);

	/* stop callback */
	if (handle->stop)
		handle->stop(handle);

	mutex_unlock(&cmdq_clock_mutex);

	mutex_lock(&cmdq_handle_list_mutex);
	list_del_init(&handle->list_entry);
	handle->check_list_del = 2;
	mutex_unlock(&cmdq_handle_list_mutex);
}

static void cmdq_pkt_err_dump_handler(struct cmdq_cb_data data)
{
	struct cmdqRecStruct *handle = (struct cmdqRecStruct *)data.data;

	if (!handle)
		return;

	if (!handle->pkt->loop && handle->gotIRQ) {
		CMDQ_ERR(
			"%s handle:0x%p irq already processed:%llu scene:%d thread:%d\n",
			__func__, handle, handle->gotIRQ, handle->scenario,
			handle->thread);
		return;
	}

	CMDQ_PROF_MMP(cmdq_mmp_get_event()->timeout, MMPROFILE_FLAG_START,
		(unsigned long)handle, handle->thread);

	if (data.err == -ETIMEDOUT) {
		atomic_inc(&handle->exec);
		cmdq_core_attach_error_handle_by_state(handle,
			TASK_STATE_TIMEOUT);
		atomic_dec(&handle->exec);
	} else if (data.err == -EINVAL) {
		/* store PC for later dump buffer */
		handle->error_irq_pc = cmdq_core_get_pc(handle->thread);
	}

	CMDQ_PROF_MMP(cmdq_mmp_get_event()->timeout, MMPROFILE_FLAG_END,
		(unsigned long)handle, handle->thread);
}

static void cmdq_pkt_flush_handler(struct cmdq_cb_data data)
{
	struct cmdqRecStruct *handle = (struct cmdqRecStruct *)data.data;
	struct cmdq_client *client = NULL;

	if (!handle)
		return;

	if (!handle->pkt->loop && handle->gotIRQ) {
		CMDQ_ERR(
			"%s handle:0x%p irq already processed:%llu scene:%d thread:%d\n",
			__func__, handle, handle->gotIRQ, handle->scenario,
			handle->thread);
		return;
	}

	if (atomic_read(&handle->exec) > 1) {
		CMDQ_ERR("irq during error dump:%d err:%d\n",
			atomic_read(&handle->exec), data.err);
		dump_stack();
		return;
	}

	handle->gotIRQ = sched_clock();

	/* revise log level for debug */
	if (data.err && data.err != -ECONNABORTED && !handle->pkt->loop) {
		CMDQ_ERR(
			"flush handler handle:0x%p err:%d thread:%d\n",
			data.data, data.err, handle->thread);
	} else {
		CMDQ_MSG(
			"flush handler handle:0x%p err:%d loop cb:0x%p thread:%d\n",
			data.data, data.err, handle->loop_cb, handle->thread);
	}

	if (handle->pkt->loop) {
		s32 loop_ret = 0;

		if (handle->loop_cb) {
			/* for loop task only callback and no need to do more */
			CMDQ_VERBOSE("loop callback handle:0x%p\n", handle);
			loop_ret = handle->loop_cb(handle->loop_user_data);
		}

		CMDQ_PROF_MMP(cmdq_mmp_get_event()->loopBeat,
			MMPROFILE_FLAG_PULSE, handle->thread, loop_ret);

		if (data.err == -ECONNABORTED) {
			/* loop stopped */
			handle->state = TASK_STATE_KILLED;
		}

		return;
	}

	client = cmdq_clients[handle->thread];

	if (data.err == -ETIMEDOUT) {
		/* error dump may processed on error handler */
		handle->state = TASK_STATE_TIMEOUT;
	} else if (data.err == -ECONNABORTED) {
		/* task killed */
		handle->state = TASK_STATE_KILLED;
	} else if (data.err == -EINVAL) {
		/* IRQ with error, dump commands */
		handle->state = TASK_STATE_ERR_IRQ;
	} else if (data.err < 0) {
		/* unknown error print it */
		handle->state = TASK_STATE_KILLED;
		CMDQ_ERR("%s error code:%d\n", __func__, data.err);
	} else {
		/* success done */
		handle->state = TASK_STATE_DONE;
	}

	/* reset hardware in timeout or error irq case,
	 * so that hardware may work again on next task.
	 */
	if (handle->state == TASK_STATE_TIMEOUT ||
		handle->state == TASK_STATE_ERR_IRQ)
		cmdq_core_group_reset_hw(handle->engineFlag);

	CMDQ_PROF_MMP(cmdq_mmp_get_event()->CMDQ_IRQ, MMPROFILE_FLAG_PULSE,
		(unsigned long)handle, handle->thread);

	wake_up(&cmdq_wait_queue[handle->thread]);
}

s32 cmdq_pkt_dump_command(struct cmdqRecStruct *handle)
{
	const void *inst;
	u32 size;
	static char text[128] = { 0 };
	const struct cmdq_pkt *pkt = handle->pkt;
	struct cmdq_pkt_buffer *buf;
	u32 cnt = 0;

	CMDQ_LOG("===== REC 0x%p command buffer =====\n", handle);
	cmdq_core_dump_error_buffer(handle, NULL, 0);
	CMDQ_LOG("===== REC 0x%p command buffer END =====\n", handle);

	list_for_each_entry(buf, &pkt->buf, list_entry) {
		if (list_is_last(&buf->list_entry, &pkt->buf)) {
			size = CMDQ_CMD_BUFFER_SIZE - pkt->avail_buf_size;
		} else if (cnt > 2) {
			CMDQ_LOG(
				"buffer %u va:0x%p pa:%pa 0x%016llx (skip detail) 0x%016llx\n",
				cnt, buf->va_base, &buf->pa_base,
				*((u64 *)buf->va_base),
				*((u64 *)(buf->va_base +
				CMDQ_CMD_BUFFER_SIZE - CMDQ_INST_SIZE)));
			cnt++;
			continue;
		} else {
			size = CMDQ_CMD_BUFFER_SIZE;
		}
		CMDQ_LOG("buffer %u va:0x%p pa:%pa\n",
			cnt, buf->va_base, &buf->pa_base);
		for (inst = buf->va_base; inst < buf->va_base + size;
			inst += CMDQ_INST_SIZE) {
			cmdq_core_parse_instruction(inst, text, 128);
			CMDQ_LOG("0x%p %s", inst, text);
		}
		cnt++;
	}

	CMDQ_LOG("======REC 0x%p command END\n", handle);

	return 0;
}

s32 cmdq_pkt_wait_flush_ex_result(struct cmdqRecStruct *handle)
{
	s32 waitq;
	s32 status = 0;
	u32 count = 0;
	const struct cmdq_controller *ctrl = handle->ctrl;

	CMDQ_PROF_MMP(cmdq_mmp_get_event()->wait_task,
		MMPROFILE_FLAG_PULSE, ((unsigned long)handle), handle->thread);

	CMDQ_SYSTRACE_BEGIN("%s_wait_done\n", __func__);
	handle->beginWait = sched_clock();

	if (!cmdq_clients[handle->thread])
		return -EINVAL;

	do {
		if (!handle->pkt->loop) {
			/* wait event and pre-dump */
			waitq = wait_event_timeout(
				cmdq_wait_queue[handle->thread],
				(handle->state != TASK_STATE_BUSY &&
				handle->state != TASK_STATE_WAITING),
				msecs_to_jiffies(CMDQ_PREDUMP_TIMEOUT_MS));
		} else {
			/* wait infinite without pre-dump, for loop case */
			wait_event(cmdq_wait_queue[handle->thread],
				handle->state != TASK_STATE_BUSY &&
				handle->state != TASK_STATE_WAITING);
			CMDQ_LOG("loop task finish:0x%p pkt:0x%p\n",
				handle, handle->pkt);
			break;
		}

		/* tick mailbox see to make pending task run */
		mbox_client_txdone(cmdq_clients[handle->thread]->chan, 0);

		if (waitq)
			break;

		/* pre-dump */
		CMDQ_LOG(
			"===== SW timeout Pre-dump %d handle:0x%p pkt:0x%p thread:%d state:%d =====\n",
			count, handle, handle->pkt, handle->thread,
			handle->state);
		cmdq_core_dump_status("INFO");
		cmdq_core_dump_pc(handle, handle->thread, "INFO");
		cmdq_core_dump_thread(handle, handle->thread, true, "INFO");

		if (handle->secData.is_secure)
			cmdq_core_dump_thread(NULL, CMDQ_SEC_IRQ_THREAD, false,
				"INFO");

		if (count == 0) {
			cmdq_core_dump_trigger_loop_thread("INFO");
			/* first time we dump full handle detail */
			cmdq_core_dump_handle(handle, "INFO");
		}

		count++;
	} while (1);

	handle->wakedUp = sched_clock();
	CMDQ_SYSTRACE_END();

	status = ctrl->handle_wait_result(handle, handle->thread);
	CMDQ_PROF_MMP(cmdq_mmp_get_event()->wait_task_done,
		MMPROFILE_FLAG_PULSE, ((unsigned long)handle),
		handle->wakedUp - handle->beginWait);

#if IS_ENABLED(CONFIG_MACH_MT6779) || IS_ENABLED(CONFIG_MACH_MT6765)
	/* debug wpe hw not waiting issue */
	if (handle->profile_exec) {
		u64 task_cost = handle->wakedUp - handle->trigger;
		u32 *exec;
		u64 diff;

		do_div(task_cost, 1000);
		exec = cmdq_pkt_get_perf_ret(handle->pkt);
		diff = exec[1] > exec[0] ?
			(exec[1] - exec[0]) : (~exec[1] + exec[0]);
		do_div(diff, 26);

		if (exec[0] == 0xdeaddead && exec[1] == 0xdeaddead)
			CMDQ_ERR(
				"profile task may not execute handle:%p pkt:%p engine:%#llx cpu cost:%llu\n",
					handle, handle->pkt,
					handle->engineFlag, task_cost);

		if (CMDQ_ENG_ISP_PERF_FLAG(handle->engineFlag)) {
			if (task_cost <= 3000)
				CMDQ_LOG(
					"[warn]isp task cost:%llu < 3ms hw begin:%u end:%u diff:%llu\n",
					task_cost, exec[0], exec[1], diff);
		}
	}
#endif


	CMDQ_SYSTRACE_BEGIN("%s_wait_release\n", __func__);
	cmdq_core_track_handle_record(handle, handle->thread);
	cmdq_pkt_release_handle(handle);

	CMDQ_SYSTRACE_END();
	CMDQ_PROF_MMP(cmdq_mmp_get_event()->wait_task_clean,
		MMPROFILE_FLAG_PULSE, ((unsigned long)handle->pkt),
		(unsigned long)handle->pkt);

	return status;
}

static void cmdq_pkt_auto_release_work(struct work_struct *work)
{
	struct cmdqRecStruct *handle;
	CmdqAsyncFlushCB cb;
	u64 user_data;

	handle = container_of(work, struct cmdqRecStruct, auto_release_work);
	if (unlikely(!handle)) {
		CMDQ_ERR("leak task! cannot get task from work item:0x%p\n",
			work);
		return;
	}

	cb = handle->async_callback;
	user_data = handle->async_user_data;

	CMDQ_PROF_MMP(cmdq_mmp_get_event()->autoRelease_done,
		MMPROFILE_FLAG_PULSE, ((unsigned long)handle), current->pid);
	cmdq_pkt_wait_flush_ex_result(handle);

	if (cb)
		cb(user_data);
}

s32 cmdq_pkt_auto_release_task(struct cmdqRecStruct *handle)
{
	if (handle->thread == CMDQ_INVALID_THREAD) {
		CMDQ_ERR(
			"handle:0x%p pkt:0x%p invalid thread:%d scenario:%d\n",
			handle, handle->pkt, handle->thread, handle->scenario);
		return -EINVAL;
	}

	CMDQ_PROF_MMP(cmdq_mmp_get_event()->autoRelease_add,
		MMPROFILE_FLAG_PULSE, ((unsigned long)handle), handle->thread);

	/* the work item is embeded in pTask already
	 * but we need to initialized it
	 */
	INIT_WORK(&handle->auto_release_work, cmdq_pkt_auto_release_work);
	queue_work(cmdq_ctx.taskThreadAutoReleaseWQ[handle->thread],
		&handle->auto_release_work);
	return 0;
}

static s32 cmdq_pkt_flush_async_ex_impl(struct cmdqRecStruct *handle,
	CmdqAsyncFlushCB cb, u64 user_data)
{
	s32 err;
	struct cmdq_client *client = NULL;
	struct cmdqRecStruct **pmqos_handle_list = NULL;
	struct ContextStruct *ctx;
	u32 handle_count;

	if (!handle->finalized) {
		CMDQ_ERR("handle not finalized:0x%p scenario:%d\n",
			handle, handle->scenario);
		return -EINVAL;
	}

	err = cmdq_pkt_lock_handle(handle, &client);
	if (err < 0)
		return err;

	/* assign user async flush callback */
	handle->async_callback = cb;
	handle->async_user_data = user_data;
	handle->pkt->err_cb.cb = cmdq_pkt_err_dump_handler;
	handle->pkt->err_cb.data = (void *)handle;

	CMDQ_MSG("%s handle:0x%p pkt:0x%p size:%zu thread:%d\n",
		__func__, handle, handle->pkt, handle->pkt->cmd_buf_size,
		handle->thread);

	handle->trigger = sched_clock();

	/* copy instruction to sram */
	if (handle->use_sram_buffer) {
		u32 cpr_offset = CMDQ_INVALID_CPR_OFFSET;
		struct cmdq_pkt_buffer *buf;

		if (handle->pkt->cmd_buf_size > CMDQ_CMD_BUFFER_SIZE) {
			CMDQ_ERR(
				"sram task only support 1 page, current size:%zu\n",
				handle->pkt->cmd_buf_size);
			return -EFAULT;
		}

		buf = list_first_entry_or_null(&handle->pkt->buf, typeof(*buf),
			list_entry);

		/* handle SRAM task, copy command to SRAM at first */
		do {
			if (!buf || !buf->pa_base) {
				CMDQ_ERR(
					"SRAM task DMA buffer list is empty!\n");
				err = -EFAULT;
				break;
			}

			err = cmdq_core_alloc_sram_buffer(
				handle->pkt->cmd_buf_size,
				handle->sram_owner_name, &cpr_offset);
			CMDQ_CHECK_AND_BREAK_STATUS(err);

			/* Note: only support 1 page size,
			 * so only copy first buffer
			 */
			err = cmdq_task_copy_to_sram(buf->pa_base, cpr_offset,
				handle->pkt->cmd_buf_size);
		} while (0);

		if (unlikely(err < 0)) {
			CMDQ_ERR("Prepare SRAM buffer task failed!\n");
			if (cpr_offset != CMDQ_INVALID_CPR_OFFSET)
				cmdq_core_free_sram_buffer(cpr_offset,
				handle->pkt->cmd_buf_size);
		} else {
			handle->sram_base = CMDQ_SRAM_ADDR(cpr_offset);
		}
	}

	/* TODO: remove pmqos in seure path */
	if (!handle->secData.is_secure) {
		/* PMQoS */
		CMDQ_SYSTRACE_BEGIN("%s_pmqos\n", __func__);
		mutex_lock(&cmdq_thread_mutex);
		ctx = cmdq_core_get_context();
		handle_count = ctx->thread[handle->thread].handle_count;

		pmqos_handle_list = kcalloc(handle_count + 1,
			sizeof(*pmqos_handle_list), GFP_KERNEL);

		if (pmqos_handle_list) {
			if (handle_count)
				cmdq_core_get_pmqos_handle_list(handle,
					pmqos_handle_list, handle_count);

			pmqos_handle_list[handle_count] = handle;
		}

		cmdq_core_group_begin_task(handle, pmqos_handle_list,
			handle_count + 1);

		kfree(pmqos_handle_list);
		ctx->thread[handle->thread].handle_count++;
		mutex_unlock(&cmdq_thread_mutex);
		CMDQ_SYSTRACE_END();
	}

	CMDQ_SYSTRACE_BEGIN("%s\n", __func__);
	cmdq_core_replace_v3_instr(handle, handle->thread);
	err = cmdq_pkt_flush_async(client, handle->pkt, cmdq_pkt_flush_handler,
		(void *)handle);
	CMDQ_SYSTRACE_END();

	if (err < 0) {
		CMDQ_ERR("pkt flush failed err:%d pkt:0x%p\n",
			err, handle->pkt);
		cmdq_pkt_release_handle(handle);
		return err;
	}

	return 0;
}

s32 cmdq_pkt_flush_ex(struct cmdqRecStruct *handle)
{
	s32 err;

	err = cmdq_pkt_flush_async_ex_impl(handle, NULL, 0);
	if (err < 0) {
		CMDQ_ERR("handle:0x%p start flush failed err:%d\n",
			handle, err);
		return err;
	}

	err = cmdq_pkt_wait_flush_ex_result(handle);
	if (err < 0)
		CMDQ_ERR("handle:%p wait flush failed err:%d\n", handle, err);

	CMDQ_MSG("%s done handle:0x%p\n", __func__, handle);

	return err;
}

s32 cmdq_pkt_flush_async_ex(struct cmdqRecStruct *handle,
	CmdqAsyncFlushCB cb, u64 user_data, bool auto_release)
{
	s32 err;

	/* mark self as running to notify client */
	if (handle->pkt->loop)
		handle->running_task = (void *)handle;

	CMDQ_SYSTRACE_BEGIN("%s\n", __func__);
	err = cmdq_pkt_flush_async_ex_impl(handle, cb, user_data);
	CMDQ_SYSTRACE_END();

	if (err < 0) {
		if (handle->thread == CMDQ_INVALID_THREAD || err == -EBUSY)
			return err;
		/* client may already wait for flush done, trigger as error */
		handle->state = TASK_STATE_ERROR;
		wake_up(&cmdq_wait_queue[handle->thread]);
		return err;
	}

	if (auto_release) {
		err = cmdq_pkt_auto_release_task(handle);
		if (err < 0) {
			CMDQ_ERR("wait flush failed err:%d\n", err);
			return err;
		}
	}

	CMDQ_MSG("%s done handle:0x%p\n", __func__, handle);

	return 0;
}

s32 cmdq_pkt_stop(struct cmdqRecStruct *handle)
{
	struct cmdq_client *client = NULL;

	CMDQ_MSG("%s handle:0x%p state:%d thread:%d\n",
		__func__, handle, handle->state, handle->thread);

	if (handle->thread == CMDQ_INVALID_THREAD) {
		CMDQ_ERR("invalid thread to stop pkt handle:0x%p pkt:0x%p\n",
			handle, handle->pkt);
		return -EINVAL;
	}

	client = cmdq_clients[handle->thread];

	/* TODO: need way to prevent user destroy handle in callback,
	 * but before we handle all things, since after callback we still
	 * hold handle and do lots of work
	 */

#if 0
	if (handle->pkt->loop && handle->state != TASK_STATE_IDLE &&
		atomic_read(&handle->exec) > 0) {
		CMDQ_MSG("%s release running handle:0x%p pkt:0x%p\n",
			__func__, handle, handle->pkt);

		err = cmdq_pkt_auto_release_task(handle);
		if (err < 0) {
			CMDQ_ERR("add loop to auto release err:%d\n", err);
			cmdq_pkt_release_handle(handle);
			return err;
		}

		/* make thread stop */
		cmdq_mbox_stop(client);

		/* wake again in case thread stops before */
		wake_up(&cmdq_wait_queue[handle->thread]);
	} else {
		/* stop directly and other thread shoudl clean up */
		cmdq_mbox_stop(client);
	}
#else
	cmdq_mbox_stop(client);

	if (handle->pkt->loop && handle->state != TASK_STATE_IDLE &&
		atomic_read(&handle->exec) > 0) {
		cmdq_pkt_release_handle(handle);
	} else {
		CMDQ_ERR(
			"still available after stop handle:0x%p loop:%s 0x%p state:%u exec:%d\n",
			handle, handle->pkt->loop ? "true" : "false",
			handle->running_task, handle->state,
			atomic_read(&handle->exec));
	}

	handle->running_task = NULL;
#endif

	CMDQ_MSG("%s done handle:0x%p\n", __func__, handle);
	return 0;
}

/* mailbox helper functions */

s32 cmdq_helper_mbox_register(struct device *dev)
{
	u32 i;
	s32 chan_id;
	struct cmdq_client *clt;

#ifdef CMDQ_SECURE_PATH_SUPPORT
	u32 sec_thread[2] = {0};
	s32 ret;

	ret = of_property_read_u32_array(dev->of_node, "secure_thread",
		sec_thread, 2);
	if (ret != 0) {
		sec_thread[0] = CMDQ_MIN_SECURE_THREAD_ID;
		sec_thread[1] = CMDQ_MIN_SECURE_THREAD_ID +
			CMDQ_MAX_SECURE_THREAD_COUNT;
	}
#endif

	/* for display we start from thread 0 */
	for (i = 0; i < CMDQ_MAX_THREAD_COUNT; i++) {
		clt = cmdq_mbox_create(dev, i);
		if (!clt || IS_ERR(clt)) {
			CMDQ_ERR("register mbox err:0x%p\n", clt);
			break;
		}

#ifdef CMDQ_SECURE_PATH_SUPPORT
		/* if channel is not valid in normal controller, check sec */
		if (i >= sec_thread[0] && i <= sec_thread[1])
			chan_id = cmdq_mbox_sec_chan_id(clt->chan);
		else
			chan_id = cmdq_mbox_chan_id(clt->chan);
#else
		chan_id = cmdq_mbox_chan_id(clt->chan);
#endif

		if (chan_id < 0 || cmdq_clients[chan_id]) {
			CMDQ_ERR("channel and client duplicate:%d\n", chan_id);
			cmdq_mbox_destroy(clt);
			continue;
		}

		cmdq_clients[chan_id] = clt;
		CMDQ_LOG("chan %d 0x%p dev:0x%p\n",
			chan_id, cmdq_clients[chan_id]->chan, dev);
	}

	cmdq_client_base = cmdq_register_device(dev);

	/* for mm like mdp set large pool count */
	for (i = CMDQ_DYNAMIC_THREAD_ID_START;
		i < CMDQ_DYNAMIC_THREAD_ID_START + 3; i++) {
		if (!cmdq_clients[i])
			continue;
		cmdq_mbox_pool_set_limit(cmdq_clients[i], CMDQ_DMA_POOL_COUNT);
	}

	return 0;
}

struct cmdq_client *cmdq_helper_mbox_client(u32 idx)
{
	if (idx >= ARRAY_SIZE(cmdq_clients))
		return NULL;
	return cmdq_clients[idx];
}

struct cmdq_base *cmdq_helper_mbox_base(void)
{
	return cmdq_client_base;
}

void cmdq_helper_mbox_clear_pools(void)
{
	u32 i;

	for (i = 0; i < CMDQ_MAX_THREAD_COUNT; i++) {
		if (!cmdq_clients[i])
			continue;
		cmdq_mbox_pool_clear(cmdq_clients[i]);
	}
}

void cmdq_core_initialize(void)
{
	const u32 max_thread_count = cmdq_dev_get_thread_count();
	s32 status;
	u32 index;
	u32 thread_id;
	char long_msg[CMDQ_LONGSTRING_MAX];
	u32 msg_offset;
	s32 msg_max_size;

	cmdq_helper_mbox_register(cmdq_dev_get());

	atomic_set(&cmdq_thread_usage, 0);

	cmdq_wait_queue = kcalloc(max_thread_count, sizeof(*cmdq_wait_queue),
		GFP_KERNEL);
	for (index = 0; index < max_thread_count; index++)
		init_waitqueue_head(&cmdq_wait_queue[index]);

	/* Reset overall context */
	memset(&cmdq_ctx, 0x0, sizeof(cmdq_ctx));
	cmdq_ctx.aee = true;

	/* mark available threads */
	for (index = 0; index < CMDQ_MAX_SCENARIO_COUNT; index++) {
		/* mark static thread dispatch thread as used */
		thread_id = cmdq_core_get_thread_id(index);
		if (thread_id != CMDQ_INVALID_THREAD)
			cmdq_ctx.thread[thread_id].used = true;
	}

	for (index = 0; index < ARRAY_SIZE(cmdq_clients); index++)
		if (!cmdq_clients[index])
			cmdq_ctx.thread[index].used = true;


#if defined(CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT) || \
	defined(CONFIG_MTK_CAM_SECURITY_SUPPORT)
	/* for secure path reserve irq notify thread */
	cmdq_ctx.thread[CMDQ_SEC_IRQ_THREAD].used = true;
#endif

	cmdq_long_string_init(false, long_msg, &msg_offset, &msg_max_size);
	for (index = 0; index < max_thread_count; index++) {
		if (!cmdq_ctx.thread[index].used)
			cmdq_long_string(long_msg, &msg_offset, &msg_max_size,
				"%d, ", index);
	}
	CMDQ_LOG("available thread pool:%s max:%u\n",
		long_msg, max_thread_count);

	/* Initialize task lists */
	INIT_LIST_HEAD(&cmdq_ctx.handle_active);
	INIT_LIST_HEAD(&cmdq_ctx.sram_allocated_list);

	/* Initialize writable address */
	INIT_LIST_HEAD(&cmdq_ctx.writeAddrList);

	/* Initialize work queue */
	cmdq_core_init_thread_work_queue();

	/* Initialize command buffer dump */
	memset(&cmdq_command_dump, 0x0, sizeof(cmdq_command_dump));

	/* Reset overall first error dump */
	cmdq_core_reset_first_dump();

	/* pre-allocate fake SPR SRAM area */
	{
		u32 fake_spr_sram = 0;

		status = cmdq_core_alloc_sram_buffer(max_thread_count *
			CMDQ_THR_CPR_MAX * sizeof(u32),
			"Fake SPR", &fake_spr_sram);
		if (status < 0) {
			CMDQ_ERR("Allocate Fake SPR failed !!");
		} else {
			CMDQ_LOG(
				"CPR for thread allocated, thread:%u free:%zu\n",
				max_thread_count,
				cmdq_core_get_free_sram_size());
		}
	}

	/* pre-allocate delay CPR SRAM area */
	{
		status = cmdq_core_alloc_sram_buffer(CMDQ_DELAY_MAX_SET *
			CMDQ_DELAY_SET_MAX_CPR * sizeof(u32),
			"Delay CPR", &cmdq_ctx.delay_cpr_start);
		if (status < 0)
			CMDQ_ERR("Allocate delay CPR failed status:%d\n",
				status);
	}
	/* allocate shared memory */
	cmdq_ctx.hSecSharedMem = NULL;
#ifdef CMDQ_SECURE_PATH_SUPPORT
	cmdq_sec_create_shared_memory(&(cmdq_ctx.hSecSharedMem),
		PAGE_SIZE);
#endif

#if 0
	cmdqCoreRegisterDebugRegDumpCB(testcase_regdump_begin,
		testcase_regdump_end);
#endif

	/* Initialize MET for statistics */
	/* note that we don't need to uninit it. */
	CMDQ_PROF_INIT();
#ifdef CMDQ_PROFILE_MMP
	cmdq_mmp_init();
#endif
#ifdef CMDQ_SECURE_PATH_SUPPORT
	/* Initialize secure path context */
	cmdqSecInitialize();
#endif
#if 0
	/* Initialize test case structure */
	cmdq_test_init_setting();
#endif

#ifdef CMDQ_TIMER_ENABLE
	/* set to true to disable delay thread when boot */
	cmdq_delay_thd_inited = false;
#endif
}

static int __init cmdq_core_late_init(void)
{
	CMDQ_LOG("CMDQ driver late init begin\n");

#ifdef CMDQ_TIMER_ENABLE
	if (!cmdq_delay_thd_inited) {
		s32 status = cmdq_delay_thread_init();

		if (status < 0)
			CMDQ_ERR("delay init failed in late init!\n");
	}
#endif

	CMDQ_MSG("CMDQ driver late init end\n");

	return 0;
}

void cmdq_core_deinitialize(void)
{
	cmdq_core_destroy_thread_work_queue();

	/* TODO: destroy all task in mailbox controller */
#if 0
	/* release all tasks in both list */
	for (index = 0; index < ARRAY_SIZE(lists); ++index) {
		list_for_each(p, lists[index]) {
			CMDQ_PROF_MUTEX_LOCK(gCmdqTaskMutex, deinitialize);

			pTask = list_entry(p, struct TaskStruct, listEntry);

			/* free allocated DMA buffer */
			cmdq_task_free_task_command_buffer(pTask);
			kmem_cache_free(cmdq_ctx.taskCache, pTask);
			list_del(p);

			CMDQ_PROF_MUTEX_UNLOCK(gCmdqTaskMutex, deinitialize);
		}
	}

	/* check if there are dangling write addresses. */
	if (!list_empty(&cmdq_ctx.writeAddrList)) {
		/* there are unreleased write buffer, raise AEE */
		CMDQ_AEE("CMDQ", "there are unreleased write buffer");
	}

	kmem_cache_destroy(cmdq_ctx.taskCache);
	cmdq_ctx.taskCache = NULL;
#endif

#ifdef CMDQ_TIMER_ENABLE
	/* Delay thread de-initialization */
	cmdq_delay_thread_deinit();
#endif

#ifdef CMDQ_SECURE_PATH_SUPPORT
	/* Deinitialize secure path context */
	cmdqSecDeInitialize();
#endif

	kfree(cmdq_dts.prefetch_size);
	cmdq_dts.prefetch_size = NULL;
	kfree(cmdq_wait_queue);
	cmdq_wait_queue = NULL;
	kfree(cmdq_ctx.inst_check_buffer);
	cmdq_ctx.inst_check_buffer = NULL;
	cmdq_helper_mbox_clear_pools();
}

late_initcall(cmdq_core_late_init);

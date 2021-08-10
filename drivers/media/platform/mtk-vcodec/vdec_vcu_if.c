/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: PC Chen <pc.chen@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/fdtable.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include "mtk_vcodec_dec_pm.h"
#include "mtk_vcodec_drv.h"
#include "mtk_vcodec_intr.h"
#include "mtk_vcodec_util.h"
#include "vdec_ipi_msg.h"
#include "vdec_vcu_if.h"

static void handle_init_ack_msg(struct vdec_vcu_ipi_init_ack *msg)
{
	struct vdec_vcu_inst *vcu = (struct vdec_vcu_inst *)
		(unsigned long)msg->ap_inst_addr;

	if (vcu == NULL)
		return;
	mtk_vcodec_debug(vcu, "+ ap_inst_addr = 0x%lx",
		(uintptr_t)msg->ap_inst_addr);

	/* mapping VCU address to kernel virtual address */
	/* the content in vsi is initialized to 0 in VCU */
	vcu->vsi = vcu_mapping_dm_addr(vcu->dev, msg->vcu_inst_addr);
	vcu->inst_addr = msg->vcu_inst_addr;
	mtk_vcodec_debug(vcu, "- vcu_inst_addr = 0x%x", vcu->inst_addr);
}

static void handle_query_cap_ack_msg(struct vdec_vcu_ipi_query_cap_ack *msg)
{
	struct vdec_vcu_inst *vcu = (struct vdec_vcu_inst *)msg->ap_inst_addr;
	void *data;
	int size = 0;

	if (vcu == NULL)
		return;
	mtk_vcodec_debug(vcu, "+ ap_inst_addr = 0x%lx, vcu_data_addr = 0x%x, id = %d",
		(uintptr_t)msg->ap_inst_addr, msg->vcu_data_addr, msg->id);
	/* mapping VCU address to kernel virtual address */
	data = vcu_mapping_dm_addr(vcu->dev, msg->vcu_data_addr);
	switch (msg->id) {
	case GET_PARAM_CAPABILITY_SUPPORTED_FORMATS:
		size = sizeof(struct mtk_video_fmt);
		memcpy((void *)msg->ap_data_addr, data,
			 size * MTK_MAX_DEC_CODECS_SUPPORT);
		break;
	case GET_PARAM_CAPABILITY_FRAME_SIZES:
		size = sizeof(struct mtk_codec_framesizes);
		memcpy((void *)msg->ap_data_addr, data,
			size * MTK_MAX_DEC_CODECS_SUPPORT);
		break;
	default:
		break;
	}
	mtk_vcodec_debug(vcu, "- vcu_inst_addr = 0x%x", vcu->inst_addr);
}

inline int get_mapped_fd(struct dma_buf *dmabuf)
{
	int target_fd = 0;
#ifndef CONFIG_MTK_IOMMU_V2
	unsigned long rlim_cur;
	unsigned long irqs;
	struct task_struct *task = NULL;
	struct files_struct *f = NULL;
	unsigned long flags = 0;

	if (dmabuf == NULL || dmabuf->file == NULL)
		return 0;

	vcu_get_file_lock();

	vcu_get_task(&task, &f, 0);
	if (task == NULL || f == NULL) {
		vcu_put_file_lock();
		return -EMFILE;
	}

	if (vcu_get_sig_lock(&flags) <= 0) {
		pr_info("%s() Failed to try lock...VPUD may die", __func__);
		vcu_put_file_lock();
		return -EMFILE;
	}

	if (vcu_check_vpud_alive() == 0) {
		pr_info("%s() Failed to check vpud alive. VPUD died", __func__);
		vcu_put_file_lock();
		vcu_put_sig_lock(flags);
		return -EMFILE;
	}
	vcu_put_sig_lock(flags);

	if (!lock_task_sighand(task, &irqs)) {
		vcu_put_file_lock();
		return -EMFILE;
	}

	// get max number of open files
	rlim_cur = task_rlimit(task, RLIMIT_NOFILE);
	unlock_task_sighand(task, &irqs);

	f = get_files_struct(task);
	if (!f) {
		vcu_put_file_lock();
		return -EMFILE;
	}

	target_fd = __alloc_fd(f, 0, rlim_cur, O_CLOEXEC);

	get_file(dmabuf->file);

	if (target_fd < 0) {
		put_files_struct(f);
		vcu_put_file_lock();
		return -EMFILE;
	}

	__fd_install(f, target_fd, dmabuf->file);

	put_files_struct(f);
	vcu_put_file_lock();

	/* pr_info("get_mapped_fd: %d", target_fd); */
#endif
	return target_fd;
}

inline void close_mapped_fd(unsigned int target_fd)
{
#ifndef CONFIG_MTK_IOMMU_V2
	struct task_struct *task = NULL;
	struct files_struct *f = NULL;
	unsigned long flags = 0;

	vcu_get_file_lock();
	vcu_get_task(&task, &f, 0);
	if (task == NULL || f == NULL) {
		vcu_put_file_lock();
		return;
	}

	if (vcu_get_sig_lock(&flags) <= 0) {
		pr_info("%s() Failed to try lock...VPUD may die", __func__);
		vcu_put_file_lock();
		return;
	}

	if (vcu_check_vpud_alive() == 0) {
		pr_info("%s() Failed to check vpud alive. VPUD died", __func__);
		vcu_put_file_lock();
		vcu_put_sig_lock(flags);
		return;
	}
	vcu_put_sig_lock(flags);

	f = get_files_struct(task);
	if (!f) {
		vcu_put_file_lock();
		return;
	}

	__close_fd(f, target_fd);

	put_files_struct(f);
	vcu_put_file_lock();
#endif
}

/*
 * This function runs in interrupt context and it means there's a IPI MSG
 * from VCU.
 */
int vcu_dec_ipi_handler(void *data, unsigned int len, void *priv)
{
	struct vdec_vcu_ipi_ack *msg = data;
	struct vdec_vcu_inst *vcu = NULL;
	int ret = 0;
	struct timeval t_s, t_e;
	struct task_struct *task = NULL;
	struct files_struct *f = NULL;

	vcu_get_file_lock();
	vcu_get_task(&task, &f, 0);
	vcu_put_file_lock();
	if (msg == NULL || task == NULL ||
	   task->tgid != current->tgid ||
	   (struct vdec_vcu_inst *)msg->ap_inst_addr == NULL) {
		ret = -EINVAL;
		return ret;
	}

	vcu = (struct vdec_vcu_inst *)(unsigned long)msg->ap_inst_addr;

	if ((vcu != priv) && msg->msg_id < VCU_IPIMSG_DEC_WAITISR) {
		pr_info("%s, vcu:%p != priv:%p\n", __func__, vcu, priv);
		return 1;
	}

	mtk_vcodec_debug(vcu, "+ id=%X status = %d\n",
		msg->msg_id, msg->status);

	if (vcu != NULL && vcu->abort)
		return -EINVAL;

	if (msg->status == 0) {
		switch (msg->msg_id) {
		case VCU_IPIMSG_DEC_INIT_ACK:
			handle_init_ack_msg(data);
			break;
		case VCU_IPIMSG_DEC_QUERY_CAP_ACK:
			handle_query_cap_ack_msg(data);
			break;
		case VCU_IPIMSG_DEC_START_ACK:
		case VCU_IPIMSG_DEC_END_ACK:
		case VCU_IPIMSG_DEC_DEINIT_ACK:
		case VCU_IPIMSG_DEC_RESET_ACK:
		case VCU_IPIMSG_DEC_SET_PARAM_ACK:
			break;
		case VCU_IPIMSG_DEC_WAITISR:
			/* wait decoder done interrupt */
			do_gettimeofday(&t_s);
			mtk_vcodec_wait_for_done_ctx(vcu->ctx,
				MTK_INST_IRQ_RECEIVED,
				WAIT_INTR_TIMEOUT_MS);
			do_gettimeofday(&t_e);
			mtk_vcodec_perf_log("irq:%ld",
				(t_e.tv_sec - t_s.tv_sec) * 1000000 +
				(t_e.tv_usec - t_s.tv_usec));
			ret = 1;
			break;
		case VCU_IPIMSG_DEC_GET_FRAME_BUFFER:
			/* TODO: return til it can get available frame buffer */
			ret = 1;
			break;
		default:
			mtk_vcodec_err(vcu, "invalid msg=%X", msg->msg_id);
			ret = 1;
			break;
		}
	}

	mtk_vcodec_debug(vcu, "- id=%X", msg->msg_id);

	/* deinit ack timeout case handling do not touch vdec_vcu_inst
	 * or memory used after freed
	 */
	if (msg->msg_id != VCU_IPIMSG_DEC_DEINIT_ACK) {
		vcu->signaled = 1;
		vcu->failure = msg->status;
	}

	return ret;
}

static int vcodec_vcu_send_msg(struct vdec_vcu_inst *vcu, void *msg, int len)
{
	int err;
	struct task_struct *task = NULL;
	struct files_struct *f = NULL;

	mtk_vcodec_debug(vcu, "id=%X", *(uint32_t *)msg);
	if (vcu->abort)
		return -EIO;

	vcu_get_file_lock();
	vcu_get_task(&task, &f, 0);
	vcu_put_file_lock();
	if (task == NULL ||
		vcu->daemon_pid != task->tgid) {
		if (task)
			mtk_vcodec_err(vcu, "send fail pid: inst %d curr %d",
				vcu->daemon_pid, task->tgid);
		vcu->abort = 1;
		return -EIO;
	}

	vcu->failure = 0;
	vcu->signaled = 0;

	err = vcu_ipi_send(vcu->dev, vcu->id, msg, len, vcu);
	if (err) {
		mtk_vcodec_err(vcu, "send fail vcu_id=%d msg_id=%X status=%d",
					   vcu->id, *(uint32_t *)msg, err);
		if (err == -EIO)
			vcu->abort = 1;
		return err;
	}
	mtk_vcodec_debug(vcu, "- ret=%d", err);
	return vcu->failure;
}

static int vcodec_send_ap_ipi(struct vdec_vcu_inst *vcu, unsigned int msg_id)
{
	struct vdec_ap_ipi_cmd msg;
	int err = 0;

	mtk_vcodec_debug(vcu, "+ id=%X", msg_id);

	memset(&msg, 0, sizeof(msg));
	msg.msg_id = msg_id;
	msg.vcu_inst_addr = vcu->inst_addr;

	err = vcodec_vcu_send_msg(vcu, &msg, sizeof(msg));
	mtk_vcodec_debug(vcu, "- id=%X ret=%d", msg_id, err);
	return err;
}

void vcu_dec_set_pid(struct vdec_vcu_inst *vcu)
{
	struct task_struct *task = NULL;
	struct files_struct *f = NULL;

	vcu_get_file_lock();
	vcu_get_task(&task, &f, 0);
	vcu_put_file_lock();
	if (task != NULL)
		vcu->daemon_pid = task->tgid;
	else
		vcu->daemon_pid = -1;
}

int vcu_dec_init(struct vdec_vcu_inst *vcu)
{
	struct vdec_ap_ipi_init msg;
	int err;

	mtk_vcodec_debug_enter(vcu);

	init_waitqueue_head(&vcu->wq);
	vcu->signaled = 0;
	vcu->failure = 0;

	err = vcu_ipi_register(vcu->dev, vcu->id, vcu->handler, NULL, vcu);
	if (err != 0) {
		mtk_vcodec_err(vcu, "vcu_ipi_register fail status=%d", err);
		return err;
	}

	memset(&msg, 0, sizeof(msg));
	msg.msg_id = AP_IPIMSG_DEC_INIT;
	msg.ap_inst_addr = (unsigned long)vcu;

	if (vcu->ctx->dec_params.svp_mode == 1)
		msg.reserved = vcu->ctx->dec_params.svp_mode;

	mtk_vcodec_debug(vcu, "vdec_inst=%p svp_mode=%d", vcu, msg.reserved);

	vcu_dec_set_pid(vcu);
	err = vcodec_vcu_send_msg(vcu, (void *)&msg, sizeof(msg));

	mtk_vcodec_debug(vcu, "- ret=%d", err);
	return err;
}

int vcu_dec_start(struct vdec_vcu_inst *vcu, unsigned int *data,
				  unsigned int len)
{
	struct vdec_ap_ipi_dec_start msg;
	int i;
	int err = 0;

	mtk_vcodec_debug_enter(vcu);

	memset(&msg, 0, sizeof(msg));
	msg.msg_id = AP_IPIMSG_DEC_START;
	msg.vcu_inst_addr = vcu->inst_addr;

	for (i = 0; i < len; i++)
		msg.data[i] = data[i];

	err = vcodec_vcu_send_msg(vcu, (void *)&msg, sizeof(msg));
	mtk_vcodec_debug(vcu, "- ret=%d", err);
	return err;
}

int vcu_dec_end(struct vdec_vcu_inst *vcu)
{
	return vcodec_send_ap_ipi(vcu, AP_IPIMSG_DEC_END);
}

int vcu_dec_deinit(struct vdec_vcu_inst *vcu)
{
	return vcodec_send_ap_ipi(vcu, AP_IPIMSG_DEC_DEINIT);
}

int vcu_dec_reset(struct vdec_vcu_inst *vcu)
{
	return vcodec_send_ap_ipi(vcu, AP_IPIMSG_DEC_RESET);
}

int vcu_dec_query_cap(struct vdec_vcu_inst *vcu, unsigned int id, void *out)
{
	struct vdec_ap_ipi_query_cap msg;
	int err = 0;

	mtk_vcodec_debug(vcu, "+ id=%X", AP_IPIMSG_DEC_QUERY_CAP);
	vcu->dev = vcu_get_plat_device(vcu->ctx->dev->plat_dev);
	vcu->id = (vcu->id == IPI_VCU_INIT) ? IPI_VDEC_COMMON : vcu->id;
	vcu->handler = vcu_dec_ipi_handler;

	err = vcu_ipi_register(vcu->dev, vcu->id, vcu->handler, NULL, vcu);
	if (err != 0) {
		mtk_vcodec_err(vcu, "vcu_ipi_register fail status=%d", err);
		return err;
	}

	memset(&msg, 0, sizeof(msg));
	msg.msg_id = AP_IPIMSG_DEC_QUERY_CAP;
	msg.id = id;
	msg.ap_inst_addr = (uintptr_t)vcu;
	msg.ap_data_addr = (uintptr_t)out;

	vcu_dec_set_pid(vcu);
	err = vcodec_vcu_send_msg(vcu, &msg, sizeof(msg));
	mtk_vcodec_debug(vcu, "- id=%X ret=%d", msg.msg_id, err);
	return err;
}

int vcu_dec_set_param(struct vdec_vcu_inst *vcu, unsigned int id, void *param,
					  unsigned int size)
{
	struct vdec_ap_ipi_set_param msg;
	uint64_t *param_ptr = (uint64_t *)param;
	int err = 0;
	int i = 0;

	mtk_vcodec_debug(vcu, "+ id=%X", AP_IPIMSG_DEC_SET_PARAM);

	memset(&msg, 0, sizeof(msg));
	msg.msg_id = AP_IPIMSG_DEC_SET_PARAM;
	msg.id = id;
	msg.vcu_inst_addr = vcu->inst_addr;
	for (i = 0; i < size; i++) {
		msg.data[i] = (__u32)(*(param_ptr + i));
		mtk_vcodec_debug(vcu, "msg.id = 0x%X, msg.data[%d]=%d",
			msg.id, i, msg.data[i]);
	}

	err = vcodec_vcu_send_msg(vcu, &msg, sizeof(msg));
	mtk_vcodec_debug(vcu, "- id=%X ret=%d", AP_IPIMSG_DEC_SET_PARAM, err);

	return err;
}

int vcu_dec_set_ctx_for_gce(struct vdec_vcu_inst *vcu)
{
	int err = 0;

	vcu_set_codec_ctx(vcu->dev,
		(void *)vcu->ctx, VCU_VDEC);

	return err;
}


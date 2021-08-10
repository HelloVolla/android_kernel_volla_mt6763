/*
 * Copyright (C) 2016 MediaTek Inc.
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

#ifndef __PORT_IPC_I_H__
#define __PORT_IPC_I_H__

#include <linux/wait.h>
#include "ccci_core.h"
#include "ccci_config.h"
#include "port_t.h"

#define MAX_NUM_IPC_TASKS 10
#define CCCI_TASK_PENDING 0x01
#define IPC_MSGSVC_RVC_DONE 0x12344321

struct ccci_ipc_ctrl {
	unsigned char task_id;
	unsigned char md_is_ready;
	unsigned long flag;
	wait_queue_head_t tx_wq;
	wait_queue_head_t md_rdy_wq;
	struct port_t *port;
};

/* IPC MD/AP id map table */
struct ipc_task_id_map {
	u32 extq_id;		/* IPC universal mapping external queue */
	u32 task_id;		/* IPC processor internal task id */
};

struct ccci_ipc_ilm {
	u32 src_mod_id;
	u32 dest_mod_id;
	u32 sap_id;
	u32 msg_id;
	u32 local_para_ptr;
	u32 peer_buff_ptr;
} __packed;	/* for MD */

struct garbage_filter_header {
	u32 filter_set_id;
	u32 filter_cnt;
	u32 uplink;
} __packed;

struct garbage_filter_item {
	u32 filter_id;
	u8 ip_type;
	u8 protocol;
	u16 dst_port;
	u32 magic_code;
} __packed;

typedef enum {
	GF_IPV4V6 = 0,
	GF_IPV4,
	GF_IPV6
} GF_IP_TYPE;

typedef enum {
	GF_TCP = 6,
	GF_UDP = 17
} GF_PROTOCOL_TYPE;

#endif				/* __PORT_IPC_I_H__ */

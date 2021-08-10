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

#ifndef __MMDVFS_PLAT_H__
#define __MMDVFS_PLAT_H__

#include "mmdvfs_pmqos.h"

#define MAX_PORT_COUNT 32
struct mm_larb_request {
	struct list_head larb_list;	/* To search master setting of larb */
	bool larb_list_init;
	u32 port_count;
	u32 ratio[MAX_PORT_COUNT];
	s32 total_bw_data;
	s32 total_hrt_data;
	s32 total_mix_limit;
	s32 total_occupied;
	u32 comm_port;
	u32 channel;
};
const char *mmdvfs_get_plat_name(void);

void mmdvfs_update_qos_sram(struct mm_larb_request larb_req[], u32 larb_update);

void mmdvfs_update_limit_config(enum mmdvfs_limit_source source,
	u32 source_value, u32 *limit_value, u32 *limit_level);

void mmdvfs_update_plat_ostd(u32 larb, u32 hrt_port, u32 *ostd);

bool is_disp_larb(u32 larb);

bool mmdvfs_log_larb_mmp(s32 common_port_id, s32 larb_id);

#define DEFAULT_BW_UPDATE(bw_value) ((bw_value) * 1 / 2)
#define DEFAULT_LIMIT_UPDATE(bw_value) ((bw_value) * 7 / 10)
static inline u32 get_comp_value(u32 bw_value, u32 comp_type, bool is_bw)
{
	if (comp_type == BW_COMP_DEFAULT) {
		if (is_bw)
			return DEFAULT_BW_UPDATE(bw_value);
		else
			return DEFAULT_LIMIT_UPDATE(bw_value);
	}

	return bw_value;
}

u32 mmdvfs_get_ccu_smi_common_port(void);

s32 get_ccu_hrt_bw(struct mm_larb_request larb_req[]);

#endif /* __MMDVFS_PLAT_H__ */

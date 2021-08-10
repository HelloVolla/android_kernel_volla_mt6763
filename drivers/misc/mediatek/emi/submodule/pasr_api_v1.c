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

#include <linux/platform_device.h>

#include <mt_emi.h>
#include "pasr_api_v1.h"

/* acquire DRAM Setting for PASR/DPD */
static unsigned int get_segment_nr(unsigned int rk_index)
{
	unsigned int rank_size;
	unsigned int num_of_1;
	unsigned int i;

	rank_size = get_rank_size(rk_index) / get_ch_num();
	num_of_1 = 0;

	for (i = 0; i < 32; i++) {
		if (rank_size & 0x1)
			num_of_1++;
		rank_size = rank_size >> 1;
	}

	if (num_of_1 > 1)
		return 6;
	else
		return 8;
}

void acquire_dram_setting(struct basic_dram_setting *pasrdpd)
{
	unsigned int ch, rk;

	pasrdpd->channel_nr = get_ch_num();

	for (ch = 0; ch < MAX_CH; ch++) {
		for (rk = 0; rk < MAX_RK; rk++) {
			if ((ch >= pasrdpd->channel_nr) ||
			    (rk >= get_rk_num())) {
				pasrdpd->channel[ch].rank[rk].valid_rank =
					false;
				pasrdpd->channel[ch].rank[rk].rank_size = 0;
				pasrdpd->channel[ch].rank[rk].segment_nr = 0;
				continue;
			}
			pasrdpd->channel[ch].rank[rk].valid_rank = true;
			pasrdpd->channel[ch].rank[rk].rank_size =
				get_rank_size(rk) / (pasrdpd->channel_nr);
			pasrdpd->channel[ch].rank[rk].segment_nr =
				get_segment_nr(rk);

		}
	}
}


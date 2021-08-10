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

#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/file.h>
#include <linux/string.h>
#include <mt-plat/mtk_chip.h>
#ifdef CONFIG_MTK_DCS
#include <mt-plat/mtk_meminfo.h>
#endif
#include "layering_rule.h"
#include "debug.h"
#include "disp_lowpower.h"
#include "ddp_ovl_wcg.h"


static struct disp_layer_info layering_info;
static int debug_resolution_level;
static struct layering_rule_info_t *l_rule_info;
static struct layering_rule_ops *l_rule_ops;
static int ext_id_tunning(struct disp_layer_info *disp_info, int disp_idx);
static unsigned int adaptive_dc_request;
static unsigned int roll_gpu_for_idle;
static int g_emi_bound_table[HRT_LEVEL_NUM];

static struct {
	enum LYE_HELPER_OPT opt;
	unsigned int val;
	const char *desc;
} help_info[] = {
	{LYE_OPT_DUAL_PIPE, 0, "LYE_OPT_DUAL_PIPE"},
	{LYE_OPT_EXT_LAYER, 0, "LYE_OPT_EXTENDED_LAYER"},
	{LYE_OPT_RPO, 0, "LYE_OPT_RPO"},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	{LYE_OPT_ROUND_CORNER, 0, "LYE_OPT_ROUND_CORNER"},
#endif
};

void set_layering_opt(enum LYE_HELPER_OPT opt, int value)
{
	if (opt >= LYE_OPT_NUM) {
		DISPMSG("%s invalid layering opt:%d\n", __func__, opt);
		return;
	}

	help_info[opt].val = !!value;
}

int get_layering_opt(enum LYE_HELPER_OPT opt)
{
	if (opt >= LYE_OPT_NUM) {
		DISPMSG("%s invalid layering opt:%d\n", __func__, opt);
		return -1;
	}

	return help_info[opt].val;
}

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
void set_round_corner_opt(enum LYE_HELPER_OPT opt, int value)
{
	if (opt >= LYE_OPT_NUM) {
		DISPMSG("%s invalid round corner opt:%d\n", __func__, opt);
		return;
	}

	help_info[opt].val = value;
}

int get_round_corner_opt(enum LYE_HELPER_OPT opt)
{
	if (opt >= LYE_OPT_NUM) {
		DISPMSG("%s invalid round corner opt:%d\n", __func__, opt);
		return -1;
	}

	return help_info[opt].val;
}

int get_round_corner_mode(int val)
{
	if ((val > 0) && (val & 0xFF) && ((val >> 16) & 0xFF))
		return (val >> 8) & 0xFF;
	else
		return -1;
}
#endif

bool is_ext_path(struct disp_layer_info *disp_info)
{
	if (disp_info->layer_num[HRT_SECONDARY] > 0)
		return true;
	else
		return false;
}

bool is_decouple_path(struct disp_layer_info *disp_info)
{
	if (disp_info->disp_mode[HRT_PRIMARY] != 1)
		return true;
	else
		return false;
}

static int get_bpp(enum DISP_FORMAT format)
{
	int Bpp = 4;
	int id = format >> 8;

	if (id <= (DISP_FORMAT_UNKNOWN >> 8) || id >= (DISP_FORMAT_NUM >> 8)) {
		DISP_PR_ERR("Invalid color format: 0x%x\n", format);
		goto err;
	}

	Bpp = format & 0xFF;

err:
	return Bpp;
}

bool is_argb_fmt(enum DISP_FORMAT format)
{
	switch (format) {
	case DISP_FORMAT_ARGB8888:
	case DISP_FORMAT_ABGR8888:
	case DISP_FORMAT_RGBA8888:
	case DISP_FORMAT_BGRA8888:
		return true;
	default:
		return false;
	}
}

bool is_yuv(enum DISP_FORMAT format)
{
	switch (format) {
	case DISP_FORMAT_YUV422:
	case DISP_FORMAT_UYVY:
	case DISP_FORMAT_YUV420_P:
	case DISP_FORMAT_YV12:
		return true;
	default:
		return false;
	}
}

bool is_layer_id_valid(struct disp_layer_info *disp_info,
	int disp_idx, int i)
{
	if (i < 0 || i >= disp_info->layer_num[disp_idx])
		return false;
	else
		return true;
}

bool is_gles_layer(struct disp_layer_info *disp_info,
		   int disp_idx, int layer_idx)
{
	if (layer_idx >= disp_info->gles_head[disp_idx] &&
		layer_idx <= disp_info->gles_tail[disp_idx])
		return true;
	else
		return false;
}

inline bool has_layer_cap(struct layer_config *layer_info,
			  enum LAYERING_CAPS l_caps)
{
	if (layer_info->layer_caps & l_caps)
		return true;
	return false;
}

static int is_overlap_on_yaxis(struct layer_config *lhs,
			       struct layer_config *rhs)
{
	/*
	 * HWC may adjust the offset of yuv layer due to alignment limitation
	 * after querying layering rule.
	 * So it have chance to make yuv layer overlap with other ext layer.
	 * We add the workaround here to avoid the yuv as the base layer of
	 * extended layer and will remove it once the HWC correct the problem.
	 */
	if (is_yuv(lhs->src_fmt))
		return 1;

	if ((lhs->dst_offset_y + lhs->dst_height <= rhs->dst_offset_y) ||
	    (rhs->dst_offset_y + rhs->dst_height <= lhs->dst_offset_y))
		return 0;
	return 1;
}

bool is_layer_across_each_pipe(struct layer_config *layer_info)
{
	int dst_x, dst_w;

	if (!get_layering_opt(LYE_OPT_DUAL_PIPE))
		return true;

	dst_x = layer_info->dst_offset_x;
	dst_w = layer_info->dst_width;
	if ((dst_x + dst_w <= primary_display_get_width() / 2) ||
	    (dst_x > primary_display_get_width() / 2))
		return false;
	return true;
}

static inline bool is_extended_layer(struct layer_config *layer_info)
{
	return (layer_info->ext_sel_layer != -1);
}

static bool is_extended_base_layer_valid(struct layer_config *configs,
					 int layer_idx)
{
	if ((layer_idx == 0 && configs->src_fmt == DISP_FORMAT_DIM) ||
	    has_layer_cap(configs, DISP_RSZ_LAYER))
		return false;

	/*
	 * Under dual pipe, if the layer is not included in each pipe,
	 * it cannot be used as a base layer for extended layer
	 * because extended layer would not find base layer in one of
	 * display pipe.
	 * So always mark this specific layer as overlap to avoid the fail case.
	 */
	if (!is_layer_across_each_pipe(configs))
		return false;

	return true;
}

static inline bool is_extended_over_limit(int ext_cnt)
{
	if (ext_cnt > 3)
		return true;
	return false;
}

/**
 * check if continuous ext layers are overlapped with each other
 * also need to check the below nearest phy layer
 * which these ext layers will be attached to
 * 1. check all ext layers, if overlapped with any one, change it to phy layer
 * 2. if more than 1 ext layer exist, need to check the phy layer
 */
static int is_continuous_ext_layer_overlap(struct layer_config *configs,
					   int curr)
{
	int overlapped;
	struct layer_config *src_info, *dst_info;
	int i;

	overlapped = 0;
	dst_info = &configs[curr];
	for (i = curr-1; i >= 0; i--) {
		src_info = &configs[i];
		if (is_extended_layer(src_info)) {
			overlapped |= is_overlap_on_yaxis(src_info, dst_info);
			if (overlapped)
				break;
		} else {
			overlapped |= is_overlap_on_yaxis(src_info, dst_info);
			if (!is_extended_base_layer_valid(src_info, i))
				overlapped |= 1;
			break;
		}
	}
	return overlapped;
}

int get_phy_ovl_layer_cnt(struct disp_layer_info *info, int disp_idx)
{
	int total_cnt = 0;
	int i;
	struct layer_config *layer_info;

	if (info->layer_num[disp_idx] > 0) {
		total_cnt = info->layer_num[disp_idx];

		if (info->gles_head[disp_idx] >= 0) {
			total_cnt -= info->gles_tail[disp_idx] -
				     info->gles_head[disp_idx];
		}

		if (get_layering_opt(LYE_OPT_EXT_LAYER)) {
			for (i = 0; i < info->layer_num[disp_idx]; i++) {
				layer_info = &info->input_config[disp_idx][i];
				if (is_extended_layer(layer_info) &&
				    !is_gles_layer(info, disp_idx, i))
					total_cnt--;
			}
		}
	}
	return total_cnt;
}

int get_phy_layer_limit(int layer_map_tb, int disp_idx)
{
	int total_cnt = 0;
	int i;

	if (disp_idx)
		layer_map_tb >>= 16;
	layer_map_tb &= 0xFFFF;

	for (i = 0; i < 16; i++) {
		if (layer_map_tb & 0x1)
			total_cnt++;
		layer_map_tb >>= 1;
	}
	return total_cnt;
}

bool is_max_lcm_resolution(void)
{
	if (debug_resolution_level == 1)
		return true;
	else if (debug_resolution_level == 2)
		return false;

	if (primary_display_get_width() > 1080)
		return true;
	else
		return false;
}

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
static int get_ovl_num(enum HRT_DISP_TYPE disp_type)
{
	unsigned int i, ovl_mapping_tb, ovl_num;

	ovl_num = 0;
	ovl_mapping_tb = l_rule_ops->get_mapping_table(DISP_HW_OVL_TB, 0);
	if (disp_type == HRT_SECONDARY)
		ovl_mapping_tb = ovl_mapping_tb >> 16;
	ovl_mapping_tb &= 0xFFFF;

	for (i = 0; i < 16; i++) {
		if (ovl_mapping_tb & 0x1)
			ovl_num++;
		ovl_mapping_tb  = ovl_mapping_tb >> 1;
	}

	return ovl_num;
}
#endif

static int get_ovl_by_phy(int layer_map_tb, int phy_layer_idx)
{
	int i, ovl_mapping_tb;
	int ovl_idx = 0, layer_idx = 0;

	ovl_mapping_tb = l_rule_ops->get_mapping_table(DISP_HW_OVL_TB, 0);
	for (layer_idx = 0; layer_idx < MAX_PHY_OVL_CNT; layer_idx++) {
		if (layer_map_tb & 0x1) {
			if (phy_layer_idx == 0)
				break;
			phy_layer_idx--;
		}
		layer_map_tb >>= 1;
	}

	if (layer_idx == MAX_PHY_OVL_CNT) {
		DISP_PR_ERR("%s fail, phy_layer_idx:%d\n", __func__,
			    phy_layer_idx);
		return -1;
	}

	for (i = 0; i < layer_idx; i++) {
		if (ovl_mapping_tb & 0x1)
			ovl_idx++;
		ovl_mapping_tb >>= 1;
	}
#ifdef HRT_DEBUG_LEVEL2
	DISPMSG("%s,phy:%d,layer_tb:0x%x,L_idx:%d ovl_idx:%d, ov_tb:0x%x\n",
		__func__, phy_layer_idx, layer_map_tb, layer_idx,
		ovl_idx, ovl_mapping_tb);
#endif
	return ovl_idx;
}

static int get_phy_ovl_index(int layer_idx)
{
	int ovl_mapping_tb = l_rule_ops->get_mapping_table(DISP_HW_OVL_TB, 0);
	int phy_layer_cnt, layer_flag;

	phy_layer_cnt = 0;
	layer_flag = 1 << layer_idx;
	while (layer_idx) {
		layer_idx--;
		layer_flag >>= 1;
		if (ovl_mapping_tb & layer_flag)
			break;
		phy_layer_cnt++;
	}

	return phy_layer_cnt;
}

static int get_larb_by_ovl(int ovl_idx, int disp_idx)
{
	int larb_mapping_tb, larb_idx;

	larb_mapping_tb = l_rule_ops->get_mapping_table(DISP_HW_LARB_TB, 0);
	if (disp_idx == HRT_SECONDARY)
		larb_mapping_tb >>= 16;

	larb_idx = (larb_mapping_tb >> ovl_idx * 4) & 0xF;

	return larb_idx;
}

static char *get_scale_name(int scale)
{
	switch (scale) {
	case HRT_SCALE_NONE:
		return "NA";
	case HRT_SCALE_133:
		return "133";
	case HRT_SCALE_150:
		return "150";
	case HRT_SCALE_200:
		return "200";
	case HRT_SCALE_266:
		return "266";
	default:
		return "unknown";
	}
}

static void dump_disp_info(struct disp_layer_info *disp_info,
			   enum DISP_DEBUG_LEVEL debug_level)
{
	int i, j;
	struct layer_config *layer_info;

#define _HRT_FMT \
	"HRT hrt_num:0x%x/fps:%d/dal:%d/p:%d/r:%s/l_tb:%d/bd_tb:%d/dc:%d/i:%d\n"
#define _L_FMT \
	"L%d->%d/(%d,%d,%d,%d)/(%d,%d,%d,%d)/f0x%x/ds%d/e%d/cap0x%x/compr%d\n"

	if (debug_level < DISP_DEBUG_LEVEL_INFO) {
		DISPMSG(_HRT_FMT,
			disp_info->hrt_num, l_rule_info->primary_fps,
			l_rule_info->dal_enable,
			HRT_GET_PATH_ID(l_rule_info->disp_path),
			get_scale_name(l_rule_info->scale_rate),
			l_rule_info->layer_tb_idx, l_rule_info->bound_tb_idx,
			HRT_GET_DC_FLAG(disp_info->hrt_num),
			roll_gpu_for_idle);

		for (i = 0; i < 2; i++) {
			if (disp_info->layer_num[i] <= 0)
				continue;

			DISPMSG("HRT D%d/M%d/LN%d/hrt:0x%x/G(%d,%d)/id%u\n",
				i, disp_info->disp_mode[i],
				disp_info->layer_num[i], disp_info->hrt_num,
				disp_info->gles_head[i],
				disp_info->gles_tail[i],
				disp_info->hrt_idx);

			for (j = 0; j < disp_info->layer_num[i]; j++) {
				layer_info = &disp_info->input_config[i][j];
				DISPMSG(_L_FMT,
					j, layer_info->ovl_id,
					layer_info->src_offset_x,
					layer_info->src_offset_y,
					layer_info->src_width,
					layer_info->src_height,
					layer_info->dst_offset_x,
					layer_info->dst_offset_y,
					layer_info->dst_width,
					layer_info->dst_height,
					layer_info->src_fmt,
					layer_info->dataspace,
					layer_info->ext_sel_layer,
					layer_info->layer_caps,
					layer_info->compress);
			}
		}
	} else {
		DISPINFO(_HRT_FMT,
			disp_info->hrt_num, l_rule_info->primary_fps,
			l_rule_info->dal_enable,
			HRT_GET_PATH_ID(l_rule_info->disp_path),
			get_scale_name(l_rule_info->scale_rate),
			l_rule_info->layer_tb_idx,
			l_rule_info->bound_tb_idx,
			HRT_GET_DC_FLAG(disp_info->hrt_num),
			roll_gpu_for_idle);

		for (i = 0; i < 2; i++) {
			if (disp_info->layer_num[i] <= 0)
				continue;

			DISPINFO("HRT D%d/M%d/LN%d/hrt:0x%x/G(%d,%d)/id%u\n",
				i, disp_info->disp_mode[i],
				disp_info->layer_num[i], disp_info->hrt_num,
				disp_info->gles_head[i],
				disp_info->gles_tail[i],
				disp_info->hrt_idx);

			for (j = 0; j < disp_info->layer_num[i]; j++) {
				layer_info = &disp_info->input_config[i][j];
				DISPINFO(_L_FMT,
					j, layer_info->ovl_id,
					layer_info->src_offset_x,
					layer_info->src_offset_y,
					layer_info->src_width,
					layer_info->src_height,
					layer_info->dst_offset_x,
					layer_info->dst_offset_y,
					layer_info->dst_width,
					layer_info->dst_height,
					layer_info->src_fmt,
					layer_info->dataspace,
					layer_info->ext_sel_layer,
					layer_info->layer_caps,
					layer_info->compress);
			}
		}
	}
}

static void print_disp_info_to_log_buffer(struct disp_layer_info *disp_info)
{
	char *status_buf;
	int i, j, n;
	struct layer_config *layer_info;

	status_buf = get_dprec_status_ptr(0);
	if (status_buf == NULL)
		return;

	n = 0;
	n += snprintf(status_buf + n, LOGGER_BUFFER_SIZE - n,
		"Last hrt query data[start]\n");
	for (i = 0; i < 2; i++) {
		n += snprintf(status_buf + n, LOGGER_BUFFER_SIZE - n,
			"HRT D%d/M%d/LN%d/hrt_num:%d/G(%d,%d)/fps:%d\n",
			i, disp_info->disp_mode[i], disp_info->layer_num[i],
			disp_info->hrt_num, disp_info->gles_head[i],
			disp_info->gles_tail[i], l_rule_info->primary_fps);

		for (j = 0; j < disp_info->layer_num[i]; j++) {
			layer_info = &disp_info->input_config[i][j];
			n += snprintf(status_buf + n, LOGGER_BUFFER_SIZE - n,
			"L%d->%d/of(%d,%d)/wh(%d,%d)/fmt:0x%x/compr:%u\n",
				j, layer_info->ovl_id,
				layer_info->dst_offset_x,
				layer_info->dst_offset_y,
				layer_info->dst_width,
				layer_info->dst_height,
				layer_info->src_fmt,
				layer_info->compress);
		}
	}
	n += snprintf(status_buf + n, LOGGER_BUFFER_SIZE - n,
		"Last hrt query data[end]\n");
}

void rollback_layer_to_GPU(struct disp_layer_info *disp_info, int disp_idx,
	int i)
{
	if (is_layer_id_valid(disp_info, disp_idx, i) == false)
		return;

	if (disp_info->gles_head[disp_idx] == -1 ||
	    disp_info->gles_head[disp_idx] > i)
		disp_info->gles_head[disp_idx] = i;
	if (disp_info->gles_tail[disp_idx] == -1 ||
		disp_info->gles_tail[disp_idx] < i)
		disp_info->gles_tail[disp_idx] = i;
	disp_info->input_config[disp_idx][i].ext_sel_layer = -1;
}

/* rollback and set NO_FBDC flag */
void rollback_compress_layer_to_GPU(struct disp_layer_info *disp_info,
	int disp_idx, int i)
{
	if (is_layer_id_valid(disp_info, disp_idx, i) == false)
		return;

	if (disp_info->input_config[disp_idx][i].compress == 0)
		return;

	rollback_layer_to_GPU(disp_info, disp_idx, i);
	disp_info->input_config[disp_idx][i].layer_caps |= NO_FBDC;
}

int rollback_resize_layer_to_GPU_range(struct disp_layer_info *disp_info,
				       int disp_idx, int start_idx, int end_idx)
{
	int i;
	struct layer_config *lc;

	if (disp_info->layer_num[disp_idx] <= 0)
		return 0;

	if (start_idx < 0 || end_idx >= disp_info->layer_num[disp_idx])
		return -EINVAL;

	for (i = start_idx; i <= end_idx; i++) {
		lc = &disp_info->input_config[disp_idx][i];
		if ((lc->src_height != lc->dst_height) ||
		    (lc->src_width != lc->dst_width)) {
			if (has_layer_cap(lc, MDP_RSZ_LAYER))
				continue;

			if (disp_info->gles_head[disp_idx] == -1 ||
			    disp_info->gles_head[disp_idx] > i)
				disp_info->gles_head[disp_idx] = i;
			if (disp_info->gles_tail[disp_idx] == -1 ||
			    disp_info->gles_tail[disp_idx] < i)
				disp_info->gles_tail[disp_idx] = i;
		}
	}

	if (disp_info->gles_head[disp_idx] != -1) {
		for (i = disp_info->gles_head[disp_idx];
		     i <= disp_info->gles_tail[disp_idx]; i++) {
			lc = &disp_info->input_config[disp_idx][i];
			lc->ext_sel_layer = -1;
		}
	}

	if (disp_idx == HRT_SECONDARY)
		return 0;

	if (l_rule_ops->rsz_by_gpu_info_change)
		l_rule_ops->rsz_by_gpu_info_change();
	else
		DISP_PR_INFO("%s, rsz_by_gpu_info_change not defined\n",
			     __func__);

	return 0;
}

int rollback_all_resize_layer_to_GPU(struct disp_layer_info *disp_info,
				     int disp_idx)
{
	rollback_resize_layer_to_GPU_range(disp_info, disp_idx, 0,
					   disp_info->layer_num[disp_idx] - 1);

	return 0;
}

static int _rollback_to_GPU_bottom_up(struct disp_layer_info *info,
				      int disp, int ovl_limit)
{
	int available_ovl_num, i, j;
	struct layer_config *l_info;

	available_ovl_num = ovl_limit;
	for (i = 0; i < info->layer_num[disp]; i++) {
		l_info = &info->input_config[disp][i];
		if (is_extended_layer(l_info))
			continue;
		available_ovl_num--;

		if (is_gles_layer(info, disp, i)) {
			info->gles_head[disp] = i;
			if (info->gles_tail[disp] == -1) {
				info->gles_tail[disp] = i;
				for (j = i + 1;
				     j < info->layer_num[disp]; j++) {
					l_info = &info->input_config[disp][j];
					if (is_extended_layer(l_info))
						info->gles_tail[disp] = j;
					else
						break;
				}
			}
			break;
		} else if (available_ovl_num <= 0) {
			available_ovl_num = 0;
			info->gles_head[disp] = i;
			info->gles_tail[disp] = info->layer_num[disp] - 1;
			break;
		}
	}

	if (available_ovl_num < 0)
		DISP_PR_ERR("%s available_ovl_num invalid:%d\n", __func__,
			    available_ovl_num);

	return available_ovl_num;
}

static int _rollback_to_GPU_top_down(struct disp_layer_info *disp_info,
				     int disp, int ovl_limit)
{
	int available_ovl_num, i;
	int tmp_ext = -1;
	struct layer_config *layer_info;

	available_ovl_num = ovl_limit;
	for (i = disp_info->layer_num[disp] - 1;
	     i > disp_info->gles_tail[disp]; i--) {
		layer_info = &disp_info->input_config[disp][i];
		if (!is_extended_layer(layer_info)) {
			if (is_gles_layer(disp_info, disp, i))
				break;
			if (available_ovl_num <= 0) {
				available_ovl_num = 0;
				if (tmp_ext == -1)
					disp_info->gles_tail[disp] = i;
				else
					disp_info->gles_tail[disp] = tmp_ext;
				break;
			}
			tmp_ext = -1;
			available_ovl_num--;
		} else {
			if (tmp_ext == -1)
				tmp_ext = i;
		}
	}

	if (available_ovl_num < 0)
		DISP_PR_ERR("%s available_ovl_num invalid:%d\n", __func__,
			    available_ovl_num);

	return available_ovl_num;
}

static int rollback_to_GPU(struct disp_layer_info *info,
			   int disp, int available)
{
	int available_ovl_num, i;
	bool has_gles_layer = false;
	struct layer_config *l_info;

	available_ovl_num = available;

	if (info->gles_head[disp] != -1)
		has_gles_layer = true;

	available_ovl_num = _rollback_to_GPU_bottom_up(info, disp,
						       available_ovl_num);
	if (has_gles_layer)
		available_ovl_num = _rollback_to_GPU_top_down(info, disp,
							available_ovl_num);

	if (is_layer_id_valid(info, disp, info->gles_head[disp]) == false) {
		dump_disp_info(info, DISP_DEBUG_LEVEL_CRITICAL);
		disp_aee_print("invalid gles_head:%d, aval:%d\n",
			info->gles_head[disp], available);
		WARN_ON(1);
	}

	if (is_layer_id_valid(info, disp, info->gles_tail[disp]) == false) {
		dump_disp_info(info, DISP_DEBUG_LEVEL_CRITICAL);
		disp_aee_print("invalid gles_tail:%d, aval:%d\n",
			info->gles_tail[disp], available);
		WARN_ON(1);
	}

	/* Clear extended layer for all GLES layer */
	for (i = info->gles_head[disp]; i <= info->gles_tail[disp]; i++) {
		l_info = &info->input_config[disp][i];
		l_info->ext_sel_layer = -1;
	}

	if (info->gles_tail[disp] + 1 < info->layer_num[disp]) {
		l_info = &info->input_config[disp][info->gles_tail[disp] + 1];
		if (is_extended_layer(l_info))
			l_info->ext_sel_layer = -1;
	}

	return available_ovl_num;
}

static int _filter_by_ovl_cnt(struct disp_layer_info *disp_info, int disp_idx)
{
	int ovl_num_limit, phy_ovl_cnt;
	int l_tb;

	if (disp_info->layer_num[disp_idx] <= 0)
		return 0;

retry:
	phy_ovl_cnt = get_phy_ovl_layer_cnt(disp_info, disp_idx);
	l_tb = l_rule_ops->get_mapping_table(DISP_HW_LAYER_TB,
					     MAX_PHY_OVL_CNT - 1);

	if (l_rule_ops->frame_has_wcg &&
	    l_rule_ops->frame_has_wcg(disp_info, HRT_PRIMARY))
		l_tb &= ~DISP_OVL_CSC_MASK;

	ovl_num_limit = get_phy_layer_limit(l_tb, disp_idx);
	if (disp_idx == 0 && l_rule_info->dal_enable)
		ovl_num_limit--;

#ifdef HRT_DEBUG_LEVEL2
	DISPMSG("phy_ovl_cnt:%d,ovl_n_limit:%d\n", phy_ovl_cnt, ovl_num_limit);
#endif
	if (phy_ovl_cnt <= ovl_num_limit)
		return 0;

	if (l_rule_ops->unset_disp_rsz_attr) {
		if (l_rule_ops->unset_disp_rsz_attr(disp_info, disp_idx))
			goto retry;
	}

	rollback_to_GPU(disp_info, disp_idx, ovl_num_limit);
	return 0;
}

static void ext_id_adjustment_and_retry(struct disp_layer_info *info,
					int disp_idx, int layer_idx)
{
	int j, ext_idx;
	struct layer_config *layer_info;

	ext_idx = -1;
	for (j = layer_idx; j < layer_idx + 3; j++) {
		layer_info = &info->input_config[disp_idx][j];

		if (ext_idx == -1) {
			layer_info->ext_sel_layer = -1;
			if (is_extended_base_layer_valid(layer_info, j))
				ext_idx = j;
		} else {
			layer_info->ext_sel_layer = ext_idx;
		}
		if (j == (info->layer_num[disp_idx] - 1) ||
		    !is_extended_layer(&info->input_config[disp_idx][j+1]))
			break;
	}
#ifdef HRT_DEBUG_LEVEL2
	DISPMSG("[%s]cannot feet current layer layout\n", __func__);
	dump_disp_info(info, DISP_DEBUG_LEVEL_ERR);
#endif
	ext_id_tunning(info, disp_idx);
}

static int ext_id_tunning(struct disp_layer_info *info, int disp)
{
	int ovl_tb, l_tb, phy_ovl_cnt, i;
	int ext_cnt = 0, cur_phy_cnt = 0;
	struct layer_config *layer_info;
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	int ovl_num = get_ovl_num(HRT_PRIMARY);
	int rc_opt = get_round_corner_opt(LYE_OPT_ROUND_CORNER);
	int rc_mode = get_round_corner_mode(rc_opt);
#endif

	if (info->layer_num[disp] <= 0)
		return 0;

	_filter_by_ovl_cnt(info, disp);
	phy_ovl_cnt = get_phy_ovl_layer_cnt(info, disp);
	if (phy_ovl_cnt > MAX_PHY_OVL_CNT) {
		DISP_PR_ERR("phy_ovl_cnt(%d) over OVL count limit\n",
			    phy_ovl_cnt);
		phy_ovl_cnt = MAX_PHY_OVL_CNT;
	}

	ovl_tb = l_rule_ops->get_mapping_table(DISP_HW_OVL_TB, 0);
	l_tb = l_rule_ops->get_mapping_table(DISP_HW_LAYER_TB, phy_ovl_cnt - 1);

	if (l_rule_ops->frame_has_wcg &&
	    l_rule_ops->frame_has_wcg(info, HRT_PRIMARY))
		l_tb &= ~DISP_OVL_CSC_MASK;

	if (l_rule_info->dal_enable) {
		l_tb = l_rule_ops->get_mapping_table(DISP_HW_LAYER_TB,
						     MAX_PHY_OVL_CNT - 1);
		l_tb &= HRT_AEE_LAYER_MASK;
	}

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	if (rc_mode == DISP_HELPER_SW_RC) {
		if (ovl_num == 1)
			ext_cnt = 1;
	}
#endif
	for (i = 0; i < info->layer_num[disp]; i++) {
		layer_info = &info->input_config[disp][i];
		if (is_extended_layer(layer_info)) {
			ext_cnt++;
			if (is_extended_over_limit(ext_cnt)) {
				ext_id_adjustment_and_retry(info, disp, i);
				break;
			}
		} else {
#ifdef HRT_DEBUG_LEVEL2
			DISPMSG("i:%d, cur_phy_cnt:%d\n", i, cur_phy_cnt);
#endif
			if (is_gles_layer(info, disp, i) &&
			    (i != info->gles_head[disp])) {
#ifdef HRT_DEBUG_LEVEL2
				DISPMSG("is gles layer, continue\n");
#endif
				continue;
			}
			if (cur_phy_cnt > 0) {
				int cur_ovl, pre_ovl;

				cur_ovl = get_ovl_by_phy(l_tb, cur_phy_cnt);
				pre_ovl = get_ovl_by_phy(l_tb, cur_phy_cnt - 1);
				if (cur_ovl != pre_ovl) {
					ext_cnt = 0;
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
					if ((rc_mode == DISP_HELPER_SW_RC) &&
					    (cur_ovl == ovl_num - 1))
						ext_cnt = 1;
#endif
				}
			}
			cur_phy_cnt++;
		}
	}

	return 0;
}

static int rollback_all_to_GPU(struct disp_layer_info *disp_info, int disp_idx)
{
	if (disp_info->layer_num[disp_idx] <= 0)
		return 0;

	disp_info->gles_head[disp_idx] = 0;
	disp_info->gles_tail[disp_idx] = disp_info->layer_num[disp_idx] - 1;
	return 0;
}

static int filter_by_ovl_cnt(struct disp_layer_info *disp_info)
{
	int ret, disp_idx;

	/* 0->primary display, 1->secondary display */
	for (disp_idx = 0; disp_idx < 2; disp_idx++) {
		if (get_layering_opt(LYE_OPT_EXT_LAYER))
			ret = ext_id_tunning(disp_info, disp_idx);
		else
			ret = _filter_by_ovl_cnt(disp_info, disp_idx);
	}

#ifdef HRT_DEBUG_LEVEL2
	DISPMSG("[%s result]\n", __func__);
	dump_disp_info(disp_info, DISP_DEBUG_LEVEL_INFO);
#endif
	return ret;
}

struct hrt_sort_entry *x_entry_list, *y_entry_list;

int dump_entry_list(bool sort_by_y)
{
	struct hrt_sort_entry *temp;
	struct layer_config *layer_info;

	if (sort_by_y)
		temp = y_entry_list;
	else
		temp = x_entry_list;

	DISPMSG("%s, sort_by_y:%d, addr:0x%p\n", __func__, sort_by_y, temp);
	while (temp) {
		layer_info = temp->layer_info;
		DISPMSG("key:%d, offset(%d, %d), w/h(%d, %d), overlap_w:%d\n",
			temp->key, layer_info->dst_offset_x,
			layer_info->dst_offset_y, layer_info->dst_width,
			layer_info->dst_height, temp->overlap_w);
		temp = temp->tail;
	}
	DISPMSG("%s end\n", __func__);
	return 0;
}

static int insert_entry(struct hrt_sort_entry **head,
			struct hrt_sort_entry *sort_entry)
{
	struct hrt_sort_entry *temp;

	temp = *head;
	while (temp) {
		if (sort_entry->key < temp->key ||
		    ((sort_entry->key == temp->key) &&
		    (sort_entry->overlap_w > 0))) {
			sort_entry->head = temp->head;
			sort_entry->tail = temp;
			if (temp->head != NULL)
				temp->head->tail = sort_entry;
			else
				*head = sort_entry;
			temp->head = sort_entry;
			break;
		}

		if (temp->tail == NULL) {
			temp->tail = sort_entry;
			sort_entry->head = temp;
			sort_entry->tail = NULL;
			break;
		}
		temp = temp->tail;
	}

	return 0;
}

static int add_layer_entry(struct layer_config *l_info,
			   bool sort_by_y, int overlap_w)
{
	struct hrt_sort_entry *begin_t, *end_t;
	struct hrt_sort_entry **p_entry;

	begin_t = kzalloc(sizeof(struct hrt_sort_entry), GFP_KERNEL);
	end_t = kzalloc(sizeof(struct hrt_sort_entry), GFP_KERNEL);

	begin_t->head = NULL;
	begin_t->tail = NULL;
	end_t->head = NULL;
	end_t->tail = NULL;
	if (sort_by_y) {
		begin_t->key = l_info->dst_offset_y;
		end_t->key = l_info->dst_offset_y + l_info->dst_height - 1;
		p_entry = &y_entry_list;
	} else {
		begin_t->key = l_info->dst_offset_x;
		end_t->key = l_info->dst_offset_x + l_info->dst_width - 1;
		p_entry = &x_entry_list;
	}

	begin_t->overlap_w = overlap_w;
	begin_t->layer_info = l_info;
	end_t->overlap_w = -overlap_w;
	end_t->layer_info = l_info;

	if (*p_entry == NULL) {
		*p_entry = begin_t;
		begin_t->head = NULL;
		begin_t->tail = end_t;
		end_t->head = begin_t;
		end_t->tail = NULL;
	} else {
		/* Inser begin entry */
		insert_entry(p_entry, begin_t);
#ifdef HRT_DEBUG_LEVEL2
		DISPMSG("Insert key:%d\n", begin_t->key);
		dump_entry_list(sort_by_y);
#endif
		/* Inser end entry */
		insert_entry(p_entry, end_t);
#ifdef HRT_DEBUG_LEVEL2
		DISPMSG("Insert key:%d\n", end_t->key);
		dump_entry_list(sort_by_y);
#endif
	}

	return 0;
}

static int remove_layer_entry(struct layer_config *layer_info, bool sort_by_y)
{
	struct hrt_sort_entry *temp, *free_entry;

	if (sort_by_y)
		temp = y_entry_list;
	else
		temp = x_entry_list;

	while (temp) {
		if (temp->layer_info == layer_info) {
			free_entry = temp;
			temp = temp->tail;
			if (free_entry->head == NULL) {
				/* Free head entry */
				if (temp != NULL)
					temp->head = NULL;
				if (sort_by_y)
					y_entry_list = temp;
				else
					x_entry_list = temp;
				kfree(free_entry);
			} else {
				free_entry->head->tail = free_entry->tail;
				if (temp)
					temp->head = free_entry->head;
				kfree(free_entry);
			}
		} else {
			temp = temp->tail;
		}
	}
	return 0;
}

static int free_all_layer_entry(bool sort_by_y)
{
	struct hrt_sort_entry *cur_entry, *next_entry;

	if (sort_by_y)
		cur_entry = y_entry_list;
	else
		cur_entry = x_entry_list;

	while (cur_entry) {
		next_entry = cur_entry->tail;
		kfree(cur_entry);
		cur_entry = next_entry;
	}

	if (sort_by_y)
		y_entry_list = NULL;
	else
		x_entry_list = NULL;

	return 0;
}

static int scan_x_overlap(struct disp_layer_info *disp_info, int disp_index,
			  int ovl_overlap_limit_w)
{
	struct hrt_sort_entry *tmp_entry;
	int overlap_w_sum, max_overlap;

	overlap_w_sum = 0;
	max_overlap = 0;
	tmp_entry = x_entry_list;
	while (tmp_entry) {
		overlap_w_sum += tmp_entry->overlap_w;
		max_overlap = (overlap_w_sum > max_overlap) ?
				overlap_w_sum : max_overlap;
		tmp_entry = tmp_entry->tail;
	}
	return max_overlap;
}

static int scan_y_overlap(struct disp_layer_info *disp_info,
			  int disp_index, int ovl_overlap_limit_w)
{
	struct hrt_sort_entry *tmp_entry;
	int overlap_w_sum, tmp_overlap, max_overlap;

	overlap_w_sum = 0;
	tmp_overlap = 0;
	max_overlap = 0;
	tmp_entry = y_entry_list;
	while (tmp_entry) {
		overlap_w_sum += tmp_entry->overlap_w;
		if (tmp_entry->overlap_w > 0) {
			add_layer_entry(tmp_entry->layer_info,
					false, tmp_entry->overlap_w);
		} else {
			remove_layer_entry(tmp_entry->layer_info, false);
		}

		if (overlap_w_sum > ovl_overlap_limit_w &&
		    overlap_w_sum > max_overlap) {
			tmp_overlap = scan_x_overlap(disp_info, disp_index,
						     ovl_overlap_limit_w);
		} else {
			tmp_overlap = overlap_w_sum;
		}

		max_overlap = (tmp_overlap > max_overlap) ?
				tmp_overlap : max_overlap;
		tmp_entry = tmp_entry->tail;
	}

	return max_overlap;
}

static int get_hrt_level(int sum_w, int is_larb)
{
	int hrt_level;
	int *bound_table;
	enum DISP_HW_MAPPING_TB_TYPE type;

	if (is_larb)
		type = DISP_HW_LARB_BOUND_TB;
	else
		type = DISP_HW_EMI_BOUND_TB;

	bound_table = l_rule_ops->get_bound_table(type);
	for (hrt_level = 0; hrt_level < HRT_LEVEL_NUM; hrt_level++) {
		if (bound_table[hrt_level] != -1 &&
		    sum_w <= bound_table[hrt_level] * HRT_UINT_BOUND_BPP)
			return hrt_level;
	}
	return hrt_level;
}

static bool has_hrt_limit(struct disp_layer_info *disp_info, int disp_idx)
{
	if (disp_info->layer_num[disp_idx] <= 0)
		return false;

	/*
	 * after we request DC mode, we need to constantly check
	 * hrt num for requesting DL next time
	 */
	if (disp_idx == HRT_PRIMARY && adaptive_dc_request)
		return true;

	if (disp_info->disp_mode[disp_idx] == DISP_SESSION_DECOUPLE_MODE ||
	    disp_info->disp_mode[disp_idx] == DISP_SESSION_DECOUPLE_MIRROR_MODE)
		return false;

	return true;
}

/**
 * Return the HRT layer weight.
 * If the layer_info is NULL, return GLES layer weight.
 */
static int get_layer_weight(int disp_idx, struct layer_config *layer_info)
{
	int bpp, weight;

	if (layer_info)
		bpp = get_bpp(layer_info->src_fmt);
	else
		bpp = HRT_UINT_BOUND_BPP;
#ifdef CONFIG_MTK_HDMI_SUPPORT
	if (disp_idx == HRT_SECONDARY) {
		struct disp_session_info dispif_info;

		/* For seconary display, set the weight 4K@30 as 2K@60.	*/
		hdmi_get_dev_info(true, &dispif_info);

		if (dispif_info.displayWidth > 2560)
			weight = HRT_UINT_WEIGHT * 2;
		else if (dispif_info.displayWidth > 1920)
			weight = HRT_UINT_WEIGHT;
		else
			weight = HRT_UINT_WEIGHT / 2;

		if (dispif_info.vsyncFPS <= 30)
			weight /= 2;

		return weight * bpp;
	}
#endif

	/* Resize layer weight adjustment */
	if (layer_info && layer_info->dst_width != layer_info->src_width) {
		switch (l_rule_info->scale_rate) {
#if 0
		case HRT_SCALE_200:
			weight = HRT_UINT_WEIGHT * 3 / 8;
			break;
		case HRT_SCALE_200:
			weight = HRT_UINT_WEIGHT / 2;
			break;
		case HRT_SCALE_150:
			weight = HRT_UINT_WEIGHT * 2 / 3;
			break;
		case HRT_SCALE_133:
			weight = HRT_UINT_WEIGHT * 3 / 4;
			break;
#endif
		default:
			weight = HRT_UINT_WEIGHT;
			break;
		}
	} else {
		weight = HRT_UINT_WEIGHT;
	}

	return weight * bpp;
}

static int _calc_hrt_num(struct disp_layer_info *disp_info, int disp,
			 int hrt_type, bool force_scan_y, bool has_dal_layer)
{
	int i, sum_overlap_w, overlap_l_bound, layer_map;
	int overlap_w, layer_idx, phy_layer_idx, ovl_cnt;
	bool has_gles = false;
	struct layer_config *layer_info;
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	int rc_opt = get_round_corner_opt(LYE_OPT_ROUND_CORNER);
	int rc_mode = get_round_corner_mode(rc_opt);
#endif

	if (!has_hrt_limit(disp_info, disp))
		return 0;

	/* 1.Initial overlap conditions. */
	sum_overlap_w = 0;
	/*
	 * The parameters of hrt table are base on ARGB color format.
	 * Multiply the bpp of it.
	 */
	overlap_l_bound = g_emi_bound_table[0] * HRT_UINT_BOUND_BPP;

	/*
	 * 2.Add each layer info to layer list and sort it by yoffset.
	 * Also add up each layer overlap weight.
	 */
	layer_idx = -1;
	ovl_cnt = get_phy_ovl_layer_cnt(disp_info, disp);
	layer_map = l_rule_ops->get_mapping_table(DISP_HW_LAYER_TB,
						  ovl_cnt - 1);

	if (l_rule_ops->frame_has_wcg &&
	    l_rule_ops->frame_has_wcg(disp_info, HRT_PRIMARY))
		layer_map &= ~DISP_OVL_CSC_MASK;

	if (l_rule_info->dal_enable) {
		layer_map = l_rule_ops->get_mapping_table(DISP_HW_LAYER_TB,
							  MAX_PHY_OVL_CNT - 1);
		layer_map &= HRT_AEE_LAYER_MASK;
	}

	for (i = 0; i < disp_info->layer_num[disp]; i++) {
		int ovl_idx;

		layer_info = &disp_info->input_config[disp][i];
		if (disp_info->gles_head[disp] == -1 ||
		    (i < disp_info->gles_head[disp] ||
		     i > disp_info->gles_tail[disp])) {
			if (hrt_type != HRT_TYPE_EMI) {
				if (layer_idx == -1)
					layer_idx = 0;
				else if (!is_extended_layer(layer_info))
					layer_idx++;

				phy_layer_idx = get_phy_ovl_index(layer_idx);
				ovl_idx = get_ovl_by_phy(layer_map, layer_idx);
				if (get_larb_by_ovl(ovl_idx, disp) != hrt_type)
					continue;
			}
			overlap_w = get_layer_weight(disp, layer_info);
			sum_overlap_w += overlap_w;
			add_layer_entry(layer_info, true, overlap_w);
		} else if (i == disp_info->gles_head[disp]) {
			/* Add GLES layer */
			if (hrt_type != HRT_TYPE_EMI) {
				if (layer_idx == -1)
					layer_idx = 0;
				else if (!is_extended_layer(layer_info))
					layer_idx++;

				phy_layer_idx = get_phy_ovl_index(layer_idx);
				ovl_idx = get_ovl_by_phy(layer_map, layer_idx);

				if (get_larb_by_ovl(ovl_idx, disp) != hrt_type)
					continue;
			}
			has_gles = true;
		}
	}
	/* Add overlap weight of Gles layer and Assert layer. */
	if (has_gles)
		sum_overlap_w += get_layer_weight(disp, NULL);

	if (has_dal_layer)
		sum_overlap_w += HRT_AEE_WEIGHT;

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	if (rc_mode == DISP_HELPER_SW_RC)
		sum_overlap_w += HRT_ROUND_CORNER_WEIGHT;
#endif

	/*
	 * 3.Calculate the HRT bound if the total layer weight over the
	 * lower bound or has secondary display.
	 */
	if (sum_overlap_w > overlap_l_bound ||
	    has_hrt_limit(disp_info, HRT_SECONDARY) || force_scan_y) {
		sum_overlap_w = scan_y_overlap(disp_info, disp,
					       overlap_l_bound);
		/* Add overlap weight of Gles layer and Assert layer. */
		if (has_gles)
			sum_overlap_w += get_layer_weight(disp, NULL);
		if (has_dal_layer)
			sum_overlap_w += HRT_AEE_WEIGHT;
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
		if (rc_mode == DISP_HELPER_SW_RC)
			sum_overlap_w += HRT_ROUND_CORNER_WEIGHT;
#endif
	}

#ifdef HRT_DEBUG_LEVEL1
	DISPMSG("%s disp:%d, disp:%d, hrt_type:%d, sum_overlap_w:%d\n",
		__func__, disp, disp, hrt_type, sum_overlap_w);
#endif

	free_all_layer_entry(true);
	return sum_overlap_w;
}

#ifdef HAS_LARB_HRT
static int calc_larb_hrt_level(struct disp_layer_info *disp_info)
{
	int larb_hrt_level, i, sum_overlap_w;

	larb_hrt_level = 0;
	for (i = HRT_TYPE_LARB0; i <= HRT_TYPE_LARB1; i++) {
		int tmp_hrt_level;

		sum_overlap_w = _calc_hrt_num(disp_info, HRT_PRIMARY, i, true,
					      l_rule_info->dal_enable);
		sum_overlap_w += _calc_hrt_num(disp_info, HRT_SECONDARY, i,
					       true, false);
		tmp_hrt_level = get_hrt_level(sum_overlap_w, true);
		if (tmp_hrt_level > larb_hrt_level)
			larb_hrt_level = tmp_hrt_level;
	}

	return larb_hrt_level;
}
#endif

static int calc_hrt_num(struct disp_layer_info *disp_info)
{
	int emi_hrt_level;
	int sum_overlap_w = 0;
#ifdef HAS_LARB_HRT
	int larb_hrt_level;
#endif
	int overlap_num;
	bool scan_overlap = !!disp_helper_get_option(DISP_OPT_HRT_MODE);

	/* Calculate HRT for EMI level */
	if (has_hrt_limit(disp_info, HRT_PRIMARY)) {
		sum_overlap_w = _calc_hrt_num(disp_info, HRT_PRIMARY,
					      HRT_TYPE_EMI, scan_overlap,
					      l_rule_info->dal_enable);
	}
	if (has_hrt_limit(disp_info, HRT_SECONDARY)) {
		sum_overlap_w += _calc_hrt_num(disp_info, HRT_SECONDARY,
					       HRT_TYPE_EMI, scan_overlap,
					       false);
	}

	emi_hrt_level = get_hrt_level(sum_overlap_w, false);

	overlap_num = sum_overlap_w / 200;

	/*
	 * The larb bound always meets the limit for HRT_LEVEL2
	 * in 8+4 ovl architecture.
	 * So calculate larb bound only for HRT_LEVEL2.
	 */
	disp_info->hrt_num = emi_hrt_level;
#ifdef HRT_DEBUG_LEVEL1
	DISPMSG("EMI hrt lv2:%d,overlap_w:%d\n", emi_hrt_level, sum_overlap_w);
#endif

#ifdef HAS_LARB_HRT
	/* Need to calculate larb hrt for HRT_LEVEL_LOW level. */
	/* TODO: Should revise larb calculation statement here */
	/* if (hrt_level != HRT_LEVEL_NUM - 2) */
	/*	return hrt_level; */

	/* Check Larb Bound here */
	larb_hrt_level = calc_larb_hrt_level(disp_info);

#ifdef HRT_DEBUG_LEVEL1
	DISPMSG("Larb hrt level:%d\n", larb_hrt_level);
#endif

	if (emi_hrt_level < larb_hrt_level)
		disp_info->hrt_num = larb_hrt_level;
	else
#endif
		disp_info->hrt_num = emi_hrt_level;

	return overlap_num;
}

/**
 * dispatch which one layer could be ext layer
 */
static int ext_layer_grouping(struct disp_layer_info *disp_info)
{
	int cont_ext_layer_cnt = 0, ext_idx = 0;
	int is_ext_layer, disp_idx, i;
	struct layer_config *src_info, *dst_info;
	int available_layers = 0, phy_layer_cnt = 0;

	for (disp_idx = 0; disp_idx < 2; disp_idx++) {
		/* initialize ext layer info */
		for (i = 0; i < disp_info->layer_num[disp_idx]; i++)
			disp_info->input_config[disp_idx][i].ext_sel_layer = -1;

		if (!get_layering_opt(LYE_OPT_EXT_LAYER))
			continue;

#ifndef LAYERING_SUPPORT_EXT_LAYER_ON_2ND_DISP
		if (disp_idx == HRT_SECONDARY)
			continue;
#endif

		/*
		 * If the physical layer > input layer,
		 * then skip using extended layer.
		 */
		phy_layer_cnt = get_phy_layer_limit(
					l_rule_ops->get_mapping_table(
						DISP_HW_LAYER_TB,
						MAX_PHY_OVL_CNT - 1), disp_idx);
		if (phy_layer_cnt > disp_info->layer_num[disp_idx])
			continue;

		for (i = 1; i < disp_info->layer_num[disp_idx]; i++) {
			dst_info = &disp_info->input_config[disp_idx][i];
			src_info = &disp_info->input_config[disp_idx][i-1];
			/* skip other GPU layers */
			if (is_gles_layer(disp_info, disp_idx, i) ||
			    is_gles_layer(disp_info, disp_idx, i - 1)) {
				cont_ext_layer_cnt = 0;
				if (i > disp_info->gles_tail[disp_idx]) {
					int tmp;

					tmp = disp_info->gles_tail[disp_idx] -
						disp_info->gles_head[disp_idx];
					ext_idx = i - tmp;
				}
				continue;
			}

			is_ext_layer = !is_continuous_ext_layer_overlap(
					disp_info->input_config[disp_idx], i);

			/*
			 * The yuv layer is not supported as extended layer
			 * as the HWC has a special for yuv content.
			 */
			if (is_yuv(dst_info->src_fmt))
				is_ext_layer = false;

			if (is_ext_layer && cont_ext_layer_cnt < 3) {
				++cont_ext_layer_cnt;
				dst_info->ext_sel_layer = ext_idx;
			} else {
				cont_ext_layer_cnt = 0;
				ext_idx = i;
				if (i > disp_info->gles_tail[disp_idx]) {
					ext_idx -=
						disp_info->gles_tail[disp_idx] -
						disp_info->gles_head[disp_idx];
				}
			}
		}
	}

#ifdef HRT_DEBUG_LEVEL1
	DISPMSG("[ext layer grouping]\n");
	dump_disp_info(disp_info, DISP_DEBUG_LEVEL_INFO);
#endif

	return available_layers;
}

static int _dispatch_ovl_id(struct disp_layer_info *disp_info, int layer_map,
			    int disp_idx)
{
	struct layer_config *layer_info;
	int i, j, ghead, gtail;
	int layer_idx = 0, ext_cnt = 0;

	ghead = disp_info->gles_head[disp_idx];
	gtail = disp_info->gles_tail[disp_idx];
	for (i = 0; i < TOTAL_OVL_LAYER_NUM; i++) {
		if ((layer_map & 0x1) == 0) {
			layer_map >>= 1;
			continue;
		}

		layer_info = &disp_info->input_config[disp_idx][layer_idx];
		layer_info->ovl_id = i + ext_cnt;
		if (is_gles_layer(disp_info, disp_idx, layer_idx)) {
			struct layer_config *gles_layer_info;

			for (j = ghead; j <= gtail; j++) {
				gles_layer_info =
					&disp_info->input_config[disp_idx][j];
				gles_layer_info->ovl_id = layer_info->ovl_id;
			}
			layer_idx += (gtail - ghead) + 1;
		} else {
			int phy_layer_idx;
			struct layer_config *l_configs;

			layer_idx++;
			phy_layer_idx = get_phy_ovl_index(i);
			l_configs = disp_info->input_config[disp_idx];
			for (j = 0; j < 3; j++) {
				if (layer_idx >= disp_info->layer_num[disp_idx])
					break;

				layer_info = &l_configs[layer_idx];
				if (is_extended_layer(layer_info)) {
					ext_cnt++;
					layer_info->ovl_id = i + ext_cnt;
					layer_idx++;
					layer_info->ext_sel_layer =
							phy_layer_idx;
				} else {
					break;
				}
			}
		}
		if (layer_idx >= disp_info->layer_num[disp_idx])
			break;

		layer_map >>= 1;
	}
	return 0;
}

static int dispatch_ovl_id(struct disp_layer_info *disp_info)
{
	int disp_idx;
	bool has_second_disp;
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	int rc_opt = get_round_corner_opt(LYE_OPT_ROUND_CORNER);
	int rc_mode = get_round_corner_mode(rc_opt);
#endif

	if (disp_info->layer_num[0] <= 0 && disp_info->layer_num[1] <= 0)
		return 0;

	if (disp_info->layer_num[1] > 0)
		has_second_disp = true;
	else
		has_second_disp = false;

	/* Dispatch gles range if necessary */
	if (disp_info->hrt_num > HRT_LEVEL_NUM - 1) {
		int max_ovl_cnt = g_emi_bound_table[HRT_LEVEL_NUM-1];
		int valid_ovl_cnt = max_ovl_cnt;

		if (l_rule_info->dal_enable)
			valid_ovl_cnt -= (HRT_AEE_WEIGHT / HRT_UINT_BOUND_BPP);

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
		if (rc_mode == DISP_HELPER_SW_RC)
			valid_ovl_cnt = valid_ovl_cnt -
				(HRT_ROUND_CORNER_WEIGHT / HRT_UINT_BOUND_BPP);
#endif

		valid_ovl_cnt /= HRT_UINT_WEIGHT;
		if (has_hrt_limit(disp_info, HRT_SECONDARY))
			valid_ovl_cnt = rollback_to_GPU(disp_info,
					HRT_SECONDARY, valid_ovl_cnt - 1) + 1;

		if (has_hrt_limit(disp_info, HRT_PRIMARY))
			rollback_to_GPU(disp_info, HRT_PRIMARY, valid_ovl_cnt);

		/* ajust hrt_num */
		disp_info->hrt_num = get_hrt_level(max_ovl_cnt *
							HRT_UINT_BOUND_BPP, 0);
		disp_info->hrt_weight = max_ovl_cnt * 2 / HRT_UINT_WEIGHT;
	}

	/* Dispatch OVL id */
	for (disp_idx = 0; disp_idx < 2; disp_idx++) {
		int ovl_cnt, layer_map, layer_idx, ext_cnt, gles_cnt;

		if (disp_info->layer_num[disp_idx] <= 0)
			continue;
		ovl_cnt = get_phy_ovl_layer_cnt(disp_info, disp_idx);
		layer_map = l_rule_ops->get_mapping_table(DISP_HW_LAYER_TB,
							  ovl_cnt - 1);

		if (l_rule_ops->frame_has_wcg &&
		    l_rule_ops->frame_has_wcg(disp_info, HRT_PRIMARY))
			layer_map &= ~DISP_OVL_CSC_MASK;

		if (l_rule_info->dal_enable) {
			layer_map = l_rule_ops->get_mapping_table(
					DISP_HW_LAYER_TB, MAX_PHY_OVL_CNT - 1);
			layer_map &= HRT_AEE_LAYER_MASK;
		}

		layer_idx = 0;
		ext_cnt = 0;
		gles_cnt = 0;

		if (disp_idx == 0)
			layer_map &= 0x0000FFFF;
		else
			layer_map = (layer_map & 0xFFFF0000) >> 16;

		_dispatch_ovl_id(disp_info, layer_map, disp_idx);
	}
	return 0;
}

static int check_layering_result(struct disp_layer_info *info)
{
	int disp_idx;

	if (info->layer_num[0] <= 0 && info->layer_num[1] <= 0)
		return 0;

	for (disp_idx = 0; disp_idx < 2; disp_idx++) {
		int layer_num, max_ovl_id, ovl_layer_num;

		if (info->layer_num[disp_idx] <= 0)
			continue;

		if (disp_idx == HRT_PRIMARY)
			ovl_layer_num = PRIMARY_OVL_LAYER_NUM;
		else
			ovl_layer_num = SECONDARY_OVL_LAYER_NUM;
		layer_num = info->layer_num[disp_idx];
		max_ovl_id = info->input_config[disp_idx][layer_num - 1].ovl_id;

		if (max_ovl_id >= ovl_layer_num) {
			DISP_PR_ERR("Inv ovl:%d,disp:%d\n",
				    max_ovl_id, disp_idx);
			WARN_ON(1);
		}
	}
	return 0;
}

int check_disp_info(struct disp_layer_info *disp_info)
{
	int disp_idx = 0;
	int ghead = 0;
	int gtail = 0;
	int layer_num = 0;

	if (disp_info == NULL) {
		DISP_PR_ERR("[HRT]disp_info is empty\n");
		return -1;
	}

	for (disp_idx = 0 ; disp_idx < 2 ; disp_idx++) {

		layer_num = disp_info->layer_num[disp_idx];
		if (layer_num > 0 &&
			disp_info->input_config[disp_idx] == NULL) {
			DISP_PR_ERR("[HRT]input config is empty,disp:%d,l_num:%d\n",
				disp_idx, disp_info->layer_num[disp_idx]);
			return -1;
		}

		ghead = disp_info->gles_head[disp_idx];
		gtail = disp_info->gles_tail[disp_idx];
		if ((!((ghead == -1) && (gtail == -1)) &&
			!((ghead >= 0) && (gtail >= 0)))
			|| (ghead >= layer_num) || (gtail >= layer_num)
			|| (ghead > gtail)) {
			DISP_PR_ERR("[HRT]gles invalid,disp:%d,head:%d,tail:%d\n",
				disp_idx, disp_info->gles_head[disp_idx],
				disp_info->gles_tail[disp_idx]);
			return -1;
		}
	}

	return 0;
}

static int _copy_layer_info_from_disp(struct disp_layer_info *disp_info_user,
	int debug_mode, int disp_idx)
{
	struct disp_layer_info *l_info = &layering_info;
	unsigned long int layer_size = 0;
	int ret = 0, layer_num = 0;

	if (l_info->layer_num[disp_idx] <= 0)
		return 0;

	layer_num = l_info->layer_num[disp_idx];
	layer_size = sizeof(struct layer_config) * layer_num;
	l_info->input_config[disp_idx] = vzalloc(layer_size);

	if (l_info->input_config[disp_idx] == NULL)
		return -ENOMEM;

	if (debug_mode) {
		memcpy(l_info->input_config[disp_idx],
		       disp_info_user->input_config[disp_idx], layer_size);
	} else {
		if (copy_from_user(l_info->input_config[disp_idx],
				   disp_info_user->input_config[disp_idx],
				   layer_size)) {
			pr_info("[DISP][FB]: copy_to_user failed! line:%d\n",
				__LINE__);

			return 0;
		}
	}

	return ret;
}

int set_disp_info(struct disp_layer_info *disp_info_user, int debug_mode)
{
	int ret;

	memcpy(&layering_info, disp_info_user, sizeof(struct disp_layer_info));

	ret = _copy_layer_info_from_disp(disp_info_user, debug_mode, 0);
	if (ret)
		return ret;

	ret = _copy_layer_info_from_disp(disp_info_user, debug_mode, 1);
	if (ret) {
		vfree(layering_info.input_config[0]);
		return ret;
	}

	l_rule_info->disp_path = HRT_PATH_UNKNOWN;

	return 0;
}

static int _copy_layer_info_by_disp(struct disp_layer_info *disp_info_user,
				    int debug_mode, int disp_idx)
{
	struct disp_layer_info *l_info = &layering_info;
	unsigned long int layer_size = 0;
	int ret = 0;

	if (l_info->layer_num[disp_idx] <= 0)
		return -EFAULT;

	disp_info_user->gles_head[disp_idx] = l_info->gles_head[disp_idx];
	disp_info_user->gles_tail[disp_idx] = l_info->gles_tail[disp_idx];

	layer_size = sizeof(struct layer_config) *
			disp_info_user->layer_num[disp_idx];

	if (debug_mode) {
		memcpy(disp_info_user->input_config[disp_idx],
		       l_info->input_config[disp_idx], layer_size);
	} else {
		if (copy_to_user(disp_info_user->input_config[disp_idx],
				 l_info->input_config[disp_idx], layer_size)) {
			pr_info("[DISP][FB]: copy_to_user failed! line:%d\n",
				__LINE__);
			ret = -EFAULT;
		}
		vfree(l_info->input_config[disp_idx]);
	}

	return ret;
}

int copy_layer_info_to_user(struct disp_layer_info *disp_info_user,
			    int debug_mode)
{
	int ret = 0;
	struct disp_layer_info *l_info = &layering_info;

	disp_info_user->hrt_num = l_info->hrt_num;
	disp_info_user->hrt_idx = l_info->hrt_idx;
	disp_info_user->hrt_weight = l_info->hrt_weight;
	_copy_layer_info_by_disp(disp_info_user, debug_mode, 0);
	_copy_layer_info_by_disp(disp_info_user, debug_mode, 1);

	return ret;
}

int set_hrt_state(enum HRT_SYS_STATE sys_state, int en)
{
	switch (sys_state) {
	case DISP_HRT_MJC_ON:
		if (en)
			l_rule_info->hrt_sys_state |= (1 << sys_state);
		else
			l_rule_info->hrt_sys_state &= ~(1 << sys_state);
		break;
	case DISP_HRT_FORCE_DUAL_OFF:
		if (en)
			l_rule_info->hrt_sys_state |= (1 << sys_state);
		else
			l_rule_info->hrt_sys_state &= ~(1 << sys_state);
		break;
	case DISP_HRT_MULTI_TUI_ON:
		if (en)
			l_rule_info->hrt_sys_state |= (1 << sys_state);
		else
			l_rule_info->hrt_sys_state &= ~(1 << sys_state);
		break;
	default:
		DISP_PR_ERR("unknown hrt scenario\n");
		break;
	}

	DISPMSG("Set hrt sys_state:%d, en:%d\n", sys_state, en);
	return 0;
}

void register_layering_rule_ops(struct layering_rule_ops *ops,
				struct layering_rule_info_t *info)
{
	l_rule_ops = ops;
	l_rule_info = info;
}

static void handle_for_set_disp_info_fail(struct disp_layer_info *info)
{
	int i, j;

	for (i = 0; i <= 1; i++) {
		struct layer_config tmp;
		int l_num = info->layer_num[i];

		if (l_num <= 0)
			continue;

		rollback_all_to_GPU(info, i);

		for (j = 0; j < l_num; j++) {
			if (copy_from_user(&tmp, &info->input_config[i][j],
				sizeof(struct layer_config)))
				DISP_PR_ERR("%s: copy_from_user fail\n",
					__func__);

			tmp.ovl_id = 0;

			if (copy_to_user(&info->input_config[i][j], &tmp,
				sizeof(struct layer_config)))
				DISP_PR_ERR("%s: copy_to_user fail\n",
					__func__);
		}
	}

	info->hrt_weight = 4; /* force overlap = 2 layer */
	info->hrt_num = 0;
	info->hrt_idx = HRT_LEVEL_LEVEL0;
}

int layering_rule_start(struct disp_layer_info *disp_info_user, int debug_mode)
{
	int ret;
	int overlap_num;

	roll_gpu_for_idle = 0;

	if (l_rule_ops == NULL || l_rule_info == NULL) {
		DISP_PR_INFO("Layering rule has not been initialize.\n");
		return -EFAULT;
	}

	if (check_disp_info(disp_info_user) < 0) {
		DISP_PR_ERR("check_disp_info fail\n");
		return -EFAULT;
	}

	/* set_disp_info return error code, do error handing:
	 * 1. Rollback all layers to GPU
	 * 2. Do not return error code in IOCTL, HWC did not handle this...
	 */
	ret = set_disp_info(disp_info_user, debug_mode);
	if (ret) {
		DISP_PR_ERR("%s: set_disp_info fail, ret=%d\n",
			__func__, ret);

		handle_for_set_disp_info_fail(disp_info_user);

		return -EFAULT;
	}

	print_disp_info_to_log_buffer(&layering_info);
#ifdef HRT_DEBUG_LEVEL1
	DISPMSG("[Input data]\n");
	dump_disp_info(&layering_info, DISP_DEBUG_LEVEL_INFO);
#endif
	l_rule_info->disp_path = HRT_PATH_UNKNOWN;

	l_rule_info->hrt_idx++;
	if (l_rule_info->hrt_idx == 0xffffffff)
		l_rule_info->hrt_idx = 0;

	l_rule_ops->copy_hrt_bound_table(0, g_emi_bound_table);

	/* 1.Pre-distribution */
	l_rule_info->dal_enable = is_DAL_Enabled();

	if (l_rule_ops->rollback_to_gpu_by_hw_limitation)
		ret = l_rule_ops->rollback_to_gpu_by_hw_limitation(
							&layering_info);

	/* Check and choose the Resize Scenario */
	if (get_layering_opt(LYE_OPT_RPO)) {
		if (l_rule_ops->resizing_rule)
			ret = l_rule_ops->resizing_rule(&layering_info);
		else
			DISP_PR_INFO("RSZ on, but no resizing rule.\n");
	} else {
		rollback_all_resize_layer_to_GPU(&layering_info,
			HRT_PRIMARY);
		rollback_all_resize_layer_to_GPU(&layering_info,
			HRT_SECONDARY);
		l_rule_info->scale_rate = HRT_SCALE_NONE;
	}

	/* fbdc_rule should be after resizing_rule
	 * for optimizing secondary display BW
	 */
	if (l_rule_ops->fbdc_rule)
		l_rule_ops->fbdc_rule(&layering_info);

	/* Add for FBDC */
	if (l_rule_ops->fbdc_pre_calculate)
		l_rule_ops->fbdc_pre_calculate(&layering_info);

	/* Initial HRT conditions */
	l_rule_ops->scenario_decision(&layering_info);

	/* Layer Grouping */
	if (l_rule_ops->fbdc_adjust_layout)
		l_rule_ops->fbdc_adjust_layout(&layering_info,
			ADJUST_LAYOUT_EXT_GROUPING);

	ret = ext_layer_grouping(&layering_info);

	if (l_rule_ops->fbdc_restore_layout)
		l_rule_ops->fbdc_restore_layout(&layering_info,
			ADJUST_LAYOUT_EXT_GROUPING);

	/* GLES adjustment and ext layer checking */
	ret = filter_by_ovl_cnt(&layering_info);

	/*
	 * 2.Overlapping
	 * Calculate overlap number of available input layers.
	 * If the overlap number is out of bound, then decrease
	 * the number of available layers to overlap number.
	 */
	/* [PVRIC] change dst layout before calculate overlap */
	if (l_rule_ops->fbdc_adjust_layout)
		l_rule_ops->fbdc_adjust_layout(&layering_info,
			ADJUST_LAYOUT_OVERLAP_CAL);
	overlap_num = calc_hrt_num(&layering_info);
	layering_info.hrt_weight = overlap_num;

	if (l_rule_ops->fbdc_restore_layout)
		l_rule_ops->fbdc_restore_layout(&layering_info,
			ADJUST_LAYOUT_OVERLAP_CAL);

	/*
	 * 3.Dispatching
	 * Fill layer id for each input layers.
	 * All the gles layers set as same layer id.
	 */
	if (l_rule_ops->rollback_all_to_GPU_for_idle != NULL &&
	    l_rule_ops->rollback_all_to_GPU_for_idle()) {
		roll_gpu_for_idle = 1;
		rollback_all_to_GPU(&layering_info, HRT_PRIMARY);
		layering_info.hrt_num = HRT_LEVEL_LEVEL0;
		layering_info.hrt_weight = 2;
	} else if (l_rule_ops->adaptive_dc_enabled == NULL ||
		   !l_rule_ops->adaptive_dc_enabled() ||
		   l_rule_info->dal_enable ||
		   layering_info.hrt_num < HRT_LEVEL_NUM) {
		/* do not request DC mode */
		adaptive_dc_request = 0;
	} else if (adaptive_dc_request == 0) {
		/* the first time of request DC mode, rollback all to GPU */
		adaptive_dc_request = 1;
		rollback_all_to_GPU(&layering_info, HRT_PRIMARY);
	}
	if (adaptive_dc_request)
		layering_info.hrt_num = 0;

	ret = dispatch_ovl_id(&layering_info);

	check_layering_result(&layering_info);

	layering_info.hrt_idx = l_rule_info->hrt_idx;
	HRT_SET_PATH_SCENARIO(layering_info.hrt_num, l_rule_info->disp_path);
	HRT_SET_SCALE_SCENARIO(layering_info.hrt_num, l_rule_info->scale_rate);
	HRT_SET_AEE_FLAG(layering_info.hrt_num, l_rule_info->dal_enable);
	HRT_SET_WROT_SRAM_FLAG(layering_info.hrt_num, l_rule_info->wrot_sram);
	HRT_SET_DC_FLAG(layering_info.hrt_num, adaptive_dc_request);
	dump_disp_info(&layering_info, DISP_DEBUG_LEVEL_INFO);

	mmprofile_log_ex(ddp_mmp_get_events()->hrt, MMPROFILE_FLAG_PULSE,
			 layering_info.hrt_num,
			 (layering_info.gles_head[0] << 24) |
			 (layering_info.gles_tail[0] << 16) |
			 (layering_info.layer_num[0] << 8) |
			 layering_info.layer_num[1]);

	ret = copy_layer_info_to_user(disp_info_user, debug_mode);
	return ret;
}

/**** UT Program ****/
#ifdef HRT_UT_DEBUG
static void debug_set_layer_data(struct disp_layer_info *disp_info,
	int disp_id, int data_type, int value)
{
	static int layer_id = -1;
	struct layer_config *layer_info = NULL;

	if (data_type != HRT_LAYER_DATA_ID && layer_id == -1)
		return;

	layer_info = &disp_info->input_config[disp_id][layer_id];
	switch (data_type) {
	case HRT_LAYER_DATA_ID:
		layer_id = value;
		break;
	case HRT_LAYER_DATA_SRC_FMT:
		layer_info->src_fmt = value;
		break;
	case HRT_LAYER_DATA_DST_OFFSET_X:
		layer_info->dst_offset_x = value;
		break;
	case HRT_LAYER_DATA_DST_OFFSET_Y:
		layer_info->dst_offset_y = value;
		break;
	case HRT_LAYER_DATA_DST_WIDTH:
		layer_info->dst_width = value;
		break;
	case HRT_LAYER_DATA_DST_HEIGHT:
		layer_info->dst_height = value;
		break;
	case HRT_LAYER_DATA_SRC_WIDTH:
		layer_info->src_width = value;
		break;
	case HRT_LAYER_DATA_SRC_HEIGHT:
		layer_info->src_height = value;
		break;
	case HRT_LAYER_DATA_SRC_OFFSET_X:
		layer_info->src_offset_x = value;
		break;
	case HRT_LAYER_DATA_SRC_OFFSET_Y:
		layer_info->src_offset_y = value;
		break;
	case HRT_LAYER_DATA_COMPRESS:
		layer_info->compress = value;
		break;
	case HRT_LAYER_DATA_CAPS:
		layer_info->layer_caps = value;
		break;
	default:
		break;
	}
}

static char *parse_hrt_data_value(char *start, long int *value)
{
	char *tok_start = NULL, *tok_end = NULL;
	int ret;

	tok_start = strchr(start + 1, ']');
	tok_end = strchr(tok_start + 1, '[');
	if (tok_end)
		*tok_end = 0;
	ret = kstrtol(tok_start + 1, 10, value);
	if (ret)
		DISP_PR_INFO("Parsing error gles_num:%d, p:%s, ret:%d\n",
			     (int)*value, tok_start + 1, ret);

	return tok_end;
}

static void print_layer_config(struct layer_config *c)
{
	DISPMSG("L%u/(%u,%u,%u,%u)/(%u,%u,%u,%u)/cpr%d/e%d/cap0x%x/cl%x\n",
		c->ovl_id,
		c->src_offset_x, c->src_offset_y,
		c->src_width, c->src_height,
		c->dst_offset_x, c->dst_offset_y,
		c->dst_width, c->dst_height,
		c->compress, c->ext_sel_layer,
		c->layer_caps, c->clip);
}

static void print_hrt_result(struct disp_layer_info *disp_info)
{
	unsigned int i = 0, j = 0;

	for (i = 0; i < 2; i++) {
		DISPMSG("### DISP%d ###\n", i);
		DISPMSG("[head]%d[tail]%d\n",
			disp_info->gles_head[i],
			disp_info->gles_tail[i]);
		DISPMSG("[hrt_num]%d\n", disp_info->hrt_num);
		for (j = 0; j < disp_info->layer_num[i]; j++)
			print_layer_config(&(disp_info->input_config[i][j]));
	}
}

static int load_hrt_test_data(struct disp_layer_info *disp_info)
{
	char filename[] = "/sdcard/hrt_data.txt";
	char line_buf[512];
	char *tok;
	struct file *filp;
	mm_segment_t oldfs;
	int ret, pos, i;
	long int disp_id, test_case;
	bool is_end = false, is_test_pass = false;
	struct layer_config *input_config;

	pos = 0;
	test_case = -1;
	oldfs = get_fs();
	set_fs(KERNEL_DS);
	filp = filp_open(filename, O_RDONLY, 0777);
	if (IS_ERR(filp)) {
		DISP_PR_INFO("File open error:%s\n", filename);
		return -1;
	}

	if (!filp->f_op) {
		DISP_PR_INFO("File Operation Method Error!!\n");
		return -1;
	}

	while (1) {
		ret = filp->f_op->llseek(filp, filp->f_pos, pos);
		memset(line_buf, 0x0, sizeof(line_buf));
		ret = filp->f_op->read(filp, line_buf, sizeof(line_buf),
			&filp->f_pos);
		tok = strchr(line_buf, '\n');
		if (tok != NULL)
			*tok = '\0';
		else
			is_end = true;

		pos += strlen(line_buf) + 1;
		filp->f_pos = pos;

		if (strncmp(line_buf, "#", 1) == 0) {
			continue;
		} else if (strncmp(line_buf, "[layer_num]", 11) == 0) {
			unsigned long int layer_num = 0;
			unsigned long int layer_size = 0;

			tok = parse_hrt_data_value(line_buf, &layer_num);
			if (!tok)
				goto end;
			tok = parse_hrt_data_value(tok, &disp_id);

			if (disp_id > HRT_SECONDARY)
				goto end;

			if (layer_num != 0) {
				layer_size =
					sizeof(struct layer_config) * layer_num;
				disp_info->input_config[disp_id] =
					kzalloc(layer_size,	GFP_KERNEL);
			}
			disp_info->layer_num[disp_id] = layer_num;

			if (disp_info->input_config[disp_id] == NULL)
				return 0;
		} else if (strncmp(line_buf, "[set_layer]", 11) == 0) {
			unsigned long int tmp_info;

			tok = strchr(line_buf, ']');
			if (!tok)
				goto end;
			tok = parse_hrt_data_value(tok, &disp_id);
			for (i = 0; i < HRT_LAYER_DATA_NUM; i++) {
				tok = parse_hrt_data_value(tok, &tmp_info);
				debug_set_layer_data(disp_info, disp_id,
					i, tmp_info);
			}
		} else if (strncmp(line_buf, "[test_start]", 12) == 0) {
			tok = parse_hrt_data_value(line_buf, &test_case);
			layering_rule_start(disp_info, 1);
			is_test_pass = true;
		} else if (strncmp(line_buf, "[test_end]", 10) == 0) {
			vfree(disp_info->input_config[0]);
			vfree(disp_info->input_config[1]);
			memset(disp_info, 0x0, sizeof(struct disp_layer_info));
			is_end = true;
		} else if (strncmp(line_buf,
				"[print_out_test_result]", 23) == 0) {
			DISP_PR_INFO("Test case %d is %s\n", (int)test_case,
				     is_test_pass?"Pass":"Fail");
		} else if (strncmp(line_buf, "[layer_result]", 14) == 0) {
			long int layer_result = 0, layer_id;

			tok = strchr(line_buf, ']');
			if (!tok)
				goto end;
			tok = parse_hrt_data_value(tok, &disp_id);
			if (!tok)
				goto end;
			tok = parse_hrt_data_value(tok, &layer_id);
			if (!tok)
				goto end;
			tok = parse_hrt_data_value(tok, &layer_result);
			input_config =
				&disp_info->input_config[disp_id][layer_id];
			if (layer_result !=	input_config->ovl_id) {
				DISP_PR_INFO("case:%d,ovl_id incorrect,%d/%d\n",
					     (int)test_case,
					     input_config->ovl_id,
					     (int)layer_result);
				is_test_pass = false;
			}
			if (!tok)
				goto end;
			tok = parse_hrt_data_value(tok, &layer_result);
			if (layer_result != input_config->ext_sel_layer) {
				DISP_PR_INFO("case:%d,ext_sel_layer wrong,%d/%d\n",
					     (int)test_case,
					     input_config->ext_sel_layer,
					     (int)layer_result);
				is_test_pass = false;
			}
		} else if (strncmp(line_buf, "[gles_result]", 13) == 0) {
			long int gles_num = 0;

			tok = strchr(line_buf, ']');
			if (!tok)
				goto end;
			tok = parse_hrt_data_value(tok, &disp_id);
			if (!tok)
				goto end;
			tok = parse_hrt_data_value(tok, &gles_num);
			if (gles_num != disp_info->gles_head[disp_id]) {
				DISP_PR_INFO("case:%d,gles head err,%d/%d\n",
					     (int)test_case,
					     disp_info->gles_head[disp_id],
					     (int)gles_num);
				is_test_pass = false;
			}

			if (!tok)
				goto end;
			tok = parse_hrt_data_value(tok, &gles_num);
			if (gles_num != disp_info->gles_tail[disp_id]) {
				DISP_PR_INFO("case:%d,gles tail err,%d/%d\n",
					     (int)test_case,
					     disp_info->gles_tail[disp_id],
					     (int)gles_num);
				is_test_pass = false;
			}
		} else if (strncmp(line_buf, "[hrt_result]", 12) == 0) {
			unsigned long int hrt_num = 0;
			int path_scen;

			tok = parse_hrt_data_value(line_buf, &hrt_num);
			if (hrt_num != HRT_GET_DVFS_LEVEL(disp_info->hrt_num))
				DISP_PR_INFO("case:%d,hrt num err,%d/%d\n",
					     (int)test_case,
					     HRT_GET_DVFS_LEVEL(
							disp_info->hrt_num),
					     (int)hrt_num);

			if (!tok)
				goto end;
			tok = parse_hrt_data_value(tok, &hrt_num);
			path_scen =
				HRT_GET_PATH_SCENARIO(disp_info->hrt_num) &
				0x1F;
			if (hrt_num != path_scen) {
				DISP_PR_INFO("case:%d,hrt path err,%d/%d\n",
					     (int)test_case, path_scen,
					     (int)hrt_num);
				is_test_pass = false;
			}

			if (!tok)
				goto end;
			tok = parse_hrt_data_value(tok, &hrt_num);
			if (hrt_num !=
				HRT_GET_SCALE_SCENARIO(disp_info->hrt_num)) {
				DISP_PR_INFO("case:%d, hrt scale err,%d/%d\n",
					     (int)test_case,
					     HRT_GET_SCALE_SCENARIO(
							disp_info->hrt_num),
					     (int)hrt_num);
				is_test_pass = false;
			}

		} else if (strncmp(line_buf,
		"[change_layer_num]", 18) == 0) {
			unsigned long int layer_num = 0;

			tok = parse_hrt_data_value(line_buf, &layer_num);
			if (!tok)
				goto end;
			tok = parse_hrt_data_value(tok, &disp_id);
			disp_info->layer_num[disp_id] = layer_num;
		} else if (!strncmp(line_buf, "[force_dual_pipe_off]", 21)) {
			unsigned long int force_off = 0;

			tok = parse_hrt_data_value(line_buf, &force_off);
			set_hrt_state(DISP_HRT_FORCE_DUAL_OFF, force_off);
		} else if (!strncmp(line_buf, "[resolution_level]", 18)) {
			unsigned long int resolution_level = 0;

			tok = parse_hrt_data_value(line_buf, &resolution_level);
			debug_resolution_level = resolution_level;
		} else if (!strncmp(line_buf, "[set_gles]", 10)) {
			long int gles_num = 0;

			tok = strchr(line_buf, ']');
			if (!tok)
				goto end;
			tok = parse_hrt_data_value(tok, &disp_id);
			if (!tok)
				goto end;
			tok = parse_hrt_data_value(tok, &gles_num);
			disp_info->gles_head[disp_id] = gles_num;

			if (!tok)
				goto end;
			tok = parse_hrt_data_value(tok, &gles_num);
			disp_info->gles_tail[disp_id] = gles_num;
		} else if (!strncmp(line_buf, "[disp_mode]", 11)) {
			unsigned long int disp_mode = 0;

			tok = parse_hrt_data_value(line_buf, &disp_mode);
			if (!tok)
				goto end;
			tok = parse_hrt_data_value(tok, &disp_id);
			disp_info->disp_mode[disp_id] = disp_mode;
		} else if (!strncmp(line_buf, "[print_out_hrt_result]", 22))
			print_hrt_result(disp_info);

		if (is_end)
			break;
	}

end:
	filp_close(filp, NULL);
	set_fs(oldfs);
	DISPINFO("end set_fs\n");
	return 0;
}
#endif

int gen_hrt_pattern(void)
{
#ifdef HRT_UT_DEBUG
	struct disp_layer_info disp_info;
	struct layer_config *layer_info;
	int i;

	memset(&disp_info, 0x0, sizeof(struct disp_layer_info));
	disp_info.gles_head[0] = -1;
	disp_info.gles_head[1] = -1;
	disp_info.gles_tail[0] = -1;
	disp_info.gles_tail[1] = -1;
	if (!load_hrt_test_data(&disp_info))
		return 0;

	/* Primary Display */
	disp_info.disp_mode[0] = DISP_SESSION_DIRECT_LINK_MODE;
	disp_info.layer_num[0] = 5;
	disp_info.gles_head[0] = 3;
	disp_info.gles_tail[0] = 5;
	disp_info.input_config[0] = kzalloc(sizeof(struct layer_config) * 5,
					    GFP_KERNEL);
	layer_info = disp_info.input_config[0];
	for (i = 0; i < disp_info.layer_num[0]; i++)
		layer_info[i].src_fmt = DISP_FORMAT_ARGB8888;

	layer_info = disp_info.input_config[0];
	layer_info[0].dst_offset_x = 0;
	layer_info[0].dst_offset_y = 0;
	layer_info[0].dst_width = 1080;
	layer_info[0].dst_height = 1920;
	layer_info[1].dst_offset_x = 0;
	layer_info[1].dst_offset_y = 0;
	layer_info[1].dst_width = 1080;
	layer_info[1].dst_height = 1920;
	layer_info[2].dst_offset_x = 269;
	layer_info[2].dst_offset_y = 72;
	layer_info[2].dst_width = 657;
	layer_info[2].dst_height = 612;
	layer_info[3].dst_offset_x = 0;
	layer_info[3].dst_offset_y = 0;
	layer_info[3].dst_width = 1080;
	layer_info[3].dst_height = 72;
	layer_info[4].dst_offset_x = 1079;
	layer_info[4].dst_offset_y = 72;
	layer_info[4].dst_width = 1;
	layer_info[4].dst_height = 1704;

	/* Secondary Display */
	disp_info.disp_mode[1] = DISP_SESSION_DIRECT_LINK_MODE;
	disp_info.layer_num[1] = 0;
	disp_info.gles_head[1] = -1;
	disp_info.gles_tail[1] = -1;

	DISPMSG("free test pattern\n");
	kfree(disp_info.input_config[0]);
	msleep(50);
#endif
	return 0;
}
/**** UT Program end ****/

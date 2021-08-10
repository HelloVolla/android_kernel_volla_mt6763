/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: PC Chen <pc.chen@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _VCODEC_IPI_MSG_H_
#define _VCODEC_IPI_MSG_H_

#include <linux/videodev2.h>
#include <linux/v4l2-controls.h>

enum mtk_venc_hw_id {
	MTK_VENC_CORE_0 = 0,
	MTK_VENC_CORE_1 = 1,
	MTK_VENC_HW_NUM = 2,
};

enum mtk_vdec_hw_id {
	MTK_VDEC_CORE = 0,
	MTK_VDEC_LAT = 1,
	MTK_VDEC_HW_NUM = 2,
};

enum mtk_fmt_type {
	MTK_FMT_DEC = 0,
	MTK_FMT_ENC = 1,
	MTK_FMT_FRAME = 2,
};

/**
 * struct mtk_video_fmt - Structure used to store information about pixelformats
 */
struct mtk_video_fmt {
	__u32	fourcc;
	__u32	type;   /* enum mtk_fmt_type */
	__u32	num_planes;
};

/**
 * struct mtk_codec_framesizes - Structure used to store information about
 *							framesizes
 */
struct mtk_codec_framesizes {
	__u32	fourcc;
	__u32	profile;
	__u32	level;
	struct	v4l2_frmsize_stepwise	stepwise;
};


struct mtk_color_desc {
	__u32	color_primaries;
	__u32	transform_character;
	__u32	matrix_coeffs;
	__u32	display_primaries_x[3];
	__u32	display_primaries_y[3];
	__u32	white_point_x;
	__u32	white_point_y;
	__u32	max_display_mastering_luminance;
	__u32	min_display_mastering_luminance;
	__u32	max_content_light_level;
	__u32	max_pic_light_level;
	__u32	is_hdr;
	__u32	full_range;
};

/**
 * struct vdec_pic_info  - picture size information
 * @pic_w: picture width
 * @pic_h: picture height
 * @buf_w   : picture buffer width (codec aligned up from pic_w)
 * @buf_h   : picture buffer heiht (codec aligned up from pic_h)
 * @fb_sz: frame buffer size
 * @bitdepth: Sequence bitdepth
 * @layout_mode: mediatek frame layout mode
 * @fourcc: frame buffer color format
 * E.g. suppose picture size is 176x144,
 *      buffer size will be aligned to 176x160.
 */
struct vdec_pic_info {
	__u32 pic_w;
	__u32 pic_h;
	__u32 buf_w;
	__u32 buf_h;
	__u32 fb_sz[VIDEO_MAX_PLANES];
	__u32 bitdepth;
	__u32 layout_mode;
	__u32 fourcc;
};

/**
 * struct vdec_dec_info - decode information
 * @dpb_sz		: decoding picture buffer size
 * @vdec_changed_info  : some changed flags
 * @bs_dma		: Input bit-stream buffer dma address
 * @bs_fd               : Input bit-stream buffer dmabuf fd
 * @fb_dma		: Y frame buffer dma address
 * @fb_fd             : Y frame buffer dmabuf fd
 * @vdec_bs_va		: VDEC bitstream buffer struct virtual address
 * @vdec_fb_va		: VDEC frame buffer struct virtual address
 * @fb_num_planes	: frame buffer plane count
 * @reserved		: reserved variable for 64bit align
 */
struct vdec_dec_info {
	__u32 dpb_sz;
	__u32 vdec_changed_info;
	__u64 bs_dma;
	__u64 bs_fd;
	__u64 fb_dma[VIDEO_MAX_PLANES];
	__u64 fb_fd[VIDEO_MAX_PLANES];
	__u64 vdec_bs_va;
	__u64 vdec_fb_va;
	__u32 fb_num_planes;
	__u32 index;
	__u32 wait_key_frame;
	__u32 error_map;
	__u32 timestamp;
	__u32 queued_frame_buf_count;
};

#endif

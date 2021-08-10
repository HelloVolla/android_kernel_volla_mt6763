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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */

/******************************************************************************
 *
 *
 * Filename:
 * ---------
 *    mtk-soc-pcm-common
 *
 * Project:
 * --------
 *     mtk-soc-pcm-common function
 *
 *
 * Description:
 * ------------
 *   common function
 *
 * Author:
 * -------
 *   Chipeng Chang (MTK02308)
 *
 *---------------------------------------------------------------------------
---
 *

****************************************************************************/

#include "mtk-soc-pcm-common.h"

unsigned long audio_frame_to_bytes(struct snd_pcm_substream *substream,
				   unsigned long count)
{
	unsigned long bytes = count;
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (runtime->format == SNDRV_PCM_FORMAT_S32_LE ||
	    runtime->format == SNDRV_PCM_FORMAT_U32_LE)
		bytes = bytes << 2;
	else
		bytes = bytes << 1;

	if (runtime->channels == 2)
		bytes = bytes << 1;
	else if (runtime->channels == 4)
		bytes = bytes << 2;
	else if (runtime->channels != 1)
		bytes = bytes << 3;

	return bytes;
}

unsigned long audio_bytes_to_frame(struct snd_pcm_substream *substream,
				   unsigned long bytes)
{
	unsigned long count = bytes;
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (runtime->format == SNDRV_PCM_FORMAT_S32_LE ||
	    runtime->format == SNDRV_PCM_FORMAT_U32_LE)
		count = count >> 2;
	else
		count = count >> 1;

	if (runtime->channels == 2)
		count = count >> 1;
	else if (runtime->channels == 4)
		count = count >> 2;
	else if (runtime->channels != 1)
		count = count >> 3;

	return count;
}

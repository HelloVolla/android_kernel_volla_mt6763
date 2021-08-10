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

#ifndef AUDIO_IPI_CLIENT_PLAYBACK_H
#define AUDIO_IPI_CLIENT_PLAYBACK_H

#include <linux/fs.h>           /* needed by file_operations* */
#include "audio_messenger_ipi.h"

void audio_ipi_client_playback_init(void);
void audio_ipi_client_playback_deinit(void);
void playback_dump_message(struct ipi_msg_t *ipi_msg);
void playback_open_dump_file(void);
void playback_close_dump_file(void);


#endif /* end of AUDIO_IPI_CLIENT_PLAYBACK_H */


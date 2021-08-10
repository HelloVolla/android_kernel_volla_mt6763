/*
 * Copyright (c) 2015-2017 MICROTRUST Incorporated
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/slab.h>
#include "teei_keymaster.h"
#include "teei_client_transfer_data.h"
#define IMSG_TAG "[tz_driver]"
#include <imsg_log.h>
#include <linux/vmalloc.h>

#define KM_COMMAND_MAGIC 'X'
unsigned long keymaster_buff_addr;

unsigned long create_keymaster_fdrv(int buff_size)
{
	unsigned long addr = 0;

	if (buff_size < 1) {
		IMSG_ERROR("Wrong buffer size %d:", buff_size);
		return 0;
	}
	addr = (unsigned long) vmalloc(buff_size);
	if (addr == 0) {
		IMSG_ERROR("kmalloc buffer failed");
		return 0;
	}
	memset((void *)addr, 0, buff_size);
	return addr;
}

int send_keymaster_command(void *buffer, unsigned long size)
{
	int ret = 0;
	struct TEEC_Context context;
	struct TEEC_UUID uuid_ta = { 0xc09c9c5d, 0xaa50, 0x4b78,
	{ 0xb0, 0xe4, 0x6e, 0xda, 0x61, 0x55, 0x6c, 0x3a } };

	if (buffer == NULL || size < 1)
		return -1;

	memset(&context, 0, sizeof(context));
	ret = ut_pf_gp_initialize_context(&context);
	if (ret) {
		IMSG_ERROR("Failed to initialize keymaster context ,err: %x",
		ret);
		goto release_1;
	}
	ret = ut_pf_gp_transfer_data(&context, &uuid_ta, KM_COMMAND_MAGIC,
	buffer, size);
	if (ret) {
		IMSG_ERROR("Failed to transfer data,err: %x", ret);
		goto release_2;
	}
release_2:
	ut_pf_gp_finalize_context(&context);
release_1:
	return ret;
}

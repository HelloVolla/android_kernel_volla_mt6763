/*
 * Copyright (C) 2018 MediaTek Inc.
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

#ifndef TRUSTED_MEM_API_H
#define TRUSTED_MEM_API_H

enum TRUSTED_MEM_REQ_TYPE {
	TRUSTED_MEM_REQ_SVP = 0,
	TRUSTED_MEM_REQ_PROT = 1,
	TRUSTED_MEM_REQ_WFD = 2,
	TRUSTED_MEM_REQ_HAPP = 3,
	TRUSTED_MEM_REQ_HAPP_EXTRA = 4,
	TRUSTED_MEM_REQ_SDSP = 5,
	TRUSTED_MEM_REQ_SDSP_SHARED = 6,
	TRUSTED_MEM_REQ_2D_FR = 7,
};

/**********************************************************/
/**** Trusted Memory Common APIs for ION kernel driver ****/
/**********************************************************/
#ifdef CONFIG_MTK_TRUSTED_MEMORY_SUBSYSTEM
int trusted_mem_api_alloc(enum TRUSTED_MEM_REQ_TYPE mem_type, u32 alignment,
			  u32 size, u32 *refcount, u32 *sec_handle,
			  uint8_t *owner, uint32_t id);
int trusted_mem_api_alloc_zero(enum TRUSTED_MEM_REQ_TYPE mem_type,
			       u32 alignment, u32 size, u32 *refcount,
			       u32 *sec_handle, uint8_t *owner, uint32_t id);
int trusted_mem_api_unref(enum TRUSTED_MEM_REQ_TYPE mem_type, u32 sec_handle,
			  uint8_t *owner, uint32_t id);
bool trusted_mem_api_get_region_info(enum TRUSTED_MEM_REQ_TYPE mem_type,
				     u64 *pa, u32 *size);
#endif

#endif /* end of TRUSTED_MEM_API_H */

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

#ifndef __BWL_PLATFORM_H__
#define __BWL_PLATFORM_H__

enum BWL_ENV {
	BWL_ENV_LPDDR4_2CH,
	BWL_ENV_MAX
};

enum CON_SCE {
	CON_SCE_ICFP,
	CON_SCE_VPWFD,
	CON_SCE_VR4K,
	CON_SCE_UI,
	BWL_SCN_MAX
};

enum BWL_CEN_REG {
	BWL_CEN_0x00000020,
	BWL_CEN_0x00000060,
	BWL_CEN_0x00000100,
	BWL_CEN_0x00000108,
	BWL_CEN_0x00000110,
	BWL_CEN_0x00000118,
	BWL_CEN_0x00000120,
	BWL_CEN_0x00000128,
	BWL_CEN_0x00000130,
	BWL_CEN_0x00000138,
	BWL_CEN_MAX
};

enum BWL_CHN_REG {
	BWL_CHN_0x00000080,
	BWL_CHN_0x00000088,
	BWL_CHN_0x00000158,
	BWL_CHN_0x0000015c,
	BWL_CHN_0x000001b8,
	BWL_CHN_MAX
};

#define SCN_DEFAULT	CON_SCE_UI

#endif /* __BWL_PLATFORM_H__ */

/*
 * Copyright (C) 2017 MediaTek Inc.
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

#ifndef _MTK_CLK_BUF_CTL_H_
#define _MTK_CLK_BUF_CTL_H_

#if defined(CONFIG_MACH_MT6735) || defined(CONFIG_MACH_MT6735M) ||\
	defined(CONFIG_MACH_MT6753)

#include "../mt6735/mtk_clkbuf_ctl.h"

#elif defined(CONFIG_MACH_MT6755)

#include "../mt6755/mtk_clkbuf_ctl.h"

#elif defined(CONFIG_MACH_MT6757)

#include "../mt6757/mtk_clkbuf_ctl.h"

#elif defined(CONFIG_MACH_MT6797)

#include "../mt6797/mtk_clkbuf_ctl.h"

#elif defined(CONFIG_MACH_MT6580)

#include "../../../include/mt-plat/mt6580/include/mach/mtk_clkbuf_ctl.h"

#elif defined(CONFIG_MACH_KIBOPLUS)

#include "../kiboplus/mtk_clkbuf_ctl.h"

#elif defined(CONFIG_MACH_ELBRUS) || defined(CONFIG_MACH_MT6799) || \
	defined(CONFIG_MACH_MT6758) || defined(CONFIG_MACH_MT6759) || \
	defined(CONFIG_MACH_MT6763) || defined(CONFIG_MACH_MT6739) || \
	defined(CONFIG_MACH_MT6775) || defined(CONFIG_MACH_MT6765) || \
	defined(CONFIG_MACH_MT6761) || defined(CONFIG_MACH_MT3967) || \
	defined(CONFIG_MACH_MT6779)

#include "clkbuf_v1/mtk_clkbuf_ctl.h"

#else

#error NO corresponding project of mtk_clkbuf_ctl.h header file can be found!!!

#endif

#endif /* _MTK_CLK_BUF_CTL_H_ */


/*
 * Copyright (C) 2011-2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>       /* needed by all modules */
#include <mt-plat/sync_write.h>
#include "sspm_define.h"

/* debug API */
__weak void dump_emi_outstanding(void) {}
__weak void mtk_spm_dump_debug_info(void) {}
__weak void usb_dump_debug_register(void) {}

/* platform callback when ipi timeout */
void sspm_ipi_timeout_cb(int ipi_id)
{
	/* for debug EMI use */
	pr_info("%s: dump_emi_outstanding\n", __func__);
	dump_emi_outstanding();

	/* for debug SPM */
	pr_info("%s: mtk_spm_dump_debug_info\n", __func__);
	mtk_spm_dump_debug_info();

	/* for debug USB */
	pr_info("%s: usb_dump_debug_register\n", __func__);
	usb_dump_debug_register();
}

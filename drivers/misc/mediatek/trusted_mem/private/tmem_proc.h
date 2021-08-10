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

#ifndef TMEM_PROC_H
#define TMEM_PROC_H

#ifdef TCORE_UT_FWK_SUPPORT
int get_multithread_test_wait_completion_time(void);
int get_saturation_stress_test_rounds(void);
int get_saturation_stress_pmem_min_chunk_size(void);
#endif

#endif /* end of TMEM_PROC_H */

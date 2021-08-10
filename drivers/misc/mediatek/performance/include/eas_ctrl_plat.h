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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef _EAS_CTRL_PLAT_H_
#define _EAS_CTRL_PLAT_H_

/* control migration cost */
extern unsigned int sysctl_sched_migration_cost;
extern unsigned int sysctl_sched_sync_hint_enable;

#if defined(CONFIG_CPU_FREQ_GOV_SCHEDPLUS)
extern void schedplus_set_down_throttle_nsec(unsigned long val);
extern unsigned long schedplus_show_down_throttle_nsec(int cpu);
#endif

#endif /* _EAS_CTRL_PLAT_H_ */

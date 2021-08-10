/*
 * Copyright (C) 2017 MediaTek Inc.
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

#include <linux/cpumask.h>

#include <mtk_idle.h>
#include <mtk_mcdi.h>
#include <mtk_idle_mcdi.h>
#include <mtk_mcdi_state.h>
#include <mtk_mcdi_plat.h>

static int mcdi_idle_state_mapping[NR_TYPES] = {
	MCDI_STATE_DPIDLE,		/* IDLE_TYPE_DP */
	MCDI_STATE_SODI3,		/* IDLE_TYPE_SO3 */
	MCDI_STATE_SODI,		/* IDLE_TYPE_SO */
	MCDI_STATE_CLUSTER_OFF	/* IDLE_TYPE_RG */
};

int mcdi_state_table_idx_map[NF_CPU] = {
	MCDI_STATE_TABLE_SET_0,
	MCDI_STATE_TABLE_SET_0,
	MCDI_STATE_TABLE_SET_0,
	MCDI_STATE_TABLE_SET_0,
	MCDI_STATE_TABLE_SET_1,
	MCDI_STATE_TABLE_SET_1,
	MCDI_STATE_TABLE_SET_1,
	MCDI_STATE_TABLE_SET_2
};

static DECLARE_BITMAP(cpu_set_0_bits, CONFIG_NR_CPUS);
struct cpumask *cpu_set_0_mask = to_cpumask(cpu_set_0_bits);

static DECLARE_BITMAP(cpu_set_1_bits, CONFIG_NR_CPUS);
struct cpumask *cpu_set_1_mask = to_cpumask(cpu_set_1_bits);

static DECLARE_BITMAP(cpu_set_2_bits, CONFIG_NR_CPUS);
struct cpumask *cpu_set_2_mask = to_cpumask(cpu_set_2_bits);

static unsigned long default_set_0_mask = 0x000F;
static unsigned long default_set_1_mask = 0x0070;
static unsigned long default_set_2_mask = 0x0080;

static int mtk_rgidle_enter(struct cpuidle_device *dev,
			struct cpuidle_driver *drv, int index);
static int mtk_mcidle_enter(struct cpuidle_device *dev,
			struct cpuidle_driver *drv, int index);

static struct cpuidle_driver mtk_cpuidle_driver_set_0 = {
	.name             = "mtk_acao_cpuidle_set_0",
	.owner            = THIS_MODULE,
	.states[0] = {
		.enter			= mtk_rgidle_enter,
		.exit_latency		= 1,
		.target_residency	= 1,
		.name			= "rgidle",
		.desc			= "wfi"
	},
	.states[1] = {
		.enter			= mtk_mcidle_enter,
		.exit_latency		= 300,
		.target_residency	= 4500,
		.flags			= CPUIDLE_FLAG_TIMER_STOP,
		.name			= "mcdi",
		.desc			= "mcdi",
	},
	.state_count = 2,
	.safe_state_index = 0,
};

static struct cpuidle_driver mtk_cpuidle_driver_set_1 = {
	.name             = "mtk_acao_cpuidle_set_1",
	.owner            = THIS_MODULE,
	.states[0] = {
		.enter			= mtk_rgidle_enter,
		.exit_latency		= 1,
		.target_residency	= 1,
		.name			= "rgidle",
		.desc			= "wfi"
	},
	.states[1] = {
		.enter			= mtk_mcidle_enter,
		.exit_latency		= 300,
		.target_residency	= 1500,
		.flags			= CPUIDLE_FLAG_TIMER_STOP,
		.name			= "mcdi",
		.desc			= "mcdi",
	},
	.state_count = 2,
	.safe_state_index = 0,
};

static struct cpuidle_driver mtk_cpuidle_driver_set_2 = {
	.name             = "mtk_cpuidle_set_2",
	.owner            = THIS_MODULE,
	.states[0] = {
		.enter			= mtk_rgidle_enter,
		.exit_latency		= 1,
		.target_residency	= 1,
		.name			= "rgidle",
		.desc			= "wfi"
	},
	.states[1] = {
		.enter			= mtk_mcidle_enter,
		.exit_latency		= 300,
		.target_residency	= 850,
		.flags			= CPUIDLE_FLAG_TIMER_STOP,
		.name			= "mcdi",
		.desc			= "mcdi",
	},
	.state_count = 2,
	.safe_state_index = 0,
};

/*
 * Used for mcdi_governor
 * only use exit_latency & target_residency
 */
static struct cpuidle_driver
	mtk_acao_mcdi_state[NF_MCDI_STATE_TABLE_TYPE] = {
	[0] = {
		.name             = "mtk_acao_mcdi_set_0",
		.owner            = THIS_MODULE,
		.states[0] = {
			.enter              = NULL,
			.exit_latency       = 1,
			.target_residency   = 1,
			.name               = "wfi",
			.desc               = "wfi"
		},
		.states[1] = {
			.enter              = NULL,
			.exit_latency       = 300,
			.target_residency   = 4500,
			.name               = "cpu_off",
			.desc               = "cpu_off",
		},
		.states[2] = {
			.enter              = NULL,
			.exit_latency       = 600,
			.target_residency   = 4500,
			.name               = "cluster_off",
			.desc               = "cluster_off",
		},
		.states[3] = {
			.enter              = NULL,
			.exit_latency       = 1200,
			.target_residency   = 5000,
			.name               = "sodi",
			.desc               = "sodi",
		},
		.states[4] = {
			.enter              = NULL,
			.exit_latency       = 1200,
			.target_residency   = 5000,
			.name               = "dpidle",
			.desc               = "dpidle",
		},
		.states[5] = {
			.enter              = NULL,
			.exit_latency       = 5000,
			.target_residency   = 10500,
			.name               = "sodi3",
			.desc               = "sodi3",
		},
		.state_count = 6,
		.safe_state_index = 0,
	},
	[1] = {
		.name             = "mtk_acao_mcdi_set_1",
		.owner            = THIS_MODULE,
		.states[0] = {
			.enter              = NULL,
			.exit_latency       = 1,
			.target_residency   = 1,
			.name               = "wfi",
			.desc               = "wfi"
		},
		.states[1] = {
			.enter              = NULL,
			.exit_latency       = 300,
			.target_residency   = 1500,
			.name               = "cpu_off",
			.desc               = "cpu_off",
		},
		.states[2] = {
			.enter              = NULL,
			.exit_latency       = 600,
			.target_residency   = 1500,
			.name               = "cluster_off",
			.desc               = "cluster_off",
		},
		.states[3] = {
			.enter              = NULL,
			.exit_latency       = 1200,
			.target_residency   = 5000,
			.name               = "sodi",
			.desc               = "sodi",
		},
		.states[4] = {
			.enter              = NULL,
			.exit_latency       = 1200,
			.target_residency   = 5000,
			.name               = "dpidle",
			.desc               = "dpidle",
		},
		.states[5] = {
			.enter              = NULL,
			.exit_latency       = 5000,
			.target_residency   = 10500,
			.name               = "sodi3",
			.desc               = "sodi3",
		},
		.state_count = 6,
		.safe_state_index = 0,
	},
	[2] = {
		.name             = "mtk_acao_mcdi_set_2",
		.owner            = THIS_MODULE,
		.states[0] = {
			.enter              = NULL,
			.exit_latency       = 1,
			.target_residency   = 1,
			.name               = "wfi",
			.desc               = "wfi"
		},
		.states[1] = {
			.enter              = NULL,
			.exit_latency       = 300,
			.target_residency   = 850,
			.name               = "cpu_off",
			.desc               = "cpu_off",
		},
		.states[2] = {
			.enter              = NULL,
			.exit_latency       = 600,
			.target_residency   = 1500,
			.name               = "cluster_off",
			.desc               = "cluster_off",
		},
		.states[3] = {
			.enter              = NULL,
			.exit_latency       = 1200,
			.target_residency   = 5000,
			.name               = "sodi",
			.desc               = "sodi",
		},
		.states[4] = {
			.enter              = NULL,
			.exit_latency       = 1200,
			.target_residency   = 5000,
			.name               = "dpidle",
			.desc               = "dpidle",
		},
		.states[5] = {
			.enter              = NULL,
			.exit_latency       = 5000,
			.target_residency   = 10500,
			.name               = "sodi3",
			.desc               = "sodi3",
		},
		.state_count = 6,
		.safe_state_index = 0,
	}
};

#define invalid_type_and_state(type, state)                \
	(state <= MCDI_STATE_WFI || state >= NF_MCDI_STATE \
		|| type < 0 || type >= NF_CPU_TYPE)        \

#define __mcdi_set_state(type, i, member, val)                           \
do {                                                                     \
	mtk_acao_mcdi_state[type].states[i].member = val;                \
	if (i == MCDI_STATE_CPU_OFF) {                                   \
		if (cpu_type == CPU_TYPE_L)                              \
			mtk_cpuidle_driver_set_1.states[i].member = val; \
		else if (cpu_type == CPU_TYPE_LL)                        \
			mtk_cpuidle_driver_set_0.states[i].member = val; \
	}                                                                \
} while (0)

void mcdi_set_state_lat(int cpu_type, int state, unsigned int val)
{
	if (invalid_type_and_state(cpu_type, state))
		return;

	__mcdi_set_state(cpu_type, state, exit_latency, val);
}

void mcdi_set_state_res(int cpu_type, int state, unsigned int val)
{
	if (invalid_type_and_state(cpu_type, state))
		return;

	__mcdi_set_state(cpu_type, state, target_residency, val);
}

static int mtk_rgidle_enter(struct cpuidle_device *dev,
			      struct cpuidle_driver *drv, int index)
{
	wfi_enter(dev->cpu);
	return index;
}

static int mtk_mcidle_enter(struct cpuidle_device *dev,
			      struct cpuidle_driver *drv, int index)
{
	mcdi_enter(dev->cpu);
	return index;
}

int mcdi_get_mcdi_idle_state(int idx)
{
	int state = MCDI_STATE_CLUSTER_OFF;

	if (idx >= 0 && idx < NR_TYPES)
		state = mcdi_idle_state_mapping[idx];

	return state;
}

struct cpuidle_driver *mcdi_state_tbl_get(int cpu)
{
	int tbl_idx;

	tbl_idx = mcdi_state_table_idx_map[cpu];

	return &mtk_acao_mcdi_state[tbl_idx];
}

int mtk_cpuidle_register_driver(void)
{
	int ret;

	cpumask_clear(cpu_set_0_mask);
	cpumask_clear(cpu_set_1_mask);
	cpumask_clear(cpu_set_2_mask);

	cpu_set_0_mask->bits[0] = default_set_0_mask;
	cpu_set_1_mask->bits[0] = default_set_1_mask;
	cpu_set_2_mask->bits[0] = default_set_2_mask;

	mtk_cpuidle_driver_set_0.cpumask = cpu_set_0_mask;
	mtk_cpuidle_driver_set_1.cpumask = cpu_set_1_mask;
	mtk_cpuidle_driver_set_2.cpumask = cpu_set_2_mask;

	ret = cpuidle_register_driver(&mtk_cpuidle_driver_set_0);
	if (ret) {
		pr_info("Failed to register cpuidle driver 0\n");
		return ret;
	}

	ret = cpuidle_register_driver(&mtk_cpuidle_driver_set_1);
	if (ret) {
		pr_info("Failed to register cpuidle driver 1\n");
		return ret;
	}

	ret = cpuidle_register_driver(&mtk_cpuidle_driver_set_2);

	if (ret) {
		pr_info("Failed to register cpuidle driver 2\n");
		return ret;
	}

	return ret;
}

void mtk_cpuidle_unregister_driver(void)
{
	cpuidle_unregister_driver(&mtk_cpuidle_driver_set_0);
	cpuidle_unregister_driver(&mtk_cpuidle_driver_set_1);
	cpuidle_unregister_driver(&mtk_cpuidle_driver_set_2);
}


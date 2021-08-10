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

#include <linux/of.h>
#include <linux/of_address.h>

#include <mtk_idle_internal.h>
#include <ddp_pwm.h>

#include <mt-plat/mtk_secure_api.h>

#define IDLE_TAG     "Power/swap"

enum subsys_id {
	SYS_DIS = 0,
	SYS_MFG,
	SYS_ISP,
	SYS_VCODEC,
	NR_SYSS__,
};

/*
 * Variable Declarations
 */
void __iomem *infrasys_base;
void __iomem *mmsys_base;
void __iomem *imgsys_base;
void __iomem *mfgsys_base;
void __iomem *vencsys_base;

void __iomem *sleepsys_base;
void __iomem  *apmixed_base_in_idle;

/* Idle handler on/off */
int idle_switch[NR_TYPES] = {
	1,	/* dpidle switch */
	1,	/* soidle3 switch */
	1,	/* soidle switch */
	1,	/* rgidle switch */
};

unsigned int dpidle_blocking_stat[NR_GRPS][32];

unsigned int idle_condition_mask[NR_TYPES][NR_GRPS] = {
	/* dpidle_condition_mask */
	[IDLE_TYPE_DP] = {
		0x00040802,	/* INFRA0 */
		0x03AFB800,	/* INFRA1 */
		0x000000C5,	/* INFRA2 */
		0xFFFFFFFB,	/* MMSYS0 */
		0x000003FF,	/* MMSYS1 */
		0x00000312,	/* IMAGE,  use SPM MTCMOS off as condition */
		0x00000312,	/* MFG,    use SPM MTCMOS off as condition */
		0x00000F12,	/* VCODEC, use SPM MTCMOS off as condition */
	},
	/* soidle3_condition_mask */
	[IDLE_TYPE_SO3] = {
		0x02040802,	/* INFRA0 */
		0x03AFB800,	/* INFRA1 */
		0x000000D1,	/* INFRA2 */
		0xFFFFFFFB,	/* MMSYS0 */
		0x000003FF,	/* MMSYS1 */
		0x00000312,	/* IMAGE,  use SPM MTCMOS off as condition */
		0x00000312,	/* MFG,    use SPM MTCMOS off as condition */
		0x00000F12,	/* VCODEC, use SPM MTCMOS off as condition */
	},
	/* soidle_condition_mask */
	[IDLE_TYPE_SO] = {
		0x00040802,	/* INFRA0 */
		0x03AFB800,	/* INFRA1 */
		0x000000C1,	/* INFRA2 */
		0x000DFFE0,	/* MMSYS0 */
		0x00000170,	/* MMSYS1 */
		0x00000312,	/* IMAGE,  use SPM MTCMOS off as condition */
		0x00000312,	/* MFG,    use SPM MTCMOS off as condition */
		0x00000F12,	/* VCODEC, use SPM MTCMOS off as condition */
	},
	/* rgidle_condition_mask */
	[IDLE_TYPE_RG] = {
		0, 0, 0, 0, 0, 0, 0, 0},
};

unsigned int soidle3_pll_condition_mask[NR_PLLS] = {
	1, /* UNIVPLL */
	0, /* MFGPLL */
	1, /* MSDCPLL */
	0, /* TVDPLL */
	0, /* MMPLL */
};

static const char *idle_name[NR_TYPES] = {
	"dpidle",
	"soidle3",
	"soidle",
	"rgidle",
};

static const char *reason_name[NR_REASONS] = {
	"by_frm",
	"by_cpu",
	"by_srr",
	"by_ufs",
	"by_tee",
	"by_clk",
	"by_dcs",
	"by_dis",
	"by_pwm",
	"by_pll",
	"by_boot"
};

static const char *cg_group_name[NR_GRPS] = {
	"INFRA0",
	"INFRA1",
	"INFRA2",
	"MMSYS0",
	"MMSYS1",
	"IMAGE",
	"MFG",
	"VCODEC",
};

/*
 * Weak functions
 */
void __attribute__((weak)) msdc_clk_status(int *status)
{
	*status = 0;
}

bool __attribute__((weak)) disp_pwm_is_osc(void)
{
	return false;
}

/*
 * Function Definitions
 */
const char *mtk_get_idle_name(int id)
{
	if (id >= 0 && id < NR_TYPES)
		return idle_name[id];
	else
		return NULL;
}

const char *mtk_get_reason_name(int id)
{
	if (id >= 0 && id < NR_REASONS)
		return reason_name[id];
	else
		return NULL;
}

const char *mtk_get_cg_group_name(int id)
{
	if (id >= 0 && id < NR_GRPS)
		return cg_group_name[id];
	else
		return NULL;
}

static int sys_is_on(enum subsys_id id)
{
	u32 pwr_sta_mask[] = {
		DIS_PWR_STA_MASK,
		MFG_PWR_STA_MASK,
		ISP_PWR_STA_MASK,
		VEN_PWR_STA_MASK,
	};

#if 0
	if (id >= NR_SYSS__)
		/* BUG(); */
#endif

	u32 mask = pwr_sta_mask[id];
	u32 sta = idle_readl(SPM_PWR_STATUS);
	u32 sta_s = idle_readl(SPM_PWR_STATUS_2ND);

	return (sta & mask) && (sta_s & mask);
}

static int is_pll_on(int id)
{
	return idle_readl(APMIXEDSYS(0x230 + id * 0x10)) & 0x1;
}

static void get_all_clock_state(u32 clks[NR_GRPS])
{
	int i;

	for (i = 0; i < NR_GRPS; i++)
		clks[i] = 0;

	clks[CG_INFRA_0] = ~idle_readl(INFRA_SW_CG_0_STA);      /* INFRA0 */
	clks[CG_INFRA_1] = ~idle_readl(INFRA_SW_CG_1_STA);      /* INFRA1 */
	clks[CG_INFRA_2] = ~idle_readl(INFRA_SW_CG_2_STA);      /* INFRA2 */

	/* MTCMOS DIS and MM_PLL */
	if (sys_is_on(SYS_DIS) && is_pll_on(MM_PLL)) {
		clks[CG_MMSYS0] = (~idle_readl(DISP_CG_CON_0));
		clks[CG_MMSYS1] = (~idle_readl(DISP_CG_CON_1));
	}

	if (sys_is_on(SYS_ISP))
		clks[CG_IMAGE] =
				/* MTCMOS IMAGE */
				~idle_readl(SPM_ISP_PWR_CON);

	if (sys_is_on(SYS_MFG))
		clks[CG_MFG] = ~idle_readl(SPM_MFG_PWR_CON); /* MTCMOS MFG */

	if (sys_is_on(SYS_VCODEC))
		/* MTCMOS VCODEC */
		clks[CG_VCODEC] =
				~idle_readl(SPM_VEN_PWR_CON);

}

static inline void mtk_idle_check_cg_internal(
		unsigned int block_mask[NR_TYPES][NF_CG_STA_RECORD],
		int idle_type)
{
	int a, b;

	for (a = 0; a < NR_GRPS; a++) {
		for (b = 0; b < 32; b++) {
			if (block_mask[idle_type][a] & (1 << b))
				dpidle_blocking_stat[a][b] += 1;
		}
	}
}

bool mtk_idle_check_secure_cg(
		unsigned int block_mask[NR_TYPES][NF_CG_STA_RECORD])
{
	int ret = 0;
	int i;

	ret = mt_secure_call(MTK_SIP_KERNEL_CHECK_SECURE_CG, 0, 0, 0, 0);

	if (ret)
		for (i = 0; i < NR_TYPES; i++)
			if (idle_switch[i])
				block_mask[i][CG_INFRA_0]
					|= 0x08000000;

	return !ret;
}

bool mtk_idle_check_cg(unsigned int block_mask[NR_TYPES][NF_CG_STA_RECORD])
{
	bool ret = true;
	int i, j;
	unsigned int sta;
	u32 clks[NR_GRPS];

/* deprecated: msdc driver use resource request api instead */
#if 0
	msdc_clk_status(&sd_mask);
#endif

	get_all_clock_state(clks);

	sta = idle_readl(SPM_PWR_STATUS);

	for (i = 0; i < NR_TYPES; i++) {
		if (idle_switch[i]) {
/* deprecated: msdc driver use resource request instead */
#if 0
			/* SD status */
			if (sd_mask) {
				block_mask[i][CG_INFRA_0] |= sd_mask;
				block_mask[i][NR_GRPS] |= 0x1;
			}
#endif
			/* CG status */
			for (j = 0; j < NR_GRPS; j++) {
				block_mask[i][j] =
					idle_condition_mask[i][j] & clks[j];
				if (block_mask[i][j])
					block_mask[i][NR_GRPS] |= 0x2;
			}
			if (i == IDLE_TYPE_DP)
				mtk_idle_check_cg_internal(
						block_mask, IDLE_TYPE_DP);

			/* mtcmos */
			if (i == IDLE_TYPE_DP && !dpidle_by_pass_pg) {
				unsigned int flag =
					DIS_PWR_STA_MASK |
					MFG_PWR_STA_MASK |
					ISP_PWR_STA_MASK |
					VEN_PWR_STA_MASK;

				if (sta & flag) {
					block_mask[i][NR_GRPS + 0] |= 0x4;
					block_mask[i][NR_GRPS + 1]
						= (sta & flag);
				}
			}
			if ((i == IDLE_TYPE_SO || i == IDLE_TYPE_SO3)
					&& !soidle_by_pass_pg) {
				unsigned int flag =
					MFG_PWR_STA_MASK |
					ISP_PWR_STA_MASK |
					VEN_PWR_STA_MASK;

				if (sta & flag) {
					block_mask[i][NR_GRPS + 0] |= 0x4;
					block_mask[i][NR_GRPS + 1] =
							(sta & flag);
				}
			}
			if (block_mask[i][NR_GRPS])
				ret = false;
		}
	}

	return ret;
}

static const char *pll_name[NR_PLLS] = {
	"UNIVPLL",
	"MFGPLL",
	"MSDCPLL",
	"TVDPLL",
	"MMPLL",
};

const char *mtk_get_pll_group_name(int id)
{
	return pll_name[id];
}

bool mtk_idle_check_pll(unsigned int *condition_mask, unsigned int *block_mask)
{
	int i, j;

	memset(block_mask, 0, NR_PLLS * sizeof(unsigned int));

	for (i = 0; i < NR_PLLS; i++) {
		if (is_pll_on(i) & condition_mask[i]) {
			for (j = 0; j < NR_PLLS; j++)
				block_mask[j] =
						is_pll_on(j)
						& condition_mask[j];
			return false;
		}
	}

	return true;
}

#if 0
/* No need to get audio base */
static int __init get_base_from_matching_node(
		const struct of_device_id *ids, void __iomem **pbase,
		int idx, const char *cmp)
{
	struct device_node *node;

	node = of_find_matching_node(NULL, ids);
	if (!node) {
		pr_err("node '%s' not found!\n", cmp);
		/* TODO: BUG() */
	}

	*pbase = of_iomap(node, idx);
	if (!(*pbase)) {
		pr_err("node '%s' cannot iomap!\n", cmp);
		/* TODO: BUG() */
	}

	return 0;
}
#endif

static int __init get_base_from_node(
	const char *cmp, void __iomem **pbase, int idx)
{
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, cmp);

	if (!node) {
		pr_err("node '%s' not found!\n", cmp);
		/* TODO: BUG() */
	}

	*pbase = of_iomap(node, idx);
	if (!(*pbase)) {
		pr_err("node '%s' cannot iomap!\n", cmp);
		/* TODO: BUG() */
	}

	return 0;
}

void __init iomap_init(void)
{
	get_base_from_node("mediatek,infracfg_ao", &infrasys_base, 0);
	get_base_from_node("mediatek,mmsys_config", &mmsys_base, 0);
	get_base_from_node("mediatek,imgsys", &imgsys_base, 0);
	get_base_from_node("mediatek,mfgcfg", &mfgsys_base, 0);
	get_base_from_node("mediatek,venc_gcon", &vencsys_base, 0);
	get_base_from_node("mediatek,apmixed", &apmixed_base_in_idle, 0);
	get_base_from_node("mediatek,sleep", &sleepsys_base, 0);
}

bool mtk_idle_disp_is_pwm_rosc(void)
{
	return disp_pwm_is_osc();
}

u32 get_spm_idle_flags1(void)
{
	return 0;
}

bool mtk_idle_check_clkmux(
	int idle_type, unsigned int block_mask[NR_TYPES][NF_CLK_CFG])
{
	return true;
}

bool mtk_idle_check_vcore_cond(void)
{
	return true;
}

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

#ifndef __MTK_DCM_AUTOGEN_H__
#define __MTK_DCM_AUTOGEN_H__

#include <mtk_dcm.h>

#if defined(__KERNEL__) && defined(CONFIG_OF)
extern unsigned long dcm_infracfg_ao_base;
extern unsigned long dcm_mcucfg_base;
extern unsigned long dcm_mcucfg_phys_base;
#ifndef USE_DRAM_API_INSTEAD
extern unsigned long dcm_dramc0_ao_base;
extern unsigned long dcm_dramc1_ao_base;
extern unsigned long dcm_ddrphy0_ao_base;
extern unsigned long dcm_ddrphy1_ao_base;
#endif /* #ifndef USE_DRAM_API_INSTEAD */
extern unsigned long dcm_chn0_emi_base;
extern unsigned long dcm_chn1_emi_base;
extern unsigned long dcm_emi_base;

#define INFRACFG_AO_BASE	(dcm_infracfg_ao_base)
#define MCUCFG_BASE		(dcm_mcucfg_base)
#define MP0_CPUCFG_BASE		(MCUCFG_BASE)
#define MP1_CPUCFG_BASE		(MCUCFG_BASE + 0x200)
#define MCU_MISCCFG_BASE	(MCUCFG_BASE + 0x400)
#define MCU_MISC1CFG_BASE	(MCUCFG_BASE + 0x800)
#define DDRPHY0_AO_BASE	(dcm_ddrphy0_ao_base)
#define DRAMC0_AO_BASE	(dcm_dramc0_ao_base)
#define DDRPHY1_AO_BASE	(dcm_ddrphy1_ao_base)
#define DRAMC1_AO_BASE	(dcm_dramc1_ao_base)
#define CHN0_EMI_BASE	(dcm_chn0_emi_base)
#define CHN1_EMI_BASE	(dcm_chn1_emi_base)
#define EMI_BASE		(dcm_emi_base)
#else /* !(defined(__KERNEL__) && defined(CONFIG_OF)) */
#undef INFRACFG_AO_BASE
#undef EMI_BASE
#undef DDRPHY0_AO_BASE
#undef DRAMC0_AO_BASE
#undef DDRPHY1_AO_BASE
#undef DRAMC1_AO_BASE
#undef CHN0_EMI_BASE
#undef CHN1_EMI_BASE
#undef MP0_CPUCFG_BASE
#undef MP1_CPUCFG_BASE
#undef MCU_MISCCFG_BASE
#undef MCU_MISC1CFG_BASE
#undef MCUCFG_BASE

/* Base */
#define INFRACFG_AO_BASE 0x10001000
#define EMI_BASE 0x10219000
#define DDRPHY0_AO_BASE 0x10228000
#define DRAMC0_AO_BASE 0x1022a000
#define CHN0_EMI_BASE 0x1022d000
#define DDRPHY1_AO_BASE 0x10230000
#define DRAMC1_AO_BASE 0x10232000
#define CHN1_EMI_BASE 0x10235000
#define MP0_CPUCFG_BASE 0xc530000
#define MP1_CPUCFG_BASE 0xc530200
#define MCU_MISCCFG_BASE 0xc530400
#define MCU_MISC1CFG_BASE 0xc530800
#define MCUCFG_BASE MP0_CPUCFG_BASE
#endif /* #if defined(__KERNEL__) && defined(CONFIG_OF) */

/* Register Definition */
#define MP0_CPUCFG_MP0_RGU_DCM_CONFIG (MP0_CPUCFG_BASE + 0x88)
#define MP1_CPUCFG_MP1_RGU_DCM_CONFIG (MP1_CPUCFG_BASE + 0x88)
#define L2C_SRAM_CTRL (MCU_MISCCFG_BASE + 0x248)
#define CCI_CLK_CTRL (MCU_MISCCFG_BASE + 0x260)
#define BUS_FABRIC_DCM_CTRL (MCU_MISCCFG_BASE + 0x268)
#define MCU_MISC_DCM_CTRL (MCU_MISCCFG_BASE + 0x26c)
#define CCI_ADB400_DCM_CONFIG (MCU_MISCCFG_BASE + 0x340)
#define SYNC_DCM_CONFIG (MCU_MISCCFG_BASE + 0x344)
#define SYNC_DCM_CLUSTER_CONFIG (MCU_MISCCFG_BASE + 0x34c)
#define MP_GIC_RGU_SYNC_DCM (MCU_MISCCFG_BASE + 0x358)
#define MP0_PLL_DIVIDER_CFG (MCU_MISCCFG_BASE + 0x3a0)
#define MP1_PLL_DIVIDER_CFG (MCU_MISCCFG_BASE + 0x3a4)
#define BUS_PLL_DIVIDER_CFG (MCU_MISCCFG_BASE + 0x3c0)
#define MCSIA_DCM_EN (MCU_MISC1CFG_BASE + 0x360)
#define INFRA_BUS_DCM_CTRL (INFRACFG_AO_BASE + 0x70)
#define PERI_BUS_DCM_CTRL (INFRACFG_AO_BASE + 0x74)
#define MEM_DCM_CTRL (INFRACFG_AO_BASE + 0x78)
#define P2P_RX_CLK_ON (INFRACFG_AO_BASE + 0xa0)
#define EMI_CONM (EMI_BASE + 0x60)
#define EMI_CONN (EMI_BASE + 0x68)
#define DRAMC_CH0_TOP0_MISC_CG_CTRL0 (DDRPHY0_AO_BASE + 0x284)
#define DRAMC_CH0_TOP0_MISC_CG_CTRL2 (DDRPHY0_AO_BASE + 0x28c)
#define DRAMC_CH0_TOP0_MISC_CTRL3 (DDRPHY0_AO_BASE + 0x2a8)
#define DRAMC_CH0_TOP0_SHU1_B0_DQ8 (DDRPHY0_AO_BASE + 0xc20)
#define DRAMC_CH0_TOP0_SHU1_B1_DQ8 (DDRPHY0_AO_BASE + 0xca0)
#define DRAMC_CH0_TOP0_SHU1_CA_CMD8 (DDRPHY0_AO_BASE + 0xd20)
#define DRAMC_CH0_TOP0_SHU2_B0_DQ8 (DDRPHY0_AO_BASE + 0x1120)
#define DRAMC_CH0_TOP0_SHU2_B1_DQ8 (DDRPHY0_AO_BASE + 0x11a0)
#define DRAMC_CH0_TOP0_SHU2_CA_CMD8 (DDRPHY0_AO_BASE + 0x1220)
#define DRAMC_CH0_TOP0_SHU3_B0_DQ8 (DDRPHY0_AO_BASE + 0x1620)
#define DRAMC_CH0_TOP0_SHU3_B1_DQ8 (DDRPHY0_AO_BASE + 0x16a0)
#define DRAMC_CH0_TOP0_SHU3_CA_CMD8 (DDRPHY0_AO_BASE + 0x1720)
#define DRAMC_CH0_TOP0_SHU4_B0_DQ8 (DDRPHY0_AO_BASE + 0x1b20)
#define DRAMC_CH0_TOP0_SHU4_B1_DQ8 (DDRPHY0_AO_BASE + 0x1ba0)
#define DRAMC_CH0_TOP0_SHU4_CA_CMD8 (DDRPHY0_AO_BASE + 0x1c20)
#define DRAMC_CH0_TOP1_DRAMC_PD_CTRL (DRAMC0_AO_BASE + 0x38)
#define DRAMC_CH0_TOP1_CLKAR (DRAMC0_AO_BASE + 0x3c)
#define CHN0_EMI_CHN_EMI_CONB (CHN0_EMI_BASE + 0x8)
#define DRAMC_CH1_TOP0_MISC_CG_CTRL0 (DDRPHY1_AO_BASE + 0x284)
#define DRAMC_CH1_TOP0_MISC_CG_CTRL2 (DDRPHY1_AO_BASE + 0x28c)
#define DRAMC_CH1_TOP0_MISC_CTRL3 (DDRPHY1_AO_BASE + 0x2a8)
#define DRAMC_CH1_TOP0_SHU1_B0_DQ8 (DDRPHY1_AO_BASE + 0xc20)
#define DRAMC_CH1_TOP0_SHU1_B1_DQ8 (DDRPHY1_AO_BASE + 0xca0)
#define DRAMC_CH1_TOP0_SHU1_CA_CMD8 (DDRPHY1_AO_BASE + 0xd20)
#define DRAMC_CH1_TOP0_SHU2_B0_DQ8 (DDRPHY1_AO_BASE + 0x1120)
#define DRAMC_CH1_TOP0_SHU2_B1_DQ8 (DDRPHY1_AO_BASE + 0x11a0)
#define DRAMC_CH1_TOP0_SHU2_CA_CMD8 (DDRPHY1_AO_BASE + 0x1220)
#define DRAMC_CH1_TOP0_SHU3_B0_DQ8 (DDRPHY1_AO_BASE + 0x1620)
#define DRAMC_CH1_TOP0_SHU3_B1_DQ8 (DDRPHY1_AO_BASE + 0x16a0)
#define DRAMC_CH1_TOP0_SHU3_CA_CMD8 (DDRPHY1_AO_BASE + 0x1720)
#define DRAMC_CH1_TOP0_SHU4_B0_DQ8 (DDRPHY1_AO_BASE + 0x1b20)
#define DRAMC_CH1_TOP0_SHU4_B1_DQ8 (DDRPHY1_AO_BASE + 0x1ba0)
#define DRAMC_CH1_TOP0_SHU4_CA_CMD8 (DDRPHY1_AO_BASE + 0x1c20)
#define DRAMC_CH1_TOP1_DRAMC_PD_CTRL (DRAMC1_AO_BASE + 0x38)
#define DRAMC_CH1_TOP1_CLKAR (DRAMC1_AO_BASE + 0x3c)
#define CHN1_EMI_CHN_EMI_CONB (CHN1_EMI_BASE + 0x8)

/* INFRACFG_AO */
bool dcm_infracfg_ao_infra_bus_dcm_is_on(void);
void dcm_infracfg_ao_infra_bus_dcm(int on);
bool dcm_infracfg_ao_infra_emi_local_dcm_is_on(void);
void dcm_infracfg_ao_infra_emi_local_dcm(int on);
bool dcm_infracfg_ao_infra_rx_p2p_dcm_is_on(void);
void dcm_infracfg_ao_infra_rx_p2p_dcm(int on);
bool dcm_infracfg_ao_peri_bus_dcm_is_on(void);
void dcm_infracfg_ao_peri_bus_dcm(int on);
bool dcm_infracfg_ao_peri_module_dcm_is_on(void);
void dcm_infracfg_ao_peri_module_dcm(int on);
/* EMI */
bool dcm_emi_dcm_emi_group_is_on(void);
void dcm_emi_dcm_emi_group(int on);
/* DRAMC_CH0_TOP0 */
bool dcm_dramc_ch0_top0_ddrphy_is_on(void);
void dcm_dramc_ch0_top0_ddrphy(int on);
/* DRAMC_CH0_TOP1 */
bool dcm_dramc_ch0_top1_dramc_dcm_is_on(void);
void dcm_dramc_ch0_top1_dramc_dcm(int on);
/* CHN0_EMI */
bool dcm_chn0_emi_dcm_emi_group_is_on(void);
void dcm_chn0_emi_dcm_emi_group(int on);
/* DRAMC_CH1_TOP0 */
bool dcm_dramc_ch1_top0_ddrphy_is_on(void);
void dcm_dramc_ch1_top0_ddrphy(int on);
/* DRAMC_CH1_TOP1 */
bool dcm_dramc_ch1_top1_dramc_dcm_is_on(void);
void dcm_dramc_ch1_top1_dramc_dcm(int on);
/* CHN1_EMI */
bool dcm_chn1_emi_dcm_emi_group_is_on(void);
void dcm_chn1_emi_dcm_emi_group(int on);
/* MP0_CPUCFG */
bool dcm_mp0_cpucfg_mp0_rgu_dcm_is_on(void);
void dcm_mp0_cpucfg_mp0_rgu_dcm(int on);
/* MP1_CPUCFG */
bool dcm_mp1_cpucfg_mp1_rgu_dcm_is_on(void);
void dcm_mp1_cpucfg_mp1_rgu_dcm(int on);
/* MCU_MISCCFG */
bool dcm_mcu_misccfg_adb400_dcm_is_on(void);
void dcm_mcu_misccfg_adb400_dcm(int on);
bool dcm_mcu_misccfg_bus_arm_pll_divider_dcm_is_on(void);
void dcm_mcu_misccfg_bus_arm_pll_divider_dcm(int on);
bool dcm_mcu_misccfg_bus_sync_dcm_is_on(void);
void dcm_mcu_misccfg_bus_sync_dcm(int on);
bool dcm_mcu_misccfg_bus_clock_dcm_is_on(void);
void dcm_mcu_misccfg_bus_clock_dcm(int on);
bool dcm_mcu_misccfg_bus_fabric_dcm_is_on(void);
void dcm_mcu_misccfg_bus_fabric_dcm(int on);
bool dcm_mcu_misccfg_gic_sync_dcm_is_on(void);
void dcm_mcu_misccfg_gic_sync_dcm(int on);
bool dcm_mcu_misccfg_l2_shared_dcm_is_on(void);
void dcm_mcu_misccfg_l2_shared_dcm(int on);
bool dcm_mcu_misccfg_mp0_arm_pll_divider_dcm_is_on(void);
void dcm_mcu_misccfg_mp0_arm_pll_divider_dcm(int on);
bool dcm_mcu_misccfg_mp0_stall_dcm_is_on(void);
void dcm_mcu_misccfg_mp0_stall_dcm(int on);
bool dcm_mcu_misccfg_mp0_sync_dcm_enable_is_on(void);
void dcm_mcu_misccfg_mp0_sync_dcm_enable(int on);
bool dcm_mcu_misccfg_mp1_arm_pll_divider_dcm_is_on(void);
void dcm_mcu_misccfg_mp1_arm_pll_divider_dcm(int on);
bool dcm_mcu_misccfg_mp1_stall_dcm_is_on(void);
void dcm_mcu_misccfg_mp1_stall_dcm(int on);
bool dcm_mcu_misccfg_mp1_sync_dcm_enable_is_on(void);
void dcm_mcu_misccfg_mp1_sync_dcm_enable(int on);
bool dcm_mcu_misccfg_mp_stall_dcm_is_on(void);
void dcm_mcu_misccfg_mp_stall_dcm(int on);
bool dcm_mcu_misccfg_mcu_misc_dcm_is_on(void);
void dcm_mcu_misccfg_mcu_misc_dcm(int on);
/* MCU_MISC1CFG */
bool dcm_mcu_misc1cfg_mcsia_dcm_is_on(void);
void dcm_mcu_misc1cfg_mcsia_dcm(int on);

#endif

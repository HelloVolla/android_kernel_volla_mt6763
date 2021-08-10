/*
 * Copyright (C) 2018 MediaTek Inc.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef _UFS_MTK_H
#define _UFS_MTK_H

#define CONFIG_MTK_UFS_DEBUG
/* #define CONFIG_MTK_UFS_DEGUG_GPIO_TRIGGER */
#ifdef CONFIG_MTK_ENG_BUILD
/* workround:disable CONFIG_MTK_UFS_LBA_CRC16_CHECK for mt6763 */
#ifndef CONFIG_MACH_MT6763
#define CONFIG_MTK_UFS_LBA_CRC16_CHECK
#endif
#endif
/* #define CONFIG_MTK_UFS_LBA_CRC16_BUG_ON */

#include <linux/of.h>
#include <linux/rpmb.h>
#include <linux/hie.h>
#include "ufshcd.h"

#define UPIU_COMMAND_CRYPTO_EN_OFFSET	23

#define UFS_RPMB_DEV_MAX_RW_SIZE_LIMITATION (8)

struct ufs_crypto_map {
	unsigned char flag;
	unsigned char key;
};

#define UFS_MAX_LBA ((64 * 1024 * 1024) / 4)

#define UFS_CRYPTO_FLAG_READ          (0x01)
#define UFS_CRYPTO_FLAG_WRITE         (0x02)
#define UFS_CRYPTO_FLAG_UNMAP         (0x04)

#define UFS_CRYPTO_FLAG_NON_ENCRYPTED (0x10)
#define UFS_CRYPTO_FLAG_ENCRYPTED     (0x20)
#define UFS_CRYPTO_FLAG_VALID         (UFS_CRYPTO_FLAG_ENCRYPTED | \
	UFS_CRYPTO_FLAG_NON_ENCRYPTED)

#define UFS_HIE_PARAM_OFS_CFG_ID         (24)
#define UFS_HIE_PARAM_OFS_MODE           (16)
#define UFS_HIE_PARAM_OFS_KEY_TOTAL_BYTE (8)
#define UFS_HIE_PARAM_OFS_KEY_START_BYTE (0)

enum ufs_crypto_type {
	UFS_CRYPTO_NON_ENCRYPTED,
	UFS_CRYPTO_ENCRYPT,
	UFS_CRYPTO_DECRYPT,
};

enum ufs_trace_event {
	UFS_TRACE_SEND,
	UFS_TRACE_COMPLETED,
	UFS_TRACE_DEV_SEND,
	UFS_TRACE_DEV_COMPLETED,
	UFS_TRACE_TM_SEND,
	UFS_TRACE_TM_COMPLETED,
	UFS_TRACE_UIC_SEND,
	UFS_TRACE_UIC_CMPL_GENERAL,
	UFS_TRACE_UIC_CMPL_PWR_CTRL,
	UFS_TRACE_REG_TOGGLE,
	UFS_TRACE_ABORTING,
	UFS_TRACE_DI_FAIL,
	UFS_TRACE_DEVICE_RESET
};

enum {
	UNIPRO_CG_CFG_NATURE        = 0,    /* not force */
	UNIPRO_CG_CFG_FORCE_ENABLE  = 1,
	UNIPRO_CG_CFG_FORCE_DISABLE = 2,
};

enum {
	UFS_CRYPTO_ALGO_AES_XTS             = 0,
	UFS_CRYPTO_ALGO_BITLOCKER_AES_CBC   = 1,
	UFS_CRYPTO_ALGO_AES_ECB             = 2,
	UFS_CRYPTO_ALGO_ESSIV_AES_CBC       = 3,
};

enum {
	/* request resource for DMA operations, e.g., DRAM */
	UFS_MTK_RESREQ_DMA_OP,
	/* request resource for mphy not in H8, e.g., main PLL, 26 mhz clock */
	UFS_MTK_RESREQ_MPHY_NON_H8
};

enum {
	UFS_H8                      = 0x0,
	UFS_H8_SUSPEND              = 0x1,
};

struct ufs_cmd_str_struct {
	char str[32];
	char cmd;
};

#ifdef MTK_UFS_HQA
#define UFS_CACHED_REGION_CNT (3)
#else
#define UFS_CACHED_REGION_CNT (2)
#endif

struct ufs_cached_region {
	char *name;
	sector_t start_sect;
	sector_t end_sect;
};

/*
 * Hynix device need max 3 seconds to clear fDeviceInit,
 * each fDeviceInit transaction takes
 * around 1~2ms to get response from UFS.
 * Max fDeviceInit clear time = 5000*(1~2)ms > 3seconds
 */
#define UFS_FDEVICEINIT_RETRIES    (5000)


#define UFS_DESCRIPTOR_SIZE (255)

struct ufs_descriptor {
	u8 descriptor_idn;
	u8 index;
	u8 descriptor[UFS_DESCRIPTOR_SIZE];

	u8 *qresp_upiu;
	u32 qresp_upiu_size;
};

union ufs_cpt_cap {
	u32 cap_raw;
	struct {
		u8 cap_cnt;
		u8 cfg_cnt;
		u8 resv;
		u8 cfg_ptr;
	} cap;
};
union ufs_cpt_capx {
	u32 capx_raw;
	struct {
		u8 alg_id;
		u8 du_size;
		u8 key_size;
		u8 resv;
	} capx;
};
union ufs_cap_cfg {
	u32 cfgx_raw[32];
	struct {
		u32 key[16];
		u8 du_size;
		u8 cap_id;
		u16 resv0  : 15;
		u16 cfg_en : 1;
		u8 mu1ti_host;
		u8 resv1;
		u16 vsb;
		u32 resv2[14];
	} cfgx;
};
struct ufs_crypto {
	u32 cfg_id;
	u32 cap_id;
	union ufs_cpt_cap cap;
	union ufs_cpt_capx capx;
	union ufs_cap_cfg cfg;
};

struct ufs_crypt_info {
	struct ufs_hba *hba;
	struct scsi_cmnd *cmd;
};


extern u32 ufs_mtk_auto_hibern8_timer_ms;
extern bool ufs_mtk_auto_hibern8_enabled;
extern enum ufs_dbg_lvl_t ufs_mtk_dbg_lvl;
extern struct ufs_hba *ufs_mtk_hba;
extern bool ufs_mtk_host_deep_stall_enable;
extern bool ufs_mtk_host_scramble_enable;
extern bool ufs_mtk_tr_cn_used;

void ufs_mtk_add_sysfs_nodes(struct ufs_hba *hba);
int ufs_mtk_auto_hiber8_quirk_handler(struct ufs_hba *hba, bool enable);
void ufs_mtk_cache_setup_cmd(struct scsi_cmnd *cmd);
void ufs_mtk_crypto_cal_dun(u32 alg_id, u64 iv, u32 *dunl, u32 *dunu);
void ufs_mtk_dbg_dump_scsi_cmd(struct ufs_hba *hba,
	struct scsi_cmnd *cmd, u32 flag);
int ufs_mtk_deepidle_hibern8_check(void);
void ufs_mtk_deepidle_leave(void);
int ufs_mtk_generic_read_dme(u32 uic_cmd, u16 mib_attribute,
	u16 gen_select_index, u32 *value, unsigned long retry_ms);
void ufs_mtk_hwfde_cfg_cmd(struct ufs_hba *hba,
	struct scsi_cmnd *cmd);
int ufs_mtk_linkup_fail_handler(struct ufs_hba *hba, int left_retry);
void ufs_mtk_parse_auto_hibern8_timer(struct ufs_hba *hba);
void ufs_mtk_parse_pm_levels(struct ufs_hba *hba);
int ufs_mtk_perf_heurisic_if_allow_cmd(struct ufs_hba *hba,
	struct scsi_cmnd *cmd);
void ufs_mtk_perf_heurisic_req_done(struct ufs_hba *hba, struct scsi_cmnd *cmd);
int ufs_mtk_ioctl_ffu(struct scsi_device *dev, void __user *buf_user);
int ufs_mtk_ioctl_get_fw_ver(struct scsi_device *dev, void __user *buf_user);
int ufs_mtk_ioctl_query(struct ufs_hba *hba, u8 lun, void __user *buf_user);
int ufs_mtk_ioctl_rpmb(struct ufs_hba *hba, void __user *buf_user);
bool ufs_mtk_is_data_write_cmd(char cmd_op);
void ufs_mtk_rpmb_dump_frame(struct scsi_device *sdev, u8 *data_frame, u32 cnt);
struct rpmb_dev *ufs_mtk_rpmb_get_raw_dev(void);
void ufs_mtk_runtime_pm_init(struct scsi_device *sdev);
void ufs_mtk_device_quiesce(struct ufs_hba *hba);
void ufs_mtk_device_resume(struct ufs_hba *hba);

#ifdef CONFIG_MTK_UFS_LBA_CRC16_CHECK
void ufs_mtk_di_init(struct ufs_hba *hba);
int ufs_mtk_di_clr(struct scsi_cmnd *cmd);
int ufs_mtk_di_cmp(struct ufs_hba *hba, struct scsi_cmnd *cmd);
int ufs_mtk_di_inspect(struct ufs_hba *hba, struct scsi_cmnd *cmd);
#endif

#ifdef CONFIG_HIE
struct hie_dev *ufs_mtk_hie_get_dev(void);
int ufs_mtk_hie_req_done(struct ufs_hba *hba,
	struct ufshcd_lrb *lrbp);
#else
static inline
struct hie_dev *ufs_mtk_hie_get_dev(void)
{
	return NULL;
}

static inline int ufs_mtk_hie_req_done(
	struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
	return 0;
}
#endif

#endif /* !_UFS_MTK_H */


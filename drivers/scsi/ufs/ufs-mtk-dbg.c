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

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "["KBUILD_MODNAME"]" fmt

#include <linux/atomic.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <mt-plat/mtk_gpt.h>
#include <linux/io.h>
#include <linux/scatterlist.h>

#include "ufs-mtk-dbg.h"
#include "ufs.h"
#include <mt-plat/mtk_boot.h>


#ifndef FPGA_PLATFORM
#ifdef UPMU_READY
#include <mt-plat/upmu_common.h>
#endif
#endif

#ifdef CONFIG_MTK_UFS_DEBUG
#define MAX_UFS_CMD_HLIST_ENTRY_CNT (500)
/* max dump size is 40KB whitch can be adjusted */
#define UFS_AEE_BUFFER_SIZE (100 * 1024)

struct ufs_cmd_hlist_struct ufs_cmd_hlist[MAX_UFS_CMD_HLIST_ENTRY_CNT];
int ufs_cmd_ptr = MAX_UFS_CMD_HLIST_ENTRY_CNT - 1;
int ufs_cmd_cnt;
static spinlock_t ufs_mtk_cmd_dump_lock;
static int ufs_mtk_is_cmd_dump_lock_init;
static atomic_t cmd_hist_enabled;
char ufs_aee_buffer[UFS_AEE_BUFFER_SIZE];

static void ufs_mtk_dbg_dump_feature(struct ufs_hba *hba, struct seq_file *m)
{
	UFS_DEVINFO_PROC_MSG(m, hba->dev, "-- Crypto Features ----\n");
	UFS_DEVINFO_PROC_MSG(m, hba->dev, "Features          = 0x%x\n",
		hba->crypto_feature);
	UFS_DEVINFO_PROC_MSG(m, hba->dev, " HW-FDE           = 0x%x\n",
		UFS_CRYPTO_HW_FDE);
	UFS_DEVINFO_PROC_MSG(m, hba->dev, " HW-FDE Encrypted = 0x%x\n",
		UFS_CRYPTO_HW_FDE_ENCRYPTED);
	UFS_DEVINFO_PROC_MSG(m, hba->dev, " HW-FBE           = 0x%x\n",
		UFS_CRYPTO_HW_FBE);
	UFS_DEVINFO_PROC_MSG(m, hba->dev, " HW-FBE Encrypted = 0x%x\n",
		UFS_CRYPTO_HW_FBE_ENCRYPTED);
	UFS_DEVINFO_PROC_MSG(m, hba->dev, "-----------------------\n");
}
#endif

void ufs_mtk_dbg_add_trace(struct ufs_hba *hba,
	enum ufs_trace_event event, u32 tag,
	u8 lun, u32 transfer_len, sector_t lba, u8 opcode,
	unsigned long long ppn, u32 region, u32 subregion, u32 resv)
{
#ifdef CONFIG_MTK_UFS_DEBUG
	int ptr;
	unsigned long flags;

	if (!ufs_mtk_is_cmd_dump_lock_init) {
		spin_lock_init(&ufs_mtk_cmd_dump_lock);
		ufs_mtk_is_cmd_dump_lock_init = 1;

		atomic_set(&cmd_hist_enabled, 1);
	}

	if (!atomic_read(&cmd_hist_enabled))
		return;

	spin_lock_irqsave(&ufs_mtk_cmd_dump_lock, flags);

	ufs_cmd_ptr++;

	if (ufs_cmd_ptr >= MAX_UFS_CMD_HLIST_ENTRY_CNT)
		ufs_cmd_ptr = 0;

	ptr = ufs_cmd_ptr;

	ufs_cmd_hlist[ptr].pid = current->pid;
	ufs_cmd_hlist[ptr].event = event;
	ufs_cmd_hlist[ptr].tag = tag;
	ufs_cmd_hlist[ptr].transfer_len = transfer_len;
	ufs_cmd_hlist[ptr].lun = lun;
	ufs_cmd_hlist[ptr].lba = lba;
	ufs_cmd_hlist[ptr].opcode = opcode;
	ufs_cmd_hlist[ptr].time = sched_clock();
	ufs_cmd_hlist[ptr].duration = 0;
	ufs_cmd_hlist[ptr].rq = NULL;
#if defined(CONFIG_UFSHPB)
	ufs_cmd_hlist[ptr].ppn = ppn;
	ufs_cmd_hlist[ptr].region = region;
	ufs_cmd_hlist[ptr].subregion = subregion;
	ufs_cmd_hlist[ptr].resv = resv;
#endif

	/* keep request pointer to dig out block layer status */
	if ((event == UFS_TRACE_SEND) || (event == UFS_TRACE_COMPLETED)) {
		if (hba->lrb[tag].cmd && hba->lrb[tag].cmd->request) {
			ufs_cmd_hlist[ptr].rq =
				hba->lrb[tag].cmd->request;
			ufs_cmd_hlist[ptr].crypted =
				hba->lrb[tag].crypto_en;
			ufs_cmd_hlist[ptr].keyslot =
				ufs_cmd_hlist[ptr].crypted ?
				hba->lrb[tag].crypto_cfgid : 0;
		}
	}

	if (event == UFS_TRACE_COMPLETED) {
		if (ufs_cmd_ptr == 0)
			ptr = MAX_UFS_CMD_HLIST_ENTRY_CNT - 1;
		else
			ptr = ufs_cmd_ptr - 1;

		while (1) {
			if (ufs_cmd_hlist[ptr].tag == tag) {
				ufs_cmd_hlist[ufs_cmd_ptr].duration =
					sched_clock() - ufs_cmd_hlist[ptr].time;
				break;
			}

			ptr--;
			if (ptr < 0)
				ptr = MAX_UFS_CMD_HLIST_ENTRY_CNT - 1;


			if (ufs_cmd_ptr == 0) {
				if (ptr == (MAX_UFS_CMD_HLIST_ENTRY_CNT - 1))
					break;
			} else {
				if (ptr == ufs_cmd_ptr - 1)
					break;
			}
		}
	}

	ufs_cmd_cnt++;

	spin_unlock_irqrestore(&ufs_mtk_cmd_dump_lock, flags);
#endif
}

void ufs_mtk_dbg_dump_trace(char **buff, unsigned long *size,
	u32 latest_cnt, struct seq_file *m)
{
#ifdef CONFIG_MTK_UFS_DEBUG
	int ptr;
	int dump_cnt;
	unsigned long flags;

	if (!ufs_mtk_is_cmd_dump_lock_init) {
		spin_lock_init(&ufs_mtk_cmd_dump_lock);
		ufs_mtk_is_cmd_dump_lock_init = 1;
	}

	spin_lock_irqsave(&ufs_mtk_cmd_dump_lock, flags);

	if (ufs_cmd_cnt > MAX_UFS_CMD_HLIST_ENTRY_CNT)
		dump_cnt = MAX_UFS_CMD_HLIST_ENTRY_CNT;
	else
		dump_cnt = ufs_cmd_cnt;

	if (latest_cnt)
		dump_cnt = min_t(u32, latest_cnt, dump_cnt);

	ptr = ufs_cmd_ptr;

	SPREAD_PRINTF(buff, size, m,
		"[ufs] CMD History: req_cnt=%d, real_cnt=%d, ptr=%d\n",
		latest_cnt, dump_cnt, ptr);

	while (dump_cnt > 0) {

		if ((ufs_cmd_hlist[ptr].event >= UFS_TRACE_UIC_SEND) &&
			(ufs_cmd_hlist[ptr].event <=
			UFS_TRACE_UIC_CMPL_PWR_CTRL)) {

			SPREAD_PRINTF(buff, size, m,
				"%3d-u,%5d,%2d,0x%2X,arg1=0x%X,arg2=0x%X,arg3=0x%X,%llu\n",
				ptr,
				ufs_cmd_hlist[ptr].pid,
				ufs_cmd_hlist[ptr].event,
				ufs_cmd_hlist[ptr].opcode,       /* command */
				ufs_cmd_hlist[ptr].tag,          /* argument1 */
				ufs_cmd_hlist[ptr].transfer_len, /* argument2 */
				(u32)ufs_cmd_hlist[ptr].lba,     /* argument3 */
				(u64)ufs_cmd_hlist[ptr].time
				);

		} else if (ufs_cmd_hlist[ptr].event == UFS_TRACE_REG_TOGGLE) {

			SPREAD_PRINTF(buff, size, m,
				"%3d-g,%5d,state=%d,on=%d,%llu\n",
				ptr,
				ufs_cmd_hlist[ptr].pid,
				ufs_cmd_hlist[ptr].tag,          /* state */
				ufs_cmd_hlist[ptr].transfer_len, /* on or off */
				(u64)ufs_cmd_hlist[ptr].time
				);

		} else if (ufs_cmd_hlist[ptr].event == UFS_TRACE_TM_SEND ||
			ufs_cmd_hlist[ptr].event == UFS_TRACE_TM_COMPLETED) {

			SPREAD_PRINTF(buff, size, m,
				"%3d-t,%5d,%2d,tm=0x%x,t=%2d,lun=0x%x,data=0x%x,%llu\n",
				ptr,
				ufs_cmd_hlist[ptr].pid,
				ufs_cmd_hlist[ptr].event,
				ufs_cmd_hlist[ptr].opcode,
				ufs_cmd_hlist[ptr].tag,
				ufs_cmd_hlist[ptr].lun,
				ufs_cmd_hlist[ptr].transfer_len,
				(u64)ufs_cmd_hlist[ptr].time
				);

		} else if (ufs_cmd_hlist[ptr].event == UFS_TRACE_DEV_SEND ||
			ufs_cmd_hlist[ptr].event == UFS_TRACE_DEV_COMPLETED) {

			SPREAD_PRINTF(buff, size, m,
				"%3d-d,%5d,%2d,0x%2x,t=%2d,lun=0x%x,idn=0x%x,idx=0x%x,sel=0x%x,%llu\n",
				ptr,
				ufs_cmd_hlist[ptr].pid,
				ufs_cmd_hlist[ptr].event,
				ufs_cmd_hlist[ptr].opcode,
				ufs_cmd_hlist[ptr].tag,
				ufs_cmd_hlist[ptr].lun,
				(u8)ufs_cmd_hlist[ptr].lba & 0xFF,
				(u8)(ufs_cmd_hlist[ptr].lba >> 8) & 0xFF,
				(u8)(ufs_cmd_hlist[ptr].lba >> 16) & 0xFF,
				(u64)ufs_cmd_hlist[ptr].time
				);

		} else {
			SPREAD_PRINTF(buff, size, m,
				"%3d-r,%5d,%2d,0x%2x,t=%2d,lun=0x%x,crypt:%d,%d,lba=0x%llx,len=%6d,%llu,\t%llu",
				ptr,
				ufs_cmd_hlist[ptr].pid,
				ufs_cmd_hlist[ptr].event,
				ufs_cmd_hlist[ptr].opcode,
				ufs_cmd_hlist[ptr].tag,
				ufs_cmd_hlist[ptr].lun,
				ufs_cmd_hlist[ptr].crypted,
				ufs_cmd_hlist[ptr].keyslot,
				(long long int)ufs_cmd_hlist[ptr].lba,
				ufs_cmd_hlist[ptr].transfer_len,
				(u64)ufs_cmd_hlist[ptr].time,
				(u64)ufs_cmd_hlist[ptr].duration
				);
#if defined(CONFIG_UFSHPB)
			SPREAD_PRINTF(buff, size, m,
				",\tppn=0x%llx,\tregion=0x%x,\tsubregion=0x%x,\tresv=0x%x",
				ufs_cmd_hlist[ptr].ppn,
				ufs_cmd_hlist[ptr].region,
				ufs_cmd_hlist[ptr].subregion,
				ufs_cmd_hlist[ptr].resv
				);
#endif
			SPREAD_PRINTF(buff, size, m, "\n");
		}

		dump_cnt--;

		ptr--;

		if (ptr < 0)
			ptr = MAX_UFS_CMD_HLIST_ENTRY_CNT - 1;
	}

	spin_unlock_irqrestore(&ufs_mtk_cmd_dump_lock, flags);
#endif
}

void ufs_mtk_dbg_hang_detect_dump(void)
{
#ifdef CONFIG_MTK_UFS_DEBUG
	int boot_type;

	boot_type = get_boot_type();
	if (boot_type != BOOTDEV_UFS)
		return;

	/*
	 * do not touch host to get unipro or mphy information via
	 * dme commands during exception handling since interrupt
	 * or preemption may be disabled.
	 */
	ufshcd_print_host_state(ufs_mtk_hba, 0, NULL, NULL, NULL);

	ufs_mtk_dbg_dump_trace(NULL, NULL,
		ufs_mtk_hba->nutrs + ufs_mtk_hba->nutrs / 2, NULL);

	ufshcd_print_all_uic_err_hist(ufs_mtk_hba, NULL, NULL, NULL);
#endif
}



void ufs_mtk_dbg_proc_dump(struct seq_file *m)
{
#ifdef CONFIG_MTK_UFS_DEBUG
	ufs_mtk_dbg_dump_feature(ufs_mtk_hba, m);

	/*
	 * do not touch host to get unipro or mphy information via
	 * dme commands during exception handling since interrupt
	 * or preemption may be disabled.
	 */
	ufshcd_print_host_state(ufs_mtk_hba, 0, m, NULL, NULL);

	ufs_mtk_dbg_dump_trace(NULL, NULL,
		MAX_UFS_CMD_HLIST_ENTRY_CNT, m);

	ufshcd_print_all_uic_err_hist(ufs_mtk_hba, m, NULL, NULL);
#endif
}

void get_ufs_aee_buffer(unsigned long *vaddr, unsigned long *size)
{
#ifdef CONFIG_MTK_UFS_DEBUG
	unsigned long free_size = UFS_AEE_BUFFER_SIZE;
	char *buff;

	if (ufs_mtk_hba == NULL) {
		pr_info("====== Null ufs_mtk_hba, dump skipped ======\n");
		return;
	}

	buff = ufs_aee_buffer;
	ufshcd_print_host_state(ufs_mtk_hba, 0, NULL, &buff, &free_size);
	ufs_mtk_dbg_dump_trace(&buff, &free_size,
		MAX_UFS_CMD_HLIST_ENTRY_CNT, NULL);

	ufshcd_print_all_uic_err_hist(ufs_mtk_hba,
		NULL, &buff, &free_size);

	/* retrun start location */
	*vaddr = (unsigned long)ufs_aee_buffer;
	*size = UFS_AEE_BUFFER_SIZE - free_size;
#endif
}
EXPORT_SYMBOL(get_ufs_aee_buffer);

static int ufsdbg_dump_health_desc(struct seq_file *file)
{
#ifdef CONFIG_MTK_UFS_DEBUG
	int err = 0;
	int buff_len = QUERY_DESC_HEALTH_MAX_SIZE;
	u8 desc_buf[QUERY_DESC_HEALTH_MAX_SIZE];
	int i;

	pm_runtime_get_sync(ufs_mtk_hba->dev);
	err = ufshcd_read_health_desc(ufs_mtk_hba, desc_buf, buff_len);
	pm_runtime_put_sync(ufs_mtk_hba->dev);


	if (err) {
		seq_printf(file, "Reading Health Descriptor failed. err = %d\n",
			err);
		goto out;
	}

	for (i = 0; i < QUERY_DESC_HEALTH_MAX_SIZE; i++) {
		seq_printf(file,
			"Health Descriptor[0x%x] = 0x%x\n",
			i,
			(u8)desc_buf[i]);
	}

	seq_printf(file,
		"Health Descriptor[offset 0x02]: %s = 0x%x\n",
		"bPreEOLInfo",
		(u8)desc_buf[2]);

	seq_printf(file,
		"Health Descriptor[offset 0x03]: %s = 0x%x\n",
		"bDeviceLifeTimeEstA",
		(u8)desc_buf[3]);

	seq_printf(file,
		"Health Descriptor[offset 0x04]: %s = 0x%x\n",
		"bDeviceLifeTimeEstB",
		(u8)desc_buf[4]);

out:
	return err;
#else
	return 0;
#endif
}

static char cmd_buf[256];

static int ufs_help_proc_show(struct seq_file *m, void *v)
{
	seq_puts(m, "\n===============[ufs_help]================\n");

	seq_printf(m, "\n   Commands dump: echo %x [host_id] > ufs_debug\n",
		UFS_CMDS_DUMP);

	seq_printf(m, "\n   Get Power Mode Status: echo %x [host_id] > ufs_debug\n",
		UFS_GET_PWR_MODE);

	seq_printf(m, "\n   Dump Health Descriptors: echo %x [host_id] > ufs_debug\n",
		UFS_DUMP_HEALTH_DESCRIPTOR);

	seq_puts(m, "\n   NOTE: All input data is Hex number!\n");

	seq_puts(m, "\n=============================================\n\n");

	return 0;
}

/* ========== driver proc interface =========== */
static int ufs_debug_proc_show(struct seq_file *m, void *v)
{
	unsigned long cmd;

	seq_printf(m, "ufsdbg: debug command: %s\n", cmd_buf);

	if (kstrtoul(cmd_buf, 10, &cmd))
		cmd = UFS_CMDS_DUMP;

	cmd_buf[0] = '\0';

	if (cmd == UFS_CMDS_DUMP) {
		/*
		 * Default print cmd history for aee:
		 * JE/NE/ANR/EE/SWT/system api dump
		 */
		seq_puts(m, "==== UFS Debug Info ====\n");
		ufs_mtk_dbg_proc_dump(m);
	} else if (cmd == UFS_GET_PWR_MODE) {
		seq_puts(m, "(1:FAST 2:SLOW 4:FAST_AUTO 5:SLOW_AUTO 7:UNCHANGE)\n");
		seq_printf(m, "Power Mode: tx 0x%x rx 0x%x\n",
			ufs_mtk_hba->pwr_info.pwr_tx,
			ufs_mtk_hba->pwr_info.pwr_rx);
		seq_printf(m, "Gear: tx 0x%x rx 0x%x\n",
			ufs_mtk_hba->pwr_info.gear_tx,
			ufs_mtk_hba->pwr_info.gear_rx);
		seq_printf(m, "HS Rate: 0x%x (1:HS_A 2:HS_B)\n",
			ufs_mtk_hba->pwr_info.hs_rate);
		seq_printf(m, "Lanes: tx 0x%x rx 0x%x\n",
			ufs_mtk_hba->pwr_info.lane_tx,
			ufs_mtk_hba->pwr_info.lane_rx);
	} else if (cmd == UFS_DUMP_HEALTH_DESCRIPTOR) {
		ufsdbg_dump_health_desc(m);
	}

	return 0;
}

static ssize_t ufs_debug_proc_write(struct file *file, const char *buf,
	size_t count, loff_t *data)
{
	unsigned long op = 0;
	bool handled = false;

	if (count == 0 || count > 255)
		return -EINVAL;

	if (copy_from_user(cmd_buf, buf, count))
		return -EINVAL;

	/*
	 * Let's handle simple commands here.
	 * Simple command can be executed immediately
	 * after command is written and do not need further
	 * "read" or "cat" anymore.
	 */
	cmd_buf[count] = '\0';
	if (kstrtoul(cmd_buf, 10, &op))
		return -EINVAL;

	if (op == UFS_CMD_HIST_BEGIN) {
		atomic_set(&cmd_hist_enabled, 1);
		pr_info("ufsdbg: cmd history on\n");
		handled = true;
	} else if (op == UFS_CMD_HIST_STOP) {
		atomic_set(&cmd_hist_enabled, 0);
		pr_info("ufsdbg: cmd history off\n");
		handled = true;
	}

	if (handled)
		cmd_buf[0] = '\0';

	return count;
}

static int ufs_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufs_debug_proc_show, inode->i_private);
}

static const struct file_operations ufs_proc_fops = {
	.open = ufs_proc_open,
	.write = ufs_debug_proc_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ufs_help_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufs_help_proc_show, inode->i_private);
}
static const struct file_operations ufs_help_fops = {
	.open = ufs_help_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

#ifndef USER_BUILD_KERNEL
#define PROC_PERM		0660
#else
#define PROC_PERM		0440
#endif

int ufs_mtk_debug_proc_init(void)
{
	struct proc_dir_entry *prEntry;
	kuid_t uid;
	kgid_t gid;

	uid = make_kuid(&init_user_ns, 0);
	gid = make_kgid(&init_user_ns, 1001);

	prEntry = proc_create("ufs_debug", PROC_PERM, NULL, &ufs_proc_fops);

	if (prEntry)
		proc_set_user(prEntry, uid, gid);
	else
		pr_info("[%s]: failed to create /proc/ufs_debug\n", __func__);

	prEntry = proc_create("ufs_help", PROC_PERM, NULL, &ufs_help_fops);

	if (!prEntry)
		pr_info("[%s]: failed to create /proc/ufs_help\n", __func__);

	return 0;
}
EXPORT_SYMBOL_GPL(ufs_mtk_debug_proc_init);

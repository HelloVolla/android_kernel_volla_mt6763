/*
 * Implementation of the GPS EMI driver.
 *
 * Copyright (C) 2014 Mediatek
 * Authors:
 * Heiping <Heiping.Lei@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*******************************************************************************
* Dependency
*******************************************************************************/
#ifdef CONFIG_MTK_GPS_EMI
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/printk.h>
#include <linux/version.h>
#include <asm/memblock.h>
#if EMI_MPU_PROTECTION_IS_READY
#include <mt_emi_api.h>
#endif
#include "gps.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "["KBUILD_MODNAME"]" fmt

/******************************************************************************
 * Definition
******************************************************************************/
/* device name and major number */
#define GPSEMI_DEVNAME            "gps_emi"
#define IOCTL_MNL_IMAGE_FILE_TO_MEM  1
#define IOCTL_MNL_NVRAM_FILE_TO_MEM  2
#define IOCTL_MNL_NVRAM_MEM_TO_FILE  3

#if defined(CONFIG_MACH_MT6765) || defined(CONFIG_MACH_MT6761)
#define GPS_EMI_MPU_REGION           29
#define GPS_EMI_BASE_ADDR_OFFSET     (2*SZ_1M + SZ_1M/2 + 0x1000)
#define GPS_EMI_MPU_SIZE             (SZ_1M + SZ_1M/2 - 0x2000)
#endif
#if defined(CONFIG_MACH_MT6771) || defined(CONFIG_MACH_MT6775) || defined(CONFIG_MACH_MT6758)
#define GPS_EMI_MPU_REGION           30
#define GPS_EMI_BASE_ADDR_OFFSET     (SZ_1M)
#define GPS_EMI_MPU_SIZE             (SZ_1M)
#endif

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define GPS_DBG_NONE(fmt, arg...)    do {} while (0)
#define GPS_DBG pr_err
#define GPS_TRC GPS_DBG_NONE
#define GPS_VER pr_err
#define GPS_ERR pr_err
/*******************************************************************************
* structure & enumeration
*******************************************************************************/
/*---------------------------------------------------------------------------*/
struct gps_emi_dev {
	struct class *cls;
	struct device *dev;
	dev_t devno;
	struct cdev chdev;
};
/* typedef unsigned char   UINT8, *PUINT8, **PPUINT8; */

/******************************************************************************
 * local variables
******************************************************************************/
phys_addr_t gGpsEmiPhyBase;
void __iomem *pGpsEmibaseaddr;
struct gps_emi_dev *devobj;

static struct semaphore fw_dl_mtx;

void mtk_wcn_consys_gps_memory_reserve(void)
{
#if 0
#ifdef MTK_WCN_ARM64
	gGpsEmiPhyBase = arm64_memblock_steal(SZ_1M, SZ_1M);
#else
	gGpsEmiPhyBase = arm_memblock_steal(SZ_1M, SZ_1M);
#endif
#else
	gGpsEmiPhyBase = gConEmiPhyBase + GPS_EMI_BASE_ADDR_OFFSET;

#endif
	if (gGpsEmiPhyBase)
		GPS_DBG("memblock done: 0x%zx\n", (size_t)gGpsEmiPhyBase);
	else
		GPS_DBG("memblock fail\n");
}

INT32 gps_emi_mpu_set_region_protection(INT32 region)
{
	#if EMI_MPU_PROTECTION_IS_READY
	struct emi_region_info_t region_info;
	/*set MPU for EMI share Memory */
	GPS_DBG("setting MPU for EMI share memory\n");
	region_info.start = gGpsEmiPhyBase;
	region_info.end = gGpsEmiPhyBase + GPS_EMI_MPU_SIZE - 1;
	region_info.region = region;
	SET_ACCESS_PERMISSION(region_info.apc, LOCK,
	FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN,
	FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN,
	NO_PROTECTION, FORBIDDEN, NO_PROTECTION);
	emi_mpu_set_protection(&region_info);
	#endif
	return 0;
}

INT32 gps_emi_patch_get(PUINT8 pPatchName, osal_firmware **ppPatch)
{
	INT32 iRet = -1;
	osal_firmware *fw = NULL;

	if (!ppPatch) {
		GPS_DBG("invalid ppBufptr!\n");
		return -1;
	}
	*ppPatch = NULL;
	iRet = request_firmware((const struct firmware **)&fw, pPatchName, NULL);
	if (iRet != 0) {
		GPS_DBG("failed to open or read!(%s)\n", pPatchName);
		return -1;
	}
	GPS_DBG("loader firmware %s  ok!!\n", pPatchName);
	iRet = 0;
	*ppPatch = fw;

	return iRet;
}

INT32 mtk_wcn_consys_gps_emi_init(void)
{
	INT32 iRet = -1;
	down(&fw_dl_mtx);
	mtk_wcn_consys_gps_memory_reserve();
	if (gGpsEmiPhyBase) {
		/*set MPU for EMI share Memory*/
		#if EMI_MPU_PROTECTION_IS_READY
		GPS_DBG("setting MPU for EMI share memory\n");
		gps_emi_mpu_set_region_protection(GPS_EMI_MPU_REGION);
		#endif
		GPS_DBG("get consys start phy address(0x%zx)\n", (size_t)gGpsEmiPhyBase);
		#if 0
		/*consys to ap emi remapping register:10001310, cal remapping address*/
		addrPhy = (gGpsEmiPhyBase & 0xFFF00000) >> 20;

		/*enable consys to ap emi remapping bit12*/
		addrPhy -= 0x400;/*Gavin ??*/
		addrPhy = addrPhy | 0x1000;

		CONSYS_REG_WRITE(conn_reg.topckgen_base + CONSYS_EMI_MAPPING_OFFSET,
			CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_EMI_MAPPING_OFFSET) | addrPhy);

		GPS_DBG("GPS_EMI_MAPPING dump(0x%08x)\n",
			CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_EMI_MAPPING_OFFSET));
		#endif

		pGpsEmibaseaddr = ioremap_nocache(gGpsEmiPhyBase, GPS_EMI_MPU_SIZE);
		if (pGpsEmibaseaddr != NULL) {
			unsigned char *pFullPatchName = "MNL.bin";
			osal_firmware *pPatch = NULL;

			GPS_DBG("EMI mapping OK(0x%p)\n", pGpsEmibaseaddr);
			memset_io(pGpsEmibaseaddr, 0, GPS_EMI_MPU_SIZE);
			if ((pFullPatchName != NULL)
				&& (gps_emi_patch_get(pFullPatchName, &pPatch) == 0)) {
				if (pPatch != NULL) {
					/*get full name patch success*/
					GPS_DBG("get full patch name(%s) buf(0x%p) size(%ld)\n",
						pFullPatchName, (pPatch)->data, (pPatch)->size);
					GPS_DBG("AF get patch, pPatch(0x%p)\n", pPatch);
				}
			}
			if (pPatch != NULL) {
				if ((pPatch)->size <= GPS_EMI_MPU_SIZE) {
					GPS_DBG("Prepare to copy FW\n");
					memcpy(pGpsEmibaseaddr, (pPatch)->data, (pPatch)->size);
					iRet = 1;
					GPS_DBG("pGpsEmibaseaddr_1:0x%08x 0x%08x 0x%08x 0x%08x\n",
						*(unsigned int *)pGpsEmibaseaddr,
						*(((unsigned int *)pGpsEmibaseaddr)+1),
						*(((unsigned int *)pGpsEmibaseaddr)+2),
						*(((unsigned int *)pGpsEmibaseaddr)+3));
					GPS_DBG("pGpsEmibaseaddr_2:0x%08x 0x%08x 0x%08x 0x%08x\n",
						*(((unsigned int *)pGpsEmibaseaddr)+4),
						*(((unsigned int *)pGpsEmibaseaddr)+5),
						*(((unsigned int *)pGpsEmibaseaddr)+6),
						*(((unsigned int *)pGpsEmibaseaddr)+7));
				}
				release_firmware(pPatch);
				pPatch = NULL;
			}
			iounmap(pGpsEmibaseaddr);
		} else {
			GPS_DBG("EMI mapping fail\n");
		}
	} else {
		GPS_DBG("gps emi memory address gGpsEmiPhyBase invalid\n");
	}
	up(&fw_dl_mtx);
	return iRet;
}

/*---------------------------------------------------------------------------*/
long gps_emi_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int retval = 0;

	GPS_DBG("cmd (%d),arg(%ld)\n", cmd, arg);

	switch (cmd) {
	case IOCTL_MNL_IMAGE_FILE_TO_MEM:
	#ifdef SUPPORT_GPS_OFFLOAD
		retval = mtk_wcn_consys_gps_emi_init();
	#endif
		GPS_DBG("IOCTL_MNL_IMAGE_FILE_TO_MEM\n");
		break;

	case IOCTL_MNL_NVRAM_FILE_TO_MEM:
		GPS_DBG("IOCTL_MNL_NVRAM_FILE_TO_MEM\n");
		break;

	case IOCTL_MNL_NVRAM_MEM_TO_FILE:
		GPS_DBG("IOCTL_MNL_NVRAM_MEM_TO_FILE\n");
		break;

	default:
		GPS_DBG("unknown cmd (%d)\n", cmd);
		retval = 0;
		break;
	}
	return retval;

}

/******************************************************************************/
long gps_emi_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return gps_emi_unlocked_ioctl(filp, cmd, arg);
}

/*****************************************************************************/
static int gps_emi_open(struct inode *inode, struct file *file)
{
	GPS_TRC();
	return nonseekable_open(inode, file);
}

/*****************************************************************************/


/*****************************************************************************/
static int gps_emi_release(struct inode *inode, struct file *file)
{
	GPS_TRC();

	return 0;
}

/******************************************************************************/
static ssize_t gps_emi_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;

	GPS_TRC();

	return ret;
}
/******************************************************************************/
static ssize_t gps_emi_write(struct file *file, const char __user *buf, size_t count,
		loff_t *ppos)
{
	ssize_t ret = 0;

	GPS_TRC();

	return ret;
}


/*****************************************************************************/
/* Kernel interface */
static const struct file_operations gps_emi_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = gps_emi_unlocked_ioctl,
	.compat_ioctl = gps_emi_compat_ioctl,
	.open = gps_emi_open,
	.read = gps_emi_read,
	.write = gps_emi_write,
	.release = gps_emi_release,
};

/*****************************************************************************/
static int gps_emi_probe(struct platform_device *dev)
{
	int ret = 0, err = 0;

	pr_err("Enter gps_emi_probe\n");

	devobj = kzalloc(sizeof(*devobj), GFP_KERNEL);
	if (devobj == NULL) {
		err = -ENOMEM;
		ret = -ENOMEM;
		goto err_out;
	}

	pr_err("Registering chardev\n");
	ret = alloc_chrdev_region(&devobj->devno, 0, 1, GPSEMI_DEVNAME);
	if (ret) {
		GPS_ERR("alloc_chrdev_region fail: %d\n", ret);
		err = -ENOMEM;
		goto err_out;
	} else {
		GPS_ERR("major: %d, minor: %d\n", MAJOR(devobj->devno), MINOR(devobj->devno));
	}
	cdev_init(&devobj->chdev, &gps_emi_fops);
	devobj->chdev.owner = THIS_MODULE;
	err = cdev_add(&devobj->chdev, devobj->devno, 1);
	if (err) {
		GPS_ERR("cdev_add fail: %d\n", err);
		goto err_out;
	}
	devobj->cls = class_create(THIS_MODULE, "gpsemi");
	if (IS_ERR(devobj->cls)) {
		GPS_ERR("Unable to create class, err = %d\n", (int)PTR_ERR(devobj->cls));
		goto err_out;
	}
	devobj->dev = device_create(devobj->cls, NULL, devobj->devno, devobj, "gps_emi");

	GPS_ERR("GPS EMI Done\n");
	return 0;

err_out:
	if (devobj != NULL) {
		if (err == 0)
			cdev_del(&devobj->chdev);
		if (ret == 0)
			unregister_chrdev_region(devobj->devno, 1);

		kfree(devobj);
		devobj = NULL;
	}
	return -1;
}

/*****************************************************************************/
static int gps_emi_remove(struct platform_device *dev)
{
	if (!devobj) {
		GPS_ERR("null pointer: %p\n", devobj);
		return -1;
	}

	GPS_DBG("Unregistering chardev\n");
	cdev_del(&devobj->chdev);
	unregister_chrdev_region(devobj->devno, 1);
	device_destroy(devobj->cls, devobj->devno);
	class_destroy(devobj->cls);
	kfree(devobj);
	GPS_DBG("Done\n");
	return 0;
}

/*****************************************************************************/
#ifdef CONFIG_PM
/*****************************************************************************/
static int gps_emi_suspend(struct platform_device *dev, pm_message_t state)
{
	GPS_DBG("dev = %p, event = %u,", dev, state.event);
	if (state.event == PM_EVENT_SUSPEND)
		GPS_DBG("Receive PM_EVENT_SUSPEND!!\n");
	return 0;
}

/*****************************************************************************/
static int gps_emi_resume(struct platform_device *dev)
{
	GPS_DBG("");
	return 0;
}

/*****************************************************************************/
#endif        /* CONFIG_PM */
/*****************************************************************************/
#ifdef CONFIG_OF
static const struct of_device_id apgps_of_ids[] = {
	{ .compatible = "mediatek,gps_emi-v1", },
	{}
};
#endif
static struct platform_driver gps_emi_driver = {
	.probe = gps_emi_probe,
	.remove = gps_emi_remove,
#if defined(CONFIG_PM)
	.suspend = gps_emi_suspend,
	.resume = gps_emi_resume,
#endif
	.driver = {
		.name = GPSEMI_DEVNAME,
		.bus = &platform_bus_type,
#ifdef CONFIG_OF
		.of_match_table = apgps_of_ids,
#endif
	},
};

/*****************************************************************************/
static int __init gps_emi_mod_init(void)
{
	GPS_ERR("gps emi mod register begin");
	int ret = 0;
	int err = 0;

	devobj = kzalloc(sizeof(*devobj), GFP_KERNEL);
	if (devobj == NULL) {
		err = -ENOMEM;
		ret = -ENOMEM;
		goto err_out;
	}

	GPS_ERR("Registering chardev\n");
	ret = alloc_chrdev_region(&devobj->devno, 0, 1, GPSEMI_DEVNAME);
	if (ret) {
		GPS_ERR("alloc_chrdev_region fail: %d\n", ret);
		err = -ENOMEM;
		goto err_out;
	} else {
		GPS_ERR("major: %d, minor: %d\n", MAJOR(devobj->devno), MINOR(devobj->devno));
	}
	cdev_init(&devobj->chdev, &gps_emi_fops);
	devobj->chdev.owner = THIS_MODULE;
	err = cdev_add(&devobj->chdev, devobj->devno, 1);
	if (err) {
		GPS_ERR("cdev_add fail: %d\n", err);
		goto err_out;
	}
	devobj->cls = class_create(THIS_MODULE, "gpsemi");
	if (IS_ERR(devobj->cls)) {
		GPS_ERR("Unable to create class, err = %d\n", (int)PTR_ERR(devobj->cls));
	goto err_out;
	}
	devobj->dev = device_create(devobj->cls, NULL, devobj->devno, devobj, "gps_emi");

	sema_init(&fw_dl_mtx, 1);

	GPS_ERR("GPS EMI Done\n");
	return 0;

err_out:
	if (devobj != NULL) {
		if (err == 0)
			cdev_del(&devobj->chdev);
		if (ret == 0)
			unregister_chrdev_region(devobj->devno, 1);

		kfree(devobj);
		devobj = NULL;
	}
	return -1;
}

/*****************************************************************************/
static void __exit gps_emi_mod_exit(void)
{
	if (!devobj) {
		GPS_ERR("null pointer: %p\n", devobj);
		return;
	}

	GPS_ERR("Unregistering chardev\n");
	cdev_del(&devobj->chdev);
	unregister_chrdev_region(devobj->devno, 1);
	device_destroy(devobj->cls, devobj->devno);
	class_destroy(devobj->cls);
	kfree(devobj);
	GPS_ERR("Done\n");
}

int mtk_gps_emi_init(void)
{
	GPS_ERR("gps emi mod init begin");
	return gps_emi_mod_init();
}

void mtk_gps_emi_exit(void)
{
	GPS_ERR("gps emi mod exit begin");
	return gps_emi_mod_exit();
}

/*****************************************************************************/
#if 0
module_init(gps_emi_mod_init);
module_exit(gps_emi_mod_exit);
#endif
/*****************************************************************************/
MODULE_AUTHOR("Heiping Lei <Heiping.Lei@mediatek.com>");
MODULE_DESCRIPTION("GPS EMI Driver");
MODULE_LICENSE("GPL");
#endif

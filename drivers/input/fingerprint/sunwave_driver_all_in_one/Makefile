#
# Makefile for the fingerprint device.
#

# Each configuration option enables a list of files.
#include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include

#just for beanpod v2 compatible
#豆荚 V2 TEE平台，兼容头文件
ccflags-y += -I$(srctree)/drivers/misc/mediatek/teei/V1.0/tz_vfs

#甁砵 TEE 平台，兼容头文件
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/tkcore/core

#sunwave driver file
obj-y          := sf_ctl.o sf_hw.o


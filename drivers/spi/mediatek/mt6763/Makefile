subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include \
					-I$(srctree)/drivers/misc/mediatek/include/mt-plat
ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
subdir-ccflags-y += -I$(srctree)/drivers/spi/trustzone/spi/Tlspi/inc
endif

ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
	include $(srctree)/drivers/misc/mediatek/gud/Makefile.include
endif

#prize lsw add for trustkernel tee start
ifeq ($(CONFIG_TRUSTKERNEL_TEE_FP_SUPPORT),y)
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/tkcore/include/linux
endif
#prize lsw add for trustkernel tee end

obj-$(CONFIG_MTK_SPI) += spi.o spi-dev.o

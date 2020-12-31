# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/hxchipset/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/i2c/$(MTK_PLATFORM)/

# Makefile for the Himax touchscreen drivers.

obj-y    += himax_platform.o himax_ic_HX83102.o himax_common.o himax_debug.o himax_ic_incell_core.o himax_inspection.o  
obj-$(CONFIG_TOUCHSCREEN_HIMAX_ITO_TEST)        +=      himax_ito_test.o
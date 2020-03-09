subdir-ccflags-y += -Werror
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
# In case the platform does NOT support this type of sensors
obj-y   +=  mtk_tpd.o
obj-y   +=  tpd_button.o
obj-y   +=  tpd_calibrate.o
obj-y   +=  tpd_debug.o
obj-y   +=  tpd_default.o
obj-y   +=  tpd_init.o
obj-y   +=  tpd_misc.o
obj-y   +=  tpd_setting.o

obj-$(CONFIG_TOUCHSCREEN_MTK_FOCALTECH_TS)	+=  focaltech_touch/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT1151)	+=  GT1151/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT5688)	+=  GT5688/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT911)	+=  GT911/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XX_LM)	+=  GT9XX_LM/
obj-$(CONFIG_TOUCHSCREEN_MTK_GSLX680)	+=  gslX680/
obj-$(CONFIG_TOUCHSCREEN_FT5X26)	+=  FT5X26/
obj-$(CONFIG_TOUCHSCREEN_MTK_NT36xxx)	+=  NT36xxx/

#prize added by chenjiaxi, input driver, 20190111-start
obj-$(CONFIG_TOUCHSCREEN_MTK_SYNAPTICS_DSX) +=  synaptics_dsx/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT1X_V14)	+=  GT1X_V14/
#prize added by chenjiaxi, input driver, 20190111-end
#prize-add HX83112-pengguangyi-20190119-start
obj-$(CONFIG_TOUCHSCREEN_MTK_HX83112)	+=  HX83112/
#prize-add HX83112-pengzhipeng-20190119-end
obj-$(CONFIG_TOUCHSCREEN_MTK_FOCALTECH_TS_V3) +=  focaltech_touch_V3/

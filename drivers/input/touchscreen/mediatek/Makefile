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
obj-y   +=  tpd_misc.o
obj-y   +=  tpd_setting.o
obj-$(CONFIG_TOUCHSCREEN_MTK_SOLOMON)	+=  SOLOMON/
obj-$(CONFIG_TOUCHSCREEN_MTK_FOCALTECH_TS)	+=  focaltech_touch/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT1151)	+=  GT1151/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT911)	+=  GT911/
obj-$(CONFIG_TOUCHSCREEN_MTK_GSLX680)	+=  gslX680/
obj-$(CONFIG_TOUCHSCREEN_MTK_NT36xxx)	+=  NT36xxx/
obj-$(CONFIG_TOUCHSCREEN_MTK_NT36572_COMMON)   +=  nt36572_common/
#prize-add-hx83102-pengzhipeng-20200304-start
obj-$(CONFIG_TOUCHSCREEN_HIMAX_CHIPSET)	+=  hxchipset/
#prize-add-hx83102-pengzhipeng-20200304-end
#prize-add-pengzhipeng-20191029-start
obj-$(CONFIG_TOUCHSCREEN_MTK_FOCALTECH_TS_V3) +=  focaltech_touch_V3/
#prize-add-pengzhipeng-20191029-end
obj-$(CONFIG_TOUCHSCREEN_MTK_FOCALTECH_HD)	+=  focaltech_hd_touch/
obj-$(CONFIG_TOUCHSCREEN_MTK_FOCALTECH_FHD)	+=  focaltech_fhd_touch/
#prize-add-lishuwen-20191226-start
obj-$(CONFIG_TOUCHSCREEN_MTK_GT1X_V14)	+=  GT1X_V14/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XX_PRIZE)	+=  GT9XX_prize/
#prize-add-lishuwen-20191226-end
#prize-add-songhezhu-20191231-start
obj-$(CONFIG_TOUCHSCREEN_CHIPONE) += chipone-tddi/
#prize-add-songhezhu-20191231-end
#prize-add HX83112-pengguangyi-20190119-start
obj-$(CONFIG_TOUCHSCREEN_MTK_HX83112)	+=  HX83112/
#prize-add HX83112-pengzhipeng-20190119-end
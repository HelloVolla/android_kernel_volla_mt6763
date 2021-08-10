# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
obj-y   +=  alsps.o
obj-y   +=  alsps_factory.o
obj-y   +=  aal_control.o

obj-$(CONFIG_MTK_ALSPSHUB)      +=  alspshub/
obj-$(CONFIG_MTK_CM36558)	+=  cm36558/

#prize-add sensor-tangcong-20191231-start
obj-$(CONFIG_MTK_STK3X3X)	+=  stk3x3x/
#prize-add sensor-tangcong-20191231-end
#prize-add sensor-pengzhipeng-20190419-start
obj-$(CONFIG_MTK_STK3332_V2000)	+=  stk3332_v2000/
#prize-add sensor-pengzhipeng-20190419-end
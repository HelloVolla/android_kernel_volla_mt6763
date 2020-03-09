# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
obj-y   +=  alsps.o
obj-y   +=  alsps_factory.o
obj-y   +=  aal_control.o

obj-$(CONFIG_MTK_ALSPSHUB)      +=  alspshub/
obj-$(CONFIG_MTK_CM36558)	+=  cm36558/
obj-$(CONFIG_MTK_EPL2182)	+=  epl2182/
obj-$(CONFIG_MTK_LTR303)	+=  ltr303/
obj-$(CONFIG_MTK_STK3X1X_NEW)	+=  stk3x1x-new/
obj-$(CONFIG_MTK_STK3X3X)	+=  stk3x3x/
#prize-add sensor-pengzhipeng-20190419-start
obj-$(CONFIG_MTK_STK3332_V2000)	+=  stk3332_v2000/
#prize-add sensor-pengzhipeng-20190419-end

#prize added by huarui, sensor driver, 20190111-start
obj-$(CONFIG_MTK_STK3332SV)   +=  stk3332sv/
obj-$(CONFIG_MTK_LTR559)	+=  ltr559/
#prize added by huarui, sensor driver, 20190111-end

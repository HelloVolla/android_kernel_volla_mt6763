# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/include

obj-y   +=  accel.o accel_factory.o

obj-$(CONFIG_MTK_ACCELHUB) += accelhub/
obj-$(CONFIG_MTK_MC3433)   += mc3433/
obj-$(CONFIG_MTK_LSM6DS3HG)+= lsm6ds3hg/
obj-$(CONFIG_MTK_LIS3DH)   += lis3dh/
obj-$(CONFIG_MTK_BMI160_I2C)   += bmi160-i2c/
obj-$(CONFIG_MTK_MC3410_I2C) += mc3410-i2c/
obj-$(CONFIG_MTK_MPU6050G)   += mpu6050g/
obj-$(CONFIG_MTK_LIS3DH_NEW) += lis3dh/
#prize-add BMA4XY-pengzhipeng-20180829-start
obj-$(CONFIG_MTK_BMA4XY)   +=  bma4xy/
#prize-add BMA4XY-pengzhipeng-20180829-end
#prize added by huarui, sensor driver, 20190111-start
obj-$(CONFIG_MTK_MXC400X_PRIZE)   +=  mxc400x_prize/
obj-$(CONFIG_MTK_DA226)   +=  da226/
#prize added by huarui, sensor driver, 20190111-end
obj-$(CONFIG_MTK_ICM426XX_A)   +=  icm426xx_a/

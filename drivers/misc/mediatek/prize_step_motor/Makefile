ccflags-y += -I$(srctree)/drivers/misc/mediatek/leds/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/imgsensor/inc/
obj-y	+= prize_step_motor.o

ccflags-y += $(addprefix -DPRIZE_MB_, $(shell echo $(CONFIG_PRIZE_BOARD) | tr a-z A-Z))
ccflags-y += -I$(srctree)/drivers/misc/mediatek/leds/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/imgsensor/inc/
obj-y	+= prize_dual_camera.o
obj-y	+= gc032a.o
obj-y	+= gc0310.o

ccflags-y += $(addprefix -DPRIZE_MB_, $(shell echo $(CONFIG_PRIZE_BOARD) | tr a-z A-Z))
# Copyright (C) 2016 MediaTek Inc.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See http://www.gnu.org/licenses/gpl-2.0.html for more details.

ifdef MTK_PLATFORM
DRVGEN_PATH := drivers/misc/mediatek/dws/$(MTK_PLATFORM)

ifeq ($(strip $(CONFIG_ARM64)), y)
MAIN_DT_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES))
else
MAIN_DT_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES))
endif

ifeq ($(strip $(CONFIG_MTK_DTBO_FEATURE)), y)
ifeq ($(strip $(CONFIG_ARM64)), y)
PROJ_DT_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM64_DTB_OVERLAY_IMAGE_NAMES))
else
PROJ_DT_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM_DTB_OVERLAY_IMAGE_NAMES))
endif
else #DTBO is not enabled, there is only one dtb
PROJ_DT_NAMES := $(MAIN_DT_NAMES)
endif #CONFIG_MTK_DTBO_FEATURE


MAIN_DTB_NAMES := $(addsuffix .dtb,$(MAIN_DT_NAMES))
PROJ_DTB_NAMES := $(addsuffix .dtb,$(PROJ_DT_NAMES))
MAIN_DTB_FILES := $(addprefix $(objtree)/arch/$(SRCARCH)/boot/dts/, $(MAIN_DTB_NAMES))
PROJ_DTB_FILES := $(addprefix $(objtree)/arch/$(SRCARCH)/boot/dts/, $(PROJ_DTB_NAMES))
PROJ_DTS_FILES := $(addsuffix .dts,$(addprefix $(srctree)/arch/$(SRCARCH)/boot/dts/, $(PROJ_DT_NAMES)))
ABS_DTB_FILES := $(abspath $(addsuffix .dtb,$(addprefix $(objtree)/arch/$(SRCARCH)/boot/dts/,$(PROJ_DT_NAMES))))
ABS_DTB2_FILES := $(abspath $(addprefix $(objtree)/arch/$(SRCARCH)/boot/,mtk.dtb))

export PROJ_DTB_FILES
export PROJ_DTS_FILES

ifndef DRVGEN_OUT
DRVGEN_OUT := $(objtree)/arch/$(SRCARCH)/boot/dts
endif
export DRVGEN_OUT

ALL_DRVGEN_FILE := $(MTK_PROJECT)/cust.dtsi

DWS_FILE := $(srctree)/$(DRVGEN_PATH)/$(MTK_PROJECT).dws
ifneq ($(wildcard $(DWS_FILE)),)
DRVGEN_FILE_LIST := $(addprefix $(DRVGEN_OUT)/,$(ALL_DRVGEN_FILE))
DRVGEN_FILE_LIST += $(PROJ_DTB_FILES)
else
DRVGEN_FILE_LIST :=
endif
DRVGEN_TOOL := $(srctree)/tools/dct/DrvGen.py
DRVGEN_FIG := $(wildcard $(dir $(DRVGEN_TOOL))config/*.fig)

.PHONY: drvgen
drvgen: $(DRVGEN_FILE_LIST)
$(DRVGEN_FILE_LIST): $(DRVGEN_TOOL) $(DWS_FILE) $(DRVGEN_FIG) $(PROJ_DTS_FILES)
	for i in $(PROJ_DTS_FILES); do \
		base_prj=`grep -m 1 '#include [<\"].*\/cust\.dtsi[>\"]' $$i | sed 's/#include [<"]//g'\
	       	| sed 's/\/cust\.dtsi[>"]//g' | sed 's/\/\*//g' | sed 's/\*\///g' | sed 's/ //g'`\
		prj_path=$(DRVGEN_OUT)/$$base_prj ;\
		dws_path=$(srctree)/$(DRVGEN_PATH)/$$base_prj.dws ;\
		if [ -f $$dws_path ] ; then \
			mkdir -p $$prj_path ;\
			$(python) $(DRVGEN_TOOL) $$dws_path $$prj_path $$prj_path cust_dtsi;\
		fi \
	done

dtbo_check: $(MAIN_DTB_NAMES) $(PROJ_DTB_NAMES)
	for i in $(PROJ_DTB_FILES); do \
		$(srctree)/scripts/dtc/ufdt_apply_overlay $(MAIN_DTB_FILES) $$i $$i.merge;\
	done

my_dtbo_id := 0
define mk_dtboimg_cfg
echo $(1) >>$(2);\
echo " id=$(my_dtbo_id)" >>$(2);\
$(eval my_dtbo_id:=$(shell echo $$(($(my_dtbo_id)+1))))
endef

my_dtb_id := 0
define mk_dtbimg_cfg
echo $(1) >>$(2);\
echo " id=$(my_dtb_id)" >>$(2);\
$(eval my_dtb_id:=$(shell echo $$(($(my_dtb_id)+1))))
endef

dtbs: $(objtree)/dtboimg.cfg $(objtree)/dtbimg.cfg
$(objtree)/dtboimg.cfg: FORCE
	rm -f $@.tmp
	$(foreach f,$(ABS_DTB_FILES),$(call mk_dtboimg_cfg,$(f),$@.tmp))
	touch $@.tmp
	if ! cmp -s $@.tmp $@; then \
		mv $@.tmp $@; \
	else \
		rm $@.tmp; \
	fi

$(objtree)/dtbimg.cfg: FORCE
	rm -f $@.tmp
	$(foreach f,$(ABS_DTB2_FILES),$(call mk_dtbimg_cfg,$(f),$@.tmp))
	touch $@.tmp
	if ! cmp -s $@.tmp $@; then \
		mv $@.tmp $@; \
	else \
		rm $@.tmp; \
	fi

endif#MTK_PLATFORM

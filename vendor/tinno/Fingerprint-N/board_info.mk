
ifneq ($(strip $(MTK_TARGET_PROJECT)),)
  TARGET_PRJ := $(MTK_TARGET_PROJECT)
else
  TARGET_PRJ := $(TARGET_PRODUCT)
endif 

# Project info.
BRANCH_PRJ := $(strip $(PROJECT_NAME))
TARGET_PRJ := $(strip $(TARGET_PRJ))

mtk_board_config := device/tinno/$(TARGET_PRJ)/BoardConfig.mk
qcom_board_config := device/qcom/$(TARGET_PRJ)/BoardConfig.mk
ifneq ($(wildcard $(mtk_board_config)),)
    BoardConfig_mk := $(mtk_board_config)

else ifneq ($(wildcard $(qcom_board_config)),)
    BoardConfig_mk := $(qcom_board_config)

else
    BoardConfig_mk :=
endif

# Board info.
ifneq ($(strip $(BoardConfig_mk)),)
board_platform := $(shell cat $(PWD)/$(BoardConfig_mk) |grep TARGET_BOARD_PLATFORM)
ifneq ($(strip $(board_platform)),)
TARGET_BOARD_PLATFORM := \
    $(strip \
    $(subst :=,, \
    $(subst TARGET_BOARD_PLATFORM,,\
    $(board_platform))))
else
$(error --SHIT! MYBE HAS ERR! TARGET_BOARD_PLATFORM--is null!-----)
endif
endif

# prj info.
$(warning ---$(TARGET_PRJ)---$(BRANCH_PRJ)---$(TARGET_BOARD_PLATFORM)---)


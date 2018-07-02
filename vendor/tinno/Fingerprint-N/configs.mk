#
# Fingerprint support dir
TINNO_FINGERPRINT_PATH := vendor/tinno/Fingerprint-N

# em-framework.
include $(TINNO_FINGERPRINT_PATH)/em_framework/configs.mk

######################### TINNO_FINGERPRINT_SUPPORT start { 
ifneq ($(strip $(TINNO_FINGERPRINT_SUPPORT)),)

# Platform board info.
include $(TINNO_FINGERPRINT_PATH)/board_info.mk

# Native.
include $(TINNO_FINGERPRINT_PATH)/native/configs.mk

# Third-part lib.
include $(TINNO_FINGERPRINT_PATH)/third-part/configs.mk

# Selinux policy.
TINNO_FINGERPRINT_SELINUX_SEPOLICY := \
    $(TINNO_FINGERPRINT_PATH)/sepolicy/configs.mk

# init.rc
include $(TINNO_FINGERPRINT_PATH)/root/configs.mk

# etc configs
include $(TINNO_FINGERPRINT_PATH)/etc/configs.mk

endif 
######################### TINNO_FINGERPRINT_SUPPORT end }

# TINNO_TP_BACKTOUCH_SUPPORT
TINNO_BACKTOUCH_SELINUX_SEPOLICY := \
    $(TINNO_FINGERPRINT_PATH)/tp_back_touch/sepolicy/configs.mk




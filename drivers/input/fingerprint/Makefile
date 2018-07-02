#
# Makefile for the fingerprint drivers.
#

TARGET_NAME := \
    $(strip $(subst ro.target =,,$(shell cat $(PWD)/../../../../build.ini |grep ro.target)))

fingerprint_supprot := \
    $(strip $(subst TINNO_FINGERPRINT_SUPPORT =,,$(shell cat $(PWD)/../../../../build.ini |grep TINNO_FINGERPRINT_SUPPORT)))

$(warning ---------------fingerprint_supprot--$(fingerprint_supprot)--------------)
ifneq ($(strip $(fingerprint_supprot)),)
    obj-y += fp_drv/
    subdir-ccflags-y += $(addprefix -D, $(shell echo $(fingerprint_supprot) | tr a-z A-Z))
    fingerprint_drv := $(addsuffix /, $(shell echo $(fingerprint_supprot)) )
    obj-y += $(fingerprint_drv)
endif

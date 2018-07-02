#
# Makefile for USB Core files and filesystem
#

# TINNO BEGIN
# Shuai.Chen, HCABNA-215, Modify For Wiko Unify Version Sensor Controller
WIKO_UNIFY_DEF := unify
$(warning -----Leo Kernel Derive Usb Core Makefile : WIKO_UNIFY_DEF = $(WIKO_UNIFY_DEF)-----)
ifeq ($(strip $(WIKO_UNIFY_DEF)),unify)
ccflags-y += -DCONFIG_WIKO_UNIFY
$(warning -----Leo Kernel Derive Usb Core Makefile : Define CONFIG_WIKO_UNIFY-----)
endif
# TINNO END

usbcore-y := usb.o hub.o hcd.o urb.o message.o driver.o
usbcore-y += config.o file.o buffer.o sysfs.o endpoint.o
usbcore-y += devio.o notify.o generic.o quirks.o devices.o
usbcore-y += port.o

usbcore-$(CONFIG_PCI)		+= hcd-pci.o
usbcore-$(CONFIG_ACPI)		+= usb-acpi.o

obj-$(CONFIG_USB)		+= usbcore.o

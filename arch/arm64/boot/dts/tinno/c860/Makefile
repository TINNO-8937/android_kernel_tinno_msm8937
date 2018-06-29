dtb-$(CONFIG_ARCH_MSM8953) += msm8953-mtp.dtb

dtb-$(CONFIG_ARCH_SDM450) += sdm450-mtp-c860.dtb \
			     sdm450-pmi8940-mtp.dtb
	
ifeq ($(CONFIG_ARM64),y)
always          := $(dtb-y)
subdir-y        := $(dts-dirs)
else
targets += dtbs
targets += $(addprefix ../, $(dtb-y))

$(obj)/../%.dtb: $(src)/%.dts FORCE
        $(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/,$(dtb-y))
endif
clean-files := *.dtb

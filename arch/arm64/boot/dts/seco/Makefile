ifeq ($(CONFIG_CPU_RK3399),y)
        DTC_FLAGS ?= -@ -H epapr
endif

dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revc.dtb
dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revc1.dtb

dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revc-hdmi.dtb
dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revc-lvds.dtb
dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revc-lvds_noconsole.dtb
dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revc-hdmi-lvds.dtb
dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revc1-hdmi.dtb
dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revc1-lvds.dtb
dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revc1-lvds_noconsole.dtb
dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revc1-hdmi-lvds.dtb

dts-dirs += overlays

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb

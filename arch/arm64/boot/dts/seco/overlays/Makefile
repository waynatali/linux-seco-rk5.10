dtbo-$(CONFIG_CPU_RK3399) += seco-rk3399-c31-hdmi.dtbo \
				seco-rk3399-c31-sn65dsi84.dtbo \
				seco-rk3399-c31-edp.dtbo\
				seco-rk3399-c31-typec-dp.dtbo\
				seco-rk3399-c31-can1.dtbo\
				seco-rk3399-c31-can2.dtbo\
				seco-rk3399-c31-spi-eeprom.dtbo\
				seco-rk3399-c31-spi-rtc.dtbo
dtbo-$(CONFIG_CPU_PX30) +=  seco-px30-d23-hdmi.dtbo \
				seco-px30-d23-lvds1280x800.dtbo \
				seco-px30-d23-lvds800x480.dtbo \
				seco-px30-d23-lvds-hdmi.dtbo
targets += dtbs dtbs_install
targets += $(dtbo-y)

always          := $(dtbo-y)
clean-files     := *.dtbo


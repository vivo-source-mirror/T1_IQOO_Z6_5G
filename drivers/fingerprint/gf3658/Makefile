ifneq ($(filter PD1957% PD1965% TD2002,$(BBK_PRODUCT_MODEL)),) ## 共用电源，不下电
$(info "******************************** $(BBK_PRODUCT_MODEL)  share power  ********************************")
ccflags-y += -DSHARE_POWER_ALWAYS_ON
endif

gf3658-objs := gf3658_spi.o gf3658_platform.o gf3658_netlink.o
obj-$(CONFIG_BBK_GF3658) := gf3658.o

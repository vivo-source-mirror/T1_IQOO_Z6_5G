ifeq ($(CONFIG_VIVO_TS),)
  CONFIG_VIVO_TS=y
endif

obj-$(CONFIG_VIVO_TS) += vivo_ts.o

vivo_ts-y += vts_core.o
vivo_ts-y += vts_node.o
vivo_ts-y += vts_incell.o
vivo_ts-y += vts_class.o
vivo_ts-y += vts_report.o
vivo_ts-y += vts_op.o
vivo_ts-y += vts_spi_drv.o

vivo_ts-y += $(addprefix ftm4/, \
				fts.o \
				fts_driver_test.o \
				fts_gui.o \
			)

vivo_ts-y += $(addprefix ftm4/fts_lib/, \
				ftsCompensation.o \
				ftsCrossCompile.o \
				ftsError.o \
				ftsFlash.o \
				ftsFrame.o \
				ftsGesture.o \
				ftsIO.o \
				ftsTest.o \
				ftsTime.o \
				ftsTool.o \
			)

vivo_ts-y += $(addprefix nt36670/, \
				nt36xxx.o \
				nt36xxx_ext_proc.o \
				nt36xxx_fw_update.o \
				nt36xxx_mp_ctrlram.o \
			)

vivo_ts-y += $(addprefix nt-no-flash/, \
				nt36xxx.o \
				nt36xxx_ext_proc.o \
				nt36xxx_fw_update.o \
				nt36xxx_mp_ctrlram.o \
			)

vivo_ts-y += $(addprefix samsung_y761/, \
				sec_ts.o \
				sec_ts_fn.o \
				sec_ts_fw.o \
				sec_ts_only_vendor.o \
			)

vivo_ts-y += $(addprefix ft8719-no-flash/, \
				focaltech_core.o \
				focaltech_esdcheck.o \
				focaltech_ex_fun.o \
				focaltech_flash.o \
				focaltech_gesture.o \
				focaltech_spi.o \
				focaltech_test.o \
			)

vivo_ts-y += $(addprefix ftm5/, \
				fts.o \
				fts_proc.o \
			)

vivo_ts-y += $(addprefix ftm5/fts_lib/, \
				ftsCompensation.o \
				ftsCore.o \
				ftsError.o \
				ftsFlash.o \
				ftsFrame.o \
				ftsGesture.o \
				ftsIO.o \
				ftsLockdownReadwrite.o \
				ftsTest.o \
				ftsTime.o \
				ftsTool.o \
			)

vivo_ts-y += $(addprefix gt9886/, \
				goodix_cfg_bin.o \
				goodix_ts_core.o \
				goodix_ts_gesture.o \
				goodix_ts_i2c.o \
				goodix_ts_node.o \
				goodix_ts_spi.o \
				goodix_ts_tools.o \
				goodix_ts_update.o \
			)

vivo_ts-y += $(addprefix ft8756-no-flash/, \
				focaltech_core.o \
				focaltech_esdcheck.o \
				focaltech_ex_fun.o \
				focaltech_flash.o \
				focaltech_gesture.o \
				focaltech_spi.o \
				focaltech_test.o \
			)

vivo_ts-y += $(addprefix himax83113/, \
				himax_common.o \
				himax_debug.o \
				himax_ic_HX83112.o \
				himax_ic_incell_core.o \
				himax_inspection.o \
				himax_platform.o \
			)


vivo_ts-y += $(addprefix ili_9882n/, \
				ili9881x.o \
				ili9881x_hostdl.o \
				ili9881x_ic.o \
				ili9881x_mp.o \
				ili9881x_node.o \
				ili9881x_plat.o \
				ili9881x_spi.o \
				ili9881x_touch.o \
				vivo_v2.o \
			)

vivo_ts-y += $(addprefix syna_S3908_tcm/, \
				synaptics_tcm_i2c.o \
				synaptics_tcm_core.o \
				synaptics_tcm_touch.o \
				synaptics_tcm_device.o \
				synaptics_tcm_testing.o \
				synaptics_tcm_reflash.o \
				synaptics_tcm_recovery.o \
				synaptics_tcm_diagnostics.o \
				synaptics_tcm_vivo_interface.o \
			)

vivo_ts-y += vts_init.o

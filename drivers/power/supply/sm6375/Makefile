# SPDX-License-Identifier: GPL-2.0-only

#obj-$(CONFIG_QPNP_SMB5) += qpnp-smb5-main.o
#qpnp-smb5-main-y += bq25890h_charger.o vivo-current-split.o pmic-voter.o
#obj-$(CONFIG_CHARGER_MCU) += vivo-chg-mcu.o 
#vivo-chg-mcu-y := vivo-stm32l011x4-dual-chg.o

#obj-$(CONFIG_BQ25890H_CHARGER)   += bq25890h-charger.o
#bq25890h-charger-y += bq25890h_charger.o vivo-current-split.o

#obj-$(CONFIG_QPNP_QG)	+= qcom-qpnp-qg.o
#qcom-qpnp-qg-y	+=vivo-sm5602-fg.o

#obj-$(CONFIG_QPNP_TI)	+= qcom-qpnp-ti.o
#qcom-qpnp-ti-y	+=vivo-bq27426-battery.o

#obj-$(CONFIG_CHARGER_MONITOR)   += vivo-chg-cms.o
#vivo-chg-cms-y += charger-monitor-v2.o charger-monitor-v2-class.o pmic-voter.o

#obj-$(CONFIG_FUEL_SUMMARY) += vivo-fuel_summary.o

#obj-$(CONFIG_BQ24192_SLAVE_CHARGER)	+= bq24192-slave-charger.o
#bq24192-slave-charger-y += bq24192_slave_charger.o bq25601d_slave_charger.o

base_project_name := $(firstword $(subst F, ,$(BBK_PRODUCT_MODEL)))
$(warning  get the project base name : base_name=$(base_project_name).......)
ccflags-y += -DVIVO_PROJECT_BASE_NAME="$(base_project_name)"
#ccflags-y += -DVIVO_BURN_FW_HV_H="vivo-burn-fw-PD2134.h"

#ifeq ($(findstring PD2165,$(BBK_PRODUCT_MODEL)),PD2165)
#	ccflags-y += -DVIVO_BQ_BMS_GMFS=\"bq27426_gmfs_coslight_PD2165.h\"
#else ifeq ($(findstring PD2165F_EX,$(BBK_PRODUCT_MODEL)),PD2165F_EX)
# 	ccflags-y += -DVIVO_BQ_BMS_GMFS=\"bq27426_gmfs_coslight_PD2165F_EX.h\"
#else ifeq ($(findstring TD2105,$(BBK_PRODUCT_MODEL)),TD2105)
#	ccflags-y += -DVIVO_BQ_BMS_GMFS=\"bq27426_gmfs_coslight_PD2165.h\"
#else
#	ccflags-y += -DVIVO_BQ_BMS_GMFS=\"bq27426_gmfs_coslight.h\"
#endif

#obj-$(CONFIG_QPNP_SMB5) += qpnp-smb5-main.o
#qpnp-smb5-main-y += step-chg-jeita.o battery.o qpnp-smb5.o smb5-lib.o pmic-voter.o storm-watch.o schgm-flash.o battery-profile-loader.o smb5-iio.o
#obj-$(CONFIG_QPNP_QG)	+= qcom-qpnp-qg.o
#qcom-qpnp-qg-y	+= qpnp-qg.o battery-profile-loader.o pmic-voter.o qg-util.o qg-soc.o qg-sdam.o qg-battery-profile.o qg-profile-lib.o fg-alg.o
#obj-$(CONFIG_SMB1398_CHARGER)           += qcom-smb1398-charger.o
#qcom-smb1398-charger-y += smb1398-charger.o pmic-voter.o
#obj-$(CONFIG_QPNP_SMBLITE) += qpnp-smblite-main.o
#qpnp-smblite-main-y += step-chg-jeita.o battery.o qpnp-smblite.o smblite-lib.o pmic-voter.o storm-watch.o battery-profile-loader.o schgm-flashlite.o smblite-iio.o
#obj-$(CONFIG_SMB1355_SLAVE_CHARGER)	+= qcom-smb1355-charger.o
#qcom-smb1355-charger-y += smb1355-charger.o pmic-voter.o
#obj-$(CONFIG_SMB1351_USB_CHARGER)	+= smb1351-charger.o pmic-voter.o
#obj-$(CONFIG_SMB1390_CHARGE_PUMP_PSY)   += qcom-smb1390-charger.o
#qcom-smb1390-charger-y += smb1390-charger-psy.o pmic-voter.o
#obj-$(CONFIG_CHARGER_MCU) += vivo-chg-mcu.o
#vivo-chg-mcu-y := vivo-stm32l011x4-dual-chg.o

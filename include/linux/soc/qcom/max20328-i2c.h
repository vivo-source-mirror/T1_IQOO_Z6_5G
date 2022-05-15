/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2018-2020, The Linux Foundation. All rights reserved.
 */
#ifndef MAX20328_I2C_H
#define MAX20328_I2C_H

#include <linux/of.h>
#include <linux/notifier.h>
#include <linux/usb/typec.h>
#include <linux/usb/ucsi_glink.h>
#ifdef VUCSI_SET_CCOM
#include <linux/usb/vucsi_set_ccom.h>
#endif

enum fsa_function {
	FSA_MIC_GND_SWAP,
	FSA_USBC_AUIDO_HP_ON,
	FSA_USBC_AUIDO_HP_OFF,
	FSA_USBC_ORIENTATION_CC1,
	FSA_USBC_ORIENTATION_CC2,
	FSA_USBC_DISPLAYPORT_DISCONNECTED,
	FSA_USBC_FAST_CHARGE_SELECT,
	FSA_USBC_FAST_CHARGE_EXIT,
	FSA_USBC_SWITCH_ENABLE,
	FSA_USBC_SWITCH_DISABLE,
	FSA_USBC_SWITCH_SBU_DIRECT_CONNECT,	//SBU1_H=SBU1、SBU2_H=SBU2
	FSA_USBC_SWITCH_SBU_FLIP_CONNECT,	//SBU1_H=SBU2、SBU2_H=SBU1
	FSA_USBC_SWITCH_SBU_HIZ,
	FSA_EVENT_MAX,
};

/*IC type*/
enum {
	MAX20328,
	FSA4480
};

/*FSA4480 RES DETECTION PIN SETTING*/
enum {
	CC_IN__PIN = 0,
	DP_R__PIN,
	DN_L__PIN,
	SBU1__PIN,
	SBU2__PIN,
	ERROR__PIN,	
};

static const char * const RES_detection_pin[] = {
	"CC_IN__PIN",
	"DP_R__PIN",
	"DN_L__PIN",
	"SBU1__PIN",
	"SBU2__PIN",
	"ERROR__PIN",
};

int max20328_extern_usbc_event_changed(int acc);

int max20328_switch_event(struct device_node *node,
			 enum fsa_function event);
int max20328_reg_notifier(struct notifier_block *nb,
			 struct device_node *node);
int max20328_unreg_notifier(struct notifier_block *nb,
			   struct device_node *node);

int max20328_switch_mode_event(enum fsa_function event);
int max20328_notify_mos(int state); /*audio_v add*/
int get_usb_connecter_pin_resistance_value(int pin_sel);

/* Add this func for AT CMD and failure detection by vivo audio team@fanyongxiang.
 * return -1: Not support;
 * return 0: No peripheral;
 * return 1: Insert peripheral.
 */
int get_usbc_peripheral_status(void);

/* Add this func for Mic and Gnd switch status to AT CMD by vivo audio team@fanyongxiang.
 * return -1: Not support;
 * return 1: MG_SR->GSNS;
 * return 2: GM_SR->GSNS.
 */
int get_usbc_mg_status(void);

struct max20328_priv {
	struct regmap *regmap;
	struct device *dev;
	struct kobject *kobj;
	//struct power_supply *usb_psy;
	struct notifier_block ucsi_nb;
	atomic_t usbc_mode;
	struct work_struct usbc_analog_work;
	struct work_struct usbc_removed_work;
	struct work_struct max20328_irq_handler_work;
	struct blocking_notifier_head max20328_notifier;

	/*audio_v: archer add for PD2049*/
	int supply_nonindepend;
	struct regulator *vdd;
	struct regulator *vdda18;
	int	 vdd_levels[3]; /* none, low, high */
	/*audio_v: archer add end*/

	struct regulator *vdda33;
	struct regulator *vdda307;
	wait_queue_head_t irq_waitq;
	bool power_enabled;
	bool sbu_vcc_enabled;
	struct mutex usbc_switch_lock;
	struct wakeup_source *usbc_wake_lock;
	int mmax_en;
	int mmax_int;
	int mmax_int_irq;
	int dev_gpio;
	int dev_gpio_irq;
	int current_plug;
	int gpio_detection_type;
	int current_switch_dev;
	int usbc_ic_mode;
	int sbu_sel;
	int is_mostest;
	int drp_suspend_state;
#ifdef VUCSI_SET_CCOM
	int (*vucsi_vote)(enum vucsi_voter, enum vucsi_cc_mode);
#endif
	int (*switch_mode_event)(enum fsa_function event);
	int is_unuseirq;
};

#endif /* MAX20328_I2C_H */


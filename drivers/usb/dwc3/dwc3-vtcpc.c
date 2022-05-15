/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/atomic.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/power_supply.h>
#include <linux/usb/typec.h>
#include <linux/errno.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
#include <linux/extcon-provider.h>
#else
#include <linux/extcon.h>
#endif

#include "../ext_typec/tcpc/inc/tcpm.h"
#include "../ext_typec/tcpc/inc/tcpci_core.h"

void vote_typec_drp(const char *voter, bool drp){
	return;
};
int get_typec_drp_voter_effective(const char *voter){
	return 0;
}

void vusb_vote_register_callback(void *func_vote, void *func_get);
extern void tcpc_usb_set_data_role(enum typec_port_data role);

struct extcon_nb {
	struct extcon_dev *edev;
	struct dwc3_vtcpc *dwc3_vtcpc;
	int idx;
	struct notifier_block vbus_nb;
};

struct dwc3_vtcpc {
	struct device *dev;
	struct extcon_dev *edev;

	struct tcpc_device *otg_tcpc_dev;
	struct notifier_block otg_nb;
	struct notifier_block vusb_nb;
	struct mutex tcpc_otg_lock;

	struct typec_capability	typec_caps;
	struct typec_port *typec_port;
	struct typec_partner *partner;
	struct typec_partner_desc partner_desc;
	struct extcon_nb *extcon;

	enum typec_port_data data_role;
	unsigned int extcon_cnt;
	unsigned int cur_dr;
	bool tcpc_boost_on;
	bool usbc_otg_attached;
};

static unsigned int usb_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

static struct dwc3_vtcpc *g_dwc3_vtcpc = NULL;

static void tcpc_audio_enable(void)
{
	tcpc_usb_set_data_role(TYPEC_PORT_DRD);
}

static void tcpc_audio_disable(void)
{
	tcpc_usb_set_data_role(TYPEC_PORT_UFP);
}

static void tcpc_otg_enable(struct dwc3_vtcpc *vtcpc)
{
	if (!vtcpc->usbc_otg_attached) {
		tcpc_usb_set_data_role(TYPEC_PORT_DFP);
		extcon_set_state_sync(vtcpc->edev, EXTCON_USB_HOST, true);
		vtcpc->usbc_otg_attached = true;
	}
}

static void tcpc_otg_disable(struct dwc3_vtcpc *vtcpc)
{
	if (vtcpc->usbc_otg_attached) {
		extcon_set_state_sync(vtcpc->edev, EXTCON_USB_HOST, false);
		tcpc_usb_set_data_role(TYPEC_PORT_UFP);
		vtcpc->usbc_otg_attached = false;
	}
}

static void tcpc_usb_connect(struct dwc3_vtcpc *vtcpc)
{
	tcpc_usb_set_data_role(TYPEC_PORT_UFP);
    extcon_set_state_sync(vtcpc->edev, EXTCON_USB, true);
}

static void tcpc_usb_disconnect(struct dwc3_vtcpc *vtcpc)
{
   extcon_set_state_sync(vtcpc->edev, EXTCON_USB, false);
   tcpc_usb_set_data_role(TYPEC_PORT_UFP);
}

static void tcpc_power_work_call(struct dwc3_vtcpc *vtcpc, bool enable)
{
	if (enable) {
		if (!vtcpc->tcpc_boost_on) {
			//mt_vbus_on();
			vtcpc->tcpc_boost_on = true;
		}
	} else {
		if (vtcpc->tcpc_boost_on) {
			//mt_vbus_off();
			vtcpc->tcpc_boost_on = false;
		}
	}
}

static int otg_tcp_notifier_call(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct dwc3_vtcpc *vtcpc = container_of(nb, struct dwc3_vtcpc, otg_nb);
	struct tcp_notify *noti = data;
	bool otg_power_enable, otg_on;

	mutex_lock(&vtcpc->tcpc_otg_lock);
	otg_on = vtcpc->usbc_otg_attached;
	mutex_unlock(&vtcpc->tcpc_otg_lock);

	pr_err("%s come in event=%d", __func__, event);
	switch (event) {
	case TCP_NOTIFY_SOURCE_VBUS:
		pr_info("%s source vbus = %dmv\n",
				__func__, noti->vbus_state.mv);
		otg_power_enable = (noti->vbus_state.mv) ? true : false;
		tcpc_power_work_call(vtcpc, otg_power_enable);
		break;
	case TCP_NOTIFY_TYPEC_STATE:
		pr_info("%s, TCP_NOTIFY_TYPEC_STATE, old_state=%d, new_state=%d\n",
				__func__, noti->typec_state.old_state,
				noti->typec_state.new_state);

		if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
			noti->typec_state.new_state == TYPEC_ATTACHED_SRC) {
			pr_info("%s OTG Plug in\n", __func__);
			tcpc_otg_enable(vtcpc);
		} else if ((noti->typec_state.old_state == TYPEC_ATTACHED_SRC ||
			noti->typec_state.old_state == TYPEC_ATTACHED_SNK) &&
			noti->typec_state.new_state == TYPEC_UNATTACHED) {
			if (otg_on) {
				pr_info("%s OTG Plug out\n", __func__);
				tcpc_otg_disable(vtcpc);
			} else {
				pr_info("%s vivo charging not config, USB Plug out\n", __func__);
				//mt_usb_disconnect();
			}
		}

		if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
			noti->typec_state.new_state == TYPEC_ATTACHED_AUDIO) {
			pr_info("%s Audio Plug in\n", __func__);
			tcpc_audio_enable();
		} else if (noti->typec_state.old_state == TYPEC_ATTACHED_AUDIO &&
					noti->typec_state.new_state == TYPEC_UNATTACHED) {
			pr_info("%s Audio Plug out\n", __func__);
			tcpc_audio_disable();
		}
		break;
	case TCP_NOTIFY_DR_SWAP:
		pr_info("%s TCP_NOTIFY_DR_SWAP, new role=%d\n",
				__func__, noti->swap_state.new_role);
		if (otg_on &&
			noti->swap_state.new_role == PD_ROLE_UFP) {
			pr_info("%s switch role to device\n", __func__);
			tcpc_otg_disable(vtcpc);
			tcpc_usb_connect(vtcpc);
		} else if (!otg_on &&
			noti->swap_state.new_role == PD_ROLE_DFP) {
			pr_info("%s switch role to host\n", __func__);
			tcpc_usb_disconnect(vtcpc);
			tcpc_otg_enable((vtcpc));
		}
		break;
	}
	return NOTIFY_OK;
}

static int dwc3_vtcpc_vbus_notifier(struct notifier_block *nb,
				     unsigned long event, void *ptr)
{
	struct extcon_dev *edev = ptr;
	struct extcon_nb *enb = container_of(nb, struct extcon_nb, vbus_nb);
	struct dwc3_vtcpc *dwc3_vtcpc = enb->dwc3_vtcpc;
	const char *edev_name;

	if (!edev || !dwc3_vtcpc)
		return NOTIFY_DONE;

	edev_name = extcon_get_edev_name(edev);
	pr_info("%s: %d: idx=%d event=%d received\n", __func__, __LINE__, enb->idx, event);

	if (event) {
		if (!dwc3_vtcpc->partner) {
			dwc3_vtcpc->partner = typec_register_partner(dwc3_vtcpc->typec_port,
								      &dwc3_vtcpc->partner_desc);
			pr_info("%s: %d: usb gadget connect\n", __func__, __LINE__);
		}
	} else {
		if (dwc3_vtcpc->partner) {
			typec_unregister_partner(dwc3_vtcpc->partner);
			dwc3_vtcpc->partner = NULL;
			pr_info("%s: %d: usb gadget disconnect\n", __func__, __LINE__);
		}
	}

	return NOTIFY_DONE;
}

static int dwc3_vtcpc_extcon_register(struct dwc3_vtcpc *dwc3_vtcpc)
{
	struct device_node *np = dwc3_vtcpc->dev->of_node;
	struct extcon_dev *edev;
	int idx, ret = 0;
	bool check_vbus_state, phandle_found = false;

	dwc3_vtcpc->extcon_cnt = of_count_phandle_with_args(np, "extcon", NULL);
	if (dwc3_vtcpc->extcon_cnt < 0) {
		pr_err("%s: %d: of_count_phandle_with_args failed\n", __func__, __LINE__);
		return -ENODEV;
	}

	pr_err("%s: %d: dwc3_vtcpc->extcon_cnt =%d \n", __func__, __LINE__, dwc3_vtcpc->extcon_cnt);
	dwc3_vtcpc->extcon = devm_kcalloc(dwc3_vtcpc->dev, dwc3_vtcpc->extcon_cnt,
					   sizeof(*dwc3_vtcpc->extcon), GFP_KERNEL);
	if (!dwc3_vtcpc->extcon)
		return -ENOMEM;
	memset(dwc3_vtcpc->extcon, 0, sizeof(*dwc3_vtcpc->extcon) * dwc3_vtcpc->extcon_cnt);

	for (idx = 0; idx < dwc3_vtcpc->extcon_cnt; idx++) {
		pr_err("%s: %d: extcon_get_edev_by_phandle times =%d \n", __func__, __LINE__, idx);
		edev = extcon_get_edev_by_phandle(dwc3_vtcpc->dev, idx);
		if (IS_ERR(edev) && PTR_ERR(edev) != -ENODEV) {
			pr_err("%s: %d: IS_ERR PTR_ERR times =%d \n", __func__, __LINE__, idx);
			return PTR_ERR(edev);
		}

		if (IS_ERR_OR_NULL(edev))
			continue;

		check_vbus_state = true;
		phandle_found = true;

		dwc3_vtcpc->extcon[idx].dwc3_vtcpc = dwc3_vtcpc;
		dwc3_vtcpc->extcon[idx].edev = edev;
		dwc3_vtcpc->extcon[idx].idx = idx;

		dwc3_vtcpc->extcon[idx].vbus_nb.notifier_call =
			dwc3_vtcpc_vbus_notifier;
		ret = extcon_register_notifier(edev, EXTCON_USB,
					       &dwc3_vtcpc->extcon[idx].vbus_nb);
		if (ret < 0)
			check_vbus_state = false;

		if (check_vbus_state && extcon_get_state(edev, EXTCON_USB))
			dwc3_vtcpc_vbus_notifier(&dwc3_vtcpc->extcon[idx].vbus_nb,
						  true, edev);
	}

	if (!phandle_found) {
		dev_err(dwc3_vtcpc->dev, "no extcon device found\n");
		return -ENODEV;
	}

	return 0;
}

static int dwc3_vtcpc_extcon_unregister(struct dwc3_vtcpc *dwc3_vtcpc)
{
	int idx;

	for (idx = 0; idx < dwc3_vtcpc->extcon_cnt; idx++) {
		extcon_unregister_notifier(dwc3_vtcpc->extcon[idx].edev, EXTCON_USB,
					  &dwc3_vtcpc->extcon[idx].vbus_nb);
	}
	return 0;
}
static int dwc3_vtcpc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct dwc3_vtcpc *vtcpc;
	int ret = 0;

	if (!np)
		return -EINVAL;
	pr_err("%s: %d: akingchen IN: %d\n", __func__, __LINE__, ret);

	vtcpc = devm_kzalloc(dev, sizeof(*vtcpc), GFP_KERNEL);
	if (!vtcpc)
		return -ENOMEM;

	vtcpc->dev = dev;
	vtcpc->edev = devm_extcon_dev_allocate(dev, usb_extcon_cable);
	if (IS_ERR_OR_NULL(vtcpc->edev)) {
		ret = PTR_ERR(vtcpc->edev);
		pr_err("%s: %d: allocate usb extcon device fail: %d\n", __func__, __LINE__, ret);
		goto err;
	}

	ret = devm_extcon_dev_register(dev, vtcpc->edev);
	if (ret < 0) {
		pr_err("%s: %d: register usb extcon device fail: %d\n", __func__, __LINE__, ret);
		goto err1;
	}

	vtcpc->typec_caps.type = TYPEC_PORT_SNK;
	vtcpc->typec_caps.data = TYPEC_PORT_UFP;
	vtcpc->typec_caps.revision = 0x0130;
	vtcpc->partner_desc.usb_pd = false;
	vtcpc->partner_desc.accessory = TYPEC_ACCESSORY_NONE;
	vtcpc->typec_port = typec_register_port(vtcpc->dev, &vtcpc->typec_caps);
	if (IS_ERR(vtcpc->typec_port)) {
		ret = PTR_ERR(vtcpc->typec_port);
		pr_err("%s: %d: failed to register typec_port: %d\n", __func__, __LINE__, ret);
		goto err2;
	}

	if (of_property_read_bool(np, "extcon")) {
		ret = dwc3_vtcpc_extcon_register(vtcpc);
		if (ret) {
			pr_err("%s: %d: fail to register extcon: %d\n",
				   __func__, __LINE__, ret);
			goto err3;
		}
	} else {
		pr_err("%s: %d: no extcon node\n", __func__, __LINE__);
		goto err3;
	}

	vtcpc->otg_tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	if (!vtcpc->otg_tcpc_dev) {
		ret = -EPROBE_DEFER;
		pr_info("%s get tcpc device type_c_port0 fail ret:%d \n", __func__, ret);
		goto err4;
	}

	vtcpc->otg_nb.notifier_call = otg_tcp_notifier_call;
	ret = register_tcp_dev_notifier(vtcpc->otg_tcpc_dev, &vtcpc->otg_nb,
		TCP_NOTIFY_TYPE_ALL);
	if (ret < 0) {
		pr_info("%s register tcpc notifer fail\n", __func__);
		goto err4;
	} else {
		pr_info("%s register tcpc notifer sucess\n", __func__);
	}

	mutex_init(&vtcpc->tcpc_otg_lock);
	vtcpc->data_role = TYPEC_PORT_UFP;
	g_dwc3_vtcpc = vtcpc;
	vusb_vote_register_callback(vote_typec_drp, get_typec_drp_voter_effective);
	platform_set_drvdata(pdev, vtcpc);
	pr_info("%s: %d: finish: %d\n", __func__, __LINE__, ret);

	return 0;
err4:
	dwc3_vtcpc_extcon_unregister(vtcpc);
err3:
	typec_unregister_port(vtcpc->typec_port);
err2:
err1:
err:
	return ret;
}

static int dwc3_vtcpc_remove(struct platform_device *pdev)
{
	struct dwc3_vtcpc *vtcpc = platform_get_drvdata(pdev);
	int idx;

	for (idx = 0; idx < vtcpc->extcon_cnt; idx++) {
		if (vtcpc->extcon[idx].edev)
			extcon_unregister_notifier(vtcpc->extcon[idx].edev, EXTCON_USB,
						   &vtcpc->extcon[idx].vbus_nb);
	}
	if (vtcpc->partner) {
		typec_unregister_partner(vtcpc->partner);
		vtcpc->partner = NULL;
	};
	typec_unregister_port(vtcpc->typec_port);
	unregister_tcp_dev_notifier(vtcpc->otg_tcpc_dev, &vtcpc->otg_nb,
		TCP_NOTIFY_TYPE_ALL);
	g_dwc3_vtcpc = NULL;
	return 0;
}

static const struct of_device_id dwc3_vtcpc_dt_ids[] = {
	{ .compatible = "v,dwc3-vtcpc", },
	{ },
};
MODULE_DEVICE_TABLE(of, dwc3_vtcpc_dt_ids);

static struct platform_driver dwc3_vtcpc_driver = {
	.probe		= dwc3_vtcpc_probe,
	.remove		= dwc3_vtcpc_remove,
	.driver		= {
		.name	= "dwc3_vtcpc",
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(dwc3_vtcpc_dt_ids),
	},
};

static int __init dwc3_vtcpc_init(void)
{
	return platform_driver_register(&dwc3_vtcpc_driver);
}

static void __init dwc3_vtcpc_exit(void)
{
	platform_driver_unregister(&dwc3_vtcpc_driver);
}

module_init(dwc3_vtcpc_init);
module_exit(dwc3_vtcpc_exit);

MODULE_DESCRIPTION("DWC3 VTCPC");
MODULE_LICENSE("GPL");


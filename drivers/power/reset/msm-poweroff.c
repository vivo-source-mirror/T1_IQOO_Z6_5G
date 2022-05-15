// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2013-2020, The Linux Foundation. All rights reserved.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>

#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/input/qpnp-power-on.h>
#include <linux/of_address.h>
#include <linux/qcom_scm.h>
#include <linux/nvmem-consumer.h>

#include <asm/cacheflush.h>
#include <asm/system_misc.h>
#include <asm/memory.h>

#include <soc/qcom/restart.h>
#include <soc/qcom/watchdog.h>
#include <soc/qcom/minidump.h>

#define EMERGENCY_DLOAD_MAGIC1    0x322A4F99
#define EMERGENCY_DLOAD_MAGIC2    0xC67E4350
#define EMERGENCY_DLOAD_MAGIC3    0x77777777
#define EMMC_DLOAD_TYPE		0x2

#define SCM_DLOAD_FULLDUMP		QCOM_DOWNLOAD_FULLDUMP
#define SCM_EDLOAD_MODE			QCOM_DOWNLOAD_EDL
#define SCM_DLOAD_MINIDUMP		QCOM_DOWNLOAD_MINIDUMP
#define SCM_DLOAD_BOTHDUMPS	(SCM_DLOAD_FULLDUMP | SCM_DLOAD_MINIDUMP)

#define DL_MODE_PROP "qcom,msm-imem-download_mode"
#define EDL_MODE_PROP "qcom,msm-imem-emergency_download_mode"
#define IMEM_DL_TYPE_PROP "qcom,msm-imem-dload-type"

#define KASLR_OFFSET_PROP "qcom,msm-imem-kaslr_offset"
#define KASLR_OFFSET_BIT_MASK	0x00000000FFFFFFFF

static int restart_mode;
static void *restart_reason, *dload_type_addr;
/* Download mode master kill-switch */
static void __iomem *msm_ps_hold;
static phys_addr_t tcsr_boot_misc_detect;
static struct nvmem_cell *nvmem_cell;

/*
 * Runtime could be only changed value once.
 * There is no API from TZ to re-enable the registers.
 * So the SDI cannot be re-enabled when it already by-passed.
 */
int download_mode;
EXPORT_SYMBOL(download_mode);
static int msm_restart_init_done;
static int download_mode_locked;
module_param(download_mode_locked, int, 0400);
static struct kobject dload_kobj;

static int in_panic;
static int dload_type = SCM_DLOAD_FULLDUMP;
static void *dload_mode_addr;
static bool dload_mode_enabled;
static void *emergency_dload_mode_addr;

static bool force_warm_reboot;

static struct notifier_block restart_nb;

extern void qpnp_pon_kpdpwr_resin_ex(int enable);

/* interface for exporting attributes */
struct reset_attribute {
	struct attribute        attr;
	ssize_t (*show)(struct kobject *kobj, struct attribute *attr,
			char *buf);
	size_t (*store)(struct kobject *kobj, struct attribute *attr,
			const char *buf, size_t count);
};
#define to_reset_attr(_attr) \
	container_of(_attr, struct reset_attribute, attr)
#define RESET_ATTR(_name, _mode, _show, _store)	\
	static struct reset_attribute reset_attr_##_name = \
			__ATTR(_name, _mode, _show, _store)

/* sysfs related globals */
static ssize_t show_emmc_dload(struct kobject *kobj, struct attribute *attr,
			       char *buf);
static size_t store_emmc_dload(struct kobject *kobj, struct attribute *attr,
			       const char *buf, size_t count);
RESET_ATTR(emmc_dload, 0644, show_emmc_dload, store_emmc_dload);

#ifdef CONFIG_QCOM_MINIDUMP
static ssize_t show_dload_mode(struct kobject *kobj, struct attribute *attr,
			       char *buf);
static size_t store_dload_mode(struct kobject *kobj, struct attribute *attr,
			       const char *buf, size_t count);
RESET_ATTR(dload_mode, 0644, show_dload_mode, store_dload_mode);
#endif /* CONFIG_QCOM_MINIDUMP */

static struct attribute *reset_attrs[] = {
	&reset_attr_emmc_dload.attr,
#ifdef CONFIG_QCOM_MINIDUMP
	&reset_attr_dload_mode.attr,
#endif
	NULL
};

static struct attribute_group reset_attr_group = {
	.attrs = reset_attrs,
};

static int dload_set(const char *val, const struct kernel_param *kp);
module_param_call(download_mode, dload_set, param_get_int,
			&download_mode, 0644);

/* zhengwei add for VIVO_VOLUME_UP_NO_CHECK start */
#define VIVO_VOLUME_UP_NO_CHECK_MAGIC_NUM    0x44504D66
int nokeyup_mode;

static int nokeyup_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret)
		return ret;

	if (nokeyup_mode > 1 || nokeyup_mode < 0)
		return -EINVAL;

	if (restart_reason) {
		__raw_writel(nokeyup_mode ? VIVO_VOLUME_UP_NO_CHECK_MAGIC_NUM : 0, restart_reason);
		mb();
	}

	return 0;
}

module_param_call(nokeyup_mode, nokeyup_set, param_get_int,
			&nokeyup_mode, 0644);
/* zhengwei add end */

static ssize_t attr_show(struct kobject *kobj, struct attribute *attr,
			       char *buf);
static ssize_t attr_store(struct kobject *kobj, struct attribute *attr,
			       const char *buf, size_t count);
static const struct sysfs_ops reset_sysfs_ops = {
	.show	= attr_show,
	.store	= attr_store,
};

static struct kobj_type reset_ktype = {
	.sysfs_ops	= &reset_sysfs_ops,
};

static int panic_prep_restart(struct notifier_block *this,
			      unsigned long event, void *ptr)
{
	in_panic = 1;
	return NOTIFY_DONE;
}

static struct notifier_block panic_blk = {
	.notifier_call	= panic_prep_restart,
};

static void set_dload_mode(int on)
{
#if !defined(VIVO_PROJECT_MODEL) || defined(VIVO_PROJECT_VMODEL)
	qpnp_pon_system_pwr_off(PON_POWER_OFF_WARM_RESET);
#else
	if (on) {
		qpnp_pon_system_pwr_off(PON_POWER_OFF_WARM_RESET);
	} else {
		qpnp_pon_system_pwr_off(PON_POWER_OFF_HARD_RESET);
	}
#endif

	if (dload_mode_addr) {
		__raw_writel(on ? 0xE47B337D : 0, dload_mode_addr);
		__raw_writel(on ? 0xCE14091A : 0,
		       dload_mode_addr + sizeof(unsigned int));
		/* Make sure the download cookie is updated */
		mb();
	}

	dload_type = on ? SCM_DLOAD_FULLDUMP : SCM_DLOAD_MINIDUMP;
	pr_err("%s on = %d dload_type = 0x%x\n", __func__, on, dload_type);
	qcom_scm_set_download_mode(dload_type, tcsr_boot_misc_detect ? : 0);

	dload_mode_enabled = on;
}

static bool get_dload_mode(void)
{
	return dload_mode_enabled;
}

#define DOWNLOAD_MODE_LOCKED_BIT_MASK (1<<1)

static int dload_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	int old_val = download_mode;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	if (download_mode_locked) {
		printk("[%s][%d] dload_set download_mode %d fail, download_mode_forced is set!\n",
				current->comm, current->pid, download_mode);
		download_mode = old_val;
		return 0;
	}

	/* If download_mode is not zero or one, ignore. */
	if (download_mode >> 1) {
		if (!msm_restart_init_done) {
			if (download_mode & DOWNLOAD_MODE_LOCKED_BIT_MASK) {
				download_mode_locked = 1;
				printk("download_mode_locked!\n");
			}
			download_mode &= 0x1;
		} else {
			download_mode = old_val;
			return -EINVAL;
		}
	}

	printk("[%s][%d] dload_set download_mode %d\n", current->comm, current->pid, download_mode);

	qpnp_pon_kpdpwr_resin_ex(download_mode);

	set_dload_mode(download_mode);

	return 0;
}

static void free_dload_mode_mem(void)
{
	iounmap(emergency_dload_mode_addr);
	iounmap(dload_mode_addr);
}

static void *map_prop_mem(const char *propname)
{
	struct device_node *np = of_find_compatible_node(NULL, NULL, propname);
	void *addr;

	if (!np) {
		pr_err("Unable to find DT property: %s\n", propname);
		return NULL;
	}

	addr = of_iomap(np, 0);
	if (!addr)
		pr_err("Unable to map memory for DT property: %s\n", propname);

	return addr;
}

#ifdef CONFIG_RANDOMIZE_BASE
static void *kaslr_imem_addr;
static void store_kaslr_offset(void)
{
	kaslr_imem_addr = map_prop_mem(KASLR_OFFSET_PROP);

	if (kaslr_imem_addr) {
		__raw_writel(0xdead4ead, kaslr_imem_addr);
		__raw_writel(KASLR_OFFSET_BIT_MASK &
		(kimage_vaddr - KIMAGE_VADDR), kaslr_imem_addr + 4);
		__raw_writel(KASLR_OFFSET_BIT_MASK &
			((kimage_vaddr - KIMAGE_VADDR) >> 32),
			kaslr_imem_addr + 8);
		iounmap(kaslr_imem_addr);
	}
}
#else
static void store_kaslr_offset(void)
{
}
#endif /* CONFIG_RANDOMIZE_BASE */

static void setup_dload_mode_support(void)
{
	int ret;

	atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);

	dload_mode_addr = map_prop_mem(DL_MODE_PROP);

	emergency_dload_mode_addr = map_prop_mem(EDL_MODE_PROP);

	store_kaslr_offset();

	dload_type_addr = map_prop_mem(IMEM_DL_TYPE_PROP);
	if (!dload_type_addr)
		return;

	ret = kobject_init_and_add(&dload_kobj, &reset_ktype,
			kernel_kobj, "%s", "dload");
	if (ret) {
		pr_err("%s:Error in creation kobject_add\n", __func__);
		kobject_put(&dload_kobj);
		return;
	}

	ret = sysfs_create_group(&dload_kobj, &reset_attr_group);
	if (ret) {
		pr_err("%s:Error in creation sysfs_create_group\n", __func__);
		kobject_del(&dload_kobj);
	}
}

static ssize_t attr_show(struct kobject *kobj, struct attribute *attr,
				char *buf)
{
	struct reset_attribute *reset_attr = to_reset_attr(attr);
	ssize_t ret = -EIO;

	if (reset_attr->show)
		ret = reset_attr->show(kobj, attr, buf);

	return ret;
}

static ssize_t attr_store(struct kobject *kobj, struct attribute *attr,
				const char *buf, size_t count)
{
	struct reset_attribute *reset_attr = to_reset_attr(attr);
	ssize_t ret = -EIO;

	if (reset_attr->store)
		ret = reset_attr->store(kobj, attr, buf, count);

	return ret;
}

static ssize_t show_emmc_dload(struct kobject *kobj, struct attribute *attr,
				char *buf)
{
	uint32_t read_val, show_val;

	if (!dload_type_addr)
		return -ENODEV;

	read_val = __raw_readl(dload_type_addr);
	if (read_val == EMMC_DLOAD_TYPE)
		show_val = 1;
	else
		show_val = 0;

	return snprintf(buf, sizeof(show_val), "%u\n", show_val);
}

static size_t store_emmc_dload(struct kobject *kobj, struct attribute *attr,
				const char *buf, size_t count)
{
	uint32_t enabled;
	int ret;

	if (!dload_type_addr)
		return -ENODEV;

	ret = kstrtouint(buf, 0, &enabled);
	if (ret < 0)
		return ret;

	if (!((enabled == 0) || (enabled == 1)))
		return -EINVAL;

	if (enabled == 1)
		__raw_writel(EMMC_DLOAD_TYPE, dload_type_addr);
	else
		__raw_writel(0, dload_type_addr);

	return count;
}

#ifdef CONFIG_QCOM_MINIDUMP
static DEFINE_MUTEX(tcsr_lock);

static ssize_t show_dload_mode(struct kobject *kobj, struct attribute *attr,
				char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "DLOAD dump type: %s\n",
		(dload_type == SCM_DLOAD_BOTHDUMPS) ? "both" :
		((dload_type == SCM_DLOAD_MINIDUMP) ? "mini" : "full"));
}

static size_t store_dload_mode(struct kobject *kobj, struct attribute *attr,
				const char *buf, size_t count)
{
	if (sysfs_streq(buf, "full")) {
		dload_type = SCM_DLOAD_FULLDUMP;
	} else if (sysfs_streq(buf, "mini")) {
		if (!msm_minidump_enabled()) {
			pr_err("Minidump is not enabled\n");
			return -ENODEV;
		}
		dload_type = SCM_DLOAD_MINIDUMP;
	} else if (sysfs_streq(buf, "both")) {
		if (!msm_minidump_enabled()) {
			pr_err("Minidump not enabled, setting fulldump only\n");
			dload_type = SCM_DLOAD_FULLDUMP;
			return count;
		}
		dload_type = SCM_DLOAD_BOTHDUMPS;
	} else {
		pr_err("Invalid Dump setup request..\n");
		pr_err("Supported dumps:'full', 'mini', or 'both'\n");
		return -EINVAL;
	}

	mutex_lock(&tcsr_lock);
	/*Overwrite TCSR reg*/
	set_dload_mode(dload_type);
	mutex_unlock(&tcsr_lock);
	return count;
}
#endif /* CONFIG_QCOM_MINIDUMP */

void msm_set_restart_mode(int mode)
{
	restart_mode = mode;
}
EXPORT_SYMBOL(msm_set_restart_mode);


static void msm_restart_prepare(const char *cmd)
{
	bool need_warm_reset = false;
	u8 reason = PON_RESTART_REASON_UNKNOWN;
	/* Write download mode flags if we're panic'ing
	 * Write download mode flags if restart_mode says so
	 * Kill download mode if master-kill switch is set
	 */

	if (cmd != NULL && !strcmp(cmd, "qcom_dload"))
		restart_mode = RESTART_DLOAD;

	set_dload_mode(download_mode &&
			(in_panic || restart_mode == RESTART_DLOAD));

	if (qpnp_pon_check_hard_reset_stored()) {
		/* Set warm reset as true when device is in dload mode */
		if (get_dload_mode() ||
			((cmd != NULL && cmd[0] != '\0') && !strncmp(cmd, "oem-", 4)))
			need_warm_reset = true;
	} else {
		need_warm_reset = (get_dload_mode() ||
				(cmd != NULL && cmd[0] != '\0'));
	}

#ifdef CONFIG_QCOM_MINIDUMP
#if !defined(VIVO_PROJECT_MODEL) || defined(VIVO_PROJECT_VMODEL)
	if (in_panic) {
		need_warm_reset = true;
	}
#endif
#endif

	if (force_warm_reboot)
		pr_info("Forcing a warm reset of the system\n");

	printk(KERN_ERR "%s %d need_warm_reset = %d get_dload_mode() = %d cmd = %s\n", __func__, __LINE__,
			need_warm_reset, get_dload_mode(), cmd);

	/* Hard reset the PMIC unless memory contents must be maintained. */
	if (force_warm_reboot || need_warm_reset)
		qpnp_pon_system_pwr_off(PON_POWER_OFF_WARM_RESET);
	else
		qpnp_pon_system_pwr_off(PON_POWER_OFF_HARD_RESET);

	if (cmd != NULL) {
		qpnp_pon_system_pwr_off(PON_POWER_OFF_HARD_RESET);
		if (!strncmp(cmd, "bootloader", 10)) {
			reason = PON_RESTART_REASON_BOOTLOADER;
			__raw_writel(0x77665500, restart_reason);
		// add by wuzengshun for silent & survival mode begin
		} else  if (!strncmp(cmd, "silent", 6)) {
			reason = PON_RESTART_REASON_BOOT_SILENT;
			pr_notice("Going restart boot-silent now\n");
			__raw_writel(0x77665504, restart_reason);
		} else  if (!strncmp(cmd, "survival", 8)) {
			reason = PON_RESTART_REASON_SURVIVAL;
			pr_notice("Going restart boot-survival now\n");
			__raw_writel(0x77665505, restart_reason);
		// add by wuzengshun for silent & survival mode end
		} else if (!strncmp(cmd, "recovery", 8)) {
			reason = PON_RESTART_REASON_RECOVERY;
			__raw_writel(0x77665502, restart_reason);
		} else if (!strcmp(cmd, "rtc")) {
			reason = PON_RESTART_REASON_RTC;
			__raw_writel(0x77665503, restart_reason);
		} else if (!strcmp(cmd, "dm-verity device corrupted")) {
			reason = PON_RESTART_REASON_DMVERITY_CORRUPTED;
			__raw_writel(0x77665508, restart_reason);
		} else if (!strcmp(cmd, "dm-verity enforcing")) {
			reason = PON_RESTART_REASON_DMVERITY_ENFORCE;
			__raw_writel(0x77665509, restart_reason);
		} else if (!strcmp(cmd, "keys clear")) {
			reason = PON_RESTART_REASON_KEYS_CLEAR;
			__raw_writel(0x7766550a, restart_reason);
		} else if (!strncmp(cmd, "oem-", 4)) {
			unsigned long code;
			int ret;

			ret = kstrtoul(cmd + 4, 16, &code);
			if (!ret)
				__raw_writel(0x6f656d00 | (code & 0xff),
					     restart_reason);
		} else {
			__raw_writel(0x77665501, restart_reason);
		}

		if (reason && nvmem_cell)
			nvmem_cell_write(nvmem_cell, &reason, sizeof(reason));
		else
			qpnp_pon_set_restart_reason(
				(enum pon_restart_reason)reason);
	}

	/*outer_flush_all is not supported by 64bit kernel*/
#ifndef CONFIG_ARM64
	outer_flush_all();
#endif

}

/*
 * Deassert PS_HOLD to signal the PMIC that we are ready to power down or reset.
 * Do this by calling into the secure environment, if available, or by directly
 * writing to a hardware register.
 *
 * This function should never return.
 */
static void deassert_ps_hold(void)
{
	qcom_scm_deassert_ps_hold();

	/* Fall-through to the direct write in case the scm_call "returns" */
	__raw_writel(0, msm_ps_hold);
}

#ifdef CONFIG_QGKI
static int pshold_reset;
static int force_pshold_reset(const char *val, const struct kernel_param *kp)
{
	pr_notice("Force a pshold reset\n");

	deassert_ps_hold();
	msleep(10000);
	pr_notice("Force a pshold reset failed\n");
	return 0;
}
module_param_call(pshold_reset, force_pshold_reset, param_get_int,
		&pshold_reset, 0644);

static int warm_reset;
static int force_warm_reset_get(char *buffer, const struct kernel_param *kp);
static int force_warm_reset_set(const char *buffer, const struct kernel_param *kp);
module_param_call(warm_reset, force_warm_reset_set, force_warm_reset_get,
						&warm_reset, 0644);

static int force_ps_hold_warm_reset(void)
{
	pr_notice("Force a ps_hold warm reset\n");
	/*	Set reset type to warm reset */
	qpnp_pon_system_pwr_off(PON_POWER_OFF_WARM_RESET);

	deassert_ps_hold();
	msleep(10000);
	pr_notice("Force a ps_hold warm reset failed\n");

	return 0;
}

#define SCM_SVC_SEC_WDOG_TRIG	0x8
static int force_tz_reset(void)
{
	int scm_ret = 0;

	printk("Force a tz warm reset\n");
	scm_ret = qcom_scm_sec_wdog_trigger();
	if (scm_ret) {
		printk("Force a tz warm reset failed, try wdog_bite reset\n");
		qcom_wdt_trigger_bite();
	}

	return scm_ret;
}

enum RESET_ROUTE {
	PS_HOLD_RESET = 1,
	TZ_WDOG_RESET = 2,
};

static int force_warm_reset_set(const char *buffer, const struct kernel_param *kp)
{
	int ret = 0;
	int old_val = warm_reset;

	ret = param_set_int(buffer, kp);
	if (ret)
		return ret;

	if (warm_reset == PS_HOLD_RESET) {
		printk("do ps_hold reset\n");
		force_ps_hold_warm_reset();
	} else if (warm_reset == TZ_WDOG_RESET) {
		printk("do tz reset\n");
		force_tz_reset();
	} else {
		warm_reset = old_val;
		printk("invalid parameters\n");
	}

	return 0;
}

static int force_warm_reset_get(char *buffer, const struct kernel_param *kp)
{
	int rc = 0;
	int max_lenth = 4096;

	rc = snprintf(buffer, max_lenth, "%s", "1: ps_hold reset;\n2: tz reset;\nOthers: invalid;\n");

	return rc;
}

static int watchdog_reset;
static int force_watchdog_reset(const char *val, const struct kernel_param *kp)
{
	pr_notice("Force a watchdog reset\n");
	/* Set reset type to warm reset */
	qpnp_pon_system_pwr_off(PON_POWER_OFF_WARM_RESET);

	qcom_wdt_trigger_bite();
	msleep(10000);
	pr_notice("Force a watchdog reset failed\n");
	return 0;
}
module_param_call(watchdog_reset, force_watchdog_reset, param_get_int,
		&watchdog_reset, 0644);
#endif


static int do_msm_restart(struct notifier_block *unused, unsigned long action,
			   void *arg)
{
	const char *cmd = arg;

	pr_err("Going down for restart now\n");

	msm_restart_prepare(cmd);

	deassert_ps_hold();

	msleep(10000);

	return NOTIFY_DONE;
}

static void do_msm_poweroff(void)
{
	pr_err("Powering off the SoC\n");

	set_dload_mode(0);
	qpnp_pon_system_pwr_off(PON_POWER_OFF_SHUTDOWN);

	deassert_ps_hold();

	msleep(10000);
	pr_err("Powering off has failed\n");
}

static int msm_restart_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *mem;
	struct device_node *np;
	int ret = 0;

	nvmem_cell = devm_nvmem_cell_get(dev, "restart_reason");
	if (PTR_ERR(nvmem_cell) == -EPROBE_DEFER)
		return PTR_ERR(nvmem_cell);
	else if (IS_ERR_VALUE(nvmem_cell))
		nvmem_cell = NULL;

	setup_dload_mode_support();

	np = of_find_compatible_node(NULL, NULL,
				"qcom,msm-imem-restart_reason");
	if (!np) {
		pr_err("unable to find DT imem restart reason node\n");
	} else {
		restart_reason = of_iomap(np, 0);
		if (!restart_reason) {
			pr_err("unable to map imem restart reason offset\n");
			ret = -ENOMEM;
			goto err_restart_reason;
		}
	}

	mem = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pshold-base");
	msm_ps_hold = devm_ioremap_resource(dev, mem);
	if (IS_ERR(msm_ps_hold))
		return PTR_ERR(msm_ps_hold);

	mem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "tcsr-boot-misc-detect");
	if (mem)
		tcsr_boot_misc_detect = mem->start;

	pm_power_off = do_msm_poweroff;
	restart_nb.notifier_call = do_msm_restart;
	restart_nb.priority = 200;
	register_restart_handler(&restart_nb);

	set_dload_mode(download_mode);
#if 0
	if (!download_mode)
		qcom_scm_disable_sdi();
#endif
	force_warm_reboot = of_property_read_bool(dev->of_node,
						"qcom,force-warm-reboot");

	printk(KERN_ERR "%s %d download_mode:%d\n", __func__, __LINE__, download_mode);

	return 0;

err_restart_reason:
	free_dload_mode_mem();
	return ret;
}

static const struct of_device_id of_msm_restart_match[] = {
	{ .compatible = "qcom,pshold", },
	{},
};
MODULE_DEVICE_TABLE(of, of_msm_restart_match);

static struct platform_driver msm_restart_driver = {
	.probe = msm_restart_probe,
	.driver = {
		.name = "msm-restart",
		.of_match_table = of_match_ptr(of_msm_restart_match),
	},
};

static int __init msm_restart_init(void)
{
	msm_restart_init_done = 1;
	return platform_driver_register(&msm_restart_driver);
}

#if IS_MODULE(CONFIG_POWER_RESET_MSM)
module_init(msm_restart_init);
#else
pure_initcall(msm_restart_init);
#endif

static __exit void msm_restart_exit(void)
{
	platform_driver_unregister(&msm_restart_driver);
}
module_exit(msm_restart_exit);

MODULE_DESCRIPTION("MSM Poweroff Driver");
MODULE_LICENSE("GPL v2");

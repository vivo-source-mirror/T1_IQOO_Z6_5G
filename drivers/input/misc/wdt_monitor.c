#include "wdt_monitor.h"

/*===========================================================================
**  Variable :  enviroment variable
** ==========================================================================*/
static unsigned int is_recovery_mode;
static unsigned int is_survival_mode;
static unsigned int is_bootsilent_mode;
static unsigned int is_boot_bsptmode;
static unsigned int is_vivologging_mode;
static unsigned int is_atboot_mode;
static unsigned int is_debugbuild_mode;
static unsigned int is_normal_boot;
static unsigned int power_off_charging_mode;


static const char *recoverymode_cmdline = "recoverymode";
static const char *survivalmode_cmdline = "survivalmode";
static const char *bootsilent_cmdline   = "boot_silent=1";
static const char *boot_bsptmode_cmdline   = "boot_bsptmode=1";
static const char *charger_boot_cmdline = "androidboot.mode=charger";
static const char *vivolog_cmdline = "vivolog_flag=";
static const char *atboot_cmdline = "console-at=";
static const char *userbuild_cmdline = "buildvariant=user";

/*===========================================================================
**  Function :  kme sys common interface
** ==========================================================================*/
#define MAX_ENV_LEN 256
#define TAG "[kernel_monitor_engine]"

static int enable_kme = 1;

struct kobject *kme_kobj;
static struct work_struct async_work;
static char **g_envp_ext;
static spinlock_t g_lock;

/*
 * report all kinds of bottlenecks to user space through uevent
 * @argument:@envp_ext : a string array to send to userspace
 * the report format should be like this:
 * char *envp_ext[] ={
 * 	"subsystem=kernel_monitor_engine",
 * 	"exception=rt_throttling",
 *	"process=process_name",
 *	"processid = pid",
 * 	NULL,
 * 	};
 */
int kernel_monitor_report(char *envp_ext[])
{
	unsigned long flags;

	spin_lock_irqsave(&g_lock, flags);
	g_envp_ext = (char **)envp_ext;
	schedule_work(&async_work);
	spin_unlock_irqrestore(&g_lock, flags);
	return 0;
}
EXPORT_SYMBOL(kernel_monitor_report);

static ssize_t enable_monitor_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return snprintf(buf, 2, "%d\n", enable_kme);
}

static ssize_t enable_monitor_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int ret;

	ret = kstrtoint(buf, 10, &enable_kme);
	if (ret < 0)
		return ret;

	return count;
}

/* Sysfs attributes cannot be world-writable. */
static struct kobj_attribute enable_kme_attribute =
__ATTR(enable_kme, 0664, enable_monitor_show, enable_monitor_store);

/*
 * Create a group of attributes so that we can create and destroy them all
 * at once.
 */
static struct attribute *kme_attrs[] = {
	&enable_kme_attribute.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group kme_attr_group = {
	.attrs = kme_attrs,
};

int  kme_merge_group(const struct attribute_group *group)
{
	if (kme_kobj) {
		return sysfs_create_group(kme_kobj, group);
	}
	return 0;
}

void  kme_remove_group(const struct attribute_group *group)
{
	if (kme_kobj) {
		return sysfs_remove_group(kme_kobj, group);
	}
}

void async_func(struct work_struct *work)
{
	if (enable_kme) {
		kobject_uevent_env(kme_kobj, KOBJ_CHANGE, g_envp_ext);
	}
}

int kme_sysfs_init(void)
{
	int retval = 0;

	kme_kobj = kobject_create_and_add("kme", kernel_kobj);
	if (!kme_kobj)
		return -ENOMEM;

	retval = kme_merge_group(&kme_attr_group);
	if (retval)
		kobject_put(kme_kobj);

	spin_lock_init(&g_lock);
	INIT_WORK(&async_work, async_func);

	return retval;
}

/*===========================================================================
**  Function :  wdt_monitor common interface
** ==========================================================================*/
enum BOOT_STAGE {
	B_NONE			= 0x00,
	B_BOOTLOADER	= 0x01,
	B_KERNEL		= 0x02,
	B_FRAMEWORK 	= 0x03,
};

enum WDT_CMD_CTL {
	CMD_STOP				= 0,
	CMD_REBOOT				= -1,
	CMD_FORCE_OFFLINE_ON	= -101,
	CMD_FORCE_OFFLINE_OFF	= -100,
};

static bool wdt_force_offline;
//module_param_named(wdt_force_offline, wdt_force_offline, bool, 0644);

extern int kme_merge_group(const struct attribute_group *);
extern void kme_remove_group(const struct attribute_group *);

static long enable_monitor;
static int boot_stage_flags = B_NONE;
static int max_crash_cnt_limit = 20;
static DEFINE_MUTEX(g_wdt_mutex_lock);

static int vivo_wdt_monitor_enable(void);
static int vivo_wdt_monitor_disable(void);
static int vivo_wdt_monitor_cfg(long timeout);

int vivo_wdt_monitor_pet(void);
int vivo_wdt_monitor_start(long timeout);
int vivo_wdt_monitor_start_kernel(long timeout);
int vivo_wdt_monitor_stop(void);
int vivo_wdt_monitor_reboot(void);
void vivo_wdt_monitor_setup(void);
int vivo_wdt_monitor_force_offline(bool force_offline);

/*common ops of wdt monitor module*/
struct wdt_monitor_ops {
	int (*wdt_enable) (void);
	int (*wdt_disable) (void);
	int (*wdt_pet) (void);
	int (*wdt_cfg) (long timeout);
	int (*wdt_reboot) (void);
	int (*wdt_force_offline)(bool offline);
};

/*setup ops*/
static struct wdt_monitor_ops *wdt_monitor_common_ops;
static void vivo_wdt_monitor_ops_setup(struct wdt_monitor_ops *ops)
{
	wdt_monitor_common_ops = ops;
}

/*enable wdt*/
static int vivo_wdt_monitor_enable(void)
{
	int rc = 0;

	if (!wdt_monitor_common_ops)
		return 0;

	if (wdt_monitor_common_ops->wdt_enable)
		rc = wdt_monitor_common_ops->wdt_enable();

	return rc;
}

/*disable wdt*/
static int vivo_wdt_monitor_disable(void)
{
	int rc = 0;

	if (!wdt_monitor_common_ops)
		return 0;

	if (wdt_monitor_common_ops->wdt_disable)
		rc = wdt_monitor_common_ops->wdt_disable();

	return rc;
}

/*cfg wdt timeout*/
static int vivo_wdt_monitor_cfg(long timeout)
{
	int rc = 0;

	if (!wdt_monitor_common_ops)
		return 0;

	if (wdt_monitor_common_ops->wdt_cfg)
		rc = wdt_monitor_common_ops->wdt_cfg(timeout);

	return rc;
}

/*force_offline wdt*/
int vivo_wdt_monitor_force_offline(bool force_offline)
{
	int rc = 0;

	if (!wdt_monitor_common_ops)
		return 0;

	if (wdt_monitor_common_ops->wdt_force_offline)
		rc = wdt_monitor_common_ops->wdt_force_offline(force_offline);

	return rc;
}

/*start monitor*/
int vivo_wdt_monitor_start(long timeout)
{
	int rc = 0;

	mutex_lock(&g_wdt_mutex_lock);
	if (wdt_force_offline) {
		printk("wdt_monitor state: force_offline, return!\n");
		enable_monitor = 0;
		goto unlock;
	}

	printk("wdt_monitor_start: 0x%02x|%ld\n",
				boot_stage_flags, enable_monitor);

	rc = vivo_wdt_monitor_disable();
	rc = vivo_wdt_monitor_cfg(timeout);
	rc = vivo_wdt_monitor_enable();

unlock:
	mutex_unlock(&g_wdt_mutex_lock);
	return rc;
}

/*start monitor from kernel*/
int vivo_wdt_monitor_start_kernel(long timeout)
{
	int rc = 0;

	if (wdt_force_offline) {
		printk("wdt_monitor state: force_offline, return!\n");
		enable_monitor = 0;
		return rc;
	}

	boot_stage_flags = B_KERNEL;
	enable_monitor = timeout;
	rc = vivo_wdt_monitor_start(timeout);

	return rc;
}

/*stop monitor*/
int vivo_wdt_monitor_stop(void)
{
	int rc = 0;

	mutex_lock(&g_wdt_mutex_lock);
	if (wdt_force_offline) {
		printk("wdt_monitor state: force_offline, return!\n");
		goto unlock;
	}

	printk("wdt_monitor_stop: 0x%02x|%ld\n",
				boot_stage_flags, enable_monitor);

	boot_stage_flags = B_NONE;
	rc = vivo_wdt_monitor_disable();
	enable_monitor = 0;

unlock:
	mutex_unlock(&g_wdt_mutex_lock);
	return rc;
}

/*pet wdt*/
int vivo_wdt_monitor_pet(void)
{
	int rc = 0;

	mutex_lock(&g_wdt_mutex_lock);
	if (wdt_force_offline) {
		printk("wdt_monitor state: force_offline, return!\n");
		goto unlock;
	}

	printk("wdt_monitor_pet...\n");
	if (!wdt_monitor_common_ops)
		goto unlock;

	if (wdt_monitor_common_ops->wdt_pet)
		rc = wdt_monitor_common_ops->wdt_pet();

unlock:
	mutex_unlock(&g_wdt_mutex_lock);
	return rc;
}

/*reboot system*/
int vivo_wdt_monitor_reboot(void)
{
	int rc = 0;

	mutex_lock(&g_wdt_mutex_lock);
	printk("wdt_monitor_reboot...\n");
	if (!wdt_monitor_common_ops)
		goto unlock;

	if (wdt_monitor_common_ops->wdt_reboot)
		rc = wdt_monitor_common_ops->wdt_reboot();

unlock:
	mutex_unlock(&g_wdt_mutex_lock);
	return rc;
}

/*sys/kernel/kme/wdt/enable_monitor*/
static ssize_t enable_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%ld\n", enable_monitor);
}

/*sys/kernel/kme/wdt/enable_monitor*/
static ssize_t enable_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t count)
{
	int ret = 0;
	long temp_enable = 0;
	long old_enable_monitor = enable_monitor;

	if (power_off_charging_mode)
		return count;

	if (is_normal_boot == 0)
		return count;

	/*validity check*/
	ret = kstrtol(buf, 10, &temp_enable);
	if (ret)
		return ret;

	/*take this value*/
	enable_monitor = temp_enable;

	/*Set flags to be framework*/
	boot_stage_flags = B_FRAMEWORK;

	/*reboot immediately if -1 or cnt excced*/
	if ((enable_monitor == CMD_REBOOT) ||
		(max_crash_cnt_limit == 0)) {
		vivo_wdt_monitor_reboot();
		return count;
	}

	/*force_offline ON, will take effect at next boot*/
	if (enable_monitor == CMD_FORCE_OFFLINE_ON) {
		vivo_wdt_monitor_force_offline(1);
		return count;
	}

	/*force_online OFF, will take effect at next boot*/
	if (enable_monitor == CMD_FORCE_OFFLINE_OFF) {
		vivo_wdt_monitor_force_offline(0);
		return count;
	}

	/*stop monitor if 0*/
	if (enable_monitor == CMD_STOP) {
		vivo_wdt_monitor_stop();
		return count;
	}

	/*invalid param*/
	if (enable_monitor < 0) {
		enable_monitor = old_enable_monitor;
		boot_stage_flags = B_NONE;
		return -EINVAL;
	}

	/*start monitor if larger than 0*/
	max_crash_cnt_limit--;
	vivo_wdt_monitor_start(enable_monitor);
	return count;
}

/*create node /sys/kernel/kme/wdt*/
static struct kobj_attribute enable_attr =
	__ATTR(enable_monitor, 0640, enable_show, enable_store);

static struct attribute *wdt_attrs[] = {
	&enable_attr.attr,
	NULL,
};

static struct attribute_group wdt_attr_group = {
	.name = "wdt",
	.attrs = wdt_attrs,
};

/*===========================================================================
**  Function :  qcom platform interface
** ==========================================================================*/
#define WDT_MIN_TIME (1)
#define WDT_MAX_TIME (120)
#define WDT_MAXIMUN_MARGIN (20)

#define VIVO_WDT_FORCE_OFFLINE_BIT BIT(6)
#define VIVO_WDT_TEST_MODE_BIT BIT(5)
#define VIVO_WDT_STATUS_FLAGS_BIT 0x0F

static int wdt_maximum_time;
static int wdt_minimum_time;
static int wdt_interval_time;

static int wdt_pet_interval;
static int wdt_pet_cylcle;
static int wdt_pet_remainder;
static struct timer_list wdt_pet_timer;
static int qpnp_wd_initialized;

static const char * const boot_state[] = {
	[B_NONE] = "None",
	[B_BOOTLOADER] = "pmic_wdt(Bootlader|reboot,system_block)",
	[B_KERNEL] = "pmic_wdt(Kernel|reboot,system_block)",
	[B_FRAMEWORK] = "pmic_wdt(Framework|reboot,system_block)",
};

/*test_mode_ctl for wdt*/
static int wdt_test_mode_ctl;
static int wdt_test_mode_ctl_set(const char *val, const struct kernel_param *kp);
module_param_call(wdt_test_mode_ctl, wdt_test_mode_ctl_set, param_get_int,
			&wdt_test_mode_ctl, 0644);

int wdt_test_mode_ctl_set_func(bool mode)
{
	int rc = 0;
	u16 addr;
	struct qpnp_pon *pon = sys_reset_dev;

	if (!pon)
		return -ENODEV;

	addr = QPNP_PON_XVDD_RB_SPARE(pon);
	rc = qpnp_pon_masked_write(pon, addr,
			VIVO_WDT_TEST_MODE_BIT, mode ? VIVO_WDT_TEST_MODE_BIT : 0);

	//rc = qpnp_pon_wd_enable(0, 0);
	return rc;
}

static int wdt_test_mode_ctl_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	int old_val = wdt_test_mode_ctl;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	if (wdt_test_mode_ctl == 1)
		wdt_test_mode_ctl_set_func(1);
	else if (wdt_test_mode_ctl == 0)
		wdt_test_mode_ctl_set_func(0);
	else
		wdt_test_mode_ctl = old_val;

	return 0;
}

int qpnp_pon_wd_set_boot_stage_flags(u8 flags)
{
	int rc = 0;
	u16 addr;
	struct qpnp_pon *pon = sys_reset_dev;

	if (!pon)
		return -ENODEV;

	addr = QPNP_PON_XVDD_RB_SPARE(pon);
	rc = qpnp_pon_masked_write(pon, addr, VIVO_WDT_STATUS_FLAGS_BIT, flags);

	return rc;
}

int qpnp_pon_wd_pet(void)
{
	struct qpnp_pon *pon = sys_reset_dev;

	if (!pon)
		return -ENODEV;

	return qpnp_pon_masked_write(pon,
				QPNP_PON_WD_RST_PET(pon), BIT(0), 1);
}

int qpnp_pon_wd_enable(bool enable, u8 flags)
{
	int rc = 0;
	int retry = 5;
	struct qpnp_pon *pon = sys_reset_dev;

	if (!pon)
		return -ENODEV;

	qpnp_pon_wd_set_boot_stage_flags(flags);
	do {
		rc = qpnp_pon_wd_config(!!enable);
	} while (rc && retry--);

	if (rc)
		dev_err(pon->dev,
				"Failed to enable pmic_wd wdt to [%d]\n", enable);
	else
		dev_err(pon->dev,
				"Success to enable pmic_wd wdt to [%d]\n", enable);

	return rc;
}

int qpnp_pon_wd_cfg_timer(u8 s1_timer, u8 s2_timer)
{
	int rc = 0;
	u16 addr;
	struct qpnp_pon *pon = sys_reset_dev;

	if (!pon)
		return -ENODEV;

	if (s1_timer > 127 || s1_timer < 0)
		return -EINVAL;

	if (s2_timer > 127 || s2_timer < 0)
		return -EINVAL;

	if (s2_timer == 0)
		s2_timer = 1;

	/*first disable watchdog*/
	rc = qpnp_pon_wd_config(0);

	/*config the s1 timer*/
	addr = QPNP_PON_WD_RST_S1_TIMER(pon);
	rc = qpnp_pon_masked_write(pon, addr,
				0xFF, s1_timer);

	if (rc) {
		dev_err(pon->dev,
				"Failed to set pmic_wd s1 timer, rc=%d\n", rc);
		return rc;
	}

	/*config the s2 timer*/
	addr = QPNP_PON_WD_RST_S2_TIMER(pon);
	rc = qpnp_pon_masked_write(pon, addr,
				0xFF, s2_timer);

	if (rc) {
		dev_err(pon->dev,
				"Failed to set pmic_wd s2 timer, rc=%d\n", rc);
		return rc;
	}

	/*config the reset type*/
	addr = QPNP_PON_WD_RST_S2_CTL(pon);
	rc = qpnp_pon_masked_write(pon, addr,
				0x0F, PON_POWER_OFF_HARD_RESET); /*dvdd hard reset*/

	if (rc) {
		dev_err(pon->dev,
				"Failed to set pmic_wd reset_type to hard, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

int qpnp_pon_wd_set_force_offline(bool force_offline)
{
	int rc = 0;
	u16 addr;
	struct qpnp_pon *pon = sys_reset_dev;

	if (!pon)
		return -ENODEV;

	addr = QPNP_PON_XVDD_RB_SPARE(pon);
	rc = qpnp_pon_masked_write(pon, addr,
			VIVO_WDT_FORCE_OFFLINE_BIT, force_offline ? VIVO_WDT_FORCE_OFFLINE_BIT : 0);

	if (force_offline)
		rc = qpnp_pon_wd_enable(0, 0);

	return rc;
}

int qpnp_pon_wd_get_force_offline(void)
{
	int rc = 0;
	u16 addr;
	int wdt_force_offline_value = 0;
	struct qpnp_pon *pon = sys_reset_dev;

	if (!pon)
		return -ENODEV;

	/*if force_offline bit set, return true*/
	addr = QPNP_PON_XVDD_RB_SPARE(pon);
	rc = qpnp_pon_read(pon, addr, &wdt_force_offline_value);
	if (rc)
		return true;

	if (wdt_force_offline_value & VIVO_WDT_FORCE_OFFLINE_BIT)
		return true;

	/*otherwise return false*/
	return false;
}

int qpnp_pon_wd_init(void)
{
	int rc = 0;
	u16 addr;
	struct qpnp_pon *pon = sys_reset_dev;

	if (!pon) {
		qpnp_wd_initialized = 0;
		return -ENODEV;
	}

	addr = QPNP_PON_WD_RST_S2_CTL2(pon);
	rc = qpnp_pon_read(pon, addr, &qpnp_wd_initialized);

	if ((qpnp_wd_initialized & QPNP_PON_WD_EN) == 0) {
		qpnp_wd_initialized = 0;
	} else
		qpnp_wd_initialized = 1;

	qpnp_pon_wd_enable(0, 0);
	return qpnp_wd_initialized;
}

void pmic_wdt_dump_debug_info(void)
{
	//show_state_filter(TASK_UNINTERRUPTIBLE);
	dump_stack();
	printk("%s: 0x%02x|%s\n", __func__, boot_stage_flags, boot_state[boot_stage_flags]);
	mdelay(50);
}

int pmic_wdt_set_emergency_reset(void)
{
	int rc = 0;

	rc = qpnp_pon_wd_cfg_timer(0, 0);
	rc = qpnp_pon_wd_enable(1, boot_stage_flags);

	return rc;
}

/*pmic pet timer start*/
void pmic_wdt_pet_timer_start(void)
{
	mod_timer(&wdt_pet_timer, jiffies + wdt_pet_interval*HZ);

	printk("%s: wdt_maximum_time:%d wdt_interval_time:%d\n",
						__func__, wdt_maximum_time, wdt_pet_interval);

	printk("%s: wdt_pet_cylcle:%d wdt_pet_remainder:%d\n",
						__func__, wdt_pet_cylcle, wdt_pet_remainder);
}

/*pmic pet timer stop*/
void pmic_wdt_pet_timer_stop(void)
{
	if (timer_pending(&wdt_pet_timer))
		del_timer_sync(&wdt_pet_timer);

	printk("%s: wdt_pet_cylcle:%d wdt_pet_remainder:%d\n",
						__func__, wdt_pet_cylcle, wdt_pet_remainder);
}

/*pmic pet timer function*/
void pmic_wdt_pet_timer_fn(struct timer_list *unused)
{
	if (wdt_pet_cylcle)
		wdt_pet_cylcle = wdt_pet_cylcle - 1;

	printk("%s: 0x%02x|wdt_pet_cylcle:%d wdt_pet_remainder:%d\n",
				__func__, boot_stage_flags, wdt_pet_cylcle, wdt_pet_remainder);

	if (wdt_pet_cylcle > 0) {
		wdt_pet_interval = wdt_interval_time;
		goto keepalive;
	}

	if (wdt_pet_remainder > wdt_minimum_time) {
		wdt_pet_interval = wdt_pet_remainder;
		wdt_pet_remainder = 0;
		goto keepalive;
	} else {
		if (power_off_charging_mode == 1)
			return;

		pmic_wdt_dump_debug_info();
		pmic_wdt_set_emergency_reset();
		return;
	}

keepalive:
	qpnp_pon_wd_pet();
	mod_timer(&wdt_pet_timer, jiffies + wdt_pet_interval*HZ);
	return;
}

/*pmic wdt timer setup*/
void pmic_wdt_timer_setup(void)
{
	timer_setup(&wdt_pet_timer, pmic_wdt_pet_timer_fn, 0);
}

/*pmic wdt enable*/
int pmic_wdt_enable(void)
{
	int rc = 0;

	printk("%s: 0x%02x|%ld\n", __func__, boot_stage_flags, enable_monitor);
	rc = qpnp_pon_wd_enable(1, boot_stage_flags);

	pmic_wdt_pet_timer_start();

	return rc;
}

/*pmic wdt disable*/
int pmic_wdt_disable(void)
{
	int rc = 0;

	/*auto set if boot_stage_flags = 0*/
	printk("%s: 0x%02x|%ld\n", __func__, boot_stage_flags, enable_monitor);
	rc = qpnp_pon_wd_enable(0, B_NONE);
	pmic_wdt_pet_timer_stop();

	wdt_pet_cylcle = 0;
	wdt_pet_remainder = 0;

	return rc;
}

/*pmic wdt pet*/
int pmic_wdt_pet(void)
{
	int rc = 0;

	printk("%s: 0x%02x|%ld\n", __func__, boot_stage_flags, enable_monitor);
	rc = qpnp_pon_wd_pet();

	return rc;
}

/*pmic wdt cfg*/
int pmic_wdt_cfg(long timeout)
{
	int rc = 0;

	if (timeout < wdt_minimum_time)
		return -EINVAL;

	wdt_pet_cylcle = timeout/wdt_interval_time;
	wdt_pet_remainder = timeout%wdt_interval_time;

	if (wdt_pet_cylcle > 0) {
		wdt_pet_interval = wdt_interval_time;
	} else {
		wdt_pet_interval = timeout;
		wdt_pet_remainder = wdt_minimum_time;
		wdt_pet_cylcle = 1;
	}

	printk("%s: 0x%02x|%ld\n", __func__, boot_stage_flags, timeout);
	rc = qpnp_pon_wd_cfg_timer(wdt_maximum_time, 0);

	return rc;
}

/*pmic wdt reboot*/
int pmic_wdt_reboot(void)
{
	int rc = 0;

	/*do immediately low-level reboot*/
	printk("%s: 0x%02x|%ld\n", __func__, boot_stage_flags, enable_monitor);

	pmic_wdt_dump_debug_info();
	rc = pmic_wdt_set_emergency_reset();

	return rc;
}

/*pmic wdt force_offline*/
int pmic_wdt_force_offline(bool force_offline)
{
	int rc = 0;

	/*force_offline*/
	printk("%s: 0x%02x|%ld\n", __func__, boot_stage_flags, enable_monitor);
	if (force_offline) {
		printk("pmic_wdt force_offline true from remote!\n");
		wdt_force_offline = force_offline;
		rc = qpnp_pon_wd_set_force_offline(1);
		rc = pmic_wdt_disable();
	} else {
		printk("pmic_wdt force_offline false from remote!\n");
		rc = qpnp_pon_wd_set_force_offline(0);
	}

	return rc;
}

/*pmic wdt ops*/
struct wdt_monitor_ops qcom_pmic_wdt_ops = {
	.wdt_enable = pmic_wdt_enable,
	.wdt_disable = pmic_wdt_disable,
	.wdt_pet = pmic_wdt_pet,
	.wdt_cfg = pmic_wdt_cfg,
	.wdt_reboot = pmic_wdt_reboot,
	.wdt_force_offline = pmic_wdt_force_offline,
};

static bool __vivo_wdt_setup_done;
void vivo_wdt_monitor_setup(void)
{
	if (__vivo_wdt_setup_done)
		return;

	wdt_minimum_time = WDT_MIN_TIME;
	wdt_maximum_time = WDT_MAX_TIME;
	if (wdt_maximum_time > WDT_MAXIMUN_MARGIN) {
		wdt_interval_time = wdt_maximum_time - WDT_MAXIMUN_MARGIN;
	} else {
		wdt_interval_time = 1;
	}
	printk("%s: wdt_maximum_time:%d wdt_interval_time:%d\n",
						__func__, wdt_maximum_time, wdt_interval_time);

	pmic_wdt_timer_setup();
	vivo_wdt_monitor_ops_setup(&qcom_pmic_wdt_ops);

	__vivo_wdt_setup_done = true;
}

/*===========================================================================
**  Function :  enviroment variable parser
** ==========================================================================*/
static __always_inline size_t strings_has_prefix(const char *str, const char *prefix)
{
	size_t len = strlen(prefix);
	return strncmp(str, prefix, len) == 0 ? len : 0;
}

static int bootmode_init(void)
{
	const char *command_line = NULL;
	struct device_node *of_dtb_chosen;
	bool force_offline_bit = 0;
	char *str = NULL;

	is_normal_boot = 0;
	of_dtb_chosen = of_find_node_opts_by_path("/chosen", NULL);
	if (!of_dtb_chosen) {
		printk(KERN_ERR "/chosen cannot be found\n");
		return -EINVAL;
	}

	if (of_property_read_string(of_dtb_chosen, "bootargs", &command_line)) {
		printk(KERN_ERR "bootargs read fail\n");
		return -EINVAL;
	}

	if (!command_line) {
		printk(KERN_ERR "command_line is null\n");
		return -EFAULT;
	}

	if (strnstr(command_line, recoverymode_cmdline, strlen(command_line))) {
		is_recovery_mode = 1;
	} else {
		is_recovery_mode = 0;
	}

	if (strnstr(command_line, survivalmode_cmdline, strlen(command_line))) {
		is_survival_mode = 1;
	} else {
		is_survival_mode = 0;
	}

	if (strnstr(command_line, bootsilent_cmdline, strlen(command_line))) {
		is_bootsilent_mode = 1;
	} else {
		is_bootsilent_mode = 0;
	}

	if (strnstr(command_line, boot_bsptmode_cmdline, strlen(command_line))) {
		is_boot_bsptmode = 1;
	} else {
		is_boot_bsptmode = 0;
	}

	if (strnstr(command_line, userbuild_cmdline, strlen(command_line))) {
		is_debugbuild_mode = 0;
	} else {
		is_debugbuild_mode = 1;
	}

	str = strnstr(command_line, vivolog_cmdline, strlen(command_line));
	if (str == NULL) {
		is_vivologging_mode = 0;
	} else {
		str += strlen(vivolog_cmdline);
		sscanf(str, "%d", &is_vivologging_mode);
	}

	str = strnstr(command_line, atboot_cmdline, strlen(command_line));
	if (str == NULL) {
		is_atboot_mode = 0;
	} else {
		str += strlen(atboot_cmdline);
		if (strings_has_prefix(str, "null") ||
				strings_has_prefix(str, "NULL") ||
				strings_has_prefix(str, "sendFastboot"))
			is_atboot_mode = 0;
		else
			is_atboot_mode = 1;
	}

	if (strnstr(command_line, charger_boot_cmdline, strlen(command_line))) {
		power_off_charging_mode = 1;
	} else {
		power_off_charging_mode = 0;
	}

	if (is_recovery_mode ||
			is_survival_mode ||
			is_bootsilent_mode ||
			is_boot_bsptmode ||
			is_atboot_mode ||
			is_debugbuild_mode ||
			is_vivologging_mode ||
			power_off_charging_mode) {
		is_normal_boot = 0;
	} else
		is_normal_boot = 1;

	if (is_normal_boot == 0)
		wdt_force_offline = 1;

	force_offline_bit = qpnp_pon_wd_get_force_offline();
	if (force_offline_bit || (qpnp_wd_initialized == 0))
		wdt_force_offline = 1;

	printk("wdt_monitor: is_recovery_mode:%d, is_survival_mode:%d\n",
							is_recovery_mode, is_survival_mode);

	printk("wdt_monitor: is_bootsilent_mode:%d, power_off_charging_mode:%d\n",
							is_bootsilent_mode, power_off_charging_mode);

	printk("wdt_monitor: is_vivologging_mode:%d, is_atboot_mode:%d\n",
							is_vivologging_mode, is_atboot_mode);

	printk("wdt_monitor: is_boot_bsptmode:%d, is_debugbuild_mode:%d\n",
							is_boot_bsptmode, is_debugbuild_mode);

	printk("wdt_monitor: is_normal_mode:%d, wdt_force_offline:%d\n",
							is_normal_boot, wdt_force_offline);

	return 0;
}

/*===========================================================================
**  Function :  wdt_monitor init interface
** ==========================================================================*/
static struct notifier_block restart_nb;
static int vivo_wdt_monitor_notifier_handler(struct notifier_block *this, unsigned long action,
			   void *data)
{
	vivo_wdt_monitor_disable();
	return NOTIFY_DONE;
}

int vivo_wdt_monitor_init(void)
{
	int ret;

	ret = kme_sysfs_init();
	if (ret)
		return ret;

	ret = kme_merge_group(&wdt_attr_group);
	if (ret) {
		//vivo_wdt_monitor_setup();
		//vivo_wdt_monitor_stop();
		vivo_wdt_monitor_ops_setup(NULL);
		return ret;
	}

	vivo_wdt_monitor_setup();

	restart_nb.notifier_call = vivo_wdt_monitor_notifier_handler;
	restart_nb.priority = 225;
	register_restart_handler(&restart_nb);

	return ret;
}

static bool vivo_wdt_monitor_initialized;
void vivo_wdt_monitor_prepare_and_start(void)
{
	if (vivo_wdt_monitor_initialized)
		return;

	qpnp_pon_wd_init();
	bootmode_init();
	vivo_wdt_monitor_init();

	if (is_normal_boot)
		vivo_wdt_monitor_start_kernel(900);

	vivo_wdt_monitor_initialized = true;
}

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/vmalloc.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/kmsg_dump.h>
#include <linux/delay.h>
#include <linux/sched/clock.h>
#include <linux/semaphore.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/timekeeping.h>
#include <linux/suspend.h>
#include <linux/reboot.h>

#include "boot_kmsg.h"

#define VIVO_BOOT_KMSG_TIMEOUT 200

extern int is_debug_mode;
static char *boot_kmsg_buf;
static size_t boot_kmsg_total_size = (1 << 20);
static size_t boot_kmsg_using_size;
static int boot_kmsg_thread_exit;
static struct semaphore boot_kmsg_wait_sem =
	__SEMAPHORE_INITIALIZER(boot_kmsg_wait_sem, 0);
static struct semaphore boot_kmsg_exit_sem =
	__SEMAPHORE_INITIALIZER(boot_kmsg_exit_sem, 0);


int get_boot_kmsg_buf_info(unsigned long *addr, size_t *size)
{
	if (!addr || !size)
		return -EINVAL;
	*addr = (unsigned long)boot_kmsg_buf;
	*size = boot_kmsg_using_size;
	return 0;
}

#ifdef VIVO_BOOT_KMSG_BACKUP_ALLOC
//module_param("boot_kmsg_total_size", uint, 0400);
#else
static char static_boot_kmsg_buf[__LOG_BUF_LEN];
#endif

static int boot_kmsg_log_show(struct seq_file *m, void *v)
{
	if (boot_kmsg_buf == NULL || boot_kmsg_total_size == 0) {
		seq_puts(m, "log buff is null.\n");
		return 0;
	}

	seq_printf(m, "boot_kmsg buffer size 0x%lx, actual using size 0x%lx.\n",
			boot_kmsg_total_size, boot_kmsg_using_size);
	seq_write(m, boot_kmsg_buf, boot_kmsg_using_size);

	return 0;
}

static int boot_kmsg_file_open(struct inode *inode, struct file *file)
{
	return single_open_size(file, boot_kmsg_log_show,
			inode->i_private, boot_kmsg_total_size);
}

static const struct file_operations proc_boot_kmsg_operations = {
	.open       = boot_kmsg_file_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static int boot_kmsg_thread(void *dummy)
{
	struct kmsg_dumper dumper;
	char *p = NULL;
	size_t len = 0, empty_size = boot_kmsg_total_size;
	int ret = 0;

#ifdef VIVO_BOOT_KMSG_BACKUP_ALLOC
	boot_kmsg_buf = vzalloc(empty_size);
	if (!boot_kmsg_buf) {
		pr_err("%s: Failed to alloc kmsg backup buffer\n", __func__);
		ret = -ENOMEM;
		goto nomem;
	}
#else
	boot_kmsg_buf = static_boot_kmsg_buf;
#endif
	p = boot_kmsg_buf;

	proc_create("boot_kmsg", 0400, NULL, &proc_boot_kmsg_operations);

	dumper.cur_seq = 0;
	dumper.cur_idx = 0;
	dumper.next_seq = 0;
	dumper.next_idx = 0;
	dumper.active = true;

	while (empty_size && !boot_kmsg_thread_exit) {
		if (!kmsg_dump_get_line(&dumper, true, p, empty_size, &len))
			if (down_timeout(&boot_kmsg_wait_sem, HZ) != -ETIME)
				printk(KERN_INFO "receive exit cmd\n");
		if (sched_clock()/1000000000 > VIVO_BOOT_KMSG_TIMEOUT)
			break;
		else if (empty_size > len + 1024) {
			empty_size -= len;
			boot_kmsg_using_size += len;
			p += len;
		} else
			break;
	}

nomem:
	up(&boot_kmsg_exit_sem);
	return ret;
}

#ifdef VIVO_KMSG_ADD_ANDROID_TIME
#define ARD_TIME_INTERVAL (20*HZ)
static struct delayed_work print_work;

static void get_android_time(struct rtc_time *tm_android, unsigned int *nsec)
{
	struct timespec64 ts = {0};

	ktime_get_real_ts64(&ts);
	ts.tv_sec -= sys_tz.tz_minuteswest * 60;
	rtc_time_to_tm(ts.tv_sec, tm_android);
	*nsec = (unsigned int)ts.tv_nsec / 1000;
}

static void print_android_time(void)
{
	struct rtc_time tm_android;
	unsigned int nsec;

	get_android_time(&tm_android, &nsec);
	pr_info("android time %d-%02d-%02d %02d:%02d:%02d.%06d\n",
			tm_android.tm_year + 1900,
			tm_android.tm_mon + 1,
			tm_android.tm_mday,
			tm_android.tm_hour,
			tm_android.tm_min,
			tm_android.tm_sec,
			nsec);
}

static void print_work_fn(struct work_struct *work)
{
	print_android_time();
	schedule_delayed_work(&print_work, ARD_TIME_INTERVAL);
}

static int print_android_time_pm_notify(struct notifier_block *self,
		unsigned long action, void *hcpu)
{
	switch (action) {
	case PM_SUSPEND_PREPARE:
	case PM_POST_SUSPEND:
		print_android_time();
		break;
	}

	return NOTIFY_OK;
}

extern void kstep_log(const char *fmt, ...);
static int print_android_time_reboot_notify(struct notifier_block *nb, 
		unsigned long code, void *args)
{
	struct rtc_time tm_android;
	unsigned int nsec;

	get_android_time(&tm_android, &nsec);
	kstep_log("android time %d-%02d-%02d %02d:%02d:%02d.%06d\n",
			tm_android.tm_year + 1900,
			tm_android.tm_mon + 1,
			tm_android.tm_mday,
			tm_android.tm_hour,
			tm_android.tm_min,
			tm_android.tm_sec,
			nsec);

	return NOTIFY_DONE;
}

static struct notifier_block reboot_notifier = {
	.notifier_call = print_android_time_reboot_notify,
};

#endif

int boot_kmsg_init(void)
{
	struct task_struct *t;
	int ret = 0;

#ifdef VIVO_KMSG_ADD_ANDROID_TIME
	INIT_DELAYED_WORK(&print_work, print_work_fn);
	schedule_delayed_work(&print_work, ARD_TIME_INTERVAL);
	pm_notifier(print_android_time_pm_notify, 0);
	register_reboot_notifier(&reboot_notifier);
#endif

	if (!is_debug_mode)
		return 0;

	printk(KERN_INFO "boot_kmsg_init\n");

	t = kthread_run(boot_kmsg_thread, NULL, "boot_kmsg");
	if (IS_ERR(t)) {
		ret = PTR_ERR(t);
		pr_err("%s: Failed to start boot_kmsg_thread (%d)\n",
				__func__, ret);
		return ret;
	}

	return 0;
}

void boot_kmsg_exit(void)
{
	if (!is_debug_mode)
		return;

	remove_proc_entry("boot_kmsg", NULL);
	boot_kmsg_thread_exit = 1;
	up(&boot_kmsg_wait_sem);
	down(&boot_kmsg_exit_sem);

#ifdef VIVO_BOOT_KMSG_BACKUP_ALLOC
	if (boot_kmsg_buf) {
		vfree(boot_kmsg_buf);
		boot_kmsg_buf = NULL;
	}
#endif

#ifdef VIVO_KMSG_ADD_ANDROID_TIME
	cancel_delayed_work_sync(&print_work);
#endif
	printk(KERN_INFO "boot_kmsg_exit\n");
}


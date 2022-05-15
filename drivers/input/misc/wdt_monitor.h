#ifndef __WDT_MONITOR_H__
#define __WDT_MONITOR_H__

#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/sysfs.h>
#include <linux/typecheck.h>
#include <linux/percpu-defs.h>
#include <linux/kobject.h>
#include <linux/seq_file.h>
#include <linux/ratelimit.h>
#include <linux/mutex.h>
#include <linux/watchdog.h>
#include <linux/sched/debug.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/reboot.h>

int kme_sysfs_init(void);
int vivo_wdt_monitor_init(void);
int kme_merge_group(const struct attribute_group *group);
int vivo_wdt_monitor_start_kernel(long timeout);
bool vivo_wdt_monitor_nack(struct watchdog_device *wdd, long timeout);
void vivo_wdt_monitor_prepare_and_start(void);

#endif //__WDT_MONITOR_H__

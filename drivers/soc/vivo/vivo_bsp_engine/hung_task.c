// SPDX-License-Identifier: GPL-2.0-only
/*
 * Detect Hung Task
 *
 * kernel/hung_task.c - kernel thread for detecting tasks stuck in D state
 *
 */

#include <linux/mm.h>
#include <linux/cpu.h>
#include <linux/nmi.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/lockdep.h>
#include <linux/export.h>
#include <linux/sysctl.h>
#include <linux/suspend.h>
#include <linux/utsname.h>
#include <linux/sched/clock.h>
#include <linux/sched/signal.h>
#include <linux/sched/debug.h>
#include <linux/sched/sysctl.h>
#include <linux/module.h>
#include <trace/events/sched.h>
#include <linux/kallsyms.h>
#include <linux/stacktrace.h>

extern int download_mode;
static struct task_struct *g_hung_task_culprit;
static bool hung_task_call_panic_critical;
static bool hung_task_during_test;
static bool hung_task_test_panic;
static int hung_ctl_sysfs_init(void);

#define PANIC(x) do { \
	if (hung_task_during_test) { \
		hung_task_test_panic = true; \
		WARN(1, x); \
	} else \
		panic(x); \
} while (0)

typedef enum {
	HUNG_TASK_CHECK_OFF,
	HUNG_TASK_CHECK_CRITICAL,
	HUNG_TASK_CHECK_ALL,
} hung_task_enable_t;

static hung_task_enable_t enable_state = HUNG_TASK_CHECK_CRITICAL;

#define TASK_WHITE_LIST_MAX  128
static const char *task_white_list[] = {
	"mmc-cmdqd",
	"cpuhp",
	"MtpServer",
	"ipacm",
	"swapon",
	"pem_policy_task",
	"kcompactd0",
	"hdcp_2x",
	"dp_hdcp2p2",
	"ion-pool-uncach",
	"ion-pool-cached",
	"mkswap",
	"sync",
};
static char task_white_list_ex[TASK_WHITE_LIST_MAX][TASK_COMM_LEN];
static DEFINE_SPINLOCK(task_white_list_ex_lock);

#define CRITICAL_LIST_TYPE       (1<<0)
#define CRITICAL_LIST_EX_TYPE    (1<<1)
#define CRITICAL_KEYWORD_TYPE    (1<<2)
#define CRITICAL_KEYWORD_EX_TYPE (1<<3)

#define WHITE_LIST_TYPE       (1<<0)
#define WHITE_LIST_EX_TYPE    (1<<1)
#define WHITE_KEYWORD_TYPE    (1<<2)
#define WHITE_KEYWORD_EX_TYPE (1<<3)

#define TASK_CRITICAL_LIST_MAX  128
static const char *task_critical_list[] = {
	"init",
};
static char task_critical_list_ex[TASK_CRITICAL_LIST_MAX][TASK_COMM_LEN];
static DEFINE_SPINLOCK(task_critical_list_ex_lock);

#define KEYWORD_CRITICAL_LIST_MAX  128
static const char *keyword_critical_list[] = {
	"pm_suspend",
	"io_schedule",
};
static char keyword_critical_list_ex[KEYWORD_CRITICAL_LIST_MAX][KSYM_SYMBOL_LEN];
static DEFINE_SPINLOCK(keyword_critical_list_ex_lock);

#define KEYWORD_WHITE_LIST_MAX  128
static char keyword_white_list_ex[KEYWORD_WHITE_LIST_MAX][KSYM_SYMBOL_LEN];
static DEFINE_SPINLOCK(keyword_white_list_ex_lock);

/*
 * The number of tasks checked:
 */
int __read_mostly sysctl_hung_task_check_count = PID_MAX_LIMIT;

/*
 * Limit number of tasks checked in a batch.
 *
 * This value controls the preemptibility of khungtaskd since preemption
 * is disabled during the critical section. It also controls the size of
 * the RCU grace period. So it needs to be upper-bound.
 */
#define HUNG_TASK_LOCK_BREAK (HZ / 10)

/*
 * Zero means infinite timeout - no checking done:
 */
static unsigned long __read_mostly vhung_task_timeout_secs = 150;

/*
 * Zero (default value) means use sysctl_hung_task_timeout_secs:
 */
#ifdef CONFIG_DETECT_HUNG_TASK
unsigned long __read_mostly sysctl_hung_task_timeout_secs = CONFIG_DEFAULT_HUNG_TASK_TIMEOUT;
unsigned long __read_mostly sysctl_hung_task_check_interval_secs;
int __read_mostly sysctl_hung_task_warnings = 10;
int sysctl_hung_task_selective_monitoring = 1;

/*
 * Process updating of timeout sysctl
 */
int proc_dohung_task_timeout_secs(struct ctl_table *table, int write,
				  void __user *buffer,
				  size_t *lenp, loff_t *ppos)
{
	int ret;

	ret = proc_doulongvec_minmax(table, write, buffer, lenp, ppos);

	if (ret || !write)
		goto out;

	//wake_up_process(watchdog_task);

 out:
	return ret;
}
#endif	/*CONFIG_DETECT_HUNG_TASK*/

static int __read_mostly did_panic;
static bool hung_task_show_lock;
static bool hung_task_call_panic;
static struct task_struct *watchdog_task;

struct task_struct *hung_task_get_culprit(void)
{
	if (g_hung_task_culprit)
		return g_hung_task_culprit;

	return NULL;
}
EXPORT_SYMBOL_GPL(hung_task_get_culprit);

static noinline bool
state_filter_match(unsigned long state_filter, struct task_struct *p)
{
	if (!(p->state & state_filter))
		return false;

	if (state_filter == TASK_UNINTERRUPTIBLE && p->state == TASK_IDLE)
		return false;

	return true;
}

static noinline void show_dsleep_task_stack(void)
{
	struct task_struct *g, *p;

	pr_info("\nAll Dsleep tasks info:\n");
	pr_info("  task                        PC stack   pid father\n");
	rcu_read_lock();
	for_each_process_thread(g, p) {
		if (state_filter_match(TASK_UNINTERRUPTIBLE, p))
			sched_show_task(p);
	}
	rcu_read_unlock();
}

static noinline int keyword_in_vivo_critical_list(char *sym)
{
	int i, found = 0;

	for (i = 0; i < ARRAY_SIZE(keyword_critical_list); i++) {
		if (!strncmp(sym, keyword_critical_list[i], KSYM_SYMBOL_LEN)) {
			found |= CRITICAL_KEYWORD_TYPE;
			break;
		}
	}
	if (found)
		goto done;

	spin_lock(&keyword_critical_list_ex_lock);
	for (i = 0; i < ARRAY_SIZE(keyword_critical_list_ex) && !found; i++) {
		if (!strncmp(sym, keyword_critical_list_ex[i], KSYM_SYMBOL_LEN)) {
			found |= CRITICAL_KEYWORD_EX_TYPE;
			break;
		}
	}
	spin_unlock(&keyword_critical_list_ex_lock);

done:
	return found;
}

static noinline int check_keyword_critical_list(struct task_struct *tsk)
{
	unsigned long stack_entries[64] = {};
	struct stack_trace callstack;
	char sym[KSYM_SYMBOL_LEN];
	int found, i;

	callstack.entries = stack_entries;
	callstack.nr_entries = 0;
	callstack.max_entries = ARRAY_SIZE(stack_entries);
	callstack.skip = 1;
	save_stack_trace_tsk(tsk, &callstack);

	for (i = 0; i < callstack.nr_entries; i++) {
		sprint_symbol(sym, callstack.entries[i]);
		strreplace(sym,  '+',  '\0');
		found = keyword_in_vivo_critical_list(sym);
		if (found)
			return found;
	}
	return 0;
}

static noinline int task_in_vivo_critical_list(struct task_struct *tsk)
{
	int i, found = 0;

	for (i = 0; i < ARRAY_SIZE(task_critical_list); i++) {
		if (!strncmp(tsk->comm, task_critical_list[i], TASK_COMM_LEN)) {
			found |= CRITICAL_LIST_TYPE;
			break;
		}
	}
	if (found)
		goto done;

	spin_lock(&task_critical_list_ex_lock);
	for (i = 0; i < ARRAY_SIZE(task_critical_list_ex); i++) {
		if (!strncmp(tsk->comm, task_critical_list_ex[i], TASK_COMM_LEN)) {
			found |= CRITICAL_LIST_EX_TYPE;
			break;
		}
	}
	spin_unlock(&task_critical_list_ex_lock);
	if (found)
		goto done;

	found |= check_keyword_critical_list(tsk);

done:
	return found;
}

static noinline int keyword_in_vivo_white_list(char *sym)
{
	int i, found = 0;

	spin_lock(&keyword_white_list_ex_lock);
	for (i = 0; i < ARRAY_SIZE(keyword_white_list_ex) && !found; i++) {
		if (!strncmp(sym, keyword_white_list_ex[i], KSYM_SYMBOL_LEN)) {
			found |= WHITE_KEYWORD_EX_TYPE;
			break;
		}
	}
	spin_unlock(&keyword_white_list_ex_lock);

	return found;
}

static noinline int check_keyword_white_list(struct task_struct *tsk)
{
	unsigned long stack_entries[64] = {};
	struct stack_trace callstack;
	char sym[KSYM_SYMBOL_LEN];
	int found, i;

	callstack.entries = stack_entries;
	callstack.nr_entries = 0;
	callstack.max_entries = ARRAY_SIZE(stack_entries);
	callstack.skip = 1;
	save_stack_trace_tsk(tsk, &callstack);

	for (i = 0; i < callstack.nr_entries; i++) {
		sprint_symbol(sym, callstack.entries[i]);
		strreplace(sym,  '+',  '\0');
		found = keyword_in_vivo_white_list(sym);
		if (found)
			return found;
	}
	return 0;
}

static noinline int task_in_vivo_white_list(struct task_struct *tsk)
{
	int i, found = 0;

	for (i = 0; i < ARRAY_SIZE(task_white_list); i++) {
		if (!strlen(task_white_list[i]))
			continue;

		if (strnstr(tsk->comm, task_white_list[i], strlen(task_white_list[i]))) {
			found |= WHITE_LIST_TYPE;
			break;
		}
	}
	if (found)
		goto done;

	spin_lock(&task_white_list_ex_lock);
	for (i = 0; i < ARRAY_SIZE(task_white_list_ex); i++) {
		if (!strlen(task_white_list_ex[i]))
			continue;

		if (strnstr(tsk->comm, task_white_list_ex[i], strlen(task_white_list_ex[i]))) {
			found |= WHITE_LIST_EX_TYPE;
			break;
		}
	}
	spin_unlock(&task_white_list_ex_lock);
	if (found)
		goto done;

	found |= check_keyword_white_list(tsk);

done:
	return found;
}


/*
 * Should we panic (and reboot, if panic_timeout= is set) when a
 * hung task is detected:
 */
unsigned int __read_mostly sysctl_hung_task_panic = 1;

module_param(sysctl_hung_task_panic, uint, 0);

static int
hung_task_panic(struct notifier_block *this, unsigned long event, void *ptr)
{
	if (!hung_task_during_test)
		did_panic = 1;

	return NOTIFY_DONE;
}

static struct notifier_block panic_block = {
	.notifier_call = hung_task_panic,
};

static noinline void check_hung_task(struct task_struct *t, unsigned long timeout)
{
	unsigned long long last_arrival = t->sched_info.last_arrival / 1000000000;
	unsigned long long now = local_clock() / 1000000000;
	int critical = 0, whitelist_task = 0;

	/*
	 * Ensure the task is not frozen.
	 * Also, skip vfork and any other user process that freezer should skip.
	 */
	if (unlikely(t->flags & (PF_FROZEN | PF_FREEZER_SKIP)))
	    return;

	/*
	 * When a freshly created task is scheduled once, changes its state to
	 * TASK_UNINTERRUPTIBLE without having ever been switched out once, it
	 * musn't be checked.
	 */
	if (now < last_arrival + vhung_task_timeout_secs) {
		return;
	}

	/*if task is in white list, then return*/
	whitelist_task = task_in_vivo_white_list(t);
	if (whitelist_task) {
		pr_info("hung check, skip task \"%s\":%d is white task, (type: 0x%x).\n",
				t->comm, t->pid, whitelist_task);
		return;
	}

	hung_task_call_panic = true;
	/*
	 * Ok, the task did not get scheduled for more than 2 minutes,
	 * complain:
	 */
	pr_err("INFO: task \"%s\":%d blocked for more than %lld seconds.\n",
			t->comm, t->pid, (now - last_arrival));
	pr_err("      %s %.*s\n",
			init_utsname()->release,
			(int)strcspn(init_utsname()->version, " "),
			init_utsname()->version);
	pr_err("\"echo 0 > /proc/sys/kernel/hung_task_timeout_secs\""
			" disables this message.\n");
	sched_show_task(t);
	hung_task_show_lock = true;

	critical = task_in_vivo_critical_list(t);
	if (critical) {
		pr_err("INFO: task \"%s\":%d is critical task, (type: 0x%x).\n", t->comm, t->pid, critical);
		hung_task_call_panic_critical = true;
	}

	/*
	 * Ok, save the earlist blocked task into global at last.
	 */
	if (!g_hung_task_culprit) {
		g_hung_task_culprit = t;
		return;
	}

	if ((g_hung_task_culprit->sched_info.last_arrival - t->sched_info.last_arrival) > 0)
		g_hung_task_culprit = t;
}

/*
 * To avoid extending the RCU grace period for an unbounded amount of time,
 * periodically exit the critical section and enter a new one.
 *
 * For preemptible RCU it is sufficient to call rcu_read_unlock in order
 * to exit the grace period. For classic RCU, a reschedule is required.
 */
static noinline bool rcu_lock_break(struct task_struct *g, struct task_struct *t)
{
	bool can_cont;

	get_task_struct(g);
	get_task_struct(t);
	rcu_read_unlock();
	cond_resched();
	rcu_read_lock();
	can_cont = pid_alive(g) && pid_alive(t);
	put_task_struct(t);
	put_task_struct(g);

	return can_cont;
}

/*
 * Check whether a TASK_UNINTERRUPTIBLE does not get woken up for
 * a really long time (120 seconds). If that happens, print out
 * a warning.
 */
static noinline void check_hung_uninterruptible_tasks(unsigned long timeout)
{
	int max_count = sysctl_hung_task_check_count;
	unsigned long last_break = jiffies;
	struct task_struct *g, *t;

	pr_info("hung_task: check_hung_uninterruptible_tasks\n");
	/*
	 * If the system crashed already then all bets are off,
	 * do not report extra hung tasks:
	 */
	if (did_panic)
		return;

	g_hung_task_culprit = NULL;
	hung_task_show_lock = false;
	hung_task_call_panic = false;
	hung_task_call_panic_critical = false;

	rcu_read_lock();
	for_each_process_thread(g, t) {
		if (!max_count--)
			goto unlock;
		if (time_after(jiffies, last_break + HUNG_TASK_LOCK_BREAK)) {
			if (!rcu_lock_break(g, t))
				goto unlock;
			last_break = jiffies;
		}
		/* use "==" to skip the TASK_KILLABLE tasks waiting on NFS */
		if (t->state == TASK_UNINTERRUPTIBLE)
			check_hung_task(t, timeout);
	}
 unlock:
	rcu_read_unlock();
	if (hung_task_show_lock)
		debug_show_all_locks();

	if (!hung_task_call_panic)
		return;

	if ((enable_state == HUNG_TASK_CHECK_ALL) || download_mode) {
		if (!download_mode)
			show_dsleep_task_stack();
		trigger_all_cpu_backtrace();
		PANIC("hung_task: blocked tasks");
	} else if ((enable_state == HUNG_TASK_CHECK_CRITICAL) && hung_task_call_panic_critical) {
		show_dsleep_task_stack();
		trigger_all_cpu_backtrace();
		PANIC("hung_task: blocked critical tasks");
	}
}

static noinline long hung_timeout_jiffies(unsigned long last_checked,
				 unsigned long timeout)
{
	/* timeout of 0 will disable the watchdog */
	return timeout ? last_checked - jiffies + timeout * HZ :
		MAX_SCHEDULE_TIMEOUT;
}

static atomic_t reset_hung_task = ATOMIC_INIT(0);

void reset_vhung_task_detector(void)
{
	atomic_set(&reset_hung_task, 1);
}
EXPORT_SYMBOL_GPL(reset_vhung_task_detector);


static noinline int string_to_array(char *target, unsigned int max_row,
		unsigned int max_col, const char *source)
{
	int i = 0, j = 0;

	memset(target, 0, max_row * max_col);
	while (*source) {
		if (*source == '|') {
			if (j) {
				i++;
				j = 0;
			}
		} else if (*source != '\n') {
			target[i * max_col + (j++)] = *source;
		}
		source++;
		if (i >= max_row || j >= max_col)
			goto fail;
	}
	return 0;
fail:
	memset(target, 0, max_row * max_col);
	return -EINVAL;
}

static ssize_t hung_task_enable_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	const char *state = "default";

	if (enable_state == HUNG_TASK_CHECK_OFF)
		state = "off";
	else if (enable_state == HUNG_TASK_CHECK_ALL)
		state = "all";
	else if (enable_state == HUNG_TASK_CHECK_CRITICAL)
		state = "critical";

	return scnprintf(buf, PAGE_SIZE, "%s\n", state);
}

static ssize_t hung_task_enable_store(struct kobject *kobj, struct kobj_attribute *attr,
		       const char *buf, size_t count)
{
	pr_info("%s: %s\n", __FUNCTION__, buf);

	if (!strcmp(buf, "off"))
		enable_state = HUNG_TASK_CHECK_OFF;
	else if (!strcmp(buf, "all"))
		enable_state = HUNG_TASK_CHECK_ALL;
	else if (!strcmp(buf, "critical"))
		enable_state = HUNG_TASK_CHECK_CRITICAL;
	else if (strcmp(buf, "\n"))	//sign of endline
		return -EINVAL;

	return count;
}

static ssize_t hung_task_whitelist_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	int i, cnt = 0;

	spin_lock(&task_white_list_ex_lock);
	for (i = 0; i < TASK_WHITE_LIST_MAX && *task_white_list_ex[i]; i++)
		cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "\"%s\"\n", task_white_list_ex[i]);
	spin_unlock(&task_white_list_ex_lock);

	return cnt;
}

static ssize_t hung_task_whitelist_store(struct kobject *kobj, struct kobj_attribute *attr,
		       const char *buf, size_t count)
{
	int ret;

	pr_info("%s in: %s\n", __FUNCTION__, buf);
	spin_lock(&task_white_list_ex_lock);
	ret = string_to_array((char *)task_white_list_ex, TASK_WHITE_LIST_MAX, TASK_COMM_LEN, buf);
	spin_unlock(&task_white_list_ex_lock);

	if (ret)
		return ret;

	return count;
}

static ssize_t hung_task_criticallist_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	int i, cnt = 0;

	spin_lock(&task_critical_list_ex_lock);
	for (i = 0; i < TASK_CRITICAL_LIST_MAX && *task_critical_list_ex[i]; i++)
		cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "\"%s\"\n", task_critical_list_ex[i]);

	spin_unlock(&task_critical_list_ex_lock);

	return cnt;
}

static ssize_t hung_task_criticallist_store(struct kobject *kobj, struct kobj_attribute *attr,
		       const char *buf, size_t count)
{
	int ret;

	pr_info("%s in: %s\n", __FUNCTION__, buf);
	spin_lock(&task_critical_list_ex_lock);
	ret = string_to_array((char *)task_critical_list_ex, TASK_CRITICAL_LIST_MAX, TASK_COMM_LEN, buf);
	spin_unlock(&task_critical_list_ex_lock);

	if (ret)
		return ret;

	return count;
}

static ssize_t hung_task_keyword_whitelist_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	int i, cnt = 0;

	spin_lock(&keyword_white_list_ex_lock);
	for (i = 0; i < KEYWORD_WHITE_LIST_MAX && *keyword_white_list_ex[i]; i++)
		cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "\"%s\"\n", keyword_white_list_ex[i]);
	spin_unlock(&keyword_white_list_ex_lock);

	return cnt;
}

static ssize_t hung_task_keyword_whitelist_store(struct kobject *kobj, struct kobj_attribute *attr,
		       const char *buf, size_t count)
{
	int ret;

	pr_info("%s in: %s\n", __FUNCTION__, buf);
	spin_lock(&keyword_white_list_ex_lock);
	ret = string_to_array((char *)keyword_white_list_ex, KEYWORD_WHITE_LIST_MAX, KSYM_SYMBOL_LEN, buf);
	spin_unlock(&keyword_white_list_ex_lock);

	if (ret)
		return ret;

	return count;
}

static ssize_t hung_task_keyword_criticallist_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	int i, cnt = 0;

	spin_lock(&keyword_critical_list_ex_lock);
	for (i = 0; i < KEYWORD_CRITICAL_LIST_MAX && *keyword_critical_list_ex[i]; i++)
		cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "\"%s\"\n", keyword_critical_list_ex[i]);
	spin_unlock(&keyword_critical_list_ex_lock);

	return cnt;
}

static ssize_t hung_task_keyword_criticallist_store(struct kobject *kobj, struct kobj_attribute *attr,
		       const char *buf, size_t count)
{
	int ret;

	pr_info("%s in: %s\n", __FUNCTION__, buf);
	spin_lock(&keyword_critical_list_ex_lock);
	ret = string_to_array((char *)keyword_critical_list_ex, KEYWORD_CRITICAL_LIST_MAX, KSYM_SYMBOL_LEN, buf);
	spin_unlock(&keyword_critical_list_ex_lock);

	if (ret)
		return ret;

	return count;
}

static ssize_t hung_task_test_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", hung_task_test_panic);
}

static ssize_t hung_task_test_store(struct kobject *kobj, struct kobj_attribute *attr,
		       const char *buf, size_t count)
{
	int ret = 0;
	long test_type = 0;
	unsigned long vhung_task_timeout_secs_bak = vhung_task_timeout_secs;

	ret = kstrtol(buf, 10, &test_type);
	if (ret)
		return ret;

	if (test_type == 1)
		hung_task_during_test = true;	//only set hung_task_test_panic flag if call PANIC
	else if (test_type == 2)
		hung_task_during_test = false;	//do the real panic if call PANIC
	else {
		pr_info("hung task test, invalid input\n");
		return -EINVAL;
	}

	hung_task_test_panic = false;
	vhung_task_timeout_secs = 5;
	wake_up_process(watchdog_task);

	pr_info("hung task test, start\n");
	ssleep(2 * vhung_task_timeout_secs);
	schedule_timeout_interruptible(2 * vhung_task_timeout_secs * HZ);
	pr_info("hung task test, end\n");

	hung_task_during_test = false;
	vhung_task_timeout_secs = vhung_task_timeout_secs_bak;

	return count;
}

static struct kobj_attribute hung_task_enable_attribute =
	__ATTR(hung_task_enable, 0644, hung_task_enable_show, hung_task_enable_store);
static struct kobj_attribute hung_task_whitelist_attribute =
	__ATTR(hung_task_whitelist, 0644, hung_task_whitelist_show, hung_task_whitelist_store);
static struct kobj_attribute hung_task_criticallist_attribute =
	__ATTR(hung_task_criticallist, 0644, hung_task_criticallist_show, hung_task_criticallist_store);
static struct kobj_attribute hung_task_keyword_whitelist_attribute =
	__ATTR(hung_task_keyword_whitelist, 0644, hung_task_keyword_whitelist_show, hung_task_keyword_whitelist_store);
static struct kobj_attribute hung_task_keyword_criticallist_attribute =
	__ATTR(hung_task_keyword_criticallist, 0644, hung_task_keyword_criticallist_show, hung_task_keyword_criticallist_store);
static struct kobj_attribute hung_task_test_attribute =
	__ATTR(hung_task_test, 0644, hung_task_test_show, hung_task_test_store);

static struct attribute *hung_ctl_attrs[] = {
	&hung_task_enable_attribute.attr,
	&hung_task_whitelist_attribute.attr,
	&hung_task_criticallist_attribute.attr,
	&hung_task_keyword_whitelist_attribute.attr,
	&hung_task_keyword_criticallist_attribute.attr,
	&hung_task_test_attribute.attr,
	NULL,	/* need to NULL terminate the list */
};

static struct attribute_group hung_ctl_attr_group = {
	.attrs = hung_ctl_attrs,
};

static struct kobject *hung_ctl_kobj;
static int hung_ctl_sysfs_init(void)
{
	int retval = 0;

	/* Create the dir under sys/kernel/hung_ctl */
	hung_ctl_kobj = kobject_create_and_add("hung_ctl", kernel_kobj);
	if (!hung_ctl_kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(hung_ctl_kobj, &hung_ctl_attr_group);
	if (retval)
		kobject_put(hung_ctl_kobj);

	return retval;
}


/*
 * kthread which checks for tasks stuck in D state
 */
static noinline int watchdog(void *dummy)
{
	unsigned long hung_last_checked = jiffies;

	set_user_nice(current, 0);

	for ( ; ; ) {
		unsigned long timeout = vhung_task_timeout_secs;
		long t;

		t = hung_timeout_jiffies(hung_last_checked, timeout);
		if (t <= 0) {
			if (!atomic_xchg(&reset_hung_task, 0))
				check_hung_uninterruptible_tasks(timeout);
			hung_last_checked = jiffies;
			continue;
		}
		schedule_timeout_interruptible(t);
	}

	return 0;
}

int hung_task_init(void)
{
	hung_ctl_sysfs_init();

	atomic_notifier_chain_register(&panic_notifier_list, &panic_block);

	pr_info("vkhungtaskd start (t)\n");
	watchdog_task = kthread_run(watchdog, NULL, "vkhungtaskd");

	return 0;
}

#ifndef MODULE
static int __init hung_task_module_init(void)
{
	return hung_task_init();
}

subsys_initcall(hung_task_module_init);
#endif


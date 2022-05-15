/*
 * 2019, Yang Yang
 */

#ifndef _LINUX_LOCK_INFO_H
#define _LINUX_LOCK_INFO_H


#define FATE_MUTEX		0
#define FATE_RWSEM_R	1
#define FATE_RWSEM_W	2


#ifdef CONFIG_BLK_FATE_QOS

union fate_lock {
	struct {
		s8			mutex;
		s8			rwsem_r;
		s8			rwsem_w;
		s8			semaphore;
		s8			mutex3;
		s8			mutex4;
		s8			mutex5;
		s8			mutex6;
	} b;
	u64 s;
};

static inline void fate_lock_info_inc(int shift)
{
	union fate_lock *info = (union fate_lock *)&current->fate_lock;

	switch (shift) {
		case FATE_MUTEX:
			info->b.mutex++;
			break;
		case FATE_RWSEM_R:
			info->b.rwsem_r++;
			break;
		case FATE_RWSEM_W:
			info->b.rwsem_w++;
			break;
		default:
			break;
	}
}

static inline void fate_lock_info_dec(int shift)
{
	union fate_lock *info = (union fate_lock *)&current->fate_lock;

	switch (shift) {
		case FATE_MUTEX:
			info->b.mutex--;
			break;
		case FATE_RWSEM_R:
			info->b.rwsem_r--;
			break;
		case FATE_RWSEM_W:
			info->b.rwsem_w--;
			break;
		default:
			break;
	}
}

extern int proc_fate_show(struct seq_file *m, struct pid_namespace *ns,
		     struct pid *pid, struct task_struct *tsk);
#else

static inline void fate_lock_info_inc(int shift) {}
static inline void fate_lock_info_dec(int shift) {}

#endif

#endif

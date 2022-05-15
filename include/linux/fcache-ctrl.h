/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_FCACHE_CTRL_H
#define _LINUX_FCACHE_CTRL_H

#include <linux/vmstat.h>

extern unsigned long android_data_ino;
extern unsigned long data_app_ino;
extern unsigned long data_data_ino;

extern unsigned long top_android_data_ino;
extern unsigned long top_data_app_ino;
extern unsigned long top_data_data_ino;

extern unsigned long vital_android_data_ino;
extern unsigned long vital_data_app_ino;
extern unsigned long vital_data_data_ino;

extern unsigned long min_file;
extern unsigned long cached_limit;
extern unsigned long vital_limit;
extern unsigned long nr_low;

extern long max_vital;
extern long max_staple;

extern int mm_kswap;
extern int fc_percent;

extern bool fctrl_en;
extern bool fctrl_remount;

extern bool is_dir_vital(unsigned long ino, enum fs_mount_point point);
extern bool is_file_vital(unsigned long ino, enum fs_mount_point point);

enum fcache_mode {
	FCACHE_OK,
	FCACHE_INVAL,
	FCACHE_CACHED,
	FCACHE_VITAL,
};

static inline bool is_top_app(struct inode *inode)
{
#if 0
	if (inode->i_sb->s_point == DIR_DATA) {
		if (!inode->i_child_of)
			return false;

		if (inode->i_child_of == top_android_data_ino ||
				inode->i_child_of == top_data_app_ino ||
				inode->i_child_of == top_data_data_ino)
			return true;
	}
#endif

	return false;
}

static inline bool is_vital_app(struct inode *inode)
{
#ifdef CONFIG_FCACHE_CTRL_DATA
	if (inode->i_sb->s_point == DIR_DATA) {
		if (!inode->i_child_of)
			return false;

		if (inode->i_child_of == vital_android_data_ino ||
				inode->i_child_of == vital_data_app_ino ||
				inode->i_child_of == vital_data_data_ino)
			return true;
	}
#endif
	return false;
}

static inline bool is_vital_inode(struct inode *inode)
{
	if (inode->i_vital)
		return true;

	return false;
}

static inline enum fcache_mode is_sufficient(void)
{
	unsigned long inactive, active, vital;
	unsigned long limit = SZ_64M >> PAGE_SHIFT;

	inactive = global_node_page_state(NR_INACTIVE_FILE);
	active = global_node_page_state(NR_ACTIVE_FILE);
	vital = global_node_page_state(NR_VITAL);

	if (inactive + active - vital < min_file + cached_limit) {
		return FCACHE_CACHED;
	} else if (inactive + active - vital < min_file + cached_limit + limit) {
		if (vital > max_vital - vital_limit)
			return FCACHE_VITAL;
	}

	return FCACHE_OK;
}

static inline bool staple_list_is_full(unsigned long nr)
{
	return nr > max_staple;
}

static inline bool vital_list_is_full(unsigned long nr)
{
	return nr > max_vital;
}

static inline bool lru_file_is_low(unsigned long nr)
{
	return nr < min_file;
}

#endif /* _LINUX_FCACHE_CTRL_H */

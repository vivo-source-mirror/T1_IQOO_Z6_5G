
#define pr_fmt(fmt) "FCACHE: " fmt

#include <linux/version.h>
#include <linux/mm.h>
#include <linux/seqlock.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/stddef.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/namei.h>
#include <linux/freezer.h>
#include <linux/uaccess.h>
#include <linux/of_fdt.h>

#ifndef CONFIG_RSC_ZRAM
static unsigned int tis_atboot = 0;
#else
extern unsigned int tis_atboot;
#endif

enum fc_mem_size {
	FC_NOP = 0,
	FC_8G,
	FC_12G,
	FC_16G,
};

struct dir_callback {
	struct dir_context ctx;
	unsigned long ino;
	char *dir;
};

struct dir_mlock {
	unsigned long ino;
	bool enable;
	enum fc_mem_size size;
	char *dir;
};

struct file_mlock {
	struct dir_mlock *d_mlock;
	unsigned int num;
};

static int check_dir(char *path, unsigned long pino);

static struct dir_mlock s_file_locked[] = {
	{0, 0, FC_8G, "/system/fonts/DroidSansFallbackMonster.ttf"},
};

static struct file_mlock v_file_locked;

static struct dir_mlock nop_v_file_locked[] = {
};

static struct dir_mlock qcom_v_file_locked[] = {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
	{0, 0, FC_8G, "/vendor/lib64/libllvm-qcom.so"},
	{0, 0, FC_8G, "/vendor/lib64/libgsl.so"},
	{0, 0, FC_8G, "/vendor/lib64/libllvm-glnext.so"},
	{0, 0, FC_8G, "/vendor/lib64/libqdMetaData.so"},
	{0, 0, FC_8G, "/vendor/lib64/libadreno_app_profiles.so"},
	{0, 0, FC_8G, "/vendor/lib64/libgrallocutils.so"},
	{0, 0, FC_8G, "/vendor/lib64/libgralloccore.so"},
	{0, 0, FC_8G, "/vendor/lib64/libgralloc.qti.so"},
	{0, 0, FC_8G, "/vendor/lib64/libadreno_utils.so"},
	{0, 0, FC_8G, "/vendor/lib64/vendor.qti.qspmhal@1.0.so"},
	{0, 0, FC_8G, "/vendor/lib64/vendor.qti.hardware.display.mapper@2.0.so"},
	{0, 0, FC_8G, "/vendor/lib64/vendor.qti.hardware.display.mapper@3.0.so"},
	{0, 0, FC_8G, "/vendor/lib64/vendor.qti.hardware.display.mapper@4.0.so"},
	{0, 0, FC_8G, "/vendor/lib64/vendor.qti.hardware.display.mapperextensions@1.0.so"},
	{0, 0, FC_8G, "/vendor/lib64/vendor.qti.hardware.display.mapperextensions@1.1.so"},
	{0, 0, FC_8G, "/vendor/lib/libgsl.so"},
	{0, 0, FC_8G, "/vendor/lib/libllvm-glnext.so"},
	{0, 0, FC_8G, "/vendor/lib/libqdMetaData.so"},
	{0, 0, FC_8G, "/vendor/lib/libadreno_app_profiles.so"},
	{0, 0, FC_8G, "/vendor/lib/libgrallocutils.so"},
	{0, 0, FC_8G, "/vendor/lib/libgralloccore.so"},
	{0, 0, FC_8G, "/vendor/lib/libgralloc.qti.so"},
	{0, 0, FC_8G, "/vendor/lib/libadreno_utils.so"},
	{0, 0, FC_8G, "/vendor/lib/vendor.qti.qspmhal@1.0.so"},
	{0, 0, FC_8G, "/vendor/lib/vendor.qti.hardware.display.mapper@2.0.so"},
	{0, 0, FC_8G, "/vendor/lib/vendor.qti.hardware.display.mapper@3.0.so"},
	{0, 0, FC_8G, "/vendor/lib/vendor.qti.hardware.display.mapper@4.0.so"},
	{0, 0, FC_8G, "/vendor/lib/vendor.qti.hardware.display.mapperextensions@1.0.so"},
	{0, 0, FC_8G, "/vendor/lib/vendor.qti.hardware.display.mapperextensions@1.1.so"},
	{0, 0, FC_8G, "/vendor/lib64/hw/gralloc.default.so"},
	{0, 0, FC_8G, "/vendor/lib64/hw/android.hardware.graphics.mapper@3.0-impl-qti-display.so"},
	{0, 0, FC_8G, "/vendor/lib64/hw/android.hardware.graphics.mapper@4.0-impl-qti-display.so"},
	{0, 0, FC_8G, "/vendor/lib/hw/gralloc.default.so"},
	{0, 0, FC_8G, "/vendor/lib/hw/android.hardware.graphics.mapper@3.0-impl-qti-display.so"},
	{0, 0, FC_8G, "/vendor/lib/hw/android.hardware.graphics.mapper@4.0-impl-qti-display.so"},
	{0, 0, FC_8G, "/vendor/etc/passwd"},
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 0, 0)
	{0, 0, FC_8G, "/vendor/lib64/hw/gralloc.kona.so"},
	{0, 0, FC_8G, "/vendor/lib/hw/gralloc.kona.so"},
	{0, 0, FC_8G, "/vendor/lib64/libhidltransport.so"},
	{0, 0, FC_8G, "/vendor/lib/libhidltransport.so"},
#endif
#endif
};

static struct dir_mlock samsung_v_file_locked[] = {
	{0, 0, FC_8G, "/vendor/lib64/libbcc_mali.so"},
	{0, 0, FC_8G, "/vendor/lib64/libbccArm_valhall.so"},
	{0, 0, FC_8G, "/vendor/lib64/arm.graphics-V1-ndk_platform.so"},
	{0, 0, FC_8G, "/vendor/lib64/libion_exynos.so"},
	{0, 0, FC_8G, "/vendor/lib64/libRSDriverArm.so"},
	{0, 0, FC_8G, "/vendor/lib/arm.graphics-V1-ndk_platform.so"},
	{0, 0, FC_8G, "/vendor/lib/libion_exynos.so"},
	{0, 0, FC_8G, "/vendor/lib/libRSDriverArm.so"},
	{0, 0, FC_8G, "/vendor/lib64/hw/android.hardware.graphics.mapper@4.0-impl.so"},
	{0, 0, FC_8G, "/vendor/lib/hw/android.hardware.graphics.mapper@4.0-impl.so"},
	{0, 0, FC_12G, "/vendor/lib64/libLLVM_android_mali.so"},
};

static struct dir_mlock mtk_v_file_locked[] = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0)
	{0, 0, FC_8G, "/vendor/lib64/libgpud.so"},
	{0, 0, FC_8G, "/vendor/lib64/libged.so"},
	{0, 0, FC_8G, "/vendor/lib64/libdrm.so"},
	{0, 0, FC_8G, "/vendor/lib64/libladder.so"},
	{0, 0, FC_8G, "/vendor/lib64/libpq_prot.so"},
	{0, 0, FC_8G, "/vendor/lib64/libgpu_aux.so"},
	{0, 0, FC_8G, "/vendor/lib64/libion_mtk.so"},
	{0, 0, FC_8G, "/vendor/lib64/libion_ulit.so"},
	{0, 0, FC_8G, "/vendor/lib64/libaiselector.so"},
	{0, 0, FC_8G, "/vendor/lib64/libgralloc_extra.so"},
	{0, 0, FC_8G, "/vendor/lib64/libgralloctypes_mtk.so"},
	{0, 0, FC_8G, "/vendor/lib64/libdpframework.so"},
	{0, 0, FC_8G, "/vendor/lib64/libFrameRecord.so"},
	{0, 0, FC_8G, "/vendor/lib64/libgralloc_metadata.so"},
	{0, 0, FC_8G, "/vendor/lib64/libNoFpsActor.so"},
	{0, 0, FC_8G, "/vendor/lib64/libDefaultFpsActor.so"},
	{0, 0, FC_8G, "/vendor/lib64/arm.graphics-V1-ndk_platform.so"},
	{0, 0, FC_8G, "/vendor/lib64/vendor.mediatek.hardware.mms@1.0.so"},
	{0, 0, FC_8G, "/vendor/lib64/vendor.mediatek.hardware.mms@1.1.so"},
	{0, 0, FC_8G, "/vendor/lib64/vendor.mediatek.hardware.mms@1.2.so"},
	{0, 0, FC_8G, "/vendor/lib64/vendor.mediatek.hardware.mms@1.3.so"},
	{0, 0, FC_8G, "/vendor/lib64/vendor.mediatek.hardware.mms@1.4.so"},
	{0, 0, FC_8G, "/vendor/lib64/vendor.mediatek.hardware.mms@1.5.so"},
	{0, 0, FC_8G, "/vendor/lib64/vendor.mediatek.hardware.pq@2.0.so"},
	{0, 0, FC_8G, "/vendor/lib64/vendor.mediatek.hardware.gpu@1.0.so"},
	{0, 0, FC_8G, "/vendor/lib64/vendor.mediatek.hardware.mmagent@1.0.so"},
	{0, 0, FC_8G, "/vendor/lib/libgpud.so"},
	{0, 0, FC_8G, "/vendor/lib/libged.so"},
	{0, 0, FC_8G, "/vendor/lib/libdrm.so"},
	{0, 0, FC_8G, "/vendor/lib/libladder.so"},
	{0, 0, FC_8G, "/vendor/lib/libpq_prot.so"},
	{0, 0, FC_8G, "/vendor/lib/libgpu_aux.so"},
	{0, 0, FC_8G, "/vendor/lib/libion_mtk.so"},
	{0, 0, FC_8G, "/vendor/lib/libion_ulit.so"},
	{0, 0, FC_8G, "/vendor/lib/libaiselector.so"},
	{0, 0, FC_8G, "/vendor/lib/libgralloc_extra.so"},
	{0, 0, FC_8G, "/vendor/lib/libgralloctypes_mtk.so"},
	{0, 0, FC_8G, "/vendor/lib/libdpframework.so"},
	{0, 0, FC_8G, "/vendor/lib/libFrameRecord.so"},
	{0, 0, FC_8G, "/vendor/lib/libgralloc_metadata.so"},
	{0, 0, FC_8G, "/vendor/lib/libNoFpsActor.so"},
	{0, 0, FC_8G, "/vendor/lib/libDefaultFpsActor.so"},
	{0, 0, FC_8G, "/vendor/lib/arm.graphics-V1-ndk_platform.so"},
	{0, 0, FC_8G, "/vendor/lib/vendor.mediatek.hardware.mms@1.0.so"},
	{0, 0, FC_8G, "/vendor/lib/vendor.mediatek.hardware.mms@1.1.so"},
	{0, 0, FC_8G, "/vendor/lib/vendor.mediatek.hardware.mms@1.2.so"},
	{0, 0, FC_8G, "/vendor/lib/vendor.mediatek.hardware.mms@1.3.so"},
	{0, 0, FC_8G, "/vendor/lib/vendor.mediatek.hardware.mms@1.4.so"},
	{0, 0, FC_8G, "/vendor/lib/vendor.mediatek.hardware.mms@1.5.so"},
	{0, 0, FC_8G, "/vendor/lib/vendor.mediatek.hardware.pq@2.0.so"},
	{0, 0, FC_8G, "/vendor/lib/vendor.mediatek.hardware.gpu@1.0.so"},
	{0, 0, FC_8G, "/vendor/lib/vendor.mediatek.hardware.mmagent@1.0.so"},
	{0, 0, FC_8G, "/vendor/lib64/hw/android.hardware.graphics.mapper@4.0-impl-mediatek.so"},
	{0, 0, FC_8G, "/vendor/lib/hw/android.hardware.graphics.mapper@4.0-impl-mediatek.so"},
#endif
};

static struct dir_mlock system_locked[] = {
	{0, 0, FC_8G, "/system/lib"},
	{0, 0, FC_8G, "/system/lib64"},
	{0, 0, FC_8G, "/system/apex"},
	{0, 0, FC_8G, "/system/framework"},
	{0, 0, FC_8G, "/system/priv-app/SystemUI"},
	{0, 0, FC_8G, "/system/product/app/WebViewGoogle"},
	{0, 0, FC_8G, "/system/product/overlay"},
	{0, 0, FC_12G, "/system/fonts"},
	{0, 0, FC_12G, "/system/app/BBKLauncher2"},
};

static struct dir_mlock vendor_locked[] = {
	{0, 0, FC_8G, "/vendor/lib/egl"},
	{0, 0, FC_8G, "/vendor/lib/vndk"},
	{0, 0, FC_8G, "/vendor/lib64/egl"},
	{0, 0, FC_8G, "/vendor/lib64/vndk"},
	{0, 0, FC_8G, "/vendor/overlay"},
};

unsigned long android_data_ino = 0;
unsigned long data_app_ino = 0;
unsigned long data_data_ino = 0;

unsigned long top_android_data_ino = 0;
unsigned long top_data_app_ino = 0;
unsigned long top_data_data_ino = 0;

unsigned long vital_android_data_ino = 0;
unsigned long vital_data_app_ino = 0;
unsigned long vital_data_data_ino = 0;


unsigned long min_file = (300 * SZ_1M) >> PAGE_SHIFT;
unsigned long cached_limit = (0 * SZ_1M) >> PAGE_SHIFT;
unsigned long vital_limit = (16 * SZ_1M) >> PAGE_SHIFT;
unsigned long nr_low = 0;

long max_vital = ((600 * SZ_1M) >> PAGE_SHIFT);
long max_staple = ((50 * SZ_1M) >> PAGE_SHIFT);

int mm_kswap = 10;
int fc_percent = 70;

bool fctrl_en = 0;
bool fctrl_remount = 0;

static int manual = 1;

#define MAX_LEN	128

static char top_app[MAX_LEN] = "NOP";
static char top_app_ver[MAX_LEN] = {};

static char vital_app[MAX_LEN] = "NOP";
static char vital_app_ver[MAX_LEN] = {};

static DEFINE_MUTEX(top_mutex);
static DEFINE_MUTEX(vital_mutex);

bool is_file_vital(unsigned long ino, enum fs_mount_point point)
{
	int i;

	if (ino == 0)
		return false;

	if (point == DIR_SYSTEM) {
		for (i=0; i < ARRAY_SIZE(s_file_locked); i++) {
			if (ino == s_file_locked[i].ino)
				return true;
		}
	} else if (point == DIR_VENDOR) {
		for (i=0; i < v_file_locked.num; i++) {
			if (ino == v_file_locked.d_mlock[i].ino)
				return true;
		}
	}

	return false;
}

bool is_dir_vital(unsigned long ino, enum fs_mount_point point)
{
	int i;

	if (point == DIR_SYSTEM) {
		for (i=0; i < ARRAY_SIZE(system_locked); i++) {
			if (ino == system_locked[i].ino)
				return true;
		}
	} else if (point == DIR_VENDOR) {
		for (i=0; i < ARRAY_SIZE(vendor_locked); i++) {
			if (ino == vendor_locked[i].ino)
				return true;
		}
	}

	return false;
}

static bool is_dir_exist(const char *dirname, unsigned long *ino)
{
	bool ret = false;

	if ((!dirname) || (!dirname[0])) {
	} else {
		struct path file_path;
		int error;

		error = kern_path(dirname, 0, &file_path);
		if (!error) {
			if (S_ISDIR(file_path.dentry->d_inode->i_mode)) {
				*ino = file_path.dentry->d_inode->i_ino;
				ret = true;
				pr_debug("%s: %s ino=%lu\n", __func__, dirname, *ino);
			}

			path_put(&file_path);
		}
	}

	return ret;
}

static bool check_file(const char *name, unsigned long *ino)
{
	bool ret = false;

	if ((!name) || (!name[0])) {
	} else {
		struct path file_path;
		int error;

		error = kern_path(name, 0, &file_path);
		if (!error) {
			if (S_ISREG(file_path.dentry->d_inode->i_mode)) {
				*ino = file_path.dentry->d_inode->i_ino;
				file_path.dentry->d_inode->i_vital = 1;
				ret = true;
				pr_debug("%s: %s ino=%lu\n", __func__, name, *ino);
			}

			path_put(&file_path);
		}
	}

	return ret;
}

static int top_data_app_dir(struct dir_context *ctx, const char *name,
		int namelen, loff_t offset, u64 ino, unsigned int d_type)
{
	char version[MAX_LEN] = {};

	strcat(version, top_app);
	strcat(version, "-");

	if (strstr(name, version)) {
		memset(top_app_ver, 0, MAX_LEN);
		strcat(top_app_ver, name);
		top_data_app_ino = ino;
		pr_info("%s: %.*s, ino=%lu, %s\n",
				__func__, namelen, name, ino, version);
		return -EEXIST;
	}

	return 0;
}

static int vital_data_app_dir(struct dir_context *ctx, const char *name,
		int namelen, loff_t offset, u64 ino, unsigned int d_type)
{
	char version[MAX_LEN] = {};

	strcat(version, vital_app);
	strcat(version, "-");

	if (strstr(name, version)) {
		memset(vital_app_ver, 0, MAX_LEN);
		strcat(vital_app_ver, name);
		vital_data_app_ino = ino;
		pr_info("%s: %.*s, ino=%lu, %s\n",
				__func__, namelen, name, ino, version);
		return -EEXIST;
	}

	return 0;
}

static int fcache_store(const char *buf,
		struct dir_context ctx[],
		char *app, char *ver,
		unsigned long *ino1,
		unsigned long *ino2,
		unsigned long *ino3)
{
	char name[MAX_LEN] = {};
	struct file *fp;
	mm_segment_t fs;
	int ret = 0;

	fp = filp_open("/data/app", O_RDONLY, 0);
	if (IS_ERR(fp)) {
		ret = PTR_ERR(fp);
		goto error;
	}

	memset(app, 0, MAX_LEN);
	snprintf(app, MAX_LEN, "%s", buf);

	if (app[strlen(app)-1] == 10)
		app[strlen(app)-1] = 0;

	pr_info("%s: %s\n", __func__, app);

	fs = get_fs();
	set_fs(KERNEL_DS);
	*ino3 = 0;
	ret = iterate_dir(fp, &ctx[0]);
	set_fs(fs);

	filp_close(fp, NULL);

	if (*ino3 == 0) {
		ret = -EINVAL;
		goto error;
	}

	memset(name, 0, sizeof(name));
	strcat(name, "/storage/emulated/0/Android/data/");
	strcat(name, app);
	pr_info("%s: %s\n", __func__, name);

	if (!is_dir_exist(name, ino1)) {
		ret = -ENOENT;
		goto error;
	}

	memset(name, 0, sizeof(name));
	strcat(name, "/data/data/");
	strcat(name, app);
	pr_info("%s: %s\n", __func__, name);

	if (!is_dir_exist(name, ino2)) {
		ret = -ENOENT;
		goto error;
	}

	goto out;

error:
	memset(app, 0, MAX_LEN);
	snprintf(app, MAX_LEN, "%s", "NOP");
	*ino1 = 0;
	*ino2 = 0;
	*ino3 = 0;

out:
	return ret;
}

static int param_set_top(const char *val, const struct kernel_param *kp)
{
	int ret = 0;

	struct dir_context ctx[] = {
		{.actor = &top_data_app_dir},
	};

	mutex_lock(&top_mutex);
	ret = fcache_store(val, ctx, top_app, top_app_ver,
			&top_android_data_ino,
			&top_data_data_ino,
			&top_data_app_ino);
	mutex_unlock(&top_mutex);
	return ret;
}

static int param_get_top(char *buf, const struct kernel_param *kp)
{
	int ret = 0;

	mutex_lock(&top_mutex);
	ret = sprintf(buf, "APK=%s Adata=%lu app=%lu data=%lu\n",
			top_app, top_android_data_ino,
			top_data_app_ino, top_data_data_ino);
	mutex_unlock(&top_mutex);
	return ret;
}

static int param_set_vital(const char *val, const struct kernel_param *kp)
{
	int ret = 0;

	struct dir_context ctx[] = {
		{.actor = &vital_data_app_dir},
	};

	mutex_lock(&vital_mutex);
	ret = fcache_store(val, ctx, vital_app, vital_app_ver,
			&vital_android_data_ino,
			&vital_data_data_ino,
			&vital_data_app_ino);
	mutex_unlock(&vital_mutex);
	return ret;
}

static int param_get_vital(char *buf, const struct kernel_param *kp)
{
	int ret = 0;

	mutex_lock(&vital_mutex);
	ret = sprintf(buf, "APK=%s Adata=%lu app=%lu data=%lu\n",
			vital_app, vital_android_data_ino,
			vital_data_app_ino, vital_data_data_ino);
	mutex_unlock(&vital_mutex);
	return ret;
}

static int param_get_fctrl(char *buf, const struct kernel_param *kp)
{
	return sprintf(buf, "%d, %d\n", fctrl_en, fctrl_remount);
}

static int set_max_vital_fn(const char *val, const struct kernel_param *kp)
{
	int rc = 0;
	long param;

	rc = kstrtol(val, 0, &param);
	if (rc)
		return rc;

	if (param < 0)
		return -EINVAL;

	if (param > (SZ_1G >> PAGE_SHIFT))
		return -EINVAL;

	max_vital = param;
	return 0;
}

static int get_max_vital_fn(char *buf, const struct kernel_param *kp)
{
	return sprintf(buf, "%ld\n", max_vital);
}

static int set_max_staple_fn(const char *val, const struct kernel_param *kp)
{
	int rc = 0;
	long param;

	rc = kstrtol(val, 0, &param);
	if (rc)
		return rc;

	if (param < 0)
		return -EINVAL;

	if (param > (SZ_512M >> PAGE_SHIFT))
		return -EINVAL;

	max_staple = param;
	return 0;
}

static int get_max_staple_fn(char *buf, const struct kernel_param *kp)
{
	return sprintf(buf, "%ld\n", max_staple);
}

static int set_percent_fn(const char *val, const struct kernel_param *kp)
{
	unsigned int param;
	int rc = 0;

	rc = kstrtou32(val, 0, &param);
	if (rc)
		return rc;

	if (param == 0 || param > 100)
		return -EINVAL;

	return param_set_uint(val, kp);;
}

static int get_inode_fn(char *buf, const struct kernel_param *kp)
{
	int len = 0, i;

	for (i=0; i < ARRAY_SIZE(system_locked); i++) {
		struct dir_mlock *locked = system_locked;
		len += sprintf(buf + len, "%s -> %lu (%d) (%d)\n",
				locked[i].dir, locked[i].ino, locked[i].enable, locked[i].size);
	}

	for (i=0; i < ARRAY_SIZE(vendor_locked); i++) {
		struct dir_mlock *locked = vendor_locked;
		len += sprintf(buf + len, "%s -> %lu (%d) (%d)\n",
				locked[i].dir, locked[i].ino, locked[i].enable, locked[i].size);
	}

	for (i=0; i < ARRAY_SIZE(s_file_locked); i++) {
		struct dir_mlock *locked = s_file_locked;
		len += sprintf(buf + len, "%s -> %lu (%d) (%d)\n",
				locked[i].dir, locked[i].ino, locked[i].enable, locked[i].size);
	}

	for (i=0; i < v_file_locked.num; i++) {
		struct dir_mlock *locked = &v_file_locked.d_mlock[i];
		len += sprintf(buf + len, "%s -> %lu (%d) (%d)\n",
				locked->dir, locked->ino, locked->enable, locked->size);
	}

	return len;
}

#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX    "customize."

module_param_call(top, param_set_top, param_get_top, NULL, 0644);
module_param_call(vital, param_set_vital, param_get_vital, NULL, 0644);

module_param_call(check, NULL, param_get_fctrl, NULL, 0644);
module_param_call(inode, NULL, get_inode_fn, NULL, 0644);

module_param_named(manual, manual, int, 0644);
module_param_named(kswap, mm_kswap, int, 0644);

module_param_named(min_file, min_file, ulong, 0644);
module_param_named(cached_limit, cached_limit, ulong, 0644);
module_param_named(vital_limit, vital_limit, ulong, 0644);
module_param_named(nr_low, nr_low, ulong, 0644);

module_param_call(max_vital, set_max_vital_fn, get_max_vital_fn, NULL, 0644);
module_param_call(max_staple, set_max_staple_fn, get_max_staple_fn, NULL, 0644);
module_param_call(percent, set_percent_fn, param_get_uint, &fc_percent, 0644);

static int iterate_dir_func(struct dir_context *ctx, const char *name,
		int namelen, loff_t offset, u64 ino, unsigned int d_type)
{
	struct dir_callback *buf =
		container_of(ctx, struct dir_callback, ctx);
	char filename[MAX_LEN] = { 0 };

	pr_debug("%s: %s, %.*s, ino=%lu, %u, pino=%lu\n",
			__func__, buf->dir, namelen, name, ino, d_type, buf->ino);
	if (d_type == DT_DIR) {
		if (!strncmp(".", name, 1) || !strncmp("..", name, 2))
			return 0;

		memset(filename, 0, sizeof(filename));
		strcat(filename, buf->dir);
		strcat(filename, "/");
		strncat(filename, name, namelen);
		pr_debug("%s: DIR: %s\n", __func__, filename);

		check_dir(filename, buf->ino);
	} else if (d_type == DT_REG) {
		struct path file_path;
		int error;

		memset(filename, 0, sizeof(filename));
		strcat(filename, buf->dir);
		strcat(filename, "/");
		strncat(filename, name, namelen);

		error = kern_path(filename, 0, &file_path);
		if (!error) {
			if (S_ISREG(file_path.dentry->d_inode->i_mode)) {
				file_path.dentry->d_inode->i_vital = 1;
			}

			path_put(&file_path);
		}
		pr_debug("%s: REG: %s, error=%d\n", __func__, filename, error);
	}
	return 0;
}

static int check_dir(char *path, unsigned long pino)
{
	struct file *fp;
	mm_segment_t fs;
	struct path file_path;
	unsigned long ino = 0;
	int ret = 0;

	struct dir_callback buffer = {
		.ctx.actor = iterate_dir_func,
		.dir = path,
		.ino = ino,
	};

	ret = kern_path(path, 0, &file_path);
	if (!ret) {
		if (S_ISDIR(file_path.dentry->d_inode->i_mode)) {
			if (pino)
				file_path.dentry->d_inode->i_child_of = pino;

			ino = file_path.dentry->d_inode->i_ino;
			pr_debug("%s: %s, ino=%lu, pino=%lu\n", __func__, path, ino, pino);
		}

		path_put(&file_path);
	}

	if (!ino)
		return ret;

	if (!pino)
		buffer.ino = ino;
	else
		buffer.ino = pino;

	fp = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		ret = PTR_ERR(fp);
		return ret;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);
	ret = iterate_dir(fp, &buffer.ctx);
	set_fs(fs);

	filp_close(fp, NULL);
	return ret;
}

static int fcache_thread(void *data)
{
	int i;

	set_user_nice(current, MIN_NICE);
	set_freezable();

	while (!kthread_should_stop()) {
		if (fctrl_remount)
			return 0;

		for (i=0; i < ARRAY_SIZE(system_locked); i++) {
			if (!system_locked[i].ino && system_locked[i].enable)
				is_dir_exist(system_locked[i].dir, &system_locked[i].ino);
		}

		for (i=0; i < ARRAY_SIZE(vendor_locked); i++) {
			if (!vendor_locked[i].ino && vendor_locked[i].enable)
				is_dir_exist(vendor_locked[i].dir, &vendor_locked[i].ino);
		}

#ifdef CONFIG_FCACHE_CTRL_DATA
		if (!data_app_ino)
			is_dir_exist("/data/app", &data_app_ino);

		if (!data_data_ino)
			is_dir_exist("/data/data", &data_data_ino);

		if (!android_data_ino)
			is_dir_exist("/storage/emulated/0/Android/data",
					&android_data_ino);
#endif

		if (ktime_to_ns(ktime_get_boottime()) > (NSEC_PER_SEC * 60)) {
			if (!manual)
				return 0;

#ifdef CONFIG_FCACHE_CTRL_DATA
			if (!android_data_ino ||
					!data_app_ino ||
					!data_data_ino)
				return 0;
#endif

			for (i=0; i < 4; i++) {
				if (!system_locked[i].ino)
					return 0;
			}

			fctrl_en = 1;

			for (i=0; i < ARRAY_SIZE(system_locked); i++) {
				if (system_locked[i].enable)
					check_dir(system_locked[i].dir, 0);
				cond_resched();
			}

			for (i=0; i < ARRAY_SIZE(vendor_locked); i++) {
				if (vendor_locked[i].enable)
					check_dir(vendor_locked[i].dir, 0);
				cond_resched();
			}

			for (i=0; i < ARRAY_SIZE(s_file_locked); i++) {
				if (s_file_locked[i].enable)
					check_file(s_file_locked[i].dir, &s_file_locked[i].ino);
				cond_resched();
			}

			for (i=0; i < v_file_locked.num; i++) {
				struct dir_mlock *locked = &v_file_locked.d_mlock[i];
				if (locked->enable)
					check_file(locked->dir, &locked->ino);
				cond_resched();
			}
			return 0;
		}

		msleep(300);
	}

	return 0;
}

static void config_list(enum fc_mem_size size)
{
	int i;

	for (i=0; i < ARRAY_SIZE(system_locked); i++) {
		if (size >= system_locked[i].size)
			system_locked[i].enable = 1;
	}

	for (i=0; i < ARRAY_SIZE(vendor_locked); i++) {
		if (size >= vendor_locked[i].size)
			vendor_locked[i].enable = 1;
	}

	for (i=0; i < ARRAY_SIZE(s_file_locked); i++) {
		if (size >= s_file_locked[i].size)
			s_file_locked[i].enable = 1;
	}

	for (i=0; i < v_file_locked.num; i++) {
		if (size >= v_file_locked.d_mlock[i].size)
			v_file_locked.d_mlock[i].enable = 1;
	}
}

#ifndef CONFIG_RSC_ZRAM
static int __init fcache_setup(char *line)
{
	if (strncmp(line, "sendAT", 6) == 0)
		tis_atboot = 1;
	return 1;
}

__setup("console-at=", fcache_setup);
#endif

static int __init filecache_init(void)
{
	struct task_struct *thread = NULL;
	const char *name;
#if KERNEL_VERSION(5, 0, 0) > LINUX_VERSION_CODE
	const u64 total = totalram_pages << PAGE_SHIFT;
#else
	const u64 total = totalram_pages() << PAGE_SHIFT;
#endif

	if (total < (SZ_256M + SZ_2G + SZ_4G)) {
		pr_info("%s: total=%llu\n", __func__, total);
		return 0;
	}

	if (tis_atboot == 1) {
		pr_info("%s: in AT mode\n", __func__);
		return 0;
	}

	pr_info("%s: tis_atboot=%d\n", __func__, tis_atboot);

	name = of_flat_dt_get_machine_name();
	if (!name)
		return 0;

	pr_info("%s\n", name);

	if (strstr(name, "Qualcomm")) {
		v_file_locked.d_mlock = qcom_v_file_locked;
		v_file_locked.num = ARRAY_SIZE(qcom_v_file_locked);
		pr_info("Qualcomm\n");
	} else if (strstr(name, "Samsung")) {
		v_file_locked.d_mlock = samsung_v_file_locked;
		v_file_locked.num = ARRAY_SIZE(samsung_v_file_locked);
		pr_info("Samsung\n");
	} else if (strstr(name, "MT")) {
		v_file_locked.d_mlock = mtk_v_file_locked;
		v_file_locked.num = ARRAY_SIZE(mtk_v_file_locked);
		pr_info("MTK\n");
	} else {
		v_file_locked.d_mlock = nop_v_file_locked;
		v_file_locked.num = ARRAY_SIZE(nop_v_file_locked);
		pr_info("Nop\n");
	}

	if (total > (SZ_4G + SZ_4G + SZ_4G)) {
		max_vital = (800 * SZ_1M) >> PAGE_SHIFT;
		config_list(FC_16G);
	} else if (total > (SZ_4G + SZ_4G)) {
		max_vital = (750 * SZ_1M) >> PAGE_SHIFT;
		config_list(FC_12G);
	} else if (total > (SZ_2G + SZ_4G)) {
		max_vital = (620 * SZ_1M) >> PAGE_SHIFT;
		config_list(FC_8G);
	} else {
		BUG();
	}

	thread = kthread_run(fcache_thread, NULL, "fcache");
	if (IS_ERR(thread))
		return PTR_ERR(thread);

	return 0;
}

module_init(filecache_init);

#include <linux/async.h>
//#include <scsi/ufs/ioctl.h>
#include <linux/nls.h>
#include <linux/of.h>
#include <linux/blkdev.h>
#include <linux/sort.h>
#include <linux/types.h>
#include "ufshcd.h"
#include "ufshci.h"
#include "ufs_quirks.h"
//#include "ufs-debugfs.h"
#include "ufs-qcom.h"
#include "ufs_ven.h"
#include "unipro.h"

unsigned int ufs_ven_lu0_tw_lu_buf_size;
int ufs_ven_lu0_lu_enable;


extern int ufshcd_read_string_desc_ven(struct ufs_hba *hba, int desc_index,
			    u8 *buf, u32 size, bool ascii);
#if 1
extern int ufshcd_read_device_desc(struct ufs_hba *hba, u8 *buf, u32 size);
extern int ufshcd_read_health_desc(struct ufs_hba *hba, u8 *buf, u32 size);

static void ufshcd_io_stats_init_intern(struct ufs_hba *hba);

/* static io_ele_t cmd_buf[UFS_STAT_CMD_COUNT * sizeof(io_ele_t)]; */
/* static u16 idx_buf[UFS_STAT_CMD_COUNT]; */
static io_ele_t *cmd_buf;
static u16 *idx_buf;

unsigned int ufs_support_size[] = { /* sector unit, refer to UFS_STAT_SIZE_MAX */
		4	* 1024 / 512,
		512	* 1024 / 512,
};

unsigned int ufs_gear_map[UFS_GEAR_MAX] = {
	GEAR_INVALID,	/* UFS_HS_DONT_CHANGE */
	GEAR_LOW,		/* HS Gear 1 */
	GEAR_INVALID,	/* HS Gear 2 */
	GEAR_HIGH,		/* HS Gear 3 */
	GEAR_HIGH,		/* HS Gear 4 */
};

unsigned char *ufs_gear_map_str[GEAR_INVALID] = {
	"L", /* GEAR_LOW */
	"H", /* GEAR_HIGH */
};

unsigned char *ufs_support_size_str[UFS_STAT_SIZE_MAX] = {
	"4k",
	"512k",
	"othr"
};

unsigned char *ufs_support_cmd_type_str[CMD_TYPE_MAX] = { /* refer to cmd_types */
	"r",
	"w",
	"d",
	"f",
};

/* snprintf(msg, sizeof(msg), "latency=%lldms", */
		/* ktime_to_ms(delta)); latency_exception_single_cmd_time */
/* static int ufshcd_io_stats_uevent_report(struct ufs_hba *hba, */
				/* unsigned int gear, unsigned int size, int cmd_type, */
				/* char *except_msg, char *msg) */
/* { */
	/* char envp_ext[4][MAX_ENV_LEN]; */
	/* char *envp[4] = {NULL}; */
	/* int idx = 0; */

	/* dev_err(hba->dev, "[%s:%d] ufs_io [%s][%s][%s] exception=%s, %s\n", */
			/* __func__, __LINE__, */
			/* ufs_gear_map_str[gear], */
			/* ufs_support_cmd_type_str[cmd_type], */
			/* ufs_support_size_str[size], */
			/* except_msg, */
			/* msg); */

	/* snprintf(envp_ext[0], MAX_ENV_LEN, "subsystem=kernel_monitor_engine"); */
	/* snprintf(envp_ext[1], MAX_ENV_LEN, "exception=%s", except_msg); */
	/* snprintf(envp_ext[2], MAX_ENV_LEN, msg); */
	/* for (idx = 0; idx < ARRAY_SIZE(envp); idx++) */
		/* envp[idx] = envp_ext[idx]; */
	/* return kernel_monitor_report(envp); */
	/* return 0; */
/* } */

static inline unsigned int ufs_io_stats_size(struct request *rq)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ufs_support_size); i++)
		if (blk_rq_sectors(rq) == ufs_support_size[i])
			break;
	return i;
}

void ufshcd_io_stats(struct ufs_hba *hba, struct ufshcd_lrb *lrbp, unsigned int tag)
{
	unsigned long flags;
	io_stats_info_t *stat;
	io_stats_cmd_t *stat_cmd;
	io_ele_t ce;
	struct scsi_cmnd *cmd = lrbp->cmd;
	struct request *rq = cmd ? cmd->request : NULL;
	char cmd_op;
	int cmd_type;
	unsigned int size, gear, rq_size;
	ktime_t delta;
	s64 time;
	struct ufs_qcom_host *qcom_host = hba->priv;

	if (!rq || !cmd)
		return;

	rq_size = blk_rq_sectors(rq);
	cmd_op = cmd->cmnd[0];
	delta = ktime_sub(lrbp->compl_time_stamp,
			lrbp->issue_time_stamp);
	if (ktime_compare(delta, qcom_host->io_stats.lat_max) > 0)
		qcom_host->io_stats.lat_max = delta;

	gear = ufs_gear_map[hba->pwr_info.gear_tx];
	if ((gear == GEAR_INVALID) && __ratelimit(&qcom_host->io_stats.ratelimit)) {
		dev_err(hba->dev, "[%s:%d] Gear %d is not supported, pwr_tx %x\n",
				__func__, __LINE__, hba->pwr_info.gear_tx, hba->pwr_info.pwr_tx);
		return;
	}

	size = ufs_io_stats_size(rq);
	if (cmd_op == READ_6 || cmd_op == READ_10 || cmd_op == READ_16)
		cmd_type = CMD_READ;
	else if (cmd_op == WRITE_6 || cmd_op == WRITE_10 || cmd_op == WRITE_16)
		cmd_type = CMD_WRITE;
	else if (cmd_op == UNMAP) {
		cmd_type = CMD_DISCARD;
		size = UFS_STAT_SIZE_OTHER;
	} else if (cmd_op == SYNCHRONIZE_CACHE) {
		cmd_type = CMD_FLUSH;
		size = UFS_STAT_SIZE_OTHER;
		rq_size = 0;
	} else {
		/* dev_err(hba->dev, "[%s:%d] cmd %#x is not supported.\n", */
				/* __func__, __LINE__, cmd_op); */
		return;
	}

	if (qcom_host->io_stats.stat_cmd_enable) {
		ce.op = cmd_op;
		ce.lun = lrbp->lun;
		ce.lba = blk_rq_pos(rq);
		if (cmd_type == CMD_FLUSH)
			ce.lba = 0;
		ce.cmd_type = cmd_type;
		ce.size = rq_size;
		ce.time = delta;
		ce.tstamp = lrbp->issue_time_stamp;
	}
	spin_lock_irqsave(&qcom_host->io_stats.lock, flags);
	stat = &qcom_host->io_stats.stat[gear][size][cmd_type];
	stat->io_cnt++;
	qcom_host->io_stats.stat_io_cnt++;
	stat->size_cnt += rq_size;
	qcom_host->io_stats.stat_size_cnt += rq_size;
	if (ktime_compare(delta, stat->lat_max) > 0)
		stat->lat_max = delta;
	if (ktime_compare(delta, stat->lat_min) < 0)
		stat->lat_min = delta;
	stat->lat = ktime_add(stat->lat, delta);

	if (qcom_host->io_stats.stat_cmd_enable) {
		stat_cmd = &qcom_host->io_stats.stat_cmd;
		stat_cmd->cmd[stat_cmd->pos] = ce;
		stat_cmd->pos = (stat_cmd->pos + 1) % UFS_STAT_CMD_COUNT;
	}
	if (ktime_to_ms(delta) > UFS_STAT_LATENCY_MAX_TIME_MS)
		qcom_host->io_stats.stat_warn_io_cnt++;

	spin_unlock_irqrestore(&qcom_host->io_stats.lock, flags);

	if (qcom_host->io_stats.log_enable == UFS_VENDOR_LATTENCY_LOG_PRINT_ENABLE) {
		time = ktime_to_us(stat->lat);
		pr_err("ufs_io [%s][%s][%s] lba %#llx sz %u t %lld db %#x lun %d tag %u send_t %lld cmp_t %lld, avg %lld\n",
				ufs_gear_map_str[gear],
				ufs_support_cmd_type_str[cmd_type],
				ufs_support_size_str[size],
				(unsigned long long)blk_rq_pos(rq),
				rq_size * 512,
				ktime_to_us(delta),
				ufshcd_readl(hba, REG_UTP_TRANSFER_REQ_DOOR_BELL),
				lrbp->lun,
				tag,
				ktime_to_ns(lrbp->issue_time_stamp),
				ktime_to_ns(lrbp->compl_time_stamp),
				time / stat->io_cnt
			   );
	}

	if (ktime_to_ms(delta) > UFS_STAT_LATENCY_MAX_TIME_MS) {
		pr_err("Warning: ufs_io latency %lldus cmd %#x lba %#llx sz %#x db %#x lun %d tag %u send_t %lld cmp_t %lld gear %d io_cnt %llu total_size %llu\n",
			ktime_to_us(delta),
			cmd_op,
			(unsigned long long)blk_rq_pos(rq),
			rq_size * 512,
			ufshcd_readl(hba, REG_UTP_TRANSFER_REQ_DOOR_BELL),
			lrbp->lun,
			tag,
			ktime_to_ns(lrbp->issue_time_stamp),
			ktime_to_ns(lrbp->compl_time_stamp),
			hba->pwr_info.gear_tx,
			qcom_host->io_stats.stat_io_cnt,
			qcom_host->io_stats.stat_size_cnt
		   );
	}

}

static ssize_t
show_io_stats(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	unsigned long flags;
	int i, j, k;
	struct seq_file m;
	io_stats_info_t stat;
	s64 time;
	struct ufs_qcom_host *qcom_host = hba->priv;

	memset(&m, 0, sizeof(struct seq_file));
	m.buf = buf;
	m.size = PAGE_SIZE;

	seq_printf(&m, "%-17s: %d\n%-17s: %d\n%-17s: %d\n%-17s: %lld\n\n",
			"stat_cmd_enable",
			qcom_host->io_stats.stat_cmd_enable,
			"log_enable",
			qcom_host->io_stats.log_enable,
			"load_thre",
			qcom_host->io_stats.load_thre,
			"latency_max(us)",
			ktime_to_us(qcom_host->io_stats.lat_max)
			);

	seq_printf(&m, "%-20s %16s %16s %16s %16s %16s\n",
			"Item", "IO Count", "Size",
			"Latency Avg(us)", "Latency Max(us)", "Latency Min(us)");
	for (i = 0; i < UFS_STAT_GEAR_MAX; i++)
		for (k = 0; k < CMD_TYPE_MAX; k++)
			for (j = 0; j < UFS_STAT_SIZE_MAX; j++) {
				spin_lock_irqsave(&qcom_host->io_stats.lock, flags);
				stat = qcom_host->io_stats.stat[i][j][k];
				spin_unlock_irqrestore(&qcom_host->io_stats.lock, flags);

				if ((((k == CMD_DISCARD) || (k == CMD_FLUSH))
							&& (j != UFS_STAT_SIZE_OTHER))
						|| (stat.io_cnt == 0))
					continue;

				time = ktime_to_us(stat.lat);
				seq_printf(&m,
						"[%s][%s][%4s] %16llu %16llu %16lld %16lld %16lld\n",
						ufs_gear_map_str[i],
						ufs_support_cmd_type_str[k],
						ufs_support_size_str[j],
						stat.io_cnt,
						stat.size_cnt * 512,
						time / stat.io_cnt,
						ktime_to_us(stat.lat_max),
						ktime_to_us(stat.lat_min)
						);
			}

	return m.count;
}

static ssize_t
set_io_stats(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	int64_t value;

	if (kstrtoll(buf, 0, &value))
		return 0;

	if (!value) {
		ufshcd_io_stats_init_intern(hba);
	} else
		dev_err(hba->dev, "[%s:%d] ufs: Input %lld is invalid.\n",
				__func__, __LINE__, value);

	return count;
}

static DEVICE_ATTR(io_stats, S_IRUGO | S_IWUSR,
		show_io_stats, set_io_stats);

struct cmd_sta_tmp {
	u_int64_t	size_cnt;						/* total access size */
	u_int64_t	io_cnt;							/* total access counter */
	ktime_t		lat_sum;
	ktime_t		lat_max;
};
#define		UFS_CMD_STRING		"ufs_cmd"
static ssize_t
show_io_stat_kmsg(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	unsigned long flags;
	int i, j, cnt;
	unsigned int pos;
	struct seq_file m;
	io_stats_cmd_t *stat_cmd;
	ktime_t	lat;
	unsigned long long first_time, last_time;
	struct cmd_sta_tmp sta_tmp[CMD_TYPE_MAX];
	ktime_t	a;
	struct ufs_qcom_host *qcom_host = hba->priv;

	if (!cmd_buf) {
		seq_printf(&m, "cmd_buf is NULL\n");
		return m.count;
	}

	a = ktime_get();
	memset(&m, 0, sizeof(struct seq_file));
	memset(&sta_tmp, 0, sizeof(sta_tmp));
	for (j = 0; j < ARRAY_SIZE(sta_tmp); j++) {
		sta_tmp[j].lat_sum =  ktime_set(0, 0); /* init as 0 seconds */
		sta_tmp[j].lat_max =  ktime_set(0, 0); /* init as 0 seconds */
	}
	m.buf = buf;
	m.size = PAGE_SIZE;

	if (!qcom_host->io_stats.stat_cmd_enable) {
		seq_printf(&m, "io_stat_cmd is disabled\n");
		return m.count;
	}

	lat = ktime_set(0, 0); /* init as 0 seconds */
	stat_cmd = &qcom_host->io_stats.stat_cmd;
	pos = stat_cmd->pos;
	/* pr_err("[%s] pos %u.\n", UFS_CMD_STRING, pos); */
	for (i = 0, cnt = 0; i < UFS_STAT_CMD_COUNT; i++) {
		spin_lock_irqsave(&qcom_host->io_stats.lock, flags);
		cmd_buf[cnt] = stat_cmd->cmd[pos];
		spin_unlock_irqrestore(&qcom_host->io_stats.lock, flags);

		if (ktime_to_us(cmd_buf[cnt].tstamp)) {
			pr_err("[%s] lun %d cmd %x T %s lba=0x%llx len=%d tstamp=%lld lat=%lld us\n",
					UFS_CMD_STRING,
					cmd_buf[cnt].lun,
					cmd_buf[cnt].op,
					ufs_support_cmd_type_str[cmd_buf[cnt].cmd_type],
					(unsigned long long)cmd_buf[cnt].lba,
					cmd_buf[cnt].size,
					ktime_to_us(cmd_buf[cnt].tstamp),
					ktime_to_us(cmd_buf[cnt].time));

			lat = ktime_add(lat, cmd_buf[cnt].time);

			for (j = 0; j < ARRAY_SIZE(sta_tmp); j++) {
				if (cmd_buf[cnt].cmd_type == j) {
					sta_tmp[j].size_cnt += cmd_buf[cnt].size;
					sta_tmp[j].io_cnt++;
					sta_tmp[j].lat_sum = ktime_add(sta_tmp[j].lat_sum, cmd_buf[cnt].time);
					if (ktime_compare(cmd_buf[cnt].time, sta_tmp[j].lat_max) > 0)
						sta_tmp[j].lat_max = cmd_buf[cnt].time;
				}
			}

			cnt++;
		}
		pos = (pos + 1) % UFS_STAT_CMD_COUNT;
	}

	first_time = ktime_to_us(cmd_buf[0].tstamp);
	last_time = ktime_to_us(cmd_buf[cnt - 1].tstamp);

	pr_err("[%s] %-20s: %llu\n", UFS_CMD_STRING, "First time stamp(ns)", first_time);
	pr_err("[%s] %-20s: %llu\n", UFS_CMD_STRING, "Last time stamp(ns)", last_time);
	pr_err("[%s] %-20s: %llu\n", UFS_CMD_STRING, "Average latency(us)", ktime_to_us(lat) / cnt);
	pr_err("[%s] %-20s: %d\n", UFS_CMD_STRING, "IO counter", cnt);

	pr_err("[%s] %1s %8s %8s %16s %16s\n",
			UFS_CMD_STRING, "T", "Size", "Counter", "Lat-Avg(us)", "Lat-max(us)");
	for (j = 0; j < ARRAY_SIZE(sta_tmp); j++) {
		pr_err("[%s] %1s %8llu %8llu %16llu %16llu\n",
				UFS_CMD_STRING,
				ufs_support_cmd_type_str[j],
				sta_tmp[j].size_cnt,
				sta_tmp[j].io_cnt,
				ktime_to_us(sta_tmp[j].lat_sum) / sta_tmp[j].io_cnt,
				ktime_to_us(sta_tmp[j].lat_max));
	}

	pr_err("[%s] take_time %llu us.\n",
			UFS_CMD_STRING,
			ktime_to_us(ktime_sub(ktime_get(), a)));

	seq_printf(&m, "show_io_stats_kmsg successfully.\n");
	return m.count;
}
static DEVICE_ATTR(io_stat_kmsg, S_IRUGO, show_io_stat_kmsg, NULL);

static ssize_t
show_io_sum(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	unsigned long flags;
	int i, j, k;
	struct seq_file m;
	io_stats_info_t *stat;
	u_int64_t size;
	struct ufs_qcom_host *qcom_host = hba->priv;

	memset(&m, 0, sizeof(struct seq_file));
	m.buf = buf;
	m.size = PAGE_SIZE;

	for (k = 0; k < CMD_FLUSH; k++)
		for (j = 0; j < UFS_STAT_SIZE_MAX; j++) {
			if ((k == CMD_DISCARD) && (j != UFS_STAT_SIZE_OTHER))
				continue;

			size = 0;
			for (i = 0; i < UFS_STAT_GEAR_MAX; i++) {
				spin_lock_irqsave(&qcom_host->io_stats.lock, flags);
				stat = &qcom_host->io_stats.stat[i][j][k];
				size += stat->size_cnt;
				spin_unlock_irqrestore(&qcom_host->io_stats.lock, flags);
			}

			seq_printf(&m,
					"%s-%-4s: %16llu\n",
					ufs_support_cmd_type_str[k],
					ufs_support_size_str[j],
					size * 512
					);
		}

	return m.count;
}

static DEVICE_ATTR(io_sum, S_IRUGO | S_IWUSR,
		show_io_sum, set_io_stats);

static int cmd_cmp_func(const void *a, const void *b)
{
	io_ele_t *ia = &cmd_buf[*(u16 *)a];
	io_ele_t *ib = &cmd_buf[*(u16 *)b];

	return ktime_compare(ib->time, ia->time);
}

static void cmd_swap_func(void *a, void *b, int size)
{
	u16 t = *(u16 *)a;
	*(u16 *)a = *(u16 *)b;
	*(u16 *)b = t;
}

static ssize_t
show_io_stat_cmd(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	unsigned long flags;
	int i, j, id, cnt, max;
	unsigned int pos;
	struct seq_file m;
	io_stats_cmd_t *stat_cmd;
	ktime_t	lat;
	unsigned long long first_time, last_time;
	struct cmd_sta_tmp sta_tmp[CMD_TYPE_MAX];
	ktime_t	a;
	struct ufs_qcom_host *qcom_host = hba->priv;

	if (!cmd_buf || !idx_buf) {
		seq_printf(&m, "cmd_buf=%p idx_buf=%p\n", cmd_buf, idx_buf);
		return m.count;
	}

	a = ktime_get();
	memset(&m, 0, sizeof(struct seq_file));
	memset(&sta_tmp, 0, sizeof(sta_tmp));
	m.buf = buf;
	m.size = PAGE_SIZE;

	if (!qcom_host->io_stats.stat_cmd_enable) {
		seq_printf(&m, "io_stat_cmd is disabled\n");
		return m.count;
	}

	lat = ktime_set(0, 0); /* init as 0 seconds */
	stat_cmd = &qcom_host->io_stats.stat_cmd;
	pos = stat_cmd->pos;
	/* dev_err(hba->dev, "[%s:%d] pos %u.\n", __func__, __LINE__, pos); */
	for (i = 0, cnt = 0; i < UFS_STAT_CMD_COUNT; i++) {
		spin_lock_irqsave(&qcom_host->io_stats.lock, flags);
		cmd_buf[cnt] = stat_cmd->cmd[pos];
		spin_unlock_irqrestore(&qcom_host->io_stats.lock, flags);

		idx_buf[i] = i;
		if (ktime_to_us(cmd_buf[cnt].tstamp)) {
			/* pr_err("[%s] lun %d cmd %x T %s lba=0x%llx len=%d tstamp=%lld lat=%lld us\n", */
					/* "ufs_cmd", */
					/* cmd_buf[cnt].lun, */
					/* cmd_buf[cnt].op, */
					/* ufs_support_cmd_type_str[cmd_buf[cnt].cmd_type], */
					/* (unsigned long long)cmd_buf[cnt].lba, */
					/* cmd_buf[cnt].size, */
					/* ktime_to_us(cmd_buf[cnt].tstamp), */
					/* ktime_to_us(cmd_buf[cnt].time)); */

			lat = ktime_add(lat, cmd_buf[cnt].time);

			for (j = 0; j < ARRAY_SIZE(sta_tmp); j++) {
				if (cmd_buf[cnt].cmd_type == j) {
					sta_tmp[j].size_cnt += cmd_buf[cnt].size;
					sta_tmp[j].io_cnt++;
				}
			}

			cnt++;
		}
		pos = (pos + 1) % UFS_STAT_CMD_COUNT;
	}

	first_time = ktime_to_us(cmd_buf[0].tstamp);
	last_time = ktime_to_us(cmd_buf[cnt - 1].tstamp);
	sort(idx_buf, cnt, sizeof(idx_buf[0]), cmd_cmp_func, cmd_swap_func);

	seq_printf(&m, "%-20s: %llu\n", "First time stamp(ns)", first_time);
	seq_printf(&m, "%-20s: %llu\n", "Last time stamp(ns)", last_time);
	seq_printf(&m, "%-20s: %llu\n", "Average latency(us)", ktime_to_us(lat) / cnt);
	seq_printf(&m, "%-20s: %d\n", "IO counter", cnt);

	seq_printf(&m, "\n%1s %8s %8s\n", "T", "Size", "Counter");
	for (j = 0; j < ARRAY_SIZE(sta_tmp); j++) {
		seq_printf(&m, "%1s %8llu %8llu\n",
				ufs_support_cmd_type_str[j],
				sta_tmp[j].size_cnt,
				sta_tmp[j].io_cnt);
	}

	max = min(cnt, UFS_STAT_CMD_SYS_MAX_COUNT);
	seq_printf(&m,
			"\n%1s %1s %2s %8s %8s %16s %8s\n",
			"L",
			"T",
			"CM",
			"LBA",
			"Size",
			"Time Stamp",
			"Latency (us)");
	for (i = 0; i < max; i++) {
		id = idx_buf[i];
		seq_printf(&m,
				"%1d %1s %2x %8llx %8d %16llu %8llu\n",
				cmd_buf[id].lun,
				ufs_support_cmd_type_str[cmd_buf[id].cmd_type],
				cmd_buf[id].op,
				(unsigned long long)cmd_buf[id].lba,
				cmd_buf[id].size,
				ktime_to_us(cmd_buf[id].tstamp),
				ktime_to_us(cmd_buf[id].time));
	}
	dev_err(hba->dev, "[%s:%d] take_time %llu us.\n",
			__func__, __LINE__,
			ktime_to_us(ktime_sub(ktime_get(), a)));

	return m.count;
}

static DEVICE_ATTR(io_stat_cmd, S_IRUGO, show_io_stat_cmd, NULL);

static ssize_t
set_log_enable(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	int64_t value;
	unsigned long flags;
	struct ufs_qcom_host *qcom_host = hba->priv;

	if (kstrtoll(buf, 0, &value))
		return 0;

	spin_lock_irqsave(&qcom_host->io_stats.lock, flags);

	qcom_host->io_stats.log_enable = (unsigned int)value;

	spin_unlock_irqrestore(&qcom_host->io_stats.lock, flags);

	return count;
}

static DEVICE_ATTR(log_enable, S_IWUSR,
		NULL, set_log_enable);

static ssize_t
set_stat_cmd_enable(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	int64_t value;
	unsigned long flags;
	struct ufs_qcom_host *qcom_host = hba->priv;

	if (kstrtoll(buf, 0, &value))
		return 0;

	spin_lock_irqsave(&qcom_host->io_stats.lock, flags);

	qcom_host->io_stats.stat_cmd_enable = (unsigned int)value;

	spin_unlock_irqrestore(&qcom_host->io_stats.lock, flags);

	return count;
}

static DEVICE_ATTR(stat_cmd_enable, S_IWUSR,
		NULL, set_stat_cmd_enable);

static ssize_t
set_load_thre(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	int64_t value;
	unsigned long flags;
	struct ufs_qcom_host *qcom_host = hba->priv;

	if (kstrtoll(buf, 0, &value))
		return 0;

	spin_lock_irqsave(&qcom_host->io_stats.lock, flags);

	qcom_host->io_stats.load_thre = (unsigned int)value;

	spin_unlock_irqrestore(&qcom_host->io_stats.lock, flags);

	return count;
}

static ssize_t
show_load_thre(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufs_qcom_host *qcom_host = hba->priv;
	unsigned long flags;
	unsigned int load_thre;

	if (!ufshcd_is_clkscaling_supported(hba))
		return snprintf(buf, PAGE_SIZE, "clk_scaling is not enabled!\n");

	spin_lock_irqsave(&qcom_host->io_stats.lock, flags);
	load_thre = qcom_host->io_stats.load_thre;
	spin_unlock_irqrestore(&qcom_host->io_stats.lock, flags);

	return snprintf(buf, PAGE_SIZE, "%u\n", load_thre);
}

static DEVICE_ATTR(load_thre, S_IRUGO,
		show_load_thre, set_load_thre);

static ssize_t
show_load(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	unsigned long flags;
	unsigned int load;
	struct ufs_qcom_host *qcom_host = hba->priv;

	if (!ufshcd_is_clkscaling_supported(hba))
		return snprintf(buf, PAGE_SIZE, "clk_scaling is not enabled!\n");

	spin_lock_irqsave(&qcom_host->io_stats.lock, flags);
	load = qcom_host->io_stats.load;
	spin_unlock_irqrestore(&qcom_host->io_stats.lock, flags);

	return snprintf(buf, PAGE_SIZE, "%u\n", load);
}

static DEVICE_ATTR(load, S_IRUGO, show_load, NULL);

static int ufs_vend_scsi_execute(struct scsi_device *sdev, const unsigned char *cmd,
		 int data_direction, void *buffer, unsigned bufflen,
		 int timeout, int retries, int *resid,
		 u64 flags)
{
	char *sense = NULL;
	struct request *req;
	struct scsi_request *rq;
	int write = (data_direction == DMA_TO_DEVICE) ? REQ_OP_SCSI_OUT : REQ_OP_SCSI_IN;
	int ret = DRIVER_ERROR << 24;
	struct scsi_sense_hdr sshdr;
	ktime_t	a;

	a = ktime_get();

	sense = kzalloc(SCSI_SENSE_BUFFERSIZE, GFP_NOIO);
	if (!sense)
		return DRIVER_ERROR << 24;

	req = blk_get_request(sdev->request_queue, write, BLK_MQ_REQ_PREEMPT);
	if (IS_ERR(req)) {
		kfree(sense);
		return ret;
	}

	rq = scsi_req(req);

	if (bufflen &&	blk_rq_map_kern(sdev->request_queue, req,
					buffer, bufflen, __GFP_RECLAIM))
		goto out;

	rq->cmd_len = 16; /* The cmd length is nonstandard. */
	memcpy(rq->cmd, cmd, rq->cmd_len);
	rq->sense = sense;
	rq->sense_len = SCSI_SENSE_BUFFERSIZE;
	rq->retries = retries;
//	if (likely(!sdev->timeout_override))
	req->timeout = timeout;
//	else
//		req->timeout = sdev->timeout_override;

	req->cmd_flags |= flags;
	req->rq_flags |= RQF_QUIET | RQF_PREEMPT;

	/*
	 * head injection *required* here otherwise quiesce won't work
	 */
	blk_execute_rq(req->q, NULL, req, 1);

	/*
	 * Some devices (USB mass-storage in particular) may transfer
	 * garbage data together with a residue indicating that the data
	 * is invalid.  Prevent the garbage from being misinterpreted
	 * and prevent security leaks by zeroing out the excess data.
	 */
	if (unlikely(rq->resid_len > 0 && rq->resid_len <= bufflen))
		memset(buffer + (bufflen - rq->resid_len), 0, rq->resid_len);

	if (resid)
		*resid = rq->resid_len;
	ret = rq->result;
 out:
	blk_put_request(req);

	if (ret) {
		pr_err("%s: cmd %#x opcode %#x failed with err %#x\n",
				__func__, cmd[0], cmd[1], ret);

		scsi_normalize_sense(sense, SCSI_SENSE_BUFFERSIZE, &sshdr);
		scsi_print_sense_hdr(sdev, "VendorCmd", &sshdr);

		print_hex_dump(KERN_ERR, "sshdr: ", DUMP_PREFIX_OFFSET,
				16, 1, &sshdr, sizeof(sshdr), false);
		/* print_hex_dump(KERN_ERR, "VendorCmd: ", DUMP_PREFIX_OFFSET, */
				/* 16, 1, cmd, req->cmd_len, false); */
	}

	kfree(sense);

	pr_err("[%s] take_time %llu us.\n",
			UFS_CMD_STRING,
			ktime_to_us(ktime_sub(ktime_get(), a)));
	return ret;
}

static int ufs_vend_set_psw(struct ufs_hba *hba, struct scsi_device *sdp)
{
	unsigned char cmd[] = {UFS_VENDOR_COMMAND,
				UFS_VENDOR_OP_CODE_SET_PSW,
				GET_BYTE(UFS_VENDOR_PSW, 3),
				GET_BYTE(UFS_VENDOR_PSW, 2),
				GET_BYTE(UFS_VENDOR_PSW, 1),
				GET_BYTE(UFS_VENDOR_PSW, 0),
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	return ufs_vend_scsi_execute(sdp, cmd, DMA_NONE, NULL,
				0, msecs_to_jiffies(1000),
				UFS_VENDOR_MAX_RETRIES, NULL, 0);
}

static int ufs_vend_enter_mode_internal(struct ufs_hba *hba, struct scsi_device *sdp)
{
	unsigned char cmd[] = {UFS_VENDOR_COMMAND,
				UFS_VENDOR_OP_CODE_ENTER_MODE,
				GET_BYTE(UFS_VENDOR_SIGNATURE, 3),
				GET_BYTE(UFS_VENDOR_SIGNATURE, 2),
				GET_BYTE(UFS_VENDOR_SIGNATURE, 1),
				GET_BYTE(UFS_VENDOR_SIGNATURE, 0),
				GET_BYTE(UFS_VENDOR_PSW, 3),
				GET_BYTE(UFS_VENDOR_PSW, 2),
				GET_BYTE(UFS_VENDOR_PSW, 1),
				GET_BYTE(UFS_VENDOR_PSW, 0),
				0, 0, 0, 0, 0, 0};

	return ufs_vend_scsi_execute(sdp, cmd, DMA_NONE, NULL,
				0, msecs_to_jiffies(1000),
				UFS_VENDOR_MAX_RETRIES, NULL, 0);
}

static int ufs_vend_enter_mode(struct ufs_hba *hba, struct scsi_device *sdp)
{
	int ret;

	ret = ufs_vend_enter_mode_internal(hba, sdp);
	if (!ret)
		return ret;

	ret = ufs_vend_set_psw(hba, sdp);
	if (ret) {
		dev_err(hba->dev, "[%s:%d] ufs: Failed to set psw : %#x\n",
				__func__, __LINE__, ret);
	}

	return ufs_vend_enter_mode_internal(hba, sdp);
}

static int ufs_vend_exit_mode(struct ufs_hba *hba, struct scsi_device *sdp)
{
	unsigned char cmd[] = {UFS_VENDOR_COMMAND,
				UFS_VENDOR_OP_CODE_EXIT_MODE,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0};

	return ufs_vend_scsi_execute(sdp, cmd, DMA_NONE, NULL,
				0, msecs_to_jiffies(1000),
				UFS_VENDOR_MAX_RETRIES, NULL, 0);
}

static int ufs_vend_nand_info_report(struct ufs_hba *hba,
		struct scsi_device *sdp, char *buffer, int buf_len)
{
	unsigned char cmd[] = {UFS_VENDOR_COMMAND,
				UFS_VENDOR_OP_CODE_NAND_INFO_REPORT,
				0, 0,
				0x01, /* Read descriptor */
				0x0A, /* Nand info report */
				0, 0, 0, 0, 0, 0,
				0, 0, 0, buf_len};

	return ufs_vend_scsi_execute(sdp, cmd, DMA_FROM_DEVICE, buffer,
				buf_len, msecs_to_jiffies(1000),
				UFS_VENDOR_MAX_RETRIES, NULL, 0);
}

static ssize_t
get_nand_info_micron(struct ufs_hba *hba, struct scsi_device *sdp,
		struct nand_info *nif)
{
	struct nand_info_micron *nif_micron;
	int ret;
	unsigned char buffer[512] = {0};
	unsigned char cmd_write[] = {0x3b, 0xe1, 0, 0, 0, 0, 0, 0, 0x2c, 0};
	unsigned char cmd_read[] = {0x3c, 0xc1, 0, 0, 0, 0, 0, 0x02, 0, 0};
	unsigned char buf_o[44] = {0xfe, 0x40, 0, 0x10, 0x01, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	ret = ufs_vend_scsi_execute(sdp, cmd_write, DMA_TO_DEVICE, buf_o,
				44, msecs_to_jiffies(1000),
				UFS_VENDOR_MAX_RETRIES, NULL, 0);

	if (ret) {
		pr_err("[%s:%d] MICRON Failed to send write buffer cmd with err %#x\n",
				__func__, __LINE__, ret);
		return ret;
	}

	ret = ufs_vend_scsi_execute(sdp, cmd_read, DMA_FROM_DEVICE, buffer,
				512, msecs_to_jiffies(1000),
				UFS_VENDOR_MAX_RETRIES, NULL, 0);

	if (ret) {
		pr_err("[%s:%d] MICRON Failed to send read buffer cmd with err %#x\n",
				__func__, __LINE__, ret);
		return ret;
	}

	nif_micron = (struct nand_info_micron *)&buffer[0];
#if 0
	for (i = 0; i < 512; i += 16)
		pr_err(
	"[%s:%d]: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x ",
			__func__, __LINE__, buffer[i], buffer[i+1], buffer[i+2],
			buffer[i+3], buffer[i+4], buffer[i+5], buffer[i+6], buffer[i+7],
			buffer[i+8], buffer[i+9], buffer[i+10], buffer[i+11], buffer[i+12],
			buffer[i+13], buffer[i+14], buffer[i+15]);
#endif
	nif->max_erase = be32_to_cpu(nif_micron->maximum_block_erase_for_TLC);
	nif->min_erase = be32_to_cpu(nif_micron->minimum_block_erase_for_TLC);
	nif->avg_erase = be32_to_cpu(nif_micron->average_block_erase_for_TLC);
	nif->max_slc_erase_cycle = be32_to_cpu(nif_micron->maximum_block_erase_for_SLC);
	nif->min_slc_erase_cycle = be32_to_cpu(nif_micron->minimum_block_erase_for_SLC);
	nif->avg_slc_erase_cycle = be32_to_cpu(nif_micron->average_block_erase_for_SLC);
	nif->read_reclaim_cnt = be32_to_cpu(nif_micron->read_reclaim_count_for_TLC);
	nif->init_bad_blk = be16_to_cpu(nif_micron->factory_bad_block_count);
	nif->runtime_bad_blk = be16_to_cpu(nif_micron->run_time_bad_block_count);
	nif->remain_reserved_blk = be16_to_cpu(nif_micron->spare_block_count);
	nif->read_size = be32_to_cpu(nif_micron->read_data_size);
	nif->write_size = be32_to_cpu(nif_micron->written_data_size);
	nif->initialize_cnt = be32_to_cpu(nif_micron->initialization_count_success)
		+ be32_to_cpu(nif_micron->initialization_count_failure);
	nif->wai = be16_to_cpu(nif_micron->write_amplification_factor);

	/* extra info by MICRON */
	nif->reserved_block_count_SLC = be16_to_cpu(nif_micron->reserved_block_count_SLC);
	nif->reserved_block_count_TLC = be16_to_cpu(nif_micron->reserved_block_count_TLC);
	nif->exhausted_life_for_SLC = nif_micron->exhausted_life_for_SLC;
	nif->exhausted_life_for_TLC = nif_micron->exhausted_life_for_TLC;
	nif->meta_data_corruption = be16_to_cpu(nif_micron->meta_data_corruption);
	nif->initialization_count_success = be32_to_cpu(nif_micron->initialization_count_success);
	nif->initialization_count_failure = be32_to_cpu(nif_micron->initialization_count_failure);
	nif->read_reclaim_count_for_SLC = be32_to_cpu(nif_micron->read_reclaim_count_for_SLC);
	nif->read_reclaim_count_for_TLC = be32_to_cpu(nif_micron->read_reclaim_count_for_TLC);
	nif->SPOR_write_fail_count = be32_to_cpu(nif_micron->SPOR_write_fail_count);
	nif->SPOR_recovery_count = be32_to_cpu(nif_micron->SPOR_recovery_count);
	nif->VDET_count = be32_to_cpu(nif_micron->VDET_count);
	nif->UECC_count = be32_to_cpu(nif_micron->UECC_count);
	nif->read_retry_count = be32_to_cpu(nif_micron->read_retry_count);

	dev_err(hba->dev, "[%s:%d]: Read ufs MICRON nand_info ok\n",
			__func__, __LINE__);
	dev_err(hba->dev, "[%s:%d]: MICRON wai = %d\n",
			__func__, __LINE__, nif_micron->write_amplification_factor);

	return 0;
}

static ssize_t
get_nand_info_hynix(struct ufs_hba *hba, struct scsi_device *sdp,
		struct nand_info *nif)
{
	u8 desc_buf[QUERY_DESC_HEALTH_DEF_SIZE];
	struct nand_info_hynix *nif_hynix;
	int ret;

	pm_runtime_get_sync(hba->dev);
	ret = ufshcd_read_health_desc(hba, desc_buf,
			QUERY_DESC_HEALTH_DEF_SIZE);
	pm_runtime_put_sync(hba->dev);
	if (ret) {
		pr_err("[%s:%d] Hynix Failed to read health descriptor with err %#x\n",
				__func__, __LINE__, ret);

		memset(desc_buf, 0, sizeof(desc_buf));

		return ret;
	}

	nif_hynix = (struct nand_info_hynix *)&desc_buf[0];

	/* print_hex_dump(KERN_ERR, "nand_info_hynix: ", DUMP_PREFIX_OFFSET, */
			/* 16, 1, desc_buf, sizeof(desc_buf), false); */

	nif->init_bad_blk     = be16_to_cpu(nif_hynix->init_bad_blk);
	nif->runtime_bad_blk  = be16_to_cpu(nif_hynix->runtime_bad_blk);
	nif->max_erase        = be16_to_cpu(nif_hynix->max_erase);
	nif->min_erase        = be16_to_cpu(nif_hynix->min_erase);
	nif->avg_erase        = be16_to_cpu(nif_hynix->avg_erase);
	nif->read_reclaim_cnt = be16_to_cpu(nif_hynix->read_reclaim_cnt);
	nif->spo_cnt          = be32_to_cpu(nif_hynix->spo_cnt);
	nif->write_size       = be32_to_cpu(nif_hynix->write_size);
	nif->lvd_cnt          = be32_to_cpu(nif_hynix->lvd_cnt);
	nif->ffu_success_cnt  = be16_to_cpu(nif_hynix->ffu_success_cnt);

/*
	seq_printf(m,
			"Hynix nand_info:\n"
			"ufsid    : %s"
			"model    : %.16s\n"
			"fw_ver   : %4.4s\n"
			"life_a   : %d\n"
			"life_b   : %d\n"
			"life_eof : %d\n"
			"itbb     : %d\n"
			"rtbb     : %d\n"
			"max_mlc  : %d\n"
			"min_mlc  : %d\n"
			"avg_mlc  : %d\n"
			"rd_recl  : %d\n"
			"spo_cnt  : %d\n"
			"w_size   : %d\n"
			"lvd_cnt  : %d\n"
			"ffu_cnt  : %d\n"
			,
			qcom_host->ufs_vinfo.ufsid,
			sdp->model,
			sdp->rev,
			nif->life_a,
			nif->life_b,
			nif->life_eof,
			nif->init_bad_blk,
			nif->runtime_bad_blk,
			nif->max_mlc_erase_cycle,
			nif->min_mlc_erase_cycle,
			nif->avg_mlc_erase_cycle,
			nif->read_reclaim_cnt,
			nif->spo_cnt,
			nif->write_size,
			nif->lvd_cnt,
			nif->ffu_success_cnt
			);
	*/

	dev_err(hba->dev, "[%s:%d]: Read ufs vend nand info ok\n",
			__func__, __LINE__);

	return 0;
}

static ssize_t
get_nand_info_samsung(struct ufs_hba *hba, struct scsi_device *sdp,
		struct nand_info *nif)
{
	char *buffer;
	struct nand_info_ss *nif_ss;
	int ret;

	ret = ufs_vend_enter_mode(hba, sdp);
	if (ret) {
		dev_err(hba->dev, "Failed to enter vendor mode : %#x\n", ret);
		goto out;
	}

	buffer = kzalloc(sizeof(struct nand_info_ss), GFP_KERNEL);
	if (!buffer) {
		ret = -ENOMEM;
		goto exit_vend;
	}

	ret = ufs_vend_nand_info_report(hba, sdp, buffer,
			UFS_VENDOR_NAND_INFO_SIZE);
	if (ret) {
		dev_err(hba->dev, "Failed to read nand info : %#x\n", ret);
		goto free_buf;
	}

	nif_ss = (struct nand_info_ss *)buffer;
	nif->max_slc_erase_cycle     = be32_to_cpu(nif_ss->max_slc_erase_cycle);
	nif->min_slc_erase_cycle     = be32_to_cpu(nif_ss->min_slc_erase_cycle);
	nif->avg_slc_erase_cycle     = be32_to_cpu(nif_ss->avg_slc_erase_cycle);
	nif->max_mlc_erase_cycle     = be32_to_cpu(nif_ss->max_mlc_erase_cycle);
	nif->min_mlc_erase_cycle     = be32_to_cpu(nif_ss->min_mlc_erase_cycle);
	nif->avg_mlc_erase_cycle     = be32_to_cpu(nif_ss->avg_mlc_erase_cycle);
	nif->read_reclaim_cnt        = be32_to_cpu(nif_ss->read_reclaim_cnt);
	nif->init_bad_blk            = be32_to_cpu(nif_ss->init_bad_blk);
	nif->runtime_bad_blk         = be32_to_cpu(nif_ss->runtime_bad_blk);
	nif->remain_reserved_blk     = be32_to_cpu(nif_ss->remain_reserved_blk);
	nif->required_recovery_level = be32_to_cpu(nif_ss->required_recovery_level);
	nif->write_size              = be32_to_cpu(nif_ss->write_size);
	nif->initialize_cnt          = be32_to_cpu(nif_ss->initialize_cnt);
	nif->ffu_success_cnt         = be32_to_cpu(nif_ss->ffu_success_cnt);

	dev_err(hba->dev, "[%s:%d]: Samsung read ufs vend nand info ok\n",
			__func__, __LINE__);

free_buf:
	kfree(buffer);
exit_vend:
	ret = ufs_vend_exit_mode(hba, sdp);
	if (ret)
		dev_err(hba->dev,
				"[%s:%d] ufs: Failed to exit vendor mode : %#x\n",
				__func__, __LINE__, ret);
out:
	return ret;
}

static int ufs_ven_tw_lifetime(struct ufs_hba *hba, u8 lun, u32 *val)
{
	int err;

	pm_runtime_get_sync(hba->dev);

	err = ufshcd_query_attr_retry_ven(hba, UPIU_QUERY_OPCODE_READ_ATTR, 0x1E,
			lun, 1, val); /* UFSFEATURE_SELECTOR is 1 */
	if (err) {
		*val = 0;

		dev_err(hba->dev, "[%s:%d] tw lifetime read failed %d \n",
				__func__, __LINE__, err);
	}

	dev_err(hba->dev, "[%s:%d] tw lifetime : %#x \n",
			__func__, __LINE__, *val);
	pm_runtime_put_sync(hba->dev);
	return err;
}

#define		UFS_NAND_INFO_OUTPUT_DEC		0
#define		UFS_NAND_INFO_OUTPUT_HEX		1
#define		UFS_NAND_INFO_OUTPUT_HEX_CLOUD		2
/* global variable stores the previous write size value */
u32 pre_w_size;

static ssize_t
__show_nand_info(struct ufs_hba *hba, char *buf, int format)
{
	struct scsi_device *sdp;
	unsigned long flags;
	struct seq_file m;
	struct nand_info *nif;
	struct nand_info *global_nif;
	struct scsi_disk *sdkp;
	struct ufs_qcom_host *qcom_host = hba->priv;
	uint32_t  avg_erase;
	int i, j, k;
	u64 lun;
	u8 desc_buf[QUERY_DESC_HEALTH_DEF_SIZE];
	int ret;
	ktime_t	a;
	u32 lifetime;
	u_int64_t stat_warn_io_cnt;

	pr_err("%s read the nand info\n", current->comm);

	a = ktime_get();
	global_nif = qcom_host->io_stats.nif;
	nif = kmalloc(sizeof(struct nand_info), GFP_KERNEL);
	if (!nif || !global_nif) {
		seq_printf(&m, "struct nand_info is NULL!\n");
		goto out;
	}
	memset(nif, 0, sizeof(struct nand_info));

	memset(&m, 0, sizeof(struct seq_file));
	m.buf = buf;
	m.size = PAGE_SIZE;

	spin_lock_irqsave(hba->host->host_lock, flags);

	for (i = 0; i < ARRAY_SIZE(qcom_host->sdev_ufs_lu_ven); i++) {
		sdp = qcom_host->sdev_ufs_lu_ven[i];
		if (sdp) {
			ret = scsi_device_get(sdp);
			if (!ret && !scsi_device_online(sdp)) {
				ret = -ENODEV;
				scsi_device_put(sdp);
				break;
			}

			sdkp = dev_get_drvdata(&sdp->sdev_gendev);
			if (sdkp->capacity <= UFS_CAPACITY_32GB)
				scsi_device_put(sdp); /* Just mtk platform put lu0/lu1  */
			else
				break; /* Find it */
		} else {
			ret = -ENODEV;
			break;
		}
	}
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	if (ret) {
		seq_printf(&m, "Failed to get device %#x\n", ret);
		goto out;
	}

	pm_runtime_get_sync(hba->dev);
	mutex_lock(&qcom_host->sdev_ufs_rpmb->state_mutex);
	pr_err("qcom_host->sdev_ufs_rpmb->sdev_gendev.power.runtime_status:%d\n",
			qcom_host->sdev_ufs_rpmb->sdev_gendev.power.runtime_status);
	if (qcom_host->sdev_ufs_rpmb->sdev_gendev.power.runtime_status != RPM_ACTIVE) {
		switch (qcom_host->ufs_vinfo.w_manufacturer_id) {
		case UFS_VENDOR_SKHYNIX:
			ret = get_nand_info_hynix(hba, sdp, nif);
			break;
		case UFS_VENDOR_SAMSUNG:
			/* Ensure device is resumed before scsi operation */
			scsi_autopm_get_device(sdp);
			ret = get_nand_info_samsung(hba, sdp, nif);
			/* Allow device to be runtime suspended */
			scsi_autopm_put_device(sdp);
			break;
		case UFS_VENDOR_MICRON:
			scsi_autopm_get_device(sdp);
			ret = get_nand_info_micron(hba, sdp, nif);
			scsi_autopm_put_device(sdp);
			break;
		default:
			ret = -EINVAL;
			break;
		}
	}
	mutex_unlock(&qcom_host->sdev_ufs_rpmb->state_mutex);

	if (ret)
		pr_err("[%s:%d] Failed to read nand_info, w_manufacturer_id 0x%04x, ret %d\n",
				__func__, __LINE__, qcom_host->ufs_vinfo.w_manufacturer_id, ret);

	ret = ufshcd_read_health_desc(hba, desc_buf,
			QUERY_DESC_HEALTH_DEF_SIZE);
	pm_runtime_put_sync(hba->dev);
	if (ret) {
		pr_err("%s: Failed to read health descriptor with err %#x\n",
				__func__, ret);
		memset(desc_buf, 0, sizeof(desc_buf));
	}

	nif->version = UFS_VENDOR_VERSION | UFS_NAND_VERSION_FLAG
		| ((uint64_t)qcom_host->ufs_vinfo.wspecversion << UFS_VERSION_SHIFT);
	nif->deviceid = qcom_host->ufs_vinfo.ufsid;
	nif->model = sdp->model;
	nif->fw_ver = sdp->rev;
	nif->life_eof = desc_buf[2];
	nif->life_a = desc_buf[3];
	nif->life_b = desc_buf[4];
	nif->capacity = sdkp->capacity; /* unit: 4KB */
	if (nif->avg_erase)
		avg_erase = nif->avg_erase;
	else
		avg_erase = nif->avg_mlc_erase_cycle;

	if (avg_erase && nif->write_size && sdkp->capacity) {
		if (nif->model[4] == '7' //if the product is SATURN controller
			|| !strncmp(nif->model, "KLUCG2K1EA-B0C1", 15)
			|| !strncmp(nif->model, "KLUDG4U1EA-B0C1", 15)
			|| !strncmp(nif->model, "KLUEG8U1EA-B0C1", 15)
			|| qcom_host->ufs_vinfo.w_manufacturer_id != UFS_VENDOR_SAMSUNG)
			//convert 4KB unit to 100MB unit. Wai should enlarge magnify 100 times so that display as float.
			nif->wai = (((avg_erase + 1) * (uint64_t)sdkp->capacity) * 4 * KB / MB) * 100
				/ (nif->write_size * 100);
		else {
			nif->wai = (((avg_erase + 1) * (uint64_t)sdkp->capacity) * 4 * KB / MB) * 100
				/ (nif->write_size * 10);
			nif->write_size /= 10;
		}
	} else {
		nif->wai = 0;
		if (!(nif->model[4] == '7' //if the product is SATURN controller
			|| !strncmp(nif->model, "KLUCG2K1EA-B0C1", 15)
			|| !strncmp(nif->model, "KLUDG4U1EA-B0C1", 15)
			|| !strncmp(nif->model, "KLUEG8U1EA-B0C1", 15)
			|| qcom_host->ufs_vinfo.w_manufacturer_id != UFS_VENDOR_SAMSUNG))
			nif->write_size /= 10;
	}
	dev_err(hba->dev, "[%s:%d]: LU%d capacity=%lld blocks, write_size %d wai=%d\n",
			__func__, __LINE__, sdp->lun,
			(uint64_t)sdkp->capacity, nif->write_size, nif->wai);

	lun = sdp->lun;
	ufs_ven_tw_lifetime(hba, lun, &lifetime);

	scsi_device_put(sdp);

	if (format == UFS_NAND_INFO_OUTPUT_HEX)
		seq_printf(&m,
			"version        : %llx\n"
			"deviceid       : 0\n"
			"model          : %.16s\n"
			"fw_ver         : %4.4s\n"
			"life_a         : %02x\n"
			"life_b         : %02x\n"
			"life_eof       : %02x\n"
			"max_erase      : %08x\n"
			"min_erase      : %08x\n"
			"avg_erase      : %08x\n"
			"max_slc        : %08x\n"
			"min_slc        : %08x\n"
			"avg_slc        : %08x\n"
			"max_mlc        : %08x\n"
			"min_mlc        : %08x\n"
			"avg_mlc        : %08x\n"
			"rd_recl        : %08x\n"
			"itbb           : %08x\n"
			"rtbb           : %08x\n"
			"rsv_blk        : %08x\n"
			"ffu_try        : %08x\n"
			"ffu_cnt        : %08x\n"
			"init_cnt       : %08x\n"
			"w_size         : %08x\n"
			"r_size         : %08x\n"
			"lvd_cnt        : %08x\n"
			"slc_rsv        : %08x\n"
			"mlc_rsv        : %08x\n"
			"spo_cnt        : %08x\n"
			"slc_tl_er      : %08x\n"
			"mlc_tl_er      : %08x\n"
			"capacity       : %d.%01dGB\n"
			"wai            : %lld.%02lld\n"
			/* add for MICRON */
			"rsv_blk_SLC    : %08x\n"
			"rsv_blk_TLC    : %08x\n"
			"ex_life_SLC    : %08x\n"
			"ex_life_TLC    : %08x\n"
			"meta_data_crpt : %08x\n"
			"init_succ      : %08x\n"
			"init_fail      : %08x\n"
			"rd_rcl_SLC     : %08x\n"
			"rd_rcl_TLC     : %08x\n"
			"SPOR_wr_fail   : %08x\n"
			"SPOR_rc_cnt    : %08x\n"
			"VDET_cnt       : %08x\n"
			"UECC_cnt       : %08x\n"
			"rd_retry_cnt   : %08x\n"
			,
			nif->version,
			nif->model,
			nif->fw_ver,
			nif->life_a,
			nif->life_b,
			nif->life_eof,
			nif->max_erase,
			nif->min_erase,
			nif->avg_erase,
			nif->max_slc_erase_cycle,
			nif->min_slc_erase_cycle,
			nif->avg_slc_erase_cycle,
			nif->max_mlc_erase_cycle,
			nif->min_mlc_erase_cycle,
			nif->avg_mlc_erase_cycle,
			nif->read_reclaim_cnt,
			nif->init_bad_blk,
			nif->runtime_bad_blk,
			nif->remain_reserved_blk,
			nif->ffu_try,
			nif->ffu_success_cnt,
			nif->initialize_cnt,
			nif->write_size,
			nif->read_size,
			nif->lvd_cnt,
			nif->slc_rsv,
			nif->mlc_rsv,
			nif->spo_cnt,
			nif->slc_tl_er,
			nif->mlc_tl_er,
			nif->capacity >> 18, (nif->capacity / 26214) % 10,
			nif->wai / 100, nif->wai % 100,
			/* add for MICRON */
			nif->reserved_block_count_SLC,
			nif->reserved_block_count_TLC,
			nif->exhausted_life_for_SLC,
			nif->exhausted_life_for_TLC,
			nif->meta_data_corruption,
			nif->initialization_count_success,
			nif->initialization_count_failure,
			nif->read_reclaim_count_for_SLC,
			nif->read_reclaim_count_for_TLC,
			nif->SPOR_write_fail_count,
			nif->SPOR_recovery_count,
			nif->VDET_count,
			nif->UECC_count,
			nif->read_retry_count
			);
	else if (format == UFS_NAND_INFO_OUTPUT_HEX_CLOUD) {
		u32 daily_w_size;
		unsigned long long t = sched_clock();

		do_div(t, 1000000000); /* seconds */
		do_div(t, 3600); /* hours */

		if (((t / 23) >= 1) && (pre_w_size > 0)) /* meet the cat time(24 hours) */
			daily_w_size = nif->write_size - pre_w_size;
		else
			daily_w_size = 0; /* just set 0 when meet not the cat time */

		/* update the global w_size for next calc */
		pre_w_size = nif->write_size;

		seq_printf(&m,
			"deviceid       : 0\n"
			"mdl            : %.16s\n"
			"fw_ver         : %4.4s\n"
			"pa_err_cnt     : %d\n"
			"dl_err_cnt     : %d\n"
			"linereset_cnt  : %d\n"
			"life_a         : %d\n"
			"life_b         : %d\n"
			"life_eof       : %d\n"
			"tw_lifetime    : %d\n"
			"diff_erase     : %d\n"
			"avg_erase      : %d\n"
			"diff_slc_erase : %d\n"
			"avg_slc        : %d\n"
			"diff_mlc_erase : %d\n"
			"avg_mlc        : %d\n"
			"rd_recl        : %d\n"
			"rtbb           : %d\n"
			"rsv_blk        : %d\n"
			"ffu_cnt        : %d\n"
			"init_cnt       : %d\n"
			"w_size         : %d\n"
			"daily_w_size   : %d\n"
			"r_size         : %d\n"
			"lvd_cnt        : %d\n"
			"spo_cnt        : %d\n"
			"wai            : %lld.%02lld\n"
			,
			nif->model,
			nif->fw_ver,
			hba->ufs_stats.pa_err_cnt_total,
			hba->ufs_stats.dl_err_cnt_total,
			hba->ufs_stats.pa_err_cnt[UFS_EC_PA_LINE_RESET],
			nif->life_a,
			nif->life_b,
			nif->life_eof,
			lifetime,
			nif->max_erase - nif->min_erase,
			nif->avg_erase,
			nif->max_slc_erase_cycle - nif->min_slc_erase_cycle,
			nif->avg_slc_erase_cycle,
			nif->max_mlc_erase_cycle - nif->min_mlc_erase_cycle,
			nif->avg_mlc_erase_cycle,
			nif->read_reclaim_cnt,
			nif->runtime_bad_blk,
			nif->remain_reserved_blk,
			nif->ffu_success_cnt,
			nif->initialize_cnt,
			nif->write_size,
			daily_w_size,
			nif->read_size,
			nif->lvd_cnt,
			nif->spo_cnt,
			nif->wai / 100, nif->wai % 100
			);
	} else
		seq_printf(&m,
			"version        : %llx\n"
			"deviceid       : 0\n"
			"model          : %.16s\n"
			"fw_ver         : %4.4s\n"
			"life_a         : %d\n"
			"life_b         : %d\n"
			"life_eof       : %d\n"
			"max_erase      : %d\n"
			"min_erase      : %d\n"
			"avg_erase      : %d\n"
			"max_slc        : %d\n"
			"min_slc        : %d\n"
			"avg_slc        : %d\n"
			"max_mlc        : %d\n"
			"min_mlc        : %d\n"
			"avg_mlc        : %d\n"
			"rd_recl        : %d\n"
			"itbb           : %d\n"
			"rtbb           : %d\n"
			"rsv_blk        : %d\n"
			"ffu_try        : %d\n"
			"ffu_cnt        : %d\n"
			"init_cnt       : %d\n"
			"w_size         : %d\n"
			"r_size         : %d\n"
			"lvd_cnt        : %d\n"
			"slc_rsv        : %d\n"
			"mlc_rsv        : %d\n"
			"spo_cnt        : %d\n"
			"slc_tl_er      : %d\n"
			"mlc_tl_er      : %d\n"
			"capacity       : %d.%01dGB\n"
			"wai            : %lld.%02lld\n"
			/* add for MICRON */
			"rsv_blk_SLC    : %d\n"
			"rsv_blk_TLC    : %d\n"
			"ex_life_SLC    : %d\n"
			"ex_life_TLC    : %d\n"
			"meta_data_crpt : %d\n"
			"init_succ      : %d\n"
			"init_fail      : %d\n"
			"rd_rcl_SLC     : %d\n"
			"rd_rcl_TLC     : %d\n"
			"SPOR_wr_fail   : %d\n"
			"SPOR_rc_cnt    : %d\n"
			"VDET_cnt       : %d\n"
			"UECC_cnt       : %d\n"
			"rd_retry_cnt   : %d\n"
			,
			nif->version,
			nif->model,
			nif->fw_ver,
			nif->life_a,
			nif->life_b,
			nif->life_eof,
			nif->max_erase,
			nif->min_erase,
			nif->avg_erase,
			nif->max_slc_erase_cycle,
			nif->min_slc_erase_cycle,
			nif->avg_slc_erase_cycle,
			nif->max_mlc_erase_cycle,
			nif->min_mlc_erase_cycle,
			nif->avg_mlc_erase_cycle,
			nif->read_reclaim_cnt,
			nif->init_bad_blk,
			nif->runtime_bad_blk,
			nif->remain_reserved_blk,
			nif->ffu_try,
			nif->ffu_success_cnt,
			nif->initialize_cnt,
			nif->write_size,
			nif->read_size,
			nif->lvd_cnt,
			nif->slc_rsv,
			nif->mlc_rsv,
			nif->spo_cnt,
			nif->slc_tl_er,
			nif->mlc_tl_er,
			nif->capacity >> 18, (nif->capacity / 26214) % 10,
			nif->wai / 100, nif->wai % 100,
			/* add for MICRON */
			nif->reserved_block_count_SLC,
			nif->reserved_block_count_TLC,
			nif->exhausted_life_for_SLC,
			nif->exhausted_life_for_TLC,
			nif->meta_data_corruption,
			nif->initialization_count_success,
			nif->initialization_count_failure,
			nif->read_reclaim_count_for_SLC,
			nif->read_reclaim_count_for_TLC,
			nif->SPOR_write_fail_count,
			nif->SPOR_recovery_count,
			nif->VDET_count,
			nif->UECC_count,
			nif->read_retry_count
			);

	for (j = 0; j < UFS_STAT_SIZE_MAX; j++)
		for (k = 0; k < CMD_TYPE_MAX; k++) {
			s64 time = 0;
			s64 io_cnt = 0;
			s64 lat_max = 0;
			u_int64_t stat_io_cnt;
			io_stats_info_t *stat;

			spin_lock_irqsave(&qcom_host->io_stats.lock, flags);
			stat_io_cnt = qcom_host->io_stats.stat_io_cnt;
			for (i = 0; i < UFS_STAT_GEAR_MAX; i++) {
				stat = &qcom_host->io_stats.stat[i][j][k];
				time += ktime_to_us(stat->lat);
				io_cnt += stat->io_cnt;
				lat_max = max(ktime_to_us(stat->lat_max), lat_max);
			}
			spin_unlock_irqrestore(&qcom_host->io_stats.lock, flags);

			if (k <= CMD_WRITE) {/* read or write */
				char *pad = "";

				if (j == UFS_STAT_SIZE_4KB)
					pad = "  ";

				seq_printf(&m,
						"%s_%s_lat_avg %s: %lld\n"
						"%s_%s_lat_max %s: %lld\n"
						"%s_%s_pct     %s: %lld\n"
						,
						ufs_support_size_str[j], ufs_support_cmd_type_str[k], pad, time / io_cnt,
						ufs_support_size_str[j], ufs_support_cmd_type_str[k], pad, lat_max,
						ufs_support_size_str[j], ufs_support_cmd_type_str[k], pad,
						(io_cnt * 100) / stat_io_cnt
						);
			} else if (((k == CMD_DISCARD) || (k == CMD_FLUSH)) && (j == UFS_STAT_SIZE_OTHER))
				seq_printf(&m,
						"%s_lat_avg      : %lld\n"
						"%s_lat_max      : %lld\n"
						,
						ufs_support_cmd_type_str[k], time / io_cnt,
						ufs_support_cmd_type_str[k], lat_max);
		}

	stat_warn_io_cnt = qcom_host->io_stats.stat_warn_io_cnt;
	seq_printf(&m, "warn_io_cnt    : %d\n", stat_warn_io_cnt);

	if (format != UFS_NAND_INFO_OUTPUT_HEX_CLOUD) {
		seq_printf(&m, "rrl            : %d\n", nif->required_recovery_level);
		seq_printf(&m, "lu_enable      : %d\n", ufs_ven_lu0_lu_enable);
		seq_printf(&m, "tw_lu_buf_size : %d\n", ufs_ven_lu0_tw_lu_buf_size);
		seq_printf(&m, "tw_lifetime    : %u\n", lifetime);
	}
	memcpy(global_nif, nif, sizeof(struct nand_info));

	dev_err(hba->dev, "[%s:%d]: Read ufs vend nand info ok\n", __func__, __LINE__);
out:
	if (nif)
		kfree(nif);
	dev_err(hba->dev, "[%s:%d] take_time %llu us.\n",
			__func__, __LINE__,
			ktime_to_us(ktime_sub(ktime_get(), a)));
	return m.count;
}

static ssize_t
show_nand_info_hex(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	return __show_nand_info(hba, buf, UFS_NAND_INFO_OUTPUT_HEX);
}
static DEVICE_ATTR(nand_info, S_IRUGO, show_nand_info_hex, NULL);

static ssize_t
show_nand_info_dec(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	return __show_nand_info(hba, buf, UFS_NAND_INFO_OUTPUT_DEC);
}
static DEVICE_ATTR(nand_info_dec, S_IRUGO, show_nand_info_dec, NULL);

static ssize_t
show_nand_info_hex_cloud(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	return __show_nand_info(hba, buf, UFS_NAND_INFO_OUTPUT_HEX_CLOUD);
}
static DEVICE_ATTR(nand_info_cloud, S_IRUGO, show_nand_info_hex_cloud, NULL);

static struct attribute *dev_attrs_io_stats[] = {
	&dev_attr_io_stats.attr,
	&dev_attr_io_sum.attr,
	&dev_attr_log_enable.attr,
	&dev_attr_load.attr,
	&dev_attr_load_thre.attr,
	&dev_attr_nand_info.attr,
	&dev_attr_nand_info_dec.attr,
	&dev_attr_nand_info_cloud.attr,
	&dev_attr_io_stat_cmd.attr,
	&dev_attr_io_stat_kmsg.attr,
	&dev_attr_stat_cmd_enable.attr,
	NULL,
};

static struct attribute_group dev_attr_grp_io_stats = {
	.name = "ufs_io_stats",
	.attrs = dev_attrs_io_stats,
};

void ufs_io_stats_sysfs_create(struct ufs_hba *hba)
{
	int err;

	err = sysfs_create_group(&hba->dev->kobj, &dev_attr_grp_io_stats);
	if (err)
		pr_err("%s: failed to create sysfs group with err %#x\n",
				__func__, err);
}

void ufs_io_stats_sysfs_remove(struct ufs_hba *hba)
{
	sysfs_remove_group(&hba->dev->kobj, &dev_attr_grp_io_stats);
}

static void ufshcd_io_stats_init_intern(struct ufs_hba *hba)
{
	int i, j, k;
	io_stats_info_t *stat;
	unsigned long flags;
	struct ufs_qcom_host *qcom_host = hba->priv;

	spin_lock_irqsave(&qcom_host->io_stats.lock, flags);

	memset(&qcom_host->io_stats.stat, 0,
			(UFS_STAT_GEAR_MAX * UFS_STAT_SIZE_MAX
			 * CMD_TYPE_MAX * sizeof(io_stats_info_t)));

	for (i = 0; i < UFS_STAT_GEAR_MAX; i++)
		for (j = 0; j < UFS_STAT_SIZE_MAX; j++)
			for (k = 0; k < CMD_TYPE_MAX; k++) {
				stat = &qcom_host->io_stats.stat[i][j][k];
				stat->lat_min = ktime_set(1000, 0); /* init as 1000 seconds */
				stat->lat_max = ktime_set(0, 0); /* init as 0 seconds */
			}

	spin_unlock_irqrestore(&qcom_host->io_stats.lock, flags);
}


void ufshcd_io_stats_get_load(struct ufs_hba *hba,
		struct devfreq_dev_status *status)
{
	unsigned long flags;
	struct ufs_qcom_host *qcom_host = hba->priv;

	spin_lock_irqsave(&qcom_host->io_stats.lock, flags);
	qcom_host->io_stats.load = (status->busy_time * 100) / status->total_time;
	spin_unlock_irqrestore(&qcom_host->io_stats.lock, flags);

	if ((qcom_host->io_stats.load > 90)
			&& __ratelimit(&qcom_host->io_stats.ratelimit))
		dev_err(hba->dev, "[Warn] Load is high %u%% : total=%lu busy=%lu\n",
				qcom_host->io_stats.load,
				status->total_time,
				status->busy_time);
}

void ufshcd_scsi_device0_get(struct ufs_hba *hba, struct scsi_device *sdev)
{
	struct ufs_qcom_host *qcom_host = hba->priv;

	if (sdev->lun < ARRAY_SIZE(qcom_host->sdev_ufs_lu_ven))
		qcom_host->sdev_ufs_lu_ven[sdev->lun] = sdev;
}

void ufshcd_scsi_device_rpmb_get(struct ufs_hba *hba, struct scsi_device *sdev)
{
	struct ufs_qcom_host *qcom_host = hba->priv;

	qcom_host->sdev_ufs_rpmb = sdev;
}

bool ufshcd_is_wb_need_enable(struct ufs_hba *hba)
{
	struct ufs_qcom_host *qcom_host = hba->priv;

	if ((qcom_host && qcom_host->io_stats.load > qcom_host->io_stats.load_thre)
			|| !hba->clk_scaling.is_allowed)
		return true;

	return false;
}

void ufshcd_io_stats_init(struct ufs_hba *hba)
{
	struct ufs_qcom_host *qcom_host = hba->priv;

	spin_lock_init(&qcom_host->io_stats.lock);
	ufs_io_stats_sysfs_create(hba);
	ufshcd_io_stats_init_intern(hba);

	qcom_host->io_stats.stat_cmd_enable = true;
	/* qcom_host->io_stats.log_enable = true; */
	qcom_host->io_stats.load_thre = 50;

	cmd_buf = kmalloc(UFS_STAT_CMD_COUNT * sizeof(io_ele_t), GFP_KERNEL);
	if (!cmd_buf)
		dev_err(hba->dev, "Failed to alloc cmd_buf!\n");

	idx_buf = kmalloc(UFS_STAT_CMD_COUNT * sizeof(u16), GFP_KERNEL);
	if (!idx_buf)
		dev_err(hba->dev, "Failed to alloc idx_buf!\n");

	qcom_host->io_stats.nif = kzalloc(sizeof(struct nand_info), GFP_KERNEL);
	if (!qcom_host->io_stats.nif)
		dev_err(hba->dev, "[%s:%d] Failed to alloc buffer for struct nand_info!\n",
				__func__, __LINE__);

	/* Not more than 5 times every 5 seconds. */
	ratelimit_state_init(&qcom_host->io_stats.ratelimit,
			DEFAULT_RATELIMIT_INTERVAL, 5);
}

void ufshcd_io_stats_remove(struct ufs_hba *hba)
{
	ufs_io_stats_sysfs_remove(hba);

	kfree(cmd_buf);
	kfree(idx_buf);
}

#else

void ufshcd_scsi_device0_get(struct ufs_hba *hba, struct scsi_device *sdev)
{
}

void ufshcd_io_stats_init(struct ufs_hba *hba)
{
}

void ufshcd_io_stats_remove(struct ufs_hba *hba)
{
}

void ufshcd_io_stats(struct ufs_hba *hba, struct ufshcd_lrb *lrbp, unsigned int tag)
{
}

void ufshcd_io_stats_get_load(struct ufs_hba *hba,
		struct devfreq_dev_status *status)
{
}

#endif

static u8 *ufsid_pointer;

static int ufs_debug_snprint (char **pp, int *left_size,  const char *fmt, ...)
{
	va_list args;
	char *p = *pp;
	int size;

	if (*left_size <= 1) {
		pr_err("[%s:%d] buf is full\n", __func__, __LINE__);
		return 0;
	}

	va_start(args, fmt);
	size = vsnprintf(p, *left_size, fmt, args);
	va_end(args);
	*pp += size;
	*left_size -= size;

	return size;
}

static int ufs_debug_print_buf(struct ufs_hba *hba, char *obuf,
		int out_size, char *ibuf, int in_size)
{
	int i;
	char *pb = obuf;
	int left_size = out_size;
	int s;

	if (in_size > 512) {
		dev_err(hba->dev, "[%s:%d] Error: in_size=%d:\n",
				__func__, __LINE__, in_size);

		in_size = 512;
	}

	s = ufs_debug_snprint(&pb, &left_size, "%8s : ", "Address");
	for (i = 0; i < 16; i++)
		s += ufs_debug_snprint(&pb, &left_size, "%02x ", i);
	s += ufs_debug_snprint(&pb, &left_size, "\n");
	s += ufs_debug_snprint(&pb, &left_size,
			"==========================================================\n");

	for (i = 0; i < in_size; i++) {
		if ((i % 16) == 0)
			s += ufs_debug_snprint(&pb, &left_size, "%08x : ", i);

		s += ufs_debug_snprint(&pb, &left_size, "%02x ", ibuf[i]);

		if ((i % 16) == 15)
			s += ufs_debug_snprint(&pb, &left_size, "\n");
	}
	s += ufs_debug_snprint(&pb, &left_size, "\n");

	return s;
}

static ssize_t
ufshcd_dev_desc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	int err = 0;
	u8 *desc_buf;

	if (hba->desc_size.dev_desc) {
		desc_buf = kzalloc(hba->desc_size.dev_desc, GFP_KERNEL);
		if (!desc_buf) {
			err = -ENOMEM;
			dev_err(hba->dev,
				"%s: Failed to allocate desc_buf\n", __func__);
			return err;
		}
	} else
		return snprintf(buf, 128, "desc_size should be init first.\n");

	pm_runtime_get_sync(hba->dev);
	err = ufshcd_read_device_desc(hba, desc_buf,
			hba->desc_size.dev_desc);
	pm_runtime_put_sync(hba->dev);
	if (err) {
		dev_err(hba->dev, "[%s:%d] Failed to read device descriptor: %d\n",
				__func__, __LINE__, err);
		snprintf(buf, 128, "Failed to read device descriptor(%d).\n",
				err);
		goto out;
	}

	err = (ssize_t)ufs_debug_print_buf(hba, buf, 512,
			(char *)desc_buf, hba->desc_size.dev_desc);
out:
	kfree(desc_buf);
	return err;
}
static DEVICE_ATTR(device_desc, S_IRUGO, ufshcd_dev_desc_show, NULL);

static ssize_t
ufshcd_manf_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	int err = 0;
	u8 *desc_buf;
	struct ufs_qcom_host *qcom_host = hba->priv;

	if (qcom_host->ufs_vinfo.manf_name[0]) {
		return snprintf(buf, qcom_host->ufs_vinfo.manf_name[QUERY_DESC_LENGTH_OFFSET],
				"%.8s\n", qcom_host->ufs_vinfo.manf_name+QUERY_DESC_HDR_SIZE);
	}

	if (hba->desc_size.dev_desc) {
		desc_buf = kzalloc(hba->desc_size.dev_desc, GFP_KERNEL);
		if (!desc_buf) {
			err = -ENOMEM;
			dev_err(hba->dev,
				"%s: Failed to allocate desc_buf\n", __func__);
			return err;
		}
	} else
		return snprintf(buf, 128, "desc_size should be init first.\n");

	pm_runtime_get_sync(hba->dev);
	err = ufshcd_read_device_desc(hba, desc_buf,
			hba->desc_size.dev_desc);
	if (err) {
		pm_runtime_put_sync(hba->dev);
		dev_err(hba->dev, "[%s:%d] Failed to read device descriptor: %d\n",
				__func__, __LINE__, err);
		snprintf(buf, 128, "Failed to read device descriptor(%d).\n",
				err);
		goto out;
	}

	err = ufshcd_read_string_desc_ven(hba,
		desc_buf[DEVICE_DESC_PARAM_MANF_NAME],
		qcom_host->ufs_vinfo.manf_name, sizeof(qcom_host->ufs_vinfo.manf_name), ASCII_STD);

	pm_runtime_put_sync(hba->dev);
	err = snprintf(buf, qcom_host->ufs_vinfo.manf_name[QUERY_DESC_LENGTH_OFFSET],
			"%.8s\n", qcom_host->ufs_vinfo.manf_name+QUERY_DESC_HDR_SIZE);

out:
	kfree(desc_buf);
	return err;
}

static DEVICE_ATTR(manufacturer, S_IRUGO, ufshcd_manf_name_show, NULL);

static ssize_t
ufshcd_product_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	int err = 0;
	u8 *desc_buf;
	struct ufs_qcom_host *qcom_host = hba->priv;

	if (qcom_host->ufs_vinfo.product_name[0]) {
		return snprintf(buf,
				qcom_host->ufs_vinfo.product_name[QUERY_DESC_LENGTH_OFFSET],
				"%.16s\n", qcom_host->ufs_vinfo.product_name+QUERY_DESC_HDR_SIZE);
	}

	if (hba->desc_size.dev_desc) {
		desc_buf = kzalloc(hba->desc_size.dev_desc, GFP_KERNEL);
		if (!desc_buf) {
			err = -ENOMEM;
			dev_err(hba->dev,
				"%s: Failed to allocate desc_buf\n", __func__);
			return err;
		}
	} else
		return snprintf(buf, 128, "desc_size should be init first.\n");

	pm_runtime_get_sync(hba->dev);
	err = ufshcd_read_device_desc(hba, desc_buf,
			hba->desc_size.dev_desc);
	if (err) {
		pm_runtime_put_sync(hba->dev);
		dev_err(hba->dev, "[%s:%d] Failed to read device descriptor: %d\n",
				__func__, __LINE__, err);
		snprintf(buf, 128, "Failed to read device descriptor(%d).\n",
				err);
		goto out;
	}

	err = ufshcd_read_string_desc_ven(hba,
		desc_buf[DEVICE_DESC_PARAM_PRDCT_NAME],
		qcom_host->ufs_vinfo.product_name, sizeof(qcom_host->ufs_vinfo.product_name),
		ASCII_STD);

	pm_runtime_put_sync(hba->dev);
	err = snprintf(buf, qcom_host->ufs_vinfo.product_name[QUERY_DESC_LENGTH_OFFSET],
			"%.16s\n", qcom_host->ufs_vinfo.product_name+QUERY_DESC_HDR_SIZE);

out:
	kfree(desc_buf);
	return err;
}
static DEVICE_ATTR(product_name, S_IRUGO, ufshcd_product_name_show, NULL);

static ssize_t ufshcd_life_time_info_show(struct ufs_hba *hba,
		u8 offset, char *buf)
{
	int err = 0;
	u8 desc_buf[QUERY_DESC_HEALTH_DEF_SIZE];
	u8 life_time_buf[128];
	struct ufs_qcom_host *qcom_host = hba->priv;

	/* if (qcom_host->ufs_vinfo.pre_end_life[0]) { */
		/* switch (offset) { */
		/* case 2: */
			/* err = snprintf(buf, 128, qcom_host->ufs_vinfo.pre_end_life); */
		/* break; */
		/* case 3: */
			/* err = snprintf(buf, 128, qcom_host->ufs_vinfo.life_time_a); */
		/* break; */
		/* case 4: */
			/* err = snprintf(buf, 128, qcom_host->ufs_vinfo.life_time_b); */
		/* break; */
		/* default: */
			/* err = snprintf(buf, 128, */
					/* "Reading LifeTimeInfo failed.\n"); */
		/* } */
		/* return err; */
	/* } */

	memset(desc_buf, 0, QUERY_DESC_HEALTH_DEF_SIZE);
	memset(life_time_buf, 0, sizeof(life_time_buf));

	pm_runtime_get_sync(hba->dev);
	err = ufshcd_read_health_desc(hba, desc_buf,
			QUERY_DESC_HEALTH_DEF_SIZE);
	pm_runtime_put_sync(hba->dev);

	if (!err) {
		snprintf(qcom_host->ufs_vinfo.pre_end_life, sizeof(qcom_host->ufs_vinfo.pre_end_life),
				"0x%02x\n", desc_buf[2]);
		snprintf(qcom_host->ufs_vinfo.life_time_a, sizeof(qcom_host->ufs_vinfo.life_time_a),
				"0x%02x\n", desc_buf[3]);
		snprintf(qcom_host->ufs_vinfo.life_time_b, sizeof(qcom_host->ufs_vinfo.life_time_b),
				"0x%02x\n", desc_buf[4]);

		snprintf(life_time_buf, sizeof(life_time_buf),
				"0x%02x\n", desc_buf[offset]);
		err = snprintf(buf, 128, life_time_buf);
		return err;
	}
	snprintf(buf, 128, "Reading LifeTimeInfo failed.\n");
	return err;
}

static ssize_t
ufshcd_life_time_a_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return ufshcd_life_time_info_show(hba, 3, buf);
}
static DEVICE_ATTR(life_time_a, S_IRUGO, ufshcd_life_time_a_show, NULL);

static ssize_t
ufshcd_life_time_b_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return ufshcd_life_time_info_show(hba, 4, buf);
}
static DEVICE_ATTR(life_time_b, S_IRUGO, ufshcd_life_time_b_show, NULL);

static ssize_t
ufshcd_pre_end_life_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return ufshcd_life_time_info_show(hba, 2, buf);
}
static DEVICE_ATTR(pre_end_life, S_IRUGO, ufshcd_pre_end_life_show, NULL);

static ssize_t
ufshcd_ufsid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufs_qcom_host *qcom_host = hba->priv;
	if (qcom_host->ufs_vinfo.ufsid[0])
		return snprintf(buf, 512, qcom_host->ufs_vinfo.ufsid);

	return snprintf(buf, 512, "ufdid is NULL\n");
}

static DEVICE_ATTR(ufsid, S_IRUGO, ufshcd_ufsid_show, NULL);

static const char *ufschd_uic_link_state_to_string(
			enum uic_link_state state)
{
	switch (state) {
	case UIC_LINK_OFF_STATE:	return "OFF";
	case UIC_LINK_ACTIVE_STATE:	return "ACTIVE";
	case UIC_LINK_HIBERN8_STATE:	return "HIBERN8";
	default:			return "UNKNOWN";
	}
}

static ssize_t
ufshcd_uic_link_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return snprintf(buf, 512, "%s\n",
			ufschd_uic_link_state_to_string(hba->uic_link_state));
}

static DEVICE_ATTR(uic_link_state, S_IRUGO, ufshcd_uic_link_state_show, NULL);

static int ufsfeature_read_desc(struct ufs_hba *hba, u8 desc_id, u8 desc_index,
			  u8 selector, u8 *desc_buf, u32 size)
{
	int err = 0;

	pm_runtime_get_sync(hba->dev);

	err = ufshcd_query_descriptor_retry(hba, UPIU_QUERY_OPCODE_READ_DESC,
					    desc_id, desc_index,
					    selector,
					    desc_buf, &size);
	if (err)
		pr_err("[%s:%d] reading Device Desc failed. err=%d\n", __func__, __LINE__, err);

	pm_runtime_put_sync(hba->dev);

	return err;
}

#if !defined(CONFIG_UFSFEATURE)
#define UFSF_QUERY_DESC_UNIT_MAX_SIZE		0x2D
#endif
#if !defined(CONFIG_UFSTW)
#define UNIT_DESC_TW_LU_MAX_BUF_SIZE		0x29
#endif
int ufs_feature_read_unit_desc(struct ufs_hba *hba)
{
	u8 unit_buf[UFSF_QUERY_DESC_UNIT_MAX_SIZE];
	int ret = 0;

	ret = ufsfeature_read_desc(hba, QUERY_DESC_IDN_UNIT, 0,
			0x01, /* 0x01 for ufs feature of tw and hpb */
			unit_buf, UFSF_QUERY_DESC_UNIT_MAX_SIZE);
	if (ret) {
		pr_err("[%s:%d] read unit desc failed. ret %d\n", __func__, __LINE__, ret);
		return ret;
	}

	ufs_ven_lu0_lu_enable = unit_buf[UNIT_DESC_PARAM_LU_ENABLE];
	ufs_ven_lu0_tw_lu_buf_size = be32_to_cpu(*(__be32 *)(&unit_buf[UNIT_DESC_TW_LU_MAX_BUF_SIZE]));
	pr_err("[%s:%d] ufs_ven_lu0_lu_enable=%d, ufs_ven_lu0_tw_lu_buf_size=%d\n",
			__func__, __LINE__, ufs_ven_lu0_lu_enable, ufs_ven_lu0_tw_lu_buf_size);

	return ret;
}

static ssize_t
ufshcd_tw_lu_buf_size_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 16, "%u\n", ufs_ven_lu0_tw_lu_buf_size);
}

static DEVICE_ATTR(tw_lu_buf_size, S_IRUGO, ufshcd_tw_lu_buf_size_show, NULL);

static ssize_t
ufshcd_wb_lu_buf_size_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	int ret;
	int ufs_ven_wb_lu_buf_size;
	u8 *desc_buf;

	desc_buf = kmalloc(hba->desc_size.dev_desc, GFP_KERNEL);
	if (!desc_buf)
		return -ENOMEM;

	ret = ufsfeature_read_desc(hba, QUERY_DESC_IDN_DEVICE, 0,
						0, desc_buf, hba->desc_size.dev_desc);

	if (ret) {
		pr_notice("[%s:%d] read device desc failed. ret %d\n", __func__, __LINE__, ret);
		ufs_ven_wb_lu_buf_size = 0;
	} else
		ufs_ven_wb_lu_buf_size =
		be32_to_cpu(*(__be32 *)(&desc_buf[DEVICE_DESC_PARAM_WB_SHARED_ALLOC_UNITS]));

	kfree(desc_buf);

	return snprintf(buf, 16, "%u\n", ufs_ven_wb_lu_buf_size);
}

static DEVICE_ATTR(wb_lu_buf_size, S_IRUGO, ufshcd_wb_lu_buf_size_show, NULL);

static ssize_t
ufshcd_lu_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 16, "%u\n", ufs_ven_lu0_lu_enable);
}

static DEVICE_ATTR(lu_enable, S_IRUGO, ufshcd_lu_enable_show, NULL);

static ssize_t
ufshcd_main_lu_size_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct scsi_device *sdp;
	unsigned long flags;
	struct scsi_disk *sdkp;
	int i;
	int ret;
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufs_qcom_host *qcom_host = hba->priv;
	u32 main_lu_size = 0;

	spin_lock_irqsave(hba->host->host_lock, flags);

	for (i = 0; i < ARRAY_SIZE(qcom_host->sdev_ufs_lu_ven); i++) {
		sdp = qcom_host->sdev_ufs_lu_ven[i];
		if (sdp) {
			ret = scsi_device_get(sdp);
			if (!ret && !scsi_device_online(sdp)) {
				ret = -ENODEV;
				scsi_device_put(sdp);
				break;
			}

			sdkp = dev_get_drvdata(&sdp->sdev_gendev);
			if (sdkp->capacity <= UFS_CAPACITY_32GB)
				scsi_device_put(sdp); /*  Just mtk platform put lu0/lu1	*/
			else
				break; /*  Find it */
		} else {
			ret = -ENODEV;
			break;
		}
	}
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	if (ret)
		pr_notice("[%s:%d] read main lu size failed. ret %d\n", __func__, __LINE__, ret);
	else
		main_lu_size = sdkp->capacity >> 18;

	scsi_device_put(sdp);

	return snprintf(buf, 16, "%d\n", main_lu_size);
}

static DEVICE_ATTR(main_lu_size, S_IRUGO, ufshcd_main_lu_size_show, NULL);

static ssize_t
ufshcd_show_hba_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct seq_file m;

	memset(&m, 0, sizeof(struct seq_file));
	m.buf = buf;
	m.size = PAGE_SIZE;

	seq_printf(&m, "hba->outstanding_tasks = 0x%x\n",
		(u32)hba->outstanding_tasks);
	seq_printf(&m, "hba->outstanding_reqs = 0x%x\n",
		(u32)hba->outstanding_reqs);

	seq_printf(&m, "hba->capabilities = 0x%x\n", hba->capabilities);
	seq_printf(&m, "hba->nutrs = %d\n", hba->nutrs);
	seq_printf(&m, "hba->nutmrs = %d\n", hba->nutmrs);
	seq_printf(&m, "hba->ufs_version = 0x%x\n", hba->ufs_version);
	seq_printf(&m, "hba->irq = 0x%x\n", hba->irq);
	seq_printf(&m, "hba->auto_bkops_enabled = %d\n",
		hba->auto_bkops_enabled);

	seq_printf(&m, "hba->ufshcd_state = 0x%x\n", hba->ufshcd_state);
	seq_printf(&m, "hba->clk_gating.state = 0x%x\n",
		hba->clk_gating.state);
	seq_printf(&m, "hba->eh_flags = 0x%x\n", hba->eh_flags);
	seq_printf(&m, "hba->intr_mask = 0x%x\n", hba->intr_mask);
	seq_printf(&m, "hba->ee_ctrl_mask = 0x%x\n", hba->ee_ctrl_mask);

	/* HBA Errors */
	seq_printf(&m, "hba->errors = 0x%x\n", hba->errors);
	seq_printf(&m, "hba->uic_error = 0x%x\n", hba->uic_error);
	seq_printf(&m, "hba->saved_err = 0x%x\n", hba->saved_err);
	seq_printf(&m, "hba->saved_uic_err = 0x%x\n", hba->saved_uic_err);

	seq_printf(&m, "power_mode_change_cnt = %d\n",
		hba->ufs_stats.power_mode_change_cnt);
	seq_printf(&m, "hibern8_exit_cnt = %d\n",
		hba->ufs_stats.hibern8_exit_cnt);

	seq_printf(&m, "pa_err_cnt_total = %d\n",
		hba->ufs_stats.pa_err_cnt_total);
	seq_printf(&m, "pa_lane_0_err_cnt = %d\n",
		hba->ufs_stats.pa_err_cnt[UFS_EC_PA_LANE_0]);
	seq_printf(&m, "pa_lane_1_err_cnt = %d\n",
		hba->ufs_stats.pa_err_cnt[UFS_EC_PA_LANE_1]);
	seq_printf(&m, "pa_line_reset_err_cnt = %d\n",
		hba->ufs_stats.pa_err_cnt[UFS_EC_PA_LINE_RESET]);
	seq_printf(&m, "dl_err_cnt_total = %d\n",
		hba->ufs_stats.dl_err_cnt_total);
	seq_printf(&m, "dl_nac_received_err_cnt = %d\n",
		hba->ufs_stats.dl_err_cnt[UFS_EC_DL_NAC_RECEIVED]);
	seq_printf(&m, "dl_tcx_replay_timer_expired_err_cnt = %d\n",
		hba->ufs_stats.dl_err_cnt[UFS_EC_DL_TCx_REPLAY_TIMER_EXPIRED]);
	seq_printf(&m, "dl_afcx_request_timer_expired_err_cnt = %d\n",
		hba->ufs_stats.dl_err_cnt[UFS_EC_DL_AFCx_REQUEST_TIMER_EXPIRED]);
	seq_printf(&m, "dl_fcx_protection_timer_expired_err_cnt = %d\n",
		hba->ufs_stats.dl_err_cnt[UFS_EC_DL_FCx_PROTECT_TIMER_EXPIRED]);
	seq_printf(&m, "dl_crc_err_cnt = %d\n",
		hba->ufs_stats.dl_err_cnt[UFS_EC_DL_CRC_ERROR]);
	seq_printf(&m, "dll_rx_buffer_overflow_err_cnt = %d\n",
		hba->ufs_stats.dl_err_cnt[UFS_EC_DL_RX_BUFFER_OVERFLOW]);
	seq_printf(&m, "dl_max_frame_length_exceeded_err_cnt = %d\n",
		hba->ufs_stats.dl_err_cnt[UFS_EC_DL_MAX_FRAME_LENGTH_EXCEEDED]);
	seq_printf(&m, "dl_wrong_sequence_number_err_cnt = %d\n",
		hba->ufs_stats.dl_err_cnt[UFS_EC_DL_WRONG_SEQUENCE_NUMBER]);
	seq_printf(&m, "dl_afc_frame_syntax_err_cnt = %d\n",
		hba->ufs_stats.dl_err_cnt[UFS_EC_DL_AFC_FRAME_SYNTAX_ERROR]);
	seq_printf(&m, "dl_nac_frame_syntax_err_cnt = %d\n",
		hba->ufs_stats.dl_err_cnt[UFS_EC_DL_NAC_FRAME_SYNTAX_ERROR]);
	seq_printf(&m, "dl_eof_syntax_err_cnt = %d\n",
		hba->ufs_stats.dl_err_cnt[UFS_EC_DL_EOF_SYNTAX_ERROR]);
	seq_printf(&m, "dl_frame_syntax_err_cnt = %d\n",
		hba->ufs_stats.dl_err_cnt[UFS_EC_DL_FRAME_SYNTAX_ERROR]);
	seq_printf(&m, "dl_bad_ctrl_symbol_type_err_cnt = %d\n",
		hba->ufs_stats.dl_err_cnt[UFS_EC_DL_BAD_CTRL_SYMBOL_TYPE]);
	seq_printf(&m, "dl_pa_init_err_cnt = %d\n",
		hba->ufs_stats.dl_err_cnt[UFS_EC_DL_PA_INIT_ERROR]);
	seq_printf(&m, "dl_pa_error_ind_received = %d\n",
		hba->ufs_stats.dl_err_cnt[UFS_EC_DL_PA_ERROR_IND_RECEIVED]);
	seq_printf(&m, "dme_err_cnt = %d\n", hba->ufs_stats.dme_err_cnt);

	return m.count;
}

static DEVICE_ATTR(show_hba, S_IRUGO, ufshcd_show_hba_show, NULL);

static struct attribute *def_ufshcd_feature_attrs[] = {
	&dev_attr_ufsid.attr,
	&dev_attr_manufacturer.attr,
	&dev_attr_product_name.attr,
	&dev_attr_device_desc.attr,
	&dev_attr_pre_end_life.attr,
	&dev_attr_life_time_a.attr,
	&dev_attr_life_time_b.attr,
	&dev_attr_uic_link_state.attr,
	&dev_attr_tw_lu_buf_size.attr,
	&dev_attr_wb_lu_buf_size.attr,
	&dev_attr_main_lu_size.attr,
	&dev_attr_lu_enable.attr,
	&dev_attr_show_hba.attr,
#if 1
	&dev_attr_io_stats.attr,
	&dev_attr_io_sum.attr,
	&dev_attr_log_enable.attr,
	&dev_attr_load.attr,
	&dev_attr_load_thre.attr,
	&dev_attr_nand_info.attr,
	&dev_attr_nand_info_dec.attr,
	&dev_attr_nand_info_cloud.attr,
	&dev_attr_io_stat_cmd.attr,
	&dev_attr_io_stat_kmsg.attr,
	&dev_attr_stat_cmd_enable.attr,
#endif
	NULL,
};

static const struct attribute_group ufshcd_ven_attr_group = {
   .attrs = def_ufshcd_feature_attrs,
};

static void ufshcd_feature_release(struct kobject *kobj)
{
	/* nothing to do temply */
}

static ssize_t ufshcd_feature_show(struct kobject *k,
		struct attribute *attr, char *buf)
{
	struct ufs_qcom_host *qcom_host = container_of(k, struct ufs_qcom_host,
			ufs_vinfo.ufs_feature_kobject);

	struct ufs_hba *hba = qcom_host->hba;

	struct device_attribute *dev_attr = to_dev_attr(attr);

	return dev_attr->show ? dev_attr->show(hba->dev, dev_attr, buf) : -EIO;
}

static ssize_t ufshcd_feature_store(struct kobject *k, struct attribute *attr,
		const char *buf, size_t count)
{
	struct ufs_qcom_host *qcom_host = container_of(k, struct ufs_qcom_host,
			ufs_vinfo.ufs_feature_kobject);
	struct ufs_hba *hba = qcom_host->hba;

	struct device_attribute *dev_attr = to_dev_attr(attr);

	return dev_attr->store ? dev_attr->store(hba->dev, dev_attr, buf, count) : -EIO;
}

static const struct sysfs_ops ufshcd_feature_sysfs_ops = {
	.show = ufshcd_feature_show,
	.store = ufshcd_feature_store,
};

static struct kobj_type ufshcd_feature_object_type = {
	.sysfs_ops  = &ufshcd_feature_sysfs_ops,
	.release	 = ufshcd_feature_release,
	.default_attrs = def_ufshcd_feature_attrs,
};

int ufs_get_ufsid (struct ufs_hba *hba)
{
	int err = 0;
	u8 *desc_buf = NULL;
	u8 *serial_buf = NULL;
	u8 serial_index = 0;
	u8 serial_len = 0;
	u8  *ptemp = NULL;
	u16  k = 0;
	struct ufs_qcom_host *qcom_host = hba->priv;

	if (hba->desc_size.dev_desc) {
		desc_buf = kmalloc(hba->desc_size.dev_desc, GFP_KERNEL);
		if (!desc_buf) {
			err = -ENOMEM;
			dev_err(hba->dev,
				"%s: Failed to allocate desc_buf\n", __func__);
			return err;
		}
		serial_buf = kzalloc(QUERY_DESC_MAX_SIZE, GFP_KERNEL);
		if (!serial_buf) {
			err = -ENOMEM;
			dev_err(hba->dev,
				"%s: Failed to allocate serial_buf\n", __func__);
			goto out;
		}
	}
	err = ufshcd_read_device_desc(hba, desc_buf, hba->desc_size.dev_desc);
	if (err)
		goto __free_serial;

	/*
	 * getting vendor (manufacturerID) and Bank Index in big endian
	 * format
	 */
	qcom_host->ufs_vinfo.w_manufacturer_id =
		desc_buf[DEVICE_DESC_PARAM_MANF_ID] << 8 |
		desc_buf[DEVICE_DESC_PARAM_MANF_ID + 1];
	serial_index = desc_buf[DEVICE_DESC_PARAM_SN];
	qcom_host->ufs_vinfo.wspecversion = desc_buf[DEVICE_DESC_PARAM_SPEC_VER] << 8 |
		desc_buf[DEVICE_DESC_PARAM_SPEC_VER + 1];
	err = ufshcd_read_string_desc_ven(hba, serial_index, serial_buf,
			QUERY_DESC_MAX_SIZE, UTF16_STD);

	if (err)
		goto __free_serial;

	serial_len = serial_buf[0] - 2;
	snprintf(qcom_host->ufs_vinfo.ufsid, 5, "%04x", qcom_host->ufs_vinfo.w_manufacturer_id);
	ptemp = qcom_host->ufs_vinfo.ufsid + 4;
	for (k = 0; k <= serial_len-1; k++) {
		snprintf(ptemp, 3, "%02x", serial_buf[2+k]);
		ptemp = ptemp + 2;
	}
	snprintf(ptemp, 5, "\n");

	ufsid_pointer = qcom_host->ufs_vinfo.ufsid;

__free_serial:
	kfree(serial_buf);
out:
	kfree(desc_buf);
	return err;
}

void ufshcd_io_vinit (struct ufs_hba *hba)
{
	int err;
	struct ufs_qcom_host *qcom_host = hba->priv;

	memset(qcom_host->ufs_vinfo.ufsid, 0, sizeof(qcom_host->ufs_vinfo.ufsid));
	memset(qcom_host->ufs_vinfo.pre_end_life, 0, sizeof(qcom_host->ufs_vinfo.pre_end_life));
	memset(qcom_host->ufs_vinfo.life_time_a, 0, sizeof(qcom_host->ufs_vinfo.life_time_a));
	memset(qcom_host->ufs_vinfo.life_time_b, 0, sizeof(qcom_host->ufs_vinfo.life_time_b));

	err = kobject_init_and_add(&qcom_host->ufs_vinfo.ufs_feature_kobject,
			&ufshcd_feature_object_type, NULL, "ufs");
	if (err)
		dev_err(hba->dev, "[%s:%d]: Create ufs_feature_kobject for ufs sys nodes failed %d\n",
				__func__, __LINE__, err);

	ufshcd_io_stats_init(hba);
}


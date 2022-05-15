#include <linux/mount.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/buffer_head.h>
#include <linux/blk_types.h>
#include <linux/genhd.h>

#define SD_PARTITION_RAW_RW_API

#define UFS_BLOCK_SIZE (4096)

static int partition_block_rw(dev_t devt, int write, sector_t index,
		sector_t index_offset, void *buffer, size_t len)
{
	struct block_device *bdev;
	struct buffer_head *bh = NULL;
	fmode_t mode = FMODE_READ;
	int err = -EIO;

	if (len > UFS_BLOCK_SIZE)
		return -EINVAL;

	mode = write ? FMODE_WRITE : FMODE_READ;
	bdev = blkdev_get_by_dev(devt, mode, NULL);
	if (IS_ERR(bdev))
		return PTR_ERR(bdev);
	
	pr_debug("[%s][%d] index %lx offset %lx\n", __func__, __LINE__,
			(unsigned long)index, (unsigned long)index_offset);
	set_blocksize(bdev, UFS_BLOCK_SIZE);
	bh = __getblk(bdev, index, UFS_BLOCK_SIZE);
	if (bh) {
		clear_buffer_uptodate(bh);
		get_bh(bh);
		lock_buffer(bh);
		bh->b_end_io = end_buffer_read_sync;
		pr_debug("[%s][%d] bh->b_blocknr %lx bh->b_size %lx\n",
				__func__, __LINE__, (unsigned long)bh->b_blocknr,
				(unsigned long)bh->b_size);
		submit_bh(REQ_OP_READ, 0, bh);
		wait_on_buffer(bh);
		if (unlikely(!buffer_uptodate(bh))) {
			pr_err("%s: read error!!\n", __func__);
			goto out;
		}
		if (write) {
			lock_buffer(bh);
			memcpy(bh->b_data+index_offset, buffer, len);
			bh->b_end_io = end_buffer_write_sync;
			get_bh(bh);
			submit_bh(REQ_OP_WRITE, 0, bh);
			wait_on_buffer(bh);
			if (unlikely(!buffer_uptodate(bh))) {
				pr_err("%s: write error!!\n", __func__);
				goto out;
			}
		} else {
			memcpy(buffer, bh->b_data+index_offset, len);
		}
		err = 0;
	} else {
		pr_info("%s error\n", __func__);
	}

out:
	brelse(bh);
	blkdev_put(bdev, mode);

	return err;
}

static int match_dev_by_label(struct device *dev, const void *data)
{
	const char *label = data;
	struct hd_struct *part = dev_to_part(dev);

	if (part->info && !strcmp(label, part->info->volname))
		return 1;

	return 0;
}

static int get_partition_info(const char *part_name, dev_t *devt,
		sector_t *start, sector_t *nr_sect)
{
	struct block_device *bdev = NULL;
	struct hd_struct *part = NULL;
	struct device *dev = NULL;
	struct class *block_class = NULL;
	fmode_t mode = FMODE_READ;
	dev_t first_devt = MKDEV(SCSI_DISK0_MAJOR, 0);

	bdev = blkdev_get_by_dev(first_devt, mode, NULL);
	if (IS_ERR(bdev)) {
		pr_info("%s bdev is error\n", __func__);
		return PTR_ERR(bdev);
	}

	if (!bdev->bd_disk) {
		pr_info("%s bd_disk is null\n", __func__);
		return -ENODEV;
	}

	block_class = disk_to_dev(bdev->bd_disk)->class;
	if (!block_class) {
		pr_info("%s block_class is null\n", __func__);
		return -ENODEV;
	}

	dev = class_find_device(block_class, NULL, part_name, &match_dev_by_label);
	if (!dev) {
		pr_info("%s dev is null\n", __func__);
		return -ENODEV;
	}

	*devt = dev->devt;

	part = dev_to_part(dev);
	*start = 0;
	*nr_sect = part->nr_sects;

	pr_debug("__dev major %d minor %d volname %s\n",
				MAJOR(*devt), MINOR(*devt), part->info ? (char *)part->info->volname : "null");
	pr_debug("__dev start %d nr_sect %d\n", (int)*start, (int)*nr_sect);

	blkdev_put(bdev, mode);

	return 0;
}

int partition_rw(const char *part_name, int write, loff_t offset,
		void *buffer, size_t len)
{
	int ret = 0;
	void *p = buffer;
	size_t size;
	dev_t devt;
	sector_t first_sect;
	sector_t index;
	sector_t index_offset;
	sector_t start, nr_sect;

	if (buffer == NULL)
		return -EINVAL;

	ret = get_partition_info(part_name, &devt, &start, &nr_sect);
	if (ret) {
		pr_err("%s: can't find partition(%s), err = %d\n", __func__, part_name, ret);
		return -ENODEV;
	}

	start = start / (UFS_BLOCK_SIZE / 512);
	nr_sect = nr_sect / (UFS_BLOCK_SIZE / 512);
	if (offset < 0 || (offset + len) > nr_sect * UFS_BLOCK_SIZE) {
		pr_err("%s: access area exceed parition(%s) range.\n", __func__,
				part_name);
		return -EINVAL;
	}

	index = start + offset / UFS_BLOCK_SIZE;
	index_offset = offset % UFS_BLOCK_SIZE;
	first_sect = index;

	while (len > 0) {
		size = len;

		if (size > UFS_BLOCK_SIZE)
			size = UFS_BLOCK_SIZE;

		if (index_offset > 0 && index == first_sect) {
			if (size > UFS_BLOCK_SIZE - index_offset)
				size = UFS_BLOCK_SIZE - index_offset;
		}

		if (index == first_sect)
			ret = partition_block_rw(devt, write, 
					index, index_offset, p, size);
		else
			ret = partition_block_rw(devt, write,
					index, 0, p, size);
		if (ret) {
			pr_err("[%s][%lu] error %d\n", __func__, (unsigned long)len, ret);
			break;
		}

		len -= size;
		index++;
		p += size;
	}

	return ret;
}

EXPORT_SYMBOL(partition_rw);

#ifdef PARTITION_TEST
static void dump_data(unsigned char *addr, unsigned long offset, unsigned long size)
{
	int i;

	while (size > 0) {
		pr_info("%016lx |", offset);
		for (i = 0; i < 16; i++)
			pr_cont(" %02x", *addr++);
		pr_cont("\n");
		offset += 16;
		size = (size > 16) ? (size - 16) : 0;
	}
}

static int dump_partition_set(const char *val, const struct kernel_param *kp)
{
	unsigned long addr = 0;
	unsigned char buf[256];
	int ret;

	if (kstrtoul(val, 16, &addr))
		return -EINVAL;

	ret = partition_rw("logdump", 0, addr, buf, 256);
	if (ret)
		pr_err("[%s]error %d\n", __func__, ret);

	dump_data(buf, addr, 256);

	return 0;
}

static const struct kernel_param_ops dump_partition_opts = {
	.set = dump_partition_set,
};

module_param_cb(partition_test, &dump_partition_opts, NULL, 0600);
#endif

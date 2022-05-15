#ifndef __PARTITION_RW_H__

int partition_rw(const char *part_name, int write, loff_t offset,
		void *buffer, size_t len);

#endif //__PARTITION_RW_H__

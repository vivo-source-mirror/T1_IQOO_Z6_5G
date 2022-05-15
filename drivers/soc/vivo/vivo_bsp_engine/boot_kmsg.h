#ifndef __BOOT_KMSG_H__
#define __BOOT_KMSG_H__

#define VIVO_BOOT_KMSG_BACKUP_ALLOC
#define VIVO_KMSG_ADD_ANDROID_TIME

int boot_kmsg_init(void);
void boot_kmsg_exit(void);

#endif //__BOOT_KMSG__

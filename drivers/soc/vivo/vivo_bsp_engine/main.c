#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include "cmdline_parser.h"
#include "blog.h"
#include "boot_kmsg.h"
#include "hung_task.h"


struct vbsp_entry {
	int (*init)(void);
	void (*exit)(void);
};

static struct vbsp_entry vbsp_entry_tbl[] = {
	{cmdline_parser_init, NULL},
	{proc_steplog_init, NULL},
	{boot_kmsg_init, boot_kmsg_exit},
	{hung_task_init, NULL},
};



static int __init vivo_bsp_engine_init(void)
{
	int i, ret;

	printk(KERN_INFO "vivo_bsp_engine init.\n");

	for (i = 0; i < ARRAY_SIZE(vbsp_entry_tbl); i++) {
		if (!vbsp_entry_tbl[i].init)
			continue;
		printk(KERN_INFO "vivo_bsp_engine %ps init.\n",
				vbsp_entry_tbl[i].init);
		ret = vbsp_entry_tbl[i].init();
		if (ret)
			printk(KERN_ERR "vivo_bsp_engine %ps init fail, ret %d.\n",
					vbsp_entry_tbl[i].init, ret);
	}

	return 0;
}
module_init(vivo_bsp_engine_init);

#ifdef MODULE_ALLOW_EXIT
static void __exit vivo_bsp_engine_exit(void)
{
	int i;

	for (i = ARRAY_SIZE(vbsp_entry_tbl) - 1; i >= 0; i--) {
		if (!vbsp_entry_tbl[i].exit)
			continue;
		printk(KERN_INFO "vivo_bsp_engine %ps exit.\n",
				vbsp_entry_tbl[i].exit);
		vbsp_entry_tbl[i].exit();
	}

	printk(KERN_INFO "vivo_bsp_engine exit.\n");
}
module_exit(vivo_bsp_engine_exit);
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("LinuxBSP");
MODULE_DESCRIPTION("vivo bsp common module.");
MODULE_VERSION("0.01");

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/serial_core.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#include "tfa9894-debug-common.h"
#include "../vivo-codec-common.h"

#define AM_DEV_NAME   "smartpa"

#define MSGS_SIZE 256
struct tfa98xx_msg {
	char msgs[256];
	char reserved[252];
	int msg_result;
};

struct tfa98xx_prars {
	int fRes_max;
	int fRes_min;
	int Qt;
	int impedance_max;
	int impedance_min;
};
#define TFA_CTL_IOC_MAGIC  'T'
#define TFA_IOCTL_SPK_REST  _IOW(TFA_CTL_IOC_MAGIC, 0x01, int)
#define TFA_IOCTL_SPK_INTS   _IOR(TFA_CTL_IOC_MAGIC, 0x02, struct tfa98xx_msg)
#define TFA_IOCTL_SPK_INTT  _IOW(TFA_CTL_IOC_MAGIC, 0x03, int)
#define TFA_IOCTL_SPK_RFDES 	_IOR(TFA_CTL_IOC_MAGIC, 0x04, struct tfa98xx_msg)
#define TFA_IOCTL_SPK_CHCK _IOR(TFA_CTL_IOC_MAGIC, 0x05, int)
#define TFA_IOCTL_SPK_PRARS _IOR(TFA_CTL_IOC_MAGIC, 0x06, struct tfa98xx_prars)
#define TFA_IOCTL_SPK_ADDR  _IOW(TFA_CTL_IOC_MAGIC, 0x07, unsigned char)
#define TFA_IOCTL_SPK_MTP_BACKUP _IOR(TFA_CTL_IOC_MAGIC, 0x08, int)
#define TFA_IOCTL_SPK_SET   _IOW(TFA_CTL_IOC_MAGIC, 0x09, struct tfa98xx_msg)

extern int tfa98xx_check_calib_dbg(void);
extern int tfa98xx_init_dbg(char *buffer, char *buffer1, int size);
extern int tfa98xx_read_freq_dbg(char *buffer, int size);
extern void tfa98xx_init_re_value(uint32_t *re_value);
extern void tfa9894_read_prars_dbg(int temp[5], unsigned char addr);
extern void tfa98xx_get_client(struct i2c_client **client, unsigned char addr);
extern void tfa98xx_read_prars_dbg(int temp[5], unsigned char addr);

static struct i2c_client *tfa98xx_debug_client;
static unsigned char last_addr;

static ssize_t tfa98xx_debug_read (struct file *file,
						char __user *buf, size_t count, loff_t *offset)
{
	char *tmp;
	int ret;

	tfa98xx_get_client(&tfa98xx_debug_client, last_addr);
	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL)
		return -ENOMEM;

	ret = i2c_master_recv(tfa98xx_debug_client, tmp, count);
	if (ret >= 0)
		ret = copy_to_user(buf, tmp, count) ? -EFAULT : ret;
	else 
		printk("[TFA98xx]%s: transfer error %d\n", __func__, ret);
	kfree(tmp);
	return ret;
}

static ssize_t tfa98xx_debug_write (struct file *file,
						const char __user *buf, size_t count, loff_t *offset)
{
	char *tmp;
	int ret;

	tfa98xx_get_client(&tfa98xx_debug_client, last_addr);
	tmp = memdup_user(buf, count);
	if (IS_ERR(tmp))
		return PTR_ERR(tmp);
	ret = i2c_master_send(tfa98xx_debug_client, tmp, count);
	if (ret < 0)
		printk("[TFA98xx]%s: transfer error %d\n", __func__, ret);
	kfree(tmp);
	return ret;
}
static long  tfa98xx_debug_shared_ioctl (struct file *file,
	unsigned int cmd, void __user *arg)
{
	int  ret = 0, check = 0;
	int temp[5];
	struct tfa98xx_msg msg;
	//struct tfa98xx_msg __user *_msg = (struct tfa98xx_msg __user *)arg;
	struct tfa98xx_prars prars;
	//struct tfa98xx_prars __user *_prars = (struct tfa98xx_prars __user *)arg;
	uint32_t re_calib[2] = {0};

	memset(&prars, 0, sizeof(struct tfa98xx_prars));
	memset(&msg, 0, sizeof(struct tfa98xx_msg));
	switch (cmd) {
	/* Reset MTP */
	case TFA_IOCTL_SPK_REST:
		printk("tfa98xx_ioctl SPK_REST\n");

		break;
	/* calibrate */
	case TFA_IOCTL_SPK_INTS:
		printk("tfa98xx_ioctl SPK_INTS\n");
		check = tfa98xx_init_dbg(msg.msgs,  msg.reserved, MSGS_SIZE);
		msg.msg_result = check;
		ret = copy_to_user((void *)arg, &msg, sizeof(struct tfa98xx_msg));
		break;
	case TFA_IOCTL_SPK_INTT:
		printk("tfa98xx_ioctl SPK_INTS\n");
		break;
	case TFA_IOCTL_SPK_SET:
		pr_debug("%s:smartpa_ioctl SPK_SET\n",__func__);
		ret = copy_from_user(&msg, (void __user *)arg,
			sizeof(struct tfa98xx_msg));
		if (ret){	
			pr_info("%s: Could not copy arg value from user\n", __func__);
			ret = -EFAULT;
		}else{
			ret = 0;
			memcpy(re_calib, msg.reserved, sizeof(re_calib));
			tfa98xx_init_re_value(re_calib);
			pr_info("%s,%s,calib: %x,%x,ret=%d\n",__func__,msg.msgs,
				re_calib[0],re_calib[1],ret);			
		}
		break;
	case TFA_IOCTL_SPK_RFDES:
		usleep_range(10*1000, 10*1000);
		printk("tfa98xx_ioctl SPK_ReadFDes\n");
		ret = tfa98xx_read_freq_dbg(msg.msgs, MSGS_SIZE);
		ret = copy_to_user((void *)arg, &msg, sizeof(struct tfa98xx_msg));
		break;
	/* checkmtp */
	case TFA_IOCTL_SPK_CHCK:
		printk("tfa98xx_ioctl SPK Check MtpEx\n");
		check = tfa98xx_check_calib_dbg();
		pr_info("%s check %d.\n", __func__, check);
		ret = copy_to_user((void *)arg, &check, sizeof(int));
		break;
	case TFA_IOCTL_SPK_PRARS:
		printk("tfa98xx_ioctl SPK Read f0 and Qt\n");
		tfa98xx_read_prars_dbg(temp, last_addr);
		prars.fRes_max = temp[0];
		prars.fRes_min = temp[1];
		prars.Qt = temp[2];
		prars.impedance_max = temp[3];
		prars.impedance_min = temp[4];

		ret = copy_to_user((void *)arg, &prars, sizeof(struct tfa98xx_prars));
		pr_info("tfa98xx_ioctl %d %d %d\n", temp[0], temp[1], temp[2]);
		break;
	case TFA_IOCTL_SPK_ADDR:
		ret = copy_from_user(&last_addr, (void __user *)arg, sizeof(unsigned char));
		printk("tfa98xx_ioctl addr %x\n", last_addr);
		break;
	case TFA_IOCTL_SPK_MTP_BACKUP:
		pr_info("%s mtp backup %d.\n", __func__, check);
		break;
	default:
		printk("tfa98xx Fail IOCTL command no such ioctl cmd = %x\n", cmd);
		ret = -1;
		break;
	}

    return ret;
}

static long tfa98xx_debug_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	pr_info("%s...\n","%s");
	return tfa98xx_debug_shared_ioctl(file, cmd, (void __user *)arg);
}

#ifdef CONFIG_COMPAT
static long tfa98xx_compat_debug_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	pr_info("%s...\n","%s");
	ret = tfa98xx_debug_shared_ioctl(file,cmd,compat_ptr(arg));
	return ret;
}
#endif

static int tfa98xx_debug_open(
    struct inode *inode, struct file *file)
{
	printk("[TFA98xx]%s\n", __func__);
	return 0;
}

int tfa98xx_debug_release(
    struct inode *inode, struct file *file)
{
	printk("[TFA98xx]%s\n", __func__);
	return 0;
}

static const struct file_operations tfa98xx_debug_fileops = {
	.owner = THIS_MODULE,
	.open  = tfa98xx_debug_open,
	.read  = tfa98xx_debug_read,
	.write = tfa98xx_debug_write,
	.unlocked_ioctl = tfa98xx_debug_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl =   tfa98xx_compat_debug_ioctl,
#endif
	.release = tfa98xx_debug_release,
};

static struct miscdevice tfa98xx_debug_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AM_DEV_NAME,
	.fops = &tfa98xx_debug_fileops,
};



int tfa9894_debug_probe(struct i2c_client *client)
{

	int err = 0;

	printk("%s tfa98xx_debug_client (0x%x)\n", __func__, tfa98xx_debug_client);

	if (tfa98xx_debug_client)
		return 0;

	err = misc_register(&tfa98xx_debug_device);
	if (err) {
		printk("%s: tfa98xx_device register failed\n", __func__);
		return err;
	}

	tfa98xx_debug_client = client;

	return 0;
}

MODULE_DESCRIPTION("NXP TFA98xx debug driver");
MODULE_AUTHOR("chenjinquan <chenjinquan@vivo.com>");
MODULE_LICENSE("GPL");

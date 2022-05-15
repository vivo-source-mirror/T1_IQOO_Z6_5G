#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/miscdevice.h>
#include "vivo-audio-test.h"

#define TIMEOUT_MS 2000

#define AUDIO_TEST_IOC_MAGIC  'C'

#define AUDIO_TEST_IOCTL_TEST_START  _IOR(AUDIO_TEST_IOC_MAGIC, 0x01, int)
#define AUDIO_TEST_IOCTL_ADSP_RESTART  _IOR(AUDIO_TEST_IOC_MAGIC, 0x02, int)
#define AUDIO_TEST_IOCTL_ADSP_MI2S_CONFIG  _IOR(AUDIO_TEST_IOC_MAGIC, 0x03, int)
#define AUDIO_TEST_IOCTL_LOAD_DYN_MODULE  _IOR(AUDIO_TEST_IOC_MAGIC, 0x04, int)
#define AUDIO_TEST_IOCTL_APR_TEST  _IOW(AUDIO_TEST_IOC_MAGIC, 0x05, int)
#define AUDIO_TEST_IOCTL_TEST_STOP  _IOR(AUDIO_TEST_IOC_MAGIC, 0xFF, int)

enum {
	ADSP_RESTART = 0,
	MAX_AUDIO_TESTS
};

static int apr_inited = 0;
static int adsp_restart_count = 0;

static void *adsp_notifier = NULL;
static struct apr_ctl audio_test_apr[MAX_AUDIO_TESTS];

static int get_index_by_opcode(uint32_t opcode){
	int index = -1;

	switch (opcode) {
	case AFE_CMD_ADSP_RESTART_STRESS_TEST:
		index = ADSP_RESTART;
		break;
	default:
		index = -1;
		break;
	}
	return index;
}

static int vivo_afe_apr_send(void *data, uint32_t opcode)
{
	struct apr_ctl *this_apr = NULL;
	int ret;
	int index = get_index_by_opcode(opcode);

	if (!apr_inited) {
		pr_err("%s() apr not inited\n", __func__);
		return -EINVAL;
	}

	if (index >= 0 && index < MAX_AUDIO_TESTS) {
		this_apr = &audio_test_apr[index];
	}else{
		pr_err("%s() invaild index %d\n", __func__, index);
		return -EINVAL;
	}

	atomic_set(&this_apr->state, 1);
	atomic_set(&this_apr->status, 0);

	ret = vivo_afe_apr_send_pkt(data);
	if (!ret) {
		ret = wait_event_timeout(this_apr->wait,
				(atomic_read(&this_apr->state) == 0),
						msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			ret = -ETIMEDOUT;
			pr_err("%s: send apr time out\n", __func__);
		} else if (atomic_read(&this_apr->status) > 0) {
			pr_err("%s: DSP returned error[%s]\n", __func__,
				adsp_err_get_err_str(atomic_read(&this_apr->status)));
			ret = adsp_err_get_lnx_err_code(atomic_read(&this_apr->status));
		} else {
			ret = 0;
		}
	}
	pr_info("%s() exit %d\n", __func__, ret);
	return ret;
}

void vivo_apr_callback(uint32_t opcode, uint32_t status)
{
	struct apr_ctl *this_apr = NULL;
	int index = get_index_by_opcode(opcode);

	if (index < 0) {
		pr_err("%s() invaild opcode 0x%x\n", __func__, opcode);
		return;
	}
	this_apr = &audio_test_apr[index];
	atomic_set(&this_apr->state, 0);
	atomic_set(&this_apr->status, status);
	wake_up(&this_apr->wait);
	pr_info("%s() opcode 0x%x\n", __func__, opcode);

	return;
}

static void vivo_test_start(void)
{
	adsp_restart_count = 0;
	pr_info("%s() adsp_restart_count set to 0\n", __func__);
}

int vivo_adsp_restart(void)
{
	int ret = 0;
	struct apr_hdr hdr;

	hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
		APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	hdr.pkt_size = sizeof(hdr);
	hdr.src_port = 0;
	hdr.dest_port = 0;
	hdr.token = 0;
	hdr.opcode = AFE_CMD_ADSP_RESTART_STRESS_TEST;

	ret = vivo_afe_apr_send(&hdr, hdr.opcode);

	pr_info("%s() count %d\n", __func__, adsp_restart_count);
	return ret;
}
EXPORT_SYMBOL(vivo_adsp_restart);

struct clock_config mi2s_config =
{
   .enable = 0,
   .port_id = -EINVAL,
};
EXPORT_SYMBOL(mi2s_config);

static struct clock_config *get_mi2s_config(void)
{
	return &mi2s_config;
}

static int vivo_adsp_mi2s_config(void)
{
	struct clock_config* mi2s_clk = NULL;
	struct afe_clk_get cfg;
	int port_id, ret = -EINVAL;

	mi2s_clk = get_mi2s_config();
	if (!mi2s_clk || !mi2s_clk->enable) {
		pr_info("%s() no mi2s active now\n", __func__);
		return 0;
	}

	port_id = mi2s_clk->port_id;

	ret = afe_get_lpass_clk_cfg(port_id, &cfg);
	if (ret < 0) {
		pr_err("%s() get config errror(%d)\n", __func__, ret);
		return ret;
	} else
		pr_info("%s() port_id 0x%x, rate %u, channels %u, bits %u,"
			" enable %d\n", __func__, port_id, cfg.sample_rate,
			cfg.channels, cfg.bit_width, cfg.enable);

	ret = 0;
	if (mi2s_clk->sample_rate != cfg.sample_rate) {
		pr_err("%s() mismatch sample rate %u\n", __func__, mi2s_clk->sample_rate);
		ret = -EINVAL;
	}

	if (mi2s_clk->bit_width != cfg.bit_width) {
		pr_err("%s() mismatch bit_width %u\n", __func__, mi2s_clk->bit_width);
		ret = -EINVAL;
	}

	if (mi2s_clk->channels != cfg.channels) {
		pr_err("%s() mismatch channels %u\n", __func__, mi2s_clk->channels);
		ret = -EINVAL;
	}

	return ret;
}

static int vivo_adsp_load_dynamic_module(void){
	int ret = 0;
	ret = adm_load_dynamic_modules();
	if (ret)
		pr_err("%s() error(%d)\n", __func__, ret);
	else
		pr_info("%s() done\n", __func__);
	return ret;
}

int adsp_subsystem_restart(void)
{
	return vivo_adsp_restart();
}
EXPORT_SYMBOL(adsp_subsystem_restart);

static int vivo_adsp_callback(struct notifier_block *nb, unsigned long value,
			       void *priv)
{
	struct apr_ctl *this_apr = &audio_test_apr[ADSP_RESTART];

	if (value == SUBSYS_BEFORE_SHUTDOWN) {
		pr_info("%s() SUBSYS_BEFORE_SHUTDOWN\n", __func__);
	}

	if (value == SUBSYS_AFTER_POWERUP) {
		if (apr_inited && (atomic_read(&this_apr->state) != 0)){
			/* adsp will never response, so wake up the apr sender */
			adsp_restart_count++;
			atomic_set(&this_apr->state, 0);
			atomic_set(&this_apr->status, 0);
			wake_up(&this_apr->wait);
		}
		pr_info("%s() SUBSYS_AFTER_POWERUP\n", __func__);
	}

	return NOTIFY_OK;
}

static struct notifier_block vivo_adsp_notifier_block = {
	.notifier_call = vivo_adsp_callback,
	.priority = -INT_MAX,
};

static long  vivo_audio_test_ioctl (struct file *file,
	unsigned int cmd, unsigned long arg)
{
	int params[2] = {0};
	int result[2] = {0};
	int ret = -EINVAL;

	if(_IOC_TYPE(cmd) != AUDIO_TEST_IOC_MAGIC)
		return ret;

	switch (cmd) {
	case AUDIO_TEST_IOCTL_TEST_START:
		vivo_test_start();
		result[0] = 0;
		ret = copy_to_user((char __user *)arg, result, sizeof(result));
		break;
	case AUDIO_TEST_IOCTL_ADSP_RESTART:
		ret = vivo_adsp_restart();
		result[0] = ret;
		result[1] = adsp_restart_count;
		ret = copy_to_user((char __user *)arg, result, sizeof(result));
		break;
	case AUDIO_TEST_IOCTL_ADSP_MI2S_CONFIG:
		ret = vivo_adsp_mi2s_config();
		result[0] = ret;
		ret = copy_to_user((char __user *)arg, result, sizeof(result));
		break;
	case AUDIO_TEST_IOCTL_LOAD_DYN_MODULE:
		ret = vivo_adsp_load_dynamic_module();
		result[0] = ret;
		ret = copy_to_user((char __user *)arg, result, sizeof(result));
		break;
	case AUDIO_TEST_IOCTL_APR_TEST:
		ret = copy_from_user(params, (void __user *)arg, sizeof(params));
		/* @params[0] is one of the following:
			APR_ERROR_TEST = 0, APR_BLOCKED_TEST, APR_PCM_BUFFER_TEST,
		   @params[1] is the amount.
		*/
		audio_apr_test_set(1, params[0], params[1]);
	    break;
	case AUDIO_TEST_IOCTL_TEST_STOP:
		audio_apr_test_set(0, 0, 0);
		pr_info("%s() stop audio test\n", __func__);
		break;
	default:
		pr_err("%s() unsupport cmd\n", __func__);
		break;
	}
	return ret;
}

static const struct file_operations vivo_audio_test_ops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = vivo_audio_test_ioctl,
};

static struct miscdevice vivo_audio_test_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "audio_test",
	.fops = &vivo_audio_test_ops,
};

static int vivo_apr_init(void){
	struct apr_ctl *this_apr = audio_test_apr;
	int i = 0;

	if (apr_inited)
		return 0;

	for (i = 0; i < MAX_AUDIO_TESTS; i++){
		atomic_set(&this_apr[i].state, 0);
		atomic_set(&this_apr[i].status, 0);
		init_waitqueue_head(&this_apr[i].wait);
	}

	adsp_notifier = subsys_notif_register_notifier("adsp",
		&vivo_adsp_notifier_block);

	apr_inited = 1;

	pr_info("%s() done.\n", __func__);

	return 0;
}

int __init vivo_audio_test_init(void)
{
	int rc = 0;

	vivo_apr_init();

	rc = misc_register(&vivo_audio_test_device);
	if (rc)
        pr_err("%s: Failed to register vivo_audio_test device " 
               "for communications with userspace daemons; rc = [%d].\n",
               __func__, rc);

	return rc;
}

void vivo_audio_test_exit(void)
{
	misc_deregister(&vivo_audio_test_device);
}

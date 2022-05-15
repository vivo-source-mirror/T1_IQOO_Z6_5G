#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "vivo-codec-common.h"

int pa_value = 74; //tfa9874
struct vivo_codec_function * vivo_codec_fun = NULL;
static DEFINE_MUTEX(smartpa_lock);

void get_smartpa_lock(void)
{
	mutex_lock(&smartpa_lock);
	pr_info("vivo-codec-common;get_smartpa_lock()\n");
}
EXPORT_SYMBOL(get_smartpa_lock);

void release_smartpa_lock(void)
{
	mutex_unlock(&smartpa_lock);
	pr_info("vivo-codec-common;release_smartpa_lock()\n");
}
EXPORT_SYMBOL(release_smartpa_lock);

int get_pa_value(void)
{
	return pa_value;
}
EXPORT_SYMBOL(get_pa_value);

void set_pa_value(int value)
{
	pa_value = value;
}
EXPORT_SYMBOL(set_pa_value);

struct vivo_codec_function * get_vivo_codec_function(void)
{
	return vivo_codec_fun;
}
EXPORT_SYMBOL(get_vivo_codec_function);

void set_vivo_codec_function(struct vivo_codec_function *fun)
{
	vivo_codec_fun = fun;
}
EXPORT_SYMBOL(set_vivo_codec_function);

static int vivo_codec_common_probe(struct platform_device *pdev)
{
	struct vivo_codec_function *vivo_codec_function = NULL;

	dev_info(&pdev->dev, "%s() enter\n", __func__);

	vivo_codec_function = kzalloc(sizeof(struct vivo_codec_function), GFP_KERNEL);
	if (!vivo_codec_function) {
		dev_err(&pdev->dev,"%s:vivo_codec_function malloc failed\n", __func__);
		return -ENOMEM;
	}

	set_vivo_codec_function(vivo_codec_function);

	dev_info(&pdev->dev, "%s() vivo_codec_fun(%p)%p\n",
	         __func__, &vivo_codec_function, vivo_codec_function);

	/* dev_set_name(&pdev->dev, "%s", "vivo-codec-common"); */

	dev_info(&pdev->dev, "%s() leave\n", __func__);

	return 0;
}

static int vivo_codec_common_remove(struct platform_device *pdev)
{
	struct vivo_codec_function *fun = get_vivo_codec_function();
	if (fun) {
		kfree(fun);
		fun = NULL;
		set_vivo_codec_function(fun);
	}
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id vivo_codec_common_of_match[] = {
	{.compatible = "vivo,vivo-codec-common",},
	{},
};
#else
#define vivo_codec_common_of_match 0
#endif

static struct platform_driver vivo_codec_common_driver = {
	.probe = vivo_codec_common_probe,
	.remove = vivo_codec_common_remove,
	.driver = {
		.name = "vivo-codec-common",
		.owner = THIS_MODULE,
		.of_match_table = vivo_codec_common_of_match,
	},
};

module_platform_driver(vivo_codec_common_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhengguobing <zhengguobing@vivo.com>");
MODULE_DESCRIPTION(" vivo codec common driver");

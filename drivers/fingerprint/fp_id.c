#define pr_fmt(fmt)     "[FP_KERN] " KBUILD_MODNAME ": " fmt

#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include "fp_id.h"


static int fp_id = -1;
static int fp_gpio = -1;
static int fp_up = -1;
static int fp_down = -1;
static int fp_en = -1;
const char *fp_project_name;
#define MAX_TIMES		7

static const char * const pctl_names[] = {
	"fp_id_gpio_up",
	"fp_id_gpio_down",
};
struct pinctrl *fingerprint_pinctrl;
struct pinctrl_state *pinctrl_state[ARRAY_SIZE(pctl_names)];

#define DEVFS_MODE_RO (S_IRUSR|S_IRGRP|S_IROTH)
struct attribute fp_id_attr = {
	.name = "fp_id",
	.mode = DEVFS_MODE_RO,
};
static struct attribute *our_own_sys_attrs[] = {
	&fp_id_attr,
	NULL,
};

static int select_pin_ctl(struct device *dev, const char *name)
{
	size_t i;
	int rc;

	for (i = 0; i < ARRAY_SIZE(pinctrl_state); i++) {
		const char *n = pctl_names[i];
		if (!strncmp(n, name, strlen(n))) {
			rc = pinctrl_select_state(fingerprint_pinctrl, pinctrl_state[i]);
			if (rc)
				pr_err("bio_fp_error cannot select '%s'", name);
			else
				pr_debug("Selected '%s'", name);
			goto exit;
		}
	}
	rc = -EINVAL;
	pr_err("bio_fp_error %s:'%s' not found", __func__, name);
exit:
	return rc;
}

static ssize_t fp_id_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	char *fp_frame_id;

	if (fp_id == GOODIX_GF3626)
		fp_frame_id = "sidefp_goodix_3626";
	else if (fp_id == GOODIX_GF3636)
		fp_frame_id = "sidefp_goodix_3636";
	else if (fp_id == FPC_FPC1540)
		fp_frame_id = "sidefp_fpc_1540";
	else if (fp_id == GOODIX_GF9518)
		fp_frame_id = "udfp_goodix_gf9518";
	else if (fp_id == GOODIX_GF9518N)
		fp_frame_id = "udfp_goodix2_gf9518";
	else if (fp_id == SYNAPTICS_FS9501)
		fp_frame_id = "udfp_syna_fs9501";
	else if (fp_id == GOODIX_GF9608)
		fp_frame_id = "udfp_goodix_gf9608";
	else if (fp_id == GOODIX_GF9578)
		fp_frame_id = "udfp_goodix_gf9578";
	else
		fp_frame_id = "default";

	pr_info("get_fp_id=%d, fp_frame_id=%s, fp_up=%d, fp_down=%d.\n", get_fp_id(), fp_frame_id, fp_up, fp_down);
	/*Use of sprintf is deprecated: use snprintf instead.*/
	return snprintf(buf, strlen(fp_frame_id)+2, "%s\n", fp_frame_id);
}

static ssize_t fp_id_object_store(struct kobject *k, struct attribute *attr,
			      const char *buf, size_t count)
{
	/* nothing to do temply */
	pr_info("fp_id cannot be writed.\n");
	return SUCCESSFP;
}

int get_fp_id(void)
{
	pr_info("get_fp_id , fp_id=%d, fp_up=%d, fp_down=%d\n", fp_id, fp_up, fp_down);
	return fp_id;
}
EXPORT_SYMBOL(get_fp_id);

static void fp_id_object_release(struct kobject *kobj)
{
	/* nothing to do temply */
	return;
}

static const struct sysfs_ops fp_id_object_sysfs_ops = {
	.show = fp_id_object_show,
	.store = fp_id_object_store,
};
static struct kobj_type fp_id_object_type = {
	.sysfs_ops	= &fp_id_object_sysfs_ops,
	.release	= fp_id_object_release,
	.default_attrs = our_own_sys_attrs,
};

struct kobject kobj;

static int
fp_id_probe(struct platform_device *pdev)
{

	int ret;
	int i;

	ret = kobject_init_and_add(&kobj, &fp_id_object_type, NULL, "fp_id");
	if (ret) {
		pr_err("%s: Create fp_id error!\n", __func__);
		return -ERRORFP;
	}

	ret = of_property_read_string(pdev->dev.of_node, "vivo,project-name", &fp_project_name);
	if (ret) {
		pr_err("%s:vivo,project-name property do not find\n", __func__);
		fp_project_name = "default";
	}
	pr_info("%s:vivo,project-name is %s\n", __func__, fp_project_name);

	if ((!strncmp(fp_project_name, "TD2105", 6))) {
		fp_id = GOODIX_GF9578;
		pr_info("%s: return GOODIX_GF9578\n", __func__);
		return SUCCESSFP;
	}

	if ((!strncmp(fp_project_name, "PD2165", 6))) {
		fp_en = of_get_named_gpio(pdev->dev.of_node, "fpc,gpio_vdd_en", 0);
			if (fp_en < 0) {
				pr_err("%s: get fpc vdd gpio failed!\n", __func__);
				return -ERRORFP;
			}
		ret = devm_gpio_request(&pdev->dev, fp_en, "fpc,gpio_vdd_en");
		if (ret) {
			pr_err("%s: request fpc vdd gpio failed!\n", __func__);
			return -ERRORFP;
		}
		ret = gpio_direction_output(fp_en, 1);
		if (ret) {
			pr_err("%s: power on failed!\n", __func__);
			return -ERRORFP;
		}
		mdelay(5);
	}
	pr_info("%s:fpc vdd gpio: %d \n", __func__, fp_en);

	fp_gpio = of_get_named_gpio(pdev->dev.of_node, "fp_id,gpios", 0);
	if (fp_gpio < 0) {
		pr_err("%s: get fp_id gpio failed!\n", __func__);
		return -ERRORFP;
	}
	pr_info("%s:fp gpio: %d \n", __func__, fp_gpio);

	ret = devm_gpio_request(&pdev->dev, fp_gpio, "fp_id,gpios");
	if (ret) {
		pr_err("%s: request fp_id gpio failed!\n", __func__);
		return -ERRORFP;
	}

	fingerprint_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(fingerprint_pinctrl)) {
		if (PTR_ERR(fingerprint_pinctrl) == -EPROBE_DEFER) {
			pr_err("bio_fp_error %s: pinctrl not ready!", __func__);
			return -ERRORFP;
		}
		pr_err("%s: Target does not use pinctrl\n", __func__);
		fingerprint_pinctrl = NULL;
		return -ERRORFP;
	}

	for (i = 0; i < ARRAY_SIZE(pinctrl_state); i++) {
		const char *n = pctl_names[i];
		struct pinctrl_state *state =
			pinctrl_lookup_state(fingerprint_pinctrl, n);
		if (IS_ERR(state)) {
			pr_err("%s: cannot find '%s'\n", __func__, n);
			return -ERRORFP;
		}
		pr_err("%s: found pin control %s\n", __func__, n);
		pinctrl_state[i] = state;
	}

	ret = select_pin_ctl(&pdev->dev, "fp_id_gpio_up");
	mdelay(5);
	if (ret)
		return -ERRORFP;
	fp_up = gpio_get_value(fp_gpio);
	printk("%s: set fp-id pull up,get gpio value = %d\n", __func__, fp_up);
	ret = select_pin_ctl(&pdev->dev, "fp_id_gpio_down");
	mdelay(5);
	if (ret)
		return -ERRORFP;
	fp_down = gpio_get_value(fp_gpio);
	printk("%s: set fp-id pull down,get gpio value = %d\n", __func__, fp_down);
	if ((!strncmp(fp_project_name, "PD2165", 6))) {
		if ((fp_up == 0) && (fp_down == 0)) {
			fp_id = GOODIX_GF3626;
			ret = select_pin_ctl(&pdev->dev, "fp_id_gpio_down");
			printk("return GOODIX_GF3626: 0x%x\n", fp_id);
			if (ret) {
				printk("%s: set fp-id pull down error,get gpio value = %d\n", __func__, gpio_get_value(fp_gpio));
				return -ERRORFP;
			}
		} else {
			fp_id = FPC_FPC1540;
			ret = select_pin_ctl(&pdev->dev, "fp_id_gpio_down");
			printk("return FPC_FPC1540: 0x%x\n", fp_id);
			if (ret) {
				printk("%s: set fp-id pull down error,get gpio value = %d\n", __func__, gpio_get_value(fp_gpio));
				return -ERRORFP;
			}
		}
	}
	if ((!strncmp(fp_project_name, "PD2165", 6))) {
		ret = gpio_direction_output(fp_en, 0);
		if (ret) {
			pr_err("%s: power off failed!\n", __func__);
			return -ERRORFP;
		}
		gpio_free(fp_en);
		mdelay(10);
	}
	return SUCCESSFP;
}

static int fp_id_remove(struct platform_device *pdev)
{
	pr_info("fp_id  remove.\n");
	kobject_del(&kobj);
	return SUCCESSFP;
}

static struct of_device_id fp_id_match_table[] = {
	{ .compatible = "fp-id",},
	{},
};

static struct platform_driver fp_id_driver = {
	.probe      = fp_id_probe,
	.remove     = fp_id_remove,
	.driver = {
		.name   = "fp_id",
		.owner  = THIS_MODULE,
		.of_match_table = fp_id_match_table,
	},
};

static int __init fp_id_init(void)
{
    return platform_driver_register(&fp_id_driver);
}
module_init(fp_id_init);

static void __exit fp_id_exit(void)
{
    platform_driver_unregister(&fp_id_driver);

}
module_exit(fp_id_exit);

MODULE_AUTHOR("Xiaot BBK Corporation");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
